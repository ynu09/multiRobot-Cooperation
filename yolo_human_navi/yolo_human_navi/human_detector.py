import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np
import threading
from queue import Queue
import time

from multitb_interfaces.srv import HumanDetection


class ImageProcessor(threading.Thread):
    def __init__(self, robot_id, model, detection_pubs, image_pub, popup_client):
        super().__init__()
        self.robot_id = robot_id
        self.model = model
        self.bridge = CvBridge()
        self.image_queue = Queue(maxsize=1)
        self.result_queue = Queue(
            maxsize=1
        )  # 처리된 이미지를 메인 스레드로 전달하기 위한 큐

        self.detection_pubs = detection_pubs  # yolo 퍼블리셔
        self.image_pub = image_pub  # flask에 이미지 publish
        self.popup_client = popup_client  # 팝업 서비스 클라이언트

        self.running = True

    def add_image(self, image_msg):

        if self.image_queue.full():
            try:
                self.image_queue.get_nowait()
            except:
                pass
        self.image_queue.put(image_msg)

    def get_processed_image(self):
        if self.result_queue.empty():
            return None
        return self.result_queue.get()

    def stop(self):
        self.running = False

    def run(self):
        print("yolo start")
        while self.running:
            try:
                image_msg = self.image_queue.get(timeout=0.1)
                print(f"[{self.robot_id}] Received image for processing")

                # ROS 이미지 -> OpenCV 이미지
                np_arr = np.frombuffer(image_msg.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                if cv_image is None or cv_image.size == 0:
                    print(f"[{self.robot_id}] Empty or invalid image")
                    continue

                # YOLO로 객체 감지
                results = self.model(cv_image)

                found_human = False

                # 사람 감지 처리
                for result in results[0].boxes.data.cpu().numpy():
                    cls_index = int(result[5])
                    cls_name = self.model.names[cls_index]
                    conf = result[4]

                    if cls_index == 0 and conf > 0.5:
                        found_human = True
                        x1, y1, x2, y2 = map(int, result[:4])

                        # 시각화 박스
                        cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
                        cv2.putText(
                            cv_image,
                            f"{cls_name}: {conf:.2f}",
                            (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1,
                            (255, 0, 0),
                            2,
                        )

                # yolo 감지 시, publish
                if found_human:
                    print("found human")
                    msg = String()
                    msg.data = "Human detected!"
                    self.detection_pubs.publish(msg)

                    # 팝업 서비스 호출
                    self.call_popup_service()

                # flask에 처리된 이미지를 CompressedImage로 변환하여 publish
                _, img_encoded = cv2.imencode(".jpg", cv_image)
                msg = CompressedImage()
                msg.format = "jpeg"
                msg.data = np.array(img_encoded).tobytes()
                self.image_pub.publish(msg)
                print(f"[{self.robot_id}] Published processed image")

                # 처리된 이미지를 결과 큐에 저장
                if self.result_queue.full():
                    try:
                        self.result_queue.get_nowait()
                    except:
                        pass
                self.result_queue.put(cv_image)

            except Exception as e:
                # print(f"[{self.robot_id}] Error processing image: {str(e)}")
                continue

    def call_popup_service(self):
        request = HumanDetection.Request()
        request.robot_id = self.robot_id

        try:
            # 서비스 클라이언트가 준비될 때까지 대기
            if not self.popup_client.wait_for_service(timeout_sec=1.0):
                print(f"Popup service not available for {self.robot_id}")
                return

            future = self.popup_client.call_async(request)
            node = self.popup_client._node
            rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)

            if future.result() is not None:
                response = future.result()
                if response.success:
                    print(f"Popup service call successful for {self.robot_id}")
                else:
                    print(f"Popup already displayed for {self.robot_id}")
            else:
                print(f"Service call failed for {self.robot_id}")
        except Exception as e:
            print(f"Error calling popup service: {str(e)}")


class HumanDetector(Node):
    def __init__(self):
        super().__init__("human_detector_node")

        # YOLOv8 모델 로드
        self.model = YOLO("yolo11s.pt")

        # [Topic Publisher] yolo 감지 (내비용)
        self.detection_pubs = {
            "tb1": self.create_publisher(String, "/tb1/human_detection", 10),
            "tb2": self.create_publisher(String, "/tb2/human_detection", 10),
            "tb3": self.create_publisher(String, "/tb3/human_detection", 10),
            "dr1f": self.create_publisher(String, "/dr1f/human_detection", 10),
            "dr1b": self.create_publisher(String, "/dr1b/human_detection", 10),
            # 'tb4': self.create_publisher(String, '/tb4/human_detection', 1),
        }

        # [Service Client] 팝업 서비스
        self.popup_clients = {
            "tb1": self.create_client(HumanDetection, "/tb1/popup_service"),
            "tb2": self.create_client(HumanDetection, "/tb2/popup_service"),
            "tb3": self.create_client(HumanDetection, "/tb3/popup_service"),
            "dr1f": self.create_client(HumanDetection, "/dr1f/popup_service"),
            "dr1b": self.create_client(HumanDetection, "/dr1b/popup_service"),
        }

        # [Topic Publisher] Flask에 publish
        self.image_pubs = {
            "tb1": self.create_publisher(
                CompressedImage, "/tb1/processed_image/compressed", 10
            ),
            "tb2": self.create_publisher(
                CompressedImage, "/tb2/processed_image/compressed", 10
            ),
            "tb3": self.create_publisher(
                CompressedImage, "/tb3/processed_image/compressed", 10
            ),
            "dr1f": self.create_publisher(
                CompressedImage, "/dr1/front/processed_image/compressed", 10
            ),
            "dr1b": self.create_publisher(
                CompressedImage, "/dr1/bottom/processed_image/compressed", 10
            ),
        }

        # 이미지 처리 스레드 생성
        self.processors = {
            "tb1": ImageProcessor(
                "tb1",
                self.model,
                self.detection_pubs["tb1"],
                self.image_pubs["tb1"],
                self.popup_clients["tb1"],
            ),
            "tb2": ImageProcessor(
                "tb2",
                self.model,
                self.detection_pubs["tb2"],
                self.image_pubs["tb2"],
                self.popup_clients["tb2"],
            ),
            "tb3": ImageProcessor(
                "tb3",
                self.model,
                self.detection_pubs["tb3"],
                self.image_pubs["tb3"],
                self.popup_clients["tb3"],
            ),
            "dr1f": ImageProcessor(
                "dr1f",
                self.model,
                self.detection_pubs["dr1f"],
                self.image_pubs["dr1f"],
                self.popup_clients["dr1f"],
            ),
            "dr1b": ImageProcessor(
                "dr1b",
                self.model,
                self.detection_pubs["dr1b"],
                self.image_pubs["dr1b"],
                self.popup_clients["dr1b"],
            ),
            # 'tb4': ImageProcessor('tb4', self.model, self.detection_pubs['tb4']),
        }

        # # 윈도우 생성 (창 크기 조절할 수 있게 처리)
        # cv2.namedWindow("tb1 Human Detection", cv2.WINDOW_NORMAL)
        # cv2.namedWindow("tb2 Human Detection", cv2.WINDOW_NORMAL)
        # cv2.namedWindow("tb3 Human Detection", cv2.WINDOW_NORMAL)
        # # cv2.namedWindow('tb4 Human Detection', cv2.WINDOW_NORMAL)

        # 스레드 시작
        for processor in self.processors.values():
            processor.start()

        # 카메라 구독
        self.create_subscription(
            CompressedImage,
            "/tb1/camera/image_raw/compressed",
            lambda msg: self.image_callback(msg, "tb1"),
            10,
        )

        self.create_subscription(
            CompressedImage,
            "/tb2/camera/image_raw/compressed",
            lambda msg: self.image_callback(msg, "tb2"),
            10,
        )

        self.create_subscription(
            CompressedImage,
            "/tb3/camera/image_raw/compressed",
            lambda msg: self.image_callback(msg, "tb3"),
            10,
        )
        self.create_subscription(
            CompressedImage,
            "/dr1/front/image_raw/compressed",
            lambda msg: self.image_callback(msg, "dr1f"),
            10,
        )
        self.create_subscription(
            CompressedImage,
            "/dr1/bottom/image_raw/compressed",
            lambda msg: self.image_callback(msg, "dr1b"),
            10,
        )

        # self.create_subscription(
        #     CompressedImage,
        #     '/tb4/camera/image_raw/compressed',
        #     lambda msg: self.image_callback(msg, 'tb4'),
        #     10
        # )

        # # GUI 업데이트를 위한 타이머 (30Hz)
        # self.create_timer(1 / 30, self.update_gui)

    def image_callback(self, msg, robot_id):
        self.processors[robot_id].add_image(msg)

        # def update_gui(self):
        #     # 각 프로세서의 결과 이미지를 가져와서 화면에 표시
        #     for robot_id, processor in self.processors.items():
        #         processed_image = processor.get_processed_image()
        #         if processed_image is not None:
        #             cv2.imshow(f"{robot_id} Human Detection", processed_image)
        # cv2.waitKey(1)

    def destroy_node(self):
        # 스레드 정리
        for processor in self.processors.values():
            processor.stop()

        for processor in self.processors.values():
            processor.join()

        # cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HumanDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
