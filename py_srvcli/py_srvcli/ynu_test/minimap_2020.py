from flask import Flask, jsonify, request, session, redirect, url_for, render_template
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from rclpy.qos import QoSDurabilityPolicy, ReliabilityPolicy, HistoryPolicy, QoSProfile
import json
import cv2
from cv_bridge import CvBridge
import base64
import hashlib
import sqlite3
import numpy as np

from pack.srv import FoundPerson

from rclpy.executors import MultiThreadedExecutor
import threading


# Flask 애플리케이션 초기화 및 템플릿 경로 설정
app = Flask(
    __name__,
    template_folder="/home/idingg/multitb_ws/src/py_srvcli/py_srvcli/templates",
    static_folder="/home/idingg/multitb_ws/src/py_srvcli/py_srvcli/static",
)
app.secret_key = "secret_key"  # 세션 관리에 사용할 비밀 키

# 전역 변수 선언
latest_messages = []  # 최신 메시지를 저장하는 리스트
latest_image = None  # 첫 번째 카메라 이미지를 저장
latest_image_2 = None  # 두 번째 카메라 이미지를 저장
bridge = CvBridge()  # ROS 이미지와 OpenCV 이미지를 변환하는 도구


# [Service] ROS 2 노드 정의
class Operator(Node):
    def __init__(self):
        super().__init__("operator")
        self.service_client = self.create_client(FoundPerson, "found_person_service")
        while not self.service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("서비스를 기다리는 중입니다...")

        self.future = None
        self.send_request()

    def send_request(self):
        """서비스 요청 전송"""
        if self.future is None or self.future.done():
            request = FoundPerson.Request()
            request.request = "Found Missing Person?"  # 요청 메시지
            self.future = self.service_client.call_async(request)

    def process_response(self):
        """서비스 응답 처리"""
        if self.future and self.future.done():
            try:
                response = self.future.result()
                if response.response:
                    self.get_logger().info(f"Received response: {response.response}")
                    try:
                        conn = sqlite3.connect("db.db")
                        cursor = conn.cursor()
                        query = "INSERT INTO log (time, message) VALUES (datetime('now'), ?)"
                        cursor.execute(query, (response.response,))
                        conn.commit()
                        conn.close()
                    except sqlite3.Error as e:
                        self.get_logger().error(f"Failed to save log to database: {e}")
                self.future = None
                self.send_request()
            except Exception as e:
                self.get_logger().error(f"Service call failed: {e}")
                self.future = None
                self.send_request()


# ROS 2 Subscriber 노드 정의
class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__("map_subscriber")

        # QoS 설정
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # 첫 번째 카메라 토픽 구독 (압축 이미지)
        self.image_subscription = self.create_subscription(
            CompressedImage,
            "/tb1/processed_image/compressed",
            self.image_callback,
            qos_profile,
        )
        self.bridge = CvBridge()

        # 두 번째 카메라 토픽 구독 (압축 이미지)
        self.image_subscription_2 = self.create_subscription(
            CompressedImage, "/drone/cam", self.image_callback_2, qos_profile
        )

        self.string_subscription = self.create_subscription(
            String, "/find/p", self.string_callback, qos_profile
        )

    def string_callback(self, msg):
        """/find/p 토픽에서 받은 메시지 처리"""
        global latest_messages
        latest_messages.append(msg.data)  # 메시지를 리스트에 저장

    def string_callback(self, msg):
        """문자열 메시지 처리 콜백"""
        global latest_messages
        try:
            # JSON 파싱 시도
            data = json.loads(msg.data)
            self.get_logger().info(f"Received JSON message: {data}")
            latest_messages.append(data)  # JSON 메시지를 리스트에 추가
        except json.JSONDecodeError:
            # JSON이 아닐 경우 일반 문자열로 처리
            self.get_logger().info(f"Received non-JSON message: {msg.data}")
            latest_messages.append(msg.data)  # 일반 문자열 메시지를 리스트에 추가

    def image_callback(self, msg):
        """첫 번째 카메라 이미지 처리 콜백"""
        global latest_image
        try:
            print("Received image in minimap subscriber")
            # 압축된 이미지 데이터를 numpy 배열로 변환
            np_arr = np.frombuffer(msg.data, np.uint8)

            # numpy 배열을 디코딩하여 OpenCV 이미지로 변환
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # JPEG 형식으로 이미지를 인코딩하고 Base64로 변환
            _, buffer = cv2.imencode(".jpg", img)
            latest_image = base64.b64encode(buffer).decode("utf-8")
            print("Successfully processed and stored image")
        except Exception as e:
            print(f"Error in image_callback: {str(e)}")
            self.get_logger().error(f"Failed to process image: {e}")

    def image_callback_2(self, msg):
        """두 번째 카메라 이미지 처리 콜백"""
        global latest_image_2
        try:
            # 압축된 이미지 데이터를 numpy 배열로 변환
            np_arr = np.frombuffer(msg.data, np.uint8)

            # numpy 배열을 디코딩하여 OpenCV 이미지로 변환
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # JPEG 형식으로 이미지를 인코딩하고 Base64로 변환
            _, buffer = cv2.imencode(".jpg", img)
            latest_image_2 = base64.b64encode(buffer).decode("utf-8")
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")


# Flask 라우트 정의
last_checked_time = None  # 마지막으로 확인된 메시지의 시간 저장


@app.route("/check_new_message")
def check_new_message():
    """DB에서 새로 추가된 'Found Missing Person!' 메시지를 확인하고 반환"""
    global last_checked_time

    if not session.get("logged_in"):
        return jsonify({"message": None}), 403

    try:
        conn = sqlite3.connect("db.db")
        cursor = conn.cursor()

        # 새로 추가된 메시지 확인 (시간 기준)
        if last_checked_time:
            query = """
            SELECT time, message FROM log
            WHERE message = 'Found Missing Person!' AND time > ?
            ORDER BY time ASC LIMIT 1
            """
            cursor.execute(query, (last_checked_time,))
        else:
            # 처음 실행 시 가장 최신 메시지 확인
            query = """
            SELECT time, message FROM log
            WHERE message = 'Found Missing Person!'
            ORDER BY time DESC LIMIT 1
            """
            cursor.execute(query)

        result = cursor.fetchone()
        conn.close()

        # 새 메시지가 있으면 반환하고 시간 업데이트
        if result:
            last_checked_time = result[0]  # 가장 최근 시간 업데이트
            return jsonify({"message": result[1]})
        else:
            return jsonify({"message": None})
    except sqlite3.Error as e:
        return jsonify({"error": f"Database error: {e}"}), 500


@app.route("/popup_message")
def get_popup_message():
    """팝업 메시지를 반환하는 Flask 엔드포인트"""
    if not session.get("logged_in"):
        return "Unauthorized", 403

    if latest_messages:
        return jsonify({"message": latest_messages.pop(0)})
    return jsonify({"message": ""})


@app.route("/")
def home():
    """홈 페이지"""
    if not session.get("logged_in"):  # 로그인 여부 확인
        return redirect(url_for("login"))  # 로그인 페이지로 리디렉션
    return render_template("home.html")


@app.route("/login", methods=["GET", "POST"])
def login():
    """로그인 처리"""
    if request.method == "POST":  # 로그인 양식 제출
        username = request.form["username"]
        password = hashlib.sha256(
            request.form["password"].encode("utf-8")
        ).hexdigest()  # 비밀번호 해시화

        try:
            # SQLite 데이터베이스 연결 및 사용자 인증
            conn = sqlite3.connect("db.db")
            cursor = conn.cursor()
            query = "SELECT * FROM users WHERE user_id = ? AND user_pw = ?"
            cursor.execute(query, (username, password))
            result = cursor.fetchone()
            conn.close()

            if result:  # 인증 성공
                session["logged_in"] = True
                return redirect(url_for("home"))
            else:  # 인증 실패
                return "Invalid credentials", 401
        except sqlite3.Error as e:  # 데이터베이스 오류 처리
            return f"Database error: {e}", 500

    return render_template("login.html")  # 로그인 페이지 렌더링


@app.route("/logout")
def logout():
    """로그아웃 처리"""
    session.pop("logged_in", None)  # 세션 초기화
    return redirect(url_for("login"))  # 로그인 페이지로 리디렉션


@app.route("/data")
def get_data():
    if not session.get("logged_in"):
        return "Unauthorized", 403
    try:
        conn = sqlite3.connect("db.db")
        cursor = conn.cursor()
        query = "SELECT x, y, name FROM robot"
        cursor.execute(query)
        rows = cursor.fetchall()
        conn.close()
        data = [{"id": row[2], "x": row[0] * 600, "y": row[1] * 400} for row in rows]
        return jsonify(data)
    except sqlite3.Error as e:
        return f"Database error: {e}", 500


@app.route("/image")
def get_image():
    """첫 번째 카메라 이미지 반환 API"""
    if not session.get("logged_in"):
        return "Unauthorized", 403
    return jsonify({"image": latest_image})  # Base64 인코딩된 이미지 반환


@app.route("/image2")
def get_image_2():
    """두 번째 카메라 이미지 반환 API"""
    if not session.get("logged_in"):
        return "Unauthorized", 403
    return jsonify({"image": latest_image_2})  # Base64 인코딩된 이미지 반환


@app.route("/logs")
def get_logs():
    """DB에서 로그 데이터를 가져오는 API"""
    if not session.get("logged_in"):
        return "Unauthorized", 403
    try:
        # SQLite 데이터베이스에서 로그 가져오기
        conn = sqlite3.connect("db.db")
        cursor = conn.cursor()
        query = "SELECT time, message FROM log ORDER BY time DESC LIMIT 10"  # 최신 10개 로그 가져오기
        cursor.execute(query)
        logs = [{"time": row[0], "message": row[1]} for row in cursor.fetchall()]
        conn.close()
        return jsonify(logs)  # JSON 형식으로 반환
    except sqlite3.Error as e:
        return f"Database error: {e}", 500


# def ros2_spin():
#     """ROS 2 Subscriber 노드 실행"""
#     rclpy.init()
#     node = MinimalSubscriber()
#     try:
#         while rclpy.ok():  # ROS 2가 활성 상태인지 확인
#             rclpy.spin_once(node, timeout_sec=1.0)
#     except Exception as e:
#         print(f"ROS 2 spin error: {e}")
#     finally:
#         if rclpy.ok():  # 아직 활성 상태라면 종료 처리
#             node.destroy_node()  # 노드 종료
#             rclpy.shutdown()  # ROS 2 종료

# def start_ros_node():
#     rclpy.init()
#     operator_node = Operator()
#     try:
#         while rclpy.ok():
#             rclpy.spin_once(operator_node, timeout_sec=0.1)
#             operator_node.process_response()
#     except KeyboardInterrupt:
#         pass
#     finally:
#         operator_node.destroy_node()
#         rclpy.shutdown()


def run_ros_nodes():
    """ROS 2 Subscriber와 Operator 노드를 멀티스레드로 실행"""
    rclpy.init()

    # 노드 초기화
    subscriber_node = MinimalSubscriber()
    operator_node = Operator()

    # MultiThreadedExecutor 생성
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(subscriber_node)
    executor.add_node(operator_node)

    # 노드 상태 모니터링을 위한 함수
    def monitor_nodes():
        while rclpy.ok():
            time.sleep(1)  # 1초마다 체크
            print("Nodes status:")
            print(f"Subscriber node: {subscriber_node.get_name()} - Active")
            print(f"Operator node: {operator_node.get_name()} - Active")
            print(f"Latest image available: {latest_image is not None}")

    # executor를 별도 스레드에서 실행
    executor_thread = threading.Thread(target=lambda: executor.spin(), daemon=True)
    executor_thread.start()

    # 모니터링 스레드 시작
    monitor_thread = threading.Thread(target=monitor_nodes, daemon=True)
    monitor_thread.start()

    try:
        # operator 응답 처리를 별도 스레드에서 수행
        while rclpy.ok():
            operator_node.process_response()
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("프로그램이 중단되었습니다.")
    except Exception as e:
        print(f"ROS 2 실행 중 에러 발생: {e}")
    finally:
        print("Shutting down ROS 2 nodes...")
        executor.shutdown()
        subscriber_node.destroy_node()
        operator_node.destroy_node()
        rclpy.shutdown()


def main():
    """메인 함수: ROS 2 및 Flask 서버 실행"""
    # ROS2 노드들을 별도 스레드에서 실행
    threading.Thread(target=run_ros_nodes, daemon=True).start()

    # Flask 서버 실행
    app.run(debug=True, host="0.0.0.0", port=5000)


if __name__ == "__main__":
    main()
