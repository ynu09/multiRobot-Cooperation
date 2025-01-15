from flask import Flask, jsonify, request, session, redirect, url_for, render_template
import threading
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
import base64
import hashlib
import sqlite3
import numpy as np
from queue import Queue

from multitb_interfaces.srv import HumanDetection

# Flask 애플리케이션 초기화 및 템플릿 경로 설정
app = Flask(
    __name__,
    template_folder="/home/ynu/Rokey/project7/prj7_git/src/py_srvcli/py_srvcli/templates",
    static_folder="/home/ynu/Rokey/project7/prj7_git/src/py_srvcli/py_srvcli/static",
)
app.secret_key = "secret_key"  # 세션 관리에 사용할 비밀 키

# 각 robot_id에 대해 팝업이 띄워졌는지 여부를 추적할 딕셔너리
popup_displayed = {"tb1": False, "tb2": False, "tb3": False, "dr1": False}

# 전역 변수 선언
latest_messages = []  # 최신 메시지를 저장하는 리스트
latest_image = None  # 첫 번째 카메라 이미지를 저장
latest_image_2 = None  # 두 번째 카메라 이미지를 저장
latest_image_3 = None
bridge = CvBridge()


class CamSubscriber(Node):
    def __init__(self):
        super().__init__("map_subscriber")
        self.bridge = CvBridge()
        self.image_queue = Queue(maxsize=10)
        self.lock = threading.Lock()

        # 이미지 저장을 위한 딕셔너리
        self.latest_images = {
            "tb1": None,
            "tb2": None,
            "tb3": None,
            "dr1f": None,
            "dr1b": None,
        }

        # 스레드 안전한 메시지 큐
        self.message_queue = Queue(maxsize=100)

        # [Topic Subscription] 터틀봇1 카메라
        self.create_subscription(
            CompressedImage,
            "/tb1/processed_image/compressed",
            lambda msg: self.image_callback(msg, "tb1"),
            10,
        )

        # [Topic Subscription] 터틀봇2 카메라
        self.create_subscription(
            CompressedImage,
            "/tb2/processed_image/compressed",
            lambda msg: self.image_callback(msg, "tb2"),
            10,
        )

        # [Topic Subscription] 터틀봇3 카메라
        self.create_subscription(
            CompressedImage,
            "/tb3/processed_image/compressed",
            lambda msg: self.image_callback(msg, "tb3"),
            10,
        )

        # [Topic Subscription] 드론1 전방 카메라
        self.create_subscription(
            CompressedImage,
            "/dr1/front/processed_image/compressed",
            lambda msg: self.image_callback(msg, "dr1f"),
            10,
        )

        # [Topic Subscription] 드론1 하방 카메라
        self.create_subscription(
            CompressedImage,
            "/dr1/bottom/processed_image/compressed",
            lambda msg: self.image_callback(msg, "dr1b"),
            10,
        )

        # [Service Server] 팝업 서비스
        self.popup_services = {
            "tb1": self.create_service(
                HumanDetection,
                "/tb1/popup_service",
                lambda req, resp: self.handle_popup_service(req, resp, "tb1"),
            ),
            "tb2": self.create_service(
                HumanDetection,
                "/tb2/popup_service",
                lambda req, resp: self.handle_popup_service(req, resp, "tb2"),
            ),
            "tb3": self.create_service(
                HumanDetection,
                "/tb3/popup_service",
                lambda req, resp: self.handle_popup_service(req, resp, "tb3"),
            ),
            "dr1f": self.create_service(
                HumanDetection,
                "/dr1f/popup_service",
                lambda req, resp: self.handle_popup_service(req, resp, "dr1f"),
            ),
            "dr1b": self.create_service(
                HumanDetection,
                "/dr1b/popup_service",
                lambda req, resp: self.handle_popup_service(req, resp, "dr1b"),
            ),
        }

    # [Topic Callback] 카메라 이미지 처리
    def image_callback(self, msg, robot_id):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if img is not None:
                _, buffer = cv2.imencode(".jpg", img)
                b64_image = base64.b64encode(buffer).decode("utf-8")

                with self.lock:
                    self.latest_images[robot_id] = b64_image
        except Exception as e:
            self.get_logger().error(f"Image processing error for {robot_id}: {str(e)}")

    def get_latest_image(self, robot_id):
        with self.lock:
            return self.latest_images.get(robot_id)

    # [Service Callback] 팝업 서비스 요청 처리
    def handle_popup_service(self, request, response, robot_id):
        global popup_displayed  # 팝업 상태를 추적하는 전역 변수

        # 이미 팝업이 표시된 상태라면 추가 요청을 무시
        if popup_displayed.get(robot_id, False):
            self.get_logger().info(
                f"Popup already displayed for {robot_id}. Ignoring request."
            )
            response.success = False
            return response

        try:
            # 새로운 요청 처리
            self.message_queue.put_nowait(f"{robot_id}: Human detected!")
            popup_displayed[robot_id] = True  # 팝업 표시 상태 업데이트
            self.get_logger().info(f"Popup displayed for {robot_id}.")
            response.success = True
        except Queue.Full:
            self.get_logger().warning("Message queue full")
            response.success = False
        return response


# Flask 라우트 정의
last_checked_time = None  # 마지막으로 확인된 메시지의 시간 저장


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
            conn = sqlite3.connect("src/db.db")
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


# 좌표변환
@app.route("/data")
def get_data():
    if not session.get("logged_in"):
        return "Unauthorized", 403
    try:
        conn = sqlite3.connect("src/db.db")
        cursor = conn.cursor()
        query = "SELECT x, y, name FROM robot"
        cursor.execute(query)
        rows = cursor.fetchall()
        conn.close()
        data = [{"id": row[2], "x": row[0], "y": row[1]} for row in rows]
        return jsonify(data)
    except sqlite3.Error as e:
        return f"Database error: {e}", 500


# Flask 라우트 수정
@app.route("/image")
def get_image():
    if not session.get("logged_in"):
        return "Unauthorized", 403

    if not hasattr(app, "ros_node"):
        return jsonify({"error": "ROS node not initialized"})

    image = app.ros_node.get_latest_image("tb1")
    return jsonify({"image": image})


@app.route("/image2")
def get_image_2():
    if not session.get("logged_in"):
        return "Unauthorized", 403

    if not hasattr(app, "ros_node"):
        return jsonify({"error": "ROS node not initialized"})

    image = app.ros_node.get_latest_image("tb2")
    return jsonify({"image": image})


@app.route("/image3")
def get_image_3():
    if not session.get("logged_in"):
        return "Unauthorized", 403

    if not hasattr(app, "ros_node"):
        return jsonify({"error": "ROS node not initialized"})

    image = app.ros_node.get_latest_image("tb3")
    return jsonify({"image": image})


@app.route("/image4")
def get_image_4():
    if not session.get("logged_in"):
        return "Unauthorized", 403

    if not hasattr(app, "ros_node"):
        return jsonify({"error": "ROS node not initialized"})

    image = app.ros_node.get_latest_image("dr1f")
    return jsonify({"image": image})


@app.route("/image5")
def get_image_5():
    if not session.get("logged_in"):
        return "Unauthorized", 403

    if not hasattr(app, "ros_node"):
        return jsonify({"error": "ROS node not initialized"})

    image = app.ros_node.get_latest_image("dr1b")
    return jsonify({"image": image})


# 팝업 메시지 출력
@app.route("/popup_message")
def get_popup_message():
    if not session.get("logged_in"):
        return "Unauthorized", 403

    if not hasattr(app, "ros_node"):
        return jsonify({"message": ""})

    try:
        message = app.ros_node.message_queue.get_nowait()
        return jsonify({"message": message})
    except Queue.Empty:
        return jsonify({"message": ""})


@app.route("/reset_popup/<robot_id>")
def reset_popup(robot_id):
    """팝업 상태 리셋"""
    global popup_displayed
    if robot_id in popup_displayed:
        popup_displayed[robot_id] = False
        return jsonify({"message": f"Popup reset for {robot_id}"})
    return jsonify({"error": "Invalid robot_id"}), 400


# 로그 데이터
@app.route("/logs")
def get_logs():
    """DB에서 로그 데이터를 가져오는 API"""
    if not session.get("logged_in"):
        return "Unauthorized", 403
    try:
        # SQLite 데이터베이스에서 로그 가져오기
        conn = sqlite3.connect("src/db.db")
        cursor = conn.cursor()
        query = "SELECT time, message FROM log ORDER BY time DESC LIMIT 10"  # 최신 10개 로그 가져오기
        cursor.execute(query)
        logs = [{"time": row[0], "message": row[1]} for row in cursor.fetchall()]
        conn.close()
        return jsonify(logs)  # JSON 형식으로 반환
    except sqlite3.Error as e:
        return f"Database error: {e}", 500


def ros2_spin(node):
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.01)  # CPU 사용량 감소
    except Exception as e:
        print(f"ROS2 spin error: {e}")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


def main():
    # ROS2 노드 초기화
    rclpy.init()
    node = CamSubscriber()
    app.ros_node = node  # Flask 앱에 ROS 노드 저장

    # ROS2 스핀 스레드 시작
    ros_thread = threading.Thread(target=ros2_spin, args=(node,), daemon=True)
    ros_thread.start()

    try:
        # Flask 서버 시작
        app.run(host="0.0.0.0", port=5000, threaded=True)
    finally:
        # 정리
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
