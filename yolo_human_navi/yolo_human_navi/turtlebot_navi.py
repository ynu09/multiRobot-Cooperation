import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose

from rclpy.action import ActionClient
from rclpy.action.client import GoalStatus
from nav2_msgs.action import FollowWaypoints

import sqlite3
from sqlite3 import Error
import time


class MultiTurtlebotNavi(Node):
    def __init__(self):
        super().__init__("turtlebot_navi_node")
        self.goal_handles = {}
        self.restart_timers = {}

        # Initialize database connection
        self.init_database()

        # gazebo_multi_nav2_world.launch에서 spawn할 때 초기 위치 정해주니까 SetInitialPose 생략

        # [Topic Subscriber] YOLO 감지 (내비용)
        self.detection_subs = {
            "tb1": self.create_subscription(
                String,
                "/tb1/human_detection",
                lambda msg: self.human_detection_callback(msg, "tb1"),
                10,
            ),
            "tb2": self.create_subscription(
                String,
                "/tb2/human_detection",
                lambda msg: self.human_detection_callback(msg, "tb2"),
                10,
            ),
            "tb3": self.create_subscription(
                String,
                "/tb3/human_detection",
                lambda msg: self.human_detection_callback(msg, "tb3"),
                10,
            ),
            "dr1f": self.create_subscription(
                String,
                "/dr1f/human_detection",
                lambda msg: self.human_detection_callback(msg, "dr1f"),
                10,
            ),
            "dr1b": self.create_subscription(
                String,
                "/dr1b/human_detection",
                lambda msg: self.human_detection_callback(msg, "dr1b"),
                10,
            ),
        }

        # [Topic2 Subscriber] 로봇 위치
        self.last_logged = 0
        self.sub_pos_tb1 = self.create_subscription(
            PoseWithCovarianceStamped,
            "/tb1/amcl_pose",
            lambda msg: self.update_robot_position(msg, "tb1"),
            10,
        )
        self.sub_pos_tb2 = self.create_subscription(
            PoseWithCovarianceStamped,
            "/tb2/amcl_pose",
            lambda msg: self.update_robot_position(msg, "tb2"),
            10,
        )
        self.sub_pos_tb3 = self.create_subscription(
            PoseWithCovarianceStamped,
            "/tb3/amcl_pose",
            lambda msg: self.update_robot_position(msg, "tb3"),
            10,
        )
        self.sub_pos_dr1 = self.create_subscription(
            Pose,
            "/dr1/gt_pose",
            lambda msg: self.update_robot_position(msg, "dr1"),
            10,
        )

        # [Action Client] 각 터틀봇 Waypoint 내비
        self.waypoint_clients = {
            "tb1": ActionClient(self, FollowWaypoints, "/tb1/follow_waypoints"),
            "tb2": ActionClient(self, FollowWaypoints, "/tb2/follow_waypoints"),
            "tb3": ActionClient(self, FollowWaypoints, "/tb3/follow_waypoints"),
        }

        # 각 터틀봇의 웨이포인트 설정
        self.waypoints = {
            "tb1": [
                {
                    "position": [3.0161631270421796, -1.4422158328945343],
                    "orientation": [0.99979924173923, 0.02003687145391784],
                },
                {
                    "position": [-5.8101827982740435, -1.383929786507276],
                    "orientation": [-0.9701417426386765, 0.24253865504284466],
                },
                {
                    "position": [-7.788308071271799, -9.630021666229611],
                    "orientation": [-0.8171301187534513, 0.5764532669921913],
                },
                {
                    "position": [-8.815692604565287, -13.861961745267651],
                    "orientation": [-0.5299286488359101, 0.8480422319336147],
                },
                {
                    "position": [-8.74504272921578, -28.885099314018372],
                    "orientation": [-0.19227061480014793, 0.981341943811826],
                },
                {
                    "position": [10.711419238366773, -28.432036994561102],
                    "orientation": [0.7877636902092908, 0.6159775713350614],
                },
                {
                    "position": [8.48740433074394, -14.928932754121428],
                    "orientation": [0.9568343400366977, 0.29063386885656894],
                },
                {
                    "position": [8.643919410263429, -7.177615645588609],
                    "orientation": [0.5640818352732621, 0.8257188886750432],
                },
            ],
            "tb2": [
                {
                    "position": [16.165101740614062, -5.747140591243882],
                    "orientation": [-0.590045409570518, 0.8073700605328139],
                },
                {
                    "position": [16.976360093201357, 0.7948136035872039],
                    "orientation": [0.8466122734932133, 0.5322101637235545],
                },
                {
                    "position": [27.889480573673328, 9.27738469393071],
                    "orientation": [0.05753832844685656, 0.9983432980490937],
                },
                {
                    "position": [29.11476630716706, -11.129364021192913],
                    "orientation": [-0.7146300809002273, 0.6995025714552696],
                },
                {
                    "position": [22.516774684472146, -12.023316619940662],
                    "orientation": [-0.9995251252549825, 0.030814347048274782],
                },
            ],
            "tb3": [
                {
                    "position": [16.09692885803449, -13.0612694406839],
                    "orientation": [-0.6372530217208481, 0.7706546478855807],
                },
                {
                    "position": [17.9209957051445, -15.940316654694865],
                    "orientation": [-0.19407855179166453, 0.9809859916096917],
                },
                {
                    "position": [22.2851252302164, -16.259286435777494],
                    "orientation": [-0.10289096948012709, 0.9946926401655134],
                },
                {
                    "position": [24.518450791571095, -15.854008904372686],
                    "orientation": [-0.03573459755819703, 0.99936131530961],
                },
                {
                    "position": [29.154423581940755, -24.232828661615255],
                    "orientation": [-0.6017866747370478, 0.7986568713214246],
                },
                {
                    "position": [29.490570904874442, -26.656547508314297],
                    "orientation": [-0.5453998575784409, 0.8381759930667404],
                },
                {
                    "position": [12.875300774942577, -29.3808703571920],
                    "orientation": [0.9998315225954153, 0.01835555568577111],
                },
                {
                    "position": [12.046465734617959, -21.43655296589399],
                    "orientation": [0.800983083053206, 0.5986869805353887],
                },
                # {'position': [, ], 'orientation': [, ]},
            ],
        }

        # 각 터틀봇의 상태 관리
        self.robot_states = {
            "tb1": {"searching": False, "human_found": False},
            "tb2": {"searching": False, "human_found": False},
            "tb3": {"searching": False, "human_found": False},
            "dr1f": {"searching": False, "human_found": False},
            "dr1b": {"searching": False, "human_found": False},
        }

    def start_search(self):
        # 모든 터틀봇 탐색 시작
        for robot_id in self.waypoint_clients.keys():
            if not self.robot_states[robot_id]["searching"]:
                self.robot_states[robot_id]["searching"] = True
                self.send_waypoints(robot_id)

    def log_to_db(self, message):
        # 현재 시간과 함께 로그 메시지 db에 저장
        try:
            cursor = self.conn.cursor()
            cursor.execute(
                """
                CREATE TABLE IF NOT EXISTS log (
                    time TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    message TEXT
                )
            """
            )
            cursor.execute("INSERT INTO log (message) VALUES (?)", (message,))
            self.conn.commit()
        except Error as e:
            self.get_logger().error(f"Database logging error: {str(e)}")

    def send_waypoints(self, robot_id):
        # 특정 터틀봇에 웨이포인트 전송
        if not self.waypoint_clients[robot_id].wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f"{robot_id}: Waypoint action server not available")
            return

        goal_msg = FollowWaypoints.Goal()
        poses = []

        for waypoint in self.waypoints[robot_id]:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = waypoint["position"][0]
            pose.pose.position.y = waypoint["position"][1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = waypoint["orientation"][0]
            pose.pose.orientation.w = waypoint["orientation"][1]
            poses.append(pose)

        goal_msg.poses = poses

        if self.robot_states[robot_id]["human_found"]:
            self.get_logger().info(
                f"Not sending waypoints for {robot_id} as human was found"
            )
            return

        self.get_logger().info(f"Sending waypoint navigation goal for {robot_id}")

        if not self.waypoint_clients[robot_id].wait_for_server(timeout_sec=2.0):
            self.get_logger().error(f"{robot_id}: Waypoint action server not available")
            return

        # 로봇별로 future 객체 저장
        setattr(
            self,
            f"{robot_id}_waypoint_future",
            self.waypoint_clients[robot_id].send_goal_async(
                goal_msg,
            ),
        )

        getattr(self, f"{robot_id}_waypoint_future").add_done_callback(
            lambda future, r_id=robot_id: self.waypoint_response_callback(future, r_id)
        )

    def convert_coords(self, x, y):
        # rviz-to-world.png 좌표변환
        p00 = (-10, 10)
        p01 = (30, -30)

        x = (x - p00[0]) / (p01[0] - p00[0]) * 814
        y = (y - p00[1]) / (p01[1] - p00[1]) * 814

        return x, y

    # db에 좌표값 저장
    def init_database(self):
        """Initialize the SQLite database and create the robot table if it doesn't exist."""
        try:
            self.conn = sqlite3.connect("src/db.db")
            cursor = self.conn.cursor()
            cursor.execute(
                """
                CREATE TABLE IF NOT EXISTS robot (
                    x REAL,
                    y REAL,
                    name TEXT
                )
            """
            )
            cursor.execute(
                """
                CREATE TABLE IF NOT EXISTS log (
                    time TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    message TEXT
                )
            """
            )
            self.conn.commit()
        except Error as e:
            self.get_logger().error(f"Database error: {str(e)}")

    def parse_xy_tb(self, msg):
        """Parse x, y coordinates from PoseStamped message."""
        return msg.pose.pose.position.x, msg.pose.pose.position.y

    def parse_xy_dr(self, msg):
        """Parse x, y coordinates from PoseStamped message."""
        return msg.position.x, msg.position.y

    def update_robot_position(self, msg, robot_id):
        print("update callback start", robot_id)
        if time.time() - self.last_logged < 0.5:
            return

        self.last_logged = time.time()
        """Update robot position in the database."""
        if robot_id.startswith("dr"):
            x, y = self.parse_xy_dr(msg)
        else:
            x, y = self.parse_xy_tb(msg)
        x, y = self.convert_coords(x, y)
        try:
            cursor = self.conn.cursor()
            # First delete any existing entries for this robot
            cursor.execute("DELETE FROM robot WHERE name = ?", (robot_id,))
            # Insert new position
            cursor.execute(
                "INSERT INTO robot (x, y, name) VALUES (?, ?, ?)", (x, y, robot_id)
            )
            self.conn.commit()
        except Error as e:
            self.get_logger().error(f"Database update error: {str(e)}")

    def __del__(self):
        """Cleanup database connection when the node is destroyed."""
        if hasattr(self, "conn"):
            self.conn.close()

    def waypoint_response_callback(self, future, robot_id):
        goal_handle = future.result()
        if not goal_handle.accepted:
            message = f"{robot_id} waypoint navigation goal rejected"
            self.get_logger().error(message)
            self.log_to_db(message)
            return

        # goal handle 저장
        self.goal_handles[robot_id] = goal_handle

        message = f"{robot_id} waypoint navigation goal accepted"
        self.get_logger().info(message)
        self.log_to_db(message)

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future, r_id=robot_id: self.waypoint_result_callback(future, r_id)
        )

    def waypoint_result_callback(self, future, robot_id):
        try:
            result = future.result().result

            # goal handle 정리
            if robot_id in self.goal_handles:
                del self.goal_handles[robot_id]

            if not self.robot_states[robot_id]["human_found"]:
                message = f"{robot_id} search completed without finding human, restarting search..."
                self.get_logger().info(message)
                self.log_to_db(message)
                self.robot_states[robot_id]["searching"] = False

                # 기존 타이머가 있다면 취소
                if robot_id in self.restart_timers:
                    self.restart_timers[robot_id].cancel()

                # 새 타이머 생성 및 저장
                self.restart_timers[robot_id] = self.create_timer(
                    2.0,  # 2초 딜레이
                    lambda: self.restart_search_if_needed(robot_id),
                    # oneshot=True  # 한 번만 실행
                )
        except Exception as e:
            self.get_logger().error(
                f"Error in waypoint_result_callback for {robot_id}: {str(e)}"
            )

    def restart_search_if_needed(self, robot_id):
        try:
            # 타이머 정리
            if robot_id in self.restart_timers:
                # 타이머 취소
                self.restart_timers[robot_id].cancel()
                del self.restart_timers[robot_id]

            if not self.robot_states[robot_id]["human_found"]:
                # Action Server 상태 확인
                if not self.waypoint_clients[robot_id].wait_for_server(timeout_sec=1.0):
                    self.get_logger().warning(
                        f"{robot_id} Action server not ready, will retry in 2 seconds"
                    )
                    self.restart_timers[robot_id] = self.create_timer(
                        2.0,
                        lambda: self.restart_search_if_needed(robot_id),
                        oneshot=True,
                    )
                    return

                self.get_logger().info(f"Restarting search for {robot_id}")
                self.robot_states[robot_id]["searching"] = True  # searching 상태 설정
                self.send_waypoints(robot_id)
            else:
                self.get_logger().info(
                    f"Not restarting search for {robot_id} as human was found"
                )
        except Exception as e:
            self.get_logger().error(
                f"Error in restart_search_if_needed for {robot_id}: {str(e)}"
            )

    def human_detection_callback(self, msg, robot_id):
        if msg.data == "Human detected!" and self.robot_states[robot_id]["searching"]:
            self.robot_states[robot_id]["human_found"] = True
            self.robot_states[robot_id]["searching"] = False
            message = f"Human found by {robot_id}! Canceling its navigation..."
            self.get_logger().info(message)
            self.log_to_db(message)

            # goal handle이 있는 경우에만 취소
            if robot_id in self.goal_handles:
                goal_handle = self.goal_handles[robot_id]
                if goal_handle:
                    message = f"Canceling navigation for {robot_id}"
                    self.get_logger().info(message)
                    self.log_to_db(message)
                    goal_handle.cancel_goal_async()
            else:
                self.get_logger().warning(f"No active goal handle found for {robot_id}")


def main(args=None):
    rclpy.init(args=args)
    turtlebot_navi = MultiTurtlebotNavi()
    turtlebot_navi.start_search()
    rclpy.spin(turtlebot_navi)
    turtlebot_navi.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
