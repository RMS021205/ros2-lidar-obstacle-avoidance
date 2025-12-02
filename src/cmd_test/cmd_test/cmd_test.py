import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class CmdVelStateMachine(Node):
    """
    ======================================================================
    LiDAR 기반 장애물 회피 로봇 상태 머신 (ROS2 /cmd_vel 제어 노드)
    ----------------------------------------------------------------------
    ✔ LiDAR(/scan) 데이터를 읽고
    ✔ 전진/정지/후진/회전(TURN) 등을 자동으로 결정
    ✔ /cmd_vel 토픽으로 속도(Twist)를 publish
    ======================================================================
    상태 정의:
        - FORWARD   : 전진
        - CHECK     : 앞·좌·우 공간 분석 후 다음 행동 결정
        - BACKWARD  : 후진하여 공간 확보
        - TURN      : 좌/우 회전하여 경로 확보
    ======================================================================
    """

    def __init__(self):
        super().__init__("cmd_vel_state_machine_node")

        # ---------------------------------------------------------
        # Publisher: /cmd_vel → 로봇 바퀴 속도 제어
        # ---------------------------------------------------------
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # ---------------------------------------------------------
        # Subscriber: /scan → LiDAR 거리 데이터 입력
        # ---------------------------------------------------------
        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.lidar_callback,
            10,
        )

        # 제어 루프 (10Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # ---------------------------------------------------------
        # 상태머신 초기 설정
        # ---------------------------------------------------------
        self.state = "FORWARD"         # 기본은 전진
        self.turn_direction = None     # left / right

        # LiDAR 데이터 저장 변수
        self.has_scan = False
        self.min_front = None
        self.left_space = 0.0
        self.right_space = 0.0
        self.front_blocked = False

        # 회전/후진 시 경과 시간 측정용 timestamp
        self.backward_start_time_ns = 0
        self.turn_start_time_ns = 0

        # ---------------------------------------------------------
        # 장애물 감지 및 경로 판단 파라미터
        # ---------------------------------------------------------
        self.FRONT_BLOCK_DIST = 0.60   # 앞 0.60m 이내면 "막힘"
        self.SAFE_FRONT_DIST  = 0.80   # 앞 0.80m 이상 확보되면 "전진 OK"
        self.MIN_SIDE_SPACE   = 0.50   # 좌/우 0.50m 이하 → 좁은 공간

        self.BACKWARD_TIME = 1.0       # 후진 유지 시간(초)
        self.TURN_TIME_MIN = 0.8       # 최소 회전 시간
        self.TURN_TIME_MAX = 3.0       # 최대 회전 시간

        self.get_logger().info("State machine cmd_vel node started.")

    # ======================================================================
    # LiDAR 콜백: 장애물 거리 분석 (정면/좌/우)
    # ======================================================================
    def lidar_callback(self, msg: LaserScan):
        ranges = msg.ranges
        n = len(ranges)

        if n == 0:
            return

        self.has_scan = True
        center = n // 2  # 라이다 데이터의 정면 인덱스

        # --------------------------------------------------------------
        # 강의실 환경에 최적화된 인덱스 범위
        # --------------------------------------------------------------
        FRONT_ANGLE = 100  # 정면 ±100
        SIDE_ANGLE  = 160  # 좌/우 공간 분석 범위

        # -----------------------------
        # 1) 정면 거리 계산
        # -----------------------------
        front_slice = ranges[max(0, center - FRONT_ANGLE): min(n, center + FRONT_ANGLE)]
        front_valid = [r for r in front_slice if 0.05 < r < 5.0]

        if front_valid:
            self.min_front = min(front_valid)
            self.front_blocked = self.min_front < self.FRONT_BLOCK_DIST
        else:
            self.min_front = None
            self.front_blocked = True

        # -----------------------------
        # 2) 좌측 여유 공간 계산
        # -----------------------------
        left_slice = ranges[max(0, center - SIDE_ANGLE): center]
        left_valid = [r for r in left_slice if 0.05 < r < 5.0]
        self.left_space = max(left_valid) if left_valid else 0.0

        # -----------------------------
        # 3) 우측 여유 공간 계산
        # -----------------------------
        right_slice = ranges[center: min(n, center + SIDE_ANGLE)]
        right_valid = [r for r in right_slice if 0.05 < r < 5.0]
        self.right_space = max(right_valid) if right_valid else 0.0

        # 디버깅용 출력
        self.get_logger().info(
            f"[LiDAR] front={self.min_front:.2f}, "
            f"left={self.left_space:.2f}, right={self.right_space:.2f}, "
            f"blocked={self.front_blocked}"
        )

    # ======================================================================
    # 주기적으로 실행되는 제어 루프 (10Hz)
    # ======================================================================
    def timer_callback(self):
        cmd = Twist()
        now_ns = self.get_clock().now().nanoseconds

        # --------------------------------------------------------------
        # LiDAR 데이터가 없으면 로봇 정지
        # --------------------------------------------------------------
        if not self.has_scan or self.min_front is None:
            self.get_logger().warn("LiDAR 데이터 없음 → 정지")
            self.cmd_pub.publish(cmd)
            return

        # ==================================================================
        # 상태머신 (FSM: Finite State Machine)
        # ==================================================================

        # --------------------------------------------------------------
        # 1) FORWARD — 전진
        # --------------------------------------------------------------
        if self.state == "FORWARD":

            if self.front_blocked:
                # 앞이 막히면 체크 단계
                self.state = "CHECK"
                self.get_logger().info("STATE: FORWARD → CHECK")

            else:
                cmd.linear.x  = 0.25  # 전진 속도
                cmd.angular.z = 0.0

        # --------------------------------------------------------------
        # 2) CHECK — 앞/좌/우 공간 분석 후 행동 결정
        # --------------------------------------------------------------
        elif self.state == "CHECK":
            self.get_logger().info(
                f"STATE: CHECK (front={self.min_front:.2f}, "
                f"left={self.left_space:.2f}, right={self.right_space:.2f})"
            )

            # 양쪽 다 좁으면 → 후진
            if self.left_space < self.MIN_SIDE_SPACE and \
               self.right_space < self.MIN_SIDE_SPACE:

                self.state = "BACKWARD"
                self.backward_start_time_ns = now_ns
                self.get_logger().info("STATE: CHECK → BACKWARD")

            else:
                # 넓은 방향으로 회전
                self.turn_direction = "left" if self.left_space > self.right_space else "right"
                self.state = "TURN"
                self.turn_start_time_ns = now_ns
                self.get_logger().info(f"STATE: CHECK → TURN ({self.turn_direction})")

        # --------------------------------------------------------------
        # 3) BACKWARD — 후진하며 공간 확보
        # --------------------------------------------------------------
        elif self.state == "BACKWARD":
            elapsed = (now_ns - self.backward_start_time_ns) / 1e9

            if elapsed < self.BACKWARD_TIME:
                cmd.linear.x  = -0.15   # 후진 속도
                cmd.angular.z = 0.0
                self.get_logger().info(f"STATE: BACKWARD ({elapsed:.2f}s)")

            else:
                self.state = "CHECK"
                self.get_logger().info("STATE: BACKWARD → CHECK")

        # --------------------------------------------------------------
        # 4) TURN — 좌/우 회전하여 경로 확보
        # --------------------------------------------------------------
        elif self.state == "TURN":
            elapsed = (now_ns - self.turn_start_time_ns) / 1e9

            cmd.linear.x = 0.0
            cmd.angular.z = 0.5 if self.turn_direction == "left" else -0.5

            self.get_logger().info(
                f"STATE: TURN ({self.turn_direction}, {elapsed:.2f}s)"
            )

            # 너무 오래 회전해도 경로가 안 열리면 → 후진
            if elapsed > self.TURN_TIME_MAX and self.front_blocked:
                self.state = "BACKWARD"
                self.backward_start_time_ns = now_ns
                self.get_logger().info("STATE: TURN → BACKWARD (길 안 열림)")

            # 일정 시간 지나고 앞이 비면 → 전진으로 복귀
            elif elapsed > self.TURN_TIME_MIN and \
                 (not self.front_blocked) and \
                 (self.min_front > self.SAFE_FRONT_DIST):

                self.state = "FORWARD"
                self.turn_direction = None
                self.get_logger().info("STATE: TURN → FORWARD (길 열림)")

        # --------------------------------------------------------------
        # 속도 명령 Publish
        # --------------------------------------------------------------
        self.cmd_pub.publish(cmd)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = CmdVelStateMachine()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
