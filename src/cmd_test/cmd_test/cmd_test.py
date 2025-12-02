import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class CmdVelStateMachine(Node):

    def __init__(self):
        super().__init__("cmd_vel_state_machine_node")

        # ---------------------------------------------------------
        # 1) Publisher: /cmd_vel → 모터(바퀴) 속도 제어 토픽
        # ---------------------------------------------------------
        # linear.x  = 전진/후진 속도 (m/s)
        # angular.z = 좌/우 회전 속도 (rad/s)
        # ---------------------------------------------------------
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # ---------------------------------------------------------
        # 2) Subscriber: /scan → LiDAR 거리 데이터
        # ---------------------------------------------------------
        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.lidar_callback,
            10,
        )

        # 제어 루프 (10Hz = 0.1초마다 실행)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # ---------------------------------------------------------
        # 상태머신 초기 상태 정의
        # ---------------------------------------------------------
        self.state = "FORWARD"         # FORWARD, CHECK, BACKWARD, TURN
        self.turn_direction = None     # left or right

        # ---------------------------------------------------------
        # 라이다 스캔 관련 변수 초기화
        # ---------------------------------------------------------
        self.has_scan = False
        self.min_front = None
        self.left_space = 0.0
        self.right_space = 0.0
        self.front_blocked = False

        # 시간 기록 변수 (회전/후진 시간 측정용)
        self.backward_start_time_ns = 0
        self.turn_start_time_ns = 0

        # ---------------------------------------------------------
        # 장애물 감지/회피 파라미터 (환경에 따라 조절 가능)
        # ---------------------------------------------------------
        self.FRONT_BLOCK_DIST = 0.60    # 앞 0.60m 이내 장애물 → 막힘
        self.SAFE_FRONT_DIST  = 0.80    # 앞 0.80m 이상 확보 → 전진 가능
        self.MIN_SIDE_SPACE   = 0.50    # 좌/우 0.50m 이내면 좁다고 판단

        self.BACKWARD_TIME = 1.0        # 후진 시간 (초)
        self.TURN_TIME_MIN = 0.8        # 회전 최소 시간
        self.TURN_TIME_MAX = 3.0        # 회전 최대 시간

        self.get_logger().info("State machine cmd_vel node started.")


    # ========================================================================
    # 라이다 콜백 함수: LiDAR 데이터가 들어올 때마다 실행됨
    # ========================================================================
    def lidar_callback(self, msg: LaserScan):
        ranges = msg.ranges       # 라이다 거리 배열
        n = len(ranges)           # 스캔 포인트 개수 (예: 600)

        if n == 0:
            return

        self.has_scan = True
        center = n // 2           # 라이다 정면 방향 인덱스

        # ====================================================================
        # 라이다 각도 설정 (강의실 환경에 최적화된 값)
        #
        # FRONT_ANGLE = 정면 장애물 감지 범위 (center 기준 ± FRONT_ANGLE)
        # SIDE_ANGLE  = 좌/우 여유 공간 판단 범위
        #
        # 강의실 = 가운데 공간 넓고, 주변 책상/벽 존재 → 중간 수준 각도가 최적
        # ====================================================================
        FRONT_ANGLE = 100     # 정면 약 ±100 포인트
        SIDE_ANGLE  = 160     # 좌/우 약 ±160 포인트


        # ====================================================================
        # 1) 정면 장애물 감지
        # ====================================================================
        f_start = max(0, center - FRONT_ANGLE)
        f_end   = min(n, center + FRONT_ANGLE)
        front_slice = ranges[f_start:f_end]

        front_valid = [r for r in front_slice if 0.05 < r < 5.0]

        if front_valid:
            self.min_front = min(front_valid)
            self.front_blocked = (self.min_front < self.FRONT_BLOCK_DIST)
        else:
            self.min_front = None
            self.front_blocked = True


        # ====================================================================
        # 2) 좌측 공간 감지
        # ====================================================================
        l_start = max(0, center - SIDE_ANGLE)
        l_end   = center
        left_slice = ranges[l_start:l_end]
        left_valid = [r for r in left_slice if 0.05 < r < 5.0]
        self.left_space = max(left_valid) if left_valid else 0.0


        # ====================================================================
        # 3) 우측 공간 감지
        # ====================================================================
        r_start = center
        r_end   = min(n, center + SIDE_ANGLE)
        right_slice = ranges[r_start:r_end]
        right_valid = [r for r in right_slice if 0.05 < r < 5.0]
        self.right_space = max(right_valid) if right_valid else 0.0


        # 디버그 출력
        self.get_logger().info(
            f"[LiDAR] front={self.min_front:.2f}, left={self.left_space:.2f}, right={self.right_space:.2f}, blocked={self.front_blocked}"
        )


    # ========================================================================
    # 주기적으로 실행되는 타이머 콜백 (10Hz)
    # ========================================================================
    def timer_callback(self):
        cmd = Twist()  # 로봇 바퀴 제어 메시지 객체

        # ====================================================================
        # LiDAR 데이터 없으면 안전 위해 정지
        # ====================================================================
        if not self.has_scan or self.min_front is None:
            self.get_logger().warn("LiDAR 데이터 없음 → 정지")
            self.cmd_pub.publish(cmd)
            return

        now_ns = self.get_clock().now().nanoseconds

        # ====================================================================
        #  상태머신 로직
        # ====================================================================

        # --------------------------------------------------------------------
        # 1) 전진 상태
        # --------------------------------------------------------------------
        if self.state == "FORWARD":

            if self.front_blocked:
                # 앞이 막혔으면 체크 단계로 넘어감
                self.state = "CHECK"
                self.get_logger().info("STATE: FORWARD → CHECK")
            else:
                #  바퀴 속도 설정 (전진)
                cmd.linear.x  = 0.25      # + 전진속도 (m/s)
                cmd.angular.z = 0.0       # 회전 없음
                # self.get_logger().info("전진 중")


        # --------------------------------------------------------------------
        # 2) CHECK 단계 → 좌/우 공간 분석해서 다음 행동 결정
        # --------------------------------------------------------------------
        elif self.state == "CHECK":
            self.get_logger().info(
                f"STATE: CHECK (front={self.min_front:.2f}, left={self.left_space:.2f}, right={self.right_space:.2f})"
            )

            # 양쪽 다 좁으면 → 후진해서 공간 다시 확보
            if self.left_space < self.MIN_SIDE_SPACE and self.right_space < self.MIN_SIDE_SPACE:
                self.state = "BACKWARD"
                self.backward_start_time_ns = now_ns
                self.get_logger().info("STATE: CHECK → BACKWARD")
            else:
                # 넓은 쪽 방향으로 회전
                self.turn_direction = "left" if self.left_space > self.right_space else "right"
                self.state = "TURN"
                self.turn_start_time_ns = now_ns
                self.get_logger().info(f"STATE: CHECK → TURN ({self.turn_direction})")


        # --------------------------------------------------------------------
        # 3) BACKWARD 상태 → 일정 시간 후진해서 탈출
        # --------------------------------------------------------------------
        elif self.state == "BACKWARD":
            elapsed = (now_ns - self.backward_start_time_ns) / 1e9

            if elapsed < self.BACKWARD_TIME:
                #  바퀴 속도 설정 (후진)
                cmd.linear.x  = -0.15     # - 후진속도 (m/s)
                cmd.angular.z = 0.0       # 회전 없음
                self.get_logger().info(f"STATE: BACKWARD ({elapsed:.2f}s)")
            else:
                self.state = "CHECK"
                self.get_logger().info("STATE: BACKWARD → CHECK")


        # --------------------------------------------------------------------
        # 4) TURN 상태 → 좌/우로 회전하여 공간 확보 후 전진으로 복귀
        # --------------------------------------------------------------------
        elif self.state == "TURN":
            elapsed = (now_ns - self.turn_start_time_ns) / 1e9

            #  바퀴 속도 설정 (좌/우 회전)
            cmd.linear.x = 0.0            # 회전 시 전진 금지
            if self.turn_direction == "left":
                cmd.angular.z = 0.5       # + 좌회전 (rad/s)
            else:
                cmd.angular.z = -0.5      # - 우회전 (rad/s)

            self.get_logger().info(
                f"STATE: TURN ({self.turn_direction}, {elapsed:.2f}s)"
            )

            # 너무 오래 회전해도 길이 안 열리면 → 후진으로 탈출
            if elapsed > self.TURN_TIME_MAX and self.front_blocked:
                self.state = "BACKWARD"
                self.backward_start_time_ns = now_ns
                self.get_logger().info("STATE: TURN → BACKWARD (길 안 열림)")

            # 일정 시간 회전 후 앞이 확보되면 → 다시 전진
            elif elapsed > self.TURN_TIME_MIN and (not self.front_blocked) and \
                    (self.min_front > self.SAFE_FRONT_DIST):
                self.state = "FORWARD"
                self.turn_direction = None
                self.get_logger().info("STATE: TURN → FORWARD (길 열림)")


        # ====================================================================
        # 최종 속도 명령 Publish
        # ====================================================================
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
