import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class UwbPathGenNode(Node):
    """
    UWB 타겟 방향 + 현재 자세(odom) → 단기 경로(/path) 생성

    uwb_follower의 theta 부호 규칙 (steer_sign=-1.0 기준):
      theta > 0  → 타겟이 로봇 오른쪽
      theta < 0  → 타겟이 로봇 왼쪽

    theta_sign=-1.0 으로 설정하면 DWA의 goal_theta_sign 과 동일하게 정렬됨.
    """

    def __init__(self):
        super().__init__('uwb_path_gen_node')

        self.declare_parameter('path_length_m', 2.0)
        self.declare_parameter('n_waypoints', 10)
        self.declare_parameter('loop_hz', 20.0)
        self.declare_parameter('frame_id', 'odom')
        # uwb theta → 로봇 기준 방향 부호 보정
        # DWA의 goal_theta_sign 과 동일값 사용
        self.declare_parameter('theta_sign', -1.0)
        self.declare_parameter('data_timeout_s', 1.0)
        # 경로 방향 theta EMA 알파 (낮을수록 강한 스무딩, uwb_follower의 theta EMA 이후 2차 필터)
        self.declare_parameter('dir_ema_alpha', 0.15)

        self.path_length_m = float(self.get_parameter('path_length_m').value)
        self.n_waypoints = int(self.get_parameter('n_waypoints').value)
        loop_hz = float(self.get_parameter('loop_hz').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.theta_sign = float(self.get_parameter('theta_sign').value)
        self.data_timeout_s = float(self.get_parameter('data_timeout_s').value)
        self.dir_ema_alpha = float(self.get_parameter('dir_ema_alpha').value)

        self.odom: Odometry | None = None
        self.uwb_theta = 0.0
        self.uwb_dist = 0.0
        self.last_odom_t = None
        self.last_uwb_t = None
        self._dir_ema: float | None = None  # 경로 방향각 EMA 상태

        self.pub_path = self.create_publisher(Path, '/path', 10)

        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(Float64, '/uwb_theta', self._on_theta, 10)
        self.create_subscription(Float64, '/uwb_avg_dist', self._on_dist, 10)

        self.create_timer(1.0 / loop_hz, self._update)

        self.get_logger().info(
            f'uwb_path_gen_node started | '
            f'path_length={self.path_length_m}m n_wp={self.n_waypoints} '
            f'theta_sign={self.theta_sign}'
        )

    def _on_odom(self, msg: Odometry):
        self.odom = msg
        self.last_odom_t = self.get_clock().now()

    def _on_theta(self, msg: Float64):
        self.uwb_theta = float(msg.data)
        self.last_uwb_t = self.get_clock().now()

    def _on_dist(self, msg: Float64):
        self.uwb_dist = float(msg.data)

    def _fresh(self, t) -> bool:
        if t is None:
            return False
        return (self.get_clock().now() - t).nanoseconds * 1e-9 < self.data_timeout_s

    def _yaw_from_odom(self) -> float:
        q = self.odom.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _update(self):
        if not self._fresh(self.last_odom_t) or not self._fresh(self.last_uwb_t):
            return

        robot_x = self.odom.pose.pose.position.x
        robot_y = self.odom.pose.pose.position.y
        robot_yaw = self._yaw_from_odom()

        # theta_sign 적용: uwb theta → 로봇 로컬 프레임 각도
        # → odom 프레임 목표 방향으로 변환
        raw_angle = robot_yaw + self.theta_sign * self.uwb_theta

        # 경로 방향 EMA: uwb theta 노이즈가 경로를 흔들지 않도록 2차 스무딩
        if self._dir_ema is None:
            self._dir_ema = raw_angle
        else:
            # 각도 wrap-around를 고려한 EMA
            diff = raw_angle - self._dir_ema
            while diff > math.pi:
                diff -= 2.0 * math.pi
            while diff < -math.pi:
                diff += 2.0 * math.pi
            self._dir_ema += self.dir_ema_alpha * diff

        target_angle = self._dir_ema

        stamp = self.get_clock().now().to_msg()
        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = self.frame_id

        for i in range(self.n_waypoints):
            d = self.path_length_m * (i + 1) / self.n_waypoints
            wp = PoseStamped()
            wp.header.stamp = stamp
            wp.header.frame_id = self.frame_id
            wp.pose.position.x = robot_x + d * math.cos(target_angle)
            wp.pose.position.y = robot_y + d * math.sin(target_angle)
            wp.pose.position.z = 0.0
            wp.pose.orientation.x = 0.0
            wp.pose.orientation.y = 0.0
            wp.pose.orientation.z = math.sin(target_angle * 0.5)
            wp.pose.orientation.w = math.cos(target_angle * 0.5)
            path.poses.append(wp)

        self.pub_path.publish(path)

        self.get_logger().info(
            f'theta={math.degrees(self.uwb_theta):.1f}deg '
            f'dist={self.uwb_dist:.2f}m '
            f'target_angle={math.degrees(target_angle):.1f}deg',
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = UwbPathGenNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
