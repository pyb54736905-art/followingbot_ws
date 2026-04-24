import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String


class UwbFollowerNode(Node):
    def __init__(self):
        super().__init__('uwb_follower_node')

        self.declare_parameter('a0_topic', '/uwb_a0')
        self.declare_parameter('a1_topic', '/uwb_a1')
        self.declare_parameter('loop_hz', 5.0)

        self.declare_parameter('stop_distance_m', 0.60)
        self.declare_parameter('resume_distance_m', 0.80)
        self.declare_parameter('slow_distance_m', 1.20)
        self.declare_parameter('max_speed_mps', 0.80)
        self.declare_parameter('min_speed_mps', 0.25)

        self.declare_parameter('turn_steer_rad', 0.35)

        self.declare_parameter('anchor_spacing_m', 0.35)
        self.declare_parameter('x_bias_m', 0.0)

        # 각도 기반 dead zone (rad) - 이 각도 이하면 직진
        self.declare_parameter('center_half_angle_rad', 0.10)
        # 이 각도 이상이면 최대 조향
        self.declare_parameter('max_steer_angle_rad', 0.45)

        self.declare_parameter('ema_alpha', 0.35)
        self.declare_parameter('data_timeout_s', 20.0)
        self.declare_parameter('invalid_max_m', 10.0)

        self.declare_parameter('steer_sign', 1.0)
        self.declare_parameter('warmup_samples', 10)

        self.a0_topic = self.get_parameter('a0_topic').value
        self.a1_topic = self.get_parameter('a1_topic').value
        loop_hz = float(self.get_parameter('loop_hz').value)

        self.stop_distance_m = float(self.get_parameter('stop_distance_m').value)
        self.resume_distance_m = float(self.get_parameter('resume_distance_m').value)
        self.slow_distance_m = float(self.get_parameter('slow_distance_m').value)
        self.max_speed_mps = float(self.get_parameter('max_speed_mps').value)
        self.min_speed_mps = float(self.get_parameter('min_speed_mps').value)

        self.turn_steer_rad = float(self.get_parameter('turn_steer_rad').value)

        self.anchor_spacing_m = float(self.get_parameter('anchor_spacing_m').value)
        self.x_bias_m = float(self.get_parameter('x_bias_m').value)
        self.center_half_angle_rad = float(self.get_parameter('center_half_angle_rad').value)
        self.max_steer_angle_rad = float(self.get_parameter('max_steer_angle_rad').value)

        self.ema_alpha = float(self.get_parameter('ema_alpha').value)
        self.data_timeout_s = float(self.get_parameter('data_timeout_s').value)
        self.invalid_max_m = float(self.get_parameter('invalid_max_m').value)
        self.steer_sign = float(self.get_parameter('steer_sign').value)
        self.warmup_samples = int(self.get_parameter('warmup_samples').value)

        self.a0_raw = None
        self.a1_raw = None
        self.a0_f = None
        self.a1_f = None
        self.a0_count = 0
        self.a1_count = 0
        self.is_stopped = True
        self.resume_steer_count = 0

        self.last_a0_time = None
        self.last_a1_time = None

        self.sub_a0 = self.create_subscription(Float64, self.a0_topic, self.a0_callback, 10)
        self.sub_a1 = self.create_subscription(Float64, self.a1_topic, self.a1_callback, 10)

        self.pub_speed = self.create_publisher(Float64, '/target_speed', 10)
        self.pub_steer = self.create_publisher(Float64, '/target_steer', 10)
        self.pub_avg = self.create_publisher(Float64, '/uwb_avg_dist', 10)
        self.pub_diff = self.create_publisher(Float64, '/uwb_lr_diff', 10)
        self.pub_theta = self.create_publisher(Float64, '/uwb_theta', 10)
        self.pub_zone = self.create_publisher(String, '/uwb_position_zone', 10)

        period = 1.0 / loop_hz
        self.timer = self.create_timer(period, self.control_loop)

        self.get_logger().info('uwb_follower_node started')

    def a0_callback(self, msg: Float64):
        v = float(msg.data)
        if self.is_valid(v):
            self.a0_raw = v
            self.a0_f = self.ema(self.a0_f, v)
            self.a0_count += 1
            self.last_a0_time = self.get_clock().now()

    def a1_callback(self, msg: Float64):
        v = float(msg.data)
        if self.is_valid(v):
            self.a1_raw = v
            self.a1_f = self.ema(self.a1_f, v)
            self.a1_count += 1
            self.last_a1_time = self.get_clock().now()

    def is_valid(self, v: float) -> bool:
        return (v > 0.01) and (v < self.invalid_max_m) and math.isfinite(v)

    def ema(self, prev, new):
        if prev is None:
            return new
        return self.ema_alpha * new + (1.0 - self.ema_alpha) * prev

    def data_fresh(self, t):
        if t is None:
            return False
        dt = (self.get_clock().now() - t).nanoseconds * 1e-9
        return dt <= self.data_timeout_s

    def publish_cmd(self, speed, steer):
        self.pub_speed.publish(Float64(data=float(speed)))
        self.pub_steer.publish(Float64(data=float(steer)))

    def control_loop(self):
        if self.a0_f is None or self.a1_f is None:
            self.pub_diff.publish(Float64(data=999.0))
            self.pub_zone.publish(String(data='INIT'))
            self.publish_cmd(0.0, 0.0)
            self.get_logger().warn(
                'a0_f or a1_f is None -> STOP',
                throttle_duration_sec=1.0
            )
            return

        if self.a0_count < self.warmup_samples or self.a1_count < self.warmup_samples:
            self.pub_zone.publish(String(data='WARMUP'))
            self.publish_cmd(0.0, 0.0)
            self.get_logger().info(
                f'warming up: a0={self.a0_count}/{self.warmup_samples}, a1={self.a1_count}/{self.warmup_samples}',
                throttle_duration_sec=0.5
            )
            return

        if not self.data_fresh(self.last_a0_time) or not self.data_fresh(self.last_a1_time):
            self.pub_diff.publish(Float64(data=888.0))
            self.pub_zone.publish(String(data='TIMEOUT'))
            self.publish_cmd(0.0, 0.0)
            self.get_logger().warn(
                'UWB timeout -> STOP',
                throttle_duration_sec=1.0
            )
            return

        a0 = self.a0_f
        a1 = self.a1_f

        avg = 0.5 * (a0 + a1)

        d = self.anchor_spacing_m
        if d <= 1e-6:
            self.pub_diff.publish(Float64(data=777.0))
            self.pub_zone.publish(String(data='INVALID'))
            self.publish_cmd(0.0, 0.0)
            self.get_logger().warn(
                'anchor_spacing_m invalid -> STOP',
                throttle_duration_sec=1.0
            )
            return

        x_raw = ((a0 * a0) - (a1 * a1) + (d * d)) / (2.0 * d)
        x = x_raw - self.x_bias_m

        # 전방 거리 y 추정 후 태그 방향 각도 계산
        y = math.sqrt(max(0.0, avg * avg - x * x))
        theta = math.atan2(x, max(0.01, y))

        self.pub_avg.publish(Float64(data=avg))
        self.pub_diff.publish(Float64(data=x))
        self.pub_theta.publish(Float64(data=theta))

        if avg <= self.stop_distance_m:
            self.is_stopped = True
            self.resume_steer_count = 0
            target_speed = 0.0
        elif self.is_stopped and avg < self.resume_distance_m:
            target_speed = 0.0
        else:
            self.is_stopped = False
            self.resume_steer_count += 1
            if avg >= self.slow_distance_m:
                target_speed = self.max_speed_mps
            else:
                ratio = (avg - self.stop_distance_m) / (self.slow_distance_m - self.stop_distance_m)
                target_speed = self.min_speed_mps + ratio * (self.max_speed_mps - self.min_speed_mps)

        if target_speed <= 0.01:
            target_steer = 0.0
            zone = 'STOP'
        elif self.resume_steer_count <= self.warmup_samples:
            target_steer = 0.0
            zone = 'RESUME'
        elif abs(theta) <= self.center_half_angle_rad:
            target_steer = 0.0
            zone = 'CENTER'
        else:
            excess = abs(theta) - self.center_half_angle_rad
            denom = max(1e-6, self.max_steer_angle_rad - self.center_half_angle_rad)
            gain = min(1.0, excess / denom)
            steer_mag = gain * self.turn_steer_rad

            if theta > 0.0:
                target_steer = self.steer_sign * steer_mag
                zone = 'RIGHT'
            else:
                target_steer = -self.steer_sign * steer_mag
                zone = 'LEFT'

        self.publish_cmd(target_speed, target_steer)
        self.pub_zone.publish(String(data=zone))

        self.get_logger().info(
            f'a0={a0:.3f}, a1={a1:.3f}, avg={avg:.3f}, '
            f'x={x:.3f}, theta={math.degrees(theta):.1f}deg, '
            f'zone={zone}, spd={target_speed:.3f}, steer={target_steer:.3f}',
            throttle_duration_sec=0.5
        )


def main(args=None):
    rclpy.init(args=args)
    node = UwbFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
