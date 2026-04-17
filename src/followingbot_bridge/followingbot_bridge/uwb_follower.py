import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class UwbFollowerNode(Node):
    def __init__(self):
        super().__init__('uwb_follower_node')

        self.declare_parameter('a0_topic', '/uwb_a0')
        self.declare_parameter('a1_topic', '/uwb_a1')
        self.declare_parameter('loop_hz', 20.0)

        self.declare_parameter('stop_distance_m', 0.60)
        self.declare_parameter('slow_distance_m', 1.20)
        self.declare_parameter('max_speed_mps', 0.80)
        self.declare_parameter('min_speed_mps', 0.25)

        self.declare_parameter('turn_steer_rad', 0.35)
        self.declare_parameter('straight_diff_m', 0.08)
        self.declare_parameter('turn_diff_m', 0.18)

        self.declare_parameter('ema_alpha', 0.35)
        self.declare_parameter('data_timeout_s', 0.30)
        self.declare_parameter('invalid_max_m', 10.0)

        self.declare_parameter('steer_sign', 1.0)

        self.a0_topic = self.get_parameter('a0_topic').value
        self.a1_topic = self.get_parameter('a1_topic').value
        loop_hz = float(self.get_parameter('loop_hz').value)

        self.stop_distance_m = float(self.get_parameter('stop_distance_m').value)
        self.slow_distance_m = float(self.get_parameter('slow_distance_m').value)
        self.max_speed_mps = float(self.get_parameter('max_speed_mps').value)
        self.min_speed_mps = float(self.get_parameter('min_speed_mps').value)

        self.turn_steer_rad = float(self.get_parameter('turn_steer_rad').value)
        self.straight_diff_m = float(self.get_parameter('straight_diff_m').value)
        self.turn_diff_m = float(self.get_parameter('turn_diff_m').value)

        self.ema_alpha = float(self.get_parameter('ema_alpha').value)
        self.data_timeout_s = float(self.get_parameter('data_timeout_s').value)
        self.invalid_max_m = float(self.get_parameter('invalid_max_m').value)
        self.steer_sign = float(self.get_parameter('steer_sign').value)

        self.a0_raw = None
        self.a1_raw = None
        self.a0_f = None
        self.a1_f = None

        self.last_a0_time = None
        self.last_a1_time = None

        self.sub_a0 = self.create_subscription(Float64, self.a0_topic, self.a0_callback, 10)
        self.sub_a1 = self.create_subscription(Float64, self.a1_topic, self.a1_callback, 10)

        self.pub_speed = self.create_publisher(Float64, '/target_speed', 10)
        self.pub_steer = self.create_publisher(Float64, '/target_steer', 10)
        self.pub_avg = self.create_publisher(Float64, '/uwb_avg_dist', 10)
        self.pub_diff = self.create_publisher(Float64, '/uwb_lr_diff', 10)

        period = 1.0 / loop_hz
        self.timer = self.create_timer(period, self.control_loop)

        self.get_logger().info('uwb_follower_node started')

    def a0_callback(self, msg: Float64):
        v = float(msg.data)
        if self.is_valid(v):
            self.a0_raw = v
            self.a0_f = self.ema(self.a0_f, v)
            self.last_a0_time = self.get_clock().now()

    def a1_callback(self, msg: Float64):
        v = float(msg.data)
        if self.is_valid(v):
            self.a1_raw = v
            self.a1_f = self.ema(self.a1_f, v)
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
        if not self.data_fresh(self.last_a0_time) or not self.data_fresh(self.last_a1_time):
            self.publish_cmd(0.0, 0.0)
            self.get_logger().warn('UWB timeout -> STOP', throttle_duration_sec=1.0)
            return

        if self.a0_f is None or self.a1_f is None:
            self.publish_cmd(0.0, 0.0)
            return

        a0 = self.a0_f
        a1 = self.a1_f

        avg = 0.5 * (a0 + a1)
        diff = a1 - a0

        self.pub_avg.publish(Float64(data=avg))
        self.pub_diff.publish(Float64(data=diff))

        if avg <= self.stop_distance_m:
            target_speed = 0.0
        elif avg >= self.slow_distance_m:
            target_speed = self.max_speed_mps
        else:
            ratio = (avg - self.stop_distance_m) / (self.slow_distance_m - self.stop_distance_m)
            target_speed = self.min_speed_mps + ratio * (self.max_speed_mps - self.min_speed_mps)

        abs_diff = abs(diff)

        if target_speed <= 0.01:
            target_steer = 0.0
        elif abs_diff <= self.straight_diff_m:
            target_steer = 0.0
        else:
            gain = (abs_diff - self.straight_diff_m) / max(1e-6, (self.turn_diff_m - self.straight_diff_m))
            gain = max(0.0, min(1.0, gain))
            steer_mag = gain * self.turn_steer_rad

            if diff > 0.0:
                target_steer = self.steer_sign * steer_mag
            else:
                target_steer = -self.steer_sign * steer_mag

        self.publish_cmd(target_speed, target_steer)

        self.get_logger().info(
            f'a0={a0:.3f}, a1={a1:.3f}, avg={avg:.3f}, diff={diff:.3f}, '
            f'spd={target_speed:.3f}, steer={target_steer:.3f}',
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
