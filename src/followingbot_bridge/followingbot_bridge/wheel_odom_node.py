import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros


class WheelOdomNode(Node):
    def __init__(self):
        super().__init__('wheel_odom_node')

        self.declare_parameter('track_m', 0.304)
        self.declare_parameter('wheel_radius_m', 0.10)
        self.declare_parameter('gear_ratio', 1.0)
        self.declare_parameter('motor_pole_pairs', 10.0)
        # True이면 /left_rpm, /right_rpm이 ERPM (VESC 기본)
        # False이면 기계적 RPM
        self.declare_parameter('erpm_input', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('update_hz', 50.0)

        self.track_m = float(self.get_parameter('track_m').value)
        self.wheel_radius_m = float(self.get_parameter('wheel_radius_m').value)
        self.gear_ratio = float(self.get_parameter('gear_ratio').value)
        self.motor_pole_pairs = float(self.get_parameter('motor_pole_pairs').value)
        self.erpm_input = bool(self.get_parameter('erpm_input').value)
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        update_hz = float(self.get_parameter('update_hz').value)

        # 차체 자세 (odom 기준)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # 최신 RPM 값
        self.left_rpm = 0.0
        self.right_rpm = 0.0

        self.last_time = self.get_clock().now()

        if self.publish_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)

        self.create_subscription(Float64, '/left_rpm', self._on_left_rpm, 10)
        self.create_subscription(Float64, '/right_rpm', self._on_right_rpm, 10)

        self.create_timer(1.0 / update_hz, self._update)

        self.get_logger().info(
            f'wheel_odom_node started | track={self.track_m}m '
            f'r={self.wheel_radius_m}m pole_pairs={self.motor_pole_pairs} '
            f'erpm_input={self.erpm_input}'
        )

    def _on_left_rpm(self, msg: Float64):
        self.left_rpm = float(msg.data)

    def _on_right_rpm(self, msg: Float64):
        self.right_rpm = float(msg.data)

    def _to_wheel_mps(self, rpm_value: float) -> float:
        """ERPM 또는 기계적 RPM → 바퀴 선속도 (m/s)"""
        if self.erpm_input:
            mech_rpm = rpm_value / self.motor_pole_pairs
        else:
            mech_rpm = rpm_value
        wheel_rpm = mech_rpm / self.gear_ratio
        return (wheel_rpm / 60.0) * 2.0 * math.pi * self.wheel_radius_m

    def _update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        # 너무 크거나 작은 dt 방어
        if dt <= 0.0 or dt > 0.5:
            return

        v_left = self._to_wheel_mps(self.left_rpm)
        v_right = self._to_wheel_mps(self.right_rpm)

        v = (v_left + v_right) * 0.5
        omega = (v_right - v_left) / self.track_m

        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt
        self.yaw += omega * dt

        while self.yaw > math.pi:
            self.yaw -= 2.0 * math.pi
        while self.yaw < -math.pi:
            self.yaw += 2.0 * math.pi

        # 쿼터니언 (yaw only)
        cy = math.cos(self.yaw * 0.5)
        sy = math.sin(self.yaw * 0.5)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = sy
        odom.pose.pose.orientation.w = cy
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega

        self.pub_odom.publish(odom)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = now.to_msg()
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.translation.z = 0.0
            tf.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf)

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
