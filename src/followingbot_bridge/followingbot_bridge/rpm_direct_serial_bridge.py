import math
import time
import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class RPMDirectSerialBridge(Node):
    def __init__(self):
        super().__init__('rpm_direct_serial_bridge')

        # ----------------------------
        # Serial params
        # ----------------------------
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('open_delay', 2.0)

        # ----------------------------
        # Robot params
        # ----------------------------
        self.declare_parameter('wheelbase_m', 0.48)
        self.declare_parameter('track_m', 0.304)
        self.declare_parameter('wheel_radius_m', 0.10)
        self.declare_parameter('gear_ratio', 1.0)
        self.declare_parameter('motor_pole_pairs', 10.0)

        # ----------------------------
        # Command shaping params
        # ----------------------------
        self.declare_parameter('tx_rate_hz', 100.0)
        self.declare_parameter('command_timeout_sec', 0.5)
        self.declare_parameter('max_erpm_per_sec', 800.0)
        self.declare_parameter('lpf_alpha', 0.25)
        self.declare_parameter('speed_lpf_alpha', 0.30)
        self.declare_parameter('zero_erpm_band', 50.0)
        self.declare_parameter('max_erpm', 6000.0)

        # 방향 반전
        self.declare_parameter('left_invert', False)
        self.declare_parameter('right_invert', False)

        # 최대 조향 입력
        self.declare_parameter('max_steer_rad', 0.49)

        # 디버그 로그
        self.declare_parameter('debug_log', True)

        # ----------------------------
        # Read params
        # ----------------------------
        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)
        self.open_delay = float(self.get_parameter('open_delay').value)

        self.wheelbase_m = float(self.get_parameter('wheelbase_m').value)
        self.track_m = float(self.get_parameter('track_m').value)
        self.wheel_radius_m = float(self.get_parameter('wheel_radius_m').value)
        self.gear_ratio = float(self.get_parameter('gear_ratio').value)
        self.motor_pole_pairs = float(self.get_parameter('motor_pole_pairs').value)

        self.tx_rate_hz = float(self.get_parameter('tx_rate_hz').value)
        self.command_timeout_sec = float(self.get_parameter('command_timeout_sec').value)
        self.max_erpm_per_sec = float(self.get_parameter('max_erpm_per_sec').value)
        self.lpf_alpha = float(self.get_parameter('lpf_alpha').value)
        self.speed_lpf_alpha = float(self.get_parameter('speed_lpf_alpha').value)
        self.zero_erpm_band = float(self.get_parameter('zero_erpm_band').value)
        self.max_erpm = float(self.get_parameter('max_erpm').value)

        self.left_invert = bool(self.get_parameter('left_invert').value)
        self.right_invert = bool(self.get_parameter('right_invert').value)
        self.max_steer_rad = float(self.get_parameter('max_steer_rad').value)
        self.debug_log = bool(self.get_parameter('debug_log').value)

        # ----------------------------
        # Internal state
        # ----------------------------
        self.target_speed_mps = 0.0
        self.target_steer_rad = 0.0
        self.filtered_steer_rad = 0.0
        self.filtered_meas_speed = None
        self.current_left_erpm = 0.0
        self.current_right_erpm = 0.0
        self.last_cmd_rx_time = self.get_clock().now()


        self.ser = None
        self.last_debug_print_time = self.get_clock().now()

        # ----------------------------
        # Publishers
        # ----------------------------
        self.pub_left_rpm = self.create_publisher(Float64, '/left_rpm', 10)
        self.pub_right_rpm = self.create_publisher(Float64, '/right_rpm', 10)
        self.pub_measured_speed = self.create_publisher(Float64, '/measured_speed', 10)

        # ----------------------------
        # Subscribers
        # ----------------------------
        self.sub_speed = self.create_subscription(
            Float64, '/target_speed', self.on_target_speed, 10
        )
        self.sub_steer = self.create_subscription(
            Float64, '/target_steer', self.on_target_steer, 10
        )

        # ----------------------------
        # Serial open
        # ----------------------------
        self.open_serial()

        # ----------------------------
        # Timers
        # ----------------------------
        self.dt = 1.0 / self.tx_rate_hz
        self.tx_timer = self.create_timer(self.dt, self.on_tx_timer)
        self.rx_timer = self.create_timer(0.01, self.read_serial)

        self.get_logger().info('rpm_direct_serial_bridge started')

    def on_target_speed(self, msg: Float64):
        self.target_speed_mps = float(msg.data)
        self.last_cmd_rx_time = self.get_clock().now()
        if self.debug_log:
            self.get_logger().info(f'RX /target_speed = {self.target_speed_mps:.3f}')

    def on_target_steer(self, msg: Float64):
        steer = float(msg.data)
        steer = max(-self.max_steer_rad, min(self.max_steer_rad, steer))
        self.target_steer_rad = steer
        self.last_cmd_rx_time = self.get_clock().now()
        if self.debug_log:
            self.get_logger().info(f'RX /target_steer = {self.target_steer_rad:.3f}')

    def open_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.01)
            self.get_logger().info(f'Serial opened: {self.port} @ {self.baud}')
            time.sleep(self.open_delay)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.get_logger().info('Serial buffers flushed')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            self.ser = None

    def send_serial_command(self, left_erpm: int, right_erpm: int, steer_rad: float):
        if self.ser is None:
            return
        cmd = f'RPMCMD,{left_erpm},{right_erpm},{steer_rad:.4f}\n'
        try:
            self.ser.write(cmd.encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f'Serial write error: {e}')

    def parse_state_line(self, line: str):
        data = {}
        parts = line.split(',')
        for item in parts[1:]:
            item = item.strip()
            if '=' not in item:
                continue
            k, v = item.split('=', 1)
            data[k.strip()] = v.strip()
        return data

    def read_serial(self):
        if self.ser is None:
            return
        try:
            while self.ser.in_waiting > 0:
                raw = self.ser.readline()
                try:
                    line = raw.decode('utf-8', errors='ignore').strip()
                except Exception:
                    continue

                if not line:
                    continue

                if self.debug_log:
                    self.get_logger().info(f'SERIAL_RX: {line}')

                if not line.startswith('STATE,'):
                    continue

                try:
                    parsed = self.parse_state_line(line)

                    lrpm = float(parsed.get('lrpm', '0.0'))
                    rrpm = float(parsed.get('rrpm', '0.0'))
                    meas_v = float(parsed.get('meas_v', '0.0'))

                    msg_l = Float64()
                    msg_l.data = lrpm
                    self.pub_left_rpm.publish(msg_l)

                    msg_r = Float64()
                    msg_r.data = rrpm
                    self.pub_right_rpm.publish(msg_r)

                    a = self.speed_lpf_alpha
                    if self.filtered_meas_speed is None:
                        self.filtered_meas_speed = meas_v
                    else:
                        self.filtered_meas_speed = a * meas_v + (1.0 - a) * self.filtered_meas_speed

                    msg_v = Float64()
                    msg_v.data = self.filtered_meas_speed
                    self.pub_measured_speed.publish(msg_v)

                except Exception as e:
                    self.get_logger().error(
                        f'Failed to parse STATE line: {line} / err={e}'
                    )
        except Exception as e:
            self.get_logger().error(f'Serial read error: {e}')

    def compute_rear_wheel_speeds(self, center_speed_mps: float, steer_rad: float):
        if abs(steer_rad) < 1e-5:
            return center_speed_mps, center_speed_mps

        R = self.wheelbase_m / math.tan(steer_rad)

        left_radius = abs(R - self.track_m * 0.5)
        right_radius = abs(R + self.track_m * 0.5)
        center_radius = abs(R)

        left_speed = center_speed_mps * (left_radius / center_radius)
        right_speed = center_speed_mps * (right_radius / center_radius)

        if center_speed_mps < 0.0:
            left_speed = -abs(left_speed)
            right_speed = -abs(right_speed)

        return left_speed, right_speed

    def mps_to_erpm(self, v_mps: float):
        wheel_rpm = (v_mps / (2.0 * math.pi * self.wheel_radius_m)) * 60.0
        mech_motor_rpm = wheel_rpm * self.gear_ratio
        erpm = mech_motor_rpm * self.motor_pole_pairs
        return erpm

    def low_pass(self, prev_value: float, target_value: float):
        a = self.lpf_alpha
        return (1.0 - a) * prev_value + a * target_value

    def slew_limit(self, current_value: float, target_value: float, max_delta: float):
        diff = target_value - current_value
        if diff > max_delta:
            return current_value + max_delta
        if diff < -max_delta:
            return current_value - max_delta
        return target_value

    def apply_target_deadband_and_limit(self, erpm: float):
        # 목표값에만 deadband 적용
        if abs(erpm) < self.zero_erpm_band:
            return 0.0
        return max(-self.max_erpm, min(self.max_erpm, erpm))

    def apply_limit_only(self, erpm: float):
        # 최종 명령에는 deadband 적용하지 않음
        return max(-self.max_erpm, min(self.max_erpm, erpm))

    def on_tx_timer(self):
        now = self.get_clock().now()
        age = (now - self.last_cmd_rx_time).nanoseconds * 1e-9

        if age > self.command_timeout_sec:
            target_speed = 0.0
            target_steer = 0.0
        else:
            target_speed = self.target_speed_mps
            target_steer = self.target_steer_rad

        # 조향 LPF: 급격한 steer 변화를 완화해 진동/슬립 방지
        self.filtered_steer_rad = self.low_pass(self.filtered_steer_rad, target_steer)
        target_steer = self.filtered_steer_rad

        # 에커만 계산
        left_speed_mps, right_speed_mps = self.compute_rear_wheel_speeds(
            target_speed, target_steer
        )

        left_target_erpm = self.mps_to_erpm(left_speed_mps)
        right_target_erpm = self.mps_to_erpm(right_speed_mps)

        if self.left_invert:
            left_target_erpm *= -1.0
        if self.right_invert:
            right_target_erpm *= -1.0

        # deadband는 중심 속도 기준으로만 판단 — 개별 바퀴에 적용 시
        # 내측 바퀴가 제로화되어 외측만 돌면서 과도하게 꺾이는 현상 발생
        center_erpm = self.mps_to_erpm(target_speed)
        if abs(center_erpm) < self.zero_erpm_band:
            left_target_erpm = 0.0
            right_target_erpm = 0.0
        else:
            left_target_erpm  = max(-self.max_erpm, min(self.max_erpm, left_target_erpm))
            right_target_erpm = max(-self.max_erpm, min(self.max_erpm, right_target_erpm))

        # ERPM slew는 VESC 내부 가속 제어에 맡김 (외부 slew 시 극저속 명령으로 모터 미작동)

        if self.debug_log:
            elapsed = (now - self.last_debug_print_time).nanoseconds * 1e-9
            if elapsed >= 0.2:
                self.last_debug_print_time = now
                self.get_logger().info(
                    f'CMD target_speed={target_speed:.3f}, '
                    f'target_steer={target_steer:.3f}, '
                    f'left_target_erpm={left_target_erpm:.1f}, '
                    f'right_target_erpm={right_target_erpm:.1f}, '
                    f'age={age:.3f}'
                )

        self.send_serial_command(
            int(round(left_target_erpm)),
            int(round(right_target_erpm)),
            target_steer
        )

    def destroy_node(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RPMDirectSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
