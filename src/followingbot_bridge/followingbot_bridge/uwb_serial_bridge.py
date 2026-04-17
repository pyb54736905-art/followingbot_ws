import serial
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class UwbSerialBridge(Node):
    def __init__(self):
        super().__init__('uwb_serial_bridge')

        self.declare_parameter('port', '/dev/ttyACM1')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('open_delay', 2.0)
        self.declare_parameter('read_period', 0.02)
        self.declare_parameter('verbose_rx', True)

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)
        self.open_delay = float(self.get_parameter('open_delay').value)
        self.read_period = float(self.get_parameter('read_period').value)
        self.verbose_rx = bool(self.get_parameter('verbose_rx').value)

        self.pub_a0 = self.create_publisher(Float64, '/uwb_a0', 10)
        self.pub_a1 = self.create_publisher(Float64, '/uwb_a1', 10)

        self.ser = None
        self.open_serial()

        self.timer = self.create_timer(self.read_period, self.read_serial)

        self.get_logger().info('uwb_serial_bridge started')

    def open_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.01)
            self.get_logger().info(f'Serial opened: {self.port} @ {self.baud}')
            time.sleep(self.open_delay)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.get_logger().info('UWB serial ready')
        except Exception as e:
            self.get_logger().error(f'Failed to open UWB serial port: {e}')
            self.ser = None

    def parse_uwb_line(self, line: str):
        """
        기대 형식:
        UWB,a0=1.234,a1=1.567
        """
        if not line.startswith('UWB,'):
            return None

        parts = line.split(',')
        if len(parts) < 3:
            return None

        data = {}
        for item in parts[1:]:
            if '=' in item:
                k, v = item.split('=', 1)
                data[k.strip()] = v.strip()

        try:
            a0 = float(data['a0'])
            a1 = float(data['a1'])
            return a0, a1
        except Exception:
            return None

    def read_serial(self):
        if self.ser is None:
            return

        try:
            while self.ser.in_waiting > 0:
                raw = self.ser.readline()
                line = raw.decode('utf-8', errors='ignore').strip()

                if not line:
                    continue

                parsed = self.parse_uwb_line(line)
                if parsed is None:
                    if self.verbose_rx:
                        self.get_logger().info(f'UWB RAW -> {line}')
                    continue

                a0, a1 = parsed

                self.pub_a0.publish(Float64(data=a0))
                self.pub_a1.publish(Float64(data=a1))

                if self.verbose_rx:
                    self.get_logger().info(f'UWB RX -> a0={a0:.3f}, a1={a1:.3f}')

        except Exception as e:
            self.get_logger().error(f'UWB serial read error: {e}')

    def destroy_node(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UwbSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
