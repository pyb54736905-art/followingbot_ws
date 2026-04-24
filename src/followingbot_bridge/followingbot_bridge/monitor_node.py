#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
import curses
import threading
import time


class MonitorNode(Node):
    def __init__(self):
        super().__init__('monitor_node')
        self.data = {
            'uwb_a0':        ('UWB A0 거리',       'm',   None),
            'uwb_a1':        ('UWB A1 거리',       'm',   None),
            'uwb_avg_dist':  ('UWB 평균거리',      'm',   None),
            'uwb_lr_diff':   ('UWB 좌우차 (x)',    'm',   None),
            'uwb_theta':     ('UWB 각도 θ',        'rad', None),
            'uwb_zone':      ('상태 Zone',         '',    None),
            'target_speed':  ('목표 속도',         'm/s', None),
            'target_steer':  ('목표 조향',         'rad', None),
            'measured_speed':('실제 속도',         'm/s', None),
            'left_rpm':      ('좌측 RPM',          'rpm', None),
            'right_rpm':     ('우측 RPM',          'rpm', None),
        }
        self.lock = threading.Lock()

        def make_float_cb(key):
            def cb(msg):
                with self.lock:
                    label, unit, _ = self.data[key]
                    self.data[key] = (label, unit, msg.data)
            return cb

        def zone_cb(msg):
            with self.lock:
                label, unit, _ = self.data['uwb_zone']
                self.data['uwb_zone'] = (label, unit, msg.data)

        float_topics = [
            ('uwb_a0',         '/uwb_a0'),
            ('uwb_a1',         '/uwb_a1'),
            ('uwb_avg_dist',   '/uwb_avg_dist'),
            ('uwb_lr_diff',    '/uwb_lr_diff'),
            ('uwb_theta',      '/uwb_theta'),
            ('target_speed',   '/target_speed'),
            ('target_steer',   '/target_steer'),
            ('measured_speed', '/measured_speed'),
            ('left_rpm',       '/left_rpm'),
            ('right_rpm',      '/right_rpm'),
        ]
        for key, topic in float_topics:
            self.create_subscription(Float64, topic, make_float_cb(key), 10)
        self.create_subscription(String, '/uwb_position_zone', zone_cb, 10)


def draw(stdscr, node):
    curses.curs_set(0)
    curses.start_color()
    curses.init_pair(1, curses.COLOR_CYAN,   curses.COLOR_BLACK)
    curses.init_pair(2, curses.COLOR_GREEN,  curses.COLOR_BLACK)
    curses.init_pair(3, curses.COLOR_YELLOW, curses.COLOR_BLACK)
    curses.init_pair(4, curses.COLOR_RED,    curses.COLOR_BLACK)
    stdscr.nodelay(True)

    sections = [
        ('── UWB 센서 ──────────────────────────────', [
            'uwb_a0', 'uwb_a1', 'uwb_avg_dist', 'uwb_lr_diff', 'uwb_theta', 'uwb_zone',
        ]),
        ('── 제어 명령 ─────────────────────────────', [
            'target_speed', 'target_steer',
        ]),
        ('── 모터 피드백 ───────────────────────────', [
            'measured_speed', 'left_rpm', 'right_rpm',
        ]),
    ]

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        stdscr.erase()
        row = 0

        stdscr.addstr(row, 0, ' FollowingBot 실시간 모니터  (q: 종료)',
                      curses.color_pair(1) | curses.A_BOLD)
        row += 1
        stdscr.addstr(row, 0, f' {time.strftime("%H:%M:%S")}', curses.color_pair(3))
        row += 2

        with node.lock:
            snapshot = dict(node.data)

        for header, keys in sections:
            stdscr.addstr(row, 0, header, curses.color_pair(1))
            row += 1
            for key in keys:
                label, unit, val = snapshot[key]
                if val is None:
                    val_str = '-- (no data)'
                    color = curses.color_pair(3)
                elif isinstance(val, str):
                    val_str = val
                    color = curses.color_pair(4) if val in ('TIMEOUT', 'INVALID', 'INIT') else curses.color_pair(2)
                else:
                    val_str = f'{val:+.4f} {unit}'
                    color = curses.color_pair(2)

                stdscr.addstr(row, 2,  f'{label:<16}', curses.A_NORMAL)
                stdscr.addstr(row, 20, val_str, color | curses.A_BOLD)
                row += 1
            row += 1

        stdscr.refresh()
        ch = stdscr.getch()
        if ch == ord('q'):
            break


def main():
    rclpy.init()
    node = MonitorNode()
    try:
        curses.wrapper(draw, node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
