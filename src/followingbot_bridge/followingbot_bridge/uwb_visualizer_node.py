import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Duration

TRAIL_MAX = 200
CIRCLE_STEPS = 72
EMA_ALPHA = 0.15        # 낮을수록 더 부드럽게 (0.1~0.3 권장)
TRAIL_MIN_DIST = 0.04   # 4cm 이상 움직여야 trail 점 추가


def circle_points(cx, cy, r):
    pts = []
    for i in range(CIRCLE_STEPS + 1):
        a = 2.0 * math.pi * i / CIRCLE_STEPS
        pts.append(Point(x=cx + r * math.cos(a), y=cy + r * math.sin(a), z=0.0))
    return pts


def make_sphere(mid, ns, frame, stamp, lifetime, x, y, z, scale, r, g, b):
    m = Marker()
    m.header.frame_id = frame
    m.header.stamp = stamp
    m.ns = ns
    m.id = mid
    m.type = Marker.SPHERE
    m.action = Marker.ADD
    m.lifetime = lifetime
    m.pose.position = Point(x=x, y=y, z=z)
    m.pose.orientation.w = 1.0
    m.scale.x = m.scale.y = m.scale.z = scale
    m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
    return m


def make_text(mid, ns, frame, stamp, lifetime, x, y, z, size, text):
    m = Marker()
    m.header.frame_id = frame
    m.header.stamp = stamp
    m.ns = ns
    m.id = mid
    m.type = Marker.TEXT_VIEW_FACING
    m.action = Marker.ADD
    m.lifetime = lifetime
    m.pose.position = Point(x=x, y=y, z=z)
    m.pose.orientation.w = 1.0
    m.scale.z = size
    m.color.r = m.color.g = m.color.b = m.color.a = 1.0
    m.text = text
    return m


def make_line_strip(mid, ns, frame, stamp, lifetime, points, width, r, g, b, a):
    m = Marker()
    m.header.frame_id = frame
    m.header.stamp = stamp
    m.ns = ns
    m.id = mid
    m.type = Marker.LINE_STRIP
    m.action = Marker.ADD
    m.lifetime = lifetime
    m.scale.x = width
    m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, a
    m.points = points
    return m


class UwbVisualizerNode(Node):
    def __init__(self):
        super().__init__('uwb_visualizer_node')

        self.declare_parameter('anchor_spacing_m', 0.35)
        self.declare_parameter('x_bias_m', 0.0)
        self.declare_parameter('show_trail', True)
        self.declare_parameter('trail_timeout_s', 2.0)

        self.d = float(self.get_parameter('anchor_spacing_m').value)
        self.x_bias = float(self.get_parameter('x_bias_m').value)
        self.show_trail = bool(self.get_parameter('show_trail').value)
        self.trail_timeout_s = float(self.get_parameter('trail_timeout_s').value)

        self.a0 = None
        self.a1 = None
        self.trail: list[tuple[float, float, float]] = []
        self.x_ema: float | None = None  # EMA 스무딩된 태그 X
        self.y_ema: float | None = None  # EMA 스무딩된 태그 Y

        self.create_subscription(Float64, '/uwb_a0', lambda msg: setattr(self, 'a0', msg.data), 10)
        self.create_subscription(Float64, '/uwb_a1', lambda msg: setattr(self, 'a1', msg.data), 10)

        self.pub = self.create_publisher(MarkerArray, '/uwb_viz', 10)
        self.create_timer(0.05, self._publish)  # 20 Hz

    def _publish(self):
        ms = []
        now = self.get_clock().now().to_msg()
        frame = 'uwb_frame'
        inf = Duration(sec=0)        # 무한 (고정 마커)
        short = Duration(sec=0, nanosec=int(0.3e9))  # 300ms (동적 마커)

        d = self.d
        half_d = d / 2.0  # A0/A1 중점을 원점으로 — A0=(-half_d,0), A1=(+half_d,0)

        # ── 앵커 고정 마커 (중점 기준 배치) ──────────────────────────────
        ms.append(make_sphere(0, 'anchors', frame, now, inf,
                              -half_d, 0.0, 0.0, 0.10, 0.2, 0.4, 1.0))  # A0 파랑
        ms.append(make_text(1, 'labels', frame, now, inf,
                            -half_d, 0.0, 0.15, 0.07, 'A0'))
        ms.append(make_sphere(2, 'anchors', frame, now, inf,
                              half_d, 0.0, 0.0, 0.10, 1.0, 0.3, 0.2))    # A1 빨강
        ms.append(make_text(3, 'labels', frame, now, inf,
                            half_d, 0.0, 0.15, 0.07, 'A1'))

        if self.a0 is None or self.a1 is None:
            arr = MarkerArray()
            arr.markers = ms
            self.pub.publish(arr)
            return

        a0, a1 = max(0.01, self.a0), max(0.01, self.a1)

        # ── 거리 원 ───────────────────────────────────────────────────────
        ms.append(make_line_strip(4, 'circles', frame, now, short,
                                  circle_points(-half_d, 0.0, a0),
                                  0.012, 0.2, 0.4, 1.0, 0.55))  # A0 원 파랑
        ms.append(make_line_strip(5, 'circles', frame, now, short,
                                  circle_points(half_d, 0.0, a1),
                                  0.012, 1.0, 0.3, 0.2, 0.55))  # A1 원 빨강

        # ── 거리 수치 텍스트 ──────────────────────────────────────────────
        ms.append(make_text(10, 'dist_labels', frame, now, short,
                            -half_d + a0 * math.cos(math.pi * 0.25),
                            a0 * math.sin(math.pi * 0.25), 0.05,
                            0.07, f'a0={a0:.2f}m'))
        ms.append(make_text(11, 'dist_labels', frame, now, short,
                            half_d + a1 * math.cos(math.pi * 0.75),
                            a1 * math.sin(math.pi * 0.75), 0.05,
                            0.07, f'a1={a1:.2f}m'))

        # ── 태그 위치 추정 (삼변측량, 중점 원점 기준) ────────────────────
        if d > 1e-6:
            # A0 기준 x → 중점 기준으로 변환 (half_d 빼기)
            x_raw = (a0 * a0 - a1 * a1 + d * d) / (2.0 * d) - half_d
            x_raw -= self.x_bias
            avg = 0.5 * (a0 + a1)
            y_raw = math.sqrt(max(0.0, avg * avg - (x_raw + half_d) * (x_raw + half_d)))

            # EMA 스무딩 — 초기화 시 첫 값으로 세팅
            if self.x_ema is None:
                self.x_ema, self.y_ema = x_raw, y_raw
            else:
                self.x_ema = EMA_ALPHA * x_raw + (1.0 - EMA_ALPHA) * self.x_ema
                self.y_ema = EMA_ALPHA * y_raw + (1.0 - EMA_ALPHA) * self.y_ema

            sx, sy = self.x_ema, self.y_ema

            # 태그 구 (초록) — 스무딩된 위치 사용
            ms.append(make_sphere(6, 'tag', frame, now, short,
                                  sx, sy, 0.0, 0.12, 0.1, 0.9, 0.2))
            ms.append(make_text(7, 'labels', frame, now, short,
                                sx, sy, 0.18, 0.09,
                                f'Tag\n({sx:.2f}, {sy:.2f})m'))

            # 앵커→태그 선분
            ms.append(make_line_strip(8, 'lines', frame, now, short,
                                      [Point(x=-half_d, y=0.0, z=0.0), Point(x=sx, y=sy, z=0.0)],
                                      0.008, 0.4, 0.6, 1.0, 0.4))
            ms.append(make_line_strip(9, 'lines', frame, now, short,
                                      [Point(x=half_d, y=0.0, z=0.0), Point(x=sx, y=sy, z=0.0)],
                                      0.008, 1.0, 0.5, 0.3, 0.4))

            # 궤적 트레일 — 최소 이동 거리 초과 시에만 추가
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            if self.show_trail:
                # 오래된 점 삭제 (시간 기반)
                self.trail = [
                    (tx, ty, ts) for tx, ty, ts in self.trail
                    if (now_sec - ts) <= self.trail_timeout_s
                ]

                if not self.trail:
                    self.trail.append((sx, sy, now_sec))
                else:
                    lx, ly, _ = self.trail[-1]
                    dist = math.sqrt((sx - lx) ** 2 + (sy - ly) ** 2)
                    if dist >= TRAIL_MIN_DIST:
                        self.trail.append((sx, sy, now_sec))
                        if len(self.trail) > TRAIL_MAX:
                            self.trail.pop(0)

            if self.show_trail and len(self.trail) >= 2:
                ms.append(make_line_strip(
                    20, 'trail', frame, now, short,
                    [Point(x=px, y=py, z=0.0) for px, py, _ in self.trail],
                    0.015, 0.3, 1.0, 0.5, 0.85))

        arr = MarkerArray()
        arr.markers = ms
        self.pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = UwbVisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
