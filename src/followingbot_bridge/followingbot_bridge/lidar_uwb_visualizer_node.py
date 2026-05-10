"""
LiDAR + UWB 통합 시각화 노드

퍼블리시:
  /scan_classified   — MarkerArray  : 초록(태그 보유자) / 빨강(장애물)
  /scan_plot         — Image        : matplotlib 2D 스캐터 플롯 (5 Hz)
"""

import io
import math

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class LidarUwbVisualizerNode(Node):
    def __init__(self):
        super().__init__('lidar_uwb_visualizer_node')

        # ── 파라미터 ──────────────────────────────────────────────────────
        self.declare_parameter('tag_radius',  0.50)   # 태그 인식 반경 (m)
        self.declare_parameter('min_range',   0.10)   # LiDAR 최소 유효 거리
        self.declare_parameter('max_range',   3.50)   # LiDAR 최대 유효 거리
        self.declare_parameter('point_size',  0.04)   # RViz2 구 크기 (m)
        self.declare_parameter('theta_sign',  1.0)    # UWB theta 부호 (-1이면 반전)
        self.declare_parameter('plot_hz',     5.0)    # 플롯 갱신 주기
        self.declare_parameter('rotate_lidar_to_uwb_axes', True)  # 전방(+X laser) -> 전방(+Y uwb) 축 정렬
        self.declare_parameter('lidar_yaw_offset_deg', 0.0)  # 라이다 좌표계 yaw 보정(도)
        self.declare_parameter('visual_frame', 'uwb_frame')

        self.tag_radius  = float(self.get_parameter('tag_radius').value)
        self.min_range   = float(self.get_parameter('min_range').value)
        self.max_range   = float(self.get_parameter('max_range').value)
        self.point_size  = float(self.get_parameter('point_size').value)
        self.theta_sign  = float(self.get_parameter('theta_sign').value)
        self.rotate_to_uwb = bool(self.get_parameter('rotate_lidar_to_uwb_axes').value)
        self.lidar_yaw_offset_rad = math.radians(
            float(self.get_parameter('lidar_yaw_offset_deg').value)
        )
        self.visual_frame = str(self.get_parameter('visual_frame').value)

        # ── 상태 ──────────────────────────────────────────────────────────
        self.uwb_theta    = None
        self.uwb_avg_dist = None
        self._last_green: list[tuple[float, float]] = []
        self._last_red:   list[tuple[float, float]] = []

        # ── 구독 ──────────────────────────────────────────────────────────
        self.create_subscription(LaserScan, '/scan',          self._cb_scan,   10)
        self.create_subscription(Float64,   '/uwb_theta',
                                  lambda m: setattr(self, 'uwb_theta',    m.data), 10)
        self.create_subscription(Float64,   '/uwb_avg_dist',
                                  lambda m: setattr(self, 'uwb_avg_dist', m.data), 10)

        # ── 퍼블리셔 ──────────────────────────────────────────────────────
        self.pub_markers = self.create_publisher(MarkerArray, '/scan_classified', 10)
        self.pub_image   = self.create_publisher(Image,       '/scan_plot',       5)

        plot_period = 1.0 / float(self.get_parameter('plot_hz').value)
        self.create_timer(plot_period, self._publish_plot)

    # ── LaserScan 콜백 ────────────────────────────────────────────────────
    def _cb_scan(self, scan: LaserScan):
        frame  = scan.header.frame_id
        stamp  = scan.header.stamp
        short  = Duration(sec=0, nanosec=int(0.3e9))

        # 태그 예상 위치 (laser 프레임 기준)
        tag_x = tag_y = None
        if self.uwb_theta is not None and self.uwb_avg_dist is not None:
            # uwb_follower의 theta 정의: theta=atan2(lateral_x, forward_y)
            # 시각화 축(uwb): x=lateral, y=forward
            t = self.theta_sign * self.uwb_theta
            d = self.uwb_avg_dist
            tag_x = d * math.sin(t)
            tag_y = d * math.cos(t)

        green_pts, red_pts = [], []
        green_xy,  red_xy  = [], []

        angle = scan.angle_min
        for r in scan.ranges:
            if self.min_range <= r <= self.max_range and math.isfinite(r):
                lx = r * math.cos(angle)
                ly = r * math.sin(angle)
                if self.rotate_to_uwb:
                    # laser: x=forward, y=left  -> uwb: x=right(lateral), y=forward
                    px = -ly
                    py = lx
                else:
                    px = lx
                    py = ly

                # 추가 yaw 보정 (예: 180도 반전)
                if abs(self.lidar_yaw_offset_rad) > 1e-9:
                    c = math.cos(self.lidar_yaw_offset_rad)
                    s = math.sin(self.lidar_yaw_offset_rad)
                    px, py = (px * c - py * s), (px * s + py * c)

                if tag_x is not None and math.hypot(px - tag_x, py - tag_y) <= self.tag_radius:
                    green_pts.append(Point(x=px, y=py, z=0.0))
                    green_xy.append((px, py))
                else:
                    red_pts.append(Point(x=px, y=py, z=0.0))
                    red_xy.append((px, py))
            angle += scan.angle_increment

        # 플롯용 캐시 갱신
        self._last_green = green_xy
        self._last_red   = red_xy

        markers = []

        # ── 초록: 태그 보유자 ─────────────────────────────────────────────
        if green_pts:
            mg = Marker()
            mg.header.frame_id = self.visual_frame
            mg.header.stamp    = stamp
            mg.ns, mg.id       = 'scan_tag', 0
            mg.type            = Marker.SPHERE_LIST
            mg.action          = Marker.ADD
            mg.lifetime        = short
            mg.scale.x = mg.scale.y = mg.scale.z = self.point_size * 1.5
            mg.color.r, mg.color.g, mg.color.b, mg.color.a = 0.0, 1.0, 0.3, 0.95
            mg.points          = green_pts
            markers.append(mg)

        # ── 빨강: 일반 장애물 ─────────────────────────────────────────────
        if red_pts:
            mr = Marker()
            mr.header.frame_id = self.visual_frame
            mr.header.stamp    = stamp
            mr.ns, mr.id       = 'scan_obstacle', 1
            mr.type            = Marker.SPHERE_LIST
            mr.action          = Marker.ADD
            mr.lifetime        = short
            mr.scale.x = mr.scale.y = mr.scale.z = self.point_size
            mr.color.r, mr.color.g, mr.color.b, mr.color.a = 1.0, 0.15, 0.15, 0.85
            mr.points          = red_pts
            markers.append(mr)

        # ── 태그 방향 화살표 ──────────────────────────────────────────────
        if tag_x is not None:
            arr = Marker()
            arr.header.frame_id = self.visual_frame
            arr.header.stamp    = stamp
            arr.ns, arr.id      = 'tag_arrow', 2
            arr.type            = Marker.ARROW
            arr.action          = Marker.ADD
            arr.lifetime        = short
            arr.scale.x = 0.03   # shaft 직경
            arr.scale.y = 0.06   # head 직경
            arr.scale.z = 0.08   # head 길이
            arr.color.r, arr.color.g, arr.color.b, arr.color.a = 0.2, 1.0, 0.3, 0.9
            arr.points = [Point(x=0.0, y=0.0, z=0.0),
                          Point(x=tag_x, y=tag_y, z=0.0)]
            markers.append(arr)

            # 태그 인식 반경 원
            circle_pts = []
            for i in range(73):
                a = 2.0 * math.pi * i / 72
                circle_pts.append(
                    Point(x=tag_x + self.tag_radius * math.cos(a),
                          y=tag_y + self.tag_radius * math.sin(a),
                          z=0.0))
            mc = Marker()
            mc.header.frame_id = self.visual_frame
            mc.header.stamp    = stamp
            mc.ns, mc.id       = 'tag_circle', 3
            mc.type            = Marker.LINE_STRIP
            mc.action          = Marker.ADD
            mc.lifetime        = short
            mc.scale.x         = 0.015
            mc.color.r, mc.color.g, mc.color.b, mc.color.a = 0.2, 1.0, 0.3, 0.5
            mc.points          = circle_pts
            markers.append(mc)

        ma = MarkerArray()
        ma.markers = markers
        self.pub_markers.publish(ma)

    # ── matplotlib 플롯 퍼블리셔 (5 Hz) ──────────────────────────────────
    def _publish_plot(self):
        if not self._last_red and not self._last_green:
            return

        fig, ax = plt.subplots(figsize=(5, 5), dpi=100)
        fig.patch.set_facecolor('#1e1e1e')
        ax.set_facecolor('#1e1e1e')
        ax.tick_params(colors='white')
        for spine in ax.spines.values():
            spine.set_edgecolor('#444444')
        ax.grid(True, color='#333333', linewidth=0.5)
        ax.set_aspect('equal')
        ax.set_xlim(-4, 4)
        ax.set_ylim(-1, 5)
        ax.set_xlabel('Y (left+) [m]', color='white', fontsize=8)
        ax.set_ylabel('X (forward) [m]', color='white', fontsize=8)
        ax.set_title('LiDAR 분류 (초록=태그, 빨강=장애물)', color='white', fontsize=9)

        # 로봇 위치 (원점)
        ax.plot(0, 0, 's', color='#ffaa00', markersize=8, zorder=5)
        ax.annotate('Robot', (0, 0), textcoords='offset points',
                    xytext=(5, -12), color='#ffaa00', fontsize=7)

        # 빨강 장애물 — x=forward, y=lateral 순으로 swap (matplotlib x축=y방향)
        if self._last_red:
            rx = [p[0] for p in self._last_red]   # lateral
            ry = [p[1] for p in self._last_red]   # forward
            ax.scatter(rx, ry, c='#ff3030', s=4, alpha=0.7, zorder=3)

        # 초록 태그 포인트
        if self._last_green:
            gx = [p[0] for p in self._last_green]
            gy = [p[1] for p in self._last_green]
            ax.scatter(gx, gy, c='#00ff66', s=8, alpha=0.9, zorder=4)

        # 태그 인식 반경 원
        if self.uwb_theta is not None and self.uwb_avg_dist is not None:
            t = self.theta_sign * self.uwb_theta
            d = self.uwb_avg_dist
            tx = d * math.sin(t)
            ty = d * math.cos(t)
            circle = mpatches.Circle(
                (tx, ty), self.tag_radius,
                fill=True, facecolor='#00ff6622', edgecolor='#00ff66',
                linewidth=1.5, linestyle='--', zorder=2)
            ax.add_patch(circle)
            ax.annotate(f'{d:.2f}m', (tx, ty),
                        color='#00ff66', fontsize=8,
                        ha='center', va='bottom')

        legend = [
            mpatches.Patch(color='#00ff66', label='태그 보유자'),
            mpatches.Patch(color='#ff3030', label='장애물'),
        ]
        ax.legend(handles=legend, loc='upper right',
                  facecolor='#2a2a2a', labelcolor='white', fontsize=7)

        # Figure → numpy → Image msg
        buf = io.BytesIO()
        fig.savefig(buf, format='raw', dpi=100)
        plt.close(fig)
        buf.seek(0)
        data = np.frombuffer(buf.getvalue(), dtype=np.uint8)
        h, w = 500, 500
        data = data.reshape((h, w, 4))   # RGBA
        rgb  = data[:, :, :3].tobytes()

        img_msg = Image()
        img_msg.header.stamp    = self.get_clock().now().to_msg()
        img_msg.header.frame_id = self.visual_frame
        img_msg.height          = h
        img_msg.width           = w
        img_msg.encoding        = 'rgb8'
        img_msg.is_bigendian    = 0
        img_msg.step            = w * 3
        img_msg.data            = list(rgb)
        self.pub_image.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarUwbVisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
