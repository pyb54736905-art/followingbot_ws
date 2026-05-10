#!/usr/bin/env python3
"""
DWA Controller Node
-------------------
입력:
  /scan            (sensor_msgs/LaserScan)  - RPLIDAR A1M8
  /uwb_theta       (std_msgs/Float64)       - UWB 삼각측량 각도 (rad, uwb_follower 출력)
  /uwb_avg_dist    (std_msgs/Float64)       - UWB 평균 거리 (m, uwb_follower 출력)
  /measured_speed  (std_msgs/Float64)       - 현재 속도 (m/s, rpm_direct_serial_bridge 출력)

출력:
  /target_speed    (std_msgs/Float64)       - 목표 속도 (m/s)
  /target_steer    (std_msgs/Float64)       - 목표 조향각 (rad)
  /dwa_state       (std_msgs/String)        - 상태: DWA_OK / DWA_BLOCKED / STOP / NO_GOAL / NO_SCAN

알고리즘:
  Dynamic Window Approach (Ackermann 운동학 적용)
  - 속도/조향 샘플 공간을 동적 윈도우로 제한
  - 각 (v, δ) 조합에 대해 Ackermann 궤적 시뮬레이션
  - LiDAR 장애물과의 충돌 검사
  - heading(UWB 방향) + clearance(장애물 여유) + speed 기반 점수화
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, String


class DWAControllerNode(Node):
    def __init__(self):
        super().__init__('dwa_controller_node')

        # ── 로봇 기구학 ─────────────────────────────────────────────
        self.declare_parameter('wheelbase_m', 0.48)
        self.declare_parameter('max_speed_mps', 0.45)
        self.declare_parameter('max_steer_rad', 0.40)
        self.declare_parameter('robot_radius_m', 0.25)

        # ── 동적 윈도우 ──────────────────────────────────────────────
        self.declare_parameter('max_accel_mps2', 0.30)
        self.declare_parameter('max_steer_rate_rads', 0.80)
        self.declare_parameter('dw_time_s', 0.5)

        # ── 궤적 시뮬레이션 ──────────────────────────────────────────
        self.declare_parameter('predict_time_s', 2.0)
        self.declare_parameter('dt_sim_s', 0.1)

        # ── 샘플링 해상도 ────────────────────────────────────────────
        self.declare_parameter('n_v_samples', 5)
        self.declare_parameter('n_steer_samples', 11)

        # ── 점수 가중치 ──────────────────────────────────────────────
        self.declare_parameter('w_heading', 0.5)
        self.declare_parameter('w_clearance', 0.3)
        self.declare_parameter('w_speed', 0.2)

        # ── 동작 설정 ────────────────────────────────────────────────
        self.declare_parameter('stop_distance_m', 0.50)
        self.declare_parameter('slow_distance_m', 1.20)
        self.declare_parameter('min_speed_mps', 0.10)
        self.declare_parameter('loop_hz', 20.0)
        self.declare_parameter('data_timeout_s', 1.0)
        self.declare_parameter('obstacle_range_max_m', 3.5)
        # 로봇 자체 프레임이 LiDAR에 잡히는 경우 이 거리 이하 포인트 무시
        self.declare_parameter('obstacle_range_min_m', 0.0)

        # uwb_follower의 theta 부호 규칙 보정
        # 현재 설정: theta > 0 = 타겟이 오른쪽 → 오른쪽으로 조향(음수)
        # goal_theta_sign = -1.0 로 반전하여 DWA 방향 정렬
        self.declare_parameter('goal_theta_sign', -1.0)

        # 태그(사람) 제외 구역 ─────────────────────────────────────────
        # LiDAR가 UWB 태그 착용자(사람)를 장애물로 오인하지 않도록
        # UWB 타겟 방향 ±half_angle, 타겟 거리 + margin 이내 포인트 제외
        self.declare_parameter('tag_exclusion_half_angle_deg', 30.0)
        self.declare_parameter('tag_exclusion_dist_margin_m', 0.5)

        # LiDAR 없이도 동작 허용 (False: LiDAR 필수, True: 없으면 장애물 없음으로 간주)
        self.declare_parameter('scan_required', True)

        self._load_params()

        # ── 내부 상태 ────────────────────────────────────────────────
        self.scan: LaserScan | None = None
        self.goal_theta = 0.0
        self.goal_dist = 0.0
        self.current_speed = 0.0
        self.current_steer = 0.0
        self.last_scan_t = None
        self.last_goal_t = None

        # ── 구독 ─────────────────────────────────────────────────────
        self.create_subscription(LaserScan, '/scan',           self._on_scan,  10)
        self.create_subscription(Float64,   '/uwb_theta',      self._on_theta, 10)
        self.create_subscription(Float64,   '/uwb_avg_dist',   self._on_dist,  10)
        self.create_subscription(Float64,   '/measured_speed', self._on_speed, 10)

        # ── 발행 ─────────────────────────────────────────────────────
        self.pub_speed = self.create_publisher(Float64, '/target_speed', 10)
        self.pub_steer = self.create_publisher(Float64, '/target_steer', 10)
        self.pub_state = self.create_publisher(String,  '/dwa_state',    10)

        self.create_timer(1.0 / self.loop_hz, self._loop)
        self.get_logger().info(
            f'dwa_controller_node started | '
            f'max_v={self.max_speed_mps} max_steer={self.max_steer_rad} '
            f'r={self.robot_radius_m} predict={self.predict_time_s}s'
        )

    # ── 파라미터 로드 ────────────────────────────────────────────────
    def _load_params(self):
        g = self.get_parameter
        self.wheelbase_m      = float(g('wheelbase_m').value)
        self.max_speed_mps    = float(g('max_speed_mps').value)
        self.max_steer_rad    = float(g('max_steer_rad').value)
        self.robot_radius_m   = float(g('robot_radius_m').value)
        self.max_accel_mps2   = float(g('max_accel_mps2').value)
        self.max_steer_rate   = float(g('max_steer_rate_rads').value)
        self.dw_time_s        = float(g('dw_time_s').value)
        self.predict_time_s   = float(g('predict_time_s').value)
        self.dt_sim_s         = float(g('dt_sim_s').value)
        self.n_v_samples      = int(g('n_v_samples').value)
        self.n_steer_samples  = int(g('n_steer_samples').value)
        self.w_heading        = float(g('w_heading').value)
        self.w_clearance      = float(g('w_clearance').value)
        self.w_speed          = float(g('w_speed').value)
        self.stop_distance_m  = float(g('stop_distance_m').value)
        self.slow_distance_m  = float(g('slow_distance_m').value)
        self.min_speed_mps    = float(g('min_speed_mps').value)
        self.loop_hz          = float(g('loop_hz').value)
        self.data_timeout_s   = float(g('data_timeout_s').value)
        self.obs_range_max    = float(g('obstacle_range_max_m').value)
        self.obs_range_min    = float(g('obstacle_range_min_m').value)
        self.goal_theta_sign  = float(g('goal_theta_sign').value)
        self.tag_excl_half    = math.radians(float(g('tag_exclusion_half_angle_deg').value))
        self.tag_excl_margin  = float(g('tag_exclusion_dist_margin_m').value)
        self.scan_required    = bool(g('scan_required').value)

    # ── 콜백 ─────────────────────────────────────────────────────────
    def _on_scan(self, msg: LaserScan):
        self.scan = msg
        self.last_scan_t = self.get_clock().now()

    def _on_theta(self, msg: Float64):
        self.goal_theta = float(msg.data)
        self.last_goal_t = self.get_clock().now()

    def _on_dist(self, msg: Float64):
        self.goal_dist = float(msg.data)

    def _on_speed(self, msg: Float64):
        self.current_speed = max(0.0, float(msg.data))

    # ── 유틸리티 ─────────────────────────────────────────────────────
    def _fresh(self, t) -> bool:
        if t is None:
            return False
        return (self.get_clock().now() - t).nanoseconds * 1e-9 < self.data_timeout_s

    def _publish(self, v: float, s: float, state: str):
        self.pub_speed.publish(Float64(data=float(v)))
        self.pub_steer.publish(Float64(data=float(s)))
        self.pub_state.publish(String(data=state))

    # ── 각도 정규화 ──────────────────────────────────────────────────
    @staticmethod
    def _wrap(a: float) -> float:
        while a >  math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a

    # ── LiDAR → 장애물 좌표 변환 (태그 착용자 제외) ───────────────────
    def _get_obstacles(self) -> np.ndarray:
        """
        LaserScan → (N, 2) 장애물 좌표 배열 (로봇 좌표계, m)

        UWB 타겟 방향의 LiDAR 포인트는 제외:
          - LiDAR 각도 기준: 로봇 전방 = 0, 왼쪽 = +, 오른쪽 = -
          - UWB theta: 오른쪽 양수 → LiDAR 기준 -goal_theta로 변환
          - 타겟 방향 ±tag_excl_half rad, 타겟 거리 + margin 이내 제외
        """
        scan = self.scan

        # UWB 타겟이 LiDAR 좌표계에서 위치하는 각도
        # uwb theta: 오른쪽 양수 / LiDAR: 왼쪽 양수 → 부호 반전
        target_lidar_angle = self._wrap(-self.goal_theta)
        target_dist_limit  = self.goal_dist + self.tag_excl_margin

        pts = []
        angle = scan.angle_min
        for r in scan.ranges:
            if self.obs_range_min < r < min(scan.range_max, self.obs_range_max):
                angle_diff = abs(self._wrap(angle - target_lidar_angle))
                # 타겟 방향 ±half_angle 이내이고 타겟 거리 근처 → 사람 몸체 → 제외
                if angle_diff < self.tag_excl_half and r < target_dist_limit:
                    pass
                else:
                    pts.append((r * math.cos(angle), r * math.sin(angle)))
            angle += scan.angle_increment
        return np.array(pts, dtype=np.float64) if pts else np.empty((0, 2))

    # ── Ackermann 궤적 시뮬레이션 ────────────────────────────────────
    def _simulate(self, v: float, delta: float) -> np.ndarray:
        """
        Ackermann 운동학으로 궤적 계산
        반환: (steps+1, 3) 배열 [x, y, theta]
        """
        steps = max(1, int(self.predict_time_s / self.dt_sim_s))
        traj = np.zeros((steps + 1, 3))
        x = y = th = 0.0
        dt = self.dt_sim_s
        L = self.wheelbase_m
        for i in range(1, steps + 1):
            x  += v * math.cos(th) * dt
            y  += v * math.sin(th) * dt
            th += v * math.tan(delta) / L * dt
            traj[i] = (x, y, th)
        return traj

    # ── 궤적-장애물 최소 거리 (벡터화) ───────────────────────────────
    def _min_dist(self, traj: np.ndarray, obs: np.ndarray) -> float:
        """
        궤적의 모든 점과 장애물 점들 사이의 최소 거리
        numpy 브로드캐스팅으로 (N×M) 일괄 계산
        """
        if len(obs) == 0:
            return float('inf')
        # traj[:,0:1]: (N,1), obs[:,0]: (M,) → broadcast → (N,M)
        dx = traj[:, 0:1] - obs[:, 0]
        dy = traj[:, 1:2] - obs[:, 1]
        return float(np.sqrt(dx * dx + dy * dy).min())

    # ── UWB 거리 기반 속도 상한 ──────────────────────────────────────
    def _dist_to_speed_cap(self, dist: float) -> float:
        """
        uwb_follower 와 동일한 감속 구간 기준으로 속도 상한 계산.
        DWA 모드에서도 사람과의 거리에 따라 적절히 감속.
        """
        if dist <= self.stop_distance_m:
            return 0.0
        if dist >= self.slow_distance_m:
            return self.max_speed_mps
        ratio = ((dist - self.stop_distance_m)
                 / (self.slow_distance_m - self.stop_distance_m))
        return self.min_speed_mps + ratio * (self.max_speed_mps - self.min_speed_mps)

    # ── DWA 핵심 알고리즘 ────────────────────────────────────────────
    def _dwa(self, obs: np.ndarray, speed_cap: float):
        """
        동적 윈도우 내 (v, δ) 샘플링 → 최적 명령 반환
        반환: (best_v, best_steer) 또는 장애물 회피 불가 시 (0.0, 0.0)
        """
        dw = self.dw_time_s

        # 동적 윈도우: 현재 속도/조향 + 가속도 한계로 제한
        # speed_cap: UWB 거리 기반 상한 적용
        v_hi = min(speed_cap,  self.current_speed + self.max_accel_mps2 * dw)
        v_lo = max(0.0,                 self.current_speed - self.max_accel_mps2 * dw)
        s_hi = min(self.max_steer_rad,  self.current_steer + self.max_steer_rate  * dw)
        s_lo = max(-self.max_steer_rad, self.current_steer - self.max_steer_rate  * dw)

        # uwb_follower의 theta 부호 → DWA 방향 좌표계로 변환
        goal_dir = self.goal_theta_sign * self.goal_theta

        best_score = -float('inf')
        best_v = 0.0
        best_s = 0.0
        found = False

        for v in np.linspace(v_lo, v_hi, self.n_v_samples):
            for s in np.linspace(s_lo, s_hi, self.n_steer_samples):
                traj = self._simulate(v, s)
                md = self._min_dist(traj, obs)

                # 충돌 → 제외
                if md < self.robot_radius_m:
                    continue

                # 점수 계산
                # heading: 궤적 끝점에서 목표에 얼마나 가까워지는가 (goal approach)
                # cos(goal_dir - final_heading) 방식은 회피 중 heading이 틀어지면
                # 직진을 선호해 회피를 되돌리는 오동작 발생 → goal 거리 감소량으로 대체
                ep_x, ep_y = traj[-1, 0], traj[-1, 1]
                gx = self.goal_dist * math.cos(goal_dir)
                gy = self.goal_dist * math.sin(goal_dir)
                final_dist = math.sqrt((gx - ep_x) ** 2 + (gy - ep_y) ** 2)
                approach = self.goal_dist - final_dist
                max_approach = max(0.01, self.predict_time_s * self.max_speed_mps)
                head_score  = max(-1.0, min(1.0, approach / max_approach))
                # clearance: 장애물과 멀수록 높음 (최대 1.0)
                clear_score = min(md / 2.0, 1.0)
                # speed: 빠를수록 높음
                spd_score   = v / self.max_speed_mps if self.max_speed_mps > 0 else 0.0

                score = (self.w_heading   * head_score  +
                         self.w_clearance * clear_score +
                         self.w_speed     * spd_score)

                if score > best_score:
                    best_score = score
                    best_v = v
                    best_s = s
                    found = True

        return (best_v, best_s) if found else (0.0, 0.0)

    # ── 메인 제어 루프 ───────────────────────────────────────────────
    def _loop(self):
        # UWB 데이터 없음
        if not self._fresh(self.last_goal_t):
            self._publish(0.0, 0.0, 'NO_GOAL')
            return

        # LiDAR 데이터 없음
        if self.scan is None or not self._fresh(self.last_scan_t):
            if self.scan_required:
                self._publish(0.0, 0.0, 'NO_SCAN')
                return
            obs = np.empty((0, 2))  # scan_required=False → 장애물 없음으로 간주
        else:
            obs = self._get_obstacles()

        # 목표 거리 이내 → 정지
        if self.goal_dist <= self.stop_distance_m:
            self._publish(0.0, 0.0, 'STOP')
            self.current_steer = 0.0
            return

        # 거리 기반 속도 상한 계산 후 DWA 실행
        speed_cap = self._dist_to_speed_cap(self.goal_dist)
        best_v, best_s = self._dwa(obs, speed_cap)

        if best_v > 0.01:
            # 경로 찾음 → 최솟값 보장
            best_v = max(best_v, self.min_speed_mps)
            state = 'DWA_OK'
        else:
            # 경로 못 찾음 → 목표 방향으로 조향 + 저속 전진 시도
            # 이전 조향에서 max_steer_rate 속도로 목표 방향 최대 조향까지 접근 (점프 방지)
            goal_dir = self.goal_theta_sign * self.goal_theta
            target_s = float(self.max_steer_rad) * (1.0 if goal_dir >= 0 else -1.0)
            max_delta = self.max_steer_rate * (1.0 / self.loop_hz)
            diff = target_s - self.current_steer
            diff = max(-max_delta, min(max_delta, diff))
            best_s = self.current_steer + diff
            best_v = self.min_speed_mps
            state = 'DWA_BLOCKED'

        self.current_speed = best_v
        self.current_steer = best_s

        self._publish(best_v, best_s, state)

        self.get_logger().info(
            f'theta={math.degrees(self.goal_theta):.1f}deg '
            f'dist={self.goal_dist:.2f}m '
            f'obs_pts={len(obs)} '
            f'v={best_v:.3f} steer={best_s:.3f} [{state}]',
            throttle_duration_sec=0.5
        )


def main(args=None):
    rclpy.init(args=args)
    node = DWAControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish(0.0, 0.0, 'SHUTDOWN')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
