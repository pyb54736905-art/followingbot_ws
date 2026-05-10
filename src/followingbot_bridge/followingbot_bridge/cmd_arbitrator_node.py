import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, String


class CmdArbitratorNode(Node):
    """
    Stanley(car_control_node) 출력과 DWA(dwa_controller_node) 출력을 중재.

    [모드 전환]
    전방 장애물 없음 (> obstacle_switch_m):
        car_control 출력 사용 → Stanley 조향 + PID 속도 (정밀 팔로잉)
    전방 장애물 근접 (<= obstacle_switch_m):
        DWA 출력 사용 → 장애물 회피 조향 + 속도
    히스테리시스: obstacle_clear_m 이상 복귀 시 Stanley 모드로 전환.

    [LiDAR 안전 기능] (우선순위 순)
    1. 비상 정지 구역: 전방 좁은 cone 내 극근거리 장애물 → 즉시 정지
    2. 끼임 감지: 속도 명령 있지만 실측 속도 없으면 stuck_timeout_s 후 정지
       - 후진 불가(서보 링크 구조 제약) → 정지 후 사람이 접근해 UWB stop 조건
         충족되는 시점에 자동 해제됨
    3. 협로 속도 감속: 좌/우 측면 여유 좁으면 속도 비율 감소

    [태그 착용자 제외]
    UWB 타겟 방향 ±tag_exclusion_half_angle_deg, 타겟 거리 + margin 이내
    LiDAR 포인트를 모드 전환 판정 및 협로 감속에서 제외.
    비상 정지는 착용자 포함 모든 장애물에 반응 (안전 우선).
    """

    def __init__(self):
        super().__init__('cmd_arbitrator_node')

        # ── 기존 파라미터 ────────────────────────────────────────────
        self.declare_parameter('obstacle_switch_m', 1.5)
        self.declare_parameter('obstacle_clear_m', 2.0)
        self.declare_parameter('forward_cone_deg', 60.0)
        self.declare_parameter('loop_hz', 20.0)
        self.declare_parameter('data_timeout_s', 0.5)
        self.declare_parameter('tag_exclusion_half_angle_deg', 30.0)
        self.declare_parameter('tag_exclusion_dist_margin_m', 0.5)
        # 로봇 자체 프레임이 LiDAR에 잡히는 경우 이 거리 이하 포인트 무시
        self.declare_parameter('scan_range_min_m', 0.0)
        # 테스트용 안전기능 토글
        self.declare_parameter('enable_emstop', True)
        self.declare_parameter('enable_stuck_protection', True)
        self.declare_parameter('enable_narrow_slowdown', True)
        # False 시 LiDAR 없어도 UWB 명령으로 주행 (LiDAR 고장 시 임시 사용)
        self.declare_parameter('require_scan', True)

        # ── 비상 정지 파라미터 ───────────────────────────────────────
        self.declare_parameter('emstop_dist_m', 0.25)
        # 비상 정지 감지 cone 폭 (좌우 합산 deg) - 좁게 유지해야 오인식 방지
        self.declare_parameter('emstop_cone_deg', 30.0)
        # 로봇 자체 프레임 제외용 emstop 전용 최솟값 (scan_range_min_m 보다 크게 설정)
        self.declare_parameter('emstop_range_min_m', 0.0)

        # ── 협로 속도 감속 파라미터 ──────────────────────────────────
        # 좌/우 측면 여유가 이 이하로 좁아지면 속도 감소 시작
        self.declare_parameter('narrow_warn_m', 0.50)
        # 이 이하이면 완전 정지
        self.declare_parameter('narrow_stop_m', 0.20)
        # 측면 스캔 cone 폭 (좌우 각각 ±half, deg) — ±90° 기준
        self.declare_parameter('side_cone_deg', 40.0)

        # ── 끼임 감지 파라미터 ───────────────────────────────────────
        # 이 이상 속도 명령 = "움직여야 함"
        self.declare_parameter('stuck_cmd_threshold_mps', 0.08)
        # 이 이하 실측 속도 = "정지 상태"
        self.declare_parameter('stuck_meas_threshold_mps', 0.04)
        # 이 시간 이상 지속되면 끼임 판정
        self.declare_parameter('stuck_timeout_s', 3.0)

        g = self.get_parameter
        self.obstacle_switch_m = float(g('obstacle_switch_m').value)
        self.obstacle_clear_m  = float(g('obstacle_clear_m').value)
        self.forward_cone_half = math.radians(float(g('forward_cone_deg').value) * 0.5)
        loop_hz                = float(g('loop_hz').value)
        self.data_timeout_s    = float(g('data_timeout_s').value)
        self.tag_excl_half     = math.radians(float(g('tag_exclusion_half_angle_deg').value))
        self.tag_excl_margin   = float(g('tag_exclusion_dist_margin_m').value)
        self.scan_range_min    = float(g('scan_range_min_m').value)
        self.enable_emstop     = bool(g('enable_emstop').value)
        self.enable_stuck      = bool(g('enable_stuck_protection').value)
        self.enable_narrow     = bool(g('enable_narrow_slowdown').value)
        self.require_scan      = bool(g('require_scan').value)

        self.emstop_dist_m    = float(g('emstop_dist_m').value)
        self.emstop_cone_half = math.radians(float(g('emstop_cone_deg').value) * 0.5)
        self.emstop_range_min = float(g('emstop_range_min_m').value)

        self.narrow_warn_m  = float(g('narrow_warn_m').value)
        self.narrow_stop_m  = float(g('narrow_stop_m').value)
        self.side_cone_half = math.radians(float(g('side_cone_deg').value) * 0.5)

        self.stuck_cmd_thr  = float(g('stuck_cmd_threshold_mps').value)
        self.stuck_meas_thr = float(g('stuck_meas_threshold_mps').value)
        self.stuck_timeout  = float(g('stuck_timeout_s').value)

        # ── 내부 상태 ────────────────────────────────────────────────
        self.scan: LaserScan | None = None
        self.nominal_speed  = 0.0
        self.nominal_steer  = 0.0
        self.dwa_speed      = 0.0
        self.dwa_steer      = 0.0
        self.dwa_state      = 'NO_GOAL'
        self.uwb_theta      = 0.0
        self.uwb_dist       = 0.0
        self.measured_speed = 0.0

        self.last_scan_t    = None
        self.last_nominal_t = None
        self.last_dwa_t     = None

        self.dwa_mode       = False
        self.stuck_since: float | None = None
        # 끼임 래치: cmd_speed가 0으로 내려가야 해제 (사람이 가까워져 UWB stop)
        self.stuck_latched  = False

        # ── 구독 ─────────────────────────────────────────────────────
        self.create_subscription(LaserScan, '/scan',           self._on_scan,           10)
        self.create_subscription(Float64,   '/nominal_speed',  self._on_nominal_speed,  10)
        self.create_subscription(Float64,   '/nominal_steer',  self._on_nominal_steer,  10)
        self.create_subscription(Float64,   '/dwa_speed',      self._on_dwa_speed,      10)
        self.create_subscription(Float64,   '/dwa_steer',      self._on_dwa_steer,      10)
        self.create_subscription(String,    '/dwa_state',      self._on_dwa_state,      10)
        self.create_subscription(Float64,   '/uwb_theta',      self._on_uwb_theta,      10)
        self.create_subscription(Float64,   '/uwb_avg_dist',   self._on_uwb_dist,       10)
        self.create_subscription(Float64,   '/measured_speed', self._on_measured_speed, 10)

        # ── 발행 ─────────────────────────────────────────────────────
        self.pub_speed  = self.create_publisher(Float64, '/target_speed',    10)
        self.pub_steer  = self.create_publisher(Float64, '/target_steer',    10)
        self.pub_mode   = self.create_publisher(String,  '/arbitrator_mode', 10)
        self.pub_safety = self.create_publisher(String,  '/safety_state',    10)

        self.create_timer(1.0 / loop_hz, self._loop)
        self.get_logger().info(
            f'cmd_arbitrator_node started | '
            f'switch={self.obstacle_switch_m}m clear={self.obstacle_clear_m}m '
            f'emstop={self.emstop_dist_m}m '
            f'narrow_warn={self.narrow_warn_m}m narrow_stop={self.narrow_stop_m}m '
            f'stuck_timeout={self.stuck_timeout}s '
            f'flags(emstop={self.enable_emstop}, '
            f'stuck={self.enable_stuck}, narrow={self.enable_narrow})'
        )

    # ── 콜백 ─────────────────────────────────────────────────────────
    def _on_scan(self, msg: LaserScan):
        self.scan = msg
        self.last_scan_t = self.get_clock().now()

    def _on_nominal_speed(self, msg: Float64):
        self.nominal_speed = float(msg.data)
        self.last_nominal_t = self.get_clock().now()

    def _on_nominal_steer(self, msg: Float64):
        self.nominal_steer = float(msg.data)

    def _on_dwa_speed(self, msg: Float64):
        self.dwa_speed = float(msg.data)
        self.last_dwa_t = self.get_clock().now()

    def _on_dwa_steer(self, msg: Float64):
        self.dwa_steer = float(msg.data)

    def _on_dwa_state(self, msg: String):
        self.dwa_state = msg.data

    def _on_uwb_theta(self, msg: Float64):
        self.uwb_theta = float(msg.data)

    def _on_uwb_dist(self, msg: Float64):
        self.uwb_dist = float(msg.data)

    def _on_measured_speed(self, msg: Float64):
        self.measured_speed = max(0.0, float(msg.data))

    # ── 유틸리티 ─────────────────────────────────────────────────────
    def _fresh(self, t) -> bool:
        if t is None:
            return False
        return (self.get_clock().now() - t).nanoseconds * 1e-9 < self.data_timeout_s

    @staticmethod
    def _wrap(a: float) -> float:
        while a >  math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a

    def _publish(self, speed: float, steer: float, mode: str, safety: str = 'SAFE'):
        self.pub_speed.publish(Float64(data=float(speed)))
        self.pub_steer.publish(Float64(data=float(steer)))
        self.pub_mode.publish(String(data=mode))
        self.pub_safety.publish(String(data=safety))

    # ── 태그 착용자 제외 판정 ─────────────────────────────────────────
    def _is_tag(self, angle: float, r: float) -> bool:
        target_lidar_angle = self._wrap(-self.uwb_theta)
        angle_diff = abs(self._wrap(angle - target_lidar_angle))
        return (angle_diff < self.tag_excl_half
                and r < self.uwb_dist + self.tag_excl_margin)

    # ── 전방 최소 거리 (Stanley ↔ DWA 모드 전환용) ────────────────────
    def _forward_min_dist(self) -> float:
        scan = self.scan
        min_dist = float('inf')
        angle = scan.angle_min
        for r in scan.ranges:
            if self.scan_range_min < r < scan.range_max:
                if abs(angle) < self.forward_cone_half and not self._is_tag(angle, r):
                    min_dist = min(min_dist, r)
            angle += scan.angle_increment
        return min_dist

    # ── 1. 비상 정지 감지 ─────────────────────────────────────────────
    def _check_emstop(self) -> bool:
        """
        전방 좁은 cone(±emstop_cone_half) 내 emstop_dist_m 이하 장애물 → True.
        태그 착용자 포함 모든 장애물에 반응 (안전 최우선).
        """
        scan = self.scan
        range_min = max(self.scan_range_min, self.emstop_range_min)
        angle = scan.angle_min
        for r in scan.ranges:
            if range_min < r < scan.range_max:
                if abs(angle) < self.emstop_cone_half and r < self.emstop_dist_m:
                    return True
            angle += scan.angle_increment
        return False

    # ── 2. 끼임 감지 ─────────────────────────────────────────────────
    def _check_stuck(self, cmd_speed: float) -> bool:
        """
        cmd_speed > stuck_cmd_thr 이지만 measured_speed < stuck_meas_thr 인 상태가
        stuck_timeout_s 이상 지속 → 끼임 판정(True).

        래치 동작: 판정 후 cmd_speed가 0으로 내려올 때까지 유지.
        후진 불가 제약으로 인해 강제 해제 수단이 없으므로,
        사람이 가까이 와서 UWB stop_distance 진입 → cmd_speed = 0 → 래치 해제.
        """
        now = self.get_clock().now().nanoseconds * 1e-9

        # cmd_speed가 0 근처 = UWB stop 상태 → 래치 해제
        if cmd_speed <= self.stuck_cmd_thr:
            self.stuck_latched = False
            self.stuck_since = None
            return False

        if self.stuck_latched:
            return True

        if self.measured_speed < self.stuck_meas_thr:
            if self.stuck_since is None:
                self.stuck_since = now
            elif now - self.stuck_since > self.stuck_timeout:
                self.stuck_latched = True
                return True
        else:
            self.stuck_since = None

        return False

    # ── 3. 협로 속도 감속 계수 ───────────────────────────────────────
    def _narrow_speed_factor(self) -> float:
        """
        좌/우 측면(±90°) 최소 거리 기반 속도 감소 계수 [0.0, 1.0].

        side_min >= narrow_warn_m → 1.0 (제한 없음)
        side_min <= narrow_stop_m → 0.0 (정지)
        그 사이                   → 선형 보간
        """
        scan = self.scan
        left_min  = float('inf')
        right_min = float('inf')

        angle = scan.angle_min
        for r in scan.ranges:
            if self.scan_range_min < r < scan.range_max and not self._is_tag(angle, r):
                # 왼쪽: LiDAR +π/2 방향
                if abs(self._wrap(angle - math.pi / 2)) < self.side_cone_half:
                    left_min = min(left_min, r)
                # 오른쪽: LiDAR -π/2 방향
                elif abs(self._wrap(angle + math.pi / 2)) < self.side_cone_half:
                    right_min = min(right_min, r)
            angle += scan.angle_increment

        side_min = min(left_min, right_min)

        if side_min >= self.narrow_warn_m:
            return 1.0
        if side_min <= self.narrow_stop_m:
            return 0.0
        return (side_min - self.narrow_stop_m) / (self.narrow_warn_m - self.narrow_stop_m)

    # ── 메인 루프 ─────────────────────────────────────────────────────
    def _loop(self):
        if self.scan is None or not self._fresh(self.last_scan_t):
            if self.require_scan:
                self._publish(0.0, 0.0, 'NO_SCAN')
                return
            # require_scan=False: LiDAR 없어도 UWB 명령으로 통과
            if not self._fresh(self.last_nominal_t):
                self._publish(0.0, 0.0, 'UWB_TIMEOUT')
                return
            self._publish(self.nominal_speed, self.nominal_steer, 'UWB_ONLY')
            return

        # ── 1. 비상 정지 (최우선) ────────────────────────────────────
        if self.enable_emstop and self._check_emstop():
            self.stuck_since = None
            self.stuck_latched = False
            self._publish(0.0, 0.0, 'EMSTOP', 'EMSTOP')
            self.get_logger().warn(
                'EMSTOP: obstacle in emergency zone',
                throttle_duration_sec=0.5
            )
            return

        fwd_dist = self._forward_min_dist()

        # ── 히스테리시스 모드 전환 ────────────────────────────────────
        if not self.dwa_mode and fwd_dist <= self.obstacle_switch_m:
            self.dwa_mode = True
            self.get_logger().warn(
                f'[ARBITRATOR] Stanley → DWA (obstacle={fwd_dist:.2f}m)'
            )
        elif self.dwa_mode and fwd_dist > self.obstacle_clear_m:
            self.dwa_mode = False
            self.get_logger().info(
                f'[ARBITRATOR] DWA → Stanley (clear={fwd_dist:.2f}m)'
            )

        # ── 모드별 속도/조향 결정 ─────────────────────────────────────
        if self.dwa_mode:
            if not self._fresh(self.last_dwa_t):
                self._publish(0.0, 0.0, 'DWA_TIMEOUT')
                return
            speed, steer = self.dwa_speed, self.dwa_steer
            mode_str = f'DWA:{self.dwa_state}'
        else:
            if not self._fresh(self.last_nominal_t):
                self._publish(0.0, 0.0, 'STANLEY_TIMEOUT')
                return
            speed, steer = self.nominal_speed, self.nominal_steer
            mode_str = 'STANLEY'

        # ── 2. 끼임 감지 ─────────────────────────────────────────────
        if self.enable_stuck and self._check_stuck(speed):
            self._publish(0.0, steer, mode_str, 'STUCK')
            self.get_logger().warn(
                'STUCK: commanded but not moving — waiting for clearance',
                throttle_duration_sec=1.0
            )
            return

        # ── 3. 협로 속도 감속 (Stanley 모드에서만) ───────────────────
        # DWA 모드에서는 DWA가 자체 속도 조절하므로 협로 감속 적용 안 함
        # 적용 시 DWA 궤적 계산 속도와 실제 속도 불일치 → DWA_BLOCKED 오작동
        if self.enable_narrow and not self.dwa_mode:
            narrow_factor = self._narrow_speed_factor()
            if narrow_factor < 1.0:
                safety_str = 'NARROW_STOP' if narrow_factor == 0.0 else 'NARROW'
                speed *= narrow_factor
            else:
                safety_str = 'SAFE'
        else:
            safety_str = 'SAFE'

        self._publish(speed, steer, mode_str, safety_str)

        self.get_logger().info(
            f'mode={"DWA" if self.dwa_mode else "STANLEY"} '
            f'fwd={fwd_dist:.2f}m '
            f'spd={speed:.3f} safety={safety_str}',
            throttle_duration_sec=0.5
        )


def main(args=None):
    rclpy.init(args=args)
    node = CmdArbitratorNode()
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
