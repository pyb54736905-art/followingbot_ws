from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    UWB + LiDAR + Stanley 통합 팔로잉 시스템

    토픽 흐름:
      UWB (ttyACM1) → uwb_serial_bridge → /uwb_a0, /uwb_a1
                                              ↓
                                       uwb_follower
                              /uwb_theta, /uwb_avg_dist, /desired_speed
                                              ↓
      rpm_bridge ← (피드백) → /left_rpm, /right_rpm
                                              ↓
                                    wheel_odom_node → /odom
                                              ↓
                                   uwb_path_gen_node → /path
                                              ↓
                                  car_control_node ← /measured_speed, /desired_speed
                                  (Stanley 조향 + PID 속도)
                                              ↓
                             /nominal_speed, /nominal_steer
                                              ↓
      LiDAR (ttyUSB0) → sllidar_node → /scan
           ↓                                  ↓
           └──→ dwa_controller_node → /dwa_speed, /dwa_steer, /dwa_state
                                              ↓
                                  cmd_arbitrator_node
                          (전방 장애물 없음 → Stanley, 근접 → DWA)
                                              ↓
                             /target_speed, /target_steer
                                              ↓
                             rpm_direct_serial_bridge (ttyACM0)
                                              ↓
                                           모터/서보
    """
    return LaunchDescription([

        # ── 1. UWB 시리얼 브릿지 ────────────────────────────────────
        Node(
            package='followingbot_bridge',
            executable='uwb_serial_bridge',
            name='uwb_serial_bridge',
            output='screen',
            parameters=[
                {'port': '/dev/ttyACM1'},
                {'baud': 115200},
                {'open_delay': 2.0},
                {'read_period': 0.02},
                {'verbose_rx': False},
            ]
        ),

        # ── 2. UWB 팔로워 ─────────────────────────────────────────
        #    /target_speed → /desired_speed  (car_control_node 동적 속도 입력)
        #    /target_steer → unused          (Stanley가 조향 담당)
        Node(
            package='followingbot_bridge',
            executable='uwb_follower',
            name='uwb_follower_node',
            output='screen',
            remappings=[
                ('/target_speed', '/desired_speed'),
                ('/target_steer', '/uwb_direct_steer_unused'),
            ],
            parameters=[
                {'a0_topic': '/uwb_a0'},
                {'a1_topic': '/uwb_a1'},
                {'loop_hz': 20.0},

                {'stop_distance_m': 0.50},
                {'slow_distance_m': 1.20},
                {'max_speed_mps': 0.50},
                {'min_speed_mps': 0.10},

                {'turn_steer_rad': 0.45},
                {'anchor_spacing_m': 0.35},
                {'x_bias_m': 0.175},
                {'center_half_angle_rad': 0.30},
                {'max_steer_angle_rad': 0.45},

                {'ema_alpha': 0.12},
                {'outlier_threshold_m': 0.70},  # 빠른 접근 시 outlier로 걸러지는 문제 방지
                {'data_timeout_s': 20.0},
                {'invalid_max_m': 10.0},
                {'steer_sign': -1.0},
                # 태그 방향이 이 각도(deg→rad) 초과 시 정지
                {'stop_angle_rad': 1.22},  # 70도
            ]
        ),

        # ── 3. 휠 오도메트리 ──────────────────────────────────────
        Node(
            package='followingbot_bridge',
            executable='wheel_odom_node',
            name='wheel_odom_node',
            output='screen',
            parameters=[
                {'track_m': 0.304},
                {'wheel_radius_m': 0.10},
                {'gear_ratio': 1.0},
                {'motor_pole_pairs': 10.0},
                {'erpm_input': True},
                {'odom_frame': 'odom'},
                {'base_frame': 'base_link'},
                {'publish_tf': True},
                {'update_hz': 50.0},
            ]
        ),

        # ── 4. UWB 경로 생성기 ────────────────────────────────────
        Node(
            package='followingbot_bridge',
            executable='uwb_path_gen_node',
            name='uwb_path_gen_node',
            output='screen',
            parameters=[
                {'path_length_m': 2.0},
                {'n_waypoints': 10},
                {'loop_hz': 20.0},
                {'frame_id': 'odom'},
                {'theta_sign': -1.0},
                {'data_timeout_s': 1.0},
                {'dir_ema_alpha': 0.15},
                # 소각도 노이즈만 직진 처리, 실제 오프셋은 Stanley 가 교정
                # center_half_angle_rad 보다 작게, 잔류 노이즈(~7-10deg) 보다 크게 설정
                {'theta_deadband_rad': 0.20},
            ]
        ),

        # ── 5. 차량 컨트롤러 (Stanley 조향 + PID 속도) ─────────────
        #    출력을 /nominal_speed, /nominal_steer 으로 리매핑
        #    → cmd_arbitrator_node 가 최종 /target_speed, /target_steer 결정
        Node(
            package='followingbot_control',
            executable='car_control_node',
            name='car_control_node',
            output='screen',
            remappings=[
                ('/target_speed', '/nominal_speed'),
                ('/target_steer', '/nominal_steer'),
            ],
            parameters=[
                {'wheelbase': 0.48},
                {'stanley_k': 1.0},
                {'max_steer_deg': 26.0},

                {'use_dynamic_speed': True},
                {'dynamic_speed_timeout': 0.5},
                {'target_speed': 0.0},

                {'speed_kp': 1.5},
                {'speed_ki': 0.1},
                {'speed_kd': 0.0},
                {'accel_limit': 0.30},
                {'decel_limit': 0.50},

                {'lookahead_idx': 3},
                {'use_lowlevel_speed_feedback': True},
                {'max_speed_cmd': 0.50},
                {'speed_deadband': 0.03},
                {'allow_reverse': False},

                # ── Stanley + Pure Pursuit 횡방향 혼합 ─────────────────
                # Pure Pursuit 전용 룩어헤드 인덱스 (낮을수록 가까운 점 참조)
                {'pp_lookahead_idx': 5},
                # 속도 기반 자동 블렌딩 비활성화 → stanley_weight 고정값 사용
                {'use_speed_blend': False},
                # Stanley 가중치 (0.0=순수PP, 1.0=순수Stanley)
                # 예: 0.3 → 0.3*Stanley + 0.7*PurePursuit
                {'stanley_weight': 0.3},
                {'pp_blend_speed_low': 0.15},
                {'pp_blend_speed_high': 0.35},
            ]
        ),

        # ── 6. RPLIDAR A1M8 ───────────────────────────────────────
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            parameters=[
                {'channel_type': 'serial'},
                {'serial_port': '/dev/ttyUSB0'},
                {'serial_baudrate': 115200},
                {'frame_id': 'laser'},
                {'inverted': False},
                {'angle_compensate': True},
                {'scan_mode': 'Sensitivity'},
            ]
        ),

        # ── 7. DWA 장애물 회피 컨트롤러 ───────────────────────────
        #    출력을 /dwa_speed, /dwa_steer 으로 리매핑
        #    → cmd_arbitrator_node 가 사용 여부를 결정
        Node(
            package='followingbot_dwa',
            executable='dwa_controller_node',
            name='dwa_controller_node',
            output='screen',
            remappings=[
                ('/target_speed', '/dwa_speed'),
                ('/target_steer', '/dwa_steer'),
            ],
            parameters=[
                {'wheelbase_m': 0.48},
                {'max_speed_mps': 0.45},
                {'max_steer_rad': 0.45},
                {'robot_radius_m': 0.20},

                {'max_accel_mps2': 0.30},
                {'max_steer_rate_rads': 0.50},
                {'dw_time_s': 0.40},

                {'predict_time_s': 1.2},
                {'dt_sim_s': 0.1},

                {'n_v_samples': 5},
                {'n_steer_samples': 31},

                {'w_heading': 0.3},
                {'w_clearance': 0.5},
                {'w_speed': 0.2},

                {'stop_distance_m': 0.50},
                {'slow_distance_m': 1.20},
                {'min_speed_mps': 0.30},
                {'loop_hz': 20.0},
                {'data_timeout_s': 1.0},
                {'obstacle_range_max_m': 2.0},
                # 로봇 프레임이 LiDAR에 잡히는 경우 제거 (0.21m에서 자체 감지됨)
                {'obstacle_range_min_m': 0.30},

                {'goal_theta_sign': -1.0},

                {'tag_exclusion_half_angle_deg': 30.0},
                {'tag_exclusion_dist_margin_m': 0.8},
            ]
        ),

        # ── 8. 명령 중재기 ────────────────────────────────────────
        #    Stanley ↔ DWA 모드 전환 및 /target_speed, /target_steer 최종 발행
        #    + LiDAR 안전 기능: 비상 정지 / 끼임 감지 / 협로 속도 감속
        Node(
            package='followingbot_bridge',
            executable='cmd_arbitrator_node',
            name='cmd_arbitrator_node',
            output='screen',
            parameters=[
                # 전방 장애물이 이 거리 이하이면 DWA 모드로 전환
                {'obstacle_switch_m': 1.0},
                # 이 거리 이상으로 복귀해야 Stanley 모드로 복귀 (히스테리시스)
                {'obstacle_clear_m': 1.8},
                # 전방 cone 각도 (좌우 합산, deg)
                {'forward_cone_deg': 40.0},
                {'loop_hz': 20.0},
                {'data_timeout_s': 0.5},
                {'enable_emstop': True},
                {'enable_stuck_protection': False},
                {'enable_narrow_slowdown': True},
                {'require_scan': True},
                # DWA 와 동일한 태그 착용자 제외 설정
                {'tag_exclusion_half_angle_deg': 30.0},
                {'tag_exclusion_dist_margin_m': 0.8},
                # 로봇 프레임이 LiDAR에 잡히는 경우 제거 (0.21m에서 자체 감지됨)
                {'scan_range_min_m': 0.30},

                # ── 비상 정지 구역 ─────────────────────────────────
                # 전방 이 거리 이내 장애물 → 즉시 정지 (태그 무관)
                {'emstop_dist_m': 0.65},
                # 비상 정지 감지 전방 cone 폭 (좌우 합산 deg)
                {'emstop_cone_deg': 20.0},
                # 로봇 자체 프레임 제거용 최솟값 (유효 감지 구간: 0.50~0.65m)
                {'emstop_range_min_m': 0.50},

                # ── 협로 속도 감속 ─────────────────────────────────
                # 좌/우 측면 여유가 이 이하이면 속도 감소 시작
                {'narrow_warn_m': 0.50},
                # 이 이하이면 완전 정지
                {'narrow_stop_m': 0.20},
                # 측면 스캔 cone 폭 (±90° 기준, 좌우 각각 ±half, deg)
                {'side_cone_deg': 40.0},

                # ── 끼임 감지 ──────────────────────────────────────
                # 이 이상 속도 명령 = "움직여야 함"
                {'stuck_cmd_threshold_mps': 0.08},
                # 이 이하 실측 속도 = "정지 상태"
                {'stuck_meas_threshold_mps': 0.04},
                # 이 시간 이상 지속되면 끼임 판정 → 정지
                {'stuck_timeout_s': 8.0},
            ]
        ),

        # ── 9. 모터 시리얼 브릿지 ────────────────────────────────
        Node(
            package='followingbot_bridge',
            executable='rpm_direct_serial_bridge',
            name='rpm_direct_serial_bridge',
            output='screen',
            parameters=[
                {'port': '/dev/ttyACM0'},
                {'baud': 115200},
                {'open_delay': 2.0},

                {'wheelbase_m': 0.48},
                {'track_m': 0.304},
                {'wheel_radius_m': 0.10},
                {'gear_ratio': 1.0},
                {'motor_pole_pairs': 10.0},

                {'tx_rate_hz': 100.0},
                {'command_timeout_sec': 0.5},
                {'max_erpm_per_sec': 800.0},
                {'lpf_alpha': 0.35},
                {'zero_erpm_band': 80.0},
                {'max_erpm': 6000.0},

                {'left_invert': False},
                {'right_invert': False},

                {'max_steer_rad': 0.45},
                {'debug_log': False},
            ]
        ),
    ])
