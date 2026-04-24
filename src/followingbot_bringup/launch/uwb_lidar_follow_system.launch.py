from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    UWB + LiDAR 통합 팔로잉 시스템

    토픽 흐름:
      UWB (ttyACM1) → uwb_serial_bridge → /uwb_a0, /uwb_a1
                                              ↓
                                       uwb_follower
                                       (/uwb_theta, /uwb_avg_dist)
                                              ↓
      LiDAR (ttyUSB0) → sllidar_node → /scan
                                              ↓
                                    dwa_controller_node
                             (/target_speed, /target_steer)
                                              ↓
                             rpm_direct_serial_bridge (ttyACM0)
                                              ↓
                                           모터/서보

    주의:
      uwb_follower가 출력하는 /target_speed, /target_steer는
      이 launch에서 unused 토픽으로 리매핑하여 dwa_controller와 충돌 방지
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

        # ── 2. UWB 팔로워 (목표 방향/거리 계산만 담당) ───────────────
        #    /target_speed, /target_steer → 미사용 토픽으로 리매핑
        #    (dwa_controller_node가 실제 명령을 담당)
        Node(
            package='followingbot_bridge',
            executable='uwb_follower',
            name='uwb_follower_node',
            output='screen',
            remappings=[
                ('/target_speed', '/uwb_direct_speed_unused'),
                ('/target_steer', '/uwb_direct_steer_unused'),
            ],
            parameters=[
                {'a0_topic': '/uwb_a0'},
                {'a1_topic': '/uwb_a1'},
                {'loop_hz': 20.0},

                {'stop_distance_m': 0.50},
                {'slow_distance_m': 1.20},
                {'max_speed_mps': 0.45},
                {'min_speed_mps': 0.10},

                {'turn_steer_rad': 0.40},
                {'anchor_spacing_m': 0.35},
                {'x_bias_m': -0.30},
                {'center_half_angle_rad': 0.15},
                {'max_steer_angle_rad': 0.45},

                {'ema_alpha': 0.20},
                {'data_timeout_s': 20.0},
                {'invalid_max_m': 10.0},
                {'steer_sign': -1.0},
            ]
        ),

        # ── 3. RPLIDAR A1M8 드라이버 ─────────────────────────────────
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

        # ── 4. DWA 제어기 (UWB 목표 + LiDAR 장애물 → 속도/조향) ──────
        Node(
            package='followingbot_dwa',
            executable='dwa_controller_node',
            name='dwa_controller_node',
            output='screen',
            parameters=[
                # 로봇 기구학
                {'wheelbase_m': 0.48},
                {'max_speed_mps': 0.45},
                {'max_steer_rad': 0.40},
                {'robot_radius_m': 0.25},

                # 동적 윈도우
                {'max_accel_mps2': 0.30},
                {'max_steer_rate_rads': 0.80},
                {'dw_time_s': 0.5},

                # 궤적 시뮬레이션
                {'predict_time_s': 2.0},
                {'dt_sim_s': 0.1},

                # 샘플링 해상도
                {'n_v_samples': 5},
                {'n_steer_samples': 11},

                # 점수 가중치 (합 = 1.0)
                {'w_heading': 0.5},
                {'w_clearance': 0.3},
                {'w_speed': 0.2},

                # 동작 설정
                {'stop_distance_m': 0.50},
                {'loop_hz': 20.0},
                {'data_timeout_s': 1.0},
                {'obstacle_range_max_m': 3.5},

                # theta 부호 보정 (-1.0: 현재 uwb_follower 부호 규칙 맞춤)
                {'goal_theta_sign': -1.0},

                # 태그 착용자 제외 구역 (사람을 장애물로 오인 방지)
                # 타겟 방향 ±30° 이내, 타겟 거리 +0.5m 이내 LiDAR 포인트 제외
                {'tag_exclusion_half_angle_deg': 30.0},
                {'tag_exclusion_dist_margin_m': 0.5},

                # LiDAR 없어도 동작 허용 (False: 없으면 장애물 없음으로 간주)
                {'scan_required': False},
            ]
        ),

        # ── 5. 모터 시리얼 브릿지 ────────────────────────────────────
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
                {'lpf_alpha': 0.25},
                {'zero_erpm_band': 50.0},
                {'max_erpm': 6000.0},

                {'left_invert': False},
                {'right_invert': False},

                {'max_steer_rad': 0.49},
                {'debug_log': False},
            ]
        ),
    ])
