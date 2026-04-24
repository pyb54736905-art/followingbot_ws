from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    UWB + Stanley 통합 팔로잉 시스템 (옵션 B)

    토픽 흐름:
      UWB (ttyACM1) → uwb_serial_bridge → /uwb_a0, /uwb_a1
                                              ↓
                                       uwb_follower
                                  /uwb_theta, /uwb_avg_dist
                                  /desired_speed (거리 기반 목표 속도)
                                              ↓
      rpm_bridge ← (피드백) → /left_rpm, /right_rpm
                                              ↓
                                    wheel_odom_node → /odom
                                              ↓
                                   uwb_path_gen_node → /path
                                              ↓
                                    car_control_node ← /measured_speed
                                  (Stanley 조향 + PID 속도)
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
        #    /target_speed → /desired_speed (car_control_node 입력)
        #    /target_steer → unused (Stanley가 조향 담당)
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
                {'max_speed_mps': 0.45},
                {'min_speed_mps': 0.10},

                {'turn_steer_rad': 0.40},
                {'anchor_spacing_m': 0.35},
                {'x_bias_m': -0.25},
                {'center_half_angle_rad': 0.15},
                {'max_steer_angle_rad': 0.45},

                {'ema_alpha': 0.20},
                {'data_timeout_s': 20.0},
                {'invalid_max_m': 10.0},
                {'steer_sign': -1.0},
            ]
        ),

        # ── 3. 휠 오도메트리 ──────────────────────────────────────
        #    /left_rpm, /right_rpm (ERPM) → /odom
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
        #    /uwb_theta + /uwb_avg_dist + /odom → /path
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
                # uwb_follower steer_sign=-1.0과 동일 부호 규칙
                {'theta_sign': -1.0},
                {'data_timeout_s': 1.0},
            ]
        ),

        # ── 5. 차량 컨트롤러 (Stanley 조향 + PID 속도) ─────────────
        #    /path + /odom + /measured_speed + /desired_speed
        #    → /target_speed, /target_steer
        Node(
            package='followingbot_control',
            executable='car_control_node',
            name='car_control_node',
            output='screen',
            parameters=[
                {'wheelbase': 0.48},
                {'stanley_k': 1.0},
                {'max_steer_deg': 28.0},

                # /desired_speed 토픽으로 동적 속도 수신
                {'use_dynamic_speed': True},
                {'dynamic_speed_timeout': 0.5},
                # 폴백 속도 (desired_speed 타임아웃 시)
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
            ]
        ),

        # ── 6. 모터 시리얼 브릿지 ────────────────────────────────
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
