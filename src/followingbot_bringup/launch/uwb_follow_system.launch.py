from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
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
                {'verbose_rx': True},
            ]
        ),

        Node(
            package='followingbot_bridge',
            executable='uwb_follower',
            name='uwb_follower_node',
            output='screen',
            parameters=[
                {'a0_topic': '/uwb_a0'},
                {'a1_topic': '/uwb_a1'},
                {'loop_hz': 20.0},

                {'stop_distance_m': 0.40},
                {'resume_distance_m': 1.00},
                {'slow_distance_m': 1.20},
                {'max_speed_mps': 0.70},
                {'min_speed_mps': 0.30},

                {'turn_steer_rad': 0.40},
                {'anchor_spacing_m': 0.35},
                {'x_bias_m': 0.05},
                {'center_half_angle_rad': 0.15},
                {'max_steer_angle_rad': 0.45},

                {'ema_alpha': 0.35},
                {'warmup_samples': 10},
                {'data_timeout_s': 20.0},
                {'invalid_max_m': 10.0},

                {'steer_sign': -1.0},
            ]
        ),

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
                {'debug_log': True},
            ]
        ),
    ])
