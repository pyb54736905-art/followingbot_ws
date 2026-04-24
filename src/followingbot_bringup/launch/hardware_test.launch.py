from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """모터/서보 단독 테스트용 launch. RPMCMD를 시리얼로 직접 전송하고 STATE 피드백만 수신."""
    return LaunchDescription([
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
