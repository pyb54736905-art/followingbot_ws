from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('followingbot_control')
    params = os.path.join(pkg, 'config', 'control.yaml')

    return LaunchDescription([
        Node(
            package='followingbot_control',
            executable='car_control_node',
            name='car_control_node',
            output='screen',
            parameters=[params],
        ),
    ])
