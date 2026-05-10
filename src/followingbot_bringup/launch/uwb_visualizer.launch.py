import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rviz_cfg = os.path.join(
        get_package_share_directory('followingbot_bringup'),
        'launch', 'uwb_visualizer.rviz'
    )

    return LaunchDescription([
        # UWB 위치 시각화 노드
        Node(
            package='followingbot_bridge',
            executable='uwb_visualizer_node',
            name='uwb_visualizer_node',
            parameters=[{
                'anchor_spacing_m': 0.35,
                'x_bias_m': 0.0,
                'show_trail': False,
                'trail_timeout_s': 2.0,
            }],
            output='screen',
        ),
        # LiDAR + UWB 통합 분류 노드
        Node(
            package='followingbot_bridge',
            executable='lidar_uwb_visualizer_node',
            name='lidar_uwb_visualizer_node',
            parameters=[{
                'tag_radius':  0.50,   # 태그 인식 반경 (m) — 튜닝 가능
                'min_range':   0.10,
                'max_range':   6.00,
                'point_size':  0.06,
                'theta_sign':  1.0,    # 방향 반전 필요 시 -1.0으로 변경
                'lidar_yaw_offset_deg': 180.0,  # 라이다 좌표 180도 반전
                'plot_hz':     5.0,
            }],
            output='screen',
        ),
        # base_link → laser : laser 프레임을 TF 트리에 등록 (없으면 RViz Fixed Frame Error)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
        ),
        # base_link → uwb_frame : UWB 마커를 laser Fixed Frame에서 표시
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='uwb_static_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'uwb_frame'],
        ),
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_cfg],
            output='screen',
        ),
    ])
