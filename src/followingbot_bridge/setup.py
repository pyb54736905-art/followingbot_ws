from setuptools import find_packages, setup

package_name = 'followingbot_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='parkmingwan',
    maintainer_email='pyb54736905@gmail.com',
    description='Serial bridge for followingbot',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'rpm_direct_serial_bridge = followingbot_bridge.rpm_direct_serial_bridge:main',
            'uwb_serial_bridge = followingbot_bridge.uwb_serial_bridge:main',
            'uwb_follower = followingbot_bridge.uwb_follower:main',
            'wheel_odom_node = followingbot_bridge.wheel_odom_node:main',
            'uwb_path_gen_node = followingbot_bridge.uwb_path_gen_node:main',
            'monitor_node = followingbot_bridge.monitor_node:main',
            'cmd_arbitrator_node = followingbot_bridge.cmd_arbitrator_node:main',
            'uwb_visualizer_node = followingbot_bridge.uwb_visualizer_node:main',
            'lidar_uwb_visualizer_node = followingbot_bridge.lidar_uwb_visualizer_node:main',
        ],
    },
)
