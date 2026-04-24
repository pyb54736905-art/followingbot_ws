from setuptools import find_packages, setup

package_name = 'followingbot_dwa'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='parkmingwan',
    maintainer_email='pyb54736905@gmail.com',
    description='DWA-based obstacle avoidance with UWB + LiDAR sensor fusion',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'dwa_controller_node = followingbot_dwa.dwa_controller_node:main',
        ],
    },
)
