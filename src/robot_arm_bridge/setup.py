from setuptools import setup
import os
from glob import glob

package_name = 'robot_arm_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files if you add any to this package
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='farouk',
    maintainer_email='farouk@todo.todo',
    description='ROS2 bridge for the ESP32 robot arm.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Entry point for the hardware bridge
            'serial_bridge = robot_arm_bridge.serial_bridge_node:main',
            'web_gui = robot_arm_bridge.web_gui_node:main',
            # Entry point for the Gazebo mirror
            'gazebo_mirror = robot_arm_bridge.gazebo_mirror_node:main',
        ],
    },
)

