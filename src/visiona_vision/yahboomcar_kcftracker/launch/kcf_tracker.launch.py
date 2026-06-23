from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yahboomcar_kcftracker',
            executable='kcf_tracker',
            name='image_converter',
            output='screen',
        ),
    ])
