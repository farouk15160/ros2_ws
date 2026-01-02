from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("visiona", package_name="visiona_moveit_config").to_moveit_configs()
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'publish_robot_description_semantic': True},
            ],
        )
    ])
