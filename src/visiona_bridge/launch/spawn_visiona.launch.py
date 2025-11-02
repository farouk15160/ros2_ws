import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('visiona_bridge')
    xacro_file_path = os.path.join(pkg_share, 'urdf', 'visiona.urdf.xacro')
    controller_config_path = os.path.join(pkg_share, 'config', 'visiona_controllers.yaml')

    robot_description_config = xacro.process_file(
        xacro_file_path,
        mappings={'controller_config_path': controller_config_path}
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description] 
    )

    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    gazebo_launch_file = os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={'extra_gazebo_args': '--verbose'}.items()
    )

    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'Visiona'],
        output='screen'
    )

    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2"
    )
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', 
                   '--controller-manager', '/controller_manager'],
        output='screen'
    )

    spawn_joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', 
                   '--controller-manager', '/controller_manager'],
        output='screen'
    )

    delay_broadcaster_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[spawn_joint_state_broadcaster],
        )
    )
    
    delay_controller_after_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[spawn_joint_trajectory_controller],
        )
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        node_rviz,        
        delay_broadcaster_after_spawn,
        delay_controller_after_broadcaster
    ])