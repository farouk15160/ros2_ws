import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # --- Paths ---
    pkg_path = get_package_share_directory('my_robot_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'my_robot.urdf.xacro')

    # --- Robot Description ---
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # --- Controller Configuration ---
    # NOTE: The launch file itself does not need to load this file.
    # The Gazebo <ros2_control> tag automatically tells the controller_manager
    # where to find and load these parameters.

    # --- Gazebo ---
    gzserver_cmd = ExecuteProcess(
        cmd=['gzserver',
             '',  # Use an empty world file
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             '--verbose'
            ],
        output='screen'
    )

    gzclient_cmd = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # --- Nodes ---
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_robot_arm'],
        output='screen'
    )

    # --- Controllers ---
    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # === UPDATED CONTROLLER NAME TO SPAWN ===
    spawn_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    )
    
    # --- Launch Description ---
    return LaunchDescription([
        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher_node,
        spawn_entity_node,
        
        # Launch controllers sequentially after the robot is spawned
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity_node,
                on_exit=[spawn_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_joint_state_broadcaster,
                on_exit=[spawn_trajectory_controller],
            )
        ),
    ])
