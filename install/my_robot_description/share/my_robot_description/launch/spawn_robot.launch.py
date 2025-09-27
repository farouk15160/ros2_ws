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
    world_file = os.path.join(pkg_path, 'worlds', 'my_robot.world')

    # --- Robot Description ---
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # --- Gazebo Server ---
    # This command starts the physics simulation in the background
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['bash', '-c', f'source /opt/ros/foxy/setup.bash && gzserver --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so {world_file}'],
        output='screen'
    )

    # ==========================================================
    # === ADDING THE GUI BACK IN ===
    # This command starts the graphical client
    # ==========================================================
    start_gazebo_client_cmd = ExecuteProcess(
        cmd=['bash', '-c', 'source /opt/ros/foxy/setup.bash && gzclient --verbose'],
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

    gazebo_mirror_node = Node(
        package='robot_arm_bridge',
        executable='gazebo_mirror',
        output='screen'
    )

    # --- Controllers ---
    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    spawn_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    )
    
    # --- Launch Description ---
    return LaunchDescription([
        start_gazebo_server_cmd,
        start_gazebo_client_cmd, # <-- Add the GUI command to the launch
        robot_state_publisher_node,
        spawn_entity_node,
        gazebo_mirror_node,
        
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

