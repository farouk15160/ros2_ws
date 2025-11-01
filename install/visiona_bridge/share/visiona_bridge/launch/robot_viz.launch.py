import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Get the path to your package's share directory
    pkg_share = FindPackageShare(package='visiona_bridge')

    # Get the path to your URDF xacro file
    # THIS IS THE CORRECTION: Use 'visiona.urdf.xacro'
    xacro_file_path = PathJoinSubstitution([
        pkg_share, 'urdf', 'visiona.urdf.xacro'
    ])

    # Find the xacro executable
    xacro_exec = FindExecutable(name='xacro')

    # Prepare the command to process the xacro file
    robot_description_content = Command([
        xacro_exec, ' ', xacro_file_path
    ])

    # --- Nodes ---

    # Robot State Publisher
    # This node publishes the robot's structure (TF)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )
    
    # Joint State Publisher GUI
    # This node provides sliders to move the joints
    node_joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui"
    )

    # RViz2
    # This node opens the RViz visualization tool
    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2"
        # We removed the config file since you don't have one created yet
        # arguments=['-d', rviz_config_file_path] 
    )
    
    # --- Launch Description ---
    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        node_rviz
    ])

