import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import LaunchConfigurationEquals
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- Arguments ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='sim',
        description='Execution mode: "sim" (Gazebo), "real" (Real Robot)'
    )
    mode = LaunchConfiguration('mode')

    visiona_bridge_dir = get_package_share_directory('visiona_bridge')
    visiona_moveit_config_dir = get_package_share_directory('visiona_moveit_config')

    # Define use_sim_time based on mode (True for Sim, False for Real/Mock)
    use_sim_time = PythonExpression(["'true' if '", mode, "' == 'sim' else 'false'"])

    # --- 1. Launch Gazebo Simulation & Controllers ---
    # This launches: Gazebo, Spawns Robot, Spawns Controllers (joint_state_broadcaster, joint_trajectory_controller)
    spawn_visiona_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(visiona_bridge_dir, 'launch', 'spawn_visiona.launch.py')
        ),
        launch_arguments={
            'gui': 'true',  # Enable Web GUI
            'launch_rviz': 'false', # Disable standard RViz, we use MoveIt's RViz
            'mode': mode
        }.items()
    )

    # --- 2. Launch MoveIt (Move Group) ---
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(visiona_moveit_config_dir, 'launch', 'move_group.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # --- 3. Launch RViz (MoveIt Config) ---
    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(visiona_moveit_config_dir, 'launch', 'moveit_rviz.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )


    # --- 4. Launch Camera (ASCamera) ---
    # User requested this to run always
    ascamera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ascamera'), 'launch', 'hp60c.launch.py')
        ),
        # Camera launch might not take use_sim_time, or has its own args. 
        # Assuming defaults are fine for now.
    )

    # --- 5. Homing Script ---
    # Homes the robot on startup (after MoveIt is ready)
    homer_node = Node(
        package='visiona_bridge',
        executable='moveit_homer.py',
        name='moveit_homer',
        output='screen'
    )
    
    # TF from robot's camera_link to specific camera driver frame
    camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_link', 'ascamera_hp60c_camera_link_0'],
        output='screen'
    )

    # --- 7. Command Forwarder (Real Mode) ---
    # Forwards controller reference to bridge joint_targets
    command_forwarder_node = Node(
        package='visiona_bridge',
        executable='command_forwarder.py',
        name='command_forwarder',
        output='screen',
        condition=LaunchConfigurationEquals('mode', 'real')
    )

    return LaunchDescription([
        mode_arg,
        spawn_visiona_launch,
        ascamera_launch,
        
        # Delay MoveIt slightly to ensure controllers are ready
        TimerAction(
            period=5.0,
            actions=[move_group_launch]
        ),
        
        # Delay RViz slightly more
        TimerAction(
            period=8.0,
            actions=[moveit_rviz_launch]
        ),

        # Delay Homer until MoveIt is definitely ready
        # TimerAction(
        #     period=15.0,
        #     actions=[homer_node]
        # ),
        camera_tf_node,
        command_forwarder_node
    ])

