import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, LogInfo, ExecuteProcess, SetLaunchConfiguration, TimerAction
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PythonExpression, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # --- 1. Arguments ---
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Launch specific GUI tools (RViz and Web App)'
    )
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz', default_value='true',
        description='Whether to launch RViz'
    )
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='sim',
        description='Execution mode: "sim" (Gazebo), "real" (Real Robot), "fake" (Mock Hardware)'
    )
    model_arg = DeclareLaunchArgument(
        'model', default_value='',
        description='Alias for mode (e.g. model:=real)'
    )
    camera_arg = DeclareLaunchArgument(
        'camera', default_value='false',
        description='Launch camera driver (ascamera hp60c.launch.py)'
    )
    
    # --- 2. Configurations ---
    gui = LaunchConfiguration('gui')
    launch_rviz = LaunchConfiguration('launch_rviz')
    mode = LaunchConfiguration('mode')
    
    # Handle alias: if model != '', set mode = model
    set_mode = SetLaunchConfiguration(
        name='mode', 
        value=LaunchConfiguration('model'), 
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('model'), "' != ''"]))
    )

    pkg_share = get_package_share_directory('visiona_bridge')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    
    # --- 3. URDF & Config ---
    xacro_file_path = os.path.join(pkg_share, 'urdf', 'visiona.urdf.xacro')
    controller_config_path = os.path.join(pkg_share, 'config', 'visiona_controllers.yaml')

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        xacro_file_path, ' ',
        'controller_config_path:=', controller_config_path, ' ',
        'use_sim:=', PythonExpression(["'true' if '", mode, "' == 'sim' else 'false'"]), ' ',
        # NOTE: For 'real' mode, we usually use Mock Hardware in ros2_control (Digital Twin)
        # while the Python Web Node handles the actual Serial communication.
        'use_mock_hardware:=', PythonExpression(["'true' if '", mode, "' == 'real' else 'false'"]), ' ',
        'use_gazebo_joint_pub:=', PythonExpression(["'true' if '", mode, "' == 'sim' else 'false'"])
    ])
    
    robot_description = {'robot_description': robot_description_content}

    # --- 4. Nodes Definition ---
    
    # A. Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': PythonExpression(["'true' if '", mode, "' == 'sim' else 'false'"])}]
    )

    # B. Gazebo (SIM only)
    gazebo_launch_file = os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={'extra_gazebo_args': '--verbose'}.items(),
        condition=LaunchConfigurationEquals('mode', 'sim')
    )

    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'Visiona', '-timeout', '90',
                   '-x', '0.0', '-y', '0.0', '-z', '0.05'],
        output='screen',
        condition=LaunchConfigurationEquals('mode', 'sim')
    )

    # C. Controller Manager (REAL only - Gazebo handles this in SIM)
    node_controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config_path],
        output="screen",
        condition=LaunchConfigurationEquals('mode', 'real')
    )

    # D. Controllers (Spawners)
    # FIX 1: Run broadcaster in both SIM and REAL so TF tree works
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' in ['sim', 'real']"]))
    )

    spawn_joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
        output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' in ['sim', 'real']"]))
    )

    # E. Move to Home (SIM only)
    move_to_home = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', '/joint_trajectory_controller/joint_trajectory', 
             'trajectory_msgs/msg/JointTrajectory',
             '{joint_names: ["base_link_joint", "link_1_shoulder_joint", "link_2_elbow_joint", "link_3_wrist_joint", "link_3_wrist_to_gripper_base_joint", "gripper"], points: [{positions: [1.57, 1.57, 1.57, 1.57, 1.57, -0.15], time_from_start: {sec: 2, nanosec: 0}}]}'],
        output='screen',
        condition=LaunchConfigurationEquals('mode', 'sim')
    )
    
    # F. RViz
    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        condition=IfCondition(launch_rviz),
        parameters=[{'use_sim_time': PythonExpression(["'", mode, "' in ['sim', 'real']"])}]
    )

    # G. Web GUI / Real Robot Bridge
    bridge_params = [{
        'publish_joint_states': 'false', # Let ros2_control handle this
        'sync_gazebo': PythonExpression(["'true' if '", mode, "' == 'real' else 'false'"]),
        'serial_port': '/dev/ttyUSB0', 
        'mode': LaunchConfiguration('mode'),
        'use_sim_time': PythonExpression(["'true' if '", mode, "' == 'sim' else 'false'"])
    }]

    # FIX 2: Added '-u' for unbuffered output and 'emulate_tty=True' to see logs
    web_node = Node(
        package='visiona_bridge',
        executable=sys.executable,
        name='web_gui_node',
        output='screen',
        emulate_tty=True, 
        parameters=bridge_params,
        arguments=[
            '-u', # Unbuffered python output (Crucial for seeing errors)
            '-c', 'from visiona_bridge.web_gui_node import main; main()', 
            PythonExpression(["'--no-gui' if '", gui, "' == 'false' else ''"])
        ]
    )

    # H. Camera Launch
    camera = LaunchConfiguration('camera')
    try:
        ascamera_share = get_package_share_directory('ascamera')
        camera_launch_file = os.path.join(ascamera_share, 'launch', 'hp60c.launch.py')
        camera_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch_file),
            condition=IfCondition(PythonExpression(["'", camera, "' == 'true'"]))
        )
    except Exception:
        camera_launch = LogInfo(
            msg="Camera package 'ascamera' not found. Skipping camera launch.",
            condition=IfCondition(PythonExpression(["'", camera, "' == 'true'"]))
        )

    # --- 5. Return Launch Description ---
    return LaunchDescription([
        gui_arg,
        launch_rviz_arg,
        mode_arg,
        model_arg,
        camera_arg,
        set_mode, 
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        node_controller_manager,
        
        # HANDLER FOR SIM: Standard ROS 2 Control lifecycle
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[spawn_joint_state_broadcaster],
            ),
            condition=LaunchConfigurationEquals('mode', 'sim')
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_joint_state_broadcaster,
                on_exit=[spawn_joint_trajectory_controller],
            ),
            condition=LaunchConfigurationEquals('mode', 'sim')
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_joint_trajectory_controller,
                on_exit=[move_to_home],
            ),
            condition=LaunchConfigurationEquals('mode', 'sim')
        ),

        # HANDLER FOR REAL: Timer delay startup
        # FIX 3: Ensure Joint State Broadcaster spawns in REAL mode too
        TimerAction(
            period=3.0,
            actions=[spawn_joint_state_broadcaster, spawn_joint_trajectory_controller],
            condition=LaunchConfigurationEquals('mode', 'real')
        ),
      
        camera_launch,
        node_rviz,
        web_node
    ])

# Fix the File , fix moveit , fix camera and World