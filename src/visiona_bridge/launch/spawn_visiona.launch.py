import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, LogInfo, OpaqueFunction, ExecuteProcess, SetLaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PythonExpression, Command, FindExecutable
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import xacro

def generate_launch_description():
    # --- Arguments ---
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Launch specific GUI tools (RViz and Web App)'
    )
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz', default_value='true',
        description='Whether to launch RViz (can be disabled if using external RViz)'
    )
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='sim',
        description='Execution mode: "sim" (Gazebo), "real" (Real Robot), "fake" (Mock Hardware)'
    )
    model_arg = DeclareLaunchArgument(
        'model', default_value='',
        description='Alias for mode (e.g. model:=real)'
    )
    
    # --- Configurations ---
    gui = LaunchConfiguration('gui')
    launch_rviz = LaunchConfiguration('launch_rviz')
    mode = LaunchConfiguration('mode')
    
    # Handle alias: if model != '', set mode = model
    # Handle alias: if model != '', set mode = model
    set_mode = SetLaunchConfiguration(
        name='mode', 
        value=LaunchConfiguration('model'), 
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('model'), "' != ''"]))
    )

    pkg_share = get_package_share_directory('visiona_bridge')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    
    # --- URDF & Config ---
    xacro_file_path = os.path.join(pkg_share, 'urdf', 'visiona.urdf.xacro')
    controller_config_path = os.path.join(pkg_share, 'config', 'visiona_controllers.yaml')

    # Use Command substitution to allow dynamic arguments for xacro
    # use_sim: Enables Ros2Control and standard physics (True for SIM and REAL/DigitalTwin)
    # use_gazebo_joint_pub: Enables classic joint state publisher (True ONLY for SIM, False for REAL to avoid conflict)
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        xacro_file_path, ' ',
        'controller_config_path:=', controller_config_path, ' ',
        'use_sim:=', PythonExpression(["'true' if '", mode, "' in ['sim', 'real'] else 'false'"]), ' ',
        'use_gazebo_joint_pub:=', PythonExpression(["'true' if '", mode, "' == 'sim' else 'false'"])
    ])
    
    robot_description = {'robot_description': robot_description_content}

    # --- Nodes Definition ---
    
    # 1. State Publisher
    # If mode is SIM, we want to publish robot_description, but maybe not joint states immediately?
    # standard robot_state_publisher usually runs in all modes to handle transforms.
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': PythonExpression(["'", mode, "' in ['sim', 'real']"])}]
    )

    # 2. Gazebo (In SIM or REAL mode for Digital Twin)
    gazebo_launch_file = os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={'extra_gazebo_args': '--verbose'}.items(),
        condition=IfCondition(PythonExpression(["'", mode, "' in ['sim', 'real']"]))
    )

    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'Visiona', '-timeout', '90',
                   '-x', '0.0', '-y', '0.0', '-z', '0.05'],
        output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' in ['sim', 'real']"]))
    )

    # 3. Controllers
    # joint_state_broadcaster: Only in SIM. In REAL, web_gui_node provides states.
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
        condition=LaunchConfigurationEquals('mode', 'sim')
    )

    # joint_trajectory_controller: In SIM (to move logic) and REAL (to follow real robot)
    spawn_joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
        output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' in ['sim', 'real']"]))
    )

    # Move to Home Position (Workaround for spawn_entity -J issues)
    # Publishes a single trajectory point to move joints to 1.57 rad.
    # Only in SIM. In REAL, robot is already at position (and we sync to it).
    move_to_home = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', '/joint_trajectory_controller/joint_trajectory', 
             'trajectory_msgs/msg/JointTrajectory',
             '{joint_names: ["base_link_joint", "link_1_shoulder_joint", "link_2_elbow_joint", "link_3_wrist_joint", "link_3_wrist_to_gripper_base_joint", "gripper"], points: [{positions: [1.57, 1.57, 1.57, 1.57, 1.57, -0.15], time_from_start: {sec: 2, nanosec: 0}}]}'],
        output='screen',
        condition=LaunchConfigurationEquals('mode', 'sim')
    )

    
    # 4. RViz (Only if GUI=true)
    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        condition=IfCondition(launch_rviz),
        parameters=[{'use_sim_time': PythonExpression(["'", mode, "' in ['sim', 'real']"])}]
    )

    # 5. Web GUI / Real Robot Bridge
    bridge_params = [{
        'publish_joint_states': PythonExpression(["'true' if '", mode, "' != 'sim' else 'false'"]),
        'sync_gazebo': PythonExpression(["'true' if '", mode, "' == 'real' else 'false'"]),
        'serial_port': '/dev/ttyUSB0', # Default
        'use_sim_time': PythonExpression(["'true' if '", mode, "' in ['sim', 'real'] else 'false'"])
    }]

    web_node = Node(
        package='visiona_bridge',
        executable='web_gui', # It's a script, ensure it's installed as executable or use regular executable name from setup.py entry_points
        name='web_gui_node',
        output='screen',
        parameters=bridge_params,
        arguments=[PythonExpression(["'--no-gui' if '", gui, "' == 'false' else ''"])] 
    )

    
    return LaunchDescription([
        gui_arg,
        mode_arg,
        model_arg,
        set_mode, # Apply alias logic first
        node_robot_state_publisher,
        # gazebo, dont remove 
        spawn_entity,
        
        # HANDLER FOR SIM: Entity -> Broadcaster -> Controller -> Home
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

        # HANDLER FOR REAL: Entity -> Controller (Skip Broadcaster/Home)
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[spawn_joint_trajectory_controller],
            ),
            condition=LaunchConfigurationEquals('mode', 'real')
        ),
      
        node_rviz,
        web_node
    ])