import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, LogInfo, OpaqueFunction, ExecuteProcess
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
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='sim',
        description='Execution mode: "sim" (Gazebo), "real" (Real Robot), "fake" (Mock Hardware)'
    )
    
    # --- Configurations ---
    gui = LaunchConfiguration('gui')
    mode = LaunchConfiguration('mode')
    
    pkg_share = get_package_share_directory('visiona_bridge')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    
    # --- URDF & Config ---
    xacro_file_path = os.path.join(pkg_share, 'urdf', 'visiona.urdf.xacro')
    controller_config_path = os.path.join(pkg_share, 'config', 'visiona_controllers.yaml')

    # Use Command substitution to allow dynamic arguments for xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        xacro_file_path, ' ',
        'controller_config_path:=', controller_config_path, ' ',
        'use_sim:=', PythonExpression(["'true' if '", mode, "' == 'sim' else 'false'"])
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
        parameters=[robot_description, {'use_sim_time': PythonExpression(["'", mode, "' == 'sim'"])}]
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

    # 3. Controllers (Only in SIM mode)
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
        condition=LaunchConfigurationEquals('mode', 'sim')
    )

    spawn_joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
        output='screen',
        condition=LaunchConfigurationEquals('mode', 'sim')
    )

    # Move to Home Position (Workaround for spawn_entity -J issues)
    # Publishes a single trajectory point to move joints to 1.57 rad.
    move_to_home = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', '/joint_trajectory_controller/joint_trajectory', 
             'trajectory_msgs/msg/JointTrajectory',
             '{joint_names: ["base_link_joint", "link_1_shoulder_joint", "link_2_elbow_joint", "link_3_wrist_joint", "link_3_wrist_to_gripper_base_joint", "gripper"], points: [{positions: [1.57, 1.57, 1.57, 1.57, 1.57, 1.57], time_from_start: {sec: 2, nanosec: 0}}]}'],
        output='screen',
        condition=LaunchConfigurationEquals('mode', 'sim')
    )


    # 4. RViz (Only if GUI=true)
    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        condition=IfCondition(gui),
        parameters=[{'use_sim_time': PythonExpression(["'", mode, "' == 'sim'"])}]
    )

    # 5. Web GUI / Real Robot Bridge
    # In REAL mode: We run the bridge to talk to serial.
    # In SIM mode: The user might want the Web GUI to control the SIM robot? 
    #   - If so, the bridge node needs to support SIM mode (which I saw earlier it does via `_simulation_worker`).
    #   - BUT, if we are running the FULL gazebo sim, do we want the bridge's internal sim?
    #   - The bridge seems to have its own internal simulation logic.
    #   - Let's assume for now:
    #       - REAL: Bridge connects to Serial.
    #       - SIM: Bridge enables its `simulation_mode`. AND we might also run Gazebo?
    #       - Wait, if we run Gazebo, we have ros2_control. The bridge node publishes joint_states too.
    #       - Conflict! `publish_joint_states` param needs to be False if Gazebo is running joint_state_broadcaster.
    
    # Let's decide:
    # If mode=sim (Gazebo):
    #   - Run Gazebo + Controllers.
    #   - Run Web GUI Node (for the Flask UI).
    #   - Configure Web GUI Node: publish_joint_states=False (let Gazebo do it).
    #   - AND tell Web GUI Node it's in a "ROS Control" mode? 
    #   - The generic bridge node seems to have a `simulation_mode` enabling its setpoint generator.
    #   - If we use Gazebo, we want the Web GUI to send commands to `/joint_targets` or similar? 
    #   - The bridge node listens to `joint_targets`.
    #   - The bridge node in "Sim Mode" does internal integration.
    #   - If we want Gazebo, we probably shouldn't use the bridge's internal sim.
    #   - However, for this task, the user asked to "run the web_Gui with it".
    #   - I'll launch the web_gui_node in all cases, but configure it.
    
    web_gui_arguments = []
    # If gui=false, pass --no-gui
    # logic: if gui==false -> args=['--no-gui']
    
    # We need a conditional node based on python logic for arguments, or use PythonExpression
    
    # Complex logic for bridge parameters
    # If mode == sim: publish_joint_states = False (Gazebo does it)
    # If mode == real: publish_joint_states = True (Bridge does it)
    
    bridge_params = [{
        'publish_joint_states': PythonExpression(["'true' if '", mode, "' != 'sim' else 'false'"]),
        'sync_gazebo': PythonExpression(["'true' if '", mode, "' == 'real' else 'false'"]),
        'serial_port': '/dev/ttyUSB0' # Default
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
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
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
      
        node_rviz,
        web_node
    ])