import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction, LogInfo
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    visiona_bridge_share = FindPackageShare('visiona_bridge')
    
    # Launch Arguments
    mode_arg = DeclareLaunchArgument('mode', default_value='real',
                                      description='Mode: real, sim, or gazebo')
    gui_arg = DeclareLaunchArgument('gui', default_value='true',
                                     description='Launch web GUI')
    launch_rviz_arg = DeclareLaunchArgument('launch_rviz', default_value='true',
                                             description='Launch RViz')
    camera_arg = DeclareLaunchArgument('camera', default_value='false',
                                        description='Launch camera driver')
    
    mode = LaunchConfiguration('mode')
    gui = LaunchConfiguration('gui')
    launch_rviz = LaunchConfiguration('launch_rviz')
    camera = LaunchConfiguration('camera')
    
    # Path to Xacro file
    xacro_file = PathJoinSubstitution([visiona_bridge_share, 'urdf', 'visiona.urdf.xacro'])
    
    # Process Xacro to XML
    robot_description_content = Command(['xacro ', xacro_file])
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': False
        }]
    )
    
    # ros2_control Node
    # Note: We pass the parsed XML content, not the file path
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description_content},
            PathJoinSubstitution([visiona_bridge_share, 'config', 'visiona_controllers.yaml'])
        ],
        output='screen'
    )
    
    # RViz
    rviz_config = PathJoinSubstitution([visiona_bridge_share, 'rviz', 'view_robot.rviz'])
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(launch_rviz),
        output='screen'
    )
    
    # Camera Launch
    try:
        camera_share = FindPackageShare('ascamera')
        camera_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([camera_share, 'launch', 'hp60c.launch.py'])
            ]),
            condition=IfCondition(camera)
        )
    except Exception:
        camera_launch = LogInfo(msg="Camera package not found, skipping camera launch")
    
    # Static TF to connect camera to robot (camera publishes as ascamera_hp60c_ascamera_0)
    camera_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_robot_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_link', 'ascamera_hp60c_ascamera_0'],
        condition=IfCondition(camera)
    )
    
    # Web GUI / Bridge Node
    web_node = Node(
        package='visiona_bridge',
        executable=sys.executable,
        name='web_gui_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'publish_joint_states': True,  # MUST publish real robot state to RViz
            'sync_gazebo': False,
            'serial_port': '/dev/ttyUSB0',
            'mode': mode,
            'use_sim_time': False
        }],
        arguments=[
            '-u',
            '-c', 'from visiona_bridge.web_gui_node import main; main()',
            PythonExpression(["'--no-gui' if '", gui, "' == 'false' else ''"])
        ]
    )
    
    # Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )
    
    # Joint Trajectory Controller Spawner
    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '-c', '/controller_manager'],
    )
    
    # --- FIX: LOGIC CHANGE ---
    # We want to delay spawners until the robot_state_publisher (or control node) starts.
    # OnProcessExit would wait for the node to DIE before running the spawner.
    
    delay_joint_state_broadcaster = TimerAction(
        period=3.0, # Give controller manager time to start up
        actions=[joint_state_broadcaster_spawner],
        condition=IfCondition(PythonExpression(["'", mode, "' != 'real'"]))
    )
    
    delay_joint_trajectory_controller = TimerAction(
        period=4.0, # Start shortly after state broadcaster
        actions=[joint_trajectory_controller_spawner],
        condition=LaunchConfigurationEquals('mode', 'real')
    )
    
    return LaunchDescription([
        mode_arg,
        gui_arg,
        launch_rviz_arg,
        camera_arg,
        robot_state_publisher,
        ros2_control_node,
        delay_joint_state_broadcaster,
        delay_joint_trajectory_controller,
        camera_launch,
        camera_tf_publisher,
        node_rviz,
        web_node
    ])