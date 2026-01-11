import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction, LogInfo
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    visiona_bridge_share = FindPackageShare('visiona_bridge')
    
    # Launch Arguments
    mode_arg = DeclareLaunchArgument('mode', default_value='real',
                                      description='Mode: real, sim, or gazebo')
    gui_arg = DeclareLaunchArgument('gui', default_value='true',
                                     description='Launch web GUI')
    
    viz_arg = DeclareLaunchArgument(
        'viz',
        default_value='none',
        choices=['none', 'rviz'],
        description='Visualization mode: none, rviz'
    )
    
    camera_arg = DeclareLaunchArgument('camera', default_value='false',
                                        description='Launch camera driver')
    mapping_arg = DeclareLaunchArgument('mapping', default_value='low',
                                         choices=['low', 'high'],
                                         description='Mapping quality: low (2cm, fast) or high (5mm, accurate)')
    
    # NEW: LLM/VLA Control option
    llm_arg = DeclareLaunchArgument('llm', default_value='false',
                                    description='Enable LLM/VLA natural language control')
    language_arg = DeclareLaunchArgument('language', default_value='en',
                                         choices=['en', 'de'],
                                         description='LLM language (English or German)')
    
    mode = LaunchConfiguration('mode')
    gui = LaunchConfiguration('gui')
    viz = LaunchConfiguration('viz')
    camera = LaunchConfiguration('camera')
    mapping_quality = LaunchConfiguration('mapping')
    llm = LaunchConfiguration('llm')
    language = LaunchConfiguration('language')
    
    # Path to Xacro file
    xacro_file = PathJoinSubstitution([visiona_bridge_share, 'urdf', 'visiona.urdf.xacro'])
    
    # Process Xacro to XML and wrap as string parameter
    robot_description_content = Command(['xacro ', xacro_file])
    robot_description_param = ParameterValue(robot_description_content, value_type=str)
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_param,
            'use_sim_time': False
        }]
    )
    
    # ros2_control Node
    # Note: We pass the parsed XML content, not the file path
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description_param},
            PathJoinSubstitution([visiona_bridge_share, 'config', 'visiona_controllers.yaml'])
        ],
        output='screen'
    )
    
    # RViz (basic visualization)
    rviz_config = PathJoinSubstitution([visiona_bridge_share, 'rviz', 'view_robot.rviz'])
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(PythonExpression(["'", viz, "' == 'rviz'"])),
        output='screen'
    )
    
    # Simple IK Solver (always enabled for Cartesian XYZ control)
    simple_ik_node = Node(
        package='visiona_bridge',
        executable='simple_ik_solver',
        name='simple_ik_solver',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                visiona_bridge_share,
                'config',
                'simple_ik_config.yaml'
            ])
        ]
    )
    
    # LLM Control System
    llm_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([visiona_bridge_share, 'launch', 'llm_control.launch.py'])
        ]),
        launch_arguments={
            'language': language,
        }.items(),
        condition=IfCondition(llm)
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
    
    # Static TF: map → world (RTAB-Map uses 'map' frame, robot uses 'world')
    map_to_world_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_world_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        condition=IfCondition(camera)
    )
    
    # Static TF: odom → base_link (for odometry, identity when stationary arm)
    odom_to_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        condition=IfCondition(camera)
    )
    
    # Static TF: odom → base_link (required by RTAB-Map)
    base_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
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
    
    # ========== 3D MAPPING WITH OCTOMAP (Camera mode only) ==========
    # Octomap builds a 3D voxel map as the arm moves - perfect for stationary arms!
    # Quality can be set via mapping:=low or mapping:=high
    octomap_server = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        condition=LaunchConfigurationEquals('camera', 'true'),
        parameters=[{
            # RESOLUTION: Conditional based on mapping quality
            'resolution': PythonExpression([
                "'0.015' if '", mapping_quality, "' == 'high' else '0.02'"
            ]),
            'frame_id': 'world',
            
            # SENSOR MODEL: Conditional based on mapping quality
            'sensor_model/max_range': PythonExpression([
                "1.8 if '", mapping_quality, "' == 'high' else 2.0"
            ]),
            'sensor_model/min': PythonExpression([
                "0.15 if '", mapping_quality, "' == 'high' else 0.1"
            ]),
            'sensor_model/hit': PythonExpression([
                "0.8 if '", mapping_quality, "' == 'high' else 0.7"
            ]),
            'sensor_model/miss': PythonExpression([
                "0.35 if '", mapping_quality, "' == 'high' else 0.4"
            ]),
            
            # COMMON SETTINGS
            'filter_ground': False,  # Keep ground in workspace
            'filter_speckles': True,  # Remove noise
            'compress_map': True,  # Save memory
            'latch': True,  # Keep publishing map
            'occupancy_min_z': -0.5,  # Workspace bounds
            'occupancy_max_z': 1.5,
        }],
        remappings=[
            ('cloud_in', '/ascamera_hp60c/camera_publisher/depth0/points'),
        ]
    )
    
    # ========== RTAB-MAP for COLORED 3D MAPPING ==========
    # Creates dense RGB point cloud maps with colors for realistic visualization
    # and VLA/autonomous navigation with obstacle avoidance
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        condition=LaunchConfigurationEquals('camera', 'true'),
        parameters=[{
            # Database - save map to file for persistence
            'database_path': '~/.ros/rtabmap.db',  # Save map here
            'delete_db_on_start': False,  # Keep previous map!
            
            # Frame configuration
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'approx_sync': True,  # Sync RGB and Depth
            'queue_size': 10,
            
            # Visual odometry settings
            'Vis/MinInliers': '15',
            'Vis/InlierDistance': '0.1',
            
            # Map settings for colored point clouds
            'Grid/FromDepth': 'true',  # Build occupancy grid from depth
            'Grid/RangeMax': '2.0',    # Max sensing range
            'Grid/RangeMin': '0.1',    # Min sensing range
            
            # Memory management
            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes': 'false',
            
            # Point cloud generation with colors
            'Rtabmap/CreateIntermediateNodes': 'true',
            'RGBD/CreateOccupancyGrid': 'true',
        }],
        remappings=[
            # Use actual camera publisher topics (check with: ros2 topic list | grep ascamera)
            ('rgb/image', '/ascamera_hp60c/camera_publisher/rgb0/image'),
            ('rgb/camera_info', '/ascamera_hp60c/camera_publisher/rgb0/camera_info'),
            ('depth/image', '/ascamera_hp60c/camera_publisher/depth0/image_raw'),
        ]
    )
    
    # NOTE: rtabmap_viz removed - RViz is sufficient for visualization
    # Use /cloud_map topic in RViz with Color Transformer: RGB8 for colored point cloud
    
    # ==============================================================================
    # Launch Description - Combine all nodes
    # ==============================================================================
    
    return LaunchDescription([
        # Arguments
        mode_arg,
        gui_arg,
        viz_arg,  # Visualization option
        camera_arg,
        mapping_arg,
        llm_arg,  # NEW: LLM control
        language_arg,  # NEW: LLM language
        
        # Core nodes
        robot_state_publisher,
        ros2_control_node,
        delay_joint_state_broadcaster,
        delay_joint_trajectory_controller,
        
        # Camera and TF tree
        camera_launch,
        camera_tf_publisher,
        map_to_world_tf,     # Connect RTAB-Map 'map' to 'world'
        odom_to_base_tf,     # Connect 'map' to 'odom'
        base_link_tf,        # Connect 'odom' to 'base_link'
        
        # Visualization (conditional)
        node_rviz,     # if viz:=rviz
        
        # Simple IK Solver (always enabled)
        simple_ik_node,
        
        # GUI
        web_node,
        
        # 3D Mapping (camera:=true)
        octomap_server,      # Voxel-based occupancy grid
        rtabmap_node,        # Colored RGB point cloud SLAM
        
        # LLM Control (conditional)
        llm_control_launch,  # if llm:=true
    ])