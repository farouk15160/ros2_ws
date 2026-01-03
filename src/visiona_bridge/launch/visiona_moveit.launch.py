#!/usr/bin/env python3
"""
Unified MoveIt Launch File for Visiona Robot
Launches: Robot Hardware + MoveIt Planning + RViz Visualization
All in one command!
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='real',
        description='Mode: real, sim, or gazebo'
    )
    
    camera_arg = DeclareLaunchArgument(
        'camera',
        default_value='true',
        description='Launch camera and Octomap'
    )
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz visualization'
    )
    
    # Get launch configurations
    mode = LaunchConfiguration('mode')
    camera = LaunchConfiguration('camera')
    launch_rviz = LaunchConfiguration('rviz')
    
    # ==============================================================================
    # 1. Robot Hardware Launch (visiona_bridge)
    # ==============================================================================
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('visiona_bridge'),
                'launch',
                'spawn_visiona.launch.py'
            ])
        ]),
        launch_arguments={
            'mode': mode,
            'camera': camera,
            'launch_rviz': 'false',  # We'll launch MoveIt RViz instead
            'gui': 'true',  # Keep web GUI
        }.items()
    )
    
    # ==============================================================================
    # 2. MoveIt Planning Server (move_group)
    # ==============================================================================
    # Wait 5 seconds for robot and controllers to initialize
    moveit_launch = TimerAction(
        period=5.0,  # Increased from 3.0 to ensure controllers are loaded
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('visiona_moveit_config'),
                        'launch',
                        'move_group.launch.py'
                    ])
                ])
            )
        ]
    )
    
    # ==============================================================================
    # 3. RViz Visualization (moveit_rviz)
    # ==============================================================================
    # Wait 7 seconds total for M oveIt to initialize
    rviz_launch = TimerAction(
        period=7.0,  # Increased to ensure MoveIt is ready
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('visiona_moveit_config'),
                        'launch',
                        'moveit_rviz.launch.py'
                    ])
                ])
            )
        ]
    )
    
    return LaunchDescription([
        mode_arg,
        camera_arg,
        rviz_arg,
        robot_launch,      # Launches immediately
        moveit_launch,     # Launches after 3 seconds
        rviz_launch,       # Launches after 5 seconds
    ])
