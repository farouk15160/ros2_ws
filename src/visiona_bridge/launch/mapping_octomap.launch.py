#!/usr/bin/env python3
"""
Octomap 3D Mapping Launch for Visiona Robot Arm
Builds a 3D voxel map from the depth camera as the arm moves.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.02',
        description='Octomap resolution in meters (0.02 = 2cm voxels)'
    )
    
    max_range_arg = DeclareLaunchArgument(
        'max_range',
        default_value='2.0',
        description='Maximum sensor range in meters (match camera depth limit)'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='world',
        description='Fixed frame for the map'
    )

    # Octomap Server Node
    octomap_server = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[{
            'resolution': LaunchConfiguration('resolution'),
            'frame_id': LaunchConfiguration('frame_id'),
            'sensor_model/max_range': LaunchConfiguration('max_range'),
            'sensor_model/min': 0.1,  # Min range in meters
            'sensor_model/max': LaunchConfiguration('max_range'),
            'sensor_model/hit': 0.7,  # Probability for occupied voxel
            'sensor_model/miss': 0.4,  # Probability for free voxel
            'sensor_model/min_range': 0.1,
            'filter_ground': False,  # Don't filter ground for arm workspace
            'filter_speckles': True,  # Remove noise
            'compress_map': True,  # Save memory
            'incremental_2D_projection': False,  # We want full 3D
            'height_map': False,  # Full 3D map, not 2D
            'colored_map': False,  # Can enable if you want RGB colors
            'latch': True,  # Keep publishing map
            'occupancy_min_z': -0.5,  # Map volume bounds
            'occupancy_max_z': 1.5,
        }],
        remappings=[
            ('cloud_in', '/ascamera_hp60c/camera_publisher/depth0/points'),
        ]
    )

    return LaunchDescription([
        resolution_arg,
        max_range_arg,
        frame_id_arg,
        octomap_server,
    ])
