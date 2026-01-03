#!/usr/bin/env python3
"""
HIGH ACCURACY Octomap Launch for Visiona Robot Arm
Maximum detail (1cm voxels) with tight probability bounds.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.005',  # 5mm ULTRA HIGH ACCURACY
        description='Octomap resolution in meters'
    )
    
    max_range_arg = DeclareLaunchArgument(
        'max_range',
        default_value='1.8',  # Conservative (camera max is 2m)
        description='Maximum sensor range in meters'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='world',
        description='Fixed frame for the map'
    )

    # High-Accuracy Octomap Server
    octomap_server = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server_hd',  # HD = High Definition
        output='screen',
        parameters=[{
            # MAXIMUM RESOLUTION
            'resolution': LaunchConfiguration('resolution'),
            'frame_id': LaunchConfiguration('frame_id'),
            
            # SENSOR MODEL - Strong confidence
            'sensor_model/max_range': LaunchConfiguration('max_range'),
            'sensor_model/min': 0.15,  # Ignore very close (noise)
            'sensor_model/hit': 0.85,  # HIGH confidence in detections
            'sensor_model/miss': 0.35,  # HIGH confidence in free space
            
            # CLAMPING - Tight bounds
            'occupancy_min_clamp': 0.08,  # Almost certain free
            'occupancy_max_clamp': 0.98,  # Almost certain occupied
            
            # FILTERING
            'filter_ground': False,  # Keep ground (workspace on table)
            'filter_speckles': True,  # Remove noise
            'compress_map': True,  # Save memory
            
            # MAP PROPERTIES
            'latch': True,  # Keep publishing
            'incremental_2D_projection': False,  # Full 3D
            'height_map': False,
            'colored_map': False,  # Can set True for RGB
            
            # WORKSPACE BOUNDS 
            'occupancy_min_z': -0.3,  # Table ~30cm below world
            'occupancy_max_z': 1.2,   # Max reach height
            
            # PERFORMANCE
            'max_depth': 16,  # Octree depth (higher = more detail)
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
