#!/usr/bin/env python3
"""
Simplified ORB-SLAM2 test launch file
Tests camera streaming and point cloud visualization without ORB-SLAM2
"""
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_path = get_package_share_directory('yahboomcar_slam')
    
    # Point cloud mapping config
    pointcloud_config = os.path.join(package_path, 'params', 'pointcloud_map.yaml')
    
    # TF publishers
    tf_world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_world_to_camera',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'camera']
    )
    
    # Point cloud mapping node
    pcl_node = Node(
        package='yahboomcar_slam',
        executable='point_cloud_mapping',
        name='pointcloud_mapping_node',
        output='screen',
        parameters=[pointcloud_config],
        # Add debug output
        prefix=['gdb -ex run --args'] if False else ''  # Set to True for debugging
    )
    
    return LaunchDescription([
        tf_world,
        pcl_node
    ])
