from launch import LaunchDescription
from launch_ros.actions import Node 
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_path = get_package_share_directory('yahboomcar_slam')
    
    # Launch ascamera instead of yahboomcar_bringup
    try:
        ascamera_share = get_package_share_directory('ascamera')
        camera_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(ascamera_share, 'launch'),
                '/hp60c.launch.py'])
        )
    except:
        camera_launch = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='dummy_camera',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'camera_link']
        )
    
    orbslam_pose_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(package_path, 'launch'),
        '/orbslam_pose_launch.py'])
    )
    
    # camera_link是相机本身的tf, camera是orbslam pose发布的
    tf_camera_link_to_camera = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0', '0', '0', '1.57', '3.14', '1.57', 'camera_link', 'camera']
    )
    
    return LaunchDescription([
        camera_launch,
        orbslam_pose_launch,
        tf_camera_link_to_camera
    ])