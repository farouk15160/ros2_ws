#!/usr/bin/env python3
"""
LLM Control Launch File

Launches all LLM/VLA control components:
- LLM Task Planner
- VLA Action Generator
- Task Executor
- Visual Servoing Controller

Author: Antigravity AI
Date: 2026-01-04
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    
    pkg_share = FindPackageShare('visiona_bridge')
    
    # Configuration files
    llm_config = PathJoinSubstitution([pkg_share, 'config', 'llm_config.yaml'])
    vla_config = PathJoinSubstitution([pkg_share, 'config', 'vla_config.yaml'])
    servo_config = PathJoinSubstitution([pkg_share, 'config', 'visual_servo_params.yaml'])
    
    # Arguments
    language_arg = DeclareLaunchArgument(
        'language',
        default_value='en',
        description='Default language for LLM (en or de)'
    )
    
    model_dir = os.path.expanduser('~/.cache/models')
    llm_model_arg = DeclareLaunchArgument(
        'llm_model',
        default_value=f'{model_dir}/llama-3.2-3b-q4_k_m.gguf',
        description='Path to LLM model file'
    )
    
    enable_vla_arg = DeclareLaunchArgument(
        'enable_vla',
        default_value='false',
        description='Enable VLA (requires GPU and large model download)'
    )
    
    # Nodes - LLM needs special environment
    # Use a wrapper approach since llama-cpp-python is in a venv
    llm_planner = Node(
        package='visiona_bridge',
        executable='llm_task_planner',
        name='llm_task_planner',
        parameters=[
            llm_config,
            {
                'model_path': LaunchConfiguration('llm_model'),
                'language': LaunchConfiguration('language')
            }
        ],
        output='screen',
        prefix='bash -c "source /home/farouk/.venv/llm/bin/activate && exec python3 ',
        suffix='"'
    )
    
    vla_generator = Node(
        package='visiona_bridge',
        executable='vla_action_generator',
        name='vla_action_generator',
        parameters=[vla_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_vla'))
    )
    
    visual_servo = Node(
        package='visiona_bridge',
        executable='visual_servo_node',
        name='visual_servo_controller',
        parameters=[servo_config],
        output='screen'
    )
    
    task_executor = Node(
        package='visiona_bridge',
        executable='task_executor',
        name='task_executor',
        output='screen'
    )
    
    # Info message
    info_msg = LogInfo(
        msg=[
            '\n',
            '========================================\n',
            'LLM/VLA Control System Started\n',
            '========================================\n',
            'Language: ', LaunchConfiguration('language'), '\n',
            'Model: ', LaunchConfiguration('llm_model'), '\n',
            '\n',
            'Send commands to: /llm/command\n',
            'Example: ros2 topic pub --once /llm/command std_msgs/String "data: \'Pick up the red cube\'"\n',
            '========================================\n'
        ]
    )
    
    return LaunchDescription([
        language_arg,
        llm_model_arg,
        enable_vla_arg,
        info_msg,
        llm_planner,
        vla_generator,
        visual_servo,
        task_executor,
    ])
