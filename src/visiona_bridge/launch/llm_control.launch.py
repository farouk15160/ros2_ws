#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("vision_llm",    default_value="true",                       description="Enable VLM detection"),
        DeclareLaunchArgument("planner_model", default_value="mistral",                    description="Ollama planner model"),
        DeclareLaunchArgument("vision_model",  default_value="llava",                      description="Ollama VLM model"),
        DeclareLaunchArgument("ollama_url",    default_value="http://localhost:11434",     description="Ollama URL"),
        DeclareLaunchArgument("sam_checkpoint",default_value="/home/farouk/mobile_sam.pt",description="SAM checkpoint path"),
        DeclareLaunchArgument("planning_mode", default_value="auto",
                              description="Motion backend: simple_ik, moveit, auto"),
        LogInfo(msg=[
            "\n==============================================================",
            "\n  JARVIS AI Pipeline starting",
            "\n  Planner  : ", LaunchConfiguration("planner_model"),
            "\n  Vision   : ", LaunchConfiguration("vision_model"),
            "\n  Ollama   : ", LaunchConfiguration("ollama_url"),
            "\n  VLM on   : ", LaunchConfiguration("vision_llm"),
            "\n  SAM      : ", LaunchConfiguration("sam_checkpoint"),
            "\n==============================================================",
        ]),
        Node(package="visiona_bridge", executable="jarvis_object_detector",  name="jarvis_object_detector",
             parameters=[{"ollama_url": LaunchConfiguration("ollama_url"), "vision_model": LaunchConfiguration("vision_model")}],
             condition=IfCondition(LaunchConfiguration("vision_llm")), output="screen", emulate_tty=True),
        Node(package="visiona_bridge", executable="jarvis_segmentation",     name="jarvis_segmentation",
             parameters=[{"sam_checkpoint": LaunchConfiguration("sam_checkpoint")}],
             condition=IfCondition(LaunchConfiguration("vision_llm")), output="screen", emulate_tty=True),
        Node(package="visiona_bridge", executable="jarvis_pose_estimator",   name="jarvis_pose_estimator",
             output="screen", emulate_tty=True),
        Node(package="visiona_bridge", executable="jarvis_world_model",      name="jarvis_world_model",
             parameters=[{"forget_timeout_sec": 15.0}], output="screen", emulate_tty=True),
        Node(package="visiona_bridge", executable="jarvis_llm_planner",      name="jarvis_llm_planner",
             parameters=[{"ollama_url": LaunchConfiguration("ollama_url"),
                          "planner_model": LaunchConfiguration("planner_model"),
                          "temperature": 0.1, "request_timeout": 120}],
             output="screen", emulate_tty=True),
        Node(package="visiona_bridge", executable="jarvis_action_executor",  name="jarvis_action_executor",
             parameters=[{"step_delay_sec": 0.3, "approach_timeout": 8.0, "grasp_height_offset": 0.05,
                          "planning_mode": LaunchConfiguration("planning_mode"),
                          "moveit_group": "arm", "moveit_ee_link": "gripper_base"}],
             output="screen", emulate_tty=True),
        Node(package="visiona_bridge", executable="visual_servo_node",       name="visual_servo_controller",
             parameters=[{"drift_threshold": 0.02, "check_rate_hz": 5.0, "max_corrections": 10}],
             output="screen", emulate_tty=True),
    ])
