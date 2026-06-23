#!/usr/bin/env python3
"""Gazebo simulation launch for Visiona digital twin (mode:=gazebo)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("visiona_bridge")
    xacro = PathJoinSubstitution([pkg, "urdf", "visiona.urdf.xacro"])
    controllers = PathJoinSubstitution([pkg, "config", "visiona_controllers.yaml"])
    world = PathJoinSubstitution([pkg, "world", "test_wordl.world"])

    robot_description = ParameterValue(
        Command([
            "xacro ", xacro,
            " controller_config_path:=", controllers,
            " use_sim:=true",
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=world, description="Gazebo world file"),
        SetEnvironmentVariable("GAZEBO_MODEL_PATH", os.path.join(
            get_package_share_directory("visiona_bridge"), "meshes")),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("gazebo_ros"),
                    "launch",
                    "gazebo.launch.py",
                ])
            ]),
            launch_arguments={"world": LaunchConfiguration("world")}.items(),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        ),
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=["-topic", "robot_description", "-entity", "visiona", "-z", "0.01"],
            output="screen",
        ),
    ])
