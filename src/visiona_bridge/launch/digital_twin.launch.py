#!/usr/bin/env python3
"""Digital Twin launch — twin agent + optional RViz."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    backend_arg = DeclareLaunchArgument(
        "twin_backend",
        default_value="bridge_sim",
        description="Twin backend: bridge_sim, rviz_only, gazebo",
    )
    gate_arg = DeclareLaunchArgument(
        "gate_motion",
        default_value="false",
        description="Gate motions through twin validation before execution",
    )

    twin_node = Node(
        package="visiona_bridge",
        executable="uraf_digital_twin",
        name="uraf_digital_twin",
        output="screen",
        parameters=[{
            "backend": LaunchConfiguration("twin_backend"),
            "pre_validate": True,
            "gate_motion": LaunchConfiguration("gate_motion"),
        }],
    )

    return LaunchDescription([
        backend_arg,
        gate_arg,
        LogInfo(msg="Digital Twin agent starting"),
        twin_node,
    ])
