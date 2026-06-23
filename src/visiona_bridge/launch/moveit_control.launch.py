#!/usr/bin/env python3
"""MoveIt 2 stack for Visiona — move_group + optional RViz."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    moveit_share = FindPackageShare("visiona_moveit_config")

    viz_arg = DeclareLaunchArgument(
        "viz",
        default_value="none",
        description="MoveIt viz: none or moveit (MoveIt RViz)",
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([moveit_share, "launch", "move_group.launch.py"]),
        ]),
    )

    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([moveit_share, "launch", "moveit_rviz.launch.py"]),
        ]),
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("viz"), "' == 'moveit'"])
        ),
    )

    return LaunchDescription([
        viz_arg,
        LogInfo(msg="MoveIt move_group starting (joint_trajectory_controller)"),
        move_group_launch,
        moveit_rviz_launch,
    ])
