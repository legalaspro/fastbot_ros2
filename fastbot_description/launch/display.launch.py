#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        default_value="fastbot",
        description="Name of the robot"
    )

    # Get launch configuration
    robot_name = LaunchConfiguration("robot_name")

    # Path to RViz config
    package_description = "fastbot_description"
    rviz_config_path = os.path.join(
        get_package_share_directory(package_description), "rviz", "fastbot.rviz"
    )

    # RViz Node with joint_states remapping
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_path],
        remappings=[
            ("/joint_states", ["/", robot_name, "/joint_states"]),
        ],
        output="screen",
    )

    return LaunchDescription([
        robot_name_arg,
        rviz_node,
    ])

