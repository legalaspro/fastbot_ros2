import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    ThisLaunchFileDir,
    PathJoinSubstitution,
)


def generate_launch_description():
    # Resolve the path to the XML launch files
    dir_path = ThisLaunchFileDir()
    launch_first_path = PathJoinSubstitution([dir_path, "botbox_world.launch.xml"])
    launch_second_path = PathJoinSubstitution([dir_path, "spawn_fastbot.launch.xml"])
    # Include the XML launch descriptions
    launch_first_action = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(launch_first_path),
        launch_arguments={"arg": "value"}.items(),
    )

    launch_second_action = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(launch_second_path),
        launch_arguments={"arg": "value"}.items(),
    )

    # Timer action to delay the launch of the second XML file
    delay_second_launch = TimerAction(
        period=2.0, actions=[launch_second_action]  # Delay in seconds
    )

    # Create and return the launch description
    return LaunchDescription([launch_first_action, delay_second_launch])