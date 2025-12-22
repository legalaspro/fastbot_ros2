import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction


def launch_setup(context, *args, **kwargs):

    robot_name = LaunchConfiguration("robot_name").perform(context)
    robot_file = LaunchConfiguration("robot_file").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_description_topic_name = "/" + robot_name + "_robot_description"
    robot_state_publisher_name = robot_name + "_robot_state_publisher"
    joint_state_topic_name = "/" + robot_name + "/joint_states"

    package_description = "fastbot_description"

    robot_desc_path = os.path.join(
        get_package_share_directory(package_description), "models/urdf/", robot_file
    )
    # Load XACRO file with ARGUMENTS
    robot_desc = xacro.process_file(
        robot_desc_path, mappings={"robot_name": robot_name}
    )

    xml = robot_desc.toxml()

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name=robot_state_publisher_name,
        emulate_tty=True,
        parameters=[{"use_sim_time": use_sim_time, "robot_description": xml}],
        remappings=[
            ("/robot_description", robot_description_topic_name),
            ("/joint_states", joint_state_topic_name),
        ],
        output="screen",
    )

    return [robot_state_publisher_node]


def generate_launch_description():
    robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        default_value="fastbot",
        description="Name of the robot"
    )

    robot_file_arg = DeclareLaunchArgument(
        "robot_file",
        default_value="fastbot_multi_sim.xacro",
        description="URDF/Xacro file to load"
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation time (True for Gazebo, False for real hardware)"
    )

    return LaunchDescription(
        [robot_name_arg, robot_file_arg, use_sim_time_arg, OpaqueFunction(function=launch_setup)]
    )