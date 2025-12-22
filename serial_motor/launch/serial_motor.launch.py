from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction


def launch_setup(context, *args, **kwargs):
    """
    Sets up and returns the configuration for a ROS 2 node based on launch arguments.

    Args:
        context: Launch context that provides access to the launch arguments.
        *args: Additional arguments (unused).
        **kwargs: Additional keyword arguments (unused).

    Returns:
        List of Node actions to be launched.
    """

    # Access top-level launch arguments defined in generate_launch_description.
    robot_name = LaunchConfiguration("robot_name")
    serial_port = LaunchConfiguration("serial_port").perform(context)
    serial_debug_str = LaunchConfiguration("serial_debug").perform(context)
    serial_debug = serial_debug_str.lower() in ['true', '1', 'yes']
    baud_rate = int(LaunchConfiguration("baud_rate").perform(context))
    loop_rate = int(LaunchConfiguration("loop_rate").perform(context))
    encoder_cpr = int(LaunchConfiguration("encoder_cpr").perform(context))

    # Define a ROS 2 Node for the motor driver with parameters such as the serial port and baud rate.
    driver_node = Node(
        package="serial_motor",
        executable="motor_driver",
        name="serial_motor_driver",
        namespace=robot_name,
        parameters=[
            {
                "serial_debug": serial_debug,
                "serial_port": serial_port,  # Port where the device is connected.
                "baud_rate": baud_rate,  # Communication speed for the serial connection.
                "loop_rate": loop_rate,  # Frequency (Hz) at which PID loop spins.
                "encoder_cpr": encoder_cpr,  # Encoder counts per revolution.
            }
        ],
        arguments=["-robot_name", robot_name],
        # remappings=[('/input/topic', '/output/topic')],
        output="screen",  # Output node logs to the screen.
    )

    # Return the driver node as a list for integration with launch description.
    return [driver_node]


def generate_launch_description():
    """
    Generates the launch description and declares required launch arguments.

    Returns:
        LaunchDescription: The launch configuration with arguments and nodes.
    """

    # Declare a launch argument for the robot's name with a default value.
    robot_name_arg = DeclareLaunchArgument("robot_name", default_value="fastbot")
    serial_port_arg = DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0")
    serial_debug_arg = DeclareLaunchArgument("serial_debug", default_value="False")
    baud_rate_arg = DeclareLaunchArgument("baud_rate", default_value="57600")
    loop_rate_arg = DeclareLaunchArgument("loop_rate", default_value="30")
    encoder_cpr_arg = DeclareLaunchArgument("encoder_cpr", default_value="2500")

    # Create a LaunchDescription with the argument and setup function.
    return LaunchDescription(
        [
            robot_name_arg,
            serial_port_arg,
            serial_debug_arg,
            baud_rate_arg,
            loop_rate_arg,
            encoder_cpr_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )