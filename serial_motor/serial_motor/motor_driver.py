#!/usr/bin/env python3

import time
import math
import rclpy
import serial
import argparse
from threading import Lock
from rclpy.node import Node
from typing import List, Optional
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from serial_motor_msgs.msg import MotorVels, EncoderVals


class MotorDriver(Node):
    """ROS2 Node for controlling and monitoring a differential drive robot."""

    def __init__(self, args) -> None:
        """Initialize the MotorDriver node with serial communication, parameters, and publishers/subscribers."""
        super().__init__("motor_driver")
        self.argument_parsing(args)

        # Logger
        self._logger = self.get_logger()

        # Setup encoder and loop rate parameters; log error if unset or zero.
        self.declare_parameter("encoder_cpr", value=0)
        self.encoder_cpr = self.get_parameter("encoder_cpr").value
        if not self.encoder_cpr:
            self._logger.error("Encoder CPR must be set to a non-zero value!")
            raise ValueError("Encoder CPR is required and must be non-zero.")

        self.declare_parameter("loop_rate", value=0)
        self.loop_rate = self.get_parameter("loop_rate").value
        if not self.loop_rate:
            self._logger.error("Loop rate must be set to a non-zero value!")
            raise ValueError("Loop rate is required and must be non-zero.")

        # Initialize serial port parameters and optional debugging.
        self.declare_parameter("serial_port", value="/dev/ttyACM0")
        self.serial_port: str = self.get_parameter("serial_port").value
        self.declare_parameter("baud_rate", value=57600)
        self.baud_rate: int = self.get_parameter("baud_rate").value
        self.declare_parameter("serial_debug", value=False)
        self.debug_serial_cmds: bool = self.get_parameter("serial_debug").value

        if self.debug_serial_cmds:
            self._logger.info("Serial debug enabled")

        # Kinematic parameters for differential drive.
        self.declare_parameter("wheel_diameter", value=0.065)
        self.wheel_diameter = self.get_parameter("wheel_diameter").value
        self.declare_parameter("wheel_separation", value=0.17)
        self.wheel_separation = self.get_parameter("wheel_separation").value
        self.wheel_radius = self.wheel_diameter / 2

        # ROS 2 publishers and subscribers
        # Reentrant callback group allows concurrent execution
        self.callback_group = ReentrantCallbackGroup()

        self._sub_cmd_vel = self.create_subscription(
            Twist,
            "cmd_vel",
            self.cmd_vel_callback,
            10,
            callback_group=self.callback_group,
        )
        self.motor_vels_pub_ = self.create_publisher(MotorVels, "motor_vels", 10)
        self.encoder_pub_ = self.create_publisher(EncoderVals, "encoder_vals", 10)
        self.odom_pub_ = self.create_publisher(Odometry, "odom", 10)

        # Timer callback to continuously publish odometry
        self.create_timer(0.1, self._timer_callback, callback_group=self.callback_group)

        # Initialize encoder and speed tracking variables.
        self.last_enc_read_time = time.time()
        self.last_m1_enc = 0
        self.last_m2_enc = 0
        self.m1_spd = 0.0
        self.m2_spd = 0.0
        self.mutex = Lock()  # Mutex to ensure thread-safe serial communication.

        # Initialize odometry variables
        self.x = 0.0  # Position in x
        self.y = 0.0  # Position in y
        self.theta = 0.0  # Orientation (yaw)
        self.last_time = time.time()  # Last update time

        # Attempt to establish a serial connection; log error if unsuccessful.
        try:
            self._logger.info(
                f"Connecting to port {self.serial_port} at {self.baud_rate}."
            )
            self.conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1.0)
            self._logger.info(f"Connected to {self.conn}")
        except serial.SerialException as e:
            self._logger.error(f"Failed to connect to {self.serial_port}: {e}")
            raise

    def argument_parsing(self, args):
        parser = argparse.ArgumentParser(description="Arguments for frame names.")
        parser.add_argument(
            "-robot_name_value",
            type=str,
            metavar="botbox_default",
            default="fastbot_X",
            help="Name of the robot",
        )
        self.args = parser.parse_args(args[1:])

    def send_pwm_motor_command(self, mot_1_pwm: float, mot_2_pwm: float) -> None:
        """Send PWM command to set motor speeds.

        Args:
            mot_1_pwm (float): PWM value for motor 1.
            mot_2_pwm (float): PWM value for motor 2.
        """
        self.send_command(f"o {int(mot_1_pwm)} {int(mot_2_pwm)}")

    def send_feedback_motor_command(
        self, mot_1_ct_per_loop: float, mot_2_ct_per_loop: float
    ) -> None:
        """Send feedback control command with counts per loop for motors.

        Args:
            mot_1_ct_per_loop (float): Encoder count per loop for motor 1.
            mot_2_ct_per_loop (float): Encoder count per loop for motor 2.
        """
        self.send_command(f"m {int(mot_1_ct_per_loop)} {int(mot_2_ct_per_loop)}")

    def send_encoder_read_command(self) -> List[int]:
        """Send command to retrieve encoder values from motors.

        Returns:
            List[int]: A list containing encoder values for both motors.
        """
        resp = self.send_command("e")
        if resp:
            try:
                return [int(raw_enc) for raw_enc in resp.split()]
            except ValueError:
                self._logger.error("Failed to parse encoder values.")
        return []

    def _timer_callback(self) -> None:
        """Timer callback to periodically publish the current odometry."""
        self.check_encoders()

    def cmd_vel_callback(self, msg: Twist) -> None:
        """Callback to handle /cmd_vel Twist messages for motor speed control.

        Args:
            msg (Twist): ROS2 Twist message with linear and angular velocity.
        """
        # Extract linear and angular velocities from the cmd_vel message.
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        # Calculate wheel velocities in m/s for differential drive kinematics.
        left_wheel_speed = linear_vel - (angular_vel * self.wheel_separation) / 2
        right_wheel_speed = linear_vel + (angular_vel * self.wheel_separation) / 2

        # Avoid divide by zero in wheel radius
        if self.wheel_radius == 0:
            self._logger.error(
                "Wheel radius is zero, cannot calculate angular velocities."
            )
            return

        # Convert wheel linear speeds (m/s) to angular velocities (rad/s).
        left_wheel_rad_per_sec = left_wheel_speed / self.wheel_radius
        right_wheel_rad_per_sec = right_wheel_speed / self.wheel_radius

        # Log wheel speeds for debugging purposes.
        self._logger.info(
            f"cmd_vel -> Linear: {linear_vel} m/s, Angular: {angular_vel} rad/s | "
            f"Left wheel: {left_wheel_rad_per_sec:.2f} rad/s, Right wheel: {right_wheel_rad_per_sec:.2f} rad/s"
        )

        # Calculate counts per loop for feedback control based on kinematic conversion.
        scaler = (1 / (2 * math.pi)) * self.encoder_cpr * (1 / self.loop_rate)
        mot_1_ct_per_loop = left_wheel_rad_per_sec * scaler
        mot_2_ct_per_loop = right_wheel_rad_per_sec * scaler

        # Check if counts per loop are finite before sending
        if math.isfinite(mot_1_ct_per_loop) and math.isfinite(mot_2_ct_per_loop):
            self.send_feedback_motor_command(mot_1_ct_per_loop, mot_2_ct_per_loop)
        else:
            self._logger.warning(
                "Non-finite motor count detected, skipping command send."
            )

    def check_encoders(self) -> None:
        """Reads encoder values, calculates speed, and publishes encoder readings and motor speeds."""
        resp = self.send_encoder_read_command()
        if resp:
            # Calculate time elapsed since the last encoder read.
            new_time = time.time()
            time_diff = new_time - self.last_enc_read_time
            self.last_enc_read_time = new_time

            # Determine encoder change and speed for each motor.
            m1_diff = resp[0] - self.last_m1_enc
            self.last_m1_enc = resp[0]
            m2_diff = resp[1] - self.last_m2_enc
            self.last_m2_enc = resp[1]

            # Convert encoder counts to radian speeds.
            rads_per_ct = 2 * math.pi / self.encoder_cpr
            self.m1_spd = m1_diff * rads_per_ct / time_diff
            self.m2_spd = m2_diff * rads_per_ct / time_diff

            # Publish the motor velocities.
            spd_msg = MotorVels()
            spd_msg.mot_1_rad_sec = self.m1_spd
            spd_msg.mot_2_rad_sec = self.m2_spd
            self.motor_vels_pub_.publish(spd_msg)

            # Publish raw encoder values.
            enc_msg = EncoderVals()
            enc_msg.mot_1_enc_val = self.last_m1_enc
            enc_msg.mot_2_enc_val = self.last_m2_enc
            self.encoder_pub_.publish(enc_msg)

            # Publish odometry based on encoder readings
            # self.publish_odometry()

    def publish_odometry(self) -> None:
        """Publish odometry data based on encoder readings."""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        if dt == 0:
            return

        # Calculate distance traveled by each wheel
        left_distance = self.m1_spd * dt * self.wheel_radius
        right_distance = self.m2_spd * dt * self.wheel_radius

        # Calculate average distance and change in orientation
        avg_distance = (left_distance + right_distance) / 2.0
        d_theta = (right_distance - left_distance) / self.wheel_separation

        # Update robot position
        self.theta = (self.theta + d_theta) % (
            2 * math.pi
        )  # Keep theta within [0, 2*pi]
        self.x += avg_distance * math.cos(self.theta)
        self.y += avg_distance * math.sin(self.theta)

        # Create and publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.args.robot_name_value + "_odom"
        odom_msg.child_frame_id = self.args.robot_name_value + "_base_link"

        # Set position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        # Set orientation as quaternion
        quat = self.euler_to_quaternion(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        # Set velocity
        odom_msg.twist.twist.linear.x = avg_distance / dt
        odom_msg.twist.twist.angular.z = d_theta / dt

        # Publish message
        self.odom_pub_.publish(odom_msg)

    def euler_to_quaternion(
        self, roll: float, pitch: float, yaw: float
    ) -> tuple[float, float, float, float]:
        """Convert Euler angles to a quaternion."""
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(
            roll / 2
        ) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(
            roll / 2
        ) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(
            roll / 2
        ) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(
            roll / 2
        ) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return (qx, qy, qz, qw)

    def send_command(self, cmd_string: str) -> Optional[str]:
        """Utility method to send a command to the motor controller via serial.

        Args:
            cmd_string (str): Command string to send to the motor controller.

        Returns:
            Optional[str]: Response from the motor controller, or None if a timeout occurs.
        """
        with self.mutex:
            try:
                cmd_string += "\r"  # Add carriage return for command termination.
                self.conn.write(cmd_string.encode("utf-8"))
                if self.debug_serial_cmds:
                    self._logger.info(f"Sent: {cmd_string}")

                # Read response until carriage return is received.
                response = self.conn.read_until(b"\r").decode("utf-8").strip("\r")
                if not response:
                    self._logger.warning(f"Serial timeout on command: {cmd_string}")
                    return None
                if self.debug_serial_cmds:
                    self._logger.info(f"Received: {response}")
                return response
            except serial.SerialTimeoutException:
                self._logger.error("Serial timeout occurred.")
                return None

    def close_conn(self) -> None:
        """Close the serial connection to the motor controller, ensuring a clean exit."""
        if self.conn.is_open:
            self.conn.close()
            self._logger.info("Serial connection closed.")


def main(args: Optional[List[str]] = None) -> None:
    """Main function to initialize the MotorDriver node and execute the ROS loop."""
    rclpy.init(args=args)

    try:
        args_without_ros = rclpy.utilities.remove_ros_args(args)

        # Create node
        motor_driver = MotorDriver(args_without_ros)

        # Initialize MultiThreadedExecutor with two threads for concurrent callbacks
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(motor_driver)

        try:
            # Spin executor to process callbacks
            executor.spin()
        finally:
            # Ensure executor shuts down gracefully
            executor.shutdown()
            motor_driver.destroy_node()

    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()