# ROS2 Motor Control

This step integrates the Arduino motor controller with ROS2 using the `serial_motor` package. Now you can send `cmd_vel` commands from ROS2 to control the robot's movement.

## Overview

The `serial_motor` package provides:

- ROS2 node that communicates with Arduino via serial
- Subscribes to `/fastbot/cmd_vel` topic (namespaced velocity commands)
- Translates velocity commands to motor control signals
- Publishes encoder feedback and motor velocities
- Publishes joint states for robot visualization
- Uses `serial_motor_msgs` for custom message types

### Published Topics

- `/fastbot/joint_states` - Joint states for wheel visualization (sensor_msgs/JointState)
- `/fastbot/motor_vels` - Motor velocities in rad/s (serial_motor_msgs/MotorVels)
- `/fastbot/encoder_vals` - Raw encoder values (serial_motor_msgs/EncoderVals)
- `/fastbot/odom` - Odometry data (nav_msgs/Odometry) [when enabled]

### Subscribed Topics

- `/fastbot/cmd_vel` - Velocity commands (geometry_msgs/Twist)

## Setup

### 1. Build the Workspace

On your Raspberry Pi, navigate to the workspace and build:

```bash
cd ~/fastbot_ros2
colcon build --symlink-install
source install/setup.bash
```

### 2. Launch Serial Motor Node

Use the provided launch file to start the serial motor node:

```bash
ros2 launch serial_motor serial_motor.launch.py
```

Or use the complete bringup which includes all sensors:

```bash
ros2 launch fastbot_bringup bringup.launch.xml
```

This will:

- Connect to the Arduino at `/dev/arduino_nano`
- Start listening for `/fastbot/cmd_vel` commands
- Publish joint states to `/fastbot/joint_states`
- Publish encoder values and motor velocities
- Enable robot visualization in RViz

## Testing

### Test with Teleop Keyboard

Control the robot using keyboard:

```bash
# In a new terminal
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/fastbot/cmd_vel
```

Use the keyboard controls:

- `i` - Move forward
- `,` - Move backward
- `j` - Turn left
- `l` - Turn right
- `k` - Stop
- `q/z` - Increase/decrease speed

## Verify Communication

Check that everything is working:

```bash
# List active topics
ros2 topic list

# Monitor cmd_vel messages
ros2 topic echo /fastbot/cmd_vel

# Check joint states (should show wheel velocities)
ros2 topic echo /fastbot/joint_states

# Monitor encoder values
ros2 topic echo /fastbot/encoder_vals

# Monitor motor velocities
ros2 topic echo /fastbot/motor_vels

# Check node info
ros2 node info /fastbot/serial_motor_driver
```

### Visualize in RViz

After launching the motor driver, you can visualize the robot in RViz:

```bash
# In a new terminal
ros2 launch fastbot_description display.launch.py
```

This will show:

- Robot model with moving wheels (based on joint states)
- TF frames
- Real-time wheel joint positions and velocities

## Configuration

The serial motor node can be configured in the launch file (`serial_motor/launch/serial_motor.launch.py`):

### Launch Arguments

- `robot_name` (default: `"fastbot"`) - Robot namespace for topics
- `serial_port` (default: `"/dev/arduino_nano"`) - Arduino serial port
- `baud_rate` (default: `57600`) - Serial communication baud rate
- `loop_rate` (default: `30`) - Control loop frequency in Hz
- `encoder_cpr` (default: `1920`) - Encoder counts per revolution
- `serial_debug` (default: `False`) - Enable serial debug output

### Example with Custom Parameters

```bash
ros2 launch serial_motor serial_motor.launch.py \
    robot_name:=my_robot \
    serial_port:=/dev/ttyUSB0 \
    baud_rate:=115200 \
    loop_rate:=50
```

### Motor Driver Features

The motor driver node (`motor_driver.py`) provides:

1. **Differential Drive Kinematics** - Converts cmd_vel to wheel velocities
2. **Encoder Feedback** - Reads encoder values and calculates wheel speeds
3. **Joint State Publishing** - Publishes wheel joint states for visualization
4. **PID Control** - Motor speed control (configured in Arduino firmware)
5. **Namespacing** - All topics are namespaced under the robot name

## Resources

- [geometry_msgs/Twist Documentation](https://docs.ros2.org/latest/api/geometry_msgs/msg/Twist.html)
- [teleop_twist_keyboard Package](https://github.com/ros2/teleop_twist_keyboard)
- [ROS2 Topics Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)
- [sensor_msgs/JointState Documentation](https://docs.ros2.org/latest/api/sensor_msgs/msg/JointState.html)
