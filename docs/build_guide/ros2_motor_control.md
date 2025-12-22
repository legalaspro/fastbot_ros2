# Step 4: ROS2 Motor Control

This step integrates the Arduino motor controller with ROS2 using the `serial_motor` package. Now you can send `cmd_vel` commands from ROS2 to control the robot's movement.

## Overview

The `serial_motor` package provides:

- ROS2 node that communicates with Arduino via serial
- Subscribes to `/cmd_vel` topic (standard ROS2 velocity commands)
- Translates velocity commands to motor control signals
- Uses `serial_motor_msgs` for custom message types

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

This will:

- Connect to the Arduino at `/dev/arduino_nano`
- Start listening for `/cmd_vel` commands
- Begin publishing motor feedback (if configured)

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

# Check node info
ros2 node info /serial_motor
```

## Configuration

The serial motor node can be configured in the launch file:

- Serial port (default: `/dev/arduino_nano`)
- Baud rate (default: `57600`)
- Topic names
- Update rates

See `serial_motor/launch/serial_motor.launch.py` for configuration options.

## Next Steps

With ROS2 motor control working, you're ready to:

- Add sensors (LiDAR, IMU, cameras)
- Implement odometry
- Set up navigation stack
- Create autonomous behaviors

## Resources

- [geometry_msgs/Twist Documentation](https://docs.ros2.org/latest/api/geometry_msgs/msg/Twist.html)
- [teleop_twist_keyboard Package](https://github.com/ros2/teleop_twist_keyboard)
- [ROS2 Topics Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)
