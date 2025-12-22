# Fastbot Gazebo

Gazebo Classic simulation environment for the Fastbot robot.

## Launch Simulation

Launch the Fastbot robot in a warehouse environment:

```bash
ros2 launch fastbot_gazebo one_fastbot_warehouse.launch.py
```

This will start:

- Gazebo Classic with warehouse world
- Fastbot robot model
- Simulated sensors (LiDAR, camera, etc.)
- Robot state publisher and transforms

## Control the Robot

Use keyboard teleoperation to control the simulated robot:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/fastbot/cmd_vel
```
