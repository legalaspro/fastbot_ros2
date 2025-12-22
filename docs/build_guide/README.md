# Fastbot Build Guide

This guide documents the evolution of building the Fastbot robot, from basic motor control to a fully functional ROS2-enabled mobile robot platform.

## Build Evolution

The Fastbot was built incrementally, with each step adding new capabilities:

### Step 1: Basic Motor Control

**[01_basic_setup.md](01_basic_setup.md)**

Starting point with minimal hardware:

- Arduino microcontroller
- 2x TT DC motors with encoders
- L298N motor driver
- ros_arduino_bridge for ROS integration
- Serial communication testing

**Status**: Foundation for differential drive control

### Step 2: Add Power Module

**[02_add_power_module.md](02_add_power_module.md)**

Simplify power distribution:

- DFRobot DFR0205 power module
- Centralized power management
- Cleaner wiring for encoders
- Regulated 5V output

**Status**: Improved power distribution

### Step 3: Add Raspberry Pi

**[03_add_raspberry_pi.md](03_add_raspberry_pi.md)**

Add the computing brain:

- Raspberry Pi with Ubuntu Server 22.04
- ROS2 Humble installation
- Serial connection to Arduino
- Environment configuration

**Status**: ROS2 ready for robot control

### Step 4: Add LSlidar N10

**[04_add_lslidar_n10.md](04_add_lslidar_n10.md)**

Add LiDAR sensor for mapping and navigation:

- LSlidar N10 2D LiDAR
- Ethernet connection to Raspberry Pi
- Pre-configured ROS2 driver
- Laser scan visualization

**Status**: LiDAR sensor integrated

### Step 5: Add Raspberry Pi Camera

**[05_add_raspberry_cam.md](05_add_raspberry_cam.md)**

Add vision capabilities with Raspberry Pi camera:

- Raspberry Pi Camera Module (v1, v2, or HQ)
- Legacy camera driver configuration
- ros2_v4l2_camera ROS2 integration
- Image topic publishing

**Status**: Camera integrated

## Philosophy

This build guide follows an incremental approach:

1. Start simple - get basic functionality working first
2. Test thoroughly at each step
3. Add complexity gradually
4. Document lessons learned

## Prerequisites

- Basic electronics knowledge
- Familiarity with Arduino
- ROS2 installed (Humble or later)
- Basic Linux/terminal skills

## Getting Started

Begin with [Step 1: Basic Setup](01_basic_setup.md) to build the foundation of your Fastbot.
