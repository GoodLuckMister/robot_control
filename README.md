A ROS2 package for controlling a mecanum-wheel robot in Gazebo simulation using the ros2_control.
[All examples you can find here gz_ros2_control](https://github.com/ros-controls/gz_ros2_control/)

## System Requirements

- **Operating System**: Ubuntu 24.04 LTS
- **ROS2 Distribution**: ROS2 Jazzy
- **Gazebo**: Gazebo Harmonic

## Installation

```bash
apt-get update && apt-get install -y --no-install-recommends \
    ros-dev-tools \
    python3-colcon-common-extensions \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-xacro 
```


### Build the Package

```bash
# Create workspace and clone repository
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
git clone https://github.com/GoodLuckMister/robot_control.git

# Build the package
cd ~/robot_ws
colcon build --packages-select robot_control
# Source the workspace
source install/setup.bash
```

## Usage

### Launch the Robot Simulation

```bash
# Source your workspace
source ~/robot_ws/install/setup.bash

# Launch the robot in Gazebo
ros2 launch robot_control robot_drive.launch.py
```

This will:
1. Start Gazebo with an empty world
2. Spawn the robot

### Control the Robot

The robot can be controlled by keyboard:

```bash
# Launch another terminal 
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

## License

This project is licensed under the Apache License 2.0.

