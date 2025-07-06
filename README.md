# robot_control

A ROS2 package for controlling a mecanum-wheel robot in Gazebo simulation using the ros2_control framework.

## System Requirements

- **Operating System**: Ubuntu 24.04 LTS
- **ROS2 Distribution**: ROS2 Jazzy
- **Gazebo**: Gazebo Garden (gz-garden)

## Dependencies

### Required ROS2 Packages

- `ros2_control` and `ros2_controllers`
- `gz_ros2_control` - Gazebo-ROS2 control integration
- `mecanum_drive_controller` - Controller for mecanum wheel robots
- `robot_state_publisher` - Robot state publishing
- `ros_gz_sim` - Gazebo simulation integration
- `ros_gz_bridge` - ROS2-Gazebo message bridge
- `xacro` - XML macro language for robot descriptions

### Additional Dependencies

- `geometry_msgs` - Geometry message types
- `std_msgs` - Standard message types
- `control_msgs` - Control system messages
- `rclcpp` - ROS2 C++ client library

## Installation

### 1. Install ROS2 Jazzy

Follow the official ROS2 Jazzy installation guide for Ubuntu 24.04:
```bash
# Add ROS2 apt repository
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS2 Jazzy
sudo apt update
sudo apt install ros-jazzy-desktop
```

### 2. Install Gazebo Garden

```bash
sudo apt install gz-garden
```

### 3. Install Required ROS2 Packages

```bash
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
                 ros-jazzy-gz-ros2-control ros-jazzy-mecanum-drive-controller \
                 ros-jazzy-robot-state-publisher ros-jazzy-ros-gz-sim \
                 ros-jazzy-ros-gz-bridge ros-jazzy-xacro
```

### 4. Build the Package

```bash
# Source ROS2
source /opt/ros/jazzy/setup.bash

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

## Robot Description

This package simulates a mecanum-wheel robot with the following specifications:

- **Base Dimensions**: 2.0m (length) × 1.0m (width) × 0.5m (height)
- **Wheels**: 4 mecanum wheels with 0.3m radius
- **Wheel Configuration**: 
  - Front Left: `front_left_wheel_joint`
  - Front Right: `front_right_wheel_joint`
  - Rear Left: `rear_left_wheel_joint`
  - Rear Right: `rear_right_wheel_joint`
- **Control**: Velocity-controlled joints via ros2_control

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
2. Spawn the mecanum robot
3. Load the robot description
4. Start the controller manager with joint state broadcaster
5. Start the mecanum drive controller
6. Set up ROS2-Gazebo bridge for clock synchronization

### Control the Robot

The robot can be controlled by publishing velocity commands to the `/cmd_vel` topic:

```bash
# Example: Move forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' --once

# Example: Rotate
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}' --once

# Example: Strafe left (mecanum wheel capability)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' --once

# Stop the robot
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' --once
```

### Monitor Robot State

```bash
# View available topics
ros2 topic list

# Monitor odometry
ros2 topic echo /odom

# Monitor joint states
ros2 topic echo /joint_states

# View active controllers
ros2 control list_controllers
```

## Topics and Services

### Published Topics

- `/odom` (`nav_msgs/msg/Odometry`) - Robot odometry information
- `/joint_states` (`sensor_msgs/msg/JointState`) - Joint state information
- `/tf` (`tf2_msgs/msg/TFMessage`) - Transform information

### Subscribed Topics

- `/cmd_vel` (`geometry_msgs/msg/Twist`) - Velocity commands for robot movement

### Services

- Controller manager services for managing controllers
- Robot state publisher services

## Configuration

### Controller Configuration

The mecanum drive controller is configured in `config/drive_controller.yaml`:

- **Update Rate**: 100 Hz
- **Reference Timeout**: 1.0 seconds
- **Wheel Radius**: 0.3 meters
- **Robot Center Projection**: 1.1 meters (sum of X and Y axis projections)
- **Base Frame**: `base_link`
- **Odometry Frame**: `odom`

### Launch Parameters

- `use_sim_time`: Boolean parameter to use simulation time (default: true)

## File Structure

```
robot_control/
├── CMakeLists.txt          # Build configuration
├── package.xml             # Package metadata and dependencies
├── README.md              # This file
├── config/
│   └── drive_controller.yaml  # Controller configuration
├── launch/
│   └── robot_drive.launch.py  # Main launch file
└── urdf/
    └── robot_drive.xacro.urdf  # Robot description in URDF format
```

## Troubleshooting

### Common Issues

1. **Gazebo doesn't start**: Ensure Gazebo Garden is properly installed
2. **Controllers fail to load**: Check that all ros2_control packages are installed
3. **Robot doesn't move**: Verify controller is active with `ros2 control list_controllers`
4. **Build errors**: Ensure all dependencies are installed and ROS2 is sourced

### Debugging Commands

```bash
# Check controller status
ros2 control list_controllers

# View controller info
ros2 control list_hardware_interfaces

# Monitor controller manager
ros2 topic echo /controller_manager/robot_description
```

## License

This project is licensed under the Apache License 2.0.

## Author

- **Author**: Oleksii Piskarov
- **Maintainer**: GOODLUCKMISTER <paet14@gmail.com>
