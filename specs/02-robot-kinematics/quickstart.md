# Quickstart: Module 2 - Digital Twin with Gazebo & Unity

## Overview
This quickstart guide will help you set up the simulation environment for Module 2, focusing on digital twin concepts using Gazebo and Unity with ROS 2 integration.

## Prerequisites
Before starting Module 2, ensure you have:
- Completed Module 1: The Robotic Nervous System
- ROS 2 Humble Hawksbill installed and configured
- Basic understanding of ROS 2 concepts (topics, services, nodes)
- Docusaurus development environment from the main project

## Environment Setup

### 1. System Requirements
- **Operating System**: Ubuntu 22.04 LTS (recommended) or Windows 10+
- **RAM**: 8GB minimum (16GB recommended)
- **CPU**: 4+ cores, 2.5GHz+
- **Disk Space**: 5GB+ free space

### 2. Install Gazebo Garden
```bash
# For Ubuntu 22.04
sudo apt update
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control ros-humble-gazebo-ros2-control-demos
sudo apt install -y ros-humble-ros-gz ros-humble-ros2-control ros-humble-ros2-controllers
```

### 3. Install Unity (Optional - Advanced Topics)
For Unity integration, download Unity Hub from the Unity website and install Unity 2022.3 LTS or later. Install the ROS# package for ROS 2 communication.

### 4. Verify Installation
```bash
# Check Gazebo installation
gz --version
# Should show Gazebo Garden version

# Check ROS 2 integration
ros2 run gazebo_ros gazebo
# Should launch Gazebo with ROS 2 bridge
```

## Getting Started with Digital Twin Concepts

### 1. Launch Basic Simulation Environment
```bash
# Create a workspace for the examples
mkdir -p ~/digital_twin_ws/src
cd ~/digital_twin_ws
colcon build
source install/setup.bash
```

### 2. Run Sample Digital Twin Simulation
```bash
# Launch a basic humanoid robot in Gazebo
ros2 launch ros_gz_sim example.launch.py
```

### 3. Explore Simulation Properties
- **Physics Engine**: Check gravity, friction, and collision properties
- **Robot Model**: Examine the URDF model and joint configurations
- **Sensors**: Identify simulated sensors and their ROS 2 topics

## Chapter-Specific Quickstarts

### Chapter 1: Digital Twin Fundamentals
1. Start with the basic empty world in Gazebo
2. Load a simple robot model
3. Observe how the simulation mirrors real-world physics
4. Compare simulation vs. real robot behavior concepts

### Chapter 2: Physics Simulation with Gazebo
1. Launch the humanoid environment example
2. Adjust gravity and collision parameters
3. Test different physics engine settings
4. Create a custom environment with obstacles

### Chapter 3: Sensors and Interaction
1. Add LiDAR sensor to your robot model
2. Configure depth camera simulation
3. Set up IMU sensor simulation
4. Test Unity interaction (if Unity is installed)

## Common Issues and Solutions

### Gazebo Won't Launch
- Ensure ROS 2 environment is sourced
- Check that gazebo packages are properly installed
- Verify GPU drivers are up to date

### Sensor Data Not Publishing
- Confirm sensor plugins are properly configured in URDF
- Check that Gazebo-ROS bridge is running
- Verify topic names match expected sensor_msgs types

### Unity-ROS Connection Issues
- Ensure ROS# package is properly installed in Unity
- Check that IP addresses and ports match between Unity and ROS
- Verify ROS master is running and accessible

## Next Steps
After completing this quickstart:
1. Proceed to Chapter 1: Digital Twin Fundamentals
2. Practice with the basic simulation examples
3. Set up your development environment for hands-on exercises
4. Join the project Discord/Slack for community support

## Resources
- [Gazebo Garden Documentation](https://gazebosim.org/docs/garden)
- [ROS 2 with Gazebo Tutorials](https://classic.gazebosim.org/tutorials?tut=ros2_overview)
- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)