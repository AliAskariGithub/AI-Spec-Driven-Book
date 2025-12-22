# Perception Systems Quickstart Guide

## Overview
This quickstart guide will help you set up and run the perception systems module, covering camera models, LiDAR, IMU sensors, and sensor fusion in ROS 2.

## Prerequisites
- ROS 2 Humble Hawksbill (or later) installed
- Basic understanding of ROS 2 concepts (topics, services, nodes)
- Gazebo simulation environment
- Computer with adequate processing power for perception tasks
- Git and build tools installed

## Setup Instructions

### 1. Clone and Setup Repository
```bash
git clone <repository-url>
cd <workspace-directory>
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### 2. Install Perception Dependencies
```bash
sudo apt update
sudo apt install ros-humble-vision-opencv
sudo apt install ros-humble-pcl-ros
sudo apt install ros-humble-image-transport
sudo apt install ros-humble-tf2-ros
sudo apt install ros-humble-robot-localization
```

### 3. Install Additional Libraries
```bash
pip3 install opencv-python
pip3 install numpy
pip3 install scipy
```

## Running Camera Perception Examples

### 1. Launch RGB Camera Simulation
```bash
ros2 launch perception_examples rgb_camera.launch.py
```

### 2. View RGB Camera Data
```bash
ros2 run image_view image_view __ns:=/camera
```

### 3. Launch Depth Camera Simulation
```bash
ros2 launch perception_examples depth_camera.launch.py
```

### 4. Convert Depth Image to Point Cloud
```bash
ros2 run depth_image_proc point_cloud_xyzrgb
```

### 5. Launch Stereo Camera Simulation
```bash
ros2 launch perception_examples stereo_camera.launch.py
```

## Running LiDAR Perception Examples

### 1. Launch LiDAR Simulation
```bash
ros2 launch perception_examples lidar_simulation.launch.py
```

### 2. View Point Cloud Data
```bash
ros2 run rviz2 rviz2 -d config/lidar_view.rviz
```

### 3. Run Ground Plane Segmentation
```bash
ros2 run perception_examples ground_segmentation
```

### 4. Run Obstacle Detection
```bash
ros2 run perception_examples obstacle_detection
```

## Running IMU and Sensor Fusion Examples

### 1. Launch IMU Simulation
```bash
ros2 launch perception_examples imu_simulation.launch.py
```

### 2. Run Complementary Filter
```bash
ros2 run perception_examples complementary_filter
```

### 3. Run Extended Kalman Filter
```bash
ros2 run robot_localization ekf_node
```

## Building Perception Pipelines

### 1. Launch Complete Perception Pipeline
```bash
ros2 launch perception_examples perception_pipeline.launch.py
```

### 2. Configure Pipeline Parameters
Edit the `config/perception_pipeline.yaml` file to adjust parameters:
```yaml
perception_pipeline:
  camera_processor:
    feature_detector: "orb"
    max_features: 500
  lidar_processor:
    clustering_method: "euclidean"
    min_cluster_size: 10
  sensor_fusion:
    confidence_threshold: 0.7
```

### 3. Visualize Pipeline Output
```bash
ros2 run rviz2 rviz2 -d config/perception_pipeline.rviz
```

## Testing and Validation

### 1. Run Unit Tests
```bash
colcon test --packages-select perception_examples
colcon test-result --all
```

### 2. Run Integration Tests
```bash
ros2 launch perception_examples test_perception_pipeline.launch.py
```

### 3. Performance Benchmarking
```bash
ros2 run perception_examples benchmark_pipeline
```

## Troubleshooting

### Common Issues:
1. **No camera feed**: Check camera topics with `ros2 topic list | grep camera`
2. **LiDAR not publishing**: Verify Gazebo simulation is running
3. **High CPU usage**: Reduce point cloud resolution or processing frequency
4. **TF errors**: Check transform tree with `ros2 run tf2_tools view_frames`

### Performance Tips:
- Reduce point cloud resolution for real-time applications
- Use voxel grid filtering to reduce data size
- Optimize algorithm parameters for your specific use case
- Monitor CPU and memory usage during operation

## Next Steps
- Explore advanced perception algorithms in the examples
- Integrate perception outputs with navigation stack
- Customize perception pipeline for your specific robot
- Implement custom perception nodes for specialized tasks

## Resources
- [ROS 2 Perception Tutorials](https://navigation.ros.org/)
- [OpenCV Documentation](https://docs.opencv.org/)
- [PCL (Point Cloud Library) Documentation](https://pointclouds.org/)
- [Robotics Sensor Integration Patterns](https://wiki.ros.org/sensors)