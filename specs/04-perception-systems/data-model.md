# Perception Systems Data Model

## Overview
This document defines the data models for the perception systems module, covering camera models, LiDAR data, IMU sensors, and sensor fusion components.

## Core Entities

### Camera Models
- **CameraType**: RGB, Depth, Stereo
- **CameraParameters**:
  - resolution: [width, height]
  - frame_rate: number
  - focal_length: number
  - distortion_coefficients: [k1, k2, p1, p2, k3]
  - baseline: number (for stereo cameras)
- **ImageData**:
  - timestamp: ISOString
  - format: string
  - data: binary
  - metadata: CameraParameters

### LiDAR Systems
- **LiDARType**: Single-line, Multi-line, Time-of-Flight
- **LiDARParameters**:
  - range_min: number
  - range_max: number
  - field_of_view: {horizontal: number, vertical: number}
  - resolution: number
  - update_rate: number
- **PointData**:
  - x, y, z: coordinates
  - intensity: number
  - timestamp: ISOString

### IMU Sensors
- **IMUData**:
  - linear_acceleration: {x, y, z}
  - angular_velocity: {x, y, z}
  - orientation: {x, y, z, w}
  - magnetic_field: {x, y, z}
  - timestamp: ISOString
  - covariance: [9-element array]

### Sensor Fusion
- **FusionResult**:
  - pose: {position: {x, y, z}, orientation: {x, y, z, w}}
  - velocity: {linear: {x, y, z}, angular: {x, y, z}}
  - confidence: number
  - source_sensors: [string]
  - timestamp: ISOString

## Relationships
- A Robot can have multiple Camera Models
- A Robot can have multiple LiDAR Systems
- A Robot can have multiple IMU Sensors
- Multiple sensors contribute to a single Fusion Result
- Point Cloud data is generated from LiDAR and Depth Camera data

## Data Flow
1. Raw sensor data flows from individual sensors (cameras, LiDAR, IMU)
2. Preprocessed data is stored temporarily in intermediate formats
3. Fused data combines multiple sensor inputs
4. Processed data is made available to perception algorithms