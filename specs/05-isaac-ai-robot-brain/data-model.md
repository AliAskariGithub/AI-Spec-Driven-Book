# Isaac AI Robot Brain - Data Model

## Overview
This document defines the data models for the Isaac AI Robot Brain module, covering Isaac Sim, synthetic data generation, Isaac ROS, and Nav2 integration.

## Core Entities

### Isaac Sim Components
- **SimulationEnvironment**:
  - name: string
  - description: string
  - assets: [AssetReference]
  - physics_properties: PhysicsProperties
  - lighting_config: LightingConfiguration
  - scene_graph: SceneGraph

- **AssetReference**:
  - asset_id: string
  - asset_type: enum (robot, object, environment, sensor)
  - position: {x, y, z}
  - orientation: {x, y, z, w}
  - scale: {x, y, z}

- **PhysicsProperties**:
  - gravity: {x, y, z}
  - friction_coefficients: [number]
  - collision_properties: CollisionProperties
  - material_properties: MaterialProperties

### Synthetic Data Generation
- **SyntheticDataset**:
  - dataset_id: string
  - name: string
  - description: string
  - generation_parameters: GenerationParameters
  - annotations: [Annotation]
  - metadata: DatasetMetadata

- **Annotation**:
  - annotation_type: enum (bounding_box, segmentation, pose, depth)
  - object_id: string
  - label: string
  - confidence: number
  - coordinates: [number] (format depends on annotation_type)

- **GenerationParameters**:
  - domain_randomization: DomainRandomizationConfig
  - rendering_settings: RenderingSettings
  - sensor_configurations: [SensorConfiguration]
  - batch_size: number

### Isaac ROS Components
- **IsaacROSNode**:
  - node_name: string
  - package_name: string
  - gpu_requirements: GPURequirements
  - input_topics: [TopicInfo]
  - output_topics: [TopicInfo]
  - parameters: {key: value}

- **GPURequirements**:
  - minimum_compute_capability: string
  - memory_requirements: number (in GB)
  - tensor_core_support: boolean
  - cuda_version: string

### Nav2 Navigation
- **NavigationPlan**:
  - plan_id: string
  - start_pose: Pose
  - goal_pose: Pose
  - path: [Pose]
  - costmap: Costmap
  - execution_status: enum (pending, executing, completed, failed)

- **Costmap**:
  - resolution: number
  - origin: {x, y}
  - width: number
  - height: number
  - data: [number] (cost values)
  - layers: [CostmapLayer]

## Relationships
- A SimulationEnvironment contains multiple AssetReferences
- A SyntheticDataset is generated from a SimulationEnvironment
- An IsaacROSNode processes data from multiple sensors
- A NavigationPlan uses data from Isaac perception systems
- Multiple IsaacROSNodes form a perception pipeline
- Costmaps are generated from sensor data processed by Isaac ROS

## Data Flow
1. SimulationEnvironment → SyntheticDataset (for training data generation)
2. Real/Simulated sensors → IsaacROSNode → Processed perception data
3. Perception data → Costmap → NavigationPlan
4. NavigationPlan → Robot control commands
5. Robot state → Localization → Updated environment model

## Data Formats
- **USD (Universal Scene Description)**: For scene representation in Isaac Sim
- **ROS Message Types**: Standard sensor and control messages
- **NITROS**: NVIDIA's Image Transport for optimized GPU data transfer
- **TensorRT Models**: Optimized neural networks for inference
- **Costmap Formats**: Grid-based representation for navigation