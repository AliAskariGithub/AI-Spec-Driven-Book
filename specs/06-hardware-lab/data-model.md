# Data Model: Module 6 - Hardware Requirements & Lab Setup

## Key Entities from Feature Specification

### Workstation Requirements
- **Entity**: Workstation Requirements
- **Attributes**:
  - CPU specifications (minimum/recommended)
  - GPU specifications (minimum/recommended)
  - Memory requirements (minimum/recommended)
  - Storage requirements (minimum/recommended)
  - Use case scenarios
- **Relationships**: Connected to Edge Computing Platforms and Lab Configuration Tiers

### Edge Computing Platforms
- **Entity**: Edge Computing Platforms
- **Attributes**:
  - Platform type (Jetson Nano, Xavier NX, Orin, etc.)
  - Specifications (CPU, GPU, memory, power consumption)
  - Use cases and applications
  - Price points
  - Compatibility requirements
- **Relationships**: Connected to Workstation Requirements and Sensors and Input Devices

### Sensors and Input Devices
- **Entity**: Sensors and Input Devices
- **Attributes**:
  - Device type (RealSense cameras, microphones, etc.)
  - Specifications (resolution, accuracy, range, etc.)
  - Use cases and applications
  - Price points
  - Interface requirements
- **Relationships**: Connected to Edge Computing Platforms and Lab Configuration Tiers

### Lab Configuration Tiers
- **Entity**: Lab Configuration Tiers
- **Attributes**:
  - Tier type (proxy, miniature, premium)
  - Budget range
  - Equipment list
  - Educational objectives
  - Space requirements
  - Implementation complexity
- **Relationships**: Connected to Workstation Requirements and Implementation Strategy

### Implementation Strategy
- **Entity**: Implementation Strategy
- **Attributes**:
  - Approach type (simulation-first, hardware-first)
  - Advantages and disadvantages
  - Recommended use cases
  - Resource requirements
  - Learning objectives
- **Relationships**: Connected to Lab Configuration Tiers and overall module objectives

## Entity Relationships
- Workstation Requirements → Lab Configuration Tiers (affects which tier is feasible)
- Edge Computing Platforms → Sensors and Input Devices (compatibility and integration)
- Lab Configuration Tiers → Implementation Strategy (strategy choice depends on configuration tier)
- Implementation Strategy → Workstation Requirements (strategy affects compute needs)