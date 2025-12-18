---
sidebar_position: 3
title: "Physics Simulation with Gazebo"
description: "Setting up physics simulations in Gazebo with gravity, collisions, and dynamics for humanoid robot environments"
---

# Physics Simulation with Gazebo

<div className="learning-objectives">
<h3>Learning Objectives</h3>
<ul>
<li>Configure basic physics parameters in Gazebo including gravity and time step</li>
<li>Set up collision detection and response for robotic systems</li>
<li>Implement dynamic simulations with realistic forces and constraints</li>
<li>Create humanoid robot environments with appropriate physics properties</li>
<li>Understand the relationship between simulation and real-world physics</li>
</ul>
</div>

<div className="educational-highlight">

**Prerequisites**: This chapter builds on the ROS 2 concepts covered in [Module 1: The Robotic Nervous System](../module-1/index.md) and the digital twin fundamentals from [Module 2: Digital Twin Fundamentals](./digital-twin-fundamentals.md). If you're new to ROS 2 or simulation concepts, we recommend reviewing those chapters first.

</div>

## Introduction to Gazebo Physics

Gazebo is built on robust physics engines (ODE, Bullet, DART) that provide accurate simulation of real-world physics phenomena. Understanding how to configure and utilize these physics capabilities is crucial for creating realistic robotic simulations.

<div className="physics-properties">

**Key Physics Parameters**: Gravity (typically -9.81 m/s²), time step (simulation update rate), solver iterations, and collision detection parameters.

</div>

## Setting Up Basic Physics Parameters

### Gravity Configuration

The gravitational force is one of the most fundamental physics parameters in Gazebo. By default, Gazebo simulates Earth's gravity of -9.81 m/s² in the z-direction, but this can be customized for different environments:

```xml
<!-- In a world file -->
<world name="custom_gravity_world">
  <gravity>0 0 -5.0</gravity> <!-- Lower gravity for Moon simulation -->
  <!-- Other world elements -->
</world>
```

### Time Step and Update Rate

The physics engine updates the simulation at regular intervals. The time step affects both accuracy and performance:

- **Smaller time steps**: More accurate but computationally expensive
- **Larger time steps**: Faster but potentially unstable

Recommended values:
- Real-time factor: 1.0 (for real-time simulation)
- Max step size: 0.001 to 0.01 seconds
- Real-time update rate: Match your desired simulation speed

## Collision Detection and Response

### Collision Models

Gazebo uses simplified geometric shapes for collision detection to maintain performance:

- **Primitive shapes**: Boxes, spheres, cylinders
- **Mesh shapes**: For complex geometries
- **Compound shapes**: Combinations of primitives

### Contact Materials

Different materials can be specified for collision responses:

```xml
<collision name="link_collision">
  <geometry>
    <sphere><radius>0.1</radius></sphere>
  </geometry>
  <surface>
    <friction>
      <ode>
        <mu>0.5</mu> <!-- Static friction coefficient -->
        <mu2>0.4</mu2> <!-- Secondary friction coefficient -->
      </ode>
    </friction>
    <bounce>
      <restitution_coefficient>0.1</restitution_coefficient> <!-- Bounciness -->
    </bounce>
  </surface>
</collision>
```

## Dynamics Simulation

### Joint Dynamics

Joints in Gazebo can have dynamic properties that affect how they respond to forces:

```xml
<joint name="revolute_joint" type="revolute">
  <parent>link1</parent>
  <child>link2</child>
  <axis>
    <xyz>0 0 1</xyz>
    <dynamics>
      <damping>0.1</damping> <!-- Resistance to motion -->
      <friction>0.05</friction> <!-- Static friction -->
    </dynamics>
  </axis>
</joint>
```

### Inertial Properties

Accurate inertial properties are crucial for realistic dynamics:

```xml
<link name="link_name">
  <inertial>
    <mass>1.0</mass>
    <inertia>
      <ixx>0.01</ixx>
      <ixy>0</ixy>
      <ixz>0</ixz>
      <iyy>0.01</iyy>
      <iyz>0</iyz>
      <izz>0.01</izz>
    </inertia>
  </inertial>
</link>
```

## Practical Examples of Humanoid Environment Setup

### Creating a Simple Humanoid World

Here's an example of a basic world file for humanoid robot simulation:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="humanoid_world">
    <!-- Include ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include sky -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Custom environment elements -->
    <model name="table">
      <pose>2 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>1 0.8 0.8</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>1 0.8 0.8</size></box>
          </geometry>
        </visual>
        <inertial>
          <mass>10.0</mass>
          <inertia>
            <ixx>1.0</ixx> <ixy>0</ixy> <ixz>0</ixz>
            <iyy>1.0</iyy> <iyz>0</iyz> <izz>1.0</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Physics parameters -->
    <physics name="ode" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>
  </world>
</sdf>
```

### Advanced Humanoid Environment Example

For more complex humanoid robot testing, consider this enhanced environment:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="advanced_humanoid_world">
    <!-- Ground plane with texture -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Obstacle course for navigation testing -->
    <model name="obstacle_1">
      <pose>3 1 0.2 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>0.4 0.4 0.4</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>0.4 0.4 0.4</size></box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.01</ixx> <ixy>0</ixy> <ixz>0</ixz>
            <iyy>0.01</iyy> <iyz>0</iyz> <izz>0.01</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Ramps for mobility testing -->
    <model name="ramp">
      <pose>5 0 0 0 0.2 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>2 1 0.1</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>2 1 0.1</size></box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>5.0</mass>
          <inertia>
            <ixx>1.0</ixx> <ixy>0</ixy> <ixz>0</ixz>
            <iyy>1.0</iyy> <iyz>0</iyz> <izz>1.0</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Interactive object for manipulation -->
    <model name="interactive_box">
      <pose>7 0 0.5 0 0 0</pose>
      <static>false</static> <!-- Make it movable -->
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>0.2 0.2 0.2</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>0.2 0.2 0.2</size></box>
          </geometry>
          <material>
            <ambient>0 1 0 1</ambient>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>0.5</mass>
          <inertia>
            <ixx>0.002</ixx> <ixy>0</ixy> <ixz>0</ixz>
            <iyy>0.002</iyy> <iyz>0</iyz> <izz>0.002</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Physics parameters -->
    <physics name="ode" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
      <solver>
        <type>quick</type>
        <iters>10</iters>
        <sor>1.3</sor>
      </solver>
      <constraints>
        <cfm>0.000001</cfm>
        <erp>0.2</erp>
        <contact_max_correcting_vel>100</contact_max_correcting_vel>
        <contact_surface_layer>0.001</contact_surface_layer>
      </constraints>
    </physics>
  </world>
</sdf>
```

### Humanoid Robot Environment Features

For humanoid robot simulation, consider these environmental elements:

1. **Flat surfaces** for stable walking and balance testing
2. **Obstacles** to test navigation and path planning
3. **Ramps/stairs** for mobility and locomotion challenges
4. **Interactive objects** for manipulation and grasping tasks
5. **Markers** for localization and mapping
6. **Different floor textures** for testing on various surfaces
7. **Narrow passages** for navigation challenges
8. **Drop-offs** for safety behavior testing

### Environment Configuration Best Practices

- **Start simple**: Begin with basic environments and add complexity gradually
- **Document parameters**: Keep records of physics parameters that work well
- **Use templates**: Create reusable world file templates for common scenarios
- **Test incrementally**: Add elements one at a time to isolate issues
- **Validate physics**: Ensure all objects behave realistically before adding more complexity

## Gazebo World File Examples and Configuration

### Launching Custom Worlds

World files can be launched directly or through ROS 2 launch files:

```bash
# Direct launch with Gazebo Garden
gz sim -r my_humanoid_world.sdf

# With ROS 2 bridge using launch file
ros2 launch ros_gz_sim gz_sim.launch.py world_file:=/path/to/my_world.sdf

# Launch with custom arguments
ros2 launch my_robot_gazebo my_robot_world.launch.py
```

### World File Structure and Best Practices

#### Directory Organization
```
my_robot_gazebo/
├── worlds/
│   ├── simple_world.sdf
│   ├── complex_world.sdf
│   └── humanoid_test.sdf
├── launch/
│   ├── simple_world.launch.py
│   └── complex_world.launch.py
└── models/  # Custom models if needed
    └── my_object/
        ├── model.sdf
        └── meshes/
```

#### World File Organization
- Keep world files in a dedicated `worlds/` directory
- Use descriptive names that indicate the purpose
- Create launch files for each world to make launching easier
- Document special parameters or requirements in comments

### Advanced World Configuration

#### Physics Engine Selection
```xml
<!-- ODE (Open Dynamics Engine) - Default and most common -->
<physics name="ode" type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <gravity>0 0 -9.8</gravity>
</physics>

<!-- DART (Dynamic Animation and Robotics Toolkit) - Better for complex articulated systems -->
<physics name="dart" type="dart">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <gravity>0 0 -9.8</gravity>
</physics>
```

#### Performance Optimization Settings
```xml
<physics name="ode" type="ode">
  <!-- Balance accuracy vs performance -->
  <max_step_size>0.001</max_step_size>  <!-- Smaller = more accurate but slower -->
  <real_time_factor>1.0</real_time_factor>  <!-- 1.0 = real-time, higher = faster -->
  <real_time_update_rate>1000</real_time_update_rate>  <!-- Updates per second -->

  <!-- Solver settings -->
  <solver>
    <type>quick</type>  <!-- quick or world -->
    <iters>10</iters>  <!-- Solver iterations (more = stable but slower) -->
    <sor>1.3</sor>  <!-- Successive over-relaxation -->
  </solver>

  <!-- Contact constraints -->
  <constraints>
    <cfm>0.000001</cfm>  <!-- Constraint force mixing -->
    <erp>0.2</erp>  <!-- Error reduction parameter -->
    <contact_max_correcting_vel>100</contact_max_correcting_vel>
    <contact_surface_layer>0.001</contact_surface_layer>
  </constraints>
</physics>
```

### Common World File Elements

- **Models**: Robot and environment objects
- **Lights**: Point, spot, and directional lights
- **Materials**: Surface appearances and properties
- **Physics**: Engine configuration and parameters
- **Plugins**: Additional functionality and ROS 2 integration
- **Fog**: Atmospheric effects for outdoor scenes
- **GUI**: Visualization settings and camera positions
- **Audio**: Sound effects and environmental audio

## Troubleshooting Guide for Common Physics Issues

### Robot Instability

**Issue**: Robot falls apart or exhibits unrealistic behavior
**Solutions**:
- Check inertial properties (mass and inertia matrix) - ensure they're physically realistic
- Verify joint limits and types - make sure they match your real robot
- Adjust solver parameters - increase iterations if needed
- Ensure proper parent-child relationships in URDF
- Check for massless links (all links need mass > 0)
- Verify center of mass is properly calculated

### Collision Problems

**Issue**: Objects pass through each other or get stuck
**Solutions**:
- Verify collision geometries are properly defined - ensure they match visual geometries
- Check for overlapping collision meshes - ensure no overlaps exist
- Adjust collision detection parameters - reduce step size if needed
- Use simpler collision geometries for performance
- Verify contact materials are properly defined
- Check for self-collisions in robot model

### Performance Issues

**Issue**: Simulation runs slowly or inconsistently
**Solutions**:
- Reduce physics update rate - balance accuracy with performance
- Simplify collision meshes - use boxes/spheres instead of complex meshes
- Limit the number of objects in simulation - batch tests if needed
- Adjust real-time factor settings - tune for your hardware
- Reduce solver iterations - decrease for faster but less accurate simulation
- Use static objects when possible - they require less computation

### Joint and Actuator Problems

**Issue**: Joints don't respond correctly or exhibit strange behavior
**Solutions**:
- Check joint types - ensure they match your intended movement
- Verify joint limits - make sure they're appropriate for your robot
- Adjust damping and friction - tune for realistic movement
- Check for conflicting controllers - ensure only one controller per joint
- Verify transmission setup - make sure ROS 2 can command joints properly

### Ground Penetration

**Issue**: Robot sinks into the ground or bounces unrealistically
**Solutions**:
- Adjust ERP (Error Reduction Parameter) in physics config
- Modify CFM (Constraint Force Mixing) values
- Check ground plane collision properties
- Verify robot link masses - ensure they're realistic
- Tune contact surface layer settings

### Solver Convergence Issues

**Issue**: Simulation becomes unstable or objects behave erratically
**Solutions**:
- Increase solver iterations for better accuracy
- Reduce step size for more precise calculations
- Adjust SOR (Successive Over-Relaxation) parameter
- Check for conflicting constraints in the model
- Simplify complex contact scenarios

## Gazebo Garden Version Compatibility Notes

This module is designed for Gazebo Garden (Fortress) with ROS 2 Humble Hawksbill. Key compatibility notes:

### Command Line Interface Changes
- Use `gz sim` command instead of `gazebo` (legacy)
- Model database access: `gz model` instead of `gazebo model`
- Service calls: Use `gz service` instead of `rosservice` for native Gazebo services

### ROS 2 Integration
- ROS 2 bridge packages: `ros_gz_sim`, `ros_gz_interfaces`, `ros_gz_common`
- Message types: Use `ros_gz_interfaces` for custom Gazebo messages
- Parameter configuration: Use ROS 2 parameters for Gazebo bridge configuration

### Plugin System
- Plugin system uses Gazebo's new interface (based on Ignition)
- Legacy Gazebo plugins may require updates for Gazebo Garden
- Use `gz::sim` namespace instead of `gazebo` namespace for custom plugins
- Plugin XML format remains largely the same but with updated library paths

### World and Model File Format
- World file format remains largely compatible (SDF 1.6 and 1.7)
- Some deprecated elements may need updates
- Model file format (SDF) is backward compatible for most features

### Migration Tips from Legacy Gazebo
- Update launch files to use `ros_gz_sim` instead of `gazebo_ros`
- Check plugin library names - they may have changed
- Verify topic names - some default topics may differ
- Test physics behavior - solver implementations may vary slightly

### Package Dependencies
```xml
<!-- In package.xml -->
<depend>ros_gz_sim</depend>
<depend>ros_gz_interfaces</depend>
<depend>ros_gz_common</depend>
<depend>gz-sim7</depend>  <!-- Gazebo Garden -->
```

### Installation Verification
```bash
# Check Gazebo Garden installation
gz --version
# Should show Garden/Fortress version

# Verify ROS 2 bridge
ros2 pkg list | grep ros_gz
# Should show ros_gz packages
```

## Example URDF Models for Humanoid Robots

When working with humanoid robots in Gazebo, your URDF files need specific Gazebo extensions. Here's a comprehensive example:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Torso -->
  <joint name="torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
  </joint>

  <link name="torso">
    <visual>
      <geometry>
        <box size="0.25 0.15 0.4"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.25 0.15 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.05"/>
    </inertial>
  </link>

  <!-- Head -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10.0" velocity="1.0"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.002" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Gazebo-specific elements -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
    <mu1>0.8</mu1>  <!-- Static friction coefficient -->
    <mu2>0.8</mu2>  <!-- Secondary friction coefficient -->
    <kp>1000000.0</kp>  <!-- Contact stiffness -->
    <kd>100.0</kd>  <!-- Contact damping -->
  </gazebo>

  <gazebo reference="torso">
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
  </gazebo>

  <gazebo reference="head">
    <mu1>0.5</mu1>
    <mu2>0.5</mu2>
  </gazebo>

  <!-- Gazebo plugins for ROS 2 integration -->
  <gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
      <parameters>$(find my_robot_description)/config/robot_control.yaml</parameters>
    </plugin>
  </gazebo>

  <!-- Optional: Joint state publisher -->
  <gazebo>
    <plugin filename="libgazebo_ros_joint_state_publisher.so" name="joint_state_publisher">
      <ros>
        <namespace>/simple_humanoid</namespace>
      </ros>
      <update_rate>30</update_rate>
      <joint_name>neck_joint</joint_name>
    </plugin>
  </gazebo>
</robot>
```

### URDF Best Practices for Humanoid Robots

1. **Inertial Properties**: Ensure all links have realistic mass and inertia values
2. **Collision Geometries**: Use simple shapes for performance, but accurate enough for physics
3. **Joint Limits**: Set appropriate limits based on real robot capabilities
4. **Friction Parameters**: Tune mu1 and mu2 values for realistic ground interaction
5. **Gazebo Materials**: Use appropriate materials for visual appearance
6. **Plugin Configuration**: Include necessary plugins for ROS 2 integration
7. **Transmission Definitions**: Add transmissions for joint control (if using ros2_control)

### Common Humanoid URDF Components

For more complex humanoid robots, consider these components:
- **Multiple limbs** with appropriate joint types (revolute, continuous)
- **Sensor mounts** for cameras, IMUs, LiDAR
- **Actuator models** with realistic dynamics
- **Kinematic chains** properly defined for arms and legs
- **Center of mass considerations** for balance and stability

## Hands-On Exercises for Physics Configuration

### Exercise 1: Gravity and Environment Tuning
<div className="practical-example">
<p><strong>Objective:</strong> Configure and test different physics parameters in Gazebo.</p>

<p><strong>Instructions:</strong></p>
<ol>
<li>Create three different world files with varying gravity settings (Earth: -9.81, Moon: -1.62, Mars: -3.71)</li>
<li>Spawn the same simple object in each environment</li>
<li>Document the differences in object behavior across environments</li>
<li>Measure fall times and compare with theoretical calculations</li>
<li>Create a report comparing simulation results with real-world physics</li>
</ol>

<p><strong>Expected Outcome:</strong> A detailed comparison of how gravity affects object motion in simulation.</p>
</div>

### Exercise 2: Collision and Dynamics Testing
<div className="practical-example">
<p><strong>Objective:</strong> Test different collision and dynamics parameters.</p>

<p><strong>Instructions:</strong></p>
<ol>
<li>Create a simple robot model with different joint types (revolute, prismatic)</li>
<li>Configure various damping and friction parameters</li>
<li>Test how these parameters affect robot movement</li>
<li>Document which parameters provide the most realistic behavior</li>
<li>Compare results with real-world robot dynamics</li>
</ol>

<p><strong>Expected Outcome:</strong> A configuration file with optimized physics parameters for realistic robot simulation.</p>
</div>

### Exercise 3: Humanoid Environment Construction
<div className="practical-example">
<p><strong>Objective:</strong> Build a complete humanoid robot testing environment.</p>

<p><strong>Instructions:</strong></p>
<ol>
<li>Design a world file with multiple testing areas (flat ground, ramps, obstacles)</li>
<li>Include interactive objects for manipulation tasks</li>
<li>Configure appropriate physics parameters for each area</li>
<li>Test the environment with a simple humanoid model</li>
<li>Document any physics issues and how you resolved them</li>
</ol>

<p><strong>Expected Outcome:</strong> A complete, functional world file ready for humanoid robot testing.</p>
</div>

## Verification Steps for Physics Simulation Functionality

### Basic Physics Check

1. Launch a simple world with a falling object
2. Verify gravity is working (object should accelerate downward at ~9.8 m/s²)
3. Check collision detection (object should stop at ground without penetrating)
4. Confirm realistic bounce/restitution behavior (objects should bounce with appropriate damping)
5. Test with different object shapes and masses to verify consistent physics behavior
6. Measure fall times and compare with theoretical physics calculations

### Humanoid Robot Check

1. Spawn your humanoid robot in the environment
2. Verify all joints respond appropriately to commands (no unexpected movements)
3. Test basic movements (if using controllers) - check for stability and realistic motion
4. Confirm stability during static poses (robot should maintain position without drifting)
5. Check that robot mass distribution is appropriate (shouldn't be too light or heavy)
6. Verify joint limits are respected (joints shouldn't exceed configured limits)
7. Test balance by applying small external forces to see if robot responds appropriately

### Environment Interaction Check

1. Place various objects in the environment (boxes, spheres, cylinders)
2. Test robot-environment interactions (pushing, lifting if applicable)
3. Verify collision responses are realistic (no objects passing through each other)
4. Check that physics parameters are appropriate for all objects in the scene
5. Test with multiple objects to ensure no performance degradation
6. Verify that static objects remain fixed in position
7. Test dynamic interactions (objects should respond realistically to forces)

### Advanced Verification

1. **Performance Testing**: Monitor simulation real-time factor to ensure it runs at acceptable speed
2. **Stability Testing**: Run simulation for extended periods to check for drift or instability
3. **Parameter Sensitivity**: Test that small changes in parameters produce expected changes in behavior
4. **Boundary Conditions**: Test extreme scenarios (high forces, fast movements) to ensure robustness
5. **ROS 2 Integration**: Verify that sensor data and control commands are properly exchanged
6. **Multi-Robot Scenarios**: If applicable, test multiple robots interacting without interference

### Quantitative Verification

For more rigorous testing, consider these measurements:
- Position accuracy: How closely does simulated position match expected values?
- Velocity accuracy: Are speeds realistic compared to real-world expectations?
- Energy conservation: Does the system behave appropriately with respect to energy?
- Computational performance: What is the real-time factor and resource usage?
- Determinism: Do identical scenarios produce identical results across runs?

### Documentation Requirements

Document your verification results including:
- Simulation parameters used
- Expected vs. actual behavior for key tests
- Any deviations or issues encountered
- Performance metrics (real-time factor, CPU usage)
- Recommendations for parameter adjustments
- Known limitations or issues for future reference

## "Try This" Experiments

### Experiment 1: Gravity Adjustment
1. Create a world file with different gravity values
2. Launch with low gravity (e.g., Moon gravity: -1.62 m/s²)
3. Observe how robot behavior changes
4. Compare with Earth gravity simulation

### Experiment 2: Friction Testing
1. Create surfaces with different friction coefficients
2. Test how they affect robot mobility
3. Document the differences in locomotion
4. Consider implications for real-world applications

## Real-World Connections

Physics simulation in Gazebo is used extensively in robotics research and development:

- **Boston Dynamics**: Uses simulation for initial algorithm testing
- **NASA**: Simulates robotic systems for space missions
- **Academic Labs**: Validates control algorithms before hardware testing
- **Industrial Applications**: Tests robot behavior in factory environments

## Summary

Physics simulation in Gazebo provides the foundation for realistic robotic testing and development. By properly configuring gravity, collision detection, and dynamic properties, you can create simulations that closely mirror real-world behavior. The next section will cover how to integrate sensors into these physics simulations.