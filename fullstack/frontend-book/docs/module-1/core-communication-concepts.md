---
sidebar_position: 3
---

# Core Communication Concepts

Robotic systems rely on effective communication between various components to function properly. This chapter introduces the fundamental communication concepts in robotics, including nodes, topics, and services, and explains how they enable distributed robotic applications.

## Nodes

A **node** is a process that performs computation in a ROS system. Nodes are the fundamental building blocks of a ROS application and can publish or subscribe to topics, provide or use services, and manage parameters.

### Node Characteristics
- Each node runs as a separate process
- Nodes can be written in different programming languages
- Nodes communicate with each other through messages
- Nodes can be started and stopped independently

### Node Management
- Nodes must register with the ROS master (in ROS 1) or DDS (in ROS 2)
- Nodes can be named to facilitate identification and debugging
- Nodes can be grouped into namespaces for organization

## Topics and Message Passing

**Topics** enable asynchronous, many-to-many communication between nodes through a publish-subscribe pattern.

### Publish-Subscribe Pattern
- Publishers send messages to a topic without knowing who will receive them
- Subscribers receive messages from a topic without knowing who sent them
- This decouples the sender and receiver in time and space

### Message Types
- Messages have strict data structures defined in `.msg` files
- Messages are serialized for network transmission
- Different message types are available for various data (sensors, commands, etc.)

### Quality of Service (QoS)
- QoS settings control how messages are delivered
- Settings include reliability, durability, and history policies
- QoS allows tuning communication for specific requirements

## Services

**Services** enable synchronous, request-response communication between nodes.

### Service Characteristics
- Services follow a client-server model
- The service client sends a request and waits for a response
- Services are synchronous and block until the response is received
- Services are appropriate for tasks that require a direct response

### Service Structure
- Services have a request message type and a response message type
- Service definitions are stored in `.srv` files
- Services are identified by unique names in the ROS graph

## Streaming vs Request/Response Patterns

### Streaming Communication (Topics)
- **Use Case**: Sensor data, continuous control commands, status updates
- **Pattern**: Publish-subscribe
- **Timing**: Asynchronous, continuous
- **Characteristics**:
  - No guaranteed delivery
  - No response required
  - Multiple subscribers possible
  - Suitable for real-time data

### Request/Response Communication (Services)
- **Use Case**: Configuration changes, computation requests, action triggers
- **Pattern**: Client-server
- **Timing**: Synchronous, on-demand
- **Characteristics**:
  - Guaranteed delivery
  - Response required
  - One client per request
  - Suitable for discrete operations

## Communication Patterns in Practice

### When to Use Topics
- Streaming sensor data (camera images, LIDAR scans, IMU readings)
- Continuous control commands (motor velocities, joint positions)
- Status broadcasts (battery level, system state)
- Event notifications

### When to Use Services
- Action triggers (start/stop operations)
- Configuration changes (update parameters)
- Computation requests (path planning, object recognition)
- Synchronous operations requiring confirmation

## Practical Examples

Throughout this chapter, we'll explore practical examples of these communication concepts in robotic applications, demonstrating how nodes, topics, and services work together to create distributed robotic systems.

## Learning Goals

After completing this chapter, you should be able to:
- Distinguish between nodes, topics, and services in robotic systems
- Explain the publish-subscribe and request-response communication patterns
- Identify appropriate use cases for streaming and request/response communication
- Understand the differences between asynchronous and synchronous communication in robotics

## Quick Test

import TestSection from '@site/src/components/TestSection';

<TestSection questions={[
  {
    question: "What is a node in a ROS system?",
    options: [
      "A process that performs computation and can publish/subscribe to topics",
      "A type of message used for communication",
      "A communication channel between components",
      "A hardware component of the robot"
    ],
    correct: 0,
    explanation: "A node is a process that performs computation in a ROS system and can publish or subscribe to topics, provide or use services, and manage parameters."
  },
  {
    question: "Which communication pattern is used by topics in ROS?",
    options: [
      "Client-server model",
      "Request-response pattern",
      "Publish-subscribe pattern",
      "Peer-to-peer model"
    ],
    correct: 2,
    explanation: "Topics enable asynchronous, many-to-many communication between nodes through a publish-subscribe pattern."
  },
  {
    question: "What is the main difference between topics and services in ROS?",
    options: [
      "Topics are synchronous while services are asynchronous",
      "Topics use request-response while services use publish-subscribe",
      "Topics are asynchronous while services are synchronous",
      "There is no significant difference between them"
    ],
    correct: 2,
    explanation: "Topics use asynchronous publish-subscribe communication, while services use synchronous request-response communication."
  },
  {
    question: "Which of the following is an appropriate use case for services?",
    options: [
      "Streaming camera images",
      "Sending continuous motor commands",
      "Requesting path planning computation",
      "Broadcasting robot status"
    ],
    correct: 2,
    explanation: "Services are appropriate for tasks that require a direct response, such as computation requests like path planning."
  },
  {
    question: "What does QoS stand for in ROS communication?",
    options: [
      "Quality of Service",
      "Quick Operating System",
      "Query and Output System",
      "Quantum Operation Specification"
    ],
    correct: 0,
    explanation: "QoS stands for Quality of Service, which includes settings that control how messages are delivered, such as reliability, durability, and history policies."
  }
]} />