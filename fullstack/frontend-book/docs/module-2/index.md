---
sidebar_position: 1
title: "Module 2: Robot Kinematics & Physical Structure"
description: "Understanding robot kinematics and physical structure for humanoid robots including links, joints, coordinate frames, and URDF models"
---

# Module 2: Robot Kinematics & Physical Structure

## Overview

Welcome to Module 2 of our robotics education platform! In this module, we'll explore the fascinating world of robot kinematics and physical structure. Building on your knowledge of ROS 2 from Module 1, you'll learn how humanoid robots are physically constructed, how their joints and links move, and how to mathematically describe their structure and motion.

## Learning Objectives

By the end of this module, you will be able to:
- Identify and describe the links, joints, and coordinate frames of a humanoid robot
- Perform forward kinematics calculations to determine end-effector positions from joint angles
- Perform inverse kinematics calculations to determine joint angles for desired positions
- Understand humanoid joint constraints and motion limits that affect robot movement
- Analyze URDF models and map them to both simulated and real robot configurations

## Prerequisites

Before starting this module, you should have:
- Completed Module 1: The Robotic Nervous System
- Basic understanding of ROS 2 concepts (topics, services, nodes)
- ROS 2 Humble Hawksbill installed and configured

## Module Structure

This module consists of four chapters that progressively build your understanding of robot kinematics and physical structure:

1. **Robot Links, Joints, and Coordinate Frames** - Understanding robot structure and spatial relationships
2. **Forward and Inverse Kinematics** - Mathematical relationships between joint angles and end-effector positions
3. **Humanoid Joint Constraints and Motion Limits** - Physical limitations that govern robot movement
4. **Mapping URDF to Real and Simulated Robots** - Connecting virtual models to physical hardware

## Getting Started

Ready to dive into robot kinematics and physical structure? If you haven't already, make sure you're familiar with the robotic middleware concepts from [Module 1: The Robotic Nervous System](../module-1/index.md) as we'll be building on those concepts.

Continue to the next chapter to explore the fundamentals of robot structure, including links, joints, and coordinate frames that form the foundation of humanoid robot design.

## Testing Module 2 Locally

To run this educational platform locally for development and testing:

1. **Install Dependencies**:
   ```bash
   cd fullstack/frontend-book
   npm install
   ```

2. **Start Local Server**:
   ```bash
   npm run start
   ```
   This will launch the Docusaurus development server, typically accessible at `http://localhost:3000`

3. **Verify Module 2 Content**:
   - Navigate to the Module 2 section in the sidebar
   - Test all chapter links to ensure they work correctly
   - Verify that cross-references between modules function properly
   - Check that all code examples and diagrams display correctly

4. **Build Static Site** (for production deployment):
   ```bash
   npm run build
   ```
   This creates a static build in the `build/` directory that can be served by any web server.

## Cloud Development Options

For users without suitable local development environments, consider these cloud-based alternatives:

- **GitHub Codespaces**: Create a cloud-based development environment directly in your browser
- **AWS RoboMaker**: Cloud-based robotics simulation and development environment
- **Docker-based Setup**: Use containerization for consistent environments across different systems

## Educational Content Best Practices

Based on research in simulation-based learning, this module incorporates these best practices:

### Theory-Practice Cycle
- Each concept is introduced with theory, followed by practical examples
- Hands-on exercises allow immediate application of learned concepts
- Verification steps ensure understanding before progressing

### Progressive Complexity
- Start with basic simulation concepts and gradually build to complex scenarios
- Each chapter assumes knowledge from previous chapters but reviews key concepts
- Advanced topics are introduced only after foundational concepts are established

### Real-World Context
- Connect simulation concepts to actual robotics applications
- Include examples from industry and research
- Highlight the relationship between simulation and real-world robot behavior

### Immediate Feedback
- Include verification steps for each exercise
- Provide expected outcomes for all practical examples
- Add troubleshooting tips throughout content

### Self-Paced Learning
- Structure content in digestible sections
- Include "Try This" experiments for exploration
- Provide multiple examples to accommodate different learning styles

## Deployment Process for GitHub Pages

This module is designed to be deployed alongside the rest of the educational platform to GitHub Pages. Here's the deployment process:

### Automatic Deployment
1. **GitHub Actions**: The repository is configured with GitHub Actions workflow that automatically builds and deploys on pushes to main branch
2. **Build Process**: Uses the standard Docusaurus build process with `npm run build`
3. **Deployment**: Output is automatically published to the GitHub Pages site

### Manual Deployment Process
1. **Prepare Build**: Run `npm run build` to generate static site in `build/` directory
2. **Verify Build**: Check that all Module 2 content renders correctly in the build
3. **Deploy**: The `build/` directory contains all static assets ready for deployment to any web server

### GitHub Pages Configuration
- **Source**: Deploy from `main` branch, `/build` folder
- **Domain**: Configured to use custom domain if needed (otherwise username.github.io/repository-name)
- **CNAME**: If using custom domain, CNAME file is included in static assets

### Verification Steps for Deployment
1. **Content Validation**: Ensure all Module 2 pages are accessible via deployed site
2. **Link Verification**: Test all internal and external links in deployed version
3. **Responsive Testing**: Verify content displays correctly on different devices
4. **Performance Check**: Confirm page load times are acceptable
5. **Search Functionality**: Verify that search works for all Module 2 content