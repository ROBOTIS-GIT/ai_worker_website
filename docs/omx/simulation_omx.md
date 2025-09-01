# Simulation

## Overview
The OMX simulation environment is designed to test and validate robot functionalities in a controlled virtual setting. This allows for comprehensive evaluation of its capabilities across a variety of tasks before deployment on the physical hardware. The simulation ecosystem leverages industry-standard tools including ROS 2.

## Simulation for Reinforcement Learning
Our simulation environments are particularly well-suited for reinforcement learning (RL). They provide realistic physics and sensor feedback, enabling the OMX to learn and adapt its behaviors through interaction. This is crucial for developing complex manipulation skills.

## Available Simulation Models and Environments

The OMX can be simulated in various environments using different model formats. These models include all necessary components of the OMX, such as the 5‑DOF arm, gripper, and required sensors, allowing for comprehensive testing.

### Gazebo / RViz (URDF/SDF)
For seamless integration with the ROS ecosystem, URDF (Unified Robot Description Format) and SDF (Simulation Description Format) models are provided. These are primarily used with Gazebo for physics simulation and RViz for 3D visualization of robot states and sensor data.

![Simulation environments list](/simulation/all/isaaclab_list_envs.png)

*   **Model Formats**: [URDF](https://docs.ros.org/en/rolling/Tutorials/Intermediate/URDF/URDF-Main.html), [SDF](http://sdformat.org/)
*   **Primary Tools**: [Gazebo Sim](https://gazebosim.org/), [RViz](https://docs.ros.org/en/rolling/Tutorials/Intermediate/RViz/RViz-Main.html)
*   **Purpose**: General robotics simulation, ROS-based algorithm testing, visualization.

<a href="/omx/gazebo_omx" class="button-shortcut">
Gazebo Information
</a>
