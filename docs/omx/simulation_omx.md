# Simulation

## Overview
The OMX simulation environment is designed to test and validate robot functionalities in a controlled virtual setting. This allows for comprehensive evaluation of its capabilities across a variety of tasks before deployment on the physical hardware. The simulation ecosystem leverages industry-standard tools including ROS 2 providing diverse options for development and research.

## Available Simulation Models and Environments

Currently, OMX simulation is available for Gazebo/RViz using URDF models. The URDF includes the necessary components of OMX for testing and visualization.

### Gazebo / RViz (URDF/SDF)
For seamless integration with the ROS ecosystem, a URDF (Unified Robot Description Format) model is provided. This is primarily used with Gazebo for physics simulation and RViz for 3D visualization of robot states and sensor data.

*   **Model Format**: [URDF](https://docs.ros.org/en/rolling/Tutorials/Intermediate/URDF/URDF-Main.html)
*   **Primary Tools**: [Gazebo Sim](https://gazebosim.org/), [RViz](https://docs.ros.org/en/rolling/Tutorials/Intermediate/RViz/RViz-Main.html)
*   **Purpose**: General robotics simulation, ROS-based algorithm testing, visualization.
*   **Link**: [OMX URDF on GitHub](https://github.com/ROBOTIS-GIT/open_manipulator/tree/main/open_manipulator_description/urdf)

<a href="/omx/gazebo_omx" class="button-shortcut">
Gazebo Information
</a>
