# Simulation

## Overview
The ROBOTIS HX hand simulation environment is designed to test and validate robot functionalities in a controlled virtual setting. This allows for comprehensive evaluation of its capabilities across a variety of tasks before deployment on the physical hardware. The simulation ecosystem leverages industry-standard tools including ROS 2 providing diverse options for development and research.

## Available Simulation Models and Environments

The ROBOTIS HX hand can be simulated in various environments using different model formats. These models include all necessary components of the ROBOTIS HX hand, such as the mobile base, dual arms, and sensors, allowing for comprehensive testing.

### Gazebo / RViz (URDF/SDF)
For seamless integration with the ROS ecosystem, URDF (Unified Robot Description Format) and SDF (Simulation Description Format) models are provided. These are primarily used with Gazebo for physics simulation and RViz for 3D visualization of robot states and sensor data.

![ROBOTIS HX hand URDF model in RViz or Gazebo](/simulation/ai_worker/ffw_bg2_urdf.png)

*   **Model Formats**: [URDF](https://docs.ros.org/en/rolling/Tutorials/Intermediate/URDF/URDF-Main.html), [SDF](http://sdformat.org/)
*   **Primary Tools**: [Gazebo Sim](https://gazebosim.org/), [RViz](https://docs.ros.org/en/rolling/Tutorials/Intermediate/RViz/RViz-Main.html)
*   **Purpose**: General robotics simulation, ROS-based algorithm testing, visualization.
*   **Link**: [ROBOTIS HX hand URDF on GitHub](https://github.com/ROBOTIS-GIT/robotis_hand/tree/main/ffw_description/urdf)

<a href="/ai_worker/gazebo_ai_worker" class="button-shortcut">
Gazebo Information
</a>
