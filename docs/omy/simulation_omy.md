# Simulation

## Overview
The OMY simulation environment is designed to test and validate robot functionalities in a controlled virtual setting. This allows for comprehensive evaluation of its capabilities across a variety of tasks before deployment on the physical hardware. The simulation ecosystem leverages industry-standard tools including ROS 2, Google DeepMind's MuJoCo, and NVIDIA Isaac Sim, providing diverse options for development and research.

## Simulation for Reinforcement Learning
Our simulation environments are particularly well-suited for reinforcement learning (RL). They provide realistic physics and sensor feedback, enabling the OMY to learn and adapt its behaviors through interaction. This is crucial for developing complex manipulation and navigation skills.

## Available Simulation Models and Environments

The OMY can be simulated in various environments using different model formats. These models include all necessary components of the OMY, such as the mobile base, dual arms, and sensors, allowing for comprehensive testing.

### Gazebo / RViz (URDF/SDF)
For seamless integration with the ROS ecosystem, URDF (Unified Robot Description Format) and SDF (Simulation Description Format) models are provided. These are primarily used with Gazebo for physics simulation and RViz for 3D visualization of robot states and sensor data.

![OMY URDF model in RViz or Gazebo](/simulation/omy/omy_urdf.png)

*   **Model Formats**: [URDF](https://docs.ros.org/en/rolling/Tutorials/Intermediate/URDF/URDF-Main.html), [SDF](http://sdformat.org/)
*   **Primary Tools**: [Gazebo Sim](https://gazebosim.org/), [RViz](https://docs.ros.org/en/rolling/Tutorials/Intermediate/RViz/RViz-Main.html)
*   **Purpose**: General robotics simulation, ROS-based algorithm testing, visualization.
*   **Link**: [OMY URDF on GitHub](https://github.com/ROBOTIS-GIT/open_manipulator/tree/main/open_manipulator_description/urdf)

<a href="/omy/gazebo_omy" class="button-dataset-preparation-option">
Gazebo Information
</a>

### MuJoCo (MJCF)
For high-fidelity physics simulation, particularly favored in reinforcement learning research and biomechanics, models in MJCF (MuJoCo XML Format) are available. MuJoCo offers fast and accurate physics, ideal for training complex behaviors.

![OMY MJCF model in MuJoCo](/simulation/omy/omy_mujoco.png)

*   **Model Format**: [MJCF](https://mujoco.readthedocs.io/en/stable/XMLreference.html)
*   **Primary Tool**: [MuJoCo (by Google DeepMind)](https://mujoco.org/)
*   **Purpose**: High-fidelity physics simulation, reinforcement learning, advanced dynamics research.
*   **Link**: [OMY MJCF on GitHub](https://github.com/ROBOTIS-GIT/robotis_mujoco_menagerie)

### NVIDIA Isaac Sim (USD)
Support for NVIDIA Isaac Sim is an exciting upcoming feature. Isaac Sim utilizes the USD (Universal Scene Description) format for photorealistic rendering and advanced physics simulation, tightly integrated with NVIDIA's AI and robotics SDKs.

![OMY USD model in NVIDIA Isaac Sim (Coming Soon)](/simulation/omy/omy_isaac_sim.png)

*   **Model Format**: [USD](https://docs.isaacsim.omniverse.nvidia.com/latest/omniverse_usd/open_usd.html)
*   **Primary Tool**: [NVIDIA Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com/)
*   **Purpose**: Photorealistic simulation, advanced physics, synthetic data generation, AI-driven robotics development.
*   **Link**: [OMY USD on GitHub](https://github.com/ROBOTIS-GIT/robotis_lab/tree/main/source/robotis_lab/data/robots)

### NVIDIA Isaac Lab
Isaac Lab is an advanced reinforcement learning framework built on top of NVIDIA Isaac Sim. It provides a scalable infrastructure for training and evaluating robotic agents in high-fidelity simulated environments.

![OMY in NVIDIA Isaac Lab](/simulation/omy/omy_isaac_lab.gif)

*   **Primary Tool**: [NVIDIA Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com/)
*   **Framework**: [NVIDIA Isaac Lab](https://github.com/isaac-sim/IsaacLab/)
*   **Purpose**: Specialized framework for reinforcement and imitation learning using Isaac Sim.
*   **Link**: [ROBOTIS Lab on GitHub](https://github.com/ROBOTIS-GIT/robotis_lab/)

<a href="/ai_worker/robotis_lab_ai_worker" class="button-dataset-preparation-option">
ROBOTIS LAB Information
</a>
