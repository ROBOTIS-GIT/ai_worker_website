# Simulation

## Overview
The AI Worker simulation environment is designed to test and validate robot functionalities in a controlled virtual setting. This allows for comprehensive evaluation of its capabilities across a variety of tasks before deployment on the physical hardware. The simulation ecosystem leverages industry-standard tools including ROS 2, Google DeepMind's MuJoCo, and NVIDIA Isaac Sim, providing diverse options for development and research.

## Simulation for Reinforcement Learning
Our simulation environments are particularly well-suited for reinforcement learning (RL). They provide realistic physics and sensor feedback, enabling the AI Worker to learn and adapt its behaviors through interaction. This is crucial for developing complex manipulation and navigation skills.

## Available Simulation Models and Environments

The AI Worker can be simulated in various environments using different model formats. These models include all necessary components of the AI Worker, such as the mobile base, dual arms, and sensors, allowing for comprehensive testing.

### Gazebo / RViz (URDF/SDF)
For seamless integration with the ROS ecosystem, URDF (Unified Robot Description Format) and SDF (Simulation Description Format) models are provided. These are primarily used with Gazebo for physics simulation and RViz for 3D visualization of robot states and sensor data.

![AI Worker URDF model in RViz or Gazebo](/simulation/simulation_urdf.png)

*   **Model Formats**: URDF, SDF
*   **Primary Tools**: Gazebo Sim, RViz
*   **Purpose**: General robotics simulation, ROS-based algorithm testing, visualization.
*   **Link**: [AI Worker URDF on GitHub](https://github.com/ROBOTIS-GIT/ai_worker/tree/main/ffw_description/urdf)

### MuJoCo (MJCF)
For high-fidelity physics simulation, particularly favored in reinforcement learning research and biomechanics, models in MJCF (MuJoCo XML Format) are available. MuJoCo offers fast and accurate physics, ideal for training complex behaviors.

![AI Worker MJCF model in MuJoCo](/simulation/simulation_mujoco.png)

*   **Model Format**: MJCF
*   **Primary Tool**: MuJoCo (by Google DeepMind)
*   **Purpose**: High-fidelity physics simulation, reinforcement learning, advanced dynamics research.
*   **Link**: [ROBOTIS MuJoCo Menagerie](https://github.com/ROBOTIS-GIT/robotis_mujoco_menagerie)

### NVIDIA Isaac Sim (USD)
Support for NVIDIA Isaac Sim is an exciting upcoming feature. Isaac Sim utilizes the USD (Universal Scene Description) format for photorealistic rendering and advanced physics simulation, tightly integrated with NVIDIA's AI and robotics SDKs.

![AI Worker USD model in NVIDIA Isaac Sim (Coming Soon)](/simulation/simulation_isaac_sim.png)

*   **Model Format**: USD
*   **Primary Tool**: NVIDIA Isaac Sim
*   **Purpose**: Photorealistic simulation, advanced physics, synthetic data generation, AI-driven robotics development.
*   **Status**: Coming Soon
*   **Link**: (TBD)
