# Simulation

## Overview
This simulation environment is designed to test the AI Worker in a controlled setting, allowing for the evaluation of its capabilities in a variety of tasks. The simulation is built using ROS 2, Google Deepmind's MuJoCo and NVIDIA Isaac Sim.

## Simulation Environments for Reinforcement Learning
The simulation environments facilitate reinforcement learning (RL) training and evaluation, providing realistic settings for the AI Worker to learn and adapt its behaviors through environmental interaction.

## Available Simulation Models

| Model Format | Purpose | Link |
|--------------|---------|------|
| **URDF/SDF** | For Gazebo Sim and RViz visualization and simulation | [AI Worker Repository](https://github.com/ROBOTIS-GIT/ai_worker) |
| **MJCF** | For MuJoCo high-fidelity physics simulation | [ROBOTIS MuJoCo Menagerie](https://github.com/ROBOTIS-GIT/robotis_mujoco_menagerie) |
| **USD** | For NVIDIA Isaac Sim advanced rendering and physics (Coming Soon) | (TBD) |

These simulation models include all necessary components of the AI Worker, such as the mobile base, dual arms, and sensors, allowing for comprehensive testing in virtual environments before deployment to physical robots.
