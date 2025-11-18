# Getting Started with ROBOTIS Lab

## Overview

**ROBOTIS Lab** is a research-oriented repository based on [Isaac Lab](https://isaac-sim.github.io/IsaacLab), designed to enable reinforcement learning and imitation learning experiments using Robotis robots in simulation.
This project provides simulation environments, configuration tools, and task definitions tailored for Robotis hardware, leveraging NVIDIA Isaac Sim’s powerful GPU-accelerated physics engine and Isaac Lab’s modular RL pipeline.

::: info
This repository currently depends on **IsaacLab v2.2.0** or higher.
:::

## Installation (Docker)
Docker installation provides a consistent environment with all dependencies pre-installed.

**Prerequisites:**
- Docker and Docker Compose installed
- NVIDIA Container Toolkit installed
- NVIDIA GPU with appropriate drivers

**Steps:**
1. Clone robotis_lab repository with submodules:

   ```bash
   git clone --recurse-submodules https://github.com/ROBOTIS-GIT/robotis_lab.git
   cd robotis_lab
   ```

   If you already cloned without submodules, initialize them:
   ```bash
   git submodule update --init --recursive
   ```

2. Build and start the Docker container:

   ```bash
   ./docker/container.sh start
   ```

3. Enter the container:

   ```bash
   ./docker/container.sh enter
   ```

**Docker Commands:**
- `./docker/container.sh start` - Build and start the container
- `./docker/container.sh enter` - Enter the running container
- `./docker/container.sh stop` - Stop the container
- `./docker/container.sh logs` - View container logs
- `./docker/container.sh clean` - Remove container and image

**What's included in the Docker image:**
- Isaac Sim 5.1.0
- Isaac Lab v2.3.0 (from third_party submodule)
- CycloneDDS 0.10.2 (from third_party submodule)
- robotis_dds_python (from third_party submodule)
- LeRobot 0.3.3 (in separate virtual environment at `~/lerobot_env`)
- All required dependencies and configurations

## Running Examples
![AI Worker in NVIDIA Isaac Lab](/simulation/ai_worker/ffw_bg2_isaac_lab2.png)

### Reinforcement Learning

You can train and run the **FFW-BG2 Reach Task** using the following commands:

```bash
# Train
python scripts/reinforcement_learning/rsl_rl/train.py --task RobotisLab-Reach-FFW-BG2-v0 --num_envs=512 --headless

# Play
python scripts/reinforcement_learning/rsl_rl/play.py --task RobotisLab-Reach-FFW-BG2-v0 --num_envs=16
```
