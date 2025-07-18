# Getting Started with ROBOTIS Lab

## Overview

**ROBOTIS Lab** is a research-oriented repository based on [Isaac Lab](https://isaac-sim.github.io/IsaacLab), designed to enable reinforcement learning and imitation learning experiments using Robotis robots in simulation.
This project provides simulation environments, configuration tools, and task definitions tailored for Robotis hardware, leveraging NVIDIA Isaac Sim’s powerful GPU-accelerated physics engine and Isaac Lab’s modular RL pipeline.

::: info
This repository currently depends on **IsaacLab v2.0.0** or higher.
:::

## Installation
[Youtube Guide](https://www.youtube.com/watch?v=GHkyxmOy5-I)

1. Follow the [Isaac Lab installation guide](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html) to set up the environment.
  Instead of the recommended local installation, you can run Isaac Lab in a Docker container to simplify dependency management and ensure consistency across systems.

2. Clone the Isaac Lab Repository:
  ```bash
  git clone https://github.com/isaac-sim/IsaacLab.git
  ```

3. Start and enter the Docker container:
  ```bash
  # start
  ./IsaacLab/docker/container.py start base

  # enter
  ./IsaacLab/docker/container.py enter base
  ```


4. Clone the robotis_lab repository (outside the IsaacLab directory):

  ```bash
  cd /workspace && git clone https://github.com/ROBOTIS-GIT/robotis_lab.git
  ```

5. Install the robotis_lab Package.

  ```bash
  cd robotis_lab
  python -m pip install -e source/robotis_lab
  ```

6. Verify that the extension is correctly installed by listing all available environments:

  ```bash
  python scripts/tools/list_envs.py
  ```

  Once the installation is complete, the available training tasks will be displayed as shown below:
  ![run list_env](/simulation/all/isaaclab_list_envs.png)

## Running Examples
![OMY in NVIDIA Isaac Lab](/simulation/omy/omy_isaac_lab2.png)

### Reinforcement Learning

You can train and run the **FFW-BG2 Reach Task** using the following commands:

```bash
# Train
python scripts/reinforcement_learning/rsl_rl/train.py --task RobotisLab-Reach-OMY-v0 --num_envs=512 --headless

# Play
python scripts/reinforcement_learning/rsl_rl/play.py --task RobotisLab-Reach-OMY-v0 --num_envs=16
```
