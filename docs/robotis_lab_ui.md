# Robotis Lab Web UI

## Overview

**robotis_lab** is a research-oriented repository based on [Isaac Lab](https://isaac-sim.github.io/IsaacLab), designed to enable reinforcement learning (RL) and imitation learning (IL) experiments using Robotis robots in simulation.
This project provides simulation environments, configuration tools, and task definitions tailored for Robotis hardware, leveraging NVIDIA Isaac Sim’s powerful GPU-accelerated physics engine and Isaac Lab’s modular RL pipeline.

> [!IMPORTANT]
> This repository currently depends on **IsaacLab v2.0.0** or higher.
>

## Installation

1. Follow the [Isaac Lab installation guide](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html) to set up the environment.  
  Instead of the recommended local installation, we installed and ran Isaac Lab within a Docker container environment to simplify dependency management and ensure consistency across systems.

2. Clone the Isaac lab Repository:
  ```bash
  git clone https://github.com/isaac-sim/IsaacLab.git
  ```

3. start and enter the Docker container:
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

5. Install the robotis_lab Package

  ```bash
  cd robotis_lab
  python -m pip install -e source/robotis_lab
  ```

6. Verify that the extension is correctly installed by listing all available environments:

  ```bash
  python scripts/tools/list_envs.py
  ```

  Once the installation is complete, the available training tasks will be displayed as shown below:
  ![run list_env](/simulation/simulaton_isaaclab_list_envs.png)

## Try examples
![AI Worker in NVIDIA Isaac Lab](/simulation/simulation_isaac_lab_ffw2.png)

### Reinforcement learning

FFW-BG2 Reach task

```bash
# Train
python scripts/reinforcement_learning/skrl/train.py --task RobotisLab-Reach-FFW-BG2-v0 --num_envs=512 --headless

# Play
python scripts/reinforcement_learning/skrl/play.py --task RobotisLab-Reach-FFW-BG2-v0 --num_envs=16
```
