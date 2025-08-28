---
next: false
---

# Setup Guide — Physical AI Tools (OMX)

## Overview
This guide shows how to set up and operate OMX using Physical AI Tools (Web UI). Follow the steps to prepare repositories, configure Docker, and run the teleoperation node.

## Download Repositories
Clone the necessary packages for OMX.
```bash
git clone https://github.com/ROBOTIS-GIT/open_manipulator
```

## Docker Setup
1. Connect the OMX to your computer via a USB‑C cable.
2. open_manipulator package is located in `/data/docker/open_manipulator`. Navigate to this location using `cd`:
```bash
cd /data/docker/open_manipulator
```
3. Update the package and recreate the container with the latest docker image:
```bash
./docker/container.sh start
```
4. Access the container:
```bash
./docker/container.sh enter
```
5. Start the Teleop node:
```bash
ros2 launch open_manipulator_bringup omx_ai.launch.py
```
::: tip
The `/workspace` folder inside the container is volume mapped (a feature that links file systems) to `/data/docker/open_manipulator/workspace` on the host. All other areas are volatile and will be lost if the container is damaged or deleted. For more details, see the [Docker Volume Configuration](#docker-volume-configuration) section.
:::


## Set ROS 2 Domain ID
To allow ROS 2 nodes to communicate properly within the same network and avoid conflicts with other systems, you should set a consistent `ROS_DOMAIN_ID`.
```bash
echo 'export ROS_DOMAIN_ID=30' >> ~/.bashrc
source ~/.bashrc
```

🎉 Physical AI Tools Setup Complete!

Click the button below to start Imitation Learning.

<div style='display: flex; justify-content: flex-start; gap: 30px; margin-top: 24px;'>
<a href="/omx/imitation_learning_omx.html" class="button-shortcut">
Imitation Learning<br>Overview
</a>
</div>


