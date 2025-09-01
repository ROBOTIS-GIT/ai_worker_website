---
next: false
---

# Setup Guide — ROS 2 (Physical AI Tools)

## Overview
This guide shows how to set up and operate OMX using Physical AI Tools (Web UI). Follow the steps to prepare repositories, configure Docker, and run the teleoperation node.

## Set up Open Manipulator Docker Container

::: warning
If the **Open Manipulator** Docker container is already set up, **you can skip this step**.
:::

### 1. Start the Docker Container:

Clone the repository:

`USER PC`
```bash
git clone https://github.com/ROBOTIS-GIT/open_manipulator
```
Start the container with the following command:

```bash
cd open_manipulator/docker && ./container.sh start
```

### 2. Set up ROS Domain ID
Set a consistent `ROS_DOMAIN_ID` across terminals to enable ROS 2 node communication.


Enter the **Open Manipulator** Docker container:

`USER PC`
```bash
./container.sh enter
```

`USER PC` `🐋 OPEN MANIPULATOR`

```bash
echo 'export ROS_DOMAIN_ID=30' >> ~/.bashrc
source ~/.bashrc
```
🎉 Open Manipulator Container Setup Complete!

Please exit the Docker container and return to your host terminal for the next steps.

## Set up Physical AI Tools Docker Container

::: warning
If the **Physical AI Tools** Docker container is already set up, **you can skip this step**.
:::

### 1. Start the Docker container

Clone the repository along with all required submodules:

`USER PC`
```bash
git clone --recurse-submodules https://github.com/ROBOTIS-GIT/physical_ai_tools.git
```

Start the **Physical AI Tools** Docker container with the following command:
```bash
cd physical_ai_tools/docker && ./container.sh start
```

### 2. Build the Physical AI Server

Enter the Docker container:

`USER PC`
```bash
./container.sh enter
```
Build the Physical AI Server with the following command:

`USER PC` `🐋 PHYSICAL AI TOOLS`
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

🎉 Physical AI Tools Container Setup Complete!

Click the button below to start Imitation Learning.

<div style='display: flex; justify-content: flex-start; gap: 30px; margin-top: 24px;'>
<a href="/omx/imitation_learning_omx.html" class="button-shortcut">
Imitation Learning<br>Overview
</a>
</div>
