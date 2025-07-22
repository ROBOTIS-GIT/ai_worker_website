# Dataset Preparation - Prerequisites

This section describes the necessary setup steps before starting data preparation.

## 1. Launch the ROS 2 Teleoperation Node

### a. Access the Robot PC (SBC)
Access the Robot PC either directly or via SSH. For SSH connection instructions, refer to the [SSH connection](/omy/setup_guide_omy#ssh-connection). 

### b. Enter the Docker Container
`ROBOT PC`
```bash
cd /data/docker/open_manipulator/docker && ./container.sh enter
```
### c. Launch the ROS 2 Teleoperation Node
Then, launch the ROS 2 teleoperation node with following command:
::: warning
Executing the code will cause OMY to move immediately. Please stay clear and be cautious.
:::
`ROBOT PC` `🐋 OPEN MANIPULATOR`
```bash
ros2 launch open_manipulator_bringup omy_ai.launch.py
```

## 2. Camera Setup

### a. Enter the Docker Container

::: info
Run `.open_manipulator/docker/container.sh start` only when starting the container for the first time. If you have already started the container, you can skip this step.
:::

Open a new terminal on your host machine and enter the **Open Manipulator** Docker container:

`USER PC` `🐋 OPEN MANIPULATOR`
```bash
cd open_manipulator/docker && ./container.sh enter
```
### b. Setup ROS Domain ID
Set a consistent `ROS_DOMAIN_ID` across terminals to enable ROS 2 node communication.

`USER PC`
```bash
echo 'export ROS_DOMAIN_ID=30' >> ~/.bashrc
source ~/.bashrc
```
### c. Launch the Camera Node:

`USER PC` `🐋 OPEN MANIPULATOR`
```bash
ros2 launch realsense2_camera rs_launch.py camera_name:='cam_wrist'
```

You can also use other camera models such as ZED2 or USB cameras, if needed.

## 3. Launch Physical AI Server

::: info
Run `.physical_ai_tools/docker/container.sh start` only when starting the container for the first time. If you have already started the container, you can skip this step.
:::

### a. Start and Enter the Docker Container:

Clone the repository along with all required submodules:

`USER PC`
```bash
git clone --recurse-submodules https://github.com/ROBOTIS-GIT/physical_ai_tools.git
```

Open a new terminal on your host machine and move to **phisical_ai_tools/docker** directory. 

`USER PC`
```bash
cd physical_ai_tools/docker
```

Start the **Physical AI Tools** Docker container with the following command:

Enter the Docker Container:

`USER PC`
```bash
./container.sh enter
```

### b. Setup ROS Domain ID
Set a consistent `ROS_DOMAIN_ID` across terminals to enable ROS 2 node communication:

`USER PC`
```bash
echo 'export ROS_DOMAIN_ID=30' >> ~/.bashrc
source ~/.bashrc
```

### c. Build Physical AI Server

Build the Physical AI Server with the following command:

`USER PC` `🐋 PHYSICAL AI TOOLS`
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### d. Launch Physical AI Server

Launch Physical AI Server with the following command:

`USER PC` `🐋 PHYSICAL AI TOOLS`
```bash
ros2 launch physical_ai_server physical_ai_server_bringup.launch.py
```
Or, use shortcut command:

`USER PC` `🐋 PHYSICAL AI TOOLS`
```bash
ai_server
```
