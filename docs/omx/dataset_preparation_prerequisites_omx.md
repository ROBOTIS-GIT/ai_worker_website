# Dataset Preparation - Prerequisites

This section describes the necessary setup steps before starting data preparation.

:::info
When executing terminal commands in this document, refer to the indicators below to run them on the correct PC or Docker container

- `USER PC`: Your PC with the camera connected, used for dataset collection with Physical AI Tools
- `🐋 OPEN MANIPULATOR`: Open Manipulator Docker container
- `🐋 PHYSICAL AI TOOLS`: Physical AI Tools Docker container
:::

## Setup Open Manipulator Docker Container

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

### 2. Setup ROS Domain ID
Set a consistent `ROS_DOMAIN_ID` across terminals to enable ROS 2 node communication.


Enter the **Open Manipulator** Docker container:

`USER PC`
```bash
./container.sh enter
```

`USER PC` ➔ `🐋 OPEN MANIPULATOR`

```bash
echo 'export ROS_DOMAIN_ID=30' >> ~/.bashrc
source ~/.bashrc
```
::: tip 🎉 Open Manipulator Container Setup Complete!

Please exit the Docker container and return to your host terminal for the next steps.
:::

## Setup Physical AI Tools Docker Container

::: warning
If the **Physical AI Tools** Docker container is already set up, **you can skip this step**.
:::

### 1. Start the Docker Container:

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

Enter the docker container:

`USER PC`
```bash
./container.sh enter
```
Build the Physical AI Server with the following command:

`USER PC` ➔ `🐋 PHYSICAL AI TOOLS`
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
::: tip 🎉 Physical AI Tools Container Setup Complete!

Please exit the Docker container and return to your host terminal for the next steps.
:::

## Launch the Camera Node

Navigate to **open_manipulator/docker** directory and enter the Docker container:

`USER PC`
```bash
cd open_manipulator/docker
```
```bash
./container.sh enter
```

Launch the Camera Node:

`USER PC` ➔ `🐋 OPEN MANIPULATOR`

**Option 1: USB Camera (Recommended)**
```bash
ros2 launch usb_cam usb_cam_launch.py camera_name:='cam_wrist'
```

**Option 2: RealSense Camera**
```bash
ros2 launch realsense2_camera rs_launch.py camera_name:='cam_wrist'
```



Choose the appropriate option based on your camera hardware.


## Launch the Physical AI Server

Navigate to **phisical_ai_tools/docker** directory and esnter the Docker Container:

`USER PC`
```bash
cd physical_ai_tools/docker
```
```bash
./container.sh enter
```

Launch Physical AI Server with the following command:

`USER PC` `🐋 PHYSICAL AI TOOLS`
```bash
ros2 launch physical_ai_server physical_ai_server_bringup.launch.py
```
Or, use shortcut command:

```bash
ai_server
```

## Launch the ROS 2 Teleoperation Node

### 1. Access the Robot PC
Access the Robot PC either directly or via SSH. For SSH connection instructions, refer to the [SSH connection](/omx/setup_guide_omx#ssh-connection). 

### 2. Launch the ROS 2 Teleoperation Node

Enter the Docker Container

`ROBOT PC`
```bash
cd /data/docker/open_manipulator/docker && ./container.sh enter
```

Then, launch the ROS 2 teleoperation node with following command:
::: warning
Executing the code will cause OMX to move immediately. Please stay clear and be cautious.
:::
`ROBOT PC` `🐋 OPEN MANIPULATOR`
```bash
ros2 launch open_manipulator_bringup omx_ai.launch.py
``` 