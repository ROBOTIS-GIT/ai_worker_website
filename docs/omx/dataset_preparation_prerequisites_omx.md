# Dataset Preparation - Prerequisites

This section describes the necessary setup steps before starting data preparation.

:::info
When executing terminal commands in this document, refer to the indicators below to run them on the correct PC or Docker container

- `USER PC`: Your PC with the camera connected, used for dataset collection with Physical AI Tools
- `🐋 OPEN MANIPULATOR`: Open Manipulator Docker container
- `🐋 PHYSICAL AI TOOLS`: Physical AI Tools Docker container
:::

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

Please exit the Docker container and return to your host terminal for the next steps.

## Launch the Camera Node

Navigate to **open_manipulator/docker** directory and enter the Docker container:

`USER PC`
```bash
cd open_manipulator/docker
```
```bash
./container.sh enter
```
`USER PC` `🐋 OPEN MANIPULATOR`

Then, launch the wrist USB camera node:
```bash
ros2 launch camera_bringup usb_camera_launch.py
```

## Launch the Physical AI Server

Navigate to **physical_ai_tools/docker** directory and enter the Docker Container:

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

Navigate to **open_manipulator/docker** directory and enter the Docker container:

`USER PC`
```bash
cd open_manipulator/docker
```
```bash
./container.sh enter
```
Then, launch the ROS 2 teleoperation node with the following command:
::: warning
Executing the code will cause OMX to move immediately. Please stay clear and be cautious.
:::

`USER PC` `🐋 OPEN MANIPULATOR`
```bash
ros2 launch open_manipulator_bringup omx_ai.launch.py
``` 