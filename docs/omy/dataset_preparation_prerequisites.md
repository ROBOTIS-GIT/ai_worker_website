# Dataset Preparation - Prerequisites

This section describes the necessary setup steps before starting data preparation.

## 1. Launch the ROS 2 teleoperation node

### a. Access the Robot PC (SBC)
Access the Robot PC either directly or via SSH. For SSH connection instructions, refer to the [SSH connection](/omy/setup_guide_omy#ssh-connection). 

### b. Enter the Docker container
```bash
cd /data/docker/open_manipulator/docker && ./container.sh enter
```
### c. Launch the ROS 2 teleoperation node
Then, launch the ROS 2 teleoperation node with following command:
::: warning
Executing the code will cause OMY to move immediately. Please stay clear and be cautious.
:::
```bash
ros2 launch open_manipulator_bringup omy_ai.launch.py
```

## 2. Camera Setup

### a. Enter the Docker container

Open a new terminal on your host machine and enter the **Open Manipulator** Docker container:
```bash
cd open_manipulator/docker && ./container.sh enter
```

### b. Setup ROS Domain ID

Set a consistent `ROS_DOMAIN_ID` across terminals to enable ROS 2 node communication.
```bash
echo 'export ROS_DOMAIN_ID=30' >> ~/.bashrc
source ~/.bashrc
```
### c. Launch the camera node:
```bash
ros2 launch realsense2_camera rs_launch.py camera_name:='cam_wrist'
```

You can also use other camera models such as ZED2 or USB cameras, if needed.

## 3. Launch Physical AI Server

::: info
The _Physical AI Server_ is the backend that connects with the Web UI. It should be running to use the interface for data recording.
:::

### a. Start and enter the Docker container:

Open a new terminal on your host machine and move to **phisical_ai_tools/docker** directory. 

```bash
cd physical_ai_tools/docker
```

Start the **Physical AI Tools** Docker container with the following command:

::: info
Run `./container.sh start` only when starting the container for the first time.  
If you have already started the container, you can skip this step.
:::

```bash
./container.sh start
```

Enter the Docker container:
```bash
./container.sh enter
```

### b. Setup ROS Domain ID
Set a consistent `ROS_DOMAIN_ID` across terminals to enable ROS 2 node communication:
```bash
echo 'export ROS_DOMAIN_ID=30' >> ~/.bashrc
source ~/.bashrc
```

### c. Build Physical AI Server

Build the Physical AI Server with the following command:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### d. Launch Physical AI Server

Launch Physical AI Server with the following command:

```bash
ros2 launch physical_ai_server physical_ai_server_bringup.launch.py
```
Or, use shortcut command:

```bash
ai_server
```
