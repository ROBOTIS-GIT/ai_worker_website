# Setup Guide for OMY

This guide will walk you through the process of setting up your OMY hardware and software environment.

## Hardware Setup
### OMY-3M
![](/quick_start_guide/omy/hardware_setup_OMY-3M.jpg)
### OMY-F3M
![](/quick_start_guide/omy/hardware_setup_OMY-F3M.jpg)
### OMY-L100
![](/quick_start_guide/omy/hardware_setup_OMY-L100.jpg)
### OMY-AI3M
![](/quick_start_guide/omy/hardware_setup_OMY-AI3M.jpg)


## Power connection
![omy_power](/quick_start_guide/omy/omy_power.png)

## SSH connection

### Network Access Method
Start by connecting the `robot pc` to the same network as the `user PC` using a LAN cable, then power on the OMY (press and release the power button until it turns white). The OMY OS uses mDNS technology to discover its IP address.
Each time OMY OS boots, it sets the SBC’s hostname to the serial number (SN) written on the product (e.g., SNPR44B9999).
In environments where UDP Multicast is available (such as being on the same router), you can connect directly to the `robot pc` using the hostname. Static IP assignment and other network settings for more advanced connection options are supported through the Manager.

![omy_serial_number](/quick_start_guide/omy/omy_serial_number.png)

### SSH Access Method
(Assuming the SN is ‘SNPR44B9999’)
To access via SSH over the local network from a Linux environment, use the following command:
```bash
ssh root@omy-SNPR44B9999.local
```

## Docker Setup
1. Connect to the OMY via SSH.
2. open_manipulator package is located in `/data/docker/open_manipulator`. Navigate to this location using `cd`:
```bash
cd /data/docker/open_manipulator
```
3. Update the package and recreate the container with the latest docker image:
```bash
git checkout jazzy && git pull && ./docker/container.sh start
```
4. Access the container:
```bash
./docker/container.sh enter
```

::: tip
The `/workspace` folder inside the container is volume mapped (a feature that links file systems) to `/data/docker/open_manipulator/workspace` on the host. All other areas are volatile and will be lost if the container is damaged or deleted. For more details, see the [Docker Volume Configuration](#docker-volume-configuration) section.
:::


## Unpacking

::: danger
- Following initial setup, the Unpacking script must be executed to prevent self-collision.
- **Run this script ONLY in the packed posture, running it in any other orientation may cause damage.**
:::

When you first receive the OMY, the manipulator is folded as shown in the image below.

![omy_pack](/quick_start_guide/omy/omy_pack.png)

You can move it to the initial position by running the following command for **UNPACKING**:

```bash
ros2 launch open_manipulator_bringup unpack_y.launch.py
```

The image below shows the initial position after **UNPACKING**.

![omy_unpack](/quick_start_guide/omy/omy_unpack.png)


and the following is the command to PACK it back into it's folded configuration.
```bash
ros2 launch open_manipulator_bringup pack_y.launch.py
```

## Software Setup
::: warning
This setup is intended for development on the **user PC**. The **robot PC** included with OMY comes pre-configured with the same software stack.
:::

### Software Setup for OMY

The **ROBOTIS OMY** robotic arm utilizes two key software packages to enable intelligent manipulation through **Physical AI**:

- **[open_manipulator](https://github.com/ROBOTIS-GIT/open_manipulator)**: Provides control of the **6-DOF OMY arm** via ros2_control using **DYNAMIXEL-Y** actuators. Supports teleoperation and low-level control.
- **[physical_ai_tools](https://github.com/ROBOTIS-GIT/physical_ai_tools)**: A toolkit for **imitation learning**, including modules for data collection, training, inference, and visualization.

### Prerequisites

* **Operating System**: Any Linux distribution

  * The container runs **Ubuntu 24.04 + ROS 2 Jazzy**
  * The Host OS version does **not** need to match.

* **Docker Engine**

  * Install using the [official Docker guide](https://docs.docker.com/engine/install/)
  * After installation:

    ```bash
    sudo usermod -aG docker $USER
    sudo systemctl enable docker
    docker run hello-world
    ```

* **Git**

  ```bash
  sudo apt install git
  ```

### Docker Volume Configuration

The Docker container uses volume mappings for **hardware access**, **development**, and **data persistence**:

```bash
volumes:
  # Hardware and system access
  - /dev:/dev
  - /tmp/.X11-unix:/tmp/.X11-unix:rw
  - /tmp/.docker.xauth:/tmp/.docker.xauth:rw

  # Development and data directories
  - ./workspace:/workspace
  - ../:/root/ros2_ws/src/open_manipulator/
```

::: tip
Store your development code in `/workspace` to preserve your codes.
:::

## Container Lifecycle

### Initial Setup

```bash
# Clone the open_manipulator repository
cd ~
git clone -b jazzy https://github.com/ROBOTIS-GIT/open_manipulator.git
cd open_manipulator
```

### Starting and Managing the Container

```bash
# Start container
./docker/container.sh start

# Enter the running container
./docker/container.sh enter

# Stop the container
./docker/container.sh stop

```
::: warning
Stopping the container with `./container.sh stop` will remove the container. Any files that are not located in the volume-mapped directories (such as `/workspace` or other mapped paths) will be permanently deleted. Make sure to save your important files in the appropriate volume-mapped locations to avoid data loss. For more details, see the [Docker Volume Configuration](#docker-volume-configuration) section.
:::
### Docker Command Reference

| Command            | Description                    |
| ------------------ | ------------------------------ |
| `help`             | Display usage help             |
| `start`    | Start container     |
| `enter`            | Enter the running container    |
| `stop`             | Stop and remove the container             |

### Example Usage

```bash
./container.sh help
./container.sh start
./container.sh enter
./container.sh stop
```
