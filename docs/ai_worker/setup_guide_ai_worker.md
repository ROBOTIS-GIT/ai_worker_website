# Setup Guide for AI Worker

This guide will walk you through the process of setting up your AI Worker hardware and software environment.

## Hardware Setup
::: info
- The term `Follower` refers to the body of the AI WORKER robot
- Please check the type of robot you are using.
:::

### FFW-BG2
#### Powering On the FFW_BG2 follower
![Back of the BG2 Base](/quick_start_guide/ai_worker/back_of_the_BG2_base.png)
1. Toggle the `Power Supply Switch` to the right.
2. Insert the `Key Switch` and turn it to the 12 o'clock position.
3. Press and hold the `Power Button` for 3 seconds. When you hear a beep, the system is powered on. (You should see the robot’s head light up at this point.)


#### Hardware Ports (FFW_BG2 Follower)
![Back of the BG2 Body](/quick_start_guide/ai_worker/back_of_the_BG2_body.png)
The back of the Follower body includes several ports for system access and external connections. These include:
- `LAN Port`: Used to access the robot PC via SSH or remote desktop.

- `USB Ports`: For connecting peripherals such as a keyboard, mouse, or USB drive.

- `HDMI Port`: Allows direct video output for connecting a monitor.



### FFW-SG2
#### Powering On the FFW_SG2 follower
![Back of the SG2 Base](/quick_start_guide/ai_worker/back_of_the_SG2_base.png)
1. Insert the `Key Switch` and turn it to the 12 o'clock position.
2. Press and hold the `Power Button` for 3 seconds. When you hear a beep, the system is powered on. (You should see the robot’s head light up at this point.)

#### Hardware Ports (FFW_SG2 Follower)
![Back of the SG2 body](/quick_start_guide/ai_worker/back_of_the_SG2_body.png)
The back of the Follower body includes several ports for system access and external connections. These include:
- `LAN Port`: Used to access the robot PC via SSH or remote desktop.

- `USB Ports`: For connecting peripherals such as a keyboard, mouse, or USB drive.

- `HDMI Port`: Allows direct video output for connecting a monitor.

- `Charge Port`: Used for battery charging.

## Software Setup
AI WORKER relies on two main repositories:
- [ai_worker](https://github.com/ROBOTIS-GIT/ai_worker): Provides support for controlling DYNAMIXEL actuators using ros2_control and enables teleoperation functionality.
- [physical_ai_tools](https://github.com/ROBOTIS-GIT/physical_ai_tools): A set of tools for imitation learning, including data collection, training, model inference, and visualization utilities.

The software setup instructions below are intended for development on a `user PC`. **_Note that the robot PC on the AI WORKER is already pre-configured with the same setup._**

::: danger
⚠️ **On the AI Worker Orin:** Do not upgrade packages using the `apt upgrade` command. Upgrading may cause conflicts between packages.
:::
### Prerequisites
- **Operating System**: Ubuntu environment<br>
(The AI WORKER software runs inside a Docker container based on `Ubuntu 24.04 (ROS 2 Jazzy)`. Therefore, the Ubuntu version of the user PC does not need to match and is not critical.)
- **Container Engine**: Docker Engine
  - Follow the [official Docker installation guide](https://docs.docker.com/engine/install/ubuntu/)
  - Complete the [post-installation steps](https://docs.docker.com/engine/install/linux-postinstall/)
  - Required steps:
    1. Install Docker Engine using the repository method
    2. Add your user to the docker group
    3. Enable Docker to start on boot
    4. Verify installation with `docker run hello-world`
- **Version Control**: Git
- **Graphics Support**:
  - NVIDIA Graphics Driver<br>
  (In Docker-based environments, the container is generally isolated from the host system. However, the graphics driver version on the host can have a direct impact on performance and compatibility—especially when using GPU acceleration. We recommend using the following driver version(s) for best results)
    - Install `nvidia-driver-570-server-open` for `CUDA 12.8`
    - Verify installation with `nvidia-smi`
  - NVIDIA Container Toolkit
    - Follow the [official installation guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#with-apt-ubuntu-debian)
    - Required steps:
      1. Configure the production repository
      2. Install `nvidia-container-toolkit`
      3. Configure Docker runtime using `nvidia-ctk`
      4. Restart Docker daemon
    - For detailed configuration, see the [Docker configuration guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#configuring-docker)

---
### Configuration

#### Docker Volume Management
The Docker container uses the following volume mappings for data persistence and hardware access:

```yaml
volumes:
  # Hardware and System Access
  - /dev:/dev                    # Hardware device access
  - /tmp/.X11-unix:/tmp/.X11-unix:rw  # X11 display
  - /tmp/.docker.xauth:/tmp/.docker.xauth:rw  # X11 authentication

  # Development and Data Storage
  - ./workspace:/workspace       # Main workspace directory
  - ../:/root/ros2_ws/src/ai_worker/  # AI Worker source code
  - ./lerobot/outputs:/root/ros2_ws/src/physical_ai_tools/lerobot/outputs  # Model outputs
  - ./huggingface:/root/.cache/huggingface  # Hugging Face cache
```

⚠️ **Important: Data Persistence**
- Container data is volatile and will be lost when the container is removed
- Always store important data in the mapped volumes:
  1. Use `/workspace` for development files
  2. Save model outputs to the mapped output directory
  3. Keep source code changes in the mapped ai_worker directory
  4. Store downloaded models in the huggingface cache

#### Container Management

1. **Initial Setup**
   ```bash
   # Clone the repository
   cd ~/  # or your preferred directory
   git clone -b jazzy https://github.com/ROBOTIS-GIT/ai_worker.git
   cd ai_worker
   ```

2. **Container Operations**
   ```bash
   # Start container
   ./docker/container.sh start

   # Enter running container
   ./docker/container.sh enter

   # Stop container
   ./docker/container.sh stop
   ```

### Docker Command Guide

The `container.sh` script provides easy container management:

#### Available Commands
- `help`: Display help message
- `start`: Start container
- `enter`: Enter running container
- `stop`: Stop container

#### Usage Examples
```bash
./container.sh help                 # Show help
./container.sh start                # Start container
./container.sh enter                # Enter container
./container.sh stop                 # Stop container
```

## Accessing the Robot PC via SSH
AI Worker supports mDNS, allowing you to connect without manually checking the IP address.

1. Connect the SBC (robot PC) to the same network as your user PC using a LAN cable.

2. On your user PC terminal, use the following command to connect via SSH:
 ```bash
  ssh robotis@ffw-SNPR48A0000.local
 ```
(Replace SNPR48A0000 with the serial number printed on the back of the robot body.)
![Back of the Follower](/quick_start_guide/ai_worker/serial_number.png)

