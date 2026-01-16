# Setup Guide for Robotis Hand

This guide will walk you through the process of setting up your ROBOTIS Hand hardware and software environment.

## Hardware Setup

### Hardware Requirements
:::tabs key:robot-type
== HX5-D20
| Required Hardware  | Qty | Description                                                                                                       | Link  |
|--------------------|-----|-------------------------------------------------------------------------------------------------------------------|-------|
| HX5-D20            |  1  | ROBOTIS Hand                                                                                                      |[Link](https://en.robotis.com/shop_en/category.php?ca_id=40)|
| U2D2               |  1  | U2D2 is a small size USB communication converter that enables to control and to operate the DYNAMIXEL with the PC |[Link](https://en.robotis.com/shop_en/item.php?it_id=902-0132-000)|
| SMPS               |  1  | Power supply and battery charging device                                                                         |[Link](https://en.robotis.com/shop_en/item.php?it_id=903-0126-000)|
| U2D2 Power Hub     |  1  | The U2D2 Power Hub Board combines with the U2D2 to supply a variety of external power sources to DYNAMIXELs      |[Link](https://en.robotis.com/shop_en/item.php?it_id=902-0145-001)|
| 4-pin Robot Cable  |  2  | The RS-485 communication 4p cable set for DYNAMIXEL                                                              |[Link](https://en.robotis.com/shop_en/item.php?it_id=903-0244-000)|

<img src="/quick_start_guide/hands/hardware_requirements.png" width="1000"/>

> \* Additional items shown sold separately.

### Hardware Connection

<img src="/quick_start_guide/hands/hardware_connection.png" width="1000"/>
:::

## Software Setup

### Prerequisites
- **Operating System**: Any Linux distribution<br>
  - The container runs `Ubuntu 24.04 (ROS 2 Jazzy)`
- **Container Engine**: Docker Engine
  - Follow the [official Docker installation guide](https://docs.docker.com/engine/install/ubuntu/)
  - Complete the [post-installation steps](https://docs.docker.com/engine/install/linux-postinstall/)
  - Required steps:
    1. Install Docker Engine using the repository method
    2. Add your user to the docker group
    3. Enable Docker to start on boot
    4. Verify installation with `docker run hello-world`
- **Version Control**: Git
---

### Configuration

#### Docker Volume Management
The Docker container uses the following volume mappings for data persistence and hardware access:

```yaml
volumes:
  # Hardware and System Access
  - /dev:/dev                                   # Hardware device access
  - /tmp/.X11-unix:/tmp/.X11-unix:rw            # X11 display
  - /tmp/.docker.xauth:/tmp/.docker.xauth:rw    # X11 authentication

  # Development and Data Storage
  - ./workspace:/workspace                      # Main workspace directory
  - ../:/root/ros2_ws/src/robotis_hand/         # ROBOTIS Hand source code
```

⚠️ **Important: Data Persistence**
- Container data is volatile and will be lost when the container is removed
- Always store important data in the mapped volumes:
  1. Use `/workspace` for development files
  2. Save model outputs to the mapped output directory
  3. Keep source code changes in the mapped volumes

#### Container Management

1. **Initial Setup**
   ```bash
   # Clone the repository
   cd ~/  # or your preferred directory
   git clone -b jazzy https://github.com/ROBOTIS-GIT/robotis_hand.git
   cd robotis_hand
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

