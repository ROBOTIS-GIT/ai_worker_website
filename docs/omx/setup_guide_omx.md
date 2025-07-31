# Setup Guide for OMX

This guide will walk you through the process of setting up your OMX hardware and software environment.

## Hardware Setup
### OMX-F
![](/quick_start_guide/omy/hardware_setup_OMY-3M.jpg)
### OMX-L
![](/quick_start_guide/omy/hardware_setup_OMY-F3M.jpg)
### OMX-AI
![](/quick_start_guide/omy/hardware_setup_OMY-L100.jpg)


## Power connection
![omy_power](/quick_start_guide/omy/omy_power.png)


## Download Repositories
Clone the necessary packages for OMX.
```bash
git clone https://github.com/ROBOTIS-GIT/open_manipulator
git clone --recurse-submodules https://github.com/ROBOTIS-GIT/physical_ai_tools.git

```

## Docker Setup
1. Connect to the OMX via SSH.
2. open_manipulator package is located in `/data/docker/open_manipulator`. Navigate to this location using `cd`:
```bash
cd /data/docker/open_manipulator
```
3. Update the package and recreate the container with the latest docker image:
```bash
git checkout jazzy
git pull
./docker/container.sh start
```
4. Access the container:
```bash
./docker/container.sh enter
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