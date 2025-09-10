# Getting Started with Gazebo

## Overview
You can launch Gazebo and RViz individually, or launch RViz together with the physical hardware.

## 1. Prepare the OM Container

Refer to the [ROS 2 (Physical AI Tools) Setup Guide](/omx/setup_guide_physical_ai_tools.html) to configure the OM container environment.


## 2. Enable GUI Access

Open a new terminal and run the following command:

```bash
xhost +
```


## 3. Access the Running Container

```bash
./container.sh enter
```


## 4. Launch Gazebo

Launch the simulation environment using:

```bash
ros2 launch open_manipulator_bringup omx_f_gazebo.launch.py
```


## 5. Run MoveIt and GUI in Simulation

Refer to the [Operation Guide](/omx/operation_omx.html) to control the robot within the Gazebo environment.


## Simulation Views

### OMX_F

![OMX F Simulation](/simulation/omx/omx_f_gazebo.png)
