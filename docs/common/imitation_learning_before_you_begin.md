# Before You Begin

This section describes the necessary setup steps before starting data preparation.
To begin data preparation, access the `Robot PC` either directly or via SSH. See the [Setup Guide](/ai_worker/setup_guide_ai_worker) for instructions on how to connect via SSH.

## 1. Launch the ROS 2 teleoperation node

a. Open a terminal and enter the Docker container:

:::tabs key:robot-type
== BG2 Type
cd ai_worker && ./docker/container.sh enter
== SG2 Type
cd ai_worker && ./docker/container.sh enter
== OMY
cd open_manipulator && ./docker/container.sh enter
:::

b. Then, launch the ROS 2 teleoperation node using the appropriate command for your robot type:

:::tabs key:robot-type
== BG2 Type
ffw_bg2_ai
== SG2 Type
ffw_sg2_ai
== OMY
ros2 launch open_manipulator_bringup hardware_y.launch.py
:::

## 2. Camera Setup
::: info
This step is required only if your robot is an **OMY** model.
If you are using the **AI Worker**, you may skip this step.
:::

a. Open a new terminal and enter the Docker container:

```bash
cd open_manipulator
./docker/container.sh enter
```
b. Launch the camera node:
```bash
ros2 launch realsense2_camera rs_camera.launch.py
```

You can also use other camera models such as ZED2 or USB cameras, if needed.
