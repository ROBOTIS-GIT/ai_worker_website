# Dataset Preparation - Prerequisites

This section describes the necessary setup steps before starting data preparation.

:::info
When executing terminal commands in this document, refer to the indicators below to run them on the correct PC or Docker container

- `USER PC`: Your PC with the camera connected, used for dataset collection with Physical AI Tools
- `🐋 OPEN MANIPULATOR`: Open Manipulator Docker container
- `🐋 PHYSICAL AI TOOLS`: Physical AI Tools Docker container
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
`USER PC` `🐋 OPEN MANIPULATOR`

Then, launch the wrist USB camera node:
```bash
ros2 launch open_manipulator_bringup camera_usb_cam.launch.py
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
