# Dataset Preparation - Prerequisites

This section describes the necessary setup steps before starting data preparation.

:::info
When executing terminal commands in this document, refer to the indicators below to run them on the correct PC or Docker container

- `USER PC`: Your PC that connects to the robot via SSH
- `ROBOT PC`: The NVIDIA Jetson AGX Orin inside the AI Worker robot
- `🐋 AI WORKER`: AI Worker Docker container
- `🐋 PHYSICAL AI TOOLS`: Physical AI Tools Docker container
:::

This section describes the necessary setup steps before starting data preparation.
To begin data preparation, access the `Robot PC` either directly or via SSH. See the [Setup Guide](/ai_worker/setup_guide_ai_worker) for instructions on how to connect via SSH.

## Launch Physical AI Server

::: info
The _Physical AI Server_ is the backend that connects with the Web UI. It should be running to use the interface for data recording.
:::

Open a terminal and enter the **Physical AI_Tools** Docker container:

`ROBOT PC`

```bash
cd ~/physical_ai_tools && ./docker/container.sh enter
```

Launch Physical AI Server with the following command:

`ROBOT PC` `🐋 PHYSICAL AI TOOLS`

```bash
ros2 launch physical_ai_server physical_ai_server_bringup.launch.py
```

Or, use shortcut command:

```bash
ai_server
```

## Launch the ROS 2 teleoperation node

Open a terminal and enter the Docker container:

`ROBOT PC`
```bash
cd ~/ai_worker && ./docker/container.sh enter
```

Then, launch the ROS 2 teleoperation node with the appropriate command for your robot type (**BG2 or SG2**):

::: warning
Executing the code will cause robot to move immediately.
:::

::: info
This command launches the **leader**, **follower**, and **camera** nodes for AI Worker.
:::

`ROBOT PC` `🐋 AI WORKER`
:::tabs key:robot-type
== BG2 Type
```bash
ffw_bg2_ai
```
== SG2 Type
```bash
ffw_sg2_ai
```
:::
