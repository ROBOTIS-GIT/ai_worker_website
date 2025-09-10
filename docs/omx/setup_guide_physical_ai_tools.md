---
next: false
---

# Setup Guide — ROS 2 (Physical AI Tools)

## Overview
This guide shows how to set up and operate OMX using Physical AI Tools (Web UI). Follow the steps to prepare repositories, configure Docker, and run the teleoperation node.

:::info
### System Requirements

| Recommended OS | Ubuntu 24.04 |
| --- | --- |
:::

## Set up Open Manipulator Docker Container

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

### 2. Set up launch file port

Enter the **Open Manipulator** Docker container:

`USER PC`
```bash
./container.sh enter
```
:::info
First, connect only the **'Leader'** USB to the port, then check and copy the OpenRB serial ID.

`USER PC` or `USER PC` `🐋 OPEN MANIPULATOR`
```bash
ls -al /dev/serial/by-id/
```
<div style="max-width: 650px; margin: 12px auto; display: flex; align-items: center; justify-content: center;">
  <img src="/quick_start_guide/omx/setup_port_name_L.png" alt="Serial device by-id listing example" style="width: 100%; height: auto; object-fit: contain; display: block; border-radius: 6px;" />
</div>

As shown in the image below, paste the serial ID you noted above into the port name parameter for the **[leader]** then save.

`USER PC` `🐋 OPEN MANIPULATOR`

```bash
sudo nano ~/ros2_ws/src/open_manipulator/open_manipulator_bringup/launch/omx_l_leader_ai.launch.py
```

<pre class="language-python"><code># omx_l_leader_ai.launch.py
DeclareLaunchArgument(
    'port_name',
    default_value='<mark style="background-color:#fff176; color:#000;">/dev/serial/by-id/</mark><mark style="background-color:#90caf9; color:#000;">your_leader_serial_id</mark>',
    description='Port name for hardware connection.',
)</code></pre>
:::

:::info
Second, connect only the **'Follower'** USB to the port, then check and copy the OpenRB serial ID.

`USER PC` or `USER PC` `🐋 OPEN MANIPULATOR`
```bash
ls -al /dev/serial/by-id/
```
<div style="max-width: 650px; margin: 12px auto; display: flex; align-items: center; justify-content: center;">
  <img src="/quick_start_guide/omx/setup_port_name_F.png" alt="Serial device by-id listing example" style="width: 100%; height: auto; object-fit: contain; display: block; border-radius: 6px;" />
</div>

As shown in the image below, paste the serial ID you noted above into the port name parameter for the **[follower]**, then save.

`USER PC` `🐋 OPEN MANIPULATOR`

```bash
sudo nano ~/ros2_ws/src/open_manipulator/open_manipulator_bringup/launch/omx_f_follower_ai.launch.py
```

<pre class="language-python"><code># omx_f_follower_ai.launch.py
DeclareLaunchArgument(
    'port_name',
    default_value='<mark style="background-color:#fff176; color:#000;">/dev/serial/by-id/</mark><mark style="background-color:#90caf9; color:#000;">your_follower_serial_id</mark>',
    description='Port name for hardware connection.',
)</code></pre>

:::

:::info
Ultimately, it will be changed as shown below.

<div style="max-width: 650px; margin: 12px auto; display: flex; align-items: center; justify-content: center;">
  <img src="/quick_start_guide/omx/setup_port_name.png" alt="Serial device by-id listing example" style="width: 100%; height: auto; object-fit: contain; display: block; border-radius: 6px;" />
</div>
:::

🎉 Open Manipulator Container Setup Complete!

Please exit the Docker container and return to your host terminal for the next steps.

## Set up Physical AI Tools Docker Container

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

### 2. Configure camera topics

If you are using more than one camera or want to use a custom camera, list the available camera topics and choose the one you want to use:

Enter the **Physical AI Tools** Docker container:

`USER PC`
```bash
./container.sh enter
```

list the available topics to find your camera stream:

`USER PC`
```bash
ros2 topic list
```

And open the configuration file and update it as described below:

`USER PC` `🐋 PHYSICAL AI TOOLS`
```bash
sudo nano ~/ros2_ws/src/physical_ai_tools/physical_ai_server/config/omx_config.yaml
```

Then update the fields outlined in red in the UI to point to your desired camera topic.

<div style="max-width: 650px; margin: 12px auto; display: flex; align-items: center; justify-content: center;">
  <img src="/quick_start_guide/omx/setup_camera.png" alt="Configure camera topic in the UI" style="width: 100%; height: auto; object-fit: contain; display: block; border-radius: 6px;" />
</div>

:::: info
Note: The topic you set must always end with `compressed` <br>(for example, `camera1/image_raw/compressed`).
::::

🎉 Physical AI Tools Container Setup Complete!

Click the button below to start Imitation Learning.

<div style='display: flex; justify-content: flex-start; gap: 30px; margin-top: 24px;'>
<a href="/omx/imitation_learning_omx.html" class="button-shortcut">
Imitation Learning<br>Overview
</a>
</div>