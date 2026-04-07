# VR Teleoperation

This guide explains how to set up the VR device and operate the AI Worker using VR device.

## VR Setup

### 1. Meta Horizon app (developer mode)

On the Meta Horizon mobile app:

**Menu → Device management → Devices → Headset settings → Developer Mode → On**

<div style="display: flex; flex-wrap: wrap; gap: 12px;">
  <img src="/vr/menu_meta.png" alt="menu_meta" width="22%" />
  <img src="/vr/headset_settings_meta.png" alt="headset_settings_meta" width="22%" />
  <img src="/vr/developer_mode_meta.png" alt="developer_mode_meta" width="22%" />
  <img src="/vr/developer_mode_toggle_meta.png" alt="developer_mode_toggle_meta" width="22%" />
</div>

### 2. On-headset settings

1. **Power / sleep** — **Settings → General** (use **Power** or **Display** depending on your firmware): set **Display off** and **Sleep mode** to **4 hours** (or the longest option available) so the headset does not sleep during use.

![Quest power and sleep settings](/vr/power_setting_meta.png)

2. **Play area** — **Settings → Environment → Stationary boundary size** → **Large**.

![Stationary boundary set to Large](/vr/boundary_large_meta.png)

   Alternatively, use **Create new boundary** to define the boundary manually.

## VR Startup

AI Worker VR teleoperation uses **Vuer** as the browser-based VR client. For more information about the VR stack and Vuer itself, see the **[ROBOTIS Vuer](/ai_worker/vuer_overview_ai_worker)**.

### 1. Clone the repository

Clone the repository on either the `Robot PC` or the `USER PC`, depending on your workflow.

The default setup is to clone it on the `Robot PC`.

**Option 1**: clone the repository on the `Robot PC`:

```bash
ssh robotis@ffw-SNPR48A0000.local
```

Replace `SNPR48A0000` with the serial number printed on the back of the robot body.

```bash
cd ~/
git clone -b jazzy https://github.com/ROBOTIS-GIT/robotis_applications.git
cd robotis_applications
```

**Option 2**: clone the repository on the `USER PC` instead:

```bash
cd ~/
git clone -b jazzy https://github.com/ROBOTIS-GIT/robotis_applications.git
cd robotis_applications
```

### 2. Start the Docker container

```bash
cd robotis_applications/docker && ./container.sh start
```

### 3. Start VR publisher node

In the Docker environment, run the VR publisher node.

:::tabs key:robot-type
== SG2 Type
```bash
ros2 launch robotis_vuer vr.launch.py model:=sg2
```
Or use the shortcut:
```bash
vr model:=sg2
```

== SH5 Type
```bash
ros2 launch robotis_vuer vr.launch.py model:=sh5
```
Or use the shortcut:
```bash
vr model:=sh5
```
:::

### 4. Open the Vuer page (Quest browser)

Use the headset browser (or a browser on the same network, if applicable) and go to:

```text
https://{pc_ip}:8012?ws=wss://{pc_ip}:8012
```

Example when the worker PC is at sg2's Orin `192.168.6.2`:

```text
https://192.168.6.2:8012?ws=wss://192.168.6.2:8012
```

Replace `{pc_ip}` with the actual IP of the machine hosting the Vuer server.

#### Self-signed HTTPS warning

If the browser shows a certificate warning, use **Advanced** → proceed to the site (wording may vary by browser).

![Browser advanced / certificate bypass](/vr/advance_webpage.png)

![Proceed to site (unsafe)](/vr/proceed_webpage.png)

#### Enter VR

Click `Enter VR`.

![Enter VR button](/vr/enter_vr_webpage.png)

When **passthrough** is active and you see axis markers on your hands, the session is ready.

![Passthrough with hand axes](/vr/pass_meta.png)

**Notice**: if you stop the vuer server, you need to refresh the page and clicking the **Enter VR** button again.

## Motion Controller Setup

AI Worker VR teleoperation uses **Cyclo Motion Controller** as the robot motion-control layer. It receives VR references and generates the arm trajectories that the robot follows.

Before running VR teleoperation, make sure the Cyclo Motion Controller is installed and set up correctly.

You can find the installation steps in the [`cyclo_control` repository](https://github.com/ROBOTIS-GIT/cyclo_control).

For more information, see **[Cyclo Motion Controller](/ai_worker/advanced_motion_controller_ai_worker)**.

## Running Teleoperation

The following teleoperation commands are executed on the `robot PC`.
You can either connect a keyboard and mouse directly to the Nvidia Orin, or access it via SSH (see the Setup Guide for instructions).<br>

Enter the **ai_worker** Docker container with:

```bash
cd ~/ai_worker
./docker/container.sh enter
```

::::tabs key:robot-type
== SG2 Type
### 1. Bring up the robot.

```bash
ros2 launch ffw_bringup ffw_sg2_follower_ai.launch.py
```
Or use the shortcut:
```bash
ffw_sg2_follower_ai
```

### 2. Start Cyclo motion controller

After the robot has fully completed bringup and moved to its initial position, start Cyclo Motion Controller in `vr` mode:

```bash
ros2 launch cyclo_motion_controller_ros ai_worker_controller.launch.py controller_type:=vr
```

Or use the shortcut:
```bash
motion_controller controller_type:=vr
```

### 3. Activate VR publisher

For the VR node to start publishing reference poses, press and hold both squeeze buttons on the controllers. This works as a deadman switch.

![Activate VR publisher](/vr/vr_squeeze_buttons.png)

::: tip
The VR node publishes reference data only while both squeeze buttons are pressed.
:::

### 4. Activate the controller

To activate VR teleoperation, publish the `/reactivate` topic.

This topic uses the `std_msgs/msg/Bool` type.

You can activate the controller by pressing the `X` button on the left controller and the `A` button on the right controller at the same time.
![Activate VR controller](/vr/vr_a_x.png)

::: info
Alternatively, you can publish the topic directly from a terminal:

```bash
ros2 topic pub --once /reactivate std_msgs/msg/Bool "{data: true}"
```
:::

Right after the controller is activated, the system checks the difference between the detected controller poses and the robot wrist poses. If the difference is small enough, the arm controller starts after 3 seconds. After startup, the `slow start` function remains active for 5 seconds. Because of this, it is recommended to keep your arm posture as close as possible to the robot posture before activating the controller.

### 5. Pause and resume teleoperation

- To pause the demonstration, release the squeeze buttons.
- To resume, press both squeeze buttons again.

::: warning
Avoid resuming VR publishing when your hands are far from the previous pose, because the robot may move quickly. Before resuming, it is recommended to make your arm posture as close as possible to the robot posture.
:::
== SH5 Type
### 1. Bring up the robot.

```bash
ros2 launch ffw_bringup ffw_sh5_follower_ai.launch.py
```

Or use the shortcut:
```bash
ffw_sh5_follower_ai
```

### 2. Start Cyclo motion controller

After the robot has fully completed bringup and moved to its initial position, start Cyclo Motion Controller in `vr` mode:

```bash
ros2 launch cyclo_motion_controller_ros ai_worker_controller.launch.py controller_type:=vr
```

Or use the shortcut:
```bash
motion_controller controller_type:=vr
```

### 3. Activate VR publisher and controller

For the VR node to start publishing reference poses, it must receive the `/reactivate` topic.

This topic uses the `std_msgs/msg/Bool` type.

You can publish it directly from a terminal:

```bash
ros2 topic pub --once /reactive std_msgs/msg/Bool "{data: true}"
```
::: tip
You can also build your own custom device, such as a pedal or button, for this purpose. In that case, your device should run a node that publishes to `/reactivate` when a specific input behavior is detected, such as a pedal press or button press.
:::

::: tip
As with VR publisher activation, you can also build your own custom device to publish this topic.
:::

Right after the controller is activated, the system checks the difference between the detected hand poses and the robot wrist poses. If the difference is small enough, the arm controller starts after 3 seconds. After startup, the `slow start` function remains active for 5 seconds. Because of this, it is recommended to make your arm posture as close as possible to the robot posture before activating the controller.

### 4. Pause and resume teleoperation

- To pause the demonstration, publish `/reactivate` with topic data set to false
- To resume, repeat `step 3`
::::

## Troubleshooting
1. If ROS communication is not working: check the ROS_DOMAIN_ID. (ROS_DOMAIN_ID is set to 30 in the container.)
2. If the Vuer server is not running: check the logs in the terminal.
3. If value updates are slow: check your Wi-Fi connection. Network performance has a major effect. A wired connection is recommended.
4. If the controller does not start moving after VR publishing is enabled: make sure `/reactivate` was called successfully and confirm that the detected hand poses are close enough to the robot wrist poses.
