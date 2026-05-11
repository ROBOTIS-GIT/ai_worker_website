# VR Teleoperation

This guide explains how to set up and operate the `AI Worker` using a VR device. Currently, only the **Meta Quest 3** is supported.

VR Teleoperation is compatible with all `AI Worker` models.

- Gripper Models: `BG2`, `SG2`
- Hand Models: `BH5`, `SH5`

For more information on the models, please refer to **[Product Models](/ai_worker/hardware_ai_worker#product-models)**.

The VR setup process is identical for all models unless otherwise specified.

::: warning
Modify **initial pose** for safety.

The default initial pose for the AI Worker (arms straight down) is not ideal for VR, as VR teleoperation requires the hands to be within the camera's field of view. It is highly recommended to change the initial position to a "ready" pose (e.g., elbows bent 90° with hands forward). Refer to this **[Discord post](https://discord.com/channels/1377230275393884170/1486960366700204204)** for instructions on modifying the initial pose (a Discord account may be required).
:::

## VR Device Setup

### 1. Meta Horizon App (Developer Mode)

Enable Developer Mode via the Meta Horizon mobile app:

**Menu → Device management → Devices → Headset settings → Developer Mode → On**

<div style="display: flex; flex-wrap: wrap; gap: 12px;">
  <img src="/vr/menu_meta.png" alt="menu_meta" width="22%" />
  <img src="/vr/headset_settings_meta.png" alt="headset_settings_meta" width="22%" />
  <img src="/vr/developer_mode_meta.png" alt="developer_mode_meta" width="22%" />
  <img src="/vr/developer_mode_toggle_meta.png" alt="developer_mode_toggle_meta" width="22%" />
</div>

### 2. On-headset settings

1. **Power / Sleep** : **Settings → General** (use **Power** or **Display** depending on your firmware): set **Display off** and **Sleep mode** to **4 hours** (or the longest option available) so the headset does not sleep during use.

![Quest power and sleep settings](/vr/power_setting_meta.png)

2. Play area : Settings → Environment setup → Boundary → Stationary boundary size → Large.

![Stationary boundary set to Large](/vr/boundary_large_meta.png)

   Alternatively, use **Create new boundary** to define the boundary manually.

## VR Startup

AI Worker VR teleoperation utilizes **Vuer** as the browser-based VR client. For more information about the VR stack and Vuer itself, see the **[ROBOTIS Vuer](https://github.com/ROBOTIS-GIT/robotis_applications/tree/main/robotis_vuer)** package.

### 1. Clone the repository

Clone the repository on **either** the `Robot PC` or a `USER PC`. The default setup uses the `Robot PC`.

:::tabs key:PC-type
== Option 1: Robot PC
Replace `SNPR48A0000` with the serial number printed on the back of the robot body.
```bash
ssh robotis@ffw-SNPR48A0000.local
```

```bash
cd ~/
git clone -b jazzy https://github.com/ROBOTIS-GIT/robotis_applications.git
cd robotis_applications
```

== Option 2: USER PC
```bash
cd ~/
git clone -b jazzy https://github.com/ROBOTIS-GIT/robotis_applications.git
cd robotis_applications
```
:::

### 2. Start and Enter the Docker Container

```bash
cd docker

# Start container
./container.sh start

# Enter running container
./container.sh enter
```

### 3. Start VR Publisher Node

In the Docker environment, launch the VR publisher node corresponding to your model.

:::tabs key:robot-type
== Gripper Type
Note that `sg2` model is used as an example.
```bash
ros2 launch robotis_vuer vr.launch.py model:=sg2
```
Or use the shortcut:
```bash
vr model:=sg2
```

== Hand Type
Note that `sh5` model is used as an example.
```bash
ros2 launch robotis_vuer vr.launch.py model:=sh5
```
Or use the shortcut:
```bash
vr model:=sh5
```
:::

### 4. Open the Vuer Page (Quest Browser)

Open the Meta Quest browser (or a browser on the same network, if applicable) and go to:

```text
https://{pc_ip}:8012?ws=wss://{pc_ip}:8012
```

Replace `{pc_ip}` with the actual IP of the machine hosting the Vuer server.

Example: The AI Worker PC (`SG2`’s Orin) has the IP address `192.168.6.2`.

```text
https://192.168.6.2:8012?ws=wss://192.168.6.2:8012
```

The PC on `BG2` should be connected to a router and use a corresponding IP address within that network.

#### Self-signed HTTPS warning

If the browser shows a certificate warning, use **Advanced** → proceed to the site (wording may vary by browser).

![Browser advanced / certificate bypass](/vr/advance_webpage.png)

![Proceed to site (unsafe)](/vr/proceed_webpage.png)

#### Enter VR

::: warning
**Positioning and Calibration**

Ensure you are at your intended operating location before proceeding. The following step initializes the VR coordinate system based on your current physical position.
:::

Click the `Enter VR` button.

![Enter VR button](/vr/enter_vr_webpage.png)

The session is ready when **passthrough** becomes active and **axis markers** appear on your hands.

![Passthrough with hand axes](/vr/pass_meta.png)

Once the VR setup is complete, **hang the headset around your neck** and tighten the strap until the device is stable and secure.

::: warning
If the Vuer server is **restarted**, you must **refresh** the browser page and click `Enter VR` again.
:::

## Cyclo Control Setup

VR teleoperation relies on `Cyclo Control` as the robot motion-control layer. It receives VR references and generates the arm trajectories that the robot follows. Ensure it is installed and configured before proceeding.

You can find the installation steps in the [`cyclo_control` repository](https://github.com/ROBOTIS-GIT/cyclo_control).

For more information, see **[Cyclo Control](/ai_worker/advanced_motion_controller_ai_worker)**.

## Running VR Teleoperation

Execute these commands on the `Robot PC`. You can either connect a keyboard and mouse directly to the Nvidia Orin, or access it via SSH (see the **[Setup Guide](/ai_worker/setup_guide_software_ai_worker)** for instructions). Enter the `ai_worker` Docker container first:

```bash
cd ~/ai_worker
./docker/container.sh enter
```

::::tabs key:robot-type
== Gripper Type
Note that `sg2` model is used here as an example.
If you are using a `bg2`, use the `sg2` launch commands in this section.
### 1. Bring up the robot.

```bash
ros2 launch ffw_bringup ffw_sg2_follower_ai.launch.py
```
Or use the shortcut:
```bash
ffw_sg2_follower_ai
```

### 2. Start Cyclo Motion Controller

Once the robot reaches its initial position, start Cyclo Motion Controller with `controller_type` parameter set as `vr`:

```bash
ros2 launch cyclo_motion_controller_ros ai_worker_controller.launch.py controller_type:=vr
```

Or use shortcut:
```bash
motion_controller controller_type:=vr
```

### 3. Activate VR Publisher

The VR node only publishes reference data while **both squeeze (grip) buttons** on the controllers are held down. This acts as a safety deadman switch.

![Activate VR publisher](/vr/vr_squeeze_buttons.png)

### 4. Activate the Controller

::: warning
**Pre-Activation Alignment**: Align your physical arms with the robot's current pose before activating to prevent sudden movements. Maintain a safe distance from the robot.
:::

You can activate the controller by pressing the `X` button on the left controller and the `A` button on the right controller at the same time.

![Activate VR controller](/vr/vr_a_x.png)

::: info
Alternatively, you can publish the topic directly from a terminal:

To activate VR teleoperation, publish the `/reactivate` topic.

This topic uses the `std_msgs/msg/Bool` type.

```bash
ros2 topic pub /reactivate std_msgs/msg/Bool "{data: true}"
```
:::

Right after the controller is activated, the system checks the difference between the detected controller poses and the robot wrist poses. If the difference is small enough, the arm controller starts after 3 seconds. After startup, the `slow start` function remains active for 5 seconds. Because of this, it is recommended to keep your arm posture as close as possible to the robot posture before activating the controller.

![SG2 Slow Start](/quick_start_guide/ai_worker/sg2_vr_slow_start.gif)

### 5. Pause and resume teleoperation

- Release the squeeze buttons.
- To resume, repeat step 3 to 4.

::: warning
Avoid resuming VR publishing when your hands are far from the previous pose, because the robot may move quickly. Before resuming, it is recommended to make your arm posture as close as possible to the robot posture.
:::

== Hand Type
Note that `sh5` model is used here as an example.
If you are using a `bh5`, use the `sh5` launch commands in this section.
### 1. Bring up the robot.

```bash
ros2 launch ffw_bringup ffw_sh5_follower_ai.launch.py
```

Or use the shortcut:
```bash
ffw_sh5_follower_ai
```

### 2. Start Cyclo Motion Controller

Once the robot has completed its bringup sequence and reached the initial position, launch the **Cyclo Motion Controller** in `vr` mode with the `hand` parameter enabled:

```bash
ros2 launch cyclo_motion_controller_ros ai_worker_controller.launch.py controller_type:=vr hand:=true
```

Or use the shortcut:
```bash
motion_controller controller_type:=vr hand:=true
```

### 3. Activate VR Publisher and Controller

::: warning
**Pre-Activation Alignment**: Align your physical arms with the robot's current pose before activating to prevent sudden movements. Maintain a safe distance from the robot.
:::

For `sh5`, VR teleoperation starts and stops with a hand gesture instead of a ROS topic or an external device.

Make **one hand squeeze** and **the other hand pinch**, then hold that combination for **3 seconds** to toggle the VR controller on or off.

Right after the controller is activated, the system checks the difference between the detected hand poses and the robot wrist poses. If the difference is small enough, the arm controller will start after 3 seconds. After the startup, the `slow start` function remains active for 5 seconds. Because of this, it is recommended to make your arm posture as close as possible to the robot posture before activating the controller.

![SH5 Slow Start](/quick_start_guide/ai_worker/sh5_vr_slow_start.gif)

### 4. Pause and resume teleoperation

- To stop teleoperation, make **one hand squeeze** and **the other hand pinch**, then hold for **3 seconds**.
- To start again, repeat `Step 3`.

::::

## Troubleshooting
### Communication Issues
- If ROS communication is not working: check the `ROS_DOMAIN_ID`. (`ROS_DOMAIN_ID` is set to `30` by default within the provided Docker container.)
- If the Vuer server is not running: check the terminal logs.
- If the robot remains stationary after the gesture trigger: confirm that you held **one-hand squeeze + one-hand pinch** for **3 seconds**, and make sure your physical hand pose is sufficiently aligned with the robot's wrist pose.

### Network Performance
- If value updates are slow: check your Wi-Fi connection. Network performance has a major effect. A **wired connection** is recommended.
- If wireless performance is insufficient, host the VR server directly on the `Robot PC` and use a USB-C to Ethernet adapter to establish a **wired connection** for the Meta Quest 3. Ensure the Ethernet cable is connected to the LAN port of the AI Worker, not the WAN port. Refer to the [Hardware Setup Overview](/ai_worker/setup_guide_hardware_ai_worker) for port identification.

### Hardware Tips
- Proximity Sensor Workaround: The Meta Quest 3 may pause the session if it detects that the headset has been removed. Placing a small piece of **non-transparent tape** over the internal proximity sensor (located between the lenses) can help keep the session active.
