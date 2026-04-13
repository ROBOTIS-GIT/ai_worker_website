# VR Teleoperation

This guide explains how to set up and operate the `HX5-D20` using a VR device. Currently, only the **Meta Quest 3** is supported.

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

ROBOTIS Hand VR teleoperation utilizes **Vuer** as the browser-based VR client. For more information regarding the VR stack and Vuer itself, see the **[ROBOTIS Vuer](https://github.com/ROBOTIS-GIT/robotis_applications/tree/main/robotis_vuer)** package.

### 1. Clone the repository

```bash
cd ~/
git clone -b jazzy https://github.com/ROBOTIS-GIT/robotis_applications.git
cd robotis_applications
```

### 2. Start and Enter the Docker Container

```bash
cd docker

# Start container
./container.sh start

# Enter running container
./container.sh enter
```

### 3. Start VR Publisher Node

Inside the Docker environment, launch the VR publisher node with the `model` parameter set to `hx5`.

```bash
ros2 launch robotis_vuer vr.launch.py model:=hx5
```
Alternatively, use the shortcut:
```bash
vr model:=hx5
```

### 4. Open the Vuer Page (Quest Browser)

Open the Meta Quest browser (or a browser on the same network, if applicable) and navigate to:

```text
https://{pc_ip}:8012?ws=wss://{pc_ip}:8012
```

Replace `{pc_ip}` with the **actual IP** of the machine hosting the Vuer server.

#### Self-signed HTTPS warning

If the browser shows a certificate warning, use **Advanced** → proceed to the site (exact wording may vary by browser).

![Browser advanced / certificate bypass](/vr/advance_webpage.png)

![Proceed to site (unsafe)](/vr/proceed_webpage.png)

#### Enter VR

Click the `Enter VR` button.

![Enter VR button](/vr/enter_vr_webpage.png)

The session is ready when **passthrough** becomes active and **axis markers** appear on your hands.

![Passthrough with hand axes](/vr/pass_meta.png)

VR setup is now complete.

::: warning
If the Vuer server is **restarted**, you must **refresh** the browser page and click `Enter VR` again.
:::

## Cyclo Control Setup

VR teleoperation relies on `Cyclo Control` as the robot retargeting layer. It receives VR references and generates the hand trajectories that the robot follows. Ensure it is installed and configured before proceeding.

You can find the installation steps in the [`cyclo_control` repository](https://github.com/ROBOTIS-GIT/cyclo_control).

For more information, see **[Cyclo Control](/ai_worker/advanced_motion_controller_ai_worker)**.

## Running VR Teleoperation

First, enter the `robotis_hand` Docker container:

```bash
cd ~/robotis_hand
./docker/container.sh enter
```

### 1. Bring up the robot.

Launch the bringup script for your specific hand model:

```bash
# For the left hand
ros2 launch robotis_hand_bringup hx5_d20_left.launch.py

# For the right hand
ros2 launch robotis_hand_bringup hx5_d20_right.launch.py
```

### 2. Start Cyclo Motion Controller

Once the robot has reached its initial position, open a **new terminal**, enter the Docker container, and run the retargeting teleoperation node:

```bash
ros2 run cyclo_motion_controller_ros_py retargeting_teleop
```

### 3. Activate VR Publisher

The VR node will only begin publishing reference poses upon receiving a message on the `/reactivate` topic.

::: warning
**Pre-Activation Alignment**: Physically align your hand with the current pose of the ROBOTIS Hand before activating. This prevents sudden, unintended movements upon startup.
:::

Publish the activation signal (type `std_msgs/msg/Bool`) directly from the terminal:

```bash
ros2 topic pub --once /reactivate std_msgs/msg/Bool "{data: true}"
```
::: tip
You can integrate a custom hardware trigger, such as a foot pedal or physical button. To do this, create a node that publishes to the `/reactivate` topic when your device input is detected.
:::

### 4. Pause and resume teleoperation

- To Pause: Publish to `/reactivate` with the data field set to `false`
- To Resume: Re-align your hand pose and repeat `Step 3`

## Troubleshooting
1. If ROS communication is not working: check the `ROS_DOMAIN_ID`. (`ROS_DOMAIN_ID` is set to `30` in the container.)
2. If the Vuer server is not running: check the logs in the terminal.
3. If value updates are slow: check your Wi-Fi connection. Network performance has a major effect. A wired connection is recommended.
4. If the ROBOTIS Hand does not start moving after VR publishing is enabled: make sure `/reactivate` was called successfully.
