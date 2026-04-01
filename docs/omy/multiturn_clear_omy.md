# Multi-turn error — clearing procedure (OMY)

If you see logs like the following while running bringup, a multi-turn error has occurred.

```
[ros2_control_node-1] [ID:01] RX_PACKET_ERROR : [RxPacketError] Hardware error occurred. Check the error at Control Table (Hardware Error Status)!
[ros2_control_node-1] [comm_id:01][ID:01] Request ping	 - RX_PACKET_ERROR : [RxPacketError] Hardware error occurred. Check the error at Control Table (Hardware Error Status)!
[ros2_control_node-1] [ReadItem][ID:01][comm_id:01] RX_PACKET_ERROR : [RxPacketError] Hardware error occurred. Check the error at Control Table (Hardware Error Status)!
[ros2_control_node-1] [comm_id:01][ID:01] Error Code Details: 0xc (Multi-turn Error): An issue has occurred with the Multi-turn IC
[ros2_control_node-1] [ID:01] Rebooting...
```

<p style="color: var(--vp-c-text-2);">In this example, the multi-turn error is on Dynamixel ID 1.</p>

Use **one** of the methods below, depending on whether you use **noVNC on the robot** or a **PC connected to the debug USB port**.

---

## Method 1: noVNC and Dynamixel Wizard 2.0

### 1. Set up the noVNC container

::: details docker-compose.yml
```yaml
services:
  novnc-server:
    container_name: novnc-server
    image: robotis/novnc-server:latest
    restart: always
    cap_add:
      - SYS_NICE
    ulimits:
      rtprio: 99
      rttime: -1
      memlock: 8428281856
    network_mode: host
    environment:
      - ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-30}
      - DISPLAY=:99
      - DISPLAY_WIDTH=1920
      - DISPLAY_HEIGHT=1080
      - WEBSOCKIFY_PORT=8090
    volumes:
      - /dev:/dev
      - /dev/shm:/dev/shm
    privileged: true
```
:::

Then run `docker-compose up -d` to start the noVNC container.

### 2. Open noVNC in a browser

In a Web browser (for example Chrome), open `http://omy-snpr44b{serial-number}.local:8090`, replacing `{serial-number}` with your robot’s serial number. Example: `http://omy-snpr44b0000.local:8090`

### 3. Connect noVNC and Dynamixel Wizard 2.0

Connect **noVNC** and **Dynamixel Wizard 2.0** so you can use the wizard on the robot’s desktop.

### 4. Configure Dynamixel Wizard 2.0 and run Scan

In **Dynamixel Wizard 2.0**, set the following and run **Scan**:

- **Select protocol to scan:** Protocol 2.0  
- **Select port to scan:** `/dev/ttyAMA2`
- **Baud rate:** 6250000bps


### 5. Search for DYNAMIXEL-Y actuators

1. In the **OMY-HAT** control table, turn on **`DXL Power Enable (512)`**.

![search dynamixelY 1](/specifications/omy/search_dynamixel_y_1.png)

2. In Wizard 2.0, open **Settings → Search**. Enable the **6 Mbps** baud rate and select the port to scan `/dev/ttyAMA4`, then set the ID search range from **1** to **10**.

![search dynamixelY 2](/specifications/omy/search_dynamixel_y_2.png)

3. Click the **DYNAMIXEL search** button in Wizard 2.0.

4. Confirm that six **DYNAMIXEL-Y** actuators (IDs **1**–**6**) are detected.

![search dynamixelY 3](/specifications/omy/search_dynamixel_y_3.png)

### 6. Manually move joints toward the home position

1. Set **`Operating Mode (33)`** on the target **DYNAMIXEL-Y** to **Current**.
2. Turn **torque on**.

::: warning
The robot may move or fall under gravity. **Support the robot by hand** while torque is enabled.
:::

![OMY manual operation 1](/specifications/omy/omy_manual_operation_1.png)

3. Move the joint toward the marked home position.
4. Turn **torque off**.
5. Repeat substeps 1–4 for each axis until the arm is at the home or packing position.

### 7. Clear the multi-turn error (Tools menu)

1. Select the Dynamixel that reported the error.
2. In the top menu, choose **Tools → Encoder battery change**.

![Encoder battery change](/troubleshooting_guide/tools_clear.png)

3. Set **Multi-turn encoder power mode** to **High**, then run **Clear**.

![Multi-turn encoder power mode](/troubleshooting_guide/multi_turn_encoder_power_mode.png)

### 8. Initialize the home position

1. Move the robot to the home position.
2. In Wizard 2.0, open the **Packet Monitoring** window, open the COM port, and send the **`Clear`** command to IDs **1** through **6**.

![Initialize home position — packet monitoring](/specifications/omy/reset_multiturn_1.png)

3. Check that each joint’s present position is within **1** degrees of the target.
4. For any joint outside that range, return to **step 6**, adjust the joint manually, then repeat step 2.

![Initialize home position reference](/specifications/omy/initialize_home_position.png)

### 9. OMY home position (slit alignment)

Align the robot to the home position using the **home-position slits** as shown.

![OMY home position slit](/specifications/omy/home_position_slit_omy.png)

### 10. Verify with bringup

Run bringup again and confirm the multi-turn error no longer appears.

---

## Method 2: Debug USB port and Dynamixel Wizard 2.0

### 1. Prepare a PC with Dynamixel Wizard 2.0

Install [Dynamixel Wizard 2.0](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/) on a PC or laptop.

### 2. Connect the debug USB port

1. Connect the **Debug USB-C** port on the main body panel to the PC with a USB-C cable.
2. Power on OMY.
3. On the PC, confirm the serial port appears (Windows: `COM*`, Ubuntu: `/dev/ttyACM*`).

![Connect OMY to Wizard](/specifications/omy/connect_omy_wizard.png)

### 3. Search for OMY (OMY-HAT)

1. In Wizard 2.0, open **Settings** and select the COM port the PC recognized.

![Search OMY on Wizard 1](/specifications/omy/search_omy_on_wizard_1.png)

2. Click **DYNAMIXEL search**.

3. Confirm **OMY-HAT (ID 200)** is found.

![Search OMY on Wizard 2](/specifications/omy/search_omy_on_wizard_2.png)

### 4. Search for DYNAMIXEL-Y actuators

1. In the **OMY-HAT** control table, turn on **`DXL Power Enable (512)`**.

![search dynamixelY 1 — USB path](/specifications/omy/search_dynamixel_y_1.png)

2. In **Settings → Search**, enable **6 Mbps** and set the ID range from **1** to **10**.

![search dynamixelY 2 — USB path](/specifications/omy/search_dynamixel_y_2.png)

3. Click **DYNAMIXEL search**.

4. Confirm six **DYNAMIXEL-Y** actuators (IDs **1**–**6**) are detected.

![search dynamixelY 3 — USB path](/specifications/omy/search_dynamixel_y_3.png)

### 5. Reset the DYNAMIXEL-Y multi-turn alarm

1. Note the **ID** of the **DYNAMIXEL-Y** that reported the alarm.
2. In **Packet Monitoring**, open the COM port and send the **`Clear`** command to that actuator.

![Reset multi-turn alarm 1](/specifications/omy/reset_multiturn_1.png)

3. Confirm the multi-turn alarm is cleared.

![Reset multi-turn alarm 2](/specifications/omy/reset_multiturn_2.png)

### 6. Manually move joints toward the home position

1. Set **`Operating Mode (33)`** on the target **DYNAMIXEL-Y** to **Current**.
2. Turn **torque on**.

::: warning
The robot may move or fall under gravity. **Support the robot by hand** while torque is enabled.
:::

![OMY manual operation 1 — USB path](/specifications/omy/omy_manual_operation_1.png)

3. Move the joint toward the marked home position.
4. Turn **torque off**.
5. Repeat substeps 1–4 for each axis until the arm is at the home or packing position.

![OMY manual operation 2](/specifications/omy/omy_manual_operation_2.png)

### 7. Initialize the home position

1. Move the robot to the home position.
2. In **Packet Monitoring**, open the COM port and send **`Clear`** to IDs **1** through **6**.

![Initialize home position — USB path](/specifications/omy/reset_multiturn_1.png)

3. Check that each joint is within **1** degree.
4. If not, return to **step 6**, adjust manually, then repeat step 2.

![Initialize home position reference — USB path](/specifications/omy/initialize_home_position.png)

### 8. OMY home position (slit alignment)

Align the robot using the **home-position slits**.

![OMY home position slit — USB path](/specifications/omy/home_position_slit_omy.png)

### 9. Verify with bringup

Run bringup again and confirm the error no longer appears.
