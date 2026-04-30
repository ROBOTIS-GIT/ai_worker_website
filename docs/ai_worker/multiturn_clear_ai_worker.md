# Multi-turn error — clearing procedure (AI Worker)

If you see logs like the following while running bringup, a multi-turn error has occurred.

```
[ros2_control_node-1] [ID:034] RX_PACKET_ERROR : [RxPacketError] Hardware error occurred. Check the error at Control Table (Hardware Error Status)!
[ros2_control_node-1] [comm_id:034][ID:034] Request ping	 - RX_PACKET_ERROR : [RxPacketError] Hardware error occurred. Check the error at Control Table (Hardware Error Status)!
[ros2_control_node-1] [ReadItem][ID:034][comm_id:034] RX_PACKET_ERROR : [RxPacketError] Hardware error occurred. Check the error at Control Table (Hardware Error Status)!
[ros2_control_node-1] [comm_id:034][ID:034] Error Code Details: 0xc (Multi-turn Error): An issue has occurred with the Multi-turn IC
[ros2_control_node-1] [ID:034] Rebooting...
```

<p style="color: var(--vp-c-text-2);">In this example, the multi-turn error is on Dynamixel ID 34.</p>

Use the steps below to clear the multi-turn error.

## How to clear the multi-turn error

### 1. Align the home position

Press the **E-stop**, align the **slit marks** while moving the arm toward the **home (zero) position**, then release the E-stop *once alignment is complete*.

You only need to set the pose for the **joint(s) you plan to clear** (for example, the Dynamixel ID shown in the error log). It is not required to align every joint on the arm if the fault is isolated—but keep the arm **supported and safe** while doing so.

**Important:** When you perform the multi-turn **Clear** in Dynamixel Wizard later (**Tools → Encoder battery change**), the robot must still be in this **physically correct alignment**. The clear procedure assumes the mechanism matches the home / slit reference at that moment; if the pose is wrong, the encoder state may not clear correctly or the arm may not behave as expected afterward.

Home positions (joint index, Dynamixel ID): the figures below are **left arm** references. Use the same idea for the **right arm** if you are clearing servos on that side.

- **Joint 1** (ID 1, 31)  
  <img src="/troubleshooting_guide/ai_worker_31.jpg" alt="Joint 1, Dynamixel ID 31, home position reference" style="max-width: 360px; width: 100%; height: auto; display: block;" />
- **Joint 2** (ID 2, 32)  
  <img src="/troubleshooting_guide/ai_worker_32.jpg" alt="Joint 2, Dynamixel ID 32, home position reference" style="max-width: 360px; width: 100%; height: auto; display: block;" />
- **Joint 3** (ID 3, 33)  
  <img src="/troubleshooting_guide/ai_worker_33.jpg" alt="Joint 3, Dynamixel ID 33, home position reference" style="max-width: 360px; width: 100%; height: auto; display: block;" />
- **Joint 4** (ID 4, 34)  
  <img src="/troubleshooting_guide/ai_worker_34.jpg" alt="Joint 4, Dynamixel ID 34, home position reference" style="max-width: 360px; width: 100%; height: auto; display: block;" />
- **Joint 5** (ID 5, 35)  
  <img src="/troubleshooting_guide/ai_worker_35.jpg" alt="Joint 5, Dynamixel ID 35, home position reference" style="max-width: 360px; width: 100%; height: auto; display: block;" />
- **Joint 6** (ID 6, 36)  
  <img src="/troubleshooting_guide/ai_worker_36.jpg" alt="Joint 6, Dynamixel ID 36, home position reference" style="max-width: 360px; width: 100%; height: auto; display: block;" />


### 2. Set up and start the noVNC container

:::info
⚠️ **noVNC and Cyclo Manager**

Previously, AI Worker repository provided noVNC compose directly. After Cyclo Manager was introduced, the noVNC container is managed from Cyclo Manager instead. Starting with **AI Worker 1.2.2**, the noVNC Compose file is no longer shipped with AI Worker.
:::

#### Method 1: Install Cyclo Manager
[(How to Install Cyclo Manager)](/ai_worker/cyclo_manager_ai_worker#install).

1. In a Web Browser(like Chrome), open `http://ffw-snpr48a{serial-number}.local:3000`, substituting your robot’s serial number for `serial-number`. (e.g. `http://ffw-snpr48a0000.local:3000`)  
2. Then navigate Cyclo Manager -> AI worker -> noVNC Page and click the NOVNC Server Start button on the top and reload the page(f5).
![noVNC Desktop](/advanced_features/cyclo_manager/novnc_desktop.png)

3. Click the `Dynamixel Wizard 2.0` icon to open the Dynamixel Wizard 2.0.
![noVNC Page](/advanced_features/cyclo_manager/novnc_page.png)



#### Method 2: Make a `docker-compose.novnc.yml` file with the following content:  
  ::: details docker-compose.novnc.yml
  ```yml
  services:
    novnc-server:
      container_name: novnc-server
      image: robotis/novnc-server:latest
      restart: no
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

  Then run `docker compose -f docker/docker-compose.novnc.yml up` to start the noVNC container and browse `http://ffw-snpr48a{serial-number}.local:8090` in a web browser.
  Click the connect button and open the Dynamixel Wizard 2.0.
  ![noVNC connect](/troubleshooting_guide/novnc_connect.png)

### 3. Configure Dynamixel Wizard 2.0 and run Scan
In **Dynamixel Wizard 2.0**, configure the following and run **Scan**:  
    - Select protocol to scan: **Protocol 2.0**  
    - Select port to scan: `/dev/ttyUSB4` **(When connecting an LG2 leader, scan ports from `ttyUSB4` to `ttyUSB6`)**  
    - Baud rate: **4000000 bps**  
![wizard options](/public/specifications/ai_worker/wizard_options_ai_worker.png)
### 4. Select the Dynamixel that reported the error
Select the Dynamixel that reported the error, then choose **Tools → Encoder battery change** from the top menu.
  ![encoder battery change](/public/specifications/ai_worker/wizard_ai_worker.png)

### 5. Clear the multi-turn error
Set **Multi-turn encoder power mode** to **High**, then perform **Clear**.

  ![multi turn encoder power mode](/public/troubleshooting_guide/multi_turn_encoder_power_mode.png)

### 6. Verify with bringup
Run **bringup** again and confirm the error no longer appears.
