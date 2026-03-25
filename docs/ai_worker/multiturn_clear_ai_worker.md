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
Press the E-stop, align the slit marks while moving the arm to the **home (zero) position**, then release the E-stop *once alignment is complete*.
  Home positions (joint index, Dynamixel ID): These are left arm images. Please apply the same to the right arm as well.  
      * **Joint 1** (ID 1, 31)
          <img src="/troubleshooting_guide/ai_worker_31.jpg" alt="Joint 1, Dynamixel ID 31, home position reference" style="max-width: 360px; width: 100%; height: auto; display: block;" />
      * **Joint 2** (ID 2, 32)
          <img src="/troubleshooting_guide/ai_worker_32.jpg" alt="Joint 2, Dynamixel ID 32, home position reference" style="max-width: 360px; width: 100%; height: auto; display: block;" />
      * **Joint 3** (ID 3, 33)
          <img src="/troubleshooting_guide/ai_worker_33.jpg" alt="Joint 3, Dynamixel ID 33, home position reference" style="max-width: 360px; width: 100%; height: auto; display: block;" />
      * **Joint 4** (ID 4, 34)
          <img src="/troubleshooting_guide/ai_worker_34.jpg" alt="Joint 4, Dynamixel ID 34, home position reference" style="max-width: 360px; width: 100%; height: auto; display: block;" />
      * **Joint 5** (ID 5, 35)
          <img src="/troubleshooting_guide/ai_worker_35.jpg" alt="Joint 5, Dynamixel ID 35, home position reference" style="max-width: 360px; width: 100%; height: auto; display: block;" />
      * **Joint 6** (ID 6, 36)
          <img src="/troubleshooting_guide/ai_worker_36.jpg" alt="Joint 6, Dynamixel ID 36, home position reference" style="max-width: 360px; width: 100%; height: auto; display: block;" />


### 2. Set up the noVNC container
  ::: details docker-compose.yml
  ```yml
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

### 3. Open noVNC in a browser
In Web Browser(like Chrome), open `http://ffw-snpr48a{serial-number}.local:8090`, substituting your robot’s serial number for `serial-number`. (e.g. `http://ffw-snpr48a0000.local:8090`)

### 4. Connect noVNC and Dynamixel Wizard 2.0
Connect **noVNC** and **Dynamixel Wizard 2.0** so you can use the wizard on the robot’s desktop.

### 5. Configure Dynamixel Wizard 2.0 and run Scan
In **Dynamixel Wizard 2.0**, configure the following and run **Scan**:  
    - Select protocol to scan: **Protocol 2.0**  
    - Select port to scan: `/dev/ttyUSB4`  
    - Baud rate: **4000000bps**  

### 6. Select the Dynamixel that reported the error
Select the Dynamixel that reported the error, then choose **Tools → Encoder battery change** from the top menu.
  ![encoder battery change](/public/troubleshooting_guide/tools_clear.png)

### 7. Clear the multi-turn error
Set **Multi-turn encoder power mode** to **High**, then perform **Clear**.

  ![multi turn encoder power mode](/public/troubleshooting_guide/multi_turn_encoder_power_mode.png)

### 8. Verify with bringup
Run **bringup** again and confirm the error no longer appears.
