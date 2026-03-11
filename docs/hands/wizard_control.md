# Wizard Control

This guide explains how to use **DYNAMIXEL Wizard 2.0** to connect, monitor, and control the ROBOTIS Hand.

## Prerequisites

Before using DYNAMIXEL Wizard 2.0, make sure:
- The ROBOTIS Hand is properly set up and powered ON
- The ROBOTIS Hand is connected to the PC via USB

## Connecting to DYNAMIXEL Wizard 2.0

DYNAMIXEL Wizard 2.0 is a configuration and management tool for DYNAMIXEL actuators. Follow the steps below to connect the ROBOTIS Hand.

### 1. Install DYNAMIXEL Wizard 2.0

1. Download and install DYNAMIXEL Wizard 2.0 from the official ROBOTIS website:
- [DYNAMIXEL Wizard 2.0 Download](https://www.robotis.com/service/download.php?no=1670)

2. Move to the downloaded folder.
```bash
cd ~/Downloads
```

3. To install Wizard 2.0, change the file permission.
```bash 
sudo chmod 775 DynamixelWizard2Setup-linux-x64.run
```

4. Run the installation command in the terminal.
```bash
./DynamixelWizard2Setup-linux-x64.run
```
> You can check the details in the link below.

[DYNAMIXEL WIZARD 2.0 Docs](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/)

### 2. Connect the Hardware

Ensure the following hardware connections are complete:
1. Connect the **U2D2** to your PC via USB
2. Connect the **U2D2 Power Hub** to the U2D2
3. Connect the **ROBOTIS Hand** to the U2D2 Power Hub using 4-pin robot cables
4. Supply power through the **SMPS** (24V recommended)

### 3. Scan for Connected Devices

1. Launch DYNAMIXEL Wizard 2.0
2. Click the `Options` button in the top menu bar
<img src='/specifications/hand/option_scan_button.png'>
3. Select the appropriate **Protocol**
4. Select the appropriate **COM port** (or `/dev/ttyUSB0` on Linux)
5. Set the **baud rate** to match the ROBOTIS Hand configuration
<img src='/specifications/hand/connection_setting.png'>
6. Click `Scan` to detect all connected actuators and sensors in the side panel
<!-- TODO: Add screenshot of the scan result : Options and base image-->

> Once the scan is complete, all connected actuators will be listed in the device panel on the left side of the application.

---


### Control Monitoring

- The hand has five fingers, and each finger has four motors and one tactile sensor.
You can easily monitor each actuator and sensor.
<img src='/specifications/hand/controllers_guide.png'>


---

### HX Hands
You can control the ROBOTIS Hand with HX Hands.

1. On the menu bar, click `Tools` and then `HX Hands`
2. Click the `Sync` button to synchronize the hand and Wizard
3. Choose a mode: `Preset` or `Hand`
4. You can check the hand's current status in the GUI.

<img src='/specifications/hand/sync_hxhands.png'>

<!-- put the explanations how to access the hands tools and note that have to sync with the hands -->


### Preset Control via Tools Menu

DYNAMIXEL Wizard 2.0 provides a **Preset** feature through the **Tools** menu, allowing you to save and load up to **3 custom presets** for hand control configurations. This is useful for quickly switching between predefined hand poses or grip patterns.


### Accessing the Preset Feature

1. Open DYNAMIXEL Wizard 2.0 and connect to the ROBOTIS Hand
2. Navigate to **Tools** in the top menu bar
3. Select the preset configuration window

<!-- TODO: Add screenshot of the Tools menu and preset window -->

### Saving Presets

You can save up to **3 presets**, each storing a specific set of joint positions for the ROBOTIS Hand:

1. Manually adjust the actuator positions to the desired hand pose using the Control Table
2. Open the preset configuration window via **Tools**
3. Select an available preset slot (**Preset 1**, **Preset 2**, or **Preset 3**)
4. Click **Save** to store the current joint configuration


### Loading and Executing Presets

Once presets are saved, you can instantly apply them to control the ROBOTIS Hand:

1. Open the preset configuration window via **Tools**
2. Select the desired preset (**Preset 1**, **Preset 2**, or **Preset 3**)
3. Click **Execute** (or **Load**) to apply the saved joint positions to the hand
4. Click **Grasping** (or **Releasing**) to execute a saved preset
    - Use the hand icon next to the Preset table
    - It is safe to execute **Grasping** and **Releasing** alternately

<img src='/specifications/hand/wizard_control_image.png'>


<!-- TODO: Add screenshot of preset execution : HX Hands-->

> **Note**: Ensure the hand is in a safe position before executing a preset to avoid unexpected collisions or movements.

### Example Use Cases

| Preset | Example Use Case |
| --- | --- |
| **Preset 1** | Pinch Grip |
| **Preset 2** | Tripod Grip |
| **Preset 3** | Full Grip |
<!-- TODO: Add details on the specific parameters saved in each preset make the table which can be open and close by the buttons -->

### Custom Preset
Click **Custom Preset** as shown in the image below.

<img src='/specifications/hand/custom_preset.png'>

You can customize the release and grasp positions as shown in the image below.
- Set a joint value for each finger
- Set gain value and current limit for the robot hand.

<img src='/specifications/hand/custom_preset_setting.png'>

> When using preset mode, executing the release pose before the grasp pose is recommended.

#### Checking Tactile Sensors

In the Wizard program, you can easily check whether the tactile sensors are working in the GUI. The color changes indicate pressure levels: **Bright Green** means strong pressure, and a **Black Background** means weak pressure.
<img src='/specifications/hand/check_tactile_sensor.png'>
