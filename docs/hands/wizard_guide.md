# DYNAMIXEL Wizard Guide

This guide explains how to use **DYNAMIXEL Wizard 2.0** to connect, monitor, and control the ROBOTIS Hand.

## Prerequisites

Before using DYNAMIXEL Wizard 2.0, make sure:
- The ROBOTIS Hand is properly set up and powered **ON**
- The ROBOTIS Hand is connected to the PC via a **U2D2**

## Connecting to DYNAMIXEL Wizard 2.0

DYNAMIXEL Wizard 2.0 is a configuration and management tool for DYNAMIXEL actuators. Follow the steps below to connect the ROBOTIS Hand.

### 1. Install DYNAMIXEL Wizard 2.0

Download and install DYNAMIXEL Wizard 2.0 from the official ROBOTIS e-Manual:
- [DYNAMIXEL Wizard 2.0 Download Page](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/)

### 2. Connect the Hardware

Ensure the following hardware connections are complete:
1. Connect the **U2D2** to your PC via USB
2. Connect the **U2D2 Power Hub** to the U2D2
3. Connect the **ROBOTIS Hand** to the U2D2 Power Hub using 4-pin robot cables
4. Supply power to the U2D2 Power Hub (24V recommended)

### 3. Scan for Connected Devices

1. Launch **DYNAMIXEL Wizard 2.0**
2. Click the `Options` button in the top menu bar

<img src='/specifications/hand/wizard_option_scan_button.png'>

3. Select **Protocol 2.0**
4. Select the appropriate **COM port** (or `/dev/ttyUSB0` on Linux)
5. Set the **Baudrate** to match the ROBOTIS Hand configuration

<img src='/specifications/hand/wizard_connection_setting.png'>

6. Click `Scan` to detect all connected actuators and sensors in the side panel

> Once the scan is complete, all connected actuators will be listed in the device panel on the left side of the application.


### Control Monitoring

- The ROBOTIS Hand HX5-D20 features five fingers, each equipped with four actuators and one tactile sensor.
You can easily monitor the real-time status of every component.

<img src='/specifications/hand/wizard_hx5_d20_scan_list.png'>

## HX Hands Tool
You can control the ROBOTIS Hand through the specialized `HX Hands` Tool.

1. In the menu bar, navigate to `Tools` and select `HX Hands`
2. Click the `Sync` button to synchronize the hand with the Wizard software
3. Choose a control mode: `Preset` or `Hand`
4. Monitor the hand's current status via the GUI

<img src='/specifications/hand/wizard_sync_hxhands.png'>

### Preset Control via Tools Menu

The **Preset** feature allows you to save and load up to **three custom configurations**. This is ideal for quickly testing and switching between predefined poses or grip patterns.

### Accessing the Preset Feature

1. Open DYNAMIXEL Wizard 2.0 and connect to the ROBOTIS Hand
2. Go to **Tools > HX Hands** and navigate to the preset configuration section

### Default Presets

| Preset | Example Use Case |
| --- | --- |
| **Preset 1** | Pinch Grip |
| **Preset 2** | 3-Finger Grasp |
| **Preset 3** | Envelope Grasp |

### Custom Preset
Click **Custom Preset** as shown in the image below.

<img src='/specifications/hand/wizard_custom_preset.png'>

You can customize the release and grasp positions as shown in the image below.
- Set a joint value for each finger
- Set **PID gain values** and **current limits** for the robot hand.

<img src='/specifications/hand/wizard_custom_preset_setting.png'>

> When using preset mode, executing the release pose before the grasp pose is recommended.

### Saving Presets

You can save up to **three presets**, each storing specific joint positions.
There are two ways to capture a pose:

#### Method A: Manual Posing
1. Select **Hand** Mode
2. Physically move the ROBOTIS Hand into the desired pose by hand
3. Select an available preset slot (**Preset 1**, **Preset 2**, or **Preset 3**)
4. Click **Save** to store the current joint configuration

#### Method B: Numerical Entry
1. Select an available preset slot (**Preset 1**, **Preset 2**, or **Preset 3**)
2. Manually type the desired joint values into the input fields for the selected preset slot
3. Click **Save** to store the current joint configuration

### Loading and Executing Presets

Once presets are saved, you can instantly apply them to control the ROBOTIS Hand:

1. Open the preset configuration window via **Tools**
2. Select the desired preset (**Preset 1**, **Preset 2**, or **Preset 3**)
3. Click **Execute** (or **Load**) to apply the saved joint positions to the hand
4. Click **Grasping** (or **Releasing**) to execute a saved preset
    - Use the hand icon next to the Preset table
    - It is safe to execute **Grasping** and **Releasing** alternately

<img src='/specifications/hand/wizard_control_image.png'>

> Ensure the hand is in a safe position before executing a preset to avoid unexpected collisions or movements.

#### Checking Tactile Sensors

The GUI provides a visual representation of tactile sensor data. Pressure levels are indicated by color changes:

- **Bright Green**: Strong pressure detected
- **Black Background**: Weak or no pressure

<img src='/specifications/hand/hx5_d20_tactile_sensor.png'>
