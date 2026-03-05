# Wizard Control

This guide explains how to use **DYNAMIXEL Wizard 2.0** to connect, monitor, and control the ROBOTIS Hand.

## Prerequisites

Before using DYNAMIXEL Wizard 2.0, make sure:
- The ROBOTIS Hand is properly set up and powered on
- The ROBOTIS Hand is connected to the PC via USB

## Connecting to DYNAMIXEL Wizard 2.0

DYNAMIXEL Wizard 2.0 is a configuration and management tool for DYNAMIXEL actuators. Follow the steps below to connect the ROBOTIS Hand.

### 1. Install DYNAMIXEL Wizard 2.0

Download and install DYNAMIXEL Wizard 2.0 from the official ROBOTIS website:
- [DYNAMIXEL Wizard 2.0 Download](https://www.robotis.com/service/download.php?no=1670)

### 2. Connect the Hardware

Ensure the following hardware connections are complete:
1. Connect the **U2D2** to your PC via USB
2. Connect the **U2D2 Power Hub** to the U2D2
3. Connect the **ROBOTIS Hand** to the U2D2 Power Hub using 4-pin robot cables
4. Supply power through the **SMPS** (24V recommended)

### 3. Scan for Connected Devices

1. Launch DYNAMIXEL Wizard 2.0
2. Select the appropriate **COM port** (or `/dev/ttyUSB0` on Linux)
3. Set the **baud rate** to match the ROBOTIS Hand configuration
4. Click **Scan** to detect all connected DYNAMIXEL actuators

<!-- TODO: Add screenshot of the scan result -->

> Once the scan is complete, all connected actuators will be listed in the device panel on the left side of the application.

---

## Control Monitoring

DYNAMIXEL Wizard 2.0 provides real-time monitoring capabilities for the ROBOTIS Hand actuators. This allows you to observe the current state of each joint and verify that the hand is operating correctly.

### Monitoring Features

<img src="/wizard_control/hands/wizard_monitoring_features.png" width="1000"/>

| Feature | Description |
| --- | --- |
| **Position** | Displays the current position value of each actuator in real time |
| **Velocity** | Shows the current velocity of each actuator |
| **Current** | Monitors the current (torque) load on each actuator |
| **Temperature** | Displays the internal temperature of each actuator |
| **Voltage** | Shows the input voltage being supplied to each actuator |

### How to Use Control Monitoring

1. After scanning, select the actuator(s) you want to monitor from the device list
2. Navigate to the **Control Table** panel
3. Observe the real-time values for position, velocity, current, temperature, and voltage
4. Use the **Graph** feature to visualize data over time for more detailed analysis

<!-- TODO: Add screenshot of the monitoring interface -->

> **Tip**: You can monitor multiple actuators simultaneously to get a comprehensive overview of the entire hand state.

---

## Preset Control via Tools Menu

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

<!-- TODO: Add details on the specific parameters saved in each preset -->

### Loading and Executing Presets

Once presets are saved, you can instantly apply them to control the ROBOTIS Hand:

1. Open the preset configuration window via **Tools**
2. Select the desired preset (**Preset 1**, **Preset 2**, or **Preset 3**)
3. Click **Execute** (or **Load**) to apply the saved joint positions to the hand

The ROBOTIS Hand will move to the saved configuration automatically.

<!-- TODO: Add screenshot of preset execution -->

> **Note**: Ensure the hand is in a safe position before executing a preset to avoid unexpected collisions or movements.

### Example Use Cases

| Preset | Example Use Case |
| --- | --- |
| **Preset 1** | Fully open hand (relaxed position) |
| **Preset 2** | Power grasp (closed fist) |
| **Preset 3** | Pinch grip (thumb and index finger) |

> These are example configurations. You can customize each preset based on your specific application requirements.
