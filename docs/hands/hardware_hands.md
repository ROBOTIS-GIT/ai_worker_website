# ROBOTIS Hand

<img src="/hx5_d20_rev2.png" width="400"/>

## Product Models
<img src="/specifications/hand/hand_lineup_R.png"/>

## Specifications

High-DOF multi-finger robot hands designed for AI Worker / OMY platforms.

:::tabs key:robot-type
== HX5-D20
### Main Specifications

| Item                      | Specification                                   |
| ------------------------- | ----------------------------------------------- |
| Number of Fingers         | 5                                               |
| Degrees of Freedom        | 20 (4 DoF / 1 Finger)                           |
| Baud Rate                 | 9,600 bps ~ 6 Mbps (default: **4 Mbps**)        |
| Control Frequency         | 1 kHz                                           |
| Weight                    | 1,000 g ± 2%                                    |
| Operating Voltage         | 24 V                                            |
| TIP Sensor                | 9-array sensor, 0 ~ 255 value                   |
| Operating Mode            | Direct Control Mode  <br> Preset Motion Mode    |
| Maximum Load for Envelop  | 15 kg                                           |
| Max Finger Tip Force      | 14 N                                            |
| Peak Current              | 2.6 A                                           |
| Finger Actuator           | XM335-T323-T                                    |
| Operating Temperature     | -5 ~ 55 ℃                                       |
| Command Signal            | Digital Packet                                  |
| Protocol Type             | RS485 Asynchronous Serial<br />(8bit, 1 stop, No Parity) |
| Physical Connection       | RS485 Multidrop Bus                             |
| Standby Current           | 370 mA                                          |
| Feedback                  | Position, Velocity, Current, Temperature, Input Voltage, etc. |

## Control & ID Mapping

### Communication & Control Overview

| Category          | Specification                            |
| ----------------- | ---------------------------------------- |
| Control Framework | ROS 2 + `ros2_control`                   |
| Bus Type          | RS485 multidrop                          |
| Default Baudrate  | 4 Mbps                                   |
| Control Mode      | Direct torque / position-level commands* |
| Command Format    | DYNAMIXEL Protocol 2.0, Modbus-RTU       |

> \* Exact low-level control mode and interfaces will be finalized with the ROS 2 drivers.

### Device ID Assignment

| Side        | Device          | ID Range      |
| ----------- |-----------------| ------------- |
| **Right**   | Hand Controller | 110           |
|             | Actuators       | 111 ~ 135     |
| **Left**    | Hand Controller | 140           |
|             | Actuators       | 141 ~ 165     |

<!-- More detailed hardware specifications (exact joint naming, connector pinouts, and mechanical drawings) will be added here later. -->
### Joint Configuration and Nomenclature

<a href="/specifications/hand/hand_joints_R.png" target="_blank">
  <img src="/specifications/hand/hand_joints_R.png" alt="Dimensions" width="100%">
</a>

#### Right Hand
| ID  | Joint Name            | Technical Name   | Range       |
|-----|-----------------------|------------------|-------------|
| 111 | Right Thumb CMC       | finger_r_joint1  |  -90° ~ 90° |
| 112 | Right Thumb MCP Yaw   | finger_r_joint2  | -180° ~ 0°  |
| 113 | Right Thumb MCP Pitch | finger_r_joint3  |    0° ~ 90° |
| 114 | Right Thumb IP        | finger_r_joint4  |    0° ~ 90° |
| 116 | Right Index MCP Roll  | finger_r_joint5  |  -35° ~ 35° |
| 117 | Right Index MCP Pitch | finger_r_joint6  |    0° ~ 115°|
| 118 | Right Index PIP       | finger_r_joint7  |    0° ~ 90° |
| 119 | Right Index DIP       | finger_r_joint8  |    0° ~ 90° |
| 121 | Right Middle MCP Roll | finger_r_joint9  |  -35° ~ 35° |
| 122 | Right Middle MCP Pitch| finger_r_joint10 |    0° ~ 115°|
| 123 | Right Middle PIP      | finger_r_joint11 |    0° ~ 90° |
| 124 | Right Middle DIP      | finger_r_joint12 |    0° ~ 90° |
| 126 | Right Ring MCP Roll    | finger_r_joint13 |  -35° ~ 35° |
| 127 | Right Ring MCP Pitch  | finger_r_joint14 |    0° ~ 115°|
| 128 | Right Ring PIP        | finger_r_joint15 |    0° ~ 90° |
| 129 | Right Ring DIP        | finger_r_joint16 |    0° ~ 90° |
| 131 | Right Pinky MCP Roll   | finger_r_joint17 |  -35° ~ 35° |
| 132 | Right Pinky MCP Pitch | finger_r_joint18 |    0° ~ 115°|
| 133 | Right Pinky PIP       | finger_r_joint19 |    0° ~ 90° |
| 134 | Right Pinky DIP       | finger_r_joint20 |    0° ~ 90° |

<a href="/specifications/hand/hand_joints_L.png" target="_blank">
  <img src="/specifications/hand/hand_joints_L.png" alt="Dimensions" width="100%">
</a>

#### Left Hand
| ID  | Joint Name            | Technical Name   | Range       |
|-----|-----------------------|------------------|-------------|
| 141 |  Left Thumb CMC       | finger_l_joint1  |  -90° ~ 90° |
| 142 |  Left Thumb MCP Yaw   | finger_l_joint2  |    0° ~ 180°|
| 143 |  Left Thumb MCP Pitch | finger_l_joint3  |  -90° ~ 0°  |
| 144 |  Left Thumb IP        | finger_l_joint4  |  -90° ~ 0°  |
| 146 |  Left Index MCP Roll  | finger_l_joint5  |  -35° ~ 35° |
| 147 |  Left Index MCP Pitch | finger_l_joint6  |    0° ~ 115°|
| 148 |  Left Index PIP       | finger_l_joint7  |    0° ~ 90° |
| 149 |  Left Index DIP       | finger_l_joint8  |    0° ~ 90° |
| 151 |  Left Middle MCP Roll | finger_l_joint9  |  -35° ~ 35° |
| 152 |  Left Middle MCP Pitch| finger_l_joint10 |    0° ~ 115°|
| 153 |  Left Middle PIP      | finger_l_joint11 |    0° ~ 90° |
| 154 |  Left Middle DIP      | finger_l_joint12 |    0° ~ 90° |
| 156 |  Left Ring MCP Roll   | finger_l_joint13 |  -35° ~ 35° |
| 157 |  Left Ring MCP Pitch  | finger_l_joint14 |    0° ~ 115°|
| 158 |  Left Ring PIP        | finger_l_joint15 |    0° ~ 90° |
| 159 |  Left Ring DIP        | finger_l_joint16 |    0° ~ 90° |
| 161 |  Left Pinky MCP Roll  | finger_l_joint17 |  -35° ~ 35° |
| 162 |  Left Pinky MCP Pitch | finger_l_joint18 |    0° ~ 115°|
| 163 |  Left Pinky PIP       | finger_l_joint19 |    0° ~ 90° |
| 164 |  Left Pinky DIP       | finger_l_joint20 |    0° ~ 90° |
