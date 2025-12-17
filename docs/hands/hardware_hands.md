# ROBOTIS HX Hand

## Specifications - Robot Hands

High-DOF multi-finger robot hands designed for AI Worker / OMY platforms.

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
| Maximum Payload           | 15 kg                                           |
| Max Finger Tip Force      | 14 N                                            |
| Peak Current              | 2.6 A                                           |
| Finger Actuator           | XM335-T323-T                                    |
| Operating Temperature     | -5 ~ 55 ℃                                       |
| Command Signal            | Digital Packet                                  |
| Protocol Type             | RS485 Asynchronous Serial<br />(8bit, 1 stop, No Parity) |
| Physical Connection       | RS485 Multidrop Bus                             |
| Standby Current           | 370 mA                                          |
| Feedback                  | Position, Velocity, Current, Temperature, Input Voltage, etc. |

### Control & ID Mapping (Temporary)

#### Communication & Control Overview

| Category          | Specification                            |
| ----------------- | ---------------------------------------- |
| Control Framework | ROS 2 + `ros2_control`                   |
| Bus Type          | RS485 multidrop                          |
| Default Baudrate  | 4 Mbps                                   |
| Control Mode      | Direct torque / position-level commands* |
| Command Format    | DYNAMIXEL-style digital packets          |

> \* Exact low-level control mode and interfaces will be finalized with the ROS 2 drivers.

#### Device ID Assignment

| Side        | Device          | ID Range      |
| ----------- |-----------------| ------------- |
| **Right**   | Hand Controller | 110           |
|             | Actuators       | 111 ~ 135     |
| **Left**    | Hand Controller | 140           |
|             | Actuators       | 141 ~ 165     |

More detailed hardware specifications (exact joint naming, connector pinouts, and mechanical drawings) will be added here later.
