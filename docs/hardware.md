# Hardware

## Product models

![model_lineup](./assets/model_lineup.png)
![model_name](./assets/model_name.png)

## Follower specification
| Properties                  | Specification                           |
|-----------------------------|-----------------------------------------|
| Dimensions                  | 600 x 600 x 1,600 (WxDxH, mm)<br>23.6 x 23.6 x 63.0 (WxDxH, inches)|
| Weight                      | 85 kg (190 lbs)                         |
| Actuator                    | Joint 1, 2, 3, 4, 5: DYNAMIXEL-Y<br>Joint 6: DYNAMIXEL-P<br> Joint 7: DYNAMIXEL-X|
| Degrees of Freedom          | - Total: 25 DOF<br>- Arm: 7 DOF x 2<br>- Gripper: 1 DOF x 2<br>- Head: 2 DOF x 1<br>- Lift: 1 DOF x 1<br>- Mobile: 6 DOF |
| Arm Reach                   | 647 mm (without Hand)                   |
| Arm Payload                 | 1.5 kg (without Hand)                   |
| Joint Resolution            | -π(rad) ~ π(rad), -262,144 ~ 262,144 (pulse/rev)|
| Joint Range                 | Joint 1, 2 : ±xxx°<br>Joint 3 : ±xxx°<br>Joint 4, 5, 6, 7 : ±xxx°|
| Gripper                     | RH-P12-RN                               |
| Mobile Type                 | Swerve Drive                            |
| Mobile Operation Velocity   | 1.5 m/s                                 |
| Battery Capacity            | 25.48V / 80Ah                           |
| Operating time              | Up to 4 hours                           |
| Ambient Operating Temperature | 0 ~ 40 ℃                              |
| Exterior Materials          | Aluminum, Plastic                      |
| Sensor                      | RGBD Camera x 3, LiDAR x 2, IMU         |
| Host Interface | Ethernet |
| Internal Communications | RS485 |
| Communication Baudrate | 4 [Mbps] |
| Computer                    | NVIDIA Jetson Orin                      |
| Software                    | ROS 2 Support, Python, C++ API, Web UI  |

## Leader specification
| Properties                  | Specification                           |
|-----------------------------|-----------------------------------------|
| Dimensions                  | 0 x 0 x 0 (WxDxH, mm)<br>0 x 0 x 0 (WxDxH, inches)|
| Weight                      | 0 kg (0 lbs)                         |
| Actuator                    | Joint 1~7: DYNAMIXEL-X|
| Degrees of Freedom          | - Total: 25 DOF<br>- Arm: 7 DOF x 2<br>- Gripper: 1 DOF x 2<br>- Head: 2 DOF x 1<br>- Lift: 1 DOF x 1<br>- Mobile: 6 DOF |
| Exterior Materials          | Aluminum, Plastic                      |
| Software                    | ROS 2 Support, Python, C++ API, Web UI  |

## Dimension

(TBD)

## Workspace

(TBD)
