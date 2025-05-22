# Hardware

## Hardware Overview
The AI Worker hardware platform combines advanced robotics with AI-powered capabilities to create a versatile semi-humanoid robot. Built around ROBOTIS [DYNAMIXEL](https://dynamixel.com/) actuators, it features a mobile base with swerve drive, dual 7-DOF arms with dexterous hand/grippers, and a sensor suite that includes RGBD cameras and LiDARs. The system is powered by an NVIDIA Jetson AGX Orin computer and offers up to 4 hours of operation time on a single charge. This comprehensive hardware design enables the AI Worker to perform complex manipulation tasks through imitation learning and reinforcement learning in various industrial environments.

![hardware_overview](./assets/hardware_overview.png)

## Product Models
![model_name](./assets/model_name.png)

## Follower Specification
![model_lineup](./assets/model_lineup.png)

::: info
The FFW-BG2 model is scheduled for release in July 2025, while the FFW-SG2 model is expected to be released in September 2025. Please note that the specifications provided below are preliminary and subject to change before the official product launch.
:::

| Properties                  | Specification (FFW-SG2)                | Specification (FFW-BG2)                |
|-----------------------------|----------------------------------------|----------------------------------------|
| Dimensions                  | 600 x 600 x 1,600 (WxDxH, mm)<br>23.6 x 23.6 x 63.0 (WxDxH, inches)| TBD (WxDxH, mm)<br>TBD (WxDxH, inches)|
| Weight                      | 85 kg (190 lbs)                         | TBD kg (TBD lbs)                      |
| Actuator                    | Arm Joint 1~5: DYNAMIXEL-Y<br>Arm Joint 6: DYNAMIXEL-X<br>Arm Joint 7: DYNAMIXEL-P<br>Neck: DYNAMIXEL-X<br>Lift: DYNAMIXEL-Y<br>(see [DYNAMIXEL Details](#dynamixel-details))|Arm Joint 1~5: DYNAMIXEL-Y<br>Arm Joint 6: DYNAMIXEL-X<br>Arm Joint 7: DYNAMIXEL-P<br>Neck: DYNAMIXEL-X<br>Lift: DYNAMIXEL-Y<br>(see [DYNAMIXEL Details](#dynamixel-details))|
| Degrees of Freedom          | - Total: 25 DOF<br>- Arm: 7 DOF x 2<br>- Gripper: 1 DOF x 2<br>- Head: 2 DOF x 1<br>- Lift: 1 DOF x 1<br>- Mobile: 6 DOF | - Total: 19 DOF<br>- Arm: 7 DOF x 2<br>- Gripper: 1 DOF x 2<br>- Head: 2 DOF x 1<br>- Lift: 1 DOF x 1 |
| Arm Reach                   | 647 mm (without Hand)                   | 647 mm (without Hand)                  |
| Arm Payload                 | 1.5 kg (without Hand)                   | 1.5 kg (without Hand)                  |
| Joint Resolution            | -π(rad) ~ π(rad), -262,144 ~ 262,144 (pulse/rev)| -π(rad) ~ π(rad), -262,144 ~ 262,144 (pulse/rev)|
| Joint Range                 | Joint 1, 2 : ±xxx°<br>Joint 3 : ±xxx°<br>Joint 4, 5, 6, 7 : ±xxx°| Joint 1, 2 : ±xxx°<br>Joint 3 : ±xxx°<br>Joint 4, 5, 6, 7 : ±xxx°|
| Gripper/Hand                | RH-P12-RN<br>(see [DYNAMIXEL Details](#dynamixel-details))| RH-P12-RN<br>(see [DYNAMIXEL Details](#dynamixel-details))|
| Mobile Type                 | Swerve Drive                            | None                                   |
| Mobile Operation Velocity   | 1.5 m/s                                 | None                                   |
| Battery Capacity            | 25.48V / 80Ah                           | 25.48V / 80Ah                          |
| Operating time              | Up to 4 hours                           | Up to 6 hours                          |
| Ambient Operating Temperature | 0 ~ 40 ℃                              | 0 ~ 40 ℃                              |
| Exterior Materials          | Aluminum, Plastic                       | Aluminum, Plastic                      |
| Sensor                      | RGBD Camera x 3, LiDAR x 2, IMU         | RGBD Camera x 3                        |
| Host Interface              | Ethernet                                | Ethernet                               |
| Internal Communications     | RS-485                                  | RS-485                                 |
| Communication Baudrate      | 4 Mbps                                  | 4 Mbps                                 |
| Computer                    | NVIDIA Jetson AGX Orin 32GB             | NVIDIA Jetson AGX Orin 32GB            |
| Software                    | ROS 2 Support, Python, C++, Web UI      | ROS 2 Support, Python, C++, Web UI     |

## Leader Specification
<div style="display: flex; justify-content: space-between; gap: 20px;">
    <div style="flex: 1; text-align: center; display: flex; flex-direction: column; align-items: center;">
        <p style="margin: 0 0 10px 0; font-weight: bold;">FFW-LG2 Model</p>
        <div style="display: flex; align-items: center; justify-content: center; height: 280px;">
            <img src="./assets/model_ffw_lg2.png" alt="FFW-LG2 Model" style="max-height: 250px; width: auto; object-fit: contain;">
        </div>
        <div style="display: flex; align-items: center; justify-content: center; height: 280px; width: 100%;">
            <img src="./assets/model_ffw_lg2.gif" alt="FFW-LG2 Model Animation" style="max-height: 250px; width: 100%; object-fit: contain;">
        </div>
    </div>
    <div style="flex: 1; text-align: center; display: flex; flex-direction: column; align-items: center;">
        <p style="margin: 0 0 10px 0; font-weight: bold;">FFW-LH5 Model</p>
        <div style="display: flex; align-items: center; justify-content: center; height: 280px;">
            <img src="./assets/model_ffw_lh5.png" alt="FFW-LH5 Model" style="max-height: 250px; width: auto; object-fit: contain;">
        </div>
        <div style="display: flex; align-items: center; justify-content: center; height: 280px; width: 100%;">
            <img src="./assets/model_ffw_lh5.gif" alt="FFW-LH5 Model Animation" style="max-height: 250px; width: 100%; object-fit: contain;">
        </div>
    </div>
</div>

| Properties                  | Specification (FFW-LG2)                 | Specification (FFW-LH5)                |
|-----------------------------|-----------------------------------------|----------------------------------------|
| Dimensions                  | TBD (WxDxH, mm)<br>TBD (WxDxH, inches)  | TBD (WxDxH, mm)<br>TBD (WxDxH, inches) |
| Weight                      | TBD kg (TBD lbs)                        | TBD kg (TBD lbs)                       |
| Actuator                    | Joint 1~7: DYNAMIXEL-X                  | Joint 1~7: DYNAMIXEL-X                 |
| Degrees of Freedom          | - Total: 22 DOF<br>- Arm: 7 DOF x 2<br>- Gripper: 1 DOF x 2<br>- JoyStick: 3 DOF x 2 | - Total: 60 DOF<br>- Arm: 7 DOF x 2<br>- Hand: 20 DOF x 2<br>- JoyStick: 3 DOF x 2  |
| Exterior Materials          | Aluminum, Plastic                       | Aluminum, Plastic                      |
| Internal Communications     | RS-485                                  | RS-485                                 |
| Communication Baudrate      | 4 Mbps                                  | 4 Mbps                                 |
| Software                    | ROS 2 Support, Python, C++              | ROS 2 Support, Python, C++|

## Dexterous Hand
Advanced 4 and 5 finger dexterous hands for the AI Worker are currently under development. These hands are designed for complex manipulation tasks requiring human-like dexterity. Detailed specifications and options will be published in the near future as development is completed. Please contact us for more information about upcoming dexterous hand availability.

## Joint Configuration and Nomenclature
<a href="./assets/joints.png" target="_blank">
  <img src="./assets/joints.png" alt="Dimensions" width="100%">
</a>

| ID | Joint Name          | Technical Name | Range     |
|----|---------------------|----------------|-----------|
|1   |Left Shoulder Pitch  |arm_r_joint1    |-180° ~180°|
|2   |Left Shoulder Roll   |arm_r_joint2    |-10° ~ 190°|
|3   |Left Shoulder Yaw    |arm_r_joint3    |-180° ~180°|
|4   |Left Elbow           |arm_r_joint4    |-170° ~ 65°|
|5   |Left Wrist Yaw       |arm_r_joint5    |-180° ~180°|
|6   |Left Wrist Pitch     |arm_r_joint6    |-95° ~100°|
|7   |Left Wrist Roll      |arm_r_joint7    |-130° ~ 95°|
|8   |Left Gripper         |r_rh_r1_joint   |0 ~ 106 mm|
|31  |Right Shoulder Pitch |arm_l_joint1    |-180° ~180°|
|32  |Right Shoulder Roll  |arm_l_joint2    |-10° ~ 190°|
|33  |Right Shoulder Yaw   |arm_l_joint3    |-180° ~180°|
|34  |Right Elbow          |arm_l_joint4    |-170° ~ 65°|
|35  |Right Wrist Yaw      |arm_l_joint5    |-180° ~180°|
|36  |Right Wrist Pitch    |arm_l_joint6    |-95° ~100°|
|37  |Right Wrist Roll     |arm_l_joint7    |-130° ~ 95°|
|38  |Right Gripper        |l_rh_r1_joint   |0 ~ 106 mm|
|61  |Head Pitch           |head_joint1     |-50° ~ 30°|
|62  |Head Yaw             |head_joint2     |-20° ~ 20°|
|81  |Lift                 |lift_joint      |0 ~ 500 mm|


| Part Name          | Technical Name | Range      |
|--------------------|----------------|------------|
|Left Wheel Steer    |l_wheel_steer   |-90° ~ 90°  |
|Right Wheel Steer   |r_wheel_steer   |-90° ~ 90°  |
|Rear Wheel Steer    |b_wheel_steer   |-90° ~ 90°  |
|Left Wheel Drive    |l_wheel_drive   |-360° ~ 360°|
|Right Wheel Drive   |r_wheel_drive   |-360° ~ 360°|
|Rear Wheel Drive    |b_wheel_drive   |-360° ~ 360°|

## DYNAMIXEL Details
The AI Worker utilizes various DYNAMIXEL actuators, each selected for specific joint applications:

| Joints         | DYNAMIXEL Series | DYNAMIXEL Model   |
|----------------|------------------|-------------------|
| Arm Joints 1~3 | DYNAMIXEL-Y      | [YM080-230-R099-RH](https://emanual.robotis.com/docs/en/dxl/y/ym080-230-r099-rh/) |
| Arm Joints 4~5 | DYNAMIXEL-Y      | [YM070-210-R099-RH](https://emanual.robotis.com/docs/en/dxl/y/ym070-210-r099-rh/) |
| Arm Joint 6    | DYNAMIXEL-X      | [XH540-V270-R](https://emanual.robotis.com/docs/en/dxl/x/xh540-v270/)      |
| Arm Joint 7    | DYNAMIXEL-P      | [PH42-020-S300-R](https://emanual.robotis.com/docs/en/dxl/p/ph42-020-s300-r/)   |
| Gripper        | Robot Hands      | [RH-P12-RN](https://emanual.robotis.com/docs/en/platform/rh_p12_rn/) |
| Head Pitch     | DYNAMIXEL-X      | [XH540-V150-R](https://emanual.robotis.com/docs/en/dxl/x/xh540-v150/)      |
| Head Yaw       | DYNAMIXEL-X      | [XH430-V210-R](https://emanual.robotis.com/docs/en/dxl/x/xh430-v210/)      |
| Lift           | DYNAMIXEL-Y      | [YM080-230-B001-RH](https://emanual.robotis.com/docs/en/dxl/y/ym080-230-b001-rh/) |
| Wheel Steer    | DYNAMIXEL-Y      | [YM070-210-R051-RH](https://emanual.robotis.com/docs/en/dxl/y/ym070-210-r051-rh/) |

For detailed specifications of each DYNAMIXEL model, please visit the [ROBOTIS DYNAMIXEL website](https://dynamixel.com/).

## Dimension
### FFW-SG2
<a href="./assets/dimension_ffw_sg2.png" target="_blank">
  <img src="./assets/dimension_ffw_sg2.png" alt="Dimensions" width="100%">
</a>

### FFW-BG2
<a href="./assets/dimension_ffw_bg2.png" target="_blank">
  <img src="./assets/dimension_ffw_bg2.png" alt="Dimensions" width="100%">
</a>

## Workspace
The torso workspace is the same for both FFW-BG2 and FFW-SG2 models.

<a href="./assets/workspace.png" target="_blank">
  <img src="./assets/workspace.png" alt="Workspace" width="100%">
</a>
