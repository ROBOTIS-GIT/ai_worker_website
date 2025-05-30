# Hardware

## Hardware Overview
The AI Worker hardware platform combines advanced robotics with AI-powered capabilities to create a versatile semi-humanoid robot. Built around ROBOTIS [DYNAMIXEL](https://dynamixel.com/) actuators, it features a mobile base with swerve drive, dual 7-DOF arms with dexterous hand/grippers, and a sensor suite that includes RGBD cameras and LiDARs. The system is powered by an NVIDIA Jetson AGX Orin computer and offers up to 8 hours of operation time on a single charge. This comprehensive hardware design enables the AI Worker to perform complex manipulation tasks through imitation learning and reinforcement learning in various industrial environments.

![hardware_overview](/specifications/hardware_overview.png)

## Product Models
![model_name](/specifications/model_name.png)

## Follower Specification
![model_lineup](/specifications/model_lineup.png)

::: info
The FFW-BG2 model is scheduled for release in July 2025, while the FFW-SG2 model is expected to be released in September 2025. Please note that the specifications provided below are preliminary and subject to change before the official product launch.
:::

| Properties                  | Specification (FFW-SG2)                | Specification (FFW-BG2)                |
|-----------------------------|----------------------------------------|----------------------------------------|
| Dimensions (WxDxH)          | 604x602x1,623 (mm)<br>23.8x23.7x63.8 (inch)| 604x564x1,607 (mm)<br>23.8x22.2x63.2 (inch)|
| Weight                      | 90 kg (198 lb)                         | 85 kg (187 lb)                         |
| Actuator                    | Arm Joint 1~6: DYNAMIXEL-Y<br>Arm Joint 7: DYNAMIXEL-P<br>Head: DYNAMIXEL-X<br>Lift: DYNAMIXEL-Y<br>(see [DYNAMIXEL Details](#dynamixel-details))|Arm Joint 1~6: DYNAMIXEL-Y<br>Arm Joint 7: DYNAMIXEL-P<br>Head: DYNAMIXEL-X<br>Lift: DYNAMIXEL-Y<br>(see [DYNAMIXEL Details](#dynamixel-details)) |
| Degrees of Freedom          | - Total: 25 DOF<br>- Arm: 7 DOF x 2<br>- Gripper: 1 DOF x 2<br>- Head: 2 DOF x 1<br>- Lift: 1 DOF x 1<br>- Mobile: 6 DOF | - Total: 19 DOF<br>- Arm: 7 DOF x 2<br>- Gripper: 1 DOF x 2<br>- Head: 2 DOF x 1<br>- Lift: 1 DOF x 1 |
| Arm Reach                   | 641 mm (to wrist) + hand               | 641 mm (to wrist) + hand               |
| Arm Payload                 | 1.5 kg (without Hand)                  | 1.5 kg (without Hand)                  |
| Joint Resolution            | -π(rad)~π(rad)<br>-262,144~262,144 (pulse/rev)| -π(rad)~π(rad)<br>-262,144~262,144 (pulse/rev)|
| Joint Range                 | (see [Joint Configuration](#joint-configuration))| (see [Joint Configuration](#joint-configuration))|
| Gripper/Hand                | Standard: RH-P12-RN<br>(see [Gripper Specification](#gripper-specification))<br>(Dexterous hands in development, see [Hand Specification](#hand-specification)) | Standard: RH-P12-RN<br>(see [Gripper Specification](#gripper-specification))<br>(Dexterous hands in development, see [Hand Specification](#hand-specification)) |
| Mobile Type                 | Swerve Drive                           | None                                   |
| Mobile Operation Velocity   | 1.5 m/s                                | None                                   |
| Power                       | Battery: 25V, 80Ah (2,040Wh)           | SMPS: 24VDC, 80A, 1,920W               |
| Operating time              | Up to 8 hours                          | Continuous operation<br>(AC powered via SMPS)|
| Ambient Operating Temperature | 0 ~ 40℃                             | 0 ~ 40℃                                |
| Exterior Materials          | Aluminum, Plastic                      | Aluminum, Plastic                      |
| Sensor                      | RGBD Camera x 3, LiDAR x 2, IMU<br>(see [Camera Specification](#camera-specification))| RGBD Camera x 3<br>(see [Camera Specification](#camera-specification))|
| Host Interface              | Ethernet<br>(Wi-Fi router up to 1Gbps)    | Ethernet<br>(Direct connection to host PC)|
| Internal Communications     | RS-485                                 | RS-485                                 |
| Communication Baudrate      | 4 Mbps                                 | 4 Mbps                                 |
| Computer                    | NVIDIA Jetson AGX Orin 32GB            | NVIDIA Jetson AGX Orin 32GB            |
| Software                    | ROS 2 Support, Python, C++, Web UI     | ROS 2 Support, Python, C++, Web UI     |

## Leader Specification
<div style="display: flex; justify-content: space-between; gap: 20px;">
    <div style="flex: 1; text-align: center; display: flex; flex-direction: column; align-items: center;">
        <p style="margin: 0 0 10px 0; font-weight: bold;">FFW-LG2 Model</p>
        <div style="display: flex; align-items: center; justify-content: center; height: 280px;">
            <img src="/specifications/model_ffw_lg2.png" alt="FFW-LG2 Model" style="max-height: 250px; width: auto; object-fit: contain;">
        </div>
        <div style="display: flex; align-items: center; justify-content: center; height: 280px; width: 100%;">
            <img src="/specifications/model_ffw_lg2.gif" alt="FFW-LG2 Model Animation" style="max-height: 250px; width: 100%; object-fit: contain;">
        </div>
    </div>
    <div style="flex: 1; text-align: center; display: flex; flex-direction: column; align-items: center;">
        <p style="margin: 0 0 10px 0; font-weight: bold;">FFW-LH5 Model</p>
        <div style="display: flex; align-items: center; justify-content: center; height: 280px;">
            <img src="/specifications/model_ffw_lh5.png" alt="FFW-LH5 Model" style="max-height: 250px; width: auto; object-fit: contain;">
        </div>
        <div style="display: flex; align-items: center; justify-content: center; height: 280px; width: 100%;">
            <img src="/specifications/model_ffw_lh5.gif" alt="FFW-LH5 Model Animation" style="max-height: 250px; width: 100%; object-fit: contain;">
        </div>
    </div>
</div>



| Properties               | Specification (FFW-LG2)                 | Specification (FFW-LH5)                |
|--------------------------|-----------------------------------------|----------------------------------------|
| Dimensions (WxDxH)       | 598x146x705 (mm)<br>23.5x5.7x27.8 (inch)| 597x211x842 (mm)<br>23.5x8.3x33.2 (inch)|
| Weight                   | 3 kg (6.6 lb)                           | 3 kg (6.6 lb)                          |
| Actuator                 | Joint 1~7: DYNAMIXEL-X                  | Joint 1~7: DYNAMIXEL-X                 |
| Degrees of Freedom       | - Total: 22 DOF<br>- Arm: 7 DOF x 2<br>- Gripper: 1 DOF x 2<br>- JoyStick: 3 DOF x 2 | - Total: 60 DOF<br>- Arm: 7 DOF x 2<br>- Hand: 20 DOF x 2<br>- JoyStick: 3 DOF x 2 |
| Exterior Materials       | Aluminum, Plastic                       | Aluminum, Plastic                      |
| Internal Communications  | RS-485                                  | RS-485                                 |
| Communication Baudrate   | 4 Mbps                                  | 4 Mbps                                 |
| Software                 | ROS 2 Support, Python, C++              | ROS 2 Support, Python, C++             |

## Camera Specification

The AI Worker utilizes a multi-camera system for robust perception and manipulation capabilities.

### Head Camera: Stereolabs ZED Mini

A ZED Mini camera is mounted on the head for wide-area environmental understanding and navigation.

| Feature           | Specification                                      |
|-------------------|----------------------------------------------------|
| Model Name        | Stereolabs ZED Mini                                |
| Type              | Stereoscopic RGBD with 6DoF IMU                    |
| Key Use           | Wide-angle depth perception, SLAM, obstacle avoidance |
| Field of View     | 102°(H) x 57°(V)                                   |
| Max Resolution    | 2208 x 1242                                        |
| Depth Range       | 0.1m to 9m (3.9inch to 354.3inch)                  |
| More Information  | [Stereolabs ZED Mini Details](https://www.stereolabs.com/store/products/zed-mini) |

### Hand Cameras: Intel RealSense D405 (x2)

Two Intel RealSense D405 cameras are integrated into the robot's hands/grippers, providing precise, short-range depth data for manipulation tasks.

| Feature           | Specification                                      |
|-------------------|----------------------------------------------------|
| Model Name        | Intel RealSense D405                               |
| Type              | Stereoscopic RGBD                                  |
| Key Use           | Close-range depth sensing for grasping & manipulation |
| Field of View     | 87°(H) × 58°(V)                                    |
| Max Resolution    | 1280 x 720                                         |
| Depth Range       | 7cm to 50cm (2.7inch to 19.6inch)                  |
| More Information  | [Intel RealSense D405 Details](https://www.intelrealsense.com/depth-camera-d405/) |

## Gripper Specification
![model_rh_p12_rn](/specifications/model_rh_p12_rn.png)
The [RH-P12-RN](https://emanual.robotis.com/docs/en/platform/rh_p12_rn/) is a multi-functional, 1-DOF two-fingered robot hand. It features an adaptive gripping design with passive joints, allowing it to grasp objects of various shapes. The hand utilizes a 10W DC motor and offers torque control and current-based position control, enabling it to handle objects made of different materials. Key features of the RH-P12-RN include its lightweight design (500g) and high payload capacity (5kg). It also has detachable fingertips that can be easily replaced with customized ones. The RH-P12-RN is designed for easy integration with the AI Worker platform, making it suitable for various applications in industrial environments.

## Hand Specification
Advanced 4 and 5 finger dexterous hands for the AI Worker are currently under development. These hands are designed for complex manipulation tasks requiring human-like dexterity. Detailed specifications and options will be published in the near future as development is completed. Please contact us for more information about upcoming dexterous hand availability.

## Joint Configuration and Nomenclature
<a href="/specifications/joints.png" target="_blank">
  <img src="/specifications/joints.png" alt="Dimensions" width="100%">
</a>

| ID  | Joint Name            | Technical Name  | Range        |
|-----|-----------------------|-----------------|--------------|
| 1   | Right Shoulder Pitch  | arm_r_joint1    | -180° ~ 180° |
| 2   | Right Shoulder Roll   | arm_r_joint2    | -190° ~ 10°  |
| 3   | Right Shoulder Yaw    | arm_r_joint3    | -180° ~ 180° |
| 4   | Right Elbow           | arm_r_joint4    | -170° ~ 65°  |
| 5   | Right Wrist Yaw       | arm_r_joint5    | -180° ~ 180° |
| 6   | Right Wrist Pitch     | arm_r_joint6    | -105° ~ 105° |
| 7   | Right Wrist Roll      | arm_r_joint7    | -120° ~ 90°  |
| 8   | Right Gripper         | gripper_r_joint1| 0 ~ 107.6 mm |
| 31  | Left Shoulder Pitch   | arm_l_joint1    | -180° ~ 180° |
| 32  | Left Shoulder Roll    | arm_l_joint2    | -10° ~ 190°  |
| 33  | Left Shoulder Yaw     | arm_l_joint3    | -180° ~ 180° |
| 34  | Left Elbow            | arm_l_joint4    | -170° ~ 65°  |
| 35  | Left Wrist Yaw        | arm_l_joint5    | -180° ~ 180° |
| 36  | Left Wrist Pitch      | arm_l_joint6    | -105° ~ 105° |
| 37  | Left Wrist Roll       | arm_l_joint7    | -90° ~ 120°  |
| 38  | Left Gripper          | gripper_l_joint1| 0 ~ 107.6 mm |
| 61  | Head Pitch            | head_joint1     | -50° ~ 30°   |
| 62  | Head Yaw              | head_joint2     | -20° ~ 20°   |
| 81  | Lift                  | lift_joint      | 0 ~ 500 mm   |

### Mobile Base Configuration (Swerve Drive)
The AI Worker's mobile base uses a swerve drive system that provides significant advantages over traditional omnidirectional wheel systems like mecanum wheels or omni wheels:

#### Advantages Over Omniwheels and Mecanum Wheels:

- **Superior Traction**: Swerve drive uses conventional wheels with full surface contact, providing better grip and stability compared to omniwheels and mecanum wheels that have smaller rollers with limited ground contact.

- **Higher Efficiency**: Without the passive rollers found in omniwheels and mecanum wheels, swerve drive transfers power more efficiently to the ground, resulting in better energy usage and longer operation time.

- **Greater Precision**: Swerve drive offers more precise control over movement direction and velocity, as each wheel's steering and drive are independently controlled with no slip dynamics inherent to roller-based wheels.

- **Improved Load Capacity**: The direct wheel contact allows swerve drive to handle heavier loads more effectively, making it ideal for industrial applications where the robot needs to carry or manipulate objects.

- **Better Performance on Various Surfaces**: While omniwheels and mecanum wheels struggle on uneven surfaces or soft carpets, swerve drive maintains consistent performance across different floor types.

- **Reduced Vibration**: The continuous wheel contact with the ground produces less vibration during movement, which is critical for tasks requiring precise manipulation.

The AI Worker's swerve drive configuration consists of three wheels positioned in a triangular arrangement, each with independent steering and driving capabilities:

| Part Name          | Technical Name   | Range        |
|--------------------|------------------|--------------|
|Right Wheel Steer   |right_wheel_steer | -90° ~ 90°   |
|Left Wheel Steer    |left_wheel_steer  | -90° ~ 90°   |
|Rear Wheel Steer    |rear_wheel_steer  | -90° ~ 90°   |
|Right Wheel Drive   |right_wheel_drive | -360° ~ 360° |
|Left Wheel Drive    |left_wheel_drive  | -360° ~ 360° |
|Rear Wheel Drive    |rear_wheel_drive  | -360° ~ 360° |

## DYNAMIXEL Details
The AI Worker utilizes various DYNAMIXEL actuators, each selected for specific joint applications:

| Joints         | DYNAMIXEL Series | DYNAMIXEL Model                                                                   |
|----------------|------------------|-----------------------------------------------------------------------------------|
| Arm Joints 1~3 | DYNAMIXEL-Y      | [YM080-230-R099-RH](https://emanual.robotis.com/docs/en/dxl/y/ym080-230-r099-rh/) |
| Arm Joints 4~6 | DYNAMIXEL-Y      | [YM070-210-R099-RH](https://emanual.robotis.com/docs/en/dxl/y/ym070-210-r099-rh/) |
| Arm Joint 7    | DYNAMIXEL-P      | [PH42-020-S300-R](https://emanual.robotis.com/docs/en/dxl/p/ph42-020-s300-r/)     |
| Gripper        | Robot Hands      | [RH-P12-RN](https://emanual.robotis.com/docs/en/platform/rh_p12_rn/)              |
| Head Pitch     | DYNAMIXEL-X      | [XH540-V150-R](https://emanual.robotis.com/docs/en/dxl/x/xh540-v150/)             |
| Head Yaw       | DYNAMIXEL-X      | [XH430-V210-R](https://emanual.robotis.com/docs/en/dxl/x/xh430-v210/)             |
| Lift           | DYNAMIXEL-Y      | [YM080-230-B001-RH](https://emanual.robotis.com/docs/en/dxl/y/ym080-230-b001-rh/) |
| Wheel Steer    | DYNAMIXEL-Y      | [YM070-210-R051-RH](https://emanual.robotis.com/docs/en/dxl/y/ym070-210-r051-rh/) |

For detailed specifications of each DYNAMIXEL model, please visit the [ROBOTIS DYNAMIXEL website](https://dynamixel.com/).

## Dimension
### FFW-SG2
<a href="/specifications/dimension_ffw_sg2.png" target="_blank">
  <img src="/specifications/dimension_ffw_sg2.png" alt="Dimensions" width="100%">
</a>

### FFW-BG2
<a href="/specifications/dimension_ffw_bg2.png" target="_blank">
  <img src="/specifications/dimension_ffw_bg2.png" alt="Dimensions" width="100%">
</a>

### FFW-LG2
<a href="/specifications/dimension_ffw_lg2.png" target="_blank">
  <img src="/specifications/dimension_ffw_lg2.png" alt="Dimensions" width="100%">
</a>

### FFW-LH5
<a href="/specifications/dimension_ffw_lh5.png" target="_blank">
  <img src="/specifications/dimension_ffw_lh5.png" alt="Dimensions" width="100%">
</a>

## Workspace
The torso workspace is the same for both FFW-BG2 and FFW-SG2 models.

<a href="/specifications/workspace.png" target="_blank">
  <img src="/specifications/workspace.png" alt="Workspace" width="100%">
</a>
