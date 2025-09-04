# OMX

The OMX hardware platform is an entry-level Physical AI manipulator designed for `imitation learning` and `teleoperation`. It consists of two main components: the `OMX-F` follower robot arm and the `OMX-L` leader device. Built on DYNAMIXEL‑X actuators and ROS 2‑native workflows, it supports end‑to‑end IL pipelines and seamless integration with Physical AI Tools and LeRobot.

![omy_f3m](/specifications/omx/main_image.png)

## [Follower] Hardware Overview
![](/specifications/omx/hw_overview_omx_f.png)

| Item | OMX (Follower) |
|:---:|:---:|
| DOF | 5 + 1Gripper |
| Full Reach | 400 [mm] |
| Weight | 560 [g] |
| Operating Voltage | 12 [VDC] |
| Joint Resolution | -π(rad) ~ π(rad), -2,048 ~ 2,048 [pulse/rev] |
| Joint Range | Joint 1 : -270° ~ +360°<br>Joint 2, 3 : -120° ~ +90°<br>Joint 4 : -100° ~ +100°<br>Joint 5 : ±270°<br>Joint 6 : 0° ~ + 100° |
| DYNAMIXEL-X Specification | ID 11~13 : XL430-W250-T<br>ID 14~16 : XL330-M288-T |
| Host Interface | USB C-Type |
| Internal Communications | TTL |
| Communication Baudrate | 1 [Mbps] |
| SW | ROS 2 |
| Payload | Full reach : 100g , Normal reach : 250g |

## [Follower] Layout
![](/specifications/omx/omx_follower_layout.png)

## [Leader] Hardware Overview

![](/specifications/omx/hw_overview_omx_l.png)

| Item | OMX (Leader) |
|:---:|:---:|
| DOF | 5 + 1Gripper |
| Full Reach | 335 [mm] |
| Weight | 360 [g] |
| Operating Voltage | 5 [VDC] |
| Joint Resolution | -π(rad) ~ π(rad), -2,048 ~ 2,048 [pulse/rev] |
| Joint Range | Joint 1 : -270° ~ +360°<br>Joint 2, 3 : -120° ~ +90°<br>Joint 4 : -100° ~ +100°<br>Joint 5 : ±270°<br>Joint 6 : 0° ~ + 100° |
| DYNAMIXEL-X Specification | Joint 1~5 : XL330-M288-T<br>Joint 6 : XL330-M077-T |
| Host Interface | USB C-Type |
| Internal Communications | TTL |
| Communication Baudrate | 1 [Mbps] |
| SW | ROS 2 |


## [Leader] Layout
![](/specifications/omx/omx_leader_layout.png)