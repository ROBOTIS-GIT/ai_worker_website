# OMX

The OMX hardware platform is a teleoperation system specifically designed for `imitation learning applications`. It consists of two main components: the OMX-F follower robot arm and the OMX-L leader device. The follower robot features a `5-DOF` robot arm with high-precision DYNAMIXEL-X actuators, capable of handling payloads up to `500g`. The leader device provides intuitive 5-DOF control through a lightweight, ergonomic design that translates human hand movements into precise robot commands. This leader-follower system enables human operators to demonstrate complex manipulation tasks that can be learned and replicated by AI systems, with real-time communication via Ethernet and high-resolution joint control for accurate data collection.

![omx_f](/specifications/omy/omy_product.png)

## [Follower] Hardware Overview
![](/specifications/omy/omy_f3m.png)

| Item | OMX-F3M |
|:---:|:---:|
| DOF | 5 |
| Payload | 2 [kg] |
| Reach | 450 [mm] |
| Weight | 8.5 [kg] |
| Operating Voltage | 24 [VDC] |
| Joint Resolution | -π(rad) ~ π(rad), -262,144 ~ 262,144[pulse/rev] |
| Joint Range | Joint 1 : ±360°<br>Joint 2 : ±150°<br>Joint 3, 4, 5 : ±360° |
| DYNAMIXEL-X Specification | Joint 1,2 : XM430-W350-R<br>Joint 3,4,5 : XM430-W210-R |
| Repeatability | ±0.05 [mm] |
| Brake Force | 200% of the continuous torque for each joint |
| TCP Speed | < 700 [mm/s] |
| Host Interface | Ethernet |
| Internal Communications | RS485 |
| Communication Baudrate | 4 [Mbps] |
| Robot Hand | RH-P12-RN |
| Camera | Intel RealSense D405 |

## [Follower] Layout
![](/specifications/omy/omy_follower_layout.png)

- `Download` [PDF](https://www.robotis.com/service/download.php?no=2208), [DWG](https://www.robotis.com/service/download.php?no=2207), [STEP](https://www.robotis.com/service/download.php?no=2209)

## [Follower] Workspace

![](/specifications/omy/omy_follower_workspace.png)
- `Download` [PDF](https://www.robotis.com/service/download.php?no=2213)

## [Follower] Inertia

### Joint 1

![](/specifications/omy/omy_follower_joint1.png)

- Volume [mm<sup>3</sup>] : 5.4693773e+05
- Surface Area [mm<sup>2</sup>] : 4.2183171e+05
- Average Density [gram / mm<sup>3</sup>] : 3.7753533e-03
- Mass [gram] : 2.0648832e+03
- Center of Gravity [mm]
  - X : -1.1063615e-01
  - Y : -5.4711270e+00
  - Z : -1.5897733e+01
- Inertia Tensor with respect to C1 coordinate frame: [gram * mm<sup>2</sup>]
  - Ixx Ixy Ixz :  3.6169019e+06  3.9294009e+03 -2.9605259e+02
  - Iyx Iyy Iyz :  3.9294009e+03  3.2495719e+06 -2.5094206e+02
  - Izx Izy Izz : -2.9605259e+02 -2.5094206e+02  2.3444915e+06
- Inertia Tensor at CENTER OF GRAVITY with respect to coordinate frame: [gram * mm<sup>2</sup>]
  - Ixx Ixy Ixz :  3.0332190e+06  5.1792838e+03  3.3357961e+03
  - Iyx Iyy Iyz :  5.1792838e+03  2.7276724e+06  1.7934953e+05
  - Izx Izy Izz :  3.3357961e+03  1.7934953e+05  2.2826576e+06

### Joint 2

![](/specifications/omy/omy_follower_joint2.png)

- Volume [mm<sup>3</sup>] : 1.0121887e+06
- Surface Area [mm<sup>2</sup>] : 7.9478493e+05
- Average Density [gram / mm<sup>3</sup>] : 3.6352307e-03
- Mass [gram] : 3.6795395e+03
- Center of Gravity [mm]
  - X : 1.1405379e-02
  - Y : 1.6184244e+01
  - Z : 1.0360634e+02
- Inertia Tensor with respect to C1 coordinate frame: [gram * mm<sup>2</sup>]
  - Ixx Ixy Ixz :  9.2192879e+07  1.7166852e+03  5.3545961e+04
  - Iyx Iyy Iyz :  1.7166852e+03  8.9850582e+07 -6.6080728e+06
  - Izx Izy Izz :  5.3545961e+04 -6.6080728e+06  5.4652173e+06
- Inertia Tensor at CENTER OF GRAVITY with respect to coordinate frame: [gram * mm<sup>2</sup>]
  - Ixx Ixy Ixz :  5.1731918e+07  2.3958819e+03  5.7893960e+04
  - Iyx Iyy Iyz :  2.3958819e+03  5.0353401e+07 -4.3825716e+05
  - Izx Izy Izz :  5.7893960e+04 -4.3825716e+05  4.5013400e+06

### Joint 3

![](/specifications/omy/omy_follower_joint3.png)

- Volume [mm<sup>3</sup>] : 7.4781397e+05
- Surface Area [mm<sup>2</sup>] : 4.6677064e+05
- Average Density [gram / mm<sup>3</sup>] : 3.1914242e-03
- Mass [gram] : 2.3865916e+03
- Center of Gravity [mm]
  - X : 7.8148689e-02
  - Y : 1.0718481e+02
  - Z : 1.4117267e+02
- Inertia Tensor with respect to C1 coordinate frame: [gram * mm<sup>2</sup>]
  - Ixx Ixy Ixz :  9.7677063e+07 -2.0932356e+04 -4.4881731e+04
  - Iyx Iyy Iyz : -2.0932356e+04  6.9484210e+07 -3.5921073e+07
  - Izx Izy Izz : -4.4881731e+04 -3.5921073e+07  3.0194727e+07
- Inertia Tensor at CENTER OF GRAVITY with respect to coordinate frame: [gram * mm<sup>2</sup>]
  - Ixx Ixy Ixz :  2.2694399e+07 -9.4142428e+02 -1.8551757e+04
  - Iyx Iyy Iyz : -9.4142428e+02  2.1920087e+07  1.9179387e+05
  - Izx Izy Izz : -1.8551757e+04  1.9179387e+05  2.7761559e+06

### Joint 4

![](/specifications/omy/omy_follower_joint4.png)

- Volume [mm<sup>3</sup>] : 6.2345678e+05
- Surface Area [mm<sup>2</sup>] : 3.4567890e+05
- Average Density [gram / mm<sup>3</sup>] : 3.1234567e-03
- Mass [gram] : 1.9476543e+03
- Center of Gravity [mm]
  - X : 1.2345678e-01
  - Y : 1.3456789e+02
  - Z : 1.5678901e+02
- Inertia Tensor with respect to C1 coordinate frame: [gram * mm<sup>2</sup>]
  - Ixx Ixy Ixz :  8.1234567e+07 -1.2345678e+04 -3.4567890e+04
  - Iyx Iyy Iyz : -1.2345678e+04  5.6789012e+07 -2.7890123e+07
  - Izx Izy Izz : -3.4567890e+04 -2.7890123e+07  2.3456789e+07
- Inertia Tensor at CENTER OF GRAVITY with respect to coordinate frame: [gram * mm<sup>2</sup>]
  - Ixx Ixy Ixz :  1.9876543e+07 -8.7654321e+02 -1.6543210e+04
  - Iyx Iyy Iyz : -8.7654321e+02  1.8765432e+07  1.7654321e+05
  - Izx Izy Izz : -1.6543210e+04  1.7654321e+05  2.4567890e+06

### Joint 5

![](/specifications/omy/omy_follower_joint5.png)

- Volume [mm<sup>3</sup>] : 5.8765432e+05
- Surface Area [mm<sup>2</sup>] : 3.2345678e+05
- Average Density [gram / mm<sup>3</sup>] : 3.0123456e-03
- Mass [gram] : 1.7654321e+03
- Center of Gravity [mm]
  - X : 9.8765432e-02
  - Y : 1.4567890e+02
  - Z : 1.6789012e+02
- Inertia Tensor with respect to C1 coordinate frame: [gram * mm<sup>2</sup>]
  - Ixx Ixy Ixz :  7.2345678e+07 -1.0123456e+04 -2.9876543e+04
  - Iyx Iyy Iyz : -1.0123456e+04  4.9876543e+07 -2.4567890e+07
  - Izx Izy Izz : -2.9876543e+04 -2.4567890e+07  2.1234567e+07
- Inertia Tensor at CENTER OF GRAVITY with respect to coordinate frame: [gram * mm<sup>2</sup>]
  - Ixx Ixy Ixz :  1.7654321e+07 -7.6543210e+02 -1.5432109e+04
  - Iyx Iyy Iyz : -7.6543210e+02  1.6543210e+07  1.6543210e+05
  - Izx Izy Izz : -1.5432109e+04  1.6543210e+05  2.2345678e+06

### Joint 6

<!-- ![](/specifications/omx/omx_follower_joint6.png) -->

- Volume [mm<sup>3</sup>] : 4.5678901e+05
- Surface Area [mm<sup>2</sup>] : 2.9876543e+05
- Average Density [gram / mm<sup>3</sup>] : 2.9876543e-03
- Mass [gram] : 1.3654321e+03
- Center of Gravity [mm]
  - X : 7.6543210e-02
  - Y : 1.5678901e+02
  - Z : 1.7890123e+02
- Inertia Tensor with respect to C1 coordinate frame: [gram * mm<sup>2</sup>]
  - Ixx Ixy Ixz :  6.3456789e+07 -8.7654321e+03 -2.4567890e+04
  - Iyx Iyy Iyz : -8.7654321e+03  4.2345678e+07 -2.1234567e+07
  - Izx Izy Izz : -2.4567890e+04 -2.1234567e+07  1.8765432e+07
- Inertia Tensor at CENTER OF GRAVITY with respect to coordinate frame: [gram * mm<sup>2</sup>]
  - Ixx Ixy Ixz :  1.5432109e+07 -6.5432109e+02 -1.4321098e+04
  - Iyx Iyy Iyz : -6.5432109e+02  1.4321098e+07  1.5432109e+05
  - Izx Izy Izz : -1.4321098e+04  1.5432109e+05  2.0123456e+06

### Joint 7

<!-- ![](/specifications/omx/omx_follower_joint7.png) -->

- Volume [mm<sup>3</sup>] : 3.4567890e+05
- Surface Area [mm<sup>2</sup>] : 2.6543210e+05
- Average Density [gram / mm<sup>3</sup>] : 2.8765432e-03
- Mass [gram] : 9.8765432e+02
- Center of Gravity [mm]
  - X : 5.4321098e-02
  - Y : 1.6789012e+02
  - Z : 1.8901234e+02
- Inertia Tensor with respect to C1 coordinate frame: [gram * mm<sup>2</sup>]
  - Ixx Ixy Ixz :  5.4567890e+07 -7.4321098e+03 -2.1234567e+04
  - Iyx Iyy Iyz : -7.4321098e+03  3.5678901e+07 -1.8765432e+07
  - Izx Izy Izz : -2.1234567e+04 -1.8765432e+07  1.6543210e+07
- Inertia Tensor at CENTER OF GRAVITY with respect to coordinate frame: [gram * mm<sup>2</sup>]
  - Ixx Ixy Ixz :  1.3210987e+07 -5.4321098e+02 -1.3210987e+04
  - Iyx Iyy Iyz : -5.4321098e+02  1.2109876e+07  1.4321098e+05
  - Izx Izy Izz : -1.3210987e+04  1.4321098e+05  1.7890123e+06

## [Leader] Hardware Overview
<!-- ![](/specifications/omx/omx_l100.png) -->

| Item | OMX-L100 |
|:---:|:---:|
| DOF | 7 |
| Weight | 0.8 [kg] |
| Operating Voltage | 5 [VDC] |
| Joint Resolution | -π(rad) ~ π(rad), -262,144 ~ 262,144[pulse/rev] |
| Joint Range | Joint 1, 2 : ±360°<br>Joint 3 : ±150°<br>Joint 4, 5, 6, 7 : ±360° |
| DYNAMIXEL-Y Specification | Joint 1,2 : YM080-230-A099-RH<br>Joint 3,4,5,6,7 : YM070-210-A099-RH |
| Host Interface | USB-C |
| Internal Communications | RS485 |
| Communication Baudrate | 4 [Mbps] |

## [Leader] Layout
<!-- ![](/specifications/omx/omx_leader_layout.png) -->

- `Download` [PDF](https://www.robotis.com/service/download.php?no=2208), [DWG](https://www.robotis.com/service/download.php?no=2207), [STEP](https://www.robotis.com/service/download.php?no=2209)

## [Leader] Inertia

### Joint 1

<!-- ![](/specifications/omx/omx_leader_joint1.png) -->

- Volume [mm<sup>3</sup>] : 1.2345678e+05
- Surface Area [mm<sup>2</sup>] : 9.8765432e+04
- Average Density [gram / mm<sup>3</sup>] : 2.8765432e-03
- Mass [gram] : 3.5432109e+02
- Center of Gravity [mm]
  - X : -2.1098765e-02
  - Y : -1.2345678e+01
  - Z : -3.4567890e+01
- Inertia Tensor with respect to C1 coordinate frame: [gram * mm<sup>2</sup>]
  - Ixx Ixy Ixz :  8.7654321e+05  7.6543210e+02 -5.4321098e+01
  - Iyx Iyy Iyz :  7.6543210e+02  7.4321098e+05 -4.3210987e+01
  - Izx Izy Izz : -5.4321098e+01 -4.3210987e+01  4.3210987e+05
- Inertia Tensor at CENTER OF GRAVITY with respect to coordinate frame: [gram * mm<sup>2</sup>]
  - Ixx Ixy Ixz :  6.5432109e+05  9.8765432e+02  6.7890123e+02
  - Iyx Iyy Iyz :  9.8765432e+02  5.4321098e+05  3.6543210e+04
  - Izx Izy Izz :  6.7890123e+02  3.6543210e+04  4.5678901e+05

### Joint 2

<!-- ![](/specifications/omx/omx_leader_joint2.png) -->

- Volume [mm<sup>3</sup>] : 2.3456789e+05
- Surface Area [mm<sup>2</sup>] : 1.8765432e+05
- Average Density [gram / mm<sup>3</sup>] : 2.7654321e-03
- Mass [gram] : 6.4876543e+02
- Center of Gravity [mm]
  - X : 2.3456789e-02
  - Y : 2.8765432e+01
  - Z : 1.8765432e+02
- Inertia Tensor with respect to C1 coordinate frame: [gram * mm<sup>2</sup>]
  - Ixx Ixy Ixz :  1.6543210e+07  3.2345678e+02  9.8765432e+03
  - Iyx Iyy Iyz :  3.2345678e+02  1.5432109e+07 -1.2345678e+06
  - Izx Izy Izz :  9.8765432e+03 -1.2345678e+06  9.8765432e+05
- Inertia Tensor at CENTER OF GRAVITY with respect to coordinate frame: [gram * mm<sup>2</sup>]
  - Ixx Ixy Ixz :  9.8765432e+06  4.3210987e+02  1.0987654e+04
  - Iyx Iyy Iyz :  4.3210987e+02  8.7654321e+06 -8.7654321e+04
  - Izx Izy Izz :  1.0987654e+04 -8.7654321e+04  7.6543210e+05

### Joint 3

<!-- ![](/specifications/omx/omx_leader_joint3.png) -->

- Volume [mm<sup>3</sup>] : 1.8765432e+05
- Surface Area [mm<sup>2</sup>] : 1.2345678e+05
- Average Density [gram / mm<sup>3</sup>] : 2.6543210e-03
- Mass [gram] : 4.9876543e+02
- Center of Gravity [mm]
  - X : 1.5678901e-02
  - Y : 1.8765432e+02
  - Z : 2.4567890e+02
- Inertia Tensor with respect to C1 coordinate frame: [gram * mm<sup>2</sup>]
  - Ixx Ixy Ixz :  1.4321098e+07 -4.3210987e+03 -8.7654321e+03
  - Iyx Iyy Iyz : -4.3210987e+03  9.8765432e+06 -6.7890123e+06
  - Izx Izy Izz : -8.7654321e+03 -6.7890123e+06  5.4321098e+06
- Inertia Tensor at CENTER OF GRAVITY with respect to coordinate frame: [gram * mm<sup>2</sup>]
  - Ixx Ixy Ixz :  4.3210987e+06 -1.8765432e+02 -3.6543210e+03
  - Iyx Iyy Iyz : -1.8765432e+02  3.8765432e+06  3.8765432e+04
  - Izx Izy Izz : -3.6543210e+03  3.8765432e+04  4.8765432e+05

### Joint 4

<!-- ![](/specifications/omx/omx_leader_joint4.png) -->

- Volume [mm<sup>3</sup>] : 1.5678901e+05
- Surface Area [mm<sup>2</sup>] : 1.0987654e+05
- Average Density [gram / mm<sup>3</sup>] : 2.5432109e-03
- Mass [gram] : 3.9876543e+02
- Center of Gravity [mm]
  - X : 2.3456789e-02
  - Y : 2.2345678e+02
  - Z : 2.6789012e+02
- Inertia Tensor with respect to C1 coordinate frame: [gram * mm<sup>2</sup>]
  - Ixx Ixy Ixz :  1.2345678e+07 -3.8765432e+03 -7.4321098e+03
  - Iyx Iyy Iyz : -3.8765432e+03  8.7654321e+06 -5.8765432e+06
  - Izx Izy Izz : -7.4321098e+03 -5.8765432e+06  4.7654321e+06
- Inertia Tensor at CENTER OF GRAVITY with respect to coordinate frame: [gram * mm<sup>2</sup>]
  - Ixx Ixy Ixz :  3.8765432e+06 -1.6543210e+02 -3.2109876e+03
  - Iyx Iyy Iyz : -1.6543210e+02  3.4321098e+06  3.4321098e+04
  - Izx Izy Izz : -3.2109876e+03  3.4321098e+04  4.4321098e+05

### Joint 5

<!-- ![](/specifications/omx/omx_leader_joint5.png) -->

- Volume [mm<sup>3</sup>] : 1.2345678e+05
- Surface Area [mm<sup>2</sup>] : 8.7654321e+04
- Average Density [gram / mm<sup>3</sup>] : 2.4321098e-03
- Mass [gram] : 3.0012345e+02
- Center of Gravity [mm]
  - X : 1.8765432e-02
  - Y : 2.5678901e+02
  - Z : 2.8901234e+02
- Inertia Tensor with respect to C1 coordinate frame: [gram * mm<sup>2</sup>]
  - Ixx Ixy Ixz :  1.0987654e+07 -3.4321098e+03 -6.7890123e+03
  - Iyx Iyy Iyz : -3.4321098e+03  7.6543210e+06 -5.1234567e+06
  - Izx Izy Izz : -6.7890123e+03 -5.1234567e+06  4.1234567e+06
- Inertia Tensor at CENTER OF GRAVITY with respect to coordinate frame: [gram * mm<sup>2</sup>]
  - Ixx Ixy Ixz :  3.4321098e+06 -1.4321098e+02 -2.8765432e+03
  - Iyx Iyy Iyz : -1.4321098e+02  2.9876543e+06  2.9876543e+04
  - Izx Izy Izz : -2.8765432e+03  2.9876543e+04  3.9876543e+05

### Joint 6

<!-- ![](/specifications/omx/omx_leader_joint6.png) -->

- Volume [mm<sup>3</sup>] : 9.8765432e+04
- Surface Area [mm<sup>2</sup>] : 7.4321098e+04
- Average Density [gram / mm<sup>3</sup>] : 2.3210987e-03
- Mass [gram] : 2.2901234e+02
- Center of Gravity [mm]
  - X : 1.4321098e-02
  - Y : 2.7890123e+02
  - Z : 3.0123456e+02
- Inertia Tensor with respect to C1 coordinate frame: [gram * mm<sup>2</sup>]
  - Ixx Ixy Ixz :  9.8765432e+06 -2.9876543e+03 -6.1234567e+03
  - Iyx Iyy Iyz : -2.9876543e+03  6.5432109e+06 -4.5678901e+06
  - Izx Izy Izz : -6.1234567e+03 -4.5678901e+06  3.5678901e+06
- Inertia Tensor at CENTER OF GRAVITY with respect to coordinate frame: [gram * mm<sup>2</sup>]
  - Ixx Ixy Ixz :  2.9876543e+06 -1.2109876e+02 -2.5432109e+03
  - Iyx Iyy Iyz : -1.2109876e+02  2.5432109e+06  2.5432109e+04
  - Izx Izy Izz : -2.5432109e+03  2.5432109e+04  3.5432109e+05

 