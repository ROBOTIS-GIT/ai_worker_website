# Hardware Setup

This guide covers the physical setup steps required before operating AI Worker, including power-on, port identification, and Remote E-STOP usage.

::: info
- The term `Follower` refers to the body of the AI WORKER robot.
- Make sure to check which robot model you are using before following the steps below.
:::

## Power On and Check Ports

:::tabs key:robot-type
== FFW-BG2
#### Powering On the FFW_BG2 Follower
![Back of the BG2 Base](/quick_start_guide/ai_worker/back_of_the_BG2_base.png)
1. Toggle the `Power Supply Switch` to the right.
2. Insert the `Key Switch` and turn it to the 12 o'clock position.
3. Press and hold the `Power Button` for 3 seconds. When you hear a beep, the system is powered on. You should see the robot's head light up at this point.

#### Hardware Ports (FFW_BG2 Follower)
![Back of the BG2 Body](/quick_start_guide/ai_worker/back_of_the_BG2_body.png)
The back of the Follower body includes several ports for system access and external connections:
- `LAN Port`: Used to access the robot PC via SSH or remote desktop.
- `USB Ports`: For connecting peripherals such as a keyboard, mouse, or USB drive.
- `HDMI Port`: Allows direct video output for connecting a monitor.

== FFW-SG2
#### Powering On the FFW_SG2 Follower
![Back of the SG2 Base](/quick_start_guide/ai_worker/back_of_the_SG2_base.png)
1. Insert the `Key Switch` and turn it to the 2 o'clock position.
2. Press and hold the `Power Button` for 3 seconds. When you hear a beep, the system is powered on. You should see the robot's head light up at this point.

#### Hardware Ports (FFW_SG2 Follower)
![Back of the SG2 body](/quick_start_guide/ai_worker/back_of_the_SG2_body.png)
The back of the Follower body includes several ports for system access and external connections:
- `WAN Port`: Used to connect the robot to an external network or the internet.
- `LAN Port`: Used to access the robot PC via SSH or remote desktop.
- `USB Ports`: For connecting peripherals such as a keyboard, mouse, or USB drive.
- `HDMI Port`: Allows direct video output for connecting a monitor.
- `Charge Port`: Used for battery charging.
:::

## Remote E-STOP Usage

![remote_e_stop](/quick_start_guide/ai_worker/remote_e_stop.png)

The Remote E-STOP device is the safety stop controller for AI Worker. It is used to perform an emergency stop and to release the initial safety lock.

Press the red mushroom button circled above to activate the E-STOP.

To release it, rotate the button clockwise, then press the **A** button.

The other buttons have no function.

::: warning
When AI WORKER is powered on for the first time, it remains in a **torque-off** state.
To enable communication with the DYNAMIXELs, you must press the **A** button on the Remote E-STOP.
When the safety lock is released, you will hear a beep sound.
:::

## Optional: ROBOTIS Hand Installation
If your application requires a multi-functional grip, you can replace the `RH-P12-RN` gripper with a `ROBOTIS Hand`. Follow these steps to perform the exchange:

1. Remove the `RH-P12-RN` gripper: Carefully unscrew the mounting bolts and disconnect the communication cable from the current gripper.

![hand_mount_bracket](/quick_start_guide/ai_worker/hand_mount_bracket.png)

2. Reorient the Mounting Bracket: Rotate the `FRP42-A110K` bracket by 90°. Ensure the bracket alignment matches the orientation shown in the image above.

<img src="/quick_start_guide/ai_worker/hand_cable_port.png" width="350"/>

3. Connect the Communication Cable: Plug the `4-pin JST cable` into the port located on the `ROBOTIS Hand`. Ensure the connector clicks into place to prevent signal loss.

4. Secure the `ROBOTIS Hand`: Align the hand with the rotated `FRP42-A110K` bracket and tighten the mounting screws.

<img src="/quick_start_guide/ai_worker/hand_attached.jpg" width="350"/>

The physical installation is now finished.
