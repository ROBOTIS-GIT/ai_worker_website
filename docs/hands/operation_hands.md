# Operation Guide

This guide explains how to operate the ROBOTIS Hand.

## Prerequisites

Before running the ROBOTIS Hand, make sure:
- The ROBOTIS Hand is properly set up and powered on
- The ROBOTIS Hand is connected to `/dev/ttyUSB0`

## Running the ROBOTIS Hand

The ROBOTIS Hand controller has been constructed to utilize the `ros2_control` framework for enhanced flexibility, modularity, and usability. This updated controller allows for seamless integration with ROS 2-based systems, offering advanced features such as trajectory planning, real-time control, and state feedback.

After accessing the Robotis Hand Docker container, use the following command:
:::tabs key:robot-type
== HX5-D20
```bash
ros2 launch robotis_hand_bringup hx5_d20_right.launch.py
```
:::

This command will sequentially execute the following procedures:
1. Move the ROBOTIS Hand to its initial position
2. Start subscribing to the input topic

The ROBOTIS Hand controller will now subscribe to the input topic described in the [Software Specifications](/hands/software_hands.html#controller-configuration-joint-mapping).

For visualization, add the option `start_rviz:=true` at the end of the command:
:::tabs key:robot-type
== HX5-D20
```bash
ros2 launch robotis_hand_bringup hx5_d20_right.launch.py start_rviz:=true
```
:::
