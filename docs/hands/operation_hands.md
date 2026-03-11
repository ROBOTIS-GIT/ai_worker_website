# Operation Guide

This guide explains how to operate the ROBOTIS Hand.

## Prerequisites

Before running the ROBOTIS Hand, make sure:
- The ROBOTIS Hand is properly set up and powered on
- The ROBOTIS Hand is connected to `/dev/ttyUSB0`

## Running the ROBOTIS Hand

The ROBOTIS Hand controller utilizes the `ros2_control` framework for enhanced flexibility and real-time control. Running the robot requires two separate terminal instances, both accessing the Docker container.

After accessing the ROBOTIS Hand Docker container, start the Zenoh daemon. For convenience, the following alias is provided to simplify the command:
```bash
zenohd
```

In a second terminal, access the same Docker container and launch the ROBOTIS Hand bringup script.
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

(Optional) For visualization, add the option `start_rviz:=true` at the end of the command:
:::tabs key:robot-type
== HX5-D20
```bash
ros2 launch robotis_hand_bringup hx5_d20_right.launch.py start_rviz:=true
```
:::
