# Operation Guide

This guide explains how to operate the ROBOTIS HX Hand.

## Prerequisites

Before running the HX Hand, make sure:
- The ROBOTIS HX Hand is properly set up and powered on
- The ROBOTIS HX Hand is connected to `/dev/ttyUSB0`

## Running the HX

The HX controller has been constructed to utilize the `ros2_control` framework for enhanced flexibility, modularity, and usability. This updated controller allows for seamless integration with ROS 2-based systems, offering advanced features such as trajectory planning, real-time control, and state feedback.

After accessing the ROBOTIS Hand Docker container, use the following command:
```bash
ros2 launch robotis_hand_bringup hx5_d20_right.launch.py
```

⚠️ **NOTE:**
Use the appropriate launch file depending on the hand type - `hx5_d20_left` or `hx5_d20_right`

This command will sequentially execute the following procedures:
1. Move the HX to its initial position
2. Start Follower mode 

The HX controller will now subscribe to the input topic described in the [Software Specifications](/hands/software_hands.html).

For visualization, add the option `start_rviz:=true` at the end of the command:
```bash
ros2 launch robotis_hand_bringup hx5_d20_right.launch.py start_rviz:=true
```
