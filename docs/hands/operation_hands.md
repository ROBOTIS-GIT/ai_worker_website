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
# For the left hand
ros2 launch robotis_hand_bringup hx5_d20_left.launch.py

# For the right hand
ros2 launch robotis_hand_bringup hx5_d20_right.launch.py
```
**Check Hand Orientation:** You must use the launch file that matches your specific hardware (`hx5_d20_left` or `hx5_d20_right`). If the software configuration does not match the physical hand, the system **will not work**.
:::

Once the command is executed, the system will perform the following steps:
1. **Initial Positioning**: The ROBOTIS Hand moves to its initial position
2. **Topic Subscription**: The node begins subscribing to the input topic

The ROBOTIS Hand controller will now subscribe to the input topic described in the [Software Specifications](/hands/software_hands.html#controller-configuration-joint-mapping).

(Optional) For visualization, add the option `start_rviz:=true` at the end of the command:
:::tabs key:robot-type
== HX5-D20
```bash
# For the left hand
ros2 launch robotis_hand_bringup hx5_d20_left.launch.py start_rviz:=true

# For the right hand
ros2 launch robotis_hand_bringup hx5_d20_right.launch.py start_rviz:=true
```
:::

## Example Hand Motion
This section provides a simple example for running predefined hand motions. It continuously alternates between smooth and quick cycles of finger pinch and grasp motions.

<img src="/specifications/hand/example_hand_motion.gif" alt="ROBOTIS Hand example motion" style="display:block;margin:0 auto;max-width:100%;" />
<p style="text-align: center;"><em>Example hand motion demo.</em></p>

### Run the Example Motion
After launching the **real robot** or **Gazebo bringup**, enter the following command in the new Docker container:

:::tabs key:robot-type
== HX5-D20
```bash
# For the left hand
ros2 run robotis_hand_playground hx5_d20_example_motion_left.py

# For the right hand
ros2 run robotis_hand_playground hx5_d20_example_motion_right.py
```
:::

The example node sends predefined motion sequences as `trajectory_msgs/JointTrajectory` messages.
> When the script starts, it first publishes one dummy trajectory.
> This helps prevent the first command from being ignored by the controller. 

## Keyboard Teleop
Control the ROBOTIS Hand using your keyboard.

### Run the Keyboard Teleop
After launching the **real robot** or **Gazebo bringup**, enter the following command in the new Docker container:
::: warning
In Gazebo, a finger may not move if its `joint_state` is outside the joint limits defined in the URDF.
:::

:::tabs key:robot-type
== HX5-D20

```bash
# For the left hand
ros2 run robotis_hand_teleop hx5_d20_left_teleop.py

# For the right hand
ros2 run robotis_hand_teleop hx5_d20_right_teleop.py
```

**Finger Control**
- `v` / `c` - Thumb close / open
- `q` / `a` - Index close / open
- `w` / `s` - Middle close / open
- `e` / `d` - Ring close / open
- `r` / `f` - Little close / open

**Shift Joint Control**
- `1` - Index (`Joint 5`)
- `2` - Middle (`Joint 9`)
- `3` - Ring (`Joint 13`)
- `4` - Little (`Joint 17`)
- `5` - Thumb (`Joint 1`) 
- `6` - Thumb (`Joint 2`)
- `TAB` - Toggle movement direction

**Hand Control**
- `z` - Open all fingers
- `x` - Close all fingers
- `b` - Print current target positions

### Keyboard Teleop Demo

<img src="/specifications/hand/keyboard_teleop.gif" alt="ROBOTIS Hand keyboard teleop demo" style="display:block;margin:16px auto 0;max-width:100%;" />
<p style="text-align: center;"><em>Keyboard control of individual fingers.</em></p>
:::
