# Cyclo Motion Controller

This guide shows how to run the Cyclo Motion Controller from [`cyclo_control`](https://github.com/ROBOTIS-GIT/cyclo_control) on AI Worker.

`cyclo_motion_controller` is the software layer that acts like the robot's motion interpreter. You give it an easier command, such as an end-effector target pose or a joint target, and it computes the joint trajectories that the real robot should follow.
![ai_worker_motion_controller](/simulation/ai_worker/aiw_motion_controller.png)

Its QP(Quadratic Programming)-based controller is especially useful because it does not only track the command, but also tries to keep the motion safe at the same time by considering limits and constraints such as **joint range**, **joint velocity**, and **self-collision avoidance**. In practice, you use it when you want to command the robot by target poses or joint commands while still relying on the controller to generate safe motion.
![motion_controller_safety](/simulation/ai_worker/ffw_sg2_collision.gif)

## Supported Controllers

- `controller_type:=movel`**(Default)**: Generates interpolated arm motion from the current hand pose to the requested goal pose.
- `controller_type:=movej`: Receives raw joint trajectories and republishes safer filtered trajectories for the robot.
<!-- - `controller_type:=vr`: Continuously tracks live right and left task-space pose references from VR input.
- `controller_type:=leader`: Converts leader-side motion into follower pose references and tracks them on the robot. -->

## Understanding `MoveL` and `MoveJ`

`MoveL` and `MoveJ` are not just "send one target and hope the robot gets there." They are motion commands that tell the controller to generate an interpolated motion from the robot's current state to the requested goal over a given time.

- `MoveL` means "move in a Cartesian line." You command a target hand pose and an interpolation time, and the controller generates a smooth motion from the current end-effector pose toward that goal.
![Default_vs_movel](/simulation/ai_worker/default_vs_movel.gif)
- `MoveJ` means "move in joint space." You command target joint values and an interpolation time, and the controller generates a smooth motion from the current joint configuration toward those values. In that sense, it follows the same interpolation idea as `MoveL`, but in joint space instead of Cartesian space.

This is different from a simple pose or joint command that only describes the desired target state. `MoveL` and `MoveJ` are higher-level motion commands because they also imply a transition from the current state to the goal, including how long that transition should take.

## Prerequisites

- Complete the hardware and software steps in the **Setup Guide**.
- In the default ai_worker Docker container environment, clone [`cyclo_control`](https://github.com/ROBOTIS-GIT/cyclo_control) into `~/ros2_ws/src`, then follow the installation steps in the README.
- Ensure the robot is on level ground with enough clearance to move both arms safely.
- Make sure the emergency stop button is always within reach during operation.
- Connect to the robot PC and make sure the battery is charged.

## Bring Up the Robot

Start the AI Worker follower bringup first. Run this on the `robot PC`:

```bash
cd ~/ai_worker
./docker/container.sh enter
ros2 launch ffw_bringup ffw_sg2_follower_ai.launch.py
```

Keep that terminal running. Open a new terminal for the motion controller commands below.

## Launch MoveL Controller

Open two new terminals and run the same environment setup in both:

```bash
cd ~/ai_worker
./docker/container.sh enter
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

1. In the first terminal, launch the default AI Worker controller:
   ```bash
   ros2 launch cyclo_motion_controller_ros ai_worker_controller.launch.py controller_type:=movel
   ```
2. If you want to use marker-based control in RViz, relaunch it with `start_interactive_marker:=true`:
   ```bash
   ros2 launch cyclo_motion_controller_ros ai_worker_controller.launch.py controller_type:=movel start_interactive_marker:=true
   ```
3. If you use marker-based control, start RViz if it is not already running:
   ```bash
   rviz2
   ```
4. In RViz, set the fixed frame to `base_link` and add these displays:
   - `RobotModel`
   - `TF`
   - `InteractiveMarkers`

When `start_interactive_marker:=true`, the launch file starts two interactive markers:

- `right_goal_marker`, which publishes `MoveL` commands to `/r_goal_move`
- `left_goal_marker`, which publishes `MoveL` commands to `/l_goal_move`

You can use the `movel` controller in two ways:

- Move the interactive markers in RViz to send new right and left Cartesian goals.
- Publish `MoveL` commands directly to `/r_goal_move` and `/l_goal_move`.

![aiw_marker](/simulation/ai_worker/aiw_marker.gif)

:::: tip
The default `movel` controller plus interactive marker is usually the easiest way to test the motion controller because the interactive markers send `MoveL` commands directly and the controller handles inverse kinematics, constraint handling, and trajectory publishing.
::::

```bash
ros2 topic pub --once /r_goal_move robotis_interfaces/msg/MoveL "{
  pose: {
    header: {frame_id: 'base_link'},
    pose: {
      position: {x: 0.35, y: -0.20, z: 0.85},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  },
  time_from_start: {sec: 2, nanosec: 0}
}"
```
```bash
ros2 topic pub --once /l_goal_move robotis_interfaces/msg/MoveL "{
  pose: {
    header: {frame_id: 'base_link'},
    pose: {
      position: {x: 0.35, y: 0.20, z: 0.85},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  },
  time_from_start: {sec: 2, nanosec: 0}
}"
```

`movel` uses the `time_from_start` field as the interpolation duration for the Cartesian motion.

![aiw_movel](/simulation/ai_worker/aiw_movel.gif)

## Launch MoveJ Controller

This controller is used to apply a safety filter to raw joint trajectory commands published for the follower arms:

```bash
ros2 launch cyclo_motion_controller_ros ai_worker_controller.launch.py controller_type:=movej
```

It subscribes to:

- `/leader/joint_trajectory_command_broadcaster_right/raw_joint_trajectory`
- `/leader/joint_trajectory_command_broadcaster_left/raw_joint_trajectory`

and republishes filtered outputs to:

- `/leader/joint_trajectory_command_broadcaster_right/joint_trajectory`
- `/leader/joint_trajectory_command_broadcaster_left/joint_trajectory`

Example commands:

```bash
ros2 topic pub --once /leader/joint_trajectory_command_broadcaster_right/raw_joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['arm_r_joint1', 'arm_r_joint2', 'arm_r_joint3', 'arm_r_joint4', 'arm_r_joint5', 'arm_r_joint6', 'arm_r_joint7', 'gripper_r_joint1'],
  points: [
    {
      positions: [0.3, -0.2, 0.1, 0.0, 0.2, -0.1, 0.0, 0.02],
      time_from_start: {sec: 3, nanosec: 0}
    }
  ]
}"
```

```bash
ros2 topic pub --once /leader/joint_trajectory_command_broadcaster_left/raw_joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['arm_l_joint1', 'arm_l_joint2', 'arm_l_joint3', 'arm_l_joint4', 'arm_l_joint5', 'arm_l_joint6', 'arm_l_joint7', 'gripper_l_joint1'],
  points: [
    {
      positions: [-0.3, 0.2, -0.1, 0.0, -0.2, 0.1, 0.0, 0.02],
      time_from_start: {sec: 3, nanosec: 0}
    }
  ]
}"
```

![aiw_movel](/simulation/ai_worker/aiw_movej.gif)

<!-- ## Launch VR Controller

This controller is used when VR teleoperation provides task-space pose references for both arms:

```bash
ros2 launch robotis_motion_controller_ros ai_worker_controller.launch.py controller_type:=vr
```

It tracks:

- `/r_goal_pose`, `/l_goal_pose`
- `/r_elbow_pose`, `/l_elbow_pose`

and publishes follower arm trajectories to the standard right and left arm trajectory topics.

The `vr` controller also runs `reference_checker_node`, waits for `/reactivate`, checks startup alignment, and then enables motion with a delayed activation sequence.

```bash
ros2 service call /reactivate std_srvs/srv/Trigger "{}"
```

![vr_control](/simulation/ai_worker/ffw_sg2_vr_teleop.gif)

## Launch Leader-Follower Retargeting Controller

This controller is used to control the follower in task space from the leader configuration through retargeting:

```bash
ros2 launch robotis_motion_controller_ros ai_worker_controller.launch.py controller_type:=leader
```

This launch starts:

- `leader_controller_node`
- `vr_controller_node`

The leader controller performs forward kinematics from the leader joint trajectories, publishes follower-side pose references such as `/r_goal_pose` and `/l_goal_pose`, and the VR controller tracks those references on the robot. In this mode, the leader controller also requests `/reactivate` automatically when valid leader commands begin arriving.

![retargeting_control](/simulation/ai_worker/ffw_sg2_retargeting_control.gif) -->

## Launch Arguments

- `controller_type`: Selects `movel`, `movej`, `vr`, or `leader`.
- `start_interactive_marker`: Starts interactive markers for the `movel` controller.
- `base_frame`: Frame used for the interactive markers. Default: `base_link`.
- `right_controlled_link`, `left_controlled_link`: Controlled link names for the right and left markers.
- `right_movel_topic`, `left_movel_topic`: `MoveL` topics published by the right and left markers.
- `reactivate_service`: Reactivation service used by the `vr` and `leader` modes. Default: `/reactivate`.
- `config_file`: Path to controller configuration yaml file.
- `follower_urdf_path`, `follower_srdf_path`: Override the follower robot model files.
- `leader_urdf_path`: Override the leader robot model file.

## Key Topics and Services

### Common

- `/joint_states`: Measured robot state required by all controller modes
- `/reactivate`: Service used to arm or re-arm the `vr` controller

### MoveL

- `/r_goal_move`: Right arm `MoveL` command topic
- `/l_goal_move`: Left arm `MoveL` command topic
- `/r_gripper_pose`: Published current right gripper pose
- `/l_gripper_pose`: Published current left gripper pose

### MoveJ

- `/leader/joint_trajectory_command_broadcaster_right/raw_joint_trajectory`: Raw right-arm moveJ input
- `/leader/joint_trajectory_command_broadcaster_left/raw_joint_trajectory`: Raw left-arm moveJ input
- `/leader/joint_trajectory_command_broadcaster_right/joint_trajectory`: Filtered or tracked right-arm output trajectory
- `/leader/joint_trajectory_command_broadcaster_left/joint_trajectory`: Filtered or tracked left-arm output trajectory

<!-- ### VR and Leader

- `/r_goal_pose`: Right task-space goal pose
- `/l_goal_pose`: Left task-space goal pose
- `/r_elbow_pose`: Right elbow guidance topic
- `/l_elbow_pose`: Left elbow guidance topic
- `/reference_diverged`: Reference jump event from the checker node -->

## Troubleshooting

- If the markers do not appear in RViz after setting `start_interactive_marker:=true`, check that the `InteractiveMarkers` display is added and that the `base_frame` is set correctly.(default set to `base_link`)
- If the `movel` controller does not move, check the terminal log to see whether `/joint_states` is updating and whether `MoveL` commands are arriving on `/r_goal_move` and `/l_goal_move`.
- If `movej` does not respond, confirm the raw trajectory topics are receiving commands and that each command includes correct joint names.
<!-- - If `vr` does not move, check the terminal log for startup mismatch or reference divergence, then call `/reactivate` again after the goal poses are near the current gripper poses.
- If `leader` does not respond, confirm the leader raw trajectory topics are active and that the controller is publishing `/r_goal_pose` and `/l_goal_pose`. -->

## Controller Parameters

The main parameters live in `cyclo_motion_controller_ros/config/ai_worker_config.yaml`. The file is divided by controller name, so you usually tune only the block that matches the controller mode you are running.

### `ai_worker_movel_controller`

- `control_frequency`, `time_step`: Main control loop speed.
- `trajectory_time`: Time field used when publishing output joint trajectories.
- `kp_position`, `kp_orientation`: Cartesian tracking gains for end-effector position and orientation.
- `weight_position`, `weight_orientation`: Relative importance of position and orientation tracking in the QP solve.
- `weight_damping`: Regularization term that discourages unnecessarily large joint velocities.
- `collision_buffer`, `collision_safe_distance`: Safety margins used for collision avoidance.
- `slack_penalty`: Cost applied when the solver must relax constraints. Larger values enforce constraints more strictly.
- `cbf_alpha`: Responsiveness of the barrier constraint. Larger values make the constraint act more like a full brake near a cliff, affecting the control input later and more abruptly as the motion gets very close to the constraint boundary.
- `joint_states_topic`: Source of measured robot state.
- `right_movel_topic`, `left_movel_topic`: Right and left `MoveL` command input topics.
- `right_traj_topic`, `left_traj_topic`: Right and left arm trajectory output topics.
- `lift_topic`, `lift_vel_bound`: Lift command path settings. When `lift_vel_bound` is `0`, the controller does not consider lift joint motion.
- `r_gripper_pose_topic`, `l_gripper_pose_topic`: Published current gripper pose topics.
- `r_gripper_name`, `l_gripper_name`: Link names used to compute the gripper pose.
- `right_gripper_joint`, `left_gripper_joint`: Gripper joint names preserved in published trajectories.
- `controller_error_topic`: Published controller error topic.

### `ai_worker_movej_controller`

- `control_frequency`, `time_step`: Main control loop speed.
- `trajectory_time`: Time field used when publishing output joint trajectories.
- `kp_joint`: Joint-space tracking gain for the moveJ target.
- `weight_tracking`: Relative importance of joint tracking in the QP solve.
- `weight_damping`: Regularization term that discourages unnecessarily large joint velocities.
- `collision_buffer`, `collision_safe_distance`: Safety margins used for collision avoidance.
- `slack_penalty`: Cost applied when the solver must relax constraints. Larger values enforce constraints more strictly.
- `cbf_alpha`: Responsiveness of the barrier constraint. Larger values make the constraint act more like a full brake near a cliff, affecting the control input later and more abruptly as the motion gets very close to the constraint boundary.
- `joint_states_topic`: Source of measured robot state.
- `right_traj_topic`, `left_traj_topic`: Raw right and left joint trajectory input topics.
- `right_traj_filtered_topic`, `left_traj_filtered_topic`: Filtered right and left output trajectory topics.
- `right_gripper_joint`, `left_gripper_joint`: Gripper joint names preserved from the input command.

<!-- ### `vr_controller`

- `control_frequency`, `time_step`: Main control loop speed.
- `trajectory_time`: Time field used when publishing output joint trajectories.
- `kp_position`, `kp_orientation`: Task-space tracking gains for gripper position and orientation.
- `weight_position`, `weight_orientation`, `weight_elbow_position`: Relative importance of end-effector and elbow tasks in the QP solve.
- `weight_damping`: Regularization term that discourages unnecessarily large joint velocities.
- `collision_buffer`, `collision_safe_distance`: Safety margins used for collision avoidance.
- `slack_penalty`: Cost applied when the solver must relax constraints. Larger values enforce constraints more strictly.
- `cbf_alpha`: Responsiveness of the barrier constraint. Larger values make the constraint act more like a full brake near a cliff, affecting the control input later and more abruptly as the motion gets very close to the constraint boundary.
- `reactivate_service`: Service used to arm or re-arm the controller.
- `joint_states_topic`: Source of measured robot state.
- `right_traj_topic`, `left_traj_topic`: Output arm trajectory topics.
- `right_raw_traj_topic`, `left_raw_traj_topic`: Raw leader trajectory topics used to preserve gripper commands.
- `lift_topic`, `lift_vel_bound`: Lift command path settings. When `lift_vel_bound` is `0`, the controller does not consider lift joint motion.
- `r_goal_pose_topic`, `l_goal_pose_topic`: Main right and left end-effector goal pose topics.
- `r_elbow_pose_topic`, `l_elbow_pose_topic`: Elbow reference topics used to shape arm posture.
- `r_gripper_pose_topic`, `l_gripper_pose_topic`: Published current gripper pose topics.
- `startup_ref_pos_threshold`, `startup_ref_ori_threshold_deg`: Startup alignment thresholds before control is enabled.

### `leader_controller`

- `control_frequency`: Forward-kinematics and retargeting update rate.
- `joint_states_topic`: Source of measured follower state, used mainly for the lift joint.
- `right_traj_topic`, `left_traj_topic`: Raw leader joint trajectory input topics.
- `reactivate_service`: Service used to arm the follower-side `vr` controller automatically when leader commands begin.
- `command_timeout`: Timeout for detecting that the leader command stream has stopped.
- `r_goal_pose_topic`, `l_goal_pose_topic`: Right and left goal pose topics generated for the follower.
- `r_elbow_pose_topic`, `l_elbow_pose_topic`: Right and left elbow reference topics generated for the follower.
- `base_frame_id`: Base frame used for published pose references.
- `r_gripper_name`, `l_gripper_name`, `r_elbow_name`, `l_elbow_name`: Link names used for forward kinematics and retargeting.
- `lift_joint_name`, `model_lift_joint_name`: Lift joint names used to map the leader and follower models correctly.

### `reference_checker`

- `ref_pos_jump_threshold`: Maximum allowed sudden position jump in the incoming reference.
- `ref_ori_jump_threshold_deg`: Maximum allowed sudden orientation jump in the incoming reference.
- `r_goal_pose_topic`, `l_goal_pose_topic`: Right and left goal pose topics monitored for discontinuities. -->

## Safety and Usage Tips

- Keep people, cables, and nearby objects clear before sending commands.
- Start with small motions first to confirm the model, frames, and topics are configured correctly.
<!-- - Re-arm the `vr` controller any time you intentionally reset references or recover from a reference divergence event. -->
- Always stay within reach of the emergency stop during operation.
- Stop the controller immediately with `Ctrl+C` if it is safe to do so and the robot behaves unexpectedly.
