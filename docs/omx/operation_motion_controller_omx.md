---
prev: 
  text: Setup Guide
  link: /omx/setup_guide_omx
---

# Motion Controller

This guide shows how to run the motion controller from [`robotis_motion_controller`](https://github.com/ROBOTIS-GIT/robotis_motion_controller/tree/feature-qp?tab=readme-ov-file) on OMX.

The default `omx` controller follows a task-space end-effector goal. In practice, it is mainly used with an interactive marker in RViz. The `movej` and `movel` controllers are command-based modes for joint-space and Cartesian-space motion.

## Supported Controllers

- `controller_type:=omx`: Main task-space controller. It tracks one end-effector pose goal and can be used with an RViz interactive marker.
- `controller_type:=movej`: Joint-space command controller. It receives a `JointTrajectory` command and tracks the target joint configuration with QP-based safety constraints.
- `controller_type:=movel`: Cartesian command controller. It receives a `MoveL` command, generates a Cartesian interpolation, and tracks the end-effector pose with QP-based safety constraints.

## Prerequisites

- Complete the hardware and software steps in the **Setup Guide**.
- Clone [`robotis_motion_controller`](https://github.com/ROBOTIS-GIT/robotis_motion_controller/tree/feature-qp?tab=readme-ov-file) into your workspace and follow the installation steps in the README.
- Ensure OMX is on a stable surface with enough clearance for arm motion.

## Bring Up OMX

Start the OMX follower bringup first. Run this on the `user PC`:

```bash
cd open_manipulator/docker
./container.sh enter
ros2 launch open_manipulator_bringup omx_f_follower_ai.launch.py
```

Keep that terminal running. Open a new terminal for the motion controller commands below.

## Launch the Default OMX Controller

Open two new terminals and run the same environment setup in both:

```bash
cd open_manipulator/docker
./container.sh enter
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

1. In the first terminal, launch the task-space controller with the interactive marker enabled:
   ```bash
   ros2 launch motion_controller_ros omx_controller.launch.py controller_type:=omx start_interactive_marker:=true
   ```
2. In another terminal, start RViz if it is not already running:
   ```bash
   rviz2
   ```
3. In RViz, set the fixed frame to `link0` and add these displays:
   - `RobotModel`
   - `TF`
   - `InteractiveMarkers`

When `start_interactive_marker:=true`, the launch file starts one interactive marker named `omx_goal_marker` and publishes its pose to `/eef_goal_pose`.

## Operating the OMX Controller from RViz

1. Launch the `omx` controller with `start_interactive_marker:=true`.
2. Open RViz and confirm the robot model and marker are visible.
3. Move the marker gradually to send a new end-effector goal pose.

![marker_control](/simulation/omx/omx_f_marker_control.gif)

::: tip
The default `omx` controller captures the current end-effector pose on startup and then follows the marker goal. Small, continuous marker motions usually give the smoothest motion.
:::

## Other Controllers

### MoveJ Controller

This controller is used to execute a joint-space target command:

```bash
ros2 launch motion_controller_ros omx_controller.launch.py controller_type:=movej
```

It subscribes to `~/movej` under the node namespace, which becomes `/omx_movej_controller/movej` with the default node name.

Example command:

```bash
ros2 topic pub --once /omx_movej_controller/movej trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5'],
  points: [
    {
      positions: [0.0, -0.5, 0.8, 0.0, 0.3],
      time_from_start: {sec: 3, nanosec: 0}
    }
  ]
}"
```

The controller interpolates from the current joint state to the requested joint configuration while applying QP-based constraint handling.

### MoveL Controller

This controller is used to execute a Cartesian end-effector command:

```bash
ros2 launch motion_controller_ros omx_controller.launch.py controller_type:=movel
```

It subscribes to `~/movel` under the node namespace, which becomes `/omx_movel_controller/movel` with the default node name.

Example command:

```bash
ros2 topic pub --once /omx_movel_controller/movel robotis_interfaces/msg/MoveL "{
  pose: {
    pose: {
      position: {x: 0.20, y: 0.00, z: 0.18},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  },
  time_from_start: {sec: 3, nanosec: 0}
}"
```

`movel` uses the `time_from_start` field as the interpolation duration for the Cartesian motion.

![movel_control](/simulation/omx/omx_f_movel_control.gif)

## Useful Launch Arguments

- `controller_type`: Selects `omx`, `movej`, or `movel`.
- `start_interactive_marker`: Starts the interactive marker when `controller_type:=omx`.
- `base_frame`: Base frame for control and marker visualization. Default: `link0`.
- `controlled_link`: End-effector link tracked by the controller. Default: `end_effector_link`.
- `config_file`: Path to `motion_controller_ros/config/omx_config.yaml`.
- `marker_goal_topic`: Goal topic published by the interactive marker. Default: `/eef_goal_pose`.
- `marker_scale`: Marker size in RViz.
- `urdf_path` and `srdf_path`: Override the OMX robot model files.

## Key Topics

| Interface | Type | Purpose |
| --- | --- | --- |
| `/joint_states` | Topic | Measured OMX joint state required by all controller modes |
| `/eef_goal_pose` | Topic | Interactive-marker goal pose for `controller_type:=omx` |
| `/leader/joint_trajectory` | Topic | Output joint trajectory command published by the controllers |
| `/omx_controller/current_pose` | Topic | Current end-effector pose published by the default controller |
| `/omx_controller/controller_error` | Topic | Controller error status for the default controller |
| `/omx_movej_controller/movej` | Topic | Joint-space command topic for `movej` |
| `/omx_movel_controller/movel` | Topic | Cartesian command topic for `movel` |

## Troubleshooting

- If the marker does not appear in RViz after setting `start_interactive_marker:=true`, check that the `InteractiveMarkers` display is added and that the `base_frame` is set correctly.
- If the controller does not move, check the terminal log to see whether `/joint_states` is updating and whether the controller is receiving goal commands.
- If `movej` does not respond, confirm the published joint names match the OMX model joints.
- If `movel` does not respond, confirm the `MoveL` command is being published to `/omx_movel_controller/movel`.

## Controller Parameters

The main parameters live in `motion_controller_ros/config/omx_config.yaml`. The file is divided by controller name, so you usually tune only the block that matches the controller mode you are running.

### `omx_controller`

- `control_frequency`, `time_step`: Main control loop speed.
- `trajectory_time`: Time field used when publishing output joint trajectories.
- `kp_position`, `kp_orientation`: Task-space tracking gains for end-effector position and orientation.
- `weight_task_position`, `weight_task_orientation`: Relative importance of position and orientation tracking in the QP solve.
- `weight_damping`: Regularization term that discourages unnecessarily large joint velocities.
- `collision_buffer`, `collision_safe_distance`: Safety margins used for collision avoidance.
- `slack_penalty`: Cost applied when the solver must relax constraints. Larger values enforce constraints more strictly.
- `cbf_alpha`: Responsiveness of the barrier constraint. Larger values make the constraint act more like a full brake near a cliff, affecting the control input later and more abruptly as the motion gets very close to the constraint boundary.
- `base_frame`, `controlled_link`: Base frame and end-effector link used by the controller.
- `joint_states_topic`: Source of measured robot state.
- `joint_command_topic`: Output joint trajectory command topic.
- `marker_goal_topic`: Goal pose topic received from the interactive marker.
- `ee_pose_topic`: Published current end-effector pose topic.
- `controller_error_topic`: Published controller error topic.

### `omx_movej_controller`

- `control_frequency`, `time_step`: Main control loop speed.
- `trajectory_time`: Time field used when publishing output joint trajectories.
- `kp_joint`: Joint-space tracking gain for the `movej` target.
- `weight_joint_tracking`: Relative importance of joint tracking in the QP solve.
- `weight_damping`: Regularization term that discourages unnecessarily large joint velocities.
- `collision_buffer`, `collision_safe_distance`: Safety margins used for collision avoidance.
- `slack_penalty`: Cost applied when the solver must relax constraints. Larger values enforce constraints more strictly.
- `cbf_alpha`: Responsiveness of the barrier constraint. Larger values make the constraint act more like a full brake near a cliff, affecting the control input later and more abruptly as the motion gets very close to the constraint boundary.
- `base_frame`, `controlled_link`: Base frame and end-effector link used for state reporting.
- `joint_states_topic`: Source of measured robot state.
- `joint_command_topic`: Output joint trajectory command topic.
- `movej_topic`: Input topic for the `movej` command.
- `ee_pose_topic`: Published current end-effector pose topic.
- `controller_error_topic`: Published controller error topic.

### `omx_movel_controller`

- `control_frequency`, `time_step`: Main control loop speed.
- `trajectory_time`: Time field used when publishing output joint trajectories.
- `kp_position`, `kp_orientation`: Cartesian tracking gains for end-effector position and orientation.
- `weight_task_position`, `weight_task_orientation`: Relative importance of position and orientation tracking in the QP solve.
- `weight_damping`: Regularization term that discourages unnecessarily large joint velocities.
- `collision_buffer`, `collision_safe_distance`: Safety margins used for collision avoidance.
- `slack_penalty`: Cost applied when the solver must relax constraints. Larger values enforce constraints more strictly.
- `cbf_alpha`: Responsiveness of the barrier constraint. Larger values make the constraint act more like a full brake near a cliff, affecting the control input later and more abruptly as the motion gets very close to the constraint boundary.
- `base_frame`, `controlled_link`: Base frame and end-effector link used by the controller.
- `joint_states_topic`: Source of measured robot state.
- `joint_command_topic`: Output joint trajectory command topic.
- `movel_topic`: Input topic for the `movel` command.
- `ee_pose_topic`: Published current end-effector pose topic.
- `controller_error_topic`: Published controller error topic.

## Safety and Usage Tips

- Keep hands and cables clear before sending marker, `movej`, or `movel` commands.
- Start with small motions first to confirm the robot model, frames, and topics are configured correctly.
- Stop the controller immediately with `Ctrl+C` if it is safe to do so and the robot behaves unexpectedly.
