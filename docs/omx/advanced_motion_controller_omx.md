# Cyclo Motion Controller

This guide shows how to run the Cyclo Motion Controller from [`cyclo_control`](https://github.com/ROBOTIS-GIT/cyclo_control) on OMX.

`cyclo_motion_controller` is the software layer that acts like the robot's motion interpreter. You give it an easier command, such as an end-effector target pose or a joint target, and it computes the joint trajectories that the real robot should follow.
![omx_motion_controller](/simulation/omx/omx_motion_controller.png)

Its QP(Quadratic Programming)-based controller is especially useful because it does not only track the command, but also tries to keep the motion safe at the same time by considering limits and constraints such as **joint range**, **joint velocity**, and **self-collision avoidance**. In practice, you use it when you want to command the robot by target poses or joint commands while still relying on the controller to generate safe motion.
![omx_collision](/simulation/omx/omx_collision.gif)

## Supported Controllers

- `controller_type:=movel`**(Default)**: Generates interpolated arm motion from the current hand pose to the requested goal pose.
- `controller_type:=movej`: Generates interpolated arm motion from the current joint configuration to the requested joint goal.

## Understanding `MoveL` and `MoveJ`

`MoveL` and `MoveJ` are not just "send one target and hope the robot gets there." They are motion commands that tell the controller to generate an interpolated motion from the robot's current state to the requested goal over a given time.

- `MoveL` means "move in a Cartesian line." You command a target hand pose and an interpolation time, and the controller generates a smooth motion from the current end-effector pose toward that goal.
![default_vs_movel_omx](/simulation/omx/default_vs_movel_omx.gif)
- `MoveJ` means "move in joint space." You command target joint values and an interpolation time, and the controller generates a smooth motion from the current joint configuration toward those values. In that sense, it follows the same interpolation idea as `MoveL`, but in joint space instead of Cartesian space.

This is different from a simple pose or joint command that only describes the desired target state. `MoveL` and `MoveJ` are higher-level motion commands because they also imply a transition from the current state to the goal, including how long that transition should take.

## Prerequisites

- Complete the hardware and software steps in the **Setup Guide**.
- In the default open_manipulator Docker container environment, clone [`cyclo_control`](https://github.com/ROBOTIS-GIT/cyclo_control) into `~/ros2_ws/src`, then follow the installation steps in the README.
- Ensure OMX is on a stable surface with enough clearance for arm motion.

## Bring Up OMX

Start the OMX follower bringup first. Run this on the `user PC`:

```bash
cd open_manipulator/docker
./container.sh enter
ros2 launch open_manipulator_bringup omx_f_follower_ai.launch.py
```

Keep that terminal running. Open a new terminal for the motion controller commands below.

## Launch MoveL Controller

Open two new terminals and run the same environment setup in both:

```bash
cd open_manipulator/docker
./container.sh enter
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

1. In the first terminal, launch the default OMX Motion Controller.
   ```bash
   ros2 launch cyclo_motion_controller_ros omx_controller.launch.py
   ```
2. If you want to use marker-based control in RViz, relaunch it with `start_interactive_marker:=true`:
   ```bash
   ros2 launch cyclo_motion_controller_ros omx_controller.launch.py start_interactive_marker:=true
   ```
3. If you use marker-based control, start RViz if it is not already running:
   ```bash
   rviz2
   ```
4. In RViz, set the fixed frame to `link0` and add these displays:
   - `RobotModel`
   - `TF`
   - `InteractiveMarkers`

When `start_interactive_marker:=true`, the launch file starts one interactive marker named `omx_goal_marker`. Marker publishes `MoveL` commands directly to `/omx_movel_controller/movel`.

You can use the `movel` controller in two ways:

- Move the interactive marker in RViz to send a new Cartesian goal.
- Publish a `MoveL` command directly to `/omx_movel_controller/movel`.

![marker_control](/simulation/omx/omx_f_marker_control.gif)

::: tip
The default `movel` controller is usually the easiest way for beginners to test the motion controller because the interactive marker sends `MoveL` commands directly and the controller handles the interpolation and safe joint motion.
:::

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

## Launch MoveJ Controller

This controller is used to execute a joint-space target command:

```bash
ros2 launch cyclo_motion_controller_ros omx_controller.launch.py controller_type:=movej
```

It subscribes to `~/movej` under the node namespace, which becomes `/omx_movej_controller/movej` with the default node name.

Example command:

```bash
ros2 topic pub --once /omx_movej_controller/movej trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'gripper_joint_1'],
  points: [
    {
      positions: [0.0, -0.5, 0.8, 0.0, 0.3, 0.02],
      time_from_start: {sec: 3, nanosec: 0}
    }
  ]
}"
```

The controller interpolates from the current joint state to the requested joint configuration while applying QP-based constraint handling.

![movej_control](/simulation/omx/omx_movej.gif)

## Launch Arguments

- `controller_type`: Selects `movej` or `movel`. Default: `movel`.
- `start_interactive_marker`: Starts the interactive marker when `controller_type:=movel`.
- `base_frame`: Base frame for control and marker visualization. Default: `link0`.
- `controlled_link`: End-effector link tracked by the controller. Default: `end_effector_link`.
- `config_file`: Path to controller configuration yaml file.
- `marker_goal_topic`: `MoveL` topic published by the interactive marker. Default: `/omx_movel_controller/movel`.
- `marker_scale`: Marker size in RViz.
- `urdf_path` and `srdf_path`: Override the OMX robot model files.

## Key Topics

### Common

- `/joint_states`: Measured OMX joint state required by all controller modes
- `/leader/joint_trajectory`: Output joint trajectory command published by the controllers

### MoveL

- `/omx_movel_controller/movel`: Cartesian `MoveL` input for the default controller
- `/omx_movel_controller/current_pose`: Current end-effector pose published by the default controller
- `/omx_movel_controller/controller_error`: Controller error status for the default controller

### MoveJ

- `/omx_movej_controller/movej`: Joint-space command topic for `movej`

## Troubleshooting

- If the marker does not appear in RViz after setting `start_interactive_marker:=true`, check that the `InteractiveMarkers` display is added and that the `base_frame` is set correctly.(default set to `link0`)
- If the `movel` controller does not move, check the terminal log to see whether `/joint_states` is updating and whether `MoveL` commands are arriving on `/omx_movel_controller/movel`.
- If `movej` does not respond, confirm the published joint names match the OMX model joints.

## Controller Parameters

The main parameters live in `cyclo_motion_controller_ros/config/omx_config.yaml`. The file is divided by controller name, so you usually tune only the block that matches the controller mode you are running.

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
- `movel_topic`: Input topic for the `MoveL` command.
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

## Safety and Usage Tips

- Keep hands and cables clear before sending marker, `movej`, or `movel` commands.
- Start with small motions first to confirm the robot model, frames, and topics are configured correctly.
- Stop the controller immediately with `Ctrl+C` if it is safe to do so and the robot behaves unexpectedly.
