# Motion Controller

This guide shows how to run the motion controller from [`robotis_motion_controller`](https://github.com/ROBOTIS-GIT/robotis_motion_controller) on AI Worker.

The default `ai_worker` controller mainly follows task-space pose references on the follower robot. In practice, it is used either with VR teleoperation inputs or with RViz interactive markers that publish end-effector goals.

![vr_control](/simulation/ai_worker/laserscan.png)

If a leader device is available, you can instead use the `joint_space` or `leader` controller paths depending on whether you want joint-space filtering or leader-to-task-space retargeting.

## Supported Controllers

- `controller_type:=ai_worker`: Main follower-side controller. It tracks task-space pose references for both arms and is typically used with VR teleoperation inputs or RViz markers.
- `controller_type:=joint_space`: Used together with a leader device. It takes the leader's raw joint trajectory commands and adds a safety filter for joint limits, joint velocity limits, and self-collision avoidance before forwarding them to the follower arm controllers.
- `controller_type:=leader`: Also used together with a leader device. It performs forward kinematics from the leader joint configuration, retargets that motion into follower link pose references, and then lets the `ai_worker` controller track those task-space references.

## Prerequisites

- Complete the hardware and software steps in the **Setup Guide**.
- Ensure the robot is on level ground with enough clearance to move both arms safely.
- Make sure you can press the emergency stop button immediately if the robot behaves unexpectedly.
- Connect to the robot PC (keyboard/monitor or SSH) and make sure the battery is charged.

## Bring Up the Robot

Start the AI Worker follower bringup first. Run this on the `robot PC`:

```bash
cd ~/ai_worker
./docker/container.sh enter
ros2 launch ffw_bringup ffw_sg2_follower_ai.launch.py
```

Keep that terminal running. Open a new terminal for the motion controller commands below.

## Launch the Default AI Worker Controller

Open two new terminals and run the same environment setup in both:

```bash
cd ~/ai_worker
./docker/container.sh enter
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

1. In the first terminal, launch the follower-side motion controller with RViz interactive markers enabled:
   ```bash
   ros2 launch motion_controller_ros controller.launch.py controller_type:=ai_worker start_interactive_marker:=true
   ```
2. In the second terminal, arm the controller:
   ```bash
   ros2 service call /reactivate std_srvs/srv/Trigger "{}"
   ```

::: warning
The `ai_worker` controller does not begin motion immediately after launch. It first waits for:

- a `/reactivate` service call
- valid goal poses on `/r_goal_pose` and `/l_goal_pose`
- the current gripper pose to be close enough to the current goal pose

After that, the controller adds a short activation delay and a slow-start ramp. If you command a large discontinuous jump, the controller can stop and require reactivation.
:::

## View RViz via noVNC (headless/SSH)
RViz cannot be displayed over plain SSH. Use noVNC to launch RViz and view it from a browser.

::: tip
- `./install.sh` is only needed once. If the noVNC service is already set up, skip step 3.
- If `192.168.6.2:8090` is unreachable, confirm the host IP and the forwarded port in your environment.
:::

1. Enter the AI Worker Docker container:
    ```bash
    cd ~/ai_worker
    ./docker/container.sh enter
    ```
2. Move to the noVNC workspace:
    ```bash
    cd /workspace/docker-novnc
    ```
3. noVNC setup (First-Time):
    ```bash
    ./install.sh
    ```
4. Start noVNC:
    ```bash
    ./entrypoint.sh
    ```
5. In a browser, open `http://192.168.6.2:8090/vnc.html` and click **Connect**. A terminal window will appear.

6. Run RViz in that terminal:
    ```bash
    rviz2
    ```
7. In RViz, set the fixed frame to `base_link` and add these displays:
   - `RobotModel`
   - `TF`
   - `InteractiveMarkers`

When `start_interactive_marker:=true`, the motion controller starts two 6-DoF markers:

- `right_gripper_goal`
- `left_gripper_goal`

Dragging these markers publishes pose goals to `/r_goal_pose` and `/l_goal_pose`.

## Operating the Controller from RViz

1. Launch the `ai_worker` controller with `start_interactive_marker:=true`.
2. Open RViz and confirm the robot model and interactive markers are visible.
3. Call `/reactivate` once the markers initialize at the current gripper pose.
4. Move the markers gradually to send new end-effector goals.

![interactive_marker_control](/simulation/ai_worker/ffw_sg2_interactive_marker.gif)

::: tip
The built-in reference checker flags sudden changes larger than the configured thresholds and can inhibit motion until you reactivate.
:::

## Other Controllers

### Joint-space Controller

Use this mode when a leader device is already publishing raw joint trajectories and you want the controller to add a safety filter before those commands reach the follower arm controllers:

```bash
ros2 launch motion_controller_ros controller.launch.py controller_type:=joint_space
```

![joint_space_control](/simulation/ai_worker/laserscan.png)

In this mode, the controller does not generate task-space goals. It only filters the leader's joint trajectory commands while considering constraints such as joint limits, velocity limits, and self-collision avoidance.

This mode subscribes to:

- `/leader/joint_trajectory_command_broadcaster_right/raw_joint_trajectory`
- `/leader/joint_trajectory_command_broadcaster_left/raw_joint_trajectory`

and republishes filtered outputs to:

- `/leader/joint_trajectory_command_broadcaster_right/joint_trajectory`
- `/leader/joint_trajectory_command_broadcaster_left/joint_trajectory`

### Leader-follower retargeting controller

Use this mode when an leader device is available and you want to control the follower in task space from the leader configuration:

```bash
ros2 launch motion_controller_ros controller.launch.py controller_type:=leader
```

![retargeting_control](/simulation/ai_worker/laserscan.png)

This launch starts:

- `leader_controller_node`
- follower-side `ai_worker_controller_node`

The `leader_controller_node` performs forward kinematics using the leader joint configuration, retargets that result into follower arm link references, and publishes task-space goal poses. Those references are then tracked by the follower-side `ai_worker_controller_node`. In this mode, the leader node also requests `/reactivate` automatically when valid leader commands begin arriving.

## Launch Arguments

- `controller_type`: Selects `ai_worker`, `joint_space`, or `leader`.
- `start_interactive_marker`: Starts RViz interactive markers when set to `true`.
- `base_frame`: Frame used for marker placement and goal poses. Default: `base_link`.
- `config_file`: Path to .yaml file containing controller parameters.

## Troubleshooting

- If the controller logs `waiting for reactivate`, call:
  ```bash
  ros2 service call /reactivate std_srvs/srv/Trigger "{}"
  ```
- If startup mismatch error occurs, move both interactive markers back near the current gripper pose.
- If the markers do not appear in RViz, confirm `start_interactive_marker:=true` and add the `InteractiveMarkers` display.
- If the controller does not move, verify `/joint_states` is updating or reference divergence event has occured:
  ```bash
  ros2 topic echo /joint_states --once
  ```
- If `leader` or `joint_space` mode does not respond, verify that the raw trajectory topics are receiving data.
- If you are using a non-default robot model, override the URDF/SRDF paths instead of using the SG2/LG2 defaults.

## Controller Parameters

The main parameters live in `controller_config.yaml`. The file is divided by controller name, so you usually tune only the block that matches the controller you are running.

### `ai_worker_controller`

- `control_frequency`, `time_step`: Main control loop speed.
- `trajectory_time`: Time field used when publishing output joint trajectories.
- `kp_position`, `kp_orientation`: Task-space tracking gains for end-effector position and orientation. Larger values make the follower react more aggressively to pose errors.
- `weight_position`, `weight_orientation`, `weight_elbow_position`: Relative importance of each task in the QP solve. Increase a weight when you want that task to be followed more strongly.
- `weight_damping`: Regularization term that discourages unnecessarily large joint velocities.
- `collision_buffer`, `collision_safe_distance`: Safety margins used for collision avoidance.
- `slack_penalty`: Cost applied when the solver must relax constraints. Larger values enforce constraints more strictly.
- `cbf_alpha`: Responsiveness of the barrier constraint. Larger values make the constraint act more like a full brake near a cliff, affecting the control input later and more abruptly as the motion gets very close to the constraint boundary.
- `reactivate_service`: Service used to arm or re-arm the controller after startup checks or reference divergence.
- `joint_states_topic`: Source of measured robot state.
- `right_traj_topic`, `left_traj_topic`: Output arm trajectory topics sent to the follower controllers.
- `lift_topic`, `lift_vel_bound`: Settings for the lift joint command path. When `lift_vel_bound` is set to `0`, the controller does not consider lift joint motion.
- `r_goal_pose_topic`, `l_goal_pose_topic`: Main right and left end-effector goal pose topics.
- `r_elbow_pose_topic`, `l_elbow_pose_topic`: Elbow reference position topics used to shape arm posture.
- `r_gripper_pose_topic`, `l_gripper_pose_topic`: Published current gripper poses.
- `r_gripper_name`, `l_gripper_name`, `r_elbow_name`, `l_elbow_name`: Link names used when computing task-space poses from the robot model.
- `right_gripper_joint`, `left_gripper_joint`: Joint names used for the grippers in trajectory messages.
- `startup_ref_pos_threshold`, `startup_ref_ori_threshold_deg`: Startup alignment thresholds. The controller waits until the current gripper pose is close enough to the commanded reference before enabling motion.

### `joint_space_controller`

- `control_frequency`, `time_step`: Joint-space filter loop speed.
- `weight_tracking`: Weight for following the incoming leader joint trajectory command.
- `weight_damping`: Regularization term that discourages unnecessarily large or abrupt joint motion.
- `collision_buffer`, `collision_safe_distance`: Safety margins used for collision avoidance during joint-space filtering.
- `slack_penalty`: Cost applied when the solver must relax constraints. Larger values enforce constraints more strictly.
- `cbf_alpha`: Responsiveness of the barrier constraint. Larger values make the constraint act more like a full brake near a cliff, affecting the control input later and more abruptly as the motion gets very close to the constraint boundary.
- `command_timeout`: Timeout for the incoming leader command stream. If commands stop arriving for longer than this value, the controller treats the stream as inactive.
- `joint_states_topic`: Source of measured follower joint state.
- `right_traj_topic`, `left_traj_topic`: Raw joint trajectory input topics from the leader side.
- `right_traj_filtered_topic`, `left_traj_filtered_topic`: Filtered joint trajectory output topics sent to the follower arm controllers.

### `leader_controller`

- `control_frequency`: Forward-kinematics and retargeting update rate.
- `joint_states_topic`: Source of measured follower state, used mainly for the lift joint.
- `right_traj_topic`, `left_traj_topic`: Raw joint trajectory input topics from the leader side.
- `reactivate_service`: Service used to arm the follower-side `ai_worker` controller automatically when a valid leader command stream starts.
- `command_timeout`: Timeout for detecting that the leader command stream has stopped.
- `r_goal_pose_topic`, `l_goal_pose_topic`: Right and left end-effector goal pose topics generated for the follower.
- `r_elbow_pose_topic`, `l_elbow_pose_topic`: Right and left elbow reference topics generated for the follower.
- `base_frame_id`: Base frame used for published task-space references.
- `r_gripper_name`, `l_gripper_name`, `r_elbow_name`, `l_elbow_name`: Link names used for forward kinematics and retargeting.
- `lift_joint_name`, `model_lift_joint_name`: Lift joint names used to map the leader model and follower model correctly.

### `reference_checker`

- `ref_pos_jump_threshold`: Maximum allowed sudden position jump in the incoming task-space reference.
- `ref_ori_jump_threshold_deg`: Maximum allowed sudden orientation jump in the incoming task-space reference.
- `r_goal_pose_topic`, `l_goal_pose_topic`: Right and left goal pose topics monitored for discontinuities.

## Safety and Usage Tips

- Keep people, cables, and nearby objects clear before arming the controller.
- Start from a neutral pose and make small pose changes first.
- Re-arm the controller any time you intentionally reset goals or recover from a large reference jump.
- Always stay within reach of the emergency stop button during operation, and use it immediately if the robot behaves unexpectedly.