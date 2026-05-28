# SH5 DDS Bridge for Isaac Sim VR Teleoperation

### Full Demo

<div style="position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden; max-width: 100%; margin: 20px 0; border-radius: 8px; border: 2px solid #1e3c72;">
  <iframe
    style="position: absolute; top: 0; left: 0; width: 100%; height: 100%;"
    src=""
    title="SH5 Isaac Sim VR Teleoperation Demo"
    frameborder="0"
    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
    allowfullscreen>
  </iframe>
</div>

<p style="text-align: center; font-style: italic; color: #888; font-size: 14px; margin-bottom: 20px;">
  This video demonstrates SH5 VR and keyboard teleoperation in Isaac Sim, including object pick-and-place and mobile manipulation tasks in kitchen, warehouse, and other virtual environments.
</p>

## 1. Overview

This project implements a DDS-based bridge that connects external ROS 2 command topics to the SH5 AI Worker model running in Isaac Sim. It allows the SH5 control stack to be tested without immediate access to physical hardware and can be used to prepare virtual task environments for imitation learning toward Physical AI applications.

The main script, `sh5_dds_bringup.py`, loads the SH5 model with `FFW_SH5.py`, receives `JointTrajectory` and `/cmd_vel` commands through `robotis_dds_python`, and applies them to the simulated SH5 articulation.

The core idea is simple:

1. ROS 2 controllers publish SH5 arm, hand, lift, and mobile base commands.
2. `sh5_dds_bringup.py` receives the commands through the DDS SDK.
3. The SH5 model moves in Isaac Sim and publishes state topics for pose checking.

### Key Packages and File Structure

The SH5 DDS bringup path mainly uses:

* `sh5_dds_bringup.py`: Main Isaac Sim bridge for SH5 model loading, DDS communication, command application, and state publishing.
* `FFW_SH5.py`: SH5 robot model configuration used by Isaac Lab/Isaac Sim to define the USD path, articulation properties, initial joint state, actuators, and joint limits.
* `swerve_drive.py`: Three-module swerve drive controller that converts `/cmd_vel` into steering positions and wheel velocities.
* `environment.py`: Environment helper for loading the default NVIDIA Simple Warehouse USD scene.
* `robotis_dds_python`: DDS SDK used to communicate with ROS 2-compatible topics through CycloneDDS.

## 2. System Architecture

The SH5 DDS bringup system is organized around the following main components:

![System Architecture](/technical_story/sh5_dds_bridge_system_architecture.jpg)

* **VR Device**: Provides operator pose input to the external ROS 2 teleoperation stack.
* **ROS 2 Controllers**: `cyclo_motion_controller` retargets the VR pose and publishes SH5 `JointTrajectory` commands for the arms, hands, and lift, while `ffw_teleop/mobile_teleop` publishes `/cmd_vel` for the swerve mobile base.
* **DDS SDK Layer**: Uses `robotis_dds_python` and CycloneDDS to exchange ROS 2-compatible messages with the Isaac Sim Python process.
* **SH5 DDS Bringup**: Spawns the SH5 model and optional Simple Warehouse environment, receives command topics, maps joint names to Isaac Sim articulation joints, applies swerve drive targets, and publishes simulated `/joint_states` and `/tf`.
* **Isaac Sim Visualization**: Shows the commanded SH5 motion and provides the main viewport for checking VR teleoperation behavior in simulation.

## 3. Start Guide

### Step 1: Launch the SH5 DDS Bringup in Isaac Sim

Enter the ROBOTIS Lab Docker container:

```bash
cd ~
git clone https://github.com/ROBOTIS-GIT/robotis_lab.git

cd ~/robotis_lab/docker
./container.sh start
./container.sh enter
```

Launch the SH5 model and DDS bridge:

```bash
python scripts/sim2real/bringup/sh5_dds_bringup.py \
  --domain_id 30 \
  --enable_gravity \
  --enable_camera_views
```

To include the default NVIDIA Simple Warehouse environment:

```bash
python scripts/sim2real/bringup/sh5_dds_bringup.py \
  --domain_id 30 \
  --enable_gravity \
  --enable_camera_views \
  --enable_environment
```

::: warning
The DDS domain ID must match the external ROS 2 containers that publish the command topics. For example, if Isaac Sim uses `--domain_id <number>`, the VR interface and ROS 2 controller containers must use the same ROS domain ID.
:::

### Step 2: Connect the VR Interface

::: info
For the complete VR device setup, Vuer page connection, and operation procedure, refer to the [AI Worker VR Teleoperation Guide](./operation_vr_teleoperation_ai_worker.md).
:::

Enter the `robotis_applications` Docker container:

```bash
cd ~
git clone https://github.com/ROBOTIS-GIT/robotis_applications.git

cd ~/robotis_applications/docker
./container.sh start
./container.sh enter
```

Launch the VR interface:

::: tabs key:robot-type
== Hand Type
```bash
ros2 launch robotis_vuer vr.launch.py model:=sh5
```
Or use the shortcut:
```bash
vr model:=sh5
```
:::

::: tip
If lift motion should be disabled for a more stable manipulation test, set the VR `lift_publisher` option to `false` and reduce the `lift_joint` velocity limit to 0.0001 in the `FFW_SH5.py`.
:::

### Step 3: Launch the ROS 2 Motion Controller

Enter the `ai_worker` Docker container:

```bash
cd ~
git clone https://github.com/ROBOTIS-GIT/ai_worker.git

cd ~/ai_worker/docker
./container.sh start
./container.sh enter
```

Launch `cyclo_motion_controller`:

::: tabs key:robot-type
== Hand Type

```bash
ros2 launch cyclo_motion_controller_ros ai_worker_controller.launch.py controller_type:=vr hand:=true
```

Or use the shortcut command:

```bash
motion_controller controller_type:=vr hand:=true
```
:::

### Step 4: Run Mobile Base Teleoperation

Run `mobile_teleop` to publish `/cmd_vel`:

```bash
ros2 run ffw_teleop mobile_teleop
```

| Key | Command |
| :---: | :--- |
| `W` / `S` | Move forward / backward |
| `A` / `D` | Turn left / right |
| `Space` | Stop |
| `Ctrl+C` | Quit |

::: tip
You can modify `mobile_teleop` to add custom key mappings and support freer base movement.
:::

## 4. SH5 Model Configuration

The SH5 Isaac Sim model is defined in `FFW_SH5.py`. This configuration does more than load the USD file; it also adjusts physics properties so the hand and swerve base can move smoothly in simulation.

### Fingertip Material

The real SH5 hand uses rubber-like fingertip surfaces for grip. To reflect this in simulation, the SH5 fingertips use a custom high-friction physics material that is bound to the finger collision prims.

```python
_SH5_FINGER_TIP_MATERIAL = RigidBodyMaterialCfg(
    static_friction=2.0,
    dynamic_friction=1.8,
    restitution=0.0,
)
```

### Hand Collision Filtering

The hand base collision is filtered against selected MCP and PIP finger links:

```python
_SH5_BASE_COLLISION_LINKS = (5, 6, 9, 10, 13, 14, 17, 18)
```

These collision pairs are disabled because the finger links can otherwise collide with the hand base during lateral opening and closing motions. Filtering them allows the fingers to move side-to-side more smoothly and prevents unnecessary self-collision resistance in the simulated hand.

### Swerve Base Collision Filtering

The mobile base also filters collision pairs between `base_link` and the wheel drive links:

```python
_SH5_WHEEL_DRIVE_LINKS = ("left_wheel_drive", "right_wheel_drive", "rear_wheel_drive")
```

This prevents the base collider from interfering with the wheel drive colliders. With these pairs filtered, the steer joints can rotate smoothly and respond more directly to swerve drive commands.

## 5. Command Flow

This architecture connects the external ROS 2 command interface to the Isaac Sim simulation runtime. The ROS 2 controllers generate robot commands outside Isaac Sim, while `sh5_dds_bringup.py` receives those topics through the DDS SDK and applies them to the SH5 model in the virtual environment.

The important point is that Isaac Sim becomes a command target for the same ROS 2 control flow used outside the simulator. This allows the SH5 model to move in virtual scenes, test base motion and manipulation behavior, and publish `/joint_states` and `/tf` as state topics for initial pose checking and transform inspection.

The overall command flow is:

```text
VR Device
  ↓
VR Pose Input
  ↓
Cyclo Motion Controller
  ↓
Retargeting
  ↓
ROS 2 JointTrajectory Command Topics
  ├── /leader/joint_trajectory_command_broadcaster_right/joint_trajectory
  ├── /leader/joint_trajectory_command_broadcaster_right_hand/joint_trajectory
  ├── /leader/joint_trajectory_command_broadcaster_left/joint_trajectory
  ├── /leader/joint_trajectory_command_broadcaster_left_hand/joint_trajectory
  └── /leader/joystick_controller_right/joint_trajectory
        ↓
robotis_dds_python / CycloneDDS
        ↓
sh5_dds_bringup.py
        ↓
Isaac Sim SH5 Articulation
        ↓
Isaac Sim Viewport
```

The mobile base command flow is:

```text
ffw_teleop / mobile_teleop
  ↓
/cmd_vel
  ↓
robotis_dds_python / CycloneDDS
  ↓
sh5_dds_bringup.py
  ↓
Swerve Drive Controller
  ↓
Isaac Sim SH5 Mobile Base
  ↓
Isaac Sim Viewport
```

The state topic flow is:

```text
Isaac Sim SH5 Articulation
  ↓
sh5_dds_bringup.py
  ↓
robotis_dds_python / CycloneDDS
  ↓
ROS 2 State Topics
  ├── /joint_states
  └── /tf
        ↓
Initial pose check / state reference
```

## 6. DDS Topic and Message Mapping

The Isaac Sim bridge uses `robotis_dds_python` to communicate with ROS 2-compatible topics through CycloneDDS. Each ROS 2 message type is represented by an IDL-generated Python class with an underscore suffix, such as `JointTrajectory_`, `Twist_`, `JointState_`, and `TFMessage_`.

| Topic                                                                      | Direction | DDS SDK Message Class | ROS 2 Message Type                    | Description                                         |
| :------------------------------------------------------------------------- | :-------: | :-------------------- | :------------------------------------ | :-------------------------------------------------- |
| `/leader/joint_trajectory_command_broadcaster_right/joint_trajectory`      | Subscribe | `JointTrajectory_`    | `trajectory_msgs/msg/JointTrajectory` | Right arm command                                   |
| `/leader/joint_trajectory_command_broadcaster_right_hand/joint_trajectory` | Subscribe | `JointTrajectory_`    | `trajectory_msgs/msg/JointTrajectory` | Right hand command                                  |
| `/leader/joint_trajectory_command_broadcaster_left/joint_trajectory`       | Subscribe | `JointTrajectory_`    | `trajectory_msgs/msg/JointTrajectory` | Left arm command                                    |
| `/leader/joint_trajectory_command_broadcaster_left_hand/joint_trajectory`  | Subscribe | `JointTrajectory_`    | `trajectory_msgs/msg/JointTrajectory` | Left hand command                                   |
| `/leader/joystick_controller_right/joint_trajectory`                       | Subscribe | `JointTrajectory_`    | `trajectory_msgs/msg/JointTrajectory` | Lift command                                        |
| `/cmd_vel`                                                                 | Subscribe | `Twist_`              | `geometry_msgs/msg/Twist`             | Mobile base command from `ffw_teleop/mobile_teleop` |
| `/joint_states`                                                            |  Publish  | `JointState_`         | `sensor_msgs/msg/JointState`          | Simulated joint state for pose checking             |
| `/tf`                                                                      |  Publish  | `TFMessage_`          | `tf2_msgs/msg/TFMessage`              | Simulated transform state for initial pose checking |

For the `JointTrajectory_` command topics, the bridge reads the latest trajectory point, extracts `joint_names` and `positions`, and maps them to the matching SH5 articulation joints in Isaac Sim.

For `/cmd_vel`, the bridge receives a `Twist_` message and uses `linear.x`, `linear.y`, and `angular.z` as the mobile base velocity command. These values are passed to the swerve drive controller, which generates steering joint targets and wheel velocity targets.

For state checking, the bridge publishes `/joint_states` using `JointState_` and `/tf` using `TFMessage_`. The `/tf` message is built from multiple `TransformStamped_` messages, using `base_link` as the parent frame and each SH5 body link as a child frame. These topics are mainly used to inspect the initial arm pose and confirm the simulated robot transform state.
