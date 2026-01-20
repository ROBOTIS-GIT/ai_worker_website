# Teleoperation

This guide explains how to set up and operate the AI Worker using teleoperation.

## Prerequisites

Before starting teleoperation, make sure:
- The follower(AI Worker) is properly set up and powered on

### Steps for Teleoperation Setup (Leader)
(*The term `Leader` refers to the control device in the AI WORKER system)
![Back of the LG2 Leader](/quick_start_guide/ai_worker/back_of_the_LG2_leader.png)
1. `Power Cable`: Connect the power adapter to supply power to the U2D2 device.
2. `U2D2 ON/OFF`: This switch is located inside a small recessed hole. The device is turned on when the inner white-dotted button is pressed.
3. `USB Cable`: Connect a USB cable from the U2D2 to one of the USB ports on the back of the `Follower`.


### How to Wear the Leader
**First Leader Version**
![Leader's wearing appearance](/quick_start_guide/ai_worker/leader_wearing_appearance.png)

1. Put both arms through the `Leader`'s `shoulder straps`.
2. Fasten the chest belt buckle and `hip belt buckle`.
3. Adjust the length of the `shoulder straps`, `chest belt`, and `hip belt` so that the `Leader` is securely fixed on your back.

**Second Leader Version**
It can be worn more easily by users with different body types.

<img src="/quick_start_guide/ai_worker/new_fixed_belt.png" alt="Web UI" style="width: 40%; ">

1. First, pull the inner belt tightly and fasten it around the waist.
2. Then, pull the outer belt and attach it over the inner belt using the Velcro.

## Running Teleoperation
The following teleoperation commands are executed on the `robot PC`.
You can either connect a keyboard and mouse directly to the Nvidia Orin, or access it via SSH (see the Setup Guide for instructions).<br>


Enter the **ai_worker** Docker container with:

```bash
cd ~/ai_worker
./docker/container.sh enter
```

### Option 1: All-in-One Launch

::: tip
After execution, the follower will not move until you push both hand triggers for more than 2 seconds. Once you do, the follower will slowly move toward the leader's position, and after reaching a certain proximity, it will move more quickly. This behavior is the same even when launching the leader and follower separately.
:::

To start both the `Leader` and `Follower(BG2/SG2)` simultaneously:

:::tabs key:robot-type
== BG2 Type
ros2 launch ffw_bringup ffw_bg2_ai.launch.py
== SG2 Type
ros2 launch ffw_bringup ffw_sg2_ai.launch.py
:::

Or use the shortcut:

:::tabs key:robot-type
== BG2 Type
ffw_bg2_ai
== SG2 Type
ffw_sg2_ai
:::

### Option 2: Separate Launches
If you want to run the `Leader` and `Follower` separately in different terminals:

1. **Launch the teleoperation `Follower`**:
:::tabs key:robot-type
== BG2 Type
ros2 launch ffw_bringup ffw_bg2_follower_ai.launch.py
== SG2 Type
ros2 launch ffw_bringup ffw_sg2_follower_ai.launch.py
:::

Or use the shortcut:

:::tabs key:robot-type
== BG2 Type
ffw_bg2_follower_ai
== SG2 Type
ffw_sg2_follower_ai
:::

#### Optional Parameters

You can customize the follower launch behavior with optional parameters:

**Disable Camera Launch**
To run the system without launching the cameras, set the `launch_cameras` parameter to `false`:
```bash
ros2 launch ffw_bringup ffw_bg2_follower_ai.launch.py launch_cameras:=false
```

**Disable Position Initialization**
To start the system without initializing the position, set the `init_position` parameter to `false`:
```bash
ros2 launch ffw_bringup ffw_bg2_follower_ai.launch.py init_position:=false
```

**Combine Multiple Parameters**
You can use both parameters together:
```bash
ros2 launch ffw_bringup ffw_bg2_follower_ai.launch.py launch_cameras:=false init_position:=false
```

2. **Launch the teleoperation `Leader`**:
   ```bash
   ros2 launch ffw_bringup ffw_lg2_leader_ai.launch.py
   ```

   Or use the shortcut:
   ```bash
   ffw_lg2_leader_ai
   ```

## Basic Operation
::: tip
You must push **both hand triggers** for more than 2 seconds for the follower to start moving. When you push both triggers forward, the follower will slowly move to the leader's position, and once it comes within a certain error range, it will move quickly.

<img src="/quick_start_guide/ai_worker/push_trigger_ai_worker.gif" alt="Push Trigger" style="width: 50%;">
:::

### FFW_BG2
1. Once both systems are running, the `Follower` will begin to mirror your movements.
2. Start with slow, gentle movements to get familiar with the response.
3. The grip buttons on the `Leader` control the gripper actions on the `Follower`.
4. The `right joystick` controls the up and down motion of the `lift`,
while the `left joystick` controls the`head` section of the robot.

### FFW_SG2
1. Once both systems are running, the `Follower` will begin to mirror your movements.
2. Start with slow, gentle movements to get familiar with the response.
3. The grip buttons on the `Leader` control the gripper actions on the `Follower`.
4. The `right joystick` controls the up and down motion of the `lift`,
while the `left joystick` controls the`head` section of the robot.
5. Pressing both switches simultaneously switches to `SWERVE DRIVE MODE`.
6. In `SWERVE DRIVE MODE`, the left joystick controls: X-axis: Linear x, Y-axis: Linear y
   The right joystick controls: Y-axis: Angular z
![Swerve joystick Control](/quick_start_guide/ai_worker/LG2_Joystick.png)

::: warning
In swerve mode, the arms continue to move. Please be careful.
:::

## Pause Teleoperation

To temporarily pause teleoperation:
1. Push both hand triggers for more than 2 seconds
2. The follower will stop and hold its current position

## Stopping Teleoperation

To stop teleoperation:
1. Return to a neutral position or pause the system.
2. Press `Ctrl+C` in the terminal where the launch file is running.

## Troubleshooting

- **Delayed movements**: Check for any obstructions or if you're reaching joint limits
- **Unresponsive gripper**: Ensure proper calibration and connection
- **System unresponsive**: Check ROS topic connections with `ros2 topic list` and `ros2 topic echo`
- **Trigger pause function not available or robot moves immediately upon startup**: Please update the ai_worker source code to version `1.1.10` or higher
