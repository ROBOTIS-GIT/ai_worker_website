# Teleoperation Guide

This guide explains how to set up and operate the AI Worker using teleoperation.

## Prerequisites

Before starting teleoperation, make sure:
- The follower(AI Worker) is properly set up and powered on

### Steps for Teleoperation Setup (Leader)
(*The term `Leader` refers to the control device in the AI WORKER system)
![Back of the Leader](/quick_start_guide/back_of_the_leader.png)
1. `Power Cable`: Connect the power adapter to supply power to the U2D2 device.
2. `U2D2 ON/OFF`: This switch is located inside a small recessed hole. The device is turned on when the inner white-dotted button is pressed.
3. `USB Cable`: Connect a USB cable from the U2D2 to one of the USB ports on the back of the `Follower`.


### How to wear the Leader
![Leader's wearing appearance](/quick_start_guide/leader_wearing_appearance.png)

1. Put both arms through the `Leader`'s `shoulder straps`.
2. Fasten the chest belt buckle and `hip belt buckle`.
3. Adjust the length of the `shoulder straps`, `chest belt`, and `hip belt` so that the `Leader` is securely fixed on your back.

## Running Teleoperation
The following teleoperation commands are executed on the `robot PC`.
You can either connect a keyboard and mouse directly to the Nvidia Orin, or access it via SSH (see the Setup Guide for instructions).<br>

If no containers are running when you execute `docker ps -a` on the robot PC,
start the container using:

```bash
cd ai_worker
```

```bash
./docker/container.sh start without_gz
```

If a container is already running, enter the **ai\_worker** Docker container with:

```bash
./docker/container.sh enter
```

### Option 1: All-in-One Launch
⚠️While the command is entered in the terminal, the teleoperator should begin in a `standing position` with both arms lowered.

To start both the `Leader` and `Follower` simultaneously:

```bash
ros2 launch ffw_bringup ffw_bg2_ai.launch.py
```
or use the shortcut command:
```bash
ffw_bg2_ai
```

### Option 2: Separate Launches

If you want to run the `Leader` and `Follower` separately in different terminals:

1. **Launch the teleoperation `Leader`**:
   ```bash
   ros2 launch ffw_bringup ffw_bg2_leader_ai.launch.py
   ```
   or use the shortcut:
   ```bash
   ffw_lg2_leader_ai
   ```

2. **Launch the teleoperation `Follower`**:
   ```bash
   ros2 launch ffw_bringup ffw_bg2_follower_ai.launch.py
   ```
   or use the shortcut:
   ```bash
   ffw_bg2_follower_ai
   ```

If you want to run the system without launching the cameras, you can set the `launch_cameras` parameter to `false`:
   ```bash
   ros2 launch ffw_bringup ffw_bg2_follower_ai.launch.py launch_cameras:=false
   ```

## Basic Operation

1. Once both systems are running, the `Follower` will begin to mirror your movements.
2. Start with slow, gentle movements to get familiar with the response.
3. The grip buttons on the `Leader` control the gripper actions on the `Follower`.
4. The `right joystick` controls the up and down motion of the `lift`,
while the `left joystick` controls the`head` section of the robot.

## Stopping Teleoperation

To stop the teleoperation:
1. Return to a neutral position
2. Press Ctrl+C in the terminal running the launch file

## Troubleshooting

- **Delayed movements**: Check for any obstructions or if you're reaching joint limits
- **Unresponsive gripper**: Ensure proper calibration and connection
- **System unresponsive**: Check ROS topic connections with `ros2 topic list` and `ros2 topic echo`
