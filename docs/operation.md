# Teleoperation Guide

This guide explains how to set up and operate the AI Worker using teleoperation.

## Prerequisites

Before starting teleoperation, make sure:
- The AI Worker is properly set up and powered on
- The Docker environment is running
- You have enough space for comfortable movement

## Preparation

1. Place the `Follower` robot in a spacious area with enough room for movement.
2. Ensure the `Leader` device is powered on and ready for use.
3. Wear the `Leader` device according to the instructions below.

![Leader's wearing appearance](/quick_start_guide/leader_wearing_appearance.png)

### How to wear the Leader:

1. Put both arms through the `Leader`'s `shoulder straps`.
2. Fasten the chest belt buckle and `hip belt buckle`.
3. Adjust the length of the `shoulder straps`, `chest belt`, and `hip belt` so that the `Leader` is securely fixed on your back.
4. Face the `red sticker` attached to the `Leader` to set the initial position.

## Running Teleoperation

### Option 1: All-in-One Launch

To start both the `Leader` and `Follower` simultaneously:

```bash
ros2 launch ffw_bringup ffw_bg2_ai.launch.py
```
or use the shortcut command:
```bash
bringup
```

### Option 2: Separate Launches

If you want to run the `Leader` and `Follower` separately in different terminals:

1. **Launch the teleoperation `Leader`**:
   ```bash
   ros2 launch ffw_bringup ffw_bg2_leader_ai.launch.py
   ```
   or use the shortcut:
   ```bash
   leader
   ```

2. **Launch the teleoperation `Follower`**:
   ```bash
   ros2 launch ffw_bringup ffw_bg2_follower_ai.launch.py
   ```
   or use the shortcut:
   ```bash
   follower
   ```

If you want to run the system without launching the cameras, you can set the `launch_cameras` parameter to `false`:
```bash
   ros2 launch ffw_bringup ffw_bg2_follower_ai.launch.py launch_cameras:=false
```



## Basic Operation

1. Once both systems are running, the `Follower` will begin to mirror your movements.
2. Start with slow, gentle movements to get familiar with the response.
3. The grip buttons on the `Leader` control the gripper actions on the `Follower`.

## Stopping Teleoperation

To stop the teleoperation:
1. Return to a neutral position
2. Press Ctrl+C in the terminal running the launch file

## Troubleshooting

- **Delayed movements**: Check for any obstructions or if you're reaching joint limits
- **Unresponsive gripper**: Ensure proper calibration and connection
- **System unresponsive**: Check ROS topic connections with `ros2 topic list` and `ros2 topic echo`
