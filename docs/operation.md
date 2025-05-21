# AI Worker Teleoperation Guide

## Preparation
1. Place the `Follower` in a spacious area
2. Wear the `Leader` device

## Leader Setup
![Leader's wearing appearance](assets/leader_wearing_appearance.png)

### Wearing Instructions
1. Put both arms through the `shoulder straps`
2. Fasten the `chest belt` and `hip belt` buckles
3. Adjust strap lengths for secure fit:
   - Shoulder straps
   - Chest belt
   - Hip belt
4. Align with the `red sticker` for initial position

## Operation

### Quick Start
Run everything in a single terminal:
```bash
ros2 launch ffw_bringup bringup.launch.py
# or
bringup
```

### Separate Terminal Operation
Run `Leader` and `Follower` in different terminals:

1. **Leader Terminal**
   ```bash
   ros2 launch ffw_bringup leader.launch.py
   # or
   leader
   ```

2. **Follower Terminal**
   ```bash
   # With camera (default)
   ros2 launch ffw_bringup follower_with_camera.launch.py
   # or
   follower

   # Without camera
   ros2 launch ffw_bringup follower.launch.py
   ```

### Important Notes
⚠️ **Camera Operation**
- Wait 30 seconds after launch for camera initialization
- Start teleoperation only after camera feed is stable
- Use `follower.launch.py` if camera is not needed
