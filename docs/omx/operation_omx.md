---
prev: 
  text: Setup Guide
  link: /omx/setup_guide_omx
---

# Operation Guide — ROS 2

## Teleoperation
After accessing the Docker container, use the following command:
```bash
ros2 launch open_manipulator_bringup omx_ai.launch.py
```
This command will sequentially execute the following procedures:
1. Move to Follower's initial position.
2. Start Leader launch file.
3. Synchronize Leader and Follower.

After these steps, the Leader-Follower system will be operational.

<img src="/quick_start_guide/omx/ai_teleop.gif" alt="AI Teleoperation" style="display:block;margin:0 auto;max-width:100%;width:720px;" />
<p style="text-align: center;"><em>Real-time synchronized motion between Leader and Follower</em></p>


::: info
The basic commands in this manual are written for **OMX_F**.
Example:
```bash
ros2 launch open_manipulator_bringup omx_f.launch.py
```
:::

## Launch Bringup
The OMX controller has been restructured to utilize the `ros2_control` framework and `MoveIt 2` for enhanced flexibility, modularity, and usability. This updated controller allows for seamless integration with ROS 2-based systems, offering advanced features such as trajectory planning, real-time control, and state feedback.

Open a new Open Manipulator Docker Container and launch the OMX packages.

```bash
 ros2 launch open_manipulator_bringup omx_f.launch.py
```

## MoveIt 2
MoveIt 2 is a set of packages for your robot to manipulate for Motion Planning, Manipulation, Inverse Kinematics, Control, 3D Perception and Collision Checking.

### Launching MoveIt 2
Enable MoveIt 2 functionality for advanced motion planning in RViz.
For more information about MoveIt 2, check out the [official documentation](https://moveit.picknik.ai/main/doc/how_to_guides/how_to_guides.html).
```bash
 ros2 launch open_manipulator_moveit_config omx_f_moveit.launch.py
```
Move interactive markers to position the robotic arm, then click Plan and Execute.
![](/quick_start_guide/omx/moveit2_core.png)

**Simple Instructions for Using MOVEIT 2:**
1. You can move the robot to your desired pose using the **Interactive Marker** visible in RViz.
2. Use the **Plan** option in the Commands column to simulate the motion of the robotic arm.
3. Click **Execute** to move the robot according to the simulated motion.
4. Set the **Planning Group** to `arm` and use the **Goal State** options `init` or `home` to move the robot to predefined poses.
5. Change the **Planning Group** to **gripper** and use **Goal State** options such as `close` or `open` to open and close the gripper.


## GUI

### Launch the OpenMANIPULATOR GUI
After running bringup and MoveIt, launch the GUI.
```bash
ros2 launch open_manipulator_gui omx_f_gui.launch.py
```

### Explore GUI Features
**Basic Controls**
- `Start Timer`: Activates the system.
- `Robot Status`: Displays current manipulator state.
- `Init Pose`: Moves the manipulator to a vertical position.
- `Home Pose`: Moves the manipulator to a compact, safe position.
- `Gripper Open/Close`: Opens or closes the gripper.

**Task Execution**
- `Joint Space Tab`: Adjust individual joint angles.
- `Task Space Tab`: Control the end-effector position.
- `Task Constructor Tab`
  - `Read Task`: View saved poses.
  - `Save Pose`: Save current state.
  - `Rap`: Set task repetition (1–999).
  - `Play`: Execute saved tasks.
  - `Stop`: Halt operations.
  - `Reset Task`: Clear saved tasks.

**Explore GUI Features**
1. Click the `Timer Start` button.
![](/quick_start_guide/omx/OMX_GUI1.png)

2. Check the status of OMX.
![](/quick_start_guide/omx/OMX_GUI2.png)

3. To manipulate OMX in the joint space, enter the joint angles and total time for the trajectory. Then click the `Send` button to start the motion.
![](/quick_start_guide/omx/OMX_GUI3.png)

4. To manipulate OMX in the task space, enter the kinematics pose of the OMX end-effector(tool) in the task space. Then click the `Send` button to start the motion.
![](/quick_start_guide/omx/OMX_GUI4.png)

5. Create a task constructor with OMX.
- Click the `Read task` button to prepare for saving tasks.
  - You can create or modify the pre-saved joint values in the `robot_joint_log.csv` file.
  - The file's load path can be checked in the terminal when the GUI node is launched.
- Adjust the manipulator's posture and save it by clicking `Save pose`.
- Save multiple postures to complete the preparation.
![](/quick_start_guide/omx/OMX_GUI5.png)

6. Press the `Play` button to make the manipulator move sequentially according to the saved tasks. Completed tasks will have their status changed to <span style="color: green; font-weight: bold;">Done</span>.
![](/quick_start_guide/omx/OMX_GUI6.png)

### OMX GUI Demo
![](/quick_start_guide/omx/omx_f_gui.gif)