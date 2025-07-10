# Operation Execution

## Teleoperation
After accessing the Docker container, use the following command:
```bash
ros2 launch open_manipulator_bringup omy_ai.launch.py
```
This command will sequentially execute the following procedures:
1. Move to Follower's initial position
2. Start Leader's gravity compensation
3. Synchronize Leader and Follower

After these steps, the Leader-Follower system will be operational. **AI Teleoperation** is only supported between OMY_F3M and OMY_L100 models.

![](/quick_start_guide/ai_teleop.gif)

---
::: info
The basic commands in this manual are written for **OMY_F3M**.
When running with the **OMY_3M** model, replace `omy_f3m` with `omy_3m` in the commands.
Example:
```bash
ros2 launch open_manipulator_bringup omy_3m.launch.py
:::

## Launch Bringup
The OMY-AI controller has been restructured to utilize the ros2_control framework and MoveIt 2 for enhanced flexibility, modularity, and usability. This updated controller allows for seamless integration with ROS 2-based systems, offering advanced features such as trajectory planning, real-time control, and state feedback.

Open a new OM Container and launch the OMY-AI packages.

```bash
$ ros2 launch open_manipulator_bringup omy_f3m.launch.py
```

## MoveIt 2
MoveIt 2 is a set of packages for your robot to manipulate for Motion Planning, Manipulation, Inverse Kinematics, Control, 3D Perception and Collision Checking.

###  Launching MoveIt 2
Enable MoveIt 2 functionality for advanced motion planning in RViz.
For more information about MoveIt 2, check out the [official documentation](https://moveit.picknik.ai/main/doc/how_to_guides/how_to_guides.html).
```bash
$ ros2 launch open_manipulator_moveit_config omy_f3m_moveit.launch.py
```
Move interactive markers to position the robotic arm, then click Plan and Execute.
![](/quick_start_guide/moveit2_core.png)

**Simple Instructions for Using MOVEIT 2:**
1. You can move the robot to your desired pose using the **Interactive Marker** visible in RViz.
2. Use the **Plan** option in the Commands column to simulate the motion of the robotic arm.
3. Click **Execute** to move the robot according to the simulated motion.
4. Set the **Planning Group** to `arm` and use the **Goal State** options `init` or `home` to move the robot to predefined poses.
5. Change the **Planning Group** to **gripper** and use **Goal State** options such as `close` or `open` to open and close the gripper.




### Launch MoveIt 2
```bash
ros2 launch open_manipulator_moveit_config omy_f3m_moveit.launch.py
```

## GUI

### Launch the OpenMANIPULATOR GUI
After running bringup and MoveIt, launch the GUI.
```bash
ros2 launch open_manipulator_gui omy_f3m_gui.launch.py
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
![](/quick_start_guide/OMY_GUI1.png)

2. Check the status of OMY-AI.
![](/quick_start_guide/OMY_GUI2.png)

3. To manipulate OMY-AI in the joint space, enter the joint angles and total time for the trajectory. Then click the `Send` button to start the motion.
![](/quick_start_guide/OMY_GUI3.png)

4. To manipulate OMY-AI in the task space, enter the kinematics pose of the OMY-AI end-effector(tool) in the task space. Then click the `Send` button to start the motion.
![](/quick_start_guide/OMY_GUI4.png)

5. Create a task constructor with OMY-AI.
- Click the `Read task` button to prepare for saving tasks.
  - You can create or modify the pre-saved joint values in the `robot_joint_log.csv` file.
  - The file's load path can be checked in the terminal when the GUI node is launched.
- Adjust the manipulator's posture and save it by clicking `Save pose`.
- Save multiple postures to complete the preparation.
![](/quick_start_guide/OMY_GUI5.png)

6. Press the `Play` button to make the manipulator move sequentially according to the saved tasks. Completed tasks will have their status changed to <span style="color: green; font-weight: bold;">Done</span>
![](/quick_start_guide/OMY_GUI6.png)

### OMY-AI GUI Demo
![](/quick_start_guide/omy_f3m_gui.gif)



## Keyboard Teleop
Control the manipulator (simulation or hardware) using your keyboard

::: info
After launching the real robot or Gazebo bringup, enter the following command in the new container:
:::
```bash
ros2 run open_manipulator_teleop omy_f3m_teleop
```
**Joint Control**
- `1` / `q` - Joint 1
- `2` / `w` - Joint 2
- `3` / `e` - Joint 3
- `4` / `r` - Joint 4
- `5` / `t` - Joint 5
- `6` / `y` - Joint 6

**Gripper Control**
- `o` - Open gripper
- `p` - Close gripper

### Keyboard Teleop Demo
![](/quick_start_guide/omy_f3m_teleoperation.gif)
