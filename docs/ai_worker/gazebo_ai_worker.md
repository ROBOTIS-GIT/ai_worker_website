# Getting Started with Gazebo

## Overview
You can launch Gazebo and RViz(Moveit) individually, or launch RViz together with the physical hardware.

## Launch Gazebo

*   **Launch Gazebo simulation:**
    Use this command to run a dynamic simulation of the AI Worker in the Gazebo simulator.  
    :::tabs key:robot-type
    == BG2 Type
    ```bash
    ros2 launch ffw_bringup ffw_bg2_follower_ai_gazebo.launch.py
    ```
    == SG2 Type
    ```bash
    ros2 launch ffw_bringup ffw_sg2_follower_ai_gazebo.launch.py
    ```
    :::

*   **View model in RViz only:**
    Use this command when you want to visualize the robot model in RViz without running a full simulation or the physical hardware.
    :::tabs key:robot-type
    == BG2 Type
    ```bash
    ros2 launch ffw_description ffw_bg2_rev4.launch.py
    ```
    == SG2 Type
    ```bash
    ros2 launch ffw_description ffw_sg2_rev1.launch.py
    # If the display looks incorrect, try adjusting the Fixed Frame
    ```
    :::

*   **Launch RViz with physical hardware:**
    Use this command when you want to operate the physical AI Worker hardware and monitor its status in RViz. This command will launch RViz alongside the hardware interface.
    :::tabs key:robot-type
    == BG2 Type
    ```bash
    ros2 launch ffw_bringup ffw_bg2_follower_ai.launch.py
    ```
    == SG2 Type
    ```bash
    ros2 launch ffw_bringup ffw_sg2_follower_ai.launch.py
    ```
    :::
    *   **Note:** This command is the same as the `Follower` launch command in the [`Teleoperation Guide`](/ai_worker/operation_ai_worker) and will also launch the cameras by default. To run without cameras, you can add the `launch_cameras:=false` argument.

## Simulation Views

### FFW_BG2

![FFW BG2 Simulation](/simulation/ai_worker/ffw_bg2_gazebo.png)


### FFW_SG2

![FFW SG2 Simulation](/simulation/ai_worker/ffw_sg2_gazebo.png)

## Launch Moveit
*   **Launch Moveit:**
    You can use the following command to launch Moveit for both FFW-BG2 and FFW-SG2:
    ```bash
    ros2 launch ffw_moveit_config moveit.launch.py
    # Launch Moveit with the AI Worker Follower model (FFW-BG2, FFW-SG2)
    ```

    If you are using Gazebo, you should add the argument `use_sim:=true`:
    ```bash
    ros2 launch ffw_moveit_config moveit.launch.py use_sim:=true
    # Launch Moveit in simulation mode
    ```
<img src="/release_note/ai_worker/ffw_moveit.gif" alt="AI Worker MoveIt">
