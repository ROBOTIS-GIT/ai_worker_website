# Getting Started with Gazebo

## Overview
You can launch Gazebo and RViz individually, or launch RViz together with the physical hardware.

## Launch Gazebo

*   **Launch Gazebo simulation:**
    Use this command to run a dynamic simulation of the AI Worker in the Gazebo simulator.
    ```bash
    ros2 launch ffw_bringup ffw_bg2_follower_gazebo.launch.py
    # Launch Gazebo with the AI Worker Follower model (FFW-BG2)
    ```

*   **View model in RViz only:**
    Use this command when you want to visualize the robot model in RViz without running a full simulation or the physical hardware.
    ```bash
    ros2 launch ffw_description model_view.launch.py
    # Launch RViz with the AI Worker model
    ```

*   **Launch RViz with physical hardware:**
    Use this command when you want to operate the physical AI Worker hardware and monitor its status in RViz. This command will launch RViz alongside the hardware interface.
    ```bash
    ros2 launch ffw_bringup ffw_bg2_follower_ai.launch.py
    # Launch the AI Worker Follower (FFW-BG2) hardware interface and RViz
    ```
    *   **Note:** This command is the same as the `Follower` launch command in the [`Teleoperation Guide`](/ai_worker/operation_ai_worker) and will also launch the cameras by default. To run without cameras, you can add the `launch_cameras:=false` argument.