# Getting Started with Gazebo

## Overview
You can launch Gazebo and RViz (Moveit) individually, launch RViz together with physical hardware, and now run Nav2 with SLAM Toolbox in simulation to map the environment and test navigation before deploying to the robot.

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
    ros2 launch ffw_bringup ffw_bg2_follower_ai.launch.py start_rviz:=true
    ```
    == SG2 Type
    ```bash
    ros2 launch ffw_bringup ffw_sg2_follower_ai.launch.py start_rviz:=true
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


## Launch Nav2 in Gazebo

Use Nav2 with the Gazebo simulation to test autonomous navigation before running on hardware.

1. **Start Gazebo simulation:** Use one of the Gazebo launch commands.
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

#### Gazebo View in the default world
![Gazebo Nav2 View](/simulation/ai_worker/gazebo_view1.png)

2. **Launch Nav2:**

   A default map matching the current gazebo world is provided. 
   Remap with SLAM Toolbox if you change the world or want a new map.

   - **Build a map with SLAM Toolbox:**
     ```bash
     ros2 launch ffw_navigation navigation.launch.py use_slam:=true use_sim_time:=true
     ```
     Nav2 starts with SLAM enabled so you can map while driving. In RViz, send goals with `2D Goal Pose` to explore and expand the map.

   #### SLAM Mapping Example  
   ![Nav2 SLAM Mapping](/simulation/ai_worker/nav2_slamtoolbox1.gif)

   #### Save a generated map
    If you ran SLAM and want to reuse the map, save it with:
    ```bash
    cd ~/ros2_ws/src/ai_worker/ffw_navigation/maps
    ros2 run nav2_map_server map_saver_cli -f ./map
    ```
    This produces `map.yaml` and `map.pgm` in the `maps` directory. Change the path with `-f /path/to/map_name` if needed.

   #### Saved Map Example
   Example of a saved occupancy grid generated with `map_saver_cli`:

   ![Saved map example](/simulation/ai_worker/my_map.png)
   The map uses a two-dimensional Occupancy Grid Map (OGM). White is free space, black is occupied, and gray is unknown.

## Navigation Using a Saved Map

- **Use a saved or default map:**
  ```bash
  ros2 launch ffw_navigation navigation.launch.py use_sim_time:=true
  ```
  Set the initial pose in RViz with `2D Pose Estimate` so AMCL can localize. After Nav2 is running, send a goal in RViz using `2D Goal Pose`. The robot will plan and drive to the target pose; if SLAM is enabled, the map updates in real time.
