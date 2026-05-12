# Navigation

This guide shows how to bring up the AI Worker's navigation stack so the robot can move autonomously after basic setup.

## Prerequisites
- Complete the hardware and setup steps in the **Setup Guide**.
- Ensure the robot is on level ground with sufficient clearance for safe autonomous driving.
- Connect to the robot PC (keyboard/monitor or SSH) and make sure the battery is charged.

## Bring Up the Robot
Before launching Nav2, start the follower using the commands from the Teleoperation Guide.
Run these inside the Docker container:
```bash
cd ~/ai_worker
./docker/container.sh enter
```

:::tabs key:robot-type
== Gripper Type
Note that `sg2` model is used as an example.
```bash
ros2 launch ffw_bringup ffw_sg2_follower_ai.launch.py
```
Or use the shortcut:
```bash
ffw_sg2_follower_ai
```

== Hand Type
Note that `sh5` model is used as an example.
```bash
ros2 launch ffw_bringup ffw_sh5_follower_ai.launch.py
```
Or use the shortcut:
```bash
ffw_sh5_follower_ai
```
:::

Keep these terminals running. Use a new terminal for the commands below.

## View RViz in Local

On the `User PC`, open a terminal and enter the `ai_worker` Docker container:
```bash
cd ~/  # or your preferred directory
git clone -b jazzy https://github.com/ROBOTIS-GIT/ai_worker.git
cd ai_worker
./docker/container.sh enter
```

Start RViz with the navigation configuration file:
```bash
rviz2 -d ~/ros2_ws/src/ai_worker/ffw_navigation/rviz/navigation.rviz
```

::: warning
Make sure the `ROS_DOMAIN_ID` in the User PC Docker container matches the `ROS_DOMAIN_ID` used by the AI Worker robot. If the domain IDs are different, RViz will not receive robot topics such as `/scan`.
:::

On the `User PC`, check that `/scan` data is being received:
```bash
ros2 topic echo /scan
```

## Lidar Scan Visualization
First, use RViz to confirm lidar scans are visible around the robot.

![Lidar scan visualization](/simulation/ai_worker/laserscan.png)
Lidar scan values visualized in RViz. Use this view to confirm the sensor is publishing correctly.
::: warning
In RViz, set the `scan` topic Reliability Policy to `Best Effort` for this display.
:::

## Launch the Navigation Stack
All commands below run on the `robot PC` in a new terminal after the bringup above is running.

1. **Enter the Docker container** in the new terminal:
    ```bash
    cd ~/ai_worker
    ./docker/container.sh enter
    ```
2. **Start Nav2**:

    ::: warning
    Make sure the bringup terminals stay running before launching Nav2, otherwise the navigation stack will fail to connect to sensors and actuators.
    :::


    The package includes only a default Gazebo map. Create your own map with SLAM Toolbox for your environment.

     **Build a map with SLAM Toolbox:**
      ```bash
      ros2 launch ffw_navigation navigation.launch.py use_slam:=true
      ```
      Nav2 starts with SLAM enabled so you can map while driving. <u>Pick **one** of the two exploration methods below:</u>
      - **Autonomous:** In RViz, send a goal with `2D Goal Pose`. the robot will plan, drive, and expand the map as it moves.
      - **Keyboard teleop:** In a new terminal, run:
        ```bash
        cd ~/ros2_ws/src/ai_worker/ffw_teleop/ffw_teleop
        python3 mobile_teleop.py
        ```
        Drive with the keyboard to explore and build the map.

    #### View Mapping in RViz
    On the `User PC`, open a new terminal and enter the `ai_worker` Docker container:
    ```bash
    cd ~/ai_worker
    ./docker/container.sh enter
    ```

    Start RViz with the mapping configuration file:
    ```bash
    rviz2 -d ~/ros2_ws/src/ai_worker/ffw_navigation/rviz/mapping.rviz
    ```

    Use this RViz view to monitor the map, lidar scan, robot pose, and navigation status while SLAM mapping is running.

    #### SLAM Mapping Process
    ![SLAM mapping (Nav2 + SLAM Toolbox)](/simulation/ai_worker/MAPPING.gif)
    White areas show free space discovered by SLAM, black indicates obstacles or walls, and gray is unknown space. As the robot drives, the explored (white) region expands and the unknown area shrinks while the map is refined.

    #### Save a generated map
    On the `Robot PC`, if you ran SLAM and want to reuse the map, save it with:
    ```bash
    cd ~/ros2_ws/src/ai_worker/ffw_navigation/maps
    ros2 run nav2_map_server map_saver_cli -f ./map
    ```
    This produces `map.yaml` and `map.pgm` in the `maps` directory. Change the path with `-f /path/to/map_name` if needed.

## Navigation Using a Saved Map

- **Use a generated map:**
  ```bash
  ros2 launch ffw_navigation navigation.launch.py
  ```
  ::: tip
  To use a different map, place your `map.yaml` and image file in the `maps/` folder of the `ffw_navigation` package.
  :::
  Set the initial pose in RViz with `2D Pose Estimate` so AMCL can localize. After Nav2 is running, send a goal in RViz using `2D Goal Pose`. The robot will plan and drive to the target pose.
  if SLAM is enabled, the map updates in real time.

### View Saved Map Navigation in RViz
On the `User PC`, open a new terminal and enter the `ai_worker` Docker container:
```bash
cd ~/ai_worker
./docker/container.sh enter
```

Start RViz with the navigation configuration file:
```bash
rviz2 -d ~/ros2_ws/src/ai_worker/ffw_navigation/rviz/navigation.rviz
```

### RViz Goal on Saved Map
![RViz goal and path with saved map](/simulation/ai_worker/aiw_rviz2.gif)
Setting `2D Goal Pose` in RViz on a saved map. the plan is sent to the real robot for execution.
### Real-World Execution
![Real robot following planned path](/simulation/ai_worker/navigation_curve.gif)
AI Worker following the planned curved path on the saved map and driving to the goal.

## Troubleshooting
- If the `/scan` display is missing in RViz, set the `scan` topic Reliability Policy to `Best Effort`.
- If the map is not visible in RViz, set the map topic Durability Policy to `Transient Local`.

## Tuning Guide
- **AMCL localization:** Adjust parameters in `config/amcl_localization.yaml`.
- **Nav2 stack (costmaps, BT navigator, controller, planner):** Adjust parameters in `config/navigation.yaml`.
- **For detailed parameter descriptions across Nav2, see https://docs.nav2.org/index.html.**

## Safety and Usage Tips
- Keep people and obstacles clear while the robot is localizing or moving.
- If you need to pause, use RViz to clear goals or stop the navigation nodes with `Ctrl+C` in the running terminals.
- For simulation-based testing (recommended before field deployment), see the Nav2 section in Gazebo.
