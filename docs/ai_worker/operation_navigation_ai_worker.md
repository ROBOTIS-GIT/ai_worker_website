# Navigation

This guide shows how to bring up the AI Worker's navigation stack so the robot can move autonomously after basic setup.

## Prerequisites
- Complete the hardware and setup steps in the **Setup Guide**.
- Ensure the robot is on level ground with sufficient clearance for safe autonomous driving.
- Connect to the robot PC (keyboard/monitor or SSH) and make sure the battery is charged.

## Bring Up the Robot
Before launching Nav2, start the SG2 follower using the commands from the Teleoperation Guide.
Run these inside the Docker container:
```bash
cd ~/ai_worker
./docker/container.sh enter
ros2 launch ffw_bringup ffw_sg2_follower_ai.launch.py
```
Keep these terminals running. use a new terminal for the commands below.

## View RViz via noVNC (headless/SSH)
RViz cannot be displayed over plain SSH. Use noVNC to launch RViz and view it from a browser.

::: tip
- If `192.168.6.2:8090` is unreachable, confirm the host IP and the forwarded port in your environment.
:::

At the `Robot PC`, run the following commands:

1. Build and start the noVNC container:

#### method 1
Update `ai_worker` repository and run `container.sh start-novnc`.  
(⚠️ **Warning:** When you doing `git pull` in the `ai_worker` repository, existing container data may be lost! If you want to keep your container data, use **method 2** instead.)
```bash
cd ~/ai_worker
./docker/container.sh start-novnc
```
#### method 2
Make a `docker-compose.yml` file with the following content:
  ::: details docker-compose.yml
  ```yml
  services:
    novnc-server:
      container_name: novnc-server
      image: robotis/novnc-server:latest
      restart: no
      cap_add:
        - SYS_NICE
      ulimits:
        rtprio: 99
        rttime: -1
        memlock: 8428281856
      network_mode: host
      environment:
        - ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-30}
        - DISPLAY=:99
        - DISPLAY_WIDTH=1920
        - DISPLAY_HEIGHT=1080
        - WEBSOCKIFY_PORT=8090
      volumes:
        - /dev:/dev
        - /dev/shm:/dev/shm
      privileged: true
  ```
  :::
Then run `docker-compose up` to start the noVNC container.


2. Open noVNC in a browser
In a web browser, open `http://ffw-snpr48a{serial-number}.local:8090` (substituting your robot's serial number) or `http://192.168.6.2:8090`. After the page loads, click **Connect**.  


4. Run RViz

## Lidar Scan Visualization
First, use RViz (via noVNC) to confirm lidar scans are visible around the robot.

![Lidar scan visualization](/simulation/ai_worker/laserscan.png)
Lidar scan values visualized in RViz. Use this view to confirm the sensor is publishing after connecting via noVNC.
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

    #### SLAM Mapping Process
    ![SLAM mapping (Nav2 + SLAM Toolbox)](/simulation/ai_worker/MAPPING.gif)
    White areas show free space discovered by SLAM, black indicates obstacles or walls, and gray is unknown space. As the robot drives, the explored (white) region expands and the unknown area shrinks while the map is refined.

    #### Save a generated map
    If you ran SLAM and want to reuse the map, save it with:
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
