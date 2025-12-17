# Simulation

## Overview
You can launch Gazebo and RViz individually, or launch RViz together with the physical hardware.

## 1. Prepare the Open Manipulator Docker Container

Refer to the `Setup Guide` to configure the Robotis Hand Docker Container environment.


## 2. Enable GUI Access

Open a new terminal and run the following command:

```bash
xhost +
```


## 3. Access the Running Container

```bash
./docker/container.sh enter
```


## 4. Launch Gazebo

*   **Launch Gazebo simulation:**

    :::tabs key:robot-type
    == Left
    ```bash
    ros2 launch robotis_hand_bringup hx5_d20_left_gazebo.launch.py
    ```
    == Right
    ```bash
    ros2 launch robotis_hand_bringup hx5_d20_right_gazebo.launch.py
    ```
    :::

*   **View model in RViz only:**
    Use this command when you want to visualize the robot model in RViz without running a full simulation or the physical hardware.
    :::tabs key:robot-type
    == Left
    ```bash
    ros2 launch robotis_hand_description hx5_d20_left.launch.py
    ```
    == Right
    ```bash
    ros2 launch robotis_hand_description hx5_d20_right.launch.py
    ```
    :::

## Simulation Views

### Left Hand

![Left Hand Simulation](/simulation/hands/hands_left_gazebo.png)


### Right Hand

![Right Hand Simulation](/simulation/hands/hands_right_gazebo.png)

## Launch Moveit
*   **Launch Moveit:**
    You can use the following command to launch Moveit for ROBOTIS HX Hand:
    :::tabs key:robot-type
    == Left
    ```bash
    ros2 launch robotis_hand_moveit_config hx5_d20_left_moveit.launch.py
    ```
    == Right
    ```bash
    ros2 launch robotis_hand_moveit_config hx5_d20_right_moveit.launch.py
    ```
    :::

    If you are using Gazebo, you should add the argument `use_sim:=true`:
    :::tabs key:robot-type
    == Left
    ```bash
    ros2 launch robotis_hand_moveit_config hx5_d20_left_moveit.launch.py use_sim:=true
    ```
    == Right
    ```bash
    ros2 launch robotis_hand_moveit_config hx5_d20_right_moveit.launch.py use_sim:=true
    ```
    :::
    
<img src="/simulation/hands/hands_moveit.gif" alt="ROBOTIS HX Hand MoveIt">