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
    ros2 launch robotis_hand_bringup hx5_d20_left_gazebo.launch.launch.py
    ```
    == Right
    ```bash
    ros2 launch robotis_hand_bringup hx5_d20_right_gazebo.launch.launch.py
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

### FFW_BG2

![FFW BG2 Simulation](/simulation/ai_worker/ffw_bg2_gazebo.png)


### FFW_SG2

![FFW SG2 Simulation](/simulation/ai_worker/ffw_sg2_gazebo.png)