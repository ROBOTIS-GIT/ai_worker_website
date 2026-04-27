# Technical Story

Welcome to the OMX Technical Story section. Here we share the vision-based robot control and motion optimization technologies implemented for the OMX.

The drawing system of OMX provides a complex control environment that converts task-space movements into stable real-time joint commands, going beyond simple shape drawing. By combining vision recognition data and a precise trajectory control algorithm, URDF parsing, Forward Kinematics (FK), Inverse Kinematics (IK), and a timer-based control loop work seamlessly together to draw complex shapes.

Let's dive into the technical stories behind OMX.

## Stories

Click on the card below to dive into the technical details of our drawing system:

<div style="display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 20px; margin: 30px 0;">

<details style="grid-column: 1 / -1;">
<summary style="list-style: none; cursor: pointer;">
  <div style="border: 2px solid #e0e0e0; border-radius: 12px; padding: 20px; transition: all 0.3s; background: linear-gradient(135deg, #1e3c72 0%, #2a5298 100%); color: white; min-height: 240px; display: inline-flex; flex-direction: column; justify-content: center; align-items: center; text-align: center; width: calc(66.666% - 14px); vertical-align: top;">
    <img src="/technical_story/omx_ik.png" alt="OMX IK" style="height: 160px; margin-bottom: 15px; border-radius: 8px;">
    <h3 style="margin: 0 0 10px 0; border: none; color: white;">OMX Drawing</h3>
    <div class="expand-indicator" style="margin-top: 15px; font-size: 12px; opacity: 0.8;">▼ Click to expand</div>
  </div>
</summary>
<div style="padding: 30px; border: 2px solid #1e3c72; border-radius: 12px; margin-top: 20px; background: #1b1b1f; color: #e0e0e0;">

Welcome to the OMX Technical Story. 
This section covers the process of performing precise drawing missions along contours extracted from an input image, using the `Cyclo Control` framework.

In particular, we introduce a drawing algorithm that maintains stability and tracks continuous trajectories even near kinematic singularities, using advanced image preprocessing via the `Shape Detector` node and a **QP (Quadratic Programming) optimization-based IK Solver**.

### ▶️ Full Demo

<div style="position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden; max-width: 100%; margin: 20px 0; border-radius: 8px; border: 2px solid #1e3c72;">
  <iframe
    style="position: absolute; top: 0; left: 0; width: 100%; height: 100%;"
    src="https://www.youtube.com/embed/7A-Y6VhFzaQ"
    title="OMX High-Precision Drawing Pipeline Demo"
    frameborder="0"
    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
    allowfullscreen>
  </iframe>
</div>

<p style="text-align: center; font-style: italic; color: #888; font-size: 14px; margin-bottom: 20px;">
  This video provides a comprehensive overview of the system architecture and the end-to-end drawing process.
</p>

---

## 1. Overview

This project focuses on real-time end-effector trajectory control of the manipulator. The core components, `shape_detector_node` and `omx_trajectory_controller_node`, handle image processing and motion control respectively, operating harmoniously through ROS 2 topic communication.

Specifically, the `shape_detector_node` applies **Adaptive Smoothing** to extract high-precision trajectories from visual data. These extracted paths are then passed to the `omx_trajectory_controller_node`, which parses the robot's URDF during runtime to perform FK and IK internally, maintaining an independent control loop without relying on a separate high-level motion planning framework.

The pipeline utilizes Cyclo Control, a numerical Inverse Kinematics solver based on QP (Quadratic Programming) optimization, to calculate joint trajectories. Trajectory data from the vision node is processed through this controller and converted into real-time joint commands, ensuring smooth and precise motion on the drawing plane via a timer-based control loop.

### Key Packages and File Structure

*   `scripts/shape_detector_node.py`: Handles image preprocessing and trajectory extraction.
*   `scripts/omx_trajectory_controller_node.py`: Handles trajectory reception and real-time motion control.
*   `launch/omx_drawing.launch.py`: The launch file that integrates and runs the entire system.

![System Architecture](/technical_story/system_architecture.png)

---

## 2. Kinematic Background

The core of manipulator control is defining the relationship between each joint angle of the robot and the physical position of the end-effector.

### 2.1 Forward Kinematics (FK) vs Inverse Kinematics (IK)

| Category | Forward Kinematics (FK) | Inverse Kinematics (IK) |
| --- | --- | --- |
| **Input** | Joint angles \(q\) | Target end-effector pose \(x_d\) |
| **Output** | End-effector position and orientation \(x\) | Joint angles \(q\) |
| **Mapping Direction** | Joint Space → Task Space | Task Space → Joint Space |
| **Uniqueness of Solution** | Unique solution always exists | Multiple solutions may exist, or none |
| **Computational Complexity** | Relatively simple matrix operations | Numerical methods or optimization needed |
| **Role in Control** | State evaluation and visualization | Real-time control and trajectory tracking |


### 2.2 Accurate Pen Tip Position Tracking via Pen_Link Addition

The Jacobian matrix provides a differential mapping that converts minute changes in Joint Space into Task Space velocities, which forms the core of task space control. In this drawing system, the geometric Jacobian is recalculated at every control step by parsing the modified URDF model at runtime, and this is shared across both forward and inverse kinematics operations.

#### Advanced TCP (Tool Center Point) Definition via Virtual Link

Standard robot models generally define the center of the gripper as the end-effector. However, for precise drawing using a pen, the position of the pen tip extending from the hardware mount must be the reference point for control (TCP). To achieve this, a hierarchical link structure starting from `end_effector_link` needs to be added to the URDF, considering the actual location where the pen tip is attached.

| Physical Pen Setup | RViz Pen Link Visualization |
| --- | --- |
| ![Physical Pen Setup](/technical_story/pen_link.png) | ![RViz Pen Link Visualization](/technical_story/pen_link_rviz.png) |




```xml
<!-- 1. Existing end-effector (gripper center) definition -->
<joint name="end_effector_joint" type="fixed">
  <parent link="link5"/>
  <child link="end_effector_link"/>
  <origin rpy="0 0 0" xyz="0.09193 -0.0016 0"/>
</joint>
<link name="end_effector_link">
  <visual>
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <geometry><box size="0.01 0.01 0.01"/></geometry>
    <material name="red"><color rgba="1.0 0.0 0.0 1"/></material>
  </visual>
</link>

<!-- 2. Added Pen Tip TCP definition -->
<joint name="pen_joint" type="fixed">
  <parent link="end_effector_link"/>
  <child link="pen_link"/>
  <origin rpy="0 0 -0.1571" xyz="0.01 0.015 0"/>
</joint>
<link name="pen_link">
  <visual>
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <geometry><box size="0.01 0.01 0.01"/></geometry>
    <material name="red"><color rgba="1.0 0.0 0.0 1"/></material>
  </visual>
</link>
```
::: info **Original URDF File Path**
`ros2_ws/src/cyclo_control/cyclo_motion_controller_models/models/omx/omx_f.urdf`
:::

With this modification, the IK solver now computes joint angles targeting the **coordinates of `pen_link`**, which actually touches the paper, rather than the center of `end_effector_link`. The core of the high-precision drawing implemented in this project lies in the following **measurement-based calibration**.

1.  **Applying Measured Parameters**: The `origin` data (`xyz="0.01 0.015 0" rpy="0 0 -0.1571"`) of the `pen_joint` is not a simple estimate. It is the result of precisely and physically measuring the **x_offset (10mm)**, **y_offset (15mm)**, and the pen's tilt **Yaw_angle (-9 degrees)** of the pen attached to the actual hardware.
2.  **Hierarchical Calibration**: The `end_effector_link` -> `pen_link` tree structure allows intuitive projection of physical measurements into the model, and even if the mounting state changes, immediate response is possible simply by modifying the URDF parameters.
3.  **Maximizing IK Precision**: By synchronizing the error between the hardware and the virtual model to sub-millimeter levels, the 2D plane tracking performance of the pen tip can be maximized without additional compensation calculations at the algorithm stage.
4.  **Visual Debugging**: By adding visualization elements (`<visual>`) to the virtual link, you can verify in real-time whether the pen tip's position in RViz matches the actual hardware. Use the command below to instantly verify if the modified URDF reflects your intentions visually.

    ```bash
    ros2 launch cyclo_motion_controller_models view_omx_f.launch.py
    ```
![TCP Calibration Result](/technical_story/pen_point.png)

#### Stable Real-Time Control Based on QP Optimization
To ensure stability in a real-time execution environment, the inverse kinematics problem is solved using a **QP (Quadratic Programming) optimization** approach. By directly reflecting joint limits as constraints, it ensures the physical safety of the hardware while enabling smooth and reliable drawing trajectory tracking.

#### Relaxing Collision Constraints for QP Solver Optimization
The QP (Quadratic Programming) optimization-based solver calculates joint limits and distance constraints in real-time to find the optimal path. If complex `collision` tags are active in the URDF model, they can unnecessarily increase computational load or cause unintended self-collision constraints that may block motion during precise drawing. Therefore, to ensure smooth drawing execution, it is recommended to disable collision conditions for links by commenting out their `collision` tags, as shown in the example below.

```xml
<link name="link1">
  <visual>
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <geometry>
      <mesh filename="package://open_manipulator_description/meshes/omx_f/follower_01_base.stl" scale="0.001 0.001 0.001"/>
    </geometry>
  </visual>
  <!-- Comment out collision tags using complex mesh data to reduce QP solver load. -->
  <!-- <collision>
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <geometry>
      <mesh filename="package://open_manipulator_description/meshes/omx_f/follower_01_base.stl" scale="0.001 0.001 0.001"/>
    </geometry>
  </collision> -->
</link>
```

---

## 3. Controller and Motion Control Parameter Optimization

For precise drawing, consistent parameter optimization from the hardware interface to the high-level controller is essential. This section details the key settings modified for high-performance trajectory tracking and the reasoning behind their optimization.

### 3.1 Cyclo Control and Real-Time Controller Settings

The `omx_movel_controller` is responsible for the linear movement of the end-effector. To maximize drawing precision and hardware safety, its parameters have been optimized as follows:

```yaml
omx_movel_controller:
  ros__parameters:
    control_frequency: 100.0        # Control loop frequency (Hz)
    time_step: 0.01                 # Computation timestep (1/frequency)
    
    kp_position: 50.0               # Task space position gain
    kp_orientation: 50.0            # Task space orientation gain

    weight_task_position: 1500.0    # Position tracking weight (Core for high-precision drawing)
    weight_task_orientation: 1.0    # Orientation maintenance weight
    weight_damping: 0.001           # Damping weight to avoid singularities
    slack_penalty: 1000.0           # Constraint relaxation penalty (Optimization stability)
    
    cbf_alpha: 5.0                  # CBF (Control Barrier Function) strength
    collision_buffer: 0.01          # Collision prevention buffer (m)
    collision_safe_distance: 0.005  # Collision safe distance (m)

    controlled_link: "pen_link"     # Target link to control (TCP)
```
::: info **Original Configuration File Path**
`ros2_ws/src/cyclo_control/cyclo_motion_controller_ros/config/omx_config.yaml`
:::

#### Detailed Parameter Explanation

1.  **Control Performance and Tracking**
    *   **Weight Task Position (1500.0)**: Sets the position tracking weight in the task space very high, making it the top priority for the robot to accurately follow the input drawing trajectory over other secondary constraints.
    *   **KP Position/Orientation (50.0)**: The key gain for converting position/orientation errors in task space into linear and angular velocity commands, respectively. Similar to generating velocity commands to reach a target point in mobile base control, it acts as an intermediary to generate a virtual velocity vector so the end-effector can track the trajectory.
    *   **Controlled Link (pen_link)**: Sets the previously defined virtual pen tip link as the control reference, performing precise drawing by minimizing physical errors.

2.  **Optimization and Stability**
    *   **Weight Damping (0.001)**: A damping component to increase the stability of the control loop and suppress minute motor vibrations (jitter).

3.  **Interface Topic Configuration**
    *   **Joint States Topic (`/joint_states`)**: Receives the current joint state of the robot.
    *   **Joint Command Topic (`/leader/joint_trajectory`)**: Sends calculated joint trajectory commands to the lower-level controller (ros2_control).
    *   **MoveL Topic (`~/movel`)**: A topic that receives linear movement commands.
    *   **EE Pose Topic (`~/current_pose`)**: Publishes the current pose of the end-effector in real-time to assist with monitoring.
    *   **Controller Error Topic (`~/controller_error`)**: A topic for monitoring the tracking error with the target trajectory in real-time.

### 3.2 ros2_control and Dynamixel PID Tuning

PID and profile parameters have also been optimized at the Dynamixel hardware level for rapid and precise response.

::: info **Original URDF File Path**
`open_manipulator_description/ros2_control/omx_f.ros2_control.xacro`
:::

| Item | Before (Default) | After (Optimized) | Role and Reason for Optimization |
| :--- | :--- | :--- | :--- |
| **Position I Gain** | 0 | **1000** | Eliminates steady-state error and overcomes frictional resistance |
| **Position D Gain** | 1000 | **1200** | Suppresses overshoot and vibration by improving braking performance |


- **I Gain**: Ensures that the target coordinates are perfectly reached despite the slight friction generated when the pen tip touches the paper.
- **D Gain**: Minimizes shaking during high-speed movements and sudden changes in direction.
---

## 4. Software Detailed Configuration and Algorithms

These are the detailed technical specifications of the vision processing and motion control algorithms.

#### 4.1 Vision Recognition and Data Preprocessing (Shape Detector Node)

`shape_detector_node.py` extracts precise linear data from the input image, considering the mechanical characteristics of the robot and the quality of the drawing. This process consists of three main algorithmic steps.

![Original Image](/technical_story/person.jpeg)
[Image Designed by Freepik](https://www.freepik.com/)

1.  **Step 1 (Image Analysis and Preprocessing)**:
    *   **Bilateral Filter**: Unlike simple Gaussian blur, it effectively removes only noise while preserving edge information.
        ![Bilateral Filter](/technical_story/bilateral_filter.jpg)
    *   **Adaptive Thresholding**: Performs robust binarization against lighting changes, and uses **Morphological Closing** operations to fill broken lines or tiny gaps in text, ensuring structural integrity.

    ![Adaptive Thresholding](/technical_story/threshold_&_closing.jpg)
    
2.  **Step 2 (Skeletonization and Single Line Extraction)**:
    *   **Skeletonization**: Converts the binarized outline into a 1-pixel thick centerline, extracting only the geometric skeleton.

![Skeletonization Result](/technical_story/skeletonization.jpg)

  *   **Line Extraction (Pixel Sorting)**: To extract valid point cloud data that the robot can follow without interruption, it applies a sorting algorithm (`extract_single_line`) based on **Nearest Neighbor** to the skeletonized pixels. By calculating the Euclidean distance between pixels, it sequentially connects the closest unvisited pixel within a threshold (50px²) from the current position.
  *   **Segment Generation**: The connected pixels are grouped into a single valid stroke sector that the robot can follow without stopping. If there are no neighboring pixels within the threshold, a new segment is created to prevent sudden jumps or discontinuities.

![Line Extraction Flow](/technical_story/stage_2_line_extraction.jpg)

3.  **Step 3 (Adaptive Smoothing & Optimization)**:
    *   **Douglas-Peucker Algorithm**: Detects vertices with sharp curvature across the entire trajectory. This preserves straight sections and selectively applies smoothing only to curved sections, preventing them from becoming blunt.
    *   **Trajectory Stitching**: If the distance between two adjacent trajectories is within 25mm, they are forcibly merged into one continuous path, minimizing unnecessary pen-lift counts.
    *   **WMA (Weighted Moving Average)**: Finally, a weighted moving average filter corrects trajectory jitter and publishes it to the `/drawing_trajectory` topic.

  ![Robotic Trajectory](/technical_story/trajectory.jpg)

#### 4.2 Trajectory Control and Motion Generation (Trajectory Controller & Cyclo Control)
- **Waypoint Sorting & Path Planning**: Sorts the collected trajectory points using a Nearest Neighbor algorithm and plans the path in three phases: Approach, Drawing, and Home.
- **MoveL Based Linear Control**: Performs real-time linear interpolation to the target pose via `Cyclo Control`, ensuring the end-effector maintains a straight path.
- **QP Based IK Solver**: Delivers calculated joint commands to the hardware through a QP optimization algorithm that considers kinematic singularities and joint limits.

---

## 5. Technology Verification via Simulation

This is the process of verifying the validity of the trajectory in advance through Gazebo and RViz before applying it to actual hardware.
 Execute the following, using three terminals for steps 1–3:

1. **OMX Mock-hardware Bring up in rviz**
    ```bash
    cd open_manipulator/docker
    ./container.sh enter
    ros2 launch open_manipulator_bringup omx_f.launch.py start_rviz:=true use_mock_hardware:=true
    ```
2. **Execute Cyclo Control**
  
    For detailed execution instructions, please refer to the link below.
    [Cyclo Control](/omx/advanced_motion_controller_omx)
    ```bash
    cd open_manipulator/docker
    ./container.sh enter
    colcon build
    source install/setup.bash
    ros2 launch cyclo_motion_controller_ros omx_controller.launch.py
    ```

3. **Start Simulation Environment**:
   ```bash
   cd open_manipulator/docker
   ./container.sh enter
   ros2 launch open_manipulator_playground omx_drawing.launch.py
   ```
4. **RViz Monitoring**: Visually check if the generated points are stably distributed within the robot's workspace.

![RViz Monitoring](/technical_story/drawing_circle.gif)

5. **Check IK Convergence**: Use Plot Juggler to monitor IK convergence and tracking performance by comparing the movel target trajectory topic (`/omx_movel_controller/movel`) with the current end-effector pose topic (`/omx_movel_controller/current_pose`) in real-time.

![PlotJuggler IK Convergence](/technical_story/plotjuggler.gif)

---

## 6. Practical Operation and Parameter Usage Guide

To perform drawing using the actual OMX, execute the following commands in three terminals.

1. **OMX Bring up**
    ```bash
    cd open_manipulator/docker
    ./container.sh enter
    ros2 launch open_manipulator_bringup omx_f.launch.py
    ```
2. **Execute Cyclo Control**
  
    For detailed execution instructions, please refer to the link below.
    [Cyclo Control](/omx/advanced_motion_controller_omx)
    ```bash
    cd open_manipulator/docker
    ./container.sh enter
    colcon build
    source install/setup.bash
    ros2 launch cyclo_motion_controller_ros omx_controller.launch.py
    ```

3. **Execute OMX_drawing launch file**:
   ```bash
   cd open_manipulator/docker
   ./container.sh enter
   ros2 launch open_manipulator_playground omx_drawing.launch.py
   ```

The omx_drawing launch file provides the options below.

```bash
ros2 launch open_manipulator_playground omx_drawing.launch.py \
    image_path:=/path/to/robotis2.png \
    drawing_height:=0.025 \
    smoothing_sigma:=1.0 \
    resample_num_pts:=100 \
    joint5_angle:=90.0 \
    home_x:=0.124 \
    home_y:=0.0 \
    home_z:=0.081 \
    approach_duration:=2.0 \
    home_duration:=4.0
```

### Detailed Available Parameters Guide

Key options applied during the execution of `omx_drawing.launch.py` are critical variables directly related to the robot's drawing quality and operational stability. A detailed guide with comments is provided.

| Category | Parameter | Default Value | Main Role and Tuning Tips |
| :--- | :--- | :--- | :--- |
| **Vision & Recognition** | `image_path` | `robotis2.png` | **Input Image Path**: Specifies the absolute path of the image for the robot to recognize. |
| **Vision & Recognition** | `smoothing_sigma` | `1.0` | **Noise Filter Strength**: Larger values create smoother curves, but if too large, details are blurred. Lower values are recommended for low-resolution images. |
| **Drawing Precision** | `drawing_height` | `0.025` | **Pressure Control**: The Z height of the paper surface. You can optimize line thickness and pressure by fine-tuning in 0.001m units. |
| **Drawing Precision** | `resample_num_pts` | `100` | **Trajectory Resolution**: The number of points configuring one stroke. For complex logos, increase the value to ensure precision. |
| **Kinematics Setting** | `joint5_angle` | `90.0` | **Tool Angle**: Fixes the pen holder's angle vertically (90 degrees) to fix the pen's position based on where the pen is attached. |
| **Kinematics Setting** | `home_x, y, z` | `0.124, 0, 0.081` | **Safe Pose**: The manipulator's position to return to after mission completion or at start. Defines the robot's initial standby state. |
| **Operational Sequence** | `approach_duration`| `2.0` | **Approach Speed**: Travel time from home to the starting point of the first stroke. Protects the mechanism by preventing sudden acceleration. |
| **Operational Sequence** | `home_duration` | `4.0` | **Return Speed**: Time to return to home after all drawing is completed. Ensures safety through a relaxed movement. |

#### Parameter Usage Tips for Experts

1.  **Precise Pressure Control**: `drawing_height` is the distance between the pen tip and the ground during drawing. It is optimal for the pen tip to slightly touch the paper and bend in actual hardware. If the floor surface is uneven, you should test while adjusting this value up/down in mm units.
2.  **Complex Shape Optimization**: When drawing complex patterns, it is recommended to increase `resample_num_pts`. Reducing the resampling interval is suitable for complex drawings.
3.  **Preserving Drawing Details**: If fine details in the image (e.g., text, thin lines of a logo) are erased, try reducing `smoothing_sigma` to 0.5~0.8. Conversely, if the outline is too jagged, increasing this value can yield a smoother appearance.
4.  **Safe Operation**: By setting `approach_duration` and `home_duration` loosely (5 seconds or more), you can reduce the inertial force generated when the robot moves to the target point, preventing step-out (desynchronization) phenomena and extending motor life.

### 6.1 Following motion from Images

This final stage demonstrates the integrated success of the drawing pipeline.

#### 6.2 Trajectories Extracted from Contour

This is the trajectory generated from the detected contours. It represents the final path the robot will follow after completing all image processing, skeletal extraction, and adaptive smoothing stages, ensuring a kinematically feasible route for the manipulator.

![Generated Trajectories](/technical_story/trajectories.gif)

#### 6.3 Actual Robot Drawing along the Trajectory

Controlled by the QP-based IK solver and optimized Dynamixel PID gains, the robot reproduces the complex contours of the input image

![Real Robot Drawing](/technical_story/drawing_person.gif)

</div>
</details>

</div>

<style>
details > summary::-webkit-details-marker {
  display: none;
}
details > summary {
  list-style: none;
}
details > summary > div:hover {
  transform: translateY(-5px);
  box-shadow: 0 10px 25px rgba(0,0,0,0.2);
}
details[open] > summary > div {
  border-radius: 12px;
}
details[open] > summary .expand-indicator {
  display: none;
}
</style>

---