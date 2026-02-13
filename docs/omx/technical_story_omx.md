# Technical Story

Welcome to the OMX Technical Story.
Here, we share how simple kinematics and control logic come together to make the manipulator move smoothly and predictably.

End-effector trajectory control isn’t just about drawing shapes. It’s about turning task-space motion into stable joint commands in real time. In this story, we introduce traj_pub_node, a ROS2 node that generates continuous trajectories and tracks them using a Jacobian-based IK solver with Damped Least Squares for stability.

Let’s take a look at how URDF parsing, FK/IK, and a timer-based control loop work together to draw circles, rectangles, and even heart-shaped paths in simulation.

## Stories

Click on the card below to explore the AI technologies we've implemented:

<div style="display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 20px; margin: 30px 0;">

<details style="grid-column: 1 / -1;">
<summary style="list-style: none; cursor: pointer;">
  <div style="border: 2px solid #e0e0e0; border-radius: 12px; padding: 20px; transition: all 0.3s; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: white; min-height: 200px; display: inline-flex; flex-direction: column; justify-content: center; align-items: center; text-align: center; width: calc(66.666% - 14px); vertical-align: top;">
    <img src="/technical_story/omx_ik.png" alt="Real-Time IK Shape Drawing" style="width: 100%; max-width: 420px; height: auto; margin-bottom: 15px; object-fit: contain; background: transparent; border-radius: 8px;">
    <h3 style="margin: 0 0 10px 0; border: none; color: white;">Jacobian IK · Real-Time Control</h3>
    <div class="expand-indicator" style="margin-top: 15px; font-size: 12px; opacity: 0.8;">▼ Click to expand</div>
  </div>
</summary>
<div style="padding: 30px; border: 2px solid #667eea; border-radius: 12px; margin-top: 20px; background: #1b1b1f; color: #e0e0e0;">


### ▶️ Full Demo

<div style="position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden; max-width: 100%; margin: 20px 0; border-radius: 8px;">
  <iframe
    style="position: absolute; top: 0; left: 0; width: 100%; height: 100%;"
    src="/technical_story/technical_story_omx.mp4"
    title="OMX Technical Story Demo"
    frameborder="0"
    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
    allowfullscreen>
  </iframe>
</div>

---

### 1. Overview

This project focuses on real-time end-effector trajectory control for robotic manipulators. We introduce traj_pub_node, a ROS 2 node that generates continuous task-space trajectories and tracks them using a Jacobian-based inverse kinematics solver.

The node directly parses the robot’s URDF at runtime and performs forward and inverse kinematics internally, without relying on high-level motion planning frameworks. To maintain stable motion near singular configurations, a Damped Least Squares formulation is applied.

The system supports multiple parametric trajectories—including circular, rectangular, and heart-shaped paths—defined in selectable planes. Joint commands are streamed in real time through a timer-based control loop, enabling smooth motion execution and clear visualization in RViz. The overall behavior and stability of the system are validated through simulation-based demonstrations.

---

### 2. Kinematic Background
#### 2.1 Forward Kinematics vs Inverse Kinematics

| Category | Forward Kinematics (FK) | Inverse Kinematics (IK) |
| --- | --- | --- |
| Input | Joint angles \(q\) | Desired end-effector pose \(x_d\) |
| Output | End-effector pose \(x\) | Joint angles \(q\) |
| Mapping direction | Joint space → Task space | Task space → Joint space |
| Solution uniqueness | Always unique | Multiple or no solutions |
| Computational complexity | Relatively simple | Requires numerical methods |
| Role in control | State evaluation and visualization | Real-time control and trajectory tracking |

<p style="margin-top:8px; font-size:13px; color:#666; text-align:center; max-width:680px;">
    <strong>Figure 2.1</strong> Conceptual illustration of Forward Kinematics (FK) and Inverse Kinematics (IK)
  </p>

#### 2.2 Jacobian-Based Inverse Kinematics (with Damped Least Squares)

<div style="display:flex; flex-direction:column; align-items:center; justify-content:center; margin: 18px 0;">
  <img src="/technical_story/jacobian_ik.png" alt="Jacobian IK illustration" style="max-width:320px; width:auto; height:auto; display:block; border-radius:8px; border:1px solid #ddd;">
  <p style="margin-top:8px; font-size:13px; color:#666; text-align:center; max-width:680px;"><strong>Figure 2.2</strong> Conceptual illustration of Jacobian-based IK</p>
</div>
The Jacobian matrix provides the differential mapping between joint space and task space, forming the core of task-space control. In this work, a URDF-based geometric Jacobian is recomputed at each control step and shared by both forward and inverse kinematics.

To achieve stable real-time execution, inverse kinematics is solved using a Damped Least Squares formulation. The damping term regularizes the Jacobian inversion near singular configurations, limiting excessive joint updates and enabling smooth and reliable trajectory tracking.

---

### 3. System Architecture and Execution Flow

#### 3.1 System Architecture
<div style="display:flex; flex-direction:column; align-items:center; justify-content:center; margin: 18px 0;">
  <img
    src="/technical_story/architecture.png"
    alt="System Architecture"
    style="max-width:600px; width:auto; height:auto; display:block; border-radius:8px; border:1px solid #ddd;">
  <p style="margin-top:8px; font-size:13px; color:#666; text-align:center; max-width:680px;">
    <strong>Figure 3.1</strong> System Architecture
  </p>
</div>
This section describes the overall system architecture of `traj_pub_node` and its
execution flow during real-time trajectory tracking.
Figure 3.1 illustrates how task-space trajectories, kinematic computation, and
joint command streaming are integrated within a single ROS 2 node operating as a
closed-loop control system.

The node directly parses the robot description (URDF) to construct the kinematic
chain. During execution, forward kinematics and the corresponding Jacobian matrix
are recomputed at each control cycle based on the current joint state feedback.
Task-space trajectory generation, inverse kinematics, and joint command publishing
are tightly coupled within a timer-driven feedback loop.

#### 3.2 Execution Flow
`traj_pub_node` operates as a timer-driven, real-time closed-loop control system.
At a fixed update rate, the node continuously converts task-space motion into
joint-level commands using feedback from the robot's current joint state.

At each timer callback, the following steps are executed:

1. A trajectory parameter is computed from the elapsed time.
2. The desired end-effector position is generated in task space.
3. Forward kinematics and the Jacobian matrix are recomputed using the current joint state.
4. Inverse kinematics is solved using the current joint configuration as the initial guess.
5. The resulting joint configuration is published as a `JointTrajectory` command.

The executed joint motion produces updated joint states, which are fed back into
the next control cycle. By streaming a single trajectory point at each iteration,
the system achieves smooth and responsive motion while maintaining robustness to
inverse kinematics failures, as illustrated in Figure 3.1.

---

### 4. How to Run 
#### 4.1 Simulation Setup
Gazebo is launched to simulate the OMX manipulator. Run the following commands in a terminal:
```bash
source /opt/ros/jazzy/setup.bash
cd ~/open_manipulator
source install/setup.bash

ros2 launch open_manipulator_bringup omx_f_gazebo.launch.py
```
To visualize the robot model, kinematic frames, and trajectory execution, RViz is executed with a predefined configuration file:
```bash
source /opt/ros/jazzy/setup.bash
cd ~/open_manipulator
source install/setup.bash

rviz2 -d $(ros2 pkg prefix --share open_manipulator_description)/rviz/open_manipulator.rviz
```

#### 4.2 Running traj_pub_node
The node is launched with the following command:
```bash
source /opt/ros/jazzy/setup.bash
cd ~/open_manipulator
source install/setup.bash

python3 open_manipulator_playground/src/drawing_shape_omx.py --ros-args -p interactive:=true

```

#### 4.3 Runtime Parameters (Interactive Mode)
`traj_pub_node` supports an interactive mode where users configure the trajectory and IK solver at runtime.
To keep experiments stable, start with moderate trajectory size and frequency, and tune IK damping only if needed.

| Group | Parameter | What it controls | Typical effect |
| --- | --- | --- | --- |
| Trajectory | `traj_mode` | shape selection (`circle`, `rectangle`, `heart`) | changes path type |
| Trajectory | `*_center` | trajectory center `[x,y,z]` (m) | moves trajectory within workspace |
| Circle | `circle_radius` | circle size (m) | too large may cause IK failure |
| Heart | `heart_scale` | heart size | too large increases curvature load |
| Rectangle | `rect_width`, `rect_height` | rectangle size (m) | larger = harder near corners |
| IK | `ik_damping` | DLS damping | higher = more stable, less accurate |
| IK | `ik_tol_pos_m` | position tolerance (m) | smaller = stricter convergence |
| IK | `ik_position_only` | ignore orientation | improves robustness for drawing |

<details>
<summary><strong>▼ Full parameter list (click to expand)</strong></summary>

| Step | Parameter | Description | Applicable Trajectory |
| --- | --- | --- | --- |
| 1 | `traj_mode` | Selects the task-space trajectory type (`heart`, `circle`, `rectangle`) | All |
| 2 | `*_center` | Center position of the trajectory [x,y,z] in meters | All |
| 3 | `circle_radius` | Radius of the circular trajectory | Circle |
|  | `circle_plane` | Embedding plane of the circle (`xy`, `xz`, `yz`) | Circle |
|  | `circle_hz` | Angular frequency of the circular trajectory | Circle |
|  | `circle_phase_deg` | Phase offset of the circle (degrees) | Circle |
|  | `heart_scale` | Global scale factor of the heart trajectory | Heart |
|  | `heart_plane` | Embedding plane of the heart trajectory | Heart |
|  | `heart_hz` | Angular frequency of the heart trajectory | Heart |
|  | `heart_phase_deg` | Phase offset of the heart (degrees) | Heart |
|  | `rect_width` | Width of the rectangular trajectory (meters) | Rectangle |
|  | `rect_height` | Height of the rectangular trajectory (meters) | Rectangle |
|  | `rect_corner_radius` | Corner rounding radius (meters) | Rectangle |
|  | `rect_plane` | Embedding plane of the rectangle (`xy`, `xz`, `yz`) | Rectangle |
|  | `rect_hz` | Traversal frequency of the rectangle | Rectangle |
|  | `rect_phase_deg` | Phase offset of the rectangle (degrees) | Rectangle |
| 4 | `ik_position_only` | Enables position-only IK (ignores orientation error) | All |
|  | `ik_damping` | Damping factor for Damped Least Squares IK | All |
|  | `ik_step_scale` | Step size scaling for joint updates | All |
|  | `ik_max_iters` | Maximum number of IK iterations | All |
|  | `ik_tol_pos_m` | Position convergence tolerance (meters) | All |
|  | `ik_tol_rot_rad` | Orientation convergence tolerance (radians) | All |
|  | `ik_w_pos` | Weight for position error in IK | All |
|  | `ik_w_rot` | Weight for orientation error in IK | All |

</details>


---
### 5. Conclusion & Future Directions
This Technical Story presented `traj_pub_node`, a ROS 2–based framework for real-time end-effector trajectory control. By directly parsing the URDF to construct the kinematic chain and applying Jacobian-based inverse kinematics with Damped Least Squares, the system achieves stable task-space control without relying on high-level motion planning frameworks.

A timer-driven streaming control architecture enables continuous tracking of parametric trajectories such as circles, rectangles, and heart-shaped paths in simulation. Using the current joint state as the initial guess for inverse kinematics significantly improves convergence and stability during continuous trajectory execution.

Future work will focus on extending this framework to real hardware platforms and incorporating dynamic constraints such as velocity, acceleration, and torque limits. In addition, integrating collision avoidance, workspace constraints, and higher-level planners or learning-based policies will further enhance autonomy and generalization for complex manipulation tasks.

---

### 6. References & Resources
**Related Projects & Tools**
- [OMX](https://github.com/ROBOTIS-GIT/open_manipulator) - OMX robot platform



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