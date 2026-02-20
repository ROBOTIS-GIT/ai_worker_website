# Technical Story

Hey there! Welcome to the AI Worker Technical Story section, where we share the AI technologies and techniques that bring our robot to life.

Building a capable robot isn't just about assembling hardware – it's about giving it the "intelligence" to perform real-world tasks effectively. AI Worker leverages cutting-edge AI techniques like Imitation Learning and Reinforcement Learning to learn from demonstrations and improve through experience.

Let's dive into the technical stories behind AI Worker.

## Stories

Click on the card below to explore the AI technologies we've implemented:

<div style="display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 20px; margin: 30px 0;">

<details style="grid-column: 1 / -1;">
<summary style="list-style: none; cursor: pointer;">
  <div style="border: 2px solid #e0e0e0; border-radius: 12px; padding: 20px; transition: all 0.3s; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: white; min-height: 200px; display: inline-flex; flex-direction: column; justify-content: center; align-items: center; text-align: center; width: calc(66.666% - 14px); vertical-align: top;">
    <img src="/technical_story/isaac_gr00t_header_compress.png" alt="Isaac GR00T" style="height: 80px; margin-bottom: 15px; filter: brightness(0) invert(1);">
    <h3 style="margin: 0 0 10px 0; border: none; color: white;">AI Worker x NVIDIA GR00T N1.5</h3>
    <div class="expand-indicator" style="margin-top: 15px; font-size: 12px; opacity: 0.8;">▼ Click to expand</div>
  </div>
</summary>
<div style="padding: 30px; border: 2px solid #667eea; border-radius: 12px; margin-top: 20px; background: #1b1b1f; color: #e0e0e0;">

### ▶️ Full Demo

<div style="position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden; max-width: 100%; margin: 20px 0; border-radius: 8px;">
  <iframe
    style="position: absolute; top: 0; left: 0; width: 100%; height: 100%;"
    src="https://www.youtube.com/embed/h2O57D7KGYg"
    title="AI Worker x NVIDIA Isaac GR00T N1.5 Demo"
    frameborder="0"
    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
    allowfullscreen>
  </iframe>
</div>

---

### Overview

We aimed to develop an autonomous robotic system capable of sorting convenience store items, with a specific focus on coffee bottle classification as our primary task scenario. The robot is presented with mixed items placed in a central area and must sort them into designated boxes positioned on the left and right sides.

To achieve this goal, we leveraged **imitation learning** as our core methodology, utilizing [**NVIDIA Isaac GR00T N1.5**](https://developer.nvidia.com/isaac/gr00t) foundation model. We collected real-world demonstration data directly in actual operational environments, ensuring our model could handle the complexities and variations inherent in real-world scenarios.

<div style="margin: 20px 0; padding: 20px; background: rgba(102, 126, 234, 0.1); border-left: 4px solid #667eea; border-radius: 4px;">

**🎯 Public Demonstrations**

We successfully showcased this system at two major robotics conferences:
- **CoRL 2025 (Conference on Robot Learning)**
- **Humanoid Conference 2025**

</div>

<div style="margin: 20px 0; padding: 20px; background: rgba(102, 126, 234, 0.1); border-left: 4px solid #667eea; border-radius: 4px;">

**📊 Performance Metrics**

Through rigorous testing over 100 trials, our system achieved approximately **85% success rate**, demonstrating robust performance while also revealing areas for future improvement.

</div>

<div style="margin: 20px 0; padding: 20px; background: rgba(102, 126, 234, 0.1); border-left: 4px solid #667eea; border-radius: 4px;">

**💡 Project Impact**

This project goes beyond simply using a foundation model – we've built a complete system infrastructure based on **AI Worker** and **Physical AI Tools** that enables the deployment and operation of GR00T N1.5 on real robotic hardware.

</div>

---

### Model Architecture

NVIDIA Isaac GR00T N1.5 is an open vision-language-action (VLA) foundation model designed for robotic manipulation tasks. The model architecture consists of two main systems working in tandem to translate multimodal inputs into precise robot actions.

<div style="text-align: center; margin: 30px 0;">
  <img src="/technical_story/isaac_gr00t_architecture.png" alt="Isaac GR00T N1.5 Architecture" style="max-width: 100%; border-radius: 8px; border: 2px solid #667eea;">
  <p style="margin-top: 10px; font-size: 14px; color: #888; font-style: italic;">Figure: Isaac GR00T N1.5 Architecture Overview</p>
</div>

#### Input Modalities

The model accepts three types of input data:

- **Image Observation**: Visual input from the robot's cameras
- **Language Instruction**: Natural language commands describing the desired task
- **Robot State**: Current robot configuration including joint positions and end-effector poses

#### System 2: Vision-Language Model (VLM)

The Vision-Language Model processes visual and linguistic information to understand the task context and scene semantics. This high-level understanding guides the action generation process.

#### System 1: Diffusion Transformer

The Diffusion Transformer takes the VLM output along with robot state information and generates action chunks - sequences of future robot actions.

This two-system architecture enables the model to leverage both high-level semantic understanding and low-level motor control for robust robotic manipulation.

---

### Data Collection

We collected demonstration data to train the model for real-world robustness.

- **Dataset Size**: 10 hours of demonstration data across 800 episodes
- **Task Scope**: Sorting four different coffee bottles into their designated bins
- **Collection Strategy**: Intentionally varied lighting conditions and backgrounds during data collection
- **Environment Diversity**: Multiple lighting scenarios and background variations to ensure robust generalization

**Key Insight**: By training with diverse environmental conditions from the start, we built a system capable of handling the intense and unpredictable lighting of exhibition halls without requiring additional adaptation.

---

### Training

The model was fine-tuned using NVIDIA's powerful GPU infrastructure.

- **Hardware Setup**: 8x NVIDIA B200 GPUs
- **Training Duration**: Approximately 20 hours for complete fine-tuning
- **Base Model**: NVIDIA Isaac GR00T N1.5 foundation model
- **Training Approach**: Fine-tuning pre-trained weights on task-specific demonstration data

**Note**: The relatively short training time (20 hours) demonstrates the efficiency of foundation model fine-tuning compared to training from scratch, which would typically require significantly more time and data.

---

### Technical Challenges & Solutions

<details>
<summary style="cursor: pointer; padding: 15px; background: rgba(102, 126, 234, 0.1); border-left: 4px solid #667eea; border-radius: 4px; margin: 10px 0; font-weight: 600;">
💡 Challenge 1: Real-World Robustness
</summary>
<div style="padding: 15px 20px; margin: 0 0 10px 0; background: rgba(102, 126, 234, 0.05); border-radius: 0 0 4px 4px;">

**Problem**: Laboratory-trained models often fail in real-world environments due to lighting variations, background changes, and unpredictable conditions.

**Solution**: We intentionally introduced environmental diversity during data collection by varying lighting conditions and backgrounds. This "train for the real world" approach ensured the model could generalize to challenging scenarios like exhibition halls with intense and unpredictable lighting.

</div>
</details>

<details>
<summary style="cursor: pointer; padding: 15px; background: rgba(102, 126, 234, 0.1); border-left: 4px solid #667eea; border-radius: 4px; margin: 10px 0; font-weight: 600;">
🎪 Challenge 2: Exhibition Hall Deployment
</summary>
<div style="padding: 15px 20px; margin: 0 0 10px 0; background: rgba(102, 126, 234, 0.05); border-radius: 0 0 4px 4px;">

**Problem**: Trade show floors present some of the most challenging conditions for robotic systems - constantly changing lighting, crowds, and visual distractions.

**Solution**: Anticipating exhibition deployment, we intentionally varied backgrounds and environments during data collection. To simulate crowded conditions, we positioned the robot in high-traffic areas during data acquisition. This proactive approach enabled the system to handle the exhibition environment without requiring any additional fine-tuning or adaptation.

</div>
</details>

<details>
<summary style="cursor: pointer; padding: 15px; background: rgba(102, 126, 234, 0.1); border-left: 4px solid #667eea; border-radius: 4px; margin: 10px 0; font-weight: 600;">
📊 Challenge 3: Data Efficiency
</summary>
<div style="padding: 15px 20px; margin: 0 0 10px 0; background: rgba(102, 126, 234, 0.05); border-radius: 0 0 4px 4px;">

**Problem**: Collecting large-scale demonstration data can be time-consuming and expensive.

**Solution**: By leveraging the NVIDIA Isaac GR00T N1.5 foundation model, we achieved strong performance with just 10 hours (800 episodes) of task-specific data, significantly reducing the data requirements compared to training from scratch.

</div>
</details>

---

### Deployment

Our deployment architecture is designed for modularity, scalability, and isolation using containerized environments. The system integrates multiple components across different machines, leveraging ROS2 for inter-process communication and Docker for dependency management.

<div style="text-align: center; margin: 30px 0;">
  <img src="/technical_story/isaac_gr00t_system_architecture.png" alt="Deployment Architecture" style="max-width: 100%; border-radius: 8px; border: 2px solid #667eea;">
  <p style="margin-top: 10px; font-size: 14px; color: #888; font-style: italic;">Figure: System Deployment Architecture</p>
</div>

#### System Architecture Overview

Our deployment consists of two primary machines communicating via ROS2 and ZMQ protocols:

**ORIN (Robot Controller)**
- **AI Worker Container**: Manages robot control and executes motor commands
- **Physical AI Tools Container**: Handles inference orchestration and UI configuration
- **Communication**: Both containers communicate via ROS2 topics

**5090 Workstation (Inference Server)**
- **GR00T N1.5 Container**: Runs the foundation model for action prediction
- **Communication**: Connects to Physical AI Tools via ZMQ socket

#### Component Details

<div style="margin: 20px 0; padding: 20px; background: rgba(102, 126, 234, 0.1); border-left: 4px solid #667eea; border-radius: 4px;">

**🤖 AI Worker (Docker Container)**
- Built on ROS2 framework for robot control
- Subscribes to action chunk topics from Physical AI Tools
- Executes motor commands to control the robot hardware
- Handles low-level robot state management and safety protocols

</div>

<div style="margin: 20px 0; padding: 20px; background: rgba(102, 126, 234, 0.1); border-left: 4px solid #667eea; border-radius: 4px;">

**🛠️ Physical AI Tools (Docker Container)**
- Provides UI for inference configuration and monitoring
- Orchestrates the inference pipeline
- Publishes action chunk ROS2 topics to AI Worker
- Communicates with GR00T N1.5 model via ZMQ socket
- Manages observation data flow and action prediction requests

</div>

<div style="margin: 20px 0; padding: 20px; background: rgba(102, 126, 234, 0.1); border-left: 4px solid #667eea; border-radius: 4px;">

**🧠 GR00T N1.5 (Docker Container)**
- Runs NVIDIA Isaac GR00T N1.5 foundation model
- Deployed using NVIDIA's official Docker image
- Receives observations via ZMQ from Physical AI Tools
- Returns predicted action chunks for robot execution
- Leverages GPU acceleration for real-time inference

</div>

#### Communication Flow

1. **Observation Collection**: AI Worker collects sensor data (camera images, robot state) and publishes to ROS2 topics
2. **Data Forwarding**: Physical AI Tools subscribes to observation topics and forwards data to GR00T N1.5 via ZMQ
3. **Action Prediction**: GR00T N1.5 processes observations and returns action chunks via ZMQ
4. **Action Execution**: Physical AI Tools publishes action chunks to ROS2 topics
5. **Robot Control**: AI Worker subscribes to action topics and executes motor commands

#### Design Benefits

- **Dependency Isolation**: Docker containers prevent dependency conflicts between different components
- **Modularity**: Each component can be developed, tested, and deployed independently
- **Scalability**: Inference server can be scaled separately from robot controller
- **Cross-Machine Communication**: ZMQ socket enables efficient communication between ORIN and workstation, while ROS2 handles communication between containers on ORIN
- **Hardware Flexibility**: High-compute inference tasks run on dedicated GPU workstation while robot control runs on embedded ORIN platform

---

### Experimental Results

We conducted extensive testing to evaluate the system's performance in real-world scenarios, focusing on both quantitative metrics and qualitative failure analysis.

#### System Performance Metrics

<div style="margin: 20px 0; padding: 20px; background: rgba(102, 126, 234, 0.1); border-left: 4px solid #667eea; border-radius: 4px;">

**⚡ Inference Performance**
- **Inference Latency**: 40ms on NVIDIA RTX 5090 GPU
- **Control Frequency**: 10 FPS robot control loop
- **Total System Latency**: End-to-end observation to action execution

</div>

<div style="margin: 20px 0; padding: 20px; background: rgba(102, 126, 234, 0.1); border-left: 4px solid #667eea; border-radius: 4px;">

**🎯 Task Success Rate**
- **Test Trials**: 100 sorting attempts
- **Success Rate**: ~85%
- **Test Environment**: Real exhibition hall conditions with varied lighting and backgrounds

</div>

#### Control Optimization Techniques

To achieve smooth and natural robot motion, we implemented several optimization strategies:

1. **Dynamixel Motor Tuning**: Fine-tuned velocity and acceleration profiles to ensure fluid movements without jerking
2. **Asynchronous Inference**: Implemented non-blocking inference pipeline to maintain consistent control frequency
3. **Action Smoothing**: Applied temporal smoothing to action chunks for coordinated multi-joint movements

#### Failure Cases

Understanding failure cases is critical for future improvements. We identified three primary failure patterns in the 15% unsuccessful trials:

- **Color Misclassification (~50% of failures)**: Correctly grasped but placed in the wrong bin due to color recognition errors
- **Grasping Failures (~30% of failures)**: Robot failed to successfully grasp the target object
- **Phantom Actions (~20% of failures)**: Attempted placement without having grasped an object

---

### Lessons Learned & Future Work

This project provided valuable insights into deploying foundation models on real robotic systems. Here are our key takeaways and directions for future development.

#### What Worked Well ✅

<div style="margin: 20px 0; padding: 20px; background: rgba(102, 126, 234, 0.1); border-left: 4px solid #667eea; border-radius: 4px;">

**Foundation Model Efficiency**
- Fine-tuning approach significantly reduced data requirements.
- 20-hour training time on 8x B200 GPUs demonstrated excellent time-to-deployment.

**Environmental Diversity Strategy**
- Collecting data in varied lighting and crowded areas proved essential for real-world robustness
- "Train for the real world" approach eliminated the need for domain adaptation

**Modular System Architecture**
- Docker containerization prevented dependency conflicts and simplified deployment
- ROS2 + ZMQ communication enabled flexible cross-machine deployment

</div>

#### Challenges & Learning Points 🎓

<div style="margin: 20px 0; padding: 20px; background: rgba(245, 87, 108, 0.1); border-left: 4px solid #f5576c; border-radius: 4px;">

**85% Success Rate**
- While acceptable for demonstrations, production deployment would require higher reliability
- Color misclassification remains the primary failure cause, indicating need for improved visual perception

**Data Collection Overhead**
- 10 hours of teleoperation data collection required significant manual effort
- Need for more efficient data collection methods or synthetic data augmentation

**Grasp Stability**
- Depth perception and grasp planning could benefit from additional sensors or algorithms

</div>

#### Future Directions 🚀

1. **Enhanced Visual Perception**: Integrate better color recognition under varied lighting, possibly with additional sensors
2. **Grasp Detection**: Add proprioceptive feedback to detect successful grasps before attempting placement
3. **More Training Data**: Collect additional edge cases to improve success rate to 90-95%

---

### References & Resources

**Related Projects & Tools**
- [NVIDIA Isaac GR00T](https://github.com/NVIDIA/Isaac-GR00T) - Official NVIDIA Isaac GR00T repository
- [Physical AI Tools](https://github.com/ROBOTIS-GIT/physical_ai_tools) - Our open-source tools for robotic AI deployment
- [AI Worker](https://github.com/ROBOTIS-GIT/ai_worker) - AI Worker robot platform

</div>
</details>

<details style="grid-column: 1 / -1;">
<summary style="list-style: none; cursor: pointer;">
  <div style="border: 2px solid #e0e0e0; border-radius: 12px; padding: 20px; transition: all 0.3s; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: white; min-height:200px; display: inline-flex; flex-direction: column; justify-content: center; align-items: center; text-align: center; width: calc(66.666% - 14px); vertical-align: top;">
    <img src="/technical_story/nav2_header_compressed.png" alt="AI Worker Navigation" style="height: 100px; max-width: 95%; margin-bottom: 6px; object-fit: contain; filter: invert(1) grayscale(1) brightness(2.3); mix-blend-mode: screen; background: transparent;">
    <h3 style="margin: 0 0 6px 0; border: none; color: white;">AI Worker x Nav2</h3>
    <div class="expand-indicator" style="margin-top: 10px; font-size: 12px; opacity: 0.9; color: #f5f7ff;">▼ Click to expand</div>
  </div>
</summary>
<div style="padding: 30px; border: 2px solid #667eea; border-radius: 12px; margin-top: 20px; background: #1b1b1f; color: #e0e0e0;">

### Overview: What You Will Learn

This post is a practical guide for engineers building autonomous navigation on ROS2 Nav2. It starts from the basic setup and walks through designing your own navigation strategies.

**Key Knowledge You Will Gain**
- **Understanding Nav2 Basics:** How the Nav2 stack is structured and what each core server does.
- **Implementing Basic Navigation (RPP):** Set up a stable baseline using the Regulated Pure Pursuit controller.
- **Holonomic Drive Potential:** Why holonomic (crab) drive matters for omnidirectional robots and how it changes maneuverability.
- **Creating Your Own Strategy (BT Customization):** How to extend Nav2’s Behavior Tree to encode custom navigation logic.

---

### System Architecture

- **Architecture Overview**  
  <div style="text-align: center; margin: 20px 0 16px 0;">
    <img src="/technical_story/nav2_architecture.png" alt="Nav2 stack architecture overview" style="max-width: 100%; border-radius: 8px; border: 2px solid #667eea;">
    <p style="margin-top: 8px; font-size: 14px; color: #bfc4e6; font-style: italic;">Nav2 stack with planning, control, smoothing, recovery, and BT-based coordination.</p>
  </div>


**Servers at a glance**  
- **Map Server**: Publishes the static map so planners, costmaps, and localization share a common world frame.  
- **Planner Server**: Computes a collision-aware global path from start to goal using the configured global planner plugin.  
- **Controller Server**: Generates real-time velocity commands by following the active path with the configured local controller plugin.  
- **Smoother Server**: Makes the planned path smoother by reducing sharp corners and wobbly sections, leading to better robot motion. 
- **Behavior Tree Navigator**: Executes the BT that sequences planning, control, and recovery behaviors for each goal.  
- **Recovery Server**: Supplies standard recovery actions (clear costmaps, spin, replan) when progress or localization is lost.  
- **Lifecycle Manager**: Manages node lifecycle states (configure/activate/deactivate/cleanup) and can restart components if they fail.

---

### Implementing Basic Navigation with RPP

To provide fundamental and stable driving capabilities, we start with the Regulated Pure Pursuit (RPP) controller as the default.

::: info
**What is RPP?**  
- RPP is a widely used, reliable geometric path-tracking algorithm that gives a stable baseline without heavy optimization.  
- **Path Tracking:** Looks ahead to a point on the path and computes curvature to steer smoothly (lookahead driving).  
- **Regulation:** Automatically reduces linear velocity on sharp curves or near obstacles for stable turning.  
- **Kinematic Constraint:** Assumes Differential Drive; outputs only forward (`v_x`) and yaw (`w_z`), ignoring lateral (`v_y`).  
:::

**RPP Performance**  
You can see the robot smoothly following curved paths and moving stably toward the goal.

<div style="display: grid; grid-template-columns: repeat(auto-fit, minmax(240px, 1fr)); gap: 16px; margin: 12px 0 6px 0;">
  <img src="/simulation/ai_worker/navigation_curve.gif" alt="Navigation curve execution" style="width: 100%; border-radius: 8px; border: 2px solid #667eea; background: #0f0f13;">
  <img src="/simulation/ai_worker/aiw_rviz2.gif" alt="RViz navigation visualization" style="width: 100%; border-radius: 8px; border: 2px solid #667eea; background: #0f0f13;">
</div>

---

<details style="margin: 12px 0 6px 0;">
<summary style="cursor: pointer; font-weight: 700;">RPP parameters example — click to expand</summary>

```yaml
# Example controller_server.yaml (RPP)
controller_server:
  ros__parameters:
    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      desired_linear_vel: 0.5
      lookahead_dist: 0.4
      min_lookahead_dist: 0.4
      max_lookahead_dist: 0.6
      lookahead_time: 2.0
      rotate_to_heading_angular_vel: 2.0
      use_velocity_scaled_lookahead_dist: false
      min_approach_linear_velocity: 0.1
      approach_velocity_scaling_dist: 0.4
      use_collision_detection: true
      max_allowed_time_to_collision_up_to_carrot: 1.0
      use_regulated_linear_velocity_scaling: true
      use_fixed_curvature_lookahead: false
      curvature_lookahead_dist: 0.25
      use_cost_regulated_linear_velocity_scaling: false
      cost_scaling_dist: 0.3
      cost_scaling_gain: 1.0
      regulated_linear_scaling_min_radius: 0.9
      regulated_linear_scaling_min_speed: 0.25
      use_rotate_to_heading: true
      allow_reversing: false
      rotate_to_heading_min_angle: 0.785
      max_angular_accel: 2.0
      min_distance_to_obstacle: 0.0
      stateful: true
```
</details>

**Key Parameters Explained**  
While RPP exposes many parameters, the core driving behavior is shaped by a few fundamentals:

1) **Basic Driving & Lookahead**  
   - `desired_linear_vel` (0.5 m/s): Target max linear speed on straights.  
   - `lookahead_dist` (0.4 m): How far ahead the controller aims. Larger = smoother but wider turns; smaller = tighter tracking but possible oscillation. (With `use_velocity_scaled_lookahead_dist=false`, this stays fixed.)  

2) **Speed Regulation**  
   - `use_regulated_linear_velocity_scaling` (true): Enables automatic slowing on sharp curves.  
   - `regulated_linear_scaling_min_radius` (0.9 m): Radius threshold to start decelerating on tight turns.  
   - `regulated_linear_scaling_min_speed` (0.25 m/s): Floor speed during regulation to avoid stalling.  

3) **Rotation-to-Heading & Safety**  
   - `use_rotate_to_heading` (true): If starting heading deviates from the path, rotate in place before moving.  
   - `rotate_to_heading_angular_vel` (2.0 rad/s): Spin speed for that alignment.  
   - `use_collision_detection` (true): Basic stop-if-obstacle detection using the costmap.  

---

### Holonomic Drive Capabilities

RPP is a great default for stability, but if your robot has mecanum or swerve drive, you can achieve significantly better efficiency by using true holonomic motion. For that, we chose MPPI as the controller.

**Holonomic Advantage**  
- Heading and motion can be decoupled; the robot can slide sideways or move diagonally without first turning.  
- Agility in confined spaces: slide laterally through tight aisles instead of multi-point turns.  

<div style="display: grid; grid-template-columns: repeat(auto-fit, minmax(260px, 1fr)); gap: 14px; margin: 12px 0 10px 0; align-items: start;">
  <div>
    <img src="/technical_story/RPP%20running.gif" alt="RPP controller run" style="width: 100%; border-radius: 10px; border: 2px solid #667eea; background: #0f0f13;">
    <p style="margin: 6px 0 0; font-size: 14px; color: #c9ceda;">RPP (Ackermann style) must yaw first before edging sideways, so tight approaches add extra arc distance.</p>
  </div>
  <div>
    <img src="/technical_story/MPPI%20running.gif" alt="MPPI holonomic run" style="width: 100%; border-radius: 10px; border: 2px solid #667eea; background: #0f0f13;">
    <p style="margin: 6px 0 0; font-size: 14px; color: #c9ceda;">MPPI Holonomic keeps heading independent from translation, using crab slides for short hops to the goal with fewer corrections.</p>
  </div>
</div>

Holonomic control shines at short range: crab motion trims turning time and keeps the robot efficient in narrow spots

**Why MPPI for Holonomic?**  
Nav2 offers multiple holonomic-capable controllers (e.g., DWB, TEB), but MPPI is highly flexible: by mixing critics(scoring functions that evaluate paths) and motion models you can change driving style without touching the core algorithm.

::: info
**What is MPPI? (Model Predictive Path Integral)**  
- **Simulate:** From the current state and motion model (e.g., Omni), MPPI samples thousands of future trajectories.  
- **Score:** Critic functions evaluate goal progress, obstacle clearance, smoothness, etc.  
- **Execute:** The weighted average of top samples becomes the next velocity command.  
This cycle repeats rapidly, producing smooth, adaptive control.
:::

---

<details style="margin: 10px 0 6px 0;">
<summary style="cursor: pointer; font-weight: 700;">MPPI parameter example — click to expand</summary>

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 30.0
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      time_steps: 56
      model_dt: 0.05
      batch_size: 2000
      vx_std: 0.2
      vy_std: 0.2
      wz_std: 0.4
      vx_max: 0.5
      vx_min: -0.35
      vy_max: 0.5
      wz_max: 1.9
      ax_max: 3.0
      ax_min: -3.0
      ay_min: -3.0
      ay_max: 3.0
      az_max: 3.5
      iteration_count: 1
      temperature: 0.3
      gamma: 0.015
      motion_model: "Omni"
      visualize: false
      reset_period: 1.0 # (only in Humble)
      regenerate_noises: false
      TrajectoryVisualizer:
        trajectory_step: 5
        time_step: 3
      TrajectoryValidator:
        plugin: "mppi::DefaultOptimalTrajectoryValidator"
        collision_lookahead_time: 2.0
        consider_footprint: false
      AckermannConstraints:
        min_turning_r: 0.2
      critics: ["ConstraintCritic", "CostCritic", "GoalCritic", "GoalAngleCritic", "PathAlignCritic", "PathFollowCritic", "PathAngleCritic", "PreferForwardCritic"]
      ConstraintCritic:
        enabled: true
        cost_power: 1
        cost_weight: 4.0
      GoalCritic:
        enabled: true
        cost_power: 1
        cost_weight: 5.0
        threshold_to_consider: 1.4
      GoalAngleCritic:
        enabled: true
        cost_power: 1
        cost_weight: 3.0
        threshold_to_consider: 0.5
      PreferForwardCritic:
        enabled: true
        cost_power: 1
        cost_weight: 5.0
        threshold_to_consider: 0.5
      CostCritic:
        enabled: true
        cost_power: 1
        cost_weight: 3.81
        critical_cost: 300.0
        consider_footprint: true
        collision_cost: 1000000.0
        near_goal_distance: 1.0
        trajectory_point_step: 2
      PathAlignCritic:
        enabled: true
        cost_power: 1
        cost_weight: 14.0
        max_path_occupancy_ratio: 0.05
        trajectory_point_step: 4
        threshold_to_consider: 0.5
        offset_from_furthest: 20
        use_path_orientations: false
      PathFollowCritic:
        enabled: true
        cost_power: 1
        cost_weight: 5.0
        offset_from_furthest: 5
        threshold_to_consider: 1.4
      PathAngleCritic:
        enabled: true
        cost_power: 1
        cost_weight: 2.0
        offset_from_furthest: 4
        threshold_to_consider: 0.5
        max_angle_to_furthest: 1.0
        mode: 0
```
</details>

**Key Parameters Explained**  
- `motion_model: "Omni"`: tells MPPI to simulate omnidirectional moves.  
- `vy_max / vy_min`: must be non-zero to actually command lateral velocity.  
- Critic tuning for holonomic use:  
  - `PreferForwardCritic`: disable or set very low weight to allow sideways motion.  
  - `PathAlignCritic`: reduce/disable to decouple heading from path direction.  
  - `GoalAngleCritic`: keep moderate to align final heading without over-constraining mid-path strafing.  
  - `CostCritic`: ensure it stays enabled to respect obstacles while allowing lateral detours.  
  - `PathFollowCritic`: keep enabled to stay near the path but balance with relaxed alignment for crab motion.

---

### Crafting Your Own Strategy with Behavior Trees

Nav2's Behavior Tree (BT) is the core logic layer that dictates the robot's decision-making process. By customizing this BT, you can move beyond standard behaviors and define your own unique navigation strategies tailored to your specific mission.
I’ll show how i modified the BT to build my path-length–aware driving strategy so you can use it as a reference.

**Hybrid Static Strategy (Path-Length Based)**  
Pick the driving mode before motion starts, based on the total length of the planned global path:
- **Long path (Cruising):** If the goal is far, favor stable, car-like motion for the whole trip.  
- **Short path (Precision):** If the goal is near, favor agile holonomic motion (sideways/diagonal moves).

::: tip
Standard Nav2 lacks a condition node to classify path length, so we add one via a custom C++ BT node. If the capability you need already exists as a Nav2 BT plugin, use it; if not, create a custom node as shown here.
:::

**Implementing a Path-Length Switch in the BT**  

1. **The Logic (Condition Node)**  
   Class inherits from BT::ConditionNode and checks if the plan length is under a threshold.

```cpp
// is_path_length_under.cpp (simplified)
BT::NodeStatus IsPathLengthUnder::tick() {
  nav_msgs::msg::Path path;
  if (!getInput("path", path)) return BT::NodeStatus::FAILURE; // read from BT blackboard

  double total_length = 0.0;
  for (size_t i = 0; i + 1 < path.poses.size(); ++i)
    total_length += distance(path.poses[i], path.poses[i+1]);

  double threshold;
  getInput("distance_threshold", threshold);

  return (total_length <= threshold) ? BT::NodeStatus::SUCCESS
                                     : BT::NodeStatus::FAILURE;
}
```

2. **Plugin Registration**  
   Add the registration macro in the same C++ file so BT Navigator can load the node from the built shared library.

```cpp
// Plugin registration
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<nav2_behavior_tree::IsPathLengthUnder>(
    "IsPathLengthUnder");
}
```

3. **Behavior Tree Wiring (XML)**  
   Use a `ReactiveFallback` to choose the controller based on path length.

```xml
<ReactiveFallback name="SwerveModeSelector">
  <Sequence name="ShortDistanceLogic">
    <IsPathLengthUnder distance_threshold="2.0" path="{path}"/>
    <FollowPath path="{path}" controller_id="OmniController"/>
  </Sequence>
  <Sequence name="LongDistanceLogic">
    <FollowPath path="{path}" controller_id="CurveController"/>
  </Sequence>
</ReactiveFallback>
```

**How the BT picks a controller (runtime flow)**  
1. Planner writes the global path to the BT blackboard.  
::: info
**What is BT Blackboard?**  
The blackboard is shared memory inside the BT where nodes read and write shared data (e.g., the global path). It avoids ROS topic overhead between BT nodes during decision-making.
:::
2. BT Navigator ticks `IsPathLengthUnder`; SUCCESS if path ≤ threshold, else FAILURE.  
3. The reactive fallback selects the matching branch and passes the chosen `controller_id` (`OmniController` or `CurveController`).  
4. BT Navigator sends the goal with that `controller_id` to the Controller Server, which runs the corresponding MPPI instance.



**Visual representation of the Hybrid Strategy BT logic flow**
<div style="text-align: center; margin: 14px 0 10px 0;">
  <img src="/simulation/ai_worker/flow2.svg" alt="BT decision flow for controller switching" style="max-width: 100%; border-radius: 8px; border: 2px solid #667eea; background: #0f0f13;">
</div>



**Dual-Controller Configuration**  
<div style="display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 20px; margin: 20px 0;">
  <div style="padding: 20px; background: rgba(102, 126, 234, 0.05); border-left: 4px solid #667eea; border-radius: 4px;">
    <strong>1. OmniController (Short Path / Precision)</strong><br>
    Goal: Holonomic agility for tight spaces and fine positioning.<br>
    Key Critics:<br>
    <ul style="margin: 8px 0 0 18px; padding: 0;">
      <li>PreferForwardCritic: disabled (sideways/backward allowed).</li>
      <li>GoalAngleCritic (w=5.0): align final heading for interaction tasks.</li>
      <li>PathFollowCritic (w=5.0): stay near the path without forcing heading alignment.</li>
    </ul>
  </div>
  <div style="padding: 20px; background: rgba(102, 126, 234, 0.05); border-left: 4px solid #667eea; border-radius: 4px;">
    <strong>2. CurveController (Long Path / Cruising)</strong><br>
    Goal: Stability and efficiency over distance; minimize lateral wobble.<br>
    Key Critics:<br>
    <ul style="margin: 8px 0 0 18px; padding: 0;">
      <li>PathAlignCritic (w=14.0): penalizes heading vs. path direction error.</li>
      <li>PreferForwardCritic (w=9.0): favors forward velocity over sliding.</li>
      <li>PathAngleCritic (w=6.0): smooths sharp curvature ahead.</li>
    </ul>
  </div>
</div>

::: info
Want to try this MPPI example? In `navigation.launch.py`, set `params_file` to `navigation_mppi.yaml` instead of `navigation.yaml`.
:::

</div>
</details>
</div>


