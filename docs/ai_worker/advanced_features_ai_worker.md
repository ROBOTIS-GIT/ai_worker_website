# Advanced Features

## Overview

This section covers advanced control and autonomy features that extend the AI Worker's capabilities beyond basic imitation learning. These features enable more sophisticated task execution, decision-making, and adaptive behaviors in complex real-world scenarios.

## Available Features

### 1. Behavior Trees

Behavior Trees provide a modular, hierarchical framework for organizing robot control logic. They enable:

- **Task Decomposition**: Break complex tasks into manageable subtasks
- **Reactive Control**: Respond dynamically to sensor inputs and state changes
- **Reusability**: Share behaviors across different tasks and scenarios
- **Visual Design with Groot 2**: Design and edit behavior trees graphically using Groot 2, then export to XML format for execution

Behavior Trees are particularly powerful when combined with imitation learning models, where learned policies serve as action nodes within a larger decision-making structure. The AI Worker uses XML-based behavior tree definitions created with [Groot 2](https://www.behaviortree.dev/groot), providing a visual workflow for designing complex robot behaviors.

<a href="/ai_worker/behavior_tree_ai_worker" class="button-shortcut">
Learn More About Behavior Trees
</a>

### 2. Cyclo Control

Cyclo Control is the software layer that turns easier robot commands, such as target hand poses or joint commands, into joint trajectories that the real robot can execute. It is useful when you want higher-level motion commands while still relying on the controller to generate safe robot motion.

- **Higher-Level Commanding**: Command target poses or joint goals instead of manually sending every joint step
- **Interpolated Motion**: Generate smooth `MoveL` and `MoveJ` motions from the current state to the goal over a requested time
- **QP-Based Safety**: Apply constraints such as joint range, joint velocity, and self-collision avoidance while tracking the command

On AI Worker, this is especially helpful when you want motion that is easier to command than raw low-level control, but still filtered through the controller's safety-aware optimization.

<a href="/ai_worker/advanced_motion_controller_ai_worker" class="button-shortcut">
Learn More About Cyclo Motion Controller
</a>

### 3. Cyclo Manager

Cyclo Manager is an integrated control system for managing and controlling the robot through a web-based UI. You can bring up the robot from the UI, and inspect and manage Docker container status from the same interface.

- **Robot bring-up system**: Start and coordinate the robot stack from the UI instead of wiring together many manual steps.
- **Topic viewer**: Inspect ROS 2 topics and message flow from the browser while the system is running.
- **Docker management system**: View container health, control lifecycle, and keep the containerized services aligned with what the robot needs.
- **noVNC**: Open remote desktop sessions in the browser so you can interact with graphical tools on the host or in containers without a separate VNC client.

<a href="/ai_worker/cyclo_manager_ai_worker" class="button-shortcut">
Learn More About Cyclo Manager
</a>

### 4. Cyclo Intelligence

Cyclo Intelligence is the AI workflow system for recording robot data, converting and managing datasets, training policies, and deploying models through the Cyclo Brain inference runtime.

- **Record Pipeline**: Collect robot episodes, preserve rosbag2 data, and convert MCAP/MP4 recordings into LeRobot dataset outputs through `cyclo_data`.
- **Dataset Management**: Merge datasets, delete episodes, convert recordings, and manage Hugging Face upload/download operations from the UI.
- **Model Deployment**: Select LeRobot or GR00T policy backends, manage Docker runtime status, and run inference with task instructions and policy checkpoints.
- **Cyclo Brain Runtime**: Run policy backends with separated `main-runtime` and `engine-process` services for model loading, action chunk generation, and robot command publishing.

<a href="/ai_worker/cyclo_intelligence_ai_worker" class="button-shortcut">
Learn More About Cyclo Intelligence
</a>

### 5. ROBOTIS Vuer

ROBOTIS Vuer lets AI Worker use a **Meta Quest 3** headset together with a browser-based **Vuer** client for immersive robot interaction. The headset connects to the robot-side ROS 2 system through **HTTPS/WSS**, allowing the system to exchange pose, visualization, and control data in real time.

- **Immersive Monitoring**: View the robot scene and related 3D information directly in the headset
- **Hand-Tracking Interaction**: Use VR hand tracking and related input as references for robot control
- **Browser-Based Workflow**: Start the VR session from the headset browser without installing a separate native application
- **ROS 2 Integration**: Connect the Vuer-based VR interface with the robot-side ROS 2 stack for teleoperation and state visualization

In AI Worker, ROBOTIS Vuer is used as the front-end interface for VR teleoperation. Vuer handles the browser-side 3D scene and WebXR interaction, while ROS 2 nodes on the robot side process the incoming references and connect them to the robot control pipeline.


<a href="/ai_worker/vuer_overview_ai_worker" class="button-shortcut">
Learn More About ROBOTIS Vuer
</a>
