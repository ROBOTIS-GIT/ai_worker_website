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

### 2. Cyclo Motion Controller

The Cyclo Motion Controller is the software layer that turns easier robot commands, such as target hand poses or joint commands, into joint trajectories that the real robot can execute. It is useful when you want higher-level motion commands while still relying on the controller to generate safe robot motion.

- **Higher-Level Commanding**: Command target poses or joint goals instead of manually sending every joint step
- **Interpolated Motion**: Generate smooth `MoveL` and `MoveJ` motions from the current state to the goal over a requested time
- **QP-Based Safety**: Apply constraints such as joint range, joint velocity, and self-collision avoidance while tracking the command

On AI Worker, this is especially helpful when you want motion that is easier to command than raw low-level control, but still filtered through the controller's safety-aware optimization.

<a href="/ai_worker/advanced_motion_controller_ai_worker" class="button-shortcut">
Learn More About Cyclo Motion Controller
</a>