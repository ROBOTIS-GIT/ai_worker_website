# Advanced Features

## Overview

This section covers advanced control and autonomy features that extend the OMX's capabilities beyond basic imitation learning. These features enable more sophisticated task execution, decision-making, and adaptive behaviors in complex real-world scenarios.

## Available Features

### 1. Cyclo Motion Controller

The Cyclo Motion Controller is the software layer that turns easier robot commands, such as target hand poses or joint commands, into joint trajectories that the real robot can execute. It is useful when you want higher-level motion commands while still relying on the controller to generate safe robot motion.

- **Higher-Level Commanding**: Command target poses or joint goals instead of manually sending every joint step
- **Interpolated Motion**: Generate smooth `MoveL` and `MoveJ` motions from the current state to the goal over a requested time
- **QP-Based Safety**: Apply constraints such as joint range, joint velocity, and self-collision avoidance while tracking the command

On OMX, this is especially helpful when you want motion that is easier to command than raw low-level control, but still filtered through the controller's safety-aware optimization.

<a href="/omx/advanced_motion_controller_omx" class="button-shortcut">
Learn More About Cyclo Motion Controller
</a>