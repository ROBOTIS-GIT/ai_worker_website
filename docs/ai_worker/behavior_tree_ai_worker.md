# Behavior Trees

## Overview

::: info
The current Behavior Tree workflow is supported for the `ffw_sg2_rev1` robot type only.
:::

Behavior Trees (BTs) organize robot tasks as modular graphs. On AI Worker, a
tree can combine rule-based robot motions, wait steps, mobile-base rotation,
and learned policy inference into one repeatable task sequence.

BT Manager replaces the older workflow where a user had to create XML in an
external editor (Groot2) and launch a separate behavior-tree package manually. The UI
builds the graph, serializes it to XML, sends it to the runtime, and shows
runtime status while the tree is executing.

The runtime has four main parts.

| Part | Role |
| --- | --- |
| BT Manager UI | Browser-based graph editor, XML loader/saver, and Start/Stop controls. |
| Behavior Tree Node | ROS 2 behavior-tree runner that loads XML, ticks nodes, and publishes BT status. |
| Cyclo Brain | Policy backend runtime used by `SendCommand` nodes for LeRobot or GR00T inference. |
| Robot ROS topics | Joint, lift, head, mobile-base, odometry, and state topics used by action nodes. |

::: tip
Groot 2 is still useful for learning Behavior Tree concepts and inspecting XML,
but it is not required for the normal Cyclo Intelligence BT Manager workflow.
:::

## Before You Begin

### 1. Start Cyclo Intelligence

Open a terminal and enter the **Cyclo Intelligence** Docker container:

`ROBOT PC`

```bash
cd ~/cyclo_intelligence && ./docker/container.sh enter
```

Launch Cyclo Intelligence with the following command:

`ROBOT PC` `🐋 CYCLO INTELLIGENCE`

```bash
ros2 launch orchestrator cyclo_intelligence_bringup.launch.py
```

Or, use shortcut command:

```bash
cyclo_intelligece
```

Open the web UI in a browser.

`http://ffw-{serial number}.local`

(Replacing `{serial number}` with the serial number)

::: warning
BT actions can publish real robot commands. Check the robot workspace, starting
pose, and policy checkpoint before pressing `Start`.
:::

### 2. Start Behavior Tree Node

<div class="UI" style="width: 80%; margin: 0 auto;">
  <img src="/advanced_features/behavior_tree/bt_start_button.png" alt="Web UI" style="width: 100%; ">
</div>

The BT Manager `BT Node ON` button starts the container's `bt_node` service. In
the current implementation, that service runs `ros2 launch orchestrator
bt_node.launch.py` with the launch file default robot type, `ffw_sg2_rev1`.

## BT Manager UI
<img src="/advanced_features/behavior_tree/bt_manager_page.png" alt="Web UI" style="width: 100%; ">
<p style="text-align: center;"><em>Initial Screen</em></p>
Open the `BT Manager` page from the Cyclo Intelligence sidebar.

The page is organized around the main editor areas and runtime controls.

| Area | What It Does |
| --- | --- |
| Left palette | Shows available node types. |
| Update Node List button | Refreshes the palette from the running BT node by calling `/bt/nodes/catalog`. Use this when you add, remove, or edit custom node files. |
| Canvas | Drag nodes, connect edges, move nodes, auto-layout the graph, and collapse or expand control-node subtrees. |
| Right parameter panel | Edit the selected node name and node parameters. |
| Header controls | Load XML, Save As, Undo, Redo, and Auto Layout. |
| Bottom runtime bar | Start/stop the BT node process, start/stop tree execution, and view runtime status. |

### Palette

The palette is split into `Controls` and `Actions`.

- Control nodes own child nodes and decide which child to tick next.
- Action nodes perform work and return `RUNNING`, `SUCCESS`, or `FAILURE`.
- The built-in node list is available without refreshing during normal use.
- Click `Update Node List` only after adding, removing, or editing custom Python node files.
- If the BT node is off, the UI can still show the built-in fallback catalog, but custom nodes will not appear until the live catalog is refreshed.

### Canvas

Use the canvas to create the graph.

1. Drag a node from the palette onto the canvas.
2. Connect a parent node's bottom handle to a child node's top handle.
3. For a `Sequence` or `Loop`, children run in left-to-right order.
4. Move siblings horizontally to adjust execution order, then use Auto Layout if needed.
5. Use the `+` or `-` button on a control node to collapse or expand its subtree.

### Parameter Panel

Click a node on the canvas to open the parameter panel on the right side of the
page. Use this panel to rename the node and edit the parameters that will be
written into the BT XML.

1. Click a node on the canvas.
2. Check the node type shown at the top of the panel.
3. Edit the node name field if you want a clearer label in the graph.
4. Press `Enter` or click outside the name field to apply the new name.
5. Edit the parameter fields for the selected node.
6. Click outside a text or number field to apply the changed value.
7. Click the close button in the panel header to hide the panel.

The node name becomes the XML `name` attribute. This is separate from the node
type, which comes from the Python class name and becomes the XML tag.

Constructor parameters from the Python node become editable fields. The UI uses
different controls depending on the parameter.

| Parameter Style | UI Control | Example |
| --- | --- | --- |
| Boolean options | Checkbox | `enable_head`, `enable_arms`, `enable_lift` |
| Numeric values | Number input | `duration`, `angle_deg`, `max_iterations` |
| Text or lists | Text area | `head_positions`, `left_positions`, `task_instruction` |
| Fixed choices | Dropdown | `SendCommand.command`, `SendCommand.model` |
| Policy path | Text area plus folder button | `policy_path` |

For `JointControl`, position fields are enabled only when the matching group is
enabled. For example, `head_positions` is editable only when `enable_head` is
checked.

For `SendCommand`, the panel enables only the fields that are meaningful for the
selected command. For example, `LOAD` uses model, policy path, instruction, and
rate settings, while `STOP` only needs the command field.

### Runtime Bar

The bottom runtime bar has two separate control groups.

| Control | Meaning |
| --- | --- |
| `ON` | Starts the `bt_node` s6 service. This starts the ROS 2 BT runtime process. |
| `OFF` | Stops the `bt_node` service. It is enabled only when the runtime is stopped. |
| `Start` | Serializes the current graph and calls `/bt/load_and_run`. |
| `Stop` | Calls `/bt/set_running` with `false`, resets the tree, and triggers inference cleanup. |

## Basic Workflow

### 1. Open BT Manager

Open Cyclo Intelligence and select `BT Manager` from the sidebar on the left.

### 2. Start The BT Runtime

Click `BT Node ON`. Wait until the BT node status changes to `Running`.

### 3. Load Or Create A Tree

Use one of these options:

- Click `Load XML` and select an existing file.
- Drag nodes from the palette onto the canvas and connect them to build a new tree manually.

### 4. Add Nodes And Connect Them

Drag nodes onto the canvas and connect them under the root control node.

Example order:

1. `SendCommand` with `command="LOAD"` to load a policy.
2. `JointControl` to move the robot to an initial pose.
3. `SendCommand` with `command="RESUME"` to run inference.
4. `Wait` to let inference run for a fixed time.
5. `SendCommand` with `command="STOP"` to pause inference.
6. `SendCommand` with `command="CLEAR"` to unload the policy.

### 5. Edit Parameters

Click each node and edit its parameters in the right panel.

### 6. Save The Tree

Click `Save As`. BT Manager saves XML files into the behavior-tree source tree through the
Cyclo Data file server. If the filename already exists, the UI asks before
overwriting.

### 7. Run The Tree

Click `Start`. The BT runtime loads that XML string and starts ticking the root node.

While the tree runs:

- The active leaf node is highlighted in the canvas.
- Active state bubbles up to parent control nodes.

### 8. Stop Or Reset After Completion

When a tree completes or fails, press `Stop` before turning the BT node off.

`Stop` resets the runtime state to `stopped` and sends inference cleanup so a
running policy does not keep publishing robot commands.

## Built-In Nodes

The live catalog is the source of truth for the available node list. The following nodes are built into the current BT runtime.

### Control Nodes

#### `Sequence`

`Sequence` ticks children from left to right.

| Child Result | Sequence Behavior |
| --- | --- |
| `RUNNING` | Keeps returning `RUNNING`; the same child remains active. |
| `FAILURE` | Resets that child and returns `FAILURE`. |
| `SUCCESS` | Resets that child and advances to the next child. |

The `Sequence` returns `SUCCESS` after every child has returned `SUCCESS`.

#### `Loop`

`Loop` repeats its children in sequence-like order.

| Parameter | Meaning |
| --- | --- |
| `max_iterations` | `0` means loop forever. A positive number stops after that many full child-list passes. |

Within each iteration, `Loop` behaves like `Sequence`: it stops on the first child failure and advances after child success.

### Action Nodes

#### `Wait`

`Wait` blocks the surrounding control node for a fixed amount of time.

| Parameter | Meaning |
| --- | --- |
| `duration` | Wait time in seconds. |

Use `Wait` between motion commands, or to let a policy run for a fixed window.

#### `Rotate`

`Rotate` turns the mobile base by a target angle.

| Parameter | Meaning |
| --- | --- |
| `angle_deg` | Target rotation in degrees. Positive and negative values rotate in opposite directions. |

`Rotate` reads `/odom` to estimate yaw and publishes a `geometry_msgs/msg/Twist`
command to the mobile-base command topic defined by the active robot config.

::: info
Only robot configs with a mobile action group can use `Rotate` directly.
:::

#### `JointControl`

`JointControl` can command head, arms, and lift from a single action node.

| Parameter | Meaning |
| --- | --- |
| `enable_head` | Enables head trajectory publishing. |
| `head_positions` | Target head joint positions. |
| `enable_arms` | Enables left and right arm trajectory publishing. |
| `left_positions` | Target left arm and gripper positions. |
| `right_positions` | Target right arm and gripper positions. |
| `enable_lift` | Enables lift trajectory publishing. |
| `lift_position` | Target lift position. |
| `duration` | Trajectory duration in seconds. |

:::info
At least one group must be enabled. If all groups are disabled, `JointControl` construction fails and the tree cannot load.
:::

#### `SendCommand`

`SendCommand` controls the Cyclo Brain inference lifecycle from a behavior tree. It calls the same orchestrator command service used by the Inference UI.

| Parameter | Meaning |
| --- | --- |
| `command` | One of `LOAD`, `RESUME`, `STOP`, `CLEAR`. |
| `model` | Policy model/backend choice, such as `lerobot:act` or `groot:n17`. |
| `policy_path` | Policy checkpoint path. Use the container path under `/workspace/model/...`. |
| `task_instruction` | Language instruction for language-conditioned policies. |
| `inference_hz` | Model inference frequency. |
| `control_hz` | Robot command publishing frequency. |
| `chunk_align_window_s` | Action chunk alignment window. |

The parameter panel disables fields that are not meaningful for the selected
command. For example, `STOP` only needs the `command` field.

::: info
Cyclo Intelligence mounts the host workspace into the container as
`/workspace`. When setting `policy_path`, enter the container path instead of a
host shell path such as `~/cyclo_intelligence/...`.

Use one of these default model roots:

```text
/workspace/model/groot/your_model_path
/workspace/model/lerobot/your_model_path
```
:::

::: info
If the next tree uses a different model from the previous tree, add a
`SendCommand` node with `command="CLEAR"` at the end of the previous tree.
Without `CLEAR`, the previously loaded model can remain in memory and the next
model load may fail due to insufficient memory. This may not be noticeable with
lightweight models such as ACT, but it can be a problem with heavier VLA models
such as GR00T.
:::

## XML Files And Storage

BT Manager stores behavior-tree XML files under:

```text
~/cyclo_intelligence/orchestrator/orchestrator/bt/trees/
```

The example file included with the runtime is:

```text
~/cyclo_intelligence/orchestrator/orchestrator/bt/trees/ffw_sg2_rev1_example.xml
```

### XML Runtime Rules

The current XML format uses `BTCPP_format="4"` and a main tree named
`MainTree`.

```xml
<?xml version="1.0" encoding="UTF-8"?>
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="DemoSequence">
      <Wait name="ShortWait" duration="1.0"/>
    </Sequence>
  </BehaviorTree>
</root>
```

The runtime loader:

- Reads `main_tree_to_execute`.
- Finds the matching `<BehaviorTree ID="...">`.
- Loads the first child of that behavior tree as the root node.
- Converts XML attributes into Python constructor parameters.
- Ignores UI-only attributes such as `bt_collapsed`, `bt_x`, and `bt_y`.

## Creating Custom BT Nodes

Custom BT nodes are Python classes that BT Manager can discover and show in the
left palette. In most cases, start with a custom **Action** node. Create a
custom **Control** node only when you need new child-ticking behavior.

The basic idea is:

1. Create a Python file in the correct BT node folder.
2. Define one class in that file.
3. Make the class inherit from `BaseAction` or `BaseControl`.
4. Add constructor arguments for values you want to edit in BT Manager.
5. Implement `tick()`.
6. Turn on the BT node and click `Update Node List`.
7. Drag the new node from the palette into your tree.

::: tip
You do not need to edit XML by hand for a simple custom node. BT Manager reads
the Python class and creates the palette item and parameter fields for you.
:::

### Where To Put Custom Nodes

Use these paths inside the Cyclo Intelligence repository:

| If You Want To Create | Put The Python File Here | Base Class |
| --- | --- | --- |
| A leaf node that performs work | `orchestrator/orchestrator/bt/actions/` | `BaseAction` |
| A parent node that ticks children | `orchestrator/orchestrator/bt/controls/` | `BaseControl` |

Copy a template if you want a starting point:

```text
orchestrator/orchestrator/bt/templates/action_template.py
orchestrator/orchestrator/bt/templates/control_template.py
```

### Example: Create A Simple Action Node

Create a file:

```text
orchestrator/orchestrator/bt/actions/my_action.py
```

Add this code:

```python
import time
from typing import TYPE_CHECKING

from orchestrator.bt.actions.base_action import BaseAction
from orchestrator.bt.bt_core import NodeStatus

if TYPE_CHECKING:
    from rclpy.node import Node


class MyAction(BaseAction):
    def __init__(self, node: 'Node', duration: float = 1.0):
        super().__init__(node, name='MyAction')
        self.duration = float(duration)
        self._start_time = None

    def tick(self) -> NodeStatus:
        if self._start_time is None:
            self._start_time = time.monotonic()
            self.log_info(f'Started for {self.duration}s')
            return NodeStatus.RUNNING

        if time.monotonic() - self._start_time >= self.duration:
            self.log_info('Finished')
            return NodeStatus.SUCCESS

        return NodeStatus.RUNNING

    def reset(self):
        super().reset()
        self._start_time = None
```

What this does:

| Code Part | Meaning In BT Manager |
| --- | --- |
| `class MyAction(BaseAction)` | The node appears in the palette as `MyAction`. |
| `duration: float = 1.0` | BT Manager creates an editable `duration` parameter. |
| `tick()` returns `RUNNING` | The tree keeps this node active. |
| `tick()` returns `SUCCESS` | The parent control node can move to the next child. |
| `reset()` | Clears local state when the tree stops or the node is reused. |

### Show The New Node In BT Manager

After creating or editing a custom node file:

1. Start or restart Cyclo Intelligence if needed.
2. Open `BT Manager`.
3. Click `BT Node ON`.
4. Click `Update Node List`.
5. Look for the class name in the left palette.
6. Drag the node onto the canvas.
7. Click the node and edit its parameters in the right panel.
8. Connect it under a `Sequence` or another control node.
9. Run a small test tree before using it in a real robot task.

::: info
`Update Node List` is mainly for custom node development. During normal BT use
with the built-in nodes, you do not need to refresh the catalog every time.
:::

### When To Use `from_xml_params()`

Most custom nodes do not need `from_xml_params()`.

Use it only when your node needs runtime information that should not be typed
manually in the UI, such as:

- The ROS node handle.
- Robot topic configuration.
- Robot joint-name groups.
- Helper methods for looking up joint names.

The loader passes those values through a `context` object.

```python
@classmethod
def from_xml_params(cls, context, name: str, params: dict):
    action = cls(
        node=context.node,
        duration=params.get('duration', 1.0),
    )
    action.name = name
    return action
```

Built-in nodes that use this advanced pattern include:

- `JointControl`
- `Rotate`
- `SendCommand`

### Custom Control Example

Create a custom control node only when you need behavior that existing controls
cannot express. A control node must tick its children and return one of the BT
statuses.

Create a file:

```text
orchestrator/orchestrator/bt/controls/my_control.py
```

For custom controls:

- Subclass `BaseControl`.
- Use `self.children` to access child nodes.
- Call `child.tick()` to run a child.
- Return `NodeStatus.RUNNING`, `NodeStatus.SUCCESS`, or `NodeStatus.FAILURE`.
- Implement `reset()` to clear child indexes or loop counters.
- Implement `get_active_node_ids()` if you want active-node highlighting in the UI.

## Optional: Behavior Tree Background

If you are new to BTs, review the basic concepts before building a large tree.

The BehaviorTree.CPP introduction is a useful reference for the general model:

```text
https://www.behaviortree.dev/docs/intro
```

Groot 2 is also useful for learning visual BT editing:

```text
https://www.behaviortree.dev/groot
```

For normal AI Worker operation, use Cyclo Intelligence `BT Manager` as the
primary editor and runner.
