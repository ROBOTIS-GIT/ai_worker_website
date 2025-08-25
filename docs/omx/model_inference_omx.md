# Model Inference with Web UI

Once your model is trained, you can deploy it on the OMX for inference.

## Model Deployment and Inference

### 1. Set up Physical AI Tools Docker Container
::: warning
If the Physical AI Tools is already set up, you can skip this step.
:::

If you haven't set up the Physical AI Tools Docker container, please refer to the link below for setup instructions.

[Set up Physical AI Tools Docker Container](/omx/dataset_preparation_prerequisites_omx#setup-physical-ai-tools-docker-container)

### 2. Prepare Your Model

`USER PC`

Please place your trained model in the following directory:

`<your_workspace>/physical_ai_tools/lerobot/outputs/train/`

Models trained using Physical AI Tools are automatically saved to that path. However, if you downloaded the model from a hub or copied it from another PC, you need to move the model to that location. 


::: details Available Folder Structures
Please refer to the folder structure tree below:

The following folder structures are all valid (example_model_folder_1, 2, 3).

```
<your_workspace>/physical_ai_tools/lerobot/outputs/train/
├── example_model_folder_1
│   └── checkpoints/
│       ├── 000250/
│       │   ├── pretrained_model/
│       │   │   ├── config.json
│       │   │   ├── model.safetensors
│       │   │   └── train_config.json
│       │   └── training_state
│       │       ├── optimizer_param_groups.json
│       │       ├── optimizer_state.safetensors
│       │       ├── rng_state.safetensors
│       │       └── training_step.json
│       └── 000500/
│           ├── pretrained_model/
|           │   └─ (...)
│           └── training_state/
|               └─ (...)
├── example_model_folder_2/
│        ├── pretrained_model/
|        |   └─ (...)
│        └── training_state
|            └─ (...)
└── example_model_folder_3/
        ├── config.json
        ├── model.safetensors
        └── train_config.json
```
:::

::: info
After placing the model in the above directory, you can access it from within the Docker container at:

`/root/ros2_ws/src/physical_ai_tools/lerobot/outputs/train/`
:::

### 3. Bring up the OMX Follower Node

::: warning
When you launch the bringup, the robot will start moving. Please be careful, and double-check that the teleoperation node used during dataset recording has been disabled.
:::

`ROBOT PC`
```bash
cd /data/docker/open_manipulator/docker && ./container.sh enter
```
`ROBOT PC` `🐋 OPEN MANIPULATOR`
```bash
ros2 launch open_manipulator_bringup omx_f_follower_ai.launch.py
```

### 4. Run Inference

#### a. Launch Physical AI Server

::: warning
If the Physical AI Server is already running, you can skip this step.
:::

Go to **physical_ai_tools/docker** directory:

`USER PC`
```bash
cd physical_ai_tools/docker
```
Enter the **Physical AI Tools** Docker container:

```bash
./container.sh enter
```
Then, launch the Physical AI Server with the following command:

`USER PC` `🐋 PHYSICAL AI TOOLS`
```bash
ai_server
```

#### b. Open the Web UI

Open your web browser and navigate to the Web UI (Physical AI Manager).

(Refer to the [Dataset Preparation > Recording > 1. Open the Web UI](/omy/dataset_preparation_recording_omy#_1-open-the-web-ui))

On the **Home** page, select the type of robot you are using.

  <img src="/imitation_learning/web_ui_robot_type_selection.png" alt="Web UI" style="width: 40%; ">

#### c. Enter Task Instruction and Policy Path

Go to the `Inference` Page.

Enter **Task Instruction** and **Policy Path** in the **Task Info Panel**, located on the right side of the page.

  <img src="/imitation_learning/web_ui_inference_task_info.png" alt="Web UI" style="width: 50%; ">

- Task Information Field Descriptions

| Item | Description |
| -------- | --- |
| **Task Instruction** | A sentence that tells the robot what action to perform, such as `"pick and place object"`.|
| **Policy Path** <br>`🐋 PHYSICAL AI TOOLS`| The **absolute** path to your trained model directory **inside the Docker container**(`🐋 PHYSICAL AI TOOLS`). This should point to the folder containing your trained model files such as `config.json`, `model.safetensors`, and `train_config.json`. <br>See the **Policy Path Example** below for reference. |

::: info
**Policy Path Example**

```
/root/ros2_ws/src/physical_ai_tools/lerobot/outputs/train/
└── example_model_folder/
         ├── pretrained_model/    # ← This folder contains config.json, model.safetensors, train_config.json
         │   ├── config.json
         │   ├── model.safetensors
         │   └── train_config.json
         └── training_state/
             ├── optimizer_param_groups.json
             ├── optimizer_state.safetensors
             ├── rng_state.safetensors
             └── training_step.json

```
For a model folder structure like the one above, the **Policy Path** would be:

`/root/ros2_ws/src/physical_ai_tools/lerobot/outputs/train/example_model_folder/pretrained_model/`

:::

::: info
Recording during inference will be supported in a future update. Coming soon!
:::

#### d. Start Inference

To begin inference, use the **Control Panel** located at the bottom of the page:

  <img src="/imitation_learning/web_ui_control_panel.png" alt="Web UI" style="width: 100%; ">

  - The `Start` button begins inference.
  - The `Finish` button stops inference.
