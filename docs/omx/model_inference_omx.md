# Model Inference with Web UI

Once your model is trained, you can deploy it on the OMX for inference.

## Model Deployment and Inference

### 1. Setup Physical AI Tools Docker Container
::: warning
If the Physical AI Tools is already set up, you can skip this step.
:::

If you haven't set up the Physical AI Tools Docker container, please refer to the link below for setup instructions.

[Setup Physical AI Tools Docker Container](/omx/dataset_preparation_prerequisites_omx#setup-physical-ai-tools-docker-container)

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

### 3. Bring up OMX Follower Node

::: warning
The robot will start moving when you run bringup. Please be careful.
:::

`ROBOT PC`
```bash
cd /data/docker/open_manipulator/docker && ./container.sh enter
```
`ROBOT PC` `🐋 OPEN MANIPULATOR`
```bash
ros2 launch open_manipulator_bringup omx_f_follower_ai.launch.py ros2_control_type:=omx_f_smooth
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