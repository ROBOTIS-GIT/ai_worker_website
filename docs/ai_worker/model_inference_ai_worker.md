# Model Inference with Web UI

Once your model is trained, you can deploy it on the AI Worker for inference.

## Model Deployment and Inference

### 1. Prepare Your Model

Choose one of the following methods.

**`Option 1)` Download your model from Hugging Face**

You can download a policy model from Hugging Face. 
Detailed steps are provided [below](#download-policy-model).

You can proceed to the next step: 👉 [2. Launch the ROS 2 Follower Node](#_2-launch-the-ros-2-follower-node)

**`Option 2)` Manually copy your model to the target directory**


Please place your trained model in the following directory:

`ROBOT PC`

`~/physical_ai_tools/lerobot/outputs/train/`

Models trained on the Robot PC using Physical AI Tools are automatically saved to this path. 
However, if you downloaded the model from a hub (without using Physical AI Tools) or copied it from another PC, you need to move the model to this location. 


::: details Available Folder Structures
Please refer to the folder structure tree below:

The following folder structures are all valid (example_model_folder_1, 2, 3).

```
~/physical_ai_tools/lerobot/outputs/train/
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

::: tip
To copy your model from the `User PC` to the `Robot PC`, use the following command:

- Change ownership of the model directory. 

`ROBOT PC`  (**not inside the Docker container**)
```bash
sudo chown -R robotis ./
```
- Copy the model from the User PC to the Robot PC using the `scp` command:

`USER PC`
```bash
scp -r <your model folder's directory> robotis@ffw-<your robot's serial number>.local:~/physical_ai_tools/lerobot/outputs/train
```
:::

::: info
After placing the model in the above directory, you can access it from within the Docker container at:

`/root/ros2_ws/src/physical_ai_tools/lerobot/outputs/train/`
:::

### 2. Launch the ROS 2 Follower Node

::: warning
Please **deactivate the ROS 2 teleoperation node** launched in the [Prerequisites > Launch the ROS 2 teleoperation node](/ai_worker/dataset_preparation_prerequisites_ai_worker#launch-the-ros-2-teleoperation-node) section before proceeding.
:::

`ROBOT PC`
```bash
cd ~/ai_worker/docker && ./container.sh enter
```
`ROBOT PC` `🐋 AI WORKER`

:::tabs key:robot-type
== BG2 Type
```bash
ffw_bg2_follower_ai
```
== SG2 Type
```bash
ffw_sg2_follower_ai
```
:::

### 3. Run Inference

#### a. Launch Physical AI Server

::: warning
If the Physical AI Server is already running, you can skip this step.
:::

Enter the **Physical AI Tools** Docker container:

`ROBOT PC`
```bash
cd ~/physical_ai_tools && ./docker/container.sh enter
```

Then, launch the Physical AI Server with the following command:

`ROBOT PC` `🐋 PHYSICAL AI TOOLS`
```bash
ai_server
```

#### b. Open the Web UI

Open your web browser and navigate the Web UI (Physical AI Manager).

(Refer to the [Dataset Preparation > Recording > 1. Open the Web UI](/ai_worker/dataset_preparation_recording_ai_worker#_1-open-the-web-ui))

On the **Home** page, select the type of robot you are using.

  <img src="/imitation_learning/web_ui/home/robot_type_selection.png" alt="Web UI" style="width: 40%; ">

#### c. Enter Task Instruction and Policy Path

Go to the `Inference` Page.

Enter **Task Instruction** and **Policy Path** in the **Task Info Panel**, located on the right side of the page.

  <img src="/imitation_learning/web_ui/inference/task_info.png" alt="Web UI" style="width: 50%; ">

- Task Information Field Descriptions

| Item | Description |
| -------- | --- |
| **Task Instruction** | A sentence that tells the robot what action to perform, such as `"pick and place object"`.|
| **Policy Path** <br>`🐋 PHYSICAL AI TOOLS`| The **absolute** path to your trained model directory **inside the Docker container**(`🐋 PHYSICAL AI TOOLS`). This should point to the folder containing your trained model files such as `config.json`, `model.safetensors`, and `train_config.json`. <br>See the **Policy Path Example** below for reference. |
| **FPS** | FPS should be set to the same FPS at which you collected the training data for your model, which serves as the control frequency.|

::: tip
**Entering Policy Path**

You can either click the `Browse Policy Path` button to select the desired model folder, or directly enter the desired path in the text input field.

<img src="/imitation_learning/web_ui/inference/browse_button.png" alt="Web UI" style="width: 30%; ">
<br>
<img src="/imitation_learning/web_ui/inference/file_browser.png" alt="Web UI" style="width: 100%; ">
:::

<a id="download-policy-model"></a>
::: tip
**Download Policy Model from Hugging Face**

You can download your model from Hugging Face. 

Click the `Download Policy` button to open a popup for downloading a policy model. 

  <img src="/imitation_learning/web_ui/inference/download_policy_button.png" alt="Web UI" style="width: 50%; ">
  <br>
  <img src="/imitation_learning/web_ui/inference/download_policy_modal.png" alt="Web UI" style="width: 70%; ">

Select the Hugging Face User ID and enter the repository to download. 

Click the `Download` button to start the download.

When the download completes, click `Finish` to close the popup. 

The downloaded model path is automatically filled into Policy Path. 

  <img src="/imitation_learning/web_ui/inference/downloaded_path_filled.png" alt="Web UI" style="width: 30%; ">


⚠️ **Important — Network recommendation**: Policy models are typically large in size. A **wired Ethernet connection** is **highly recommended** for downloading to ensure stable and fast download speeds.


:::

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

  <img src="/imitation_learning/web_ui/record/control_panel.png" alt="Web UI" style="width: 100%; ">

  - The `Start` button begins inference.
  - The `Finish` button stops inference.
