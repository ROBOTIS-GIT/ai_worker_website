# Model Training
This guide walks you through training imitation learning models for OMY, based on datasets collected via the Web UI.

Once [preparing your dataset](/omy/dataset_preparation_omy) is done, the policy model can be trained using either the Web UI or the LeRobot CLI.

You can choose one of the following options:

<div style='display: flex; justify-content: flex-start; gap: 30px;'>
<a href="#model-training-with-web-ui" class="button-shortcut">
Option 1<br>Web UI (Recommended)
</a>

<a href="#model-training-with-lerobot-cli" class="button-shortcut">
Option 2<br>LeRobot CLI (Optional)
</a>
</div>


## Model Training With Web UI

### 1. Prepare Your Dataset

The dataset to be used for training should be located at

`USER PC`

 `<your_workspace>/physical_ai_tools/docker/huggingface/lerobot/${HF_USER}/`
 
Datasets collected using Physical AI Tools are automatically saved to that path. However, if you downloaded the dataset from a hub or copied it from another PC, you need to move the dataset to that location. 

::: info
- `${HF_USER}` can be any folder name you prefer.
- `<your_workspace>` is the directory containing physical_ai_tools

:::

Please refer to the folder structure tree below:

```
<your_workspace>/physical_ai_tools/docker/huggingface/lerobot/
  ├── USER_A/           # ← ${HF_USER} folder
  │   ├── dataset_1/    # ← Dataset
  │   │   ├── data/
  │   │   ├── meta/
  │   │   └── videos/
  │   └── dataset_2/
  └── USER_B/
      └── dataset_3/
```

### 2. Setup Physical AI Tools Docker Container
::: warning
If the Physical AI Tools is already set up, you can skip this step.
:::

If you haven't set up the Physical AI Tools Docker container, please refer to the link below for setup instructions.

[Setup Physical AI Tools Docker Container](/omy/dataset_preparation_prerequisites_omy#setup-physical-ai-tools-docker-container)

### 3. Train the Policy

#### a. Launch Physical AI Server
::: warning
If the Physical AI Tools Docker container is already running, you can skip this step.
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

(Refer to the [Dataset Preparation > Web UI > 1. Open the Web UI](/omy/dataset_preparation_recording_omy#_1-open-the-web-ui) for more details.)

On the **Home** page, select the type of robot you are using.

  <img src="/imitation_learning/web_ui_robot_type_selection.png" alt="Web UI" style="width: 50%; display: block; margin: 0 auto;">

#### c. Train the Policy

Go to the `Training` page and follow the steps below:

  <img src="/imitation_learning/web_ui_training_page.png" alt="Web UI" style="width: 100%; ">

- Step 1: Select the `Dataset`, `Policy Type` and `Device`.
- Step 2: Enter the `Output Folder Name`.
- Step 3: (Optional) Modify `Additional Options` if needed.

For more information about these **options**, please refer to the descriptions below.

:::tabs
== Dataset

The datasets stored in the `<your_workspace>/physical_ai_tools/docker/huggingface/` directory on the host (or `/root/.cache/huggingface/` inside the Docker container) will be listed automatically.
<img src="/imitation_learning/web_ui_training_dataset_selection.png" alt="Web UI" style="width: 50%; ">

== Policy type and device
Select the policy and computation device for training your model.
- **Policy Type**: Choose the imitation learning algorithm (e.g., act, pi0, etc.).
- **Device**: Select the hardware to be used for training (e.g. cuda, cpu, etc.)

<img src="/imitation_learning/web_ui_training_policy_selection.png" alt="Web UI" style="width: 50%; ">

== Output folder name
Specify the name of the folder where your trained model will be saved. Then, check for duplicates. 
This folder will be created in the default output directory (`<your_workspace>/physical_ai_tools/lerobot/outputs/`).
Choose a descriptive and meaningful name so you can easily identify the trained model later.

<img src="/imitation_learning/web_ui_training_output_folder_input.png" alt="Web UI" style="width: 50%; ">

== Additional options
<img src="/imitation_learning/web_ui_training_additional_options.png" alt="Web UI" 
style="width: 50%; ">

- Additional Option Descriptions

| Parameter | Description |
|-----------|-------------|
| **seed** | Random seed for reproducible training results. Setting the same seed ensures consistent model training across different runs. |
| **num workers** | Number of worker processes for data loading. Higher values can speed up training but consume more memory. Recommended: 4-8 for most systems. |
| **batch size** | Number of training samples processed simultaneously. Larger batch sizes can improve training stability but require more GPU memory. |
| **steps** | Total number of training steps to perform. This determines how long the training will run. |
| **eval frequency** | How often (in steps) to evaluate the model on validation data. Lower values provide more frequent monitoring but slow down training. |
| **log frequency** | How often (in steps) to log training metrics and losses. Used for monitoring training progress. |
| **save frequency** | How often (in steps) to save model checkpoints. Lower values create more backup points but use more storage space. |
:::

Click `Start Training` to begin training the policy. The training results will be saved in the `physical_ai_tools/lerobot/outputs/train/` directory.

You can monitor the training loss while training is in progress.
<img src="/imitation_learning/web_ui_training_loss.png" alt="Web UI" style="width: 50%; display: block; margin: 0 auto;">

### (Optional) Uploading Checkpoints to Hugging Face

Navigate to **physical_ai_tools/docker** directory and enter the Docker container:

`USER PC`
```bash
cd physical_ai_tools/docker
```

```bash
./container.sh enter
```

Navigate to the LeRobot directory:

`USER PC` `🐋 PHYSICAL AI TOOLS`
```bash
cd /root/ros2_ws/src/physical_ai_tools/lerobot
```

To upload the latest trained checkpoint to the Hugging Face Hub:

```bash
huggingface-cli upload ${HF_USER}/act_omy_test \
  outputs/train/act_omy_test/checkpoints/last/pretrained_model
```

This makes your model accessible from anywhere and simplifies deployment.

## Model Training With LeRobot CLI

### 1. Prepare Your Dataset

The dataset to be used for training should be located at `<your_workspace>/physical_ai_tools/docker/huggingface/lerobot/${HF_USER}/`. If your dataset is in a different location, please move it to this path.

::: info
- You can replace `${HF_USER}` with the folder name you used when recording your dataset.
- `<your_workspace>` is the directory containing physical_ai_tools
:::


### 2. Train the Policy

Go to **physical_ai_tools/docker** directory:

`USER PC`
```bash
cd physical_ai_tools/docker
```
Enter the **Physical AI Tools** Docker container:

```bash
container.sh enter
```

Navigate to the LeRobot directory:

`USER PC` `🐋 PHYSICAL AI TOOLS`
```bash
cd /root/ros2_ws/src/physical_ai_tools/lerobot
```

Once the dataset has been transferred, you can train a policy using the following command:

```bash
python -m lerobot.scripts.train \
  --dataset.repo_id=${HF_USER}/omy_test \
  --policy.type=act \
  --output_dir=outputs/train/act_omy_test \
  --policy.device=cuda \
  --log_freq=100 \
  --save_freq=1000 \
  --policy.push_to_hub=false
```
