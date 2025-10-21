# Model Training
This guide walks you through training imitation learning models for the AI Worker, based on datasets collected via the Web UI.

Once [preparing your dataset](/ai_worker/dataset_preparation_ai_worker) is done, the policy model can be trained using either the Web UI or the LeRobot CLI.

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

You can train the model on either the `Robot PC` or your own training PC (`USER PC`).
Choose the appropriate option below.

:::tabs key:pc-type
== ROBOT PC
Train the model on the Robot PC (NVIDIA Jetson AGX Orin)
== USER PC
Train the model on your separate workstation
:::


### 1. Setup Physical AI Tools
:::tabs key:pc-type
== ROBOT PC
**You can skip this step**
== USER PC
If the **Physical AI Tools** is already set up in your training PC, **you can skip this step**.

#### 1. Clone the Physical AI Tools repository:
Clone the repository along with all required submodules:

`USER PC`
```bash
git clone --recurse-submodules https://github.com/ROBOTIS-GIT/physical_ai_tools.git
```

#### 2. Start the Docker Container:
Navigate to **physical_ai_tools/docker** directory and start docker container:
```bash
cd physical_ai_tools/docker && ./container start
```

#### 3. Build Physical AI Server

Enter the docker container:

`USER PC`
```bash
./container.sh enter
```

Build with the following command:

`USER PC` `🐋 PHYSICAL AI TOOLS`
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
:::


### 2. Prepare Your Dataset

:::tabs key:pc-type
== ROBOT PC

The dataset to be used for training should be located at:

`~/physical_ai_tools/docker/huggingface/lerobot/${HF_USER}/`

Datasets collected using Physical AI Tools are automatically saved to that path. However, if you downloaded the dataset from a hub or copied it from another PC, you need to move the dataset to that location. 

Please refer to the folder structure tree below:

```
~/physical_ai_tools/docker/huggingface/lerobot/
  ├── USER_A/           # ← ${HF_USER} folder
  │   ├── dataset_1/    # ← Dataset
  │   │   ├── data/
  │   │   ├── meta/
  │   │   └── videos/
  │   └── dataset_2/
  └── USER_B/
      └── dataset_3/
```
== USER PC
The dataset to be used for training should be located at:

`<your_workspace>/physical_ai_tools/docker/huggingface/lerobot/${HF_USER}/`

If you downloaded the dataset from a hub or copied it from the Robot PC, you need to move the dataset to that location. 

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

::: tip
To copy your dataset from `Robot PC` to `User PC`, use the following `scp` command:


- If the **lerobot** folder does not exist in the `<your_workspace>/physical_ai_tools/docker/huggingface/` path, please create the folder with the following command:

`USER PC`
```bash
cd <your_workspace>/physical_ai_tools/docker/huggingface && mkdir lerobot
```

- Change ownership of the model directory. 

```bash
cd <your_workspace>/physical_ai_tools && sudo chown -R ${USER} ./
```

`ROBOT PC`

```bash
scp -r ~/physical_ai_tools/docker/huggingface/lerobot/${HF_USER}/ffw_test <USER>@<IP>:<your_workspace>/physical_ai_tools/docker/huggingface/lerobot/${HF_USER}/
```

::: warning
Do not add a trailing slash (/) at the end of `${HF_USER}/ffw_test` in the scp command.
:::


::: info
- `${HF_USER}` can be any folder name you prefer.
- `<your_workspace>` is the directory containing physical_ai_tools
:::

### 3. Train the Policy

#### a. Launch Physical AI Server
::: warning
If the Physical AI Server is already running, you can skip this step.
:::

:::tabs key:pc-type
== ROBOT PC
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
== USER PC
Go to **physical_ai_tools/docker** directory:

`USER PC`
```bash
cd physical_ai_tools/docker
```

Enter the **Physical AI Tools** Docker container:

`USER PC`
```bash
./container.sh enter
```
Then, launch the Physical AI Server with the following command:

`USER PC` `🐋 PHYSICAL AI TOOLS`
```bash
ai_server
```
:::

#### b. Open the Web UI

Open your web browser and navigate to the Web UI (Physical AI Manager).

(Refer to the [Dataset Preparation > Web UI > 1. Open the Web UI](/ai_worker/dataset_preparation_recording_ai_worker#_1-open-the-web-ui) for more details.)

:::tabs key:pc-type
== ROBOT PC
Open your web browser and access

`http://ffw-{serial number}.local`
== USER PC
Open your web browser and access

`http://{IP address of USER PC}`<br>
or<br>
`http://localhost`
:::

On the **Home** page, select the type of robot you are using.

  <img src="/imitation_learning/web_ui/home/robot_type_selection.png" alt="Web UI" style="width: 50%; display: block; margin: 0 auto;">

#### c. Train the Policy

Go to the `Training` page and follow the steps below:

  <img src="/imitation_learning/web_ui/training/page.png" alt="Web UI" style="width: 100%; ">

At the bottom of the page, select either **New Training** or **Resume Training**.

| Training Type | Description | When to Use |
|---------------|-------------|-------------|
| **New Training** | Start training a new model | - First time training<br>- Starting with a new dataset<br>- Training with a different policy on the same dataset |
| **Resume Training** | Continue training from a saved checkpoint | - Training was interrupted and you want to resume<br>- Want to incorporate additional datasets into the model<br>- Want to modify training options (Steps, Save Frequency, etc.) and retrain |


:::tabs
== New Training

For new training, follow these steps:

- **Step 1**: Select the `Dataset`, `Policy Type` and `Device`.

- **Step 2**: Enter the `Output Folder Name`.

- **Step 3**: (Optional) Modify `Additional Options` if needed.

== Resume Training

For resuming training, follow these steps:

- **Step 1**: Select the **policy path to resume** with file browsing(📁) and click the **`Load`** button to load the training informations(Dataset, Policy, Device, Output Folder Name, and Additional Options) automatically.
<img src="/imitation_learning/web_ui/training/resume_policy_path.png" alt="Web UI" style="width: 70%; ">
- **Step 2**: (Optional) Change the training `Dataset` if needed.
- **Step 3**: (Optional) Modify `Additional Options` if needed.
:::

- **Training Informations**
:::tabs
== Dataset

The datasets stored in the `~/physical_ai_tools/docker/huggingface/` directory on the host (or `/root/.cache/huggingface/` inside the Docker container) will be listed automatically.
<img src="/imitation_learning/web_ui/training/dataset_selection.png" alt="Web UI" style="width: 50%; ">

== Policy type and device
Select the policy and computation device for training your model.
- **Policy Type**: Choose the imitation learning algorithm (e.g., act, pi0, etc.).
- **Device**: Select the hardware to be used for training (e.g. cuda, cpu, etc.)

<img src="/imitation_learning/web_ui/training/policy_selection.png" alt="Web UI" style="width: 50%; ">

== Output folder name 
Specify the name of the folder where your trained model will be saved. Then, check for duplicates. 
This folder will be created in the default output directory (`<your_workspace>/physical_ai_tools/lerobot/outputs/`).
Choose a descriptive and meaningful name so you can easily identify the trained model later.

<img src="/imitation_learning/web_ui/training/output_folder_input.png" alt="Web UI" style="width: 50%; ">

== Additional options
<img src="/imitation_learning/web_ui/training/additional_options.png" alt="Web UI" 
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

:::tabs
== New Training
Click `Start Training` to begin training the policy.

<img src="/imitation_learning/web_ui/training/button_start_training.png" alt="Web UI" style="width: 30%; ">

== Resume Training
Click `Resume Training` to continue training from the checkpoint.

<img src="/imitation_learning/web_ui/training/button_resume_training.png" alt="Web UI" style="width: 35%; ">
::: 

:::tabs key:pc-type
== ROBOT PC
The training results will be saved in the `~/physical_ai_tools/lerobot/outputs/train/` directory.
== USER PC
The training results will be saved in the `<your_workspace>/physical_ai_tools/lerobot/outputs/train/` directory.
:::

You can monitor the training loss while training is in progress.
<img src="/imitation_learning/web_ui/training/training_loss.png" alt="Web UI" style="width: 50%; display: block; margin: 0 auto;">

### (Optional) Uploading Checkpoints to Hugging Face

:::tabs key:pc-type
== ROBOT PC
Enter the **Physical AI Tools** Docker container:

`ROBOT PC`
```bash
cd ~/physical_ai_tools && ./docker/container.sh enter
```

Navigate to the LeRobot directory:

`ROBOT PC` `🐋 PHYSICAL AI TOOLS`
```bash
cd /root/ros2_ws/src/physical_ai_tools/lerobot
```

To upload the latest trained checkpoint to the Hugging Face Hub:

```bash
huggingface-cli upload ${HF_USER}/act_ffw_test \
  outputs/train/act_ffw_test/checkpoints/last/pretrained_model
```
== USER PC
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
huggingface-cli upload ${HF_USER}/act_ffw_test \
  outputs/train/act_ffw_test/checkpoints/last/pretrained_model
```
:::

This makes your model accessible from anywhere and simplifies deployment.

## Model Training With LeRobot CLI

### 1. Prepare Your Dataset

:::tabs key:pc-type
== ROBOT PC
The dataset to be used for training should be located at:

`~/physical_ai_tools/docker/huggingface/lerobot/${HF_USER}/`.

If your dataset is in a different location, please move it to this path.
== USER PC
The dataset to be used for training should be located at:

`<your_workspace>/physical_ai_tools/docker/huggingface/lerobot/${HF_USER}/`.

If your dataset is in a different location, please move it to this path.
:::

::: info
You can replace `${HF_USER}` with the folder name you used when recording your dataset.
:::

### 2. Train the Policy

:::tabs key:pc-type
== ROBOT PC
Enter the **Physical AI Tools** Docker container:

`ROBOT PC`
```bash
cd ~/physical_ai_tools && ./docker/container.sh enter
```

Navigate to the LeRobot directory:

`ROBOT PC` `🐋 PHYSICAL AI TOOLS`
```bash
cd /root/ros2_ws/src/physical_ai_tools/lerobot
```

Once the dataset has been transferred, you can train a policy using the following command:

```bash
python -m lerobot.scripts.train \
  --dataset.repo_id=${HF_USER}/ffw_test \
  --policy.type=act \
  --output_dir=outputs/train/act_ffw_test \
  --policy.device=cuda \
  --log_freq=100 \
  --save_freq=1000 \
  --policy.push_to_hub=false
```
== USER PC
Go to **physical_ai_tools/docker** directory:

`USER PC`
```bash
cd physical_ai_tools/docker
```
Enter the **Physical AI Tools** Docker container:

```bash
./container.sh enter
```

Navigate to the LeRobot directory:

`USER PC` `🐋 PHYSICAL AI TOOLS`
```bash
cd /root/ros2_ws/src/physical_ai_tools/lerobot
```

Once the dataset has been transferred, you can train a policy using the following command:

```bash
python -m lerobot.scripts.train \
  --dataset.repo_id=${HF_USER}/ffw_test \
  --policy.type=act \
  --output_dir=outputs/train/act_ffw_test \
  --policy.device=cuda \
  --log_freq=100 \
  --save_freq=1000 \
  --policy.push_to_hub=false
```
:::