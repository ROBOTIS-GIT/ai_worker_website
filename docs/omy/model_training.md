# Model Training
This guide walks you through the training imitation learning models for the OMY, based on datasets collected via Web UI or Lerobot CLI

After [preparing your dataset](/omy/dataset_preparation_omy), you can proceed to train the policy model.

You can choose one of the following options:


<div style='display: flex; justify-content: flex-start; gap: 30px;'>
<a href="#model-training-with-web-ui" class="button-dataset-preparation-option">
Option 1<br>Web UI (Recommended)
</a>

<a href="#model-training-with-lerobot-cli" class="button-dataset-preparation-option">
Option 2<br>LeRobot CLI (Optional)
</a>
</div>

---

# Model Training With Web UI


### 1. Prepare Your Dataset

The dataset to be used for training should be located at `/home/.cache/huggingface/lerobot/${HF_USER}/`. If your dataset is in a different location, please move it to this path.

::: info
You can replace `${HF_USER}` with any folder name you prefer.
:::

### 2. Setup Physical AI Tools
::: warning
If the Physical AI Tools is already set, you can skip this step.
:::

```bash
git clone --recurse-submodules https://github.com/ROBOTIS-GIT/physical_ai_tools.git
```
```bash
cd physical_ai_tools/docker
```
```bash
./container start
```

### 3. Train the Policy

#### a. Launch Physical AI Server
::: warning
If the Physical AI Server is already running, you can skip this step.
:::

Go to **physical_ai_tools/docker** directory
```bash
cd physical_ai_tools/docker
```
Enter the **Physical AI Tools** docker container:

```bash
container.sh enter
```
Then, launch the Physical AI Server with the following command:
```bash
ai_server
```

#### b. Open the Web UI 

Open your web browser and navigate the Web UI (Physical AI Manager).

(Refer to the [Dataset Preparation > Web UI > 3. Open the Web UI](/ai_worker/dataset_preparation_with_web_ui_ai_worker#_3-open-the-web-ui))

On the **Home** page, select the type of robot you are using.

  <img src="/imitation_learning/web_ui_robot_type_selection.png" alt="Web UI" style="width: 40%; ">

#### c. Train the Policy

Go to the `Training` page:

  <img src="/imitation_learning/web_ui_training_page.png" alt="Web UI" style="width: 100%; ">

- Select `Dataset` and `Policy type and device`
- Enter `Output folder name`
- Change `Additional Options` if you want

:::tabs
== Dataset

The datasets located in the `~/.cache/huggingface/` (inside the Docker container, `/root/.cache/huggingface/`) directory will be displayed.
<img src="/imitation_learning/web_ui_training_dataset_selection.png" alt="Web UI" style="width: 50%; ">
== Policy type and device
Select the policy algorithm and computation device for training your model.
- **Policy Type**: Choose the imitation learning algorithm (e.g., act, pi0, etc.)
- **Device**: Select the hardware for training (e.g. cuda, cpu, npu, etc.)

<img src="/imitation_learning/web_ui_training_policy_selection.png" alt="Web UI" style="width: 50%; ">
== Output folder name 
Specify the name of the folder where your trained model will be saved. Then, check for duplicates. 

This folder will be created in the default output directory (`<your_workspace>/physical_ai_tools/docker/output/`).

Choose a descriptive name to easily identify your trained model later.

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

Click `Start Training` to start

Training results will be saved to the `physical_ai_tools/lerobot/outputs/train/` directory.

### (Optional) Uploading Checkpoints to Hugging Face

To upload the latest trained checkpoint to the Hugging Face Hub:

```bash
huggingface-cli upload ${HF_USER}/act_omy_test \
  outputs/train/act_omy_test/checkpoints/last/pretrained_model
```

This makes your model accessible from anywhere and simplifies deployment.

---

# Model Training With LeRobot CLI

### 1. Set Up the LeRobot Framework

First, follow the [LeRobot installation instructions](https://github.com/ROBOTIS-GIT/lerobot) to set up the framework locally.


### 2. Prepare Your Dataset

The dataset to be used for training should be located at `/home/.cache/huggingface/lerobot/${HF_USER}/`. If your dataset is in a different location, please move it to this path.

::: info
You can replace `${HF_USER}` with any folder name you prefer.
:::

### 3. Train the Policy

Once the dataset has been transferred, you can train a policy using the following command:

```bash
python lerobot/scripts/train.py \
  --dataset.repo_id=${HF_USER}/ffw_test \
  --policy.type=act \
  --output_dir=outputs/train/act_ffw_test \
  --policy.device=cuda \
  --log_freq=100 \
  --save_freq=1000
```

Training time depends on your hardware and dataset size, but typically ranges from several hours to a full day.

### (Optional) Uploading Checkpoints to Hugging Face

To upload the latest trained checkpoint to the Hugging Face Hub:

```bash
huggingface-cli upload ${HF_USER}/act_ffw_test \
  outputs/train/act_ffw_test/checkpoints/last/pretrained_model
```

This makes your model accessible from anywhere and simplifies deployment.
