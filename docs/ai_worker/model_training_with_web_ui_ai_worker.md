# Model Training with Web UI

This guide walks you through the training imitation learning models for the AI Worker, based on datasets collected via Web UI.

::: info
You can train the policy either on your local PC or an NVIDIA Jetson AGX Orin device.
:::

After [preparing your dataset](/ai_worker/dataset_preparation_ai_worker), you can proceed to train the policy model.

<a href="#training-on-nvidia-jetson-agx-orin" class="button-dataset-preparation-option">
Training on NVIDIA Jetson AGX Orin 
</a>
<a href="#training-on-your-pc" class="button-dataset-preparation-option">
Training on Your PC
</a>

## Training on NVIDIA Jetson AGX Orin

### 1. Launch Physical AI Server 

::: warning
If the Physical AI Server is already running, you can skip this step.
:::

Open a terminal on the Jetson device and enter the Docker container:
:::tabs key:robot-type
== BG2 Type
cd ai_worker && ./docker/container.sh enter
== SG2 Type
cd ai_worker && ./docker/container.sh enter
:::

Then, launch the Physical AI Server with the following command:

```bash
ai_server
```

### 2. Open the Web UI 

Open your web browser and navigate the Web UI (Physical AI Manager).

(Refer to the [Dataset Preparation > Web UI > 3. Open the Web UI](/ai_worker/dataset_preparation_with_web_ui_ai_worker#_3-open-the-web-ui))

On the **Home** page, select the type of robot you are using.

  <img src="/imitation_learning/web_ui_robot_type_selection.png" alt="Web UI" style="width: 40%; ">

### 3. Train the Policy

Go to the `Training` page.

  <img src="/imitation_learning/web_ui_training_page.png" alt="Web UI" style="width: 100%; ">

Select `Dataset`, `Policy type and device`.
Enter `Output folder name`.
Change `Additional Options` if you want.

:::tabs
== Dataset

The datasets located in the `~/.cache/huggingface/` (inside the Docker container, `/root/.cache/huggingface/`) directory will be displayed.
<img src="/imitation_learning/web_ui_dataset_selection.png" alt="Web UI" style="width: 50%; ">
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

## Training on Your PC

