# Model Training
This guide walks you through training imitation learning models for OMX, based on datasets collected via the Web UI.

Once [preparing your dataset](/omx/dataset_preparation_omx) is done, the policy model can be trained using either the Web UI or the LeRobot CLI.

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

 `<your_workspace>/physical_ai_tools/docker/.cache/huggingface/lerobot/${HF_USER}/`
 
Datasets collected using Physical AI Tools are automatically saved to that path. However, if you downloaded the dataset from a hub or copied it from another PC, you need to move the dataset to that location. 

::: info
- `${HF_USER}` can be any folder name you prefer.
- `<your_workspace>` is the directory containing physical_ai_tools

:::

Please refer to the folder structure tree below:

```
<your_workspace>/physical_ai_tools/docker/.cache/huggingface/lerobot/
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

[Setup Physical AI Tools Docker Container](/omx/dataset_preparation_prerequisites_omx#setup-physical-ai-tools-docker-container)

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

(Refer to the [Dataset Preparation > Web UI > 1. Open the Web UI](/omx/dataset_preparation_recording_omx#_1-open-the-web-ui) for more details.)

On the **Home** page, select the type of robot you are using.

  <img src="/imitation_learning/web_ui_robot_type_selection.png" alt="Web UI" style="width: 40%; ">

#### c. Train the Policy

Go to the `Training` page and follow the steps below:

  <img src="/imitation_learning/web_ui_training_page.png" alt="Web UI" style="width: 100%; "> 