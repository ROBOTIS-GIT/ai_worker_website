# Dataset Preparation

## Overview

### General Information

This dataset follows the standard [🤗 Hugging Face datasets format](https://huggingface.co/docs/datasets/index) and contains imitation learning demonstrations collected from the AI Worker via ROS 2 teleoperation using the [lerobot](https://github.com/huggingface/lerobot) framework.

### Tutorial Video: Data Collection Process

The following video demonstrates the complete process of recording a dataset with AI Worker, from teleoperation to dataset creation:

<YouTube videoId="hnJpFX2G3P4" />

### Dataset Schema

| Field              | Type            | Description          |
|--------------------|-----------------|----------------------|
| `action`           | `List[float32]` | Leader state vector  |
| `observation.state`| `List[float32]` | Follower state vector |
| `observation.images.cam_head`        | `Image`   | RGB image from the head-mounted camera |
| `observation.images.cam_wrist_1`     | `Image`   | RGB image from the first wrist camera |
| `observation.images.cam_wrist_2`     | `Image`   | RGB image from the second wrist camera |
| `timestamp`                          | `float32` | Time (in seconds) when the step was recorded |
| `frame_index`                        | `int64`   | Index of the frame within an episode |
| `episode_index`                      | `int64`   | Index of the episode |
| `index`                              | `int64`   | Global index across the dataset |
| `task_index`                         | `int64`   | Task identifier |


## Create Your Datasets

### 1. Authenticate with Hugging Face

To create a Hugging Face dataset, you first need to log in using a **write access token**, which can be generated from your [Hugging Face settings](https://huggingface.co/settings/tokens):

```bash
huggingface-cli login --token ${HUGGINGFACE_TOKEN} --add-to-git-credential
```

Store your Hugging Face username in a variable:

```bash
HF_USER=$(huggingface-cli whoami | head -n 1)
echo $HF_USER
```

### 2. Record Your Dataset

First, launch the ROS 2 teleoperation node:

```bash
container
bringup
```

Open a new terminal and navigate to the `lerobot` directory:

```bash
container
cd /root/colcon_ws/src/physical_ai_tools/lerobot
```

Run the following command to start recording your Hugging Face dataset:

```bash
python lerobot/scripts/control_robot.py \
  --robot.type=ffw \
  --control.type=record \
  --control.single_task="pick and place objects" \
  --control.fps=30 \
  --control.repo_id=${HF_USER}/ffw_test \
  --control.tags='["tutorial"]' \
  --control.warmup_time_s=5 \
  --control.episode_time_s=30 \
  --control.reset_time_s=10 \
  --control.num_episodes=10 \
  --control.push_to_hub=true \
  --control.play_sounds=false
```

::: tip
- Make sure to replace `${HF_USER}` with your actual Hugging Face username.
- To save the dataset locally without uploading to the Hugging Face Hub, set `--control.push_to_hub=false`.
:::

### Key Parameters to Customize

To create your own dataset, here are some important parameters you may want to adjust:

| Parameter                  | Description | Example |
|----------------------------|-------------|---------|
| `--control.repo_id`        | The Hugging Face dataset repository ID in the format `<username>/<dataset_name>` | `username/ffw_pick_place` |
| `--control.single_task`    | The name of the task you're performing | "pick and place objects" |
| `--control.fps`            | Frame rate for dataset recording | 30 (recommended) |
| `--control.episode_time_s` | Duration (in seconds) to record each episode | 30-60 for simple tasks |
| `--control.reset_time_s`   | Time allocated (in seconds) for resetting between episodes | 10-20 seconds |
| `--control.num_episodes`   | Total number of episodes to record | 10-50 depending on task complexity |

Of course, you can modify additional parameters as needed to fit your specific use case.

## Dataset Visualization

Once data collection is complete, you can preview and inspect your recorded dataset using the following command:

```bash
python lerobot/scripts/visualize_dataset_html.py \
  --repo-id ${HF_USER}/ffw_test
```

You should see an output similar to the following:

```
Fetching 4 files: 100%|██████████| 4/4 [00:00<00:00, 3457.79it/s]
.gitattributes: 100%|██████████| 2.46k/2.46k [00:00<00:00, 45.9MB/s]
Fetching 126 files: 100%|██████████| 126/126 [00:00<00:00, 266.66it/s]
Resolving data files: 100%|██████████| 30/30 [00:00<00:00, 662258.53it/s]
INFO 2025-05-15 16:18:07 set_html.py:364 Output directory already exists. Loading from it: '/tmp/lerobot_visualize_dataset_uo6ddbb1'
 * Serving Flask app 'visualize_dataset_html'
 * Debug mode: off
INFO 2025-05-15 16:18:07 _internal.py:97 WARNING: This is a development server. Do not use it in a production deployment. Use a production WSGI server instead.
 * Running on http://127.0.0.1:9090
INFO 2025-05-15 16:18:07 _internal.py:97 Press CTRL+C to quit
```

::: tip
Once the server is running, open [http://127.0.0.1:9090](http://127.0.0.1:9090) in your browser to preview the dataset.
:::
