# LeRobot Imitation Learning for OMX

## Overview

This tutorial explains how to train a neural network to control OMX autonomously using LeRobot. You'll learn the complete workflow from data collection to model deployment.

By following these steps, you'll be able to replicate tasks such as picking up objects and placing them with high success rates.

## Before You Begin

First, identify the bus servo adapter ports for the leader and follower by running the following command:
```bash
lerobot-find-port
```

You will see an output similar to:

```bash
(lerobot) username@username:~/lerobot$ lerobot-find-port
Finding all available ports for the MotorsBus.
Ports before disconnecting: ['/dev/ttyACM1', '/dev/ttyACM0', ...]
Remove the USB cable from your MotorsBus and press Enter when done.
```

Next, disconnect the USB cable from either the leader or the follower and press Enter to determine the corresponding port.
The output will look like this:

```bash
The port of this MotorsBus is '/dev/ttyACM1'
Reconnect the USB cable.

```

For the purposes of this tutorial, we assume:
- Follower’s port → /dev/ttyACM0
- Leader’s port → /dev/ttyACM1

## Teleoperation

You can test if OMX is working properly through teleoperation. 

### 1. Teleoperate OMX

Start teleoperation with OMX:

```bash
python -m lerobot.teleoperate \
    --robot.type=omx_follower \
    --robot.port=/dev/ttyACM0 \
    --robot.id=omx_follower_arm \
    --teleop.type=omx_leader \
    --teleop.port=/dev/ttyACM1 \
    --teleop.id=omx_leader_arm
```

> **Note**: the `--robot.id/--teleop.id` values persist metadata (e.g., calibrations/settings). Use consistent IDs across teleop, recording, and evaluation for the same setup.

### 2. Teleoperate with camera

For camera integration:

```bash
python -m lerobot.teleoperate \
    --robot.type=omx_follower \
    --robot.port=/dev/ttyACM0 \
    --robot.id=omx_follower_arm \
    --robot.cameras="{ front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}}" \
    --teleop.type=omx_leader \
    --teleop.port=/dev/ttyACM1 \
    --teleop.id=omx_leader_arm \
    --display_data=true
```

## Data Collection

### 1. Record Dataset

We use the Hugging Face hub features for uploading your dataset. If you haven’t previously used the Hub, make sure you can log in via the CLI using a write-access token, this token can be generated from the Hugging Face settings.

Add your token to the CLI by running this command:

```bash
huggingface-cli login --token ${HUGGINGFACE_TOKEN} --add-to-git-credential
```
Then store your Hugging Face username in a variable:

```bash
HF_USER=$(hf auth whoami | head -n 1)
echo $HF_USER
```

Record your first dataset:

> **Note**: If you have teleoperation running from the previous step, please stop it first before running this command as it includes teleoperation with camera.

```bash
lerobot-record \
    --robot.type=omx_follower \
    --robot.port=/dev/ttyACM0 \
    --robot.id=omx_follower_arm \
    --robot.cameras="{ front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}}" \
    --teleop.type=omx_leader \
    --teleop.port=/dev/ttyACM1 \
    --teleop.id=omx_leader_arm \
    --display_data=true \
    --dataset.repo_id=${HF_USER}/record-test \
    --dataset.num_episodes=5 \
    --dataset.single_task="Pick up the Dynamixel Motor"
```

### 2. Recording Controls & Details

**Keyboard Shortcuts:**
- **Right Arrow (→)**: End episode and move to next
- **Left Arrow (←)**: Cancel episode and re-record
- **Escape (ESC)**: Stop session and upload dataset

**Recording Parameters:**
```bash
--dataset.episode_time_s=60    # Episode duration (default: 60s)
--dataset.reset_time_s=60      # Reset time (default: 60s)
--dataset.num_episodes=50      # Total episodes (default: 50)
```

**Additional behavior:**
- Disable automatic push to the Hub with `--dataset.push_to_hub=false`
- Resume a failed/interrupted session with `--resume=true`
  - When resuming, set `--dataset.num_episodes` to the number of additional episodes to record (not the final total)


### 3. Dataset Management

**Local Storage:**
- Dataset stored in: `~/.cache/huggingface/lerobot/{repo-id}`

**Dataset Upload:**
```bash
huggingface-cli upload ${HF_USER}/record-test ~/.cache/huggingface/lerobot/${HF_USER}/record-test --repo-type dataset
echo https://huggingface.co/datasets/${HF_USER}/record-test
```

## Data Visualization

### View Dataset Online

If uploaded with `--control.push_to_hub=true`:

```bash
echo ${HF_USER}/record-test
```

### Replay Episodes

Replay recorded episodes:

```bash
lerobot-replay \
  --robot.type=omx_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.id=omx_follower_arm \
  --dataset.repo_id=${HF_USER}/record-test \
  --dataset.episode=0 # choose the episode you want to replay
```

## Model Training

### 1. Train Policy

Train your imitation learning model:

```bash
lerobot-train \
  --dataset.repo_id=${HF_USER}/record-test \
  --policy.type=act \
  --output_dir=outputs/train/omx_act_policy \
  --job_name=act_record-test \
  --policy.device=cuda \
  --wandb.enable=true \
  --policy.repo_id=${HF_USER}/omx_act_policy
```
Let’s break down the command:

- **Dataset repo**: We point `--dataset.repo_id` to your training dataset. In this guide it’s `${HF_USER}/record-test`.
- **Policy type**: `--policy.type=act` selects the ACT policy. It loads the appropriate configuration and automatically adapts to your robot’s setup (motor state/action dimensions and any cameras) as stored in the dataset.
- **Compute device**: `--policy.device=cuda` runs on an NVIDIA GPU. On Apple Silicon, use `--policy.device=mps`.
- **Experiment logging (optional)**: `--wandb.enable=true` enables Weights & Biases logging. If you use it, make sure you’re logged in first:

  ```bash
  wandb login
  ```
::: tip
If you do not want to push your trained model to the Hugging Face Hub,
use the option `--policy.push_to_hub=false` instead of `--policy.repo_id=${HF_USER}/omx_act_policy`.
:::

### 2. Resume Training

To resume training from the latest checkpoint:

```bash
python -m lerobot.scripts.train \
  --config_path=outputs/train/omx_act_policy/checkpoints/last/pretrained_model/train_config.json \
  --resume=true
```

### 3. Upload Checkpoints (Optional)

To upload your trained model to the Hugging Face Hub:

```bash
huggingface-cli upload ${HF_USER}/omx_policy \
  outputs/train/omx_act_policy/checkpoints/last/pretrained_model
```

## Model Evaluation

### Run inference and evaluate your policy

Evaluate your trained model:

```bash
python -m lerobot.record \
  --robot.type=omx_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.cameras="{ front: {type: opencv, index_or_path: 0, width: 1920, height: 1080, fps: 30}}" \
  --robot.id=omx_follower_arm \
  --display_data=false \
  --dataset.repo_id=${HF_USER}/eval_act_omx \
  --dataset.single_task="Pick and place object" \
  --policy.path=${HF_USER}/omx_act_policy
```

**Note**: use an `eval_*` dataset name (e.g., `eval_act_omx`) to clearly separate evaluation runs.

As you can see, it’s almost the same command as previously used to record your training dataset. Two things have changed:

- There is an additional `--control.policy.path` argument which indicates the path to your policy checkpoint (e.g., `outputs/train/eval_act_omx/checkpoints/last/pretrained_model`). You can also use the model repository if you uploaded a model checkpoint to the hub (e.g., `${HF_USER}/omx_act_policy`).
- The dataset name begins with `eval_` to reflect that you are running inference (e.g., `${HF_USER}/eval_act_omx`).
