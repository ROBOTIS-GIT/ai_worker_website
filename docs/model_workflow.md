# Model Workflow

This guide explains the process of training and deploying imitation learning models for the AI Worker using your prepared datasets.

## Model Training

> **Note:** You can train a policy using either your local PC or an NVIDIA Jetson AGX Orin device

After [preparing your dataset](/dataset_preparation), you can proceed to train the policy model.

### Training on NVIDIA Jetson AGX Orin

#### 1. Open a new terminal and enter the Docker container:
```bash
cd ai_worker
./docker/container.sh enter
```

#### 2. Navigate to the lerobot directory inside the Docker container:

```bash
cd /root/ros2_ws/src/physical_ai_tools/lerobot
```

#### 3. Run the training script with the following command:

```bash
python lerobot/scripts/train.py \
  --dataset.repo_id=${HF_USER}/ffw_test \
  --policy.type=act \
  --output_dir=outputs/train/act_ffw_test \
  --job_name=act_ffw_test \
  --policy.device=cuda \
  --log_freq=100 \
  --save_freq=1000
```

##### Key Training Parameters

| Parameter | Description |
|-----------|-------------|
| `--dataset.repo_id` | The Hugging Face dataset ID you created in the data collection step |
| `--policy.type` | Model architecture to use (e.g., 'act' for Action Chunking Transformer) |
| `--output_dir` | Where to save model checkpoints and logs |
| `--job_name` | Name of the training run (useful for tracking) |
| `--policy.device` | Device to use for training ('cuda' for GPU, 'cpu' for CPU) |
| `--log_freq` | How often to log training statistics (in iterations) |
| `--save_freq` | How often to save model checkpoints (in iterations) |

##### Expected Training Output

During training, you'll see output like this:

```
INFO 2025-05-28 12:12:40 ts/train.py:232 step:200 smpl:2K ep:3 epch:0.16 loss:7.490 grdn:154.502 lr:1.0e-05 updt_s:0.047 data_s:0.002
INFO 2025-05-28 12:12:48 ts/train.py:232 step:400 smpl:3K ep:7 epch:0.33 loss:3.128 grdn:85.109 lr:1.0e-05 updt_s:0.041 data_s:0.000
INFO 2025-05-28 12:12:57 ts/train.py:232 step:600 smpl:5K ep:10 epch:0.49 loss:2.615 grdn:74.954 lr:1.0e-05 updt_s:0.041 data_s:0.000
INFO 2025-05-28 12:13:05 ts/train.py:232 step:800 smpl:6K ep:13 epch:0.65 loss:2.331 grdn:68.764 lr:1.0e-05 updt_s:0.042 data_s:0.000
INFO 2025-05-28 12:13:14 ts/train.py:232 step:1K smpl:8K ep:16 epch:0.81 loss:2.075 grdn:64.323 lr:1.0e-05 updt_s:0.042 data_s:0.000
INFO 2025-05-28 12:13:22 ts/train.py:232 step:1K smpl:10K ep:20 epch:0.98 loss:1.903 grdn:61.364 lr:1.0e-05 updt_s:0.042 data_s:0.000
INFO 2025-05-28 12:13:31 ts/train.py:232 step:1K smpl:11K ep:23 epch:1.14 loss:1.716 grdn:57.887 lr:1.0e-05 updt_s:0.042 data_s:0.001
INFO 2025-05-28 12:13:40 ts/train.py:232 step:2K smpl:13K ep:26 epch:1.30 loss:1.558 grdn:54.819 lr:1.0e-05 updt_s:0.041 data_s:0.000
INFO 2025-05-28 12:13:48 ts/train.py:232 step:2K smpl:14K ep:29 epch:1.47 loss:1.454 grdn:53.859 lr:1.0e-05 updt_s:0.042 data_s:0.000
...
```

### Training on Your PC

#### 1. First, follow the [LeRobot installation instructions](https://github.com/ROBOTIS-GIT/lerobot) to set up the framework locally. 

#### 2. Transfer your dataset directory using scp:

```bash
scp -r /home/robotis/Desktop/ai_worker/docker/huggingface/lerobot/${HF_USER}/ffw_test/ <USER>@<IP>:/home/.cache/huggingface/lerobot/${HF_USER}/
```

#### 3. You can train the policy using the same command as above:

```bash
python lerobot/scripts/train.py \
  --dataset.repo_id=${HF_USER}/ffw_test \
  --policy.type=act \
  --output_dir=outputs/train/act_ffw_test \
  --job_name=act_ffw_test \
  --policy.device=cuda \
  --log_freq=100 \
  --save_freq=1000
```

Training time depends on your hardware and dataset size, but typically ranges from a few hours to a day.

#### (Optional) Uploading Checkpoints to Hugging Face

To upload the latest trained checkpoint to the Hugging Face Hub:

```bash
huggingface-cli upload ${HF_USER}/act_ffw_test \
  outputs/train/act_ffw_test/checkpoints/last/pretrained_model
```

This makes your model accessible from anywhere and simplifies deployment.

## Model Inference

Once your model is trained, you can deploy it on the AI Worker for inference.

### 1. Change file ownership (on the host machine, not inside the Docker container):
```bash
sudo chown -R robotis ./
```

### 2. Open a terminal and run Docker container:
```bash
cd ai_worker
./docker/container.sh enter
```

### 3. Launch the ROS 2 Follower inside the Docker container:
```bash
ffw_bg2_follower_ai
```

### 4. Open a new terminal and run Docker container:
```bash
cd ai_worker
./docker/container.sh enter
```

### 5. Navigate to the lerobot directory inside the Docker container:
```bash
cd /root/colcon_ws/src/physical_ai_tools/lerobot
```

### 6. Run the following command to evaluate your trained policy:
```bash
python lerobot/scripts/control_robot.py \
  --robot.type=ffw \
  --control.type=record \
  --control.single_task="pick and place objects" \
  --control.fps=15 \
  --control.repo_id=${HF_USER}/eval_ffw_test \
  --control.tags='["tutorial"]' \
  --control.episode_time_s=20 \
  --control.reset_time_s=10 \
  --control.num_episodes=10 \
  --control.push_to_hub=true \
  --control.policy.path=outputs/train/act_ffw_test/checkpoints/last/pretrained_model \
  --control.play_sounds=false
```

#### Key Inference Parameters

| Parameter | Description |
|-----------|-------------|
| `--control.type=record` | Records the policy performance for later evaluation |
| `--control.policy.path` | Path to your trained model checkpoint |
| `--control.episode_time_s` | Duration of each inference episode (in seconds) |
| `--control.repo_id` | Where to save evaluation results (different from training dataset) |

## Visualizing Inference Results

After running inference, you can visualize the results using the same visualization tool used for datasets:

```bash
python lerobot/scripts/visualize_dataset_html.py \
  --repo-id ${HF_USER}/eval_ffw_test
```

Then open [http://127.0.0.1:9090](http://127.0.0.1:9090) in your browser to see how your model performed.

## Troubleshooting

- **Out of memory errors**: Try reducing the batch size with `--train.batch_size=16` or lower
- **Low performance**: Consider collecting more diverse training data or increasing training duration
- **Robot not responding**: Ensure the follower node is running and communication is established
- **Training divergence**: Check your dataset quality and try decreasing the learning rate
