# Model Workflow

This guide explains the process of training and deploying imitation learning models for the AI Worker using your prepared datasets.

## Model Training

After [preparing your dataset](/dataset_preparation), you can train a policy using either your local PC or an NVIDIA Jetson AGX Orin device.

### Training on NVIDIA Jetson AGX Orin

Open a new terminal and navigate to the `lerobot` directory:

```bash
container
cd /root/colcon_ws/src/physical_ai_tools/lerobot
```

Then run the following command:

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

### Training on Your PC

First, follow the [LeRobot installation instructions](https://github.com/ROBOTIS-GIT/lerobot) to set up the framework locally. Once installed, you can train the policy using the same command as above.

### Key Training Parameters

| Parameter | Description |
|-----------|-------------|
| `--dataset.repo_id` | The Hugging Face dataset ID you created in the data collection step |
| `--policy.type` | Model architecture to use (e.g., 'act' for Action Chunking Transformer) |
| `--output_dir` | Where to save model checkpoints and logs |
| `--job_name` | Name of the training run (useful for tracking) |
| `--policy.device` | Device to use for training ('cuda' for GPU, 'cpu' for CPU) |
| `--log_freq` | How often to log training statistics (in iterations) |
| `--save_freq` | How often to save model checkpoints (in iterations) |

### Expected Training Output

During training, you'll see output similar to this:

```
[2025-05-16 13:45:22,885] epoch: 1/10, batch: 100/5000, loss: 0.18745
[2025-05-16 13:47:45,312] epoch: 1/10, batch: 200/5000, loss: 0.12481
[2025-05-16 13:50:12,769] epoch: 1/10, batch: 300/5000, loss: 0.09342
...
```

Training time depends on your hardware and dataset size, but typically ranges from a few hours to a day.

### (Optional) Uploading Checkpoints to Hugging Face

To upload the latest trained checkpoint to the Hugging Face Hub:

```bash
huggingface-cli upload ${HF_USER}/act_ffw_test \
  outputs/train/act_ffw_test/checkpoints/last/pretrained_model
```

This makes your model accessible from anywhere and simplifies deployment.

## Model Inference

Once your model is trained, you can deploy it on the AI Worker for inference.

### 1. Launch the ROS 2 Follower

First, launch the robot follower node:

```bash
container
follower
```

### 2. Run Model Inference

Open a new terminal and navigate to the lerobot directory:

```bash
container
cd /root/colcon_ws/src/physical_ai_tools/lerobot
```

Then run the following command to evaluate your trained policy:

```bash
python lerobot/scripts/control_robot.py \
  --robot.type=ffw \
  --control.type=record \
  --control.single_task="pick and place objects" \
  --control.fps=30 \
  --control.repo_id=${HF_USER}/eval_ffw_test \
  --control.tags='["tutorial"]' \
  --control.episode_time_s=20 \
  --control.reset_time_s=10 \
  --control.num_episodes=10 \
  --control.push_to_hub=true \
  --control.policy.path=outputs/train/act_ffw_test/checkpoints/last/pretrained_model \
  --control.play_sounds=false
```

### Key Inference Parameters

| Parameter | Description |
|-----------|-------------|
| `--control.type=record` | Records the policy performance for later evaluation |
| `--control.policy.path` | Path to your trained model checkpoint |
| `--control.episode_time_s` | Duration of each inference episode (in seconds) |
| `--control.repo_id` | Where to save evaluation results (different from training dataset) |

### Visualizing Inference Results

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
