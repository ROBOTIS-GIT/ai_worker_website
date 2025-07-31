# LeRobot for OMX

This page provides information about using LeRobot with the OMX system for imitation learning.

## Overview

LeRobot is an open-source library for robotics that provides tools for imitation learning, reinforcement learning, and robot control. It can be used with OMX for training and deploying AI models.

## Installation

### Prerequisites

Before installing LeRobot, ensure you have:

- Python 3.8 or higher
- ROS 2 Humble or newer
- OMX system properly configured
- CUDA-compatible GPU (recommended for training)

### Install LeRobot

```bash
# Install LeRobot
pip install lerobot

# Install additional dependencies
pip install torch torchvision torchaudio
pip install transformers datasets
```

## Configuration

### Environment Setup

1. Set up your environment variables:
```bash
export OMX_MODEL_PATH="/path/to/your/models"
export OMX_DATA_PATH="/path/to/your/datasets"
```

2. Configure LeRobot for OMX:
```bash
# Create configuration file
mkdir -p ~/.lerobot
touch ~/.lerobot/omx_config.yaml
```

### Configuration File

Create `~/.lerobot/omx_config.yaml`:

```yaml
robot:
  name: "omx"
  type: "manipulator"
  
hardware:
  joints: 6
  end_effector: true
  camera: true
  
training:
  batch_size: 32
  learning_rate: 1e-4
  epochs: 100
  
inference:
  model_path: "/path/to/trained/model"
  device: "cuda"  # or "cpu"
```

## Usage

### Dataset Preparation

1. **Record demonstrations**:
```bash
lerobot record --robot omx --output /path/to/dataset
```

2. **Process recordings**:
```bash
lerobot process --input /path/to/dataset --output /path/to/processed
```

### Model Training

1. **Train imitation learning model**:
```bash
lerobot train --config ~/.lerobot/omx_config.yaml --data /path/to/processed
```

2. **Monitor training**:
```bash
lerobot monitor --experiment_name omx_training
```

### Model Inference

1. **Deploy trained model**:
```bash
lerobot deploy --model /path/to/trained/model --robot omx
```

2. **Run inference**:
```bash
lerobot run --model /path/to/trained/model --task pick_and_place
```

## Examples

### Basic Pick and Place

```python
import lerobot
from lerobot.robots import OMX

# Initialize robot
robot = OMX()

# Load trained model
model = lerobot.load_model("/path/to/model")

# Execute task
robot.execute_task(model, "pick_and_place")
```

### Custom Training Loop

```python
import lerobot
from lerobot.training import ImitationTrainer

# Initialize trainer
trainer = ImitationTrainer(
    robot_type="omx",
    model_type="transformer",
    config_path="~/.lerobot/omx_config.yaml"
)

# Train model
trainer.train(
    data_path="/path/to/dataset",
    output_path="/path/to/output",
    epochs=100
)
```

## Troubleshooting

### Common Issues

1. **CUDA out of memory**:
   - Reduce batch size in configuration
   - Use gradient accumulation
   - Use mixed precision training

2. **Robot connection issues**:
   - Check ROS 2 network configuration
   - Verify robot IP address
   - Ensure proper permissions

3. **Training convergence**:
   - Adjust learning rate
   - Increase dataset size
   - Modify model architecture

### Debug Mode

Enable debug logging:

```bash
export LEROBOT_LOG_LEVEL=DEBUG
lerobot train --config ~/.lerobot/omx_config.yaml
```

## Performance Optimization

### Training Optimization

- Use mixed precision training
- Implement gradient accumulation
- Use distributed training for large datasets
- Optimize data loading with multiple workers

### Inference Optimization

- Use TensorRT for faster inference
- Implement model quantization
- Use batch inference when possible
- Optimize model architecture

## Integration with OMX

### ROS 2 Integration

LeRobot integrates with OMX through ROS 2:

```python
import rclpy
from lerobot.robots.ros2 import OMXROS2

# Initialize ROS 2 node
rclpy.init()
robot = OMXROS2()

# Use with LeRobot
model = lerobot.load_model("/path/to/model")
robot.execute_task(model, "custom_task")
```

### Custom Tasks

Define custom tasks for OMX:

```python
from lerobot.tasks import Task

class OMXCustomTask(Task):
    def __init__(self):
        super().__init__("omx_custom")
    
    def execute(self, robot, model):
        # Custom task implementation
        pass
```

## Resources

### Documentation

- [LeRobot Documentation](https://lerobot.github.io/)
- [OMX Documentation](/omx/introduction_omx)
- [ROS 2 Documentation](https://docs.ros.org/)

### Community

- [LeRobot GitHub](https://github.com/huggingface/lerobot)
- [ROBOTIS Forum](https://forum.robotis.com/)
- [ROS 2 Community](https://discourse.ros.org/)

### Examples Repository

- [LeRobot Examples](https://github.com/huggingface/lerobot/tree/main/examples)
- [OMX Examples](https://github.com/ROBOTIS-GIT/ai_worker)

## Next Steps

After setting up LeRobot with OMX:

1. Review [Model Training](/omx/model_training_omx)
2. Check [Model Inference](/omx/model_inference_omx)
3. Explore [Simulation](/omx/simulation_omx)

## Support

For issues with LeRobot and OMX:

- Check the [FAQ](/omx/faq_omx)
- Visit the [Community Forum](https://forum.robotis.com/)
- Contact [Support](/omx/contact_omx) 