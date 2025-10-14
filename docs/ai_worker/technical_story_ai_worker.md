# Technical Story

Hey there! Welcome to the AI Worker Technical Story section, where we share the AI technologies and techniques that bring our robot to life.

Building a capable robot isn't just about assembling hardware – it's about giving it the "intelligence" to perform real-world tasks effectively. AI Worker leverages cutting-edge AI techniques like Imitation Learning and Reinforcement Learning to learn from demonstrations and improve through experience.

Let's dive into the technical stories behind AI Worker.

## Stories

Click on the card below to explore the AI technologies we've implemented:

<div style="display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 20px; margin: 30px 0;">

<details style="grid-column: 1 / -1;">
<summary style="list-style: none; cursor: pointer;">
  <div style="border: 2px solid #e0e0e0; border-radius: 12px; padding: 20px; transition: all 0.3s; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: white; min-height: 200px; display: inline-flex; flex-direction: column; justify-content: center; align-items: center; text-align: center; width: calc(66.666% - 14px); vertical-align: top;">
    <img src="/technical_story/isaac_gr00t_header_compress.png" alt="Isaac GROOT" style="height: 80px; margin-bottom: 15px; filter: brightness(0) invert(1);">
    <h3 style="margin: 0 0 10px 0; border: none; color: white;">AI Worker x NVIDIA GR00T N1.5</h3>
    <div class="expand-indicator" style="margin-top: 15px; font-size: 12px; opacity: 0.8;">▼ Click to expand</div>
  </div>
</summary>
<div style="padding: 30px; border: 2px solid #667eea; border-radius: 12px; margin-top: 20px; background: #1b1b1f; color: #e0e0e0;">

### ▶️ Full Demo

<div style="position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden; max-width: 100%; margin: 20px 0; border-radius: 8px;">
  <iframe 
    style="position: absolute; top: 0; left: 0; width: 100%; height: 100%;" 
    src="https://www.youtube.com/embed/h2O57D7KGYg" 
    title="AI Worker x NVIDIA Gr00t N1.5 Demo" 
    frameborder="0" 
    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" 
    allowfullscreen>
  </iframe>
</div>

---

### Overview

We aimed to develop an autonomous robotic system capable of sorting convenience store items, with a specific focus on coffee bottle classification as our primary task scenario. The robot is presented with mixed items placed in a central area and must sort them into designated boxes positioned on the left and right sides.

To achieve this goal, we leveraged **imitation learning** as our core methodology, utilizing NVIDIA's **Isaac GR00T N1.5** foundation model. We collected real-world demonstration data directly in actual operational environments, ensuring our model could handle the complexities and variations inherent in real-world scenarios.

<div style="margin: 20px 0; padding: 20px; background: rgba(102, 126, 234, 0.1); border-left: 4px solid #667eea; border-radius: 4px;">

**🎯 Public Demonstrations**

We successfully showcased this system at two major robotics conferences:
- **CoRL (Conference on Robot Learning)** 
- **Humanoid Conference**

</div>

<div style="margin: 20px 0; padding: 20px; background: rgba(102, 126, 234, 0.1); border-left: 4px solid #667eea; border-radius: 4px;">

**📊 Performance Metrics**

Through rigorous testing over 100 trials, our system achieved approximately **70% success rate**, demonstrating robust performance while also revealing areas for future improvement.

</div>

<div style="margin: 20px 0; padding: 20px; background: rgba(102, 126, 234, 0.1); border-left: 4px solid #667eea; border-radius: 4px;">

**💡 Project Impact**

This project goes beyond simply using a foundation model – we've built a complete system infrastructure based on **AI Worker** and **Physical AI Tools** that enables the deployment and operation of GR00T N1.5 on real robotic hardware.

</div>

---

### Model Architecture

NVIDIA's Isaac Gr00t N1.5 is a vision-language-action (VLA) foundation model designed for robotic manipulation tasks. The model architecture consists of two main systems working in tandem to translate multimodal inputs into precise robot actions.

<div style="text-align: center; margin: 30px 0;">
  <img src="/technical_story/isaac_gr00t_architecture.png" alt="Isaac Gr00t N1.5 Architecture" style="max-width: 100%; border-radius: 8px; border: 2px solid #667eea;">
  <p style="margin-top: 10px; font-size: 14px; color: #888; font-style: italic;">Figure: Isaac Gr00t N1.5 Architecture Overview</p>
</div>

#### Input Modalities

The model accepts three types of input data:

- **Image Observation**: Visual input from the robot's cameras
- **Language Instruction**: Natural language commands describing the desired task
- **Robot State**: Current robot configuration including joint positions and end-effector poses

#### System 2: Vision-Language Model (VLM)

The Vision-Language Model processes visual and linguistic information to understand the task context and scene semantics. This high-level understanding guides the action generation process.

#### System 1: Diffusion Transformer

The Diffusion Transformer takes the VLM output along with robot state information and generates action chunks - sequences of future robot actions.

This two-system architecture enables the model to leverage both high-level semantic understanding and low-level motor control for robust robotic manipulation.

---

### Data Collection

We collected demonstration data to train the model for real-world robustness.

- **Dataset Size**: 10 hours of demonstration data across 800 episodes
- **Task Scope**: Sorting four different coffee bottles into their designated bins
- **Collection Strategy**: Intentionally varied lighting conditions and backgrounds during data collection
- **Environment Diversity**: Multiple lighting scenarios and background variations to ensure robust generalization

**Key Insight**: By training with diverse environmental conditions from the start, we built a system capable of handling the intense and unpredictable lighting of exhibition halls without requiring additional adaptation.

---

### Training

The model was fine-tuned using NVIDIA's powerful GPU infrastructure.

- **Hardware Setup**: 8x NVIDIA B200 GPUs
- **Training Duration**: Approximately 20 hours for complete fine-tuning
- **Base Model**: NVIDIA Isaac GR00T N1.5 foundation model
- **Training Approach**: Fine-tuning pre-trained weights on task-specific demonstration data

**Note**: The relatively short training time (20 hours) demonstrates the efficiency of foundation model fine-tuning compared to training from scratch, which would typically require significantly more time and data.

---

### Technical Challenges & Solutions

<details>
<summary style="cursor: pointer; padding: 15px; background: rgba(102, 126, 234, 0.1); border-left: 4px solid #667eea; border-radius: 4px; margin: 10px 0; font-weight: 600;">
💡 Challenge 1: Real-World Robustness
</summary>
<div style="padding: 15px 20px; margin: 0 0 10px 0; background: rgba(102, 126, 234, 0.05); border-radius: 0 0 4px 4px;">

**Problem**: Laboratory-trained models often fail in real-world environments due to lighting variations, background changes, and unpredictable conditions.

**Solution**: We intentionally introduced environmental diversity during data collection by varying lighting conditions and backgrounds. This "train for the real world" approach ensured the model could generalize to challenging scenarios like exhibition halls with intense and unpredictable lighting.

</div>
</details>

<details>
<summary style="cursor: pointer; padding: 15px; background: rgba(102, 126, 234, 0.1); border-left: 4px solid #667eea; border-radius: 4px; margin: 10px 0; font-weight: 600;">
🎪 Challenge 2: Exhibition Hall Deployment
</summary>
<div style="padding: 15px 20px; margin: 0 0 10px 0; background: rgba(102, 126, 234, 0.05); border-radius: 0 0 4px 4px;">

**Problem**: Trade show floors present some of the most challenging conditions for robotic systems - constantly changing lighting, crowds, and visual distractions.

**Solution**: Anticipating exhibition deployment, we intentionally varied backgrounds and environments during data collection. To simulate crowded conditions, we positioned the robot in high-traffic areas during data acquisition. This proactive approach enabled the system to handle the exhibition environment without requiring any additional fine-tuning or adaptation.

</div>
</details>

<details>
<summary style="cursor: pointer; padding: 15px; background: rgba(102, 126, 234, 0.1); border-left: 4px solid #667eea; border-radius: 4px; margin: 10px 0; font-weight: 600;">
📊 Challenge 3: Data Efficiency
</summary>
<div style="padding: 15px 20px; margin: 0 0 10px 0; background: rgba(102, 126, 234, 0.05); border-radius: 0 0 4px 4px;">

**Problem**: Collecting large-scale demonstration data can be time-consuming and expensive.

**Solution**: By leveraging the NVIDIA Isaac GR00T N1.5 foundation model, we achieved strong performance with just 10 hours (800 episodes) of task-specific data, significantly reducing the data requirements compared to training from scratch.

</div>
</details>

---

### Deployment

Our deployment architecture is designed for modularity, scalability, and isolation using containerized environments. The system integrates multiple components across different machines, leveraging ROS2 for inter-process communication and Docker for dependency management.

<div style="text-align: center; margin: 30px 0;">
  <img src="/technical_story/isaac_gr00t_system_architecture.png" alt="Deployment Architecture" style="max-width: 100%; border-radius: 8px; border: 2px solid #667eea;">
  <p style="margin-top: 10px; font-size: 14px; color: #888; font-style: italic;">Figure: System Deployment Architecture</p>
</div>

#### System Architecture Overview

Our deployment consists of two primary machines communicating via ROS2 and ZMQ protocols:

**ORIN (Robot Controller)**
- **AI Worker Container**: Manages robot control and executes motor commands
- **Physical AI Tools Container**: Handles inference orchestration and UI configuration
- **Communication**: Both containers communicate via ROS2 topics

**5090 Workstation (Inference Server)**
- **GR00T N1.5 Container**: Runs the foundation model for action prediction
- **Communication**: Connects to Physical AI Tools via ZMQ socket

#### Component Details

<div style="margin: 20px 0; padding: 20px; background: rgba(102, 126, 234, 0.1); border-left: 4px solid #667eea; border-radius: 4px;">

**🤖 AI Worker (Docker Container)**
- Built on ROS2 framework for robot control
- Subscribes to action chunk topics from Physical AI Tools
- Executes motor commands to control the robot hardware
- Handles low-level robot state management and safety protocols

</div>

<div style="margin: 20px 0; padding: 20px; background: rgba(102, 126, 234, 0.1); border-left: 4px solid #667eea; border-radius: 4px;">

**🛠️ Physical AI Tools (Docker Container)**
- Provides UI for inference configuration and monitoring
- Orchestrates the inference pipeline
- Publishes action chunk ROS2 topics to AI Worker
- Communicates with GR00T N1.5 model via ZMQ socket
- Manages observation data flow and action prediction requests

</div>

<div style="margin: 20px 0; padding: 20px; background: rgba(102, 126, 234, 0.1); border-left: 4px solid #667eea; border-radius: 4px;">

**🧠 GR00T N1.5 (Docker Container)**
- Runs NVIDIA Isaac GR00T N1.5 foundation model
- Deployed using NVIDIA's official Docker image
- Receives observations via ZMQ from Physical AI Tools
- Returns predicted action chunks for robot execution
- Leverages GPU acceleration for real-time inference

</div>

#### Communication Flow

1. **Observation Collection**: AI Worker collects sensor data (camera images, robot state) and publishes to ROS2 topics
2. **Data Forwarding**: Physical AI Tools subscribes to observation topics and forwards data to GR00T N1.5 via ZMQ
3. **Action Prediction**: GR00T N1.5 processes observations and returns action chunks via ZMQ
4. **Action Execution**: Physical AI Tools publishes action chunks to ROS2 topics
5. **Robot Control**: AI Worker subscribes to action topics and executes motor commands

#### Design Benefits

- **Dependency Isolation**: Docker containers prevent dependency conflicts between different components
- **Modularity**: Each component can be developed, tested, and deployed independently
- **Scalability**: Inference server can be scaled separately from robot controller
- **Cross-Machine Communication**: ZMQ socket enables efficient communication between ORIN and workstation, while ROS2 handles communication between containers on ORIN
- **Hardware Flexibility**: High-compute inference tasks run on dedicated GPU workstation while robot control runs on embedded ORIN platform

---

### Experimental Results

We conducted extensive testing to evaluate the system's performance in real-world scenarios, focusing on both quantitative metrics and qualitative failure analysis.

#### System Performance Metrics

<div style="margin: 20px 0; padding: 20px; background: rgba(102, 126, 234, 0.1); border-left: 4px solid #667eea; border-radius: 4px;">

**⚡ Inference Performance**
- **Inference Latency**: 40ms on NVIDIA RTX 5090 GPU
- **Control Frequency**: 10 FPS robot control loop
- **Total System Latency**: End-to-end observation to action execution

</div>

<div style="margin: 20px 0; padding: 20px; background: rgba(102, 126, 234, 0.1); border-left: 4px solid #667eea; border-radius: 4px;">

**🎯 Task Success Rate**
- **Test Trials**: 100 sorting attempts
- **Success Rate**: ~70%
- **Test Environment**: Real exhibition hall conditions with varied lighting and backgrounds

</div>

#### Control Optimization Techniques

To achieve smooth and natural robot motion, we implemented several optimization strategies:

1. **Dynamixel Motor Tuning**: Fine-tuned velocity and acceleration profiles to ensure fluid movements without jerking
2. **Asynchronous Inference**: Implemented non-blocking inference pipeline to maintain consistent control frequency
3. **Action Smoothing**: Applied temporal smoothing to action chunks for coordinated multi-joint movements

#### Failure Cases

Understanding failure cases is critical for future improvements. We identified three primary failure patterns in the 30% unsuccessful trials:

- **Color Misclassification (~50% of failures)**: Correctly grasped but placed in the wrong bin due to color recognition errors
- **Grasping Failures (~30% of failures)**: Robot failed to successfully grasp the target object
- **Phantom Actions (~20% of failures)**: Attempted placement without having grasped an object

---

### Lessons Learned & Future Work

This project provided valuable insights into deploying foundation models on real robotic systems. Here are our key takeaways and directions for future development.

#### What Worked Well ✅

<div style="margin: 20px 0; padding: 20px; background: rgba(102, 126, 234, 0.1); border-left: 4px solid #667eea; border-radius: 4px;">

**Foundation Model Efficiency**
- Fine-tuning approach significantly reduced data requirements.
- 20-hour training time on 8x B200 GPUs demonstrated excellent time-to-deployment.

**Environmental Diversity Strategy**
- Collecting data in varied lighting and crowded areas proved essential for real-world robustness
- "Train for the real world" approach eliminated the need for domain adaptation

**Modular System Architecture**
- Docker containerization prevented dependency conflicts and simplified deployment
- ROS2 + ZMQ communication enabled flexible cross-machine deployment

</div>

#### Challenges & Learning Points 🎓

<div style="margin: 20px 0; padding: 20px; background: rgba(245, 87, 108, 0.1); border-left: 4px solid #f5576c; border-radius: 4px;">

**70% Success Rate**
- While acceptable for demonstrations, production deployment would require higher reliability
- Color misclassification remains the primary failure cause, indicating need for improved visual perception

**Data Collection Overhead**
- 10 hours of teleoperation data collection required significant manual effort
- Need for more efficient data collection methods or synthetic data augmentation

**Grasp Stability**
- Depth perception and grasp planning could benefit from additional sensors or algorithms

</div>

#### Future Directions 🚀

1. **Enhanced Visual Perception**: Integrate better color recognition under varied lighting, possibly with additional sensors
2. **Grasp Detection**: Add proprioceptive feedback to detect successful grasps before attempting placement
3. **More Training Data**: Collect additional edge cases to improve success rate to 85-90%

---

### References & Resources

**Related Projects & Tools**
- [NVIDIA Isaac GR00T](https://github.com/NVIDIA/Isaac-GR00T) - Official NVIDIA Isaac GR00T repository
- [Physical AI Tools](https://github.com/ROBOTIS-GIT/physical_ai_tools) - Our open-source tools for robotic AI deployment
- [AI Worker](https://github.com/ROBOTIS-GIT/aiworker) - AI Worker robot platform

</div>
</details>

</div>

<style>
details > summary::-webkit-details-marker {
  display: none;
}
details > summary {
  list-style: none;
}
details > summary > div:hover {
  transform: translateY(-5px);
  box-shadow: 0 10px 25px rgba(0,0,0,0.2);
}
details[open] > summary > div {
  border-radius: 12px;
}
details[open] > summary .expand-indicator {
  display: none;
}
</style>

---

