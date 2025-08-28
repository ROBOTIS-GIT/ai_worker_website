# Setup Guide — LeRobot (OMX)

## Overview

LeRobot is an open-source robotics framework with utilities for imitation learning. With OMX, you can train control policies and deploy them to the robot.

## Install LeRobot

Prerequisite: Install Miniconda first. See the [Miniconda Quick command line install](https://www.anaconda.com/docs/getting-started/miniconda/install#quick-command-line-install).

#### 1) Create a Virtual Environment (Miniconda, Python 3.10)
```bash
conda create -y -n lerobot python=3.10
```

#### 2) Activate the Environment
```bash
conda activate lerobot
```

#### 3) Install FFmpeg
```bash
conda install -c conda-forge ffmpeg=6.1.1 -y
```

::::: tip FFmpeg Installation Options
This typically installs FFmpeg 7.X on your system with the libsvtav1 encoder enabled. If libsvtav1 isn't available (check with `ffmpeg -encoders`), use one of the options below:

**[On any platform]** Explicitly install FFmpeg 7.X using:
```bash
conda install ffmpeg=7.1.1 -c conda-forge
```

**[On Linux only]** To build FFmpeg yourself: install the build dependencies and compile it with libsvtav1, then ensure your shell uses that binary (verify with `which ffmpeg`).
:::::

#### 4) Clone Source and Enter Directory
```bash
git clone https://github.com/ROBOTIS-GIT/lerobot.git
cd lerobot
```

#### 5) Install from Source (editable)
```bash
pip install -e .
```

#### 6) Install Core Library from PyPI (optional)
```bash
pip install lerobot
```

#### 7) Install Dynamixel SDK extras
```bash
pip install -e ".[dynamixel]"
```

### Troubleshooting

If you encounter build errors, you may need additional packages such as cmake, build-essential, and FFmpeg libraries. On Linux, install them with:

```bash
sudo apt-get install cmake build-essential python-dev pkg-config libavformat-dev libavcodec-dev libavdevice-dev libavutil-dev libswscale-dev libswresample-dev libavfilter-dev pkg-config
```

For other platforms, see: [Compiling PyAV](https://pyav.org/docs/stable/overview/installation.html)

🎉 Lerobot Setup Complete!

Click the button below to start Imitation Learning.
