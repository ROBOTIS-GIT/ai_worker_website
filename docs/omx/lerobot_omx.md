# LeRobot for OMX

## Overview

LeRobot is an open-source framework for robotics that provides tools for imitation learning. It can be used with OMX for training and deploying AI models.

## Installation LeRobot

#### 1) Create Virtual Environment (Miniconda, Python 3.10)
```bash
conda create -y -n lerobot python=3.10
```

#### 2) Activate Environment
```bash
conda activate lerobot
```

#### 3) Install FFmpeg
```bash
conda install ffmpeg -c conda-forge
```

::: tip FFmpeg Installation Options
This usually installs FFmpeg 7.X for your platform compiled with the libsvtav1 encoder. If libsvtav1 is not supported (check supported encoders with `ffmpeg -encoders`), you can:

**[On any platform]** Explicitly install FFmpeg 7.X using:
```bash
conda install ffmpeg=7.1.1 -c conda-forge
```

**[On Linux only]** If you want to bring your own FFmpeg: Install FFmpeg build dependencies and compile FFmpeg from source with libsvtav1, and make sure you use the corresponding FFmpeg binary to your install with `which ffmpeg`.
:::

#### 4) Clone Source (feature-omx-devel) and Enter Directory
```bash
git clone -b feature-omx-devel https://github.com/ROBOTIS-GIT/lerobot.git
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

#### 8) Find bus servo adapter ports
```bash
lerobot-find-port
```

### Troubleshooting

If you encounter build errors, you may need to install additional dependencies: cmake, build-essential, and FFmpeg libs. To install these for Linux run:

```bash
sudo apt-get install cmake build-essential python-dev pkg-config libavformat-dev libavcodec-dev libavdevice-dev libavutil-dev libswscale-dev libswresample-dev libavfilter-dev pkg-config
```

For other systems, see: [Compiling PyAV](https://pyav.org/docs/stable/overview/installation.html)