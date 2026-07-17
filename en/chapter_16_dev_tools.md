# Ch.16 — Development Environment & Tools

Robotics research code is sensitive to combinations of CUDA, Python, ROS, and system-library versions. This chapter covers language tools, package environments, and Docker for isolating and reproducing those environments.

## 16.1 Programming Languages

Even when AI coding agents assist with much of the writing, researchers still need to read and understand existing code. They must inspect the structure of cloned research code, verify generated code, and identify where to make corrections when something goes wrong.

### 16.1.1 C++

**Use cases**: real-time systems, ROS nodes, SLAM, performance-critical modules

Most of the core code in a lab is C++. SLAM, real-time control, and the core logic of ROS packages are all written in C++, and you often have to read and modify this code. To understand code like ORB-SLAM3, LOAM, or VINS-Mono, you need to be comfortable with C++.

**Pros**:
- Fast execution
- Direct memory control
- Most ROS/SLAM code is C++

**Cons**:
- Hard to learn
- Slow development
- Memory management mistakes

**modern C++ (C++17/20)**:

```cpp
// Smart pointer
auto ptr = std::make_shared<MyClass>();

// Range-based for
for (const auto& item : container) { ... }

// Lambda
auto func = [&](int x) { return x * 2; };
```

> **Further reading**
> - [C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines) — modern C++ coding guide (Bjarne Stroustrup, Herb Sutter)
> - [The Cherno - C++ Playlist](https://www.youtube.com/playlist?list=PLlrATfBNZ98dudnM48yfGUldqGD0S4FFb) — video series covering C++ from basics to advanced topics
> - [Modernes C++](https://www.modernescpp.com/index.php) — blog with a systematic treatment of modern C++ (C++17/20/23) features

> **⚠ Target-environment check**: State whether the target is x86 or Jetson (ARM) and whether cross-compilation is involved. Then verify that the generated dependencies and build flags support that architecture.

### 16.1.2 Python

**Use cases**: prototyping, deep-learning training/inference, data analysis, visualization

Python is widely used for PyTorch training scripts, data preprocessing, and related tasks. An agent can help produce a first draft, but the researcher still needs to read the code and inspect its outputs and performance bottlenecks.

**Frequently used libraries**:

```bash
pip install numpy scipy matplotlib
pip install opencv-python open3d
pip install torch torchvision
pip install transformers  # HuggingFace
```

> **Further reading**
> - [Real Python](https://realpython.com/) — systematic tutorials covering Python from basics to advanced
> - [Fireship - Python in 100 Seconds](https://www.youtube.com/watch?v=x7X9w_GIm1s) — video that skims all of Python quickly

## 16.2 Development Environment Setup

### 16.2.1 Ubuntu

Ubuntu is the primary supported platform for ROS, and it has extensive documentation for GPU driver, CUDA, and cuDNN combinations. Some development is also possible on macOS and Windows; the appropriate choice depends on the ROS distribution and the operating system deployed on the robot.

**Recommended versions**:
- Ubuntu 22.04 LTS (ROS2 Humble)
- Ubuntu 24.04 LTS (ROS2 Jazzy)

**Initial setup**:

```bash
# Basic tools
sudo apt update && sudo apt upgrade -y
sudo apt install -y build-essential cmake git curl wget

# Python-related
sudo apt install -y python3-pip python3-venv

# Development tools
sudo apt install -y vim tmux htop
```

> **Further reading**
> - [The Missing Semester of Your CS Education (MIT)](https://missing.csail.mit.edu/) — systematic treatment of shell, vim, tmux, Git, and other "development tools you use every day but no class teaches". Recommended
> - [Fireship - Linux in 100 Seconds](https://www.youtube.com/watch?v=rrB13utjYV4) — quickly get a feel for what Linux is

### 16.2.2 CUDA / cuDNN

Deep-learning models are usually trained with GPU acceleration. In an NVIDIA environment, an incompatible combination of CUDA and PyTorch can fail at the first `import torch`.

**Installation check**:

```bash
nvidia-smi          # GPU status
nvcc --version      # CUDA version
```

**Recommended versions**: CUDA 12.x, cuDNN 8.x

**Caveat**: always verify compatibility between the CUDA version and your PyTorch/TensorFlow version.

> **Further reading**
> - [PyTorch - Previous Versions](https://pytorch.org/get-started/previous-versions/) — check PyTorch-CUDA version matching. Consult before installing
> - [NVIDIA CUDA Toolkit Documentation](https://docs.nvidia.com/cuda/) — official CUDA documentation

**NVIDIA driver install troubleshooting**

When installing NVIDIA drivers on Ubuntu, the most common issue is a conflict with `nouveau` (the open-source driver).

```bash
# Disable nouveau
sudo bash -c "echo blacklist nouveau > /etc/modprobe.d/blacklist-nvidia-nouveau.conf"
sudo bash -c "echo options nouveau modeset=0 >> /etc/modprobe.d/blacklist-nvidia-nouveau.conf"
sudo update-initramfs -u
sudo reboot

# Install the driver (recommended: use apt)
sudo apt install nvidia-driver-535  # adjust the version to your GPU
sudo reboot

# Verify
nvidia-smi
```

If the GPU does not show up in `nvidia-smi` after installation: (1) check whether Secure Boot is on (turn it off in the BIOS); (2) check driver module status with `sudo dkms status`. (Reference: [Jinyong Jeong's blog](https://jinyongjeong.github.io/2016/11/22/ubuntu_graphic_driver_install/))

**CUDA/cuDNN version compatibility**

If PyTorch and the CUDA version do not match, a single `import torch` errors out. Check in this order:

```bash
# 1. Check the GPU
nvidia-smi  # The "CUDA Version" in the top right is the "max version supported by the driver"

# 2. Check the installed CUDA toolkit
nvcc --version

# 3. Check which CUDA PyTorch is using
python -c "import torch; print(torch.version.cuda)"

# All three versions must be compatible. Check combinations on the PyTorch official site:
# https://pytorch.org/get-started/locally/
```

(Reference: [Jinyong Jeong's blog](https://jinyongjeong.github.io/2016/09/19/cuda_setting/))

**Building OpenCV + CUDA from source**

`python3-opencv` installed via apt and `pip install opencv-python` do not have CUDA acceleration. If you need GPU acceleration (DNN module, optical flow, etc.), you have to build from source.

```bash
# Install dependencies
sudo apt install -y cmake git libgtk2.0-dev pkg-config \
    libavcodec-dev libavformat-dev libswscale-dev \
    libtbb-dev libjpeg-dev libpng-dev

# OpenCV + contrib source
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git

# Build (enable CUDA)
cd opencv && mkdir build && cd build
cmake -D CMAKE_BUILD_TYPE=Release \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
      -D WITH_CUDA=ON \
      -D CUDA_ARCH_BIN="8.6"  \  # adjust to your GPU (RTX 3090=8.6, RTX 4090=8.9)
      -D WITH_CUDNN=ON \
      -D OPENCV_DNN_CUDA=ON \
      -D BUILD_opencv_python3=ON \
      ..
make -j$(nproc)
sudo make install
```

`CUDA_ARCH_BIN` must match your GPU. If it is wrong, the build succeeds but you get slow runtime or errors. Check [NVIDIA GPU Compute Capability](https://developer.nvidia.com/cuda-gpus).

Caveat: pip opencv and apt cv_bridge conflict in ROS environments (see Ch.19, AI agent troubleshooting). Building CUDA OpenCV directly can make this problem more tangled, so isolating it with Docker is recommended.

(Reference: [Dark Programmer — building OpenCV + CUDA from source](https://darkpgmr.tistory.com/184))

### 16.2.3 Environment Management

Each project needs different Python and library versions. Without an environment manager, running `pip install` globally puts you in "dependency hell", where a library required by project A conflicts with project B. Creating an isolated per-project environment with Conda or venv is the baseline.

**Conda** (recommended):

```bash
# Install Miniconda
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh

# Create an environment
conda create -n myenv python=3.10
conda activate myenv

# Install packages
conda install pytorch torchvision pytorch-cuda=12.1 -c pytorch -c nvidia
```

**venv** (lightweight):

```bash
python3 -m venv myenv
source myenv/bin/activate
pip install -r requirements.txt
```

For example, SLAM research might need Python 3.8 while a recent Transformer model might need Python 3.10. With Conda, `conda activate slam_env` or `conda activate transformer_env` switches between them.

> **Further reading**
> - [Conda Documentation - Managing Environments](https://docs.conda.io/projects/conda/en/latest/user-guide/tasks/manage-environments.html) — official Conda environment management guide
> - [Python venv Documentation](https://docs.python.org/3/library/venv.html) — official Python virtual environment docs

## 16.3 Docker

### 16.3.1 Why Docker?

Docker packages the operating-system user space, libraries, and environment settings into an image. Sharing the same image reduces dependency differences across machines and lets a paper distribute its execution environment with the code.

- **Reproducibility**: pins the user-space image and reduces environment drift; host kernel, driver, hardware, and external services still matter
- **Isolation**: avoids polluting the system
- **Deployment**: easy sharing and deployment
- **Dependencies**: handles complex dependencies

### 16.3.2 Basic Usage

```bash
# Pull an image
docker pull nvidia/cuda:12.1.0-devel-ubuntu22.04

# Run a container
docker run -it --rm \
    --gpus all \
    -v $(pwd):/workspace \
    nvidia/cuda:12.1.0-devel-ubuntu22.04 bash

# Build a Dockerfile
docker build -t my_image .
```

**Dockerfile example**:

```dockerfile
FROM nvidia/cuda:12.1.0-devel-ubuntu22.04

RUN apt-get update && apt-get install -y \
    python3-pip git

COPY requirements.txt /tmp/
RUN pip3 install -r /tmp/requirements.txt

WORKDIR /workspace
```

> **Further reading**
> - [Docker official Getting Started Guide](https://docs.docker.com/get-started/) — start here if Docker is new to you. Explains containers, images, and volumes well
> - [NetworkChuck - Docker Tutorial](https://www.youtube.com/watch?v=eGz9DS-aIeY) — video that explains Docker in a fun, accessible way. Good for beginners
> - [Fireship - Docker in 100 Seconds](https://www.youtube.com/watch?v=Gjnup-PuquQ) — quick skim of Docker's core concepts

### 16.3.3 NVIDIA Container Toolkit

A plain Docker container does not see the GPU. Deep-learning training and CUDA-based computation require nvidia-container-toolkit and the `--gpus all` flag.

Note: NVIDIA's current documentation uses `nvidia-container-toolkit`; with modern Docker, GPU access is requested with `--gpus all` rather than the older `--runtime=nvidia` pattern.

```bash
# Install nvidia-container-toolkit (Ubuntu 22.04/24.04)
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
    sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit

# Register the NVIDIA runtime with the Docker daemon
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# Test
docker run --rm --gpus all nvidia/cuda:12.4.1-base-ubuntu22.04 nvidia-smi
```

### 16.3.4 Practical Recipe: ROS2 + GPU + GUI + Sensors

A robotics container may need GPU, GUI (RViz/Gazebo), and USB-sensor access together. The recipe below configures these permissions and the runtime in one place.

The recipe adapts the structure of [turlucode/ros-docker-gui](https://github.com/turlucode/ros-docker-gui) to an nvidia-container-toolkit and ROS2 Humble environment.

**Step 1: Prepare X11 forwarding (host)**

```bash
# Run once on the host
sudo apt-get install -y xauth
xhost +local:docker
```

`xhost +local:docker` allows X server access only from Docker containers. Safer than `xhost +` (allow all), but in security-sensitive environments `xauth`-based authentication is the right choice.

**Step 2: Run script**

```bash
#!/bin/bash
# run_ros2_docker.sh — full setup for GPU + GUI + USB sensors

docker run --rm -it \
    --gpus all \
    --privileged \
    --net=host \
    --ipc=host \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e ROS_DOMAIN_ID=42 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $HOME/.Xauthority:/root/.Xauthority:ro \
    -v /dev:/dev \
    -v $HOME/catkin_ws:/root/catkin_ws \
    --name ros2_dev \
    osrf/ros:humble-desktop \
    bash
```

What each flag does:

| Flag | Role |
|--------|------|
| `--gpus all` | GPU passthrough (nvidia-container-toolkit) |
| `--privileged` | Full access to USB/serial devices. In production, map individually with `--device` |
| `--net=host` | Share host network for DDS multicast. Essential for ROS2 inter-node communication |
| `--ipc=host` | Shared memory. Required by GUI tools like RViz |
| `-e QT_X11_NO_MITSHM=1` | Without this, RViz crashes with a segfault. MIT-SHM does not work in Docker |
| `-e ROS_DOMAIN_ID=42` | Isolates from other ROS2 systems on the same network. Essential when multiple people use the lab |
| `-v /dev:/dev` | Sensor USB may be plugged in at any time, so mount all of /dev. Pairs with `--privileged` |
| `-v .Xauthority` | X11 authentication. Without it you get `cannot open display` |

**Step 3: Save the container after work**

```bash
# If you installed packages or did other work inside the container, commit it
docker commit ros2_dev my_ros2_workspace:v1

# Next time, run from the saved image
# In run_ros2_docker.sh just change the image name
```

**Managing it with a Dockerfile** (more recommended):

```dockerfile
FROM osrf/ros:humble-desktop

# Basic tools
RUN apt-get update && apt-get install -y \
    python3-pip git wget curl vim \
    ros-humble-rviz2 \
    ros-humble-rqt* \
    && rm -rf /var/lib/apt/lists/*

# Python packages
RUN pip3 install torch torchvision numpy opencv-python-headless

# ROS2 workspace
RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws

# If there are packages that need source build, put them here
# RUN cd src && git clone https://github.com/...
# RUN . /opt/ros/humble/setup.sh && colcon build

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
```

Why a Dockerfile beats `docker commit`: later you can trace "what is installed in this image?". An image built by commit has no history, so you cannot reproduce it.

> **Further reading**
> - [turlucode/ros-docker-gui](https://github.com/turlucode/ros-docker-gui) — reference for ROS + NVIDIA + GUI Docker setup. Supports Melodic through Humble
> - [NVIDIA Container Toolkit Documentation](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/index.html) — official install guide
> - [OSRF Docker Images](https://hub.docker.com/r/osrf/ros) — official ROS Docker images. `humble-desktop` is the version that includes GUI

> **Docker requirements check**: State up front whether the setup combines ROS2, a GPU, USB sensors, and GUI visualization. Independently generated snippets can conflict in their permissions and network options. Check whether the final command needs `QT_X11_NO_MITSHM`, `ROS_DOMAIN_ID`, and explicit device mappings.

## 16.4 Remote Management: Git, SSH, File Transfer

A remote experiment environment is managed through SSH access, Git history, and file transfer between servers.

### 16.4.1 Git/GitHub

### 16.4.1.1 Basic Workflow

Git records code changes and the state used for each experiment. Recording the commit with a result makes it possible to reproduce an earlier state or compare the source of a change.

```bash
# Clone a repository
git clone https://github.com/user/repo.git

# Check changes
git status
git diff

# Commit
git add .
git commit -m "feat: add new feature"

# Push
git push origin main
```

> **Further reading**
> - [GitHub's Git Handbook](https://docs.github.com/en/get-started/using-git/about-git) — official guide that cleanly organizes Git's core concepts
> - [The Missing Semester - Version Control (Git)](https://missing.csail.mit.edu/2020/version-control/) — MIT lecture. Explains Git's internal model (DAG), giving a deeper understanding

### 16.4.1.2 Branching Strategy

**Git Flow**:
- `main`: stable version
- `develop`: development version
- `feature/*`: feature development
- `hotfix/*`: urgent fixes

**GitHub Flow** (simple):
- `main`: always deployable
- `feature-branch`: per-feature branch → PR → merge

When several people edit the same code, feature branches can isolate each change before review and merge into `main`. A simple GitHub Flow is often enough to separate conflict scope and change intent.

### 16.4.1.3 Collaboration

**Pull Request (PR)**:
1. Fork or create a branch
2. Commit changes
3. Open a PR and request review
4. Merge after code review

**Commit message convention** (Conventional Commits):

```
feat: new feature
fix: bug fix
docs: documentation change
refactor: refactoring
test: add/modify tests
chore: build/config changes
```

> **Further reading**
> - [GitHub's Git Handbook](https://docs.github.com/en/get-started/using-git/about-git) — Git introduction written by GitHub itself
> - [Conventional Commits Specification](https://www.conventionalcommits.org/) — official spec for commit message conventions

### 16.4.2 SSH

The basic tool for accessing a lab GPU server. Using key authentication instead of a password is both convenient and secure.

```bash
# Generate keys (once, the first time)
ssh-keygen -t ed25519

# Register the public key on the server
ssh-copy-id user@server_ip

# Connect
ssh user@server_ip

# Port forwarding (view the server's Jupyter/TensorBoard locally)
ssh -L 8888:localhost:8888 user@server_ip
```

Configuring **~/.ssh/config** removes the need to type the IP and username every time:

```
Host lab-server
    HostName 192.168.1.100
    User junholee
    IdentityFile ~/.ssh/id_ed25519
```

After this, `ssh lab-server` connects directly. VS Code Remote-SSH also reads this config.

### 16.4.3 SCP & rsync

Tools for file transfer between server and local. **SCP** is simple file copy; **rsync** transfers only changed parts.

**SCP**:

```bash
# Local → server
scp model.pth user@server:/home/user/weights/

# Server → local
scp user@server:/home/user/results/log.txt ./

# Directory copy
scp -r dataset/ user@server:/data/
```

**rsync** — better for large datasets or repeated transfers:

```bash
# Local → server (transfer only changes, show progress)
rsync -avz --progress dataset/ user@server:/data/dataset/

# Server → local
rsync -avz user@server:/home/user/results/ ./results/

# Mirror deleted files as well
rsync -avz --delete source/ user@server:/data/source/
```

`scp` copies the whole thing every time; `rsync` sends only the diff, which makes a big difference when syncing datasets of tens of GB.

### 16.4.4 Tailscale

If the lab server sits behind NAT or a firewall, SSH from outside does not work. Tailscale is a WireGuard-based VPN; install it and you can connect directly to the lab server from anywhere.

```bash
# Install (on both server and local)
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up

# Check status — list of connected devices and IPs
tailscale status

# Then SSH to the Tailscale IP
ssh user@100.x.y.z
```

**Pros**:
- No port forwarding or router configuration needed
- Reach the server from cafe, home, or school at the same IP
- With Tailscale SSH, SSH key management is also automated

Registering the Tailscale IP in **~/.ssh/config** is convenient:

```
Host lab-gpu
    HostName 100.x.y.z
    User junholee
```

> **Further reading**
> - [Tailscale official docs](https://tailscale.com/kb/) — from installation to ACL configuration
> - [The Missing Semester - Remote Machines](https://missing.csail.mit.edu/2020/command-line/#remote-machines) — SSH, port forwarding, tmux, and other remote work basics

## 16.5 Experiment Management

### 16.5.1 Weights & Biases (wandb)

When running deep-learning experiments, the question "what were the hyperparameters of the model I ran yesterday?" comes up every day. Logging in Excel or a notebook hits its limits quickly. wandb automatically logs and visualizes training, and it also makes sharing results with teammates easy. It can track metrics and hyperparameters, version model artifacts, and share dashboards with a team.

```python
import wandb

# Initialize
wandb.init(project="my-project", config={
    "learning_rate": 0.001,
    "epochs": 100
})

# Logging
for epoch in range(epochs):
    loss = train_one_epoch()
    wandb.log({"loss": loss, "epoch": epoch})

# Finish
wandb.finish()
```

> **Further reading**
> - [Weights & Biases official docs and Quickstart](https://docs.wandb.ai/quickstart) — wandb getting-started guide. You can log your first experiment in 5 minutes
> - [Weights & Biases YouTube](https://www.youtube.com/@WeightsBiases) — tutorials and MLOps talks

### 16.5.2 MLflow

Where wandb is a cloud-based service, MLflow is an open-source alternative you can run on your own server. Useful where data security matters.

```python
import mlflow

mlflow.set_experiment("my-experiment")

with mlflow.start_run():
    mlflow.log_param("lr", 0.001)
    mlflow.log_metric("accuracy", 0.95)
    mlflow.pytorch.log_model(model, "model")
```

### 16.5.3 TensorBoard

```python
from torch.utils.tensorboard import SummaryWriter

writer = SummaryWriter("runs/experiment1")
writer.add_scalar("Loss/train", loss, epoch)
writer.add_image("Sample", image, epoch)
writer.close()
```

```bash
tensorboard --logdir runs
```

TensorBoard works directly with PyTorch and visualizes locally without a separate account. For simple experiments, TensorBoard alone is enough, without wandb.

> **Further reading**
> - [PyTorch TensorBoard Tutorial](https://pytorch.org/tutorials/recipes/recipes/tensorboard_with_pytorch.html) — official guide to using TensorBoard from PyTorch
> - [MLflow Documentation](https://mlflow.org/docs/latest/index.html) — official MLflow documentation

## 16.6 Code Formatting

### 16.6.1 Linting & Formatting

When code style differs between people, code reviews spend more time on style disputes than on logic. Automatic formatters remove this problem. Even when working alone, a consistent code style helps a lot when you re-read your own code later.

**Python**:

```bash
# Ruff (fast linter, Black-compatible formatter)
pip install ruff
ruff check .
ruff format .

# Black (formatter)
pip install black
black .

# Type checking
pip install mypy
mypy .
```

**C++**:

```bash
# clang-format
clang-format -i src/*.cpp
```

### 16.6.2 Testing

Writing tests matters even for research code. Just having basic tests for "does the model forward pass work" or "is the data preprocessing output as expected" makes refactoring much less stressful.

**Python (pytest)**:

```python
# test_module.py
def test_addition():
    assert 1 + 1 == 2

def test_function():
    result = my_function(input)
    assert result == expected
```

```bash
pytest tests/ -v
```

**C++ (gtest)**:

```cpp
#include <gtest/gtest.h>

TEST(MyTest, BasicTest) {
    EXPECT_EQ(1 + 1, 2);
}
```

> **Further reading**
> - [Real Python - Python Testing with pytest](https://realpython.com/pytest-python-testing/) — detailed tutorial on using pytest
> - [The Missing Semester (MIT)](https://missing.csail.mit.edu/) — shell, editor, debugging, profiling, and development tools broadly. Worth a full pass before you start graduate school
> - [Fireship YouTube](https://www.youtube.com/@Fireship) — skim various development tools quickly through the "100 Seconds" series
> - [Jinyong Jeong's blog — robot software development culture](https://jinyongjeong.github.io/2025/02/14/developmen_culture/) — establishing code review, CI/CD, style guides, and other development-culture practices in a robotics team
> - [Jinyong Jeong's blog — robot development and test code](https://jinyongjeong.github.io/2025/02/19/test_code/) — six reasons test code is essential in robot software

## Technical Timeline: Development Environment & Tools — Past → Present → Future

```
2005 ─── Git is born (Linus Torvalds)
  │       the start of distributed version control
  │
2008 ─── GitHub launches
  │       becomes the hub of open-source collaboration
  │
2010 ─── Conda (Anaconda) appears
  │       becomes a widely used Python environment manager in data science
  │
2013 ─── Docker released
  │       container images make user-space dependencies easier to reproduce
  │
2015 ─── TensorBoard (released with TensorFlow)
  │       the beginning of deep-learning training visualization
  │
2017 ─── nvidia-docker released
  │       GPU usage inside Docker containers becomes possible
  │
2018 ─── Weights & Biases launches
  │       experiment tracking, visualization, and team collaboration in the cloud
  │
2020 ─── Ruff appears (2022), Black goes mainstream
  │       Python code-quality tooling speeds up
  │
2023+ ── AI-assisted development tools spread
          AI coding assistants such as Copilot and Cursor
          Dev Container standardization (VS Code Remote)
          Docker + wandb combination for reproducible research becomes common
```
