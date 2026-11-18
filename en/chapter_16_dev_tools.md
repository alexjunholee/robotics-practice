# Ch.16 — Development Environment & Tools

The frameworks and simulators introduced in Ch.15 depend heavily on exact combinations of Ubuntu, CUDA, Python, ROS2, and system libraries. Without disciplined environment management, a minor driver upgrade or conflicting `pip` install can break an entire pipeline. Reproducible robotics engineering rests on three habits: knowing the host language toolchains, isolating dependencies with virtual environments and Docker, and standardizing remote workflows, experiment tracking, and automated checks.

## 16.1 Programming Languages

Even when AI coding assistants generate boilerplate or draft functions, robotics engineers still have to read existing repositories, trace runtime bugs, and verify generated patches. Inspecting cloned SLAM pipelines or tuning real-time control loops requires fluency in both C++ and Python.

### 16.1.1 C++

Most performance-critical robotics codebases—including real-time control loops, core ROS packages, and SLAM systems such as ORB-SLAM3, LOAM, and VINS-Mono—are written in C++. Engineers regularly read and extend these repositories because C++ delivers deterministic execution speed and direct control over memory layout. Those advantages come with trade-offs: a steeper learning curve, slower compile-edit cycles, and memory-management bugs that can crash a node silently or corrupt state. Modern C++ (C++17/20) mitigates many of those hazards through smart pointers, range-based loops, and lambdas:

```cpp
// Smart pointer
auto ptr = std::make_shared<MyClass>();

// Range-based for
for (const auto& item : container) { ... }

// Lambda
auto func = [&](int x) { return x * 2; };
```

> **Further reading**
> - [C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines) — Modern C++ coding guidelines maintained by Bjarne Stroustrup and Herb Sutter.
> - [The Cherno - C++ Playlist](https://www.youtube.com/playlist?list=PLlrATfBNZ98dudnM48yfGUldqGD0S4FFb) — Video series covering C++ from fundamentals to advanced internals.
> - [Modernes C++](https://www.modernescpp.com/index.php) — Blog offering a systematic treatment of C++17, C++20, and C++23 features.

> **⚠ Target-environment check**: Always state whether your target hardware is x86 or Jetson (ARM) and whether cross-compilation is involved, then verify that every generated dependency and compiler flag supports that architecture.

### 16.1.2 Python

Python dominates rapid prototyping, PyTorch model training and inference, data preprocessing, and visualization. While coding agents can scaffold training scripts quickly, the researcher still needs to audit tensor shapes, verify preprocessing logic, and profile data-loading bottlenecks. Most robotics ML workflows rely on a compact set of core packages covering numerical arrays, 2D/3D vision, deep learning, and foundation models:

```bash
pip install numpy scipy matplotlib
pip install opencv-python open3d
pip install torch torchvision
pip install transformers  # HuggingFace
```

> **Further reading**
> - [Real Python](https://realpython.com/) — Structured tutorials covering Python from fundamentals to advanced patterns.
> - [Fireship - Python in 100 Seconds](https://www.youtube.com/watch?v=x7X9w_GIm1s) — Fast high-level overview of Python's syntax and ecosystem.

## 16.2 Development Environment Setup

Building a reliable workstation or lab server starts with aligning the operating system, GPU driver stack, and per-project Python environments.

### 16.2.1 Ubuntu

Ubuntu is the primary supported operating system for ROS and provides the best-documented paths for NVIDIA driver, CUDA, and cuDNN installation. Partial workflows on macOS or Windows are feasible, though the right host choice ultimately depends on your target ROS distribution and the OS running on the physical robot.

**Recommended versions**:
- Ubuntu 22.04 LTS (ROS2 Humble)
- Ubuntu 24.04 LTS (ROS2 Jazzy)

On a fresh Ubuntu install, configure compilers, Python environment tools, and terminal utilities first:

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
> - [The Missing Semester of Your CS Education (MIT)](https://missing.csail.mit.edu/) — Practical course covering shell scripting, Vim, `tmux`, Git, and everyday developer tools rarely taught in standard CS classes.
> - [Fireship - Linux in 100 Seconds](https://www.youtube.com/watch?v=rrB13utjYV4) — Rapid conceptual overview of Linux.

### 16.2.2 CUDA / cuDNN

Deep-learning models rely on GPU acceleration for training and high-throughput inference. On NVIDIA machines, if the host driver is too old for the CUDA runtime bundled inside PyTorch, `torch.cuda` fails to initialize the GPU. Start by inspecting the driver status and any locally installed CUDA compiler:

```bash
nvidia-smi          # GPU status
nvcc --version      # CUDA version
```

For clean setups, match your CUDA and cuDNN versions directly against the official installation matrix of the PyTorch or TensorFlow release you plan to run.

> **Further reading**
> - [PyTorch - Previous Versions](https://pytorch.org/get-started/previous-versions/) — Compatibility matrix for matching PyTorch releases with CUDA versions.
> - [NVIDIA CUDA Toolkit Documentation](https://docs.nvidia.com/cuda/) — Official NVIDIA CUDA documentation.

**NVIDIA driver install troubleshooting**

When installing proprietary NVIDIA drivers on Ubuntu, the most frequent failure is a conflict with `nouveau`, the default open-source kernel driver. Blacklist `nouveau`, rebuild the initramfs, and install the driver via `apt`:

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

If `nvidia-smi` still fails to detect the GPU after rebooting, check two things. First, run `sudo dkms status` to verify that the kernel module compiled cleanly and `lsmod | grep nvidia` to confirm it actually loaded. Second, if UEFI Secure Boot is enabled, the kernel will reject unsigned modules—either enroll a Machine Owner Key (MOK), switch to a distribution-signed driver package, or disable Secure Boot in firmware. (Reference: [Jinyong Jeong's blog](https://jinyongjeong.github.io/2016/11/22/ubuntu_graphic_driver_install/))

**CUDA/cuDNN version compatibility**

Official PyTorch wheels bundle their own CUDA runtime libraries, which means the host's standalone CUDA toolkit (`nvcc`) does not need to match PyTorch's CUDA version. What actually matters is whether the installed NVIDIA display driver is new enough to support the runtime inside the wheel. When that requirement fails, `import torch` succeeds silently, only for `torch.cuda.is_available()` to return `False` or throw an error on the first GPU call. Diagnose mismatches in the following order:

```bash
# 1. Check the GPU
nvidia-smi  # The "CUDA Version" in the top right is the "max version supported by the driver"

# 2. Check the installed CUDA toolkit
nvcc --version

# 3. Check which CUDA PyTorch is using
python -c "import torch; print(torch.version.cuda)"

# With wheels there is only one condition: does the driver (1) support PyTorch's CUDA runtime (3)?
# nvcc (2) only matters when building from source or compiling custom CUDA extensions. Check combinations on the PyTorch official site:
# https://pytorch.org/get-started/locally/
```

(Reference: [Jinyong Jeong's blog](https://jinyongjeong.github.io/2016/09/19/cuda_setting/))

**Building OpenCV + CUDA from source**

Prebuilt OpenCV packages from `apt` (`python3-opencv`) and `pip` (`opencv-python`) omit CUDA support. Workflows that require GPU-accelerated vision—such as OpenCV's DNN module or CUDA optical flow—must compile OpenCV together with `opencv_contrib` from source:

```bash
# Install dependencies
sudo apt install -y build-essential cmake git libgtk2.0-dev pkg-config \
    libavcodec-dev libavformat-dev libswscale-dev \
    libtbb-dev libjpeg-dev libpng-dev \
    python3-dev python3-numpy   # required for BUILD_opencv_python3=ON. Check the Python 3 entry in the cmake summary

# OpenCV + contrib source
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git

# Build (enable CUDA)
cd opencv && mkdir build && cd build
# Adjust CUDA_ARCH_BIN to your GPU (RTX 3090=8.6, RTX 4090=8.9). A comment cannot follow a line continuation (\)
cmake -D CMAKE_BUILD_TYPE=Release \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
      -D WITH_CUDA=ON \
      -D CUDA_ARCH_BIN="8.6" \
      -D WITH_CUDNN=ON \
      -D OPENCV_DNN_CUDA=ON \
      -D BUILD_opencv_python3=ON \
      ..
make -j$(nproc)
sudo make install
```

Set `CUDA_ARCH_BIN` carefully to match your GPU's compute capability; an incorrect architecture flag can compile cleanly yet produce degraded performance or runtime kernel errors. Consult [NVIDIA GPU Compute Capability](https://developer.nvidia.com/cuda-gpus) before building.

Caveat: `pip`-installed OpenCV and `apt`-installed `cv_bridge` frequently collide in ROS environments due to conflicting shared library versions (see Ch.21 Appendix C.4, troubleshooting). Compiling CUDA OpenCV directly onto the host amplifies that collision risk, so isolating custom OpenCV builds inside a Docker container is strongly preferred.

(Reference: [Dark Programmer — building OpenCV + CUDA from source](https://darkpgmr.tistory.com/184))

### 16.2.3 Environment Management

Different projects routinely demand incompatible Python interpreters and library versions. Installing packages with global `pip` quickly creates "dependency hell," where upgrading a library for one repository silently breaks another. Isolating each project inside a dedicated **Conda** or **`venv`** environment prevents those collisions.

**Conda** (recommended when managing multiple Python versions or compiled binary toolchains):

```bash
# Install Miniconda
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh

# Create an environment
conda create -n myenv python=3.10
conda activate myenv

# Install packages
# PyTorch stopped publishing to the pytorch Anaconda channel as of 2.6. Use the pip index inside the conda environment
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121
```

**venv** (lightweight standard-library option when the system Python version already matches):

```bash
python3 -m venv myenv
source myenv/bin/activate
pip install -r requirements.txt
```

In practice, a legacy SLAM codebase might require Python 3.8 while a newer Transformer model requires Python 3.10. With Conda, switching contexts is as clean as running `conda activate slam_env` or `conda activate transformer_env`.

> **Further reading**
> - [Conda Documentation - Managing Environments](https://docs.conda.io/projects/conda/en/latest/user-guide/tasks/manage-environments.html) — Official guide to creating, exporting, and managing Conda environments.
> - [Python venv Documentation](https://docs.python.org/3/library/venv.html) — Official Python virtual environment reference.

## 16.3 Docker

### 16.3.1 Why Docker?

Docker packages the OS user space, system libraries, and runtime configuration into a portable container image. Distributing that image alongside research code minimizes machine-to-machine drift across labs and deployment targets:

- **Reproducibility**: Pinning the user-space image eliminates most dependency drift across machines, though host kernel, GPU driver, hardware, and external services still influence execution.
- **Isolation**: Heavy system packages and custom builds stay contained inside the image rather than polluting the host OS.
- **Deployment**: Teams and reviewers can pull an identical runtime environment across workstations, servers, and robots.
- **Complex dependencies**: Intricate combinations of ROS2, CUDA, and C++ libraries can be built once and versioned cleanly.

### 16.3.2 Basic Usage

The three core Docker operations—pulling a prebuilt base image, launching an interactive GPU container with a mounted workspace, and building a custom image from a `Dockerfile`—look like this:

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

A minimal `Dockerfile` layers system packages and Python requirements on top of an NVIDIA CUDA base image:

```dockerfile
FROM nvidia/cuda:12.1.0-devel-ubuntu22.04

RUN apt-get update && apt-get install -y \
    python3-pip git

COPY requirements.txt /tmp/
RUN pip3 install -r /tmp/requirements.txt

WORKDIR /workspace
```

> **Further reading**
> - [Docker official Getting Started Guide](https://docs.docker.com/get-started/) — Best starting point for understanding containers, images, and bind mounts.
> - [NetworkChuck - Docker Tutorial](https://www.youtube.com/watch?v=eGz9DS-aIeY) — Accessible introductory video on Docker concepts.
> - [Fireship - Docker in 100 Seconds](https://www.youtube.com/watch?v=Gjnup-PuquQ) — Quick overview of how Docker containers work.

### 16.3.3 NVIDIA Container Toolkit

Standard Docker containers cannot see host GPUs by default. Running PyTorch training or GPU-accelerated robotics packages inside Docker requires installing `nvidia-container-toolkit` on the host and passing `--gpus all` to `docker run` (modern Docker uses `--gpus all` rather than the older `--runtime=nvidia` flag):

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

Robotics development frequently requires GPU compute, GUI tools such as RViz and Gazebo, and live USB sensor access inside the same container. The recipe below—adapting [turlucode/ros-docker-gui](https://github.com/turlucode/ros-docker-gui) for `nvidia-container-toolkit` and ROS2 Humble—configures those permissions and mounts together.

**Step 1: Prepare X11 forwarding (host)**

Grant local X11 display access on the host before starting GUI tools inside the container:

```bash
# Run once on the host
sudo apt-get install -y xauth
xhost +local:docker
```

Note that `xhost +local:docker` adds a host-based access rule permitting local connections without distinguishing between containers; when stricter per-container isolation is needed, rely on `xauth` cookie-based authentication instead.

**Step 2: Run script**

Save the launch command in a helper script so runtime flags stay consistent across sessions:

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

Each flag serves a specific role in bridging GPU hardware, DDS networking, X11 graphics, and USB devices:

| Flag | Role |
|--------|------|
| `--gpus all` | GPU passthrough (nvidia-container-toolkit) |
| `--privileged` | Full access to USB/serial devices. In production, map individually with `--device` |
| `--net=host` | Share host network to simplify DDS multicast setup. Optional, depending on the communication setup |
| `--ipc=host` | Shared memory. Required by GUI tools like RViz |
| `-e QT_X11_NO_MITSHM=1` | Workaround used when RViz and similar tools exit because of MIT-SHM compatibility problems |
| `-e ROS_DOMAIN_ID=42` | Isolates from other ROS2 systems on the same network. Essential when multiple people use the lab |
| `-v /dev:/dev` | Sensor USB may be plugged in at any time, so mount all of /dev. Pairs with `--privileged` |
| `-v .Xauthority` | X11 authentication. If you are not using the xhost setup from Step 1, without it you get `cannot open display` |

**Step 3: Save the container after work**

Because Step 2 passes `--rm`, Docker deletes the container the moment you exit its shell, discarding any changes made inside it. To snapshot an interactive session with `docker commit`, either commit from a second terminal while the container is still running or omit `--rm` when launching:

```bash
# If you installed packages or did other work inside the container, commit it
docker commit ros2_dev my_ros2_workspace:v1

# Next time, run from the saved image
# In run_ros2_docker.sh just change the image name
```

**Managing it with a Dockerfile** (recommended over `docker commit`):

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

A `Dockerfile` beats `docker commit` for long-term maintenance because every installed package and build step stays readable and version-controlled in plain text. While a committed image preserves binary layer history, reconstructing what was installed months later—or rebuilding for a newer base image—becomes guesswork without a `Dockerfile`.

> **Further reading**
> - [turlucode/ros-docker-gui](https://github.com/turlucode/ros-docker-gui) — Reference repository for ROS + NVIDIA + GUI Docker setups (supports Melodic through Humble).
> - [NVIDIA Container Toolkit Documentation](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/index.html) — Official NVIDIA Container Toolkit installation guide.
> - [OSRF Docker Images](https://hub.docker.com/r/osrf/ros) — Official ROS Docker images (`humble-desktop` includes GUI packages).

> **Docker requirements check**: State up front whether your setup combines ROS2, GPU acceleration, USB sensors, and GUI visualization. Snippets generated in isolation often conflict in their permission and network flags; verify explicitly whether the final `docker run` command requires `QT_X11_NO_MITSHM`, `ROS_DOMAIN_ID`, and explicit `--device` mappings.

## 16.4 Remote Management: Git, SSH, File Transfer

Robotics research rarely happens on a single machine: code is edited locally, trained on remote GPU servers, and deployed onto onboard robot computers. Managing that distributed workflow reliably rests on Git history, key-authenticated SSH, and delta-aware file transfers.

### 16.4.1 Git/GitHub

Version control anchors both team collaboration and experimental reproducibility across local laptops, GPU servers, and robot platforms.

### 16.4.1.1 Basic Workflow

Git tracks code changes and ties every experiment back to the exact source state that produced it. Logging the commit hash alongside an experiment makes it straightforward to reproduce prior results or pinpoint regressions:

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
> - [GitHub's Git Handbook](https://docs.github.com/en/get-started/using-git/about-git) — Official guide introducing Git's core model and workflows.
> - [The Missing Semester - Version Control (Git)](https://missing.csail.mit.edu/2020/version-control/) — MIT lecture explaining Git's internal DAG data model.

### 16.4.1.2 Branching Strategy

When multiple lab members modify the same repository, feature branches isolate unfinished work before review and integration into `main`. Two branching conventions cover most robotics teams:

- **Git Flow**: Separates `main` (stable releases), `develop` (integration), `feature/*` (new work), and `hotfix/*` (urgent patches) for larger multi-release codebases.
- **GitHub Flow** (simpler): Keeps `main` always deployable and routes every change through `feature-branch` → Pull Request → merge, which is usually sufficient for research labs to bound merge conflicts and isolate change intent.

### 16.4.1.3 Collaboration

In collaborative repositories, changes move from local development into `main` through a structured **Pull Request (PR)** cycle: fork or create a feature branch, commit focused changes, open a PR for peer review, and merge once tests and review pass. Adopting **Conventional Commits** keeps commit logs scannable when bisecting bugs or assembling release notes:

```
feat: new feature
fix: bug fix
docs: documentation change
refactor: refactoring
test: add/modify tests
chore: build/config changes
```

> **Further reading**
> - [GitHub's Git Handbook](https://docs.github.com/en/get-started/using-git/about-git) — Git introduction published by GitHub.
> - [Conventional Commits Specification](https://www.conventionalcommits.org/) — Official specification for structured commit messages.

### 16.4.2 SSH

SSH is the standard entry point into lab GPU servers and onboard robot computers. Using Ed25519 key-based authentication rather than passwords improves both security and daily speed, and SSH port forwarding lets you inspect remote Jupyter notebooks or TensorBoard dashboards safely in a local browser:

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

Defining host aliases in **`~/.ssh/config`** eliminates repeated typing of IP addresses, usernames, and key paths:

```
Host lab-server
    HostName 192.168.1.100
    User junholee
    IdentityFile ~/.ssh/id_ed25519
```

Once configured, `ssh lab-server` connects directly, and tools such as VS Code Remote-SSH automatically pick up the same host definition.

### 16.4.3 SCP & rsync

Moving model weights, logs, and datasets between laptops and GPU servers relies on **`scp`** and **`rsync`**. Use `scp` for quick, one-off copies of individual files or small directories:

```bash
# Local → server
scp model.pth user@server:/home/user/weights/

# Server → local
scp user@server:/home/user/results/log.txt ./

# Directory copy
scp -r dataset/ user@server:/data/
```

For large datasets or repeated syncs, switch to **`rsync`**:

```bash
# Local → server (transfer only changes, show progress)
rsync -avz --progress dataset/ user@server:/data/dataset/

# Server → local
rsync -avz user@server:/home/user/results/ ./results/

# Mirror deleted files as well
rsync -avz --delete source/ user@server:/data/source/
```

Whereas `scp` re-copies every file unconditionally, `rsync` transfers only changed blocks—saving substantial bandwidth and time when syncing datasets spanning tens of gigabytes.

### 16.4.4 Tailscale

When a lab server or robot sits behind NAT or a campus firewall, inbound SSH from off-campus fails. **Tailscale**, a mesh VPN built on WireGuard, assigns persistent private IPs across your devices so you can connect directly from off-site without router port forwarding:

```bash
# Install (on both server and local)
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up

# Check status — list of connected devices and IPs
tailscale status

# Then SSH to the Tailscale IP
ssh user@100.x.y.z
```

Beyond bypassing NAT and router configuration, Tailscale provides a stable IP regardless of whether you connect from home, campus, or on the road, and **Tailscale SSH** can automate SSH key distribution across authorized devices. Adding the Tailscale address to **`~/.ssh/config`** keeps remote access seamless:

```
Host lab-gpu
    HostName 100.x.y.z
    User junholee
```

> **Further reading**
> - [Tailscale official docs](https://tailscale.com/kb/) — Guides covering installation, networking, and ACL configuration.
> - [The Missing Semester - Remote Machines](https://missing.csail.mit.edu/2020/command-line/#remote-machines) — Practical coverage of SSH, port forwarding, and `tmux` for remote sessions.

## 16.5 Experiment Management

Running dozens of training jobs quickly raises the question of which hyperparameters, dataset splits, and code revisions produced a given checkpoint. Structured experiment trackers replace ad-hoc spreadsheets with searchable run histories.

### 16.5.1 Weights & Biases (wandb)

Weights & Biases (**wandb**) logs hyperparameters, training metrics, and model artifacts automatically to a shared cloud dashboard, making it straightforward to compare runs and share progress across a lab:

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
> - [Weights & Biases official docs and Quickstart](https://docs.wandb.ai/quickstart) — Quickstart guide to logging your first experiment in minutes.
> - [Weights & Biases YouTube](https://www.youtube.com/@WeightsBiases) — Tutorials and MLOps engineering talks.

### 16.5.2 MLflow

Whereas wandb runs as a hosted cloud service, **MLflow** is an open-source tracking platform you can self-host on internal lab infrastructure—a useful advantage when proprietary datasets or strict data-governance rules prohibit external logging:

```python
import mlflow

mlflow.set_experiment("my-experiment")

with mlflow.start_run():
    mlflow.log_param("lr", 0.001)
    mlflow.log_metric("accuracy", 0.95)
    mlflow.pytorch.log_model(model, "model")
```

### 16.5.3 TensorBoard

**TensorBoard** integrates directly with PyTorch via `SummaryWriter` and runs completely locally without any external account or tracking server. For focused single-machine experiments, logging scalars and sample images to disk and launching the local viewer is often all you need:

```python
from torch.utils.tensorboard import SummaryWriter

writer = SummaryWriter("runs/experiment1")
writer.add_scalar("Loss/train", loss, epoch)
writer.add_image("Sample", image, epoch)
writer.close()
```

Serve the recorded event files locally with:

```bash
tensorboard --logdir runs
```

> **Further reading**
> - [PyTorch TensorBoard Tutorial](https://pytorch.org/tutorials/recipes/recipes/tensorboard_with_pytorch.html) — Official guide to logging PyTorch training runs to TensorBoard.
> - [MLflow Documentation](https://mlflow.org/docs/latest/index.html) — Official MLflow tracking and model registry reference.

## 16.6 Code Formatting

Clean formatting and lightweight automated tests keep research codebases readable as projects grow from quick prototypes into shared lab infrastructure.

### 16.6.1 Linting & Formatting

When contributors follow inconsistent formatting conventions, code reviews bog down in whitespace nitpicks instead of algorithmic logic. Enforcing automated formatters and static checks eliminates those disputes—and pays off even on solo projects when revisiting code months later.

**Python** workflows commonly pair **Ruff** (a fast linter and Black-compatible formatter) or **Black** with **mypy** for static type checking:

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

**C++** packages standardize indentation and brace style with `clang-format`:

```bash
# clang-format
clang-format -i src/*.cpp
```

### 16.6.2 Testing

Even fast-moving research code benefits from basic automated tests. Verifying invariants such as "does a model's forward pass emit expected tensor shapes?" or "does the dataset parser produce valid bounding boxes?" lets you refactor confidently without silently breaking downstream experiments.

**Python (`pytest`)** keeps unit tests concise and easy to run after every edit:

```python
# test_module.py
def test_addition():
    assert 1 + 1 == 2

def test_function():
    result = my_function(input)
    assert result == expected
```

Execute the test suite with verbose reporting:

```bash
pytest tests/ -v
```

**C++ (`gtest`)** integrates cleanly into CMake and `colcon` builds for validating core geometry, control, and ROS node logic:

```cpp
#include <gtest/gtest.h>

TEST(MyTest, BasicTest) {
    EXPECT_EQ(1 + 1, 2);
}
```

> **Further reading**
> - [Real Python - Python Testing with pytest](https://realpython.com/pytest-python-testing/) — Detailed tutorial on writing and structuring tests with `pytest`.
> - [The Missing Semester (MIT)](https://missing.csail.mit.edu/) — Broad coverage of shells, editors, debugging, profiling, and developer tools; worth completing early in graduate research.
> - [Fireship YouTube](https://www.youtube.com/@Fireship) — Fast conceptual overviews of developer tooling via the "100 Seconds" series.
> - [Jinyong Jeong's blog — robot software development culture](https://jinyongjeong.github.io/2025/02/14/developmen_culture/) — Building code review, CI/CD, and style-guide practices in a robotics team.
> - [Jinyong Jeong's blog — robot development and test code](https://jinyongjeong.github.io/2025/02/19/test_code/) — Six reasons automated tests are essential in robot software.

## Technical Timeline: Development Environment & Tools — Past → Present → Future

```
2005 ─── Git is born (Linus Torvalds)
  │       the start of distributed version control
  │
2008 ─── GitHub launches
  │       becomes the hub of open-source collaboration
  │
2012 ─── Conda (Anaconda) appears
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
2020~2022 ─── Black goes mainstream, Ruff appears (2022)
  │       Python code-quality tooling speeds up
  │
2023+ ── AI-assisted development tools spread
          AI coding assistants such as Copilot and Cursor
          Dev Container standardization (VS Code Remote)
          Docker + wandb combination for reproducible research becomes common
```

With reproducible environments, containerized GPU stacks, remote workflows, and experiment tracking in place, the remaining pillar of empirical robotics is data. Ch.17 examines the public datasets and benchmarks used across autonomous driving, SLAM, and 3D vision, along with practical procedures for writing data loaders and collecting synchronized, calibrated custom recordings.
