# Ch.21 — Appendix


## A. Glossary

### A.1 Abbreviations

| Abbr. | Expansion | Description |
| --- | --- | --- |
| SLAM | Simultaneous Localization and Mapping | simultaneous localization and mapping |
| VO | Visual Odometry | visual odometry |
| VIO | Visual-Inertial Odometry | visual-inertial odometry |
| LIO | LiDAR-Inertial Odometry | LiDAR-inertial odometry |
| IMU | Inertial Measurement Unit | inertial measurement unit |
| DoF | Degrees of Freedom | degrees of freedom |
| SE(3) | Special Euclidean Group (3D) | 3D rigid-body transformation group |
| SO(3) | Special Orthogonal Group (3D) | 3D rotation group |
| FoV | Field of View | field of view |
| ToF | Time of Flight | time of flight (distance-measurement method) |
| CNN | Convolutional Neural Network | convolutional neural network |
| ViT | Vision Transformer | vision Transformer |
| VFM | Vision Foundation Model | vision foundation model |
| VLA | Vision-Language-Action | vision-language-action model |
| VLM | Vision-Language Model | vision-language model |
| LLM | Large Language Model | large language model |
| mAP | mean Average Precision | mean average precision |
| ICP | Iterative Closest Point | iterative closest point |
| NDT | Normal Distributions Transform | normal distributions transform |
| NeRF | Neural Radiance Fields | neural radiance fields |
| 3DGS | 3D Gaussian Splatting | 3D Gaussian splatting |
| BEV | Bird's Eye View | bird's-eye view |
| TSDF | Truncated Signed Distance Function | truncated signed distance function |
| BA | Bundle Adjustment | bundle adjustment |
| PGO | Pose Graph Optimization | pose graph optimization |
| DDS | Data Distribution Service | ROS2's communication middleware |
| ONNX | Open Neural Network Exchange | model conversion format |
| TRT | TensorRT | NVIDIA's inference optimization engine |
| ATE | Absolute Trajectory Error | absolute trajectory error |
| RPE | Relative Pose Error | relative pose error |

### A.2 Terms

**Keyframe**: A selected frame that carries significant information. Processing every frame is too slow, so only frames with meaningful changes are picked and used.

**Loop Closure**: Drift correction through recognition of a previously visited location. "Ah, we were here before" → accumulated error gets corrected all at once.

**Drift**: Accumulation of error. Walking 100 m with 1 cm of error at every step yields 100 cm of error on arrival.

**Reprojection Error**: The error when a 3D point is reprojected onto the image. The difference between the predicted "where should this 3D point appear in the camera image" and the actual observation.

**Feature Descriptor**: A vector that describes the neighborhood around a keypoint. When finding the same point across two images, these vectors are compared.

**Homography**: A transformation between planes. Used when registering two photos taken of a desktop.

**Essential Matrix**: The geometric relation between a calibrated camera pair. 5 DoF (3 for rotation + 2 for translation direction).

**Fundamental Matrix**: The geometric relation between an uncalibrated camera pair. 7 DoF.

**Epipole**: The point where the center of one camera is projected onto the other camera's image.

**Zero-shot**: Performing a new task without training. "Find the cat" works even though "cat" was never trained on.

**Few-shot**: Learning a new task from a small number of examples. Learns from just 3–5 examples.

**Fine-tuning**: Retraining a pretrained model for a specific task. Adjusting a large model to your own data.

**Domain Adaptation**: Adapting from a source domain to a target domain. Train in simulation → deploy in the real environment.

**Sim-to-Real**: Transferring from simulation to the real environment. The canonical case of domain adaptation.

**Gaussian Splatting**: A method that represents a 3D scene with millions of 3D Gaussians. Faster than NeRF and editable.

**Factor Graph**: Representing constraints between variables as a graph. The core data structure of SLAM optimization.

**Knowledge Distillation**: A technique for transferring the knowledge of a large model (teacher) to a small model (student).

## B. Frequently Asked Questions (FAQ)

**Q: Should I learn Python or C++ first?**

A: The core of lab code is C++. SLAM, ROS packages, and real-time control modules are all written in C++, and you will frequently read and modify this code. Python is used for deep-learning scripts and data preprocessing, but AI coding agents handle this area well, so the need to master it yourself has diminished. For both, "the ability to read and understand code" is the core, and writing can be done in collaboration with AI.

**Q: Can I do research without a GPU?**

A: Simple experiments are possible on CPU. But a GPU is essential for deep-learning training. Use Google Colab (free) or the lab server. The free version of Colab is enough for something like YOLO fine-tuning.

**Q: Should I learn ROS1 or ROS2?**

A: If you are learning from scratch, ROS2 is recommended. ROS1's official support ended (EOL) in 2025. However, if the package you want to use only supports ROS1, you may have no choice but to learn ROS1 first. That said, knowing ROS1 makes ROS2 quick to pick up.

**Q: Where do I find papers?**

A: Use [arXiv](https://arxiv.org/) (free preprint server), [Google Scholar](https://scholar.google.com/) (paper search), and [Papers With Code](https://paperswithcode.com/) (paper search with code). If you want to browse by venue, [CVPR Open Access](https://openaccess.thecvf.com/) and [IEEE Xplore](https://ieeexplore.ieee.org/) are also useful.

**Q: Where should I start to study SLAM?**

A: Start with Cyrill Stachniss's [YouTube SLAM lectures](https://www.youtube.com/playlist?list=PLgnQpQtFTOGQrZ4O5QzbIHgl3b1JHimN_) and analyze the ORB-SLAM3 code. Reading Ch.9 (camera models) and Ch.14 (visual odometry) of this document beforehand will make the lectures much easier to follow.

**Q: How do I find research ideas?**

A: Read the Limitations section of recent conference papers. You can get ideas from unresolved problems. Another approach: combining two fields that have not yet been merged, like "3D Gaussian Splatting + Semantic SLAM".

**Q: Which GPU should I buy?**

A: The most important spec in GPU selection is **VRAM**. If the model does not fit in VRAM, you cannot run it at all. Next is compute speed (TFLOPS), which directly affects training time.

**Personal (desktop)**

| VRAM | Card | FP32 TFLOPS | FP16 TFLOPS | Use case |
|------|------|-------------|-------------|------|
| 8GB | RTX 4060 | 15.1 | 15.1 | YOLOv8, ResNet training, small-scale fine-tuning |
| 8GB | RTX 5060 Ti 8GB | ~30 | ~30 | 4070-class compute, but 8GB VRAM is tight for VFMs |
| 12GB | RTX 3060 12GB | 12.7 | 12.7 | Compute is slow, but 12GB VRAM is unique at this price point. Cheap on the used market. Student entry-level |
| 12GB | RTX 4070 | 29.1 | 29.1 | Depth Anything, SegFormer. SAM is just barely possible at batch 1 |
| 16GB | RTX 5060 Ti 16GB | ~30 | ~30 | SAM, DINOv2 inference. Mid-scale training. **Best value recommendation** |
| 16GB | RTX 4070 Ti Super | 44.1 | 44.1 | Same VRAM as above, 1.5× compute speed |
| 24GB | RTX 3090 (used) | 35.6 | 35.6 | Cheapest way to secure 24GB of VRAM. Training works, just slow |
| 24GB | RTX 4090 | 82.6 | 82.6 | Top of the personal tier. VLA fine-tuning, large 3DGS scenes |
| 32GB | RTX 5090 | 104.8 | 209.6 | Maximum VRAM for personal use. 2.5× the 4090 in FP16 |

**Server/lab (data center)**

| VRAM | Card | FP32 TFLOPS | TF32 TFLOPS | BF16 TFLOPS | Characteristics |
|------|------|-------------|-------------|-------------|------|
| 16/32GB | V100 SXM2 | 15.7 | — | — | 1st-gen Tensor Cores. No TF32. Still active in many labs. Can be bought cheaply used |
| 24GB | A10 | 31.2 | 62.5 | 125.0 | For inference servers. Slow for training |
| 40/80GB | A100 SXM | 19.5 | 156 | 312 | FP32 is slow, but overwhelming in TF32/BF16. Multi-GPU scaling via NVLink |
| 80GB | H100 SXM | 66.9 | 989 | 1979 | 6× A100 in TF32, 6× in BF16. Transformer Engine support |
| 80GB | H200 SXM | 66.9 | 989 | 1979 | Same compute as H100, 1.5× memory bandwidth via HBM3e |
| 141GB | B200 | 90 | 2250 | 4500 | Latest. Can train 70B+ models on a single GPU |

How to read the numbers:
- **FP32**: Traditional floating point. Used in OpenCV, classical SLAM, etc. For personal GPUs, this number reflects actual performance.
- **TF32/BF16**: Applied when using `torch.cuda.amp` (mixed precision) in PyTorch. Training speed increases 2–6×. Data-center GPUs (A100, H100) show their true strength in this mode, so do not look only at FP32 TFLOPS and conclude "A100 is slower than a 4090?".
- **TFLOPS**: Tera Floating Point Operations Per Second. Higher is faster.

**Notes**:
- The RTX 5060 Ti comes in 8GB and 16GB versions. Be sure to buy the 16GB. The 8GB runs out of VRAM and hits a wall quickly.
- AMD GPUs (RX 7900 XTX, etc.) have improving ROCm support, but compatibility issues with the CUDA ecosystem remain. If you do not want to spend time troubleshooting, buy NVIDIA.
- If the lab server has an A100/H100, your personal GPU is for debugging/prototyping. Check the server specs first.
- Used RTX 3090 (24GB) offers the best price per VRAM. The high power draw (350W) and loud noise have to be factored in.
- You can also use an A100 by the hour on Google Colab Pro (\$10/month). Worth trying before buying a GPU.

**Q: How many papers should I read per day?**

A: A standard like "N papers per day" is meaningless. At the start, *thoroughly* understanding 1 paper per week is far better. Follow the three-pass method from §20.4 and dig deeply into a single paper. After about 6 months, even glancing at the abstract will give you a sense of "ah, this is that kind of paper". From then on the pace picks up. For reference, reading a paper for a lab-meeting presentation and reading one for your own research are different in depth. The latter requires analyzing the code too.

**Q: I am not good at coding — can I still do research?**

A: As of 2026, the ability to *make good use of coding agents (Claude, Copilot, etc.)* has become more important than the ability to write code line by line yourself. Tell the agent "build me a KITTI dataset loader" or "add wandb logging to this training loop" and the code appears. Time spent typing directly has dropped sharply.

That said, to judge whether the agent's code is right or wrong, you need domain knowledge. "Why this DataLoader is slow when num_workers is 0", "why this loss is coming out NaN", "why the coordinate frame in this SLAM code is flipped" — the agent cannot catch these on its own (see Ch.14). In the end, you need the eye to distinguish why good code is good and why bad code is bad, and that eye comes from reading a lot of good code.

Recommended approach: Read the code of well-known open source (ORB-SLAM3, Ultralytics, HuggingFace Transformers, etc.) and understand "why it was written this way". Coding skill is not typing speed but the ability to read code and make judgments.

**Q: How do I prepare a conference presentation?**

A: Conference presentations are largely divided into **oral presentations** and **poster presentations**.

- **Poster**: Most first presentations are posters. You summarize the research on an A0-size sheet. The key is big figures, little text. A passerby should become interested within 3 seconds. For practice, rehearse in front of lab colleagues at least 3 times.
- **Oral**: Usually 15–20 minutes. Keep slides under 20, one message per slide. A demo video is a plus. Prepare supplementary slides for questions.
- Common: Most presentations are in English, so write a script and practice, but do not memorize it. Content delivery matters more than natural English.

**Q: Reading English papers is too hard — what do I do?**

A: This really is something time resolves. A few tips:

- **Grasp the structure first**: Most papers follow Introduction → Related Work → Method → Experiments → Conclusion. Only the Method is genuinely new; the rest follow similar patterns.
- **Learn field-specific vocabulary first**: Expressions like "ablation study", "state-of-the-art", "we empirically show" repeat. After reading about the first 20 papers, you get used to them.
- **Do not be embarrassed to use translation tools**: Translating unknown sentences with DeepL or Google Translate is not embarrassing at all. That said, if you rely only on translation, your English will not improve. Read in the order "original → check translation → back to original".
- **Use the highlighter in your PDF reader**: Coloring key sentences while reading raises concentration. Use whatever tool you prefer — Adobe Acrobat, Zotero's built-in viewer, etc.

## C. Troubleshooting Guide

**Frequently used apt commands**

```bash
sudo apt update                  # refresh the package list
sudo apt upgrade                 # upgrade installed packages
sudo apt install <package>       # install a package
sudo apt remove <package>        # remove a package (keep config files)
sudo apt purge <package>         # fully remove package + config files
sudo apt autoremove              # remove unused dependencies
apt list --installed             # list installed packages
apt search <keyword>             # search for a package
sudo apt --fix-broken install    # recover from broken dependencies
```

(Reference: [Jinyong Jeong's blog](https://jinyongjeong.github.io/2016/06/07/Ubuntu_apt_get_commend/))

**SSH key setup (server access without a password)**

```bash
# Generate a key (hit Enter repeatedly to accept defaults)
ssh-keygen -t ed25519

# Copy the public key to the server
ssh-copy-id user@server_ip

# Connect without a password afterward
ssh user@server_ip
```

Registering the same public key (`~/.ssh/id_ed25519.pub`) on GitHub also removes the need for a password on `git push`. (Reference: [Jinyong Jeong's blog](https://jinyongjeong.github.io/2016/06/02/SSH_keygen_setting/))

**CPU performance mode setup (for experiments)**

In SLAM or deep-learning experiments, CPU throttling sometimes makes performance uneven.

```bash
# Check the current CPU governor
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor

# Switch to performance mode (all cores)
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# Persistent setting (survives reboot)
sudo apt install cpufrequtils
echo 'GOVERNOR="performance"' | sudo tee /etc/default/cpufrequtils
sudo systemctl restart cpufrequtils
```

On laptops, battery drain increases, so use it only while plugged in. (Reference: [Jinyong Jeong's blog](https://jinyongjeong.github.io/2020/02/04/Ubuntu_cpu_freq_change/))

### C.1 CUDA / PyTorch

**Problem**: `CUDA out of memory`

**Solution**:

```python
# 1. Reduce batch size (try this first)
batch_size = 16  # → 8 or 4

# 2. Clear memory
torch.cuda.empty_cache()

# 3. Use gradient accumulation (keep the effective batch, save memory)
accumulation_steps = 4
for i, (inputs, labels) in enumerate(dataloader):
    loss = model(inputs, labels) / accumulation_steps
    loss.backward()
    if (i + 1) % accumulation_steps == 0:
        optimizer.step()
        optimizer.zero_grad()

# 4. Mixed Precision Training (halves memory)
from torch.cuda.amp import autocast, GradScaler
scaler = GradScaler()
with autocast():
    output = model(input)
    loss = criterion(output, target)
```

**Problem**: `CUDA version mismatch`

**Solution**:

```bash
# Check the installed CUDA version
nvcc --version

# Check the CUDA version PyTorch sees
python -c "import torch; print(torch.version.cuda)"

# If they differ, reinstall PyTorch (matching the CUDA version)
pip install torch --index-url https://download.pytorch.org/whl/cu121
```

**Problem**: `RuntimeError: CUDA error: device-side assert triggered`

**Solution**: This usually happens when a label index is out of range. Running on CPU produces a more detailed error message.

```bash
CUDA_LAUNCH_BLOCKING=1 python train.py
```

### C.2 ROS

**Problem**: `Package not found`

**Solution**:

```bash
# Check that the workspace is sourced
source ~/ros2_ws/install/setup.bash

# Check that the package is installed
ros2 pkg list | grep package_name

# Add sourcing to .bashrc (so you do not do it manually every time)
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

**Problem**: `TF tree not connected`

**Solution**:

```bash
# Check the TF tree
ros2 run tf2_tools view_frames

# Add a static transform (example)
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link camera_link
```

**Problem**: `Topic not published` / data is not coming in

**Solution**:

```bash
# List currently active topics
ros2 topic list

# Check data on a specific topic
ros2 topic echo /camera/image_raw --once

# Check for QoS mismatches (common in ROS2)
ros2 topic info /camera/image_raw -v
```

### C.3 Docker

**Problem**: `Permission denied`

**Solution**:

```bash
# Add the user to the docker group
sudo usermod -aG docker $USER
# Log out and log back in
```

**Problem**: GUI programs do not run

**Solution**:

```bash
# X11 forwarding
xhost +local:docker
docker run -it --env DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix ...
```

**Problem**: GPU not detected inside Docker

**Solution**:

```bash
# Install nvidia-container-toolkit
sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker

# Run with the GPU option added
docker run --gpus all -it nvidia/cuda:12.1.0-base-ubuntu22.04 nvidia-smi
```

### C.4 OpenCV

**Problem**: `cv2.imshow() not working`

**Solution**:

```bash
# Remove the headless OpenCV build and reinstall
pip uninstall opencv-python-headless
pip install opencv-python
```

**Problem**: OpenCV conflicts with ROS's cv_bridge

**Solution**:

```bash
# When ROS's cv_bridge references the system OpenCV,
# it can conflict with OpenCV in a conda/venv environment.
# Fix: specify the Python path explicitly when building the ROS workspace.

colcon build --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

### C.5 Build/compile

**Problem**: ORB-SLAM3 build error (OpenCV version conflict)

**Solution**:

```bash
# Some APIs changed in OpenCV 4.x
# Check the OpenCV version in CMakeLists.txt
find_package(OpenCV 4 REQUIRED)

# On Pangolin build errors
sudo apt-get install libglew-dev libpython2.7-dev
```

**Problem**: Eigen version errors

**Solution**:

```bash
# Check the system Eigen version
pkg-config --modversion eigen3

# Install a specific version directly if needed
sudo apt-get install libeigen3-dev
```

## D. Checklist: Things to Confirm Before Starting Research

### D.1 Environment setup

- [ ] Ubuntu installed (22.04 LTS recommended)
- [ ] NVIDIA driver installed (confirm with `nvidia-smi`)
- [ ] CUDA / cuDNN installed (confirm with `nvcc --version`)
- [ ] Conda or venv environment set up
- [ ] PyTorch GPU operation confirmed (`torch.cuda.is_available()`)
- [ ] ROS2 installed (if needed, Humble or Jazzy)
- [ ] Git configured (`git config --global user.name/email`)
- [ ] Docker installed (optional, recommended for reproducibility)
- [ ] VS Code + essential extensions installed (Python, Remote-SSH, Jupyter)

### D.2 Foundational knowledge

- [ ] Python basics (classes, decorators, list comprehensions)
- [ ] NumPy array operations (broadcasting, indexing, reshape)
- [ ] OpenCV image processing (read, transform, filter, keypoints)
- [ ] Linear algebra basics (matrix multiplication, eigenvalue decomposition, SVD)
- [ ] Probability/statistics basics (Bayes' theorem, Gaussian distribution, MLE/MAP)

### D.3 Research tools

*The full list of research tools has been absorbed into [`../../research-notes/part2_writing/`](../../research-notes/part2_writing/) and [`part3_presentations/ch02_conference_prep.md`](../../research-notes/chapter_34_conference_prep.md) *(Korean; English version planned)*. Field-specific application: see §20.4 "Paper-writing tools" and §20.7 recommended learning roadmap.*

### D.4 Dataset preparation

- [ ] Download the dataset relevant to your research
- [ ] Understand the data format (image size, depth units, coordinate frame)
- [ ] Implement a DataLoader (PyTorch Dataset/DataLoader)
- [ ] Write data visualization code (for debugging)

## E. First-Week Survival Guide

When you first join a lab, it is natural to feel lost about what to do. This guide sums up "at minimum, do at least this in your first week".

### Day 1–2: Build the environment

```
[ ] Get a lab server account (ask the admin)
[ ] Confirm SSH access to the server
[ ] Configure VS Code Remote-SSH
[ ] Create a conda environment on the server
[ ] Confirm PyTorch + CUDA work
[ ] Join the lab's GitHub organization
[ ] Join the Slack/Discord channel
```

> Tip: If you get stuck setting up the server environment, ask a senior. Say "I tried X, but got an unexpected result Y", and they will help you almost 100% of the time. If you ask without having tried anything... they may just tell you to figure it out.

### Day 3–4: Get to know the existing code

```
[ ] Clone the lab's existing code/project repositories
[ ] Read the README (if any)
[ ] Try building and running the existing code
[ ] Download datasets and configure paths
[ ] Run a simple demo
```

> Tip: It is normal for the code not to run. Environments differ, paths differ, versions differ. Copy the error message and search Google — most of the time the answer is on Stack Overflow.

### Day 5: Start reading papers

*The meta-skill operations of Day 5 and Day 6–7 (asking for paper recommendations, getting the research direction, preparing a self-introduction) are treated in depth in [`../../grad-notes/chapter_04_two_way_relationship.md`](../../grad-notes/chapter_04_two_way_relationship.md) (Day 5) and [`p3_ch01_my_research.md`](../../grad-notes/chapter_07_my_research.md) (the Day 1–7 sequence) *(Korean; English version planned)*.*

> Tip: It is normal not to understand a paper on the first read. Even grasping just "what problem is this paper trying to solve?" is enough for the first week.

### Day 6–7: Get the research direction

*See the link above. Field-specific one-liner: skim Ch.18 of this document (the lab's research directions) and map the lab's recent papers/projects to the seniors' topics.*

### Things you do not need to do in the first week

- Understand papers perfectly — time will take care of this
- Grasp every latest research trend — gradually
- Write code from scratch — start by modifying existing code
- Set up the GPU server perfectly — copy a senior's environment
- Come up with research ideas — expect at least 1–2 months of learning time

### Mindset for survival

The general survival mindset (not-knowing is normal, leveraging seniors, writing it down, starting small, not comparing) is treated in the meta-skill guide — see [`../../research-notes/part0_starting/`](../../research-notes/part0_starting/) (5 chapters on entering research) *(Korean; English version planned)*.

Field-specific application: in SLAM/CV/robotics labs the *senior code* is the highest-leverage starting point — copy the senior's environment, run their pipeline first, and only then begin modifying. Setup pain in robotics is high (CUDA · ROS · simulator versions); the time saved by not reinventing it goes straight into reading papers.
