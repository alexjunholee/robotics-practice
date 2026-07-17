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

A: C++ is common in SLAM, ROS packages, and real-time control modules, so researchers need it to read and modify lab code. Python is used mainly for deep-learning scripts and data preprocessing. AI coding agents can assist with writing either language, but researchers must still understand existing code and verify its behavior.

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

A: First measure whether the model, optimizer state, activations, and batch fit in **VRAM** under the intended configuration. Then consider precision support, memory bandwidth, power, framework compatibility, and an application-level benchmark. The table narrows candidates by memory class; it is not a purchase ranking.

**Personal (desktop)**

| VRAM | Example cards | Role to evaluate | Check before buying |
|------|---------------|------------------|---------------------|
| 8GB | RTX 4060, RTX 5060 Ti 8GB | Small-CNN training, inference with a limited batch | Peak memory of the intended model; VFM fine-tuning may not fit |
| 12GB | RTX 3060 12GB, RTX 4070 | Mid-sized inference and training experiments | Generation-specific runtime and used-card condition |
| 16GB | RTX 5060 Ti 16GB, RTX 4070 Ti Super | Larger batches, VFM inference, mid-scale training | Model-specific activation and optimizer memory |
| 24GB | RTX 3090, RTX 4090 | Training that fits within 24GB, 3DGS and VLA experiments | Power, cooling, used warranty, and runtime differences |
| 32GB | RTX 5090 | Local experiments that exceed 24GB | Power, case, PSU, and software support |

**Server/lab (data center)**

| GPU memory | Card | Characteristics | Official specifications |
|------------|------|-----------------|-------------------------|
| 16/32GB | V100 SXM2 | First-generation Tensor Cores; no TF32 or BF16 | [V100 data center GPU](https://www.nvidia.com/en-us/data-center/v100/) |
| 24GB | A10 | PCIe inference and graphics family | [A10 datasheet](https://www.nvidia.com/content/dam/en-zz/Solutions/Data-Center/a10/pdf/a10-datasheet.pdf) |
| 40/80GB | A100 | TF32, BF16, MIG, PCIe/SXM variants | [A100 specifications](https://www.nvidia.com/en-us/data-center/a100/) |
| 80GB | H100 SXM | Hopper, Transformer Engine, NVLink | [H100 specifications](https://www.nvidia.com/en-us/data-center/h100/) |
| 141GB | H200 SXM | 141GB HBM3e and 4.8TB/s memory bandwidth | [H200 specifications](https://www.nvidia.com/en-us/data-center/h200/) |
| 180GB | B200 | Blackwell and 180GB HBM3e; delivered in server configurations | [DGX B200 specifications](https://www.nvidia.com/en-us/data-center/dgx-b200/) |

TFLOPS denotes trillions of floating-point operations per second at a stated precision; it is a theoretical peak, not an application benchmark. FP32 is the conventional 32-bit format, while TF32, BF16, FP16, and FP8 trade precision and range differently for accelerated tensor operations. When reading specifications, distinguish precision, CUDA cores from Tensor Cores, dense from structured-sparsity figures, and PCIe from SXM variants. Equal peak TFLOPS does not imply equal training time because memory bandwidth, kernels, batch size, and data loading differ. The benefit of `torch.amp` is also model- and hardware-dependent, so compare cards with a short run of the same repository, batch, and precision.

**Notes**:
- For cards such as the RTX 5060 Ti that come with 8GB and 16GB options, estimate the VRAM required by the intended model and batch. Eight gigabytes can be restrictive for local VFM work.
- When considering an AMD GPU, check whether the required frameworks and libraries support ROCm. CUDA-only dependencies add migration cost.
- If the lab server has an A100 or H100, a personal GPU may serve mainly for debugging and prototyping. Check the server specifications and availability before purchasing.
- A used RTX 3090 is a 24GB option, but price, warranty, and cooling condition vary by listing. Check rated power and the PSU and case requirements.
- Colab and cloud-GPU prices, assigned GPU types, and usage limits change. Benchmark the real workload on a rented GPU before buying, but verify the current price and quota on the provider page.

**Q: How many papers should I read per day?**

A: The purpose and depth of the reading matter more than a daily paper count. At first, reading one paper carefully each week with the three-pass method in §20.4 can be more useful. With experience, the abstract alone becomes enough to judge a paper's type and relevance. Reading for a lab meeting also differs from reading for one's own research, which may extend to code analysis.

**Q: I am not good at coding — can I still do research?**

A: Coding agents such as Claude and Copilot can quickly draft requests such as "build a KITTI dataset loader" or "add wandb logging to this training loop." They reduce the time spent typing, but the output still needs review.

Judging generated code still requires domain knowledge. An agent may not reliably identify why a DataLoader is slow with `num_workers=0`, why a loss becomes NaN, or where a coordinate frame is reversed in SLAM code (see Ch.14). Run the code and compare it with existing implementations before accepting the result.

Reading projects such as ORB-SLAM3, Ultralytics, and HuggingFace Transformers and tracing their design choices helps develop code-review skills.

**Q: How do I prepare a conference presentation?**

A: Conference presentations are largely divided into **oral presentations** and **poster presentations**. Talk length, poster dimensions, and presentation language vary by venue, so the official presenter instructions take precedence.

- **Poster**: A0 is a common size, but check the venue's specification. Use large figures and little text so a passerby can locate the topic and result quickly. Rehearse in front of lab colleagues.
- **Oral**: Fifteen to twenty minutes is a common example, not a rule; the session limit comes first. Set the slide count from that limit and keep one message per slide. Prepare a demo video and supplementary slides when they help.
- **Common**: Confirm the presentation language, then rehearse from a script while prioritizing delivery and timing over memorization.

**Q: Reading English papers is too hard — what do I do?**

A: Repeated reading and accumulated domain knowledge reduce the burden. A few practical steps help:

- **Grasp the structure first**: Many experimental papers use Introduction → Related Work → Method → Experiments → Conclusion, but contributions can also lie in problem formulation, data, evaluation, or analysis. Use the title and headings to identify the paper's actual structure.
- **Learn field-specific vocabulary first**: Expressions like "ablation study", "state-of-the-art", and "we empirically show" recur. The number of papers needed for familiarity depends on the reader's background and field.
- **Do not be embarrassed to use translation tools**: Translating unknown sentences with DeepL or Google Translate is not embarrassing at all. That said, if you rely only on translation, your English will not improve. Read in the order "original → check translation → back to original".
- **Use the highlighter in your PDF reader**: Marking important sentences makes them easier to find again. Use whichever tool is comfortable, such as Adobe Acrobat or Zotero's built-in viewer.

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

See [Research Notes Part 2](../../research-notes/guide.html#chapter-16) for tools used in reading and writing papers, and [Research Notes Ch.34](../../research-notes/guide.html#chapter-34) for conference preparation *(Korean only)*. Sections 20.4 and 20.7 cover tools and learning paths specific to Spatial AI.

### D.4 Dataset preparation

- [ ] Download the dataset relevant to your research
- [ ] Understand the data format (image size, depth units, coordinate frame)
- [ ] Implement a DataLoader (PyTorch Dataset/DataLoader)
- [ ] Write data visualization code (for debugging)

## E. First-Week Survival Guide

The first week needs a minimum task list for preparing accounts and the execution environment, then locating the code, data, and documents for the research topic. The Day 1–7 allocation below is an example; reorder it around account provisioning, equipment schedules, and the lab's onboarding process.

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

> Tip: When asking a senior about the server environment, include the command you tried, the expected result, and the actual output. This information makes the problem much faster to narrow down.

### Day 3–4: Get to know the existing code

```
[ ] Clone the lab's existing code/project repositories
[ ] Read the README (if any)
[ ] Try building and running the existing code
[ ] Download datasets and configure paths
[ ] Run a simple demo
```

> Tip: Code often fails on its first run because environments, paths, and versions differ. Use the error message to check the official documentation and issue tracker before changing the setup.

### Day 5: Start reading papers

[Grad Notes Ch.4](../../grad-notes/guide.html#chapter-4) discusses how to ask for a first paper recommendation and talk with lab members, while [Grad Notes Ch.7](../../grad-notes/guide.html#chapter-7) covers setting a research direction during the first week *(Korean only)*.

> Tip: It is normal not to understand a paper on the first read. Even grasping just "what problem is this paper trying to solve?" is enough for the first week.

### Day 6–7: Get the research direction

Read Ch.18 and classify the lab's recent papers and projects by research topic. Mark where they overlap with the work of senior lab members.

### Things you do not need to do in the first week

- Understand papers perfectly — time will take care of this
- Grasp every latest research trend — gradually
- Write code from scratch — start by modifying existing code
- Set up the GPU server perfectly — begin from an environment file, container, or installation procedure the lab has already verified
- Produce a fully formed research idea — it is fine to learn the lab's problems and tools first

### Mindset for survival

Research Notes and Grad Notes discuss the habits needed at the beginning of a project. The individual links preserve the five decisions summarized in the Korean edition *(linked chapters are Korean only)*.

- *Not knowing at first is expected* → [Grad Notes Ch.14 — The Weight of Autonomy](../../grad-notes/guide.html#chapter-14), §2
- *"It does not work" is not a report; give prediction, attempt, and result* → [Grad Notes Ch.10 — One Question per Email](../../grad-notes/guide.html#chapter-10), §3
- *Keep records that let your future self reconstruct the work* → [Grad Notes Ch.8 — Using Time](../../grad-notes/guide.html#chapter-8), §5
- *Start from a small code path* → [Grad Notes Ch.11 — The Tool Trap](../../grad-notes/guide.html#chapter-11), §1
- *Compare against your past work, not a peer's current position* → [Grad Notes Ch.15 — The Comparison Trap](../../grad-notes/guide.html#chapter-15), §3

In a SLAM, CV, or robotics lab, begin with an existing project from a senior member. Reproduce its environment and run the pipeline before modifying it. Reusing a known CUDA, ROS, and simulator configuration reduces setup time and leaves more time for understanding the method.
