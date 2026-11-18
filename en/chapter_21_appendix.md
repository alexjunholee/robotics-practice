# Ch.21 — Appendix

This appendix is a quick-reference toolkit for day-to-day research and engineering. Keep it handy when you run into an unfamiliar acronym (A), a practical question about starting out (B), a broken build or CUDA error (C), pre-project setup checks (D), or your first week in a new lab (E).

## A. Glossary

Robotics and Spatial AI papers move quickly across geometry, deep learning, and systems engineering. Use the tables and definitions below whenever an acronym or core concept needs a quick refresher.

### A.1 Abbreviations

The table below gathers the abbreviations that appear most often across papers, codebases, and lab discussions.

| Abbr. | Expansion | Description |
| --- | --- | --- |
| SLAM | Simultaneous Localization and Mapping | Jointly estimating pose and building a map of an unknown environment |
| VO | Visual Odometry | Estimating camera motion from consecutive image frames |
| VIO | Visual-Inertial Odometry | Motion estimation fusing camera frames with IMU measurements |
| LIO | LiDAR-Inertial Odometry | Motion estimation fusing LiDAR scans with IMU measurements |
| IMU | Inertial Measurement Unit | Sensor package measuring linear acceleration and angular velocity |
| DoF | Degrees of Freedom | Number of independent parameters defining a system's pose or motion |
| SE(3) | Special Euclidean Group (3D) | Mathematical group representing 3D rigid-body transformations |
| SO(3) | Special Orthogonal Group (3D) | Mathematical group representing 3D rotations |
| FoV | Field of View | Angular extent of the scene observable by a sensor |
| ToF | Time of Flight | Distance-measurement technique based on light travel time |
| CNN | Convolutional Neural Network | Deep neural architecture built around spatial convolutions |
| ViT | Vision Transformer | Transformer architecture applied directly to image patches |
| VFM | Vision Foundation Model | Large-scale pretrained vision model transferable across tasks |
| VLA | Vision-Language-Action | Multimodal model mapping vision and language directly to robot actions |
| VLM | Vision-Language Model | Multimodal model reasoning jointly over images and text |
| LLM | Large Language Model | Large-scale transformer model trained on text |
| mAP | mean Average Precision | Standard object-detection metric averaged across classes |
| ICP | Iterative Closest Point | Iterative algorithm for aligning two 3D point clouds |
| NDT | Normal Distributions Transform | Point cloud registration using voxelized Gaussian distributions |
| NeRF | Neural Radiance Fields | Implicit 3D scene representation rendered via volume ray marching |
| 3DGS | 3D Gaussian Splatting | Explicit 3D scene representation using rasterized 3D Gaussians |
| BEV | Bird's Eye View | Top-down spatial representation projected onto the ground plane |
| TSDF | Truncated Signed Distance Function | Volumetric 3D surface representation storing signed distances near surfaces |
| BA | Bundle Adjustment | Joint nonlinear optimization of camera poses and 3D landmarks |
| PGO | Pose Graph Optimization | Global optimization over relative pose constraints between keyframes |
| DDS | Data Distribution Service | Publish-subscribe communication middleware underlying ROS2 |
| ONNX | Open Neural Network Exchange | Portable intermediate format for exchanging neural network models |
| TRT | TensorRT | NVIDIA's inference engine for optimizing and deploying models on GPUs |
| ATE | Absolute Trajectory Error | Global trajectory accuracy metric comparing estimated and true poses |
| RPE | Relative Pose Error | Local drift metric comparing relative motion over fixed intervals |

### A.2 Terms

Beyond abbreviations, the concepts below carry specific geometric or learning meanings that are easy to blur on a first read.

**Keyframe**: A representative frame selected as a reference for tracking and mapping. Processing every incoming frame is computationally prohibitive, so SLAM systems retain only frames where the camera has moved enough or the scene has changed meaningfully.

**Loop Closure**: Correcting accumulated drift by recognizing a previously visited place. When the robot realizes it has returned to a known spot, it aligns the current map with the earlier visit and distributes the accumulated error across the trajectory.

**Drift**: The gradual accumulation of small estimation errors over time or distance. Walk 100 m at a 1 m stride with 1 cm of error per step: if every error points in the same direction, you end up 100 cm off target; if the errors are uncorrelated across steps, the error grows with the square root of the step count, leaving roughly 10 cm of drift.

**Reprojection Error**: The pixel distance between where a known 3D point is predicted to project onto an image plane and where it is actually observed. Minimizing reprojection error is the objective behind bundle adjustment.

**Feature Descriptor**: A numerical vector summarizing the local appearance around a keypoint. Comparing descriptor distances lets algorithms match the same physical 3D point across two images.

**Homography**: A 2D projective transformation relating two views of the same planar surface—used, for instance, to align two photos of a tabletop or building facade taken from different angles.

**Essential Matrix**: The 3×3 matrix encoding the epipolar geometry between two calibrated cameras, carrying 5 DoF (3 for relative rotation and 2 for translation direction up to scale).

**Fundamental Matrix**: The 3×3 matrix encoding the epipolar geometry between two uncalibrated cameras directly in pixel coordinates, carrying 7 DoF.

**Epipole**: The image point where one camera's optical center projects onto the other camera's image plane.

**Zero-shot**: Performing a new task with no task-specific training examples—such as querying a model for *"cat"* when *"cat"* never appeared in fine-tuning labels.

**Few-shot**: Adapting to a new task from only a handful of examples, typically 3–5 labeled instances.

**Fine-tuning**: Adapting a pretrained model to a specific downstream task or dataset by continuing training with your own data.

**Domain Adaptation**: Bridging a distribution shift between a source domain and a target domain, such as transferring a policy trained in simulation to a physical environment.

**Sim-to-Real**: Transferring models or policies learned in a simulator onto physical hardware—the canonical domain-adaptation challenge in robotics.

**Gaussian Splatting**: A method that represents a 3D scene as millions of anisotropic 3D Gaussians rendered via fast rasterization, offering much faster rendering and easier editing than NeRF.

**Factor Graph**: A bipartite graph encoding probabilistic constraints (factors) between state variables—the standard mathematical representation behind modern SLAM optimization.

**Knowledge Distillation**: Training a compact student model to reproduce the outputs or internal representations of a larger teacher model.

## B. Frequently Asked Questions (FAQ)

These questions come up repeatedly when students start out in a Spatial AI or robotics lab.

**Q: Should I learn Python or C++ first?**

A: Start with whichever language the lab codebase you need to run uses first. C++ dominates SLAM, ROS packages, and real-time control loops, so you will need enough fluency to read, build, and modify existing lab code. Python drives most deep-learning training scripts and data preprocessing pipelines. AI coding agents can accelerate writing in either language, but you still need to read existing code and verify what it actually does.

**Q: Can I do research without a GPU?**

A: Small prototypes and classical geometry experiments run fine on a CPU, though training deep-learning models almost always demands a GPU. When you do not have a local GPU, start with the free tier of Google Colab or your lab's shared server; Colab's free GPUs are plenty for tasks like fine-tuning YOLO.

**Q: Should I learn ROS1 or ROS2?**

A: If you are starting from scratch, go straight to ROS2—official support for ROS1 reached end-of-life (EOL) in 2025. The main exception is if a legacy package your project depends on exists only in ROS1; in that case, learn what you need for ROS1 first, and you will find that picking up ROS2 afterward is straightforward.

**Q: Where do I find papers?**

A: Start with [arXiv](https://arxiv.org/) for free preprints and [Google Scholar](https://scholar.google.com/) for citation tracking and literature search. When you also want public code, look for the repository link inside the paper or check [Hugging Face Papers](https://huggingface.co/papers/trending) (Papers With Code shut down in 2025). To browse proceedings by conference, [CVPR Open Access](https://openaccess.thecvf.com/) and [IEEE Xplore](https://ieeexplore.ieee.org/) are reliable starting points.

**Q: Where should I start to study SLAM?**

A: Work through Cyrill Stachniss's [YouTube SLAM lectures](https://www.youtube.com/playlist?list=PLgnQpQtFTOGQrZ4O5QzbIHgl3b1JHimN_) alongside reading the ORB-SLAM3 codebase. Going through Ch.9 (camera models) and Ch.14 (visual odometry) of this guide beforehand makes the lectures much easier to follow.

**Q: How do I find research ideas?**

A: Read the Limitations and Future Work sections of recent conference papers to spot unsolved bottlenecks. Another productive angle is bridging two subfields that have not yet been cleanly unified—though combining two buzzwords by name alone does not make an idea novel. Several papers on *"3D Gaussian Splatting + Semantic SLAM"* already exist, and Ch.18 of this guide also discusses Gaussian maps carrying semantic features. Survey prior work carefully first, then look for the specific gap that remains open.

**Q: Which GPU should I buy?**

A: Begin by measuring whether your target model, optimizer state, activations, and batch size actually fit in **VRAM** under your intended training or inference setup. After VRAM capacity, weigh precision support, memory bandwidth, power draw, framework compatibility, and an actual benchmark on your workload. The tables below group candidates by memory tier rather than ranking them for purchase.

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

Keep in mind that TFLOPS measures theoretical peak floating-point throughput at a stated precision, not real-world training speed. While FP32 is the standard 32-bit format, TF32, BF16, FP16, and FP8 make different trade-offs between numerical range and precision to accelerate tensor operations. When comparing spec sheets, separate precision modes, CUDA cores from Tensor Cores, dense from structured-sparsity figures, and PCIe from SXM variants. Two GPUs with identical peak TFLOPS can still train at very different speeds depending on memory bandwidth, kernel support, batch size, and data loading. Even `torch.amp` speedups vary by model and architecture, so test candidates with a short run of the exact repository, batch size, and precision you intend to use.

**Notes**:
- For GPUs sold in both 8GB and 16GB variants (such as the RTX 5060 Ti), estimate the peak VRAM of your model and batch beforehand—8GB quickly becomes restrictive for local VFM experiments.
- If you are considering an AMD GPU, confirm ROCm support across every framework and library you rely on; CUDA-only dependencies add non-trivial porting work.
- When your lab already provides A100 or H100 nodes, a personal desktop GPU is mainly useful for interactive debugging and prototyping—check server capacity and queue availability before spending your budget.
- A used RTX 3090 offers an economical 24GB option, though pricing, warranty, and thermal wear vary widely; verify rated power draw along with your PSU and case clearance.
- Cloud GPU and Colab pricing, assigned hardware, and usage quotas shift frequently. Renting an instance briefly to benchmark your real workload before buying hardware is smart—just confirm current rates and quotas directly on the provider page.

**Q: How many papers should I read per day?**

A: Reading depth and purpose matter far more than hitting a daily quota. Early on, working carefully through one paper per week using the three-pass method in §20.4 builds far stronger instincts than skimming dozens. Over time, reading just the abstract becomes enough to classify a paper's type and relevance. How deeply you read also depends on context: preparing for a lab seminar requires a different level of detail than dissecting a paper to modify its code for your own research.

**Q: I am not good at coding — can I still do research?**

A: Coding agents like Claude and Copilot can rapidly scaffold tasks such as *"write a KITTI dataset loader"* or *"add wandb logging to this training loop,"* cutting down boilerplate typing—though you still have to inspect every line they produce.

Reviewing generated code requires domain judgment. An agent may miss why a DataLoader bottlenecks with `num_workers=0`, why training loss collapses to NaN, or where a coordinate convention is inverted inside SLAM code (see Ch.14). Always run the code on known inputs and cross-check it against trusted reference implementations before accepting it.

Reading well-engineered open-source codebases such as ORB-SLAM3, Ultralytics, and HuggingFace Transformers—and tracing why they are structured the way they are—is one of the best ways to sharpen that review instinct.

**Q: How do I prepare a conference presentation?**

A: Conference talks generally fall into **oral presentations** and **poster presentations**. Because session lengths, poster board dimensions, and presentation languages vary by conference, always follow the venue's official presenter guidelines first.

- **Poster**: A0 is a common format, though you should confirm the exact dimensions in the presenter guide. Favor large figures and crisp visual takeaways over dense paragraphs so passersby can grasp the problem and main result in seconds, and rehearse your walk-through with labmates beforehand.
- **Oral**: Fifteen to twenty minutes is a typical slot, not a universal rule—plan strictly around your assigned time limit. Size your slide deck to that budget, keep one clear message per slide, and prepare backup demo videos or appendix slides for likely Q&A points.
- **For both**: Confirm the presentation language early, then rehearse from a written script while focusing on pacing, clarity, and timing rather than word-for-word memorization.

**Q: Reading English papers is too hard — what do I do?**

A: It gets noticeably easier once domain familiarity builds up alongside reading mileage. A few practical habits speed up that curve:

- **Map the structure first**: Many empirical papers follow Introduction → Related Work → Method → Experiments → Conclusion, though contributions can also center on problem formulation, datasets, evaluation protocols, or analysis. Scan the title and section headings first to see the paper's real outline.
- **Learn recurring phrasing early**: Terms like *"ablation study"*, *"state-of-the-art"*, and *"we empirically show"* appear in nearly every paper. How quickly they feel natural varies with your background and subfield, so give yourself a few weeks of steady reading.
- **Use translation tools without guilt**: Leaning on DeepL or Google Translate for tricky sentences is completely fine; just avoid reading *only* the translation if you want your English fluency to grow. Follow a loop of *"original English → check translation → reread the original."*
- **Highlight as you read**: Marking key claims in your PDF reader makes revisiting papers weeks later effortless. Use whichever viewer feels comfortable, such as Adobe Acrobat or Zotero's built-in reader.

## C. Troubleshooting Guide

Keep these quick fixes nearby when package managers, SSH keys, CUDA runtimes, ROS2 graphs, or Docker containers stall an experiment.

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

Adding that same public key (`~/.ssh/id_ed25519.pub`) to your GitHub account also eliminates password prompts on `git push` for SSH remotes (`git@github.com:...`). Note that repositories cloned over HTTPS rely on personal access tokens or a credential helper instead, so this SSH key does not apply to HTTPS remotes. (Reference: [Jinyong Jeong's blog](https://jinyongjeong.github.io/2016/06/02/SSH_keygen_setting/))

**CPU performance mode setup (for experiments)**

In SLAM and deep-learning benchmarks, aggressive CPU frequency scaling can introduce erratic latency spikes. Switch the governor to `performance` during timed runs:

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

On laptops, performance mode drains the battery quickly, so enable it only when plugged into AC power. (Reference: [Jinyong Jeong's blog](https://jinyongjeong.github.io/2020/02/04/Ubuntu_cpu_freq_change/))

### C.1 CUDA / PyTorch

**Problem**: `CUDA out of memory`

**Solution**: Lower batch size first, clear cached allocations if needed, or combine gradient accumulation with automatic mixed precision (AMP):

```python
import torch

# 1. Reduce batch size (try this first)
batch_size = 16  # → 8 or 4

# 2. Clear memory
torch.cuda.empty_cache()

# 3. Gradient accumulation + AMP
accumulation_steps = 4
num_batches = len(dataloader)
scaler = torch.amp.GradScaler("cuda")
optimizer.zero_grad()

for i, (inputs, labels) in enumerate(dataloader):
    group_start = (i // accumulation_steps) * accumulation_steps
    group_size = min(accumulation_steps, num_batches - group_start)

    with torch.autocast(device_type="cuda", dtype=torch.float16):
        output = model(inputs)
        loss = criterion(output, labels) / group_size

    scaler.scale(loss).backward()

    is_update_step = ((i + 1) % accumulation_steps == 0) or ((i + 1) == num_batches)
    if is_update_step:
        scaler.step(optimizer)
        scaler.update()
        optimizer.zero_grad()
```

Memory savings and speedups from AMP depend on the model, GPU generation, and specific operators used, so check both peak VRAM and numerical stability with the exact same batch and model.

**Problem**: `CUDA version mismatch` (mostly when building CUDA extensions yourself)

**Solution**:

```bash
# Check the installed CUDA version
nvcc --version

# Check the CUDA version PyTorch sees
python -c "import torch; print(torch.version.cuda)"

# The two values may differ, and that is normal. The wheel bundles its own CUDA
# runtime, so if the driver supports that runtime, leave the install alone.
# The exception is building CUDA extensions yourself: then the major versions of
# nvcc and torch.version.cuda must match, and only then should you reinstall.
pip install torch --index-url https://download.pytorch.org/whl/cu121
```

**Problem**: `RuntimeError: CUDA error: device-side assert triggered`

**Solution**: This error typically happens when a class index or tensor index falls out of range. Switching briefly to CPU often surfaces a clearer traceback; alternatively, setting `CUDA_LAUNCH_BLOCKING=1` forces synchronous kernel execution on the GPU so the stack trace points directly to the offending operation:

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
# Install nvidia-container-toolkit — register the NVIDIA apt repository and key first, or the package will not be found
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
  sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker   # Register the hook with the Docker runtime
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

# Install the Eigen development package provided by the repository
sudo apt-get install libeigen3-dev
```

## D. Checklist: Things to Confirm Before Starting Research

Run through these four checklists before diving into your first real experiment so environment mismatches do not derail you mid-week.

### D.1 Environment setup

- [ ] Ubuntu installed (22.04 LTS recommended)
- [ ] NVIDIA driver installed (verify with `nvidia-smi`)
- [ ] CUDA toolkit installed (verify with `nvcc --version`)
- [ ] cuDNN installed (verify a system install via `cudnn_version.h`; `torch.backends.cudnn.version()` reports the cuDNN version bundled with PyTorch)
- [ ] Conda or venv environment configured
- [ ] PyTorch GPU execution verified (`torch.cuda.is_available()`)
- [ ] ROS2 installed if needed (Humble on Ubuntu 22.04, Jazzy on 24.04)
- [ ] Git identity configured (`git config --global user.name/email`)
- [ ] Docker installed (optional, though strongly recommended for reproducibility)
- [ ] VS Code and core extensions installed (Python, Remote-SSH, Jupyter)

### D.2 Foundational knowledge

- [ ] Python fundamentals (classes, decorators, list comprehensions)
- [ ] NumPy array operations (broadcasting, indexing, reshaping)
- [ ] OpenCV image processing (reading, geometric transforms, filtering, keypoints)
- [ ] Linear algebra essentials (matrix multiplication, eigendecomposition, SVD)
- [ ] Probability and statistics essentials (Bayes' theorem, Gaussian distributions, MLE/MAP)

### D.3 Research tools

For paper-reading tools (note templates and citation workflows), see [Research Notes Ch.15](../../research-notes/guide.html#chapter-15); for manuscript-writing tools, see [Research Notes Part 2](../../research-notes/guide.html#chapter-16); and for conference preparation, see [Research Notes Ch.34](../../research-notes/guide.html#chapter-34) *(Korean only)*. Sections 20.4 and 20.7 cover tools and learning paths tailored specifically to Spatial AI.

### D.4 Dataset preparation

- [ ] Download the dataset required for your task
- [ ] Inspect the data layout (image dimensions, depth scale and units, coordinate conventions)
- [ ] Implement a clean `Dataset`/`DataLoader` pipeline in PyTorch
- [ ] Write quick visualization utilities to sanity-check inputs and labels

## E. First-Week Survival Guide

Your first week in a new lab calls for a focused checklist: get your accounts and compute environment working, then locate the codebase, datasets, and documentation tied to your research area. The Day 1–7 schedule below is an example—reorder it around account provisioning, hardware access, and your lab's onboarding routine.

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

> **Tip**: Whenever you ask a senior lab member for help with server setup, include the exact command you ran, what you expected to happen, and the full output or error you got. That context lets them pinpoint the issue in seconds.

### Day 3–4: Get to know the existing code

```
[ ] Clone the lab's existing code/project repositories
[ ] Read the README (if any)
[ ] Try building and running the existing code
[ ] Download datasets and configure paths
[ ] Run a simple demo
```

> **Tip**: Code rarely runs cleanly on the first try because environments, file paths, and library versions differ across machines. Read the error message carefully and check the official documentation or issue tracker before tearing down and reinstalling your setup.

### Day 5: Start reading papers

[Grad Notes Ch.4](../../grad-notes/guide.html#chapter-4) covers how to ask for a first paper recommendation and talk productively with labmates, while [Grad Notes Ch.7](../../grad-notes/guide.html#chapter-7) walks through finding your research footing during the first week *(Korean only)*.

> **Tip**: Not understanding a paper on your first pass is completely normal. In week one, simply walking away able to answer *"What problem is this paper trying to solve?"* is already a win.

### Day 6–7: Get the research direction

Read Ch.18 and group your lab's recent papers and active projects by research theme, noting where each connects to the work of senior lab members.

### Things you do not need to do in the first week

- Understand every equation in a paper—fluency comes with repetition.
- Master the entire research frontier—pick up trends gradually.
- Write everything from scratch—start by reading and modifying working code.
- Configure a GPU server from zero—reuse an environment file, Docker image, or setup guide the lab has already verified.
- Arrive with a polished research proposal—learn the lab's core problems and tools first.

### Mindset for survival

Both *Research Notes* and *Grad Notes* unpack the habits that matter most at the start of a project. The links below preserve the five principles highlighted in the Korean edition *(linked chapters are Korean only)*:

- *Not knowing at first is expected* → [Grad Notes Ch.14 — The Weight of Autonomy](../../grad-notes/guide.html#chapter-14), §2
- *"It does not work" is not a report; share your prediction, what you tried, and what happened* → [Grad Notes Ch.10 — One Question per Email](../../grad-notes/guide.html#chapter-10), §3
- *Keep records clean enough for your future self to reconstruct the experiment* → [Grad Notes Ch.8 — Using Time](../../grad-notes/guide.html#chapter-8), §5
- *Start from a small, working code path* → [Grad Notes Ch.11 — The Tool Trap](../../grad-notes/guide.html#chapter-11), §1
- *Measure yourself against your own past work, not a peer's current position* → [Grad Notes Ch.15 — The Comparison Trap](../../grad-notes/guide.html#chapter-15), §3

In a SLAM, computer vision, or robotics lab, the fastest ramp-up starts with an existing project from a senior member: reproduce its environment and run the full pipeline end to end before changing a line. Reusing a known-good CUDA, ROS, and simulator setup spares you days of debugging and frees up your energy for understanding the method itself.
