# Ch.20 — Further Reading

The resources are grouped by topic, and the **Learning Path** section at the bottom orders them by prerequisite background.

## 20.1 Textbooks

### Computer Vision

**Multiple View Geometry in Computer Vision** (Hartley & Zisserman)
- The core reference on multi-view geometry
- Camera models, Epipolar Geometry, 3D reconstruction
- Mathematically rigorous and useful as a reference for selected topics
- Link: [Cambridge University Press](https://www.cambridge.org/core/books/multiple-view-geometry-in-computer-vision/0B6F289C78B2B23F596CAA76D3D43F7A)
- Some chapter PDFs are available on the authors' page: https://www.robots.ox.ac.uk/~vgg/hzbook/

**Computer Vision: Algorithms and Applications** (Szeliski)
- A comprehensive CV textbook
- The latest edition (2022) includes deep learning
- **Free PDF available**
- Free PDF: https://szeliski.org/Book/

### Robotics

**Probabilistic Robotics** (Thrun, Burgard, Fox)
- The standard text on probabilistic robotics
- Kalman Filter, Particle Filter, SLAM
- Required reading — if you want to research SLAM, you must read it
- Link: [MIT Press](https://mitpress.mit.edu/9780262201629/probabilistic-robotics/)
- The PDF is not officially free, but the authors' lecture slides cover most of the content

**State Estimation for Robotics** (Tim Barfoot)
- Advanced state estimation
- Lie Groups, Factor Graph — mathematically deep but the exposition is approachable
- **Free PDF available**
- Free PDF: http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf

### Deep Learning

**Deep Learning** (Goodfellow, Bengio, Courville)
- The standard textbook on deep learning theory
- **Free online** edition
- Free PDF: https://www.deeplearningbook.org/

**Dive into Deep Learning** (d2l.ai)
- Hands-on — you learn alongside code
- **Free and interactive**
- Link: https://d2l.ai/
- Supports PyTorch, TensorFlow, and JAX versions

### Math supplements

**Introduction to Linear Algebra** (Gilbert Strang)
- A classic that explains linear algebra intuitively
- Maximum effect when paired with the MIT OCW lectures
- Link: https://math.mit.edu/~gs/linearalgebra/ila6/indexila6.html

**Convex Optimization** (Boyd & Vandenberghe)
- The standard text on optimization theory
- **Free PDF available**
- Free PDF: https://web.stanford.edu/~boyd/cvxbook/

## 20.2 Online Courses

### Computer Vision

**CS231n: Convolutional Neural Networks for Visual Recognition** (Stanford)
- A course on the foundations of deep-learning-based vision
- Free materials and videos
- Lectures: https://www.youtube.com/playlist?list=PL3FW7Lu3i5JvHM8ljYj-zLfQRF3EO8sYv
- Notes: https://cs231n.github.io/

**CS231A: Computer Vision, From 3D Reconstruction to Recognition** (Stanford)
- 3D Vision focused
- Geometry-based
- Materials: https://web.stanford.edu/class/cs231a/

### SLAM

**Cyrill Stachniss SLAM Course** (YouTube)
- SLAM theory lectures — German accent, but the explanations are genuinely clear
- The most recommended course for SLAM beginners
- YouTube: https://www.youtube.com/playlist?list=PLgnQpQtFTOGQrZ4O5QzbIHgl3b1JHimN_

**Multiple View Geometry** (TUM, Prof. Daniel Cremers)
- Available on YouTube — mathematically rigorous lectures
- YouTube: https://www.youtube.com/playlist?list=PLTBdjV_4f-EJn6udZ34tht9EVIW7lbeo4

**SLAM introduction (Korean)**:
- Study materials from the SLAM KR community: https://github.com/slam-kr

### ROS

**ROS2 official tutorials**
- The most up-to-date information
- Link: https://docs.ros.org/en/humble/Tutorials.html (Humble)
- For other versions such as ROS2 Iron/Jazzy, switch via the dropdown at the top

**The Construct** (online platform)
- Dedicated ROS courses
- Partly free
- Link: https://www.theconstructsim.com/

### Deep learning fundamentals

**CS229: Machine Learning** (Stanford, Andrew Ng)
- ML foundations — recommended to watch before deep learning
- YouTube: https://www.youtube.com/playlist?list=PLoROMvodv4rMiGQp3WXShtMGgzqpfVfbU

**Neural Networks: Zero to Hero** (Andrej Karpathy)
- Learn neural networks by building them from scratch
- Explanations paired with code
- YouTube: https://www.youtube.com/playlist?list=PLAqhIrjkxbuWI23v9cThsA9GvCAUhRvKZ

## 20.3 Recommended YouTube Channels

These YouTube channels divide the material into shorter segments than a textbook or full course.

| Channel | Topic | Notes |
| --- | --- | --- |
| **Cyrill Stachniss** | SLAM, Robotics | Systematic SLAM explanations at an undergraduate-course level |
| **First Principles of Computer Vision** (Shree Nayar) | Computer Vision | Explains CV fundamentals concept by concept |
| **Andrej Karpathy** | Deep Learning, AI | Former Tesla AI Director. Builds neural nets from scratch |
| **Yannic Kilcher** | Paper reviews | Weekly reviews of the latest ML/AI papers. You learn how to read papers |
| **Two Minute Papers** | AI research trends | Introduces the latest research in 2-3 minute videos. "What a time to be alive!" |
| **3Blue1Brown** | Math visualization | Visual explanations of linear algebra and calculus. When math gets stuck |
| **Computerphile** | CS broadly | Wide range of computer science topics explained simply |
| **sentdex** | Python, ML | ML/robotics practice with Python. Code-centric |
| **The Coding Train** | Algorithm visualization | Understand algorithms visually. Energetic delivery |

**Link list**:
- Cyrill Stachniss: https://www.youtube.com/@CyrillStachniss
- First Principles of Computer Vision: https://www.youtube.com/@firstprinciplesofcomputerv3258
- Andrej Karpathy: https://www.youtube.com/@AndrejKarpathy
- Yannic Kilcher: https://www.youtube.com/@YannicKilcher
- Two Minute Papers: https://www.youtube.com/@TwoMinutePapers
- 3Blue1Brown: https://www.youtube.com/@3blue1brown
- Computerphile: https://www.youtube.com/@Computerphile
- sentdex: https://www.youtube.com/@sentdex
- The Coding Train: https://www.youtube.com/@TheCodingTrain

## 20.4 Reading Papers

### How to read them

[*Research Notes* Ch.6–12 — Reading](../../research-notes/guide.html#chapter-6) covers how to select papers, Keshav's three-pass method, the 5 Cs, the reviewer perspective, and the CCC lens *(Korean only)*.

### Must-read paper list

**Classical CV/SLAM**:
- ORB-SLAM: Mur-Artal et al., 2015 — [arXiv:1502.00956](https://arxiv.org/abs/1502.00956)
- LOAM: Zhang & Singh, 2014 — [RSS 2014](https://www.ri.cmu.edu/pub_files/2014/7/Ji_LidarMapping_RSS2014_v8.pdf)
- VINS-Mono: Qin et al., 2018 — [arXiv:1708.03852](https://arxiv.org/abs/1708.03852)

**Deep Learning fundamentals**:
- ResNet: He et al., 2015 — [arXiv:1512.03385](https://arxiv.org/abs/1512.03385)
- Transformer (Attention Is All You Need): Vaswani et al., 2017 — [arXiv:1706.03762](https://arxiv.org/abs/1706.03762)
- ViT: Dosovitskiy et al., 2020 — [arXiv:2010.11929](https://arxiv.org/abs/2010.11929)

**Object Detection**:
- Faster R-CNN: Ren et al., 2015 — [arXiv:1506.01497](https://arxiv.org/abs/1506.01497)
- YOLO (original): Redmon et al., 2015 — [arXiv:1506.02640](https://arxiv.org/abs/1506.02640)
- DETR: Carion et al., 2020 — [arXiv:2005.12872](https://arxiv.org/abs/2005.12872)

**Foundation Models**:
- CLIP: Radford et al., 2021 — [arXiv:2103.00020](https://arxiv.org/abs/2103.00020)
- SAM (Segment Anything): Kirillov et al., 2023 — [arXiv:2304.02643](https://arxiv.org/abs/2304.02643)
- DINOv2: Oquab et al., 2023 — [arXiv:2304.07193](https://arxiv.org/abs/2304.07193)

**Recent trends**:
- RT-2: Brohan et al., 2023 — [arXiv:2307.15818](https://arxiv.org/abs/2307.15818)
- 3D Gaussian Splatting: Kerbl et al., 2023 — [arXiv:2308.14737](https://arxiv.org/abs/2308.14737)
- Depth Anything: Yang et al., 2024 — [arXiv:2401.10891](https://arxiv.org/abs/2401.10891)

> For paper search, use [Google Scholar](https://scholar.google.com/), [Semantic Scholar](https://www.semanticscholar.org/), [Papers With Code](https://paperswithcode.com/), and [arXiv](https://arxiv.org/). Papers With Code is especially convenient because it shows benchmark rankings alongside code links.

### Paper-writing tools

> **Further reading**
> - [Overleaf](https://www.overleaf.com/) — online LaTeX editor with collaborative editing
> - [Mathpix](https://mathpix.com/) — convert equation screenshots into LaTeX code
> - [Detexify](http://detexify.kirelabs.org/classify.html) — draw a symbol by hand to search for its LaTeX
> - [Tables Generator](https://www.tablesgenerator.com/) — LaTeX/HTML table generator
> - [QuillBot](https://quillbot.com/) — English paraphrasing tool. Useful for paper writing in English
> - [Ludwig](https://ludwig.guru/) — English phrase search engine. Check what native speakers actually write
> - [DL Monitor (deeplearn.org)](https://deeplearn.org/) — automatically tracks deep learning papers from major venues and arXiv

## 20.5 Major Conferences

The *Conferences by field* table in [Research Notes Ch.34](../../research-notes/guide.html#chapter-34) summarizes the schedules and character of venues in CV, robotics, and autonomous driving *(Korean only)*.

The same chapter discusses why researchers attend conferences and how to open a presentation.

## 20.6 Useful GitHub Repositories

### SLAM

```
# ORB-SLAM3 — reference for Visual(-Inertial) SLAM
https://github.com/UZ-SLAMLab/ORB_SLAM3

# VINS-Fusion — multi-camera + IMU fusion
https://github.com/HKUST-Aerial-Robotics/VINS-Fusion

# LIO-SAM — LiDAR-Inertial SLAM (factor graph based)
https://github.com/TixiaoShan/LIO-SAM

# FAST-LIO2 — fast LiDAR-Inertial Odometry
https://github.com/hku-mars/FAST_LIO

# RTAB-Map — RGB-D SLAM, supports large-scale environments
https://github.com/introlab/rtabmap

# SplaTAM — SLAM based on 3D Gaussian Splatting
https://github.com/spla-tam/SplaTAM
```

### Deep Learning

```
# Ultralytics YOLO — YOLOv8/v11, the easiest detection framework to use
https://github.com/ultralytics/ultralytics

# HuggingFace Transformers — NLP/Vision model hub
https://github.com/huggingface/transformers

# OpenMMLab — comprehensive framework for Detection, Segmentation, 3D, etc.
https://github.com/open-mmlab

# PyTorch Lightning — structuring training code
https://github.com/Lightning-AI/pytorch-lightning

# timm (PyTorch Image Models) — collection of pretrained Vision models
https://github.com/huggingface/pytorch-image-models
```

### 3D Vision

```
# Open3D — point cloud and mesh processing
https://github.com/isl-org/Open3D

# 3D Gaussian Splatting — original implementation
https://github.com/graphdeco-inria/gaussian-splatting

# NeRF Studio — unified framework for NeRF/3DGS
https://github.com/nerfstudio-project/nerfstudio

# Depth Anything V2 — general-purpose depth estimation
https://github.com/DepthAnything/Depth-Anything-V2

# COLMAP — Structure from Motion pipeline
https://github.com/colmap/colmap
```

### VFM/VLA

```
# Segment Anything (SAM) — Meta's general-purpose segmentation
https://github.com/facebookresearch/segment-anything

# SAM 2 — extended to video
https://github.com/facebookresearch/sam2

# DINOv2 — Self-supervised vision features
https://github.com/facebookresearch/dinov2

# Grounded-SAM — find objects by text + segmentation
https://github.com/IDEA-Research/Grounded-Segment-Anything

# OpenVLA — open-source Vision-Language-Action model
https://github.com/openvla/openvla
```

### ROS / robot development

```
# ROS2 official repository
https://github.com/ros2

# Nav2 — ROS2 navigation stack
https://github.com/ros-navigation/navigation2

# MoveIt2 — motion planning for robot arms
https://github.com/moveit/moveit2

# micro-ROS — ROS for microcontrollers
https://github.com/micro-ROS
```

### Useful Awesome lists

```
# Awesome SLAM — comprehensive SLAM resources
https://github.com/SilenceOverflow/Awesome-SLAM

# Awesome Robotics — comprehensive robotics resources
https://github.com/kiloreux/awesome-robotics

# Awesome 3D Gaussian Splatting — 3DGS papers and code
https://github.com/MrNeRF/awesome-3D-gaussian-splatting
```

## 20.7 Recommended Learning Path

This extends the learning path from Section 1.4. Each stage lists concrete materials and links, so start from whichever stage matches your level.

### Beginner stage (1-3 months)

**Goal**: Acquire basic tools — reach a state of "being able to run something".

| Topic | What to learn | Recommended resources |
| --- | --- | --- |
| Python fluency | Syntax, classes, file I/O | [Jump to Python](https://wikidocs.net/book/1) (free, Korean) |
| NumPy, OpenCV basics | Array operations, reading/processing images | [OpenCV official tutorials](https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html) |
| Linear algebra review | Matrices, eigenvalues, SVD | [3Blue1Brown: Essence of Linear Algebra](https://www.youtube.com/playlist?list=PLZHQObOWTQDPD3MizzM2xVFitgF8hE_ab) |
| Probability/statistics review | Bayes' rule, Gaussians | [StatQuest](https://www.youtube.com/@statquest) |
| ROS2 basics | Nodes, topics, services | [ROS2 official tutorials](https://docs.ros.org/en/humble/Tutorials.html) |
| Git usage | commit, branch, PR | [Git introduction](https://backlog.com/git-tutorial/kr/) (Korean) |

**Exercises**:
- Image processing with OpenCV (grayscale conversion, edge detection, feature extraction)
- Write a simple ROS2 node (publisher/subscriber)
- Perform camera calibration — see Chapter 9 of this document
- Read **Chapters 3 and 9** of this document to understand coordinate transformations and camera models

**Milestone**: If you can read an image in Python, extract keypoints, and visualize matches between two images, you have graduated from the beginner stage.

### Intermediate stage (3-6 months)

**Goal**: Understand the core techniques — reach a state of "being able to read a paper and run its code".

| Topic | What to learn | Recommended resources |
| --- | --- | --- |
| Deep learning basics (PyTorch) | CNN, training, backpropagation | [CS231n](https://www.youtube.com/playlist?list=PL3FW7Lu3i5JvHM8ljYj-zLfQRF3EO8sYv) + [PyTorch official tutorials](https://pytorch.org/tutorials/) |
| Object Detection | YOLO, Faster R-CNN | [Ultralytics docs](https://docs.ultralytics.com/) + Chapter 10 of this document |
| Understanding Visual SLAM | ORB-SLAM3 analysis | [Cyrill Stachniss SLAM lectures](https://www.youtube.com/playlist?list=PLgnQpQtFTOGQrZ4O5QzbIHgl3b1JHimN_) + Chapter 14 of this document |
| Point cloud processing | Using Open3D | [Open3D tutorials](http://www.open3d.org/docs/release/tutorial/) + Chapter 13 of this document |
| Depth Estimation | Monocular depth estimation | Chapter 10 of this document + [Depth Anything code](https://github.com/DepthAnything/Depth-Anything-V2) |

**Exercises**:
- Work with the KITTI dataset — [KITTI homepage](https://www.cvlibs.net/datasets/kitti/)
- YOLOv8 fine-tuning — fine-tune on a custom dataset
- Run and analyze ORB-SLAM3 — evaluate on the TUM RGB-D dataset
- TUM RGB-D benchmark — compute ATE and RPE yourself
- Read **Chapters 9-14** of this document to solidify the theoretical background

**Milestone**: If you can build ORB-SLAM3 yourself, run it on a dataset, and compare the trajectory against ground truth, you have graduated from the intermediate stage.

### Advanced stage (6 months+)

**Goal**: Develop research ability — reach a state of "being able to propose and test new ideas".

| Topic | What to learn | Recommended resources |
| --- | --- | --- |
| Understanding and using VFMs | DINOv2, SAM, CLIP | Chapters 10-11 of this document + read the papers directly |
| Advanced 3D reconstruction | NeRF, 3D Gaussian Splatting | [NeRF Studio](https://github.com/nerfstudio-project/nerfstudio) + Chapter 13 of this document |
| Reading and implementing papers | Analyzing the latest papers | [Papers With Code](https://paperswithcode.com/) + [Yannic Kilcher's channel](https://www.youtube.com/@YannicKilcher) — *the full guide is [*Research Notes* Ch.6–15 — Reading](../../research-notes/guide.html#chapter-6)* |
| Experimenting with new ideas | Hypothesis formulation, experimental design | Lab seminars + attending conference workshops — *the full guide is [*Research Notes* Ch.1–5 — Starting](../../research-notes/guide.html#chapter-1)* |
| Benchmark evaluation | Quantitative comparison | Standard benchmarks per subfield (KITTI, ScanNet, Replica, etc.) — *the result-interpretation frame is [Research Notes Ch.32](../../research-notes/guide.html#chapter-32)* |

**Exercises**:
- Analyze the code of recent papers — clone from GitHub and run it yourself
- Experiment with your own improvement ideas — try "what if I change this part?"
- Attempt paper writing — *the full frame is [Research Notes Part 2](../../research-notes/guide.html#chapter-16)*
- Read **Chapters 10-13** of this document to track recent research directions

**Milestone**: If you can run an experiment that modifies or improves an existing paper's method and compare the result quantitatively, you have entered the advanced stage. Aim to reach a level where the work could be submitted to a conference workshop.

### Learning order summary

```
Beginner (1-3 months)            Intermediate (3-6 months)          Advanced (6 months+)
─────────────                   ─────────────                    ─────────────
Python + NumPy                  PyTorch + CNN                    VFM (DINOv2, SAM)
OpenCV basics                   YOLO fine-tuning                 3DGS / NeRF
Linear algebra/prob review      ORB-SLAM3 analysis               Paper implementation
ROS2 basics                     KITTI/TUM benchmarks             Idea experimentation
Git usage                       Point clouds (Open3D)            Paper writing
                                Depth Estimation
     ↓                                ↓                                ↓
 "can run code"                "can read and reproduce papers"   "can experiment with new ideas"
```

## 20.8 Research Skills

*Graduate level.* Paper reading and writing, experimental design, conference presentations, and peer review are treated in detail in [Research Notes](../../research-notes/guide.html); long-term PhD management is covered in [Grad Notes](../../grad-notes/guide.html). This section links the parts that apply directly to SLAM, CV, and robotics.

Research practice requires attention to direction, a sustainable way of working, and tools at the same time.

### 20.8.0 Researcher Mindset

- Direction, engine, and tools as three layers within one frame → [*Grad Notes* Ch.17 — Conditions Under Which Research Becomes a Life](../../grad-notes/guide.html#chapter-17), §5
- The weight of autonomy, Hyun's *everything is optimization*, and the optimization horizon as a long game → [*Grad Notes* Ch.14 — The Weight of Autonomy](../../grad-notes/guide.html#chapter-14), §1
- Consistency versus explosive growth, and the trap of comparing pace with the person next to you → [*Grad Notes* Ch.15 — The Comparison Trap](../../grad-notes/guide.html#chapter-15), §3
- Solid foundations and the frame of a paper with no obvious reason for rejection → [*Research Notes* Ch.16 — Mindset](../../research-notes/guide.html#chapter-16), §1

### 20.8.1 Writing a Paper

The Abstract → Introduction → Related Work → Method → Experiments → Conclusion structure, and the progression from problem and prior limitation to approach and contributions in an introduction, are covered in [*Research Notes* Part 2 — Writing](../../research-notes/guide.html#chapter-16) and [Ch.23 — Introduction](../../research-notes/guide.html#chapter-23).

### 20.8.2 Experimental Design and Ablation

SLAM and CV experiments require ablation, control of variables, repeated trials, and comparisons on the same data, split, and hardware. Copying baseline numbers from another paper can make a comparison unfair when the conditions differ.

### 20.8.3 Conference Presentations

A 1+1+3+3+1-minute talk structure, one message per slide, poster legibility at 3 m, and a 30-second to one-minute demo are discussed more fully in [*Research Notes* Part 4 — Presenting](../../research-notes/guide.html#chapter-33).

### 20.8.4 Reviewing a Paper — Peer Review

For a reviewer-oriented checklist covering novelty, soundness, experiments, clarity, and reproducibility, as well as constructive feedback and rebuttals, see [*Research Notes* Ch.10 — Reading as a Reviewer](../../research-notes/guide.html#chapter-10) and [Ch.32 — Revision/Rebuttal](../../research-notes/guide.html#chapter-32).

### 20.8.5 Tools

- LaTeX: Overleaf or local installation (TeX Live + VS Code)
- References: Mark the primary text in a PDF reader and manage metadata with Zotero + Better BibTeX. Check AI-generated summaries, related-work comparisons, and draft BibTeX against the paper and DOI metadata.
- Pipeline figures: TikZ for precision, draw.io for speed, or Inkscape for SVG
- Tables: `booktabs` (`\toprule`, `\midrule`, `\bottomrule`)
- Algorithms: `algorithm2e`
- Equations: maintain a notation table and apply it consistently across the paper

For consistent LaTeX notation, notation tables, and equation explanations, see [*Research Notes* Ch.30 — Writing Equations, Theorems, and Proofs](../../research-notes/guide.html#chapter-30).

> Recommended references:
> - [How to Write a Great Research Paper (Simon Peyton Jones, Microsoft Research)](https://www.microsoft.com/en-us/research/academic-program/write-great-research-paper/) — a classic talk on paper writing
> - [How to Read a Paper (S. Keshav)](http://ccr.sigcomm.org/online/files/p83-keshavA.pdf) — the 3-pass reading method
> - [Tips for Writing Technical Papers (Jennifer Widom, Stanford)](https://cs.stanford.edu/people/widom/paper-writing.html) — concise, practical advice
