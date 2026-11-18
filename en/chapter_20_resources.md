# Ch.20 — Further Reading

Once you have a map of the field, the fastest way to deepen your footing is to pair hands-on code with a few trusted textbooks, lecture series, and landmark papers. The resources below are grouped by topic, and the **Learning Path** at the end orders them by background so you can jump in at the right level.

## 20.1 Textbooks

### Computer Vision

**Multiple View Geometry in Computer Vision** (Hartley & Zisserman)
The standard mathematical reference on multi-view geometry, covering camera models, epipolar geometry, and 3D reconstruction in depth. Use it as a targeted reference when working through derivations rather than trying to read it straight through.
- Link: [Cambridge University Press](https://www.cambridge.org/core/books/multiple-view-geometry-in-computer-vision/0B6F289C78B2B23F596CAA76D3D43F7A)
- Selected chapter PDFs on the authors' page: https://www.robots.ox.ac.uk/~vgg/hzbook/

**Computer Vision: Algorithms and Applications** (Szeliski)
A comprehensive computer vision textbook whose latest edition (2022) incorporates modern deep learning alongside classical methods, with a free PDF edition provided by the author.
- Free PDF: https://szeliski.org/Book/

### Robotics

**Probabilistic Robotics** (Thrun, Burgard, Fox)
The foundational textbook on probabilistic robotics—essential reading if you plan to work on Kalman filters, particle filters, or SLAM. Although the full PDF is not officially free, the authors' lecture slides cover most of the core material.
- Link: [MIT Press](https://mitpress.mit.edu/9780262201629/probabilistic-robotics/)

**State Estimation for Robotics** (Tim Barfoot)
An advanced treatment of state estimation using Lie groups and factor graphs; mathematically rigorous yet remarkably clear in its exposition, and available as a free PDF.
- Free PDF: http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf

### Deep Learning

**Deep Learning** (Goodfellow, Bengio, Courville)
The classic reference on deep learning theory, available free online chapter by chapter.
- Free online (HTML, chapter by chapter): https://www.deeplearningbook.org/

**Dive into Deep Learning** (d2l.ai)
A free, interactive textbook designed around learning through runnable code, with PyTorch, TensorFlow, and JAX editions.
- Link: https://d2l.ai/

### Math supplements

**Introduction to Linear Algebra** (Gilbert Strang)
A classic text that builds geometric and intuitive fluency in linear algebra, especially effective when paired with the accompanying MIT OpenCourseWare lectures.
- Link: https://math.mit.edu/~gs/linearalgebra/ila6/indexila6.html

**Convex Optimization** (Boyd & Vandenberghe)
The standard reference on convex optimization theory and algorithms, freely available as a PDF.
- Free PDF: https://web.stanford.edu/~boyd/cvxbook/

## 20.2 Online Courses

### Computer Vision

**CS231n: Convolutional Neural Networks for Visual Recognition** (Stanford)
A cornerstone course on deep-learning-based computer vision, with free lecture videos and detailed course notes.
- Lectures: https://www.youtube.com/playlist?list=PL3FW7Lu3i5JvHM8ljYj-zLfQRF3EO8sYv
- Notes: https://cs231n.github.io/

**CS231A: Computer Vision, From 3D Reconstruction to Recognition** (Stanford)
A geometry-driven course focused on 3D vision, camera modeling, and reconstruction.
- Materials: https://web.stanford.edu/class/cs231a/

### SLAM

**Cyrill Stachniss SLAM Course** (YouTube)
Clear, well-paced lectures on SLAM theory—widely regarded as the single best starting point for newcomers to SLAM.
- YouTube: https://www.youtube.com/playlist?list=PLgnQpQtFTOGQrZ4O5QzbIHgl3b1JHimN_

**Multiple View Geometry** (TUM, Prof. Daniel Cremers)
A mathematically rigorous graduate lecture series on multi-view geometry and 3D reconstruction, available in full on YouTube.
- YouTube: https://www.youtube.com/playlist?list=PLTBdjV_4f-EJn6udZ34tht9EVIW7lbeo4

**SLAM introduction (Korean)**:
- Study materials maintained by the SLAM KR community, a Korean group of SLAM researchers.

### ROS

**ROS2 official tutorials**
The most up-to-date guide to core ROS2 concepts and tooling. The link below opens the Humble documentation; use the version dropdown at the top of the page for distributions such as Iron or Jazzy.
- Link: https://docs.ros.org/en/humble/Tutorials.html

**The Construct** (online platform)
Browser-based hands-on ROS courses with simulated robots, offering both free introductory modules and paid tracks.
- Link: https://www.theconstructsim.com/

### Deep learning fundamentals

**CS229: Machine Learning** (Stanford, Andrew Ng)
A thorough grounding in core machine learning concepts, worth watching before diving deep into neural architectures.
- YouTube: https://www.youtube.com/playlist?list=PLoROMvodv4rMiGQp3WXShtMGgzqpfVfbU

**Neural Networks: Zero to Hero** (Andrej Karpathy)
Teaches neural networks from first principles by writing every component in code from scratch.
- YouTube: https://www.youtube.com/playlist?list=PLAqhIrjkxbuWI23v9cThsA9GvCAUhRvKZ

## 20.3 Recommended YouTube Channels

When you want focused explanations rather than a full semester-long course, these channels break concepts down into shorter, digestible videos.

| Channel | Topic | Notes |
| --- | --- | --- |
| **Cyrill Stachniss** | SLAM, Robotics | Systematic SLAM lectures at an accessible course level |
| **First Principles of Computer Vision** (Shree Nayar) | Computer Vision | Builds up CV fundamentals concept by concept |
| **Andrej Karpathy** | Deep Learning, AI | Former Tesla AI Director; builds neural nets from scratch |
| **Yannic Kilcher** | Paper reviews | Weekly deep dives into new ML/AI papers—great for learning how to read papers |
| **Two Minute Papers** | AI research trends | Summarizes recent papers in 2–3 minute clips (*"What a time to be alive!"*) |
| **3Blue1Brown** | Math visualization | Visual intuition for linear algebra and calculus when equations stall |
| **Computerphile** | CS broadly | Broad computer science topics explained clearly and concisely |
| **sentdex** | Python, ML | Code-first Python tutorials covering ML and practical robotics |
| **The Coding Train** | Algorithm visualization | Energetic, visual walkthroughs of algorithms in code |

**Channel links**:
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

[*Research Notes* Ch.6–15 — Reading](../../research-notes/guide.html#chapter-6) walks through how to choose papers, apply Keshav's three-pass method, check the 5 Cs, adopt a reviewer's perspective, and analyze work through the CCC lens *(Korean only)*.

### Must-read paper list

If you are building a foundational reading list across Spatial AI, start with these landmark papers organized by subfield:

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
- 3D Gaussian Splatting: Kerbl et al., 2023 — [arXiv:2308.04079](https://arxiv.org/abs/2308.04079)
- Depth Anything: Yang et al., 2024 — [arXiv:2401.10891](https://arxiv.org/abs/2401.10891)

> For literature search, rely on [Google Scholar](https://scholar.google.com/), [Semantic Scholar](https://www.semanticscholar.org/), and [arXiv](https://arxiv.org/). Papers With Code, which previously paired benchmark leaderboards with repository links, shut down in 2025; its domain now redirects to Hugging Face.

### Paper-writing tools

These utilities speed up LaTeX typesetting, equation entry, and English phrasing when preparing a manuscript:

> **Further reading**
> - [Overleaf](https://www.overleaf.com/) — Collaborative online LaTeX editor
> - [Mathpix](https://mathpix.com/) — Converts screenshots of equations into LaTeX
> - [Detexify](http://detexify.kirelabs.org/classify.html) — Lets you sketch a symbol by hand to look up its LaTeX command
> - [Tables Generator](https://www.tablesgenerator.com/) — Visual editor for generating LaTeX and HTML tables
> - [QuillBot](https://quillbot.com/) — English paraphrasing tool helpful during manuscript drafting
> - [Ludwig](https://ludwig.guru/) — English phrase search engine for checking how native writers use an expression in context
> - [DL Monitor (deeplearn.org)](https://deeplearn.org/) — Aggregates new deep learning papers from major venues and arXiv

## 20.5 Major Conferences

The *Conferences by field* table in [Research Notes Ch.34](../../research-notes/guide.html#chapter-34) summarizes submission schedules and venue profiles across computer vision, robotics, and autonomous driving *(Korean only)*. That same chapter also discusses why researchers attend conferences and how to open a talk effectively.

## 20.6 Useful GitHub Repositories

Reading and running well-maintained open-source implementations is one of the quickest ways to bridge theory and real systems.

### SLAM

These repositories cover the major classical and Gaussian-splatting SLAM paradigms:

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

For training, pretrained vision backbones, and modular detection or segmentation pipelines, start here:

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

These tools handle point-cloud geometry, Structure from Motion, depth prediction, and neural 3D reconstruction:

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

To experiment directly with modern foundation models and vision-language-action policies, check out these repositories:

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

For middleware, autonomous navigation, arm motion planning, and embedded microcontrollers, rely on the core ROS2 ecosystem:

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

Curated "awesome" lists are handy when surveying a subfield or hunting for recent papers and codebases:

```
# Awesome SLAM — comprehensive SLAM resources
https://github.com/SilenceOverflow/Awesome-SLAM

# Awesome Robotics — comprehensive robotics resources
https://github.com/kiloreux/awesome-robotics

# Awesome 3D Gaussian Splatting — 3DGS papers and code
https://github.com/MrNeRF/awesome-3D-gaussian-splatting
```

## 20.7 Recommended Learning Path

This roadmap builds directly on the overview in Section 1.4. Each stage pairs concrete topics with curated links and exercises, so jump in at whichever stage fits your background.

### Beginner stage (1-3 months)

**Goal**: Build fluency with the essential tools until you can reliably get code up and running.

| Topic | What to learn | Recommended resources |
| --- | --- | --- |
| Python fluency | Syntax, classes, file I/O | [Jump to Python](https://wikidocs.net/book/1) (free, Korean) |
| NumPy, OpenCV basics | Array operations, reading and processing images | [OpenCV official tutorials](https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html) |
| Linear algebra review | Matrices, eigenvalues, SVD | [3Blue1Brown: Essence of Linear Algebra](https://www.youtube.com/playlist?list=PLZHQObOWTQDPD3MizzM2xVFitgF8hE_ab) |
| Probability/statistics review | Bayes' rule, Gaussians | [StatQuest](https://www.youtube.com/@statquest) |
| ROS2 basics | Nodes, topics, services | [ROS2 official tutorials](https://docs.ros.org/en/humble/Tutorials.html) |
| Git usage | Commit, branch, pull request workflows | [Git introduction](https://backlog.com/git-tutorial/kr/) (Korean) |

**Exercises**:
- Practice basic OpenCV image processing (grayscale conversion, edge detection, keypoint extraction).
- Write a minimal ROS2 publisher/subscriber pair.
- Run a camera calibration workflow (see Chapter 9 of this guide).
- Read **Chapters 3 and 9** to solidify coordinate transformations and camera models.

**Milestone**: Once you can load an image in Python, extract keypoints, and visualize feature matches between two frames, you are ready to move on from the beginner stage.

### Intermediate stage (3-6 months)

**Goal**: Master the core algorithms so you can read a paper and reproduce its implementation.

| Topic | What to learn | Recommended resources |
| --- | --- | --- |
| Deep learning basics (PyTorch) | CNNs, training loops, backpropagation | [CS231n](https://www.youtube.com/playlist?list=PL3FW7Lu3i5JvHM8ljYj-zLfQRF3EO8sYv) + [PyTorch official tutorials](https://pytorch.org/tutorials/) |
| Object Detection | YOLO, Faster R-CNN | [Ultralytics docs](https://docs.ultralytics.com/) + Chapter 10 of this document |
| Understanding Visual SLAM | ORB-SLAM3 pipeline analysis | [Cyrill Stachniss SLAM lectures](https://www.youtube.com/playlist?list=PLgnQpQtFTOGQrZ4O5QzbIHgl3b1JHimN_) + Chapter 14 of this document |
| Point cloud processing | Using Open3D | [Open3D tutorials](http://www.open3d.org/docs/release/tutorial/) + Chapter 13 of this document |
| Depth Estimation | Monocular depth estimation | Chapter 10 of this document + [Depth Anything code](https://github.com/DepthAnything/Depth-Anything-V2) |

**Exercises**:
- Explore the KITTI dataset via the [KITTI homepage](https://www.cvlibs.net/datasets/kitti/).
- Fine-tune YOLOv8 on a custom dataset.
- Build and run ORB-SLAM3, then evaluate it on the TUM RGB-D dataset.
- Compute Absolute Trajectory Error (ATE) and Relative Pose Error (RPE) yourself on a TUM RGB-D sequence.
- Read **Chapters 9–14** to connect the underlying theory across perception and SLAM.

**Milestone**: Once you can build ORB-SLAM3 from source, run it on a benchmark sequence, and evaluate its trajectory against ground truth, you have cleared the intermediate stage.

### Advanced stage (6 months+)

**Goal**: Develop independent research capability—proposing hypotheses, modifying architectures, and testing them systematically.

| Topic | What to learn | Recommended resources |
| --- | --- | --- |
| VFMs in depth (extending the intermediate-stage item from Section 1.4) | DINOv2, SAM, CLIP | Chapters 10–11 of this document + the original papers |
| Advanced 3D reconstruction | NeRF, 3D Gaussian Splatting | [NeRF Studio](https://github.com/nerfstudio-project/nerfstudio) + Chapter 13 of this document |
| Reading and implementing papers | Dissecting recent papers and codebases | [Hugging Face Papers](https://huggingface.co/papers/trending) + [Yannic Kilcher's channel](https://www.youtube.com/@YannicKilcher) — *see [*Research Notes* Ch.6–15 — Reading](../../research-notes/guide.html#chapter-6)* |
| Experimenting with new ideas | Hypothesis formulation, experimental design | Lab seminars and conference workshops — *see [*Research Notes* Ch.1–5 — Starting](../../research-notes/guide.html#chapter-1)* |
| Benchmark evaluation | Fair quantitative comparison | Standard subfield benchmarks (KITTI, ScanNet, Replica, etc.) — *see [Research Notes Ch.32](../../research-notes/guide.html#chapter-32)* |

**Exercises**:
- Clone recent paper repositories from GitHub, trace the code, and reproduce key runs yourself.
- Run targeted ablation or modification experiments (*"What happens if I replace this component?"*).
- Draft a short paper or workshop report following the structure in [Research Notes Part 2](../../research-notes/guide.html#chapter-16).
- Read **Chapters 10–13** to stay grounded in current research directions.

**Milestone**: Once you can modify or improve an existing paper's method and evaluate the change quantitatively against a clean baseline, you have entered the advanced stage—aim for a submission-ready workshop paper.

### Learning order summary

The diagram below condenses the three stages into a quick-reference progression:

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

*Graduate level.* Paper reading and writing, experimental design, conference talks, and peer review are covered in depth in [Research Notes](../../research-notes/guide.html), while long-term PhD habits are treated in [Grad Notes](../../grad-notes/guide.html). The subsections below point to the pieces most directly relevant to SLAM, computer vision, and robotics.

Sustainable research depends on balancing direction, daily execution, and reliable tooling at the same time.

### 20.8.0 Researcher Mindset

- Framing direction, execution engine, and tools within one system → [*Grad Notes* Ch.17 — Conditions Under Which Research Becomes a Life](../../grad-notes/guide.html#chapter-17), §5
- Carrying the weight of autonomy, Hyun's *everything is optimization*, and treating the optimization horizon as a long game → [*Grad Notes* Ch.14 — The Weight of Autonomy](../../grad-notes/guide.html#chapter-14), §1
- Choosing steady consistency over bursts of speed, and avoiding the trap of comparing your pace with the person next to you → [*Grad Notes* Ch.15 — The Comparison Trap](../../grad-notes/guide.html#chapter-15), §3
- Building solid foundations and designing a paper with no obvious reason for rejection → [*Research Notes* Ch.16 — Mindset](../../research-notes/guide.html#chapter-16), §1

### 20.8.1 Writing a Paper

The standard Abstract → Introduction → Related Work → Method → Experiments → Conclusion arc—and the progression in an introduction from problem statement and prior limitations to approach and contributions—are covered in [*Research Notes* Part 2 — Writing](../../research-notes/guide.html#chapter-16) and [Ch.23 — Introduction](../../research-notes/guide.html#chapter-23).

### 20.8.2 Experimental Design and Ablation

Convincing SLAM and computer vision experiments rely on clean ablations, controlled variables, repeated trials, and comparisons run under identical datasets, splits, and hardware. Copying baseline numbers directly from another paper can invalidate a comparison whenever evaluation conditions differ.

### 20.8.3 Conference Presentations

Slide budgets for 5-minute and 20-minute talks—along with the 30-second elevator pitch and 2-minute walk-through for poster sessions—are laid out in [*Research Notes* Part 4 — Presenting](../../research-notes/guide.html#chapter-33).

### 20.8.4 Reviewing a Paper — Peer Review

For a reviewer's checklist covering novelty, technical soundness, experiments, clarity, and reproducibility—as well as how to write constructive reviews and rebuttals—see [*Research Notes* Ch.10 — Reading as a Reviewer](../../research-notes/guide.html#chapter-10) and [Ch.32 — Revision/Rebuttal](../../research-notes/guide.html#chapter-32).

### 20.8.5 Tools

Keeping your writing and citation workflow clean saves days before every submission deadline:

- **LaTeX**: Overleaf for collaboration, or a local TeX Live + VS Code setup for speed and offline control.
- **References**: Highlight primary claims in a PDF reader and manage citations with Zotero + Better BibTeX. Always verify AI-generated summaries, related-work comparisons, and draft BibTeX entries against the original paper and DOI metadata.
- **Pipeline figures**: TikZ for publication precision, draw.io for rapid iteration, or Inkscape for clean SVG editing.
- **Tables**: `booktabs` (`\toprule`, `\midrule`, `\bottomrule`) for clean academic formatting.
- **Algorithms**: `algorithm2e` for readable pseudocode blocks.
- **Equations**: Maintain a shared notation table and enforce symbol consistency across the entire paper.

For consistent LaTeX notation, notation tables, and clear presentation of equations, see [*Research Notes* Ch.30 — Writing Equations, Theorems, and Proofs](../../research-notes/guide.html#chapter-30).

> **Recommended references**:
> - [How to Write a Great Research Paper (Simon Peyton Jones, Microsoft Research)](https://www.microsoft.com/en-us/research/academic-program/write-great-research-paper/) — A classic talk on structuring clear, persuasive papers
> - [How to Read a Paper (S. Keshav)](http://ccr.sigcomm.org/online/files/p83-keshavA.pdf) — The widely used three-pass reading method
> - [Tips for Writing Technical Papers (Jennifer Widom, Stanford)](https://cs.stanford.edu/people/widom/paper-writing.html) — Concise, practical advice for technical manuscripts
