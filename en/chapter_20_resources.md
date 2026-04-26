# Ch.20 — Further Reading

A curated list of textbooks, courses, papers, and learning paths for starting robotics research. If there is too much material and you do not know where to look, start with the **Learning Path** section at the bottom.

## 20.1 Textbooks

### Computer Vision

**Multiple View Geometry in Computer Vision** (Hartley & Zisserman)
- The core reference on multi-view geometry
- Camera models, Epipolar Geometry, 3D reconstruction
- Mathematically rigorous — honestly painful to read cover to cover, but you can pick the chapters you need
- Link: [Cambridge University Press](https://www.cambridge.org/core/books/multiple-view-geometry-in-computer-vision/0B6F289C78B2B23F596CAA76D3D43F7A)
- Some chapter PDFs are available on the authors' page: https://www.robots.ox.ac.uk/~vgg/hzbook/

**Computer Vision: Algorithms and Applications** (Szeliski)
- A comprehensive CV textbook
- The latest edition (2022) includes deep learning
- **Free PDF available** — a blessing for students
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
- The foundation of deep learning vision — almost everyone starting in this field watches it
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
- Extremely intuitive explanations, paired with code
- YouTube: https://www.youtube.com/playlist?list=PLAqhIrjkxbuWI23v9cThsA9GvCAUhRvKZ

## 20.3 Recommended YouTube Channels

These are YouTube channels you can watch more casually than textbooks or full courses. Play them on your commute, during meals, or on breaks, and the intuition accumulates.

| Channel | Topic | Notes |
| --- | --- | --- |
| **Cyrill Stachniss** | SLAM, Robotics | The canonical SLAM lectures. Undergraduate-class level of systematic explanation |
| **First Principles of Computer Vision** (Shree Nayar) | Computer Vision | A Columbia professor walks through CV fundamentals one by one. Genuinely approachable |
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

Reading papers is painfully hard at first. Everyone has the experience of spending two hours on a single paper and still having no idea what it says. For this reason, the **three-pass method** is recommended:

1. **First pass** (5-10 minutes)
    - Title, Abstract, Conclusion
    - Skim the figures and tables
    - Identify the key contribution
    - At this stage, decide "is this paper relevant to me?"
2. **Second pass** (1 hour)
    - Read the whole thing (you can skip equations)
    - Understand the methodology
    - Map out the related work
    - Look at the figures carefully — that is where the authors invested the most
3. **Third pass** (several hours to several days)
    - Follow the derivations
    - Analyze the code
    - Attempt a reimplementation
    - If you get here, you are an expert on that paper

> At the start, fully understanding one paper a week via the three-pass method beats skimming one per day. Speed comes naturally later.

> Using AI: after pass 1, give the PDF to Claude or GPT and ask for "a three-line summary of this paper's contribution", "a step-by-step explanation of this equation (Eq.5)", or "the differences between this paper and [comparison paper]" — this can cut pass 2 time significantly. But reading only the AI summary without the original is not acceptable — AI often misses subtle assumptions and limitations. AI is a comprehension aid, not a replacement.

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
> - [Overleaf](https://www.overleaf.com/) — online LaTeX editor. The de facto standard for collaborative paper writing
> - [Mathpix](https://mathpix.com/) — convert equation screenshots into LaTeX code
> - [Detexify](http://detexify.kirelabs.org/classify.html) — draw a symbol by hand to search for its LaTeX
> - [Tables Generator](https://www.tablesgenerator.com/) — LaTeX/HTML table generator
> - [QuillBot](https://quillbot.com/) — English paraphrasing tool. Useful for paper writing in English
> - [Ludwig](https://ludwig.guru/) — English phrase search engine. Check what native speakers actually write
> - [DL Monitor (deeplearn.org)](https://deeplearn.org/) — automatically tracks deep learning papers from major venues and arXiv

## 20.5 Major Conferences

### Computer Vision

| Conference | Tier | Notes |
| --- | --- | --- |
| **CVPR** | Top | The largest CV conference. Every June |
| **ICCV** | Top | Biennial (odd years). One of the two CV pillars together with CVPR |
| **ECCV** | Top | Europe-centered, biennial (even years) |
| **NeurIPS** | Top | ML broadly, including Vision. Every December |
| **ICML** | Top | ML broadly. Every July |
| **ICLR** | Top | Deep learning focused. Every May |

### Robotics

| Conference | Tier | Notes |
| --- | --- | --- |
| **ICRA** | Top | IEEE robotics. The largest robotics conference |
| **IROS** | Top | IEEE/RSJ. One of the two pillars with ICRA |
| **RSS** | Top | Small scale, selective. High quality |
| **CoRL** | Top | Specialized in robot learning. Rapidly growing recently |

### Autonomous driving

| Conference/Journal | Notes |
| --- | --- |
| **CVPR Workshop** (WAD, OmniCV) | Autonomous driving workshops |
| **T-IV** | Intelligent vehicles journal |
| **T-ITS** | Intelligent transportation systems journal |

> **Further reading**
> - [CV Conference Deadlines](http://conferences.visionbib.com/Iris-Conferences.html) — collected deadlines for major CV/robotics conferences

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
| Reading and implementing papers | Analyzing the latest papers | [Papers With Code](https://paperswithcode.com/) + [Yannic Kilcher's channel](https://www.youtube.com/@YannicKilcher) |
| Experimenting with new ideas | Hypothesis formulation, experimental design | Lab seminars + attending conference workshops |
| Benchmark evaluation | Quantitative comparison | Standard benchmarks per subfield (KITTI, ScanNet, Replica, etc.) |

**Exercises**:
- Analyze the code of recent papers — clone from GitHub and run it yourself
- Experiment with your own improvement ideas — try "what if I change this part?"
- Attempt paper writing — write a 4-6 page draft in LaTeX
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

*Graduate level.*

### 20.8.0 The researcher's mindset

Technical skills like writing papers and designing experiments improve with practice, but if the mindset is wrong, you can work for years and not move forward.

**Engineer is an identity, not a job**

When people hear "robotics engineer", they usually picture "someone who builds robots" — someone who assembles robot arms, runs SLAM code, and debugs motor drivers. That is not wrong, but it describes what an engineer "does", not what an engineer "is".

Engineer is an identity, not a job. It is a way of seeing the world. When there is a problem, model it; solve the modeled problem; find the best possible solution. That is the engineer's stance toward the world, and that stance does not switch off when you leave the lab.

**Everything is an optimization problem**

> "Everything in this world is optimization."
>
> — Dr. Dongjin Hyun, Head of Robotics LAB, Hyundai Motor Company

There is no 100% in engineering. There is no 0% either. Science is the discipline of discovering natural laws. It produces categorical statements like "the speed of light is 299,792,458 m/s". Engineering is different. We deal with the messy, ambiguous problems of the real world. Sensors spit out noise, motors do not precisely deliver the commanded torque, and environments change unpredictably. These problems have no "correct answer". What exists is only "the best choice under these conditions".

So the essence of what an engineer does is optimization. Model an uncertain, probabilistic world as expectations, decide the appropriate level of abstraction, and run optimization on top. In SLAM, modeling sensor noise as Gaussian and solving a factor graph to estimate the optimal pose is like this; in MPC, predicting a finite-horizon future and computing control inputs that minimize a cost function is like this.

But this is not a story only about robot problems. Choosing where to have lunch, deciding whether to pull an all-nighter to write a paper or concentrate tomorrow, picking a research topic in grad school — the structure is the same. Within the observable state you have, in an uncertain world, find the option that maximizes expected reward. Through an engineer's eyes, almost every decision in the world fits this frame.

**The true identity of shortsighted judgment**

From this perspective, what "shortsighted judgment" means also becomes clear. When we look at someone and think "why did they make that choice?", it is not that the person is stupid. It is that long-term state is not in their loss function. If the future version of yourself — your career three years from now, your health, your relationships — is not in the decision model, you end up doing greedy optimization over only the state visible in front of you. And from that person's own vantage point, they have made the optimal decision within the state space they constructed. They are not wrong; the model is incomplete.

Research is no different. Pulling an all-nighter to get this week's experimental result versus spending a week to properly structure the code and then experimenting — on the short-term loss function, the all-nighter wins, but extend the state up to six months out and the latter is overwhelmingly better. Clean code lets you repeat subsequent experiments ten times faster. The advice that "the long game matters" in research, stated mathematically, is "extend your optimization horizon".

So to become a good researcher you must consciously expand the state space of your own decision model. Not "this paper right now" but "the me of three years from now" and "this field five years out" must be in the state. Of course the far-future state is uncertain. But dropping it because it is uncertain makes you shortsighted; including it with a large variance yields sensible long-term planning. It is the same principle as increasing process noise in a Kalman filter.

Personally, I think this difference in horizon ultimately comes from attitude. Smart people being quick is true. But after a few years, the gap is not as large as you would expect. Over the long run, what separates people is... admitting when you are wrong, saying you do not know when you do not know, being firm without becoming rigid. A three-month gap is visible; over three years, the order often flips. The saying "attitude is everything" is about this context (of course super-geniuses are excluded here).

**Then where does direction come from?**

A key question arises here. To optimize, you need a cost function. "What do I minimize? What do I maximize?" In research, is it paper count? Citations? Time to graduation? Salary? These are measurable metrics, but whether they are truly the objective function to optimize is a separate question.

What do I want? Where should I go? — the answer to this question is not found inside engineering. No matter how well you solve linear algebra, no matter how many SLAM papers you read, this question will not resolve. This is the domain of the humanities. Philosophy, literature, history.

There is a saying that other people are my mirror. Reading books is about understanding their content, but it is also about looking into what life the writer lived, what they struggled with, and why they arrived at those thoughts. Through that, you can look back at yourself. Recognizing what you feel when you read this piece of writing — that is all metacognition is.

Humans are creatures that repeat the same mistakes. That is unavoidable. But the reason we have come this far despite repeating mistakes for thousands of years is that there has been record. Writing, transmitting knowledge, and recognizing the self that receives that knowledge. The progress of civilization comes from here. You can empathize with classical literature because human DNA has hardly changed on a scale of thousands of years. The troubles that ancient people faced — anxiety about direction, loss of motivation, the frustration of comparison — are the same as what a grad student faces today.

So to find direction as a researcher, read books sometimes — not papers. Books from outside engineering. In them you can find hints about what you value and which keywords recur in your own life — growth, people, freedom, recognition. There is no right answer, and knowing that there is no right answer is itself the start.

**Three things: direction, engine, tools**

To summarize, to last as a researcher you need three things.

First, **direction**. Where you should go. As above, this must be found outside engineering. Without direction, no matter how hard you run, only the fact that "I ran hard" remains.

Second, **the engine**. Motivation, driving force. What fuel does the engine in your heart take to produce output? It may be pure curiosity, a hunger for growth, or energy from exchanges with people. For some people, "the thrill of solving a problem" is fuel; for others, "seeing what I built operate in the world" is fuel. This differs by person and has no right answer. But you must know what your engine runs on. If you do not, you will not know why it stopped when the fuel runs out, and you will not be able to restart it.

Third, **tools**. A degree, programming skill, math, lab infrastructure, colleagues, family support. All the technical content covered across 21 chapters of this document falls here. Tools matter. A knife must be sharp to cut wood. But if you only sharpen tools without direction and engine, you become a person with a very sharp knife who does not know where to go. And there are more such people than you might expect.

The rest of this document is entirely a story about "tools". Direction and engine are for each person to find.

---

The following is a concrete strategy for the early stages of research. (This part is restructured with reference to [Giseop Kim, "A Sustainable Growth Guide for Research Beginners"](https://gsk1m.github.io/productivity/2024/05/25/entering-research.html).)

**Consistency beats explosive growth**

Early in research, growth is slow. You read papers and do not understand them, your code does not run, experimental results differ from expectations. This is normal. What matters is pushing forward at a steady pace, one cycle at a time, without burning out. Reading one paper a week for six months is better than reading ten papers in a month and burning out. Linear growth returns as compound interest.

**Do not be shaken by the pace next to you**

Do not get anxious because a classmate who started with you has already published, or because the lab next door has better equipment. Different research fields produce results at different speeds, and within the same field speeds vary enormously by topic. The reference should be yesterday's self. Learn from others' research philosophies and ways of working, but if you obsess over numbers (paper count, citations), you lose direction.

**A solid foundation before a magnum opus**

For a first paper, aiming for "a paper with no grounds for rejection" is more realistic than aiming for Nature. A paper where the experiments are reproducible, comparisons are fair, and claims are backed by data. For a first paper, airtight completeness matters more than flashy novelty. Put only one core message in one paper. Title it first — and check whether that title summarizes the contribution in a single sentence.

**You become good at research by doing research, not by studying theory**

If you try to finish reading every textbook before starting, you will never start. The mindset of "I will experiment after I have perfectly understood this concept" is the most dangerous. Study only as much as you need and jump into experiments. When you get stuck, look things up then. Farming starts by planting seeds, not by finishing the agricultural theory book. Technical debt will pile up, but you can pay it off later. You must pay it off eventually, but that is better than delaying the start.

**Intellectual honesty**

A good researcher always keeps open the possibility that they are wrong. When experimental results differ from the hypothesis, doubt the hypothesis before doubting the results. Being able to change your own mind is an intellectually honest attitude. Metacognition is cultivated by reading papers from diverse perspectives. "Knowing what you do not know" is the core of research competence.

**20.8.1 Paper writing**
- Structure: Abstract → Introduction → Related Work → Method → Experiments → Conclusion
- How to write the Introduction: (1) problem definition, (2) limitations of existing methods, (3) our approach, (4) list of contributions
- Experiments section: baseline comparisons, ablation study, qualitative results
- Common mistakes: unclear contribution, unfair experimental comparisons, missing key papers in related work
- Before writing the paper, draw the figures/tables first. The story locks in

**20.8.2 Experimental Design and Ablation Studies**
- Ablation: measure each component's contribution by removing them one at a time from the model/system
- Variable control: change only one thing at a time. Change two or more and you will not know which caused the effect
- Statistical significance: repeat the same experiment multiple times (at least 3) and report mean/standard deviation
- Fair comparison: compare on the same data, same split, same hardware. Copying numbers from other papers may mean different conditions

**20.8.3 Conference Presentations**
- Talk structure: problem (1 min) → existing limitations (1 min) → proposed method (3 min) → experimental results (3 min) → conclusion (1 min)
- Slides: less text, more figures/diagrams. One message per slide
- Posters: title and key figure must be visible from 3 m away
- Demo videos: 30 seconds to 1 minute. Summarize the result first, then details

**20.8.4 Peer Review**
- What to check from a reviewer's viewpoint: novelty, technical soundness, experiments, clarity, reproducibility
- Constructive feedback: instead of "this part is wrong", say "this part would be stronger with X added"
- Writing a rebuttal: address each of the reviewer's core concerns one by one. No emotional reactions

**20.8.5 Tools**
- LaTeX: Overleaf or local (texlive + vscode)
- Reference management: in the past, dedicated tools like Mendeley and Zotero were the norm, but these days the combination of **PDF reader + AI** is more efficient. For example:
  - Read a paper in Adobe Acrobat with highlights and notes, then pass the PDF to Claude or GPT for a core summary, BibTeX generation, or related-work comparison
  - Finding a paper on Google Scholar and copying BibTeX directly via the "Cite" button is often sufficient
  - Zotero + Better BibTeX is still usable but not required. Once the paper count passes 100, having a management tool becomes convenient
  - Subscribing to related papers via Semantic Scholar's Research Feed is also good
- Pipeline figures: TikZ (precise control), draw.io (quick production), Inkscape (SVG editing)
- Tables: the booktabs package (\toprule, \midrule, \bottomrule)
- Algorithms: the algorithm2e package
- Equations: unify notation across the whole paper (build a notation table)

> Further reading:
> - [How to Write a Great Research Paper (Simon Peyton Jones, Microsoft Research)](https://www.microsoft.com/en-us/research/academic-program/write-great-research-paper/) — a classic talk on paper writing
> - [How to Read a Paper (S. Keshav)](http://ccr.sigcomm.org/online/files/p83-keshavA.pdf) — the 3-pass reading method
> - [Tips for Writing Technical Papers (Jennifer Widom, Stanford)](https://cs.stanford.edu/people/widom/paper-writing.html) — concise, practical advice
