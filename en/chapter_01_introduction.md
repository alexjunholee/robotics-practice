# Ch.1 — Introduction: What Is Spatial AI?

Before diving into individual algorithms, it helps to have a map of the Spatial AI landscape. Seeing how its subfields fit together provides the context needed to understand why the techniques in later chapters matter and how they reinforce one another.

## 1.1 Defining Spatial AI

**Spatial AI** refers to the family of techniques that enable machines to understand 3D space and act reliably within it. Rather than merely classifying images or recognizing objects in isolation, a Spatial AI system has to answer four coupled questions:

- "Where am I right now?" (Localization)
- "What does the surrounding environment look like?" (Mapping)
- "What is that object, and where is it?" (Object Detection & Localization)
- "How do I reach the destination safely?" (Navigation & Planning)

Conventional deep learning asks, "Is there a cat in this image?" Spatial AI has to go further: "How many meters away is that cat, which way is it heading, and how should I move to avoid it?" Spatial context is what separates passive recognition from physical action.

To answer those questions, Spatial AI brings together five core disciplines:

- **Computer Vision**: extracting geometric and semantic information from camera images
- **3D Vision**: estimating depth and processing 3D point clouds
- **SLAM**: performing simultaneous localization and mapping in unknown environments
- **Deep Learning**: recognizing objects, scenes, and motion patterns from data
- **Sensor Fusion**: combining complementary measurements from multiple sensors

Every one of these disciplines confronts the same fundamental obstacle: uncertainty. In *Probabilistic Robotics* (§1.1), Thrun, Burgard, and Fox (2005) trace that uncertainty to five distinct sources. First, the physical environment itself is inherently unpredictable. Second, every sensor measurement is bounded by finite resolution and noise. Third, motor torque variations and wheel slip mean that actual robot motion never matches commanded motion exactly. Fourth, abstracting a real robot and environment into equations inevitably introduces modeling error. Fifth, strict real-time deadlines often force algorithms to settle for approximate solutions rather than exact optima. Those five sources motivate every estimation algorithm in this guide, from Bayes filters to SLAM; they are treated in detail in Ch.3 §3.9–3.11 and Ch.14 §14.16.

> **Further reading**
> - [Andrew Davison — From SLAM to Spatial AI (MIT Robotics)](https://www.youtube.com/watch?v=BRRtlR0C_CY) — Prof. Andrew Davison's talk laying out the vision for Spatial AI, useful for orienting yourself in the field.
> - [FutureMapping paper (arXiv:1803.11288)](https://arxiv.org/abs/1803.11288) — A 2018 position paper examining how SLAM evolves into Spatial AI perception that unifies geometry and semantics, alongside the co-design of algorithms, processors, and sensors.
> - [Cyrill Stachniss — SLAM Course (2013)](https://www.youtube.com/playlist?list=PLgnQpQtFTOGQrZ4O5QzbIHgl3b1JHimN_) — Prof. Cyrill Stachniss's graduate SLAM lectures from his Freiburg years, covering EKF-SLAM, FastSLAM, and graph SLAM—a structured introduction to the foundations behind Spatial AI.

## 1.2 Why It Matters

Which techniques deserve the deepest focus depends heavily on the target application. Autonomous driving leans on LiDAR and multi-sensor fusion, whereas AR/VR headsets rely on compact visual-inertial tracking. Surveying the broader application landscape makes it easier to choose a study path matched to your goals:

| Field | Example applications |
| --- | --- |
| **Autonomous driving** | Vehicle localization, obstacle detection, path planning |
| **Service robots** | Indoor navigation, object manipulation, human collaboration |
| **Drones** | Autonomous flight, 3D map generation, inspection/delivery |
| **AR/VR** | Spatial tracking, virtual object placement, hand tracking |
| **Industrial automation** | Logistics robots, quality inspection, assembly automation |

Across all of these domains, however, putting algorithms onto physical hardware introduces challenges that rarely appear in pure software or benchmark datasets.

## 1.3 Why Robotics Is Hard

Newcomers often wonder whether rapid advances in AI will soon solve every problem in robotics. In reality, stronger AI models address only part of the stack—many of the field's hardest bottlenecks live at the interface with the physical world:

- A bug that causes a collision can damage hardware or injure someone; unlike a software deployment, physical impact cannot be rolled back.
- A single physical experiment often requires editing code, uploading it, resetting the environment, verifying safety interlocks, running the trial, and inspecting the hardware—a cycle that takes minutes or hours rather than seconds.
- Real sensor streams contain lens flare, motion blur, drift, and dropped frames, so clean benchmark performance rarely predicts field behavior.
- Safety-critical loops such as obstacle avoidance must hit strict latency deadlines in addition to accuracy targets.
- Even a rare edge case can trigger a collision or safety incident.
- Simulators only approximate friction, inertia, and sensor noise, and accumulated modeling error can cause a physical robot to behave very differently from simulation.

The contrast between conventional software engineering and robotics becomes clear side by side:

| General software | Robotics |
|---|---|
| Bug → log → fix → redeploy | Bug → crash → damage → repair → retry |
| Iteration in seconds | Iteration in minutes to hours |
| Structured inputs | Sensor data riddled with noise |
| 99% accuracy is excellent | 99.9999% may not be enough |
| Response lag → inconvenience | Response lag → accident |
| Same input → same output | Same code, different results depending on environment |

Better AI models improve capabilities like visual recognition and natural-language command parsing, yet the challenges in the right column—slow physical iteration, noisy sensors, hard real-time deadlines, and collision risk—stem from interacting with physical reality. Scaling up model size alone cannot eliminate them, and training a single model does not produce a complete, reliable robotic system.

### What Roboticists Do in the AI Era

When AI tools can generate code, summarize papers, and draft experiment plans, where does a roboticist's core value lie? Four responsibilities remain squarely human:

- **Problem definition**: AI can help solve a clearly specified task, but it cannot decide which problem is worth solving. Selecting an appropriate sensor suite for an environment, specifying required accuracy, and weighing engineering trade-offs all demand domain judgment. (*The general framework for problem definition is covered in [Research Notes Ch.1](../../research-notes/guide.html#chapter-1) and [Research Notes Ch.2](../../research-notes/guide.html#chapter-2) (Korean only).*)
- **System integration**: Perception modules, control loops, middleware, and hardware have to function as one coherent system. AI can draft individual components, but designing interfaces, timing budgets, and fault handling across modules falls on the engineer.
- **Interface with the physical world**: A loose connector, dust on a lens, or an overheating actuator is hard to diagnose from logs or a remote terminal alone. Someone still has to inspect the physical robot and its environment firsthand.
- **Judging reliability**: When an AI tool reports "99% accuracy," the engineer must verify what was counted, against what denominator, and what failures lurk in the remaining 1%. The same 99% carries vastly different stakes when a failure can injure someone versus when it merely triggers a retry—the real criterion is the cost of failure and how often the system faces it, not the industry label.

AI accelerates coding and literature review, but it cannot shoulder these four responsibilities for you.

## 1.4 How to Use This Document

Treat this guide as a working **reference** rather than a cover-to-cover textbook:

1. **On a first pass**: skim the table of contents to grasp how the topics connect.
2. **When starting a research topic**: read the relevant sections in depth and work through the further reading.
3. **When stuck**: consult the glossary and troubleshooting notes in the appendix.

If you are building foundations from scratch, the sequence below offers a natural order of study:

```
Mathematical foundations → Sensors → Computer vision basics → SLAM → Deep learning → VFM/VLA → Lab direction
```

Mathematics unlocks the equations inside a SLAM paper, while understanding sensor behavior explains why an algorithm breaks down under specific physical conditions. Building those foundations early prevents gaps that otherwise compound in later chapters.

### Staged Learning Path

The roadmap below translates that progression into three concrete stages. Adjust the pace to match your background, though skipping stages tends to leave gaps later on.

**Entry stage — learning the tools**

At this stage, the goal is to grow comfortable with the core engineering tools used in robotics research so you can read existing code, run experiments, and interpret outputs.

**What to learn**:
1. **Reading C++ code** — Core lab software (SLAM pipelines, ROS packages) is written in C++. You do not need to write complex C++ from scratch right away, but you must be able to read and modify existing codebases.
2. **Python basics** — The standard language for deep learning training scripts, data preprocessing, and visualization, and one that pairs well with AI coding assistants.
3. **Linear algebra and probability/statistics refresh** — Reconnecting undergraduate math to robotics applications (covered in Ch.3).
4. **ROS2 basics** — Topics, services, actions, and launch files for the robot middleware used in the lab.
5. **Git usage** — Branching, merging, and rebasing for everyday collaborative code management.

**Practice exercises**:
- Build an image-processing pipeline in OpenCV (read → filter → feature extraction → visualization)
- Write a minimal ROS2 publisher/subscriber node pair
- Calibrate a physical or recorded camera using a chessboard pattern

**Intermediate stage — practicing the core techniques**

Once the tooling feels familiar, move on to running core Spatial AI algorithms yourself, analyzing failure modes, and reading primary research papers.

**What to learn**:
1. **Deep learning basics (PyTorch)** — Tensors, automatic differentiation, training loops, and model architecture; PyTorch is the dominant framework in robotics research.
2. **Object Detection (YOLO family)** — Bounding boxes, non-maximum suppression (NMS), mean average precision (mAP), and standard detection pipelines.
3. **Understanding Visual SLAM (ORB-SLAM3)** — Running a classic feature-based SLAM system and tracing how its modules interact in code.
4. **Point cloud processing (Open3D)** — Filtering, registration, and visualization for 3D geometric data.
5. **Understanding and using VFMs (DINOv2, SAM)** — Seeing how vision foundation models reshape classical perception pipelines.

**Practice exercises**:
- Run benchmark evaluations on the KITTI dataset
- Fine-tune YOLOv8 on a custom dataset
- Run ORB-SLAM3 and inspect the estimated trajectory
- Evaluate SLAM accuracy on the TUM RGB-D dataset

**Advanced stage — first steps as a researcher**

At the advanced stage, you practice the full research loop: reading papers critically, formulating hypotheses, running controlled experiments, and writing up results.

**What to learn**:
1. **Reading and implementing papers** — Aiming for at least 1–2 papers per week, inspecting code line by line for foundational works.
2. **Experimenting with new ideas** — Identifying concrete failure modes in existing methods and testing targeted improvements.
3. **Benchmark evaluation** — Following standardized evaluation protocols so comparisons remain fair and reproducible.

**Practice exercises**:
- Reproduce and analyze the open-source implementation of a recent paper
- Implement a small improvement of your own and compare it quantitatively against baselines
- Draft a short paper targeting a conference workshop submission

> **Further reading**
> - [Missing Semester of Your CS Education (MIT)](https://missing.csail.mit.edu/) — An MIT course covering the practical tools research relies on: Git, the shell, debugging, and workflows.
> - [ROS2 official tutorials](https://docs.ros.org/en/humble/Tutorials.html) — Official hands-on tutorials built around ROS2 Humble.
> - [Andrej Karpathy — Neural Networks: Zero to Hero](https://www.youtube.com/playlist?list=PLAqhIrjkxbuWI23v9cThsA9GvCAUhRvKZ) — A hands-on series that builds deep learning concepts from scratch in code.

## 1.5 Prerequisite Checklist

Before starting research, review the checklist below. Rather than merely ticking boxes, make sure you understand *why* each item matters in day-to-day robotics work.

**Required**:
- [ ] **Ability to read C++**
  - Core lab systems (SLAM, real-time control, ROS packages) rely on C++. Modifying open-source projects like ORB-SLAM3 or LOAM requires reading C++ fluently.
- [ ] **Basic Linux commands (cd, ls, cp, mv, grep)**
  - Nearly all lab workstations and GPU servers run Ubuntu, so working comfortably in a terminal over SSH is essential.
- [ ] **Basic Git usage (clone, commit, push, pull)**
  - Managing experiments, pulling paper repositories, and sharing lab code all run through Git—cloning and running open-source repositories is daily routine.

**Recommended**:
- [ ] **Python basics (functions, classes, modules)**
  - Used across training scripts, preprocessing, and visualization. Even as AI assistants write more boilerplate, you still need to read, audit, and debug Python code.
- [ ] **Basic NumPy usage**
  - Matrix operations, broadcasting, and array indexing underpin sensor data processing and coordinate transformations.
- [ ] **Linear algebra basics (matrix operations, eigenvalues)**
  - 3D transformations, camera projection models, and optimization all rest on linear algebra. Understanding what an equation represents geometrically (developed further in Ch.3) makes papers far easier to follow.
- [ ] **Probability/statistics basics (normal distribution, Bayes' theorem)**
  - Sensor noise modeling, state estimation, and filtering are grounded in probability—it gives you the language to quantify how much to trust a noisy reading.
- [ ] **Calculus basics (partial derivatives, chain rule)**
  - Gradient descent, Jacobians, and nonlinear optimization all depend on multivariable calculus; both neural-network backpropagation and bundle adjustment in SLAM come down to differentiation.

> **Further reading**
> - [3Blue1Brown — Essence of Linear Algebra](https://www.youtube.com/playlist?list=PLZHQObOWTQDPD3MizzM2xVFitgF8hE_ab) — Visualizes the geometric intuition behind linear algebra before you dive into formal equations.
> - [3Blue1Brown — Essence of Calculus](https://www.youtube.com/playlist?list=PLZHQObOWTQDMsr9K-rj53DwVRMYO3t5Yr) — Builds visual intuition for calculus, especially why the chain rule and partial derivatives work as they do.
> - [Python for Data Analysis (Wes McKinney)](https://wesmckinney.com/book/) — The standard reference for NumPy, Pandas, and data-wrangling workflows, with a free online edition.

> **Technical Timeline: the Spatial AI field as a whole**
> - **~2005**: Classical robotics — mathematical model-based methods, Kalman filters, and EKF-SLAM; hand-crafted features and geometric formulations dominated.
> - **2007~2015**: The rise of real-time Visual SLAM — MonoSLAM (2007), PTAM (2007), and ORB-SLAM (2015) proved that real-time localization and mapping could run with a camera alone.
> - **2012~2018**: The deep learning revolution — starting with AlexNet (2012), recognition accuracy surged through ResNet (2015), Faster R-CNN (2015), and their successors, bringing learned components into Spatial AI.
> - **2020~2023**: The foundation model era — CLIP (2021), SAM (2023), and DINOv2 (2023) shifted perception away from per-environment data collection, labeling, and retraining toward broad zero-shot generalization.
> - **2024~**: End-to-end systems and embodied AI — VLA models, world models, and 3D Gaussian Splatting + SLAM blur the boundaries between perception, planning, and control, while deployed systems weigh end-to-end vs. modular architectures against safety, latency, and verification needs.
> - **Recent direction**: Active research focuses on open-vocabulary SLAM, VFM-driven scene understanding, and hybrid systems that pair classical geometry with learned priors.

With the broader landscape mapped out, the next step is to examine the physical hardware that anchors every Spatial AI system to the world: sensors (Ch.2), followed by the mathematical toolkit used to turn raw sensor streams into state estimates (Ch.3).

> **Interactive materials**: Interactive exercises for the key concepts in this document are available [here](https://alexjunholee.github.io/robotics-practice/).
