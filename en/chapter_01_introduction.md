# Ch.1 — Introduction: What Is Spatial AI?

Sketch the full map of the Spatial AI field in your head first. Only then do the individual techniques in later chapters feel necessary, and you see how they connect to each other.

## 1.1 Defining Spatial AI

**Spatial AI** is the umbrella term for AI techniques that let machines understand 3D space and act within it. Beyond classifying images or recognizing objects, it must answer questions like:

- "Where am I right now?" (Localization)
- "What does the surrounding environment look like?" (Mapping)
- "What is that object, and where is it?" (Object Detection & Localization)
- "How do I get to the destination?" (Navigation & Planning)

General AI/deep learning answers "is there a cat in this image?", whereas Spatial AI must answer "how many meters away is that cat, which direction is it moving, and how do I move to avoid it?". That is, spatial context is the crux.

Spatial AI is the fusion of these techniques:
- **Computer Vision**: extracting information from camera images
- **3D Vision**: depth perception, point cloud processing
- **SLAM**: simultaneous localization and mapping
- **Deep Learning**: learning-based recognition and prediction
- **Sensor Fusion**: integrating information from multiple sensors

> **Further reading**
> - [Andrew Davison — From SLAM to Spatial AI (MIT Robotics)](https://www.youtube.com/watch?v=BRRtlR0C_CY) — Prof. Andrew Davison's talk laying out the vision for Spatial AI. Worth watching to orient yourself in this field.
> - [FutureMapping paper (arXiv:1803.11288)](https://arxiv.org/abs/1803.11288) — The paper that first systematized the Spatial AI concept.
> - [Cyrill Stachniss — Introduction to Mobile Robotics](https://www.youtube.com/playlist?list=PLgnQpQtFTOGQrZ4O5QzbIHgl3b1JHimN_) — Prof. Cyrill Stachniss's mobile robotics lectures at the University of Bonn. A well-organized treatment of the foundational concepts behind Spatial AI.

## 1.2 Why It Matters

The techniques you need to dig into depend on which domain you want to work in. For autonomous driving, focus on LiDAR and sensor fusion; for AR/VR, dig into visual-inertial systems. Knowing the full application landscape is what lets you build your own roadmap.

Spatial AI is the core technology in the following fields:

| Field | Example applications |
| --- | --- |
| **Autonomous driving** | Vehicle localization, obstacle detection, path planning |
| **Service robots** | Indoor navigation, object manipulation, human collaboration |
| **Drones** | Autonomous flight, 3D map generation, inspection/delivery |
| **AR/VR** | Spatial tracking, virtual object placement, hand tracking |
| **Industrial automation** | Logistics robots, quality inspection, assembly automation |

## 1.3 Why Robotics Is Hard

"Won't robotics be solved once AI advances?" — a question I hear often. It won't. What AI improves and what makes robotics hard are different things.

Most of the difficulty in robotics arises at the interface with the physical world:

- **No undo.** A code bug that triggers a robot collision damages equipment or injures people. There is no rollback.
- **Iteration is slow.** Edit code → upload → reset the environment → ensure safety → run → physically verify. One cycle takes minutes to tens of minutes.
- **Sensor data is always dirty.** Backlight, motion blur, drift, frame drops. "Works well on clean data" is meaningless in robotics.
- **Real-time constraints apply.** If obstacle avoidance is 200 ms late, you collide. High accuracy is useless without speed.
- **Edge cases are fatal.** An LLM that is wrong 5 times out of 100 is still useful. An autonomous vehicle that is wrong once in a million is an accident.
- **There is a sim-to-real gap.** A simulator's friction coefficients, inertia, and noise are approximations. When those errors accumulate, behavior on the real robot diverges.

In summary:

| General software | Robotics |
|---|---|
| Bug → log → fix → redeploy | Bug → crash → damage → repair → retry |
| Iteration in seconds | Iteration in minutes to hours |
| Structured inputs | Sensor data riddled with noise |
| 99% accuracy is excellent | 99.9999% may not be enough |
| Response lag → inconvenience | Response lag → accident |
| Same input → same output | Same code, different results depending on environment |

There are clearly areas that AI advances will improve: recognition accuracy, decision quality, natural language command understanding, and so on. But the right column of the table above — iteration speed, sensor noise, real-time constraints, breakage risk — belongs to the domain of physical law. Scaling models doesn't solve it.

This is why this document covers math, sensors, SLAM, and calibration. Training one AI model alone won't make a robot work.

### What Roboticists Do in the AI Era

In an era where AI writes code, summarizes papers, and proposes experiments, where does a roboticist's value lie?

- **Problem definition**: tell AI to "solve this problem" and it solves it, but it cannot judge "which problem should be solved". Which sensor combination suits this environment, which accuracy is sufficient for this application, which trade-offs are acceptable — only someone who knows the domain can judge these. (*The general frame for problem definition is covered in [`../../survival-research/part0_starting/ch01_review_paper.md`](../../survival-research/part0_starting/ch01_review_paper.md) and [`ch02_feynman_problem.md`](../../survival-research/part0_starting/ch02_feynman_problem.md) *(Korean; English version planned)*.*)
- **System integration**: Turning perception modules, control modules, communication stacks, and hardware into a single working system. AI can write code for each module, but designing the interfaces, timing, and exception handling between modules is the engineer's job.
- **Interface with the physical world**: Whether a cable came loose, whether dust settled on a sensor lens, whether a motor overheated — AI cannot solve problems it cannot reach via ssh. You need someone standing in front of the robot, touching it with their hands.
- **Judging reliability**: Even when AI reports "99% accuracy", whether that 1% leads to a safety incident is something an engineer must judge. 99% is not enough in autonomous driving, but 99% may be enough for an indoor serving robot.

Even when AI writes the code and summarizes the papers, it can't do these four things for you.

## 1.4 How to Use This Document

Use this document as a **reference**:

1. **On first read**: skim the table of contents and grasp the overall picture.
2. **When starting research**: read the relevant sections in depth and work through the further reading.
3. **When stuck**: consult the glossary and troubleshooting in the appendix.

**Recommended study order**:

```
Mathematical foundations → Sensors → Computer vision basics → SLAM → Deep learning → VFM/VLA → Lab direction
```

Read a SLAM paper without the math and you get stuck on the equations; without knowing sensor characteristics, you cannot understand why an algorithm fails in particular situations. Building up from the fundamentals in order is ultimately the fast path.

### Staged Learning Path

Below is a more concrete staged roadmap. Adjust the pace to your background, but don't skip any stage.

**Entry stage — getting tools under your fingers**

The goal of this stage is to handle the basic tools research requires with full fluency. You should be able to read code, run it, and interpret the results.

**What to learn**:
1. **Reading C++ code** — the lab's core code (SLAM, ROS packages) is in C++. You don't need to write it from scratch at first, but you need to be able to read and modify its structure.
2. **Python basics** — used for deep learning training scripts, data preprocessing, and visualization. A language well-suited to AI-agent assistance.
3. **Linear algebra and probability/statistics refresh** — reorganizing what you learned as an undergraduate from a robotics perspective. Refer to Ch.3.
4. **ROS2 basics** — topics, services, actions, launch files. The robot framework used in the lab.
5. **Git usage** — up through branch, merge, rebase. Code management in the lab goes through Git.

**Practice exercises**:
- Build an image processing pipeline with OpenCV (read → filter → feature extraction → visualization)
- Write a simple ROS2 node (publisher/subscriber)
- Perform a camera calibration (using a chessboard pattern)

**Intermediate stage — internalizing the core techniques**

At this stage, you should be able to run the core Spatial AI algorithms yourself and analyze the results. It is also the stage where you start reading papers.

**What to learn**:
1. **Deep learning basics (PyTorch)** — tensors, autodiff, training loops, model design. In research, PyTorch dominates over TensorFlow.
2. **Object Detection (YOLO family)** — bounding boxes, NMS, mAP, and other basic concepts of a recognition pipeline.
3. **Understanding Visual SLAM (ORB-SLAM3)** — the flagship algorithm for feature-based SLAM. Run it and tear into the code.
4. **Point cloud processing (Open3D)** — how to handle 3D data. Filtering, registration, visualization.
5. **Understanding and using VFMs (DINOv2, SAM)** — understanding how foundation models reshape existing pipelines.

**Practice exercises**:
- Run benchmark experiments on the KITTI dataset
- Fine-tune YOLOv8 (on a custom dataset)
- Run ORB-SLAM3 and analyze the trajectory
- Evaluate SLAM accuracy on the TUM RGB-D dataset

**Advanced stage — first steps as a researcher**

From this stage onward, you experience the full research cycle: reading papers, generating ideas, running experiments, and writing.

**What to learn**:
1. **Reading and implementing papers** — at least 1-2 papers per week. For key papers, analyze down to the code.
2. **Experimenting with new ideas** — identify limitations of existing methods and experiment with improvement ideas.
3. **Benchmark evaluation** — master evaluation protocols for fair comparison.

**Practice exercises**:
- Analyze and reproduce code from recent papers
- Experiment with your own improvement ideas and compare quantitatively
- Attempt paper writing (targeting conference workshop submission)

> **Further reading**
> - [Missing Semester of Your CS Education (MIT)](https://missing.csail.mit.edu/) — An MIT course that teaches the practical tools research demands: Git, shell, debugging, and more.
> - [ROS2 official tutorials](https://docs.ros.org/en/humble/Tutorials.html) — Official learning materials based on ROS2 Humble.
> - [Andrej Karpathy — Neural Networks: Zero to Hero](https://www.youtube.com/playlist?list=PLAqhIrjkxbuWI23v9cThsA9GvCAUhRvKZ) — An outstanding series that teaches deep learning by implementing it from scratch.

## 1.5 Prerequisite Checklist

Before starting research, check the following items. Each entry also notes why it is necessary, so don't just check boxes — understand "why this is needed" before moving on.

**Required**:
- [ ] **Ability to read C++**
  - The lab's core code (SLAM, real-time control, ROS packages) is in C++. To understand and modify open-source projects like ORB-SLAM3 and LOAM, you must be comfortable with C++.
- [ ] **Basic Linux commands (cd, ls, cp, mv, grep)**
  - Lab servers are almost 100% Ubuntu. To SSH into a GPU server and run experiments, you need to be comfortable in the terminal.
- [ ] **Basic Git usage (clone, commit, push, pull)**
  - Research code management, pulling paper code, sharing code within the lab — all through Git. Cloning open-source code from GitHub and running it is daily work.

**Recommended**:
- [ ] **Python basics (functions, classes, modules)**
  - Used for deep learning training scripts, data preprocessing, and visualization. Since it is a language AI agents handle well, the need to write it directly is decreasing, but you must be able to read and understand it.
- [ ] **Basic NumPy usage**
  - Matrix operations, broadcasting, indexing. Used for sensor data processing and coordinate transformations.
- [ ] **Linear algebra basics (matrix operations, eigenvalues)**
  - 3D transformations, camera models, and optimization are all linear algebra. To understand "what this equation means", you need to know the geometric meaning of matrices. Continued in Ch.3.
- [ ] **Probability/statistics basics (normal distribution, Bayes' theorem)**
  - Sensor noise modeling, state estimation, and filtering are all probability-based. Expressing "how much can I trust this sensor's measurement?" mathematically requires this knowledge.
- [ ] **Calculus basics (partial derivatives, chain rule)**
  - The foundation of gradient descent, Jacobians, and optimization algorithms. Backpropagation in deep learning and bundle adjustment in SLAM both come down to differentiation.

> **Further reading**
> - [3Blue1Brown — Essence of Linear Algebra](https://www.youtube.com/playlist?list=PLZHQObOWTQDPD3MizzM2xVFitgF8hE_ab) — A video series that explains the geometric intuition of linear algebra well. Helpful before diving into the equations.
> - [3Blue1Brown — Essence of Calculus](https://www.youtube.com/playlist?list=PLZHQObOWTQDMsr9K-rj53DwVRMYO3t5Yr) — Intuitive understanding of calculus. Shows visually why the chain rule and partial derivatives matter.
> - [Python for Data Analysis (Wes McKinney)](https://wesmckinney.com/book/) — The standard text on NumPy, Pandas, and other data analysis tools. Free online version available.

> **Technical Timeline: the Spatial AI field as a whole**
> - **~2005**: Classical robotics — mathematical model-based, Kalman filter, EKF-SLAM. Hand-crafted features and geometric methods dominated.
> - **2007~2015**: The rise of real-time Visual SLAM — MonoSLAM (2007), PTAM (2007), ORB-SLAM (2015). Real-time localization and map generation became possible with a camera alone.
> - **2012~2018**: The deep learning revolution — starting with AlexNet (2012), recognition performance surged with ResNet (2015), Faster R-CNN (2015), and others. Learning-based methods started entering Spatial AI as well.
> - **2020~2023**: The foundation model era — CLIP (2021), SAM (2023), DINOv2 (2023), and other large-scale pretrained models appeared. Previously, every new environment required repeated data collection → labeling → training; the tasks that can be handled zero-shot have grown sharply.
> - **2024~**: End-to-end systems and embodied AI — VLA (vision-language-action) models, world models, 3D Gaussian Splatting + SLAM, and more. Attempts to unify perception, planning, and control in a single model are emerging, but as of 2026 most deployed systems remain modular.
> - **What to watch now**: Research grafting foundation models onto robot perception (e.g., open-vocabulary SLAM, VFM-based scene understanding) grew noticeably at CVPR/ICRA 2025. The thread of combining classical geometry with learning-based methods also continues.

> **Interactive materials**: Interactive exercises for the key concepts in this document are available [here](https://alexjunholee.github.io/robotics-practice/).
