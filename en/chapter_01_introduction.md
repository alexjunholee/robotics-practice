# Ch.1 — Introduction: What Is Spatial AI?

Begin with a map of the Spatial AI field. It provides the context needed to see why the techniques in later chapters matter and how they connect.

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

All of these techniques face a common obstacle: uncertainty. Thrun, Burgard, Fox (2005) *Probabilistic Robotics*, §1.1 identifies five sources. First, the environment itself is inherently unpredictable. Second, every measurement carries resolution limits and noise. Third, motor torque variation and wheel slip mean the actual motion differs from the commanded motion — robot actuation is never exact. Fourth, the moment you abstract an environment or a robot into equations, the model is already an approximation. Fifth, under real-time constraints, approximate solutions replace optimal ones. These five sources are the shared motivation for every algorithm this guide covers, from Bayes filters to SLAM; the detailed treatment appears in Ch.3 §3.9–3.11 and Ch.14 §14.16.

> **Further reading**
> - [Andrew Davison — From SLAM to Spatial AI (MIT Robotics)](https://www.youtube.com/watch?v=BRRtlR0C_CY) — Prof. Andrew Davison's talk laying out the vision for Spatial AI. Worth watching to orient yourself in this field.
> - [FutureMapping paper (arXiv:1803.11288)](https://arxiv.org/abs/1803.11288) — A 2018 paper proposing a Spatial AI system that combines object-level mapping with prediction of future states.
> - [Cyrill Stachniss — Introduction to Mobile Robotics](https://www.youtube.com/playlist?list=PLgnQpQtFTOGQrZ4O5QzbIHgl3b1JHimN_) — Prof. Cyrill Stachniss's mobile robotics lectures at the University of Bonn. A well-organized treatment of the foundational concepts behind Spatial AI.

## 1.2 Why It Matters

The techniques to study in depth depend on the application domain. Autonomous driving places more emphasis on LiDAR and sensor fusion, whereas AR/VR relies heavily on visual-inertial systems. A view of the broader application landscape helps readers choose an appropriate learning path.

Spatial AI is the core technology in the following fields:

| Field | Example applications |
| --- | --- |
| **Autonomous driving** | Vehicle localization, obstacle detection, path planning |
| **Service robots** | Indoor navigation, object manipulation, human collaboration |
| **Drones** | Autonomous flight, 3D map generation, inspection/delivery |
| **AR/VR** | Spatial tracking, virtual object placement, hand tracking |
| **Industrial automation** | Logistics robots, quality inspection, assembly automation |

## 1.3 Why Robotics Is Hard

"Won't advances in AI solve every problem in robotics?" The answer is no: AI improves some parts of a robotic system, while many of the field's difficulties arise elsewhere.

Many of the difficulties in robotics arise at the interface with the physical world:

- A code bug that causes a collision can damage equipment or injure a person, so the outcome cannot be rolled back like a software deployment.
- Each experiment may require editing, uploading, resetting the environment, establishing safety, running, and physical inspection. One cycle can take minutes or tens of minutes.
- Sensor data contains backlight, motion blur, drift, and dropped frames. Performance on clean data alone does not predict behavior in the field.
- Functions such as obstacle avoidance must satisfy response-time requirements as well as accuracy requirements.
- A rare edge case can still lead to a collision or safety incident.
- A simulator approximates friction, inertia, and noise, and accumulated modeling error can change the behavior of a physical robot.

| General software | Robotics |
|---|---|
| Bug → log → fix → redeploy | Bug → crash → damage → repair → retry |
| Iteration in seconds | Iteration in minutes to hours |
| Structured inputs | Sensor data riddled with noise |
| 99% accuracy is excellent | 99.9999% may not be enough |
| Response lag → inconvenience | Response lag → accident |
| Same input → same output | Same code, different results depending on environment |

Advances in AI improve areas such as recognition accuracy and natural-language command understanding. The problems in the right column of the table—iteration speed, sensor noise, real-time constraints, and risk of damage—arise from interaction with the physical world. Increasing model scale alone does not resolve them, and training a single AI model does not produce a working robotic system.

### What Roboticists Do in the AI Era

In an era where AI writes code, summarizes papers, and proposes experiments, where does a roboticist's value lie?

- **Problem definition**: AI can help solve a stated problem, but it cannot determine which problem deserves attention. Choosing a sensor suite for an environment, deciding what accuracy an application requires, and accepting the right trade-offs all demand domain knowledge. (*The general framework for problem definition is covered in [Research Notes Ch.1](../../research-notes/guide.html#chapter-1) and [Research Notes Ch.2](../../research-notes/guide.html#chapter-2) (Korean only).*)
- **System integration**: Turning perception modules, control modules, communication stacks, and hardware into a single working system. AI can write code for each module, but designing the interfaces, timing, and exception handling between modules is the engineer's job.
- **Interface with the physical world**: A loose cable, dust on a sensor lens, or an overheating motor can be difficult to diagnose through a remote connection alone. Someone must inspect the robot and its hardware directly.
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

The mathematics explains the equations in a SLAM paper, while sensor characteristics explain why an algorithm fails under particular conditions. Studying these foundations in sequence reduces the gaps that otherwise appear in later chapters.

### Staged Learning Path

Below is a more concrete staged roadmap. Adjust the pace to your background, but don't skip any stage.

**Entry stage — learning the tools**

The goal of this stage is to become comfortable with the basic tools used in research. You should be able to read code, run it, and interpret the results.

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

**Intermediate stage — practicing the core techniques**

At this stage, you should be able to run the core Spatial AI algorithms yourself and analyze the results. It is also the stage where you start reading papers.

**What to learn**:
1. **Deep learning basics (PyTorch)** — tensors, autodiff, training loops, model design. In research, PyTorch dominates over TensorFlow.
2. **Object Detection (YOLO family)** — bounding boxes, NMS, mAP, and other basic concepts of a recognition pipeline.
3. **Understanding Visual SLAM (ORB-SLAM3)** — a representative feature-based SLAM system. Run it and inspect the code.
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
> - **2024~**: End-to-end systems and embodied AI — VLA models, world models, 3D Gaussian Splatting + SLAM, and related work reduce the boundaries between perception, planning, and control. Physical systems compare end-to-end and modular designs according to safety, latency, and verification requirements.
> - **Recent direction**: Work continues on open-vocabulary SLAM, VFM-based scene understanding, and systems that combine classical geometry with learned components.

> **Interactive materials**: Interactive exercises for the key concepts in this document are available [here](https://alexjunholee.github.io/robotics-practice/).
