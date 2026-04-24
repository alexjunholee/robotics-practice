# Ch.14 — SLAM & Odometry


The problem of a robot figuring out "where am I, and what does the surrounding environment look like?" at the same time, in an unfamiliar environment, is simultaneous localization and mapping (SLAM). When a robot moves autonomously in GPS-denied spaces — indoors, underground, inside a building — SLAM is not optional but essential. It is one of the skills most frequently required of a robotics software engineer, so both theory and practice need a solid foundation.

---

## Part 1. Foundations and Systems

### 14.1 Concept Introduction

Run a robot without a map and you feel it right away. A robot that does not know where it is cannot do anything. Navigation, obstacle avoidance, path planning — every one of them presupposes "current position" and "information about the surrounding environment."

**SLAM (simultaneous localization and mapping)**:
The problem of estimating one's own pose while at the same time building a map of the surrounding environment.

Chicken-and-egg problem:
- You need a map to know your position
- You need your position to build a map
→ Solve both at once

Sensors always carry noise. Wheels slip, camera images shake. This uncertainty accumulates over time and the pose estimate gradually drifts (drift). The core challenge of SLAM is to correct this drift and produce a consistent map.

**Odometry vs SLAM**:
| Feature | Odometry | SLAM |
|---|---|---|
| Output | Relative motion | Pose + map |
| Loop closure | None | Present |
| Drift | Accumulates | Can be corrected |
| Compute | Light | Heavy |

> **Further reading**
> - [Cyrill Stachniss — SLAM Course (University of Bonn)](https://www.youtube.com/playlist?list=PLgnQpQtFTOGQrZ4O5QzbIHgl3b1JHimN_) — The canonical SLAM lecture series. If you are learning SLAM for the first time, watch this series.
> - [Thrun, Burgard, Fox, "Probabilistic Robotics" (Textbook)](https://mitpress.mit.edu/9780262201629/probabilistic-robotics/) — Textbook covering the mathematical foundations of SLAM. Kalman filter, particle filter, EKF-SLAM, and more.
> - [Barfoot, "State Estimation for Robotics" (Free PDF)](http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) — Textbook with deep coverage of the mathematics of state estimation. Free PDF available.
> - [Awesome-SLAM GitHub](https://github.com/SilenceOverflow/Awesome-SLAM) — Curated list of SLAM-related papers, libraries, and datasets.
> - [Jinyong Jeong's blog — SLAM lecture series (based on Freiburg Robot Mapping)](https://jinyongjeong.github.io/2017/02/13/lec01_SLAM_bayes_filter/) — A 15-part series covering Bayes filter through EKF/UKF/particle filter, Graph SLAM, and Robust SLAM. The most systematic Korean-language introduction to SLAM.
> - [Giseop Kim's blog — 5 recommended study materials for SLAM back-end](https://gisbi-kim.github.io/blog/2021/10/03/slam-textbooks.html) — A curated list of core materials including Error-state KF, Factor Graphs, and Bundle Adjustment.
> - [Robot Mapping Course (Uni Freiburg, Cyrill Stachniss)](http://ais.informatik.uni-freiburg.de/teaching/ws13/mapping/) — Lecture slides and assignments for the SLAM course. Pairs well with the video lectures.
> - [EKF-SLAM slides (Freiburg)](http://ais.informatik.uni-freiburg.de/teaching/ws12/mapping/pdf/slam04-ekf-slam.pdf) — The EKF-SLAM portion of the course above. The derivations are cleanly organized.

> **Practice**: [SE(2) Odometry](https://alexjunholee.github.io/robotics-practice/app.html#se2_odometry)
> Interactively drive the odometry accumulation process on a 2D plane and observe how drift arises.

### 14.2 Visual Odometry (VO)

Estimate relative motion from the camera alone. This corresponds to the SLAM "front-end"; if the motion estimated here is inaccurate, the entire SLAM system falls apart.

#### 14.2.1 Feature-based vs Direct Method

These two approaches have sharply different strengths and weaknesses. The choice depends on the environment in which you operate the robot.

**Feature-based** methods (ORB-SLAM family) extract invariant distinctive points (corners, blobs, and so on) from images and infer camera motion by matching them between frames. In linear-algebra terms, it reduces to solving for the essential matrix or the fundamental matrix. They are robust to illumination changes and the methodology is well-established, but they have limits in environments where keypoints are hard to extract, such as white walls or textureless floors.

```
Image → Feature extraction → Matching → Motion estimation
```

**Direct methods** (DSO, LSD-SLAM family) compare pixel intensities directly. They exploit the assumption that "if the same 3D point is observed in consecutive frames, the intensity must be the same" (brightness constancy), so they do not need to extract keypoints and can operate even in low-texture environments. In return, they are sensitive to illumination changes.

```
Image → Direct pixel intensity comparison → Motion estimation
```

#### 14.2.2 Mono vs Stereo vs RGB-D

You have to understand each configuration's trade-offs to choose a sensor that fits your robot.

| Configuration | Scale | Characteristics | Suitable environment |
|---|---|---|---|
| **Monocular** | Unavailable (ambiguity) | Light and simple; scale cannot be recovered without an IMU | Low-cost drones, mobile |
| **Stereo** | Available | Baseline limits the measurement range | General indoor/outdoor |
| **RGB-D** | Available | Measures depth directly; weak outdoors and under direct sunlight | Indoor structured environments |

To elaborate on scale ambiguity: with a single camera you cannot tell "a small object up close" from "a large object far away." A monocular SLAM map comes out at an arbitrary scale, and an IMU or another sensor must recover it. This is also why sufficient motion is required during initialization.

> **Further reading**
> - [Daniel Cremers — Multiple View Geometry (TUM)](https://www.youtube.com/playlist?list=PLTBdjV_4f-EJn6udZ34tht9EVIW7lbeo4) — The best resource for learning the mathematical foundations of Visual Odometry.
> - [EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) — Benchmark dataset for Visual(-Inertial) Odometry.
> - [TUM RGB-D Benchmark](https://cvg.cit.tum.de/data/datasets/rgbd-dataset) — The standard benchmark for RGB-D SLAM/VO.

### 14.3 Visual SLAM

#### 14.3.1 ORB-SLAM2/3

ORB-SLAM is the standard baseline for Visual SLAM. Most Visual SLAM papers compare against ORB-SLAM, and because the code is open-sourced you can build and run it yourself. If you are studying SLAM, doing this at least once is recommended.

**Structure**:
1. **Tracking**: Pose estimation on the current frame
2. **Local Mapping**: Keyframe-based local map management
3. **Loop Closing**: Loop detection and global optimization

This three-thread structure is the core design of ORB-SLAM. Tracking runs in real-time on every frame, Local Mapping runs when a keyframe arrives, and Loop Closing runs when a loop is detected. Each runs in parallel at a different rate, allowing real-time performance while preserving global consistency.

**ORB-SLAM3 features**:
- Visual-inertial mode supported
- Multi-map supported
- Fish-eye cameras supported

Historical context of ORB-SLAM:
- **MonoSLAM (2007)**: The first real-time monocular SLAM. It ran on an EKF, but suffered from compute blowup as the map grew.
- **PTAM (Parallel Tracking and Mapping, 2007)**: The first system to split tracking and mapping into separate threads. This architecture had a strong influence on later ORB-SLAM.
- **ORB-SLAM (2015)**: A complete SLAM system that inherited PTAM's design and added ORB keypoints, loop closure, and relocalization.
- **ORB-SLAM2 (2017)**: Added stereo and RGB-D support.
- **ORB-SLAM3 (2021)**: Added visual-inertial, multi-map, and more.

```bash
# ORB-SLAM3 run example
./Examples/Monocular/mono_euroc \
    Vocabulary/ORBvoc.txt \
    Examples/Monocular/EuRoC.yaml \
    ~/Datasets/EuRoC/MH01
```

> **Further reading**
> - [Campos et al., "ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM" (2021)](https://arxiv.org/abs/2007.11898) — The ORB-SLAM3 paper.
> - [ORB-SLAM3 GitHub](https://github.com/UZ-SLAMLab/ORB_SLAM3) — Official code.
> - [EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) — Standard test dataset for ORB-SLAM3.
> - [Jinyong Jeong's blog — Visual SLAM comparison experiment (KAIST Urban Dataset)](https://jinyongjeong.github.io/2019/10/22/visual_slam_compare/) — Head-to-head ORB-SLAM2 vs VINS-Fusion on real data. Analyzes performance differences on actual datasets.

#### 14.3.2 DSO (Direct Sparse Odometry)

**Direct Method** + **Sparse Points**

Direct methods are often used densely (every pixel), and sparse representations are typical in feature-based methods, but DSO takes the combination of "direct and sparse." It minimizes photometric error using only a selected set of high-quality points.

- Uses pixel intensities directly, no keypoint extraction
- Uses only selected points (sparse)
- Photometric bundle adjustment

> **Further reading**
> - [Engel et al., "Direct Sparse Odometry" (2018)](https://arxiv.org/abs/1607.02565) — The DSO paper.

#### 14.3.3 VINS-Mono/Fusion

Run it yourself and you will see: with a camera alone, tracking easily fails under fast motion or in textureless environments. Combining an IMU keeps the system stable in these situations. It is the most widely used visual-inertial SLAM system on real drones and mobile robots.

**Visual-Inertial Navigation System**

- Camera + IMU tight coupling
- Sliding window optimization
- Loop closure supported
- Widely used on mobile robots and drones

```
Sensor input → IMU Preintegration →
Visual Feature Tracking →
Sliding Window Optimization →
Loop Closure (optional)
```

Core contribution of VINS-Mono: with a technique called IMU preintegration, it compresses the hundreds of IMU measurements between two keyframes into a single relative transformation. You then no longer need to handle every IMU measurement during optimization; you just add the one compressed constraint. The gain in compute efficiency is substantial.

> **Further reading**
> - [Qin et al., "VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator" (2018)](https://arxiv.org/abs/1708.03852) — The VINS-Mono paper.
> - [VINS-Mono GitHub](https://github.com/HKUST-Aerial-Robotics/VINS-Mono) — Official code, with ROS support.

### 14.4 LiDAR Odometry & SLAM

Camera-based methods are sensitive to illumination and texture, whereas LiDAR measures 3D range directly and is free from these issues. In autonomous driving and outdoor robotics, LiDAR SLAM is effectively the standard.

#### 14.4.1 LOAM (Lidar Odometry and Mapping)

LOAM is the starting point of LiDAR SLAM. LeGO-LOAM, LIO-SAM, FAST-LIO, and nearly every LiDAR SLAM system that followed inherits or extends LOAM's ideas.

- Classify edge points and planar points
- Minimize point-to-edge and point-to-plane distances
- Separate odometry and mapping (at different rates)

It uses only the geometrically meaningful points (corners and planes) extracted from the point cloud. Matching every point is slow and fragile against noise, but picking only edge/planar points is both fast and accurate.

#### 14.4.2 LeGO-LOAM

**Lightweight and Ground-Optimized LOAM**:
- Ground separation reduces compute
- Uses the ground plane for an initial estimate
- Suited to mobile robots

#### 14.4.3 LIO-SAM

A representative work that applies factor graph-based optimization to LiDAR-inertial SLAM. The core strength of a factor graph is extensibility. To add one more sensor, you just add one factor.

**LiDAR-Inertial Odometry via Smoothing and Mapping**:
- Factor graph based
- Tight IMU-LiDAR coupling
- Integrates GPS and loop closure

```
                    ┌──────────────┐
IMU ──────────────→ │              │
                    │ Factor Graph │ ──→ Pose
LiDAR ────────────→ │              │
                    │  iSAM2       │
GPS (optional) ───→ │              │
                    └──────────────┘
```

What a factor graph is: a graph that represents the relationships between variables (robot poses, landmark positions) and constraints (sensor measurements). An IMU measurement is one factor, a LiDAR match is one factor, GPS is one factor, a loop closure is one factor... To add a sensor, you simply add its factor. The GTSAM library carries out this optimization efficiently.

> **Further reading**
> - [Shan et al., "LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping" (2020)](https://arxiv.org/abs/2007.00258) — The LIO-SAM paper.
> - [Vizzo et al., "KISS-ICP: In Defense of Point-to-Point ICP" (RA-L 2023, arXiv:2209.15397)](https://arxiv.org/abs/2209.15397) — A well-built vanilla ICP matches complex LiDAR odometry in performance. The power of simplicity.
> - [LIO-SAM GitHub](https://github.com/TixiaoShan/LIO-SAM) — Official code, with ROS support.
> - [GTSAM Documentation](https://gtsam.org/) — Factor graph optimization library. Used as the back-end of many SLAM systems including LIO-SAM and ORB-SLAM3.
> - [Frank Dellaert — Factor Graphs for Perception and Action (MIT Robotics)](https://www.youtube.com/watch?v=-yCC7mpgL4w) — The GTSAM developer explaining factor graphs himself.
> - [Giseop Kim's blog — Scan Context-based LiDAR Pose-graph SLAM implementation](https://gisbi-kim.github.io/blog/2021/05/17/sclidarslam.html) — A walk-through of integrating Scan Context into LiDAR SLAM.

#### 14.4.4 FAST-LIO / FAST-LIO2

**Fast LiDAR-Inertial Odometry**:
- Kalman filter based (instead of optimization)
- ikd-Tree: dynamic KD-tree for fast mapping
- Real-time performance

Why FAST-LIO is fast: LIO-SAM uses factor graph optimization (nonlinear least squares), whereas FAST-LIO uses an iterated extended Kalman filter (IEKF). It does not solve an optimization problem, just filtering, so compute is much lighter. It also uses an incremental KD-tree called ikd-Tree, which keeps insertion of new points into the map fast.

> **Further reading**
> - [Xu & Zhang, "FAST-LIO: A Fast, Robust LiDAR-Inertial Odometry Package by Tightly-Coupled Iterated Kalman Filter" (2021)](https://arxiv.org/abs/2010.14709) — The FAST-LIO paper.
> - [Xu et al., "FAST-LIO2: Fast Direct LiDAR-Inertial Odometry" (2022)](https://arxiv.org/abs/2107.06829) — The FAST-LIO2 paper.
> - [FAST-LIO2 GitHub](https://github.com/hku-mars/FAST_LIO) — Official code.

### 14.5 Multi-sensor Fusion

A single sensor struggles to cover every situation. A camera does not like the dark, a LiDAR has a hard time in the rain, and an IMU alone drifts heavily. Combining sensors (fusion) lets each sensor compensate for the others' weaknesses.

#### 14.5.1 Camera + IMU (VIO)

There are two strategies for combining visual and inertial. **Loosely-coupled** has the camera and the IMU each estimate state separately and then fuses the results based on covariance. Implementation is simple, but it fails to exploit the information fully. **Tightly-coupled** puts the reprojection error of camera keypoints and the acceleration/angular-velocity measurements of the IMU into a single cost function and optimizes them jointly (VINS-Mono, MSCKF). More accurate, but more complex to implement.

**IMU Preintegration**:
Pre-integrate IMU measurements between two keyframes to compute a relative transformation. Optimization can proceed without relinearization.

#### 14.5.2 LiDAR + IMU (LIO)

A LiDAR scans at 10–20 Hz, but if the robot moves fast it travels within a single scan (motion distortion). The basic structure of LIO is to de-skew the within-scan motion using an IMU that measures at 200–400 Hz, then perform precise matching with the LiDAR. This is why LIO beats LiDAR-only in high-speed motion.

#### 14.5.3 Camera + LiDAR + IMU

**Recent trend**: integrate all sensors
- Examples: R3LIVE, LVI-SAM
- Exploits each sensor's strengths

R3LIVE combines LiDAR (geometric information) + camera (texture/color information) + IMU (high-speed motion compensation). It produces not just accurate pose estimation but also a colored, high-density 3D map in real time.

> **Further reading**
> - [KITTI Odometry Benchmark](https://www.cvlibs.net/datasets/kitti/eval_odometry.php) — The standard benchmark for LiDAR/Visual Odometry.
> - [EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) — Benchmark dataset for VIO.
> - [Lin & Zhang, "R3LIVE: A Robust, Real-time, RGB-colored, LiDAR-Inertial-Visual tightly-coupled state Estimation and mapping package" (2022)](https://arxiv.org/abs/2109.07982) — A representative three-sensor fusion work.
> - [Giseop Kim's blog — Filter-based VIO: a history of the MSCKF family](https://gisbi-kim.github.io/blog/2021/04/27/msckf-history.html) — From the original MSCKF to stereo extensions, a lineage summary.

### 14.6 Loop Closure & Global Optimization

Run SLAM and you will see the map twist more and more over time. The robot walks a large loop back to the starting point, but on the map the start and the end no longer align. Loop closure is the core mechanism that corrects this twist. Without it, SLAM in large-scale environments is essentially impossible.

#### 14.6.1 Place Recognition

Recognize previously visited places to correct drift.

The same place looks entirely different when the time of day, lighting, or season changes. If you mistake a similar-looking different place for the same place (a false positive), the map gets even worse. This is why the precision of place recognition has to be very high.

**Bag of Words (BoW)**: computes similarity between images based on a visual vocabulary. The DBoW2 library is the representative and is used in ORB-SLAM. Fast and well-tested, but fragile against illumination and viewpoint changes.

**NetVLAD**: a deep learning-based end-to-end trained global descriptor that is robust against illumination and weather changes. (see Section 14.14)

**LiDAR Place Recognition**: Scan Context compresses a point cloud into a 2D bird-eye-view descriptor, and PointNetVLAD learns directly from point clouds.

#### 14.6.2 Pose Graph Optimization

When a loop is detected, correct the entire trajectory.

```
Nodes: robot poses
Edges: relative transformations (odometry, loop closure)

Goal: find node positions that satisfy all edge constraints
```

Intuitively: a trajectory built from odometry is "locally roughly correct but globally twisted." When loop closure adds a constraint that "this place and that place are the same spot," pose graph optimization "smoothly adjusts the entire trajectory to satisfy all the constraints as well as possible." This is a nonlinear least squares problem.

The main tools are **g2o**, a lightweight library dedicated to pose graph / BA (ORB-SLAM); **GTSAM**, based on factor graphs and iSAM2 (LIO-SAM); and **Ceres Solver**, a general-purpose nonlinear least squares library developed by Google. For the selection criteria, refer to the comparison table in Section 14.9.4.

> **Further reading**
> - [GTSAM Documentation & Tutorials](https://gtsam.org/) — Factor graph-based optimization library. Includes pose graph optimization examples.
> - [Cyrill Stachniss — Graph-based SLAM](https://www.youtube.com/watch?v=uHbRKvD8TWg) — Intuitive explanation of pose graph optimization.
> - [g2o GitHub](https://github.com/RainerKuemmerle/g2o) — Graph optimization framework.
> - [Jinyong Jeong's blog — Robust Graph SLAM](https://jinyongjeong.github.io/2017/03/04/lec15_Robust_Graph_SLAM/) — Korean-language walkthrough of robust SLAM techniques including M-estimators, max-mixture, and DCS.

> **Practice**: [Pose Graph Optimization](https://alexjunholee.github.io/robotics-practice/app.html#pose_graph)
> Manipulate the nodes (poses) and edges (constraints) of a pose graph and observe how the trajectory is corrected when you add a loop closure.

### 14.7 Localization

Estimate the current pose given a prior map. If SLAM is "estimate pose while building a map," then localization is "estimate pose only, in an already built map." In practice, service robots often build a map with SLAM ahead of time and then run only localization during operation.

Map-based localization uses a pre-built map and is thus lighter than SLAM, but the map has to be updated when the environment changes.

**Monte Carlo Localization (MCL)**:
- Particle filter based
- 2D LiDAR + occupancy grid map
- ROS AMCL package

Intuition of MCL: scatter thousands of "virtual robots (particles)" across the map. Each particle is a hypothesis of the form "I am here, facing this direction." Compare against the actual sensor measurements; particles that match survive, and the ones that don't match die out. Over time, the particles cluster around the true position.

**LiDAR Localization**: estimate pose precisely by matching against a point cloud map with ICP or NDT.

> **Further reading**
> - [Cyrill Stachniss — Monte Carlo Localization](https://www.youtube.com/watch?v=MsYlueVDLI0) — Intuitive explanation of MCL/particle filter.
> - [ROS Navigation Stack — AMCL](http://wiki.ros.org/amcl) — Using MCL in ROS.

> **Practice**: [Particle Filter](https://alexjunholee.github.io/robotics-practice/app.html#particle_filter)
> Visualize the process of particle filter-based robot localization and observe particle convergence interactively.

> **Practice**: [Occupancy Grid](https://alexjunholee.github.io/robotics-practice/app.html#occupancy_grid)
> Visualize the construction of a 2D occupancy grid map and observe how sensor measurements turn into a probabilistic map.

---

## Part 2. Recent Trends

### 14.8 Learning-based & Neural SLAM

Traditional SLAM uses hand-designed keypoints, matching algorithms, and optimization pipelines. Recent work has been replacing part or all of this pipeline with deep learning.

**DROID-SLAM (2021)**:
- SLAM based on dense recurrent optical flow
- Without keypoint extraction/matching, it iteratively refines dense optical flow to jointly estimate camera pose and depth
- Improved robustness in settings where existing methods fail, such as textureless environments and illumination changes
- Uses a differentiable dense bundle adjustment (DBA) layer for end-to-end training

Why DROID-SLAM drew attention: existing feature-based SLAM (ORB-SLAM) fails in keypoint-starved environments, and direct methods (DSO) are weak against illumination changes. DROID-SLAM uses learned representations, and as a result it overcomes these limits to a significant degree. That said, it requires a GPU, and its real-time performance does not always match the older methods.

**3DGS-SLAM fusion**:
The 3D Gaussian Splatting covered in 13.5.2 is also being used as the map representation in SLAM. SplaTAM and MonoGS are representative examples; they replace the sparse/dense point maps of classic SLAM with 3D Gaussians as the environment representation. The scene's visual fidelity improves, and rendering-based applications (virtual view synthesis, AR overlays, and so on) become possible.

> **Further reading**
> - [Teed & Deng, "DROID-SLAM: Deep Visual SLAM for Monocular, Stereo, and RGB-D Cameras" (2021)](https://arxiv.org/abs/2108.10869) — The DROID-SLAM paper.
> - [Keetha et al., "SplaTAM" (2024)](https://arxiv.org/abs/2312.02126) — Dense SLAM based on 3DGS.
> - [Awesome-SLAM GitHub](https://github.com/SilenceOverflow/Awesome-SLAM) — Collection of recent SLAM papers and projects.

---

## Part 3. Advanced

### 14.9 Advanced: SLAM Back-end Optimization

*If you want to become a researcher, start reading here.*

The SLAM front-end processes sensor data to produce constraints; the back-end finds the optimal state (poses, landmarks) that jointly satisfies these constraints. This process is a nonlinear least squares problem. What we cover here is the mathematical background needed to understand "why you configure libraries like g2o, GTSAM, and Ceres the way you do."

**Intuition for the problem the SLAM back-end solves**

Before looking at complicated equations, remember one thing: the SLAM back-end ultimately **solves Ax = b**.

The robot produces two kinds of data as it drives:
1. **Odometry**: "I moved 1 m forward" (relative motion)
2. **Observations**: "that landmark is visible at 3 m"

You want to find poses and landmark positions that satisfy all of these measurements, but because of sensor noise no solution satisfies them perfectly. Instead, you look for the solution that "minimizes the sum of squared errors against all the measurements." This is the nonlinear least squares problem, and solving it efficiently is the role of the SLAM back-end.

Because it is nonlinear you cannot solve it in one shot; you linearize around the current estimate and update iteratively. This loop of "linearize → solve Ax=b → update → repeat" is Gauss-Newton.

(Reference: [Giseop Kim's blog — SLAM back-end series](https://gisbi-kim.github.io/blog/2021/03/04/slambackend-1.html))

#### 14.9.1 Gauss-Newton on a Manifold

The state variable in SLAM (a pose) lives on SE(3). SE(3) is not a Euclidean space but a Lie group, so you cannot simply apply the usual Gauss-Newton update `x ← x + δx`. Adding a vector to a rotation matrix no longer produces a rotation matrix.

The fix is to define the perturbation on the Lie algebra se(3).

**Update step (left perturbation)**:
```
T ← exp(δξ^) · T
```
Here `δξ ∈ R^6` is a small perturbation on se(3), `exp(·)` is the exponential map, and `^` (the hat operator) converts a 6-vector into a 4x4 matrix.

**Jacobian computation**: Compute the Jacobian of the error function `e(T)` with respect to `δξ`.
```
J = ∂e / ∂δξ
```
By the chain rule this becomes `∂e/∂T · ∂T/∂δξ`, where `∂T/∂δξ` is the left Jacobian of SE(3).

**Normal equation**:
```
(J^T Σ^{-1} J) δξ* = -J^T Σ^{-1} e
```
- `Σ` is the measurement noise covariance
- `H = J^T Σ^{-1} J` is the Gauss-Newton approximation of the Hessian; this is the **information matrix**
- With multiple constraints, sum the per-constraint `J^T Σ^{-1} J` (additive property)

Iterate this process until convergence. At every iteration, recompute the Jacobian at the current estimate and apply the update.

#### 14.9.2 Schur Complement (Marginalization)

In bundle adjustment (BA) the state variables are of two kinds: camera poses (p) and landmarks (l). The Hessian `H` of the normal equation has the following block structure:

```
[H_pp  H_pl] [δp]   [b_p]
[H_lp  H_ll] [δl] = [b_l]
```

Let the number of poses be `m` and the number of landmarks be `n`; typically `n >> m`. Solving this large system directly is expensive.

Use the **Schur complement** to marginalize out the landmarks:

```
(H_pp - H_pl · H_ll^{-1} · H_lp) δp = b_p - H_pl · H_ll^{-1} · b_l
```

This is possible because **`H_ll` is block diagonal**. Each landmark is not directly coupled to other landmarks (no shared factor between two landmarks), so the inverse of `H_ll` can be computed by inverting each block independently. The cost is a cheap `O(n)`.

The system you actually solve now scales with the pose count `m` only, independent of `n`. This is why BA handles tens of thousands of landmarks while maintaining near-real-time performance.

Once `δp` is found, recover `δl` by back-substitution:
```
δl = H_ll^{-1} (b_l - H_lp · δp)
```

#### 14.9.3 Sparsity and Variable Ordering

In pose graph optimization the matrix `H` is **sparse**. Each pose has constraint relations only with temporally adjacent poses and with poses connected by loop closure. Even if there are 1000 poses in total, each pose is connected to at most a few or a few tens of others.

When solving a sparse linear system you use Cholesky factorization (`H = L L^T`), and the **fill-in** problem shows up here. Positions that were originally zero become non-zero during factorization. Heavy fill-in blows up memory and compute cost.

To minimize fill-in, you have to choose the variable ordering well:
- **COLAMD** (Column Approximate Minimum Degree): The most widely used heuristic. It eliminates the least-connected variables first.
- **AMD** (Approximate Minimum Degree): Similar to COLAMD but specialized for symmetric matrices.
- **Nested dissection**: Determines the ordering by recursively partitioning the graph. Effective on large-scale problems.

When configuring solvers in libraries like g2o, GTSAM, and Ceres, you have to choose a linear solver type (DENSE_SCHUR, SPARSE_NORMAL_CHOLESKY, and so on) and an ordering strategy. Running with the default settings, without this background, produces the kind of inefficiency that ends with "it was slow so I switched libraries." Changing the ordering alone can make a 10× or greater speed difference.

```python
# Example of setting the ordering in Ceres Solver (Python binding)
options = ceres.SolverOptions()
options.linear_solver_type = ceres.LinearSolverType.SPARSE_NORMAL_CHOLESKY
options.sparse_linear_algebra_library_type = ceres.SparseLinearAlgebraLibraryType.SUITE_SPARSE
# ordering typically defaults to COLAMD automatically, but manual configuration is possible
```

#### 14.9.4 Comparison of Optimization Libraries

| Library | Characteristics | Primary uses |
|---|---|---|
| **g2o** | Dedicated to pose graph / BA, lightweight, C++ only | ORB-SLAM2/3, LSD-SLAM |
| **GTSAM** | Factor graph based, supports Bayes tree (iSAM2), strong at incremental optimization | LIO-SAM, VINS-Fusion, research |
| **Ceres Solver** | General-purpose nonlinear least squares, supports auto-diff, developed by Google | Cartographer, various projects |

Selection criteria:
- SLAM-only and want to stay lightweight → g2o
- Need factor graph modeling, and incremental update (progressive optimization as keyframes are added) matters → GTSAM (iSAM2)
- Need general-purpose optimization beyond SLAM, and do not want to derive Jacobians by hand → Ceres (auto-diff)

> **Further reading**
> - Barfoot, "State Estimation for Robotics" Ch.4 (Nonlinear Estimation) — Systematic treatment of optimization on manifolds.
> - [CMU 16-833 Robot Localization and Mapping Lecture Notes](https://www.cs.cmu.edu/~kaess/pub/Dellaert17fnt.pdf) — Factor graphs and SLAM back-end theory.
> - [g2o Tutorial](https://github.com/RainerKuemmerle/g2o) / [GTSAM Tutorial](https://gtsam.org/tutorials/intro.html) — Hands-on tutorials per library.
> - [Giseop Kim's blog — Gauss-Newton Opt == IEKF update?](https://gisbi-kim.github.io/blog/2022/03/05/gn-iekf-same.html) — An exposition of the mathematical equivalence between GN optimization and iterated Kalman filtering. A reference for the filter vs optimization debate.

> **Practice**: [Bundle Adjustment Visualization](https://alexjunholee.github.io/robotics-practice/app.html#bundle_adjustment)
> Observe the bundle adjustment process — jointly optimizing camera poses and 3D points — interactively.

### 14.10 Advanced: IMU Preintegration

*If you want to become a researcher, start reading here.*

When introducing VINS-Mono in 14.3.3 we mentioned IMU preintegration briefly. Here we look at the mathematical background.

**Problem statement**: An IMU typically outputs acceleration and angular velocity at 200–1000 Hz. In contrast, SLAM optimization is done on a keyframe basis (a few Hz to tens of Hz). Hundreds of IMU measurements sit between two keyframes, and if you put all of them into optimization as state variables the problem size explodes.

**The idea of preintegration**: Compress the IMU measurements between two keyframes `i` and `j` into a single "relative motion measurement." This compressed measurement enters optimization as a factor.

**Preintegrated measurements**: Compute three relative quantities from keyframe `i` to `j`.

```
ΔR_ij = Π_{k=i}^{j-1} Exp((ω_k - b_g) · Δt)          # relative rotation
Δv_ij = Σ_{k=i}^{j-1} ΔR_ik · (a_k - b_a) · Δt        # relative velocity
Δp_ij = Σ_{k=i}^{j-1} (Δv_ik · Δt + 0.5 · ΔR_ik · (a_k - b_a) · Δt^2)  # relative position
```

Here `ω_k` and `a_k` are IMU measurements, `b_g` and `b_a` are the gyroscope/accelerometer biases, and `Δt` is the IMU sampling interval.

Key point: these preintegrated measurements are computed **in the coordinate frame of keyframe `i`**. Even if the absolute pose of keyframe `i` changes during optimization, you do not need to recompute the preintegrated measurement.

**Covariance propagation**: Compute how the IMU measurement noise propagates into the preintegrated measurement. Discrete-time propagation updates the covariance at every IMU measurement.

```
Σ_{k+1} = A_k · Σ_k · A_k^T + B_k · Q · B_k^T
```
- `A_k`: state transition matrix (the Jacobian at the current state)
- `B_k`: noise input matrix
- `Q`: IMU noise covariance (from the datasheet)

This covariance becomes the information matrix (`Σ^{-1}`) of the corresponding factor in optimization.

**Correction for bias changes**: During optimization the IMU bias estimate can change. If the bias changes, in principle you should redo the preintegration from scratch. But that is expensive. Instead, you correct it with a **first-order approximation**:

```
ΔR_ij ≈ ΔR_ij^0 · Exp(∂ΔR/∂b_g · δb_g)
Δv_ij ≈ Δv_ij^0 + ∂Δv/∂b_g · δb_g + ∂Δv/∂b_a · δb_a
Δp_ij ≈ Δp_ij^0 + ∂Δp/∂b_g · δb_g + ∂Δp/∂b_a · δb_a
```

`^0` denotes the value computed with the previous bias estimate, `δb` is the bias change, and the partial derivatives are accumulated alongside the preintegration. As long as the bias change is not large (which is usually the case), this approximation is accurate enough.

**Why preintegrate on the manifold**: The earlier approach interpolated IMU measurements to the nearest keyframe timestamp. But rotation lives on SO(3), so simple linear interpolation is not accurate. Integrating ahead of time on the Lie group 1) accumulates rotation in a mathematically correct way, and 2) produces a result that plugs directly into a factor graph as a relative motion measurement. This is the core contribution of Forster et al. (2015 RSS, 2017 TRO).

**Tightly-coupled vs loosely-coupled**: Using LIO-SAM as an example:
- **Loosely-coupled**: Uses the IMU only as an initial guess for the next pose. LiDAR odometry and the IMU estimate state independently and are combined later based on covariance. LeGO-LOAM follows this approach.
- **Tightly-coupled**: Optimizes an IMU preintegration factor jointly with the LiDAR odometry factor inside the same factor graph. The IMU acts not as a mere initial guess but as an independent observation of the relative pose between keyframes. LIO-SAM follows this approach.

The advantage of tightly-coupled shows up in aggressive motion (fast rotation, sharp acceleration/deceleration). The IMU factor catches fast changes that LiDAR scan matching alone cannot capture. A practical advantage is that, because it is in factor graph form, GPS factors, loop closure factors, and others can be plugged in like modules.

**Structure of LIO-SAM**: Built on GTSAM, it optimizes IMU preintegration factors + LiDAR odometry factors + GPS factors + loop closure factors in a single graph. LiDAR odometry extracts edge features and planar features separately and manages them in voxel maps of different resolutions. During scan matching, planar features solve for the relative transformation that minimizes point-to-plane distance, and edge features minimize point-to-line distance.

Modern VIO/LIO systems such as VINS-Mono, ORB-SLAM3 (visual-inertial mode), and LIO-SAM use this technique directly to implement their IMU factors.

> **Further reading**
> - [Forster et al., "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry" (TRO 2017, arXiv:1512.02363)](https://arxiv.org/abs/1512.02363) — The original preintegration paper. Equation-heavy, but required reading for the field.
> - [Forster et al., "IMU Preintegration on Manifold for Efficient VIO" (2015 RSS)](https://rpg.ifi.uzh.ch/docs/RSS15_Forster.pdf) — Earlier version of the paper above. The core idea is organized more concisely.
> - [Shan et al., "LIO-SAM" (IROS 2020)](https://github.com/TixiaoShan/LIO-SAM) — Reference implementation of tightly-coupled LIO. Read both the code and the paper.
> - [Sola et al., "A micro Lie theory for state estimation in robotics" (arXiv:1812.01537)](https://arxiv.org/abs/1812.01537) — A practical summary of Lie groups and algebras. Good to read before preintegration.
> - Source code of GTSAM's `PreintegratedImuMeasurements` class — see how the theory turns into code.
> - [IMU Preintegration MATLAB implementation](https://github.com/GentleDell/imu_preintegration_matlab) — MATLAB code tested on KITTI. Good for studying by cross-referencing equations and code.

### 14.11 Advanced: Observability Analysis

*If you want to become a researcher, start reading here.*

Run a SLAM/VIO system and you run into phenomena like "why is drift so bad in this situation?" and "why does the pose wobble when I stand still?" Many of these phenomena stem from limits of the system's **observability**.

**Unobservable states of visual-inertial systems**: VIO has 4 degrees of freedom that cannot be estimated (unobservable):

1. **Global position (3 DoF)** — The absolute position is unknown. Without an absolute reference like GPS, you can only treat the starting point as the origin.
2. **Global yaw (1 DoF)** — The rotation (heading) about the gravity-direction axis. Without a compass, you cannot tell "which way is north."

On the other hand, the following are observable:
- **Roll/pitch**: The IMU accelerometer senses the gravity direction, so roll/pitch relative to gravity can be estimated.
- **Scale** (when stereo/IMU is present): The stereo camera's baseline or the IMU's acceleration measurement lets you recover scale. However, **with a monocular camera alone, scale is unobservable**.

**Degenerate motion** — Under specific motion patterns, additional states become unobservable:

- **Pure rotation**: In monocular VO, translation cannot be estimated. The reason is that in epipolar geometry the epipole goes to infinity. This is the cause of the practical phenomenon "tracking breaks when you rotate the camera in place."
- **Constant velocity**: The IMU accelerometer distinguishes gravity from acceleration, and when there is no acceleration (constant velocity), the accelerometer bias cannot be distinguished from a small error in the gravity direction. IMU bias becomes unobservable.
- **Stationary**: A special case of constant velocity. Stand still and there is no parallax in the visual features and no IMU acceleration, so both bias and scale are unobservable. This is the answer to "why does VINS drift when I stand still?"

**Problem in EKF-based systems**: Apply a standard EKF to VIO and, due to linearization error, the covariance shrinks even along theoretically unobservable directions (the uncertainty is reduced artificially). This is a major cause of inconsistency.

**OC-EKF (Observability-Constrained EKF)**: To fix this problem, the EKF's Jacobian is modified to preserve the null space of the unobservable directions. It forces the estimator to "keep not knowing what it does not know."

Practical implications:
- When using a VIO system, you must initialize by **moving in diverse directions**. If you only walk in one direction, IMU bias estimation does not come out correctly.
- A VIO without loop closure will always drift over long-term operation. Error along the unobservable yaw direction keeps accumulating.
- Monocular VIO's scale is observable only when there is acceleration/deceleration. Moving at a constant velocity produces scale drift.

> **Further reading**
> - [Hesch et al., "Observability-constrained Vision-aided Inertial Navigation" (TRO 2014)](https://ieeexplore.ieee.org/document/6672119) — The original paper for OC-EKF/OC-VINS.
> - Barfoot, "State Estimation for Robotics" Ch.9 — Theoretical foundation of observability analysis.
> - [Huang & Dissanayake, "A critique of current developments in Simultaneous Localization and Mapping" (IJRR 2016)](https://journals.sagepub.com/doi/10.1177/0278364916643566) — A critical summary of observability/consistency issues in SLAM.

> **Practice**: [Odometry Uncertainty Visualization](https://alexjunholee.github.io/robotics-practice/app.html#odom_uncertainty)
> Observe interactively how the uncertainty of odometry accumulates over time and how the covariance ellipse grows.

#### 14.11.1 Filter-based vs Optimization-based: Which Is Better?

This is a long-standing debate in SLAM/VIO. Here is the bottom line up front: mathematically, Gauss-Newton optimization and the iterated EKF (IEKF) are equivalent. They solve the same problem from different angles.

- **Filter (EKF, MSCKF, and so on)**: Updates state incrementally as new measurements arrive. Past states are marginalized out and only the current state is kept. Memory-efficient, and natural to combine with proprioceptive sensors (IMU).
- **Optimization (BA, factor graph)**: Keeps all past states and optimizes them jointly. Because past data can be relinearized, accuracy is higher. But compute grows with the number of states (mitigated by sliding window or iSAM2).

So is "VINS-Mono (optimization) being better than MSCKF (filter)" a matter of the solver? No. The difference comes not from the solver but from the **system structure** (which states are kept, which measurements are used). The practical edge of optimization-based systems is that they can reduce past linearization error through relinearization.

Practical choice:
- IMU-centric + lightweight → filter (MSCKF, the IEKF in FAST-LIO2)
- Camera-centric + accuracy → optimization (VINS-Mono, ORB-SLAM3)
- Both needed → hybrid (LIO-SAM: optimization with IMU preintegration plugged in as a factor)

(Reference: [Giseop Kim's blog — Gauss-Newton Opt == IEKF update?](https://gisbi-kim.github.io/blog/2022/03/05/gn-iekf-same.html))

### 14.12 Advanced: Semantic SLAM

*If you want to become a researcher, start reading here.*

Classic SLAM produces a purely geometric map. Point clouds, meshes, occupancy grids and so on record only "the shape of space." It knows there is a wall, but not whether that wall is a wall, a door, or a bookshelf. Semantic SLAM adds semantic information to the map.

Approaches split by landmark representation. **Object-level SLAM** (CubeSLAM, QuadricSLAM) estimates objects as landmarks — 3D cuboids, dual quadrics, and the like — rather than points. It depends on an object detector, but data association is more robust than point-based, and object-level reasoning becomes possible. **Panoptic SLAM** fuses panoptic segmentation results into 3D to produce a map where every pixel carries a semantic label. The robot can directly query "there are 3 chairs in this room" on the map. **Open-vocabulary SLAM** (ConceptGraphs) stores features from a vision-language model like CLIP in the map, so places can be searched with natural language. It connects directly to the 3D Scene Graph (Hydra, etc.) discussed in Chapter 13.

**Handling dynamic objects**: Semantic labels are also used to improve SLAM robustness in dynamic environments. If you drop features from classes likely to be dynamic — "person," "car," and so on — from tracking/mapping, you can do clean SLAM with the static environment alone.
- DynaSLAM: ORB-SLAM2 + Mask R-CNN to mask dynamic objects
- DS-SLAM: semantic segmentation to filter dynamic regions

```python
# Pseudocode for dynamic object filtering
dynamic_labels = {'person', 'car', 'bicycle', 'dog'}
for feature in detected_features:
    pixel = feature.pixel_coords
    label = semantic_map[pixel.y, pixel.x]
    if label in dynamic_labels:
        feature.ignore = True  # exclude from SLAM
```

> **Further reading**
> - [Nicholson et al., "QuadricSLAM: Dual Quadrics from Object Detections as Landmarks in Object-Oriented SLAM" (RA-L 2019)](https://arxiv.org/abs/1804.04011) — Representative paper for object-level SLAM.
> - [ConceptGraphs (arXiv:2309.16650)](https://arxiv.org/abs/2309.16650) — Open-vocabulary 3D scene graph. Read alongside Chapter 13.
> - [Bescos et al., "DynaSLAM: Tracking, Mapping and Inpainting in Dynamic Scenes" (RA-L 2018)](https://arxiv.org/abs/1806.05620) — SLAM in dynamic environments.

### 14.13 Advanced: Multi-Robot SLAM

*If you want to become a researcher, start reading here.*

Having one robot explore a large environment takes a long time. Multiple robots exploring in parallel can cut the time, but merging each robot's partial map (submap) into one consistent global map is not trivial.

The **centralized approach** sends every robot's sensor data or local map to a central server that runs the full SLAM. It is simple to implement and stays close to the optimum, but sending all the raw data makes communication bandwidth the bottleneck, and the server becomes a single point of failure.

The **distributed approach** has each robot run local SLAM independently, and when a rendezvous or inter-robot loop closure occurs, it estimates relative poses to align the maps. Instead of raw data, robots exchange compressed descriptors (NetVLAD vectors, summary maps, and so on), which gives strong communication efficiency. Each robot optimizes only its own poses while constraints with neighboring robots drive the whole system toward convergence.

A distributed system has three problems it must solve. **Inter-robot loop closure** is when robot B later recognizes a place that robot A visited — the place recognition of Section 14.14 is the core. **Coordinate frame alignment** is needed because each robot starts in its own coordinate frame; the relative SE(3) transformation must be estimated from at least 3 inter-robot correspondences. **Outlier rejection** is required because inter-robot loop closures can produce many false positives, so robust methods such as PCM (Pairwise Consistency Maximization) or GNC (Graduated Non-Convexity) are needed. For distributed optimization, Distributed Gauss-Seidel, ADMM, and the like are used.

**Representative systems**:
| System | Characteristics |
|---|---|
| **Kimera-Multi** | Distributed, 3D mesh + semantic, Kimera based |
| **DOOR-SLAM** | Distributed, outlier-robust, DGS optimization |
| **Swarm-SLAM** | ROS2 based, supports diverse sensors, lightweight |

> **Further reading**
> - [Lajoie et al., "DOOR-SLAM: Distributed, Online, and Outlier Resilient SLAM for Robotic Teams" (RA-L 2020)](https://arxiv.org/abs/1909.12198) — Distributed SLAM + robust optimization.
> - [Tian et al., "Kimera-Multi: Robust, Distributed, Dense Metric-Semantic SLAM" (ICRA 2022)](https://arxiv.org/abs/2106.14386) — Multi-robot semantic SLAM.
> - [Cieslewski et al., "Data-Efficient Decentralized Visual SLAM" (ICRA 2018)](https://arxiv.org/abs/1710.05772) — Early work on communication-efficient distributed SLAM.

### 14.14 Advanced: Place Recognition

*If you want to become a researcher, start reading here.*

The core question of loop closure: "have I seen this scene before?" This is an image retrieval problem. The descriptor of the current frame is compared against the descriptors of all past keyframes and the most similar one is found. The accuracy of SLAM depends on loop closure, and loop closure depends on place recognition.

**Classical approach: Bag of Visual Words (BoVW)**

The DBoW2 library is representative and is used in ORB-SLAM2/3.
1. Extract local features (ORB, etc.) from a large image set
2. Build a visual vocabulary (word dictionary) with k-means clustering
3. Represent each image as a histogram (BoW vector) of "how often each visual word appears"
4. Compare images by similarity between BoW vectors (L1-score, etc.)

Strengths: fast (via an inverted index) and well-tested. Weaknesses: fragile against viewpoint/illumination changes, and the vocabulary needs training.

**Learning-based approach: global descriptors**

This approach compresses a whole image into a single compact vector and is more robust than BoVW. **NetVLAD** (2016) combines CNN features with VLAD aggregation and significantly outperformed prior methods on city-scale place recognition. **CosPlace** (2022) simplified the training pipeline with contrastive learning while raising performance. **MixVPR** (2023) uses feature mixing to stay robust across diverse conditions such as day/night and seasonal change. **AnyLoc** (2023) leverages DINOv2 features and delivered zero-shot place recognition that works indoors/outdoors and for aerial/ground without any fine-tuning.

**LiDAR-based place recognition**:

Recognize places from 3D structure alone, with no visual information. Fully immune to illumination changes, but can be confused in structurally similar environments (long corridors, for instance).

- **Scan Context** (IROS 2018): Projects a 3D point cloud into a bird-eye view and generates a 2D descriptor based on range/height. Supports rotation-invariant matching.
- **OverlapTransformer** (2022): Learns a global descriptor on LiDAR range images with a Transformer.

**Cross-modal place recognition**: Query with a camera image and retrieve from a LiDAR map, or vice versa. Important for multi-robot SLAM across robots with different sensors.

**Sequence matching**: To overcome the limits of single-image matching, match sequences of consecutive frames together.
- **SeqSLAM** (2012): Individual image similarities can be low, but if the sequence pattern matches, the system declares it the same place. Works even under dramatic appearance changes (day vs night).
- Recent methods: learn a sequence descriptor for more efficient sequence matching.

Practical tip: most SLAM systems use DBoW2 by default. If you have to operate in environments with large illumination/seasonal changes, consider swapping in a learning-based method (anything post-NetVLAD). AnyLoc has a low barrier to entry because it can be used without fine-tuning.

> **Further reading**
> - [Arandjelovic et al., "NetVLAD: CNN architecture for weakly supervised place recognition" (arXiv:1511.07247)](https://arxiv.org/abs/1511.07247) — Starting point of learning-based place recognition.
> - [Keetha et al., "AnyLoc: Towards Universal Visual Place Recognition" (arXiv:2308.00688)](https://arxiv.org/abs/2308.00688) — Foundation model-based zero-shot place recognition.
> - [Kim & Kim, "Scan Context: Egocentric Spatial Descriptor for Place Recognition within 3D Point Cloud Map" (IROS 2018)](https://ieeexplore.ieee.org/document/8593953) — Representative method for LiDAR place recognition.
> - [Giseop Kim's blog — Scan Context-based LiDAR Pose-graph SLAM implementation](https://gisbi-kim.github.io/blog/2021/05/17/sclidarslam.html) — A walk-through of integrating Scan Context into LiDAR SLAM.
> - [Dark Programmer — Bag of Words technique](https://darkpgmr.tistory.com/125) — Explains the principles of BoW by connecting it to image retrieval.

> **Technical Timeline: SLAM & Odometry**
> - **~2007**: The classical era. EKF-SLAM and FastSLAM (particle-filter based) dominated. MonoSLAM (2007) announced the arrival of real-time monocular SLAM. PTAM (2007) proposed the tracking/mapping split architecture.
> - **2010–2015**: LSD-SLAM, SVO, and other direct methods emerged. LOAM (2014) laid the groundwork for LiDAR SLAM. ORB-SLAM (2015) became the definitive feature-based Visual SLAM.
> - **2015–2020**: The visual-inertial era. VIO systems such as VINS-Mono (2018) and MSCKF became the standard on drones and mobile platforms. DSO (2018) proposed direct sparse. LiDAR-inertial integration took off in earnest: LIO-SAM (2020).
> - **2020–2023**: FAST-LIO/FAST-LIO2 (2021/2022) became the new standard for lightweight LiDAR-inertial systems. ORB-SLAM3 (2021) added visual-inertial and multi-map support. DROID-SLAM (2021) showed the potential of learning-based SLAM. Multi-sensor integrated systems like R3LIVE emerged.
> - **2024–**: 3DGS-based SLAM (SplaTAM, MonoGS, Gaussian-SLAM) is changing the direction of Neural SLAM. Research combining foundation models with SLAM (for instance, finding a location by describing a place in natural language) is also beginning.
> - **What to watch now**: Existing geometric SLAM (ORB-SLAM3, FAST-LIO2) is already mature technology, so master it; track 3DGS-SLAM and learning-based methods as trends. In practice, LIO-SAM/FAST-LIO2 (outdoor) and ORB-SLAM3 (indoor) are still used the most. Running them yourself on benchmark datasets (KITTI, EuRoC, TUM RGB-D) is the fastest way to learn.

### 14.15 Advanced: Long-term Mapping

*If you want to become a researcher, start reading here.*

When you operate a robot in a real environment, "build the map once and be done" is not the reality. You visit the same place multiple times, update the map, remove dynamic objects (people, vehicles), and integrate data from multiple sessions. This is long-term mapping, and it is unavoidable in practical robot systems.

#### 14.15.1 Incremental Smoothing: from iSAM to iSAM2

Filter-based SLAM (EKF, etc.) struggles with real-time processing as the Jacobian matrix grows with the number of states. iSAM (Kaess et al., TRO 2008) showed that the R matrix of the QR factorization can be updated incrementally with Givens rotations. When a new measurement is added, rather than recomputing the whole thing, only the affected part is updated. However, as non-zero elements accumulate, periodic re-ordering is needed.

iSAM2 (Kaess et al., IJRR 2012) overcame this limit by introducing the Bayes tree structure. Only the affected subtree is re-eliminated, delivering consistent performance even on large-scale problems. The Bayes tree is exactly the core engine of GTSAM.

#### 14.15.2 Dynamic Object Removal

Removing dynamic objects from the map is an essential task in long-term mapping.

**Removert** (Kim et al., 2020): Static/dynamic classification using multi-resolution range images. Projects a point cloud into a range image and compares against ranges observed from other viewpoints to decide whether each point is dynamic. It conservatively secures static points first, then restores falsely removed points — a two-stage design. The key point is that multiple confidence levels let you tune the trade-off.

Compared with prior approaches: voxel ray-casting is accurate but expensive; visibility-based methods assume that static points behind dynamics are preserved; segmentation-based methods are weak on unknown labels and ignore the scan-to-map relationship. Removert compensates for the drawbacks of these three using multi-resolution range-image comparison.

**SuMa++** (Chen et al., IROS 2019): Adds semantic labels to surfel-based mapping. It augments LiDAR points with normals and semantic information, and removes a surfel only when it is judged dynamic by both semantics and motion. It does not just erase everything because, in motion-degenerate environments, there can be points that are dynamic yet geometrically useful.

#### 14.15.3 Multi-Session SLAM

When you map the same environment across multiple days, the trajectories of the sessions have to be merged into one. The problem is gauge freedom — each session's coordinate frame is different, so naively merging does not align them.

**LT-mapper** (Kim et al., 2021): Aligns multiple sessions via Scan Context-based anchor nodes and updates the map with positive/negative change detection. Distinguishes high-dynamic, low-dynamic, weak non-dynamic, and strong positive-dynamic by the degree of change, managing a delta map.

**Continuous-Time Estimation** (Furgale et al., ICRA 2012): Representing the trajectory with B-spline basis functions instead of discrete time lets you integrate sensors at different Hz with fewer variables. It is also applicable to self-calibration between fast sensors (IMU) and slow sensors (LiDAR, camera).

> **Further reading**
> - [Kaess et al., "iSAM2: Incremental Smoothing and Mapping Using the Bayes Tree" (IJRR 2012)](https://www.cs.cmu.edu/~kaess/pub/Kaess12ijrr.pdf) — Original paper on Bayes tree-based incremental SLAM.
> - [Kim et al., "Remove, then Revert: Static Point Cloud Map Construction using Multiresolution Range Images" (IROS 2020)](https://github.com/irapkaist/removert) — Practical method for dynamic point removal. Code released.
> - [Kim et al., "LT-mapper: A Modular Framework for LiDAR-based Lifelong Mapping" (ICRA 2022)](https://github.com/gisbi-kim/lt-mapper) — Multi-session SLAM framework.
> - [Chen et al., "SuMa++: Efficient LiDAR-based Semantic SLAM" (IROS 2019)](https://github.com/PRBonn/semantic_suma) — LiDAR SLAM that exploits semantic information.
