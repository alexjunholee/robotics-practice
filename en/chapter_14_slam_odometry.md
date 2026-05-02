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

The derivation of the MCL algorithm is in §3.11 (Ch.3). The mathematical foundation of EKF is in §3.10 (Ch.3). For localization extended with IMU coupling, see §14.10. What follows covers the classification of localization scenarios and algorithm variants.

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

#### 14.7.1 Classification of Localization Problems

The difficulty of localization does not collapse into a single number. Four axes interact to determine the algorithm choice.

| Axis | Options | Note |
|---|---|---|
| Prior knowledge | position tracking → global localization → kidnapped robot | increasing difficulty |
| Environment | static (robot only moves) → dynamic (people, doors, lighting) | harder as dynamism grows |
| Agency | passive (observe only) → active (choose exploration actions) | active converges faster |
| Robot count | single → multi (belief sharing via mutual observation) | multi yields richer information |

Two scenarios are canonical, one is an extension.

**Position tracking**: The initial pose is known and belief stays as a narrow unimodal Gaussian. EKF Localization fits well.

**Global localization**: The initial pose is unknown. The belief must start from a uniform distribution and converge as measurements accumulate. Multi-modal belief representation is needed, so Grid Localization or MCL is appropriate.

**Kidnapped robot**: The robot is forcibly moved to a different location during operation. It is harder than global localization because the robot does not detect the displacement itself. Every algorithm will eventually face this situation, so recovery capability is itself a measure of robot autonomy.

ROS Nav2's `recovery_alpha_slow/fast` parameters are designed with the kidnapped scenario in mind. Warehouse AGV and cleaning robot boot-up corresponds to global localization; normal operation corresponds to tracking.

#### 14.7.2 Markov Localization

Markov localization is less an algorithm than a name for **the direct application of the Bayes filter to the localization problem**. EKF Localization, Grid Localization, and MCL all branch off from this shared Bayes filter framework; what differs is how each one represents the belief.

The only difference from the Bayes filter (Ch.3 §3.9) is that the motion model and observation model both take **the map m** as an additional input.

```
Markov_localization(bel(x_{t-1}), u_t, z_t, m):
  for all x_t do
    bel̄(x_t) = ∫ p(x_t | u_t, x_{t-1}, m) bel(x_{t-1}) dx_{t-1}   // motion update
    bel(x_t) = η p(z_t | x_t, m) bel̄(x_t)                          // measurement update
  endfor
  return bel(x_t)
```

The initial belief bel(x_0) is initialized differently for each scenario:
- Position tracking: $\text{bel}(x_0) = \mathcal{N}(x_0;\, \bar{x}_0, \Sigma)$ — narrow Gaussian
- Global localization: $\text{bel}(x_0) = 1/|X|$ — uniform over all valid poses
- Partial knowledge: uniform over the known vicinity, zero elsewhere

The algorithms in §14.7.3–§14.7.7 are variations on "how to implement the bel representation in the box above."

#### 14.7.3 EKF Localization

EKF Localization is a special case of Markov localization that represents belief as a Gaussian $(\mu_t, \Sigma_t)$. **The unimodal assumption makes it suitable only for position tracking.** Global localization and the kidnapped problem require multi-modal belief, so EKF cannot address them.

It applies the EKF of §3.10.2 (Ch.3) to localization. What follows is the assumption structure — a feature-based map with known landmark correspondences — and the concrete algorithm.

**Assumptions**: The map m is feature-based (a set of point landmarks). Each measurement $z_t^i = (r, \phi, s)^T$ (range, bearing, signature). The correspondence $c_t^i$ is known (identifiable landmarks such as ARTags, QR codes, or the Eiffel Tower).

```
EKF_localization_known_correspondences(μ_{t-1}, Σ_{t-1}, u_t, z_t, c_t, m):
  // Motion update (linearized velocity model)
  μ̄_t = μ_{t-1} + [velocity model displacement]
  G_t = ∂g/∂x |_{μ_{t-1}, u_t}         // 3×3 Jacobian
  Σ̄_t = G_t Σ_{t-1} G_t^T + R_t

  // Measurement update (loop over landmarks)
  for each observed z_t^i = (r, φ, s)^T do
    j = c_t^i
    δ = (m_{j,x} − μ̄_{t,x},  m_{j,y} − μ̄_{t,y})^T,   q = δ^T δ
    ẑ_t^i = (√q,  atan2(δ_y, δ_x) − μ̄_{t,θ},  m_{j,s})^T
    H_t^i = Jacobian (3×3, last row is 0 — signature independent of pose)
    K_t^i = Σ̄_t H_t^{i,T} (H_t^i Σ̄_t H_t^{i,T} + Q_t)^{-1}
  endfor
  μ_t = μ̄_t + Σ_i K_t^i (z_t^i − ẑ_t^i)
  Σ_t = (I − Σ_i K_t^i H_t^i) Σ̄_t
  return μ_t, Σ_t
```

Summing multiple gains $K^i$ is valid because of the **conditional independence assumption** $p(z_t | x_t, m) = \prod_i p(z_t^i | x_t, m)$. In information space, measurements add together.

Practical limit: when heading standard deviation exceeds ±20°, linearization error becomes serious. Violating this heuristic causes the EKF covariance to be underestimated (overconfident), and the filter diverges. As of 2026, EKF Localization survives in systems with ARTag/AprilTag ceiling markers and in GNSS+IMU dead-reckoning fusion.

**Unknown correspondences**: In practice $c_t^i$ is usually unknown. Maximum likelihood (ML) data association selects the map landmark with the smallest Mahalanobis distance.

$$j(i) = \arg\min_k (z_t^i - \hat{z}_t^k)^T \Psi_k^{-1} (z_t^i - \hat{z}_t^k), \quad \Psi_k = H_t^k \bar\Sigma_t H_t^{k,T} + Q_t$$

Minimizing Mahalanobis distance is equivalent to maximizing the log Gaussian likelihood (up to normalization constants). Two practical additions: (1) outlier rejection by $\chi^2$ 95% threshold on the Mahalanobis distance, (2) mutual exclusion — two measurements within one frame cannot correspond to the same landmark. ORB-SLAM's descriptor matching plus RANSAC is the modern implementation of this skeleton.

#### 14.7.4 Multi-Hypothesis Tracking (MHT)

EKF uses a unimodal Gaussian and cannot represent data association ambiguity. MHT represents belief as a **Gaussian mixture** and maintains multiple hypotheses in parallel.

Each hypothesis $h$ runs an independent EKF. When a measurement arrives, each hypothesis is extended, and hypotheses whose weight (posterior probability) falls below the threshold $\psi_{\min}$ are pruned. A pruning policy is essential to prevent hypothesis count from exploding.

In autonomous driving multi-object tracking (MOT), Hungarian algorithm plus Mahalanobis gating is the direct descendant of MHT.

#### 14.7.5 Grid Localization

Where MHT represented belief as a Gaussian mixture, Grid Localization takes a more direct approach: it divides the entire pose space into cells and accumulates probability per cell.

A **histogram filter** that discretizes pose space into cells. It can represent global and multi-modal belief that EKF cannot, but the computational cost proportional to the number of cells $K$ is the trade-off.

```
Grid_localization({p_{k,t-1}}, u_t, z_t, m):
  for all k do
    p̄_{k,t} = Σ_i p_{i,t-1} · motion_model(mean(x_k), u_t, mean(x_i))
    p_{k,t}  = η · measurement_model(z_t, mean(x_k), m) · p̄_{k,t}
  endfor
  return {p_{k,t}}
```

$\text{bel}(x_t) = \{p_{k,t}\}$: one probability per cell $x_k$, summing to 1.

**Resolution trade-off**: smaller cells reduce estimation error — 5 cm cells yield 4 cm error in LiDAR, 65 cm cells yield 25 cm error (from Probabilistic Robotics experiments). But smaller cells mean sharply higher CPU time for global localization. Practical tricks include caching raycast results, scan subsampling, and selective updates (only cells above a threshold).

It serves as an educational bridge: represents global belief on a discrete grid and illustrates why particle filters perform better. ROS `amcl` is Grid Localization with the grid cells replaced by particles.

#### 14.7.6 MCL Algorithm (Expanded)

The derivation and principles of MCL are in §3.11 (Ch.3). Here the full skeleton of the algorithm as a localization method is stated explicitly.

```
MCL(X_{t-1}, u_t, z_t, m):
  X̄_t = X_t = ∅
  for k = 1 to M do
    x_t^[k] = sample_motion_model(u_t, x_{t-1}^[k])    // motion proposal
    w_t^[k] = measurement_model(z_t, x_t^[k], m)        // likelihood weight
    X̄_t += ⟨x_t^[k], w_t^[k]⟩
  endfor
  for k = 1 to M do
    i ~ Categorical(w_t^[1], ..., w_t^[M])              // importance-proportional resample
    X_t += x_t^[i]
  endfor
  return X_t
```

Three phases: **predict (sample) → weight → resample**. Initialization depends on the scenario: for global localization, sample $M$ particles from a uniform distribution over free space; for position tracking, sample from a narrow Gaussian.

**Computational adaptability**: rather than fixing $M$, sampling "as many as possible before the next measurement arrives" means faster CPUs yield larger $M$ and automatically better accuracy.

The proposal is the motion model, so with a perfect sensor (extremely narrow measurement likelihood) nearly all particle weights approach zero. This is what Mixture MCL (§14.7.8) fixes.

ROS2 Nav2's `nav2_amcl` implements this structure directly.

#### 14.7.7 Augmented MCL — Kidnapping Recovery

Standard MCL is fragile against kidnapping. Once particles converge on a single pose and the robot is forcibly moved, no particle sits near the new location and there is no recovery path. Augmented MCL **injects random particles when the short-term average of measurement likelihood suddenly drops relative to the long-term average.** "The sensor suddenly stops matching the map" equals "the robot is lost" — this intuition is quantified as the ratio of two exponential moving averages.

```
Augmented_MCL(X_{t-1}, u_t, z_t, m):
  static w_slow, w_fast
  X̄_t = X_t = ∅,  w_avg = 0
  for k = 1 to M do
    x_t^[k] = sample_motion_model(u_t, x_{t-1}^[k])
    w_t^[k] = measurement_model(z_t, x_t^[k], m)
    X̄_t += ⟨x_t^[k], w_t^[k]⟩
    w_avg += w_t^[k] / M
  endfor
  w_slow += α_slow (w_avg − w_slow)    // long-term average (slow to change)
  w_fast += α_fast (w_avg − w_fast)    // short-term average (fast to change)
  for k = 1 to M do
    with probability max(0, 1 − w_fast/w_slow) do
      X_t += random pose from bel(x_0)  // random particle injection
    else
      i ~ Categorical(w_t^[1], ..., w_t^[M])
      X_t += x_t^[i]
  endfor
  return X_t
```

Requirement: $0 \le \alpha_{\text{slow}} \ll \alpha_{\text{fast}}$ (e.g., $\alpha_{\text{slow}} = 0.001$, $\alpha_{\text{fast}} = 0.1$).

$$p_{\text{inject}} = \max\!\left(0,\, 1 - \frac{w_{\text{fast}}}{w_{\text{slow}}}\right)$$

Normally $w_{\text{fast}} \approx w_{\text{slow}}$ → ratio $\approx 1$ → injection probability $\approx 0$ → identical to standard MCL. Immediately after kidnapping, measurements no longer match anywhere → $w_{\text{fast}}$ drops sharply → injection probability rises. When the long-term average catches up, the ratio returns to 1 → injection stops.

Simple noise spikes do not trigger $w_{\text{slow}}$, so false positives are suppressed.

The `recovery_alpha_slow` and `recovery_alpha_fast` parameters in ROS `amcl` correspond exactly to these equations. Recent warehouse robots use a hybrid variant that injects deep relocalization results (NetVLAD + PnP) in place of random poses.

#### 14.7.8 Mixture MCL

Where Augmented MCL injects random poses, Mixture MCL **changes the proposal distribution itself**. A fraction of particles is sampled directly from the **measurement model** rather than the motion model.

$$x_t^{[k]} \sim \begin{cases} p(z_t | x_t, m) & \text{with probability } \rho \\ \text{sample\_motion\_model}(u_t, x_{t-1}^{[k]}) & \text{with probability } 1 - \rho \end{cases}$$

Particles sampled directly from measurements concentrate in regions of strong sensor information, fixing the proposal inefficiency of basic MCL in low-noise sensor environments. The advantage over Augmented MCL is that it handles both kidnapping recovery and low-noise sensor failure. The implementation burden is that sampling directly from $p(z_t | x_t, m)$ requires an inverse sensor model.

#### 14.7.9 Dynamic Environment Filtering

When dynamic objects (people, vehicles) are present, some beams observe obstacles not in the map. The posterior probability of the short-hit component $p_{\text{short}}(z | x, m)$ from the beam sensor model is used to exclude suspicious beams from the localization weight computation.

For each beam $z_t^k$, the four-component mixture model (§2.7, Ch.2) is evaluated and beams with high posterior probability on the short component are excluded from the weight calculation. Without this filtering, MCL becomes unstable when many people occupy a corridor.

#### 14.7.10 Filter Comparison Summary

| Algorithm | Belief representation | Position tracking | Global loc | Kidnapped | Compute cost |
|---|---|---|---|---|---|
| EKF Loc | Gaussian (μ, Σ) | good | impossible | impossible | O(N) |
| MHT | Gaussian mixture | good | limited | limited | O(H·N) |
| Grid Loc | histogram | good | possible | possible | O(K) |
| MCL | particle set | good | possible | possible with Augmented MCL | O(M) |

N: landmark count, H: hypothesis count, K: grid cell count, M: particle count. EKF cannot address global/kidnapped problems in principle because of the Gaussian unimodal assumption. Grid and MCL are practical because the compute-accuracy trade-off can be tuned by adjusting the resource budget.

#### 14.7.11 Implementation Notes: Landmark Efficiency and Negative Information

Several issues arise frequently when implementing EKF Localization in practice.

**Efficient landmark search**: When the map contains N landmarks, a full search per observation costs O(N). Using a KD-tree or grid index to search only landmarks within range brings this down to O(log N).

**Mutual exclusion**: Two measurements within one frame cannot correspond to the same landmark. ML data association is a component-wise optimization and does not enforce this constraint automatically. When conflicting pairs occur, a repair step is needed: choose the measurement with the smaller Mahalanobis distance and discard the other.

**Outlier rejection** removes measurements whose Mahalanobis distance exceeds the $\chi^2_{95\%}$ threshold. This single step substantially reduces EKF brittleness.

**Negative information**: "No landmark was observed in this angular range" can also be informative for localization, but the correct probabilistic treatment is complex and the implementation burden is high. Most practical systems ignore negative information.

---

### 14.7B Occupancy Grid Mapping

§14.7 estimated location given a map. This section reverses the direction: Occupancy Grid Mapping builds the map given known poses. In a real SLAM pipeline the two phases alternate — pose graph optimization fixes the pose trajectory, and then the algorithm in this section completes the final map. The foundation in binary Bayes filters is in §3.11.2 (Ch.3).

Where SLAM "estimates pose and map simultaneously," Occupancy Grid Mapping **estimates the occupancy probability of each cell given known poses.** It is the core post-processing step that produces the final map from the pose trajectory delivered by pose graph optimization.

#### 14.7B.1 Introduction: Why Mapping Is Hard

Mapping is said to be harder than localization. A pose is a continuous variable $x_t \in \mathbb{R}^3$, but a map m is a high-dimensional discrete variable composed of tens of thousands to millions of cells. The number of possible maps is $2^{|m|}$, making direct search impossible.

Two assumptions prevent combinatorial explosion: (1) **poses are known** ($x_{1:t}$ given), (2) **cells are conditionally independent**. The second assumption lets the map posterior factor into a product of per-cell marginals, splitting the whole problem into independent binary Bayes filters — one per cell.

$$p(m \mid z_{1:t}, x_{1:t}) = \prod_i p(m_i \mid z_{1:t}, x_{1:t})$$

Additional difficulties include sensor noise, perceptual aliasing (different measurements from the same location), environmental dynamics, and error accumulation over closed loops.

#### 14.7B.2 Standard Algorithm: Log-Odds Accumulation

The occupancy posterior of each cell is accumulated in **log-odds** form.

$$l_{t,i} = \log \frac{p(m_i \mid z_{1:t}, x_{1:t})}{1 - p(m_i \mid z_{1:t}, x_{1:t})}$$

Prior log-odds: $l_0 = \log[p(m_i) / (1 - p(m_i))]$. From the binary Bayes filter derivation (§3.11.2, Ch.3), the update rule is:

$$l_{t,i} = l_{t-1,i} + \text{inverse\_sensor\_model}(m_i, x_t, z_t) - l_0$$

Intuition: when a new measurement gives hit evidence for cell $m_i$, the log-odds rises; free evidence lowers it. The $-l_0$ term prevents the prior from being counted twice.

```
occupancy_grid_mapping({l_{t-1,i}}, x_t, z_t):
  for all cells m_i do
    if m_i is in perceptual field of z_t then
      l_{t,i} = l_{t-1,i} + inverse_sensor_model(m_i, x_t, z_t) − l_0
    else
      l_{t,i} = l_{t-1,i}    // outside sensing range — no change
  endfor
  return {l_{t,i}}
```

Recovery to probability: $p(m_i | z_{1:t}, x_{1:t}) = 1 - 1/(1 + \exp\{l_{t,i}\})$.

**inverse_sensor_model** (simplified example for a range finder):
```
inverse_range_sensor_model(m_i, x_t, z_t):
  compute range r and bearing φ to cell center
  nearest beam index k = argmin_j |φ − θ_{j,sens}|
  if outside beam or beyond z_t^k + α/2:
    return l_0                   // no information
  if |r − z_t^k| < α/2:
    return l_occ                 // hit (> l_0)
  if r ≤ z_t^k:
    return l_free                // free (< l_0)
```

$\alpha$ is the obstacle thickness parameter, $\beta$ is the beam opening angle. ROS Nav2's `costmap_2d`, SLAM Toolbox, and Cartographer's submap representation all use this log-odds accumulation directly.

#### 14.7B.3 Multi-Sensor Fusion

Camera, LiDAR, sonar, and infrared each have a different inverse_sensor_model. The simplest fusion strategy is **conservative max per cell**: if any sensor reports a hit, that cell is classified as occupied. This conservative policy is safe for collision avoidance but tends to underestimate free space.

An alternative is to accumulate each sensor's log-odds updates independently and then sum per cell. When sensors carry different amounts of information, weighted summation is needed.

#### 14.7B.4 Learning the inverse_sensor_model

A hand-designed inverse_sensor_model is a simple geometric model. **If the forward model $p(z | x, m)$ is already available, the inverse can be derived by learning.**

The procedure: generate triples $\{(x^{(k)}, z^{(k)}, m_i^{(k)})\}$ from simulation, then train a function approximator with cross-entropy loss.

$$\mathcal{L} = -\sum_k \left[m_i^{(k)} \log \hat{p}_i + (1 - m_i^{(k)}) \log(1 - \hat{p}_i)\right]$$

A neural network with input $(x, z)$ and output $\hat{p}_i = p(m_i | x, z)$ takes over the role of inverse_sensor_model. This is useful when complex sensor geometry (sonar reflection patterns, LiDAR behavior on glass) is too difficult to model explicitly.

#### 14.7B.5 MAP Occupancy Mapping (Advanced)

The cell independence assumption of the standard algorithm creates one contradiction: adjacent cells inside the same beam cone share correlated evidence in reality, but the independence assumption ignores this correlation. The problem is most noticeable with wide-beam sensors such as sonar.

MAP Occupancy Mapping directly maximizes the mode of the map posterior.

$$m^* = \arg\max_m \left[\sum_t \log p(z_t \mid x_t, m) + \log p(m)\right]$$

Rather than an inverse model, it uses the **forward model** $p(z_t | x_t, m)$ directly. Starting from an all-free map, hill-climbing flips cells one at a time in the direction that increases log-likelihood.

```
MAP_occupancy_grid_mapping(x_{1:t}, z_{1:t}):
  m ← initialize all cells free
  repeat until convergence:
    for all cells m_i do
      m_i ← argmax_{k ∈ {0,1}} [k·l_0 + Σ_t log measurement_model(z_t, x_t, m | m_i=k)]
  return m
```

Practical limits: it is batch and does not fit incremental SLAM; hill-climbing gets trapped in local maxima; posterior uncertainty disappears. But the insight that **"the cell independence assumption must be broken"** carries forward.

#### 14.7B.6 Direct Descendants: OctoMap, Voxblox, NeRF, 3DGS

The 2D and 3D descendants of occupancy grids form the foundation of modern Spatial AI. **OctoMap** compresses 3D occupancy into an octree to reduce memory. **Voxblox** and **nvblox** represent the signed distance to the surface (TSDF — Truncated Signed Distance Function) per voxel rather than per cell, raising surface precision. **NeRF**'s density field and **3D Gaussian Splatting**'s opacity are continuous, differentiable generalizations of occupancy — integrating occupancy probability along each ray via ray marching is equivalent to replacing the per-cell binary Bayes filter with a volumetric rendering loss. The intuition from MAP occupancy of using a forward model carries directly into NeRF's rendering loss. Standard occupancy has not disappeared — it still underpins collision avoidance in SLAM Toolbox and the Nav2 costmap.

> **Practice**: [Occupancy Grid](https://alexjunholee.github.io/robotics-practice/app.html#occupancy_grid)
> Visualize the log-odds accumulation process cell by cell and observe how the hit/free regions of the inverse_sensor_model build into a map.

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

For the history of information-form SLAM before the factor graph era (EKF-SLAM, EIF, SEIF, EM), see §14.16 Advanced: History of Information-Form SLAM.

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

For localization extended with IMU coupling, see §14.7 and §14.7B for the mapping foundation. When introducing VINS-Mono in 14.3.3 we mentioned IMU preintegration briefly. Here we look at the mathematical background.

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

The combination of place recognition and backward-in-time correction traces back to the cycle posterior in §14.16.5.

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

---

### 14.16 Advanced: History of Information-Form SLAM

*Cross-reference with §14.9 factor graph optimization (bidirectional).*

When Probabilistic Robotics was published in 2005, SLAM algorithms were in a competition over how to represent information. EKF-SLAM was the standard; EIF/SEIF were pioneering attempts that exploited the additivity of information; EM mapping was a refined approach for handling unknown data association statistically. All of these lineages were absorbed, in the 2010s, into a single framework: **factor graph + GTSAM/iSAM2**. Without this history, why the factor graph won is hard to understand.

#### 14.16.1 EKF-SLAM (PR §10)

The lineage begins with **Smith, Self, and Cheeseman (1986/1990)**, "Estimating Uncertain Spatial Relationships in Robotics." Their proposal of a "stochastic map" — bundling robot pose and landmarks into a single random variable — is the prototype of EKF-SLAM. Leonard and Durrant-Whyte in the 1990s, then Dissanayake et al. (2001, IEEE T-RA), completed the formalization.

**Algorithm skeleton**: The pose $x_t = (x, y, \theta)$ and N landmarks $(m_{j,x}, m_{j,y}, s_j)$ are packed into a $(3N+3)$-dimensional state vector $y_t$ and an EKF is run over it.

```
EKF_SLAM_known_correspondences(μ_{t-1}, Σ_{t-1}, u_t, z_t, c_t):
  // Motion: lift 3D motion into (3N+3)D via F_x
  // F_x = [I_3 | 0_{3×3N}] — (3×(3N+3)) projection, F_x^T is (3N+3)×3
  // G_t = I_{3N+3} + F_x^T G_t^{pose} F_x — (3N+3)×(3N+3), G_t^{pose} is 3×3 pose Jacobian
  μ̄_t = μ_{t-1} + F_x^T · g(u_t, μ_{t-1}[pose part])
  Σ̄_t = G_t Σ_{t-1} G_t^T + F_x^T R_t F_x

  // Measurement loop
  for each observation z_t^i with j = c_t^i do
    if j is new landmark:
      μ̄_{j} ← initialize via inverse range-bearing transform
    ẑ_t^i = h(μ̄_t, j),   H_t^i = Jacobian  // H_t^i: 3×(3N+3)
    K_t^i = Σ̄_t H_t^{i,T} (H_t^i Σ̄_t H_t^{i,T} + Q_t)^{-1}
  endfor

  // Update
  μ_t = μ̄_t + Σ_i K_t^i (z_t^i − ẑ_t^i)
  Σ_t = (I − Σ_i K_t^i H_t^i) Σ̄_t
  return μ_t, Σ_t
```

**The Kalman gain $K_t^i$ is a $(3N+3) \times 3$ matrix** — a single landmark observation updates the entire state. This is both the magic and the curse of EKF-SLAM: one observation improves other landmark estimates through the covariance off-diagonals, while the update cost grows as $O(N^2)$.

With unknown correspondences, a provisional $(N_t+1)$-th landmark is temporarily appended to the map; Mahalanobis distances to all candidates are computed and the ML correspondence is selected. If the distance exceeds threshold $\alpha$, the observation is registered as a new landmark. This greedy ML decision, once wrong, cannot be undone — the fundamental weakness of ML data association.

EKF-SLAM's limits appeared on three axes. The covariance matrix $\Sigma \in \mathbb{R}^{(3N+3) \times (3N+3)}$ requires memory proportional to $N^2$ — 100 landmarks means a 303×303 matrix, 1000 landmarks means 3003×3003. As landmarks accumulate, past linearization error compounds and estimation becomes inconsistent (Bailey et al. 2006). Because past poses are marginalized out, full posterior optimization over them is impossible.

As of 2026, EKF-SLAM itself has disappeared from practical systems. The EKF skeleton survives in embedded systems with few landmarks (<50) and in sliding-window EKF variants such as MSCKF (see §14.9 and §14.10). JCBB (Neira & Tardós 2001) appeared as an alternative to ML data association, and ORB-SLAM's map point culling inherits the provisional landmark list idea. Direct descendant: **GTSAM/iSAM2** (see §14.9).

#### 14.16.2 EIF SLAM / GraphSLAM (PR §11)

In EKF-SLAM, $\Sigma$ requires dense full-matrix updates for every measurement. The information form $\Omega = \Sigma^{-1}$ is additive and can maintain a sparse structure. That is the motivation for moving to EIF.

##### The intuition of information form: spring-mass analogy

The central idea of EIF SLAM is that **information is additive**. Instead of covariance $\Sigma$, information matrix $\Omega = \Sigma^{-1}$ and information vector $\xi = \Omega \mu$ are used.

Viewed as a spring-mass system: each variable (pose, landmark) is a node; off-diagonal elements of $\Omega$ are springs connecting two nodes.
- Control $u_t$: a spring between $x_{t-1}$ and $x_t$. Stiffness = $R_t^{-1}$ (stronger coupling when motion noise is smaller).
- Measurement $z_t^i$: a spring between pose $x_t$ and landmark $m_j$. Stiffness = $Q_t^{-1}$.
- No direct spring between two different landmarks — they have never been observed relative to each other.

**Information-form update rule**:
$$\Omega \leftarrow \Omega + H_t^{iT} Q_t^{-1} H_t^i, \qquad \xi \leftarrow \xi + H_t^{iT} Q_t^{-1}[z_t^i - h(\mu_t) + H_t^i \mu_t]$$

This eliminates the global Kalman gain and Schur complement operations of EKF. New information is added by **local addition only**. The core contribution of Thrun, Liu, Koller, Ng, Ghahramani, and Durrant-Whyte (2004, IJRR). This intuition is where factor graphs start — each factor in a factor graph is exactly one such spring.

##### Four-step pipeline

EIF SLAM (= GraphSLAM) solves the full posterior $p(x_{0:t}, m | z_{1:t}, u_{1:t})$ offline in batch.

```
EIF_SLAM_known_correspondence(u_{1:t}, z_{1:t}, c_{1:t}):
  1. Initialize:  μ_{0:t} ← initial estimate from motion model alone (ignore observations)
  2. Construct:   starting from Ω = 0, ξ = 0,
                  accumulate prior, controls, and measurements by local addition
  3. Reduce:      for each landmark j, eliminate via Schur complement
                  Ω̄ ← Ω̄ − Ω_{τ(j),j} Ω_{j,j}^{-1} Ω_{j,τ(j)}
                  ξ̄ ← ξ̄ − Ω_{τ(j),j} Ω_{j,j}^{-1} ξ_j
                  → reduced Ω̄, ξ̄ with poses only
  4. Solve:       Σ_{0:t} = Ω̄^{-1},  μ_{0:t} = Σ_{0:t} ξ̄
                  each landmark: μ_j = Ω_{j,j}^{-1}(ξ_j − Ω_{j,τ(j)} μ_{τ(j)})
  iterate 2-3 times total (to improve linearization)
  return μ_{0:t}, {μ_j}
```

$\tau(j)$ = all pose time steps at which landmark $j$ was observed. The Reduce step is essentially the process of *creating new springs between poses adjacent to each landmark and then detaching the landmark node* — mathematically identical to the **block diagonal Schur complement** trick in Bundle Adjustment (see §14.9.2).

**Marginalization Lemma**: In information form, marginals are cleanly expressed via Schur complement.
$$\bar\Omega_{xx} = \Omega_{xx} - \Omega_{xy} \Omega_{yy}^{-1} \Omega_{yx}$$

Thrun and Montemerlo (2006, IJRR) reformulated EIF SLAM under the name "GraphSLAM." Lu and Milios (1997) were the earlier pioneers who first proposed information accumulation in pose graph form.

Unknown correspondence handling computes the probability that each feature pair $(m_j, m_k)$ corresponds to the same physical object; pairs above threshold are merged and EIF is re-run. The difference from EKF's greedy ML is that the additivity of information form allows **past decisions to be reversed** — a wrong merge can be canceled by subtraction. This philosophy was later inherited by switchable constraints (Sünderhauf & Protzel 2012).

The four-step pipeline (Initialize → Construct → Reduce → Solve) **remains the standard for modern SLAM.** Batch simply evolved into incremental (iSAM, iSAM2), and variable elimination evolved into the Bayes tree.

#### 14.16.3 SEIF — Sparse Extended Information Filter (PR §12)

EIF SLAM is an offline batch algorithm. For a robot building a map while moving in real time, an online filter is needed. **SEIF** achieves *constant-time updates independent of map size* by keeping the information matrix sparse at all times. Thrun, Liu, Koller, Ng, Ghahramani, and Durrant-Whyte (2004, IJRR) demonstrated on the Victoria Park 3.5 km dataset that it matched EKF-SLAM accuracy at half the time and one-quarter the memory.

##### Four-step update

```
SEIF_SLAM_known_correspondences(ξ_{t-1}, Ω_{t-1}, μ_{t-1}, u_t, z_t, c_t):
  1. Motion update:       ξ̄_t, Ω̄_t, μ̄_t ← update in information form using u_t
                          (only active features + robot pose change; sparsity preserved)
  2. Measurement update:  Ω_t ← Ω̄_t + Σ_i H_t^{iT} Q_t^{-1} H_t^i  [additive]
                          ξ_t ← ξ̄_t + corresponding additive term
  3. Sparsification:      force some active features to passive
                          — sever link to robot and redistribute information to neighboring nodes
  4. State estimate:      update active feature estimates only, via amortized coordinate descent
  return ξ_t, Ω_t, μ_t
```

##### Sparsification

This is the core mechanism. The direct dependency between variables $a, b$ is approximated by the product of two marginals, creating a zero element in $\Omega$.

$$\tilde p(a,b,c) = \frac{p(a,c)\, p(b,c)}{p(c)} \quad \Longrightarrow \quad \Omega_{a,b} = 0$$

This approximation is the KL-optimal one enforcing $a \perp b | c$: it minimizes KL$(p \| q)$ over all such distributions $q$. **Variance never decreases** — information is lost, but over-confidence is guaranteed absent.

Fixing the active feature count $K$ at a constant means that the matrix inversion in the motion update is $(2K+3) \times (2K+3)$, making **every step O(1)**.

The recommended active feature count is about 6. Below this empirical number, estimation can become inconsistent (Eustice et al. 2006, "Exactly Sparse EIF"). The key visual in PR Figure 12.3 shows how the link structure of the information graph changes across the measurement, motion, and sparsification steps.

##### Tree-based data association

The additivity of information form gives a special capability in data association: **soft correspondence constraints can be added or subtracted**. A soft constraint that features $m_i$ and $m_j$ are the same object is added as

$$\Omega \leftarrow \Omega + F_{m_i - m_j}^T C\, F_{m_i - m_j}$$

and if wrong, it is removed by subtraction. This add/subtract capability lets the data association tree be searched with an A*-type frontier search. The tree itself has exponential worst-case cost and disappeared, but the philosophy that *decisions can be reversed* was inherited by switchable constraints (Sünderhauf & Protzel 2012) and Max-mixtures (Olson & Agarwal 2013).

##### Multi-robot map fusion

The additivity of information form makes multi-robot SLAM natural. After coordinate-transforming two robots' information states, **simply adding them** yields a joint map.

$$\Omega^{\text{fused}} = \Omega^{j \leftarrow k\text{-aligned}} + \Omega^k, \qquad \xi^{\text{fused}} = \xi^{j \leftarrow k\text{-aligned}} + \xi^k$$

Covariance $\Sigma$ is an inverse and cannot be added this way. Nettleton, Thrun, and Durrant-Whyte (2003) formalized this, and it is the starting point for the distributed factor graph SLAM lineage continuing through DDF-SAM (Cunningham et al. 2010), Kimera-Multi (Tian et al. 2022), and Swarm-SLAM (Lajoie & Beltrame 2024) (see §14.13).

SEIF's 2026 assessment: iSAM2 solved incremental smoothing more accurately without approximation, and SEIF itself has disappeared. The sparsification idea passes precisely into Eustice's ESEIF, then into the sliding-window marginalization of VINS-Mono, OKVIS, and MSCKF (see §14.9.2 Schur complement marginalization).

#### 14.16.4 EM Mapping (PR §13)

SEIF added sparsification to information-form additivity. One problem remained untouched. When data association is uncertain, rather than discarding ambiguous measurements, EM Mapping handles them statistically. That is where EM Mapping begins.

EKF-SLAM, EIF SLAM, and SEIF all assumed that data association was either known or decided greedily by ML. EM Mapping **treats unknown data association as an EM latent variable, exploiting ambiguous data instead of discarding it.** The prototype is from Thrun, Burgard, and Fox (1998–2000, AAAI/JAIR); a variant was used in the RHINO museum guide robot (Burgard et al. 1999).

##### E-step / M-step skeleton

```
EM_mapping(d):
  m ← initialize uniform map
  repeat until satisfied:
    // E-step (forward α)
    α^(0) = δ(⟨0,0,0⟩)
    for t = 1 to T:
      α^(t) = η P(o^(t)|s^(t),m) ∫ P(s^(t)|a^(t-1),s^(t-1)) α^(t-1) ds^(t-1)

    // E-step (backward β)
    β^(T) = uniform
    for t = T-1 downto 0:
      β^(t) = ∫ P(o^(t+1)|s^(t+1),m) P(s^(t+1)|a^(t),s^(t)) β^(t+1) ds^(t+1)

    // E-step (combine)
    Bel(s^(t)) = α^(t) · β^(t)   [normalize]

    // M-step
    for each cell ⟨x,y⟩, property l:
      m_{⟨x,y⟩=l} ∝ Σ_t ∫ P(o^(t)|s^(t),m_{⟨x,y⟩}=l) · I_{⟨x,y⟩ ∈ range} · Bel(s^(t)) ds^(t)
    normalize
  return m
```

$\alpha$ is forward localization (Markov localization); $\beta$ is backward (correcting past belief using future data). The $\beta$ term lets past belief be corrected backwards when a loop closes — the statistical core of EM mapping, most clearly visible in PR Figures 13.10–13.12. The forward-backward structure is identical to the Baum-Welch algorithm for HMMs.

The M-step is a frequentist count: "number of times the cell was observed as property l / total observations of anything," weighted by belief. Convergence typically takes 3–5 iterations.

##### Layered EM Mapping

A variant that fixes a problem in the basic EM_mapping M-step, where geometric consistency within the sensor cone is broken. A **local occupancy grid** is built from each short motion segment first; EM then optimizes only the *position* of those local maps. **Deterministic annealing** ($\sigma: 1.0 \to 0$ cooling) prevents EM from getting trapped in local maxima.

```
layered_EM_mapping(d):
  1. for each t: m^(t) = occupancy_grid(o^(t))  [local map construction]
     Bel(s^(t)) ← uniform initialization
  2. repeat until satisfied  [σ = 1.0 → 0]:
     E-step (α, β)  [using layered perceptual model]
     M-step (annealed): Bel(s^(t)) = η (α^(t) β^(t))^{1/σ}
     σ ← 0.9σ
  3. extract ML pose of each local map → compose global map via occupancy_grid()
  return m_global
```

Deterministic annealing was inherited in modified form by GNC (Yang et al. 2020) and robust kernel scheduling.

##### Why EM Mapping disappeared

Both EM_mapping and layered_EM_mapping are extinct algorithms as of 2026. The reasons: the attempt to alternate pose and map via E/M-steps lost its niche when factor graph joint optimization replaced it; the batch/offline nature does not fit real-time SLAM; Cartographer (Hess et al. 2016) and GMapping (Grisetti et al. 2007) both solved the problem directly with scan matching plus pose graph, without EM.

That said, **the submap + global alignment pattern from layered EM is Cartographer's direct ancestor** — Cartographer's local SLAM builds submaps and its global SLAM aligns them via loop closure, the same structure exactly.

#### 14.16.5 Cycle Posterior (PR §14)

The stepwise ML mapper of PR §14.3 has two limits: it cannot handle large odometry errors, and it cannot correct past poses backward in time. §14.4 runs a pose posterior estimator in parallel with the ML mapper to fix both.

**Algorithm skeleton**:
```
Incremental Mapping with Posterior Estimation:
  1. incremental_ML_mapping(o, a, s, m)  → ⟨m', s'⟩    [ML update]
  2. Bel(s') = P(o,s') ∫ P(s'|a,s) Bel(s) ds            [posterior one step]
  3. s'' = argmax Bel(s')                                 [posterior mode]
  4. s'' ≠ s'  →  cycle closure detected
                   distribute s'' − s' linearly along the cycle path
  5. run incremental_ML_mapping backwards in time          [nested ML refinement]
```

A sudden narrowing of the posterior signals cycle closure, and the difference between the narrowed mode and the ML estimate is the correction signal. Because two estimators (ML mapper + posterior estimator) run simultaneously, an MCL-based implementation is natural. The algorithm is designed to operate without odometry.

This is the **direct ancestor of loop closure detection and correction**. It is one of the earliest cases that unified explicit cycle detection with backwards correction in a single framework. If Lu and Milios (1997) batch graph SLAM is the ancestor of offline optimization, this algorithm is the ancestor of *online incremental loop closure*.

As of 2026 the algorithm itself (MCL + linear distribution + nested ML) is retired. But the *framework* — a separate detector (place recognition) plus corrector (GTSAM), posterior convergence as the closure signal, residual distributed through the graph — is the standard skeleton of modern SLAM. iSAM (Kaess et al. 2008), iSAM2 (2012), and GTSAM's incremental smoothing all inherit the core idea: "re-solve only what changed."

#### 14.16.6 Summary: What Survived

How the information-form SLAM lineage of Probabilistic Robotics (2005) was absorbed into the 2026 standard:

| PR algorithm | Core contribution | 2026 descendant | Status |
|---|---|---|---|
| EKF-SLAM | $(3N+3)$-dimensional unified state, off-diagonal covariance | MSCKF sliding window, visual fiducial systems | survives only at small scale |
| EIF/GraphSLAM | information-form additivity, variable elimination, full posterior | GTSAM, g2o, Ceres, iSAM2 | **absorbed as standard** |
| SEIF | constant-time online SLAM, sparsification | iSAM2 Bayes tree, VINS marginalization | replaced, no approximation needed |
| EM Mapping | forward-backward localization, submap concept | Cartographer submap, annealing → GNC | submap pattern survives |
| Cycle Posterior | online loop closure, detector+corrector separation | GTSAM+place recognition, iSAM2 | standardized as framework |

Information-form **additivity** → each factor in a factor graph. **Sparsification** → variable elimination and the Bayes tree. **Cycle posterior** → loop closure detection and correction. **EM's submap** → Cartographer local/global SLAM. The 2026 standard of GTSAM/iSAM2 + factor graph (see §14.9) is the union of all these insights.

> **Further reading**
> - [Thrun et al., "Simultaneous Localization and Mapping with Sparse Extended Information Filters" (IJRR 2004)](https://journals.sagepub.com/doi/10.1177/0278364904045026) — Original SEIF paper.
> - [Thrun & Montemerlo, "The GraphSLAM Algorithm with Applications to Large-Scale Mapping of Urban Structures" (IJRR 2006)](https://journals.sagepub.com/doi/10.1177/0278364906065390) — EIF/GraphSLAM formalization.
> - [Dissanayake et al., "A Solution to the Simultaneous Localization and Map Building (SLAM) Problem" (IEEE T-RA 2001)](https://ieeexplore.ieee.org/document/938381) — Classic EKF-SLAM formalization.
> - [Kaess et al., "iSAM2: Incremental Smoothing and Mapping Using the Bayes Tree" (IJRR 2012)](https://www.cs.cmu.edu/~kaess/pub/Kaess12ijrr.pdf) — Original paper of the direct descendant.

---
