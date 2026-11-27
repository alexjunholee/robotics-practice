# Ch.14 — SLAM & Odometry


In Ch.13, 3D vision algorithms reconstructed point clouds, depth maps, neural fields, and scene graphs under the assumption that camera poses were already known or easy to obtain. On an actual mobile robot operating indoors, underground, or in other GPS-denied environments, no external tracker supplies those poses. Simultaneous localization and mapping (SLAM) solves both sides of the problem at once: it estimates the robot's trajectory over time while assembling a consistent map of an unfamiliar environment, providing the spatial backbone for autonomous navigation.

---

## Part 1. Classical SLAM Foundations and Systems

### 14.1 Concept Introduction

Navigation and path planning depend on knowing both where the robot is and what surrounds it. **SLAM (simultaneous localization and mapping)** estimates the robot's pose and builds a map of the environment concurrently, resolving a classic chicken-and-egg coupling: you need a map to localize against, yet you need an accurate pose to register sensor measurements into a map.

Because physical sensors are always noisy—wheels slip on smooth floors and cameras suffer from motion blur and calibration residuals—integrating incremental motion alone causes pose estimates to accumulate error over time. That progressive error growth is called **drift**. The central task of SLAM is to detect when the robot revisits familiar places and correct accumulated drift so that the resulting trajectory and map remain globally consistent.

The table below summarizes how pure odometry differs from full SLAM across output, error behavior, and computational cost.

| Feature | Odometry | SLAM |
|---|---|---|
| Output | Relative motion | Pose + map |
| Loop closure | None | Present |
| Drift | Accumulates | Can be corrected |
| Compute | Light | Heavy |

> **Further reading**
> - [Cyrill Stachniss — SLAM Course (2013)](https://www.youtube.com/playlist?list=PLgnQpQtFTOGQrZ4O5QzbIHgl3b1JHimN_) — Public lecture series running from Bayes filters to graph-based SLAM.
> - [Thrun, Burgard, Fox, "Probabilistic Robotics" (Textbook)](https://mitpress.mit.edu/9780262201629/probabilistic-robotics/) — Foundational textbook covering the mathematics of SLAM, including Kalman filters, particle filters, and EKF-SLAM.
> - [Barfoot, "State Estimation for Robotics" (Free PDF)](http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) — Textbook with rigorous coverage of the mathematics of state estimation (free PDF available).
> - [Awesome-SLAM GitHub](https://github.com/SilenceOverflow/Awesome-SLAM) — Curated index of SLAM papers, open-source libraries, and datasets.
> - [Jinyong Jeong's blog — SLAM lecture series (based on Freiburg Robot Mapping)](https://jinyongjeong.github.io/2017/02/13/lec01_SLAM_bayes_filter/) — 15-part Korean-language series covering Bayes filters, EKF/UKF/particle filters, Graph SLAM, and Robust SLAM.
> - [Giseop Kim's blog — 5 recommended study materials for SLAM back-end](https://gisbi-kim.github.io/blog/2021/10/03/slam-textbooks.html) — Curated reading list covering the error-state Kalman filter, factor graphs, and bundle adjustment.
> - [Robot Mapping Course (Uni Freiburg, Cyrill Stachniss)](http://ais.informatik.uni-freiburg.de/teaching/ws13/mapping/) — Lecture slides and assignments for the SLAM course that pair well with the video lectures.
> - [EKF-SLAM slides (Freiburg)](http://ais.informatik.uni-freiburg.de/teaching/ws12/mapping/pdf/slam04-ekf-slam.pdf) — Cleanly organized derivations for the EKF-SLAM module of the Freiburg course.

> **Practice**: [SE(2) Odometry](https://alexjunholee.github.io/robotics-practice/app.html#se2_odometry)
> Interactively drive the odometry accumulation process on a 2D plane and observe how drift arises.

### 14.2 Visual Odometry (VO)

Visual odometry (VO) estimates a robot's frame-to-frame relative motion from camera images alone. It serves as the SLAM front-end: if the relative motion estimates produced here are noisy or biased, every downstream mapping and global optimization stage suffers accordingly.

#### 14.2.1 Feature-based vs Direct Method

Visual odometry splits into two main paradigms—feature-based and direct—whose contrasting strengths dictate which environments they handle reliably.

**Feature-based** methods, exemplified by the ORB-SLAM family, extract repeatable keypoints such as corners or blobs from each image and recover camera motion from cross-frame correspondences. In linear-algebra terms, the pipeline divides into two regimes. During initialization, before any 3D map exists, 2D-2D matches determine an essential or fundamental matrix (or a homography for planar scenes); once 3D map points are triangulated, tracking instead minimizes the reprojection error between those 3D landmarks and their 2D image observations. Feature-based pipelines handle illumination shifts well and rest on mature geometric tooling, though they struggle on textureless surfaces such as blank walls or uniform floors where few reliable keypoints appear.

```
Image → Feature extraction → Matching → Motion estimation
```

**Direct methods**, such as DSO and LSD-SLAM, skip keypoint extraction and compare raw pixel intensities across frames. Under the brightness constancy assumption—that a 3D point appears with the same intensity across consecutive views—they exploit intensity gradients anywhere in the image and therefore continue tracking in low-texture scenes. That reliance on raw intensities, however, makes them noticeably more sensitive to sudden exposure or illumination changes.

```
Image → Direct pixel intensity comparison → Motion estimation
```

#### 14.2.2 Mono vs Stereo vs RGB-D

Beyond the estimation algorithm, the choice of camera configuration determines what geometric quantities are directly observable and where the system can operate. The table below contrasts the three standard visual configurations.

| Configuration | Scale | Characteristics | Suitable environment |
|---|---|---|---|
| **Monocular** | Unavailable (ambiguity) | Light and simple; scale cannot be recovered without an IMU | Low-cost drones, mobile |
| **Stereo** | Available | Baseline limits the measurement range | General indoor/outdoor |
| **RGB-D** | Available | Measures depth directly; weak outdoors and under direct sunlight | Indoor structured environments |

Monocular cameras suffer from inherent **scale ambiguity**: from a single viewpoint, a small nearby object projects identically to a large distant one, so a purely monocular map is determined only up to an arbitrary global scale factor unless an inertial measurement unit (IMU) or another metric sensor supplies the missing reference. It is also important to separate two reasons people say "sufficient motion is required during initialization." Monocular structure initialization needs translational parallax because two-view geometry degenerates under pure rotation—yet even with arbitrary motion, pure monocular vision never makes absolute metric scale observable. Motion excitation designed to make metric scale, IMU biases, and the gravity vector observable belongs specifically to visual-inertial initialization.

> **Further reading**
> - [Daniel Cremers — Multiple View Geometry (TUM)](https://www.youtube.com/playlist?list=PLTBdjV_4f-EJn6udZ34tht9EVIW7lbeo4) — Public lectures on the multiple-view geometry underlying visual odometry.
> - [EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) — Benchmark dataset for visual and visual-inertial odometry.
> - [TUM RGB-D Benchmark](https://cvg.cit.tum.de/data/datasets/rgbd-dataset) — Widely used indoor dataset and evaluation toolkit for RGB-D SLAM and VO.

### 14.3 Visual SLAM

While VO focuses on local frame-to-frame motion, visual SLAM embeds that front-end inside a persistent mapping and loop-closure architecture so the trajectory stays globally consistent over long runs.

#### 14.3.1 ORB-SLAM2/3

The ORB-SLAM family is one of the most widely used open baselines in visual SLAM, making it an ideal reference for building and inspecting real failure cases and modeling assumptions. Its real-time performance rests on splitting the workload across three parallel threads that operate at different frequencies:

1. **Tracking** estimates the camera pose on every incoming frame in real time and decides when to promote a frame to a keyframe.
2. **Local Mapping** processes newly inserted keyframes, triangulates fresh map points, and runs local bundle adjustment to refine nearby poses and landmarks.
3. **Loop Closing** searches for revisited places across the entire map and triggers global pose-graph and bundle-adjustment optimization whenever a loop is verified.

By decoupling frame-rate tracking from slower local optimization and infrequent global corrections, ORB-SLAM maintains real-time responsiveness without sacrificing long-horizon consistency. Building on that foundation, ORB-SLAM3 extends the framework with tightly coupled visual-inertial mode, multi-map management across tracking losses, and support for fisheye camera models.

Historically, this design emerged through several milestones:
- **MonoSLAM (2007)** demonstrated real-time monocular SLAM using an EKF, though its computational cost scaled poorly as the landmark count grew.
- **PTAM (Parallel Tracking and Mapping, 2007)** introduced the crucial architectural split between parallel tracking and mapping threads that shaped modern keyframe SLAM.
- **ORB-SLAM (2015)** combined PTAM's multi-threaded design with ORB keypoints, bag-of-words loop closure, and relocalization into a complete monocular system.
- **ORB-SLAM2 (2017)** extended the pipeline to stereo and RGB-D cameras.
- **ORB-SLAM3 (2021)** added visual-inertial fusion, multi-map Atlas support, and wide-angle fisheye models.

Once built, running ORB-SLAM3 on a standard benchmark sequence such as EuRoC requires passing the vocabulary file, sensor calibration YAML, and dataset path:

```bash
# ORB-SLAM3 run example
./Examples/Monocular/mono_euroc \
    Vocabulary/ORBvoc.txt \
    Examples/Monocular/EuRoC.yaml \
    ~/Datasets/EuRoC/MH01
```

> **Further reading**
> - [Campos et al., "ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM" (2021)](https://arxiv.org/abs/2007.11898) — The ORB-SLAM3 paper.
> - [ORB-SLAM3 GitHub](https://github.com/UZ-SLAMLab/ORB_SLAM3) — Official source repository.
> - [EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) — Standard evaluation dataset for ORB-SLAM3.
> - [Jinyong Jeong's blog — Visual SLAM comparison experiment (KAIST Urban Dataset)](https://jinyongjeong.github.io/2019/10/22/visual_slam_compare/) — Head-to-head comparison of ORB-SLAM2 and VINS-Fusion on real urban data, analyzing where each system succeeds or fails.

#### 14.3.2 DSO (Direct Sparse Odometry)

Visual odometry often pairs direct methods with dense pixel grids and feature-based methods with sparse keypoints. **Direct Sparse Odometry (DSO)** breaks that convention by combining a direct photometric formulation with a sparse set of sampled points. Instead of extracting geometric descriptors, DSO selects high-gradient pixels distributed across the image and jointly optimizes camera poses, inverse depths, and camera intrinsics via photometric bundle adjustment—minimizing raw intensity residuals without keypoint extraction.

> **Further reading**
> - [Engel et al., "Direct Sparse Odometry" (2018)](https://arxiv.org/abs/1607.02565) — The DSO paper.

#### 14.3.3 VINS-Mono/Fusion

Monocular visual tracking degrades quickly during aggressive motion, motion blur, or featureless corridors. An IMU bridges those blind spots by measuring high-rate angular velocity and linear acceleration, though its own estimates drift rapidly if left uncorrected. **VINS-Mono** (and its multi-sensor successor VINS-Fusion) fuses camera frames and IMU readings in a tightly coupled visual-inertial navigation system widely deployed on drones and mobile robots.

The pipeline below illustrates how high-rate inertial measurements and visual feature tracks converge inside a sliding-window optimizer before optional loop closure:

```
Sensor input → IMU Preintegration →
Visual Feature Tracking →
Sliding Window Optimization →
Loop Closure (optional)
```

A key enabler here is **IMU preintegration**: instead of reintegrating hundreds of raw inertial samples between keyframes whenever pose or bias estimates change during nonlinear optimization, VINS-Mono summarizes the inertial stream between consecutive keyframes into a single relative-motion constraint.

> **Further reading**
> - [Qin et al., "VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator" (2018)](https://arxiv.org/abs/1708.03852) — The VINS-Mono paper.
> - [VINS-Mono GitHub](https://github.com/HKUST-Aerial-Robotics/VINS-Mono) — Official implementation with ROS support.

### 14.4 LiDAR Odometry & SLAM

Camera-based SLAM relies heavily on ambient lighting and visual texture. LiDAR avoids both dependencies by emitting laser pulses to measure 3D range directly, which makes LiDAR odometry and SLAM the workhorse for autonomous driving and outdoor mobile robots.

#### 14.4.1 LOAM (Lidar Odometry and Mapping)

Matching every point in a dense 3D LiDAR scan against a global map at frame rate is computationally expensive. **LOAM** solved this bottleneck with a two-part design that influenced virtually every subsequent LiDAR SLAM pipeline, including LeGO-LOAM and LIO-SAM. First, it classifies incoming points by local smoothness into sharp **edge features** and flat **planar features**, minimizing point-to-edge and point-to-plane distances rather than matching arbitrary raw points. Second, it separates fast scan-to-scan odometry from slower scan-to-map registration so that high-frequency motion estimates can be refined periodically against a consistent global map.

#### 14.4.2 LeGO-LOAM

Targeting lightweight ground robots with limited onboard compute, **LeGO-LOAM (Lightweight and Ground-Optimized LOAM)** refines LOAM's feature pipeline for wheeled and legged platforms. By segmenting out ground points early in each scan, it shrinks the point count fed into feature extraction and exploits the ground plane directly to stabilize roll, pitch, and vertical translation estimates.

#### 14.4.3 LIO-SAM

While early LOAM variants relied on loosely coupled scan matching, **LIO-SAM (LiDAR-Inertial Odometry via Smoothing and Mapping)** casts LiDAR-inertial SLAM as a factor graph optimization problem. A factor graph models unknown states—such as robot poses, velocities, and IMU biases—as variable nodes connected by constraint factors derived from sensor measurements. Each IMU preintegration segment, LiDAR scan match, GPS fix, or loop closure simply adds another factor to the graph, making multi-sensor extensibility clean and modular via the GTSAM library:

```
                    ┌──────────────┐
IMU ──────────────→ │              │
                    │ Factor Graph │ ──→ Pose
LiDAR ────────────→ │              │
                    │  iSAM2       │
GPS (optional) ───→ │              │
                    └──────────────┘
```

> **Further reading**
> - [Shan et al., "LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping" (2020)](https://arxiv.org/abs/2007.00258) — The LIO-SAM paper.
> - [Vizzo et al., "KISS-ICP: In Defense of Point-to-Point ICP" (RA-L 2023, arXiv:2209.15397)](https://arxiv.org/abs/2209.15397) — Shows that a carefully engineered vanilla ICP pipeline can rival complex LiDAR odometry systems, highlighting the power of simplicity.
> - [LIO-SAM GitHub](https://github.com/TixiaoShan/LIO-SAM) — Official repository with ROS support.
> - [GTSAM Documentation](https://gtsam.org/) — Factor graph optimization library used as the back-end of many SLAM systems including LIO-SAM.
> - [Frank Dellaert — Factor Graphs for Perception and Action (MIT Robotics)](https://www.youtube.com/watch?v=-yCC7mpgL4w) — GTSAM's creator explaining factor graphs from first principles.
> - [Giseop Kim's blog — Scan Context-based LiDAR Pose-graph SLAM implementation](https://gisbi-kim.github.io/blog/2021/05/17/sclidarslam.html) — Practical walk-through of integrating Scan Context into LiDAR SLAM.

#### 14.4.4 FAST-LIO / FAST-LIO2

Whereas LIO-SAM solves a nonlinear least squares problem over a factor graph, **FAST-LIO** and **FAST-LIO2** formulate tightly coupled LiDAR-inertial odometry using an iterated extended Kalman filter (IEKF). Because the iterated update of an IEKF is mathematically equivalent to Gauss-Newton optimization on a maximum a posteriori (MAP) objective, FAST-LIO is still solving an optimization problem at each scan. Its speed comes instead from two architectural choices: it reformulates the Kalman gain so its matrix inversion scales with the low state dimension rather than the thousands of LiDAR measurements, and it avoids maintaining a growing sliding window of historical states. **FAST-LIO2** pushes throughput further by replacing static spatial indices with **ikd-Tree**, an incremental KD-tree that supports fast point insertion, deletion, and nearest-neighbor queries without rebuilding the map tree from scratch, enabling direct registration without explicit edge/plane feature extraction.

> **Further reading**
> - [Xu & Zhang, "FAST-LIO: A Fast, Robust LiDAR-Inertial Odometry Package by Tightly-Coupled Iterated Kalman Filter" (2021)](https://arxiv.org/abs/2010.08196) — The FAST-LIO paper.
> - [Xu et al., "FAST-LIO2: Fast Direct LiDAR-Inertial Odometry" (2022)](https://arxiv.org/abs/2107.06829) — The FAST-LIO2 paper.
> - [FAST-LIO2 GitHub](https://github.com/hku-mars/FAST_LIO) — Official source code.

### 14.5 Multi-sensor Fusion

No single sensor covers every operating regime: cameras falter in the dark, LiDAR degrades in heavy rain or smoke, and standalone IMUs drift within seconds. Fusing complementary modalities allows each sensor to cover another's blind spots.

#### 14.5.1 Camera + IMU (VIO)

Visual and inertial streams can be combined at two levels of integration. In a **loosely-coupled** architecture, the camera and IMU estimate motion independently and their outputs are fused afterward using covariance weighting; this keeps implementation modular but discards cross-correlations between visual features and inertial states. In a **tightly-coupled** architecture, visual reprojection residuals and IMU measurements constrain one shared state directly. How that shared state is solved is a separate design axis: VINS-Mono minimizes a joint nonlinear cost function over a sliding window, whereas the Multi-State Constraint Kalman Filter (MSCKF) runs a sliding-window EKF that marginalizes feature landmarks via null-space projection. Tightly coupled systems demand more engineering effort, but they achieve higher accuracy and robustness under rapid motion.

Both approaches rely on **IMU preintegration** to summarize high-rate inertial samples between two keyframes into one relative-motion factor, avoiding repeated raw-data integration at every solver iteration (see §14.10 for the mathematical derivation).

#### 14.5.2 LiDAR + IMU (LIO)

LiDAR and IMU sampling rates vary by hardware, yet any platform motion occurring during a single LiDAR sweep warps the resulting point cloud—a phenomenon known as **motion distortion**. LiDAR-inertial odometry uses high-rate IMU measurements and per-point timestamps to de-skew each scan back into a common frame and then estimates the state jointly with LiDAR geometry. How much LIO improves over LiDAR-only odometry depends on hardware time synchronization, IMU bias stability, platform dynamics, and the LiDAR's scan pattern.

#### 14.5.3 Camera + LiDAR + IMU

A growing line of systems—such as **R3LIVE** and **LVI-SAM**—unifies all three modalities so that geometric ranging, dense visual texture, and inertial kinematics reinforce one another. R3LIVE, for example, fuses LiDAR point geometry, camera RGB texture, and IMU measurements to estimate trajectory while reconstructing a dense, photorealistic, colored 3D map in real time.

> **Further reading**
> - [KITTI Odometry Benchmark](https://www.cvlibs.net/datasets/kitti/eval_odometry.php) — Standard benchmark for LiDAR and visual odometry.
> - [EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) — Benchmark dataset for VIO.
> - [Lin & Zhang, "R3LIVE: A Robust, Real-time, RGB-colored, LiDAR-Inertial-Visual tightly-coupled state Estimation and mapping package" (2022)](https://arxiv.org/abs/2109.07982) — Representative tri-sensor fusion system.
> - [Giseop Kim's blog — Filter-based VIO: a history of the MSCKF family](https://gisbi-kim.github.io/blog/2021/04/27/msckf-history.html) — Lineage overview from the original MSCKF to stereo and modern extensions.

### 14.6 Loop Closure & Global Optimization

Even with multi-sensor fusion, dead-reckoning drift accumulates steadily as a robot traverses long distances. When the robot completes a large loop and returns to its starting area, the estimated end pose often fails to meet the start of the map. **Loop closure** resolves this discrepancy by recognizing previously visited locations and propagating that geometric constraint backward to bring the entire trajectory and map into global alignment.

#### 14.6.1 Place Recognition

The first half of loop closure is **place recognition**: identifying that the current observation corresponds to an earlier location in the trajectory. Appearance can change dramatically with time of day, weather, or viewpoint, yet false positives are far more damaging than missed detections—mistaking an unrelated corridor for a loop closure inserts a contradictory constraint that can warp the entire map. Place recognition algorithms therefore prioritize near-zero false-positive precision alongside geometric verification:

- **Bag of Words (BoW)** methods quantize local keypoints against a pre-trained visual vocabulary and compare image histograms. The DBoW2 library used in ORB-SLAM is fast and battle-tested, though sensitive to severe illumination and viewpoint shifts.
- **NetVLAD** replaces hand-crafted vocabularies with an end-to-end learned global descriptor, offering greater resilience across lighting and seasonal appearance changes (see Section 14.14).
- **LiDAR Place Recognition** avoids appearance sensitivity altogether by matching 3D geometry—for instance, **Scan Context** encodes a point cloud into a compact 2D bird's-eye-view descriptor, while **PointNetVLAD** learns global descriptors directly from 3D points.

#### 14.6.2 Pose Graph Optimization

Once place recognition proposes and geometrically verifies a loop closure, **pose graph optimization** distributes the accumulated error consistently across the trajectory. The trajectory is modeled as a graph whose nodes are robot poses and whose edges encode relative transformations from odometry and loop closures:

```
Nodes: robot poses
Edges: relative transformations (odometry, loop closure)

Goal: find node positions that satisfy all edge constraints
```

Intuitively, an odometry trajectory is locally accurate but globally warped over long distances. When a loop-closure edge asserts that two distant nodes actually represent the same physical location, pose graph optimization solves a nonlinear least squares problem to bend the entire trajectory smoothly so that all local odometry and loop constraints are satisfied as closely as possible.

In practice, three C++ solvers dominate this step: **g2o**, a lightweight solver tailored for pose graphs and bundle adjustment (used in ORB-SLAM); **GTSAM**, built around factor graphs and incremental iSAM2 optimization (used in LIO-SAM); and **Ceres Solver**, Google's general-purpose nonlinear least squares library. Selection trade-offs among these libraries are compared in Section 14.9.4.

> **Further reading**
> - [GTSAM Documentation & Tutorials](https://gtsam.org/) — Factor graph optimization library with pose graph optimization examples.
> - [Cyrill Stachniss — Graph-based SLAM](https://www.youtube.com/watch?v=uHbRKvD8TWg) — Intuitive lecture on pose graph optimization.
> - [g2o GitHub](https://github.com/RainerKuemmerle/g2o) — Graph optimization framework.
> - [Jinyong Jeong's blog — Robust Graph SLAM](https://jinyongjeong.github.io/2017/03/04/lec15_Robust_Graph_SLAM/) — Korean-language walkthrough of robust back-end techniques including M-estimators, max-mixture, and DCS.

> **Practice**: [Pose Graph Optimization](https://alexjunholee.github.io/robotics-practice/app.html#pose_graph)
> Manipulate the nodes (poses) and edges (constraints) of a pose graph and observe how the trajectory is corrected when you add a loop closure.

### 14.7 Localization

Once a map already exists, the robot no longer needs to build geometry from scratch—it only needs to estimate its current pose within that map. In production deployments, service robots commonly build a facility map once with SLAM and then run lightweight map-based localization during daily operation, though the map must be updated whenever the physical layout changes substantially.

The mathematical derivation of Monte Carlo Localization (MCL) appears in §3.11 (Ch.3), and the foundation of the extended Kalman filter (EKF) is covered in §3.10 (Ch.3). For localization extended with IMU coupling, see §14.10. Here, we organize the taxonomy of localization problems and compare the core filtering algorithms.

Two classical families dominate map-based localization:
- **Monte Carlo Localization (MCL)** uses a particle filter—most commonly pairing a 2D LiDAR with an occupancy grid map, as implemented in the ROS `AMCL` package. Intuitively, MCL scatters thousands of virtual robot hypotheses ("particles") across the map, each proposing a candidate position and heading. As sensor scans arrive, particles consistent with the map survive and multiply while inconsistent ones die out, causing the cloud to converge around the true pose.
- **LiDAR Localization** refines 3D poses directly against a point cloud map using scan-registration methods such as Iterative Closest Point (ICP) or Normal Distributions Transform (NDT).

> **Further reading**
> - [Cyrill Stachniss — Monte Carlo Localization](https://www.youtube.com/watch?v=MsYlueVDLI0) — Intuitive lecture on MCL and particle filtering.
> - [ROS Navigation Stack — AMCL](http://wiki.ros.org/amcl) — Reference documentation for running MCL in ROS.

> **Practice**: [Particle Filter](https://alexjunholee.github.io/robotics-practice/app.html#particle_filter)
> Visualize particle filter localization interactively and observe how particles converge around the true pose.

> **Practice**: [Occupancy Grid](https://alexjunholee.github.io/robotics-practice/app.html#occupancy_grid)
> Visualize 2D occupancy grid construction and see how range measurements turn into a probabilistic map.

#### 14.7.1 Classification of Localization Problems

Localization difficulty cannot be ranked on a single scale; four operational axes interact to determine which algorithm and belief representation are required.

| Axis | Options | Note |
|---|---|---|
| Prior knowledge | position tracking → global localization → kidnapped robot | increasing difficulty |
| Environment | static (robot only moves) → dynamic (people, doors, lighting) | harder as dynamism grows |
| Agency | passive (observe only) → active (choose exploration actions) | active converges faster |
| Robot count | single → multi (belief sharing via mutual observation) | multi yields richer information |

Along the prior-knowledge axis, two problems are canonical and the third tests recovery:

- **Position tracking** assumes the initial pose is known, so the belief remains a tight unimodal Gaussian around the true state—making EKF Localization a natural fit.
- **Global localization** begins with no knowledge of the initial pose. The belief starts uniform across all valid space and must represent multiple competing modes until measurements break the symmetry, requiring Grid Localization or MCL.
- **Kidnapped robot** occurs when an operating robot is abruptly moved to a different location without odometry awareness. It is harder than global localization because the filter has already converged confidently on the wrong pose; surviving this failure is a direct measure of autonomy.

In practice, warehouse AGVs and cleaning robots face global localization at boot-up and transition to position tracking during routine operation, while ROS Nav2's `recovery_alpha_slow/fast` parameters explicitly safeguard against kidnapped-robot events.

#### 14.7.2 Markov Localization

Markov localization is less a standalone algorithm than **the direct application of the Bayes filter to pose estimation**. EKF Localization, Grid Localization, and MCL all instantiate this same probabilistic skeleton; they differ only in how they represent and propagate the belief `bel(x_t)`.

Compared with the general Bayes filter (Ch.3 §3.9), the only change is that both the motion model and the measurement model condition explicitly on **the map `m`**:

```
Markov_localization(bel(x_{t-1}), u_t, z_t, m):
  for all x_t do
    bel̄(x_t) = ∫ p(x_t | u_t, x_{t-1}, m) bel(x_{t-1}) dx_{t-1}   // motion update
    bel(x_t) = η p(z_t | x_t, m) bel̄(x_t)                          // measurement update
  endfor
  return bel(x_t)
```

Different problem regimes enter through the initial belief `bel(x_0)`:
- Position tracking starts with a narrow Gaussian: $\text{bel}(x_0) = \mathcal{N}(x_0;\, \bar{x}_0, \Sigma)$.
- Global localization starts uniform over all valid poses: $\text{bel}(x_0) = 1/|X|$.
- Partial prior knowledge initializes uniformly over the plausible neighborhood and zero elsewhere.

The methods in §14.7.3–§14.7.7 are concrete numerical realizations of how `bel` is stored and updated in the loop above.

#### 14.7.3 EKF Localization

EKF Localization specializes Markov localization by representing the belief as a single Gaussian $(\mu_t, \Sigma_t)$. Because that representation is strictly unimodal, **EKF Localization is suited only to position tracking**; global localization and the kidnapped-robot problem require multi-modal beliefs that a single Gaussian cannot express.

Applying the EKF from §3.10.2 (Ch.3) to a feature-based map `m` of point landmarks yields the following formulation. Suppose each measurement $z_t^i = (r, \phi, s)^T$ records range, bearing, and a landmark signature, and the correspondence $c_t^i$ is known (as with identifiable fiducials such as ARTags, QR codes, or distinct landmarks like the Eiffel Tower):

```
EKF_localization_known_correspondences(μ_{t-1}, Σ_{t-1}, u_t, z_t, c_t, m):
  // Motion update (linearized velocity model)
  μ̄_t = μ_{t-1} + [velocity model displacement]
  G_t = ∂g/∂x |_{μ_{t-1}, u_t}         // 3×3 Jacobian
  Σ̄_t = G_t Σ_{t-1} G_t^T + R_t

  // Measurement update (sequential updates over landmarks)
  μ_t = μ̄_t
  Σ_t = Σ̄_t
  for each observed z_t^i = (r, φ, s)^T do
    j = c_t^i
    δ = (m_{j,x} − μ_{t,x},  m_{j,y} − μ_{t,y})^T,   q = δ^T δ
    ẑ_t^i = (√q,  atan2(δ_y, δ_x) − μ_{t,θ},  m_{j,s})^T
    H_t^i = Jacobian (3×3, last row is 0 — signature independent of pose)
    K_t^i = Σ_t H_t^{i,T} (H_t^i Σ_t H_t^{i,T} + Q_t)^{-1}
    μ_t = μ_t + K_t^i (z_t^i − ẑ_t^i)
    Σ_t = (I − K_t^i H_t^i) Σ_t
  endfor
  return μ_t, Σ_t
```

Under the **conditional independence assumption** $p(z_t | x_t, m) = \prod_i p(z_t^i | x_t, m)$, observations from a single time step can either be stacked into one batch update or processed sequentially as written above, where each landmark update immediately refines $(\mu_t, \Sigma_t)$ for the next one. Because the observation model is nonlinear, relinearization at intermediate means can cause the sequential and stacked updates to diverge slightly.

As a practical caveat, *Probabilistic Robotics* cites heading uncertainty around ±20° as an illustrative regime where first-order linearization begins to break down. That figure is not a universal cutoff; actual consistency depends on observation geometry, motion dynamics, and sensor noise, which should be verified with normalized innovation squared (NIS), normalized estimation error squared (NEES), or Monte Carlo consistency tests. When the belief stays unimodal—such as in ARTag/AprilTag navigation or certain GNSS+IMU fusion pipelines—EKF localization remains fast and reliable.

**Unknown correspondences**: In most real environments, $c_t^i$ is unknown. Maximum likelihood (ML) data association pairs each observation with the map landmark that minimizes the Mahalanobis distance under the innovation covariance $\Psi_k$:

$$j(i) = \arg\min_k (z_t^i - \hat{z}_t^k)^T \Psi_k^{-1} (z_t^i - \hat{z}_t^k), \quad \Psi_k = H_t^k \bar\Sigma_t H_t^{k,T} + Q_t$$

When candidate priors and covariance determinants are equal, minimizing Mahalanobis distance is equivalent to maximizing the Gaussian log-likelihood. Production implementations supplement this rule with a $\chi^2$ validation gate matched to the measurement dimension and one-to-one assignment constraints within each frame. Descriptor matching followed by geometric verification in ORB-SLAM plays a comparable role of candidate proposal and outlier rejection, though it does not directly evaluate this EKF Mahalanobis association rule.

#### 14.7.4 Multi-Hypothesis Tracking (MHT)

Because an EKF maintains only a single Gaussian, it cannot preserve competing data-association hypotheses when landmarks look alike. **Multi-Hypothesis Tracking (MHT)** overcomes this limitation by representing the belief as a **Gaussian mixture** and propagating multiple association hypotheses in parallel.

Each hypothesis $h$ runs its own EKF instance. When new observations arrive, candidate associations branch each hypothesis, and any branch whose posterior weight drops below a threshold $\psi_{\min}$ is pruned so the hypothesis tree does not grow exponentially. Note that multi-object tracking pipelines often pair Mahalanobis gating with Hungarian assignment; that chooses a single one-to-one matching per frame and should not be confused with MHT, which preserves multiple competing association histories across time.

#### 14.7.5 Grid Localization

Rather than tracking a Gaussian or a small mixture of Gaussians, **Grid Localization** tackles global and multi-modal beliefs directly by partitioning the entire pose space into discrete cells and maintaining a probability mass per cell as a **histogram filter**. It handles global localization cleanly, at the cost of computation scaling linearly with the total cell count $K$:

```
Grid_localization({p_{k,t-1}}, u_t, z_t, m):
  for all k do
    p̄_{k,t} = Σ_i p_{i,t-1} · motion_model(mean(x_k), u_t, mean(x_i))
    p_{k,t}  = η · measurement_model(z_t, mean(x_k), m) · p̄_{k,t}
  endfor
  return {p_{k,t}}
```

Here $\text{bel}(x_t) = \{p_{k,t}\}$ assigns one probability weight to each cell $x_k$, normalized to sum to 1.

This discretization introduces a direct **resolution trade-off**: finer cells reduce pose quantization error, but shrinking cell size causes CPU cost to balloon during global localization. Practical implementations mitigate that cost by pre-caching raycast tables, subsampling laser beams, and skipping updates on cells whose prior probability falls below a threshold. Conceptually, Grid Localization provides the clearest bridge to particle filters—ROS `amcl` can be viewed as Grid Localization with fixed histogram cells replaced by adaptive particles.

#### 14.7.6 MCL Algorithm (Expanded)

With the mathematical derivation of MCL established in §3.11 (Ch.3), we state the complete localization loop explicitly here. Each time step executes three phases—**predict (sample), weight, and resample**—over a set of $M$ weighted pose hypotheses:

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

Initialization mirrors Markov localization: sample $M$ particles uniformly over free space for global localization, or from a narrow Gaussian for position tracking. Instead of fixing $M$, systems can also adaptively sample as many particles as time permits before the next scan arrives, automatically trading faster CPUs for higher localization accuracy.

One subtle failure mode arises because the proposal distribution is solely the motion model: when a sensor is nearly noiseless, its measurement likelihood peaks sharply in a tiny region of state space, causing almost every motion-predicted particle to receive near-zero weight. Mixture MCL (§14.7.8) resolves this degeneracy. In ROS 2 Nav2, `nav2_amcl` implements this particle-filter architecture directly.

#### 14.7.7 Augmented MCL — Kidnapping Recovery

Standard MCL recovers poorly if a robot is kidnapped after its particle cloud has collapsed onto a single pose, because no particles remain near the new true location. **Augmented MCL** solves this by monitoring two exponential moving averages of the average particle weight—a slow long-term baseline $w_{\text{slow}}$ and a fast short-term average $w_{\text{fast}}$—and injecting random global particles whenever recent measurements suddenly stop matching the map:

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

With smoothing rates satisfying $0 \le \alpha_{\text{slow}} \ll \alpha_{\text{fast}}$ (for example, $\alpha_{\text{slow}} = 0.001$ and $\alpha_{\text{fast}} = 0.1$), the random-injection probability at each resampling draw is:

$$p_{\text{inject}} = \max\!\left(0,\, 1 - \frac{w_{\text{fast}}}{w_{\text{slow}}}\right)$$

During normal operation, $w_{\text{fast}} \approx w_{\text{slow}}$, so the ratio stays near 1, $p_{\text{inject}} \approx 0$, and the filter behaves identically to standard MCL. Immediately after a kidnapping event, measurements fail to match the current hypothesis cluster, driving $w_{\text{fast}}$ down sharply and triggering uniform particle injection until the robot relocalizes and $w_{\text{slow}}$ catches back up. Because a transient burst of sensor noise can also depress $w_{\text{fast}}$—and the slow drift of $w_{\text{slow}}$ serves as the reference baseline rather than a spike filter—avoiding spurious injections depends on tuning $\alpha_{\text{fast}}$.

In ROS `amcl`, `recovery_alpha_slow` and `recovery_alpha_fast` control this mechanism. While some research systems inject learned pose candidates from NetVLAD-style place recognition and PnP into the particle set, that is not standard AMCL behavior and requires validation in the target environment.

#### 14.7.8 Mixture MCL

Where Augmented MCL injects uniform random poses when likelihood drops, **Mixture MCL** alters the proposal distribution itself by drawing a fraction $\rho$ of particles directly from the **measurement model** rather than solely from the motion model:

$$x_t^{[k]} \sim \begin{cases} p(z_t | x_t, m) & \text{with probability } \rho \\ \text{sample\_motion\_model}(u_t, x_{t-1}^{[k]}) & \text{with probability } 1 - \rho \end{cases}$$

Sampling directly from observations concentrates particles in high-likelihood regions, eliminating proposal collapse with low-noise sensors while simultaneously enabling kidnapping recovery. Its main implementation cost is requiring an inverse sensor model capable of sampling poses from $p(z_t | x_t, m)$.

#### 14.7.9 Dynamic Environment Filtering

In crowded spaces filled with pedestrians or moving vehicles, range beams frequently strike dynamic obstacles absent from the static map. For each beam $z_t^k$, evaluating the four-component mixture beam model (§2.7, Ch.2) yields the posterior probability of the short-hit component $p_{\text{short}}(z | x, m)$. Excluding beams with high short-hit probability from the particle-weight calculation prevents moving crowds in corridors or lobbies from destabilizing MCL.

#### 14.7.10 Filter Comparison Summary

The table below compares the four localization filters across belief representation, supported operating regimes, and per-step computational scaling, where $N$ is the landmark count, $H$ is the active hypothesis count, $K$ is the grid-cell count, and $M$ is the particle count.

| Algorithm | Belief representation | Position tracking | Global loc | Kidnapped | Compute cost |
|---|---|---|---|---|---|
| EKF Loc | Gaussian (μ, Σ) | good | impossible | impossible | O(N) |
| MHT | Gaussian mixture | good | limited | limited | O(H·N) |
| Grid Loc | histogram | good | possible | possible | O(K) |
| MCL | particle set | good | possible | possible with Augmented MCL | O(M) |

Because an EKF enforces a unimodal Gaussian belief, it cannot solve global localization or kidnapped-robot recovery. Grid Localization and MCL both support multi-modal beliefs and let engineers tune compute against accuracy by adjusting $K$ or $M$.

#### 14.7.11 Implementation Notes: Landmark Efficiency and Negative Information

Deploying EKF Localization in practice introduces four recurring engineering details:

- **Efficient landmark search**: Exhaustive correspondence search over $N$ landmarks costs $O(N)$ per observation. In low dimensions, a balanced KD-tree achieves $O(\log N)$ average lookup time (though worst-case remains $O(N)$), while spatial grid hashing depends on local cell occupancy and search radius.
- **Mutual exclusion**: Two observations in the same scan cannot originate from the same physical landmark, yet element-wise ML association does not enforce uniqueness. When two measurements claim the same landmark, keep the match with the smaller Mahalanobis distance and discard or re-associate the conflicting one.
- **Outlier rejection**: Dropping observations whose Mahalanobis distance exceeds the $\chi^2_{95\%}$ threshold is one of the simplest and most effective safeguards against EKF divergence.
- **Negative information**: Failing to observe an expected landmark within the sensor's field of view carries probabilistic evidence about pose, but modeling missed detections rigorously adds substantial algorithmic complexity, so most production systems omit negative information.

---

### 14.7B Occupancy Grid Mapping

While §14.7 estimated robot pose given a known map, **Occupancy Grid Mapping** flips the problem around: **given known poses, it estimates the occupancy probability of every grid cell.** In a complete SLAM system, pose graph optimization first locks down a consistent pose trajectory, and occupancy grid mapping runs afterward to construct the final navigable map. Its probabilistic foundation in binary Bayes filters is derived in §3.11.2 (Ch.3).

#### 14.7B.1 Introduction: Why Mapping Is Hard

Mapping is fundamentally higher-dimensional than localization. A planar pose is a continuous vector $x_t \in \mathbb{R}^3$, whereas a discretized map $m$ contains tens of thousands to millions of cells—yielding $2^{|m|}$ possible binary maps and making joint search intractable.

Two assumptions keep the problem tractable: (1) **poses $x_{1:t}$ are known**, and (2) **cells are conditionally independent** given poses and measurements. Independence factors the intractable joint map posterior into a product of single-cell marginals, turning mapping into one independent binary Bayes filter per cell:

$$p(m \mid z_{1:t}, x_{1:t}) = \prod_i p(m_i \mid z_{1:t}, x_{1:t})$$

Even under this factorization, practical mapping must still contend with sensor noise, perceptual aliasing, dynamic obstacles, and residual pose errors around closed loops.

#### 14.7B.2 Standard Algorithm: Log-Odds Accumulation

To avoid numerical underflow near 0 and 1 and turn multiplicative Bayes updates into simple additions, each cell's occupancy belief is stored in **log-odds** form:

$$l_{t,i} = \log \frac{p(m_i \mid z_{1:t}, x_{1:t})}{1 - p(m_i \mid z_{1:t}, x_{1:t})}$$

Let $l_0 = \log[p(m_i) / (1 - p(m_i))]$ denote the prior log-odds. From the binary Bayes filter derivation (§3.11.2, Ch.3), each cell inside the sensor's perceptual field updates recursively as:

$$l_{t,i} = l_{t-1,i} + \text{inverse\_sensor\_model}(m_i, x_t, z_t) - l_0$$

Intuitively, hit evidence for cell $m_i$ raises $l_{t,i}$, free-space rays lower it, and the $-l_0$ term prevents the prior from being counted repeatedly across updates:

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

Whenever probabilities are needed, they are recovered via $p(m_i | z_{1:t}, x_{1:t}) = 1 - 1/(1 + \exp\{l_{t,i}\})$.

For a 2D range finder, a simplified `inverse_sensor_model` assigns each cell within the beam cone to occupied (`l_occ`), free (`l_free`), or uninformative (`l_0`) based on its range relative to the nearest ray:

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

Here $\alpha$ controls modeled obstacle thickness and $\beta$ denotes the beam opening angle. Real SLAM and navigation packages vary in how they implement these updates: Cartographer's submap probability grid updates occupancy via an odds product backed by a lookup table; ROS Nav2's `costmap_2d` maintains 0–254 cost values via raytraced marking and clearing; and SLAM Toolbox (from the Karto lineage) thresholds per-cell hit-to-visit ratios rather than storing log-odds directly.

#### 14.7B.3 Multi-Sensor Fusion

Cameras, LiDAR, sonar, and infrared sensors each require a tailored `inverse_sensor_model`. The simplest way to fuse them is a **conservative per-cell maximum**: if any sensor indicates occupancy, the cell is marked occupied. While safe for collision avoidance, this rule can systematically underestimate navigable free space. An alternative is to accumulate log-odds updates independently per sensor and combine them via a weighted sum reflecting each modality's reliability.

#### 14.7B.4 Learning the inverse_sensor_model

Hand-crafted geometric inverse models are simple approximations that break down when sensor physics grows complicated. **When a realistic forward model $p(z | x, m)$ or simulator is available, the inverse model can be learned directly from data.** By generating pose-scan-occupancy triples $\{(x^{(k)}, z^{(k)}, m_i^{(k)})\}$ in simulation, a function approximator can be trained with binary cross-entropy loss:

$$\mathcal{L} = -\sum_k \left[m_i^{(k)} \log \hat{p}_i + (1 - m_i^{(k)}) \log(1 - \hat{p}_i)\right]$$

A neural network taking $(x, z)$ as input and outputting $\hat{p}_i = p(m_i | x, z)$ takes over the role of `inverse_sensor_model`, which is especially useful when complex sensor geometry—such as multipath sonar reflections or LiDAR behavior on glass—resists analytical modeling.

#### 14.7B.5 MAP Occupancy Mapping (Advanced)

Assuming independent cells creates a subtle inconsistency: adjacent cells within a single beam cone receive correlated evidence in reality, yet independent binary Bayes updates ignore that coupling—an effect especially visible with wide-beam sonar.

**MAP Occupancy Mapping** drops cell independence and directly seeks the maximum a posteriori map across all poses and measurements:

$$m^* = \arg\max_m \left[\sum_t \log p(z_t \mid x_t, m) + \log p(m)\right]$$

Instead of an inverse model, it evaluates the **forward sensor model** $p(z_t | x_t, m)$ directly. Initializing every cell as free, hill-climbing flips individual cells whenever doing so increases the joint log-posterior:

```
MAP_occupancy_grid_mapping(x_{1:t}, z_{1:t}):
  m ← initialize all cells free
  repeat until convergence:
    for all cells m_i do
      m_i ← argmax_{k ∈ {0,1}} [k·l_0 + Σ_t log measurement_model(z_t, x_t, m | m_i=k)]
  return m
```

In practice, this algorithm is strictly batch (unsuited to incremental SLAM), vulnerable to local maxima during coordinate ascent, and discards posterior cell uncertainty. Its lasting value lies in highlighting why breaking the cell-independence assumption produces cleaner maps under wide-beam or correlated observations.

#### 14.7B.6 Comparison with Other Spatial Representations: OctoMap, Voxblox, NeRF, 3DGS

Occupancy grids anchor 2D navigation stacks such as SLAM Toolbox and the Nav2 costmap, and **OctoMap** extends them directly into 3D via hierarchical octrees. **Voxblox** and **nvblox**, by contrast, belong to the signed-distance-field family (TSDFs), storing continuous distances to surfaces rather than binary occupancy probabilities. **NeRF** density fields and **3D Gaussian Splatting** opacities also integrate color and transparency along viewing rays, but they are learned novel-view synthesis representations rather than direct descendants of binary Bayes occupancy filters. Although these families share mathematical tools such as forward sensor models and ray integration, their objectives and theoretical lineages remain distinct.

> **Practice**: [Occupancy Grid](https://alexjunholee.github.io/robotics-practice/app.html#occupancy_grid)
> Step through log-odds accumulation cell by cell and observe how hit and free updates from `inverse_sensor_model` build a coherent map.

---

## Part 2. Learning-Based SLAM

### 14.8 Learning-based & Neural SLAM

Classical SLAM pipelines rely on hand-crafted keypoints, geometric descriptors, and explicit optimization back-ends. Over the past few years, deep learning has begun replacing individual front-end modules—and in some cases the entire map representation itself.

**DROID-SLAM (2021)** replaces discrete keypoint extraction and matching with dense recurrent optical flow. By iteratively updating dense flow fields and passing them through a differentiable dense bundle adjustment (DBA) layer, DROID-SLAM estimates camera poses and pixel depths end to end. Because learned optical flow degrades far more gracefully than hand-crafted corners in textureless corridors or shifting lighting, DROID-SLAM overcomes key failure modes of both feature-based pipelines like ORB-SLAM and photometric direct methods like DSO—at the cost of requiring a GPU and higher compute than lightweight CPU trackers.

**3DGS-SLAM fusion** bridges classical tracking with the 3D Gaussian Splatting representation introduced in Section 13.5.2. Systems such as **SplaTAM** and **MonoGS** replace sparse point clouds or occupancy grids with anisotropic 3D Gaussians as the live map, yielding photorealistic reconstructions suited to novel-view synthesis and augmented-reality overlays as the robot moves.

> **Further reading**
> - [Teed & Deng, "DROID-SLAM: Deep Visual SLAM for Monocular, Stereo, and RGB-D Cameras" (2021)](https://arxiv.org/abs/2108.10869) — The DROID-SLAM paper.
> - [Keetha et al., "SplaTAM" (2024)](https://arxiv.org/abs/2312.02126) — Dense SLAM built on 3D Gaussian Splatting.
> - [Awesome-SLAM GitHub](https://github.com/SilenceOverflow/Awesome-SLAM) — Curated index of recent SLAM papers and open-source projects.

---

## Part 3. Advanced Topics

### 14.9 Advanced: SLAM Back-end Optimization

For the history of information-form SLAM before the factor graph era (EKF-SLAM, EIF, SEIF, EM), see §14.16 Advanced: History of Information-Form SLAM.

While the SLAM front-end extracts relative motion and landmark observations from raw sensor streams, the **back-end** solves for the globally consistent set of poses and landmarks that best agrees with all of those measurements. Every time a robot drives forward ("I moved 1 m") and observes a feature ("that landmark sits 3 m ahead"), it adds a noisy constraint. Because no exact solution satisfies every noisy measurement simultaneously, the back-end formulates a **nonlinear least squares** problem that minimizes the sum of weighted squared residuals. Understanding this mathematical machinery explains why solvers such as g2o, GTSAM, and Ceres are structured—and configured—the way they are.

Because observation models and 3D rigid transforms are nonlinear, the back-end linearizes the objective around the current state estimate and updates it iteratively via Gauss-Newton steps: linearize around the current estimate, solve the linear system `Ax = b`, apply the state increment, and repeat until convergence (see also [Giseop Kim's blog — SLAM back-end series](https://gisbi-kim.github.io/blog/2021/03/04/slambackend-1.html)).

#### 14.9.1 Gauss-Newton on a Manifold

Robot poses live on the Lie group `SE(3)`, not in a flat Euclidean vector space. A naive vector update `x ← x + δx` cannot be applied directly to a rotation matrix without breaking orthogonality and determinant constraints.

The standard remedy is to parameterize small increments `δξ ∈ R^6` in the Lie algebra `se(3)` and apply a **left perturbation** on the manifold:

```
T ← exp(δξ^) · T
```

Here `exp(·)` denotes the exponential map and `^` (the hat operator) maps a 6-vector into a 4x4 `se(3)` matrix.

Linearizing an error term `e(T)` with respect to this perturbation requires computing the Jacobian:

```
J = ∂e / ∂δξ
```

By the chain rule, `J` factors into `∂e/∂(Tp) · ∂(exp(δξ^)Tp)/∂δξ`, where the second factor is the derivative of the group action under left perturbation; for a transformed 3D point `Tp`, it equals `[I, -(Tp)^]` when `δξ` orders translation before rotation. (Note that this action derivative is distinct from the 6x6 left Jacobian of `SE(3)`, which relates Lie algebra perturbations through the Baker–Campbell–Hausdorff (BCH) formula.)

Accumulating the linearized errors across all factors yields the Gauss-Newton **normal equation**:

```
(J^T Σ^{-1} J) δξ* = -J^T Σ^{-1} e
```

Here `Σ` is the measurement noise covariance, and `H = J^T Σ^{-1} J` is the Gauss-Newton Hessian approximation—also called the **information matrix**. Crucially, independent constraints sum additively into `H` via their individual `J^T Σ^{-1} J` blocks. At each iteration, the solver updates `T` on the manifold, recomputes the Jacobians at the new operating point, and repeats until the step size falls below a tolerance.

#### 14.9.2 Schur Complement (Marginalization)

In visual bundle adjustment (BA), the state partitions naturally into camera poses `p` and 3D landmarks `l`. Grouping variables by type exposes a clean 2x2 block structure in the normal equation:

```
[H_pp  H_pl] [δp]   [b_p]
[H_lp  H_ll] [δl] = [b_l]
```

Suppose there are `m` poses and `n` landmarks, where typically `n >> m` (hundreds of poses observing tens of thousands of points). Solving the full `(m + n)`-sized linear system directly is prohibitively slow. Instead, eliminating the landmarks via the **Schur complement** yields a much smaller reduced camera system solely in `δp`:

```
(H_pp - H_pl · H_ll^{-1} · H_lp) δp = b_p - H_pl · H_ll^{-1} · b_l
```

This elimination is fast because **`H_ll` is block diagonal**: landmarks never share direct measurement factors with other landmarks, only with the camera poses that observe them. Inverting `H_ll` therefore reduces to inverting `n` tiny 3x3 blocks independently in `O(n)` time. While the reduced system dimension depends only on pose count `m`, forming the Schur complement and recovering landmarks still scale with the number of observations and landmarks; exploiting this block sparsity is what makes large-scale BA practical, with actual throughput governed by graph connectivity, linear solver choice, and hardware.

Once the reduced system yields `δp`, each landmark update `δl` follows cheaply by back-substitution:

```
δl = H_ll^{-1} (b_l - H_lp · δp)
```

#### 14.9.3 Sparsity and Variable Ordering

In pose-graph optimization, the Hessian `H` is typically **sparse** because each factor couples only a handful of poses—odometry links consecutive frames, while loop closures link revisited pairs. Node degree varies with trajectory length and loop frequency, and dense loop proposals or poor elimination orderings can degrade that sparsity.

Solving `H δx = b` directly via Cholesky factorization (`H = L L^T`) encounters **fill-in**: entries that were zero in `H` become non-zero in the Cholesky factor `L`, inflating both memory and runtime. Choosing a good variable ordering before factorization is essential to keep `L` sparse:

- **COLAMD** (Column Approximate Minimum Degree) orders columns heuristically to limit fill-in during sparse least-squares QR or Cholesky solves.
- **AMD** (Approximate Minimum Degree) targets symmetric positive-definite matrices such as `H` directly.
- **Nested dissection** recursively partitions the factor graph via vertex separators, scaling well on very large graphs.

When configuring g2o, GTSAM, or Ceres, both the linear solver (such as `DENSE_SCHUR` or `SPARSE_NORMAL_CHOLESKY`) and the ordering strategy matter; because fill-in depends strongly on graph topology, measure memory footprint and solve latency on representative trajectories:

```python
# Example of setting the ordering in Ceres Solver (Python binding)
options = ceres.SolverOptions()
options.linear_solver_type = ceres.LinearSolverType.SPARSE_NORMAL_CHOLESKY
options.sparse_linear_algebra_library_type = ceres.SparseLinearAlgebraLibraryType.SUITE_SPARSE
# ordering typically defaults to COLAMD automatically, but manual configuration is possible
```

#### 14.9.4 Comparison of Optimization Libraries

The table below contrasts the three standard C++ back-end solvers across design philosophy and common SLAM deployments.

| Library | Characteristics | Primary uses |
|---|---|---|
| **g2o** | Dedicated to pose graph / BA, lightweight, C++ only | ORB-SLAM2/3, LSD-SLAM |
| **GTSAM** | Factor graph based, supports Bayes tree (iSAM2), strong at incremental optimization | LIO-SAM, research |
| **Ceres Solver** | General-purpose nonlinear least squares, supports auto-diff, developed by Google | Cartographer, VINS-Fusion, various projects |

As a rule of thumb, choose **g2o** when you want a minimal, lightweight C++ solver dedicated to pose graphs or BA; choose **GTSAM (iSAM2)** when factor-graph modeling and incremental updates as new keyframes arrive are priorities; and choose **Ceres Solver** when you need general-purpose nonlinear optimization or automatic differentiation without deriving analytical Jacobians by hand.

> **Further reading**
> - Barfoot, "State Estimation for Robotics" Ch.4 (Nonlinear Estimation) — Systematic treatment of optimization on manifolds.
> - [Dellaert & Kaess, "Factor Graphs for Robot Perception" (Foundations and Trends in Robotics, 2017)](https://www.cs.cmu.edu/~kaess/pub/Dellaert17fnt.pdf) — Definitive reference on factor graphs and SLAM back-end theory.
> - [g2o Tutorial](https://github.com/RainerKuemmerle/g2o) / [GTSAM Tutorial](https://gtsam.org/tutorials/intro.html) — Hands-on tutorials for each library.
> - [Giseop Kim's blog — Gauss-Newton Opt == IEKF update?](https://gisbi-kim.github.io/blog/2022/03/05/gn-iekf-same.html) — Exposition of the mathematical equivalence between Gauss-Newton optimization and iterated Kalman filtering.

> **Practice**: [Bundle Adjustment Visualization](https://alexjunholee.github.io/robotics-practice/app.html#bundle_adjustment)
> Interactively explore bundle adjustment as camera poses and 3D landmarks are optimized jointly.

### 14.10 Advanced: IMU Preintegration

For localization extended with IMU coupling, see §14.7, and see §14.7B for the mapping foundation. Section 14.3.3 introduced IMU preintegration conceptually inside VINS-Mono; here we examine the mathematical formulation that makes high-rate inertial fusion tractable inside a keyframe optimizer.

An IMU typically outputs angular velocities and linear accelerations at 200–1000 Hz, whereas visual or LiDAR keyframes are added at a few Hz to tens of Hz. Creating a full state node (pose, velocity, and biases) at every single inertial sample would cause the factor graph to explode in size. **IMU preintegration** avoids that bottleneck by summarizing all inertial measurements between two keyframes `i` and `j` into a single relative-motion factor.

Specifically, integrating the raw gyroscope readings `ω_k` and accelerometer readings `a_k` (with sampling interval `Δt` and current bias estimates `b_g`, `b_a`) yields three preintegrated quantities expressed **in the local coordinate frame of keyframe `i`**:

```
ΔR_ij = Π_{k=i}^{j-1} Exp((ω_k - b_g) · Δt)          # relative rotation
Δv_ij = Σ_{k=i}^{j-1} ΔR_ik · (a_k - b_a) · Δt        # relative velocity
Δp_ij = Σ_{k=i}^{j-1} (Δv_ik · Δt + 0.5 · ΔR_ik · (a_k - b_a) · Δt^2)  # relative position
```

Because these increments live purely in keyframe `i`'s local frame, adjusting keyframe `i`'s global pose or velocity during optimization does not require reintegrating the raw IMU stream.

To weight this factor properly in the optimizer, discrete-time covariance propagation tracks how sensor noise `Q` (from the IMU datasheet) accumulates across each step via the state-transition Jacobian `A_k` and noise-input Jacobian `B_k`:

```
Σ_{k+1} = A_k · Σ_k · A_k^T + B_k · Q · B_k^T
```

The inverse of the resulting covariance `Σ^{-1}` supplies the information matrix of the preintegrated factor.

One subtlety remains: if the optimizer updates the gyroscope or accelerometer bias estimates (`b_g`, `b_a`) during iteration, the preintegrated quantities strictly depend on those biases. Rather than reintegrating hundreds of samples from scratch whenever `b` shifts slightly by `δb`, preintegration applies a **first-order Taylor correction** using bias Jacobians accumulated alongside the original integration:

```
ΔR_ij ≈ ΔR_ij^0 · Exp(∂ΔR/∂b_g · δb_g)
Δv_ij ≈ Δv_ij^0 + ∂Δv/∂b_g · δb_g + ∂Δv/∂b_a · δb_a
Δp_ij ≈ Δp_ij^0 + ∂Δp/∂b_g · δb_g + ∂Δp/∂b_a · δb_a
```

Here `^0` marks the quantities computed at the nominal bias estimate. As long as bias updates remain small—which holds well in practice—this linear correction stays accurate while keeping optimization fast.

**Why preintegrate on the manifold**: Lupton and Sukkarieh originally proposed inertial preintegration using Euler angles, which suffer from gimbal-lock singularities. Forster et al. (2015 RSS, 2017 TRO) reformulated preintegration directly on the rotation group `SO(3)`, preserving group structure without singularities and producing residuals that plug cleanly into on-manifold factor graphs (discrete-time integration error and sensor noise still apply).

**Tightly-coupled vs loosely-coupled LIO**: LIO-SAM illustrates why this distinction matters in LiDAR-inertial systems. In a **loosely-coupled** pipeline, the IMU either merely de-skews point clouds and seeds scan matching (as in LeGO-LOAM) or runs as a separate estimator fused afterward by covariance. In a **tightly-coupled** pipeline such as LIO-SAM, the IMU preintegration factor sits directly inside the same GTSAM factor graph alongside LiDAR odometry, optional GPS, and loop-closure factors. LiDAR odometry separately extracts edge and planar features into multi-resolution voxel maps to minimize point-to-line and point-to-plane distances, while the IMU factor bridges rapid rotations or sharp accelerations that scan matching alone would miss—a design shared across modern VIO/LIO systems including VINS-Mono, ORB-SLAM3's inertial mode, and LIO-SAM.

> **Further reading**
> - [Forster et al., "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry" (TRO 2017, arXiv:1512.02363)](https://arxiv.org/abs/1512.02363) — Foundational on-manifold preintegration paper (building on the preintegration concept of Lupton & Sukkarieh 2012); mathematically dense, essential reading.
> - [Forster et al., "IMU Preintegration on Manifold for Efficient VIO" (2015 RSS)](https://rpg.ifi.uzh.ch/docs/RSS15_Forster.pdf) — Earlier, more concise conference presentation of the method above.
> - [Shan et al., "LIO-SAM" (IROS 2020)](https://github.com/TixiaoShan/LIO-SAM) — Reference open-source implementation of tightly coupled LIO; worth reading alongside the paper.
> - [Sola et al., "A micro Lie theory for state estimation in robotics" (arXiv:1812.01537)](https://arxiv.org/abs/1812.01537) — Practical primer on Lie groups and Lie algebras; ideal preparation before studying preintegration.
> - Source code of GTSAM's `PreintegratedImuMeasurements` class — Shows exactly how the preintegration equations map into production C++ code.
> - [IMU Preintegration MATLAB implementation](https://github.com/GentleDell/imu_preintegration_matlab) — Clean MATLAB code tested on KITTI for cross-referencing equations against implementation.

### 14.11 Advanced: Observability Analysis

Anyone running VIO or SLAM on hardware eventually hits puzzling failure modes: drift spikes unexpectedly along a straight corridor, or the pose drifts slowly even when the robot is sitting completely still. Many of these symptoms reflect fundamental **observability limits** rather than coding bugs.

In an influential analysis, Hesch et al. (2014) examined an ideal linearized VINS model undergoing arbitrary motion while observing a point feature and showed that four global degrees of freedom are strictly **unobservable**:

1. **Global position (3 DoF)** — Without an external reference such as GPS, no combination of onboard camera and IMU measurements can recover absolute world coordinates; the starting pose simply defines the origin.
2. **Global yaw (1 DoF)** — Gravity fixes roll and pitch relative to the vertical axis, yet rotation about gravity (heading) remains unobservable without a compass or known landmark frame.

While those four gauge directions are always unobservable under the stated model, whether roll/pitch, metric scale, IMU biases, and extrinsics can be recovered depends on sensor configuration and trajectory excitation. A stereo rig supplies scale from its calibrated baseline, whereas a monocular VIO system requires accelerated motion to excite the IMU and render metric scale observable.

**Degenerate motion** creates additional unobservable directions on top of those four baseline gauge freedoms:

- **Pure rotation** produces zero translational parallax, preventing monocular triangulation of depth or translation (though rotation estimation and tracking survival depend on the front-end).
- **Insufficient motion during initialization**—such as moving at constant velocity or remaining stationary—leaves multiple solutions for IMU biases indistinguishable, weakening scale, gravity alignment, and bias convergence depending on the exact state and measurement model.

**Inconsistency in standard EKFs**: When a standard EKF is applied to VIO, evaluating Jacobians at successively changing state estimates breaks the theoretical unobservable subspace. As a result, covariance shrinks spuriously along unobservable directions—creating false confidence that drives estimator inconsistency and divergence.

**OC-EKF (Observability-Constrained EKF)** fixes this defect by constraining the EKF's transition and measurement Jacobians so they preserve the true unobservable null space, forcing the filter to remain honest about what it cannot observe.

In practice, these observability properties translate into three operational checks:
- Excite both rotation and translation sufficiently during VIO initialization, and verify convergence indicators for scale, gravity, and biases before trusting odometry.
- Remember that loop closure reduces relative drift between revisited segments, but cannot by itself anchor absolute global position or yaw.
- Because monocular images never fix metric scale alone, confirm that inertial excitation during startup is rich enough to make scale observable.

> **Further reading**
> - [Hesch et al., "Consistency Analysis and Improvement of Vision-aided Inertial Navigation" (TRO 2014)](https://ieeexplore.ieee.org/document/6605544) — The foundational OC-EKF / OC-VINS paper.
> - [Hong & Lim, "Visual-Inertial Odometry with Robust Initialization and Online Scale Estimation" (Sensors 2018)](https://pmc.ncbi.nlm.nih.gov/articles/PMC6308559/) — Practical study addressing initialization degradation during constant-velocity and stationary segments.
> - Barfoot, "State Estimation for Robotics" Ch.9 — Theoretical treatment of observability analysis.
> - [Huang & Dissanayake, "A critique of current developments in Simultaneous Localization and Mapping" (IJRR 2016)](https://journals.sagepub.com/doi/10.1177/0278364916643566) — Critical survey of observability and consistency issues in SLAM.

> **Practice**: [Odometry Uncertainty Visualization](https://alexjunholee.github.io/robotics-practice/app.html#odom_uncertainty)
> Interactively observe how odometry uncertainty compounds over time and watch the pose covariance ellipse grow.

#### 14.11.1 Filter-based vs Optimization-based: Which Is Better?

The relative merits of filtering versus optimization have been debated throughout the history of SLAM and VIO. Mathematically, Gauss-Newton optimization and an iterated EKF (IEKF) solve the exact same MAP objective in different algebraic forms.

In practice, the distinction comes down to state retention and relinearization:
- **Filters (EKF, MSCKF)** update the state and covariance incrementally as each measurement arrives. Simple EKF odometry retains only the current state, whereas MSCKF keeps a bounded window of past camera poses; bounding retained history typically lowers memory usage relative to full smoothing.
- **Optimizers (BA, factor graphs)** retain historical states (or a sliding window / Bayes tree in iSAM2) and optimize them jointly, allowing past linearization points to be refined for higher accuracy at greater computational cost.

Performance gaps between systems like VINS-Mono (optimization) and MSCKF (filtering) therefore stem primarily from **system architecture**—which states and measurements are kept—alongside the ability of optimizers to relinearize earlier states. In practice, IMU-centric platforms with tight compute budgets often favor filters (MSCKF or the IEKF in FAST-LIO2), camera-centric systems prioritizing accuracy favor sliding-window optimization (VINS-Mono, ORB-SLAM3), and factor-graph hybrids such as LIO-SAM combine both worlds by embedding IMU preintegration directly inside an incremental optimizer (see [Giseop Kim's blog — Gauss-Newton Opt == IEKF update?](https://gisbi-kim.github.io/blog/2022/03/05/gn-iekf-same.html)).

### 14.12 Advanced: Semantic SLAM

Classical SLAM constructs purely geometric maps—point clouds, meshes, or occupancy grids that record where surfaces lie without knowing whether a cluster of points is a wall, a door, or a bookshelf. **Semantic SLAM** enriches geometric mapping with object categories and high-level concepts so robots can reason about *what* occupies the space.

Systems differ primarily in how semantic information attaches to the map:
- **Object-level SLAM** (such as **CubeSLAM** and **QuadricSLAM**) represents landmarks as 3D cuboids or dual quadrics rather than anonymous points. Although it relies on an object detector, object-level data association is far more distinctive across viewpoints and supports reasoning at the level of whole objects.
- **Panoptic SLAM** fuses per-pixel panoptic segmentation predictions into 3D, allowing queries such as "how many chairs are in this room" directly against the reconstructed map.
- **Open-vocabulary SLAM** (such as **ConceptGraphs**) stores CLIP-style vision-language embeddings inside the map so arbitrary objects and places can be retrieved by natural language—connecting directly to the 3D scene graphs introduced in Chapter 13.

Semantic labels also harden tracking in **dynamic environments**: by identifying and masking out keypoints belonging to transient classes such as people, cars, or animals, pipelines like **DynaSLAM** (ORB-SLAM2 paired with Mask R-CNN) and **DS-SLAM** restrict pose estimation and mapping to static structure alone:

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
> - [Nicholson et al., "QuadricSLAM: Dual Quadrics from Object Detections as Landmarks in Object-Oriented SLAM" (RA-L 2019)](https://arxiv.org/abs/1804.04011) — Representative paper on object-level SLAM.
> - [ConceptGraphs (arXiv:2309.16650)](https://arxiv.org/abs/2309.16650) — Open-vocabulary 3D scene graphs; pairs well with Chapter 13.
> - [Bescos et al., "DynaSLAM: Tracking, Mapping and Inpainting in Dynamic Scenes" (RA-L 2018)](https://arxiv.org/abs/1806.05620) — Visual SLAM in dynamic environments.

### 14.13 Advanced: Multi-Robot SLAM

Mapping a large campus or warehouse with a single robot takes time. Deploying multiple robots in parallel speeds up coverage, but merging their independent submaps into a single globally consistent coordinate frame poses significant communication and consistency challenges.

In a **centralized** architecture, each robot streams sensor data or local submaps to a central server for joint optimization. While a central server simplifies global data association, bandwidth limits and server compute create bottlenecks, and the server becomes a single point of failure—nor does centralization alone guarantee a global optimum for non-convex SLAM objectives.

In a **distributed** architecture, each robot runs local SLAM onboard and exchanges compact descriptors or submaps only during rendezvous or inter-robot loop closures. Transmitting descriptors rather than raw point clouds saves bandwidth, though dropping raw evidence can make geometric verification harder; exactly which variables are shared and when asynchronous optimization converges depend on the distributed solver.

Any distributed multi-robot pipeline must solve **inter-robot loop closure**, **coordinate-frame alignment**, and **outlier rejection** jointly. A verified inter-robot match may arrive as a full 6-DoF `SE(3)` constraint, or be estimated from 3D point correspondences (requiring at least three non-collinear points and sufficient inliers). Frameworks such as Pairwise Consistent Measurement Set Maximization (PCM), Graduated Non-Convexity (GNC), Distributed Gauss-Seidel (DGS), and Alternating Direction Method of Multipliers (ADMM) trade off communication overhead, outlier resilience, and convergence guarantees differently.

The table below highlights three representative open-source multi-robot SLAM systems.

| System | Characteristics |
|---|---|
| **Kimera-Multi** | Distributed, 3D mesh + semantic, Kimera based |
| **DOOR-SLAM** | Distributed, outlier-robust, DGS optimization |
| **Swarm-SLAM** | ROS2 based, supports diverse sensors, lightweight |

> **Further reading**
> - [Lajoie et al., "DOOR-SLAM: Distributed, Online, and Outlier Resilient SLAM for Robotic Teams" (RA-L 2020)](https://arxiv.org/abs/1909.12198) — Distributed SLAM with outlier-resilient back-end optimization.
> - [Tian et al., "Kimera-Multi: Robust, Distributed, Dense Metric-Semantic SLAM for Multi-Robot Systems" (T-RO 2022)](https://arxiv.org/abs/2106.14386) — Distributed metric-semantic multi-robot SLAM.
> - [Cieslewski et al., "Data-Efficient Decentralized Visual SLAM" (ICRA 2018)](https://arxiv.org/abs/1710.05772) — Foundational work on communication-efficient decentralized visual SLAM.

### 14.14 Advanced: Place Recognition

Loop closure begins with a retrieval problem: *"Have I seen this place before?"* Given the current sensor observation, place recognition searches past keyframes for the closest match. Because a single false-positive loop closure can corrupt an otherwise accurate trajectory, loop-closure reliability hinges directly on place-recognition precision and geometric verification.

**Classical approach: Bag of Visual Words (BoVW)**

Used in ORB-SLAM2/3 via the **DBoW2** library, BoVW adapts text-retrieval inverted indices to image matching:
1. Extract local keypoints and descriptors (such as ORB) across a large training corpus.
2. Quantize those descriptors into a discrete visual vocabulary using k-means clustering.
3. Summarize each incoming image as a normalized histogram (BoW vector) counting how often each visual word appears.
4. Score candidate matches by comparing BoW histograms (for example via L1 distance).

BoVW lookup is fast via inverted indexing and thoroughly battle-tested, though it struggles under severe viewpoint shifts or lighting changes and depends on a pre-trained vocabulary.

**Learning-based approach: global descriptors**

Learned place recognition encodes an entire image directly into a single compact vector. **NetVLAD** (2016) combined CNN feature maps with a differentiable VLAD pooling layer and outperformed classical baselines on city-scale benchmarks. Later works such as **CosPlace** (2022) and **MixVPR** (2023) refined training objectives and feature aggregation across their respective datasets, while **AnyLoc** (2023) leveraged frozen DINOv2 foundation features to perform place recognition without task-specific fine-tuning across indoor, outdoor, and aerial benchmarks. None of these methods automatically beats BoVW in every environment; recall and false-positive rates still need measurement on the target deployment domain.

**LiDAR-based place recognition**

Matching 3D point-cloud structure avoids sensitivity to ambient illumination, though snow, rain, dynamic traffic, and repetitive geometry such as identical long corridors can still trigger aliasing. **Scan Context** (IROS 2018) projects each 3D scan into a polar bird's-eye-view matrix encoding maximum height and supports fast rotation-invariant matching, whereas **OverlapTransformer** (2022) learns rotation-invariant global descriptors directly from LiDAR range images using a Transformer.

**Cross-modal and sequence matching**

Multi-robot teams or heterogeneous mapping pipelines increasingly require **cross-modal place recognition**—querying a LiDAR map with a camera image or vice versa. Meanwhile, when individual frames are ambiguous across day/night or seasonal transitions, **sequence matching** compares short temporal windows instead of isolated snapshots. **SeqSLAM** (2012) demonstrated that matching sequences of frames remains reliable even when single-frame similarity collapses, and newer methods learn compact sequence descriptors to accelerate retrieval.

In practice, DBoW2 remains a clean public baseline for standard visual SLAM; under major lighting or seasonal shifts, benchmark learned descriptors from NetVLAD onward on target data, weighing AnyLoc's zero-shot convenience against backbone latency, descriptor memory, and domain recall. Note also that the cycle-posterior method covered later in §14.16.5 detects a similar place-consistency event to trigger backward trajectory correction—a helpful functional analogy rather than a claim of direct algorithmic lineage.

> **Further reading**
> - [Arandjelovic et al., "NetVLAD: CNN architecture for weakly supervised place recognition" (arXiv:1511.07247)](https://arxiv.org/abs/1511.07247) — Landmark paper in learned visual place recognition.
> - [Keetha et al., "AnyLoc: Towards Universal Visual Place Recognition" (arXiv:2308.00688)](https://arxiv.org/abs/2308.00688) — Zero-shot place recognition built on visual foundation models.
> - [Kim & Kim, "Scan Context: Egocentric Spatial Descriptor for Place Recognition within 3D Point Cloud Map" (IROS 2018)](https://ieeexplore.ieee.org/document/8593953) — Widely adopted descriptor for 3D LiDAR place recognition.
> - [Giseop Kim's blog — Scan Context-based LiDAR Pose-graph SLAM implementation](https://gisbi-kim.github.io/blog/2021/05/17/sclidarslam.html) — Practical walk-through of integrating Scan Context into LiDAR SLAM.
> - [Dark Programmer — Bag of Words technique](https://darkpgmr.tistory.com/125) — Explains BoW fundamentals through the lens of image retrieval.

> **Technical Timeline: SLAM & Odometry**
> - **~2007**: Classical filtering dominated via EKF-SLAM and particle-filter FastSLAM. MonoSLAM (2007) brought real-time monocular SLAM to desktop CPUs, while PTAM (2007) introduced the parallel tracking-and-mapping split.
> - **2010–2015**: Direct photometric pipelines such as LSD-SLAM and SVO emerged. LOAM (2014) established edge/plane LiDAR odometry and mapping, and ORB-SLAM (2015) set a lasting open-source baseline for feature-based visual SLAM.
> - **2015–2020**: Open visual-inertial estimators matured around VINS-Mono and the MSCKF family, DSO formalized direct sparse odometry, and tightly coupled LiDAR-inertial graph systems such as LIO-SAM appeared.
> - **2020–2023**: Production and research systems—including FAST-LIO/FAST-LIO2, ORB-SLAM3, DROID-SLAM, and R3LIVE—combined iterated filtering, incremental smoothing, and learned front-ends in diverse architectures.
> - **2024–**: 3DGS-based SLAM (SplaTAM, MonoGS, Gaussian-SLAM) is reshaping neural mapping, alongside early systems pairing foundation models with SLAM for natural-language spatial queries.
> - **Recent direction**: Mature geometric systems such as ORB-SLAM3, LIO-SAM, and FAST-LIO2 anchor public benchmarks, while 3DGS-SLAM and learned front-ends broaden scene representations. Standard benchmarks including KITTI, EuRoC, and TUM RGB-D remain essential for testing assumptions and failure modes head-to-head.

### 14.15 Advanced: Long-term Mapping

A robot operating continuously in a real building or city cannot simply map its environment once and freeze that map forever. Layouts evolve, parked cars come and go, pedestrians walk across scans, and multiple mapping sessions must merge seamlessly over weeks or months. **Long-term mapping** extends SLAM to stay accurate, clean, and bounded across lifelong deployments.

#### 14.15.1 Incremental Smoothing: from iSAM to iSAM2

Classical EKF-SLAM slows down as state covariance grows, while naive batch nonlinear optimization recomputes the entire matrix factorization on every new keyframe. **iSAM** (Kaess et al., TRO 2008) avoided full recomputation by updating the QR factor `R` incrementally via Givens rotations whenever new measurements arrived—though accumulating fill-in still forced periodic global variable reordering.

**iSAM2** (Kaess et al., IJRR 2012) eliminated that bottleneck by organizing elimination as a **Bayes tree**, re-eliminating and relinearizing only the local subtree touched by new factors. That incremental tree engine is what powers GTSAM across large-scale SLAM back-ends today.

#### 14.15.2 Dynamic Object Removal

Leaving ghost trails of walking pedestrians or moving cars inside a static map degrades later localization and path planning. Cleaning dynamic artifacts from point-cloud maps is therefore essential:

- **Removert** (Kim et al., 2020) projects scans and maps into multi-resolution range images to flag discrepancies across viewpoints. Rather than carving out everything suspicious in one pass, its two-stage "remove, then revert" strategy aggressively prunes dynamic points first and then restores mistakenly removed static structure across confidence scales. Compared with voxel ray-casting (accurate but costly), naive visibility checks (which assume background points always survive), and pure semantic segmentation (blind to unseen classes and scan-to-map geometry), multi-resolution range-image comparison strikes a practical balance.
- **SuMa++** (Chen et al., IROS 2019) enriches surfel maps with surface normals and semantic labels, suppressing surfels flagged as dynamic by both semantics and motion—while preserving moving-object constraints carefully when static geometry alone is degenerate.

#### 14.15.3 Multi-Session SLAM

Combining maps collected across different days introduces **gauge freedom**: each session starts in its own arbitrary coordinate origin, so raw trajectories cannot simply be overlaid.

- **LT-mapper** (Kim et al., 2021) aligns separate sessions using Scan Context anchor nodes and tracks map evolution through a delta map, separating transient high-dynamic clutter from low-dynamic structural changes—both newly added obstacles (positive differences) and removed objects (negative differences).
- **Continuous-Time Estimation** (Furgale et al., ICRA 2012) replaces discrete pose timestamps with smooth B-spline basis functions, allowing asynchronous sensors at vastly different rates (such as high-rate IMUs, rolling-shutter cameras, and spinning LiDARs) to query states continuously with fewer variables and perform joint spatiotemporal self-calibration.

> **Further reading**
> - [Kaess et al., "iSAM2: Incremental Smoothing and Mapping Using the Bayes Tree" (IJRR 2012)](https://www.cs.cmu.edu/~kaess/pub/Kaess12ijrr.pdf) — Original Bayes-tree incremental smoothing paper.
> - [Kim et al., "Remove, then Revert: Static Point Cloud Map Construction using Multiresolution Range Images" (IROS 2020)](https://github.com/irapkaist/removert) — Practical dynamic-point removal algorithm and code.
> - [Kim et al., "LT-mapper: A Modular Framework for LiDAR-based Lifelong Mapping" (ICRA 2022)](https://github.com/gisbi-kim/lt-mapper) — Multi-session lifelong LiDAR mapping framework.
> - [Chen et al., "SuMa++: Efficient LiDAR-based Semantic SLAM" (IROS 2019)](https://github.com/PRBonn/semantic_suma) — Semantic surfel-based LiDAR SLAM.

---

### 14.16 Advanced: History of Information-Form SLAM

*Cross-reference with §14.9 factor graph optimization (bidirectional).*

When *Probabilistic Robotics* appeared in 2005, several competing formulations of SLAM uncertainty vied for dominance: EKF-SLAM maintained a joint state and dense covariance, EIF and SEIF exploited information-form additivity, and EM mapping treated unknown data association statistically. Modern factor graphs and incremental smoothers such as GTSAM and iSAM2 later brought additivity, variable elimination, and localized updates into a unified nonlinear optimization framework. Tracing this earlier generation clarifies *why* modern SLAM back-ends are structured the way they are.

#### 14.16.1 EKF-SLAM (PR §10)

EKF-SLAM traces back to **Smith, Self, and Cheeseman (1986/1990)**, whose "stochastic map" stacked robot pose and landmarks into a single joint state vector. Leonard and Durrant-Whyte in the 1990s, followed by Dissanayake et al. (2001, IEEE T-RA), completed its mathematical formalization.

To formulate the filter, the 2D pose $x_t = (x, y, \theta)$ and $N$ landmarks $(m_{j,x}, m_{j,y}, s_j)$ are stacked into a $(3N+3)$-dimensional joint state vector $y_t$, and a single EKF propagates both together:

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

Because **the Kalman gain $K_t^i$ is a $(3N+3) \times 3$ matrix**, observing even a single landmark updates the entire joint state vector. Off-diagonal covariance entries propagate that correction to every correlated landmark, causing each update step to scale as $O(N^2)$.

When correspondences are unknown, a provisional $(N_t+1)$-th landmark is appended temporarily to the map, Mahalanobis distances are evaluated against all existing candidates, and the maximum-likelihood (ML) match is selected; if the distance exceeds threshold $\alpha$, the observation initializes a new landmark. Because greedy ML decisions are irrevocable, a single false association permanently corrupts the covariance structure.

Over larger trajectories, EKF-SLAM hits three structural limits: the dense covariance matrix $\Sigma \in \mathbb{R}^{(3N+3) \times (3N+3)}$ demands $O(N^2)$ memory (303×303 for 100 landmarks, 3003×3003 for 1,000); accumulating linearization errors ruin filter consistency (Bailey et al. 2006); and because past poses are marginalized out online, they can never be relinearized globally.

For large landmark maps, dense covariance and linearization drift have led modern pipelines toward smoothing and factor graphs. Although no universal landmark-count cutoff invalidates EKF-SLAM, it survives mostly in small fiducial-mapping or tightly constrained compute regimes. Sliding-window filters such as MSCKF retain EKF linearization and covariance propagation without keeping landmarks permanently in the state; joint compatibility branch and bound (JCBB) improves on greedy ML association; and GTSAM/iSAM2 solve SLAM via smoothing and factor graphs as a distinct formulation rather than a direct algorithmic descendant of EKF-SLAM.

#### 14.16.2 GraphSLAM — Information-Form Batch SLAM (PR §11)

While EKF-SLAM's covariance matrix $\Sigma$ couples every pair of variables densely, its inverse—the **information matrix** $\Omega = \Sigma^{-1}$—exposes the natural graph sparsity of SLAM because independent measurement factors enter by local addition. (Motion updates and variable marginalization can still create fill-in, so sparsity is not automatic.) That structural insight motivates both EIF and sparse graph formulations.

##### The intuition of information form: spring-mass analogy

Information-form SLAM replaces $(\mu, \Sigma)$ with the information matrix $\Omega = \Sigma^{-1}$ and information vector $\xi = \Omega \mu$, exploiting the fact that **information is additive**. Intuitively, variables (poses and landmarks) act like point masses, and non-zero off-diagonal blocks in $\Omega$ act like elastic springs tying coupled nodes together:
- A control input $u_t$ inserts a spring between $x_{t-1}$ and $x_t$ with stiffness $R_t^{-1}$ (stiffer coupling for quieter odometry).
- A landmark measurement $z_t^i$ inserts a spring between pose $x_t$ and landmark $m_j$ with stiffness $Q_t^{-1}$.
- Two landmarks that have never been co-observed share no direct spring.

Each measurement updates the information pair purely by **local addition**:

$$\Omega \leftarrow \Omega + H_t^{iT} Q_t^{-1} H_t^i, \qquad \xi \leftarrow \xi + H_t^{iT} Q_t^{-1}[z_t^i - h(\mu_t) + H_t^i \mu_t]$$

While measurements add locally, motion updates and marginalization still require elimination steps that can introduce fill-in. Note that factor graphs share a similar spring-network intuition because each factor links a small subset of variables, even though factor graphs have a separate origin and do not begin with SEIF.

##### Four-step pipeline

GraphSLAM applies this information-form view offline to the full batch posterior $p(x_{0:t}, m | z_{1:t}, u_{1:t})$, unlike EIF and SEIF, which run as online filters that marginalize out past robot poses:

```
GraphSLAM_known_correspondence(u_{1:t}, z_{1:t}, c_{1:t}):
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

Here $\tau(j)$ denotes the set of time steps at which landmark $j$ was observed. Geometrically, the Reduce step replaces each landmark node with new springs directly linking all poses in $\tau(j)$—algebraically identical to the **block-diagonal Schur complement** used in modern bundle adjustment (§14.9.2).

More generally, the **Marginalization Lemma** expresses marginalization in any linear Gaussian information form as a Schur complement, which can introduce fill-in among the remaining variables:

$$\bar\Omega_{xx} = \Omega_{xx} - \Omega_{xy} \Omega_{yy}^{-1} \Omega_{yx}$$

GraphSLAM (Thrun and Montemerlo, 2006, IJRR) thus optimizes the full trajectory and map in batch as a sparse information graph—building on earlier global pose-relation optimization by Lu and Milios (1997) and sharing information additivity with EIF, while remaining distinct from online filtering. When correspondences are unknown, GraphSLAM tests whether candidate feature pairs $(m_j, m_k)$ refer to the same landmark, inserts accepted constraints, and re-optimizes (removing explicit constraints before re-solving if needed—a discrete check distinct from switchable constraints by Sünderhauf & Protzel 2012, which jointly optimize continuous activation variables). Comparing GraphSLAM's `initialize → construct → eliminate → solve` cycle with modern iSAM/iSAM2 reveals both shared algebraic structure and a deeper advance: iSAM2 updates only affected subtrees in a Bayes tree rather than re-eliminating the entire graph in batch.

#### 14.16.3 SEIF — Sparse Extended Information Filter (PR §12)

Whereas GraphSLAM operates in batch, an online EIF suffers from dense fill-in during exact motion updates. The **Sparse Extended Information Filter (SEIF)** bounds the set of "active" features and applies a **sparsification** step to pursue constant-time online updates independent of total map size. In their 3.5 km Victoria Park benchmark, Thrun et al. (2004, IJRR) reported accuracy comparable to EKF-SLAM at roughly half the runtime and one-quarter the memory.

##### Four-step update

Each step of SEIF alternates local information updates with periodic sparsification and amortized state recovery:

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

Sparsification is SEIF's defining approximation: it severs the direct link between variables $a$ and $b$ by approximating the joint distribution as a product of marginals conditioned on $c$, forcing $\Omega_{a,b} = 0$:

$$\tilde p(a,b,c) = \frac{p(a,c)\, p(b,c)}{p(c)} \quad \Longrightarrow \quad \Omega_{a,b} = 0$$

This step corresponds to a KL-divergence projection enforcing conditional independence $a \perp b | c$. Because repeated approximation and relinearization can still erode filter consistency over long trajectories, sparsification cannot be treated as a blanket guarantee against overconfidence.

Keeping a fixed cap of $K$ active features bounds the local update matrix to size $(2K+3) \times (2K+3)$, delivering $O(1)$ motion and measurement updates with respect to map size (though full-map mean recovery and global data association are not automatically $O(1)$). *Probabilistic Robotics* illustrates SEIF with roughly six active features, though the safe number depends on sensor geometry and map density. Later work by Eustice et al. (2006), "Exactly Sparse EIF" (ESEIF), eliminates the sparsification approximation under specific structural conditions (see also PR Figure 12.3 for how graph links evolve across motion, measurement, and sparsification).

##### Tree-based data association

Information-form additivity also enables reversible data association: **soft correspondence constraints can be added or subtracted cleanly**. Asserting that features $m_i$ and $m_j$ represent the same landmark adds the quadratic term

$$\Omega \leftarrow \Omega + F_{m_i - m_j}^T C\, F_{m_i - m_j}$$

which can be subtracted back out if kept as an explicit factor. This reversibility supports A*-style frontier search over a data-association tree (albeit with exponential worst-case branching). Later robust back-ends such as switchable constraints (Sünderhauf & Protzel 2012) and Max-mixtures (Olson & Agarwal 2013) share the goal of tolerating bad loop closures, but operate via continuous switch variables and mixture factors rather than tree search.

##### Multi-robot map fusion

In multi-robot mapping, information additivity lets two robots merge maps after aligning into a shared coordinate frame—provided they do not share unmodeled priors or duplicate observations:

$$\Omega^{\text{fused}} = \Omega^{j \leftarrow k\text{-aligned}} + \Omega^k, \qquad \xi^{\text{fused}} = \xi^{j \leftarrow k\text{-aligned}} + \xi^k$$

Summing correlated estimates naively double-counts shared evidence and produces overconfident covariances (covariance intersection avoids this in covariance form, while Nettleton et al. 2003, DDF-SAM, Kimera-Multi, and Swarm-SLAM address distributed fusion under distinct communication and bookkeeping assumptions; see §14.13).

Taken together, SEIF and iSAM2 target very different regimes: approximate online filtering versus incremental nonlinear smoothing (where iSAM2's accuracy also hinges on relinearization policies). ESEIF sits squarely in the SEIF family, whereas sliding-window marginalization in VINS-Mono/OKVIS and feature null-space projection in MSCKF represent separate strategies for bounding state size—united by the shared challenge of managing sparsity and compute rather than direct algorithmic inheritance (see §14.9.2).

#### 14.16.4 EM Mapping

Whereas EKF-SLAM, EIF, and SEIF either assumed known correspondences or committed greedily to ML matches, an earlier lineage—including MHT, JPDA, and particle-based FastSLAM—kept data association probabilistic. **EM Mapping** (Thrun, Burgard, and Fox, 1998–2000, AAAI/JAIR; deployed in variant form on the RHINO museum guide robot, Burgard et al. 1999) **treats unknown correspondences as latent variables inside an Expectation-Maximization loop**, extracting information from ambiguous scans instead of discarding them.

##### E-step / M-step skeleton

Structurally identical to the Baum-Welch algorithm for hidden Markov models, EM Mapping alternates between computing smoothed pose posteriors given the current map (E-step) and updating cell occupancy frequencies given those trajectories (M-step):

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

Here $\alpha$ propagates belief forward (Markov localization) while $\beta$ propagates future observations backward; when a loop closes at time $T$, that $\beta$ term retroactively sharpens earlier pose estimates—the statistical core of EM mapping. The M-step simply counts belief-weighted frequencies for each cell property $l$, typically converging within 3–5 iterations.

##### Layered EM Mapping

Because the basic M-step evaluates cells independently and breaks geometric consistency inside each sensor cone, **Layered EM Mapping** first builds a **local occupancy grid** from each short motion segment and then uses EM solely to align the *poses* of those local submaps, adding **deterministic annealing** ($\sigma: 1.0 \to 0$) to escape shallow local optima:

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

Gradually sharpening the objective via deterministic annealing reflects the same continuation principle seen later in graduated non-convexity (GNC; Yang et al. 2020) and robust-kernel scheduling.

##### Why EM Mapping Left the Mainstream

Neither EM_mapping nor layered_EM_mapping remains a core component of modern real-time SLAM. Once sparse factor-graph optimization matured, optimizing poses jointly proved far faster and cleaner than alternating E/M passes over full batch trajectories. Modern systems take very different routes: Cartographer (Hess et al. 2016) combines scan matching directly with pose-graph optimization without EM, while GMapping (Grisetti et al. 2007) uses a Rao-Blackwellized particle filter whose proposal distribution is sharpened by scan matching, without a pose graph at all. Even so, Layered EM's **submap + global alignment** motif—building locally consistent submaps before optimizing their global relative poses—echoes structurally in Cartographer's local/global split, even without direct historical descent.

#### 14.16.5 Cycle Posterior

Incremental ML mapping fails when odometry drift grows large or when a loop closes, because it has no mechanism to push corrections backward along past poses. The **Cycle Posterior** method remedies both flaws by running a global pose posterior estimator alongside the incremental ML mapper:

```
Incremental Mapping with Posterior Estimation:
  1. incremental_ML_mapping(o, a, s, m)  → ⟨m', s'⟩    [ML update]
  2. Bel(s') = P(o,s') ∫ P(s'|a,s) Bel(s) ds            [posterior one step]
  3. s'' = argmax Bel(s')                                 [posterior mode]
  4. s'' ≠ s'  →  cycle closure detected
                   distribute s'' − s' linearly along the cycle path
  5. run incremental_ML_mapping backwards in time          [nested ML refinement]
```

When a previously ambiguous multi-modal posterior suddenly collapses onto a sharp mode $s''$ that disagrees with the ML pose $s'$, the discrepancy triggers a cycle-closure correction distributed linearly backward along the loop—even without odometry, naturally implemented using MCL for the parallel posterior tracker.

While this specific pairing of MCL, linear error distribution, and nested backward ML refinement is rarely used today, its **two-stage division of labor**—a global detector flagging a revisited place, followed by a back-end corrector propagating the constraint through past states—parallels modern place recognition plus incremental graph optimization (such as iSAM, iSAM2, and GTSAM), serving as an instructive early precedent rather than a direct ancestor.

#### 14.16.6 Summary: What Survived

The classical algorithms in *Probabilistic Robotics* (2005) introduced structural principles that still resonate across modern SLAM and localization stacks:

| PR algorithm | Core contribution | Comparable modern structure | Scope of the relationship |
|---|---|---|---|
| EKF-SLAM | unified landmark state, off-diagonal covariance | MSCKF-family filters, fiducial mapping | shares EKF machinery but not the same state design |
| GraphSLAM | additive information factors, variable elimination, full trajectory | GTSAM, g2o, Ceres, iSAM2 | sparse least squares and smoothing viewpoints remain relevant |
| SEIF | bounded active set, sparsification | ESEIF, bounded-state estimators | distinguish the SEIF family from other sparse estimators |
| EM Mapping | latent association, forward-backward localization, submaps | EM-based mapping, submap systems | a statistical or structural comparison, not direct lineage |
| Cycle Posterior | combined online closure detection and correction | place recognition + graph optimization | similar functional separation |

Information-form **additivity** maps directly onto the summed factor costs of modern nonlinear least squares. By contrast, SEIF **sparsification**, sliding-window marginalization, and Bayes-tree elimination all manage computational complexity through distinct mathematical operations, and the submap and detector/corrector motifs in EM Mapping and Cycle Posterior offer structural parallels rather than direct historical lineage (see §14.9).

> **Further reading**
> - [Thrun et al., "Simultaneous Localization and Mapping with Sparse Extended Information Filters" (IJRR 2004)](https://journals.sagepub.com/doi/10.1177/0278364904045026) — Original SEIF paper.
> - [Thrun & Montemerlo, "The GraphSLAM Algorithm with Applications to Large-Scale Mapping of Urban Structures" (IJRR 2006)](https://journals.sagepub.com/doi/10.1177/0278364906065390) — EIF/GraphSLAM formalization.
> - [Dissanayake et al., "A Solution to the Simultaneous Localization and Map Building (SLAM) Problem" (IEEE T-RA 2001)](https://ieeexplore.ieee.org/document/938381) — Classic EKF-SLAM formalization.
> - [Kaess et al., "iSAM2: Incremental Smoothing and Mapping Using the Bayes Tree" (IJRR 2012)](https://www.cs.cmu.edu/~kaess/pub/Kaess12ijrr.pdf) — Original paper on incremental smoothing and the Bayes tree.

---

Across this chapter, no single SLAM or localization architecture solves every operating condition. Service-robot localization hinges on choosing the right belief representation, pose-graph optimization must still be paired with a dedicated map-generation step such as occupancy grid mapping, factor graphs dominate large-scale smoothing, and classical EKF localization remains practical for ceiling-marker systems and compute-constrained platforms. In deployed robots, none of these estimators runs in isolation: sensor drivers, SLAM nodes, localization filters, planners, and controllers must execute concurrently while exchanging time-synchronized messages and coordinate transforms—the role of robot middleware frameworks such as ROS, covered next in Ch.15.
