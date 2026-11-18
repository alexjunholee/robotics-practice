# Ch.2 — Sensors

In Ch.1, we saw that physical-world uncertainty separates robotics from pure software. Every perception and estimation algorithm begins with sensors—the hardware interface between the robot and its surroundings—so selecting the right sensor suite and designing algorithms that tolerate real-world noise both depend on understanding how each sensor actually works.

When a SLAM pipeline loses tracking in the field, diagnosing whether the failure started with rolling-shutter distortion, a low-reflectance LiDAR surface, or uncalibrated IMU bias requires knowing each sensor's physical measurement principle and error model. Those characteristics define the exact data stream—and failure modes—that every downstream estimation algorithm receives.

## 2.1 Camera

Cameras capture color and visual texture at high spatial resolution, making them indispensable for object recognition, semantic understanding, and visual SLAM. Yet "camera" covers several distinct sensing architectures—monocular, stereo, RGB-D, and event cameras—that trade off direct depth measurement, temporal resolution, power, and sensitivity to lighting.

### 2.1.1 Monocular Camera

A monocular camera is the simplest visual sensor, using a single lens and image sensor to project a 3D scene onto a 2D pixel grid. Because it captures rich color and texture at low cost and weight, it is a workhorse for visual SLAM, object detection, and semantic scene understanding.

However, projecting 3D points onto a single 2D image collapses depth along each viewing ray: a small toy nearby and a large building far away can occupy the exact same pixels. That is why a single frame cannot recover metric depth on its own, and why monocular depth estimation or Structure from Motion (SfM) must infer scene geometry from motion parallax or learned visual cues—leaving absolute scale ambiguous unless resolved by stereo baselines, depth sensors, or an IMU.

**Pros**:
- Inexpensive, compact, and lightweight
- Rich color and texture detail
- High spatial resolution

**Cons**:
- Cannot measure metric depth directly from a single image
- Scale ambiguity: real-world object size and trajectory scale cannot be determined without external cues

**Key specifications**:
- Resolution: 720p, 1080p, 4K, etc.
- Frame rate: 30fps, 60fps, 120fps, etc.
- Field of View (FoV): narrow FoV vs. wide FoV (fisheye)
- Global shutter (exposes all pixels simultaneously) vs. rolling shutter (scans line by line, causing geometric skew during fast motion)

Common hardware spans consumer webcams, industrial machine-vision cameras, and embedded modules:

```
Typical camera sensors:
- Webcams: Logitech C920, C930e
- Industrial: FLIR (Point Grey), Basler, Allied Vision
- Embedded: Raspberry Pi Camera, OAK-D
```

> **Further reading**
> - [First Principles of Computer Vision — Camera Calibration / Uncalibrated Stereo](https://www.youtube.com/playlist?list=PL2zRqk16wsdoCCLpou-dGo7QQNks1Ppzo) — Columbia University Prof. Shree Nayar's lectures covering the pinhole camera model, intrinsic/extrinsic calibration, and stereo geometry.
> - [OpenCV Camera Calibration Tutorial](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html) — Hands-on guide to calibrating camera intrinsics and lens distortion.

### 2.1.2 Stereo Camera

A stereo camera recovers metric depth by placing two calibrated cameras side by side at a known separation (the baseline), mimicking human binocular vision. By finding the same physical point in both the left and right images, the horizontal pixel shift between them—called **disparity**—triangulates the point's 3D distance.

Unlike active depth sensors that project infrared light, a passive stereo camera relies purely on ambient illumination. Because sunlight does not drown out a passive image the way it swamps active IR patterns or Time-of-Flight (ToF) pulses, stereo remains a primary way to obtain dense depth outdoors. Its geometry also ties directly to epipolar geometry, one of the core mathematical tools in multi-view vision.

From similar triangles in a parallel stereo pair, depth is inversely proportional to disparity:

```
Depth (Z) = (focal_length × baseline) / disparity
```

- **Disparity**: the difference in x-coordinate of the same 3D point between the left and right images
- **Baseline**: the physical distance between the optical centers of the two cameras

Because disparity sits in the denominator, depth resolution drops rapidly as objects move farther away; extending the useful range requires widening the baseline or increasing focal length and image resolution. Matching corresponding pixels across both images also assumes visible surface texture—blank walls, glass, or uniform shadows give stereo matchers nothing distinctive to lock onto.

**Pros**:
- Passive sensor (requires no active light emitter)
- Works reliably in bright outdoor environments
- Captures aligned RGB appearance and depth simultaneously

**Cons**:
- Stereo matching fails on textureless or reflective surfaces (white walls, glass)
- Higher computational cost for dense correspondence matching
- Effective range and depth precision are constrained by the physical baseline

**Representative products**:
- Intel RealSense D435/D455: pairs stereo IR cameras with an active IR pattern projector to assist matching on bland surfaces
- ZED 2: wide baseline suited to longer-range outdoor depth
- OAK-D: stereo pair with onboard edge AI inference

> **Further reading**
> - [Cyrill Stachniss — Stereo Normal Case (5 Minutes with Cyrill)](https://www.youtube.com/watch?v=OpiqzVkwNz4) — Five-minute explanation of the geometry behind parallel (normal-case) stereo cameras.
> - [Stanford CS231A — Epipolar Geometry and Stereo](https://web.stanford.edu/class/cs231a/) — Stanford's computer vision lecture notes covering epipolar geometry and stereo matching.

> **Exercise**: [Stereo Disparity visualization](https://alexjunholee.github.io/robotics-practice/app.html#stereo_disparity)
> Experiment with computing disparity from a stereo image pair and observe how baseline and focal length affect depth estimation accuracy.

### 2.1.3 RGB-D Camera

An RGB-D camera outputs a standard color image alongside a per-pixel depth map directly from the sensor, sparing the host processor from running dense stereo matching from scratch. That convenience makes RGB-D cameras the most common starting point for indoor SLAM and 3D reconstruction experiments on a lab bench.

Still, RGB-D sensors achieve direct depth through active infrared illumination, and understanding *how* they emit and decode that light explains why they struggle outdoors or interfere when multiple units face the same scene. Three main designs dominate commercial hardware:

**ToF (Time of Flight) method**:
- Emits modulated infrared light and infers distance from the round-trip travel time or phase shift of the reflected signal
- Pros: works even on textureless walls; fast real-time depth output
- Cons: ambient sunlight saturates the IR receiver; glossy or specular surfaces cause multipath errors
- Examples: Microsoft Azure Kinect, PMD Pico Flexx

**Structured light method**:
- Projects a known IR dot or stripe pattern onto the scene and computes depth from how surfaces deform that pattern
- Pros: high short-range accuracy at relatively low hardware cost
- Cons: outdoor sunlight washes out the projected pattern; overlapping patterns from multiple sensors interfere with each other
- Examples: Orbbec Astra

**Active IR stereo method**:
- Performs stereo matching between left and right infrared cameras while an IR projector adds artificial texture to featureless surfaces
- Examples: Intel RealSense D400 series

The table below summarizes the practical trade-offs between the two pure active-sensing mechanisms, ToF and structured light:

| Characteristic | ToF | Structured Light |
|------|-----|------------------|
| Outdoor use | Limited | Difficult |
| Accuracy | Medium | High |
| Range | 0.2-5m | 0.2-8m |
| Multi-sensor | Possible with synchronization (staggered exposures); interference otherwise | Interference occurs |

> **Further reading**
> - [Intel RealSense — Depth Cameras D415 & D435](https://www.youtube.com/watch?v=A4Kjvosvx5I) — Intel's overview of how its active stereo and depth cameras operate.
> - [Open3D RGB-D Reconstruction Tutorial](http://www.open3d.org/docs/release/tutorial/pipelines/rgbd_integration.html) — Hands-on tutorial for fusing RGB-D streams into 3D meshes and point clouds.

To get an Intel RealSense camera streaming on Ubuntu 22.04, install the official SDK and verify the feed with the viewer tool:

**Installing the RealSense driver (Ubuntu 22.04)**

```bash
# Install the Intel RealSense SDK
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.realsenseai.com/Debian/librealsenseai.asc | gpg --dearmor | sudo tee /etc/apt/keyrings/librealsenseai.gpg > /dev/null
echo "deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo `lsb_release -cs` main" | \
    sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update
sudo apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev

# Test
realsense-viewer
```

If you plan to ingest the RGB and depth topics inside ROS2, add the wrapper package as well:
```bash
sudo apt install ros-humble-realsense2-camera
ros2 launch realsense2_camera rs_launch.py
```

(Reference: [Jinyong Jeong's blog](https://jinyongjeong.github.io/2020/06/20/Realsense-Ubuntu-driver-%EC%84%A4%EC%B9%98/))

### 2.1.4 Event Camera

Conventional frame cameras expose every pixel simultaneously at fixed intervals (such as 30 or 60 Hz), which causes motion blur during agile flight or high-speed maneuvers and clips highlights or shadows in high-dynamic-range (HDR) lighting. An **event camera** takes a fundamentally different approach: each pixel operates independently and asynchronously emits an event only when it detects a **brightness change** (in log-intensity) exceeding a threshold.

Because pixels fire continuously as edges move rather than waiting for a global shutter window, event cameras achieve microsecond temporal resolution with virtually no motion blur, alongside a dynamic range around 140 dB. The flip side is that a completely static camera viewing a static, evenly lit scene produces zero events, and standard frame-based computer vision pipelines cannot ingest sparse event streams without specialized representations or algorithms. For projects targeting agile drones or extreme HDR scenes, the Gallego et al. survey (TPAMI 2020) and the `rpg_dvs_ros` driver are the standard starting points.

Rather than a 2D array of RGB intensities, the sensor outputs a continuous stream of 4-tuples:

```
(x, y, timestamp, polarity)
- x, y: pixel coordinates
- timestamp: time in microseconds
- polarity: brighter (+1) or darker (-1)
```

**Pros**:
- Microsecond-level temporal resolution
- Very high dynamic range (140 dB vs. roughly 60 dB for a typical frame camera)
- Low power consumption and ultra-low latency
- Immune to conventional frame-exposure motion blur

**Cons**:
- Produces no output when both the camera and scene are static under constant illumination
- Incompatible with standard frame-based computer vision algorithms without conversion or specialized architectures
- Relatively expensive compared with mainstream CMOS cameras

**Representative products**:
- Prophesee: high-resolution event sensors
- iniVation: DAVIS (combines asynchronous event pixels with standard APS frames on one chip)
- Samsung: mobile event sensor development

> **Further reading**
> - [Davide Scaramuzza — Event Cameras: a New Way of Sensing (ICCP 2024 Keynote)](https://www.youtube.com/watch?v=0wGBpgIrd9M) — Overview keynote by Prof. Scaramuzza, a pioneer in event-based robotics.
> - [Gallego et al. — Event-based Vision: A Survey (TPAMI 2020)](https://arxiv.org/abs/1904.08405) — Comprehensive survey of event-camera physics, representations, and algorithms.
> - [rpg_dvs_ros — Event Camera ROS driver](https://github.com/uzh-rpg/rpg_dvs_ros) — Open-source ROS package for interfacing with event cameras.

## 2.2 LiDAR

**LiDAR (Light Detection and Ranging)** measures distance directly by emitting laser pulses and timing their return—or, in frequency-modulated designs, measuring phase/frequency shifts. Depending on the scanning optics, a 2D LiDAR sweeps a single plane to return a planar range scan, whereas a 3D LiDAR sweeps multiple vertical beams or scan patterns to produce a metric 3D point cloud.

Whereas passive monocular cameras record rich appearance without direct scale, LiDAR directly recovers metric 3D geometry largely independent of ambient lighting. Maximum range and measurement noise still depend on the unit, target reflectivity, incidence angle, atmospheric scatter, sunlight, and return mode—some long-range automotive LiDARs specify detection beyond 100 m under stated reflectivity conditions. Active RGB-D cameras and radar also measure range directly, so LiDAR's distinguishing advantage is the combination of fine angular resolution and centimeter-level range accuracy over tens or hundreds of meters.

Hardware architecture is also shifting: solid-state LiDAR is replacing mechanically spinning units in several automotive and robotics applications because removing large rotating assemblies improves durability and manufacturability. At the same time, non-repetitive scan patterns—such as Livox's prism-based rosette trajectory—break the regular ring structure assumed by classic spinning-LiDAR algorithms, requiring point-cloud pipelines to adapt.

### 2.2.1 2D LiDAR vs. 3D LiDAR

Depending on whether the robot operates on a flat indoor floor or in full 3D outdoor environments, teams typically choose between planar 2D scanners and multi-beam 3D units:

**2D LiDAR**:
- Sweeps a single horizontal plane
- Use cases: indoor mobile robot localization, planar mapping, obstacle avoidance
- Examples: SICK TiM, Hokuyo URG, RPLIDAR

**3D LiDAR**:
- Captures full 3D point clouds via stacked laser channels or multi-axis scanning
- Use cases: autonomous driving, outdoor mapping, rough-terrain navigation
- Examples: Velodyne VLP-16 / VLP-32C / HDL-64E, Ouster OS1, Hesai

> **Further reading**
> - [Cyrill Stachniss — LiDAR-based SLAM](https://www.youtube.com/watch?v=vrdlk2p9AZI) — Lecture explaining scan matching and SLAM using LiDAR range scans.
> - [PCL (Point Cloud Library) official tutorials](https://pcl.readthedocs.io/projects/tutorials/en/latest/) — Widely used C++ library tutorials for filtering, registration, and segmentation of point clouds.

### 2.2.2 Spinning vs. Solid-State

Beyond dimensionality, how a LiDAR steers its laser beams shapes both mechanical reliability and point-cloud geometry:

**Spinning (mechanical)**:
- Internal laser emitters and receivers rotate continuously
- Delivers a full 360° horizontal field of view
- Cons: moving parts add bulk, cost, and long-term wear
- Examples: Velodyne, Ouster

**Solid-State**:
- Eliminates large rotating assemblies (flash designs have no moving parts, MEMS designs oscillate only a micro-mirror, and Livox's non-repetitive scan rotates compact internal prisms)
- Narrower field of view (typically under 120°)
- Pros: higher durability and stronger path toward low-cost mass production
- Examples: Livox (non-repetitive scan pattern), Innoviz

That scan-pattern difference directly affects algorithm design. Spinning LiDARs produce uniform 360° concentric scan rings, and classic SLAM methods such as LOAM and LeGO-LOAM rely on that ring structure to extract edge and planar features. Non-repetitive solid-state scans invalidate those assumptions, which helped motivate direct-registration LiDAR-inertial odometry pipelines such as FAST-LIO2.

### 2.2.3 Key specifications

When comparing LiDAR datasheets for a robot platform, five parameters determine whether the sensor matches the environment's scale, density, and occlusion geometry:

| Specification | Description |
| --- | --- |
| Channels | Number of vertical layers (16, 32, 64, 128) |
| Range | Maximum measurement distance (50m ~ 300m) |
| Points/sec | Points per second (300K to several million; 128-channel units exceed 2M) |
| Accuracy | Measurement accuracy (±2cm ~ ±5cm) |
| FoV | Horizontal/vertical field of view |

> **Further reading**
> - [Livox technical documents](https://www.livoxtech.com/downloads) — Technical notes explaining non-repetitive scanning geometry and point accumulation over time.
> - [Xu et al. — FAST-LIO2 (T-RO 2022)](https://arxiv.org/abs/2107.06829) — LiDAR-inertial odometry that replaces feature extraction with direct registration using an ikd-Tree, supporting both spinning and solid-state LiDARs.

## 2.3 IMU (Inertial Measurement Unit)

An **inertial measurement unit (IMU)** measures a robot's linear acceleration and angular velocity directly from internal inertial forces, typically at hundreds to thousands of hertz. Because cameras and LiDARs operate at much lower frame rates (10–60 Hz) and suffer from motion blur or scan distortion during rapid turns, IMUs bridge the gaps between frames in visual-inertial odometry (VIO) and LiDAR-inertial odometry (LIO).

When a VIO or LIO system drifts badly, diagnosing whether the root cause is an uncalibrated bias or an IMU grade mismatched to the trajectory requires understanding what the sensor actually measures and why integration amplifies small errors.

### 2.3.1 Components

A standard 6-axis IMU combines an accelerometer and a gyroscope; 9-axis modules add a magnetometer:

**Accelerometer**:
- Measures 3-axis specific force (linear acceleration minus gravitational acceleration, in m/s²)
- Always includes gravity, so a stationary sensor sitting flat on a table reads roughly $9.81\,\text{m/s}^2$ upward rather than zero

**Gyroscope**:
- Measures 3-axis angular velocity (rad/s or deg/s)
- Tracks how fast the sensor body rotates around each axis

**Magnetometer** (included on some IMUs):
- Measures the 3-axis local magnetic field
- Can provide an absolute heading reference relative to magnetic north
- Easily distorted indoors or near motors, steel frames, and power cables

### 2.3.2 Key error characteristics

If IMU errors are left unmodeled, sensor fusion quickly destabilizes because dead reckoning integrates every small error continuously over time:

**Bias**:
- Nonzero output even when the sensor is completely stationary
- Drifts slowly over time (bias instability, defined as the floor of the Allan deviation curve; thermal drift from temperature changes is a separate error term)

**Noise**:
- High-frequency random measurement fluctuations
- Quantified via Allan variance analysis across averaging times

**Integration drift**:
- Integrating angular velocity once accumulates orientation error over time
- Double-integrating linear acceleration (after subtracting gravity using the estimated orientation) accumulates position error even faster
- Without external observations, both errors grow unboundedly

Double-integrating acceleration compounds white noise, bias, scale-factor errors, and initial tilt errors—even a tiny orientation mistake leaks a fraction of the $9.81\,\text{m/s}^2$ gravity vector into horizontal acceleration. Because growth depends on the specific hardware, motion profile, calibration, temperature, and initialization, no single time constant fits all platforms; unaided position from a low-cost MEMS IMU quickly blows past typical error budgets, which is why practical systems fuse IMU data with external references from cameras, LiDAR, or GNSS.

**IMU grades** do not follow a single standardized price ladder. Rather than relying on marketing labels, evaluate bias stability, noise density, scale-factor error, thermal calibration, vibration tolerance, and certification requirements. Consumer MEMS chips optimize for tiny size and low power, whereas industrial, tactical, and navigation-grade units invest heavily in long-term bias stability and factory calibration. When comparing devices such as the `VN-100`, `MTi`, `KVH 1750`, and `HG1700`, convert current datasheet specifications into common units and verify them experimentally with Allan variance tests.

> **Further reading**
> - [Probabilistic Robotics, Ch.5–6 — Robot Motion / Robot Perception (Thrun, Burgard, Fox)](http://probabilistic-robotics.org/) — Classic reference on probabilistic motion models (Ch.5) and sensor measurement models (Ch.6); note that it does not cover IMU error models (bias, random walk, Allan variance), for which Titterton & Weston is the standard reference.
> - [Titterton & Weston — Strapdown Inertial Navigation Technology](https://doi.org/10.1049/PBRA017E) — Authoritative textbook on inertial sensor physics, error equations, and strapdown navigation.
> - [Allan Variance — IMU noise analysis guide (Vectornav)](https://www.vectornav.com/resources/inertial-navigation-primer/specifications--and--error-budgets/specs-imucal) — Practical guide to extracting IMU noise and bias-instability parameters via Allan variance.
> - [Jinyong Jeong's blog — IMU Filter (AHRS)](https://jinyongjeong.github.io/2020/01/10/IMU_filter/) — Overview of Attitude and Heading Reference System (AHRS) filters such as Madgwick and their ROS packages.

## 2.4 GPS/GNSS

**GNSS (Global Navigation Satellite System)** determines global position by timing radio signals broadcast from satellites. GPS is the U.S. constellation, while GNSS serves as the umbrella term encompassing GPS, GLONASS (Russia), Galileo (Europe), BeiDou (China), and regional constellations.

Whereas SLAM estimates a robot's pose relative to its starting frame—accumulating drift over long distances—GNSS anchors outdoor autonomous vehicles and drones in an Earth-fixed coordinate frame (latitude, longitude, and altitude). Outdoor robots routinely fuse GNSS with local odometry to bound global drift, and RTK-GPS trajectories also serve as ground truth when evaluating SLAM accuracy.

**Interpreting accuracy**: under open sky, a standalone code-based GNSS receiver typically yields meter-class accuracy, while differential corrections tighten that error to sub-meter or centimeter levels depending on atmospheric and link conditions. Real-Time Kinematic (RTK) positioning achieves horizontal centimeter-class accuracy when a short baseline, favorable satellite geometry, low multipath, and fixed carrier-phase ambiguities hold. In urban canyons with heavy signal reflections off glass and concrete, however, even RTK can lose ambiguity fix and jump by several meters—which is why urban autonomous vehicles pair GNSS with LiDAR and camera SLAM rather than trusting satellite fixes blindly. Always read quoted GNSS specs together with the error metric (CEP, RMS, or 95%), horizontal versus vertical component, baseline length, correction link, and fix state.

**RTK-GPS principle**:
- A surveyed, stationary base station measures local satellite signal errors and broadcasts correction data
- The moving rover applies those carrier-phase corrections in real time to eliminate common atmospheric and clock errors
- Requires a continuous real-time correction link (radio or cellular/NTRIP internet)

**Limitations**:
- Unavailable indoors and in tunnels; severely degraded in dense urban canyons or under thick tree canopy
- Vulnerable to multipath errors when signals bounce off nearby buildings
- Vertical (altitude) accuracy is noticeably worse than horizontal accuracy due to satellite constellation geometry

> **Further reading**
> - [Cyrill Stachniss — Robot Localization Overview](https://www.youtube.com/watch?v=8VJ-A9OlhAE) — Broad overview of robot localization principles across global and local sensors.
> - [u-blox — RTK and GNSS corrections](https://www.u-blox.com/en/technologies/rtk-real-time-kinematic) — Practical primer contrasting code-based GNSS ranging with carrier-phase RTK and correction services.

## 2.5 Other sensors

Beyond cameras, LiDAR, IMUs, and GNSS, several complementary modalities provide critical redundancy when primary optical or satellite sensors degrade.

**Radar**

Autonomous vehicles and field robots frequently pair radar with cameras and LiDAR. Millimeter-wave radio signals penetrate fog, rain, dust, and harsh glare far more reliably than visible light or many LiDAR wavelengths, though radar still contends with heavy rain attenuation, ground clutter, multipath reflections, wet radomes, and coarser angular resolution. Pricing across modern imaging radars also overlaps with LiDAR families depending on antenna array size, bandwidth, and automotive qualification, so system designers compare actual quotes for the target configuration.

**FMCW (Frequency Modulated Continuous Wave) Radar**:
- Sweeps the transmitted frequency continuously over time and measures both the beat frequency and Doppler shift of the echo to recover range and radial velocity simultaneously in a single chirp
- Output: range-Doppler map (a 2D grid of distance vs. radial velocity) and range-azimuth map
- 77 GHz is the standard band for automotive radar

**Applications in robotics**:
- Autonomous driving: forward collision warning and adaptive cruise control (ACC)
- Radar odometry: estimating vehicle motion from radar scans alone in smoke, fog, or snow
- Radar SLAM: building maps and localizing using scanning or imaging radar

To see where radar complements optical sensors, compare the three core perception modalities side by side:

| Characteristic | Camera | LiDAR | Radar |
|------|--------|-------|-------|
| Resolution | Very high | High | Low |
| Range measurement | Not possible (monocular) | Accurate | Possible |
| Velocity measurement | Not possible | Not possible for ToF units (FMCW LiDAR: possible via Doppler) | Possible (Doppler) |
| Adverse weather | Weak | Weak (rain, fog) | Robust |
| Price | Cheap | Expensive | Medium |
| Nighttime | Weak (needs illumination) | Possible | Possible |

**Representative products**: Texas Instruments AWR1843, Continental ARS548, Navtech CTS350-X (spinning radar)

> **Further reading**
> - [Giseop Kim's blog — ICRA 2021 Radar in Robotics Workshop summary](https://gisbi-kim.github.io/blog/2021/05/31/icra21-radar-ws.html) — Summary of research trends and challenges in robotics radar.
> - [Giseop Kim's blog — Radar Odometry Results on MulRan dataset](https://gisbi-kim.github.io/blog/2021/05/30/yeti-radar-odom-mulran1.html) — Trajectory evaluation of Yeti radar odometry (Cen2018 and Cen2019 methods) on MulRan urban sequences.
> - [Kim et al., "MulRan: Multimodal Range Dataset for Urban Place Recognition" (ICRA 2020)](https://sites.google.com/view/mulran-pr/home) — Urban multimodal dataset combining LiDAR, spinning radar, and GPS.

**Ultrasonic**:
- Detects close-range obstacles (roughly 0.2–5 m) via acoustic echo timing
- Very low cost and unaffected by optical transparency such as clear glass
- Widely used for parking assistance and low-speed proximity safety bumpers

**Wheel encoder**:
- Counts wheel shaft ticks to measure rotation
- Provides high-rate planar dead reckoning on wheeled platforms
- Prone to unbounded drift whenever wheels slip on wet, loose, or uneven terrain

Grouping radar, sonar, and wheel encoders under "other" does not make them secondary. Radar often acts as the weatherproof safety net when rain or dust blinds LiDAR, and wheel encoders remain the bedrock odometry source for indoor mobile bases—in a fused pipeline, these auxiliary sensors often determine overall system robustness.

> **Further reading**
> - [Probabilistic Robotics, Ch.6 — Robot Perception (Thrun, Burgard, Fox)](http://probabilistic-robotics.org/) — Thorough treatment of probabilistic measurement models across ultrasonic, laser, and feature-based sensors.

## 2.6 Sensor Fusion

Every sensor modality we have examined has blind spots: no single device handles every combination of lighting, range, weather, occlusion, and drift. Depending on payload, cost, and operating environment, robotic platforms combine subsets of cameras, LiDAR, radar, IMUs, and GNSS—and *when*, *where*, and *how* those streams are combined shapes system accuracy and resilience.

**Why is it needed?**

Each sensor fills in where another fails, as the trade-off matrix below illustrates:

| Sensor | Pros | Cons |
| --- | --- | --- |
| Camera | Rich information, cheap | Lighting-dependent, no depth |
| LiDAR | Accurate 3D, lighting-independent | Expensive, sparse |
| IMU | High frequency, lighting-independent | Drift |
| GPS | Global position | Outdoor only, low frequency |

Architectures generally combine sensor streams at one of three stages:

**Fusion approaches**:
1. **Early Fusion**: combines raw or lightly preprocessed sensor measurements (for example, projecting LiDAR points onto image pixels)
2. **Late Fusion**: runs perception or estimation independently per sensor and merges their output poses, tracks, or detections
3. **Mid-Level Fusion**: merges intermediate feature representations extracted from each modality

Each stage balances fidelity against modularity. Early fusion preserves fine cross-modal correlations at the cost of tight calibration coupling and heavy compute, whereas late fusion keeps modules decoupled and easy to swap—at the expense of discarding cross-sensor cues before integration. Mid-level fusion sits between the two and has become standard in deep multimodal perception.

In state estimation and SLAM, three canonical sensor pairings dominate:

**Representative combinations**:
- Camera + IMU → VIO (visual-inertial odometry)
- LiDAR + IMU → LIO (LiDAR-inertial odometry)
- Camera + LiDAR + IMU → multimodal SLAM

To fuse noisy measurements mathematically—weighting a high-rate IMU against a sparse LiDAR scan or an intermittent GNSS fix—an estimator needs a formal rule stating how likely any sensor reading is given a candidate pose and map. That is the role of probabilistic measurement models, covered next in §2.7.

> **Further reading**
> - [State Estimation for Robotics (Tim Barfoot) — free PDF](http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) — Leading graduate textbook on sensor fusion, covering Kalman filters, nonlinear batch estimation, and Lie-group formulations.
> - [Cyrill Stachniss — Kalman Filter & EKF](https://www.youtube.com/watch?v=E-6paM_Iwfc) — Clear explanation of the Kalman filter and extended Kalman filter (EKF) at the heart of classical sensor fusion.
> - [Qin et al. — VINS-Mono (TRO 2018)](https://arxiv.org/abs/1708.03852) — Landmark paper on monocular visual-inertial odometry showing how tight optimization fuses camera and IMU streams in practice.

---

## 2.7 Deep Dive: Measurement Models — Probabilistic Formulation

Whenever a new scan or image arrives, Bayes filters, Monte Carlo Localization (MCL), and SLAM need a principled way to answer: "How consistent is this sensor reading with the robot being at pose $x_t$ in map $m$?" Formally, that question is expressed as the conditional probability density $p(z_t \mid x_t, m)$, known as the **measurement model**.

### 2.7.1 The Distribution a Sensor Produces

Aim a laser rangefinder at the exact same wall from the exact same pose a hundred times, and you will get a distribution of slightly different readings rather than one fixed number. Incidence angle, surface reflectance, a pedestrian briefly crossing the beam, and multipath bounces each leave a distinct statistical footprint. The per-beam likelihood $p(z_t^k \mid x_t, m)$ captures that spread, where $z_t^k$ is the $k$-th beam's range reading at time $t$, $x_t$ is the robot's pose, and $m$ is the map.

A single LiDAR or sonar scan contains tens to hundreds of beams. Computing the full joint distribution over all beams simultaneously is intractable, so *Probabilistic Robotics* (PR §6.1, Eq. 6.2) introduces the **conditional independence assumption**: given the true pose and map, noise on individual beams is treated as independent, reducing the full-scan likelihood to a product over beams:

$$p(z_t \mid x_t, m) = \prod_{k=1}^{K} p(z_t^k \mid x_t, m)$$

In practice, conditional independence is only an approximation. Neighboring beams hitting the same flat wall share systematic errors—ignoring that correlation causes the product to double-count evidence and peak too sharply around a single pose, a pitfall revisited in §2.7.8.

The structure of the map $m$ also dictates how measurements are evaluated. Maps generally take one of two forms: a **feature-based map** stores a list of discrete landmarks indexed by ID, whereas a **location-based map** stores occupancy probabilities over grid cells indexed by spatial coordinate.

Four major families of measurement models build on those two map representations:
- Beam model: a four-component mixture approximating physical causes of a range reading. Location-based map.
- Likelihood field: scores endpoints by distance to the nearest obstacle. Location-based map.
- Correlation-based (map matching): normalized correlation between a local scan grid and the global map.
- Feature-based (landmark model): extracted features modeled as (range, bearing, signature). Feature-based map.

### 2.7.2 Beam Model — Four-Component Mixture

A single Gaussian centered on the true distance cannot explain real rangefinder data: in addition to small measurement jitter around the target wall, beams occasionally bounce off unmapped pedestrians, fail to return a reflection at all, or produce erratic spikes. To capture all four behaviors cleanly, [Thrun et al. 2005](http://probabilistic-robotics.org/) (PR §6.3.1) models each beam reading as a weighted mixture of four distinct distributions. Importantly, these four components are statistical terms in an observation model, not separate physical hardware channels inside the sensor.

The dominant component is **hit**, representing a genuine return from the mapped obstacle at predicted range $z_t^{k*}$. Small range jitter is modeled by a Gaussian with variance $\sigma_{\text{hit}}^2$, truncated to the sensor's valid interval $[0, z_{\max}]$ and renormalized by $\eta$ so the density integrates to one:

$$p_{\text{hit}}(z_t^k \mid x_t, m) = \eta\, \mathcal{N}(z_t^k;\, z_t^{k*},\, \sigma_{\text{hit}}^2), \quad 0 \le z_t^k \le z_{\max}$$

**short (unexpected nearby obstacle)**: A person walking past or an unmapped chair intercepts the beam before it reaches the mapped wall, so the reading is always shorter than $z_t^{k*}$ and lies in $[0, z_t^{k*}]$. Because closer unmapped objects are more likely to block a ray before farther ones do, PR models this with a truncated exponential decay parameterized by $\lambda_{\text{short}}$:

$$p_{\text{short}}(z_t^k \mid x_t, m) = \eta\, \lambda_{\text{short}}\, e^{-\lambda_{\text{short}} z_t^k}, \quad 0 \le z_t^k \le z_t^{k*}$$

**max (maximum-range failure)**: Black matte surfaces, specular mirror angles, or fog can swallow the laser return completely, causing the sensor firmware to output $z_{\max}$. This behavior concentrates probability mass directly at $z_{\max}$ as an indicator (point mass):

$$p_{\text{max}}(z_t^k \mid x_t, m) = \mathbf{1}[z_t^k = z_{\max}]$$

**rand (unexplained noise)**: Crosstalk between ultrasonic transducers, multipath reflections, or phantom returns yield readings with no predictable pattern across $[0, z_{\max}]$, modeled as a uniform distribution:

$$p_{\text{rand}}(z_t^k \mid x_t, m) = \frac{1}{z_{\max}}$$

Combining these four hypotheses gives the complete per-beam likelihood as a convex weighted mixture (PR Eq. 6.13):

$$p(z_t^k \mid x_t, m) = \begin{pmatrix} z_{\text{hit}} \\ z_{\text{short}} \\ z_{\text{max}} \\ z_{\text{rand}} \end{pmatrix}^T \cdot \begin{pmatrix} p_{\text{hit}}(z_t^k \mid x_t, m) \\ p_{\text{short}}(z_t^k \mid x_t, m) \\ p_{\text{max}}(z_t^k \mid x_t, m) \\ p_{\text{rand}}(z_t^k \mid x_t, m) \end{pmatrix}$$

Here the mixture weights must sum to one: $z_{\text{hit}} + z_{\text{short}} + z_{\text{max}} + z_{\text{rand}} = 1$.

To evaluate $p_{\text{hit}}$ and $p_{\text{short}}$, we first need the expected noiseless range $z_t^{k*}$ given pose $x_t$ and map $m$. That value is computed via **ray casting**: stepping outward along the $k$-th beam's ray until it intersects the first occupied cell in the map; that distance is $z_t^{k*}$. (Ray casting applies the same geometric projection logic introduced in §2.1 for camera rays and §2.2 for LiDAR beams, now evaluated against an occupancy grid.)

Putting ray casting and the four-component mixture together yields the classic full-scan likelihood algorithm:

**Algorithm: beam_range_finder_model** (adapted from PR Table 6.1)

```
Input:  z_t = {z_t^1, ..., z_t^K}, x_t, m
Output: p(z_t | x_t, m)

1. q ← 1
2. for k = 1 to K do:
3.     z_t^{k*} ← ray_cast(x_t, k, m)   // predicted range
4.     p ← z_hit  * p_hit(z_t^k | z_t^{k*}, σ_hit)
             + z_short * p_short(z_t^k | z_t^{k*}, λ_short)
             + z_max   * p_max(z_t^k | z_max)
             + z_rand  * p_rand(z_t^k | z_max)
5.     q ← q * p
6. return q
```

While conceptually faithful to sensor physics, this model has six intrinsic parameters that depend heavily on the specific hardware and environment—raising the question of how to estimate them from data rather than guessing by hand.

### 2.7.3 Beam Model — EM Parameter Learning

Whenever you change the sensor model, swap from an empty warehouse to a crowded hallway, or remount the scanner, the balance of `hit`, `short`, `max`, and `rand` events—as well as $\sigma_{\text{hit}}$ and $\lambda_{\text{short}}$—shifts. Hand-tuned parameters tailored to one room tend to misfire somewhere else.

Rather than tuning the six intrinsic parameters ($z_{\text{hit}}, z_{\text{short}}, z_{\text{max}}, z_{\text{rand}}, \sigma_{\text{hit}}, \lambda_{\text{short}}$) manually, PR §6.3.2 learns them via maximum likelihood using the **Expectation-Maximization (EM)** algorithm on paired measurements $\{(z_t^k, z_t^{k*})\}$ recorded while driving through a known map. EM is needed because the logged data records only the measured range $z^i$, not *which* of the four physical causes generated it; introducing a latent correspondence variable $c_i \in \{\text{hit, short, max, rand}\}$ lets the algorithm alternate between inferring component responsibilities and updating parameters.

**E-step**: Holding the current parameters fixed, compute each measurement's posterior responsibility—the probability that component $c_i$ generated $z^i$ (PR Eq. 6.15–6.32):

$$e_{\text{hit}}^i = \frac{z_{\text{hit}} \cdot p_{\text{hit}}(z^i \mid z^{i*})}{p(z^i \mid z^{i*})}, \quad e_{\text{short}}^i = \frac{z_{\text{short}} \cdot p_{\text{short}}(z^i \mid z^{i*})}{p(z^i \mid z^{i*})}, \quad \dots$$

**M-step**: Using those soft assignments as weights, re-estimate the parameters to maximize expected log-likelihood. Both $\sigma_{\text{hit}}^2$ and $\lambda_{\text{short}}$ admit clean closed-form updates:

$$\sigma_{\text{hit}}^2 = \frac{\sum_i e_{\text{hit}}^i (z^i - z^{i*})^2}{\sum_i e_{\text{hit}}^i}, \qquad \lambda_{\text{short}} = \frac{\sum_i e_{\text{short}}^i}{\sum_i e_{\text{short}}^i \cdot z^i}$$

The four mixture weights $z_{\text{hit}}, z_{\text{short}}, z_{\text{max}}, z_{\text{rand}}$ are updated as the average responsibility assigned to each component across the dataset.

Iterating these two steps until convergence gives the parameter-learning procedure below:

**Algorithm: learn_intrinsic_parameters** (condensed from PR Table 6.2)

```
Input:  {(z^i, z^{i*})} — (measured, predicted) pairs
Output: z_hit, z_short, z_max, z_rand, σ_hit, λ_short

Initialize: set parameters to uniform or arbitrary values
repeat until convergence:
    // E-step
    for each i:
        e_hit^i, e_short^i, e_max^i, e_rand^i ← posterior(z^i, z^{i*}, params)
    // M-step
    z_hit  ← mean(e_hit^i);   z_short ← mean(e_short^i)
    z_max  ← mean(e_max^i);   z_rand  ← mean(e_rand^i)
    σ_hit² ← weighted variance of (z^i - z^{i*}) by e_hit^i
    λ_short ← sum(e_short^i) / sum(e_short^i * z^i)
return params
```

Keep in mind that EM fits the specific sensor, map, and crowd conditions present in the training log. Although packages such as AMCL expose `sigma_hit`, `lambda_short`, and the mixture weights as parameters, their default settings are convenient starting points rather than universal constants learned across all environments.

Even with well-calibrated parameters, running the beam model inside a real-time localization loop introduces computational bottlenecks that require a few practical workarounds.

### 2.7.4 Beam Model — Practical Considerations

Evaluating the beam model in Monte Carlo Localization (MCL) requires ray casting every beam from every particle's pose—a cost that scales as (number of particles) × (number of beams) on every scan update. Three adjustments help keep runtime tractable and prevent statistical overconfidence:

- **Beam subsampling**: Evaluating a uniformly spaced subset of beams cuts runtime proportionally while reducing redundant correlation between adjacent rays, though the chosen stride should be validated against scan density, corridor geometry, and particle count.
- **Exponentiation correction ($p^{\alpha}$)**: Because neighboring beams are not truly independent, multiplying hundreds of per-beam likelihoods can concentrate probability mass too aggressively around one pose. Tempering the scan likelihood as $p(z_t \mid x_t, m)^{\alpha}$ with $0 < \alpha < 1$ dampens each beam's vote and prevents particle collapse; $\alpha$ is tuned empirically or via cross-validation.
- **Precomputed range table**: For static maps, ray-cast distances can be precomputed across discretized $(x, y, \theta)$ cells and stored in a lookup table, turning runtime ray casting into an $O(1)$ memory fetch. Although memory consumption grows rapidly for large maps, this caching trick makes beam-model MCL fast enough for real-time operation (see Ch.3 §3.11 for particle filters).

### 2.7.5 Likelihood Field

Even with subsampling or lookup tables, the beam model suffers from two deeper structural drawbacks. First, ray casting is expensive to compute on the fly. Second, shifting the robot pose $x_t$ by a tiny amount can cause a ray to graze past a doorway corner and suddenly hit a wall ten meters farther back—creating sharp discontinuities in $z_t^{k*}$ and therefore in $p(z_t \mid x_t, m)$. Non-smooth likelihood surfaces stall gradient-based scan matchers and hill-climbing optimizers.

The **likelihood field** avoids both issues by abandoning ray casting altogether. Instead of tracing a ray through the map, it takes each measured range $z_t^k$, projects its endpoint directly into global map coordinates, and scores the Euclidean distance $\text{dist}$ from that endpoint to the nearest occupied cell in $m$. Because distance to the nearest obstacle varies smoothly across space, the resulting likelihood surface becomes smooth and differentiable almost everywhere.

Given robot pose $(x, y, \theta)^T$, the global coordinates of the $k$-th beam's endpoint are computed by rigid-body transformation (PR Eq. 6.33):

$$\begin{pmatrix} x_{z_t^k} \\ y_{z_t^k} \end{pmatrix} = \begin{pmatrix} x \\ y \end{pmatrix} + \begin{pmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{pmatrix} \begin{pmatrix} x_{k,\text{sens}} \\ y_{k,\text{sens}} \end{pmatrix} + z_t^k \begin{pmatrix} \cos(\theta + \theta_{k,\text{sens}}) \\ \sin(\theta + \theta_{k,\text{sens}}) \end{pmatrix}$$

Here $(x_{k,\text{sens}}, y_{k,\text{sens}})$ denotes the sensor's mounting offset in the robot body frame, and $\theta_{k,\text{sens}}$ is the $k$-th beam's firing angle relative to the sensor.

Once the endpoint is in map coordinates, the likelihood field drops the `short` and `max` components and evaluates a simple mixture of a zero-mean Gaussian over $\text{dist}$ (the Euclidean distance from the beam endpoint to the nearest occupied cell) plus a uniform background term (PR Eq. 6.34–6.35):

$$p(z_t^k \mid x_t, m) = z_{\text{hit}} \cdot \mathcal{N}(\text{dist};\, 0,\, \sigma_{\text{hit}}^2) + z_{\text{rand}} \cdot \frac{1}{z_{\max}}$$

Max-range readings ($z_t^k = z_{\max}$) are skipped entirely because projecting a non-return ray to $z_{\max}$ carries no geometric information about obstacle boundaries.

With a precomputed distance table, evaluating the full scan requires only coordinate transforms and array lookups:

**Algorithm: likelihood_field_range_finder_model** (adapted from PR Table 6.3)

```
Input:  z_t = {z_t^1, ..., z_t^K}, x_t = (x, y, θ)^T, m
Output: p(z_t | x_t, m)

1. q ← 1
2. for each k do:
3.     if z_t^k == z_max: continue   // skip max-range readings
4.     // transform beam endpoint to global coordinates
5.     x_ep ← x + x_{k,sens}·cos(θ) - y_{k,sens}·sin(θ) + z_t^k · cos(θ + θ_{k,sens})
6.     y_ep ← y + y_{k,sens}·cos(θ) + x_{k,sens}·sin(θ) + z_t^k · sin(θ + θ_{k,sens})
7.     // distance to nearest obstacle (lookup from precomputed distance transform table)
8.     dist ← nearest_obstacle_distance(x_ep, y_ep, m)
9.     q ← q * (z_hit · N(dist; 0, σ_hit²) + z_rand / z_max)
10. return q
```

For any static map, the distance transform table can be precomputed once—matching the positive half of a Signed Distance Field (SDF)—so every $\text{dist}$ query runs in $O(1)$ time. Combined with spatial smoothness, that speed makes the likelihood field ideal for gradient-based scan matching.

That simplicity comes with clear trade-offs. Because the likelihood field has no `short` component, it does not explicitly explain dynamic obstacles, and because it checks only the endpoint rather than the ray path leading to it, a beam passing straight through an occupied wall incurs zero penalty—allowing the model to effectively "see through walls" while also ignoring map uncertainty.

Even so, speed and pose continuity make the likelihood field the default choice in 2D LiDAR indoor navigation packages like AMCL. In 3D LiDAR and RGB-D pipelines, iterative point-cloud registration methods such as ICP and NDT have largely taken over that scan-matching role (Ch.3 §3.10; see Ch.14 §14.7 for Kalman-filter integration).

When the task shifts from fine pose tracking to loop closure—rapidly checking whether a current submap matches a previously visited corridor—teams often prioritize raw matching speed even further, trading away probabilistic rigor altogether.

### 2.7.6 Correlation-Based Model (Map Matching)

The correlation-based model is the most heuristic of the four families. Rather than scoring individual beams, it rasterizes recent scans into a small local grid $m_{\text{local}}$ and compares its occupancy values against the global map $m$ at candidate pose $x_t$ using the Pearson normalized correlation coefficient $\rho$. PR §6.5 clips negative correlations at zero to form a pseudo-likelihood:

$$p(m_{\text{local}} \mid x_t, m) = \max\{\rho(m_{\text{local}}, m \mid x_t),\ 0\}$$

Computing $\rho$ across overlapping grid cells is fast and easy to implement, which suits coarse loop-closure search. Its limitation is purely theoretical: $\rho$ is simply normalized and clipped below zero, so it does not produce a valid probability density grounded in sensor physics and is best treated as a fast similarity score rather than a calibrated Bayesian likelihood.

All three models above operate directly on dense range scans or grid cells. The final family takes a different route: compressing raw sensor streams into compact geometric or visual landmarks first.

### 2.7.7 Feature-Based Measurement — Landmark Model

Instead of processing hundreds of raw laser returns or millions of pixels, the **landmark model** extracts compact features $f(z_t)$—such as corners, line intersections, or visual keypoints—and matches them against a feature-based map. Inference over a handful of landmarks is far cheaper computationally and avoids dense grid storage.

Feature extraction depends on the sensor modality: range scans yield line segments, corners, or local distance minima, whereas cameras yield edges, corners, or SIFT/ORB descriptors (connecting back to §2.1.1 on visual texture and §2.6 on VIO/Visual SLAM). Each detected landmark observation is represented as a triple $(r, \phi, s)$: range $r$, bearing $\phi$, and an optional signature $s$ (such as an ID, color, or descriptor).

Suppose the $j$-th landmark in the map sits at $(m_{j,x}, m_{j,y})$ with signature $s_j$. When the robot stands at pose $x_t = (x, y, \theta)^T$, geometry dictates the predicted range and bearing, and PR Eq. 6.40 models the noisy measurement as that prediction corrupted by independent zero-mean Gaussian noise $\varepsilon_{\sigma^2}$ with variance $\sigma^2$ on each channel:

$$\begin{pmatrix} r_t^i \\ \phi_t^i \\ s_t^i \end{pmatrix} = \begin{pmatrix} \sqrt{(m_{j,x} - x)^2 + (m_{j,y} - y)^2} \\ \operatorname{atan2}(m_{j,y} - y,\, m_{j,x} - x) - \theta \\ s_j \end{pmatrix} + \begin{pmatrix} \varepsilon_{\sigma_r^2} \\ \varepsilon_{\sigma_\phi^2} \\ \varepsilon_{\sigma_s^2} \end{pmatrix}$$

Notice that adding Gaussian noise directly to the bearing $\varepsilon_{\sigma_\phi^2}$ can wrap awkwardly near $\pm\pi$; practical implementations normalize angular residuals into $[-\pi, \pi]$ or use circular distributions such as the von Mises distribution.

When the correspondence $c_t^i = j$ (indicating that the $i$-th feature corresponds to the $j$-th landmark) is known, the observation likelihood simplifies to the product of three one-dimensional Gaussians:

**Algorithm: landmark_model_known_correspondence** (adapted from PR Table 6.4)

```
Input:  f_t^i = (r_t^i, φ_t^i, s_t^i)^T, correspondence c_t^i = j,
        x_t = (x, y, θ)^T, m
Output: p(f_t^i | c_t^i = j, x_t, m)

1. j ← c_t^i
2. r̂ ← sqrt((m_{j,x} - x)² + (m_{j,y} - y)²)
3. φ̂ ← atan2(m_{j,y} - y, m_{j,x} - x) - θ
4. q ← prob(r_t^i - r̂, σ_r²)
       * prob(φ_t^i - φ̂, σ_φ²)
       * prob(s_t^i - s_j, σ_s²)
5. return q
   // prob(a, σ²) = N(a; 0, σ²) — zero-mean Gaussian density
```

Under conditional independence across detected features, the full-scan likelihood is again the product $\prod_i$ over all matched landmarks.

**Reverse direction — pose sampling** (condensed from PR Table 6.5): the same geometric relation can also run backward to sample candidate robot poses from a single landmark observation. Because one $(r, \phi)$ pair supplies only two scalar constraints in a 3-DoF planar pose space $(x, y, \theta)$, the compatible poses form a circle around the landmark in 2D (or a helix in 3D), parameterized by a free angle $\hat{\gamma} \sim U(0, 2\pi)$. Under range and bearing noise, that circle smears into an annular "donut" of plausible robot positions—explaining geometrically why observing a single landmark alone cannot uniquely localize a robot without additional landmarks or motion constraints.

<!-- DEMO: landmark_donut.html -->

Visual SLAM uses the same residual structure when it minimizes pixel reprojection error $\| \pi(K[R|t]\, X_w) - u \|^2_\Sigma$: predict an observation from the current pose and 3D point, then compare it against the measured pixel. That structural parallel does not make pixel reprojection and range-bearing sensing identical models, however—in visual SLAM, ORB or SIFT descriptors typically drive discrete data association rather than entering the likelihood as a continuous Gaussian signature term, and while fiducial tags like AprilTag or ArUco sharply cut correspondence ambiguity, false positives and misread IDs can still occur.

### 2.7.8 Practical Summary: Choosing a Model

Comparing the four measurement model families side by side highlights the trade-offs between physical fidelity, runtime speed, and smoothness (exact performance always depends on sensor rate, map resolution, and implementation):

| Model | Accuracy | Speed | Differentiable | Primary use |
|------|--------|-----------|------------|-----------|
| Beam model | High | Slow (ray casting) | Low (discontinuous) | MCL high-fidelity, diagnostics |
| Likelihood field | Medium | Fast (DT lookup) | High | AMCL default, gradient matching |
| Correlation-based | Low | Very fast | Low | Loop closure detection |
| Landmark model | High (feature-dependent) | Fast (low-dimensional) | High | Visual SLAM, fiducial |

Across all dense scan models, overconfidence remains a common failure mode. When conditional independence breaks down across correlated adjacent rays, $p(z_t \mid x_t, m)$ spikes too sharply around a single pose hypothesis. Tempering the likelihood as $p(z_t \mid x_t, m)^{\alpha}$ ($\alpha < 1$), as noted in §2.7.4, softens each scan's weight; subsampling beams, modeling spatial correlation explicitly, or adopting robust loss functions are complementary fixes, and $\alpha$ should always be tuned against validation data rather than treated as a universal constant.

With these trade-offs and mitigations in view, the natural next question is how these classical formulations carry over—or give way—in modern production robotics stacks.

### 2.7.9 What Survived

The four families in PR §6 remain a valuable mental map for designing and debugging perception pipelines, yet it would be misleading to call every modern scan matcher or visual SLAM system a **direct descendant** of those exact formulas. Modern systems share the overarching pattern of comparing predicted observations against measurements, even when their objective functions and map representations diverge.

In 2D navigation, the [Nav2 AMCL documentation](https://docs.nav2.org/configuration/packages/configuring-amcl.html) exposes three laser sensor models—`beam`, `likelihood_field`, and `likelihood_field_prob`—defaulting to `likelihood_field`. Its `max_beams` setting controls uniform scan subsampling, whereas the `beam_skip_*` parameters belong specifically to `likelihood_field_prob` and discard beams that conflict with a majority of particles (a distinct mechanism from ordinary subsampling). Configured defaults like `sigma_hit` and `lambda_short` serve as practical starting values rather than universally converged outputs from a single EM run.

Other LiDAR systems adopt altogether different matching objectives. Cartographer pairs correlative scan matching on probability grids with nonlinear optimization, while `hdl_localization` registers 3D point clouds via Normal Distributions Transform (`NDT_OMP` or `NDT_CUDA`). Both compare incoming scans against a map, yet neither implements a likelihood-field distance transform; similarly, ESDF planners and neural implicit maps store distance or implicit fields for planning and reconstruction without deriving from PR's sensor likelihood model.

The same caution applies to landmark and visual SLAM. Range-bearing observation equations (Eq. 6.40), pinhole reprojection residuals, and DROID-SLAM's dense bundle adjustment all compare predicted observations with sensor measurements, yet their measurement spaces, correspondence pipelines, noise assumptions, and state parameterizations are fundamentally distinct rather than steps along a single historical lineage. Likewise, fiducial markers simplify data association without eliminating false detections.

Ultimately, `hit`, `short`, `max`, and `rand` are mixture components approximating how range errors arise—not separate physical channels inside a sensor—and choosing a measurement model depends on your hardware, environment, outlier profile, compute budget, and calibration data.

To see how `beam_range_finder_model` is invoked inside a full localization loop and how `inverse_sensor_model` drives occupancy mapping, jump ahead to Ch.14 §14.7. Before building those full SLAM pipelines, however, we need the shared mathematical toolkit—coordinate transformations, linear algebra, Lie groups, and Bayesian filtering—that turns raw sensor streams into 3D state estimates, which is the focus of Ch.3.

> **⚠ Sensor connection check**: When a sensor stops publishing data, check physical cables, IP/subnet settings, power rails, and USB bandwidth alongside software drivers. System tools like `dmesg`, `lsusb`, and `ping` quickly reveal whether the fault lies at the hardware or driver layer.

> **Technical Timeline: sensor technology**
> - **~2010**: 2D LiDAR and standard frame cameras anchored most mobile-robot research, while real-time stereo range remained tightly constrained by onboard compute and scene texture.
> - **2010s**: Affordable RGB-D cameras and compact multi-beam 3D LiDARs unlocked indoor 3D reconstruction and outdoor mapping, while camera–IMU VIO matured from lab prototypes into deployed robotics and AR products.
> - **Late 2010s to early 2020s**: Non-repetitive prism scanners, MEMS mirrors, and flash architectures expanded under the umbrella of `solid-state LiDAR`, alongside growing research in event cameras and automotive imaging radar; price and performance varied too widely across tiers to capture with a single trendline.
> - **2020s**: Spinning and solid-state LiDARs coexist across platforms because field of view, range, resolution, motion distortion, and cost impose distinct trade-offs, while event cameras and Doppler radar see targeted adoption in high-speed, HDR, and adverse-weather regimes.
> - **Design implication**: Changing a sensor's scan pattern or timestamp layout alters core assumptions in motion deskewing, extrinsic calibration, and data association—always inspect actual sampling geometry and noise statistics rather than relying on a hardware category label.
