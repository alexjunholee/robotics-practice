# Ch.2 — Sensors


A robot needs sensors to perceive its environment. Understanding each sensor's characteristics enables proper sensor selection and algorithm design.

Diagnosing whether a SLAM tracking failure began with rolling shutter, LiDAR reflectance, or IMU bias requires the sensor's measurement and error model. That model determines the data received by the downstream perception and estimation algorithms.

## 2.1 Camera

A camera measures color and texture at high spatial resolution. Monocular, stereo, RGB-D, and event cameras differ in depth measurement, temporal resolution, and response to illumination, so the task conditions determine which type is appropriate.

### 2.1.1 Monocular Camera

The most basic visual sensor, capturing a 2D image with a single lens.

A monocular camera uses one lens to capture color and texture for tasks such as visual SLAM, object recognition, and semantic understanding. Because it does not measure depth directly, monocular depth estimation and SfM infer scene structure from other cues. Stereo and depth cameras address this ambiguity through different measurements.

**Pros**:
- Cheap and lightweight
- Rich color and texture information
- High resolution

**Cons**:
- Cannot directly measure depth from a single image
- Scale ambiguity: the real size of an object is unknown

**Key specifications**:
- Resolution: 720p, 1080p, 4K, etc.
- Frame rate: 30fps, 60fps, 120fps, etc.
- Field of View (FoV): narrow FoV vs. wide FoV (fisheye)
- Global shutter vs. rolling shutter

```
Typical camera sensors:
- Webcams: Logitech C920, C930e
- Industrial: FLIR (Point Grey), Basler, Allied Vision
- Embedded: Raspberry Pi Camera, OAK-D
```

> **Further reading**
> - [First Principles of Computer Vision — Camera and Imaging](https://www.youtube.com/playlist?list=PL2zRqk16wsdoCCLpou-dGo7QQNks1Ppzo) — Columbia University Prof. Shree Nayar's lectures on camera principles. Covers from pinhole models to lens distortion.
> - [OpenCV Camera Calibration Tutorial](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html) — Hands-on guide to performing camera calibration yourself.

### 2.1.2 Stereo Camera

Two cameras placed at a fixed interval (baseline) measure depth. The principle is similar to human binocular vision.

Obtaining depth outdoors requires stereo vision. A stereo camera is almost the only passive way (without actively emitting light) to measure depth. In outdoor settings such as autonomous driving and drones, structured light and ToF break down under sunlight, so the principles of stereo vision matter. It ties directly to epipolar geometry and therefore to the mathematical foundations.

**Depth computation principle**:

```
Depth (Z) = (focal_length × baseline) / disparity
```

- **Disparity**: the difference in x-coordinate of the same point in the left and right images
- **Baseline**: the distance between the two cameras

**Pros**:
- Passive sensor (no illumination required)
- Usable in outdoor environments
- Acquires RGB information and depth simultaneously

**Cons**:
- Matching fails on textureless surfaces (white walls, glass)
- High computational cost
- Measurement range limited by baseline

**Representative products**:
- Intel RealSense D435/D455: active IR pattern projection to assist matching
- ZED 2: wide baseline, long-range measurement
- OAK-D: built-in edge AI

> **Further reading**
> - [Cyrill Stachniss — Stereo Vision](https://www.youtube.com/watch?v=SyB7Wg1e62A) — Explains the mathematical principles of stereo vision clearly.
> - [Stanford CS231A — Epipolar Geometry and Stereo](https://web.stanford.edu/class/cs231a/) — Stanford's computer vision course. Covers epipolar geometry well.

> **Exercise**: [Stereo Disparity visualization](https://alexjunholee.github.io/robotics-practice/app.html#stereo_disparity)
> Compute disparity from a stereo image pair and observe how baseline and focal length affect depth estimation.

### 2.1.3 RGB-D Camera

A sensor that directly provides an RGB image and a depth image.

The first sensor you are likely to encounter in a lab is an RGB-D camera, because it is the most convenient for experimenting with SLAM or 3D reconstruction in a desktop environment. Without knowing the difference between ToF and structured light, you cannot explain why depth values break down outdoors or why running several units at once causes interference.

**ToF (Time of Flight) method**:
- Emits infrared light and measures the return time
- Pros: texture-independent, real-time processing
- Cons: sunlight interference, issues with reflective surfaces
- Examples: Microsoft Azure Kinect, PMD Pico Flexx

**Structured light method**:
- Projects a known pattern and analyzes its deformation
- Pros: high accuracy, low cost
- Cons: hard to use outdoors, multi-sensor interference
- Examples: Intel RealSense D400 series, Orbbec Astra

**Comparison**:
| Characteristic | ToF | Structured Light |
|------|-----|------------------|
| Outdoor use | Limited | Difficult |
| Accuracy | Medium | High |
| Range | 0.2-5m | 0.2-10m |
| Multi-sensor | Possible | Interference occurs |

> **Further reading**
> - [Intel RealSense — Depth Cameras D415 & D435](https://www.youtube.com/watch?v=A4Kjvosvx5I) — Intel's own explanation of depth camera principles.
> - [Open3D RGB-D Reconstruction Tutorial](http://www.open3d.org/docs/release/tutorial/pipelines/rgbd_integration.html) — Hands-on tutorial for 3D reconstruction with RGB-D data.

**Installing the RealSense driver (Ubuntu 22.04)**

```bash
# Install the Intel RealSense SDK
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
    sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update
sudo apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev

# Test
realsense-viewer
```

For use with ROS2, additionally:
```bash
sudo apt install ros-humble-realsense2-camera
ros2 launch realsense2_camera rs_launch.py
```

(Reference: [Jinyong Jeong's blog](https://jinyongjeong.github.io/2020/06/20/Realsense-Ubuntu-driver-%EC%84%A4%EC%B9%98/))

### 2.1.4 Event Camera

A sensor with a different paradigm from conventional cameras. Instead of capturing frame by frame, each pixel asynchronously outputs an event only when a **brightness change** occurs.

Event-camera research has continued to expand around high-speed motion and HDR conditions, where frame cameras often struggle. Because an event sensor records per-pixel brightness changes asynchronously, it avoids the motion blur caused by frame exposure. If your work involves fast motion or HDR scenes, start with the Gallego et al. survey (TPAMI 2020) and the rpg_dvs_ros package.

**Event output format**:

```
(x, y, timestamp, polarity)
- x, y: pixel coordinates
- timestamp: time in microseconds
- polarity: brighter (+1) or darker (-1)
```

**Pros**:
- Very high temporal resolution (microseconds)
- High dynamic range (140dB vs. 60dB for a typical camera)
- Low power consumption, low latency
- No motion blur

**Cons**:
- No output for static scenes
- Difficult to apply traditional CV algorithms
- Relatively expensive

**Representative products**:
- Prophesee: high-resolution event sensors
- iniVation: DAVIS (simultaneous event + frame output)
- Samsung: mobile event sensor in development

> **Further reading**
> - [Davide Scaramuzza — Event Cameras: A Paradigm Shift for Computer Vision](https://www.youtube.com/watch?v=LauQ6LWTkxM) — Overview lecture by Prof. Scaramuzza, a pioneer in event cameras.
> - [Gallego et al. — Event-based Vision: A Survey (TPAMI 2020)](https://arxiv.org/abs/1904.08405) — Comprehensive survey of event camera technology. A good starting point for understanding this field.
> - [rpg_dvs_ros — Event Camera ROS driver](https://github.com/uzh-rpg/rpg_dvs_ros) — Open-source package for handling event cameras in ROS.

## 2.2 LiDAR

**LiDAR (Light Detection and Ranging)** is a sensor that measures distance using lasers. It directly produces a 3D point cloud.

Cameras record color and texture, but passive monocular images do not directly determine metric depth. LiDAR measures the range of each return and forms 3D points. Range and error depend on the model, surface reflectivity, incidence angle, atmosphere, sunlight, and return mode; some long-range automotive units specify ranges beyond 100 m under stated reflectivity conditions. This direct range measurement is why LiDAR is used in autonomous-driving systems.

Solid-state LiDAR is replacing spinning (mechanical) LiDAR in some applications. With no moving parts, it offers advantages in durability and mass production for automotive systems. Scan patterns such as Livox's non-repetitive design also differ from those of spinning sensors, and point-cloud processing algorithms must account for those differences.

### 2.2.1 2D LiDAR vs. 3D LiDAR

**2D LiDAR**:
- Single-plane scan
- Use cases: indoor robot navigation, obstacle avoidance
- Examples: SICK TiM, Hokuyo URG, RPLIDAR

**3D LiDAR**:
- Generates 3D point clouds via multiple layers or rotational scanning
- Use cases: autonomous driving, large-scale mapping
- Examples: Velodyne VLP-16/32/64, Ouster OS1, Hesai

> **Further reading**
> - [Cyrill Stachniss — LiDAR-based SLAM](https://www.youtube.com/watch?v=vrdlk2p9AZI) — Explains the principles of SLAM using LiDAR data.
> - [PCL (Point Cloud Library) official tutorials](https://pcl.readthedocs.io/projects/tutorials/en/latest/) — a widely used public point-cloud processing library.

### 2.2.2 Spinning vs. Solid-State

**Spinning (mechanical)**:
- Laser and receiver rotate
- Provides 360° FoV
- Cons: durability issues due to moving parts
- Examples: Velodyne, Ouster

**Solid-State**:
- No moving parts
- Limited FoV (usually under 120°)
- Pros: high durability, potential for low cost
- Examples: Livox (non-repetitive scan pattern), Innoviz

The difference directly affects algorithm design. Spinning LiDAR produces a uniform 360° point cloud, so existing SLAM algorithms (LOAM, LeGO-LOAM, etc.) were designed on that assumption. Solid-state LiDAR changes the scan pattern significantly and forces algorithm changes. That is why FAST-LIO2 and similar algorithms, targeting Livox's non-repetitive scans, have emerged.

### 2.2.3 Key specifications

| Specification | Description |
| --- | --- |
| Channels | Number of vertical layers (16, 32, 64, 128) |
| Range | Maximum measurement distance (50m ~ 300m) |
| Points/sec | Points per second (300K ~ 2M) |
| Accuracy | Measurement accuracy (±2cm ~ ±5cm) |
| FoV | Horizontal/vertical field of view |

> **Further reading**
> - [Livox technical documents](https://www.livoxtech.com/downloads) — Technical material explaining the non-repetitive scan pattern of solid-state LiDAR and its advantages.
> - [Xu et al. — FAST-LIO2 (RA-L 2022)](https://arxiv.org/abs/2107.06829) — A LiDAR-inertial odometry paper optimized for solid-state LiDAR.

## 2.3 IMU (Inertial Measurement Unit)

An IMU is a sensor that measures motion using inertia.

When SLAM drifts badly, without understanding IMU characteristics you cannot even identify the cause. Questions like "is the IMU bias being properly corrected?" or "can this grade of IMU deliver this level of accuracy?" require a proper grasp of the IMU error model. In visual-inertial odometry (VIO) or LiDAR-inertial odometry (LIO), the IMU fills the gaps between camera/LiDAR frames, and filling that role correctly demands knowing the limits of IMU data.

### 2.3.1 Components

**Accelerometer**:
- Measures 3-axis linear acceleration (m/s²)
- Includes gravitational acceleration

**Gyroscope**:
- Measures 3-axis angular velocity (rad/s or deg/s)
- Detects rotational speed

**Magnetometer** (on some IMUs):
- Measures 3-axis magnetic field
- Can estimate absolute heading
- Vulnerable to magnetic field distortion

### 2.3.2 Key error characteristics

If you cannot model IMU errors, the entire sensor fusion system wobbles.

**Bias**:
- Nonzero output even at rest
- Changes with temperature (bias instability)

**Noise**:
- High-frequency random noise
- Characterized by Allan variance

**Integration drift**:
- Double integration of acceleration → accumulated position error
- Integration of angular velocity → accumulated orientation error
- Trustworthy only for a short time (usually a few seconds)

Double-integrating acceleration accumulates noise, bias, scale-factor error, and initial-attitude error. The growth depends on the sensor, motion, calibration, temperature, and initialization, so no single elapsed time characterizes it. Unaided position from a low-cost MEMS IMU can quickly exceed a long-term position-error budget; systems that need sustained position therefore use external observations from a camera, LiDAR, GNSS, or another reference to limit drift.

**IMU grades** are not a single standardized price ladder. Compare bias stability, noise density, scale-factor error, temperature calibration, vibration tolerance, and certification requirements. Consumer MEMS prioritizes size and power; products described as industrial, tactical, or navigation grade generally add long-term stability and calibration. For devices such as the `VN-100`, `MTi`, `KVH 1750`, and `HG1700`, compare current datasheets in common units and confirm them with Allan measurements.

> **Further reading**
> - [Probabilistic Robotics, Ch.5 — Robot Motion (Thrun, Burgard, Fox)](https://www.probabilistic-robotics.org/) — A leading reference on sensor noise modeling and motion models. Covers the theoretical basis of IMU error modeling.
> - [Titterton & Weston — Strapdown Inertial Navigation Technology](https://ieeexplore.ieee.org/book/5765860) — The textbook on IMU principles and inertial navigation.
> - [Cyrill Stachniss — IMU and Inertial Navigation](https://www.youtube.com/watch?v=uHbRKvD8TWg) — Explains IMU operating principles and error characteristics visually.
> - [Allan Variance — IMU noise analysis guide (Vectornav)](https://www.vectornav.com/resources/inertial-navigation-primer/specifications--background/specifications--allan-variance) — How to extract IMU noise parameters using Allan variance.
> - [Jinyong Jeong's blog — IMU Filter (AHRS)](https://jinyongjeong.github.io/2020/01/10/IMU_filter/) — Overview of AHRS filters for IMU sensors. Introduces the Madgwick filter and ROS packages.

## 2.4 GPS/GNSS

**GNSS (Global Navigation Satellite System)** is a position measurement system using satellite signals. GPS is the U.S. system, and GNSS is the umbrella term covering GPS, GLONASS (Russia), Galileo (Europe), BeiDou (China), and others.

GNSS provides outdoor autonomous vehicles and drones with coordinates in an Earth-fixed frame. SLAM estimates position relative to a starting point, whereas GNSS expresses location as latitude, longitude, and altitude. Outdoor robots combine these frames, and RTK-GPS measurements are also used as ground truth in high-precision localization evaluations.

**Interpreting accuracy**: a standalone code solution is commonly meter-class in open sky, while differential corrections may produce sub-meter or meter-class results depending on the setup. RTK can reach horizontal centimeter-class results when a short baseline, sufficient satellite geometry, low multipath, and fixed ambiguities are maintained. Any quoted number should state the metric (CEP, RMS, or 95%), horizontal versus vertical component, baseline, correction link, and fix state.

**RTK-GPS principle**:
- A fixed base station provides correction data
- The rover receives the correction data to improve accuracy
- Requires real-time communication (radio or internet)

**Limitations**:
- Unusable indoors, in tunnels, and in urban canyons
- Multipath errors (building reflections)
- Altitude accuracy is lower than horizontal

> **Further reading**
> - [Cyrill Stachniss — Robot Localization Overview](https://www.youtube.com/watch?v=8VJ-A9OlhAE) — Overview of the principles and methods of robot localization.
> - [u-blox GNSS guide](https://www.u-blox.com/en/technologies/gnss) — A practical guide from GNSS basics to RTK.

## 2.5 Other sensors

**Radar**

Autonomous-driving and robotics systems use radar alongside cameras and LiDAR. Radio waves can be more robust than visible light and some LiDAR wavelengths in fog, rain, dust, and backlight, but rain attenuation, clutter, multipath, wet radomes, and limited angular resolution remain. Prices overlap with LiDAR product families as antenna count, bandwidth, imaging capability, and automotive qualification change, so compare current quotations for the required configuration.

**FMCW (Frequency Modulated Continuous Wave) Radar**:
- Transmits a frequency modulated over time and uses the frequency difference with the reflected wave to measure both range and velocity simultaneously.
- Output: range-Doppler map (distance × velocity 2D map), range-azimuth map
- 77 GHz automotive radar is the most common.

**Applications in robotics**:
- Autonomous driving: forward collision detection, adaptive cruise control (ACC)
- Radar odometry: estimating ego-motion from radar alone
- Radar SLAM: radar-based map building + localization

**Comparison with camera/LiDAR**:

| Characteristic | Camera | LiDAR | Radar |
|------|--------|-------|-------|
| Resolution | Very high | High | Low |
| Range measurement | Not possible (monocular) | Accurate | Possible |
| Velocity measurement | Not possible | Not possible (directly) | Possible (Doppler) |
| Adverse weather | Weak | Weak (rain, fog) | Robust |
| Price | Cheap | Expensive | Medium |
| Nighttime | Not possible | Possible | Possible |

**Representative products**: Texas Instruments AWR1843, Continental ARS548, Navtech CTS350-X (spinning radar)

> **Further reading**
> - [Giseop Kim's blog — ICRA 2021 Radar in Robotics Workshop summary](https://gisbi-kim.github.io/blog/2021/05/31/icra21-radar-ws.html) — Overall trends in radar robotics.
> - [Giseop Kim's blog — Radar Odometry Results on MulRan dataset](https://gisbi-kim.github.io/blog/2021/05/30/yeti-radar-odom-mulran1.html) — Radar odometry experimental results. LiDAR-level performance in urban environments.
> - [Kim et al., "MulRan: Multimodal Range Dataset for Urban Place Recognition" (ICRA 2020)](https://sites.google.com/view/mulran-pr/home) — LiDAR + radar + GPS multimodal dataset.

**Ultrasonic**:
- Detects obstacles at short range (0.2-5m)
- Low cost
- Parking assistance, proximity sensing

**Wheel encoder**:
- Measures wheel rotation
- Position estimation via dead reckoning
- Vulnerable to slip

Classifying these as "other" does not make them unimportant. Radar acts as the safety net in adverse weather where LiDAR fails in autonomous driving, and the wheel encoder is the most basic odometry source for ground robots. In sensor fusion, such "auxiliary" sensors determine the robustness of the whole system.

> **Further reading**
> - [Probabilistic Robotics, Ch.6 — Robot Perception (Thrun, Burgard, Fox)](https://www.probabilistic-robotics.org/) — Thoroughly covers probabilistic models of various sensors. The textbook on sensor modeling.

## 2.6 Sensor Fusion

Since each single sensor has its own limits, multiple sensors are combined to complement each other.

A single sensor cannot cover every combination of illumination, range, occlusion, and drift. Autonomous-vehicle sensor suites vary with the vehicle and operating conditions, combining several of camera, LiDAR, radar, IMU, and GNSS. When, where, and how the selected sensors are fused determines system performance.

**Why is it needed?**

| Sensor | Pros | Cons |
| --- | --- | --- |
| Camera | Rich information, cheap | Lighting-dependent, no depth |
| LiDAR | Accurate 3D, lighting-independent | Expensive, sparse |
| IMU | High frequency, lighting-independent | Drift |
| GPS | Global position | Outdoor only, low frequency |

**Fusion approaches**:
1. **Early Fusion**: combine at the raw data level
2. **Late Fusion**: combine the results from each sensor
3. **Mid-Level Fusion**: combine at the feature level

Each approach has trade-offs. Early fusion loses less information but is computationally expensive; late fusion allows each sensor to be processed independently, which helps modularity, but some information is lost. Mid-level fusion sits in between and is heavily used in recent deep-learning-based fusion.

**Representative combinations**:
- Camera + IMU → VIO (visual-inertial odometry)
- LiDAR + IMU → LIO (LiDAR-inertial odometry)
- Camera + LiDAR + IMU → multimodal SLAM

The probabilistic foundation of fusion — how sensor measurements are formally expressed as likelihoods — is covered in §2.7 Measurement Models.

> **Further reading**
> - [State Estimation for Robotics (Tim Barfoot) — free PDF](http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) — A leading textbook covering the mathematical foundations of sensor fusion. Covers both Kalman filter and factor-graph-based estimation.
> - [Cyrill Stachniss — Kalman Filter & EKF](https://www.youtube.com/watch?v=E-6paM_Iwfc) — Explains the Kalman filter and EKF, the core of sensor fusion.
> - [Qin et al. — VINS-Mono (TRO 2018)](https://arxiv.org/abs/1708.03852) — A representative paper on visual-inertial fusion. Shows how a real VIO system is implemented.

---

## 2.7 Deep Dive: Measurement Models — Probabilistic Formulation

Bayes filters, SLAM, and MCL need a number that says "how much to trust this reading" every time sensor data arrives. That number is the measurement model $p(z_t \mid x_t, m)$.

### 2.7.1 The Distribution a Sensor Produces

Fire a laser range sensor at the same wall from the same pose one hundred times, and the hundred measurements differ. Reflectance angle, a passing person, and multi-path reflections each leave a different variance signature. The probability distribution $p(z_t^k \mid x_t, m)$ captures that variance structure. $z_t^k$ is the $k$-th beam reading at time $t$, $x_t$ is the robot pose, and $m$ is the map.

A single scan contains tens to hundreds of beams. PR §6.2 assumes the error on each beam is independent (conditional independence assumption); under that assumption the full-scan likelihood is the product of the per-beam likelihoods:

$$p(z_t \mid x_t, m) = \prod_{k=1}^{K} p(z_t^k \mid x_t, m)$$

This conditional independence assumption does not fully hold in practice. Adjacent beams looking at the same wall are correlated, and ignoring that correlation concentrates the likelihood too sharply around a particular pose. This is revisited in §2.7.8.

The map $m$ comes in two forms. A feature-based map is a list of landmarks indexed by ID. A location-based map is an array of occupancy probabilities over grid cells, indexed by coordinate. The four measurement model families each depend on one of these two map types.

The four measurement model families:
- Beam model: a mixture that approximates possible causes of a measurement. Location-based map.
- Likelihood field: endpoint-to-nearest-obstacle distance. Location-based map.
- Correlation-based (map matching): normalized correlation between local and global map.
- Feature-based (landmark model): extracted features modeled as (range, bearing, signature). Feature-based map.

### 2.7.2 Beam Model — Four-Component Mixture

A range reading is approximated using four hypotheses about how it was produced. [Thrun et al. 2005](https://www.probabilistic-robotics.org/) (PR §6.3.1) assigns a probability distribution to each hypothesis and builds the likelihood as a weighted mixture. These components are terms in an observation model, not independent physical channels inside the sensor.

The most frequent component is **hit** — the beam actually detects the obstacle. A truncated Gaussian centered on the predicted range $z_t^{k*}$ with variance $\sigma_{\text{hit}}^2$ models this. The truncation removes probability mass outside $[0, z_{\max}]$.

$$p_{\text{hit}}(z_t^k \mid x_t, m) = \eta\, \mathcal{N}(z_t^k;\, z_t^{k*},\, \sigma_{\text{hit}}^2), \quad 0 \le z_t^k \le z_{\max}$$

**short (unexpected nearby obstacle)**: An unmapped obstacle — a passing person, another robot — blocks the beam. The reading is always shorter than $z_t^{k*}$. An exponential distribution over $[0, z_t^{k*}]$ models this.

$$p_{\text{short}}(z_t^k \mid x_t, m) = \eta\, \lambda_{\text{short}}\, e^{-\lambda_{\text{short}} z_t^k}, \quad 0 \le z_t^k \le z_t^{k*}$$

**max (maximum-range failure)**: Dark surfaces, mirror angles, and fog cause the return signal to vanish. The sensor outputs $z_{\max}$ directly. A Dirac delta at $z_{\max}$ models this.

$$p_{\text{max}}(z_t^k \mid x_t, m) = \mathbf{1}[z_t^k = z_{\max}]$$

**rand (unexplained noise)**: Sonar crosstalk, multi-path, and other unknown sources produce readings with no identifiable cause. A uniform distribution over $[0, z_{\max}]$ models this.

$$p_{\text{rand}}(z_t^k \mid x_t, m) = \frac{1}{z_{\max}}$$

The final likelihood is the weighted mixture of the four components (PR Eq. 6.13):

$$p(z_t^k \mid x_t, m) = \begin{pmatrix} z_{\text{hit}} \\ z_{\text{short}} \\ z_{\text{max}} \\ z_{\text{rand}} \end{pmatrix}^T \cdot \begin{pmatrix} p_{\text{hit}}(z_t^k \mid x_t, m) \\ p_{\text{short}}(z_t^k \mid x_t, m) \\ p_{\text{max}}(z_t^k \mid x_t, m) \\ p_{\text{rand}}(z_t^k \mid x_t, m) \end{pmatrix}$$

The weights must sum to one: $z_{\text{hit}} + z_{\text{short}} + z_{\text{max}} + z_{\text{rand}} = 1$.

The predicted range $z_t^{k*}$ is computed from pose $x_t$ and map $m$ by ray casting: follow the beam direction until it hits the first occupied cell; that distance is $z_t^{k*}$. (Ray casting applies the same geometric principle seen in §2.1's camera projection model and §2.2's LiDAR beam structure, now to an occupancy grid.)

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

### 2.7.3 Beam Model — EM Parameter Learning

Every time the sensor type, environment configuration, or mounting position changes, the hit/short/max/rand ratios and variances shift. Setting parameters by hand produces values that fit one environment and drift in another.

The four-component mixture has six intrinsic parameters: $z_{\text{hit}}, z_{\text{short}}, z_{\text{max}}, z_{\text{rand}}, \sigma_{\text{hit}}, \lambda_{\text{short}}$. PR §6.3.2 estimates these by maximum likelihood using the EM algorithm on data $\{(z_t^k, z_t^{k*})\}$ collected while the robot navigates a known environment.

The EM formulation introduces a correspondence variable. For each measurement $z_t^k$, the latent variable $c_i \in \{\text{hit, short, max, rand}\}$ indicates which component generated the value.

**E-step**: Use the current parameter estimates to compute the expected value of $c_i$. For each measurement, compute the posterior probability of each of the four components (PR Eq. 6.15–6.32):

$$e_{\text{hit}}^i = \frac{z_{\text{hit}} \cdot p_{\text{hit}}(z^i \mid z^{i*})}{p(z^i \mid z^{i*})}, \quad e_{\text{short}}^i = \frac{z_{\text{short}} \cdot p_{\text{short}}(z^i \mid z^{i*})}{p(z^i \mid z^{i*})}, \quad \dots$$

**M-step**: Update the parameters using the expectations from the E-step. Closed-form solutions exist for $\sigma_{\text{hit}}$ and $\lambda_{\text{short}}$:

$$\sigma_{\text{hit}}^2 = \frac{\sum_i e_{\text{hit}}^i (z^i - z^{i*})^2}{\sum_i e_{\text{hit}}^i}, \qquad \lambda_{\text{short}} = \frac{\sum_i e_{\text{short}}^i}{\sum_i e_{\text{short}}^i \cdot z^i}$$

The mixture weights $z_{\text{hit}}, z_{\text{short}}, z_{\text{max}}, z_{\text{rand}}$ are updated as the fraction of measurements assigned to each component.

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

EM estimates these parameters for the sensor, map, and environment represented in its training data. AMCL implementations expose `sigma_hit`, `lambda_short`, and the mixture weights as configurable values, but package defaults should not be interpreted as universal EM convergence values across environments.

Once parameters are in hand, putting the model to work in a real system needs a few more practical adjustments.

### 2.7.4 Beam Model — Practical Considerations

The main computational bottleneck of the beam model is ray casting. In MCL, running ray casting for every beam of every particle requires (number of particles) × (number of beams) operations.

One option is to reduce the beam count. A uniformly spaced subset lowers computation and removes some redundancy between adjacent beams. The number of beams still needs validation against scan resolution, environment structure, and particle count.

**Exponentiation correction $p^{\alpha}$**: When the conditional independence assumption is violated, the likelihood $p(z_t \mid x_t, m)$ can become overconfident, concentrating too sharply. Replacing it with $p(z_t \mid x_t, m)^{\alpha}$ ($0 < \alpha < 1$) reduces each beam's contribution and flattens the distribution. $\alpha$ is set empirically or by cross-validation.

**Precomputed range table**: Precomputing ray casting results for all (cell, direction) combinations in the map and storing them in a table makes range lookup $O(1)$ at runtime. Memory cost is high for large maps, but the approach is practical for real-time MCL (see Ch.3 §3.11 on particle filters).

### 2.7.5 Likelihood Field

Two weaknesses of the beam model cause problems in real systems. First, ray casting is expensive. Second, a small change in pose $x_t$ can cause a beam to hit a different obstacle first, making $z_t^{k*}$ jump discontinuously. The likelihood is discontinuous with respect to pose, which interferes with gradient-based scan matching and hill-climbing optimization.

The likelihood field drops ray casting entirely. It transforms the beam endpoint into global coordinates, then evaluates the likelihood using the Euclidean distance $\text{dist}$ from that endpoint to the nearest occupied cell in the map.

Beam endpoint transformation to global coordinates (PR Eq. 6.33):

$$\begin{pmatrix} x_{z_t^k} \\ y_{z_t^k} \end{pmatrix} = \begin{pmatrix} x \\ y \end{pmatrix} + \begin{pmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{pmatrix} \begin{pmatrix} x_{k,\text{sens}} \\ y_{k,\text{sens}} \end{pmatrix} + z_t^k \begin{pmatrix} \cos(\theta + \theta_{k,\text{sens}}) \\ \sin(\theta + \theta_{k,\text{sens}}) \end{pmatrix}$$

Here $(x_{k,\text{sens}}, y_{k,\text{sens}})$ is the position of the $k$-th beam's sensor in the robot frame, and $\theta_{k,\text{sens}}$ is the beam's angular offset.

Beam likelihood (PR Eq. 6.34–6.35):

$$p(z_t^k \mid x_t, m) = z_{\text{hit}} \cdot \mathcal{N}(\text{dist};\, 0,\, \sigma_{\text{hit}}^2) + z_{\text{rand}} \cdot \frac{1}{z_{\max}}$$

Here $\text{dist}$ is the Euclidean distance from the beam endpoint to the nearest occupied cell, and the Gaussian models the distance error as zero-mean. Max-range beams ($z_t^k = z_{\max}$) are ignored in this model: projecting their endpoint is meaningless.

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

When the map is fixed, the distance transform can be computed once and stored as a table, making every $\text{dist}$ lookup $O(1)$. The table corresponds to the positive region of an SDF (Signed Distance Field). The likelihood is differentiable with respect to pose $x_t$, which makes it suitable for gradient-based scan matching.

The model has limits. It does not model dynamic obstacles explicitly (no short component). It can "see through walls" — a beam endpoint landing in free space on the far side of a wall produces a large $\text{dist}$, not an impossibility signal. Occlusion is absent. Map uncertainty is ignored.

In 2D LiDAR indoor navigation, AMCL uses the likelihood field as its default — not the beam model — because it is faster and produces a pose-continuous likelihood. For 3D LiDAR and RGB-D, ICP and NDT have taken over that role (Ch.3 §3.10; see also Ch.14 §14.7 for Kalman-filter integration).

When the goal is not pose estimation but loop closure detection — quickly deciding whether two maps cover the same place — even faster methods are needed, even if they sacrifice probabilistic rigor.

### 2.7.6 Correlation-Based Model (Map Matching)

The correlation-based model is the most ad hoc of the four. It builds a local map $m_{\text{local}}$ from a recent set of scans, then compares it to the global map $m$ using the normalized correlation coefficient $\rho$. PR §6.5 uses this comparison directly as the likelihood:

$$p(m_{\text{local}} \mid x_t, m) = \max\{\rho(m_{\text{local}}, m \mid x_t),\ 0\}$$

$\rho$ is the Pearson correlation coefficient between corresponding cells when the two maps are aligned by $x_t$. Computation is fast and the implementation is simple. The weakness: this likelihood has no probabilistic justification. $\rho$ is normalized, and values below zero are simply clipped. It is used in settings like loop closure detection where a fast similarity score matters more than a proper likelihood.

Unlike the three models above, which work with raw range measurements directly, the final model works with structured features extracted from sensor data.

### 2.7.7 Feature-Based Measurement — Landmark Model

The beam model and likelihood field work with raw range measurements. The landmark model works with features $f(z_t)$ extracted from sensor data. Inference over low-dimensional features is cheaper, and the model pairs naturally with feature-based maps.

Feature extraction takes different forms depending on the sensor. From range scans: line segments, corners, local minima. From cameras: edges, corners, SIFT/ORB-style local patterns (see §2.1.1 on monocular camera texture, and §2.6 for VIO/Visual SLAM). Each extracted feature is represented as a triple $(r, \phi, s)$: $r$ is range, $\phi$ is bearing, and $s$ is signature (ID, color, descriptor, etc.).

The $j$-th landmark in the map sits at $(m_{j,x}, m_{j,y})$ with signature $s_j$. From pose $x_t = (x, y, \theta)^T$, the relationship between predicted and observed measurement is (PR Eq. 6.41):

$$\begin{pmatrix} r_t^i \\ \phi_t^i \\ s_t^i \end{pmatrix} = \begin{pmatrix} \sqrt{(m_{j,x} - x)^2 + (m_{j,y} - y)^2} \\ \operatorname{atan2}(m_{j,y} - y,\, m_{j,x} - x) - \theta \\ s_j \end{pmatrix} + \begin{pmatrix} \varepsilon_{\sigma_r^2} \\ \varepsilon_{\sigma_\phi^2} \\ \varepsilon_{\sigma_s^2} \end{pmatrix}$$

$\varepsilon_{\sigma^2}$ denotes zero-mean Gaussian noise with variance $\sigma^2$. The three channels carry independent Gaussian noise. Adding Gaussian noise directly to the bearing channel $\varepsilon_{\sigma_\phi^2}$ can produce wrap-around errors near $\pm\pi$; in real implementations, angular differences are normalized to $[-\pi, \pi]$ or modeled with the von Mises distribution instead.

When the correspondence $c_t^i = j$ (the $i$-th feature corresponds to the $j$-th landmark) is known, the likelihood is the product of Gaussians over the three channels.

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

Assuming conditional independence across features in the full scan, the full scan likelihood is $\prod_i$.

**Reverse direction — pose sampling** (condensed from PR Table 6.5): the model can also run in reverse, sampling possible poses from a measurement. A single $(r, \phi)$ reading gives only two constraints in pose space; the set of compatible poses lies on a circle (in 2D) or a helix (in 3D) around the landmark. A free parameter $\hat{\gamma} \sim U(0, 2\pi)$ samples positions on that circle. This is the geometric explanation for why a single observation of one landmark is not enough to determine position.

<!-- DEMO: landmark_donut.html -->

The reprojection residual $\| \pi(K[R|t]\, X_w) - u \|^2_\Sigma$ used in visual SLAM has the same broad structure: predict an observation from pose and landmark, then compare it with the measurement. The pixel reprojection model and the range-bearing model are nevertheless different sensor models, and ORB/SIFT descriptors are normally used for data association rather than as a continuous signature term in the likelihood. AprilTag and ArUco IDs reduce correspondence ambiguity substantially, but they do not rule out false detections or misread IDs.

### 2.7.8 Practical Summary: Choosing a Model

The four families can be compared qualitatively as follows. Accuracy and speed depend on the sensor, map resolution, implementation, and parameters.

| Model | Accuracy | Speed | Differentiable | Primary use |
|------|--------|-----------|------------|-----------|
| Beam model | High | Slow (ray casting) | Low (discontinuous) | MCL high-fidelity, diagnostics |
| Likelihood field | Medium | Fast (DT lookup) | High | AMCL default, gradient matching |
| Correlation-based | Low | Very fast | Low | Loop closure detection |
| Landmark model | High (feature-dependent) | Fast (low-dimensional) | High | Visual SLAM, fiducial |

One more practical concern is over-confidence. When the conditional independence assumption is violated, $p(z_t \mid x_t, m)$ can become too sharply peaked at a particular pose. Tempering it as $p(z_t \mid x_t, m)^{\alpha}$ ($\alpha < 1$), as in §2.7.4, reduces each scan's influence and flattens the distribution. Beam subsampling, models that account for correlation, and robust likelihoods are alternatives; $\alpha$ must be chosen with calibration or validation data rather than treated as a universal constant.

With the model limits and mitigations understood, the natural next question is which of these models survived into production systems.

### 2.7.9 What Survived

The four families in PR §6 remain useful for classifying and designing systems. It would be misleading, however, to label every modern scan matcher or visual SLAM method a **direct descendant** of them. Methods can share the broad observation-versus-prediction structure while using different objectives and map representations.

The [Nav2 AMCL documentation](https://docs.nav2.org/configuration/packages/configuring-amcl.html) exposes three laser models: `beam`, `likelihood_field`, and `likelihood_field_prob`; its default is `likelihood_field`. `max_beams` selects an evenly spaced subset of a scan. By contrast, the `beam_skip_*` parameters belong to `likelihood_field_prob` and skip beams that disagree with many particles, so they are not ordinary beam subsampling. Defaults such as `sigma_hit` and `lambda_short` are implementation starting points, not universally converged values traceable to one EM experiment.

Other LiDAR systems use distinct matching objectives. Cartographer combines correlative scan matching on a probability grid with nonlinear optimization, while `hdl_localization` uses NDT/GICP-family registration on 3D point clouds. They share the broad idea of comparing an observation against a map, but they do not all implement a likelihood-field distance transform. ESDF planners and neural implicit maps also use distance or implicit fields; similarity in data structure alone does not establish descent from the likelihood-field sensor model.

The same distinction applies to landmarks and visual SLAM. The range-bearing model in Eq. 6.41, camera reprojection models, and DROID-SLAM's dense bundle adjustment all predict observations from pose and scene structure and form residuals. Their measurement spaces, association procedures, noise models, and optimization variables differ, so one should not assert a single historical lineage among them. Fiducial IDs simplify association but do not eliminate false detections.

`hit`, `short`, `max`, and `rand` are mixture components that approximate possible causes of a range measurement, not independent physical channels. The choice of components and map representation depends on the sensor, environment, outliers, compute budget, and calibration data. These four families are a starting point for comparison, not a genealogy that covers every modern system.

Ch.14 §14.7 shows how `beam_range_finder_model` is called inside MCL and how `inverse_sensor_model` connects to occupancy mapping — where these models sit in the full pipeline.

> **⚠ Sensor connection check**: When sensor data is missing, inspect the cable, IP configuration, power, and USB bandwidth as well as the driver. Commands such as `dmesg`, `lsusb`, and `ping` record the device and connection state and help narrow the fault.

> **Technical Timeline: sensor technology**
> - **~2010**: 2D LiDAR and frame cameras were common in mobile-robot research. The practical range of real-time stereo depended strongly on the available compute and scene conditions.
> - **2010s**: RGB-D cameras and compact multi-beam 3D LiDAR broadened the options for indoor 3D perception and outdoor mapping. Camera–IMU VIO also moved beyond research prototypes into several robotics and AR systems.
> - **Late 2010s to early 2020s**: Non-repetitive scanning, MEMS, and flash architectures appeared under the broad label `solid-state LiDAR`; research in event cameras and automotive imaging radar also expanded. Price and performance trends varied too much across product classes to summarize with one number.
> - **2020s**: Spinning and solid-state LiDAR coexist because field of view, range, resolution, motion distortion, and cost impose different trade-offs. Adoption of event cameras and Doppler radar is likewise application-dependent, including high-speed, HDR, and adverse-weather settings.
> - **Design implication**: A change in scan pattern or timestamp structure changes assumptions used by deskewing, calibration, and data association. Inspect the actual sampling geometry and noise characteristics instead of relying on the hardware category name.
