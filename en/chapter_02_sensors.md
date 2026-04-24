# Ch.2 — Sensors


A robot needs sensors to perceive its environment. Understanding each sensor's characteristics enables proper sensor selection and algorithm design.

No matter how well an algorithm is written, without knowing sensor characteristics you cannot diagnose "why does this algorithm fail here?" For example, when SLAM loses tracking in a particular segment, telling apart rolling shutter, LiDAR reflectance, and IMU bias as the cause requires sensor knowledge. Sensors are the entry point of a robotics system; if you do not understand the data coming in at the entry, everything downstream wobbles.

## 2.1 Camera

The camera is the most information-rich sensor. Just as humans understand most of the world through vision, robots also get the most information from cameras. Camera types differ widely in their characteristics, so understanding the trade-offs of each and picking the right one for the task matters.

### 2.1.1 Monocular Camera

The most basic visual sensor, capturing a 2D image with a single lens.

A monocular camera is the cheapest and lightest sensor, and it is also the starting point for most vision tasks such as Visual SLAM, object recognition, and semantic understanding. It cannot measure depth directly, and various algorithms (monocular depth estimation, SfM, etc.) have been developed to work around this structural limit. Understanding this limit is also what makes clear why stereo cameras or depth cameras are needed.

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

Event camera papers at CVPR have grown from around 5 in 2019 to over 30 in 2024, because they operate without motion blur in high-speed settings (drone high-speed flight, sharp vehicle turns). They are not yet mainstream, but if you plan to work with high-speed (>100 km/h) environments or HDR conditions, look at the Gallego et al. survey (TPAMI 2020) and the rpg_dvs_ros package.

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

If cameras give "rich but depth-less" data, LiDAR gives "accurate 3D coordinates directly". This precise ranging is exactly why LiDAR became a core sensor in autonomous driving. Depth estimated from cameras alone carries large errors and depends on weather, while LiDAR measures objects over 100m away with centimeter-level accuracy.

A recent trend worth noting: solid-state LiDAR is rapidly replacing spinning (mechanical) LiDAR. No moving parts means higher durability and easier mass production, which fits automotive volume manufacturing. New approaches such as Livox's non-repetitive scan pattern are also emerging, so point cloud processing algorithms need to change as well.

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
> - [PCL (Point Cloud Library) official tutorials](https://pcl.readthedocs.io/projects/tutorials/en/latest/) — The de facto standard library for point cloud processing.

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

You feel this immediately in practice: double-integrating acceleration to recover position accumulates noise and bias errors proportionally to time squared. With a consumer-grade IMU (as built into smartphones), position error can reach several meters after only 10 seconds. That is why IMUs are almost never used alone; they are always fused with a camera or LiDAR to correct drift.

**IMU grades**:
| Grade | Use | Price | Examples |
|------|------|------|------|
| Consumer | Smartphones, games | $1-10 | MPU6050, BMI160 |
| Industrial | Robots, drones | $100-1K | VectorNav VN-100, Xsens MTi |
| Tactical | Autonomous driving, aviation | $1K-10K | KVH 1750 |
| Navigation | Ships, aircraft | $10K+ | Honeywell HG1700 |

> **Further reading**
> - [Probabilistic Robotics, Ch.5 — Robot Motion (Thrun, Burgard, Fox)](https://www.probabilistic-robotics.org/) — A leading reference on sensor noise modeling and motion models. Covers the theoretical basis of IMU error modeling.
> - [Titterton & Weston — Strapdown Inertial Navigation Technology](https://ieeexplore.ieee.org/book/5765860) — The textbook on IMU principles and inertial navigation.
> - [Cyrill Stachniss — IMU and Inertial Navigation](https://www.youtube.com/watch?v=uHbRKvD8TWg) — Explains IMU operating principles and error characteristics visually.
> - [Allan Variance — IMU noise analysis guide (Vectornav)](https://www.vectornav.com/resources/inertial-navigation-primer/specifications--background/specifications--allan-variance) — How to extract IMU noise parameters using Allan variance.
> - [Jinyong Jeong's blog — IMU Filter (AHRS)](https://jinyongjeong.github.io/2020/01/10/IMU_filter/) — Overview of AHRS filters for IMU sensors. Introduces the Madgwick filter and ROS packages.

## 2.4 GPS/GNSS

**GNSS (Global Navigation Satellite System)** is a position measurement system using satellite signals. GPS is the U.S. system, and GNSS is the umbrella term covering GPS, GLONASS (Russia), Galileo (Europe), BeiDou (China), and others.

For outdoor autonomous driving or drones, GNSS is the only sensor that provides "global coordinates". SLAM estimates relative position (how far you have moved from where you started), while GNSS gives absolute position on Earth (latitude, longitude, altitude). Combining these two is the core challenge of outdoor robotics. RTK-GPS's centimeter-level accuracy is also used as ground truth for high-precision autonomous driving localization, so the principles are worth knowing.

**Accuracy**:
- Standard GPS: 2-5m
- DGPS (Differential): 0.5-2m
- RTK-GPS (Real-Time Kinematic): 1-2cm

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

Radar is growing in importance in autonomous driving and robotics. In environments where LiDAR and cameras fail — fog, rain, dust, strong backlight — radar still operates reliably. It is also cheaper than LiDAR.

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

In the real world, perfect perception with a single sensor is impossible. Autonomous vehicles use camera, LiDAR, radar, IMU, and GNSS all at once, and when, where, and how the data from these sensors are combined determines system performance.

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

> **Further reading**
> - [State Estimation for Robotics (Tim Barfoot) — free PDF](http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) — A leading textbook covering the mathematical foundations of sensor fusion. Covers both Kalman filter and factor-graph-based estimation.
> - [Cyrill Stachniss — Kalman Filter & EKF](https://www.youtube.com/watch?v=E-6paM_Iwfc) — Explains the Kalman filter and EKF, the core of sensor fusion.
> - [Qin et al. — VINS-Mono (TRO 2018)](https://arxiv.org/abs/1708.03852) — A representative paper on visual-inertial fusion. Shows how a real VIO system is implemented.

> **⚠ Note for AI agents**: When a sensor "is not getting data", the cause is usually not software but a physical connection (cable, IP configuration, power, USB bandwidth). AI tends to suggest reinstalling the driver first, but check the physical connection first with system commands such as `dmesg`, `lsusb`, `ping`.

> **Technical Timeline: sensor technology**
> - **~2010**: Centered on 2D LiDAR (SICK, Hokuyo) and monocular cameras. Sensors were expensive and bulky, and processing power was limited. Stereo cameras were hard to run in real time due to computational cost.
> - **2012~2017**: 3D LiDAR (Velodyne VLP-16) became widespread, and RGB-D cameras (Kinect) reached the mass market. LiDAR prices dropped from tens of thousands to thousands of dollars. Visual-inertial systems (VIO) also began to be used in real systems.
> - **2018~2022**: Solid-state LiDAR (Livox) appeared, with prices falling to the hundreds of dollars. Event camera research became more active. Multimodal sensor fusion (camera + LiDAR + IMU) became the standard.
> - **2023~**: Solid-state LiDAR is rapidly replacing the spinning type. Event camera adoption is starting to grow in high-speed/HDR applications. 4D radar (including Doppler velocity) is also emerging as a new auxiliary sensor.
> - **Worth watching now**: As solid-state LiDAR goes mainstream, algorithms built on the assumption of spinning LiDAR need to be redesigned. Event cameras are not yet mainstream, but they are being adopted quickly in fields where the limits of conventional cameras are clear, such as high-speed drones and autonomous driving. When sensor hardware changes, algorithm research directions follow.
