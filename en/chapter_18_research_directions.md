# Ch.18 — Lab Research Directions

Our lab designs a Spatial AI system as two modules. This split is forced by physical constraints and real-time requirements, and it is where the concepts built up in earlier chapters come together.

## 18.1 Overview

We split a Spatial AI system into **two modules**.

```
┌──────────────────────────────────────────────────────────────┐
│                     Spatial AI System                         │
├──────────────────────────────────────────────────────────────┤
│  ┌─────────────────────┐     ┌─────────────────────────────┐ │
│  │   Local Module      │     │      Global Module          │ │
│  │   (lightweight,     │ ←→  │   (heavy, server/cloud)     │ │
│  │    on-board)        │     │                             │ │
│  │ • Real-time Geometry│     │ • VFM-based Understanding   │ │
│  │ • Odometry          │     │ • Semantic Scene Graph      │ │
│  │ • Local Obstacle    │     │ • Long-term Memory          │ │
│  │ • control-budget rate│    │ • task-budget rate          │ │
│  └─────────────────────┘     └─────────────────────────────┘ │
└──────────────────────────────────────────────────────────────┘
```

### Why the System Uses Two Modules

A single onboard computer cannot handle every function within the robot's weight, power, and response-time limits.

Start with the **physical constraints**. An NVIDIA A100 GPU server weighs tens of kilograms and draws hundreds of watts, so a battery-powered drone cannot carry one. Robots commonly use embedded boards such as the Jetson Orin, but these boards cannot run large models such as DINOv2 or SAM in real time.

The **time constraints** also differ. Obstacle avoidance must respond within tens of milliseconds, whereas semantic interpretation can run more slowly. The former cannot wait for a server response; the latter has room to use a larger model.

The two modules divide their roles accordingly:

1. **The reality of compute**: on-board computers on the robot (Jetson, etc.) do not have the headroom to run large models
2. **Real-time requirements**: obstacle avoidance needs a response within tens of milliseconds
3. **Semantic understanding**: VFMs and VLAs distinguish an object's class and state, such as identifying a broken glass
4. **Complementary roles**: combine geometric precision in the Local Module with semantic understanding in the Global Module

> By analogy, the Local Module provides the robot's **reflexes**, while the Global Module acts as its **cerebral cortex**. Touch a hot pot and you pull your hand away first (reflex), then think "ah, the stove was on" (cognition). Robots work the same way.

## 18.2 Local Module: Lightweight Geometry

The Local Module runs directly on the robot and processes the information needed for safe, real-time motion.

### 18.2.1 Goals

- **Odometry**: estimate the robot's own motion — "where am I right now?"
- **Obstacle Detection**: immediate obstacle sensing — "something is in front, dodge!"
- **Local Mapping**: a geometric map of the surroundings — "within 3 m of me, the world looks like this"

**Operating example**: when a child suddenly runs in front of a delivery robot in an apartment hallway, the Local Module detects the obstacle with a depth sensor, estimates pose through odometry, and computes an avoidance path within the deadline derived by the control and safety analysis. At this stage, collision risk matters before the object class. The Global Module interprets meaning separately.

### 18.2.2 Characteristics

- **Latency budget**: derive update rate and deadline from platform speed, braking distance, control bandwidth, and sensor rate
- **Resource budget**: choose the embedded module, power mode, and cooling for the measured workload
- **Timing evidence**: measure worst-case latency, jitter, and deadline misses as well as mean FPS on the target hardware

### 18.2.3 Tech Stack

**Classical methods**:
- ORB-SLAM3: feature-based Visual SLAM — pose estimation from a single camera (see Ch.9, Ch.14)
- VINS-Mono: Visual-Inertial Odometry — camera + IMU fusion (see Ch.14)
- FAST-LIO2: LiDAR-Inertial Odometry — LiDAR + IMU fusion (see Ch.2, Ch.14)

**Lightweight learning models**:
- Lightweight depth estimation — compressed with a MobileNet backbone (see Ch.10)
- Compressed segmentation models — knowledge distillation applied (see Ch.10, Ch.11)
- TensorRT optimization — a candidate for graph, kernel, and precision optimization on NVIDIA GPUs

**Edge deployment**:

```bash
# TensorRT optimization example
trtexec --onnx=model.onnx --saveEngine=model.trt --fp16 --workspace=4096
```

> TensorRT builds an inference engine for NVIDIA GPUs. FP16 can reduce memory and latency, but gains and task-metric changes depend on the model, input, batch, power mode, and software versions. Compare end-to-end latency and validation metrics on the target Jetson before adopting it.

### 18.2.4 Example Implementation

```python
# Local Module conceptual code
class LocalModule:
    def __init__(self):
        self.odometry = FastLIO2()
        self.obstacle_detector = LightweightObstacleNet()  # TensorRT

    def process(self, sensor_data):
        # 1. Odometry update (100 Hz)
        pose = self.odometry.update(sensor_data.imu, sensor_data.lidar)

        # 2. Obstacle detection (30 Hz)
        obstacles = self.obstacle_detector(sensor_data.image)

        # 3. Send keyframe to the Global Module
        if self.is_keyframe(pose):
            self.send_to_global(sensor_data, pose)

        return pose, obstacles
```

**Reading it as a scenario**: in the code above, `process()` is called every time sensor data arrives. IMU data comes in at 100 Hz (100 times per second), camera images at 30 Hz. Every frame, it computes "where am I now?" (odometry) and "what is in front?" (obstacle), and only at important moments (keyframes) sends data to the Global Module. Sending every frame would saturate the network.

## 18.3 Global Module: VFM-based Understanding

The Global Module runs on a server or in the cloud and interprets object classes and relations. When the Local Module detects an obstacle, the Global Module can classify it as a broken glass and connect it to a location in the scene graph.

### 18.3.1 Goals

- **Global map understanding**: grasp spatial structure and meaning — "this is the kitchen, that is the living room"
- **Semantic Scene Graph**: represent relations between objects — "the cup is on the table"
- **Long-term Memory**: track environmental changes — "yesterday there was no chair here, today there is"

**A real scenario**: a home service robot moves around the house every day and learns the environment. The Global Module maintains a high-level map like "the living room has a sofa, a TV, and a table; the kitchen has a refrigerator and a sink." When the user says "bring the remote from the living room table," it looks up the remote's location in the Scene Graph and hands a waypoint to the Local Module.

### 18.3.2 Characteristics

- **Model choice**: DINOv2 and SAM2 have variants of different sizes; they are not all billion-parameter models
- **Compute choice**: select hardware from the variant, input resolution, precision, scene count, and measured memory/latency
- **Update budget**: some global tasks can run outside the local control deadline, while interaction and change detection still need an explicit end-to-end latency budget

### 18.3.3 Tech Stack

**Vision Foundation Models** (see Ch.11):
- DINOv2: dense feature extraction — produces a meaningful feature vector for every pixel in an image
- SAM2: promptable image/video segmentation — tracks a target mask from point, box, or mask prompts
- GroundingDINO: text-guided detection — say "red cup" and it finds it

**3D Understanding** (see Ch.11, Ch.13):
- Gaussian Splatting with semantic features — pretty, fast 3D reconstruction plus semantic information
- 3D Scene Graph construction — represent object relations as a graph
- 3D lifting of VFM features — lift features pulled from 2D images into 3D space

**Language Integration** (see Ch.11, Ch.12):
- CLIP features for open-vocabulary — even a "never-before-seen object" is searchable by text
- LLM for scene reasoning — inferring "what is this room used for?"
- VLA for action planning — "how should the arm move to pick up the cup?"

### 18.3.4 Example Implementation

```python
# Global Module conceptual code
class GlobalModule:
    def __init__(self):
        self.dinov2 = load_dinov2()
        self.sam = load_sam2()
        self.scene_graph = SemanticSceneGraph()
        self.gaussian_map = GaussianSplatMap()

    def process_keyframe(self, image, depth, pose):
        # 1. Extract VFM features
        features = self.dinov2.extract(image)

        # 2. Open-vocabulary segmentation
        masks = self.sam.segment(image, prompts=self.get_prompts())

        # 3. Update the 3D Scene Graph
        self.scene_graph.update(masks, depth, pose, features)

        # 4. Update the Gaussian Map
        self.gaussian_map.add_keyframe(image, depth, pose, features)

    def query(self, text_prompt):
        # "Where is the red cup?" -> return location
        return self.scene_graph.find(text_prompt)
```

**Runtime behavior**: whenever a keyframe arrives from the Local Module, `process_keyframe()` is called. DINOv2 extracts image features, SAM segments the objects, and the results accumulate in the 3D Scene Graph and Gaussian Map. Later, when the user asks "where is the red cup?", `query()` looks it up. This process can take about a second because the Local Module handles real-time safety.

## 18.4 Cooperation Between the Two Modules

The two modules operate independently, but exchange information and cooperate. It resembles the relationship between a driver (Local) and a navigation app (Global) — the driver watches the road in front of them while the navigation guides the full route.

### 18.4.1 Local → Global

**What is transmitted**:
- Keyframe images / point clouds
- Local pose
- Sensor metadata

**Keyframe selection criteria**:
- Thresholds on travel distance / rotation — "send one after moving 1 m or rotating 30 degrees"
- Scene-change detection — "entered a new room"
- Information content (feature count, coverage) — "this frame carries a lot of new information"

### 18.4.2 Global → Local

**What is transmitted**:
- Prior map (for needed regions) — "obstacle information near the kitchen"
- Semantic information (object locations, classes) — "table here, chair there"
- Navigation waypoints — "follow this path"

**Example scenario**:

```
1. User: "Go to the kitchen and bring the cup"

2. Global:
   - Understand the command via VLM
   - Look up kitchen, cup locations in the Scene Graph
   - Plan the path

3. Global -> Local:
   - Waypoints: [current -> hallway -> kitchen -> in front of cup]
   - Local map of the kitchen area
   - Expected location of the cup

4. Local:
   - Follow the waypoints
   - Real-time obstacle avoidance
   - Precision approach near the cup
```

**Communication failure**: if WiFi drops while the robot is working in an underground parking lot, it must continue on the Local Module alone. It estimates its position through odometry, avoids obstacles, and moves toward the last waypoint it received. When WiFi returns, it sends the accumulated data to the Global Module and receives an updated plan. Real robots need this form of **graceful degradation**.

### 18.4.3 Communication and Synchronization

**Communication methods**:
- ROS2 DDS: local network (within the same building)
- WebSocket: cloud connection (remote server)
- 5G/WiFi: mobile robots (outdoor environments)

**Synchronization strategy**:
- Keyframe-based (no continuous streaming) — saves bandwidth
- Asynchronous processing (does not wait for Global to finish) — Local never stops
- Caching (frequently visited regions) — avoid resending the same data every time

## 18.5 Example Research Topics

The research topics below are ones our lab is actively working on or could take on. For each, we note the **prerequisite chapters**, so if a topic interests you, start from those chapters.

### Local Module Research

1. **Lighter SLAM**
    - Neural-network-based lightweight VO — replace classical VO with a neural network, but make it run on a Jetson
    - Event camera utilization — ultra-fast, low-power cameras for SLAM in extreme environments
    - Hardware acceleration (FPGA) — implement SLAM's core operations in hardware
    - **Prerequisites**: Ch.9 (camera models) and Ch.14 (Visual Odometry, SLAM) required. Ch.3 (optimization) also recommended
2. **Efficient obstacle recognition**
    - Depth-only obstacle detection — detect obstacles from depth alone, without RGB
    - Temporal consistency — maintain consistency across frames (no flickering in and out frame by frame)
    - Uncertainty-aware — also use the signal of "not sure whether this is an obstacle"
    - **Prerequisites**: Ch.10 (depth estimation, object detection) required. Ch.3 (coordinate transforms) also important
3. **Sensor fusion optimization**
    - Lightweight tight coupling — fuse IMU + camera + LiDAR tightly, but keep it light
    - Coping with sensor dropout — keep running even when one sensor fails
    - **Prerequisites**: Ch.2 (sensors), Ch.14 (Visual Odometry), and Ch.3 (optimization) required

### Global Module Research

1. **3D extension of VFMs**
    - DINOv2 features in 3D — lift 2D features into 3D space and use them there
    - Semantic Gaussian Splatting — bake semantic information into the 3D reconstruction
    - 3D scene understanding — understanding "what kind of structure this space has"
    - **Prerequisites**: Ch.10 (depth), Ch.13 (3D representation), and Ch.11 (VFM) required. Ch.9 (camera models) is basic
2. **VLA integration**
    - Open-vocabulary manipulation — control a robot arm via commands like "pick up that red thing"
    - Language-guided navigation — move via natural-language commands
    - Context-aware behavior — "there is a child nearby, move slowly"
    - **Prerequisites**: Ch.11 (VFM usage) and Ch.12 (VLA) required. Ch.10 (detection) is also useful
3. **Scalability**
    - Large-scale environment representation — an entire apartment complex or campus in a single map
    - Map compression and updating — efficiently manage multi-gigabyte maps
    - Multi-robot collaboration — multiple robots build and share a map together
    - **Prerequisites**: Ch.14 (SLAM), Ch.3 (optimization), and Ch.11 (VFM) required

### Integration Research

1. **Efficient communication**
    - What to send, and when? — sending everything wastes bandwidth; sending nothing makes Global useless
    - Optimal strategy under bandwidth limits — what if 5G drops? what if WiFi is slow?
    - **Prerequisites**: Ch.14 (SLAM, keyframe selection), plus the Local/Global module understanding above
2. **Fallback strategies**
    - Local-only operation when communication drops — perform the basic mission even without a server connection
    - Graceful degradation — capabilities shrink gradually instead of stopping abruptly
    - **Prerequisites**: a whole-system understanding is needed. Read at least Ch.3–14 first
3. **Consistency maintenance**
    - Local/Global map synchronization — if the two modules' maps disagree, the robot gets confused
    - Semantic consistency — "that is a chair" should not later flip to "table"
    - **Prerequisites**: Ch.3 (optimization) and Ch.14 (SLAM, map management) required

## 18.6 Questions That Separate Motivation from Novelty

Motivation explains why a problem needs to be solved; novelty identifies what an approach changes and how. The two are often conflated when choosing a research direction or writing a first paper.

Saying only that "an existing method cannot do X, so we added a module" rarely goes beyond motivation. Novelty becomes concrete only when the paper explains why the module is necessary and why it must take that particular form.

The following three papers illustrate the difference between posing a problem and contributing a design.

### Case 1 — ORB-SLAM2 (Mur-Artal & Tardós 2017)

- **Motivation**: Extend the map-reuse, loop-closing, and relocalization structure of monocular ORB-SLAM to stereo and RGB-D inputs.
- **Direct extension**: Build a separate SLAM system for each input modality.
- **Paper's design**: All three modalities share the tracking, local-mapping, and loop-closing structure and use ORB features. Stereo and RGB-D observations contribute depth from disparity and enter metric-scale bundle adjustment.
- **Design principle**: Preserve a common system architecture while placing modality-specific differences in observation construction and bundle-adjustment residuals.

The primary source is [*ORB-SLAM2: An Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras*](https://doi.org/10.1109/TRO.2017.2705103). The 2015 ORB-SLAM paper describes a monocular system and therefore cannot support the three-modality example.

### Case 2 — 3D Gaussian Splatting (Kerbl et al. 2023)

- **Motivation**: NeRF rendering is too slow for the desired interactive use.
- **Direct extension**: Add acceleration modules such as sparse sampling, pruning, or distillation on top of NeRF.
- **Paper's design**: Treat ray marching as the bottleneck and replace the representation with explicit 3D Gaussian primitives that can be rasterized directly.
- **Design principle**: Locate the speed limit in the combination of representation and rendering, rather than in one isolated operation.

### Case 3 — DUSt3R (Wang et al. 2024)

Traditional SfM requires camera intrinsics and is sensitive to errors passed between stages. One could replace only matching or triangulation with a neural network, but Wang et al. changed the output representation itself. Given two views, the model predicts pointmaps in a common coordinate frame, obtaining correspondence and structure together; camera intrinsics can then be recovered from the pointmaps instead of being supplied as an input. DUSt3R's contribution is this reformulation of the staged SfM pipeline as pointmap prediction.

### The Design Question Shared by the Three Papers

All three papers ask *why must this module take this form?* The answer lies less in the number of modules than in how the interface, representation, and output format are chosen.

> A contribution section should answer *why this module must take this form*. Explaining only why the problem matters remains motivation; novelty appears when the basis for the design choice is also explicit.

For a fuller treatment of motivation and method in a paper, see [*Research Notes* Ch.23 — Introduction](../../research-notes/guide.html#chapter-23) and [Ch.25 — Method](../../research-notes/guide.html#chapter-25).
