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
│  │ • 10-100 Hz         │     │ • 1-10 Hz                   │ │
│  └─────────────────────┘     └─────────────────────────────┘ │
└──────────────────────────────────────────────────────────────┘
```

**Why two modules? — an intuitive view**

You might think, "Can't we just put one good computer on the robot?" Honestly, if that were possible we would do it. But it is not.

First, consider the **physical constraints**. A robot has to move. You cannot mount an NVIDIA A100 GPU server on a drone — the weight alone is tens of kilograms, and it draws hundreds of watts. That is unrealistic for a battery-powered robot. So the computer actually carried on the robot is an embedded board like the Jetson Orin, and such a board cannot run large models like DINOv2 or SAM in real time.

Next are the **time constraints**. Suppose a robot is walking down a hallway, 0.1 seconds away from hitting a wall — it cannot pause and say "wait, waiting for the server to respond...". Things that must react "right now," like obstacle avoidance, are a different kind of problem from "understanding what that object is," which can afford to be slower.

So we split it this way:

1. **The reality of compute**: on-board computers on the robot (Jetson, etc.) do not have the headroom to run large models
2. **Real-time requirements**: obstacle avoidance needs immediate reaction — 0.1 seconds is the difference between life and death
3. **Deep understanding**: VFMs and VLAs need heavy compute — judgments like "that is a broken glass, be careful"
4. **Mutual complementarity**: geometric precision (Local) + semantic understanding (Global) = a truly intelligent robot

> By analogy, the Local Module is the robot's **reflex nervous system**, and the Global Module is its **cerebral cortex**. Touch a hot pot and you pull your hand away first (reflex), then think "ah, the stove was on" (cognition). Robots work the same way.

## 18.2 Local Module: Lightweight Geometry

This module runs directly on the robot in real time. It processes the minimum information the robot needs to move safely "in this moment."

### 18.2.1 Goals

- **Odometry**: estimate the robot's own motion — "where am I right now?"
- **Obstacle Detection**: immediate obstacle sensing — "something is in front, dodge!"
- **Local Mapping**: a geometric map of the surroundings — "within 3 m of me, the world looks like this"

**A real scenario**: say a delivery robot is passing through an apartment hallway and a child suddenly runs out. The Local Module detects the obstacle instantly through the depth sensor, locates itself via odometry, and computes an avoidance path within 0.05 seconds. It does not need to know whether it is a child or a dog — that is the Global Module's job. The Local Module only needs to know "something is in front, so avoid it."

### 18.2.2 Characteristics

- **Low latency**: 10-100 Hz operation (one processing pass every 10-100 ms)
- **Limited resources**: Jetson, embedded GPU — 15-30 W power envelope
- **Deterministic behavior**: predictable response time — "an answer within 50 ms even in the worst case"

### 18.2.3 Tech Stack

**Classical methods**:
- ORB-SLAM3: feature-based Visual SLAM — pose estimation from a single camera (see Ch.9, Ch.14)
- VINS-Mono: Visual-Inertial Odometry — camera + IMU fusion (see Ch.14)
- FAST-LIO2: LiDAR-Inertial Odometry — LiDAR + IMU fusion (see Ch.2, Ch.14)

**Lightweight learning models**:
- Lightweight depth estimation — compressed with a MobileNet backbone (see Ch.10)
- Compressed segmentation models — knowledge distillation applied (see Ch.10, Ch.11)
- TensorRT optimization — 2-5x speedup on NVIDIA GPUs

**Edge deployment**:

```bash
# TensorRT optimization example
trtexec --onnx=model.onnx --saveEngine=model.trt --fp16 --workspace=4096
```

> What is TensorRT: a tool that converts a PyTorch-trained model into a form optimized for NVIDIA GPUs. Switching to FP16 (half precision) halves the model size while keeping accuracy nearly intact. Running YOLO on a Jetson: without TensorRT, 5 FPS; with it, 30 FPS — on a real robot, that gap is the difference between "usable" and "not usable."

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

A high-level understanding module that runs on a server or in the cloud. Where the Local Module only gets as far as "something is in front," the Global Module understands "that is a broken glass, probably dropped by the owner in the living room."

### 18.3.1 Goals

- **Global map understanding**: grasp spatial structure and meaning — "this is the kitchen, that is the living room"
- **Semantic Scene Graph**: represent relations between objects — "the cup is on the table"
- **Long-term Memory**: track environmental changes — "yesterday there was no chair here, today there is"

**A real scenario**: a home service robot moves around the house every day and learns the environment. The Global Module maintains a high-level map like "the living room has a sofa, a TV, and a table; the kitchen has a refrigerator and a sink." When the user says "bring the remote from the living room table," it looks up the remote's location in the Scene Graph and hands a waypoint to the Local Module.

### 18.3.2 Characteristics

- **High accuracy**: uses large VFMs — models with billions of parameters like DINOv2 and SAM2
- **Abundant compute**: GPU servers, cloud — GPUs at the level of RTX 4090 or A100
- **Non-real-time is acceptable**: 1-10 Hz — "updating once per second is fine"

### 18.3.3 Tech Stack

**Vision Foundation Models** (see Ch.11):
- DINOv2: dense feature extraction — produces a meaningful feature vector for every pixel in an image
- SAM2: open-vocabulary segmentation — point at "any object" and it gets segmented accurately
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

**Reading it as a scenario**: whenever a keyframe arrives from the Local Module, `process_keyframe()` is called. DINOv2 pulls rich features from the image, SAM segments the objects, and these accumulate into the 3D Scene Graph and the Gaussian Map. Later, when the user asks "where is the red cup?", `query()` looks it up. This whole process taking about a second is fine — real-time safety is the Local Module's job.

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

**Another scenario — unstable communication**: the robot is working in an underground parking lot and WiFi drops. In this case it must run on the Local Module alone. It estimates position via odometry and, while avoiding obstacles, moves to the last waypoint it received. When WiFi is restored, it sends the accumulated data to the Global module in one batch and receives an updated plan. This kind of **graceful degradation** is very important on real robots.

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
