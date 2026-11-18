# Ch.18 — Lab Research Directions

Our lab structures a Spatial AI system around two complementary modules. That split grows out of physical hardware limits and real-time deadlines, and it is where the concepts from earlier chapters come together in a working robot.

## 18.1 Overview

We divide a Spatial AI system into **two modules**, separating fast onboard geometry from heavier semantic reasoning.

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

A single onboard computer cannot handle every function within a robot's weight, power, and response-time limits.

Start with **physical constraints**. An NVIDIA A100 GPU server weighs tens of kilograms and draws hundreds of watts, so a battery-powered drone cannot carry one. Robots commonly rely on embedded boards such as the Jetson Orin instead, yet those boards cannot run large models like DINOv2 or SAM in real time.

The **timing demands** pull in different directions as well. Obstacle avoidance must react within tens of milliseconds and cannot wait on a server round-trip, whereas semantic interpretation can afford to run more slowly on a larger model.

The two modules divide their responsibilities accordingly:

1. **Compute limits**: Depending on model size, power budget, and latency targets, an onboard computer (such as a Jetson) often cannot run large models in real time.
2. **Real-time safety**: Obstacle avoidance requires a response within tens of milliseconds.
3. **Semantic understanding**: Vision foundation models (VFMs) and vision-language-action (VLA) models distinguish object classes and states—recognizing, for instance, a broken glass on the floor.
4. **Complementary roles**: Pairing the Local Module's geometric precision with the Global Module's semantic understanding lets each side play to its strengths.

> By analogy, the Local Module provides the robot's **reflexes**, while the Global Module acts as its **cerebral cortex**. Touch a hot pot and you pull your hand away first (reflex), then register, *"Right, the stove was on"* (cognition). Robots work the same way.

## 18.2 Local Module: Lightweight Geometry

The Local Module runs directly on the robot and handles the real-time perception needed for safe motion.

### 18.2.1 Goals

- **Odometry**: Estimate the robot's ego-motion continuously (*"Where am I right now?"*).
- **Obstacle detection**: Sense immediate hazards in the path (*"Something is right in front—steer clear!"*).
- **Local mapping**: Maintain a metric map of the immediate surroundings (*"Here is what the geometry within 3 m looks like."*).

**In practice**: Imagine a child darting in front of a delivery robot in an apartment hallway. The Local Module spots the obstacle with a depth sensor, estimates the robot's pose through odometry, and computes an avoidance path within the deadline derived from control and safety analysis. At that instant, avoiding a collision takes priority over knowing what the obstacle is; semantic classification happens separately in the Global Module.

### 18.2.2 Characteristics

Because this module sits directly inside the control loop, its design centers on predictable timing and tight resource bounds:

- **Latency budget**: Derive update rates and hard deadlines from platform speed, braking distance, control bandwidth, and sensor rate.
- **Resource budget**: Choose the embedded board, power mode, and cooling setup to match the measured workload.
- **Timing evidence**: Measure worst-case latency, jitter, and deadline misses—not only mean FPS—directly on the target hardware.

### 18.2.3 Tech Stack

The Local Module combines proven geometric pipelines with compact neural models optimized for edge hardware.

**Classical methods**:
- **ORB-SLAM3**: Feature-based visual SLAM for camera pose estimation (see Ch.9, Ch.14).
- **VINS-Mono**: Visual-inertial odometry fusing camera frames with IMU measurements (see Ch.14).
- **FAST-LIO2**: LiDAR-inertial odometry fusing LiDAR scans with IMU data (see Ch.2, Ch.14).

**Lightweight learning models**:
- **Lightweight depth estimation**: Compact depth predictors built on backbones such as MobileNet (see Ch.10).
- **Compressed segmentation models**: Smaller models trained with knowledge distillation (see Ch.10, Ch.11).
- **TensorRT optimization**: Graph, kernel, and precision optimization for inference on NVIDIA GPUs.

**Edge deployment**:

```bash
# TensorRT optimization example
trtexec --onnx=model.onnx --saveEngine=model.trt --fp16 --memPoolSize=workspace:4096
```

> TensorRT compiles an optimized inference engine for NVIDIA GPUs. Using FP16 can cut memory footprint and latency, though both the speedup and any shift in task metrics depend on the model, input size, batch, power mode, and software stack. Measure end-to-end latency and validation accuracy on the target Jetson before adopting it.

### 18.2.4 Example Implementation

The skeleton below shows how odometry, obstacle detection, and keyframe transmission fit together inside a single update cycle:

```python
# Local Module conceptual code
class LocalModule:
    def __init__(self):
        self.odometry = FastLIO2()
        self.obstacle_detector = LightweightObstacleNet()  # TensorRT

    def process(self, sensor_data):
        # 1. Odometry update (IMU input, 100 Hz)
        pose = self.odometry.update(sensor_data.imu, sensor_data.lidar)

        # 2. Obstacle detection (camera input, 30 Hz)
        obstacles = self.obstacle_detector(sensor_data.image)

        # 3. Send keyframe to the Global Module
        if self.is_keyframe(pose):
            self.send_to_global(sensor_data, pose)

        return pose, obstacles
```

**Walking through the loop**: Suppose `process()` runs once per fusion tick. Because IMU readings arrive at 100 Hz and camera images at 30 Hz, each sensor callback writes its latest measurement into a buffer, and each tick consumes the buffered readings together. On every tick, the module computes *"where am I now?"* (odometry) and *"what is in front?"* (obstacles), forwarding data to the Global Module only on keyframes so it does not saturate the network link.

## 18.3 Global Module: VFM-based Understanding

Running on a server or in the cloud, the Global Module interprets object classes and spatial relations. Where the Local Module sees an unnamed obstacle, the Global Module can recognize it as broken glass and attach it to a node in the scene graph.

### 18.3.1 Goals

- **Global map understanding**: Capture room-level layout and meaning (*"This is the kitchen; that is the living room."*).
- **Semantic scene graph**: Represent relationships among objects (*"The cup is on the table."*).
- **Long-term memory**: Track environmental changes over time (*"Yesterday there was no chair here; today there is."*).

**In practice**: Picture a home service robot moving through an apartment day after day. Its Global Module maintains a high-level map—a sofa, TV, and table in the living room; a refrigerator and sink in the kitchen. When the user asks it to *"bring the remote from the living room table,"* the Global Module looks up the remote's location in the scene graph and sends a waypoint to the Local Module.

### 18.3.2 Characteristics

Because the Global Module sits off-board, it trades tight cycle times for richer models and broader context:

- **Model choice**: DINOv2 and SAM2 ship in multiple sizes; they are not all billion-parameter models.
- **Compute choice**: Size the server hardware around the model variant, input resolution, precision, active scene count, and measured memory and latency.
- **Update budget**: While global mapping tasks can run outside the local control deadline, user interaction and change detection still need an explicit end-to-end latency budget.

### 18.3.3 Tech Stack

The Global Module pairs large pretrained vision models with 3D scene representations and language interfaces.

**Vision Foundation Models** (see Ch.11):
- **DINOv2**: Extracts dense patch-level features across an image grid (reaching full pixel resolution requires interpolation or upsampling).
- **SAM2**: Performs promptable image and video segmentation, tracking target masks from point, box, or mask prompts.
- **GroundingDINO**: Runs text-guided detection—given a query like *"red cup"*, it localizes the matching object.

**3D Understanding** (see Ch.11, Ch.13):
- **Gaussian Splatting with semantic features**: Produces fast, detailed 3D reconstructions augmented with semantic embeddings.
- **3D Scene Graph construction**: Encodes objects and their spatial relations as a structured graph.
- **3D lifting of VFM features**: Projects 2D image features back into 3D space.

**Language Integration** (see Ch.11, Ch.12):
- **CLIP features for open-vocabulary search**: Makes even previously unseen object categories searchable by text.
- **LLMs for scene reasoning**: Infers higher-level context, such as what a room is used for.
- **VLAs for action planning**: Translates visual and language context into manipulation plans, such as how an arm should move to pick up a cup.

### 18.3.4 Example Implementation

The conceptual class below illustrates how incoming keyframes update both the semantic scene graph and the 3D Gaussian map:

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

        # 2. Promptable segmentation (text-vocabulary prompts come from GroundingDINO/CLIP)
        masks = self.sam.segment(image, prompts=self.get_prompts())

        # 3. Update the 3D Scene Graph
        self.scene_graph.update(masks, depth, pose, features)

        # 4. Update the Gaussian Map
        self.gaussian_map.add_keyframe(image, depth, pose, features)

    def query(self, text_prompt):
        # "Where is the red cup?" -> return location
        return self.scene_graph.find(text_prompt)
```

**Walking through a keyframe**: Whenever a keyframe arrives from the Local Module, `process_keyframe()` runs. DINOv2 extracts visual features, SAM segments the objects, and the results accumulate in both the 3D scene graph and the Gaussian map. Later, when a user asks, *"Where is the red cup?"*, `query()` looks up the target's coordinates. Because the Local Module already handles real-time safety, this semantic update can comfortably take around a second.

## 18.4 Cooperation Between the Two Modules

Although the two modules operate independently, they rely on continuous exchange—much like a driver (Local) and a navigation app (Global), where the driver watches the road immediately ahead while the app guides the full route.

### 18.4.1 Local → Global

To keep bandwidth manageable, the Local Module sends compact summaries only when the scene changes meaningfully.

**What is transmitted**:
- Keyframe images and point clouds
- Local pose estimates
- Sensor metadata

**Keyframe selection criteria**:
- Motion thresholds—for example, sending a keyframe after moving 1 m or rotating 30 degrees
- Scene-change detection—such as entering a new room
- Information gain—measured by feature count or new spatial coverage

### 18.4.2 Global → Local

In return, the Global Module sends semantic context and high-level navigation targets back down to the robot.

**What is transmitted**:
- Local prior maps for upcoming regions (such as obstacle layout near the kitchen)
- Semantic annotations (object locations and classes, such as *"table here, chair there"*)
- Navigation waypoints along the planned route

**Example scenario**: Tracing a single user request from voice command to physical motion clarifies how the handoff works:

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

**Handling communication loss**: If Wi-Fi drops while the robot is working in an underground parking garage, it must keep operating on the Local Module alone—tracking pose through odometry, avoiding obstacles, and continuing toward the last received waypoint. Once Wi-Fi returns, it flushes the accumulated data to the Global Module and pulls an updated plan. Real robots depend on this kind of **graceful degradation**.

### 18.4.3 Communication and Synchronization

The transport layer and update cadence depend on where the robot operates:

**Communication methods**:
- **ROS2 DDS**: Local-network messaging within the same building
- **WebSocket**: Connections to a remote or cloud server
- **5G/WiFi**: Wireless links for outdoor mobile robots

**Synchronization strategy**:
- **Keyframe-based updates**: Avoid continuous raw streaming to save bandwidth.
- **Asynchronous processing**: Never block the Local Module waiting on the Global Module.
- **Caching**: Keep frequently visited regions local so the robot does not resend static data repeatedly.

## 18.5 Example Research Topics

The topics below reflect active or prospective directions in our lab. Each entry lists its **prerequisite chapters** so you can jump straight to the background material if a direction catches your eye.

### Local Module Research

1. **Lighter SLAM**
    - Neural-network-based lightweight VO: Replace classical visual odometry with a learned model that still runs in real time on a Jetson.
    - Event camera utilization: Leverage ultra-fast, low-power event cameras for SLAM in extreme motion or lighting conditions.
    - Hardware acceleration (FPGA): Move core SLAM operations into dedicated hardware.
    - **Prerequisites**: Ch.9 (camera models) and Ch.14 (visual odometry, SLAM) are essential; Ch.3 (optimization) is also recommended.
2. **Efficient obstacle recognition**
    - Depth-only obstacle detection: Identify obstacles from depth measurements alone, without RGB images.
    - Temporal consistency: Keep detections stable across frames instead of flickering frame by frame.
    - Uncertainty-aware detection: Explicitly represent and use uncertainty when the model is unsure whether a region is an obstacle.
    - **Prerequisites**: Ch.10 (depth estimation, object detection) is essential; Ch.3 (coordinate transforms) is also important.
3. **Sensor fusion optimization**
    - Lightweight tight coupling: Fuse IMU, camera, and LiDAR tightly while staying within an embedded compute budget.
    - Coping with sensor dropout: Keep the estimator running cleanly even when one sensor drops out.
    - **Prerequisites**: Ch.2 (sensors), Ch.14 (visual odometry), and Ch.3 (optimization) are all required.

### Global Module Research

1. **3D extension of VFMs**
    - DINOv2 features in 3D: Lift 2D foundation features into 3D space for downstream spatial queries.
    - Semantic Gaussian Splatting: Embed semantic features directly into 3D reconstructions.
    - 3D scene understanding: Infer the layout and functional structure of an entire room or building.
    - **Prerequisites**: Ch.10 (depth), Ch.13 (3D representation), and Ch.11 (VFM) are required; Ch.9 (camera models) provides the geometric base.
2. **VLA integration**
    - Open-vocabulary manipulation: Control a robot arm from free-form commands such as *"pick up that red thing."*
    - Language-guided navigation: Steer mobile robots using natural-language instructions.
    - Context-aware behavior: Adjust behavior to social context, such as slowing down when a child is nearby.
    - **Prerequisites**: Ch.11 (VFM usage) and Ch.12 (VLA) are required; Ch.10 (detection) is also helpful.
3. **Scalability**
    - Large-scale environment representation: Represent an entire apartment complex or campus within a unified map.
    - Map compression and updating: Manage and incrementally update multi-gigabyte maps efficiently.
    - Multi-robot collaboration: Enable multiple robots to build and share a common map.
    - **Prerequisites**: Ch.14 (SLAM), Ch.3 (optimization), and Ch.11 (VFM) are required.

### Integration Research

1. **Efficient communication**
    - What to send, and when: Streaming everything wastes bandwidth, while sending too little starves the Global Module.
    - Adaptive strategies under bandwidth limits: Maintain useful updates when 5G drops or Wi-Fi slows down.
    - **Prerequisites**: Ch.14 (SLAM, keyframe selection), along with the Local/Global module architecture covered above.
2. **Fallback strategies**
    - Local-only operation on disconnect: Complete core missions safely even when the server connection is lost.
    - Graceful degradation: Scale capabilities down smoothly rather than halting abruptly.
    - **Prerequisites**: System-wide fluency; read at least Ch.3–14 first.
3. **Consistency maintenance**
    - Local/Global map synchronization: Prevent conflicting local and global maps from confusing navigation.
    - Semantic consistency: Avoid spurious label flips (such as *"chair"* to *"table"*) while still allowing legitimate updates when new observations justify them.
    - **Prerequisites**: Ch.3 (optimization) and Ch.14 (SLAM, map management) are required.

## 18.6 Questions That Separate Motivation from Novelty

Motivation explains why a problem needs solving; novelty identifies what an approach changes and how. Those two ideas are easy to blur when choosing a research direction or drafting a first paper.

Stating only that *"an existing method cannot do X, so we added a module"* rarely goes beyond motivation. Novelty becomes concrete only when the paper explains why that module is necessary and why it must take that particular form.

The three papers below illustrate the difference between posing a problem and contributing an architectural design.

### Case 1 — ORB-SLAM2 (Mur-Artal & Tardós 2017)

- **Motivation**: Extend the map-reuse, loop-closing, and relocalization capabilities of monocular ORB-SLAM to stereo and RGB-D cameras.
- **Direct extension**: Build a separate SLAM pipeline for each input modality.
- **Paper's design**: All three modalities share the same tracking, local-mapping, and loop-closing architecture built around ORB features. Stereo cameras obtain depth from disparity, whereas RGB-D cameras synthesize a virtual right-image coordinate from measured depth, unifying both into a stereo observation format before entering metric-scale bundle adjustment.
- **Design principle**: Keep a single system architecture intact while isolating modality-specific differences inside observation construction and bundle-adjustment residuals.

The primary source is [*ORB-SLAM2: An Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras*](https://doi.org/10.1109/TRO.2017.2705103). The earlier 2015 ORB-SLAM paper covered only monocular input and therefore does not illustrate this three-modality design.

### Case 2 — 3D Gaussian Splatting (Kerbl et al. 2023)

- **Motivation**: NeRF rendering is too slow for real-time interactive use.
- **Direct extension**: Layer acceleration tricks—such as sparse sampling, pruning, or distillation—onto NeRF.
- **Paper's design**: Identify ray marching itself as the bottleneck and replace the volumetric field with explicit 3D Gaussian primitives that can be rasterized directly.
- **Design principle**: Trace the speed bottleneck to the coupling of representation and rendering rather than optimizing a single operation in isolation.

### Case 3 — DUSt3R (Wang et al. 2024)

Traditional Structure from Motion (SfM) requires known camera intrinsics and is fragile when errors cascade from one stage to the next. Rather than swapping out only feature matching or triangulation for a neural network, Wang et al. changed the output representation itself. Given two views, the model predicts pointmaps in a shared coordinate frame, recovering dense correspondence and 3D structure together; camera intrinsics can then be estimated directly from those pointmaps instead of being supplied up front. DUSt3R's contribution lies in reformulating the staged SfM pipeline as unified pointmap prediction.

### The Design Question Shared by the Three Papers

All three papers ask the same question: *why must this module take this form?* The answer rarely lies in how many modules were added; it lies in how the interface, internal representation, and output format were chosen.

> A contribution section should answer *why this module must take this form*. Explaining only why the problem matters stops at motivation; novelty emerges when the reasoning behind the design choice itself is made explicit.

For a fuller treatment of framing motivation and method in a paper, see [*Research Notes* Ch.23 — Introduction](../../research-notes/guide.html#chapter-23) and [Ch.25 — Method](../../research-notes/guide.html#chapter-25).
