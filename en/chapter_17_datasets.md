# Ch.17 — Datasets & Benchmarks

Once containerized toolchains and experiment tracking from Ch.16 are in place, empirical performance hinges on data. A dataset's sensor suite, collection conditions, splits, and annotations define what a model can learn and what comparisons are fair. This chapter surveys standard benchmarks across autonomous driving, SLAM, and computer vision, then walks through dataset ingestion and the practical steps required to synchronize, calibrate, and annotate your own recordings.

In recent years, **synthetic data** has claimed a growing share of training pipelines. Because collecting and labeling real-world sensor logs is expensive and slow, teams increasingly pretrain on simulator-generated data and fine-tune on smaller real-world datasets—using tools such as NVIDIA Isaac Sim's domain randomization or Habitat's large-scale indoor scenes. **Sim-to-Real datasets**, which pair simulated captures with corresponding real-world recordings, are also being built to measure and close that transfer gap directly.

## 17.1 Autonomous Driving / Robotics Datasets

Robotics benchmarks split roughly between outdoor multi-sensor driving suites—such as KITTI, nuScenes, and the Waymo Open Dataset—and controlled indoor or aerial sequences designed to evaluate visual SLAM and visual-inertial odometry (VIO).

### 17.1.1 KITTI / KITTI360

Released in 2012, **KITTI** served as the foundational benchmark for autonomous driving and 3D computer vision. Even after larger and more diverse datasets emerged, KITTI remains a standard reference point when comparing visual odometry, SLAM, and stereo-depth methods against prior literature. Its sensor suite pairs stereo cameras and a 3D LiDAR (Velodyne HDL-64E) with GPS/IMU measurements and 2D/3D annotations across five core tasks: stereo depth estimation, optical flow, visual odometry / SLAM, 3D object detection, and semantic segmentation.

**Download**: https://www.cvlibs.net/datasets/kitti/

> **Further reading**
> - [KITTI Benchmark official site](https://www.cvlibs.net/datasets/kitti/) — Dataset downloads and task-specific leaderboards.
> - [KITTI-360 site](https://www.cvlibs.net/datasets/kitti-360/) — Expanded 360-degree successor to KITTI.
> - [Dark Programmer — Using KITTI Data (LiDAR-camera transforms)](https://darkpgmr.tistory.com/190) — Practical walkthrough of coordinate transforms and LiDAR-to-camera projection on KITTI.

### 17.1.2 nuScenes

**nuScenes** scales autonomous driving evaluation beyond KITTI with full 360-degree sensing and radar coverage, making it a primary benchmark for 3D object detection and Bird's Eye View (BEV) perception. Each recording captures 6 cameras providing 360° coverage, 5 Radars, and 1 LiDAR across 1,000 scenes (40K keyframes) spanning diverse conditions such as night and rain. Annotations cover 23 object classes along with rich attribute and visibility metadata, and models are evaluated primarily via mAP and the nuScenes Detection Score (NDS).

> **Further reading**
> - [nuScenes devkit Documentation](https://www.nuscenes.org/nuscenes) — Official guide covering dataset layout, devkit APIs, and tutorial notebooks.
> - [nuScenes devkit GitHub](https://github.com/nutonomy/nuscenes-devkit) — Python devkit source code and usage examples.

### 17.1.3 Waymo Open Dataset

Released by Waymo (an Alphabet subsidiary), the **Waymo Open Dataset** stands alongside nuScenes as a flagship benchmark in autonomous driving perception and prediction. Its 1,150 scenes (20 seconds each) provide high-grade multi-sensor captures across urban, suburban, and nighttime environments, annotated with 12M LiDAR labels and 12M camera labels. An accompanying annual challenge makes it a useful barometer of current state-of-the-art methods.

> **Further reading**
> - [Waymo Open Dataset official site](https://waymo.com/open/) — Dataset downloads and annual challenge details.
> - [Waymo Open Dataset GitHub](https://github.com/waymo-research/waymo-open-dataset) — Official evaluation tools and starter code.

### 17.1.4 Datasets for RGB-D SLAM and VIO / VINS

For indoor visual SLAM and aerial visual-inertial odometry, **TUM RGB-D** and **EuRoC MAV** are the standard comparison benchmarks:

- **TUM RGB-D**: Provides indoor RGB-D camera sequences paired with high-precision motion-capture ground-truth trajectories, serving as the classic benchmark for RGB-D Visual SLAM.
- **EuRoC MAV**: Captures micro aerial vehicle flights with synchronized stereo cameras and IMU streams across sequences of graded motion and lighting difficulty, making it the standard testbed for VIO and VINS algorithms.

> **Further reading**
> - [TUM RGB-D Benchmark](https://cvg.cit.tum.de/data/datasets/rgbd-dataset) — Standard RGB-D SLAM sequences and trajectory evaluation tools.
> - [EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) — Standard VIO evaluation dataset.

## 17.2 Computer Vision Datasets

General computer vision benchmarks underpin robotics perception both as standard pretraining sources for visual backbones and as dense 3D indoor scene benchmarks.

### 17.2.1 ImageNet

**ImageNet**—comprising 1.2M training images across 1,000 object classes—is the benchmark that catalyzed the modern deep-learning era following AlexNet's breakthrough in 2012. ImageNet-pretrained weights quickly became the default initialization across vision architectures, and camera-based robotics perception modules still routinely rely on ImageNet-pretrained backbones.

### 17.2.2 COCO

**COCO** is the primary benchmark for 2D object detection and instance segmentation, offering 330K images and 1.5M densely annotated object instances (bounding boxes and segmentation masks) across 80 object categories, alongside keypoint detection and image captioning tasks. Unlike the PASCAL VOC convention—which evaluates at a single IoU threshold—COCO mAP averages precision across multiple IoU thresholds, penalizing loose localization.

### 17.2.3 ScanNet / NYU Depth V2

For indoor service and home robotics, **ScanNet** and **NYU Depth V2** serve as foundational 3D perception benchmarks. ScanNet provides 1,513 indoor RGB-D sequences complete with camera poses, reconstructed meshes, and 3D semantic segmentation labels, making it central to 3D scene understanding research. NYU Depth V2 focuses on monocular and indoor depth estimation, supplying 464 indoor scenes totaling 407K RGB-D frames.

> **Further reading**
> - [COCO Dataset](https://cocodataset.org/) — Official site, dataset downloads, and evaluation scripts.
> - [ScanNet Benchmark](http://www.scan-net.org/) — 3D scene understanding dataset and leaderboard.
> - [Papers With Code - Datasets](https://paperswithcode.com/datasets) — Searchable directory of benchmarks and task leaderboards.

## 17.3 How to Use Datasets

### 17.3.1 Download and Format Understanding

Every public benchmark organizes raw sensor files, calibration matrices, and annotations in its own convention. Before writing training or evaluation code, inspect the directory layout and label specification carefully—especially for 3D bounding boxes, where coordinate frames vary across datasets (camera frame vs. LiDAR frame, y-up vs. z-up). Skipping that step often costs days debugging inverted axes or misaligned projections.

As a concrete example, the **KITTI Object Detection** split separates left RGB images, binary LiDAR point clouds, calibration matrices, and text labels into parallel directories:

```
kitti/
├── training/
│   ├── image_2/       # Left RGB images
│   ├── velodyne/      # LiDAR point clouds (.bin)
│   ├── calib/         # Calibration files
│   └── label_2/       # 2D/3D annotations
└── testing/
    └── ...
```

Each line in `label_2/` encodes one object's class, truncation/occlusion flags, 2D bounding box, 3D dimensions, and camera-frame location and yaw, which can be parsed directly in Python:

```python
# KITTI label format: type truncated occluded alpha bbox(4) dimensions(3) location(3) rotation_y
with open('label.txt', 'r') as f:
    for line in f:
        parts = line.strip().split()
        obj_type = parts[0]
        bbox = [float(x) for x in parts[4:8]]  # left, top, right, bottom
        dimensions = [float(x) for x in parts[8:11]]  # height, width, length
        location = [float(x) for x in parts[11:14]]  # x, y, z
```

> **Further reading**
> - [KITTI Benchmark official site - Object Detection DevKit](https://www.cvlibs.net/datasets/kitti/eval_object.php?obj_benchmark=3d) — Label format specification and official evaluation code.
> - [nuScenes devkit Tutorial Notebooks](https://github.com/nutonomy/nuscenes-devkit/tree/master/python-sdk/tutorials) — Interactive notebooks introducing the nuScenes schema.

### 17.3.2 DataLoader Implementation

In PyTorch, data ingestion splits into two responsibilities: a `Dataset` class defining how individual samples are loaded and preprocessed, and a `DataLoader` orchestrating batching, shuffling, and parallel worker processes. Keep preprocessing inside `__getitem__` lean and tune `num_workers` to your storage and CPU budget, as I/O and CPU transforms frequently bottleneck GPU utilization:

```python
from torch.utils.data import Dataset, DataLoader

class MyDataset(Dataset):
    def __init__(self, root_dir, transform=None):
        self.root_dir = root_dir
        self.transform = transform
        self.samples = self._load_samples()

    def _load_samples(self):
        # Load file list
        return list_of_samples

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        sample = self.samples[idx]
        image = load_image(sample['image_path'])
        label = sample['label']

        if self.transform:
            image = self.transform(image)

        return image, label

# Usage
dataset = MyDataset(root_dir='./data')
dataloader = DataLoader(dataset, batch_size=32, shuffle=True, num_workers=4)
```

> **Further reading**
> - [PyTorch Data Loading Tutorial](https://pytorch.org/tutorials/beginner/data_loading_tutorial.html) — Official guide to building custom `Dataset` classes.
> - [PyTorch documentation — torch.utils.data](https://docs.pytorch.org/docs/stable/data.html) — Reference for `DataLoader` batching, samplers, `num_workers`, and `collate_fn`.

## 17.4 Collecting Your Own Data

Public datasets rarely match the exact sensor mount, lens geometry, or target environment of a custom robot platform. When you must record your own data, handling time synchronization, calibration, and annotation systematically up front is essential—flaws at capture time usually make recordings unusable downstream.

### 17.4.1 Sensor Synchronization

Multi-sensor fusion rests on the assumption that measurements from different modalities describe the robot and world at the same instant. At highway or fast driving speeds, even a 10 ms offset between camera and LiDAR timestamps introduces tens of centimeters of spatial misalignment, corrupting projection and association.

When hardware supports it, **hardware synchronization** locks sensors together via physical trigger signals or Pulse Per Second (PPS) timing. When hardware triggers are unavailable, **software synchronization** pairs nearby timestamps approximately or interpolates high-rate streams (such as IMU or odometry) to match image or scan times.

In ROS, `message_filters` synchronizes messages using the timestamp in each message header—falling back to arrival time only for headerless messages, and only when `allow_headerless` is explicitly enabled:

```python
import message_filters

# Approximate Time Synchronizer
image_sub = message_filters.Subscriber(self, Image, '/camera/image')
lidar_sub = message_filters.Subscriber(self, PointCloud2, '/lidar/points')

sync = message_filters.ApproximateTimeSynchronizer(
    [image_sub, lidar_sub], queue_size=10, slop=0.1
)
sync.registerCallback(self.callback)
```

### 17.4.2 Calibration

Even perfectly synchronized sensors produce inconsistent detections if their internal optics or relative mounting poses are miscalibrated, degrading fusion accuracy across the entire stack. Three calibration steps cover most robot sensor rigs:

- **Camera Intrinsic**: Estimates focal length, principal point, and lens distortion using a checkerboard target (for example, via OpenCV `calibrateCamera`), yielding the 3×3 camera matrix K.
- **Camera-LiDAR Extrinsic**: Estimates the rigid transformation between camera and LiDAR via checkerboard plane fitting, dedicated calibration targets, or target-less automatic feature matching. Expressed in linear algebra terms, the extrinsic forms a 3×4 block [R|t] inside camera projection equations, and a 4×4 homogeneous transformation—appending a [0 0 0 1] row at the bottom—when transforming coordinates between sensors.
- **Camera-IMU**: Jointly estimates spatial extrinsics and time offsets between camera and inertial sensors, for which **Kalibr** is the standard open-source tool.

> **Further reading**
> - [OpenCV Camera Calibration Tutorial](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html) — Checkerboard-based intrinsic calibration walkthrough.
> - [Kalibr GitHub](https://github.com/ethz-asl/kalibr) — Standard toolbox for camera-IMU and multi-camera calibration.

### 17.4.3 Labeling Tools

Raw sensor logs still require annotations before supervised training or quantitative evaluation. Manual labeling is one of the highest-cost bottlenecks in robotics research, and label consistency sets an upper bound on model quality—though since 2023, foundation models such as SAM (Segment Anything Model) have increasingly powered semi-automatic labeling pipelines:

- **CVAT (Computer Vision Annotation Tool)**: Open-source, web-based tool for image and video annotation supporting bounding boxes, polygons, and keypoints.
- **Labelbox**: Cloud-based annotation platform offering team workflows and 3D labeling support.
- **3D Labeling**: Specialized tools such as **SUSTechPOINTS** for LiDAR point clouds and the **KITTI-360 labeling tool** for 3D scene annotations.
- **Automatic labeling via synthetic data**: Simulators such as NVIDIA Isaac Sim and AI2-THOR emit ground-truth masks, depth, and 3D boxes automatically alongside rendered frames, while domain randomization (varying textures, lighting, and backgrounds) improves generalization. While synthetic pipelines cut manual annotation and real-world collection overhead, building accurate simulation assets, rendering at scale, and auditing synthetic data still carry significant engineering cost.

> **Further reading**
> - [CVAT Documentation](https://docs.cvat.ai/) — Official documentation for the open-source CVAT annotator.
> - [Roboflow](https://roboflow.com/) — Integrated platform for annotation, augmentation, and training dataset management.
> - [NVIDIA Isaac Sim - Synthetic Data Generation](https://docs.omniverse.nvidia.com/isaacsim/latest/replicator_tutorials/index.html) — Guide to generating labeled synthetic datasets in Isaac Sim.

## Technical Timeline

```
2009 ─── ImageNet released
  │       Start of large-scale image classification benchmarks
  │
2012 ─── KITTI released / AlexNet dominates ImageNet
  │       Birth of the autonomous driving benchmark, start of the deep learning revolution
  │
2014 ─── COCO released
  │       Standard benchmark for Object Detection and Segmentation
  │
2017 ─── ScanNet released
  │       Indoor 3D Scene Understanding research takes off
  │
2019 ─── nuScenes and Waymo Open Dataset released
  │       Era of large-scale, high-quality autonomous driving datasets
  │
2020 ─── Synthetic data research in full swing
  │       Domain Randomization, Sim-to-Real Transfer
  │       Large-scale synthetic data generation based on NVIDIA Isaac Sim
  │
2023 ─── Datasets for the Foundation Model era
  │       SA-1B (for training SAM, 1 billion masks)
  │       Open X-Embodiment (unified robot manipulation data)
  │
2024+ ── Future trends in datasets
          Mixed training on synthetic + real data becomes standard
          Sim-to-Real datasets (paired sim/real data)
          Automatic labeling (Foundation Model based)
          Large-scale collection and sharing of robot manipulation data (Open X-Embodiment)
          Multimodal datasets (vision + language + tactile + force/torque)
```

With perception, SLAM, middleware frameworks (Ch.15), reproducible development environments (Ch.16), and datasets and synchronization pipelines (Ch.17) in place, the final architectural question is how all of these pieces fit onto an actual robot under strict power, compute, and latency limits. Ch.18 brings these threads together in a concrete Spatial AI system design split between a lightweight, real-time onboard Local Module and a heavyweight semantic Global Module.
