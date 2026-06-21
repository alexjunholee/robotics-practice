# Ch.17 — Datasets & Benchmarks

In robotics and computer vision research, datasets are as important as algorithms. Without good data you cannot build good models, and without fair benchmarks you cannot prove in a paper that your method is genuinely better. The useful view is the structure and characteristics of major datasets, plus the methods for collecting and managing your own data.

The share of **synthetic data** has been growing recently. Collecting and labeling real data is costly and time-consuming, so a workflow of pretraining on automatically generated synthetic data from a simulator and then fine-tuning on a small amount of real data has taken hold. NVIDIA Isaac Sim's Domain Randomization and Habitat's large-scale scene generation are representative examples. **Sim-to-Real datasets** — datasets that provide simulator data paired with the corresponding real data — are also being actively constructed.

## 17.1 Autonomous Driving / Robotics Datasets

### 17.1.1 KITTI / KITTI360

A long-standing dataset that became the starting point for autonomous driving research.

Since its release in 2012, KITTI has served as the de facto standard benchmark for autonomous driving and 3D vision research. Larger and more diverse datasets exist today, but many papers still report KITTI results, so you need to know it as a reference point. For Visual Odometry and Stereo Depth Estimation in particular, KITTI is still the primary benchmark.

**Composition**:
- Stereo cameras
- 3D LiDAR (Velodyne HDL-64E)
- GPS/IMU
- 2D/3D labels

**Tasks**:
- Stereo depth estimation
- Optical flow
- Visual odometry / SLAM
- 3D object detection
- Semantic segmentation

**Download**: https://www.cvlibs.net/datasets/kitti/

> **Further reading**
> - [KITTI Benchmark official site](https://www.cvlibs.net/datasets/kitti/) — dataset download and per-task leaderboards.
> - [KITTI-360 site](https://www.cvlibs.net/datasets/kitti-360/) — a broader 360-degree dataset.
> - [Dark Programmer — Using KITTI Data (LiDAR-camera transforms)](https://darkpgmr.tistory.com/190) — hands-on with coordinate frame transforms and LiDAR-camera mapping on KITTI.

### 17.1.2 nuScenes

A large-scale autonomous driving dataset.

It has a richer sensor suite than KITTI (360-degree cameras, Radar included) and is much larger in scale. Alongside KITTI, it is one of the most cited datasets in recent autonomous driving papers. It is central to 3D Object Detection and BEV (Bird's Eye View) based perception research.

**Composition**:
- 6 cameras (360° coverage)
- 5 Radars
- 1 LiDAR
- 1000 scenes, 40K keyframes

**Features**:
- 23 object classes
- Rich annotations (attributes, visibility)
- Diverse conditions including night and rain

**Evaluation metrics**: mAP, NDS

> **Further reading**
> - [nuScenes devkit Documentation](https://www.nuscenes.org/nuscenes) — dataset usage, devkit API, tutorial notebooks.
> - [nuScenes devkit GitHub](https://github.com/nutonomy/nuscenes-devkit) — Python devkit code and examples.

### 17.1.3 Waymo Open Dataset

Google's large-scale autonomous driving dataset.

Together with nuScenes, it is one of the two main benchmarks in current autonomous driving research. It leads in data quality and scale, and its annual challenge lets you track the latest technical trends.

**Scale**:
- 1,150 scenes (20 seconds each)
- 12M LiDAR labels
- 12M camera labels

**Features**:
- High-quality sensors
- Diverse environments (urban, suburban, night)
- Annual challenge

> **Further reading**
> - [Waymo Open Dataset official site](https://waymo.com/open/) — dataset download and challenge participation.
> - [Waymo Open Dataset GitHub](https://github.com/waymo-research/waymo-open-dataset) — official tools and example code.

### 17.1.4 Datasets for VIO / VINS

If you work on Visual-Inertial Odometry (VIO) or SLAM, you need to know the datasets below. Papers in this area almost always report results on them.

**TUM RGB-D**:
- RGB-D camera sequences
- Precise ground truth (motion capture)
- Indoor environments
- Standard for Visual SLAM evaluation

**EuRoC MAV**:
- Drone flight data
- Stereo + IMU
- Standard for VIO evaluation
- Varied difficulty levels

> **Further reading**
> - [TUM RGB-D Benchmark](https://cvg.cit.tum.de/data/datasets/rgbd-dataset) — standard Visual SLAM evaluation dataset and evaluation tools.
> - [EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) — standard VIO evaluation dataset.

## 17.2 Computer Vision Datasets

### 17.2.1 ImageNet

The standard benchmark for image classification.

This is the dataset that marked the turn into deep learning. After AlexNet's overwhelming performance on ImageNet in 2012, nearly every vision model started using ImageNet-pretrained weights. In robotics too, the backbone of camera-based perception modules is mostly an ImageNet-pretrained model.

- 1000 classes
- 1.2M training images
- Pretraining standard

### 17.2.2 COCO

The standard for object detection and segmentation.

If you work on object detection, you need to understand the COCO dataset's evaluation metric (COCO mAP), since it is the industry standard. Note that the way AP is computed per IoU threshold differs from PASCAL VOC.

**Features**:
- 80 object categories
- 330K images, 1.5M object instances
- Dense annotation (bounding box, segmentation mask)

**Tasks**:
- Object detection
- Instance segmentation
- Keypoint detection
- Captioning

### 17.2.3 ScanNet / NYU Depth V2

**ScanNet**:
- 1513 indoor scenes
- RGB-D sequences
- 3D semantic segmentation
- Camera poses and meshes provided

**NYU Depth V2**:
- Indoor RGB-D
- Depth estimation benchmark
- 464 scenes, 407K frames

If you work on indoor robots (home, service robots, and so on), ScanNet and NYU Depth V2 are core benchmarks. ScanNet in particular is indispensable for 3D Scene Understanding research.

> **Further reading**
> - [COCO Dataset](https://cocodataset.org/) — official site, dataset download, and evaluation tools.
> - [ScanNet Benchmark](http://www.scan-net.org/) — 3D Scene Understanding benchmark.
> - [Papers With Code - Datasets](https://paperswithcode.com/datasets) — integrated site for task-wise dataset search and leaderboards.

## 17.3 How to Use Datasets

### 17.3.1 Download and Format Understanding

Each dataset has its own directory structure and format.

If you download a dataset but do not properly understand the directory structure and label format, writing a data loader alone can take days. For 3D labels in particular, the coordinate frame differs across datasets (camera frame vs LiDAR frame, y-up vs z-up, and so on), so read the documentation carefully.

**Example: KITTI Object Detection**:

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

**Example of reading a label file**:

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
> - [KITTI Benchmark official site - Object Detection DevKit](https://www.cvlibs.net/datasets/kitti/eval_object.php?obj_benchmark=3d) — label format description and evaluation code.
> - [nuScenes devkit Tutorial Notebooks](https://github.com/nutonomy/nuscenes-devkit/tree/master/python-sdk/tutorials) — Jupyter notebooks for understanding the data structure.

### 17.3.2 DataLoader Implementation

The standard pattern for data loading in PyTorch.

Without knowing PyTorch's `Dataset` and `DataLoader` pattern you cannot write training code. How you preprocess data in `__getitem__` and what you set `num_workers` to can change training speed significantly.

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
> - [PyTorch Data Loading Tutorial](https://pytorch.org/tutorials/beginner/data_loading_tutorial.html) — official guide for writing a custom Dataset.
> - [Real Python - PyTorch DataLoader](https://realpython.com/python-data-loading/) — detailed walkthrough of DataLoader usage.

## 17.4 Collecting Your Own Data

Public datasets often cannot give you data that fits your own research exactly. You sometimes have to collect data yourself to match your robot's sensor configuration or particular environmental conditions. If you do not handle sensor synchronization, calibration, and labeling systematically at this stage, the data becomes unusable later.

### 17.4.1 Sensor Synchronization

If you do not time-synchronize data from multiple sensors, fusion itself is meaningless.

A 10 ms offset between camera and LiDAR timestamps produces a position error of tens of centimeters at high driving speeds. The basic premise of sensor fusion is "data from the same instant", and without synchronization that premise collapses.

**Hardware synchronization**:
- Simultaneous capture via trigger signals
- PPS (Pulse Per Second) signals

**Software synchronization**:
- Approximate synchronization based on timestamps
- Use of interpolation

**In ROS, filtering is only possible at reception time:**

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

**Camera Intrinsic**: use a checkerboard (OpenCV calibrateCamera).

**Camera-LiDAR Extrinsic**:
- Checkerboard-based (plane fitting)
- Target-based (using a special target)
- Target-less (automatic feature matching)

**Camera-IMU**: Kalibr is recommended.

If calibration is inaccurate, the object position seen by the camera and the one seen by the LiDAR do not match. Sensor fusion accuracy depends on calibration quality. In linear algebra terms, the intrinsic corresponds to the 3×3 camera matrix K, and the extrinsic corresponds to the 4×4 transformation matrix [R|t].

> **Further reading**
> - [OpenCV Camera Calibration Tutorial](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html) — checkerboard-based camera calibration.
> - [Kalibr GitHub](https://github.com/ethz-asl/kalibr) — standard tool for Camera-IMU calibration.

### 17.4.3 Labeling Tools

Once data is collected, you have to annotate it. Labeling is one of the most time-consuming tasks in research, and label quality determines model performance. Recently, semi-automatic labeling using foundation models such as SAM (Segment Anything Model) has become widespread.

**CVAT (Computer Vision Annotation Tool)**:
- Web-based, free
- Image and video annotation
- Supports various tasks (bbox, polygon, points)

**Labelbox**:
- Cloud-based
- Team collaboration features
- Supports 3D annotation

**3D Labeling**:
- SUSTechPOINTS: LiDAR point clouds
- KITTI-360 labeling tool

**Automatic labeling via synthetic data**: when you generate data in a simulator (NVIDIA Isaac Sim, AI2-THOR, and so on), labels are produced along with the data, so no manual labeling is needed. Domain randomization, which randomly varies texture, lighting, and background, can also improve a model's generalization. Collection cost is close to zero compared to real data.

> **Further reading**
> - [CVAT Documentation](https://docs.cvat.ai/) — official documentation for the open-source labeling tool.
> - [Roboflow](https://roboflow.com/) — integrated platform for labeling, data augmentation, and model training.
> - [NVIDIA Isaac Sim - Synthetic Data Generation](https://docs.omniverse.nvidia.com/isaacsim/latest/replicator_tutorials/index.html) — synthetic data generation guide.

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
