# Ch.17 — 데이터셋 & 벤치마크

데이터셋의 센서 구성, 수집 조건, split, annotation 방식에 따라 학습한 모델이 어떤 환경까지 통하는지, 어떤 결과끼리 비교할 수 있는지가 정해진다.

실제 데이터를 모으고 라벨을 붙이는 데는 비용과 시간이 많이 든다. 그래서 시뮬레이터로 합성 데이터(Synthetic Data)를 만들어 사전학습하고, 적은 양의 실제 데이터로 파인튜닝(fine-tuning)하는 방식을 함께 쓴다. NVIDIA Isaac Sim에서 Domain Randomization으로 합성 영상을 만들거나, Habitat에서 대규모 3D 스캔 장면을 렌더링해 학습 데이터를 만드는 것이 그 예다.

## 17.1 자율주행/로봇 데이터셋

### 17.1.1 KITTI / KITTI360

KITTI는 2012년 공개 뒤 자율주행과 3D 비전의 공통 비교 기준으로 쓰여 왔다. 연구자들은 지금도 VO·SLAM과 stereo depth estimation 결과를 과거 연구와 비교할 때 KITTI를 쓴다.

구성:
- 스테레오 카메라
- 3D LiDAR (Velodyne HDL-64E)
- GPS/IMU
- 2D/3D 라벨

태스크:
- Stereo depth estimation
- Optical flow
- Visual odometry / SLAM
- 3D object detection
- Semantic segmentation

> **추천 자료**
> - [KITTI Benchmark 공식 사이트](https://www.cvlibs.net/datasets/kitti/) — 데이터셋 다운로드 및 각 태스크별 리더보드 확인
> - [KITTI-360 사이트](https://www.cvlibs.net/datasets/kitti-360/) — 더 넓은 범위의 360도 데이터셋
> - [다크 프로그래머 — KITTI 데이터 사용하기 (LiDAR-카메라 변환)](https://darkpgmr.tistory.com/190) — KITTI 데이터의 좌표계 변환과 LiDAR-카메라 매핑 실습

### 17.1.2 nuScenes

Motional(당시 nuTonomy)이 2019년에 공개한 자율주행 데이터셋이다. 360도 카메라와 Radar를 갖췄고, 3D Object Detection과 BEV(Bird's Eye View) 인식 연구에서 평가셋으로 많이 쓰인다.

구성:
- 6개 카메라 (360° 커버)
- 5개 Radar
- 1개 LiDAR
- 1000 장면, 40K 키프레임

특징:
- 23개 객체 클래스
- 풍부한 Annotation (속성, 가시성)
- 밤, 비 등 다양한 조건

평가 메트릭: mAP, NDS(nuScenes Detection Score)

> **추천 자료**
> - [nuScenes devkit Documentation](https://www.nuscenes.org/nuscenes) — 데이터셋 사용법, devkit API, 튜토리얼 노트북
> - [nuScenes devkit GitHub](https://github.com/nutonomy/nuscenes-devkit) — Python devkit 코드 및 예제

### 17.1.3 Waymo Open Dataset

Waymo(Alphabet 산하)가 2019년에 공개한 자율주행 데이터셋이다.

규모:
- 1,150 장면 (20초)
- 12M LiDAR 라벨
- 12M 카메라 라벨

특징:
- 다양한 환경 (도시, 교외, 밤)
- 연간 챌린지 개최

> **추천 자료**
> - [Waymo Open Dataset 공식 사이트](https://waymo.com/open/) — 데이터셋 다운로드 및 챌린지 참가
> - [Waymo Open Dataset GitHub](https://github.com/waymo-research/waymo-open-dataset) — 공식 도구 및 예제 코드

### 17.1.4 RGB-D SLAM 및 VIO / VINS용 데이터셋

TUM RGB-D는 실내 RGB-D SLAM 평가에, EuRoC MAV는 드론 VIO(Visual-Inertial Odometry, 카메라·IMU 결합 주행 거리 추정) 평가에 많이 쓴다.

**TUM RGB-D**:
- RGB-D 카메라 시퀀스
- 정밀 ground truth (모션 캡처)
- 실내 환경
- Visual SLAM 평가 표준

**EuRoC MAV**:
- 드론 비행 데이터
- 스테레오 + IMU
- VIO 평가 표준
- 다양한 난이도

> **추천 자료**
> - [TUM RGB-D Benchmark](https://cvg.cit.tum.de/data/datasets/rgbd-dataset) — Visual SLAM 평가 표준 데이터셋 및 평가 도구
> - [EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) — VIO 평가 표준 데이터셋

## 17.2 컴퓨터 비전 데이터셋

### 17.2.1 ImageNet

이미지 분류 벤치마크다. 2012년 AlexNet이 ImageNet 분류 대회(ILSVRC)에서 top-5 오류율을 크게 낮춘 뒤, ImageNet 사전학습(pretrained) 가중치는 여러 비전 모델의 초기값으로 쓰였다. 로보틱스에서도 카메라 기반 인식 모듈의 백본(backbone)에 ImageNet 사전학습 모델을 널리 쓴다.

- 1000 클래스
- 120만 학습 이미지
- 사전학습 표준

### 17.2.2 COCO

COCO는 객체 탐지와 instance segmentation을 함께 평가한다.

COCO mAP는 여러 IoU(Intersection over Union, 예측 영역과 정답 영역의 겹침 비율) threshold에서 AP를 계산해 평균한다. 기본값은 IoU 0.50부터 0.95까지 0.05 간격의 10개 threshold다. PASCAL VOC는 IoU 0.5 하나로 AP를 계산한다.

특징:
- 80 객체 카테고리
- 33만 이미지, 150만 객체 인스턴스
- Dense annotation (bounding box, segmentation mask)

태스크:
- Object detection
- Instance segmentation
- Keypoint detection
- Captioning

### 17.2.3 ScanNet / NYU Depth V2

**ScanNet**:
- 실내 공간 707곳을 찍은 RGB-D 스캔 1513개
- RGB-D 시퀀스
- 3D semantic segmentation
- 카메라 포즈, 메시 제공

**NYU Depth V2**:
- 실내 RGB-D
- Depth estimation 벤치마크
- 464 장면, 407K 프레임

가정용·서비스 로봇처럼 실내에서 움직이는 로봇을 다룰 때 ScanNet과 NYU Depth V2로 성능을 비교한다. 3D 장면 이해(Scene Understanding) 연구에는 ScanNet을 많이 쓴다.

> **추천 자료**
> - [COCO Dataset](https://cocodataset.org/) — 공식 사이트, 데이터셋 다운로드 및 evaluation 도구
> - [ScanNet Benchmark](http://www.scan-net.org/) — 3D Scene Understanding 벤치마크

## 17.3 데이터셋 활용법

### 17.3.1 다운로드 및 포맷 이해

데이터셋마다 디렉터리 구조와 라벨 포맷이 다르다. 데이터 로더를 짜기 전에 문서에서 이 둘을 먼저 확인한다. 3D 라벨은 데이터셋마다 기준 좌표계(카메라 좌표계나 LiDAR 좌표계, y-up이나 z-up)가 달라서 특히 꼼꼼히 읽는다.

예시 — KITTI Object Detection:

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

라벨 파일 읽기 예시:

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

> **추천 자료**
> - [KITTI Benchmark 공식 사이트 - Object Detection DevKit](https://www.cvlibs.net/datasets/kitti/eval_object.php?obj_benchmark=3d) — 라벨 포맷 설명 및 평가 코드
> - [nuScenes devkit Tutorial Notebooks](https://github.com/nutonomy/nuscenes-devkit/tree/master/python-sdk/tutorials) — Jupyter 노트북으로 데이터 구조 이해

### 17.3.2 DataLoader 구현

PyTorch에서 샘플 읽기와 전처리는 `Dataset`에 정의한다. batch 구성과 병렬 로딩은 `DataLoader`가 맡는다. `__getitem__`의 전처리 비용과 `num_workers` 값에 따라 학습 처리량이 달라진다.

```python
from torch.utils.data import Dataset, DataLoader

class MyDataset(Dataset):
    def __init__(self, root_dir, transform=None):
        self.root_dir = root_dir
        self.transform = transform
        self.samples = self._load_samples()

    def _load_samples(self):
        # 파일 목록 로드
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

# 사용
dataset = MyDataset(root_dir='./data')
dataloader = DataLoader(dataset, batch_size=32, shuffle=True, num_workers=4)
```

> **추천 자료**
> - [PyTorch Data Loading Tutorial](https://docs.pytorch.org/tutorials/beginner/data_loading_tutorial.html) — 커스텀 Dataset 작성 공식 가이드
> - [PyTorch 공식 문서 — torch.utils.data](https://docs.pytorch.org/docs/stable/data.html) — DataLoader의 batch·sampler·`num_workers`·`collate_fn` 옵션 레퍼런스

## 17.4 자체 데이터 수집

자기 로봇의 센서 구성이나 특수한 환경 조건에 맞는 데이터가 필요하면 직접 수집한다. 수집할 때는 센서 동기화, 캘리브레이션, 라벨링을 처음부터 정해 둔 절차대로 진행한다.

### 17.4.1 센서 동기화

센서 퓨전은 여러 센서가 같은 시점에 잰 데이터를 합친다고 가정한다. 카메라와 LiDAR의 타임스탬프가 10 ms 어긋나면, 30 m/s(108 km/h)로 달리는 차에서 약 30 cm의 위치 오차가 생긴다.

하드웨어 동기화:
- 트리거 신호로 동시 촬영
- PPS (Pulse Per Second) 신호

소프트웨어 동기화:
- 타임스탬프 기반 근사 동기화
- 보간(interpolation) 사용

ROS의 `message_filters`는 메시지 header의 stamp를 기준으로 동기화한다. header가 없는 메시지에 `allow_headerless`를 지정하면 도착 시점을 쓴다:

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

### 17.4.2 캘리브레이션

Camera Intrinsic: 체커보드 사용 (OpenCV calibrateCamera)

Camera-LiDAR Extrinsic:
- 체커보드 기반 (평면 맞춤)
- Target-based (특수 타겟 사용)
- Target-less (자동 특징점 매칭)

Camera-IMU: Kalibr 사용 권장

캘리브레이션 오차가 있으면 카메라에서 본 객체 위치와 LiDAR에서 본 객체 위치가 어긋나고, 그만큼 퓨전 결과도 틀어진다. 행렬로 쓰면 intrinsic은 3×3 카메라 행렬 K다. extrinsic을 카메라 투영에 넣을 때는 3×4 블록 [R|t]로 쓴다. 센서 간 좌표 변환에 쓸 때는 여기에 [0 0 0 1] 행을 붙여 4×4 동차 변환으로 만든다.

> **추천 자료**
> - [OpenCV Camera Calibration Tutorial](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html) — 체커보드 기반 카메라 캘리브레이션
> - [Kalibr GitHub](https://github.com/ethz-asl/kalibr) — Camera-IMU 캘리브레이션 표준 도구

### 17.4.3 라벨링 도구

수집한 데이터에는 라벨(annotation)을 붙인다. 라벨링에는 시간이 많이 들고, 라벨 품질에 따라 모델 성능이 달라진다. 2023년 SAM(Segment Anything Model)이 공개된 뒤로 SAM을 쓴 반자동 라벨링 도구가 나왔다.

**CVAT (Computer Vision Annotation Tool)**:
- 웹 기반, 무료
- 이미지, 비디오 annotation
- 다양한 태스크 지원 (bbox, polygon, points)

**Labelbox**:
- 클라우드 기반
- 팀 협업 기능
- 3D annotation 지원

**3D Labeling**:
- SUSTechPOINTS: LiDAR 포인트 클라우드
- KITTI-360 labeling tool

합성 데이터로 라벨 자동 생성: 시뮬레이터(NVIDIA Isaac Sim, AI2-THOR 등)에서 데이터를 만들면 라벨이 함께 나와 수동 라벨링이 줄어든다. Domain Randomization은 텍스처, 조명, 배경을 무작위로 바꿔 가며 데이터를 만든다. Tobin et al.(2017)은 이렇게 만든 시뮬레이션 영상만으로 학습한 물체 위치 검출기가 실제 로봇에서도 동작함을 보였다. 실제 데이터 수집과 수동 라벨링 비용은 줄지만, 시뮬레이터를 꾸미고 렌더링하고 결과를 검수하는 비용이 새로 든다.

> **추천 자료**
> - [CVAT Documentation](https://docs.cvat.ai/) — 오픈소스 라벨링 도구 공식 문서
> - [Roboflow](https://roboflow.com/) — 라벨링, 데이터 증강, 모델 학습을 한 서비스에서 처리하는 플랫폼
> - [NVIDIA Isaac Sim - Synthetic Data Generation](https://docs.isaacsim.omniverse.nvidia.com/latest/replicator_tutorials/index.html) — 합성 데이터 생성 가이드

## 기술 흐름: 데이터셋 & 벤치마크의 과거 → 현재 → 미래

```
2009 ─── ImageNet 공개
  │       WordNet 계층에 맞춰 모은 대규모 이미지 데이터베이스 (CVPR 2009)
  │
2012 ─── KITTI 공개 / AlexNet의 ImageNet 제패
  │
2014 ─── COCO 공개
  │
2017 ─── ScanNet 공개
  │
2019 ─── nuScenes, Waymo Open Dataset 공개
  │
2021 ─── Omniverse 기반 NVIDIA Isaac Sim 공개 (2021.1)
  │
2023 ─── SA-1B, Open X-Embodiment 공개
  │       SA-1B (SAM 학습용, 약 11억 마스크)
  │       Open X-Embodiment (로봇 조작 데이터 통합)
  │
2024 ─── DROID 공개 (RSS 2024)
          Franka 로봇 조작 시연 76k 궤적(350시간), 564개 장면에서 수집
```
