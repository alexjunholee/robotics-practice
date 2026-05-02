# Ch.17 — 데이터셋 & 벤치마크

로보틱스와 컴퓨터 비전 연구에서 데이터셋은 알고리즘만큼 중요하다. 좋은 데이터 없이는 좋은 모델을 만들 수 없고, 공정한 벤치마크 없이는 논문에서 자기 방법이 진짜 좋은지 증명할 수 없다. 여기서는 주요 데이터셋의 구성과 특징, 자체 데이터 수집·관리 방법을 본다.

최근 **합성 데이터(Synthetic Data)**의 비중이 높아지고 있다. 실제 데이터 수집과 라벨링은 비용과 시간이 많이 드는데, 시뮬레이터에서 자동 생성한 합성 데이터로 사전학습한 뒤 소량의 실제 데이터로 미세조정(fine-tuning)하는 방식이 자리를 잡았다. NVIDIA Isaac Sim의 Domain Randomization이나 Habitat의 대규모 장면 생성이 대표적이다. **Sim-to-Real 데이터셋** — 시뮬레이터 데이터와 대응하는 실제 데이터를 쌍으로 제공하는 데이터셋 — 도 활발히 구축되고 있다.

## 17.1 자율주행/로봇 데이터셋

### 17.1.1 KITTI / KITTI360

자율주행 연구의 시작점이 된 오래된 데이터셋이다.

KITTI는 2012년에 공개된 이후 자율주행·3D 비전 연구의 사실상 표준 벤치마크 역할을 해 왔다. 비록 지금은 더 크고 다양한 데이터셋이 있지만, 2024년 기준으로도 주요 VO·SLAM 논문들이 KITTI 결과를 보고하므로 기준점으로 알아 두어야 한다. 특히 Visual Odometry, Stereo Depth Estimation 분야에서는 아직도 KITTI가 1차 벤치마크이다.

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

다운로드: https://www.cvlibs.net/datasets/kitti/

> **추천 자료**
> - [KITTI Benchmark 공식 사이트](https://www.cvlibs.net/datasets/kitti/) — 데이터셋 다운로드 및 각 태스크별 리더보드 확인
> - [KITTI-360 사이트](https://www.cvlibs.net/datasets/kitti-360/) — 더 넓은 범위의 360도 데이터셋
> - [다크 프로그래머 — KITTI 데이터 사용하기 (LiDAR-카메라 변환)](https://darkpgmr.tistory.com/190) — KITTI 데이터의 좌표계 변환과 LiDAR-카메라 매핑 실습

### 17.1.2 nuScenes

대규모 자율주행 데이터셋이다.

KITTI보다 센서 구성이 더 풍부하고(360도 카메라, Radar 포함), 데이터 규모도 훨씬 크다. 3D Object Detection과 BEV(Bird's Eye View) 기반 인식 연구에서 KITTI와 함께 표준 평가셋으로 자리잡았다.

구성:
- 6개 카메라 (360° 커버)
- 5개 Radar
- 1개 LiDAR
- 1000 장면, 40K 키프레임

특징:
- 23개 객체 클래스
- 풍부한 Annotation (속성, 가시성)
- 밤, 비 등 다양한 조건

평가 메트릭: mAP, NDS

> **추천 자료**
> - [nuScenes devkit Documentation](https://www.nuscenes.org/nuscenes) — 데이터셋 사용법, devkit API, 튜토리얼 노트북
> - [nuScenes devkit GitHub](https://github.com/nutonomy/nuscenes-devkit) — Python devkit 코드 및 예제

### 17.1.3 Waymo Open Dataset

Google의 대규모 자율주행 데이터셋이다.

nuScenes와 함께 최신 자율주행 연구의 양대 벤치마크이다. 데이터 품질과 규모 면에서 가장 앞서 있으며, 매년 챌린지를 통해 최신 기술 동향을 파악할 수 있다.

규모:
- 1,150 장면 (20초)
- 12M LiDAR 라벨
- 12M 카메라 라벨

특징:
- 높은 품질의 센서
- 다양한 환경 (도시, 교외, 밤)
- 연간 챌린지 개최

> **추천 자료**
> - [Waymo Open Dataset 공식 사이트](https://waymo.com/open/) — 데이터셋 다운로드 및 챌린지 참가
> - [Waymo Open Dataset GitHub](https://github.com/waymo-research/waymo-open-dataset) — 공식 도구 및 예제 코드

### 17.1.4 VIO / VINS용 데이터셋

Visual-Inertial Odometry(VIO)나 SLAM 연구를 한다면 아래 데이터셋은 알아야 한다. VIO·SLAM 논문이 평가셋으로 이 두 데이터셋을 빠뜨리는 경우는 드물다.

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

이미지 분류의 표준 벤치마크이다.

딥러닝 전환기의 출발점이 된 데이터셋이다. 2012년 AlexNet이 ImageNet에서 압도적 성능을 보여준 이후, 거의 모든 비전 모델이 ImageNet 사전학습(pretrained) 가중치를 쓰기 시작했다. 로보틱스에서도 카메라 기반 인식 모듈의 백본(backbone)은 대부분 ImageNet에서 사전학습된 모델을 가져다 쓴다.

- 1000 클래스
- 120만 학습 이미지
- 사전학습(pretraining) 표준

### 17.2.2 COCO

객체 탐지, 세그멘테이션의 표준이다.

Object detection 연구를 한다면 COCO 데이터셋의 평가 메트릭(COCO mAP)은 업계 표준이므로 이해해야 한다. IoU threshold별 AP를 계산하는 방식이 PASCAL VOC와 다르니 주의할 것.

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
- 1513개 실내 장면
- RGB-D 시퀀스
- 3D semantic segmentation
- 카메라 포즈, 메시 제공

**NYU Depth V2**:
- 실내 RGB-D
- Depth estimation 벤치마크
- 464 장면, 407K 프레임

실내 로봇(가정용, 서비스 로봇 등)을 한다면 ScanNet과 NYU Depth V2는 핵심 벤치마크이다. 특히 ScanNet은 3D 장면 이해(Scene Understanding) 연구에서 빠지지 않는다.

> **추천 자료**
> - [COCO Dataset](https://cocodataset.org/) — 공식 사이트, 데이터셋 다운로드 및 evaluation 도구
> - [ScanNet Benchmark](http://www.scan-net.org/) — 3D Scene Understanding 벤치마크
> - [Papers With Code - Datasets](https://paperswithcode.com/datasets) — 태스크별 데이터셋 검색 및 리더보드 통합 사이트

## 17.3 데이터셋 활용법

### 17.3.1 다운로드 및 포맷 이해

각 데이터셋마다 고유한 디렉토리 구조와 포맷이 있다.

데이터셋을 다운로드했는데 디렉토리 구조와 라벨 포맷을 제대로 이해하지 못하면, 데이터 로더를 짜는 데만 며칠이 걸릴 수 있다. 특히 3D 라벨은 좌표계(coordinate system)가 데이터셋마다 다르므로(카메라 좌표계 vs LiDAR 좌표계, y-up vs z-up 등) 문서를 꼼꼼히 읽어야 한다.

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

PyTorch에서 데이터 로딩을 위한 표준 패턴이다.

PyTorch의 `Dataset`과 `DataLoader` 패턴을 모르면 학습 코드를 짤 수 없다. 특히 `__getitem__`에서 데이터를 어떻게 전처리하느냐, `num_workers`를 몇으로 설정하느냐에 따라 학습 속도가 크게 달라질 수 있다.

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
> - [PyTorch Data Loading Tutorial](https://pytorch.org/tutorials/beginner/data_loading_tutorial.html) — 커스텀 Dataset 작성 공식 가이드
> - [Real Python - PyTorch DataLoader](https://realpython.com/python-data-loading/) — DataLoader 활용법 상세 설명

## 17.4 자체 데이터 수집

공개 데이터셋만으로는 자기 연구에 딱 맞는 데이터를 구하기 어렵다. 자체 로봇에 맞는 센서 구성, 특수한 환경 조건을 위해 직접 데이터를 수집해야 할 때가 있다. 이때 센서 동기화, 캘리브레이션, 라벨링 과정을 체계적으로 해 두지 않으면 나중에 데이터를 쓸 수 없게 된다.

### 17.4.1 센서 동기화

여러 센서의 데이터를 시간 동기화하지 않으면 퓨전 자체가 의미가 없다.

카메라와 LiDAR의 타임스탬프가 10ms만 어긋나도 고속 주행 시 수십 cm의 위치 오차가 생긴다. 센서 퓨전의 기본 전제가 "같은 시점의 데이터"인데, 동기화가 안 되면 그 전제가 무너진다.

하드웨어 동기화:
- 트리거 신호로 동시 촬영
- PPS (Pulse Per Second) 신호

소프트웨어 동기화:
- 타임스탬프 기반 근사 동기화
- 보간(interpolation) 사용

ROS에서는 받는 시점만 필터링 가능:

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

캘리브레이션이 부정확하면 카메라에서 본 객체 위치와 LiDAR에서 본 객체 위치가 일치하지 않는다. 센서 퓨전 정확도는 캘리브레이션 품질에 달려 있다. 선형대수 기준으로 보면, intrinsic은 3×3 카메라 행렬 K, extrinsic은 4×4 변환 행렬 [R|t]에 해당한다.

> **추천 자료**
> - [OpenCV Camera Calibration Tutorial](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html) — 체커보드 기반 카메라 캘리브레이션
> - [Kalibr GitHub](https://github.com/ethz-asl/kalibr) — Camera-IMU 캘리브레이션 표준 도구

### 17.4.3 라벨링 도구

데이터를 수집했으면 라벨링(annotation)을 해야 한다. 라벨링은 연구에서 가장 시간이 많이 드는 작업 중 하나이며, 라벨 품질이 모델 성능을 좌우한다. SAM(Segment Anything Model) 같은 기초 모델을 활용한 반자동 라벨링도 2023년 이후 연구 환경에서 쓰이기 시작했다.

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

합성 데이터를 통한 자동 라벨링: 시뮬레이터(NVIDIA Isaac Sim, AI2-THOR 등)에서 데이터를 생성하면 라벨도 함께 만들어지므로 수동 라벨링이 필요 없다. Domain Randomization으로 텍스처, 조명, 배경을 무작위 변형하면 모델의 일반화 성능도 높일 수 있다. 실제 데이터 대비 수집 비용이 거의 0에 가깝다.

> **추천 자료**
> - [CVAT Documentation](https://docs.cvat.ai/) — 오픈소스 라벨링 도구 공식 문서
> - [Roboflow](https://roboflow.com/) — 라벨링, 데이터 증강, 모델 학습을 통합 제공하는 플랫폼
> - [NVIDIA Isaac Sim - Synthetic Data Generation](https://docs.omniverse.nvidia.com/isaacsim/latest/replicator_tutorials/index.html) — 합성 데이터 생성 가이드

## 기술 흐름: 데이터셋 & 벤치마크의 과거 → 현재 → 미래

```
2009 ─── ImageNet 공개
  │       대규모 이미지 분류 벤치마크의 시작
  │
2012 ─── KITTI 공개 / AlexNet의 ImageNet 제패
  │       자율주행 벤치마크의 탄생, 딥러닝 혁명 시작
  │
2014 ─── COCO 공개
  │       Object Detection, Segmentation 표준 벤치마크
  │
2017 ─── ScanNet 공개
  │       실내 3D Scene Understanding 연구 활성화
  │
2019 ─── nuScenes, Waymo Open Dataset 공개
  │       대규모·고품질 자율주행 데이터셋 시대
  │
2020 ─── 합성 데이터 연구 본격화
  │       Domain Randomization, Sim-to-Real Transfer
  │       NVIDIA Isaac Sim 기반 대규모 합성 데이터 생성
  │
2023 ─── Foundation Model 시대의 데이터셋
  │       SA-1B (SAM 학습용, 10억 마스크)
  │       Open X-Embodiment (로봇 조작 데이터 통합)
  │
2024+ ── 데이터셋의 미래 트렌드
          합성 데이터 + 실제 데이터 혼합 학습 보편화
          Sim-to-Real 데이터셋 (시뮬·실제 쌍 데이터)
          자동 라벨링 (Foundation Model 기반)
          로봇 조작 데이터의 대규모 수집·공유 (Open X-Embodiment)
          멀티모달 데이터셋 (비전 + 언어 + 촉각 + 힘/토크)
```
