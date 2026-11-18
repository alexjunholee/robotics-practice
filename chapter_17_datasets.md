# 17장 데이터셋과 벤치마크

로보틱스 알고리즘의 일반화 성능과 공정한 비교 기준은 결국 데이터셋이 결정한다. 어떤 센서 조합을 달았는지, 어떤 날씨와 조명 조건에서 모았는지, 학습·검증 split과 annotation 규약을 어떻게 잡았는지에 따라 학습된 모델이 통하는 환경과 비교 가능한 경쟁 모델이 갈린다. 하지만 실제 로봇을 굴려 데이터를 모으고 사람이 일일이 라벨을 붙이는 데는 막대한 비용과 시간이 든다. 그래서 연구 현장에서는 시뮬레이터로 합성 데이터(Synthetic Data)를 먼저 만들어 사전학습을 돌린 뒤, 소량의 실제 데이터로 파인튜닝(fine-tuning)해 격차를 메우는 전략을 자주 쓴다. NVIDIA Isaac Sim에서 Domain Randomization으로 합성 영상을 뽑아내거나, Habitat에서 대규모 3D 스캔 장면을 렌더링해 학습 데이터를 확보하는 방식이 대표적이다.

## 17.1 자율주행/로봇 데이터셋

야외 도로를 달리는 자율주행차와 실내를 누비는 서비스 로봇·드론은 센서 구성과 평가 과제가 크게 다르다. 연구 목적에 맞는 공개 벤치마크를 고르려면 각 데이터셋이 어떤 센서 조합과 환경 조건을 담고 있는지부터 파악해야 한다.

### 17.1.1 KITTI / KITTI360

2012년 공개된 **KITTI**는 자율주행과 3D 비전 분야에서 오랫동안 공통 비교 기준으로 쓰여 온 벤치마크다. 스테레오 카메라와 3D LiDAR(Velodyne HDL-64E), GPS/IMU를 함께 실어 주행 데이터를 모았고, 2D와 3D 라벨을 모두 제공한다. 이 데이터셋 위에서 Stereo depth estimation, Optical flow, Visual odometry / SLAM, 3D object detection, Semantic segmentation 같은 핵심 태스크가 나란히 평가되며, 지금도 VO·SLAM이나 스테레오 깊이 추정 논문에서는 과거 연구와 선을 맞춰 비교하기 위해 KITTI를 기본으로 올린다. 이후 공개된 KITTI-360은 전방 중심이던 시야를 360도로 넓혀 더 폭넓은 장면 이해 연구에 쓰인다.

> **추천 자료**
> - [KITTI Benchmark 공식 사이트](https://www.cvlibs.net/datasets/kitti/) — 데이터셋 다운로드 및 각 태스크별 리더보드 확인
> - [KITTI-360 사이트](https://www.cvlibs.net/datasets/kitti-360/) — 더 넓은 범위의 360도 데이터셋
> - [다크 프로그래머 — KITTI 데이터 사용하기 (LiDAR-카메라 변환)](https://darkpgmr.tistory.com/190) — KITTI 데이터의 좌표계 변환과 LiDAR-카메라 매핑 실습

### 17.1.2 nuScenes

KITTI가 전방 카메라와 LiDAR 중심이었다면, Motional(당시 nuTonomy)이 2019년에 공개한 **nuScenes**는 차량 주변 360도를 감싸는 다중 센서 구성을 전면에 내세웠다. 차량 사방을 덮는 6개 카메라와 5개 Radar, 1개 LiDAR로 1000개 장면(40K 키프레임)을 수집했으며, 야간과 빗길 같은 악조건까지 폭넓게 담았다. 23개 객체 클래스에 대해 속성과 가시성까지 포함한 촘촘한 annotation을 제공하기 때문에, 최근 3D Object Detection과 BEV(Bird's Eye View) 인식 연구에서 표준 평가셋으로 자리 잡았다. 평가 지표로는 전통적인 mAP와 함께 속도·방향·속성 오차까지 종합해 매기는 전용 지표인 NDS(nuScenes Detection Score)를 사용한다.

> **추천 자료**
> - [nuScenes devkit Documentation](https://www.nuscenes.org/nuscenes) — 데이터셋 사용법, devkit API, 튜토리얼 노트북
> - [nuScenes devkit GitHub](https://github.com/nutonomy/nuscenes-devkit) — Python devkit 코드 및 예제

### 17.1.3 Waymo Open Dataset

2019년 Waymo(Alphabet 산하)가 공개한 **Waymo Open Dataset**은 압도적인 라벨 규모가 강점이다. 도심과 교외, 야간 환경을 아울러 20초 길이의 1,150개 장면을 담았고, 여기에 12M개의 LiDAR 라벨과 12M개의 카메라 라벨을 붙였다. 대규모 3D 인식과 모션 예측 모델을 학습·검증할 때 널리 쓰이며, 매년 챌린지를 열어 최신 기법들의 성능을 겨루는 장을 제공한다.

> **추천 자료**
> - [Waymo Open Dataset 공식 사이트](https://waymo.com/open/) — 데이터셋 다운로드 및 챌린지 참가
> - [Waymo Open Dataset GitHub](https://github.com/waymo-research/waymo-open-dataset) — 공식 도구 및 예제 코드

### 17.1.4 RGB-D SLAM 및 VIO / VINS용 데이터셋

실내 로봇이나 드론처럼 자율주행차보다 작고 빠른 플랫폼을 연구한다면 전용 벤치마크를 써야 한다. **TUM RGB-D**는 실내 환경에서 찍은 RGB-D 카메라 시퀀스에 모션 캡처 장비로 잰 정밀 ground truth 궤적을 함께 제공해 Visual SLAM 평가의 표준으로 통한다. 반면 **EuRoC MAV**는 드론이 실내 산업 공간을 비행하며 기록한 스테레오 영상과 IMU 데이터를 담고 있고 난이도별 시퀀스가 나뉘어 있어, 카메라와 IMU를 결합하는 VIO(Visual-Inertial Odometry, 카메라·IMU 결합 주행 거리 추정) 성능을 검증할 때 기준 데이터셋으로 쓰인다.

> **추천 자료**
> - [TUM RGB-D Benchmark](https://cvg.cit.tum.de/data/datasets/rgbd-dataset) — Visual SLAM 평가 표준 데이터셋 및 평가 도구
> - [EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) — VIO 평가 표준 데이터셋

## 17.2 컴퓨터 비전 데이터셋

로봇 전용 데이터셋만으로는 대규모 시각 표현을 바닥부터 학습시키기 어렵다. 그래서 로봇 인식 모듈도 출발점으로는 컴퓨터 비전 분야의 대형 데이터셋에서 사전학습한 가중치와 평가 기준을 그대로 빌려 오는 흐름이 일반적이다.

### 17.2.1 ImageNet

**ImageNet**은 1000개 클래스에 걸쳐 120만 장의 학습 이미지를 모은 대표적인 이미지 분류 벤치마크다. 2012년 AlexNet이 ImageNet 분류 대회(ILSVRC)에서 top-5 오류율을 극적으로 끌어내린 뒤로, ImageNet 사전학습(pretrained) 가중치는 거의 모든 비전 모델의 기본 초기값으로 쓰였다. 로보틱스에서도 카메라 기반 인식 모듈의 백본(backbone)을 세울 때 ImageNet 사전학습 모델에서 출발해 도메인 데이터에 맞게 미세 조정한다.

### 17.2.2 COCO

이미지 전체를 분류하는 단계를 넘어 화면 속 개별 물체의 위치와 윤곽까지 따져야 할 때는 **COCO**가 기준 역할을 한다. COCO는 80개 객체 카테고리에 걸쳐 33만 장의 이미지와 150만 개의 객체 인스턴스를 담고 있으며, bounding box와 segmentation mask 같은 촘촘한 annotation을 바탕으로 Object detection, Instance segmentation, Keypoint detection, Captioning을 함께 평가한다.

논문을 읽을 때는 평가 지표의 계산 방식 차이를 눈여겨보아야 한다. COCO의 대표 지표인 **COCO mAP**는 예측 영역과 정답 영역의 겹침 비율을 뜻하는 IoU(Intersection over Union) threshold를 하나로 고정하지 않고, 0.50부터 0.95까지 0.05 간격으로 10개 구간을 쪼개 각각 AP를 구한 뒤 평균한다. IoU 0.5 하나만으로 AP를 계산하던 과거 PASCAL VOC 기준보다 박스의 위치 정밀도를 훨씬 까다롭게 따지는 셈이다.

### 17.2.3 ScanNet / NYU Depth V2

가정용·서비스 로봇처럼 실내에서 움직이는 플랫폼은 2D 사진뿐 아니라 3차원 공간의 깊이와 물체 배치를 읽어야 한다. **ScanNet**은 실내 공간 707곳을 훑은 1513개의 RGB-D 스캔 시퀀스와 함께 카메라 포즈, 복원된 3D 메시, 3D semantic segmentation 라벨을 제공해 3D 장면 이해(Scene Understanding) 연구의 주력 벤치마크로 쓰인다. **NYU Depth V2**는 464개 실내 장면의 407K 프레임으로 이루어진 RGB-D 데이터셋으로, 단안 또는 실내 Depth estimation 모델의 성능을 비교할 때 자주 인용된다.

> **추천 자료**
> - [COCO Dataset](https://cocodataset.org/) — 공식 사이트, 데이터셋 다운로드 및 evaluation 도구
> - [ScanNet Benchmark](http://www.scan-net.org/) — 3D Scene Understanding 벤치마크

## 17.3 데이터셋 활용법

적당한 벤치마크를 골라 내려받았더라도 곧바로 학습 루프에 밀어 넣기는 어렵다. 데이터셋마다 폴더 구조와 라벨 형식, 좌표계 규약이 제각각이기 때문이다.

### 17.3.1 다운로드 및 포맷 이해

데이터 로더 코드를 짜기 전에 공식 문서에서 디렉터리 배치와 라벨 포맷부터 먼저 확인해야 한다. 특히 3D 바운딩 박스나 포인트 클라우드 라벨은 데이터셋마다 기준 좌표계(카메라 좌표계인지 LiDAR 좌표계인지, y-up인지 z-up인지)가 달라서, 축 방향 하나만 잘못 가정해도 객체가 바닥에 눕거나 엉뚱한 곳에 찍힌다.

예를 들어 KITTI Object Detection 데이터셋은 아래처럼 센서별 폴더와 캘리브레이션, 라벨 폴더가 나란히 놓인다.

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

`label_2/` 안의 텍스트 파일은 한 줄이 객체 하나에 해당하며, 공백으로 나뉜 열 순서대로 클래스, 가림 정도, 2D 박스, 3D 크기와 중심 위치, 회전각을 담고 있다. 파이썬으로 필요한 2D·3D 정보를 파싱하는 기본 코드는 다음과 같다.

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

여기서 `dimensions`가 `(height, width, length)` 순이고 `location`은 카메라 좌표계 기준 객체 바닥 중심이라는 세부 정의까지 문서에서 함께 확인해야 뒤이은 3D 투영 계산이 어긋나지 않는다.

> **추천 자료**
> - [KITTI Benchmark 공식 사이트 - Object Detection DevKit](https://www.cvlibs.net/datasets/kitti/eval_object.php?obj_benchmark=3d) — 라벨 포맷 설명 및 평가 코드
> - [nuScenes devkit Tutorial Notebooks](https://github.com/nutonomy/nuscenes-devkit/tree/master/python-sdk/tutorials) — Jupyter 노트북으로 데이터 구조 이해

### 17.3.2 DataLoader 구현

포맷을 파악했다면 이제 학습 루프가 GPU를 놀리지 않고 데이터를 계속 받아먹을 수 있게 로더를 짠다. PyTorch에서는 한 샘플을 디스크에서 읽어 전처리하는 책임을 `Dataset` 클래스에 두고, 여러 샘플을 묶어 배치를 만들고 백그라운드 프로세스로 미리 읽어 오는 일은 `DataLoader`에 맡긴다. `__getitem__` 안의 디코딩·증강 연산이 얼마나 무거운지, `num_workers`를 몇 개로 잡는지에 따라 전체 학습 처리량이 크게 달라진다.

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

커스텀 데이터셋에서 이미지 크기가 제각각이거나 포인트 클라우드 점 개수가 샘플마다 다르다면 `DataLoader`에 `collate_fn`을 직접 넘겨 배치 묶기 규칙을 정의한다.

> **추천 자료**
> - [PyTorch Data Loading Tutorial](https://docs.pytorch.org/tutorials/beginner/data_loading_tutorial.html) — 커스텀 Dataset 작성 공식 가이드
> - [PyTorch 공식 문서 — torch.utils.data](https://docs.pytorch.org/docs/stable/data.html) — DataLoader의 batch·sampler·`num_workers`·`collate_fn` 옵션 레퍼런스

## 17.4 자체 데이터 수집

공개 벤치마크만으로는 내 로봇의 고유한 센서 배치나 특수한 작업 환경(예: 어두운 창고, 비정형 농장, 특정 공장 라인)을 충분히 대변하지 못할 때가 많다. 결국 직접 데이터를 모아야 한다면, 수집 버튼을 누르기 전부터 센서 동기화, 캘리브레이션, 라벨링 절차를 한 묶음으로 설계해 두어야 나중에 쓸 수 있는 데이터가 쌓인다.

### 17.4.1 센서 동기화

다중 센서 융합은 카메라, LiDAR, IMU가 "같은 순간"에 세상을 측정했다는 가정 위에서 돌아간다. 하지만 타임스탬프가 조금만 어긋나도 고속 이동 환경에서는 오차가 순식간에 커진다. 예를 들어 30 m/s(108 km/h)로 달리는 차량에서 카메라와 LiDAR의 시각이 단 10 ms만 벌어져도, 두 센서가 바라보는 같은 물체의 위치는 약 30 cm나 어긋난다.

가장 확실한 해법은 전기 신호로 물리적 촬영 시점을 맞추는 **하드웨어 동기화**다. 외부 트리거 신호로 카메라 셔터와 LiDAR 스캔을 동시에 맞추거나, GPS 수신기의 PPS(Pulse Per Second) 신호로 모든 센서 보드의 시계를 동기화한다. 하드웨어 트리거를 지원하지 않는 저가형 센서를 섞어 쓸 때는 **소프트웨어 동기화**에 기댄다. 각 메시지에 찍힌 타임스탬프를 기준으로 가장 가까운 쌍을 짝짓거나, IMU·자세 값을 시간축으로 보간(interpolation)해 기준 시각에 맞추는 방식이다.

ROS에서는 `message_filters` 패키지가 이 타임스탬프 정렬을 맡는다. 메시지 header의 `stamp` 필드를 기준으로 묶으며, header가 없는 메시지라면 `allow_headerless` 옵션을 켜서 메시지가 도착한 시점을 대신 쓸 수 있다. 아래 코드는 이미지와 포인트 클라우드 토픽을 최대 0.1초(`slop=0.1`) 허용 오차 안에서 근사 동기화해 콜백으로 넘기는 예시다.

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

`slop`을 너무 크게 잡으면 빠른 회전 구간에서 어긋난 프레임끼리 묶이고, 너무 좁히면 콜백이 아예 불리지 않으므로 센서 주기와 로봇 속도를 보고 적정값을 고른다.

### 17.4.2 캘리브레이션

시간을 맞췄다면 이제 센서 사이의 공간적 위치와 렌즈 왜곡을 맞출 차례다. 캘리브레이션에 오차가 남으면 카메라 영상에서 검출한 보행자와 LiDAR 포인트 클라우드가 서로 빗나가 융합 성능이 그대로 무너진다.

먼저 카메라 한 대의 초점거리, 주점, 렌즈 왜곡을 구하는 **Camera Intrinsic** 캘리브레이션은 체커보드 패턴을 여러 각도에서 찍어 OpenCV의 `calibrateCamera`로 추정한다. 수학적으로 내재 파라미터(intrinsic)는 3×3 카메라 행렬 K로 요약된다. 서로 다른 센서 사이의 상대 위치와 회전(**Extrinsic**)을 구할 때는 조합에 따라 도구가 달라진다. **Camera-LiDAR Extrinsic**은 체커보드 평면을 두 센서에서 동시에 잡아 평면을 맞추거나(target-based), 특수 타겟 없이 자연 환경의 모서리·특징점을 자동 매칭하는 target-less 기법을 쓴다. 카메라와 IMU를 함께 보정하는 **Camera-IMU** 캘리브레이션에는 **Kalibr**가 표준 도구로 널리 쓰인다. 구해진 extrinsic을 카메라 투영식에 넣을 때는 3×4 블록 [R|t] 형태로 쓰고, 센서 사이의 3차원 좌표를 직접 변환할 때는 마지막에 [0 0 0 1] 행을 붙여 4×4 동차 변환 행렬로 만들어 계산한다.

> **추천 자료**
> - [OpenCV Camera Calibration Tutorial](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html) — 체커보드 기반 카메라 캘리브레이션
> - [Kalibr GitHub](https://github.com/ethz-asl/kalibr) — Camera-IMU 캘리브레이션 표준 도구

### 17.4.3 라벨링 도구

동기화와 캘리브레이션을 거쳐 원시 데이터를 모았다면 이제 정답 라벨(annotation)을 붙일 차례다. 수동 라벨링은 시간과 인력이 가장 많이 드는 병목이자 모델 성능을 좌우하는 핵심 공정이다. 최근에는 2023년 공개된 SAM(Segment Anything Model)을 도구의 보조 엔진으로 붙여, 사람이 점이나 박스만 몇 번 찍으면 윤곽 마스크를 자동으로 따 주는 반자동 라벨링이 널리 퍼졌다.

작업 대상과 팀 규모에 따라 주로 선택하는 도구는 다음과 같다.

- **CVAT (Computer Vision Annotation Tool)**: 웹 기반으로 무료로 띄워 쓸 수 있어 이미지와 비디오 annotation에 가장 널리 쓰인다. Bounding box, polygon, points 등 다양한 2D 태스크를 폭넓게 지원한다.
- **Labelbox**: 클라우드 기반 플랫폼으로, 여러 작업자가 라벨을 나누어 붙이고 검수하는 팀 협업 기능과 3D annotation 도구를 함께 제공한다.
- **3D Labeling 전용 도구**: LiDAR 포인트 클라우드에 3D 바운딩 박스를 칠 때는 `SUSTechPOINTS`나 `KITTI-360 labeling tool` 같은 전용 3D 라벨러가 작업 속도를 높여 준다.

수동 라벨링 부담을 근본적으로 줄이려면 **합성 데이터로 라벨을 자동 생성**하는 방법도 고려한다. NVIDIA Isaac Sim이나 AI2-THOR 같은 시뮬레이터에서 장면을 렌더링하면 2D/3D 박스와 세그멘테이션 마스크, 깊이 지도가 오차 없는 정답으로 함께 떨어진다. 이때 텍스처, 조명, 배경을 무작위로 바꾸며 데이터를 뽑아내는 **Domain Randomization** 기법을 쓰면 시뮬레이션과 현실의 차이를 상당 부분 넘길 수 있으며, 실제로 Tobin et al.(2017)은 이렇게 생성한 시뮬레이션 영상만으로 학습한 물체 위치 검출기가 실제 로봇에서도 동작함을 보였다. 물론 실제 데이터를 모으고 손으로 라벨링하는 수고는 줄어들지만, 그 대신 시뮬레이터 환경을 꾸미고 대규모 렌더링을 돌리며 생성된 데이터의 물리적 타당성을 검수하는 새로운 엔지니어링 비용이 따라온다는 점은 함께 계산해 두어야 한다.

> **추천 자료**
> - [CVAT Documentation](https://docs.cvat.ai/) — 오픈소스 라벨링 도구 공식 문서
> - [Roboflow](https://roboflow.com/) — 라벨링, 데이터 증강, 모델 학습을 한 서비스에서 처리하는 플랫폼
> - [NVIDIA Isaac Sim - Synthetic Data Generation](https://docs.isaacsim.omniverse.nvidia.com/latest/replicator_tutorials/index.html) — 합성 데이터 생성 가이드

프레임워크와 개발 환경, 데이터셋 파이프라인이 모두 준비됐다면 이제 실제 로봇 앞에서 코드를 고치고 런타임 오류를 진단하는 속도를 끌어올릴 차례다. 다음 장(→ 18장)에서는 AI 코딩 에이전트에게 로봇의 실시간 상태와 로그를 정확히 넘겨 문제 해결에 활용하는 실전 절차를 살펴본다.

## 발전 과정

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
