# Ch.2 — 센서 (Sensors)

로봇이 환경을 인식하려면 센서가 필요하다. 각 센서의 특성을 이해해야 적절한 선택과 알고리즘 설계가 가능하다.

알고리즘을 아무리 잘 짜도, 센서 특성을 모르면 "왜 이 알고리즘이 이 상황에서 실패하는가?"를 진단할 수 없다. 예를 들어, SLAM을 돌렸는데 특정 구간에서 트래킹이 날아가는 원인이 Rolling Shutter 때문인지, LiDAR 반사 특성 때문인지, IMU 바이어스 때문인지를 센서 지식 없이는 파악하기 어렵다. 센서는 로보틱스 시스템의 입구이며, 입구에서 들어오는 데이터의 특성을 모르면 나머지 모든 것이 흔들린다.

## 2.1 카메라 (Camera)

카메라는 가장 정보량이 풍부한 센서이다. 사람이 시각으로 세상의 대부분을 이해하듯, 로봇도 카메라에서 가장 많은 정보를 얻는다. 하지만 카메라 종류마다 특성이 많이 다르기 때문에, 각각의 장단점을 이해하고 목적에 맞는 카메라를 선택하는 것이 중요하다.

### 2.1.1 Monocular Camera (단안 카메라)

가장 기본적인 시각 센서로, 하나의 렌즈로 2D 이미지를 촬영한다.

단안 카메라는 가장 저렴하고 가벼운 센서이면서도 Visual SLAM, 객체 인식, 시맨틱 이해 등 대부분의 비전 태스크의 출발점이다. 다만 깊이를 직접 측정할 수 없다는 구조적 한계 때문에, 이를 극복하기 위한 다양한 알고리즘(Monocular Depth Estimation, SfM 등)이 발전해왔다. 이 한계를 이해해야 왜 스테레오 카메라나 깊이 카메라가 필요한지도 알 수 있다.

**장점**:
- 저렴하고 가벼움
- 풍부한 색상 및 텍스처 정보
- 높은 해상도

**단점**:
- 단일 이미지에서 깊이(depth) 직접 측정 불가
- Scale ambiguity: 물체의 실제 크기를 알 수 없음

**주요 사양**:
- 해상도: 720p, 1080p, 4K 등
- Frame rate: 30fps, 60fps, 120fps 등
- Field of View (FoV): 좁은 화각 vs 광각 (fisheye)
- Global shutter vs Rolling shutter

```
일반적인 카메라 센서:
- 웹캠: Logitech C920, C930e
- 산업용: FLIR (Point Grey), Basler, Allied Vision
- 임베디드: Raspberry Pi Camera, OAK-D
```

> **추천 자료**
> - [First Principles of Computer Vision — Camera and Imaging](https://www.youtube.com/playlist?list=PL2zRqk16wsdoCCLpou-dGo7QQNks1Ppzo) — Columbia 대학교 Shree Nayar 교수의 카메라 원리 강의. 핀홀 모델부터 렌즈 왜곡까지 설명.
> - [OpenCV Camera Calibration Tutorial](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html) — 카메라 캘리브레이션을 직접 해보는 실습 가이드

### 2.1.2 Stereo Camera (스테레오 카메라)

두 개의 카메라를 일정 간격(baseline)으로 배치하여 깊이를 측정한다. 인간의 양안 시각과 같은 원리다.

실외 환경에서 깊이를 얻는 방법 중 패시브(능동적 빛 방출 없이) 방식은 스테레오 카메라가 거의 유일하다. Structured Light나 ToF는 햇빛 간섭으로 실외에서 제대로 작동하지 않기 때문이다. 에피폴라 기하학(Epipolar Geometry)과 직결되므로 수학적 기초와도 연결된다.

**깊이 계산 원리**:

```
Depth (Z) = (focal_length × baseline) / disparity
```

- **Disparity**: 좌우 이미지에서 동일 점의 x좌표 차이
- **Baseline**: 두 카메라 사이의 거리

**장점**:
- 패시브 센서 (조명 불필요)
- 실외 환경에서도 사용 가능
- RGB 정보와 깊이를 동시에 획득

**단점**:
- 텍스처가 없는 표면에서 매칭 실패 (흰 벽, 유리)
- 계산 비용이 높음
- Baseline에 따라 측정 범위 제한

**대표 제품**:
- Intel RealSense D435/D455: Active IR 패턴 투사로 매칭 보조
- ZED 2: 넓은 baseline, 장거리 측정
- OAK-D: 엣지 AI 내장

> **추천 자료**
> - [Cyrill Stachniss — Stereo Vision](https://www.youtube.com/watch?v=SyB7Wg1e62A) — 스테레오 비전의 수학적 원리를 명확하게 설명
> - [Stanford CS231A — Epipolar Geometry and Stereo](https://web.stanford.edu/class/cs231a/) — Stanford의 Computer Vision 강의. 에피폴라 기하학을 잘 다룬다.

> **실습**: [Stereo Disparity 시각화](https://alexjunholee.github.io/robotics-practice/app.html#stereo_disparity)
> 스테레오 이미지 쌍에서 disparity를 계산하고, baseline과 focal length가 깊이 추정에 미치는 영향을 확인할 수 있다.

### 2.1.3 RGB-D Camera (깊이 카메라)

RGB 이미지와 Depth 이미지를 직접 제공하는 센서이다.

연구실에서 처음 만지게 되는 센서는 대개 RGB-D 카메라다. 데스크탑 환경에서 SLAM이나 3D 복원을 실험할 때 가장 편리하기 때문이다. 다만 ToF와 Structured Light 방식의 차이를 모르면, 왜 실외에서 깊이 값이 날아가는지, 왜 여러 대를 동시에 쓰면 간섭이 생기는지 이해하기 어렵다.

**ToF (Time of Flight) 방식**:
- 적외선을 발사하고 돌아오는 시간을 측정
- 장점: 텍스처 무관, 실시간 처리
- 단점: 햇빛 간섭, 반사 표면 문제
- 예시: Microsoft Azure Kinect, PMD Pico Flexx

**Structured Light 방식**:
- 알려진 패턴을 투사하고 변형을 분석
- 장점: 높은 정확도, 저비용
- 단점: 실외 사용 어려움, 다중 센서 간섭
- 예시: Intel RealSense D400 시리즈, Orbbec Astra

**비교**:
| 특성 | ToF | Structured Light |
|------|-----|------------------|
| 실외 사용 | 제한적 | 어려움 |
| 정확도 | 중간 | 높음 |
| 범위 | 0.2-5m | 0.2-10m |
| 다중 센서 | 가능 | 간섭 발생 |

> **추천 자료**
> - [Intel RealSense — Depth Cameras D415 & D435](https://www.youtube.com/watch?v=A4Kjvosvx5I) — Intel에서 직접 설명하는 깊이 카메라 원리
> - [Open3D RGB-D Reconstruction Tutorial](http://www.open3d.org/docs/release/tutorial/pipelines/rgbd_integration.html) — RGB-D 데이터로 3D 복원을 실습하는 튜토리얼

**RealSense 드라이버 설치 (Ubuntu 22.04)**

```bash
# Intel RealSense SDK 설치
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
    sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update
sudo apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev

# 테스트
realsense-viewer
```

ROS2에서 사용하려면 추가로:
```bash
sudo apt install ros-humble-realsense2-camera
ros2 launch realsense2_camera rs_launch.py
```

(참고: [정진용 블로그](https://jinyongjeong.github.io/2020/06/20/Realsense-Ubuntu-driver-%EC%84%A4%EC%B9%98/))

### 2.1.4 Event Camera (이벤트 카메라)

기존 카메라와 다른 패러다임의 센서이다. 프레임 단위 촬영 대신, 각 픽셀이 **밝기 변화**가 생길 때만 비동기적으로 이벤트를 출력한다.

Event Camera 관련 논문은 CVPR 기준 2019년 5편 수준에서 2024년 30편 이상으로 늘었다. 고속 환경(드론 고속 비행, 자동차 급회전)에서 모션 블러 없이 동작하기 때문이다. 아직 주류는 아니지만, 고속(>100km/h) 환경이나 HDR 조건을 다룰 예정이라면 Gallego et al. survey (TPAMI 2020)와 rpg_dvs_ros 패키지를 살펴보라.

**이벤트 출력 형식**:

```
(x, y, timestamp, polarity)
- x, y: 픽셀 좌표
- timestamp: 마이크로초 단위 시간
- polarity: 밝아짐(+1) 또는 어두워짐(-1)
```

**장점**:
- 매우 높은 시간 해상도 (마이크로초 단위)
- 높은 다이나믹 레인지 (140dB vs 일반 카메라 60dB)
- 낮은 전력 소모, 낮은 지연
- 모션 블러 없음

**단점**:
- 정지 장면에서는 출력 없음
- 전통적인 CV 알고리즘 적용 어려움
- 비교적 높은 가격

**대표 제품**:
- Prophesee: 고해상도 이벤트 센서
- iniVation: DAVIS (이벤트 + 프레임 동시 출력)
- Samsung: 모바일용 이벤트 센서 개발 중

> **추천 자료**
> - [Davide Scaramuzza — Event Cameras: A Paradigm Shift for Computer Vision](https://www.youtube.com/watch?v=LauQ6LWTkxM) — Event Camera 분야의 선구자인 Scaramuzza 교수의 개요 강연
> - [Gallego et al. — Event-based Vision: A Survey (TPAMI 2020)](https://arxiv.org/abs/1904.08405) — Event Camera 기술의 종합 서베이 논문. 이 분야를 이해하는 데 좋은 출발점이다.
> - [rpg_dvs_ros — Event Camera ROS 드라이버](https://github.com/uzh-rpg/rpg_dvs_ros) — Event Camera를 ROS에서 다루는 오픈소스 패키지

## 2.2 LiDAR

**LiDAR (Light Detection and Ranging)**는 레이저를 이용하여 거리를 측정하는 센서이다. 3D 포인트 클라우드를 직접 생성한다.

카메라는 풍부하지만 깊이가 없는 데이터를 준다. LiDAR는 정확한 3D 좌표를 직접 준다. 카메라만으로 깊이를 추정하면 오차가 크고 날씨 영향을 받지만, LiDAR는 수 센티미터 정확도로 100m 이상 떨어진 물체까지 측정할 수 있다. 자율주행에서 LiDAR가 핵심 센서가 된 건 그 때문이다.

최근 주목할 트렌드: Solid-State LiDAR가 기존의 Spinning(기계식) LiDAR를 빠르게 대체하고 있다. 움직이는 부품이 없어 내구성이 높고 대량 생산에 유리하며, 자동차 양산에 적합하기 때문이다. Livox의 비반복 스캔 패턴 같은 새로운 접근도 나오고 있어, 포인트 클라우드 처리 알고리즘에도 변화가 필요한 시점이다.

### 2.2.1 2D LiDAR vs 3D LiDAR

**2D LiDAR**:
- 단일 평면 스캔
- 용도: 실내 로봇 내비게이션, 장애물 회피
- 예시: SICK TiM, Hokuyo URG, RPLIDAR

**3D LiDAR**:
- 다중 레이어 또는 회전 스캔으로 3D 포인트 클라우드 생성
- 용도: 자율주행, 대규모 매핑
- 예시: Velodyne VLP-16/32/64, Ouster OS1, Hesai

> **추천 자료**
> - [Cyrill Stachniss — LiDAR-based SLAM](https://www.youtube.com/watch?v=vrdlk2p9AZI) — LiDAR 데이터를 이용한 SLAM의 원리를 설명
> - [PCL (Point Cloud Library) 공식 튜토리얼](https://pcl.readthedocs.io/projects/tutorials/en/latest/) — 포인트 클라우드 처리의 사실상 표준 라이브러리

### 2.2.2 Spinning vs Solid-State

**Spinning (기계식)**:
- 레이저와 수광부가 회전
- 360° FoV 제공
- 단점: 움직이는 부품으로 인한 내구성 이슈
- 예시: Velodyne, Ouster

**Solid-State**:
- 움직이는 부품 없음
- 제한된 FoV (보통 120° 이하)
- 장점: 높은 내구성, 저비용 가능성
- 예시: Livox (비반복 스캔 패턴), Innoviz

알고리즘 설계에 직접 영향을 미치는 차이다. Spinning LiDAR는 360° 균일한 포인트 클라우드를 생성하므로, 기존 SLAM 알고리즘(LOAM, LeGO-LOAM 등)이 이 특성을 전제로 설계되었다. Solid-State LiDAR로 넘어가면 스캔 패턴이 크게 달라져 알고리즘 수정이 필요하다. Livox의 비반복 스캔을 겨냥해 FAST-LIO2 같은 알고리즘이 등장한 것도 그 때문이다.

### 2.2.3 주요 사양

| 사양 | 설명 |
| --- | --- |
| Channels | 수직 레이어 수 (16, 32, 64, 128) |
| Range | 최대 측정 거리 (50m ~ 300m) |
| Points/sec | 초당 포인트 수 (300K ~ 2M) |
| Accuracy | 측정 정확도 (±2cm ~ ±5cm) |
| FoV | 수평/수직 화각 |

> **추천 자료**
> - [Livox 기술 문서](https://www.livoxtech.com/downloads) — Solid-State LiDAR의 비반복 스캔 패턴과 그 장점을 설명하는 기술 자료
> - [Xu et al. — FAST-LIO2 (RA-L 2022)](https://arxiv.org/abs/2107.06829) — Solid-State LiDAR에 최적화된 LiDAR-Inertial Odometry 논문

## 2.3 IMU (Inertial Measurement Unit)

IMU는 관성을 이용하여 움직임을 측정하는 센서이다.

SLAM을 돌렸는데 드리프트가 심할 때, IMU 특성을 모르면 원인조차 파악하기 어렵다. "IMU 바이어스가 제대로 보정되고 있나?", "이 등급의 IMU로 이 정도 정확도를 기대할 수 있나?" 같은 질문에 답하려면 오차 모델을 이해해야 한다. Visual-Inertial Odometry(VIO)나 LiDAR-Inertial Odometry(LIO)에서 IMU는 카메라/LiDAR 프레임 사이를 메워주는 역할을 한다. 그 역할을 제대로 하려면 IMU 데이터의 한계를 알아야 한다.

### 2.3.1 구성 요소

**Accelerometer (가속도계)**:
- 3축 선형 가속도 측정 (m/s²)
- 중력 가속도 포함

**Gyroscope (자이로스코프)**:
- 3축 각속도 측정 (rad/s 또는 deg/s)
- 회전 속도 감지

**Magnetometer (지자기 센서)** (일부 IMU):
- 3축 자기장 측정
- 절대 방위(heading) 추정 가능
- 자기장 왜곡에 취약

### 2.3.2 주요 오차 특성

IMU 오차를 모델링하지 못하면 센서 퓨전 시스템 전체가 흔들린다.

**Bias (바이어스)**:
- 정지 상태에서도 0이 아닌 출력
- 온도에 따라 변화 (bias instability)

**Noise**:
- 고주파 랜덤 노이즈
- Allan Variance로 특성화

**Integration Drift**:
- 가속도 이중 적분 → 위치 오차 누적
- 각속도 적분 → 자세 오차 누적
- 짧은 시간 동안만 신뢰 가능 (보통 수 초)

실제로 겪어보면 바로 느끼는데, 가속도를 두 번 적분해서 위치를 구하면 노이즈와 바이어스 오차가 시간의 제곱에 비례하여 누적된다. Consumer 등급 IMU(스마트폰 내장)로는 10초만 지나도 위치 오차가 수 미터에 달할 수 있다. 그래서 IMU를 단독으로 쓰는 경우는 거의 없고, 항상 카메라나 LiDAR와 퓨전하여 드리프트를 보정한다.

**IMU 등급**:
| 등급 | 용도 | 가격 | 예시 |
|------|------|------|------|
| Consumer | 스마트폰, 게임 | $1-10 | MPU6050, BMI160 |
| Industrial | 로봇, 드론 | $100-1K | VectorNav VN-100, Xsens MTi |
| Tactical | 자율주행, 항공 | $1K-10K | KVH 1750 |
| Navigation | 선박, 항공기 | $10K+ | Honeywell HG1700 |

> **추천 자료**
> - [Probabilistic Robotics, Ch.5 — Robot Motion (Thrun, Burgard, Fox)](https://www.probabilistic-robotics.org/) — 센서 노이즈 모델링과 모션 모델의 핵심 참고서. IMU 오차 모델링의 이론적 기초를 다룬다.
> - [Titterton & Weston — Strapdown Inertial Navigation Technology](https://ieeexplore.ieee.org/book/5765860) — IMU 원리와 관성 항법의 교과서
> - [Cyrill Stachniss — IMU and Inertial Navigation](https://www.youtube.com/watch?v=uHbRKvD8TWg) — IMU의 작동 원리와 오차 특성을 시각적으로 설명
> - [Allan Variance — IMU 노이즈 분석 가이드 (Vectornav)](https://www.vectornav.com/resources/inertial-navigation-primer/specifications--background/specifications--allan-variance) — Allan Variance를 이용한 IMU 노이즈 파라미터 추출 방법
> - [정진용 블로그 — IMU Filter (AHRS)](https://jinyongjeong.github.io/2020/01/10/IMU_filter/) — IMU 센서의 AHRS 필터 개요. Madgwick 필터와 ROS 패키지 소개

## 2.4 GPS/GNSS

**GNSS (Global Navigation Satellite System)**는 위성 신호를 이용한 위치 측정 시스템이다. GPS는 미국 시스템이며, GNSS는 GPS, GLONASS(러시아), Galileo(유럽), BeiDou(중국) 등을 포함하는 총칭이다.

실외 자율주행이나 드론에서 GNSS는 "전역 좌표"를 제공하는 유일한 센서이다. SLAM은 상대적 위치(어디서 출발해서 얼마나 움직였나)를 추정하지만, GNSS는 지구상 절대 위치(위도, 경도, 고도)를 알려준다. 이 두 가지 정보를 결합하는 것이 실외 로봇의 핵심 과제이다. RTK-GPS의 센티미터급 정확도가 자율주행 고정밀 측위의 Ground Truth로도 사용되므로, 원리를 이해해두어야 한다.

**정확도**:
- 일반 GPS: 2-5m
- DGPS (Differential): 0.5-2m
- RTK-GPS (Real-Time Kinematic): 1-2cm

**RTK-GPS 원리**:
- 고정된 Base Station이 보정 데이터 제공
- Rover가 보정 데이터를 수신하여 정확도 향상
- 실시간 통신 필요 (Radio 또는 인터넷)

**한계**:
- 실내, 터널, 도심 캐년에서 사용 불가
- 멀티패스 오차 (건물 반사)
- 고도 정확도는 수평보다 낮음

> **추천 자료**
> - [Cyrill Stachniss — Robot Localization Overview](https://www.youtube.com/watch?v=8VJ-A9OlhAE) — 로봇 위치 추정의 원리와 방법론 개요
> - [u-blox GNSS 가이드](https://www.u-blox.com/en/technologies/gnss) — GNSS 기초부터 RTK까지 실용적 가이드

## 2.5 기타 센서

**Radar**

자율주행과 로보틱스에서 레이더의 중요성이 높아지고 있다. LiDAR와 카메라가 안 되는 환경 — 안개, 비, 먼지, 강한 역광 — 에서도 레이더는 안정적으로 동작한다. 가격도 LiDAR보다 저렴하다.

**FMCW (Frequency Modulated Continuous Wave) Radar**:
- 주파수를 시간에 따라 변조하여 송신하고, 반사파와의 주파수 차이로 거리와 속도를 동시에 측정한다.
- 출력: Range-Doppler map (거리 × 속도 2D 맵), Range-Azimuth map
- 자동차용 77GHz radar가 가장 흔하다.

**로보틱스에서의 활용**:
- 자율주행: 전방 충돌 감지, 적응형 크루즈 컨트롤 (ACC)
- Radar odometry: 레이더만으로 자기 위치 변화 추정
- Radar SLAM: 레이더 기반 지도 작성 + 위치 추정

**카메라/LiDAR와의 비교**:

| 특성 | 카메라 | LiDAR | Radar |
|------|--------|-------|-------|
| 해상도 | 매우 높음 | 높음 | 낮음 |
| 거리 측정 | 불가 (단안) | 정확 | 가능 |
| 속도 측정 | 불가 | 불가 (직접) | 가능 (Doppler) |
| 악천후 | 취약 | 취약 (비, 안개) | 강건 |
| 가격 | 저렴 | 비쌈 | 중간 |
| 야간 | 불가 | 가능 | 가능 |

**대표 제품**: Texas Instruments AWR1843, Continental ARS548, Navtech CTS350-X (spinning radar)

> **추천 자료**
> - [김기섭 블로그 — ICRA 2021 Radar in Robotics Workshop 요약](https://gisbi-kim.github.io/blog/2021/05/31/icra21-radar-ws.html) — 레이더 로보틱스의 전반적 동향 정리
> - [김기섭 블로그 — Radar Odometry Results on MulRan dataset](https://gisbi-kim.github.io/blog/2021/05/30/yeti-radar-odom-mulran1.html) — 레이더 오도메트리 실험 결과. 도시 환경에서 LiDAR급 성능
> - [Kim et al., "MulRan: Multimodal Range Dataset for Urban Place Recognition" (ICRA 2020)](https://sites.google.com/view/mulran-pr/home) — LiDAR + radar + GPS 멀티모달 데이터셋

**Ultrasonic (초음파)**:
- 가까운 거리 장애물 감지 (0.2-5m)
- 저비용
- 주차 보조, 근접 센서

**Wheel Encoder (휠 엔코더)**:
- 바퀴 회전량 측정
- Dead reckoning 기반 위치 추정
- 슬립에 취약

이 센서들을 "기타"로 분류했다고 중요하지 않은 게 아니다. Radar는 자율주행에서 LiDAR가 실패하는 악천후 상황의 안전망 역할을 하며, Wheel Encoder는 지상 로봇의 가장 기본적인 오도메트리 소스이다. 센서 퓨전에서는 이런 "보조" 센서들이 시스템 전체의 로버스트니스를 결정한다.

> **추천 자료**
> - [Probabilistic Robotics, Ch.6 — Robot Perception (Thrun, Burgard, Fox)](https://www.probabilistic-robotics.org/) — 각종 센서의 확률 모델을 꼼꼼하게 다룬다. 센서 모델링의 교과서.

## 2.6 센서 퓨전 (Sensor Fusion)

단일 센서는 각각 한계가 있다. 여러 센서를 결합해 서로 보완한다.

현실에서 단일 센서만으로 완벽한 인식을 구현하는 건 불가능하다. 자율주행차는 카메라, LiDAR, Radar, IMU, GNSS를 전부 동시에 사용한다. 이 센서들의 데이터를 언제, 어디서, 어떻게 결합하느냐가 시스템 성능을 좌우한다.

**왜 필요한가?**

| 센서 | 장점 | 단점 |
| --- | --- | --- |
| Camera | 풍부한 정보, 저렴 | 조명 의존, 깊이 없음 |
| LiDAR | 정확한 3D, 조명 무관 | 비쌈, sparse |
| IMU | 고주파, 조명 무관 | 드리프트 |
| GPS | 전역 위치 | 실외 전용, 저주파 |

**퓨전 방식**:
1. **Early Fusion**: Raw 데이터 레벨에서 결합
2. **Late Fusion**: 각 센서의 결과를 결합
3. **Mid-Level Fusion**: Feature 레벨에서 결합

방식마다 트레이드오프가 있다. Early Fusion은 정보 손실이 적지만 계산 비용이 높다. Late Fusion은 각 센서를 독립적으로 처리할 수 있어 모듈화에 유리하지만 정보가 일부 날아간다. Mid-Level Fusion은 그 중간이며, 딥러닝 기반 퓨전에서 많이 쓰인다.

**대표적인 조합**:
- Camera + IMU → VIO (Visual-Inertial Odometry)
- LiDAR + IMU → LIO (LiDAR-Inertial Odometry)
- Camera + LiDAR + IMU → 멀티모달 SLAM

퓨전의 수학적 토대는 §2.7 측정 모델의 확률적 정형화에서 다룬다.

> **추천 자료**
> - [State Estimation for Robotics (Tim Barfoot) — 무료 PDF](http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) — 센서 퓨전의 수학적 기초를 다루는 대표적인 교재. Kalman Filter, Factor Graph 기반 추정을 모두 커버한다.
> - [Cyrill Stachniss — Kalman Filter & EKF](https://www.youtube.com/watch?v=E-6paM_Iwfc) — 센서 퓨전의 핵심인 칼만 필터와 EKF를 설명
> - [Qin et al. — VINS-Mono (TRO 2018)](https://arxiv.org/abs/1708.03852) — Visual-Inertial 퓨전의 대표 논문. 실제 VIO 시스템이 어떻게 구현되는지 보여준다.

---

## 2.7 심화: 측정 모델 — 확률적 정형화 (Probabilistic Measurement Models)

베이즈 필터·SLAM·MCL은 센서 데이터를 알고리즘에 공급할 때 "측정값이 얼마나 믿을 만한가"를 수치로 표현해야 한다. 그 표현이 측정 모델 $p(z_t \mid x_t, m)$이다.

### 2.7.1 센서가 만드는 분포

레이저 거리 센서를 같은 자세로 같은 벽을 향해 100번 쏘면, 100개의 측정값이 모두 다르다. 반사면 거울각, 지나가는 사람, 다중 반사가 원인이고, 측정값의 분산 구조도 원인마다 다르다. 이 분산 구조를 하나의 확률 분포로 나타낸 것이 $p(z_t^k \mid x_t, m)$이다. $z_t^k$는 시각 $t$의 $k$번째 빔 측정값, $x_t$는 로봇 포즈, $m$은 환경 지도다.

하나의 스캔에는 수십~수백 개의 빔이 있다. PR §6.2는 각 빔의 오차가 독립적으로 발생한다는 조건부 독립 가정을 도입한다. 이 가정 아래에서 전체 스캔의 likelihood는 각 빔 likelihood의 곱이 된다:

$$p(z_t \mid x_t, m) = \prod_{k=1}^{K} p(z_t^k \mid x_t, m)$$

이 조건부 독립 가정은 현실에서 완전히 성립하지 않는다. 같은 벽을 보는 인접 빔들은 상관되어 있고, 그 상관을 무시하면 likelihood가 특정 포즈에 과도하게 몰린다. 이 문제는 §2.7.8에서 다룬다.

지도 $m$의 형태도 두 가지다. feature-based 지도는 랜드마크 목록으로 구성되며 각 요소를 ID로 참조한다. location-based 지도는 격자 셀의 점유 확률 배열이며 셀을 좌표로 참조한다. 측정 모델 4가족은 이 두 지도 형태 중 하나에 의존한다.

측정 모델 4가족:
- 빔 모델 (beam model): 물리 오차 채널의 혼합. location-based 지도.
- likelihood field: 빔 끝점→nearest obstacle 거리. location-based 지도.
- 상관 기반 (map matching): local map과 global map의 정규화 상관계수.
- 특징 기반 (landmark model): 추출된 특징을 (range, bearing, signature)로 모델링. feature-based 지도.

### 2.7.2 빔 모델 — 4성분 혼합

거리 센서의 한 빔이 잘못된 측정값을 내는 원인은 네 가지 물리 채널로 분류된다. [Thrun et al. 2005](https://www.probabilistic-robotics.org/) (PR §6.3.1)은 각 채널을 독립된 확률 분포로 모델링하고, 네 분포의 가중 혼합으로 최종 likelihood를 구성했다.

4성분 중 가장 빈번한 것은 hit, 즉 실제 장애물을 정확히 감지한 경우다. 예측 거리 $z_t^{k*}$를 평균으로 분산 $\sigma_{\text{hit}}^2$의 절단 가우시안으로 모델링한다. 절단은 $[0, z_{\max}]$ 범위 밖의 확률 질량을 제거한다.

$$p_{\text{hit}}(z_t^k \mid x_t, m) = \eta\, \mathcal{N}(z_t^k;\, z_t^{k*},\, \sigma_{\text{hit}}^2), \quad 0 \le z_t^k \le z_{\max}$$

**short (예상치 못한 가까운 장애물)**: 지도에 없는 장애물(지나가는 사람, 다른 로봇)이 빔을 가로막는다. 측정값이 $z_t^{k*}$보다 항상 짧다. $[0, z_t^{k*}]$ 범위에서 지수 분포를 따른다.

$$p_{\text{short}}(z_t^k \mid x_t, m) = \eta\, \lambda_{\text{short}}\, e^{-\lambda_{\text{short}} z_t^k}, \quad 0 \le z_t^k \le z_t^{k*}$$

**max (최대 사거리 실패)**: 검은 표면, 거울각, 안개 등에서 반사파가 돌아오지 않는다. 센서가 $z_{\max}$를 그대로 출력하는 경우다. $z_{\max}$에서의 점질량(Dirac delta)으로 모델링된다.

$$p_{\text{max}}(z_t^k \mid x_t, m) = \mathbf{1}[z_t^k = z_{\max}]$$

**rand (정체불명 노이즈)**: sonar crosstalk, 다중 반사 등 원인을 알 수 없는 측정값이다. $[0, z_{\max}]$에서 균등 분포로 모델링된다.

$$p_{\text{rand}}(z_t^k \mid x_t, m) = \frac{1}{z_{\max}}$$

최종 likelihood는 4성분의 가중 혼합이다 (PR 식 6.13):

$$p(z_t^k \mid x_t, m) = \begin{pmatrix} z_{\text{hit}} \\ z_{\text{short}} \\ z_{\text{max}} \\ z_{\text{rand}} \end{pmatrix}^T \cdot \begin{pmatrix} p_{\text{hit}}(z_t^k \mid x_t, m) \\ p_{\text{short}}(z_t^k \mid x_t, m) \\ p_{\text{max}}(z_t^k \mid x_t, m) \\ p_{\text{rand}}(z_t^k \mid x_t, m) \end{pmatrix}$$

가중치 합은 1이어야 한다: $z_{\text{hit}} + z_{\text{short}} + z_{\text{max}} + z_{\text{rand}} = 1$.

예측 거리 $z_t^{k*}$는 포즈 $x_t$와 지도 $m$에서 ray casting으로 계산한다. 빔의 방향을 따라 점유된 셀에 처음 닿는 거리가 $z_t^{k*}$다. ray casting은 §2.1 카메라 투영 모델·§2.2 LiDAR 빔 구조와 같은 기하 원리를 점유 격자에 적용한 것이다.

**알고리즘: beam_range_finder_model** (PR Table 6.1 의역)

```
입력: z_t = {z_t^1, ..., z_t^K}, x_t, m
출력: p(z_t | x_t, m)

1. q ← 1
2. for k = 1 to K do:
3.     z_t^{k*} ← ray_cast(x_t, k, m)   // 예측 거리
4.     p ← z_hit  * p_hit(z_t^k | z_t^{k*}, σ_hit)
         + z_short * p_short(z_t^k | z_t^{k*}, λ_short)
         + z_max   * p_max(z_t^k | z_max)
         + z_rand  * p_rand(z_t^k | z_max)
5.     q ← q * p
6. return q
```

### 2.7.3 빔 모델 — 파라미터 EM 학습

센서 종류, 환경 구성, 마운팅 위치가 바뀔 때마다 hit/short/max/rand의 비율과 분산이 달라진다. 파라미터를 손으로 정하면 특정 환경에서 맞추기 위해 다른 환경에서 어긋난다.

내재 파라미터는 6개다: $z_{\text{hit}}, z_{\text{short}}, z_{\text{max}}, z_{\text{rand}}, \sigma_{\text{hit}}, \lambda_{\text{short}}$. PR §6.3.2는 로봇이 알려진 환경을 주행하며 수집한 데이터 $\{(z_t^k, z_t^{k*})\}$로부터 EM 알고리즘으로 이 파라미터를 최대우도 추정한다.

핵심 아이디어는 correspondence variable이다. 각 측정값 $z_t^k$에 대해 어느 성분이 그 값을 생성했는지를 나타내는 잠재 변수 $c_i \in \{\text{hit, short, max, rand}\}$를 도입한다.

**E-step**: 현재 파라미터 추정값으로 각 측정에 대해 4성분의 사후 확률을 계산한다 (PR 식 6.15~6.32):

$$e_{\text{hit}}^i = \frac{z_{\text{hit}} \cdot p_{\text{hit}}(z^i \mid z^{i*})}{p(z^i \mid z^{i*})}, \quad e_{\text{short}}^i = \frac{z_{\text{short}} \cdot p_{\text{short}}(z^i \mid z^{i*})}{p(z^i \mid z^{i*})}, \quad \dots$$

**M-step**: E-step에서 계산된 기댓값으로 파라미터를 업데이트한다. $\sigma_{\text{hit}}$와 $\lambda_{\text{short}}$는 닫힌 해가 존재한다:

$$\sigma_{\text{hit}}^2 = \frac{\sum_i e_{\text{hit}}^i (z^i - z^{i*})^2}{\sum_i e_{\text{hit}}^i}, \qquad \lambda_{\text{short}} = \frac{\sum_i e_{\text{short}}^i}{\sum_i e_{\text{short}}^i \cdot z^i}$$

혼합 가중치 $z_{\text{hit}}, z_{\text{short}}, z_{\text{max}}, z_{\text{rand}}$는 각 성분에 속하는 측정 비율로 갱신된다.

**알고리즘: learn_intrinsic_parameters** (PR Table 6.2 압축)

```
입력: {(z^i, z^{i*})} — (측정값, 예측값) 쌍
출력: z_hit, z_short, z_max, z_rand, σ_hit, λ_short

초기화: 파라미터를 균등 또는 임의값으로 설정
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

ROS AMCL의 기본 파라미터 `laser_sigma_hit`(기본값 0.2m), `laser_lambda_short`(기본값 0.1)가 이 EM 과정의 수렴값이다. 2005년 PR 출판 이후 여러 실제 환경에서 EM을 돌린 결과다.

EM으로 파라미터를 얻었더라도, 실제 시스템에서 이 모델을 빠르게 동작시키려면 추가적인 고려가 필요하다.

### 2.7.4 빔 모델 실무 고려

빔 모델의 주요 계산 병목은 ray casting이다. 입자마다 스캔의 모든 빔에 ray casting을 수행하면 MCL에서 입자 수 × 빔 수의 연산이 필요하다.

우선 빔 수를 줄인다. 전체 스캔에서 균일하게 샘플된 소수의 빔(보통 8~50개)만 사용해도, 인접 빔들은 상관이 높으므로 정보 손실이 제한적이다.

**$p^{\alpha}$ 지수화 보정**: 빔들 사이의 독립 가정이 위반될 때, likelihood를 $p(z_t \mid x_t, m)^{\alpha}$ ($0 < \alpha < 1$)로 지수화하면 각 빔의 기여를 축소하여 over-confidence를 완화한다. $\alpha$는 경험적으로 설정하거나 교차 검증으로 정한다.

**range 사전 계산**: 지도에서 미리 모든 (셀, 방향) 조합에 대해 ray casting 결과를 테이블로 저장해 두면, 실행 시에 테이블 조회($O(1)$)로 ray casting을 대체할 수 있다. 지도가 크면 메모리 사용량이 크지만, 실시간 MCL에서 실용적이다 (Ch.3 §3.11 파티클 필터 참조).

### 2.7.5 Likelihood Field

빔 모델의 두 약점은 실용 시스템에서 골치거리다. 첫째, ray casting 비용이 높다. 둘째, 포즈 $x_t$가 작은 양만큼 바뀌어도 빔이 다른 장애물에 먼저 닿으면 $z_t^{k*}$가 갑자기 크게 변한다. likelihood가 포즈에 대해 불연속이다. 이 불연속성은 gradient 기반 정합이나 hill-climbing 최적화를 방해한다.

likelihood field는 ray casting을 버리고 다른 계산을 쓴다. 빔 끝점을 전역 좌표계로 변환한 뒤, 그 점에서 지도의 가장 가까운 점유 셀까지의 유클리드 거리 $\text{dist}$로 likelihood를 평가한다.

빔 끝점의 전역 좌표 변환 (PR 식 6.33):

$$\begin{pmatrix} x_{z_t^k} \\ y_{z_t^k} \end{pmatrix} = \begin{pmatrix} x \\ y \end{pmatrix} + \begin{pmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{pmatrix} \begin{pmatrix} x_{k,\text{sens}} \\ y_{k,\text{sens}} \end{pmatrix} + z_t^k \begin{pmatrix} \cos(\theta + \theta_{k,\text{sens}}) \\ \sin(\theta + \theta_{k,\text{sens}}) \end{pmatrix}$$

여기서 $(x_{k,\text{sens}}, y_{k,\text{sens}})$는 로봇 기준계에서 $k$번째 빔 센서의 위치, $\theta_{k,\text{sens}}$는 빔의 방향 오프셋이다.

빔 likelihood (PR 식 6.34~6.35):

$$p(z_t^k \mid x_t, m) = z_{\text{hit}} \cdot \mathcal{N}(\text{dist};\, 0,\, \sigma_{\text{hit}}^2) + z_{\text{rand}} \cdot \frac{1}{z_{\max}}$$

여기서 $\text{dist}$는 빔 끝점에서 가장 가까운 점유 셀까지의 유클리드 거리이고, 가우시안은 거리 오차를 0-평균으로 모델링한다. max-range 빔($z_t^k = z_{\max}$)은 이 모델에서 무시한다. 끝점 투영이 의미 없기 때문이다.

**알고리즘: likelihood_field_range_finder_model** (PR Table 6.3 의역)

```
입력: z_t = {z_t^1, ..., z_t^K}, x_t = (x, y, θ)^T, m
출력: p(z_t | x_t, m)

1. q ← 1
2. for each k do:
3.     if z_t^k == z_max: continue   // max-range 무시
4.     // 빔 끝점을 전역 좌표로 변환
5.     x_ep ← x + x_{k,sens}·cos(θ) - y_{k,sens}·sin(θ) + z_t^k · cos(θ + θ_{k,sens})
6.     y_ep ← y + y_{k,sens}·cos(θ) + x_{k,sens}·sin(θ) + z_t^k · sin(θ + θ_{k,sens})
7.     // nearest obstacle까지 거리 (사전계산된 거리 변환 테이블에서 조회)
8.     dist ← nearest_obstacle_distance(x_ep, y_ep, m)
9.     q ← q * (z_hit · N(dist; 0, σ_hit²) + z_rand / z_max)
10. return q
```

지도가 고정되면 거리 변환(distance transform)을 한 번만 수행하고 테이블로 저장해 두면 $\text{dist}$ 조회가 $O(1)$이 된다. 이 테이블은 본질적으로 SDF(Signed Distance Field)의 양수 부분이다. 포즈 $x_t$에 대한 likelihood의 gradient를 계산할 수 있어, gradient 기반 scan matching에 적합하다.

한계도 있다. short 성분이 없어 동적 장애물을 명시적으로 모델링하지 않으며, occlusion도 없어서 지도에 없는 자유 공간을 통과하는 빔도 끝점 거리만으로 평가한다. 벽 너머까지 "볼 수" 있고 지도 자체의 불확실성도 무시한다.

2D LiDAR 실내 내비게이션에서 AMCL은 빔 모델보다 likelihood field를 기본으로 쓴다. 계산이 빠르고 포즈에 대해 연속적이기 때문이다. 3D LiDAR와 RGB-D에서는 ICP, NDT가 이 역할을 넘겨받았다 (Ch.3 §3.10 칼만 필터 계열과의 연동은 Ch.14 §14.7 참조).

루프 클로저 검출처럼 두 맵이 같은 장소인지를 빠르게 판단해야 할 때는, 확률론적 엄밀성보다 계산 속도가 우선이다.

### 2.7.6 상관 기반 모델 (Map Matching)

상관 기반 모델은 가장 ad hoc한 방법이다. 최근 스캔 집합으로 local map $m_{\text{local}}$을 구성하고, 이것을 global map $m$과 정규화 상관계수 $\rho$로 비교한다. PR §6.5는 이 비교 결과를 그대로 likelihood로 사용한다:

$$p(m_{\text{local}} \mid x_t, m) = \max\{\rho(m_{\text{local}}, m \mid x_t),\ 0\}$$

$\rho$는 두 맵을 $x_t$로 정렬했을 때 대응하는 셀들 사이의 피어슨 상관계수다. 계산이 빠르고 구현이 단순하다. 다만 이 likelihood는 확률론적으로 정당화되지 않는다. $\rho$는 정규화되어 있고, 0보다 큰 값은 그냥 잘라버린다. SLAM 백엔드의 loop closure 검출처럼 정확한 likelihood보다 빠른 유사도 점수가 필요한 경우에 쓰인다.

마지막 모델은 raw 거리 측정값 대신 센서 데이터에서 추출한 구조적 특징을 다룬다.

### 2.7.7 특징 기반 측정 — 랜드마크 모델

랜드마크 모델은 센서 데이터에서 추출한 특징 $f(z_t)$를 다룬다. 저차원 특징으로 추론하므로 연산량이 작고, feature-based 지도와 자연스럽게 연결된다.

특징 추출의 형태는 센서마다 다르다. 거리 스캔에서는 선분, 코너, 국소 극솟값. 카메라에서는 edge, corner, SIFT/ORB 같은 국소 패턴 (§2.1.1 단안 카메라의 texture 특성, §2.6 VIO/Visual SLAM 참조). 추출된 각 특징은 $(r, \phi, s)$ 삼중항으로 표현된다: $r$은 range, $\phi$는 bearing, $s$는 signature(ID, 색상, 디스크립터 등).

지도의 $j$번째 랜드마크가 좌표 $(m_{j,x}, m_{j,y})$에 있고 signature $s_j$를 갖는다. 포즈 $x_t = (x, y, \theta)^T$에서의 예측 측정과 실제 측정 사이의 관계 (PR 식 6.41):

$$\begin{pmatrix} r_t^i \\ \phi_t^i \\ s_t^i \end{pmatrix} = \begin{pmatrix} \sqrt{(m_{j,x} - x)^2 + (m_{j,y} - y)^2} \\ \operatorname{atan2}(m_{j,y} - y,\, m_{j,x} - x) - \theta \\ s_j \end{pmatrix} + \begin{pmatrix} \varepsilon_{\sigma_r^2} \\ \varepsilon_{\sigma_\phi^2} \\ \varepsilon_{\sigma_s^2} \end{pmatrix}$$

$\varepsilon_{\sigma^2}$는 0-평균 분산 $\sigma^2$ 가우시안이다. 세 채널이 독립 가우시안 잡음을 갖는다는 가정이다. bearing 채널 $\varepsilon_{\sigma_\phi^2}$에 가우시안을 직접 더하는 이 모델은 $\pm\pi$ 근방에서 wrap-around 오류를 낼 수 있다. 실전 구현에서는 각도 차이를 $[-\pi, \pi]$로 정규화하거나 von Mises 분포로 대체한다.

대응 $c_t^i = j$($i$번째 특징이 $j$번째 랜드마크에 대응)가 알려진 경우의 likelihood는 세 채널 가우시안의 곱이다.

**알고리즘: landmark_model_known_correspondence** (PR Table 6.4 의역)

```
입력: f_t^i = (r_t^i, φ_t^i, s_t^i)^T, 대응 c_t^i = j, x_t = (x, y, θ)^T, m
출력: p(f_t^i | c_t^i = j, x_t, m)

1. j ← c_t^i
2. r̂ ← sqrt((m_{j,x} - x)² + (m_{j,y} - y)²)
3. φ̂ ← atan2(m_{j,y} - y, m_{j,x} - x) - θ
4. q ← prob(r_t^i - r̂, σ_r²)
       * prob(φ_t^i - φ̂, σ_φ²)
       * prob(s_t^i - s_j, σ_s²)
5. return q
   // prob(a, σ²) = N(a; 0, σ²) — 0-평균 가우시안 밀도
```

전체 스캔의 특징들 사이 조건부 독립을 가정하면 전체 likelihood는 $\prod_i$다.

**역방향 — 포즈 샘플링** (PR Table 6.5 압축): 측정으로부터 가능한 포즈를 샘플링하는 방향도 존재한다. 하나의 $(r, \phi)$ 측정은 포즈 공간에서 두 제약만 주므로, 가능한 포즈들은 랜드마크를 중심으로 한 원(2D) 또는 나선(3D) 위에 분포한다. 자유 파라미터 $\hat{\gamma} \sim U(0, 2\pi)$로 원 상의 위치를 샘플링한다. 랜드마크 하나를 한 번만 보면 위치를 특정할 수 없다는 사실의 기하학적 설명이기도 하다.

<!-- DEMO: landmark_donut.html -->

visual SLAM에서 사용하는 reprojection error $\| \pi(K[R|t]\, X_w) - u \|^2_\Sigma$는 이 랜드마크 모델의 직접적인 계승이다. range/bearing 대신 픽셀 좌표 $(u, v)$, signature 대신 ORB/SIFT 디스크립터가 들어간다. EKF가 이 likelihood를 업데이트 단계에서 사용하는 방식은 Ch.3 §3.10 가우시안 필터·Ch.14 §14.7를 참조한다. AprilTag, ArUco 같은 fiducial marker는 ID로 대응이 보장되므로 known correspondence 가정이 그대로 성립한다. 이 모델이 문자 그대로 현역이다.

### 2.7.8 실무 정리: 모델 선택

4가족 모델을 비교하면:

| 모델 | 정확도 | 계산 속도 | 미분 가능성 | 주요 용도 |
|------|--------|-----------|------------|-----------|
| 빔 모델 | 높음 | 느림 (ray casting) | 낮음 (불연속) | MCL high-fidelity, 진단 |
| Likelihood field | 중간 | 빠름 (DT lookup) | 높음 | AMCL 기본, gradient 정합 |
| 상관 기반 | 낮음 | 매우 빠름 | 낮음 | loop closure 검출 |
| 랜드마크 | 높음 (특징 의존) | 빠름 (저차원) | 높음 | visual SLAM, fiducial |

over-confidence도 실용적 고려사항이다. 빔들 사이의 조건부 독립 가정이 위반되면 likelihood $p(z_t \mid x_t, m)$의 절댓값이 실제보다 훨씬 작아지거나 특정 포즈에 과도하게 몰린다. §2.7.4의 지수화 기법, 즉 likelihood를 $p(z_t \mid x_t, m)^{\alpha}$ ($\alpha < 1$)로 치환하는 것이 표준 완화책이다. $\alpha < 1$이면 각 빔의 기여가 줄어들어 분포가 더 평탄해진다.

그렇다면 이 모델들이 실제 시스템에서 얼마나 살아남았는가.

### 2.7.9 무엇이 살아남았나

PR §6에서 정형화된 4가족 모델의 직계 후손들이 2026년 현재 실전 시스템에서 작동한다.

빔 모델의 4-혼합과 EM으로 학습한 파라미터는 ROS Navigation Stack의 AMCL 노드에 그대로 남아 있다. `amcl` 패키지의 `laser_sigma_hit = 0.2`, `laser_lambda_short = 0.1` 기본값이 2000년대 초 실제 환경에서 EM을 돌린 결과다. `beam_skip_*` 파라미터는 §2.7.4의 빔 부분집합 아이디어다.

likelihood field는 더 넓게 살아있다. Cartographer, SLAM Toolbox, hdl_localization 모두 2D LiDAR localization 코어에서 거리 변환 기반 likelihood 평가를 쓴다. ESDF 기반 경로 계획이나 NeRF-SLAM처럼 SDF/ESDF를 핵심 자료구조로 쓰는 최근 시스템도 likelihood field의 사전계산 아이디어를 그대로 계승한다.

랜드마크 모델 (식 6.41)은 visual SLAM 전체의 골격이다. ORB-SLAM3의 reprojection error 최소화, SfM의 bundle adjustment, DROID-SLAM의 DBA layer가 모두 이 식의 카메라 투영 일반화다. fiducial 기반 AR/로봇 시스템에서는 식 그대로가 현역이다.

측정 모델의 형태는 센서 물리가 결정한다. hit·short·max·rand는 물리 채널이다. 모든 모델은 지도 형태(location-based vs feature-based)와 짝을 이루고, 계산 속도와 확률론적 정당성은 트레이드오프다. 빠른 시스템은 likelihood field나 상관 기반을 쓰고, 진단이 필요하면 빔 모델로 돌아간다.

Ch.14 §14.7에서 MCL의 `beam_range_finder_model` 호출과 occupancy mapping의 `inverse_sensor_model`을 보면, 이 모델들이 어떻게 연결되는지 확인할 수 있다.

> **⚠ AI 에이전트 주의**: 센서가 "데이터가 안 온다"는 문제의 원인은 대부분 소프트웨어가 아니라 물리적 연결(케이블, IP 설정, 전원, USB 대역폭)이다. AI는 드라이버 재설치부터 권하지만, `dmesg`, `lsusb`, `ping` 같은 시스템 명령으로 물리 연결부터 확인하라.

> **기술 흐름: 센서 기술**
> - **~2010**: 2D LiDAR(SICK, Hokuyo)와 단안 카메라 중심. 센서가 비싸고 크며, 처리 능력도 제한적. Stereo 카메라는 계산 비용 때문에 실시간 처리가 어려웠다.
> - **2012~2017**: 3D LiDAR(Velodyne VLP-16) 보급, RGB-D 카메라(Kinect) 대중화. LiDAR 가격이 수만 달러에서 수천 달러로 하락. Visual-Inertial 시스템(VIO)도 실제 시스템에서 쓰이기 시작했다.
> - **2018~2022**: Solid-State LiDAR(Livox) 등장, 가격이 수백 달러 수준까지 하락. Event Camera 연구 활발해짐. 멀티모달 센서 퓨전(Camera + LiDAR + IMU)이 표준으로 자리잡음.
> - **2023~**: Solid-State LiDAR가 Spinning 방식을 빠르게 대체 중. 고속/HDR 응용에서 Event Camera 채택이 늘기 시작했다. 4D Radar(도플러 속도 포함)도 보조 센서로 채택이 늘고 있다.
> - **지금 주목할 것**: Solid-State LiDAR가 대중화되면서 기존 Spinning LiDAR 전제의 알고리즘을 재설계해야 하는 상황이다. Event Camera는 아직 주류가 아니지만, 고속 드론·자율주행처럼 기존 카메라의 한계가 명확한 분야에서 빠르게 채택되고 있다. 센서 하드웨어가 바뀌면 알고리즘 연구 방향도 따라 바뀐다.
