# Ch.2 — 센서 (Sensors)


로봇이 환경을 인식하기 위해서는 센서가 있어야 한다. 각 센서의 특성을 이해하면 적절한 센서 선택과 알고리즘 설계가 가능하다.

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

실외 환경에서 깊이를 얻으려면 스테레오 비전을 알아야 한다. 스테레오 카메라는 패시브(능동적 빛 방출 없이) 방식으로 깊이를 얻을 수 있는 거의 유일한 방법이다. 자율주행, 드론 등 실외 환경에서는 Structured Light나 ToF 방식이 햇빛 때문에 제대로 작동하지 않기 때문에, 스테레오 비전의 원리를 아는 것이 매우 중요하다. 에피폴라 기하학(Epipolar Geometry)과 직결되므로 수학적 기초와도 연결된다.

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

연구실에서 가장 먼저 접하게 될 센서가 바로 RGB-D 카메라일 가능성이 높다. 데스크탑 환경에서 SLAM이나 3D 복원을 실험할 때 가장 편리하기 때문이다. 하지만 ToF와 Structured Light 방식의 차이를 모르면, 왜 실외에서 깊이 값이 날아가는지, 왜 여러 대를 동시에 쓰면 간섭이 생기는지 이해할 수 없다.

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

기존 카메라와 다른 패러다임의 센서이다. 프레임 단위로 촬영하는 것이 아니라, 각 픽셀이 **밝기 변화**가 발생할 때만 비동기적으로 이벤트를 출력한다.

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

카메라가 "풍부하지만 깊이 없는" 데이터를 준다면, LiDAR는 "정확한 3D 좌표를 직접" 준다. 자율주행 분야에서 LiDAR가 핵심 센서로 자리잡은 이유가 바로 이 정확한 거리 측정 능력 때문이다. 카메라만으로 깊이를 추정하면 오차가 크고 날씨 영향을 받지만, LiDAR는 수 센티미터 정확도로 100m 이상 떨어진 물체까지 측정할 수 있다.

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

SLAM을 돌렸는데 드리프트가 심할 때, IMU 특성을 이해하지 못하면 원인조차 파악하기 어렵다. "IMU 바이어스가 제대로 보정되고 있나?", "이 등급의 IMU로 이 정도 정확도를 기대할 수 있나?" 같은 질문에 답하려면 IMU의 오차 모델을 제대로 이해해야 한다. Visual-Inertial Odometry(VIO)나 LiDAR-Inertial Odometry(LIO)에서 IMU는 카메라/LiDAR의 프레임 사이를 메워주는 역할을 하는데, 이 역할을 제대로 하려면 IMU 데이터의 한계를 알아야 한다.

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

단일 센서는 각각의 한계가 있으므로, 여러 센서를 결합하여 보완한다.

현실 세계에서 단일 센서만으로 완벽한 인식을 구현하는 건 불가능하다. 자율주행차는 카메라, LiDAR, Radar, IMU, GNSS를 전부 동시에 사용하고 있으며, 이 센서들의 데이터를 언제, 어디서, 어떻게 결합하느냐가 시스템 성능을 좌우한다.

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

방식마다 트레이드오프가 있다. Early Fusion은 정보 손실이 적지만 계산 비용이 높고, Late Fusion은 각 센서를 독립적으로 처리할 수 있어 모듈화에 유리하지만 정보가 일부 날아간다. Mid-Level Fusion은 그 중간이며, 최근 딥러닝 기반 퓨전에서 많이 쓰인다.

**대표적인 조합**:
- Camera + IMU → VIO (Visual-Inertial Odometry)
- LiDAR + IMU → LIO (LiDAR-Inertial Odometry)
- Camera + LiDAR + IMU → 멀티모달 SLAM

> **추천 자료**
> - [State Estimation for Robotics (Tim Barfoot) — 무료 PDF](http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) — 센서 퓨전의 수학적 기초를 다루는 대표적인 교재. Kalman Filter, Factor Graph 기반 추정을 모두 커버한다.
> - [Cyrill Stachniss — Kalman Filter & EKF](https://www.youtube.com/watch?v=E-6paM_Iwfc) — 센서 퓨전의 핵심인 칼만 필터와 EKF를 설명
> - [Qin et al. — VINS-Mono (TRO 2018)](https://arxiv.org/abs/1708.03852) — Visual-Inertial 퓨전의 대표 논문. 실제 VIO 시스템이 어떻게 구현되는지 보여준다.

> **⚠ AI 에이전트 주의**: 센서가 "데이터가 안 온다"는 문제의 원인은 대부분 소프트웨어가 아니라 물리적 연결(케이블, IP 설정, 전원, USB 대역폭)이다. AI는 드라이버 재설치부터 권하지만, `dmesg`, `lsusb`, `ping` 같은 시스템 명령으로 물리 연결부터 확인하라.

> **기술 흐름: 센서 기술**
> - **~2010**: 2D LiDAR(SICK, Hokuyo)와 단안 카메라 중심. 센서가 비싸고 크며, 처리 능력도 제한적. Stereo 카메라는 계산 비용 때문에 실시간 처리가 어려웠다.
> - **2012~2017**: 3D LiDAR(Velodyne VLP-16) 보급, RGB-D 카메라(Kinect) 대중화. LiDAR 가격이 수만 달러에서 수천 달러로 하락. Visual-Inertial 시스템(VIO)도 실제 시스템에서 쓰이기 시작했다.
> - **2018~2022**: Solid-State LiDAR(Livox) 등장, 가격이 수백 달러 수준까지 하락. Event Camera 연구 활발해짐. 멀티모달 센서 퓨전(Camera + LiDAR + IMU)이 표준으로 자리잡음.
> - **2023~**: Solid-State LiDAR가 Spinning 방식을 빠르게 대체 중. 고속/HDR 응용에서 Event Camera 채택이 늘기 시작했다. 4D Radar(도플러 속도 포함)도 새로운 보조 센서로 부상 중.
> - **지금 주목할 것**: Solid-State LiDAR가 대중화되면서 기존 Spinning LiDAR 전제의 알고리즘을 재설계해야 하는 상황이다. Event Camera는 아직 주류가 아니지만, 고속 드론·자율주행처럼 기존 카메라의 한계가 명확한 분야에서 빠르게 채택되고 있다. 센서 하드웨어가 바뀌면 알고리즘 연구 방향도 따라 바뀐다.
