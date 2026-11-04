# Ch.2 — 센서 (Sensors)

로봇은 센서로 환경을 측정한다. 센서마다 측정 원리와 오차가 다르므로, 센서를 고르고 알고리즘을 설계하려면 이 차이를 알아야 한다. SLAM 추적이 실패했을 때 원인이 rolling shutter인지, LiDAR 반사 특성인지, IMU bias인지 가릴 때도 이 지식을 쓴다.

## 2.1 카메라 (Camera)

카메라는 색상과 텍스처를 높은 공간 해상도로 측정한다. 단안·스테레오·RGB-D·event camera는 깊이를 얻는 방식, 시간 해상도, 조명 변화에 반응하는 방식이 저마다 달라서 작업 조건에 맞춰 고른다.

### 2.1.1 Monocular Camera (단안 카메라)

단안 카메라는 렌즈 하나로 2D 이미지를 찍는다. Visual SLAM, 객체 인식, 시맨틱 이해에 쓴다. 이미지 한 장에는 깊이가 없어서, 깊이는 학습 기반 단안 깊이 추정(monocular depth estimation)이나 여러 시점 이미지로 구조를 복원하는 SfM(Structure from Motion)으로 추정한다.

**장점**:
- 싸고 가볍다
- 색상과 텍스처 정보가 많다
- 해상도가 높다

**단점**:
- 이미지 한 장으로는 깊이(depth)를 잴 수 없다
- Scale ambiguity: 물체의 실제 크기를 알 수 없다(가까운 작은 물체와 먼 큰 물체가 똑같이 찍힌다)

**주요 사양**:
- 해상도: 720p, 1080p, 4K 등
- Frame rate: 30 fps, 60 fps, 120 fps 등
- Field of View (FoV): 좁은 화각부터 광각(fisheye)까지
- 셔터 방식: global shutter(모든 픽셀을 한 번에 노출) 또는 rolling shutter(줄마다 시차를 두고 노출, 빨리 움직이면 영상이 기울어진다)

```
일반적인 카메라 센서:
- 웹캠: Logitech C920, C930e
- 산업용: FLIR (Point Grey), Basler, Allied Vision
- 임베디드: Raspberry Pi Camera, OAK-D
```

> **추천 자료**
> - [First Principles of Computer Vision — Camera Calibration / Uncalibrated Stereo](https://www.youtube.com/playlist?list=PL2zRqk16wsdoCCLpou-dGo7QQNks1Ppzo) — Columbia 대학교 Shree Nayar 교수의 강의. 선형(핀홀) 카메라 모델, 내·외부 파라미터 캘리브레이션, 스테레오 기하를 설명한다.
> - [OpenCV Camera Calibration Tutorial](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html) — 카메라 캘리브레이션을 직접 해 보는 OpenCV 공식 튜토리얼

### 2.1.2 Stereo Camera (스테레오 카메라)

스테레오 카메라는 카메라 두 대를 일정 간격(baseline)으로 놓고 삼각측량으로 깊이를 구한다. 빛을 내보내지 않는 패시브 방식이라 햇빛이 강한 실외에서도 깊이를 얻는다. 빛을 쏘는 Structured Light나 ToF 방식은 실외에서 햇빛 간섭으로 성능이 떨어진다. 좌우 이미지의 같은 점을 찾는 스테레오 매칭은 에피폴라 기하학(Epipolar Geometry)을 쓴다.

**깊이 계산 원리**:

```
Depth (Z) = (focal_length × baseline) / disparity
```

- **Disparity**: 좌우 이미지에서 같은 점의 x좌표 차이
- **Baseline**: 두 카메라 사이의 거리

**장점**:
- 조명 장치가 필요 없는 패시브 센서다
- 실외에서도 쓸 수 있다
- 색 정보와 깊이를 함께 얻는다

**단점**:
- 흰 벽이나 유리처럼 텍스처가 없는 표면에서는 매칭이 실패한다
- 계산량이 많다
- 잴 수 있는 거리 범위가 baseline에 따라 정해진다

대표 제품:
- Intel RealSense D435/D455: 적외선 패턴을 쏘아 매칭을 돕는다
- ZED 2: baseline이 넓어 먼 거리까지 잰다
- OAK-D: 신경망을 돌리는 칩을 카메라에 내장했다

> **추천 자료**
> - [Cyrill Stachniss — Stereo Normal Case (5 Minutes with Cyrill)](https://www.youtube.com/watch?v=OpiqzVkwNz4) — 두 카메라를 나란히 놓은 스테레오(stereo normal case)의 기하를 5분 동안 설명한다
> - [Stanford CS231A — Epipolar Geometry and Stereo](https://web.stanford.edu/class/cs231a/) — Stanford의 Computer Vision 강의. 에피폴라 기하학과 스테레오를 다룬다.

> **실습**: [Stereo Disparity 시각화](https://alexjunholee.github.io/robotics-practice/app.html#stereo_disparity)
> 스테레오 이미지 쌍에서 disparity를 계산하고, baseline과 focal length를 바꾸면 깊이 추정이 어떻게 달라지는지 본다.

### 2.1.3 RGB-D Camera (깊이 카메라)

RGB-D 카메라는 RGB 이미지와 깊이 이미지를 함께 내보낸다. 실내 책상 위에서 SLAM이나 3D 복원을 실험하기 편하다. 실외에서 깊이 값이 비거나 여러 대를 함께 쓸 때 서로 간섭하는 이유는 아래 측정 방식에 있다.

**ToF (Time of Flight) 방식**:
- 적외선을 쏘고 돌아오는 시간(또는 위상 차)으로 거리를 잰다
- 장점: 텍스처와 상관없이 잰다, 실시간으로 처리한다
- 단점: 햇빛 간섭, 반짝이는 표면에서 생기는 오차
- 예시: Microsoft Azure Kinect, PMD Pico Flexx

**Structured Light 방식**:
- 미리 아는 패턴을 비추고, 물체 표면에서 패턴이 얼마나 일그러졌는지로 깊이를 구한다
- 장점: 가까운 거리에서 정확하다, 싸다
- 단점: 실외에서 쓰기 어렵다, 여러 대를 쓰면 서로 간섭한다
- 예시: Orbbec Astra

**Active IR Stereo 방식**:
- 좌우 적외선 카메라로 스테레오 매칭을 하고, IR 프로젝터가 쏜 무늬로 텍스처 없는 면의 매칭을 돕는다
- 예시: Intel RealSense D400 시리즈

**비교**:
| 특성 | ToF | Structured Light |
|------|-----|------------------|
| 실외 사용 | 제한적 | 어려움 |
| 정확도 | 중간 | 높음 |
| 범위 (예시 제품) | 0.25~5.46 m (Azure Kinect, 모드에 따라 다름) | 0.6~8 m (Orbbec Astra) |
| 다중 센서 | 동기화(노출 시점 분리) 시 가능, 미동기 시 간섭 | 간섭 발생 |

> **추천 자료**
> - [Intel RealSense — Depth Cameras D415 & D435](https://www.youtube.com/watch?v=A4Kjvosvx5I) — Intel이 D415·D435 깊이 카메라의 원리를 소개하는 영상
> - [Open3D RGB-D Reconstruction Tutorial](https://www.open3d.org/docs/release/tutorial/pipelines/rgbd_integration.html) — RGB-D 데이터로 3D 복원을 실습하는 튜토리얼

**RealSense 드라이버 설치 (Ubuntu 22.04)**

```bash
# Intel RealSense SDK 설치
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.realsenseai.com/Debian/librealsenseai.asc | gpg --dearmor | sudo tee /etc/apt/keyrings/librealsenseai.gpg > /dev/null
echo "deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo `lsb_release -cs` main" | \
    sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update
sudo apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev

# 테스트
realsense-viewer
```

ROS2에서 쓰려면 다음 패키지를 더 설치한다.
```bash
sudo apt install ros-humble-realsense2-camera
ros2 launch realsense2_camera rs_launch.py
```

(참고: [정진용 블로그](https://jinyongjeong.github.io/2020/06/20/Realsense-Ubuntu-driver-%EC%84%A4%EC%B9%98/))

### 2.1.4 Event Camera (이벤트 카메라)

이벤트 카메라는 프레임을 찍지 않는다. 픽셀마다 밝기가 바뀔 때 그 픽셀만 따로 이벤트를 내보낸다. 마이크로초 단위로 기록하므로 빠른 움직임도 블러 없이 이벤트 흐름으로 남는다. 빠른 움직임이나 밝기 차가 큰(HDR) 장면을 다룬다면 아래 추천 자료의 Gallego et al. survey와 rpg_dvs_ros 패키지부터 본다.

**이벤트 출력 형식**:

```
(x, y, timestamp, polarity)
- x, y: 픽셀 좌표
- timestamp: 마이크로초 단위 시간
- polarity: 밝아짐(+1) 또는 어두워짐(-1)
```

**장점**:
- 시간 해상도가 마이크로초 단위다
- 다이나믹 레인지가 140 dB로 일반 카메라(60 dB)보다 넓다
- 전력을 적게 쓰고 지연이 짧다
- 모션 블러가 없다

**단점**:
- 장면과 카메라가 모두 멈춰 있고 조명이 일정하면 이벤트가 나오지 않는다
- 프레임 영상용 CV 알고리즘을 그대로 쓰기 어렵다
- 값이 비싼 편이다

대표 제품:
- Prophesee: 고해상도 이벤트 센서
- iniVation: 이벤트와 프레임을 함께 내보내는 DAVIS
- Samsung: DVS(Dynamic Vision Sensor)

> **추천 자료**
> - [Davide Scaramuzza — Event Cameras: a New Way of Sensing (ICCP 2024 Keynote)](https://www.youtube.com/watch?v=0wGBpgIrd9M) — UZH의 Davide Scaramuzza 교수가 ICCP 2024 기조강연에서 이벤트 카메라를 개괄한다
> - [Gallego et al. — Event-based Vision: A Survey (TPAMI 2022)](https://arxiv.org/abs/1904.08405) — 이벤트 카메라의 원리, 알고리즘, 응용을 정리한 서베이 논문.
> - [rpg_dvs_ros — Event Camera ROS 드라이버](https://github.com/uzh-rpg/rpg_dvs_ros) — UZH RPG가 만든, 이벤트 카메라를 ROS에서 쓰는 오픈소스 드라이버

## 2.2 LiDAR

LiDAR (Light Detection and Ranging)는 레이저를 쏘고 반사되어 돌아온 빛(return)으로 거리를 재서 점 하나를 만든다. 3D LiDAR는 이런 점을 모아 포인트 클라우드로 내보낸다. 측정 범위와 오차는 모델, 표면 반사율, 입사각, 대기, 햇빛, return mode에 따라 달라진다. 자동차용 장거리 LiDAR 가운데 일부는 지정 반사율 조건에서 100 m 이상을 잰다. LiDAR는 radar보다 각분해능이 높고, ToF·structured-light 방식 RGB-D보다 먼 곳까지 정확히 잰다.

### 2.2.1 2D LiDAR와 3D LiDAR

**2D LiDAR**:
- 한 평면만 훑는다
- 용도: 실내 로봇 내비게이션, 장애물 회피
- 예시: SICK TiM, Hokuyo URG, RPLIDAR

**3D LiDAR**:
- 여러 층의 레이저를 회전시키거나 패턴으로 훑어 3D 포인트 클라우드를 만든다
- 용도: 자율주행, 대규모 매핑
- 예시: Velodyne VLP-16·VLP-32C·HDL-64E, Ouster OS1, Hesai

> **추천 자료**
> - [Cyrill Stachniss — LiDAR-based SLAM](https://www.youtube.com/watch?v=vrdlk2p9AZI) — ITSC 2020 SLAM 워크숍 기조강연. 기하와 시맨틱 정보를 함께 쓰는 LiDAR SLAM을 소개한다
> - [PCL (Point Cloud Library) 공식 튜토리얼](https://pcl.readthedocs.io/projects/tutorials/en/latest/) — 널리 쓰이는 공개 포인트 클라우드 처리 라이브러리

### 2.2.2 Spinning LiDAR와 Solid-State LiDAR

**Spinning (기계식)**:
- 레이저와 수광부가 돈다
- 수평으로 360°를 본다
- 단점: 회전 부품이 닳는다
- 예시: Velodyne, Ouster

**Solid-State**:
- 대형 회전 기구 없음 (flash 방식은 가동부가 없고, MEMS 방식은 미소 거울만 진동하며, Livox의 비반복 스캔은 내부 회전 프리즘을 쓴다)
- 수평 화각이 좁다 (보통 120° 이하)
- 장점: 잘 고장 나지 않고, 대량 생산하면 값이 내려갈 여지가 있다
- 예시: Livox (비반복 스캔 패턴), Innoviz

Spinning LiDAR는 360° 방향으로 고르게 퍼진 스캔 라인을 만든다. LOAM, LeGO-LOAM 같은 SLAM 알고리즘은 이 스캔 라인 구조를 가정하고 특징을 뽑는다. 스캔 패턴이 다른 Solid-State LiDAR에 쓰려면 알고리즘을 고쳐야 한다. FAST-LIO2는 특징을 따로 뽑지 않고 raw point를 지도에 바로 정합하므로 Livox 센서와 회전식 센서를 모두 다룬다.

### 2.2.3 주요 사양

| 사양 | 설명 |
| --- | --- |
| Channels | 수직 레이어 수 (16, 32, 64, 128) |
| Range | 최대 측정 거리 (50 m~300 m) |
| Points/sec | 초당 포인트 수 (30만~수백만, 128채널급은 200만 이상) |
| Accuracy | 거리 정확도 (±2 cm~±5 cm) |
| FoV | 수평/수직 화각 |

> **추천 자료**
> - [Livox 기술 문서](https://www.livoxtech.com/downloads) — Livox 센서의 사양서와 비반복 스캔 패턴 설명 자료
> - [Xu et al. — FAST-LIO2 (T-RO 2022)](https://arxiv.org/abs/2107.06829) — 특징을 뽑지 않고 raw point를 바로 정합하며, 증분 k-d tree(ikd-Tree)로 지도를 갱신해 spinning·solid-state LiDAR를 모두 다루는 LiDAR-Inertial Odometry 논문

## 2.3 IMU (Inertial Measurement Unit)

IMU는 가속도와 각속도를 잰다. Visual-Inertial Odometry(VIO)나 LiDAR-Inertial Odometry(LIO)에서 IMU는 카메라나 LiDAR 프레임 사이의 움직임을 채운다. SLAM 드리프트가 클 때 바이어스가 제대로 보정됐는지, 그 등급의 IMU라면 어느 정도 정확도를 기대해야 하는지는 아래 오차 모델로 따진다.

### 2.3.1 구성 요소

**Accelerometer (가속도계)**:
- 3축 선형 가속도를 잰다 (m/s²)
- 측정값에 중력 가속도가 섞여 있다

**Gyroscope (자이로스코프)**:
- 3축 각속도를 잰다 (rad/s 또는 deg/s)

**Magnetometer (지자기 센서)** (일부 IMU):
- 3축 자기장을 잰다
- 이를 이용해 절대 방위(heading)를 추정한다
- 주변 철골이나 전류가 만드는 자기장 왜곡에 약하다

### 2.3.2 주요 오차 특성

센서 퓨전 시스템은 아래 IMU 오차를 상태 변수나 잡음으로 모델링한다.

**Bias (바이어스)**:
- 가만히 있어도 출력이 0이 아니다
- 시간이 지나며 천천히 변한다. 그 크기를 bias instability라 하고, Allan deviation 곡선의 가장 낮은 값으로 정한다. 온도 때문에 바뀌는 thermal drift는 따로 다룬다

**Noise**:
- 고주파 랜덤 노이즈
- Allan Variance로 크기를 잰다

**Integration Drift**:

각속도를 적분하면 자세 오차가 쌓인다. 가속도를 두 번 적분해 위치를 구하면 noise, bias, scale factor 오차와 초기 자세 오차가 함께 쌓인다. 오차가 커지는 속도는 센서, 움직임, 보정, 온도, 초기화에 따라 다르다. 값싼 MEMS IMU만으로 위치를 추정하면 오차가 빠르게 커지므로, 오래 위치를 알아야 하는 시스템은 camera, LiDAR, GNSS 같은 외부 관측으로 drift를 바로잡는다.

IMU 등급은 bias stability, noise density, scale-factor error, 온도 보정, 진동 내성, 인증 요구로 나눈다. 소비자용(consumer) MEMS는 작고 전력을 적게 쓰도록 만든다. industrial, tactical, navigation 등급으로 올라갈수록 오래 안정적이고 보정 범위가 넓다. `VN-100`, `MTi`, `KVH 1750`, `HG1700` 같은 제품을 비교할 때는 등급 이름보다 같은 단위의 최신 datasheet와 Allan 측정 결과를 확인한다.

> **추천 자료**
> - [Probabilistic Robotics, Ch.5–6 — Robot Motion / Robot Perception (Thrun, Burgard, Fox)](http://probabilistic-robotics.org/) — 모션 모델(5장)과 센서 측정 모델(6장)을 다룬다. IMU 오차 모델(bias, random walk, Allan variance)은 이 책에 없으니 Titterton & Weston을 본다.
> - [Titterton & Weston — Strapdown Inertial Navigation Technology](https://doi.org/10.1049/PBRA017E) — IMU 원리와 strapdown 관성 항법을 다루는 교과서
> - [Allan Variance — IMU 노이즈 분석 가이드 (Vectornav)](https://www.vectornav.com/resources/inertial-navigation-primer/specifications--and--error-budgets/specs-imucal) — Allan Variance로 IMU 노이즈 파라미터를 구하는 방법을 설명한다
> - [정진용 블로그 — IMU Filter (AHRS)](https://jinyongjeong.github.io/2020/01/10/IMU_filter/) — IMU로 자세를 추정하는 AHRS(Attitude and Heading Reference System) 필터를 설명하고, Madgwick 필터와 ROS 패키지를 소개한다

---

## 2.4 GNSS / GPS

GNSS(Global Navigation Satellite System)는 실외 자율주행차와 드론에 위도·경도·고도의 전역 좌표를 준다. 실외 로봇은 이 좌표를 SLAM이 추정한 출발점 기준 상대 위치와 결합해 쓴다. 데이터셋에서는 RTK-GPS 측정치를 위치 ground truth로 많이 쓴다.

하늘이 트인 곳에서 보정 없이 수신기 하나로 위치를 구하면(standalone) 오차가 미터 수준이다. 기준국 보정(differential correction)을 받으면 환경에 따라 1 m 아래에서 센티미터 수준까지 줄어든다. 고층 건물 사이에서는 신호가 건물에 반사되어(multipath) RTK도 반송파의 정수 모호성(integer ambiguity)을 풀지 못하고, 오차가 수 미터로 커질 수 있다. 그래서 도시 자율주행은 GNSS에 LiDAR·카메라 SLAM을 결합한다. GNSS 정확도를 인용할 때는 지표(CEP·RMS·95%), 수평/수직 구분, 기준국 거리(baseline), 보정 신호 경로, fix 상태를 함께 적는다.

**RTK-GPS 원리**:
- 위치를 아는 기준국(base station)이 보정 데이터를 보낸다
- 움직이는 수신기(rover)가 이를 받아 위치를 보정한다
- 둘 사이에 실시간 통신(무선 또는 인터넷)이 있어야 한다

**한계**:
- 실내와 터널에서는 쓰기 어렵고, 고층 건물 사이에서는 수신이 나빠진다
- 건물에 반사된 신호 때문에 오차가 생긴다(multipath)
- 고도는 수평 위치보다 덜 정확하다

> **추천 자료**
> - [Cyrill Stachniss — Robot Localization Overview](https://www.youtube.com/watch?v=8VJ-A9OlhAE) — 로봇 위치 추정 방법을 개괄하는 강의
> - [u-blox — RTK와 GNSS 보정](https://www.u-blox.com/en/technologies/rtk-real-time-kinematic) — 의사거리(code)로 위치를 구하는 방식과 반송파 위상(carrier phase)을 쓰는 RTK가 어떻게 다른지, 보정 서비스는 어떻게 쓰는지 설명하는 u-blox 자료

## 2.5 기타 센서

**Radar**

자율주행차와 로봇은 radar를 camera·LiDAR와 함께 쓴다. 전파는 가시광이나 LiDAR의 적외선보다 안개·비·먼지·역광의 영향을 덜 받는다. 대신 radar는 각해상도가 낮다. 강한 비에는 신호가 약해지고, 원치 않는 반사(clutter)와 다중 반사(multipath)가 섞이며, 덮개(radome)가 젖으면 성능이 떨어진다. 가격은 안테나 수, 대역폭, 이미징 기능, 자동차 인증에 따라 다르므로 현재 견적으로 비교한다.

FMCW (Frequency Modulated Continuous Wave) Radar:
- 시간에 따라 주파수를 바꾸며 전파를 보내고, 보낸 신호와 돌아온 반사파의 주파수 차이로 거리와 속도를 함께 잰다.
- 출력: Range-Doppler map(거리 × 속도 2D 맵), Range-Azimuth map(거리 × 방위각 2D 맵)
- 자동차용 77 GHz radar가 가장 흔하다.

**로봇에서 쓰는 곳**:
- 자율주행: 전방 충돌 감지, 적응형 크루즈 컨트롤 (ACC)
- Radar odometry: 레이더만으로 자기 위치 변화를 추정한다
- Radar SLAM: 레이더로 지도를 만들며 위치를 추정한다

**카메라, LiDAR, Radar 비교**:

| 특성 | 카메라 | LiDAR | Radar |
|------|--------|-------|-------|
| 해상도 | 매우 높음 | 높음 | 낮음 |
| 거리 측정 | 불가 (단안) | 정확 | 가능 |
| 속도 측정 | 불가 | 불가 (ToF 방식 기준, FMCW LiDAR는 Doppler로 가능) | 가능 (Doppler) |
| 악천후 | 취약 | 취약 (비, 안개) | 강건 |
| 가격 | 저렴 | 비쌈 | 중간 |
| 야간 | 취약 (조명 필요) | 가능 | 가능 |

대표 제품: Texas Instruments AWR1843, Continental ARS548, Navtech CTS350-X (spinning radar)

> **추천 자료**
> - [김기섭 블로그 — ICRA 2021 Radar in Robotics Workshop 요약](https://gisbi-kim.github.io/blog/2021/05/31/icra21-radar-ws.html) — ICRA 2021 Radar in Robotics 워크숍 발표를 정리한 글
> - [김기섭 블로그 — Radar Odometry Results on MulRan dataset](https://gisbi-kim.github.io/blog/2021/05/30/yeti-radar-odom-mulran1.html) — Yeti radar odometry(Cen2018·Cen2019 방식)를 MulRan 도시 시퀀스에 돌린 궤적 결과
> - [Kim et al., "MulRan: Multimodal Range Dataset for Urban Place Recognition" (ICRA 2020)](https://sites.google.com/view/mulran-pr/home) — LiDAR + radar + GPS 멀티모달 데이터셋

**Ultrasonic (초음파)**:
- 가까운 장애물을 감지한다 (대략 0.2~5 m)
- 싸다
- 주차 보조와 근접 감지에 쓴다

**Wheel Encoder (휠 엔코더)**:
- 바퀴가 얼마나 돌았는지 잰다
- 이 값을 누적해 위치를 추정한다(dead reckoning)
- 바퀴가 미끄러지면 오차가 생긴다



> **추천 자료**
> - [Probabilistic Robotics, Ch.6 — Robot Perception (Thrun, Burgard, Fox)](http://probabilistic-robotics.org/) — 거리 센서와 특징 기반 센서의 확률 모델을 다룬다.

## 2.6 센서 퓨전 (Sensor Fusion)

센서마다 약한 조건이 다르다. 카메라는 어두우면, LiDAR는 비와 안개 속에서, IMU는 시간이 오래 지나면 믿기 어렵다. 여러 센서를 결합하면 한 센서가 약한 조건을 다른 센서가 메운다. 자율주행차는 운행 조건에 맞춰 카메라·LiDAR·Radar·IMU·GNSS 가운데 여러 종류를 함께 쓴다.

**왜 필요한가?**

| 센서 | 장점 | 단점 |
| --- | --- | --- |
| Camera | 풍부한 정보, 저렴 | 조명 의존, 깊이 없음 |
| LiDAR | 정확한 3D, 조명 무관 | 비쌈, sparse |
| IMU | 고주파, 조명 무관 | 드리프트 |
| GPS | 전역 위치 | 실외 전용, 저주파 |

**퓨전 방식**:
- **Early Fusion**: 센서의 raw 데이터를 합친다. 원본 정보를 모두 쓰지만 계산량이 많다.
- **Mid-Level Fusion**: 센서마다 뽑은 feature를 합친다. 딥러닝 기반 퓨전에서 많이 쓴다.
- **Late Fusion**: 센서마다 따로 처리한 결과를 합친다. 모듈을 나누기 쉽지만, 결과로 요약하는 동안 정보가 줄어든다.

**대표적인 조합**:
- Camera + IMU → VIO (Visual-Inertial Odometry)
- LiDAR + IMU → LIO (LiDAR-Inertial Odometry)
- Camera + LiDAR + IMU → 멀티모달 SLAM

퓨전에 쓰는 확률 측정 모델은 §2.7에 있다.

> **추천 자료**
> - [State Estimation for Robotics (Tim Barfoot) — 무료 PDF](https://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) — 로봇 상태 추정 교재. Kalman Filter 계열과 batch 최적화 기반 추정, 3차원 회전·자세 표현을 다룬다. 저자가 PDF를 무료로 공개했다.
> - [Cyrill Stachniss — Kalman Filter & EKF](https://www.youtube.com/watch?v=E-6paM_Iwfc) — 칼만 필터와 EKF를 설명하는 강의
> - [Qin et al. — VINS-Mono (TRO 2018)](https://arxiv.org/abs/1708.03852) — 단안 카메라와 IMU를 결합한 VIO 시스템 논문. 초기화, 최적화 기반 추정, loop closure까지 실제 시스템의 구성을 보여 준다.

---

## 2.7 심화: 확률 측정 모델 (Probabilistic Measurement Models)

베이즈 필터, SLAM, MCL(Monte Carlo Localization, 입자 필터(particle filter)로 하는 위치 추정)은 측정값을 얼마나 믿을지 측정 모델 $p(z_t \mid x_t, m)$로 나타낸다.

### 2.7.1 센서가 만드는 분포

레이저 거리 센서를 같은 자세로 같은 벽을 향해 100번 쏘면 측정값 100개가 조금씩 다르다. 빔이 반사면에 비스듬히 맞아 엉뚱한 쪽으로 튕기거나, 사람이 지나가거나, 여러 번 반사되기 때문이다. 원인마다 측정값이 흩어지는 모양이 다르다. $p(z_t^k \mid x_t, m)$는 이 흩어짐을 확률 분포 하나로 나타낸다. $z_t^k$는 시각 $t$의 $k$번째 빔 측정값, $x_t$는 로봇 포즈, $m$은 환경 지도다.

스캔 하나에는 빔이 수십~수백 개 있다. PR §6.1(식 6.2)은 빔마다 오차가 따로 생긴다고 가정한다(조건부 독립). 그러면 전체 스캔의 likelihood는 빔마다 구한 likelihood의 곱이다.

$$p(z_t \mid x_t, m) = \prod_{k=1}^{K} p(z_t^k \mid x_t, m)$$

실제로는 같은 벽을 보는 이웃 빔들의 오차가 서로 얽혀 있다. 이를 무시하면 likelihood가 한 포즈에 지나치게 몰린다. 대처법은 §2.7.4와 §2.7.8에 있다.

지도 $m$은 feature-based 지도와 location-based 지도로 나뉜다. feature-based 지도는 랜드마크 목록이고, 랜드마크마다 ID로 찾는다. location-based 지도는 격자 셀마다 점유 확률을 담은 배열이고, 셀을 좌표로 찾는다.

측정 모델은 다음과 같다.
- 빔 모델(beam model): 측정값이 생긴 원인별 분포를 섞은 혼합 모델. location-based 지도를 쓴다.
- likelihood field: 빔 끝점에서 가장 가까운 장애물까지의 거리로 평가한다. location-based 지도를 쓴다.
- 상관 기반(map matching): local map과 global map의 정규화 상관계수로 평가한다. 격자 지도를 쓴다.
- 특징 기반(landmark model): 뽑아낸 특징을 (range, bearing, signature)로 모델링한다. feature-based 지도를 쓴다.

### 2.7.2 빔 모델 — 4성분 혼합

빔 모델은 거리 센서의 빔 하나가 낸 측정값을 네 가지 원인으로 나눠 설명한다. [Thrun et al. 2005](http://probabilistic-robotics.org/) (PR §6.3.1)는 원인마다 확률 분포를 두고, 네 분포를 가중치로 섞어 likelihood를 만든다.

hit은 빔이 지도의 장애물에 맞고 돌아온 경우다. 평균이 예측 거리 $z_t^{k*}$이고 분산이 $\sigma_{\text{hit}}^2$인 가우시안을 $[0, z_{\max}]$ 범위로 잘라 쓴다.

$$p_{\text{hit}}(z_t^k \mid x_t, m) = \eta\, \mathcal{N}(z_t^k;\, z_t^{k*},\, \sigma_{\text{hit}}^2), \quad 0 \le z_t^k \le z_{\max}$$

$\eta$는 잘라 낸 범위에서 확률을 모두 더하면 1이 되게 맞추는 정규화 상수다. 아래 short 식의 $\eta$도 같은 역할을 한다.

short는 지도에 없는 장애물(지나가는 사람, 다른 로봇)이 빔을 가로막은 경우다. 측정값이 $z_t^{k*}$보다 짧고, $[0, z_t^{k*}]$ 범위의 지수 분포로 모델링한다.

$$p_{\text{short}}(z_t^k \mid x_t, m) = \eta\, \lambda_{\text{short}}\, e^{-\lambda_{\text{short}} z_t^k}, \quad 0 \le z_t^k \le z_t^{k*}$$

max는 검은 표면에 빛이 흡수되거나, 매끈한 면에 비스듬히 맞아 다른 쪽으로 튕기거나, 안개에 흩어져서 반사파가 돌아오지 않는 경우다. 센서는 최대 거리 $z_{\max}$를 내보낸다. $z_{\max}$ 한 점에 확률을 모두 둔 점질량으로 모델링한다.

$$p_{\text{max}}(z_t^k \mid x_t, m) = \mathbf{1}[z_t^k = z_{\max}]$$

rand는 sonar끼리의 간섭(crosstalk)이나 다중 반사처럼 설명할 수 없는 측정값이다. $[0, z_{\max}]$의 균등 분포로 모델링한다.

$$p_{\text{rand}}(z_t^k \mid x_t, m) = \frac{1}{z_{\max}}$$

빔 하나의 likelihood는 네 분포를 가중치로 섞은 값이다 (PR 식 6.13).

$$p(z_t^k \mid x_t, m) = \begin{pmatrix} z_{\text{hit}} \\ z_{\text{short}} \\ z_{\text{max}} \\ z_{\text{rand}} \end{pmatrix}^T \cdot \begin{pmatrix} p_{\text{hit}}(z_t^k \mid x_t, m) \\ p_{\text{short}}(z_t^k \mid x_t, m) \\ p_{\text{max}}(z_t^k \mid x_t, m) \\ p_{\text{rand}}(z_t^k \mid x_t, m) \end{pmatrix}$$

가중치를 모두 더하면 1이다: $z_{\text{hit}} + z_{\text{short}} + z_{\text{max}} + z_{\text{rand}} = 1$.

예측 거리 $z_t^{k*}$는 포즈 $x_t$와 지도 $m$에서 ray casting으로 계산한다. 포즈에서 빔 방향으로 선을 그어 처음 닿는 점유 셀까지의 거리가 $z_t^{k*}$다.

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

센서 종류, 환경, 장착 위치가 바뀌면 hit/short/max/rand의 비율과 분산이 달라진다. 한 환경에서 손으로 맞춘 파라미터는 다른 환경에 맞지 않는다.

조정할 내재 파라미터(intrinsic parameter)는 $z_{\text{hit}}, z_{\text{short}}, z_{\text{max}}, z_{\text{rand}}, \sigma_{\text{hit}}, \lambda_{\text{short}}$다. PR §6.3.2는 지도를 아는 환경에서 로봇을 움직이며 모은 데이터 $\{(z_t^k, z_t^{k*})\}$로 이 파라미터를 최대우도 추정하고, 대응 변수를 모를 때는 EM 알고리즘을 쓴다.

잠재 변수인 대응 변수(correspondence variable) $c_i \in \{\text{hit, short, max, rand}\}$는 측정값 $z_t^k$가 어느 성분에서 나왔는지 나타낸다.

E-step에서는 현재 파라미터로 측정값마다 네 성분의 사후 확률을 계산한다 (PR 식 6.28~6.29).

$$e_{\text{hit}}^i = \frac{z_{\text{hit}} \cdot p_{\text{hit}}(z^i \mid z^{i*})}{p(z^i \mid z^{i*})}, \quad e_{\text{short}}^i = \frac{z_{\text{short}} \cdot p_{\text{short}}(z^i \mid z^{i*})}{p(z^i \mid z^{i*})}, \quad \dots$$

M-step에서는 이 기댓값으로 파라미터를 갱신한다 (PR 식 6.30~6.32). $\sigma_{\text{hit}}$와 $\lambda_{\text{short}}$는 닫힌 해가 있다.

$$\sigma_{\text{hit}}^2 = \frac{\sum_i e_{\text{hit}}^i (z^i - z^{i*})^2}{\sum_i e_{\text{hit}}^i}, \qquad \lambda_{\text{short}} = \frac{\sum_i e_{\text{short}}^i}{\sum_i e_{\text{short}}^i \cdot z^i}$$

혼합 가중치 $z_{\text{hit}}, z_{\text{short}}, z_{\text{max}}, z_{\text{rand}}$는 성분별 기댓값의 평균으로 갱신한다.

**알고리즘: learn_intrinsic_parameters** (PR Table 6.2 압축)

```
입력: {(z^i, z^{i*})} — (측정값, 예측값) 쌍
출력: z_hit, z_short, z_max, z_rand, σ_hit, λ_short

초기화: σ_hit, λ_short는 적당한 값으로, 혼합 가중치는 균등하게 둔다 (PR은 σ_hit, λ_short의 초기값이 좋아야 한다고 적는다)
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

EM은 특정 센서·지도·환경에서 모은 데이터에 맞춰 파라미터를 추정한다. AMCL은 `sigma_hit`, `lambda_short`와 혼합 가중치를 설정값으로 열어 둔다. 패키지 기본값은 출발점일 뿐이니, 쓰려는 환경에서 다시 추정하거나 조정한다.

### 2.7.4 빔 모델 실무 고려

빔 모델에서 계산이 가장 오래 걸리는 곳은 ray casting이다. MCL에서 파티클마다 스캔의 모든 빔에 ray casting을 하면 파티클 수 × 빔 수만큼 계산해야 한다.

계산을 줄이려면 먼저 빔 수를 줄인다. 전체 스캔에서 균일 간격으로 일부 빔만 쓰면 계산량과 인접 빔의 중복이 함께 줄어든다. 사용할 빔 수는 scan resolution, 환경 구조, 파티클 수에 맞춰 검증한다.

빔이 서로 독립이라는 가정이 맞지 않으면 likelihood를 $p(z_t \mid x_t, m)^{\alpha}$ ($0 < \alpha < 1$)로 거듭제곱해 낮춘다. 그러면 스캔 하나가 포즈 분포를 지나치게 좁히는 과신(over-confidence)이 줄어든다. $\alpha$는 경험으로 정하거나 교차 검증으로 정한다.

지도의 모든 (셀, 방향) 조합마다 ray casting 결과를 미리 계산해 테이블에 저장해 두면, 실행 중에는 테이블만 찾으면 된다($O(1)$). 실시간 MCL에서 효과가 크지만, 지도가 크면 메모리를 많이 쓴다 (Ch.3 §3.11.3 입자 필터 참조).

### 2.7.5 Likelihood Field

빔 모델에서는 포즈 $x_t$가 조금만 바뀌어도 빔이 다른 장애물에 먼저 닿으면 $z_t^{k*}$가 갑자기 크게 변한다. likelihood가 포즈에 따라 끊기며 바뀌므로 gradient 기반 정합이나 hill-climbing이 잘 수렴하지 않는다.

likelihood field는 빔 끝점을 전역 좌표계로 옮긴 뒤, 그 점에서 가장 가까운 점유 셀까지의 유클리드 거리 $\text{dist}$로 likelihood를 매긴다.

빔 끝점을 전역 좌표로 옮기는 식 (PR 식 6.33):

$$\begin{pmatrix} x_{z_t^k} \\ y_{z_t^k} \end{pmatrix} = \begin{pmatrix} x \\ y \end{pmatrix} + \begin{pmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{pmatrix} \begin{pmatrix} x_{k,\text{sens}} \\ y_{k,\text{sens}} \end{pmatrix} + z_t^k \begin{pmatrix} \cos(\theta + \theta_{k,\text{sens}}) \\ \sin(\theta + \theta_{k,\text{sens}}) \end{pmatrix}$$

여기서 $(x, y, \theta)$는 로봇 포즈 $x_t$의 성분, $(x_{k,\text{sens}}, y_{k,\text{sens}})$는 로봇 기준계에서 $k$번째 빔 센서의 위치, $\theta_{k,\text{sens}}$는 빔의 방향 오프셋이다.

빔 하나의 likelihood (PR 식 6.34~6.35, max-range 빔을 버렸으므로 $p_{\max}$ 항은 빠진다):

$$p(z_t^k \mid x_t, m) = z_{\text{hit}} \cdot \mathcal{N}(\text{dist};\, 0,\, \sigma_{\text{hit}}^2) + z_{\text{rand}} \cdot \frac{1}{z_{\max}}$$

가우시안의 평균은 0이다. max-range 빔($z_t^k = z_{\max}$)은 끝점이 실제 물체 위치가 아니므로 계산에서 뺀다.

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

지도가 바뀌지 않으면 거리 변환(distance transform)을 한 번만 계산해 테이블로 저장해 둔다. 그러면 $\text{dist}$를 $O(1)$에 찾는다. 이 테이블은 SDF(Signed Distance Field)의 양수 영역과 같다. likelihood가 포즈 $x_t$에 따라 매끄럽게 변해 gradient를 구할 수 있으므로 gradient 기반 scan matching에 맞는다.

likelihood field에는 short 성분이 없어서, 움직이는 장애물이 만든 측정은 rand 성분으로 처리한다. 끝점과 가장 가까운 장애물 사이의 거리만 보고 빔 경로가 막혔는지는 보지 않으므로, 센서가 벽 너머를 보는 것처럼 계산된다. 지도는 정확하다고 가정한다.

2D LiDAR로 실내를 다니는 로봇에서 AMCL은 likelihood field를 기본값으로 쓴다. 계산이 빠르고 포즈에 따라 매끄럽게 변하기 때문이다. 3D LiDAR와 RGB-D에서는 ICP, NDT 정합을 많이 쓴다(정합 알고리즘은 Ch.13 §13.2.3, 위치 추정 필터는 Ch.14 §14.7).

### 2.7.6 상관 기반 모델 (Map Matching)

상관 기반 모델은 최근 스캔 몇 개로 local map $m_{\text{local}}$을 만들고, 이를 global map $m$과 정규화 상관계수 $\rho$로 비교한다. PR §6.5는 이 값을 그대로 likelihood로 쓴다.

$$p(m_{\text{local}} \mid x_t, m) = \max\{\rho(m_{\text{local}}, m \mid x_t),\ 0\}$$

$\rho$는 두 맵을 $x_t$로 맞춰 놓았을 때 대응하는 셀끼리 구한 피어슨 상관계수이고, 0보다 작으면 0으로 자른다. $\rho$를 확률처럼 쓰는 데는 이론 근거가 없고 경험으로 택한 것이다. 계산이 빠르고 구현이 단순해서, SLAM 백엔드의 loop closure 검출처럼 빠른 유사도 점수가 필요한 곳에 쓴다.

### 2.7.7 특징 기반 측정 — 랜드마크 모델

랜드마크 모델은 센서 데이터에서 뽑은 특징 $f(z_t)$를 쓴다. 특징은 차원이 낮아 계산량이 적고, feature-based 지도와 바로 맞물린다.

어떤 특징을 뽑을지는 센서마다 다르다. 거리 스캔에서는 선분, 코너, 국소 극솟값을 뽑는다. 카메라에서는 edge, corner, SIFT/ORB 같은 국소 패턴을 뽑는다. 뽑은 특징은 $(r, \phi, s)$로 나타낸다. $r$은 거리(range), $\phi$는 방위각(bearing), $s$는 signature(ID, 색상, 디스크립터 같은 식별 정보)다.

지도의 $j$번째 랜드마크가 좌표 $(m_{j,x}, m_{j,y})$에 있고 signature가 $s_j$라고 하자. 포즈 $x_t = (x, y, \theta)^T$에서 이 랜드마크를 본 측정은 다음과 같다 (PR 식 6.41).

$$\begin{pmatrix} r_t^i \\ \phi_t^i \\ s_t^i \end{pmatrix} = \begin{pmatrix} \sqrt{(m_{j,x} - x)^2 + (m_{j,y} - y)^2} \\ \operatorname{atan2}(m_{j,y} - y,\, m_{j,x} - x) - \theta \\ s_j \end{pmatrix} + \begin{pmatrix} \varepsilon_{\sigma_r^2} \\ \varepsilon_{\sigma_\phi^2} \\ \varepsilon_{\sigma_s^2} \end{pmatrix}$$

$\varepsilon_{\sigma^2}$는 평균 0, 분산 $\sigma^2$인 가우시안 잡음이고, 세 채널의 잡음은 서로 독립이라고 가정한다. bearing에 가우시안을 그대로 더하면 각도가 $\pm\pi$ 근처에서 반대편으로 넘어갈 때(wrap-around) 오차가 거의 $2\pi$로 잘못 계산된다. 그래서 실제 구현에서는 각도 차이를 $[-\pi, \pi]$로 정규화하거나 von Mises 분포를 쓴다.

$i$번째 특징이 $j$번째 랜드마크에 대응한다는 것($c_t^i = j$)을 알면, likelihood는 세 채널 가우시안의 곱이다.

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

특징끼리 조건부 독립이라고 가정하면(PR 식 6.40) 전체 likelihood는 특징별 likelihood의 곱 $\prod_i$다.

거꾸로 측정에서 포즈를 샘플링할 수도 있다 (PR Table 6.5 압축). $(r, \phi)$ 측정 하나는 3차원 포즈 $(x, y, \theta)$에 제약을 둘만 주므로, 가능한 포즈는 랜드마크를 중심으로 한 반지름 $r$의 원 위에 놓이고 각 위치마다 방향 $\theta$가 하나씩 정해진다. 원 위의 위치는 $\hat{\gamma} \sim U(0, 2\pi)$로 샘플링한다.

<!-- DEMO: landmark_donut.html -->

Visual SLAM의 reprojection residual $\| \pi(K[R|t]\, X_w) - u \|^2_\Sigma$도 pose와 landmark로 관측을 예측해 실제 관측과 비교한다. 측정이 픽셀 좌표라는 점에서 range-bearing 모델과는 다른 sensor model이다. ORB/SIFT descriptor는 주로 어느 특징이 어느 랜드마크인지 맞추는 data association에 쓴다. AprilTag·ArUco의 ID를 쓰면 이 대응을 거의 헷갈리지 않지만, 잘못 검출하거나 ID를 잘못 읽는 일은 여전히 생긴다.

### 2.7.8 실무 정리: 모델 선택

아래 표는 네 모델을 대략 비교한 것이다. 실제 정확도와 속도는 센서, 지도 해상도, 구현, 파라미터에 따라 달라진다.

| 모델 | 정확도 | 계산 속도 | 미분 가능성 | 주요 용도 |
|------|--------|-----------|------------|-----------|
| 빔 모델 | 높음 | 느림 (ray casting) | 낮음 (불연속) | MCL high-fidelity, 진단 |
| Likelihood field | 중간 | 빠름 (DT lookup) | 높음 | AMCL 기본, gradient 정합 |
| 상관 기반 | 낮음 | 매우 빠름 | 낮음 | loop closure 검출 |
| 랜드마크 | 높음 (특징 의존) | 빠름 (저차원) | 높음 | visual SLAM, fiducial |

빔이 서로 독립이라는 가정이 깨질 때는 §2.7.4처럼 likelihood를 $\alpha$ 제곱해 낮춘다(tempering). 빔을 솎아 쓰거나(beam subsampling), 빔 사이 상관을 반영한 모델이나 outlier에 강한 likelihood를 쓰기도 한다.

### 2.7.9 지금 시스템에서 쓰는 측정 모델

최신 scan matcher와 visual SLAM도 관측과 예측을 비교한다. 목적함수와 지도 표현은 시스템마다 다르다.

[Nav2 AMCL 문서](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/others/configuring_amcl/)에 따르면 laser model은 `beam`, `likelihood_field`, `likelihood_field_prob` 가운데 고를 수 있고, 기본값은 `likelihood_field`다. `max_beams`는 한 스캔에서 균등 간격으로 쓸 빔 수다. `beam_skip_*`는 `likelihood_field_prob`에서 파티클 대부분과 맞지 않는 빔을 건너뛰는 기능이다.

다른 LiDAR 시스템은 자기만의 정합 목적함수를 쓴다. Cartographer는 확률 격자(probability grid)에서 correlative scan matching을 한 뒤 비선형 최적화로 다듬는다. `hdl_localization`은 3D point cloud를 NDT로 정합한다(NDT_OMP·NDT_CUDA).

PR 식 6.41의 range-bearing 모델, 카메라 reprojection 모델, DROID-SLAM의 dense bundle adjustment는 모두 pose와 장면 구조로 관측을 예측해 residual을 만든다. 다만 무엇을 측정으로 보는지, 대응을 어떻게 찾는지, 잡음을 어떻게 모델링하는지, 무엇을 최적화하는지는 서로 다르다.

Ch.14 §14.7.6의 MCL 의사코드가 부르는 `measurement_model` 자리에 이 절의 `beam_range_finder_model`이나 likelihood field 모델이 들어간다. §14.7B의 occupancy grid mapping은 역방향 모델인 `inverse_sensor_model`을 쓴다.

> **⚠ 센서 연결 점검**: 센서 데이터가 들어오지 않으면 드라이버, 케이블, IP 설정, 전원, USB 대역폭을 확인한다. `dmesg`, `lsusb`, `ping`으로 장치와 연결 상태를 먼저 기록한다.

> **기술 흐름: 센서 기술**
> - **~2010**: 이동 로봇 연구에서는 2D LiDAR와 일반 카메라를 널리 썼다. 당시 연산 자원으로는 스테레오를 실시간으로 처리하기 버거웠다.
> - **2010년대**: Kinect(2010) 같은 RGB-D 카메라와 작은 multi-beam 3D LiDAR가 널리 퍼졌다. 카메라와 IMU를 결합한 VIO가 로봇과 AR 기기에 들어갔다.
> - **2010년대 후반~2020년대 초반**: 비반복 스캔, MEMS, flash처럼 서로 다른 방식이 `solid-state LiDAR`라는 이름으로 나왔다. event camera와 자동차용 imaging radar 연구도 늘었다.
> - **2020년대**: Spinning LiDAR와 solid-state 계열을 field of view, range, resolution, motion distortion, cost에 따라 골라 쓴다. Event camera와 Doppler radar는 고속·HDR·악천후 같은 조건에서 쓴다.
> - 센서의 스캔 방식과 timestamp 구조가 바뀌면 deskew(스캔 중 움직임 보정), calibration, data association의 가정도 바뀐다. 센서를 바꾸면 실제로 점이 어떻게 찍히는지와 노이즈 특성부터 확인한다.
