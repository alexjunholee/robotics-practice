# 2장 센서

1장에서 살펴본 것처럼 로봇이 다루는 불확실성은 대부분 물리 세계를 측정하는 첫 관문, 즉 센서에서 비롯된다. 로봇은 눈과 귀에 해당하는 여러 센서로 주변 환경을 수치화하는데, 센서마다 빛·전파·관성을 재는 물리 원리와 고유한 오차 패턴이 전혀 다르다. 따라서 작업 환경에 맞는 센서를 고르고 그에 걸맞은 알고리즘을 설계하려면 각 센서의 동작 원리와 한계를 먼저 이해해야 한다. 실제로 현장에서 SLAM 추적이 갑자기 끊겼을 때, 원인이 카메라의 rolling shutter 왜곡인지, LiDAR의 반사 특성 때문인지, 아니면 IMU의 bias 드리프트인지 가려내는 진단 작업 역시 이 기초 지식 위에서 이루어진다.

## 2.1 카메라

카메라는 주변의 색상과 미세한 질감(텍스처)을 가장 높은 공간 해상도로 담아내는 센서다. 다만 일반 카메라 한 대는 3차원 장면을 2차원 평면에 눌러 찍기 때문에 거리 정보를 직접 내주지 못하며, 극단적인 속도나 급격한 조명 변화 앞에서는 화질이 쉽게 무너진다. 깊이를 복원하는 방식과 빛·시간 변화에 반응하는 설계에 따라 로봇에서 쓰는 카메라는 크게 단안, 스테레오, RGB-D, 이벤트 카메라 네 갈래로 나뉜다.

### 2.1.1 단안 카메라 (Monocular Camera)

단안 카메라는 렌즈 하나로 2차원 이미지를 촬영하는 가장 기본적인 시각 센서다. 가격이 저렴하고 가벼우며 색상·텍스처 정보가 풍부해 Visual SLAM, 객체 인식, 장면의 의미 이해까지 두루 쓰인다.

하지만 단안 카메라가 찍은 사진 한 장에는 **깊이(depth)** 정보가 빠져 있다. 3차원 공간의 한 광선 위에 놓인 점들이 전부 이미지 평면의 같은 픽셀에 투영되기 때문이다. 그 결과 물체의 실제 크기를 알 수 없는 **스케일 모호성(scale ambiguity)**이 생긴다. 예컨대 바로 눈앞에 놓인 작은 장난감 자동차와 멀리 도로 위에 서 있는 실제 자동차가 카메라 화면에는 똑같은 크기로 찍힐 수 있다. 그래서 단안 카메라만으로 3차원 거리를 알아내려면, 학습 기반 단안 깊이 추정(monocular depth estimation) 모델에 의존하거나 카메라를 움직여가며 여러 시점의 이미지를 엮어 구조를 복원하는 SfM(Structure from Motion) 기법을 써야 한다.

실무에서 단안 카메라를 고를 때는 해상도와 초당 프레임 수(frame rate), 화각(Field of View)뿐 아니라 **셔터 방식**을 유심히 살펴야 한다. 모든 픽셀을 동시에 노출하는 global shutter와 달리, 많은 저가형 카메라는 이미지의 위쪽 줄부터 아래쪽 줄까지 미세한 시차를 두고 차례로 노출하는 rolling shutter를 쓴다. 로봇이 천천히 움직일 때는 문제가 드러나지 않지만, 드론이나 고속 주행 로봇처럼 빠르게 회전하거나 진동할 때는 수직 기둥이 비스듬히 기울어지는 왜곡이 생겨 Visual SLAM의 정합 정확도를 크게 떨어뜨린다.

단안 카메라의 핵심 장단점과 실무 사양을 정리하면 다음과 같다.

**장점**:
- 싸고 가벼워 소형 플랫폼에도 달기 쉽다
- 색상과 텍스처 정보가 풍부해 의미 인식에 유리하다
- 다른 거리 센서에 비해 공간 해상도가 매우 높다

**단점**:
- 이미지 한 장만으로는 절대 깊이(depth)를 잴 수 없다
- 스케일 모호성 때문에 외부 기준(IMU, 휠 엔코더, 알려진 물체 크기 등) 없이는 실제 미터 단위 크기를 확정하기 어렵다

**주요 사양**:
- 해상도: 720p, 1080p, 4K 등
- Frame rate: 30 fps, 60 fps, 120 fps 등
- Field of View (FoV): 좁은 표준 화각부터 광각(fisheye)까지
- 셔터 방식: global shutter 또는 rolling shutter

목적과 예산에 따라 현장에서 자주 쓰이는 대표적인 카메라 군은 다음과 같다.

```
일반적인 카메라 센서:
- 웹캠: Logitech C920, C930e
- 산업용: FLIR (Point Grey), Basler, Allied Vision
- 임베디드: Raspberry Pi Camera, OAK-D
```

단안 카메라의 영상을 3차원 기하 계산에 쓰려면 렌즈 왜곡과 초점 거리를 수치로 맞추는 캘리브레이션이 선행되어야 한다. 아래 자료에서 핀홀 카메라 모델부터 캘리브레이션 실습까지 차근차근 익힐 수 있다.

> **추천 자료**
> - [First Principles of Computer Vision — Camera Calibration / Uncalibrated Stereo](https://www.youtube.com/playlist?list=PL2zRqk16wsdoCCLpou-dGo7QQNks1Ppzo) — Columbia 대학교 Shree Nayar 교수의 강의. 선형(핀홀) 카메라 모델, 내·외부 파라미터 캘리브레이션, 스테레오 기하를 설명한다.
> - [OpenCV Camera Calibration Tutorial](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html) — 카메라 캘리브레이션을 직접 해 보는 OpenCV 공식 튜토리얼

### 2.1.2 스테레오 카메라 (Stereo Camera)

사람이 두 눈으로 원근감을 느끼듯, 스테레오 카메라는 렌즈 두 개를 일정한 간격(**baseline**)으로 나란히 배치해 삼각측량으로 깊이를 구한다. 두 카메라에서 왼쪽·오른쪽 이미지를 동시에 찍고 같은 물체가 양쪽 영상에서 가로(x축)로 얼마나 어긋나 보이는지(**disparity**)를 재면 거리가 계산된다. 좌우 이미지에서 짝이 되는 점을 효율적으로 찾아내는 스테레오 매칭에는 에피폴라 기하학(Epipolar Geometry)이 쓰인다.

두 카메라가 평행하게 놓인 기본 배치에서 깊이는 다음 간단한 비례식으로 구해진다.

```
Depth (Z) = (focal_length × baseline) / disparity
```

- **Disparity**: 좌우 이미지에서 같은 점이 찍힌 x좌표의 차이
- **Baseline**: 두 카메라 광학 중심 사이의 물리적 거리

이 식이 말해주는 직관은 분명하다. 물체가 카메라에 가까울수록 좌우 영상의 위치 차이(disparity)가 커지고, 멀어질수록 disparity가 0에 가까워진다. 또한 두 카메라 사이 간격인 baseline을 넓히면 멀리 있는 물체에서도 disparity가 충분히 크게 측정되므로 더 먼 거리까지 정밀하게 잴 수 있다. 반대로 baseline이 좁으면 먼 거리의 disparity가 픽셀 오차에 묻혀 깊이 해상도가 급격히 떨어진다.

스테레오 카메라는 스스로 적외선 빛을 쏘지 않고 주변 자연광만 받아들이는 패시브(passive) 방식이라, 뒤에서 볼 Structured Light나 ToF 카메라와 달리 한낮의 강한 햇빛 아래 실외 환경에서도 깊이 정보를 안정적으로 얻는다. 다만 좌우 영상에서 "여기가 저기랑 같은 지점"이라는 짝을 맞춰야 하므로, 무늬가 전혀 없는 흰 벽이나 투명한 유리 앞에서는 매칭이 실패해 깊이 값이 비거나 튄다. 고해상도 스테레오 매칭은 연산량도 적지 않은 편이다.

**장점**:
- 조명 장치가 필요 없는 패시브 센서다
- 강한 햇빛 아래 실외에서도 깊이 측정이 가능하다
- 한 시스템에서 색상 정보와 미터 단위의 깊이를 동시에 얻는다

**단점**:
- 흰 벽이나 유리처럼 텍스처가 없는 표면에서는 스테레오 매칭이 실패한다
- 좌우 대응점을 찾는 계산량이 많다
- 유효 측정 거리와 정밀도가 baseline 크기에 묶인다

실무에서 많이 쓰이는 대표 제품들은 각자의 보완 장치를 갖추고 있다.
- Intel RealSense D435/D455: 적외선 패턴을 보조로 투사해 실내 흰 벽에서도 매칭이 잘 되도록 돕는다
- ZED 2: baseline이 넓어 비교적 먼 거리까지 깊이를 잰다
- OAK-D: 신경망 추론 전용 칩을 카메라 내부에 탑재했다

> **추천 자료**
> - [Cyrill Stachniss — Stereo Normal Case (5 Minutes with Cyrill)](https://www.youtube.com/watch?v=OpiqzVkwNz4) — 두 카메라를 나란히 놓은 스테레오(stereo normal case)의 기하를 5분 동안 설명한다
> - [Stanford CS231A — Epipolar Geometry and Stereo](https://web.stanford.edu/class/cs231a/) — Stanford의 Computer Vision 강의. 에피폴라 기하학과 스테레오를 다룬다.

수식만 읽고 넘어가기보다, 아래 인터랙티브 시각화로 baseline과 초점 거리를 직접 바꿔 보며 깊이 추정 오차가 거리에 따라 어떻게 달라지는지 확인해 보자.

> **실습**: [Stereo Disparity 시각화](https://alexjunholee.github.io/robotics-practice/app.html#stereo_disparity)
> 스테레오 이미지 쌍에서 disparity를 계산하고, baseline과 focal length를 바꾸면 깊이 추정이 어떻게 달라지는지 본다.

### 2.1.3 깊이 카메라 (RGB-D Camera)

스테레오 카메라가 무늬 없는 실내 벽면에서 약점을 드러낸다면, 아예 센서 쪽에서 빛을 쏘아 깊이를 재는 방식은 어떨까? RGB-D 카메라는 일반 컬러(RGB) 이미지와 픽셀별 깊이(Depth) 이미지를 한 쌍으로 내보내는 센서다. 별도 스테레오 캘리브레이션이나 매칭 알고리즘을 직접 짜지 않아도 곧바로 3차원 포인트 클라우드를 얻을 수 있어, 실내 책상 위 조작 실험이나 실내 SLAM 프로토타이핑에 가장 널리 쓰인다.

다만 RGB-D 카메라는 실외 햇빛 아래서 깊이 값이 뭉텅이로 비거나, 같은 공간에 두 대 이상 켜 두면 서로 측정값이 망가지는 현상을 자주 겪는다. 이런 특징은 각 제품이 깊이를 재는 세 가지 물리적 방식에서 비롯된다.

**ToF (Time of Flight) 방식**:
- 적외선 빛을 쏜 뒤 물체에 반사되어 돌아오는 비행 시간(또는 변조된 광파의 위상 차)을 재어 거리를 계산한다.
- 장점: 표면에 텍스처가 없어도 거리를 재며, 칩 내부에서 실시간으로 깊이를 뽑아낸다.
- 단점: 태양광에 섞인 적외선 간섭에 약하고, 거울처럼 반짝이는 표면에서는 빛이 여러 번 튕겨(multipath) 잘못된 거리가 찍힌다.
- 예시: Microsoft Azure Kinect, PMD Pico Flexx

**Structured Light 방식**:
- 적외선 프로젝터로 미리 약속된 점이나 격자 패턴을 공간에 투사하고, 물체 굴곡에 따라 패턴이 일그러진 정도를 분석해 깊이를 복원한다.
- 장점: 가까운 거리에서 매우 정밀한 깊이를 얻고 센서 단가가 낮다.
- 단점: 실외에서는 햇빛 때문에 투사한 패턴이 지워져 쓰기 어렵고, 두 대를 나란히 켜면 패턴이 서로 뒤섞여 간섭이 생긴다.
- 예시: Orbbec Astra

**Active IR Stereo 방식**:
- 기본 원리는 좌우 적외선 카메라 두 대를 쓰는 스테레오 삼각측량이지만, 여기에 적외선 프로젝터로 무작위 점 패턴을 뿌려 텍스처 없는 흰 벽에서도 매칭이 가능하도록 보완한 하이브리드 방식이다. 햇빛이 강한 야외에서는 IR 패턴이 묻히더라도 주변 빛만으로 일반 스테레오처럼 동작할 수 있다.
- 예시: Intel RealSense D400 시리즈

ToF 방식과 Structured Light 방식의 실무적인 차이를 나란히 비교하면 다음과 같다.

| 특성 | ToF | Structured Light |
|------|-----|------------------|
| 실외 사용 | 제한적 | 어려움 |
| 정확도 | 중간 | 높음 |
| 범위 (예시 제품) | 0.25~5.46 m (Azure Kinect, 모드에 따라 다름) | 0.6~8 m (Orbbec Astra) |
| 다중 센서 | 동기화(노출 시점 분리) 시 가능, 미동기 시 간섭 | 간섭 발생 |

연구실에서 가장 흔히 접하는 Intel RealSense 카메라를 Ubuntu 22.04 환경에서 바로 켜 보려면 아래 순서대로 공식 SDK를 설치하면 된다.

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

`realsense-viewer` 창에서 RGB와 Depth 스트림이 정상적으로 뜨는 것을 확인했다면, ROS2 토픽으로 데이터를 받아 쓰기 위해 다음 래퍼 패키지를 추가로 설치한다.

```bash
sudo apt install ros-humble-realsense2-camera
ros2 launch realsense2_camera rs_launch.py
```

(참고: [정진용 블로그](https://jinyongjeong.github.io/2020/06/20/Realsense-Ubuntu-driver-%EC%84%A4%EC%B9%98/))

RGB-D 카메라의 내부 동작 원리를 더 살펴보거나, 취득한 깊이 영상으로 3차원 실내 장면을 복원하는 실습을 해 보려면 아래 자료가 도움이 된다.

> **추천 자료**
> - [Intel RealSense — Depth Cameras D415 & D435](https://www.youtube.com/watch?v=A4Kjvosvx5I) — Intel이 D415·D435 깊이 카메라의 원리를 소개하는 영상
> - [Open3D RGB-D Reconstruction Tutorial](https://www.open3d.org/docs/release/tutorial/pipelines/rgbd_integration.html) — RGB-D 데이터로 3D 복원을 실습하는 튜토리얼

### 2.1.4 이벤트 카메라 (Event Camera)

일반 카메라는 장면이 변하든 말든 1초에 30번이나 60번씩 화면 전체 픽셀을 일괄 촬영한다. 그 탓에 아무 변화 없는 배경 데이터를 불필요하게 계속 쏟아내면서도, 정작 드론이 급회전하는 순간에는 셔터 노출 시간 동안 물체가 번지는 모션 블러에 시달린다. 어두운 터널에서 밝은 밖으로 막 빠져나오는 장면처럼 명암 차가 극심할 때도 화면이 하얗게 날아가거나 까맣게 묻힌다.

이벤트 카메라는 이 프레임 기반 촬영 패러다임을 근본적으로 뒤집은 센서다. 전체 프레임을 주기적으로 찍는 대신, 수많은 픽셀이 각자 독립적으로 밝기 변화를 감시하다가 밝기가 일정 문턱값 이상으로 바뀐 **그 순간, 그 픽셀만** 비동기적으로 신호를 내보낸다. 시간 단위가 마이크로초($\mu\text{s}$) 수준으로 촘촘하기 때문에 아주 빠른 움직임도 블러 없이 선명한 이벤트 궤적으로 남는다. 고속 비행이나 극단적 명암비(HDR) 환경을 다루어야 한다면 아래 추천 자료의 Gallego et al. 서베이 논문과 `rpg_dvs_ros` 드라이버부터 살펴보기를 권한다.

기존 영상 프레임 대신 이벤트 카메라가 내보내는 데이터는 다음 네 가지 값의 연속적인 흐름이다.

**이벤트 출력 형식**:

```
(x, y, timestamp, polarity)
- x, y: 픽셀 좌표
- timestamp: 마이크로초 단위 시간
- polarity: 밝아짐(+1) 또는 어두워짐(-1)
```

이 독특한 출력 구조는 뚜렷한 강점과 동시에 새로운 숙제를 함께 안겨 준다.

**장점**:
- 마이크로초 단위의 극도로 높은 시간 해상도를 지닌다
- 다이내믹 레인지가 140 dB에 달해 일반 카메라(60 dB)보다 훨씬 넓은 명암 폭을 담아낸다
- 변화가 있는 픽셀만 전송하므로 전력 소모가 적고 지연 시간이 짧다
- 노출 시간 동안 프레임을 쌓지 않아 모션 블러가 생기지 않는다

**단점**:
- 카메라와 주변 장면이 모두 정지해 있고 조명까지 일정하면 출력 이벤트 자체가 전혀 나오지 않는다
- 2차원 격자 이미지가 아닌 비동기 이벤트 스트림이므로 기존 프레임 기반 컴퓨터 비전 알고리즘을 그대로 적용하기 어렵다
- 아직 생산 규모가 작아 일반 카메라에 비해 가격이 비싼 편이다

대표적인 이벤트 카메라 제조사와 제품군은 다음과 같다.
- Prophesee: 고해상도 이벤트 센서
- iniVation: 이벤트 스트림과 일반 프레임을 한 센서에서 동시에 내보내는 DAVIS
- Samsung: DVS(Dynamic Vision Sensor)

> **추천 자료**
> - [Davide Scaramuzza — Event Cameras: a New Way of Sensing (ICCP 2024 Keynote)](https://www.youtube.com/watch?v=0wGBpgIrd9M) — UZH의 Davide Scaramuzza 교수가 ICCP 2024 기조강연에서 이벤트 카메라를 개괄한다
> - [Gallego et al. — Event-based Vision: A Survey (TPAMI 2022)](https://arxiv.org/abs/1904.08405) — 이벤트 카메라의 원리, 알고리즘, 응용을 정리한 서베이 논문.
> - [rpg_dvs_ros — Event Camera ROS 드라이버](https://github.com/uzh-rpg/rpg_dvs_ros) — UZH RPG가 만든, 이벤트 카메라를 ROS에서 쓰는 오픈소스 드라이버

## 2.2 LiDAR

카메라는 색상과 질감 표현에서는 타의 추종을 불허하지만, 수십 미터 밖 물체까지의 정확한 미터 단위 거리를 캄캄한 밤이나 그림자 속에서까지 일관되게 뽑아내기는 어렵다. **LiDAR**(Light Detection and Ranging)는 반대로 색상 정보는 포기하는 대신, 레이저 펄스를 직접 쏘고 반사되어 돌아온 빛(return)의 왕복 시간을 재어 정밀한 3차원 점 하나를 찍는다.

3D LiDAR는 이런 레이저 점 수만~수백만 개를 모은 포인트 클라우드를 매 초 내보낸다. 다만 측정 가능 거리와 오차는 고정된 상수가 아니며, 센서 모델은 물론 대상 표면의 반사율, 레이저가 부딪히는 입사각, 대기 상태, 강한 태양광, 다중 반사 처리 방식(return mode)에 따라 크게 달라진다. 자동차용 장거리 LiDAR 가운데 일부는 지정 반사율 조건에서 100 m 이상을 안정적으로 측정한다. 레이저 파장이 짧아 전파를 쓰는 radar보다 각분해능이 월등히 높고, 적외선 LED나 패턴을 쓰는 ToF·structured-light 방식 RGB-D 카메라보다 훨씬 먼 거리까지 정확하게 거리를 잰다는 점이 LiDAR의 독보적 강점이다.

### 2.2.1 2D LiDAR와 3D LiDAR

LiDAR는 레이저가 공간을 훑는 차원에 따라 2D와 3D로 나뉜다. 실내 바닥처럼 평평한 환경에서 장애물만 피할 것인지, 아니면 야외 지형과 건물 전체를 3차원으로 재구성할 것인지에 따라 선택이 갈린다.

**2D LiDAR**:
- 수평 한 평면(single scan line)만 360° 또는 부채꼴로 훑는다
- 용도: 실내 이동 로봇 내비게이션, 2D 격자 지도 작성, 근거리 장애물 회피
- 예시: SICK TiM, Hokuyo URG, RPLIDAR

**3D LiDAR**:
- 수직 방향으로 여러 층의 레이저를 배치해 회전시키거나 특정 스캔 패턴으로 훑어 3D 포인트 클라우드를 만든다
- 용도: 자율주행차 인지, 대규모 3D 매핑, 야외 지형 주행
- 예시: Velodyne VLP-16·VLP-32C·HDL-64E, Ouster OS1, Hesai

LiDAR 데이터를 기반으로 지도를 만들고 위치를 추정하는 알고리즘과 오픈소스 라이브러리는 아래 자료에서 출발할 수 있다.

> **추천 자료**
> - [Cyrill Stachniss — LiDAR-based SLAM](https://www.youtube.com/watch?v=vrdlk2p9AZI) — ITSC 2020 SLAM 워크숍 기조강연. 기하와 시맨틱 정보를 함께 쓰는 LiDAR SLAM을 소개한다
> - [PCL (Point Cloud Library) 공식 튜토리얼](https://pcl.readthedocs.io/projects/tutorials/en/latest/) — 널리 쓰이는 공개 포인트 클라우드 처리 라이브러리

### 2.2.2 Spinning LiDAR와 Solid-State LiDAR

같은 3D LiDAR라도 레이저 빔을 공간에 뿌리는 하드웨어 메커니즘에 따라 성격과 내구성이 갈린다.

**Spinning (기계식)**:
- 모터로 레이저 송신부와 수광부 어셈블리 전체를 물리적으로 회전시킨다
- 수평 방향 360° 전방위를 사각지대 없이 균일하게 본다
- 단점: 계속 돌아가는 기계적 부품이 있어 진동·충격에 따른 마모와 크기·가격 부담이 따른다
- 예시: Velodyne, Ouster

**Solid-State**:
- 센서 뭉치 전체를 돌리는 대형 회전 기구 없이 빔을 조향한다 (flash 방식은 가동부가 전혀 없고, MEMS 방식은 초소형 거울만 진동하며, Livox의 비반복 스캔은 내부 회전 프리즘 두 개를 조합해 쓴다)
- 대부분 정면을 향한 좁은 수평 화각을 가진다 (보통 120° 이하)
- 장점: 기계적 고장이 덜하고, 소형화하고 대량 생산하면 가격을 낮출 여지가 크다
- 예시: Livox (비반복 스캔 패턴), Innoviz

이 하드웨어 구조 차이는 SLAM 소프트웨어 선택에도 곧바로 영향을 미친다. Spinning LiDAR는 360° 방향으로 일정한 고도각을 가진 고른 스캔 라인(ring)을 만들어 내며, LOAM이나 LeGO-LOAM 같은 고전 LiDAR SLAM 알고리즘은 바로 이 스캔 라인 구조를 가정하고 에지·평면 특징점을 뽑는다. 따라서 꽃잎 모양의 비반복 스캔 패턴을 그리는 Solid-State LiDAR(예: Livox)에 이런 코드를 그대로 넣으면 특징점 추출부터 어긋나므로 코드를 수정해야 한다. 반면 FAST-LIO2는 별도의 특징점 추출 단계를 건너뛰고 원시 포인트(raw point)를 지도에 직접 정합하기 때문에, 회전식 LiDAR와 Livox 같은 비반복 스캔 LiDAR를 모두 수정 없이 다룬다.

### 2.2.3 주요 사양

LiDAR 데이터시트를 읽을 때는 다음 다섯 가지 수치를 확인하면 점군의 조밀도와 유효 거리를 가늠할 수 있다.

| 사양 | 설명 |
| --- | --- |
| Channels | 수직 레이어 수 (16, 32, 64, 128) |
| Range | 최대 측정 거리 (50 m~300 m) |
| Points/sec | 초당 포인트 수 (30만~수백만, 128채널급은 200만 이상) |
| Accuracy | 거리 정확도 (±2 cm~±5 cm) |
| FoV | 수평/수직 화각 |

가령 채널 수가 많을수록 수직 해상도가 촘촘해져 멀리 떨어진 작은 장애물도 여러 개의 점으로 잡히지만, 그만큼 포인트 클라우드 처리 연산량도 함께 커진다. 비반복 스캔 방식의 특성과 이를 매끄럽게 처리하는 최신 LiDAR-Inertial Odometry 구조는 아래 자료에서 자세히 확인할 수 있다.

> **추천 자료**
> - [Livox 기술 문서](https://www.livoxtech.com/downloads) — Livox 센서의 사양서와 비반복 스캔 패턴 설명 자료
> - [Xu et al. — FAST-LIO2 (T-RO 2022)](https://arxiv.org/abs/2107.06829) — 특징을 뽑지 않고 raw point를 바로 정합하며, 증분 k-d tree(ikd-Tree)로 지도를 갱신해 spinning·solid-state LiDAR를 모두 다루는 LiDAR-Inertial Odometry 논문

## 2.3 IMU (Inertial Measurement Unit)

카메라나 LiDAR는 보통 초당 10~60번(10~60 Hz) 장면을 찍는다. 로봇이 천천히 이동할 때는 충분해 보이지만, 울퉁불퉁한 바닥을 지나거나 드론이 순간적으로 급회전할 때는 한 프레임과 다음 프레임 사이의 틈이 너무 넓어 추적 알고리즘이 길을 잃기 쉽다. **IMU**는 바로 이 틈을 메우는 센서다. 수백에서 수천 Hz에 달하는 빠른 주기로 로봇의 가속도와 각속도를 쉼 없이 측정해, Visual-Inertial Odometry(VIO)나 LiDAR-Inertial Odometry(LIO)에서 프레임 사이의 움직임을 촘촘히 이어 준다.

하지만 IMU는 외부 풍경을 보는 센서가 아니라 자기 몸이 느끼는 관성만 재기 때문에, 오차가 조금씩 쌓이는 **드리프트(drift)**를 피할 수 없다. SLAM 궤적이 시간에 따라 휘거나 발산할 때 IMU bias가 제대로 추정·보정되고 있는지, 혹은 지금 쓰는 IMU 등급에서 기대할 수 있는 한계가 어디까지인지 따지려면 아래의 내부 구성과 오차 모델을 정확히 이해해야 한다.

### 2.3.1 구성 요소

일반적으로 말하는 6축 IMU는 가속도계와 자이로스코프를, 9축 IMU는 여기에 지자기 센서까지 품고 있다.

**Accelerometer (가속도계)**:
- 3축 선형 가속도를 잰다 ($\text{m/s}^2$)
- 주의할 점은 **측정값에 항상 중력 가속도가 섞여 나온다는 것**이다. 가속도계가 재는 물리량은 순수한 좌표 가속도가 아니라 중력을 포함한 비력(specific force)이므로, 책상 위에 가만히 놓아두어도 위쪽 방향으로 약 $9.81\ \text{m/s}^2$가 찍힌다. 따라서 이동 가속도를 얻으려면 현재 자세를 추정해 중력 벡터를 정확히 빼 주어야 한다.

**Gyroscope (자이로스코프)**:
- 3축 각속도를 잰다 ($\text{rad/s}$ 또는 $\text{deg/s}$)
- 짧은 시간 동안 로봇이 어느 축으로 얼마나 빠르게 돌았는지 매우 민감하게 포착한다.

**Magnetometer (지자기 센서)** (일부 9축 IMU에 포함):
- 3축 지구 자기장을 잰다
- 이 값으로 나침반처럼 절대 방위(heading)를 추정해 자이로스코프의 요(yaw) 방향 드리프트를 잡아주는 데 쓸 수 있다
- 다만 실내 건물의 철골 구조물이나 모터·배터리에 흐르는 전류가 만드는 주변 자기장 왜곡에 극도로 취약해, 실내 로봇이나 모터 근처에서는 오히려 방위 추정을 망가뜨리기도 한다

### 2.3.2 주요 오차 특성

센서 융합 필터나 최적화 기반 SLAM은 IMU의 원시 측정값을 그대로 믿지 않고, 아래 세 가지 오차 특성을 상태 변수와 잡음 모델로 수식에 함께 태워 실시간으로 보정한다.

**Bias (바이어스)**:
- 센서를 완전히 정지시켜 놓아도 출력이 정확히 0으로 떨어지지 않고 일정한 오프셋(offset)이 남는 현상이다.
- 더 까다로운 점은 이 바이어스가 고정값이 아니라 시간이 흐르면서 천천히 흔들린다는 것이다. 이렇게 서서히 변하는 정도를 **bias instability**라 부르며, 정지 상태에서 오랫동안 데이터를 모아 그린 Allan deviation 곡선의 가장 낮은 골짜기 값으로 규격을 정한다. 센서 온도에 따라 바이어스가 크게 달라지는 thermal drift는 별도로 보정 곡선을 구해 다룬다.

**Noise (백색 잡음)**:
- 매 샘플마다 무작위로 얹히는 고주파 랜덤 노이즈다.
- 이 랜덤 노이즈의 밀도(noise density)와 random walk 계수는 **Allan Variance** 분석으로 크기를 정량화한다.

**Integration Drift (적분 드리프트)**:

자세 변화를 얻으려고 각속도를 시간에 대해 한 번 적분하면 자이로스코프의 노이즈와 바이어스 오차가 고스란히 자세 오차로 누적된다. 더 나아가 위치를 구하려고 가속도를 시간에 대해 **두 번 적분**하면, 가속도계의 노이즈·바이어스·스케일 팩터(scale factor) 오차는 물론 자세 추정 오차 때문에 중력 성분이 잘못 빠져나간 잔여분까지 제곱으로 증폭되어 쌓인다. 오차가 불어나는 속도는 센서 등급, 기동 패턴, 온도 변화, 초기 정렬 정확도에 따라 다르지만, 저렴한 MEMS IMU 단독으로 위치를 적분하면 불과 몇 초~수십 초 만에 위치 오차가 크게 벌어진다. 그래서 장시간 정확한 위치가 필요한 로봇은 반드시 카메라, LiDAR, GNSS 같은 외부 관측 센서와 결합해 IMU의 드리프트를 주기적으로 눌러 주어야 한다.

시중에 나오는 IMU는 bias stability, noise density, scale-factor error, 온도 보정 여부, 진동 내성, 항공·방산 인증 요구 수준에 따라 등급이 나뉜다. 스마트폰이나 드론에 들어가는 소비자용(consumer) MEMS는 크기와 전력 소모를 줄이는 데 초점을 맞추고, industrial, tactical, navigation 등급으로 올라갈수록 훨씬 오랜 시간 안정적이며 넓은 온도 범위에서 보정이 유지된다. `VN-100`, `MTi`, `KVH 1750`, `HG1700` 같은 구체적 모델을 비교할 때는 마케팅용 등급 이름만 보지 말고, 반드시 동일한 단위로 환산한 최신 데이터시트 수치와 실제 Allan variance 측정 결과를 나란히 놓고 확인해야 한다.

IMU의 물리 원리와 오차 모델링, 그리고 실제로 자세를 추정하는 필터 구현은 아래 자료에서 체계적으로 배울 수 있다.

> **추천 자료**
> - [Probabilistic Robotics, Ch.5–6 — Robot Motion / Robot Perception (Thrun, Burgard, Fox)](http://probabilistic-robotics.org/) — 모션 모델(5장)과 센서 측정 모델(6장)을 다룬다. IMU 오차 모델(bias, random walk, Allan variance)은 이 책에 없으니 Titterton & Weston을 본다.
> - [Titterton & Weston — Strapdown Inertial Navigation Technology](https://doi.org/10.1049/PBRA017E) — IMU 원리와 strapdown 관성 항법을 다루는 교과서
> - [Allan Variance — IMU 노이즈 분석 가이드 (Vectornav)](https://www.vectornav.com/resources/inertial-navigation-primer/specifications--and--error-budgets/specs-imucal) — Allan Variance로 IMU 노이즈 파라미터를 구하는 방법을 설명한다
> - [정진용 블로그 — IMU Filter (AHRS)](https://jinyongjeong.github.io/2020/01/10/IMU_filter/) — IMU로 자세를 추정하는 AHRS(Attitude and Heading Reference System) 필터를 설명하고, Madgwick 필터와 ROS 패키지를 소개한다

---

## 2.4 GNSS / GPS

카메라, LiDAR, IMU로 동작하는 VIO·LIO는 출발점을 원점으로 삼아 상대적인 이동량을 이어 붙이기 때문에, 수 킬로미터를 주행하다 보면 결국 조금씩 누적된 전역 위치 오차가 남는다. 실외를 달리는 자율주행차나 야외 드론은 **GNSS**(Global Navigation Satellite System)가 주는 지구 좌표계 기준 위도·경도·고도를 상대 위치와 결합해 장기 드리프트를 잡는다. 또한 학술 데이터셋을 구축할 때는 고정밀 **RTK-GPS** 측정치를 로봇 궤적의 정답(ground truth)으로 널리 활용한다.

하늘이 탁 트인 개활지에서 별도 보정 없이 일반 수신기 한 대만으로 위치를 계산하면(standalone) 오차가 대략 미터 수준에 머문다. 반면 정확한 좌표를 이미 알고 있는 지상의 기준국으로부터 보정 신호를 받는 차분 보정(differential correction)을 적용하면, 환경에 따라 1 m 아래에서 수 센티미터 수준까지 정밀도가 올라간다. 하지만 고층 빌딩이 빽빽한 도심 협곡(urban canyon)에서는 위성 신호가 건물 외벽에 튕겨 지연 도달하는 **다중 경로 오차(multipath)**가 심해진다. 이 상태에서는 정밀한 RTK 수신기조차 위성 전파 반송파의 정수 모호성(integer ambiguity)을 풀지 못해 고정밀 상태가 풀리며 오차가 다시 수 미터로 커질 수 있다. 도심 자율주행 시스템이 GNSS만 믿지 않고 LiDAR·카메라 기반 SLAM을 반드시 결합하는 이유가 여기에 있다. 아울러 논문이나 보고서에서 GNSS 정확도를 인용할 때는 단일 숫자만 적지 말고, 측정 지표(CEP·RMS·95%), 수평/수직 구분, 기준국과의 거리(baseline), 보정 신호 전달 경로, 그리고 RTK의 `fix` 상태 여부를 함께 밝혀야 공정한 비교가 된다.

**RTK-GPS 원리**:
- 측량이 완료된 정확한 위치의 기준국(base station)이 위성 신호 오차에 대한 보정 데이터를 실시간으로 계산해 보낸다
- 이동 중인 로봇의 수신기(rover)가 이 보정 데이터를 받아 자기 위치 계산에서 오차를 상쇄한다
- 기준국과 수신기 사이에 무선 모뎀이나 인터넷(NTRIP 등)을 통한 실시간 통신 연결이 유지되어야 한다

**한계**:
- 위성 전파가 닿지 않는 실내나 터널에서는 사용할 수 없으며, 고층 건물이나 울창한 숲 아래에서는 수신 품질이 급격히 떨어진다
- 건물이나 지면에 반사되어 늦게 도착한 신호(multipath)가 위치 오차를 크게 키운다
- 위성들이 하늘 위쪽 반구에만 분포하는 기하 구조 탓에, 수직 고도 정확도는 수평(위도·경도) 정확도보다 늘 떨어진다

전역 위치 추정 개념과 코드 기반·반송파 기반 GNSS의 차이는 아래 자료에서 더 깊이 확인할 수 있다.

> **추천 자료**
> - [Cyrill Stachniss — Robot Localization Overview](https://www.youtube.com/watch?v=8VJ-A9OlhAE) — 로봇 위치 추정 방법을 개괄하는 강의
> - [u-blox — RTK와 GNSS 보정](https://www.u-blox.com/en/technologies/rtk-real-time-kinematic) — 의사거리(code)로 위치를 구하는 방식과 반송파 위상(carrier phase)을 쓰는 RTK가 어떻게 다른지, 보정 서비스는 어떻게 쓰는지 설명하는 u-blox 자료

## 2.5 기타 센서

지금까지 본 주요 센서 외에도, 악천후를 돌파하거나 바퀴의 회전량·초근접 장애물을 저렴하게 재기 위해 현장에서 빠지지 않고 쓰이는 보조·특수 센서들이 있다.

**Radar**

가시광선을 쓰는 카메라나 적외선 레이저를 쓰는 LiDAR는 짙은 안개, 폭우, 먼지, 눈보라 앞에서 빛이 산란되어 시야가 급격히 짧아진다. **Radar**는 파장이 긴 밀리미터파 전파를 쓰기 때문에 이런 악천후와 역광의 영향을 훨씬 덜 받으며, 자율주행차와 야외 로봇에서 카메라·LiDAR의 든든한 백업 역할을 한다. 물론 전파 특성상 각분해능(angular resolution)이 LiDAR보다 낮아 세밀한 물체 형태를 구분하기는 어렵고, 아주 거센 폭우에서는 신호 감쇠가 일어나며, 금속 가드레일 등에 의한 불필요한 반사(clutter)와 다중 반사(multipath)가 섞이거나 센서 덮개(radome)에 물기가 고이면 성능이 떨어진다는 점은 감안해야 한다. 가격은 안테나 채널 수, 대역폭, 이미징 기능 지원 여부, 자동차 등급 인증에 따라 편차가 크므로 도입 시점의 견적으로 비교한다.

로보틱스와 자율주행에서 주로 쓰는 방식은 **FMCW (Frequency Modulated Continuous Wave) Radar**다.
- 시간에 따라 주파수를 선형으로 바꾸며 전파를 내보낸 뒤, 물체에 맞고 돌아온 반사파와 방금 보낸 신호의 주파수 차이를 분석해 **거리와 도플러(Doppler) 상대 속도를 한 번에 측정**한다.
- 출력 데이터: Range-Doppler map(거리 × 속도 2D 맵), Range-Azimuth map(거리 × 방위각 2D 맵)
- 자동차 분야에서는 77 GHz 대역 radar가 사실상 표준으로 가장 널리 쓰인다.

실제 로봇 시스템에서 Radar는 단순 충돌 경보를 넘어 위치 추정과 지도 작성까지 영역을 넓히고 있다.
- 자율주행: 전방 충돌 감지, 적응형 크루즈 컨트롤 (ACC)
- Radar odometry: 악천후에서도 레이더 스캔 매칭만으로 자기 위치 변화를 추정한다
- Radar SLAM: 레이더 반사 지도를 만들며 동시에 위치를 추정한다

대표 제품: Texas Instruments AWR1843, Continental ARS548, Navtech CTS350-X (spinning radar)

카메라, LiDAR, Radar 세 핵심 인지 센서의 성격을 한눈에 견주어 보면, 왜 실제 자율주행차가 어느 하나만 고집하지 않고 세 센서를 섞어 쓰는지 분명해진다.

| 특성 | 카메라 | LiDAR | Radar |
|------|--------|-------|-------|
| 해상도 | 매우 높음 | 높음 | 낮음 |
| 거리 측정 | 불가 (단안) | 정확 | 가능 |
| 속도 측정 | 불가 | 불가 (ToF 방식 기준, FMCW LiDAR는 Doppler로 가능) | 가능 (Doppler) |
| 악천후 | 취약 | 취약 (비, 안개) | 강건 |
| 가격 | 저렴 | 비쌈 | 중간 |
| 야간 | 취약 (조명 필요) | 가능 | 가능 |

**Ultrasonic (초음파)**

광학 센서나 라이다가 투명한 유리문이나 검은 광택 표면을 놓치기 쉬운 반면, 음파를 이용하는 초음파 센서는 재질과 색상에 거의 영향을 받지 않는다.
- 음파 반사 시간으로 가까운 장애물을 감지한다 (대략 0.2~5 m)
- 단가가 매우 저렴해 로봇 둘레에 여러 개를 촘촘히 붙일 수 있다
- 주차 보조 센서나 모바일 로봇의 최후방 충돌 방지용 근접 감지에 쓴다

**Wheel Encoder (휠 엔코더)**

바퀴가 달린 지상 로봇이라면 모터 축에 달린 휠 엔코더가 가장 기본이 되는 고유 수용 센서다.
- 바퀴가 몇 바퀴, 몇 도 회전했는지 펄스를 세어 잰다
- 바퀴 반지름과 차축 간격을 곱해 이동 거리와 회전각을 누적하며 현재 위치를 추정한다(dead reckoning)
- 다만 젖은 바닥이나 경사로, 급가속 구간에서 바퀴가 헛돌거나 미끄러지면(wheel slip) 실제 로봇은 움직이지 않았는데 엔코더 값만 올라가 치명적인 누적 오차가 생긴다

거리 센서 모델링과 최신 Radar 오도메트리·멀티모달 데이터셋 연구 흐름은 아래 자료에서 이어서 살펴본다.

> **추천 자료**
> - [Probabilistic Robotics, Ch.6 — Robot Perception (Thrun, Burgard, Fox)](http://probabilistic-robotics.org/) — 거리 센서와 특징 기반 센서의 확률 모델을 다룬다.
> - [김기섭 블로그 — ICRA 2021 Radar in Robotics Workshop 요약](https://gisbi-kim.github.io/blog/2021/05/31/icra21-radar-ws.html) — ICRA 2021 Radar in Robotics 워크숍 발표를 정리한 글
> - [김기섭 블로그 — Radar Odometry Results on MulRan dataset](https://gisbi-kim.github.io/blog/2021/05/30/yeti-radar-odom-mulran1.html) — Yeti radar odometry(Cen2018·Cen2019 방식)를 MulRan 도시 시퀀스에 돌린 궤적 결과
> - [Kim et al., "MulRan: Multimodal Range Dataset for Urban Place Recognition" (ICRA 2020)](https://sites.google.com/view/mulran-pr/home) — LiDAR + radar + GPS 멀티모달 데이터셋

## 2.6 센서 융합

지금까지 훑어본 것처럼 만능 센서는 존재하지 않는다. 카메라는 어두운 밤이나 흰 벽 앞에서 무력해지고, LiDAR는 폭우나 안개 속에서 시야가 흐려지며, IMU는 몇 초만 지나도 적분 오차가 쌓이고, GNSS는 건물 숲이나 터널에 들어서면 신호를 잃는다. **센서 융합(sensor fusion)**은 바로 이 지점에서 출발한다. 서로 약점이 다른 센서들을 짝지어 한 센서가 눈을 감는 순간 다른 센서가 공백을 메우게 만드는 전략이다. 실제 자율주행차가 운행 환경에 맞춰 카메라·LiDAR·Radar·IMU·GNSS 가운데 여러 조합을 동시에 탑재하는 이유도 여기에 있다.

각 센서가 서로에게 어떤 빈틈을 채워 주는지 장단점을 나란히 놓아 보자.

| 센서 | 장점 | 단점 |
| --- | --- | --- |
| Camera | 풍부한 정보, 저렴 | 조명 의존, 깊이 없음 |
| LiDAR | 정확한 3D, 조명 무관 | 비쌈, sparse |
| IMU | 고주파, 조명 무관 | 드리프트 |
| GPS | 전역 위치 | 실외 전용, 저주파 |

여러 센서 데이터를 파이프라인의 어느 단계에서 섞느냐에 따라 융합 구조는 세 가지 층위로 나뉜다.
- **Early Fusion**: 가공하지 않은 원시(raw) 센서 데이터를 입구 단계에서 곧바로 합친다. 정보 손실 없이 세밀한 상관관계를 활용할 수 있지만, 센서 간 정확한 시공간 동기화와 큰 연산량이 요구된다.
- **Mid-Level Fusion**: 각 센서에서 의미 있는 특징(feature)을 먼저 뽑은 뒤 중간 표현 단계에서 결합한다. 최근 딥러닝 기반 멀티모달 인지 모델에서 가장 널리 쓰이는 방식이다.
- **Late Fusion**: 센서마다 독립적인 파이프라인으로 객체 검출이나 위치 추정 결과를 따로 뽑은 다음, 마지막 의사결정 단계에서 결과들을 합친다. 모듈을 깔끔하게 분리해 디버깅하기 좋지만, 중간 요약 과정에서 원래 센서 신호가 품고 있던 풍부한 정보가 적지 않게 버려진다.

로보틱스 현장에서 이미 검증되어 표준처럼 자리 잡은 대표적인 센서 융합 조합은 다음과 같다.
- Camera + IMU → **VIO** (Visual-Inertial Odometry): 카메라의 정밀한 시점 변화 추정과 IMU의 고주파 회전·스케일 정보를 결합한다
- LiDAR + IMU → **LIO** (LiDAR-Inertial Odometry): 빠른 움직임 중 LiDAR 점이 찌그러지는 현상(motion distortion)을 IMU로 펴 주고 스캔 정합의 초기값을 제공한다
- Camera + LiDAR + IMU → **멀티모달 SLAM**: 텍스처가 없는 터널이나 역광 야외처럼 어느 한 센서만으로는 버티기 어려운 극한 환경에서도 안정성을 유지한다

이렇듯 성격과 단위, 주기가 모두 다른 센서들을 수학적으로 한 식 안에서 결합하려면, 각 센서 측정값이 얼마나 믿을 만한지를 일관된 확률 언어로 적어 내야 한다. 바로 그 확률 측정 모델이 이어지는 §2.7의 주제다.

> **추천 자료**
> - [State Estimation for Robotics (Tim Barfoot) — 무료 PDF](https://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) — 로봇 상태 추정 교재. Kalman Filter 계열과 batch 최적화 기반 추정, 3차원 회전·자세 표현을 다룬다. 저자가 PDF를 무료로 공개했다.
> - [Cyrill Stachniss — Kalman Filter & EKF](https://www.youtube.com/watch?v=E-6paM_Iwfc) — 칼만 필터와 EKF를 설명하는 강의
> - [Qin et al. — VINS-Mono (TRO 2018)](https://arxiv.org/abs/1708.03852) — 단안 카메라와 IMU를 결합한 VIO 시스템 논문. 초기화, 최적화 기반 추정, loop closure까지 실제 시스템의 구성을 보여 준다.

---

## 2.7 확률 측정 모델 [심화]

실제 로봇에서 센서 값을 곧이곧대로 정답으로 받아들이면 필터가 금세 망가진다. 레이저가 지나가는 사람의 다리에 맞아 예상보다 짧게 찍히기도 하고, 검은 유리창에 흡수되어 아예 측정값이 돌아오지 않기도 하기 때문이다. 그래서 베이즈 필터, SLAM, MCL(Monte Carlo Localization, 입자 필터로 위치를 추정하는 기법)은 센서 값을 절대 사실로 취급하는 대신, **측정 모델 $p(z_t \mid x_t, m)$**로 "로봇이 지도 $m$ 위의 특정 포즈 $x_t$에 정말로 서 있다면, 지금 들어온 측정값 $z_t$가 관측될 가능성이 얼마나 되는가?"를 확률로 묻는다.

### 2.7.1 센서가 만드는 분포

로봇을 완전히 멈춰 세워 두고 같은 벽을 향해 레이저 거리 센서를 100번 쏘면, 100개의 값이 소수점 아래까지 똑같이 나올까? 결코 그렇지 않다. 평평한 벽면이라도 미세한 전자 잡음 때문에 값이 흔들리고, 빔이 벽면에 비스듬히 스치면 엉뚱한 방향으로 튕겨 나가며, 중간에 사람이 지나가거나 다중 반사가 일어나기도 한다. 게다가 이렇게 오차가 생기는 원인마다 측정값이 흩어지는 모양새가 완전히 다르다. 확률 측정 모델 $p(z_t^k \mid x_t, m)$은 이 물리적 불확실성을 하나의 확률 분포로 요약한 것이다. 여기서 $z_t^k$는 시각 $t$에 들어온 스캔의 $k$번째 빔 측정값, $x_t$는 로봇 포즈, $m$은 환경 지도를 뜻한다.

실제 스캔 한 장에는 보통 수십에서 수백 개의 빔이 묶여 들어온다. *Probabilistic Robotics*(PR) §6.1(식 6.2)에서는 계산을 다루기 쉽게 만들기 위해 **로봇 포즈 $x_t$와 지도 $m$이 주어졌을 때 각 빔의 측정 오차는 서로 독립**이라는 조건부 독립(conditional independence) 가정을 둔다. 이 가정 덕분에 전체 스캔의 likelihood는 개별 빔 likelihood의 단순 곱으로 깔끔하게 쪼개진다.

$$p(z_t \mid x_t, m) = \prod_{k=1}^{K} p(z_t^k \mid x_t, m)$$

물론 현실에서는 같은 벽면을 나란히 때리는 인접 빔들의 오차가 서로 강하게 얽혀 있기 마련이다. 이 상관을 무시한 채 수백 개 빔의 확률을 그대로 곱하면, 필터가 단 한 번의 스캔만으로 특정 포즈에 지나친 확신(over-confidence)을 품는 부작용이 생긴다. 이 문제를 실무에서 누그러뜨리는 요령은 §2.7.4와 §2.7.8에서 다룬다.

한편, 측정 모델은 환경 지도 $m$을 어떻게 표현하느냐에 따라 크게 두 갈래와 맞물린다. **feature-based 지도**는 코너나 나무 기둥 같은 랜드마크 좌표의 목록으로 지도를 저장하며, 각 랜드마크를 고유 ID로 찾아간다. 반면 **location-based 지도**는 공간을 일정 크기의 격자(grid) 셀로 쪼개고 셀마다 장애물 점유 확률을 채운 배열로, 특정 $(x, y)$ 좌표로 곧바로 조회한다.

이 지도 표현 위에서 로봇 공학이 정립한 대표적 측정 모델 네 가지는 다음과 같다.
- **빔 모델(beam model)**: 빔 하나에 측정값이 찍히는 네 가지 물리적 원인을 각각 분포로 만들고 가중 혼합한 모델. location-based 지도를 쓴다.
- **likelihood field**: 광선 경로를 추적하는 대신, 빔 끝점이 떨어진 위치에서 가장 가까운 지도상 장애물까지의 거리로 점수를 매기는 모델. location-based 지도를 쓴다.
- **상관 기반(map matching)**: 최근 스캔으로 만든 작은 지역 지도(local map)와 전체 지도(global map)를 겹쳐 정규화 상관계수로 유사도를 재는 모델. 격자 지도를 쓴다.
- **특징 기반(landmark model)**: 센서 데이터에서 추출한 추상 특징을 (거리, 방위각, 식별자) 묶음으로 모델링하는 방식. feature-based 지도를 쓴다.

### 2.7.2 빔 모델 — 4성분 혼합

**빔 모델(beam model)**은 거리 센서의 물리적 현상을 가장 정직하게 수식으로 옮긴 모델이다. [Thrun et al. 2005](http://probabilistic-robotics.org/) (PR §6.3.1)는 레이저나 초음파 빔 하나가 내놓는 측정값을 **정상 충돌(hit), 예상치 못한 앞선 장애물(short), 최대 거리 이탈(max), 원인 불명 난수(rand)** 네 가지 시나리오로 나누고, 각 시나리오의 확률 분포를 가중치로 섞어 전체 likelihood를 구성한다.

첫째, **hit**은 빔이 지도 $m$에 그려진 실제 장애물에 정상적으로 부딪혀 돌아온 이상적인 경우다. 로봇 포즈 $x_t$에서 빔이 지도의 장애물까지 날아갈 것으로 계산되는 예측 거리를 $z_t^{k*}$라 할 때, 실제 측정값 $z_t^k$는 $z_t^{k*}$를 평균으로 하고 센서 정밀도에 해당하는 분산 $\sigma_{\text{hit}}^2$을 갖는 가우시안 분포를 따른다. 다만 거리 센서는 음수나 최대 측정 거리 $z_{\max}$ 바깥의 값을 낼 수 없으므로 분포를 $[0, z_{\max}]$ 구간으로 잘라 정규화한다.

$$p_{\text{hit}}(z_t^k \mid x_t, m) = \eta\, \mathcal{N}(z_t^k;\, z_t^{k*},\, \sigma_{\text{hit}}^2), \quad 0 \le z_t^k \le z_{\max}$$

여기서 $\eta$는 잘라 낸 $[0, z_{\max}]$ 범위의 적분값이 정확히 1이 되도록 맞춰 주는 정규화 상수다. 아래 $p_{\text{short}}$ 식의 $\eta$ 역시 주어진 구간에서 확률의 총합을 1로 맞추는 같은 역할을 한다.

둘째, **short**는 지도에 표시되지 않은 동적 장애물(지나가는 사람이나 다른 로봇)이 빔을 중간에 가로막은 경우다. 이 경우 측정값은 반드시 예측 거리 $z_t^{k*}$보다 짧게 찍히며, 센서 근처에서 물체를 만날 확률이 더 높다는 경험적 경향을 반영해 $[0, z_t^{k*}]$ 구간에서 감소하는 지수 분포로 모델링한다.

$$p_{\text{short}}(z_t^k \mid x_t, m) = \eta\, \lambda_{\text{short}}\, e^{-\lambda_{\text{short}} z_t^k}, \quad 0 \le z_t^k \le z_t^{k*}$$

셋째, **max**는 빛을 흡수하는 검은 표면, 빔이 미끄러져 딴 데로 튕겨 나가는 유리·광택 면, 혹은 안개 속 산란 때문에 반사파가 아예 돌아오지 않은 실패 사례다. 이런 경우 대다수 센서는 "아무것도 못 잡았다"는 신호로 최대 사거리 $z_{\max}$를 그대로 출력하므로, $z_{\max}$ 한 지점에 확률이 전부 몰린 점질량(point mass) 분포로 표현한다.

$$p_{\text{max}}(z_t^k \mid x_t, m) = \mathbf{1}[z_t^k = z_{\max}]$$

넷째, **rand**는 초음파 센서 간의 혼선(crosstalk)이나 복잡한 다중 반사처럼 앞선 세 가지 이유로도 설명하기 어려운 예측 불가 노이즈다. 어떤 거리든 무작위로 찍힐 수 있다고 보고 $[0, z_{\max}]$ 전 구간에 걸친 균등 분포(uniform distribution)를 둔다.

$$p_{\text{rand}}(z_t^k \mid x_t, m) = \frac{1}{z_{\max}}$$

최종적으로 빔 하나의 likelihood는 이 네 가지 확률 분포에 각각의 발생 비중을 뜻하는 가중치를 곱해 더한 혼합 모델이 된다(PR 식 6.13).

$$p(z_t^k \mid x_t, m) = \begin{pmatrix} z_{\text{hit}} \\ z_{\text{short}} \\ z_{\text{max}} \\ z_{\text{rand}} \end{pmatrix}^T \cdot \begin{pmatrix} p_{\text{hit}}(z_t^k \mid x_t, m) \\ p_{\text{short}}(z_t^k \mid x_t, m) \\ p_{\text{max}}(z_t^k \mid x_t, m) \\ p_{\text{rand}}(z_t^k \mid x_t, m) \end{pmatrix}$$

네 사건 중 하나가 측정값을 만들어 낸다는 뜻이므로 네 가중치의 합은 반드시 1이어야 한다: $z_{\text{hit}} + z_{\text{short}} + z_{\text{max}} + z_{\text{rand}} = 1$.

여기서 기준점이 되는 예측 거리 $z_t^{k*}$는 로봇 포즈 $x_t$와 지도 $m$이 주어졌을 때 **ray casting**으로 구한다. 쉽게 말해 지도 위 포즈 $x_t$에서 $k$번째 빔이 향하는 각도로 가상 광선을 쭉 그어 나가다가, 지도상에서 가장 먼저 부딪히는 점유 셀까지의 거리를 재면 그것이 바로 $z_t^{k*}$다.

아래 의사코드는 한 스캔에 포함된 모든 빔에 대해 ray casting을 수행하고 4성분 혼합 likelihood를 곱해 전체 스캔 점수를 계산하는 흐름을 보여 준다.

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

이 알고리즘은 물리적 직관이 명쾌한 반면, 모든 빔마다 `ray_cast`를 실행해야 한다는 계산 부담을 안고 있다. 또한 6개의 모델 파라미터를 환경에 맞게 어떻게 맞출 것인가라는 숙제도 남는다.

### 2.7.3 빔 모델 — 파라미터 EM 학습

센서 종류가 바뀌거나, 같은 센서라도 붐비는 복도와 텅 빈 창고처럼 환경이 달라지면 hit·short·max·rand가 나타나는 비율과 분산 $\sigma_{\text{hit}}$이 크게 달라진다. 실험실 책상 위에서 손으로 대충 맞춘 파라미터를 실외나 복잡한 복도에 그대로 가져다 쓰면 위치 추정이 불안정해지기 쉽다.

따라서 여섯 개의 내재 파라미터(intrinsic parameter) $z_{\text{hit}}, z_{\text{short}}, z_{\text{max}}, z_{\text{rand}}, \sigma_{\text{hit}}, \lambda_{\text{short}}$는 실제 데이터로부터 학습하는 편이 안전하다. PR §6.3.2는 이미 지도를 알고 있는 환경에서 로봇을 움직여 모은 데이터 쌍 $\{(z_t^k, z_t^{k*})\}$에 대해 이 파라미터들을 최대우도(maximum likelihood)로 추정하는 방법을 제시한다.

문제는 로그 데이터만 보고서는 특정 측정값 $z^i$가 네 성분 중 도대체 어느 원인에서 비롯되었는지 직접 알 수 없다는 점이다. 어느 성분 소속인지 알면 파라미터를 곧바로 계산할 수 있고, 거꾸로 파라미터를 알면 각 측정값이 어느 성분일 확률이 높은지 짐작할 수 있는 닭과 달걀의 순환 구조다. **EM(Expectation-Maximization) 알고리즘**은 바로 이 순환을 두 단계의 반복으로 풀어낸다.

각 측정값 $z^i$의 숨은 출처를 나타내는 잠재 대응 변수(correspondence variable)를 $c_i \in \{\text{hit, short, max, rand}\}$라 두자. 먼저 **E-step**에서는 현재 추정된 파라미터를 고정한 채, 측정값 $z^i$가 각 성분에서 나왔을 사후 확률(기댓값)을 계산한다(PR 식 6.28~6.29).

$$e_{\text{hit}}^i = \frac{z_{\text{hit}} \cdot p_{\text{hit}}(z^i \mid z^{i*})}{p(z^i \mid z^{i*})}, \quad e_{\text{short}}^i = \frac{z_{\text{short}} \cdot p_{\text{short}}(z^i \mid z^{i*})}{p(z^i \mid z^{i*})}, \quad \dots$$

이어지는 **M-step**에서는 방금 구한 성분별 소속 확률 $e^i$를 가중치 삼아 파라미터를 새로 갱신한다(PR 식 6.30~6.32). 특히 분산 $\sigma_{\text{hit}}^2$과 지수 분포율 $\lambda_{\text{short}}$는 미분을 0으로 놓아 깔끔한 닫힌 해(closed-form)로 바로 얻어진다.

$$\sigma_{\text{hit}}^2 = \frac{\sum_i e_{\text{hit}}^i (z^i - z^{i*})^2}{\sum_i e_{\text{hit}}^i}, \qquad \lambda_{\text{short}} = \frac{\sum_i e_{\text{short}}^i}{\sum_i e_{\text{short}}^i \cdot z^i}$$

네 성분의 혼합 가중치 $z_{\text{hit}}, z_{\text{short}}, z_{\text{max}}, z_{\text{rand}}$ 역시 전체 데이터에서 각 성분이 차지한 기댓값의 평균으로 단순 갱신한다. 이 과정을 수렴할 때까지 반복하는 전체 절차가 아래 알고리즘이다.

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

이처럼 EM은 특정 센서와 환경의 물리적 특성을 데이터에서 자동으로 추출해 준다. ROS 생태계의 대표적 2D 위치 추정 패키지인 AMCL 역시 `sigma_hit`, `lambda_short`와 혼합 가중치들을 사용자 설정 파라미터로 열어 두고 있다. 패키지 기본값은 어디까지나 무난한 출발점일 뿐이므로, 실제 배포할 현장 환경이 기본 가정과 다르다면 데이터를 모아 파라미터를 다시 추정하거나 튜닝해야 제 성능을 낼 수 있다.

### 2.7.4 빔 모델 실무 고려

물리적으로 충실한 빔 모델을 실시간 로봇에 그대로 올리면 곧바로 두 가지 벽에 부딪힌다. 첫째는 **연산량**이고, 둘째는 앞서 언급한 **과신(over-confidence)**이다. 빔 모델에서 가장 비싼 연산은 ray casting인데, MCL 입자 필터에서 수백~수천 개의 파티클마다 스캔의 모든 빔에 ray casting을 돌리면 매 프레임 `파티클 수 × 빔 수`만큼 격자 지도를 순회해야 한다.

실무에서는 이를 세 가지 기법으로 해결한다. 먼저 계산량과 인접 빔 간의 중복성을 동시에 줄이기 위해 스캔 전체를 쓰지 않고 **균일한 각도 간격으로 일부 빔만 솎아 내어(beam subsampling)** 사용한다. 몇 개의 빔을 남길지는 센서의 각분해능, 환경의 복잡도, 파티클 수에 맞춰 실험으로 검증한다.

둘째로, 빔들이 서로 독립이라는 가정이 어긋나 확률 곱이 지나치게 뾰족해지는 현상을 누그러뜨리려면, 최종 스캔 likelihood를 $p(z_t \mid x_t, m)^{\alpha}$ ($0 < \alpha < 1$)처럼 거듭제곱해 낮추는 **tempering**을 적용한다. 지수 $\alpha$를 1보다 작게 잡으면 스캔 한 번이 파티클 분포를 과도하게 좁혀 정답 포즈까지 날려 버리는 위험이 줄어든다. $\alpha$ 값은 경험적으로 잡거나 교차 검증으로 찾는다.

셋째로, 정적인 환경 지도가 미리 주어져 있다면 지도의 모든 `(셀, 각도)` 조합에 대한 ray casting 결과를 사전에 몽땅 계산해 조회 테이블(lookup table)에 저장해 둘 수 있다. 이렇게 하면 실제 주행 중에는 테이블에서 값만 꺼내 오면 되므로 탐색 시간이 $O(1)$로 줄어 실시간 MCL 속도가 극적으로 빨라진다. 다만 지도가 넓고 해상도가 높을수록 테이블이 차지하는 메모리 크기가 폭증한다는 대가가 따른다(입자 필터의 상세 구현은 3장 §3.11.3 참조).

### 2.7.5 Likelihood Field

빔 모델은 사후 테이블을 만들더라도 여전히 근본적인 수학적 약점을 하나 안고 있다. **포즈 $x_t$에 대해 미분 가능하지 않고 불연속적으로 튄다**는 점이다. 예컨대 로봇 위치 $x_t$가 불과 몇 밀리미터 움직였을 뿐인데 가상 광선이 문설주 모서리를 살짝 스쳐 지나가면서, 예측 거리 $z_t^{k*}$가 1 m에서 저 멀리 복도 끝 15 m로 갑자기 점프할 수 있다. 이처럼 포즈 변화에 따라 likelihood 값이 계단처럼 뚝뚝 끊기면, 미분(gradient)을 따라 최적 포즈를 찾아가는 scan matching이나 hill-climbing 최적화 기법이 도중에 갇혀 수렴하지 못한다.

**Likelihood field**는 이 문제를 발상의 전환으로 깔끔하게 피해 간다. 로봇에서 광선을 쏘는 대신, 센서 측정값 $z_t^k$가 가리키는 **빔의 끝점(endpoint)**을 현재 후보 포즈 $x_t$를 이용해 전역 지도 좌표계 위로 곧장 떨어뜨려 보는 것이다. 그리고 그 끝점 좌표에서 지도상 가장 가까운 점유 셀(장애물)까지의 유클리드 거리 $\text{dist}$만 재서 점수를 매긴다. 빔의 끝점을 로봇 기준계에서 전역 좌표계로 옮기는 변환식은 다음과 같다(PR 식 6.33).

$$\begin{pmatrix} x_{z_t^k} \\ y_{z_t^k} \end{pmatrix} = \begin{pmatrix} x \\ y \end{pmatrix} + \begin{pmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{pmatrix} \begin{pmatrix} x_{k,\text{sens}} \\ y_{k,\text{sens}} \end{pmatrix} + z_t^k \begin{pmatrix} \cos(\theta + \theta_{k,\text{sens}}) \\ \sin(\theta + \theta_{k,\text{sens}}) \end{pmatrix}$$

여기서 $(x, y, \theta)$는 지도 좌표계에서의 로봇 포즈 $x_t$이고, $(x_{k,\text{sens}}, y_{k,\text{sens}})$는 로봇 중심에서 센서가 장착된 위치 오프셋, $\theta_{k,\text{sens}}$는 로봇 정면 대비 $k$번째 빔이 향하는 상대 각도다. 즉 로봇 위치에 센서 장착 위치를 회전해 더한 뒤, 그 지점에서 빔 방향으로 $z_t^k$만큼 직진한 좌표를 계산한 것이다.

최대 사거리 측정값($z_t^k = z_{\max}$)은 실제 물체에 닿아 나온 끝점이 아니므로 계산에서 제외한다. 따라서 $p_{\max}$ 항이 빠지고, 빔 하나의 likelihood는 끝점이 장애물 위에 떨어질수록(즉 $\text{dist} = 0$에 가까울수록) 높아지는 0-평균 가우시안과 균등 잡음의 합으로 단순해진다(PR 식 6.34~6.35).

$$p(z_t^k \mid x_t, m) = z_{\text{hit}} \cdot \mathcal{N}(\text{dist};\, 0,\, \sigma_{\text{hit}}^2) + z_{\text{rand}} \cdot \frac{1}{z_{\max}}$$

이 모델을 코드로 옮기면 ray casting 루프 없이 훨씬 간결한 형태가 된다.

**알고리즘: likelihood_field_range_finder_model** (PR Table 6.3 의역)

```
입력: z_t = {z_t^1, ..., z_t^K}, x_t = (x, y, θ)^T, m
출력: p(z_t | x_t, m)

1. q ← 1
2. for each k do:
3.     if z_t^k == z_max: continue   // max-range 무시
4.     // 빔 끝점을 전역 좌표로 변환
5.     x_ep ← x + x_{k,sens}·cos(θ) - y_{k,sens}·sin(θ) + z_t^k · cos(θ + θ_{k,\text{sens}})
6.     y_ep ← y + y_{k,sens}·cos(θ) + x_{k,sens}·sin(θ) + z_t^k · sin(θ + θ_{k,\text{sens}})
7.     // nearest obstacle까지 거리 (사전계산된 거리 변환 테이블에서 조회)
8.     dist ← nearest_obstacle_distance(x_ep, y_ep, m)
9.     q ← q * (z_hit · N(dist; 0, σ_hit²) + z_rand / z_max)
10. return q
```

지도가 정적이라면 격자 위의 모든 셀에서 가장 가까운 장애물까지의 거리를 미리 구해 놓는 **거리 변환(distance transform)**을 딱 한 번만 계산해 두면 된다. 실행 중에는 좌표 $(x_{\text{ep}}, y_{\text{ep}})$에 해당하는 거리 `dist`를 $O(1)$ 시간 만에 꺼내 쓸 수 있다. 참고로 이 사전 계산 테이블은 컴퓨터 그래픽스와 3D 비전에서 널리 쓰이는 SDF(Signed Distance Field)의 양수 영역과 본질적으로 같다. 무엇보다 로봇 포즈 $x_t$를 조금 움직일 때 `dist`와 likelihood가 모서리에서도 끊기지 않고 매끄럽게 변하기 때문에, gradient를 계산해 포즈를 정밀하게 밀어 넣는 scan matching에 매우 잘 어울린다.

물론 얻는 것이 있으면 잃는 것도 있다. Likelihood field에는 앞선 동적 장애물을 다루는 `short` 성분이 따로 없어, 지나가는 사람이 만든 측정값도 전부 무작위 잡음(`rand`) 성분으로 뭉뚱그려 처리된다. 또한 빔의 시작점부터 끝점까지 경로가 비어 있는지 전혀 검사하지 않고 오직 끝점의 거리만 보기 때문에, 센서 모델 입장에서는 마치 **벽 너머를 투시해 반대편 장애물에 닿은 것처럼** 잘못 계산할 수도 있다는 물리적 비현실성을 안고 있으며 지도가 정확하다는 가정에 크게 기대고 있다.

그럼에도 계산이 압도적으로 빠르고 포즈에 대해 매끄럽다는 장점 덕분에, 2D LiDAR로 실내를 주행하는 로봇의 AMCL 패키지는 likelihood field를 기본 모델로 채택하고 있다. 한편 점군 밀도가 높고 3차원 구조를 다루는 3D LiDAR나 RGB-D 환경에서는 격자 거리 테이블 대신 ICP나 NDT 같은 점군 정합 기법을 주로 쓴다(정합 알고리즘은 13장 §13.2.3, 위치 추정 필터는 14장 §14.7 참조).

### 2.7.6 상관 기반 모델 (Map Matching)

빔 단위 확률을 일일이 곱하는 대신, 이미지 템플릿 매칭처럼 지도 조각 두 개를 통째로 겹쳐 비교하는 직관적인 접근도 있다. **상관 기반 모델(map matching)**은 최근 몇 번의 스캔을 모아 작은 지역 격자 지도 $m_{\text{local}}$을 빚은 뒤, 이를 전체 지도 $m$ 위의 후보 포즈 $x_t$에 포개어 두 지도의 정규화 상관계수 $\rho$를 구한다. PR §6.5는 이 상관계수 값 자체를 다음과 같이 likelihood 대용으로 사용한다.

$$p(m_{\text{local}} \mid x_t, m) = \max\{\rho(m_{\text{local}}, m \mid x_t),\ 0\}$$

여기서 $\rho$는 두 지도를 $x_t$ 기준으로 정렬했을 때 겹치는 셀들의 점유 값끼리 계산한 피어슨(Pearson) 상관계수이며, 음의 상관관계가 나오면 0으로 잘라 낸다. 엄밀히 말해 $\rho$를 확률 분포처럼 취급할 수학적 근거가 있는 것은 아니며, 오차 모델 역시 따로 가정하지 않은 경험적 점수 함수다. 하지만 격자 연산만으로 계산이 매우 빠르고 구현이 단순하기 때문에, SLAM 백엔드에서 "지금 보고 있는 복도가 예전에 지나갔던 그 복도인가?"를 빠르게 판별하는 loop closure 검출 후보 점수 같은 곳에 실용적으로 쓰인다.

### 2.7.7 특징 기반 측정 — 랜드마크 모델

앞선 세 모델이 센서의 원시 거리 값이나 격자 지도 전체를 직접 다루었다면, **랜드마크 모델(landmark model)**은 센서 데이터에서 먼저 핵심적인 기하·시각 **특징 $f(z_t)$**만 뽑아내어 사용한다. 수백 개의 빔 대신 몇 개의 압축된 특징만 다루므로 차원이 극적으로 낮아져 연산이 가볍고, 랜드마크 목록으로 구성된 feature-based 지도와 곧바로 맞물린다.

추출하는 특징의 모습은 센서에 따라 다르다. 2D 거리 스캔에서는 직선 선분, 벽 모서리(corner), 국소 극솟값(기둥 등)을 뽑고, 카메라 영상에서는 에지, 코너, 혹은 SIFT·ORB 같은 국소 특징 디스크립터를 뽑는다. 이렇게 추상화된 특징 하나는 보통 $(r, \phi, s)$ 세 값으로 요약한다. 여기서 $r$은 로봇으로부터 특징까지의 거리(range), $\phi$는 로봇 정면 기준 방위각(bearing), $s$는 특징을 구별하는 signature(랜드마크 ID, 색상, 디스크립터 벡터 등)다.

지도 상의 $j$번째 랜드마크가 고정 좌표 $(m_{j,x}, m_{j,y})$에 놓여 있고 고유 signature가 $s_j$라고 하자. 로봇이 포즈 $x_t = (x, y, \theta)^T$에서 이 랜드마크를 관측했을 때 얻어지는 예상 측정값과 가우시안 관측 잡음 모델은 다음과 같다(PR 식 6.41).

$$\begin{pmatrix} r_t^i \\ \phi_t^i \\ s_t^i \end{pmatrix} = \begin{pmatrix} \sqrt{(m_{j,x} - x)^2 + (m_{j,y} - y)^2} \\ \operatorname{atan2}(m_{j,y} - y,\, m_{j,x} - x) - \theta \\ s_j \end{pmatrix} + \begin{pmatrix} \varepsilon_{\sigma_r^2} \\ \varepsilon_{\sigma_\phi^2} \\ \varepsilon_{\sigma_s^2} \end{pmatrix}$$

각 $\varepsilon_{\sigma^2}$는 평균이 0이고 분산이 $\sigma^2$인 가우시안 잡음이며, 거리·방위각·signature 세 채널의 잡음은 서로 독립이라고 가정한다. 여기서 구현할 때 꼭 챙겨야 할 함정이 하나 있다. 방위각(bearing)은 $-\pi$와 $+\pi$가 맞닿아 이어지는 원형 각도 공간이므로, 단순 뺄셈으로 가우시안 오차를 계산하면 각도가 $\pm\pi$ 경계를 살짝 넘나들 때 실제 오차는 아주 작은데도 수식상 오차가 거의 $2\pi$로 잘못 치솟는 wrap-around 현상이 벌어진다. 따라서 실제 코드에서는 각도 차이를 반드시 $[-\pi, \pi]$ 구간으로 정규화하거나, 원형 통계 분포인 von Mises 분포를 사용해야 한다.

만약 $i$번째 관측 특징이 지도의 $j$번째 랜드마크에 해당한다는 대응 관계($c_t^i = j$)를 이미 알고 있다면, 한 특징의 likelihood는 세 채널 가우시안 밀도의 곱으로 간단히 구해진다.

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

서로 다른 특징들끼리도 조건부 독립이라고 가정하면(PR 식 6.40) 전체 프레임의 likelihood는 각 특징 likelihood의 곱 $\prod_i$가 된다. 반대로 관측된 특징 하나를 바탕으로 "로봇이 있을 만한 포즈"를 거꾸로 샘플링할 수도 있다(PR Table 6.5 압축). 생각해보면 $(r, \phi)$ 관측 한 쌍은 3자유도 포즈 $(x, y, \theta)$ 가운데 두 개의 제약(거리와 상대 각도)만 고정한다. 따라서 나머지 1자유도가 풀려, 가능한 로봇 위치는 랜드마크를 중심으로 반지름이 $r$인 원 둘레 위에 도넛 모양으로 분포하며 원 위의 각 위치마다 로봇이 바라보는 방향 $\theta$가 자동으로 결정된다. 실제 샘플링에서는 원 위 각도 $\hat{\gamma} \sim U(0, 2\pi)$를 균등하게 뽑아 로봇 포즈 후보를 생성한다.

<!-- DEMO: landmark_donut.html -->

Visual SLAM에서 최적화 목적함수로 핵심을 이루는 재투영 잔차(reprojection residual) $\| \pi(K[R|t]\, X_w) - u \|^2_\Sigma$ 역시 랜드마크 모델과 같은 철학을 공유한다. 로봇 포즈와 3차원 랜드마크 좌표를 바탕으로 관측값을 예측한 뒤 실제 센서 관측과 비교해 차이를 줄여 나가기 때문이다. 다만 측정 대상이 거리·방위각이 아니라 2차원 픽셀 좌표라는 점에서 구체적 센서 모델만 다를 뿐이다. 또한 ORB나 SIFT 같은 시각 디스크립터는 방금 관측한 특징점이 지도의 몇 번 랜드마크에 해당하는지 짝짓는 데이터 연관(data association) 단계에서 주로 쓰인다. AprilTag나 ArUco 같은 인공 마커의 ID를 쓰면 이 대응 관계를 거의 헷갈리지 않고 풀 수 있지만, 모션 블러나 조명 탓에 마커를 아예 놓치거나 엉뚱한 ID로 잘못 읽는 오검출 문제는 여전히 대비해야 한다.

### 2.7.8 실무 정리: 모델 선택

네 가지 측정 모델은 물리적 정밀도, 계산 비용, 미분 가능성에서 서로 뚜렷한 트레이드오프를 이룬다. 실무에서 목적에 맞는 모델을 고를 때 참고할 수 있도록 핵심 특성을 한 표에 모았다(실제 성능과 속도는 센서 특성, 지도 해상도, 구현 최적화, 파라미터 설정에 따라 달라진다).

| 모델 | 정확도 | 계산 속도 | 미분 가능성 | 주요 용도 |
|------|--------|-----------|------------|-----------|
| 빔 모델 | 높음 | 느림 (ray casting) | 낮음 (불연속) | MCL high-fidelity, 진단 |
| Likelihood field | 중간 | 빠름 (DT lookup) | 높음 | AMCL 기본, gradient 정합 |
| 상관 기반 | 낮음 | 매우 빠름 | 낮음 | loop closure 검출 |
| 랜드마크 | 높음 (특징 의존) | 빠름 (저차원) | 높음 | visual SLAM, fiducial |

앞서 짚었듯 어느 모델을 택하든 빔이나 특징들이 서로 독립이라는 가정이 깨져 필터가 지나치게 과신하는 문제가 생길 수 있다. 실무에서는 이를 막기 위해 §2.7.4의 tempering($\alpha$ 제곱으로 likelihood 완화)을 쓰거나, 스캔 빔을 일정 간격으로 솎아 내어(beam subsampling) 상관을 줄이거나, 아예 인접 빔의 상관관계 또는 이상치(outlier)에 둔감한 robust likelihood 함수를 결합해 해결한다.

### 2.7.9 지금 시스템에서 쓰는 측정 모델

고전 교과서에 담긴 이 측정 모델들은 오늘날 실제 로봇 소프트웨어 스택 안에서도 이름만 조금씩 바뀐 채 현역으로 돌아가고 있다. 최신 scan matcher와 visual SLAM 역시 관측과 예측의 차이를 줄인다는 본질은 같으며, 목적함수와 지도 표현만 각 시스템의 목적에 맞게 진화했을 뿐이다.

먼저 ROS2의 표준 2D 위치 추정 패키지인 [Nav2 AMCL 문서](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/others/configuring_amcl/)를 열어 보면, `laser_model_type` 설정으로 `beam`, `likelihood_field`, `likelihood_field_prob` 가운데 하나를 고를 수 있고 기본값은 `likelihood_field`로 잡혀 있다. 또한 앞서 다룬 beam subsampling을 위한 `max_beams` 파라미터와, `likelihood_field_prob` 모드에서 파티클 대다수와 전혀 맞지 않는 동적 장애물 빔을 건너뛰는 `beam_skip_*` 옵션도 그대로 제공된다.

다른 주류 LiDAR 시스템들도 각자의 지도 표현에 맞춰 정합 목적함수를 빚어 쓴다. Cartographer는 확률 격자(probability grid) 위에서 상관 기반 scan matching으로 대략적인 초기 위치를 빠르게 찾은 뒤, 비선형 최적화로 포즈를 정교하게 다듬는다. 3D 점군 위치 추정 패키지인 `hdl_localization`은 3차원 공간의 점 분포를 가우시안 셀로 근사해 정합하는 NDT(Normal Distributions Transform)를 사용하며(`NDT_OMP`·`NDT_CUDA`), 이는 3D 공간 버전의 매끄러운 likelihood field에 비유할 수 있다.

정리하자면 PR 식 6.41의 range-bearing 모델, 카메라의 reprojection 모델, 그리고 DROID-SLAM 같은 최신 딥러닝 기반 시스템의 dense bundle adjustment는 모두 "현재 포즈와 장면 구조로 센서 관측을 예측해 잔차(residual)를 만든다"는 공통된 뼈대를 공유한다. 다만 무엇을 관측량으로 삼는지, 데이터 대응을 어떻게 찾는지, 잡음 분포를 어떻게 가정하는지, 무엇을 변수로 놓고 최적화하는지가 응용에 따라 갈릴 뿐이다.

이어지는 뒷장에서도 이 측정 모델들은 핵심 부품으로 다시 등장한다. 14장 §14.7.6의 MCL 의사코드가 호출하는 `measurement_model` 자리에 바로 이 절에서 배운 `beam_range_finder_model`이나 likelihood field 모델이 들어가며, §14.7B의 occupancy grid mapping에서는 거꾸로 센서 측정치에서 지도 셀의 점유 확률을 갱신하는 역방향 모델 `inverse_sensor_model`을 사용한다.

현장에서 센서를 처음 연결해 데이터를 띄울 때 코드나 수식보다 먼저 부딪히는 것은 하드웨어 연결 문제다. 아래 점검 절차와 센서 기술의 변화 흐름을 마지막으로 정리해 두자.

> **⚠ 센서 연결 점검**: 센서 데이터가 들어오지 않으면 드라이버, 케이블, IP 설정, 전원, USB 대역폭을 확인한다. `dmesg`, `lsusb`, `ping`으로 장치와 연결 상태를 먼저 기록한다.

> **기술 흐름: 센서 기술**
> - **~2010**: 이동 로봇 연구에서는 2D LiDAR와 일반 카메라를 널리 썼다. 당시 연산 자원으로는 스테레오를 실시간으로 처리하기 버거웠다.
> - **2010년대**: Kinect(2010) 같은 RGB-D 카메라와 작은 multi-beam 3D LiDAR가 널리 퍼졌다. 카메라와 IMU를 결합한 VIO가 로봇과 AR 기기에 들어갔다.
> - **2010년대 후반~2020년대 초반**: 비반복 스캔, MEMS, flash처럼 서로 다른 방식이 `solid-state LiDAR`라는 이름으로 나왔다. event camera와 자동차용 imaging radar 연구도 늘었다.
> - **2020년대**: Spinning LiDAR와 solid-state 계열을 field of view, range, resolution, motion distortion, cost에 따라 골라 쓴다. event camera와 Doppler radar는 고속·HDR·악천후 같은 조건에서 쓴다.
> - 센서의 스캔 방식과 timestamp 구조가 바뀌면 deskew(스캔 중 움직임 보정), calibration, data association의 가정도 바뀐다. 센서를 바꾸면 실제로 점이 어떻게 찍히는지와 노이즈 특성부터 확인한다.

이번 장에서 카메라, LiDAR, IMU, GNSS, Radar가 물리 세계를 어떤 신호로 포착하고 어떤 오차 분포를 만들어 내는지 살펴보았다. 눈치챘겠지만, 빔의 끝점을 로봇 좌표계에서 전역 좌표계로 옮길 때 이미 회전 행렬이 등장했고, 센서 잡음과 오차 전파를 다룰 때는 가우시안 분포와 기댓값, 그리고 gradient 기반 최적화가 자연스럽게 따라 나왔다. 이제 다음 **3장 수학적 기초**로 넘어가, 3차원 회전·변환을 다루는 선형대수와 리 군($SO(3), SE(3)$), 불확실성을 다루는 확률 및 베이즈 추정, 그리고 최적해를 찾는 미분과 최적화 이론을 탄탄히 다져 보자.
