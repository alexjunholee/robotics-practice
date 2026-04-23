# Ch.18 — 연구실 연구 방향

우리 연구실은 Spatial AI 시스템을 두 개의 모듈로 나눠 설계한다. 이 구조는 물리적 제약과 실시간 요구사항에서 나온 선택이고, 앞 챕터들에서 쌓은 개념들이 여기서 맞물린다.

## 18.1 개요

Spatial AI 시스템을 **두 개의 모듈**로 구분하여 설계한다.

```
┌──────────────────────────────────────────────────────────────┐
│                     Spatial AI System                         │
├──────────────────────────────────────────────────────────────┤
│  ┌─────────────────────┐     ┌─────────────────────────────┐ │
│  │   Local Module      │     │      Global Module          │ │
│  │   (경량, 온보드)     │ ←→  │   (중량, 서버/클라우드)      │ │
│  │                     │     │                             │ │
│  │ • 실시간 Geometry    │     │ • VFM 기반 이해             │ │
│  │ • Odometry          │     │ • Semantic Scene Graph      │ │
│  │ • Local Obstacle    │     │ • Long-term Memory          │ │
│  │ • 10-100 Hz         │     │ • 1-10 Hz                   │ │
│  └─────────────────────┘     └─────────────────────────────┘ │
└──────────────────────────────────────────────────────────────┘
```

**왜 두 모듈인가? — 직관적으로 이해하기**

"그냥 좋은 컴퓨터 하나 올리면 안 되나요?"라고 생각할 수 있다. 솔직히 그게 가능하면 그렇게 하고 싶다. 하지만 현실은 그렇지 않다.

먼저 **물리적 제약**을 생각해 보자. 로봇은 움직여야 한다. NVIDIA A100 GPU 서버를 드론에 올릴 수는 없다 — 무게만 해도 수십 kg이고, 전력도 수백 와트를 먹는다. 배터리로 돌아가는 로봇에게는 비현실적이다. 그래서 로봇에 실제로 탑재할 수 있는 컴퓨터는 Jetson Orin 같은 임베디드 보드인데, 이 보드로는 DINOv2나 SAM 같은 대형 모델을 실시간으로 돌릴 수 없다.

다음으로 **시간 제약**이 있다. 로봇이 복도를 걸어가고 있는데, 벽에 부딪히기 0.1초 전에 "잠깐, 서버 응답 기다리는 중..."이면 안 된다. 장애물 회피처럼 "지금 당장" 반응해야 하는 것과, "저 물체가 뭔지 이해하기"처럼 좀 느려도 괜찮은 것은 다른 종류의 문제다.

그래서 우리는 이렇게 나눈다:

1. **계산 자원의 현실**: 로봇 온보드 컴퓨터(Jetson 등)는 대형 모델을 돌릴 여력이 없다
2. **실시간 요구사항**: 장애물 회피는 즉각 반응 필요 — 0.1초가 생사를 가른다
3. **깊은 이해**: VFM/VLA는 높은 계산량 필요 — "저건 깨진 유리컵이니 조심해" 같은 판단
4. **상호 보완**: 기하학적 정밀함(Local) + 의미론적 이해(Global) = 진짜 똑똑한 로봇

> 비유하자면, Local Module은 로봇의 **반사 신경**이고, Global Module은 로봇의 **대뇌 피질**이다. 뜨거운 냄비를 만지면 손을 먼저 떼고(반사), 그다음에 "아, 불이 켜져 있었구나"라고 이해한다(인지). 로봇도 마찬가지다.

## 18.2 Local Module: Lightweight Geometry

로봇에 직접 탑재되어 실시간으로 동작하는 모듈이다. 로봇이 "지금 이 순간" 안전하게 움직이기 위해 필요한 최소한의 정보를 처리한다.

### 18.2.1 목표

- **Odometry**: 자신의 움직임 추정 — "나는 지금 어디에 있는가?"
- **Obstacle Detection**: 즉각적인 장애물 감지 — "앞에 뭔가 있다, 피해!"
- **Local Mapping**: 주변 환경의 기하학적 지도 — "내 주변 3m 이내는 이렇게 생겼다"

**실제 시나리오**: 배달 로봇이 아파트 복도를 지나가고 있다고 하자. 갑자기 아이가 뛰어나온다. 이때 Local Module은 depth 센서로 즉시 장애물을 감지하고, odometry로 자기 위치를 파악해서, 0.05초 안에 회피 경로를 계산한다. "아이인지 강아지인지"는 몰라도 된다 — 그건 Global Module의 일이다. Local Module은 "앞에 뭔가 있으니 피하자"만 알면 된다.

### 18.2.2 특징

- **저지연**: 10-100 Hz 동작 (10ms~100ms마다 한 번씩 처리)
- **제한된 자원**: Jetson, 임베디드 GPU — 전력 15~30W 수준
- **확정적 동작**: 예측 가능한 응답 시간 — "최악의 경우에도 50ms 안에 답을 준다"

### 18.2.3 기술 스택

**Classical 방법**:
- ORB-SLAM3: Feature-based Visual SLAM — 카메라 하나로 위치 추정 (→ 4장, 9장 참고)
- VINS-Mono: Visual-Inertial Odometry — 카메라+IMU 융합 (→ 9장 참고)
- FAST-LIO2: LiDAR-Inertial Odometry — LiDAR+IMU 융합 (→ 2장, 9장 참고)

**경량 학습 모델**:
- 경량 depth estimation — MobileNet 기반으로 압축 (→ 5장 참고)
- 압축된 segmentation 모델 — knowledge distillation 적용 (→ 5장, 6장 참고)
- TensorRT 최적화 — NVIDIA GPU에서 2-5배 속도 향상

**Edge 배포**:

```bash
# TensorRT 최적화 예시
trtexec --onnx=model.onnx --saveEngine=model.trt --fp16 --workspace=4096
```

> TensorRT가 뭐냐면, PyTorch로 학습한 모델을 NVIDIA GPU에 최적화된 형태로 변환해주는 도구다. FP16(반정밀도)으로 바꾸면 모델 크기가 절반이 되면서도 정확도는 거의 유지된다. Jetson에서 YOLO를 돌릴 때 TensorRT 없이는 5 FPS, 있으면 30 FPS — 이 차이가 실제 로봇에서는 "쓸 수 있다 vs 없다"의 차이다.

### 18.2.4 예시 구현

```python
# Local Module 개념 코드
class LocalModule:
    def __init__(self):
        self.odometry = FastLIO2()
        self.obstacle_detector = LightweightObstacleNet()  # TensorRT

    def process(self, sensor_data):
        # 1. Odometry 업데이트 (100 Hz)
        pose = self.odometry.update(sensor_data.imu, sensor_data.lidar)

        # 2. 장애물 감지 (30 Hz)
        obstacles = self.obstacle_detector(sensor_data.image)

        # 3. Global Module로 키프레임 전송
        if self.is_keyframe(pose):
            self.send_to_global(sensor_data, pose)

        return pose, obstacles
```

**시나리오로 읽기**: 위 코드에서 `process()`는 센서 데이터가 들어올 때마다 호출된다. IMU 데이터는 100Hz(초당 100번)로 오고, 카메라 이미지는 30Hz로 온다. 매 프레임마다 "나 지금 어디?" (odometry)와 "앞에 뭐 있어?" (obstacle)를 계산하고, 중요한 순간(키프레임)에만 Global Module에 데이터를 보낸다. 모든 프레임을 보내면 네트워크가 터지니까.

## 18.3 Global Module: VFM-based Understanding

서버 또는 클라우드에서 동작하는 고수준 이해 모듈이다. Local Module이 "앞에 뭔가 있다"까지만 알면, Global Module은 "저건 깨진 유리잔이고, 주인이 거실에서 떨어뜨린 것 같다"까지 이해한다.

### 18.3.1 목표

- **전체 지도 이해**: 공간 구조와 의미 파악 — "여기는 주방이고, 저기는 거실이다"
- **Semantic Scene Graph**: 객체 간 관계 표현 — "컵이 테이블 위에 있다"
- **Long-term Memory**: 환경 변화 추적 — "어제는 여기에 의자가 없었는데 오늘은 있다"

**실제 시나리오**: 가정용 서비스 로봇이 매일 집 안을 돌아다니면서 환경을 학습한다. Global Module은 "거실에 소파, TV, 테이블이 있고, 주방에는 냉장고, 싱크대가 있다"는 고수준 지도를 유지한다. 사용자가 "거실 테이블에 있는 리모컨 가져와"라고 하면, Scene Graph에서 리모컨의 위치를 찾아 Local Module에 waypoint를 전달한다.

### 18.3.2 특징

- **높은 정확도**: 대형 VFM 활용 — DINOv2, SAM2 같은 수십억 파라미터 모델
- **풍부한 컴퓨팅**: GPU 서버, 클라우드 — RTX 4090, A100 수준의 GPU
- **비실시간 허용**: 1-10 Hz — "1초에 한 번 업데이트해도 괜찮다"

### 18.3.3 기술 스택

**Vision Foundation Models** (→ 6장 참고):
- DINOv2: Dense feature extraction — 이미지의 모든 픽셀에 대해 의미 있는 feature 벡터 생성
- SAM2: Open-vocabulary segmentation — "아무 물체나" 지정하면 정확하게 분리
- GroundingDINO: Text-guided detection — "빨간 컵"이라고 말하면 찾아줌

**3D Understanding** (→ 6장, 8장 참고):
- Gaussian Splatting with semantic features — 예쁘고 빠른 3D 재구성 + 의미 정보
- 3D Scene Graph 구축 — 객체들의 관계를 그래프로 표현
- VFM features의 3D lifting — 2D 이미지에서 뽑은 feature를 3D 공간에 올리기

**Language Integration** (→ 6장, 7장 참고):
- CLIP features for open-vocabulary — "본 적 없는 물체"도 텍스트로 검색 가능
- LLM for scene reasoning — "이 방은 어떤 용도일까?" 추론
- VLA for action planning — "컵을 집으려면 어떻게 팔을 움직여야 할까?"

### 18.3.4 예시 구현

```python
# Global Module 개념 코드
class GlobalModule:
    def __init__(self):
        self.dinov2 = load_dinov2()
        self.sam = load_sam2()
        self.scene_graph = SemanticSceneGraph()
        self.gaussian_map = GaussianSplatMap()

    def process_keyframe(self, image, depth, pose):
        # 1. VFM feature 추출
        features = self.dinov2.extract(image)

        # 2. Open-vocabulary segmentation
        masks = self.sam.segment(image, prompts=self.get_prompts())

        # 3. 3D Scene Graph 업데이트
        self.scene_graph.update(masks, depth, pose, features)

        # 4. Gaussian Map 업데이트
        self.gaussian_map.add_keyframe(image, depth, pose, features)

    def query(self, text_prompt):
        # "Where is the red cup?" → 위치 반환
        return self.scene_graph.find(text_prompt)
```

**시나리오로 읽기**: Local Module에서 키프레임이 올 때마다 `process_keyframe()`이 호출된다. DINOv2로 이미지에서 풍부한 feature를 뽑고, SAM으로 물체들을 분리하고, 이걸 3D Scene Graph와 Gaussian Map에 누적한다. 나중에 사용자가 "빨간 컵 어디 있어?"라고 물으면 `query()`로 검색한다. 이 전체 과정이 1초 정도 걸려도 괜찮다 — 실시간 안전은 Local Module이 책임지니까.

## 18.4 두 모듈의 협업

두 모듈은 독립적으로 동작하지만, 서로 정보를 주고받으며 협력한다. 마치 드라이버(Local)와 네비게이션 앱(Global)의 관계와 비슷하다 — 드라이버는 눈앞의 도로를 보고 운전하고, 네비게이션은 전체 경로를 안내한다.

### 18.4.1 Local → Global

**전송 내용**:
- 키프레임 이미지/포인트 클라우드
- 로컬 포즈
- 센서 메타데이터

**키프레임 선택 기준**:
- 이동 거리/회전량 threshold — "1m 이동하거나 30도 회전하면 보내기"
- 장면 변화 감지 — "새로운 방에 들어갔다"
- 정보량 (특징점 수, 커버리지) — "이 프레임에 새로운 정보가 많다"

### 18.4.2 Global → Local

**전송 내용**:
- 사전 지도 (필요 영역) — "주방 근처의 장애물 정보"
- Semantic 정보 (객체 위치, 클래스) — "테이블은 여기, 의자는 저기"
- 네비게이션 waypoints — "이 경로를 따라가라"

**예시 시나리오**:

```
1. 사용자: "Go to the kitchen and bring the cup"

2. Global:
   - VLM으로 명령 이해
   - Scene Graph에서 kitchen, cup 위치 찾기
   - 경로 계획

3. Global → Local:
   - Waypoints: [현재 → 복도 → 주방 → 컵 앞]
   - 주방 영역의 local map
   - 컵의 예상 위치

4. Local:
   - Waypoints 따라 이동
   - 실시간 장애물 회피
   - 컵 근처에서 정밀 접근
```

**다른 시나리오 — 통신 불안정 상황**: 로봇이 지하 주차장에서 작업 중인데 WiFi가 끊어졌다. 이 경우 Local Module만으로 동작해야 한다. Odometry로 위치를 추정하고, 장애물을 피하면서 마지막으로 받은 waypoint까지 이동한다. WiFi가 복구되면 그동안의 데이터를 Global에 한꺼번에 보내고, 업데이트된 계획을 받는다. 이런 **graceful degradation**이 실제 로봇에서는 매우 중요하다.

### 18.4.3 통신 및 동기화

**통신 방식**:
- ROS2 DDS: 로컬 네트워크 (같은 건물 안)
- WebSocket: 클라우드 연결 (원격 서버)
- 5G/WiFi: 모바일 로봇 (실외 환경)

**동기화 전략**:
- 키프레임 기반 (연속 스트리밍 X) — 대역폭 절약
- 비동기 처리 (Global 완료 안 기다림) — Local은 멈추지 않는다
- 캐싱 (자주 방문 영역) — 매번 같은 데이터를 보내지 않는다

## 18.5 연구 과제 예시

아래 연구 과제들은 실제로 우리 연구실에서 진행하거나 진행할 수 있는 주제들이다. 각 과제에 대해 **선행 학습이 필요한 챕터**를 표시해두었으니, 관심 있는 주제가 있으면 해당 챕터부터 공부하자.

### Local Module 연구

1. **더 가벼운 SLAM**
    - 신경망 기반 경량 VO — 기존 VO를 neural network로 대체하되, Jetson에서 돌아가게
    - 이벤트 카메라 활용 — 초고속, 저전력 카메라로 극한 환경에서 SLAM
    - 하드웨어 가속 (FPGA) — SLAM의 핵심 연산을 하드웨어로 구현
    - **선행 학습**: 4장(카메라 모델), 9장(Visual Odometry, SLAM) 필수. 3장(최적화)도 권장
2. **효율적 장애물 인식**
    - Depth-only obstacle detection — RGB 없이 depth 정보만으로 장애물 감지
    - Temporal consistency — 프레임 간 일관성 유지 (한 프레임에서 보였다 안 보였다 하면 안 됨)
    - Uncertainty-aware — "이게 장애물인지 확실하지 않다"는 정보도 활용
    - **선행 학습**: 5장(Depth Estimation, Object Detection) 필수. 3장(좌표 변환)도 중요
3. **센서 융합 최적화**
    - Tight coupling 경량화 — IMU+Camera+LiDAR를 촘촘하게 결합하되 가볍게
    - 센서 드롭아웃 대응 — 센서 하나가 고장나도 계속 동작
    - **선행 학습**: 2장(센서), 9장(Visual Odometry), 3장(최적화) 필수

### Global Module 연구

1. **VFM의 3D 확장**
    - DINOv2 features in 3D — 2D feature를 3D 공간에 올려서 활용
    - Semantic Gaussian Splatting — 3D 재구성에 의미 정보를 같이 넣기
    - 3D scene understanding — "이 공간은 어떤 구조인가" 이해
    - **선행 학습**: 5장(Depth), 8장(3D 표현), 6장(VFM) 필수. 4장(카메라 모델)은 기본
2. **VLA 통합**
    - Open-vocabulary manipulation — "저 빨간 거 집어" 같은 명령으로 로봇팔 제어
    - Language-guided navigation — 자연어 명령으로 이동
    - 상황 인식 행동 — "아이가 있으니 천천히 움직여라"
    - **선행 학습**: 6장(VFM 활용), 7장(VLA) 필수. 5장(Detection)도 알면 좋다
3. **Scalability**
    - 대규모 환경 표현 — 아파트 단지 전체, 캠퍼스 전체를 하나의 지도로
    - 지도 압축 및 업데이트 — 수 GB짜리 지도를 효율적으로 관리
    - Multi-robot 협업 — 여러 로봇이 함께 지도를 만들고 공유
    - **선행 학습**: 9장(SLAM), 3장(최적화), 6장(VFM) 필수

### Integration 연구

1. **효율적 통신**
    - 무엇을 언제 전송할 것인가? — 모든 걸 보내면 대역폭 낭비, 안 보내면 Global이 무용지물
    - 대역폭 제한 하 최적 전략 — 5G가 끊기면? WiFi가 느리면?
    - **선행 학습**: 9장(SLAM, 키프레임 선택), 위 Local/Global 모듈 이해
2. **Fallback 전략**
    - 통신 끊김 시 Local-only 동작 — 서버 연결 없이도 기본 임무 수행
    - Graceful degradation — 기능이 점진적으로 줄어들되, 갑자기 멈추지는 않기
    - **선행 학습**: 전체적인 시스템 이해 필요. 최소 3-9장은 읽고 오자
3. **일관성 유지**
    - Local/Global 지도 동기화 — 두 모듈의 지도가 다르면 로봇이 혼란
    - Semantic 정보 일관성 — "저건 의자"라고 했는데 나중에 "테이블"로 바뀌면 안 됨
    - **선행 학습**: 3장(최적화), 9장(SLAM, 맵 관리) 필수
