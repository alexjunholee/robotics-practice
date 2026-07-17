# Ch.18 — 연구실 연구 방향

우리 연구실은 Spatial AI 시스템을 두 개의 모듈로 나눠 설계한다. 물리적 제약과 실시간 요구사항이 이 구조를 만들었고, 앞 챕터들에서 쌓은 개념들이 여기서 맞물린다.

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
│  │ • 제어 예산 기반 rate│     │ • 태스크 예산 기반 rate      │ │
│  └─────────────────────┘     └─────────────────────────────┘ │
└──────────────────────────────────────────────────────────────┘
```

### 두 모듈로 나누는 이유

온보드 컴퓨터 하나에서 모든 기능을 처리하기에는 무게와 전력, 응답 시간의 제약이 크다.

**물리적 제약**부터 보자. NVIDIA A100 GPU 서버는 무게가 수십 kg이고 전력도 수백 와트를 사용하므로 배터리로 움직이는 드론에 실을 수 없다. 로봇에는 대개 Jetson Orin 같은 임베디드 보드를 탑재하지만, 이 장치에서 DINOv2나 SAM 같은 대형 모델을 실시간으로 실행하기는 어렵다.

**시간 제약**도 다르다. 장애물 회피는 수십 ms 안에 반응해야 하지만, 물체의 의미를 해석하는 작업은 그보다 느리게 처리해도 된다. 전자는 서버 응답을 기다릴 수 없고, 후자는 더 큰 모델을 사용할 여지가 있다.

두 모듈은 이 차이에 맞춰 역할을 나눈다.

1. **계산 자원의 현실**: 로봇 온보드 컴퓨터(Jetson 등)는 대형 모델을 돌릴 여력이 없다
2. **실시간 요구사항**: 장애물 회피에는 수십 ms 단위의 응답이 필요하다.
3. **의미 이해**: VFM/VLA로 "깨진 유리컵"처럼 물체의 종류와 상태를 구분한다.
4. **상호 보완**: Local의 기하학적 정밀도와 Global의 의미론적 이해를 결합한다.

> 비유하자면, Local Module은 로봇의 **반사 신경**이고, Global Module은 로봇의 **대뇌 피질**이다. 뜨거운 냄비를 만지면 손을 먼저 떼고(반사), 그다음에 "아, 불이 켜져 있었구나"라고 이해한다(인지). 로봇도 마찬가지다.

## 18.2 Local Module: Lightweight Geometry

Local Module은 로봇에 직접 탑재되어 실시간으로 동작하며, 안전한 이동에 필요한 정보를 처리한다.

### 18.2.1 목표

- **Odometry**: 자신의 움직임 추정 — "나는 지금 어디에 있는가?"
- **Obstacle Detection**: 즉각적인 장애물 감지 — "앞에 뭔가 있다, 피해!"
- **Local Mapping**: 주변 환경의 기하학적 지도 — "내 주변 3m 이내는 이렇게 생겼다"

**운영 예**: 아파트 복도를 지나던 배달 로봇 앞에 아이가 갑자기 뛰어나오면, Local Module은 depth 센서로 장애물을 감지하고 odometry로 위치를 추정해 제어·안전 분석에서 정한 deadline 안에 회피 경로를 계산한다. 이 단계에서는 장애물의 종류보다 충돌 가능성을 먼저 판단한다. 물체의 의미는 Global Module이 별도로 해석한다.

### 18.2.2 특징

Local Module의 update rate와 latency deadline은 플랫폼 속도, braking distance, control bandwidth와 sensor rate에서 유도한다. 전력 한도도 선택한 embedded module과 power mode, 냉각 조건으로 정한다. 평균 FPS뿐 아니라 worst-case latency, jitter와 deadline miss를 target hardware에서 측정해야 한다.

### 18.2.3 기술 스택

**Classical 방법**:
- ORB-SLAM3: Feature-based Visual SLAM — 카메라 하나로 위치 추정 (→ 9장, 14장 참고)
- VINS-Mono: Visual-Inertial Odometry — 카메라+IMU 융합 (→ 14장 참고)
- FAST-LIO2: LiDAR-Inertial Odometry — LiDAR+IMU 융합 (→ 2장, 14장 참고)

**경량 학습 모델**:
- 경량 depth estimation — MobileNet 기반으로 압축 (→ 10장 참고)
- 압축된 segmentation 모델 — knowledge distillation 적용 (→ 10장, 11장 참고)
- TensorRT 최적화 — NVIDIA GPU용 graph·kernel·precision 최적화 후보

**Edge 배포**:

```bash
# TensorRT 최적화 예시
trtexec --onnx=model.onnx --saveEngine=model.trt --fp16 --workspace=4096
```

> TensorRT는 NVIDIA GPU용 inference engine을 만든다. FP16이 memory와 latency를 줄일 수 있지만 이득과 task metric 변화는 model, input, batch, power mode와 software version에 따라 달라진다. Target Jetson에서 end-to-end latency와 validation metric을 함께 비교해 채택 여부를 정한다.

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

**실행 시나리오**: 위 코드에서 `process()`는 센서 데이터가 들어올 때마다 호출된다. IMU 데이터는 100Hz(초당 100번), 카메라 이미지는 30Hz로 들어온다. 매 프레임마다 "나 지금 어디?"(odometry)와 "앞에 뭐 있어?"(obstacle)를 계산하고, 중요한 순간인 키프레임만 Global Module에 보낸다. 모든 프레임을 전송하면 네트워크 대역폭을 초과할 수 있기 때문이다.

## 18.3 Global Module: VFM-based Understanding

Global Module은 서버나 cloud에서 동작하며 물체의 종류와 관계를 해석한다. Local Module이 앞의 장애물을 감지하면, Global Module은 이를 깨진 유리잔으로 분류하고 scene graph의 위치 정보와 연결할 수 있다.

### 18.3.1 목표

- **전체 지도 이해**: 공간 구조와 의미 파악 — "여기는 주방이고, 저기는 거실이다"
- **Semantic Scene Graph**: 객체 간 관계 표현 — "컵이 테이블 위에 있다"
- **Long-term Memory**: 환경 변화 추적 — "어제는 여기에 의자가 없었는데 오늘은 있다"

**실제 시나리오**: 가정용 서비스 로봇이 매일 집 안을 돌아다니면서 환경을 학습한다. Global Module은 "거실에 소파, TV, 테이블이 있고, 주방에는 냉장고, 싱크대가 있다"는 고수준 지도를 유지한다. 사용자가 "거실 테이블에 있는 리모컨 가져와"라고 하면, Scene Graph에서 리모컨의 위치를 찾아 Local Module에 waypoint를 전달한다.

### 18.3.2 특징

DINOv2와 SAM2에는 크기가 다른 model variant가 있고 모두 수십억 parameter인 것은 아니다. Global Module의 hardware와 update period는 variant, input resolution, precision, scene 수와 허용 응답 시간으로 정한다. Local control deadline과 분리할 수 있는 task도 있지만, 사용자 상호작용이나 변화 감지처럼 end-to-end latency 요구가 있는 경우에는 별도 budget이 필요하다.

### 18.3.3 기술 스택

**Vision Foundation Models** (→ 11장 참고):
- DINOv2: Dense feature extraction — 이미지의 모든 픽셀에 대해 의미 있는 feature 벡터 생성
- SAM2: promptable image/video segmentation — point·box·mask prompt로 대상 mask를 추적
- GroundingDINO: Text-guided detection — "빨간 컵"이라고 말하면 찾아줌

**3D Understanding** (→ 11장, 13장 참고):
- Gaussian Splatting with semantic features — 예쁘고 빠른 3D 재구성 + 의미 정보
- 3D Scene Graph 구축 — 객체들의 관계를 그래프로 표현
- VFM features의 3D lifting — 2D 이미지에서 뽑은 feature를 3D 공간에 올리기

**Language Integration** (→ 11장, 12장 참고):
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

**다른 시나리오 — 통신 불안정 상황**: 로봇이 지하 주차장에서 작업 중인데 WiFi가 끊어졌다. 이 경우 Local Module만으로 동작해야 한다. Odometry로 위치를 추정하고, 장애물을 피하면서 마지막으로 받은 waypoint까지 이동한다. WiFi가 복구되면 그동안의 데이터를 Global에 한꺼번에 보내고, 업데이트된 계획을 받는다. 실제 로봇은 이런 **graceful degradation**이 필요하다.

### 18.4.3 통신 및 동기화

**통신 방식**:
- ROS2 DDS: 로컬 네트워크 (같은 건물 안)
- WebSocket: 클라우드 연결 (원격 서버)
- 5G/WiFi: 모바일 로봇 (실외 환경)

**동기화 전략**: 연속 스트리밍 대신 키프레임 단위로 보내 대역폭을 줄이고, Global 응답을 기다리지 않고 비동기로 처리해 Local이 멈추지 않도록 한다. 자주 방문하는 영역은 캐싱해 중복 전송을 피한다.

## 18.5 연구 과제 예시

아래 연구 과제들은 실제로 우리 연구실에서 진행하거나 진행할 수 있는 주제들이다. 각 과제마다 선행 학습이 필요한 챕터를 표시해두었으니, 관심 있는 주제가 있으면 해당 챕터부터 공부하자.

### Local Module 연구

1. **더 가벼운 SLAM**
    - 신경망 기반 경량 VO — 기존 VO를 대체하되 Jetson에서 동작하도록 설계
    - 이벤트 카메라 활용 — 저전력·초고속 센서로 극한 환경의 SLAM을 구성
    - 하드웨어 가속(FPGA) — SLAM의 핵심 연산을 전용 하드웨어로 구현
    - 선행 학습: 9장(카메라 모델), 14장(Visual Odometry, SLAM) 필수, 3장(최적화) 권장
2. **효율적 장애물 인식**
    - Depth-only obstacle detection — RGB 없이 depth 정보만으로 장애물 감지
    - 시간적 일관성 — 물체가 프레임마다 나타났다 사라지는 깜빡임을 억제
    - Uncertainty-aware — "이게 장애물인지 확실하지 않다"는 불확실성 정보도 회피 결정에 반영
    - 선행 학습: 10장(Depth Estimation, Object Detection) 필수, 3장(좌표 변환) 중요
3. **센서 융합 최적화**
    - Tight coupling 경량화 — IMU+Camera+LiDAR를 촘촘하게 결합하되 가볍게
    - 센서 드롭아웃 대응 — 센서 하나가 고장나도 계속 동작
    - 선행 학습: 2장(센서), 14장(Visual Odometry), 3장(최적화) 필수

### Global Module 연구

1. **VFM의 3D 확장**
    - DINOv2 features in 3D — 2D feature를 3D 공간에 올려서 활용
    - Semantic Gaussian Splatting — 3D 재구성에 의미 정보를 같이 넣기
    - 3D scene understanding — "이 공간이 어떤 구조인지" 이해하기
    - 선행 학습: 10장(Depth), 13장(3D 표현), 11장(VFM) 필수, 9장(카메라 모델) 기본
2. **VLA 통합**
    - Open-vocabulary manipulation — "저 빨간 거 집어" 같은 자연어 명령으로 로봇팔 제어
    - 언어 기반 내비게이션 — 자연어 명령에 따라 이동
    - 상황 인식 행동 — "아이가 있으니 천천히"처럼 맥락을 행동 제약으로 변환
    - 선행 학습: 11장(VFM 활용), 12장(VLA) 필수, 10장(Detection)도 알면 좋다
3. **Scalability**: 아파트 단지·캠퍼스 전체를 단일 지도로 표현하고, 수 GB짜리 지도를 압축·갱신하며, 여러 로봇이 함께 만든 지도를 공유하는 문제다. 선행 학습: 14장(SLAM), 3장(최적화), 11장(VFM) 필수.

### Integration 연구

1. **효율적 통신**: 무엇을 언제 보낼지 정해야 한다. 모든 프레임을 보내면 대역폭을 소진하고, 키프레임만 보내면 Global이 환경 변화를 놓칠 수 있다. 5G가 끊기거나 WiFi가 느릴 때의 열화 전략도 함께 설계한다. 선행 학습: 14장(SLAM, 키프레임 선택), Local/Global 모듈 이해.
2. **Fallback 전략**
    - 통신 끊김 시 Local-only 동작 — 서버 연결 없이도 기본 임무 수행
    - Graceful degradation — 기능이 점진적으로 줄어들되, 갑자기 멈추지는 않기
    - 선행 학습: 시스템 전체 이해 필요. 최소 3~14장은 읽고 오자
3. **일관성 유지**: Local이 "여기 빈 공간"이라 보고, Global이 "거기 의자 있음"이라 기억하면 로봇은 어느 쪽을 믿어야 할지 모른다. 두 지도를 동기화하고, 의미 정보("저건 의자"가 나중에 "테이블"로 바뀌는 일)를 억제하는 문제다. 선행 학습: 3장(최적화), 14장(SLAM, 맵 관리) 필수.

## 18.6 Motivation과 Novelty를 가르는 질문

Motivation은 왜 이 문제를 풀어야 하는지 설명하고, novelty는 기존 방법에서 무엇을 어떻게 바꾸었는지 특정한다. 둘은 연구 방향을 잡거나 첫 논문을 쓸 때 자주 뒤섞인다.

"기존 방법이 X를 하지 못하므로 모듈을 붙였다"는 문장만으로는 motivation을 넘어가기 어렵다. 그 모듈이 왜 필요한지, 왜 그 형태여야 하는지까지 설명해야 novelty가 구체화된다.

아래 세 논문은 문제 제기와 설계 기여의 차이를 보여준다.

### Case 1 — ORB-SLAM2 (Mur-Artal & Tardós 2017)

- **Motivation**: 단안용 ORB-SLAM의 map reuse·loop closing·relocalization 구조를 stereo와 RGB-D 입력에도 적용한다.
- **직접적 확장**: 입력 modality마다 별도의 SLAM 시스템을 만든다.
- **논문의 설계**: 세 modality가 tracking·local mapping·loop closing의 시스템 구조와 ORB 특징을 공유하되, stereo/RGB-D 관측은 disparity에서 얻은 depth와 metric-scale bundle adjustment에 반영한다.
- **설계 원리**: 공통 시스템 구조는 유지하고, modality별 차이를 관측 생성과 bundle-adjustment 잔차에 둔다.

원 논문은 [*ORB-SLAM2: An Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras*](https://doi.org/10.1109/TRO.2017.2705103)다. 2015년 ORB-SLAM은 단안 시스템이므로 이 사례의 세 modality 통합 근거로 사용할 수 없다.

### Case 2 — 3D Gaussian Splatting (Kerbl et al. 2023)

- **Motivation**: NeRF가 너무 느리다 → 빠르게 만들어야 한다
- **직접적 확장**: NeRF 위에 sparse sampling · pruning · distillation 같은 가속 모듈을 추가
- **논문의 설계**: ray-marching의 탐색 비용을 병목으로 보고, 명시적 primitive인 3D Gaussian으로 표현을 교체. Primitive는 직접 rasterization할 수 있게 설계
- **설계 원리**: 속도 한계의 원인을 개별 연산이 아니라 표현과 rendering 방식의 조합에서 찾는다.

### Case 3 — DUSt3R (Wang et al. 2024)

기존 SfM은 camera intrinsics가 필요하고 단계별 오차에 민감하다. Matching이나 triangulation 한 단계만 신경망으로 교체할 수도 있지만, Wang et al.은 출력 형식 자체를 바꿨다. 두 view를 입력받아 공통 좌표계의 pointmap을 직접 예측하면 intrinsics, correspondence, structure를 함께 얻을 수 있다. Camera intrinsics는 별도의 입력이 아니라 pointmap에서 유도되는 값이 된다. DUSt3R의 기여는 이처럼 SfM의 단계별 분해를 pointmap 예측 문제로 다시 정식화한 데 있다.

### 세 논문이 공유하는 설계 질문

세 논문은 모두 *왜 이 모듈이 이 형태여야 하는가*를 묻는다. 답은 모듈의 수보다 인터페이스, 표현, 출력 형식을 어떻게 정했는지에 있다.

> Contribution 절에는 *왜 이 모듈이어야 하는가*에 대한 답이 있어야 한다. 문제의 필요성만 설명한다면 motivation에 머물고, 설계 선택의 근거까지 제시해야 novelty가 드러난다.

논문에서 motivation과 method를 전개하는 방법은 [「연구노트」 Ch.23 Introduction](../research-notes/guide.html#chapter-23)과 [Ch.25 Method](../research-notes/guide.html#chapter-25)에서 자세히 다룬다.
