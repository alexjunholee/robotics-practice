# Ch.18 — 연구실 연구 방향

우리 연구실은 Spatial AI 시스템을 두 모듈로 나눠 설계한다. 앞 장들에서 다룬 기술이 이 두 모듈에 나뉘어 들어간다.

## 18.1 개요

시스템은 로봇에 싣는 Local Module과 서버에서 돌리는 Global Module로 이루어진다.

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

로봇에 싣는 컴퓨터 하나로 모든 기능을 처리하기에는 무게, 전력, 응답 시간의 제약이 크다.

물리적 제약부터 보자. NVIDIA A100 GPU 한 장의 최대 TDP는 PCIe 40GB 250 W, PCIe 80GB 300 W, SXM 400 W다. 이런 GPU를 꽂는 서버는 랙에 설치하는 장비다. 배터리로 움직이는 로봇에는 대개 Jetson Orin 같은 임베디드 보드(AGX Orin 64GB 모듈은 15~60 W)를 싣는다. 이 보드에서 DINOv2나 SAM의 큰 variant를 실시간으로 돌리기는 어렵다.

시간 제약도 다르다. 장애물 회피는 수십 ms 안에 반응해야 하므로 로봇 안에서 처리한다. 물체의 의미 해석은 수백 ms 이상 걸려도 되므로 서버의 큰 모델로 보낼 수 있다.

Global Module은 VFM이나 VLA로 "깨진 유리컵"처럼 물체의 종류와 상태를 구분하고, 그 결과를 Local Module이 만든 기하 정보와 합친다.

## 18.2 Local Module: Lightweight Geometry

Local Module은 로봇에 실려 실시간으로 돌며, 안전하게 움직이는 데 필요한 정보를 계산한다.

### 18.2.1 목표

- **Odometry**: 자신의 움직임 추정 — "나는 지금 어디에 있는가?"
- **Obstacle Detection**: 즉각적인 장애물 감지 — "앞에 뭔가 있다, 피해!"
- **Local Mapping**: 주변 환경의 기하학적 지도 — "내 주변 3 m 이내는 이렇게 생겼다"

**운영 예**: 아파트 복도를 지나던 배달 로봇 앞에 아이가 갑자기 뛰어나오면, Local Module은 depth 센서로 장애물을 감지하고 odometry로 위치를 추정해 제어·안전 분석에서 정한 deadline 안에 회피 경로를 계산한다. 이 단계에서는 장애물이 무엇인지 따지지 않고 부딪힐지만 판단한다. 물체가 무엇인지는 Global Module이 따로 해석한다.

### 18.2.2 특징

Local Module의 update rate와 latency deadline(응답 마감 시간)은 로봇의 속도, 제동 거리(braking distance), 제어 대역폭, 센서 주기로 정한다. 전력 한도는 고른 임베디드 보드와 power mode, 냉각 조건으로 정한다. 평균 FPS와 함께 최악의 지연(worst-case latency), 지연의 흔들림(jitter), deadline을 놓친 횟수도 실제로 실을 하드웨어에서 잰다.

### 18.2.3 기술 스택

**Classical 방법**:
- ORB-SLAM3: Feature-based Visual SLAM — 단안·스테레오·RGB-D 카메라로, IMU가 있으면 함께 써서 위치 추정 (→ 9장, 14장 참고)
- VINS-Mono: Visual-Inertial Odometry — 카메라+IMU 융합 (→ 14장 참고)
- FAST-LIO2: LiDAR-Inertial Odometry — LiDAR+IMU 융합 (→ 2장, 14장 참고)

**경량 학습 모델**:
- 경량 depth estimation — MobileNet 기반으로 압축 (→ 10장 참고)
- 압축된 segmentation 모델 — knowledge distillation 적용 (→ 10장, 11장 참고)
- TensorRT 최적화 — NVIDIA GPU에서 그래프, 커널, 정밀도를 최적화하는 도구

**Edge 배포**:

```bash
# TensorRT 최적화 예시
trtexec --onnx=model.onnx --saveEngine=model.trt --fp16 --memPoolSize=workspace:4096
```

> TensorRT는 학습된 모델을 NVIDIA GPU용 inference engine으로 바꾼다. FP16을 쓰면 메모리와 지연이 줄어드는 경우가 많지만, 얼마나 줄고 정확도가 얼마나 떨어지는지는 모델, 입력, batch, power mode, 소프트웨어 버전에 따라 다르다. 실제로 실을 Jetson에서 end-to-end 지연과 검증 지표를 함께 재 보고 쓸지 정한다.

### 18.2.4 예시 구현

```python
# Local Module 개념 코드
class LocalModule:
    def __init__(self):
        self.odometry = FastLIO2()
        self.obstacle_detector = LightweightObstacleNet()  # TensorRT

    def process(self, sensor_data):
        # 1. Odometry 업데이트 (IMU 입력 100 Hz)
        pose = self.odometry.update(sensor_data.imu, sensor_data.lidar)

        # 2. 장애물 감지 (카메라 입력 30 Hz)
        obstacles = self.obstacle_detector(sensor_data.image)

        # 3. Global Module로 키프레임 전송
        if self.is_keyframe(pose):
            self.send_to_global(sensor_data, pose)

        return pose, obstacles
```

**실행 시나리오**: 위 코드의 `process()`가 일정한 주기(tick)마다 호출된다고 하자. IMU 데이터는 100 Hz(초당 100번), 카메라 이미지는 30 Hz로 들어오니, 센서마다 콜백이 최신 측정을 버퍼에 넣고 tick마다 그 값을 꺼내 쓴다. tick마다 odometry로 현재 위치를, obstacle detector로 앞의 장애물을 계산하고, 키프레임만 Global Module에 보낸다. 모든 프레임을 보내면 네트워크 대역폭이 모자랄 수 있다.

## 18.3 Global Module: VFM-based Understanding

Global Module은 서버나 클라우드에서 돌며 물체가 무엇이고 서로 어떤 관계인지 해석한다.

### 18.3.1 목표

- **전체 지도 이해**: 공간 구조와 의미 파악 — "여기는 주방이고, 저기는 거실이다"
- **Semantic Scene Graph**: 객체 간 관계 표현 — "컵이 테이블 위에 있다"
- **Long-term Memory**: 환경 변화 추적 — "어제는 여기에 의자가 없었는데 오늘은 있다"

**실제 시나리오**: 가정용 서비스 로봇이 매일 집 안을 다니며 방 구조와 물건 위치를 기록한다. Global Module은 "거실에는 소파, TV, 테이블이 있고 주방에는 냉장고와 싱크대가 있다" 같은 의미 지도(semantic map)를 유지한다. 사용자가 "거실 테이블에 있는 리모컨 가져와"라고 하면 Scene Graph에서 리모컨의 위치를 찾아 Local Module에 waypoint를 전달한다.

### 18.3.2 특징

DINOv2와 SAM2에는 크기가 다른 model variant가 있다(DINOv2 ViT-S/14 약 21M부터 ViT-g/14 약 1.1B까지). Global Module의 하드웨어와 갱신 주기는 어떤 variant를 쓰는지, 입력 해상도와 정밀도, 처리할 장면 수, 허용 응답 시간으로 정한다. Local Module의 제어 deadline과 무관하게 돌려도 되는 작업도 있다. 사용자와 대화하거나 환경 변화를 감지하는 작업은 응답 시간 요구가 있으니 따로 시간 예산을 잡는다.

### 18.3.3 기술 스택

**Vision Foundation Models** (→ 11장 참고):
- DINOv2: Dense feature extraction — 이미지를 패치 격자로 나눠 각 패치의 feature 벡터를 만든다 (픽셀 단위로 쓰려면 보간·업샘플링이 필요)
- SAM2: promptable image/video segmentation — point·box·mask prompt로 대상 mask를 추적
- GroundingDINO: 텍스트로 지정한 물체를 검출 — "빨간 컵"이라고 쓰면 그 물체의 상자를 찾는다

**3D Understanding** (→ 11장, 13장 참고):
- Gaussian Splatting with semantic features — 빠른 3D 재구성에 의미 정보를 더한다
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

        # 2. Promptable segmentation (텍스트 어휘 기반 프롬프트는 GroundingDINO/CLIP이 만들어 준다)
        masks = self.sam.segment(image, prompts=self.get_prompts())

        # 3. 3D Scene Graph 업데이트
        self.scene_graph.update(masks, depth, pose, features)

        # 4. Gaussian Map 업데이트
        self.gaussian_map.add_keyframe(image, depth, pose, features)

    def query(self, text_prompt):
        # "Where is the red cup?" → 위치 반환
        return self.scene_graph.find(text_prompt)
```

**시나리오로 읽기**: Local Module에서 키프레임이 올 때마다 `process_keyframe()`이 호출된다. DINOv2로 이미지 feature를 뽑고 SAM2로 물체를 분리한 뒤, 그 결과를 3D Scene Graph와 Gaussian Map에 누적한다. 나중에 사용자가 "빨간 컵 어디 있어?"라고 물으면 `query()`로 검색한다. 실시간 안전은 Local Module이 맡으니 이 과정은 1초쯤 걸려도 된다.

## 18.4 두 모듈의 협업

두 모듈은 따로 돌면서 정보를 주고받는다.

### 18.4.1 Local → Global

**전송 내용**:
- 키프레임 이미지/포인트 클라우드
- 로컬 포즈
- 센서 메타데이터

**키프레임 선택 기준**:
- 이동 거리/회전량 threshold — "1 m 이동하거나 30도 회전하면 보내기"
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

**다른 시나리오 — 통신 불안정 상황**: 로봇이 지하 주차장에서 작업 중인데 WiFi가 끊어졌다. 이 경우 Local Module만으로 동작해야 한다. Odometry로 위치를 추정하고, 장애물을 피하면서 마지막으로 받은 waypoint까지 이동한다. WiFi가 복구되면 그동안의 데이터를 Global에 한꺼번에 보내고, 업데이트된 계획을 받는다. 이렇게 기능을 조금씩 줄여 가며 버티는 것을 graceful degradation이라고 하며, 실제로 쓰는 로봇에는 꼭 필요하다.

### 18.4.3 통신 및 동기화

**통신 방식**:
- ROS2 DDS: 로컬 네트워크 (같은 건물 안)
- WebSocket: 클라우드 연결 (원격 서버)
- 5G/WiFi: 모바일 로봇 (실외 환경)

**동기화 전략**: 연속 스트리밍 대신 키프레임 단위로 보내 대역폭을 줄이고, Global 응답은 비동기로 받아 Local이 계속 돌게 한다. 자주 방문하는 영역은 캐싱해 중복 전송을 피한다.

## 18.5 연구 과제 예시

아래는 우리 연구실에서 진행하고 있거나 앞으로 다룰 만한 주제다. 과제마다 먼저 읽을 장을 적어 두었다.

### Local Module 연구

1. **더 가벼운 SLAM**
    - 신경망 기반 경량 VO — 기존 VO를 대체하되 Jetson에서 동작하도록 설계
    - 이벤트 카메라 활용 — 전력을 적게 쓰고 반응이 빠른 센서로 어둡거나 움직임이 빠른 환경에서 SLAM
    - 하드웨어 가속(FPGA) — SLAM의 핵심 연산을 전용 하드웨어로 구현
    - 선행 학습: 9장(카메라 모델), 14장(Visual Odometry, SLAM) 필수, 3장(최적화) 권장
2. **효율적 장애물 인식**
    - Depth-only obstacle detection — RGB 없이 depth 정보만으로 장애물 감지
    - 시간적 일관성 — 물체가 프레임마다 나타났다 사라지는 깜빡임을 억제
    - Uncertainty-aware — "이게 장애물인지 확실하지 않다"는 불확실성 정보도 회피 결정에 반영
    - 선행 학습: 10장(Depth Estimation, Object Detection) 필수, 3장(좌표 변환) 중요
3. **센서 융합 최적화**
    - Tight coupling 경량화 — IMU, 카메라, LiDAR 측정을 한 최적화에서 함께 쓰면서 계산량은 줄이기
    - 센서 드롭아웃 대응 — 센서 하나가 고장나도 계속 동작
    - 선행 학습: 2장(센서), 14장(Visual Odometry), 3장(최적화) 필수

### Global Module 연구

1. **VFM의 3D 확장**
    - DINOv2 features in 3D — 2D feature를 3D 공간에 올려서 활용
    - Semantic Gaussian Splatting — 3D 재구성에 의미 정보를 같이 넣기
    - 3D scene understanding — 공간이 어떻게 생겼고 무엇이 있는지 이해하기
    - 선행 학습: 10장(Depth), 13장(3D 표현), 11장(VFM) 필수, 9장(카메라 모델) 기본
2. **VLA 통합**
    - Open-vocabulary manipulation — "저 빨간 거 집어" 같은 자연어 명령으로 로봇팔 제어
    - 언어 기반 내비게이션 — 자연어 명령에 따라 이동
    - 상황 인식 행동 — "아이가 있으니 천천히"처럼 맥락을 행동 제약으로 변환
    - 선행 학습: 11장(VFM 활용), 12장(VLA) 필수, 10장(Detection)도 알면 좋다
3. **Scalability**: 아파트 단지·캠퍼스 전체를 단일 지도로 표현하고, 수 GB짜리 지도를 압축·갱신하며, 여러 로봇이 함께 만든 지도를 공유하는 문제다. 선행 학습: 14장(SLAM), 3장(최적화), 11장(VFM) 필수.

### Integration 연구

1. **효율적 통신**: 무엇을 언제 보낼지 정해야 한다. 모든 프레임을 보내면 대역폭을 소진하고, 키프레임만 보내면 Global이 환경 변화를 놓칠 수 있다. 5G가 끊기거나 WiFi가 느릴 때 어떤 기능부터 줄일지도 함께 정한다. 선행 학습: 14장(SLAM, 키프레임 선택), Local/Global 모듈 이해.
2. **Fallback 전략**
    - 통신 끊김 시 Local-only 동작과 graceful degradation — §18.4.2의 통신 불안정 시나리오 참고
    - 선행 학습: 시스템 전체를 알아야 한다. 3~14장은 읽고 오자
3. **일관성 유지**: Local이 "여기 빈 공간"이라 보고, Global이 "거기 의자 있음"이라 기억하면 로봇은 어느 쪽을 믿어야 할지 모른다. 두 지도를 동기화하고, 의미 정보의 근거 없는 변동("저건 의자"가 근거 없이 "테이블"로 바뀌는 일)을 억제하되 새 관측에 따른 갱신은 허용하는 문제다. 선행 학습: 3장(최적화), 14장(SLAM, 맵 관리) 필수.

## 18.6 Motivation과 Novelty를 가르는 질문

Motivation은 그 문제를 왜 풀어야 하는지 밝힌다. Novelty는 기존 방법에서 무엇을 어떻게 바꿨는지 밝힌다. 첫 논문을 쓸 때는 둘을 섞기 쉽다. Novelty를 쓸 때는 모듈이 왜 필요한지와 왜 그 형태인지까지 설명해야 한다.

아래 논문으로 문제 제기와 설계 기여를 나눠 보자.

### Case 1 — ORB-SLAM2 (Mur-Artal & Tardós 2017)

- **Motivation**: 단안용 ORB-SLAM의 map reuse·loop closing·relocalization 구조를 stereo와 RGB-D 입력에도 적용한다.
- **직접적 확장**: 입력 modality마다 별도의 SLAM 시스템을 만든다.
- **논문의 설계**: 세 modality가 tracking·local mapping·loop closing의 시스템 구조와 ORB 특징을 공유하되, stereo는 좌우 영상의 매칭으로 depth를 얻고, RGB-D는 측정한 depth를 가상의 오른쪽 영상 좌표(virtual right coordinate)로 바꿔 stereo 관측과 같은 형태로 만든다. Bundle adjustment는 이 stereo 관측과 depth가 없는 monocular 관측을 함께 써서 metric scale로 궤적을 추정한다.
- **설계 원리**: 공통 시스템 구조는 유지하고, modality별 차이를 관측 생성과 bundle-adjustment 잔차에 둔다.

원 논문은 [*ORB-SLAM2: An Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras*](https://doi.org/10.1109/TRO.2017.2705103)다. 2015년의 ORB-SLAM은 단안 카메라만 지원했고, 세 입력을 한 시스템에서 처리한 것은 ORB-SLAM2부터다.

### Case 2 — 3D Gaussian Splatting (Kerbl et al. 2023)

- **Motivation**: NeRF는 너무 느리므로 빠르게 만들어야 한다.
- **직접적 확장**: NeRF에 sparse sampling · pruning · distillation 같은 가속 모듈을 더한다.
- **논문의 설계**: ray-marching의 탐색 비용을 병목으로 보고, 표현을 명시적 primitive인 3D Gaussian으로 바꾼다. Primitive는 바로 rasterization할 수 있게 설계한다.
- **설계 원리**: 느린 원인을 공간을 표현하는 방식과 렌더링 방식에서 찾는다.

### Case 3 — DUSt3R (Wang et al. 2024)

기존 SfM은 camera intrinsics가 필요하고 단계별 오차에 민감하다. Matching이나 triangulation 한 단계만 신경망으로 교체할 수도 있지만, Wang et al.은 출력 형식 자체를 바꿨다. DUSt3R는 두 view를 받아 공통 좌표계의 pointmap(픽셀마다 3D 점을 대응시킨 지도)을 바로 예측한다. Camera intrinsics와 correspondence는 미리 넣어 주지 않고 이 pointmap에서 계산해 낸다. DUSt3R의 기여는 단계별로 나뉜 SfM을 pointmap 예측 문제 하나로 다시 정의한 데 있다.

### 세 논문이 공유하는 설계 질문

세 논문은 모두 *왜 이 모듈이 이 형태여야 하는가*를 묻고, 인터페이스·표현·출력 형식을 다시 정해서 답했다.

논문에서 motivation과 method를 전개하는 방법은 [「연구노트」 Ch.23 — 서론은 어떻게 좁혀지는가](../research-notes/guide.html#chapter-23)와 [Ch.25 — 방법 섹션](../research-notes/guide.html#chapter-25)에서 자세히 다룬다.
