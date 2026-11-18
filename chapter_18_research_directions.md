## F. 연구실 연구 방향

지금까지 다룬 기하·인식 기술과 개발 환경을 실제 연구실 프로젝트로 모을 때 어떤 그림이 나올까? 우리 연구실은 Spatial AI 시스템을 역할이 뚜렷하게 다른 두 모듈로 나누어 설계하고, 앞서 배운 기하학과 딥러닝 기술을 각 모듈의 제약 조건에 맞게 배치한다. 전체 구조는 로봇 본체에 싣는 Local Module과 외부 서버에서 돌리는 Global Module로 나뉜다.

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

#### 두 모듈로 나누는 이유

로봇에 싣는 온보드 컴퓨터 하나로 모든 인식과 추론을 떠안기에는 무게, 전력, 응답 시간의 장벽이 너무 높다. 먼저 물리적 제약부터 보자. 서버용 NVIDIA A100 GPU는 카드 한 장의 최대 TDP만 해도 PCIe 40GB 모델이 250 W, PCIe 80GB가 300 W, SXM 버전이 400 W에 달해 랙(rack)형 서버에나 들어갈 수 있는 장비다. 반면 배터리로 움직이는 모바일 로봇에는 대개 Jetson Orin 같은 임베디드 보드(AGX Orin 64GB 모듈 기준 15~60 W)를 싣는다. 이 제한된 전력·연산 예산 위에서 DINOv2나 SAM의 대형 모델을 실시간으로 돌리기는 현실적으로 어렵다.

시간 제약 역시 두 갈래로 갈린다. 당장 앞을 가로막은 장애물을 피하려면 수십 ms 안에 판단이 끝나야 하므로 로봇 내부에서 곧바로 처리해야 한다. 반면 물체의 종류와 의미를 깊게 해석하는 일은 수백 ms 이상 걸려도 안전에 지장을 주지 않으므로, 네트워크 너머 서버의 큰 모델에 맡길 수 있다. 이렇게 역할을 나누면 Global Module이 VFM이나 VLA로 "깨진 유리컵"처럼 물체의 정체와 상태를 구분해 내고, 그 의미 정보를 Local Module이 실시간으로 쌓은 기하 지도와 결합해 정교한 행동을 이끌어 낼 수 있다.

### F.1 Local Module: Lightweight Geometry

Local Module은 로봇 본체에 실려 매 순간 멈추지 않고 돌아가며, 로봇이 주변과 부딪히지 않고 움직이는 데 꼭 필요한 기하 정보를 계산한다.

#### F.1.1 목표

Local Module이 책임지는 핵심 기능은 크게 세 가지다.

- **Odometry**: 자신의 움직임 추정 — "나는 지금 어디에 있는가?"
- **Obstacle Detection**: 즉각적인 장애물 감지 — "앞에 뭔가 있다, 피해!"
- **Local Mapping**: 주변 환경의 기하학적 지도 — "내 주변 3 m 이내는 이렇게 생겼다"

**운영 예**: 아파트 복도를 지나던 배달 로봇 앞에 아이가 갑자기 뛰어나왔다고 가정해 보자. 이때 Local Module은 depth 센서로 장애물을 포착하고 odometry로 현재 위치를 계산해, 제어·안전 설계에서 정해 둔 deadline 안에 회피 경로를 내놓는다. 이 긴박한 단계에서는 앞을 막은 대상이 '아이'인지 '상자'인지 따지지 않고, 오직 충돌 여부와 피할 공간만 판단한다. 물체의 의미 해석은 Global Module이 별도 주기로 맡는다.

#### F.1.2 특징

Local Module의 update rate와 latency deadline(응답 마감 시간)은 로봇의 최대 이동 속도, 제동 거리(braking distance), 제어 대역폭, 센서 출력 주기에 맞춰 엄격하게 정한다. 소비 전력 상한은 탑재한 임베디드 보드와 power mode, 방열 설계에 따라 결정된다. 따라서 개발 과정에서는 평균 FPS만 보고 안심해서는 안 되며, 실제 로봇에 올릴 보드 위에서 최악의 지연(worst-case latency), 지연의 흔들림(jitter), deadline 초과 횟수까지 함께 측정해야 한다.

#### F.1.3 기술 스택

온보드 제약을 지키면서 실시간 성능을 내기 위해 고전 기하학 기반 알고리즘과 경량화된 딥러닝 모델을 적재적소에 섞어 쓴다.

**Classical 방법**:
- ORB-SLAM3: Feature-based Visual SLAM — 단안·스테레오·RGB-D 카메라를 지원하며, IMU가 있으면 함께 결합해 위치를 추정한다 (→ 9장, 14장 참고)
- VINS-Mono: Visual-Inertial Odometry — 카메라와 IMU를 결합한 주행거리 추정 (→ 14장 참고)
- FAST-LIO2: LiDAR-Inertial Odometry — LiDAR와 IMU를 결합한 고속 주행거리 추정 (→ 2장, 14장 참고)

**경량 학습 모델**:
- 경량 depth estimation — MobileNet 계열 백본으로 압축 (→ 10장 참고)
- 압축된 segmentation 모델 — knowledge distillation 기법 적용 (→ 10장, 11장 참고)
- TensorRT 최적화 — NVIDIA GPU 환경에 맞춰 연산 그래프, 커널, 수치 정밀도를 최적화하는 도구

**Edge 배포**: 학습을 마친 신경망을 Jetson 보드에 올릴 때는 보통 ONNX를 거쳐 TensorRT 엔진으로 변환한다.

```bash
# TensorRT 최적화 예시
trtexec --onnx=model.onnx --saveEngine=model.trt --fp16 --memPoolSize=workspace:4096
```

> TensorRT는 학습된 모델을 NVIDIA GPU 전용 inference engine으로 컴파일한다. `--fp16` 옵션을 켜면 메모리 점유율과 추론 지연이 눈에 띄게 줄어드는 편이지만, 실제 절감 폭과 정확도 변화는 모델 구조, 입력 해상도, batch 크기, power mode, 소프트웨어 버전에 따라 크게 달라진다. 반드시 실제 탑재할 Jetson 보드에서 end-to-end 지연 시간과 검증 지표를 함께 재 본 뒤에 적용 여부를 정한다.

#### F.1.4 예시 구현

실제 온보드 파이프라인이 어떻게 주기를 나눠 도는지 개념 코드로 살펴보자.

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

**실행 시나리오**: 위 코드의 `process()`가 정해진 제어 주기(tick)마다 호출된다고 보자. IMU 측정값은 100 Hz(초당 100번), 카메라 영상은 30 Hz로 주기가 다르게 들어오므로, 각 센서 콜백이 최신 데이터를 버퍼에 갱신해 두고 tick마다 값을 꺼내 쓰는 구조가 된다. 매 tick마다 odometry로 현재 위치를 계산하고 obstacle detector로 전방 장애물을 확인하되, Global Module로는 모든 프레임을 보내지 않고 키프레임만 추려 보낸다. 모든 원시 프레임을 네트워크로 실어 나르면 무선 대역폭이 금세 포화되기 때문이다.

### F.2 Global Module: VFM-based Understanding

Global Module은 넉넉한 연산 자원을 갖춘 서버나 클라우드 위에서 돌며, 공간 속 물체가 무엇이고 서로 어떤 맥락으로 놓여 있는지 깊이 해석한다.

#### F.2.1 목표

Global Module은 단순한 점군 지도를 넘어 로봇이 태스크를 추론할 수 있는 의미 기반 지도를 유지한다.

- **전체 지도 이해**: 공간 구조와 방의 용도 파악 — "여기는 주방이고, 저기는 거실이다"
- **Semantic Scene Graph**: 객체 간 위상·의미 관계 표현 — "컵이 테이블 위에 있다"
- **Long-term Memory**: 시간에 따른 환경 변화 추적 — "어제는 여기에 의자가 없었는데 오늘은 있다"

**실제 시나리오**: 가정용 서비스 로봇이 매일 집 안을 순찰하며 방의 구조와 물건 배치를 기록한다. Global Module은 "거실에는 소파·TV·테이블이 있고, 주방에는 냉장고와 싱크대가 있다"는 형태의 의미 지도(semantic map)를 계속 최신 상태로 유지한다. 사용자가 "거실 테이블에 있는 리모컨 가져와"라고 말하면, Scene Graph에서 리모컨의 3D 좌표를 찾아내 Local Module이 따라갈 waypoint를 내려준다.

#### F.2.2 특징

DINOv2나 SAM2 같은 Vision Foundation Model은 파라미터 규모가 다른 여러 variant를 제공한다(예컨대 DINOv2는 약 21M 파라미터의 ViT-S/14부터 약 1.1B 파라미터의 ViT-g/14까지 나뉜다). Global Module의 서버 사양과 갱신 주기는 어떤 크기의 variant를 고르는지, 입력 해상도와 정밀도는 얼마로 두는지, 동시에 처리할 장면 수와 허용 응답 시간이 얼마인지에 따라 설계한다. Local Module의 실시간 제어 deadline과 직접 얽히지 않는 백그라운드 매핑 작업도 있지만, 사용자와 자연어로 대화하거나 갑작스러운 환경 변화를 반영해야 하는 경로는 체감 응답 시간이 중요하므로 별도로 시간 예산을 배정한다.

#### F.2.3 기술 스택

Global Module은 최신 비전 파운데이션 모델, 3D 표현 기술, 언어 모델 추론을 하나의 파이프라인으로 엮어 구성한다.

**Vision Foundation Models** (→ 11장 참고):
- DINOv2: Dense feature extraction — 이미지를 패치 격자로 나눠 각 패치의 고차원 feature 벡터를 뽑는다 (픽셀 단위로 쓰려면 보간이나 업샘플링이 뒤따라야 한다)
- SAM2: promptable image/video segmentation — point·box·mask prompt를 주면 해당 대상의 mask를 이미지와 비디오 전반에서 추적한다
- GroundingDINO: 텍스트 프롬프트 기반 객체 검출 — "빨간 컵"이라고 입력하면 이미지에서 해당 물체의 경계 상자를 찾아낸다

**3D Understanding** (→ 11장, 13장 참고):
- Gaussian Splatting with semantic features — 빠른 3D 장면 재구성에 의미 정보 채널을 함께 심는다
- 3D Scene Graph 구축 — 공간 속 객체들의 위치와 관계를 그래프 구조로 엮는다
- VFM features의 3D lifting — 2D 이미지 평면에서 뽑은 의미 feature를 3D 공간 좌표 위로 끌어올린다

**Language Integration** (→ 11장, 12장 참고):
- CLIP features for open-vocabulary — 미리 라벨을 학습하지 않은 "본 적 없는 물체"도 자연어 텍스트로 검색할 수 있게 한다
- LLM for scene reasoning — "이 방은 어떤 용도로 쓰이는 공간일까?" 같은 고수준 맥락을 추론한다
- VLA for action planning — "컵을 집으려면 로봇 팔을 어떤 궤적으로 움직여야 할까?" 같은 행동 계획으로 연결한다

#### F.2.4 예시 구현

Local Module이 보낸 키프레임이 서버에 도착했을 때 Global Module이 이를 의미 지도와 Scene Graph로 녹여 내는 흐름은 다음과 같다.

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

**시나리오로 읽기**: Local Module에서 새 키프레임이 올라올 때마다 `process_keyframe()`이 실행된다. DINOv2로 밀집된 시각 feature를 추출하고 SAM2로 물체별 마스크를 분리한 뒤, 깊이(depth)와 포즈 정보를 얹어 3D Scene Graph와 Gaussian Map에 차곡차곡 쌓는다. 이후 사용자가 "빨간 컵 어디 있어?"라고 물으면 `query()`가 누적된 Scene Graph를 뒤져 좌표를 꺼내 준다. 즉각적인 충돌 회피는 이미 Local Module이 도맡고 있으므로, 이 무거운 추론 파이프라인은 1초 안팎이 걸려도 시스템 전체의 안전을 해치지 않는다.

### F.3 두 모듈의 협업

두 모듈은 각자 다른 주기와 하드웨어 위에서 독립적으로 돌면서, 꼭 필요한 시점에만 요약된 정보를 주고받으며 하나의 시스템처럼 맞물린다.

#### F.3.1 Local → Global

로봇에서 서버로 올려보낼 때는 통신 대역폭을 아끼기 위해 모든 센서를 흘려보내지 않고 선별된 묶음만 보낸다.

**전송 내용**:
- 키프레임 이미지 및 포인트 클라우드
- 온보드에서 추정한 로컬 포즈
- 타임스탬프 등 센서 메타데이터

**키프레임 선택 기준**:
- 이동 거리·회전량 threshold — "1 m 이상 이동했거나 30도 이상 회전했을 때만 보내기"
- 장면 변화 감지 — "새로운 방에 진입해 배경이 크게 바뀌었다"
- 정보량 (특징점 수, 커버리지) — "이번 프레임이 기존 지도의 빈틈을 많이 채워 준다"

#### F.3.2 Global → Local

반대로 서버에서 로봇으로 내려보낼 때는 당장 경로 추종과 작업 수행에 필요한 요약 정보만 전달한다.

**전송 내용**:
- 사전 지도 (필요 영역만 잘라 전달) — "주방 근처의 고정 장애물 배치"
- Semantic 정보 (객체 위치, 클래스) — "테이블은 여기, 의자는 저기"
- 네비게이션 waypoints — "목적지까지 따라갈 경유점 목록"

**예시 시나리오**: 사용자의 자연어 명령 하나가 두 모듈 사이를 오가며 실제 이동으로 이어지는 과정을 단계별로 따라가 보자.

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

**다른 시나리오 — 통신 불안정 상황**: 로봇이 지하 주차장에서 작업하던 중 WiFi 연결이 끊겼다고 하자. 이 상황에서도 로봇이 그 자리에 얼어붙어서는 안 되므로 Local Module 단독으로 버텨야 한다. 온보드 odometry로 자기 위치를 계속 추정하고, 장애물을 피해 가며 마지막으로 전달받은 waypoint까지 이동한다. 이후 WiFi가 복구되면 그동안 쌓아 둔 키프레임과 관측 데이터를 Global Module에 일괄 전송하고, 갱신된 전역 계획을 다시 내려받는다. 이처럼 외부 연결이나 상위 기능이 끊겼을 때 핵심 안전 기능을 유지한 채 성능을 단계적으로 낮춰 버티는 설계를 graceful degradation이라 하며, 실제 현장에 투입되는 로봇 시스템에는 필수 조건이다.

#### F.3.3 통신 및 동기화

두 모듈이 놓인 물리적 환경에 따라 통신 프로토콜과 동기화 전략도 달라진다.

**통신 방식**:
- ROS2 DDS: 같은 건물이나 로컬 네트워크 안에서 저지연으로 통신할 때
- WebSocket: 원격 서버나 클라우드 환경과 연결할 때
- 5G/WiFi: 넓은 실외 공간을 이동하는 모바일 로봇을 다룰 때

**동기화 전략**: 연속 영상 스트리밍 대신 키프레임 단위로만 데이터를 보내 대역폭 점유를 낮추고, Global Module의 응답은 비동기로 수신해 서버 추론을 기다리느라 Local 제어 루프가 멈추지 않게 한다. 자주 드나드는 구역의 지도 정보는 로봇 측에 캐싱해 두어 같은 데이터를 거듭 주고받는 낭비를 막는다.

### F.4 연구 과제 예시

이 구조 위에서 우리 연구실이 현재 파고들고 있거나 앞으로 도전해 볼 만한 구체적인 연구 주제를 세 영역으로 나누어 정리했다. 각 과제마다 먼저 읽어 두면 좋을 본문 장을 함께 표시해 두었으니 관심 있는 주제를 고르는 나침반으로 삼아 보자.

#### Local Module 연구

1. **더 가벼운 SLAM**
    - 신경망 기반 경량 VO — 기존 기하 기반 VO를 보완·대체하되 Jetson 급 보드에서 실시간으로 동작하도록 설계
    - 이벤트 카메라 활용 — 전력 소모가 적고 마이크로초 단위로 반응하는 센서를 이용해 어둡거나 회전이 빠른 환경에서 강인한 SLAM 구현
    - 하드웨어 가속(FPGA) — SLAM의 반복 연산 병목을 전용 하드웨어 파이프라인으로 가속
    - 선행 학습: 9장(카메라 모델), 14장(Visual Odometry, SLAM) 필수, 3장(최적화) 권장
2. **효율적 장애물 인식**
    - Depth-only obstacle detection — 조명 변화에 민감한 RGB 없이 depth 정보만으로 빠르고 안정적인 장애물 감지
    - 시간적 일관성 — 물체가 프레임마다 나타났다 사라지는 깜빡임(flickering)을 억제해 안정적인 회피 유도
    - Uncertainty-aware — "앞의 물체가 진짜 장애물인지 불확실하다"는 예측 불확실성까지 회피 비용 함수에 반영
    - 선행 학습: 10장(Depth Estimation, Object Detection) 필수, 3장(좌표 변환) 중요
3. **센서 융합 최적화**
    - Tight coupling 경량화 — IMU, 카메라, LiDAR 측정을 하나의 최적화 그래프에 촘촘히 묶으면서도 실시간 한도 안에 들도록 계산량 절감
    - 센서 드롭아웃 대응 — 주행 중 센서 하나가 가려지거나 고장 나도 나머지 센서로 추정을 이어 가는 강인성 확보
    - 선행 학습: 2장(센서), 14장(Visual Odometry), 3장(최적화) 필수

#### Global Module 연구

1. **VFM의 3D 확장**
    - DINOv2 features in 3D — 2D 이미지에서 뽑은 조밀한 feature를 3D 공간 표현 위로 정합해 활용
    - Semantic Gaussian Splatting — 고속 3D 재구성 위에 물체별 의미 정보와 개방형 어휘 검색 기능을 결합
    - 3D scene understanding — 공간이 어떤 기하 구조로 이루어졌고 각 물체가 무슨 의미로 놓여 있는지 통합 이해
    - 선행 학습: 10장(Depth), 13장(3D 표현), 11장(VFM) 필수, 9장(카메라 모델) 기본
2. **VLA 통합**
    - Open-vocabulary manipulation — "저 빨간 거 집어"처럼 정해진 카테고리 밖의 자연어 지시로 로봇 팔 제어
    - 언어 기반 내비게이션 — 일상적인 문장 지시를 따라 복합 공간을 탐색·이동
    - 상황 인식 행동 — "아이가 있으니 천천히"처럼 언어와 장면 맥락을 실제 제어 제약 조건으로 변환
    - 선행 학습: 11장(VFM 활용), 12장(VLA) 필수, 10장(Detection)도 알면 좋다
3. **Scalability**: 방 하나를 넘어 아파트 단지나 캠퍼스 전체를 하나의 일관된 지도로 표현하고, 수 GB에 달하는 지도를 효율적으로 압축·갱신하며, 여러 대의 로봇이 각자 모은 지도를 공유하고 합치는 문제다. 선행 학습: 14장(SLAM), 3장(최적화), 11장(VFM) 필수.

#### Integration 연구

1. **효율적 통신**: 제한된 무선 대역폭 속에서 언제 무엇을 보낼지 결정하는 문제다. 모든 프레임을 보내면 망이 막히고, 키프레임을 너무 성기게 보내면 Global 지도가 현실의 변화를 놓친다. 5G가 끊기거나 WiFi 속도가 뚝 떨어졌을 때 어떤 데이터와 기능부터 줄일지 우선순위를 정하는 정책까지 함께 다룬다. 선행 학습: 14장(SLAM, 키프레임 선택), Local/Global 모듈 이해.
2. **Fallback 전략**
    - 통신 단절 시 Local-only 자율 주행과 graceful degradation 설계 — §F.3.2의 통신 불안정 시나리오 참고
    - 선행 학습: 시스템 전체의 맞물림을 이해해야 하므로 3~14장을 두루 읽고 시작한다.
3. **일관성 유지**: Local Module은 당장 라이다로 보고 "여기는 빈 공간"이라 판단했는데, Global Module은 과거 기억을 바탕으로 "거기 의자 있음"이라 알려 주면 로봇은 어느 쪽을 믿어야 할지 혼란에 빠진다. 두 지도의 시간·공간 어긋남을 동기화하고, 의미 라벨이 근거 없이 흔들리는 현상("저건 의자"가 갑자기 "테이블"로 뒤집히는 문제)을 억제하면서도 실제로 물체가 치워졌을 때는 제때 지도를 고치는 균형 잡힌 갱신 문제를 다룬다. 선행 학습: 3장(최적화), 14장(SLAM, 맵 관리) 필수.

### F.5 Motivation과 Novelty를 가르는 질문

연구 주제를 골라 첫 논문을 쓰기 시작할 때 가장 많이 섞어 쓰는 개념이 motivation과 novelty다. Motivation은 "이 문제를 왜 굳이 풀어야 하는가"라는 출발점의 필요성을 밝히고, novelty는 "그 문제를 풀기 위해 기존 방법의 무엇을 어떻게 바꿨는가"라는 설계상의 기여를 밝힌다. 둘을 뚜렷이 구분하면서도, 내가 제안한 모듈이 왜 꼭 필요한지(motivation)와 왜 하필 그런 형태여야 하는지(novelty)가 한 줄기로 이어지게 설명해야 설득력 있는 논문이 된다.

실제 대표 논문 세 편을 놓고 보면 문제 제기와 설계 기여가 어떻게 갈리고 맞물리는지 선명하게 드러난다.

#### Case 1 — ORB-SLAM2 (Mur-Artal & Tardós 2017)

- **Motivation**: 단안 카메라용 ORB-SLAM이 보여 준 map reuse·loop closing·relocalization 구조를 stereo 카메라와 RGB-D 센서 입력에서도 그대로 쓰고 싶다.
- **직접적 확장**: 가장 손쉬운 타협은 입력 modality마다 별도의 SLAM 파이프라인을 따로 두는 방식이다.
- **논문의 설계**: 세 modality 모두 tracking·local mapping·loop closing이라는 뼈대와 ORB 특징 표현을 그대로 공유하되, stereo는 좌우 영상의 시차 매칭으로 깊이를 얻고, RGB-D는 측정된 depth를 가상의 오른쪽 영상 좌표(virtual right coordinate)로 환산해 stereo 관측과 똑같은 형식으로 통일한다. 이렇게 맞춰 놓으면 Bundle adjustment가 stereo 관측과 depth 없는 monocular 관측을 한 식 안에서 함께 최적화해 정확한 metric scale 궤적을 뽑아낼 수 있다.
- **설계 원리**: 공통 시스템 아키텍처는 건드리지 않고, modality별 차이만 관측 생성 규칙과 bundle-adjustment 잔차 항에 가두었다.

원 논문은 [*ORB-SLAM2: An Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras*](https://doi.org/10.1109/TRO.2017.2705103)다. 2015년에 발표된 초기 ORB-SLAM은 단안 카메라만 지원했고, 세 센서 입력을 단일 시스템으로 통합한 것은 ORB-SLAM2부터다.

#### Case 2 — 3D Gaussian Splatting (Kerbl et al. 2023)

- **Motivation**: NeRF는 화질이 뛰어나지만 렌더링이 너무 느려서 실시간 활용이 어렵다.
- **직접적 확장**: 기존 NeRF의 볼륨 렌더링 틀을 유지한 채 sparse sampling·pruning·distillation 같은 가속 기법을 덧붙인다.
- **논문의 설계**: 느린 진짜 병목이 ray-marching 기반 공간 탐색 비용 자체에 있다고 보고, 장면을 표현하는 기본 단위를 암묵적 신경망에서 명시적 primitive인 3D Gaussian으로 바꾼다. 각 primitive가 GPU에서 곧바로 고속 rasterization될 수 있게 표현과 최적화를 함께 설계했다.
- **설계 원리**: 속도 저하의 원인을 국소적인 샘플링 효율이 아니라 공간을 표현하는 자료구조와 렌더링 방식의 근본에서 찾았다.

#### Case 3 — DUSt3R (Wang et al. 2024)

기존 Structure-from-Motion(SfM) 파이프라인은 사전에 캘리브레이션된 camera intrinsics가 필요하고, 특징점 추출·매칭·삼각측량으로 이어지는 앞 단계의 작은 오차가 뒤 단계로 번지며 무너지기 쉽다. 여기서 매칭이나 삼각측량 한 조각만 딥러닝으로 갈아 끼울 수도 있었지만, Wang et al.은 아예 문제의 출력 형식 자체를 뒤바꾸었다. DUSt3R는 두 시점의 이미지만 입력받아 공통 좌표계 위에 놓인 3D pointmap(각 픽셀에 대응하는 3D 좌표를 직접 담은 지도)을 한 번에 예측한다. camera intrinsics와 픽셀 간 correspondence는 미리 넣어 주는 입력값이 아니라, 예측된 pointmap으로부터 뒤늦게 역산해 낸다. 결국 DUSt3R의 핵심 novelty는 여러 단계로 쪼개져 있던 SfM을 단일 pointmap 회귀 문제로 재정의한 데 있다.

#### 세 논문이 공유하는 설계 질문

ORB-SLAM2, 3D Gaussian Splatting, DUSt3R는 다루는 문제도 시대도 다르지만, 모두 *"왜 이 모듈이 하필 이 형태여야 하는가"*를 끝까지 물었고, 그 답을 작은 트릭 추가가 아니라 인터페이스·공간 표현·출력 형식의 재설계에서 찾아냈다.

논문 본문에서 이러한 motivation과 method를 설득력 있게 엮어 쓰는 구체적인 서술 전략은 [「연구노트」 23장 서론](../research-notes/guide.html#chapter-23)과 [25장 방법 섹션](../research-notes/guide.html#chapter-25)에서 자세히 다룬다.
