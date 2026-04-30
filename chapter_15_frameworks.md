# Ch.15 — 로봇 프레임워크 (Robot Frameworks)


로봇 소프트웨어를 처음 짜는 사람이 가장 당황하는 순간은, 센서 드라이버·경로 계획·모터 제어를 하나의 프로그램 안에서 전부 돌려야 한다는 걸 깨달을 때이다. 프레임워크는 이 문제를 구조적으로 해결해 준다. 프레임워크 없이 로봇을 만들면 "센서 데이터 받기 → 판단하기 → 모터 명령 보내기"만 해도 스레드 관리·메시지 직렬화·좌표 변환을 전부 직접 구현해야 하는데, 이걸 모르면 첫 프로젝트에서 벽에 부딪힌다. 여기서는 사실상 업계 표준인 ROS를 중심으로, 시뮬레이터와 주변 도구까지 폭넓게 본다.

## 15.1 ROS (Robot Operating System)

ROS는 로봇 소프트웨어 개발을 위한 오픈소스 프레임워크이다. 운영체제가 아닌 **미들웨어**로, 프로세스 간 통신, 패키지 관리, 도구 등을 제공한다.

로봇은 카메라·LiDAR·모터·제어기 같은 수십 가지 모듈이 동시에 돌아가야 하는데, 이 모듈들이 서로 데이터를 주고받는 방법을 통일해 주는 것이 ROS의 핵심 역할이다. ROS 없이 이 작업을 하면 소켓 프로그래밍부터 시작해야 하는데, 연구 시간의 대부분이 인프라 구축에 날아간다.

### 15.1.1 ROS1 vs ROS2

| 특징 | ROS1 | ROS2 |
| --- | --- | --- |
| 출시 | 2007 | 2017 |
| 통신 | Custom (TCPROS) | DDS 기반 |
| 실시간 | 미지원 | 지원 |
| 보안 | 없음 | SROS2 |
| Multi-robot | 어려움 | 쉬움 |
| Master | 필요 (roscore) | 불필요 |
| Python | 2/3 | 3 only |

**현재 권장**: ROS2 (Humble)

하지만 많은 패키지가 아직 ROS1만 지원하므로 상황에 따라 선택.

**ROS1 → ROS2 마이그레이션 현황**: 2024년을 기점으로 ROS1 Noetic의 공식 지원이 종료(EOL)되었다. 새 프로젝트라면 특별한 이유가 없는 한 ROS2로 시작한다. 기존 ROS1 패키지를 써야 한다면 `ros1_bridge`로 두 노드를 동시에 운용할 수 있다. Nav2, MoveIt2 등 핵심 패키지의 ROS2 포팅이 완료된 상태이므로, 대부분의 로봇 개발에서는 ROS2만으로 충분하다.

> **추천 자료**
> - [ROS2 공식 튜토리얼](https://docs.ros.org/en/humble/Tutorials.html) — ROS2 Humble 기준 공식 단계별 가이드. 처음이라면 "Beginner: CLI tools"부터 시작
> - [The Construct - ROS2 Basics](https://www.youtube.com/@TheConstruct) — ROS 전문 교육 채널. 시뮬레이터 내에서 실습 가능
> - [ROS1 to ROS2 Migration Guide](https://docs.ros.org/en/humble/How-To-Guides/Migrating-from-ROS1.html) — 기존 ROS1 코드 이전 공식 가이드

### 15.1.2 핵심 개념

이 개념들은 ROS의 뼈대이다. Topic, Service, Action의 차이를 정확히 모르면 "센서 데이터를 어떤 방식으로 보내야 하지?" 하는 질문에서 매번 막히게 된다. 각각이 언제 적합한지 감을 잡는 것이 중요하다.

**Node (노드)**:
- 실행 가능한 프로세스
- 단일 목적 (sensor driver, controller 등)

**Topic (토픽)**:
- 비동기 메시지 스트림
- Publisher/Subscriber 패턴
- 예: 센서 데이터, 명령

```python
# ROS2 Publisher 예시
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello, World!'
        self.publisher_.publish(msg)
```

**Service (서비스)**:
- 동기 요청/응답
- 일회성 작업에 적합
- 예: 설정 변경, 상태 조회

**Action (액션)**:
- 비동기 목표 지향 작업
- Feedback 제공
- 취소 가능
- 예: 네비게이션, 조작

정리하면, Topic은 카메라 영상처럼 계속 흘러가는 데이터에, Service는 "지금 배터리 잔량 알려줘"처럼 한 번 물어보고 답 받는 상황에, Action은 "저기까지 가"처럼 시간이 걸리는 작업에 쓰인다. 이 세 가지를 구분하지 못하면 시스템 설계에서 계속 꼬인다.

**Parameter (파라미터)**:
- 노드 설정값
- 런타임 변경 가능

> **⚠ AI 에이전트 주의**: AI에게 ROS2 코드를 짜달라고 할 때는 QoS 설정을 명시하라. AI는 기본적으로 QoS를 빠뜨리고, 센서 토픽이 조용히 안 들어와서 한참 헤매게 된다.

> **추천 자료**
> - [ROS2 Concepts — Understanding nodes, topics, services, actions](https://docs.ros.org/en/humble/Concepts.html) — 공식 개념 문서
> - [The Construct - ROS2 Topics vs Services vs Actions](https://www.youtube.com/@TheConstruct) — 세 가지 통신 방식 비교 영상

### 15.1.3 도구

로봇을 개발할 때 "일단 코드 짜고 로봇에 올려 보자"는 접근은 위험하다. 센서 데이터가 제대로 들어오는지, 좌표계가 맞는지 눈으로 확인할 수 있어야 디버깅 시간이 줄어든다. 아래 도구들은 ROS 개발자라면 매일 쓰게 되는 필수 유틸리티이다.

**rviz / rviz2**:
- 3D 시각화 도구
- 센서 데이터, TF, 경로 등 표시

**rqt**:
- Qt 기반 GUI 도구 모음
- rqt_graph: 노드/토픽 관계 시각화
- rqt_plot: 데이터 그래프

**rosbag / ros2 bag**:
- 데이터 녹화/재생
- 디버깅, 알고리즘 개발에 필수

이 도구들을 알아두면 실험 시간이 크게 줄어든다. 실제 로봇을 매번 돌려가며 알고리즘을 테스트하면 시간도 비용도 많이 든다. rosbag으로 한 번 녹화해 두면 같은 데이터로 몇 번이고 반복 실험이 가능하다. 재현성 측면에서 필수 도구다.

```bash
# ROS2 bag 녹화
ros2 bag record -a -o my_bag

# 재생
ros2 bag play my_bag
```

**tf2 (Transform Library)**:
- 좌표계 변환 관리
- 시간에 따른 변환 추적

로봇에는 카메라 좌표계, LiDAR 좌표계, 베이스 좌표계, 월드 좌표계 등이 동시에 존재한다. "카메라에서 본 점이 로봇 기준으로 어디인가?"를 계산하려면 좌표 변환이 필수인데, tf2가 이를 자동으로 관리해 준다. 선형대수를 배웠다면 SE(3) 변환 행렬로 이해하면 된다.

```python
# tf2 리스너 예시
from tf2_ros import Buffer, TransformListener

tf_buffer = Buffer()
tf_listener = TransformListener(tf_buffer, self)

# base_link → camera_link 변환 조회
transform = tf_buffer.lookup_transform('base_link', 'camera_link', rclpy.time.Time())
```

> **추천 자료**
> - [ROS2 tf2 Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html) — 좌표 변환 공식 튜토리얼
> - [The Construct - rviz2 Complete Guide](https://www.youtube.com/@TheConstruct) — rviz2 활용 영상
> - [ros2 bag CLI 문서](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html) — 데이터 녹화/재생 공식 가이드

### 15.1.4 주요 패키지

ROS의 진짜 힘은 커뮤니티가 만들어 놓은 패키지 생태계에 있다. 아래 패키지들은 거의 모든 로봇 프로젝트에서 한 번은 쓰게 되니, 이름이라도 기억해 두자.

| 패키지 | 용도 |
| --- | --- |
| sensor_msgs | 센서 메시지 타입 |
| geometry_msgs | 기하학 메시지 (Pose, Twist 등) |
| cv_bridge | OpenCV ↔︎ ROS 이미지 변환 |
| image_transport | 이미지 압축 전송 |
| pcl_ros | PCL ↔︎ ROS 포인트 클라우드 |
| nav2 | Navigation 스택 (ROS2) |

> **추천 자료**
> - [Nav2 Documentation](https://docs.nav2.org/) — ROS2 Navigation 스택 공식 문서
> - [ROS2 Package Index](https://index.ros.org/packages/) — ROS2 패키지 검색

## 15.2 시뮬레이션

실물 로봇으로 바로 실험하면 하드웨어가 망가지거나, 사람이 다칠 수 있다. 시뮬레이터에서 먼저 충분히 테스트하고 실물로 넘어가는 것이 안전하고 효율적이다. 특히 강화학습처럼 수만 번의 에피소드가 필요한 학습 방법론에서는 시뮬레이터 없이는 사실상 연구가 불가능하다.

최근에는 **Embodied AI** 연구가 빠르게 늘면서, 로봇이 가상 환경에서 자율적으로 학습하는 시뮬레이터의 역할도 커졌다. NVIDIA Isaac Sim, AI2-THOR, Habitat 같은 플랫폼이 이 흐름을 이끌고 있으며, 시뮬레이터에서 학습한 정책을 실제 로봇에 전이하는 Sim-to-Real Transfer가 핵심 연구 주제다.

### 15.2.1 Gazebo

Gazebo는 ROS와 가장 긴밀하게 연동되는 시뮬레이터이다. ROS 프로젝트 대부분의 시뮬레이션 데모가 Gazebo 기반으로 제공되므로, ROS를 쓰겠다면 Gazebo 사용법은 알아야 한다.

**구성 요소**:
- **SDF (Simulation Description Format)**: 환경 정의
- **URDF (Unified Robot Description Format)**: 로봇 모델

**Gazebo Classic vs Gazebo Sim (Ignition)**:
- Gazebo Sim: 새로운 버전, ROS2 권장
- 모듈화된 구조, 더 나은 확장성

```xml
<!-- URDF 예시 -->
<robot name="my_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </collision>
  </link>
</robot>
```

> **추천 자료**
> - [Gazebo Sim 공식 튜토리얼](https://gazebosim.org/docs) — Gazebo Sim(구 Ignition) 공식 가이드
> - [URDF Tutorial (ROS2)](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html) — 로봇 모델링 기초
> - [The Construct - Gazebo Sim with ROS2](https://www.youtube.com/@TheConstruct) — Gazebo + ROS2 실습 영상

### 15.2.2 NVIDIA Isaac Sim

Embodied AI 연구에서 대규모 합성 데이터 생성과 Sim-to-Real 학습에 많이 쓰인다. 사실적인 렌더링과 정확한 물리 엔진을 결합해, 시뮬레이터에서 학습한 정책이 실제 로봇에서도 잘 작동하도록 지원한다.

**특징**:
- 사실적인 그래픽 (RTX 렌더링)
- 정확한 물리 시뮬레이션 (PhysX 5)
- 합성 데이터 생성 (Domain Randomization)
- ROS2 통합

**주 용도**:
- 조작(Manipulation) 연구
- 대규모 합성 데이터 생성
- Sim-to-Real 학습

**Embodied AI 시뮬레이터 비교**: Isaac Sim 외에도 Embodied AI 연구에 많이 쓰이는 시뮬레이터들이 있다.

| 시뮬레이터 | 주 용도 | 특징 |
| --- | --- | --- |
| NVIDIA Isaac Sim | 범용 (Manipulation, Navigation) | RTX 렌더링, PhysX 5, 대규모 합성 데이터 |
| AI2-THOR | 실내 내비게이션, 물체 상호작용 | 120+ 실내 장면, 사실적 상호작용 |
| Habitat (Meta) | 시각 내비게이션, Embodied QA | 초고속 렌더링 (수천 FPS), 대규모 학습 |
| iGibson | 실내 로봇 작업 | 물리 기반 렌더링, 가정환경 |
| MuJoCo | 로봇 제어, 강화학습 | 정확한 접촉 역학, 빠른 시뮬레이션 |

> **추천 자료**
> - [NVIDIA Isaac Sim 공식 문서](https://docs.omniverse.nvidia.com/isaacsim/latest/index.html) — 설치부터 고급 활용까지
> - [AI2-THOR Documentation](https://ai2thor.allenai.org/ithor/documentation) — Embodied AI 연구용 실내 시뮬레이터
> - [Habitat Documentation](https://aihabitat.org/docs/habitat2/) — Meta의 Embodied AI 플랫폼

### 15.2.3 CARLA

자율주행 연구를 위한 시뮬레이터이다.

자율주행 논문에서 실험 환경으로 CARLA를 사용하는 경우가 매우 많다. 자율주행 쪽 연구를 하고 싶다면 CARLA 환경을 다루는 법을 익혀 두면 좋다.

**특징**:
- 도시 환경 시뮬레이션
- 다양한 날씨, 시간대
- 센서 시뮬레이션 (카메라, LiDAR, Radar)
- ROS 브릿지 제공

> **추천 자료**
> - [CARLA Documentation](https://carla.readthedocs.io/) — 공식 문서 및 Python API 레퍼런스
> - [CARLA Simulator YouTube](https://www.youtube.com/@intaborlado5265) — 시뮬레이터 활용 데모 영상

## 15.3 기타 프레임워크

ROS와 시뮬레이터 외에도, 특정 용도에 특화된 프레임워크와 라이브러리들이 있다. 이것들을 직접 만들 필요 없이 가져다 쓸 수 있다는 것이 로보틱스 생태계의 장점이다.

**Isaac ROS**:
- NVIDIA GPU 가속 ROS 패키지
- DNN 추론, Visual SLAM, 3D 인식
- Jetson 최적화

**Autoware**:
- 완전한 자율주행 스택
- 인식, 계획, 제어 포함
- ROS2 기반 (Autoware.Universe)

**ROS 외 도구**:
- **OpenCV**: 컴퓨터 비전
- **Open3D**: 3D 처리/시각화
- **Eigen**: 선형대수 (C++)
- **Sophus**: SE(3), SO(3) 연산

예를 들어 Eigen과 Sophus는 로보틱스에서 좌표 변환을 다룰 때 핵심적으로 쓰이는 라이브러리이다. 선형대수를 수업에서 배웠다면, Eigen이 그 행렬 연산을 C++ 코드로 구현한 것이라고 보면 된다. Sophus는 거기에 회전(SO(3))과 강체 변환(SE(3))을 편하게 다루는 기능을 추가한 것이다.

> **추천 자료**
> - [OpenCV Tutorials](https://docs.opencv.org/4.x/d9/df8/tutorial_root.html) — 컴퓨터 비전 기초부터 고급까지
> - [Open3D Documentation](http://www.open3d.org/docs/) — 3D 데이터 처리 라이브러리 공식 문서
> - [Eigen Getting Started](https://eigen.tuxfamily.org/dox/GettingStarted.html) — C++ 선형대수 라이브러리 입문

## 15.4 심화: 시스템 설계

*연구자가 되고 싶다면 여기서부터 읽어라.*

**15.4.1 Latency Budgeting**
- 전체 파이프라인의 latency를 구간별로 할당
- 예시: 자율주행 — 센서 입력(10ms) → 인식(50ms) → 계획(30ms) → 제어(10ms) = 100ms total
- 각 구간이 budget을 초과하면 전체가 실패. 가장 느린 구간이 bottleneck
- profiling 방법: ROS2 callback duration, `ros2 topic delay`, tracing (ros2_tracing)

**15.4.2 Behavior Tree**
- 유한 상태 기계(FSM)보다 확장성이 좋은 로봇 행동 설계 방법
- 노드 유형: Sequence (순차), Fallback (대안), Action (실행), Condition (조건)
- 장점: 모듈적 — 하위 트리를 독립적으로 테스트/재사용 가능
- ROS2에서: BehaviorTree.CPP, Nav2에서 사용
- FSM은 상태가 늘어나면 전이가 기하급수적으로 복잡해진다. BT는 트리 구조로 복잡도를 관리

**15.4.3 Safety와 Failsafe**
- Watchdog timer: 특정 시간 내에 heartbeat 없으면 safe stop
- E-stop (Emergency Stop): 하드웨어 레벨의 전원 차단
- Software safety: 속도 제한, workspace 제한, collision check
- ISO 13482: 서비스 로봇 안전 표준 (개요만)
- 실무: 새 알고리즘을 올릴 때는 safety wrapper를 먼저 만들고, 그 안에서 실험

**15.4.4 배포와 필드 테스트**
- CI/CD: colcon build + test 자동화, Docker 이미지 빌드
- Hardware-in-the-Loop (HIL): 실제 센서 데이터를 재생하면서 새 코드 테스트
- 필드 테스트 프로토콜: 통제된 환경 → 반통제 → 실제 환경, 단계적으로
- 로그 수집: rosbag + 시스템 로그 (journalctl) + 센서 상태 모니터링

> **추천 자료**
> - [BehaviorTree.CPP Documentation](https://www.behaviortree.dev/) — BT 설계 패턴과 튜토리얼
> - [Nav2 Documentation](https://docs.nav2.org/) — ROS2 Navigation2 스택. BT 기반 설계의 실전 예시

## 기술 흐름: 로봇 프레임워크의 과거 → 현재 → 미래

```
2007 ─── ROS1 출시 (Willow Garage)
  │       로봇 미들웨어로 널리 사용됨
  │
2012 ─── Gazebo Classic 독립 프로젝트화
  │       시뮬레이션이 로봇 개발의 필수 단계로 자리잡음
  │
2017 ─── ROS2 첫 릴리즈
  │       DDS 기반 통신, 실시간 지원, 보안 추가
  │
2019 ─── NVIDIA Isaac Sim 공개
  │       RTX 기반 고품질 렌더링 + 합성 데이터 생성
  │
2020 ─── Habitat, AI2-THOR 등 Embodied AI 시뮬레이터 부상
  │       대규모 학습 기반 로봇 정책 연구 본격화
  │
2022 ─── ROS2 Humble LTS 출시
  │       산업계 채택 가속화, Nav2/MoveIt2 안정화
  │
2024 ─── ROS1 Noetic EOL (지원 종료)
  │       ROS2 전환의 사실상 마감 시점
  │
2025+ ── Embodied AI + Foundation Models 시대
          시뮬레이터에서 대규모 사전학습 → Sim-to-Real 전이
          언어 명령 기반 로봇 조작 (VLA)
          NVIDIA Isaac Lab 등에서 시뮬레이터→학습→실제 로봇 배포를 일관된 파이프라인으로 제공 시작
```
