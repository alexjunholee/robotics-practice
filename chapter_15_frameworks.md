# Ch.15 — 로봇 프레임워크 (Robot Frameworks)


로봇 소프트웨어는 센서 드라이버, 경로 계획, 모터 제어를 동시에 실행하면서 그 사이의 통신도 관리해야 한다. 스레드 관리, 메시지 직렬화, 좌표 변환처럼 여러 모듈이 함께 쓰는 기능은 프레임워크가 맡는다.

## 15.1 ROS (Robot Operating System)

ROS는 로봇 소프트웨어 개발을 위한 오픈소스 프레임워크다. 운영체제 위에서 도는 미들웨어로, 프로세스 간 통신, 패키지 관리, 빌드 도구를 갖췄다. 모듈 사이의 통신 방식과 메시지 형식을 통일해 각 기능을 별도 노드로 나누어 개발할 수 있게 한다.

### 15.1.1 ROS1 vs ROS2

| 특징 | ROS1 | ROS2 |
| --- | --- | --- |
| 첫 공개 릴리스 | 2010 (ROS 1.0 Box Turtle; 개발 시작 2007) | 2017 (Ardent Apalone) |
| 통신 | Custom (TCPROS) | DDS(Data Distribution Service) 기반 |
| 실시간 | 고려하지 않은 설계 | 실시간 실행을 고려한 설계 (실시간 커널과 DDS 설정이 따로 필요) |
| 보안 | 기본 제공 없음 | SROS2 |
| Multi-robot | 어려움 | 쉬움 |
| Master | 필요 (roscore) | 불필요 |
| Python | 2/3 | 3 only |

2026년 10월 기준 최신 LTS(Long-Term Support, 장기 지원판)는 2026년 5월에 나온 ROS2 Lyrical Luth(Ubuntu 26.04, 2031년 5월까지 지원)다. 쓰려는 package가 아직 Lyrical을 지원하지 않으면 Jazzy LTS(Ubuntu 24.04)를 쓰고, Ubuntu 22.04나 기존 package 제약 때문에 Humble을 유지하는 프로젝트도 있다.

배포판을 고를 때는 사용하는 robot driver와 package의 지원 distribution, Ubuntu 버전, EOL(End of Life, 지원 종료) 날짜를 함께 확인한다. ROS1 Noetic은 2025년 5월에 공식 지원이 끝났다.

새 프로젝트는 ROS2로 시작한다. 기존 ROS1 package가 필요하면 ROS1·ROS2·Ubuntu 버전 조합을 먼저 확인하고 `ros1_bridge`를 검토한다. Nav2와 MoveIt 2는 ROS2용이다.

> **추천 자료**
> - [ROS2 공식 튜토리얼](https://docs.ros.org/en/jazzy/Tutorials.html) — ROS2 Jazzy LTS 기준 공식 단계별 가이드. 처음이라면 "Beginner: CLI tools"부터 시작
> - [The Construct 채널](https://www.youtube.com/@TheConstruct) — ROS 교육 YouTube 채널
> - [ROS1 to ROS2 Migration Guide](https://docs.ros.org/en/jazzy/How-To-Guides/Migrating-from-ROS1.html) — 기존 ROS1 코드 이전 공식 가이드

### 15.1.2 핵심 개념

**Node (노드)**:
- 그래프에 참여하는 기능 단위. ROS 1의 노드는 사실상 프로세스 하나였다. ROS 2에서는 노드를 component로 만들어 한 프로세스에 여럿 담을 수 있다
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
- 요청/응답 인터페이스. ROS 2 클라이언트 API는 기본으로 비동기 호출을 쓴다. executor를 돌리는 스레드(콜백 안)에서 응답을 동기로 기다리면 교착이 생긴다
- 일회성 작업에 적합
- 예: 설정 변경, 상태 조회

**Action (액션)**:
- 비동기 목표 지향 작업
- Feedback 제공
- 취소 가능
- 예: 네비게이션, 조작

카메라 영상처럼 계속 들어오는 데이터는 Topic으로 보낸다. "지금 배터리 잔량 알려줘" 같은 한 번의 요청과 응답은 Service로 처리하고, "저기까지 가"처럼 시간이 걸리는 작업은 Action으로 맡긴다.

**Parameter (파라미터)**:
- 노드 설정값
- 런타임 변경 가능

> **⚠ 생성된 코드 점검**: ROS2 코드를 요청할 때는 센서 토픽의 QoS를 함께 알려 준다. 생성된 subscriber의 reliability와 durability가 publisher와 호환되는지 확인한다. QoS가 호환되지 않으면 연결이 아예 맺어지지 않아 메시지가 하나도 오가지 않는다. 이 상황은 incompatible QoS 이벤트로 알아챌 수 있다. 두 설정이 같을 필요는 없다. RELIABLE publisher는 RELIABLE과 BEST_EFFORT subscriber 모두와 연결된다. BEST_EFFORT publisher는 BEST_EFFORT subscriber와만 연결된다. durability에서는 TRANSIENT_LOCAL publisher가 두 종류의 subscriber와 모두 연결되고, VOLATILE publisher는 VOLATILE subscriber와만 연결된다.

> **추천 자료**
> - [ROS2 Concepts — Understanding nodes, topics, services, actions](https://docs.ros.org/en/humble/Concepts.html) — 공식 개념 문서

### 15.1.3 도구

코드를 로봇에 올리기 전에 센서 데이터가 제대로 들어오는지, 좌표계가 맞는지 눈으로 확인하면 디버깅 시간이 줄어든다. ROS 개발에서는 아래 도구로 확인한다.

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

rosbag으로 센서 데이터를 한 번 녹화해 두면 같은 데이터로 실험을 몇 번이고 반복할 수 있다. 같은 입력을 다시 넣을 수 있으니 결과도 재현된다.

```bash
# ROS2 bag 녹화
ros2 bag record -a -o my_bag

# 재생
ros2 bag play my_bag
```

**tf2 (Transform Library)**:
- 좌표계 변환 관리
- 시간에 따른 변환 추적

로봇에는 카메라 좌표계, LiDAR 좌표계, 베이스 좌표계, 월드 좌표계 등이 함께 있다. "카메라에서 본 점이 로봇 기준으로 어디인가?"를 계산하려면 좌표 변환이 필요하다. tf2는 이 변환들을 SE(3) 변환으로 저장하고 시간에 따라 관리한다.

```python
# tf2 리스너 예시
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException

class TfDemo(Node):
    def __init__(self):
        super().__init__('tf_demo')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # 리스너가 버퍼를 채울 시간을 주기 위해 타이머 안에서 조회한다
        self.create_timer(0.1, self.lookup)

    def lookup(self):
        try:
            # camera_link 좌표를 base_link 기준으로 바꾸는 변환을 조회한다
            t = self.tf_buffer.lookup_transform('base_link', 'camera_link', rclpy.time.Time())
        except (LookupException, ExtrapolationException) as e:
            self.get_logger().warn(f'tf not ready: {e}')
```

> **추천 자료**
> - [ROS2 tf2 Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html) — 좌표 변환 공식 튜토리얼
> - [ros2 bag CLI 문서](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html) — 데이터 녹화/재생 공식 가이드

### 15.1.4 주요 패키지

표준 메시지, 영상·포인트 클라우드 변환, 내비게이션에 쓰는 주요 패키지는 아래와 같다.

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

실물 로봇에서 바로 실험하면 하드웨어가 손상되거나 사람이 다칠 수 있다. 시뮬레이터에서는 동작 범위와 실패 조건을 먼저 확인한다. 강화학습에 필요한 시행착오 에피소드도 시뮬레이터에서 빠르게 반복해 만든다.

Embodied AI 연구에서는 로봇 정책을 NVIDIA Isaac Sim, AI2-THOR, Habitat 같은 가상 환경에서 학습시킨다. 가상 환경에서 학습한 정책을 실제 로봇으로 옮기는 일을 Sim-to-Real Transfer라고 한다.

### 15.2.1 Gazebo

Gazebo는 ROS와 연동되는 시뮬레이터다. Gazebo용 시뮬레이션 데모와 로봇 모델을 함께 배포하는 ROS 패키지가 많다.

구성 요소:
- **SDF (Simulation Description Format)**: 환경 정의
- **URDF (Unified Robot Description Format)**: 로봇 모델

Gazebo Classic vs Gazebo Sim (Ignition):
- Gazebo Sim: 새 버전, ROS2 권장
- 모듈화 구조, 더 나은 확장성
- Gazebo Classic(Gazebo 11)은 2025년 1월 31일에 지원이 끝났다

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

### 15.2.2 NVIDIA Isaac Sim

Isaac Sim은 Embodied AI 연구에서 Domain Randomization으로 대규모 합성 데이터를 만들고 Sim-to-Real 학습을 할 때 많이 쓴다. RTX 렌더링과 PhysX 5 물리 엔진을 쓴다. ROS2와 연동되며 Manipulation과 Navigation 연구에 쓰인다.

Embodied AI 연구에서 쓰는 시뮬레이터는 아래와 같다.

| 시뮬레이터 | 주 용도 | 특징 |
| --- | --- | --- |
| NVIDIA Isaac Sim | 범용 (Manipulation, Navigation) | RTX 렌더링, PhysX 5, 대규모 합성 데이터 |
| AI2-THOR | 실내 내비게이션, 물체 상호작용 | 120+ 실내 장면, 사실적 상호작용 |
| Habitat (Meta) | 시각 내비게이션, Embodied QA | 초고속 렌더링 (수천 FPS), 대규모 학습 |
| iGibson | 실내 로봇 작업 | 물리 기반 렌더링, 가정환경 |
| MuJoCo | 로봇 제어, 강화학습 | 정확한 접촉 역학, 빠른 시뮬레이션 |

> **추천 자료**
> - [NVIDIA Isaac Sim 공식 문서](https://docs.isaacsim.omniverse.nvidia.com/latest/index.html) — 설치부터 고급 활용까지
> - [AI2-THOR Documentation](https://ai2thor.allenai.org/ithor/documentation) — Embodied AI 연구용 실내 시뮬레이터
> - [Habitat Documentation](https://aihabitat.org/docs/habitat2/) — Meta의 Embodied AI 플랫폼

### 15.2.3 CARLA

자율주행 연구용 오픈소스 시뮬레이터다. 도시 환경과 다양한 날씨·시간대를 지원한다. 카메라·LiDAR·Radar 센서를 시뮬레이션하며, ROS와 잇는 브릿지도 있다.

> **추천 자료**
> - [CARLA Documentation](https://carla.readthedocs.io/) — 공식 문서 및 Python API 레퍼런스
> - [CARLA Simulator YouTube](https://www.youtube.com/channel/UC1llP9ekCwt8nEJzMJBQekg) — 시뮬레이터 활용 데모 영상

## 15.3 기타 프레임워크

특정 용도에 맞춘 프레임워크와 라이브러리는 아래와 같다.

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

C++로 좌표 변환을 구현할 때는 Eigen과 Sophus를 많이 쓴다. Eigen은 범용 선형대수 라이브러리다. Sophus는 Eigen으로 회전군 SO(3)과 특수 유클리드군 SE(3)의 연산(exp, log, 합성)을 구현한 라이브러리다.

> **추천 자료**
> - [OpenCV Tutorials](https://docs.opencv.org/4.x/d9/df8/tutorial_root.html) — 컴퓨터 비전 기초부터 고급까지
> - [Open3D Documentation](https://www.open3d.org/docs/release/) — 3D 데이터 처리 라이브러리 공식 문서
> - [Eigen Getting Started](https://libeigen.gitlab.io/eigen/docs-nightly/GettingStarted.html) — C++ 선형대수 라이브러리 입문

## 15.4 심화: 시스템 설계

15.4.1 Latency Budgeting
- 전체 파이프라인의 latency를 구간별로 할당
- 예시: 자율주행 — 센서 입력(10 ms) → 인식(50 ms) → 계획(30 ms) → 제어(10 ms) = 100 ms total
- 구간별 값은 종단 deadline을 나눠 배분한 설계 목표다. 구간별 deadline을 강제하지 않으면 한 구간이 넘긴 시간을 다른 구간의 여유로 메울 수 있다. 처리율은 가장 느린 구간이 정한다
- profiling 방법: ROS2 callback duration, `ros2 topic delay`, tracing (ros2_tracing)

15.4.2 Behavior Tree
- 유한 상태 기계(FSM)보다 확장성이 좋은 로봇 행동 설계 방법
- 노드 유형: Sequence (순차), Fallback (대안), Action (실행), Condition (조건)
- 장점: 모듈적 — 하위 트리를 독립적으로 테스트/재사용 가능
- ROS2에서: BehaviorTree.CPP, Nav2에서 사용
- FSM은 상태가 늘수록 전이 수가 빠르게 늘어난다. BT는 행동을 트리로 묶어 이 복잡도를 관리한다

15.4.3 Safety와 Failsafe
- Watchdog timer: 정해진 시간 동안 heartbeat가 끊기면 safe stop
- E-stop (Emergency Stop): 하드웨어 레벨의 전원 차단
- Software safety: 속도 제한, workspace 제한, collision check
- ISO 13482: 개인 돌봄 로봇(personal care robot)의 안전 요구사항 표준
- 실무: 새 알고리즘을 올릴 때는 safety wrapper를 먼저 만들고, 그 안에서 실험

15.4.4 배포와 필드 테스트
- CI/CD: colcon build + test 자동화, Docker 이미지 빌드
- 로그 재생 테스트: 기록한 센서 데이터를 재생하면서 새 코드 테스트
- Hardware-in-the-Loop (HIL): 실제 하드웨어를 폐루프에 넣고 플랜트 쪽을 실시간 시뮬레이션하며 테스트
- 필드 테스트 프로토콜: 통제된 환경 → 반통제 → 실제 환경, 단계적으로
- 로그 수집: rosbag + 시스템 로그 (journalctl) + 센서 상태 모니터링

> **추천 자료**
> - [BehaviorTree.CPP Documentation](https://www.behaviortree.dev/) — BT 설계 패턴과 튜토리얼
> - [Nav2 Documentation](https://docs.nav2.org/) — ROS2 Navigation2 스택. BT 기반 설계의 실전 예시

## 기술 흐름: 로봇 프레임워크의 과거 → 현재 → 미래

```
2007 ─── ROS 개발 시작 (Stanford → Willow Garage; 1.0 릴리스는 2010)
  │
2012 ─── Gazebo Classic 독립 프로젝트화
  │
2017 ─── ROS2 첫 릴리즈
  │       DDS 기반 통신, 실시간 실행을 고려한 설계, 보안(SROS2) 추가
  │
2019 ─── NVIDIA Isaac Sim 공개 (GTC 2019에서 Isaac SDK와 함께 발표)
  │       RTX 렌더링을 쓰는 Omniverse 기반 판은 2021년(2021.1)부터
  │
2019 ─── Habitat 공개 (Facebook AI Research 등, ICCV 2019; AI2-THOR는 2017년 공개)
  │
2022 ─── ROS2 Humble LTS 출시 (2022년 5월, 2027년 5월까지 지원)
  │
2024 ─── NVIDIA Isaac Lab 1.0 공개 (Isaac Sim 4.0과 함께, Orbit 후속)
  │       Isaac Sim 위에서 돌아가는 로봇 학습 프레임워크
  │
2025 ─── NVIDIA Isaac GR00T N1 공개 (2025년 3월 GTC)
  │       휴머노이드용 공개 VLA 기반 모델. 실제 로봇 궤적·사람 영상·합성 데이터로 학습
  │
2025 ─── ROS1 Noetic EOL (2025년 5월)
  │
2026 ─── ROS2 Lyrical Luth LTS 출시 (2026년 5월)
```
