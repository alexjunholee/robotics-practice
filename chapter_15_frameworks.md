# 15장 로봇 프레임워크

앞선 장들에서 다룬 센서 처리, 위치 추정, 경로 계획, 모터 제어는 실제 로봇 위에서 따로따로 돌지 않는다. 카메라와 LiDAR 드라이버가 데이터를 읽어 들이는 동시에 SLAM 노드가 현재 위치를 계산하고, 플래너가 경로를 갱신하며, 제어기가 모터 명령을 내려보내야 한다. 이렇게 성격이 다른 모듈을 한꺼번에 실행하고 서로 데이터를 주고받으려면 스레드 관리, 메시지 직렬화, 좌표계 변환 같은 기반 작업이 뒤따른다. 매 프로젝트마다 이 배관 코드를 바닥부터 다시 짜지 않도록 공통 기반을 묶어 제공하는 것이 로봇 프레임워크다.

## 15.1 ROS (Robot Operating System)

ROS는 로봇 소프트웨어 개발에서 사실상 표준으로 자리 잡은 오픈소스 프레임워크다. 이름에 'Operating System'이 들어가지만 독자적인 운영체제가 아니라 리눅스 같은 OS 위에서 동작하는 미들웨어다. 프로세스 간 통신, 패키지 관리, 빌드 도구를 한데 갖추고 있으며, 통신 규약과 메시지 형식이 통일되어 있어 카메라 드라이버나 경로 계획기를 독립된 노드(Node)로 나누어 개발하고 레고 블록처럼 조합할 수 있다.

### 15.1.1 ROS1 vs ROS2

새 프로젝트를 시작하거나 연구실의 기존 코드를 물려받을 때는 가장 먼저 ROS1과 ROS2의 차이를 확인해야 한다. ROS2는 기존 ROS1의 단일 마스터 구조와 통신 제약을 해결하기 위해 설계를 바닥부터 다시 잡았다. 두 세대의 핵심 차이는 아래 표와 같다.

| 특징 | ROS1 | ROS2 |
| --- | --- | --- |
| 첫 공개 릴리스 | 2010 (ROS 1.0 Box Turtle; 개발 시작 2007) | 2017 (Ardent Apalone) |
| 통신 | Custom (TCPROS) | DDS(Data Distribution Service) 기반 |
| 실시간 | 고려하지 않은 설계 | 실시간 실행을 고려한 설계 (실시간 커널과 DDS 설정이 따로 필요) |
| 보안 | 기본 제공 없음 | SROS2 |
| Multi-robot | 어려움 | 쉬움 |
| Master | 필요 (roscore) | 불필요 |
| Python | 2/3 | 3 only |

배포판을 고를 때는 OS 버전과 지원 기간을 함께 따져야 한다. 2026년 10월 기준 최신 LTS(Long-Term Support, 장기 지원판)는 2026년 5월에 출시된 ROS2 Lyrical Luth(Ubuntu 26.04 기반, 2031년 5월까지 지원)다. 다만 연구실에서 쓰려는 외부 패키지가 아직 Lyrical을 지원하지 않는다면 Jazzy LTS(Ubuntu 24.04)를 선택하는 편이 무난하며, Ubuntu 22.04 환경이나 레거시 패키지 제약 때문에 Humble을 유지하는 프로젝트도 적지 않다. 어느 쪽을 고르든 사용할 로봇 드라이버와 핵심 패키지가 어떤 ROS 배포판과 Ubuntu 버전을 지원하는지 먼저 확인하고, EOL(End of Life, 지원 종료) 날짜도 함께 대조해 두어야 나중에 의존성 충돌을 피할 수 있다.

참고로 ROS1의 마지막 배포판인 Noetic은 2025년 5월에 공식 지원이 끝났다. 따라서 새 프로젝트는 ROS2로 시작하는 것이 원칙이며, Nav2와 MoveIt 2 역시 ROS2 전용으로 개발된다. 꼭 써야 하는 기존 ROS1 패키지가 남아 있다면 ROS1·ROS2·Ubuntu 버전 조합이 호환되는지 확인한 뒤 `ros1_bridge` 도입을 검토한다.

ROS2를 처음 접하거나 ROS1 코드를 이전해야 한다면 아래 공식 문서와 강의가 출발점으로 유용하다.

> **추천 자료**
> - [ROS2 공식 튜토리얼](https://docs.ros.org/en/jazzy/Tutorials.html) — ROS2 Jazzy LTS 기준 공식 단계별 가이드. 처음이라면 "Beginner: CLI tools"부터 시작
> - [The Construct 채널](https://www.youtube.com/@TheConstruct) — ROS 교육 YouTube 채널
> - [ROS1 to ROS2 Migration Guide](https://docs.ros.org/en/jazzy/How-To-Guides/Migrating-from-ROS1.html) — 기존 ROS1 코드 이전 공식 가이드

### 15.1.2 핵심 개념

ROS는 로봇 시스템을 기능별 작은 조각으로 쪼개고, 각 조각이 정해진 규약에 따라 대화하도록 만든다. 그 대화 방식을 이해하려면 노드, 토픽, 서비스, 액션, 파라미터라는 다섯 가지 개념을 먼저 잡아야 한다.

**Node (노드)**는 ROS 통신 그래프에 참여하는 최소 기능 단위다. 센서 드라이버, 상태 추정기, 제어기처럼 단일 목적을 맡는 모듈이 각각 하나의 노드가 된다. ROS1에서는 노드 하나가 사실상 운영체제 프로세스 하나와 1:1로 대응했지만, ROS2에서는 여러 노드를 컴포넌트(component) 형태로 묶어 한 프로세스 안에 함께 띄워 통신 오버헤드를 줄일 수 있다.

**Topic (토픽)**은 비동기 메시지 스트림을 주고받는 Publisher/Subscriber 패턴의 통신 채널이다. 센서 데이터나 속도 명령처럼 주기적으로 계속 흘러가는 데이터를 보낼 때 쓴다. 발행자(Publisher)는 누가 듣는지 신경 쓰지 않고 토픽에 메시지를 올리고, 구독자(Subscriber)는 필요한 토픽 이름만 지정해 받아 본다. 파이썬(`rclpy`)으로 0.5초마다 문자열 메시지를 발행하는 가장 기본적인 Publisher 노드는 다음과 같이 작성한다.

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

위 코드에서 `create_publisher`는 메시지 타입(`String`), 토픽 이름(`'topic'`), 큐 크기(`10`)를 등록하고, `create_timer`가 0.5초 주기로 `timer_callback`을 호출해 메시지를 내보낸다.

반면 매번 스트림으로 보낼 필요 없이 한 번 묻고 답을 받는 작업에는 **Service (서비스)**를 쓴다. 설정 변경이나 현재 상태 조회 같은 일회성 요청/응답 인터페이스다. 주의할 점은 ROS2 클라이언트 API가 기본으로 비동기 호출을 쓴다는 사실이다. 콜백을 실행하는 executor 스레드 안에서 서비스 응답을 동기로 기다리면 콜백이 멈춘 채 응답이 처리되지 않아 교착 상태(deadlock)에 빠진다.

목표 지점까지 이동하거나 팔을 뻗어 물체를 집는 일처럼 완료까지 시간이 오래 걸리는 작업은 **Action (액션)**에 맡긴다. 액션은 비동기 목표 지향 인터페이스로, 작업이 실행되는 동안 중간 진행 상황(Feedback)을 돌려받고 필요하면 도중에 취소할 수도 있다.

세 통신 방식 중 무엇을 고를지는 데이터의 성격에 달려 있다. 카메라 영상처럼 끊임없이 들어오는 데이터는 **Topic**으로 흘려보내고, "지금 배터리 잔량을 알려 줘"처럼 한 번의 요청과 응답으로 끝나는 일은 **Service**로 처리하며, "저기까지 이동해"처럼 시간이 걸리고 중간 상태 확인이나 취소가 필요한 작업은 **Action**으로 설계한다.

마지막으로 **Parameter (파라미터)**는 노드의 설정값을 외부에서 주입하거나 실행 도중에 바꿀 때 쓰는 키-값 저장소다. 제어 게인이나 센서 주기처럼 코드를 다시 컴파일하지 않고 튜닝하고 싶은 값을 파라미터로 빼 두면 실험 속도가 훨씬 빨라진다.

> **⚠ 생성된 코드 점검**: ROS2 코드를 요청할 때는 센서 토픽의 QoS를 함께 알려 준다. 생성된 subscriber의 reliability와 durability가 publisher와 호환되는지 확인한다. QoS가 호환되지 않으면 연결이 아예 맺어지지 않아 메시지가 하나도 오가지 않는다. 이 상황은 incompatible QoS 이벤트로 알아챌 수 있다. 두 설정이 같을 필요는 없다. RELIABLE publisher는 RELIABLE과 BEST_EFFORT subscriber 모두와 연결된다. BEST_EFFORT publisher는 BEST_EFFORT subscriber와만 연결된다. durability에서는 TRANSIENT_LOCAL publisher가 두 종류의 subscriber와 모두 연결되고, VOLATILE publisher는 VOLATILE subscriber와만 연결된다.

통신 모델의 세부 동작과 공식 예제는 아래 개념 문서에서 이어서 확인할 수 있다.

> **추천 자료**
> - [ROS2 Concepts — Understanding nodes, topics, services, actions](https://docs.ros.org/en/humble/Concepts.html) — 공식 개념 문서

### 15.1.3 도구

노드를 띄웠는데 원하는 대로 움직이지 않을 때, 코드를 로봇에 곧장 올리기 전에 센서 데이터가 제대로 들어오는지, 토픽 연결이 끊기지 않았는지, 좌표계가 어긋나지 않았는지 눈으로 확인하면 디버깅 시간이 크게 줄어든다. ROS2는 이 과정을 돕는 시각화·진단 도구를 기본으로 제공한다.

- **rviz / rviz2**: 센서 포인트 클라우드, 카메라 영상, TF 좌표축, 계획된 경로를 3차원 공간에 겹쳐 보여 주는 시각화 도구다. 알고리즘 출력을 직관적으로 검증할 때 가장 먼저 띄우는 창이기도 하다.
- **rqt**: Qt 기반의 GUI 플러그인 모음이다. 노드와 토픽의 연결망을 한눈에 그려 주는 `rqt_graph`, 토픽의 숫자 값을 실시간 시계열 그래프로 띄우는 `rqt_plot` 등이 자주 쓰인다.
- **rosbag / ros2 bag**: 토픽으로 오가는 센서 데이터를 그대로 파일로 녹화하고 나중에 다시 재생하는 도구다. 현장에서 드물게 재현되는 버그를 연구실 자리로 가져와 똑같은 입력으로 알고리즘을 몇 번이고 반복 검증할 때 필수다.

녹화와 재생 명령은 두 줄이면 된다.

```bash
# ROS2 bag 녹화
ros2 bag record -a -o my_bag

# 재생
ros2 bag play my_bag
```

`-a` 옵션은 현재 열려 있는 모든 토픽을 기록하겠다는 뜻이며, 필요한 토픽만 골라 기록하면 용량을 크게 아낄 수 있다.

여기에 더해 로봇 시스템에서 빠지지 않는 도구가 **tf2 (Transform Library)**다. 실제 로봇에는 카메라 좌표계, LiDAR 좌표계, 몸체 중심인 베이스 좌표계(`base_link`), 고정된 월드 좌표계가 동시에 존재한다. "카메라가 본 장애물의 위치가 로봇 몸체 기준으로 어디인가?"를 계산하려면 시간에 따라 변하는 좌표 변환을 계속 추적해야 한다. tf2는 각 좌표계 사이의 관계를 SE(3) 변환 트리로 저장해 두고, 원하는 시점의 좌표 변환을 조회할 수 있게 해 준다. 아래는 `camera_link` 좌표를 `base_link` 기준으로 바꾸는 변환을 주기적으로 조회하는 파이썬 코드다.

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

`TransformListener`가 백그라운드에서 TF 토픽을 구독해 버퍼를 채우기 전에는 변환을 찾을 수 없으므로, 타이머 콜백 안에서 조회하고 `LookupException`이나 `ExtrapolationException`을 예외 처리해 두는 것이 안전하다.

> **추천 자료**
> - [ROS2 tf2 Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html) — 좌표 변환 공식 튜토리얼
> - [ros2 bag CLI 문서](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html) — 데이터 녹화/재생 공식 가이드

### 15.1.4 주요 패키지

ROS 생태계의 강점은 표준 메시지와 공용 변환 패키지가 이미 잘 정비되어 있다는 데 있다. 이미지, 포인트 클라우드, 내비게이션 파이프라인을 짤 때 거의 모든 프로젝트가 공통으로 가져다 쓰는 핵심 패키지는 다음과 같다.

| 패키지 | 용도 |
| --- | --- |
| sensor_msgs | 센서 메시지 타입 |
| geometry_msgs | 기하학 메시지 (Pose, Twist 등) |
| cv_bridge | OpenCV ↔︎ ROS 이미지 변환 |
| image_transport | 이미지 압축 전송 |
| pcl_ros | PCL ↔︎ ROS 포인트 클라우드 |
| nav2 | Navigation 스택 (ROS2) |

이 패키지들이 제공하는 표준 메시지 타입에 맞추어 입출력을 설계해 두면, 이후 다른 오픈소스 알고리즘이나 시뮬레이터로 갈아탈 때도 최소한의 어댑터 코드만으로 연결할 수 있다.

> **추천 자료**
> - [Nav2 Documentation](https://docs.nav2.org/) — ROS2 Navigation 스택 공식 문서
> - [ROS2 Package Index](https://index.ros.org/packages/) — ROS2 패키지 검색

## 15.2 시뮬레이션

통신 구조와 시각화 도구를 갖췄더라도 곧바로 실물 로봇에 새 코드를 올리는 것은 위험하다. 제어 게인 하나만 잘못 들어가도 하드웨어가 파손되거나 주변 사람이 다칠 수 있고, 강화학습처럼 수천~수만 번의 시행착오가 필요한 실험은 물리 로봇으로 감당하기 어렵다. 그래서 실제 실험에 앞서 시뮬레이터에서 동작 범위와 실패 조건을 검증하고 대규모 에피소드를 만들어 낸다. 특히 Embodied AI 분야에서는 NVIDIA Isaac Sim, AI2-THOR, Habitat 같은 가상 환경에서 정책을 학습시킨 뒤 실제 로봇으로 옮기는 **Sim-to-Real Transfer**가 핵심 연구 흐름으로 자리 잡았다.

### 15.2.1 Gazebo

Gazebo는 ROS와 오랫동안 짝을 이루어 온 대표적인 오픈소스 물리 시뮬레이터다. 로봇 패키지를 내려받으면 Gazebo용 시뮬레이션 데모와 로봇 모델이 함께 딸려 오는 경우가 흔하다. Gazebo에서 세계와 로봇을 기술할 때는 두 가지 XML 형식을 나란히 쓴다. **URDF (Unified Robot Description Format)**는 로봇의 링크, 관절, 시각 형상, 충돌 범위를 정의하고, **SDF (Simulation Description Format)**는 조명·지형·물체 같은 주변 환경 전체와 물리 속성을 기술한다.

버전을 고를 때는 세대교체를 염두에 두어야 한다. 예전부터 쓰이던 Gazebo Classic(Gazebo 11)은 2025년 1월 31일부로 공식 지원이 종료되었다. 새 ROS2 프로젝트에서는 모듈화된 구조와 확장성을 갖춘 **Gazebo Sim**(구 Ignition)을 쓰는 것이 표준이다.

아래는 가장 단순한 형태의 URDF 예시로, 로봇의 기본 몸체(`base_link`) 하나에 눈에 보이는 외형(`visual`)과 물리 충돌 계산에 쓰이는 상자(`collision`)를 함께 정의한다.

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

실제 로봇 모델에서는 여기에 관성 텐서(`<inertial>`)와 센서 플러그인을 추가하고, 복잡한 외형 메시는 `visual`에 두되 연산량이 큰 `collision`에는 단순한 기하 도형을 넣어 시뮬레이션 속도를 확보한다.

> **추천 자료**
> - [Gazebo Sim 공식 튜토리얼](https://gazebosim.org/docs) — Gazebo Sim(구 Ignition) 공식 가이드
> - [URDF Tutorial (ROS2)](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html) — 로봇 모델링 기초

### 15.2.2 NVIDIA Isaac Sim

사실적인 렌더링과 GPU 병렬 물리 시뮬레이션이 필요할 때는 **NVIDIA Isaac Sim**을 많이 쓴다. RTX 기반 레이 트레이싱 렌더링과 PhysX 5 물리 엔진을 결합해 조명이 까다롭거나 접촉이 많은 조작(Manipulation) 및 내비게이션(Navigation) 작업을 정밀하게 모사할 수 있으며, ROS2와도 직접 연동된다. 특히 Embodied AI 연구에서는 Domain Randomization으로 조명·질감·배치를 무작위로 바꿔 가며 대규모 합성 데이터를 생성해 Sim-to-Real 학습을 돌릴 때 자주 활용한다.

물론 모든 연구에 무거운 RTX 시뮬레이터가 정답은 아니다. 실내 상호작용 중심인지, 수천 FPS의 빠른 시각 학습이 필요한지, 정교한 접촉 역학이 핵심인지에 따라 선택지가 달라진다. Embodied AI와 로보틱스 연구에서 널리 쓰이는 대표 시뮬레이터를 비교하면 다음과 같다.

| 시뮬레이터 | 주 용도 | 특징 |
| --- | --- | --- |
| NVIDIA Isaac Sim | 범용 (Manipulation, Navigation) | RTX 렌더링, PhysX 5, 대규모 합성 데이터 |
| AI2-THOR | 실내 내비게이션, 물체 상호작용 | 120+ 실내 장면, 사실적 상호작용 |
| Habitat (Meta) | 시각 내비게이션, Embodied QA | 초고속 렌더링 (수천 FPS), 대규모 학습 |
| iGibson | 실내 로봇 작업 | 물리 기반 렌더링, 가정환경 |
| MuJoCo | 로봇 제어, 강화학습 | 정확한 접촉 역학, 빠른 시뮬레이션 |

실내 가정환경의 의미론적 상호작용을 빠르게 실험하려면 AI2-THOR나 iGibson이 가볍고, 초고속 대량 에피소드가 필요하다면 Habitat이나 MuJoCo가 유리하며, 고충실도 렌더링과 ROS2·합성 데이터 파이프라인을 한 번에 묶고 싶다면 Isaac Sim이 잘 맞는다.

> **추천 자료**
> - [NVIDIA Isaac Sim 공식 문서](https://docs.isaacsim.omniverse.nvidia.com/latest/index.html) — 설치부터 고급 활용까지
> - [AI2-THOR Documentation](https://ai2thor.allenai.org/ithor/documentation) — Embodied AI 연구용 실내 시뮬레이터
> - [Habitat Documentation](https://aihabitat.org/docs/habitat2/) — Meta의 Embodied AI 플랫폼

### 15.2.3 CARLA

자율주행 연구에 특화된 오픈소스 시뮬레이터로는 **CARLA**가 널리 쓰인다. 도심 도로망과 보행자·차량 트래픽을 갖추고 있으며, 비·안개·야간 같은 다양한 날씨와 시간대 조건을 손쉽게 바꿀 수 있다. 카메라, LiDAR, Radar 센서를 동시에 시뮬레이션할 수 있고 ROS와 연결하는 공식 브리지도 제공해, 인식부터 경로 계획·제어까지 자율주행 파이프라인 전체를 가상 도로 위에서 검증할 때 유용하다.

> **추천 자료**
> - [CARLA Documentation](https://carla.readthedocs.io/) — 공식 문서 및 Python API 레퍼런스
> - [CARLA Simulator YouTube](https://www.youtube.com/channel/UC1llP9ekCwt8nEJzMJBQekg) — 시뮬레이터 활용 데모 영상

## 15.3 기타 프레임워크

범용 ROS2와 시뮬레이터 외에도 특정 도메인이나 연산 계층을 겨냥한 프레임워크와 라이브러리가 함께 쓰인다.

**Isaac ROS**는 NVIDIA GPU와 Jetson 임베디드 보드에 최적화해 만든 GPU 가속 ROS2 패키지 모음이다. CPU만으로는 실시간 처리가 버거운 DNN 추론, Visual SLAM, 3D 인식 파이프라인을 하드웨어로 가속할 때 쓴다. **Autoware**는 인식·계획·제어를 모두 통합한 완전한 오픈소스 자율주행 소프트웨어 스택으로, 현재 주력인 `Autoware.Universe`가 ROS2 위에서 동작한다.

한편 ROS 바깥에서 알고리즘의 알맹이를 구현할 때 단골로 등장하는 라이브러리들도 익혀 두어야 한다. 영상 처리에 쓰이는 **OpenCV**와 3차원 포인트 클라우드·메시 처리를 맡는 **Open3D**가 양대 축이며, C++로 상태 추정이나 좌표 변환을 직접 구현할 때는 **Eigen**과 **Sophus** 조합이 사실상 표준이다. Eigen이 행렬과 벡터 연산을 담당하는 범용 선형대수 라이브러리라면, Sophus는 Eigen 위에서 회전군 SO(3)과 특수 유클리드군 SE(3)의 리 군 연산(`exp`, `log`, 변환 합성)을 깔끔하게 구현해 준다.

> **추천 자료**
> - [OpenCV Tutorials](https://docs.opencv.org/4.x/d9/df8/tutorial_root.html) — 컴퓨터 비전 기초부터 고급까지
> - [Open3D Documentation](https://www.open3d.org/docs/release/) — 3D 데이터 처리 라이브러리 공식 문서
> - [Eigen Getting Started](https://libeigen.gitlab.io/eigen/docs-nightly/GettingStarted.html) — C++ 선형대수 라이브러리 입문

## 15.4 시스템 설계 [심화]

개별 알고리즘과 프레임워크를 고르는 일만큼, 이 조각들을 하나의 안전한 실시간 시스템으로 묶는 아키텍처 설계도 중요하다. 연구실 프로토타입을 넘어 실제 현장에서 굴러가는 로봇을 만들 때는 지연 시간 예산, 행동 트리, 안전장치, 배포 절차를 함께 설계해야 한다.

**15.4.1 Latency Budgeting**

Latency Budgeting은 전체 파이프라인이 반응해야 하는 마감 시간을 정해 두고 구간별로 지연 시간 예산을 배분하는 작업이다. 예를 들어 자율주행 시스템의 종단 마감 시간이 100 ms라면, 센서 입력(10 ms) → 인식(50 ms) → 계획(30 ms) → 제어(10 ms)처럼 단계별 목표치를 나눈다. 이때 구간별 수치는 절대적 상한이라기보다 설계 목표에 가깝다. 구간마다 엄격한 마감 시간을 강제하지 않는 구조라면, 한 구간이 예산을 넘겼더라도 다른 구간의 여유 시간으로 만회해 전체 마감 시간을 맞출 수 있다. 다만 전체 시스템의 처리율(throughput)은 결국 가장 느린 병목 구간이 결정하므로, ROS2 콜백 실행 시간, `ros2 topic delay`, `ros2_tracing` 같은 프로파일링 도구로 어느 노드가 시간을 잡아먹는지 꾸준히 측정해야 한다.

**15.4.2 Behavior Tree**

로봇의 임무 상태가 복잡해질수록 단순한 유한 상태 기계(FSM)는 상태 간 전이 화살표가 기하급수적으로 불어나 관리가 어려워진다. Behavior Tree(BT)는 행동을 순차(Sequence), 대안(Fallback), 실행(Action), 조건(Condition) 노드의 트리 구조로 묶어 이 복잡도를 다스린다. 하위 트리를 독립된 모듈처럼 떼어 내 테스트하고 재사용하기 좋다는 것이 큰 장점이며, ROS2 생태계에서는 `BehaviorTree.CPP`와 `Nav2`가 BT 기반 행동 설계를 표준으로 채택하고 있다.

**15.4.3 Safety와 Failsafe**

물리 세계에서 움직이는 로봇에는 소프트웨어 버그나 통신 끊김에 대비한 다층 안전장치가 필수다. 구체적으로는 Watchdog timer, 하드웨어 E-stop(Emergency Stop), Software safety 레이어가 겹쳐 동작해야 한다. Watchdog timer는 정해진 시간 동안 생존 신호(heartbeat)가 끊기면 즉시 정지 모드로 전환하고, 하드웨어 E-stop은 소프트웨어와 무관하게 전원을 물리적으로 끊는다. Software safety 레이어는 속도와 작업 반경을 제한하고 충돌 여부를 먼저 검사한다. 개인 돌봄 로봇(personal care robot) 분야에서는 이러한 안전 요구사항을 다룬 국제 표준 ISO 13482도 참고한다. 실무에서는 새 알고리즘을 테스트하기 전에 먼저 safety wrapper를 씌우고, 그 안전 울타리 안에서만 실험을 진행하는 원칙을 지킨다.

**15.4.4 배포와 필드 테스트**

로봇 코드를 현장에 내보내기 전에는 검증 단계를 촘촘히 밟는다. `colcon build`와 단위 테스트를 CI/CD 파이프라인과 Docker 이미지 빌드로 자동화해 두고, 앞서 `ros2 bag`으로 녹화해 둔 센서 로그를 재생하며 새 코드가 기존 주행에서 회귀 오류를 일으키지 않는지 먼저 확인한다. 실제 제어기나 센서를 폐루프에 물리고 나머지 플랜트만 실시간 시뮬레이션으로 돌리는 Hardware-in-the-Loop(HIL) 테스트도 유용하다. 야외나 현장 테스트에 들어갈 때는 통제된 환경에서 반통제 환경을 거쳐 실제 환경으로 단계적으로 범위를 넓힌다. 실행 중에는 `rosbag` 데이터뿐만 아니라 `journalctl` 같은 시스템 로그와 센서 상태 모니터링 지표까지 함께 수집해야 문제가 터졌을 때 원인을 추적할 수 있다.

> **추천 자료**
> - [BehaviorTree.CPP Documentation](https://www.behaviortree.dev/) — BT 설계 패턴과 튜토리얼
> - [Nav2 Documentation](https://docs.nav2.org/) — ROS2 Navigation2 스택. BT 기반 설계의 실전 예시

프레임워크와 시뮬레이터, 안전 설계의 큰 그림을 잡았더라도 CUDA, Python, ROS, 시스템 라이브러리 버전이 어긋나면 실행 환경이 금세 꼬인다. 다음으로 해결해야 할 과제는 프로젝트마다 의존성을 격리하고 다른 컴퓨터나 로봇 보드에서도 똑같은 실행 환경을 재현하는 일이다(→ 16장).

## 발전 과정

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
