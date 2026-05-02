# Ch.19 — AI 코딩 에이전트와 협업하기

## 19.1 왜 이 챕터가 필요한가

AI 코딩 에이전트(Claude, Copilot, ChatGPT 등)는 일반 소프트웨어 개발에서는 강력한 도구다. 함수 하나 짜달라고 하면 꽤 쓸만한 코드가 나오고, 에러 메시지를 붙여넣으면 원인을 잘 짚어준다. 하지만 로보틱스는 다르다. 하드웨어, OS, 네트워크, 실시간성이 복잡하게 얽혀 있고, "코드는 맞는데 안 되는" 상황이 일상이다. AI는 이런 영역에서 자주 틀리거나, 자신 있게 엉뚱한 방향을 제시하거나, 아예 포기한다.

이 챕터는 AI에게 속지 않고, AI를 제대로 부려먹기 위한 가이드다. 여기 나오는 내용은 전부 실제로 겪은 문제들이다. AI가 틀리는 패턴을 미리 알아두면 삽질이 줄어든다.

## 19.2 ROS에서 AI가 자주 틀리는 것들

### 19.2.1 QoS 설정

AI에게 ROS2 subscriber를 짜달라고 하면 QoS(Quality of Service)를 default(RELIABLE)로 놓는다. 센서 토픽(카메라, LiDAR)은 BEST_EFFORT로 퍼블리시되는 경우가 대부분인데, subscriber가 RELIABLE이면 데이터가 아예 안 들어온다. 에러 메시지도 안 뜨고 그냥 조용히 안 되니까, AI는 "토픽이 없나?" 하고 엉뚱한 방향으로 디버깅을 시작한다.

```bash
# 토픽의 QoS 프로파일 확인
ros2 topic info /camera/image_raw --verbose
```

출력에서 `Reliability: BEST_EFFORT`, `Durability: VOLATILE` 같은 정보를 확인하고, subscriber의 QoS를 맞춰야 한다.

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=10
)
self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, qos)
```

AI에게 코드를 요청할 때는 "이 토픽의 QoS는 BEST_EFFORT / SENSOR_DATA다"라고 명시적으로 알려줘야 한다. 안 그러면 기본값으로 만들어서 데이터가 안 들어오는데, AI는 원인을 못 찾는다.

### 19.2.2 use_sim_time과 tf2 타이밍

rosbag 재생 시 `use_sim_time:=true`를 안 하면 tf lookup이 전부 실패한다. AI는 "tf2 lookup failed" 에러를 보면 `static_transform_publisher`를 추가하라고 한다. 엉뚱한 방향이다.

실제 원인은 시뮬레이션 클럭과 시스템 클럭의 불일치다. bag 파일의 타임스탬프는 과거 시점인데, 노드는 현재 시스템 시간을 기준으로 tf를 조회하니까 당연히 못 찾는다.

```bash
# 올바른 rosbag 재생
ros2 bag play my_bag --clock

# 노드 실행 시 sim_time 활성화
ros2 launch my_package my_launch.py use_sim_time:=true
```

tf2 lookup에는 timeout과 try/except가 필요한데, AI는 이걸 빠뜨린다.

```python
from rclpy.duration import Duration

try:
    transform = tf_buffer.lookup_transform(
        'base_link', 'camera_link',
        rclpy.time.Time(),
        timeout=Duration(seconds=1.0)
    )
except tf2_ros.LookupException as e:
    self.get_logger().warn(f'TF lookup failed: {e}')
```

### 19.2.3 Workspace 소싱 순서

ROS2 workspace의 소싱 순서가 중요하다. `/opt/ros/humble/setup.bash`를 먼저 source하고, 그다음에 `~/ros2_ws/install/setup.bash`를 source해야 한다. AI는 하나만 source하거나 순서를 뒤집는다. overlay workspace 개념을 아예 모르는 경우도 있다.

```bash
# 올바른 순서
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

`.bashrc`에 넣었는데 새 터미널에서 패키지를 못 찾으면, AI는 패키지 재설치를 권한다. `source` 문제다. `echo $AMENT_PREFIX_PATH`로 현재 소싱된 workspace를 확인하자.

### 19.2.4 커스텀 메시지와 빌드

AI는 `.msg` 파일은 잘 만든다. 하지만 `CMakeLists.txt`와 `package.xml`의 의존성 추가를 빠뜨린다.

`rosidl_generate_interfaces` 설정이 빠지면 빌드는 되는데 Python에서 import할 때 실패한다. 이 에러가 나면 AI는 "패키지가 설치 안 됐다"고 오진하기 쉽다.

```cmake
# CMakeLists.txt에 반드시 추가
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MyCustomMsg.msg"
  DEPENDENCIES std_msgs geometry_msgs
)
```

```xml
<!-- package.xml에 반드시 추가 -->
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

또 하나: `--symlink-install` 없이 빌드하면 Python 코드 수정이 반영되지 않는다. AI는 이걸 "캐시 문제"라고 오진한다.

```bash
# Python 패키지 수정이 바로 반영되려면
colcon build --symlink-install
```

### 19.2.5 네임스페이스와 리매핑

`ros2 topic echo /camera/image_raw` 했는데 데이터가 안 오면, AI는 드라이버 문제라고 진단한다. 네임스페이스 때문에 토픽 이름이 `/robot1/camera/image_raw`일 수 있는데도.

```bash
# 토픽 목록부터 확인하라
ros2 topic list

# 특정 패턴으로 필터링
ros2 topic list | grep camera
```

AI에게 디버깅을 시킬 때는 `ros2 topic list`와 `ros2 node list` 출력을 먼저 제공해야 한다. 이 정보 없이 "토픽이 안 들어와요"라고 하면 AI는 추측으로 답할 수밖에 없다.

### 19.2.6 Launch 파일

AI가 ROS2 Python launch 파일을 쓸 때 반복적으로 하는 실수들:

- ROS1 XML 문법을 섞는다 (ROS2 launch는 Python이 기본이다)
- `LaunchDescription`의 action 순서를 잘못 잡는다 (노드 의존성 고려 필요)
- `ComposableNode` vs 일반 `Node` 구분을 못 한다
- `PushRosNamespace`를 빠뜨려서 multi-robot 설정이 꼬인다

```python
# multi-robot launch 파일에서 네임스페이스 적용
from launch.actions import GroupAction, PushRosNamespace

robot1_group = GroupAction([
    PushRosNamespace('robot1'),
    Node(package='my_pkg', executable='my_node', name='sensor_node'),
])
```

AI에게 launch 파일을 요청할 때는 "ROS2 Python launch 파일", "multi-robot이면 네임스페이스 적용", "ComposableNode 사용 여부" 등을 명시해야 한다.

## 19.3 Docker에서 AI가 모르는 것들

### 19.3.1 GUI/시각화 문제

Docker 안에서 RViz나 Gazebo 같은 GUI 도구를 띄우려면 X11 포워딩이 필요하다. AI는 `xhost +local:docker`를 권하는데, 이는 모든 로컬 연결에 X 서버 접근을 허용하는 것이라 보안상 위험하다.

제대로 하려면 다음과 같이 설정한다:

```bash
docker run -it \
  --env DISPLAY=$DISPLAY \
  --env QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --ipc=host \
  my_image
```

각 옵션의 역할:
- `QT_X11_NO_MITSHM=1` — AI는 이 옵션을 거의 모른다. 없으면 RViz가 segfault로 죽는다. MIT-SHM(공유 메모리) 확장이 Docker 환경에서 제대로 동작하지 않기 때문이다.
- `--ipc=host` — 호스트와 IPC 네임스페이스를 공유한다. 없으면 shared memory 문제로 시각화 도구가 죽는 경우가 많다.
- Wayland 환경(Ubuntu 22.04+ 기본)에서는 X11 소켓 마운트만으로 안 되는 경우가 있다. `XDG_SESSION_TYPE=x11`로 X11 세션을 강제하거나, XWayland를 거쳐야 할 수 있다.

### 19.3.2 USB 디바이스 패스스루

카메라, LiDAR, IMU 등 USB 장치를 Docker 안에서 쓰려면 디바이스를 명시적으로 매핑해야 한다. AI는 이걸 모르고 "드라이버 설치하라"고 한다.

```bash
# 특정 디바이스만 매핑 (권장)
docker run -it --device=/dev/ttyUSB0 --device=/dev/video0 my_image

# 모든 디바이스 접근 허용 (보안상 비추, 디버깅용으로만)
docker run -it --privileged my_image
```

`--privileged`는 편하지만 컨테이너에 호스트의 거의 모든 권한을 주는 것이므로, 프로덕션에서는 필요한 device만 매핑하는 게 맞다.

한 가지 더: USB 장치가 Docker 컨테이너 시작 후에 꽂히면 인식이 안 된다. AI는 이 상황을 전혀 고려하지 못한다. 컨테이너를 재시작하거나, `--privileged` + `-v /dev:/dev` 조합을 써야 한다.

### 19.3.3 ROS 네트워킹

Docker 컨테이너 간 ROS2 통신에서 `--network=host`가 가장 간단하지만, 호스트의 포트를 전부 공유하므로 포트 충돌 위험이 있다.

AI는 bridge 네트워크에서 ROS2가 왜 안 되는지 설명하지 못한다. 원인은 DDS(Data Distribution Service)가 multicast를 사용하는데, Docker bridge 네트워크에서는 multicast가 기본적으로 안 되기 때문이다.

```bash
# 가장 간단한 방법 (개발 환경에서)
docker run -it --network=host my_ros2_image

# ROS_DOMAIN_ID로 다른 사람과 충돌 방지
docker run -it --network=host -e ROS_DOMAIN_ID=42 my_ros2_image
```

`ROS_DOMAIN_ID`가 같은 네트워크의 다른 사람 ROS2와 충돌할 수 있다. 연구실에서 여러 명이 동시에 ROS2를 쓰면 서로의 토픽이 보이는 상황이 생긴다.

DDS 설정을 세밀하게 해야 할 때는 Cyclone DDS config XML로 특정 네트워크 인터페이스만 사용하게 제한한다:

```xml
<!-- cyclone_dds.xml -->
<CycloneDDS>
  <Domain>
    <General>
      <NetworkInterfaceAddress>eth0</NetworkInterfaceAddress>
    </General>
  </Domain>
</CycloneDDS>
```

```bash
export CYCLONEDDS_URI=file:///path/to/cyclone_dds.xml
```

### 19.3.4 파일 권한 문제

Docker 안에서 생성된 파일은 기본적으로 root 소유다. 호스트에서 편집하거나 삭제하려면 `sudo`가 필요하다.

```bash
# 호스트 유저 권한으로 실행
docker run -it --user $(id -u):$(id -g) my_image
```

하지만 일부 ROS 패키지가 root 권한을 필요로 해서 `--user` 옵션을 쓰면 또 안 되는 경우가 있다. AI는 이런 상황에서 `chmod 777`을 남발하는데, 실무에서는 이러면 안 된다. Dockerfile에서 non-root 유저를 만들고 필요한 디렉토리 권한만 설정하는 것이 올바른 방법이다.

```dockerfile
# Dockerfile에서 non-root 유저 설정
RUN useradd -m -s /bin/bash rosuser && \
    usermod -aG dialout rosuser
USER rosuser
```

## 19.4 하드웨어/드라이버에서 AI가 포기하는 것들

### 19.4.1 시리얼 포트 권한

`/dev/ttyUSB0` 접근 시 `Permission denied`가 뜨면, AI는 `sudo chmod 666 /dev/ttyUSB0`을 권한다. 되긴 되지만, 재부팅하면 리셋된다. 매번 이걸 치고 있을 수는 없다.

올바른 방법은 udev rule을 작성하는 것이다:

```bash
# 벤더/프로덕트 ID 확인
udevadm info -a -n /dev/ttyUSB0 | grep -E 'idVendor|idProduct'
```

```bash
# /etc/udev/rules.d/99-sensors.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", MODE="0666", SYMLINK+="gps"
```

```bash
# udev rule 적용
sudo udevadm control --reload-rules && sudo udevadm trigger
```

이렇게 하면 해당 USB 장치가 항상 `/dev/gps`라는 고정 이름으로 잡히고, 권한도 자동으로 설정된다. 여러 개의 동일 장치를 구분해야 할 때(예: IMU 2개)도 시리얼 넘버로 구분할 수 있다. AI는 udev를 모른다.

### 19.4.2 USB 대역폭

USB3 카메라 3대를 같은 USB 허브에 꽂으면 대역폭 부족으로 프레임 드롭이 발생한다. AI는 "드라이버 업데이트하라" 또는 "해상도를 낮춰라"고 하지만, 실제 원인은 USB 컨트롤러의 대역폭 한계다.

```bash
# 어떤 카메라가 어떤 USB 컨트롤러에 붙어있는지 확인
lsusb -t
```

이 문제는 소프트웨어로 해결할 수 없다. 물리적으로 다른 USB 컨트롤러에 카메라를 분산 연결해야 한다. 데스크탑 PC의 앞면 USB와 뒷면 USB는 다른 컨트롤러에 연결된 경우가 많으니, `lsusb -t`의 Bus 번호를 확인하고 분산 배치하자.

### 19.4.3 LiDAR 연결 (IP 설정)

Velodyne이나 Ouster LiDAR에서 "데이터가 안 온다"는 문제의 90%는 네트워크 설정이 원인이다. AI는 드라이버 재설치나 ROS 패키지 재빌드를 권하지만, 그 전에 확인해야 할 것이 있다.

LiDAR는 고정 IP(예: `192.168.1.201`)를 사용한다. 호스트 PC의 이더넷 인터페이스를 같은 서브넷(예: `192.168.1.100`)으로 설정해야 통신이 된다.

```bash
# 1단계: LiDAR에 ping이 되는지 확인
ping 192.168.1.201

# 2단계: 호스트 이더넷 인터페이스 IP 설정
sudo ip addr add 192.168.1.100/24 dev eth0
sudo ip link set eth0 up

# 3단계: UDP 패킷이 오는지 Wireshark로 확인
sudo tcpdump -i eth0 udp port 2368 -c 10
```

`ping`부터 해보면 대부분 여기서 걸린다. Wireshark(또는 tcpdump)로 UDP 패킷이 오는지 확인하는 게 가장 확실한 디버깅 방법이다. 패킷이 오는데 ROS에서 안 보이면 그때 드라이버를 의심해도 늦지 않다.

### 19.4.4 카메라 드라이버 (v4l2)

AI는 `cv2.VideoCapture(0)`만 알지, `/dev/video*`가 여러 개일 때 어떤 게 실제 카메라인지 구분하지 못한다. USB 카메라 하나를 꽂아도 `/dev/video0`, `/dev/video1`이 생기는 경우가 흔한데(metadata용 디바이스), AI는 이걸 모른다.

```bash
# 카메라 디바이스 매핑 확인
v4l2-ctl --list-devices

# 지원하는 포맷과 해상도 확인
v4l2-ctl -d /dev/video0 --list-formats-ext
```

자동 노출(auto exposure), 자동 화이트밸런스 — 이런 자동 설정이 SLAM 성능을 망치는 경우가 많다. 밝기가 계속 변하면 특징점 추출이 불안정해진다.

```bash
# 수동 노출 설정 (SLAM용)
v4l2-ctl -d /dev/video0 --set-ctrl=exposure_auto=1
v4l2-ctl -d /dev/video0 --set-ctrl=exposure_absolute=100

# 화이트밸런스 고정
v4l2-ctl -d /dev/video0 --set-ctrl=white_balance_automatic=0
```

AI는 이런 low-level 카메라 제어를 모른다. "SLAM이 불안정하다"고 하면 알고리즘 파라미터 튜닝을 권하지만, 카메라 자동 설정을 끄는 것만으로 크게 개선되는 경우가 있다.

### 19.4.5 Jetson (ARM) 환경

AI가 생성한 코드나 Docker 설정은 x86 기준이다. NVIDIA Jetson(ARM64)에서는 안 돌아가는 경우가 많다.

주의해야 할 점:
- `pip install`로 설치할 때 pre-built 바이너리(wheel)가 ARM용으로 없는 패키지가 많다. 특히 `scipy`, `opencv-python`은 소스에서 빌드해야 해서 수십 분이 걸린다.
- JetPack 버전에 따라 CUDA, cuDNN, TensorRT 버전이 고정된다. AI가 최신 버전을 설치하라고 권하면 전체 시스템이 꼬인다.
- Docker를 쓸 때는 NVIDIA에서 제공하는 `l4t`(Linux for Tegra) 기반 이미지를 써야 한다.

```bash
# Jetson에서 Docker 이미지 — x86 이미지 쓰면 안 된다
docker pull nvcr.io/nvidia/l4t-pytorch:r35.2.1-pth2.0-py3

# JetPack 버전 확인
cat /etc/nv_tegra_release
```

AI에게 코드를 요청할 때는 "Jetson Orin, JetPack 5.1.2, CUDA 11.4 환경이다"라고 명시해야 한다.

### 19.4.6 실시간 제어와 타이밍

AI는 `time.sleep(0.01)`로 100Hz 루프를 만들라고 하지만, 정확하지 않다. `time.sleep()`의 정밀도는 OS 스케줄러에 의존하며, 일반 Linux 커널에서는 수 밀리초의 jitter가 발생한다.

```python
# AI가 주로 권하는 방법 (정확하지 않음)
import time
while True:
    do_control()
    time.sleep(0.01)  # 실제로는 10~15ms가 될 수 있다
```

Python의 GIL(Global Interpreter Lock) 때문에 멀티스레드 타이밍은 더 보장이 안 된다. 진짜 실시간 제어가 필요하면 C++과 RT(Real-Time) 커널(PREEMPT_RT)을 써야 한다.

```bash
# 실제 퍼블리시 주파수 확인 — 항상 이걸로 검증하라
ros2 topic hz /cmd_vel
```

AI가 "100Hz로 제어하면 된다"고 해도, `ros2 topic hz`로 실제 주파수를 확인해야 한다. 기대한 주파수와 실제 주파수가 다르면 제어가 제대로 안 된다.

## 19.5 AI 에이전트가 포기하는 패턴들

### 19.5.1 "It works in simulation"

Gazebo에서 잘 되는데 실제 로봇에서 안 되는 상황. AI는 "시뮬레이션에서 되니까 코드는 맞고 하드웨어 문제"라고 결론 낸다. 실제 원인은 sim-to-real gap이다:

- **센서 노이즈**: 시뮬레이션의 가우시안 노이즈와 실제 센서 노이즈는 분포가 다르다
- **통신 지연**: Gazebo 안에서는 토픽 전달이 즉시 이뤄지지만, 실제로는 수~수십 ms의 지연이 있다
- **타이밍 불일치**: 시뮬레이션은 완벽한 동기화를 보장하지만, 실제로는 센서 간 타임스탬프가 어긋난다
- **좌표계 불일치**: URDF와 실제 로봇의 센서 위치/각도가 미세하게 다르면 tf가 틀어진다

AI에게는 "시뮬레이션에서는 되는데 실제 로봇에서 안 된다. 센서 노이즈 수준은 X이고, 통신 지연은 Y ms이고, 좌표계 캘리브레이션은 Z 방법으로 했다"처럼 sim-to-real의 차이를 구체적으로 알려줘야 한다.

### 19.5.2 하드웨어 문제를 소프트웨어로 고치려 함

케이블 불량, 접촉 불량, 전원 부족 — AI는 이런 물리적 문제를 진단할 수 없다.

"센서 데이터가 간헐적으로 끊긴다"고 하면 AI는 버퍼 크기 조절, 타임아웃 설정, QoS 변경 등을 권한다. USB 케이블이 느슨하거나 USB 허브의 전원이 부족한 것인데도.

```bash
# 커널 로그에서 하드웨어 문제 단서 찾기
dmesg | tail -20

# USB 연결 해제/재연결 이벤트 확인
dmesg | grep -i usb | tail -20
```

`dmesg`에 `USB disconnect`, `device descriptor read/64, error -71` 같은 메시지가 보이면 소프트웨어 문제가 아니다. 케이블을 교체하거나, 유전원 USB 허브를 쓰거나, 다른 포트에 꽂아보는 게 먼저다.

### 19.5.3 환경 문제 진단 포기

라이브러리 버전 충돌이 복잡하게 얽히면 AI는 "전부 재설치하라"고 한다. `pip show package_name`으로 버전을 확인하고 어떤 것이 충돌하는지 좁히는 게 먼저다.

특히 OpenCV 관련 충돌은 로보틱스의 클래식이다:

- `opencv-python` (기본)
- `opencv-python-headless` (GUI 없는 서버용)
- `opencv-contrib-python` (추가 모듈 포함)
- `cv_bridge` (ROS 패키지, 자체 OpenCV를 참조)

이 네 가지가 동시에 설치되면 서로 충돌한다.

```bash
# 현재 설치된 OpenCV 확인
pip show opencv-python opencv-python-headless opencv-contrib-python

# 해결: ROS 환경에서는 pip opencv를 설치하지 않는다
pip uninstall opencv-python opencv-python-headless opencv-contrib-python
sudo apt install ros-humble-cv-bridge
```

ROS 환경에서는 `apt install ros-humble-cv-bridge`만 쓰고, pip으로 opencv를 따로 설치하지 않는 것이 가장 깔끔하다.

### 19.5.4 "2-3번 시도 후 포기"

AI는 같은 접근을 두세 번 반복하다 안 되면 "다른 방법을 시도해 보세요"라며 넘긴다. 엔지니어는 여기서 포기하지 않는다. 로그를 뒤지고, strace를 걸고, 패킷을 캡처한다.

AI에게 더 좋은 답을 얻으려면, 에러 메시지뿐 아니라 low-level 정보를 함께 줘야 한다:

```bash
# 시스템 로그
dmesg | tail -30
journalctl -u my_service --since "5 minutes ago"

# 프로세스 추적
strace -f -e trace=open,read,write ros2 run my_pkg my_node 2>&1 | head -100

# 네트워크 패킷 캡처
sudo tcpdump -i eth0 -w capture.pcap
```

이런 정보를 AI에게 제공하면, "재설치하세요" 대신 실제 원인에 가까운 답을 받을 수 있다.

## 19.6 AI 에이전트를 제대로 쓰는 법

*AI 부려먹기의 일반 frame (writing·reading 영역)은 [`../research-notes/chapter_07_keshav_three_passes.md`](../research-notes/chapter_07_keshav_three_passes.md) + [`part2_writing/A_workflow/ch01_mindset.md`](../research-notes/chapter_16_mindset.md) 에서 본격 다룬다. 본 § 19.6은 코드·하드웨어 영역의 분야 적용.*

### 19.6.1 컨텍스트를 충분히 제공하라

AI에게 질문할 때 가장 중요한 것은 컨텍스트의 양과 질이다.

틀린 예: "카메라가 안 돼요"

맞는 예: "Ubuntu 22.04, ROS2 Humble, Intel RealSense D435, `rs-enumerate-devices`에서는 보이는데 `ros2 launch realsense2_camera rs_launch.py`하면 `Could not open device` 에러가 난다. Docker 안에서 실행 중이고, `--device=/dev/video0`은 매핑했다."

**AI에게 제공해야 할 정보 체크리스트**:
- OS 버전, ROS 버전
- 하드웨어 플랫폼 (x86 vs ARM/Jetson)
- 센서 모델명
- 에러 메시지 전문 (일부만 복사하지 말고 전체를 줘라)
- `ros2 topic list`, `ros2 node list` 출력
- Docker 사용 여부와 실행 옵션 (`docker run` 명령 전체)
- 네트워크 구성 (유선/무선, IP 대역)

### 19.6.2 AI의 답을 검증하는 방법

AI의 답을 그대로 실행하기 전에 다음을 확인하라:

- **"이 패키지를 설치하라"** → 해당 패키지가 자기 ROS 버전과 Ubuntu 버전을 지원하는지 먼저 확인. `apt search ros-humble-<패키지명>`으로 존재 여부를 체크한다.
- **"이 설정을 바꿔라"** → 바꾸기 전에 현재 설정을 백업하고, 왜 바꿔야 하는지 근거를 AI에게 물어본다. 근거를 설명하지 못하면 의심하라.
- **"재설치하라"** → 90%는 재설치 안 해도 된다. 먼저 정확한 에러 원인을 좁혀라. `pip show`, `dpkg -l | grep`, `apt policy` 등으로 현재 상태를 확인하는 게 먼저다.
- **AI가 코드를 줄 때** → 하드코딩된 경로(`/home/user/...`), 하드코딩된 IP(`192.168.1.100`), x86 전용 패키지(`amd64` wheel) 등이 포함되어 있는지 확인한다.

### 19.6.3 AI가 잘하는 것 vs 못하는 것

| AI가 잘하는 것 | AI가 못하는 것 |
|---|---|
| 알고리즘 구현 (SLAM, detection 등) | 하드웨어 디버깅 |
| ROS2 노드/서비스 코드 작성 | QoS/DDS 설정 최적화 |
| Python/C++ 코드 리팩토링 | USB/시리얼 권한 문제 |
| 논문 읽기/요약 | 네트워크 설정 (LiDAR IP 등) |
| CMakeLists.txt 작성 | 실시간 타이밍 문제 |
| 데이터 전처리 파이프라인 | Docker 안에서의 하드웨어 접근 |
| 시각화 코드 (matplotlib, Open3D) | 센서 간 시간 동기화 실전 |
| 에러 메시지 해석 (일반적인) | dmesg/커널 로그 기반 디버깅 |

AI는 "순수 소프트웨어" 영역에서는 강하지만, 하드웨어와 소프트웨어가 만나는 경계에서 약하다. 로보틱스 문제의 대부분이 그 경계에서 발생한다. AI가 강한 영역은 맡기고, AI가 약한 영역에서는 직접 디버깅한 결과를 AI에게 먹여서 분석하게 하는 게 맞다.

## 19.7 AI를 연구 도구로 쓰는 워크플로우

로보틱스 연구자의 하루 일과에서 AI가 어디에 쓰이는지 구체적으로 본다.

### 19.7.1 논문 읽기

*논문 읽기 워크플로우 (3-pass + AI layer cameo)는 [`../research-notes/chapter_07_keshav_three_passes.md`](../research-notes/chapter_07_keshav_three_passes.md) 의 AI layer cameo에서 본격 다룬다.*

분야 적용은 분야 핵심 논문에 3-pass + AI summary 결합 — abstract 읽고 contribution 3줄 요청, Eq. 단계별 유도 요청.

### 19.7.2 코드 작성

- 프로토타이핑: "KITTI 데이터셋에서 ORB 특징점 뽑아서 매칭하는 코드 짜줘. OpenCV 쓰고, Lowe's ratio test 0.75로" — 이런 식으로 구체적으로 지시
- 디버깅: 에러 메시지 + 코드 + "이 에러의 원인이 뭐야" — AI가 잘하는 영역
- 리팩토링: "이 코드를 PyTorch Dataset 클래스로 바꿔줘" — 구조 변환에 강함
- AI가 못하는 것 (이 장 앞부분 참조): ROS QoS, 하드웨어 권한, 네트워크 설정, 실시간 타이밍

### 19.7.3 실험 설계

*실험 설계·ablation·결과 해석에 AI 부려먹기 frame은 [`../research-notes/chapter_32_revision_rebuttal.md`](../research-notes/chapter_32_revision_rebuttal.md) (또는 [`ch12_figures.md`](../research-notes/chapter_27_figures.md)) 에 cameo로 담겼다.*

분야 적용은 baseline 비교 표를 AI에게 던지고 *내가 놓친 비교 축*을 묻는 워크플로우.

### 19.7.4 논문 쓰기

- 초고 작성: 핵심 아이디어와 실험 결과를 AI에게 주고 "Introduction 초고를 써줘" — 구조 잡기에 유용
- 문법/표현 교정: 영어 논문의 어색한 표현 수정. Grammarly보다 AI가 context를 더 잘 이해한다
- 주의: AI가 쓴 문장을 그대로 제출하면 안 된다. 본인의 voice로 다시 써야 한다. 리뷰어는 AI 생성 문체를 알아본다
- BibTeX 생성: "이 논문의 BibTeX를 만들어줘" — Google Scholar에서 복사하는 것보다 빠를 때가 있다. 단, AI가 year이나 venue를 틀리는 경우가 있으니 반드시 검증

### 19.7.5 일상 워크플로우 예시

하루 일과에서 AI를 어떻게 쓰는지 구체적 시나리오:

```
09:00 — 새 논문 3편 arXiv에서 확인. AI에게 각각 1문장 요약 요청
09:30 — 흥미로운 1편 선택, 2패스 읽기. 모르는 수식은 AI에게 유도 요청
10:30 — 어제 학습 결과 분석. loss curve 캡처해서 AI에게 "이 패턴이 정상인가?" 확인
11:00 — 새 실험 코드 작성. AI에게 DataLoader 구조 생성 시킴. 수동으로 augmentation 로직 수정
14:00 — SLAM 코드 디버깅. ROS2 에러 → 이 장 참고해서 직접 해결 (AI는 QoS를 모른다)
16:00 — 논문 Related Work 섹션 초고. AI에게 비교 논문 5편의 차이점 표 만들게 시킴
17:00 — 표 검증. AI가 2편의 method를 혼동한 것 발견, 수동 수정
```
