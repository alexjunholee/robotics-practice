# 18장 AI 코딩 에이전트 활용

4부에서 개발 환경과 데이터셋을 갖추었다면, 이제 실제 연구와 개발 현장에서 자주 손에 쥐는 도구를 살펴볼 차례다. 최근 로보틱스 연구실에서도 Claude나 Copilot 같은 AI 코딩 에이전트로 작은 ROS2 노드나 launch 파일을 작성하고, 긴 로그를 요약하거나 실험 스크립트를 정리하는 일이 자연스러워졌다. 다만 물리적 하드웨어와 실시간 통신이 얽힌 로봇 환경에서는 막연히 "안 된다"고 묻기보다 현재 로봇의 상태를 먼저 확보해 건네야 에이전트가 제 역할을 할 수 있다.

## 18.1 로봇 앞에서 에이전트를 쓰는 순서

AI 코딩 에이전트는 코드 문법이나 보일러플레이트 작성에는 강하지만, 지금 눈앞의 로봇에 어떤 센서가 꽂혀 있고 네트워크가 어떻게 물려 있는지는 알지 못한다. 따라서 로봇 실험 중 문제가 생겼을 때는 에이전트에게 바로 해결책을 묻기 전에 현재 상태를 명령 출력으로 남겨 두어야 한다. 먼저 센서가 운영체제에 잡히는지, 기대한 topic이 실제로 발행되고 있는지, 발행자와 구독자 사이에 QoS(Quality of Service, 통신 품질 설정)가 맞는지, 그리고 container 안으로 device 파일이 제대로 들어왔는지를 차례로 확인한다.

## 18.2 먼저 남길 런타임 신호

18.1절의 기본 점검 항목 외에도 실제 로봇 시스템에서는 `/clock` 동기화 여부, TF buffer 상태, USB bus 대역폭, LiDAR IP 연결, 카메라 노출(exposure), Jetson 같은 target architecture 차이까지 함께 살펴야 한다.

문제가 발생하면 추측으로 코드를 고치기 전에 아래 명령들을 실행해 ROS2 그래프와 커널·USB 상태 신호부터 모은다.

```bash
ros2 topic list
ros2 topic info /camera/image_raw --verbose
ros2 node list
ros2 topic hz /cmd_vel
sudo dmesg | tail -30
lsusb -t
```

LiDAR처럼 이더넷으로 통신하는 네트워크 센서가 반응하지 않을 때는 코드보다 패킷 도달 여부부터 확인하면 곧바로 원인을 가려낼 수 있다.

```bash
ping 192.168.1.201
sudo tcpdump -i eth0 udp port 2368 -c 10
```

카메라 드라이버가 영상을 띄우지 못한다면 장치 노드가 잡혔는지, 그리고 지원 포맷·해상도가 무엇인지 먼저 조회한다.

```bash
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --list-formats-ext
```

원인은 이 출력을 모두 확인한 뒤에 판단한다. 파이썬이나 C++ 코드가 문법적으로 완벽해도 필수 device가 container에 마운트되지 않았거나, DDS QoS 조합이 서로 호환되지 않거나, 여러 센서가 하나의 USB controller에 몰려 대역폭과 전력이 부족해지면 로봇은 그대로 멈춰 선다.

## 18.3 질문에 붙일 정보

로봇 런타임 문제를 에이전트에게 넘길 때는 증상만 한 줄 적기보다 최소한 아래 항목을 채워 함께 전달한다. 이 맥락이 있어야 에이전트가 엉뚱한 가정을 하지 않고 실제 환경에 맞는 원인을 짚어 낸다.

```text
OS / ROS version:
hardware platform:
sensor model:
full error message:
ros2 topic list:
ros2 topic info --verbose:
ros2 node list:
docker run command:
network / IP range:
dmesg or device log:
what changed since last working run:
```

에이전트가 답을 내놓았다고 해서 곧장 믿고 실행해서는 안 된다. 돌려받은 답 역시 위에서 모은 사실에 비추어 검증해야 한다. 명령이나 코드를 적용하기 전에는 패키지 지원 범위, 현재 설정 파일, device 접근 권한, target architecture(예: x86_64 vs arm64), 평가 metric 조건을 다시 확인한다.

로봇 런타임에서 AI 에이전트와 협업할 때 활용할 상세 체크리스트는 자매 가이드에 따로 정리해 두었다(→ [`ai-research-practice` 11장과 부록 F](../ai-research-practice/guide.html#part-03-rules-11장-로봇-실험의-조건을-기록한다)).
