# Ch.19 — AI 코딩 에이전트 활용하기

## 19.1 로봇 앞에서 에이전트를 쓰는 순서

AI 코딩 에이전트는 작은 ROS2 노드, launch 파일, 로그 요약, 실험 스크립트 정리에 쓸 만하다. 로봇 실험에서는 먼저 현재 상태를 출력으로 남겨야 한다. 센서가 보이는지, topic이 떠 있는지, QoS가 맞는지, container 안에 device가 들어왔는지부터 확인한다.

자세한 운영 절차는 [`AI와 로봇 연구하기` Ch.9 — 로봇은 코드 밖에서 실패한다](../ai-research-practice/guide.html#ch-9-로봇은-코드-밖에서-실패한다)에 모아 두었다. 여기에는 `robotics-practice`를 읽는 동안 바로 붙여 쓸 런타임 관측값만 둔다.

## 19.2 먼저 남길 런타임 신호

로봇의 현재 상태는 코드만으로 드러나지 않는다. ROS2 topic, DDS QoS, `/clock`, TF buffer, Docker device mapping, USB bus, LiDAR IP, camera exposure, Jetson architecture는 명령 출력으로 확인한다.

문제가 생기면 먼저 다음 신호를 본다.

```bash
ros2 topic list
ros2 topic info /camera/image_raw --verbose
ros2 node list
ros2 topic hz /cmd_vel
dmesg | tail -30
lsusb -t
```

LiDAR나 네트워크 센서는 packet부터 본다.

```bash
ping 192.168.1.201
sudo tcpdump -i eth0 udp port 2368 -c 10
```

카메라는 device와 노출 설정을 본다.

```bash
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --list-formats-ext
```

이 출력이 없으면 원인 판단을 보류한다. 코드가 맞아도 device가 container 안에 없거나, QoS가 맞지 않거나, 센서가 같은 USB controller에 몰려 있으면 로봇은 움직이지 않는다.

## 19.3 질문에 붙일 정보

로봇 runtime 문제를 다룰 때는 최소한 다음 묶음을 붙인다.

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

이 묶음은 답을 채점하는 기준이다. 실행 전에는 package 지원 범위, 현재 설정, device 권한, target architecture, metric 조건을 다시 확인한다.

## 19.4 다음으로 읽을 곳

로봇 runtime과 AI 에이전트 협업의 상세 체크리스트는 [`ai-research-practice` Ch.9](../ai-research-practice/guide.html#ch-9-로봇은-코드-밖에서-실패한다)에 있다.

AI 답변을 연구 행동으로 옮기는 일반 원칙은 같은 가이드의 Ch.1-8에서 다룬다. 논문과 코드를 맞춰 보는 법, 실험 숫자의 조건, reviewer 답변의 주장과 근거까지 함께 읽으면 runtime 체크리스트의 쓰임이 분명해진다.
