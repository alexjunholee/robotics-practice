# Ch.19 — AI 코딩 에이전트 활용하기

## 19.1 로봇 앞에서 에이전트를 쓰는 순서

AI 코딩 에이전트는 작은 ROS2 노드나 launch 파일을 쓰고, 로그를 요약하고, 실험 스크립트를 정리하는 데 쓸 만하다. 로봇 실험에서는 에이전트에게 묻기 전에 로봇의 현재 상태를 명령 출력으로 남긴다. 센서가 보이는지, topic이 떠 있는지, QoS(Quality of Service, 통신 품질 설정)가 맞는지, container 안에 device가 들어왔는지부터 확인한다.

## 19.2 먼저 남길 런타임 신호

19.1절의 항목에 더해 확인할 대상은 `/clock`, TF buffer, USB bus, LiDAR IP, camera exposure, Jetson architecture다.

문제가 생기면 먼저 다음 신호를 본다.

```bash
ros2 topic list
ros2 topic info /camera/image_raw --verbose
ros2 node list
ros2 topic hz /cmd_vel
sudo dmesg | tail -30
lsusb -t
```

LiDAR나 네트워크 센서는 packet부터 본다.

```bash
ping 192.168.1.201
sudo tcpdump -i eth0 udp port 2368 -c 10
```

아래 명령으로 카메라 device와 지원 영상 포맷을 본다.

```bash
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --list-formats-ext
```

원인은 이 출력을 본 뒤에 판단한다. 코드가 맞아도 필수 device가 container 안에 빠졌거나, QoS 조합이 호환되지 않거나, 센서가 같은 USB controller에 몰려 대역폭·전력이 부족하면 로봇이 멈춘다.

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

에이전트의 답이 맞는지도 이 정보에 비춰 확인한다. 답을 실행하기 전에는 package 지원 범위, 현재 설정, device 권한, target architecture, metric 조건을 다시 확인한다.

## 19.4 다음으로 읽을 곳

로봇 runtime과 AI 에이전트 협업의 상세 체크리스트는 [`ai-research-practice` 11장과 부록 F](../ai-research-practice/guide.html#part-03-rules-11장-로봇-실험의-조건을-기록한다)에 있다.

AI 답변을 연구에 옮겨 쓰는 일반 원칙, 논문과 코드를 맞춰 보는 법, 실험 숫자에 붙는 조건, reviewer 답변의 주장과 근거는 같은 가이드의 1–10장과 12–13장에 있다.
