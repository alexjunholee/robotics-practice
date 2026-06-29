# Ch.19 — AI 코딩 에이전트와 로봇 런타임

## 19.1 이 장의 위치

AI와 함께 로봇 연구를 운영하는 방법은 별도 가이드로 분리했다. 자세한 내용은 [`AI와 로봇 연구하기` Ch.9 — 로봇은 코드 밖에서 실패한다](../ai-research-practice/guide.html#ch-9-로봇은-코드-밖에서-실패한다)에서 본다.

이 장은 `robotics-practice` 안에서의 연결만 남긴다. 앞 장들에서 배운 SLAM, 센서, Docker, 데이터셋, 프레임워크 지식이 AI 에이전트와 만날 때 어디서 흔들리는지 확인하는 관문이다.

## 19.2 로봇 문제는 코드 밖에서 먼저 갈린다

AI는 코드와 에러 메시지를 빠르게 읽지만, 로봇의 현재 상태는 대화 밖에 있다. ROS2 topic, DDS QoS, `/clock`, TF buffer, Docker device mapping, USB bus, LiDAR IP, camera exposure, Jetson architecture는 실제 명령 출력으로 확인해야 한다.

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

이 출력이 없으면 AI의 답은 후보일 뿐이다. 코드가 맞아도 device가 container 안에 없거나, QoS가 맞지 않거나, 센서가 같은 USB controller에 몰려 있으면 로봇은 움직이지 않는다.

## 19.3 AI에게 질문하기 전에 붙일 자료

로봇 runtime 문제를 AI에게 넘길 때는 최소한 다음 묶음을 함께 준다.

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

이 묶음은 AI를 믿기 위한 자료가 아니다. AI가 낸 설명을 검증할 기준이다. 답을 실행하기 전에는 package 지원 범위, 현재 설정, device 권한, target architecture, metric 조건을 다시 확인한다.

## 19.4 다음으로 읽을 곳

로봇 runtime과 AI 에이전트 협업의 상세 체크리스트는 [`ai-research-practice` Ch.9](../ai-research-practice/guide.html#ch-9-로봇은-코드-밖에서-실패한다)에 있다.

AI 답변을 연구 행동으로 옮기는 일반 원칙은 같은 가이드의 Ch.1-8에서 다룬다. 논문과 코드를 맞춰 보는 법, 실험 숫자의 조건, reviewer 답변의 주장·근거 경계까지 함께 읽으면 이 장의 runtime 체크리스트가 어디에 쓰이는지 보인다.
