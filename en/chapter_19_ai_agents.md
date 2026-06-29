# Ch.19 — AI Coding Agents and Robot Runtime

## 19.1 Where This Chapter Now Points

The detailed guide for using AI in robot research has moved to [`AI와 로봇 연구하기` Ch.9 — 로봇은 코드 밖에서 실패한다](../../ai-research-practice/guide.html#ch-9-로봇은-코드-밖에서-실패한다).

This chapter keeps only the bridge inside `robotics-practice`: how the SLAM, sensor, Docker, dataset, and framework material from the previous chapters meets AI coding agents at the runtime boundary.

## 19.2 Robot Failures Start Outside The Code

AI can read code and error messages quickly, but the robot's current state lives outside the chat. ROS2 topics, DDS QoS, `/clock`, TF buffers, Docker device mapping, USB buses, LiDAR IP addresses, camera exposure, and Jetson architecture must be checked with real command output.

When a problem appears, first inspect these signals.

```bash
ros2 topic list
ros2 topic info /camera/image_raw --verbose
ros2 node list
ros2 topic hz /cmd_vel
dmesg | tail -30
lsusb -t
```

For LiDAR and networked sensors, inspect packets before rebuilding code.

```bash
ping 192.168.1.201
sudo tcpdump -i eth0 udp port 2368 -c 10
```

For cameras, inspect the device and exposure controls.

```bash
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --list-formats-ext
```

Without these outputs, the AI's answer is only a candidate. Correct code still fails when the device is missing inside the container, QoS does not match, or multiple sensors share the same USB controller.

## 19.3 What To Give The AI Before Asking

For robot runtime problems, give the AI this minimum bundle.

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

This bundle is not evidence that the AI is right. It is the evidence used to check the AI's explanation. Before executing a suggestion, verify package support, current configuration, device permissions, target architecture, and metric conditions.

## 19.4 What To Read Next

The full runtime checklist now lives in [`ai-research-practice` Ch.9](../../ai-research-practice/guide.html#ch-9-로봇은-코드-밖에서-실패한다).

The general rules for turning AI answers into research actions live in Ch.1-8 of that guide. Read them together with this runtime bridge when connecting code, experiments, paper claims, and reviewer responses.
