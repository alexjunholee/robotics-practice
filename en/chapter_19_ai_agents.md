# Ch.19 — Using AI Coding Agents

## 19.1 The Order of Work at the Robot

AI coding agents are useful for small ROS2 nodes, launch files, log summaries, and experiment-script cleanup. In a robot experiment, first capture the current state in command output: whether the sensors are visible, the topics exist, the QoS settings match, and the devices are mapped into the container.

A detailed workflow is available in [*Researching with AI and Robotics*, Ch.9 — Robots Fail Outside the Code](../../ai-research-practice/guide.html#ch-9-로봇은-코드-밖에서-실패한다). This chapter keeps only the runtime observations needed while reading `robotics-practice`.

## 19.2 Runtime Signals to Capture First

The robot's current state is not visible from code alone. Check ROS2 topics, DDS QoS, `/clock`, the TF buffer, Docker device mapping, the USB bus, the LiDAR IP address, camera exposure, and the Jetson architecture in command output.

When a problem appears, first inspect these signals.

```bash
ros2 topic list
ros2 topic info /camera/image_raw --verbose
ros2 node list
ros2 topic hz /cmd_vel
dmesg | tail -30
lsusb -t
```

For LiDAR and networked sensors, inspect the packets first.

```bash
ping 192.168.1.201
sudo tcpdump -i eth0 udp port 2368 -c 10
```

For cameras, inspect the device and exposure settings.

```bash
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --list-formats-ext
```

Without these outputs, defer the diagnosis. Correct code still fails when the device is missing inside the container, QoS does not match, or multiple sensors share the same USB controller.

## 19.3 Information to Attach to a Question

For a robot runtime problem, attach at least the following bundle.

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

This bundle is the basis for evaluating an answer. Before executing a suggestion, verify package support, the current configuration, device permissions, the target architecture, and the metric conditions.

## 19.4 Where to Read Next

For a detailed checklist on robot runtime and collaboration with an AI agent, see [`ai-research-practice` Ch.9](../../ai-research-practice/guide.html#ch-9-로봇은-코드-밖에서-실패한다).

Chapters 1–8 of the same guide cover the general principles for turning an AI answer into research action. Reading them together with the runtime checklist clarifies how to compare papers with code, preserve the conditions behind experimental numbers, and connect reviewer claims to evidence.
