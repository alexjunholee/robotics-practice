# Ch.19 — Using AI Coding Agents

## 19.1 The Order of Work at the Robot

AI coding agents are genuinely helpful for drafting small ROS2 nodes, launch files, log summaries, and cleanup scripts for experiments. When debugging on a physical robot, however, always start by capturing live runtime state in terminal output: verify that sensors are enumerated, expected topics exist, QoS profiles match, and hardware devices are mapped into the container.

For the full workflow, see [*Researching with AI and Robotics*, Ch.11 — Record the Conditions of Robot Experiments](../../ai-research-practice/guide.html#part-03-rules-11장-로봇-실험의-조건을-기록한다). This chapter summarizes only the runtime checks you need on hand while working through `robotics-practice`.

## 19.2 Runtime Signals to Capture First

You cannot read a robot's current state from source code alone. Check ROS2 topics, DDS QoS profiles, `/clock`, the TF buffer, Docker device mappings, the USB bus, the LiDAR IP address, camera exposure, and the Jetson architecture directly from live command output.

When something breaks, capture these baseline signals first:

```bash
ros2 topic list
ros2 topic info /camera/image_raw --verbose
ros2 node list
ros2 topic hz /cmd_vel
sudo dmesg | tail -30
lsusb -t
```

If you are debugging LiDAR or other networked sensors, verify packet delivery next:

```bash
ping 192.168.1.201
sudo tcpdump -i eth0 udp port 2368 -c 10
```

For camera issues, check which video devices are attached and which pixel formats or frame rates they advertise:

```bash
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --list-formats-ext
```

Hold off on guessing a root cause until you have these outputs. Even bug-free code will fail if a device node never made it into the container, if publisher and subscriber QoS profiles are incompatible, or if multiple sensors share the same USB controller and run short of bandwidth or power.

## 19.3 Information to Attach to a Question

Whenever you ask a colleague or an AI agent to help diagnose a robot runtime issue, attach at least the following context bundle:

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

This bundle gives anyone reading the question a concrete basis for evaluating a fix. Before running any suggested command or patch, verify package support, your active configuration, device permissions, the target CPU/GPU architecture, and the metric conditions.

## 19.4 Where to Read Next

For a complete checklist on robot runtime logs and collaborating effectively with AI agents, see [`ai-research-practice` Ch.11 and Appendix F](../../ai-research-practice/guide.html#part-03-rules-11장-로봇-실험의-조건을-기록한다).

Chapters 1–10 and 12–13 of that same guide lay out broader principles for turning AI-assisted answers into sound research practice. Reading them alongside the runtime checklist helps you compare papers against actual code, keep experimental conditions intact behind every reported number, and tie reviewer claims directly to evidence.
