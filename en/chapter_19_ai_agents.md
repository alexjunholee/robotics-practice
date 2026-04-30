# Ch.19 — Collaborating with AI Coding Agents

## 19.1 Why This Chapter Is Needed

AI coding agents (Claude, Copilot, ChatGPT, and so on) are powerful tools in general software development. Ask for a single function and you get usable code; paste an error message and they usually pin down the cause. Robotics is different. Hardware, OS, networking, and real-time constraints are tangled together, and "the code is right but it doesn't work" is an everyday situation. AI often gets things wrong in this territory, confidently proposes the wrong direction, or simply gives up.

This chapter is a guide for not getting fooled by AI and for putting it to proper use. Everything here comes from problems encountered in practice. Knowing the patterns where AI goes wrong in advance reduces wasted effort.

## 19.2 Things AI Frequently Gets Wrong in ROS

### 19.2.1 QoS Settings

AI almost always ignores QoS (Quality of Service) or leaves it at the default (RELIABLE). The problem is that sensor topics (cameras, LiDAR) are typically published as BEST_EFFORT. If the subscriber is RELIABLE, no data arrives at all. There is no error message — it just silently fails, so AI starts debugging in the wrong direction with "is the topic missing?"

```bash
# Check the topic's QoS profile
ros2 topic info /camera/image_raw --verbose
```

Check the output for information like `Reliability: BEST_EFFORT`, `Durability: VOLATILE`, and match the subscriber's QoS.

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=10
)
self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, qos)
```

When asking AI for code, explicitly state "this topic's QoS is BEST_EFFORT / SENSOR_DATA." Otherwise it will generate defaults, data won't arrive, and AI won't figure out the cause.

### 19.2.2 use_sim_time and tf2 Timing

If you play back a rosbag without `use_sim_time:=true`, every tf lookup fails. When AI sees a "tf2 lookup failed" error, it will most likely suggest adding a `static_transform_publisher`. Wrong direction.

The real cause is a mismatch between the simulation clock and the system clock. The bag file's timestamps are in the past, while the node queries tf based on the current system time, so of course it can't find anything.

```bash
# Correct rosbag playback
ros2 bag play my_bag --clock

# Enable sim_time when launching the node
ros2 launch my_package my_launch.py use_sim_time:=true
```

tf2 lookups need a timeout and try/except, and AI often leaves these out.

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

### 19.2.3 Workspace Sourcing Order

The sourcing order of ROS2 workspaces matters. Source `/opt/ros/humble/setup.bash` first, then `~/ros2_ws/install/setup.bash`. AI often sources only one, reverses the order, or doesn't know about overlay workspaces at all.

```bash
# Correct order
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

If it's in `.bashrc` but a new terminal can't find the package, AI will recommend reinstalling the package. Most of the time it's a `source` problem. Check the currently sourced workspaces with `echo $AMENT_PREFIX_PATH`.

### 19.2.4 Custom Messages and Build

AI is good at writing `.msg` files. But it often omits dependency additions to `CMakeLists.txt` and `package.xml`.

If `rosidl_generate_interfaces` is missing, the build succeeds but imports fail in Python. Faced with this error, AI is likely to misdiagnose it as "the package isn't installed."

```cmake
# Must be added to CMakeLists.txt
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MyCustomMsg.msg"
  DEPENDENCIES std_msgs geometry_msgs
)
```

```xml
<!-- Must be added to package.xml -->
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

One more: building without `--symlink-install` means Python code changes are not reflected. AI misdiagnoses this as "a cache problem."

```bash
# So Python package changes take effect immediately
colcon build --symlink-install
```

### 19.2.5 Namespaces and Remapping

If `ros2 topic echo /camera/image_raw` produces no data, AI is prone to diagnose it as a driver issue. But the topic name might actually be `/robot1/camera/image_raw` due to a namespace.

```bash
# Check the topic list first
ros2 topic list

# Filter by a specific pattern
ros2 topic list | grep camera
```

When letting AI debug, provide the output of `ros2 topic list` and `ros2 node list` first. Without this information, saying "the topic isn't coming through" leaves AI no choice but to guess.

### 19.2.6 Launch Files

Common mistakes AI makes when writing ROS2 Python launch files:

- Mixes in ROS1 XML syntax (ROS2 launch is Python by default)
- Orders `LaunchDescription` actions incorrectly (node dependencies must be considered)
- Can't distinguish `ComposableNode` from a regular `Node`
- Omits `PushRosNamespace`, tangling up multi-robot setups

```python
# Applying a namespace in a multi-robot launch file
from launch.actions import GroupAction, PushRosNamespace

robot1_group = GroupAction([
    PushRosNamespace('robot1'),
    Node(package='my_pkg', executable='my_node', name='sensor_node'),
])
```

When asking AI for a launch file, specify "ROS2 Python launch file," "apply namespaces if multi-robot," "whether to use ComposableNode," and so on.

## 19.3 Things AI Doesn't Know About Docker

### 19.3.1 GUI/Visualization Issues

To run GUI tools like RViz or Gazebo inside Docker, X11 forwarding is required. AI commonly recommends `xhost +local:docker`, but this allows X server access from all local connections and is a security risk.

The proper setup is:

```bash
docker run -it \
  --env DISPLAY=$DISPLAY \
  --env QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --ipc=host \
  my_image
```

Roles of each option:
- `QT_X11_NO_MITSHM=1` — AI is almost never aware of this option. Without it, RViz dies with a segfault. The reason is that the MIT-SHM (shared memory) extension doesn't work properly inside Docker.
- `--ipc=host` — shares the IPC namespace with the host. Without it, visualization tools often die due to shared memory issues.
- On Wayland environments (default on Ubuntu 22.04+), mounting the X11 socket alone may not be enough. You may need to force an X11 session with `XDG_SESSION_TYPE=x11` or route through XWayland.

### 19.3.2 USB Device Passthrough

To use USB devices like cameras, LiDAR, or IMUs inside Docker, devices must be mapped explicitly. AI usually doesn't know this and says "install the driver."

```bash
# Map only specific devices (recommended)
docker run -it --device=/dev/ttyUSB0 --device=/dev/video0 my_image

# Allow access to all devices (not recommended for security; only for debugging)
docker run -it --privileged my_image
```

`--privileged` is convenient but gives the container almost every host permission, so in production you should map only the devices you need.

One more thing: if a USB device is plugged in after the Docker container starts, it won't be recognized. AI doesn't consider this situation at all. You have to restart the container, or use the `--privileged` + `-v /dev:/dev` combination.

### 19.3.3 ROS Networking

For ROS2 communication between Docker containers, `--network=host` is the simplest, but it shares all of the host's ports and risks port conflicts.

AI often doesn't know why ROS2 fails on a bridge network. The reason is that DDS (Data Distribution Service) uses multicast, and multicast is off by default on Docker bridge networks.

```bash
# Simplest approach (for dev environments)
docker run -it --network=host my_ros2_image

# Prevent conflicts with others via ROS_DOMAIN_ID
docker run -it --network=host -e ROS_DOMAIN_ID=42 my_ros2_image
```

`ROS_DOMAIN_ID` can conflict with another person's ROS2 on the same network. When multiple people use ROS2 simultaneously in a lab, each other's topics can become visible.

When DDS needs fine-grained configuration, restrict it to a specific network interface via a Cyclone DDS config XML:

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

### 19.3.4 File Permission Issues

Files created inside Docker are owned by root by default. Editing or deleting them from the host requires `sudo`.

```bash
# Run as the host user
docker run -it --user $(id -u):$(id -g) my_image
```

But some ROS packages need root, and using `--user` breaks them. AI throws around `chmod 777` in this situation, which you should not do in practice. The correct approach is to create a non-root user in the Dockerfile and set permissions only on the directories that need them.

```dockerfile
# Non-root user setup in the Dockerfile
RUN useradd -m -s /bin/bash rosuser && \
    usermod -aG dialout rosuser
USER rosuser
```

## 19.4 Things AI Gives Up On in Hardware/Drivers

### 19.4.1 Serial Port Permissions

When `Permission denied` appears on `/dev/ttyUSB0`, AI recommends `sudo chmod 666 /dev/ttyUSB0`. It works, but resets on reboot. You can't type this every single time.

The correct approach is to write a udev rule:

```bash
# Check vendor / product IDs
udevadm info -a -n /dev/ttyUSB0 | grep -E 'idVendor|idProduct'
```

```bash
# /etc/udev/rules.d/99-sensors.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", MODE="0666", SYMLINK+="gps"
```

```bash
# Apply the udev rule
sudo udevadm control --reload-rules && sudo udevadm trigger
```

This way the USB device always gets the fixed name `/dev/gps` and permissions are set automatically. You can also distinguish multiple identical devices (e.g., two IMUs) by serial number. AI barely knows udev.

### 19.4.2 USB Bandwidth

Plug three USB3 cameras into the same USB hub and you get frame drops from bandwidth shortage. AI says "update the driver" or "lower the resolution," but the real cause is the bandwidth limit of the USB controller.

```bash
# Check which camera is attached to which USB controller
lsusb -t
```

This problem cannot be solved in software. You must physically spread the cameras across different USB controllers. On desktop PCs, the front-panel USB and rear-panel USB are often on different controllers, so check the Bus numbers in `lsusb -t` and distribute them.

### 19.4.3 LiDAR Connection (IP Configuration)

90% of "no data" problems with Velodyne or Ouster LiDARs are caused by network settings. AI recommends reinstalling the driver or rebuilding the ROS package, but there are things to check first.

LiDARs use a fixed IP (e.g., `192.168.1.201`). The host PC's Ethernet interface must be on the same subnet (e.g., `192.168.1.100`) for communication to work.

```bash
# Step 1: check if the LiDAR pings
ping 192.168.1.201

# Step 2: set the host Ethernet interface IP
sudo ip addr add 192.168.1.100/24 dev eth0
sudo ip link set eth0 up

# Step 3: check that UDP packets arrive via Wireshark
sudo tcpdump -i eth0 udp port 2368 -c 10
```

Most cases get stuck right at `ping`. Confirming UDP packet arrival with Wireshark (or tcpdump) is the most reliable debugging method. If packets arrive but ROS doesn't see them, only then is it time to suspect the driver.

### 19.4.4 Camera Drivers (v4l2)

AI only knows `cv2.VideoCapture(0)` and can't tell which `/dev/video*` is the actual camera when there are several. It's common for a single USB camera to expose `/dev/video0` and `/dev/video1` (a metadata device), and AI doesn't know this.

```bash
# Check camera device mapping
v4l2-ctl --list-devices

# Check supported formats and resolutions
v4l2-ctl -d /dev/video0 --list-formats-ext
```

Auto exposure, auto white balance — these automatic settings often wreck SLAM performance. Constantly changing brightness destabilizes feature extraction.

```bash
# Manual exposure settings (for SLAM)
v4l2-ctl -d /dev/video0 --set-ctrl=exposure_auto=1
v4l2-ctl -d /dev/video0 --set-ctrl=exposure_absolute=100

# Lock white balance
v4l2-ctl -d /dev/video0 --set-ctrl=white_balance_automatic=0
```

AI barely knows this kind of low-level camera control. When told "SLAM is unstable," it recommends tuning algorithm parameters, but disabling the camera's automatic settings alone can give dramatic improvements.

### 19.4.5 Jetson (ARM) Environment

AI-generated code and Docker configs are almost 100% x86-based. On NVIDIA Jetson (ARM64) they often don't run.

Points to watch:
- When installing via `pip install`, many packages have no pre-built binaries (wheels) for ARM. `scipy` and `opencv-python` in particular must be built from source, which can take tens of minutes.
- The JetPack version pins CUDA, cuDNN, and TensorRT versions. If AI recommends installing the latest version, the whole system falls apart.
- When using Docker, you must use NVIDIA's `l4t` (Linux for Tegra) based images.

```bash
# Docker image on Jetson — don't use x86 images
docker pull nvcr.io/nvidia/l4t-pytorch:r35.2.1-pth2.0-py3

# Check the JetPack version
cat /etc/nv_tegra_release
```

When asking AI for code, specify "Jetson Orin, JetPack 5.1.2, CUDA 11.4 environment."

### 19.4.6 Real-Time Control and Timing

AI tells you to build a 100Hz loop with `time.sleep(0.01)`, but this isn't accurate. The precision of `time.sleep()` depends on the OS scheduler, and on a generic Linux kernel there is jitter of several milliseconds.

```python
# What AI commonly recommends (not accurate)
import time
while True:
    do_control()
    time.sleep(0.01)  # can actually be 10-15 ms
```

Python's GIL (Global Interpreter Lock) further weakens multithreaded timing guarantees. For actual real-time control, use C++ and an RT (Real-Time) kernel (PREEMPT_RT).

```bash
# Check the actual publish rate — always verify with this
ros2 topic hz /cmd_vel
```

Even when AI says "100Hz control is fine," verify the actual rate with `ros2 topic hz`. If the expected rate differs from the actual rate, control won't work properly.

## 19.5 Patterns Where AI Agents Give Up

### 19.5.1 "It works in simulation"

Works fine in Gazebo but fails on the real robot. AI is prone to conclude "it works in simulation, so the code is fine and it must be a hardware problem." But the real cause is usually the sim-to-real gap:

- **Sensor noise**: simulated Gaussian noise and real sensor noise have different distributions
- **Communication latency**: topic delivery is instantaneous inside Gazebo, but in reality there is latency of a few to tens of ms
- **Timing mismatch**: simulation guarantees perfect synchronization, but in reality timestamps drift between sensors
- **Frame mismatch**: if the URDF and the real robot's sensor positions/angles differ slightly, tf goes off

Tell AI the sim-to-real differences concretely: "it works in simulation but not on the real robot. Sensor noise level is X, communication latency is Y ms, and the frame calibration was done with method Z."

### 19.5.2 Trying to Fix Hardware Problems in Software

Bad cables, loose connections, insufficient power — AI cannot diagnose these physical issues.

Say "sensor data drops out intermittently" and AI will recommend adjusting buffer sizes, setting timeouts, changing QoS. But it's often a loose USB cable or an underpowered USB hub.

```bash
# Find hardware clues in the kernel log
dmesg | tail -20

# Check USB disconnect / reconnect events
dmesg | grep -i usb | tail -20
```

If `dmesg` shows messages like `USB disconnect` or `device descriptor read/64, error -71`, it's not a software problem. Replacing the cable, using a powered USB hub, or plugging into a different port comes first.

### 19.5.3 Giving Up on Environment Diagnosis

When library version conflicts get tangled, AI says "reinstall everything." Checking versions with `pip show package_name` and narrowing down what conflicts comes first.

OpenCV-related conflicts in particular are a robotics classic:

- `opencv-python` (default)
- `opencv-python-headless` (server-side, no GUI)
- `opencv-contrib-python` (with extra modules)
- `cv_bridge` (ROS package, references its own OpenCV)

When these four are installed simultaneously, they conflict with each other.

```bash
# Check the currently installed OpenCV
pip show opencv-python opencv-python-headless opencv-contrib-python

# Fix: don't install pip opencv in a ROS environment
pip uninstall opencv-python opencv-python-headless opencv-contrib-python
sudo apt install ros-humble-cv-bridge
```

In a ROS environment, using only `apt install ros-humble-cv-bridge` and not installing opencv separately via pip is the cleanest approach.

### 19.5.4 "Gives up after 2-3 tries"

AI repeats the same approach two or three times, and when it fails, moves on with "try a different approach." Engineers don't give up here. They dig through logs, run strace, and capture packets.

To get better answers from AI, provide not just the error message but low-level information:

```bash
# System logs
dmesg | tail -30
journalctl -u my_service --since "5 minutes ago"

# Process tracing
strace -f -e trace=open,read,write ros2 run my_pkg my_node 2>&1 | head -100

# Network packet capture
sudo tcpdump -i eth0 -w capture.pcap
```

Provide this information to AI and you'll get answers closer to the real cause instead of "reinstall it."

## 19.6 How to Use AI Agents Properly

*The general frame for putting AI to work (writing/reading territory) is covered in depth in [`../../survival-research/part1_reading/ch02_keshav_three_passes.md`](../../survival-research/part1_reading/ch02_keshav_three_passes.md) and [`part2_writing/A_workflow/ch01_mindset.md`](../../survival-research/part2_writing/A_workflow/ch01_mindset.md) *(Korean; English version planned)*. This §19.6 is the field-specific application to code and hardware.*

### 19.6.1 Provide Enough Context

The most important thing when asking AI is the quantity and quality of context.

Wrong example: "the camera isn't working"

Right example: "Ubuntu 22.04, ROS2 Humble, Intel RealSense D435. It shows up in `rs-enumerate-devices`, but `ros2 launch realsense2_camera rs_launch.py` gives a `Could not open device` error. Running inside Docker, and `--device=/dev/video0` is mapped."

**Checklist of information to provide to AI**:
- OS version, ROS version
- Hardware platform (x86 vs ARM/Jetson)
- Sensor model name
- Full error message (don't copy just a piece — give the whole thing)
- Output of `ros2 topic list`, `ros2 node list`
- Whether Docker is used and the run options (the full `docker run` command)
- Network configuration (wired/wireless, IP range)

### 19.6.2 How to Verify AI's Answers

Before running AI's answers as-is, check the following:

- **"Install this package"** → first check that the package supports your ROS version and Ubuntu version. Check existence with `apt search ros-humble-<package_name>`.
- **"Change this setting"** → back up the current setting before changing, and ask AI for the rationale. If it can't explain why, be suspicious.
- **"Reinstall"** → 90% of the time you don't need to reinstall. First narrow down the exact cause of the error. Checking current state with `pip show`, `dpkg -l | grep`, `apt policy`, etc. comes first.
- **When AI gives you code** → check for hardcoded paths (`/home/user/...`), hardcoded IPs (`192.168.1.100`), x86-only packages (`amd64` wheels), and the like.

### 19.6.3 What AI Is Good At vs Bad At

| What AI is good at | What AI is bad at |
|---|---|
| Algorithm implementation (SLAM, detection, etc.) | Hardware debugging |
| Writing ROS2 node/service code | QoS/DDS tuning |
| Python/C++ refactoring | USB/serial permission issues |
| Reading/summarizing papers | Network configuration (LiDAR IPs, etc.) |
| Writing CMakeLists.txt | Real-time timing issues |
| Data preprocessing pipelines | Hardware access inside Docker |
| Visualization code (matplotlib, Open3D) | Sensor time synchronization in practice |
| Interpreting error messages (generic) | Debugging based on dmesg/kernel logs |

The structure is this: AI is strong in "pure software" territory but weak where hardware and software meet. And most robotics problems occur right at that boundary. The right move is to delegate AI-strong areas and, in AI-weak areas, do the debugging yourself and feed the results to AI for analysis.

## 19.7 Workflows for Using AI as a Research Tool

Section 19.6 covered AI's strengths and weaknesses. Building on that, this section looks at practical workflows for how a robotics researcher uses AI throughout a daily routine.

### 19.7.1 Reading Papers

*The paper-reading workflow (3-pass + AI layer cameo) is covered in depth in the AI-layer cameo of [`../../survival-research/part1_reading/ch02_keshav_three_passes.md`](../../survival-research/part1_reading/ch02_keshav_three_passes.md) *(Korean; English version planned)*.*

Field-specific application: 3-pass + AI summary on the field's core papers — read the abstract, ask for a 3-line contribution summary, ask for step-by-step derivation of equations.

### 19.7.2 Writing Code

- Prototyping: "write code that extracts ORB features from the KITTI dataset and matches them. Use OpenCV, with Lowe's ratio test at 0.75" — give concrete instructions like this
- Debugging: error message + code + "what causes this error" — AI's strong area
- Refactoring: "turn this code into a PyTorch Dataset class" — strong at structural transformations
- What AI can't do (see earlier in this chapter): ROS QoS, hardware permissions, network settings, real-time timing

### 19.7.3 Experiment Design

*The frame for putting AI to work in experiment design, ablations, and result interpretation is captured as a cameo in [`../../survival-research/part2_writing/E_after/ch17_revision_rebuttal.md`](../../survival-research/part2_writing/E_after/ch17_revision_rebuttal.md) (or [`ch12_figures.md`](../../survival-research/part2_writing/C_sections/ch12_figures.md)) *(Korean; English version planned)*.*

Field-specific application: throw the baseline-comparison table at AI and ask *which comparison axis I missed* — that workflow.

### 19.7.4 Writing Papers

- Drafting: give AI the core idea and experimental results and ask "draft an Introduction" — useful for structuring
- Grammar/expression correction: fixing awkward expressions in English papers. AI understands context better than Grammarly
- Caveat: don't submit AI's sentences as-is. You must rewrite them in your own voice. Reviewers can spot AI-generated prose
- BibTeX generation: "make a BibTeX entry for this paper" — sometimes faster than copying from Google Scholar. But AI sometimes gets the year or venue wrong, so always verify

### 19.7.5 Example Daily Workflow

A concrete scenario for how to use AI in a daily routine:

```
09:00 — Check 3 new papers on arXiv. Ask AI for a 1-sentence summary of each
09:30 — Pick one interesting paper, second-pass read. Ask AI to derive unfamiliar equations
10:30 — Analyze yesterday's training results. Screenshot the loss curve and ask AI "is this pattern normal?"
11:00 — Write new experiment code. Have AI generate the DataLoader structure. Manually fix the augmentation logic
14:00 — Debug SLAM code. ROS2 error → solve directly using this chapter (AI doesn't know QoS)
16:00 — Draft the Related Work section. Have AI build a comparison table for 5 papers
17:00 — Verify the table. Find that AI confused the methods of 2 papers, fix manually
```
