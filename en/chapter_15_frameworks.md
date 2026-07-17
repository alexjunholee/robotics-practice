# Ch.15 — Robot Frameworks


Robot software must run sensor drivers, path planning, and motor control concurrently while managing communication among them. A framework supplies shared functions such as thread management, message serialization, and coordinate transforms. This chapter begins with ROS communication and package structure, then moves to simulators and related tools.

## 15.1 ROS (Robot Operating System)

ROS is an open-source framework for robot software development. It is not an operating system but **middleware**, providing inter-process communication, package management, and tooling.

A robot runs modules such as cameras, LiDAR, motors, and controllers concurrently. ROS standardizes their communication and message formats so that each function can be developed as a separate node.

### 15.1.1 ROS1 vs ROS2

| Feature | ROS1 | ROS2 |
| --- | --- | --- |
| Release | 2007 | 2017 |
| Communication | Custom (TCPROS) | DDS-based |
| Real-time | Not supported | Supported |
| Security | None | SROS2 |
| Multi-robot | Difficult | Easy |
| Master | Required (roscore) | Not required |
| Python | 2/3 | 3 only |

**Current recommendation**: For a new long-lived project, evaluate ROS2 Jazzy LTS first. Projects tied to Ubuntu 22.04 or existing packages may still remain on Humble.

Choose only after checking the supported ROS distribution, Ubuntu version, and EOL date of the required drivers and packages. Official support for ROS1 Noetic ended in May 2025.

For a new project, start with ROS2 unless there is a specific reason not to. If an existing ROS1 package is required, evaluate `ros1_bridge` only after checking the supported ROS1, ROS2, and Ubuntu combination. Nav2 and MoveIt 2 are available for ROS2.

> **Further reading**
> - [ROS2 Official Tutorials](https://docs.ros.org/en/jazzy/Tutorials.html) — Official step-by-step guide for ROS2 Jazzy LTS. If it is your first time, start from "Beginner: CLI tools"
> - [The Construct - ROS2 Basics](https://www.youtube.com/@TheConstruct) — ROS-focused education channel. Hands-on inside a simulator
> - [ROS1 to ROS2 Migration Guide](https://docs.ros.org/en/jazzy/How-To-Guides/Migrating-from-ROS1.html) — Official guide for porting existing ROS1 code

### 15.1.2 Core Concepts

Topics, services, and actions differ in timing and response behavior. They represent sensor streams, short request-response operations, and long-running tasks that expose intermediate state, respectively.

**Node**:
- An executable process
- Single-purpose (sensor driver, controller, etc.)

**Topic**:
- Asynchronous message stream
- Publisher/subscriber pattern
- Examples: sensor data, commands

```python
# ROS2 Publisher example
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

**Service**:
- Synchronous request/response
- Suited to one-shot operations
- Examples: changing settings, querying state

**Action**:
- Asynchronous goal-directed task
- Provides feedback
- Cancelable
- Examples: navigation, manipulation

Use a Topic for continuous data such as camera images. A Service fits a single request and response, such as "tell me the current battery level," whereas an Action fits a task that takes time, such as "go over there." Distinguishing the three leads to a clearer communication design.

**Parameter**:
- Node configuration values
- Changeable at runtime

> **⚠ Generated-code check**: Include the sensor topic's QoS when requesting ROS2 code. Verify that the generated subscriber matches the publisher's reliability and durability settings; a mismatch can prevent delivery without an obvious error message.

> **Further reading**
> - [ROS2 Concepts — Understanding nodes, topics, services, actions](https://docs.ros.org/en/humble/Concepts.html) — Official concepts document
> - [The Construct - ROS2 Topics vs Services vs Actions](https://www.youtube.com/@TheConstruct) — Video comparing the three communication patterns

### 15.1.3 Tools

When developing a robot, the "just write code and throw it on the robot" approach is dangerous. You need to be able to see with your own eyes whether sensor data is arriving properly and whether coordinate frames line up; that is what cuts debugging time. The tools below are daily essentials for any ROS developer.

**rviz / rviz2**:
- 3D visualization tool
- Displays sensor data, TF, paths, etc.

**rqt**:
- Qt-based collection of GUI tools
- rqt_graph: visualizes node/topic relationships
- rqt_plot: plots data

**rosbag / ros2 bag**:
- Records and replays data
- Essential for debugging and algorithm development

Knowing these cuts experiment time substantially. Testing algorithms by running the real robot every time is costly in both time and money. Record once with rosbag and you can repeat experiments on the same data as many times as you want. In terms of reproducibility, it is an essential tool.

```bash
# Record a ROS2 bag
ros2 bag record -a -o my_bag

# Replay
ros2 bag play my_bag
```

**tf2 (Transform Library)**:
- Manages coordinate frame transforms
- Tracks transforms over time

A robot has camera, LiDAR, base, and world coordinate frames all existing at the same time. Computing "where is this point seen from the camera in the robot's frame?" requires coordinate transforms, and tf2 manages them automatically. If you have studied linear algebra, think of it as SE(3) transformation matrices.

```python
# tf2 listener example
from tf2_ros import Buffer, TransformListener

tf_buffer = Buffer()
tf_listener = TransformListener(tf_buffer, self)

# Look up base_link → camera_link transform
transform = tf_buffer.lookup_transform('base_link', 'camera_link', rclpy.time.Time())
```

> **Further reading**
> - [ROS2 tf2 Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html) — Official coordinate transform tutorial
> - [The Construct - rviz2 Complete Guide](https://www.youtube.com/@TheConstruct) — Video on using rviz2
> - [ros2 bag CLI documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html) — Official guide for recording and replaying data

### 15.1.4 Key Packages

The packages below provide standard messages, image and point-cloud conversion, and navigation functions.

| Package | Purpose |
| --- | --- |
| sensor_msgs | Sensor message types |
| geometry_msgs | Geometry messages (Pose, Twist, etc.) |
| cv_bridge | OpenCV ↔︎ ROS image conversion |
| image_transport | Compressed image transport |
| pcl_ros | PCL ↔︎ ROS point cloud |
| nav2 | Navigation stack (ROS2) |

> **Further reading**
> - [Nav2 Documentation](https://docs.nav2.org/) — Official documentation for the ROS2 Navigation stack
> - [ROS2 Package Index](https://index.ros.org/packages/) — ROS2 package search

## 15.2 Simulation

Experimenting directly on a real robot can damage the hardware or injure people. A simulator provides a place to check motion limits and failure conditions before hardware tests, and to run the many repeated episodes required by methods such as reinforcement learning.

Recently, as **embodied AI** research has expanded, the role of simulators in which robots learn autonomously within virtual environments has grown as well. Platforms like NVIDIA Isaac Sim, AI2-THOR, and Habitat are leading this trend, and sim-to-real transfer, moving policies learned in simulation onto real robots, is a central research topic.

### 15.2.1 Gazebo

Gazebo integrates with ROS, and many ROS packages provide Gazebo simulation demos and robot models.

**Components**:
- **SDF (Simulation Description Format)**: environment definition
- **URDF (Unified Robot Description Format)**: robot model

**Gazebo Classic vs Gazebo Sim (Ignition)**:
- Gazebo Sim: the newer version, recommended for ROS2
- Modular architecture with better extensibility

```xml
<!-- URDF example -->
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

> **Further reading**
> - [Gazebo Sim Official Tutorials](https://gazebosim.org/docs) — Official guide for Gazebo Sim (formerly Ignition)
> - [URDF Tutorial (ROS2)](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html) — Robot modeling basics
> - [The Construct - Gazebo Sim with ROS2](https://www.youtube.com/@TheConstruct) — Hands-on video of Gazebo + ROS2

### 15.2.2 NVIDIA Isaac Sim

Isaac Sim is widely used in embodied AI research for large-scale synthetic-data generation and sim-to-real training. It combines RTX rendering with the PhysX 5 physics engine, generates synthetic data through domain randomization, integrates with ROS 2, and is used mainly for manipulation research.

**Embodied AI simulator comparison**: Besides Isaac Sim, several simulators are widely used in embodied AI research.

| Simulator | Primary use | Features |
| --- | --- | --- |
| NVIDIA Isaac Sim | General purpose (Manipulation, Navigation) | RTX rendering, PhysX 5, large-scale synthetic data |
| AI2-THOR | Indoor navigation, object interaction | 120+ indoor scenes, realistic interaction |
| Habitat (Meta) | Visual navigation, embodied QA | Ultra-fast rendering (thousands of FPS), large-scale training |
| iGibson | Indoor robot tasks | Physically-based rendering, home environments |
| MuJoCo | Robot control, reinforcement learning | Accurate contact dynamics, fast simulation |

> **Further reading**
> - [NVIDIA Isaac Sim Official Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/index.html) — From installation to advanced usage
> - [AI2-THOR Documentation](https://ai2thor.allenai.org/ithor/documentation) — Indoor simulator for embodied AI research
> - [Habitat Documentation](https://aihabitat.org/docs/habitat2/) — Meta's embodied AI platform

### 15.2.3 CARLA

Autonomous driving papers very often use CARLA as their experimental environment. If you want to work on autonomous driving research, it is worth learning how to use CARLA.

**Features**:
- Urban environment simulation
- Various weather and time-of-day conditions
- Sensor simulation (camera, LiDAR, radar)
- ROS bridge provided

> **Further reading**
> - [CARLA Documentation](https://carla.readthedocs.io/) — Official documentation and Python API reference
> - [CARLA Simulator YouTube](https://www.youtube.com/@intaborlado5265) — Demo videos of simulator usage

## 15.3 Other Frameworks

Besides ROS and simulators, there are frameworks and libraries specialized for particular purposes. Being able to pull these off the shelf instead of building them yourself is one of the advantages of the robotics ecosystem.

**Isaac ROS**:
- NVIDIA GPU-accelerated ROS packages
- DNN inference, Visual SLAM, 3D perception
- Optimized for Jetson

**Autoware**:
- Complete autonomous driving stack
- Includes perception, planning, and control
- ROS2-based (Autoware.Universe)

**Tools outside ROS**:
- **OpenCV**: computer vision
- **Open3D**: 3D processing/visualization
- **Eigen**: linear algebra (C++)
- **Sophus**: SE(3), SO(3) operations

For example, Eigen and Sophus are core libraries in robotics for handling coordinate transforms. If you learned linear algebra in class, think of Eigen as a C++ implementation of those matrix operations. Sophus builds on top of it, adding convenient handling of rotations (SO(3)) and rigid-body transforms (SE(3)).

> **Further reading**
> - [OpenCV Tutorials](https://docs.opencv.org/4.x/d9/df8/tutorial_root.html) — Computer vision from basics to advanced
> - [Open3D Documentation](http://www.open3d.org/docs/) — Official documentation for the 3D data processing library
> - [Eigen Getting Started](https://eigen.tuxfamily.org/dox/GettingStarted.html) — Introduction to the C++ linear algebra library

## 15.4 Advanced: System Design

**15.4.1 Latency Budgeting**
- Allocate the latency of the full pipeline segment by segment
- Example: autonomous driving — sensor input (10 ms) → perception (50 ms) → planning (30 ms) → control (10 ms) = 100 ms total
- If any segment exceeds its budget the whole system fails. The slowest segment is the bottleneck
- Profiling methods: ROS2 callback duration, `ros2 topic delay`, tracing (ros2_tracing)

**15.4.2 Behavior Tree**
- A robot behavior design method with better extensibility than finite state machines (FSM)
- Node types: Sequence, Fallback, Action, Condition
- Advantage: modular — subtrees can be tested and reused independently
- In ROS2: BehaviorTree.CPP, used in Nav2
- As states grow, FSM transitions blow up exponentially. BT manages complexity via a tree structure

**15.4.3 Safety and Failsafe**
- Watchdog timer: safe stop if no heartbeat within a given time
- E-stop (Emergency Stop): hardware-level power cutoff
- Software safety: speed limits, workspace limits, collision checks
- ISO 13482: service robot safety standard (overview only)
- In practice: when deploying a new algorithm, build the safety wrapper first and experiment inside it

**15.4.4 Deployment and Field Testing**
- CI/CD: automated colcon build + test, Docker image builds
- Hardware-in-the-Loop (HIL): test new code while replaying real sensor data
- Field test protocol: controlled environment → semi-controlled → real environment, in stages
- Log collection: rosbag + system logs (journalctl) + sensor status monitoring

> **Further reading**
> - [BehaviorTree.CPP Documentation](https://www.behaviortree.dev/) — BT design patterns and tutorials
> - [Nav2 Documentation](https://docs.nav2.org/) — ROS2 Navigation2 stack. A real-world example of BT-based design

## Technical Timeline: Robot Frameworks — Past → Present → Future

```
2007 ─── ROS1 released (Willow Garage)
  │       Widely adopted as robot middleware
  │
2012 ─── Gazebo Classic becomes an independent project
  │       Simulation becomes an essential step in robot development
  │
2017 ─── First ROS2 release
  │       DDS-based communication, real-time support, security added
  │
2019 ─── NVIDIA Isaac Sim released
  │       RTX-based high-quality rendering + synthetic data generation
  │
2020 ─── Embodied AI simulators like Habitat and AI2-THOR rise
  │       Large-scale learning-based research on robot policies takes off
  │
2022 ─── ROS2 Humble LTS released
  │       Industry adoption accelerates, Nav2/MoveIt2 stabilize
  │
2024 ─── ROS1 Noetic EOL (end of support)
  │       The effective deadline for the ROS2 transition
  │
2025+ ── Era of embodied AI + foundation models
          Large-scale pretraining in simulators → sim-to-real transfer
          Language-instructed robot manipulation (VLA)
          NVIDIA Isaac Lab and others begin offering a unified simulator → training → real-robot deployment pipeline
```
