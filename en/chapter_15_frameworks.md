# Ch.15 — Robot Frameworks

Perception, SLAM (Ch.14), planning, and motor control rarely run as a single monolithic binary on a physical robot. Instead, sensor drivers, estimators, planners, and controllers execute concurrently across separate threads and processes, exchanging data while tracking shared coordinate frames in real time. A robot framework supplies that common plumbing—inter-process communication, message serialization, package management, and transform tracking. This chapter starts with ROS communication and tooling, then surveys the simulators and specialized libraries built around it.

## 15.1 ROS (Robot Operating System)

Despite its name, **ROS** is middleware rather than an operating system. Sitting atop Linux, it provides standardized inter-process communication, package management, and debugging tools. Because modules such as cameras, LiDAR, actuators, and high-level planners run concurrently, isolating each capability inside an independent node with shared message formats lets teams develop, swap, and test components without rewriting the rest of the stack.

### 15.1.1 ROS1 vs ROS2

ROS2 redesigned the original ROS1 architecture around the Data Distribution Service (DDS) standard, eliminating the single central master (`roscore`) while adding real-time support, security, and native multi-robot scalability:

| Feature | ROS1 | ROS2 |
| --- | --- | --- |
| First public release | 2010 (ROS 1.0 Box Turtle; development began 2007) | 2017 (Ardent Apalone) |
| Communication | Custom (TCPROS) | DDS-based |
| Real-time | Not supported | Supported |
| Security | None | SROS2 |
| Multi-robot | Difficult | Easy |
| Master | Required (roscore) | Not required |
| Python | 2/3 | 3 only |

**Current recommendation**: For a new long-lived project, start with **ROS2 Jazzy LTS** unless hardware drivers or existing packages anchor the stack to Ubuntu 22.04 and **ROS2 Humble**. Before committing to a distribution, check the compatibility matrix across your Ubuntu release, required sensor drivers, and each distribution's end-of-life (EOL) date. Official support for ROS1 Noetic ended in May 2025, making ROS2 the default baseline for new work. If a legacy ROS1 package is genuinely unavoidable, evaluate `ros1_bridge` only after confirming that the exact ROS1, ROS2, and Ubuntu combination is supported; major stacks such as Nav2 and MoveIt 2 are already native to ROS2.

> **Further reading**
> - [ROS2 Official Tutorials](https://docs.ros.org/en/jazzy/Tutorials.html) — Official step-by-step guide for ROS2 Jazzy LTS; if you are new to ROS2, start with "Beginner: CLI tools".
> - [The Construct - ROS2 Basics](https://www.youtube.com/@TheConstruct) — ROS-focused education channel featuring hands-on simulator walkthroughs.
> - [ROS1 to ROS2 Migration Guide](https://docs.ros.org/en/jazzy/How-To-Guides/Migrating-from-ROS1.html) — Official guide for porting existing ROS1 codebases.

### 15.1.2 Core Concepts

A ROS2 computation graph connects independent modules through three communication primitives—topics, services, and actions—alongside runtime parameters. Choosing among them comes down to timing and response semantics: continuous streams, short request-response queries, and long-running tasks that expose intermediate progress.

- **Node**: A single-purpose functional unit participating in the graph, such as a sensor driver or motor controller. Whereas a ROS1 node effectively corresponded to one OS process, ROS2 allows multiple nodes to be compiled as components inside a single process to reduce serialization and transport overhead.
- **Topic**: An asynchronous, many-to-many message stream built on the publisher/subscriber pattern, suited to continuous data such as camera frames, LiDAR scans, and velocity commands. The minimal Python node below publishes a string message on a periodic timer:

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

- **Quality of Service (QoS)**: Because ROS2 runs over DDS, every publisher and subscriber declares a QoS profile governing delivery guarantees. **Reliability** (`RELIABLE` vs. `BEST_EFFORT`) determines whether dropped packets are retransmitted, while **durability** (`TRANSIENT_LOCAL` vs. `VOLATILE`) determines whether late-joining subscribers receive the most recent latched sample. Crucially, publisher and subscriber profiles must be *compatible*, not necessarily identical—a rule detailed in the check below.
- **Service**: A request/response interface suited to brief, one-shot operations such as updating a mode or querying current state. Because ROS2 client APIs are asynchronous by default, blocking synchronously on a service call inside a callback on the executor's own thread will deadlock, as the thread waiting for the response is the same thread needed to dispatch it.
- **Action**: An asynchronous, goal-directed interface designed for long-running tasks such as navigation or manipulation. Unlike a service, an action streams periodic progress feedback and supports mid-execution cancellation.
- **Parameter**: Runtime configuration values attached to a node—such as controller gains or sensor frame IDs—that can be read and updated without recompiling.

In practice, map continuous sensor streams to topics, quick one-shot queries ("what is the battery level?") to services, and multi-second tasks ("navigate to the goal") to actions. Keeping those roles distinct yields a cleaner, easier-to-debug communication architecture.

> **⚠ Generated-code check**: Always specify the sensor topic's QoS profile when prompting for or reviewing ROS2 code. Verify that the subscriber's reliability and durability settings are compatible with the publisher's, because an incompatible pair silently drops delivery without raising an obvious error. Compatibility does not require equality: a `RELIABLE` publisher connects to a `BEST_EFFORT` subscriber, whereas the reverse fails; similarly, a `TRANSIENT_LOCAL` publisher connects to a `VOLATILE` subscriber, whereas the reverse fails.

> **Further reading**
> - [ROS2 Concepts — Understanding nodes, topics, services, actions](https://docs.ros.org/en/humble/Concepts.html) — Official ROS2 concepts reference.
> - [The Construct - ROS2 Topics vs Services vs Actions](https://www.youtube.com/@TheConstruct) — Video comparing the three communication patterns.

### 15.1.3 Tools

Pushing untested code straight onto a physical robot is slow and hazardous. Cutting debugging time requires verifying firsthand that sensor streams arrive at expected rates and that coordinate frames align properly. Four built-in tools form the daily debugging workflow in ROS2:

- **`rviz` / `rviz2`**: The standard 3D visualizer for inspecting sensor streams, TF trees, planned paths, and occupancy maps in real time.
- **`rqt`**: A Qt-based suite of diagnostic plugins, including `rqt_graph` for inspecting live node-and-topic connectivity and `rqt_plot` for plotting scalar telemetry over time.
- **`rosbag` / `ros2 bag`**: Records and replays serialized topic streams. Running the real robot for every algorithm tweak wastes hardware time and money; recording a bag once lets you replay identical sensor inputs offline as often as needed for debugging and reproducible benchmarking:

```bash
# Record a ROS2 bag
ros2 bag record -a -o my_bag

# Replay
ros2 bag play my_bag
```

- **`tf2` (Transform Library)**: Maintains time-varying 3D coordinate transforms across a robot's frame tree—camera, LiDAR, robot base, and world—so you do not have to manually chain SE(3) transformation matrices whenever you project a point from a sensor frame into the base frame. Because the listener populates its buffer asynchronously over `/tf` and `/tf_static`, lookups normally run inside a timer callback after the buffer has received initial transforms:

```python
# tf2 listener example
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException

class TfDemo(Node):
    def __init__(self):
        super().__init__('tf_demo')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Look up inside a timer so the listener has time to fill the buffer
        self.create_timer(0.1, self.lookup)

    def lookup(self):
        try:
            # Look up the transform from camera_link coordinates to base_link coordinates
            t = self.tf_buffer.lookup_transform('base_link', 'camera_link', rclpy.time.Time())
        except (LookupException, ExtrapolationException) as e:
            self.get_logger().warn(f'tf not ready: {e}')
```

> **Further reading**
> - [ROS2 tf2 Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html) — Official coordinate-transform tutorial.
> - [The Construct channel](https://www.youtube.com/@TheConstruct) — ROS2 video tutorials, including `rviz2` walkthroughs.
> - [ros2 bag CLI documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html) — Official guide for recording and replaying bags.

### 15.1.4 Key Packages

Beyond core middleware, a handful of standard packages supply common message definitions, bridge ROS streams to vision and point-cloud libraries, and provide autonomous navigation:

| Package | Purpose |
| --- | --- |
| sensor_msgs | Sensor message types |
| geometry_msgs | Geometry messages (Pose, Twist, etc.) |
| cv_bridge | OpenCV ↔︎ ROS image conversion |
| image_transport | Compressed image transport |
| pcl_ros | PCL ↔︎ ROS point cloud |
| nav2 | Navigation stack (ROS2) |

> **Further reading**
> - [Nav2 Documentation](https://docs.nav2.org/) — Official documentation for the ROS2 navigation stack.
> - [ROS2 Package Index](https://index.ros.org/packages/) — Searchable index of ROS2 packages.

## 15.2 Simulation

Testing unverified code on real hardware risks damaging expensive actuators or injuring people, and physical trials run far too slowly for data-hungry training loops such as reinforcement learning. Simulators solve both problems by letting teams validate joint limits, collision behavior, and edge-case failures safely before touching hardware while running thousands of parallel episodes. With the rise of **embodied AI**, platforms such as NVIDIA Isaac Sim, AI2-THOR, and Habitat have also become central data-generation engines, making **sim-to-real transfer**—carrying policies trained in virtual environments onto physical robots—a major research focus.

### 15.2.1 Gazebo

Gazebo integrates tightly with ROS, and many open-source ROS packages ship Gazebo simulation worlds and robot descriptions ready to launch. In this ecosystem, **SDF (Simulation Description Format)** defines full simulation worlds and physics properties, while **URDF (Unified Robot Description Format)** describes the robot's links, joints, visuals, and collision geometry. For ROS2 projects, the modern **Gazebo Sim** (formerly Ignition) is preferred over legacy Gazebo Classic thanks to its modular plugin architecture and cleaner extensibility. The minimal URDF snippet below defines a box-shaped `base_link` with matching visual and collision shapes:

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
> - [Gazebo Sim Official Tutorials](https://gazebosim.org/docs) — Official documentation for Gazebo Sim (formerly Ignition).
> - [URDF Tutorial (ROS2)](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html) — Fundamentals of URDF robot modeling.
> - [The Construct - Gazebo Sim with ROS2](https://www.youtube.com/@TheConstruct) — Hands-on walkthrough combining Gazebo and ROS2.

### 15.2.2 NVIDIA Isaac Sim

NVIDIA Isaac Sim is widely used in embodied AI for large-scale synthetic-data generation and sim-to-real training, particularly in manipulation research. Combining RTX ray-traced rendering with the PhysX 5 physics engine, it integrates directly with ROS2 and supports domain randomization across lighting, textures, and object arrangements. Alongside Isaac Sim, several specialized simulators target distinct embodied AI workloads, trading off photorealism, contact fidelity, and raw frame rate:

| Simulator | Primary use | Features |
| --- | --- | --- |
| NVIDIA Isaac Sim | General purpose (Manipulation, Navigation) | RTX rendering, PhysX 5, large-scale synthetic data |
| AI2-THOR | Indoor navigation, object interaction | 120+ indoor scenes, realistic interaction |
| Habitat (Meta) | Visual navigation, embodied QA | Ultra-fast rendering (thousands of FPS), large-scale training |
| iGibson | Indoor robot tasks | Physically-based rendering, home environments |
| MuJoCo | Robot control, reinforcement learning | Accurate contact dynamics, fast simulation |

> **Further reading**
> - [NVIDIA Isaac Sim Official Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/index.html) — From installation to advanced simulation workflows.
> - [AI2-THOR Documentation](https://ai2thor.allenai.org/ithor/documentation) — Indoor interactive simulator for embodied AI.
> - [Habitat Documentation](https://aihabitat.org/docs/habitat2/) — Meta's high-speed embodied AI platform.

### 15.2.3 CARLA

For autonomous driving research, **CARLA** serves as a standard virtual testbed across perception, planning, and end-to-end driving papers. It simulates multi-lane urban road networks under configurable weather and time-of-day conditions, provides virtual camera, LiDAR, and radar sensors, and includes a ROS bridge for connecting autonomy stacks directly to simulated vehicles.

> **Further reading**
> - [CARLA Documentation](https://carla.readthedocs.io/) — Official documentation and Python API reference.
> - [CARLA Simulator YouTube](https://www.youtube.com/channel/UC1llP9ekCwt8nEJzMJBQekg) — Official demo videos showcasing simulator capabilities.

## 15.3 Other Frameworks

Beyond general middleware and simulators, the robotics ecosystem provides domain-specific stacks and standalone numerical libraries that spare teams from building core infrastructure from scratch:

- **Isaac ROS**: A collection of NVIDIA GPU-accelerated ROS2 packages for DNN inference, visual SLAM, and 3D perception, optimized specifically for Jetson edge hardware.
- **Autoware (`Autoware.Universe`)**: A complete open-source ROS2 autonomous driving stack covering perception, localization, planning, and vehicle control.
- **Standalone C++ and vision libraries**: Outside ROS, **OpenCV** handles 2D computer vision, **Open3D** supports 3D point-cloud and mesh processing, **Eigen** provides high-performance C++ linear algebra, and **Sophus** extends Eigen with clean Lie-group abstractions for 3D rotations (`SO(3)`) and rigid-body transforms (`SE(3)`).

> **Further reading**
> - [OpenCV Tutorials](https://docs.opencv.org/4.x/d9/df8/tutorial_root.html) — Computer vision tutorials from basics to advanced topics.
> - [Open3D Documentation](http://www.open3d.org/docs/) — Official documentation for 3D data processing.
> - [Eigen Getting Started](https://eigen.tuxfamily.org/dox/GettingStarted.html) — Introduction to the C++ linear algebra library.

## 15.4 Advanced: System Design

Selecting packages and simulators is only part of building a working robot. Production and field systems depend equally on how end-to-end latency is budgeted, how complex behaviors are structured, how safety boundaries are enforced, and how code progresses from simulation to real hardware.

**15.4.1 Latency Budgeting**
End-to-end reaction time is managed by partitioning the total deadline across each stage of the pipeline—for example, an autonomous driving budget of sensor input (10 ms) → perception (50 ms) → planning (30 ms) → control (10 ms) for a 100 ms total. These per-stage figures act as design targets rather than automatic guarantees: unless each module enforces its own deadline, slack in a fast stage can quietly mask an overrun upstream, while overall system throughput remains bottlenecked by the slowest stage. In ROS2, engineers measure and verify these budgets using callback duration profiling, `ros2 topic delay`, and system-level tracing via `ros2_tracing`.

**15.4.2 Behavior Tree**
As tasks gain edge cases and recovery branches, finite state machines (FSMs) become hard to maintain because transitions between states grow quadratically. **Behavior Trees (BTs)** organize decision logic into modular trees built from **Sequence**, **Fallback**, **Action**, and **Condition** nodes, allowing individual subtrees to be tested and reused in isolation. In ROS2, **BehaviorTree.CPP** is the dominant implementation and drives the task-execution layer in Nav2.

**15.4.3 Safety and Failsafe**
When deploying a new algorithm on physical hardware, always build the safety wrapper first and run experimental code inside it. Reliable architectures combine a physical **E-stop** (hardware-level power cutoff), a **watchdog timer** that commands a safe stop if heartbeats cease within a timeout, and software guardrails enforcing velocity caps, workspace limits, and collision checks. For service robots operating near people, standards such as **ISO 13482** define baseline safety principles.

**15.4.4 Deployment and Field Testing**
Teams bridge simulation and real-world operation through layered validation: automated CI/CD (`colcon` builds, unit tests, and Docker image builds), deterministic **log replay testing** over recorded sensor bags, and **Hardware-in-the-Loop (HIL)** testing where real flight or vehicle compute runs against a real-time simulated plant. Physical field trials then advance in stages—controlled indoor spaces, semi-controlled testbeds, and finally open environments—while collecting synchronized `rosbag` recordings, system logs (`journalctl`), and sensor health telemetry.

> **Further reading**
> - [BehaviorTree.CPP Documentation](https://www.behaviortree.dev/) — Behavior tree design patterns and tutorials.
> - [Nav2 Documentation](https://docs.nav2.org/) — Real-world example of a BehaviorTree.CPP-based ROS2 architecture.

## Technical Timeline: Robot Frameworks — Past → Present → Future

```
2007 ─── ROS development begins (Stanford → Willow Garage; 1.0 released in 2010)
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
2025 ─── ROS1 Noetic EOL (end of support)
  │       The effective deadline for the ROS2 transition
  │
2025+ ── Era of embodied AI + foundation models
          Large-scale pretraining in simulators → sim-to-real transfer
          Language-instructed robot manipulation (VLA)
          NVIDIA Isaac Lab and others begin offering a unified simulator → training → real-robot deployment pipeline
```

Frameworks and simulators establish how robot modules communicate and where they are tested, yet getting those stacks to compile and behave identically across developer laptops, GPU servers, and onboard computers demands strict toolchain discipline. Ch.16 covers the practical development environment—languages, Ubuntu and CUDA setup, Docker containers, remote Git/SSH workflows, experiment tracking, and automated testing—that underpins reproducible robotics engineering.
