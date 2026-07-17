# Ch.22 — Closing: Where to Start?

The previous twenty-one chapters covered the main elements of Spatial AI, from sensors and coordinate frames to robot motion, spatial perception, and learning-based methods. This final chapter identifies where to begin for different backgrounds and projects.

## 22.1 The Map So Far

The guide consists of four parts.

- **Foundations** (Ch.1–3): the scope of Spatial AI, how sensors measure the environment, and the mathematics used to process those measurements. Rotation, transformation, optimization, and probability recur throughout the guide.
- **Robots** (Ch.4–8): physical systems built from joints and links. Kinematics determines pose, dynamics computes forces, and control and motion planning produce desired states and paths. Robot learning acquires parts of this process from data.
- **Perception and Spatial Understanding** (Ch.9–14): the path from image processing to 3D spatial perception. Classical CV, deep learning, foundation models, and VLA lead into SLAM, which connects pose and maps over time.
- **Research in Practice** (Ch.15–21): the frameworks, development tools, datasets, and references needed to run experiments.

Real projects cross these boundaries. Operating a robot may require the equations in Ch.3, the SLAM methods in Ch.14, and the Docker setup in Ch.16 at the same time.

## 22.2 Starting Points by Profile

The best starting point depends on the reader's background.

For a **third- or fourth-year undergraduate new to robotics**, Ch.1 → Ch.3 → Ch.9 → Ch.14 is a useful sequence. It introduces the scope of Spatial AI and its mathematical language before connecting images, 3D geometry, and SLAM. Setting up the environment in Ch.16 alongside these chapters makes it possible to run the examples immediately.

A **new master's student with a deep-learning background** can begin with Ch.2 → Ch.3 → Ch.10 → Ch.11. This sequence starts from familiar learning-based methods, fills in the sensor and mathematical foundations, and then shows how robotics uses foundation models. Ch.13 and Ch.14 extend the path to 3D vision and SLAM.

Readers with a **classical robotics background and less experience in deep learning** can start with Ch.8 → Ch.10 → Ch.11 → Ch.12. They can consult Ch.4–7 as needed and concentrate on the progression from robot learning to VFMs and VLAs.

Readers with a **specific project** can begin with the datasets and benchmarks in Ch.17. After selecting one or two papers on a similar task, they can work backward to the chapters those papers require.

Ch.20.7 provides learning plans for one-, three-, and six-month periods.

## 22.3 What Not to Do

New researchers may focus on increasing the paper count, postpone experiments until the environment is perfect, or spend too much time implementing every component from scratch. Accepting AI-generated output without verification creates the opposite problem. [Research Notes Ch.6](../../research-notes/guide.html#chapter-6) and [Grad Notes Ch.11](../../grad-notes/guide.html#chapter-11) discuss these issues in detail *(Korean only)*.

SLAM and CV already have widely used public implementations, including ORB-SLAM3, COLMAP, and Gaussian Splatting. Unless the goal is education or a clearly new contribution, running an existing implementation and analyzing its limits may lead to the research problem more directly.

## 22.4 A Sense of the Long Game

The first year, the eighteen-month mark, and the five-year horizon of a PhD are discussed in [Grad Notes Ch.7](../../grad-notes/guide.html#chapter-7) and [Grad Notes Ch.1](../../grad-notes/guide.html#chapter-1) *(Korean only)*.

Robotics experiments can have long cycles because they require hardware preparation, safety procedures, data collection, and repeated runs. Since preparation time and cycle length vary widely by equipment, environment, and laboratory, planning around complete experiment cycles is more realistic than judging progress day by day.

## 22.5 Next Steps

There is no need to reread the guide from beginning to end. When a project stalls, return to the relevant chapter and check the equations, implementation details, or datasets.

Continue with [Research Notes](../../research-notes/guide.html) for reading and writing papers, and [Grad Notes](../../grad-notes/guide.html) for managing the PhD process *(Korean only)*.

Draft date: 2025.12.28 · Revision date: 2026.05.01
