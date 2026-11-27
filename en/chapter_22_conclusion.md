# Ch.22 — Closing: Where to Start?

Over the previous twenty-one chapters, we moved from sensors and coordinate frames to robot motion, spatial perception, and learning-based methods. This final chapter helps you choose an entry point matched to your background and the project in front of you.

## 22.1 The Map So Far

The guide is organized into four parts:

- **Foundations** (Ch.1–3): Defines the scope of Spatial AI, explains how physical sensors measure the world, and introduces the mathematics used to process those measurements—rotations, rigid-body transformations, optimization, and probability recur throughout the rest of the book.
- **Robots** (Ch.4–8): Covers physical systems built from joints and links. Kinematics relates joint states to pose, dynamics models forces and torques, control and motion planning generate feasible trajectories, and robot learning acquires parts of this pipeline directly from data.
- **Perception and Spatial Understanding** (Ch.9–14): Traces the progression from 2D image processing to 3D spatial understanding. Classical computer vision, deep perception, foundation models, and VLAs all feed into SLAM, which maintains consistent poses and maps over time.
- **Research in Practice** (Ch.15–21): Gathers the middleware frameworks, developer tooling, benchmarks, and reference material needed to run real experiments.

Real projects rarely stay inside a single part. Getting a robot working often means combining the math in Ch.3, the SLAM pipeline in Ch.14, and the Docker workflow in Ch.16 all at once.

## 22.2 Starting Points by Profile

The best place to begin depends on where you are coming from:

For a **third- or fourth-year undergraduate new to robotics**, the sequence Ch.1 → Ch.3 → Ch.9 → Ch.14 works well. It sets out the scope of Spatial AI and its mathematical language before connecting camera geometry to 3D reconstruction and SLAM. Setting up your environment with Ch.16 alongside those chapters lets you run code immediately.

A **new master's student with a deep-learning background** can start with Ch.2 → Ch.3 → Ch.10 → Ch.11. Building on your existing ML fluency, this path fills in sensor characteristics and geometric foundations first, then shows how robotics puts foundation models to work; Ch.13 and Ch.14 then carry you into 3D vision and SLAM.

Readers with a **classical robotics background and less deep-learning experience** can jump in at Ch.8 → Ch.10 → Ch.11 → Ch.12. Dip back into Ch.4–7 only as needed, and focus on how robot learning bridges into VFMs and VLAs.

If you are already **starting a specific project**, begin with the datasets and benchmarks in Ch.17. Pick one or two baseline papers on a closely related task, then trace backward to whichever chapters those methods depend on.

For structured one-, three-, and six-month study schedules, see Ch.20.7.

## 22.3 What Not to Do

Early-stage researchers often fall into one of a few familiar traps: chasing paper count over depth, waiting for a "perfect" setup before running a first experiment, or rebuilding every component from scratch. Taking AI-generated code at face value without checking it creates the opposite problem. [Research Notes Ch.6](../../research-notes/guide.html#chapter-6) and [Grad Notes Ch.11](../../grad-notes/guide.html#chapter-11) examine these pitfalls in detail *(Korean only)*.

Both SLAM and computer vision already have battle-tested public implementations—ORB-SLAM3, COLMAP, and 3D Gaussian Splatting among them. Unless your goal is strictly educational or you have a concrete new design to test, running an existing baseline and diagnosing where it breaks is usually the fastest route to a real research problem.

## 22.4 Pacing Yourself for Long-Term Research

What the first year, the first summer, and the five-year arc of a PhD feel like is covered in [Grad Notes Ch.7](../../grad-notes/guide.html#chapter-7) and [Grad Notes Ch.1](../../grad-notes/guide.html#chapter-1) *(Korean only)*.

Robotics experiments naturally move in long cycles because they involve hardware setup, safety checks, data collection, and repeated physical trials. Preparation overhead and turnaround times vary enormously across platforms, environments, and labs, so gauging your momentum across complete experiment cycles is far more realistic than measuring progress day by day.

## 22.5 Next Steps

You do not need to read this guide straight through from cover to cover. Whenever a project hits a wall, flip back to the chapter you need and check the equations, implementation notes, or benchmark links.

From here, continue with [Research Notes](../../research-notes/guide.html) for reading and writing papers, and [Grad Notes](../../grad-notes/guide.html) for navigating the longer PhD journey *(Korean only)*.

Draft date: 2025.12.28 · Revision date: 2026.05.01
