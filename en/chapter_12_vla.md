# Ch.12 — Vision-Language-Action (VLA) & Embodied AI

The goal of VLA is for a robot to take a natural-language command like "pick up the red cup and place it on the table" and execute it. It is the field that unifies vision, language models, and control, and you need the concepts in this chapter to understand "why ChatGPT cannot move a robot" and "why a policy that worked well in simulation falls apart on a real robot." At CoRL and ICRA 2024–2025, the share of VLA-related papers has grown sharply.

## 12.1 VLA Concepts

Earlier robot systems separated visual perception, language understanding, and action generation into independent pipelines.

**VLA (Vision-Language-Action)** handles these three roles with a single model.

```
Input: image + natural-language command ("pick up the red cup")
Output: robot action (joint angles, gripper commands, etc.)
```

**Embodied AI**: AI that learns while interacting in a physical environment
- Extends beyond perception to include action
- The gap between simulation and the real environment (sim-to-real)

What sets embodied AI apart from earlier AI is that the model does not stop at classifying "this is a cup" — it must physically pick the cup up. This process has to account for gravity, friction, and collisions, which makes it far harder than simple image classification.

> **Further reading**
> - [Google DeepMind Robotics Blog](https://deepmind.google/discover/blog/) — official blog posts on RT-1, RT-2, PaLM-E, and more
> - [Brohan et al., "RT-2: Vision-Language-Action Models" (2023)](https://arxiv.org/abs/2307.15818) — the seminal VLA paper

## 12.2 Key Models and Research

### 12.2.1 RT-1, RT-2 (Google DeepMind)

RT-1 and RT-2 were the first models to empirically demonstrate the concept of "a general-purpose robot policy trained on large-scale data." Before RT-1, the standard approach was to train one task per robot. RT-1/RT-2 showed that a single model can perform hundreds of tasks.

**RT-1 (Robotics Transformer 1)**:
- Trained on large-scale robot demonstration data
- 130K episodes, 700+ tasks
- Tokenized action output

**RT-2 (Robotics Transformer 2)**:
- Fine-tunes a VLM (PaLI-X, PaLM-E) to produce robot actions
- Transfers web-scale knowledge to robots
- Capable of "chain of thought" reasoning

The idea behind RT-2 is simple. Large language/vision models trained on the internet already carry "knowledge about the world," so fine-tuning them to produce robot actions lets them handle new objects or situations zero-shot. For example, RT-2 can pick up objects it never saw in training by leveraging its language knowledge.

> **Further reading**
> - [Google DeepMind — RT-2 Demo Video](https://deepmind.google/discover/blog/rt-2-new-model-translates-vision-and-language-into-action/) — footage and commentary on RT-2 in action
> - [Brohan et al., "RT-1: Robotics Transformer" (2022)](https://arxiv.org/abs/2212.06817) — original RT-1 paper
> - [Brohan et al., "RT-2" (2023)](https://arxiv.org/abs/2307.15818) — original RT-2 paper

### 12.2.2 PaLM-E

**Embodied Multimodal Language Model**:
- PaLM (language) + ViT (vision) + robot state
- 562B parameters
- "Multi-purpose" robot task execution

What makes PaLM-E interesting is that it demonstrated "positive transfer." Jointly training on robot data, web images, and text actually improves robot task performance compared to training on each separately. It empirically showed that general-purpose knowledge also helps robot actions.

> **Further reading**
> - [Driess et al., "PaLM-E: An Embodied Multimodal Language Model" (2023)](https://arxiv.org/abs/2303.03378) — original PaLM-E paper

### 12.2.3 OpenVLA

RT-2 and PaLM-E cannot be used without Google's internal infrastructure. OpenVLA is an open-source VLA model that a lab can actually download, fine-tune, and deploy on a robot.

**Open-source VLA**:
- 7B parameters (based on Llama 2)
- Trained on 970K episodes
- Applicable to diverse robot embodiments

```python
# OpenVLA usage example (conceptual)
from openvla import OpenVLAModel

model = OpenVLAModel.from_pretrained("openvla/openvla-7b")

action = model.predict(
    image=current_image,
    instruction="pick up the blue block and place it on the red target"
)
```

**RT-X project**: Another project to know alongside OpenVLA is RT-X. It collects robot data gathered by many research institutions into one large dataset (Open X-Embodiment) and uses it to train a general-purpose robot policy. The data covers more than 22 robot types.

**Octo**: Another open-source model trained on RT-X data. It is smaller than OpenVLA (93M parameters) and therefore lighter to use. The model is designed for quick fine-tuning on diverse robot platforms.

> **Further reading**
> - [OpenVLA GitHub](https://github.com/openvla/openvla) — code and model weights released
> - [Kim et al., "OpenVLA" (2024)](https://arxiv.org/abs/2406.09246) — OpenVLA paper
> - [Open X-Embodiment Collaboration, "Open X-Embodiment" (2023)](https://arxiv.org/abs/2310.08864) — RT-X dataset paper
> - [Octo GitHub](https://github.com/octo-models/octo) — lightweight open-source robot policy model

### 12.2.4 Navigation

Beyond manipulation, moving (navigation) within an environment is also a core problem for embodied AI. The studies below apply the language understanding of LLMs to navigation.

**LINGO**: Language-guided Indoor Navigation
**SayCan**: separates what the LLM "can do" from what it "should do"
- Affordance function: the actions the robot can currently perform
- LLM: the actions required to achieve the goal

Unpacking SayCan's core idea a bit more: tell an LLM "make me coffee" and it can plan "1. grab a cup, 2. walk to the coffee machine, 3. press the button..." But if the robot is not near a cup right now, "grab a cup" is not executable. SayCan multiplies the LLM's plan (what should be done) with the robot's currently feasible actions (what can be done) to pick an action that is both executable and close to the goal.

> **Further reading**
> - [Ahn et al., "Do As I Can, Not As I Say: Grounding Language in Robotic Affordances" (2022)](https://arxiv.org/abs/2204.01691) — SayCan paper
> - [SayCan project page](https://say-can.github.io/) — includes demo videos

## 12.3 World Models

Running tens of thousands of trial-and-error attempts on a real robot is nearly impossible in terms of time and cost. A world model lets the robot "run a simulation in its head" and decide actions based on the result. It is drawing particular attention in autonomous driving, because it can predict "what happens if the car ahead suddenly stops" without actually having to experience it.

**World Model**: a model that predicts how an environment behaves

**Why is it needed?**
- Enables model-based RL without a real robot
- Handles dangerous exploration inside simulation

**In autonomous driving**:
- **GAIA-1 (Wayve)**: video prediction + action conditioning. A generative model trained on real driving footage that predicts "this is the scene if you turn the wheel this way."
- **DriveDreamer**: driving-scenario generation. Uses text conditioning to generate diverse scenarios, applied to augmenting training data.
- **MILE**: end-to-end driving based on a world model. Trains an implicit world model that predicts future states and derives the driving policy from it.

**Structure**:

```
z_{t+1} = f(z_t, a_t)     # Dynamics model (current state + action → next state)
o_t = g(z_t)              # Observation model (latent state → observation)
r_t = h(z_t, a_t)         # Reward model (reward prediction)
```

You may have noticed this has a structure similar to the state-space model you learned in linear algebra. Think of it as x_{t+1} = Ax_t + Bu_t extended to a nonlinear neural-network version.

> **Further reading**
> - [Hu et al., "GAIA-1: A Generative World Model for Autonomous Driving" (2023)](https://arxiv.org/abs/2309.17080) — Wayve's world model paper
> - [Wang et al., "DriveDreamer" (2023)](https://arxiv.org/abs/2309.09777) — driving-scenario generation paper
> - [Yannic Kilcher — World Models Explained](https://www.youtube.com/watch?v=dPsXxLyqpfs) — video explainer on world models

## 12.4 End-to-End vs Modular

This is the first architectural decision to make when designing a robot system. Without understanding it, you cannot tell "why this system is designed the way it is" when reading a paper.

These are the two philosophies of robot system design.

**End-to-End**:

```
sensor input → [single neural network] → action output
```

- Pros: simple pipeline, no bottleneck from intermediate representations
- Cons: lack of interpretability, requires large-scale data
- Examples: NVIDIA PilotNet, Tesla FSD (presumed)

**Recent end-to-end work in autonomous driving**:
- **UniAD (Unified Autonomous Driving, 2023)**: an end-to-end model that still keeps detection, tracking, mapping, prediction, and planning modules inside to preserve interpretability. CVPR 2023 Best Paper.
- **VAD (Vectorized Scene Representation for Efficient Autonomous Driving, 2023)**: converts scenes into a vectorized representation to train an efficient end-to-end driving policy.
- **GenAD (Generalized Autonomous Driving, 2024)**: an end-to-end system built on generative models that generalizes to diverse driving scenarios.

**Modular**:

```
sensors → [perception] → [prediction] → [planning] → [control] → action
```

- Pros: each module can be developed/debugged independently, interpretable
- Cons: information loss between modules, hard to jointly optimize
- Examples: Apollo, Autoware

**Recent trend**: hybrid approaches. Perception is learning-based while planning and control are model-based to ensure safety, and, as in UniAD, explicit modules sit inside an end-to-end framework.

> **Further reading**
> - [Hu et al., "Planning-oriented Autonomous Driving (UniAD)" (2023)](https://arxiv.org/abs/2212.10156) — CVPR 2023 Best Paper
> - [Jiang et al., "VAD" (2023)](https://arxiv.org/abs/2303.12077) — vectorization-based autonomous driving
> - [Andrej Karpathy — Tesla AI Day 2022 Presentation](https://www.youtube.com/watch?v=ODSJsviD_SU) — end-to-end autonomous driving from a practitioner's view

### End-to-End vs Modular: the 2026 reality

End-to-end is the hot topic at conferences, but most robot systems actually deployed are modular. Why?

- **Debugging**: when an end-to-end model fails, the cause is hard to find. For "why did the robot drop the cup?", a modular system lets you narrow it down to "depth estimation was wrong" or "grasp planning was wrong," but with end-to-end you do not know where it went wrong.
- **Safety guarantees**: a modular system lets you insert safety checks into each module (speed limits, collision detection, etc.). Putting such guarantees into an end-to-end system is difficult.
- **Partial updates**: when you want to improve only the perception module, a modular system lets you swap just that module. End-to-end requires retraining the whole thing.
- **Data efficiency**: end-to-end training needs large-scale data. RT-2 used 130k episodes, OpenVLA 970k. Most labs do not have the resources to collect data at that scale.

A realistic direction: **hybrid**. Perception is generalized with a VFM (foundation model), while planning/control stays modular for safety. The lab's Local/Global Module design (Ch.18) follows this direction.

For end-to-end to fully replace modular, two things must come first: a debuggable end-to-end structure, and few-shot policies that can learn from small-scale data. The safety-guarantee problem is also still open.

## 12.5 Spatial AI + VLA Integration

No matter how good a VLA model is, if it cannot avoid obstacles in real time the robot will run into a wall. Conversely, a robot that is only good at obstacle avoidance cannot carry out a complex command like "bring me coffee." A real robot system runs only when the two levels are integrated.

Connection to the lab's 2-Module Architecture:

**Local (Fast) Perception**:
- Geometric understanding: depth, obstacles, pose
- Real-time response (10–100 Hz)
- Classical or lightweight learned models

**Global (Heavy) Understanding**:
- Semantic understanding: objects, relations, context
- VFM/VLA-based
- Server or cloud processing (1–10 Hz)

**Integration scenario**:

```
1. Local: real-time obstacle avoidance, odometry
2. Global: "find a cup in the kitchen and bring it to the table"
   - Recognize the cup with a VLM
   - Plan a route on the semantic map
3. Local receives Global's waypoints and carries out the actual motion
```

## 12.6 Sim-to-Real & Simulation Platforms

Gathering data on a real robot is slow, expensive, and risky. That is why sim-to-real — training first in simulation and transferring to the real robot — has effectively become the default. But a "reality gap" exists between simulation and reality. The main techniques for closing this gap are summarized below.

**Domain Randomization**: randomly varies textures, lighting, physics parameters, and so on in simulation during training. Once the model is exposed to many conditions, the real environment can be treated as just one more variation among them.

**Major simulation platforms**:
- **NVIDIA Isaac Sim/Lab**: GPU-accelerated physics simulation. It can run thousands of environments in parallel, making it well suited for large-scale reinforcement learning. Isaac Lab is an integrated framework for robot learning research.
- **AI2-THOR (Allen Institute)**: an indoor-environment simulator. You can practice object manipulation in household settings like kitchens and living rooms. One of the most used platforms in embodied AI research.
- **Habitat (Meta)**: enables navigation learning in large-scale 3D-scanned environments (Matterport3D, Gibson, etc.). Provides an annual benchmark through the Habitat Challenge.
- **MuJoCo**: a simulator with strong contact physics. Widely used for robot-arm manipulation and locomotion learning. DeepMind acquired it and then released it as open source.

> **Further reading**
> - [NVIDIA Isaac Lab Documentation](https://isaac-sim.github.io/IsaacLab/) — a simulation framework for robot learning
> - [AI2-THOR Documentation](https://ai2thor.allenai.org/) — indoor-environment simulator
> - [Habitat Documentation](https://aihabitat.org/) — Meta's embodied AI platform
> - [Tobin et al., "Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World" (2017)](https://arxiv.org/abs/1703.06907) — original Domain Randomization paper

## 12.7 Advanced: Imitation Learning

*If you want to become a researcher, start reading from here.*

In VLA and embodied AI, methods for learning a policy split broadly into reinforcement learning (RL) and imitation learning (IL). In robotics, IL is used far more often than RL. To understand why, you need to know the structure of each approach.

**Behavioral Cloning (BC)**

The simplest IL method. Collect demonstration data `{(s_t, a_t)}` from an expert (a human or a script) and perform supervised learning that predicts action `a_t` from state `s_t`.

```
Loss = E[ || π_θ(s_t) - a_t ||^2 ]
```

Simple and easy to implement, but it has a fatal problem: **distribution shift**. At training time it follows the expert's state distribution, but at inference time its own (imperfect) actions determine the next state. Small errors accumulate, the policy drifts into states the expert never visited, and there it does not know what to do.

**DAgger (Dataset Aggregation)**

A representative method for mitigating distribution shift. The core idea is to collect data with the learned policy while querying the expert for labels and adding them to the dataset.

```
1. Train policy π_1 on initial data D = {expert demonstrations}
2. for i = 1, 2, ...
     rollout with π_i → collect visited states {s_t}
     query the expert for actions {a_t^*} at {s_t}
     D = D ∪ {(s_t, a_t^*)}
     train π_{i+1} on D
```

Because querying the expert every time is expensive, human-in-the-loop variants or approximate versions of DAgger (HG-DAgger, ThriftyDAgger, etc.) are used.

**Why is IL used more than RL in robotics?**

| Criterion | RL | IL |
|------|----|----|
| Sample efficiency | needs millions of episodes | hundreds to thousands of demos suffice |
| Reward function | must be designed directly (reward engineering) | not needed |
| Safety | dangerous actions possible during exploration | imitates expert, so relatively safe |
| Sim-to-Real | reward function's sim-real gap is also a problem | using real demo data reduces the gap |

In robotics, designing a reward function properly is very hard. How do you define the reward for "grasp the cup"? Distance between the cup and the gripper? Then the robot may stop just next to the cup. Whether it was grasped? Then you hit the sparse-reward problem. IL sidesteps this.

> **Further reading**
> - [Ross et al., "A Reduction of Imitation Learning and Structured Prediction to No-Regret Online Learning" (2011)](https://arxiv.org/abs/1011.0686) — original DAgger paper
> - [Florence et al., "Implicit Behavioral Cloning" (CoRL 2021)](https://arxiv.org/abs/2109.00137) — an implicit approach to overcome BC's limitations
> - [Zare et al., "A Survey of Imitation Learning" (2024)](https://arxiv.org/abs/2309.15894) — a survey of IL

## 12.8 Advanced: Diffusion Policy

*If you want to become a researcher, start reading from here.*

Diffusion Policy, proposed by Chi et al. (RSS 2023), is a policy representation that is rapidly replacing BC-family methods in robot manipulation. The core idea is to generate an action trajectory through a denoising diffusion process.

**Why diffusion?**

Standard BC deterministically predicts a single action as `π_θ(s) → a`. But in reality, several actions are possible from the same state (multi-modality). For example, when grasping a cup on a table you can grab it from the left or from the right. Deterministic BC outputs the average of the two actions and fails at both. Gaussian mixture models are another option, but you must fix the number of modes in advance.

Diffusion Policy represents this multi-modal distribution naturally.

**How it works**

```
1. Start from random noise a_T ~ N(0, I) (T = diffusion steps)
2. Denoise iteratively conditioned on the current observation s:
   a_{t-1} = denoise_θ(a_t, s, t)    for t = T, T-1, ..., 1
3. The final a_0 is the action trajectory to execute
```

An action trajectory is not a single action but a sequence of actions over several future steps `[a_0, a_1, ..., a_H]`. Only the first few steps are executed (receding horizon), and a new trajectory is generated from the next observation.

**Pros**:
- Represents multi-modal action distributions without explicit assumptions
- Generates an action sequence at once, producing temporally coherent behavior
- Training is stable (denoising score matching converges well)

**Cons**:
- Inference requires many denoising steps and is therefore slow (10–100 steps)
- May be unsuitable for real-time control (>100 Hz). Can be alleviated with acceleration techniques like DDIM or with consistency distillation

**Practical note**: In the experiments of Chi et al.'s original paper, Diffusion Policy beat BC-family methods on 11 of 12 continuous-action tasks. The gap is especially large on contact-rich insertion and assembly tasks.

> **Further reading**
> - [Chi et al., "Diffusion Policy: Visuomotor Policy Learning via Action Diffusion" (RSS 2023)](https://arxiv.org/abs/2303.04137) — original Diffusion Policy paper
> - [Diffusion Policy project page](https://diffusion-policy.cs.columbia.edu/) — code, demos, videos
> - [Ho et al., "Denoising Diffusion Probabilistic Models" (NeurIPS 2020)](https://arxiv.org/abs/2006.11239) — foundational diffusion-model paper

## 12.9 Advanced: Sim-to-Real Transfer

*If you want to become a researcher, start reading from here.*

Section 12.6 briefly covered simulation platforms and domain randomization. Here we systematically lay out the concrete techniques for sim-to-real transfer.

**1. Domain Randomization (DR)**

Randomly varies the parameters of the simulation environment each training step. The assumption is that if the model trains under a sufficiently wide variety of conditions, the real environment will be contained among those variations.

Randomization targets:
- **Visual**: textures, lighting direction/intensity, camera position/field of view, background
- **Physical**: friction coefficients, moments of inertia, link masses, joint damping
- **Dynamics**: actuator latency, sensor noise, control period

DR's limitation: widening the randomization range too much makes training itself hard, while narrowing it too much fails to cover reality. Finding the right range is crucial in practice.

**2. System Identification (SysID)**

Measures or estimates the physical parameters of the real system and calibrates the simulator accordingly.

```
1. Execute a specific trajectory on the real robot to collect data
2. Optimize the simulator's parameters φ:
   φ* = argmin_φ || f_sim(φ) - f_real ||^2
3. Train the policy in the calibrated simulator
```

Traditional and effective, but estimating every parameter accurately is hard, and it is powerless against phenomena the simulator does not model (cable compliance, microscopic deformation of contact surfaces, etc.).

**3. Real-to-Sim-to-Real (R2S2R)**

A recent approach that combines the strengths of DR and SysID.

```
1. Collect a small amount of real data
2. Use the real data to calibrate the simulator (SysID) or model the discrepancy
3. Train a policy in the calibrated simulator
4. Apply the learned policy to the real robot
5. (Repeat) recalibrate the simulator with the real-world results
```

**4. Judging whether transfer succeeded**

The most direct quantitative check: compare the success rate of the same task in sim and real.

- **Sim success rate ≈ Real success rate**: transfer succeeded. The simulator reflects reality well.
- **Sim >> Real**: large reality gap. Expand DR range or correct with SysID.
- **Sim < Real**: rare but happens. The simulator was set to a harder (conservative) condition than reality.

Trajectory similarity, contact-force comparisons, and other metrics are sometimes used as additional indicators.

> **Further reading**
> - [Tobin et al., "Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World" (2017)](https://arxiv.org/abs/1703.06907) — original DR paper
> - [Muratore et al., "Robot Learning from Randomized Simulations" (2022)](https://arxiv.org/abs/2111.00137) — a systematic treatment of DR
> - [Hanna & Stone, "Grounded Action Transformation for Sim-to-Real" (AAAI 2017)](https://arxiv.org/abs/1511.07461) — transfer methodology
> - [NVIDIA Isaac Lab Tutorials](https://isaac-sim.github.io/IsaacLab/) — hands-on DR/SysID pipelines

> **Additional papers (3D/spatial understanding + benchmarks)**
> - [Hong et al., "3D-LLM: Injecting the 3D World into Large Language Models" (NeurIPS 2023, arXiv:2307.12981)](https://arxiv.org/abs/2307.12981) — gives LLMs 3D spatial understanding. 3D captioning, QA, navigation
> - [Chen et al., "SpatialVLM: Endowing Vision-Language Models with Spatial Reasoning" (CVPR 2024, arXiv:2401.12168)](https://arxiv.org/abs/2401.12168) — adds spatial reasoning about distance/size to VLMs
> - [Nasiriany et al., "RoboCasa: Large-Scale Simulation of Everyday Tasks for Generalist Robots" (RSS 2024, arXiv:2406.02523)](https://arxiv.org/abs/2406.02523) — 100 kitchen tasks, 150+ object categories. A household-robot benchmark
> - [Szot et al., "Habitat 3.0: A Co-Habitat for Humans, Avatars, and Robots" (ICLR 2024, arXiv:2310.13724)](https://arxiv.org/abs/2310.13724) — human-robot coexistence simulation. Social navigation, collaborative tasks

> **Technical Timeline: VLA & Embodied AI**
> - **~2015**: per-task imitation learning, research centered on single-object grasping
> - **2017–**: sim-to-real transfer via domain randomization takes off, research on MuJoCo/PyBullet
> - **2020–**: first attempts to combine large language models (LLMs) with vision. Language-based robot control such as CLIPort and SayCan emerges
> - **2022–**: foundation-model-based robot policies appear, including RT-1, RT-2, and PaLM-E. The Open X-Embodiment dataset is built
> - **2024–**: open-source VLA models such as OpenVLA and Octo are released. World-model-based planning draws attention in both autonomous driving and manipulation. End-to-end autonomous driving (UniAD, VAD, GenAD) starts to replace modular approaches
> - **What to watch now**: research applying foundation models to robots has grown rapidly since 2023 (RT-2, OpenVLA, Octo, pi0, etc.). You can fine-tune open-source models like OpenVLA/Octo on your own robot, so hands-on experimentation is recommended.
