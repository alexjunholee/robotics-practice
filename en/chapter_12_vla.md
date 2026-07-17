# Ch.12 — Vision-Language-Action (VLA) & Embodied AI

The goal of VLA is for a robot to take a natural-language command like "pick up the red cup and place it on the table" and execute it. It is the field that unifies vision, language models, and control, and you need the concepts in this chapter to understand "why ChatGPT cannot move a robot" and "why a policy that worked well in simulation falls apart on a real robot."

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
> - [Brohan et al., "RT-2: Vision-Language-Action Models" (2023)](https://arxiv.org/abs/2307.15818) — an early paper that set the VLA framing

## 12.2 Key Models and Research

### 12.2.1 RT-1, RT-2 (Google DeepMind)

RT-1 and RT-2 are examples of combining large robot datasets with web-scale visual and language knowledge in a single policy. In the environment reported by its paper, RT-1 showed that one model could perform hundreds of tasks.

RT-1 (Robotics Transformer 1) was trained on large-scale robot demonstrations: 130K episodes spanning more than 700 tasks. Its output is a tokenized action.

RT-2 (Robotics Transformer 2) fine-tunes VLMs such as PaLI-X and PaLM-E to produce robot actions. Its paper reports transfer from web-scale data and a separate robot-chain-of-thought experiment.

The idea behind RT-2 is simple. Large language/vision models trained on the internet already carry "knowledge about the world," so fine-tuning them to produce robot actions lets them handle new objects or situations zero-shot. For example, RT-2 can pick up objects it never saw in training by leveraging its language knowledge.

> **Further reading**
> - [Google DeepMind — RT-2 Demo Video](https://deepmind.google/discover/blog/rt-2-new-model-translates-vision-and-language-into-action/) — footage and commentary on RT-2 in action
> - [Brohan et al., "RT-1: Robotics Transformer" (2022)](https://arxiv.org/abs/2212.06817) — original RT-1 paper
> - [Brohan et al., "RT-2" (2023)](https://arxiv.org/abs/2307.15818) — original RT-2 paper

### 12.2.2 PaLM-E

PaLM-E is a 562B-parameter embodied multimodal language model that combines PaLM, a ViT, and robot-state inputs. It handles several robot tasks in one model.

What makes PaLM-E interesting is that it demonstrated "positive transfer." Jointly training on robot data, web images, and text actually improves robot task performance compared to training on each separately. It empirically showed that general-purpose knowledge also helps robot actions.

> **Further reading**
> - [Driess et al., "PaLM-E: An Embodied Multimodal Language Model" (2023)](https://arxiv.org/abs/2303.03378) — original PaLM-E paper

### 12.2.3 OpenVLA

The full weights of RT-2 and PaLM-E are not public. OpenVLA publishes its code and weights, so a lab with sufficient compute can download, fine-tune, and deploy it on a robot.

OpenVLA has 7B parameters, is based on Llama 2, and was trained on 970K robot episodes from multiple embodiments.

```python
# OpenVLA usage example (conceptual)
from openvla import OpenVLAModel

model = OpenVLAModel.from_pretrained("openvla/openvla-7b")

action = model.predict(
    image=current_image,
    instruction="pick up the blue block and place it on the red target"
)
```

The RT-X project is another project to know alongside OpenVLA. It combines robot data from many institutions into the Open X-Embodiment dataset and uses it to train general-purpose robot policies. The dataset includes data from more than 22 robot embodiments.

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

Consider how SayCan handles a simple request. Tell an LLM to "make me coffee," and it can plan: "1. grab a cup, 2. walk to the coffee machine, 3. press the button..." But if the robot is not near a cup, it cannot execute the first step. SayCan combines the LLM's plan (what should be done) with the robot's feasible actions (what can be done) to select an action that is both executable and likely to advance the goal.

> **Further reading**
> - [Ahn et al., "Do As I Can, Not As I Say: Grounding Language in Robotic Affordances" (2022)](https://arxiv.org/abs/2204.01691) — SayCan paper
> - [SayCan project page](https://say-can.github.io/) — includes demo videos

## 12.3 World Models

Repeated trial and error on physical robots carries substantial time and equipment costs. A world model predicts a next state or observation from the current state and action, allowing a model-based policy to evaluate candidate actions without executing all of them on hardware.

A world model can be used for model-based RL rollouts and for evaluating risky actions before hardware execution.

In autonomous driving, GAIA-1 predicts action-conditioned driving video, DriveDreamer generates text-conditioned driving scenes, and MILE jointly learns future states and a driving policy through an implicit world model.

Its structure resembles a state-space model.

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

End-to-end and modular architectures differ in where they separate perception, planning, and control.

**End-to-End**:

```
sensor input → [single neural network] → action output
```

- Pros: simple pipeline, no bottleneck from intermediate representations
- Cons: lack of interpretability, requires large-scale data
- Examples: NVIDIA PilotNet, Tesla FSD (presumed)

End-to-end autonomous driving has developed in several forms. UniAD (2023) retains detection, tracking, mapping, prediction, and planning modules inside an end-to-end framework and received the CVPR 2023 Best Paper award. VAD (2023) converts a scene into a vectorized representation, while GenAD (2024) uses a generative formulation for driving scenarios.

**Modular**:

```
sensors → [perception] → [prediction] → [planning] → [control] → action
```

- Pros: each module can be developed/debugged independently, interpretable
- Cons: information loss between modules, hard to jointly optimize
- Examples: Apollo, Autoware

Hybrid designs are another option: perception can be learning-based while planning and control retain explicit models and safety checks, or explicit modules can sit inside an end-to-end framework as in UniAD.

> **Further reading**
> - [Hu et al., "Planning-oriented Autonomous Driving (UniAD)" (2023)](https://arxiv.org/abs/2212.10156) — CVPR 2023 Best Paper
> - [Jiang et al., "VAD" (2023)](https://arxiv.org/abs/2303.12077) — vectorization-based autonomous driving
> - [Andrej Karpathy — Tesla AI Day 2022 Presentation](https://www.youtube.com/watch?v=ODSJsviD_SU) — end-to-end autonomous driving from a practitioner's view

### End-to-End vs Modular in Practice

Which architecture is preferable depends on the application and its verification requirements. Practical systems often retain modular or hybrid structure for the following reasons.

- **Debugging**: when an end-to-end model fails, the cause is hard to find. For "why did the robot drop the cup?", a modular system lets you narrow it down to "depth estimation was wrong" or "grasp planning was wrong," but with end-to-end you do not know where it went wrong.
- **Safety guarantees**: a modular system lets you insert safety checks into each module (speed limits, collision detection, etc.). Putting such guarantees into an end-to-end system is difficult.
- **Partial updates**: a modular system can replace only the perception module. In an end-to-end system, a change may require joint retraining or revalidation of several components.
- **Data efficiency**: the general-purpose policies discussed here use large datasets. RT-1's dataset contained about 130K episodes, and OpenVLA was trained on 970K episodes. Most labs cannot collect the same scale of data themselves.

One practical direction is **hybrid**: use a VFM for perception while retaining explicit planning, control, and safety checks. The Local/Global Module design in Ch.18 follows this direction.

For end-to-end systems to replace modular designs broadly, they must demonstrate debuggability, data efficiency on small datasets, and application-specific safety guarantees. Public systems address these requirements over different scopes; no single architecture establishes all three across applications.

## 12.5 Spatial AI + VLA Integration

A VLA interprets long-horizon tasks such as "bring me coffee," while a local controller handles real-time obstacle avoidance and stabilization. A physical system must connect outputs at both time scales.

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

Gathering data on a real robot is slow, expensive, and risky. Sim-to-real therefore trains first in simulation and transfers the result to a physical robot. A "reality gap" remains between simulation and reality; the main techniques for narrowing it are summarized below.

**Domain Randomization**: randomly varies textures, lighting, physics parameters, and so on in simulation during training. Once the model is exposed to many conditions, the real environment can be treated as just one more variation among them.

Four major simulation platforms illustrate the range of uses. NVIDIA Isaac Sim/Lab provides GPU-accelerated physics and can run thousands of environments in parallel; Isaac Lab is an integrated robot-learning framework. AI2-THOR provides household environments such as kitchens and living rooms for indoor interaction tasks. Habitat supports navigation learning in large-scale scanned environments such as Matterport3D and Gibson and hosts the Habitat Challenge. MuJoCo emphasizes contact dynamics and is used for manipulation and locomotion; after acquiring it, DeepMind released it as open source.

> **Further reading**
> - [NVIDIA Isaac Lab Documentation](https://isaac-sim.github.io/IsaacLab/) — a simulation framework for robot learning
> - [AI2-THOR Documentation](https://ai2thor.allenai.org/) — indoor-environment simulator
> - [Habitat Documentation](https://aihabitat.org/) — Meta's embodied AI platform
> - [Tobin et al., "Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World" (2017)](https://arxiv.org/abs/1703.06907) — original Domain Randomization paper

## 12.7 Advanced: Imitation Learning

Policy learning in VLA and embodied AI broadly divides into reinforcement learning (RL) and imitation learning (IL). The two approaches differ in how they collect data, define rewards, and pay for exploration on physical robots.

**Behavioral Cloning (BC)**

The simplest IL method. Collect demonstration data `{(s_t, a_t)}` from an expert (a human or a script) and perform supervised learning that predicts action `a_t` from state `s_t`.

```
Loss = E[ || π_θ(s_t) - a_t ||^2 ]
```

The method is simple and easy to implement, but it has an important limitation: **distribution shift**. During training, the policy follows the expert's state distribution. During inference, its own imperfect actions determine each subsequent state. Small errors can accumulate until the policy reaches states the expert never visited, where it may fail to choose an appropriate action.

**DAgger (Dataset Aggregation)**

DAgger is a representative method for mitigating distribution shift. It collects data with the learned policy while querying the expert for labels, then adds those labeled examples to the dataset.

```
1. Train policy π_1 on initial data D = {expert demonstrations}
2. for i = 1, 2, ...
     rollout with π_i → collect visited states {s_t}
     query the expert for actions {a_t^*} at {s_t}
     D = D ∪ {(s_t, a_t^*)}
     train π_{i+1} on D
```

Because querying the expert every time is expensive, human-in-the-loop variants or approximate versions of DAgger (HG-DAgger, ThriftyDAgger, etc.) are used.

The table summarizes typical tradeoffs. The actual sample count and safety profile depend on the algorithm, simulator, and quality of the expert data.

| Criterion | RL | IL |
|------|----|----|
| Sample efficiency | may require extensive environment interaction | depends on the number and diversity of expert demonstrations |
| Reward function | must be designed directly (reward engineering) | not needed |
| Safety | dangerous actions possible during exploration | imitates expert, so relatively safe |
| Sim-to-Real | reward function's sim-real gap is also a problem | using real demo data reduces the gap |

In robotics, designing a reward function properly is very hard. How do you define the reward for "grasp the cup"? Distance between the cup and the gripper? Then the robot may stop just next to the cup. Whether it was grasped? Then you hit the sparse-reward problem. IL sidesteps this.

> **Further reading**
> - [Ross et al., "A Reduction of Imitation Learning and Structured Prediction to No-Regret Online Learning" (2011)](https://arxiv.org/abs/1011.0686) — original DAgger paper
> - [Florence et al., "Implicit Behavioral Cloning" (CoRL 2021)](https://arxiv.org/abs/2109.00137) — an implicit approach to overcome BC's limitations
> - [Zare et al., "A Survey of Imitation Learning" (2024)](https://arxiv.org/abs/2309.15894) — a survey of IL

## 12.8 Advanced: Diffusion Policy

Diffusion Policy, proposed by Chi et al. (RSS 2023), is used in robot manipulation as an alternative to BC-family methods. It generates an action trajectory through a denoising diffusion process.

Standard BC deterministically predicts a single action as `π_θ(s) → a`. But in reality, several actions are possible from the same state (multi-modality). For example, when grasping a cup on a table you can grab it from the left or from the right. Deterministic BC outputs the average of the two actions and fails at both. Gaussian mixture models are another option, but you must fix the number of modes in advance.

Diffusion Policy represents this multi-modal distribution naturally.

```
1. Start from random noise a_T ~ N(0, I) (T = diffusion steps)
2. Denoise iteratively conditioned on the current observation s:
   a_{t-1} = denoise_θ(a_t, s, t)    for t = T, T-1, ..., 1
3. The final a_0 is the action trajectory to execute
```

An action trajectory is not a single action but a sequence of actions over several future steps `[a_0, a_1, ..., a_H]`. Only the first few steps are executed (receding horizon), and a new trajectory is generated from the next observation.

This formulation represents a multi-modal action distribution without fixing the number of modes in advance, and it produces a temporally connected action sequence in one trajectory. Training uses denoising score matching.

Its main cost is speed. Inference requires repeated denoising steps (often 10–100), so it may be unsuitable for control above 100 Hz. DDIM-style sampling or consistency distillation can reduce the cost.

The original project page reports an average improvement of 46.9% over prior robot-learning methods across 12 tasks from four benchmarks. Interpret that number within the tasks, metrics, and baselines used in the paper.

> **Further reading**
> - [Chi et al., "Diffusion Policy: Visuomotor Policy Learning via Action Diffusion" (RSS 2023)](https://arxiv.org/abs/2303.04137) — original Diffusion Policy paper
> - [Diffusion Policy project page](https://diffusion-policy.cs.columbia.edu/) — code, demos, videos
> - [Ho et al., "Denoising Diffusion Probabilistic Models" (NeurIPS 2020)](https://arxiv.org/abs/2006.11239) — foundational diffusion-model paper

## 12.9 Advanced: Sim-to-Real Transfer

Section 12.6 introduced simulation platforms and domain randomization. The techniques below address the visual and physical differences encountered when a policy moves from simulation to a physical robot.

**1. Domain Randomization (DR)**

Randomly varies the parameters of the simulation environment each training step. The assumption is that if the model trains under a sufficiently wide variety of conditions, the real environment will be contained among those variations.

Randomization targets:
- **Visual**: textures, lighting direction/intensity, camera position/field of view, background
- **Physical**: friction coefficients, moments of inertia, link masses, joint damping
- **Dynamics**: actuator latency, sensor noise, control period

If the randomization range is too wide, the training problem becomes harder; if it is too narrow, the simulated variations may not cover the physical system. Choosing a suitable range is therefore important.

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

This approach combines the diversity of DR with the precision of SysID.

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
> - **2024–**: open-source VLA models such as OpenVLA and Octo are released. World-model-based planning, end-to-end autonomous driving (UniAD, VAD, GenAD), and modular or hybrid designs are all active research directions
> - **Recent direction**: foundation-model-based robot policies published since 2023 include RT-2, OpenVLA, Octo, and pi0. OpenVLA and Octo provide public code and weights for adaptation experiments.
