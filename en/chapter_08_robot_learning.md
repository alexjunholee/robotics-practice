# Ch.8 — Robot Learning


Robot learning is the field where robots learn behavior from data and experience instead of explicit programming. The main pieces are reinforcement learning (RL), sim-to-real transfer, imitation learning, and recent foundation-model-based approaches.

---

## 8.1 Why Study Robot Learning

**Where traditional methods work well**

Traditional control and planning methods like PID, MPC, and RRT work very well when the dynamics model is accurate and the environment is structured. An industrial robot arm picking and assembling parts at predetermined positions is a representative example. With an accurate model, you get the performance that optimal control theory mathematically guarantees. Learning-based methods have a hard time beating that.

**Where traditional methods struggle**

The problem is that the real world is not clean.

- **Dynamics that are hard to model**: Building an accurate physical model for deformable objects like cloth, rope, or fluids is practically impossible.
- **Complex contact**: Tasks like turning an object in hand or inserting it involve contact modes that change frequently. Accurately modeling contact dynamics is still an open problem.
- **Unstructured environments**: Operating in environments that cannot be pre-modeled, such as home kitchens or disaster sites. You cannot know in advance what objects are where.

In these situations, learning-based approaches approximate the input-output relation directly from data, so they can operate without an explicit model.

**But it is not a silver bullet**

The limitations of learning-based methods must be understood clearly.

- **Sample efficiency**: RL often requires millions of interaction steps. Collecting this data on a real robot is unrealistic in terms of time and cost.
- **Safety**: During learning, the robot can damage itself or its surroundings. Exploration is inherently dangerous.
- **Generalization**: Performance often drops sharply when conditions differ even slightly from those seen during training.

If a problem can be solved with traditional methods, use traditional methods. Learning is a tool to apply where traditional methods hit their limits. Combining the two appropriately is the most realistic approach in practice.

Where, then, are these limits in concrete terms? *Probabilistic Robotics* (Thrun, Burgard, and Fox) frames the motivation by domain. An industrial manipulator working inside a controlled workspace is well served by traditional control. An autonomous underwater vehicle, however, faces changing currents, limited visibility, and shifting buoyancy — building a prior model is not realistic. Autonomous helicopter flight contends with aerodynamic nonlinearities and external disturbances large enough that learning-based control has a measurable edge. A Mars rover must make decisions autonomously because of the communication delay with Earth, and no one can hand it a complete terrain model in advance. Across these domains, how much learning is needed tracks directly with how large the model uncertainty is.


---

## 8.2 RL Basics

### MDP (Markov Decision Process)

The mathematical framework for RL is the MDP. Its components are as follows.

- **State (s)**: The current state of the environment. Robot joint angles, velocities, object positions, etc.
- **Action (a)**: The action taken by the agent. Joint torques, target joint angles, etc.
- **Reward (r)**: A scalar reward signal received as a result of an action. r = R(s, a).
- **Transition (T)**: The state transition probability. T(s'|s, a). The distribution of the next state given the current state and action.
- **Discount factor (γ)**: The discount rate for future rewards. 0 < γ ≤ 1. In robot RL, γ = 0.99 is a common starting value.

The goal is to find a policy π(a|s) that maximizes the cumulative discounted reward.

```
J(π) = E[ Σ_{t=0}^{∞} γ^t · r_t ]
```

The Markov property is the assumption that "the next state depends only on the current state and action." It means you do not need to look at the entire prior history, but in real robots this assumption can break down (partial observability, i.e., a POMDP situation). In that case, observation history is used as the state, or a recurrent policy is used.

The MDP definition establishes the criterion for what is optimal. How to *compute* that optimal policy when the environment model $p(x'|x,a)$ and reward $r$ are known is the subject of the next section.

### MDP Value Iteration

When the environment model $p(x'|x,a)$ and reward $r$ are **known**, dynamic programming computes the optimal policy directly. Following the notation of *Probabilistic Robotics* (Thrun, Burgard, and Fox), the state is written as $x$ here — the same concept as $s$ in the preceding section.

#### Payoff and Horizon

The reward $r(x, a)$ is called the payoff function: the scalar value received immediately when action $a$ is taken in state $x$. A policy $\pi: x \mapsto a$ maps every state to an action. The quality of a policy is measured by the expected cumulative discounted reward.

$$V^\pi(x_0) = \mathbb{E}\left[\sum_{t=0}^{T} \gamma^t \, r(x_t, \pi(x_t))\right]$$

The horizon $T$ splits into three cases.

- **T=1 (greedy)**: Maximize the reward for the next single step only. Simple, but ignores long-term consequences.
- **Finite-horizon**: Optimize up to a fixed $T$ steps. The policy must vary with time $t$ (a time-dependent policy), which makes representation more complex.
- **Infinite-horizon (T=∞)**: When $\gamma < 1$, $V^\pi$ remains finite ($|V^\pi| \leq r_{\max}/(1-\gamma)$), and a time-independent stationary policy exists. Infinite-horizon with discounting is the default setting in robot RL.

The intuition behind $\gamma$: a reward one step away is worth $\gamma$ times as much; two steps away, $\gamma^2$. A lower $\gamma$ makes the robot short-sighted; a higher $\gamma$ makes it plan further ahead. The reason $\gamma = 0.99$ is a common starting point is that it looks far enough ahead to matter while still guaranteeing convergence.

#### Bellman Equation

Under infinite horizon, the optimal value function $V^*(x)$ satisfies the Bellman equation.

$$V^*(x) = \max_a \left[ r(x, a) + \gamma \sum_{x'} p(x' \mid x, a) \, V^*(x') \right]$$

The meaning: the optimal value at state $x$ is the maximum, over all actions $a$, of the immediate reward $r(x,a)$ plus the discounted sum of optimal values of the next states. The structure is recursive.

The optimal policy is extracted greedily from this equation.

$$\pi^*(x) = \arg\max_a \left[ r(x, a) + \gamma \sum_{x'} p(x' \mid x, a) \, V^*(x') \right]$$

Taking the T=1 optimum, extending recursively to T=2, then to $T \to \infty$ derives the Bellman equation above. Each step combines "optimal now plus optimal remainder" — the standard dynamic programming structure.

#### Value Iteration Algorithm

The Bellman equation is a fixed-point equation for $V^*$. Rather than solving it directly, substitute the current estimate $V_k$ on the right-hand side to produce $V_{k+1}$ and repeat. That is Value Iteration.

```
Algorithm MDP_value_iteration():
  For all states x:
    V_0(x) ← 0

  Repeat until convergence:           ε: tolerance (e.g., 1e-6)
    For all states x:
      V_{k+1}(x) ← max_a [ r(x, a) + γ · Σ_{x'} p(x'|x, a) · V_k(x') ]
    if max_x |V_{k+1}(x) - V_k(x)| < ε: break

  Extract optimal policy:
    π*(x) ← argmax_a [ r(x, a) + γ · Σ_{x'} p(x'|x, a) · V_k(x') ]

  return V_k, π*
```

Variable summary: $V_k$ is the value estimate at iteration $k$; $V^*$ is the optimal value after convergence; $\pi^*$ is the optimal policy; $r$ is reward; $\gamma$ is the discount factor; $p(x'|x,a)$ is the transition probability.

The update order is arbitrary. According to Sutton & Barto (2018) §4.4, convergence is guaranteed as long as each state is updated infinitely often. In a discrete state space, the integral in the formula becomes a sum (Σ).

Convergence guarantee: the Bellman update operator is a contraction mapping with contraction rate $\gamma$, so the iteration converges to the unique fixed point $V^*$. After $k$ iterations, the error is at most $\gamma^k$ times the initial error: $\|V_k - V^*\|_\infty \leq \gamma^k \|V_0 - V^*\|_\infty$.

<!-- DEMO: mdp_value_iteration_grid.html -->

#### 2D Grid World Example

Imagine a $5 \times 5$ grid. Each cell is a state $x$, and the four actions are up, down, left, and right. Reaching the goal cell yields $r = +100$; hitting an obstacle cell yields $r = -10$; all other moves cost $r = -1$. Transitions go in the intended direction with probability 0.8 and in each orthogonal direction with probability 0.1 (stochastic transitions).

Running Value Iteration produces a value function shaped like a contour map — high around the goal cell, low around obstacles. In the early iterations, only the states adjacent to the goal hold positive values; as the iterations proceed, the values propagate outward. After convergence, following the direction of maximum value from any cell yields the optimal path. The path automatically detours around obstacles.

That greedy path is $\pi^*$. No explicit path search (RRT, A*) is needed — following the gradient of the value function is enough.

MDP Value Iteration applies when the environment model $p(x'|x,a)$ and $r$ are *given*. Learning by experience when the model is *unknown* is covered in §8.3 (model-free RL). Planning under *partial observation* is in Ch.7 §7.9 (advanced POMDP).

On real robots, the transition probability $p(x'|x,a)$ is rarely known in advance. A method that improves the policy directly, without a model, is what is needed next.

### Policy Gradient Intuition

The core idea of policy gradient is simple.

1. Collect several trajectories with the current policy.
2. Increase the probability of actions in trajectories with high return.
3. Decrease the probability of actions in trajectories with low return.

Written as an equation:

```
∇J(θ) = E[ Σ_t ∇log π_θ(a_t|s_t) · A_t ]
```

A_t is the advantage function, which indicates how much better the action was compared to the average. Parameters θ are updated along this gradient.

Intuitively, the gradient of `log π(a|s)` points in the direction of increasing the probability of action a, and multiplying by the advantage makes good actions selected more often and bad actions less often.

### Value Function, Q-function

- **Value function V^π(s)**: The expected cumulative reward when following policy π from state s.
- **Q-function Q^π(s, a)**: The expected cumulative reward when taking action a in state s and then following π.
- **Advantage A^π(s, a) = Q^π(s, a) - V^π(s)**: How much better action a is compared to the average.

Learning a value function separately reduces variance. Most modern RL algorithms use an actor-critic structure that trains a policy network and a value network together.

### On-policy vs Off-policy

- **On-policy**: Trains only on data collected by the current policy. Data is used once and discarded. PPO is representative. Stable but with low sample efficiency.
- **Off-policy**: Reuses data collected by past policies (replay buffer). SAC and TD3 are representative. Sample efficient but training can be unstable.

In robotics, data collection is costly, so the sample efficiency of off-policy methods is attractive. However, if large-scale parallel environments can be run in simulation, on-policy PPO is also sufficiently competitive.


---

## 8.3 Major RL Algorithms

### PPO (Proximal Policy Optimization)

PPO is an on-policy algorithm proposed by Schulman et al. (2017). The core idea is to limit the size of policy updates. If the policy changes too much from the previous one, the change is clipped.

```
L_CLIP(θ) = E[ min( r_t(θ) · A_t, clip(r_t(θ), 1-ε, 1+ε) · A_t ) ]
```

Here r_t(θ) = π_θ(a_t|s_t) / π_θ_old(a_t|s_t) is the probability ratio, and ε has a default value of 0.2 in the original paper.

PPO is popular because it is relatively simple to implement, insensitive to hyperparameters, and trains stably. When combined with large-scale parallel simulation such as NVIDIA Isaac Lab, data can be collected from thousands of environments simultaneously, so the sample efficiency problem can be solved by sheer volume.

### SAC (Soft Actor-Critic)

SAC is an off-policy algorithm characterized by the addition of entropy regularization. It maximizes reward while simultaneously maximizing the entropy of the policy. That is, it encourages trying as diverse a set of actions as possible.

```
J(π) = E[ Σ_t γ^t ( r_t + α · H(π(·|s_t)) ) ]
```

α is the temperature parameter that balances entropy and reward. There are also methods that adjust α automatically.

It is sample efficient in continuous action spaces. This is because a replay buffer lets collected data be reused multiple times. When collecting data directly on a real robot, off-policy SAC has an advantage over on-policy PPO in terms of data efficiency.

### TD3 (Twin Delayed DDPG)

TD3 is an improved version of DDPG, an off-policy algorithm similar to SAC. Three key improvements:

1. **Twin Q-networks**: Trains two Q-functions and uses the smaller value to reduce overestimation bias.
2. **Delayed policy update**: Updates the policy once after updating the critic multiple times.
3. **Target policy smoothing**: Adds noise to the target action.

Performance is similar to SAC, but since entropy tuning is not needed, there are slightly fewer hyperparameters. However, exploration can be weaker than in SAC.

### Algorithm Selection Guide

| Situation | Recommended algorithm | Reason |
|------|-------------|------|
| Simulation, GPU parallelization possible | PPO | Parallel environments compensate for sample efficiency |
| Real robot, little data | SAC | Off-policy, sample efficient |
| Continuous action space, stability important | SAC or TD3 | Both strong in continuous spaces |
| Discrete action space | PPO or DQN | SAC is continuous-space only |
| Project just getting started | PPO | Easy to tune, easy to debug |

### Stable-Baselines3 Code Example

Basic code for training PPO on the MuJoCo Ant environment.

```python
import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.evaluation import evaluate_policy

# Create parallel environments (8)
vec_env = make_vec_env("Ant-v4", n_envs=8)

# Create PPO agent
model = PPO(
    "MlpPolicy",
    vec_env,
    learning_rate=3e-4,
    n_steps=2048,        # number of steps to collect per rollout
    batch_size=64,
    n_epochs=10,         # number of epochs to train on collected data
    gamma=0.99,
    gae_lambda=0.95,     # GAE (Generalized Advantage Estimation)
    clip_range=0.2,
    verbose=1,
    tensorboard_log="./ppo_ant_tb/",
)

# Train (2M steps total)
model.learn(total_timesteps=2_000_000)

# Evaluate
eval_env = gym.make("Ant-v4")
mean_reward, std_reward = evaluate_policy(model, eval_env, n_eval_episodes=20)
print(f"Mean reward: {mean_reward:.1f} +/- {std_reward:.1f}")

# Save/load model
model.save("ppo_ant")
loaded_model = PPO.load("ppo_ant")
```

The SAC example has a similar structure.

```python
from stable_baselines3 import SAC

model = SAC(
    "MlpPolicy",
    "Ant-v4",
    learning_rate=3e-4,
    buffer_size=1_000_000,    # replay buffer size
    learning_starts=10_000,   # start training after this many steps
    batch_size=256,
    tau=0.005,                # target network soft update rate
    gamma=0.99,
    verbose=1,
)

model.learn(total_timesteps=1_000_000)
```

> Stable-Baselines3 is good for fast prototyping. If you want to understand algorithm internals, CleanRL is recommended. Every algorithm is implemented in a single file, making it easy to follow along with the code.


---

## 8.4 Simulation Environments

In robot RL, simulation is not optional but mandatory, because collecting millions of steps of data on a real robot is unrealistic. The main simulators are summarized below.

### MuJoCo (Multi-Joint dynamics with Contact)

After being acquired by DeepMind, it was open-sourced in 2022. Thanks to its contact simulation quality and stable numerical integration, it has become the standard benchmark environment for RL research. The default engine is CPU-based, and MuJoCo 3.0+ supports GPU parallelization through MJX (JAX backend), but its ecosystem is smaller than Isaac Lab's. It is suited for algorithm benchmarks and small-scale experiments.

### Isaac Lab (NVIDIA)

A robot learning framework built on top of NVIDIA Isaac Sim. With GPU parallel simulation, it can run thousands to tens of thousands of environments simultaneously and supports photorealistic rendering and sensor simulation. An NVIDIA GPU is required, and installation and configuration are complex. It is used for large-scale locomotion training and sim-to-real pipelines.

### PyBullet

An open-source physics engine suitable for beginners. It installs in a single `pip install` line. Its physical accuracy and speed are lower than MuJoCo's, but it is sufficient for first runs of RL code or for quickly validating ideas.

### Brax

A JAX-based physics engine developed by Google. Thanks to JAX's JIT compilation and automatic differentiation, it runs at very high speed on GPU/TPU and can be used for differentiable physics research. However, physical accuracy is limited and it is weak on complex contact scenarios.

### Environment Comparison Table

| Simulator | Physical accuracy | Speed | GPU parallelization | Installation difficulty | Main use |
|-----------|-----------|------|-----------|-----------|---------|
| MuJoCo | High | Moderate | Possible via MJX | Easy | Algorithm benchmarks |
| Isaac Lab | High | Very fast | Thousands to tens of thousands | Hard | Large-scale robot learning |
| PyBullet | Moderate | Slow | No | Very easy | Introduction/education |
| Brax | Low | Very fast | Yes | Moderate | Fast iteration experiments |

For getting started, the MuJoCo + Gymnasium combination is recommended. Move to Isaac Lab when large-scale experiments become necessary.


---

## 8.5 Sim-to-Real Transfer

Applying a policy trained in simulation to a real robot is called sim-to-real transfer. In theory, you train enough in simulation and deploy to the real robot, and you are done. In practice, it does not work that way.

### Reality Gap

A gap exists between simulation and reality.

- **Physical parameter differences**: Friction coefficients, masses, moments of inertia, etc., differ from simulation.
- **Sensor noise**: Real sensors have noise, latency, and drift.
- **Actuator modeling error**: Motor nonlinearity, gear backlash, compliance, etc.
- **Contact model differences**: Simulation's contact models are only approximations of reality.

Even if you hit a reward of 10,000 in simulation, it is common for the real robot to fall over.

### Domain Randomization

The idea is to randomly vary the physical parameters of the simulation so that the policy is trained to be robust and does not depend on specific parameters. OpenAI's Dactyl (2019) randomized hundreds of physical parameters simultaneously and succeeded at sim-to-real, demonstrating the potential of this approach.

Representative parameters to randomize:

- Friction coefficient: uniform sampling between 0.5 and 1.5
- Object mass: 0.8 to 1.2 times the default
- Joint damping: 0.5 to 2.0 times the default
- Sensor noise: add Gaussian noise
- Actuator strength: 0.8 to 1.2 times the default
- Communication delay: random delay of 0 to 2 steps

```python
# Example domain randomization config in Isaac Lab style (pseudo-code)
class RandomizationConfig:
    # Randomized at the start of each episode
    friction_range = (0.5, 1.5)
    mass_scale_range = (0.8, 1.2)
    joint_damping_scale_range = (0.5, 2.0)

    # Applied every step
    obs_noise_std = 0.05           # Gaussian noise on observations
    action_delay_steps = (0, 2)    # delay before applying actions
    push_force_range = (-5.0, 5.0) # external disturbance (N)

def randomize_env(env, config):
    """Called at the start of each episode."""
    import numpy as np
    friction = np.random.uniform(*config.friction_range)
    mass_scale = np.random.uniform(*config.mass_scale_range)
    damping_scale = np.random.uniform(*config.joint_damping_scale_range)
    env.set_friction(friction)
    env.scale_mass(mass_scale)
    env.scale_joint_damping(damping_scale)

def add_obs_noise(obs, config):
    """Add noise to the observation at each step."""
    import numpy as np
    noise = np.random.normal(0, config.obs_noise_std, size=obs.shape)
    return obs + noise
```

With a wide enough randomization range, reality is likely to fall within that range. In exchange, the upper bound on policy performance drops. Performance will necessarily be lower than a policy optimized for specific parameters.

### System Identification (Sys-ID)

This is the opposite approach from domain randomization. The physical parameters of the real robot are measured or estimated as accurately as possible and reflected in the simulation.

Methods:
- Direct measurement: measure mass with an electronic scale, measure friction coefficient experimentally
- Parameter optimization: find parameters that minimize the difference between real robot trajectories and simulation trajectories
- Online adaptation: continuously estimate and update parameters during actual operation

Sys-ID is often used together with domain randomization. The common pattern is to use Sys-ID to pin down approximate parameters and cover the remaining uncertainty with domain randomization.

### Teacher-Student Structure

A method that leverages privileged information, accessible in simulation but not in reality. Training proceeds in two stages.

1. **Teacher training**: In simulation, a policy is trained with privileged information (exact terrain height, exact friction coefficient, exact object position, etc.) included in the state. With abundant information, training is easy.
2. **Student training**: The student is trained to imitate the teacher's behavior using only observations available on the real robot (IMU, joint encoders, cameras, etc.).

This approach had major success in locomotion research on the ANYmal quadruped robot. The teacher knows the exact terrain height map, but the student learns behavior similar to the teacher using only proprioception history.

### Case Studies

**ANYmal Locomotion (ETH Zurich / Robotic Systems Lab)**
- Quadruped locomotion trained with PPO + domain randomization + teacher-student
- Billions of simulation steps followed by zero-shot transfer to the real robot
- Robust walking across stairs, gravel, slopes, and other varied terrain
- Key: large-scale domain randomization + privileged learning + proprioception history

**Dexterous Hand Manipulation (OpenAI, NVIDIA, etc.)**
- Solving a Rubik's cube with the Shadow Hand (OpenAI, 2019)
- Large-scale domain randomization is key: hundreds of physical parameters randomized simultaneously
- Trained on roughly 13,000 years' worth of simulated experience
- Real-world success rate was considerably lower than in simulation, but demonstrated the potential of the learning-based approach


---

## 8.6 Imitation Learning

RL requires designing a reward function and a large amount of training data. Imitation learning, in contrast, learns a policy directly from demonstration data provided by an expert (a human). This is "learning by watching."

### Behavioral Cloning (BC)

The simplest form of imitation learning. Expert (observation, action) pairs are collected, and a policy is trained with supervised learning.

```python
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset

class BCPolicy(nn.Module):
    def __init__(self, obs_dim, act_dim, hidden_dim=256):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, act_dim),
        )

    def forward(self, obs):
        return self.net(obs)

# Load expert data (NumPy -> Tensor)
# expert_obs: (N, obs_dim), expert_act: (N, act_dim)
dataset = TensorDataset(
    torch.FloatTensor(expert_obs),
    torch.FloatTensor(expert_act),
)
loader = DataLoader(dataset, batch_size=256, shuffle=True)

policy = BCPolicy(obs_dim=48, act_dim=7)
optimizer = torch.optim.Adam(policy.parameters(), lr=1e-3)
loss_fn = nn.MSELoss()

# Training
for epoch in range(100):
    total_loss = 0.0
    for obs_batch, act_batch in loader:
        pred_act = policy(obs_batch)
        loss = loss_fn(pred_act, act_batch)
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
        total_loss += loss.item()
    if (epoch + 1) % 10 == 0:
        print(f"Epoch {epoch+1}, Loss: {total_loss/len(loader):.4f}")
```

**Compounding error problem**: A structural limitation of BC. Once the learned policy deviates even slightly from the expert trajectory, it reaches states not in the training data. Behavior there is unpredictable, deviation grows, and errors accumulate. Errors can grow exponentially over time.

### DAgger (Dataset Aggregation)

DAgger is a method for addressing compounding error.

1. Train a BC policy on initial expert data.
2. Run the trained policy to collect new trajectories.
3. Label what action the expert would take at each state in these trajectories.
4. Add the new data to the existing dataset and retrain.
5. Repeat steps 2-4.

The key is to include the expert action at "states the policy actually visits" in the training data. Theoretically, DAgger has a no-regret guarantee.

The downside is that the expert must label repeatedly. A human has to provide corrections one by one, which is labor-intensive.

### ACT (Action Chunking with Transformers)

A method proposed in Stanford's ALOHA project. Two core ideas:

1. **Action chunking**: Instead of predicting one action at a time, predict a sequence of k future action steps at once. This captures temporal correlation and reduces compounding error.
2. **CVAE (Conditional Variational Autoencoder)**: Models the multimodal distribution of actions. Even in the same situation, there can be multiple valid actions, and a plain MSE loss averages them out, producing mediocre actions.

The architecture uses a Transformer encoder-decoder, taking joint positions and camera images as input.

### Diffusion Policy

A method proposed by Chi et al. (2023) at CMU, applying diffusion models to action generation.

Whereas traditional BC models actions with a unimodal Gaussian, diffusion policy can express arbitrarily complex action distributions through a denoising process. It handles multimodal distributions particularly well.

```python
# Action generation process of Diffusion Policy (pseudo-code)
# 1. Start from pure noise
action = torch.randn(batch_size, horizon, action_dim)

# 2. K denoising steps
for k in reversed(range(K)):
    # Predict noise conditioned on current observation
    predicted_noise = noise_pred_net(action, k, obs_encoding)
    # Remove noise (using a DDPM or DDIM scheduler)
    action = scheduler.step(predicted_noise, k, action)

# 3. Output the final action sequence
```

Diffusion policy and ACT have become the main baselines for manipulation imitation learning since 2023. Both are implemented in public frameworks such as LeRobot (HuggingFace).

### Data Collection Methods

Imitation learning performance depends decisively on data quality. Main data collection methods:

- **Teleoperation**: A human remotely controls the robot. ALOHA uses a leader-follower structure and can collect bimanual manipulation data relatively cheaply.
- **VR controller**: A VR controller specifies end-effector position/orientation. Intuitive, but may lack force feedback in contact-rich tasks.
- **Kinesthetic teaching**: Grab the robot arm directly and move it. The most intuitive, but difficult for large or heavy robots.
- **Space mouse**: A 6-DoF input device. Operable with one hand. Useful for precision work.

The amount of data varies by task and method. The Chi et al. (2023) Diffusion Policy paper showed meaningful performance with about 100-200 demonstrations. More is better, but there is a trade-off with collection cost.


---

## 8.7 Advanced: Foundation Models for Robot Control

*If you want to become a researcher, read from here.*

Inspired by the success of LLMs and VLMs, attempts to build large-scale pretrained models (foundation models) continue in robotics. The idea is to train a generalist policy on large quantities of robot data and adapt quickly to new robots or tasks.

### RT-1, RT-2 (Google DeepMind)

**RT-1 (2022)**: A Transformer-based policy trained on 130,000 robot demonstrations (collected over about 17 months). It takes images and natural language commands as input and outputs actions. A single model performs more than 700 tasks.

**RT-2 (2023)**: A VLM (Vision-Language Model) fine-tuned directly to produce action outputs. PaLM-E and PaLI-X are used as base models. It showed that web-scale pretrained knowledge transfers to robot control. It generalized to some extent even to objects not seen in the training data.

### Octo

An open-source generalist robot policy developed by UC Berkeley and others. It was trained on the Open X-Embodiment dataset (data collected from diverse robots and diverse institutions). It uses a diffusion-based action head and is designed to be fine-tuned to new robots.

### pi0 (Physical Intelligence)

A diffusion-based generalist robot policy released in 2024. It uses a VLM as backbone and generates actions via flow matching. It achieved state-of-the-art performance on diverse manipulation tasks and also worked on complex long-horizon tasks such as folding laundry.

### OpenVLA

An open-source VLA (Vision-Language-Action) model. A 7B-parameter VLM was fine-tuned to output action tokens. Its core contribution is being open source and accessible to anyone.

### Realistic Assessment

Foundation models for robotics are still in an early stage. To be honest:

- On specific tasks, task-specialized traditional methods or task-specific learning often do better.
- The cost of large-scale robot data collection is very high. The scale is different from internet text/image data.
- There is no safety guarantee. It is hard to predict foundation model behavior.
- Inference latency may not be sufficient for real-time control.

### Research Directions

- **Data scaling**: Efforts to combine data from multiple institutions, like Open X-Embodiment. Whether more data improves generalization is still being verified.
- **Cross-embodiment transfer**: Research on transferring a policy trained on one robot to another. The core problem is how to unify different action spaces.
- **Efficient fine-tuning**: Rapid adaptation to new tasks via parameter-efficient fine-tuning such as LoRA.
- **Action representation**: How to tokenize/represent actions. Discretization, continuous distributions, diffusion, and other approaches are competing.


---

## 8.8 Advanced: Reward Design and Safe RL

*If you want to become a researcher, read from here.*

It is no exaggeration to say that the success of RL hinges on reward function design. And applying RL to real robots requires addressing safety.

### Reward Shaping

**The problem with sparse rewards**: Sparse rewards like "+1 if the goal is reached, 0 otherwise" are easy to define, but the agent has to search randomly until it happens to receive a reward. When the state-action space is large, learning effectively fails.

**Dense reward**: Add rewards for intermediate progress. For example, in an object-grasping task:

```python
def compute_reward(gripper_pos, object_pos, target_pos, is_grasped):
    # 1. Bring the gripper close to the object
    dist_to_object = np.linalg.norm(gripper_pos - object_pos)
    reaching_reward = -1.0 * dist_to_object

    # 2. Bonus if the object is grasped
    grasp_reward = 5.0 if is_grasped else 0.0

    # 3. Bring the object close to the target position
    if is_grasped:
        dist_to_target = np.linalg.norm(object_pos - target_pos)
        place_reward = -1.0 * dist_to_target
    else:
        place_reward = 0.0

    # 4. Goal-reached bonus
    success_reward = 10.0 if (is_grasped and
        np.linalg.norm(object_pos - target_pos) < 0.05) else 0.0

    return reaching_reward + grasp_reward + place_reward + success_reward
```

**Curriculum learning**: A method of starting from easy tasks and gradually moving to harder ones. For example, in locomotion, start with walking on flat ground, then move to small obstacles, then stairs. This way, the agent can accumulate success experiences early in training even under sparse rewards.

### Reward Hacking

A phenomenon where the agent maximizes reward but in ways unintended by the designer.

Representative examples:
- A robot arm told to "move" an object instead pushes the object to the target position (without grasping)
- A walking robot told to "move fast" slides while falling
- Told to learn to jump, it evolves into an abnormally elongated shape (when combined with morphology optimization)

Countermeasures:
- Iteratively modify the reward function and review the learned behavior (essentially trial-and-error).
- Add penalty terms for undesired behavior.
- Review qualitatively by watching video. This is a part that is difficult to automate.

### Constrained RL

RL that explicitly handles safety constraints. While standard RL maximizes reward, constrained RL maximizes reward subject to keeping cost below a bound.

```
max_π E[ Σ γ^t r_t ]   subject to   E[ Σ γ^t c_t ] ≤ d
```

c_t is cost (e.g., exceeding joint torque limits, colliding with obstacles), and d is the allowed bound.

Representative algorithms include CPO (Constrained Policy Optimization), PCPO, and Lagrangian-relaxation-based methods. On real robots, putting torque limits and joint angle limits as constraints for hardware protection is the realistic approach.

### Human-in-the-loop RL

An approach that uses human feedback as a reward signal. It applies the same idea as RLHF (RL from Human Feedback) in LLMs to robotics.

Method:
1. Show pairs of robot behaviors and have humans indicate preferences (A is better than B).
2. Train a reward model on the preference data.
3. Run RL using the learned reward model.

Useful for tasks where reward is hard to define numerically (e.g., "walk naturally", "place objects carefully"). The drawbacks are that it takes a lot of human time, and the reward model can be inaccurate.


---

## 8.9 Further Reading

> **Sutton & Barto, "Reinforcement Learning: An Introduction" (2nd edition)**
> http://incompleteideas.net/book/the-book-2nd.html
> The essential RL textbook. Free PDF available. Required reading to build foundations from MDPs to policy gradients. If you do not have time to read it all, prioritize Ch.1-6 and Ch.13.

> **Sergey Levine, CS285: Deep Reinforcement Learning**
> https://rail.eecs.berkeley.edu/deeprlcourse/
> A graduate-level course focused on robot RL. Lecture videos and slides are publicly available. Covers most of this chapter's topics in more depth.

> **Stable-Baselines3**
> https://stable-baselines3.readthedocs.io/
> A PyTorch-based RL algorithm library. PPO, SAC, TD3, and other major algorithms are implemented. Suitable for fast prototyping.

> **CleanRL**
> https://github.com/vwxyzjn/cleanrl
> A collection of single-file RL implementations. Each file contains an entire algorithm, making it easy to study by following along with the code. If you want to understand algorithm internals, this is recommended over SB3.

> **Isaac Lab**
> https://isaac-sim.github.io/IsaacLab/
> NVIDIA's GPU parallel robot simulation framework. Widely adopted in large-scale training projects such as ANYmal locomotion and dexterous manipulation.

> **LeRobot (HuggingFace)**
> https://github.com/huggingface/lerobot
> A framework for imitation learning and robot learning. Includes implementations of ACT, Diffusion Policy, and others. Datasets are provided as well.

> **robomimic**
> https://robomimic.github.io/
> An imitation learning algorithm benchmark. Various imitation learning methods such as BC, BC-RNN, and HBC can be compared under identical conditions.

> **Additional papers**
> - [Andrychowicz et al., "Hindsight Experience Replay" (NeurIPS 2017, arXiv:1707.01495)](https://arxiv.org/abs/1707.01495) — A cornerstone for solving the sparse reward problem. Relabels failed trajectories as successes.
> - [Hafner et al., "Mastering Diverse Domains through World Models" (DreamerV3, arXiv:2301.04104)](https://arxiv.org/abs/2301.04104) — Trains 150+ domains with a single set of hyperparameters. A representative recent result in world-model-based RL.
> - [Chi et al., "Universal Manipulation Interface" (UMI, RSS 2024, arXiv:2402.10329)](https://arxiv.org/abs/2402.10329) — Data collection with a handheld gripper, zero-shot deployment across diverse robots.
> - [Fu et al., "Mobile ALOHA" (CoRL 2024, arXiv:2401.02117)](https://arxiv.org/abs/2401.02117) — Mobile base + bimanual teleoperation. Co-training significantly improves success rate.


---

## Technical Timeline

```
1992 ── REINFORCE algorithm (Williams)
         The first policy gradient method. Convergence is proven but variance is high.

2013 ── DQN (Mnih et al., Atari)
         The beginning of Deep RL. Stable training via replay buffer + target network.

2015 ── TRPO (Schulman et al.)
         Stable policy updates with a trust region constraint. Strong theory, complex implementation.

2017 ── PPO (Schulman et al.)
         A practical alternative to TRPO. Simple to implement with clipping. The default baseline for robot RL experiments.

2018 ── SAC (Haarnoja et al.)
         Entropy regularization + off-policy. Sample efficient in continuous spaces.

2019 ── ANYmal: sim-to-real locomotion (ETH Zurich)
         Succeeded at quadruped sim-to-real through large-scale domain randomization.

2020 ── Widespread practical adoption of DAgger
         Imitation learning proved practical. Applied across diverse robot platforms.

2022 ── RT-1 (Google)
         Large-scale robot data + Transformer. A multi-task generalist policy.

2023 ── ACT/ALOHA (Stanford), Diffusion Policy (CMU)
         A new standard for imitation learning. Performance gains via action chunking and diffusion.

2023 ── RT-2 (Google)
         VLM used directly for action generation. Transfer of web knowledge to robotics.

2024 ── Octo, OpenVLA, pi0
         Emergence of open-source generalist policy models. Start of cross-embodiment learning.

2025 ── Spread of cross-embodiment learning research
         Policy transfer across different robots. Data scaling laws under verification.
```
