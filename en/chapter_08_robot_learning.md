# Ch.8 — Robot Learning

Classical control (Ch.6) and motion planning (Ch.7) perform remarkably well when geometry and dynamics are known in advance, yet they stall when contact dynamics, deformable materials, or cluttered human spaces resist analytical modeling. **Robot learning** turns the problem around: instead of hand-coding every trajectory or physical equation, the robot acquires behavior directly from interaction data and human demonstrations—spanning reinforcement learning (RL), sim-to-real transfer, imitation learning, and vision-language-action (VLA) foundation models.

---

## 8.1 Why Study Robot Learning

Traditional controllers and planners such as PID, MPC, and RRT remain the gold standard whenever an accurate dynamics model exists inside a structured environment. On an industrial assembly line where a robot arm picks rigid parts from fixed fixtures, classical methods offer mathematical stability and optimality guarantees under their model and solver assumptions; on tasks that satisfy those assumptions, any learned policy has to prove its value empirically before earning its place.

The real world, however, rarely stays that tidy. Three regimes routinely defeat purely analytical modeling:

- **Dynamics that defy closed-form physics**: Writing an accurate, fast physical model for deformable cloth, tangled cables, or sloshing fluids is practically impossible.
- **Rich, rapidly switching contact**: Twisting a cap off a bottle, turning a key, or inserting a tight peg flips between sticking, slipping, and separation faster than contact-mode models can reliably predict.
- **Unstructured environments**: Domestic kitchens and disaster sites cannot be CAD-modeled ahead of time, leaving object shapes, friction, and clutter unknown until execution.

Where explicit models break down, learning-based methods approximate perception-to-action mappings directly from data. Yet learning carries real engineering costs of its own: RL often demands millions of interaction steps (prohibitive on physical hardware), exploratory trial-and-error can damage the robot or workspace, and learned policies frequently degrade when test conditions shift slightly from training. Good engineering judgment therefore puts classical methods first whenever they solve the problem cleanly, reserves learning for where analytical models run out, and combines both whenever possible.

*Probabilistic Robotics* (Thrun, Burgard, and Fox; Chapter 15 of the 1999–2000 early draft) illustrates this gradient across four contrasting domains—a manipulator, an autonomous underwater vehicle (AUV), a helicopter, and planetary exploration rovers. Viewed through the lens of robot learning, the degree of learning required tracks directly with model uncertainty: an industrial arm inside a fixed cell runs cleanly on classical control, whereas an AUV battles unpredictable currents, murkiness, and shifting buoyancy; a helicopter fights wind gusts severe enough that obstacle clearance itself becomes a stochastic trade-off; and planetary rovers traverse unknown terrain without prior relative localization.

---

## 8.2 RL Basics

### MDP (Markov Decision Process)

Before an agent can learn from trial and error, we need a clean mathematical language for sequential decision-making under uncertainty. Reinforcement learning adopts the **Markov Decision Process (MDP)**, packaging every closed-loop interaction into five pieces:

- **State (s)**: A complete description of the environment at the current step—joint angles, joint velocities, object poses, and so on.
- **Action (a)**: The command the agent applies—joint torques, target positions, or end-effector velocities.
- **Reward (r)**: A scalar feedback signal returned after an action, written r = R(s, a), telling the agent *what* to optimize without specifying *how*.
- **Transition (T)**: The transition probability T(s'|s, a) governing how the environment evolves to the next state given the current state and action.
- **Discount factor (γ)**: A multiplier in 0 < γ ≤ 1 that down-weights distant future rewards; in robot RL, γ = 0.99 is a common starting point.

The agent seeks a policy π(a|s) that maximizes the expected cumulative discounted reward over time:

```
J(π) = E[ Σ_{t=0}^{∞} γ^t · r_t ]
```

At the heart of this formulation lies the **Markov property**: the next state depends solely on the current state and action, not on the entire past trajectory. On real hardware, the underlying physics still evolves Markovianly, yet raw sensor readings rarely reveal the full latent state (occlusion, velocity estimation lag, or unmeasured friction). That partial observability turns the observation stream into a POMDP (Ch.7 §7.9), which practical RL systems handle by stacking observation histories, feeding belief estimates, or adding recurrent layers to the policy.

Defining the MDP specifies *what* counts as optimal. When the transition model $p(x'|x,a)$ and reward $r$ happen to be known, dynamic programming can compute that optimal policy directly—the subject of the next section.

### MDP Value Iteration

When the environment model $p(x'|x,a)$ and reward $r$ are **known**, dynamic programming solves for the optimal policy without ever collecting trial-and-error rollouts. Following the notation of *Probabilistic Robotics* (Thrun, Burgard, and Fox), we denote the state by $x$ here—representing the same quantity as $s$ above.

#### Payoff and Horizon

The immediate reward $r(x, a)$—the **payoff function**—measures the instant scalar value earned by executing action $a$ in state $x$. A deterministic policy $\pi: x \mapsto a$ assigns an action to every state, and its long-term quality from initial state $x_0$ is the expected discounted return:

$$V^\pi(x_0) = \mathbb{E}\left[\sum_{t=0}^{T} \gamma^t \, r(x_t, \pi(x_t))\right]$$

Depending on the planning horizon $T$, three distinct regimes arise:

- **T=1 (greedy)**: Maximizes only the next step's immediate reward—trivial to compute, yet blind to delayed consequences or traps.
- **Finite-horizon**: Optimizes over a fixed $T$-step window, forcing the optimal policy to vary with time remaining (a time-dependent policy).
- **Infinite-horizon (T=∞)**: Whenever $\gamma < 1$, the sum stays bounded ($|V^\pi| \leq r_{\max}/(1-\gamma)$) and admits a single stationary, time-independent policy—making discounted infinite horizons the standard choice in robot RL.

Intuitively, $\gamma$ sets the agent's effective planning horizon: a reward one step ahead is worth $\gamma$, two steps ahead $\gamma^2$, and so on. Small $\gamma$ breeds short-sighted behavior, whereas $\gamma = 0.99$ looks roughly a hundred steps ahead while keeping the value sum convergent.

#### Bellman Equation

In an infinite-horizon MDP, the optimal value function $V^*(x)$ obeys a simple self-consistency rule: the best total return from state $x$ equals the best immediate reward plus the discounted optimal return from wherever the system lands next. That recursive identity is the **Bellman optimality equation**:

$$V^*(x) = \max_a \left[ r(x, a) + \gamma \sum_{x'} p(x' \mid x, a) \, V^*(x') \right]$$

Once $V^*$ is known, extracting the optimal policy $\pi^*(x)$ simply picks the action that maximizes that one-step lookahead:

$$\pi^*(x) = \arg\max_a \left[ r(x, a) + \gamma \sum_{x'} p(x' \mid x, a) \, V^*(x') \right]$$

Unrolling the $T=1$ optimum backward to $T=2$ and letting $T \to \infty$ produces this exact fixed-point relation: "act optimally now, then follow the optimal remainder."

#### Value Iteration Algorithm

Because $V^*$ appears on both sides of the Bellman equation, we cannot solve for it in closed form with $\max_a$ inside. Instead, **Value Iteration** turns the equation into an iterative update: start with a zero guess $V_0$, plug the current estimate $V_k$ into the right-hand side to obtain $V_{k+1}$, and sweep across all states until updates shrink below a tolerance $\varepsilon$:

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

Here $V_k$ is the value estimate at sweep $k$, $V^*$ is its converged limit, $\pi^*$ is the extracted greedy policy, $r$ is reward, $\gamma$ is the discount factor, and $p(x'|x,a)$ is the transition model.

Sweeping order does not matter: as shown in Sutton & Barto (2018) §4.5, convergence holds as long as every state is updated infinitely often. (In continuous state spaces, the sum Σ becomes an integral.) Furthermore, because the Bellman optimality operator is a $\gamma$-contraction in the max norm, convergence to the unique fixed point $V^*$ is guaranteed geometrically: after $k$ iterations, $\|V_k - V^*\|_\infty \leq \gamma^k \|V_0 - V^*\|_\infty$.

<!-- DEMO: mdp_value_iteration_grid.html -->

#### 2D Grid World Example

To watch Value Iteration in action, picture a $5 \times 5$ grid where each cell is a state $x$ and the agent chooses among four actions (up, down, left, right). Reaching the goal pays $r = +100$, stepping onto an obstacle pays $r = -10$, and every other move costs $r = -1$. Motion is stochastic: the agent moves in the commanded direction with probability 0.8 and slips sideways to each orthogonal cell with probability 0.1.

At first, only cells adjacent to the goal hold high values. With each sweep, high values ripple outward like contour lines across the grid while obstacles carve out deep valleys. Extracting the greedy action in every cell after convergence yields a complete policy $\pi^*$ that steers clear of hazards automatically—no explicit path planner such as A* or RRT is needed, though stochastic transitions still make each individual trajectory vary from run to run.

Value Iteration relies on knowing $p(x'|x,a)$ and $r$ upfront. When partial observability obscures the state, planning moves into belief space as in Ch.7 §7.9; when the transition model itself is unknown—as on nearly every real robot—we turn to model-free RL (§8.3), improving the policy directly from sampled experience.

### Policy Gradient Intuition

Without a transition model $p(x'|x,a)$, the agent cannot compute expected next-state values analytically. **Policy gradient** methods bypass the model altogether by running the parameterized policy $\pi_\theta$, observing which rollouts pay off, and nudging the parameters directly:

1. Roll out several trajectories under the current policy.
2. Push up the log-probability of actions belonging to high-return trajectories.
3. Push down the log-probability of actions belonging to low-return trajectories.

Mathematically, this intuition takes the form of the policy gradient theorem:

```
∇J(θ) = E[ Σ_t ∇log π_θ(a_t|s_t) · A_t ]
```

Here `∇log π_θ(a_t|s_t)` points in the direction that makes action `a_t` more likely in state `s_t`, while the **advantage** `A_t` measures how much better that action performed relative to the state's baseline expectation—ensuring better-than-average choices are reinforced and worse-than-average ones suppressed.

### Value Function, Q-function

To estimate `A_t` cleanly without wild Monte Carlo variance, RL defines three linked value quantities:

- **Value function V^π(s)**: The expected discounted return starting from state s and following policy π thereafter.
- **Q-function Q^π(s, a)**: The expected discounted return starting from state s, taking action a, and following π thereafter.
- **Advantage A^π(s, a) = Q^π(s, a) - V^π(s)**: How much better (or worse) action a is compared to the policy's average behavior in state s.

Subtracting the baseline `V^π(s)` drastically lowers gradient variance. Consequently, most modern robot RL algorithms follow an **actor-critic** design, co-training a policy network (the actor) alongside a value network (the critic).

### On-policy vs Off-policy

A fundamental design split governs how RL algorithms consume experience:

- **On-policy**: Updates only from trajectories generated by the current policy; a fresh rollout may be reused for a few gradient epochs, after which old data is discarded rather than kept in a persistent replay buffer. **PPO** is the canonical example—stable and predictable, yet relatively sample-hungry.
- **Off-policy**: Stores past transitions from earlier policies inside a **replay buffer** and samples mini-batches repeatedly. **SAC** and **TD3** follow this route, achieving high sample efficiency at the price of greater sensitivity to critic instability and hyperparameter tuning.

On physical robots, hardware wear and wall-clock time favor off-policy data reuse. Inside GPU-accelerated simulators running thousands of parallel environments, however, generating fresh rollouts is cheap enough that on-policy PPO often wins on overall speed and stability.

---

## 8.3 Major RL Algorithms

With the actor-critic and on-policy/off-policy trade-offs established, three algorithms dominate modern continuous-control robotics: **PPO**, **SAC**, and **TD3**.

### PPO (Proximal Policy Optimization)

Unconstrained policy gradient updates carry a nasty failure mode: a single overly large gradient step can wreck the policy, causing subsequent rollouts to collect garbage data from which training never recovers. TRPO solved this by imposing an explicit KL-divergence trust-region constraint at every update, at the cost of complex second-order optimization. **PPO** (Schulman et al., 2017) achieves the same safeguard in first-order code by clipping the probability ratio inside its surrogate objective:

```
L_CLIP(θ) = E[ min( r_t(θ) · A_t, clip(r_t(θ), 1-ε, 1+ε) · A_t ) ]
```

Here `r_t(θ) = π_θ(a_t|s_t) / π_θ_old(a_t|s_t)` measures how much the action probability changed since the rollout was collected, and `ε` defaults to 0.2 in the original paper. Whenever `r_t(θ)` moves outside `[1-ε, 1+ε]` in the direction that would inflate the objective, clipping removes the incentive to push the policy further.

PPO has become the default baseline in robotics because its implementation is concise and its hyperparameters generalize across many tasks—though reproducibility studies caution that seemingly minor code-level details such as advantage normalization and value clipping strongly affect final scores. Paired with massively parallel GPU simulation such as NVIDIA Isaac Lab, PPO turns thousands of simultaneous environments into a fast, stable training engine.

### SAC (Soft Actor-Critic)

When data comes from a real robot or a slow simulator rather than thousands of parallel GPU workers, throwing away every rollout after one update is wasteful. **SAC** pairs an off-policy replay buffer with **maximum-entropy regularization**, optimizing expected return plus the policy's own entropy:

```
J(π) = E[ Σ_t γ^t ( r_t + α · H(π(·|s_t)) ) ]
```

where `α` is a temperature weight (often tuned automatically) balancing reward against behavioral diversity. Rather than collapsing prematurely onto one brittle trajectory, SAC actively spreads probability mass across all high-performing actions, improving both exploration and robustness to disturbances while reusing past transitions from its replay buffer for continuous control.

### TD3 (Twin Delayed DDPG)

**TD3** offers a deterministic off-policy alternative to SAC by fixing the overestimation bias and brittle actor updates of earlier DDPG algorithms through three targeted additions:

1. **Twin Q-networks**: Trains two independent critics and takes the minimum of their estimates to curb optimistic overestimation.
2. **Delayed policy update**: Updates the actor less frequently than the critics so value estimates settle before shaping the policy.
3. **Target policy smoothing**: Adds small clipped noise to target actions when computing critic backups, smoothing sharp Q-function peaks.

TD3 matches SAC's asymptotic performance on many continuous benchmarks and drops the entropy temperature tuning, though its deterministic exploration noise can explore less broadly than SAC's stochastic entropy objective.

### Algorithm Selection Guide

Choosing among these algorithms comes down to where rollouts are generated and how expensive each environment step is:

| Situation | Recommended algorithm | Reason |
|------|-------------|------|
| Simulation, GPU parallelization possible | PPO | Parallel environments compensate for sample efficiency |
| Real robot, little data | SAC | Off-policy, sample efficient |
| Continuous action space, stability important | SAC or TD3 | Both strong in continuous spaces |
| Discrete action space | PPO or DQN | The original SAC targets continuous actions; a discrete variant requires a separate implementation |
| Project just getting started | PPO | Easy to tune, easy to debug |

### Stable-Baselines3 Code Example

For rapid prototyping, **Stable-Baselines3** wraps these algorithms behind a consistent PyTorch API. The snippet below trains an 8-environment vectorized PPO agent on MuJoCo's `Ant-v4` task, evaluates it, and saves the checkpoint:

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

Switching to off-policy SAC follows nearly the same interface, replacing rollout length and clipping parameters with replay-buffer capacity and soft target-update rate (`tau`):

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

> Stable-Baselines3 is well suited to fast prototyping. When you need to inspect or modify algorithm internals directly, CleanRL provides self-contained single-file implementations that are easy to read line by line.

---

## 8.4 Simulation Environments

Because collecting millions of exploratory steps on physical hardware is slow, costly, and hazardous, nearly all robot RL training begins inside a physics simulator. Choosing a simulator trades off contact accuracy, GPU throughput, installation overhead, and available robot assets.

### MuJoCo (Multi-Joint dynamics with Contact)

Acquired by DeepMind, released free in 2021, and fully open-sourced in 2022, **MuJoCo** sets the standard for RL control benchmarks thanks to its accurate contact solver and stable numerical integration. Its classic engine runs on CPU, while MuJoCo 3.0+ adds GPU parallelization via MJX (its JAX backend)—making it well suited for algorithm prototyping and medium-scale benchmarks, though its pre-built sim-to-real asset ecosystem is smaller than Isaac Lab's.

### Isaac Lab (NVIDIA)

Built atop NVIDIA Isaac Sim, **Isaac Lab** runs thousands to tens of thousands of robot environments simultaneously on a single GPU alongside photorealistic rendering and sensor simulation. Setup requires an NVIDIA GPU and demands more installation effort, making it the primary choice for large-scale locomotion, dexterous manipulation, and end-to-end sim-to-real pipelines.

### PyBullet

**PyBullet** installs with a single `pip install` and provides an approachable open-source playground for beginners. Although slower and less physically accurate in tight contact than MuJoCo, it is ample for learning RL workflows and verifying early prototypes.

### Brax

Developed by Google on top of JAX, **Brax** leverages JIT compilation and automatic differentiation to achieve massive simulation throughput on GPUs and TPUs as well as differentiable-physics research. Its simplified contact model, however, limits fidelity on contact-rich manipulation tasks.

### Environment Comparison Table

The table below summarizes how the four engines trade off accuracy, speed, and engineering overhead:

| Simulator | Physical accuracy | Speed | GPU parallelization | Installation difficulty | Main use |
|-----------|-----------|------|-----------|-----------|---------|
| MuJoCo | High | Moderate | Possible via MJX | Easy | Algorithm benchmarks |
| Isaac Lab | High | Very fast | Thousands to tens of thousands | Hard | Large-scale robot learning |
| PyBullet | Moderate | Slow | No | Very easy | Introduction/education |
| Brax | Low | Very fast | Yes | Moderate | Fast iteration experiments |

For most newcomers, starting with MuJoCo and Gymnasium provides fast feedback without setup friction, before scaling up to Isaac Lab when GPU-parallel sim-to-real experiments require thousands of simultaneous worlds.


---

## 8.5 Sim-to-Real Transfer

Training inside a simulator solves the data bottleneck—until you deploy the policy onto physical hardware. **Sim-to-real transfer** bridges the gap between simulated rollouts and real-world deployment, where policies trained solely on idealized physics routinely fail on contact-rich hardware.

### Reality Gap

Even a policy hitting peak reward in simulation can stumble or topple immediately on a physical robot because four mismatches separate simulated models from reality:

- **Physical parameter differences**: Real friction coefficients, link masses, and inertia tensors rarely match CAD specifications exactly.
- **Sensor noise**: Physical sensors introduce measurement noise, communication latency, and calibration drift absent from clean simulation states.
- **Actuator modeling error**: Real motors and gearboxes exhibit torque saturation, backlash, and structural compliance that rigid-body joints ignore.
- **Contact model differences**: Simulator contact solvers approximate deformable impacts and stick-slip friction using numerical shortcuts.

### Domain Randomization

Rather than trying to make one simulator match reality to five decimal places, **domain randomization** trains a single policy across a wide distribution of randomized physics worlds—forcing it to succeed regardless of the exact friction, mass, or sensor delay it encounters. When OpenAI's Dactyl (2018) randomized hundreds of visual and physical parameters simultaneously and solved a Rubik's cube on a real anthropomorphic hand, it proved that brute-force randomization can cross the reality gap.

Typical parameters varied during training include:
- Friction coefficient sampled uniformly (e.g., 0.5 to 1.5)
- Object and link masses scaled between 0.8 and 1.2 times nominal values
- Joint damping scaled between 0.5 and 2.0 times nominal values
- Gaussian observation noise and external push disturbances
- Actuator strength scaled between 0.8 and 1.2 times nominal values
- Action or communication latency of 0 to 2 control steps

The pseudo-code below illustrates an Isaac Lab–style randomization configuration that perturbs dynamics at each episode reset while injecting sensor noise and disturbances on every step:

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

Wide randomization makes it likely that the real robot lies somewhere inside the training distribution, at a clear trade-off: hedging across many worlds reduces peak optimality on any single nominal parameter setting. Yet because exact real-world parameters fluctuate with wear and temperature anyway, a slightly conservative randomized policy often outperforms a finely tuned specialist on hardware.

### System Identification (Sys-ID)

**System identification (Sys-ID)** attacks the reality gap from the opposite direction: instead of bracing for wide parameter uncertainty, measure and fit the real robot's physical properties so simulation mirrors hardware closely. Common approaches include:

- **Direct measurement**: Weighing links and payloads or measuring surface friction experimentally.
- **Parameter optimization**: Fitting mass, inertia, friction, and actuator models so simulated trajectories match recorded hardware rollouts.
- **Online adaptation**: Estimating shifting payload or terrain parameters continuously during deployment.

In practice, Sys-ID and domain randomization complement each other: Sys-ID pins down realistic nominal parameters—including learned actuator models—while domain randomization covers the residual uncertainty that identification leaves behind.

### Teacher-Student Structure

Simulation grants access to **privileged information**—exact terrain elevation maps, true friction coefficients, and ground-truth object poses—that real onboard sensors cannot observe directly. Rather than discarding that signal or forcing a vision policy to learn locomotion from scratch, **teacher-student distillation** splits training into two phases:

1. **Teacher training**: Train an initial RL policy inside simulation with full privileged state access; abundant ground truth lets the teacher discover agile recovery and foothold strategies quickly.
2. **Student training**: Distill that teacher via supervised imitation into a deployable student policy that receives only onboard real-robot observations (IMU, joint encoders, cameras, or proprioception history).

This two-stage pipeline drove breakthrough locomotion on ETH Zurich's ANYmal quadruped: the teacher saw exact terrain geometry around each foot, while the deployed student learned to infer terrain compliance and steps purely from a short history of joint proprioception.

### Case Studies

Two landmark deployments demonstrate how scale, randomization, and privilege distillation turn simulated RL into physical capability:

- **ANYmal Locomotion (ETH Zurich / Robotic Systems Lab)**: Combining PPO, large-scale domain randomization, teacher-student distillation, and proprioception history across billions of simulated steps enabled zero-shot transfer onto physical quadruped hardware traversing stairs, gravel, and steep slopes.
- **Dexterous Hand Manipulation (OpenAI, NVIDIA, etc.)**: OpenAI's 2019 Shadow Hand system solved a Rubik's cube in hand by randomizing hundreds of physical parameters across roughly 13,000 years of simulated experience; although real-world success rates remained considerably lower than in simulation, it demonstrated that contact-rich sim-to-real manipulation is viable without explicit analytical contact plans.

---

## 8.6 Imitation Learning

Reinforcement learning eliminates manual trajectory coding, yet still asks the engineer to craft dense reward functions and survive millions of exploratory trials. When a human operator can already demonstrate the task smoothly, **imitation learning** takes a much more direct route—learning a policy straight from expert demonstrations.

### Behavioral Cloning (BC)

The simplest form of imitation learning, **behavioral cloning (BC)**, treats control as supervised regression: record `(observation, action)` pairs from human demonstrations and train a neural network to predict the expert's action at each observation, as shown in the PyTorch example below.

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

Despite its simplicity, plain BC suffers from a structural **compounding error problem**: one small prediction error nudges the robot slightly off the demonstrated trajectory into states absent from the training dataset, where the network produces even worse actions until the task fails completely. Over a horizon $T$, worst-case error can grow quadratically as $O(\epsilon T^2)$ (Ross & Bagnell, 2010).

### DAgger (Dataset Aggregation)

**DAgger** breaks this covariate shift by putting the *learned* policy in the driver's seat during data collection so the expert teaches it how to recover from its own mistakes:

1. Train an initial BC policy on human demonstrations.
2. Execute the learned policy on the robot to visit states induced by its own errors.
3. Ask the expert to label the corrective action at each visited state.
4. Aggregate the new labeled data into the dataset and retrain.
5. Repeat steps 2–4 iteratively.

By collecting corrective labels on states the policy actually encounters, DAgger achieves a no-regret performance bound under its online-learning and oracle-query assumptions. Its practical drawback is human labor: an expert must remain on call to annotate rollout corrections round after round.

### ACT (Action Chunking with Transformers)

Developed in Stanford's ALOHA bimanual manipulation project, **ACT** tackles compounding error and multimodal human behavior without interactive expert relabeling by pairing a Transformer encoder-decoder (consuming joint states and camera feeds) with two key mechanisms:

1. **Action chunking**: Instead of predicting a single next-step action, the policy predicts an entire sequence of k future steps at once—cutting effective decision horizon, preserving smooth temporal correlation, and curbing drift.
2. **CVAE (Conditional Variational Autoencoder)**: When human demonstrators reach around an obstacle either to the left or to the right, naive MSE regression averages both valid modes into a single command that crashes straight into the obstacle. A CVAE latent variable lets ACT capture multimodal action distributions cleanly.

### Diffusion Policy

Proposed by Chi et al. (2023), **Diffusion Policy** brings iterative denoising diffusion models from image generation into robot action-sequence synthesis. Starting from Gaussian noise over a future action horizon, a conditional network iteratively denoises the trajectory conditioned on recent observations:

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

Why does diffusion work so well for manipulation? Ordinary BC trained with an MSE regression head effectively assumes a unimodal Gaussian output distribution, whereas diffusion captures arbitrarily complex multimodal action distributions naturally while remaining stable to train. (Note that "BC" names the supervised training scheme rather than a distribution family—paired with an energy-based or mixture-density head, BC can also represent multiple modes.) Since 2023, Diffusion Policy and ACT have become the primary baselines for visuomotor manipulation, both supported in open frameworks such as HuggingFace's LeRobot.

### Data Collection Methods

Because imitation policies cannot easily surpass the consistency of their training demonstrations, hardware interfaces for gathering clean human trajectories matter enormously:

- **Teleoperation**: An operator steers the robot remotely; leader-follower rigs such as ALOHA capture bimanual manipulation trajectories accurately at low cost.
- **VR controller**: Tracks a handheld controller in 3D space to command end-effector pose—intuitive for reaching, though lacking physical force feedback in tight contact tasks.
- **Kinesthetic teaching**: The human physically guides a compliant arm through the motion; natural for tabletop skills, yet awkward on heavy industrial arms.
- **Space mouse**: A compact 6-DoF desktop joystick operable with one hand, well suited for precise alignment.

Dataset size varies by task complexity, though Chi et al. (2023) demonstrated strong manipulation performance with roughly 100–200 clean demonstrations—always balancing extra coverage against collection effort.

---

## 8.7 Advanced: Foundation Models for Robot Control

Single-task imitation policies learn each skill from scratch on a single robot morphology. Inspired by large language models (LLMs) and vision-language models (VLMs), robotics has moved toward **vision-language-action (VLA)** foundation models: pretrained generalist policies trained across massive, multi-task robot datasets so web-scale visual and semantic priors transfer directly to physical control, allowing rapid adaptation to novel objects, instructions, and robot platforms.

### RT-1, RT-2 (Google DeepMind)

**RT-1 (2022)** scaled Transformer imitation learning to 130,000 real-robot demonstrations collected over roughly 17 months, mapping camera images and language instructions directly into tokenized actions across more than 700 tasks with a single model. **RT-2 (2023)** took the next conceptual leap into VLA design: rather than training a visuomotor network from scratch, it fine-tuned web-pretrained VLMs (PaLM-E and PaLI-X) to emit discretized robot action tokens alongside text—demonstrating that internet-scale semantic knowledge transfers directly to zero-shot reasoning over unseen objects.

### Octo

Developed by UC Berkeley and collaborators, **Octo** is an open-source generalist policy trained on the multi-institution Open X-Embodiment dataset. By pairing a Transformer backbone with a diffusion-based action head, Octo supports flexible observation/action spaces and is built specifically for fast fine-tuning onto new robots and sensors.

### π0 (Physical Intelligence)

The [π0 technical report (2024)](https://arxiv.org/abs/2410.24164) pairs a pretrained VLM backbone with a continuous **flow-matching** action expert, enabling smooth high-frequency dexterous control across single-arm, dual-arm, and mobile-manipulator embodiments. Evaluated across zero-shot rollouts, language instruction following, and fine-tuning on multi-stage tasks such as laundry folding, table clearing, and box assembly, its reported results should be interpreted within the specific robots, task setups, and baselines tested in the report.

### OpenVLA

**OpenVLA** provides a fully open-source 7B-parameter VLA model fine-tuned from an open VLM backbone to output discrete action tokens, making generalist VLA weights, training recipes, and fine-tuning pipelines accessible for academic reproduction.

### Realistic Assessment

Despite rapid progress, robotic foundation models remain at an early stage compared to text or image foundation models:

- On narrow, high-precision tasks, task-specific classical planners or single-task imitation policies still frequently achieve higher reliability.
- Collecting diverse physical robot trajectories is orders of magnitude slower and costlier than scraping web text or images.
- Large neural policies offer no formal safety guarantees, making failure modes hard to bound before deployment.
- Multi-billion-parameter inference latency can exceed tight real-time control budgets without quantization, action chunking, or asynchronous control loops.

### Research Directions

Current research focuses on four bottlenecks limiting generalist policies:

- **Data scaling**: Pooling multi-lab corpora such as Open X-Embodiment to test whether robotic scaling laws mirror language and vision.
- **Cross-embodiment transfer**: Unifying heterogeneous kinematics, camera placements, and action spaces so policies transfer cleanly across different robot bodies.
- **Efficient fine-tuning**: Adapting large VLA backbones rapidly to local workspaces using parameter-efficient tuning such as LoRA.
- **Action representation**: Comparing discrete action tokenization, continuous flow matching, and diffusion action heads for precision and control frequency.

---

## 8.8 Advanced: Reward Design and Safe RL

In reinforcement learning, the reward function defines the entire optimization target—and on physical robots, poorly shaped rewards or unconstrained exploration can ruin training or damage hardware.

### Reward Shaping

**Sparse rewards** (such as `+1` upon task completion and `0` everywhere else) are unambiguous to write, yet force an agent in a large continuous state-action space to wander blindly until a lucky trajectory accidentally discovers the goal. **Dense rewards** supply continuous gradient feedback for intermediate progress—distance to the object, grasp acquisition, and distance to the placement target—as illustrated in the pick-and-place reward function below:

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

Alongside dense shaping, **curriculum learning** eases exploration on hard tasks by staging difficulty progressively—for instance, training a quadruped to walk on flat ground first before introducing uneven obstacles and stairs, allowing the agent to accumulate early successes even when rewards remain sparse.

### Reward Hacking

Optimizers exploit every loophole in an imperfect objective. **Reward hacking** occurs when an agent achieves high numerical reward through unintended shortcuts:

- A manipulator rewarded for moving an object to a target pushes or bats it across the table instead of picking it up.
- A legged robot rewarded solely for forward velocity flings itself forward and slides along the floor.
- Co-optimizing morphology and jumping height produces bizarrely elongated limbs that satisfy the metric without walking naturally.

Guardrails combine iterative reward tuning and explicit behavioral penalties with qualitative inspection of rollout videos—a manual sanity check that remains difficult to automate.

### Constrained RL

On physical hardware, trading off collision risk or joint torque violations inside a single scalar reward sum is dangerous. **Constrained RL** separates task performance from safety by maximizing expected return subject to an explicit discounted cost budget:

```
max_π E[ Σ γ^t r_t ]   subject to   E[ Σ γ^t c_t ] ≤ d
```

Here `c_t` measures safety violations (excess torque, self-collision, or workspace boundary breaches) and `d` caps the acceptable budget. Algorithms such as CPO (Constrained Policy Optimization), PCPO, and Lagrangian-relaxation methods enforce hardware-protection limits directly during optimization.

### Human-in-the-loop RL

When desired qualities resist clean formulas—such as "walk smoothly" or "hand over an object gently"—**human-in-the-loop RL** adapts RLHF (Reinforcement Learning from Human Feedback) from LLMs to robotics:

1. Present pairs of recorded robot rollouts and ask human evaluators which trajectory looks better.
2. Train a learned reward model to match those pairwise preferences.
3. Optimize an RL policy against the learned reward model.

Although preference learning captures subtle human intent without manual reward engineering, it demands substantial human annotation time and remains sensitive to inaccuracies in the learned reward model.

---

## 8.9 Further Reading

> **Sutton & Barto, "Reinforcement Learning: An Introduction" (2nd edition)**
> http://incompleteideas.net/book/the-book-2nd.html
> A standard textbook covering topics from MDPs to policy gradients, with a free PDF. Ch.1-6 and Ch.13 provide a focused starting path through the foundations.

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
> - [Hafner et al., "Mastering Diverse Domains through World Models" (DreamerV3, arXiv:2301.04104)](https://arxiv.org/abs/2301.04104) — Trains more than 150 tasks with a single fixed hyperparameter configuration. A representative recent result in world-model-based RL.
> - [Chi et al., "Universal Manipulation Interface" (UMI, RSS 2024, arXiv:2402.10329)](https://arxiv.org/abs/2402.10329) — Data collection with a handheld gripper, zero-shot deployment across diverse robots.
> - [Fu et al., "Mobile ALOHA" (CoRL 2024, arXiv:2401.02117)](https://arxiv.org/abs/2401.02117) — Mobile base + bimanual teleoperation. Co-training significantly improves success rate.

---

## Technical Timeline

```
1992 ── REINFORCE algorithm (Williams)
         An influential early Monte Carlo policy-gradient method whose score-function estimator has high variance.

2013 ── DQN (Mnih et al., Atari)
         The beginning of Deep RL. A replay buffer mitigated the correlated-sample problem.
         The fixed target network was added in the 2015 Nature version.

2015 ── TRPO (Schulman et al.)
         Stable policy updates with a trust region constraint. Strong theory, complex implementation.

2017 ── PPO (Schulman et al.)
         A practical alternative to TRPO. Simple to implement with clipping. The default baseline for robot RL experiments.

2018 ── SAC (Haarnoja et al.)
         Entropy regularization + off-policy. Sample efficient in continuous spaces.

2019 ── ANYmal: sim-to-real locomotion (ETH Zurich)
         Succeeded at quadruped sim-to-real by modeling the drivetrain dynamics with an actuator network trained on real robot data.

2020 ── Widespread practical adoption of DAgger
         Imitation learning proved practical. Applied across diverse robot platforms.

2022 ── RT-1 (Google)
         Large-scale robot data + Transformer. A multi-task generalist policy.

2023 ── ACT/ALOHA (Stanford), Diffusion Policy (Columbia / TRI / MIT)
         A new standard for imitation learning. Performance gains via action chunking and diffusion.

2023 ── RT-2 (Google)
         VLM used directly for action generation. Transfer of web knowledge to robotics.

2024 ── Octo, OpenVLA, pi0
         Emergence of open-source generalist policy models. Start of cross-embodiment learning.

2025 ── Spread of cross-embodiment learning research
         Policy transfer across different robots. Data scaling laws under verification.
```

---

Whether a robot executes a classical motion plan (Ch.7) or a learned visuomotor policy (Ch.8), its actions are only as trustworthy as the observations feeding them. The next several chapters turn to perception—starting in Ch.9 with the computer vision fundamentals that convert raw camera pixels into structured geometric measurements.
