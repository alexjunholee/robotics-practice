# Ch.8 — 로봇 러닝 (Robot Learning)


로봇 러닝은 로봇이 명시적 프로그래밍 대신 데이터와 경험으로부터 행동을 학습하는 분야다. 강화학습 기초부터 sim-to-real transfer, 모방학습, 최근 foundation model 기반 접근까지 다룬다.

---

## 8.1 왜 로봇 러닝을 배우는가

**전통적 방법이 잘 되는 영역**

PID 제어, MPC, RRT 같은 전통적 제어/플래닝은 dynamics 모델이 정확하고, 환경이 정형화되어 있을 때 매우 잘 동작한다. 산업용 로봇 팔이 정해진 위치의 부품을 집어 조립하는 작업이 대표적이다. 모델이 정확하면 최적 제어 이론이 수학적으로 보장하는 성능을 얻을 수 있다. 학습 기반 방법이 이걸 이기기는 쉽지 않다.

**전통적 방법이 힘든 영역**

문제는 현실 세계가 깔끔하지 않다는 점이다.

- **모델링이 어려운 dynamics**: 천, 로프, 유체 같은 deformable object의 물리 모델을 정확히 세우는 건 현실적으로 불가능에 가깝다.
- **복잡한 접촉(contact)**: 물체를 손으로 돌리거나 끼워 맞추는 작업은 접촉 모드가 수시로 바뀐다. 접촉 역학을 정확히 모델링하는 것은 아직 열린 문제다.
- **비정형 환경**: 가정집 부엌, 재난 현장 등 미리 모델링할 수 없는 환경에서의 동작. 어떤 물체가 어디에 있을지 알 수 없다.

이런 상황에서 학습 기반 접근은 데이터에서 직접 입력-출력 관계를 근사하므로, 명시적 모델 없이도 동작할 수 있다.

**하지만 만능은 아니다**

학습 기반 방법의 한계를 명확히 알아야 한다.

- **데이터 효율(sample efficiency)**: 강화학습은 수백만 스텝의 상호작용이 필요한 경우가 많다. 실제 로봇에서 이 데이터를 모으는 건 시간과 비용 면에서 비현실적이다.
- **안전성(safety)**: 학습 중 로봇이 자기 자신이나 주변 환경을 파손할 수 있다. 탐색(exploration)이 본질적으로 위험하다.
- **일반화(generalization)**: 학습한 조건과 조금만 달라져도 성능이 급락하는 경우가 흔하다.

전통적 방법으로 풀 수 있으면 전통적 방법을 쓰는 게 낫다. 학습은 전통적 방법이 한계에 부딪히는 문제에 적용하는 도구다. 둘을 적절히 조합하는 것이 실무에서 가장 현실적이다.


---

## 8.2 강화학습 기초 (RL Basics)

### MDP (Markov Decision Process)

강화학습의 수학적 프레임워크는 MDP다. 구성 요소는 다음과 같다.

- **State (s)**: 환경의 현재 상태. 로봇의 관절 각도, 속도, 물체 위치 등.
- **Action (a)**: 에이전트가 취하는 행동. 관절 토크, 목표 관절 각도 등.
- **Reward (r)**: 행동의 결과로 받는 스칼라 보상 신호. r = R(s, a).
- **Transition (T)**: 상태 전이 확률. T(s'|s, a). 현재 상태에서 행동을 취했을 때 다음 상태의 분포.
- **Discount factor (γ)**: 미래 보상의 할인율. 0 < γ ≤ 1. 로봇 RL에서는 γ = 0.99가 흔한 초기값이다.

목표는 cumulative discounted reward를 최대화하는 policy π(a|s)를 찾는 것이다.

```
J(π) = E[ Σ_{t=0}^{∞} γ^t · r_t ]
```

Markov property는 "다음 상태는 현재 상태와 행동에만 의존한다"는 가정이다. 이전 히스토리 전체를 볼 필요가 없다는 뜻인데, 실제 로봇에서는 이 가정이 깨지는 경우도 있다 (부분 관측, 즉 POMDP 상황). 이 경우 observation history를 state로 사용하거나 recurrent policy를 쓴다.

### Policy Gradient 직관

Policy gradient의 핵심 아이디어는 단순하다.

1. 현재 policy로 여러 trajectory를 수집한다.
2. 높은 return을 받은 trajectory에서의 action 확률을 올린다.
3. 낮은 return을 받은 trajectory에서의 action 확률을 내린다.

수식으로 쓰면:

```
∇J(θ) = E[ Σ_t ∇log π_θ(a_t|s_t) · A_t ]
```

A_t는 advantage function으로, 해당 action이 평균 대비 얼마나 좋았는지를 나타낸다. 이 gradient를 따라 파라미터 θ를 업데이트한다.

직관적으로, `log π(a|s)`의 gradient는 action a의 확률을 올리는 방향이고, 여기에 advantage를 곱해서 좋은 action은 더 자주, 나쁜 action은 덜 자주 선택하도록 만든다.

### Value Function, Q-function

- **Value function V^π(s)**: state s에서 policy π를 따랐을 때 기대되는 cumulative reward.
- **Q-function Q^π(s, a)**: state s에서 action a를 취하고 이후 π를 따랐을 때의 기대 cumulative reward.
- **Advantage A^π(s, a) = Q^π(s, a) - V^π(s)**: action a가 평균 대비 얼마나 좋은지.

Value function을 따로 학습해두면 variance를 줄일 수 있다. 대부분의 현대 RL 알고리즘은 policy network와 value network를 함께 학습하는 actor-critic 구조를 사용한다.

### On-policy vs Off-policy

- **On-policy**: 현재 policy가 수집한 데이터로만 학습한다. 데이터를 한번 쓰고 버린다. PPO가 대표적이다. 안정적이지만 sample efficiency가 낮다.
- **Off-policy**: 과거 policy가 수집한 데이터도 재사용한다 (replay buffer). SAC, TD3가 대표적이다. sample efficient하지만 학습이 불안정할 수 있다.

로봇에서는 데이터 수집 비용이 크기 때문에 off-policy 방법의 sample efficiency가 매력적이다. 하지만 시뮬레이션에서 대규모 병렬 환경을 돌릴 수 있다면 on-policy(PPO)도 충분히 경쟁력이 있다.


---

## 8.3 주요 RL 알고리즘

### PPO (Proximal Policy Optimization)

PPO는 Schulman et al. (2017)이 제안한 on-policy 알고리즘이다. 핵심 아이디어는 policy update의 크기를 제한하는 것이다. 이전 policy 대비 너무 크게 바뀌면 clipping으로 잘라낸다.

```
L_CLIP(θ) = E[ min( r_t(θ) · A_t, clip(r_t(θ), 1-ε, 1+ε) · A_t ) ]
```

여기서 r_t(θ) = π_θ(a_t|s_t) / π_θ_old(a_t|s_t)는 probability ratio, ε의 원 논문 기본값은 0.2다.

PPO가 인기 있는 이유는 구현이 비교적 간단하고, 하이퍼파라미터에 둔감하며, 안정적으로 학습된다는 점이다. NVIDIA Isaac Lab 등 대규모 병렬 시뮬레이션과 결합하면 수천 개 환경에서 동시에 데이터를 수집할 수 있어서 sample efficiency 문제를 물량으로 해결할 수 있다.

### SAC (Soft Actor-Critic)

SAC는 off-policy 알고리즘으로, entropy regularization을 추가한 것이 특징이다. 보상을 최대화하면서 동시에 policy의 entropy를 최대화한다. 즉, 가능한 한 다양한 action을 시도하도록 유도한다.

```
J(π) = E[ Σ_t γ^t ( r_t + α · H(π(·|s_t)) ) ]
```

α는 temperature parameter로, entropy와 reward 사이의 균형을 조절한다. 자동으로 α를 조절하는 방법도 있다.

연속 action space에서 sample efficient하다. Replay buffer를 써서 수집한 데이터를 여러 번 재사용할 수 있기 때문이다. 실제 로봇에서 데이터를 직접 수집할 때 off-policy인 SAC가 on-policy PPO보다 데이터 효율 면에서 유리하다.

### TD3 (Twin Delayed DDPG)

TD3는 DDPG의 개선 버전으로, SAC와 비슷한 off-policy 알고리즘이다. 핵심 개선점 세 가지:

1. **Twin Q-networks**: Q-function 두 개를 학습하고 작은 값을 사용하여 overestimation bias를 줄인다.
2. **Delayed policy update**: critic을 여러 번 업데이트한 후에 policy를 한번 업데이트한다.
3. **Target policy smoothing**: target action에 노이즈를 추가한다.

SAC와 성능이 비슷하지만, entropy tuning이 필요 없어서 하이퍼파라미터가 약간 적다. 다만 탐색(exploration)이 SAC보다 약할 수 있다.

### 알고리즘 선택 가이드

| 상황 | 추천 알고리즘 | 이유 |
|------|-------------|------|
| 시뮬레이션, GPU 병렬화 가능 | PPO | 병렬 환경으로 sample efficiency 보상 가능 |
| 실제 로봇, 데이터 적음 | SAC | off-policy, sample efficient |
| 연속 action space, 안정성 중시 | SAC 또는 TD3 | 둘 다 연속 공간에 강함 |
| 이산 action space | PPO 또는 DQN | SAC는 연속 공간 전용 |
| 처음 시작하는 프로젝트 | PPO | 튜닝이 쉽고, 디버깅이 용이 |

### Stable-Baselines3 코드 예시

PPO로 MuJoCo Ant 환경을 학습하는 기본 코드다.

```python
import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.evaluation import evaluate_policy

# 병렬 환경 생성 (8개)
vec_env = make_vec_env("Ant-v4", n_envs=8)

# PPO 에이전트 생성
model = PPO(
    "MlpPolicy",
    vec_env,
    learning_rate=3e-4,
    n_steps=2048,        # 한 번의 rollout에서 수집할 스텝 수
    batch_size=64,
    n_epochs=10,         # 수집한 데이터로 몇 epoch 학습할지
    gamma=0.99,
    gae_lambda=0.95,     # GAE (Generalized Advantage Estimation)
    clip_range=0.2,
    verbose=1,
    tensorboard_log="./ppo_ant_tb/",
)

# 학습 (총 2M 스텝)
model.learn(total_timesteps=2_000_000)

# 평가
eval_env = gym.make("Ant-v4")
mean_reward, std_reward = evaluate_policy(model, eval_env, n_eval_episodes=20)
print(f"Mean reward: {mean_reward:.1f} +/- {std_reward:.1f}")

# 모델 저장/로드
model.save("ppo_ant")
loaded_model = PPO.load("ppo_ant")
```

SAC 예시도 구조는 비슷하다.

```python
from stable_baselines3 import SAC

model = SAC(
    "MlpPolicy",
    "Ant-v4",
    learning_rate=3e-4,
    buffer_size=1_000_000,    # replay buffer 크기
    learning_starts=10_000,   # 이 스텝 이후부터 학습 시작
    batch_size=256,
    tau=0.005,                # target network soft update rate
    gamma=0.99,
    verbose=1,
)

model.learn(total_timesteps=1_000_000)
```

> Stable-Baselines3는 빠르게 프로토타이핑하기에 좋다. 알고리즘 내부를 이해하고 싶으면 CleanRL을 권장한다. 모든 알고리즘이 단일 파일로 구현되어 있어서 코드를 따라 읽기에 좋다.


---

## 8.4 시뮬레이션 환경

로봇 RL에서 시뮬레이션은 사실상 필수다. 실제 로봇에서 수백만 스텝의 데이터를 수집하는 건 비현실적이기 때문이다. 주요 시뮬레이터를 정리한다.

### MuJoCo (Multi-Joint dynamics with Contact)

DeepMind가 인수한 후 2022년에 오픈소스로 공개했다. 접촉 시뮬레이션 품질과 안정적인 수치 적분 덕분에 RL 연구의 표준 벤치마크 환경으로 자리잡았다. 기본 엔진은 CPU 기반이고, MuJoCo 3.0+에서 MJX (JAX backend)로 GPU 병렬화가 가능하지만 Isaac Lab 대비 생태계가 작다. 알고리즘 벤치마크와 소규모 실험에 적합하다.

### Isaac Lab (NVIDIA)

NVIDIA Isaac Sim 위에 구축된 로봇 학습 프레임워크다. GPU 병렬 시뮬레이션으로 수천~수만 개 환경을 동시에 실행할 수 있고, 사실적 렌더링과 sensor 시뮬레이션을 지원한다. NVIDIA GPU가 필수이고 설치·설정이 복잡하다. 대규모 locomotion 학습과 sim-to-real 파이프라인에 쓴다.

### PyBullet

입문용으로 적합한 오픈소스 물리 엔진이다. `pip install` 한 줄로 설치된다. 물리 정확도와 속도는 MuJoCo보다 낮지만, 처음 RL 코드를 돌려보거나 빠르게 아이디어를 검증할 때는 충분하다.

### Brax

Google에서 개발한 JAX 기반 물리 엔진이다. JAX의 JIT 컴파일과 자동 미분 덕분에 GPU/TPU에서 초고속으로 실행되고, differentiable physics 연구에 쓸 수 있다. 다만 물리 정확도가 제한적이고 복잡한 접촉 시나리오에 약하다.

### 환경 비교표

| 시뮬레이터 | 물리 정확도 | 속도 | GPU 병렬화 | 설치 난이도 | 주 용도 |
|-----------|-----------|------|-----------|-----------|---------|
| MuJoCo | 높음 | 보통 | MJX로 가능 | 쉬움 | 알고리즘 벤치마크 |
| Isaac Lab | 높음 | 매우 빠름 | 수천~만 | 어려움 | 대규모 로봇 학습 |
| PyBullet | 보통 | 느림 | 불가 | 매우 쉬움 | 입문/교육 |
| Brax | 낮음 | 매우 빠름 | 가능 | 보통 | 빠른 반복 실험 |

시작하는 단계라면 MuJoCo + Gymnasium 조합을 권장한다. 대규모 실험이 필요해지면 Isaac Lab으로 넘어간다.


---

## 8.5 Sim-to-Real Transfer

시뮬레이션에서 학습한 policy를 실제 로봇에 적용하는 것을 sim-to-real transfer라 한다. 이론적으로는 시뮬레이션에서 충분히 학습하고 실제 로봇에 배포하면 끝이지만, 현실은 그렇지 않다.

### Reality Gap

시뮬레이션과 현실 사이에는 차이(gap)가 존재한다.

- **물리 파라미터 차이**: 마찰 계수, 질량, 관성 모멘트 등이 시뮬레이션과 다르다.
- **센서 노이즈**: 실제 센서는 노이즈, 지연, 드리프트가 있다.
- **액추에이터 모델링 오차**: 모터의 비선형성, 기어 백래시, 컴플라이언스 등.
- **접촉 모델 차이**: 시뮬레이션의 접촉 모델은 현실의 근사에 불과하다.

시뮬레이션에서 reward 10,000을 찍어도 실제 로봇에서 쓰러지는 건 흔한 일이다.

### Domain Randomization

아이디어는 시뮬레이션의 물리 파라미터를 랜덤하게 변화시켜서, policy가 특정 파라미터에 의존하지 않고 robust하게 학습되도록 하는 것이다. OpenAI의 Dactyl(2019)이 수백 개 물리 파라미터를 동시에 랜덤화해 sim-to-real을 성공시키면서 이 접근의 가능성을 보여줬다.

랜덤화하는 대표적인 파라미터들:

- 마찰 계수: 0.5 ~ 1.5 사이에서 uniform 샘플링
- 물체 질량: 기본값의 0.8 ~ 1.2배
- 관절 damping: 기본값의 0.5 ~ 2.0배
- 센서 노이즈: Gaussian noise 추가
- 액추에이터 강도(strength): 기본값의 0.8 ~ 1.2배
- 통신 지연: 0 ~ 2 스텝 랜덤 지연

```python
# Isaac Lab 스타일의 domain randomization 설정 예시 (pseudo-code)
class RandomizationConfig:
    # 에피소드 시작마다 랜덤화
    friction_range = (0.5, 1.5)
    mass_scale_range = (0.8, 1.2)
    joint_damping_scale_range = (0.5, 2.0)

    # 매 스텝마다 적용
    obs_noise_std = 0.05           # observation에 Gaussian noise
    action_delay_steps = (0, 2)    # action 적용 지연
    push_force_range = (-5.0, 5.0) # 외부 교란 (N)

def randomize_env(env, config):
    """에피소드 시작 시 호출."""
    import numpy as np
    friction = np.random.uniform(*config.friction_range)
    mass_scale = np.random.uniform(*config.mass_scale_range)
    damping_scale = np.random.uniform(*config.joint_damping_scale_range)
    env.set_friction(friction)
    env.scale_mass(mass_scale)
    env.scale_joint_damping(damping_scale)

def add_obs_noise(obs, config):
    """매 스텝 observation에 노이즈 추가."""
    import numpy as np
    noise = np.random.normal(0, config.obs_noise_std, size=obs.shape)
    return obs + noise
```

충분히 넓은 범위로 랜덤화하면, 현실은 그 범위 안에 포함될 가능성이 높다. 대신 policy 성능의 상한은 내려간다. 특정 파라미터에 최적화한 policy보다 낮을 수밖에 없다.

### System Identification (Sys-ID)

Domain randomization과 반대 방향의 접근이다. 실제 로봇의 물리 파라미터를 가능한 한 정확하게 측정/추정해서 시뮬레이션에 반영한다.

방법:
- 직접 측정: 전자저울로 질량 측정, 마찰 계수 실험 측정
- 파라미터 최적화: 실제 로봇의 trajectory와 시뮬레이션 trajectory의 차이를 최소화하는 파라미터를 찾음
- 온라인 적응: 실제 운용 중에 파라미터를 지속적으로 추정/업데이트

Sys-ID는 domain randomization과 같이 쓰는 경우가 많다. Sys-ID로 대략적인 파라미터를 잡고, 나머지 불확실성은 domain randomization으로 커버하는 방식이다.

### Teacher-Student 구조

시뮬레이션에서는 접근할 수 있지만 현실에서는 접근할 수 없는 정보(privileged information)를 활용하는 방법이다. 2단계로 학습한다.

1. **Teacher 학습**: 시뮬레이션에서 privileged information (정확한 지형 높이, 정확한 마찰 계수, 물체의 정확한 위치 등)을 state에 포함하여 policy를 학습한다. 정보가 많으므로 학습이 쉽다.
2. **Student 학습**: 실제 로봇에서 사용 가능한 observation (IMU, 관절 encoder, 카메라 등)만으로 teacher의 행동을 모방하도록 학습한다.

이 방식은 ANYmal 사족보행 로봇의 locomotion 연구에서 큰 성공을 거뒀다. Teacher는 정확한 지형 높이맵을 알지만, student는 proprioception history만으로 teacher와 비슷한 행동을 학습한다.

### 실제 사례

**ANYmal 보행 (ETH Zurich / Robotic Systems Lab)**
- PPO + domain randomization + teacher-student로 사족보행 학습
- 시뮬레이션에서 수십억 스텝 학습 후 실제 로봇에 zero-shot transfer
- 계단, 자갈, 경사면 등 다양한 지형에서 robust하게 보행
- 핵심: 대규모 domain randomization + privileged learning + proprioception history

**Dexterous Hand Manipulation (OpenAI, NVIDIA 등)**
- Rubik's cube를 Shadow Hand로 풀기 (OpenAI, 2019)
- 대규모 domain randomization이 핵심: 수백 개의 물리 파라미터를 동시에 랜덤화
- 시뮬레이션에서 약 13,000년 분량의 경험으로 학습
- 현실에서의 성공률은 시뮬레이션 대비 상당히 낮았지만, 학습 기반 접근의 가능성을 보여줌


---

## 8.6 모방 학습 (Imitation Learning)

강화학습은 reward 함수를 설계해야 하고, 학습에 많은 데이터가 필요하다. 반면 모방 학습은 전문가(사람)의 시연 데이터로부터 직접 policy를 학습한다. "보고 배우기"에 해당한다.

### Behavioral Cloning (BC)

가장 단순한 모방 학습이다. 전문가의 (observation, action) 쌍을 수집하고, 지도학습(supervised learning)으로 policy를 학습한다.

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

# 전문가 데이터 로드 (NumPy -> Tensor)
# expert_obs: (N, obs_dim), expert_act: (N, act_dim)
dataset = TensorDataset(
    torch.FloatTensor(expert_obs),
    torch.FloatTensor(expert_act),
)
loader = DataLoader(dataset, batch_size=256, shuffle=True)

policy = BCPolicy(obs_dim=48, act_dim=7)
optimizer = torch.optim.Adam(policy.parameters(), lr=1e-3)
loss_fn = nn.MSELoss()

# 학습
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

**Compounding error 문제**: BC의 구조적 한계다. 학습된 policy가 조금이라도 전문가 trajectory에서 벗어나면, 학습 데이터에 없는 상태에 도달하게 된다. 거기서의 행동은 예측 불가능하고, 더 벗어나게 되고, 에러가 누적된다. 시간에 따라 에러가 기하급수적으로 커질 수 있다.

### DAgger (Dataset Aggregation)

DAgger는 compounding error를 해결하기 위한 방법이다.

1. 초기 전문가 데이터로 BC policy를 학습한다.
2. 학습된 policy를 실행하여 새로운 trajectory를 수집한다.
3. 이 trajectory의 각 state에서 전문가가 어떤 action을 취할지 레이블링한다.
4. 새 데이터를 기존 데이터에 추가하고 다시 학습한다.
5. 2-4를 반복한다.

핵심은 "policy가 실제로 방문하는 state"에서의 전문가 action을 학습 데이터에 포함시키는 것이다. 이론적으로 DAgger는 no-regret guarantee를 가진다.

단점은 전문가가 반복적으로 레이블링해야 한다는 점이다. 사람이 일일이 correction을 해줘야 하므로 노동 집약적이다.

### ACT (Action Chunking with Transformers)

Stanford의 ALOHA 프로젝트에서 제안한 방법이다. 핵심 아이디어 두 가지:

1. **Action chunking**: 한 번에 하나의 action을 예측하는 대신, 미래 k 스텝의 action sequence를 한 번에 예측한다. 이렇게 하면 temporal correlation을 잡을 수 있고, compounding error를 줄인다.
2. **CVAE (Conditional Variational Autoencoder)**: action의 다봉(multimodal) 분포를 모델링한다. 같은 상황에서도 여러 유효한 행동이 있을 수 있는데, 단순 MSE loss로는 이걸 평균내버려서 어중간한 action이 나온다.

구조는 Transformer encoder-decoder를 사용하며, 입력으로 joint position과 카메라 이미지를 받는다.

### Diffusion Policy

CMU의 Chi et al. (2023)이 제안한 방법으로, diffusion model을 action 생성에 적용한다.

기존 BC가 unimodal Gaussian으로 action을 모델링하는 반면, diffusion policy는 denoising 과정을 통해 임의의 복잡한 action 분포를 표현할 수 있다. 특히 다봉 분포를 잘 다룬다.

```python
# Diffusion Policy의 action 생성 과정 (pseudo-code)
# 1. 순수 noise에서 시작
action = torch.randn(batch_size, horizon, action_dim)

# 2. K번의 denoising step
for k in reversed(range(K)):
    # 현재 observation 조건 하에 noise 예측
    predicted_noise = noise_pred_net(action, k, obs_encoding)
    # noise 제거 (DDPM 또는 DDIM scheduler 사용)
    action = scheduler.step(predicted_noise, k, action)

# 3. 최종 action sequence 출력
```

Diffusion policy와 ACT는 2023년 이후 manipulation 모방학습의 주요 베이스라인으로 자리잡았다. LeRobot(HuggingFace) 등 공개 프레임워크에도 둘 다 구현이 포함되어 있다.

### 데이터 수집 방법

모방 학습의 성능은 데이터 품질에 결정적으로 의존한다. 주요 데이터 수집 방법:

- **Teleoperation**: 사람이 원격으로 로봇을 조종한다. ALOHA는 leader-follower 구조를 사용했고, 비교적 저렴하게 양팔 조작 데이터를 수집할 수 있다.
- **VR controller**: VR 컨트롤러로 end-effector 위치/자세를 지정한다. 직관적이지만 contact-rich 작업에서는 힘 피드백이 부족할 수 있다.
- **Kinesthetic teaching**: 로봇 팔을 직접 잡고 움직인다. 가장 직관적이지만, 로봇 크기가 크거나 무거우면 어렵다.
- **Space mouse**: 6-DoF 입력 장치. 한 손으로 조작 가능. 정밀 작업에 유용하다.

데이터 양은 태스크와 방법에 따라 다르다. Chi et al. (2023) Diffusion Policy 논문에서는 약 100~200개의 시연으로 유의미한 성능을 보였다. 더 많을수록 좋지만, 수집 비용과의 trade-off가 있다.


---

## 8.7 심화: Foundation Models for Robot Control

*연구자가 되고 싶다면 여기서부터 읽어라.*

LLM과 VLM의 성공에 영감을 받아, 로봇 분야에서도 대규모 사전학습 모델(foundation model)을 만들려는 시도가 이어지고 있다. 핵심 아이디어는 대량의 로봇 데이터로 범용 정책(generalist policy)을 학습해 두고, 새로운 로봇이나 태스크에 빠르게 적응시키는 것이다.

### RT-1, RT-2 (Google DeepMind)

**RT-1 (2022)**: 13만 개의 로봇 시연 데이터(약 17개월 수집)로 학습한 Transformer 기반 policy. 이미지와 자연어 명령을 입력으로 받아 action을 출력한다. 700개 이상의 태스크를 하나의 모델로 수행.

**RT-2 (2023)**: VLM (Vision-Language Model)을 직접 action 출력으로 fine-tuning한 것. PaLM-E나 PaLI-X를 base model로 사용. 웹 스케일 사전학습 지식이 로봇 제어에도 전이된다는 것을 보여줬다. 학습 데이터에 없던 물체에 대해서도 어느 정도 일반화됐다.

### Octo

UC Berkeley 등에서 개발한 오픈소스 범용 로봇 정책이다. Open X-Embodiment 데이터셋(다양한 로봇, 다양한 기관에서 수집한 데이터)으로 학습했다. Diffusion 기반 action head를 사용하며, 새로운 로봇에 fine-tuning할 수 있도록 설계했다.

### pi0 (Physical Intelligence)

2024년에 공개된 diffusion 기반 범용 로봇 정책이다. VLM을 backbone으로 사용하고, flow matching으로 action을 생성한다. 다양한 manipulation 태스크에서 state-of-the-art 성능을 보였으며, 빨래 접기 같은 복잡한 장시간(long-horizon) 태스크에서도 동작했다.

### OpenVLA

오픈소스 VLA (Vision-Language-Action) 모델이다. 7B 파라미터의 VLM을 fine-tuning하여 action token을 출력하도록 학습했다. 누구나 접근 가능한 오픈소스라는 점이 핵심 기여다.

### 현실적 평가

Foundation model for robotics는 아직 초기 단계다. 솔직히 말하면:

- 특정 태스크에서는 해당 태스크에 특화된 전통적 방법이나 task-specific 학습이 더 나은 경우가 많다.
- 대규모 로봇 데이터 수집 비용이 매우 크다. 인터넷 텍스트/이미지 데이터와는 규모가 다르다.
- 안전성 보장이 없다. Foundation model의 행동을 예측하기 어렵다.
- 추론 속도(inference latency)가 실시간 제어에 충분하지 않을 수 있다.

### 연구 방향

- **Data scaling**: Open X-Embodiment처럼 여러 기관의 데이터를 합치는 시도. 데이터가 많을수록 일반화가 좋아지는지 검증 중.
- **Cross-embodiment transfer**: 한 로봇에서 학습한 정책을 다른 로봇에 전이하는 연구. 서로 다른 action space를 어떻게 통일할 것인가가 핵심 문제.
- **Efficient fine-tuning**: LoRA 같은 parameter-efficient fine-tuning으로 새 태스크에 빠르게 적응.
- **Action representation**: action을 어떻게 토큰화/표현할 것인가. 이산화, 연속 분포, diffusion 등 다양한 접근이 경쟁 중.


---

## 8.8 심화: Reward Design과 Safe RL

*연구자가 되고 싶다면 여기서부터 읽어라.*

RL의 성패는 reward 함수 설계에 달려 있다고 해도 과언이 아니다. 그리고 실제 로봇에 RL을 적용하려면 안전성 문제를 반드시 다뤄야 한다.

### Reward Shaping

**Sparse reward의 문제**: "목표에 도달하면 +1, 아니면 0" 같은 sparse reward는 정의하기 쉽지만, agent가 우연히 보상을 받기까지 무작위 탐색을 해야 한다. state-action 공간이 크면 사실상 학습이 안 된다.

**Dense reward**: 중간 과정에 대한 보상을 추가한다. 예를 들어 물체 잡기 태스크에서:

```python
def compute_reward(gripper_pos, object_pos, target_pos, is_grasped):
    # 1. 그리퍼를 물체에 가까이 가져가기
    dist_to_object = np.linalg.norm(gripper_pos - object_pos)
    reaching_reward = -1.0 * dist_to_object

    # 2. 물체를 잡았으면 보너스
    grasp_reward = 5.0 if is_grasped else 0.0

    # 3. 물체를 목표 위치에 가까이
    if is_grasped:
        dist_to_target = np.linalg.norm(object_pos - target_pos)
        place_reward = -1.0 * dist_to_target
    else:
        place_reward = 0.0

    # 4. 목표 도달 보너스
    success_reward = 10.0 if (is_grasped and
        np.linalg.norm(object_pos - target_pos) < 0.05) else 0.0

    return reaching_reward + grasp_reward + place_reward + success_reward
```

**Curriculum learning**: 쉬운 태스크에서 시작해 점차 어려운 태스크로 넘어가는 방법이다. 예를 들어 locomotion에서 처음에는 평지에서 걷기, 다음에 작은 장애물, 그 다음 계단 순으로 난이도를 올린다. 이렇게 하면 sparse reward 상황에서도 학습 초기에 agent가 성공 경험을 쌓을 수 있다.

### Reward Hacking

Agent가 reward를 최대화하되, 설계자가 의도하지 않은 방식으로 행동하는 현상이다.

대표적인 예시:
- 로봇 팔이 물체를 "옮기라"고 했는데 물체를 밀어서 목표 위치로 보내기 (잡지 않음)
- 보행 로봇이 "빠르게 이동하라"고 했는데 넘어지면서 미끄러지기
- 점프를 학습하라고 했는데 비정상적으로 긴 형태로 진화 (형태 최적화와 결합 시)

대응 방법:
- reward 함수를 반복적으로 수정하고 학습된 행동을 검토한다 (사실상 trial-and-error).
- 원치 않는 행동에 대한 penalty term을 추가한다.
- 비디오를 보면서 정성적으로 검토한다. 자동화하기 어려운 부분이다.

### Constrained RL

Safety constraint를 명시적으로 다루는 RL이다. 일반적인 RL이 reward를 최대화하는 것이라면, constrained RL은 reward를 최대화하되 cost를 일정 한도 이하로 유지한다.

```
max_π E[ Σ γ^t r_t ]   subject to   E[ Σ γ^t c_t ] ≤ d
```

c_t는 cost (예: 관절 토크 한계 초과, 장애물 충돌), d는 허용 한도다.

대표적인 알고리즘으로 CPO (Constrained Policy Optimization), PCPO, Lagrangian relaxation 기반 방법 등이 있다. 실제 로봇에서는 하드웨어 보호를 위해 토크 제한, 관절 각도 제한 등을 constraint로 넣는 것이 현실적이다.

### Human-in-the-loop RL

사람의 피드백을 reward 신호로 사용하는 접근이다. LLM의 RLHF (RL from Human Feedback)와 같은 아이디어를 로보틱스에 적용한 것이다.

방법:
1. 로봇의 행동 쌍을 보여주고 사람이 선호도를 표시한다 (A가 B보다 나음).
2. 선호도 데이터로 reward model을 학습한다.
3. 학습된 reward model로 RL을 수행한다.

수치적으로 reward를 정의하기 어려운 태스크 (예: "자연스럽게 걷기", "조심스럽게 물건 놓기")에서 유용하다. 단점은 사람의 시간이 많이 든다는 점과, reward model이 부정확할 수 있다는 점이다.


---

## 8.9 추천 자료

> **Sutton & Barto, "Reinforcement Learning: An Introduction" (2nd edition)**
> http://incompleteideas.net/book/the-book-2nd.html
> RL의 필수 교재. 무료 PDF 제공. MDP부터 policy gradient까지 기초를 다지려면 반드시 읽어야 한다. 전부 읽을 시간이 없으면 Ch.1-6, Ch.13을 우선으로.

> **Sergey Levine, CS285: Deep Reinforcement Learning**
> https://rail.eecs.berkeley.edu/deeprlcourse/
> 로봇 RL에 초점을 맞춘 대학원 수준 강의. 강의 영상과 슬라이드 모두 공개되어 있다. 이 챕터의 대부분의 주제를 더 깊이 다룬다.

> **Stable-Baselines3**
> https://stable-baselines3.readthedocs.io/
> PyTorch 기반 RL 알고리즘 라이브러리. PPO, SAC, TD3 등 주요 알고리즘이 구현되어 있다. 빠른 프로토타이핑에 적합.

> **CleanRL**
> https://github.com/vwxyzjn/cleanrl
> 단일 파일 RL 구현 모음. 한 파일에 알고리즘 전체가 들어 있어서 코드를 따라가며 공부하기에 좋다. 알고리즘 내부를 이해하고 싶으면 SB3보다 이쪽을 권장한다.

> **Isaac Lab**
> https://isaac-sim.github.io/IsaacLab/
> NVIDIA의 GPU 병렬 로봇 시뮬레이션 프레임워크. ANYmal locomotion, dexterous manipulation 등 대규모 학습 프로젝트에서 폭넓게 채택되고 있다.

> **LeRobot (HuggingFace)**
> https://github.com/huggingface/lerobot
> 모방학습과 로봇 학습을 위한 프레임워크. ACT, Diffusion Policy 등의 구현이 포함되어 있다. 데이터셋도 함께 제공한다.

> **robomimic**
> https://robomimic.github.io/
> 모방학습 알고리즘 벤치마크. BC, BC-RNN, HBC 등 다양한 모방학습 방법을 동일 조건에서 비교할 수 있다.

> **추가 논문**
> - [Andrychowicz et al., "Hindsight Experience Replay" (NeurIPS 2017, arXiv:1707.01495)](https://arxiv.org/abs/1707.01495) — sparse reward 문제 해결의 핵심. 실패한 trajectory를 성공으로 재레이블링
> - [Hafner et al., "Mastering Diverse Domains through World Models" (DreamerV3, arXiv:2301.04104)](https://arxiv.org/abs/2301.04104) — 단일 하이퍼파라미터로 150+ 도메인 학습. World model 기반 RL의 현 시점 대표적 성과
> - [Chi et al., "Universal Manipulation Interface" (UMI, RSS 2024, arXiv:2402.10329)](https://arxiv.org/abs/2402.10329) — 핸드헬드 그리퍼로 데이터 수집, 다양한 로봇에 zero-shot 배포
> - [Fu et al., "Mobile ALOHA" (CoRL 2024, arXiv:2401.02117)](https://arxiv.org/abs/2401.02117) — 모바일 베이스 + 양팔 텔레오퍼레이션. co-training으로 성공률 대폭 향상


---

## 기술 흐름

```
1992 ── REINFORCE algorithm (Williams)
         최초의 policy gradient 방법. 수렴 증명은 있으나 variance가 높다.

2013 ── DQN (Mnih et al., Atari)
         Deep RL의 시작. replay buffer + target network로 안정적 학습.

2015 ── TRPO (Schulman et al.)
         trust region 제약으로 안정적 policy update. 이론은 좋으나 구현이 복잡.

2017 ── PPO (Schulman et al.)
         TRPO의 실용적 대안. clipping으로 간단하게 구현. 로봇 RL 실험의 기본 베이스라인.

2018 ── SAC (Haarnoja et al.)
         entropy regularization + off-policy. 연속 공간에서 sample efficient.

2019 ── ANYmal: sim-to-real locomotion (ETH Zurich)
         대규모 domain randomization으로 사족보행 sim-to-real 성공.

2020 ── DAgger 실전 적용 확산
         모방학습의 실용성 입증. 다양한 로봇 플랫폼에 적용.

2022 ── RT-1 (Google)
         대규모 로봇 데이터 + Transformer. 다중 태스크 범용 정책.

2023 ── ACT/ALOHA (Stanford), Diffusion Policy (CMU)
         모방학습의 새로운 표준. action chunking과 diffusion으로 성능 향상.

2023 ── RT-2 (Google)
         VLM을 직접 action 생성에 사용. 웹 지식의 로봇 전이.

2024 ── Octo, OpenVLA, pi0
         오픈소스 범용 정책 모델 등장. Cross-embodiment 학습 시작.

2025 ── Cross-embodiment learning 연구 확산
         서로 다른 로봇 간 정책 전이. 데이터 스케일링 법칙 검증 중.
```
