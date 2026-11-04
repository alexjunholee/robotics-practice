# Ch.8 — 로봇 러닝 (Robot Learning)


Ch.6의 제어와 Ch.7의 플래닝에서는 사람이 모델과 목표를 정했다. 로봇 러닝에서는 로봇이 데이터와 경험으로 행동을 학습한다.

---

## 8.1 왜 로봇 러닝을 배우는가

**전통적 방법이 잘 맞는 작업**

PID 제어, MPC, RRT 같은 전통적 제어·플래닝은 dynamics 모델이 정확하고 환경이 정해져 있을 때 잘 동작한다. 산업용 로봇 팔이 정해진 위치의 부품을 집어 조립하는 작업이 대표적이다. 이 방법들의 안정성·최적성 보장은 각 방법이 가정한 모델·제약·해법 조건 안에서 성립한다. 그 조건을 만족하는 작업에 학습 기반 방법을 쓰려면 이점을 따로 입증해야 한다.

**전통적 방법으로 풀기 어려운 작업**

- **모델링이 어려운 dynamics**: 천, 로프, 유체 같은 deformable object는 정확한 물리 모델을 세우기가 매우 어렵다.
- **복잡한 접촉(contact)**: 물체를 손으로 돌리거나 끼워 맞추는 작업에서는 접촉 모드가 수시로 바뀐다. 접촉 역학을 정확히 모델링하는 일은 아직 풀리지 않았다.
- **비정형 환경**: 가정집 부엌이나 재난 현장에서는 물체의 종류와 위치가 그때그때 바뀐다.

학습 기반 방법은 데이터에서 입력과 출력의 관계를 직접 근사한다. 그래서 이런 작업에서 명시적 모델 대신 쓴다.

**학습 기반 방법의 한계**

- **데이터 효율(sample efficiency)**: 강화학습에는 수백만 스텝의 상호작용이 필요할 때가 많다. 실제 로봇으로 이만큼 모으려면 시간과 비용이 지나치게 든다.
- **안전성(safety)**: 탐색(exploration) 중에 로봇이 자기 몸이나 주변 물건을 부술 수 있다.
- **일반화(generalization)**: 학습 조건에서 조금 벗어난 환경에서 성능이 크게 떨어질 수 있다.

실무에서는 전통적 방법으로 풀리는 부분에 전통적 방법을 쓰고, 모델을 세우기 어려운 부분에 학습을 쓴다.

Probabilistic Robotics(Thrun·Burgard·Fox)의 MDP 장(1999–2000년 초고 15장)은 네 가지 예로 불확실성 아래에서 행동을 고르는 문제를 소개한다. 컨베이어 벨트에 무작위 자세로 실려 오는 부품을 집어 조립하는 매니퓰레이터는 부품 자세를 모르는 채로 잡는 방법을 정해야 한다. 북극 얼음 밑을 지나는 수중 차량은 방향을 잃을 위험을 안고 최단 경로로 갈지, 시간이 더 들더라도 특징 없는 넓은 구역을 피할지 골라야 한다. 숲속을 자율 비행하는 헬리콥터는 예상하지 못한 돌풍이 불어도 부딪히지 않으려면 장애물에서 얼마나 떨어져야 하는지 정해야 한다. 미지의 행성을 탐사하는 로봇 팀은 서로의 처음 상대 위치를 모를 때 서로를 찾아 위치를 맞출지, 흩어져 더 넓은 지형을 탐사할지 정해야 한다.


---

## 8.2 강화학습 기초 (RL Basics)

### MDP (Markov Decision Process)

강화학습은 문제를 MDP로 정식화한다. 구성 요소는 다음과 같다.

- **State (s)**: 환경의 현재 상태. 로봇의 관절 각도, 속도, 물체 위치 등.
- **Action (a)**: 에이전트가 취하는 행동. 관절 토크, 목표 관절 각도 등.
- **Reward (r)**: 행동의 결과로 받는 스칼라 보상 신호. r = R(s, a).
- **Transition (T)**: 상태 전이 확률. T(s'|s, a). 현재 상태에서 행동을 취했을 때 다음 상태의 분포.
- **Discount factor (γ)**: 미래 보상의 할인율. 0 < γ ≤ 1. 로봇 RL에서는 γ = 0.99가 흔한 초기값이다.

목표는 누적 할인 보상(cumulative discounted reward)을 최대로 만드는 policy π(a|s)를 찾는 것이다.

```
J(π) = E[ Σ_{t=0}^{∞} γ^t · r_t ]
```

Markov property는 "다음 상태는 현재 상태와 행동에만 의존한다"는 가정이다. 현재 상태가 과거 히스토리의 정보를 모두 담는다. 실제 로봇에서 자주 깨지는 쪽은 관측이 곧 상태라는 가정이다(부분 관측, 즉 POMDP 상황). 이때도 잠재 상태의 전이는 Markov다. 관측만 입력으로 받는 정책에는 상태를 추정할 정보가 모자란다. 그래서 observation history나 belief를 state로 쓰거나 recurrent policy를 쓴다.

MDP는 최적 정책의 기준을 정한다.

### MDP Value Iteration

환경 모델 $p(x'|x,a)$와 보상 $r$이 알려진 경우 dynamic programming으로 직접 최적 정책을 계산할 수 있다. 여기서는 PR(Thrun·Burgard·Fox) 표기를 따라 상태를 $x$, 전이 확률을 $p(x' \mid x, a)$로 쓴다(앞 절의 $s$, $T(s' \mid s, a)$와 같다). 이 절의 $T$는 horizon이다.

#### Payoff와 Horizon

보상 $r(x, a)$를 payoff 함수라 부른다. 상태 $x$에서 행동 $a$를 취했을 때 즉시 받는 스칼라 값이다. 정책 $\pi: x \mapsto a$는 모든 상태를 행동으로 보내는 함수다. 정책의 품질은 누적 할인 보상의 기댓값으로 측정한다.

$$V^\pi(x_0) = \mathbb{E}\left[\sum_{t=0}^{T} \gamma^t \, r(x_t, \pi(x_t))\right]$$

Horizon $T$에 따라 세 경우로 나뉜다.

- **T=1 (greedy)**: 다음 한 스텝의 보상만 최대화한다. 계산은 단순하지만 그다음 스텝부터의 결과는 따지지 않는다.
- **유한 horizon**: 정해진 $T$ 스텝까지 최적화한다. 시간 $t$에 따라 정책이 달라져야 하므로(time-dependent policy) 표현이 복잡해진다.
- **무한 horizon (T=∞)**: $\gamma < 1$이고 보상이 유계이면 $V^\pi$가 유한하다. 모든 $x, a$에서 $|r(x, a)| \leq r_{\max}$이면 $|V^\pi| \leq r_{\max}/(1-\gamma)$다. 이 설정에서는 시간에 무관한 정상 정책(stationary policy) 가운데 최적 정책이 있다. 로봇 RL에서는 무한 horizon + 할인이 기본 설정이다.

할인 인자 $\gamma$의 직관: 1 스텝 뒤의 보상은 $\gamma$배, 2 스텝 뒤의 보상은 $\gamma^2$배로 줄어든다. $\gamma$를 낮추면 로봇이 가까운 보상 위주로 행동한다. $\gamma$를 1에 가깝게 올리면 먼 미래 보상까지 반영한다. $\gamma = 0.99$는 약 100 스텝($1/(1-\gamma)$) 앞까지 반영하면서 $\gamma<1$이라 수렴도 보장된다.

#### Bellman 방정식

무한 horizon에서 최적 가치 함수 $V^*(x)$는 다음 Bellman 방정식을 만족한다.

$$V^*(x) = \max_a \left[ r(x, a) + \gamma \sum_{x'} p(x' \mid x, a) \, V^*(x') \right]$$

상태 $x$의 최적 가치는 행동마다 즉시 보상 $r(x,a)$에 다음 상태들의 최적 가치를 할인해 더한 값 가운데 가장 큰 값이다. 같은 $V^*$가 양변에 나오는 재귀식이다.

최적 정책은 이 식에서 최댓값을 주는 행동을 골라 얻는다.

$$\pi^*(x) = \arg\max_a \left[ r(x, a) + \gamma \sum_{x'} p(x' \mid x, a) \, V^*(x') \right]$$

$T=1$의 최적 가치에서 $T=2$를 재귀로 구하고 $T \to \infty$ 극한을 취하면 위 Bellman 방정식이 나온다. 이번 스텝의 최적 선택과 남은 스텝의 최적 가치를 합치는 Dynamic Programming 구조다.

#### Value Iteration 알고리즘

Bellman 방정식은 $V^*$의 고정점 방정식(fixed-point equation)이다. Value Iteration은 오른쪽에 현재 추정값 $V_k$를 넣어 $V_{k+1}$을 구하는 계산을 수렴할 때까지 반복한다.

```
Algorithm MDP_value_iteration():
  모든 상태 x에 대해:
    V_0(x) ← 0

  수렴할 때까지 반복:           ε: 허용 오차 (예: 1e-6)
    모든 상태 x에 대해:
      V_{k+1}(x) ← max_a [ r(x, a) + γ · Σ_{x'} p(x'|x, a) · V_k(x') ]
    if max_x |V_{k+1}(x) - V_k(x)| < ε: break

  최적 정책 추출:
    π*(x) ← argmax_a [ r(x, a) + γ · Σ_{x'} p(x'|x, a) · V_k(x') ]

  return V_k, π*
```

변수 정리: $V_k$는 k번째 반복의 가치 추정값, $V^*$는 수렴 후 최적 가치, $\pi^*$는 최적 정책, $r$은 보상, $\gamma$는 할인 인자, $p(x'|x,a)$는 전이 확률.

상태를 갱신하는 순서는 자유롭게 정한다. Sutton & Barto(2018) §4.5에 따르면 모든 상태가 무한히 자주 갱신되면 수렴한다. 위 수식은 이산 상태 공간을 가정해 합(Σ)으로 썼다. 연속 상태 공간에서는 합 자리에 적분을 쓴다.

수렴 보장: Bellman 갱신 연산자는 $\gamma$를 수축률(contraction rate)로 하는 수축 사상(contraction mapping)이다. 그래서 반복은 유일한 고정점 $V^*$로 수렴하고, $k$번 반복 후 오차는 초기 오차의 $\gamma^k$배 이하로 줄어든다: $\|V_k - V^*\|_\infty \leq \gamma^k \|V_0 - V^*\|_\infty$.

<!-- DEMO: mdp_value_iteration_grid.html -->

#### 2D Grid World 예시

$5 \times 5$ 격자를 예로 든다. 각 셀이 상태 $x$이고, 행동은 상하좌우 4방향이다. 목표 셀에 도달하면 $r = +100$, 장애물 셀에 부딪히면 $r = -10$, 그 외는 $r = -1$ (이동 비용). 전이는 의도한 방향으로 0.8 확률, 직교 방향으로 각 0.1 확률 (확률적 전이).

Value Iteration을 돌리면 가치 함수가 목표 셀 근처에서 높아지고 장애물 근처에서 낮아진다. 첫 반복에서는 목표에 붙은 셀만 큰 값을 얻는다. 반복할 때마다 그 값이 한 칸씩 더 먼 셀로 퍼진다. 수렴한 뒤 각 셀에서 즉시 보상과 다음 상태 가치의 기댓값을 더한 값이 가장 큰 행동을 고르면 최적 정책 $\pi^*$가 나온다. 이 정책은 장애물을 돌아간다. $\pi^*$는 모든 셀에 행동을 하나씩 정해 주므로, 로봇은 가치 함수만으로 움직이고 RRT나 A* 같은 경로 탐색은 쓸 일이 없다. 전이가 확률적이라서 실제 궤적은 실행할 때마다 달라진다.

Value Iteration에는 환경 모델 $p(x'|x,a)$와 $r$이 필요하다. 실제 로봇에서는 전이 확률을 미리 알기 어려워서, 경험으로 policy를 직접 개선하는 방법(아래와 §8.3)을 쓴다. 부분 관측 상황의 플래닝은 Ch.7 §7.9에 있다.

### Policy Gradient 직관

Policy gradient는 아래 순서로 policy를 개선한다.

1. 현재 policy로 여러 trajectory를 수집한다.
2. return이 높았던 trajectory에 들어 있는 action의 확률을 올린다.
3. return이 낮았던 trajectory에 들어 있는 action의 확률을 내린다.

수식으로 쓰면:

```
∇J(θ) = E[ Σ_t ∇log π_θ(a_t|s_t) · A_t ]
```

θ는 policy의 파라미터이고, A_t는 advantage로, 그 action이 그 상태의 평균보다 얼마나 좋았는지를 나타낸다. `log π(a|s)`의 gradient는 action a의 확률을 올리는 방향이다. 여기에 advantage를 곱하면 평균보다 좋았던 action의 확률이 오르고 나빴던 action의 확률이 내려간다. 이 gradient를 따라 θ를 업데이트한다.

### Value Function, Q-function

- **Value function V^π(s)**: state s에서 policy π를 따랐을 때 기대되는 cumulative reward.
- **Q-function Q^π(s, a)**: state s에서 action a를 취하고 이후 π를 따랐을 때의 기대 cumulative reward.
- **Advantage A^π(s, a) = Q^π(s, a) - V^π(s)**: action a가 평균 대비 얼마나 좋은지.

Value function을 따로 학습해 baseline으로 쓰면 gradient 추정의 variance가 줄어든다. PPO, SAC, TD3는 모두 policy network와 value network를 함께 학습하는 actor-critic 구조다.

### On-policy vs Off-policy

- **On-policy**: 현재 policy가 수집한 데이터로 학습한다. 방금 모은 rollout으로 몇 epoch 학습한 뒤 버린다. PPO가 여기에 속한다. 학습이 안정적인 대신 sample efficiency(같은 성능에 드는 데이터 양의 효율)가 낮다.
- **Off-policy**: 과거 policy가 수집한 데이터를 replay buffer에 쌓아 재사용한다. SAC, TD3가 여기에 속한다. 데이터를 적게 쓰는 대신 하이퍼파라미터에 따라 학습이 흔들릴 수 있다.

실제 로봇에서는 데이터 수집 비용이 커서 off-policy 방법을 많이 쓴다. 시뮬레이션에서 대규모 병렬 환경을 돌릴 때는 PPO 같은 on-policy 방법을 많이 쓴다.


---

## 8.3 주요 RL 알고리즘

### PPO (Proximal Policy Optimization)

PPO는 Schulman et al. (2017)이 제안한 on-policy 알고리즘이다. surrogate 목적함수 안의 확률 비율을 clipping해서, 이전 policy에서 크게 벗어난 변화에서는 목적함수 값이 더 오르지 않게 한다. TRPO가 KL 신뢰 영역 제약으로 정책 변화를 제한하던 일을 PPO는 목적함수의 clipping으로 한다.

```
L_CLIP(θ) = E[ min( r_t(θ) · A_t, clip(r_t(θ), 1-ε, 1+ε) · A_t ) ]
```

여기서 r_t(θ) = π_θ(a_t|s_t) / π_θ_old(a_t|s_t)는 probability ratio다. 원 논문 기호를 따랐으며, §8.2의 보상 r_t와는 다른 양이다. ε의 원 논문 기본값은 0.2다.

PPO는 구현이 간단하고 기본 하이퍼파라미터로 여러 문제에서 학습이 된다. Engstrom et al.(2020)은 value function clipping, reward scaling, 가중치 초기화 같은 구현 세부를 켜고 끄며 실험했고, PPO가 TRPO보다 높은 보상을 얻은 이유의 대부분이 clipping보다 이런 구현 세부에 있다고 보고했다. NVIDIA Isaac Lab 같은 병렬 시뮬레이터에서는 환경 수천 개를 동시에 돌려 데이터를 모으므로 낮은 sample efficiency가 문제가 되지 않는다.

### SAC (Soft Actor-Critic)

SAC는 entropy regularization을 더한 off-policy 알고리즘이다. 보상과 policy의 entropy를 함께 최대화해서 여러 action을 고르게 시도하게 한다.

```
J(π) = E[ Σ_t γ^t ( r_t + α · H(π(·|s_t)) ) ]
```

temperature parameter α로 entropy와 reward의 비중을 정한다. 목표 entropy를 정해 α를 자동으로 조절하는 버전도 있다(Haarnoja et al., 2018b).

SAC는 replay buffer의 데이터를 여러 번 재사용해서 연속 action space에서 데이터를 적게 쓴다. 실제 로봇에서 데이터를 직접 모을 때 PPO보다 SAC를 많이 고른다.

### TD3 (Twin Delayed DDPG)

TD3는 DDPG를 고친 off-policy 알고리즘이다. DDPG에서 바꾼 점은 아래와 같다.

1. **Twin Q-networks**: Q-function 두 개를 학습하고 둘 중 작은 값을 써서 overestimation bias(과대추정 편향)를 줄인다.
2. **Delayed policy update**: critic을 여러 번 업데이트한 뒤에 policy를 한 번 업데이트한다.
3. **Target policy smoothing**: target action에 노이즈를 추가한다.

TD3는 entropy 항이 없어 하이퍼파라미터가 조금 적다. 탐색(exploration)은 action에 더하는 노이즈에 맡긴다.

### 알고리즘 선택 가이드

| 상황 | 추천 알고리즘 | 이유 |
|------|-------------|------|
| 시뮬레이션, GPU 병렬화 가능 | PPO | 병렬 환경으로 데이터를 많이 모아 sample efficiency를 메움 |
| 실제 로봇, 데이터 적음 | SAC | off-policy, sample efficient |
| 연속 action space | SAC 또는 TD3 | 둘 다 연속 action space용으로 설계됨 |
| 이산 action space | PPO 또는 DQN | SAC를 이산 행동에 쓰려면 별도 변형을 구현해야 함 |
| 처음 시작하는 프로젝트 | PPO | 기본값으로 학습이 되는 경우가 많아 디버깅이 쉬움 |

### Stable-Baselines3 코드 예시

PPO로 MuJoCo Ant 환경을 학습하는 기본 코드다.

```python
import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.evaluation import evaluate_policy

# 병렬 환경 생성 (8개)
vec_env = make_vec_env("Ant-v5", n_envs=8)

# PPO 에이전트 생성
model = PPO(
    "MlpPolicy",
    vec_env,
    learning_rate=3e-4,
    n_steps=2048,        # 환경 하나가 한 번의 rollout에서 수집할 스텝 수 (8개 환경이면 2048 × 8)
    batch_size=64,
    n_epochs=10,         # 수집한 데이터로 몇 epoch 학습할지
    gamma=0.99,
    gae_lambda=0.95,     # GAE (Generalized Advantage Estimation)
    clip_range=0.2,
    verbose=1,
    tensorboard_log="./ppo_ant_tb/",  # tensorboard 패키지가 있어야 한다 (pip install tensorboard)
)

# 학습 (총 2M 스텝)
model.learn(total_timesteps=2_000_000)

# 평가
eval_env = gym.make("Ant-v5")
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
    "Ant-v5",
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

> 빠른 프로토타이핑에는 Stable-Baselines3를 쓴다. 알고리즘 내부를 공부할 때는 알고리즘마다 파일 하나로 구현된 CleanRL을 읽는다.


---

## 8.4 시뮬레이션 환경

실제 하드웨어에서 수백만 스텝을 모으려면 비용과 위험이 커서, 로봇 RL은 주로 시뮬레이션에서 학습한다. 시뮬레이터마다 접촉 모델, 병렬화 방식, 로봇 자산이 다르다.

### MuJoCo (Multi-Joint dynamics with Contact)

DeepMind가 인수해 2021년에 무료로 풀었고, 2022년에 소스 코드를 공개했다. 접촉 시뮬레이션과 수치 적분이 안정적이라 Gymnasium의 연속 제어 벤치마크(Ant, Humanoid 등)가 MuJoCo로 돌아간다. 기본 엔진은 CPU에서 돌고, MuJoCo 3.0부터 MJX(JAX backend)로 GPU 병렬화를 지원한다. 알고리즘 벤치마크와 소규모 실험에 많이 쓴다.

### Isaac Lab (NVIDIA)

NVIDIA Isaac Sim 위에서 돌아가는 로봇 학습 프레임워크다. GPU 병렬 시뮬레이션으로 수천~수만 개 환경을 동시에 돌리고, 사실적 렌더링과 sensor 시뮬레이션을 지원한다. NVIDIA GPU가 있어야 하고 설치·설정이 복잡하다. 대규모 locomotion 학습과 sim-to-real 파이프라인에 쓴다.

### PyBullet

`pip install` 한 줄로 설치되는 오픈소스 물리 엔진이다. 처음 RL 코드를 돌려 보거나 아이디어를 빠르게 확인할 때 쓴다.

### Brax

Google이 개발한 JAX 기반 물리 엔진이다. JAX의 JIT 컴파일 덕분에 GPU·TPU에서 빠르게 돈다. 자동 미분이 되므로 differentiable physics(미분 가능한 물리 시뮬레이션) 연구에도 쓴다. 접촉 모델이 단순해서 복잡한 접촉 작업의 정확도는 MuJoCo보다 낮다.

### 환경 비교표

| 시뮬레이터 | 물리 정확도 | 속도 | GPU 병렬화 | 설치 난이도 | 주 용도 |
|-----------|-----------|------|-----------|-----------|---------|
| MuJoCo | 높음 | 보통 | MJX로 가능 | 쉬움 | 알고리즘 벤치마크 |
| Isaac Lab | 높음 | 매우 빠름 | 수천~만 | 어려움 | 대규모 로봇 학습 |
| PyBullet | 보통 | 느림 | 불가 | 매우 쉬움 | 입문/교육 |
| Brax | 낮음 | 매우 빠름 | 가능 | 보통 | 빠른 반복 실험 |

처음에는 MuJoCo + Gymnasium으로 시작하고, 대규모 실험이 필요할 때 Isaac Lab으로 옮긴다.


---

## 8.5 Sim-to-Real Transfer

시뮬레이션에서 학습한 policy를 실제 로봇에 옮기는 일을 Sim-to-Real transfer라 한다.

### Reality Gap

시뮬레이션과 현실은 아래 항목에서 차이(gap)가 난다.

- **물리 파라미터 차이**: 마찰 계수, 질량, 관성 모멘트 등이 시뮬레이션과 다르다.
- **센서 노이즈**: 실제 센서는 노이즈, 지연, 드리프트가 있다.
- **액추에이터 모델링 오차**: 모터의 비선형성, 기어 백래시, 컴플라이언스 등.
- **접촉 모델 차이**: 시뮬레이션의 접촉 모델은 현실을 근사한다.

그래서 시뮬레이션에서 높은 reward를 얻은 policy도 실제 로봇에서 넘어질 수 있다.

### Domain Randomization

시뮬레이션의 물리 파라미터를 무작위로 바꿔 가며 학습해, policy가 넓은 파라미터 범위에서 동작하게 만든다. OpenAI의 Dactyl(IJRR 2020, arXiv 2018)은 물리 파라미터와 시각 요소를 함께 랜덤화해, 시뮬레이션에서만 학습한 정책으로 실제 Shadow Hand의 물체 회전 작업을 해냈다.

랜덤화하는 파라미터의 예:

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

범위를 넓게 잡을수록 현실 파라미터가 범위 안에 들 가능성이 커진다. 대가로 특정 파라미터 하나에서 잰 성능은 그 파라미터에 맞춰 학습한 정책보다 낮아진다. 실제 시스템의 파라미터는 정확히 알기 어려워서, 실제 로봇에서는 무작위화 정책이 더 잘 동작할 때가 있다.

### System Identification (Sys-ID)

Sys-ID(System Identification)에서는 실제 로봇의 물리 파라미터를 재거나 추정해 시뮬레이션에 넣는다.

방법:
- 직접 측정: 전자저울로 질량 측정, 마찰 계수 실험 측정
- 파라미터 최적화: 실제 로봇과 시뮬레이션의 trajectory 차이를 최소화하는 파라미터를 찾음
- 온라인 적응: 로봇을 운용하는 동안 파라미터를 계속 추정해 고침

Sys-ID와 domain randomization을 함께 쓸 때는 Sys-ID로 파라미터의 중심값을 잡고, 그 주변 범위를 domain randomization으로 흔든다.

### Teacher-Student 구조

시뮬레이션에서만 얻을 수 있는 정보(privileged information)를 학습에 쓰는 방법이다. 두 단계로 학습한다.

1. **Teacher 학습**: 시뮬레이션에서 privileged information (정확한 지형 높이, 정확한 마찰 계수, 물체의 정확한 위치 등)을 state에 넣어 policy를 학습한다. 정답에 가까운 정보를 입력으로 받으므로 학습이 빠르다.
2. **Student 학습**: 실제 로봇에서 얻을 수 있는 observation (IMU, 관절 encoder, 카메라 등)만으로 teacher의 행동을 모방하도록 학습한다.

ETH Zurich의 Lee et al.(2020)은 이 방식으로 ANYmal 사족보행 정책을 학습했다. Teacher는 정확한 지형 정보를 입력으로 받는다. Student는 관절·IMU 측정의 이력(proprioception history)만 받아 teacher의 행동을 따라 한다.

### 실제 사례

**ANYmal 보행 (ETH Zurich / Robotic Systems Lab)**
- 강화학습 + domain randomization + teacher-student로 사족보행 학습 (Lee et al., Science Robotics 2020)
- 시뮬레이션에서만 학습한 뒤 실제 로봇에 zero-shot transfer
- 진흙, 눈, 잔해, 우거진 수풀, 흐르는 물 같은 지형에서 보행

**Dexterous Hand Manipulation (OpenAI)**
- Rubik's cube를 Shadow Hand로 풀기 (OpenAI, 2019)
- Automatic Domain Randomization(ADR)으로 랜덤화 범위를 학습 중에 자동으로 넓힘
- 시뮬레이션에서 약 13,000년 분량의 경험으로 학습
- 실제 로봇의 성공률은 가장 어려운 스크램블(26수)에서 20%, 15수 스크램블에서 60%였다


---

## 8.6 모방학습 (Imitation Learning)

모방학습에서는 reward 함수를 설계하는 대신 전문가(사람)의 시연 데이터로 policy를 직접 학습한다.

### Behavioral Cloning (BC)

가장 단순한 모방학습이다. 전문가의 (observation, action) 쌍을 수집하고, 지도학습(supervised learning)으로 policy를 학습한다.

```python
import numpy as np
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

# 전문가 데이터 (NumPy -> Tensor)
# expert_obs: (N, obs_dim), expert_act: (N, act_dim)
# 여기서는 형태만 맞춘 임의 배열이다. 실제로는 모은 시연 데이터를 불러온다.
expert_obs = np.random.randn(1000, 48).astype(np.float32)
expert_act = np.random.randn(1000, 7).astype(np.float32)
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

**Compounding error**: 학습된 policy가 전문가 trajectory에서 조금 벗어나면 학습 데이터 밖의 상태에 들어간다. 그 상태에서 낸 행동이 로봇을 더 멀리 보낸다. 이 과정이 쌓인다. Ross & Bagnell(2010)에 따르면, 한 스텝 비용이 0~1이고 전문가가 지나간 상태에서 policy가 전문가와 다른 행동을 낼 확률이 $\epsilon$일 때 horizon $T$ 동안의 비용은 전문가보다 최대 $\epsilon T^2$만큼 커질 수 있다.

### DAgger (Dataset Aggregation)

DAgger는 compounding error를 줄이는 방법이다.

1. 초기 전문가 시연 데이터로 기본 BC policy를 학습한다.
2. 학습된 policy를 실행해 새 trajectory를 모은다.
3. 그 궤적의 각 상태에서 전문가가 취할 행동을 라벨로 붙인다.
4. 새 상태-행동 쌍을 기존 데이터셋에 합쳐 정책을 다시 학습한다.
5. 성능이 수렴할 때까지 2~4단계를 반복한다.

DAgger는 policy가 실제로 방문한 state에서 전문가가 선택할 action을 다시 받아 학습 데이터에 포함한다. 원 논문(Ross, Gordon & Bagnell, 2011)은 각 반복의 정책을 no-regret online learning 알고리즘으로 고른다는 가정 아래, 반복을 충분히 하면 비용이 전문가보다 $T$에 비례하는 만큼만 커지는 정책이 그 가운데 있음을 보였다.

반복마다 전문가가 라벨을 붙여야 해서 사람 시간이 많이 든다.

### ACT (Action Chunking with Transformers)

Stanford의 ALOHA 프로젝트(Zhao et al., RSS 2023)가 제안한 방법이다.

1. **Action chunking**: 미래 k 스텝의 action sequence를 한 번에 예측한다. 정책이 결정을 내리는 횟수가 1/k로 줄어 compounding error가 줄어든다.
2. **CVAE (Conditional Variational Autoencoder)**: 사람 시연의 다양한 행동 분포를 잠재 변수로 모델링한다. MSE loss만 쓰면 서로 다른 시연 행동의 평균이 나와 어느 시연과도 다른 action이 된다.

Transformer encoder-decoder 구조이고, joint position과 카메라 이미지를 입력으로 받는다.

### Diffusion Policy

Chi et al. (RSS 2023)은 diffusion model로 action을 생성했다. Diffusion policy는 denoising을 반복해 복잡한 action 분포에서 샘플을 뽑는다. MSE 회귀로 구현한 BC는 사실상 단봉 Gaussian을 가정한다. Diffusion policy는 다봉 분포를 그대로 표현한다. BC를 에너지 기반 모델이나 혼합 밀도 모델로 구현해도 다봉 분포를 표현할 수 있다.

```python
# Diffusion Policy의 action 생성 과정 (pseudo-code)
# 1. 순수 noise에서 시작
action = torch.randn(batch_size, horizon, action_dim)

# 2. K번의 denoising step
for k in reversed(range(K)):
    # 현재 observation을 조건으로 noise를 예측
    predicted_noise = noise_pred_net(action, k, obs_encoding)
    # noise 제거 (DDPM 또는 DDIM scheduler 사용)
    action = scheduler.step(predicted_noise, k, action)

# 3. 최종 action sequence 출력
```

2023년 이후 manipulation 모방학습 논문은 Diffusion policy와 ACT를 베이스라인으로 많이 쓴다. LeRobot(HuggingFace)에 둘 다 구현되어 있다.

### 데이터 수집 방법

모방학습의 성능은 데이터 품질에 따라 크게 달라진다. 데이터는 아래 방법으로 모은다.

- **Teleoperation**: 사람이 원격으로 로봇을 조종한다. ALOHA는 사람이 leader 팔을 움직이면 follower 팔이 따라 하는 구조로, 2만 달러 미만의 장비로 양팔 조작 데이터를 모은다.
- **VR controller**: VR 컨트롤러로 end-effector 위치/자세를 지정한다. 손 움직임을 그대로 옮기기 쉽다. contact-rich 작업에서는 조작자가 접촉력을 느낄 수단이 없다.
- **Kinesthetic teaching**: 사람이 로봇 팔을 직접 잡고 움직인다. 크고 무거운 로봇에는 쓰기 어렵다.
- **Space mouse**: 6-DoF 입력 장치. 한 손으로 조작 가능. 정밀 작업에 유용하다.

필요한 데이터 양은 태스크와 방법에 따라 다르다. Chi et al.은 실제 로봇 태스크마다 시연을 수백 개 이하로 모았다. Push-T는 136개, 머그컵 뒤집기는 250개였고(RSS 2023), IJRR 확장판(2024)에 더한 양팔 셔츠 접기는 284개였다. 시연을 늘리면 성능이 오르지만 수집 비용도 는다.


---

## 8.7 심화: Foundation Models for Robot Control

LLM과 VLM처럼 로봇에서도 대규모 사전학습 모델(foundation model)을 만들고 있다. 많은 로봇 데이터로 범용 정책(generalist policy)을 학습한 뒤 새 로봇이나 태스크에 맞춰 조정한다.

### RT-1, RT-2 (Google DeepMind)

**RT-1 (RSS 2023)**: 로봇 13대로 17개월 동안 모은 약 13만 개의 시연으로 학습한 Transformer 기반 policy다. 이미지와 자연어 명령을 받아 action을 출력한다. 모델 하나로 700개 이상의 태스크 지시를 처리했다.

**RT-2 (CoRL 2023)**: PaLM-E나 PaLI-X 같은 VLM(Vision-Language Model)을 action 토큰을 출력하도록 파인튜닝했다. 로봇 데이터에 없던 물체와 지시도 웹에서 사전학습한 지식으로 일부 처리했다.

### Octo

UC Berkeley, Stanford, CMU 등이 개발한 오픈소스 범용 로봇 정책이다. 여러 기관이 여러 로봇으로 모은 Open X-Embodiment 데이터셋으로 학습했다. Diffusion 기반 action head를 쓰고, 새 로봇에 맞춰 파인튜닝하도록 설계됐다.

### π0 (Physical Intelligence)

[π0 기술 보고서(2024)](https://arxiv.org/abs/2410.24164)가 제안한 범용 로봇 정책이다. 사전학습 VLM에 flow-matching action expert를 붙였다. 저자들은 single-arm·dual-arm·mobile manipulator 데이터로 학습한 뒤 zero-shot, 언어 지시, fine-tuning 조건을 평가했으며, 빨래 접기·테이블 닦기·상자 조립 등을 시연했다.

### OpenVLA

Stanford 등이 공개한 VLA (Vision-Language-Action) 모델이다. 7B 파라미터 VLM(Prismatic, Llama 2 기반)을 action token을 출력하도록 파인튜닝했고, 코드와 가중치를 공개했다.

### 실사용의 걸림돌

특정 도메인의 특정 태스크에서는 전통적 제어 기법이나 태스크 특화 학습 모델이 더 높은 성공률을 낼 때가 있다. 로봇 데이터는 로봇과 사람이 직접 움직여 모으므로, 인터넷 텍스트나 웹 이미지보다 한 건당 수집 비용이 훨씬 크다. 안전을 보장하기 어렵고, 어떤 행동을 할지 미리 내다보기도 어렵다. 7B급 모델의 추론 지연(inference latency)은 실시간 제어 주기를 넘을 수 있다.

### 연구 방향

- **Data scaling**: Open X-Embodiment처럼 여러 기관의 데이터를 합쳐, 데이터 양에 따라 일반화가 얼마나 좋아지는지 잰다.
- **Cross-embodiment transfer**: 한 로봇에서 학습한 정책을 다른 로봇에 옮긴다. 로봇마다 다른 action space를 하나로 맞추는 방법을 찾는다.
- **Efficient fine-tuning**: LoRA 같은 parameter-efficient fine-tuning으로 새 태스크에 맞춘다.
- **Action representation**: action을 이산 토큰, 연속 분포, diffusion·flow 출력 가운데 어떤 형태로 표현할지 연구한다.


---

## 8.8 심화: Reward Design과 Safe RL

reward 함수를 잘못 설계하면 RL은 설계자가 바라지 않은 행동을 학습한다. 실제 로봇에 RL을 쓸 때는 안전 제약도 함께 넣는다.

### Reward Shaping

**Sparse reward의 문제**: "목표에 도달하면 +1, 그 밖에는 0" 같은 sparse reward는 정의하기 쉽다. agent는 우연히 목표에 닿을 때까지 무작위로 탐색한다. state-action 공간이 크면 그 첫 보상을 얻기까지 오래 걸린다.

**Dense reward**: 중간 과정에도 보상을 준다. 예를 들어 물체 잡기 태스크에서:

```python
import numpy as np

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

**Curriculum learning**: 쉬운 태스크에서 시작해 점차 어려운 태스크로 넘어가는 방법이다. 예를 들어 locomotion에서 처음에는 평지에서 걷기, 다음에 작은 장애물, 그 다음 계단 순으로 난이도를 올린다. 쉬운 단계에서 먼저 보상을 얻으므로 sparse reward에서도 학습 초기에 성공 경험이 쌓인다.

### Reward Hacking

Agent가 설계자가 의도하지 않은 방식으로 reward를 최대화하는 현상이다.

예:
- 빨간 블록을 파란 블록 위에 쌓는 과제에서 빨간 블록 아랫면의 높이를 보상으로 주자, 로봇 팔이 블록을 집어 올리지 않고 뒤집기만 했다([Popov et al., 2017](https://arxiv.org/abs/1704.03073)).
- 형태와 제어기를 함께 진화시킨 가상 생물에게 평균 이동 속도를 적합도로 주자, 몸을 밀고 나갈 영리한 다리나 뱀 같은 움직임을 만들지 않고 키 크고 단단한 몸이 되어 넘어지면서 속도를 냈다(Karl Sims의 1994년 연구, [Lehman et al., Artificial Life 2020](https://arxiv.org/abs/1803.03453)에 기록).
- 점프를 배우도록 형태를 진화시키면서 처음에 가장 낮던 블록의 높이를 적합도로 주자, 긴 막대 위에 큰 블록 몇 개를 얹은 몸을 만들어 뛰지 않고 머리부터 넘어지며 공중제비를 돌았다(Krcah의 실험, 같은 글).

이런 사례는 [Krakovna의 specification gaming 목록](https://vkrakovna.wordpress.com/2018/04/02/specification-gaming-examples-in-ai/)과 [DeepMind 블로그 글(2020)](https://deepmind.google/blog/specification-gaming-the-flip-side-of-ai-ingenuity/)에 모여 있다.

대응 방법:
- 학습된 행동을 확인하면서 reward 함수를 반복해서 조정한다.
- 원치 않는 행동에 penalty term을 붙인다.
- 학습된 행동을 비디오로 보며 사람이 검토한다.

### Constrained RL

Constrained RL은 cost의 기댓값을 한도 아래로 유지한다는 조건을 걸고 reward를 최대화한다.

```
max_π E[ Σ γ^t r_t ]   subject to   E[ Σ γ^t c_t ] ≤ d
```

c_t는 cost (예: 관절 토크 한계 초과, 장애물 충돌), d는 허용 한도다.

CPO (Constrained Policy Optimization), PCPO, Lagrangian relaxation 기반 방법 등이 있다. 실제 로봇에서는 하드웨어를 보호하려고 토크 제한과 관절 각도 제한을 constraint로 넣는다.

### Human-in-the-loop RL

사람의 피드백으로 reward를 만든다. LLM의 RLHF (RL from Human Feedback)와 같은 방식이다.

방법:
1. 로봇의 행동 쌍을 보여주고 사람이 선호도를 표시한다 (A가 B보다 나음).
2. 선호도 데이터로 reward model을 학습한다.
3. 학습된 reward model로 RL을 돌린다.

걸음걸이의 자연스러움이나 물건을 놓을 때의 조심성처럼 수치로 정의하기 어려운 목표에 쓴다. 선호도를 매기는 데 사람 시간이 들고, 학습된 reward model이 틀리면 policy가 그 오류를 이용한다.


---

## 8.9 추천 자료

> **Sutton & Barto, "Reinforcement Learning: An Introduction" (2nd edition)**
> http://incompleteideas.net/book/the-book-2nd.html
> MDP부터 policy gradient까지 다루는 교재다. 무료 PDF가 있다. 기초는 Ch.1–6에 있고, policy gradient는 Ch.13에 있다.

> **Sergey Levine, CS285: Deep Reinforcement Learning**
> https://rail.eecs.berkeley.edu/deeprlcourse/
> 로봇 RL 예제가 많은 대학원 강의. 강의 영상과 슬라이드가 공개되어 있다. 이 장의 주제 대부분을 더 깊이 설명한다.

> **Stable-Baselines3**
> https://stable-baselines3.readthedocs.io/
> PyTorch 기반 RL 알고리즘 라이브러리. PPO, SAC, TD3 같은 주요 알고리즘이 들어 있어 빠른 프로토타이핑에 쓴다.

> **CleanRL**
> https://github.com/vwxyzjn/cleanrl
> 알고리즘 하나를 파일 하나에 구현한 RL 코드 모음. 알고리즘 내부를 코드로 따라 읽을 때 쓴다.

> **Isaac Lab**
> https://isaac-sim.github.io/IsaacLab/
> NVIDIA의 GPU 병렬 로봇 시뮬레이션 프레임워크. ANYmal locomotion, dexterous manipulation 예제가 들어 있다.

> **LeRobot (HuggingFace)**
> https://github.com/huggingface/lerobot
> 모방학습과 로봇 학습 프레임워크. ACT, Diffusion Policy 구현과 데이터셋이 들어 있다.

> **robomimic**
> https://robomimic.github.io/
> 모방학습 알고리즘 벤치마크. BC, BC-RNN, HBC 등을 같은 데이터와 조건에서 비교한다.

> **추가 논문**
> - [Andrychowicz et al., "Hindsight Experience Replay" (NeurIPS 2017, arXiv:1707.01495)](https://arxiv.org/abs/1707.01495) — 실패한 trajectory의 도달 지점을 목표로 바꿔 라벨을 다시 붙여 sparse reward 문제를 줄임
> - [Hafner et al., "Mastering Diverse Control Tasks through World Models" (DreamerV3, Nature 2025, arXiv:2301.04104)](https://arxiv.org/abs/2301.04104) — 하이퍼파라미터 설정 하나로 150개 이상의 태스크를 학습한 world model 기반 RL
> - [Chi et al., "Universal Manipulation Interface" (UMI, RSS 2024, arXiv:2402.10329)](https://arxiv.org/abs/2402.10329) — 핸드헬드 그리퍼로 데이터 수집, 다양한 로봇에 zero-shot 배포
> - [Fu et al., "Mobile ALOHA" (CoRL 2024, arXiv:2401.02117)](https://arxiv.org/abs/2401.02117) — 모바일 베이스 + 양팔 텔레오퍼레이션. 기존 정적 ALOHA 데이터와 co-training해 성공률을 높임


---

## 기술 흐름

```
1992 ── REINFORCE algorithm (Williams)
         Monte Carlo policy-gradient 방법. score-function 추정량의 variance가 높다.

2013 ── DQN (Mnih et al., Atari)
         CNN으로 Atari를 화면 픽셀에서 학습. replay buffer로 표본 간 상관을 줄였다.
         고정 target network는 2015년 Nature 판에서 더해졌다.

2015 ── TRPO (Schulman et al.)
         trust region 제약으로 policy update 크기를 묶음. 구현에 2차 최적화가 필요.

2017 ── PPO (Schulman et al.)
         clipping으로 TRPO의 제약을 근사. 로봇 RL 실험에서 많이 쓰는 베이스라인.

2018 ── SAC (Haarnoja et al.)
         entropy regularization + off-policy. 연속 action space용.

2019 ── ANYmal: sim-to-real locomotion (ETH Zurich)
         실제 로봇 데이터로 학습한 actuator network로 구동계 동역학을 모사해 사족보행 sim-to-real 성공.

2020 ── ANYmal teacher-student (Lee et al.)
         privileged teacher와 proprioceptive student로 험지 보행.

2022 ── RT-1 (Google)
         대규모 로봇 데이터 + Transformer. 다중 태스크 범용 정책.

2023 ── ACT/ALOHA (Stanford), Diffusion Policy (Columbia·TRI·MIT)
         action chunking으로 compounding error를 줄이고, CVAE와 diffusion으로 다봉 행동 분포를 다룸.

2023 ── RT-2 (Google)
         VLM을 직접 action 생성에 사용. 웹 지식의 로봇 전이.

2023 ── Open X-Embodiment / RT-X
         21개 기관이 22종 로봇으로 모은 데이터(527 skills)를 합쳐 cross-embodiment 학습.

2024 ── Octo, OpenVLA, π0
         여러 로봇의 데이터로 학습한 범용 정책 모델. Octo와 OpenVLA는 발표와 함께 가중치를 공개했고, π0 가중치는 2025년 2월 openpi로 공개됐다.

```
