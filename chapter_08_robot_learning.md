# 8장 로봇 러닝

6장의 제어와 7장의 모션 플래닝에서는 사람이 직접 로봇의 동역학 모델과 충돌 제약, 비용 함수를 수식으로 적어 넣었다. 모델이 정확하고 작업장이 잘 통제되어 있다면 이 고전적 해법이 여전히 가장 빠르고 믿을 만하다. 하지만 빨래를 개거나 케이블을 꽂고 어질러진 부엌을 치우는 일처럼, 물리 법칙을 일일이 수식으로 쓰기 어렵거나 환경이 시시각각 바뀌는 문제 앞에서는 사람이 모델을 짜는 방식이 금세 한계에 부딪힌다. **로봇 러닝(robot learning)**은 바로 그 빈자리를 채우기 위해 로봇이 방대한 시뮬레이션 경험과 사람의 시연 데이터에서 직접 제어 정책을 배우게 하는 분야다.

---

## 8.1 로봇 러닝이 쓰이는 곳

현장에서 학습 기반 기법을 도입할 때 가장 먼저 따져 보아야 할 질문은 "이 문제에 정말 학습이 필요한가?"다. 전통적 제어·플래닝과 학습 기반 방법에는 각자 잘 맞는 영역과 분명한 대가가 있다.

**전통적 방법이 여전히 유리한 작업**

PID 제어, MPC, RRT 같은 전통적 제어와 플래닝 기법은 로봇과 환경의 동역학 모델이 정확하고 작업 환경이 정형화되어 있을 때 압도적인 신뢰성을 보여 준다. 공장 라인에서 산업용 로봇 팔이 고정된 위치의 부품을 집어 조립하는 작업이 대표적이다. 이 기법들의 안정성과 최적성은 각 방법이 가정한 모델·제약·해법 조건 안에서 보장되므로, 그 가정이 잘 들어맞는 문제에 굳이 학습 모델을 가져다 쓰려면 계산 비용이나 검증 부담을 뛰어넘는 이점을 따로 입증해야 한다.

**전통적 방법으로 풀기 어려운 작업**

반면 다음과 같은 영역에서는 명시적인 수학 모델을 세우는 일 자체가 병목이 된다.
- **모델링이 어려운 동역학**: 천이나 로프, 액체 같은 변형체(deformable object)는 상태 차원이 사실상 무한대라 실시간 제어에 쓸 정확한 물리 모델을 세우기가 대단히 어렵다.
- **복잡한 다중 접촉(contact-rich manipulation)**: 손가락 안에서 물체를 굴려 자세를 바꾸거나 빡빡한 구멍에 부품을 끼워 맞출 때는 접촉면과 마찰 모드가 매 순간 뒤바뀐다. 복잡한 3차원 접촉 역학을 오차 없이 예측하고 푸는 문제는 아직도 미완의 과제로 남아 있다.
- **비정형·예측 불가 환경**: 가정집 부엌이나 재난 현장에서는 물체의 모양과 무게, 놓인 위치가 매번 달라지므로 사전에 모든 상황을 규칙으로 적어 넣을 수 없다.

학습 기반 방법에서는 이처럼 모델로 옮기기 까다로운 물리 상호작용을 데이터를 바탕으로 입력(센서 관측)과 출력(행동) 사이의 함수로 직접 근사한다.

**학습 기반 방법의 현실적 한계**

물론 학습이라고 해서 만능 열쇠는 아니다. 실제 로봇에 적용하려면 세 가지 높은 벽을 넘어야 한다.
- **데이터 효율(sample efficiency)**: 심층 강화학습에서는 쓸 만한 정책 하나를 배우기까지 수백만 스텝의 시행착오가 필요할 때가 잦다. 실제 로봇 하드웨어를 돌려 그만큼 데이터를 모으려면 막대한 시간과 장비 유지비가 든다.
- **안전성(safety)**: 아직 아무것도 모르는 정책이 무작위 탐색(exploration)을 하다가는 로봇 팔이 작업대를 내리치거나 주변 물건과 자기 몸체를 부술 위험이 크다.
- **일반화(generalization)**: 학습 데이터나 시뮬레이터 조건에서 조금만 벗어나도—가령 조명이 바뀌거나 바닥 마찰이 달라지기만 해도—성능이 급격히 무너지기 쉽다.

그래서 실무에서는 전통적 방법으로 풀리는 부분은 고전 제어·플래닝에 맡기고, 모델링이 막히는 지각·접촉·의사결정 부분에 학습을 결합하는 전략을 택한다.

불확실성이 얽힌 순차 의사결정이 실제 로봇 문제에서 어떤 모습으로 나타나는지 감을 잡기 위해, *Probabilistic Robotics*(Thrun·Burgard·Fox, 1999–2000년 초고 15장)에서 든 네 가지 고전 사례를 짚어 보자.
- 매니퓰레이터는 컨베이어 벨트에 무작위 자세로 실려 오는 부품을 집어 조립할 때, 부품의 정확한 자세를 모르는 채로 어떤 파지 행동을 취해야 할지 정해야 한다.
- 북극 얼음 밑을 지나는 수중 차량은 길을 잃을 위험을 감수하고 최단 경로로 질러갈지, 시간이 더 걸리더라도 특징이 없는 빈 구역을 피해 안전하게 돌아갈지 저울질해야 한다.
- 숲속을 자율 비행하는 헬리콥터는 예측할 수 없는 돌풍이 불어닥쳐도 나무에 부딪히지 않으려면 장애물에서 얼마나 여유를 두고 날아야 하는지 정해야 한다.
- 미지의 행성을 탐사하는 로봇 팀은 서로의 초기 상대 위치를 모를 때, 먼저 서로를 찾아 위치를 맞출지 아니면 각자 흩어져 더 넓은 지형을 탐사할지 결정해야 한다.

이처럼 미래의 불확실한 결과까지 내다보며 지금 가장 현명한 행동을 고르는 문제는 다음 절의 **MDP와 강화학습**이라는 수학적 틀로 정식화한다.

---

## 8.2 강화학습 기초

### MDP (Markov Decision Process)

강화학습(Reinforcement Learning, RL)은 시행착오를 거듭하며 보상을 최대화하는 행동을 배우는 틀이며, 그 수학적 무대로 **마르코프 결정 과정(MDP)**을 사용한다. MDP는 로봇과 환경의 상호작용을 다섯 가지 요소로 깔끔하게 정의한다.

- **State ($s$)**: 의사결정에 필요한 환경과 로봇의 현재 상태(관절 각도·속도, 물체 위치 등).
- **Action ($a$)**: 에이전트가 환경에 가하는 행동(관절 토크, 목표 관절 각도·속도 등).
- **Reward ($r$)**: 현재 상태에서 행동을 취한 대가로 환경에서 받는 스칼라 보상 신호 $r = R(s, a)$.
- **Transition ($T$)**: 현재 상태 $s$에서 행동 $a$를 했을 때 다음 상태 $s'$로 옮겨 갈 확률 분포 $T(s' \mid s, a)$.
- **Discount factor ($\gamma$)**: 먼 미래의 보상을 현재 가치로 얼마나 할인해서 볼지 정하는 계수($0 < \gamma \leq 1$). 로봇 RL에서는 보통 $\gamma = 0.99$를 기본 출발점으로 삼는다.

에이전트의 목표는 지금 당장의 점수만 챙기는 것이 아니라, 앞으로 받을 **누적 할인 보상(cumulative discounted reward)**의 기댓값을 최대로 만드는 정책 $\pi(a \mid s)$를 찾는 일이다.

```
J(π) = E[ Σ_{t=0}^{∞} γ^t · r_t ]
```

이 틀이 성립하려면 "미래의 다음 상태는 오직 현재 상태와 현재 행동에만 의존하며, 과거 이력은 추가로 영향을 미치지 않는다"는 **Markov 성질**이 만족되어야 한다. 즉 현재 상태 $s_t$가 과거의 모든 정보를 빠짐없이 요약하고 있다는 가정이다.

하지만 실제 로봇 현장에서는 센서 시야가 가려지거나 노이즈가 끼는 탓에 현재 관측이 곧 완전한 상태가 되지 못하는 **부분 관측(POMDP)** 상황이 흔하다. 이때도 눈에 보이지 않는 잠재 상태의 물리 전이는 Markov 성질을 따르지만, 로봇이 손에 쥔 관측값만으로는 상태를 온전히 복원할 정보가 모자란다. 그래서 실전 로봇 학습에서는 최근 여러 스텝의 관측 이력(observation history)을 쌓아 상태처럼 쓰거나, 필터로 추정한 belief를 입력으로 넣거나, LSTM·Transformer 같은 순환·시퀀스 정책을 사용해 이 빈틈을 메운다.

### MDP Value Iteration

실전 강화학습에서는 보통 환경의 전이 확률을 모르는 상태에서 출발한다. 그런데 7장 §7.9에서 잠깐 언급했듯, 만약 환경 모델 $p(x' \mid x, a)$와 보상 $r$을 정확히 알고 있다면 어떨까? 이때는 무작정 부딪혀 보며 배울 필요 없이 **동적 계획법(Dynamic Programming)**으로 최적 정책을 곧장 계산해 낼 수 있다.

여기서는 *Probabilistic Robotics* 표기를 따라 상태를 $x$, 전이 확률을 $p(x' \mid x, a)$로 적는다(앞의 $s$, $T(s' \mid s, a)$와 같은 대상이다). 또한 이 절에서 $T$는 전이 함수가 아니라 계획할 시간 길이인 **horizon**을 뜻한다.

#### Payoff와 Horizon

상태 $x$에서 행동 $a$를 택할 때 즉시 얻는 보상 $r(x, a)$를 payoff 함수라 부른다. 모든 상태에 행동을 지정하는 결정론적 정책 $\pi: x \mapsto a$의 품질은, 시작 상태 $x_0$에서 출발해 $T$ 스텝 동안 정책 $\pi$를 따랐을 때 기대되는 누적 할인 보상으로 잰다.

$$V^\pi(x_0) = \mathbb{E}\left[\sum_{t=0}^{T} \gamma^t \, r(x_t, \pi(x_t))\right]$$

계획 기간인 horizon $T$를 어떻게 두느냐에 따라 문제의 성격이 세 가지로 갈린다.

- **$T=1$ (탐욕적, greedy)**: 바로 다음 한 스텝의 보상만 최대로 만든다. 계산은 단순하지만 당장 눈앞의 작은 보상을 쫓느라 그다음 스텝에 함정에 빠질 위험을 살피지 못한다.
- **유한 horizon ($T < \infty$)**: 정해진 $T$ 스텝 끝까지만 최적화한다. 남은 스텝 수에 따라 최적 행동이 달라지므로 시간 $t$에 의존하는 정책(time-dependent policy)을 따로 구해야 해서 표현이 번거롭다.
- **무한 horizon ($T = \infty$)**: $\gamma < 1$이고 보상이 유계($|r(x, a)| \leq r_{\max}$)라면 무한히 더해도 가치 함수는 $|V^\pi| \leq r_{\max}/(1-\gamma)$ 안쪽으로 유한하게 수렴한다. 게다가 앞으로 남은 시간이 늘 무한대이므로 시간에 따라 정책을 바꿀 필요가 없어, 시간에 무관한 **정상 정책(stationary policy)** 가운데 최적 정책이 반드시 존재한다. 그래서 로봇 RL에서는 무한 horizon과 할인율 $\gamma < 1$ 조합을 표준으로 삼는다.

할인 인자 $\gamma$의 직관도 짚고 넘어가자. 한 스텝 뒤의 보상은 $\gamma$배, 두 스텝 뒤의 보상은 $\gamma^2$배로 가치가 줄어든다. $\gamma$를 작게 잡으면 근시안적으로 당장 가까운 보상 위주로 움직이고, 1에 가깝게 올리면 먼 미래의 보상까지 길게 내다본다. 흔히 쓰는 $\gamma = 0.99$는 실용적인 균형점이다. 대략 $1/(1-\gamma) = 100$ 스텝 앞의 미래까지 비중 있게 고려하면서도, $\gamma < 1$ 조건 덕분에 수학적 수렴도 보장되기 때문이다.

#### Bellman 방정식

무한 horizon에서 가능한 모든 정책 가운데 가장 높은 가치를 내는 최적 가치 함수 $V^*(x)$는 다음과 같은 **Bellman 최적 방정식**을 만족한다.

$$V^*(x) = \max_a \left[ r(x, a) + \gamma \sum_{x'} p(x' \mid x, a) \, V^*(x') \right]$$

이 식의 뜻은 직관적이다. 상태 $x$의 최적 가치는 "지금 행동 $a$를 택해 얻는 즉시 보상 $r(x,a)$"에 "전이될 다음 상태 $x'$들의 최적 가치를 확률로 평균 내고 $\gamma$를 곱해 할인한 값"을 더했을 때, 그 합이 가장 큰 행동 $a$의 값과 일치한다. 양변에 똑같이 미지수 $V^*$가 들어 있는 재귀식이며, $T=1$의 최적 가치에서 출발해 $T=2, 3, \dots$로 한 스텝씩 거슬러 올라가 $T \to \infty$ 극한을 취하면 자연스럽게 이 식에 도달한다.

일단 모든 상태의 최적 가치 $V^*$만 손에 넣으면, 최적 정책 $\pi^*(x)$는 각 상태에서 우변의 괄호 안 값을 가장 크게 만드는 행동을 고르는 것만으로 구해진다.

$$\pi^*(x) = \arg\max_a \left[ r(x, a) + \gamma \sum_{x'} p(x' \mid x, a) \, V^*(x') \right]$$

#### Value Iteration 알고리즘

Bellman 방정식은 $V^*$를 자기 자신으로 되돌리는 고정점 방정식(fixed-point equation)이다. **가치 반복(Value Iteration)** 알고리즘은 이 성질을 그대로 이용한다. 아무 초기 추정값 $V_0$(보통 0)에서 시작해 방정식 우변에 현재 추정치 $V_k$를 대입하고, 그 결과를 새 추정치 $V_{k+1}$로 삼는다. 값이 더는 변하지 않을 때까지 이 계산을 되풀이한다. 직관적으로 보면, 반복을 거듭할수록 목표 지점의 높은 보상이 한 스텝씩 바깥 상태로 물결처럼 퍼져 나간다.

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

여기서 $V_k$는 $k$번째 반복의 가치 추정값, $V^*$는 수렴 후 최적 가치, $\pi^*$는 최적 정책, $r$은 보상, $\gamma$는 할인 인자, $p(x'\mid x,a)$는 전이 확률을 뜻한다. 위 식은 이산 상태 공간을 가정해 합($\Sigma$)으로 적었으며, 연속 상태 공간에서는 합 자리에 적분이 들어간다.

놀라운 점은 상태들을 어떤 순서로 갱신하더라도(Sutton & Barto, 2018 §4.5에 따라 모든 상태가 무한히 자주 갱신되기만 하면) 반드시 유일한 해로 수렴한다는 사실이다. Bellman 갱신 연산자가 수축률(contraction rate)이 $\gamma$인 수축 사상(contraction mapping)이기 때문이며, $k$번 반복한 뒤 오차는 초기 오차의 $\gamma^k$배 이하로 기하급수적으로 줄어든다: $\|V_k - V^*\|_\infty \leq \gamma^k \|V_0 - V^*\|_\infty$.

<!-- DEMO: mdp_value_iteration_grid.html -->

#### 2D Grid World 예시

가치 값이 어떻게 퍼지고 장애물을 피하는 정책이 만들어지는지 $5 \times 5$ 격자 세계로 확인해 보자. 각 셀이 상태 $x$이고 로봇은 상하좌우 4방향으로 움직인다. 목표 셀에 닿으면 $r = +100$, 장애물 셀에 부딪히면 $r = -10$, 빈 칸을 지날 때마다 이동 비용으로 $r = -1$을 받는다. 이때 움직임에는 확률적 잡음이 있어서 의도한 방향으로 갈 확률이 0.8, 엉뚱한 양옆 직교 방향으로 미끄러질 확률이 각각 0.1이다.

Value Iteration을 실행하면 첫 번째 반복에서는 목표 바로 옆 셀들만 큰 값을 얻고, 반복을 한 번씩 거듭할 때마다 그 높은 가치가 한 칸씩 바깥 셀로 퍼져 나간다. 결국 가치 함수는 목표 셀 근처에서 봉우리처럼 솟고 장애물 근처에서 움푹 꺼진 지형을 이룬다. 수렴 뒤 각 셀에서 기대 가치를 최대화하는 행동을 뽑아내면, 로봇이 미끄러져 장애물에 부딪힐 위험까지 계산에 넣어 여유 있게 돌아가는 최적 정책 $\pi^*$가 얻어진다. 모든 셀에 최적 행동이 미리 정해져 있으므로 실행 중에는 RRT나 A* 같은 경로 탐색을 다시 돌릴 필요가 없으며, 전이가 확률적이어서 실제 이동 궤적은 실행할 때마다 조금씩 달라진다.

이처럼 Value Iteration은 환경 모델 $p(x'\mid x,a)$와 보상 $r$을 정확히 알 때 완벽한 답을 내놓는다. 그러나 실제 로봇 문제에서는 전이 확률을 미리 적어 넣기 어려운 상황이 훨씬 많다. 그래서 이제 환경 모델 없이 실제 상호작용 경험만으로 정책을 직접 개선하는 **model-free 강화학습**으로 넘어간다(부분 관측 상황의 모델 기반 플래닝은 7장 §7.9에서 다루었다).

### Policy Gradient 직관

환경 모델을 모른다면, 아예 파라미터 $\theta$로 표현된 신경망 정책 $\pi_\theta(a \mid s)$를 두고 직접 경험을 모아가며 조금씩 고치는 방법을 쓸 수 있다. **Policy Gradient**는 다음 세 단계를 되풀이하며 정책을 개선한다.

1. 현재 정책 $\pi_\theta$로 로봇을 움직여 여러 개의 궤적(trajectory)을 수집한다.
2. 총 누적 보상(return)이 높았던 궤적에 속한 행동들의 선택 확률을 끌어올린다.
3. 반대로 return이 낮았던 궤적의 행동들은 선택 확률을 끌어내린다.

이를 목적 함수 $J(\theta)$의 기울기(gradient) 수식으로 적으면 다음과 같다.

```
∇J(θ) = E[ Σ_t ∇log π_θ(a_t|s_t) · A_t ]
```

여기서 $\nabla\log \pi_\theta(a_t \mid s_t)$는 상태 $s_t$에서 행동 $a_t$를 고를 확률이 가장 빠르게 커지는 방향을 가리키고, 여기에 곱해진 **어드밴티지(advantage)** $A_t$는 그 행동이 해당 상태에서 평균적으로 기대되는 성적보다 얼마나 더 좋았는지를 나타내는 점수다. 따라서 평균보다 잘한 행동($A_t > 0$) 쪽으로 파라미터 $\theta$가 당겨지고, 평균보다 못한 행동($A_t < 0$) 쪽에서는 밀려난다.

### Value Function, Q-function

Policy Gradient를 쓸 때 단순히 에피소드의 총 보상만 곱하면 궤적마다 우연히 낀 노이즈 때문에 기울기 추정의 분산(variance)이 너무 커져 학습이 들쭉날쭉해진다. 이를 안정시키려면 "이 상태는 원래 얼마나 좋은 상태인가"라는 기준선(baseline)을 빼 주어, 순수하게 행동의 상대적 우수성인 advantage만 남겨야 한다. 이를 위해 정책과 함께 학습하는 세 가지 가치 함수가 등장한다.

- **Value function $V^\pi(s)$**: 상태 $s$에서 출발해 정책 $\pi$를 끝까지 따랐을 때 기대되는 누적 보상.
- **Q-function $Q^\pi(s, a)$**: 상태 $s$에서 먼저 특정 행동 $a$를 취한 뒤, 그다음부터 정책 $\pi$를 따랐을 때 기대되는 누적 보상.
- **Advantage $A^\pi(s, a) = Q^\pi(s, a) - V^\pi(s)$**: 상태 $s$의 평균 기대 가치($V$)와 비교했을 때 행동 $a$를 고른 것이 얼마나 이득이었는지 나타내는 지표.

이렇게 행동을 고르는 **Policy network(Actor)**와 그 행동의 가치를 채점하는 **Value network(Critic)**를 함께 학습하는 구조를 **Actor-Critic**이라 부르며, 다음 절에서 만날 PPO, SAC, TD3가 모두 이 골격을 따른다.

### On-policy vs Off-policy

강화학습 알고리즘을 고를 때 가장 먼저 부딪히는 갈림길은 "수집한 데이터를 한 번 쓰고 버릴 것인가, 저장해 두고 계속 재사용할 것인가"다.

- **On-policy**: 지금 학습 중인 최신 정책이 방금 수집한 데이터만 사용해 학습한다. 한 번 데이터를 모아 몇 epoch 업데이트하고 나면 정책이 바뀌었으므로 기존 데이터를 미련 없이 버린다. **PPO**가 대표적이며 학습이 안정적이고 구현이 단순한 대신, 같은 성능에 도달하는 데 필요한 데이터 양이 많아 데이터 효율(sample efficiency)이 낮다.
- **Off-policy**: 과거 버전의 정책들이 모아 둔 경험까지 모두 **replay buffer**라는 거대한 저장소에 쌓아 두고 무작위로 꺼내 재사용한다. **SAC**와 **TD3**가 여기에 속하며, 데이터를 알뜰하게 써서 sample efficiency가 높은 대신 가치 함수 오차와 분포 불일치 때문에 하이퍼파라미터에 민감하고 학습이 흔들릴 수 있다.

그래서 실제 로봇 하드웨어에서 데이터를 직접 모아야 한다면 수집 비용이 비싸기 때문에 **off-policy** 기법이 유리하고, 반대로 GPU 시뮬레이터 위에서 수천 대의 로봇을 동시에 돌릴 수 있다면 데이터를 공짜로 쏟아부으며 안정적인 **on-policy** 기법을 밀어붙이는 쪽이 실무에서 더 잘 통한다.

---

## 8.3 주요 RL 알고리즘

앞서 본 Policy Gradient와 Actor-Critic 원리를 실제 로봇 제어에 바로 적용하려면 학습이 갑자기 망가지지 않도록 잡아 주는 안전장치가 필요하다. 현재 로봇 러닝 연구와 실무에서 사실상의 표준으로 자리 잡은 세 가지 대표 알고리즘을 살펴보자.

### PPO (Proximal Policy Optimization)

Policy Gradient에서 한 번 업데이트할 때 파라미터를 너무 크게 바꾸면 정책이 엉뚱한 방향으로 꺾여 버리고, 그렇게 망가진 정책이 다음번에 형편없는 데이터만 모아 오면서 학습이 영영 회복되지 않는 일이 잦다. Schulman et al.(2017)이 제안한 **PPO**는 이 문제를 막기 위해 이전 정책 대비 새 정책의 확률 비율을 일정 구간 안에 가두는(clipping) on-policy 알고리즘이다. 전신인 TRPO가 복잡한 2차 미분과 KL 발산 신뢰 영역 제약으로 걸었던 안전장치를, PPO는 단순한 목적 함수 클리핑으로 대체했다.

```
L_CLIP(θ) = E[ min( r_t(θ) · A_t, clip(r_t(θ), 1-ε, 1+ε) · A_t ) ]
```

여기서 $r_t(\theta) = \pi_\theta(a_t \mid s_t) / \pi_{\theta_{\text{old}}}(a_t \mid s_t)$는 확률 비율이며(원 논문의 기호를 따랐고, §8.2의 보상 $r_t$와는 다른 양이다), $\varepsilon$의 원 논문 기본값은 $0.2$다. 이 식 덕분에 Advantage $A_t$가 양수일 때는 $r_t$가 $1+\varepsilon$을 넘어 무한정 커져도 이득이 더 늘어나지 않고, 반대로 음수일 때는 $1-\varepsilon$ 아래로 떨어져도 더 깎이지 않아 정책이 한 번에 과격하게 변하는 일을 막아 준다.

PPO는 구현이 간결하고 기본 하이퍼파라미터만으로도 다양한 문제에서 무난히 수렴한다. 흥미롭게도 Engstrom et al.(2020)은 value function clipping, reward scaling, 가중치 초기화 같은 자잘한 구현 디테일을 하나씩 켜고 끄는 실험을 거쳐, PPO가 TRPO보다 높은 점수를 낸 비결의 상당 부분이 클리핑 자체보다 이러한 코드 수준 최적화에 있었다고 보고했다. 한편 NVIDIA Isaac Lab 같은 GPU 병렬 시뮬레이터에서 수천 개 환경을 한꺼번에 돌릴 수 있는 상황이라면, 데이터 수집 속도가 워낙 빨라 on-policy 특유의 낮은 sample efficiency가 큰 문제가 되지 않으므로 PPO가 가장 먼저 선택된다.

### SAC (Soft Actor-Critic)

실제 로봇 팔이나 이동 로봇처럼 데이터를 모으는 데 비용이 많이 들 때는 과거 경험을 replay buffer에 담아 두었다가 여러 번 꺼내 쓰는 off-policy 알고리즘이 필요하다. **SAC**는 여기에 **엔트로피 정규화(entropy regularization)**를 결합한 알고리즘이다. 에이전트가 보상의 합만 쫓지 않고, 정책의 확률 분포가 얼마나 다양한 행동을 골고루 시도하는지(엔트로피 $H$)까지 함께 최대화한다.

```
J(π) = E[ Σ_t γ^t ( r_t + α · H(π(·|s_t)) ) ]
```

온도 파라미터 $\alpha$는 보상과 탐색(엔트로피) 사이의 비중을 조절하며, 목표 엔트로피를 미리 정해 두면 학습 도중 $\alpha$를 자동으로 맞춰 주는 변형도 널리 쓰인다(Haarnoja et al., 2018b). SAC는 한 가지 해법에 일찍 갇히지 않고 여러 가지 행동 방식을 폭넓게 시험하면서 동시에 replay buffer의 데이터를 알뜰하게 재사용한다. 이 덕분에 연속 행동 공간에서 실제 로봇으로 데이터를 직접 모아야 할 때 PPO보다 SAC를 훨씬 자주 선택한다.

### TD3 (Twin Delayed DDPG)

**TD3**는 초기 연속 제어용 off-policy 알고리즘인 DDPG가 겪던 Q값 과대추정과 학습 불안정 문제를 세 가지 장치로 고친 알고리즘이다.

1. **Twin Q-networks**: 서로 독립적으로 초기화한 Q-function 두 개를 동시에 학습하되, 타깃 값을 계산할 때는 둘 중 더 작은 값을 취해 낙관적 오차가 눈덩이처럼 커지는 과대추정 편향(overestimation bias)을 억제한다.
2. **Delayed policy update**: 가치 함수를 평가하는 critic을 충분히 안정될 때까지 여러 번 업데이트한 뒤에야 정책(actor)을 한 번씩 업데이트한다.
3. **Target policy smoothing**: 타깃 Q값을 구할 때 행동에 작은 노이즈를 섞어 주어, Q함수의 뾰족한 극댓값 하나에 정책이 과적합되는 현상을 누그러뜨린다.

SAC와 달리 엔트로피 항이 없어 손댈 하이퍼파라미터가 조금 더 적고, 탐색은 주로 행동 출력에 가우시안 노이즈를 더하는 단순한 방식으로 처리한다.

### 알고리즘 선택 가이드

프로젝트 조건에 따라 어떤 알고리즘부터 잡아야 할지 정리하면 다음과 같다.

| 상황 | 추천 알고리즘 | 이유 |
|------|-------------|------|
| 시뮬레이션, GPU 병렬화 가능 | PPO | 병렬 환경으로 데이터를 많이 모아 낮은 sample efficiency를 보완함 |
| 실제 로봇, 데이터 적음 | SAC | off-policy, sample efficient |
| 연속 action space | SAC 또는 TD3 | 둘 다 연속 action space용으로 설계됨 |
| 이산 action space | PPO 또는 DQN | SAC를 이산 행동에 쓰려면 별도 변형을 구현해야 함 |
| 처음 시작하는 프로젝트 | PPO | 기본값으로 학습이 무난히 돌아 디버깅이 쉬움 |

### Stable-Baselines3 코드 예시

이 알고리즘들을 바닥부터 직접 구현하지 않더라도 검증된 오픈소스 라이브러리를 쓰면 곧바로 실험을 돌릴 수 있다. 아래는 Stable-Baselines3로 MuJoCo의 `Ant-v5` 보행 환경을 8개 병렬로 띄워 PPO로 200만 스텝 학습하고 평가하는 기본 코드다.

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

SAC로 바꿀 때도 호출 구조는 거의 같으며, rollout 길이 대신 `buffer_size`와 `learning_starts`, target network soft update 계수 `tau`를 지정한다는 점만 다르다.

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

> 빠른 프로토타이핑과 벤치마크 확인에는 **Stable-Baselines3**가 편리하다. 반면 알고리즘 내부의 손실 함수나 정규화 디테일을 직접 뜯어보고 고쳐야 할 때는 알고리즘 하나를 단일 파일로 명쾌하게 구현해 둔 **CleanRL** 코드를 따라 읽으면 동작 원리가 한눈에 들어온다.

---

## 8.4 시뮬레이션 환경

위 코드에서 200만 스텝을 학습하는 모습을 보았듯, 강화학습에는 엄청난 양의 시행착오가 필요하다. 실제 로봇 하드웨어로 수백만 스텝을 모으려면 수개월의 시간과 부서진 모터를 갈아 끼울 막대한 비용이 들기 때문에, 현대 로봇 RL에서는 대부분 **물리 시뮬레이터** 안에서 먼저 정책을 훈련한다. 이때 연구 목적과 계산 자원에 따라 어떤 시뮬레이터를 고르느냐가 실험 속도와 성패를 가른다.

### MuJoCo (Multi-Joint dynamics with Contact)

DeepMind가 인수해 2021년에 무료로 전환하고 2022년에 오픈소스로 공개한 물리 엔진이다. 다관절 강체 동역학과 접촉 시뮬레이션의 수치 적분이 매우 안정적이어서, Gymnasium의 표준 연속 제어 벤치마크(Ant, Humanoid 등)가 모두 MuJoCo 위에서 돌아간다. 전통적인 엔진은 CPU에서 실행되지만 MuJoCo 3.0부터는 JAX 백엔드 기반의 MJX가 추가되어 GPU 대규모 병렬화까지 지원한다. 알고리즘 아이디어를 검증하거나 중소규모 조작·보행 실험을 돌릴 때 가장 무난한 출발점이다.

### Isaac Lab (NVIDIA)

NVIDIA Isaac Sim 위에서 동작하는 대규모 로봇 학습 프레임워크다. 물리 연산과 렌더링을 통째로 GPU에 올려 수천에서 수만 개의 로봇 환경을 동시에 시뮬레이션하므로, 1년 치 보행 경험을 몇 분 만에 모아 학습할 수 있다. 사실적인 카메라·LiDAR 센서 시뮬레이션까지 함께 제공해 대규모 locomotion 연구와 sim-to-real 파이프라인의 주력으로 자리 잡았다. 다만 NVIDIA GPU가 필수이고 설치·설정 장벽이 다소 높다.

### PyBullet

`pip install` 한 줄이면 바로 설치되는 가벼운 오픈소스 물리 엔진이다. GPU 병렬화는 지원하지 않고 속도도 느린 편이지만 진입 장벽이 낮아, 처음 강화학습 코드를 돌려 보거나 로봇 URDF 모델을 불러와 동작을 빠르게 확인할 때 유용하다.

### Brax

Google이 개발한 JAX 기반 물리 엔진이다. JAX의 JIT 컴파일 덕분에 GPU와 TPU 위에서 매우 빠르게 돌아가며, 시뮬레이션 전체를 자동 미분할 수 있어 differentiable physics(미분 가능한 물리) 연구에도 활용된다. 다만 계산 속도를 끌어올리기 위해 접촉 모델을 단순화했으므로 복잡한 접촉이 얽힌 조작 작업에서는 MuJoCo보다 물리 정확도가 떨어진다.

### 환경 비교표

네 가지 시뮬레이터의 강점과 제약을 한눈에 비교하면 다음과 같다.

| 시뮬레이터 | 물리 정확도 | 속도 | GPU 병렬화 | 설치 난이도 | 주 용도 |
|-----------|-----------|------|-----------|-----------|---------|
| MuJoCo | 높음 | 보통 | MJX로 가능 | 쉬움 | 알고리즘 벤치마크 |
| Isaac Lab | 높음 | 매우 빠름 | 수천~만 | 어려움 | 대규모 로봇 학습 |
| PyBullet | 보통 | 느림 | 불가 | 매우 쉬움 | 입문/교육 |
| Brax | 낮음 | 매우 빠름 | 가능 | 보통 | 빠른 반복 실험 |

일반적으로는 설치가 간편하고 물리가 정확한 MuJoCo + Gymnasium 조합으로 알고리즘을 검증한 뒤, 수천 대의 병렬 환경과 렌더링이 필요한 대규모 실험 단계에서 Isaac Lab으로 넘어가는 흐름이 매끄럽다.

---

## 8.5 Sim-to-Real Transfer

시뮬레이터 안에서 수백만 스텝을 돌려 완벽하게 걷거나 물건을 집는 정책을 얻었다고 곧바로 끝이 아니다. 시뮬레이션에서 훈련한 정책을 실제 물리 로봇에 그대로 올려 실행하는 과정을 **Sim-to-Real transfer**라 부르며, 이 문턱을 넘지 못하면 시뮬레이션 점수는 아무 소용이 없다.

### Reality Gap

시뮬레이션의 높은 점수가 현실로 그대로 이어지지 않는 이유는 시뮬레이터와 물리 세계 사이의 **현실 간극(reality gap)** 때문이다.

- **물리 파라미터 차이**: 실제 바닥의 마찰 계수, 링크의 질량과 관성 모멘트, 무게중심이 시뮬레이터에 적은 수치와 미세하게 다르다.
- **센서 노이즈와 지연**: 실제 카메라나 IMU, 관절 엔코더에는 측정 노이즈와 통신 지연(latency), 시간 경과에 따른 드리프트가 섞여 들어간다.
- **액추에이터 모델링 오차**: 실제 모터의 비선형 토크 특성, 기어 백래시(backlash), 케이블과 감속기의 탄성(compliance)은 이상적인 강체 모델에 다 담기지 않는다.
- **접촉 모델의 근사**: 시뮬레이터의 접촉·충돌 계산은 속도를 위해 현실의 복잡한 탄성 변형과 마찰을 단순화한다.

이 작은 차이들이 제어 루프 안에서 누적되면, 시뮬레이터에서 한 번도 넘어지지 않던 보행 정책이 실제 바닥에 발을 딛자마자 중심을 잃고 쓰러져 버린다.

### Domain Randomization

현실 간극을 메우는 가장 강력한 방법 중 하나가 **도메인 무작위화(Domain Randomization)**다. 시뮬레이터의 물리 파라미터를 실제 값과 똑같이 맞추려 애쓰는 대신, 아예 학습하는 동안 마찰 계수·질량·모터 힘·센서 노이즈를 매 에피소드 무작위로 바꿔 가며 훈련하는 전략이다. 이렇게 하면 정책이 특정 시뮬레이션 설정 하나에 과적합되지 않고, 파라미터가 꽤 넓은 범위 안에서 흔들려도 버텨 내는 강건한 제어 법칙을 배우게 된다. OpenAI의 **Dactyl**(IJRR 2020, arXiv 2018)은 물리 파라미터와 시각적 요소(조명·질감)를 함께 무작위화해, 시뮬레이션에서만 학습한 정책으로 실제 다지 로봇 손(Shadow Hand)이 큐브를 돌리는 고난도 작업에 성공했다.

현장에서 주로 흔드는 파라미터 범위와 스텝별 교란은 다음과 같다.
- 마찰 계수: 0.5 ~ 1.5 사이 균등 분포 샘플링
- 물체 및 링크 질량: 기본값의 0.8 ~ 1.2배
- 관절 댐핑(damping): 기본값의 0.5 ~ 2.0배
- 센서 노이즈: 매 스텝 관측값에 가우시안 노이즈 추가
- 액추에이터 강도(strength): 기본값의 0.8 ~ 1.2배
- 통신 지연: 0 ~ 2 스텝의 랜덤 행동 지연

아래는 Isaac Lab 스타일의 환경에서 에피소드마다 물성을 바꾸고 매 스텝 관측 노이즈를 섞는 무작위화 설정의 의사 코드다.

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

물론 무작위화 범위를 무한정 넓힐 수는 없다. 범위를 넓힐수록 실제 로봇의 물리 파라미터가 그 안에 들어갈 확률은 높아지지만, 그 대가로 정책이 모든 상황에 두루 맞추려 들면서 보수적으로 변해 특정 환경에서의 최고 성능은 다소 떨어진다. 그럼에도 실제 환경의 정확한 파라미터를 알기 어려운 현실에서는, 단일 설정에 맞춘 정책보다 무작위화로 단련된 정책이 실제 로봇 위에서 훨씬 안정적으로 살아남는다.

### System Identification (Sys-ID)

무작위화 범위를 무턱대고 키우는 대신 시뮬레이터 자체를 현실에 바짝 당겨 오는 접근법이 **시스템 식별(System Identification, Sys-ID)**이다. 실제 로봇의 물리 파라미터를 직접 측정하거나 실험 데이터로부터 역추정해 시뮬레이션 모델에 반영한다.

- **직접 측정**: 전자저울로 링크와 탑재물의 질량을 재거나 마찰 실험으로 마찰 계수를 구한다.
- **파라미터 최적화**: 시뮬레이터와 실제 로봇에 똑같은 제어 입력을 넣고, 두 궤적 사이의 오차가 최소화되도록 물리 파라미터를 최적화한다.
- **온라인 적응**: 로봇이 움직이는 도중에도, 변화하는 파라미터(예: 방금 집어 올린 물체의 무게)를 관측 데이터를 바탕으로 실시간으로 추정해 보정한다.

실전에서는 Sys-ID와 Domain Randomization을 서로 대립하는 기법으로 보지 않고 짝지어 쓴다. Sys-ID로 현실에 가장 가까운 중심 파라미터 값을 맞춘 뒤, 여전히 남는 오차와 환경 변화를 감싸도록 그 주변 좁은 범위에 Domain Randomization을 얹는 방식이다.

### Teacher-Student 구조

시뮬레이터에는 실제 로봇이 절대 가질 수 없는 강력한 특권이 하나 있다. 바로 바닥의 정확한 마찰 계수, 로봇 앞 지형의 정확한 높이 지도, 물체의 실제 위치 같은 **특권 정보(privileged information)**를 공짜로 읽을 수 있다는 점이다. **Teacher-Student** 구조에서는 이 특권을 두 단계 학습으로 영리하게 활용한다.

1. **Teacher 학습**: 먼저 시뮬레이션 안에서 privileged information까지 모두 상태 입력으로 넣어 교사 정책(Teacher)을 강화학습으로 빠르고 정밀하게 훈련한다. 정답에 가까운 물리 정보를 보고 움직이므로 험지 보행 같은 어려운 기술도 금방 터득한다.
2. **Student 학습**: 이제 특권 정보를 떼어 내고 학생 정책(Student)을 둔다. 그런 다음 Student가 실제 로봇에 달린 센서(IMU, 관절 엔코더, 카메라 등)의 측정 이력만 입력으로 받아 Teacher가 내는 것과 똑같은 행동을 출력하도록 지도학습으로 증류(distillation)한다.

ETH Zurich의 Lee et al.(2020)은 이 구조로 사족보행 로봇 ANYmal의 험지 보행 정책을 학습했다. Teacher가 정확한 지형 높이를 보고 발을 디딜 곳을 배웠다면, Student는 오직 관절·IMU 측정의 시간 이력(proprioception history)만으로도 발끝이 돌에 걸리거나 미끄러지는 느낌을 역추정해 Teacher처럼 험지를 헤쳐 나갔다.

### 실제 사례

이러한 Sim-to-Real 기법들이 실제로 어떤 수준까지 도달했는지 보여 주는 두 가지 이정표가 있다.

**ANYmal 보행 (ETH Zurich / Robotic Systems Lab)**
- 강화학습 + domain randomization + teacher-student 결합으로 사족보행 학습 (Lee et al., Science Robotics 2020)
- 실제 환경 파인튜닝 없이 시뮬레이션에서만 학습한 정책을 실제 로봇에 곧바로 배포(zero-shot transfer)
- 시뮬레이션에서 한 번도 보지 못한 진흙, 눈, 무너진 잔해, 우거진 수풀, 흐르는 계곡물 위를 안정적으로 보행

**Dexterous Hand Manipulation (OpenAI)**
- 사람 손과 비슷한 24자유도 Shadow Hand로 루빅스 큐브(Rubik's cube)를 한 손으로 돌려서 맞추기 (OpenAI, 2019)
- 학습 진행도에 맞춰 무작위화 난이도를 자동으로 높여 가는 **Automatic Domain Randomization(ADR)** 도입
- 시뮬레이션에서 약 13,000년 분량의 상호작용 경험을 압축 학습
- 실제 로봇 테스트에서 가장 어려운 스크램블(26수) 기준 20%, 15수 스크램블 기준 60%의 성공률 달성

---

## 8.6 모방학습

강화학습은 보상을 스스로 찾아가는 강력한 도구이지만, 현실의 조작(manipulation) 작업에서는 셔츠 개기나 머그컵 뒤집기처럼 보상 함수를 깔끔하게 적기 어렵고 물리 시뮬레이터로 정교하게 옮기기도 까다로운 과제가 수두룩하다. 이때 보상을 설계해 시행착오를 거치게 하는 대신, 사람(전문가)이 직접 시연한 데이터에서 곧장 정책을 배우는 방법이 **모방학습(Imitation Learning)**이다.

### Behavioral Cloning (BC)

모방학습의 출발점은 가장 단순한 형태인 **행동 복제(Behavioral Cloning, BC)**다. 사람이 로봇을 조종해 `(observation, action)` 쌍을 잔뜩 모아 두고, 관측을 입력받아 행동을 출력하는 지도학습(supervised learning) 문제로 푼다.

아래 코드는 48차원 관측 벡터에서 7차원 행동 벡터(예: 7축 팔의 관절 속도)를 MSE 손실로 예측하는 가장 기본적인 PyTorch BC 학습 루프다.

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

**Compounding error(오차 누적)**: BC는 구현이 놀라울 만큼 쉽지만 치명적인 약점이 하나 있다. 일반 지도학습은 테스트 입력이 학습 데이터와 같은 분포에서 나온다고 가정하지만, 순차 의사결정에서는 로봇이 내린 결정이 다음 관측 상태를 바꿔 버린다. 학습된 정책이 전문가 궤적에서 아주 살짝만 벗어나도 학습 데이터에서는 한 번도 본 적 없는 낯선 상태에 진입하고, 그 상태에서 낸 어설픈 행동이 로봇을 정답 경로에서 더 멀리 밀어내면서 오차가 눈덩이처럼 커진다. Ross & Bagnell(2010)의 분석에 따르면, 한 스텝 비용이 0~1 사이이고 전문가가 지나간 상태에서 정책이 전문가와 다른 행동을 고를 확률이 $\epsilon$일 때, 시간 길이 $T$ 동안 쌓이는 총비용은 전문가보다 최대 $\epsilon T^2$만큼, 즉 시간에 대해 **이차(quadratic)**로 불어날 수 있다.

### DAgger (Dataset Aggregation)

오차가 이차로 불어나는 이유를 뜯어보면 해법이 보인다. 정책이 궤적을 벗어난 상태에서 어떻게 복구해야 할지 데이터셋에 적혀 있지 않기 때문이다. **DAgger**는 이 공백을 메우기 위해 정책을 직접 굴려 보면서 전문가에게 실시간으로 교정을 받는 대화형 학습 절차를 밟는다.

1. 초기 전문가 시연 데이터로 기본 BC 정책을 학습한다.
2. 현재 학습된 정책을 실제로 실행해 새 궤적을 수집한다(이때 정책이 삐끗하며 낯선 상태로 들어간다).
3. 그렇게 정책이 방문한 궤적의 각 상태에서 "이런 상황에서는 이렇게 복구해야 한다"고 전문가가 정답 행동을 라벨로 붙여 준다.
4. 새로 얻은 `(상태, 전문가 행동)` 쌍을 기존 데이터셋에 합쳐 정책을 다시 학습한다.
5. 성능이 수렴할 때까지 2~4단계를 반복한다.

이처럼 정책이 실제로 방문할 상태 분포 위에서 전문가의 교정 라벨을 받아 데이터셋을 채워 나가면, 각 반복의 정책을 no-regret online learning 알고리즘으로 고른다는 가정 아래, Ross, Gordon & Bagnell(2011)이 증명했듯 충분한 반복 끝에 총비용 격차를 $T^2$에서 $T$에 비례하는 선형 수준까지 낮출 수 있다. 다만 반복마다 사람이 정책의 주행을 지켜보며 라벨을 붙여 주어야 하므로 수고가 많이 든다는 실무적 부담이 남는다.

### ACT (Action Chunking with Transformers)

Stanford의 ALOHA 프로젝트에서 제안된 **ACT**(Zhao et al., RSS 2023) 아키텍처는 DAgger처럼 매번 사람에게 교정을 받지 않고, 오프라인 시연 데이터만으로도 compounding error와 다봉 분포 문제를 크게 완화했다. ACT는 두 가지 핵심 장치로 섬세한 양팔 조작의 성공률을 끌어올렸다.

1. **Action chunking**: 매 스텝 다음 한 스텝 행동만 예측하는 대신, 미래 $k$ 스텝의 행동 시퀀스(chunk)를 한꺼번에 예측해 실행한다. 덕분에 에피소드 동안 정책이 새로 판단을 내려야 하는 횟수가 $1/k$로 줄어들어, 사소한 오차가 스텝마다 눈덩이처럼 번질 틈을 막아 준다.
2. **CVAE (Conditional Variational Autoencoder)**: 사람이 같은 작업을 시연해도 어떤 때는 왼쪽으로 돌아가고 어떤 때는 오른쪽으로 돌아가는 등 여러 갈래의 정답(다봉 분포, multimodal distribution)이 섞여 있다. 여기에 단순 MSE 손실을 들이대면 왼쪽과 오른쪽 행동의 평균인 "가운데 벽으로 들이받는 행동"이 나오고 만다. ACT는 CVAE의 잠재 변수를 도입해 시연마다 다른 스타일의 변동을 포착함으로써 평균 내기 현상을 피한다.

전체 구조는 카메라 이미지와 관절 각도를 입력받아 행동 묶음을 디코딩하는 Transformer encoder-decoder로 이루어져 있다.

### Diffusion Policy

ACT와 같은 해에 발표된 **Diffusion Policy**(Chi et al., RSS 2023)는 이미지 생성 분야에서 위력을 떨치던 확산 모델(diffusion model)을 로봇 행동 시퀀스 생성에 그대로 옮겨 온 방법이다. 앞서 짚었듯 MSE 회귀 기반 BC는 사실상 출력 분포로 봉우리가 하나뿐인 단봉 가우시안(unimodal Gaussian)을 가정한다(에너지 기반 모델이나 혼합 밀도 모델로도 다봉 분포를 다룰 수 있지만 학습이 까다롭다). 반면 Diffusion Policy는 순수 가우시안 노이즈에서 출발해 현재 관측을 조건으로 삼아 노이즈를 $K$번 단계적으로 걷어 내며(denoising) 미래 행동 궤적을 빚어내므로, 복잡하게 갈라진 다봉 행동 분포를 아주 자연스럽게 샘플링한다.

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

2023년 이후 매니퓰레이션 모방학습 연구에서는 ACT와 Diffusion Policy가 사실상 양대 기준선(baseline)으로 자리 잡았으며, HuggingFace의 **LeRobot** 라이브러리에 두 모델 모두 깔끔하게 구현되어 있다.

### 데이터 수집 방법

모방학습에서는 알고리즘 구조만큼이나 전문가가 모은 시연 데이터의 품질과 다양성이 성능을 좌우한다. 현장에서 주로 쓰는 데이터 수집 인터페이스는 크게 네 가지다.

- **Teleoperation (원격 조종)**: 사람이 조종용 마스터 장비를 움직이면 작업 로봇이 그대로 따라 움직이게 하여 데이터를 모은다. ALOHA는 사람이 가벼운 leader 팔을 직접 움직이면 작업용 follower 팔이 추종하는 양팔 텔레오퍼레이션 시스템으로, 2만 달러 미만의 하드웨어 비용으로 정교한 양팔 조작 데이터를 대량 수집해 주목받았다.
- **VR controller**: VR 컨트롤러로 3차원 공간에서 손끝(end-effector)의 목표 위치와 자세를 직관적으로 지정한다. 사람 손의 움직임을 곧바로 옮기기 편하지만, 빡빡하게 끼워 넣는 contact-rich 작업에서는 조종자가 반력을 손끝으로 느낄 수 없어 섬세한 힘 조절 시연을 남기기 어렵다.
- **Kinesthetic teaching (직접 교시)**: 중력 보상 모드로 띄워 둔 로봇 팔을 사람이 손으로 직접 잡고 원하는 궤적으로 움직여 기록한다. 직관적이지만 무겁고 출력이 큰 산업용 로봇에는 안전상 쓰기 어렵다.
- **Space mouse**: 6자유도 병진·회전 입력을 한 손으로 미세하게 조종할 수 있는 장치로, 느리지만 정밀한 정렬이 필요한 작업에 유용하다.

필요한 시연 횟수는 작업 난이도와 표현력에 따라 달라진다. Chi et al.(RSS 2023)은 실제 로봇 실험에서 Push-T 작업에 136개, 머그컵 뒤집기에 250개의 시연을 모았고, IJRR 확장판(2024)에 추가한 양팔 셔츠 접기 작업에는 284개의 시연을 사용했다. 대체로 시연 데이터가 늘수록 성공률이 올라가지만, 그만큼 사람이 로봇 앞에 붙어 데이터를 모으는 인력과 시간 비용도 함께 커진다.

---

## 8.7 Foundation Models for Robot Control [심화]

앞선 모방학습 기법들은 특정 작업 하나마다 수백 개의 시연을 모아 전용 정책을 새로 학습시키는 데 탁월하다. 하지만 로봇이 작업실 밖으로 나와 매번 다른 물건과 지시를 마주하려면, 대규모 언어 모델(LLM)이나 비전-언어 모델(VLM)처럼 방대한 로봇 데이터로 미리 학습해 둔 **범용 정책(generalist policy / robotics foundation model)**이 필요해진다. 웹에서 익힌 일반 상식과 여러 로봇에서 모은 행동 데이터를 하나로 묶어, 처음 보는 물체나 새 로봇에도 적은 파인튜닝만으로 적응하려는 시도다.

### RT-1, RT-2 (Google DeepMind)

범용 제어 모델의 포문을 연 대표 시리즈가 Google DeepMind의 RT 계열이다.
- **RT-1 (RSS 2023)**: 로봇 13대를 투입해 17개월 동안 모은 약 13만 개의 시연 데이터로 학습한 Transformer 기반 제어 정책이다. 카메라 이미지와 자연어 명령을 함께 받아 이산화한 행동 토큰을 출력하며, 모델 하나로 700개 이상의 서로 다른 작업 지시를 소화했다.
- **RT-2 (CoRL 2023)**: 한 걸음 더 나아가 PaLM-E나 PaLI-X처럼 웹 규모의 이미지-텍스트로 사전학습된 VLM을 가져와, 텍스트 토큰과 똑같은 형식으로 로봇의 action 토큰을 출력하도록 공동 파인튜닝(VLA, Vision-Language-Action)했다. 덕분에 로봇 시연 데이터에는 한 번도 등장하지 않았던 물체 이름이나 추상적 지시(예: "멸종 위기 동물 그림 위로 물체를 옮겨라")도 웹에서 배운 시맨틱 지식을 끌어와 처리하는 모습을 보였다.

### Octo

폐쇄적인 사내 데이터로 학습된 RT 시리즈와 달리, **Octo**는 UC Berkeley·Stanford·CMU 등 여러 대학이 힘을 합쳐 만든 오픈소스 범용 로봇 정책이다. 20여 개 기관이 다양한 로봇 팔로 모은 **Open X-Embodiment** 데이터셋으로 사전학습되었으며, 연속적인 정밀 제어를 위해 diffusion 기반 action head를 채택해 사용자의 새 로봇과 센서 구성에 맞춰 손쉽게 파인튜닝할 수 있도록 설계되었다.

### π0 (Physical Intelligence)

[π0 기술 보고서(2024)](https://arxiv.org/abs/2410.24164)에서 제시된 범용 로봇 정책은 사전학습된 VLM의 의미 이해 능력 위에 연속 행동을 고주파로 뽑아내는 **flow-matching action expert**를 결합한 구조다. 저자들은 단일 팔(single-arm), 양팔(dual-arm), 모바일 매니퓰레이터 데이터를 아울러 학습한 뒤 zero-shot 실행, 자연어 지시 따르기, 후속 fine-tuning 성능을 검증했고, 빨래 개기·테이블 닦기·상자 조립처럼 길고 섬세한 동작을 연속으로 이어 가는 작업을 시연했다.

### OpenVLA

Stanford 등이 공개한 **OpenVLA**는 7B 파라미터 규모의 오픈소스 VLM(Llama 2 기반 Prismatic VLM)을 Open X-Embodiment 데이터셋으로 파인튜닝해 action token을 생성하도록 만든 VLA 모델이다. 학습 코드와 모델 가중치를 전면 공개해 연구자들이 직접 파인튜닝 실험을 재현하고 확장할 수 있는 기반을 마련했다.

### 실사용의 걸림돌

화려한 데모와 달리, 파운데이션 모델을 실제 산업 현장이나 서비스 로봇에 곧바로 투입하기까지는 아직 넘어야 할 현실적 벽이 뚜렷하다. 우선 정해진 작업 하나만 놓고 보면 전통적인 제어 기법이나 그 작업 전용으로 짠 소형 모방학습 모델이 훨씬 높은 성공률과 일관성을 보이는 사례가 많다. 게다가 인터넷을 긁어모으면 되는 텍스트·이미지와 달리, 로봇 데이터는 사람과 물리 장비가 직접 움직여 모아야 하므로 데이터 한 건당 수집 비용이 비교할 수 없을 만큼 높다. 무엇보다 무거운 7B급 거대 모델은 추론 지연(inference latency)이 수십~수백 Hz로 돌아가야 하는 실시간 제어 주기를 넘기기 쉽다. 겉보기에 멀쩡하던 모델이 낯선 상황에서 어떤 돌발 행동을 할지 몰라 사전에 안전을 증명하기 어렵다는 점도 큰 숙제다.

### 연구 방향

현재 연구 커뮤니티는 이 한계를 넘기 위해 네 가지 축으로 돌파구를 찾고 있다.
- **Data scaling**: Open X-Embodiment처럼 여러 기관의 데이터를 통합해, 데이터 규모와 다양성을 키울 때 일반화 능력이 어떤 스케일링 법칙을 따르는지 검증한다.
- **Cross-embodiment transfer**: 관절 수와 그리퍼 형태가 서로 다른 로봇들 사이에서 공통된 행동 표현을 찾아 한 로봇의 경험을 다른 로봇으로 옮긴다.
- **Efficient fine-tuning**: 거대한 모델 전체를 다시 학습시키는 대신 LoRA 같은 parameter-efficient fine-tuning 기법으로 새 작업 현장에 빠르게 적응시킨다.
- **Action representation**: 로봇의 행동을 이산 토큰으로 쪼갤지, 연속 확률 분포나 diffusion·flow matching 출력으로 다룰지, 그리고 제어 주파수와 정밀도를 어떻게 맞출지 탐구한다.

---

## 8.8 Reward Design과 Safe RL [심화]

다시 강화학습으로 시선을 돌려 보자. 시뮬레이션이든 파운데이션 모델의 후속 학습이든, 결국 로봇에게 "무엇이 잘한 행동인가"를 알려 주는 보상 함수를 잘못 설계하면 에이전트는 설계자의 의도를 교묘히 비껴가는 엉뚱한 꼼수를 학습한다. 게다가 실제 물리 로봇에 RL을 올리려면 보상 최대화와 함께 하드웨어를 보호하는 **안전 제약**까지 명시적으로 걸어야 한다.

### Reward Shaping

**Sparse reward의 한계**: "목표에 성공하면 $+1$, 아니면 $0$"처럼 끝점에만 점수를 주는 희소 보상(sparse reward)은 식을 적기는 매우 편하다. 하지만 에이전트가 우연히 첫 성공을 거둘 때까지 완전한 무작위 탐색에 의존해야 하므로, 상태-행동 공간이 조금만 넓어져도 영원히 $+1$을 한 번도 보지 못한 채 학습이 멈춰 버리기 일쑤다.

**Dense reward**: 학습의 길잡이가 되어 주도록 목표에 가까워지는 중간 과정에도 촘촘하게 보상을 깔아 주는 방법이다. 아래는 그리퍼로 물체를 집어 목표 지점에 옮기는 작업에서 거리 비례 페널티, 파지 보너스, 최종 성공 보너스를 조합한 전형적인 dense reward 함수다.

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

**Curriculum learning**: 보상 함수를 복잡하게 쪼개는 대신 과제 난이도를 단계적으로 높여 가는 전략도 효과적이다. 가령 보행 학습에서 처음부터 가파른 계단을 오르게 하지 않고 평지 걷기 → 낮은 장애물 넘기 → 계단 오르기 순으로 환경 난이도를 올려 주면, 쉬운 단계에서 먼저 성공 경험을 쌓은 정책이 어려운 단계에서도 길을 잃지 않는다.

### Reward Hacking

에이전트는 사람이 머릿속으로 상상한 "바람직한 목표"가 아니라, 오직 코드에 적힌 숫자 보상을 극대화하는 방향으로만 최적화한다. 그 결과 설계자가 전혀 의도하지 않은 기상천외한 방식으로 높은 점수를 따내는 현상이 벌어지는데, 이를 **보상 해킹(Reward Hacking)** 또는 **명세 게임(specification gaming)**이라 부른다.

- 빨간 블록을 파란 블록 위에 쌓는 과제에서 빨간 블록 아랫면의 높이를 보상으로 주자, 로봇 팔이 힘들여 블록을 집어 올리는 대신 제자리에서 빨간 블록을 옆으로 뒤집어 아랫면만 위로 향하게 만들었다([Popov et al., 2017](https://arxiv.org/abs/1704.03073)).
- 형태와 제어기를 함께 진화시킨 가상 생물에게 평균 이동 속도를 적합도로 주자, 생물이 다리나 지느러미를 만들어 성실히 기어가는 대신 키가 크고 묵직한 몸체를 만든 뒤 앞으로 쿵 넘어지면서 순간적인 이동 속도를 챙겼다(Karl Sims의 1994년 연구, [Lehman et al., Artificial Life 2020](https://arxiv.org/abs/1803.03453)에 기록).
- 점프를 배우도록 형태를 진화시키면서 에피소드 중 가장 낮았던 블록의 최대 도달 높이를 보상으로 주자, 점프는커녕 긴 막대 위에 거대한 블록 몇 개를 아슬아슬하게 쌓아 올린 뒤 머리부터 넘어지며 공중제비를 돌아 높이를 부풀렸다(Krcah의 실험, 같은 글).

이와 같은 기발하면서도 당황스러운 사례들은 [Krakovna의 specification gaming 목록](https://vkrakovna.wordpress.com/2018/04/02/specification-gaming-examples-in-ai/)과 [DeepMind 블로그 글(2020)](https://deepmind.google/blog/specification-gaming-the-flip-side-of-ai-ingenuity/)에 풍성하게 정리되어 있다. 실무에서는 이 함정을 피하기 위해 단순히 학습 곡선(reward graph)만 바라보지 않고, 학습된 정책의 실제 주행·조작 영상을 사람이 직접 눈으로 확인해 원치 않는 꼼수에 페널티 항을 추가하며 보상 함수를 반복해서 다듬는다.

### Constrained RL

실제 로봇에서는 "점수를 높이는 것" 못지않게 "모터 토크 한계를 넘기지 말 것"이나 "장애물에 절대 부딪히지 말 것"처럼 **지켜야 할 안전 하한선**이 중요하다. 단순히 충돌에 큰 음수 보상을 주는 방식에서는 학습 중 우연히 한 번 부딪혀 보상을 깎아 먹으면서도 여전히 평균 보상이 더 크면 충돌을 감수할 위험이 남는다. **Constrained RL**은 비용(cost) $c_t$의 기대 누적합을 허용 한도 $d$ 이하로 묶는 제약을 별도로 건 상태에서 보상을 최대화한다.

```
max_π E[ Σ γ^t r_t ]   subject to   E[ Σ γ^t c_t ] ≤ d
```

여기서 $c_t$는 관절 토크 한계 초과나 장애물 충돌 같은 위험 비용이고, $d$는 허용 상한이다. 이 문제는 CPO(Constrained Policy Optimization), PCPO, 라그랑주 승수(Lagrangian relaxation) 기반 기법으로 푼다. 이 기법들은 실제 하드웨어 배포 전 단계에서 토크·관절 각도 제한을 명시적 제약으로 걸어 장비를 보호하는 데 쓰인다.

### Human-in-the-loop RL

보행 동작의 자연스러움이나 깨지기 쉬운 물건을 내려놓을 때의 조심성처럼 아예 수식으로 적기 애매한 기준은 사람의 직관적 평가를 직접 끌어와 해결한다. LLM 학습의 RLHF(RL from Human Feedback)와 똑같은 원리다.

1. 로봇이 실행한 두 행동 영상을 나란히 보여 주고 사람이 더 나은 쪽을 고른다(선호도 수집).
2. 이렇게 모은 선호도 데이터로 사람의 취향을 모사하는 보상 모델(reward model)을 학습한다.
3. 학습된 보상 모델을 기준으로 삼아 강화학습을 돌린다.

이 방식으로는 수치화하기 어려운 섬세한 움직임을 가르칠 수 있지만, 사람이 일일이 영상을 보고 선호도를 매기는 데 상당한 시간이 든다. 또한 학습된 보상 모델이 완벽하지 않으면 정책이 그 허점을 파고드는 또 다른 reward hacking이 생길 수 있음을 늘 염두에 두어야 한다.

---

지금까지 6~8장을 거치며 로봇이 어떻게 물리적으로 움직이고(제어), 충돌 없이 경로를 짜며(플래닝), 데이터와 경험으로부터 행동을 배우는지(로봇 러닝) 살펴보았다. 그런데 이 모든 의사결정과 학습 파이프라인이 제대로 돌아가려면, 먼저 **카메라가 찍은 날것의 픽셀에서 주변 물체와 3차원 공간 정보를 정확히 읽어 내는 눈**이 뒷받침되어야 한다. 이어지는 9장에서는 바로 그 출발점인 컴퓨터 비전의 기초로 넘어간다.

---

## 참고 자료

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

## 발전 과정

```
1992 ── REINFORCE algorithm (Williams)
         Monte Carlo policy-gradient 방법. score-function 추정량의 variance가 높다.

2013 ── DQN (Mnih et al., Atari)
         CNN으로 Atari를 화면 픽셀에서 학습. replay buffer로 표본 간 상관을 줄였다.
         고정 target network는 2015년 Nature 판에서 더해졌다.

2015 ── TRPO (Schulman et al.)
         trust region 제약으로 policy update 크기를 묶음. 구현에 2차 최적화가 필요.

2017 ── PPO (Schulman et al.)
         clipping으로 TRPO의 제약을 근사. 로봇 RL 실험에서 많이 쓰는 기준선.

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
