# 5장 강체 역학과 동역학

4장에서 우리는 관절 각도와 끝단 자세 사이의 기하학적 관계, 즉 기구학을 다뤘다. 하지만 로봇이 실제로 팔을 뻗어 무거운 물체를 들어 올리거나, 빠른 궤적을 따라 움직이거나, 두 발로 바닥을 딛고 균형을 잡으려면 위치와 자세만 계산해서는 부족하다. 로봇의 질량과 관성, 중력, 그리고 외부 접촉력이 어우러져 어떤 가속도와 토크를 만들어내는지 계산해야 비로소 움직임을 물리적으로 다룰 수 있다. 이 장에서는 뉴턴-오일러 역학과 라그랑주 역학이라는 두 관점에서 로봇의 운동 방정식을 세우고, 이를 실시간에 맞게 계산하는 재귀 알고리즘(RNEA, ABA)과 접촉 역학, 공간 벡터 대수(spatial algebra), 그리고 발이 지면에서 떨어지기도 하는 부유 베이스 시스템의 중심 동역학(centroidal dynamics)까지 차근차근 살펴본다.

---

## 5.1 동역학이 쓰이는 곳

기구학이 "로봇이 어디에 있고 어떤 자세를 취하는가"라는 기하학의 질문에 답한다면, 동역학(dynamics)은 "그 움직임을 만들어 내려면 얼마나 큰 힘과 토크가 필요한가"를 묻는다. 물론 모든 로봇에 동역학 모델이 꼭 필요한 것은 아니다. 관절을 천천히 움직이는 산업용 매니퓰레이터라면 속도에 비례하는 관성력과 코리올리 힘이 무시할 수준으로 작기 때문에, 기구학으로 목표 각도를 구하고 각 관절의 PID 제어기가 나머지 오차를 잡아주는 방식만으로도 충분히 잘 작동한다.

하지만 로봇의 움직임이 빨라지거나 환경과 힘을 주고받기 시작하면 이야기가 달라진다. 다음과 같은 현장에서는 동역학 모델이 설계와 제어의 핵심으로 들어온다.

- **고속 매니퓰레이션**: 작업 주기(cycle time)를 줄이려고 팔을 빠르게 휘두르면 관성력, 원심력, 코리올리 힘이 급격히 커진다. 제어기가 이 힘들을 미리 계산해 보상하지 않으면 경로 추종 오차가 벌어지고, 심하면 관절 모터가 낼 수 있는 최대 토크를 넘겨 포화(saturation) 상태에 빠진다.
- **보행 로봇 (legged robots)**: 두 발이나 네 발로 걷는 로봇은 지면을 미는 접촉력을 조절해 몸통의 균형을 잡는다. 접촉력 자체가 힘의 변수이므로 동역학 없이는 보행 제어를 세울 수 없다.
- **시뮬레이션**: 물리 시뮬레이터는 매 스텝 현재 상태와 입력 토크로부터 각 관절의 가속도를 구하고, 이를 시간 적분해 다음 상태를 계산한다.
- **최적 제어 (optimal control)**: 에너지나 소요 시간을 최소화하는 궤적을 찾을 때, 로봇이 물리 법칙을 거스르지 않도록 동역학 방정식을 제약 조건(constraint)으로 넣는다.
- **충돌과 접촉 처리**: 물체를 집고(grasp), 밀고(push), 던지는(throw) 작업에서는 로봇과 물체 사이에 오가는 충격과 마찰을 접촉 역학으로 다룬다.

동역학이 로보틱스에서 왜 중요한지 큰 그림을 잡고 싶다면 아래 자료가 좋은 길잡이가 된다.

> **추천 자료**
> - Featherstone, *Rigid Body Dynamics Algorithms*, Chapter 1 — 동역학이 왜 필요한지를 간결하게 설명한다.
> - Russ Tedrake, *Underactuated Robotics* Ch.1 (https://underactuated.csail.mit.edu/) — 동역학 모델을 제어에 쓰는 이유를 예제로 보여준다.

지금까지는 동역학이 필요한 이유를 짚었다. 이제 실제로 로봇 링크 하나하나에 걸리는 힘과 토크를 어떻게 계산하는지, 가장 직관적인 뉴턴-오일러 역학부터 출발해 보자.

---

## 5.2 뉴턴-오일러 역학

### 기본 원리

3차원 공간에서 움직이는 강체의 운동은 크게 두 가지로 쪼개어 볼 수 있다. 질량중심이 공간을 이동하는 병진 운동과, 그 중심 주위를 도는 회전 운동이다. 뉴턴-오일러 역학은 이 두 운동에 각각 힘의 평형과 토크의 평형을 적용해 운동 방정식을 적는다.

먼저 직선으로 가속하는 병진 운동은 익숙한 뉴턴의 제2법칙을 그대로 따른다.

**병진 운동 (translational motion):**
```
F = ma
```
물체에 작용하는 합력 F는 질량 m과 질량중심(CoM) 가속도 a의 곱과 같다.

회전 운동은 조금 더 주의가 필요하다. 물체의 관성이 회전체 모양과 축 방향에 따라 달라지는 데다, 3차원에서는 회전축 자체가 공간에서 돌아가면서 추가적인 관성 모멘트 변화가 생기기 때문이다. 물체 고정 좌표계에서 회전 운동 방정식은 다음과 같이 쓴다.

**회전 운동 (rotational motion):**
```
τ = Iα + ω × (Iω)
```
여기서 합토크 τ는 관성 텐서 I와 각가속도 α의 곱에 자이로스코픽 항 `ω × (Iω)`를 더한 값이다. 평면 위에서만 움직이는 2D 문제에서는 각속도 벡터 ω와 각운동량 벡터 Iω가 항상 같은 축(지면에 수직인 축)을 가리키므로 외적 `ω × (Iω)`가 0으로 사라져 `τ = Iα`로 단순해진다. 반면 3D에서는 회전축이 수시로 바뀌기 때문에 이 자이로스코픽 항이 살아남는다. 시뮬레이터나 제어기에서 구현을 단순화하려고 이 항을 빼먹으면, 팽이가 세차 운동을 하거나 로봇 팔이 빠르게 선회할 때 물체가 실제 물리 법칙과 전혀 다르게 돌아간다.

### Recursive Newton-Euler Algorithm (RNEA)

강체 하나의 방정식을 세웠다면, 이제 관절로 여러 링크가 줄지어 연결된 직렬 매니퓰레이터(serial manipulator)로 확장할 차례다. 원하는 관절 위치 q, 속도 q̇, 가속도 q̈이 주어졌을 때 각 모터가 내야 하는 토크 τ를 구하는 문제를 **역동역학(inverse dynamics)**이라 부른다. RNEA는 이 역동역학을 링크 개수 n에 비례하는 O(n) 시간에 풀어내는 대표적인 알고리즘이며, 계산은 단 두 번의 순회(pass)로 끝난다.

1. **Forward pass (base → end-effector):** 고정된 베이스에서 손끝 방향으로 올라가며 각 링크의 속도와 가속도를 차례로 구한다. 링크 i의 속도는 부모 링크 i-1의 속도에 관절 i의 회전·직동 속도를 더한 값이고, 가속도 역시 같은 방식으로 전파된다.
2. **Backward pass (end-effector → base):** 이제 손끝에서 베이스 방향으로 되돌아 내려오면서 각 링크에 걸리는 힘과 토크를 합산한다. 각 링크의 속도와 가속도를 이미 알고 있으므로 뉴턴-오일러 방정식으로 그 링크를 가속시키는 데 필요한 합력·합토크를 계산하고, 끝단 쪽 자식 링크들이 밀고 당기는 반작용력을 더해 부모 관절 i가 버텨야 할 토크를 추출한다.

굳이 재귀(recursive) 구조를 쓰는 까닭은 연산량 때문이다. 강체 하나의 동역학 계산은 O(1)이므로 n개 링크를 앞뒤로 한 번씩만 훑고 지나가면 전체 계산량이 O(n)에 그친다. 반면 뒤에서 볼 라그랑주 방정식을 기계적으로 직접 전개해 관성 행렬 M(q)을 매번 조립하고 미분하면 O(n³) 이상의 연산이 든다. 관절 수 n이 6개 남짓한 산업용 팔에서는 차이가 작아 보이지만, 관절이 30개 안팎에 이르는 휴머노이드 로봇에서는 O(n)과 O(n³)의 격차가 수십 배 이상 벌어져 1kHz 제어 루프 주기를 맞출 수 있느냐를 가른다.

아래 의사 코드는 RNEA의 두 패스가 구체적으로 어떤 값을 주고받는지 보여 준다. `Forward pass`에서 끝단까지 올라가며 각 링크의 운동과 필요한 합력 `f[i]`를 구하고, `Backward pass`에서 내려오며 관절 축에 투영한 토크 `τ[i]`를 뽑아 부모 링크로 힘을 넘기는 흐름에 주목해 보자.

```
RNEA(model, q, q̇, q̈):
    # Forward pass: i = 1, 2, ..., n
    for i = 1 to n:
        v[i] = v[i-1] + S[i] * q̇[i]        # 관절축 방향 속도 추가
        a[i] = a[i-1] + S[i] * q̈[i] + v[i] × (S[i] * q̇[i])
        f[i] = I[i] * a[i] + v[i] × (I[i] * v[i])  # Newton-Euler

    # Backward pass: i = n, n-1, ..., 1
    for i = n downto 1:
        τ[i] = S[i]^T * f[i]               # 관절 토크 추출
        f[parent(i)] += f[i]                # 부모 링크로 전파

    return τ
```

식에서 `S[i]`는 관절 i의 움직임 방향을 나타내는 motion subspace matrix이고, `v[i]`와 `I[i]`는 각각 링크 i의 공간 속도(spatial velocity)와 공간 관성(spatial inertia)이다. 이 6차원 표기법은 Featherstone의 spatial vector algebra를 따른 것으로, 5.7절에서 자세히 다룬다.

### 실제 코드: Pinocchio

원리를 이해했더라도 실무에서 매번 링크별 좌표 변환과 관성 텐서 회전을 손으로 구현하면 실수가 나기 쉽고 속도도 느리다. 그래서 현장에서는 URDF 파일을 읽어 RNEA를 곧바로 실행해 주는 검증된 라이브러리를 쓴다. 대표적인 C++/Python 동역학 라이브러리인 Pinocchio로 역동역학과 중력 보상 토크를 계산하는 코드는 다음과 같다.

```python
import pinocchio as pin
import numpy as np

# URDF에서 모델 로드
model = pin.buildModelFromUrdf("robot.urdf")
data = model.createData()

# 현재 상태 설정
q = pin.randomConfiguration(model)     # 관절 위치
v = np.random.randn(model.nv)          # 관절 속도
a = np.random.randn(model.nv)          # 관절 가속도

# RNEA: (q, v, a) → τ
tau = pin.rnea(model, data, q, v, a)
print("Joint torques:", tau)

# 중력 토크만 계산 (v=0, a=0)
tau_g = pin.rnea(model, data, q, np.zeros(model.nv), np.zeros(model.nv))
print("Gravity compensation torques:", tau_g)
```

위 코드의 두 번째 호출처럼 속도 `v`와 가속도 `a`를 모두 0으로 두고 `pin.rnea`를 부르면 로봇이 현재 자세를 그대로 버티는 데 필요한 **중력 보상(gravity compensation)** 토크가 곧바로 나온다. 알고리즘 내부에서 베이스 가속도를 중력과 반대 방향인 `a[0] = -g`로 초기화하면, 로봇 전체가 위로 가속하는 것과 같은 등가 원리가 적용되어 중력 효과가 계산에 반영되기 때문이다(앞선 의사 코드는 설명을 간결히 하려고 이 초기화를 생략했다). 이렇게 구한 `tau_g`를 모터에 흘려보내면 로봇 팔은 중력을 이기고 허공에 멈춰 서며, 사람이 손으로 톡 밀어도 가볍게 따라 움직인다. 실제 로봇 팔을 처음 세우고 테스트할 때 가장 먼저 구현해 보는 제어기가 바로 이 중력 보상이다.

동일한 역동역학 계산을 제어·최적화 프레임워크인 Drake에서도 `MultibodyPlant`로 수행할 수 있다. 궤적 최적화나 접촉 시뮬레이션까지 한 프레임워크 안에서 엮어야 한다면 다음과 같이 작성한다.

```python
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.parsing import Parser
from pydrake.multibody.tree import MultibodyForces
import numpy as np

plant = MultibodyPlant(time_step=0.0)
Parser(plant).AddModels("robot.urdf")
plant.Finalize()

context = plant.CreateDefaultContext()

q = np.random.randn(plant.num_positions())
v = np.random.randn(plant.num_velocities())
vdot = np.random.randn(plant.num_velocities())

plant.SetPositions(context, q)
plant.SetVelocities(context, v)

# Inverse dynamics: vdot → τ
tau = plant.CalcInverseDynamics(context, vdot, MultibodyForces(plant))
```

Pinocchio와 Drake 모두 내부에서 재귀 알고리즘으로 빠르게 토크를 계산해 준다. 더 깊이 파고들고 싶다면 아래 문헌을 참고하자.

> **추천 자료**
> - Featherstone, *Rigid Body Dynamics Algorithms*, Chapter 5 — RNEA의 원본 설명
> - Pinocchio documentation (https://github.com/stack-of-tasks/pinocchio) — 실무에서 RNEA를 가장 쉽게 써볼 수 있는 라이브러리
> - Luh, Walker, Paul (1980), "On-Line Computational Scheme for Mechanical Manipulators" — RNEA의 원 논문

뉴턴-오일러 역학은 수치를 빠르게 계산하는 데 탁월하지만, 각 링크 내부에 걸리는 내력과 구속력이 식에 일일이 얽혀 있어 시스템 전체의 구조나 에너지 성질을 한눈에 파악하기는 어렵다. 제어기를 설계하거나 안정성을 증명하려면 전체 시스템을 하나의 깔끔한 행렬 방정식으로 요약할 필요가 있는데, 이때 등장하는 도구가 바로 라그랑주 역학이다.

---

## 5.3 라그랑주 역학

### Lagrangian이란

라그랑주 역학은 힘과 모멘트 벡터를 하나씩 추적하는 대신, 시스템 전체의 스칼라 값인 **운동에너지(T)**와 **위치에너지(V)**의 차이로부터 운동 방정식을 유도한다. 이 두 에너지의 차이를 라그랑지안(Lagrangian) `L`이라 정의한다.

```
L(q, q̇) = T(q, q̇) - V(q)
```

여기서 T는 시스템의 운동에너지(kinetic energy), V는 위치에너지(potential energy)다. 라그랑지안 L을 구하고 나면, 각 일반화 좌표(generalized coordinate) q_i마다 아래의 **Euler-Lagrange 방정식**에 대입해 미분하기만 하면 운동 방정식이 자동으로 도출된다.

```
d/dt (∂L/∂q̇_i) - ∂L/∂q_i = τ_i
```

이 방식의 큰 장점은 일반화 좌표를 자유롭게 선택할 수 있다는 데 있다. 뉴턴-오일러 방식에서는 링크마다 좌표 프레임을 잡고 관절이 버티는 구속력(constraint force)을 어떻게 소거할지 신경 써야 한다. 반면 라그랑주 역학에서는 관절 각도를 일반화 좌표로 잡아두기만 하면, 그 관절의 기하학적 구속이 좌표 선택 자체에 이미 녹아 있으므로 내부 구속력이 식에서 저절로 사라진다. 다만 구속 조건이 이렇게 깔끔하게 소거되는 경우는 구속이 홀로노믹(holonomic)이고 독립적인 일반화 좌표를 잡을 수 있는 개방 사슬(open chain) 구조일 때뿐이다. 바퀴의 미끄럼 없음 조건 같은 비홀로노믹 구속이나 고리 모양의 폐쇄 사슬(closed chain)에서는 라그랑주 승수(Lagrange multiplier) 항이 식에 남는다.

### Manipulator Equation

개방 사슬 형태의 n-DOF 직렬 매니퓰레이터에 대해 Euler-Lagrange 방정식을 전개하고 q̈, q̇, q 항으로 묶어 정리하면, 로보틱스 전역에서 표준처럼 쓰는 **manipulator equation**이 나타난다.

```
M(q)q̈ + C(q, q̇)q̇ + g(q) = τ
```

이 방정식은 로봇 팔에 가해지는 토크 τ가 가속, 속도 커플링, 중력이라는 세 물리적 저항을 어떻게 이겨내는지를 한눈에 보여 준다.

- **M(q)** — 질량/관성 행렬(mass/inertia matrix). n×n 대칭 양정치(symmetric positive definite) 행렬이며 로봇의 자세 q에 따라 값이 변한다. 예컨대 팔을 앞으로 쭉 뻗으면 회전 반경이 커져 관성이 커지고, 몸 쪽으로 접으면 관성이 작아진다.
- **C(q, q̇)q̇** — 코리올리 및 원심력 항(Coriolis and centrifugal terms). 여러 관절이 동시에 움직일 때 한 관절의 회전이 다른 관절에 미치는 관성 커플링 효과다. 관절 속도의 2차 항이므로 속도가 두 배 빨라지면 이 힘은 네 배로 커진다.
- **g(q)** — 중력 벡터(gravity vector). 중력장에서 로봇이 현재 자세를 유지하려 할 때 각 관절에 걸리는 정적 중력 토크다.
- **τ** — 관절 토크 벡터. 각 관절 모터가 실제로 내는 구동 토크를 뜻하며, 관절 마찰(friction)은 보통 따로 모델링해 이 식에 덧붙인다.

앞으로 6장의 제어기 설계, 7장의 궤적 최적화, 그리고 시뮬레이터의 적분 루프가 모두 이 방정식에서 출발한다.

### 2-Link Planar Arm 예제

식만 바라보면 M, C, g가 구체적으로 어떤 삼각함수와 질량 항으로 얽혀 있는지 감이 잘 오지 않는다. 2관절 평면 로봇 팔(2-link planar arm)은 손으로 직접 유도해 볼 수 있는 가장 간단한 다관절 예제이면서도, n-DOF manipulator equation의 모든 구조적 특징을 빠짐없이 담고 있다.

질량이 각 링크의 끝점에 집중되어 있다고(point mass) 가정하고 다음과 같이 기호를 정하자.

- 링크 길이: l_1, l_2
- 링크 질량: m_1, m_2 (질량이 링크 끝에 집중된다고 가정 — point mass)
- 관절 각도: q_1, q_2 (base에서부터)
- 중력: g (아래 방향)

먼저 직교 좌표계에서 각 질량점의 위치를 관절 각도 q_1, q_2로 적으면 순기구학 식이 된다.

링크 1 끝점의 위치:
```
x_1 = l_1 cos(q_1)
y_1 = l_1 sin(q_1)
```

링크 2 끝점의 위치:
```
x_2 = l_1 cos(q_1) + l_2 cos(q_1 + q_2)
y_2 = l_1 sin(q_1) + l_2 sin(q_1 + q_2)
```

이제 각 좌표를 시간으로 미분해 속도의 제곱을 구하고, `T = (1/2)m_1 v_1^2 + (1/2)m_2 v_2^2`에 대입해 정리하면 운동에너지 T를 얻는다.

**운동에너지 T:**
```
T = (1/2)(m_1 + m_2) l_1^2 q̇_1^2
  + (1/2) m_2 l_2^2 (q̇_1 + q̇_2)^2
  + m_2 l_1 l_2 cos(q_2) q̇_1 (q̇_1 + q̇_2)
```

위치에너지는 각 질량의 높이 y_1, y_2에 중력 가속도를 곱해 더하면 된다.

**위치에너지 V:**
```
V = m_1 g l_1 sin(q_1) + m_2 g [l_1 sin(q_1) + l_2 sin(q_1 + q_2)]
```

이제 `L = T - V`를 Euler-Lagrange 방정식에 넣고 q̈_1, q̈_2의 계수를 모으면 관성 행렬 M(q)가 드러난다.

**M(q) 행렬:**
```
M(q) = [ (m_1+m_2)l_1^2 + m_2 l_2^2 + 2 m_2 l_1 l_2 cos(q_2)    m_2 l_2^2 + m_2 l_1 l_2 cos(q_2) ]
        [ m_2 l_2^2 + m_2 l_1 l_2 cos(q_2)                          m_2 l_2^2                          ]
```

행렬의 첫 번째 대각 원소 `M_{11}`을 보면 베이스 관절의 유효 관성이 두 번째 관절 각도 `q_2`에 따라 어떻게 달라지는지 직관적으로 읽힌다. 팔을 쭉 편 `q_2 = 0`일 때 `cos(q_2) = 1`이 되어 베이스 관성이 최대가 되고, 팔을 완전히 접은 `q_2 = π`일 때 `cos(q_2) = -1`이 되어 최소가 된다. 피겨 스케이터가 팔을 오므리면 회전 관성이 줄어들어 빠르게 도는 것과 같은 이치다.

다음으로 속도 제곱 항과 교차 항을 묶으면 코리올리·원심력 행렬 C(q, q̇)가 나온다.

**C(q, q̇) 행렬:**
```
C(q, q̇) = [ -m_2 l_1 l_2 sin(q_2) q̇_2    -m_2 l_1 l_2 sin(q_2)(q̇_1 + q̇_2) ]
            [  m_2 l_1 l_2 sin(q_2) q̇_1     0                                     ]
```

일반적으로 C 행렬은 M(q)의 편미분으로부터 Christoffel symbols를 계산해 체계적으로 뽑아내지만, 2-link처럼 자유도가 작을 때는 Euler-Lagrange 방정식을 전개한 뒤 속도 항을 직접 묶어 정리하는 쪽이 훨씬 빠르다.

마지막으로 위치에너지 V를 각 관절 각도로 편미분한 결과가 중력 벡터 g(q)다.

**g(q) 벡터:**
```
g(q) = [ (m_1 + m_2) g l_1 cos(q_1) + m_2 g l_2 cos(q_1 + q_2) ]
        [ m_2 g l_2 cos(q_1 + q_2)                                 ]
```

자유도가 3개만 넘어가도 삼각함수 미분과 전개가 급격히 번거로워지므로, 기호 연산(symbolic computation) 도구로 검산하는 습관을 들이면 좋다. 아래 SymPy 코드는 위 2-link 팔의 위치와 속도로부터 운동에너지 `T`와 위치에너지 `V`를 자동으로 조립하고 삼각함수 항을 정리해 앞서 손으로 유도한 식과 대조한다.

```python
import sympy as sp

q1, q2, dq1, dq2, ddq1, ddq2 = sp.symbols('q1 q2 dq1 dq2 ddq1 ddq2')
m1, m2, l1, l2, g = sp.symbols('m1 m2 l1 l2 g', positive=True)

# 위치
x1 = l1 * sp.cos(q1)
y1 = l1 * sp.sin(q1)
x2 = x1 + l2 * sp.cos(q1 + q2)
y2 = y1 + l2 * sp.sin(q1 + q2)

# 속도 (chain rule)
vx1 = sp.diff(x1, q1) * dq1
vy1 = sp.diff(y1, q1) * dq1
vx2 = sp.diff(x2, q1) * dq1 + sp.diff(x2, q2) * dq2
vy2 = sp.diff(y2, q1) * dq1 + sp.diff(y2, q2) * dq2

# 운동에너지
T = sp.Rational(1,2)*m1*(vx1**2 + vy1**2) + sp.Rational(1,2)*m2*(vx2**2 + vy2**2)
T = sp.trigsimp(sp.expand(T))

# 위치에너지
V = m1*g*y1 + m2*g*y2

# Lagrangian
L = T - V

# Euler-Lagrange equations
# d/dt(∂L/∂q̇_i) - ∂L/∂q_i = τ_i
# d/dt를 계산하려면 q1, q2를 시간의 함수로 바꿔 넣어야 한다.
# M, C, g를 뽑아내는 과정은 교재를 참고하라.

print("T =", T)
print("V =", V)
```

코드를 실행해 출력된 `T`와 `V`를 보면 `trigsimp`가 `cos(q1)^2 + sin(q1)^2 = 1`과 덧셈 정리를 정리해 주면서 앞서 유도한 에너지 식과 정확히 일치함을 확인할 수 있다.

> **추천 자료**
> - Murray, Li, Sastry, *A Mathematical Introduction to Robotic Manipulation*, Ch. 4 (https://www.cds.caltech.edu/~murray/mlswiki/) — 라그랑주 역학을 로보틱스 맥락에서 엄밀하게 다룬다. 무료 PDF 제공.
> - Spong, Hutchinson, Vidyasagar, *Robot Modeling and Control*, 2판(2020) Ch. 6 "Dynamics" — Euler-Lagrange와 Newton-Euler 정식화를 학부 수준으로 설명
> - Craig, *Introduction to Robotics*, Ch. 6 — 2-link arm 예제가 상세히 나와 있다

---

## 5.4 뉴턴-오일러 vs 라그랑주

뉴턴-오일러 역학과 라그랑주 역학은 출발점이 전혀 달라 보이지만, 결국 같은 물리 법칙을 다른 언어로 적은 식이므로 완전히 동일한 운동 방정식을 내놓는다. 실무에서 두 관점이 갈리는 지점은 식을 유도하는 편의성, 물리적 직관, 그리고 컴퓨터가 매 제어 주기마다 계산하는 속도다. 아래 표는 두 접근법의 핵심 차이를 항목별로 나란히 비교한다.

| 항목 | 뉴턴-오일러 (RNEA) | 라그랑주 |
|------|-------------------|---------|
| 관점 | 힘/토크 (force-based) | 에너지 (energy-based) |
| 계산 복잡도 | O(n) | O(n^3) (M 행렬 직접 계산 시) |
| 유도 난이도 | 재귀적이라 n이 커져도 같은 패턴 | n이 커지면 편미분이 폭발 |
| 물리적 직관 | 각 링크의 힘/토크를 직접 볼 수 있음 | 에너지 보존/변환을 볼 수 있음 |
| 주 용도 | 실시간 제어, 시뮬레이션 | 모델 유도, 에너지 기반 분석, Lyapunov 안정성 |
| 구속력 | 명시적으로 계산 가능 | 일반화 좌표 사용 시 자동으로 소거 |

이 표를 보면 둘 중 하나를 버리고 다른 하나만 택할 이유가 없다는 점이 분명해진다. 실제 로보틱스 엔지니어는 이론 해석과 수치 구현의 역할을 나누어 두 방법을 자연스럽게 조합한다.

1. **모델 유도**: 라그랑주 역학으로 manipulator equation의 대칭성, 수동성(passivity) 같은 구조적 성질을 이해한다.
2. **수치 계산**: 제어 루프 안에서는 RNEA(또는 뒤이어 볼 ABA) 같은 O(n) 재귀 알고리즘으로 실시간 계산을 돌린다.
3. **제어기 설계**: manipulator equation의 M, C, g 구조를 활용하는 computed torque control이나 passivity-based control 등을 설계한다.
4. **코드 구현**: Pinocchio나 Drake 같은 라이브러리가 내부에서 RNEA와 ABA를 이미 구현해 두고 있으므로, 현장에서는 해당 API를 호출해 검증된 성능을 곧바로 쓴다.

> **추천 자료**
> - Featherstone, *Rigid Body Dynamics Algorithms*, Ch. 3 — 두 formulation의 관계를 명확히 설명
> - Siciliano et al., *Robotics: Modelling, Planning and Control*, Ch. 7 — 두 방법으로 같은 로봇의 동역학을 유도하는 비교 예제

지금까지는 목표 가속도 q̈이 주어졌을 때 필요한 모터 토크 τ를 구하는 역동역학에 집중했다. 그런데 물리 시뮬레이터처럼 "모터에 이 토크 τ를 걸었을 때 로봇이 앞으로 어떻게 가속할까?"를 풀어야 하는 반대 상황도 못지않게 중요하다. 다음 절에서는 이 두 계산 방향의 차이와 순동역학을 O(n)에 푸는 알고리즘을 살펴본다.

---

## 5.5 Forward Dynamics vs Inverse Dynamics

동역학 계산은 우리가 이미 알고 있는 입력과 구하려는 미지수가 무엇이냐에 따라 정반대 방향의 두 문제로 나뉜다.

먼저 **Inverse Dynamics(역동역학)**는 궤적이 정해졌을 때 필요한 힘을 묻는다.
```
주어진 것: q, q̇, q̈
구하는 것: τ
```
"로봇이 이 궤적을 그대로 따라가려면 모터가 얼마의 토크를 내야 하는가?"에 답하는 계산이므로 주로 제어기에 들어간다. 6장에서 다룰 computed torque control 등이 이 계산을 핵심으로 쓴다.

반대로 **Forward Dynamics(순동역학)**는 힘을 가했을 때 벌어질 운동을 묻는다.
```
주어진 것: q, q̇, τ
구하는 것: q̈
```
"지금 모터에 이 토크를 흘려보내면 관절이 어떻게 가속하는가?"에 답하기 때문에, 물리 시뮬레이터가 매 time step마다 가속도를 구해 다음 상태를 적분할 때 반복해서 호출한다.

수식만 놓고 보면 forward dynamics는 manipulator equation을 q̈에 대해 풀기만 하면 끝나는 것처럼 보인다.

```
q̈ = M(q)^{-1} [τ - C(q, q̇)q̇ - g(q)]
```

하지만 여기에는 함정이 있다. n×n 관성 행렬 M(q)를 조립하고 그 역행렬(또는 Cholesky 분해)을 직접 구하면 O(n^3)의 연산 비용이 든다. 관절 수가 늘어날수록 시뮬레이션 속도가 급격히 느려지는 까닭이 여기에 있다.

### Articulated Body Algorithm (ABA)

이 병목을 푼 알고리즘이 Featherstone이 1983년에 제안한 **ABA(Articulated Body Algorithm)**로, forward dynamics 역시 O(n) 시간에 계산해 낸다. 핵심 아이디어는 각 링크를 단독 강체가 아니라, 자기 아래에 매달린 자식 링크들까지 하나로 묶은 복합 관절체(articulated body)로 취급하는 데 있다. 끝단에서 베이스로 거슬러 올라오며 각 서브트리의 유효 관성과 편향력(bias force)을 재귀적으로 합산하면, M(q) 행렬을 아예 만들거나 역행렬을 구하지 않고도 베이스에서 끝단으로 내려가며 관절 가속도 q̈을 바로 뽑아낼 수 있다.

ABA는 세 번의 순회로 이루어지며, 각 패스의 흐름은 아래 의사 코드와 같다.

```
ABA(model, q, q̇, τ):
    # Pass 1 (forward): 속도 전파
    for i = 1 to n:
        v[i] = v[parent(i)] + S[i] * q̇[i]
        c[i] = v[i] × (S[i] * q̇[i])  # Coriolis acceleration

    # Pass 2 (backward): articulated body inertia 계산
    for i = n downto 1:
        I_A[i] = I[i]  # spatial inertia
        p_A[i] = v[i] × (I[i] * v[i]) - f_ext[i]  # bias force
        # 자식 링크들의 기여를 합산 (생략)
        # 관절 가속도의 중간값 계산

    # Pass 3 (forward): 가속도 전파
    for i = 1 to n:
        q̈[i] = ...  # articulated body inertia를 이용해 계산
        a[i] = a[parent(i)] + S[i] * q̈[i] + c[i]

    return q̈
```

Pass 2에서 자식 링크의 관성을 관절 축 방향으로 투영하고 소거하는 수식은 부호와 좌표 변환이 매우 까다롭다. 직접 짜다가 미세한 버그가 숨어들기 쉬우므로, 실무에서는 Pinocchio나 Drake처럼 수년간 검증되고 SIMD/캐시 최적화까지 끝난 라이브러리를 가져다 쓰는 것이 안전하다.

### 시뮬레이터에서 하는 일

우리가 일상적으로 쓰는 물리 시뮬레이터들도 내부에서 서로 다른 전략으로 순동역학을 푼다. MuJoCo는 자체 개발한 forward dynamics 알고리즘을 사용하는데, 접촉 구속까지 하나의 convex optimization 솔버 안에서 함께 푸는 것이 특징이며, 트리 형태의 분기(branching) 구조에 맞춰 sparse factorization을 적용한다. Drake의 `MultibodyPlant`는 ABA를 기반으로 가속도를 계산하고 접촉은 time-stepping이나 hydroelastic 같은 별도 솔버로 처리하며, Bullet(PyBullet) 역시 Featherstone의 ABA를 쓰되 접촉은 sequential impulse solver로 해결한다.

Pinocchio에서는 `pin.aba` 한 줄로 순동역학을 호출할 수 있다. 아래 코드는 임의의 토크 `tau`를 입력해 `aba`로 관절 가속도 `qdd`를 구한 뒤, 그 `qdd`를 다시 `rnea`에 넣어 원래 토크 `tau`가 복원되는지 검증하는 예제다.

```python
# Pinocchio: forward dynamics (ABA)
import pinocchio as pin
import numpy as np

model = pin.buildModelFromUrdf("robot.urdf")
data = model.createData()

q = pin.randomConfiguration(model)
v = np.random.randn(model.nv)
tau = np.random.randn(model.nv)

# ABA: (q, v, τ) → q̈
qdd = pin.aba(model, data, q, v, tau)
print("Joint accelerations:", qdd)

# 검증: RNEA로 역계산
tau_check = pin.rnea(model, data, q, v, qdd)
print("Torque error:", np.linalg.norm(tau - tau_check))  # ≈ 0
```

이 코드를 돌려 보면 오차 노름이 부동소수점 정밀도 수준으로 0에 수렴한다. 수학적으로 `RNEA(q, v, ABA(q, v, τ)) ≈ τ`가 성립해, RNEA와 ABA가 정확히 역연산 관계에 있기 때문이다.

> **추천 자료**
> - Featherstone, *Rigid Body Dynamics Algorithms*, Ch. 7 — ABA의 원본 설명
> - MuJoCo documentation: Computation (https://mujoco.readthedocs.io/en/latest/computation/) — MuJoCo의 동역학 파이프라인 설명
> - Drake MultibodyPlant tutorial (https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_multibody_plant.html) — Drake의 동역학 계산 API

지금까지 다룬 RNEA와 ABA는 로봇이 허공에서 아무것에도 부딪히지 않는 자유 공간(free space)을 가정했다. 하지만 현실의 로봇은 바닥을 딛고 서고, 책상 위 컵을 잡고, 상자를 밀며 일한다. 로봇이 환경에 닿는 순간 동역학에는 어떤 새로운 어려움이 닥칠까?

---

## 5.6 접촉 역학

허공을 가로지르는 로봇의 움직임은 매끄러운 상미분방정식(ODE)으로 적을 수 있다. 그러나 로봇의 손이나 발이 외부 물체에 닿는 순간 상황은 급변한다. "물체 표면을 파고들 수 없다"는 부등식 구속 조건이 걸리고, 닿고 떨어지는 찰나마다 속도와 힘이 뚝 끊어지는 불연속성이 생기기 때문이다.

### Rigid Contact vs Compliant Contact

물리 엔진과 제어 이론이 이 접촉을 수식으로 다루는 방식은 크게 두 갈래로 나뉜다.

**Rigid contact (경성 접촉):**
- 두 물체가 서로 관통하지 않는다는 기하학적 구속 조건을 엄격하게 건다.
- 이때 발생하는 접촉력은 구속 조건을 유지하기 위한 Lagrange multiplier로 계산된다.
- 접촉과 비접촉이 전환되는 순간 불연속성이 튀어나와 수치적으로 다루기가 무척 까다롭다. 이 부등식과 상보성 조건(거리가 0이면 힘이 양수, 거리가 양수면 힘이 0)을 풀려고 LCP(Linear Complementarity Problem)나 NCP(Nonlinear Complementarity Problem) 정식화를 동원하며, Stewart-Trinkle의 LCP time-stepping 기법이 이 계보에 속한다.

**Compliant contact (연성 접촉):**
- 접촉면에 아주 단단한 가상의 스프링-댐퍼가 깔려 있다고 보고, 물체가 살짝 파고든 깊이(penetration depth)와 속도에 비례하는 반발력을 만들어 낸다.
- 물리 현상이 부드러운 함수로 바뀌어 수치 적분이 안정적이고 구현도 한결 수월하다.
- 스프링 강성(stiffness)을 높이면 rigid contact에 가까워지지만, 그만큼 방정식이 뻣뻣해져(stiff ODE) 발산을 막으려면 적분 time step을 아주 잘게 쪼개야 한다.
- MuJoCo의 기본 접촉 모델이 바로 이 계열이다.

### Coulomb Friction Model

두 물체가 맞닿으면 법선 방향의 수직항력뿐만 아니라 접선 방향의 마찰(friction)도 반드시 따라온다. 로보틱스에서 가장 널리 쓰는 모델은 고전적인 Coulomb 마찰 법칙이다.

```
|f_t| ≤ μ f_n          (static friction: 정지 마찰)
|f_t| = μ f_n, f_t ∥ -v_t  (sliding friction: 운동 마찰)
```

여기서 `f_t`는 접선 방향 마찰력, `f_n`은 법선 방향 수직항력, `μ`는 마찰 계수, `v_t`는 접촉면에서의 접선 상대 속도다.

식은 간결해 보이지만, 시뮬레이션이나 최적화에 그대로 넣으면 세 가지 골칫거리가 생긴다. 첫째, 물체가 정지해 있다가 미끄러지기 시작하는(stick → slip) 경계에서 마찰력이 불연속적으로 튄다. 둘째, 3차원 공간에서 `|f_t| ≤ μ f_n`이 그리는 원뿔(friction cone)은 비선형 영역이라, 이를 빠른 선형 최적화(LP/QP)로 풀려고 다각형 피라미드(friction pyramid)로 선형 근사하면 방향에 따라 마찰력이 부정확해진다. 셋째, 특정 기하 구조와 높은 마찰 계수가 겹치면 rigid contact와 Coulomb 마찰을 연립한 문제 자체가 해가 없거나 여러 해가 동시에 존재하는 **Painlevé paradox**에 빠지기도 한다.

### Contact-Rich Manipulation이 어려운 이유

실제 작업에서 로봇이 물체를 집어 돌리고, 구멍에 축을 끼워 넣는(peg-in-hole, in-hand manipulation) 동작은 접촉이 끊임없이 바뀌는 고난도 문제다. 접촉이 생기고 떨어지거나(contact/no-contact), 붙어 있다가 미끄러지는(stick/slip) 모드가 수시로 전환될 뿐 아니라, 모드마다 지배 방정식이 다르고 전환 시점조차 미리 알기 어렵다. 게다가 접촉이 일어나는 찰나의 충격(impact)으로 속도가 불연속적으로 꺾이며, 실제 물체의 마찰 계수나 표면 강성을 정확히 알아내기 어려워 시뮬레이션과 현실 사이의 간극(sim-to-real gap)이 크게 벌어진다. 접촉점이 여러 개로 늘어나면 각 접점의 접촉 여부와 정지·미끄럼 조합이 기하급수적으로 불어나는 조합론적 복잡성까지 겹쳐, 계획과 제어를 한층 어렵게 만든다(정확한 모드 수는 마찰 모델과 접선 방향 이산화 방식에 따라 달라진다).

### 시뮬레이터마다 접촉 처리가 다른 이유

접촉 방정식을 정확하고 빠르고 안정적으로 한꺼번에 푸는 완벽한 수치 기법은 아직 없기에, 물리 시뮬레이터마다 정확도·계산 속도·수치 안정성 가운데 무엇을 우선순위에 두느냐에 따라 설계가 갈린다.

- **MuJoCo**: compliant contact와 convex optimization을 결합해, 약간의 관통을 soft contact 모델의 일부로 허용하는 대신 빠르고 수치적으로 매우 안정적인 해를 낸다. 대규모 강화학습(RL) 연구에서 널리 쓰이는 배경에도 이 안정성이 자리한다.
- **Drake**: compliant point contact(TAMSI/SAP 이산 솔버)와 더불어, 접촉면 전체의 압력 분포까지 계산하는 hydroelastic contact를 지원한다. 둘 다 compliant 계열이며, 물리적 충실도가 높은 만큼 계산 비용이 커질 수 있다.
- **Bullet (PyBullet)**: 게임과 VR 엔진에서 출발해 velocity-level LCP와 sequential impulse 방식을 채택했다. 계산 속도에 최적화되어 있지만, 접촉 물리의 정밀도는 설정과 작업 성격에 따라 크게 달라지므로 MuJoCo나 Drake보다 유리한지는 목표 작업에서 직접 비교해 판단해야 한다.
- **DART**: LCP 기반 rigid contact를 엄밀하게 구현해 왔으나, MuJoCo나 Drake에 비해 사용자 생태계는 작은 편이다.

따라서 시뮬레이터를 고를 때는 목표로 하는 접촉 모델의 물리적 정밀도, 미분 가능성(gradient 지원 여부), 병렬 실행 속도, 그리고 실제 하드웨어 실험과의 검증 사례를 함께 따져 보아야 한다. 아래 코드는 MuJoCo 시뮬레이션 한 스텝을 진행한 뒤, 현재 발생한 모든 접촉점의 위치·법선·관통 깊이를 읽어 내는 전형적인 방법이다.

```python
# MuJoCo에서 접촉 정보 접근
import mujoco
import numpy as np

model = mujoco.MjModel.from_xml_path("scene.xml")
data = mujoco.MjData(model)

mujoco.mj_step(model, data)

# 접촉점 개수
n_contacts = data.ncon
print(f"Number of contacts: {n_contacts}")

# 각 접촉의 정보
for i in range(n_contacts):
    contact = data.contact[i]
    print(f"Contact {i}:")
    print(f"  Position: {contact.pos}")
    print(f"  Normal: {contact.frame[:3]}")  # 접촉 법선
    print(f"  Signed distance (음수면 관통): {contact.dist}")
    print(f"  Geom pair: ({contact.geom1}, {contact.geom2})")
```

여기서 `contact.dist`가 음수라는 사실은 두 형상이 그만큼 서로 파고들어 반발력을 만들어 내고 있다는 뜻이다. 더 자세한 접촉 모델링 이론과 시뮬레이터 구현은 아래 자료에서 확인할 수 있다.

> **추천 자료**
> - Stewart, "Rigid-Body Dynamics with Friction and Impact", SIAM Review 2000 — 접촉 역학의 수학적 기초
> - Todorov, "Convex and analytically-invertible dynamics with contacts and constraints", ICRA 2014 — MuJoCo의 접촉 모델 논문
> - [Todorov et al., "MuJoCo: A Physics Engine for Model-Based Control" (IROS 2012)](https://ieeexplore.ieee.org/document/6386109) — MuJoCo의 convex contact formulation과 velocity stepping을 설명한 원 논문
> - Drake의 접촉 모델 documentation (https://drake.mit.edu/doxygen_cxx/group__hydroelastic__user__guide.html) — Hydroelastic contact 설명
> - Russ Tedrake, *Underactuated Robotics*, Ch. "Contact" (https://underactuated.csail.mit.edu/) — 접촉 역학 개론

앞선 5.2절과 5.5절의 RNEA, ABA 의사 코드에서 우리는 `S[i]`, `v[i]`, `f[i]` 같은 기호를 별다른 유도 없이 사용했다. Pinocchio와 Drake의 소스 코드가 내부에서 어떻게 이토록 깔끔한 수식을 구현하고 있는지 이해하려면, 이제 Featherstone이 정립한 6차원 공간 벡터 대수(spatial algebra)의 언어를 들여다볼 차례다.

---

## 5.7 Featherstone 알고리즘과 Spatial Algebra [심화]

3차원 강체 동역학을 처음 구현해 보면 병진 운동(힘 `F`, 선속도 `v`)과 회전 운동(모멘트 `n`, 각속도 `ω`)의 식이 쌍둥이처럼 닮아 있으면서도, 기준점을 옮길 때마다 외적 항이 튀어나와 코드가 금세 지저분해진다는 점을 느끼게 된다. Featherstone의 spatial vector algebra는 3차원 병진과 3차원 회전을 하나의 6차원 벡터로 묶어, 동역학 알고리즘을 놀라울 만큼 간결하게 기술하는 수학적 틀이다.

### Spatial Vectors (6D Vectors)

3차원 공간에서 강체의 운동 자유도는 병진 3자유도와 회전 3자유도를 합쳐 총 6자유도다. 이 둘을 한데 묶은 6D 벡터를 spatial vector라 부르며, 움직임을 나타내는 motion vector와 작용을 나타내는 force vector의 두 종류로 나뉜다.

**Motion vector (spatial velocity, twist):**
```
v = [ω; v_O]
```
위쪽 3개 성분은 각속도(ω), 아래쪽 3개 성분은 공간상의 기준점 O를 지나는 점의 선속도(v_O)를 뜻한다.

**Force vector (spatial force, wrench):**
```
f = [n_O; f]
```
위쪽 3개 성분은 기준점 O 주위의 모멘트(n_O), 아래쪽 3개 성분은 선형 힘(f)이다.

여기서 눈여겨볼 점은 두 벡터의 성분 배열 순서가 우연이 아니라는 사실이다. spatial force와 spatial velocity를 내적하면 정확히 일률(power) `P`가 되도록 정의되어 있다.

```
P = f^T v = n_O · ω + f · v_O
```

Featherstone은 일률이 좌표계 선택에 무관한 물리량이라는 성질에서 출발해 spatial vector와 그 좌표 변환 행렬을 정의했고, 덕분에 motion vector와 force vector의 쌍대(dual) 구조가 깔끔하게 맞아떨어진다.

### Spatial Inertia

병진 질량과 회전 관성 역시 6×6 행렬 하나로 합칠 수 있다. 기준점 O에서 바라본 강체의 6×6 spatial inertia matrix는 질량 `m`, 기준점에서 질량중심까지의 벡터 `c`, 질량중심 주위의 회전 관성 `I_cm`을 모두 한 행렬에 담는다.

```
I_sp = [ I_cm + m·[c]×[c]×^T    m·[c]× ]
       [ m·[c]×^T                 m·1    ]
```

여기서 `[c]×`는 벡터 `c`의 외적에 해당하는 반대칭 행렬(skew-symmetric matrix)이다.

이 6×6 표현을 쓰면 복잡한 계산이 단순한 행렬 연산으로 바뀐다. 여러 강체를 하나의 덩어리로 묶을 때도 같은 좌표계와 기준점으로 표현해 두기만 하면 단순 덧셈(`I_composite = I_1 + I_2 + ...`)으로 합성 관성을 구할 수 있고, 다른 좌표계로 관성 행렬을 옮길 때도 대칭성을 보존하는 합동 변환(`I_B = X^T I_A X`) 한 번으로 끝난다.

### RNEA와 ABA의 Spatial Vector 표현

이제 5.2절과 5.5절의 의사 코드가 왜 그토록 짧았는지 명확해진다. 식에 나온 `v[i]`와 `f[i]`가 각각 6D spatial velocity와 spatial force이고, `S[i]`는 관절 i가 허용하는 6D 움직임 방향인 motion subspace(회전 관절이면 `[e_z; 0]`, 직동 관절이면 `[0; e_z]`)였다.

6D 공간 벡터를 쓰면 회전 관절이든 직동 관절이든 `S[i]` 벡터 값만 바뀔 뿐 나머지 알고리즘 코드는 한 줄도 고칠 필요가 없다. Pinocchio와 Drake가 수십 종류의 관절을 일관된 코드로 빠르게 처리하는 비결도 내부의 spatial algebra 덕분이다.

### Pinocchio에서 Spatial Quantities 접근

실제로 Pinocchio는 각 관절 프레임의 spatial velocity뿐 아니라, Composite Rigid Body Algorithm(CRBA)으로 조립한 질량 행렬 `M(q)`와 다음 절에서 다룰 centroidal momentum 행렬 `A_g`까지 바로 꺼내 쓸 수 있는 API를 제공한다.

```python
import pinocchio as pin
import numpy as np

model = pin.buildModelFromUrdf("robot.urdf")
data = model.createData()

q = pin.randomConfiguration(model)
v = np.random.randn(model.nv)

# 순기구학 + 속도 계산
pin.forwardKinematics(model, data, q, v)

# 각 프레임의 spatial velocity
for i in range(model.njoints):
    # 월드 프레임 기준 spatial velocity
    v_world = pin.getVelocity(model, data, i, pin.ReferenceFrame.WORLD)
    print(f"Joint {i} spatial velocity (world): {v_world}")

# Composite Rigid Body Algorithm (CRBA): M(q) 계산
M = pin.crba(model, data, q)
print("Mass matrix M(q):\n", data.M)

# Centroidal momentum matrix
pin.computeCentroidalMap(model, data, q)
Ag = data.Ag  # 6 x nv matrix
# h = Ag @ v 가 centroidal momentum (선운동량 + 각운동량)
```

프로토타이핑은 Python으로 빠르게 진행하더라도, 1kHz 이상의 실시간 제어 루프에 올릴 때는 Python 인터프리터의 오버헤드를 없애려고 C++ API를 쓴다. Pinocchio의 C++ 인터페이스는 Eigen 행렬 라이브러리를 기반으로 하며 함수 이름과 흐름이 Python과 거의 같아 코드를 옮기기도 수월하다.

```cpp
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/parsers/urdf.hpp>

pinocchio::Model model;
pinocchio::urdf::buildModel("robot.urdf", model);
pinocchio::Data data(model);

Eigen::VectorXd q = pinocchio::randomConfiguration(model);
Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);
Eigen::VectorXd tau = Eigen::VectorXd::Random(model.nv);

// Inverse dynamics (RNEA)
Eigen::VectorXd tau_id = pinocchio::rnea(model, data, q, v, Eigen::VectorXd::Zero(model.nv));

// Forward dynamics (ABA)
Eigen::VectorXd qdd = pinocchio::aba(model, data, q, v, tau);
```

> **추천 자료**
> - Featherstone, *Rigid Body Dynamics Algorithms*, Ch. 2 — Spatial vector algebra의 원본 설명
> - Featherstone, "A Beginner's Guide to 6-D Vectors" (IEEE Robotics & Automation Magazine, 2010) — 교과서보다 접근하기 쉬운 소개 논문
> - Pinocchio GitHub (https://github.com/stack-of-tasks/pinocchio) — 소스 코드 자체가 spatial algebra의 좋은 구현 예시이다

지금까지 살펴본 산업용 매니퓰레이터는 베이스가 작업대 바닥에 볼트로 단단히 고정되어 있었다. 하지만 휴머노이드, 사족 보행 로봇, 드론처럼 스스로 이동하는 로봇은 베이스 자체가 3차원 공간에 떠서 움직인다. 베이스의 구속이 풀리는 순간 동역학의 구조는 어떻게 바뀔까?

---

## 5.8 부유 베이스 시스템 [심화]

고정 베이스 매니퓰레이터와 달리, 보행 로봇이나 드론, 수중 로봇은 몸통(base)이 공간 속에서 자유롭게 위치와 자세를 바꾼다. 이렇게 몸체가 고정되지 않은 시스템을 **부유 베이스(floating-base)** 로봇이라 하며, 베이스의 6자유도가 전체 상태 변수에 새로 합류하면서 동역학의 성질도 사뭇 달라진다.

### 부유 베이스의 Configuration

관절이 n개인 고정 베이스 로봇의 configuration은 단순히 관절 각도 벡터 `q ∈ R^n`으로 적을 수 있다. 반면 부유 베이스 로봇은 베이스의 위치·자세와 관절 각도를 함께 묶어 표현해야 한다.

```
q = [q_base; q_joints]
```

여기서 `q_base`는 3차원 위치(3개 성분)와 회전 자세(오일러 각 3개 또는 쿼터니언 4개)로 이루어진 SE(3)의 원소다. 이때 회전 표현으로 특이점 없는 쿼터니언을 선택하면 위치 벡터 `q`의 차원(`nq`)과 속도 벡터 `v`의 차원(`nv`)이 달라진다(구체적으로 `nq = nv + 1`). `q`와 `v`가 같은 유클리드 벡터 공간에 놓여 있지 않기 때문에, 상태 적분이나 차분을 수행할 때 단순 덧셈 `q += v*dt`를 쓰면 단위 쿼터니언 성질이 깨진다. 반드시 리 군(Lie group) 구조를 보존하는 `pin.integrate(model, q, v*dt)` 함수를 사용해야 한다.

### Underactuated Systems

보행 로봇이나 쿼드로터 같은 부유 베이스 시스템은 대체로 부족구동(underactuated) 시스템이다. 보행 로봇의 몸통에는 허공을 밀어낼 모터가 따로 달려 있지 않아 발이 지면을 밀 때 생기는 반력으로만 몸통을 움직이고, 일반 쿼드로터도 4개의 프로펠러 추력만으로 6자유도 공간을 다룬다. 물론 시스템이 항상 부족구동인 것은 아니다. 추력기를 여섯 방향 이상으로 배치한 우주선이나 전방향 멀티로터는 완전 구동(fully actuated)이고, 보행 로봇 역시 여러 발이나 손이 지면을 충분히 넓게 짚고 마찰 조건을 만족하면 베이스 6자유도를 모두 제어할 수 있다. 핵심은 독립적인 입력 수와 현재 맺고 있는 접촉 조건이 제어 가능성을 결정한다는 점이다.

이를 확인하려면 전체 manipulator equation을 베이스 행(6차원)과 관절 행(n차원)으로 나누어 적어 보면 된다.

```
[ M_bb  M_bj ] [ a_base  ]   [ C_b ]   [ g_b ]   [  0  ]   [ J_{c,b}^T ]
[ M_jb  M_jj ] [ q̈_joints] + [ C_j ] + [ g_j ] = [ τ_j ] + [ J_{c,j}^T ] λ
```

등호 오른쪽 첫 번째 벡터의 위쪽 성분이 `0`이라는 사실이 모든 것을 말해 준다. 베이스 자유도에는 직접 토크를 거는 액추에이터가 없다는 뜻이다. 식에서 `λ`는 k차원의 접촉력 벡터이고, `J_{c,b}`(k×6)와 `J_{c,j}`(k×n)는 전체 접촉 자코비안 `J_c`(k×(6+n))를 베이스 열과 관절 열로 분할한 블록 행렬이다. 결국 베이스 가속도 `a_base`는 오직 중력과 접촉력 `λ`만으로 결정된다. 고정 베이스 매니퓰레이터라면 원하는 토크를 관절 모터에 바로 명령하면 끝나지만, 보행 로봇은 관절 토크 `τ_j`를 조절해 발끝에 알맞은 접촉력 `λ`를 만들어 내야만 몸통을 원하는 방향으로 가속시킬 수 있다. 이것이 보행(locomotion) 제어가 매니퓰레이션보다 한층 까다로운 근본 이유다.

### Centroidal Dynamics

수십 개 관절이 얽힌 `(6+n)`자유도 전신 동역학을 매번 통째로 최적화하기는 계산량이 너무 크다. 그래서 보행 로봇 연구에서는 시스템 전체의 운동량(momentum)을 전체 질량중심(Center of Mass, CoM) 한 점에 모아 표현하는 **centroidal dynamics**를 널리 쓴다.

**선운동량 (linear momentum):**
```
p = m v_CoM = Σ m_i v_i
```

**각운동량 (angular momentum about CoM):**
```
L = Σ_i [ (r_i - r_CoM) × (m_i v_i) + I_i ω_i ]
```

뉴턴-오일러 법칙을 시스템 전체로 합산하면 내부의 모든 관절 토크와 내력이 상쇄되고, 오직 중력과 외부 접촉력만이 전체 운동량의 시간 변화율을 결정한다는 놀라운 식을 얻는다.

**Centroidal momentum의 시간 미분:**
```
ṗ = m g + Σ f_contact
L̇ = Σ (r_contact - r_CoM) × f_contact
```

이 식이 바로 로봇의 균형(balance)을 지배한다. 정지 상태에서는 CoM의 수직 투영이 지지 영역(support polygon) 안에 머물러야 넘어지지 않고(정적 안정 조건), 걷는 동안에는 접촉면의 압력 중심인 ZMP(Zero Moment Point)가 지지 영역 안에 들어가야 발바닥이 들리지 않는다(ZMP 조건). 이 두 조건을 3차원 동적 보행까지 아울러 일반화하면 결국 centroidal momentum `p`와 `L`을 목표 궤적대로 조절하는 문제로 귀결된다.

다시 말해, `(6+n)`차원에 달하는 전신 동역학에서 균형과 이동에 직결되는 핵심만 추려 6차원(선운동량 3 + 각운동량 3) 공간으로 압축한 모델이 centroidal dynamics다. 따라서 보행 제어에서는 먼저 이 6차원 공간에서 원하는 운동량 궤적과 이를 만들어 내는 접촉력 패턴을 계획한 뒤, 전신 관절 수준으로 분해해 토크를 구하는 전략을 쓴다.

Pinocchio에서는 `JointModelFreeFlyer()`를 붙여 부유 베이스 모델을 만들고, 아래처럼 centroidal momentum `hg`와 centroidal momentum matrix `A_g(q)`, 그리고 CoM의 위치·속도를 한꺼번에 계산할 수 있다.

```python
# Pinocchio에서 centroidal dynamics 계산
import pinocchio as pin
import numpy as np

model = pin.buildModelFromUrdf("humanoid.urdf", pin.JointModelFreeFlyer())
data = model.createData()

q = pin.randomConfiguration(model)
v = np.random.randn(model.nv)

# Centroidal momentum
pin.computeCentroidalMomentum(model, data, q, v)
h = data.hg  # 6D centroidal momentum, Pinocchio 순서는 [linear; angular]
print("Angular momentum:", h.angular)
print("Linear momentum:", h.linear)

# Centroidal momentum matrix: h = A_g(q) * v
pin.computeCentroidalMap(model, data, q)
Ag = data.Ag  # 6 x nv
h_check = Ag @ v
print("Centroidal momentum (via Ag):", h_check)

# CoM 위치 및 속도
pin.centerOfMass(model, data, q, v)
print("CoM position:", data.com[0])
print("CoM velocity:", data.vcom[0])
```

### Centroidal Dynamics 기반 Locomotion 제어의 구조

실제 보행 로봇 제어기에서는 앞서 살펴본 접촉 일정과 centroidal dynamics, 그리고 전신 동역학이 계층적인 파이프라인으로 맞물려 돌아간다.

```
[Contact Schedule] → [Centroidal Trajectory Optimization] → [Whole-Body Control] → [Joint Torques]

1단계: 언제 어떤 발이 땅에 닿는지 결정 (gait pattern)
2단계: Centroidal dynamics를 만족하는 CoM 궤적 + 접촉력 계획
3단계: Centroidal 목표를 달성하면서 관절 수준의 여러 제약을 만족하는 토크 계산
4단계: 모터에 토크 명령
```

이 파이프라인의 3단계에 해당하는 Whole-Body Control은 다음 장(6장 §6.7)에서 QP 기반 제어기 설계와 함께 본격적으로 다룬다.

> **추천 자료**
> - Orin et al., "Centroidal Dynamics of a Humanoid Robot", Autonomous Robots 2013 — Humanoid의 centroidal dynamics를 분석한 논문
> - Wensing et al., "Optimization-Based Control for Dynamic Legged Robots", IEEE T-RO 2024 (arXiv 2022) — Locomotion 제어 survey
> - Russ Tedrake, *Underactuated Robotics*, Ch. "Walking" (https://underactuated.csail.mit.edu/) — Underactuated 시스템과 보행의 관계
> - Carpentier, Mansard, "Pinocchio: fast forward and inverse dynamics for poly-articulated systems" (https://github.com/stack-of-tasks/pinocchio) — Pinocchio의 centroidal dynamics 구현

---

## 참고 자료

동역학을 더 깊이 공부할 때 어떤 교재와 도구를 고를지는 현재 배경지식과 목표에 따라 달라진다.

**학부 3-4학년, 동역학 입문:**

> - Spong, Hutchinson, Vidyasagar, *Robot Modeling and Control* — 학부 수준의 입문 교재. Manipulator equation 유도가 상세하다.
> - Craig, *Introduction to Robotics: Mechanics and Control* — 산업 현장 관점. 실용적이고, 수학적 깊이는 얕은 편이다.

**대학원, 수학적으로 엄밀한 이해가 필요할 때:**

> - Murray, Li, Sastry, *A Mathematical Introduction to Robotic Manipulation* (https://www.cds.caltech.edu/~murray/mlswiki/) — Lie group/algebra 관점에서 동역학을 엄밀하게 다룬다. 무료 PDF가 공개되어 있으나, 입문자가 바로 읽기에는 어렵다.
> - Featherstone, *Rigid Body Dynamics Algorithms* — Spatial vector algebra, RNEA, ABA, composite rigid body algorithm을 체계적으로 다루는 대학원 수준의 참고서.

**동역학 + 제어 통합:**

> - Russ Tedrake, *Underactuated Robotics* (https://underactuated.csail.mit.edu/) — 동역학 모델을 제어와 최적화에 어떻게 활용하는지를 다룬다. MIT OCW에 강의 영상도 있다. 무료.

**라이브러리 & 도구:**

> - Pinocchio (https://github.com/stack-of-tasks/pinocchio) — 순수 동역학 계산용 C++/Python 라이브러리. RNEA, ABA, CRBA, centroidal dynamics, analytical derivatives를 폭넓게 지원하며, Pinocchio 3.x에서는 CasADi·CppAD와 연동해 자동 미분(autodiff)까지 수행한다.
> - Drake (https://drake.mit.edu/) — 시뮬레이션 + 최적화 + 제어를 통합한 프레임워크. MultibodyPlant가 동역학 엔진이다. Mathematical programming 인터페이스가 강력해서 trajectory optimization에 특히 쓸모 있다.
> - MuJoCo (https://mujoco.org/) — DeepMind가 관리하는 물리 시뮬레이터. 접촉을 포함한 로봇 학습 연구에 널리 쓰인다.
> - PyBullet (https://pybullet.org/) — Bullet Physics의 Python 인터페이스. 진입 장벽이 낮아 교육용으로 적합하지만, 접촉 물리의 정확도와 속도는 관절체(articulated body) 구성, 접촉 설정, 적분 시간 간격에 따라 갈리므로 목표 작업에서 직접 비교해 선택한다.

---

## 발전 과정

```
1687 ── Newton의 운동 법칙 (Principia Mathematica)
1788 ── Lagrange의 해석역학 (Mécanique Analytique)
1965 ── Uicker, 4×4 행렬로 공간 링크 동역학 해석 (Northwestern 박사논문)
1980 ── Luh, Walker, Paul의 Newton-Euler 재귀 알고리즘 (RNEA, O(n))
1983 ── Featherstone의 Articulated Body Algorithm (ABA, O(n) forward dynamics)
1987 ── Featherstone, *Robot Dynamics Algorithms* (Kluwer) — spatial vector 기반 동역학 알고리즘 정리
2000 ── Stewart의 rigid contact dynamics 수학적 정리 (SIAM Review)
2001 ── ODE (Open Dynamics Engine) — 초기 오픈소스 물리 엔진
2012 ── MuJoCo 공개 (Todorov, Erez, Tassa)
2015 ── Pinocchio 1.0 공개 (LAAS-CNRS)
2016 ── Bullet Physics 2.x → PyBullet 인터페이스
2021 ── DeepMind가 MuJoCo 인수, 무료 공개 (소스 공개는 2022년 5월 Apache 2.0)
2022 ── Drake 1.0 (MIT → Toyota Research Institute)
2022 ── MuJoCo 2.3 릴리스 (implicitfast 적분기는 2023년 3월 2.3.3에서 추가, elliptic friction cone은 이전부터 제공)
2023 ── MuJoCo 3.0: MJX (JAX backend for GPU parallelism)
2024 ── Pinocchio 3.0 정식 릴리스: CasADi·CppAD autodiff 지원
```

---

## 요약

1. 로봇 동역학의 표준 뼈대는 manipulator equation `M(q)q̈ + C(q,q̇)q̇ + g(q) = τ`다.
2. 실시간 계산에서는 역동역학(τ 계산)에 RNEA, 순동역학(q̈ 계산)에 ABA를 쓰며, 두 알고리즘 모두 O(n) 복잡도를 갖는다.
3. 환경과의 접촉은 부등식 구속과 불연속성을 끌어들여 시뮬레이션과 제어 난도를 급격히 높이므로, 목표 작업에 맞는 접촉 모델과 시뮬레이터를 함께 검토해야 한다.
4. 보행 로봇처럼 베이스가 고정되지 않은 시스템에서는 전체 자유도를 6차원 운동량으로 압축한 centroidal dynamics가 계획과 제어의 핵심 도구가 된다.
5. 실무에서는 Pinocchio나 Drake 같은 검증된 라이브러리를 활용하되, 각 API가 어떤 좌표계와 중력·접촉 가정을 전제로 계산하는지 반드시 확인한다.

이제 우리는 힘과 토크가 로봇의 움직임을 어떻게 지배하는지 수학적으로 모델링할 수 있게 되었다. 하지만 현실 세계에서는 모델에 들어 있지 않은 마찰, 적재 물체 무게 변화, 예기치 못한 외란이 끊임없이 끼어든다. 다음 6장에서는 센서 피드백으로 이 불확실성을 이겨 내고 로봇이 원하는 상태와 힘을 정확히 지켜 내도록 만드는 제어 이론을 다룬다.
