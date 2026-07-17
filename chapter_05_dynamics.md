# Ch.5 — 강체 역학 & 동역학 (Rigid Body Dynamics)


---

## 5.1 왜 동역학을 배우는가

기구학(kinematics)이 "로봇이 *어디로* 움직이는가"를 다룬다면, 동역학(dynamics)은 "*어떤 힘으로* 움직이는가"를 다룬다. 기구학만으로 로봇을 제어할 수 있는 경우도 있다. 느리게 움직이는 산업용 매니퓰레이터가 그렇다. 관절 속도가 충분히 낮으면 관성력과 코리올리 힘이 무시할 만하고, PID 제어기가 나머지 오차를 보정한다.

그런데 다음과 같은 상황에서는 동역학 없이 버틸 수 없다:

- **고속 매니퓰레이션**: 산업 현장에서 cycle time을 줄이려면 로봇을 빠르게 움직여야 한다. 빠르게 움직이면 관성력, 원심력, 코리올리 힘이 커진다. 이걸 무시하면 경로 추종 오차가 커지고, 최악의 경우 관절 모터가 포화(saturation)된다.
- **보행 로봇 (legged robots)**: 두 발이든 네 발이든, 지면과의 접촉력을 관리하면서 넘어지지 않아야 한다. 이건 순수하게 동역학 문제다.
- **시뮬레이션**: 물리 시뮬레이터는 힘/토크를 받아서 가속도를 계산하고, 이를 적분하여 다음 상태를 구한다. 동역학 모델이 곧 시뮬레이터의 핵심이다.
- **최적 제어 (optimal control)**: 에너지를 최소화하거나 시간을 최소화하는 궤적을 찾으려면 동역학 모델이 constraints로 들어간다.
- **충돌/접촉 처리**: 물체를 잡거나(grasp), 밀거나(push), 던지는(throw) 작업은 접촉 역학 없이 불가능하다.

기구학은 로봇의 "기하학"이고, 동역학은 로봇의 "물리학"이다. 기하학만으로는 세상이 움직이지 않는다.

> **추천 자료**
> - Featherstone, *Rigid Body Dynamics Algorithms*, Chapter 1 — 동역학이 왜 필요한지를 간결하게 설명한다.
> - Russ Tedrake, *Underactuated Robotics* Ch.1 (https://underactuated.csail.mit.edu/) — 동역학 기반 제어가 왜 기구학 기반보다 강력한지 직관적으로 보여준다.

---

## 5.2 뉴턴-오일러 역학 (Newton-Euler Formulation)

### 기본 원리

뉴턴 역학은 병진 운동과 회전 운동을 따로 기술한다.

**병진 운동 (translational motion):**
```
F = ma
```
물체에 작용하는 합력 F는 질량 m과 질량중심(CoM) 가속도 a의 곱이다.

**회전 운동 (rotational motion):**
```
τ = Iα + ω × (Iω)
```
물체에 작용하는 합토크 τ는 관성 텐서 I와 각가속도 α의 곱에 자이로스코픽 항 ω × (Iω)를 더한 것이다. 2D에서는 뒤의 항이 사라져 τ = Iα로 단순해지지만, 3D에서는 이 항을 포함해야 한다. 빠뜨리면 시뮬레이션에서 비현실적인 회전 거동이 나타난다.

### Recursive Newton-Euler Algorithm (RNEA)

RNEA는 직렬 매니퓰레이터(serial manipulator)의 역동역학(inverse dynamics)을 푸는 가장 효율적인 방법이다. 두 pass로 구성된다.

**Forward pass (base → end-effector):** 각 링크의 속도와 가속도를 순방향으로 전파한다. 링크 i의 속도는 링크 i-1의 속도에 관절 i의 기여분을 더한 것이다.

**Backward pass (end-effector → base):** 각 링크에 작용하는 힘과 토크를 역방향으로 전파한다. 뉴턴-오일러 방정식으로 링크 i에 필요한 합력/합토크를 구하고, 이를 관절 i의 토크로 변환한다.

왜 재귀적(recursive)으로 푸는가? 단일 강체의 동역학은 O(1)이다. n개의 링크를 순차적으로 처리하면 O(n)이다. 반면 라그랑주 방정식을 직접 풀면 M(q) 행렬 계산에 O(n^3)이 걸린다. 관절 수가 많은 로봇(예: humanoid의 30+ DOF)에서 이 차이는 실시간 제어 가능 여부를 결정한다.

RNEA의 의사 코드를 정리하면 다음과 같다:

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

여기서 S[i]는 관절 i의 motion subspace matrix (관절 축 방향), v[i]는 링크 i의 공간 속도, I[i]는 링크 i의 공간 관성이다. Featherstone의 spatial vector 표기를 따랐다. 5.7절에서 더 자세히 다룬다.

### 실제 코드: Pinocchio

Pinocchio는 RNEA를 포함한 다양한 동역학 알고리즘을 구현한 C++/Python 라이브러리이다. 다음은 RNEA로 역동역학을 계산하는 예시다:

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

중력 보상(gravity compensation)은 RNEA에서 v=0, a=0을 넣으면 바로 나온다. 이것만으로도 로봇이 중력에 처지지 않는다. 로봇 팔을 처음 세울 때 첫 번째로 구현하는 제어기다.

Drake에서의 동일한 계산:

```python
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.parsing import Parser
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

> **추천 자료**
> - Featherstone, *Rigid Body Dynamics Algorithms*, Chapter 5 — RNEA의 원본 설명
> - Pinocchio documentation (https://github.com/stack-of-tasks/pinocchio) — 실무에서 RNEA를 가장 쉽게 써볼 수 있는 라이브러리
> - Luh, Walker, Paul (1980), "On-Line Computational Scheme for Mechanical Manipulators" — RNEA의 원 논문

---

## 5.3 라그랑주 역학 (Lagrangian Mechanics)

### Lagrangian이란

라그랑주 역학은 힘과 토크를 직접 다루는 대신, 에너지를 통해 운동 방정식을 유도한다.

**Lagrangian** L은 다음과 같이 정의된다:

```
L(q, q̇) = T(q, q̇) - V(q)
```

여기서 T는 시스템의 운동에너지(kinetic energy), V는 위치에너지(potential energy)이다.

**Euler-Lagrange 방정식:**

```
d/dt (∂L/∂q̇_i) - ∂L/∂q_i = τ_i
```

각 일반화 좌표(generalized coordinate) q_i에 대해 이 방정식을 세우면, 시스템의 운동 방정식이 나온다.

좌표계를 자유롭게 선택할 수 있다. 뉴턴 역학에서는 각 링크의 질량중심 위치와 자세를 월드 프레임에서 표현하고 구속 조건(constraint)을 별도로 관리해야 한다. 라그랑주 역학에서는 관절 각도를 일반화 좌표로 선택하면 구속 조건이 자동으로 사라진다.

### Manipulator Equation

n-DOF 직렬 매니퓰레이터의 Euler-Lagrange 방정식을 정리하면 다음과 같은 표준 형태가 나온다:

```
M(q)q̈ + C(q, q̇)q̇ + g(q) = τ
```

각 항의 의미:

- **M(q)** — 질량/관성 행렬(mass/inertia matrix). n×n 대칭 양정치(symmetric positive definite) 행렬이다. 로봇의 자세 q에 따라 달라진다 — 팔을 쭉 펴면 관성이 커지고, 접으면 작아지는 것과 같은 원리다.
- **C(q, q̇)q̇** — 코리올리 및 원심력 항(Coriolis and centrifugal terms). 관절들이 동시에 움직일 때 발생하는 관성 커플링이다. 느리게 움직이면 무시해도 되지만, 빠르게 움직이면 이 항이 크다.
- **g(q)** — 중력 벡터(gravity vector). 로봇이 중력장에 있을 때 각 관절에 작용하는 중력 토크이다.
- τ — 관절 토크 벡터. 모터가 내는 힘이다. 마찰(friction)은 보통 별도로 모델링하여 더한다.

제어, 시뮬레이션, 궤적 최적화 전부 이 방정식에서 출발한다.

### 2-Link Planar Arm 예제

2-link planar arm은 동역학 입문에서 빠지지 않는 예제이다. 노트에 직접 유도해보는 것을 강력히 권한다 — 한 번 해보면 n-DOF 경우의 구조가 명확해진다.

설정:
- 링크 길이: l_1, l_2
- 링크 질량: m_1, m_2 (질량이 링크 끝에 집중된다고 가정 — point mass)
- 관절 각도: q_1, q_2 (base에서부터)
- 중력: g (아래 방향)

**운동에너지 T:**

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

각 질량의 속도를 구하고 T = (1/2)m_1 v_1^2 + (1/2)m_2 v_2^2 을 전개하면:

```
T = (1/2)(m_1 + m_2) l_1^2 q̇_1^2
  + (1/2) m_2 l_2^2 (q̇_1 + q̇_2)^2
  + m_2 l_1 l_2 cos(q_2) q̇_1 (q̇_1 + q̇_2)
```

**위치에너지 V:**

```
V = m_1 g l_1 sin(q_1) + m_2 g [l_1 sin(q_1) + l_2 sin(q_1 + q_2)]
```

**M(q) 행렬:**

```
M(q) = [ (m_1+m_2)l_1^2 + m_2 l_2^2 + 2 m_2 l_1 l_2 cos(q_2)    m_2 l_2^2 + m_2 l_1 l_2 cos(q_2) ]
        [ m_2 l_2^2 + m_2 l_1 l_2 cos(q_2)                          m_2 l_2^2                          ]
```

M(q)는 q_2에 따라 달라진다. q_2 = 0일 때는 팔이 완전히 펴져 관성이 최대가 되고, q_2 = π일 때는 팔이 접혀 관성이 최소가 된다.

**C(q, q̇) 행렬:**

```
C(q, q̇) = [ -m_2 l_1 l_2 sin(q_2) q̇_2    -m_2 l_1 l_2 sin(q_2)(q̇_1 + q̇_2) ]
            [  m_2 l_1 l_2 sin(q_2) q̇_1     0                                     ]
```

C 행렬의 유도 방법은 여러 가지가 있다(Christoffel symbols 등). 가장 체계적인 방법은 Christoffel symbols를 쓰는 것이지만, 2-link의 경우 Euler-Lagrange 방정식에서 직접 항을 정리하는 편이 빠르다.

**g(q) 벡터:**

```
g(q) = [ (m_1 + m_2) g l_1 cos(q_1) + m_2 g l_2 cos(q_1 + q_2) ]
        [ m_2 g l_2 cos(q_1 + q_2)                                 ]
```

이것을 SymPy로 검증하는 코드:

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
# 여기서 d/dt는 q1, q2에 대한 시간 미분을 포함해야 하므로 치환이 필요하다.
# 간단하게 M, C, g를 추출하는 것은 교재를 참고하라.

print("T =", T)
print("V =", V)
```

이 코드를 직접 실행해보면 위에서 손으로 유도한 결과와 일치하는 것을 확인할 수 있다. SymPy가 trigsimp을 적용하면 깔끔한 형태가 나온다.

> **추천 자료**
> - Murray, Li, Sastry, *A Mathematical Introduction to Robotic Manipulation*, Ch. 4 (https://www.cds.caltech.edu/~murray/mlswiki/) — 라그랑주 역학을 로보틱스 맥락에서 가장 엄밀하게 다룬 교재. 무료 PDF 제공.
> - Spong, Hutchinson, Vidyasagar, *Robot Modeling and Control*, Ch. 6-7 — 학부 수준에서 가장 접근하기 쉬운 설명
> - Craig, *Introduction to Robotics*, Ch. 6 — 2-link arm 예제가 상세히 나와 있다

---

## 5.4 뉴턴-오일러 vs 라그랑주

이 둘은 같은 물리를 다른 관점에서 보는 것이다. 최종 결과(운동 방정식)는 동일하다. 차이는 유도 과정과 계산 효율에 있다.

| 항목 | 뉴턴-오일러 (RNEA) | 라그랑주 |
|------|-------------------|---------|
| 관점 | 힘/토크 (force-based) | 에너지 (energy-based) |
| 계산 복잡도 | O(n) | O(n^3) (M 행렬 직접 계산 시) |
| 유도 난이도 | 재귀적이라 n이 커져도 같은 패턴 | n이 커지면 편미분이 폭발 |
| 물리적 직관 | 각 링크의 힘/토크를 직접 볼 수 있음 | 에너지 보존/변환을 볼 수 있음 |
| 주 용도 | 실시간 제어, 시뮬레이션 | 모델 유도, 에너지 기반 분석, Lyapunov 안정성 |
| 구속력 | 명시적으로 계산 가능 | 일반화 좌표 사용 시 자동으로 소거 |

실무 워크플로우는 대체로 이렇다:

1. 라그랑주 역학으로 manipulator equation의 구조를 이해한다 (모델 유도).
2. RNEA(또는 ABA)로 실시간 계산한다 (수치 계산).
3. manipulator equation의 구조(M, C, g)를 이용한 computed torque control, passivity-based control 등을 설계한다 (제어기 설계).
4. Pinocchio나 Drake가 내부적으로 RNEA/ABA를 사용하므로, 라이브러리를 호출하면 된다 (코드 구현).

두 formulation은 서로 다른 역할을 맡는다. 라그랑주 방식은 제어 이론의 구조를 드러내고, 뉴턴-오일러 방식은 실시간 계산에 적합하다.

> **추천 자료**
> - Featherstone, *Rigid Body Dynamics Algorithms*, Ch. 3 — 두 formulation의 관계를 명확히 설명
> - Siciliano et al., *Robotics: Modelling, Planning and Control*, Ch. 7 — 두 방법으로 같은 로봇의 동역학을 유도하는 비교 예제

---

## 5.5 Forward Dynamics vs Inverse Dynamics

동역학에는 두 가지 "방향"이 있다:

**Inverse Dynamics (역동역학):**
```
주어진 것: q, q̇, q̈
구하는 것: τ
```
"이 궤적을 따라가려면 모터가 얼마의 토크를 내야 하는가?"에 답한다. 제어에서 주로 사용한다. Computed torque control의 핵심이다.

**Forward Dynamics (순동역학):**
```
주어진 것: q, q̇, τ
구하는 것: q̈
```
"이 토크를 가하면 로봇이 어떻게 가속하는가?"에 답하며, 시뮬레이터는 매 time step마다 이 계산을 반복한다.

수학적으로 forward dynamics는 manipulator equation에서 q̈을 풀어내는 것이다:

```
q̈ = M(q)^{-1} [τ - C(q, q̇)q̇ - g(q)]
```

단순히 M(q)의 역행렬을 구하면 O(n^3)이다. 이건 관절 수가 많으면 느리다.

### Articulated Body Algorithm (ABA)

Featherstone이 제안한 ABA는 forward dynamics를 O(n)에 계산한다. RNEA가 inverse dynamics의 O(n) 알고리즘이듯, ABA는 forward dynamics의 O(n) 알고리즘이다.

ABA는 각 링크를 "articulated body"로 보고, 해당 서브트리의 관성을 재귀적으로 합산한다. M 행렬을 명시적으로 구성하지 않고도 q̈을 직접 계산할 수 있다.

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

실제 구현은 상당히 복잡하다. Pinocchio나 Drake 같은 검증된 라이브러리를 쓰는 편이 현명하다.

### 시뮬레이터에서의 역할

시뮬레이터마다 사용하는 알고리즘이 다르다. **MuJoCo**는 forward dynamics에 자체 알고리즘을 쓴다. 접촉까지 포함한 통합 solver가 특징이고, 내부적으로 sparse factorization을 활용하며 분기형(branching) 구조에 특화되어 있다. **Drake**는 MultibodyPlant에서 ABA를 쓰고, 접촉은 별도의 solver(time-stepping, hydroelastic 등)로 처리한다. **Bullet(PyBullet)**은 Featherstone ABA를 기반으로 하되, 접촉은 sequential impulse solver를 사용한다.

코드로 보면:

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

RNEA와 ABA는 서로 역연산 관계이다. RNEA(q, v, ABA(q, v, τ)) ≈ τ 가 성립한다 (부동소수점 오차 이내).

> **추천 자료**
> - Featherstone, *Rigid Body Dynamics Algorithms*, Ch. 7 — ABA의 원본 설명
> - MuJoCo documentation: Computation (https://mujoco.readthedocs.io/en/latest/computation/) — MuJoCo의 동역학 파이프라인 설명
> - Drake MultibodyPlant tutorial (https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_multibody_plant.html) — Drake에서의 동역학 계산 API

---

## 5.6 접촉 역학 (Contact Dynamics)

로봇이 환경과 접촉하는 순간, 동역학은 한 단계 더 복잡해진다. 자유 공간에서의 동역학은 ODE(ordinary differential equation)로 깔끔하게 표현되지만, 접촉이 들어가면 부등식 구속 조건과 불연속성이 생긴다.

### Rigid Contact vs Compliant Contact

접촉을 모델링하는 두 가지 큰 틀이 있다:

**Rigid contact (경성 접촉):**
- 물체가 서로 관통하지 않는다는 구속 조건을 직접 부과한다.
- 접촉력은 구속 조건의 Lagrange multiplier로 나온다.
- 수학적으로 깔끔하지만, 수치적으로 어렵다 — 접촉/비접촉 전환 시 불연속성이 생기고, 이를 처리하기 위해 LCP(Linear Complementarity Problem)나 NCP(Nonlinear Complementarity Problem)를 풀어야 한다.
- Drake의 time-stepping 방식이 이 계열이다.

**Compliant contact (연성 접촉):**
- 접촉면에 가상의 스프링-댐퍼를 놓는다. 관통 깊이에 비례하는 반발력을 생성한다.
- 수치적으로 안정적이고 구현이 쉽다.
- 스프링 강성(stiffness)을 높이면 rigid contact에 가까워지지만, 수치 적분의 time step을 줄여야 한다 (stiff ODE).
- MuJoCo의 기본 접촉 모델이 이 계열이다.

### Coulomb Friction Model

접촉이 있으면 마찰(friction)이 따라온다. 가장 기본적인 마찰 모델은 Coulomb friction이다:

```
|f_t| ≤ μ f_n          (static friction: 정지 마찰)
|f_t| = μ f_n, f_t ∥ -v_t  (sliding friction: 운동 마찰)
```

여기서 f_t는 접선 방향 마찰력, f_n은 법선 방향 수직항력, μ는 마찰 계수, v_t는 접선 방향 상대 속도이다.

다만 한계가 있다. 정지 마찰에서 운동 마찰로의 전환이 불연속이고, 3D 마찰 원뿔(friction cone)은 비선형이라 선형화하면(friction pyramid) 정확도가 떨어진다. 더 심각하게는, 특정 조건에서 rigid contact + Coulomb friction 조합의 해가 존재하지 않거나 유일하지 않다 — Painleve's paradox다.

### Contact-Rich Manipulation이 어려운 이유

물체를 잡고, 돌리고, 끼우는 작업(peg-in-hole, in-hand manipulation 등)은 왜 그렇게 어려운가?

첫째, 접촉 모드가 수시로 바뀐다(contact/no-contact, stick/slip). 각 모드마다 동역학이 다르고, 모드 전환 시점을 예측하기 어렵다. 둘째, 전환 순간에 상태가 불연속적으로 변할 수 있다(충격, impact). 셋째, 마찰 계수나 접촉 강성의 정확한 값을 모르면 sim-to-real gap이 커진다. 접촉점 수가 늘면 contact/separation과 stick/slip 조합이 지수적으로 늘어나는 조합론적 복잡성도 겹친다. 정확한 모드 수는 마찰 모델과 접선 방향의 이산화 방식에 따라 달라진다.

### 시뮬레이터마다 접촉 처리가 다른 이유

접촉을 수치적으로 근사하는 방식이 여러 가지이기 때문이다. 각 시뮬레이터는 정확도, 속도, 안정성 사이의 트레이드오프를 다르게 선택한다.

**MuJoCo**는 compliant contact + convex optimization 방식을 쓴다. 빠르고 안정적이지만 물리적으로 완벽하지는 않다. 특히 관통이 허용되는데, 이를 "soft contact"의 일부로 받아들인다. RL 환경으로 인기 많은 이유 중 하나가 이 안정성이다. **Drake**는 rigid contact + time-stepping(Stewart-Trinkle) 또는 hydroelastic contact을 지원한다. 물리적으로 더 엄밀하지만 계산 비용이 높을 수 있다. Hydroelastic contact은 접촉면의 압력 분포까지 계산한다.

**Bullet**은 velocity-level LCP + sequential impulse 방식으로, 게임/VR에서 출발한 엔진이라 속도에 최적화되어 있으나 정밀한 접촉이 필요한 로보틱스 작업에서는 한계가 있다. **DART**는 LCP 기반 rigid contact으로 학술적으로 엄밀한 구현이지만, MuJoCo나 Drake에 비해 사용자 기반이 작다.

어떤 시뮬레이터를 쓸지는 연구 목적에 따라 다르다. MuJoCo는 locomotion RL에서 널리 쓰이고, Drake와 MuJoCo는 모두 contact-rich manipulation을 다룰 수 있다. 최종 선택은 필요한 접촉 모델, gradient, 처리 속도, 재현할 hardware와 검증 사례를 기준으로 한다.

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
    print(f"  Penetration depth: {contact.dist}")
    print(f"  Geom pair: ({contact.geom1}, {contact.geom2})")
```

> **추천 자료**
> - Stewart, "Rigid-Body Dynamics with Friction and Impact", SIAM Review 2000 — 접촉 역학의 수학적 기초
> - Todorov, "Convex and analytically-invertible dynamics with contacts and constraints", ICRA 2014 — MuJoCo의 접촉 모델 논문
> - [Todorov et al., "MuJoCo: A Physics Engine for Model-Based Control" (IROS 2012)](https://ieeexplore.ieee.org/document/6386109) — MuJoCo의 convex contact formulation과 velocity stepping을 설명한 원 논문
> - Drake의 접촉 모델 documentation (https://drake.mit.edu/doxygen_cxx/group__hydroelastic__user__guide.html) — Hydroelastic contact 설명
> - Russ Tedrake, *Underactuated Robotics*, Ch. "Contact" (https://underactuated.csail.mit.edu/) — 접촉 역학 개론

---

## 5.7 심화: Featherstone 알고리즘과 Spatial Algebra

여기서부터는 대학원 수준이다. Featherstone의 spatial vector algebra는 동역학 알고리즘을 간결하고 효율적으로 표현하기 위한 수학적 틀이다.

### Spatial Vectors (6D Vectors)

3D 공간에서 강체의 운동은 병진(3 DOF) + 회전(3 DOF) = 6 DOF이다. 이를 하나의 6D 벡터로 통합한 것이 spatial vector이다.

**Motion vector (spatial velocity, twist):**
```
v = [ω; v_O]
```
위 3개는 각속도(ω), 아래 3개는 기준점 O에서의 선속도(v_O)이다.

**Force vector (spatial force, wrench):**
```
f = [n_O; f]
```
위 3개는 기준점 O 주위의 모멘트(n_O), 아래 3개는 힘(f)이다.

spatial velocity와 spatial force의 내적이 곧 power(일률)이다.
```
P = f^T v = n_O · ω + f · v_O
```

Featherstone은 이 성질이 성립하도록 spatial vector를 정의했다.

### Spatial Inertia

6×6 spatial inertia matrix는 질량, 질량중심 위치, 회전 관성을 하나의 행렬에 통합한다:

```
I_sp = [ I_cm + m·[c]×[c]×^T    m·[c]× ]
       [ m·[c]×^T                 m·1    ]
```

여기서 m은 질량, c는 질량중심까지의 벡터, I_cm은 질량중심 주위의 회전 관성, [c]×는 c의 skew-symmetric matrix이다.

여러 강체의 관성을 합칠 때 그냥 더하면 된다(I_composite = I_1 + I_2 + ...). 좌표 변환은 congruence transform 하나로 끝난다(I_B = X^T I_A X).

### RNEA와 ABA의 Spatial Vector 표현

5.2절과 5.5절에서 보인 의사 코드가 사실 spatial vector 표기였다. S[i]는 관절 i의 motion subspace(revolute 관절이면 [e_z; 0], prismatic이면 [0; e_z]), v[i]는 spatial velocity, f[i]는 spatial force이다.

Spatial vector를 쓰면 회전 관절이든 직동 관절이든 같은 코드로 처리할 수 있다. Pinocchio와 Drake가 내부적으로 spatial algebra를 사용하는 이유다.

### Pinocchio에서 Spatial Quantities 접근

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

C++에서 Pinocchio를 사용할 때:

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

Pinocchio의 C++ API는 Eigen 기반이며, Python API와 거의 동일한 인터페이스를 제공한다. kHz급 실시간 제어에서는 Python 오버헤드를 피하려고 C++ API를 사용한다.

> **추천 자료**
> - Featherstone, *Rigid Body Dynamics Algorithms*, Ch. 2 — Spatial vector algebra의 원본 설명
> - Featherstone, "A Beginner's Guide to 6-D Vectors" (IEEE Robotics & Automation Magazine, 2010) — 교과서보다 접근하기 쉬운 소개 논문
> - Pinocchio GitHub (https://github.com/stack-of-tasks/pinocchio) — 소스 코드 자체가 spatial algebra의 좋은 구현 예시이다

---

## 5.8 심화: 부유 베이스 시스템 (Floating Base)

산업용 매니퓰레이터는 base가 바닥에 볼트로 고정되어 있다. 반면 보행 로봇, 드론, 수중 로봇은 base 자체가 움직인다. 이 경우 base의 위치와 자세가 자유도에 추가되면서 동역학의 구조도 달라진다.

### 부유 베이스의 Configuration

고정 base 로봇의 configuration은 q ∈ R^n이다. 부유 base 로봇의 configuration은:

```
q = [q_base; q_joints]
```

q_base는 SE(3)의 원소이다 — 위치(3) + 자세(3, 또는 quaternion으로 4). Pinocchio에서는 q의 차원(nq)과 v의 차원(nv)이 다를 수 있다(quaternion을 쓰면 nq = nv + 1). q와 v가 같은 벡터 공간에 살지 않으므로, 적분이나 차분을 할 때 단순히 `q += v*dt`를 하면 안 된다. `pin.integrate(model, q, v*dt)`를 써야 한다.

### Underactuated Systems

부유 base 시스템은 **underactuated**다. Base에는 직접 구동기(actuator)가 없어서, 보행 로봇은 발이 지면을 밀어야 base가 움직이고, 드론은 프로펠러의 추력으로 base를 움직인다.

Manipulator equation을 base와 joints로 나누면:

```
[ M_bb  M_bj ] [ a_base  ]   [ C_b ]   [ g_b ]   [  0  ]   [ J_c^T ]
[ M_jb  M_jj ] [ q̈_joints] + [ C_j ] + [ g_j ] = [ τ_j ] + [ J_c^T ] λ
```

왼쪽 위의 `0`은 base에 관절 토크가 없음을 나타낸다. λ는 접촉력이고, J_c는 접촉 Jacobian이다. Base는 접촉력과 중력을 통해 가속한다.

이 구속 조건이 locomotion 제어를 어렵게 만든다. 고정 base 매니퓰레이터는 원하는 관절 토크를 그냥 모터에 명령하면 되지만, 보행 로봇은 적절한 접촉력을 만들어내야 base를 원하는 대로 움직일 수 있다.

### Centroidal Dynamics

전체 시스템의 운동량(momentum)을 질량중심(center of mass, CoM)에서 표현한 것이 centroidal dynamics이다:

**선운동량 (linear momentum):**
```
p = m v_CoM = Σ m_i v_i
```

**각운동량 (angular momentum about CoM):**
```
L = Σ (r_i - r_CoM) × (m_i v_i) + I_i ω_i
```

**Centroidal momentum의 시간 미분:**
```
ṗ = m g + Σ f_contact
L̇ = Σ (r_contact - r_CoM) × f_contact
```

CoM 역학이 balance를 결정한다. 로봇이 넘어지지 않으려면 CoM의 궤적이 지지 영역(support polygon) 위에 있어야 한다(ZMP 조건). 더 정확히는 centroidal momentum가 적절히 조절되어야 한다.

차원 축소의 효과도 있다. n-DOF 보행 로봇의 전체 동역학은 n차원이지만, centroidal dynamics는 6차원(선운동량 3 + 각운동량 3)이다. 이 6차원 공간에서 원하는 운동량 궤적을 먼저 계획하고, 그다음 전체 관절 수준으로 분해하는 것이 일반적인 접근이다.

Centroidal momentum의 변화율은 외력(접촉력 + 중력)으로 결정된다. 따라서 locomotion에서는 원하는 운동량 궤적을 만드는 접촉력 패턴을 계획해야 한다.

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
h = data.hg  # 6D centroidal momentum [angular; linear]
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

현대적인 보행 로봇 제어 파이프라인의 전형적인 구조는 다음과 같다:

```
[Contact Schedule] → [Centroidal Trajectory Optimization] → [Whole-Body Control] → [Joint Torques]

1단계: 언제 어떤 발이 땅에 닿는지 결정 (gait pattern)
2단계: Centroidal dynamics를 만족하는 CoM 궤적 + 접촉력 계획
3단계: Centroidal 목표를 달성하면서 관절 수준의 여러 제약을 만족하는 토크 계산
4단계: 모터에 토크 명령
```

부유 base를 가진 시스템에는 이 구조를 적용할 수 있다. 드론의 trajectory optimization과 수중 로봇 제어에도 유사한 구조를 사용한다.

> **추천 자료**
> - Orin et al., "Centroidal Dynamics of a Humanoid Robot", Autonomous Robots 2013 — Humanoid의 centroidal dynamics를 분석한 논문
> - Wensing et al., "Optimization-Based Control for Dynamic Legged Locomotion", 2023 — Locomotion 제어 survey
> - Russ Tedrake, *Underactuated Robotics*, Ch. "Walking" (https://underactuated.csail.mit.edu/) — Underactuated 시스템과 보행의 관계
> - Carpentier, Mansard, "Pinocchio: fast forward and inverse dynamics for poly-articulated systems" (https://github.com/stack-of-tasks/pinocchio) — Pinocchio의 centroidal dynamics 구현

---

## 5.9 추천 자료

추천 순서는 배경지식에 따라 다르다.

**학부 3-4학년, 동역학 입문:**

> - Spong, Hutchinson, Vidyasagar, *Robot Modeling and Control* — 학부 수준의 입문 교재. Manipulator equation 유도가 상세하다.
> - Craig, *Introduction to Robotics: Mechanics and Control* — 산업 현장 관점. 실용적이지만 수학적 깊이는 좀 얕다.

**대학원, 수학적으로 엄밀한 이해가 필요할 때:**

> - Murray, Li, Sastry, *A Mathematical Introduction to Robotic Manipulation* (https://www.cds.caltech.edu/~murray/mlswiki/) — Lie group/algebra 관점에서 동역학을 엄밀하게 다룬다. 무료 PDF를 제공하며, 입문자가 바로 읽기에는 어렵다.
> - Featherstone, *Rigid Body Dynamics Algorithms* — Spatial vector algebra, RNEA, ABA, composite rigid body algorithm을 체계적으로 다루는 대학원 수준의 참고서.

**동역학 + 제어 통합:**

> - Russ Tedrake, *Underactuated Robotics* (https://underactuated.csail.mit.edu/) — 동역학 모델을 제어와 최적화에 어떻게 활용하는지를 다룬다. MIT OCW에 강의 영상도 있다. 무료.

**라이브러리 & 도구:**

> - Pinocchio (https://github.com/stack-of-tasks/pinocchio) — 순수 동역학 계산용 C++/Python 라이브러리. RNEA, ABA, CRBA, centroidal dynamics, analytical derivatives 등을 지원한다. CasADi/JAX를 통한 autodiff도 가능하다 (Pinocchio 3.x).
> - Drake (https://drake.mit.edu/) — 시뮬레이션 + 최적화 + 제어를 통합한 프레임워크. MultibodyPlant가 동역학 엔진이다. Mathematical programming 인터페이스가 강력하여 trajectory optimization에 특히 유용하다.
> - MuJoCo (https://mujoco.org/) — DeepMind가 관리하는 물리 시뮬레이터. 접촉을 포함한 로봇 학습 연구에 널리 쓰인다.
> - PyBullet (https://pybullet.org/) — Bullet Physics의 Python 인터페이스. 진입 장벽이 낮아 교육용으로 적합하지만, 접촉 물리의 정밀도는 MuJoCo나 Drake에 미치지 못한다.

---

## 기술 흐름

```
1687 ── Newton의 운동 법칙 (Principia Mathematica)
1788 ── Lagrange의 해석역학 (Mécanique Analytique)
1965 ── Uicker의 동역학 방정식 (symbolic, 비효율적)
1980 ── Luh, Walker, Paul의 Newton-Euler 재귀 알고리즘 (RNEA, O(n))
1983 ── Featherstone의 Articulated Body Algorithm (ABA, O(n) forward dynamics)
1987 ── Featherstone의 Spatial Vector Algebra 체계 정립
2000 ── Stewart의 rigid contact dynamics 수학적 정리 (SIAM Review)
2004 ── ODE (Open Dynamics Engine) — 초기 오픈소스 물리 엔진
2012 ── MuJoCo 공개 (Todorov, Erez, Tassa)
2015 ── Bullet Physics 2.x → PyBullet 인터페이스
2016 ── Pinocchio 1.0 공개 (LAAS-CNRS)
2022 ── Drake 1.0 (MIT → Toyota Research Institute)
2021 ── MuJoCo 오픈소스화 (DeepMind 인수 후)
2022 ── MuJoCo 2.3: implicit integration, elliptic friction cone
2023 ── Pinocchio 3.0: CasADi/JAX autodiff 지원
2023 ── MuJoCo 3.0: MJX (JAX backend for GPU parallelism)
```

---

## 정리

실무 요점:

1. Manipulator equation `M(q)q̈ + C(q,q̇)q̇ + g(q) = τ`가 동역학 계산의 표준 형태다.
2. 역동역학(τ 계산)에는 RNEA, 순동역학(q̈ 계산)에는 ABA를 쓴다. 둘 다 O(n)이다.
3. 접촉은 제약과 불연속성을 추가하므로, 접촉 모델과 시뮬레이터의 선택을 함께 검토한다.
4. 부유 base 시스템에서는 centroidal dynamics가 핵심 도구다.
5. 실무에서는 Pinocchio나 Drake를 활용하되, 라이브러리가 내부에서 계산하는 양과 가정을 확인한다.

이 동역학 모델 위에서 computed torque control, operational space control, whole-body control 같은 제어 기법이 만들어진다.
