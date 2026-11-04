# Ch.6 — 제어 이론 (Control Theory)


---

## 6.1 왜 제어를 배우는가

인식(Perception)은 환경과 로봇의 상태를 추정한다. 제어(Control)는 그 추정을 바탕으로 목표 상태에 도달하도록 actuator 입력을 계산한다. 제어 입력을 잘못 계산하면 로봇이 불안정해지거나 충돌할 수 있다.

- **정확한 위치 추종**: feedback 없이 큰 전류를 흘리면 관절이 목표 각도를 지나쳐 진동할 수 있다. 제어기는 오차를 줄이면서 목표 위치에 도달하도록 입력을 조절한다.
- **외란 대응**: 미끄러운 바닥, 바람, 예상과 다른 payload처럼 모델에 없는 변화가 생긴다. 센서와 actuator 사이의 loop를 닫아 이런 변화에 대응한다.
- **안전한 접촉**: 사람과 함께 일하는 산업용 로봇은 위치와 함께 접촉력도 제한해야 한다.

이 장은 행렬 연산, 고유값(eigenvalue), 미분방정식을 쓴다.

---

## 6.2 PID 제어

PID(Proportional-Integral-Derivative) 제어는 1922년 Minorsky가 선박 자동 조타를 분석하며 정식화했고, 이후 산업 현장에 널리 쓰였다. 구조가 단순하고 각 항의 역할이 분명해 제어 입문에서 먼저 다룬다.

### 기본 구조

오차는 e(t) = r(t) - y(t)다. r(t)는 목표값(reference), y(t)는 현재 출력이다.

```
u(t) = Kp * e(t) + Ki * integral(e(τ)dτ, 0, t) + Kd * de(t)/dt
```

각 항의 역할:

- P (Proportional): 현재 오차에 비례하는 제어 입력을 낸다. Kp를 키우면 반응이 빨라지고, 오버슈트와 진동도 커진다. 적분기가 없는 플랜트에 계단 입력을 주면 P 항만으로는 정상상태 오차(steady-state error)가 남는다. 목표값 근처에서 오차가 작아지면 제어 입력도 작아지기 때문이다. 플랜트에 자유 적분기가 있으면 계단 입력의 정상 오차는 0이 될 수 있다. 다만 중력 같은 정상 외란이 만드는 오차는 따로 남는다.

- I (Integral): 오차의 누적값에 비례한다. 정상상태 오차를 없애며, 중력이나 마찰 같은 상수 외란이 있을 때 필요하다. 제어 입력이 포화(saturation)된 동안 오차가 계속 쌓이면, 목표에 도달한 뒤에도 쌓인 적분값 때문에 큰 오버슈트가 생긴다. 이를 wind-up이라 하며, 실무에서는 anti-windup 로직을 넣는다.

- D (Derivative): 오차의 변화율에 비례한다. 오차가 빠르게 줄고 있으면 제어 입력을 줄여 오버슈트를 억제한다. 미분 연산은 고주파 노이즈를 키운다. 센서 노이즈가 있는 실제 시스템에서는 D 항에 저역통과 필터(low-pass filter)를 건다. D 항을 빼고 PI 제어만 쓰기도 한다.

### Python 구현

```python
class PIDController:
    """이산시간 PID 제어기. Anti-windup 포함."""

    def __init__(self, kp: float, ki: float, kd: float, dt: float,
                 output_limit: tuple[float, float] = (-float('inf'), float('inf')),
                 d_filter_coeff: float = 0.1):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.output_limit = output_limit
        self.d_filter_coeff = d_filter_coeff  # D항 저역통과 필터 계수

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_d_filtered = 0.0

    def compute(self, error: float) -> float:
        # Proportional
        p_term = self.kp * error

        # Integral (trapezoidal integration)
        self.integral += 0.5 * (error + self.prev_error) * self.dt
        i_term = self.ki * self.integral

        # Derivative (with low-pass filter)
        d_raw = (error - self.prev_error) / self.dt
        d_filtered = (self.d_filter_coeff * d_raw
                      + (1.0 - self.d_filter_coeff) * self.prev_d_filtered)
        d_term = self.kd * d_filtered

        # 제어 출력
        output = p_term + i_term + d_term

        # Output saturation + anti-windup (clamping)
        lo, hi = self.output_limit
        if output > hi:
            output = hi
            # Anti-windup: 포화되면 이번 스텝에 더한 적분을 되돌린다
            self.integral -= 0.5 * (error + self.prev_error) * self.dt
        elif output < lo:
            output = lo
            self.integral -= 0.5 * (error + self.prev_error) * self.dt

        self.prev_error = error
        self.prev_d_filtered = d_filtered
        return output


# 사용 예시: 1-DOF 위치 제어
import numpy as np

dt = 0.001  # 1kHz 제어 주기
pid = PIDController(kp=100.0, ki=10.0, kd=5.0, dt=dt,
                    output_limit=(-50.0, 50.0))

position = 0.0
velocity = 0.0
mass = 1.0
target = 1.0

positions = []
for step in range(5000):
    error = target - position
    force = pid.compute(error)

    # 간단한 2차 위치 동역학: F = ma, 감쇠 포함
    acceleration = (force - 0.5 * velocity) / mass
    velocity += acceleration * dt
    position += velocity * dt
    positions.append(position)
```

### 튜닝 방법

**Ziegler-Nichols 방법**: Ki = 0, Kd = 0으로 놓고 Kp를 올려가면서 시스템이 지속 진동(sustained oscillation)하는 임계 이득 Ku와 진동 주기 Tu를 구한다. 이 두 값으로 게인을 정한다.

```
PID:  Kp = 0.6 * Ku,  Ki = 2 * Kp / Tu,  Kd = Kp * Tu / 8
PI:   Kp = 0.45 * Ku,  Ki = 1.2 * Kp / Tu
P:    Kp = 0.5 * Ku
```

Ziegler-Nichols 게인은 오버슈트가 크게 나올 수 있어, 초기값으로 쓰고 응답을 보며 다시 조정한다.

**경험적 튜닝**: 흔히 쓰는 순서는 이렇다.

1. D, I를 0으로 설정한다.
2. P를 높여 시스템이 빠르게 반응하되 진동하지 않는 지점을 찾는다.
3. 정상상태 오차가 남으면 I를 조금씩 올리고, anti-windup을 함께 넣는다.
4. 오버슈트가 크면 D를 소량 추가하고 노이즈 필터를 함께 확인한다.

시스템 모델을 쓸 수 있다면 시뮬레이션에서 먼저 게인을 조정한 뒤 실기에 적용한다.

### PID의 한계


- SISO(Single-Input Single-Output) 전용이다. 6축 로봇 팔처럼 관절 간 커플링이 있는 시스템에서는 각 관절에 독립적으로 PID를 걸면 성능이 떨어진다. 한 관절의 움직임이 다른 관절에 외란으로 작용하기 때문이다.
- PID는 선형 제어기이고 로봇 동역학은 비선형이라, 작동점(operating point) 근처에서만 잘 동작한다.
- 토크 제한, 관절 각도 제한, 속도 제한 같은 물리적 제약은 PID 바깥에서 saturation이나 anti-windup으로 따로 처리한다.
- 현재 오차, 누적 오차, 오차의 변화율에만 반응한다. 빠른 궤적을 추종하려면 feedforward를 더한다.

PID는 구현과 해석이 단순하다. 관절 사이 결합이 약하고 성능 요구를 맞출 수 있으면 PID로 충분하다. 산업용 로봇의 관절 서보에도 PID 계열 제어기가 쓰인다.

---

## 6.3 상태공간 표현 (State-Space Representation)

PID는 입력과 출력의 관계만 보고 시스템 내부 상태는 쓰지 않는다. 상태공간 표현은 내부 상태를 변수로 직접 적는다.

### 기본 형태

연속시간 선형 시스템:

```
x_dot(t) = A * x(t) + B * u(t)    (상태 방정식)
y(t)     = C * x(t) + D * u(t)    (출력 방정식)
```

- x(t): 상태 벡터 (n x 1). 시스템을 완전히 기술하는 데 필요한 최소 변수 집합.
- u(t): 입력 벡터 (m x 1). 제어 입력.
- y(t): 출력 벡터 (p x 1). 측정 가능한 출력.
- A: 시스템 행렬 (n x n). 입력이 없을 때의 동특성을 정한다.
- B: 입력 행렬 (n x m). 입력이 상태에 미치는 영향.
- C: 출력 행렬 (p x n). 상태를 출력으로 옮기는 매핑.
- D: 직접 전달 행렬 (p x m). 대부분의 물리 시스템에서 0이다.

질량-스프링-댐퍼 시스템 (m * x_ddot + c * x_dot + k * x = F)에서 상태를 x1 = 위치, x2 = 속도로 잡으면 다음과 같다. 이 식의 m은 질량(위 목록의 입력 차원 m과 다르다), c는 감쇠 계수, k는 스프링 상수, F는 입력 힘이다.

```
A = [[0, 1], [-k/m, -c/m]]
B = [[0], [1/m]]
C = [[1, 0]]   (위치만 측정)
D = [[0]]
```

### 전달함수와의 관계

전달함수는 G(s) = C * (sI - A)^(-1) * B + D 이고, SISO 시스템 분석에 편하다. 다입출력 로봇 제어에서는 변수 사이의 결합이 그대로 보이는 상태공간 표현을 많이 쓴다.

### 가제어성 (Controllability)

가제어(controllable) 시스템은 어떤 초기 상태에서든 원하는 최종 상태로 유한 시간 안에 옮길 수 있다. 가제어성 행렬:

```
C_ctrl = [B, A*B, A^2*B, ..., A^(n-1)*B]
```

이 행렬의 rank가 n이면 가제어이다. rank가 n보다 작으면 제어 입력으로 도달할 수 없는 상태가 있다. 무한시간 LQR의 안정화 해에는 완전한 가제어성 대신 안정화 가능성(stabilizability)이 필요하다.

### 가관측성 (Observability)

가관측(observable) 시스템은 출력 y(t)로 초기 상태 x(0)를 유일하게 정할 수 있다. 가관측성 행렬:

```
O = [C; C*A; C*A^2; ...; C*A^(n-1)]
```

rank가 n이면 가관측이다. 가관측 시스템이면 상태 추정기(observer, Kalman filter)의 추정값이 실제 상태로 수렴하도록 설계할 수 있다.

### 왜 PID에서 상태공간으로 넘어가야 하는가

PID로 각 관절을 따로 제어하면 관절 사이의 동적 커플링을 무시한다. 2-DOF 로봇 팔에서도 한 관절이 빠르게 움직이면 다른 관절에 원심력과 코리올리 힘이 걸린다. 이 힘을 외란으로 두면 I 항이 보상하지만 응답이 느려진다.

상태공간에서는 시스템 전체를 한 모델로 적고, 모든 상태 변수를 함께 써서 제어 입력을 계산한다. 다음 절의 LQR과 MPC가 이 방식을 쓴다.

```python
import numpy as np
import control  # pip install control

# 도립진자(inverted pendulum) 상태공간 모델
# 상태: [x, x_dot, theta, theta_dot]
# x: 카트 위치, theta: 진자 각도 (수직에서)
M = 1.0    # 카트 질량 (kg)
m = 0.1    # 진자 질량 (kg)
l = 0.5    # 진자 길이 (m)
g = 9.81   # 중력 (m/s^2)

# 선형화된 상태공간 행렬 (theta ≈ 0 근처)
A = np.array([
    [0, 1, 0, 0],
    [0, 0, -m * g / M, 0],
    [0, 0, 0, 1],
    [0, 0, (M + m) * g / (M * l), 0]
])
B = np.array([[0], [1 / M], [0], [-1 / (M * l)]])
C = np.array([[1, 0, 0, 0],
              [0, 0, 1, 0]])  # 카트 위치와 진자 각도 측정
D = np.zeros((2, 1))

# 가제어성 확인
ctrb_matrix = control.ctrb(A, B)
print(f"가제어성 행렬 rank: {np.linalg.matrix_rank(ctrb_matrix)}")  # 4 = 가제어

# 가관측성 확인
obsv_matrix = control.obsv(A, C)
print(f"가관측성 행렬 rank: {np.linalg.matrix_rank(obsv_matrix)}")  # 4 = 가관측

# 시스템 극점 (eigenvalues of A)
eigenvalues = np.linalg.eigvals(A)
print(f"시스템 극점: {eigenvalues}")
# 실수부가 양수인 극점이 하나라도 있으면 불안정하다 (도립진자가 그렇다)
```

---

## 6.4 LQR (Linear Quadratic Regulator)

LQR은 이차 비용 함수를 최소화하는 상태 피드백 게인을 구하는 제어기다. 게인은 Riccati 방정식을 풀어 얻는다.

### 비용 함수

```
J = integral_0^inf (x(t)^T * Q * x(t) + u(t)^T * R * u(t)) dt
```

- Q (n×n, 양의 반정치): 상태가 0에서 벗어날 때의 페널티 가중치.
- R (m×m, 양정치): 제어 입력 크기의 페널티 가중치.

Q를 크게 하면 상태가 빠르게 0으로 수렴하지만 제어 입력은 커진다. R을 크게 하면 제어 입력이 작아지는 대신 상태 수렴이 느려진다. Q와 R의 비율로 이 둘 사이를 조절한다.

### Q, R 행렬 튜닝

흔히 쓰는 출발점은 Q와 R을 대각 행렬로 놓고 각 대각 원소를 해당 상태나 입력의 허용 최댓값의 역수 제곱으로 두는 것이다.

```
Q_ii = 1 / (허용 가능한 x_i의 최대값)^2
R_jj = 1 / (허용 가능한 u_j의 최대값)^2
```

예: 카트 위치가 0.5m 이내, 진자 각도가 0.1rad 이내, 힘이 20N 이내를 원한다면:

```
Q = diag(1/0.5^2, 0, 1/0.1^2, 0) = diag(4, 0, 100, 0)
R = [1/20^2] = [0.0025]
```

이 값에서 시작해 시뮬레이션을 돌리며 조정한다.

### Algebraic Riccati Equation (ARE)

LQR의 최적 게인 K는 Algebraic Riccati Equation의 해 P로 구한다.

```
A^T * P + P * A - P * B * R^(-1) * B^T * P + Q = 0
```

최적 상태 피드백 게인: K = R^(-1) * B^T * P

제어 법칙: u(t) = -K * x(t)

(A, B)가 안정화 가능하고 (Q^{1/2}, A)가 검출 가능(detectable)하며 R이 양정치(positive definite)이면 폐루프 행렬 (A - BK)의 모든 고유값은 실수부가 음수이고, 폐루프는 점근적으로 안정하다.

### Python 구현

```python
import numpy as np
from scipy.linalg import solve_continuous_are

# 앞 절의 도립진자 모델 사용
M, m, l, g = 1.0, 0.1, 0.5, 9.81

A = np.array([
    [0, 1, 0, 0],
    [0, 0, -m * g / M, 0],
    [0, 0, 0, 1],
    [0, 0, (M + m) * g / (M * l), 0]
])
B = np.array([[0], [1 / M], [0], [-1 / (M * l)]])

# 비용 함수 가중치
Q = np.diag([4.0, 0.0, 100.0, 0.0])  # 위치, 속도, 각도, 각속도
R = np.array([[0.0025]])

# ARE 풀기
P = solve_continuous_are(A, B, Q, R)

# 최적 게인 계산
K = np.linalg.inv(R) @ B.T @ P
print(f"LQR 게인 K: {K}")

# 폐루프 극점 확인
A_cl = A - B @ K
eigenvalues_cl = np.linalg.eigvals(A_cl)
print(f"폐루프 극점: {eigenvalues_cl}")
# 모든 실수부가 음수 → 안정


def simulate_lqr(A, B, K, x0, dt=0.001, t_final=5.0):
    """LQR 폐루프 시뮬레이션 (Euler 적분)."""
    n_steps = int(t_final / dt)
    n = A.shape[0]
    x_history = np.zeros((n_steps, n))
    u_history = np.zeros((n_steps, 1))

    x = x0.copy()
    for i in range(n_steps):
        u = -K @ x
        x_history[i] = x.flatten()
        u_history[i] = u.flatten()
        x_dot = A @ x + B @ u
        x = x + x_dot * dt

    return x_history, u_history


# 초기 조건: 진자가 10도 기울어진 상태
x0 = np.array([[0.0], [0.0], [np.radians(10)], [0.0]])
x_hist, u_hist = simulate_lqr(A, B, K, x0)

# x_hist[:, 2]가 0으로 수렴하면 성공
print(f"최종 진자 각도: {np.degrees(x_hist[-1, 2]):.4f} deg")
```

### LQR의 한계

- 선형 모델이 필요하다. 비선형 시스템은 작동점 근처에서 선형화하므로, 작동점에서 멀어질수록 성능이 떨어진다.
- 토크 제한, 속도 제한 같은 부등식 제약은 표준 LQR 문제의 바깥에 있다. 제어 입력이 포화되면 그 순간부터 최적성이 깨진다.
- 전체 상태를 알아야 한다. u = -Kx이므로 모든 상태 변수를 측정하거나 추정(observer)해야 한다.
- 기본 LQR은 regulator, 즉 상태를 0으로 보내는 문제를 푼다. 시변 목표를 추종하려면 오차 좌표로 바꾸거나 tracking LQR로 확장한다.

---

## 6.5 MPC (Model Predictive Control)

MPC(Model Predictive Control)는 매 제어 주기마다 유한 구간(finite horizon) 최적화 문제를 풀어 제어 입력을 계산하는 방법이다. 제약을 최적화 문제에 직접 넣을 수 있어 로봇 제어에 많이 쓴다.

### 기본 개념

매 time step k에서 하는 일:

1. 현재 상태 x(k)를 센서로 측정하거나 필터로 추정한다.
2. 시스템 모델을 바탕으로 N step 앞까지 미래 거동을 예측한다.
3. 제약 조건을 지키면서 비용 함수를 최소화하는 입력 시퀀스 {u(k), u(k+1), ..., u(k+N-1)}을 구한다.
4. 구한 시퀀스에서 첫 입력 u(k)만 실제로 넣고 나머지는 버린다.
5. 다음 time step에서 1로 돌아간다.

이를 receding horizon 전략이라 한다. 매번 측정한 현재 상태에서 최적화를 다시 풀므로, 모델 오차나 외란이 다음 계산에 반영된다.

### 로보틱스에서 MPC가 유용한 이유

- 제약 조건 처리: 토크 제한, 관절 각도 제한, 속도 제한, 충돌 회피 등을 최적화 문제의 제약 조건으로 직접 넣을 수 있다. 기본 PID나 LQR에는 이런 제약을 넣을 자리가 없어, saturation, reference governor, constrained-LQR 같은 확장으로 처리한다.
- 비선형 모델 사용 가능: Nonlinear MPC에서는 비선형 동역학 모델을 그대로 쓸 수 있다.
- 미래 예측: 모델로 미래 궤적을 예측하고, 그 궤적의 오차까지 줄이도록 입력을 정한다. 보행 로봇은 이 예측을 이용해 다음 발을 내딛기 전에 무게 중심을 미리 옮긴다.
- 다목적 최적화: 목표 궤적 추종과 에너지 절약을 한 비용 함수에 넣고, 토크 제한은 제약으로 둘 수 있다.

### Linear MPC vs Nonlinear MPC

Linear MPC: 선형 모델(x(k+1) = A*x(k) + B*u(k))을 사용하고, 비용 함수가 이차(quadratic), 제약이 선형이면 문제가 QP(Quadratic Program)가 된다. feasible한 convex QP에서는 전역 최적해를 구할 수 있다. 실제 연산 시간은 문제 크기, 희소성 구조, solver와 실행 하드웨어에 좌우된다.

Nonlinear MPC (NMPC): 비선형 동역학 모델을 사용한다. 문제가 비볼록(non-convex)이 되어 풀기 어렵고, solver가 찾는 해는 지역 최적해다.

시스템이 선형에 충분히 가깝거나 제어 주기가 매우 짧아야 하면 Linear MPC를 쓰고, 비선형성이 크고 제어 주기에 여유가 있으면 NMPC를 쓴다.

### 실시간성 문제

MPC는 제어 주기 안에 최적화를 반복해서 풀어야 한다. MPC를 1kHz inner loop와 같은 주기로 동기 실행한다면 1ms 안에 상태 처리와 QP 풀이를 끝내야 한다. 그래서 MPC를 더 느린 outer loop로 돌리고 빠른 tracking controller와 묶기도 한다.

주요 QP solver:
- OSQP (https://osqp.org/): operator splitting(ADMM) 기반으로 sparse QP에 쓴다.
- qpOASES: active-set 기반이고 warm start를 지원한다. 연달아 비슷한 QP를 풀 때 빠르다.
- ECOS/Clarabel: second-order cone programming까지 처리 가능.

NMPC는:
- CasADi + IPOPT: 자동 미분 모델과 범용 interior-point NLP solver의 조합.
- acados (https://docs.acados.org/): CasADi로 모델을 쓰고, 실시간 실행용 C 코드를 생성한다.
- FORCESPRO, SNOPT 같은 상용 solver도 쓴다.

동기식으로 한 번 푸는 데 5ms가 걸리면 갱신 주기의 상한은 200Hz다. 실제 제어 주기는 전처리·통신·jitter를 포함하고, asynchronous solve나 별도 inner loop를 쓰면 구조가 달라진다.

### Linear MPC Python 예시

```python
import numpy as np
from scipy import sparse
import osqp

def linear_mpc(A, B, Q, R, Q_f, x0, N, x_min, x_max, u_min, u_max):
    """
    Linear MPC: QP로 변환하여 OSQP로 풀기.

    A, B: 이산시간 시스템 행렬
    Q: 상태 비용 (stage)
    R: 입력 비용
    Q_f: 종단 비용 (terminal)
    x0: 현재 상태
    N: 예측 구간 (horizon)
    x_min, x_max: 상태 제약
    u_min, u_max: 입력 제약
    """
    n = A.shape[0]  # 상태 차원
    m = B.shape[1]  # 입력 차원

    # 결정 변수: z = [x(0), x(1), ..., x(N), u(0), ..., u(N-1)]
    n_var = (N + 1) * n + N * m

    # --- 비용 함수 행렬 (P, q) ---
    # min 0.5 * z^T P z + q^T z
    P_blocks = [sparse.kron(sparse.eye(N), Q)]      # x(0) ~ x(N-1)
    P_blocks.append(Q_f)                              # x(N) terminal cost
    P_blocks.append(sparse.kron(sparse.eye(N), R))   # u(0) ~ u(N-1)
    P = sparse.block_diag(P_blocks, format='csc')
    q = np.zeros(n_var)

    # --- 등식 제약: 동역학 ---
    # x(k+1) = A*x(k) + B*u(k)
    # → A*x(k) + B*u(k) - x(k+1) = 0
    Ax_eq = sparse.kron(sparse.eye(N + 1), -sparse.eye(n), format='lil')  # 슬라이스 대입이 가능한 포맷
    # 대각선 아래 블록에 A 채우기
    for i in range(N):
        row_start = (i + 1) * n
        col_start = i * n
        Ax_eq[row_start:row_start + n, col_start:col_start + n] = A

    Bu_eq = sparse.lil_matrix(((N + 1) * n, N * m))
    for i in range(N):
        Bu_eq[(i + 1) * n:(i + 2) * n, i * m:(i + 1) * m] = B
    Bu_eq = sparse.csc_matrix(Bu_eq)

    A_eq = sparse.hstack([Ax_eq, Bu_eq], format='csc')
    l_eq = np.zeros((N + 1) * n)
    l_eq[:n] = -x0.flatten()  # 초기 조건
    u_eq = l_eq.copy()

    # --- 부등식 제약: 상태 및 입력 범위 ---
    A_ineq = sparse.eye(n_var, format='csc')
    l_ineq = np.concatenate([
        np.tile(x_min, N + 1),
        np.tile(u_min, N)
    ])
    u_ineq = np.concatenate([
        np.tile(x_max, N + 1),
        np.tile(u_max, N)
    ])

    # --- 전체 제약 결합 ---
    A_total = sparse.vstack([A_eq, A_ineq], format='csc')
    l_total = np.concatenate([l_eq, l_ineq])
    u_total = np.concatenate([u_eq, u_ineq])

    # --- OSQP 풀기 ---
    solver = osqp.OSQP()
    solver.setup(P, q, A_total, l_total, u_total,
                 warm_starting=True, verbose=False,
                 eps_abs=1e-6, eps_rel=1e-6)
    result = solver.solve()

    if result.info.status != 'solved':
        print(f"MPC 풀이 실패: {result.info.status}")
        return None, None

    # 첫 입력 u(0)과 예측 상태 궤적을 반환
    u_opt = result.x[(N + 1) * n:(N + 1) * n + m]
    x_pred = result.x[:(N + 1) * n].reshape(N + 1, n)
    return u_opt, x_pred


# 사용 예시: 1축 더블 인티그레이터 (상태 2차원: 위치, 속도)
dt = 0.1
A_d = np.array([[1, dt], [0, 1]])   # 이산시간
B_d = np.array([[0.5 * dt**2], [dt]])
n, m_ctrl = 2, 1

Q_mpc = sparse.diags([10.0, 1.0])
R_mpc = sparse.diags([0.1])
Q_f_mpc = sparse.diags([100.0, 10.0])  # terminal cost 크게

x0 = np.array([5.0, 0.0])  # 초기 위치 5m, 속도 0
N_horizon = 20

x_min_val = np.array([-10.0, -5.0])
x_max_val = np.array([10.0, 5.0])
u_min_val = np.array([-1.0])   # 가속도 제한
u_max_val = np.array([1.0])

u_opt, x_pred = linear_mpc(
    A_d, B_d,
    Q_mpc, R_mpc, Q_f_mpc,
    x0, N_horizon,
    x_min_val, x_max_val,
    u_min_val, u_max_val
)
print(f"최적 제어 입력: {u_opt}")
print(f"예측 궤적 (위치): {x_pred[:5, 0]}")
```

### 산업 사례

- **Boston Dynamics Atlas**: Boston Dynamics 블로그 "Picking Up Momentum"(2023년 1월, https://bostondynamics.com/blog/picking-up-momentum/)에 따르면 Atlas 컨트롤러의 중심에 MPC가 있다. 물체를 들고 던지는 실험에서는 전신 관절의 운동, 각 링크의 운동량, 로봇과 물체 사이의 힘을 모델에 포함했다.

---

## 6.6 임피던스/어드미턴스 제어 (Impedance/Admittance Control)

앞 절의 제어 기법은 주로 상태를 원하는 값으로 보내는 문제를 다뤘다. 로봇이 환경과 물리적으로 접촉하면 접촉력도 함께 다뤄야 한다.

### 위치 제어 vs 힘 제어 vs 임피던스 제어

- 위치 제어(Position Control): 목표 위치를 추종한다. 자유 공간이나 컴플라이언트한 환경에 맞는다. 강성이 큰 환경에서는 접촉력이 환경 강성과 위치 오차의 곱만큼 커진다. 테이블 위 컵을 집을 때 테이블 높이가 모델과 1mm만 달라도, 위치 제어기는 목표 위치까지 밀어 넣으려 하고 큰 힘이 생긴다.

- 힘 제어(Force Control): 목표 힘을 추종한다. 연마, 조립 같은 접촉 태스크에서 필요하다. 순수 힘 제어는 비접촉 상태에서 불안정하고, 힘 센서 노이즈에도 민감하다.

- 임피던스 제어(Impedance Control): 위치와 힘의 관계를 제어해, 로봇이 가상의 스프링-댐퍼처럼 움직이게 한다. 환경과 접촉하면 위치 편차에 비례하는 힘이 생기고, 접촉이 없으면 위치 제어처럼 동작한다.

### 가상 스프링-댐퍼 모델

목표 질량-스프링-댐퍼 관계:

```
F = M_d * (x_ddot_d - x_ddot) + D_d * (x_dot_d - x_dot) + K_d * (x_d - x)
```

관성 항을 뺀 간단한 형태:

```
F = K_d * (x_d - x) + D_d * (x_dot_d - x_dot)
```

- K_d: 가상 강성(virtual stiffness). 키우면 위치 추종이 정확해지고 접촉력도 커진다.
- D_d: 가상 감쇠(virtual damping). 진동을 억제한다.
- M_d: 가상 관성(virtual inertia). 조정하기 어려워 생략할 때가 많다.

K_d와 D_d는 태스크에 맞게 조정한다:
- 유리잔을 집을 때: K_d 낮게 (부드럽게), D_d 높게 (안정적으로)
- 볼트를 조일 때: K_d 높게 (정밀하게)
- 사람과 협업할 때: K_d 매우 낮게 (안전하게)

```python
import numpy as np

class ImpedanceController:
    """카르테시안 공간 임피던스 제어기 (1-DOF 간소화)."""

    def __init__(self, k_d: float, d_d: float, m_d: float = 0.0):
        self.k_d = k_d   # 가상 강성 (N/m)
        self.d_d = d_d   # 가상 감쇠 (N*s/m)
        self.m_d = m_d   # 가상 관성 (kg)

    def compute_force(self, x_d, x, x_dot_d, x_dot,
                      x_ddot_d=0.0, x_ddot=0.0) -> float:
        """목표 임피던스 관계에 따른 힘 계산."""
        f = (self.k_d * (x_d - x)
             + self.d_d * (x_dot_d - x_dot)
             + self.m_d * (x_ddot_d - x_ddot))
        return f


# 시뮬레이션: 로봇이 벽에 접근하여 접촉
dt = 0.001
controller = ImpedanceController(k_d=500.0, d_d=50.0)

# 로봇 + 환경
robot_mass = 2.0
position = 0.0
velocity = 0.0
target_position = 0.15  # 목표 위치
wall_position = 0.10    # 벽 위치 (목표보다 가까움)
wall_stiffness = 10000.0  # 벽의 강성

positions = []
forces = []
contact_forces = []

for step in range(10000):
    # 환경 접촉력
    if position > wall_position:
        f_env = -wall_stiffness * (position - wall_position)
    else:
        f_env = 0.0

    # 임피던스 제어 출력
    f_ctrl = controller.compute_force(
        x_d=target_position, x=position,
        x_dot_d=0.0, x_dot=velocity
    )

    # 동역학
    acceleration = (f_ctrl + f_env) / robot_mass
    velocity += acceleration * dt
    position += velocity * dt

    positions.append(position)
    forces.append(f_ctrl)
    contact_forces.append(-f_env)

# 결과: position은 wall_position 근처에서 안정화
# 정상상태에서 500*(0.15-x) = 10000*(x-0.10)이므로 x ≈ 0.1024 m, 접촉력 ≈ 24 N
print(f"최종 위치: {positions[-1]:.4f} m (벽: {wall_position} m)")
print(f"최종 접촉력: {contact_forces[-1]:.2f} N")
# 목표 위치(0.15 m)까지 밀어 넣는 순수 위치 제어였으면 벽을 10000 N/m * 0.05 m = 500 N으로 눌렀을 것이다
```

### Admittance Control

임피던스 제어는 위치 편차를 받아 힘을 낸다. 어드미턴스 제어는 측정한 힘을 받아 위치 명령을 낸다.

```
x_d_new = x_d + (1 / K_d) * F_ext      # 강성 항: 힘 → 위치 오프셋
x_dot_d = (1 / D_d) * F_ext             # 감쇠 항: 힘 → 목표 속도 (힘의 미분이 아니다)
```

외력 F_ext를 측정해 가상 임피던스 모델에 넣고, 그 결과로 고친 목표 위치를 강성이 높은 기존 위치 제어기에 넘긴다.

산업용 로봇에는 정밀한 위치 제어기가 내장되어 있다. 외부 인터페이스도 주로 위치나 속도 명령을 받는다. 그래서 힘 센서(F/T sensor)로 외력을 측정해 위치 명령을 고치는 어드미턴스 제어를 많이 쓴다.

Franka Emika Panda처럼 외부에서 관절 토크를 명령할 수 있는 연구용 로봇에서는 임피던스 제어를 바로 구현할 수 있다.

---

## 6.7 심화: Whole-Body Control

휴머노이드 로봇이나 사족 보행 로봇은 관절이 수십 개이고, 여러 개의 접촉점(발, 손)을 동시에 관리해야 하며, 균형도 유지해야 한다. 이런 시스템은 관절별 PID로는 접촉과 균형을 함께 다루기 어려워, 전신(whole-body) 수준에서 한꺼번에 제어한다.

### Task-space vs Joint-space

- Joint-space control: 관절 각도 q를 직접 제어한다. 구조가 간단하다. 태스크 수준의 목표(end-effector 위치, 무게중심 위치)를 이루려면 역기구학(inverse kinematics)을 먼저 풀어야 한다.

- Task-space control: 태스크 좌표(카르테시안 위치, 방향)에서 직접 제어한다. 태스크 목표를 그대로 적을 수 있고, 관절 공간으로 옮기는 계산은 제어기 안에서 한다.

### Operational Space Control (Khatib, 1987)

Khatib의 Operational Space Framework는 태스크 공간에서 동역학을 직접 유도해 task-space 제어의 바탕이 됐다.

조인트 공간 동역학:

```
M(q) * q_ddot + C(q, q_dot) * q_dot + g(q) = tau + J^T * F_ext
```

M(q), C(q, q_dot), g(q), tau는 Ch.5 §5.3의 관성 행렬, 코리올리·원심력 행렬, 중력 토크, 관절 토크다(6.3절의 출력 행렬 C와는 다르다). J는 end-effector의 자코비안, F_ext는 end-effector에 가해지는 외력이다.

태스크 공간으로 변환:

```
Lambda(q) * x_ddot + mu(q, q_dot) * x_dot + p(q) = F + F_ext
```

여기서 Lambda = (J * M^(-1) * J^T)^(-1)은 태스크 공간 관성 행렬이다.

태스크 공간 목표 가속도 x_ddot_d를 내는 관절 토크:

```
tau = J^T * Lambda * (x_ddot_d - J_dot * q_dot) + C * q_dot + g(q)
```

x_ddot_d를 임피던스 법칙으로 정하면 태스크 공간에서 원하는 임피던스를 구현할 수 있다.

### QP 기반 Whole-Body Control

QP 기반 WBC는 매 제어 주기에 QP(Quadratic Program)를 풀어 여러 태스크를 함께 처리한다.

기본 구조:

```
minimize    || J_task * q_ddot + J_task_dot * q_dot - x_ddot_d ||^2  (태스크 추종)
subject to  M(q)*q_ddot + h(q,q_dot) = S^T*tau + J_c^T*F_c  (동역학)
            F_c ∈ friction cone                   (접촉력 제약)
            tau_min ≤ tau ≤ tau_max               (토크 제한)
```

여기서:
- J_task: 태스크 자코비안
- J_c: 접촉 자코비안
- F_c: 접촉력
- S: selection matrix (underactuated 자유도 제거)

**다중 태스크 우선순위**: 오른손을 목표 위치로 보내는 일, 균형 유지, 관절 한계 준수가 동시에 걸리면 서로 충돌할 수 있다. 이때 태스크에 우선순위를 둔다.

1. 최고 우선순위: 접촉 제약 (발이 바닥에 붙어 있어야 한다), 관절 한계
2. 높은 우선순위: 균형 유지 (CoM 제어)
3. 중간 우선순위: end-effector 위치 제어
4. 낮은 우선순위: 자세 유지 (null-space)

엄격한 우선순위(strict hierarchy)는 null-space projection을 쓰거나, 각 우선순위의 QP를 차례로 풀어(hierarchical QP) 구현한다. 가중치만 달리 두어 한 QP로 합치는 soft priority 방식도 있다.

### Contact-Consistent Control

보행 로봇에서 접촉력은 물리적으로 타당해야 한다:

- 단방향 접촉(unilateral contact): 발은 바닥을 밀기만 한다. F_z >= 0.
- 마찰 원뿔(friction cone): 접선력이 수직력 × 마찰계수 이하여야 한다. sqrt(F_x^2 + F_y^2) <= mu * F_z.
- ZMP/CoP 제약: 압력 중심(Center of Pressure)을 지지 다각형(support polygon) 안에 둔다. 압력 중심이 다각형 경계에 닿으면 발이 기울기 시작한다.

이 제약을 QP에 넣으면 물리적으로 실현 가능한 제어 입력을 얻는다. 마찰 원뿔은 원래 비선형(second-order cone)이지만, 다면체로 근사(linearized friction cone)하면 QP로 풀 수 있다.

```python
import numpy as np

def linearized_friction_cone(mu, n_edges=8):
    """
    마찰 원뿔의 다면체 근사.
    반환: A_cone * F <= 0 형태의 제약 행렬.
    F = [fx, fy, fz]^T
    """
    A_rows = []
    for i in range(n_edges):
        theta = 2 * np.pi * i / n_edges
        # mu * fz >= cos(theta)*fx + sin(theta)*fy
        # → cos(theta)*fx + sin(theta)*fy - mu*fz <= 0
        row = [np.cos(theta), np.sin(theta), -mu]
        A_rows.append(row)
    # fz >= 0 → -fz <= 0
    A_rows.append([0, 0, -1])
    return np.array(A_rows)

# 마찰계수 0.7, 8각형 근사
A_friction = linearized_friction_cone(mu=0.7)
print(f"마찰 원뿔 제약 행렬 shape: {A_friction.shape}")
# (9, 3) → 9개의 선형 부등식으로 3D 마찰 원뿔을 근사
```

---

## 6.8 심화: Lyapunov 안정성과 적응 제어

제어기를 설계한 뒤에는 폐루프가 안정한지 증명한다. 시뮬레이션에서 잘 동작한 결과는 안정성 증명이 되지 않는다. Lyapunov 이론이 이 증명에 쓰인다.

### Lyapunov 안정성

비선형 시스템 x_dot = f(x)에서 원점이 평형점이라 하자 (f(0) = 0).

Lyapunov의 직접 방법(direct method): 함수 V(x)가 다음을 만족하면 원점은 안정하다.

1. V(0) = 0
2. V(x) > 0 for all x != 0 (양정치)
3. V_dot(x) = dV/dx * f(x) <= 0 (비증가)

x ≠ 0에서 V_dot(x) < 0이면 점근적 안정(asymptotically stable)이다. 원점 근처에서 시작한 상태가 원점으로 수렴한다.

V(x)를 에너지로 보면, 양수이면서 시간에 따라 줄어드는 에너지는 최솟값인 평형점에서 멈춘다.

V(x)는 시스템마다 따로 찾는다. 기계 시스템에서는 역학적 에너지(운동에너지 + 위치에너지)를 먼저 후보로 놓는다. LQR 폐루프에서는 V(x) = x^T * P * x (P는 ARE의 해)가 Lyapunov 함수가 되고, LQR의 안정성 증명이 여기서 나온다.

### 적응 제어 (Adaptive Control)

로봇 팔에 실린 payload의 무게를 모르거나 마찰 계수가 시간에 따라 변할 때처럼, 모델 파라미터를 정확히 모를 때 쓴다.

제어기 안에 파라미터 추정기(estimator)를 두고 제어와 추정을 함께 한다.

로봇 동역학은 파라미터에 선형인 형태로 쓸 수 있다.

```
M(q)*q_ddot + C(q,q_dot)*q_dot + g(q) = Y(q, q_dot, q_ddot) * theta
```

여기서 Y는 regressor matrix다. theta는 질량·관성·마찰 등을 묶은 동역학 파라미터 벡터다.

적응 제어 법칙:

```
tau = Y * theta_hat - K_d * s
theta_hat_dot = -Gamma * Y^T * s
```

여기서 s = q̃_dot + Λ*q̃ (q̃ = q - q_d)는 sliding variable, theta_hat은 파라미터 추정값, Gamma는 적응 게인 행렬이다. Slotine–Li(1987)의 원래 법칙에서 이 Y는 실제 q_dot, q_ddot 대신 기준 속도 q_dot_r = q_dot_d - Λ*q̃와 그 미분으로 계산하므로, 관절 가속도를 측정할 필요가 없다.

Lyapunov 함수를 V = 0.5*s^T*M*s + 0.5*theta_tilde^T*Gamma^(-1)*theta_tilde로 잡으면 V_dot <= 0을 보일 수 있고, 이로부터 추종 오차가 0으로 수렴함을 증명한다. theta_hat이 실제 theta로 수렴하려면 입력 궤적이 persistent excitation 조건을 만족해야 한다.

### Robust Control

모델 불확실성이 있지만 그 범위(bound)는 아는 경우에 쓴다.

- H-infinity control: 최악의 외란에 대한 성능을 최적화한다. 보장하는 것은 유도 L2 노름의 상한으로, 외란에서 오차로 가는 에너지 이득이 $\gamma$ 이하라는 뜻이다. 외란의 에너지가 커지면 오차의 상한도 비례해 커진다. 설계에 Riccati 부등식과 LMI를 쓰며, 최악의 경우를 기준으로 하므로 결과 제어기가 보수적일 수 있다.

- Sliding Mode Control: 상태를 슬라이딩 면(sliding surface)으로 유한 시간 내에 끌어온 뒤, 슬라이딩 면 위에서 원하는 동특성을 따르게 한다. 모델 불확실성에 강건하다. 슬라이딩 면 근처에서 고주파 스위칭이 일어나 actuator에 부담을 주는데, 이를 chattering이라 한다. Boundary layer approach나 higher-order sliding mode로 완화한다.

### 언제 쓰는가, 언제 안 쓰는가

| 상황 | 추천 | 비추천 |
|------|------|--------|
| 모델이 정확하고 선형성 충분 | LQR, MPC | 적응 제어 (과설계) |
| 파라미터 불확실성이 큼 | 적응 제어 | PID만으로 버티기 |
| 불확실성 범위를 알고 있음 | Robust control (H-inf) | 적응 제어 (불필요) |
| 안전 인증이 필요함 | Lyapunov 기반 증명 | "시뮬레이션에서 됐으니까 OK" |
| 빠르게 프로토타입 | PID + feedforward | 처음부터 H-infinity |

적응 제어와 sliding mode는 불확실성의 구조와 필요한 보장을 보고 고른다. 제어기 안정성을 체계적으로 입증하려면 Lyapunov 함수나 그에 준하는 수학적 분석이 필요하다. 기능안전 규격은 위험 분석, 안전 등급 결정, 검증·확인 증거를 요구하며 특정 증명 기법을 지정하지 않는다. 안정성 해석 결과는 그 증거의 하나로 쓸 수 있다.

---

## 6.9 추천 자료

> **Åström & Murray, "Feedback Systems: An Introduction for Scientists and Engineers"**
> https://fbswiki.org/
> 무료 PDF. PID, 상태공간, 주파수 응답을 한 흐름으로 설명하는 입문서다.

> **Steve Brunton, "Control Bootcamp" (YouTube)**
> https://www.youtube.com/playlist?list=PLMrJAkhIeNNR20Mz-VpzgfQs5zrYi085m
> 상태공간, 가제어성, 가관측성, LQR을 영상 한 편에 5~30분씩 나눠 설명한다. 교재의 수식 유도 전에 개념을 잡을 때 본다.

> **Slotine & Li, "Applied Nonlinear Control"**
> 비선형 제어, Lyapunov 안정성, 적응 제어를 다루는 교재다. 6.8절 내용을 더 깊이 볼 때 참고한다.

> **Russ Tedrake, "Underactuated Robotics" (MIT OCW)**
> https://underactuated.csail.mit.edu/
> 무료 온라인 교재 + 강의. MPC, trajectory optimization, 제어와 계획(planning)의 연결을 다룬다. Drake 라이브러리의 이론적 배경이기도 하다.

> **python-control library**
> https://python-control.readthedocs.io/
> Python으로 제어 시스템을 분석·설계하는 라이브러리로, MATLAB Control System Toolbox와 기능이 비슷하다. Bode plot, root locus, state-space 분석 등을 지원한다.

> **CasADi**
> https://web.casadi.org/
> 자동 미분과 NLP solver(IPOPT, SNOPT 등)를 연결하는 프레임워크. Python, MATLAB, C++ 인터페이스가 있다.

> **OSQP (Operator Splitting Quadratic Program)**
> https://osqp.org/
> QP solver. 코드 생성(code generation)으로 임베디드 시스템에 배포할 수 있다. C로 구현되어 있고 Python, MATLAB, Julia 등의 바인딩이 있다.

> **주요 논문**
> - [Hogan, "Impedance Control: An Approach to Manipulation: Part I—Theory" (ASME JDSMC 107(1), 1985)](https://doi.org/10.1115/1.3140702) — 임피던스 제어의 원논문(3부작 중 1부)
> - [Khatib, "A Unified Approach for Motion and Force Control of Robot Manipulators: The Operational Space Formulation" (IEEE JRA 1987)](https://doi.org/10.1109/JRA.1987.1087068) — Operational Space Control의 원논문(IEEE JRA 3(1):43–53)
> - [Khazoom et al., "Tailoring Solution Accuracy for Fast Whole-Body Model Predictive Control of Legged Robots" (RA-L 2024, arXiv:2407.10789)](https://arxiv.org/abs/2407.10789) — SQP 한 번의 QP 부분 문제를 ADMM(OSQP)으로 낮은 정확도까지만 풀어 whole-body NMPC를 실시간으로 돌리는 방법. MIT Humanoid 실기에서 90Hz(dt=25ms 설정)로 실행

---

## 기술 흐름

```
1922 ── Minorsky, 자동 조타선 방향 안정성 분석으로 PID 제어 정식화
1960 ── 상태공간 이론 (Kalman)
1960 ── LQR (Kalman)
1981 ── H-infinity 최적 감도 문제 제기 (Zames)
1985 ── Impedance Control 개념 (Hogan)
1987 ── Operational Space Control (Khatib)
1989 ── H-infinity 문제의 상태공간 해 (Doyle, Glover, Khargonekar, Francis)
2002 ── Explicit MPC (Bemporad et al.): 오프라인으로 푼 구간별 선형 제어 법칙으로 빠른 샘플링 MPC
2010 ── Fast online MPC (Wang & Boyd): 구조를 이용한 온라인 QP로 원 논문 예제에서 200 Hz MPC
2023 ── Boston Dynamics Atlas: 전신 동작과 물체에 가하는 힘을 포함한 MPC 설명 공개
```

---
