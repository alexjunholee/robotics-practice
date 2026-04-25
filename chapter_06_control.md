# Ch.6 — 제어 이론 (Control Theory)


로봇이 세상을 인식하는 것과 세상에 실제로 영향을 미치는 것은 완전히 다른 문제다. 인식 파이프라인이 아무리 정교해도, 모터에 보내는 전류 하나를 잘못 계산하면 로봇은 넘어지거나 물건을 부수거나 사람을 다치게 한다. 제어 이론은 "원하는 동작을 실제로 실현하는 방법"에 대한 학문이다.

---

## 6.1 왜 제어를 배우는가

인식(Perception)이 "세상을 이해하는 것"이라면, 제어(Control)는 "세상에 영향을 미치는 것"이다. 둘 다 없으면 로봇이 아니라 그냥 센서 덩어리 또는 모터 덩어리다.

제어를 배워야 하는 이유는 단순하다:

- **모터는 생각보다 멍청하다.** "관절을 30도로 보내라"라고 명령하면, 모터는 그냥 최대 전류를 때려넣고 30도를 지나쳐서 진동한다. 이걸 부드럽게 원하는 위치에 도달시키는 것이 제어다.
- **외란(disturbance)은 항상 존재한다.** 바닥이 미끄럽거나, 바람이 불거나, 페이로드 무게가 예상과 다르거나. 센서-액추에이터 루프를 닫아서(feedback) 이런 불확실성에 대응해야 한다.
- **안전이 걸려 있다.** 산업용 로봇 팔이 사람 옆에서 작업하는데, 힘 제어가 없으면 사람 팔뼈가 부러진다. 과장이 아니다.

제어 이론의 범위는 넓다. 여기서는 로보틱스 현장에서 실제로 쓰이는 것들에 집중한다. PID, 상태공간 제어, MPC, 임피던스 제어, Whole-Body Control 순으로 이어진다.

한 가지 미리 말해두자면, 제어 이론은 수학이 많이 나온다. 선형대수와 미분방정식에 익숙하지 않다면, 이 장을 읽기 전에 최소한 행렬 연산과 고유값(eigenvalue) 개념은 복습하고 오는 것을 권한다.

---

## 6.2 PID 제어

PID(Proportional-Integral-Derivative)는 1922년 Minorsky가 선박 조타 시스템을 위해 제안한 이래로, 100년이 넘게 산업 현장에서 가장 널리 쓰이는 제어기이다. 세상 모든 제어 엔지니어가 처음 배우는 것이고, 은퇴할 때까지 쓰는 것이다.

### 기본 구조

오차 e(t) = r(t) - y(t)로 정의한다. r(t)는 목표값(reference), y(t)는 현재 출력이다.

```
u(t) = Kp * e(t) + Ki * integral(e(τ)dτ, 0, t) + Kd * de(t)/dt
```

각 항의 역할:

- **P (Proportional)**: 현재 오차에 비례하여 제어 입력을 생성한다. Kp가 크면 반응이 빠르지만, 오버슈트가 커지고 진동이 발생한다. P 항만으로는 정상상태 오차(steady-state error)가 남는다. 목표값 근처에서 오차가 작아지면 제어 입력도 작아지기 때문이다.

- **I (Integral)**: 오차의 누적값에 비례한다. 정상상태 오차를 제거하는 역할을 한다. 중력이나 마찰 같은 상수 외란이 있을 때 필수적이다. 다만 과도하면 wind-up 현상이 발생한다. 오차가 오랫동안 누적되어 제어 입력이 포화(saturation)된 상태에서, 목표에 도달한 후에도 누적된 적분값 때문에 큰 오버슈트가 생기는 문제다. 실무에서는 anti-windup 로직을 반드시 구현해야 한다.

- **D (Derivative)**: 오차의 변화율에 비례한다. 오차가 빠르게 줄어들고 있으면 제어 입력을 줄여서 오버슈트를 억제한다. 일종의 "브레이크" 역할이다. 문제는 미분이 노이즈에 극도로 민감하다는 것이다. 센서 노이즈가 있는 실제 시스템에서는 D 항에 저역통과 필터(low-pass filter)를 걸어야 한다. 그래서 현장에서는 D 항을 아예 안 쓰고 PI 제어만 하는 경우도 많다.

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
            # Anti-windup: 포화 시 적분값 역산
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

    # 간단한 1차 동역학: F = ma, 감쇠 포함
    acceleration = (force - 0.5 * velocity) / mass
    velocity += acceleration * dt
    position += velocity * dt
    positions.append(position)
```

### 튜닝 방법

**Ziegler-Nichols 방법**: 고전적 튜닝법이다. Ki = 0, Kd = 0으로 놓고 Kp를 올려가면서 시스템이 지속 진동(sustained oscillation)하는 임계 이득 Ku와 진동 주기 Tu를 구한다. 그리고 다음 표에 따라 게인을 설정한다.

```
PID:  Kp = 0.6 * Ku,  Ki = 2 * Kp / Tu,  Kd = Kp * Tu / 8
PI:   Kp = 0.45 * Ku,  Ki = 1.2 * Kp / Tu
P:    Kp = 0.5 * Ku
```

솔직히 Ziegler-Nichols로 튜닝하면 오버슈트가 꽤 크게 나온다. 시작점으로는 괜찮지만, 그 후에 수동 미세조정이 필수다.

**실무에서의 경험적 튜닝**: 현실에서는 대부분 이렇게 한다.

1. D, I를 0으로 놓는다.
2. P를 올린다. 시스템이 빠르게 반응하되 진동하지 않는 선에서 멈춘다.
3. 정상상태 오차가 있으면 I를 조금씩 올린다. Wind-up 조심.
4. 오버슈트가 크면 D를 조금 넣는다. 노이즈 필터 확인.

이 과정을 "느낌으로 한다"고 하면 교수님이 싫어하겠지만, 현장에서는 대부분 이렇게 한다. 시스템 모델이 정확하면 시뮬레이션에서 먼저 튜닝하고 실기에 적용하는 것이 훨씬 효율적이다.

### PID의 한계

PID는 강력하지만 분명한 한계가 있다:

- **SISO(Single-Input Single-Output) 전용이다.** 6축 로봇 팔처럼 관절 간 커플링이 있는 시스템에서는 각 관절에 독립적으로 PID를 걸면 성능이 떨어진다. 한 관절의 움직임이 다른 관절에 외란으로 작용하기 때문이다.
- **비선형 시스템에 약하다.** PID는 기본적으로 선형 제어기다. 로봇 동역학은 비선형이다. 작동점(operating point) 근처에서만 잘 동작한다.
- **제약 조건을 처리할 수 없다.** 토크 제한, 관절 각도 제한, 속도 제한 같은 물리적 제약을 PID 구조 안에서 명시적으로 다룰 방법이 없다.
- **미래를 예측하지 않는다.** 현재 오차만 보고 반응한다. Feedforward가 없으면 추종 성능이 제한된다.

그럼에도 PID가 100년째 쓰이는 이유는 간단하다: 구현이 쉽고, 이해하기 쉽고, 웬만한 시스템에서 "적당히" 동작한다. 제어 대상이 단순하고 성능 요구가 극단적이지 않으면 PID로 충분하다. 산업용 로봇의 각 관절 서보 제어는 지금도 PID 기반이 대부분이다.

---

## 6.3 상태공간 표현 (State-Space Representation)

PID는 입력-출력 관계만 본다. 시스템 "내부"에서 무슨 일이 일어나는지는 모른다. 상태공간 표현은 시스템의 내부 상태를 명시적으로 기술하는 방법이다.

### 기본 형태

연속시간 선형 시스템:

```
x_dot(t) = A * x(t) + B * u(t)    (상태 방정식)
y(t)     = C * x(t) + D * u(t)    (출력 방정식)
```

- x(t): 상태 벡터 (n x 1). 시스템을 완전히 기술하는 데 필요한 최소 변수 집합.
- u(t): 입력 벡터 (m x 1). 제어 입력.
- y(t): 출력 벡터 (p x 1). 측정 가능한 출력.
- A: 시스템 행렬 (n x n). 시스템의 고유 동특성을 결정한다.
- B: 입력 행렬 (n x m). 입력이 상태에 미치는 영향.
- C: 출력 행렬 (p x n). 상태에서 출력으로의 매핑.
- D: 직접 전달 행렬 (p x m). 대부분의 물리 시스템에서 0이다.

예를 들어, 질량-스프링-댐퍼 시스템 (m * x_ddot + c * x_dot + k * x = F)에서 상태를 x1 = 위치, x2 = 속도로 잡으면:

```
A = [[0, 1], [-k/m, -c/m]]
B = [[0], [1/m]]
C = [[1, 0]]   (위치만 측정)
D = [[0]]
```

### 전달함수와의 관계

전달함수 G(s) = C * (sI - A)^(-1) * B + D 이다. 전달함수는 SISO 시스템에서 편리하지만, MIMO(Multi-Input Multi-Output) 시스템에서는 상태공간이 훨씬 자연스럽다. 로봇은 거의 항상 MIMO 시스템이므로, 상태공간 표현이 표준이다.

### 가제어성 (Controllability)

시스템이 가제어(controllable)하다는 것은, 임의의 초기 상태에서 임의의 최종 상태로 유한 시간 내에 이동할 수 있다는 것이다. 가제어성 행렬:

```
C_ctrl = [B, A*B, A^2*B, ..., A^(n-1)*B]
```

이 행렬의 rank가 n이면 가제어이다. rank가 n보다 작으면, 제어 입력으로 도달할 수 없는 상태가 존재한다는 뜻이다. 그런 시스템에 LQR을 적용하면 안 된다.

### 가관측성 (Observability)

시스템이 가관측(observable)하다는 것은, 출력 y(t)를 관찰하여 초기 상태 x(0)를 유일하게 결정할 수 있다는 것이다. 가관측성 행렬:

```
O = [C; C*A; C*A^2; ...; C*A^(n-1)]
```

rank가 n이면 가관측이다. 가관측하지 않으면 상태 추정(observer, Kalman filter)이 제대로 동작하지 않는다.

### 왜 PID에서 상태공간으로 넘어가야 하는가

PID로 각 관절을 독립적으로 제어하면, 관절 간 동적 커플링을 무시하게 된다. 2-DOF 로봇 팔만 해도 한 관절이 빠르게 움직이면 다른 관절에 원심력과 코리올리 힘이 작용한다. 이걸 외란으로 처리하면 PID의 I 항이 열심히 보상하겠지만, 응답이 느리고 성능이 나쁘다.

상태공간에서는 시스템 전체를 하나의 모델로 기술하고, 모든 상태 변수를 동시에 고려하여 제어 입력을 계산한다. 이 방식이 다음 절의 LQR과 MPC의 기반이 된다.

```python
import numpy as np
from scipy import signal
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
# 양의 실수부를 가진 극점이 있으면 → 불안정 시스템 (도립진자가 그렇다)
```

---

## 6.4 LQR (Linear Quadratic Regulator)

PID가 "경험과 튜닝"에 의존한다면, LQR은 "최적화"에 기반한 제어기이다. 주어진 비용 함수를 최소화하는 제어 입력을 해석적으로 구할 수 있다.

### 비용 함수

```
J = integral_0^inf (x(t)^T * Q * x(t) + u(t)^T * R * u(t)) dt
```

- Q (n x n, 양의 반정치): 상태 오차에 대한 페널티. "상태가 0에서 벗어나는 것이 얼마나 싫은가."
- R (m x m, 양정치): 제어 입력에 대한 페널티. "제어 에너지를 얼마나 아끼고 싶은가."

Q를 크게 하면 상태가 빠르게 0으로 수렴하지만 제어 입력이 커진다. R을 크게 하면 제어 입력이 작아지지만 상태 수렴이 느려진다. 이것이 LQR의 본질적인 트레이드오프다.

### Q, R 행렬 튜닝

실용적인 방법: Q와 R을 대각 행렬로 놓고, 각 대각 원소를 해당 상태/입력의 허용 범위의 역수 제곱으로 설정한다.

```
Q_ii = 1 / (허용 가능한 x_i의 최대값)^2
R_jj = 1 / (허용 가능한 u_j의 최대값)^2
```

예: 카트 위치가 0.5m 이내, 진자 각도가 0.1rad 이내, 힘이 20N 이내를 원한다면:

```
Q = diag(1/0.5^2, 0, 1/0.1^2, 0) = diag(4, 0, 100, 0)
R = [1/20^2] = [0.0025]
```

이것은 출발점일 뿐이다. 이후 시뮬레이션을 돌려가며 조정한다.

### Algebraic Riccati Equation (ARE)

LQR의 최적 게인 K는 다음 Algebraic Riccati Equation의 해 P로부터 구한다:

```
A^T * P + P * A - P * B * R^(-1) * B^T * P + Q = 0
```

최적 상태 피드백 게인: K = R^(-1) * B^T * P

제어 법칙: u(t) = -K * x(t)

이 결과의 핵심은, 폐루프 시스템 (A - BK)의 모든 고유값이 좌반면에 놓인다는 것이 보장된다는 점이다. 즉, 안정성이 수학적으로 증명된다.

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

- **선형 모델이 필요하다.** 비선형 시스템은 작동점 근처에서 선형화해야 한다. 작동점에서 멀어지면 성능이 급격히 떨어진다.
- **제약 조건을 명시적으로 처리할 수 없다.** 토크 제한, 속도 제한 같은 물리적 제약을 비용 함수에 넣을 수 없다. 제어 입력이 포화되면 최적성이 깨진다.
- **전체 상태를 알아야 한다.** u = -Kx이므로 모든 상태 변수를 측정하거나 추정(observer)해야 한다.
- **추종(tracking) 문제에 그대로 적용 불가.** 기본 LQR은 regulator, 즉 상태를 0으로 보내는 문제만 풀 수 있다. 시변 목표를 추종하려면 확장이 필요하다.

이런 한계를 극복하기 위해 MPC가 등장한다.

---

## 6.5 MPC (Model Predictive Control)

MPC(Model Predictive Control)는 매 제어 주기마다 유한 구간(finite horizon) 최적화 문제를 풀어 제어 입력을 계산하는 방법이다. 2020년대 로보틱스에서 가장 널리 쓰이는 제어 기법 중 하나다.

### 기본 개념

매 time step k에서 다음을 수행한다:

1. 현재 상태 x(k)를 측정 또는 추정한다.
2. 모델을 이용하여 N step 앞까지 미래를 예측한다.
3. 비용 함수를 최소화하는 입력 시퀀스 {u(k), u(k+1), ..., u(k+N-1)}을 구한다. 이때 제약 조건을 명시적으로 반영한다.
4. 첫 번째 입력 u(k)만 실제로 적용하고, 나머지는 버린다.
5. 다음 time step에서 1로 돌아간다.

이것을 "receding horizon" 전략이라 한다. 매번 최적화를 새로 풀기 때문에, 모델 오차나 외란에 대한 피드백 효과가 자연스럽게 생긴다.

### 왜 로보틱스에서 MPC가 대세인가

- **제약 조건 처리**: 토크 제한, 관절 각도 제한, 속도 제한, 충돌 회피 등을 최적화 문제의 제약 조건으로 직접 넣을 수 있다. PID나 LQR로는 불가능하다.
- **비선형 모델 사용 가능**: Nonlinear MPC에서는 비선형 동역학 모델을 그대로 쓸 수 있다.
- **미래 예측**: 단순히 현재 오차에 반응하는 것이 아니라, 미래 궤적을 예측하여 능동적으로 대응한다. 보행 로봇이 다음 발을 내딛기 전에 미리 무게 중심을 이동시키는 것이 이 원리다.
- **다목적 최적화**: 비용 함수에 여러 목표를 동시에 넣을 수 있다. "목표 궤적을 추종하면서 에너지를 아끼고 토크 제한을 지켜라."

### Linear MPC vs Nonlinear MPC

**Linear MPC**: 선형 모델(x(k+1) = A*x(k) + B*u(k))을 사용하고, 비용 함수가 이차(quadratic), 제약이 선형이면 문제가 QP(Quadratic Program)가 된다. QP는 볼록(convex) 최적화이므로 전역 최적해를 빠르게 구할 수 있다. 실시간 제어에 적합하다.

**Nonlinear MPC (NMPC)**: 비선형 동역학 모델을 사용한다. 문제가 비볼록(non-convex)이 되어 풀기 어렵고, 전역 최적해를 보장하지 못한다. 하지만 로봇 동역학을 정확히 반영할 수 있으므로 성능이 좋다. CasADi + IPOPT 조합이 표준 도구다.

실무에서의 선택: 시스템이 충분히 선형에 가깝거나 제어 주기가 매우 짧아야 하면 Linear MPC를 쓰고, 비선형성이 크고 제어 주기에 여유가 있으면 NMPC를 쓴다.

### 실시간성 문제

MPC의 최대 난관은 매 제어 주기마다 최적화를 풀어야 한다는 것이다. 보행 로봇이 1kHz로 제어된다면, 1ms 안에 QP를 풀어야 한다.

주요 QP solver:
- **OSQP** (https://osqp.org/): operator splitting 기반, sparse QP에 강하다. 대부분의 Linear MPC에서 첫 번째 선택.
- **qpOASES**: active-set 기반, warm-starting이 가능하여 연속적인 QP 풀이에 효율적.
- **ECOS/Clarabel**: second-order cone programming까지 처리 가능.

NMPC는:
- **CasADi** + **IPOPT**: 자동 미분 + interior-point method. NMPC 구현의 사실상 표준.
- **acados** (https://docs.acados.org/): CasADi 기반이지만 실시간성에 최적화됨. C 코드 생성 가능.

solver 속도가 곧 제어 주기를 결정한다. solver가 5ms 걸리면 200Hz가 한계다. MPC 엔지니어가 solver를 매우 신경 쓰는 이유다.

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
    Ax_eq = sparse.kron(sparse.eye(N + 1), -sparse.eye(n))
    Au_shift = sparse.kron(sparse.eye(N, N + 1, 1), sparse.eye(n))
    # 수정: 좌하단에 A 추가
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

    # 첫 번째 입력만 반환
    u_opt = result.x[(N + 1) * n:(N + 1) * n + m]
    x_pred = result.x[:(N + 1) * n].reshape(N + 1, n)
    return u_opt, x_pred


# 사용 예시: 2차원 더블 인티그레이터
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
u_min_val = np.array([-1.0])   # 힘 제한
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

- **Boston Dynamics Atlas (2019~)**: MPC + Whole-Body Control 조합. 비선형 MPC로 접촉 시퀀스를 예측하고, WBC로 관절 토크를 실시간 분배한다.
- **Unitree H1/G1 (2023~)**: 학습 기반 정책(reinforcement learning)이 high-level 명령을 생성하고, MPC가 low-level 궤적 추종을 담당하는 하이브리드 구조.
- **Figure 01 (2024)**: LLM이 태스크 레벨 계획을 세우고, MPC가 manipulation 궤적을 최적화한다. 제어와 AI의 결합 사례.

---

## 6.6 임피던스/어드미턴스 제어 (Impedance/Admittance Control)

지금까지 다룬 제어 기법들은 주로 "위치를 원하는 곳에 보내는 것"에 집중했다. 하지만 로봇이 환경과 물리적으로 접촉하는 순간, 위치 제어만으로는 부족해진다.

### 위치 제어 vs 힘 제어 vs 임피던스 제어

- **위치 제어(Position Control)**: 목표 위치를 추종한다. 환경이 없거나 매우 강성(rigid)인 환경에서 적합하다. 하지만 로봇 팔이 테이블 위의 컵을 집으려는데, 테이블 높이가 1mm만 달라도 위치 제어기는 이를 모른 채 계속 밀어 넣으려 하고, 과도한 힘이 발생한다.

- **힘 제어(Force Control)**: 목표 힘을 추종한다. 연마, 조립 같은 접촉 태스크에서 필요하다. 그러나 순수 힘 제어는 비접촉 상태에서 불안정하다. 힘 센서 노이즈에도 민감하다.

- **임피던스 제어(Impedance Control)**: 위치와 힘의 관계를 제어한다. 로봇이 가상의 스프링-댐퍼 시스템처럼 행동하도록 만든다. 환경과 접촉하면 자연스럽게 힘이 발생하고, 비접촉 상태에서는 위치 제어처럼 동작한다.

### 가상 스프링-댐퍼 모델

임피던스 제어의 핵심 아이디어:

```
F = M_d * (x_ddot_d - x_ddot) + D_d * (x_dot_d - x_dot) + K_d * (x_d - x)
```

또는 관성 항을 무시한 간소화 버전:

```
F = K_d * (x_d - x) + D_d * (x_dot_d - x_dot)
```

- K_d: 가상 강성(virtual stiffness). 크면 위치 추종이 정확하지만, 접촉 시 힘이 크다.
- D_d: 가상 감쇠(virtual damping). 진동을 억제한다.
- M_d: 가상 관성(virtual inertia). 보통 조정하기 어려워서 관성 항은 생략하는 경우가 많다.

핵심은 K_d와 D_d를 태스크에 맞게 조정하는 것이다:
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
# 벽을 부수지 않고, 적절한 접촉력으로 밀고 있다
print(f"최종 위치: {positions[-1]:.4f} m (벽: {wall_position} m)")
print(f"최종 접촉력: {contact_forces[-1]:.2f} N")
# 순수 위치 제어였으면 벽에 10000 N/m * 0.05 m = 500 N을 때렸을 것이다
```

### Admittance Control

임피던스 제어가 "위치 편차 → 힘 출력"이라면, 어드미턴스 제어는 반대다: "힘 입력 → 위치 출력."

```
x_d_new = x_d + (1 / K_d) * F_ext + (1 / D_d) * F_ext_dot
```

좀 더 정확히, 외력 F_ext가 측정되면 이를 가상 임피던스 모델에 넣어서 목표 위치를 수정하고, 그 수정된 목표를 기존 (강성이 높은) 위치 제어기에 전달한다.

산업용 로봇에서 어드미턴스 제어가 많이 쓰이는 이유: 산업용 로봇은 이미 매우 정밀한 위치 제어기가 내장되어 있고, 대부분 외부에서 토크 명령을 직접 줄 수 없다. 그래서 힘 센서(F/T sensor)로 외력을 측정하고, 위치 명령을 수정하는 어드미턴스 방식이 더 실용적이다.

반면 연구용 토크 제어 가능 로봇(Franka Emika Panda 등)에서는 임피던스 제어가 더 자연스럽다.

---

## 6.7 심화: Whole-Body Control

*연구자가 되고 싶다면 여기서부터 읽어라.*

휴머노이드 로봇이나 사족 보행 로봇은 관절이 수십 개이고, 여러 개의 접촉점(발, 손)을 동시에 관리해야 하며, 균형도 유지해야 한다. 이런 시스템에서 "각 관절에 PID를 걸어라"는 것은 사실상 의미가 없다. 전신(whole-body) 레벨에서 통합적으로 제어해야 한다.

### Task-space vs Joint-space

- **Joint-space control**: 관절 각도 q를 직접 제어한다. 간단하지만 태스크 수준의 목표(end-effector 위치, 무게중심 위치)를 달성하려면 역기구학(inverse kinematics)을 먼저 풀어야 한다.

- **Task-space control**: 태스크 좌표(카르테시안 위치, 방향)에서 직접 제어한다. 태스크 목표를 자연스럽게 기술할 수 있다. 관절 공간으로의 매핑은 제어기 내부에서 처리한다.

### Operational Space Control (Khatib, 1987)

Khatib의 Operational Space Framework는 task-space 제어의 기초다. 핵심 아이디어: 태스크 공간에서의 동역학을 직접 유도한다.

조인트 공간 동역학:

```
M(q) * q_ddot + C(q, q_dot) * q_dot + g(q) = tau + J^T * F_ext
```

태스크 공간으로 변환:

```
Lambda(q) * x_ddot + mu(q, q_dot) * x_dot + p(q) = F + F_ext
```

여기서 Lambda = (J * M^(-1) * J^T)^(-1)은 태스크 공간 관성 행렬이다.

태스크 공간에서 원하는 가속도 x_ddot_d를 달성하기 위한 관절 토크:

```
tau = J^T * Lambda * x_ddot_d + C * q_dot + g(q)
```

이 프레임워크 위에 임피던스 제어를 결합하면, 태스크 공간에서 원하는 동적 행동(impedance)을 구현할 수 있다.

### QP 기반 Whole-Body Control

현대적 WBC는 매 제어 주기에 QP(Quadratic Program)를 풀어 여러 태스크를 동시에 처리한다.

기본 구조:

```
minimize    || J_task * q_ddot - x_ddot_d ||^2  (태스크 추종)
subject to  M(q)*q_ddot + h(q,q_dot) = S^T*tau + J_c^T*F_c  (동역학)
            F_c ∈ friction cone                   (접촉력 제약)
            tau_min ≤ tau ≤ tau_max               (토크 제한)
```

여기서:
- J_task: 태스크 자코비안
- J_c: 접촉 자코비안
- F_c: 접촉력
- S: selection matrix (underactuated 자유도 제거)

**다중 태스크 우선순위**: 실제 로봇에서는 여러 태스크가 충돌한다. 예를 들어 "오른손을 목표 위치에 보내라" + "균형을 유지하라" + "관절 한계를 지켜라". 이때 태스크에 우선순위를 부여한다:

1. 최고 우선순위: 접촉 제약 (발이 바닥에 붙어 있어야 한다), 관절 한계
2. 높은 우선순위: 균형 유지 (CoM 제어)
3. 중간 우선순위: end-effector 위치 제어
4. 낮은 우선순위: 자세 유지 (null-space)

이것을 strict hierarchy로 구현하려면 null-space projection을 쓰거나, 각 우선순위 레벨의 QP를 순차적으로 푼다 (hierarchical QP). 또는 soft priority로 가중치를 다르게 두어 하나의 QP로 합칠 수도 있다.

### Contact-Consistent Control

보행 로봇에서 접촉력은 물리적으로 타당해야 한다:

- **단방향 접촉(unilateral contact)**: 발이 바닥을 당길 수 없다. F_z >= 0.
- **마찰 원뿔(friction cone)**: 접선력이 수직력 x 마찰계수보다 작아야 한다. sqrt(F_x^2 + F_y^2) <= mu * F_z.
- **ZMP/CoP 제약**: 압력 중심(Center of Pressure)이 지지 다각형(support polygon) 안에 있어야 넘어지지 않는다.

이 모든 제약을 QP에 넣으면, 물리적으로 실현 가능한 제어 입력을 얻을 수 있다. 마찰 원뿔은 원래 비선형(second-order cone)이지만, 다면체로 근사(linearized friction cone)하면 QP로 풀 수 있다.

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

*연구자가 되고 싶다면 여기서부터 읽어라.*

제어기를 설계했으면, "이 제어기가 시스템을 정말 안정하게 만드는가?"를 증명해야 한다. 시뮬레이션에서 잘 되는 것과 수학적으로 안정성이 보장되는 것은 전혀 다른 문제다. Lyapunov 이론은 이 증명의 핵심 도구다.

### Lyapunov 안정성

비선형 시스템 x_dot = f(x)에서 원점이 평형점이라 하자 (f(0) = 0).

Lyapunov의 직접 방법(direct method): 함수 V(x)가 다음을 만족하면 원점은 안정하다.

1. V(0) = 0
2. V(x) > 0 for all x != 0 (양정치)
3. V_dot(x) = dV/dx * f(x) <= 0 (비증가)

V_dot(x) < 0이면 점근적 안정(asymptotically stable), 즉 시간이 지남에 따라 상태가 원점으로 수렴한다.

물리적 직관: V(x)를 에너지로 생각하면 된다. 에너지가 항상 양수이고, 시간에 따라 감소하면, 시스템은 에너지가 최소인 평형점으로 수렴한다.

어려운 점: V(x)를 찾는 것이다. 일반적인 방법론이 없다. 기계 시스템에서는 역학적 에너지(운동에너지 + 위치에너지)가 자연스러운 Lyapunov 함수 후보이다. 선형 시스템에서는 V(x) = x^T * P * x (P는 ARE의 해)가 Lyapunov 함수가 된다. LQR의 안정성 증명이 여기서 나온다.

### 적응 제어 (Adaptive Control)

모델 파라미터가 정확히 알려져 있지 않을 때 쓴다. 예를 들어 로봇 팔에 실린 페이로드의 무게를 모른다거나, 마찰 계수가 시간에 따라 변한다거나.

기본 아이디어: 제어기 내에 파라미터 추정기(estimator)를 내장하고, 제어와 추정을 동시에 수행한다.

로봇 동역학은 다음과 같이 파라미터에 대해 선형인 형태로 쓸 수 있다:

```
M(q)*q_ddot + C(q,q_dot)*q_dot + g(q) = Y(q, q_dot, q_ddot) * theta
```

여기서 Y는 regressor matrix이고, theta는 동역학 파라미터 벡터(질량, 관성, 마찰 등)이다.

적응 제어 법칙:

```
tau = Y * theta_hat - K_d * s
theta_hat_dot = -Gamma * Y^T * s
```

여기서 s는 sliding variable, theta_hat은 파라미터 추정값, Gamma는 적응 게인 행렬이다.

Lyapunov 함수를 적절히 잡으면 (V = 0.5*s^T*M*s + 0.5*theta_tilde^T*Gamma^(-1)*theta_tilde), V_dot <= 0을 보일 수 있고, 추종 오차가 0으로 수렴함을 증명할 수 있다. 단, theta_hat이 실제 theta로 수렴하는 것은 보장되지 않는다. 수렴하는 것은 추종 오차뿐이다.

### Robust Control

모델 불확실성이 있지만 그 범위(bound)는 아는 경우에 쓴다.

- **H-infinity control**: 최악의 외란에 대한 성능을 최적화한다. "어떤 외란이 들어오든 출력 오차가 이 이하로 유지된다"는 보장을 준다. 수학이 무겁고 (Riccati 부등식, LMI), 보수적인 경향이 있다. 1990년대에 산업 현장에서 널리 적용했다.

- **Sliding Mode Control**: 상태를 슬라이딩 면(sliding surface)으로 유한 시간 내에 끌어온 뒤, 슬라이딩 면 위에서 원하는 동특성을 따르게 한다. 모델 불확실성에 매우 강건하다. 문제는 chattering: 슬라이딩 면 근처에서 고주파 스위칭이 발생하여 액추에이터에 부담을 준다. Boundary layer approach나 higher-order sliding mode로 완화한다.

### 언제 쓰는가, 언제 안 쓰는가

| 상황 | 추천 | 비추천 |
|------|------|--------|
| 모델이 정확하고 선형성 충분 | LQR, MPC | 적응 제어 (과설계) |
| 파라미터 불확실성이 큼 | 적응 제어 | PID만으로 버티기 |
| 불확실성 범위를 알고 있음 | Robust control (H-inf) | 적응 제어 (불필요) |
| 안전 인증이 필요함 | Lyapunov 기반 증명 | "시뮬레이션에서 됐으니까 OK" |
| 빠르게 프로토타입 | PID + feedforward | 처음부터 H-infinity |

솔직히, 논문을 쓰는 것이 아니라면 적응 제어나 sliding mode를 실제 시스템에 쓰는 일은 많지 않다. MPC가 충분히 강력하고 직관적이기 때문이다. 하지만 "왜 이 제어기가 안정한가?"를 설명해야 할 때, Lyapunov 이론은 피할 수 없다. 특히 안전이 중요한 시스템(의료 로봇, 자율주행)에서는 수학적 안정성 증명이 필수다.

---

## 6.9 추천 자료

> **Åström & Murray, "Feedback Systems: An Introduction for Scientists and Engineers"**
> https://fbswiki.org/
> 무료 PDF 제공. 제어 이론 입문서로 가장 적합하다. 수학이 과하지 않으면서 핵심을 정확히 짚는다. PID부터 상태공간, 주파수 응답까지. 학부생이라면 이 책부터 시작하라.

> **Steve Brunton, "Control Bootcamp" (YouTube)**
> https://www.youtube.com/playlist?list=PLMrJAkhIeNNR20Mz-VpzgfQs5zrYi085m
> 상태공간, 가제어성, 가관측성, LQR을 직관적으로 설명한다. 영상 하나가 15분 내외로 짧고 밀도가 높다. 교과서를 읽기 전에 먼저 보면 이해가 훨씬 빠르다.

> **Slotine & Li, "Applied Nonlinear Control"**
> 비선형 제어, Lyapunov 안정성, 적응 제어의 표준 교재. 6.8절의 내용을 본격적으로 공부하려면 이 책이다. 절판이지만 PDF가 돌아다닌다 (알아서 찾아라).

> **Russ Tedrake, "Underactuated Robotics" (MIT OCW)**
> https://underactuated.csail.mit.edu/
> 무료 온라인 교재 + 강의. MPC, trajectory optimization, 그리고 제어와 계획(planning)의 연결을 깊이 있게 본다. Drake 라이브러리의 이론적 배경이기도 하다.

> **python-control library**
> https://python-control.readthedocs.io/
> Python으로 제어 시스템을 분석하고 설계하는 라이브러리. MATLAB Control System Toolbox의 Python 대안. Bode plot, root locus, state-space 분석 등을 지원한다.

> **CasADi**
> https://web.casadi.org/
> Nonlinear MPC 구현의 사실상 표준 도구. 자동 미분(automatic differentiation)과 다양한 NLP solver (IPOPT, SNOPT)를 지원한다. Python, MATLAB, C++ 인터페이스 제공.

> **OSQP (Operator Splitting Quadratic Program)**
> https://osqp.org/
> Linear MPC용 QP solver. 빠르고, robust하고, 코드 생성(code generation)이 가능하여 임베디드 시스템에 배포할 수 있다. C 구현 기반으로 Python, MATLAB, Julia 등 다양한 바인딩을 제공한다.

> **주요 논문**
> - [Hogan, "Impedance Control: An Approach to Manipulation" (ASME JDSMC 1985)](https://doi.org/10.1115/1.3140702) — 임피던스 제어의 원논문. 위치 제어와 힘 제어를 통합하는 프레임워크 제시
> - [Khatib, "A Unified Approach for Motion and Force Control of Robot Manipulators: The Operational Space Formulation" (IEEE RA 1987)](https://doi.org/10.1109/JRA.1987.1087068) — Operational Space Control의 원논문. Task-space 동역학 유도와 제어의 기초
> - [Khazoom et al., "Tailoring Solution Accuracy for Fast Whole-Body MPC" (RA-L 2024, arXiv:2407.10789)](https://arxiv.org/abs/2407.10789) — 실시간 whole-body MPC의 최신 접근

---

## 기술 흐름

```
1922 ── PID 제어 개념 정립 (Minorsky)
1960 ── 상태공간 이론 (Kalman)
1960 ── LQR (Kalman)
1985 ── Impedance Control 개념 (Hogan)
1987 ── Operational Space Control (Khatib)
1990s ─ Robust control (H-infinity) 산업 적용
2004 ── 실시간 MPC 실용화 시작
2019 ── Boston Dynamics Atlas: MPC + WBC
2023 ── Unitree H1/G1: 학습 기반 + MPC 하이브리드
2024 ── Figure 01: LLM + MPC + manipulation
```

---

이 장은 제어 이론 전체 중 로보틱스에서 실제로 쓰이는 핵심을 골랐다. 각 기법의 수학적 세부사항은 추천 자료로 보충하기 바란다. 한 가지 조언하자면, 제어 이론은 시뮬레이션 없이 이해하기 어렵다. 이 장의 코드를 직접 실행하고, 파라미터를 바꿔가며 시스템 응답이 어떻게 달라지는지 관찰하는 것이 가장 효과적인 학습 방법이다. 교과서를 세 번 읽는 것보다 시뮬레이션을 한 번 돌리는 것이 낫다.
