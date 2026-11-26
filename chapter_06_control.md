# 6장 제어 이론

5장에서는 로봇에 토크를 가했을 때 관절과 몸체가 어떻게 가속하는지, 그리고 원하는 움직임을 내기 위해 역으로 어떤 토크가 필요한지 계산하는 법을 배웠다. 하지만 현실의 로봇은 교과서 수식처럼 깔끔하게 움직이지 않는다. 모델에 빠져 있는 관절 마찰, 손에 쥔 물체의 무게 변화, 예상치 못한 외란, 그리고 센서 노이즈가 매 순간 끼어들기 때문이다. 동역학 모델을 한 번 계산해 토크를 그대로 흘려보내는 개루프(open-loop) 방식만으로는 로봇이 목표 궤적을 안정적으로 지킬 수 없다. 센서로 현재 상태를 읽어 목표와의 오차를 줄여 나가는 피드백 루프, 즉 제어(control)가 필요한 까닭이다. 이 장에서는 산업 현장의 기본기인 PID 제어부터 다변수 시스템을 다루는 상태공간 표현과 LQR, 물리적 제약을 직접 품는 MPC, 접촉을 부드럽게 다루는 임피던스·어드미턴스 제어, 전신 제어(WBC), 그리고 폐루프의 안전을 수학적으로 보증하는 Lyapunov 안정성과 적응·강인 제어까지 실전 코드와 함께 단계적으로 살펴본다.

---

## 6.1 제어가 쓰이는 곳

인식(Perception) 모듈이 카메라와 IMU 같은 센서로 환경과 로봇의 현재 상태를 추정한다면, 제어(Control) 모듈은 그 추정을 바탕으로 목표 상태에 안전하고 정확하게 도달하도록 액추에이터 입력을 계산한다. 제어 입력을 잘못 계산하면 로봇이 목표 주변에서 격렬하게 떨거나(진동), 발산해 주변 장비와 충돌할 수도 있다. 실제 로봇 시스템에서 제어기가 떠맡는 핵심 임무는 크게 세 가지다.

- **정확한 위치 추종**: 피드백 없이 큰 전류를 흘려보내기만 하면 관절이 관성 때문에 목표 각도를 지나쳐 한참 진동하거나 엉뚱한 곳에 멈춘다. 제어기는 센서로 잰 현재 위치와 목표 사이의 오차를 매 주기 확인하며 부드럽게 수렴하도록 입력을 조절한다.
- **외란 대응**: 현장에서는 미끄러운 바닥, 갑작스러운 바람, 예상보다 무거운 물체(payload)처럼 동역학 모델에 미처 넣지 못한 변화가 끊임없이 일어난다. 센서와 액추에이터 사이의 피드백 루프를 닫아 두면 이런 외란이 들어와도 오차를 감지해 즉각 보상한다.
- **안전한 접촉**: 사람과 같은 공간에서 협업하거나 깨지기 쉬운 물건을 다루는 로봇은 위치만 정확히 맞출 게 아니라 환경에 가하는 접촉력까지 함께 제한해야 한다.

이처럼 다양한 요구를 수식으로 정식화하고 안정성을 따지다 보면 행렬 연산, 고유값(eigenvalue), 미분방정식이 제어 이론의 핵심 언어로 자연스럽게 등장한다. 이제 가장 단순하면서도 현장에서 여전히 가장 널리 쓰이는 PID 제어부터 출발해 보자.

---

## 6.2 PID 제어

PID(Proportional-Integral-Derivative) 제어는 1922년 Minorsky가 선박의 자동 조타 장치를 분석하며 수학적으로 정식화한 이래, 한 세기가 넘도록 산업 현장을 지배해 온 고전 제어 기법이다. 수식이 직관적이고 각 항이 맡는 역할이 뚜렷해, 제어 이론에 처음 발을 디딜 때 가장 먼저 익히게 된다.

### 기본 구조

현재 시각 t에서 목표값(reference) `r(t)`와 센서가 측정한 출력 `y(t)`의 차이, 즉 오차를 `e(t) = r(t) - y(t)`라고 두자. PID 제어기는 이 오차의 현재 값, 과거 누적값, 미래 변화 추세를 모두 합산해 제어 입력 `u(t)`를 만든다.

```
u(t) = Kp * e(t) + Ki * integral(e(τ)dτ, 0, t) + Kd * de(t)/dt
```

세 항이 각기 어떤 역할을 맡고, 게인을 올리거나 내릴 때 실제 시스템이 어떻게 반응하는지 살펴보자.

- **P (Proportional, 비례항)**: 현재 오차에 비례해 곧바로 힘을 낸다. 비례 게인 `Kp`를 키우면 목표를 향해 치고 나가는 반응이 빨라지지만, 지나치게 키우면 목표를 지나치는 오버슈트(overshoot)와 진동이 커진다. 또한 적분기(1/s)가 없는 플랜트에 계단 입력을 주면 P 항만으로는 정상상태 오차(steady-state error)가 남는다. 목표 근처로 다가갈수록 오차가 줄면서 제어 입력도 함께 0에 가까워지기 때문이다. 플랜트 자체에 자유 적분기(예: 마찰 없는 순수 관성체에서 힘 → 속도 → 위치)가 들어 있으면 이론상 계단 입력의 정상 오차가 0이 되기도 하지만, 중력처럼 계속 끌어당기는 일정한 외란이 작용하면 그 외란을 버티는 데 필요한 정상 오차는 여전히 남는다.
- **I (Integral, 적분항)**: 과거부터 쌓여 온 오차에 비례해 입력을 더한다. 아주 작은 오차라도 오래 지속되면 적분값이 불어나 제어 입력을 키워 주므로, 정상상태 오차를 완전히 없애는 데 필수적이다. 특히 중력이나 정지 마찰 같은 상수 외란이 있는 로봇 팔에서 목표 각도를 끝까지 밀어 올리는 역할을 맡는다. 다만 모터가 최대 출력에 걸려 포화(saturation)된 상태에서도 오차가 계속 적분기에 쌓이면, 정작 목표에 도달한 뒤에 잔뜩 불어난 적분값이 남아 큰 오버슈트를 일으킨다. 이를 **와인드업(wind-up)**이라 하며, 실무에서는 출력이 포화될 때 적분을 멈추거나 되돌리는 **안티 와인드업(anti-windup)** 로직을 반드시 함께 넣는다.
- **D (Derivative, 미분항)**: 오차의 시간 변화율에 비례해 제동을 건다. 목표에 빠르게 접근해 오차가 급격히 줄어드는 순간에는 브레이크처럼 반대 방향 성분을 더해 오버슈트를 눌러 준다. 문제는 미분 연산이 신호의 고주파 성분, 즉 센서 노이즈를 크게 증폭시킨다는 점이다. 실제 엔코더나 힘 센서에는 늘 노이즈가 섞여 있으므로 D 항 앞에는 저역통과 필터(low-pass filter)를 달아 고주파 노이즈를 걸러 내고, 노이즈가 너무 심한 공정에서는 아예 D 항을 빼고 PI 제어만 쓰기도 한다.

### Python 구현

교과서 수식을 그대로 코드로 옮기면 이산 시간 적분 오차, 미분 노이즈 폭주, 적분기 와인드업 문제에 곧바로 부딪힌다. 아래 `PIDController` 클래스는 사다리꼴 적분(trapezoidal integration), D 항 저역통과 필터, 출력 클램핑(clamping) 기반 anti-windup을 모두 갖춘 실무형 구현이다. 이어서 1kHz 주기로 1자유도 질량체를 목표 위치 `1.0`으로 보내는 예제도 함께 담았다.

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

PID 제어기는 구조는 단순하지만 세 게인(`Kp`, `Ki`, `Kd`)을 어떻게 잡느냐에 따라 성능이 천차만별이다. 현장에서 주로 쓰는 튜닝 절차는 크게 두 가지다.

먼저 고전적인 **Ziegler-Nichols 방법**에서는 `Ki = 0`, `Kd = 0`으로 묶어 둔 채 `Kp`를 서서히 끌어올린다. 시스템이 감쇠 없이 일정하게 진동하기 시작하면 그때의 임계 이득 `Ku`와 진동 주기 `Tu`를 측정한 뒤, 이 두 값을 게인 공식에 대입한다.

```
PID:  Kp = 0.6 * Ku,  Ki = 2 * Kp / Tu,  Kd = Kp * Tu / 8
PI:   Kp = 0.45 * Ku,  Ki = 1.2 * Kp / Tu
P:    Kp = 0.5 * Ku
```

다만 Ziegler-Nichols 공식은 외란 억제에 초점을 맞춘 공격적인 세팅이라 계단 응답에서 오버슈트가 꽤 크게 나오는 편이다. 따라서 이 수치는 어디까지나 출발점으로 삼고, 실제 응답 파형을 보며 다시 다듬어야 한다. 게다가 실제 로봇 팔을 일부러 한계 진동(sustained oscillation)까지 몰아넣는 실험은 장비 파손 위험이 커서, 실무에서는 다음의 **경험적 튜닝** 순서를 더 자주 밟는다.

1. 안전하게 I와 D 게인을 0으로 둔 상태에서 시작한다.
2. P 게인을 조금씩 높여 시스템이 적당히 빠르게 반응하되 진동이 번지지 않는 지점을 찾는다.
3. 목표 위치 근처에서 정상상태 오차가 남으면 I 게인을 서서히 추가하되, 반드시 anti-windup 동작을 함께 확인한다.
4. 목표를 지나치는 오버슈트가 크다면 D 게인을 조금씩 더하면서 센서 노이즈로 인한 떨림이 생기지 않도록 저역통과 필터 계수를 같이 맞춘다.

동역학 모델이 있다면 처음부터 실기에 게인을 올리지 말고, 시뮬레이션에서 안전한 범위의 초기 게인을 먼저 찾은 뒤 실제 로봇에 적용해 미세 조정하자.

### PID의 한계

이처럼 손쉽고 강력한 PID에도 로봇 공학자가 더 고차원 제어기로 눈을 돌리게 만드는 분명한 한계가 있다.

- **SISO(Single-Input Single-Output) 전용 구조**: 단일 입력-단일 출력 구조이므로, 6축 로봇 팔처럼 관절들이 동역학적으로 얽혀 있는 시스템에서 관절마다 독립적인 PID를 걸면 성능이 떨어진다. 1번 관절이 빠르게 움직일 때 발생하는 원심력과 코리올리 힘이 2번 관절에는 그대로 외란처럼 작용하기 때문이다.
- **선형성 가정**: PID는 선형 제어기인 반면 실제 로봇 동역학은 자세에 따라 관성이 변하는 비선형 시스템이다. 한 작동점(operating point) 근처에서 게인을 완벽하게 튜닝했더라도, 팔을 쭉 뻗거나 무거운 물체를 집으면 응답이 달라진다.
- **제약 조건 처리 불가**: 토크 한계, 관절 가동 범위, 속도 제한 같은 물리적 제약을 제어 법칙 내부에 반영할 방법이 없어, 외부에서 사후적으로 출력을 자르거나(saturation) anti-windup으로 땜질해야 한다.
- **예측 없는 반응형 제어**: 오직 현재 오차, 과거 누적 오차, 순간 변화율에만 반응할 뿐 앞으로 다가올 궤적 변화나 동역학을 미리 내다보지 못한다. 빠른 궤적을 정밀하게 추종하려면 역동역학 기반의 피드포워드(feedforward) 항을 따로 얹어 주어야 한다.

물론 관절 간 결합이 약하거나 감속비가 매우 높아 동역학 커플링이 억제되는 산업용 매니퓰레이터에서는 지금도 관절 서보의 기본기로 PID 계열이 굳건히 쓰인다. 하지만 고속·다축 제어, 혹은 도립진자나 보행 로봇처럼 여러 상태 변수가 강하게 맞물려 돌아가는 시스템으로 넘어가려면, 입출력 관계만 들여다보는 관점에서 벗어나 시스템 내부 상태 전체를 한꺼번에 모델링하는 상태공간 표현이 필요하다.

---

## 6.3 상태공간 표현

고전 PID 제어가 "목표값과 출력 사이의 오차"라는 바깥 창문만 보고 입력을 결정했다면, 현대 제어 이론의 출발점인 **상태공간(state-space) 표현**은 시스템 내부에서 벌어지는 모든 핵심 변수의 움직임을 하나의 벡터로 묶어 직접 다룬다.

### 기본 형태

연속시간 선형 시불변(LTI) 시스템의 상태공간 방정식은 두 개의 행렬 식으로 깔끔하게 적힌다.

```
x_dot(t) = A * x(t) + B * u(t)    (상태 방정식)
y(t)     = C * x(t) + D * u(t)    (출력 방정식)
```

- `x(t)`: 상태 벡터 (`n x 1`). 시스템의 미래 거동을 결정하는 데 필요한 최소한의 내부 변수 집합이다.
- `u(t)`: 입력 벡터 (`m x 1`). 모터 토크나 전압 같은 제어 입력이다.
- `y(t)`: 출력 벡터 (`p x 1`). 엔코더나 IMU 등 센서로 실제로 측정하는 값이다.
- `A`: 시스템 행렬 (`n x n`). 외부 입력이 없을 때 상태가 스스로 진화하는 동특성을 결정한다.
- `B`: 입력 행렬 (`n x m`). 제어 입력이 각 상태 변화율에 미치는 영향을 나타낸다.
- `C`: 출력 행렬 (`p x n`). 내부 상태를 측정 가능한 출력으로 사상한다.
- `D`: 직접 전달 행렬 (`p x m`). 입력이 출력으로 지연 없이 곧장 튀어나오는 성분을 뜻하며, 관성을 가진 대부분의 물리 시스템에서는 0이다.

예를 들어 익숙한 질량-스프링-댐퍼 시스템(`m * x_ddot + c * x_dot + k * x = F`)에서 상태를 `x1 = 위치`, `x2 = 속도`로 잡으면 상태공간 행렬은 다음과 같이 적힌다. 여기서 `m`은 질량(앞선 입력 차원 `m`과 기호가 같으니 문맥으로 구분한다), `c`는 감쇠 계수, `k`는 스프링 상수, `F`는 외력이다.

```
A = [[0, 1], [-k/m, -c/m]]
B = [[0], [1/m]]
C = [[1, 0]]   (위치만 측정)
D = [[0]]
```

### 전달함수와의 관계

상태공간 방정식을 라플라스 변환하면 고전 제어에서 익숙한 전달함수 `G(s) = C * (sI - A)^(-1) * B + D`로 바로 연결된다. 단일 입출력(SISO) 시스템의 주파수 응답이나 여유(margin)를 따질 때는 전달함수가 편리하지만, 입력과 출력이 여러 개이고 내부 변수들이 서로 얽혀 있는 다입출력(MIMO) 로봇 제어에서는 결합 구조가 행렬 원소에 그대로 드러나는 상태공간 표현이 훨씬 자연스럽다.

### 가제어성 (Controllability)

제어기를 설계하기 전에 가장 먼저 던져야 할 질문은 "우리에게 달린 모터만으로 시스템의 모든 상태를 원하는 대로 움직일 수 있는가?"다. 어떤 초기 상태에서 출발하든 유한한 시간 안에 원하는 최종 상태로 옮길 수 있는 시스템은 **가제어(controllable)**하다. 가제어 여부는 아래 가제어성 행렬의 계수(rank)로 판정한다.

```
C_ctrl = [B, A*B, A^2*B, ..., A^(n-1)*B]
```

n차원 시스템에서 `C_ctrl`의 rank가 n이면 완전 가제어다. 만약 rank가 n보다 작다면 아무리 큰 제어 입력을 넣어도 결코 도달하거나 바꿀 수 없는 상태 부분공간이 존재한다는 신호다. 다만 무한 시간 LQR처럼 시스템을 안정화하기만 하면 되는 문제에서는 완전한 가제어성 대신 **안정화 가능성(stabilizability)**만 만족해도 해가 존재한다. 안정화 가능성이란 제어할 수 없는 모드들이 스스로 감쇠해 안정한 조건을 뜻한다.

### 가관측성 (Observability)

반대편 질문도 똑같이 중요하다. 엔코더로 위치만 재고 속도는 재지 못하거나, 카트 위치만 재고 진자 각도는 재지 못할 때처럼 모든 상태 변수를 직접 측정할 수 없는 경우가 흔하다. 이때 측정된 출력 `y(t)`의 기록만으로 초기 상태 `x(0)`를 유일하게 역추적할 수 있다면 시스템은 **가관측(observable)**하다.

```
O = [C; C*A; C*A^2; ...; C*A^(n-1)]
```

마찬가지로 가관측성 행렬 `O`의 rank가 n이면 시스템은 가관측이다. 이 조건이 충족되어야 Luenberger observer나 Kalman filter 같은 상태 추정기를 붙였을 때 추정값이 실제 상태로 수렴하도록 보장할 수 있다.

### 왜 PID에서 상태공간으로 넘어가야 하는가

2자유도 로봇 팔만 해도 한 관절을 빠르게 돌리면 다른 관절에 원심력과 코리올리 힘이 실린다. 관절마다 독립적인 PID를 붙여 놓으면 이 커플링 힘을 그저 알 수 없는 외란으로 취급해 I 항이 뒤늦게 쫓아가며 보상하므로 응답이 둔해지거나 흔들린다. 반면 상태공간 표현에서는 관절 위치·속도를 포함한 시스템 전체를 하나의 모델로 적고, 모든 상태 변수를 동시에 참고해 입력 벡터 `u`를 한꺼번에 계산한다. 뒤이어 배울 LQR과 MPC가 바로 이 틀 위에서 동작한다.

아래 코드는 제어 공학의 대표 벤치마크인 카트-도립진자(inverted pendulum)를 수직 평형점(`theta ≈ 0`) 주변에서 선형화하고, `python-control` 라이브러리로 가제어성·가관측성·고유값을 점검하는 예제다.

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

출력 결과를 보면 `ctrb`와 `obsv`의 rank는 모두 4여서 가제어성·가관측성 조건을 충족하지만, 시스템 행렬 `A`의 고유값 중 하나는 실수부가 양수로 나온다. 손을 놓으면 진자가 곧바로 쓰러지는 불안정한 개루프 시스템이기 때문이다. 그렇다면 카트를 밀어 이 불안정한 시스템을 세우면서도 카트 위치와 입력 힘의 균형을 가장 잘 맞추려면, 상태 피드백 게인 `K`를 어떻게 체계적으로 구할까? 이 질문에 답하는 기법이 바로 LQR이다.

---

## 6.4 LQR (Linear Quadratic Regulator)

상태 피드백 `u(t) = -K * x(t)`에서 폐루프 행렬 `(A - BK)`의 극점(고유값)을 손으로 일일이 지정해 게인 `K`를 구하는 방식(pole placement)도 있다. 하지만 상태 변수가 4개만 넘어도 어디에 극점을 놓아야 힘도 적게 쓰고 응답도 빠를지 감으로 잡기가 막막해진다. **LQR(Linear Quadratic Regulator)**은 "상태 오차"와 "제어 입력 에너지"를 함께 평가하는 이차 비용 함수(quadratic cost function)를 정의하고, 이를 최소화하는 최적 피드백 게인 `K`를 Riccati 방정식 한 방으로 자동 계산해 주는 방법이다.

### 비용 함수

무한 시간 구간에서 LQR이 최소화하려는 비용 함수 `J`는 다음과 같다.

```
J = integral_0^inf (x(t)^T * Q * x(t) + u(t)^T * R * u(t)) dt
```

- `Q` (`n×n`, 양의 반정치): 상태 `x(t)`가 목표(원점 0)에서 벗어나 있을 때 매기는 페널티 가중치다.
- `R` (`m×m`, 양정치): 액추에이터가 제어 입력 `u(t)`를 세게 쓸 때 매기는 페널티 가중치다.

직관적으로 보면 명쾌하다. `Q`의 원소를 키우면 제어기는 오차를 용납하지 않아 상태가 0으로 빠르게 수렴하지만, 그 대가로 큰 토크나 힘을 쏟아붓는다. 반대로 `R`을 키우면 모터를 아껴 쓰면서 부드럽게 움직이는 대신 수렴이 느려진다. 엔지니어는 극점 좌표를 추측하는 대신 `Q`와 `R`의 상대적 비율로 "정밀함"과 "에너지 절약" 사이의 균형점을 조절한다.

### Q, R 행렬 튜닝

처음 `Q`와 `R`을 잡을 때 막연하다면, 각 상태와 입력의 단위를 정규화해 주는 **Bryson's rule**이 좋은 출발점이 된다. `Q`와 `R`을 대각 행렬로 두고, 각 대각 원소를 해당 변수에 허용 가능한 최대 편차를 제곱한 값의 역수로 잡는 방식이다.

```
Q_ii = 1 / (허용 가능한 x_i의 최대값)^2
R_jj = 1 / (허용 가능한 u_j의 최대값)^2
```

예를 들어 앞선 도립진자에서 카트 위치는 0.5m 이내, 진자 각도는 0.1rad 이내로 묶어 두고 싶고 카트에 가하는 힘은 20N을 넘지 않길 바란다면, 가중치 행렬은 다음과 같다.

```
Q = diag(1/0.5^2, 0, 1/0.1^2, 0) = diag(4, 0, 100, 0)
R = [1/20^2] = [0.0025]
```

이렇게 물리 단위가 맞물린 값에서 출발한 뒤, 시뮬레이션 파형을 관찰하며 부족한 항의 가중치를 조정해 나가면 된다.

### Algebraic Riccati Equation (ARE)

비용 함수 `J`를 최소화하는 최적 상태 피드백 게인 `K`는 다음의 **Algebraic Riccati Equation(ARE)**에서 대칭 양정치 해 `P`를 구하면 곧바로 얻어진다.

```
A^T * P + P * A - P * B * R^(-1) * B^T * P + Q = 0
```

이 행렬 `P`를 이용하면 최적 게인과 제어 법칙이 다음과 같이 결정된다.

최적 상태 피드백 게인: `K = R^(-1) * B^T * P`

제어 법칙: `u(t) = -K * x(t)`

이론적으로 `(A, B)`가 안정화 가능하고 `(Q^{1/2}, A)`가 검출 가능(detectable)하며 `R`이 양정치이면, ARE는 유일한 해를 갖고 폐루프 행렬 `(A - BK)`의 모든 고유값은 실수부가 음수가 된다. 즉 수학적으로 점근 안정성(asymptotic stability)이 보장된다.

### Python 구현

`scipy.linalg.solve_continuous_are`를 쓰면 몇 줄의 코드만으로 ARE를 풀어 도립진자를 세우는 LQR 제어기를 완성할 수 있다. 아래 예제에서는 앞서 선형화한 도립진자 모델에 `Q`, `R`을 넣고 게인 `K`를 계산한 뒤, 폐루프 극점이 모두 좌반평면(음의 실수부)으로 옮겨 갔는지 확인한다. 이어서 진자가 10도 기울어진 초기 상태에서 출발하는 시뮬레이션을 돌린다.

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

LQR은 다변수 선형 시스템에서 놀라울 만큼 깔끔한 최적 해를 주지만, 실전 로봇에 바로 적용하려면 네 가지 한계를 염두에 두어야 한다.

- **선형 모델 의존**: 비선형 시스템을 특정 작동점 주변에서 선형화한 모델 위에서 동작하므로, 진자가 크게 기울거나 팔 자세가 크게 바뀌어 작동점에서 멀어지면 성능이 급격히 떨어진다.
- **제약 조건 미지원**: 모터 토크 제한, 관절 한계, 장애물 회피 같은 부등식 제약은 표준 LQR 수식 안에 넣을 자리가 없다. 게인으로 계산한 입력이 모터 한계를 넘어 포화되는 순간 LQR의 최적성과 안정성 보장은 깨진다.
- **전체 상태 필요**: `u = -Kx`를 계산하려면 상태 벡터 `x`의 모든 성분을 매 주기 알아야 하므로, 측정되지 않는 상태는 Kalman filter 같은 추정기(observer)로 복원해야 한다(이를 결합한 것이 LQG다).
- **원점 안정화 중심**: 기본 LQR은 상태를 0으로 보내는 regulator 문제다. 시간에 따라 움직이는 목표 궤적을 따라가려면 오차 좌표계로 변환하거나, 그 궤적 주변에서 선형화해 tracking LQR로 확장해야 한다.

특히 두 번째 한계인 "물리적 제약 조건을 제어기 수식 안에 직접 넣을 수 없다"는 점은 로봇 현장에서 매우 치명적이다. 보행 로봇이 발을 내딛기 전에 ZMP 제약을 지키거나, 매니퓰레이터가 토크 한계 안에서 움직이려면 제약을 1급 시민으로 다루는 최적화 기반 제어가 필요하다. 이 지점에서 MPC가 등장한다.

---

## 6.5 MPC (Model Predictive Control)

**MPC(Model Predictive Control)**는 제어 주기마다 현재 상태를 출발점으로 삼아 미래의 유한 구간(finite horizon) 동안 최적의 입력 시퀀스를 실시간으로 계산하는 제어 기법이다. LQR이 무한 구간의 닫힌 해를 오프라인에서 미리 구해 두는 반면, MPC는 온라인에서 최적화를 직접 풀기 때문에 토크 제한이나 장애물 회피 같은 부등식 제약을 문제 정의 안에 곧바로 집어넣을 수 있다. 이 유연성 덕분에 보행 로봇, 드론, 자율주행 제어의 핵심 엔진으로 자리 잡았다.

### 기본 개념

MPC 제어기는 스텝 `k`마다 다음 다섯 단계를 쉼 없이 반복한다.

1. 센서 측정치나 상태 추정기(필터)로 현재 상태 `x(k)`를 파악한다.
2. 동역학 모델을 굴려 `N` 스텝 앞까지 로봇이 어떻게 움직일지 예측한다.
3. 상태·입력 제약 조건을 모두 만족하면서 비용 함수를 최소화하는 미래 입력 시퀀스 `{u(k), u(k+1), ..., u(k+N-1)}`을 계산한다.
4. 구한 입력 시퀀스에서 **오직 첫 번째 입력 `u(k)`만** 실제 로봇에 넣고 나머지 미래 계획은 버린다.
5. 다음 time step `k+1`이 되면 새로 측정한 상태에서 1단계부터 다시 시작한다.

창문을 한 칸씩 앞으로 밀며 매번 계획을 새로 짠다고 해서 이를 **receding horizon(후퇴 구간)** 전략이라 부른다. 미래 계획을 한 번 짜서 끝까지 밀고 나가지 않고 매 스텝 현재 상태를 다시 측정해 최적화를 새로 풀기 때문에, 모델 오차나 갑작스러운 외란이 생겨도 그 영향이 다음 주기의 입력 계산에 즉시 반영되어 피드백 효과를 얻는다.

### 로보틱스에서 MPC가 유용한 이유

현장의 로봇 제어에서 MPC가 각광받는 까닭은 크게 네 가지로 요약된다.

- **제약 조건의 직접 통합**: 모터 토크 제한, 관절 각도와 속도 제한, 마찰 원뿔, 충돌 회피 영역을 최적화 문제의 제약 조건으로 그대로 적을 수 있다. 기본 PID나 LQR에는 이런 제약을 넣을 자리가 없어 출력 포화, reference governor, constrained-LQR 같은 우회 기법을 덧붙여야 하지만, MPC는 한 번에 품는다.
- **비선형 모델 수용**: Nonlinear MPC(NMPC)로 확장하면 선형화 근사 없이 비선형 동역학 모델을 그대로 예측에 사용할 수 있다.
- **미래를 내다보는 선행 제어**: 현재 오차만 보고 뒤늦게 반응하는 대신, 앞으로 다가올 목표 변화와 동역학을 미리 계산해 선제적으로 움직인다. 보행 로봇이 다음 발을 떼기 전에 미리 무게 중심을 지지발 쪽으로 옮기는 동작도 이 예측 능력 덕분이다.
- **다목적 절충**: 목표 궤적 추종 오차와 에너지 소모를 하나의 비용 함수에 가중합으로 엮고, 안전과 하드웨어 한계는 절대 어기면 안 되는 제약(hard constraint)으로 분리해 체계적으로 절충할 수 있다.

### Linear MPC vs Nonlinear MPC

내부 예측 모델과 비용·제약의 형태에 따라 MPC는 두 갈래로 나뉜다.

- **Linear MPC**: 선형 이산 모델(`x(k+1) = A*x(k) + B*u(k)`)과 이차(quadratic) 비용 함수, 선형 부등식 제약을 결합하면 매 스텝 푸는 문제가 볼록 이차 계획법(convex **QP**)이 된다. 해가 존재하는(feasible) convex QP에서는 지역 최적에 빠질 걱정 없이 전역 최적해를 구할 수 있으며, 실제 연산 시간은 변수 크기, 행렬 희소성(sparsity) 구조, 솔버와 실행 하드웨어 사양에 달려 있다.
- **Nonlinear MPC (NMPC)**: 비선형 동역학이나 비선형 장애물 제약을 그대로 쓰므로 문제가 비볼록(non-convex) 비선형 계획법(NLP)으로 바뀐다. 그만큼 풀기가 까다롭고 솔버가 돌려주는 해 역시 지역 최적해(local optimum)다.

따라서 시스템이 선형 모델로 충분히 근사되거나 제어 주기가 아주 빨라야 할 때는 Linear MPC를 쓰고, 공중제비나 급격한 자세 변화처럼 비선형성이 지배적이면서 제어 주기에 약간의 여유가 있을 때는 NMPC를 선택한다.

### 실시간성 문제

MPC의 가장 큰 실무 장벽은 연산 시간이다. 제어 주기 안에 최적화 문제를 끝까지 풀어야 하기 때문이다. 만약 MPC를 1kHz의 빠른 내부 루프(inner loop)와 같은 주기로 동기 실행한다면 상태 취득부터 QP 풀이까지 모든 과정을 1ms 안에 끝내야 한다. 이 때문에 실제 로봇에서는 계층 구조를 자주 쓴다. MPC를 50~200Hz의 조금 느린 상위 루프(outer loop)로 돌려 목표 궤적이나 기준 접촉력을 만들어 주고, 그 아래에 1kHz로 도는 빠른 하위 추종 제어기(tracking controller)를 붙인다.

실시간 MPC 구현에 널리 쓰이는 오픈소스·상용 솔버는 다음과 같다.

주요 QP solver (Linear MPC 및 SQP 내부 부분 문제용):
- **OSQP** (https://osqp.org/): operator splitting(ADMM) 기반으로 행렬이 큰 sparse QP를 빠르게 푸는 데 널리 쓰인다.
- **qpOASES**: active-set 방식 솔버로 warm start를 지원해, 직전 스텝과 비슷한 QP를 연속해서 풀 때 특히 빠르다.
- **ECOS / Clarabel**: 선형·이차 제약뿐만 아니라 마찰 원뿔 같은 second-order cone programming(SOCP) 문제까지 직접 처리한다.

NMPC용 도구:
- **CasADi + IPOPT**: 연구·프로토타이핑에서 표준으로 쓰는 조합이다. 자동 미분(AD)으로 야코비안·헤시안을 구해 범용 interior-point NLP 솔버에 넘긴다.
- **acados** (https://docs.acados.org/): CasADi로 수식을 정의하면 임베디드·실시간 실행에 최적화된 C 코드를 생성해 준다.
- **FORCESPRO, SNOPT**: 고성능 상용 솔버로 산업·자율주행 양산 환경에서도 쓰인다.

참고로 동기식 루프에서 한 번 푸는 데 5ms가 걸린다면 단순 계산상 갱신 주기의 상한은 200Hz다. 실제 현장에서는 센서 전처리, 통신 지연, 운영체제 jitter까지 시간 예산에 포함해야 하며, 비동기 실행(asynchronous solve)을 쓰는지 혹은 별도 inner loop를 두는지에 따라 전체 타이밍 설계가 달라진다.

### Linear MPC Python 예시

Linear MPC가 코드 수준에서 어떻게 거대한 QP 행렬로 조립되는지 살펴보자. 결정 변수 `z`에 예측 구간의 모든 상태 `[x(0), ..., x(N)]`와 입력 `[u(0), ..., u(N-1)]`을 나란히 세우면, 동역학 식 `x(k+1) = Ax(k) + Bu(k)`는 희소 등식 제약(`A_eq * z = l_eq`)이 되고 상태·입력 한계는 단순 범위 제약(`A_ineq`)이 된다. 아래 예제 코드에서는 1축 더블 인티그레이터(위치·속도 2차원 상태, 가속도 1차원 입력)를 대상으로 이 QP를 조립하고 `osqp`로 풀어 첫 최적 입력과 예측 궤적을 뽑는다.

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

- **Boston Dynamics Atlas**: Boston Dynamics가 공개한 기술 블로그 "Picking Up Momentum"(2023년 1월, https://bostondynamics.com/blog/picking-up-momentum/)을 보면 Atlas 제어 아키텍처의 중심에도 MPC가 놓여 있다. 무거운 공구 가방을 집어 들고 발판을 뛰어올라 던지는 고난도 작업에서는 전신 관절의 운동과 각 링크의 운동량, 그리고 로봇과 물체 사이에 작용하는 힘까지 MPC 모델 안에 통합해 실시간으로 궤적을 생성했다.

지금까지 살펴본 PID, LQR, MPC는 기본적으로 위치나 속도 같은 상태를 목표값으로 끌어오는 데 초점을 맞춘다. 그런데 로봇이 허공이 아니라 단단한 책상이나 사람, 깨지기 쉬운 물건과 직접 맞닿아 힘을 주고받기 시작하면, 위치만 고집하는 제어기는 순식간에 물체를 부수거나 스스로 망가진다. 물리적 접촉을 부드럽게 받아내는 임피던스·어드미턴스 제어를 살펴볼 차례다.

---

## 6.6 임피던스/어드미턴스 제어

로봇이 허공을 이동할 때는 위치를 정확히 추종하는 일만 신경 쓰면 된다. 하지만 로봇 손끝이 단단한 표면에 닿는 순간, 환경과의 물리적 상호작용이 제어기의 성패를 좌우한다.

### 위치 제어 vs 힘 제어 vs 임피던스 제어

접촉이 일어나는 작업을 다룰 때 제어 전략은 크게 세 가지로 갈린다.

- **위치 제어(Position Control)**: 오로지 목표 위치를 따르는 데 집중한다. 허공을 이동하거나 환경이 아주 말랑할 때는 잘 작동하지만, 단단한 물체와 만나면 문제가 터진다. 강성이 높은 벽이나 테이블에서는 작은 위치 오차에도 접촉력이 `(환경 강성) × (위치 오차)`만큼 폭증하기 때문이다. 예컨대 테이블 위 컵을 집을 때 모델상 테이블 높이가 실제보다 겨우 1mm 낮게 잡혀 있어도, 순수 위치 제어기는 그 1mm 아래 목표까지 팔을 우격다짐으로 밀어 넣으려 들면서 거대한 충돌력을 발생시킨다.
- **힘 제어(Force Control)**: 위치 대신 목표 접촉력을 추종한다. 표면을 일정한 힘으로 문지르는 연마(polishing)나 부품을 끼우는 조립 작업에는 힘 제어가 필요하다. 하지만 순수 힘 제어는 물체와 떨어져 있는 비접촉 상태에서 힘 피드백이 0이 되면 무한정 가속하면서 불안정해지기 쉽고, 힘 센서 노이즈에도 예민하게 반응한다.
- **임피던스 제어(Impedance Control)**: 위치와 힘 중 어느 하나만 고집하는 대신, **위치 편차와 힘 사이의 동적 관계**를 제어해 로봇 끝단이 마치 프로그램 가능한 가상 스프링-댐퍼처럼 동작하게 만든다. 허공에서는 목표 위치를 따라가는 위치 제어기처럼 움직이다가, 장애물이나 물체에 부딪히면 딱딱하게 버티는 대신 위치 편차에 비례해 부드럽게 힘을 양보한다.

### 가상 스프링-댐퍼 모델

Hogan(1985)이 정립한 임피던스 제어에서는 로봇 끝단의 움직임이 사용자가 지정한 가상 질량-스프링-댐퍼 관계를 따르도록 설계한다.

```
F = M_d * (x_ddot_d - x_ddot) + D_d * (x_dot_d - x_dot) + K_d * (x_d - x)
```

실제 구현에서는 가속도 피드백과 가상 관성 `M_d`를 맞추기 까다로워 관성 항을 생략한 스프링-댐퍼 형태를 자주 쓴다.

```
F = K_d * (x_d - x) + D_d * (x_dot_d - x_dot)
```

- `K_d`: 가상 강성(virtual stiffness). 이 값을 키우면 위치 추종이 깐깐해지는 대신 부딪혔을 때 발생하는 접촉력도 커진다.
- `D_d`: 가상 감쇠(virtual damping). 속도에 저항하는 점성 항으로 진동을 가라앉힌다.
- `M_d`: 가상 관성(virtual inertia). 로봇 본래의 관성을 임의로 바꾸려면 가속도 측정이나 정확한 역동역학 보상이 필요하므로, 가상 관성 항은 생략하고 로봇 자체 관성을 그대로 두는 편이 많다.

따라서 `K_d`와 `D_d`만 작업 성격에 맞춰 바꾸어 주어도 같은 로봇이 전혀 다른 손놀림을 보여 준다.
- 유리잔을 집을 때: `K_d`를 낮춰 부드럽게 감싸 쥐고, `D_d`를 적절히 높여 흔들림을 잡는다.
- 볼트를 조일 때: `K_d`를 높여 공구가 흔들리지 않고 정밀하게 파고들게 한다.
- 사람과 협업할 때: `K_d`를 아주 낮게 설정해 사람과 스쳐도 다치지 않게 순응(compliance)하도록 만든다.

1자유도 로봇이 `0.15 m` 지점을 목표로 다가가는데, `0.10 m` 위치의 단단한 벽(`k = 10,000 N/m`)이 앞을 가로막고 있다. 아래 시뮬레이션을 보면 이때 임피던스 제어기가 어떻게 과도한 힘을 피하며 벽 앞에서 안정화되는지 알 수 있다.

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

시뮬레이션 결과에서 보듯, 순수 위치 제어였다면 벽을 500 N의 힘으로 부수듯 밀어붙였을 상황에서도 임피던스 제어(`k_d = 500 N/m`)는 벽 표면에서 살짝 안쪽인 `0.1024 m` 지점에 멈춰 서며 접촉력을 약 24 N 수준으로 순하게 억제한다.

### Admittance Control

실무에서 임피던스 제어와 자주 짝지어 불리지만 신호 흐름이 정반대인 기법이 **어드미턴스 제어(Admittance Control)**다. 임피던스 제어기가 "위치 편차"를 입력받아 "힘(또는 관절 토크)" 명령을 내보낸다면, 어드미턴스 제어기는 힘 센서로 "외력"을 측정해 가상 임피던스 모델에 넣은 뒤 수정된 "위치·속도" 명령을 내보낸다.

```
x_d_new = x_d + (1 / K_d) * F_ext      # 강성 항: 힘 → 위치 오프셋
x_dot_d = (1 / D_d) * F_ext             # 감쇠 항: 힘 → 목표 속도 (힘의 미분이 아니다)
```

왜 이 둘을 구분해야 할까? 기성 산업용 로봇 팔 대부분은 내부에 고강성 위치 제어 루프가 꽉 닫혀 있고, 외부 사용자에게는 토크 명령 대신 위치나 속도 명령 인터페이스만 열어 두기 때문이다. 이런 로봇에서는 손목에 F/T(Force/Torque) 센서를 달아 외력 `F_ext`를 재고, 그만큼 목표 위치를 슬쩍 비켜 주어 부드럽게 양보하게 만드는 어드미턴스 제어가 현실적인 해법이다. 반면 Franka Emika Panda처럼 외부에서 관절 토크를 직접 명령할 수 있는 토크 제어 기반 연구용 로봇에서는 힘 센서 없이도(혹은 관절 토크 센서만으로도) 임피던스 제어를 곧바로 구현할 수 있다.

지금까지는 로봇 팔의 끝단 하나를 제어하는 데 집중했다. 하지만 관절이 30개에 달하는 휴머노이드나 사족 보행 로봇이 두 발을 땅에 딛고 균형을 잡으면서 동시에 손으로 문을 연다면 이야기가 훨씬 복잡해진다. 여러 목표와 다중 접촉을 전신 차원에서 동시에 조율하는 Whole-Body Control을 살펴보자.

---

## 6.7 Whole-Body Control [심화]

휴머노이드나 사족 보행 로봇은 관절이 수십 개인 데다 발과 손처럼 지면·물체와 맞닿는 접촉점 여러 개를 동시에 관리하면서 무게중심의 균형까지 지켜 내야 한다. 관절마다 독립적인 PID를 걸어 두는 식으로는 이 복잡한 물리 제약과 균형 조건을 도저히 충족할 수 없다. 그래서 로봇 전체를 하나의 시스템으로 보고 모든 관절 토크를 한꺼번에 계산하는 **전신 제어(Whole-Body Control, WBC)**를 쓴다.

### Task-space vs Joint-space

전신 제어를 이해하려면 먼저 제어를 거는 좌표 공간의 차이부터 짚어야 한다.

- **Joint-space control**: 관절 각도 `q`를 직접 추종한다. 제어 구조가 단순하지만, "손끝을 저 컵 위로 옮긴다"거나 "무게중심을 왼발 위로 보낸다" 같은 작업 목표를 이루려면 먼저 역기구학(IK)을 풀어 관절 궤적으로 바꾸어 놓아야 한다.
- **Task-space control**: 카르테시안 공간의 손끝 위치·자세나 무게중심 위치 같은 작업(task) 좌표에서 직접 목표와 오차를 정의한다. 작업 목표를 직관적으로 적을 수 있고, 이를 실제로 관절 토크로 변환하는 일은 제어기가 내부에서 동역학과 자코비안을 이용해 자동으로 처리한다.

### Operational Space Control (Khatib, 1987)

task-space 제어의 이론적 주춧돌은 Khatib의 Operational Space Framework가 놓았다. 5장에서 유도한 관절 공간 동역학에 끝단 외력 `F_ext`를 더한 식에서 출발해 보자.

```
M(q) * q_ddot + C(q, q_dot) * q_dot + g(q) = tau + J^T * F_ext
```

여기서 `M(q)`, `C(q, q_dot)`, `g(q)`, `tau`는 5장 §5.3에서 다룬 관성 행렬, 코리올리·원심력 행렬, 중력 토크, 관절 토크다(6.3절 상태공간 방정식의 출력 행렬 `C`와는 기호만 같고 전혀 다른 행렬이다). `J`는 끝단 자코비안이고, `F_ext`는 끝단에 가해지는 외력이다.

이 식을 자코비안 `J`를 이용해 태스크 공간 좌표 `x`의 방정식으로 사상하면 놀랍도록 닮은꼴인 태스크 공간 동역학이 나온다.

```
Lambda(q) * x_ddot + mu(q, q_dot) * x_dot + p(q) = F + F_ext
```

여기서 `Lambda = (J * M^(-1) * J^T)^(-1)`는 끝단에서 느껴지는 **태스크 공간 관성 행렬(operational space inertia matrix)**이다. 따라서 태스크 공간에서 원하는 목표 가속도 `x_ddot_d`를 만들어 내려면, 아래처럼 역동역학 형태로 관절 토크를 계산해 주면 된다.

```
tau = J^T * Lambda * (x_ddot_d - J_dot * q_dot) + C * q_dot + g(q)
```

여기에 `x_ddot_d`를 6.6절의 임피던스 법칙으로 정해 넣으면, 로봇 팔 끝단이 태스크 공간에서 정확히 원하는 강성과 감쇠를 띤 가상 스프링-댐퍼처럼 움직인다.

### QP 기반 Whole-Body Control

자유도가 딱 맞고 부등식 제약이 없을 때는 Khatib의 식에서 아름다운 닫힌 해가 나온다. 하지만 보행 로봇처럼 토크 한계도 있고, 발이 미끄러지지 않아야 하는 마찰 제약도 걸려 있으며, 목표 태스크도 여러 개라면 해석적인 식만으로는 풀 수 없다. 그래서 현대의 실전 WBC는 제어 주기(보통 500Hz~1kHz)마다 **QP(Quadratic Program)**를 풀어 이 모든 요구를 한꺼번에 충족하는 최적의 관절 가속도와 접촉력, 토크를 구한다.

```
minimize    || J_task * q_ddot + J_task_dot * q_dot - x_ddot_d ||^2  (태스크 추종)
subject to  M(q)*q_ddot + h(q,q_dot) = S^T*tau + J_c^T*F_c  (동역학)
            F_c ∈ friction cone                   (접촉력 제약)
            tau_min ≤ tau ≤ tau_max               (토크 제한)
```

각 기호가 뜻하는 바는 다음과 같다.
- `J_task`: 추종하려는 작업(손끝 위치, 무게중심 등)의 태스크 자코비안
- `J_c`: 지면이나 환경과 맞닿은 접촉점의 접촉 자코비안
- `F_c`: 각 접촉점에 작용하는 접촉력 벡터
- `S`: 액추에이터가 붙어 있는 관절만 골라내고 부유 베이스의 비구동(underactuated) 자유도를 제거하는 selection matrix

**다중 태스크 우선순위**: 실제 보행 중에는 오른손을 목표 지점으로 뻗는 작업, 몸통 균형 유지, 관절 각도 한계 준수가 한꺼번에 요구되다 보니 태스크끼리 서로 충돌할 때가 잦다. 이럴 때는 로봇의 생존과 직결되는 순서대로 명확하게 계층을 나눈다.

1. 최고 우선순위: 발이 바닥에 미끄러지지 않고 붙어 있도록 하는 접촉 제약, 관절 한계
2. 높은 우선순위: 균형 유지 (CoM 및 상체 자세 제어)
3. 중간 우선순위: 손끝(end-effector) 위치 제어
4. 낮은 우선순위: 팔꿈치나 허리가 자연스러운 기본 자세를 유지하도록 남는 자유도를 쓰는 null-space 제어

이 우선순위를 엄격하게 지키려면(strict hierarchy) 상위 태스크의 영공간(null-space) 안에서만 하위 태스크를 푸는 null-space projection이나 순차적 QP(hierarchical QP)를 쓴다. 반면 구현을 단순화하고 연산 속도를 높이려고 목적 함수 항마다 가중치만 크게 달리 두어 단일 QP로 합쳐 푸는 soft priority 방식도 현장에서 널리 쓰인다.

### Contact-Consistent Control

보행 로봇의 QP 제약 조건 가운데 가장 신경 써야 하는 부분이 접촉력 `F_c`의 물리적 타당성이다. 아무리 수학적 해가 잘 나와도 발바닥이 바닥을 뚫고 당기거나 미끄러지는 힘을 전제로 한 계획이라면 로봇은 그 자리에서 넘어진다.

- **단방향 접촉(unilateral contact)**: 발은 바닥을 밀기만 할 뿐 바닥에 접착제처럼 붙어 끌어당길 수는 없다. 따라서 수직력은 항상 `F_z >= 0`이어야 한다.
- **마찰 원뿔(friction cone)**: 발이 미끄러지지 않으려면 접선 방향 힘이 수직력에 마찰 계수를 곱한 값 이하여야 한다(`sqrt(F_x^2 + F_y^2) <= mu * F_z`).
- **ZMP/CoP 제약**: 발바닥 압력 중심(Center of Pressure)이 발바닥 지지 다각형(support polygon) 안쪽에 머물러야 한다. 압력 중심이 발끝이나 뒤꿈치 경계에 닿는 순간 발이 들리며 넘어지기 시작한다.

이 중 원래 마찰 원뿔 제약 `sqrt(F_x^2 + F_y^2) <= mu * F_z`은 비선형 원뿔(second-order cone)이라 일반 QP 솔버에 그대로 넣을 수 없다. 그래서 실무에서는 원뿔을 4각형이나 8각형 다면체로 선형 근사(linearized friction cone)해 선형 부등식 행렬 `A_cone * F <= 0` 형태로 만들어 QP에 넣는다. 아래 함수는 마찰 계수 `mu`에 대해 3D 마찰 원뿔을 `n_edges`각형으로 근사하는 선형 제약 행렬을 생성한다.

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

이처럼 8개의 접선 부등식과 1개의 수직력 부등식을 합친 `(9, 3)` 행렬 하나면, 어떤 QP 솔버에서도 미끄러짐 없는 안전한 접촉력을 실시간으로 계산해 낼 수 있다.

지금까지 PID, LQR, MPC, 임피던스, WBC까지 다양한 제어기를 조립하는 법을 배웠다. 하지만 실제 로봇의 전원을 켜기 전, 엔지니어가 마지막으로 반드시 확인해야 할 관문이 남아 있다. "시뮬레이션 몇 번 돌려 보니 잘 되더라"는 경험적 안심을 넘어, 이 폐루프 시스템이 정말로 발산하지 않고 안전하게 수렴하는지 수학적으로 어떻게 증명할 수 있을까?

---

## 6.8 Lyapunov 안정성과 적응 제어 [심화]

제어기를 설계했다면 마지막으로 폐루프 시스템이 안정한지 검증해야 한다. 시뮬레이션에서 초기 조건 몇 가지를 넣어 보고 잘 돌아갔다고 해서, 미처 시험하지 못한 다른 초기 상태나 외란에서도 로봇이 넘어지지 않는다는 보장은 없다. 특히 비선형 시스템의 안정성을 엄밀하게 증명할 때 제어 이론이 꺼내 드는 도구가 바로 **Lyapunov 안정성 이론**이다.

### Lyapunov 안정성

비선형 자율 시스템 `x_dot = f(x)`에서 원점이 평형점이라고 하자(`f(0) = 0`). 1892년 리아푸노프가 제안한 직접 방법(direct method)에서는 미분방정식을 직접 풀지 않고도 안정성을 판정할 수 있다. 시스템의 "가상 에너지" 역할을 하는 스칼라 함수 `V(x)` 하나만 잘 찾으면 된다. 함수 `V(x)`가 아래 세 조건을 만족하면 원점은 안정하다(Lyapunov stable).

1. `V(0) = 0`
2. 모든 `x != 0`에 대해 `V(x) > 0` (양정치)
3. `V_dot(x) = dV/dx * f(x) <= 0` (시간에 따라 증가하지 않음)

여기서 한 걸음 더 나아가 `x ≠ 0`인 모든 곳에서 `V_dot(x) < 0`으로 엄격하게 감소한다면 원점은 **점근적 안정(asymptotically stable)**하다. 평형점 근처 어디에서 출발하더라도 상태가 결국 원점으로 수렴한다는 뜻이다.

물리적 직관은 간단하다. `V(x)`를 시스템의 에너지라고 생각해 보자. 에너지가 0 이상인데 마찰 등으로 인해 시간이 흐를수록 에너지가 계속 빠져나가기만 한다면, 시스템은 결국 에너지가 가장 낮은 바닥인 평형점에 멈춰 설 수밖에 없다. 물론 만능 공식은 없어서 시스템마다 엔지니어가 직접 `V(x)` 후보 함수를 찾아내야 한다. 기계 시스템에서는 전체 역학적 에너지(운동에너지 + 위치에너지)를 첫 번째 후보로 놓는 일이 많으며, 앞서 6.4절에서 본 LQR 폐루프의 안정성 역시 ARE의 해 `P`로 만든 이차 함수 `V(x) = x^T * P * x`를 Lyapunov 함수로 잡아 증명한다.

### 적응 제어 (Adaptive Control)

로봇 팔이 무게를 미리 알 수 없는 상자를 집어 올리거나, 관절 감속기의 마찰 계수가 온도와 마모에 따라 서서히 바뀔 때는 고정된 동역학 파라미터만으로는 정밀 제어가 어렵다. **적응 제어(Adaptive Control)**는 제어기 안에 파라미터 추정기(estimator)를 내장해, 로봇을 움직이면서 미지의 파라미터를 온라인으로 동시에 추정하고 보상하는 기법이다.

이처럼 온라인으로 파라미터를 추정하고 보상할 수 있는 것은 로봇의 manipulator equation 덕분이다. 이 방정식은 겉보기에 복잡하고 비선형이지만, 사실 질량·관성 모멘트·마찰 계수 같은 동역학 파라미터 벡터 `theta`에 대해서는 **선형**으로 깔끔하게 분리된다.

```
M(q)*q_ddot + C(q,q_dot)*q_dot + g(q) = Y(q, q_dot, q_ddot) * theta
```

여기서 행렬 `Y`를 **regressor matrix**라고 부르며 관절 각도와 속도·가속도의 비선형 함수들로 채워지고, `theta`는 각 링크의 질량과 관성 성분을 모아 둔 미지 파라미터 벡터다.

이를 이용한 대표적인 적응 제어 법칙(Slotine–Li)은 다음과 같다.

```
tau = Y * theta_hat - K_d * s
theta_hat_dot = -Gamma * Y^T * s
```

식에서 `s = q̃_dot + Λ*q̃` (`q̃ = q - q_d`)는 위치 오차와 속도 오차를 묶은 sliding variable이고, `theta_hat`은 파라미터의 현재 추정값, `Gamma`는 추정 속도를 조절하는 적응 게인 행렬이다. 참고로 Slotine–Li(1987)의 원래 법칙에서는 위 식의 `Y`를 계산할 때 노이즈가 심한 실제 가속도 `q_ddot` 대신 기준 속도 `q_dot_r = q_dot_d - Λ*q̃`와 그 시간 미분을 집어넣는다. 이 방식에는 현장에서 관절 가속도를 직접 측정할 필요가 없다는 큰 장점이 있다.

이제 Lyapunov 함수 후보를 오차 에너지와 파라미터 추정 오차 에너지의 합인 `V = 0.5*s^T*M*s + 0.5*theta_tilde^T*Gamma^(-1)*theta_tilde`로 잡으면, 코리올리 행렬의 반대칭성(`M_dot - 2C`) 덕분에 `V_dot <= 0`이 성립하여 추종 오차 `q̃`가 0으로 수렴함을 깔끔하게 증명할 수 있다. 다만 추종 오차가 0으로 간다고 해서 추정값 `theta_hat`이 언제나 진짜 물리 파라미터 `theta`와 일치하는 것은 아니다. 파라미터 참값까지 정확히 식별하려면 입력 궤적이 **persistent excitation** 조건을 만족해야 한다. 즉, 궤적에 충분히 다양한 주파수와 동작이 포함되어야 한다.

### Robust Control

파라미터를 온라인으로 추정해 맞추는 적응 제어와 달리, 모델에 불확실성이나 외란이 끼어들더라도 그 **상한(bound)**을 미리 알고 있을 때 최악의 상황까지 견디도록 제어기를 고정 설계하는 접근을 **강인 제어(Robust Control)**라 한다.

- **H-infinity control**: 최악의 외란이 들어오는 시나리오에서도 성능 저하를 일정 한계 안으로 억제한다. 수학적으로는 시스템의 유도 L2 노름 상한을 보장해, 외란 신호에서 제어 오차로 건너가는 에너지 증폭 비율(이득)이 설정값 $\gamma$ 이하가 되도록 만든다. 외란 에너지 자체가 커지면 오차 상한도 비례해 커지지만 그 증폭 배율을 묶어 두는 셈이다. 설계 과정에서 Riccati 부등식이나 선형 행렬 부등식(LMI)을 푸는데, 최악의 경우를 기준으로 방어선을 치다 보니 평상시 응답이 다소 보수적으로 나올 수 있다.
- **Sliding Mode Control (SMC)**: 상태 공간에 미리 정해 둔 슬라이딩 면(sliding surface)으로 상태를 유한 시간 안에 강제로 끌어당긴 뒤, 그 면 위를 미끄러지며 원하는 동특성을 따라 원점으로 내려오게 만든다. 모델 불확실성과 외란에 대단히 강인하지만, 슬라이딩 면 근처에서 제어 입력 부호가 고주파로 뒤집히는 현상 때문에 액추에이터에 무리가 가는 **채터링(chattering)**이 발생한다. 실무에서는 면 주변에 얇은 경계층을 두어 스위칭을 부드럽게 눕히는 boundary layer approach나 고차 슬라이딩 모드(higher-order sliding mode)로 채터링을 완화해 쓴다.

### 언제 쓰는가, 언제 안 쓰는가

지금까지 다룬 수많은 제어기 가운데 내 로봇과 작업 조건에 무엇이 맞는지 한눈에 가늠할 수 있도록 상황별 선택 기준을 정리하면 다음과 같다.

| 상황 | 추천 | 비추천 |
|------|------|--------|
| 모델이 정확하고 선형성 충분 | LQR, MPC | 적응 제어 (과설계) |
| 파라미터 불확실성이 큼 | 적응 제어 | PID만으로 버티기 |
| 불확실성 범위를 알고 있음 | Robust control (H-inf) | 적응 제어 (불필요) |
| 안전 인증이 필요함 | Lyapunov 기반 증명 | "시뮬레이션에서 됐으니까 OK" |
| 빠르게 프로토타입 | PID + feedforward | 처음부터 H-infinity |

결국 적응 제어와 sliding mode, MPC, PID 중 무엇을 고를지는 시스템 불확실성의 구조와 작업이 요구하는 수학적 보장의 수준을 따져 결정해야 한다. 특히 사람과 부대끼는 협동 로봇이나 자율주행차처럼 안전 인증이 걸린 시스템에서는 시뮬레이션 성공 사례를 나열하는 데 그치지 않고 Lyapunov 함수 등 엄밀한 해석으로 안정성을 뒷받침해야 한다. 물론 산업계 기능안전 규격은 위험 분석과 안전 등급 결정, 검증·확인(V&V) 증거를 요구할 뿐 특정 수학 증명 기법 하나를 강제하지는 않으므로, 안정성 해석 결과는 전체 안전 근거를 탄탄히 받쳐 주는 핵심 자료로 활용하면 된다.

---

## 참고 자료

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
> QP solver. 코드 생성(code generation)으로 임베디드 시스템에 배포할 수 있다. C로 구현되어 있고 Python, MATLAB, Julia 바인딩을 함께 제공한다.

> **주요 논문**
> - [Hogan, "Impedance Control: An Approach to Manipulation: Part I—Theory" (ASME JDSMC 107(1), 1985)](https://doi.org/10.1115/1.3140702) — 임피던스 제어의 원논문(3부작 중 1부)
> - [Khatib, "A Unified Approach for Motion and Force Control of Robot Manipulators: The Operational Space Formulation" (IEEE JRA 1987)](https://doi.org/10.1109/JRA.1987.1087068) — Operational Space Control의 원논문(IEEE JRA 3(1):43–53)
> - [Khazoom et al., "Tailoring Solution Accuracy for Fast Whole-Body Model Predictive Control of Legged Robots" (RA-L 2024, arXiv:2407.10789)](https://arxiv.org/abs/2407.10789) — SQP 한 번의 QP 부분 문제를 ADMM(OSQP)으로 낮은 정확도까지만 풀어 whole-body NMPC를 실시간으로 돌리는 방법. MIT Humanoid 실기에서 90Hz(dt=25ms 설정)로 실행

---

## 발전 과정

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

이 장의 제어기들은 모두 "따라가야 할 목표 상태나 기준 궤적 `r(t)`가 이미 주어져 있다"는 전제 위에서 출발했다. 하지만 실제 작업에서는 "책상 위 장애물을 피하면서 저 컵까지 어떤 경로로 팔을 뻗을 것인가", 그리고 "토크와 속도 한계를 넘지 않는 매끄러운 기준 궤적을 애초에 어떻게 만들 것인가"를 먼저 결정해야 한다. 다음 7장에서는 바로 그 앞단 문제인 모션 플래닝과 궤적 최적화를 다룬다.
