# Ch.4 — 기구학 & 메카트로닉스 (Kinematics & Mechatronics)


로봇 팔 하나를 책상 위에 올려놓았다고 하자. 모터 6개에 각각 어떤 각도를 줘야 손끝이 커피잔에 닿는가? 이 질문에 답하는 학문이 기구학이다. 그리고 그 모터를 실제로 돌리고, 센서를 읽고, 제어 루프를 1kHz로 돌리는 현실의 문제가 메카트로닉스이다.

기구학의 수식은 하드웨어 선정과 통신 프로토콜을 거쳐 실제 로봇의 동작으로 이어진다.

---

## 4.1 왜 기구학을 배우는가

로봇 매니퓰레이터는 여러 개의 관절(joint)과 링크(link)로 구성된다. 우리가 원하는 것은 끝단(end-effector)의 위치와 자세(pose)이다. 하지만 우리가 직접 제어하는 것은 각 관절의 각도(또는 변위)이다.

이 둘 사이의 관계를 수학적으로 기술하는 것이 **기구학(Kinematics)**이다.

- 순기구학(Forward Kinematics, FK): 관절 각도 → 끝단 위치/자세
- 역기구학(Inverse Kinematics, IK): 끝단 위치/자세 → 관절 각도

동역학(Dynamics)과 다르다. 기구학은 힘과 질량을 고려하지 않는다. 어디에 있는가의 문제이지, 어떤 힘이 필요한가의 문제가 아니다. 동역학은 다음 장에서 다룬다.

기구학은 다음 작업에 쓰인다:
- 로봇 팔 경로 계획 (motion planning)
- 텔레오퍼레이션 (원격 조종 시 마스터-슬레이브 매핑)
- 캘리브레이션 (실제 로봇과 모델 사이 오차 보정)
- 충돌 회피 (각 링크가 공간 어디에 있는지 알아야 피한다)

---

## 4.2 순기구학 (Forward Kinematics)

### 4.2.1 동차 변환 행렬 (Homogeneous Transformation Matrix)

기구학의 기본 도구는 4×4 동차 변환 행렬이다:

```
T = | R  p |
    | 0  1 |
```

여기서 R은 3×3 회전 행렬, p는 3×1 위치 벡터이다. 동차 변환 행렬 하나로 강체의 위치와 자세를 함께 표현하고, 여러 변환을 행렬 곱으로 연쇄(chain)할 수 있다.

두 프레임 사이의 변환 T_01이 있고, 또 다른 변환 T_12가 있으면:

```
T_02 = T_01 * T_12
```

순기구학은 베이스에서 끝단까지 각 관절의 변환을 순서대로 곱해 이 합성 변환을 구한다.


### 4.2.2 DH Parameters (Denavit-Hartenberg)

1955년 Denavit와 Hartenberg가 제안한 방법이다. 지금도 로봇 기구학에서 널리 쓰이는 좌표계 규약이며, 4개의 파라미터로 인접한 두 링크 사이의 관계를 정의한다:

| 파라미터 | 의미 |
|---------|------|
| a_i (link length) | x_i 축을 따른 z_{i-1}에서 z_i까지의 거리 |
| α_i (link twist) | z_{i-1}에서 z_i까지 x_i 축 기준 회전 각도 |
| d_i (link offset) | z_{i-1} 축을 따른 x_{i-1}에서 x_i까지의 거리 |
| θ_i (joint angle) | x_{i-1}에서 x_i까지 z_{i-1} 축 기준 회전 각도 |

회전 관절(revolute joint)에서는 θ_i가 변수이고, 나머지 3개는 상수이다.
직선 관절(prismatic joint)에서는 d_i가 변수이다.

각 관절의 변환 행렬:

```
T_i = Rot_z(θ_i) * Trans_z(d_i) * Trans_x(a_i) * Rot_x(α_i)

    = | cos(θ)  -sin(θ)cos(α)   sin(θ)sin(α)   a*cos(θ) |
      | sin(θ)   cos(θ)cos(α)  -cos(θ)sin(α)   a*sin(θ) |
      | 0        sin(α)          cos(α)          d        |
      | 0        0               0               1        |
```

주의: DH convention에는 "standard"와 "modified (Craig convention)" 두 가지가 있다. Craig 교과서를 쓴다면 modified DH를 보게 되고, 많은 다른 교재는 standard DH를 사용한다. 둘은 프레임 부착 방식이 다르다. 혼용하면 결과가 틀리니 어떤 convention을 쓰는지 항상 명시해야 한다.


### 4.2.3 예제: 2-link Planar Arm의 FK

가장 간단한 예제부터 하자. 평면 위의 2-링크 로봇 팔이다.

```
       q1         q2
  O────────O────────O → end-effector
  (base)   L1       L2
```

DH 테이블 (standard convention):

| Link | a    | α   | d   | θ    |
|------|------|-----|-----|------|
| 1    | L1   | 0   | 0   | θ_1  |
| 2    | L2   | 0   | 0   | θ_2  |

끝단 위치는 단순히 삼각함수로 유도된다:

```
x = L1*cos(θ_1) + L2*cos(θ_1 + θ_2)
y = L1*sin(θ_1) + L2*sin(θ_1 + θ_2)
```

Python으로 구현하면:

```python
import numpy as np

def fk_2link(theta1, theta2, L1=1.0, L2=1.0):
    """2-link planar arm의 순기구학."""
    x = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
    y = L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2)
    phi = theta1 + theta2  # 끝단의 절대 방향
    return x, y, phi

# θ_1=30°, θ_2=45°, 링크 길이 각각 1m
x, y, phi = fk_2link(np.radians(30), np.radians(45))
print(f"End-effector position: ({x:.3f}, {y:.3f}), orientation: {np.degrees(phi):.1f}°")
# 출력: End-effector position: (0.259, 1.366), orientation: 75.0°
```

단순해 보인다면 맞다. 실제 6축 로봇 팔의 FK도 원리는 같다. 4×4 행렬을 6번 곱하면 된다.


### 4.2.4 Product of Exponentials (PoE)

DH 파라미터의 대안으로, Lie group/Lie algebra에 기반한 PoE (Product of Exponentials) 방법이 있다. Lynch & Park의 "Modern Robotics"에서 채택한 방법이다.

PoE는 각 관절을 twist(나선 운동)로 표현하고, 행렬 지수(matrix exponential)로 변환을 계산한다.

```
T(θ) = e^{[S_1]θ_1} * e^{[S_2]θ_2} * ... * e^{[S_n]θ_n} * M
```

여기서:
- S_i는 i번째 관절의 screw axis (6×1 벡터)
- [S_i]는 S_i의 4×4 skew-symmetric matrix 표현 (se(3) 원소)
- M은 모든 관절이 영 위치(home configuration)일 때의 끝단 자세
- θ_i는 관절 변수

DH vs PoE 비교:

| 항목 | DH | PoE |
|------|-----|-----|
| 프레임 부착 | 각 링크에 프레임 필요 | 기준 프레임과 끝단 프레임만 필요 |
| Convention 혼동 | standard vs modified 주의 | 없음 (space form vs body form 구분은 있음) |
| 수학적 기반 | 행렬 곱 | Lie group, 행렬 지수 |
| 특이점 분석 | 별도 처리 필요 | 자연스럽게 통합 |
| 산업계 채택 | 매우 높음 | 학계 중심, 점점 확산 |
| 교재 | Craig, Siciliano | Lynch & Park |

DH 파라미터는 교재와 산업용 로봇 매뉴얼에서 흔히 쓰인다. URDF는 같은 링크·관절 변환을 직접 기술한다. PoE는 Lie group에 기반한 정돈된 표현으로 연구에서 널리 쓰인다. 매뉴얼, 로봇 기술 파일, 수식 유도 사이를 오가려면 두 관례의 대응을 알아두는 편이 좋다.

```python
# robotics-toolbox-python으로 DH 기반 FK 예제 (Puma 560)
import roboticstoolbox as rtb

puma = rtb.models.DH.Puma560()
q = [0, -np.pi/4, np.pi/4, 0, np.pi/6, 0]  # 6개 관절 각도
T = puma.fkine(q)
print(T)  # 4x4 SE(3) 동차 변환 행렬 출력
print(f"Position: {T.t}")  # 끝단 위치
print(f"RPY angles: {T.rpy()}")  # Roll-Pitch-Yaw
```

> **추천 자료**
> - Lynch & Park, *Modern Robotics*, Chapter 4 — PoE를 중심으로 설명하며 무료 PDF와 Coursera 강의를 제공한다: https://modernrobotics.org
> - Craig, *Introduction to Robotics*, Chapter 3 — Modified DH convention을 사용하는 교재
> - Peter Corke, *Robotics, Vision and Control* — Python 코드와 함께 FK를 실습할 수 있다: https://github.com/petercorke/robotics-toolbox-python

---

## 4.3 역기구학 (Inverse Kinematics)

FK는 쉽다. 행렬 곱이면 된다. 문제는 IK이다.

"끝단을 (x, y, z)에 놓고 싶은데, 관절 각도를 각각 얼마로 해야 하는가?"

이 문제가 어려운 이유는 네 가지다. 삼각함수가 얽힌 비선형 방정식이고, 같은 끝단 위치에 도달하는 관절 각도 조합이 여러 개일 수 있다(elbow-up, elbow-down). Workspace 밖의 점은 아예 해가 없고, 자유도가 남으면(redundant manipulator) 해가 무한히 많다.


### 4.3.1 Analytical IK (해석적 방법)

닫힌 형태(closed-form)의 해를 구하는 방법이다. 해가 존재하면 반복 최적화 없이 후보를 계산할 수 있지만, 수치 정확도와 실행 시간은 구현과 특이점 처리에 따라 달라진다.

**2-link planar arm의 IK:**

목표 위치 (x, y)가 주어졌을 때:

```
cos(θ_2) = (x² + y² - L1² - L2²) / (2 * L1 * L2)
θ_2 = atan2(±√(1 - cos²(θ_2)), cos(θ_2))

θ_1 = atan2(y, x) - atan2(L2*sin(θ_2), L1 + L2*cos(θ_2))
```

±에서 보듯이 해가 두 개다(elbow-up, elbow-down). 여러 해가 존재한다는 점이 IK를 어렵게 만든다.

```python
def ik_2link(x, y, L1=1.0, L2=1.0, elbow_up=True):
    """2-link planar arm의 역기구학. 해가 없으면 None 반환."""
    d_sq = x**2 + y**2
    # 도달 가능 여부 체크
    if d_sq > (L1 + L2)**2 or d_sq < (L1 - L2)**2:
        return None

    cos_q2 = (d_sq - L1**2 - L2**2) / (2 * L1 * L2)
    cos_q2 = np.clip(cos_q2, -1.0, 1.0)  # 수치 안전

    if elbow_up:
        q2 = np.arctan2(np.sqrt(1 - cos_q2**2), cos_q2)
    else:
        q2 = np.arctan2(-np.sqrt(1 - cos_q2**2), cos_q2)

    q1 = np.arctan2(y, x) - np.arctan2(L2 * np.sin(q2), L1 + L2 * np.cos(q2))
    return q1, q2

# 검증: FK → IK → FK
target_x, target_y = 1.2, 0.8
result = ik_2link(target_x, target_y)
if result:
    q1, q2 = result
    x_check, y_check, _ = fk_2link(q1, q2)
    print(f"Target: ({target_x}, {target_y})")
    print(f"IK solution: q1={np.degrees(q1):.2f}°, q2={np.degrees(q2):.2f}°")
    print(f"FK check: ({x_check:.6f}, {y_check:.6f})")
    print(f"Error: {np.sqrt((x_check-target_x)**2 + (y_check-target_y)**2):.2e}")
```

**6R 매니퓰레이터의 해석적 IK:**

6축 로봇 중 Pieper의 조건을 만족하는 구조 — 마지막 3축이 한 점에서 만나는(spherical wrist) 경우 — 는 해석적으로 풀 수 있다. 대부분의 산업용 6축 로봇(UR, KUKA, ABB 등)이 이 구조이다.

이 경우 위치 문제(처음 3축)와 자세 문제(마지막 3축)를 분리하여 풀 수 있다. 최대 8개의 해가 존재하며, 관절 제한(joint limits)과 이전 관절 각도에 가까운 해를 선택하는 것이 일반적이다.


### 4.3.2 Numerical IK (수치적 방법)

해석적 해가 불가능한 경우 (복잡한 구조, 7축 이상, 비표준 구조) 수치적으로 풀어야 한다. 반복적 최적화 문제이다.

**Jacobian Pseudo-Inverse 방법:**

```
Δq = J†(q) * Δx
```

여기서 J†는 자코비안의 pseudo-inverse이다. 이를 반복하여 목표에 수렴한다.

```python
def numerical_ik_2link(target_x, target_y, L1=1.0, L2=1.0,
                        max_iter=100, tol=1e-6):
    """Jacobian pseudo-inverse 기반 수치적 IK."""
    # 초기 추정값 (랜덤 또는 현재 관절 각도)
    q = np.array([0.5, 0.5])

    for i in range(max_iter):
        # 현재 FK
        x = L1 * np.cos(q[0]) + L2 * np.cos(q[0] + q[1])
        y = L1 * np.sin(q[0]) + L2 * np.sin(q[0] + q[1])

        # 오차
        error = np.array([target_x - x, target_y - y])
        if np.linalg.norm(error) < tol:
            print(f"수렴: {i+1}회 반복")
            return q

        # 자코비안
        J = np.array([
            [-L1*np.sin(q[0]) - L2*np.sin(q[0]+q[1]), -L2*np.sin(q[0]+q[1])],
            [ L1*np.cos(q[0]) + L2*np.cos(q[0]+q[1]),  L2*np.cos(q[0]+q[1])]
        ])

        # Pseudo-inverse로 관절 각도 업데이트
        dq = np.linalg.pinv(J) @ error
        q += dq

    print("수렴 실패")
    return q
```

**Damped Least Squares (DLS, Levenberg-Marquardt):**

Pseudo-inverse의 문제는 특이점 근처에서 관절 속도가 폭발한다는 것이다. DLS는 damping factor λ를 추가하여 이를 완화한다:

```
Δq = J^T (J * J^T + λ²I)^{-1} * Δx
```

λ가 크면 특이점 근처에서 안정적이지만 수렴이 느리고, λ가 작으면 pseudo-inverse에 가까워진다. 적응적으로 λ를 조절하는 방법(Nakamura & Hanafusa, 1986)이 실무에서 많이 쓰인다.


### 4.3.3 특이점 (Singularity)

자코비안의 rank가 부족해지는 관절 배치를 특이점(singularity)이라 한다. 특이점에서는 특정 방향으로 끝단을 전혀 움직일 수 없고, 미소 이동에도 관절 속도가 폭발하며, IK 해가 불연속적이어서 경로 추종 시 관절이 급격히 점프한다.

2-link arm의 특이점은 간단하다: θ_2 = 0 (팔이 완전히 펴진 경우) 또는 θ_2 = π (완전히 접힌 경우). 이때 끝단은 반지름 방향으로만 움직일 수 있고, 접선 방향 속도는 낼 수 없다.

6축 로봇의 대표적 특이점:
- Wrist singularity: 축 4와 6이 정렬됨 (q5 ≈ 0)
- Shoulder singularity: 끝단이 축 1 위에 위치
- Elbow singularity: 팔이 완전히 펴짐

실무 대처법:
- 특이점 근처를 피하는 경로 계획
- DLS 방법으로 특이점 통과 시 속도 제한
- Redundancy(여분의 자유도) 활용


### 4.3.4 IK 솔버들

직접 IK를 구현할 일은 드물다. 검증된 솔버를 사용하는 것이 현명하다.

| 솔버 | 방법 | 특징 |
|------|------|------|
| KDL | Numerical (Newton-Raphson) | ROS 생태계에서 제공, 관절 한계·초기값·특이점에 민감 |
| IKFast (OpenRAVE) | Analytical (코드 생성) | 특정 구조에 대해 C++ 코드 자동 생성. 빠름 |
| TRAC-IK | KDL + SQP 듀얼 | KDL보다 성공률 높음, ROS 패키지 존재 |
| MoveIt2 IK | 위 솔버들을 통합 | ROS2 생태계, 충돌 회피 통합 |
| pinocchio | PoE 기반 | 현대적, 빠름, 미분 가능 (differentiable) |

```python
# Beeson & Ames (2015)는 5개 로봇 모델의 도달 가능한 자세를
# 모델별 10,000개씩, 해 하나당 5ms 제한으로 비교했다.
# 그 실험에서는 TRAC-IK가 stock KDL보다 높은 solve rate를 보였지만,
# 수치는 관절 체인·초기값·허용 오차에 따라 달라진다.
```

> **추천 자료**
> - [Beeson & Ames, "TRAC-IK: An Open-Source Library for Improved Solving of Generic Inverse Kinematics" (2015)](https://doi.org/10.1109/HUMANOIDS.2015.7363472) — 모델별 조건과 solve rate는 원 논문의 표를 확인
> - MoveIt2 IK 문서: https://moveit.picknik.ai/main/doc/concepts/inverse_kinematics.html
> - Pinocchio (rigid body dynamics library): https://github.com/stack-of-tasks/pinocchio

---

## 4.4 자코비안 (Jacobian)

자코비안은 기구학에서 가장 많이 쓰이는 도구 중 하나이다. FK가 "위치"의 문제라면, 자코비안은 "속도"의 문제이다.


### 4.4.1 관절 속도 → 끝단 속도

끝단 속도(선속도 v, 각속도 ω)와 관절 속도 q̇의 관계:

```
ẋ = J(q) * q̇

여기서 ẋ = [v; ω] ∈ ℝ^6 (6축의 경우)
      q̇ ∈ ℝ^n
      J(q) ∈ ℝ^{6×n}
```

n < 6이면 under-actuated, n = 6이면 fully-actuated, n > 6이면 redundant이다.

차륜 mobile robot의 경우 관절 속도가 아닌 차체 속도 $(v, \omega)$가 주 제어이며, 노이즈가 동반된 형태는 §4.7 확률적 운동 모델 참조.


### 4.4.2 힘/토크 관계 (Duality)

자코비안의 전치(transpose)는 끝단 힘을 관절 토크로 매핑한다:

```
τ = J^T(q) * F
```

여기서 τ는 관절 토크, F는 끝단에 작용하는 힘/모멘트이다.

이것이 **정역학적 이중성(static duality)**이다. 속도와 힘은 자코비안과 그 전치를 통해 쌍대 관계를 이룬다. 파워 보존 원리에서 자연스럽게 유도된다:

```
P = F^T * ẋ = F^T * J * q̇ = (J^T * F)^T * q̇ = τ^T * q̇
```

이 관계는 힘 제어(force control)에서 핵심적이다. 끝단에 원하는 힘 F를 가하려면, 각 관절에 τ = J^T * F의 토크를 인가하면 된다.


### 4.4.3 Manipulability Ellipsoid

자코비안은 로봇이 현재 자세에서 "얼마나 잘 움직일 수 있는지"도 알려준다.

```
manipulability index = √det(J * J^T)
```

이 값이 0이면 특이점이다. 값이 클수록 모든 방향으로 고르게 움직일 수 있다.

J * J^T의 고유값(eigenvalue)과 고유벡터(eigenvector)로 타원체(ellipsoid)를 그릴 수 있다. 고유값이 크면 그 방향으로 빠르게 움직일 수 있고, 작으면 느리다. 고유값이 모두 비슷하면 등방적(isotropic)이고, 차이가 크면 비등방적이다.

```python
import roboticstoolbox as rtb
import numpy as np

# Puma 560의 자코비안과 manipulability
puma = rtb.models.DH.Puma560()
q = [0, -np.pi/4, np.pi/4, 0, np.pi/6, 0]

J = puma.jacob0(q)  # 6x6 자코비안 (기저 프레임 기준)

# Manipulability index
m = np.sqrt(np.linalg.det(J @ J.T))
print(f"Manipulability index: {m:.4f}")

# 속도 타원체의 주축 (고유값 분석)
JJT = J[:3, :] @ J[:3, :].T  # 선속도 부분만
eigenvalues, eigenvectors = np.linalg.eigh(JJT)
print(f"Velocity ellipsoid semi-axes: {np.sqrt(eigenvalues)}")

# Condition number: 등방성 지표 (1에 가까울수록 좋다)
sigma = np.linalg.svd(J, compute_uv=False)
cond = sigma[0] / sigma[-1]
print(f"Condition number: {cond:.2f}")
# cond가 1이면 완벽한 등방성, 무한대면 특이점
```


### 4.4.4 실용 코드: 자코비안 기반 속도 제어

```python
import numpy as np

def jacobian_velocity_control(robot_fk, robot_jacob, q_current,
                               desired_twist, dt=0.001):
    """
    자코비안 기반 분해 속도 제어 (resolved rate control).

    Args:
        robot_fk: FK 함수 (q -> SE3)
        robot_jacob: 자코비안 함수 (q -> 6xn matrix)
        q_current: 현재 관절 각도
        desired_twist: 원하는 끝단 속도 [vx, vy, vz, wx, wy, wz]
        dt: 제어 주기
    Returns:
        q_new: 새 관절 각도
    """
    J = robot_jacob(q_current)

    # Damped least squares
    lambda_dls = 0.01
    n = J.shape[1]
    JJT = J @ J.T
    J_dls = J.T @ np.linalg.inv(JJT + lambda_dls**2 * np.eye(JJT.shape[0]))

    q_dot = J_dls @ desired_twist

    # 관절 속도 제한 (실제 로봇에서 필수)
    max_qdot = 2.0  # rad/s
    scale = np.max(np.abs(q_dot)) / max_qdot
    if scale > 1.0:
        q_dot /= scale

    q_new = q_current + q_dot * dt
    return q_new
```

> **추천 자료**
> - Siciliano et al., *Robotics: Modelling, Planning and Control*, Chapter 3 — 자코비안을 기구학·동역학 맥락에서 폭넓게 설명
> - Corke, *Robotics, Vision and Control*, Chapter 8 — 코드 예제와 시각화 포함: https://petercorke.com/rvc/
> - robotics-toolbox-python 문서: https://github.com/petercorke/robotics-toolbox-python

---

## 4.5 메카트로닉스 기초

관절 각도를 정한다고 로봇이 움직이는 것은 아니다. 모터와 센서, 그 사이를 연결하는 전자 회로와 통신이 있어야 한다. 이것이 메카트로닉스다.


### 4.5.1 액추에이터

**DC 모터:**
가장 기본적인 액추에이터. 전압을 가하면 회전한다. 토크는 전류에 비례하고 (τ = K_t * i), 역기전력은 속도에 비례한다 (V_emf = K_e * ω). 제어가 쉽고 가격이 저렴하지만, 브러시 마모가 있다.

**BLDC (Brushless DC) 모터:**
브러시 없이 전자적으로 전류를 전환한다. 수명이 길고 토크 밀도와 효율이 높아 현대 로봇에서 자주 선택된다. FOC(Field-Oriented Control)는 토크 리플(ripple)을 줄이는 데 쓰인다.

**서보 모터 (Dynamixel 시리즈):**
모터 + 감속기 + 엔코더 + 컨트롤러를 일체형으로 묶은 제품이다. Robotis의 Dynamixel은 연구·교육용 플랫폼에서 널리 쓰이는 서보 제품군이다.

| 모델 | 공칭 최대 토크 예시 (Nm) | 통신 | 용도 |
|------|-----------|------|------|
| XL330 | 0.5 | TTL | 소형 그리퍼, SO-ARM100 등 |
| XM540 | 10.0 | RS-485 | 중형 로봇 팔 |
| PH54  | 44.7 | RS-485 | 대형 매니퓰레이터, 모바일 로봇 |

표의 토크는 모델과 공급 전압에 따라 달라지므로 실제 선정에는 각 e-Manual의 정격·stall 조건과 연속 운전 한계를 확인해야 한다. Dynamixel의 장점은 데이지 체인 연결, 위치/속도/전류 기반 제어 모드, PID 게인 조절이다. 단점은 제품별 통신·제어 주기와 열 한계이고, 필요한 대역폭과 제어 모드가 기본 펌웨어에서 지원되는지 먼저 확인해야 한다.

**Quasi-Direct Drive (QDD):**

MIT Mini Cheetah(2019)로 주목받은 방식으로, 감속비를 낮춘다.

일반적인 로봇 관절: 감속비 100:1 이상 (harmonic drive)
QDD: 감속비 6:1 ~ 10:1 (유성기어 또는 belt)

낮은 감속비의 장점은 세 방향에서 나타난다. 외력이 가해졌을 때 관절이 따라가기 쉬운 백드라이버빌리티(backdrivability)가 높아지고, 충돌 대응과 힘 제어 설계가 단순해질 수 있다. 감속기 마찰을 충분히 모델링하면 모터 전류에서 관절 토크를 근사하기도 쉽다. 감속기의 마찰과 탄성이 작을수록 높은 토크 응답 대역폭을 설계할 여지도 커진다.

단점: 동일 크기 대비 출력 토크가 낮다. 큰 토크가 필요하면 더 큰 모터를 써야 한다.

QDD를 사용하는 최근 시스템들:
- MIT Mini Cheetah / Cheetah 3
- ALOHA (low-cost bimanual teleop)
- Unitree 로봇 시리즈

```
# QDD vs 전통적 감속기의 토크 제어 비교
#
# 전통적 (감속비 100:1, harmonic drive):
#   반사 관성 (reflected inertia) = N² × I_motor
#   → 모터 관성 0.001 kg·m² × 100² = 10 kg·m²
#   → 관절 출력 측에 반사되는 모터 관성이 매우 크다
#   → 정밀한 힘 제어가 어렵다
#
# QDD (감속비 8:1):
#   같은 모터를 비교하면 반사 관성 = 8² × 0.001 = 0.064 kg·m²
#   → 감속비만 바꾼 이 예에서는 약 156배 작다
#   → 실제 힘 제어 성능은 링크 관성·마찰·제어기에도 좌우된다
```


**감속기 종류:**

| 종류 | 감속비 | 백래시 | 효율 | 가격 | 용도 |
|------|--------|--------|------|------|------|
| Planetary | 3~100:1 | 중간 | 85-95% | 저렴 | 범용, QDD에 적합 |
| Harmonic Drive | 30~320:1 | 매우 낮음 | 65-85% | 비쌈 | 산업용 로봇, 정밀 |
| Cycloidal | 6~120:1 | 낮음 | 85-93% | 중간 | 최근 대안으로 부상 |

감속비·효율·백래시는 구조와 제품에 따라 크게 달라진다. 표의 범위는 계열을 비교하기 위한 출발점이며, 선정할 때는 제조사 데이터시트의 정격 부하 조건을 사용한다.

**액추에이터 선정 기준:**

로봇 관절의 액추에이터를 선정할 때는 정적 토크(자세 유지), 동적 토크(가속), 충격 하중을 합산하고 하중 불확실성·수명·고장 결과에 맞는 여유 계수를 둔다. 아래 코드의 2배는 계산 예시이지 보편 규칙이 아니다. 필요 속도는 관절의 최대 각속도와 감속비로부터 모터 RPM으로 환산한다. 백드라이버빌리티, 크기와 무게, 연속 토크와 열 한계도 함께 검토한다. QDD와 harmonic drive 중 어느 쪽이 나은지는 토크 밀도, 투명도, 정밀도, 비용 요구에 따라 달라진다.

```python
# 간단한 액추에이터 선정 계산 예시
import numpy as np

# 목표: 1kg 물체를 팔 끝에서 들어올리기 (팔 길이 0.5m)
m_payload = 1.0  # kg
m_link = 0.5     # 링크 자체 무게
L = 0.5          # m
g = 9.81         # m/s²

# 최악의 경우 토크 (수평으로 뻗었을 때)
tau_static = (m_payload * L + m_link * L/2) * g
print(f"정적 토크: {tau_static:.2f} Nm")

# 가속 토크 (최대 각가속도 10 rad/s²)
alpha_max = 10.0  # rad/s²
I_total = m_payload * L**2 + m_link * (L/2)**2  # 관성 모멘트 (단순화)
tau_dynamic = I_total * alpha_max
print(f"동적 토크: {tau_dynamic:.2f} Nm")

# 총 필요 토크 (안전 계수 2)
tau_required = (tau_static + tau_dynamic) * 2.0
print(f"필요 토크 (안전 계수 2): {tau_required:.2f} Nm")

# 최대 각속도 → 모터 RPM
omega_max = 3.0  # rad/s (관절)
gear_ratio = 8    # QDD
motor_rpm = omega_max * gear_ratio * 60 / (2 * np.pi)
print(f"모터 필요 RPM: {motor_rpm:.0f}")
```

> **추천 자료**
> - Katz, "A Low Cost Modular Actuator for Dynamic Robots" (MIT, 2018) — QDD의 핵심 논문: https://dspace.mit.edu/handle/1721.1/118671
> - Dynamixel 제품 라인업 및 문서: https://emanual.robotis.com/
> - Seok et al., "Design Principles for Energy-Efficient Legged Locomotion and Implementation on the MIT Cheetah Robot" (2015)


### 4.5.2 센서 인터페이싱

**엔코더 (Encoder):**

관절 각도를 측정하는 가장 기본적인 센서이다.

*Incremental encoder*: A, B 두 채널의 펄스를 세어 상대적 회전량을 측정한다. 전원이 꺼지면 위치를 잊는다 (homing 필요). 가격이 저렴하고, 분해능이 높다 (10,000 PPR 이상도 흔함).

*Absolute encoder*: 현재 위치를 절대값으로 출력한다. 전원을 켜자마자 위치를 안다. Multi-turn absolute encoder는 여러 바퀴를 기억한다. 가격이 비싸지만 homing이 필요 없어, 재기동 뒤 위치 복원이 중요한 산업용 로봇에 널리 쓰인다.

```
분해능 계산 예시:
  Incremental encoder, 4096 PPR, quadrature decoding (x4)
  → 분해능 = 360° / (4096 × 4) = 0.022° ≈ 0.38 mrad
  → 감속비 100:1 관절 → 출력 분해능 0.0038 mrad
```

**토크 센서:**

관절 토크 또는 끝단 힘을 직접 측정한다. 스트레인 게이지(strain gauge) 기반이 대부분이다.

*관절 토크 센서 (Joint Torque Sensor, JTS)*: 감속기 출력 측에 장착. KUKA LBR iiwa가 7개 관절 모두에 JTS를 장착하여 힘 제어의 기준을 세웠다.

*힘/토크 센서 (F/T Sensor)*: 끝단에 장착하여 6축(Fx, Fy, Fz, Tx, Ty, Tz)을 측정한다. ATI Industrial Automation을 비롯한 업체가 연구용 센서를 공급하며, 선정할 때는 측정 범위·분해능·과부하 한계·인터페이스와 견적을 함께 확인한다.

**관성 센서 (IMU):**

2장에서 이미 다루었으므로 간략히 언급한다. 가속도계 + 자이로스코프 + (자력계). 모바일 로봇이나 legged robot의 몸체 자세 추정에 사용. 매니퓰레이터에서는 링크별 IMU를 달아 진동 감쇠에 활용하기도 한다.


### 4.5.3 통신 프로토콜

센서와 액추에이터를 마이크로컨트롤러/PC에 연결하는 방법이다. 로봇 시스템에서 통신은 생각보다 많은 문제를 일으킨다. 지연(latency)이 크면 제어가 불안정해지고, 대역폭이 부족하면 데이터가 누락된다.

**기초 프로토콜:**

| 프로토콜 | 배선 | 속도 | 거리 | 특징 |
|---------|------|------|------|------|
| **UART** | 2선 (TX, RX) | ~1 Mbps | ~15m | 가장 단순, 1:1 통신 |
| **SPI** | 4선 (MOSI, MISO, SCK, CS) | ~50 Mbps | ~1m (PCB 내) | 빠름, 다수 슬레이브는 CS 추가 |
| **I2C** | 2선 (SDA, SCL) | 100k~3.4 Mbps | ~1m | 주소 기반, 센서 연결에 편리 |

이 셋은 마이크로컨트롤러 수준의 기초이다. 로봇 시스템에서는 더 강건한 프로토콜이 필요하다.

**CAN Bus:**

자동차 산업에서 시작했으며 로봇의 모터와 센서 네트워크에도 쓰인다. 차동 신호(differential signaling)로 노이즈에 강하고, 멀티마스터 구조에 우선순위 기반 중재(arbitration)를 지원한다.

- 속도: 최대 1 Mbps (CAN 2.0), 5 Mbps (CAN FD)
- 거리: 최대 1km (125 kbps에서)
- 토폴로지: 버스 (데이지 체인 가능)

로봇에서의 활용: 모터 드라이버와 메인 컨트롤러 사이 통신. MIT Cheetah, 많은 legged robot이 CAN을 사용한다.

```cpp
// CAN bus를 통한 모터 명령 전송 예시 (pseudo-code, STM32 HAL)
#include "can.h"

struct MotorCommand {
    float position;   // rad
    float velocity;   // rad/s
    float torque;     // Nm
    float kp;         // position gain
    float kd;         // velocity gain
};

void send_motor_command(CAN_HandleTypeDef* hcan, uint8_t motor_id,
                        MotorCommand cmd) {
    CAN_TxHeaderTypeDef header;
    header.StdId = motor_id;   // 각 모터에 고유 CAN ID
    header.DLC = 8;            // 8 bytes (CAN 2.0 기본)
    header.RTR = CAN_RTR_DATA;

    // 부동소수점을 정수로 패킹 (로봇 모터 드라이버의 일반적 방식)
    uint8_t data[8];
    int16_t pos_int = (int16_t)(cmd.position / 0.001f);   // 0.001 rad 단위
    int16_t vel_int = (int16_t)(cmd.velocity / 0.01f);    // 0.01 rad/s 단위
    int16_t tau_int = (int16_t)(cmd.torque / 0.01f);      // 0.01 Nm 단위
    int16_t kp_int  = (int16_t)(cmd.kp / 0.01f);

    data[0] = pos_int >> 8;  data[1] = pos_int & 0xFF;
    data[2] = vel_int >> 8;  data[3] = vel_int & 0xFF;
    data[4] = tau_int >> 8;  data[5] = tau_int & 0xFF;
    data[6] = kp_int >> 8;   data[7] = kp_int & 0xFF;

    uint32_t mailbox;
    HAL_CAN_AddTxMessage(hcan, &header, data, &mailbox);
}
```

**EtherCAT:**

산업용 실시간 이더넷 프로토콜이다. 일반 이더넷 하드웨어를 사용하면서 마이크로초 단위의 결정론적(deterministic) 통신을 제공한다.

왜 로봇에서 쓰는가: 100 Mbps로 수십~수백 개 노드를 마이크로초 주기로 동기화하고, 패킷 지연이 일정하여 실시간 제어에 맞다. 마스터가 보낸 프레임을 각 슬레이브가 on-the-fly로 읽고 쓰는 방식이라 대역폭 효율이 극히 높다.

KUKA, Beckhoff, 그리고 최근의 많은 연구용 로봇 플랫폼이 EtherCAT을 사용한다.

단점: 전용 마스터 소프트웨어 필요 (SOEM, IgH EtherCAT Master 등), 설정이 복잡하다. 취미 수준에서는 과도한 선택이다.

**RS-485 / Dynamixel Protocol:**

Dynamixel 서보의 통신 방식이다. RS-485는 차동 신호 기반의 시리얼 통신으로, 최대 1 Mbps, 여러 장치를 데이지 체인으로 연결할 수 있다.

```python
# Dynamixel SDK를 이용한 서보 제어 예시
from dynamixel_sdk import *

PROTOCOL_VERSION = 2.0
BAUDRATE = 1000000
DEVICENAME = '/dev/ttyUSB0'
DXL_ID = 1

# 포트 열기
port = PortHandler(DEVICENAME)
packet = PacketHandler(PROTOCOL_VERSION)
port.openPort()
port.setBaudRate(BAUDRATE)

# 토크 활성화
ADDR_TORQUE_ENABLE = 64
packet.write1ByteTxRx(port, DXL_ID, ADDR_TORQUE_ENABLE, 1)

# 목표 위치로 이동 (단위: 0~4095, 0~360도)
ADDR_GOAL_POSITION = 116
goal_position = 2048  # 중앙 (180도)
packet.write4ByteTxRx(port, DXL_ID, ADDR_GOAL_POSITION, goal_position)

# 현재 위치 읽기
ADDR_PRESENT_POSITION = 132
pos, _, _ = packet.read4ByteTxRx(port, DXL_ID, ADDR_PRESENT_POSITION)
print(f"현재 위치: {pos} (= {pos * 360 / 4096:.1f}°)")
```


### 4.5.4 실시간 시스템

로봇 제어에서 "실시간(real-time)"은 "빠른"이 아니라 "정해진 시간 안에 반드시 완료되는"을 의미한다. 1kHz 제어 루프라면, 매 1ms마다 센서 읽기 → 제어 계산 → 모터 명령 전송이 완료되어야 한다. 한 번이라도 지연되면 로봇이 불안정해질 수 있다.

**RTOS (Real-Time Operating System):**

| RTOS | 특징 | 용도 |
|------|------|------|
| FreeRTOS | 경량, 마이크로컨트롤러용, 무료 | STM32, ESP32 등 |
| Zephyr | 최신, 다양한 하드웨어 지원, Linux Foundation | IoT, 로봇 임베디드 |
| VxWorks | 상용, NASA도 사용 | 항공우주, 산업용 |

마이크로컨트롤러에서 직접 모터를 제어할 때는 RTOS를 쓴다. 태스크 우선순위를 설정하여 제어 루프가 다른 태스크에 밀리지 않도록 한다.

**PREEMPT_RT Linux:**

문제: ROS2는 Linux에서 돌아간다. 그런데 일반 Linux 커널은 실시간이 아니다. 스케줄러가 제어 스레드를 아무 때나 중단시킬 수 있고, 수 밀리초의 지연이 발생할 수 있다.

해결: PREEMPT_RT 패치를 적용한 Linux 커널. 커널 대부분의 코드 경로를 선점(preemptible)으로 만들어서 실시간에 가까운 성능을 제공한다.

설정 방법 (개략):
```bash
# 1. PREEMPT_RT 패치가 적용된 커널 설치 (Ubuntu 예시)
sudo apt install linux-image-rt-amd64   # Debian/Ubuntu

# 2. GRUB에서 RT 커널로 부팅 설정

# 3. 제어 스레드에 실시간 우선순위 부여
sudo chrt -f 99 ./my_robot_controller

# 4. CPU isolation (선택적이지만 권장)
#    /etc/default/grub에 isolcpus=2,3 추가
#    → CPU 2, 3을 일반 프로세스에서 격리
#    → 제어 스레드를 이 CPU에 고정(affinity)

# 5. 성능 확인
sudo cyclictest -m -p 99 -t 1 -n
# 최대 지연을 목표 제어 주기와 여유 시간에 대조한다
```

**제어 주기를 어떻게 정하는가:**

1kHz(1ms)는 torque·impedance control에서 자주 쓰이는 설계점이지만 보편 표준은 아니다. 필요한 주기는 폐루프 대역폭, 기계 공진, 센서와 actuator 지연, solver 시간, jitter 여유로 정한다. Nyquist의 2배는 aliasing을 피하기 위한 하한일 뿐 제어 성능을 보장하지 않으므로, 실제 설계에서는 목표 폐루프 대역폭보다 충분히 빠르게 sampling하고 주파수 응답과 지연 여유를 검증한다. CAN 대역폭도 모터 수만으로 정해지지 않는다. frame 크기, arbitration, bus load와 feedback rate를 합산해 계산해야 한다.

일부 가벼운 robot, 고속 충돌 대응, tactile control은 수 kHz 주기를 사용한다. 이때도 EtherCAT이나 FPGA가 항상 필수인 것은 아니며, 필요한 결정성·대역폭·I/O 구조에 맞춰 fieldbus, MCU, FPGA를 고른다.

> **추천 자료**
> - FreeRTOS 공식 문서: https://www.freertos.org/
> - PREEMPT_RT Wiki: https://wiki.linuxfoundation.org/realtime/start
> - Dynamixel SDK: https://github.com/ROBOTIS-GIT/DynamixelSDK
> - IgH EtherCAT Master (Linux용 오픈소스): https://etherlab.org/en/ethercat/
> - SOEM (Simple Open EtherCAT Master): https://github.com/OpenEtherCATsociety/SOEM

---

## 4.6 심화: Workspace Analysis와 최적 설계

기구학은 "주어진 로봇을 어떻게 움직이나"의 문제이기도 하지만, "어떤 로봇을 설계해야 하나"의 문제이기도 하다. 이 절은 설계 최적화와 관련된 고급 주제를 다룬다.


### 4.6.1 Reachable Workspace vs Dexterous Workspace

**Reachable workspace**: 끝단이 적어도 하나의 자세(orientation)로 도달할 수 있는 모든 점의 집합. "어디까지 손이 닿는가."

**Dexterous workspace**: 끝단이 임의의 자세로 도달할 수 있는 점의 집합. "어디서 자유롭게 움직일 수 있는가." 당연히 reachable workspace의 부분집합이고, 보통 훨씬 작다.

6-DOF 로봇의 경우 dexterous workspace는 상당히 제한적일 수 있다. 이 제약이 7-DOF 로봇이 등장한 이유 중 하나이다.

Workspace 분석은 Monte Carlo 방법으로 수행할 수 있다: 관절 공간을 무작위로 샘플링하고, FK로 끝단 위치를 계산하여 점구름(point cloud)을 만든다.

```python
import numpy as np
import roboticstoolbox as rtb

# Puma 560의 workspace 시각화 (Monte Carlo)
puma = rtb.models.DH.Puma560()
n_samples = 50000
positions = []

for _ in range(n_samples):
    # 각 관절의 범위 내에서 무작위 샘플링
    q = puma.random_q()
    T = puma.fkine(q)
    positions.append(T.t)  # [x, y, z]

positions = np.array(positions)

# 시각화 (matplotlib)
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
ax.scatter(positions[:, 0], positions[:, 1], positions[:, 2],
           s=0.1, alpha=0.1, c='blue')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title('Puma 560 Reachable Workspace')
plt.savefig('workspace.png', dpi=150)
```


### 4.6.2 Condition Number와 Isotropy

자코비안의 condition number (κ)는 로봇이 특정 자세에서 얼마나 "잘" 움직일 수 있는지의 지표이다.

```
κ(J) = σ_max / σ_min
```

σ_max, σ_min은 자코비안의 최대/최소 특이값(singular value)이다.

- κ = 1: 완벽한 등방성 (isotropic). 모든 방향으로 균일하게 움직인다. 실현 불가능하지만 이상적.
- κ → ∞: 특이점. 한 방향으로는 전혀 움직이지 못한다.

로봇 설계 시 작업 영역 전체에 걸쳐 condition number를 최소화하는 것이 목표가 될 수 있다. 이를 **kinematic optimization** 또는 **optimal design**이라 한다.

주의: 자코비안의 condition number를 계산할 때, 선속도(m/s)와 각속도(rad/s)의 단위가 다르므로 직접 비교하면 의미가 없다. 특성 길이(characteristic length)로 정규화하거나, 선속도와 각속도를 별도로 분석해야 한다. 이 문제는 로봇 기구학 최적화에서 오래된 논쟁거리이다.


### 4.6.3 Redundancy Resolution (7-DOF Arms)

7-DOF 로봇 팔 (Kinova Gen3, KUKA LBR iiwa, Franka Emika Panda 등)은 6-DOF 작업 공간에 비해 자유도가 1개 남는다. 이 여분의 자유도를 **kinematic redundancy**라 한다.

같은 끝단 자세를 유지하면서 팔 전체의 형태(configuration)를 바꿀 수 있다. 사람 팔이 주먹의 위치를 고정한 채 팔꿈치를 올리거나 내리는 것과 같다.

이 자유도를 활용하는 전략:
1. 특이점 회피: 자코비안의 manipulability를 최대화하는 방향으로 여분 자유도 사용
2. 관절 제한 회피: 관절이 한계에 가까워지면 여분 자유도로 중앙 위치 복귀
3. 장애물 회피: 팔꿈치가 장애물과 충돌하지 않도록 형태 조정
4. 에너지 최적화: 토크를 최소화하는 자세 선택

수학적으로, 여분 자유도는 자코비안의 null space에 해당한다:

```
q̇ = J† * ẋ + (I - J† * J) * q̇_0
```

첫째 항은 끝단 속도를 달성하는 최소 norm 관절 속도이다. 둘째 항 (I - J†J)은 null space projector로, 끝단 속도에 영향을 주지 않으면서 관절을 움직인다. q̇_0는 2차 목적(예: manipulability 최대화)의 그래디언트이다.

```python
def redundancy_resolution(J, x_dot, q, q_center, k_null=0.5):
    """
    7-DOF 로봇의 redundancy resolution.

    Args:
        J: 6x7 자코비안
        x_dot: 6x1 원하는 끝단 속도
        q: 7x1 현재 관절 각도
        q_center: 7x1 관절 중앙값 (null space 목표)
        k_null: null space 게인
    Returns:
        q_dot: 7x1 관절 속도
    """
    # Damped pseudo-inverse
    lam = 0.01
    J_pinv = J.T @ np.linalg.inv(J @ J.T + lam**2 * np.eye(6))

    # 1차 목적: 끝단 속도 추종
    q_dot_primary = J_pinv @ x_dot

    # 2차 목적: 관절 중앙으로 복귀 (null space)
    null_projector = np.eye(7) - J_pinv @ J
    q_dot_null = null_projector @ (k_null * (q_center - q))

    return q_dot_primary + q_dot_null
```

> **추천 자료**
> - Siciliano et al., *Robotics: Modelling, Planning and Control*, Chapter 3.9 — Redundancy resolution 상세 설명
> - Nakamura, "Advanced Robotics: Redundancy and Optimization" (1991) — 고전
> - Dietrich et al., "An Overview of Null Space Projections for Redundant, Torque-Controlled Robots" (2015)
> - Franka Emika 연구 인터페이스: https://frankaemika.github.io/docs/

여기까지는 결정론적 기구학이었다. §4.7에서는 그 위에 확률을 얹는다.

---

## 4.7 심화: 확률적 운동 모델 (Probabilistic Motion Models)

### 4.7.1 도입: 결정론에서 확률로

§4.2 순기구학과 §4.3 역기구학은 결정론적이다. 관절 각도를 넣으면 끝단 위치 하나가 나오고, 끝단 위치를 넣으면 관절 각도 집합이 나온다. 입력에 대한 출력은 점 추정이다. 팔 끝이 어디 있는지 수학적으로 정확히 계산할 수 있는 매니퓰레이터의 세계다.

차륜 구동 모바일 로봇은 다르다. 명령한 속도대로 바퀴가 정확히 굴러가지 않는다. 슬립이 있고, 바퀴 마모로 실효 반지름이 변한다. 좌우 바퀴의 비대칭 마모가 직진 오차를 만들기도 한다. 결과적으로, 제어 명령 $u_t$를 내리더라도 다음 pose $x_t$는 하나의 점이 아니라 확률 분포이다. 이 분포를 정형화하는 것이 **확률적 운동 모델(probabilistic motion model)**이다.

상태는 평면 상의 pose, $x_t = (x, y, \theta)^T \in SE(2)$이다. 운동 모델은 이전 pose $x_{t-1}$과 제어 입력 $u_t$가 주어졌을 때 다음 pose의 조건부 확률 분포

$$p(x_t \mid u_t, x_{t-1})$$

를 정의한다. 이 분포를 표현하는 방법으로 두 가지가 있다.

**Velocity model**: 제어 입력이 선속도와 각속도 $u_t = (v, \omega)^T$로 주어진다. 계획 단계에서 사용할 수 있다. 실제 로봇의 명령 속도와 실제 속도 사이의 오차를 노이즈로 모델링한다.

**Odometry model**: 제어 입력이 휠 인코더에서 측정한 두 pose pair $u_t = (\bar{x}_{t-1}, \bar{x}_t)$로 주어진다. 사후(retrospective) 정보이므로 계획에는 쓸 수 없지만, 인코더가 직접 측정한 값이므로 velocity model보다 정확하다.

두 모델 각각에 대해 **폐쇄형 밀도 평가(closed-form density evaluation)**와 **샘플링(sampling)** 두 가지 사용 방법이 있다. 폐쇄형은 "이 가설 pose $x_t$가 얼마나 그럴듯한가"를 확률밀도 수치로 돌려준다. EKF·UKF의 prediction 단계에서 필요하다. 샘플링은 "다음 pose 하나를 생성하라"는 forward 시뮬레이션이다. Particle filter(MCL)가 이 형태를 직접 쓴다. 4개의 조합을 §4.7.2~§4.7.5에서 각각 다룬다.


### 4.7.2 Velocity Motion Model — 폐쇄형

**직관.** 노이즈가 없다면, 선속도 $v$와 각속도 $\omega$로 움직이는 로봇은 원형 호(circular arc)를 그린다. $\omega = 0$이면 직선이다. 노이즈가 있으면 실제로 그린 호는 명령값과 다르다. 폐쇄형 평가는 이 논리를 뒤집는다: 두 pose $x_{t-1}$과 $x_t$가 주어지면, 이 두 점을 잇는 원호의 회전 중심 $(x_c, y_c)$와 반지름 $r^*$를 역산하고, 그 호를 만들었을 가상의 속도 $(\hat{v}, \hat{\omega})$를 구한 다음, 명령 속도 $(v, \omega)$와의 차이를 노이즈 분포로 평가한다.

**수식.** 두 pose $x_{t-1} = (x, y, \theta)^T$와 가설 $x_t = (x', y', \theta')^T$가 주어졌을 때:

$$\mu = \frac{1}{2} \cdot \frac{(x - x')\cos\theta + (y - y')\sin\theta}{(y - y')\cos\theta - (x - x')\sin\theta}$$

$$x_c = \frac{x + x'}{2} + \mu(y - y'), \quad y_c = \frac{y + y'}{2} + \mu(x' - x)$$

$$r^* = \sqrt{(x - x_c)^2 + (y - y_c)^2}$$

$$\Delta\theta = \text{atan2}(y' - y_c,\ x' - x_c) - \text{atan2}(y - y_c,\ x - x_c)$$

$$\hat{v} = \frac{\Delta\theta \cdot r^*}{\Delta t}, \quad \hat{\omega} = \frac{\Delta\theta}{\Delta t}, \quad \hat{\gamma} = \frac{\theta' - \theta}{\Delta t} - \hat{\omega}$$

잡음 모델은 분산이 명령 크기에 비례하는 가산형이다. `prob(a, b)`의 두 번째 인자 $b$(분산)는 다음과 같이 결정된다:

$$b_v = \alpha_1|v| + \alpha_2|\omega|, \quad b_\omega = \alpha_3|v| + \alpha_4|\omega|, \quad b_\gamma = \alpha_5|v| + \alpha_6|\omega|$$

$b_v$는 선속도, $b_\omega$는 각속도의 노이즈 분산이다. $\hat{\gamma}$는 "최종 방향 보정" 항이다. $(v, \omega)$ 두 노이즈 변수만으로는 3D pose 공간 안의 2D 매니폴드 위에서만 가설 pose가 생성되는 *축퇴(degeneracy)* 문제가 생긴다. $\hat{\gamma}$를 추가하면 3D 지지(support)가 확보된다.

6개 파라미터의 물리적 의미: $\alpha_1, \alpha_2$는 선속도 노이즈의 분산 가중치, $\alpha_3, \alpha_4$는 각속도 노이즈, $\alpha_5, \alpha_6$는 최종 회전 노이즈. 분산이 명령 크기에 선형 비례하므로 빠를수록 더 불확실해지는 직관과 일치한다. 로봇마다 직선·원·8자 주행 데이터로 $\alpha_i$를 calibration해야 한다.

알고리즘 박스 (PR Table 5.1: `motion_model_velocity`).

```
Algorithm motion_model_velocity(x_t, u_t, x_{t-1}):
  # 입력: x_t=(x',y',θ'), u_t=(v,ω), x_{t-1}=(x,y,θ)
  # 출력: p(x_t | u_t, x_{t-1}) 확률밀도

  μ = 0.5 * ((x − x')cosθ + (y − y')sinθ) / ((y − y')cosθ − (x − x')sinθ)
  x* = (x + x')/2 + μ(y − y')
  y* = (y + y')/2 + μ(x' − x)
  r* = sqrt((x − x*)² + (y − y*)²)
  Δθ = atan2(y' − y*, x' − x*) − atan2(y − y*, x − x*)
  v̂ = Δθ·r*/Δt
  ω̂ = Δθ/Δt
  γ̂ = (θ' − θ)/Δt − ω̂

  p1 = prob(v − v̂,  α₁|v| + α₂|ω|)
  p2 = prob(ω − ω̂,  α₃|v| + α₄|ω|)
  p3 = prob(γ̂,       α₅|v| + α₆|ω|)

  return p1 · p2 · p3
```

`prob(a, b)`는 평균 0, 분산 $b$의 정규 분포 또는 삼각 분포의 밀도값이다.

같은 노이즈 파라미터로 pose를 직접 생성하는 것도 가능하다. 방향만 반대다.

### 4.7.3 Velocity Motion Model — 샘플링

폐쇄형은 "가설 pose가 얼마나 그럴듯한가"를 역산으로 평가했다. Sampling은 반대 방향이다. 노이즈를 먼저 뽑아 명령 속도를 perturb하고, perturbed 속도로 forward 시뮬레이션을 돌려 다음 pose 하나를 생성한다. Particle filter는 매 입자마다 이 샘플 하나가 필요하다. 구현도 폐쇄형보다 단순하다.

Perturbed 제어:

$$\hat{v} = v + \text{sample}(\alpha_1|v| + \alpha_2|\omega|)$$
$$\hat{\omega} = \omega + \text{sample}(\alpha_3|v| + \alpha_4|\omega|)$$
$$\hat{\gamma} = \text{sample}(\alpha_5|v| + \alpha_6|\omega|)$$

Forward 원호 적분:

$$x' = x - \frac{\hat{v}}{\hat{\omega}}\sin\theta + \frac{\hat{v}}{\hat{\omega}}\sin(\theta + \hat{\omega}\Delta t)$$
$$y' = y + \frac{\hat{v}}{\hat{\omega}}\cos\theta - \frac{\hat{v}}{\hat{\omega}}\cos(\theta + \hat{\omega}\Delta t)$$
$$\theta' = \theta + \hat{\omega}\Delta t + \hat{\gamma}\Delta t$$

주의: $|\hat{\omega}| < \epsilon$이면 위 식이 발산한다. 실제 구현에서는 직선 fallback $x' = x + \hat{v}\cos\theta\,\Delta t,\ y' = y + \hat{v}\sin\theta\,\Delta t$로 처리해야 한다.

`sample(b)`는 분산 $b$의 zero-mean 표본을 뽑는 함수이다. 정규 근사: $\frac{b}{6}\sum_{i=1}^{12}\text{rand}(-1,1)$ (중심극한정리 기반, 12개 균등 합).

알고리즘 박스 (PR Table 5.3: `sample_motion_model_velocity`).

```
Algorithm sample_motion_model_velocity(u_t, x_{t-1}):
  # 입력: u_t=(v,ω), x_{t-1}=(x,y,θ)
  # 출력: 샘플 x_t ~ p(x_t | u_t, x_{t-1})

  v̂ = v + sample(α₁|v| + α₂|ω|)
  ω̂ = ω + sample(α₃|v| + α₄|ω|)
  γ̂ = sample(α₅|v| + α₆|ω|)

  if |ω̂| < ε:   # 직선 fallback
    x' = x + v̂·cosθ·Δt
    y' = y + v̂·sinθ·Δt
  else:
    x' = x − (v̂/ω̂)sinθ + (v̂/ω̂)sin(θ + ω̂Δt)
    y' = y + (v̂/ω̂)cosθ − (v̂/ω̂)cos(θ + ω̂Δt)
  θ' = θ + ω̂Δt + γ̂Δt

  return (x', y', θ')ᵀ
```

**Closed-form vs Sampling 용도 차이.** Closed-form(`motion_model_velocity`)은 확률밀도 수치를 반환한다. EKF·UKF의 prediction 단계에서 자코비안과 함께 쓰인다. Sampling(`sample_motion_model_velocity`)은 pose 하나를 생성한다. Particle filter(MCL, §14.7)에서 각 입자를 propagate할 때 직접 호출된다. 두 알고리즘은 같은 노이즈 파라미터 $\alpha_1..\alpha_6$를 공유하지만, 방향이 반대이다: closed-form은 *가설 pose를 평가*하고, sampling은 *다음 pose를 생성*한다.

```python
import numpy as np

def sample_normal(b):
    """분산 b의 zero-mean 정규 근사 표본 (12개 균등 합)."""
    return (b / 6.0) * sum(np.random.uniform(-1, 1) for _ in range(12))

def sample_motion_model_velocity(v, omega, x, y, theta, dt,
                                  alpha, eps=1e-6):
    """
    Velocity motion model sampling.
    alpha: [α₁, α₂, α₃, α₄, α₅, α₆]
    """
    v_hat   = v     + sample_normal(alpha[0]*abs(v) + alpha[1]*abs(omega))
    w_hat   = omega + sample_normal(alpha[2]*abs(v) + alpha[3]*abs(omega))
    g_hat   =         sample_normal(alpha[4]*abs(v) + alpha[5]*abs(omega))

    if abs(w_hat) < eps:
        x_new = x + v_hat * np.cos(theta) * dt
        y_new = y + v_hat * np.sin(theta) * dt
    else:
        r = v_hat / w_hat
        x_new = x - r * np.sin(theta) + r * np.sin(theta + w_hat * dt)
        y_new = y + r * np.cos(theta) - r * np.cos(theta + w_hat * dt)
    theta_new = theta + w_hat * dt + g_hat * dt

    return x_new, y_new, theta_new
```

velocity model 두 버전은 같은 노이즈 파라미터 $\alpha_1..\alpha_6$를 공유한다. 제어 입력이 명령 속도 $(v, \omega)$라는 가정 자체는 두 버전 모두 동일하다. 이 가정을 바꾸면 두 번째 모델 계열이 나온다.

### 4.7.4 Odometry Motion Model — 폐쇄형

**직관.** Velocity model은 명령 속도로부터 모션을 추정한다. Odometry model은 반대로, 실제 바퀴 회전을 인코더로 측정한 두 pose pair $u_t = (\bar{x}_{t-1}, \bar{x}_t)$를 control처럼 다룬다. 이 두 pose의 상대 운동을 세 파라미터 $(\delta_{\text{rot1}}, \delta_{\text{trans}}, \delta_{\text{rot2}})$로 분해한다. 목적지 방향으로 먼저 회전한 다음 직진하고, 도착 후 최종 방향 보정이다. 이 분해는 임의의 평면 운동을 항상 표현할 수 있다.

실제 odometry measurement는 엄밀히는 센서 측정값이지만, 여기서는 control처럼 다룬다. 진짜 측정 모델로 취급하면 상태 공간에 속도를 추가해야 해서 차원이 커진다. 실용적 단순화이다.

**수식.** Odometry 측정 $u_t = (\bar{x}_{t-1}, \bar{x}_t)$에서 상대 운동 추출:

$$\delta_{\text{rot1}} = \text{atan2}(\bar{y}' - \bar{y},\ \bar{x}' - \bar{x}) - \bar{\theta}$$
$$\delta_{\text{trans}} = \sqrt{(\bar{x} - \bar{x}')^2 + (\bar{y} - \bar{y}')^2}$$
$$\delta_{\text{rot2}} = \bar{\theta}' - \bar{\theta} - \delta_{\text{rot1}}$$

잡음 모델 (파라미터 4개 $\alpha_1..\alpha_4$). `prob()`의 분산 인자는 가설 pose에서 역산한 $(\hat\delta_{\text{rot1}}, \hat\delta_{\text{trans}}, \hat\delta_{\text{rot2}})$에 의존한다:

$$b_{\text{rot1}} = \alpha_1|\hat\delta_{\text{rot1}}| + \alpha_2|\hat\delta_{\text{trans}}|$$
$$b_{\text{trans}} = \alpha_3|\hat\delta_{\text{trans}}| + \alpha_4(|\hat\delta_{\text{rot1}}| + |\hat\delta_{\text{rot2}}|)$$
$$b_{\text{rot2}} = \alpha_1|\hat\delta_{\text{rot2}}| + \alpha_2|\hat\delta_{\text{trans}}|$$

$\alpha_1$: 회전이 회전을 흔드는 정도(회전 슬립), $\alpha_2$: 직진이 회전을 흔드는 정도, $\alpha_3$: 직진의 자체 분산, $\alpha_4$: 회전이 직진을 흔드는 정도. Velocity model의 $\alpha_5, \alpha_6$에 해당하는 "최종 회전" trick이 필요 없다. 3개의 독립 노이즈 변수가 자연스럽게 3D 지지를 확보한다.

주의: 각도 차는 반드시 $[-\pi, \pi]$로 wrap해야 한다. 미준수 시 분포가 발산하는 흔한 버그이다.

알고리즘 박스 (PR Table 5.5: `motion_model_odometry`).

```
Algorithm motion_model_odometry(x_t, u_t, x_{t-1}):
  # 입력: x_t=(x',y',θ'), u_t=(x̄_{t-1}, x̄_t), x_{t-1}=(x,y,θ)
  # 출력: p(x_t | u_t, x_{t-1}) 확률밀도

  # odometry 측정에서 (δ_rot1, δ_trans, δ_rot2) 추출
  δ_rot1  = atan2(ȳ' − ȳ, x̄' − x̄) − θ̄
  δ_trans = sqrt((x̄ − x̄')² + (ȳ − ȳ')²)
  δ_rot2  = θ̄' − θ̄ − δ_rot1

  # 가설 pose 쌍에서 같은 분해 (역모델)
  δ̂_rot1  = atan2(y' − y, x' − x) − θ
  δ̂_trans = sqrt((x − x')² + (y − y')²)
  δ̂_rot2  = θ' − θ − δ̂_rot1

  # 세 파라미터 차이를 독립 노이즈로 평가
  p1 = prob(δ_rot1  − δ̂_rot1,  α₁|δ̂_rot1|  + α₂|δ̂_trans|)
  p2 = prob(δ_trans − δ̂_trans, α₃|δ̂_trans| + α₄(|δ̂_rot1| + |δ̂_rot2|))
  p3 = prob(δ_rot2  − δ̂_rot2,  α₁|δ̂_rot2|  + α₂|δ̂_trans|)

  return p1 · p2 · p3
```

세 파라미터 $(\delta_{\text{rot1}}, \delta_{\text{trans}}, \delta_{\text{rot2}})$는 각각 독립 노이즈 변수로 다루어지므로, EKF·UKF의 prediction 단계에서 가설 pose 평가에 바로 쓸 수 있다.

### 4.7.5 Odometry Model — Particle Filter에서의 Sampling

Odometry 폐쇄형은 역모델을 써서 가설 pose를 평가했다. Sampling은 반대 방향이다. Odometry에서 추출한 $(\delta_{\text{rot1}}, \delta_{\text{trans}}, \delta_{\text{rot2}})$에 노이즈를 가산하고, perturbed 값으로 forward 합성하여 새 pose를 생성한다. Inverse 모델이 전혀 필요 없어 폐쇄형보다 구현이 훨씬 단순하다.

**수식.** Forward 합성 (PR 식 5.40):

$$\begin{pmatrix}x'\\y'\\\theta'\end{pmatrix} = \begin{pmatrix}x\\y\\\theta\end{pmatrix} + \begin{pmatrix}\hat{\delta}_{\text{trans}}\cos(\theta + \hat{\delta}_{\text{rot1}})\\\hat{\delta}_{\text{trans}}\sin(\theta + \hat{\delta}_{\text{rot1}})\\\hat{\delta}_{\text{rot1}} + \hat{\delta}_{\text{rot2}}\end{pmatrix}$$

이것은 원호가 아닌 *직선 + 두 회전*으로 운동을 근사한다. 짧은 $\Delta t$에서 원호의 1차 근사이다. Velocity sampling과 달리 $\omega \to 0$ 분기 처리가 필요 없다는 것이 장점이다.

알고리즘 박스 (PR Table 5.6: `sample_motion_model_odometry`).

```
Algorithm sample_motion_model_odometry(u_t, x_{t-1}):
  # 입력: u_t=(x̄_{t-1}, x̄_t), x_{t-1}=(x,y,θ)
  # 출력: 샘플 x_t ~ p(x_t | u_t, x_{t-1})

  # odometry에서 상대 운동 추출
  δ_rot1  = atan2(ȳ' − ȳ, x̄' − x̄) − θ̄
  δ_trans = sqrt((x̄ − x̄')² + (ȳ − ȳ')²)
  δ_rot2  = θ̄' − θ̄ − δ_rot1

  # 노이즈로 perturb
  δ̂_rot1  = δ_rot1  − sample(α₁|δ_rot1|  + α₂|δ_trans|)
  δ̂_trans = δ_trans − sample(α₃|δ_trans| + α₄(|δ_rot1| + |δ_rot2|))
  δ̂_rot2  = δ_rot2  − sample(α₁|δ_rot2|  + α₂|δ_trans|)

  # forward 합성
  x' = x + δ̂_trans · cos(θ + δ̂_rot1)
  y' = y + δ̂_trans · sin(θ + δ̂_rot1)
  θ' = θ + δ̂_rot1 + δ̂_rot2

  return (x', y', θ')ᵀ
```

ROS2 Nav2의 `nav2_amcl`은 이 형태를 `differential` motion model로 구현한다. 차륜 AMR localization의 직계 응용이다.

```python
import numpy as np

def sample_motion_model_odometry(bar_x_prev, bar_x_curr, x, y, theta,
                                  alpha):
    """
    Odometry motion model sampling.
    bar_x_prev, bar_x_curr: odometry pose pair (x̄,ȳ,θ̄)
    alpha: [α₁, α₂, α₃, α₄]
    """
    bx,  by,  bt  = bar_x_prev
    bx_, by_, bt_ = bar_x_curr

    d_rot1  = np.arctan2(by_ - by, bx_ - bx) - bt
    d_trans = np.sqrt((bx - bx_)**2 + (by - by_)**2)
    d_rot2  = bt_ - bt - d_rot1

    def sample_normal(b):
        return (b / 6.0) * sum(np.random.uniform(-1, 1) for _ in range(12))

    dh_rot1  = d_rot1  - sample_normal(alpha[0]*abs(d_rot1)  + alpha[1]*abs(d_trans))
    dh_trans = d_trans - sample_normal(alpha[2]*abs(d_trans) + alpha[3]*(abs(d_rot1) + abs(d_rot2)))
    dh_rot2  = d_rot2  - sample_normal(alpha[0]*abs(d_rot2)  + alpha[1]*abs(d_trans))

    x_new     = x + dh_trans * np.cos(theta + dh_rot1)
    y_new     = y + dh_trans * np.sin(theta + dh_rot1)
    theta_new = theta + dh_rot1 + dh_rot2

    return x_new, y_new, theta_new
```

<!-- DEMO: probabilistic_motion_model.html -->

지금까지 네 알고리즘은 모두 지도 없이 운동만 모델링했다. localization 문제에서는 지도 $m$이 함께 있다.

### 4.7.6 Motion + Map: 지도 조건부 운동 모델

지금까지의 모델은 지도 정보를 무시했다. 그런데 localization에서는 지도 $m$이 있다. 이를 이용하면 물리적으로 불가능한 pose를 걸러낼 수 있다.

**수식.** 지도를 조건에 포함한 전이 분포를 정확히 계산하면 까다롭다. 실용적인 근사 분해:

$$p(x_t \mid u_t, x_{t-1}, m) \propto p(x_t \mid u_t, x_{t-1}) \cdot p(x_t \mid m)$$

앞 항 $p(x_t \mid u_t, x_{t-1})$은 §4.7.2~§4.7.5의 운동 모델이다. 뒷 항 $p(x_t \mid m)$은 지도 조건부 확률로, occupancy grid에서 $x_t$가 자유 공간(free cell)이면 1에 가깝고, 벽이나 점유 공간이면 0에 가깝다.

**효과.** Particle filter에서 입자가 벽을 통과하는 현상이 방지된다. 샘플링 후 새 pose를 지도에 조회하여 점유 공간이면 해당 입자의 weight를 0(또는 매우 낮게)으로 설정하는 것이 가장 단순한 구현이다. 정확히는 $p(x_t \mid m)$이 likelihood가 아니라 prior로 작용하는 형태이므로, 이 근사는 운동 모델과 지도 정보가 독립이라고 가정한다는 한계가 있다.

실제 구현에서 occupancy grid는 자유 공간 외에도 unknown 영역을 갖는다. Unknown 영역에 대해 $p(x_t \mid m)$을 어떻게 설정하느냐(1로 보느냐, 중간값으로 보느냐)는 localization 성능에 영향을 준다. ROS2 Nav2의 기본 설정은 unknown 영역을 free로 취급한다.

이 분해의 수학적 근거는 §3.3(베이즈 정리와 조건부 독립, Ch.3 참조)에 있다. 운동 모델과 지도 prior가 독립이라는 가정이 성립할 때만 이 곱 분해가 엄밀하다.

### 4.7.7 무엇이 살아남았나

Velocity model과 Odometry model의 sampling formulation은 차륜 로봇 particle-filter localization의 motion prior를 설명한다. ROS2 Nav2 `nav2_amcl`의 `differential` motion model은 이 가운데 odometry 기반 formulation에 해당한다. 실제 입자 수와 update rate는 지도 크기, 센서 update, CPU, beam 수와 오차 파라미터에 맞춰 측정해 정한다.

휴머노이드, 드론, legged robot은 차체 속도 $(v, \omega)$로 운동을 기술하기 어렵다. 발이 있으면 슬립 모델 자체가 다르고, 드론은 SE(2)가 아닌 SE(3) 위에서 움직인다. 이 플랫폼에서는 IMU preintegration이 motion prior를 제공한다(§14.10). 확률적 운동 모델의 형식적 프레임워크 $p(x_t \mid u_t, x_{t-1})$는 같지만, 내용이 완전히 다르다.

여기서 다룬 모델은 모두 SE(2) 한정이다. Holonomic robot(Mecanum 바퀴)이나 차량 dynamics(횡활 포함)처럼 구동 방식이 다른 경우에는 별도의 모델이 필요하다.

이 운동 모델의 직접 응용은 §14.7 Monte Carlo Localization(MCL)이다. Particle filter의 prediction 단계에서 `sample_motion_model_odometry`가 호출된다(Ch.14 참조). §14.10 IMU preintegration에서는 차륜 odometry 모델과 IMU 모델의 차이를 비교할 수 있다. $p(x_t \mid u_t, x_{t-1})$ 형식 자체는 §3.10·§3.11의 가우시안 필터·비모수 필터에서 prediction 항으로 그대로 쓰인다(Ch.3 참조).

---

결정론적 FK/IK는 "입력 → 출력 한 점"이다. 확률적 운동 모델은 "입력 → 출력 분포"이다. 두 모델(Velocity, Odometry)과 두 사용 방식(밀도 평가, 샘플링)의 조합 4개가 실제 localization 시스템을 구성하는 기본 단위다. Odometry model은 인코더 직접 측정이므로 계획에 쓸 수 없지만 정확도가 높고, Velocity model은 사전 계획에 쓸 수 있지만 실제 슬립을 반영하지 못한다. Sampling은 particle filter에서, closed-form은 EKF/UKF에서 각각의 역할이 있다.

한 가지 물음을 남긴다. 여기서 다룬 모든 모델은 바퀴가 미끄러지지 않는다는 운동학적 제약 위에 노이즈를 얹는 구조다. 진흙탕이나 경사 주행처럼 그 제약 자체가 무너지는 환경에서 $\alpha_i$ calibration은 어느 정도까지 보상할 수 있을까.

---

## 4.8 추천 자료

기구학과 메카트로닉스를 진지하게 공부하려면 교재 하나를 처음부터 끝까지 풀어보는 것이 가장 효과적이다.

**교재:**

- Craig, "Introduction to Robotics: Mechanics and Planning" — DH 파라미터와 기구학을 다루며 Modified DH convention을 사용한다. 선수지식과 강의 구성에 맞는지는 목차와 예제를 보고 판단한다.

- Lynch & Park, "Modern Robotics: Mechanics, Planning, and Control" — PoE 기반. 무료 PDF와 Coursera 강의를 제공하여 접근성이 뛰어나다. 수학적으로 더 깔끔하지만 처음 보면 어렵다. https://modernrobotics.org

- Corke, "Robotics, Vision and Control" — MATLAB/Python 코드와 함께 기구학을 실습할 수 있다. robotics-toolbox-python은 이 책의 동반 라이브러리이다. 3판은 Python 기반. https://petercorke.com/rvc/

- Siciliano et al., "Robotics: Modelling, Planning and Control" — 기구학, 동역학, 제어를 한 권에서 폭넓게 다루는 대학원 교과서

**온라인 강의:**

- Modern Robotics, Coursera (Northwestern University): https://www.coursera.org/specializations/modernrobotics
- Introduction to Robotics, Stanford CS223A (Khatib): https://see.stanford.edu/Course/CS223A

**소프트웨어/라이브러리:**

- robotics-toolbox-python: https://github.com/petercorke/robotics-toolbox-python
- Pinocchio (고속 동역학, 미분 가능 기구학): https://github.com/stack-of-tasks/pinocchio
- MoveIt2 (ROS2 모션 플래닝): https://moveit.picknik.ai/
- Drake (시뮬레이션 + 최적화 + 제어): https://drake.mit.edu/

---

## 기술 흐름

```
1955 ── DH 파라미터 제안 (Denavit & Hartenberg)
1969 ── Stanford Arm (초기의 전기식 컴퓨터 제어 로봇 팔)
1985 ── Product of Exponentials 정형화
1998 ── Harmonic Drive 로봇 적용 확산
2019 ── MIT Mini Cheetah: QDD 액추에이터
2019 ── MoveIt2 (ROS2 기반 모션 플래닝 프레임워크)
2023 ── ALOHA: 저비용 양팔 텔레오퍼레이션 플랫폼
2024 ── SO-ARM100: 공개 BOM과 조립 문서를 제공한 오픈소스 5축 로봇 팔
```

---

*기구학 위에 힘과 질량을 얹으면 동역학(Dynamics)과 제어(Control)로 넘어간다. 관절 각도를 "원하는 값으로 보내는" 것이 아니라, "원하는 토크를 가하는" 관점으로 바뀐다.*
