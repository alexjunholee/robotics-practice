# Ch.4 — 기구학 & 메카트로닉스 (Kinematics & Mechatronics)


로봇 팔 하나를 책상 위에 올려놓았다고 하자. 모터 6개에 각각 어떤 각도를 줘야 손끝이 커피잔에 닿는가? 이 질문에 답하는 학문이 기구학이다. 그리고 그 모터를 실제로 돌리고, 센서를 읽고, 제어 루프를 1kHz로 돌리는 현실의 문제가 메카트로닉스이다.

수학부터 실제 하드웨어 선정과 통신 프로토콜까지 다룬다. 수식이 좀 나오지만, 목적은 "로봇을 실제로 움직이는 것"이다.

---

## 4.1 왜 기구학을 배우는가

로봇 매니퓰레이터는 여러 개의 관절(joint)과 링크(link)로 구성된다. 우리가 원하는 것은 끝단(end-effector)의 위치와 자세(pose)이다. 하지만 우리가 직접 제어하는 것은 각 관절의 각도(또는 변위)이다.

이 둘 사이의 관계를 수학적으로 기술하는 것이 **기구학(Kinematics)**이다.

- **순기구학 (Forward Kinematics, FK)**: 관절 각도 → 끝단 위치/자세
- **역기구학 (Inverse Kinematics, IK)**: 끝단 위치/자세 → 관절 각도

동역학(Dynamics)과 다르다. 기구학은 힘과 질량을 고려하지 않는다. "어디에 있는가"의 문제이지, "어떤 힘이 필요한가"의 문제가 아니다. 동역학은 다음 장에서 다룬다.

기구학을 모르면 다음 상황에서 막힌다:
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

여기서 R은 3×3 회전 행렬, p는 3×1 위치 벡터이다. 이 행렬 하나로 강체의 위치와 자세를 동시에 표현할 수 있고, 여러 변환을 행렬 곱으로 연쇄(chain)할 수 있다는 점이 핵심이다.

두 프레임 사이의 변환 T_01이 있고, 또 다른 변환 T_12가 있으면:

```
T_02 = T_01 * T_12
```

이것이 순기구학의 본질이다. 베이스에서 끝단까지 각 관절의 변환을 순서대로 곱하면 된다.


### 4.2.2 DH Parameters (Denavit-Hartenberg)

1955년 Denavit와 Hartenberg가 제안한 방법이다. 70년이 지났지만 여전히 산업계의 표준이다. 4개의 파라미터로 인접한 두 링크 사이의 관계를 정의한다:

| 파라미터 | 의미 |
|---------|------|
| **a_i** (link length) | x_i 축을 따른 z_{i-1}에서 z_i까지의 거리 |
| **α_i** (link twist) | z_{i-1}에서 z_i까지 x_i 축 기준 회전 각도 |
| **d_i** (link offset) | z_{i-1} 축을 따른 x_{i-1}에서 x_i까지의 거리 |
| **θ_i** (joint angle) | x_{i-1}에서 x_i까지 z_{i-1} 축 기준 회전 각도 |

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

이게 지나치게 단순해 보인다면 정상이다. 실제 6축 로봇 팔의 FK도 원리는 같다. 4×4 행렬을 6번 곱하면 될 뿐이다.


### 4.2.4 Product of Exponentials (PoE)

DH 파라미터의 대안으로, Lie group/Lie algebra에 기반한 PoE (Product of Exponentials) 방법이 있다. Lynch & Park의 "Modern Robotics"에서 채택한 방법이다.

핵심 아이디어: 각 관절을 twist(나선 운동)로 표현하고, 행렬 지수(matrix exponential)를 통해 변환을 계산한다.

```
T(θ) = e^{[S_1]θ_1} * e^{[S_2]θ_2} * ... * e^{[S_n]θ_n} * M
```

여기서:
- S_i는 i번째 관절의 screw axis (6×1 벡터)
- [S_i]는 S_i의 4×4 skew-symmetric matrix 표현 (se(3) 원소)
- M은 모든 관절이 영 위치(home configuration)일 때의 끝단 자세
- θ_i는 관절 변수

**DH vs PoE 비교:**

| 항목 | DH | PoE |
|------|-----|-----|
| 프레임 부착 | 각 링크에 프레임 필요 | 기준 프레임과 끝단 프레임만 필요 |
| Convention 혼동 | standard vs modified 주의 | 없음 (space form vs body form 구분은 있음) |
| 수학적 기반 | 행렬 곱 | Lie group, 행렬 지수 |
| 특이점 분석 | 별도 처리 필요 | 자연스럽게 통합 |
| 산업계 채택 | 매우 높음 | 학계 중심, 점점 확산 |
| 교재 | Craig, Siciliano | Lynch & Park |

실무적 조언: DH 파라미터는 반드시 알아야 한다. URDF(로봇 기술 파일)에 들어가는 파라미터가 결국 DH 기반이고, 산업용 로봇 매뉴얼은 모두 DH 테이블을 제공한다. PoE는 이론적으로 더 깔끔하고 연구에서 선호되지만, 현장에서 DH를 모르면 곤란하다. 둘 다 익혀라.

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
> - Lynch & Park, *Modern Robotics*, Chapter 4 — PoE 설명이 가장 잘 되어 있는 교재. 무료 PDF와 Coursera 강의 제공: https://modernrobotics.org
> - Craig, *Introduction to Robotics*, Chapter 3 — DH 파라미터의 정석. Modified DH convention 사용.
> - Peter Corke, *Robotics, Vision and Control* — Python 코드와 함께 FK를 실습할 수 있다: https://github.com/petercorke/robotics-toolbox-python

---

## 4.3 역기구학 (Inverse Kinematics)

FK는 쉽다. 행렬 곱이면 된다. 문제는 IK이다.

"끝단을 (x, y, z)에 놓고 싶은데, 관절 각도를 각각 얼마로 해야 하는가?"

이 문제가 어려운 이유:
1. **비선형 방정식** — 삼각함수가 얽혀 있다
2. **다중 해** — 같은 끝단 위치에 도달하는 관절 각도 조합이 여러 개일 수 있다 (elbow-up, elbow-down 등)
3. **해가 없을 수도 있다** — workspace 밖의 점은 도달 불가
4. **무한히 많은 해** — 자유도가 남으면 (redundant manipulator) 해가 무한대


### 4.3.1 Analytical IK (해석적 방법)

닫힌 형태(closed-form)의 해를 구하는 방법이다. 가능한 경우 가장 빠르고 정확하다.

**2-link planar arm의 IK:**

목표 위치 (x, y)가 주어졌을 때:

```
cos(θ_2) = (x² + y² - L1² - L2²) / (2 * L1 * L2)
θ_2 = atan2(±√(1 - cos²(θ_2)), cos(θ_2))

θ_1 = atan2(y, x) - atan2(L2*sin(θ_2), L1 + L2*cos(θ_2))
```

±에서 보듯이 해가 두 개다 (elbow-up, elbow-down). 이것이 IK의 본질적 어려움이다.

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

자코비안의 rank가 부족해지는 관절 배치를 특이점(singularity)이라 한다. 특이점에서는:

1. **특정 방향으로 끝단을 움직일 수 없다** — 자유도 상실
2. **미소 이동에 관절 속도가 무한대** — 실제 모터는 이를 따라갈 수 없다
3. **IK 해가 불연속** — 경로 추종 시 갑작스러운 관절 점프

2-link arm의 특이점은 간단하다: θ_2 = 0 (팔이 완전히 펴진 경우) 또는 θ_2 = π (완전히 접힌 경우). 이때 끝단은 반지름 방향으로만 움직일 수 있고, 접선 방향 속도는 낼 수 없다.

6축 로봇의 대표적 특이점:
- **Wrist singularity**: 축 4와 6이 정렬됨 (q5 ≈ 0)
- **Shoulder singularity**: 끝단이 축 1 위에 위치
- **Elbow singularity**: 팔이 완전히 펴짐

실무 대처법:
- 특이점 근처를 피하는 경로 계획
- DLS 방법으로 특이점 통과 시 속도 제한
- Redundancy(여분의 자유도) 활용


### 4.3.4 IK 솔버들

직접 IK를 구현할 일은 드물다. 검증된 솔버를 사용하는 것이 현명하다.

| 솔버 | 방법 | 특징 |
|------|------|------|
| **KDL** | Numerical (Newton-Raphson) | ROS 기본, 느림, 특이점 취약 |
| **IKFast** (OpenRAVE) | Analytical (코드 생성) | 특정 구조에 대해 C++ 코드 자동 생성. 빠름 |
| **TRAC-IK** | KDL + SQP 듀얼 | KDL보다 성공률 높음, ROS 패키지 존재 |
| **MoveIt2 IK** | 위 솔버들을 통합 | ROS2 생태계, 충돌 회피 통합 |
| **pinocchio** | PoE 기반 | 현대적, 빠름, 미분 가능 (differentiable) |

```python
# TRAC-IK가 KDL보다 나은 이유: 시간 내 해를 찾을 확률
# 벤치마크 결과 (Beeson & Ames, 2015):
#   KDL:     해 성공률 ~50-70% (시간 제한 5ms 기준)
#   TRAC-IK: 해 성공률 ~95-99% (동일 조건)
```

> **추천 자료**
> - Beeson & Ames, "TRAC-IK: An Open-Source Library for Improved Solving of Generic Inverse Kinematics" (2015): https://traclabs.com/projects/trac-ik/
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


### 4.4.2 힘/토크 관계 (Duality)

자코비안의 전치(transpose)는 끝단 힘을 관절 토크로 매핑한다:

```
τ = J^T(q) * F
```

여기서 τ는 관절 토크, F는 끝단에 작용하는 힘/모멘트이다.

이것이 **정역학적 이중성(static duality)**이다. 속도와 힘이 자코비안과 그 전치를 통해 쌍대 관계를 이룬다. 파워 보존 원리에서 자연스럽게 유도된다:

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
> - Siciliano et al., *Robotics: Modelling, Planning and Control*, Chapter 3 — 자코비안의 가장 포괄적인 설명
> - Corke, *Robotics, Vision and Control*, Chapter 8 — 코드 예제와 시각화 포함: https://petercorke.com/rvc/
> - robotics-toolbox-python 문서: https://github.com/petercorke/robotics-toolbox-python

---

## 4.5 메카트로닉스 기초

수학은 여기까지다. 이제 현실로 돌아오자. 관절 각도를 "정한다"고 해서 로봇이 움직이는 것이 아니다. 모터가 있어야 하고, 센서가 있어야 하고, 그 사이를 연결하는 전자 회로와 통신이 있어야 한다. 이것이 메카트로닉스이다.


### 4.5.1 액추에이터

**DC 모터:**
가장 기본적인 액추에이터. 전압을 가하면 회전한다. 토크는 전류에 비례하고 (τ = K_t * i), 역기전력은 속도에 비례한다 (V_emf = K_e * ω). 제어가 쉽고 가격이 저렴하지만, 브러시 마모가 있다.

**BLDC (Brushless DC) 모터:**
브러시 없이 전자적으로 전류를 전환한다. 수명이 길고, 토크 밀도가 높고, 효율이 좋다. 현대 로봇의 표준이다. FOC(Field-Oriented Control)로 제어하면 토크 리플(ripple)을 최소화할 수 있다.

**서보 모터 (Dynamixel 시리즈):**
모터 + 감속기 + 엔코더 + 컨트롤러를 일체형으로 묶은 제품이다. Robotis의 Dynamixel 시리즈가 연구용으로 가장 널리 쓰인다.

| 모델 | 토크 (Nm) | 통신 | 용도 |
|------|-----------|------|------|
| XL330 | 0.5 | TTL | 소형 그리퍼, SO-ARM100 등 |
| XM540 | 10.0 | RS-485 | 중형 로봇 팔 |
| PH54  | 44.7 | RS-485 | 대형 매니퓰레이터, 모바일 로봇 |

Dynamixel의 장점은 데이지 체인 연결, 위치/속도/토크 제어 모드, PID 게인 조절, 가격 대비 성능이다. 단점은 통신 속도 한계이고, 고급 제어를 하려면 커스텀 펌웨어가 필요한 경우가 있다.

**Quasi-Direct Drive (QDD):**

MIT Mini Cheetah(2019)로 주목받은 방식이다. 핵심 아이디어는 단순하다: **감속비를 낮추는 것.**

일반적인 로봇 관절: 감속비 100:1 이상 (harmonic drive)
QDD: 감속비 6:1 ~ 10:1 (유성기어 또는 belt)

낮은 감속비의 장점:
- **높은 백드라이버빌리티(backdrivability)**: 외력이 가해졌을 때 관절이 자연스럽게 따라간다. 충돌 시 안전하고, 힘 제어가 용이하다.
- **높은 투명도(transparency)**: 토크 센서 없이도 모터 전류만으로 끝단 힘을 추정할 수 있다.
- **높은 대역폭**: 감속기의 마찰과 탄성이 적어 빠른 토크 응답이 가능하다.

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
#   → 끝단에서 느끼는 관성이 매우 크다
#   → 정밀한 힘 제어가 어렵다
#
# QDD (감속비 8:1):
#   반사 관성 = 8² × 0.01 = 0.64 kg·m²
#   → 15배 이상 가볍다
#   → 힘 제어가 훨씬 쉽다
```


**감속기 종류:**

| 종류 | 감속비 | 백래시 | 효율 | 가격 | 용도 |
|------|--------|--------|------|------|------|
| Planetary | 3~100:1 | 중간 | 85-95% | 저렴 | 범용, QDD에 적합 |
| Harmonic Drive | 30~320:1 | 매우 낮음 | 65-85% | 비쌈 | 산업용 로봇, 정밀 |
| Cycloidal | 6~120:1 | 낮음 | 85-93% | 중간 | 최근 대안으로 부상 |

**액추에이터 선정 기준:**

로봇 관절의 액추에이터를 선정할 때 고려해야 할 것들:

1. **필요 토크 (torque)**: 정적 토크 (자세 유지) + 동적 토크 (가속). 안전 계수 2~3배.
2. **필요 속도 (speed)**: 관절의 최대 각속도. 감속비를 고려하여 모터 RPM 결정.
3. **백드라이버빌리티**: 협동 로봇이나 힘 제어가 필요하면 QDD, 아니면 harmonic drive.
4. **크기와 무게**: 로봇 링크에 장착해야 하므로 물리적 제약 존재.
5. **열 (thermal)**: 연속 토크 사양 확인. 피크 토크는 짧은 시간만 가능.

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

*Absolute encoder*: 현재 위치를 절대값으로 출력한다. 전원을 켜자마자 위치를 안다. Multi-turn absolute encoder는 여러 바퀴를 기억한다. 가격이 비싸지만 homing 불필요. 산업용 로봇에서 표준.

```
분해능 계산 예시:
  Incremental encoder, 4096 PPR, quadrature decoding (x4)
  → 분해능 = 360° / (4096 × 4) = 0.022° ≈ 0.38 mrad
  → 감속비 100:1 관절 → 출력 분해능 0.0038 mrad
```

**토크 센서:**

관절 토크 또는 끝단 힘을 직접 측정한다. 스트레인 게이지(strain gauge) 기반이 대부분이다.

*관절 토크 센서 (Joint Torque Sensor, JTS)*: 감속기 출력 측에 장착. KUKA LBR iiwa가 7개 관절 모두에 JTS를 장착하여 힘 제어의 기준을 세웠다.

*힘/토크 센서 (F/T Sensor)*: 끝단에 장착하여 6축(Fx, Fy, Fz, Tx, Ty, Tz)을 측정. ATI Industrial Automation의 센서가 연구용 표준이다. 가격이 비싸다 ($3,000~$20,000).

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

자동차 산업에서 시작하여 로봇에서도 표준으로 자리 잡았다. 차동 신호(differential signaling)로 노이즈에 강하고, 멀티마스터 구조에 우선순위 기반 중재(arbitration)를 지원한다.

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

산업용 실시간 이더넷 프로토콜이다. 일반 이더넷 하드웨어를 사용하면서 마이크로초 단위의 결정적(deterministic) 통신을 제공한다.

왜 로봇에서 쓰는가:
- **속도**: 100 Mbps, 수십~수백 개 노드를 마이크로초 주기로 동기화
- **결정론적 타이밍**: 패킷 지연이 일정 → 실시간 제어에 적합
- **처리 방식**: 마스터가 보낸 프레임을 각 슬레이브가 on-the-fly로 읽고 쓴다 (프레임이 지나가면서 처리). 대역폭 효율이 극히 높다.

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

로봇 제어에서 "실시간(real-time)"은 "빠른"이 아니라 **"정해진 시간 안에 반드시 완료되는"**을 의미한다. 1kHz 제어 루프라면, 매 1ms마다 센서 읽기 → 제어 계산 → 모터 명령 전송이 완료되어야 한다. 한 번이라도 지연되면 로봇이 불안정해질 수 있다.

**RTOS (Real-Time Operating System):**

| RTOS | 특징 | 용도 |
|------|------|------|
| **FreeRTOS** | 경량, 마이크로컨트롤러용, 무료 | STM32, ESP32 등 |
| **Zephyr** | 최신, 다양한 하드웨어 지원, Linux Foundation | IoT, 로봇 임베디드 |
| **VxWorks** | 상용, NASA도 사용 | 항공우주, 산업용 |

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
# 최대 지연(max latency)이 50μs 이하면 양호
```

**왜 1kHz인가:**

로봇 제어 루프의 표준 주기가 1kHz (1ms)인 이유:

1. **임피던스/힘 제어**: 기계적 공진 주파수보다 충분히 높은 제어 주파수가 필요하다. 대부분의 로봇 팔은 수~수십 Hz의 고유 진동수를 가지므로, 안정적 제어를 위해 적어도 10배 이상 (→ 수백 Hz~1kHz)이 필요하다.
2. **Nyquist 정리**: 100Hz의 동적 현상을 제어하려면 최소 200Hz 샘플링이 필요하고, 실제로는 5~10배(→ 1kHz) 정도가 바람직하다.
3. **통신 대역폭**: CAN bus 1 Mbps로 10개 모터를 1kHz로 제어하면 꽉 찬다. 그 이상은 EtherCAT이 필요하다.
4. **관행**: MIT Cheetah가 CAN + 1kHz 구성으로 동적 보행을 시연한 이후 QDD + 1kHz가 사실상 표준이 되었다.

고주파 제어 (5~10kHz)가 필요한 경우: 매우 가벼운 로봇 (낮은 관성), 고속 충돌 대응, 일부 촉각 제어. 이 경우 EtherCAT이나 FPGA 기반 제어가 필요하다.

> **추천 자료**
> - FreeRTOS 공식 문서: https://www.freertos.org/
> - PREEMPT_RT Wiki: https://wiki.linuxfoundation.org/realtime/start
> - Dynamixel SDK: https://github.com/ROBOTIS-GIT/DynamixelSDK
> - IgH EtherCAT Master (Linux용 오픈소스): https://etherlab.org/en/ethercat/
> - SOEM (Simple Open EtherCAT Master): https://github.com/OpenEtherCATsociety/SOEM

---

## 4.6 심화: Workspace Analysis와 최적 설계

*연구자가 되고 싶다면 여기서부터 읽어라.*

기구학은 "주어진 로봇을 어떻게 움직이나"의 문제이기도 하지만, "어떤 로봇을 설계해야 하나"의 문제이기도 하다. 이 절은 설계 최적화와 관련된 고급 주제를 다룬다.


### 4.6.1 Reachable Workspace vs Dexterous Workspace

**Reachable workspace**: 끝단이 적어도 하나의 자세(orientation)로 도달할 수 있는 모든 점의 집합. "어디까지 손이 닿는가."

**Dexterous workspace**: 끝단이 임의의 자세로 도달할 수 있는 점의 집합. "어디서 자유롭게 움직일 수 있는가." 당연히 reachable workspace의 부분집합이고, 보통 훨씬 작다.

6-DOF 로봇의 경우, dexterous workspace는 상당히 제한적일 수 있다. 이것이 7-DOF 로봇이 존재하는 이유 중 하나이다.

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
1. **특이점 회피**: 자코비안의 manipulability를 최대화하는 방향으로 여분 자유도 사용
2. **관절 제한 회피**: 관절이 한계에 가까워지면 여분 자유도로 중앙 위치 복귀
3. **장애물 회피**: 팔꿈치가 장애물과 충돌하지 않도록 형태 조정
4. **에너지 최적화**: 토크를 최소화하는 자세 선택

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

---

## 4.7 추천 자료

기구학과 메카트로닉스를 진지하게 공부하려면 교재 하나를 처음부터 끝까지 풀어보는 것이 가장 효과적이다.

**교재:**

- **Craig, "Introduction to Robotics: Mechanics and Planning"** — DH 파라미터와 기구학의 정석. Modified DH convention을 사용한다. 학부 수준에서 가장 적합하다.

- **Lynch & Park, "Modern Robotics: Mechanics, Planning, and Control"** — PoE 기반. 무료 PDF와 Coursera 강의를 제공하여 접근성이 뛰어나다. 수학적으로 더 깔끔하지만 처음 보면 어렵다. https://modernrobotics.org

- **Corke, "Robotics, Vision and Control"** — MATLAB/Python 코드와 함께 기구학을 실습할 수 있다. robotics-toolbox-python은 이 책의 동반 라이브러리이다. 3판은 Python 기반. https://petercorke.com/rvc/

- **Siciliano et al., "Robotics: Modelling, Planning and Control"** — 가장 포괄적인 대학원 교과서. 기구학, 동역학, 제어를 모두 다룬다. 두껍지만 그만큼 빠짐없다.

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
1969 ── Stanford Arm (최초의 전기식 컴퓨터 제어 로봇 팔)
1985 ── Product of Exponentials 정형화
1998 ── Harmonic Drive 로봇 적용 확산
2019 ── MIT Mini Cheetah: QDD 액추에이터
2019 ── MoveIt2 (ROS2 기반 모션 플래닝 프레임워크)
2023 ── ALOHA: 저비용 양팔 텔레오퍼레이션 플랫폼
2024 ── SO-ARM100: 오픈소스 5축 로봇 팔 ($200 이하)
```

---

*다음 장에서는 이 기구학 위에 힘과 질량을 얹는다: 동역학(Dynamics)과 제어(Control). 관절 각도를 "원하는 값으로 보내는" 것이 아니라, "원하는 토크를 가하는" 관점으로 바뀐다.*
