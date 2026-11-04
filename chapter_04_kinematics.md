# Ch.4 — 기구학 & 메카트로닉스 (Kinematics & Mechatronics)


책상 위에 로봇 팔이 하나 있다. 모터 6개에 각각 어떤 각도를 줘야 손끝이 커피잔에 닿을까? 이 물음을 푸는 분야가 기구학이다. 그 모터를 실제로 돌리고 센서를 읽고 제어 루프를 1kHz로 돌리는 일은 메카트로닉스가 맡는다.

---

## 4.1 왜 기구학을 배우는가

로봇 매니퓰레이터는 여러 관절(joint)과 링크(link)로 이루어진다. 작업에 필요한 값은 끝단(end-effector)의 위치와 자세(pose)인데, 모터로 직접 정하는 값은 관절의 각도(또는 변위)다. 기구학(Kinematics)은 이 두 값 사이의 관계를 식으로 쓴다.

- 순기구학(Forward Kinematics, FK): 관절 각도 → 끝단 위치/자세
- 역기구학(Inverse Kinematics, IK): 끝단 위치/자세 → 관절 각도

기구학은 위치와 자세의 기하만 다룬다. 운동에 필요한 힘과 토크는 다음 장의 동역학(Dynamics)에서 다룬다.

로봇 팔의 경로 계획(motion planning), 원격 조종에서 마스터 팔과 슬레이브 팔의 자세를 맞추는 텔레오퍼레이션, 실제 로봇과 모델의 오차를 잡는 캘리브레이션에 모두 기구학이 들어간다. 충돌을 피하려면 링크마다 공간 어디에 있는지 알아야 하는데, 이 위치도 기구학으로 계산한다.

---

## 4.2 순기구학 (Forward Kinematics)

### 4.2.1 동차 변환 행렬 (Homogeneous Transformation Matrix)

기구학의 기본 도구는 4×4 동차 변환 행렬이다:

```
T = | R  p |
    | 0  1 |
```

R은 3×3 회전 행렬, p는 3×1 위치 벡터다. 행렬 하나에 강체의 위치와 자세가 함께 들어가고, 여러 변환은 행렬 곱으로 이어 붙인다.

프레임 0에서 1로 가는 변환 T_01과 1에서 2로 가는 변환 T_12를 곱하면 0에서 2로 가는 변환이 나온다.

```
T_02 = T_01 * T_12
```
순기구학은 관절 각도를 알 때 끝단의 위치와 자세를 구하는 문제다. 베이스에서 끝단까지 관절마다 변환을 하나씩 곱해 나가면 된다.

```
q = [θ_1, θ_2, ..., θ_n]^T  →  T = [R, t; 0, 1] ∈ SE(3)
```

관절 각도가 정해지면 끝단의 위치와 자세는 하나로 정해진다.

### 4.2.2 Denavit-Hartenberg (DH) 파라미터

DH 파라미터는 이웃한 두 링크 프레임의 상대 위치와 방향을 파라미터 4개로 나타낸다.

| 파라미터 | 의미 | 설명 |
|---------|------|------|
| $a_i$ | 링크 길이 (link length) | $z_{i-1}$과 $z_i$ 사이의 $x_i$ 축을 따른 거리 |
| $\alpha_i$ | 링크 꼬임각 (link twist) | $z_{i-1}$과 $z_i$ 사이의 $x_i$ 축 기준 회전각 |
| $d_i$ | 링크 오프셋 (link offset) | $x_{i-1}$과 $x_i$ 사이의 $z_{i-1}$ 축을 따른 거리 |
| $\theta_i$ | 관절각 (joint angle) | $x_{i-1}$과 $x_i$ 사이의 $z_{i-1}$ 축 기준 회전각 |

회전 관절(revolute joint)에서는 $\theta_i$만 변하고 나머지 세 값은 상수다. 직선 관절(prismatic joint)에서는 $d_i$가 변한다.

관절 하나의 변환 행렬은 다음과 같다.

```
T_i = Rot_z(θ_i) * Trans_z(d_i) * Trans_x(a_i) * Rot_x(α_i)

    = | cos(θ)  -sin(θ)cos(α)   sin(θ)sin(α)   a*cos(θ) |
      | sin(θ)   cos(θ)cos(α)  -cos(θ)sin(α)   a*sin(θ) |
      | 0        sin(α)          cos(α)          d        |
      | 0        0               0               1        |
```

DH convention은 standard와 modified로 나뉜다. modified는 Craig 교과서가 쓰는 방식이라 Craig convention이라고도 부르고, 다른 교재는 대개 standard를 쓴다. 두 방식은 링크에 프레임을 붙이는 위치가 달라서 섞어 쓰면 결과가 틀린다. 어느 방식을 쓰는지 늘 적어 둔다.


### 4.2.3 예제: 2-link Planar Arm의 FK

평면에서 움직이는 2-링크 로봇 팔로 시작한다.

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

끝단 위치는 삼각함수로 바로 나온다.

```
x = L1*cos(θ_1) + L2*cos(θ_1 + θ_2)
y = L1*sin(θ_1) + L2*sin(θ_1 + θ_2)
```

Python 코드는 다음과 같다.

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
# 출력: End-effector position: (1.125, 1.466), orientation: 75.0°
```

6축 로봇 팔의 FK도 같은 계산이다. 4×4 행렬을 여섯 번 곱하면 된다.


### 4.2.4 Product of Exponentials (PoE)

PoE(Product of Exponentials)는 Lie group과 Lie algebra로 FK를 쓰는 방법이다. Brockett가 1983년 MTNS 학회에서 발표해 1984년에 출간했고, Lynch와 Park의 *Modern Robotics*가 이 방법으로 기구학을 설명한다. PoE는 관절마다 screw axis(회전축과 이동 방향을 묶은 6차원 벡터)를 두고, 행렬 지수(matrix exponential)로 변환을 계산한다.

```
T(θ) = e^{[S_1]θ_1} * e^{[S_2]θ_2} * ... * e^{[S_n]θ_n} * M
```

- S_i는 i번째 관절의 screw axis (6×1 벡터)
- [S_i]는 S_i의 4×4 행렬 표현 (se(3) 원소이며, 왼쪽 위 3×3 회전 블록이 skew-symmetric)
- M은 모든 관절이 영 위치(home configuration)일 때의 끝단 자세
- θ_i는 관절 변수


| 항목 | DH | PoE |
|------|-----|-----|
| 프레임 부착 | 각 링크에 프레임 필요 | 기준 프레임과 끝단 프레임만 필요 |
| Convention 혼동 | standard vs modified 주의 | 없음 (space form vs body form 구분은 있음) |
| 수학적 기반 | 행렬 곱 | Lie group, 행렬 지수 |
| 산업계 채택 | 매우 높음 | 학계 중심, 점점 확산 |
| 교재 | Craig, Siciliano | Lynch & Park |

산업용 로봇 매뉴얼과 교재는 대개 DH 파라미터로 로봇을 적는다. ROS의 로봇 기술 파일인 URDF는 관절마다 부모 링크에서 자식 링크로 가는 변환을 직접 적는다. 연구 논문에서는 PoE를 자주 본다. 매뉴얼, URDF, 논문의 식을 오가려면 세 표기를 서로 바꿔 쓸 줄 알아야 한다.

```python
# robotics-toolbox-python으로 DH 기반 FK 예제 (Puma 560)
import numpy as np
import roboticstoolbox as rtb

puma = rtb.models.DH.Puma560()
q = [0, -np.pi/4, np.pi/4, 0, np.pi/6, 0]  # 6개 관절 각도
T = puma.fkine(q)
print(T)  # 4x4 SE(3) 동차 변환 행렬 출력
print(f"Position: {T.t}")  # 끝단 위치
print(f"RPY angles: {T.rpy()}")  # Roll-Pitch-Yaw
```

> **추천 자료**
> - Lynch & Park, *Modern Robotics*, Chapter 4 — PoE로 FK를 설명한다. 저자가 무료 PDF를 공개했고 Coursera 강의도 있다: https://modernrobotics.org
> - Craig, *Introduction to Robotics*, Chapter 3 — modified DH convention으로 기구학을 설명한다
> - Peter Corke, *Robotics, Vision and Control* — 동반 라이브러리 robotics-toolbox-python으로 FK를 직접 돌려 본다: https://github.com/petercorke/robotics-toolbox-python

---

## 4.3 역기구학 (Inverse Kinematics)

IK는 끝단을 (x, y, z)에 놓으려면 관절 각도를 각각 얼마로 해야 하는지 묻는다. 식은 삼각함수가 얽힌 비선형 방정식이다. 같은 끝단 위치에 닿는 관절 조합이 팔꿈치를 올린 자세(elbow-up)와 내린 자세(elbow-down)처럼 여럿일 때가 많다. 팔이 닿는 범위(workspace) 밖의 점에는 해가 없고, 자유도가 작업에 필요한 수보다 많은 팔(redundant manipulator)에서는 해가 무수히 많다.


### 4.3.1 Analytical IK (해석적 방법)

해석적 방법은 관절 각도를 식 하나로 바로 계산하는 닫힌 형태(closed-form)의 해를 구한다. 해가 있으면 반복 계산 없이 후보를 모두 얻는다. 수치 정확도와 실행 시간은 구현과 특이점 근처 처리 방식에 따라 다르다.

**2-link planar arm의 IK:**

목표 위치 (x, y)에 대한 해는 다음과 같다.

```
cos(θ_2) = (x² + y² - L1² - L2²) / (2 * L1 * L2)
θ_2 = atan2(±√(1 - cos²(θ_2)), cos(θ_2))

θ_1 = atan2(y, x) - atan2(L2*sin(θ_2), L1 + L2*cos(θ_2))
```

± 부호에 따라 해가 elbow-up과 elbow-down 두 개 나온다. 실제 로봇에서는 이 가운데 하나를 골라야 한다.

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

Pieper는 6축 로봇에서 마지막 세 관절 축이 한 점에서 만나면(spherical wrist) IK를 해석적으로 풀 수 있음을 보였다. KUKA, ABB의 산업용 6축 로봇 상당수가 이 구조다. 이때는 처음 세 축으로 손목 중심의 위치를 맞추고, 마지막 세 축으로 자세를 맞추는 두 문제로 나눠 푼다. 해는 최대 8개이고, 실제로는 관절 제한(joint limit) 안에 있으면서 직전 관절 각도에 가까운 해를 고른다. UR 팔은 손목 세 축이 한 점에서 만나지 않아 이 분해가 그대로 통하지 않는다. UR 팔에도 닫힌 해가 있지만 따로 유도해야 한다.


### 4.3.2 Numerical IK (수치적 방법)

닫힌 해를 구할 수 없는 구조나 7축 이상인 팔은 반복 계산으로 IK를 푼다.

**Jacobian Pseudo-Inverse 방법:**

```
Δq = J†(q) * Δx
```

J†는 자코비안의 pseudo-inverse다. 이 갱신을 오차가 충분히 작아질 때까지 되풀이한다.

```python
def numerical_ik_2link(target_x, target_y, L1=1.0, L2=1.0,
                        max_iter=100, tol=1e-6):
    """Jacobian pseudo-inverse 기반 수치적 IK."""
    # 초기 추정값 (여기서는 고정값. 실제로는 현재 관절 각도를 넣는다)
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

Pseudo-inverse를 쓰면 특이점 근처에서 관절 속도가 매우 커진다. DLS는 감쇠 계수(damping factor) λ를 더해 이 폭주를 누른다.

```
Δq = J^T (J * J^T + λ²I)^{-1} * Δx
```

λ를 키우면 특이점 근처에서 안정해지는 대신 목표에 늦게 다가간다. λ를 줄이면 pseudo-inverse와 거의 같아진다. Nakamura와 Hanafusa(1986)는 특이점에 가까워질수록 λ를 키우는 방법을 제안했다.


### 4.3.3 특이점 (Singularity)

자코비안의 rank가 떨어지는 관절 배치를 특이점(singularity)이라 한다. 특이점에서는 끝단이 움직일 수 있는 방향이 줄어든다. 그 근처에서는 끝단을 조금만 옮겨도 관절 속도가 매우 커지고, IK 해가 바뀌면서 관절 각도가 갑자기 튀기도 한다.

2-link arm은 팔이 완전히 펴진 θ_2 = 0과 완전히 접힌 θ_2 = π에서 특이점이 된다. 이 자세에서 끝단은 베이스와 끝단을 잇는 직선에 수직인 방향으로만 움직인다.

spherical wrist가 있는 6축 로봇에서는 축 4와 축 6이 일직선이 될 때(q5 = 0 또는 π) wrist singularity, 손목 중심이 축 1 위에 올 때 shoulder singularity, 팔이 완전히 펴지거나 접힐 때 elbow singularity가 생긴다.

실제로는 경로 계획 단계에서 특이점 근처를 피하고, 지나가야 하면 DLS로 관절 속도를 묶는다. 7축 팔은 남는 자유도로 특이점을 비켜 간다(§4.6.3).


### 4.3.4 IK 솔버들

실무에서는 IK를 직접 짜기보다 아래 솔버를 가져다 쓴다.

| 솔버 | 방법 | 특징 |
|------|------|------|
| KDL | Numerical (Newton-Raphson) | ROS에 기본으로 들어 있다. 관절 한계·초기값·특이점에 민감 |
| IKFast (OpenRAVE) | Analytical (코드 생성) | 로봇 구조마다 C++ 해석해 코드를 자동 생성. 빠름 |
| TRAC-IK | KDL-RR과 SQP를 동시에 실행 | KDL보다 성공률 높음, ROS 패키지 있음 |
| MoveIt2 IK | 위 솔버들을 통합 | ROS2 생태계, 충돌 회피 통합 |
| pinocchio | 자코비안 기반 수치 반복 (CLIK) | 강체 동역학 라이브러리, 빠름, 해석적 미분 제공 |

```python
# Beeson & Ames (2015)는 5개 로봇 모델의 관절 체인마다 도달 가능한 자세를
# 10,000개씩 뽑아, 해 하나당 5ms 제한으로 비교했다.
# 그 실험에서는 TRAC-IK가 stock KDL보다 높은 solve rate를 보였지만,
# 수치는 관절 체인·초기값·허용 오차에 따라 달라진다.
```

> **추천 자료**
> - [Beeson & Ames, "TRAC-IK: An Open-Source Library for Improved Solving of Generic Inverse Kinematics" (2015)](https://doi.org/10.1109/HUMANOIDS.2015.7363472) — 모델별 조건과 solve rate는 원 논문의 표를 확인
> - MoveIt2 IK 문서: https://moveit.picknik.ai/main/doc/concepts/kinematics.html
> - Pinocchio (rigid body dynamics library): https://github.com/stack-of-tasks/pinocchio

---

## 4.4 자코비안 (Jacobian)


### 4.4.1 관절 속도 → 끝단 속도

자코비안 J(q)는 관절 속도 q̇를 끝단 속도(선속도 v, 각속도 ω)로 옮긴다.

```
ẋ = J(q) * q̇

여기서 ẋ = [v; ω] ∈ ℝ^6 (6축의 경우)
      q̇ ∈ ℝ^n
      J(q) ∈ ℝ^{6×n}
```

관절 수 n이 6보다 작으면 끝단을 모든 방향으로 움직일 수 없고, 6보다 크면 남는 자유도(redundancy)가 생긴다. 지금 자세에서 끝단이 실제로 움직일 수 있는 방향의 수는 J(q)의 rank로 정해진다. underactuated는 이와 다른 개념으로, 독립적으로 구동하는 입력이 시스템 자유도보다 적은 경우를 말한다.

바퀴로 움직이는 모바일 로봇에는 차체 속도 $(v, \omega)$를 명령으로 준다. 이 명령에 노이즈를 넣은 모델은 §4.7에서 다룬다.


### 4.4.2 힘/토크 관계 (Duality)

자코비안의 전치(transpose)는 끝단 힘을 관절 토크로 옮긴다.

```
τ = J^T(q) * F
```

τ는 관절 토크, F는 끝단에 걸리는 힘과 모멘트다. 이 관계를 정역학적 이중성(static duality)이라 한다. 관절이 하는 일률과 끝단이 하는 일률이 같다는 조건에서 바로 나온다.

```
P = F^T * ẋ = F^T * J * q̇ = (J^T * F)^T * q̇ = τ^T * q̇
```

힘 제어(force control)는 이 식을 그대로 쓴다. 끝단으로 힘 F를 내려면 관절마다 τ = J^T * F만큼 토크를 건다.


### 4.4.3 Manipulability Ellipsoid

자코비안으로 지금 자세에서 끝단이 각 방향으로 얼마나 쉽게 움직이는지도 잰다.

```
manipulability index = √det(J * J^T)
```

6차원 작업에서 J가 보통 full row rank인 로봇이라면, 이 값이 0인 자세가 특이점이다. 값이 클수록 조작성 타원체(manipulability ellipsoid)의 부피가 크다. 모든 방향으로 고르게 움직이는지는 축 길이의 비로 따로 본다.

타원체의 축 방향은 J * J^T의 고유벡터이고, 반축 길이는 고유값의 제곱근이다. 반축이 긴 방향으로는 같은 관절 속도로 끝단이 빨리 움직인다. 고유값이 모두 비슷한 자세를 등방(isotropic) 자세라 한다.

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

# Condition number: 등방성 지표 (1에 가까울수록 등방)
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
> - Siciliano et al., *Robotics: Modelling, Planning and Control*, Chapter 3 — 미분 기구학과 정역학(Differential Kinematics and Statics). 자코비안, 특이점, 조작성 타원체를 다룬다
> - Corke, *Robotics, Vision and Control*, Chapter 8 — 매니퓰레이터 속도와 자코비안을 코드와 그림으로 설명한다: https://petercorke.com/rvc/
> - robotics-toolbox-python 문서: https://github.com/petercorke/robotics-toolbox-python

---

## 4.5 메카트로닉스 기초

관절 각도 명령이 실제 움직임이 되려면 모터, 센서, 전자 회로, 통신이 모두 맞물려야 한다. 이 하드웨어를 다루는 분야가 메카트로닉스다.


### 4.5.1 액추에이터

**DC 모터:**
전압을 걸면 도는 가장 단순한 모터다. 토크는 전류에 비례하고(τ = K_t * i), 역기전력은 회전 속도에 비례한다(V_emf = K_e * ω). 제어가 쉽고 값이 싸다. 브러시가 닳기 때문에 수명이 짧다.

**BLDC (Brushless DC) 모터:**
브러시 대신 드라이버 회로가 코일 전류의 방향을 바꾼다. 수명이 길고 토크 밀도와 효율이 높아 요즘 로봇 관절에 많이 쓴다. 드라이버가 FOC(Field-Oriented Control, 회전자 자속 방향에 맞춰 전류를 제어하는 방식)를 쓰면 토크 리플(ripple, 회전 중 토크가 출렁이는 현상)이 줄어든다.

**서보 모터 (Dynamixel 시리즈):**
모터, 감속기, 엔코더, 컨트롤러를 한 몸체에 넣은 제품이다. Robotis의 Dynamixel이 연구용·교육용 로봇에 많이 쓰인다.

| 모델 | 공칭 토크 예시 (Nm) | 통신 | 용도 |
|------|-----------|------|------|
| XL330 | 0.5 | TTL | 소형 그리퍼, 소형 저가 팔 |
| XM540 | 10.0 | RS-485 | 중형 로봇 팔 |
| PH54  | 44.7 (연속 운전) | RS-485 | 대형 매니퓰레이터, 모바일 로봇 |

토크는 모델과 공급 전압에 따라 달라진다. 고를 때는 e-Manual에서 stall 토크를 잰 조건과 연속 운전 한계를 확인한다. Dynamixel은 여러 개를 한 선에 줄줄이 잇는 데이지 체인 연결을 지원하고, 위치·속도·전류 제어 모드를 고를 수 있으며, PID 게인도 바꿀 수 있다. 제품마다 통신 주기와 발열 한계가 다르니, 필요한 제어 주기와 모드를 기본 펌웨어가 지원하는지 먼저 본다.

**Quasi-Direct Drive (QDD):**

QDD는 감속비를 6:1에서 10:1 정도로 낮춘 관절이다. harmonic drive를 쓰는 일반 로봇 관절의 감속비는 100:1을 넘는다. 감속기로는 유성기어나 벨트를 쓴다.

감속비가 낮으면 바깥에서 힘을 줄 때 관절이 쉽게 밀려 돌아간다. 이 성질을 백드라이버빌리티(backdrivability)라 하고, 덕분에 충돌에 대응하거나 힘을 제어하기가 쉬워진다. 감속기 마찰을 잘 모델링하면 모터 전류만으로 관절 토크를 꽤 정확히 추정할 수 있다. 감속기의 마찰과 탄성이 작으니 토크 응답 대역폭도 높게 잡을 수 있다. 대신 같은 크기의 모터에서 나오는 출력 토크가 작아서, 큰 토크가 필요하면 모터를 키워야 한다.

MIT Mini Cheetah와 Unitree의 사족 보행 로봇이 이런 관절을 쓴다.

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
| Planetary | 3~100:1 | 중간 | 85~95% | 저렴 | 범용, QDD에 적합 |
| Harmonic Drive | 30~320:1 | 매우 낮음 | 65~85% | 비쌈 | 산업용 로봇, 정밀 |
| Cycloidal | 6~120:1 | 낮음 | 85~93% | 중간 | 최근 대안으로 부상 |

감속비, 효율, 백래시는 같은 계열 안에서도 제품마다 크게 다르다. 표의 수치는 계열끼리 비교하는 참고값이고, 부품을 고를 때는 제조사 데이터시트의 정격 부하 조건을 본다.

**액추에이터 선정 기준:**

관절 액추에이터를 고를 때는 자세를 버티는 정적 토크, 가속에 드는 동적 토크, 충격 하중을 더한다. 여기에 하중을 얼마나 정확히 아는지, 수명을 얼마나 요구하는지, 고장 나면 어떤 일이 생기는지를 따져 여유 계수를 곱한다. 아래 코드의 2배는 계산 예시로 넣은 값이다. 모터 RPM은 관절 최대 각속도에 감속비를 곱해 구한다. 백드라이버빌리티, 크기와 무게, 연속 토크와 발열 한계도 함께 본다. QDD와 harmonic drive 가운데 무엇을 쓸지는 토크 밀도, 외력이 모터까지 그대로 전달되는 정도(투명도), 위치 정밀도, 비용 요구로 정한다.

```python
# 간단한 액추에이터 선정 계산 예시
import numpy as np

# 목표: 1kg 물체를 팔 끝에서 들어올리기 (팔 길이 0.5m)
m_payload = 1.0  # kg
m_link = 0.5     # kg (링크 자체 질량)
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
> - Katz, "A Low Cost Modular Actuator for Dynamic Robots" (MIT 석사학위 논문, 2018) — Mini Cheetah에 들어간 저가 모듈형 액추에이터의 하드웨어와 제어를 다룬다: https://dspace.mit.edu/handle/1721.1/118671
> - Dynamixel 제품 라인업 및 문서: https://emanual.robotis.com/
> - Seok et al., "Design Principles for Energy-Efficient Legged Locomotion and Implementation on the MIT Cheetah Robot" (2015)


### 4.5.2 센서 인터페이싱

**엔코더 (Encoder):**

엔코더는 관절이 얼마나 돌았는지 잰다.

*Incremental encoder*는 A, B 두 채널의 펄스를 세어 처음 위치에서 얼마나 돌았는지 잰다. 전원이 꺼지면 위치를 잃으므로 켤 때마다 원점을 다시 찾아야(homing) 한다. 값이 싸고 분해능이 높다. 한 바퀴에 10,000펄스(PPR) 이상인 제품도 흔하다.

*Absolute encoder*는 현재 각도를 바로 출력하므로 전원을 켜자마자 위치를 안다. Multi-turn absolute encoder는 몇 바퀴째인지도 기억한다. 값은 더 비싸지만 원점 복귀가 필요 없어서, 다시 켰을 때 곧바로 이전 위치에서 일을 이어 가야 하는 산업용 로봇에 많이 쓴다.

```
분해능 계산 예시:
  Incremental encoder, 4096 PPR, quadrature decoding (x4)
  → 분해능 = 360° / (4096 × 4) = 0.022° ≈ 0.38 mrad
  → 감속비 100:1 관절 → 출력 분해능 0.0038 mrad
```

**토크 센서:**

토크 센서는 관절 토크나 끝단 힘을 직접 잰다. 대부분 스트레인 게이지(strain gauge)로 변형을 읽는다.

*관절 토크 센서(Joint Torque Sensor, JTS)*는 감속기 출력 쪽에 단다. KUKA LBR iiwa는 관절 7개에 모두 토크 센서를 달았다.

*힘/토크 센서(F/T Sensor)*는 끝단에 달아 세 방향 힘과 세 축 토크(Fx, Fy, Fz, Tx, Ty, Tz)를 잰다. 연구실에서는 ATI Industrial Automation 제품을 많이 쓴다. 고를 때는 측정 범위, 분해능, 과부하 한계, 인터페이스, 가격을 함께 본다.

**관성 센서 (IMU):**

IMU는 가속도계와 자이로스코프를 묶은 센서이고, 자력계가 더해진 제품도 있다(Ch.2). 모바일 로봇과 보행 로봇은 IMU로 몸체 자세를 추정한다. 매니퓰레이터에서는 링크에 IMU를 달아 진동을 줄이는 데 쓰기도 한다.


### 4.5.3 통신 프로토콜

통신 프로토콜은 센서와 액추에이터를 마이크로컨트롤러나 PC에 잇는 규칙이다. 지연(latency)이 크면 제어가 불안정해진다. 대역폭이 모자라면 데이터가 빠진다.

**기초 프로토콜:**

| 프로토콜 | 배선 | 속도 | 거리 | 특징 |
|---------|------|------|------|------|
| UART | 2선 (TX, RX) | ~1 Mbps | ~15 m | 가장 단순, 1:1 통신 |
| SPI | 4선 (MOSI, MISO, SCK, CS) | ~50 Mbps | ~1 m (PCB 내) | 빠름, 다수 슬레이브는 CS 추가 |
| I2C | 2선 (SDA, SCL) | 100 kbps~3.4 Mbps | ~1 m | 주소 기반, 센서 연결에 편리 |

UART, SPI, I2C는 보드 안이나 가까운 센서를 잇는 데 쓴다. 모터 드라이버처럼 몇 미터 떨어진 장치를 잡음 속에서 잇는 데는 아래 프로토콜을 쓴다.

**CAN Bus:**

CAN은 자동차 산업에서 시작한 버스이고, 로봇의 모터와 센서를 잇는 데도 쓴다. 두 선의 전압 차로 신호를 보내는 차동 신호(differential signaling) 방식이라 잡음에 강하다. 여러 노드가 동시에 보내려 하면 ID 우선순위로 한 노드만 남기는 중재(arbitration)를 한다.

- 속도: 최대 1 Mbps (CAN 2.0), 5 Mbps (CAN FD)
- 거리: 50 kbps에서 약 1 km, 125 kbps에서 약 500 m, 1 Mbps에서 약 25 m
- 토폴로지: 버스 (데이지 체인 가능)

로봇에서는 주로 모터 드라이버와 메인 컨트롤러 사이를 CAN으로 잇는다. MIT Cheetah를 비롯한 보행 로봇 여러 대가 CAN을 쓴다.

```cpp
// CAN bus를 통한 모터 명령 전송 예시 (pseudo-code, STM32 HAL)
#include "can.h"

// x를 [x_min, x_max] 범위의 bits비트 부호 없는 정수로 바꾼다
uint16_t float_to_uint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    if (x < x_min) x = x_min;
    if (x > x_max) x = x_max;
    return (uint16_t)((x - x_min) * ((float)((1 << bits) - 1)) / span);
}

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

    // 다섯 값을 8바이트에 넣는다. 배치는 MIT Mini Cheetah 모터 드라이버(Ben Katz)의
    // 명령 패킷을 따른다: position 16비트, 나머지 넷은 12비트.
    // 범위는 펌웨어마다 다르다(아래는 한 예).
    uint8_t data[8];
    uint16_t p_int  = float_to_uint(cmd.position, -12.5f, 12.5f, 16);  // rad
    uint16_t v_int  = float_to_uint(cmd.velocity, -30.0f, 30.0f, 12);  // rad/s
    uint16_t kp_int = float_to_uint(cmd.kp,         0.0f, 500.0f, 12); // Nm/rad
    uint16_t kd_int = float_to_uint(cmd.kd,         0.0f,   5.0f, 12); // Nm·s/rad
    uint16_t t_int  = float_to_uint(cmd.torque,   -18.0f, 18.0f, 12);  // Nm

    data[0] = p_int >> 8;
    data[1] = p_int & 0xFF;
    data[2] = v_int >> 4;
    data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
    data[4] = kp_int & 0xFF;
    data[5] = kd_int >> 4;
    data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
    data[7] = t_int & 0xFF;

    uint32_t mailbox;
    HAL_CAN_AddTxMessage(hcan, &header, data, &mailbox);
}
```

**EtherCAT:**

EtherCAT은 Beckhoff가 만든 산업용 실시간 이더넷이다. 일반 이더넷 하드웨어로 지연이 매번 같은(deterministic) 통신을 한다. 마스터가 보낸 프레임 하나가 슬레이브를 차례로 지나가고, 슬레이브는 프레임이 지나가는 동안 자기 몫의 데이터를 읽고 쓴다. 그래서 100 Mbps 선 하나로 노드 수백 개를 묶어도 대역폭을 거의 낭비하지 않고, 노드 사이 시계를 1 µs 안으로 맞춘다.

KUKA 컨트롤러와 Beckhoff 제품이 EtherCAT을 쓴다. PC를 마스터로 쓰려면 SOEM이나 IgH EtherCAT Master 같은 마스터 소프트웨어를 깔고 설정해야 해서, 취미용 로봇에는 손이 너무 많이 간다.

**RS-485 / Dynamixel Protocol:**

Dynamixel 서보는 RS-485나 TTL로 통신한다. RS-485는 차동 신호를 쓰는 시리얼 통신이고, 짧은 거리에서는 수 Mbps 넘게 보낸다. 여러 장치를 데이지 체인으로 이을 수 있다. 아래 예제의 1 Mbps는 Dynamixel 통신 속도 설정값이다.

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

로봇 제어에서 실시간(real-time)은 정해진 마감 시간(deadline) 안에 처리가 끝난다고 보장한다는 뜻이다. 1kHz 제어 루프라면 1ms마다 센서를 읽고, 제어 입력을 계산하고, 모터에 명령을 보내는 일이 끝나야 한다. 마감을 넘기는 일이 생기면 로봇이 불안정해질 수 있다.

**RTOS (Real-Time Operating System):**

| RTOS | 특징 | 용도 |
|------|------|------|
| FreeRTOS | 경량, 마이크로컨트롤러용, 무료 | STM32, ESP32 등 |
| Zephyr | 최신, 다양한 하드웨어 지원, Linux Foundation | IoT, 로봇 임베디드 |
| VxWorks | 상용, NASA도 사용 | 항공우주, 산업용 |

마이크로컨트롤러에서 모터를 직접 제어할 때는 RTOS를 쓴다. 제어 루프 태스크에 높은 우선순위를 줘서 다른 태스크에 밀리지 않게 한다.

**PREEMPT_RT Linux:**

ROS2는 주로 Linux에서 돌린다. 일반 Linux 커널은 마감 시간을 보장하지 않아서 제어 스레드가 수 밀리초씩 밀릴 수 있다. PREEMPT_RT 패치는 커널 코드 대부분을 다른 스레드가 끼어들 수 있게(preemptible) 바꿔 이 지연을 줄인다. 이 패치는 Linux 6.12부터 메인라인 커널에 들어갔다.

설정 방법 (개략):
```bash
# 1. PREEMPT_RT 패치가 적용된 커널 설치 (Debian 예시)
sudo apt install linux-image-rt-amd64   # Debian 메타패키지. Ubuntu는 이 이름이 없고 Ubuntu Pro의 real-time kernel 등 별도 경로를 쓴다

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

토크 제어와 임피던스 제어는 1kHz(1ms)로 많이 설계한다. 필요한 주기는 폐루프 대역폭, 기계 공진, 센서와 액추에이터 지연, 계산 시간, 주기 흔들림(jitter) 여유를 보고 정한다. Nyquist 기준의 2배는 aliasing을 피하는 최소 조건일 뿐이라, 실제로는 목표 폐루프 대역폭보다 훨씬 빠르게 샘플링하고 주파수 응답과 지연 여유를 확인한다. CAN을 쓴다면 모터 수, 프레임 크기, 중재, 버스 점유율, 피드백 주기를 더해 대역폭이 충분한지 계산한다.

가벼운 로봇의 고속 충돌 대응이나 촉각 제어는 수 kHz로 돌리기도 한다. 이때도 필요한 결정성, 대역폭, 입출력 구조를 보고 fieldbus, MCU, FPGA 가운데 고른다.

> **추천 자료**
> - FreeRTOS 공식 문서: https://www.freertos.org/
> - PREEMPT_RT Wiki: https://wiki.linuxfoundation.org/realtime/start
> - Dynamixel SDK: https://github.com/ROBOTIS-GIT/DynamixelSDK
> - IgH EtherCAT Master (Linux용 오픈소스): https://etherlab.org/en/ethercat/
> - SOEM (Simple Open EtherCAT Master): https://github.com/OpenEtherCATsociety/SOEM

---

## 4.6 심화: Workspace Analysis와 최적 설계

기구학은 로봇을 설계할 때도 쓴다. 팔이 어디까지 닿는지, 어느 자세에서 잘 움직이는지를 설계 단계에서 계산해 링크 길이와 관절 배치를 정한다.


### 4.6.1 Reachable Workspace vs Dexterous Workspace

**Reachable workspace**는 끝단이 적어도 한 가지 자세로 닿을 수 있는 점의 집합이다. **Dexterous workspace**는 끝단이 모든 자세로 닿을 수 있는 점의 집합이다. dexterous workspace는 reachable workspace 안에 들어 있고, 대개 훨씬 작다.

workspace는 Monte Carlo 방법으로 그려 본다. 관절 각도를 범위 안에서 무작위로 뽑고, FK로 끝단 위치를 계산해 점구름(point cloud)으로 모은다.

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

자코비안의 condition number κ는 특정 자세에서 끝단이 방향마다 얼마나 고르게 움직이는지 보여 준다.

```
κ(J) = σ_max / σ_min
```

σ_max, σ_min은 자코비안의 가장 큰 특이값과 가장 작은 특이값(singular value)이다. κ = 1이면 모든 방향으로 고르게 움직이는 등방(isotropic) 자세다. 특이점에 다가가면 κ가 무한대로 커지고, 끝단이 움직이지 못하는 방향이 생긴다.

로봇을 설계할 때 작업 영역 전체에서 condition number가 작아지도록 링크 길이와 배치를 고르기도 한다. 이를 kinematic optimization 또는 optimal design이라 한다.

자코비안에는 선속도(m/s) 행과 각속도(rad/s) 행이 섞여 있어서, 그대로 condition number를 구하면 단위가 다른 값을 비교하게 된다. 특성 길이(characteristic length)로 각속도 행의 단위를 길이에 맞추거나, 선속도와 각속도를 따로 분석한다.


### 4.6.3 Redundancy Resolution (7-DOF Arms)

Kinova Gen3, KUKA LBR iiwa, Franka Emika Panda 같은 7-DOF 팔은 6차원 끝단 자세를 맞추고도 자유도가 하나 남는다. 이를 kinematic redundancy라 한다. 끝단 자세를 그대로 둔 채 팔 전체의 자세(configuration)를 바꿀 수 있다. 주먹 위치를 고정하고 팔꿈치만 올리거나 내려 보면 같은 움직임을 느낀다.

남는 자유도는 manipulability를 키워 특이점을 피하거나, 한계에 다가간 관절을 가운데로 돌리거나, 팔꿈치를 장애물에서 비키거나, 토크가 덜 드는 자세를 고르는 데 쓴다. 이 움직임은 자코비안의 null space 안에서 일어난다.

```
q̇ = J† * ẋ + (I - J† * J) * q̇_0
```

첫째 항은 원하는 끝단 속도를 내는 관절 속도 가운데 norm이 가장 작은 것이다. 둘째 항의 (I - J†J)는 null space projector로, 끝단 속도를 바꾸지 않는 관절 움직임만 남긴다. q̇_0에는 manipulability 최대화 같은 2차 목적의 그래디언트를 넣는다.

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
> - Siciliano et al., *Robotics: Modelling, Planning and Control*, §3.4–3.5 — 여유 자유도 분석과 역미분 기구학(redundancy resolution 포함)
> - Nakamura, *Advanced Robotics: Redundancy and Optimization* (Addison-Wesley, 1991) — 여유 자유도 로봇의 기구학과 최적화를 다룬 책
> - Dietrich et al., "An Overview of Null Space Projections for Redundant, Torque-Controlled Robots" (2015)
> - Franka Emika 연구 인터페이스: https://frankarobotics.github.io/docs/

---

## 4.7 심화: 확률적 운동 모델 (Probabilistic Motion Models)

### 4.7.1 도입: 결정론에서 확률로

§4.2와 §4.3의 모델은 결정론적이다. 관절 각도를 넣으면 끝단 위치가 하나 나오고, 끝단 위치를 넣으면 관절 각도 해가 정해진 개수만큼 나온다. 실제 매니퓰레이터에는 관절 백래시, 링크가 휘는 정도(compliance), 캘리브레이션 오차가 있어 모델과 어긋난다.

바퀴로 움직이는 모바일 로봇은 명령한 속도대로 굴러가지 않는다. 바퀴가 미끄러지고, 마모로 실제 반지름이 줄며, 좌우 바퀴가 다르게 닳으면 직진해도 한쪽으로 휜다. 그래서 제어 명령 $u_t$를 내린 뒤의 pose $x_t$를 확률 분포로 쓴다. 이 분포를 확률적 운동 모델(probabilistic motion model)이라 한다.

상태는 평면 위의 pose $x_t = (x, y, \theta)^T \in SE(2)$다. 운동 모델은 이전 pose $x_{t-1}$과 제어 입력 $u_t$를 알 때 다음 pose의 조건부 확률 분포

$$p(x_t \mid u_t, x_{t-1})$$

를 정한다. 이 분포는 제어 입력을 무엇으로 보느냐에 따라 두 모델로 나뉜다.

**Velocity model**은 선속도와 각속도 명령 $u_t = (v, \omega)^T$를 제어 입력으로 본다. 명령 속도와 실제 속도의 차이를 노이즈로 모델링한다. 움직이기 전에 명령만으로 계산하므로 계획에도 쓴다.

**Odometry model**은 바퀴 엔코더 회전을 기구학 모델로 적분한 두 pose $u_t = (\bar{x}_{t-1}, \bar{x}_t)$를 제어 입력으로 본다. 움직인 뒤에야 얻는 값이라 추정에 쓴다. Thrun 등의 *Probabilistic Robotics*는 실제 경험상 odometry가 velocity보다 대개 더 정확하다고 적었다.

모델마다 쓰는 방식이 둘이다. 폐쇄형 밀도 평가(closed-form density evaluation)는 가설 pose $x_t$를 넣으면 그 확률밀도를 계산해 준다. 베이즈 필터의 적분을 격자 위에서 직접 계산하는 격자 국지화(grid localization)에 필요하다. 샘플링(sampling)은 다음 pose를 하나 뽑는 forward 시뮬레이션이고, particle filter(MCL)가 이 방식을 쓴다. EKF와 UKF는 둘 다 쓰지 않고 자코비안이나 시그마점으로 평균과 공분산을 넘긴다.


### 4.7.2 Velocity Motion Model — 폐쇄형

노이즈가 없으면 선속도 $v$와 각속도 $\omega$로 움직이는 로봇은 원호(circular arc)를 그리고, $\omega = 0$이면 직선으로 간다. 폐쇄형 평가는 거꾸로 계산한다. 두 pose $x_{t-1}$과 $x_t$를 잇는 원호의 회전 중심 $(x_c, y_c)$와 반지름 $r^*$를 구하고(PR 표와 아래 의사 코드는 중심을 $(x^*, y^*)$로 적는다), 그 호를 그리려면 필요했을 속도 $(\hat{v}, \hat{\omega})$를 계산한다. 이 속도가 명령 속도 $(v, \omega)$와 얼마나 다른지를 노이즈 분포로 평가한다.

**수식.** 두 pose $x_{t-1} = (x, y, \theta)^T$와 가설 $x_t = (x', y', \theta')^T$가 주어졌을 때:

$$\mu = \frac{1}{2} \cdot \frac{(x - x')\cos\theta + (y - y')\sin\theta}{(y - y')\cos\theta - (x - x')\sin\theta}$$

$$x_c = \frac{x + x'}{2} + \mu(y - y'), \quad y_c = \frac{y + y'}{2} + \mu(x' - x)$$

$$r^* = \sqrt{(x - x_c)^2 + (y - y_c)^2}$$

$$\Delta\theta = \text{atan2}(y' - y_c,\ x' - x_c) - \text{atan2}(y - y_c,\ x - x_c)$$

$$\hat{v} = \frac{\Delta\theta \cdot r^*}{\Delta t}, \quad \hat{\omega} = \frac{\Delta\theta}{\Delta t}, \quad \hat{\gamma} = \frac{\theta' - \theta}{\Delta t} - \hat{\omega}$$

노이즈는 명령에 더해지고, 그 분산은 명령 크기에 비례한다. `prob(a, b)`의 두 번째 인자인 분산 $b$는 다음과 같다.

$$b_v = \alpha_1|v| + \alpha_2|\omega|, \quad b_\omega = \alpha_3|v| + \alpha_4|\omega|, \quad b_\gamma = \alpha_5|v| + \alpha_6|\omega|$$

$b_v$는 선속도, $b_\omega$는 각속도, $b_\gamma$는 최종 회전의 노이즈 분산이다. $\hat{\gamma}$는 도착한 뒤 방향을 한 번 더 돌리는 항이다. 노이즈 변수가 $(v, \omega)$ 둘뿐이면 가설 pose가 3차원 pose 공간 안의 2차원 곡면에만 놓이고, 그 밖의 pose는 확률이 0이 된다(degeneracy). $\hat{\gamma}$를 더하면 3차원 공간 전체에 확률이 퍼진다.

$\alpha_1, \alpha_2$는 선속도 노이즈, $\alpha_3, \alpha_4$는 각속도 노이즈, $\alpha_5, \alpha_6$는 최종 회전 노이즈의 분산 가중치다. 빠르게 움직일수록 분산이 커진다. $\alpha_i$는 로봇마다 다르므로 직선, 원, 8자 주행 데이터로 맞춘다.

*Probabilistic Robotics*(이하 PR) Table 5.1의 `motion_model_velocity`를 옮기면 다음과 같다.

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

`prob(a, b)`는 평균 0, 분산 $b$인 정규 분포나 삼각 분포에서 $a$의 밀도를 돌려준다(PR Table 5.2).

### 4.7.3 Velocity Motion Model — 샘플링

샘플링은 같은 노이즈 파라미터로 노이즈를 먼저 뽑아 명령 속도에 더하고, 흔들린 속도로 원호를 따라가 다음 pose를 하나 만든다. Particle filter는 입자마다 이 샘플을 하나씩 뽑는다. 역산이 없어서 폐쇄형보다 구현이 짧다.

노이즈를 더한 제어 입력은 다음과 같다.

$$\hat{v} = v + \text{sample}(\alpha_1|v| + \alpha_2|\omega|)$$
$$\hat{\omega} = \omega + \text{sample}(\alpha_3|v| + \alpha_4|\omega|)$$
$$\hat{\gamma} = \text{sample}(\alpha_5|v| + \alpha_6|\omega|)$$

이 속도로 $\Delta t$ 동안 원호를 따라가면 다음 pose가 나온다.

$$x' = x - \frac{\hat{v}}{\hat{\omega}}\sin\theta + \frac{\hat{v}}{\hat{\omega}}\sin(\theta + \hat{\omega}\Delta t)$$
$$y' = y + \frac{\hat{v}}{\hat{\omega}}\cos\theta - \frac{\hat{v}}{\hat{\omega}}\cos(\theta + \hat{\omega}\Delta t)$$
$$\theta' = \theta + \hat{\omega}\Delta t + \hat{\gamma}\Delta t$$

$|\hat{\omega}|$가 0에 가까우면 위 식이 발산하므로, 구현에서는 직선 이동 $x' = x + \hat{v}\cos\theta\,\Delta t,\ y' = y + \hat{v}\sin\theta\,\Delta t$로 바꿔 계산한다. PR Table 5.3에는 이 분기가 없다.

`sample(b)`는 평균 0, 분산 $b$인 분포에서 값을 하나 뽑는다. 정규 분포는 균등 난수 12개를 더해 $\frac{\sqrt{b}}{2}\sum_{i=1}^{12}\text{rand}(-1,1)$로 근사한다. rand(−1,1) 12개의 합은 분산이 4라서 $\sqrt{b}/2$를 곱하면 분산이 $b$가 된다.

PR Table 5.3의 `sample_motion_model_velocity`에 직선 분기를 더하면 다음과 같다.

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

```python
import numpy as np

def sample_normal(b):
    """분산 b의 zero-mean 정규 근사 표본 (12개 균등 합)."""
    return (np.sqrt(b) / 2.0) * sum(np.random.uniform(-1, 1) for _ in range(12))

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

### 4.7.4 Odometry Motion Model — 폐쇄형

Odometry model은 엔코더로 얻은 두 pose $u_t = (\bar{x}_{t-1}, \bar{x}_t)$ 사이의 상대 운동을 세 값 $(\delta_{\text{rot1}}, \delta_{\text{trans}}, \delta_{\text{rot2}})$으로 나눈다. 로봇이 목적지 쪽으로 먼저 돌고($\delta_{\text{rot1}}$), 곧게 간 다음($\delta_{\text{trans}}$), 도착해서 마지막 방향으로 한 번 더 돈다($\delta_{\text{rot2}}$)고 보는 분해다. 평면 위의 어떤 상대 운동이든 이 세 값으로 나타낼 수 있다.

odometry는 원래 센서 측정값이다. 측정으로 다루려면 실제 속도를 상태 변수에 넣어야 해서 상태 공간의 차원이 커진다. 그래서 PR은 odometry를 제어 입력처럼 다룬다(PR §5.4.1).

odometry 값 $u_t = (\bar{x}_{t-1}, \bar{x}_t)$에서 $\bar{x}_{t-1} = (\bar{x}, \bar{y}, \bar{\theta})^T$, $\bar{x}_t = (\bar{x}', \bar{y}', \bar{\theta}')^T$라 하면 상대 운동은 다음처럼 구한다.

$$\delta_{\text{rot1}} = \text{atan2}(\bar{y}' - \bar{y},\ \bar{x}' - \bar{x}) - \bar{\theta}$$
$$\delta_{\text{trans}} = \sqrt{(\bar{x} - \bar{x}')^2 + (\bar{y} - \bar{y}')^2}$$
$$\delta_{\text{rot2}} = \bar{\theta}' - \bar{\theta} - \delta_{\text{rot1}}$$

노이즈 파라미터는 $\alpha_1$부터 $\alpha_4$까지 넷이다. `prob()`의 분산은 가설 pose에서 거꾸로 구한 $(\hat\delta_{\text{rot1}}, \hat\delta_{\text{trans}}, \hat\delta_{\text{rot2}})$로 계산한다.

$$b_{\text{rot1}} = \alpha_1|\hat\delta_{\text{rot1}}| + \alpha_2|\hat\delta_{\text{trans}}|$$
$$b_{\text{trans}} = \alpha_3|\hat\delta_{\text{trans}}| + \alpha_4(|\hat\delta_{\text{rot1}}| + |\hat\delta_{\text{rot2}}|)$$
$$b_{\text{rot2}} = \alpha_1|\hat\delta_{\text{rot2}}| + \alpha_2|\hat\delta_{\text{trans}}|$$

$\alpha_1$은 회전량이 회전 오차에, $\alpha_2$는 직진 거리가 회전 오차에, $\alpha_3$은 직진 거리가 직진 오차에, $\alpha_4$는 회전량이 직진 오차에 주는 영향의 크기다. 노이즈 변수가 이미 셋이라 velocity model의 $\hat{\gamma}$ 같은 항을 따로 넣지 않아도 3차원 pose 공간 전체에 확률이 퍼진다.

각도 차는 반드시 $[-\pi, \pi]$ 범위로 접어 넣는다. 이걸 빠뜨리면 가끔 계산이 발산하는데, PR도 흔히 저지르는 실수로 꼽는다.

PR Table 5.5의 `motion_model_odometry`는 다음과 같다.

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

### 4.7.5 Odometry Motion Model — 샘플링

샘플링은 odometry에서 구한 $(\delta_{\text{rot1}}, \delta_{\text{trans}}, \delta_{\text{rot2}})$에 노이즈를 더하고, 그 값으로 돌고 가고 돌아 새 pose를 만든다. 가설 pose에서 거꾸로 계산하는 단계가 없어 폐쇄형보다 구현이 짧다.

PR 식 (5.37)~(5.39)로 각 성분에 노이즈를 더하고, 새 pose는 식 (5.40)으로 구한다.

$$\begin{pmatrix}x'\\y'\\\theta'\end{pmatrix} = \begin{pmatrix}x\\y\\\theta\end{pmatrix} + \begin{pmatrix}\hat{\delta}_{\text{trans}}\cos(\theta + \hat{\delta}_{\text{rot1}})\\\hat{\delta}_{\text{trans}}\sin(\theta + \hat{\delta}_{\text{rot1}})\\\hat{\delta}_{\text{rot1}} + \hat{\delta}_{\text{rot2}}\end{pmatrix}$$

운동을 원호 대신 회전, 직진, 회전으로 근사한다. $\Delta t$가 짧으면 원호와 거의 같다. 각속도로 나누는 항이 없어서 velocity 샘플링처럼 각속도가 0일 때를 따로 처리할 필요도 없다.

PR Table 5.6의 `sample_motion_model_odometry`는 다음과 같다.

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

ROS2 Nav2의 `nav2_amcl`은 이 알고리즘을 `differential` motion model로 구현했다.

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
        return (np.sqrt(b) / 2.0) * sum(np.random.uniform(-1, 1) for _ in range(12))

    dh_rot1  = d_rot1  - sample_normal(alpha[0]*abs(d_rot1)  + alpha[1]*abs(d_trans))
    dh_trans = d_trans - sample_normal(alpha[2]*abs(d_trans) + alpha[3]*(abs(d_rot1) + abs(d_rot2)))
    dh_rot2  = d_rot2  - sample_normal(alpha[0]*abs(d_rot2)  + alpha[1]*abs(d_trans))

    x_new     = x + dh_trans * np.cos(theta + dh_rot1)
    y_new     = y + dh_trans * np.sin(theta + dh_rot1)
    theta_new = theta + dh_rot1 + dh_rot2

    return x_new, y_new, theta_new
```

<!-- DEMO: probabilistic_motion_model.html -->

### 4.7.6 Motion + Map: 지도 조건부 운동 모델

앞의 네 알고리즘은 지도를 쓰지 않았다. localization에서는 지도 $m$이 있으니, 벽 속처럼 로봇이 있을 수 없는 pose를 걸러 낸다.

지도까지 조건으로 넣은 전이 분포를 정확히 계산하기는 어렵다. PR은 다음 근사를 쓴다.

$$p(x_t \mid u_t, x_{t-1}, m) \propto p(x_t \mid u_t, x_{t-1}) \cdot p(x_t \mid m)$$

앞 항 $p(x_t \mid u_t, x_{t-1})$은 §4.7.2~§4.7.5의 운동 모델이다. 뒷 항 $p(x_t \mid m)$은 지도에서 그 pose가 가능한 정도로, occupancy grid에서 $x_t$가 빈 칸(free cell)이면 1에 가깝고 벽이나 점유된 칸이면 0에 가깝다.

이 근사를 쓰면 particle filter의 입자가 벽 안에 놓이지 않는다. 가장 단순하게는 샘플링한 새 pose를 지도에서 찾아보고, 점유된 칸이면 그 입자의 weight를 0이나 아주 작은 값으로 둔다. 끝점만 보므로 시간 간격이 길면 벽을 뚫고 지나간 입자도 걸러지지 않는다.

occupancy grid에는 빈 칸과 점유된 칸 말고도 아직 관측하지 않은 unknown 칸이 있다. unknown 칸에서 $p(x_t \mid m)$을 1로 둘지 중간값으로 둘지에 따라 localization 결과가 달라진다. ROS2 Nav2 costmap은 기본값(`track_unknown_space: false`)에서 unknown 칸을 free로 취급한다. AMCL은 unknown 칸을 따로 구분한다.

이 곱 분해는 운동 모델과 지도 prior가 서로 독립이라고 가정할 때만 정확하다. 근거가 되는 베이즈 정리와 조건부 독립은 §3.3에서 다뤘다.

### 4.7.7 지금 쓰이는 곳

바퀴 로봇의 particle filter localization은 지금도 이 샘플링 모델을 motion prior(움직임에서 오는 사전 분포)로 쓴다. ROS2 Nav2 `nav2_amcl`의 `differential` 모델은 odometry 샘플링이다. 입자 수와 갱신 주기는 지도 크기, 센서 갱신 주기, CPU, 빔 수, 오차 파라미터를 보고 직접 재서 정한다.

여기서 다룬 모델은 모두 평면(SE(2))에서 차동 구동 바퀴로 움직이는 로봇을 가정한다. 휴머노이드, 보행 로봇, 드론은 차체 속도 $(v, \omega)$로 운동을 쓰기 어렵다. 발로 걷는 로봇은 미끄러지는 방식이 바퀴와 다르고, 드론은 SE(3)에서 움직인다. 이런 로봇은 IMU preintegration으로 motion prior를 만든다(§14.10). 식 $p(x_t \mid u_t, x_{t-1})$의 꼴은 같고 안에 들어가는 모델이 다르다. Mecanum 바퀴를 단 holonomic 로봇이나 옆으로 미끄러지는 차량도 따로 모델을 세워야 한다.

§14.7.6 Monte Carlo Localization(MCL)은 prediction 단계에서 §4.7.3·§4.7.5의 샘플링 모델(`sample_motion_model`)을 부른다. §3.10·§3.11의 가우시안 필터와 비모수 필터도 $p(x_t \mid u_t, x_{t-1})$를 prediction 항으로 쓴다.

---

## 4.8 추천 자료

**교재:**

- Craig, "Introduction to Robotics: Mechanics and Control" — modified DH convention으로 기구학을 설명한다.

- Lynch & Park, "Modern Robotics: Mechanics, Planning, and Control" — PoE로 기구학을 설명한다. 무료 PDF와 Coursera 강의가 있다. Lie group 표기에 익숙하지 않으면 처음에는 낯설다. https://modernrobotics.org

- Corke, "Robotics, Vision and Control" — 코드를 돌리며 기구학을 익힌다. 3판은 Python 기반이고, 동반 라이브러리가 robotics-toolbox-python이다. https://petercorke.com/rvc/

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
1970~80년대 ── Harmonic Drive가 산업용 로봇 관절에 널리 쓰이기 시작 (1971~73년 유럽 로봇 채택, 1977년 로봇용 R 시리즈)
1984 ── Product of Exponentials 공식 (Brockett)
2019 ── MIT Mini Cheetah: QDD 액추에이터
2019 ── MoveIt2 (ROS2 기반 모션 플래닝 프레임워크)
2023 ── ALOHA: 저비용 양팔 텔레오퍼레이션 플랫폼
2024 ── SO-ARM100: 공개 BOM과 조립 문서를 제공한 오픈소스 5축 로봇 팔
```
