# 4장 기구학과 메카트로닉스

3장에서 회전 행렬, 동차 변환, Lie 군, 확률 분포 같은 수학 도구를 마련했다. 이제 그 수학으로 책상 위 로봇 팔을 실제로 움직여 볼 차례다. 모터 여섯 개에 각각 몇 도를 줘야 손끝이 커피잔 손잡이에 정확히 닿을까? 이 기하학적 관계를 푸는 분야가 **기구학(kinematics)**이다. 그리고 계산해 낸 각도 명령을 실제 전류로 바꾸어 모터를 돌리고, 엔코더 값을 읽어 1kHz 제어 루프를 빈틈없이 돌리는 일은 하드웨어와 임베디드의 영역인 **메카트로닉스(mechatronics)**가 맡는다.

---

## 4.1 기구학이 쓰이는 곳

로봇 매니퓰레이터는 여러 개의 강체 링크(link)를 회전하거나 직선으로 움직이는 관절(joint)로 이어 붙인 사슬 구조다. 사람이 작업을 설계할 때 원하는 값은 손끝(end-effector)이 3차원 공간 어디에 어떤 자세(pose)로 놓이는지다. 반면 모터 제어기가 직접 받아들이는 명령은 각 관절의 각도나 직선 변위뿐이다. 기구학은 이 두 세계—작업 공간의 위치·자세와 관절 공간의 변수—사이를 잇는 기하학적 번역기다.

- **순기구학(Forward Kinematics, FK)**: 관절 각도 → 끝단 위치와 자세
- **역기구학(Inverse Kinematics, IK)**: 끝단 위치와 자세 → 관절 각도

기구학은 물체의 질량이나 관성, 중력, 마찰을 따지지 않고 오직 위치와 자세의 기하만 다룬다. 그 자세를 유지하거나 빠르게 움직이기 위해 모터가 얼마나 큰 힘과 토크를 내야 하는지는 동역학(dynamics)의 영역이다(→ 5장).

기하학만 다루는데도 기구학은 로봇 소프트웨어 파이프라인 곳곳에 들어간다. 로봇 팔이 장애물을 피해 목적지까지 가는 경로를 짤 때(motion planning)도, 원격 조종에서 조종자의 마스터 팔 움직임을 현장의 슬레이브 팔 자세로 옮길 때(teleoperation)도 기구학이 필요하다. 또한 공장에서 조립된 로봇의 미세한 치수 오차를 보정하는 캘리브레이션이나, 링크들이 서로 또는 주변 물체와 부딪히지 않는지 링크별 공간 좌표를 실시간으로 확인하는 충돌 회피 역시 기구학 계산 위에서 돌아간다.

---

## 4.2 순기구학

순기구학은 관절에 달린 엔코더가 지금 몇 도를 가리키고 있는지 알 때, 로봇 손끝이 공간의 어느 좌표에 어떤 방향으로 놓여 있는지를 구하는 문제다.

### 4.2.1 동차 변환 행렬 (Homogeneous Transformation Matrix)

관절 하나를 건널 때마다 좌표계는 회전하고 동시에 이동한다. 이 두 변화를 따로 추적하면 링크가 여섯 개만 되어도 식이 금세 엉킨다. 3장 §3.2.3에서 다룬 4×4 동차 변환 행렬 $SE(3)$을 쓰면 회전과 평행이동을 하나의 행렬 안에 묶어 다룰 수 있다.

```
T = | R  p |
    | 0  1 |
```

여기서 `R`은 3×3 회전 행렬, `p`는 3×1 위치 벡터다. 동차 변환 행렬의 가장 큰 장점은 여러 변환을 단순한 행렬 곱으로 이어 붙일 수 있다는 점이다. 베이스 프레임 0에서 링크 1로 가는 변환 `T_01`을 적용한 뒤, 그 새 좌표계를 기준으로 링크 2로 가는 변환 `T_12`를 곱하면 곧바로 베이스에서 링크 2까지의 전체 자세가 나온다.

```
T_02 = T_01 * T_12
```

따라서 직렬 로봇 팔의 순기구학은 베이스부터 끝단까지 관절마다 계산한 변환 행렬을 순서대로 곱해 나가는 연쇄 계산에 지나지 않는다.

```
q = [θ_1, θ_2, ..., θ_n]^T  →  T = [R, t; 0, 1] ∈ SE(3)
```

입력인 관절 각도 벡터 `q`가 정해지면 행렬 곱의 결과는 단 하나로 결정되므로, 순기구학에서는 해가 여럿 나오거나 해가 없는 경우가 생기지 않는다.

### 4.2.2 Denavit-Hartenberg (DH) 파라미터

3차원 공간에서 두 좌표계의 상대 자세를 아무런 제약 없이 적으려면 이동 3개, 회전 3개로 총 6개의 파라미터가 필요하다. 하지만 로봇 링크를 잇는 관절 축을 기준으로 좌표계를 규칙에 맞춰 배치하면, 이웃한 두 링크의 기하학적 관계를 단 4개의 숫자로 압축할 수 있다. 1955년 Denavit과 Hartenberg가 제안한 이 규약이 바로 **DH 파라미터**다.

| 파라미터 | 의미 | 설명 |
|---------|------|------|
| $a_i$ | 링크 길이 | $z_{i-1}$과 $z_i$ 사이의 $x_i$ 축을 따른 거리 |
| $\alpha_i$ | 링크 꼬임각 | $z_{i-1}$과 $z_i$ 사이의 $x_i$ 축 기준 회전각 |
| $d_i$ | 링크 오프셋 | $x_{i-1}$과 $x_i$ 사이의 $z_{i-1}$ 축을 따른 거리 |
| $\theta_i$ | 관절각 | $x_{i-1}$과 $x_i$ 사이의 $z_{i-1}$ 축 기준 회전각 |

네 값 가운데 로봇이 움직일 때 변하는 값은 관절 종류에 따라 딱 하나뿐이다. 회전 관절(revolute joint)에서는 $\theta_i$만 변하고 나머지 세 값은 기계를 제작할 때 정해진 고정 치수다. 직선 관절(prismatic joint)에서는 $d_i$만 변한다. 네 파라미터를 $z$축 회전·이동과 $x$축 이동·회전 순서로 엮으면 관절 하나를 건너는 4×4 변환 행렬이 완성된다.

```
T_i = Rot_z(θ_i) * Trans_z(d_i) * Trans_x(a_i) * Rot_x(α_i)

    = | cos(θ)  -sin(θ)cos(α)   sin(θ)sin(α)   a*cos(θ) |
      | sin(θ)   cos(θ)cos(α)  -cos(θ)sin(α)   a*sin(θ) |
      | 0        sin(α)          cos(α)          d        |
      | 0        0               0               1        |
```

현장에서 DH 파라미터를 가져다 쓸 때 가장 흔히 겪는 함정은 **규약(convention) 혼동**이다. DH 규약은 링크 $i$의 좌표계를 관절 $i+1$에 두는 standard 방식과, 관절 $i$에 두는 modified 방식(Craig 교과서가 사용해 Craig convention으로도 불린다)으로 갈린다. 프레임 위치가 다르다 보니 같은 로봇이라도 $a_i$와 $\alpha_i$의 번호가 한 칸씩 어긋나고 변환 행렬 곱셈 순서도 달라져, 둘을 섞어 쓰면 끝단 위치가 엉뚱한 곳으로 계산된다. 데이터시트나 오픈소스 모델을 참고할 때는 어느 규약으로 작성된 표인지부터 확인하고 기록해 두어야 한다.


### 4.2.3 예제: 2-link Planar Arm의 FK

4×4 행렬 곱이 실제로 어떤 삼각함수로 풀리는지 확인하기 위해, 평면 위에서 움직이는 가장 단순한 2-링크 로봇 팔을 살펴보자.

```
       q1         q2
  O────────O────────O → end-effector
  (base)   L1       L2
```

두 관절의 회전축이 모두 지면에서 수직으로 튀어나온 방향($z$축)으로 나란히 놓여 있으므로, 링크 꼬임각 $\alpha$와 오프셋 $d$는 모두 0이 된다. Standard convention으로 DH 테이블을 적으면 아래와 같다.

| Link | a    | α   | d   | θ    |
|------|------|-----|-----|------|
| 1    | L1   | 0   | 0   | θ_1  |
| 2    | L2   | 0   | 0   | θ_2  |

두 링크의 변환 행렬 `T_1`과 `T_2`를 곱해 마지막 열의 위치 성분을 읽으면, 고등학교 삼각함수로 유도한 결과와 똑같은 식이 떨어진다.

```
x = L1*cos(θ_1) + L2*cos(θ_1 + θ_2)
y = L1*sin(θ_1) + L2*sin(θ_1 + θ_2)
```

이를 Python 함수로 옮기면 몇 줄로 끝난다.

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

6축이나 7축 산업용 로봇 팔의 순기구학도 본질은 다르지 않다. 관절마다 만든 4×4 행렬을 여섯 번, 혹은 일곱 번 차례대로 곱하면 끝단 위치와 자세가 나온다.


### 4.2.4 Product of Exponentials (PoE)

DH 파라미터는 오래 쓰인 표준이지만, 각 링크마다 특정 기하 조건에 맞춰 중간 좌표계를 일일이 세워야 해서 축이 비스듬한 로봇에서는 좌표계 잡기가 까다롭다. 이와 달리 **PoE(Product of Exponentials)**는 3장 §3.5에서 배운 Lie 군과 Lie 대수를 직접 활용해 순기구학을 훨씬 깔끔하게 기술한다. Brockett이 1983년 MTNS 학회에서 발표하고 1984년에 출간한 공식이며, Lynch와 Park의 *Modern Robotics*가 이 체계를 주축으로 삼는다.

PoE의 핵심 발상은 직관적이다. 중간 링크마다 좌표계를 붙이는 대신, 모든 관절 각도가 0인 기준 자세(home configuration)에서 끝단의 자세 `M`만 딱 하나 정의해 둔다. 그다음 각 관절이 공간 속에서 나사(screw)처럼 회전·이동하는 축 벡터 `S_i`를 잡고, 3장 §3.5.4의 행렬 지수(matrix exponential)로 각 관절의 회전량 $\theta_i$만큼 `M`을 차례로 밀고 돌린다.

```
T(θ) = e^{[S_1]θ_1} * e^{[S_2]θ_2} * ... * e^{[S_n]θ_n} * M
```

식 속 기호의 의미는 다음과 같다.

- `S_i`는 `i`번째 관절의 screw axis로, 회전축 방향과 원점 기준 모멘트(또는 직선 이동 방향)를 묶은 6×1 벡터다.
- `[S_i]`는 `S_i`의 4×4 행렬 표현으로, `se(3)`의 원소이며 왼쪽 위 3×3 회전 블록이 skew-symmetric 행렬이다.
- `M`은 모든 관절이 영 위치(home configuration)일 때의 끝단 자세다.
- `θ_i`는 각 관절의 변수(각도 또는 변위)다.

DH와 PoE는 같은 기하학을 서로 다른 언어로 풀어낸다. 아래 표처럼 장단점이 뚜렷하다.

| 항목 | DH | PoE |
|------|-----|-----|
| 프레임 부착 | 각 링크에 프레임 필요 | 기준 프레임과 끝단 프레임만 필요 |
| Convention 혼동 | standard vs modified 주의 | 없음 (space form vs body form 구분은 있음) |
| 수학적 기반 | 행렬 곱 | Lie group, 행렬 지수 |
| 산업계 채택 | 매우 높음 | 학계 중심, 점점 확산 |
| 교재 | Craig, Siciliano | Lynch & Park |

현장에서는 세 가지 표기를 한꺼번에 마주치게 된다. 산업용 로봇 매뉴얼과 전통 교과서는 대부분 DH 파라미터를 싣고, ROS의 로봇 기술 파일(URDF)은 관절마다 부모 링크에서 자식 링크로 가는 6-자유도 상대 변환(`xyz`, `rpy`)을 직접 적으며, 최신 로보틱스 논문과 미분 가능한 물리 엔진은 PoE를 즐겨 쓴다. 매뉴얼의 스펙을 시뮬레이터나 논문 수식에 옮겨 쓰려면 이 표기들을 서로 변환해 읽을 줄 알아야 한다. 아래는 `robotics-toolbox-python`으로 고전적인 Puma 560 6축 로봇의 DH 기반 순기구학을 계산하는 예시다.

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

## 4.3 역기구학

순기구학에서는 관절 각도가 정해지면 손끝의 위치와 자세가 곧바로 하나로 떨어졌다. 하지만 실제 작업에서 우리가 먼저 손에 쥐고 있는 정보는 반대다. "손끝을 `(x, y, z)`에 놓으려면 모터 여섯 개를 각각 몇 도로 돌려야 하는가?" 이 물음을 푸는 과정이 **역기구학(Inverse Kinematics, IK)**이다.

방향을 거꾸로 뒤집는 순간 문제는 훨씬 까다로워진다. 방정식에 사인과 코사인이 얽힌 비선형 식이 등장하는 데다, 해의 개수도 하나로 보장되지 않는다. 같은 지점에 손끝을 대더라도 팔꿈치를 위로 치켜세운 자세(elbow-up)와 아래로 내린 자세(elbow-down)처럼 여러 관절 조합이 가능하다. 반대로 팔이 닿을 수 있는 작업 범위(workspace) 바깥의 점을 주문하면 해가 아예 없고, 작업에 필요한 자유도보다 관절 수가 많은 여유 자유도 팔(redundant manipulator)에서는 해가 무수히 쏟아져 나온다.


### 4.3.1 해석적 방법 (Analytical IK)

가장 이상적인 풀이는 관절 각도를 닫힌 형태(closed-form)의 수식 하나로 곧장 계산하는 **해석적 방법**이다. 해가 존재하는 경우 반복 탐색 없이 가능한 후보 전체를 즉시 뽑아낼 수 있어, 1ms 단위로 돌아가는 고속 제어 루프에서 특히 빛을 발한다. 다만 실제 구현에서의 수치 정확도와 실행 시간은 특이점 근처 분기 처리와 삼각함수 계산 방식에 따라 달라진다.

**2-link planar arm의 IK:**

평면 위 2-링크 팔에서는 기하학만으로 깔끔한 닫힌 해를 얻는다. 베이스와 목표점 `(x, y)`를 잇는 직선과 두 링크 `L1`, `L2`가 이루는 삼각형에 코사인 제2법칙을 적용하면 먼저 팔꿈치 각도 $\theta_2$가 풀리고, 이어서 전체 기울기에서 링크 2의 편향을 빼 어깨 각도 $\theta_1$이 나온다.

```
cos(θ_2) = (x² + y² - L1² - L2²) / (2 * L1 * L2)
θ_2 = atan2(±√(1 - cos²(θ_2)), cos(θ_2))

θ_1 = atan2(y, x) - atan2(L2*sin(θ_2), L1 + L2*cos(θ_2))
```

$\theta_2$ 식의 `±` 부호가 바로 팔꿈치를 위로 꺾은 자세(elbow-up)와 아래로 꺾은 자세(elbow-down)를 가른다. 수학적으로는 둘 다 정답이지만, 실제 로봇에서는 주변 장애물이나 직전 관절 각도와의 거리를 따져 이 가운데 하나만 골라야 한다. 아래 코드는 목표점의 도달 가능 여부를 먼저 걸러 낸 뒤 두 가지 해 중 하나를 계산하고, 앞서 작성한 FK에 다시 넣어 오차를 검증한다.

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

관절이 여섯 개인 3차원 로봇 팔도 모두 해석적으로 풀리는 것은 아니다. 1968년 Pieper는 6축 로봇에서 마지막 세 회전 관절의 축이 공간의 한 점에서 만나는 **구형 손목(spherical wrist)** 구조를 갖추면 닫힌 해가 존재함을 증명했다. KUKA나 ABB의 산업용 6축 로봇 상당수가 바로 이 구조를 채택한다. 손목 축 세 개가 한 점에서 만나면 끝단 방향을 아무리 돌려도 손목 중심점의 위치는 바뀌지 않기 때문에, 앞선 세 관절로 손목 중심의 위치를 맞추고 마지막 세 관절로 손끝의 자세를 맞추는 두 개의 3-자유도 문제로 깔끔하게 쪼개어 풀 수 있다. 이때 해는 최대 8개까지 나오며, 제어기에서는 각 관절의 회전 한계(joint limit) 안에 들어오면서 현재 관절 각도에서 가장 덜 움직여도 되는 해를 고른다. 반면 UR(Universal Robots) 계열 팔은 관절 오프셋 때문에 손목 세 축이 한 점에서 만나지 않아 Pieper 분해가 그대로 먹히지 않는다. 다행히 UR 팔 역시 구조적 대칭성을 이용해 유도한 별도의 닫힌 해가 존재한다.


### 4.3.2 수치적 방법 (Numerical IK)

관절 축이 일반적인 각도로 어긋나 닫힌 해를 구할 수 없거나, 7축 이상이라 아예 해가 무수히 많은 로봇 팔은 미분을 이용해 조금씩 정답에 다가가는 반복 계산으로 IK를 푼다.

**Jacobian Pseudo-Inverse 방법:**

아이디어는 3장 §3.4에서 본 비선형 최적화와 닮아 있다. 현재 관절 각도 `q`에서 계산한 끝단 위치와 목표 위치 사이에 오차 $\Delta x$가 남았다면, "관절을 살짝 움직였을 때 끝단이 어느 방향으로 얼마나 이동하는지" 알려 주는 자코비안 행렬 `J(q)`를 뒤집어 관절 수정량 $\Delta q$를 구하고, 오차가 허용치 아래로 떨어질 때까지 한 걸음씩 반복해 갱신한다.

```
Δq = J†(q) * Δx
```

여기서 `J†`는 정방행렬이 아닌 경우에도 역연산을 정의할 수 있는 자코비안의 의사 역행렬(pseudo-inverse)이다. 아래 코드는 2-링크 팔에 이 반복 루프를 적용한 예시다. 실제 로봇에서는 초기 추정값으로 현재 엔코더에서 읽은 관절 각도를 넣어 가장 가까운 해로 수렴하도록 유도한다.

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

단순 pseudo-inverse는 평소에는 빠르게 수렴하지만, 자코비안이 퇴화하는 특이점 근처에만 가면 역행렬의 성분이 무한대로 치솟아 관절 속도 $\Delta q$가 갑자기 폭주하는 치명적인 약점이 있다. 3장 §3.4.4의 Levenberg-Marquardt와 똑같은 원리로, 대각 성분에 작은 감쇠 계수(damping factor) $\lambda$를 더해 역행렬이 0으로 나누어지는 사태를 막는 기법이 **DLS**다.

```
Δq = J^T (J * J^T + λ²I)^{-1} * Δx
```

감쇠 계수 $\lambda$는 안정성과 수렴 속도 사이의 줄타기다. $\lambda$를 키우면 특이점 근처에서도 관절이 얌전하게 움직이는 대신 목표 지점에 다가가는 속도가 더뎌지고, $\lambda$를 0에 가깝게 줄이면 일반 pseudo-inverse와 거의 같아진다. Nakamura와 Hanafusa(1986)는 평소에는 $\lambda = 0$으로 빠르게 움직이다가 특이점에 가까워질수록 $\lambda$를 서서히 키우는 적응형 감쇠 기법을 제안했다.


### 4.3.3 특이점 (Singularity)

앞에서 여러 차례 등장한 **특이점(singularity)**을 물리적으로 그려 보자. 팔꿈치를 완전히 180도로 쭉 펴고 있는 상태에서 손끝을 팔이 뻗은 방향 그대로 1cm만 더 밀어 보라고 하면, 어깨나 팔꿈치 관절을 아무리 돌려도 그 방향으로는 단 1mm도 나아갈 수 없다. 이처럼 로봇이 특정 방향으로 움직일 능력을 잃어버리는 자세, 수학적으로는 자코비안 행렬의 rank가 떨어지는 관절 배치를 특이점이라 부른다.

특이점 자체가 나쁜 것보다 더 위험한 것은 **특이점 근처**다. 손끝을 그 막힌 방향으로 아주 조금만 움직이려 해도 기구학 식이 관절에 무한대에 가까운 속도를 요구하기 때문에, 모터가 갑자기 비명을 지르며 급가속하거나 IK 해가 다른 분기로 갈아타면서 팔 전체가 휙 튀어 버릴 수 있다.

2-link planar arm에서는 팔이 완전히 펴진 $\theta_2 = 0$과 완전히 접힌 $\theta_2 = \pi$ 두 자세가 특이점이다. 이 자세에서 끝단은 베이스와 끝단을 잇는 직선에 수직인 원주 방향으로만 움직일 수 있고, 반지름 방향 자유도를 잃는다.

구형 손목(spherical wrist)을 가진 6축 산업용 로봇에서는 세 가지 전형적인 특이점이 나타난다. 손목의 축 4와 축 6이 일직선으로 겹쳐 두 관절의 회전 효과가 같아지는 **wrist singularity**($q_5 = 0$ 또는 $\pi$), 손목 중심이 베이스의 축 1 회전축 바로 위에 올라오는 **shoulder singularity**, 그리고 팔꿈치가 완전히 펴지거나 접히는 **elbow singularity**다.

현장에서는 경로 계획 단계에서부터 특이점 주변을 피해 궤적을 짜는 것이 상책이다. 작업 경로상 부득이하게 특이점 근처를 스쳐 지나가야 한다면 DLS로 관절 속도를 묶어 폭주를 막고, 7축 이상 로봇이라면 손끝은 그대로 둔 채 남는 자유도로 팔꿈치를 비틀어 특이점을 비켜 간다(§4.6.3).


### 4.3.4 IK 솔버들

실제 로봇 프로젝트를 진행할 때 매번 IK 솔버를 처음부터 직접 짜지는 않는다. 로봇의 관절 수와 구조, 실시간 요구 주기에 맞춰 검증된 오픈소스 솔버를 골라 쓴다.

| 솔버 | 방법 | 특징 |
|------|------|------|
| KDL | Numerical (Newton-Raphson) | ROS에 기본으로 들어 있다. 관절 한계·초기값·특이점에 민감 |
| IKFast (OpenRAVE) | Analytical (코드 생성) | 로봇 구조마다 C++ 해석해 코드를 자동 생성. 빠름 |
| TRAC-IK | KDL-RR과 SQP를 동시에 실행 | KDL보다 성공률 높음, ROS 패키지 있음 |
| MoveIt2 IK | 위 솔버들을 통합 | ROS2 생태계, 충돌 회피 통합 |
| pinocchio | 자코비안 기반 수치 반복 (CLIK) | 강체 동역학 라이브러리, 빠름, 해석적 미분 제공 |

수치적 방법과 해석적 방법, 혹은 서로 다른 수치 솔버 사이의 성능 차이는 조건을 통제해 비교해 보면 드러난다.

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

## 4.4 자코비안

지금까지는 수치적 IK를 풀기 위한 보조 도구로 자코비안을 먼저 꺼내 썼다. 하지만 자코비안은 IK 방정식을 푸는 데만 쓰이는 행렬이 아니다. 끝단 **속도**를 제어하고, 손끝에 걸리는 **힘을 관절 토크로 환산**하며, 현재 자세가 얼마나 **부드럽게 움직일 수 있는 자세인지** 진단하는 미분 기구학의 중심축이 바로 자코비안이다.

### 4.4.1 관절 속도 → 끝단 속도

로봇이 정지된 목표점 하나만 찍고 멈추는 것이 아니라 궤적을 따라 매끄럽게 움직이려면, 위치가 아니라 속도를 연결해야 한다. 순기구학 식을 시간에 대해 한 번 미분하면 관절 각속도 벡터 $\dot{q}$를 끝단의 6차원 공간 속도(선속도 $v$, 각속도 $\omega$)로 옮기는 선형 사상, 즉 **자코비안(Jacobian) $J(q)$**가 나온다.

```
ẋ = J(q) * q̇

여기서 ẋ = [v; ω] ∈ ℝ^6 (6축의 경우)
      q̇ ∈ ℝ^n
      J(q) ∈ ℝ^{6×n}
```

행렬의 모양을 보면 로봇의 운동 능력이 한눈에 들어온다. 관절 수 `n`이 6보다 작으면 6차원 작업 공간 전체를 마음대로 움직일 수 없고, 반대로 6보다 크면 같은 끝단 속도를 만들어 내는 관절 속도 조합이 여럿 생겨 남는 자유도(redundancy)가 생긴다. 물론 관절이 6개 이상이더라도 특정 자세에서 실제로 끝단이 뻗어 나갈 수 있는 독립적인 방향의 개수는 `J(q)`의 rank로 결정된다. 참고로 이와 혼동하기 쉬운 **underactuated**는 기구학적 자유도 부족이 아니라, 드론이나 도립 진자처럼 독립적으로 힘을 가할 수 있는 액추에이터 수가 시스템의 전체 자유도보다 적은 동역학적 특성을 가리킨다.

한편 매니퓰레이터가 관절 속도 $\dot{q}$로 움직임을 기술하는 것과 달리, 바퀴로 주행하는 모바일 로봇은 차체의 선속도와 각속도 $(v, \omega)$를 직접 제어 명령으로 주고받는다. 이 속도 명령에 실제 주행 노이즈를 얹어 다음 위치를 예측하는 확률 모델은 §4.7에서 따로 다룬다.


### 4.4.2 힘/토크 관계 (Duality)

자코비안의 진가는 속도를 넘어 힘의 세계까지 연결한다는 데 있다. 로봇 손끝으로 책상을 아래로 10N만큼 누르거나 무거운 공구를 쥐고 버틸 때, 각 관절 모터는 얼마나 큰 토크를 내야 버틸 수 있을까? 놀랍게도 새로운 행렬을 구할 필요 없이 자코비안의 전치 행렬 $J^T(q)$만 곱하면 된다.

```
τ = J^T(q) * F
```

여기서 $\tau$는 각 관절에 걸리는 토크 벡터, $F$는 끝단이 외부 환경에 가하거나 외부로부터 받는 힘과 모멘트(wrench)다. 속도를 변환하는 $J$와 힘을 변환하는 $J^T$가 서로 거울처럼 마주 보는 이 관계를 **정역학적 이중성(static duality)**이라 부른다. 가상 일의 원리, 즉 마찰과 가속이 없는 정적 평형 상태에서 관절 모터들이 하는 일률과 끝단이 외부에 하는 일률이 같아야 한다는 에너지 보존 법칙에서 한 줄로 유도된다.

```
P = F^T * ẋ = F^T * J * q̇ = (J^T * F)^T * q̇ = τ^T * q̇
```

힘 제어(force control)나 임피던스 제어는 이 이중성 공식을 그대로 심장으로 쓴다. 예컨대 조립 작업 중 끝단으로 일정한 힘 $F$를 눌러야 한다면, 제어기는 매 주기마다 현재 관절 각도 $q$에서 자코비안을 계산한 뒤 $\tau = J^T(q) F$만큼 각 관절 모터에 전류(토크)를 인가한다.


### 4.4.3 Manipulability Ellipsoid

같은 크기의 관절 속도($\|\dot{q}\| = 1$)로 모터를 돌려도 로봇 팔의 자세에 따라 어떤 방향으로는 손끝이 시원하게 뻗어 나가고 어떤 방향으로는 좀처럼 움직이지 못한다. 단위 구(unit sphere) 크기의 관절 속도를 자코비안 $J(q)$로 통과시켰을 때 끝단 속도 공간에 그려지는 모양이 바로 **조작성 타원체(manipulability ellipsoid)**다. 이 타원체의 부피를 하나의 숫자로 요약한 값이 조작성 지수(manipulability index)다.

```
manipulability index = √det(J * J^T)
```

6차원 작업 공간에서 보통 full row rank를 갖는 매니퓰레이터라면, 이 지수가 0으로 떨어지는 자세가 곧 타원체가 납작하게 찌그러져 자유도를 잃는 특이점이다. 지수가 클수록 타원체의 부피가 커져 전반적으로 움직임의 여유가 많다는 뜻이 되지만, 부피만 크다고 해서 모든 방향으로 고르게 잘 움직이는 것은 아니다. 한쪽 축으로만 길쭉한 럭비공 모양의 타원체도 부피는 클 수 있기 때문이다.

그래서 방향별 균형까지 보려면 타원체의 모양을 뜯어봐야 한다. 타원체의 주축 방향은 $J J^T$의 고유벡터이고, 각 반축의 길이는 고유값의 제곱근(즉 자코비안의 특이값)에 해당한다(3장 §3.1.2~§3.1.3). 반축이 긴 방향으로는 작은 관절 움직임만으로도 끝단이 빠르게 움직이는 반면, 반축이 짧은 방향으로는 관절을 많이 돌려도 끝단이 굼뜨다. 모든 축의 길이가 비슷해 타원체가 구에 가까운 자세를 **등방(isotropic) 자세**라 하며, 가장 긴 축과 가장 짧은 축의 비율인 condition number가 1에 가까울수록 방향 편차가 적다.

아래 코드는 Puma 560의 한 자세에서 자코비안을 구하고, 조작성 지수와 선속도 타원체의 반축 길이, 그리고 SVD 기반 condition number를 함께 계산한다.

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

실제 로봇 팔을 조이스틱으로 텔레오퍼레이션하거나 카메라 영상을 보며 손끝을 미세 조정하는 시각 서보잉(visual servoing)에서는 매 제어 주기마다 목표 **위치** 대신 목표 **끝단 속도**(`desired_twist`)가 들어온다. 이때 매번 전체 IK를 처음부터 푸는 대신, 현재 자세에서 자코비안에 DLS를 적용해 관절 속도 $\dot{q}$를 바로 뽑아내고 제어 주기 `dt`만큼 적분해 다음 관절 각도를 내보내는 방식을 **분해 속도 제어(resolved motion rate control)**라 한다. 실제 하드웨어에 적용할 때는 특이점 근처나 큰 속도 명령이 들어왔을 때 모터가 안전 한계를 넘지 않도록 최대 관절 속도(`max_qdot`)로 비율을 맞추는 클리핑(scaling)이 반드시 따라붙는다.

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

이렇게 계산한 `q_new`와 토크 $\tau$는 아직 제어 컴퓨터 메모리 속의 부동소수점 숫자에 지나지 않는다. 이 숫자들이 실제 금속 링크를 밀고 당기며 1ms마다 정밀하게 움직이게 하려면 모터·감속기·센서·통신 버스·실시간 운영체제가 한 몸처럼 돌아가는 메카트로닉스의 세계로 내려가야 한다.

> **추천 자료**
> - Siciliano et al., *Robotics: Modelling, Planning and Control*, Chapter 3 — 미분 기구학과 정역학(Differential Kinematics and Statics). 자코비안, 특이점, 조작성 타원체를 다룬다
> - Corke, *Robotics, Vision and Control*, Chapter 8 — 매니퓰레이터 속도와 자코비안을 코드와 그림으로 설명한다: https://petercorke.com/rvc/
> - robotics-toolbox-python 문서: https://github.com/petercorke/robotics-toolbox-python

---

## 4.5 메카트로닉스 기초

기구학이 계산한 관절 각도와 속도 명령이 실제 로봇 팔의 물리적 움직임으로 이어지려면, 제어 컴퓨터에서 나온 명령이 통신 버스를 타고 모터 드라이버로 건너가 모터와 감속기를 돌리고, 다시 엔코더와 토크 센서가 읽은 값이 1ms 안팎의 주기로 제어기에 돌아오는 하드웨어 사슬이 빈틈없이 맞물려야 한다. 이 물리 시스템 전체를 설계하고 통합하는 분야가 메카트로닉스다.


### 4.5.1 액추에이터

로봇의 근육에 해당하는 액추에이터는 구동 방식과 감속기 선택에 따라 힘 제어 성능, 정밀도, 가격, 내구성이 크게 갈린다.

**DC 모터:**
전압을 걸면 돌아가는 가장 단순한 형태다. 토크는 흐르는 전류에 비례하고($\tau = K_t \cdot i$), 회전에 따라 생기는 역기전력은 각속도에 비례한다($V_{\text{emf}} = K_e \cdot \omega$). 구동 회로가 단순하고 값이 싸서 소형 장난감이나 저가 이동 로봇에 알맞지만, 내부 브러시가 물리적으로 맞닿아 닳기 때문에 장시간 고부하 운용에는 수명이 짧다.

**BLDC (Brushless DC) 모터:**
브러시를 없애고 전자식 드라이버 회로가 코일 전류의 방향을 고속으로 바꿔 주는 모터다. 마모 부품이 없어 수명이 길고, 같은 무게 대비 낼 수 있는 토크(토크 밀도)와 에너지 효율이 높아 현대 로봇 관절의 사실상 표준으로 쓰인다. 여기에 드라이버가 회전자 자속 방향에 맞춰 전류를 정교하게 쪼개 넣는 FOC(Field-Oriented Control) 방식을 쓰면, 회전 도중 토크가 울렁거리는 토크 리플(ripple)까지 크게 줄어든다.

**서보 모터 (Dynamixel 시리즈):**
모터 본체와 감속기, 위치 센서(엔코더), 제어 회로를 작은 하우징 하나에 통째로 묶어 둔 모듈형 제품이다. 배선과 드라이버 설계를 따로 하지 않아도 곧바로 로봇 팔을 조립해 볼 수 있어 Robotis의 Dynamixel 시리즈가 연구실과 교육용 플랫폼에서 널리 쓰인다.

| 모델 | 공칭 토크 예시 (Nm) | 통신 | 용도 |
|------|-----------|------|------|
| XL330 | 0.5 | TTL | 소형 그리퍼, 소형 저가 팔 |
| XM540 | 10.0 | RS-485 | 중형 로봇 팔 |
| PH54  | 44.7 (연속 운전) | RS-485 | 대형 매니퓰레이터, 모바일 로봇 |

서보 모터를 고를 때는 카탈로그에 적힌 최대 순간 토크만 보고 결정하면 낭패를 보기 쉽다. 실제 토크는 공급 전압과 방열 조건에 크게 좌우되므로, e-Manual에서 stall 토크를 측정한 조건과 장시간 버틸 수 있는 **연속 운전(continuous) 토크 한계**를 반드시 따로 확인해야 한다. Dynamixel은 여러 관절을 한 가닥 선으로 줄줄이 잇는 데이지 체인(daisy-chain) 배선을 지원하며, 위치·속도·전류 제어 모드 전환과 내부 PID 게인 조정을 펌웨어 수준에서 제공한다. 다만 제품군마다 통신 주기와 발열 허용치가 다르니, 목표 제어 주기와 제어 모드를 기본 펌웨어가 지원하는지부터 점검하는 편이 좋다.

**Quasi-Direct Drive (QDD):**

일반 산업용 로봇 관절은 작은 모터로 무거운 팔을 들어 올리기 위해 감속비가 100:1을 훌쩍 넘는 하모닉 드라이브(harmonic drive)를 끼운다. 하지만 감속비 `N`이 커질수록 기어 뒷단에서 느껴지는 모터 회전자의 관성 모멘트(반사 관성, reflected inertia)는 `N`이 아니라 **`N²`에 비례해 폭증**한다. 감속비 100:1 기어를 물리는 순간 손끝에서 느끼는 모터 관성은 무려 10,000배로 불어나는 셈이다. 이 때문에 외부에서 팔을 밀어도 관절이 뻣뻣하게 버티며 돌아가지 않고(낮은 백드라이버빌리티), 모터 전류로 외력을 재려 해도 거대한 기어 마찰과 반사 관성에 신호가 묻혀 정밀한 힘 제어가 어려워진다.

이 문제를 정면으로 뒤집은 설계가 **QDD(Quasi-Direct Drive)**다. 감속비를 6:1에서 10:1 수준으로 확 낮추고 유성기어(planetary gear)나 벨트를 조합해 기어 마찰과 반사 관성을 극단적으로 줄인다. 그 결과 바깥에서 힘을 주면 관절이 스르르 밀려 돌아가는 높은 **백드라이버빌리티(backdrivability)**를 얻어, 예기치 못한 충돌을 부드럽게 흡수하고 전류 측정만으로도 관절 토크를 꽤 정확히 추정할 수 있다. 탄성과 마찰이 작으니 토크 응답 대역폭도 높다. 다만 감속비의 도움을 적게 받는 만큼 같은 크기의 모터에서 나오는 출력 토크가 작아, 큰 힘이 필요하면 모터를 더 크고 굵게 만들어야 한다는 트레이드오프가 따른다. MIT Mini Cheetah와 Unitree의 사족 보행 로봇이 지면 충격을 흡수하면서도 민첩하게 뛰기 위해 바로 이 QDD 관절을 채택했다.

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

모터의 고속·저토크 출력을 관절에 맞는 저속·고토크로 바꾸는 감속기는 크게 세 계열로 나뉘며, 정밀도와 백드라이버빌리티, 가격 사이에서 명확한 선택지가 갈린다.

| 종류 | 감속비 | 백래시 | 효율 | 가격 | 용도 |
|------|--------|--------|------|------|------|
| Planetary | 3~100:1 | 중간 | 85~95% | 저렴 | 범용, QDD에 적합 |
| Harmonic Drive | 30~320:1 | 매우 낮음 | 65~85% | 비쌈 | 산업용 로봇, 정밀 |
| Cycloidal | 6~120:1 | 낮음 | 85~93% | 중간 | 최근 대안으로 부상 |

같은 계열 안에서도 제조사와 정밀 등급에 따라 감속비, 효율, 백래시(기어 이빨 사이의 유격) 편차가 상당히 크다. 위 표는 계열 간 성격을 가늠하는 참고치일 뿐이므로, 실제 부품을 선정할 때는 반드시 데이터시트에 명시된 정격 부하 조건을 확인해야 한다.

**액추에이터 선정 기준:**

실제 관절용 액추에이터의 용량을 산정할 때는 세 가지 요소를 따져야 한다. 팔을 수평으로 뻗은 최악의 자세에서 중력을 버티는 **정적 토크**, 목표 가속도로 링크를 밀어 올리는 **동적 토크**, 그리고 충돌·하중 오차·마찰·수명 마진을 반영한 **여유 계수(안전 계수)**다. 여기에 관절 최대 각속도에 감속비를 곱해 모터가 감당해야 할 RPM을 구하고, 연속 토크와 발열 한계, 크기와 무게까지 함께 점검한다. 결국 QDD와 하모닉 드라이브 가운데 어느 쪽을 택할지는 토크 밀도, 외력이 모터까지 왜곡 없이 전달되는 정도(투명도), 위치 반복 정밀도, 예산이라는 네 가지 요구 조건의 균형으로 결정된다. 아래 예시는 팔 길이 0.5m 끝에서 1kg 물체를 들어 올리는 관절의 최소 요구 사양을 계산하는 과정을 보여 준다(안전 계수 2는 예시용 가정값이다).

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

아무리 훌륭한 모터와 감속기를 달아도, 현재 관절이 정확히 몇 도에 서 있는지, 끝단에 얼마나 큰 힘이 걸리고 있는지를 제어기가 읽지 못하면 폐루프 제어를 걸 수 없다.

**엔코더 (Encoder):**

엔코더는 회전축의 각도와 각속도를 측정하는 핵심 센서로, 전원을 껐다 켰을 때 위치를 기억하는지 여부에 따라 두 종류로 나뉜다.

*Incremental encoder*는 원판의 눈금을 통과하는 A, B 두 채널의 펄스 수를 세어 출발 시점 대비 얼마나 회전했는지 상대 변화를 잰다. 구조가 단순하고 값이 싸면서 한 바퀴당 10,000펄스(PPR)를 넘는 고분해능 제품도 흔하지만, 전원이 꺼지면 현재 각도 정보를 잃어버리기 때문에 로봇을 켤 때마다 리미트 스위치 등을 이용해 원점을 다시 찾는 **homing(원점 복귀)** 절차가 필요하다. 또한 A, B 두 신호의 상승·하강 에지를 모두 세는 4체배(quadrature decoding)를 쓰면 표시된 PPR의 4배 분해능을 얻을 수 있고, 감속기 앞단(모터 축)에 달면 출력 측 각도 분해능은 감속비만큼 더 촘촘해진다.

*Absolute encoder*는 디스크마다 고유한 디지털 패턴이 새겨져 있어 전원을 켜자마자 현재 각도를 즉시 출력한다. 여러 바퀴 도는 축에서 몇 바퀴째인지까지 함께 기억하는 multi-turn absolute encoder도 있다. 가격은 더 비싸지만 원점 복귀 없이 전원을 넣자마자 이전 자세 그대로 작업을 재개해야 하는 산업용 로봇 팔에 널리 쓰인다.

```
분해능 계산 예시:
  Incremental encoder, 4096 PPR, quadrature decoding (x4)
  → 분해능 = 360° / (4096 × 4) = 0.022° ≈ 0.38 mrad
  → 감속비 100:1 관절 → 출력 분해능 0.0038 mrad
```

**토크 센서:**

고감속비 기어의 마찰 때문에 모터 전류만으로 외력을 정확히 알기 어려울 때는 변형량을 직접 재는 스트레인 게이지(strain gauge) 기반 토크 센서를 붙인다.

*관절 토크 센서(Joint Torque Sensor, JTS)*는 감속기 출력단 바로 뒤에 장착해 기어 마찰을 거친 뒤 실제로 링크에 전달되는 순수 토크를 측정한다. KUKA LBR iiwa가 7개 관절 모두에 JTS를 내장해 정밀한 충돌 감지와 힘 제어를 구현한 대표 사례다.

*힘/토크 센서(F/T Sensor)*는 손목 끝단에 장착해 3축 힘과 3축 모멘트(`Fx, Fy, Fz, Tx, Ty, Tz`) 총 6성분을 동시에 측정한다. 연구실에서는 ATI Industrial Automation 제품이 표준처럼 쓰이며, 도입할 때는 정격 측정 범위와 분해능뿐 아니라 순간 충격을 견디는 과부하 한계, 통신 인터페이스, 비용을 함께 따져야 한다.

**관성 센서 (IMU):**

가속도계와 자이로스코프를 하나로 묶고 필요에 따라 자력계를 더한 IMU(2장 참고)는 공간 속에서 몸체가 얼마나 기울어지고 회전하는지 추정한다. 모바일 로봇이나 보행 로봇이 넘어지지 않고 자세를 잡는 데 필수적이며, 대형 매니퓰레이터에서는 길게 뻗은 링크 끝에 IMU를 붙여 고속 이동 시 발생하는 구조 진동을 억제하는 데 활용하기도 한다.


### 4.5.3 통신 프로토콜

모터와 센서가 준비되면 이제 이를 마이크로컨트롤러나 제어 PC와 선으로 이어야 한다. 제어 루프에서 통신 지연(latency)이 길어지면 위상 지연이 쌓여 제어기가 진동하거나 발산할 위험이 커지고, 반대로 대역폭이 부족하면 센서 데이터가 중간에 누락된다.

**기초 프로토콜:**

보드 내부 칩 사이나 센서 몇 개를 짧은 거리로 연결할 때는 흔히 아래 세 가지 직렬 프로토콜을 쓴다.

| 프로토콜 | 배선 | 속도 | 거리 | 특징 |
|---------|------|------|------|------|
| UART | 2선 (TX, RX) | ~1 Mbps | ~15 m | 가장 단순, 1:1 통신 |
| SPI | 4선 (MOSI, MISO, SCK, CS) | ~50 Mbps | ~1 m (PCB 내) | 빠름, 다수 슬레이브는 CS 추가 |
| I2C | 2선 (SDA, SCL) | 100 kbps~3.4 Mbps | ~1 m | 주소 기반, 센서 연결에 편리 |

하지만 고전류 모터가 돌아가며 전자기 노이즈를 뿜어내는 로봇 관절 배선에서 수 미터 떨어진 모터 드라이버들을 믿고 연결하기에는 UART, SPI, I2C가 너무 취약하다. 이때는 차동 신호나 결정적 실시간 전송을 지원하는 필드버스가 필요하다.

**CAN Bus:**

자동차 내부 전장 장치들을 잇기 위해 개발된 CAN(Controller Area Network)은 두 가닥 선의 전압 차이로 신호를 주고받는 차동 신호(differential signaling) 방식을 써서 모터 드라이버 주변의 심한 전기적 잡음에도 잘 버틴다. 버스 위에 여러 노드가 동시에 말을 꺼내려 할 때는 메시지 ID의 우선순위에 따라 충돌 없이 한 노드만 전송을 이어 가는 중재(arbitration) 메커니즘이 하드웨어 수준에서 동작한다.

- 속도: 최대 1 Mbps (CAN 2.0), 5 Mbps (CAN FD)
- 거리: 50 kbps에서 약 1 km, 125 kbps에서 약 500 m, 1 Mbps에서 약 25 m
- 토폴로지: 버스 (데이지 체인 가능)

로봇에서는 주로 메인 컨트롤러와 관절 모터 드라이버들을 CAN으로 묶으며, MIT Cheetah 계열을 비롯한 수많은 보행 로봇과 액추에이터 모듈이 이 방식을 채택한다. 다만 고전적인 CAN 2.0은 한 프레임의 데이터 크기가 최대 8바이트로 작다. 아래 C++ 예제(STM32 HAL 의사코드)는 MIT Mini Cheetah 모터 드라이버의 패킷 포맷을 따라, 위치·속도·게인·토크 다섯 개의 부동소수점 명령을 정수 비트로 압축(bit-packing)해 단 8바이트 안에 실어 보내는 실전 구현을 보여 준다.

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

노드 수가 늘어나고 제어 주기가 1kHz를 넘어 4~8kHz까지 올라가면 CAN의 1~5 Mbps 대역폭과 중재 지연이 빠듯해진다. Beckhoff가 개발한 **EtherCAT**은 일반 이더넷 물리 계층을 쓰면서도 매번 지연 시간이 똑같이 유지되는(deterministic) 산업용 실시간 프로토콜이다. 각 모터마다 별도 패킷을 보내고 응답을 기다리는 대신, 마스터가 보낸 하나의 이더넷 프레임이 슬레이브 노드들을 기차처럼 스쳐 지나가면서 각 노드가 자기 칸에 해당하는 명령을 꺼내 읽고 센서 피드백을 그 자리에서 실어 보낸다(processing on the fly). 덕분에 100 Mbps 회선 하나로 수백 개 노드를 묶어도 대역폭 손실이 거의 없고, 분산 시계 동기화로 노드 간 시간 오차를 1 µs 이내로 맞출 수 있다.

KUKA 컨트롤러나 Beckhoff 기반 산업 설비가 EtherCAT을 표준으로 쓴다. 다만 PC를 마스터로 돌리려면 SOEM이나 IgH EtherCAT Master 같은 전용 스택을 설치하고 NIC 드라이버와 실시간 커널 설정을 맞추어야 해서, 가벼운 취미·프로토타입 로봇에는 도입 부담이 크다.

**RS-485 / Dynamixel Protocol:**

Dynamixel 같은 통합 서보 계열은 RS-485나 TTL 시리얼 통신을 쓴다. RS-485는 CAN처럼 두 선의 차동 신호를 써서 배선 잡음에 강하고 짧은 거리에서는 수 Mbps까지 속도를 올릴 수 있으며, 여러 서보를 한 가닥에 데이지 체인으로 엮기 쉽다. 아래 Python 코드는 Dynamixel SDK를 이용해 1 Mbps 속도로 서보의 토크를 켜고 목표 각도를 보낸 뒤 현재 위치를 읽어 들이는 기본 흐름이다.

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

로봇 제어에서 **실시간(real-time)**이란 단순히 계산이 평균적으로 빠르다는 뜻이 아니라, **최악의 상황에서도 정해진 마감 시간(deadline) 안에 반드시 처리가 끝난다고 보장함**을 의미한다. 1kHz 제어 루프라면 1ms마다 센서 읽기, 제어량 계산, 명령 송신이 어김없이 완료되어야 한다. 한 번이라도 마감을 놓쳐 제어 주기가 출렁이면(jitter) 미분 제어항이 튀거나 로봇이 떨리며 불안정해질 수 있다.

**RTOS (Real-Time Operating System):**

| RTOS | 특징 | 용도 |
|------|------|------|
| FreeRTOS | 경량, 마이크로컨트롤러용, 무료 | STM32, ESP32 등 |
| Zephyr | 최신, 다양한 하드웨어 지원, Linux Foundation | IoT, 로봇 임베디드 |
| VxWorks | 상용, NASA도 사용 | 항공우주, 산업용 |

모터 드라이버 내부나 펌웨어처럼 마이크로컨트롤러 위에서 모터를 직접 제어할 때는 RTOS를 올려 제어 루프 태스크에 최고 우선순위를 부여함으로써 로깅이나 통신 같은 부가 작업에 절대 선점당하지 않도록 막는다.

**PREEMPT_RT Linux:**

반면 상위 제어기나 ROS2 노드는 대부분 PC급의 Linux 위에서 돌아간다. 일반 Linux 커널은 처리량(throughput)을 높이는 데 최적화되어 있어, 디스크 I/O나 커널 락 때문에 제어 스레드가 갑자기 수 밀리초씩 멈추는 일이 잦다. **PREEMPT_RT** 패치는 커널 내부 코드의 대부분을 필요할 때 다른 고우선순위 스레드가 즉시 끼어들어 가로챌 수 있도록(preemptible) 바꾸어 최대 지연 시간을 수십 마이크로초 수준으로 끌어내린다. 오랫동안 별도 패치로 관리되던 PREEMPT_RT는 Linux 6.12부터 마침내 메인라인 커널에 공식 합류했다.

아래는 Debian 계열에서 RT 커널을 설치하고 CPU 코어를 격리해 제어 프로세스의 지연을 검증하는 전형적인 설정 절차다.

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

토크 제어와 임피던스 제어는 경험상 **1kHz(1ms)**를 표준 출발점으로 흔히 잡는다. 필요한 제어 주기는 목표 폐루프 대역폭, 기구부의 기계적 공진 주파수, 센서·액추에이터 지연, 제어 연산 시간, 주기 흔들림(jitter) 여유를 종합해 결정한다. 신호처리에서 흔히 인용하는 Nyquist 기준의 2배는 에일리어싱(aliasing)을 피하기 위한 이론적 최소치일 뿐, 제어에서는 위상 지연을 줄이기 위해 목표 폐루프 대역폭보다 훨씬 빠르게 샘플링한 뒤 주파수 응답과 지연 여유(phase margin)를 확인해야 한다. 특히 CAN 버스를 쓴다면 관절 모터 수, 프레임 크기, 버스 중재 오버헤드, 버스 점유율, 피드백 주기를 모두 더해 물리 회선 대역폭 안에 들어오는지부터 계산해야 한다.

충돌 감지 반응이 극도로 빨라야 하는 경량 팔이나 고속 촉각 제어는 수 kHz 단위로 돌리기도 한다. 이처럼 요구되는 결정성과 대역폭, 입출력 배선 구조에 따라 PC + EtherCAT 조합을 쓸지, MCU + CAN이나 FPGA 기반 저지연 루프를 둘지를 선택하게 된다.

> **추천 자료**
> - FreeRTOS 공식 문서: https://www.freertos.org/
> - PREEMPT_RT Wiki: https://wiki.linuxfoundation.org/realtime/start
> - Dynamixel SDK: https://github.com/ROBOTIS-GIT/DynamixelSDK
> - IgH EtherCAT Master (Linux용 오픈소스): https://etherlab.org/en/ethercat/
> - SOEM (Simple Open EtherCAT Master): https://github.com/OpenEtherCATsociety/SOEM

---

## 4.6 Workspace Analysis와 최적 설계 [심화]

지금까지는 이미 만들어진 로봇 팔을 어떻게 움직이고 제어할지를 다뤘다. 반대로 로봇을 처음부터 **설계**하는 자리에서는 기구학을 거꾸로 거울삼아 "링크 길이를 얼마로 잡고 관절을 어떻게 배치해야 원하는 작업 공간 구석구석까지 막힘없이 닿을까?"를 먼저 따져야 한다.


### 4.6.1 Reachable Workspace vs Dexterous Workspace

로봇 팔이 움직일 수 있는 공간은 요구하는 자유도에 따라 두 층위로 나뉜다.

**Reachable workspace**는 손끝이 *적어도 한 가지 이상의 자세*로 닿을 수 있는 공간상 점들의 집합이다. 반면 **Dexterous workspace**는 손끝이 그 점에 닿는 데서 그치지 않고 *원하는 모든 방향(자세)*으로 손목을 비틀어 접근할 수 있는 점들의 집합을 뜻한다. 가령 팔을 끝까지 쭉 뻗어야 겨우 닿는 선반 끝은 reachable workspace에는 들어가지만 손목 각도를 바꿀 수 없어 dexterous workspace에서는 빠진다. 따라서 dexterous workspace는 늘 reachable workspace의 부분집합이며 실제 크기는 훨씬 작다.

다관절 로봇에는 각 관절마다 물리적 회전 한계(joint limit)가 걸려 있어 작업 공간의 경계를 종이 위에서 해석적으로 풀기가 꽤 번거롭다. 실무에서는 관절 범위 안에서 각도를 수만 번 무작위로 뽑아 순기구학(FK)을 통과시킨 뒤 끝단 좌표를 점구름(point cloud)으로 찍어 보는 **Monte Carlo 방식**으로 작업 영역을 시각화한다.

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

팔이 닿는 영역을 그렸다고 설계가 끝나지 않는다. 그 영역 안에서도 특정 자세에서 손끝이 한쪽 방향으로는 잘 움직이는데 다른 방향으로는 꼼짝도 못 한다면 작업 품질이 떨어진다. §4.4.3에서 본 조작성 타원체가 모든 방향으로 얼마나 둥글고 고르게 뻗어 있는지를 수치로 재는 지표가 자코비안의 **condition number($\kappa$)**다.

```
κ(J) = σ_max / σ_min
```

여기서 $\sigma_{\max}$와 $\sigma_{\min}$은 자코비안의 가장 큰 특이값과 가장 작은 특이값(singular value)이다. $\kappa = 1$이면 모든 방향의 움직임 민감도가 똑같은 이상적인 **등방(isotropic) 자세**가 되고, 반대로 특이점에 가까워질수록 $\sigma_{\min} \to 0$이 되면서 $\kappa$가 무한대로 치솟는다. 로봇을 설계할 때는 주 작업 영역 전체에서 이 condition number가 가능한 한 작아지도록 링크 길이 비율과 관절 오프셋을 최적화하는데, 이를 **kinematic optimization** 또는 최적 기구 설계라 부른다.

한 가지 실무적 주의점이 있다. 6차원 자코비안에는 선속도(단위: m/s) 행과 각속도(단위: rad/s) 행이 한 행렬 안에 위아래로 섞여 있다. 단위 차원이 다른 두 물리량을 그대로 한 행렬에 넣고 SVD를 돌리면 링크 길이 단위(m vs mm)에 따라 $\kappa$ 값이 들쑥날쑥해지는 모순이 생긴다. 이를 피하려면 로봇의 대표 치수인 특성 길이(characteristic length)로 각속도 행의 단위를 길이에 맞춰 정규화하거나, 선속도 블록과 각속도 블록을 떼어내어 따로 분석해야 한다.


### 4.6.3 Redundancy Resolution (7-DOF Arms)

6차원 공간에서 손끝의 위치(3)와 자세(3)를 모두 맞추는 데 필요한 최소 자유도는 6이다. 그런데 Kinova Gen3, KUKA LBR iiwa, Franka Emika Panda처럼 연구와 협동 작업에 많이 쓰이는 로봇 팔은 관절이 하나 더 많은 **7-DOF** 구조를 택한다. 이 남는 1개의 자유도를 **기구학적 여유 자유도(kinematic redundancy)**라 한다.

여유 자유도가 있으면 손끝을 공간의 한 점에 고정한 채로 팔 전체의 형상을 바꿀 수 있다. 실제로 손바닥을 책상 한 지점에 꾹 눌러 고정하고 팔꿈치만 위아래로 움직여 보면, 손끝의 위치와 각도는 변하지 않는데 어깨와 팔꿈치 각도가 매끄럽게 변하는 모습을 바로 확인할 수 있다. 로봇에서는 이 남는 자유도를 활용해 조작성(manipulability)을 키워 특이점을 피하거나, 관절 한계에 몰린 축을 중앙으로 되돌리고, 팔꿈치를 장애물에서 비키게 하며, 중력 토크가 덜 드는 자세를 선택한다.

수학적으로 이런 "끝단 자세를 건드리지 않는 관절 움직임"은 자코비안의 영공간(**null space**) 안에서 일어난다.

```
q̇ = J† * ẋ + (I - J† * J) * q̇_0
```

첫째 항 `J† * ẋ`는 원하는 끝단 속도 `ẋ`를 만들어 내는 무수히 많은 관절 속도 해 가운데 제곱 크기(norm)가 가장 작은 최소 노름 해다. 둘째 항의 `(I - J† * J)`가 바로 **null space projector**로, 어떤 관절 속도 벡터 `q̇_0`를 집어넣든 끝단 속도에 영향을 주는 성분은 모두 지워 버리고 끝단을 가만히 둔 채 내부 자세만 바꾸는 성분만 걸러 남긴다. 따라서 `q̇_0` 자리에 관절 중앙 복귀나 조작성 최대화 같은 2차 목표의 그래디언트를 넣어 주면, 주 임무(끝단 추종)를 전혀 방해하지 않으면서 팔의 내부 자세를 실시간으로 최적화할 수 있다.

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

## 4.7 확률적 운동 모델 [심화]

### 4.7.1 도입: 결정론에서 확률로

§4.2와 §4.3에서 다룬 순기구학과 역기구학은 완벽하게 결정론적(deterministic)이다. 관절 각도를 입력하면 손끝 위치가 딱 한 점으로 나오고, 손끝 위치를 주면 정해진 개수의 관절 해가 떨어진다. 물론 실제 매니퓰레이터에서도 기어 백래시, 링크의 미세한 휨(compliance), 조립 오차 때문에 모델과 실제 위치가 조금씩 어긋나기는 한다.

하지만 바퀴를 굴려 바닥을 달리는 **모바일 로봇**으로 넘어오면 이 오차가 아예 모델링의 본질이 된다. 로봇에게 "초속 1m로 똑바로 가라"고 명령해도 바퀴가 바닥에서 미끄러지고, 타이어 마모로 실제 반지름이 줄며, 좌우 바퀴 지름이 미세하게 다르면 로봇은 직진 대신 한쪽으로 슬슬 휘어 간다. 따라서 현실의 모바일 로봇에서는 이전 상태 $x_{t-1}$에서 제어 입력 $u_t$를 주었을 때 도착하는 다음 포즈 $x_t$를 하나의 고정된 점이 아니라 **확률 분포**로 표현해야 한다. 이것이 3장 §3.9~§3.11의 상태 추정 필터들이 예측 단계에서 사용하는 **확률적 운동 모델(probabilistic motion model)**이다.

평면 위를 움직이는 로봇의 상태는 포즈 $x_t = (x, y, \theta)^T \in SE(2)$로 둔다. 확률적 운동 모델은 직전 포즈 $x_{t-1}$과 제어 입력 $u_t$가 주어졌을 때 다음 포즈가 놓일 조건부 확률 분포

$$p(x_t \mid u_t, x_{t-1})$$

를 정의한다. 이때 제어 입력 $u_t$를 어떤 물리량으로 잡느냐에 따라 두 가지 대표 모델로 갈린다.

**Velocity model**은 로봇에 내린 선속도와 각속도 명령 $u_t = (v, \omega)^T$를 그대로 입력으로 삼아, 명령 속도와 실제 속도 사이의 오차를 노이즈로 모델링한다. 실제로 바퀴를 굴리기 전에도 명령만으로 다음 분포를 계산할 수 있으므로 경로 계획(planning) 단계에서도 쓸 수 있다.

**Odometry model**은 바퀴 엔코더 회전량을 기구학적으로 적분해 얻은 두 시점의 상대 포즈 $u_t = (\bar{x}_{t-1}, \bar{x}_t)$를 입력으로 쓴다. 로봇이 이미 한 걸음 움직인 뒤에야 읽을 수 있는 사후 값이므로 계획에는 못 쓰지만, 제어 명령과 실제 움직임 사이의 모터 지연이나 미끄러짐 일부가 이미 엔코더에 반영되어 있어 Thrun 등의 *Probabilistic Robotics*도 경험상 대개 velocity model보다 더 정확하다고 평가한다.

각 모델을 실제로 구현해 쓰는 방식 역시 두 갈래로 나뉜다. **폐쇄형 밀도 평가(closed-form density evaluation)**는 후보 포즈 $x_t$가 주어졌을 때 그 지점의 확률밀도 값을 직접 계산해 주는 함수로, 베이즈 필터의 적분을 공간 격자 위에서 직접 계산하는 격자 국지화(grid localization)에 쓰인다. 반면 **샘플링(sampling)**은 밀도 식을 일일이 평가하지 않고 확률 분포를 따르는 다음 포즈 샘플 하나를 무작위로 뽑는 순방향 시뮬레이션으로, 수많은 입자를 굴리는 파티클 필터(MCL)가 이 방식을 쓴다(참고로 EKF와 UKF는 이 둘을 쓰지 않고 자코비안 선형화나 시그마점으로 평균과 공분산을 직접 전파한다).


### 4.7.2 Velocity Motion Model — 폐쇄형

노이즈가 전혀 없는 이상적인 세상이라면, 일정한 선속도 $v$와 각속도 $\omega$를 유지하며 달리는 로봇은 평면 위에서 매끈한 원호(circular arc)를 그린다($\omega = 0$일 때는 직선). 폐쇄형 밀도 평가는 이 기하학을 거꾸로 되감아 계산한다. 직전 포즈 $x_{t-1}$과 가설 포즈 $x_t$가 주어졌을 때, 먼저 두 포즈의 위치와 출발 방향을 매끄럽게 잇는 유일한 원호의 회전 중심 $(x_c, y_c)$와 반지름 $r^*$를 역산한다(PR 표와 아래 의사 코드는 중심을 $(x^*, y^*)$로 적는다). 그다음 그 원호를 $\Delta t$ 동안 돌아 도착하려면 실제로 어떤 속도 $(\hat{v}, \hat{\omega})$가 필요했을지를 구하고, 이 역산 속도가 명령 속도 $(v, \omega)$와 얼마나 어긋나는지를 노이즈 분포로 채점하는 방식이다.

**수식.** 두 포즈 $x_{t-1} = (x, y, \theta)^T$와 가설 $x_t = (x', y', \theta')^T$가 주어졌을 때:

$$\mu = \frac{1}{2} \cdot \frac{(x - x')\cos\theta + (y - y')\sin\theta}{(y - y')\cos\theta - (x - x')\sin\theta}$$

$$x_c = \frac{x + x'}{2} + \mu(y - y'), \quad y_c = \frac{y + y'}{2} + \mu(x' - x)$$

$$r^* = \sqrt{(x - x_c)^2 + (y - y_c)^2}$$

$$\Delta\theta = \text{atan2}(y' - y_c,\ x' - x_c) - \text{atan2}(y - y_c,\ x - x_c)$$

$$\hat{v} = \frac{\Delta\theta \cdot r^*}{\Delta t}, \quad \hat{\omega} = \frac{\Delta\theta}{\Delta t}, \quad \hat{\gamma} = \frac{\theta' - \theta}{\Delta t} - \hat{\omega}$$

현실에서는 빠르게 달리거나 급하게 회전할수록 바퀴 미끄러짐과 오차가 커지므로, 노이즈의 분산 역시 명령 크기에 비례하도록 잡는다. `prob(a, b)`의 두 번째 인자인 분산 $b$는 다음과 같이 정해진다.

$$b_v = \alpha_1|v| + \alpha_2|\omega|, \quad b_\omega = \alpha_3|v| + \alpha_4|\omega|, \quad b_\gamma = \alpha_5|v| + \alpha_6|\omega|$$

여기서 $b_v$는 선속도, $b_\omega$는 각속도, $b_\gamma$는 도착 순간 최종 회전의 노이즈 분산이다. 언뜻 낯설어 보이는 $\hat{\gamma}$ 항이 왜 꼭 들어가야 하는지 짚고 넘어가자. 원호는 출발 방향 $\theta$와 도착 위치 $(x', y')$가 정해지면 도착 방향까지 하나로 고정된다. 만약 노이즈 변수가 $(v, \omega)$ 두 개뿐이라면, 가설 포즈는 3차원 $(x, y, \theta)$ 공간 안에 놓인 2차원 곡면 위에만 존재하게 되어 그 곡면을 1도라도 벗어난 포즈에는 전부 확률 0이 매겨지는 퇴화(degeneracy) 문제가 생긴다. 따라서 도착 직후 제자리에서 방향을 살짝 비트는 $\hat{\gamma}$ 노이즈를 추가해야만 3차원 포즈 공간 전체로 확률이 매끄럽게 퍼진다.

여섯 개의 노이즈 계수 가운데 $\alpha_1, \alpha_2$는 선속도 노이즈, $\alpha_3, \alpha_4$는 각속도 노이즈, $\alpha_5, \alpha_6$는 최종 회전 노이즈의 분산 가중치다. 이 계수는 바닥 재질과 바퀴 마모 상태, 로봇 기구마다 달라지므로 실제 로봇을 직선, 원, 8자 경로로 주행시키며 얻은 데이터로 보정한다. 아래는 *Probabilistic Robotics*(이하 PR) Table 5.1의 `motion_model_velocity` 알고리즘을 정리한 의사 코드다.

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

세 확률의 곱을 계산할 때 부르는 `prob(a, b)`는 평균 0, 분산 $b$인 정규 분포나 삼각 분포에서 편차 $a$에 대한 확률밀도를 반환하는 함수다(PR Table 5.2).

### 4.7.3 Velocity Motion Model — 샘플링

파티클 필터처럼 수많은 가설 입자를 굴려야 할 때는 가설 포즈마다 회전 중심을 역산할 필요가 없다. 명령 속도 $(v, \omega)$에 먼저 무작위 노이즈를 섞어 흔들린 속도 $(\hat{v}, \hat{\omega}, \hat{\gamma})$를 뽑은 뒤, 그 속도로 $\Delta t$ 동안 원호를 따라가 새 포즈를 생성하면 그만이다. 역산 과정이 사라진 덕분에 폐쇄형보다 수식도, 코드도 훨씬 간결하다.

먼저 노이즈 파라미터 $\alpha_1 \sim \alpha_6$를 이용해 흔들린 제어 입력을 뽑는다.

$$\hat{v} = v + \text{sample}(\alpha_1|v| + \alpha_2|\omega|)$$
$$\hat{\omega} = \omega + \text{sample}(\alpha_3|v| + \alpha_4|\omega|)$$
$$\hat{\gamma} = \text{sample}(\alpha_5|v| + \alpha_6|\omega|)$$

이 속도로 $\Delta t$ 동안 원호를 따라 이동하면 다음 포즈가 계산된다.

$$x' = x - \frac{\hat{v}}{\hat{\omega}}\sin\theta + \frac{\hat{v}}{\hat{\omega}}\sin(\theta + \hat{\omega}\Delta t)$$
$$y' = y + \frac{\hat{v}}{\hat{\omega}}\cos\theta - \frac{\hat{v}}{\hat{\omega}}\cos(\theta + \hat{\omega}\Delta t)$$
$$\theta' = \theta + \hat{\omega}\Delta t + \hat{\gamma}\Delta t$$

구현할 때 한 가지 꼭 챙겨야 할 예외가 있다. 직진 명령처럼 $|\hat{\omega}|$가 0에 가까워지면 분모의 $\hat{\omega}$ 때문에 식이 발산하므로, $|\hat{\omega}| < \varepsilon$인 구간에서는 원호 대신 직선 이동 $x' = x + \hat{v}\cos\theta\,\Delta t,\ y' = y + \hat{v}\sin\theta\,\Delta t$로 분기해 계산해야 한다(PR Table 5.3에는 이 안전 분기가 생략되어 있다).

또한 `sample(b)`는 평균 0, 분산 $b$인 분포에서 표본을 하나 뽑는 함수다. 고전 문헌에서는 정규 분포를 균등 난수 12개의 합 $\frac{\sqrt{b}}{2}\sum_{i=1}^{12}\text{rand}(-1,1)$로 근사해 자주 쓰는데, `rand(-1, 1)` 하나가 분산 $1/3$을 가지므로 12개를 더하면 분산이 4가 되고, 여기에 $\sqrt{b}/2$를 곱해 원하는 분산 $b$를 맞출 수 있기 때문이다. 아래는 직선 분기를 보강한 `sample_motion_model_velocity` 의사 코드와 Python 구현이다.

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

실제 SLAM이나 국지화에서는 속도 명령 대신 바퀴 엔코더를 적분해 만든 오도메트리 포즈 쌍 $u_t = (\bar{x}_{t-1}, \bar{x}_t)$를 훨씬 자주 쓴다. **Odometry model**은 두 포즈 사이의 상대 운동을 직관적인 세 동작—목적지 방향을 바라보도록 먼저 제자리 회전하고($\delta_{\text{rot1}}$), 일직선으로 주행한 뒤($\delta_{\text{trans}}$), 도착해서 목표 방향을 맞추기 위해 한 번 더 회전한다($\delta_{\text{rot2}}$)—으로 분해한다. 평면 $SE(2)$ 위의 어떤 상대 이동이든 이 세 값 $(\delta_{\text{rot1}}, \delta_{\text{trans}}, \delta_{\text{rot2}})$으로 빠짐없이 표현할 수 있다.

한 가지 짚고 갈 모델링 트릭이 있다. 오도메트리는 엄밀히 말해 제어 입력이 아니라 바퀴 엔코더라는 **센서 측정값**이다. 하지만 이를 센서 측정으로 취급하면 실제 속도를 상태 변수에 추가해야 해서 상태 공간의 차원이 커지고 필터가 무거워진다. 그래서 *Probabilistic Robotics*(§5.4.1)는 오도메트리 쌍을 마치 제어 입력 $u_t$인 것처럼 다루어 상태를 포즈 3차원 그대로 유지하는 실용적 근사를 쓴다.

오도메트리 포즈 쌍 $u_t = (\bar{x}_{t-1}, \bar{x}_t)$에서 $\bar{x}_{t-1} = (\bar{x}, \bar{y}, \bar{\theta})^T$, $\bar{x}_t = (\bar{x}', \bar{y}', \bar{\theta}')^T$가 주어지면, 회전–직진–회전 세 성분은 삼각함수로 바로 추출된다.

$$\delta_{\text{rot1}} = \text{atan2}(\bar{y}' - \bar{y},\ \bar{x}' - \bar{x}) - \bar{\theta}$$
$$\delta_{\text{trans}} = \sqrt{(\bar{x} - \bar{x}')^2 + (\bar{y} - \bar{y}')^2}$$
$$\delta_{\text{rot2}} = \bar{\theta}' - \bar{\theta} - \delta_{\text{rot1}}$$

폐쇄형 밀도 평가에서는 가설 포즈 쌍 $(x_{t-1}, x_t)$에서도 똑같은 분해 $(\hat\delta_{\text{rot1}}, \hat\delta_{\text{trans}}, \hat\delta_{\text{rot2}})$를 계산한 뒤, 오도메트리가 보고한 값과의 차이가 각 성분의 노이즈 분포 안에 얼마나 그럴듯하게 들어오는지를 평가한다. 이때 노이즈 파라미터는 $\alpha_1$부터 $\alpha_4$까지 네 개가 쓰인다.

$$b_{\text{rot1}} = \alpha_1|\hat\delta_{\text{rot1}}| + \alpha_2|\hat\delta_{\text{trans}}|$$
$$b_{\text{trans}} = \alpha_3|\hat\delta_{\text{trans}}| + \alpha_4(|\hat\delta_{\text{rot1}}| + |\hat\delta_{\text{rot2}}|)$$
$$b_{\text{rot2}} = \alpha_1|\hat\delta_{\text{rot2}}| + \alpha_2|\hat\delta_{\text{trans}}|$$

네 계수의 물리적 의미는 명확하다. $\alpha_1$은 회전 자체가 만드는 회전 오차, $\alpha_2$는 직진 주행 중 좌우 바퀴 편차 등으로 생기는 회전 오차(드리프트), $\alpha_3$은 주행 거리에 비례하는 직진 오차, $\alpha_4$는 회전 동작이 직진 거리 추정에 섞어 넣는 오차를 나타낸다. 독립 노이즈 변수가 이미 세 개($\delta_{\text{rot1}}, \delta_{\text{trans}}, \delta_{\text{rot2}}$) 확보되어 있으므로, velocity model처럼 $\hat{\gamma}$ 항을 억지로 덧붙이지 않아도 3차원 포즈 공간 전체로 확률이 퍼진다.

구현 시 반드시 기억해야 할 버그 방지 규칙이 하나 있다. $\delta_{\text{rot1}} - \hat{\delta}_{\text{rot1}}$처럼 각도의 차이를 구할 때는 결과를 반드시 $[-\pi, \pi]$ 범위로 접어 넣어야(angle wrapping) 한다. $-\pi$와 $+\pi$ 경계 근처에서 $2\pi$ 차이가 그대로 남아 버리면 확률이 0으로 떨어져 필터가 엉뚱하게 발산하는데, PR에서도 개발자가 가장 흔히 저지르는 실수로 꼽는다.

```
Algorithm motion_model_odometry(x_t, u_t, x_{t-1}):
  # 입력: x_t=(x',y',θ'), u_t=(x̄_{t-1}, x̄_t), x_{t-1}=(x,y,θ)
  # 출력: p(x_t | u_t, x_{t-1}) 확률밀도

  # odometry 측정에서 (δ_rot1, δ_trans, δ_rot2) 추출
  δ_rot1  = atan2(ȳ' − ȳ, x̄' − x̄) − θ̄
  δ_trans = sqrt((x̄ − x̄')² + (ȳ − ȳ')²)
  δ_rot2  = θ̄' − θ̄ − δ_rot1

  # 가설 포즈 쌍에서 같은 분해 (역모델)
  δ̂_rot1  = atan2(y' − y, x' − x) − θ
  δ̂_trans = sqrt((x − x')² + (y − y')²)
  δ̂_rot2  = θ' − θ − δ̂_rot1

  # 세 파라미터 차이를 독립 노이즈로 평가
  p1 = prob(δ_rot1  − δ̂_rot1,  α₁|δ̂_rot1|  + α₂|δ̂_trans|)
  p2 = prob(δ_trans − δ̂_trans, α₃|δ̂_trans| + α₄(|δ̂_rot1| + |δ̂_rot2|))
  p3 = prob(δ_rot2  − δ̂_rot2,  α₁|δ̂_rot2|  + α₂|δ̂_trans|)

  return p1 · p2 · p3
```

(PR Table 5.5의 `motion_model_odometry` 구현)

### 4.7.5 Odometry Motion Model — 샘플링

오도메트리 모델 역시 파티클 필터에 쓰려면 샘플링 형태가 훨씬 간결하다. 오도메트리 쌍에서 추출한 $(\delta_{\text{rot1}}, \delta_{\text{trans}}, \delta_{\text{rot2}})$ 각각에 노이즈를 뺀(또는 더한) 값 $(\hat{\delta}_{\text{rot1}}, \hat{\delta}_{\text{trans}}, \hat{\delta}_{\text{rot2}})$를 뽑은 뒤(PR 식 (5.37)~(5.39)), 식 (5.40)에 따라 회전–직진–회전을 순방향으로 합성하면 새 포즈 샘플이 만들어진다.

$$\begin{pmatrix}x'\\y'\\\theta'\end{pmatrix} = \begin{pmatrix}x\\y\\\theta\end{pmatrix} + \begin{pmatrix}\hat{\delta}_{\text{trans}}\cos(\theta + \hat{\delta}_{\text{rot1}})\\\hat{\delta}_{\text{trans}}\sin(\theta + \hat{\delta}_{\text{rot1}})\\\hat{\delta}_{\text{rot1}} + \hat{\delta}_{\text{rot2}}\end{pmatrix}$$

이 방식은 매끈한 원호 대신 '제자리 회전 → 직진 → 제자리 회전'의 세 직선 구간으로 움직임을 근사한다. 시간 간격 $\Delta t$가 충분히 짧으면 원호와 사실상 구별되지 않을 만큼 정확할 뿐 아니라, 분모에 각속도 $\hat{\omega}$가 들어가지 않으므로 velocity 샘플링처럼 각속도가 0에 가까울 때를 위한 예외 분기를 따로 둘 필요도 없다. PR Table 5.6의 `sample_motion_model_odometry` 알고리즘과 Python 구현은 다음과 같으며, 실제로 ROS2 Nav2의 `nav2_amcl` 패키지가 `differential` motion model이라는 이름으로 이 알고리즘을 그대로 사용한다.

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
  δ̂_trans = δ_trans − sample(α₃|δ_trans| + α₄(|δ̂_rot1| + |δ̂_rot2|))
  δ̂_rot2  = δ_rot2  − sample(α₁|δ_rot2|  + α₂|δ_trans|)

  # forward 합성
  x' = x + δ̂_trans · cos(θ + δ̂_rot1)
  y' = y + δ̂_trans · sin(θ + δ̂_rot1)
  θ' = θ + δ̂_rot1 + δ̂_rot2

  return (x', y', θ')ᵀ
```

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

지금까지 살펴본 네 가지 알고리즘은 모두 벽이나 장애물이 없는 텅 빈 평면을 가정했다. 하지만 실제 건물 안에서 자기 위치를 추정(localization)할 때는 이미 가지고 있는 **지도 $m$**이라는 강력한 힌트가 있다. 아무리 바퀴가 미끄러졌더라도 로봇이 콘크리트 벽을 뚫고 옆방으로 순간이동하거나 기둥 한가운데에 서 있을 수는 없기 때문이다.

지도 정보까지 엄밀하게 조건으로 넣은 정확한 전이 분포를 구하기는 계산상 어렵다. 대신 PR은 §3.3에서 다룬 베이즈 정리와 조건부 독립 가정을 바탕으로, 운동 모델과 지도 사전 확률을 따로 계산해 곱하는 실용적 근사를 제안한다(단, 이 곱 분해는 운동 모델과 지도 prior가 서로 독립이라고 가정할 때만 정확하다).

$$p(x_t \mid u_t, x_{t-1}, m) \propto p(x_t \mid u_t, x_{t-1}) \cdot p(x_t \mid m)$$

앞 항 $p(x_t \mid u_t, x_{t-1})$은 §4.7.2~§4.7.5에서 구한 순수 운동 모델이고, 뒷 항 $p(x_t \mid m)$은 지도 위에서 포즈 $x_t$가 물리적으로 가능한 자리인지를 나타낸다. 점유 격자 지도(occupancy grid)라면 $x_t$가 빈 공간(free cell)일 때 1에 가까운 값을, 벽이나 장애물로 점유된 칸일 때 0에 가까운 값을 준다.

이 근사를 적용하면 파티클 필터에서 입자들이 벽 안쪽으로 새어 나가는 현상을 손쉽게 막을 수 있다. 가장 단순한 구현은 일단 운동 모델로 새 포즈를 샘플링한 뒤 지도에서 그 좌표를 조회해 점유된 칸에 떨어졌다면 해당 입자의 가중치(weight)를 0이나 극히 작은 값으로 깎아 내리는 방식이다. 다만 끝점 $x_t$의 좌표만 확인하는 방식이므로, 제어 주기 $\Delta t$가 너무 길면 얇은 벽을 건너뛰어 반대편 빈 칸에 도착한 입자까지 살아남는 허점이 생길 수 있다.

또한 실제 점유 격자 지도에는 빈 칸과 점유된 칸 외에도 아직 센서가 훑어보지 못한 미탐색 영역(**unknown 칸**)이 존재한다. unknown 칸에 놓인 포즈의 $p(x_t \mid m)$을 빈 칸처럼 1로 볼지, 아니면 불확실성을 반영해 중간값으로 둘지에 따라 국지화 결과가 미묘하게 달라진다. 참고로 ROS2 Nav2의 costmap은 기본 설정(`track_unknown_space: false`)에서 unknown 영역을 통과 가능한 빈 공간으로 취급하는 반면, AMCL은 unknown 칸을 별도 상태로 구분해 처리한다.

### 4.7.7 지금 쓰이는 곳

바퀴로 달리는 모바일 로봇의 파티클 필터 국지화는 지금도 여기서 다룬 샘플링 모델을 핵심 motion prior로 사용한다. 앞서 언급한 ROS2 Nav2 `nav2_amcl`의 `differential` 모델이 바로 §4.7.5의 오도메트리 샘플링이며, 실제 현장에서는 지도 크기, 센서 주기, CPU 여력, 라이다 빔 개수, 주행 오차 특성을 직접 측정해 입자 수와 갱신 주기를 튜닝한다. 구체적으로 §14.7.6의 Monte Carlo Localization(MCL)이 예측(prediction) 단계마다 §4.7.3·§4.7.5의 샘플링 모델(`sample_motion_model`)을 호출하고, §3.10·§3.11의 가우시안 필터와 비모수 필터 역시 $p(x_t \mid u_t, x_{t-1})$를 예측 항으로 삼는다.

다만 여기서 유도한 식들은 모두 평면 $SE(2)$ 위를 굴러가는 차동 구동 바퀴 로봇을 전제로 한다. 네 발로 걷는 보행 로봇이나 휴머노이드, 하늘을 나는 드론에는 차체 속도 $(v, \omega)$만으로 움직임을 기술하기 어렵다. 다리가 땅에 닿았다 떨어지는 보행 로봇은 미끄러짐 양상이 바퀴와 전혀 다르고, 드론은 3차원 공간 $SE(3)$에서 움직이기 때문이다. 이런 플랫폼에서는 대신 IMU 측정을 적분하는 IMU preintegration으로 motion prior를 구성한다(§14.10). $p(x_t \mid u_t, x_{t-1})$라는 확률적 뼈대는 같지만 그 속을 채우는 물리 모델이 달라지는 셈이며, 옴니·메카넘(Mecanum) 휠을 단 전방향(holonomic) 로봇이나 옆으로 미끄러지는(drift) 고속 차량 역시 각 기구에 맞는 별도의 운동 모델을 세워야 한다.

기구학과 메카트로닉스는 관절 각도와 끝단 속도, 구동기 제어, 그리고 바퀴 로봇의 확률적 이동을 다루지만, "질량과 관성을 가진 링크를 이 속도로 가속하려면 모터에 몇 Nm의 토크가 필요한가"나 "발이나 손끝이 바닥·물체와 부딪힐 때 반력은 어떻게 작용하는가"에는 답하지 못한다. 질량·관성·중력·접촉력을 포함해 운동의 **원인**을 계산하는 동역학은 5장에서 이어진다.

---

## 참고 자료

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

## 발전 과정

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
