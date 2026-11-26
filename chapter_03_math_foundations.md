# 3장 수학적 기초

2장에서는 카메라, LiDAR, IMU 같은 센서가 물리적 세계를 어떻게 디지털 측정값으로 바꾸는지 살펴봤다. 하지만 센서가 건네주는 것은 잡음이 섞인 픽셀, 점군, 가속도 시계열일 뿐, 로봇이 당장 필요로 하는 "내가 지금 어디에 있고 어느 방향을 보고 있는가"라는 답이 아니다. 이 측정값들을 하나의 일관된 공간 좌표로 엮고, 불확실성을 다루며, 최적의 해를 뽑아내는 언어가 바로 수학이다.

실제로 Spatial AI나 SLAM 논문을 펼쳐 보면 "SE(3) 위에서 최적화했다"거나 "Jacobian을 유도해 Gauss-Newton으로 풀었다" 같은 문장이 별다른 설명 없이 지나간다. 최근에는 PyTorch나 JAX 같은 도구가 Differentiable Programming과 자동 미분(Auto-Differentiation)을 제공해 복잡한 파이프라인의 gradient를 손으로 유도하지 않고도 구할 수 있게 됐다. 덕분에 End-to-End 학습 기반 SLAM이나 NeRF·3D Gaussian Splatting 같은 미분 가능 렌더링이 빠르게 발전했다. 그렇더라도 자동 미분이 실제로 어떤 값을 계산하는지 해석하고, 수렴하지 않는 원인을 찾아 고치려면 그 밑바탕에 놓인 선형대수, 기하학, 확률, 최적화 원리를 반드시 짚고 넘어가야 한다.

이 장은 3.1~3.4절에서 선형대수, 3D 기하학, 확률·통계, 최적화의 뼈대를 먼저 세운 뒤, 3.5~3.11절의 심화 절에서 현대 SLAM과 상태 추정의 핵심 엔진인 Lie Group, Factor Graph, Robust Estimation, 정보 이론, 베이즈·칼만·입자 필터를 차례로 다룬다. 여기서 익힌 좌표 변환과 Jacobian, $SE(3)$ 개념은 바로 이어지는 4장(기구학과 메카트로닉스)에서 로봇 팔의 관절과 끝단을 연결할 때도 그대로 쓰인다.

## 3.1 선형대수

선형대수는 Spatial AI와 로보틱스 전반을 관통하는 공용어다. 3D 공간의 좌표 변환부터 카메라 투영 모델, 비선형 최적화의 선형 근사, 딥러닝 신경망의 연산까지 모두 벡터와 행렬의 언어로 표현된다.

### 3.1.1 벡터와 행렬

로봇이 공간을 이해하려면 먼저 위치, 속도, 힘처럼 크기와 방향을 함께 지닌 물리량을 숫자의 묶음으로 적을 수 있어야 한다. 이 숫자 묶음이 바로 벡터(vector)이며, 로보틱스에서는 관례적으로 숫자를 세로로 세운 열 벡터(column vector)로 표기한다.

```
v = [v_x, v_y, v_z]^T  (열 벡터)
```

예를 들어 "로봇이 월드 좌표계에서 $(3, 2, 1)$ 지점에 서 있다"고 말할 때, 그 위치가 곧 3차원 열 벡터다.

행렬(matrix)은 이런 벡터를 다른 벡터로 옮기거나 변형하는 선형 변환(linear transformation)을 나타낸다. 여러 개의 센서 데이터나 연립방정식을 한꺼번에 다루기 위해 다음과 같은 기본 연산을 자주 쓴다.

- **덧셈과 뺄셈**: 같은 위치의 원소끼리 더하거나 빼서 벡터의 평행 이동이나 차이를 계산한다.
- **곱셈**: 앞 행렬의 행과 뒤 행렬의 열을 내적해 두 변환을 하나로 합성한다.
- **전치(Transpose, $A^T$)**: 행과 열을 뒤바꾼다. 특히 회전 행렬에서는 전치가 곧 역변환이 되어 계산량을 크게 줄여 준다.
- **역행렬(Inverse, $A^{-1}$)**: $AA^{-1} = I$를 만족하며, 변환을 거꾸로 되돌리는 역할을 한다.

카메라가 3D 공간의 점을 2D 이미지 평면에 투영하거나, 로봇의 한 부품에 묶인 좌표계를 다른 좌표계로 옮기는 계산은 모두 이 행렬 곱의 연쇄로 귀결된다. 아래 추천 자료는 계산 규칙 암기를 넘어 행렬을 "공간의 변형"으로 시각화하는 데 큰 도움이 된다.

> **추천 자료**
> - [3Blue1Brown — Essence of Linear Algebra](https://www.youtube.com/playlist?list=PLZHQObOWTQDPD3MizzM2xVFitgF8hE_ab) — 행렬 곱셈과 고유값을 기하학적으로 시각화한다. 선형대수를 계산 규칙보다 공간 변환으로 이해하는 데 유용하다.
> - [Introduction to Applied Linear Algebra (Boyd & Vandenberghe) — 무료 PDF](https://web.stanford.edu/~boyd/vmls/) — Stanford의 Boyd와 UCLA의 Vandenberghe가 쓴 응용 선형대수 교재. 응용 예제 중심이고 Julia·Python 보조 자료가 있다.
> - [다크 프로그래머 — 선형대수학 시리즈 (6편: 기본공식~PCA)](https://darkpgmr.tistory.com/103) — 주요용어, 역행렬, 고유값, SVD, 연립방정식, PCA를 한글로 정리한 글
> - [다크 프로그래머 — 벡터 미분과 행렬 미분](https://darkpgmr.tistory.com/141) — 벡터·행렬 미분 규칙을 정리한 글. Jacobian을 계산할 때 필요하다

### 3.1.2 고유값 분해 (Eigenvalue Decomposition)

정방행렬 $A$를 어떤 벡터에 곱하면 보통은 벡터의 길이가 늘어나고 방향도 함께 돌아간다. 그런데 행렬이 정해지면 유독 방향은 바뀌지 않고 크기만 $\lambda$배로 늘어나는 특별한 축들이 나타난다. 이 불변 방향과 배율을 찾는 식이 고유값 분해다.

```
Av = λv
```

여기서 $v$를 고유벡터(eigenvector), $\lambda$를 그 방향의 고유값(eigenvalue)이라 부른다. 이 개념은 데이터 분포나 공분산 행렬을 분석하고 시스템의 안정성을 판별할 때 빛을 발한다.

특히 LiDAR나 RGB-D 카메라로 얻은 3D 포인트 클라우드를 다룰 때 곧바로 쓰인다. 주성분 분석(PCA)으로 점군 데이터의 주축을 구하면, 공분산 행렬의 고유벡터는 데이터가 가장 크게 뻗어 있는 주축 방향을 가리키고 고유값은 각 축 방향의 분산 크기를 말해 준다. 세 고유값의 비율을 비교하면 이 점군이 납작한 평면인지, 가느다란 직선인지, 아니면 둥근 덩어리인지 판별할 수 있다. 평면의 법선 벡터(normal) 역시 점이 가장 얇게 퍼진 방향, 즉 가장 작은 고유값에 대응하는 고유벡터로 바로 추정한다.

> **추천 자료**
> - [3Blue1Brown — Eigenvectors and Eigenvalues](https://www.youtube.com/watch?v=PFDu9oVAE-g) — 고유벡터와 고유값의 기하학적 의미를 시각화한 영상(Essence of linear algebra 14장)
> - [MIT 18.06 Linear Algebra — Gilbert Strang (YouTube)](https://www.youtube.com/playlist?list=PLE7DDD91010BC51F8) — MIT 18.06 공개 강의. 고유값 분해를 포함한 전체 선형대수를 깊이 있게 다룬다.

> **실습**: [PCA 3D · 차원 축소](https://alexjunholee.github.io/robotics-practice/app.html#pca_3d)
> 3D 분포에서 공분산 행렬의 고유벡터가 주축이 되는 과정을 직접 조작하고, PC1·PC2 평면(3D→2D)과 PC1 축(2D→1D)으로 차원을 줄이는 과정을 함께 시각화한다.

### 3.1.3 특이값 분해 (SVD: Singular Value Decomposition)

고유값 분해는 입력과 출력 차원이 같은 정방행렬($n \times n$)에만 쓸 수 있다. 하지만 로보틱스에서는 수천 개의 센서 측정값으로 소수의 미지수를 푸는 것처럼 행과 열의 개수가 다른 직사각 행렬($m \times n$)이 훨씬 자주 등장한다. 특이값 분해(SVD)는 이 한계를 넘어, 임의의 직사각 행렬을 "직교 회전($V^T$) → 직교 축별 크기 조절($\Sigma$) → 또 한 번의 직교 회전($U$)"이라는 세 단계의 기하학적 동작으로 쪼개어 준다.

```
A = UΣV^T
```

여기서 $U$는 $m \times m$ 직교행렬(좌측 특이벡터), $\Sigma$는 0 이상의 특이값이 대각선에 놓인 $m \times n$ 행렬, $V$는 $n \times n$ 직교행렬(우측 특이벡터)이다.

SVD는 행렬이 특이(singular)하거나 정방이 아니어도 언제나 분해할 수 있고 수치적으로도 매우 안정적이어서, 로보틱스의 거의 모든 선형 문제 뒤에 숨어 있다. 식의 개수가 미지수보다 많은 과결정(overdetermined) 시스템에서 최소자승 해를 구할 때는 물론이고, 행렬의 저차원 근사를 구하거나 두 카메라 시점 사이의 Fundamental Matrix를 계산할 때도 핵심 역할을 한다. 예컨대 8-point algorithm은 8개 이상의 대응점으로 선형 해를 뽑아낼 때와, 구한 행렬에 기하학적 조건인 rank 2 제약을 걸 때 모두 SVD를 사용한다. 포인트 클라우드 정합(registration)에서 두 점군 사이의 최적 회전을 닫힌 형식으로 구하는 과정 역시 SVD 한 번으로 끝난다.

> **추천 자료**
> - [Steve Brunton — Singular Value Decomposition (YouTube)](https://www.youtube.com/watch?v=nbBvuuNVfco) — University of Washington의 Steve Brunton이 SVD의 수학적 개요를 설명한 강의
> - [Introduction to Linear Algebra, 6th ed. (Gilbert Strang, 2023)](https://math.mit.edu/~gs/linearalgebra/ila6/indexila6.html) — MIT 18.06의 교재. SVD를 별도 장으로 다룬다.

## 3.2 3D 기하학 (3D Geometry)

선형대수의 일반 도구를 손에 쥐었다면, 이제 이를 로봇이 살아가는 3차원 물리 공간에 적용할 차례다. 로봇 본체, 카메라, LiDAR, IMU는 각자 서로 다른 위치와 방향으로 움직이기 때문에, 이들의 위치와 자세(pose)를 혼동 없이 주고받을 공통의 기하학적 언어가 필요하다.

### 3.2.1 좌표계 (Coordinate Frames)

실제 로봇 시스템은 단 하나의 기준 좌표계만으로 움직이지 않는다. 방 전체를 기준으로 삼는 World Frame($W$), 로봇 몸체 중심의 Body Frame($B$), 카메라 광학 중심의 Camera Frame($C$), 관성 센서의 IMU Frame($I$) 등 여러 좌표계가 공존한다. 카메라가 앞의 물체를 $(0, 0, 2)$ 위치에서 관측했다고 할 때, 그 숫자는 카메라 기준 좌표($C$)일 뿐이다. 로봇 팔을 뻗어 그 물체를 집거나 지도에 표시하려면, 이 좌표를 로봇 중심($B$)이나 월드 좌표계($W$)로 정확히 옮겨야 한다. 이처럼 센서마다 제각각인 좌표축을 맞추려면 센서들이 로봇 몸체 위에서 서로 얼마나 떨어져 있고 기울어져 있는지 재는 extrinsic calibration이 선행되어야 한다.

좌표 변환 표기법은 방향을 헷갈리지 않는 것이 생명이다. 예를 들어 카메라 좌표계에서 잰 점 $p_C$를 월드 좌표계 기준 $p_W$로 바꾸는 식은 다음과 같이 쓴다.

```
p_W = T_WC × p_C
```

여기서 $T_{WC}$는 Camera 프레임의 점을 World 프레임으로 옮기는 $4 \times 4$ 변환 행렬(동시에 World 기준에서 바라본 Camera의 포즈)이다. 아래첨자 순서를 볼 때 오른쪽의 $C$가 왼쪽의 $W$로 바뀐다고 기억하면 복잡한 좌표 변환 식을 읽을 때 실수를 크게 줄일 수 있다.

> **추천 자료**
> - [State Estimation for Robotics, Ch.6 — Primer on Three-Dimensional Geometry (Tim Barfoot) — 무료 PDF](https://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) — 좌표계, 회전, 변환을 상태 추정 관점에서 정리한 장
> - [Stanford CS231A — Camera Models](https://web.stanford.edu/class/cs231a/) — Stanford의 CV 강의에서 카메라 좌표계와 투영 모델을 다루는 부분

### 3.2.2 회전 표현 (Rotation Representations)

3차원 평행 이동은 $(x, y, z)$ 세 숫자면 충분하고 서로 더하기만 해도 잘 합쳐진다. 반면 3차원 회전은 자유도가 3인데도, 단 세 개의 숫자만으로 어떤 특이점(singularity)도 없이 모든 방향을 매끄럽게 표현하는 방법이 수학적으로 존재하지 않는다. 이 근본적인 까다로움 때문에 로보틱스에서는 회전 행렬, 오일러 각(Euler angle), 쿼터니언(quaternion), 축-각(axis-angle)을 상황에 따라 골라 쓴다.

- **회전 행렬(Rotation Matrix $R$)**: $3 \times 3$ 직교행렬로 $R^T = R^{-1}$과 $\det(R) = 1$을 만족한다. 숫자는 9개나 쓰지만 6개의 제약이 걸려 실제 자유도는 3이다. 벡터에 바로 곱해 회전시키거나 여러 회전을 연달아 합성하기 좋지만, 9개 숫자를 그대로 최적화 변수로 쓰기엔 제약이 번거롭다.
- **오일러 각(Euler Angles)**: Roll($\phi$), Pitch($\theta$), Yaw($\psi$) 세 각도로 회전을 나눈다. 사람이 직관적으로 읽기에는 가장 편하지만, Pitch가 $\pm 90^\circ$가 되면 두 회전축이 겹치며 자유도 하나를 잃어버리는 **Gimbal Lock** 현상이 생긴다. 게다가 어느 축부터 돌리는지(ZYX, XYZ 등) 순서에 따라 전혀 다른 회전이 나오므로 규약을 철저히 맞춰야 한다.
- **쿼터니언(Quaternion $q = [w, x, y, z]$, $\|q\| = 1$)**: 4개 파라미터로 3자유도 회전을 나타낸다. 단위 초구면 위의 점으로 회전을 표현하므로 Gimbal Lock이 전혀 없고, 두 자세 사이를 최단 경로로 부드럽게 잇는 구면 선형 보간(Slerp)도 깔끔하다.
- **축-각(Axis-Angle)**: 회전축 단위 벡터 $\hat{n}$과 회전 각도 $\theta$를 묶어 3개 파라미터($\theta\hat{n}$)로 적는 방식이다. 3.5절에서 다룰 Rodrigues 공식을 거치면 곧바로 회전 행렬로 바뀐다.

실제 소프트웨어 생태계에서도 라이브러리마다 선호하는 표현이 다르다. ROS는 자세 메시지의 표준으로 쿼터니언을 쓰고, OpenCV는 회전을 Rodrigues 벡터(axis-angle)로 주고받으며, Ceres나 GTSAM 같은 최적화 라이브러리는 뒤이어 배울 Lie Group 기반 표현을 지원한다. 따라서 서로 다른 라이브러리를 이어 붙일 때는 쿼터니언의 실수부 위치(`wxyz`인지 `xyzw`인지)와 오일러 회전 순서, 단위 길이 정규화 여부를 반드시 확인해야 한다.

> **추천 자료**
> - [3Blue1Brown — Quaternions and 3D Rotation](https://www.youtube.com/watch?v=zjMuIxRvygQ) — 쿼터니언의 기하학적 의미와 3D 회전을 인터랙티브하게 시각화한 영상.
> - [State Estimation for Robotics, Ch.6 — Primer on Three-Dimensional Geometry (Tim Barfoot)](https://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) — 회전 표현과 표현 사이의 변환을 정리한 장
> - [Sola — Quaternion Kinematics for the Error-State Kalman Filter (Tech Report)](https://arxiv.org/abs/1711.02508) — VIO/INS 구현 시 쿼터니언 기반 에러 상태 칼만 필터의 수학적 기초를 정리한 테크니컬 리포트.
> - [3D Rotation Converter](https://www.andre-gaschler.com/rotationconverter/) — 쿼터니언, 오일러 각, 회전 행렬 간 변환을 확인할 수 있는 온라인 도구

> **실습**: [회전 표현과 Gimbal Lock](https://alexjunholee.github.io/robotics-practice/app.html#rotation_gimbal) | [6DoF 포즈 시각화](https://alexjunholee.github.io/robotics-practice/app.html#xyzrpy_6dof)
> 오일러 각의 Gimbal Lock 현상과 쿼터니언 회전을 직접 조작하며 비교하고, 6자유도 포즈(x, y, z, roll, pitch, yaw)를 인터랙티브하게 확인할 수 있다.

### 3.2.3 Homogeneous Coordinates

일반적인 3차원 좌표에서 회전 $R$과 평행 이동 $t$를 함께 적용하면 $p' = Rp + t$처럼 행렬 곱셈과 벡터 덧셈이 섞인다. 변환을 한 번만 할 때는 문제없지만, 4장에서 다룰 6관절 로봇 팔처럼 관절 여섯 개를 연쇄적으로 통과하거나 여러 카메라 사이의 변환을 이어 붙일 때는 식이 금세 지저분해진다. 이때 3D 점 뒤에 숫자 `1`을 덧붙여 4차원 벡터로 올리는 동차 좌표(Homogeneous Coordinates)를 쓰면, 회전과 이동을 단 하나의 $4 \times 4$ 행렬 곱으로 통일할 수 있다.

```
[X, Y, Z, 1]^T  (3D 점)

T = | R   t |   (4×4 변환 행렬)
    | 0   1 |
```

이제 강체 변환은 $p' = Tp$라는 깔끔한 행렬 곱 한 번으로 끝난다. 여러 좌표계를 거치는 연쇄 변환도 변환 행렬들을 차례대로 곱해 하나의 $T$로 합칠 수 있어, 로봇 기구학 체인과 SLAM 포즈 그래프 모두 이 형식을 표준으로 삼는다.

### 3.2.4 SE(3)와 SO(3)

동차 변환 행렬이 이루는 집합을 수학에서는 좀 더 정교하게 분류한다. 회전과 이동을 모두 포함한 3D 강체 변환 전체의 집합은 6자유도의 **SE(3)**(Special Euclidean Group), 순수하게 회전만 모은 집합은 3자유도의 **SO(3)**(Special Orthogonal Group)다.

그런데 $SE(3)$와 $SO(3)$는 평평한 유클리드 공간이 아니라 매끄럽게 휘어진 다양체(manifold)이며 동시에 군(Lie Group)의 구조를 가진다. 만약 일반적인 최적화처럼 행렬 원소에 기울기를 그냥 더해 값을 갱신하면, 회전 행렬이 지켜야 할 직교 조건($R^T R = I$)과 행렬식 조건($\det R = 1$)이 곧장 깨져 버린다. 이 문제를 풀기 위해 현대 로보틱스에서는 각 지점의 접선 공간인 **Lie Algebra**($\mathfrak{se}(3), \mathfrak{so}(3)$) 위에서 제약 없는 유클리드 공간처럼 갱신량을 구한 뒤, Exponential Map으로 다시 $SE(3)$·$SO(3)$ 위로 올려보내는 전략을 쓴다. SLAM의 Pose Graph Optimization을 이해하는 열쇠가 바로 여기에 있으며, 3.5절에서 이 원리를 바닥부터 자세히 유도한다.

> **추천 자료**
> - [State Estimation for Robotics, Ch.7 — Matrix Lie Groups (Tim Barfoot) — 무료 PDF](https://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) — SE(3), SO(3), Lie group·algebra를 로보틱스 상태 추정 관점에서 설명한 교재
> - [Sola — A Micro Lie Theory for State Estimation in Robotics (arXiv:1812.01537)](https://arxiv.org/abs/1812.01537) — Lie Group 이론을 로보틱스 상태 추정에 필요한 만큼만 간결하게 정리한 논문. 매우 실용적.

## 3.3 확률 및 통계

3.2절의 기하학이 공간을 오차 없이 재는 이상적인 자라면, 현실의 로봇은 언제나 흐릿한 자로 세상을 읽는다. 2장에서 확인했듯 카메라 이미지에는 블러와 양자화 오차가 끼고, LiDAR 거리 값에는 반사율에 따른 흔들림이 남으며, IMU는 시간이 지날수록 적분 오차가 쌓인다. 확률과 통계는 센서 측정값을 "정확히 $3.0\,\text{m}$"라는 단 하나의 숫자로 못 박는 대신, "$3.0\,\text{m} \pm 0.05\,\text{m}$ (95% 신뢰구간)"처럼 불확실성까지 함께 서술하게 해 준다. 로봇의 상태 추정이란 결국 이 확률 분포를 시간에 따라 전파하고 새 관측이 들어올 때마다 좁혀 나가는 과정이다.

### 3.3.1 정규분포 (Gaussian Distribution)

연속적인 센서 노이즈와 로봇 위치의 불확실성을 모델링할 때 가장 먼저 꺼내 드는 도구가 정규분포다. 일변량 정규분포의 확률 밀도 함수는 평균 $\mu$를 중심으로 대칭인 종 모양 곡선으로 나타난다.

```
p(x) = (1 / √(2πσ²)) × exp(-(x-μ)²/(2σ²))
```

실제 로봇의 위치나 센서 오차처럼 변수가 여러 개 얽혀 있을 때는 이를 다차원으로 확장한 다변량 정규분포를 쓴다.

```
p(x) = N(μ, Σ)
```

여기서 $\mu$는 각 변수의 기댓값을 모은 평균 벡터, $\Sigma$는 변수별 분산과 서로 간의 상관관계를 담은 공분산 행렬이다.

정규분포가 로보틱스의 사실상 표준이 된 데는 이론적 이유와 실용적 이유가 모두 있다. 먼저 중심극한정리에 따르면 수많은 작은 독립 오차 요인들이 합쳐질수록 그 결과는 자연스럽게 정규분포에 가까워진다. 게다가 정규분포는 선형 변환을 거쳐도 여전히 정규분포로 남고, 두 정규 밀도 함수를 곱해 정규화해도 다시 정규분포가 된다. 분포의 모양이 무너지지 않고 평균과 공분산만으로 닫힌 계산이 이어지기 때문에, 뒤이어 배울 칼만 필터가 실시간으로 동작할 수 있다.

> **추천 자료**
> - [3Blue1Brown — But what is the Central Limit Theorem?](https://www.youtube.com/watch?v=zeJD6dqJ5lo) — 중심극한정리를 시각적으로 설명하고, 정규분포가 여러 곳에서 나타나는 이유를 보여 준다.
> - [Kalman Filter — How it works, in pictures](https://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/) — 칼만 필터의 작동 원리를 시각적으로 설명. 수식 전에 직관을 잡기 좋다

> **실습**: [Kalman Filter](https://alexjunholee.github.io/robotics-practice/app.html#kalman_filter)
> 칼만 필터의 predict-update 사이클을 인터랙티브하게 조작하며, 정규분포 기반 상태 추정 과정을 확인할 수 있다.

**Mahalanobis Distance**

다변량 분포를 다룰 때는 거리를 재는 기준도 달라져야 한다. 보통 쓰는 유클리드 거리는 $x, y, z$ 모든 방향을 똑같은 자로 잰다. 하지만 실제 센서는 방향마다 정밀도가 크게 다르다. 예를 들어 GPS는 수평 오차보다 수직 고도 오차가 훨씬 크고, 스테레오 카메라는 가로·세로 방향보다 깊이(depth) 방향 불확실성이 길게 늘어진다.

Mahalanobis 거리는 이처럼 방향마다 다른 불확실성, 즉 공분산 $\Sigma$의 역행렬을 가중치로 넣어 거리를 재조정한다.

```
d_M = sqrt((x - μ)^T Σ^{-1} (x - μ))
```

공분산 $\Sigma$가 단위 행렬이면 유클리드 거리와 똑같아지고, 대각 행렬이면 각 축을 표준편차로 나눈 거리가 된다. 변수 사이에 상관이 있는 일반적인 $\Sigma$에서는 분포 타원체의 주축 방향에 맞춰 스케일을 보정한 거리가 나온다.

SLAM에서 이 거리는 데이터 연관(data association)의 핵심 잣대로 쓰인다. 새 관측이 기존 랜드마크에서 나온 것인지 판별할 때, 유클리드 거리상으로는 가까워 보여도 불확실성이 매우 작은 방향에서 어긋나 Mahalanobis 거리가 크게 나온다면 잘못된 매칭일 가능성이 높다.

(참고: [다크 프로그래머 — 평균, 표준편차, 분산, 그리고 Mahalanobis 거리](https://darkpgmr.tistory.com/41))

### 3.3.2 베이즈 정리 (Bayes' Rule)

정규분포로 불확실성의 모양을 정했다면, 이제 새 센서 값이 들어왔을 때 기존 믿음을 어떻게 고쳐 나갈지 정할 차례다. 베이즈 정리는 관측 결과 $B$를 확인한 뒤 숨겨진 원인 $A$의 사후 확률을 계산하는 규칙이다.

```
P(A|B) = P(B|A) × P(A) / P(B)
```

로보틱스의 상태 추정에서는 원인 $A$가 로봇의 상태 $x_t$이고, 결과 $B$가 지금까지 모은 센서 측정값 $z_{1:t}$다. 이전 시점까지의 예측에 새 측정 $z_t$를 결합해 현재 상태의 확률을 갱신하는 식은 다음과 같이 쓴다.

```
P(x_t | z_{1:t}) ∝ P(z_t | x_t) × P(x_t | z_{1:t-1})
```

- **$P(z_t | x_t)$ (관측 모델, Measurement Model)**: "로봇이 정말 $x_t$에 있다면 센서가 지금처럼 $z_t$를 출력할 확률은 얼마인가?"라는 우도(likelihood)다.
- **$P(x_t | z_{1:t-1})$ (사전 예측, Prior)**: 직전까지의 정보만으로 예측한 현재 위치 확률이다.

새 센서 데이터가 들어올 때마다 사전 예측과 관측 우도를 곱해 사후 확률(posterior)을 업데이트한다. 3.10절의 칼만 필터와 3.11절의 입자 필터는 표현 방식과 근사 방법만 다를 뿐, 모두 이 베이즈 갱신을 구현한 알고리즘이다. 이 갱신 식을 시간 축을 따라 재귀적으로 엄밀하게 세운 형태가 3.9절의 베이즈 필터다.

> **추천 자료**
> - [3Blue1Brown — Bayes' Theorem](https://www.youtube.com/watch?v=HZGCoVF3YvM) — 베이즈 정리를 시각적으로 이해하기 좋은 영상
> - [Probabilistic Robotics (Thrun, Burgard, Fox)](http://probabilistic-robotics.org/) — 베이즈 필터, 칼만 필터, 입자 필터, SLAM을 확률론의 틀로 정리한 교과서.
> - [김기섭 블로그 — Bayesian Filtering 시리즈 (2편)](https://gisbi-kim.github.io/blog/2021/03/09/bayesfiltering-1.html) — 베이즈 필터를 한글로 해설하고 칼만 필터까지 이어지는 글

### 3.3.3 MLE와 MAP

확률 분포 전체를 추적하는 대신, 가장 그럴듯한 "단 하나의 최적 상태 $x^*$"를 점 추정(point estimation)해야 할 때도 많다. 이때 사전 믿음을 반영하느냐에 따라 MLE와 MAP로 나뉜다.

먼저 **MLE(Maximum Likelihood Estimation)**는 오직 현재 주어진 관측 데이터 $z$가 나올 확률만을 가장 크게 만드는 $x$를 고른다.

```
x* = argmax P(z | x)
```

반면 **MAP(Maximum A Posteriori)**는 베이즈 정리를 바탕으로 관측 우도에 사전 확률 $P(x)$까지 곱해 사후 확률 전체를 최대화한다.

```
x* = argmax P(x | z) = argmax P(z | x) × P(x)
```

SLAM에 비유하면 MLE는 "지금 들어온 카메라·LiDAR 관측만 보고 로봇 위치를 끼워 맞추는 것"이고, MAP는 "방금 전 로봇 위치와 이동량에 대한 사전 믿음까지 함께 고려해 가장 타당한 위치를 찾는 것"에 가깝다. 실전 SLAM 시스템은 거의 예외 없이 MAP 추정을 채택한다. 센서가 잠깐 가려지거나 노이즈가 튀어도 prior가 해를 안정적으로 붙잡아 주기 때문이다.

여기서 중요한 연결고리가 하나 드러난다. 곱셈을 덧셈으로 바꾸기 위해 목적 함수에 음의 로그($-\log$)를 취해 보자. 관측 노이즈가 가우시안이고 공분산이 주어져 있다면, 최대화 문제였던 MAP는 곧바로 "가중 관측 오차의 제곱합 + 사전 믿음에서 온 정규화 항"을 최소화하는 문제로 바뀐다. 가우시안 prior를 두면 이 추가 항이 정확히 L2 정규화 꼴이 되며, 딥러닝 학습에서 쓰는 weight decay 역시 가우시안 prior를 얹은 MAP 추정으로 해석할 수 있다. 이처럼 SLAM에서 odometry 우도와 랜드마크 관측 우도, 이전 상태의 prior를 모두 곱해 MAP 추정을 하는 구조가 바로 3.6절 Factor Graph의 토대다.

(참고: [다크 프로그래머 — 베이즈 정리, ML과 MAP, 그리고 영상처리](https://darkpgmr.tistory.com/62))

> **추천 자료**
> - [Probabilistic Robotics, Ch.2 — Recursive State Estimation (Thrun)](http://probabilistic-robotics.org/) — MLE, MAP, 베이즈 필터의 관계를 로보틱스 맥락에서 설명
> - [StatQuest — Maximum Likelihood, clearly explained](https://www.youtube.com/watch?v=XepXtl9YKwc) — Josh Starmer가 MLE를 예시로 설명한 영상

## 3.4 최적화 기초

앞 절에서 확인했듯 MAP 추정에 음의 로그를 취하면 확률 문제가 곧바로 비용 함수(cost function) 최소화 문제로 탈바꿈한다. SLAM의 Bundle Adjustment, 카메라 캘리브레이션, 딥러닝 학습은 겉모습은 달라도 결국 모두 이 목적 함수를 바닥까지 끌어내리는 최적화다. 잔차(residual), Jacobian, 갱신 규칙의 역할을 또렷이 구분해 두면, 실제 코드가 수렴하지 않고 발산할 때 모델 식·초깃값·솔버 설정 중 어디에 탈이 났는지 빠르게 짚어낼 수 있다.

### 3.4.1 Least Squares

노이즈가 섞인 센서로 측정을 거듭하다 보면 미지수 개수보다 방정식 개수가 훨씬 많은 과결정(overdetermined) 시스템을 만나게 된다. 현실의 오차 때문에 $Ax = b$를 완벽히 만족하는 해는 존재하지 않는다. 이때 가장 자연스러운 타협은 각 식의 오차(잔차) $Ax - b$의 제곱합을 최소화하는 해를 찾는 일이며, 이를 최소자승법(Least Squares)이라 한다.

```
x* = argmin ||Ax - b||²
```

제곱 오차는 모든 곳에서 매끄럽게 미분되고 큰 오차에 더 무거운 벌점을 준다. 특히 노이즈가 가우시안일 때는 최소자승 해가 MLE 해와 정확히 일치한다. 목적 함수를 $x$로 미분해 0으로 놓으면 정규방정식(Normal Equation) $A^T A x = A^T b$가 나온다. 여기서 $A$가 full column rank라면 역행렬을 취해 닫힌 해를 단번에 구할 수 있다.

**정규방정식의 해 (Normal Equation, `A`가 full column rank인 경우)**:

```
x* = (A^T A)^(-1) A^T b
```

한 가지 주의할 점이 있다. 관측 데이터가 한쪽으로 치우치거나 중복된 변수가 있어 $A^T A$가 특이 행렬(singular, rank 부족)이 되면 위 역행렬이 존재하지 않고 해 또한 유일하지 않게 된다. 이럴 때는 3.1.3절에서 배운 SVD가 구원투수로 나선다. `A = U Σ V^T`로 분해해 Moore–Penrose pseudo-inverse `A^+ = V Σ^+ U^T`를 만들면, 특이값이 0인 방향을 건드리지 않는 가장 작은 크기의 해(최소 norm 해) `x = A^+ b`를 안전하게 구할 수 있다.

(참고: [다크 프로그래머 — 최소자승법 이해와 다양한 활용예](https://darkpgmr.tistory.com/56))

> **추천 자료**
> - [Cyrill Stachniss — Least Squares for Robotics](https://www.youtube.com/watch?v=r2cyMQ5NB1o) — 최소자승법을 로보틱스 문제에 적용하는 방법을 구체적으로 설명
> - [김기섭 블로그 — SLAM back-end 시리즈 (3편)](https://gisbi-kim.github.io/blog/2021/03/04/slambackend-1.html) — "SLAM은 Ax=b를 푸는 문제"에서 시작하는 back-end 입문. Factor graph까지 3편 시리즈
> - [김기섭 블로그 — Iterative Optimization 1편](https://web.archive.org/web/20240819154205/https://gisbi-kim.github.io/blog/2021/03/16/leastsquare-1.html) — 비선형 최적화의 직관적 한글 해설

### 3.4.2 Gradient Descent

선형 모델 $Ax = b$는 정규방정식 한 번으로 답이 나오지만, 카메라 원근 투영이나 삼각함수가 들어간 로봇 기구학, 회전 변환은 모두 비선형이다. 이처럼 닫힌 해가 없는 비선형 목적 함수 $f(x)$의 최솟값을 찾는 가장 기본적인 반복법이 경사하강법(Gradient Descent)이다. 현재 위치에서 함숫값이 가장 가파르게 내려가는 방향, 즉 기울기(gradient)의 반대 방향으로 보폭 $\alpha$만큼 한 걸음씩 내딛는다.

```
x_{k+1} = x_k - α × ∇f(x_k)
```

여기서 $\alpha$는 학습률(learning rate), $\nabla f$는 현재 지점의 gradient다. 딥러닝 학습에서는 확률적 변형인 SGD나 Adam이 주력으로 쓰이지만, 로보틱스의 비선형 최소자승 문제에서는 경사하강법이 고전하기 쉽다. 골짜기가 좁고 길쭉한 지형에서 보폭 $\alpha$를 맞추기 까다롭고, 좌우로 진동하며 매우 느리게 수렴하기 때문이다. 그래서 SLAM과 캘리브레이션에서는 2차 곡률 정보까지 반영하는 Gauss-Newton이나 Levenberg-Marquardt(LM)를 표준으로 쓴다.

이를 이해하려면 **Gradient, Jacobian, Hessian** 세 미분 도구의 차이를 먼저 정리해 두어야 한다.

- **Gradient($\nabla f$)**: 스칼라 함수 $f: \mathbb{R}^n \to \mathbb{R}$의 1차 편미분을 모은 $n \times 1$ 벡터로, 값이 가장 빠르게 커지는 오르막 방향을 가리킨다.
- **Jacobian($J$)**: 이를 벡터 함수 $f: \mathbb{R}^n \to \mathbb{R}^m$로 넓힌 $m \times n$ 행렬이다. $(i, j)$ 원소는 $i$번째 출력 성분을 $j$번째 입력 변수로 편미분한 값이다.
- **Hessian($H$)**: 스칼라 함수 $f$의 2차 편미분을 모은 $n \times n$ 대칭 행렬이다. 지형이 얼마나 뾰족하거나 완만하게 휘었는지(곡률)를 알려 주어 Newton 방법의 핵심 재료가 된다.

최소자승 비용 함수 $C(x) = \frac{1}{2}\|r(x)\|^2$에서는 잔차 $r$의 Jacobian $J$만으로 gradient와 근사 Hessian을 손쉽게 조합할 수 있다.

관계:
```
비용 함수 C(x) = (1/2)||r(x)||² 일 때:
  Gradient:  ∇C = J^T r           (J는 r의 Jacobian)
  Hessian:   H ≈ J^T J            (Gauss-Newton 근사: 2차 미분 항 무시)
  Update:    δx = -(J^T J)^{-1} J^T r
```

원래 정확한 Hessian에는 잔차의 2차 미분($r \cdot \nabla^2 r$)이 더 붙지만 계산 비용이 너무 크다. 게다가 최적해 근처처럼 잔차 $r$ 자체가 작아지는 영역에서는 그 2차 항도 덩달아 작아지므로, $H \approx J^T J$라는 근사가 놀라울 만큼 잘 들어맞는다.

(참고: [다크 프로그래머 — Gradient, Jacobian 행렬, Hessian 행렬, Laplacian](https://darkpgmr.tistory.com/132))

### 3.4.3 Gauss-Newton

Gauss-Newton 방법은 비선형 최소자승 문제를 단번에 푸는 대신, 현재 추정값 $x_k$ 주변에서 잔차 벡터 $r(x)$를 1차 테일러 전개로 평평하게 선형화한 뒤 3.4.1절의 선형 최소자승 문제를 푸는 과정을 되풀이한다. 위에서 본 근사 Hessian을 그대로 대입하면 매 반복 단계의 갱신량 $\Delta x$는 다음 선형 연립방정식의 해로 구해진다.

```
(J^T J) Δx = -J^T r
x_{k+1} = x_k + Δx
```

여기서 $J$는 현재 추정값에서 구한 Jacobian 행렬이고 $r$은 잔차 벡터다. 경사하강법이 기울기만 보고 보폭을 감으로 정한다면, Gauss-Newton은 $J^T J$에 담긴 곡률 정보를 활용해 이차함수 바닥까지 한 번에 다다를 보폭과 방향을 함께 계산한다. 덕분에 변수가 수천에서 수만 개에 달하는 SLAM 문제라도 초기 추정값이 해와 크게 어긋나지 않는 한 단 몇 번의 반복만으로 빠르게 수렴한다.

### 3.4.4 Levenberg-Marquardt (LM)

하지만 Gauss-Newton에도 약점이 있다. 초깃값이 나빠 선형 근사가 빗나가거나 $J^T J$가 특이 행렬에 가까워지면, 계산된 한 걸음 $\Delta x$가 너무 커져 오히려 비용이 폭발하고 발산해 버린다. Levenberg-Marquardt(LM) 알고리즘은 정규방정식의 대각선에 감쇠(damping) 항 $\lambda I$를 더해 이 불안정성을 해결한다.

```
(J^T J + λI) Δx = -J^T r
```

여기서 $\lambda$는 상황에 따라 보폭과 방향을 조절하는 damping factor다. $\lambda$가 0에 가까워지면 LM은 순수한 Gauss-Newton으로 돌아가 빠르게 수렴하고, 반대로 $\lambda$가 아주 커지면 $\Delta x \approx -\frac{1}{\lambda} J^T r$가 되어 보폭은 작지만 언제나 내리막을 보장하는 경사하강법처럼 움직인다.

실제 최적화에서는 한 걸음 내디딜 때마다 비용이 줄었는지 확인해 $\lambda$를 동적으로 조절한다. 갱신 후 비용이 실제로 줄었다면 선형 근사가 믿을 만하다는 뜻이므로 $\lambda$를 줄여 Gauss-Newton 쪽으로 속도를 높이고, 비용이 오히려 늘었다면 발걸음을 무르고 $\lambda$를 키워 더 안전한 방향과 더 짧은 보폭으로 다시 시도한다. 이러한 신뢰 영역(trust region) 전략 덕분에 LM은 초깃값에 덜 민감하면서도 수렴 속도가 빨라, Ceres Solver·g2o·GTSAM 등 거의 모든 현대 SLAM 최적화 라이브러리의 기본 솔버로 탑재되어 있다.

(참고: [다크 프로그래머 — 함수최적화 기법 정리 (LM 방법 등)](https://darkpgmr.tistory.com/142))

> **추천 자료**
> - [Cyrill Stachniss — Graph-based SLAM with Gauss-Newton Optimization](https://www.youtube.com/watch?v=hRyL5KwFLAE) — Freiburg 강의(2009–2010). Graph-based SLAM을 Gauss-Newton으로 최적화하는 과정을 설명한다
> - [State Estimation for Robotics, Ch.4 — Nonlinear Non-Gaussian Estimation (Tim Barfoot) — 무료 PDF](https://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) — 비선형 추정과 최적화를 상태 추정 관점에서 다룬다
> - [Ceres Solver Tutorial](http://ceres-solver.org/tutorial.html) — Google이 공개한 비선형 최소자승 라이브러리의 튜토리얼. LM을 코드에서 쓰는 방법을 따라 해 볼 수 있다.
> - [다크 프로그래머 — 최적화 기법의 직관적 이해](https://darkpgmr.tistory.com/149) — Gradient Descent, Newton, LM 등의 기하학적 직관
> - [김기섭 블로그 — SLAM Back-end 공부자료 5개 추천](https://gisbi-kim.github.io/blog/2021/10/03/slam-textbooks.html) — Error-state KF, Factor Graphs, Bundle Adjustment 등 핵심 자료를 추천한 글
> - [Derivative Calculator](https://www.derivative-calculator.net/) — 수식 미분을 단계별로 보여주는 온라인 도구. Jacobian 유도할 때 검산에 유용

## 3.5 Lie Group과 Lie Algebra [심화]

지금까지 3D 공간을 표현하고 최적화를 돌리는 기본 재료는 모두 모였다. 이제부터 3.5~3.11절에서는 이 기본 도구를 조합해 현대 SLAM과 상태 추정의 핵심 엔진을 세우는 과정을 심화 주제로 다룬다. (당장 4장의 로봇 기구학으로 넘어가고 싶다면 3.5.1~3.5.4절의 핵심 아이디어만 먼저 훑어보고, 나중에 SLAM과 VIO를 다룰 때 다시 돌아와도 좋다.)

로보틱스 최적화에서 가장 자주 부딪히는 벽은 "회전을 어떻게 미분하고 업데이트할 것인가"라는 문제다. Lie Group과 Lie Algebra는 바로 이 회전과 강체 변환을 미분과 최적화의 언어로 매끄럽게 연결하는 수학적 틀로, SLAM 백엔드와 Visual-Inertial Odometry, Bundle Adjustment 수식 전반에 빠짐없이 등장한다.

### 3.5.1 왜 Lie Group이 필요한가

3.2절에서 살펴본 회전 표현들을 그대로 3.4절의 Gauss-Newton이나 LM 솔버에 집어넣으면 곧바로 난관에 부딪힌다.

- **회전 행렬 $R$**: 원소는 9개지만 실제 자유도는 3뿐이라 $R^T R = I$와 $\det(R) = 1$이라는 엄격한 제약을 지켜야 한다. 제약 없는 일반 최적화로 행렬 원소에 $\Delta R$을 그냥 더해 버리면, 갱신된 행렬은 더 이상 순수한 회전이 아닌 찌그러진 변환이 된다.
- **쿼터니언 $q$**: 4개 원소에 단위 길이 제약($\|q\| = 1$)이 걸려 있어, 한 걸음 내디딜 때마다 강제로 정규화를 다시 해야 하고 이 과정에서 수치 오차가 쌓이기 쉽다.
- **오일러 각**: Gimbal lock이라는 특이점을 안고 있으며, $2\pi$ 경계에서 각도가 튀는 wrap-around 문제도 까다롭다.

문제의 본질은 회전이 평평한 유클리드 공간($\mathbb{R}^n$)이 아니라 휘어진 다양체(manifold) 위에 놓여 있다는 데 있다. 반면 Gauss-Newton과 LM은 덧셈과 뺄셈이 자유로운 유클리드 공간을 전제로 설계되었다. Lie Group 이론은 이 간극을 아주 우아하게 메운다. 곡면 위의 현재 위치(예: 회전 행렬 $R$)에 평평한 접선 공간(tangent space), 즉 **Lie Algebra**를 덧대어 그 위에서 유클리드 공간처럼 자유롭게 갱신량 $\delta\xi$를 계산한 뒤, **Exponential Map**으로 결과를 다시 곡면 위로 되돌려 놓는다.

> **배경 지식: 군(Group)이란 무엇인가**
> 수학에서 **군(group)**은 어떤 집합과 연산 하나가 만나 다음 조건을 만족하는 구조를 뜻한다. ① 연산에 대해 닫혀 있고(closure), ② 결합법칙이 성립하며(associativity), ③ 항등원(identity)과 ④ 역원(inverse)이 존재한다. 예를 들어 역행렬이 존재하는 $n \times n$ 행렬 전체는 행렬 곱에 대해 군을 이루며 이를 일반 선형군 $GL(n)$이라 부른다. 그 가운데 행렬식이 $\det = 1$인 행렬만 추리면 특수 선형군 $SL(n)$이 된다. 한편 벡터의 길이와 내적을 보존하는 직교 행렬들의 모임은 직교군 $O(n)$이고, 여기서 거울 반사($\det = -1$)를 빼고 $\det = 1$인 순수 회전만 남긴 부분군이 바로 **특수 직교군 $SO(n)$**이다. (반사 행렬 두 개를 곱하면 $\det = (-1)(-1) = 1$이 되어 원래 집합을 벗어나므로, $\det = -1$인 집합 자체는 군을 이루지 못한다.)

### 3.5.2 SO(3): 3D 회전군

3차원 회전 전체가 이루는 군 $SO(3)$는 다음과 같이 정의된다.

```
SO(3) = { R in R^{3x3} | R^T R = I, det(R) = 1 }
```

$SO(3)$는 행렬 곱에 대해 군의 네 조건을 모두 만족한다. 두 회전 $R_1, R_2$를 연달아 곱한 $R_1 R_2$ 역시 여전히 $SO(3)$ 안에 머무르고, 항등원은 단위 행렬 $I$, 역원은 직교성 덕분에 곧바로 전치 행렬 $R^T(= R^{-1})$가 된다. 결합법칙은 성립한다. 다만 3D 회전은 순서에 민감하므로 일반적으로 교환법칙은 성립하지 않는다($R_1 R_2 \neq R_2 R_1$).

**Lie algebra $\mathfrak{so}(3)$:**

이제 항등원 $I$ 근처에서 무한히 작은 회전을 살펴보자. $SO(3)$의 접선 공간인 Lie algebra $\mathfrak{so}(3)$는 $3 \times 3$ 반대칭 행렬(skew-symmetric matrix)들이 이루는 3차원 벡터 공간이다. 3차원 벡터 $w$를 반대칭 행렬로 바꾸어 주는 연산을 **Hat operator** `[.]x`라 부른다.

```
w = [w1, w2, w3]^T  (in R^3)

        [  0   -w3   w2 ]
[w]x =  [  w3   0   -w1 ]   in so(3)
        [ -w2   w1   0  ]
```

이 행렬은 다른 벡터 $v$에 곱했을 때 정확히 벡터 외적(cross product)과 같은 결과를 낸다: `[w]x v = w x v`. 반대로 반대칭 행렬에서 원래 3차원 벡터 $w$를 꺼내는 역연산은 **Vee operator** `(.)v`다. 직관적으로 $\mathfrak{so}(3)$의 원소 $w$는 그 방향이 회전축을, 크기 $\|w\|$가 회전각을 뜻하므로 앞서 3.2.2절에서 본 축-각(axis-angle) 표현과 똑같은 정보를 담는다.

### 3.5.3 Exponential Map과 Logarithmic Map

**Exponential map**: `so(3) -> SO(3)`

Exponential map은 접선 공간 $\mathfrak{so}(3)$에서 구한 작은 회전 벡터를 실제 곡면 $SO(3)$ 위의 회전 행렬로 옮겨 주는 사상이다. 이 식이 단순한 정의가 아니라 물리 법칙에서 자연스럽게 튀어나온다는 사실을 간단한 미분으로 확인해 보자.

시간 $t$에 따라 부드럽게 돌아가는 회전 행렬 $R(t)$가 $R(0) = I$에서 출발한다고 하자. 어느 순간에도 $R(t)$는 $SO(3)$의 원소이므로 항상 `R(t) R(t)^T = I`를 만족한다. 이 항등식의 양변을 시간 $t$로 미분해 보자.

```
d/dt (R R^T) = R_dot R^T + R R_dot^T = 0
→  R_dot R^T = -(R_dot R^T)^T
```

우변을 보면 `R_dot R^T`가 전치했을 때 부호가 뒤집히는 반대칭 행렬임을 알 수 있다. 따라서 이 행렬을 어떤 3차원 각속도 벡터 $w(t)$의 hat 형태 `[w(t)]x`로 쓸 수 있다.

```
R_dot(t) R^T(t) = [w(t)]x
→  R_dot(t) = [w(t)]x R(t)
```

만약 각속도 $w$가 시간에 따라 변하지 않는 상수라면, 이 선형 미분 방정식의 해는 익숙한 행렬 지수 함수(matrix exponential)로 구해진다.

```
R(t) = exp([w]x * t) = sum_{n=0}^{inf} ([w]x * t)^n / n!
```

단위 시간 $t=1$을 대입한 `exp([w]x)`가 바로 축 $w$를 중심으로 $\|w\|$ 라디안만큼 돌리는 회전 행렬이다. 컴퓨터로 무한 급수를 끝까지 더할 수는 없으므로, 회전각을 $\theta = \|w\|$로 두고 `[w]x^3 = -theta^2 [w]x`라는 성질을 이용해 $\sin(\theta)$와 $\cos(\theta)$의 테일러 급수로 묶으면 그 유명한 **Rodrigues 공식**이 닫힌 형식으로 나온다.

```
exp([w]x) = I + (sin(theta) / theta) [w]x + ((1 - cos(theta)) / theta^2) [w]x^2
```

회전각 $\theta$가 0에 아주 가까울 때($|\theta| < \varepsilon$)는 분모가 0이 되는 수치 불안을 피하기 위해 $\sin(\theta)/\theta \approx 1$, $(1 - \cos(\theta))/\theta^2 \approx 1/2$ 근사를 쓴다.

```
exp([w]x) ≈ I + [w]x + (1/2)[w]x^2   (2차 항까지의 근사)
```

여기서 한 가지 짚고 넘어가자. 하나의 회전 행렬 $R$을 만드는 $w$는 단 하나가 아니다. 같은 회전축 방향으로 크기만 $2\pi k$ (정수 $k$)만큼 더 돌려도 결과 행렬 $R$은 똑같기 때문이다. 따라서 반대 방향으로 되돌아갈 때는 주의를 기울여야 한다.

**Logarithmic map**: `SO(3) -> so(3)`

Logarithmic map은 Exponential map을 거꾸로 뒤집어, 주어진 회전 행렬 $R$로부터 축-각 벡터 $w$를 복원하는 사상이다.

```
theta = arccos((tr(R) - 1) / 2)
[w]x = (theta / (2 sin(theta))) (R - R^T)
```

구현할 때 $\theta \approx 0$ (항등 회전 근처)에서는 테일러 근사를 쓰고, $\theta \approx \pi$ ($180^\circ$ 회전 근처)에서는 $\sin(\theta) \to 0$이 되어 부호가 모호해지므로 대각 원소에서 회전축을 따로 뽑아내는 예외 처리가 꼭 들어간다.

요컨대 Lie algebra는 곡면 위 기준점(보통 항등원 $I$)에 맞닿은 평평한 접선 공간이다. 미소한 회전 변화 $\delta w$는 접선 공간 $\mathbb{R}^3$의 일반 벡터처럼 덧셈·뺄셈으로 다룰 수 있고, 마지막에 `exp([dw]x)`를 현재 회전에 곱해 주기만 하면 다양체에서 한 치도 벗어나지 않고 안전하게 회전을 갱신할 수 있다.

### 3.5.4 SE(3): 3D 강체 변환군

실제 로봇의 포즈에는 회전뿐 아니라 평행 이동도 함께 담긴다. **SE(3)**는 이 둘을 하나의 $4 \times 4$ 동차 변환 행렬로 묶어 다루는 강체 변환군이다.

```
SE(3) = { T = [ R  t ] | R in SO(3), t in R^3 }
              [ 0  1 ]
```

$SE(3)$ 역시 행렬 곱 $T_1 T_2$에 대해 군을 이룬다. 항등원은 $4 \times 4$ 단위 행렬이고, 역원은 회전 전치와 역이동을 결합한 $T^{-1} = \begin{bmatrix} R^T & -R^T t \\ 0 & 1 \end{bmatrix}$다.

**Lie algebra $\mathfrak{se}(3)$:**

$SE(3)$는 6자유도이므로 그 Lie algebra $\mathfrak{se}(3)$ 역시 6차원 공간이다. 원소 $\xi$는 보통 선형 성분 $\rho$와 각성분 $w$를 쌓은 **twist** 벡터로 적는다.

```
xi = [rho; w] in R^6     (rho in R^3: 이동 성분, w in R^3: 회전 성분)
```

Hat operator는 이 6차원 벡터를 다음과 같은 $4 \times 4$ 행렬로 펼친다. (문헌에 따라 $[\rho; w]$ 대신 $[w; \rho]$ 순서를 쓰기도 하므로 라이브러리의 벡터 순서를 꼭 확인하자.)

```
        [ [w]x  rho ]
xi^ =   [  0     0  ]   in se(3)    (4x4 행렬)
```

**Exponential map**: `se(3) -> SE(3)`

이제 행렬 지수를 취해 `se(3)`에서 `SE(3)`로 넘어가 보자. 여기서 많은 입문자가 눈여겨보아야 할 대목이 있다. $SE(3)$는 단순히 회전과 독립 이동의 짝($SO(3) \times \mathbb{R}^3$)이 아니다. 로봇이 앞으로 움직이면서 동시에 몸을 돌리면(나사 운동, screw motion) 이동 방향 자체가 회전을 따라 계속 휘어지기 때문에, 변환 행렬의 이동 블록 $t$는 단순한 $\rho$가 아니라 $w$와 얽힌 $J\rho$가 된다.

```
exp(xi^) = [ exp([w]x)   J rho ]   in SE(3)
           [    0           1   ]
```

이때 $\rho$ 앞에 곱해지는 $3 \times 3$ 행렬 $J$가 바로 $SO(3)$의 left Jacobian이다.

```
J = I + ((1 - cos(theta)) / theta^2) [w]x + ((theta - sin(theta)) / theta^3) [w]x^2
```

결국 6자유도 포즈(회전 3 + 이동 3) 전체가 아무런 제약 조건이 없는 6차원 벡터 $\xi \in \mathbb{R}^6$으로 매개변수화된다. 평평한 $\mathbb{R}^6$ 공간에서 최적화 문제를 푼 뒤 exponential map 한 번으로 결과를 다시 $SE(3)$ 다양체 위에 올릴 수 있기에, Lie Group은 현대 SLAM 최적화의 표준 기반이 되었다.

> **실습**: [SE(3) Pose Composition](https://alexjunholee.github.io/robotics-practice/app.html#pose_composition_3d)
> SE(3) 변환의 합성을 3D로 직접 조작하며, 회전과 이동이 결합된 강체 변환이 어떻게 연쇄되는지 확인할 수 있다.

### 3.5.5 Perturbation Model과 Jacobian

Gauss-Newton이나 LM으로 포즈 $T$를 최적화하려면, 현재 추정값 $T$를 아주 조금 흔들어 보았을 때 오차 함수 $e(T)$가 얼마나 변하는지(Jacobian) 재야 한다. 이때 $T + \delta T$처럼 행렬을 더하면 $SE(3)$를 벗어나 버리므로, 반드시 접선 공간의 작은 변화량 `d_xi`를 exponential map으로 올려 현재 포즈에 **곱하는** 섭동(perturbation) 모델을 쓴다. 곱하는 방향에 따라 두 가지 규약이 있다.

**Left perturbation (global frame 기준):**
```
T' = exp(d_xi^) * T
```

**Right perturbation (body frame 기준):**
```
T' = T * exp(d_xi^)
```

물리적으로 Left perturbation은 월드(global) 좌표계 축을 기준으로 포즈를 살짝 밀고 돌리는 것이고, Right perturbation은 로봇 몸체(body) 좌표계 축을 기준으로 살짝 움직이는 것이다. 어느 쪽을 골라도 한 규약만 일관되게 지키면 결과는 같아진다. 다만 문헌과 코드마다 선호하는 쪽이 다르다. 예컨대 Barfoot의 교재는 주로 left perturbation을 택하고, Sophus 라이브러리가 Ceres 연동을 위해 제공하는 Jacobian `Dx_this_mul_exp_x_at_0`은 right perturbation(`T·exp(x)`)을 따른다.

**Jacobian 계산:**

Left perturbation을 기준으로 잡았을 때, 에러 함수 $e(T)$의 perturbation Jacobian은 다음 극한으로 정의된다.

```
de/d(d_xi) = lim_{d_xi->0}  (e(exp(d_xi^) * T) - e(T)) / d_xi     (left perturbation의 경우)
```

이 Jacobian 행렬의 열 개수는 언제나 자유도와 같은 6개다(에러 차원 $\times 6$). 이제 유클리드 공간에서 `x <- x + dx`로 더하는 대신, $SE(3)$ 위에서는 두 단계로 갱신한다.

1. 접선 공간 $\mathbb{R}^6$ 위에서 Gauss-Newton 방정식을 풀어 증분 벡터 `d_xi`를 구한다: `d_xi = -(J^T J)^{-1} J^T e`
2. 구한 증분을 exponential map으로 올려 군 연산(행렬 곱)으로 포즈를 갱신한다: `T <- exp(d_xi^) * T`

이 절차를 따르면 별도의 직교화나 정규화 과정을 덧붙이지 않아도 $T$는 매 반복마다 $SE(3)$ 안에 정확히 머문다. g2o, GTSAM, Ceres(`Manifold`, 구 버전 `LocalParameterization`) 모두 내부적으로 바로 이 접선 공간 섭동 방식을 사용한다. 예컨대 GTSAM의 `Pose3`에는 `Pose3::Expmap()`과 `Pose3::Logmap()`이 기본 내장되어 있고, Ceres 2.1부터는 `Manifold` 인터페이스로 동일한 국소 매개변수화를 정의한다.

### 3.5.6 Adjoint Representation

로봇에는 여러 센서가 달려 있다. 만약 카메라나 IMU 좌표계에서 측정한 속도(twist $\xi_b$)를 로봇 몸체 좌표계나 월드 좌표계의 속도($\xi_a$)로 옮겨 적어야 한다면 어떻게 해야 할까? twist는 단순한 위치 벡터가 아니라 회전과 이동 속도가 한 몸으로 묶여 있어, 이를 다른 좌표계로 변환할 때는 $6 \times 6$ **Adjoint 행렬** $Ad_T$를 쓴다.

```
Ad_T = [ R    [t]x R ]   in R^{6x6}
       [ 0      R    ]
```

두 좌표계 사이의 변환 $T_{ab}$가 주어지면 twist는 다음과 같이 선형 변환된다.

```
xi_a = Ad_{T_ab} * xi_b
```

수식을 자세히 보면 선형 속도와 각속도가 서로 어떻게 얽히는지 직관적으로 드러난다. 회전 성분은 순수하게 $R$로만 돌아가지만, 이동 성분에는 레버 암(lever arm) 효과인 `[t]x R` 항이 더해져, 떨어진 거리만큼 각속도가 선형 속도에 기여하는 몫이 반영된다. (참고로 IMU가 직접 재는 각속도와 비력(specific force) 벡터는 3D 자유 벡터이므로 축 회전에는 $R$을 적용하지만, 센서 간 6-DoF 속도·오차 공분산을 함께 전파하는 VIO 시스템에서는 이 Adjoint 행렬이 좌표계 변환의 핵심으로 쓰인다.)

### 3.5.7 실무에서 쓰기

이론을 직접 구현할 일은 거의 없다. 실전 로보틱스에서는 이미 수치적 예외 처리와 최적화가 검증된 Lie Group 라이브러리를 가져다 쓴다.

C++ 진영에서 가장 널리 쓰이는 라이브러리는 Strasdat가 만든 **Sophus**다. $SO(3)$와 $SE(3)$, exponential/logarithmic map, Adjoint 연산을 깔끔하게 제공해 ORB-SLAM3를 비롯한 수많은 SLAM 시스템의 뼈대가 되었다. (반면 Kimera 같은 시스템은 GTSAM의 `Pose3`/`Rot3`로 Lie Group 연산을 처리한다.) 아래는 Sophus에서 $SE(3)$ 포즈에 작은 left perturbation을 가하고 다시 `log()`로 twist를 꺼내는 기본 사용법이다.

```cpp
#include <sophus/se3.hpp>

// SE(3) 포즈 초기화 (항등 변환)
Sophus::SE3d T_world_body;

// se(3) perturbation (6-vector): [translation; rotation]
Sophus::SE3d::Tangent delta;
delta << 0.01, 0.0, 0.0, 0.0, 0.0, 0.001;  // 작은 x-이동 + 작은 z-회전

// Left perturbation update
T_world_body = Sophus::SE3d::exp(delta) * T_world_body;

// Log map: SE(3) -> se(3)
Sophus::SE3d::Tangent xi = T_world_body.log();
```

Python과 JAX 환경에서 미분 가능한 로보틱스 파이프라인을 실험할 때는 Brent Yi가 만든 **Jaxlie**가 편리하다. JAX의 자동 미분과 자연스럽게 맞물리므로 복잡한 Jacobian을 손으로 유도하지 않고도 연구 프로토타입을 빠르게 짤 수 있다.

```python
import jaxlie
import jax.numpy as jnp

T = jaxlie.SE3.identity()
delta = jnp.array([0.01, 0.0, 0.0, 0.0, 0.0, 0.001])
T_updated = jaxlie.SE3.exp(delta) @ T
```

이 밖에도 **GTSAM**을 쓰면 `gtsam::Pose3`가 내부적으로 $SE(3)$ 연산을 품고 있어, factor graph를 최적화할 때 접선 공간 섭동과 갱신을 별도 코드 없이 자동으로 처리해 준다.

> **추천 자료**
> - [State Estimation for Robotics, Ch.7 — Matrix Lie Groups (Barfoot)](https://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) — Lie group을 로보틱스 상태 추정 관점에서 다룬 장. 8장은 이를 포즈 추정 문제에 적용한다
> - [A micro Lie theory for state estimation in robotics (Sola et al., arXiv:1812.01537)](https://arxiv.org/abs/1812.01537) — Lie group 이론을 상태 추정에 필요한 만큼 간추린 글
> - [TUM Multiple View Geometry, Ch.2 — Rigid Body Motion](https://cvg.cit.tum.de/teaching/online/mvg) — Daniel Cremers 교수의 강의. SO(3), SE(3)를 시각적으로 설명
> - [Sophus GitHub](https://github.com/strasdat/Sophus) — C++ Lie group 라이브러리. 코드를 읽으면 이해가 빨라진다
> - [정진용 블로그 — SE(3) and SO(3) transformation](https://jinyongjeong.github.io/2016/06/07/se3_so3_transformation/) — SE(3), SO(3) 변환의 한글 정리. GL(3), O(3)부터 체계적으로 설명
> - [T-Robotics: Lie Group Formulation for Robot Mechanics](http://t-robotics.blogspot.com/2015/07/lie-group-formulation-for-robot.html) — 한국어로 작성된 Lie Group 설명. 로봇 역학에 Lie Group을 쓰는 방법을 정리

## 3.6 Factor Graph [심화]

3.5절에서는 단일 포즈 $T \in SE(3)$를 다양체 위에서 업데이트하는 방법을 익혔다. 하지만 실제 SLAM은 포즈 하나만 맞추는 문제가 아니다. 로봇이 이동하는 동안 수백~수만 개의 포즈와 그 주변의 수많은 랜드마크를 동시에 추정해야 하며, 이들 사이에는 바퀴 주행 거리계, 카메라 특징점 재투영, IMU 적분, 루프 클로저 등 온갖 센서 제약이 얽혀 있다. **Factor Graph**는 이 거대하고 복잡한 확률적 최적화 문제를 한눈에 그려지는 희소 그래프로 정리해 빠르게 풀기 위한 현대 SLAM 백엔드의 표준 프레임워크다.

### 3.6.1 Factor Graph란

Factor Graph는 성격이 다른 두 종류의 노드가 번갈아 연결된 이분 그래프(bipartite graph)다.

- **변수 노드(Variable Nodes)**: 우리가 추정하고 싶은 미지수다. 시간에 따른 로봇의 포즈($x_1, x_2, \ldots$)나 지도 위의 랜드마크 좌표($l_1, l_2, \ldots$)가 여기에 해당한다.
- **팩터 노드(Factor Nodes)**: 변수들 사이에 걸린 확률적 제약이나 센서 측정을 뜻한다. 각 팩터는 자신과 연결된 몇 개의 변수만으로 오차(비용 함수)를 정의한다.

확률론 관점에서 보면, 전체 변수의 사후 확률 분포는 그래프에 매달린 모든 팩터 함수의 곱으로 깨끗하게 쪼개진다.

```
p(X | Z) proportional to  prod_i  f_i(X_i)
```

여기서 $X_i$는 $i$번째 팩터 $f_i$에 직접 연결된 소수의 변수 묶음이다. 3.3.3절에서 보았듯 사후 확률을 최대화하는 MAP 추정에서 음의 로그를 취하면 곱셈은 덧셈으로 바뀐다. 그 결과 곧바로 가중 비선형 최소자승 문제로 이어진다.

```
X* = argmin_X  sum_i  ||e_i(X_i)||^2_{Sigma_i}
```

각 항에서 $e_i$는 예측값과 측정값의 차이인 잔차 함수, $\Sigma_i$는 해당 센서의 측정 공분산이다. 즉 3.3.1절의 Mahalanobis 거리 제곱을 모든 팩터에 걸쳐 합산해 최소화하는 셈이다.

### 3.6.2 SLAM을 Factor Graph로 표현

실제 SLAM 그래프를 조립할 때 주로 쓰이는 팩터들은 다음과 같다.

| 팩터 | 역할 |
|---|---|
| Prior factor | 초기 포즈의 사전 정보. 예: "시작점은 원점이다" |
| Odometry factor | 두 연속 포즈 사이의 상대 변환. wheel odometry 등에서 온다 |
| Landmark observation factor | 포즈에서 랜드마크를 관측해 얻은 측정값. reprojection error가 대표적 |
| Loop closure factor | 이전에 방문한 장소를 재인식했을 때 추가. 전체 궤적의 드리프트를 보정하는 핵심 |
| IMU preintegration factor | 두 키프레임 사이의 IMU 측정을 하나의 팩터로 요약 |

이 팩터들이 시간에 따라 궤적에 달라붙어 만드는 그래프 구조를 간단히 그려 보면 다음과 같다.

```
 [prior]---x1---[odom]---x2---[odom]---x3
                  |                      |
              [landmark]            [landmark]
                  |                      |
                  l1                     l2

            x3 ---[loop closure]--- x1
```

로봇이 앞으로 나아가면 odometry와 랜드마크 관측 팩터만 붙어 오차가 조금씩 쌓인다. 그러다 예전에 지나왔던 $x_1$ 자리를 $x_3$에서 다시 알아보고 `loop closure` 팩터가 연결되는 순간, 그래프 전체가 탄성 있는 스프링 네트워크처럼 팽팽하게 당겨지며 누적된 드리프트가 한꺼번에 교정된다. 각 팩터에 측정값과 노이즈 모델(공분산)을 끼워 넣기만 하면, 그다음부터는 Gauss-Newton이나 LM 솔버가 전체 그래프를 동시에 최적화해 준다.

### 3.6.3 풀이: Variable Elimination과 Bayes Tree

Factor Graph를 최적화하려면 결국 매 반복마다 정규 방정식 `H d = -b`를 풀어야 한다(여기서 $H$는 근사 Hessian, $b$는 gradient다). 만약 로봇 포즈와 랜드마크가 수만 개에 달하는데 $H$를 일반 밀집 행렬처럼 다루면 역행렬 계산에 $O(n^3)$의 시간이 걸려 실시간 주행이 불가능해진다. 다행히 각 팩터는 전체 변수가 아니라 자기와 맞닿은 소수의 변수에만 연결되므로, 행렬 $H$는 원소 대부분이 `0`인 희소(sparse) 구조를 띤다. 이 희소성을 끝까지 살려 푸는 핵심 기법은 다음과 같다.

- **Variable elimination**: 그래프에서 변수를 하나씩 소거해 나가며 해를 구하는 과정으로, 선형대수에서는 희소 Cholesky 분해(sparse Cholesky factorization)와 정확히 같은 일을 한다. 이때 원래 `0`이었던 자리가 숫자로 채워지는 현상을 **fill-in**이라 한다. 어떤 순서로 변수를 지우느냐에 따라 fill-in의 양이 달라지고, 이 fill-in이 곧 전체 계산 속도를 좌우한다.
- **Variable ordering**: fill-in을 줄이려면 연결된 이웃이 적은 변수부터 먼저 소거하는 것이 유리하다. 실무에서는 COLAMD(Column Approximate Minimum Degree) 같은 정렬 휴리스틱이 표준처럼 쓰인다.
- **Bayes tree**: Kaess 등(WAFR 2010)이 제안하고 iSAM2(Kaess et al., IJRR 2012)의 심장이 된 자료구조다. Factor Graph에 variable elimination을 적용해 Bayes net을 얻고, 이를 트리 형태로 재구성해 둔다. 그러면 새 센서 측정이 들어와 그래프 끝에 팩터 하나가 추가되었을 때, 전체를 처음부터 다시 풀지 않고 실제로 영향을 받는 부분 트리(subtree)만 골라 다시 소거할 수 있다.

덕분에 프레임마다 팩터가 쉴 새 없이 붙는 실시간 SLAM에서도, 이전 계산을 재활용하는 증분 업데이트(incremental update)로 밀리초 단위의 응답 속도를 지켜 낼 수 있다. 이러한 희소 정보 행렬의 가산 구조는 뒤이어 3.10절(§3.10)에서 만날 정보 필터(Information Filter)와도 수학적으로 맞닿아 있다.

> **추천 자료**
> - [Factor Graphs and GTSAM (Dellaert & Kaess)](https://gtsam.org/tutorials/intro.html) — GTSAM 공식 튜토리얼. Factor graph가 SLAM에 어떻게 쓰이는지 설명
> - [Factor Graphs for Robot Perception (Dellaert & Kaess, 2017)](https://www.cs.cmu.edu/~kaess/pub/Dellaert17fnt.pdf) — Foundations and Trends in Robotics에 실린 단행본 분량의 리뷰
> - [CMU 16-833 Lecture Notes](https://www.cs.cmu.edu/~kaess/teaching/16833/) — Michael Kaess 교수의 SLAM 강의. Factor graph와 iSAM2를 깊이 다룬다

> **실습**: [Factor Graph 시각화](https://alexjunholee.github.io/robotics-practice/app.html#factor_graph_viz)
> Factor graph의 변수 노드와 팩터 노드를 직접 구성하고, 그래프 구조가 최적화에 미치는 영향을 확인할 수 있다.

### 3.6.4 Ceres Solver로 Pose Graph 최적화 구현하기

GTSAM이 SLAM 전용 블록을 미리 조립해 둔 도구라면, Google의 **Ceres Solver**는 범용 비선형 최소자승 솔버다. Ceres에서는 SLAM용 팩터 클래스를 직접 작성해야 하는 대신, 잔차 계산부터 자동 미분과 다양체 처리까지 솔버 내부에서 어떤 일이 벌어지는지를 코드 수준에서 투명하게 들여다볼 수 있다. 아래 발췌 코드는 Ceres 공식 예제 `pose_graph_3d`([pose_graph_3d_error_term.h](https://github.com/ceres-solver/ceres-solver/blob/master/examples/slam/pose_graph_3d/pose_graph_3d_error_term.h))의 핵심부만 추린 것으로, `Create()` 팩토리 함수와 멤버 변수 선언을 생략해 단독으로는 컴파일되지 않는다.

**Error Term 정의:**

먼저 두 포즈 $a, b$(위치 `p_a`·`p_b`, 회전 `q_a`·`q_b`) 사이의 상대 변환 측정값 `t_ab_measured`가 주어졌을 때, 현재 추정치가 측정값과 얼마나 어긋났는지를 계산하는 잔차 함수를 정의한다.

```cpp
class PoseGraph3dErrorTerm {
 public:
  PoseGraph3dErrorTerm(Pose3d t_ab_measured,
                       Eigen::Matrix<double, 6, 6> sqrt_information)
      : t_ab_measured_(std::move(t_ab_measured)),
        sqrt_information_(std::move(sqrt_information)) {}

  template <typename T>
  bool operator()(const T* const p_a_ptr, const T* const q_a_ptr,
                  const T* const p_b_ptr, const T* const q_b_ptr,
                  T* residuals_ptr) const {
    // 파라미터 포인터를 Eigen 타입으로 감싼다
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_a(p_a_ptr);
    Eigen::Map<const Eigen::Quaternion<T>> q_a(q_a_ptr);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_b(p_b_ptr);
    Eigen::Map<const Eigen::Quaternion<T>> q_b(q_b_ptr);

    // 추정된 상대 변환 계산
    Eigen::Quaternion<T> q_a_inverse = q_a.conjugate();
    Eigen::Quaternion<T> q_ab_estimated = q_a_inverse * q_b;
    Eigen::Matrix<T, 3, 1> p_ab_estimated = q_a_inverse * (p_b - p_a);

    // 측정값과 비교한 차이
    Eigen::Quaternion<T> delta_q =
        t_ab_measured_.q.cast<T>() * q_ab_estimated.conjugate();

    // residual = [position_error; orientation_error]
    Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);
    residuals.block<3,1>(0,0) = p_ab_estimated - t_ab_measured_.p.cast<T>();
    residuals.block<3,1>(3,0) = T(2.0) * delta_q.vec();

    // information matrix 적용 (covariance의 역)
    residuals.applyOnTheLeft(sqrt_information_.cast<T>());
    return true;
  }
};
```

이 코드에서 눈여겨볼 네 가지 핵심 포인트는 다음과 같다.

- **`template <typename T>`**: Ceres는 잔차 값만 필요할 때는 `T=double`로, Jacobian이 필요할 때는 이원수(dual number) 타입인 `T=Jet<double>`로 이 연산자를 호출해 자동 미분(AutoDiff)을 수행한다.
- **`sqrt_information`**: 오차를 불확실성으로 정규화하려면 공분산의 역행렬(정보 행렬)을 Cholesky 분해(`information.llt().matrixL()`)해 제곱근 행렬을 구한 뒤 잔차 왼쪽에 곱해 준다. 그러면 솔버가 계산하는 $\|r\|^2$가 곧바로 Mahalanobis 거리 제곱과 같아진다.
- **`AutoDiffCostFunction` 차원**: `<PoseGraph3dErrorTerm, 6, 3, 4, 3, 4>`는 출력 잔차가 6차원이고 입력 블록이 차례로 `pos_a`(3), `quat_a`(4), `pos_b`(3), `quat_b`(4)차원임을 뜻한다.
- **`SetManifold`**: 쿼터니언은 저장 파라미터가 4개지만 실제 자유도는 3이므로, `EigenQuaternionManifold`(구 API의 `LocalParameterization`)를 지정해 3.5절에서 배운 접선 공간 위에서 갱신되도록 묶어 준다.

**문제 구성:**

에러 항 클래스를 준비했다면, 모든 엣지 제약(`constraints`)을 순회하며 `ceres::Problem`에 잔차 블록과 쿼터니언 다양체를 등록한다. 이때 첫 번째 포즈 하나를 고정해 주지 않으면 그래프 전체가 공간 속에서 둥둥 떠다니는 기준 모호성(gauge freedom)이 남으므로 `SetParameterBlockConstant`로 기준점을 못 박는다.

```cpp
ceres::Problem problem;
ceres::LossFunction* loss_function = nullptr;  // robust loss 필요시 HuberLoss 등
ceres::Manifold* quaternion_manifold = new EigenQuaternionManifold;

for (const auto& constraint : constraints) {
    ceres::CostFunction* cost_function =
        PoseGraph3dErrorTerm::Create(constraint.t_be, sqrt_information);
    problem.AddResidualBlock(cost_function, loss_function,
                              pose_begin.p.data(), pose_begin.q.coeffs().data(),
                              pose_end.p.data(), pose_end.q.coeffs().data());
    problem.SetManifold(pose_begin.q.coeffs().data(), quaternion_manifold);
    problem.SetManifold(pose_end.q.coeffs().data(), quaternion_manifold);
}

// 첫 번째 포즈 고정 (gauge freedom 제거)
problem.SetParameterBlockConstant(poses.begin()->second.p.data());
problem.SetParameterBlockConstant(poses.begin()->second.q.coeffs().data());
```

**풀이:**

마지막으로 희소 선형 솔버 옵션(`SPARSE_NORMAL_CHOLESKY`)을 지정하고 `ceres::Solve`를 호출하면 최적화가 실행된다.

```cpp
ceres::Solver::Options options;
options.max_num_iterations = 200;
options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
ceres::Solver::Summary summary;
ceres::Solve(options, &problem, &summary);
```

`SPARSE_NORMAL_CHOLESKY`는 pose graph처럼 변수들이 성글게 이어진 문제에 잘 맞는다. 반면 Bundle Adjustment처럼 3D 점 변수 블록을 Schur complement로 먼저 소거하기 좋은 구조에서는 `SPARSE_SCHUR` 솔버를 쓰는 편이 훨씬 빠르다. 실제 프로젝트에서 GTSAM과 Ceres 중 무엇을 고를지는 아래 비교를 참고해 판단할 수 있다.

**GTSAM vs Ceres 비교**

| | GTSAM | Ceres |
|---|---|---|
| 특성 | SLAM 특화 | 범용 nonlinear least squares |
| 기본 제공 | `BetweenFactor`, `PriorFactor` 등 미리 정의된 팩터 | SLAM용 팩터 없음. cost function 직접 정의 |
| 증분 최적화 | iSAM2로 incremental 풀이 가능 | iSAM2 같은 증분 솔버 없음 |
| Manifold | Lie group 기본 지원 | `LocalParameterization` / `Manifold`로 직접 설정 |
| 적합한 상황 | SLAM 시스템 구축 | 유연한 구조가 필요할 때, 대규모 BA |

> **추천 자료**
> - [Ceres Solver 공식 pose_graph_3d 예제](https://ceres-solver.googlesource.com/ceres-solver/+/master/examples/slam/pose_graph_3d/) — 위 코드의 전체 버전
> - [Ceres Solver Tutorial](http://ceres-solver.org/tutorial.html) — AutoDiff, Manifold 개념 설명
> - [정진용 블로그 — Ceres Solver Tutorial](https://jinyongjeong.github.io/2023/07/22/Ceres_tutorial/) — Ceres Solver 발표자료와 GitHub 실습 코드. 비선형 최적화 입문에 적합

## 3.7 Robust Estimation [심화]

지금까지의 최소자승법과 Factor Graph는 센서 오차가 예쁘게 가우시안 분포를 따른다는 낙관적인 가정 위에 서 있다. 그러나 실제 환경에서 로봇은 매칭이 완전히 틀린 특징점(false match), 카메라 앞을 지나가는 행인이나 자동차 같은 동적 물체, 일시적인 센서 오작동을 피할 수 없다. 이처럼 정상 범위에서 크게 벗어난 **이상치(outlier)**가 단 하나만 섞여 들어가도 제곱 오차 최적화는 큰 타격을 입는다. Robust estimation은 이런 이상치가 전체 추정 결과를 망가뜨리지 않도록 방어하는 기술이다.

### 3.7.1 왜 필요한가

표준 최소자승법은 잔차의 제곱 $\rho(r) = r^2$을 최소화한다. 이 함수에서는 오차가 커질수록 비용이 제곱으로 치솟고 기울기($2r$)도 비례해서 커지므로, 정직한 데이터 백 개보다 엉뚱하게 튄 outlier 단 하나가 훨씬 강한 힘으로 해를 자기 쪽으로 끌어당긴다.

실제 SLAM 현장에서 이 취약점은 치명적인 장애로 이어진다.
- 복도 모양이 비슷해 잘못 연결된 loop closure 하나가 방 전체 지도를 엿가락처럼 휘어 버린다.
- 시각 특징점 매칭에서 생긴 소수의 오매칭이 Bundle Adjustment 수렴을 망친다.
- 움직이는 사람이나 차량에 찍힌 특징점이 정적 배경이라는 가정을 깨고 궤적을 흔든다.

### 3.7.2 M-Estimator

이상치의 영향력을 꺾는 첫 번째 방법은 제곱 함수 $\rho(r) = r^2$ 대신 **M-Estimator** 손실 함수를 쓰는 것이다. M-Estimator는 잔차가 어느 한계를 넘어서면 비용 증가 폭을 완만하게 눕히거나 아예 평평하게 만들어 버린다. 대표적인 M-estimator들의 식과 성질은 다음과 같다.

| M-Estimator | rho(r) | 특성 |
|---|---|---|
| L2 (표준) | r^2 | Outlier에 취약 |
| Huber | r^2 (abs(r) <= k), 2k*abs(r) - k^2 (abs(r) > k) | 작은 잔차는 L2, 큰 잔차는 L1. 여러 최적화 라이브러리가 지원 |
| Cauchy | c^2 * log(1 + (r/c)^2) | Huber보다 outlier 억제가 강함 |
| Geman-McClure | r^2 / (1 + r^2) | 극단적 outlier를 사실상 무시 |

실무에서는 우선 **Huber** 손실을 안전한 기본값으로 깔고 시작하는 경우가 많다. 작은 오차는 가우시안 가정 그대로 정밀하게 맞추되, 큰 오차의 영향력은 선형(일정한 기울기)으로 제한해 주기 때문이다. 만약 이상치 비율이 매우 높거나 이상치가 극단적으로 크게 튄다면 기울기마저 0으로 가라앉히는 **Cauchy**나 **Geman-McClure**를 고려할 수 있다. 이때 문턱 파라미터 $k$나 $c$는 정상 데이터(inlier)의 예상 잔차 통계(예: 픽셀 재투영 오차의 표준편차)에 맞춰 조정해야 한다.

코드 적용도 간단해, Ceres Solver에서는 위 `AddResidualBlock`의 두 번째 인자로 `new ceres::HuberLoss(k)`나 `ceres::CauchyLoss(c)`를 넘기기만 하면 되고, GTSAM에서는 `gtsam::noiseModel::mEstimator::Huber`로 노이즈 모델을 감싸 주면 된다.

> **실습**: [M-Estimator 비교](https://alexjunholee.github.io/robotics-practice/app.html#m_estimator)
> L2, Huber, Cauchy, Geman-McClure 등 다양한 비용 함수가 outlier에 어떻게 반응하는지 인터랙티브하게 비교할 수 있다.

### 3.7.3 RANSAC와 변종

M-estimator는 최적화 도중 큰 잔차의 가중치를 깎아 내리는 연속적 방어책이다. 반면 **RANSAC(Random Sample Consensus)**은 조합적 알고리즘으로, 아예 데이터를 정상치(inlier)와 이상치(outlier)로 칼같이 가른 뒤 깨끗한 데이터만으로 모델을 맞춘다. 특히 초기값이 전혀 없거나 이상치가 전체 데이터의 절반 가까이 차지할 때 M-estimator보다 훨씬 강인하게 살아남는다.

**기본 RANSAC 알고리즘:**
1. 모델을 결정하는 데 필요한 최소 개수의 샘플을 무작위로 뽑는다. (예: 직선이면 2점, Homography면 4쌍)
2. 뽑은 최소 샘플만으로 임시 모델을 피팅한다.
3. 전체 데이터에 대해 이 모델과의 오차를 재고, 허용 오차(threshold) 안에 들어오는 inlier 개수를 센다.
4. 1~3단계를 정해진 횟수만큼 반복해 가장 많은 inlier의 지지를 얻은 모델을 고른다.
5. 마지막으로, 선택된 모델의 모든 inlier를 모아 그 모델을 정밀하게 다시 피팅(re-fit)한다.

기본 RANSAC은 단순하고 강력하지만 무작위 추출이라 데이터에 이상치가 많으면 반복 횟수가 늘어나고, inlier 판별 기준인 threshold 설정에 결과가 민감하다. 이를 보완하기 위해 실무에서는 다음과 같은 변종들이 널리 쓰인다.

| 변종 | 핵심 아이디어 | 트레이드오프 |
|---|---|---|
| RANSAC (기본) | 무작위 샘플 → 반복 | 단순하고 구현이 쉽지만 threshold·반복 횟수에 민감 |
| PROSAC | matching score로 좋은 샘플을 먼저 시도 | 빠르게 수렴하지만 사전 품질 정보의 질에 의존 |
| Lo-RANSAC | 좋은 모델 발견 시 로컬 최적화 추가 | 정확도 향상. 원 논문에서는 표본 수가 줄어 2~3배 빨랐다 |
| MAGSAC++ | 노이즈 스케일 sigma를 하나로 정하지 않고 범위에 걸쳐 marginalize, soft inlier/outlier | 파라미터 프리에 가까우나 계산 비용이 높음 |

실제 OpenCV 코드에서도 `cv::findHomography`나 `cv::findFundamentalMat`를 호출할 때 `cv::USAC_MAGSAC` 플래그를 넘기는 것만으로 최신 MAGSAC++ 알고리즘을 손쉽게 활용할 수 있다.

> **추천 자료**
> - [State Estimation for Robotics, Ch.5 — Biases, Correspondences, and Outliers (Barfoot)](https://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) — 바이어스, 대응 문제, outlier를 다루는 실전적 챕터
> - [Hartley & Zisserman, Ch.4 — Estimation: 2D Projective Transforms](https://www.robots.ox.ac.uk/~vgg/hzbook/) — RANSAC(원 제안은 Fischler & Bolles, 1981)과 robust estimation 이론을 정리한 교재 설명
> - [다크 프로그래머 — RANSAC의 이해와 영상처리 활용](https://darkpgmr.tistory.com/61) — RANSAC의 원리, threshold 설정, 반복 횟수 계산을 한글로 설명
> - [정진용 블로그 — Bundle Adjustment의 Jacobian 계산](https://jinyongjeong.github.io/2020/03/01/Jacobian_of_BA/) — BA의 reprojection error Jacobian을 Lie algebra와 quaternion으로 유도. 손필기 수식 포함

> **실습**: [RANSAC 시각화](https://alexjunholee.github.io/robotics-practice/app.html#ransac)
> Outlier가 포함된 데이터에서 RANSAC이 inlier/outlier를 분류하고 모델을 피팅하는 과정을 단계별로 확인할 수 있다.

## 3.8 정보 이론 기초 [심화]

지금까지 다룬 기법들이 "주어진 센서 데이터에서 현재 상태를 가장 정확히 추정하는" 수동적(passive) 문제를 풀었다면, 로봇이 미지의 공간을 스스로 탐험하는 Active SLAM이나 탐색(exploration)에서는 질문이 바뀐다. "내 불확실성을 가장 빨리 줄이려면 다음 순간 어디로 움직여 어디를 쳐다봐야 하는가?" 이처럼 불확실성의 크기를 숫자로 재고 정보의 가치를 비교할 때 정보 이론을 활용한다.

먼저 확률 변수 $X$가 가진 불확실성의 크기, 즉 평균적인 놀라움의 양을 재는 기본 척도가 **Shannon entropy**다.

```
H(X) = -sum  p(x) log p(x)
```

분포가 날카롭게 한 곳에 모여 있을수록 엔트로피는 작고, 여기저기 넓게 퍼져 갈피를 못 잡을수록 엔트로피는 커진다. 가우시안 분포에서는 공분산 행렬의 행렬식(불확실성 타원체의 부피)이 클수록 엔트로피가 높다.

두 확률 분포 $p$와 $q$가 서로 얼마나 다른지 재고 싶을 때는 **KL divergence(Kullback-Leibler divergence)**를 쓴다.

```
D_KL(p || q) = sum  p(x) log(p(x) / q(x))
```

KL divergence는 $D_{KL}(p\|q) \neq D_{KL}(q\|p)$인 비대칭 척도라 엄밀한 거리는 아니지만, "참 분포 $p$를 근사 분포 $q$로 대신 표현할 때 잃어버리는 정보량"으로 분명하게 해석된다. 뒤이어 나오는 변분 추론이나 강화학습의 정책 제약에서도 단골로 쓰인다.

Active SLAM에서 가장 직접적으로 쓰이는 개념은 **Mutual information(상호정보량)**이다. 다른 변수 $Y$를 관측했을 때 내가 알고 싶은 변수 $X$의 불확실성이 얼마나 줄어드는지를 잰다.

```
I(X; Y) = H(X) - H(X|Y)
```

$H(X)$가 관측 전의 불확실성이고 $H(X|Y)$가 관측 후 남은 불확실성이므로, 그 차이인 $I(X; Y)$는 곧 관측 $Y$로부터 새로 얻는 $X$의 순수 정보량이다.

이 식을 **Active SLAM**의 행동 결정 문제에 그대로 대입해 보자. 로봇이 취할 수 있는 후보 행동 $a$마다 앞으로 얻게 될 예상 관측을 $Z_a$, 지도와 로봇 상태를 $X$라고 하면, 기대 정보 획득량(expected information gain)을 최대화하는 최적 행동 $a^*$는 다음과 같이 고른다.

```
a* = argmax_a  I(X; Z_a)  =  argmax_a  [ H(Z_a) - H(Z_a | X) ]
```

즉 아직 가 보지 않아 지도가 불확실한 구역을 비추거나, 이전에 지나온 랜드마크를 다시 확인해 루프 클로저로 전체 궤적의 분산을 확 줄일 수 있다면, 그런 경로가 자연히 높은 점수를 받는다.

> **추천 자료**
> - [Elements of Information Theory (Cover & Thomas)](https://onlinelibrary.wiley.com/doi/book/10.1002/047174882X) — 정보 이론 교과서
> - Placed et al., "A Survey on Active Simultaneous Localization and Mapping: State of the Art and New Frontiers" (IEEE T-RO 2023) — Active SLAM에서 정보 이론이 쓰이는 방식을 정리한 서베이

> **기술 흐름: 로보틱스 수학 및 최적화**
> - **~2005**: 칼만 필터(EKF) 중심의 상태 추정. 선형 근사 기반이라 소규모 문제에는 알맞았으나, 랜드마크가 늘어나면 공분산 갱신 비용이 커져 대규모 실시간 SLAM에 한계가 있었다.
> - **2006~2015**: Factor Graph 기반 최적화(iSAM, g2o, GTSAM) 등장. 희소 행렬 구조를 활용해 대규모 SLAM 문제를 효율적으로 풀면서, Lie Group/Algebra가 커뮤니티의 표준 도구로 자리 잡았다.
> - **2016~2019**: VIO와 LiDAR odometry에서 sliding-window 최적화와 증분 최적화가 실시간 시스템의 기본 골격이 되었고, Ceres Solver 같은 공개 비선형 최소자승 라이브러리가 연구와 산업 현장에 널리 퍼졌다.
> - **2020~**: Differentiable Programming 시대. PyTorch/JAX의 자동 미분을 활용한 End-to-End 최적화의 길이 열렸고, NeRF(ECCV 2020)·3D Gaussian Splatting(SIGGRAPH 2023) 등 미분 가능 렌더링이 등장하면서 손으로 유도하던 Jacobian을 자동 미분이 대신하게 되었다. Theseus(Meta) 같은 미분 가능 최적화 라이브러리도 함께 발전했다.

---

## 3.9 베이즈 필터 [심화]

원전: Thrun, Burgard, Fox (2005) *Probabilistic Robotics*, Ch.2 (Recursive State Estimation).
(국문: 『확률론적 로보틱스』, 에이콘, 2020)

앞서 3.6절에서 본 Factor Graph는 지나온 모든 포즈와 랜드마크를 통째로 쥐고 최적화하는 방식(smoothing)이다. 반면 이제 다룰 **베이즈 필터(Bayes Filter)**는 재귀적 추정 틀(filtering)로서, 과거 상태들을 확률적으로 적분해 지워 버리고(marginalization) "지금 이 순간 로봇이 어디에 있는가"에 대한 현재 믿음(belief)만 실시간으로 갱신해 나간다. §3.3.2에서 배운 베이즈 정리를 시간 축 위에서 한 걸음씩 굴리면 바로 이 필터가 탄생한다.

### 3.9.1 상태와 Markov 가정

먼저 필터가 추적하려는 대상인 **상태(state)** $x_t$를 정의하자. 상태란 시각 $t$에 로봇과 주변 환경이 놓인 상황을 요약한 변수 묶음이다. 만약 현재 상태 $x_t$ 하나만 알면 그보다 앞선 과거의 경로($x_{0:t-1}$), 과거의 센서 측정($z_{1:t}$), 과거의 제어 명령($u_{1:t}$)을 더 들춰보지 않아도 미래 $x_{t+1}$을 예측하기에 충분하다면, 이 상태를 **완전 상태(complete state)**라 부른다. 미래가 오직 현재에만 의존한다는 이 성질이 곧 **Markov 성질**이다.

$$p(x_{t+1} \mid x_t,\, x_{0:t-1},\, z_{1:t},\, u_{1:t+1}) = p(x_{t+1} \mid x_t,\, u_{t+1})$$

즉, 현재 상태 $x_t$가 과거 이력 전체의 충분 통계량(sufficient statistic) 역할을 한다는 뜻이다.

실제 문제에서 다루는 상태 변수는 성격에 따라 몇 가지로 나뉜다. 로봇의 위치나 속도처럼 매 순간 바뀌는 **동적 상태**와 벽·랜드마크 위치처럼 가만히 있는 **정적 상태**가 있다. 값의 형태에 따라서는 연속적인 포즈 같은 연속 상태, 문 열림·닫힘이나 센서 고장 여부 같은 이산 상태, 그리고 둘이 섞인 하이브리드 상태로 구분한다. 물론 현실에서 우리가 필터에 넣는 상태 벡터는 이상적인 완전 상태의 근사치에 가깝다. 모델링하지 못한 마찰이나 기구학적 오차, 이산화 근사 오차가 끼어들면 실제 시스템은 엄밀한 Markov 가정에서 조금씩 벗어나게 된다.

### 3.9.2 환경 상호작용: 측정과 제어

시간이 흐르며 로봇과 환경이 주고받는 데이터는 두 가지 흐름으로 정리된다.

- **측정 데이터 $z_t$**: 환경이 로봇에게 건네주는 감각 정보다(LiDAR 거리 스캔, 카메라 이미지 등). 시간 구간 $(t-1, t]$에 들어온 측정은 환경에 관한 로봇의 지식을 늘려 불확실성을 줄여 준다.
- **제어 데이터 $u_t$**: 로봇이 환경에 가하는 움직임 정보다(바퀴 속도 명령, 조향 입력 등). 실제 움직임에는 바퀴 미끄러짐 같은 운동 잡음이 따르므로, 제어에 따라 상태를 예측할 때는 대개 불확실성이 커진다.

시간 구간 $t_1$부터 $t_2$까지 쌓인 시퀀스는 다음처럼 표기한다.

$$z_{t_1:t_2} = z_{t_1},\, z_{t_1+1},\, \ldots,\, z_{t_2} \qquad u_{t_1:t_2} = u_{t_1},\, \ldots,\, u_{t_2}$$

한 가지 재미있는 관례가 있다. **휠 오도메트리(odometry)는 물리적으로는 바퀴 엔코더라는 센서에서 읽지만, 확률 모델에서는 제어 데이터 $u_t$로 취급한다.** 환경의 모습을 잰 것이 아니라 "로봇이 직전 상태에서 얼마나 이동했는가"라는 상태 변화량을 담고 있기 때문이다. 또한 로봇이 가만히 서 있더라도 주변 환경이나 동역학 탓에 상태는 변할 수 있으므로, 원전 *Probabilistic Robotics*(이하 PR)에서는 아무것도 하지 않는 '정지(`do_nothing`)' 역시 수식의 일관성을 유지하기 위해 하나의 제어 입력으로 둔다.

### 3.9.3 Belief의 정의

로봇은 세상의 진짜(true) 상태 $x_t$를 직접 들여다볼 수 없다. 대신 지금까지 얻은 모든 측정 $z_{1:t}$와 제어 $u_{1:t}$를 종합해 계산한 사후 확률 분포만을 내부에 품고 살아가는데, 이를 **belief**라 부른다.

$$\text{bel}(x_t) = p(x_t \mid z_{1:t},\, u_{1:t})$$

한 사이클을 두 단계로 나누어 생각하기 위해, 시각 $t$의 제어 $u_t$까지 반영한 뒤 새 측정 $z_t$를 받아들이기 직전의 분포를 **예측 belief**라 하고 위쪽 막대($\overline{\text{bel}}$)로 구분해 적는다.

$$\overline{\text{bel}}(x_t) = p(x_t \mid z_{1:t-1},\, u_{1:t})$$

여기에 방금 들어온 측정 $z_t$를 결합해 $\overline{\text{bel}}(x_t)$에서 $\text{bel}(x_t)$로 넘어가는 과정을 보정(correction) 또는 **measurement update**라 부른다. PR §2.3.4에서 짚고 있듯, 설령 GPS 수신기를 달아도 그 출력은 오차가 섞인 측정값일 뿐 참 포즈 그 자체가 아니다. 따라서 로봇의 belief는 언제나 측정을 바탕으로 추론한 확률 분포로 남는다.

### 3.9.4 생성 법칙: 모션 모델과 측정 모델

3.9.1절의 완전 상태(Markov) 가정을 적용하면, 시간에 따라 상태가 어떻게 움직이고 센서가 어떻게 반응하는지를 설명하는 두 가지 조건부 독립 법칙이 깔끔하게 떨어진다.

$$p(x_t \mid x_{0:t-1},\, z_{1:t-1},\, u_{1:t}) = p(x_t \mid x_{t-1},\, u_t) \quad \text{(모션 모델)}$$

$$p(z_t \mid x_{0:t},\, z_{1:t-1},\, u_{1:t}) = p(z_t \mid x_t) \quad \text{(측정 모델)}$$

$x_{t-1}$이 과거 모든 이력을 대변하므로 다음 상태는 오직 직전 상태 $x_{t-1}$과 직전 제어 $u_t$에만 의존하고, 현재 측정 $z_t$ 역시 오직 현재 상태 $x_t$에만 의존한다. 모델이 시간에 따라 변하지 않는다고(time-invariant) 가정하면 첨자를 떼고 $p(x' \mid u, x)$와 $p(z \mid x)$로 간단히 적기도 한다.

결국 모션 모델, 측정 모델, 그리고 초기 상태 분포 $p(x_0)$ 세 가지가 모이면 로봇과 환경의 **완전 생성 모델(generative model)**이 완성된다. 확률 그래프 모델의 눈으로 보면 이는 은닉 마르코프 모델(HMM) 또는 동적 베이즈 네트워크(Dynamic Bayes Network)와 같은 구조다.

### 3.9.5 베이즈 필터 일반형

이제 동적 상태 추정을 재귀적으로 푸는 **베이즈 필터(Bayes Filter)**의 일반식을 만날 차례다. 한 타임스텝이 지날 때마다 모션 모델을 적분해 상태를 앞으로 미는 **prediction** 단계와, 측정 모델을 곱해 믿음을 조이는 **correction** 단계가 번갈아 돌아간다.

$$\overline{\text{bel}}(x_t) = \int p(x_t \mid u_t,\, x_{t-1})\, \text{bel}(x_{t-1})\, dx_{t-1} \tag{prediction}$$

$$\text{bel}(x_t) = \eta\, p(z_t \mid x_t)\, \overline{\text{bel}}(x_t) \tag{correction}$$

여기서 $\eta$는 전체 확률의 합(적분)이 1이 되도록 맞춰 주는 정규화 상수이며, §3.3.2 베이즈 정리 분모 $P(B)$의 역수 역할을 한다. 이를 알고리즘 의사코드로 옮기면 다음과 같다.

```
# Algorithm Bayes_filter (Table 2.1 의역)
# 입력: bel(x_{t-1}), u_t, z_t
# 출력: bel(x_t)

for all x_t do
    # prediction: 모션 모델로 x_{t-1} 적분
    bel_bar(x_t) = ∫ p(x_t | u_t, x_{t-1}) · bel(x_{t-1}) dx_{t-1}

    # correction: 측정 모델로 가중치 부여 후 정규화
    bel(x_t) = η · p(z_t | x_t) · bel_bar(x_t)
endfor
return bel(x_t)
```

상태 공간이 이산적이라면 prediction의 적분 기호는 단순한 합(summation)으로 바뀐다. 또한 재귀를 시작하려면 $t=0$ 시점의 초기 분포 $\text{bel}(x_0)$가 있어야 하는데, 출발 위치를 정확히 안다면 그 위치에만 확률이 모인 점질량(point mass) 분포로, 전혀 모른다면 전 공간에 걸친 균등 분포로 초기화한다.

다만 이 일반형 식을 글자 그대로 코드에 옮길 수 있는 경우는 상태 공간이 손에 꼽을 만큼 작거나 적분이 닫힌 형식으로 풀릴 때뿐이다. 그래서 실전에서는 §3.10의 칼만 필터 계열처럼 분포를 정규분포로 제한해 적분을 대수식으로 풀거나, §3.11의 입자 필터처럼 무작위 표본으로 적분을 근사하는 길을 택한다.

### 3.9.6 문 추정 워킹 예제

추상적인 수식이 실제로 어떻게 작동하는지 감을 잡기 위해, '열림(`open`)'과 '닫힘(`closed`)' 두 상태만 있는 문 앞의 로봇 예제를 손으로 따라가 보자. 상태가 2개뿐이므로 적분은 두 항의 덧셈으로 줄어든다.

**모델 설정:**

- 측정 모델: 문이 실제로 열려 있을 때 센서가 열림을 감지할 확률 $p(\text{sense\_open} \mid \text{is\_open}) = 0.6$, 닫혀 있는데도 열렸다고 오인할 확률 $p(\text{sense\_open} \mid \text{is\_closed}) = 0.2$
- 모션 모델 `push`: 문이 이미 열려 있으면 그대로 열려 있고(확률 1), 닫혀 있을 때 밀면 확률 0.8로 열린다(확률 0.2로 안 열림)
- 모션 모델 `do_nothing`: 아무것도 하지 않으므로 상태가 변하지 않는 결정론적 항등 변환
- 초기 belief: 문이 열렸는지 닫혔는지 전혀 모르므로 $\text{bel}(X_0 = \text{open}) = \text{bel}(X_0 = \text{closed}) = 0.5$

**단계 1: $u_1$ = `do_nothing` (제어 적용)**

가만히 있었으니 상태 전이가 일어나지 않아 예측 belief도 초기 belief와 같다: $\overline{\text{bel}}(X_1) = (0.5,\; 0.5)$.

**단계 2: $z_1$ = `sense_open` (측정 반영)**

이제 센서가 "문이 열려 있다"고 보고했다. 예측 belief에 측정 우도를 각각 곱해 보자.

$$\overline{\text{bel}}(X_1 = \text{open}) = 0.5, \quad p(\text{sense\_open} \mid \text{open}) = 0.6$$
$$\overline{\text{bel}}(X_1 = \text{closed}) = 0.5, \quad p(\text{sense\_open} \mid \text{closed}) = 0.2$$

두 값을 곱한 비정규화 사후 확률은 $(0.6 \times 0.5,\; 0.2 \times 0.5) = (0.30,\; 0.10)$이고, 합이 1이 되도록 정규화 상수 $\eta = 1/(0.30 + 0.10) = 2.5$를 곱하면 첫 번째 보정 결과가 나온다.

$$\text{bel}(X_1 = \text{open}) = 0.75, \quad \text{bel}(X_1 = \text{closed}) = 0.25$$

**단계 3: $u_2$ = `push` (제어 적용)**

이제 로봇이 문을 밀었다. prediction 단계에서 문이 열려 있을 확률은 "원래 열려 있던 경우 + 닫혀 있었는데 밀려서 열린 경우"를 모두 합산해 구한다.

$$\overline{\text{bel}}(X_2 = \text{open}) = 1 \cdot 0.75 + 0.8 \cdot 0.25 = 0.95$$
$$\overline{\text{bel}}(X_2 = \text{closed}) = 0 \cdot 0.75 + 0.2 \cdot 0.25 = 0.05$$

**단계 4: $z_2$ = `sense_open` (측정 반영)**

밀고 난 뒤 다시 센서를 읽었더니 이번에도 `sense_open`이 나왔다. 한 번 더 측정 우도를 곱하면 비정규화 값은 $(0.6 \times 0.95,\; 0.2 \times 0.05) = (0.570,\; 0.010)$이 되고, $\eta = 1/0.580 \approx 1.724$로 정규화하면 믿음은 확신에 가까워진다.

$$\text{bel}(X_2 = \text{open}) \approx 0.983, \quad \text{bel}(X_2 = \text{closed}) \approx 0.017$$

지금까지의 흐름을 표로 한눈에 정리하면 다음과 같다.

| 단계 | $\text{bel(open)}$ | $\text{bel(closed)}$ |
|------|:-----------------:|:-------------------:|
| 초기 | 0.500 | 0.500 |
| $z_1$ 반영 후 | 0.750 | 0.250 |
| $u_2$ 적용 후 | 0.950 | 0.050 |
| $z_2$ 반영 후 | 0.983 | 0.017 |

센서가 완벽하지 않고(정탐률 60%, 오탐률 20%) 문을 미는 행동조차 실패할 수 있는 비결정적 상황에서도, 측정과 제어가 몇 번 오가는 사이에 belief가 올바른 가설로 빠르게 수렴한다. 물론 $0.983$이라는 숫자를 보고 로봇이 문을 통과해도 안전할지 결정할 때, 적용할 문턱값은 충돌 비용 등 응용 분야의 위험 기준에 따라 별도로 정한다.

### 3.9.7 수학적 유도

마지막으로 3.9.5절의 두 갱신식이 베이즈 규칙, 전확률 정리, 그리고 Markov(완전 상태) 가정만으로 빈틈없이 유도된다는 점을 확인해 두자.

**Correction step 유도:**

먼저 $\text{bel}(x_t) = p(x_t \mid z_{1:t},\, u_{1:t})$에서 가장 최근 측정 $z_t$를 조건부 밖으로 끄집어내기 위해 베이즈 규칙을 적용한다.

$$p(x_t \mid z_{1:t},\, u_{1:t}) = \eta\, p(z_t \mid x_t,\, z_{1:t-1},\, u_{1:t})\, p(x_t \mid z_{1:t-1},\, u_{1:t})$$

여기서 완전 상태 가정에 따라 $x_t$만 주어지면 과거 측정과 제어가 아무런 추가 정보를 주지 못하므로, 첫 번째 우도 항은 $p(z_t \mid x_t,\, z_{1:t-1},\, u_{1:t}) = p(z_t \mid x_t)$로 단순화된다. 두 번째 항은 정의 그대로 $\overline{\text{bel}}(x_t)$이므로 곧바로 correction 식을 얻는다.

$$\text{bel}(x_t) = \eta\, p(z_t \mid x_t)\, \overline{\text{bel}}(x_t)$$

**Prediction step 유도:**

이제 $\overline{\text{bel}}(x_t) = p(x_t \mid z_{1:t-1},\, u_{1:t})$를 직전 상태 $x_{t-1}$에 대한 적분으로 풀기 위해 전확률 정리를 적용한다.

$$\overline{\text{bel}}(x_t) = \int p(x_t \mid x_{t-1},\, z_{1:t-1},\, u_{1:t})\, p(x_{t-1} \mid z_{1:t-1},\, u_{1:t})\, dx_{t-1}$$

피적분 함수의 첫 번째 항은 완전 상태 가정에 따라 직전 상태와 최신 제어에만 의존하므로 $p(x_t \mid x_{t-1},\, u_t)$로 줄어든다. 두 번째 항에서는 아직 발생하지 않은 제어 $u_t$가 과거 상태 $x_{t-1}$에 영향을 줄 수 없으므로 $u_t$를 지울 수 있고, 남은 항은 정확히 $\text{bel}(x_{t-1})$이 된다.

$$\overline{\text{bel}}(x_t) = \int p(x_t \mid u_t,\, x_{t-1})\, \text{bel}(x_{t-1})\, dx_{t-1}$$

이 유도 과정 전체가 Markov 가정 위에 서 있다는 점을 기억해 두자. 만약 상태 벡터에 빠진 중요 변수(예: 바퀴의 지속적 편향이나 미모델링 동역학)가 있어 Markov 가정이 무너지면, 유도된 필터 식 자체가 편향을 낳기 시작한다.

> **추천 자료**
> - [Thrun, Burgard, Fox — Probabilistic Robotics (2005)](http://probabilistic-robotics.org/) — Ch.2 전체가 이 절의 원전. 알고리즘, 예제, 유도가 모두 실려 있다.
> - [Cyrill Stachniss — Bayes Filter Lecture (YouTube)](https://www.youtube.com/watch?v=0lKHFJpaZvE) — Cyrill Stachniss의 베이즈 필터 강의(2020)

---

## 3.10 가우시안 필터 계열 [심화]

원전: Thrun, Burgard, Fox (2005) *Probabilistic Robotics*, Ch.3 (Gaussian Filters).
(국문: 『확률론적 로보틱스』, 에이콘, 2020)

앞서 본 §3.9 베이즈 필터의 일반 적분식은 연속 공간에서 대개 닫힌 형식으로 풀리지 않는다. 이 적분을 컴퓨터가 다룰 수 있는 깔끔한 행렬 연산으로 바꾸는 가장 성공적인 해법이 바로 **가우시안 필터(Gaussian Filter)**다. 가우시안 필터는 belief의 모양을 언제나 다변량 정규분포 $\mathcal{N}(\mu_t, \Sigma_t)$ 하나로 제한해 무한 차원의 확률 밀도 함수 적분을 유한 차원의 평균·공분산 갱신으로 축소한다. 칼만 필터(KF), 확장 칼만 필터(EKF), 정보 필터(IF)는 모두 이 계열에 속하며, 똑같이 §3.9의 prediction–correction 뼈대를 따른다.

### 3.10.1 칼만 필터 (Kalman Filter)

#### 선형 가우시안 시스템 가정

**칼만 필터(KF)**가 어떤 근사도 없이 *정확한* 베이즈 필터가 되려면, prediction과 correction을 몇 번이고 돌려도 belief가 가우시안 모양에서 벗어나지 않아야 한다. 이를 보장하는 세 가지 조건이 바로 선형 가우시안 시스템 가정이다.

**상태 천이(모션 모델):**
$$x_t = A_t x_{t-1} + B_t u_t + \varepsilon_t, \quad \varepsilon_t \sim \mathcal{N}(0, R_t)$$

여기서 $A_t$는 직전 상태를 다음 상태로 변환하는 $n \times n$ 상태 천이 행렬, $B_t$는 제어 입력을 상태 변화로 바꾸는 $n \times m$ 행렬, $R_t$는 움직임에 섞이는 프로세스 노이즈 공분산이다.

**측정 모델:**
$$z_t = C_t x_t + \delta_t, \quad \delta_t \sim \mathcal{N}(0, Q_t)$$

$C_t$는 $n$차원 상태를 $k$차원 센서 관측 공간으로 사상하는 $k \times n$ 측정 행렬이고, $Q_t$는 센서 자체의 측정 노이즈 공분산이다.

**초기 belief:**
$$\text{bel}(x_0) = \mathcal{N}(\mu_0, \Sigma_0)$$

선형 변환을 거친 가우시안은 여전히 가우시안이고, 두 가우시안의 곱 역시 정규화하면 다시 가우시안이 된다. 따라서 위 세 가정이 성립하면 모션 모델과 측정 모델이 모두 정규분포 꼴로 적히며, 모든 시점 $t$에서 belief는 완벽한 정규분포로 유지된다.

$$p(x_t \mid u_t, x_{t-1}) = \mathcal{N}(x_t;\; A_t x_{t-1} + B_t u_t,\; R_t)$$
$$p(z_t \mid x_t) = \mathcal{N}(z_t;\; C_t x_t,\; Q_t)$$

#### 칼만 필터 알고리즘

가우시안 분포는 평균 $\mu_t$와 공분산 $\Sigma_t$ 두 값만으로 완전히 결정된다. 따라서 칼만 필터는 §3.9.5의 복잡한 적분 알고리즘을 단 다섯 줄의 행렬 연산으로 압축한다. 앞의 두 줄이 prediction, 뒤의 세 줄이 correction이다.

```
# Algorithm Kalman_filter (Table 3.1 의역)
# 입력: μ_{t-1}, Σ_{t-1}, u_t, z_t
# 출력: μ_t, Σ_t

# --- prediction ---
1: μ̄_t = A_t μ_{t-1} + B_t u_t          # 상태 예측: 모션 모델 적용
2: Σ̄_t = A_t Σ_{t-1} A_t^T + R_t        # 공분산 예측: 불확실성 증가

# --- correction ---
3: K_t  = Σ̄_t C_t^T (C_t Σ̄_t C_t^T + Q_t)^{-1}   # 칼만 이득
4: μ_t  = μ̄_t + K_t (z_t - C_t μ̄_t)    # 혁신(innovation)으로 평균 보정
5: Σ_t  = (I - K_t C_t) Σ̄_t             # 공분산 감소

return μ_t, Σ_t
```

라인 1~2의 prediction 단계에서는 제어 입력 $u_t$를 반영해 평균을 옮기는 동시에 프로세스 노이즈 $R_t$가 더해지며 공분산(불확실성)이 커진다. 반대로 라인 3~5의 measurement update 단계에서는 새 관측 $z_t$가 들어오면서 공분산이 줄어든다. 이때 핵심 역할을 하는 두 요소를 눈여겨보자.

- **칼만 이득(Kalman Gain) $K_t$**: 예측 믿음과 센서 측정 중 어느 쪽에 얼마나 무게를 실을지 결정하는 가중치다. 센서 노이즈 $Q_t$가 매우 크면 $K_t$가 작아져 측정을 조심스럽게 조금만 반영하고, 반대로 예측 불확실성 $\bar\Sigma_t$가 커질수록 $K_t$가 커져 센서의 말을 더 신뢰하게 된다.
- **혁신(Innovation) $z_t - C_t \bar\mu_t$**: 예측 상태로부터 기대한 센서 값과 실제 들어온 측정값의 차이다. 이 차이가 0이라면 예측이 완벽히 맞았으므로 평균은 움직이지 않는다. 반면 라인 5의 공분산 갱신에는 실제 측정값 $z_t$가 전혀 들어가지 않는다는 점도 흥미롭다(즉 "어느 센서가 얼마나 정확하게 재는지"만 알면 불확실성이 줄어드는 비율은 측정값 자체와 무관하게 계산된다).

#### 1D KF 도해: 정보가 어떻게 결합되는가

수식이 1차원 직선 위 로봇의 위치 추정에서 실제로 어떤 그림을 그리는지 분포의 모양으로 따라가 보자.

- **Prior $\text{bel}(x_{t-1})$**: 이전 단계의 추정이 뾰족하고 좁은 가우시안 종 모양으로 서 있다.
- **Prediction 후**: 모션 명령을 따라 종이 이동하고, 운동 잡음 $R_t$ 때문에 공분산이 커지며 종이 옆으로 납작하게 퍼진다($\bar\Sigma_t = A_t \Sigma_{t-1} A_t^T + R_t$).
- **측정 $z_t$**: 센서 역시 자기 정밀도 $Q_t$에 맞는 폭을 가진 또 하나의 가우시안 곡선으로 위치를 가리킨다.
- **Correction 후**: 예측 가우시안과 측정 가우시안을 곱하면, 두 종이 겹치는 가운데 봉우리가 솟아오르며 분산은 *원래 두 분포 어느 쪽보다도 좁아진다*. 새로운 평균은 두 평균을 각 분산의 역수(정밀도)로 가중 평균한 지점에 자리 잡는다.
- 이후 다시 이동(분산 증가)과 측정(분산 감소)이 쉼 없이 교차한다.

이처럼 상태 추정은 불확실성이 퍼지는 prediction과 정보가 모여들며 불확실성이 좁혀지는 correction의 리듬을 탄다. 이 직관은 뒤이어 볼 §3.10.2 EKF와 §3.11.3 입자 필터, 그리고 14장 §14.7.3의 EKF localization에도 똑같이 적용된다.

<!-- DEMO: kalman_1d_illustration.html -->

#### KF의 수학적 유도 (핵심)

칼만 필터의 다섯 줄은 어림짐작으로 만든 공식이 아니라 §3.9.5 베이즈 필터의 두 식(적분식과 곱셈식)을 선형 가우시안 조건에서 정확히 풀어낸 대수적 결과다.

**Part 1 (Prediction).** 베이즈 필터의 prediction 적분 안에 두 정규분포를 넣고 지수부 $L_t$를 펼치면, $x_{t-1}$과 $x_t$에 관한 이차식(quadratic)이 나타난다. 여기서 완전제곱식을 이용해 $L_t$를 $x_{t-1}$에 의존하는 이차식 부분과 $x_t$에만 의존하는 부분으로 분해하면, $x_{t-1}$에 대한 적분은 상태 $x_t$와 무관한 상수가 되어 정규화 상수 $\eta$ 안으로 흡수된다. 살아남은 $x_t$ 이차식의 1차·2차 계수를 정리하면 곧바로 $\bar\mu_t = A_t \mu_{t-1} + B_t u_t$와 $\bar\Sigma_t = A_t \Sigma_{t-1} A_t^T + R_t$가 튀어나온다.

**Part 2 (Measurement update).** Correction 식 $\text{bel}(x_t) \propto \exp\{-J_t\}$에서도 지수부 $J_t$를 1차·2차 미분해 정리하면 역공분산(정밀도 행렬) $\Sigma_t^{-1} = C_t^T Q_t^{-1} C_t + \bar\Sigma_t^{-1}$을 얻는다. 이 식을 그대로 뒤집어 $\Sigma_t$를 구하려면 $n \times n$ 상태 차원 행렬의 역행렬을 계산해야 해서 상태 변수가 많을 때 부담이 크다. 이때 **행렬 역변환 보조정리(Matrix Inversion Lemma, Woodbury identity)**를 적용하면 다음처럼 칼만 이득을 얻는다.

$$K_t = \bar\Sigma_t C_t^T (C_t \bar\Sigma_t C_t^T + Q_t)^{-1}$$

덕분에 $n \times n$ 행렬 대신 측정 차원 크기인 $k \times k$ 행렬의 역행렬만 구하면 된다. 보통 상태 차원 $n$보다 측정 차원 $k$가 훨씬 작으므로($k \ll n$) 계산 비용이 극적으로 줄어든다.

**복잡도:** 한 사이클당 $O(k^{2.8} + n^2)$ ($k$: 측정 차원, $n$: 상태 차원).

참고로 §3.10.2의 EKF도 선형화만 추가될 뿐 똑같은 완전제곱식 분해와 Woodbury 항등식으로 유도된다. 또한 $\Sigma_t^{-1}$에 $C_t^T Q_t^{-1} C_t$ 항이 더해지는 대수적 구조는 §3.6 Factor Graph의 Gauss-Newton 정규방정식에 측정 팩터 하나가 추가되는 꼴과 본질적으로 같다.

### 3.10.2 확장 칼만 필터 (Extended Kalman Filter)

#### 비선형 시스템으로 확장

현실의 로봇은 선형 방정식처럼 얌전하게 움직이지 않는다. 바퀴 달린 로봇이 각도 $\theta$를 틀며 전진할 때는 $\cos\theta, \sin\theta$가 들어가고, LiDAR나 카메라는 직교 좌표를 거리·방위각이나 픽셀 좌표로 투영하므로 모션 모델 $g$와 측정 모델 $h$ 모두 비선형 함수가 된다.

$$x_t = g(u_t, x_{t-1}) + \varepsilon_t, \quad \varepsilon_t \sim \mathcal{N}(0, R_t)$$
$$z_t = h(x_t) + \delta_t, \quad \delta_t \sim \mathcal{N}(0, Q_t)$$

문제는 아무리 깔끔한 가우시안 분포라도 비선형 함수 $g$를 한 번 통과하고 나면 더 이상 가우시안이 아니라는 데 있다. **확장 칼만 필터(EKF)**는 이 장벽을 1차 테일러 전개(국소 선형화)로 정면 돌파한다. 모션 모델 $g$는 직전 사후 평균 $\mu_{t-1}$ 주변에서, 측정 모델 $h$는 예측 평균 $\bar\mu_t$ 주변에서 접평면으로 근사해 가우시안 형태를 억지로 유지하는 전략이다.

$$g(u_t, x_{t-1}) \approx g(u_t, \mu_{t-1}) + G_t (x_{t-1} - \mu_{t-1})$$

$$G_t := \frac{\partial g(u_t, x_{t-1})}{\partial x_{t-1}}\bigg|_{\mu_{t-1}} \quad (n \times n \text{ Jacobian})$$

$$h(x_t) \approx h(\bar\mu_t) + H_t (x_t - \bar\mu_t)$$

$$H_t := \frac{\partial h(x_t)}{\partial x_t}\bigg|_{\bar\mu_t} \quad (k \times n \text{ Jacobian})$$

이 1차 접평면 근사가 얼마나 정확한지는 두 가지에 달렸다. 함수 $g, h$가 심하게 휘어 있을수록, 그리고 현재 belief의 분산(불확실성 구름의 폭)이 클수록 접평면과 실제 곡면 사이의 간극이 벌어진다. 따라서 EKF는 불확실성이 비교적 작은 영역을 추적할 때 가장 믿음직하게 작동한다.

#### EKF 알고리즘

KF의 다섯 줄에서 선형 행렬 곱 부분을 비선형 함수 평가로 바꾸고, 공분산과 칼만 이득 식의 $A_t, C_t$ 자리에 위에서 구한 Jacobian $G_t, H_t$를 끼워 넣기만 하면 EKF가 완성된다.

```
# Algorithm Extended_Kalman_filter (Table 3.3 의역)
# 입력: μ_{t-1}, Σ_{t-1}, u_t, z_t
# 출력: μ_t, Σ_t

# --- prediction ---
1: μ̄_t = g(u_t, μ_{t-1})                         # 비선형 모션 모델
2: Σ̄_t = G_t Σ_{t-1} G_t^T + R_t                  # Jacobian으로 선형화된 공분산 전파

# --- correction ---
3: K_t  = Σ̄_t H_t^T (H_t Σ̄_t H_t^T + Q_t)^{-1}  # 칼만 이득 (H_t로 치환)
4: μ_t  = μ̄_t + K_t (z_t - h(μ̄_t))               # 비선형 측정 예측 사용
5: Σ_t  = (I - K_t H_t) Σ̄_t

return μ_t, Σ_t
```

KF와 EKF 사이에서 평균을 구하는 식이 바뀐 곳은 딱 두 줄(라인 1의 $A_t \mu_{t-1} + B_t u_t \to g(u_t, \mu_{t-1})$, 라인 4의 $C_t \bar\mu_t \to h(\bar\mu_t)$)뿐이며, 공분산 전파와 이득 계산에서는 $A_t \to G_t$, $C_t \to H_t$ 치환만 일어난다.

#### 유도 요약 및 실무 비교

유도 과정 역시 §3.10.1 KF와 완전히 나란히 간다. 비선형 $g$와 $h$를 1차 테일러 근사로 바꾼 뒤 똑같이 완전제곱식 분해와 Woodbury 항등식을 통과시키면 정확히 위 EKF 업데이트 식이 나온다.

$$\bar\mu_t = g(u_t, \mu_{t-1}), \quad \bar\Sigma_t = G_t \Sigma_{t-1} G_t^T + R_t$$
$$\mu_t = \bar\mu_t + K_t (z_t - h(\bar\mu_t)), \quad \Sigma_t = (I - K_t H_t) \bar\Sigma_t, \quad K_t = \bar\Sigma_t H_t^T (H_t \bar\Sigma_t H_t^T + Q_t)^{-1}$$

**실무 비교:**

EKF는 SLAM과 IMU 융합의 오랜 주역이며, 지금도 연산 자원이 빠듯한 드론이나 모바일 기기의 MSCKF 계열 VIO에서 현역으로 뛴다. 비선형성이 강할 때 EKF의 1차 선형화 한계를 넘기 위해 다음과 같은 변형들도 현장에서 널리 쓰인다.

- **UKF (Unscented KF):** 평균 주위에 대표적인 표본 점(sigma point) 몇 개를 뽑아 실제 비선형 함수에 그대로 통과시킨 뒤 다시 평균·공분산을 모은다. 덕분에 Jacobian을 손으로 미분할 필요가 없고 비선형 전파 정확도도 더 높다.
- **IEKF (Iterated EKF):** correction 단계에서 한 번에 끝내지 않고, 보정된 평균 지점에서 측정 Jacobian $H_t$를 여러 번 다시 계산하며 선형화 오차를 줄인다.
- **LIEKF (Left-Invariant EKF):** $SO(3)$나 $SE(3)$ 상태의 오차를 3.5절 Lie Group 위의 불변 오차(invariant error)로 정의해 선형화함으로써, 자세 불확실성이 클 때도 일관성을 잃지 않는다.

14장 §14.7.3의 EKF localization과 IEKF·MSCKF 구조는 모두 지금 본 5줄 알고리즘의 직접적인 응용이다. (참고로 §14.10의 IMU preintegration은 필터 대신 Factor Graph 최적화 쪽에서 키프레임 사이의 IMU 측정을 한 팩터로 묶어 쓰는 기법이다.) 각 시스템의 $g, h$와 그 Jacobian $G_t, H_t$가 무엇인지만 파악하면, 14장의 복잡한 센서 융합 코드도 이 뼈대 위에서 한눈에 읽힌다.

### 3.10.3 정보 필터 (Information Filter)

#### Canonical 표현: $(\Omega, \xi)$

지금까지 KF와 EKF는 가우시안을 평균과 공분산 $(\mu, \Sigma)$ 쌍(moments parameterization)으로 들고 다녔다. **정보 필터(Information Filter, IF)**는 이와 정반대의 좌표계(canonical parameterization), 즉 공분산의 역행렬인 **정보 행렬(information matrix)** $\Omega$와 **정보 벡터(information vector)** $\xi$로 똑같은 가우시안 분포를 표현한다.

$$\Omega = \Sigma^{-1}, \quad \xi = \Sigma^{-1} \mu$$

필요할 때는 언제든 역행렬을 취해 원래의 평균과 공분산으로 되돌릴 수 있다($\Sigma = \Omega^{-1}$, $\mu = \Omega^{-1} \xi$).

굳이 역행렬을 취해 놓은 이 좌표계로 가우시안 밀도 함수의 음의 로그 우도를 적어 보면 놀라운 대칭이 드러난다.

$$p(x) = \eta \exp\!\left\{-\tfrac{1}{2} x^T \Omega x + x^T \xi\right\}$$

$$-\log p(x) = \mathrm{const} + \tfrac{1}{2} x^T \Omega x - x^T \xi$$

최솟값을 찾기 위해 미분해 0으로 놓으면 $\Omega x = \xi$, 즉 $x = \Omega^{-1} \xi = \mu$가 된다. 눈썰미가 있다면 이 식이 §3.4.3 Gauss-Newton의 정규 방정식 $H \delta x = -b$ ($H = J^T J$, $b = J^T r$)와 완전히 같은 대수 구조임을 바로 알아챌 것이다. 또한 정보 표현에는 이런 장점도 있다. 정보가 전혀 없는 완전한 무지 상태(균등 분포)를 $(\mu, \Sigma)$ 표현에서는 $\Sigma = \infty$라 수치적으로 다룰 수 없지만, 정보 표현에서는 깔끔하게 $\Omega = 0$으로 적을 수 있다.

#### 정보 필터 알고리즘

표현을 뒤집어 놓으면 계산 부담의 위치도 정반대로 뒤집힌다. KF에서는 prediction이 단순 행렬 곱·덧셈이고 measurement update에 역행렬이 들어갔지만, 정보 필터에서는 오히려 measurement update가 순수 덧셈으로 끝나고 prediction에 역행렬이 들어간다. 이 대칭성 때문에 둘을 서로의 쌍대(dual)라고 부른다.

```
# Algorithm Information_filter (Table 3.4 의역)
# 입력: ξ_{t-1}, Ω_{t-1}, u_t, z_t
# 출력: ξ_t, Ω_t

# --- prediction (두 번의 역행렬 필요) ---
1: Ω̄_t = (A_t Ω_{t-1}^{-1} A_t^T + R_t)^{-1}
2: ξ̄_t = Ω̄_t (A_t Ω_{t-1}^{-1} ξ_{t-1} + B_t u_t)

# --- correction (덧셈) ---
3: Ω_t  = C_t^T Q_t^{-1} C_t + Ω̄_t       # 측정 한 번 = Ω에 한 항 추가
4: ξ_t  = C_t^T Q_t^{-1} z_t + ξ̄_t       # 측정 한 번 = ξ에 한 항 추가

return ξ_t, Ω_t
```

두 필터의 계산 복잡도 쌍대성을 나란히 비교하면 성격 차이가 더욱 또렷해진다.

| 단계 | KF | IF |
|------|:---:|:---:|
| Prediction | $O(n^2)$ 가산형 | $O(n^{2.8})$ 역행렬 2회 |
| Measurement update | $O(k^{2.8})$ 역행렬 필요 | $O(n^2)$ 가산형 |

여기서 $k$는 측정 차원, $n$은 상태 차원이다. 특히 측정 하나가 상태 벡터의 극히 일부(예: 로봇 현재 포즈와 랜드마크 하나)만 건드려 측정 행렬 $C_t$가 희소(sparse)할수록, 정보 필터의 measurement update는 실제로 관련된 작은 블록에만 값을 더하면 되므로 매우 적은 비용으로 끝난다.

#### EIF (Extended Information Filter)

비선형 시스템에서도 EKF와 똑같이 모션과 측정 Jacobian $G_t, H_t$를 도입해 $A_t \to G_t$, $C_t \to H_t$로 갈아 끼우면 **확장 정보 필터(EIF)**를 얻는다(PR Table 3.5).

```
# Algorithm Extended_Information_filter (핵심 변경만)
# prediction  (μ_{t-1} = Ω_{t-1}^{-1} ξ_{t-1})
Ω̄_t = (G_t Ω_{t-1}^{-1} G_t^T + R_t)^{-1}
ξ̄_t = Ω̄_t · g(u_t, μ_{t-1})       # 선형 IF의 A_t μ_{t-1}+B_t u_t → g(u_t,μ_{t-1})

# correction
Ω_t  = H_t^T Q_t^{-1} H_t + Ω̄_t
ξ_t  = H_t^T Q_t^{-1} z_t + ξ̄_t - H_t^T Q_t^{-1} h(μ̄_t) + H_t^T Q_t^{-1} H_t μ̄_t
```

#### 정보형의 중요성: 가산성과 SLAM 연결

정보 필터가 SLAM 이론에서 각별한 대접을 받는 이유는 correction 단계의 **가산성(additivity)** 덕분이다. $\Omega_t = \bar\Omega_t + C_t^T Q_t^{-1} C_t$라는 식은 새 측정이 들어올 때마다 기존 정보 행렬에 항 하나를 얹기만 하면 된다는 뜻이다. 심지어 여러 대의 로봇이 각자 모은 독립적인 관측 정보도 $\Omega_{\text{total}} = \sum_i \Omega_i$, $\xi_{\text{total}} = \sum_i \xi_i$처럼 그저 더하기만 하면 손쉽게 융합된다.

이 가산 구조는 바로 §3.6 Factor Graph에서 "측정 팩터 하나 = $H^T Q^{-1} H$와 $H^T Q^{-1} z$ 항 하나 추가"로 일반화된다. 각 팩터가 소수의 변수에만 매달리는 국소성 덕분에 Jacobian $H$의 열 대부분이 0이 되고, 결과적으로 정보 행렬 $\Omega$는 대부분이 비어 있는 희소 행렬이 된다. 다만 희소 Cholesky 분해로 이 행렬을 실제 풀 때 희소성이 끝까지 유지되는지는 §3.6.3에서 살펴본 소거 순서와 fill-in 관리에 달려 있다. EIF-SLAM과 SEIF(Sparse Extended Information Filter)는 이 정보형의 희소·가산 성질을 정면으로 파고든 대표적인 SLAM 알고리즘이며, 그 역사적 맥락은 14장 §14.16에서 자세히 다룬다.

> **추천 자료**
> - [Thrun, Burgard, Fox — Probabilistic Robotics (2005)](http://probabilistic-robotics.org/) — Ch.3이 이 절의 원전. KF·EKF·IF 세 알고리즘의 유도가 나란히 서술되어 있다.
> - [Cyrill Stachniss 채널 — EKF Localization (Nived Chebrolu)](https://www.youtube.com/watch?v=PiCC-SxWlH8) — EKF localization 강의.
> - [Welch & Bishop — An Introduction to the Kalman Filter (2006)](https://web.archive.org/web/20250111082128/https://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf) — KF의 식과 직관을 함께 정리한 입문 자료.

---

## 3.11 비모수 필터 [심화]

원전: Thrun, Burgard, Fox (2005) *Probabilistic Robotics*, Ch.4 (Nonparametric Filters).
(국문: 『확률론적 로보틱스』, 에이콘, 2020)

§3.10에서 다룬 가우시안 필터들은 아무리 복잡한 belief라도 평균과 공분산 $(\mu, \Sigma)$라는 단 한 쌍의 숫자로 압축한다. 덕분에 빠르고 우아하지만, 동시에 치명적인 제약도 안고 있다. 예를 들어 로봇이 똑같이 생긴 복도 두 군데 중 어디에 있는지 헷갈리면 확률 분포가 두 개의 봉우리로 갈라진다. 이런 **다봉(multi-modal) 상황**이나 비선형성이 너무 강해 분포가 바나나 모양으로 휘는 상황에서는, 단일 가우시안으로 근사하는 순간 큰 오차가 생긴다. 이 한계를 넘기 위해 **비모수 필터(Nonparametric Filter)**는 정해진 분포 모양에 가두는 대신, 공간을 잘게 쪼개거나 수많은 무작위 표본을 뿌려 임의 형태의 분포를 있는 그대로 표현한다. 계산량은 더 들지만, 전역 위치 추정(global localization)이나 로봇 납치(kidnapped robot) 문제처럼 불확실성이 큰 상황에서 빛을 발한다.

### 3.11.1 히스토그램 필터 / 이산 베이즈 필터

#### 유한 상태공간: 적분을 합으로

§3.9.5의 베이즈 필터 적분이 닫힌 형식으로 풀리지 않을 때 가장 먼저 떠올릴 수 있는 직접적인 방법은 상태공간을 유한한 개수로 잘라 적분을 덧셈으로 바꾸는 것이다. 만약 상태가 원래부터 $K$개의 유한한 이산 값 $\{x_1, x_2, \ldots, x_K\}$ 중 하나를 취한다면, §3.9.5의 두 갱신식은 정확한 유한 합으로 바뀐다.

$$\bar p_{k,t} = \sum_i p(x_k \mid u_t, x_i)\, p_{i,t-1} \quad \text{(prediction)}$$
$$p_{k,t} = \eta\, p(z_t \mid x_k)\, \bar p_{k,t} \quad \text{(correction)}$$

이를 의사코드로 옮기면 앞선 3.9.6절의 문 추정 예제와 정확히 같은 루프가 된다.

```
# Algorithm Discrete_Bayes_filter (Table 4.1 의역)
# 입력: {p_{k,t-1}}, u_t, z_t
# 출력: {p_{k,t}}

for all k do
    # prediction: 모든 이전 상태에서 x_k로 가는 전이 합산
    p̄_{k,t} = Σ_i p(X_t = x_k | u_t, X_{t-1} = x_i) · p_{i,t-1}

    # correction: 측정 likelihood로 가중치 부여
    p_{k,t} = η · p(z_t | X_t = x_k) · p̄_{k,t}
endfor
return {p_{k,t}}
```

이 알고리즘은 음성인식의 은닉 마르코프 모델(HMM) 전향 알고리즘(forward algorithm)과 수학적으로 뼈대가 똑같다. 상태공간이 본래 이산적인 문제(문 열림/닫힘, 의미론적 클래스 분류 등)에서는 지금도 가장 깔끔하고 군더더기 없는 선택이다.

#### 연속 상태: 히스토그램 필터

그렇다면 연속적인 위치 공간에는 이 아이디어를 어떻게 옮길까? 연속 공간을 유한 개의 격자 영역 $\{\mathbf{x}_{k,t}\}$로 바둑판처럼 쪼갠 뒤, 각 격자 칸 안에서는 확률 밀도가 균일(piecewise uniform)하다고 가정하면 된다.

$$p(x_t) = \frac{p_{k,t}}{|\mathbf{x}_{k,t}|} \quad x_t \in \mathbf{x}_{k,t}$$

그리고 각 영역을 대표하는 중심값(평균 상태) $\hat x_{k,t}$를 하나 뽑아 측정 모델과 모션 모델을 근사한다.

$$p(z_t \mid \mathbf{x}_{k,t}) \approx p(z_t \mid \hat x_{k,t})$$
$$p(\mathbf{x}_{k,t} \mid u_t, \mathbf{x}_{i,t-1}) \approx \eta\,|\mathbf{x}_{k,t}|\, p(\hat x_{k,t} \mid u_t, \hat x_{i,t-1})$$

모든 격자 칸의 크기 $|\mathbf{x}_{k,t}|$가 똑같다면 이 상수 인수는 정규화 상수 $\eta$에 흡수되어 식이 더 단순해진다. 이렇게 연속 공간을 이산 격자로 쪼개어 이산 베이즈 필터를 돌리는 방식을 **히스토그램 필터(Histogram Filter)**라 부른다.

**한계:** 히스토그램 필터는 격자 해상도를 높일수록 정확해지지만, 차원의 저주에 정면으로 부딪힌다. 예컨대 한 축을 100칸으로 쪼갤 때 2D 평면에서는 $10^4$칸이면 되지만, 3D 위치와 자세를 포함한 6-DoF 공간으로 가면 $100^6 = 10^{12}$칸이 필요해 메모리에 올릴 수조차 없다. 이를 완화하기 위해 동적 분할(density tree)이나 선택적 업데이트(selective updating) 기법이 쓰인다. 동적 분할은 확률이 낮은 텅 빈 영역을 큼직하게 묶고 확률이 모인 곳만 잘게 쪼개며, 선택적 업데이트는 확률이 유의미한 셀만 골라 갱신한다. PR §4.1.3에서는 이러한 selective updating을 적용하면 3차원 이상 공간에서도 격자 분해가 실용적으로 동작한다고 설명한다. 또한 14장 §14.7B의 점유 격자 지도(occupancy grid mapping) 역시 공간을 격자로 나눈 뒤 각 셀마다 §3.11.2의 이진 베이즈 필터를 돌리는 방식으로 구현된다.

### 3.11.2 이진 베이즈 필터 (Binary Bayes Filter, Log-Odds 형식)

#### 정적 상태의 이진 추정

로봇의 위치가 아니라 "지도 위의 이 격자 셀에 벽(장애물)이 있는가 없는가"처럼 시간에 따라 변하지 않는 정적(static) 이진 상태를 추정할 때는 문제가 한결 단순해진다. 상태가 저절로 바뀌지 않으므로 모션 모델을 적분하는 prediction 단계가 통째로 사라지고, 새 센서가 들어올 때마다 correction 단계만 반복하면 되기 때문이다.

그런데 사후 확률 $p(x \mid z_{1:t})$를 그대로 들고 다니며 매번 곱셈으로 갱신하다 보면 금방 골치 아픈 수치 문제가 터진다. 확률이 $0.0001$이나 $0.9999$ 근처까지 몰릴 때 거듭 곱하면 부동소수점 언더플로우가 나기 쉽고, $[0, 1]$ 구간을 벗어나지 않도록 절단 처리도 해 주어야 한다. 확률 대신 **log-odds** 좌표계로 값을 바꾸어 적으면 이 두 문제가 한 번에 사라진다.

$$l(x) := \log \frac{p(x)}{1 - p(x)} \in (-\infty, +\infty)$$

확률이 $(0, 1)$ 사이를 움직일 때 log-odds는 실수 전체 $(-\infty, +\infty)$를 막힘없이 오가므로 0이나 1 부근에서 숫자가 뭉개질 걱정이 없다. 게다가 곱셈이었던 베이즈 갱신이 놀라울 만큼 단순한 **덧셈**으로 바뀐다.

$$l_t = l_{t-1} + \log\frac{p(x \mid z_t)}{1 - p(x \mid z_t)} - l_0$$

여기서 $l_0 = \log\frac{p(x)}{1-p(x)}$는 사전 확률에 해당하는 prior log-odds다. 매번 측정을 반영할 때 직전 믿음에 들어 있던 사전 확률 성분이 중복으로 더해지지 않도록 한 번 빼 주는 것이다. 실제 확률 값으로 복원하고 싶을 때는 시그모이드 형태인 $\text{bel}_t(x) = 1 - \dfrac{1}{1 + \exp(l_t)}$를 계산하기만 하면 된다.

```
# Algorithm Binary_Bayes_filter (Table 4.2 의역)
# 입력: l_{t-1}, z_t
# 출력: l_t
# (정적 상태 가정: prediction 없음)

1: l_t = l_{t-1}
         + log( p(x|z_t) / (1 - p(x|z_t)) )   # inverse sensor model로 측정 반영
         - log( p(x) / (1 - p(x)) )            # prior 차감 (이중 계산 방지)
return l_t
```

#### Inverse sensor model

위 식을 자세히 보면 흔히 쓰던 순방향 측정 모델 $p(z \mid x)$가 아니라, 원인과 결과의 방향을 거꾸로 뒤집은 $p(x \mid z)$가 들어가 있다. 이를 **역센서 모델(inverse sensor model)**이라 부른다. 실전에서는 이 역방향 모델이 훨씬 만들기 편할 때가 많다. 예컨대 PR §4.1.4에서 드는 사례처럼, "문이 열려 있을 때 나올 법한 모든 카메라 픽셀 분포 $p(z \mid \text{open})$"를 수식으로 짜는 것보다 "지금 찍힌 카메라 이미지 $z$를 보고 문이 열려 있을 확률 $p(\text{open} \mid z)$"을 추정하는 편이 압도적으로 쉽다. 이진 베이즈 필터는 이 역센서 모델을 입력으로 받아 곧바로 덧셈으로 갱신한다. 바로 이 log-odds 갱신식이 14장의 Occupancy Grid Mapping에서 지도의 셀마다 적용하는 핵심 공식이다.

### 3.11.3 입자 필터 (Particle Filter)

#### 비모수 표현의 원리

히스토그램 필터는 공간을 빈틈없이 격자로 채우기 때문에 차원이 올라갈수록 아무 일도 안 일어나는 빈 칸까지 껴안고 가야 하는 비효율이 컸다. **입자 필터(Particle Filter)**는 고정된 격자를 버리고, 확률이 높을 법한 영역에 $M$개의 무작위 표본(입자, particle)을 직접 뿌려 분포를 근사함으로써 이 문제를 영리하게 우회한다.

$$\mathcal{X}_t = \{x_t^{[1]},\, x_t^{[2]},\, \ldots,\, x_t^{[M]}\}$$

쉽게 말해 입자 하나하나가 "로봇이 지금 여기에 있을지도 모른다"는 하나의 가설이다. 진짜 사후 확률이 높은 곳에는 입자들이 빽빽하게 모여들고, 가능성이 희박한 허허벌판에는 입자가 거의 놓이지 않는다. 덕분에 가우시안 같은 형태 제약 없이도 다봉 분포나 비대칭적인 긴 꼬리 분포까지 자유롭게 그려 낼 수 있다.

#### 입자 필터 알고리즘: sampling → weighting → resampling

입자 필터의 한 사이클은 직관적인 세 단계(전개 → 평가 → 재추출)로 굴러간다.

```
# Algorithm Particle_filter (Table 4.3 의역)
# 입력: X_{t-1}, u_t, z_t
# 출력: X_t (M개 입자)

X̄_t = X_t = ∅
for m = 1 to M do
    # Step 1: sampling — 모션 모델로 각 입자 전개
    x_t^[m] ~ p(x_t | u_t, x_{t-1}^[m])

    # Step 2: weighting — 측정 likelihood로 중요도 가중치 계산
    w_t^[m] = p(z_t | x_t^[m])

    X̄_t = X̄_t ∪ {x_t^[m], w_t^[m]}
endfor

for m = 1 to M do
    # Step 3: resampling — 가중치에 비례하여 M개 다시 뽑기
    draw i with probability ∝ w_t^[i] from X̄_t
    add x_t^[i] to X_t
endfor
return X_t
```

1단계(**Sampling**)에서는 직전 입자들 각각에 제어 입력 $u_t$와 운동 잡음을 섞어 모션 모델로부터 다음 위치 후보를 뽑아낸다(prediction). 2단계(**Weighting**)에서는 방금 들어온 센서 측정 $z_t$와 각 입자 가설이 얼마나 잘 맞는지에 따라 측정 우도 $w_t^{[m]} = p(z_t \mid x_t^{[m]})$를 매긴다(correction). 마지막 3단계(**Resampling**)에서는 이 가중치가 높은 유망한 입자일수록 더 많이 복제하고 엉뚱한 곳에 떨어진 입자는 도태시켜, 입자의 총개수 $M$을 유지한 채 유력한 가설 주변으로 표본을 집중시킨다. 이론적으로 입자 수 $M \to \infty$가 되면 이 입자 구름은 참 사후 분포 $p(x_t \mid z_{1:t}, u_{1:t})$로 정확히 수렴한다.

#### 중요도 샘플링(Importance Sampling) 직관

왜 2단계 가중치에 딱 측정 우도 $p(z_t \mid x_t^{[m]})$를 넣으면 참 사후 분포가 복원될까? 그 수학적 비밀은 **중요도 샘플링(Importance Sampling)**에 있다. 우리가 원하는 목표 분포(target) $f$에서 표본을 직접 뽑기 어려울 때, 대신 뽑기 쉬운 제안 분포(proposal) $g$에서 표본을 추출한 뒤 각 표본에 비율 $w = f/g$를 가중치로 곱해 주면 적분과 확률 분포를 보정할 수 있다.

$$w^{[m]} = \frac{f(x^{[m]})}{g(x^{[m]})}$$

이렇게 구한 가중 경험분포는 임의의 영역(Borel 집합) $A$에 대해 입자 수가 늘어날수록 목표 분포의 적분값으로 수렴한다.

$$\left[\sum_{m=1}^M w^{[m]}\right]^{-1} \sum_{m=1}^M \mathbf{1}(x^{[m]} \in A)\, w^{[m]} \;\longrightarrow\; \int_A f(x)\, dx$$

수렴 속도는 $O(1/\sqrt{M})$이며, 제안 분포 $g$와 목표 분포 $f$가 서로 닮아 있을수록 앞의 비례 상수가 작아져 적은 입자로도 정밀한 근사가 나온다. 입자 필터에서 제안 분포 $g$는 모션 모델 $p(x_t \mid u_t, x_{t-1})$로 전개한 분포($\overline{\text{bel}}(x_t)$)이고, 목표 분포 $f$는 여기에 최신 측정 $z_t$까지 반영한 진짜 사후 분포 $\text{bel}(x_t) = \eta\, p(z_t \mid x_t)\, \overline{\text{bel}}(x_t)$다. 따라서 두 분포의 비율 $f / g$를 취하면 $\overline{\text{bel}}(x_t)$가 약분되고, 아직 반영되지 않은 "빠진 정보"인 $p(z_t \mid x_t^{[m]})$만 남는다. Step 2의 가중치 계산은 바로 이 중요도 비율을 그대로 잰 것이다.

#### 수렴 및 구현

조금 더 엄밀하게 증명하려면 입자 하나를 현재 점이 아니라 시간 전체의 상태 시퀀스 $x_{0:t}^{[m]}$로 놓고 보면 된다. 목표 시퀀스 분포를 베이즈 정리와 Markov 성질로 두 번 쪼갠 뒤, 이 식을 제안 분포의 귀납적 분해식과 나란히 놓고 비율을 계산하면 과거 항들이 차례로 지워지며 정확히 $\eta\, p(z_t \mid x_t^{[m]})$만 남는다. 다만 가중 입자 집합은 어디까지나 $M \to \infty$ 극한에서 참 분포와 일치할 뿐, 유한한 개수의 입자를 쓰는 실제 구현에서는 Step 3의 resampling이 필수적이다. 리샘플링을 건너뛰고 가중치만 계속 곱해 나가면 결국 한두 개의 입자에만 가중치가 100% 쏠리고 나머지 입자들은 죽은 계산을 반복하는 **가중치 퇴화(weight degeneracy)**에 빠지기 때문이다.

<!-- DEMO: particle_filter_1d.html -->

### 3.11.4 입자 필터의 4가지 오차 원천

입자 필터는 임의의 분포를 다룰 수 있는 강력한 근사 도구지만, 무작위 표본 추출과 유한한 입자 개수 때문에 피할 수 없는 네 가지 구조적 오차 원천을 안고 있다(PR §4.2.4). 실전에서 입자 필터(예: ROS AMCL)가 갑자기 길을 잃거나 한 점으로 뭉쳐 버릴 때, 대부분 아래 네 가지 중 하나가 원인이다.

#### (1) 유한 $M$의 체계적 편향 (Systematic Bias)

입자가 단 하나($M = 1$)뿐인 극단적인 경우를 상상해 보자. 가중치가 아무리 작든 크든 자기 자신으로 정규화되면 $w/w = 1$이 되어 버린다. 즉 측정 정보를 전혀 반영하지 못한 채 순수 모션 모델 $p(x_t \mid u_{1:t})$만 따라가게 된다. 일반적으로도 $M$개의 가중치는 정규화 전 $M$차원에서 정규화 후 $M-1$차원 공간으로 바뀌면서 자유도 하나를 잃는다. 이 때문에 사후 분산을 실제보다 과소평가하는 편향이 생긴다. 입자 수 $M$을 키울수록 이 편향은 줄어들지만 계산 시간 역시 $M$에 비례해 늘어난다.

#### (2) Resampling에 따른 다양성 상실 (Sample Impoverishment)

로봇이 제자리에 멈춰 서 있는 정적 상태($x_t = x_{t-1}$)에서 전이 잡음까지 0으로 두면 입자들은 움직이지 않고 초기 위치에 가만히 붙어 있다. 이 상태에서 리샘플링을 계속 돌리면 우연한 표집 분산(sampling variance) 때문에 살아남는 입자의 가짓수가 매 턴 줄어든다. 그러다 결국 단 하나의 입자만 수백 개로 복제되어 버리는 **sample impoverishment** 현상이 일어난다.

이를 막는 가장 실용적인 처방은 로봇이 정지해 있거나 이동량이 미미할 때 리샘플링을 잠시 멈추는 것이다. 또는 가중치의 분산이 기준치를 넘어 퇴화가 심해졌을 때만 리샘플링을 실행하고, 그 사이 구간에서는 이전 가중치에 새 측정 우도를 곱해 누적해 둔다.

$$w_t^{[m]} = \begin{cases} 1 & \text{(resampling 직후)} \\ p(z_t \mid x_t^{[m]})\, w_{t-1}^{[m]} & \text{(resampling 없을 때)} \end{cases}$$

#### (3) Proposal-Target 괴리 (Proposal-Target Divergence)

모션 오도메트리는 매우 엉성해서 입자들이 사방으로 넓게 퍼지는데, LiDAR 같은 센서는 너무나 정밀해서 참 위치 주변의 극히 좁은 영역에만 높은 우도를 준다면 어떻게 될까? 넓은 제안 분포에서 뽑은 입자 대부분이 좁은 목표 분포 바깥에 떨어져 $w \approx 0$을 받으면서 샘플링 효율이 급락한다. 극단적으로 노이즈가 0인 이상적인 거리 센서라면 측정과 정확히 일치하는 저차원 곡선 위에 입자가 하나도 걸리지 않을 수도 있다.

현장에서는 이 간극을 줄이려고 측정 노이즈 파라미터를 실제 센서 스펙보다 일부러 조금 넉넉하게 키워 봉우리를 넓혀 주거나(정밀도를 약간 양보하는 대신 안정성 확보), 최신 측정 정보까지 반영한 measurement-aware proposal 분포로 입자를 뽑는다.

#### (4) Particle Deprivation

고차원 공간에 유한한 입자만 뿌려 두면, 가끔 참 상태 근처에 입자가 단 한 개도 남지 않는 불운이 닥친다. 리샘플링 자체가 확률적 과정이라 매 사이클 참 위치 부근의 입자들이 우연히 모두 탈락할 확률이 0보다 크기 때문이다. 참 위치에서 입자가 한번 전멸하면 그 뒤로는 아무리 좋은 측정을 받아도 복구가 불가능하다.

이 문제에 대비한 가장 흔한 안전장치는 사이클마다 전체 공간(또는 측정이 가리키는 영역)에 무작위 입자를 소량 새로 섞어 넣는 **random injection**이다. 사후 분포에 약간의 잡음을 섞는 대가로 추정이 통째로 붕괴해 로봇이 영영 길을 잃는 참사를 막아 준다.

#### Low-Variance Sampler

마지막으로 Step 3 리샘플링을 실제로 구현할 때는 단순 무작위 추첨 대신 **Low-Variance(systematic) Sampler**를 표준처럼 쓴다. 난수를 딱 하나만 뽑고, 룰렛 바퀴를 $1/M$의 일정한 간격으로 한 바퀴 훑으며 입자를 집어 올리는 기법이다.

```
# Algorithm Low_variance_sampler (Table 4.4 의역)
# 입력: X̄_t (가중 입자), W_t (가중치 배열)
# 출력: X̄_t (재샘플된 입자)

r = rand(0, M^{-1})    # [0, 1/M) 균등 난수 단 1개
c = w_t^[1]             # 누적 가중치
i = 1
X̄_t = ∅
for m = 1 to M do
    u = r + (m-1) · M^{-1}   # 일정 간격으로 위치 이동
    while u > c do
        i = i + 1
        c = c + w_t^[i]       # 누적 가중치에 더함
    endwhile
    add x_t^[i] to X̄_t       # 해당 위치의 입자 선택
endfor
return X̄_t
```

이 방식에는 두 가지 큰 장점이 있다. 첫째, 모든 입자의 가중치가 같다면($w^{[m]} = 1/M$) 출력 집합이 입력과 똑같이 유지되므로 측정을 못 받은 스텝에서도 입자 다양성이 쓸데없이 줄지 않는다. 둘째, 이진 탐색을 쓰는 독립 샘플링은 $O(M \log M)$ 시간이 걸리는 데 비해, 이 방식은 누적 합 배열을 한 번만 훑고 지나가므로 계산 복잡도가 $O(M)$이다. 14장 §14.7.7–14.7.8에서는 지금 살펴본 오차들을 정면으로 다루어 로봇 납치 문제까지 해결하는 augmented MCL과 mixture MCL을 자세히 만나게 된다.

> **추천 자료**
> - [Thrun, Burgard, Fox — Probabilistic Robotics (2005)](http://probabilistic-robotics.org/) — Ch.4 전체가 이 절의 원전. 히스토그램 필터, 이진 베이즈 필터, 입자 필터의 알고리즘과 분석이 실려 있다.
> - [Arulampalam et al. — A Tutorial on Particle Filters (IEEE Trans. Signal Processing 2002)](https://ieeexplore.ieee.org/document/978374) — 입자 필터의 이론과 응용을 종합한 표준 튜토리얼.
> - [Thrun — Particle Filters in Robotics (UAI 2002)](https://arxiv.org/abs/1301.0607) — Rao-Blackwellized PF·FastSLAM과 어떻게 이어지는지 설명하는 짧은 논문.
> - [ROS AMCL package (ROS 1 문서)](https://wiki.ros.org/amcl) — §3.11.3~3.11.4의 입자 필터를 구현한 ROS 패키지. KLD-sampling MCL과 augmented MCL(recovery_alpha_slow/fast)을 쓴다.

---

이로써 로보틱스와 Spatial AI를 지탱하는 수학의 큰 그림을 한 바퀴 돌았다. 선형대수와 3D 기하학으로 공간의 자세와 변환을 적고, 확률과 최적화·Lie Group·Factor Graph·베이즈 필터로 불확실한 센서 데이터 속에서 최적의 상태를 길어 올리는 도구가 이제 손안에 들어왔다. 다음 4장(기구학과 메카트로닉스)에서는 여기서 익힌 동차 변환 행렬과 $SE(3)$, Jacobian을 실제 로봇 팔과 관절 메커니즘에 입혀, 모터의 회전 각도를 손끝의 3차원 위치·속도로 바꾸고 실제 하드웨어를 정밀하게 움직이는 세계로 나아간다.
