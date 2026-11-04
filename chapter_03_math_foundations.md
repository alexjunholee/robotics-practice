# Ch.3 — 수학적 기초 (Mathematical Foundations)

Spatial AI 논문을 읽고 구현하려면 수학적 기초가 필요하다. SLAM 논문은 "SE(3) 위의 최적화", "Jacobian을 유도하여 Gauss-Newton으로 풀었다" 같은 문장을 설명 없이 쓴다.

Differentiable Programming과 Auto-Differentiation(자동 미분)은 고전적인 수학 도구를 계산 파이프라인 안에 넣는 방식을 바꿨다. PyTorch나 JAX는 손으로 유도하던 Jacobian과 복잡한 파이프라인의 gradient를 자동으로 계산한다. 이 기능은 End-to-End 학습 기반 SLAM과 Differentiable Rendering(NeRF, 3D Gaussian Splatting)의 기반이 된다. 다만 자동 미분이 계산하는 내용을 해석하고 오류를 찾으려면 이 장에서 다루는 선형대수와 최적화가 필요하다.

## 3.1 선형대수 (Linear Algebra)

선형대수는 Spatial AI 전반에서 쓰이는 기본 도구다. 좌표 변환, 카메라 모델, 최적화, 딥러닝을 모두 행렬과 벡터로 쓴다.

### 3.1.1 벡터와 행렬

**벡터**: 크기와 방향이 있는 양

```
v = [v_x, v_y, v_z]^T  (열 벡터)
```

로보틱스에서 벡터는 3D 공간의 점, 힘, 속도 등을 나타낸다. "로봇이 월드 좌표계에서 (3, 2, 1)에 있다"는 위치를 벡터로 표현한 예다.

**행렬 연산**:
- 덧셈/뺄셈: 요소별 연산
- 곱셈: 행×열 내적
- 전치(Transpose): A^T
- 역행렬(Inverse): A^(-1), AA^(-1) = I

좌표 변환, 회전, 투영(projection)은 모두 행렬 곱으로 쓴다. 카메라가 3D 점을 2D 이미지에 투영하는 계산과 로봇의 좌표계를 바꾸는 계산이 여기에 속한다.

> **추천 자료**
> - [3Blue1Brown — Essence of Linear Algebra](https://www.youtube.com/playlist?list=PLZHQObOWTQDPD3MizzM2xVFitgF8hE_ab) — 행렬 곱셈과 고유값을 기하학적으로 시각화한다. 선형대수를 계산 규칙보다 공간 변환으로 이해하는 데 유용하다.
> - [Introduction to Applied Linear Algebra (Boyd & Vandenberghe) — 무료 PDF](https://web.stanford.edu/~boyd/vmls/) — Stanford의 Boyd와 UCLA의 Vandenberghe가 쓴 응용 선형대수 교재. 응용 예제 중심이고 Julia·Python 보조 자료가 있다.
> - [다크 프로그래머 — 선형대수학 시리즈 (6편: 기본공식~PCA)](https://darkpgmr.tistory.com/103) — 주요용어, 역행렬, 고유값, SVD, 연립방정식, PCA를 한글로 정리한 글
> - [다크 프로그래머 — 벡터 미분과 행렬 미분](https://darkpgmr.tistory.com/141) — 벡터·행렬 미분 규칙을 정리한 글. Jacobian을 계산할 때 필요하다

### 3.1.2 고유값 분해 (Eigenvalue Decomposition)

```
Av = λv
```

- v: 고유벡터 (eigenvector)
- λ: 고유값 (eigenvalue)

**활용**: PCA, 공분산 행렬 분석, 안정성 분석

포인트 클라우드를 다루면 바로 쓸 일이 생긴다. PCA(Principal Component Analysis)로 포인트 클라우드의 주축을 구할 때, 공분산 행렬의 고유벡터가 바로 주축 방향이고 고유값이 그 방향의 분산이다. "이 포인트 클라우드가 평면인지 직선인지"를 판별하는 것도 고유값의 비율로 한다. 법선 벡터(normal)는 가장 작은 고유값의 고유벡터로 추정한다.

> **추천 자료**
> - [3Blue1Brown — Eigenvectors and Eigenvalues](https://www.youtube.com/watch?v=PFDu9oVAE-g) — 고유벡터와 고유값의 기하학적 의미를 시각화한 영상(Essence of linear algebra 14장)
> - [MIT 18.06 Linear Algebra — Gilbert Strang (YouTube)](https://www.youtube.com/playlist?list=PLE7DDD91010BC51F8) — MIT 18.06 공개 강의. 고유값 분해를 포함한 전체 선형대수를 깊이 있게 다룬다.

> **실습**: [PCA 3D · 차원 축소](https://alexjunholee.github.io/robotics-practice/app.html#pca_3d)
> 3D 분포에서 공분산 행렬의 고유벡터가 주축이 되는 과정을 직접 조작하고, PC1·PC2 평면(3D→2D)과 PC1 축(2D→1D)으로 차원을 줄이는 과정을 함께 시각화한다.

### 3.1.3 특이값 분해 (SVD: Singular Value Decomposition)

```
A = UΣV^T
```

- U: 좌측 특이벡터 (m×m 직교행렬)
- Σ: 특이값 대각행렬 (m×n)
- V: 우측 특이벡터 (n×n 직교행렬)

**활용**: 최소자승법 해, 행렬 근사, Fundamental Matrix 계산

SVD는 로보틱스에서 자주 등장한다. 과결정(overdetermined) 시스템의 최소자승 해를 구하는 데 수치적으로 안정적이기 때문이다. 두 시점 사이의 Fundamental Matrix를 구할 때와 포인트 클라우드 정합에서 최적 회전을 구할 때 SVD를 쓴다. 8-point algorithm은 8개 이상의 대응점으로 선형 해를 구할 때와 결과에 rank 2 제약을 걸 때 모두 SVD를 쓴다.

> **추천 자료**
> - [Steve Brunton — Singular Value Decomposition (YouTube)](https://www.youtube.com/watch?v=nbBvuuNVfco) — University of Washington의 Steve Brunton이 SVD의 수학적 개요를 설명한 강의
> - [Introduction to Linear Algebra, 6th ed. (Gilbert Strang, 2023)](https://math.mit.edu/~gs/linearalgebra/ila6/indexila6.html) — MIT 18.06의 교재. SVD를 별도 장으로 다룬다.

## 3.2 3D 기하학 (3D Geometry)

3D 기하학은 로봇, 카메라, 물체의 위치와 방향을 같은 수학적 언어로 표현한다. SLAM은 이 표현 위에서 여러 시점의 관측을 연결한다.

### 3.2.1 좌표계 (Coordinate Frames)

Spatial AI에서는 여러 좌표계를 오가며 작업한다. World Frame(W)은 전역 고정 좌표계다. Camera Frame(C)은 카메라 중심, Body Frame(B)은 로봇 중심, IMU Frame(I)은 IMU 센서 기준 좌표계를 나타낸다.

카메라가 관측한 물체 위치는 camera frame 좌표다. 로봇이 그 위치를 쓰려면 body frame이나 world frame으로 옮겨야 한다. 센서마다 좌표계가 다르므로, sensor fusion에는 좌표계 사이의 extrinsic calibration이 필요하다.

**좌표 변환**:

```
p_W = T_WC × p_C
```

T_WC: Camera → World 변환 행렬 (4×4)

> **추천 자료**
> - [State Estimation for Robotics, Ch.6 — Primer on Three-Dimensional Geometry (Tim Barfoot) — 무료 PDF](https://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) — 좌표계, 회전, 변환을 상태 추정 관점에서 정리한 장
> - [Stanford CS231A — Camera Models](https://web.stanford.edu/class/cs231a/) — Stanford의 CV 강의에서 카메라 좌표계와 투영 모델을 다루는 부분

### 3.2.2 회전 표현 (Rotation Representations)

회전 행렬, Euler angle, quaternion, axis-angle은 파라미터 수, 제약, 특이점, 보간 방식이 다르다. SLAM 코드가 표현을 고르는 이유는 이 차이와 최적화 방식에 있다.

Rotation Matrix R은 3×3 직교행렬(det(R) = 1, R^T = R^(-1))로, 9개 파라미터에 6개 제약이 걸려 실제 자유도는 3이다.

Euler Angles은 Roll(φ), Pitch(θ), Yaw(ψ) 세 각도로 회전을 표현한다. 직관적이지만 Gimbal Lock 문제가 있고, 적용 순서(ZYX, XYZ 등)에 따라 결과가 달라진다.

**Quaternion q = [w, x, y, z]**(||q|| = 1)는 4개 파라미터로 3 DoF를 표현한다. Gimbal Lock이 없고 구면 선형 보간(Slerp, spherical linear interpolation)이 쉬워 로보틱스의 자세 표현에 널리 쓰인다.

Axis-Angle은 회전축 n과 회전각 θ를 조합한 3개 파라미터 표현이다. Rodrigues formula로 Rotation Matrix로 변환한다.

ROS는 quaternion을 기본 회전 표현으로 쓴다. OpenCV는 회전을 Rodrigues vector(axis-angle)로 주고받는다. Ceres와 GTSAM 같은 최적화 라이브러리는 Lie group 기반 표현을 지원한다. 인터페이스를 연결할 때는 각 표현의 좌표 순서와 정규화 조건을 확인한다.

> **추천 자료**
> - [3Blue1Brown — Quaternions and 3D Rotation](https://www.youtube.com/watch?v=zjMuIxRvygQ) — 쿼터니언의 기하학적 의미와 3D 회전을 인터랙티브하게 시각화한 영상.
> - [State Estimation for Robotics, Ch.6 — Primer on Three-Dimensional Geometry (Tim Barfoot)](https://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) — 회전 표현과 표현 사이의 변환을 정리한 장
> - [Sola — Quaternion Kinematics for the Error-State Kalman Filter (Tech Report)](https://arxiv.org/abs/1711.02508) — VIO/INS 구현 시 쿼터니언 기반 에러 상태 칼만 필터의 수학적 기초를 정리한 테크니컬 리포트.
> - [3D Rotation Converter](https://www.andre-gaschler.com/rotationconverter/) — 쿼터니언, 오일러 각, 회전 행렬 간 변환을 확인할 수 있는 온라인 도구

> **실습**: [회전 표현과 Gimbal Lock](https://alexjunholee.github.io/robotics-practice/app.html#rotation_gimbal) | [6DoF 포즈 시각화](https://alexjunholee.github.io/robotics-practice/app.html#xyzrpy_6dof)
> 오일러 각의 Gimbal Lock 현상과 쿼터니언 회전을 직접 조작하며 비교하고, 6자유도 포즈(x, y, z, roll, pitch, yaw)를 인터랙티브하게 확인할 수 있다.

### 3.2.3 Homogeneous Coordinates

3D 점에 1을 덧붙여 4차원 벡터로 쓰면 회전과 이동을 한 행렬로 표현한다.

```
[X, Y, Z, 1]^T  (3D 점)

T = | R   t |   (4×4 변환 행렬)
    | 0   1 |
```

일반 좌표에서는 변환을 p' = Rp + t처럼 곱셈과 덧셈으로 쓴다. Homogeneous coordinates(동차 좌표)로 쓰면 이동(translation)까지 p' = Tp 한 번의 곱에 들어간다. 여러 변환을 이어 적용할 때 행렬을 차례로 곱하면 되므로 로봇 팔의 관절 변환 같은 체인을 다루기 편하다.

### 3.2.4 SE(3)와 SO(3)

**SE(3)**(Special Euclidean Group)는 3D 강체 변환(회전 + 이동) 전체의 집합이며 6 DoF다. **SO(3)**(Special Orthogonal Group)는 회전만의 집합으로 3 DoF다.

SE(3)와 SO(3)는 Lie Group이다. 최적화에서는 회전 행렬의 제약(직교, 행렬식 1)을 지키며 값을 갱신해야 한다. 대응하는 **Lie Algebra**(se(3), so(3))에서 제약 없이 갱신량을 구한 뒤 Exponential Map으로 Lie Group에 되돌리면 이 제약이 저절로 지켜진다. SLAM의 Pose Graph Optimization에서 이 개념이 핵심으로 쓰인다.

> **추천 자료**
> - [State Estimation for Robotics, Ch.7 — Matrix Lie Groups (Tim Barfoot) — 무료 PDF](https://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) — SE(3), SO(3), Lie group·algebra를 로보틱스 상태 추정 관점에서 설명한 교재
> - [Sola — A Micro Lie Theory for State Estimation in Robotics (arXiv:1812.01537)](https://arxiv.org/abs/1812.01537) — Lie Group 이론을 로보틱스 상태 추정에 필요한 만큼만 간결하게 정리한 논문. 매우 실용적.

## 3.3 확률 및 통계 (Probability & Statistics)

센서 관측에는 노이즈가 섞이고, 로봇이 추정한 상태에도 불확실성이 있다. 확률과 통계는 이 불확실성을 수식으로 다룬다. 센서 값을 "정확히 3.0 m"로 두지 않고 "3.0 m ± 0.05 m (95% 신뢰구간)" 같은 분포로 다루며, 상태 추정은 이 불확실성을 전파하고 갱신하는 일이다.

### 3.3.1 정규분포 (Gaussian Distribution)

```
p(x) = (1 / √(2πσ²)) × exp(-(x-μ)²/(2σ²))
```

**다변량 정규분포**:

```
p(x) = N(μ, Σ)
```

- μ: 평균 벡터
- Σ: 공분산 행렬

센서 노이즈와 위치 불확실성을 모델링할 때 쓴다.

정규분포는 수학적으로 다루기 편해서 많이 쓴다. 중심극한정리는 적절한 조건에서 많은 작은 독립 효과의 합이 정규분포로 근사되는 이유를 설명한다. 독립 정규 확률변수의 합은 정규분포이고, 같은 변수의 정규 밀도끼리 곱해도 정규화하면 정규 밀도가 되어 분석이 쉽다. 칼만 필터가 정규분포를 가정하는 것도 같은 이유다.

> **추천 자료**
> - [3Blue1Brown — But what is the Central Limit Theorem?](https://www.youtube.com/watch?v=zeJD6dqJ5lo) — 중심극한정리를 시각적으로 설명하고, 정규분포가 여러 곳에서 나타나는 이유를 보여 준다.
> - [Kalman Filter — How it works, in pictures](https://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/) — 칼만 필터의 작동 원리를 시각적으로 설명. 수식 전에 직관을 잡기 좋다

> **실습**: [Kalman Filter](https://alexjunholee.github.io/robotics-practice/app.html#kalman_filter)
> 칼만 필터의 predict-update 사이클을 인터랙티브하게 조작하며, 정규분포 기반 상태 추정 과정을 확인할 수 있다.

**Mahalanobis Distance**

유클리드 거리는 모든 방향을 같은 무게로 잰다. 센서 데이터는 방향마다 불확실성이 다르다. GPS는 수직 방향 오차가 수평 방향 오차보다 크다.

Mahalanobis 거리는 공분산(covariance)을 고려한 거리다.

```
d_M = sqrt((x - μ)^T Σ^{-1} (x - μ))
```

Σ가 단위 행렬이면 유클리드 거리와 같다. Σ가 대각 행렬이면 축마다 스케일을 맞춘 거리다. 축 사이에 상관이 있는 Σ에서는 공분산의 주축 방향마다 스케일을 맞춘 거리가 된다.

SLAM 활용 예: 데이터 연관(data association) 시 "이 관측이 이 랜드마크에서 왔는가?"를 판단할 때 Mahalanobis 거리를 쓴다. 유클리드로 가까워도 Mahalanobis로 멀면 (불확실성 방향과 맞지 않으면) 잘못된 연관일 가능성이 높다.

(참고: [다크 프로그래머 — 평균, 표준편차, 분산, 그리고 Mahalanobis 거리](https://darkpgmr.tistory.com/41))

### 3.3.2 베이즈 정리 (Bayes' Rule)

```
P(A|B) = P(B|A) × P(A) / P(B)
```

베이즈 정리는 센서 측정이 주어졌을 때 상태의 확률을 갱신한다. 칼만 필터와 입자 필터는 이 갱신을 서로 다른 분포 표현과 근사로 구현한다.

**재귀적 상태 추정**:

```
P(x_t | z_{1:t}) ∝ P(z_t | x_t) × P(x_t | z_{1:t-1})
```

- P(z_t | x_t): Measurement model (관측 모델) — "로봇이 이 위치에 있다면, 센서가 이 값을 출력할 확률은?"
- P(x_t | z_{1:t-1}): Prior (이전 상태 기반 예측) — "이전까지의 정보로 볼 때 로봇이 여기 있을 확률은?"

새 센서 데이터가 들어올 때마다 prior를 measurement likelihood와 결합해 posterior를 구한다. 칼만 필터의 predict-update cycle은 선형 Gaussian 조건에서 이 재귀를 구현한다.

베이즈 정리를 시간에 대해 *재귀적으로* 적용하면 §3.9의 베이즈 필터가 된다.

> **추천 자료**
> - [3Blue1Brown — Bayes' Theorem](https://www.youtube.com/watch?v=HZGCoVF3YvM) — 베이즈 정리를 시각적으로 이해하기 좋은 영상
> - [Probabilistic Robotics (Thrun, Burgard, Fox)](http://probabilistic-robotics.org/) — 베이즈 필터, 칼만 필터, 입자 필터, SLAM을 확률론의 틀로 정리한 교과서.
> - [김기섭 블로그 — Bayesian Filtering 시리즈 (2편)](https://gisbi-kim.github.io/blog/2021/03/09/bayesfiltering-1.html) — 베이즈 필터를 한글로 해설하고 칼만 필터까지 이어지는 글

### 3.3.3 MLE와 MAP

**MLE (Maximum Likelihood Estimation)**:

```
x* = argmax P(z | x)
```

관측 z가 나올 확률(likelihood)을 가장 크게 만드는 x를 구한다.

**MAP (Maximum A Posteriori)**:

```
x* = argmax P(x | z) = argmax P(z | x) × P(x)
```

사전 확률(prior)을 고려해 추정한다.

SLAM에서 "관측 데이터만 보고 최적 위치를 구하는 것(MLE)"과 "이전 위치 정보도 함께 고려하여 최적 위치를 구하는 것(MAP)"의 차이다. 실제 SLAM 시스템은 대부분 MAP 추정을 쓴다. Prior를 넣으면 노이즈가 심한 관측에도 안정적으로 추정할 수 있기 때문이다. 관측 노이즈가 가우시안이고 공분산이 고정되어 있으면, 음의 로그 사후확률을 최소화하는 MAP 문제는 "가중 관측 오차의 제곱합 + 음의 로그 사전확률에 따른 정규화 항"이 된다. 최적화 관점에서 보면 Regularized Least Squares와 같은 꼴이다.

> **추천 자료**
> - [Probabilistic Robotics, Ch.2 — Recursive State Estimation (Thrun)](http://probabilistic-robotics.org/) — MLE, MAP, 베이즈 필터의 관계를 로보틱스 맥락에서 설명
> - [StatQuest — Maximum Likelihood, clearly explained](https://www.youtube.com/watch?v=XepXtl9YKwc) — Josh Starmer가 MLE를 예시로 설명한 영상

**MLE와 MAP의 직관적 차이**

MLE(Maximum Likelihood)는 "이 데이터가 관측될 확률을 가장 높이는 파라미터는?"이라고 묻는다. 데이터만 본다. MAP(Maximum A Posteriori)는 여기에 prior를 더한다. "이 데이터가 관측되었을 때, 사전 지식까지 합쳐서 파라미터의 사후 확률을 가장 높이는 값은?"이 그 질문이다.

로그를 취하면 MAP의 목적함수는 MLE의 목적함수에 log prior 항을 더한 꼴이다. 가우시안 prior를 두면 이 항이 L2 정규화가 된다. 딥러닝의 weight decay는 가우시안 prior를 둔 MAP 추정으로 해석된다.

SLAM에서: odometry 측정값의 likelihood와 센서 관측의 likelihood를 곱하고, 이전 상태의 prior를 결합하여 MAP 추정을 한다. Factor graph에서 각 factor가 바로 이 likelihood/prior에 해당한다.

(참고: [다크 프로그래머 — 베이즈 정리, ML과 MAP, 그리고 영상처리](https://darkpgmr.tistory.com/62))

## 3.4 최적화 기초 (Optimization Basics)

SLAM의 bundle adjustment, 카메라 calibration, 딥러닝 학습은 모두 목적함수를 최소화한다. 잔차, Jacobian, 갱신 규칙을 구분하면 수렴 실패가 모델·초기값·solver 가운데 어디에서 생겼는지 추적할 수 있다.

### 3.4.1 Least Squares

```
x* = argmin ||Ax - b||²
```

**정규방정식의 해 (Normal Equation, `A`가 full column rank인 경우)**:

```
x* = (A^T A)^(-1) A^T b
```

최소자승법은 "노이즈가 있는 여러 측정값에서 가장 적합한 모델 파라미터를 찾는" 가장 기본적인 방법이다. 직선 피팅부터 카메라 캘리브레이션까지 추정 문제 대부분의 출발점이다.

> **추천 자료**
> - [Cyrill Stachniss — Least Squares for Robotics](https://www.youtube.com/watch?v=r2cyMQ5NB1o) — 최소자승법을 로보틱스 문제에 적용하는 방법을 구체적으로 설명
> - [김기섭 블로그 — SLAM back-end 시리즈 (3편)](https://gisbi-kim.github.io/blog/2021/03/04/slambackend-1.html) — "SLAM은 Ax=b를 푸는 문제"에서 시작하는 back-end 입문. Factor graph까지 3편 시리즈
> - [김기섭 블로그 — Iterative Optimization 1편](https://web.archive.org/web/20240819154205/https://gisbi-kim.github.io/blog/2021/03/16/leastsquare-1.html) — 비선형 최적화의 직관적 한글 해설

**최소자승법의 직관**

제곱 오차는 어디서나 미분되고, 큰 오차에 더 큰 페널티를 준다. 노이즈가 가우시안이면 최소자승 해가 Maximum Likelihood 해와 같다.

over-determined system (방정식 수 > 미지수 수)에서는 Ax = b를 정확히 만족하는 x가 없을 수 있다. 대신 ||Ax - b||²를 최소화하는 x를 찾으면, normal equation `A^T A x = A^T b`가 된다.

주의: `A^T A`가 singular하면 (rank 부족) 위의 역행렬은 존재하지 않고, 최소자승 해도 유일하지 않을 수 있다. `A = U Σ V^T`를 SVD했을 때 Moore–Penrose pseudo-inverse를 `A^+ = V Σ^+ U^T`로 구하면, 최소 norm 해 `x = A^+ b`를 얻을 수 있다.

(참고: [다크 프로그래머 — 최소자승법 이해와 다양한 활용예](https://darkpgmr.tistory.com/56))

### 3.4.2 Gradient Descent

```
x_{k+1} = x_k - α × ∇f(x_k)
```

- α: Learning rate
- ∇f: Gradient (기울기)

Gradient Descent는 딥러닝에서 매일 쓰는 알고리즘이고, 로보틱스 최적화에서도 기본이다. 함수의 기울기 반대 방향으로 조금씩 움직여 최솟값을 찾는다. learning rate를 정하기 어렵고 수렴이 느려서, 로보틱스의 비선형 최소자승 문제에는 보통 Gauss-Newton이나 LM(Levenberg-Marquardt)을 쓴다.

**Gradient, Jacobian, Hessian**

Gradient ∇f는 스칼라 함수 f의 1차 미분으로 n×1 벡터를 출력한다. "어느 방향으로 가야 f가 가장 빠르게 증가하는가"를 알려준다. Jacobian J는 이를 벡터 함수 f: R^n → R^m으로 확장한 m×n 행렬이다. (i, j) 원소는 출력 i를 입력 j로 편미분한 값이다. Hessian H는 스칼라 함수 f의 2차 미분으로 n×n 대칭 행렬이며, 곡률 정보를 담아 Newton's method에서 쓴다.

관계:
```
비용 함수 C(x) = (1/2)||r(x)||² 일 때:
  Gradient:  ∇C = J^T r           (J는 r의 Jacobian)
  Hessian:   H ≈ J^T J            (Gauss-Newton 근사: 2차 미분 항 무시)
  Update:    δx = -(J^T J)^{-1} J^T r
```

Gauss-Newton은 J^T J를 Hessian 근사로 쓴다. 정확한 Hessian은 계산이 비싸다. 잔차 r이 작은 영역에서는 버린 2차 항도 작다.

(참고: [다크 프로그래머 — Gradient, Jacobian 행렬, Hessian 행렬, Laplacian](https://darkpgmr.tistory.com/132))

### 3.4.3 Gauss-Newton

비선형 최소자승 문제를 현재 추정값 주변에서 선형화해 풀고, 이를 반복한다.

```
(J^T J) Δx = -J^T r
x_{k+1} = x_k + Δx
```

- J: Jacobian 행렬
- r: Residual (잔차)

로보틱스에서 Gauss-Newton을 Gradient Descent보다 많이 쓰는 이유는 Hessian의 근사인 J^T J로 곡률 정보를 써서 더 빨리 수렴하기 때문이다. 변수가 수천~수만 개인 SLAM 문제에서도 Gauss-Newton은 초기값이 좋으면 몇 번의 반복으로 수렴한다.

### 3.4.4 Levenberg-Marquardt (LM)

Gauss-Newton 정규방정식에 damping 항 λI를 더한다.

```
(J^T J + λI) Δx = -J^T r
```

- λ: Damping factor
- λ 작음 → Gauss-Newton (빠른 수렴)
- λ 큼 → Gradient Descent (안정적)

SLAM의 bundle adjustment와 pose graph optimization에 쓴다.

Gauss-Newton은 초기값이 좋으면 빠르게 수렴하고, 초기값이 나쁘면 발산하기도 한다. Ceres Solver, g2o, GTSAM 같은 최적화 라이브러리에 LM이 구현되어 있다.

> **추천 자료**
> - [Cyrill Stachniss — Graph-based SLAM with Gauss-Newton Optimization](https://www.youtube.com/watch?v=hRyL5KwFLAE) — Freiburg 강의(2009–2010). Graph-based SLAM을 Gauss-Newton으로 최적화하는 과정을 설명한다
> - [State Estimation for Robotics, Ch.4 — Nonlinear Non-Gaussian Estimation (Tim Barfoot) — 무료 PDF](https://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) — 비선형 추정과 최적화를 상태 추정 관점에서 다룬다
> - [Ceres Solver Tutorial](http://ceres-solver.org/tutorial.html) — Google이 공개한 비선형 최소자승 라이브러리의 튜토리얼. LM을 코드에서 쓰는 방법을 따라 해 볼 수 있다.
> - [다크 프로그래머 — 최적화 기법의 직관적 이해](https://darkpgmr.tistory.com/149) — Gradient Descent, Newton, LM 등의 기하학적 직관
> - [김기섭 블로그 — SLAM Back-end 공부자료 5개 추천](https://gisbi-kim.github.io/blog/2021/10/03/slam-textbooks.html) — Error-state KF, Factor Graphs, Bundle Adjustment 등 핵심 자료를 추천한 글
> - [Derivative Calculator](https://www.derivative-calculator.net/) — 수식 미분을 단계별로 보여주는 온라인 도구. Jacobian 유도할 때 검산에 유용

**LM의 직관: Gauss-Newton과 Gradient Descent 사이의 스위칭**

λ가 작으면 Gauss-Newton에 가까워 해 근처에서 빠르게 수렴하고, 크면 Gradient Descent에 가까워 해에서 먼 초기 단계에서 안정적이다. update가 비용을 줄이면 λ를 줄이고, 비용이 늘면 λ를 키운다. Ceres Solver의 trust region 기본 전략도 LM이다.

(참고: [다크 프로그래머 — 함수최적화 기법 정리 (LM 방법 등)](https://darkpgmr.tistory.com/142))

## 3.5 심화: Lie Group과 Lie Algebra

로보틱스에서 자주 부딪히는 수학 문제가 회전을 어떻게 최적화하느냐다. Lie group과 Lie algebra는 회전과 강체 변환을 체계적으로 다루는 도구이고, SLAM 백엔드, Visual-Inertial Odometry, Bundle Adjustment의 수식에 계속 나온다.

### 3.5.1 왜 Lie Group이 필요한가

3.2절에서 회전을 표현하는 여러 방법을 다뤘다. 그런데 이 표현들로 바로 최적화하면 문제가 생긴다.

- **회전 행렬 R**: 3x3이므로 파라미터가 9개인데, 실제 자유도는 3이다. R^T R = I 와 det(R) = 1이라는 제약 조건이 있기 때문이다. 일반적인 unconstrained optimization으로 업데이트하면 R이 회전 행렬의 조건을 벗어난다.
- **쿼터니언**: 4개 파라미터에 정규화 제약(||q|| = 1)이 있다. 업데이트할 때마다 re-normalize해야 하고, 이 과정에서 수치 오류가 누적될 수 있다.
- **오일러 각**: Gimbal lock 문제가 있고, 각도 wrapping도 까다롭다.

회전은 비선형 manifold 위에 있다. Gauss-Newton과 LM은 유클리드 공간에서 동작한다. Lie group 이론이 둘을 잇는다. Manifold 위의 점(회전 행렬) 근처에 접선 공간(Lie algebra)을 정의하고, 그 공간에서 유클리드 공간처럼 최적화한 뒤 결과를 다시 manifold 위로 올린다.

배경 지식: 여기서 말하는 "group"이란, 어떤 연산에 대해 닫힘(closure), 결합법칙(associativity), 항등원(identity), 역원(inverse)이 성립하는 집합이다. 예를 들어 invertible한 n x n 행렬의 집합은 행렬 곱에 대해 group을 이루며, 이를 general linear group GL(n)이라 한다. 그중 det = 1인 부분군이 special linear group SL(n)이다. Orthogonal group O(n)은 내적을 보존하는 행렬의 집합이다. 여기서 det = 1인 것만 모으면 SO(n) — 즉 회전군이 된다. det = -1인 행렬은 반사(reflection)를 포함한다. 두 반사의 곱은 det = 1이므로, det = -1인 집합은 부분군이 아니다.

### 3.5.2 SO(3): 3D 회전군

**정의:**
```
SO(3) = { R in R^{3x3} | R^T R = I, det(R) = 1 }
```

SO(3)는 group이다. 군 연산은 행렬 곱이고, 두 회전 R_1, R_2의 합성 R_1 R_2도 SO(3)의 원소다. 항등원은 단위 행렬 I, 역원은 R^T(= R^{-1})이다. 직교 행렬이므로 전치가 곧 역행렬이 된다. 행렬 곱은 결합법칙을 만족하지만 교환법칙은 성립하지 않는다(일반적으로 R_1 R_2 != R_2 R_1).

**Lie algebra so(3):**

SO(3)의 Lie algebra는 3x3 반대칭 행렬(skew-symmetric matrix)의 공간이며, 3차원이다.

Hat operator `[.]x` 는 3차원 벡터를 반대칭 행렬로 변환한다:

```
w = [w1, w2, w3]^T  (in R^3)

        [  0   -w3   w2 ]
[w]x =  [  w3   0   -w1 ]   in so(3)
        [ -w2   w1   0  ]
```

이 행렬은 벡터 외적(cross product)에 대응한다: `[w]x v = w x v`

Vee operator `(.)v` 는 역변환이다: 반대칭 행렬에서 3차원 벡터를 추출한다.

so(3)의 원소 w는 방향으로 회전축을, 크기로 회전각을 담는다. 축-각(axis-angle) 표현과 같은 정보다.

### 3.5.3 Exponential Map과 Logarithmic Map

**Exponential map**: so(3) -> SO(3)

Lie algebra의 원소(벡터)를 Lie group의 원소(회전 행렬)로 보내는 사상이다. 이것이 어디서 나오는지 유도해 보자.

시간에 따라 연속적으로 회전하는 행렬 R(t)가 있다고 하자 (R(0) = I). R(t)는 항상 SO(3)에 있으므로 `R(t) R(t)^T = I`이다. 양변을 t로 미분하면:

```
d/dt (R R^T) = R_dot R^T + R R_dot^T = 0
→  R_dot R^T = -(R_dot R^T)^T
```

즉 `R_dot R^T`는 반대칭 행렬(skew-symmetric)이다. 이를 어떤 벡터 w(t)의 hat form으로 쓸 수 있다:

```
R_dot(t) R^T(t) = [w(t)]x
→  R_dot(t) = [w(t)]x R(t)
```

w가 상수(일정한 각속도)인 경우, 이 미분 방정식의 해는:

```
R(t) = exp([w]x * t) = sum_{n=0}^{inf} ([w]x * t)^n / n!
```

여기서 `exp([w]x)`는 축 w 방향으로 ||w|| 라디안만큼 회전시키는 행렬이 된다. 구체적으로, theta = ||w||로 두면 Rodrigues' formula로 닫힌 형태를 얻는다:

```
exp([w]x) = I + (sin(theta) / theta) [w]x + ((1 - cos(theta)) / theta^2) [w]x^2
```

이 공식은 `sin(theta)`와 `cos(theta)`의 Taylor 전개를 `[w]x`의 거듭제곱에 대입하면 유도된다. `[w]x^3 = -theta^2 [w]x`라는 성질을 이용하면 급수가 sin, cos 항으로 정리된다.

theta가 작을 때(|theta| < eps)는 sin(theta)/theta ≈ 1, (1-cos(theta))/theta^2 ≈ 1/2이므로:

```
exp([w]x) ≈ I + [w]x + (1/2)[w]x^2   (2차 항까지의 근사)
```

주의: 하나의 회전 행렬 R에 대해 `R = exp([w]x)`를 만족하는 w는 여러 개다. 같은 축 방향으로 크기만 2*pi*k (정수 k)만큼 바꾼 w도 같은 R을 준다. 이것이 logarithmic map에서 주의해야 하는 부분이다.

**Logarithmic map**: SO(3) -> so(3)

역변환이다. 주어진 회전 행렬 R에서 축-각 벡터 w를 복원한다.

```
theta = arccos((tr(R) - 1) / 2)
[w]x = (theta / (2 sin(theta))) (R - R^T)
```

theta = 0 (항등 회전) 이나 theta = pi (180도 회전) 근처에서는 특별한 처리가 필요하다.

**직관**: Lie algebra는 group 위의 한 점(보통 항등원 I)의 접선 공간(tangent space)이다. "작은 회전"은 접선 공간의 벡터로 표현할 수 있고, exponential map이 이 벡터를 manifold 위의 실제 회전으로 매핑한다. 최적화는 업데이트량 dw를 접선 공간(R^3)에서 계산한 뒤, exp([dw]x)를 현재 회전에 곱해 manifold 위에서 이동한다.

### 3.5.4 SE(3): 3D 강체 변환군

로봇의 포즈는 회전뿐 아니라 이동도 포함한다. 이를 다루는 것이 SE(3)이다.

**정의:**
```
SE(3) = { T = [ R  t ] | R in SO(3), t in R^3 }
              [ 0  1 ]
```

T는 4x4 homogeneous transformation matrix이다. SE(3)도 group이다. 군 연산은 행렬 곱(T_1 T_2)이다. 항등원은 4x4 단위 행렬, 역원은 T^{-1} = [ R^T  -R^T t ; 0  1 ]이다.

**Lie algebra se(3):**

SE(3)의 Lie algebra는 6차원이다. 원소를 twist 벡터라 부른다:

```
xi = [rho; w] in R^6     (rho in R^3: 이동 성분, w in R^3: 회전 성분)
```

Hat operator는 6차원 벡터를 4x4 행렬로 변환한다:

```
        [ [w]x  rho ]
xi^ =   [  0     0  ]   in se(3)    (4x4 행렬)
```

**Exponential map**: se(3) -> SE(3)

```
exp(xi^) = [ exp([w]x)   J rho ]   in SE(3)
           [    0           1   ]
```

여기서 J는 left Jacobian of SO(3)이다:

```
J = I + ((1 - cos(theta)) / theta^2) [w]x + ((theta - sin(theta)) / theta^3) [w]x^2
```

6-DoF 포즈(3 회전 + 3 이동)는 6차원 벡터 xi in R^6으로 매개변수화할 수 있다. 제약 조건이 없는 6차원 유클리드 공간에서 최적화한 뒤, exponential map으로 결과를 SE(3) manifold 위에 올린다. SLAM 최적화에서 Lie group을 쓰는 이유가 여기에 있다.

> **실습**: [SE(3) Pose Composition](https://alexjunholee.github.io/robotics-practice/app.html#pose_composition_3d)
> SE(3) 변환의 합성을 3D로 직접 조작하며, 회전과 이동이 결합된 강체 변환이 어떻게 연쇄되는지 확인할 수 있다.

### 3.5.5 Perturbation Model과 Jacobian

Gauss-Newton이나 LM 알고리즘으로 포즈를 최적화할 때, 현재 추정값 T에 작은 변화(perturbation) d_xi를 가하는 방법이 두 가지 있다.

**Left perturbation (global frame 기준):**
```
T' = exp(d_xi^) * T
```

**Right perturbation (body frame 기준):**
```
T' = T * exp(d_xi^)
```

어느 쪽을 쓰든 한 convention을 끝까지 지키면 된다. 문헌마다 convention이 다르다. Barfoot의 교재는 주로 left perturbation을 쓰고, Sophus가 Ceres용으로 제공하는 Jacobian `Dx_this_mul_exp_x_at_0`은 right perturbation(T·exp(x)) 기준이다.

**Jacobian 계산:**

에러 함수 e(T)의 perturbation Jacobian은:

```
de/d(d_xi) = lim_{d_xi->0}  (e(exp(d_xi^) * T) - e(T)) / d_xi     (left perturbation의 경우)
```

이 Jacobian은 6열짜리 행렬이 된다 (에러 차원 x 6).

일반적인 최적화에서 업데이트는 `x <- x + dx` (유클리드 덧셈)이다. SE(3) 위에서는 군 연산(행렬 곱)으로 업데이트한다.

1. 접선 공간에서 d_xi in R^6을 Gauss-Newton으로 계산한다: `d_xi = -(J^T J)^{-1} J^T e`
2. Manifold 위에서 업데이트한다: `T <- exp(d_xi^) * T`

이렇게 하면 별도의 제약 조건 처리 없이도 업데이트 후의 T가 항상 유효한 SE(3) 원소다.

g2o, GTSAM, Ceres (with local parameterization / manifold)에서 내부적으로 이 방식을 쓴다. GTSAM의 `Pose3`는 SE(3)를 직접 구현하고, `Pose3::Expmap()`과 `Pose3::Logmap()`이 있다. Ceres에서는 `LocalParameterization` (Ceres 2.1부터는 `Manifold`)으로 같은 개념을 구현한다.

### 3.5.6 Adjoint Representation

twist를 다른 좌표계로 변환해야 할 때 Adjoint를 쓴다.

SE(3)의 원소 T에 대해, Adjoint 행렬 Ad_T는 6x6 행렬이다:

```
Ad_T = [ R    [t]x R ]   in R^{6x6}
       [ 0      R    ]
```

twist 변환:
```
xi_a = Ad_{T_ab} * xi_b
```

**실용적 의미**: 센서 프레임에서 표현한 twist(angular velocity, linear velocity)를 body 프레임이나 world 프레임의 twist 표현으로 바꿀 때 Adjoint를 쓴다. IMU는 각속도와 비력(specific force)을 측정한다. 이 측정 벡터의 축 변환에는 회전 행렬을 쓴다. VIO처럼 여러 센서를 fusion하는 시스템에서는 좌표계 변환이 자주 일어난다.

### 3.5.7 실무에서의 사용

**Sophus (C++)**: Strasdat가 만든 Lie group 라이브러리. SO(3), SE(3)와 그 exponential/logarithmic map, Adjoint 등을 구현한다. ORB-SLAM3 등 여러 SLAM 시스템이 쓴다. Kimera는 Lie group 연산을 GTSAM의 Pose3/Rot3로 처리한다.

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

**Jaxlie (Python/JAX)**: Brent Yi가 만든 JAX 기반 Lie group 라이브러리. 자동 미분이 가능하므로, Jacobian을 손으로 유도하지 않아도 된다. 연구 프로토타이핑에 유용하다.

```python
import jaxlie
import jax.numpy as jnp

T = jaxlie.SE3.identity()
delta = jnp.array([0.01, 0.0, 0.0, 0.0, 0.0, 0.001])
T_updated = jaxlie.SE3.exp(delta) @ T
```

**GTSAM**: `gtsam::Pose3`가 내부적으로 SE(3)를 쓴다. Factor graph 최적화 시 Lie group 위의 perturbation을 자동으로 처리한다.

> **추천 자료**
> - [State Estimation for Robotics, Ch.7 — Matrix Lie Groups (Barfoot)](https://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) — Lie group을 로보틱스 상태 추정 관점에서 다룬 장. 8장은 이를 포즈 추정 문제에 적용한다
> - [A micro Lie theory for state estimation in robotics (Sola et al., arXiv:1812.01537)](https://arxiv.org/abs/1812.01537) — Lie group 이론을 상태 추정에 필요한 만큼 간추린 글
> - [TUM Multiple View Geometry, Ch.2 — Rigid Body Motion](https://cvg.cit.tum.de/teaching/online/mvg) — Daniel Cremers 교수의 강의. SO(3), SE(3)를 시각적으로 설명
> - [Sophus GitHub](https://github.com/strasdat/Sophus) — C++ Lie group 라이브러리. 코드를 읽으면 이해가 빨라진다
> - [정진용 블로그 — SE(3) and SO(3) transformation](https://jinyongjeong.github.io/2016/06/07/se3_so3_transformation/) — SE(3), SO(3) 변환의 한글 정리. GL(3), O(3)부터 체계적으로 설명
> - [T-Robotics: Lie Group Formulation for Robot Mechanics](http://t-robotics.blogspot.com/2015/07/lie-group-formulation-for-robot.html) — 한국어로 작성된 Lie Group 설명. 로봇 역학에 Lie Group을 쓰는 방법을 정리

## 3.6 심화: Factor Graph

Factor graph는 SLAM 문제를 체계적으로 정의하고 효율적으로 푸는 프레임워크다. 현대 SLAM 시스템 가운데 여럿이 백엔드에 factor graph를 쓴다.

### 3.6.1 Factor Graph란

Factor graph는 두 종류의 노드로 이루어진 이분 그래프(bipartite graph)다:

- 변수 노드(variable nodes): 추정하고자 하는 상태. 로봇 포즈(x_1, x_2, ...), 랜드마크 위치(l_1, l_2, ...) 등.
- 팩터 노드(factor nodes): 변수들 사이의 제약 조건 또는 측정. 각 팩터는 연결된 변수들의 비용 함수를 정의한다.

확률적으로, 전체 사후 분포는 팩터들의 곱으로 분해된다:

```
p(X | Z) proportional to  prod_i  f_i(X_i)
```

여기서 X_i는 팩터 f_i에 연결된 변수들의 부분 집합이다.

MAP 추정 = 모든 팩터의 곱을 최대화 = 음의 로그를 취하면 합을 최소화 = nonlinear least squares 문제가 된다:

```
X* = argmin_X  sum_i  ||e_i(X_i)||^2_{Sigma_i}
```

e_i는 에러 함수, Sigma_i는 해당 측정의 공분산(불확실성 가중치)이다.

### 3.6.2 SLAM을 Factor Graph로 표현

SLAM에서 쓰는 대표적인 팩터:

| 팩터 | 역할 |
|---|---|
| Prior factor | 초기 포즈의 사전 정보. 예: "시작점은 원점이다" |
| Odometry factor | 두 연속 포즈 사이의 상대 변환. wheel odometry 등에서 온다 |
| Landmark observation factor | 포즈에서 랜드마크를 관측한 측정. reprojection error가 대표적 |
| Loop closure factor | 이전에 방문한 장소를 재인식했을 때 추가. 전체 궤적의 drift를 보정하는 핵심 |
| IMU preintegration factor | 두 키프레임 사이의 IMU 측정을 하나의 팩터로 요약 |

ASCII로 간략히 표현하면:

```
 [prior]---x1---[odom]---x2---[odom]---x3
                  |                      |
              [landmark]            [landmark]
                  |                      |
                  l1                     l2

            x3 ---[loop closure]--- x1
```

각 팩터에는 측정값과 공분산(노이즈 모델)이 포함된다. 그래프를 만들면 Gauss-Newton 또는 LM으로 전체 변수를 동시에 최적화한다.

### 3.6.3 풀이: Variable Elimination과 Bayes Tree

Factor graph를 최적화하려면 정규 방정식 `H d = -b`를 풀어야 한다 (H는 Hessian 근사, b는 gradient).

**Variable elimination**: 변수를 하나씩 소거하는 과정. 수학적으로는 sparse Cholesky factorization과 같다. 소거 순서에 따라 fill-in (원래 0이었던 곳이 non-zero가 되는 현상)이 달라지고, fill-in이 계산 비용을 바로 좌우한다.

**Variable ordering**: 소거 순서를 최적화하는 것이 중요하다. COLAMD (Column Approximate Minimum Degree) 같은 heuristic이 널리 쓰인다. 직관적으로, 연결이 적은 변수를 먼저 소거하면 fill-in이 적다.

**Bayes tree**: Kaess et al. (WAFR 2010)이 제안한 자료구조로, iSAM2(Kaess et al., IJRR 2012)의 핵심이다. Factor graph를 elimination하면 Bayes net이 되고, 이를 tree 구조로 재편하면 Bayes tree가 된다. 새로운 측정이 들어올 때, 영향을 받는 subtree만 re-elimination하면 된다.

실시간 SLAM에서는 매 프레임마다 새로운 팩터가 추가된다. 전체 시스템을 밀집 행렬로 처음부터 다시 풀면 최악의 경우 O(n^3)이 들지만, Bayes tree를 이용한 incremental update는 영향받는 부분만 갱신하므로 실시간 처리가 가능하다.

Factor graph는 §3.10의 정보 필터(Information Filter)와 깊이 연결된다 — 두 표현 모두 정보 행렬 $\Omega$의 sparse 구조를 활용한다.

> **추천 자료**
> - [Factor Graphs and GTSAM (Dellaert & Kaess)](https://gtsam.org/tutorials/intro.html) — GTSAM 공식 튜토리얼. Factor graph가 SLAM에 어떻게 쓰이는지 설명
> - [Factor Graphs for Robot Perception (Dellaert & Kaess, 2017)](https://www.cs.cmu.edu/~kaess/pub/Dellaert17fnt.pdf) — Foundations and Trends in Robotics에 실린 단행본 분량의 리뷰
> - [CMU 16-833 Lecture Notes](https://www.cs.cmu.edu/~kaess/teaching/16833/) — Michael Kaess 교수의 SLAM 강의. Factor graph와 iSAM2를 깊이 다룬다

> **실습**: [Factor Graph 시각화](https://alexjunholee.github.io/robotics-practice/app.html#factor_graph_viz)
> Factor graph의 변수 노드와 팩터 노드를 직접 구성하고, 그래프 구조가 최적화에 미치는 영향을 확인할 수 있다.

### 3.6.4 Ceres Solver로 Pose Graph 최적화 구현하기

GTSAM 외에 Google의 Ceres Solver로도 factor graph 기반 최적화를 구현할 수 있다. Ceres는 범용 nonlinear least squares 솔버라 SLAM용 팩터를 직접 정의해야 하고, 그 과정에서 내부 동작이 드러난다. 아래 코드는 Ceres 공식 예제 `pose_graph_3d`([pose_graph_3d_error_term.h](https://github.com/ceres-solver/ceres-solver/blob/master/examples/slam/pose_graph_3d/pose_graph_3d_error_term.h))를 줄인 발췌라 그대로는 컴파일되지 않는다. `Create()` 함수와 멤버 변수 선언은 뺐다.

**Error Term 정의:**

두 포즈 a, b(위치 `p_a`·`p_b`, 회전 `q_a`·`q_b`) 사이의 상대 변환 측정값 `t_ab_measured`가 있을 때, residual은 추정된 상대 변환과 측정값의 차이다.

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

    // 측정값과의 차이
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

- **template \<typename T\>**: Ceres는 residual 값만 필요하면 `T=double`로, Jacobian이 필요하면 dual number인 `T=Jet<double>`로 이 함수를 실행한다. Ceres의 AutoDiff는 이렇게 미분을 얻는다.
- **sqrt_information**: information matrix의 Cholesky decomposition. `information.llt().matrixL()`로 구한다.
- **AutoDiffCostFunction 차원**: `<PoseGraph3dErrorTerm, 6, 3, 4, 3, 4>` — residual 6차원, pos_a 3차원, quat_a 4차원, pos_b 3차원, quat_b 4차원.
- **SetManifold**: quaternion은 4차원이지만 자유도는 3이므로, `EigenQuaternionManifold`를 지정해서 manifold 위에서 최적화하도록 한다. 이전 API에서는 `LocalParameterization`이었다.

**문제 구성:**

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

```cpp
ceres::Solver::Options options;
options.max_num_iterations = 200;
options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
ceres::Solver::Summary summary;
ceres::Solve(options, &problem, &summary);
```

`SPARSE_NORMAL_CHOLESKY`는 pose graph처럼 sparse한 문제에 적합하다. Bundle adjustment처럼 제거할 점 변수 블록이 분명한 문제에서는 `SPARSE_SCHUR`도 고려할 수 있다.

**GTSAM vs Ceres 비교**

| | GTSAM | Ceres |
|---|---|---|
| 특성 | SLAM 특화 | 범용 nonlinear least squares |
| 기본 제공 | `BetweenFactor`, `PriorFactor` 등 미리 정의된 팩터 | SLAM용 팩터 없음. cost function 직접 정의 |
| 증분 최적화 | iSAM2로 incremental 풀이 가능 | iSAM2 같은 증분 풀이기 없음 |
| Manifold | Lie group 기본 지원 | `LocalParameterization` / `Manifold`로 직접 설정 |
| 적합한 상황 | SLAM 시스템 구축 | 유연한 구조가 필요할 때, 대규모 BA |

> **추천 자료**
> - [Ceres Solver 공식 pose_graph_3d 예제](https://ceres-solver.googlesource.com/ceres-solver/+/master/examples/slam/pose_graph_3d/) — 위 코드의 전체 버전
> - [Ceres Solver Tutorial](http://ceres-solver.org/tutorial.html) — AutoDiff, Manifold 개념 설명
> - [정진용 블로그 — Ceres Solver Tutorial](https://jinyongjeong.github.io/2023/07/22/Ceres_tutorial/) — Ceres Solver 발표자료와 GitHub 실습 코드. 비선형 최적화 입문에 적합

## 3.7 심화: Robust Estimation

잘못된 데이터 연관(false match), 동적 물체, 센서 고장이 outlier를 만든다. 제곱 오차 최적화에서는 outlier 하나가 결과를 크게 끌어당긴다. Robust estimation은 outlier의 영향을 줄이는 기법이다.

### 3.7.1 왜 필요한가

Standard least squares는 에러의 제곱을 최소화한다: `rho(r) = r^2`. 이 함수는 큰 잔차(residual)에 큰 가중치를 주기 때문에, 하나의 outlier가 전체 해를 끌고 갈 수 있다.

SLAM의 구체적 사례:
- 잘못된 loop closure 하나가 전체 지도를 뒤틀어 버린다
- Visual feature matching의 false positive가 BA 결과를 망친다
- 동적 물체(사람, 차)에 붙은 feature가 정적 장면 가정을 위반한다

### 3.7.2 M-Estimator

M-estimator는 `rho(r) = r^2` 대신 다른 비용 함수 rho를 써서 outlier의 영향을 줄인다.

| M-Estimator | rho(r) | 특성 |
|---|---|---|
| L2 (표준) | r^2 | Outlier에 취약 |
| Huber | r^2 (abs(r) <= k), 2k*abs(r) - k^2 (abs(r) > k) | 작은 잔차는 L2, 큰 잔차는 L1. 여러 최적화 라이브러리가 지원 |
| Cauchy | c^2 * log(1 + (r/c)^2) | Huber보다 outlier 억제가 강함 |
| Geman-McClure | r^2 / (1 + r^2) | 극단적 outlier를 사실상 무시 |

Huber가 안전한 기본 선택이다. Outlier 비율이 높거나 극단적이면 Cauchy나 Geman-McClure를 고려한다. 파라미터(k 또는 c)는 잔차의 통계적 분포에 맞춰 튜닝해야 한다.

실무적으로, Ceres Solver에서는 `ceres::HuberLoss`, `ceres::CauchyLoss` 등을 cost function에 감싸서 적용한다. GTSAM에서는 `gtsam::noiseModel::mEstimator::Huber`를 쓴다.

> **실습**: [M-Estimator 비교](https://alexjunholee.github.io/robotics-practice/app.html#m_estimator)
> L2, Huber, Cauchy, Geman-McClure 등 다양한 비용 함수가 outlier에 어떻게 반응하는지 인터랙티브하게 비교할 수 있다.

### 3.7.3 RANSAC와 변종

RANSAC (Random Sample Consensus)은 outlier가 포함된 데이터에서 모델을 피팅하는 반복적 알고리즘이다. M-estimator와 달리, 데이터를 inlier/outlier로 명시적으로 분류한다.

**기본 RANSAC 알고리즘:**
1. 최소 샘플을 무작위로 선택
2. 해당 샘플로 모델을 피팅
3. 전체 데이터에서 inlier 수를 계산 (threshold 이내의 잔차를 가진 점)
4. 반복 -> 가장 많은 inlier를 가진 모델을 선택
5. 최종적으로 모든 inlier를 사용해 모델을 re-fit

**변종들:**

| 변종 | 핵심 아이디어 | 트레이드오프 |
|---|---|---|
| RANSAC (기본) | 무작위 샘플 → 반복 | 단순하고 구현이 쉽지만 threshold·반복 횟수에 민감 |
| PROSAC | matching score로 좋은 샘플을 먼저 시도 | 빠르게 수렴하지만 사전 품질 정보의 질에 의존 |
| Lo-RANSAC | 좋은 모델 발견 시 로컬 최적화 추가 | 정확도 향상. 원 논문에서는 표본 수가 줄어 2~3배 빨랐다 |
| MAGSAC++ | 노이즈 스케일 sigma를 하나로 정하지 않고 범위에 걸쳐 marginalize, soft inlier/outlier | 파라미터 프리에 가까우나 계산 비용이 높음 |

OpenCV의 `cv::findHomography`, `cv::findFundamentalMat` 등에서 `cv::USAC_MAGSAC` 플래그로 MAGSAC++를 쓸 수 있다.

> **추천 자료**
> - [State Estimation for Robotics, Ch.5 — Biases, Correspondences, and Outliers (Barfoot)](https://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) — 바이어스, 대응 문제, outlier를 다루는 실전적 챕터
> - [Hartley & Zisserman, Ch.4 — Estimation: 2D Projective Transforms](https://www.robots.ox.ac.uk/~vgg/hzbook/) — RANSAC(원 제안은 Fischler & Bolles, 1981)과 robust estimation 이론을 정리한 교재 설명
> - [다크 프로그래머 — RANSAC의 이해와 영상처리 활용](https://darkpgmr.tistory.com/61) — RANSAC의 원리, threshold 설정, 반복 횟수 계산을 한글로 설명
> - [정진용 블로그 — Bundle Adjustment의 Jacobian 계산](https://jinyongjeong.github.io/2020/03/01/Jacobian_of_BA/) — BA의 reprojection error Jacobian을 Lie algebra와 quaternion으로 유도. 손필기 수식 포함

> **실습**: [RANSAC 시각화](https://alexjunholee.github.io/robotics-practice/app.html#ransac)
> Outlier가 포함된 데이터에서 RANSAC이 inlier/outlier를 분류하고 모델을 피팅하는 과정을 단계별로 확인할 수 있다.

## 3.8 심화: 정보 이론 기초

Active SLAM, exploration, 불확실성 기반 의사결정에서 정보 이론 개념이 쓰인다.

**Shannon entropy**: 확률 변수 X의 불확실성을 측정한다.

```
H(X) = -sum  p(x) log p(x)
```

Entropy가 높을수록 불확실성이 크다. 가우시안 분포의 경우 공분산이 클수록 entropy가 높다.

**KL divergence (Kullback-Leibler divergence)**: 두 확률 분포 p와 q 사이의 "차이"를 측정한다.

```
D_KL(p || q) = sum  p(x) log(p(x) / q(x))
```

비대칭이다: D_KL(p||q) != D_KL(q||p). p를 q로 근사할 때 생기는 정보 손실로 해석할 수 있다.

**Mutual information**: Y를 관측하면 X에 대해 얼마나 알게 되는가를 측정한다.

```
I(X; Y) = H(X) - H(X|Y)
```

H(X)는 Y를 관측하기 전 X의 불확실성, H(X|Y)는 관측 후 불확실성. 그 차이가 Y가 X에 대해 주는 정보량이다.

**Active SLAM 응용**: 로봇이 다음에 어디로 갈지 결정할 때, "이 행동을 취하면 지도/포즈의 불확실성이 얼마나 줄어드는가?"를 mutual information으로 수치화할 수 있다. 정보 이론 기반 탐색은 expected information gain이 가장 큰 행동을 고른다.

```
a* = argmax_a  I(X; Z_a)  =  argmax_a  [ H(Z_a) - H(Z_a | X) ]
```

여기서 a는 행동(action), Z_a는 그 행동으로 얻을 관측, X는 환경 상태이다.

> **추천 자료**
> - [Elements of Information Theory (Cover & Thomas)](https://onlinelibrary.wiley.com/doi/book/10.1002/047174882X) — 정보 이론 교과서
> - Placed et al., "A Survey on Active Simultaneous Localization and Mapping: State of the Art and New Frontiers" (IEEE T-RO 2023) — Active SLAM에서 정보 이론이 쓰이는 방식을 정리한 서베이

> **기술 흐름: 로보틱스 수학 및 최적화**
> - **~2005**: 칼만 필터(EKF) 중심의 상태 추정. 선형 근사 기반, 소규모 문제에 적합. 실시간 처리가 어려워 문제 크기에 제약이 있었다.
> - **2006~2015**: Factor Graph 기반 최적화(iSAM, g2o, GTSAM) 등장. 스파스 행렬 구조를 활용해 대규모 SLAM 문제를 효율적으로 풀었다. Lie Group/Algebra가 SLAM 커뮤니티에서 표준 도구로 자리잡았다.
> - **2016~2019**: VIO와 LiDAR odometry에서 sliding-window 최적화와 증분 최적화(incremental optimization)가 실시간 시스템의 기본 구성이 되었다. Ceres Solver 같은 공개 비선형 최소자승 라이브러리가 연구와 산업 응용에 널리 쓰였다.
> - **2020~**: Differentiable Programming 시대. PyTorch/JAX의 자동 미분(Auto-Diff)을 활용한 End-to-End 최적화. NeRF(ECCV 2020), 3D Gaussian Splatting(SIGGRAPH 2023) 등 미분 가능 렌더링이 등장하면서, 기존에 손으로 유도하던 Jacobian을 자동 미분으로 대체했다. Theseus(Meta) 같은 미분 가능 최적화 라이브러리도 나왔다.

---

## 3.9 심화: 베이즈 필터 (Bayes Filter)

원전: Thrun, Burgard, Fox (2005) *Probabilistic Robotics*, Ch.2 (Recursive State Estimation).
(국문: 『확률론적 로보틱스』, 에이콘, 2020)

§3.3.2의 베이즈 정리를 시간 축 위에서 재귀적으로 적용하면 로봇이 지금 어디 있는지를 관측이 들어올 때마다 갱신하는 필터가 된다. §3.6 Factor Graph도 연속 포즈 사이의 odometry·IMU factor로 시간 관계를 담는다. 베이즈 필터는 과거 상태를 marginalize해 현재 belief만 남긴다(filtering).

### 3.9.1 상태와 Markov 가정

상태 $x_t$ 는 로봇과 환경의 미래 예측에 필요한 모든 정보를 담은 변수 묶음이다. "완전 상태(complete state)"란 그 자체만으로 미래를 예측하기에 충분한 요약(sufficient statistic)이다. 이 성질이 곧 Markov 성질이다.

$$p(x_{t+1} \mid x_t,\, x_{0:t-1},\, z_{1:t},\, u_{1:t+1}) = p(x_{t+1} \mid x_t,\, u_{t+1})$$

완전 상태 가정에서는 현재 $x_t$를 알면 과거 정보가 미래 예측에 보태는 것이 없다.

상태 변수는 다양한 방식으로 분류된다. 시간에 따라 변하는 동적 상태(로봇 위치, 속도)와 고정된 정적 상태(벽 위치, 랜드마크)로 나뉜다. 값의 형태에 따라 연속 상태(pose), 이산 상태(센서 고장 여부), 하이브리드 상태(둘의 결합) 등으로 세분화할 수 있다. 실제 필터의 상태는 완전 상태의 근사다. 모델의 부정확성, 미모델링 동역학, 수치 근사 오차가 Markov 가정을 깬다.

### 3.9.2 환경 상호작용: 측정과 제어

로봇과 환경의 상호작용은 두 데이터 스트림으로 분해된다.

- 측정 데이터 $z_t$: 환경이 로봇에 주는 정보 (LiDAR 거리, 카메라 이미지). 시간 구간 $(t-1, t]$에서 로봇의 지식을 늘린다.
- 제어 데이터 $u_t$: 로봇이 환경에 가하는 행동 (모터 명령). 제어에 따른 상태 예측에서는 운동 잡음 때문에 상태 불확실성이 증가할 수 있다.

$$z_{t_1:t_2} = z_{t_1},\, z_{t_1+1},\, \ldots,\, z_{t_2} \qquad u_{t_1:t_2} = u_{t_1},\, \ldots,\, u_{t_2}$$

**odometry는 제어 데이터로 취급한다.** 휠 인코더는 물리적으로 센서지만 상태 변화 정보(로봇이 얼마나 이동했는가)를 담으므로 $u_t$로 분류된다. 로봇이 아무것도 하지 않아도 상태는 대개 변하므로, PR은 일관성을 위해 정지(do_nothing)도 제어로 둔다.

### 3.9.3 Belief의 정의

Belief는 실제(true) 상태 $x_t$의 사후 분포로, 로봇이 내부에 들고 있는 추정이다.

$$\text{bel}(x_t) = p(x_t \mid z_{1:t},\, u_{1:t})$$

측정 $z_t$를 반영하기 전의 예측 belief를 별도로 표기한다.

$$\overline{\text{bel}}(x_t) = p(x_t \mid z_{1:t-1},\, u_{1:t})$$

$\overline{\text{bel}} \to \text{bel}$ 전환을 correction 또는 **measurement update**라 부른다. PR은 GPS로도 pose를 직접 측정할 수는 없다고 지적한다(PR §2.3.4). belief는 늘 측정에서 추론한 결과다.

### 3.9.4 생성 법칙: 모션 모델과 측정 모델

완전 상태 가정으로 두 조건부 독립이 성립한다.

$$p(x_t \mid x_{0:t-1},\, z_{1:t-1},\, u_{1:t}) = p(x_t \mid x_{t-1},\, u_t) \quad \text{(모션 모델)}$$

$$p(z_t \mid x_{0:t},\, z_{1:t-1},\, u_{1:t}) = p(z_t \mid x_t) \quad \text{(측정 모델)}$$

$x_{t-1}$이 과거 모든 데이터의 충분 통계량이므로, 다음 상태는 직전 상태와 직전 제어에만, 측정은 현재 상태에만 의존한다. 시간 불변(time-invariant) 가정 시 $p(x' \mid u, x)$와 $p(z \mid x)$로 축약된다.

완전 생성 모델 = 모션 모델 + 측정 모델 + 초기 분포 $p(x_0)$. 이 구조가 곧 Hidden Markov Model / Dynamic Bayes Network다.

### 3.9.5 베이즈 필터 일반형

베이즈 필터는 동적 상태 추정을 재귀적으로 쓰는 일반적인 틀이다. prediction 단계와 correction 단계를 반복한다.

$$\overline{\text{bel}}(x_t) = \int p(x_t \mid u_t,\, x_{t-1})\, \text{bel}(x_{t-1})\, dx_{t-1} \tag{prediction}$$

$$\text{bel}(x_t) = \eta\, p(z_t \mid x_t)\, \overline{\text{bel}}(x_t) \tag{correction}$$

$\eta$는 정규화 상수로, 전확률 정리에서 나오는 §3.3.2의 베이즈 분모($P(B)$)의 역수에 해당한다.

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

이산 상태공간에서는 적분이 합(summation)이 된다. 초기 belief $\text{bel}(x_0)$도 필요하다. 초기 상태를 정확히 알면 점질량으로, 전혀 모르면 균등 분포로 둔다. 이 일반형은 닫힌 형식 적분이 가능하거나 이산 공간이 충분히 작을 때만 직접 구현할 수 있다. §3.10의 칼만 필터, §3.11의 입자 필터는 각각 다른 방식으로 이 일반형을 근사한다.

### 3.9.6 문 추정 워킹 예제

두 상태(열림/닫힘) 문에 대해 belief가 어떻게 갱신되는지 손으로 추적한다.

**모델 설정:**

- 측정 모델: $p(\text{sense\_open} \mid \text{is\_open}) = 0.6$, $p(\text{sense\_open} \mid \text{is\_closed}) = 0.2$
- 모션 모델 push: 열려있으면 그대로(확률 1), 닫혀있으면 확률 0.8로 열림
- 모션 모델 do_nothing: 결정론적 항등 (상태 불변)
- 초기: $\text{bel}(X_0 = \text{open}) = \text{bel}(X_0 = \text{closed}) = 0.5$

**단계 1: $u_1$ = do_nothing (제어 적용)**

do_nothing은 항등 변환이므로 $\overline{\text{bel}}(X_1) = (0.5,\; 0.5)$로 변화 없다.

**단계 2: $z_1$ = sense_open (측정 반영)**

$$\overline{\text{bel}}(X_1 = \text{open}) = 0.5, \quad p(\text{sense\_open} \mid \text{open}) = 0.6$$
$$\overline{\text{bel}}(X_1 = \text{closed}) = 0.5, \quad p(\text{sense\_open} \mid \text{closed}) = 0.2$$

비정규화 사후: $(0.6 \times 0.5,\; 0.2 \times 0.5) = (0.30,\; 0.10)$. 정규화 상수 $\eta = 1/(0.30 + 0.10) = 2.5$.

$$\text{bel}(X_1 = \text{open}) = 0.75, \quad \text{bel}(X_1 = \text{closed}) = 0.25$$

**단계 3: $u_2$ = push (제어 적용)**

$$\overline{\text{bel}}(X_2 = \text{open}) = 1 \cdot 0.75 + 0.8 \cdot 0.25 = 0.95$$
$$\overline{\text{bel}}(X_2 = \text{closed}) = 0 \cdot 0.75 + 0.2 \cdot 0.25 = 0.05$$

**단계 4: $z_2$ = sense_open (측정 반영)**

비정규화: $(0.6 \times 0.95,\; 0.2 \times 0.05) = (0.570,\; 0.010)$. $\eta = 1/0.580 \approx 1.724$.

$$\text{bel}(X_2 = \text{open}) \approx 0.983, \quad \text{bel}(X_2 = \text{closed}) \approx 0.017$$

| 단계 | $\text{bel(open)}$ | $\text{bel(closed)}$ |
|------|:-----------------:|:-------------------:|
| 초기 | 0.500 | 0.500 |
| $z_1$ 반영 후 | 0.750 | 0.250 |
| $u_2$ 적용 후 | 0.950 | 0.050 |
| $z_2$ 반영 후 | 0.983 | 0.017 |

센서 잡음이 크고(60% / 20%) 제어가 비결정적인 이 예제에서도, 측정과 제어가 쌓이면서 belief가 한 가설로 빠르게 모인다. 0.983이 의사결정에 충분한지는 응용의 위험 기준으로 따로 정한다.

### 3.9.7 수학적 유도

베이즈 필터의 두 갱신식은 Bayes 규칙, 전확률 정리, Markov(완전 상태) 가정으로 유도한다.

**Correction step 유도:**

Bayes 규칙 적용:
$$p(x_t \mid z_{1:t},\, u_{1:t}) = \eta\, p(z_t \mid x_t,\, z_{1:t-1},\, u_{1:t})\, p(x_t \mid z_{1:t-1},\, u_{1:t})$$

완전 상태 가정으로 $p(z_t \mid x_t,\, z_{1:t-1},\, u_{1:t}) = p(z_t \mid x_t)$이므로:
$$\text{bel}(x_t) = \eta\, p(z_t \mid x_t)\, \overline{\text{bel}}(x_t)$$

**Prediction step 유도:**

전확률 정리로 $\overline{\text{bel}}$ 분해:
$$\overline{\text{bel}}(x_t) = \int p(x_t \mid x_{t-1},\, z_{1:t-1},\, u_{1:t})\, p(x_{t-1} \mid z_{1:t-1},\, u_{1:t})\, dx_{t-1}$$

완전 상태 가정으로 첫 항 $\to p(x_t \mid x_{t-1},\, u_t)$. 둘째 항에서 $u_t$는 $x_{t-1}$보다 늦게 도달하므로 제거할 수 있다:
$$\overline{\text{bel}}(x_t) = \int p(x_t \mid u_t,\, x_{t-1})\, \text{bel}(x_{t-1})\, dx_{t-1}$$

이 유도 전체가 Markov 가정에 의존한다. Markov 가정이 깨지면 식 자체가 부정확해진다.

> **추천 자료**
> - [Thrun, Burgard, Fox — Probabilistic Robotics (2005)](http://probabilistic-robotics.org/) — Ch.2 전체가 이 절의 원전. 알고리즘, 예제, 유도가 모두 실려 있다.
> - [Cyrill Stachniss — Bayes Filter Lecture (YouTube)](https://www.youtube.com/watch?v=0lKHFJpaZvE) — Cyrill Stachniss의 베이즈 필터 강의(2020)

---

## 3.10 심화: 가우시안 필터 계열 (Gaussian Filters: KF, EKF, IF)

원전: Thrun, Burgard, Fox (2005) *Probabilistic Robotics*, Ch.3 (Gaussian Filters).
(국문: 『확률론적 로보틱스』, 에이콘, 2020)

§3.9 베이즈 필터의 적분은 일반적으로 닫힌 형식이 없다. 가우시안 필터는 belief를 가우시안 $\mathcal{N}(\mu_t, \Sigma_t)$로 제한해 적분을 닫힌 형식으로 푼다. 칼만 필터(KF), 확장 칼만 필터(EKF), 정보 필터(IF)가 이 범주에 속하며, 셋 다 §3.9의 prediction-correction 구조를 따른다.

### 3.10.1 칼만 필터 (Kalman Filter)

#### 선형 가우시안 시스템 가정

KF가 정확한 베이즈 필터가 되려면 belief가 항상 가우시안이어야 한다. 이를 보장하는 세 가정이 필요하다.

**상태 천이(모션 모델):**
$$x_t = A_t x_{t-1} + B_t u_t + \varepsilon_t, \quad \varepsilon_t \sim \mathcal{N}(0, R_t)$$

$A_t$는 $n \times n$ 상태 천이 행렬, $B_t$는 $n \times m$ 제어 입력 행렬, $R_t$는 프로세스 노이즈 공분산이다.

**측정 모델:**
$$z_t = C_t x_t + \delta_t, \quad \delta_t \sim \mathcal{N}(0, Q_t)$$

$C_t$는 $k \times n$ 측정 행렬, $Q_t$는 측정 노이즈 공분산이다.

**초기 belief:**
$$\text{bel}(x_0) = \mathcal{N}(\mu_0, \Sigma_0)$$

이 세 가정 아래, 모든 시점의 belief는 가우시안으로 유지된다. 즉:
$$p(x_t \mid u_t, x_{t-1}) = \mathcal{N}(x_t;\; A_t x_{t-1} + B_t u_t,\; R_t)$$
$$p(z_t \mid x_t) = \mathcal{N}(z_t;\; C_t x_t,\; Q_t)$$

#### 칼만 필터 알고리즘

KF는 belief를 $(\mu_t, \Sigma_t)$로 표현하고, prediction 두 줄과 correction 세 줄로 한 사이클을 돈다.

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

라인 1~2가 prediction (제어 $u_t$ 반영, 불확실성 증가), 라인 3~5가 measurement update (관측 $z_t$ 반영, 불확실성 감소)다.

**칼만 이득 $K_t$의 의미:** $K_t$는 prediction과 측정 사이의 신뢰 균형을 결정한다. 측정 노이즈 $Q_t$가 크면 $K_t$가 작아져 측정을 덜 신뢰하고, prediction 불확실성 $\bar\Sigma_t$가 크면 $K_t$가 커져 측정을 더 신뢰한다.

**혁신(innovation):** $z_t - C_t \bar\mu_t$는 예측된 측정과 실제 측정의 차이로, 이 값이 0이면 평균은 그대로다. 공분산 갱신(라인 5)은 innovation 값과 상관없이 일어난다.

#### 1D KF 도해: 정보가 어떻게 결합되는가

1D 위치 추정에서 KF 각 단계의 분포 모양을 따라가 본다.

- **Prior $\text{bel}(x_{t-1})$**: 좁은 가우시안. 이전 추정의 확신.
- **Prediction 후**: 모션이 더해지며 분산이 증가한다 ($\bar\Sigma_t = A_t \Sigma_{t-1} A_t^T + R_t$). 가우시안이 납작해진다.
- **측정 $z_t$**: 별도의 가우시안으로 표현. 센서 정밀도 $Q_t$가 이 곡선의 폭을 결정한다.
- **Correction 후**: 두 가우시안을 곱하면 분산이 둘 다보다 좁아진다 — 정보 결합 효과다. 평균은 두 평균을 분산의 역수로 가중 평균한 위치에 놓인다.
- 그 다음 모션: 다시 분산 증가. 그 다음 측정: 다시 분산 감소.

상태 추정은 분산이 커지는 prediction과 분산이 줄어드는 correction을 번갈아 반복한다. 같은 해석은 §3.10.2 EKF, §3.11.3 입자 필터, Ch.14 §14.7.3 EKF localization에도 적용된다.

<!-- DEMO: kalman_1d_illustration.html -->

#### KF의 수학적 유도 (핵심)

KF 5줄은 §3.9.5의 베이즈 필터 두 갱신식을 선형 가우시안 가정 아래 닫힌 형식으로 푼 결과다.

**Part 1 (Prediction).** 베이즈 필터의 prediction 적분에서, 지수 $L_t$가 $x_{t-1}$과 $x_t$의 quadratic임을 확인한다. $L_t$를 $x_{t-1}$의 quadratic 부분과 $x_t$에만 의존하는 부분으로 분해하면 $x_{t-1}$ 적분은 상수가 되어 정규화에 흡수된다. 남은 $x_t$ quadratic의 1차·2차 계수에서 바로 $\bar\mu_t = A_t \mu_{t-1} + B_t u_t$와 $\bar\Sigma_t = A_t \Sigma_{t-1} A_t^T + R_t$가 읽힌다.

**Part 2 (Measurement update).** Correction 식 $\text{bel}(x_t) \propto \exp\{-J_t\}$에서, $J_t$의 1차·2차 도함수로부터 $\Sigma_t^{-1} = C_t^T Q_t^{-1} C_t + \bar\Sigma_t^{-1}$을 얻는다. 이를 직접 역행렬하면 $n \times n$ 연산이 필요하지만, **inversion lemma** (Woodbury identity)로:
$$K_t = \bar\Sigma_t C_t^T (C_t \bar\Sigma_t C_t^T + Q_t)^{-1}$$
처럼 $k \times k$ ($k$ = 측정 차원) 역행렬로 치환할 수 있다. $k \ll n$이면 계산 비용이 크게 줄어든다.

**복잡도:** 한 사이클 $O(k^{2.8} + n^2)$ ($k$: 측정 차원, $n$: 상태 차원).

§3.10.2 EKF도 같은 quadratic 분해와 inversion lemma로 유도한다. $\Sigma_t^{-1}$에 측정 항을 더하는 구조는 §3.6 factor graph의 Gauss-Newton 정규방정식과 같다.

### 3.10.2 확장 칼만 필터 (Extended Kalman Filter)

#### 비선형 시스템으로 확장

실제 로봇 시스템은 대부분 비선형이다. 로봇이 회전하며 이동하는 모션 모델 $g$와 거리 센서의 측정 모델 $h$는 모두 비선형이다.

$$x_t = g(u_t, x_{t-1}) + \varepsilon_t, \quad \varepsilon_t \sim \mathcal{N}(0, R_t)$$
$$z_t = h(x_t) + \delta_t, \quad \delta_t \sim \mathcal{N}(0, Q_t)$$

비선형 $g$를 통과한 가우시안은 일반적으로 가우시안이 아니다. EKF는 이 문제를 1차 Taylor 전개로 해결한다. 이전 사후 평균 주위에서 $g$를, 예측 평균 주위에서 $h$를 선형화하여 가우시안 근사를 유지한다.

$$g(u_t, x_{t-1}) \approx g(u_t, \mu_{t-1}) + G_t (x_{t-1} - \mu_{t-1})$$

$$G_t := \frac{\partial g(u_t, x_{t-1})}{\partial x_{t-1}}\bigg|_{\mu_{t-1}} \quad (n \times n \text{ Jacobian})$$

$$h(x_t) \approx h(\bar\mu_t) + H_t (x_t - \bar\mu_t)$$

$$H_t := \frac{\partial h(x_t)}{\partial x_t}\bigg|_{\bar\mu_t} \quad (k \times n \text{ Jacobian})$$

선형화 품질은 함수의 비선형도와 belief의 폭에 달려 있다. 분산이 클수록 접평면 근사가 빨리 무너지므로 EKF는 분산이 작을 때 잘 작동한다.

#### EKF 알고리즘

KF의 5줄에서 선형 항을 비선형 함수와 그 Jacobian으로 교체하기만 하면 EKF가 된다.

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

KF와 EKF의 차이는 두 줄이다: (라인 1) $A_t \mu_{t-1} + B_t u_t \to g(u_t, \mu_{t-1})$, (라인 4) $C_t \bar\mu_t \to h(\bar\mu_t)$. 공분산 전파에서는 $A_t \to G_t$, $C_t \to H_t$로 Jacobian이 대신 들어간다.

#### 유도 요약 및 실무 비교

유도는 §3.10.1 KF와 평행하다. 비선형 $g$와 $h$를 1차 Taylor로 치환한 뒤, 같은 quadratic 분해 + inversion lemma 절차를 돌리면 EKF 식이 나온다. 결과:
$$\bar\mu_t = g(u_t, \mu_{t-1}), \quad \bar\Sigma_t = G_t \Sigma_{t-1} G_t^T + R_t$$
$$\mu_t = \bar\mu_t + K_t (z_t - h(\bar\mu_t)), \quad \Sigma_t = (I - K_t H_t) \bar\Sigma_t, \quad K_t = \bar\Sigma_t H_t^T (H_t \bar\Sigma_t H_t^T + Q_t)^{-1}$$

**실무 비교:**

EKF는 SLAM·VIO·IMU 융합에서 오래 쓰였고, MSCKF 계열 VIO처럼 지금도 쓰인다. 다음 변형도 함께 쓰인다.

- **UKF (Unscented KF):** sigma point로 분포를 비선형 함수에 통과시켜 1차 선형화보다 정확하게 전파한다. Jacobian을 손으로 구할 필요가 없다.
- **IEKF (Iterated EKF):** 선형화 지점을 갱신하며 Jacobian을 반복해서 다시 계산한다. 비선형성이 강할 때 EKF보다 정확하다.
- **LIEKF (Left-Invariant EKF):** SO(3)/SE(3) 상태의 오차를 Lie group 위의 invariant error로 정의하고 그 오차를 선형화한다.

Ch.14 §14.7.3 EKF localization과 IEKF·MSCKF의 필터 구조는 이 알고리즘 박스와 연결된다. §14.10의 IMU preintegration은 키프레임 사이의 IMU 측정을 최적화 factor로 묶는 별도의 측정 구성 방법이다. 여기서 $g$, $h$, $G_t$, $H_t$가 무엇인지 이해하면, Ch.14의 구체적인 필터와 센서 모델에도 이 골격을 그대로 쓴다.

### 3.10.3 정보 필터 (Information Filter)

#### Canonical 표현: $(\Omega, \xi)$

KF와 EKF는 가우시안을 $(\mu, \Sigma)$로 표현했다. 정보 필터는 평균·공분산 $(\mu, \Sigma)$ 대신 정보 행렬(information matrix) $\Omega$와 정보 벡터(information vector) $\xi$를 쓴다.

$$\Omega = \Sigma^{-1}, \quad \xi = \Sigma^{-1} \mu$$

역방향: $\Sigma = \Omega^{-1}$, $\mu = \Omega^{-1} \xi$.

이 좌표에서 가우시안의 음의 로그 우도는 상태 $x$에 대해 quadratic이다.

$$p(x) = \eta \exp\!\left\{-\tfrac{1}{2} x^T \Omega x + x^T \xi\right\}$$

$$-\log p(x) = \mathrm{const} + \tfrac{1}{2} x^T \Omega x - x^T \xi$$

최솟값은 $\Omega x = \xi$, 즉 $x = \Omega^{-1} \xi = \mu$. 이것이 §3.4.3 Gauss-Newton의 정규 방정식 $H \delta x = -b$($H = J^T J$, $b = J^T r$)와 정확히 같은 구조다.

$\Omega = 0$은 정보가 전혀 없는 상태(완전 불확실, 균등 분포)다. Moments(평균·공분산) 표현에서는 $\Sigma = \infty$가 되어 쓸 수 없는 상태를 정보 표현은 $\Omega = 0$으로 그대로 쓴다.

#### 정보 필터 알고리즘

정보 필터에서는 measurement update가 덧셈이 되고 prediction에 역행렬이 들어간다. 계산 부담이 KF와 정반대로 놓여 둘을 쌍대(dual)라 부른다.

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

**KF와 IF의 복잡도 쌍대성:**

| 단계 | KF | IF |
|------|:---:|:---:|
| Prediction | $O(n^2)$ 가산형 | $O(n^{2.8})$ 역행렬 2회 |
| Measurement update | $O(k^{2.8})$ 역행렬 필요 | $O(n^2)$ 가산형 |

$k$: 측정 차원, $n$: 상태 차원. 측정이 부분 상태만 건드리면(sparse $C_t$) IF의 measurement update는 더 저렴해진다.

#### EIF (Extended Information Filter)

EKF처럼 비선형 $g, h$의 Jacobian $G_t, H_t$를 써서 prediction의 $A_t$를 $G_t$로, correction의 $C_t$를 $H_t$로 바꾸면 EIF가 된다(PR Table 3.5).

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

정보 필터의 measurement update $\Omega_t = \bar\Omega_t + C_t^T Q_t^{-1} C_t$는 측정마다 $\Omega$에 한 항을 더한다. 여러 로봇의 독립 측정도 $\Omega_{\text{total}} = \sum_i \Omega_i$, $\xi_{\text{total}} = \sum_i \xi_i$로 합산할 수 있다.

이 가산성은 §3.6 factor graph에서 "측정 factor 하나 = $H^T Q^{-1} H$와 $H^T Q^{-1} z$ 한 항 추가"로 일반화된다. 정보 행렬의 sparsity는 각 factor가 소수의 변수에만 연결되는 국소성에서 나온다. $H$의 비영 열이 적어 더해지는 블록이 희소해진다. sparse Cholesky가 실제로 희소성을 유지하는지는 소거 순서가 만드는 fill-in이 정한다.

EIF-SLAM과 SEIF는 이 정보형 가산성을 SLAM 표현에 쓴다. 역사적 연결은 Ch.14 §14.16에서 다룬다.

> **추천 자료**
> - [Thrun, Burgard, Fox — Probabilistic Robotics (2005)](http://probabilistic-robotics.org/) — Ch.3이 이 절의 원전. KF·EKF·IF 세 알고리즘의 유도가 나란히 서술되어 있다.
> - [Cyrill Stachniss 채널 — EKF Localization (Nived Chebrolu)](https://www.youtube.com/watch?v=PiCC-SxWlH8) — EKF localization 강의.
> - [Welch & Bishop — An Introduction to the Kalman Filter (2006)](https://web.archive.org/web/20250111082128/https://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf) — KF의 식과 직관을 함께 정리한 입문 자료.

---

## 3.11 심화: 비모수 필터 (Nonparametric Filters)

원전: Thrun, Burgard, Fox (2005) *Probabilistic Robotics*, Ch.4 (Nonparametric Filters).
(국문: 『확률론적 로보틱스』, 에이콘, 2020)

§3.10의 가우시안 필터는 belief를 $(\mu, \Sigma)$ 두 개로 압축한다. 그래서 강한 비선형성과 다봉(multi-modal) 분포에서 오차가 커진다. 비모수 필터는 더 많은 계산으로 임의 분포를 표현한다.

### 3.11.1 히스토그램 필터 / 이산 베이즈 필터

#### 유한 상태공간: 적분을 합으로

§3.9.5의 베이즈 필터 적분에 닫힌 형식이 없으면, 상태공간을 유한 개로 나누는 방법이 가장 직접적이다. 상태가 $K$개의 이산 값 $\{x_1, x_2, \ldots, x_K\}$ 중 하나라면, §3.9.5의 적분은 합이 된다.

$$\bar p_{k,t} = \sum_i p(x_k \mid u_t, x_i)\, p_{i,t-1} \quad \text{(prediction)}$$
$$p_{k,t} = \eta\, p(z_t \mid x_k)\, \bar p_{k,t} \quad \text{(correction)}$$

```
# Algorithm Discrete_Bayes_filter (Table 4.1 의역)
# 입력: {p_{k,t-1}}, u_t, z_t
# 출력: {p_{k,t}}

for all k do
    # prediction: 모든 이전 상태에서 x_k로의 전이 합산
    p̄_{k,t} = Σ_i p(X_t = x_k | u_t, X_{t-1} = x_i) · p_{i,t-1}

    # correction: 측정 likelihood로 가중치 부여
    p_{k,t} = η · p(z_t | X_t = x_k) · p̄_{k,t}
endfor
return {p_{k,t}}
```

이 알고리즘은 음성인식의 HMM forward algorithm과 구조가 같다. 상태공간이 원래 이산인 문제(문 열림/닫힘, semantic class 분류)에서는 지금도 가장 간단한 선택이다.

#### 연속 상태: 히스토그램 필터

연속 상태공간을 유한개 영역 $\{\mathbf{x}_{k,t}\}$로 분할하고, 각 영역 안에서 belief가 균일(piecewise uniform)하다고 가정한다.

$$p(x_t) = \frac{p_{k,t}}{|\mathbf{x}_{k,t}|} \quad x_t \in \mathbf{x}_{k,t}$$

영역 대표값(평균 상태) $\hat x_{k,t}$로 모델을 근사한다.

$$p(z_t \mid \mathbf{x}_{k,t}) \approx p(z_t \mid \hat x_{k,t})$$
$$p(\mathbf{x}_{k,t} \mid u_t, \mathbf{x}_{i,t-1}) \approx \eta\,|\mathbf{x}_{k,t}|\, p(\hat x_{k,t} \mid u_t, \hat x_{i,t-1})$$

영역 크기가 모두 같으면 $|\mathbf{x}_{k,t}|$ 인수는 정규화에 흡수된다. 이렇게 만든 이산 베이즈 필터를 **히스토그램 필터**라 부른다.

**한계:** 격자 수가 차원에 따라 지수적으로 늘어나 6-DoF pose 추정에는 맞지 않는다. 계산을 줄이는 방법으로 density tree(사후 확률이 낮은 영역을 거칠게 나누는 동적 분할)와 selective updating(확률이 무시할 수 없는 셀만 갱신)이 있다. PR은 selective updating으로 3차원 이상 공간에서도 격자 분해를 쓸 수 있다고 쓴다(PR §4.1.3).

Ch.14 §14.7B occupancy grid mapping도 공간을 격자로 나누고, 셀마다 §3.11.2의 이진 베이즈 필터를 돌린다.

### 3.11.2 이진 베이즈 필터 (Binary Bayes Filter, Log-Odds 형식)

#### 정적 상태의 이진 추정

시간에 변하지 않는 이진 상태(예: "이 셀이 점유되었는가?")를 추정할 때, 상태 천이 모델이 없으므로 prediction 단계가 사라진다. Correction 단계만 반복하면 된다.

그런데 매 측정마다 사후 확률 $p(x \mid z_{1:t})$를 직접 계산하면, likelihood 곱셈이 누적되면서 수치 언더플로우 위험이 있다. 또 [0, 1] 구간 절단도 처리해야 한다. Log-odds 표현이 이를 해결한다.

$$l(x) := \log \frac{p(x)}{1 - p(x)} \in (-\infty, +\infty)$$

Log-odds의 치역은 실수 전체 $(-\infty, +\infty)$이므로 0과 1 근처의 절단 문제가 생기지 않는다. 곱셈으로 하던 Bayes 갱신도 덧셈이 된다.

$$l_t = l_{t-1} + \log\frac{p(x \mid z_t)}{1 - p(x \mid z_t)} - l_0$$

여기서 $l_0 = \log\frac{p(x)}{1-p(x)}$는 prior log-odds다.

Belief 복원: $\text{bel}_t(x) = 1 - \dfrac{1}{1 + \exp(l_t)}$

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

순방향 측정 모델 $p(z \mid x)$를 역방향으로 뒤집은 $p(x \mid z)$를 **inverse sensor model**이라 한다. 카메라 이미지 z에서 문이 열렸을 확률 $p(\text{open} \mid z)$를 정하는 편이, 문이 열렸을 때 나올 이미지 분포 $p(z \mid \text{open})$를 정하는 것보다 쉽다(PR §4.1.4). 이진 베이즈 필터는 이 inverse model을 직접 입력으로 받는다.

Ch.14 Occupancy Grid Mapping은 이 log-odds 갱신식을 셀마다 그대로 쓴다.

### 3.11.3 입자 필터 (Particle Filter)

#### 비모수 표현의 원리

히스토그램 필터의 격자는 차원이 늘면 지수적으로 커진다. 입자 필터는 격자 대신 표본으로 분포를 근사해 이 문제를 우회한다.

입자 필터는 belief를 $M$개의 무작위 표본(입자)으로 표현한다.

$$\mathcal{X}_t = \{x_t^{[1]},\, x_t^{[2]},\, \ldots,\, x_t^{[M]}\}$$

입자 $x_t^{[m]}$들은 belief가 높은 곳에 더 밀집한다. 가우시안 가정 없이 임의 형태의 분포 — 다봉 분포, 긴 꼬리 분포 — 를 표현할 수 있다.

#### 입자 필터 알고리즘: sampling → weighting → resampling

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

이상적으로 $M \to \infty$에서 $x_t^{[m]} \sim p(x_t \mid z_{1:t}, u_{1:t})$로 수렴한다.

#### 중요도 샘플링(Importance Sampling) 직관

목표 분포 $f$에서 직접 샘플링이 어려울 때, proposal 분포 $g$에서 뽑고 가중치 $w = f/g$로 보정한다.

$$w^{[m]} = \frac{f(x^{[m]})}{g(x^{[m]})}$$

가중 경험분포는 임의 Borel 집합 $A$에 대해:
$$\left[\sum_{m=1}^M w^{[m]}\right]^{-1} \sum_{m=1}^M \mathbf{1}(x^{[m]} \in A)\, w^{[m]} \;\longrightarrow\; \int_A f(x)\, dx$$

수렴률은 $O(1/\sqrt{M})$이고, proposal과 target이 비슷할수록 상수가 작아진다.

입자 필터에서 proposal은 모션 모델 $p(x_t \mid u_t, x_{t-1})$로 각 입자를 전개한다. target은 측정까지 반영한 $\text{bel}(x_t)$다. 측정 $z_t$를 반영하지 않은 proposal과 반영한 target 사이의 "빠진 정보"가 $p(z_t \mid x_t^{[m]})$이다. Step 2는 이 값을 가중치로 쓴다.

#### 수렴 및 구현

엄밀한 유도는 입자를 상태 시퀀스 $x_{0:t}^{[m]}$로 보면 간단해진다. Target의 시퀀스 분해(Bayes + Markov 두 번)와 proposal의 귀납 분해의 비율이 $\eta\, p(z_t \mid x_t^{[m]})$으로 정확하게 떨어진다. 가중 입자 집합이 target 분포와 같아지는 것은 $M \to \infty$의 극한에서다.

Step 3의 resampling은 가중치가 소수의 입자에 몰리는 weight degeneracy를 막는다.

<!-- DEMO: particle_filter_1d.html -->

### 3.11.4 입자 필터의 4가지 오차 원천

입자 필터는 근사이므로 구조적 오차가 있다. PR §4.2.4는 그 원천을 다음처럼 나눈다.

#### (1) 유한 $M$의 체계적 편향 (Systematic Bias)

$M = 1$이면 weight가 자기 자신으로 정규화되어 $w/w = 1$이 되고, 필터는 측정을 전혀 반영하지 않은 $p(x_t \mid u_{1:t})$에서 입자를 뽑는다. 일반적으로 정규화 전 weight는 $M$차원 공간에 있지만 정규화 뒤에는 $M-1$차원 공간에 놓여 자유도 하나를 잃는다. $M$이 클수록 이 효과는 약해지고, 계산 비용은 $M$에 비례해 늘어난다.

#### (2) Resampling으로 인한 다양성 상실 (Sample Impoverishment)

정적 상태($x_t = x_{t-1}$)에서 전이 잡음을 0으로 두면 입자는 각자의 초기 상태에 머문다. 이 상태로 재표집을 반복하면 표집 분산 때문에 살아남는 입자의 종류가 줄어들어(sample impoverishment) 결국 단일 상태로 붕괴된다.

완화: 로봇이 정지하면 resampling을 보류한다. 또는 weight variance가 높을 때만 resampling하고, 나머지 시간에는 weight를 곱셈적으로 누적한다.

$$w_t^{[m]} = \begin{cases} 1 & \text{(resampling 직후)} \\ p(z_t \mid x_t^{[m]})\, w_{t-1}^{[m]} & \text{(resampling 없을 때)} \end{cases}$$

#### (3) Proposal-Target 괴리 (Proposal-Target Divergence)

센서가 매우 정확하고 모션이 부정확하면, target belief는 좁고 proposal은 넓어 효율이 급락한다. 극단적 경우: 노이즈 없는 range sensor라면 $p(z \mid x)$의 지지가 저차원 manifold에 국한되어 대부분의 입자가 weight $\approx 0$을 받는다.

완화: 측정 노이즈를 일부러 키우거나(정밀도 손실 대가), measurement-aware proposal을 써서 측정 정보를 sampling 단계에 통합한다.

#### (4) Particle Deprivation

고차원 공간에서 참 상태 근처에 입자가 하나도 없을 수 있다. Resampling의 무작위성이 참 상태 근처 입자를 한 cycle에 쓸어버릴 확률이 매 cycle마다 0보다 크다. 한 번 잃으면 복구가 어렵다.

완화: 매 cycle마다 무작위 입자를 소량 새로 넣는다(random injection). posterior가 약간 왜곡되지만 추정이 통째로 무너지는 일을 막는다.

#### Low-Variance Sampler

Resampling은 보통 low-variance (systematic) sampler로 구현한다. 난수 하나로 $M$개를 일정 간격으로 뽑아 $O(M)$에 끝낸다.

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

가중치가 모두 같으면 결과가 입력과 같아서, 측정이 없는 step에서도 입자 다양성이 유지된다. 독립 샘플링 $O(M \log M)$ 대비 $O(M)$.

Ch.14 §14.7.7–14.7.8은 이 오차에 대응하는 augmented MCL과 mixture MCL을 다룬다.

> **추천 자료**
> - [Thrun, Burgard, Fox — Probabilistic Robotics (2005)](http://probabilistic-robotics.org/) — Ch.4 전체가 이 절의 원전. 히스토그램 필터, 이진 베이즈 필터, 입자 필터의 알고리즘과 분석이 실려 있다.
> - [Arulampalam et al. — A Tutorial on Particle Filters (IEEE Trans. Signal Processing 2002)](https://ieeexplore.ieee.org/document/978374) — 입자 필터의 이론과 응용을 종합한 표준 튜토리얼.
> - [Thrun — Particle Filters in Robotics (UAI 2002)](https://arxiv.org/abs/1301.0607) — Rao-Blackwellized PF·FastSLAM과의 연결을 설명하는 짧은 논문.
> - [ROS AMCL package (ROS 1 문서)](https://wiki.ros.org/amcl) — §3.11.3~3.11.4의 입자 필터를 구현한 ROS 패키지. KLD-sampling MCL과 augmented MCL(recovery_alpha_slow/fast)을 쓴다.
