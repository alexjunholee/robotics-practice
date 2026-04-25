# Ch.3 — 수학적 기초 (Mathematical Foundations)

Spatial AI를 제대로 이해하려면 수학적 기초가 필요하다. 여기서는 핵심 개념만 간략히 짚고, 깊은 학습은 추천 자료를 참고하자.

수학 파트를 건너뛰고 싶은 마음은 이해한다. 하지만 논문을 읽다 보면 결국 수식에서 막힌다. SLAM 논문에서 "SE(3) 위의 최적화"라는 말이 나오는데 SE(3)이 뭔지 모르면 논문의 핵심 아이디어를 놓치게 되고, "Jacobian을 유도하여 Gauss-Newton으로 풀었다"는 한 줄이 이해가 안 되면 그 논문의 방법론 전체를 이해할 수 없다. 여기서 다루는 수학은 "수학 시험을 위한 수학"이 아니라, "로봇 논문을 읽고 구현하기 위한 수학"이다. 공대 3학년이면 선형대수를 들었을 테니, 여기서는 학부 때 배운 것이 로보틱스에서 어떻게 쓰이는지 연결하는 데 집중한다.

고전적인 수학 도구가 여전히 핵심이지만, Differentiable Programming과 Auto-Differentiation(자동 미분)이 최적화 문제 접근 방식을 바꾸고 있다. 예전에는 Jacobian을 손으로 유도해야 했지만, 이제는 PyTorch나 JAX의 자동 미분으로 복잡한 파이프라인의 gradient를 계산할 수 있다. End-to-End 학습 기반 SLAM, Differentiable Rendering(NeRF, 3D Gaussian Splatting) 등이 가능해진 배경이다. 자동 미분이 내부적으로 무엇을 하는지 이해하려면 결국 여기서 다루는 기초가 필요하다.

## 3.1 선형대수 (Linear Algebra)

선형대수는 Spatial AI 전반에서 쓰이는 기본 도구이다. 좌표 변환, 카메라 모델, 최적화, 딥러닝까지 전부 행렬과 벡터로 표현된다. "학부 때 선형대수를 들었다"와 "선형대수를 로보틱스에 활용할 수 있다"는 다른 레벨이다. 여기서는 로보틱스에서 가장 많이 쓰이는 개념을 짚는다.

### 3.1.1 벡터와 행렬

**벡터**: 크기와 방향을 가진 양

```
v = [v_x, v_y, v_z]^T  (열 벡터)
```

로보틱스에서 벡터는 3D 공간의 점, 힘, 속도 등을 나타낸다. "로봇이 월드 좌표계에서 (3, 2, 1)에 있다"는 것은 위치를 벡터로 표현한 것이다.

**행렬 연산**:
- 덧셈/뺄셈: 요소별 연산
- 곱셈: 행×열 내적
- 전치(Transpose): A^T
- 역행렬(Inverse): A^(-1), AA^(-1) = I

좌표 변환, 회전, 투영(projection) 전부 행렬 곱으로 표현된다. 카메라가 3D 점을 2D 이미지로 투영하는 것도, 로봇의 좌표계를 변환하는 것도 전부 행렬 곱이다.

> **추천 자료**
> - [3Blue1Brown — Essence of Linear Algebra](https://www.youtube.com/playlist?list=PLZHQObOWTQDPD3MizzM2xVFitgF8hE_ab) — 이 시리즈를 안 봤다면 보는 걸 권한다. 행렬 곱셈이 기하학적으로 무엇을 의미하는지, 고유값이 왜 중요한지를 시각적으로 보여준다. 선형대수를 "계산"이 아닌 "변환"으로 이해하는 데 도움이 된다.
> - [Introduction to Applied Linear Algebra (Boyd & Vandenberghe) — 무료 PDF](https://web.stanford.edu/~boyd/vmls/) — Stanford의 Boyd 교수가 쓴 응용 선형대수 교재. 실용적 관점, Python 예제 포함.
> - [다크 프로그래머 — 선형대수학 시리즈 (6편: 기본공식~PCA)](https://darkpgmr.tistory.com/103) — 주요용어, 역행렬, 고유값, SVD, 연립방정식, PCA를 한글로 정리
> - [다크 프로그래머 — 벡터 미분과 행렬 미분](https://darkpgmr.tistory.com/141) — 벡터/행렬 미분 규칙 정리. Jacobian 계산에 필요한 기초

### 3.1.2 고유값 분해 (Eigenvalue Decomposition)

```
Av = λv
```

- v: 고유벡터 (eigenvector)
- λ: 고유값 (eigenvalue)

**활용**: PCA, 공분산 행렬 분석, 안정성 분석

포인트 클라우드를 다루면 바로 쓸 일이 생긴다. PCA(Principal Component Analysis)로 포인트 클라우드의 주축을 구할 때, 공분산 행렬의 고유벡터가 바로 주축 방향이고 고유값이 그 방향의 분산이다. "이 포인트 클라우드가 평면인지 직선인지"를 판별하는 것도 고유값의 비율로 한다. Normal 벡터 추정도 가장 작은 고유값에 대응하는 고유벡터를 사용한다.

> **추천 자료**
> - [3Blue1Brown — Eigenvectors and Eigenvalues](https://www.youtube.com/watch?v=PFDu9oVAE-g) — 고유값의 기하학적 의미를 직관적으로 설명
> - [MIT 18.06 Linear Algebra — Gilbert Strang (YouTube)](https://www.youtube.com/playlist?list=PLE7DDD91010BC51F8) — 선형대수의 널리 알려진 강의. 고유값 분해를 포함한 전체 선형대수를 깊이 있게 다룬다.

> **실습**: [PCA 3D · 차원 축소](https://alexjunholee.github.io/robotics-practice/app.html#pca_3d)
> 3D 분포에서 공분산 행렬의 고유벡터가 주축이 되는 과정을 직접 조작하고, PC1·PC2 평면(3D→2D)과 PC1 축(2D→1D)으로의 차원 축소를 동시에 시각화한다.

### 3.1.3 특이값 분해 (SVD: Singular Value Decomposition)

```
A = UΣV^T
```

- U: 좌측 특이벡터 (m×m 직교행렬)
- Σ: 특이값 대각행렬 (m×n)
- V: 우측 특이벡터 (n×n 직교행렬)

**활용**: 최소자승법 해, 행렬 근사, Fundamental Matrix 계산

SVD는 로보틱스에서 정말 많이 나온다. 과결정(overdetermined) 시스템의 최소자승 해를 구하는 데 가장 수치적으로 안정적인 방법이기 때문이다. 카메라 캘리브레이션에서 Fundamental Matrix를 구할 때, 포인트 클라우드 정합에서 최적 변환을 구할 때, 전부 SVD를 사용한다. "8-point algorithm"에서 8개 이상의 대응점으로 Fundamental Matrix를 구하는 마지막 단계가 바로 SVD이다.

> **추천 자료**
> - [Steve Brunton — Singular Value Decomposition (YouTube)](https://www.youtube.com/watch?v=nbBvuuNVfco) — SVD의 수학적 의미와 응용을 명쾌하게 설명하는 워싱턴 대학교 교수의 강의
> - [Linear Algebra and Its Applications (Gilbert Strang)](https://math.mit.edu/~gs/linearalgebra/ila6/indexila6.html) — 선형대수 표준 교재. SVD 챕터가 특히 잘 쓰여 있다.

## 3.2 3D 기하학 (3D Geometry)

3D 기하학은 Spatial AI의 핵심이다. "3D 공간에서 로봇은 어디에 있고, 카메라는 어디를 보고 있으며, 저 물체는 어디에 있는가?"를 수학적으로 표현한다. 이 파트를 모르면 SLAM 논문의 첫 페이지부터 막힌다.

### 3.2.1 좌표계 (Coordinate Frames)

Spatial AI에서는 여러 좌표계를 오가며 작업한다. **World Frame(W)**은 전역 고정 좌표계이고, **Camera Frame(C)**은 카메라 중심, **Body Frame(B)**은 로봇 중심, **IMU Frame(I)**은 IMU 센서 기준 좌표계다.

직접 로봇 시스템을 만들어보면 바로 체감한다. 하나의 데이터가 여러 좌표계를 거쳐야 의미가 있기 때문이다. "카메라가 본 물체의 위치"는 카메라 좌표계에서 표현되어 있지만, 로봇이 물체에 접근하려면 그 위치를 로봇 좌표계 또는 월드 좌표계로 변환해야 한다. 센서마다 자기만의 좌표계가 있고, 이들 사이의 변환을 정확히 알아야(extrinsic calibration) 센서 퓨전이 가능하다.

**좌표 변환**:

```
p_W = T_WC × p_C
```

T_WC: Camera → World 변환 행렬 (4×4)

> **추천 자료**
> - [State Estimation for Robotics, Ch.6 — Coordinate Frames (Tim Barfoot) — 무료 PDF](http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) — 좌표계 변환을 로보틱스 관점에서 가장 잘 정리한 교재
> - [Stanford CS231A — Camera Models](https://web.stanford.edu/class/cs231a/) — Stanford의 CV 강의에서 카메라 좌표계와 투영 모델을 다루는 부분

### 3.2.2 회전 표현 (Rotation Representations)

회전 표현이 여러 가지인 이유는 각각 장단점이 다르기 때문이다. SLAM 최적화에서는 어떤 표현을 쓰느냐에 따라 수렴 속도와 안정성이 달라진다. 이 내용을 모르면 "왜 이 코드에서는 쿼터니언을 쓰고, 저 코드에서는 Rotation Matrix를 쓰는지" 이해할 수 없다.

**Rotation Matrix R**은 3×3 직교행렬(det(R) = 1, R^T = R^(-1))로, 9개 파라미터에 6개 제약이 걸려 실제 자유도는 3이다.

**Euler Angles**은 Roll(φ), Pitch(θ), Yaw(ψ) 세 각도로 회전을 표현한다. 직관적이지만 **Gimbal Lock** 문제가 있고, 적용 순서(ZYX, XYZ 등)에 따라 결과가 달라진다.

**Quaternion q = [w, x, y, z]**(||q|| = 1)는 4개 파라미터로 3 DoF를 표현한다. Gimbal Lock이 없고 보간(Slerp)이 용이해 가장 널리 쓰인다.

**Axis-Angle**은 회전축 n과 회전각 θ를 조합한 3개 파라미터 표현이다. Rodrigues formula를 통해 Rotation Matrix로 변환된다.

실전에서의 팁: ROS에서는 Quaternion이 기본 회전 표현이고, OpenCV에서는 Rodrigues 벡터(Axis-Angle)를 주로 사용하며, 최적화 라이브러리(Ceres, GTSAM)에서는 Lie Group 기반 표현(so(3) → SO(3) 매핑)을 사용하는 경우가 많다. 이들 사이의 변환을 자유자재로 할 수 있어야 한다.

> **추천 자료**
> - [3Blue1Brown — Quaternions and 3D Rotation](https://www.youtube.com/watch?v=zjMuIxRvygQ) — 쿼터니언의 기하학적 의미를 시각화한 영상. 4차원이 왜 3D 회전에 필요한지 직관적으로 이해할 수 있다.
> - [State Estimation for Robotics, Ch.7 — Rotation (Tim Barfoot)](http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) — 모든 회전 표현과 그들 사이의 변환을 깔끔하게 정리한 교재
> - [Sola — Quaternion Kinematics for the Error-State Kalman Filter (Tech Report)](https://arxiv.org/abs/1711.02508) — VIO/INS 구현 시 쿼터니언 기반 에러 상태 칼만 필터의 수학적 기초. 실전에서 매우 유용한 테크니컬 리포트.
> - [3D Rotation Converter](https://www.andre-gaschler.com/rotationconverter/) — 쿼터니언, 오일러 각, 회전 행렬 간 변환을 확인할 수 있는 온라인 도구

> **실습**: [회전 표현과 Gimbal Lock](https://alexjunholee.github.io/robotics-practice/app.html#rotation_gimbal) | [6DoF 포즈 시각화](https://alexjunholee.github.io/robotics-practice/app.html#xyzrpy_6dof)
> 오일러 각의 Gimbal Lock 현상과 쿼터니언 회전을 직접 조작하며 비교하고, 6자유도 포즈(x, y, z, roll, pitch, yaw)를 인터랙티브하게 확인할 수 있다.

### 3.2.3 Homogeneous Coordinates

3D 점을 4D로 확장하여 변환을 단일 행렬로 표현:

```
[X, Y, Z, 1]^T  (3D 점)

T = | R   t |   (4×4 변환 행렬)
    | 0   1 |
```

Homogeneous Coordinates를 쓰는 이유: 회전과 이동(translation)을 하나의 행렬 곱으로 표현할 수 있기 때문이다. 일반 좌표에서는 p' = Rp + t (곱셈 + 덧셈)이지만, Homogeneous Coordinates에서는 p' = Tp (곱셈만)로 쓸 수 있다. 여러 변환을 연쇄적으로 적용할 때 행렬을 그냥 곱하면 되어, 로봇 팔의 관절 변환 같은 체인을 다룰 때 편리하다.

### 3.2.4 SE(3)와 SO(3)

**SE(3)**(Special Euclidean Group)는 3D 강체 변환(회전 + 이동) 전체의 집합으로 6 DoF를 가진다. **SO(3)**(Special Orthogonal Group)는 회전만의 집합으로 3 DoF다.

SE(3)와 SO(3)는 **Lie Group**이다. 최적화를 할 때 "회전 행렬의 제약조건(직교, 행렬식 1)을 만족하면서 업데이트"해야 하는데, Lie Group 이론이 이를 우아하게 해결한다. 대응되는 **Lie Algebra** (se(3), so(3))에서 제약 없이 최적화한 후 Exponential Map으로 다시 Lie Group으로 매핑하는 방식이다. SLAM의 Pose Graph Optimization에서 이 개념이 핵심으로 쓰인다.

> **추천 자료**
> - [State Estimation for Robotics, Ch.7 (Tim Barfoot) — 무료 PDF](http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) — SE(3), SO(3), Lie Group/Algebra를 로보틱스 관점에서 가장 잘 풀어낸 교재. 이 분야를 판다면 꼭 읽자.
> - [Sola — A Micro Lie Theory for State Estimation in Robotics (arXiv:1812.01537)](https://arxiv.org/abs/1812.01537) — Lie Group 이론을 로보틱스 상태 추정에 필요한 만큼만 간결하게 정리한 논문. 매우 실용적.

## 3.3 확률 및 통계 (Probability & Statistics)

센서 데이터에는 항상 노이즈가 있고, 로봇의 상태에는 항상 불확실성이 있다. 이 불확실성을 수학적으로 표현하고 다루는 것이 확률과 통계이다. "센서 값이 정확히 3.0m"가 아니라 "3.0m ± 0.05m (95% 신뢰구간)"으로 표현해야 의미가 있고, 이 불확실성을 전파하고 업데이트하는 것이 상태 추정의 기본이다.

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

센서 노이즈, 위치 불확실성 모델링에 널리 쓰인다.

정규분포가 이렇게까지 많이 쓰이는 이유는 수학적 편의성이다. 중심극한정리 덕분에 많은 자연현상이 정규분포를 따르고, 정규분포끼리의 연산(곱, 합)이 닫혀 있어 분석이 쉽다. 칼만 필터가 정규분포를 가정하는 것도 같은 이유다.

> **추천 자료**
> - [3Blue1Brown — But what is the Central Limit Theorem?](https://www.youtube.com/watch?v=zeJD6dqJ5lo) — 중심극한정리를 시각적으로 설명. 왜 정규분포가 어디에나 나타나는지 직관적으로 이해할 수 있다.
> - [Kalman Filter — How it works, in pictures](http://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/) — 칼만 필터의 작동 원리를 시각적으로 설명. 수식 전에 직관을 잡기 좋다

> **실습**: [Kalman Filter](https://alexjunholee.github.io/robotics-practice/app.html#kalman_filter)
> 칼만 필터의 predict-update 사이클을 인터랙티브하게 조작하며, 정규분포 기반 상태 추정 과정을 확인할 수 있다.

**Mahalanobis Distance**

유클리드 거리는 모든 방향을 동등하게 취급한다. 하지만 센서 데이터는 방향에 따라 불확실성이 다르다. 예를 들어 GPS는 수평 방향(수 미터)보다 수직 방향(수십 미터)의 오차가 크다.

Mahalanobis 거리는 공분산(covariance)을 고려한 거리이다:

```
d_M = sqrt((x - μ)^T Σ^{-1} (x - μ))
```

Σ가 단위 행렬이면 유클리드 거리와 같다. Σ가 대각 행렬이면 각 축별로 스케일링된 거리이다. 일반적인 Σ에서는 공분산의 주축 방향으로 거리가 재정의된다.

SLAM에서의 활용: 데이터 연관(data association) 시 "이 관측이 이 랜드마크에서 왔는가?"를 판단할 때 Mahalanobis 거리를 쓴다. 유클리드로 가까워도 Mahalanobis로 멀면 (불확실성 방향과 맞지 않으면) 잘못된 연관일 가능성이 높다.

(참고: [다크 프로그래머 — 평균, 표준편차, 분산, 그리고 Mahalanobis 거리](https://darkpgmr.tistory.com/41))

### 3.3.2 베이즈 정리 (Bayes' Rule)

```
P(A|B) = P(B|A) × P(A) / P(B)
```

베이즈 정리는 상태 추정의 수학적 기반이다. "센서 측정값이 주어졌을 때, 로봇의 실제 상태는 무엇인가?"라는 질문에 답하는 공식이다. 베이즈 정리를 모르면 칼만 필터, 파티클 필터, Factor Graph 기반 SLAM 전부 이해할 수 없다.

**재귀적 상태 추정**:

```
P(x_t | z_{1:t}) ∝ P(z_t | x_t) × P(x_t | z_{1:t-1})
```

- P(z_t | x_t): Measurement model (관측 모델) — "로봇이 이 위치에 있다면, 센서가 이 값을 출력할 확률은?"
- P(x_t | z_{1:t-1}): Prior (이전 상태 기반 예측) — "이전까지의 정보로 볼 때 로봇이 여기 있을 확률은?"

이 재귀적 구조를 이해하면 칼만 필터가 바로 이해된다. 새로운 센서 데이터가 들어올 때마다 기존 믿음(prior)을 업데이트하여 더 정확한 추정(posterior)을 얻는다. 칼만 필터의 predict-update 사이클이 바로 이 구조의 구현이다.

> **추천 자료**
> - [3Blue1Brown — Bayes' Theorem](https://www.youtube.com/watch?v=HZGCoVF3YvM) — 베이즈 정리를 시각적으로 이해하기 좋은 영상
> - [Probabilistic Robotics (Thrun, Burgard, Fox)](https://www.probabilistic-robotics.org/) — 확률적 로보틱스의 필수 교재. 베이즈 필터, 칼만 필터, 파티클 필터, SLAM까지 확률론적 관점에서 깔끔하게 정리한 교과서다.
> - [김기섭 블로그 — Bayesian Filtering 시리즈 (2편)](https://gisbi-kim.github.io/blog/2021/03/09/bayesfiltering-1.html) — 베이즈 필터의 한글 해설. 칼만 필터로 이어지는 기초

### 3.3.3 MLE와 MAP

**MLE (Maximum Likelihood Estimation)**:

```
x* = argmax P(z | x)
```

데이터가 주어졌을 때 가장 가능성 높은 파라미터

**MAP (Maximum A Posteriori)**:

```
x* = argmax P(x | z) = argmax P(z | x) × P(x)
```

사전 확률(prior)을 고려한 추정

SLAM에서 "관측 데이터만 보고 최적 위치를 구하는 것(MLE)"과 "이전 위치 정보도 함께 고려하여 최적 위치를 구하는 것(MAP)"의 차이다. 실제 SLAM 시스템은 대부분 MAP 추정을 쓴다. Prior를 넣으면 노이즈가 심한 관측에도 안정적으로 추정할 수 있기 때문이다. 수학적으로는 MAP의 로그를 취하면 "관측 오차의 제곱합 + 정규화 항"이 되어, 최적화 관점에서 Regularized Least Squares와 같은 형태가 된다.

> **추천 자료**
> - [Probabilistic Robotics, Ch.2 — Recursive State Estimation (Thrun)](https://www.probabilistic-robotics.org/) — MLE, MAP, 베이즈 필터의 관계를 로보틱스 맥락에서 설명
> - [Cyrill Stachniss — Maximum Likelihood and MAP Estimation](https://www.youtube.com/watch?v=XepXtl9YKwc) — MLE와 MAP의 차이를 예시와 함께 명쾌하게 설명

**MLE와 MAP의 직관적 차이**

둘 다 "가장 그럴듯한 파라미터를 찾는다"는 목표는 같지만, 접근이 다르다.

MLE(Maximum Likelihood)는 "이 데이터가 관측될 확률을 가장 높이는 파라미터는?"을 묻는다. 데이터만 본다. MAP(Maximum A Posteriori)는 여기에 prior를 더한다. "이 데이터가 관측되었을 때, 사전 지식까지 합쳐서 파라미터의 사후 확률을 가장 높이는 값은?"이 그 질문이다.

수식으로: MAP = MLE + prior. 가우시안 prior를 쓰면 MAP는 MLE에 L2 정규화를 추가한 것과 같다. 딥러닝에서 weight decay가 MAP의 구현이라고 볼 수 있다.

SLAM에서: odometry 측정값의 likelihood와 센서 관측의 likelihood를 곱하고, 이전 상태의 prior를 결합하여 MAP 추정을 한다. Factor graph에서 각 factor가 바로 이 likelihood/prior에 해당한다.

(참고: [다크 프로그래머 — 베이즈 정리, ML과 MAP, 그리고 영상처리](https://darkpgmr.tistory.com/62))

## 3.4 최적화 기초 (Optimization Basics)

최적화는 Spatial AI 알고리즘의 마지막 단계다. SLAM의 Bundle Adjustment, 카메라 캘리브레이션, 딥러닝 학습 전부 최적화 문제다. 이 섹션의 내용을 모르면 코드를 돌릴 수는 있어도, 왜 수렴하지 않는지, 왜 결과가 이상한지 디버깅할 수 없다.

### 3.4.1 Least Squares

```
x* = argmin ||Ax - b||²
```

**정규방정식 (Normal Equation)**:

```
x* = (A^T A)^(-1) A^T b
```

최소자승법은 "노이즈가 있는 여러 측정값에서 가장 적합한 모델 파라미터를 찾는" 가장 기본적인 방법이다. 직선 피팅부터 카메라 캘리브레이션까지, 거의 모든 추정 문제의 출발점이다.

> **추천 자료**
> - [Cyrill Stachniss — Least Squares for Robotics](https://www.youtube.com/watch?v=r2cyMQ5NB1o) — 최소자승법을 로보틱스 문제에 적용하는 방법을 구체적으로 설명
> - [김기섭 블로그 — SLAM back-end 시리즈 (3편)](https://gisbi-kim.github.io/blog/2021/03/04/slambackend-1.html) — "SLAM은 Ax=b를 푸는 문제"에서 시작하는 back-end 입문. Factor graph까지 3편 시리즈
> - [김기섭 블로그 — Iterative Optimization 1편](https://gisbi-kim.github.io/blog/2021/03/16/leastsquare-1.html) — 비선형 최적화의 직관적 한글 해설

**최소자승법의 직관**

"왜 오차의 제곱을 최소화하는가?" 절댓값이 아니라 제곱인 이유는 두 가지다. 미분이 가능하고 큰 오차에 더 큰 페널티를 준다. 부수적으로 가우시안 노이즈 가정 하에서 Maximum Likelihood Estimation과 동일한 해를 준다는 성질도 있다.

over-determined system (방정식 수 > 미지수 수)에서 Ax = b를 정확히 만족하는 x는 없다. 대신 ||Ax - b||²를 최소화하는 x를 찾으면, normal equation `A^T A x = A^T b`가 된다. 이것이 최소자승법의 전부다.

주의: `A^T A`가 singular하면 (rank 부족) 유일한 해가 없다. 이때는 pseudo-inverse `x = (A^T A)^{-1} A^T b` 대신 SVD를 써야 수치적으로 안정적이다.

(참고: [다크 프로그래머 — 최소자승법 이해와 다양한 활용예](https://darkpgmr.tistory.com/56))

### 3.4.2 Gradient Descent

```
x_{k+1} = x_k - α × ∇f(x_k)
```

- α: Learning rate
- ∇f: Gradient (기울기)

Gradient Descent는 딥러닝에서 매일 쓰는 알고리즘이지만, 로보틱스 최적화에서도 기본이 된다. 함수의 기울기 반대 방향으로 조금씩 이동하여 최솟값을 찾는 직관적인 방법이다. 다만 learning rate 설정이 어렵고, local minimum에 빠질 수 있으며, 수렴 속도가 느리다는 한계가 있어서, 로보틱스에서는 보통 더 효율적인 방법(Gauss-Newton, LM)을 사용한다.

**Gradient, Jacobian, Hessian의 관계 정리**

혼동하기 쉬운 세 개념을 정리한다:

**Gradient** ∇f는 스칼라 함수 f의 1차 미분으로 n×1 벡터를 출력한다. "어느 방향으로 가야 f가 가장 빠르게 증가하는가"를 알려준다. **Jacobian** J는 이를 벡터 함수 f: R^n → R^m으로 확장한 것으로 m×n 행렬이다. 각 출력의 각 입력에 대한 편미분을 담는다. **Hessian** H는 스칼라 함수 f의 2차 미분으로 n×n 대칭 행렬이며, 곡률 정보를 담아 Newton's method에서 사용한다.

관계:
```
비용 함수 C(x) = ||r(x)||² 일 때:
  Gradient:  ∇C = J^T r           (J는 r의 Jacobian)
  Hessian:   H ≈ J^T J            (Gauss-Newton 근사: 2차 미분 항 무시)
  Update:    δx = -(J^T J)^{-1} J^T r
```

Gauss-Newton이 J^T J를 Hessian 근사로 쓰는 이유: 정확한 Hessian은 계산이 비싸고, 잔차 r이 작은 영역에서는 2차 항이 무시할 수 있을 만큼 작기 때문이다.

(참고: [다크 프로그래머 — Gradient, Jacobian 행렬, Hessian 행렬, Laplacian](https://darkpgmr.tistory.com/132))

### 3.4.3 Gauss-Newton

비선형 최소자승 문제를 반복적으로 선형화하여 해결:

```
(J^T J) Δx = -J^T r
x_{k+1} = x_k + Δx
```

- J: Jacobian 행렬
- r: Residual (잔차)

Gauss-Newton이 Gradient Descent보다 로보틱스에서 선호되는 이유: 2차 정보(Hessian의 근사인 J^T J)를 사용하므로 수렴이 훨씬 빠르다. SLAM에서 수천~수만 개의 변수를 최적화할 때, Gradient Descent로는 수렴에 너무 오래 걸리지만 Gauss-Newton으로는 몇 번의 반복만에 수렴할 수 있다.

### 3.4.4 Levenberg-Marquardt (LM)

Gauss-Newton과 Gradient Descent의 결합:

```
(J^T J + λI) Δx = -J^T r
```

- λ: Damping factor
- λ 작음 → Gauss-Newton (빠른 수렴)
- λ 큼 → Gradient Descent (안정적)

SLAM의 Bundle Adjustment, Pose Graph Optimization에서 핵심 알고리즘으로 쓰인다.

LM이 실전에서 가장 많이 쓰이는 이유: Gauss-Newton은 초기값이 좋으면 매우 빠르게 수렴하지만, 초기값이 나쁘면 발산할 수 있다. LM은 λ를 자동으로 조절하여, 초기에는 Gradient Descent처럼 안정적으로 시작하고, 해에 가까워지면 Gauss-Newton처럼 빠르게 수렴한다. Ceres Solver, g2o, GTSAM 같은 로보틱스 최적화 라이브러리에서 기본 알고리즘으로 채택하고 있다.

> **추천 자료**
> - [Cyrill Stachniss — Gauss-Newton and Levenberg-Marquardt for SLAM](https://www.youtube.com/watch?v=hRyL5KwFLAE) — SLAM에서 Gauss-Newton과 LM이 어떻게 사용되는지 단계별로 설명
> - [State Estimation for Robotics, Ch.4 — Nonlinear Optimization (Tim Barfoot) — 무료 PDF](http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) — 비선형 최적화를 로보틱스 상태 추정 관점에서 잘 설명한다
> - [Ceres Solver Tutorial](http://ceres-solver.org/tutorial.html) — Google의 비선형 최소자승 최적화 라이브러리. 실제로 LM 알고리즘을 코드로 어떻게 사용하는지 실습할 수 있다.
> - [다크 프로그래머 — 최적화 기법의 직관적 이해](https://darkpgmr.tistory.com/149) — Gradient Descent, Newton, LM 등의 기하학적 직관
> - [김기섭 블로그 — SLAM Back-end 공부자료 5개 추천](https://gisbi-kim.github.io/blog/2021/10/03/slam-textbooks.html) — Error-state KF, Factor Graphs, Bundle Adjustment 등 핵심 자료 큐레이션
> - [Derivative Calculator](https://www.derivative-calculator.net/) — 수식 미분을 단계별로 보여주는 온라인 도구. Jacobian 유도할 때 검산에 유용

**LM의 직관: Gauss-Newton과 Gradient Descent 사이의 스위칭**

비선형 최소자승 문제에서 Gauss-Newton은 수렴이 빠르지만 초기값이 나쁘면 발산한다. Gradient Descent는 느리지만 안정적이다. LM은 damping factor λ로 둘 사이를 자동으로 전환한다.

λ가 작으면 Gauss-Newton에 가까워 해 근처에서 빠르게 수렴하고, 크면 Gradient Descent에 가까워 해에서 먼 초기 단계에서 안정적이다. update가 비용을 줄이면 λ를 줄이고, 비용이 늘면 λ를 키운다. 이 adaptive한 전환이 LM의 핵심이다. Ceres Solver의 기본 solver가 LM인 이유이기도 하다.

(참고: [다크 프로그래머 — 함수최적화 기법 정리 (LM 방법 등)](https://darkpgmr.tistory.com/142))

## 3.5 심화: Lie Group과 Lie Algebra

*연구자가 되고 싶다면 여기서부터 읽어라.*

로보틱스에서 가장 자주 마주치는 수학적 난관 중 하나는 "회전을 어떻게 최적화할 것인가"이다. Lie group과 Lie algebra는 회전과 강체 변환을 체계적으로 다루는 틀을 제공한다. SLAM 백엔드, Visual-Inertial Odometry, Bundle Adjustment를 이해하려면 이 내용이 필수다.

### 3.5.1 왜 Lie Group이 필요한가

3.2절에서 회전을 표현하는 여러 방법을 다뤘다. 그런데 이 표현들을 가지고 최적화를 하려고 하면 문제가 생긴다.

- **회전 행렬 R**: 3x3이므로 파라미터가 9개인데, 실제 자유도는 3이다. R^T R = I 와 det(R) = 1이라는 제약 조건이 있기 때문이다. 일반적인 unconstrained optimization을 적용하면 업데이트 후 R이 더 이상 유효한 회전 행렬이 아니게 된다.
- **쿼터니언**: 4개 파라미터에 정규화 제약(||q|| = 1)이 있다. 업데이트할 때마다 re-normalize해야 하고, 이 과정에서 수치 오류가 누적될 수 있다.
- **오일러 각**: Gimbal lock 문제가 있고, 각도 wrapping도 까다롭다.

핵심 문제는 이것이다: 회전은 비선형 manifold 위에 살고 있는데, 우리가 아는 최적화 알고리즘(Gauss-Newton, LM)은 유클리드 공간에서 동작한다. Lie group 이론은 이 간극을 메운다. Manifold 위의 점(회전 행렬) 근처에 접선 공간(Lie algebra)을 정의하고, 이 접선 공간에서 유클리드 최적화를 수행한 뒤, 결과를 다시 manifold 위로 올리는 것이다.

배경 지식: 여기서 말하는 "group"이란, 어떤 연산에 대해 닫힘(closure), 결합법칙(associativity), 항등원(identity), 역원(inverse)이 성립하는 집합이다. 예를 들어 invertible한 n x n 행렬의 집합은 행렬 곱에 대해 group을 이루며, 이를 general linear group GL(n)이라 한다. 그 중 det = 1인 부분군이 special linear group SL(n)이다. Orthogonal group O(n)은 내적을 보존하는 행렬의 집합이고, 여기서 det = 1인 것만 모으면 SO(n) — 즉 회전군이 된다. det = -1인 것들은 반사(reflection)를 포함하며, 이들은 군 연산에 대해 닫혀있지 않으므로 부분군을 형성하지 않는다.

### 3.5.2 SO(3): 3D 회전군

**정의:**
```
SO(3) = { R in R^{3x3} | R^T R = I, det(R) = 1 }
```

SO(3)는 group이다. 군 연산은 행렬 곱이고, 두 회전 R_1, R_2의 합성 R_1 R_2도 SO(3)의 원소다. 항등원은 단위 행렬 I, 역원은 R^T(= R^{-1})이다. 직교 행렬이므로 전치가 곧 역행렬이 된다. 행렬 곱은 결합법칙을 만족하지만 교환법칙은 성립하지 않는다(일반적으로 R_1 R_2 != R_2 R_1).

**Lie algebra so(3):**

SO(3)의 Lie algebra는 3x3 반대칭 행렬(skew-symmetric matrix)의 공간이며, 3차원이다.

**Hat operator** `[.]x` 는 3차원 벡터를 반대칭 행렬로 변환한다:

```
w = [w1, w2, w3]^T  (in R^3)

        [  0   -w3   w2 ]
[w]x =  [  w3   0   -w1 ]   in so(3)
        [ -w2   w1   0  ]
```

이 행렬은 벡터 외적(cross product)에 대응한다: `[w]x v = w x v`

**Vee operator** `(.)v` 는 역변환이다: 반대칭 행렬에서 3차원 벡터를 추출한다.

직관적으로, so(3)의 원소 w는 "회전축 방향"과 "회전 크기"를 하나의 벡터로 인코딩한다. 축-각(axis-angle) 표현과 직접 대응된다.

### 3.5.3 Exponential Map과 Logarithmic Map

**Exponential map**: so(3) -> SO(3)

Lie algebra의 원소(벡터)를 Lie group의 원소(회전 행렬)로 보내는 사상이다. 이것이 어디서 나오는지 유도해 보자.

시간에 따라 연속적으로 회전하는 행렬 R(t)가 있다고 하자 (R(0) = I). R(t)는 항상 SO(3)에 있으므로 `R(t) R(t)^T = I`이다. 양변을 t로 미분하면:

```
d/dt (R R^T) = R_dot R^T + R R_dot^T = 0
→  R_dot R^T = -(R R_dot^T)^T
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

여기서 `exp([w]x)`는 축 w 방향으로 ||w|| 라디안만큼 회전시키는 행렬이 된다. 구체적으로, theta = ||w||로 두면 **Rodrigues' formula**로 닫힌 형태를 얻는다:

```
exp([w]x) = I + (sin(theta) / theta) [w]x + ((1 - cos(theta)) / theta^2) [w]x^2
```

이 공식은 `sin(t)`와 `cos(t)`의 Taylor 전개를 `[w]x`의 거듭제곱에 대입하면 유도된다. `[w]x^3 = -theta^2 [w]x`라는 성질을 이용하면 급수가 sin, cos 항으로 정리된다.

theta가 작을 때(|theta| < eps)는 sin(theta)/theta ≈ 1, (1-cos(theta))/theta^2 ≈ 1/2이므로:

```
exp([w]x) ≈ I + [w]x + (1/2)[w]x^2   (1차 근사)
```

주의: 하나의 회전 행렬 R에 대해 `R = exp([w]x)`를 만족하는 w는 유일하지 않다. ||w|| + 2*pi*k (정수 k)에 대해 같은 R을 준다. 이것이 logarithmic map에서 주의해야 하는 부분이다.

**Logarithmic map**: SO(3) -> so(3)

역변환이다. 주어진 회전 행렬 R에서 축-각 벡터 w를 복원한다.

```
theta = arccos((tr(R) - 1) / 2)
[w]x = (theta / (2 sin(theta))) (R - R^T)
```

theta = 0 (항등 회전) 이나 theta = pi (180도 회전) 근처에서는 특별한 처리가 필요하다.

**직관**: Lie algebra는 group 위의 한 점(보통 항등원 I)에서의 접선 공간(tangent space)이다. "작은 회전"은 접선 공간의 벡터로 표현할 수 있고, exponential map이 이 벡터를 manifold 위의 실제 회전으로 매핑한다. 이것이 최적화에서 핵심이 되는 이유다: 업데이트량 dw를 접선 공간(R^3)에서 계산한 뒤, exp([dw]x)를 현재 회전에 곱해서 manifold 위에서 이동하는 것이다.

### 3.5.4 SE(3): 3D 강체 변환군

로봇의 포즈는 회전뿐 아니라 이동도 포함한다. 이를 다루는 것이 SE(3)이다.

**정의:**
```
SE(3) = { T = [ R  t ] | R in SO(3), t in R^3 }
              [ 0  1 ]
```

T는 4x4 homogeneous transformation matrix이다. SE(3)도 group이다. 군 연산은 행렬 곱(T_1 T_2)이며, 항등원은 4x4 단위 행렬, 역원은 T^{-1} = [ R^T  -R^T t ; 0  1 ]이다.

**Lie algebra se(3):**

SE(3)의 Lie algebra는 6차원이다. 원소를 **twist** 벡터라 부른다:

```
xi = [rho; w] in R^6     (rho in R^3: 이동 성분, w in R^3: 회전 성분)
```

**Hat operator**는 6차원 벡터를 4x4 행렬로 변환한다:

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

**핵심**: 6-DoF 포즈(3 회전 + 3 이동)를 6차원 벡터 xi in R^6으로 매개변수화할 수 있다. 제약 조건 없는 6차원 유클리드 공간에서 최적화를 수행하고, exponential map으로 결과를 SE(3) manifold 위로 올릴 수 있다. 이것이 SLAM 최적화에서 Lie group을 쓰는 이유다.

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

어느 쪽을 쓰든 수학적으로 일관성 있게 유지하면 된다. 문헌마다 convention이 다르니 주의해야 한다. Barfoot의 교재는 left를 주로 쓰고, Strasdat의 Sophus는 right를 기본으로 한다.

**Jacobian 계산:**

에러 함수 e(T)가 있을 때, perturbation에 대한 Jacobian은:

```
de/d(d_xi) = lim_{d_xi->0}  (e(exp(d_xi^) * T) - e(T)) / d_xi     (left perturbation의 경우)
```

이 Jacobian은 6열짜리 행렬이 된다 (에러 차원 x 6).

**왜 이게 중요한가:**

일반적인 최적화에서 업데이트는 `x <- x + dx` (유클리드 덧셈)이다. 하지만 SE(3) 위에서는 덧셈이 정의되지 않는다. 대신:

1. 접선 공간에서 d_xi in R^6을 Gauss-Newton으로 계산한다: `d_xi = -(J^T J)^{-1} J^T e`
2. Manifold 위에서 업데이트한다: `T <- exp(d_xi^) * T`

이렇게 하면 업데이트 후에도 T가 항상 유효한 SE(3) 원소임이 보장된다. 별도의 제약 조건 처리가 필요 없다.

**실용적 참고**: g2o, GTSAM, Ceres (with local parameterization / manifold)에서 내부적으로 이 방식을 쓴다. GTSAM의 `Pose3`는 SE(3)를 직접 구현하고, `Pose3::Expmap()`, `Pose3::Logmap()`을 제공한다. Ceres에서는 `LocalParameterization` (또는 최신 API의 `Manifold`)을 통해 같은 개념을 구현한다.

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

**실용적 의미**: 센서(예: IMU)가 측정한 속도(angular velocity, linear velocity)는 센서 프레임에서 표현된다. 이를 body 프레임이나 world 프레임으로 변환할 때 Adjoint를 쓴다. 여러 센서를 fusion하는 VIO 시스템에서 좌표계 간 변환이 빈번하게 일어나므로, Adjoint의 의미를 이해하고 있어야 한다.

### 3.5.7 실무에서의 사용

**Sophus (C++)**: Strasdat가 만든 Lie group 라이브러리. SO(3), SE(3)와 그 exponential/logarithmic map, Adjoint 등을 구현한다. ORB-SLAM3, Kimera 등 주요 SLAM 시스템이 사용한다.

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

**GTSAM**: `gtsam::Pose3`가 내부적으로 SE(3)를 쓴다. Factor graph 최적화 시 Lie group 위에서의 perturbation을 자동으로 처리한다.

> **추천 자료**
> - [State Estimation for Robotics, Ch.7-8 (Barfoot)](http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) — Lie group을 로보틱스 상태 추정 관점에서 다루는 핵심 레퍼런스
> - [A micro Lie theory for state estimation in robotics (Sola et al., arXiv:1812.01537)](https://arxiv.org/abs/1812.01537) — Lie group의 핵심만 20페이지로 요약. 논문 읽기 전에 이것부터
> - [TUM Multiple View Geometry, Ch.2 -- Rigid Body Motion](https://cvg.cit.tum.de/teaching/online/mvg) — Daniel Cremers 교수의 강의. SO(3), SE(3)를 시각적으로 설명
> - [Sophus GitHub](https://github.com/strasdat/Sophus) — C++ Lie group 라이브러리. 코드를 읽으면 이해가 빨라진다
> - [정진용 블로그 — SE(3) and SO(3) transformation](https://jinyongjeong.github.io/2016/06/07/se3_so3_transformation/) — SE(3), SO(3) 변환의 한글 정리. GL(3), O(3)부터 체계적으로 설명
> - [T-Robotics: Lie Group Formulation for Robot Mechanics](http://t-robotics.blogspot.com/2015/07/lie-group-formulation-for-robot.html) — 한국어로 작성된 Lie Group 설명. 로봇 역학에서의 Lie Group 활용을 정리

## 3.6 심화: Factor Graph

*연구자가 되고 싶다면 여기서부터 읽어라.*

Factor graph는 SLAM 문제를 체계적으로 정의하고 효율적으로 푸는 프레임워크다. 현대 SLAM 시스템의 백엔드는 거의 예외 없이 factor graph 기반이다.

### 3.6.1 Factor Graph란

Factor graph는 두 종류의 노드로 구성된 이분 그래프(bipartite graph)이다:

- **변수 노드 (variable nodes)**: 추정하고자 하는 상태. 로봇 포즈(x_1, x_2, ...), 랜드마크 위치(l_1, l_2, ...) 등.
- **팩터 노드 (factor nodes)**: 변수들 사이의 제약 조건 또는 측정. 각 팩터는 연결된 변수들에 대한 비용 함수를 정의한다.

확률적으로, 전체 사후 분포는 팩터들의 곱으로 분해된다:

```
p(X | Z) proportional to  prod_i  f_i(X_i)
```

여기서 X_i는 팩터 f_i에 연결된 변수들의 부분 집합이다.

**MAP 추정** = 모든 팩터의 곱을 최대화 = 음의 로그를 취하면 합을 최소화 = **nonlinear least squares** 문제가 된다:

```
X* = argmin_X  sum_i  ||e_i(X_i)||^2_{Sigma_i}
```

e_i는 에러 함수, Sigma_i는 해당 측정의 공분산(불확실성 가중치)이다.

### 3.6.2 SLAM을 Factor Graph로 표현

SLAM에서 흔히 사용되는 팩터 유형들:

| 팩터 | 역할 |
|---|---|
| Prior factor | 초기 포즈에 대한 사전 정보. 예: "시작점은 원점이다" |
| Odometry factor | 두 연속 포즈 사이의 상대 변환. IMU preintegration이나 wheel odometry에서 온다 |
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

각 팩터에는 측정값과 공분산(노이즈 모델)이 포함된다. 그래프가 구축되면 Gauss-Newton 또는 LM으로 전체 변수를 동시에 최적화한다.

### 3.6.3 풀이: Variable Elimination과 Bayes Tree

Factor graph를 최적화하려면 정규 방정식 `H d = -b`를 풀어야 한다 (H는 Hessian 근사, b는 gradient). 이 시스템의 구조를 이해하는 것이 효율적 풀이의 핵심이다.

**Variable elimination**: 변수를 하나씩 소거하는 과정. 이것은 sparse Cholesky factorization과 수학적으로 동등하다. 소거 순서에 따라 fill-in (원래 0이었던 곳이 non-zero가 되는 현상)이 달라지며, 이는 계산 비용에 직접 영향을 미친다.

**Variable ordering**: 소거 순서를 최적화하는 것이 중요하다. COLAMD (Column Approximate Minimum Degree) 같은 heuristic이 널리 쓰인다. 직관적으로, 연결이 적은 변수를 먼저 소거하면 fill-in이 적다.

**Bayes tree**: Kaess et al. (2012)이 제안한 자료구조로, iSAM2의 핵심이다. Factor graph를 elimination하면 Bayes net이 되고, 이를 tree 구조로 재편하면 Bayes tree가 된다. 새로운 측정이 들어올 때, 영향을 받는 subtree만 re-elimination하면 된다.

실시간 SLAM에서는 매 프레임마다 새로운 팩터가 추가된다. 전체 시스템을 처음부터 다시 풀면 O(n^3)이지만, Bayes tree를 이용한 incremental update는 영향받는 부분만 갱신하므로 실시간 처리가 가능하다.

> **추천 자료**
> - [Factor Graphs and GTSAM (Dellaert & Kaess)](https://gtsam.org/tutorials/intro.html) — GTSAM 공식 튜토리얼. Factor graph에서 SLAM으로의 연결을 설명
> - [Factor Graphs for Robot Perception (Dellaert & Kaess, 2017)](https://www.cs.cmu.edu/~kaess/pub/Dellaert17fnt.pdf) — 100페이지 분량의 종합 레퍼런스
> - [CMU 16-833 Lecture Notes](https://www.cs.cmu.edu/~kaess/teaching/16833/) — Michael Kaess 교수의 SLAM 강의. Factor graph와 iSAM2를 깊이 다룬다

> **실습**: [Factor Graph 시각화](https://alexjunholee.github.io/robotics-practice/app.html#factor_graph_viz)
> Factor graph의 변수 노드와 팩터 노드를 직접 구성하고, 그래프 구조가 최적화에 미치는 영향을 확인할 수 있다.

### 3.6.4 Ceres Solver로 Pose Graph 최적화 구현하기

GTSAM 외에 Google의 Ceres Solver로도 factor graph 기반 최적화를 구현할 수 있다. Ceres는 범용 nonlinear least squares 솔버라서 SLAM에 특화된 기능은 없지만, 그만큼 내부 동작을 직접 이해하기 좋다. 아래는 Ceres 공식 예제인 `pose_graph_3d`를 기반으로 한 분석이다.

**Error Term 정의:**

두 포즈 `x_a`, `x_b` 사이의 상대 변환 측정값 `T_ab_measured`가 있을 때, residual은 추정된 상대 변환과 측정값의 차이다.

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
    // 추정된 상대 변환 계산
    Eigen::Quaternion<T> q_a_inverse = q_a.conjugate();
    Eigen::Quaternion<T> q_ab_estimated = q_a_inverse * q_b;
    Eigen::Matrix<T, 3, 1> p_ab_estimated = q_a_inverse * (p_b - p_a);

    // 측정값과의 차이
    Eigen::Quaternion<T> delta_q =
        t_ab_measured_.q.cast<T>() * q_ab_estimated.conjugate();

    // residual = [position_error; orientation_error]
    residuals.block<3,1>(0,0) = p_ab_estimated - t_ab_measured_.p.cast<T>();
    residuals.block<3,1>(3,0) = T(2.0) * delta_q.vec();

    // information matrix 적용 (covariance의 역)
    residuals.applyOnTheLeft(sqrt_information_.cast<T>());
    return true;
  }
};
```

- **template \<typename T\>**: Ceres 내부에서 residual 값이 필요하면 `T=double`, Jacobian이 필요하면 `T=Jet<double>`로 자동 전환된다. 이것이 AutoDiff의 원리다.
- **sqrt_information**: covariance의 Cholesky decomposition. `information.llt().matrixL()`로 구한다.
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

`SPARSE_NORMAL_CHOLESKY`는 pose graph처럼 sparse한 문제에 적합하다. 변수가 많아지면 `SPARSE_SCHUR`도 고려할 수 있다.

**GTSAM vs Ceres 비교**

| | GTSAM | Ceres |
|---|---|---|
| 특성 | SLAM 특화 | 범용 nonlinear least squares |
| 기본 제공 | `BetweenFactor`, `PriorFactor` 등 미리 정의된 팩터 | 없음. 모든 cost function 직접 정의 |
| 증분 최적화 | iSAM2로 incremental 풀이 가능 | 지원 안 함 |
| Manifold | Lie group 기본 지원 | `LocalParameterization` / `Manifold`로 직접 설정 |
| 적합한 상황 | SLAM 시스템 구축 | 유연한 구조가 필요할 때, 대규모 BA |

> **추천 자료**
> - [Ceres Solver 공식 pose_graph_3d 예제](https://ceres-solver.googlesource.com/ceres-solver/+/master/examples/slam/pose_graph_3d/) — 위 코드의 전체 버전
> - [Ceres Solver Tutorial](http://ceres-solver.org/tutorial.html) — AutoDiff, Manifold 개념 설명
> - [정진용 블로그 — Ceres Solver Tutorial](https://jinyongjeong.github.io/2023/07/22/Ceres_tutorial/) — Ceres Solver 발표자료와 GitHub 실습 코드. 비선형 최적화 입문에 적합

## 3.7 심화: Robust Estimation

*연구자가 되고 싶다면 여기서부터 읽어라.*

현실 세계의 데이터는 깨끗하지 않다. 잘못된 데이터 연관(false match), 동적 물체, 센서 고장이 outlier를 만들고, outlier는 최적화 결과를 심각하게 왜곡한다. Robust estimation은 이런 상황에서도 합리적인 추정을 내놓기 위한 기법이다.

### 3.7.1 왜 필요한가

Standard least squares는 에러의 제곱을 최소화한다: `rho(r) = r^2`. 이 함수는 큰 잔차(residual)에 큰 가중치를 주기 때문에, 하나의 outlier가 전체 해를 끌고 갈 수 있다.

SLAM에서의 구체적 사례:
- 잘못된 loop closure 하나가 전체 지도를 뒤틀어 버린다
- Visual feature matching에서의 false positive가 BA 결과를 망친다
- 동적 물체(사람, 차)에 붙은 feature가 정적 장면 가정을 위반한다

### 3.7.2 M-Estimator

M-estimator는 `rho(r) = r^2` 대신 다른 비용 함수 rho를 사용하여 outlier의 영향을 줄인다.

| M-Estimator | rho(r) | 특성 |
|---|---|---|
| **L2 (표준)** | r^2 | Outlier에 취약 |
| **Huber** | r^2 (abs(r) <= k), 2k*abs(r) - k^2 (abs(r) > k) | 작은 잔차는 L2, 큰 잔차는 L1. 가장 널리 쓰임 |
| **Cauchy** | c^2 * log(1 + (r/c)^2) | Huber보다 outlier 억제가 강함 |
| **Geman-McClure** | r^2 / (1 + r^2) | 극단적 outlier를 사실상 무시 |

Huber가 대부분의 경우 안전한 기본 선택이다. Outlier 비율이 높거나 극단적인 경우 Cauchy나 Geman-McClure를 고려한다. 파라미터(k 또는 c)는 잔차의 통계적 분포에 맞춰 튜닝해야 한다.

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
| Lo-RANSAC | 좋은 모델 발견 시 로컬 최적화 추가 | 정확도 향상, 속도 감소 |
| MAGSAC++ | 노이즈 스케일 sigma 자동 추정, soft inlier/outlier | 파라미터 프리에 가까우나 계산 비용이 높음 |

OpenCV의 `cv::findHomography`, `cv::findFundamentalMat` 등에서 `cv::USAC_MAGSAC` 플래그로 MAGSAC++를 사용할 수 있다.

> **추천 자료**
> - [State Estimation for Robotics, Ch.5 (Barfoot)](http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) — 바이어스, 대응 문제, outlier를 다루는 실전적 챕터
> - [Hartley & Zisserman, Ch.4 -- Estimation: 2D Projective Transforms](https://www.robots.ox.ac.uk/~vgg/hzbook/) — RANSAC의 원본 설명과 robust estimation 이론
> - [다크 프로그래머 — RANSAC의 이해와 영상처리 활용](https://darkpgmr.tistory.com/61) — RANSAC의 원리, threshold 설정, 반복 횟수 계산을 한글로 설명
> - [정진용 블로그 — Bundle Adjustment의 Jacobian 계산](https://jinyongjeong.github.io/2020/03/01/Jacobian_of_BA/) — BA의 reprojection error Jacobian을 Lie algebra와 quaternion으로 유도. 손필기 수식 포함

> **실습**: [RANSAC 시각화](https://alexjunholee.github.io/robotics-practice/app.html#ransac)
> Outlier가 포함된 데이터에서 RANSAC이 inlier/outlier를 분류하고 모델을 피팅하는 과정을 단계별로 확인할 수 있다.

## 3.8 심화: 정보 이론 기초

*연구자가 되고 싶다면 여기서부터 읽어라.*

Active SLAM, exploration, 불확실성 기반 의사결정에서 정보 이론 개념이 쓰인다. 핵심만 짚는다.

**Shannon entropy**: 확률 변수 X의 불확실성을 측정한다.

```
H(X) = -sum  p(x) log p(x)
```

Entropy가 높을수록 불확실성이 크다. 가우시안 분포의 경우 공분산이 클수록 entropy가 높다.

**KL divergence (Kullback-Leibler divergence)**: 두 확률 분포 p와 q 사이의 "차이"를 측정한다.

```
D_KL(p || q) = sum  p(x) log(p(x) / q(x))
```

비대칭이다: D_KL(p||q) != D_KL(q||p). "p라고 생각했는데 실제로 q일 때의 정보 손실"로 해석할 수 있다.

**Mutual information**: Y를 관측하면 X에 대해 얼마나 알게 되는가를 측정한다.

```
I(X; Y) = H(X) - H(X|Y)
```

H(X)는 Y를 관측하기 전 X의 불확실성, H(X|Y)는 관측 후 불확실성. 그 차이가 Y가 X에 대해 제공하는 정보량이다.

**Active SLAM 응용**: 로봇이 다음에 어디로 갈지 결정할 때, "이 행동을 취하면 지도/포즈의 불확실성이 얼마나 줄어드는가?"를 mutual information으로 수치화할 수 있다. Expected information gain이 가장 큰 행동을 선택하는 것이 정보 이론 기반 탐색의 핵심이다.

```
a* = argmax_a  I(X; Z_a)  =  argmax_a  [ H(Z_a) - H(Z_a | X) ]
```

여기서 a는 행동(action), Z_a는 그 행동을 통해 얻을 관측, X는 환경 상태이다.

> **추천 자료**
> - [Elements of Information Theory (Cover & Thomas)](https://onlinelibrary.wiley.com/doi/book/10.1002/047174882X) — 정보 이론 교과서
> - [Placed: An exploration planner using information gain (2022)](https://arxiv.org/abs/2206.05193) — Active SLAM에서 정보 이론 활용 예시

> **기술 흐름: 로보틱스 수학 및 최적화**
> - **~2005**: 칼만 필터(EKF) 중심의 상태 추정. 선형 근사 기반, 소규모 문제에 적합. 실시간 처리가 어려워 문제 크기에 제약이 있었다.
> - **2006~2015**: Factor Graph 기반 최적화(iSAM, g2o, GTSAM) 등장. 스파스 행렬 구조를 활용해 대규모 SLAM 문제를 효율적으로 풀었다. Lie Group/Algebra가 SLAM 커뮤니티에서 표준 도구로 자리잡았다.
> - **2016~2020**: 실시간 대규모 최적화 실용화. 증분적 최적화(incremental optimization)로 매 프레임 실시간 업데이트가 가능해졌다. Ceres Solver가 산업계 표준으로 자리잡았다.
> - **2021~**: Differentiable Programming 시대. PyTorch/JAX의 자동 미분(Auto-Diff)을 활용한 End-to-End 최적화. NeRF, 3D Gaussian Splatting 등 미분 가능 렌더링이 등장하면서, 기존에 손으로 유도하던 Jacobian을 자동 미분으로 대체했다. Theseus(Meta) 같은 미분 가능 최적화 라이브러리도 나왔다.
> - **지금**: 고전적 수학(Lie Group, 확률, 최적화)은 여전히 필수다. Differentiable Programming이 최적화 문제 접근 방식을 바꾸고 있지만, 자동 미분이 내부에서 무엇을 하는지 이해하려면 여기서 다룬 기초가 필요하다. 도구만 쓸 줄 알면 디버깅할 수 없다.
