# Ch.14 — SLAM & Odometry


SLAM(Simultaneous Localization and Mapping)은 낯선 환경에서 로봇의 위치를 추정하는 동시에 주변 지도를 만드는 문제다. 실내나 지하처럼 GPS 신호가 닿지 않는 곳에서 로봇이 스스로 움직이려면 SLAM이 있어야 한다.

---

## Part 1. 기초와 시스템

### 14.1 개념 소개

내비게이션과 경로 계획에는 로봇의 현재 위치와 주변 환경 정보가 필요하다. SLAM은 두 정보를 함께 추정한다.

닭과 달걀 문제:
- 지도가 있어야 위치를 알 수 있음
- 위치를 알아야 지도를 만들 수 있음
→ 동시에 해결

센서 측정에는 노이즈가 있다. 바퀴는 미끄러진다. 카메라 영상은 흔들린다. 이 오차가 쌓여 위치 추정이 점점 틀어진다(drift). SLAM은 drift를 보정하면서 일관된 지도를 만든다.

**Odometry vs SLAM**:
| 특징 | Odometry | SLAM |
|---|---|---|
| 출력 | 상대적 이동 | 위치 + 지도 |
| Loop Closure | 없음 | 있음 |
| Drift | 누적 | 보정 가능 |
| 계산량 | 적음 | 많음 |

> **추천 자료**
> - [Cyrill Stachniss — SLAM Course (2013)](https://www.youtube.com/playlist?list=PLgnQpQtFTOGQrZ4O5QzbIHgl3b1JHimN_) — Bayes filter부터 graph-based SLAM까지 이어지는 공개 강의 시리즈
> - [Thrun, Burgard, Fox, "Probabilistic Robotics" (Textbook)](https://mitpress.mit.edu/9780262201629/probabilistic-robotics/) — SLAM의 수학적 기반을 다루는 교과서. 칼만 필터, 입자 필터, EKF-SLAM 등
> - [Barfoot, "State Estimation for Robotics" (Free PDF)](https://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) — 상태 추정의 수학을 깊이 있게 다루는 교재. 무료 PDF가 있다
> - [Awesome-SLAM GitHub](https://github.com/SilenceOverflow/Awesome-SLAM) — SLAM 관련 논문, 라이브러리, 데이터셋을 모아놓은 목록
> - [정진용 블로그 — SLAM 강의 시리즈 (Freiburg Robot Mapping 기반)](https://jinyongjeong.github.io/2017/02/13/lec01_SLAM_bayes_filter/) — Bayes filter부터 EKF/UKF/Particle filter, Graph SLAM, Robust SLAM까지 잇는 15편의 한국어 시리즈
> - [김기섭 블로그 — SLAM Back-end 공부자료 5개 추천](https://gisbi-kim.github.io/blog/2021/10/03/slam-textbooks.html) — Error-state KF, Factor Graphs, Bundle Adjustment 등 핵심 자료 큐레이션
> - [Robot Mapping Course (Uni Freiburg, Cyrill Stachniss)](http://ais.informatik.uni-freiburg.de/teaching/ws13/mapping/) — SLAM 강의 슬라이드와 과제 자료. 영상과 함께 보면 좋다
> - [EKF-SLAM 슬라이드 (Freiburg)](http://ais.informatik.uni-freiburg.de/teaching/ws12/mapping/pdf/slam04-ekf-slam.pdf) — 위 강의 중 EKF-SLAM 파트. 수식 전개가 깔끔하게 정리되어 있다

> **실습**: [SE(2) Odometry](https://alexjunholee.github.io/robotics-practice/app.html#se2_odometry)
> 2D 평면에서 odometry가 쌓이는 과정을 직접 조작하며 drift가 어떻게 발생하는지 확인할 수 있다.

### 14.2 Visual Odometry (VO)

카메라 영상만으로 로봇이 얼마나 움직였는지 추정한다. SLAM에서 "front-end"를 맡는 부분이라, 여기서 이동을 잘못 추정하면 SLAM 전체가 부정확해진다.

#### 14.2.1 Feature-based vs Direct Method

어느 방식을 쓸지는 로봇이 다닐 환경을 보고 정한다.

**Feature-based** (ORB-SLAM 계열)는 이미지에서 시점이 바뀌어도 다시 찾을 수 있는 특징점(코너, 블롭 등)을 추출한다. 프레임 사이에서 특징점을 짝지어 카메라 움직임을 거꾸로 계산한다. 계산은 두 단계로 나뉜다. 맵이 없는 초기화에서는 2D-2D 대응으로 Essential 또는 Fundamental Matrix(평면에서는 Homography)를 구한다. 맵이 생긴 뒤의 추적에서는 3D 맵 포인트와 2D 관측의 재투영 오차를 최소화한다. 조명 변화에 강하고 오래 써 온 방식이다. 흰 벽이나 무늬 없는 바닥처럼 특징점이 적은 환경에서는 추적이 끊기기 쉽다.

```
이미지 → 특징점 추출 → 매칭 → 움직임 추정
```

**Direct Method** (DSO, LSD-SLAM 계열)는 픽셀 밝기를 직접 비교한다. "연속 프레임에서 같은 3D 점을 관측하면 밝기가 같아야 한다"는 가정(brightness constancy)을 쓴다. 특징점을 뽑지 않으므로 텍스처가 적은 환경에서도 돌아갈 때가 있다. 대신 조명이 바뀌면 쉽게 틀린다.

```
이미지 → 픽셀 밝기 직접 비교 → 움직임 추정
```

#### 14.2.2 Mono vs Stereo vs RGB-D

| 구성 | Scale | 특징 | 적합 환경 |
|---|---|---|---|
| Monocular | 불가 (ambiguity) | 경량·단순, IMU 없이 스케일 복원 불가 | 저비용 드론, 모바일 |
| Stereo | 가능 | 베이스라인이 측정 범위를 제한 | 일반 실내·실외 |
| RGB-D | 가능 | 깊이 직접 측정, 실외·직사광선에 취약 | 실내 구조화 환경 |

Scale ambiguity는 단안 카메라에서 "가까이 있는 작은 물체"와 "멀리 있는 큰 물체"가 같은 영상을 만드는 데서 생긴다. 모노 SLAM의 지도는 임의의 스케일로 나오므로 IMU나 다른 센서로 복원해야 한다. 단안 카메라로 처음 3D 구조를 잡으려면 카메라가 옆으로 움직여 시차(parallax)가 생겨야 한다. 제자리에서 돌기만 하면 두 영상만으로는 깊이를 풀 수 없다. 카메라와 IMU를 함께 초기화할 때도 scale, IMU bias, 중력 방향이 드러날 만큼 가속하고 회전해야 한다. IMU 없이 단안 카메라만 쓰면 아무리 많이 움직여도 절대 scale은 정해지지 않는다.

> **추천 자료**
> - [Daniel Cremers — Multiple View Geometry (TUM)](https://www.youtube.com/playlist?list=PLTBdjV_4f-EJn6udZ34tht9EVIW7lbeo4) — Visual Odometry에 필요한 다중 뷰 기하를 다루는 공개 강의
> - [EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) — Visual(-Inertial) Odometry 벤치마크 데이터셋
> - [TUM RGB-D Benchmark](https://cvg.cit.tum.de/data/datasets/rgbd-dataset) — RGB-D SLAM/VO에서 널리 쓰이는 실내 데이터셋과 평가 도구

### 14.3 Visual SLAM

#### 14.3.1 ORB-SLAM2/3

ORB-SLAM 계열은 Visual SLAM 논문이 비교 기준(baseline)으로 자주 쓰는 공개 시스템이다. 코드가 공개돼 있어 직접 빌드해서 어떤 입력에서 잘 되고 어디서 실패하는지 확인해 볼 수 있다.

**구성**:
1. **Tracking**: 현재 프레임에서 포즈 추정
2. **Local Mapping**: 키프레임 기반 지역 지도 관리
3. **Loop Closing**: 루프 감지 및 전역 최적화

ORB-SLAM은 이 세 일을 스레드 세 개로 나눠 돌린다. Tracking은 프레임마다 돈다. Local Mapping은 키프레임이 들어올 때 돌고, Loop Closing은 새 키프레임마다 루프가 있는지 보고, 있으면 보정한다. 주기가 다른 일을 병렬로 돌리므로 Tracking이 실시간으로 도는 동안 지도 전체의 일관성도 맞춘다.

**ORB-SLAM3 특징**:
- Visual-Inertial 모드 지원
- 멀티맵 지원
- Fish-eye 카메라 지원

ORB-SLAM의 역사적 맥락:
- **MonoSLAM (2007)**: 초기의 실시간 단안 SLAM. EKF로 상태를 추정해서 지도가 커질수록 계산량이 빠르게 늘었다.
- **PTAM (Parallel Tracking and Mapping, 2007)**: Tracking과 Mapping을 서로 다른 thread로 나눴다. ORB-SLAM이 이 구조를 이어받았다.
- **ORB-SLAM (2015)**: PTAM의 구조에 ORB 특징점과 Loop Closure를 더하고, 시점이 크게 바뀌어도 동작하는 재위치 추정(relocalization)을 넣었다.
- **ORB-SLAM2 (2017)**: Stereo, RGB-D 지원 추가.
- **ORB-SLAM3 (2021)**: Visual-Inertial, 멀티맵 등 추가.

```bash
# ORB-SLAM3 실행 예시
./Examples/Monocular/mono_euroc \
    Vocabulary/ORBvoc.txt \
    Examples/Monocular/EuRoC.yaml \
    ~/Datasets/EuRoC/MH01
```

> **추천 자료**
> - [Campos et al., "ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM" (2021)](https://arxiv.org/abs/2007.11898) — ORB-SLAM3 논문
> - [ORB-SLAM3 GitHub](https://github.com/UZ-SLAMLab/ORB_SLAM3) — 공식 코드
> - [EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) — ORB-SLAM3 테스트용 표준 데이터셋
> - [정진용 블로그 — Visual SLAM 비교 실험 (KAIST Urban Dataset)](https://jinyongjeong.github.io/2019/10/22/visual_slam_compare/) — ORB-SLAM2 vs VINS-Fusion 실전 비교. 실제 데이터셋으로 성능 차이를 분석

#### 14.3.2 DSO (Direct Sparse Odometry)

Direct Method + Sparse Points

Direct Method는 보통 모든 픽셀을 쓰고(dense), 점을 골라 쓰는 sparse 방식은 주로 Feature-based에서 쓴다. DSO는 direct 방식이면서 점을 골라 쓴다. 밝기 변화가 뚜렷한 점만 골라 광도(photometric) 오차를 최소화한다.

- 특징점 추출 없이 픽셀 밝기 직접 사용
- 선별된 점들만 사용 (Sparse)
- Photometric bundle adjustment

> **추천 자료**
> - [Engel et al., "Direct Sparse Odometry" (2018)](https://arxiv.org/abs/1607.02565) — DSO 논문

#### 14.3.3 VINS-Mono/Fusion

단안 카메라만으로는 빠르게 움직이거나 텍스처가 적은 곳에서 추적이 끊기기 쉽다. IMU는 영상 프레임 사이의 빠른 움직임을 재서 이 빈틈을 메운다. VINS-Mono는 카메라와 IMU를 결합한 드론·모바일 로봇용 Visual-Inertial SLAM 시스템이다.

**Visual-Inertial Navigation System**

- Camera + IMU tight coupling
- Sliding window optimization
- Loop closure 지원
- 모바일/드론에서 널리 사용

```
센서 입력 → IMU Preintegration →
Visual Feature Tracking →
Sliding Window Optimization →
Loop Closure (optional)
```

VINS-Mono는 IMU preintegration으로 두 키프레임 사이의 IMU 측정 여러 개를 상대 운동 제약 하나로 묶는다. 최적화할 때 원시 측정을 다시 적분하지 않고 이 제약을 쓴다.

> **추천 자료**
> - [Qin et al., "VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator" (2018)](https://arxiv.org/abs/1708.03852) — VINS-Mono 논문
> - [VINS-Mono GitHub](https://github.com/HKUST-Aerial-Robotics/VINS-Mono) — 공식 코드, ROS 지원

### 14.4 LiDAR Odometry & SLAM

카메라 기반 방법은 조명과 텍스처의 영향을 받는다. LiDAR는 레이저로 3D 거리를 직접 측정한다. 이런 차이 때문에 자율주행과 실외 로봇에서 LiDAR SLAM을 쓴다.

#### 14.4.1 LOAM (Lidar Odometry and Mapping)

LOAM의 edge·planar feature 분리와 odometry-mapping 이중 주기 구조는 LeGO-LOAM, LIO-SAM 등 후속 LiDAR SLAM에 영향을 주었다.

- Edge points와 Planar points 분류
- Point-to-edge, point-to-plane 거리 최소화
- Odometry와 Mapping 분리 (주파수 다르게)

LOAM은 포인트 클라우드에서 모서리와 평면에 놓인 점만 추린다. 모든 점을 짝짓지 않고 이 점들로만 제약을 세워서 계산량을 줄인다.

#### 14.4.2 LeGO-LOAM

**Lightweight and Ground-Optimized LOAM**:
- 지면 분리로 계산량 감소
- 지면을 기반으로 초기 추정
- 모바일 로봇에 적합

#### 14.4.3 LIO-SAM

LIO-SAM은 LiDAR와 IMU를 Factor Graph로 최적화하는 SLAM이다. Factor Graph에서는 새 센서의 측정을 factor로 붙이기만 하면 되므로 센서를 늘리기 쉽다.

**LiDAR-Inertial Odometry via Smoothing and Mapping**:
- Factor graph 기반
- Tight IMU-LiDAR coupling
- GPS, Loop closure 통합

```
                    ┌──────────────┐
IMU ──────────────→ │              │
                    │ Factor Graph │ ──→ Pose
LiDAR ────────────→ │              │
                    │  iSAM2       │
GPS (optional) ───→ │              │
                    └──────────────┘
```

Factor Graph는 변수(로봇 포즈, 랜드마크 위치)와 제약(센서 측정)의 관계를 그래프로 표현한다. IMU 측정, LiDAR 매칭, GPS, Loop Closure가 각각 factor가 된다. 센서를 늘릴 때는 그 센서의 factor를 더한다. 최적화는 GTSAM 라이브러리가 푼다.

> **추천 자료**
> - [Shan et al., "LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping" (2020)](https://arxiv.org/abs/2007.00258) — LIO-SAM 논문
> - [Vizzo et al., "KISS-ICP: In Defense of Point-to-Point ICP" (RA-L 2023, arXiv:2209.15397)](https://arxiv.org/abs/2209.15397) — 잘 다듬은 point-to-point ICP로 복잡한 LiDAR odometry와 비슷한 성능을 보고
> - [LIO-SAM GitHub](https://github.com/TixiaoShan/LIO-SAM) — 공식 코드, ROS 지원
> - [GTSAM Documentation](https://gtsam.org/) — Factor Graph 최적화 라이브러리. LIO-SAM 같은 SLAM 시스템이 백엔드로 쓴다
> - [Frank Dellaert — Factor Graphs for Perception and Action (MIT Robotics)](https://www.youtube.com/watch?v=-yCC7mpgL4w) — GTSAM 개발자가 직접 설명하는 Factor Graph
> - [김기섭 블로그 — Scan Context-based LiDAR Pose-graph SLAM 구현](https://gisbi-kim.github.io/blog/2021/05/17/sclidarslam.html) — Scan Context를 LiDAR SLAM에 통합한 구현 해설

#### 14.4.4 FAST-LIO / FAST-LIO2

**Fast LiDAR-Inertial Odometry**:
- Iterated Kalman Filter 기반 (Gauss-Newton과 동등한 반복 갱신)
- ikd-Tree: 동적 KD-트리로 빠른 매핑
- 실시간 성능

LIO-SAM은 비선형 최소자승 기반의 Factor Graph 최적화를 쓴다. FAST-LIO는 반복 확장 칼만 필터(Iterated EKF, IEKF)를 쓴다. IEKF의 반복 갱신은 MAP 목적함수의 Gauss-Newton과 동등하다. FAST-LIO가 빠른 까닭은 Kalman gain 공식에 있다. 원래 공식은 측정 차원이 커질수록 계산이 무거워지는데, FAST-LIO는 공식을 바꿔 계산량이 상태 차원에 따르게 했다. 상태를 슬라이딩 윈도로 쌓지 않는 것도 속도에 보탬이 된다. 후속작 FAST-LIO2는 점을 하나씩 넣고 뺄 수 있는 KD-트리인 ikd-Tree를 더해 맵에 새 점을 넣는 비용도 줄였다.

> **추천 자료**
> - [Xu & Zhang, "FAST-LIO: A Fast, Robust LiDAR-Inertial Odometry Package by Tightly-Coupled Iterated Kalman Filter" (2021)](https://arxiv.org/abs/2010.08196) — FAST-LIO 논문
> - [Xu et al., "FAST-LIO2: Fast Direct LiDAR-Inertial Odometry" (2022)](https://arxiv.org/abs/2107.06829) — FAST-LIO2 논문
> - [FAST-LIO2 GitHub](https://github.com/hku-mars/FAST_LIO) — 공식 코드

### 14.5 Multi-sensor Fusion

카메라는 어두운 곳에서 성능이 떨어진다. LiDAR는 비와 안개에 약하다. IMU는 적분만 하면 drift가 빠르게 커진다. 여러 센서를 함께 쓰면(fusion) 한 센서가 약한 조건을 다른 센서가 메운다.

#### 14.5.1 Camera + IMU (VIO)

카메라와 IMU를 묶는 방식은 크게 둘이다. Loosely-coupled 방식에서는 카메라와 IMU가 각자 상태를 추정하고, 그 결과를 covariance로 가중해 합친다. 구현은 단순하지만 두 센서 오차 사이의 상관관계를 버린다. Tightly-coupled 방식에서는 카메라 특징점의 재투영 오차와 IMU 측정이 같은 상태 변수를 직접 제약한다. 더 정확하지만 구현이 복잡하다. VINS-Mono는 두 오차를 비용 함수 하나에 넣어 최적화한다. MSCKF는 sliding window EKF로 풀면서 특징점 변수를 null-space 투영으로 상태에서 뺀다.

#### 14.5.2 LiDAR + IMU (LIO)

회전형 LiDAR는 한 바퀴를 도는 동안 점을 차례로 찍는다. 그사이 로봇이 움직이면 한 scan 안의 점들이 서로 다른 위치에서 찍혀 형상이 일그러진다(motion distortion). LIO는 LiDAR보다 측정 주기가 빠른 IMU와 timestamp로 scan 안의 움직임을 보정(de-skewing)하고, LiDAR 관측과 함께 상태를 추정한다. 보정이 얼마나 도움이 되는지는 시간 동기화, IMU bias, 움직임의 크기, scan 패턴에 따라 다르다.

#### 14.5.3 Camera + LiDAR + IMU

**세 센서 통합**
- 예시: R3LIVE, LVI-SAM
- 각 센서의 장점 활용

R3LIVE는 기하 정보를 주는 LiDAR, 색을 주는 카메라, 빠른 움직임을 재는 IMU를 함께 쓴다. 위치를 추정하면서 색이 입혀진(colored) 조밀한 3D 맵도 실시간으로 만든다.

> **추천 자료**
> - [KITTI Odometry Benchmark](https://www.cvlibs.net/datasets/kitti/eval_odometry.php) — LiDAR/Visual Odometry 벤치마크의 표준
> - [EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) — VIO 벤치마크 데이터셋
> - [Lin & Zhang, "R3LIVE: A Robust, Real-time, RGB-colored, LiDAR-Inertial-Visual tightly-coupled state Estimation and mapping package" (2022)](https://arxiv.org/abs/2109.07982) — 3센서 융합의 대표작
> - [김기섭 블로그 — Filter-based VIO: MSCKF 계열 history 정리](https://web.archive.org/web/20240820015803/https://gisbi-kim.github.io/blog/2021/04/27/msckf-history.html) — MSCKF 원본부터 stereo 확장까지 후속 연구 정리

### 14.6 Loop Closure & Global Optimization

SLAM을 오래 돌리면 지도가 점점 뒤틀린다. 로봇이 큰 원을 그리며 출발점으로 돌아와도 지도에서는 시작점과 끝점이 어긋난다. Loop Closure는 전에 왔던 장소를 알아보고 이 오차를 바로잡는다. 넓은 환경에서 쌓이는 drift를 잡으려면 이 과정이 있어야 한다.

#### 14.6.1 Place Recognition

같은 장소라도 시간, 조명, 계절이 바뀌면 아주 다르게 보인다. 거꾸로 비슷하게 생긴 다른 장소를 같은 곳으로 잘못 판정하면(false positive) 지도 전체가 크게 일그러진다. 그래서 Place Recognition은 정밀도(precision)가 아주 높아야 한다.

**Bag of Words (BoW)**: 특징점을 미리 학습한 시각 단어(visual vocabulary)로 바꿔 이미지끼리 얼마나 비슷한지 계산한다. ORB-SLAM은 DBoW2 라이브러리를 쓴다. 빠르고 오래 써 왔지만 조명과 시점이 바뀌면 약하다.

**NetVLAD**: 신경망을 end-to-end로 학습해서 조명과 날씨가 바뀌어도 잘 버티는 이미지 전체 기술자(global descriptor)를 만든다. (14.14절 참조)

**LiDAR Place Recognition**: Scan Context는 3D 포인트 클라우드를 위에서 내려다본(bird's-eye view) 2D 기술자로 압축한다. PointNetVLAD는 원시 포인트 집합에서 전역 임베딩을 신경망으로 학습한다.

#### 14.6.2 Pose Graph Optimization

루프가 감지되면 전체 경로를 보정한다.

```
노드: 로봇 포즈
에지: 상대 변환 (odometry, loop closure)

목표: 모든 에지 제약을 만족하는 노드 위치 찾기
```

Odometry가 만든 경로는 구간마다 대충 맞지만 전체로 보면 뒤틀려 있다. Loop Closure가 "이 위치와 저 위치는 같은 곳"이라는 제약을 더하면, Pose Graph Optimization이 모든 제약을 되도록 만족하게 전체 경로를 고르게 고친다. 이 계산은 비선형 최소자승 문제다.

주요 도구로는 경량 pose graph/BA 전용인 **g2o** (ORB-SLAM), factor graph와 iSAM2 기반의 **GTSAM** (LIO-SAM), Google이 개발한 범용 비선형 최소자승 라이브러리 Ceres Solver가 있다. 선택 기준은 14.9.4절의 비교표를 참고하라.

> **추천 자료**
> - [GTSAM Documentation & Tutorials](https://gtsam.org/) — Factor Graph 기반 최적화 라이브러리. Pose Graph Optimization 예제 포함
> - [Cyrill Stachniss — Graph-based SLAM](https://www.youtube.com/watch?v=uHbRKvD8TWg) — Pose Graph Optimization의 직관적 설명
> - [g2o GitHub](https://github.com/RainerKuemmerle/g2o) — Graph Optimization 프레임워크
> - [정진용 블로그 — Robust Graph SLAM](https://jinyongjeong.github.io/2017/03/04/lec15_Robust_Graph_SLAM/) — M-estimator, Max-mixture, DCS 등 robust SLAM 기법 한글 해설

> **실습**: [Pose Graph Optimization](https://alexjunholee.github.io/robotics-practice/app.html#pose_graph)
> Pose Graph의 노드(포즈)와 에지(제약)를 조작하고, loop closure 추가 시 전체 경로가 어떻게 보정되는지 확인할 수 있다.

### 14.7 Localization

Localization은 이미 만든 지도 안에서 현재 위치만 추정한다. 서비스 로봇은 흔히 오프라인 SLAM으로 정밀 지도를 먼저 만든다. 운용 중에는 그 지도에서 Localization을 돌린다.

MCL 알고리즘의 원리와 유도는 §3.11 비모수 필터에, EKF의 수학은 §3.10에 있다. IMU를 더한 위치추정은 §14.10에서 다룬다.

지도를 미리 만들어 두므로 SLAM보다 계산이 가볍다. 대신 환경이 바뀌면 지도를 고쳐야 한다.

**Monte Carlo Localization (MCL)**:
- 입자 필터 기반
- 2D LiDAR + 점유 격자 지도
- ROS AMCL 패키지

MCL은 수천 개의 "가상 로봇(파티클)"을 지도 위에 뿌린다. 파티클 하나는 "로봇이 이 위치에서 이 방향을 보고 있다"는 가설 하나다. 실제 센서 측정과 잘 맞는 파티클은 남고 안 맞는 파티클은 사라진다. 이를 되풀이하면 파티클이 실제 위치 주변에 모인다.

**LiDAR Localization**: 현재 scan을 포인트 클라우드 맵에 ICP나 NDT로 맞춰 위치를 정밀하게 추정한다.

> **추천 자료**
> - [Cyrill Stachniss — Monte Carlo Localization](https://www.youtube.com/watch?v=MsYlueVDLI0) — MCL/입자 필터의 직관적 설명
> - [ROS Navigation Stack — AMCL](https://wiki.ros.org/amcl) — ROS 1에서 MCL 사용하기

> **실습**: [Particle Filter](https://alexjunholee.github.io/robotics-practice/app.html#particle_filter)
> 입자 필터 기반 로봇 위치 추정 과정을 시각화하며, 파티클의 수렴 과정을 인터랙티브하게 확인할 수 있다.

> **실습**: [Occupancy Grid](https://alexjunholee.github.io/robotics-practice/app.html#occupancy_grid)
> 2D 점유 격자 지도를 구축하는 과정을 시각화하며, 센서 측정이 어떻게 확률적 지도로 변환되는지 확인할 수 있다.

#### 14.7.1 위치추정 문제 분류

위치추정의 난이도는 다음 네 축으로 본다.

| 축 | 옵션 | 비고 |
|---|---|---|
| 사전지식 | position tracking → global localization → kidnapped robot | 난이도 상승 |
| 환경 | static (로봇만 이동) → dynamic (사람·문·조명) | 동적일수록 어려움 |
| 능동성 | passive (관찰만) → active (탐색 행동 선택) | active가 더 빠른 수렴 |
| 로봇 수 | single → multi (상호 관측으로 belief 공유) | multi는 정보 풍부 |

대표 시나리오는 position tracking과 global localization이고, kidnapped robot은 global localization을 더 어렵게 만든 경우다.

**Position tracking**: 처음 자세를 알고 시작하므로 belief가 좁은 단봉 Gaussian으로 남는다. EKF Localization이 잘 맞는다.

**Global localization**: 처음 자세를 모른다. 균등 분포에서 시작해 측정이 쌓이면서 belief가 한곳으로 모여야 한다. 봉우리가 여럿인(multi-modal) belief를 나타내야 하므로 Grid Localization이나 MCL을 쓴다.

**Kidnapped robot**: 운용 중에 누군가 로봇을 다른 곳으로 옮긴다. 로봇은 옮겨진 줄 모르고 이전 belief를 그대로 믿으므로 global localization보다 어렵다. Thrun 등은 이 복구 능력을 localization 알고리즘의 시험대로 쓴다.

ROS Nav2의 `recovery_alpha_slow/fast` 파라미터는 kidnapped 상황에 대비해 둔 것이다. 물류 창고 AGV나 청소 로봇이 켜질 때는 global localization 문제를, 돌아다니는 동안은 tracking 문제를 푼다.

#### 14.7.2 Markov Localization

Markov localization은 베이즈 필터를 위치추정에 그대로 쓴 틀이다. EKF Localization, Grid Localization, MCL은 이 필터의 belief를 무엇으로 나타내는지만 다르다.

베이즈 필터(Ch.3 §3.9)와 다른 점은 운동 모델과 관측 모델에 지도 m이 입력으로 더 들어간다는 것 하나다.

```
Markov_localization(bel(x_{t-1}), u_t, z_t, m):
  for all x_t do
    bel̄(x_t) = ∫ p(x_t | u_t, x_{t-1}, m) bel(x_{t-1}) dx_{t-1}   // motion update
    bel(x_t) = η p(z_t | x_t, m) bel̄(x_t)                          // measurement update
  endfor
  return bel(x_t)
```

처음 belief bel(x_0)는 시나리오마다 다르게 잡는다.
- Position tracking: $\text{bel}(x_0) = \mathcal{N}(x_0;\, \bar{x}_0, \Sigma)$ — 좁은 Gaussian
- Global localization: $\text{bel}(x_0) = 1/|X|$ — 로봇이 있을 수 있는 모든 자세에 균등
- Partial knowledge: 알려진 구역 근방에만 균등, 그 외 0

§14.7.3~§14.7.7의 알고리즘은 위 의사코드의 bel을 무엇으로 나타내는지에 따라 나뉜다.

#### 14.7.3 EKF Localization

EKF Localization은 belief를 가우시안 $(\mu_t, \Sigma_t)$로 나타내는 Markov localization이다. 봉우리가 하나(unimodal)라고 가정하므로 position tracking에 쓴다. Global localization과 kidnapped 문제에는 다봉 belief를 표현하는 MHT, Grid, MCL을 쓴다.

EKF 자체는 §3.10.2에 있다. 여기서는 지도가 feature로 이뤄져 있고 랜드마크 대응을 안다고 가정한 알고리즘을 본다.

**가정**: 지도 m이 feature 기반 (점 랜드마크 집합). 각 측정 $z_t^i = (r, \phi, s)^T$ (range, bearing, signature). 대응 $c_t^i$는 안다고 가정한다(ARTag, QR 코드, 에펠탑처럼 무엇인지 알아볼 수 있는 랜드마크).

```
EKF_localization_known_correspondences(μ_{t-1}, Σ_{t-1}, u_t, z_t, c_t, m):
  // Motion update (velocity model 선형화)
  μ̄_t = μ_{t-1} + [velocity model 변위]
  G_t = ∂g/∂x |_{μ_{t-1}, u_t}         // 3×3 Jacobian
  Σ̄_t = G_t Σ_{t-1} G_t^T + R_t

  // Measurement update (랜드마크별 순차 갱신)
  μ_t = μ̄_t
  Σ_t = Σ̄_t
  for each observed z_t^i = (r, φ, s)^T do
    j = c_t^i
    δ = (m_{j,x} − μ_{t,x},  m_{j,y} − μ_{t,y})^T,   q = δ^T δ
    ẑ_t^i = (√q,  atan2(δ_y, δ_x) − μ_{t,θ},  m_{j,s})^T
    H_t^i = Jacobian (3×3, 마지막 행은 0 — signature는 pose와 무관)
    K_t^i = Σ_t H_t^{i,T} (H_t^i Σ_t H_t^{i,T} + Q_t)^{-1}
    μ_t = μ_t + K_t^i (z_t^i − ẑ_t^i)
    Σ_t = (I − K_t^i H_t^i) Σ_t
  endfor
  return μ_t, Σ_t
```

측정이 서로 조건부 독립이면($p(z_t | x_t, m) = \prod_i p(z_t^i | x_t, m)$) 측정을 한데 쌓아 한 번에 갱신해도 되고 하나씩 갱신해도 된다. 위 코드는 측정 하나로 갱신한 $(\mu_t, \Sigma_t)$를 다음 측정에 쓰는 순차형이다. 모델이 비선형이면 매번 다시 선형화하는지에 따라 한 번에 갱신한 결과와 조금 달라진다.

*Probabilistic Robotics*는 경험칙(rule of thumb)으로, heading θ의 표준편차가 ±20°를 넘으면 선형화 오차 때문에 알고리즘이 실패하기 쉽다고 적는다. EKF와 MHT 모두에 해당한다. 경험으로 잡은 값이라 어디에나 맞지는 않는다. 관측 기하, 움직임, 노이즈에 따라 NIS·NEES나 Monte Carlo로 일관성을 확인해야 한다. ARTag·AprilTag처럼 알아볼 수 있는 랜드마크를 쓰거나 GNSS와 IMU를 합칠 때처럼 belief가 단봉으로 남는 문제에는 EKF localization을 계속 쓸 수 있다.

**미지 대응(unknown correspondences)**: 실전에서 $c_t^i$는 보통 모른다. ML(maximum likelihood) data association은 마할라노비스 거리가 가장 작은 지도 랜드마크를 고른다.

$$j(i) = \arg\min_k (z_t^i - \hat{z}_t^k)^T \Psi_k^{-1} (z_t^i - \hat{z}_t^k), \quad \Psi_k = H_t^k \bar\Sigma_t H_t^{k,T} + Q_t$$

마할라노비스 거리 최소화는 공분산 determinant와 prior가 후보마다 같으면 Gaussian log-likelihood 최대화와 같다. 실제로는 측정 차원에 맞춘 $\chi^2$ gate와, 한 프레임의 측정과 랜드마크를 하나씩만 짝짓는 one-to-one assignment 같은 제약을 더한다. ORB-SLAM도 descriptor matching으로 후보를 만든 뒤 geometric verification으로 outlier를 거르지만, 거르는 방식은 EKF의 ML association과 다르다.

#### 14.7.4 Multi-Hypothesis Tracking (MHT)

EKF의 단봉 Gaussian은 가설 하나만 담는다. MHT는 belief를 가우시안 혼합(Gaussian mixture)으로 나타내 여러 가설을 함께 들고 간다.

가설 $h$마다 EKF를 따로 돌린다. 측정이 들어오면 가설을 늘리고, 가중치(사후확률)가 임계값 $\psi_{\min}$ 아래로 떨어진 가설은 버린다. 이렇게 버리지 않으면 가설 수가 감당할 수 없이 늘어난다.

자율주행의 multi-object tracking(MOT)은 Mahalanobis gating과 Hungarian assignment를 자주 함께 쓴다. 이 조합은 프레임마다 가장 좋은 짝 하나를 확정하므로, 여러 가설을 시간이 지나도록 들고 가는 MHT와 구조가 다르다.

#### 14.7.5 Grid Localization

Grid Localization은 포즈 공간 전체를 격자로 나누고 셀마다 확률을 쌓는 히스토그램 필터다. global하고 봉우리가 여럿인 belief를 나타낼 수 있고, 계산 비용은 셀 수 $K$에 비례한다.

```
Grid_localization({p_{k,t-1}}, u_t, z_t, m):
  for all k do
    p̄_{k,t} = Σ_i p_{i,t-1} · motion_model(mean(x_k), u_t, mean(x_i))
    p_{k,t}  = η · measurement_model(z_t, mean(x_k), m) · p̄_{k,t}
  endfor
  return {p_{k,t}}
```

$\text{bel}(x_t) = \{p_{k,t}\}$이며 각 셀 $x_k$에 확률 하나, 합은 1이다.

**해상도 트레이드오프**: 격자가 촘촘할수록 위치를 칸 단위로 끊어서 생기는 오차가 줄어든다. 그 대신 global localization에 드는 CPU 계산량이 빠르게 늘어난다. 실시간으로 돌리려면 raycast 결과를 캐시에 저장해 두거나, 스캔을 솎아 쓰거나, 확률이 임계값을 넘는 셀만 갱신한다.

Grid Localization은 이산 격자로 global belief를 표현한다. 격자를 샘플 집합으로 바꾸면 MCL이 된다. ROS `amcl` 노드는 KLD-sampling을 쓰는 adaptive MCL 구현이다.

#### 14.7.6 MCL 알고리즘 (보강)

MCL의 원리와 유도는 §3.11에 있다. 위치추정 알고리즘으로 쓸 때의 전체 구조는 다음과 같다.

```
MCL(X_{t-1}, u_t, z_t, m):
  X̄_t = X_t = ∅
  for k = 1 to M do
    x_t^[k] = sample_motion_model(u_t, x_{t-1}^[k])    // motion proposal
    w_t^[k] = measurement_model(z_t, x_t^[k], m)        // likelihood weight
    X̄_t += ⟨x_t^[k], w_t^[k]⟩
  endfor
  for k = 1 to M do
    i ~ Categorical(w_t^[1], ..., w_t^[M])              // 중요도 비례 재샘플
    X_t += x_t^[i]
  endfor
  return X_t
```

MCL은 predict(sample), weight, resample을 되풀이한다. global localization이면 빈 공간 전체의 균등 분포에서 샘플 $M$개를 뽑아 시작하고, position tracking이면 좁은 Gaussian에서 뽑는다.

**계산 자원에 맞추기**: 다음 측정이 올 때까지 뽑을 수 있는 만큼 샘플을 뽑도록 $M$을 정하면, CPU가 빠를수록 $M$이 커지고 정확도도 오른다.

입자를 motion model에서 뽑기 때문에 센서가 아주 정확하면(측정 우도가 아주 좁으면) 거의 모든 입자의 가중치가 0에 가까워진다. Mixture MCL(§14.7.8)이 이 문제를 푼다.

ROS2 Nav2의 `nav2_amcl`은 이 구조에 KLD-sampling(§14.7.5)과 무작위 입자 주입(§14.7.7)을 더해 구현한다.

#### 14.7.7 Augmented MCL — 납치 복구

표준 MCL은 납치에 약하다. 입자가 한 자세로 모인 뒤 누군가 로봇을 옮기면 새 위치 근처에 입자가 하나도 없어서 되찾을 방법이 없다. Augmented MCL은 측정 우도의 단기 평균이 장기 평균보다 갑자기 낮아지면 무작위 입자를 넣는다. 센서가 갑자기 지도와 안 맞으면 길을 잃었다고 보는 것이고, 이를 두 지수이동평균의 비율로 판단한다.

```
Augmented_MCL(X_{t-1}, u_t, z_t, m):
  static w_slow, w_fast
  X̄_t = X_t = ∅,  w_avg = 0
  for k = 1 to M do
    x_t^[k] = sample_motion_model(u_t, x_{t-1}^[k])
    w_t^[k] = measurement_model(z_t, x_t^[k], m)
    X̄_t += ⟨x_t^[k], w_t^[k]⟩
    w_avg += w_t^[k] / M
  endfor
  w_slow += α_slow (w_avg − w_slow)    // 장기 평균 (천천히 변함)
  w_fast += α_fast (w_avg − w_fast)    // 단기 평균 (빨리 변함)
  for k = 1 to M do
    with probability max(0, 1 − w_fast/w_slow) do
      X_t += random pose from bel(x_0)  // 무작위 입자 주입
    else
      i ~ Categorical(w_t^[1], ..., w_t^[M])
      X_t += x_t^[i]
  endfor
  return X_t
```

요건: $0 \le \alpha_{\text{slow}} \ll \alpha_{\text{fast}}$ (예: $\alpha_{\text{slow}} = 0.001$, $\alpha_{\text{fast}} = 0.1$).

$$p_{\text{inject}} = \max\!\left(0,\, 1 - \frac{w_{\text{fast}}}{w_{\text{slow}}}\right)$$

평소에는 $w_{\text{fast}} \approx w_{\text{slow}}$라서 비율이 1에 가깝고 주입 확률은 0에 가깝다. 표준 MCL과 같다. 납치 직후에는 측정이 어디에도 안 맞아 $w_{\text{fast}}$가 먼저 뚝 떨어지고 주입 확률이 오른다. 장기 평균이 따라 내려오면 비율이 다시 1에 가까워져 주입이 멎는다.

일시적인 노이즈 스파이크도 $w_{\text{fast}}$를 떨어뜨려 주입 확률을 올린다. 비율이 의미를 가지려면 $w_{\text{slow}}$는 느리게 움직여야 하므로, 오탐을 줄이려면 $w_{\text{fast}}$의 평활 계수 $\alpha_{\text{fast}}$를 조절한다.

§14.7.1의 `recovery_alpha_slow/fast` 파라미터가 이 적응형 random-pose 주입을 제어한다. 연구 시스템 중에는 NetVLAD 같은 place recognition과 PnP로 얻은 pose 후보를 입자 후보에 섞는 것도 있다. 기본 AMCL에는 없는 동작이라 쓸 환경에서 따로 검증해야 한다.

#### 14.7.8 Mixture MCL

Augmented MCL은 무작위 포즈를 주입한다. Mixture MCL은 proposal 분포를 바꿔, 일부 입자를 측정 모델에서 직접 샘플한다.

$$x_t^{[k]} \sim \begin{cases} p(z_t | x_t, m) & \text{확률 } \rho \\ \text{sample\_motion\_model}(u_t, x_{t-1}^{[k]}) & \text{확률 } 1 - \rho \end{cases}$$

측정에서 바로 뽑은 입자는 측정과 잘 맞는 곳에 몰리므로, 센서 노이즈가 작을 때 기본 MCL에서 입자가 낭비되는 문제를 푼다. 납치에서 회복하는 문제와 노이즈 작은 센서에서 실패하는 문제를 함께 다룬다는 점이 Augmented MCL보다 낫다. 대신 $p(z_t | x_t, m)$에서 직접 샘플하려면 측정으로부터 자세를 뽑는 역방향 센서 모델을 따로 만들어야 한다.

#### 14.7.9 동적 환경 필터링

동적 물체(사람, 차량)가 있는 환경에서는 빔의 일부가 지도에 없는 장애물을 관측한다. 빔 센서 모델의 short hit 성분 $p_{\text{short}}(z | x, m)$으로 이런 빔을 골라낸다. 빔 $z_t^k$마다 네 성분 혼합 모델(§2.7.2)을 계산하고, short 성분의 사후확률이 높은 빔은 가중치 계산에서 뺀다. 이렇게 거르면 복도에 사람이 많아도 MCL이 안정적으로 돈다.

#### 14.7.10 필터 비교 정리

| 알고리즘 | Belief 표현 | Position tracking | Global loc | Kidnapped | 계산 비용 |
|---|---|---|---|---|---|
| EKF Loc | Gaussian (μ, Σ) | 좋음 | 불가 | 불가 | O(N) |
| MHT | Gaussian 혼합 | 좋음 | 제한 | 제한 | O(H·N) |
| Grid Loc | 히스토그램 | 좋음 | 가능 | 가능 | O(K) |
| MCL | 파티클 집합 | 좋음 | 가능 | Augmented MCL로 가능 | O(M) |

N은 랜드마크 수, H는 가설 수, K는 격자 셀 수, M은 파티클 수다. EKF는 Gaussian 단봉 가정이라 position tracking에 쓴다. Grid와 MCL은 셀 수나 입자 수를 바꿔 정확도와 속도 사이에서 고를 수 있다.

#### 14.7.11 실무 고려: 랜드마크 효율·Negative Information

**랜드마크 빨리 찾기**: 지도에 landmark가 N개 있을 때 관측마다 모두 뒤지면 O(N)이다. 저차원에서 균형 잡힌 KD-tree는 평균 O(log N)이지만 최악이면 O(N)이다. 격자로 색인하면 계산량은 셀에 든 랜드마크 수와 탐색 반경에 달려 있다.

**Mutual exclusion**: 한 프레임에서 랜드마크 하나에는 측정 하나만 대응한다. ML data association은 측정마다 따로 짝을 고르므로 이 제약을 따로 걸어야 한다. 두 측정이 한 랜드마크에 붙으면 마할라노비스 거리가 더 작은 측정을 남기고 나머지를 버린다.

Outlier rejection은 마할라노비스 거리가 $\chi^2_{95\%}$ 임계값을 넘는 측정을 버린다. 이 한 줄로 EKF가 엉뚱한 측정 하나에 무너지는 일이 크게 줄어든다.

**Negative information**: "이 각도 범위에서는 랜드마크가 보이지 않았다"는 사실도 위치추정에 도움이 된다. 다만 확률로 정확히 다루기가 복잡해서 실용 시스템은 대부분 이 정보를 버린다.

---

### 14.7B Occupancy Grid Mapping

§14.7은 지도가 *주어졌을 때* 위치를 추정했다. Occupancy Grid Mapping은 거꾸로 위치를 안다고 놓고 셀마다 점유 확률을 추정해 지도를 *만든다*. SLAM에서는 pose graph 최적화로 자세 궤적을 확정한 뒤, 이 절의 알고리즘으로 최종 점유 지도를 만든다. 바탕이 되는 binary Bayes 필터는 §3.11.2에 있다.

#### 14.7B.1 도입: 지도 작성의 어려움

지도를 만드는 일은 위치를 추정하는 일보다 어렵다. 평면 위 로봇의 자세 $x_t \in \mathbb{R}^3$(x, y, θ)는 연속 변수 셋이지만, 지도 m은 셀 수만에서 수백만 개로 이뤄진 고차원 이산 변수다. 가능한 지도가 $2^{|m|}$개라 하나씩 따져 볼 수 없다.

그래서 두 가지를 가정한다. 하나는 자세 $x_{1:t}$를 안다는 것이고, 다른 하나는 셀끼리 조건부 독립이라는 것이다. 두 번째 가정 덕분에 지도 사후확률을 셀별 marginal의 곱으로 나눌 수 있고, 전체 문제가 셀마다 따로 도는 binary Bayes 필터로 쪼개진다.

$$p(m \mid z_{1:t}, x_{1:t}) = \prod_i p(m_i \mid z_{1:t}, x_{1:t})$$

*Probabilistic Robotics*는 이 밖에 지도 작성을 어렵게 하는 요인으로 센서 범위에 비한 환경의 크기, 센서와 구동의 노이즈, 서로 다른 장소가 비슷하게 보이는 모호성(perceptual ambiguity), 그리고 환경 속 순환(cycle)을 든다.

#### 14.7B.2 표준 알고리즘: Log-Odds 누산

셀마다 점유 사후확률을 log-odds로 바꿔 더해 간다.

$$l_{t,i} = \log \frac{p(m_i \mid z_{1:t}, x_{1:t})}{1 - p(m_i \mid z_{1:t}, x_{1:t})}$$

사전 log-odds를 $l_0 = \log[p(m_i) / (1 - p(m_i))]$로 두면, binary Bayes 필터(§3.11.2)에서 갱신식이 나온다.

$$l_{t,i} = l_{t-1,i} + \text{inverse\_sensor\_model}(m_i, x_t, z_t) - l_0$$

새 측정이 셀 $m_i$가 차 있다고 말하면(hit) log-odds가 오르고, 비어 있다고 말하면(free) 내린다. $-l_0$ 항은 사전확률을 두 번 세지 않게 한다.

```
occupancy_grid_mapping({l_{t-1,i}}, x_t, z_t):
  for all cells m_i do
    if m_i is in perceptual field of z_t then
      l_{t,i} = l_{t-1,i} + inverse_sensor_model(m_i, x_t, z_t) − l_0
    else
      l_{t,i} = l_{t-1,i}    // 관측 범위 밖 — 변화 없음
  endfor
  return {l_{t,i}}
```

확률로 복원: $p(m_i | z_{1:t}, x_{1:t}) = 1 - 1/(1 + \exp\{l_{t,i}\})$.

**inverse_sensor_model** (range finder용 단순 예시):
```
inverse_range_sensor_model(m_i, x_t, z_t):
  셀 중심까지 거리 r, 방위각 φ 계산
  가장 가까운 빔 인덱스 k = argmin_j |φ − θ_{j,sens}|
  if |φ − θ_{k,sens}| > β/2 이거나 r > min(z_max, z_t^k + α/2):
    return l_0                   // 정보 없음
  if z_t^k < z_max 이고 |r − z_t^k| < α/2:
    return l_occ                 // hit (> l_0)
  if r ≤ z_t^k:
    return l_free                // free (< l_0)
```

$\alpha$는 장애물 두께, $\beta$는 빔이 퍼지는 각도다. Cartographer의 submap probability grid는 이 누산을 odds 곱으로 바꿔 lookup table로 계산한다. ROS Nav2의 `costmap_2d`는 0~254 cost 값에 raytracing 기반 marking/clearing을 쓰고, SLAM Toolbox(Karto 계열)는 셀별 hit/visit 카운트 비율에 임계를 적용하므로 표현과 갱신 방식이 다르다.

#### 14.7B.3 다중 센서 융합

카메라, LiDAR, 소나, 적외선 센서는 inverse_sensor_model이 저마다 다르다. 가장 단순하게 합치는 방법은 셀마다 최댓값을 쓰는 것(conservative max)이다. 센서 하나라도 hit이라고 하면 그 셀을 차 있다고 본다. 충돌을 피하기에는 안전하지만 빈 공간을 실제보다 좁게 잡는다.

센서마다 log-odds 갱신값을 따로 쌓은 뒤 셀별로 더하는 방법도 있다. 센서마다 믿을 만한 정도가 다르면 가중치를 두고 더한다.

#### 14.7B.4 inverse_sensor_model 학습

사람이 손으로 만든 inverse_sensor_model은 단순한 기하 모델이다. forward 모델 $p(z | x, m)$이 이미 있으면 역방향 모델을 학습으로 얻을 수 있다.

먼저 (자세, 측정, 점유) 묶음 $\{(x^{(k)}, z^{(k)}, m_i^{(k)})\}$를 시뮬레이션으로 만들고, cross-entropy 손실로 함수 근사기를 학습한다.

$$\mathcal{L} = -\sum_k \left[m_i^{(k)} \log \hat{p}_i + (1 - m_i^{(k)}) \log(1 - \hat{p}_i)\right]$$

입력이 $(x, z)$이고 출력이 $\hat{p}_i = p(m_i | x, z)$인 신경망이 inverse_sensor_model을 대신한다. 소나의 반사 패턴이나 유리에 닿은 LiDAR처럼 사람이 식으로 적기 어려운 센서 특성이 있을 때 쓸모 있다.

#### 14.7B.5 MAP Occupancy Mapping (심화)

표준 알고리즘의 셀 독립 가정에는 문제가 있다. 같은 빔 원뿔 안에 있는 이웃 셀들은 같은 측정을 증거로 나눠 갖는데, 독립이라고 가정하면 이 관계를 버리게 된다. 소나처럼 빔 폭이 넓은 센서에서 이 문제가 두드러진다.

MAP Occupancy Mapping은 지도 사후확률이 가장 큰 지도(mode)를 바로 찾는다.

$$m^* = \arg\max_m \left[\sum_t \log p(z_t \mid x_t, m) + \log p(m)\right]$$

inverse model 대신 forward 모델 $p(z_t | x_t, m)$을 그대로 쓴다. 모든 셀이 빈 지도에서 시작해, 셀 하나씩 점유 상태를 뒤집어 log-likelihood가 커지면 받아들이는 hill-climbing이다.

```
MAP_occupancy_grid_mapping(x_{1:t}, z_{1:t}):
  m ← 모든 셀 free로 초기화
  repeat until convergence:
    for all cells m_i do
      m_i ← argmax_{k ∈ {0,1}} [k·l_0 + Σ_t log measurement_model(z_t, x_t, m | m_i=k)]
  return m
```

한계도 있다. 데이터를 한꺼번에 처리하므로 점진적으로 도는 SLAM에 맞지 않고, hill-climbing이라 local maximum에 갇힌다. 결과가 지도 하나뿐이라 셀마다 얼마나 불확실한지도 알 수 없다.

#### 14.7B.6 다른 공간 표현과의 비교: OctoMap·Voxblox·NeRF·3DGS

OctoMap은 3D 점유 확률을 octree에 저장한다. occupancy grid를 그대로 3D로 넓힌 셈이다. Voxblox와 nvblox는 점유 확률 대신 TSDF(Truncated Signed Distance Function)로 표면까지의 부호 있는 거리를 저장한다. NeRF의 density field와 3D Gaussian Splatting의 opacity는 새 시점 렌더링을 위해 학습하는 장면 표현인데, ray를 따라 값을 합성한다는 점에서 occupancy grid와 수학이 닮았다. Occupancy grid는 SLAM Toolbox와 Nav2 costmap 같은 내비게이션 구성에서 지금도 쓴다.

> **실습**: [Occupancy Grid](https://alexjunholee.github.io/robotics-practice/app.html#occupancy_grid)
> Log-odds 누산 과정을 셀별로 시각화하며, inverse_sensor_model의 hit/free 영역이 어떻게 지도로 쌓이는지 확인할 수 있다.

---

## Part 2. 학습 기반 SLAM

### 14.8 Learning-based & Neural SLAM

전통적인 SLAM은 사람이 설계한 특징점, 매칭 알고리즘, 최적화 파이프라인을 쓴다. 이 절의 방법들은 이 과정의 일부나 전체를 딥러닝으로 바꾼다.

**DROID-SLAM (2021)**:
- 이름은 Differentiable Recurrent Optimization-Inspired Design(DROID)에서 왔다
- 특징점을 뽑고 짝짓지 않고, dense optical flow를 반복해서 다듬으며 카메라 포즈와 깊이를 함께 추정
- 미분 가능한 Dense Bundle Adjustment(DBA) 레이어를 넣어 end-to-end로 학습

특징점 기반 SLAM(ORB-SLAM)은 텍스처가 부족한 환경에서, 직접법(DSO)은 조명 변화에서 성능이 떨어진다. DROID-SLAM 논문은 TartanAir, EuRoC, TUM-RGBD, ETH3D-SLAM에서 이전 방법보다 오차가 작고 추적이 완전히 실패하는(catastrophic failure) 경우가 훨씬 적었다고 보고했다. 대신 GPU가 있어야 하고, 설정에 따라 기존 CPU 방법보다 느리다.

**3DGS-SLAM 융합**:
13.5.2에서 다룬 3D Gaussian Splatting을 SLAM의 맵 표현(map representation)으로도 쓴다. SplaTAM과 MonoGS가 이렇게 한다. 포인트 맵 대신 3D Gaussian으로 환경을 나타내므로 맵이 실제 장면과 더 비슷하게 보이고, 맵을 렌더링해서 가상 시점 영상을 만들거나 AR 화면에 겹쳐 보일 수 있다.

> **추천 자료**
> - [Teed & Deng, "DROID-SLAM: Deep Visual SLAM for Monocular, Stereo, and RGB-D Cameras" (2021)](https://arxiv.org/abs/2108.10869) — DROID-SLAM 논문
> - [Keetha et al., "SplaTAM" (CVPR 2024)](https://arxiv.org/abs/2312.02126) — 3DGS 기반 Dense SLAM
> - [Awesome-SLAM GitHub](https://github.com/SilenceOverflow/Awesome-SLAM) — SLAM 논문/프로젝트 모음

---

## Part 3. 심화

### 14.9 심화: SLAM 백엔드 최적화

factor graph 이전의 정보형 SLAM(EKF-SLAM, EIF, SEIF, EM)은 §14.16에서 다룬다.

SLAM 프론트엔드가 센서 데이터로 제약 조건(constraint)을 만들면, 백엔드는 이 제약을 한꺼번에 가장 잘 만족하는 상태(포즈, 랜드마크)를 찾는다. 이것은 비선형 최소자승(nonlinear least squares) 문제다. g2o, GTSAM, Ceres 같은 라이브러리의 설정값을 이해하려면 이 수학을 알아야 한다.

**SLAM 백엔드가 푸는 문제**

SLAM 백엔드는 결국 Ax = b를 푼다.

로봇이 주행하면서 얻는 데이터는 두 종류다.
1. **Odometry**: "나는 1 m 앞으로 갔다" (상대적 이동)
2. **관측**: "저 랜드마크가 3 m 거리에 보인다"

센서 노이즈 때문에 이 측정값을 모두 정확히 만족하는 포즈와 랜드마크 위치는 없다. 그래서 모든 측정값과의 오차 제곱합이 가장 작은 해를 찾는다.

비선형이라 한 번에 못 풀고, 현재 추정값 근처에서 선형화(linearize)해서 반복적으로 갱신한다. 이 "선형화 → Ax=b 풀기 → 업데이트 → 반복"이 Gauss-Newton이다.

(참고: [김기섭 블로그 — SLAM back-end 시리즈](https://gisbi-kim.github.io/blog/2021/03/04/slambackend-1.html))

#### 14.9.1 Manifold 위의 Gauss-Newton

SLAM의 상태 변수인 포즈는 Lie group SE(3)의 원소다. 회전 행렬에 벡터를 더하면 회전 행렬의 조건(직교, det = 1)이 깨지므로, Gauss-Newton update `x ← x + δx` 대신 군 위에서 update한다. 작은 변화량(perturbation)은 Lie algebra se(3)에서 정의한다.

**Update step (left perturbation)**:
```
T ← exp(δξ^) · T
```
여기서 `δξ ∈ R^6`는 se(3) 리 대수 위의 미소 섭동(perturbation) 벡터다. `exp(·)`는 지수 사상(exponential map)을, `^`(hat 연산자)는 6차원 벡터를 4x4 행렬로 사상하는 연산자다.

**Jacobian 계산**: 오차 함수 `e(T)`의 Jacobian을 `δξ`에 대해 계산한다.
```
J = ∂e / ∂δξ
```
chain rule로 풀면 `∂e/∂(Tp) · ∂(exp(δξ^)Tp)/∂δξ`다. 뒤 항은 좌섭동을 점에 작용시킨 결과의 미분이고, δξ를 병진·회전 순으로 두면 3차원 점에 대해 `[I, -(Tp)^]` 꼴이다. 이름이 비슷한 SE(3)의 left Jacobian은 BCH 공식에서 Lie 대수 섭동을 잇는 6×6 행렬로, 이 항과는 다른 것이다.

**Normal equation**:
```
(J^T Σ^{-1} J) δξ* = -J^T Σ^{-1} e
```
- `Σ`는 측정 노이즈 공분산
- `H = J^T Σ^{-1} J`는 Hessian의 Gauss-Newton 근사이자 information matrix다
- 제약 조건이 여럿이면 각각의 `J^T Σ^{-1} J`를 더한다

수렴할 때까지 반복하면서, iteration마다 현재 추정치에서 Jacobian을 다시 계산하고 update한다.

#### 14.9.2 Schur Complement (Marginalization)

Bundle Adjustment(BA)의 상태 변수는 카메라 포즈(p)와 랜드마크(l) 두 종류다. Normal equation의 Hessian `H`는 다음처럼 블록으로 나뉜다.

```
[H_pp  H_pl] [δp]   [b_p]
[H_lp  H_ll] [δl] = [b_l]
```

포즈 수를 `m`, 랜드마크 수를 `n`이라 하면, 보통 `n >> m`이다. 이 큰 시스템을 직접 풀면 비싸다.

Schur complement로 랜드마크를 marginalize한다:

```
(H_pp - H_pl · H_ll^{-1} · H_lp) δp = b_p - H_pl · H_ll^{-1} · b_l
```

이렇게 할 수 있는 까닭은 `H_ll`이 block diagonal이기 때문이다. 랜드마크끼리는 공통 factor가 없어서, `H_ll`의 역행렬은 블록마다 따로 구하면 된다. 비용은 `O(n)`이다.

Schur complement로 줄인 카메라 시스템의 크기는 포즈 수 `m`이 정한다. 랜드마크를 소거하고 다시 대입하는 비용은 여전히 관측 수와 랜드마크 수 `n`에 따라 늘어난다. 이 블록 희소 구조 덕분에 큰 BA 문제도 풀 수 있지만, 실제 속도는 그래프 구조, solver, 하드웨어에 따라 다르다.

`δp`를 구한 뒤, `δl`은 back-substitution으로 복원한다:
```
δl = H_ll^{-1} (b_l - H_lp · δp)
```

#### 14.9.3 희소성과 Variable Ordering

Pose graph optimization의 `H`는 factor 하나가 pose 몇 개만 잇기 때문에 대개 sparse하다. Odometry factor는 이웃한 pose를, loop factor는 멀리 떨어진 pose를 잇는다. 한 노드에 붙는 factor 수는 데이터셋과 loop closure 수에 따라 다르다.

sparse 선형 시스템은 Cholesky 분해(`H = L L^T`)로 푸는데, 이때 원래 0이던 자리가 0이 아닌 값으로 채워진다. 이를 fill-in이라 한다. Fill-in이 많으면 메모리와 계산량이 크게 늘어난다. loop가 많거나 변수 순서를 잘못 정하면 fill-in이 늘어난다.

Fill-in을 줄이려면 변수 순서(variable ordering)를 잘 정해야 한다.
- **COLAMD** (Column Approximate Minimum Degree): sparse least-squares에서 흔히 쓰는 heuristic. fill-in이 작을 것 같은 열 순서를 근사로 찾는다
- **AMD** (Approximate Minimum Degree): COLAMD와 비슷하지만 대칭 행렬용이다
- **Nested dissection**: 그래프를 재귀적으로 쪼개 순서를 정한다. 큰 문제에서 효과가 크다

g2o, GTSAM, Ceres에서 solver를 설정할 때는 linear solver 종류(DENSE_SCHUR, SPARSE_NORMAL_CHOLESKY 등)와 ordering 방식을 함께 봐야 한다. ordering이 fill-in과 실행 시간을 얼마나 바꾸는지는 그래프 구조에 따라 다르므로, 대표 데이터셋에서 메모리와 지연 시간을 재 보고 고른다.

```cpp
// Ceres Solver(C++)에서 linear solver와 ordering 설정 예시
ceres::Solver::Options options;
options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
// fill-reducing ordering 알고리즘(linear_solver_ordering_type)은 기본값이 AMD다.
// METIS를 넣어 빌드했다면 ceres::NESDIS(nested dissection)로 바꿀 수 있다
```

#### 14.9.4 최적화 라이브러리 비교

| 라이브러리 | 특징 | 주요 사용처 |
|---|---|---|
| g2o | Pose graph / BA 전용, 가벼움, C++ only | ORB-SLAM2/3, LSD-SLAM |
| GTSAM | Factor graph 기반, Bayes tree(iSAM2) 지원, incremental 최적화에 강함 | LIO-SAM, 연구용 |
| Ceres Solver | 범용 nonlinear least squares, auto-diff 지원, Google 개발 | Cartographer, VINS-Fusion, 다양한 프로젝트 |

선택 기준:
- SLAM 전용이고 가볍게 쓰고 싶다 → g2o
- Factor graph 모델링이 필요하고, incremental update(키프레임이 추가될 때마다 점진적으로 최적화)가 중요하다 → GTSAM (iSAM2)
- SLAM 이외의 범용 최적화도 해야 하고, Jacobian을 직접 유도하기 싫다 → Ceres (auto-diff)

> **추천 자료**
> - Barfoot, "State Estimation for Robotics" Ch.7 (Matrix Lie Groups, §7.1.9 Calculus and Optimization)와 Ch.8 — Manifold 위의 최적화를 체계적으로 설명
> - [Dellaert & Kaess, "Factor Graphs for Robot Perception" (Foundations and Trends in Robotics, 2017)](https://www.cs.cmu.edu/~kaess/pub/Dellaert17fnt.pdf) — Factor graph와 SLAM 백엔드 이론
> - [g2o Tutorial](https://github.com/RainerKuemmerle/g2o) / [GTSAM Tutorial](https://gtsam.org/tutorials/intro.html) — 라이브러리별 실습
> - [김기섭 블로그 — Gauss-Newton Opt == IEKF update?](https://gisbi-kim.github.io/blog/2022/03/05/gn-iekf-same.html) — GN 최적화와 반복 칼만 필터의 수학적 동치성 설명

> **실습**: [Bundle Adjustment 시각화](https://alexjunholee.github.io/robotics-practice/app.html#bundle_adjustment)
> 카메라 포즈와 3D 포인트를 동시에 최적화하는 Bundle Adjustment 과정을 인터랙티브하게 확인할 수 있다.

### 14.10 심화: IMU Preintegration

IEKF와 MSCKF가 바탕으로 삼는 EKF는 §3.10.2에 있다. 14.3.3의 VINS-Mono에서 짧게 언급한 IMU preintegration의 수학을 여기서 본다.

**문제**: IMU는 보통 200~1000 Hz로 가속도와 각속도를 낸다. 비전 SLAM 백엔드는 키프레임 단위로 수 Hz에서 수십 Hz에 한 번 최적화한다. 그래서 키프레임 사이에 IMU 측정이 수십에서 수백 개 들어온다. IMU 측정마다 상태(포즈, 속도, bias)를 새로 두면 변수가 감당할 수 없이 늘어난다.

**Preintegration**: 두 키프레임 `i`와 `j` 사이의 IMU 측정을 상대 운동 측정(relative motion measurement) 하나로 압축하고, 이것을 factor로 최적화에 넣는다.

**Preintegrated measurements**: 키프레임 `i`에서 `j`까지의 상대 변화량 세 가지를 계산한다.

```
ΔR_ij = Π_{k=i}^{j-1} Exp((ω_k - b_g) · Δt)          # 상대 회전
Δv_ij = Σ_{k=i}^{j-1} ΔR_ik · (a_k - b_a) · Δt        # 상대 속도
Δp_ij = Σ_{k=i}^{j-1} (Δv_ik · Δt + 0.5 · ΔR_ik · (a_k - b_a) · Δt^2)  # 상대 위치
```

여기서 `ω_k`, `a_k`는 IMU 측정값, `b_g`, `b_a`는 gyroscope/accelerometer bias, `Δt`는 IMU 샘플링 간격이다.

이 값들은 키프레임 `i`의 좌표계에서 계산한다. 그래서 최적화 중에 키프레임 `i`의 절대 포즈가 바뀌어도 다시 계산하지 않아도 된다.

**공분산 전파**: IMU 측정 노이즈가 preintegrated measurement에 얼마나 퍼지는지 계산한다. IMU 측정이 들어올 때마다 이산 시간 식으로 공분산을 갱신한다.

```
Σ_{k+1} = A_k · Σ_k · A_k^T + B_k · Q · B_k^T
```
- `A_k`: 상태 전이 행렬 (현재 상태에서 구한 Jacobian)
- `B_k`: 노이즈 입력 행렬
- `Q`: IMU 노이즈 공분산 (데이터시트에서 확인)

최적화에서는 이 공분산의 역행렬(`Σ^{-1}`)을 factor의 information matrix로 쓴다.

**Bias가 바뀔 때**: 최적화 중에 IMU bias 추정치가 바뀌면 원칙대로라면 preintegration을 처음부터 다시 해야 한다. 이는 비싸므로 1차 근사로 보정한다.

```
ΔR_ij ≈ ΔR_ij^0 · Exp(∂ΔR/∂b_g · δb_g)
Δv_ij ≈ Δv_ij^0 + ∂Δv/∂b_g · δb_g + ∂Δv/∂b_a · δb_a
Δp_ij ≈ Δp_ij^0 + ∂Δp/∂b_g · δb_g + ∂Δp/∂b_a · δb_a
```

`^0`은 이전 bias 추정치로 계산한 값이고 `δb`는 bias 변화량이다. 편미분은 preintegration을 하면서 함께 쌓아 둔다. bias가 크게 바뀌지 않으면 이 근사로 충분하다.

**manifold 위의 preintegration**: preintegration은 Lupton과 Sukkarieh가 먼저 내놓았고, 회전을 Euler angle로 나타냈다. Forster et al.(RSS 2015, TRO 2017)이 이를 SO(3) 위에서 다시 세웠다. Lie group 위에서 적분하면 회전이 회전 행렬의 조건을 지키고 Euler 각의 특이점도 피한다. 결과는 relative motion measurement로 factor graph에 바로 들어간다. 이산 시간 적분 오차와 측정 노이즈는 그대로 남는다.

**Tightly-coupled와 Loosely-coupled**: LIO-SAM을 예로 들면 다음과 같다.
- **Loosely-coupled**: 두 갈래가 있다. IMU를 포인트 클라우드 de-skew와 scan matching 초기값으로만 쓰는 방식과, LiDAR odometry와 IMU가 각자 상태를 추정한 뒤 covariance로 합치는 방식이다. LeGO-LOAM은 앞쪽에 해당한다.
- **Tightly-coupled**: IMU preintegration factor를 LiDAR odometry factor와 같은 factor graph 안에서 동시에 최적화한다. IMU는 키프레임 사이 상대 포즈의 독립 관측으로 들어간다. LIO-SAM이 이 방식이다.

Tightly-coupled 방식은 빠르게 돌거나 갑자기 가감속할 때 차이가 난다. LiDAR scan matching만으로는 놓치는 빠른 움직임을 IMU factor가 잡기 때문이다. factor graph라서 GPS factor나 loop closure factor를 따로 붙이기 쉽다는 점도 실제로 쓸 때 좋다.

**LIO-SAM의 구조**: GTSAM으로 IMU preintegration factor, LiDAR odometry factor, GPS factor, loop closure factor를 한 그래프에서 최적화한다. LiDAR odometry는 edge feature와 planar feature를 따로 뽑아 해상도가 다른 voxel map에 저장한다. scan matching에서는 planar feature의 point-to-plane 거리와 edge feature의 point-to-line 거리를 최소로 하는 상대 변환을 구한다.

VINS-Mono, ORB-SLAM3(Visual-Inertial 모드), LIO-SAM 같은 VIO/LIO 시스템이 IMU factor를 이 기법으로 구현한다.

> **추천 자료**
> - [Forster et al., "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry" (TRO 2017, arXiv:1512.02363)](https://arxiv.org/abs/1512.02363) — on-manifold preintegration의 원본 논문(사전 적분 개념 자체는 Lupton & Sukkarieh 2012). 수식이 많지만 이 분야의 필수 논문
> - [Forster et al., "IMU Preintegration on Manifold for Efficient Visual-Inertial Maximum-a-Posteriori Estimation" (RSS 2015)](https://rpg.ifi.uzh.ch/docs/RSS15_Forster.pdf) — 위 논문의 초기 버전으로, 방법을 더 간결하게 설명한다.
> - [Shan et al., "LIO-SAM" (IROS 2020)](https://github.com/TixiaoShan/LIO-SAM) — Tightly-coupled LIO의 레퍼런스 구현. 코드와 논문 모두 읽을 것
> - [Sola et al., "A micro Lie theory for state estimation in robotics" (arXiv:1812.01537)](https://arxiv.org/abs/1812.01537) — Lie group/algebra의 실용적 정리. Preintegration 읽기 전에 이것부터 보면 좋다
> - GTSAM의 `PreintegratedImuMeasurements` 클래스 소스코드 — 이론이 코드로 어떻게 구현되는지 확인
> - [IMU Preintegration MATLAB 구현](https://github.com/GentleDell/imu_preintegration_matlab) — KITTI에서 테스트한 MATLAB 코드. 수식과 코드를 대조하며 공부하기 좋다

### 14.11 심화: 관측가능성 분석 (Observability)

SLAM이나 VIO를 돌리다 보면 어떤 상황에서 drift가 유난히 심하거나, 가만히 서 있는데 위치가 흔들린다. 이런 현상은 대개 관측가능성(observability)의 한계에서 온다.

**Visual-Inertial 시스템에서 관측할 수 없는 상태**: Hesch et al.(2014)은 임의로 움직이며 점 특징 하나를 보는 이상적인 VINS 모델을 선형화해 분석했다. 관측할 수 없는 방향은 모두 4자유도였다.

1. **Global position (3 DoF)** — 보통 시작점을 원점으로 잡는다. 절대 위치는 GPS 같은 외부 기준으로 정한다
2. **Global yaw (1 DoF)** — 중력 방향 축 기준의 회전(heading). 북쪽 방향은 나침반 같은 외부 기준으로 정한다

이 결과는 위의 모델과 운동 조건에서만 성립한다. Roll/pitch, metric scale, IMU bias, calibration 파라미터를 추정할 수 있는지는 센서 구성과 운동이 얼마나 다양한지(excitation)에 따라 달라진다. Stereo 카메라는 baseline 길이로 metric 기준을 얻는다. Monocular 영상만으로는 scale이 정해지지 않으므로, Monocular VIO는 충분히 움직일 때 IMU 측정을 함께 써서 scale을 추정한다.

**Degenerate motion**: 어떤 움직임에서는 관측할 수 없는 상태가 더 생긴다.

- **순수 회전(pure rotation)**: 병진 parallax가 0이라 monocular 영상에서 translation과 새 점의 깊이가 미정으로 남는다. 회전은 이 경우에도 추정된다.
- **초기화 구간의 부족한 운동**: 단안 VIO 초기화에서 등속 운동이나 정지 상태만 관측하면 IMU bias의 해가 여러 개 남기도 한다. 이때 scale·중력·bias 초기화가 약해진다. 정확한 degeneracy는 상태와 측정 모델에 따라 다르다.

**EKF 기반 시스템의 문제**: 표준 EKF로 VIO를 풀면 선형화 오차 때문에 이론상 관측할 수 없는 방향에서도 공분산이 줄어든다. 실제보다 확신이 커지는 셈이고, 이것이 추정이 일관성을 잃는(inconsistency) 주된 원인이다.

**OC-EKF (Observability-Constrained EKF)**: EKF의 Jacobian을 고쳐 관측할 수 없는 방향(null space)을 지킨다. 그러면 그 방향의 불확실성이 추정기 안에서도 줄지 않는다.

실제로 쓸 때는 다음을 챙긴다.
- VIO를 초기화할 때 쓰는 구현이 요구하는 만큼 회전하고 이동한 뒤, scale·중력·bias가 수렴했는지 확인한다
- Loop closure는 다시 찾은 장소로 누적 drift를 줄인다. 절대 위치나 yaw 기준은 loop closure로 생기지 않는다
- Monocular VIO의 metric scale은 영상만으로는 정해지지 않으므로, 초기화 구간의 IMU 측정과 운동이 충분한 정보를 주는지 확인한다

> **추천 자료**
> - [Hesch et al., "Consistency Analysis and Improvement of Vision-aided Inertial Navigation" (TRO 2014)](https://ieeexplore.ieee.org/document/6605544) — OC-EKF/OC-VINS의 원본 논문
> - [Hong & Lim, "Visual-Inertial Odometry with Robust Initialization and Online Scale Estimation" (Sensors 2018)](https://pmc.ncbi.nlm.nih.gov/articles/PMC6308559/) — 등속·정지 구간에서 초기화가 약해지는 조건을 다루는 구현 예
> - Barfoot, "State Estimation for Robotics" Ch.3 (§3.1.4, §3.3.7 Existence, Uniqueness, and Observability) — 관측가능성 분석의 이론적 토대
> - [Huang & Dissanayake, "A critique of current developments in Simultaneous Localization and Mapping" (IJARS 2016)](https://journals.sagepub.com/doi/10.1177/1729881416669482) — SLAM의 관측가능성/일관성 문제를 비판적으로 정리

> **실습**: [Odometry Uncertainty 시각화](https://alexjunholee.github.io/robotics-practice/app.html#odom_uncertainty)
> Odometry의 불확실성이 시간에 따라 어떻게 누적되는지, 공분산 타원이 어떻게 커지는지 인터랙티브하게 확인할 수 있다.

#### 14.11.1 필터 기반 vs 최적화 기반: 뭐가 더 나은가?

필터와 최적화 중 어느 쪽이 나은지는 SLAM/VIO 분야에서 오래 다퉈 온 문제다. 수학으로 보면 Gauss-Newton 최적화와 Iterated EKF(IEKF)는 같은 문제를 다른 형태로 푼다.

- **필터 (EKF, MSCKF 등)**: 새 측정이 들어올 때마다 상태와 covariance를 갱신한다. 단순한 EKF odometry는 현재 상태만 들고 가고, MSCKF는 과거 카메라 포즈 몇 개를 복사해 상태에 남겨 둔다. 남기는 범위가 정해져 있어 과거 전체를 최적화하는 방식보다 메모리를 덜 쓴다.
- **최적화 (BA, factor graph)**: 과거 상태를 모두 남겨 두고 한꺼번에 최적화한다. 과거 데이터를 다시 선형화(relinearize)할 수 있어 정확하다. 상태가 늘수록 계산량이 커지는데, sliding window나 iSAM2로 이를 줄인다.

VINS-Mono(최적화)와 MSCKF(필터)의 성능 차이는 주로 시스템 구조에서 온다. 어떤 상태를 남기고 어떤 측정을 쓰는지가 다르다. 최적화 기반 시스템은 다시 선형화해서 과거의 선형화 오차를 줄일 수 있다.

실제로는 이렇게 고른다.
- IMU 중심 + 경량 → 필터 (MSCKF, FAST-LIO2의 IEKF)
- 카메라 중심 + 정확도 → 최적화 (VINS-Mono, ORB-SLAM3)
- 둘 다 필요 → 하이브리드 (LIO-SAM: IMU preintegration을 factor로 넣은 최적화)

(참고: [김기섭 블로그 — Gauss-Newton Opt == IEKF update?](https://gisbi-kim.github.io/blog/2022/03/05/gn-iekf-same.html))

### 14.12 심화: Semantic SLAM

전통적인 SLAM은 공간의 기하 형상만 복원한다. 포인트 클라우드, 메시, 점유 격자 모두 공간이 어떻게 생겼는지만 기록한다. 이 지도에는 장애물이 어디 있는지만 있고, 그것이 벽인지 문인지 책장인지는 없다. Semantic SLAM은 지도에 이런 의미(semantic) 정보를 붙인다.

방법은 랜드마크를 무엇으로 나타내는지에 따라 나뉜다. **Object-level SLAM**(CubeSLAM, QuadricSLAM)은 점 대신 3D cuboid나 dual quadric 같은 물체를 랜드마크로 추정한다. 물체 검출기가 있어야 하지만, 점보다 짝짓기(data association)가 안정적이고 물체 단위로 추론할 수 있다. Panoptic SLAM은 panoptic segmentation 결과를 3D로 모아 모든 점에 semantic label이 붙은 지도를 만든다. 그러면 로봇이 "이 방에 의자가 몇 개인가"를 지도에서 바로 찾아본다. **Open-vocabulary SLAM**(ConceptGraphs)은 CLIP 같은 vision-language model의 feature를 지도에 저장해 자연어로 장소를 찾게 한다. 13장의 3D Scene Graph(Hydra 등)와 바로 이어지는 주제다.

**움직이는 물체 처리**: Semantic label은 움직이는 물체가 많은 환경에서 SLAM을 안정시키는 데도 쓴다. 사람이나 차처럼 움직일 가능성이 큰 클래스의 feature를 tracking과 mapping에서 빼면, 정지한 배경만으로 SLAM을 돌릴 수 있다.
- DynaSLAM: ORB-SLAM2 + Mask R-CNN으로 동적 물체 마스킹
- DS-SLAM: semantic segmentation으로 동적 영역 필터링

```python
# 동적 물체 필터링의 의사코드
dynamic_labels = {'person', 'car', 'bicycle', 'dog'}
for feature in detected_features:
    pixel = feature.pixel_coords
    label = semantic_map[pixel.y, pixel.x]
    if label in dynamic_labels:
        feature.ignore = True  # SLAM에서 제외
```

> **추천 자료**
> - [Nicholson et al., "QuadricSLAM: Dual Quadrics from Object Detections as Landmarks in Object-Oriented SLAM" (RA-L 2019)](https://arxiv.org/abs/1804.04011) — Object-level SLAM의 대표 논문
> - [ConceptGraphs (arXiv:2309.16650)](https://arxiv.org/abs/2309.16650) — Open-vocabulary 3D scene graph. 13장과 연계해서 읽을 것
> - [Bescos et al., "DynaSLAM: Tracking, Mapping and Inpainting in Dynamic Scenes" (RA-L 2018)](https://arxiv.org/abs/1806.05620) — 동적 환경 SLAM

### 14.13 심화: Multi-Robot SLAM

로봇 한 대로 넓은 환경을 탐색하면 오래 걸린다. 여러 대가 나눠 탐색하면 시간은 줄지만, 로봇마다 만든 부분 지도(submap)를 하나의 전체 지도로 합쳐야 한다.

Centralized 방식은 로봇들의 센서 데이터나 local map을 중앙 서버로 보내 한꺼번에 최적화한다. 서버가 전체 정보를 보기 쉬운 대신 통신량과 서버 계산량이 크고, 서버가 멈추면 시스템 전체가 멈춘다. SLAM은 비볼록 문제라서 중앙에서 푼다고 전역 최적해가 보장되지도 않는다.

Distributed 방식에서는 로봇마다 local SLAM을 돌리고, 로봇끼리 만나거나(rendezvous) 같은 장소를 알아봤을 때(inter-robot loop closure) 상대 포즈 제약을 만든다. 원본 데이터 대신 descriptor나 submap만 주고받으면 통신량이 줄지만, 대응을 검증할 정보도 함께 줄어든다. 로봇이 어떤 변수와 factor를 들고 주고받는지, 비동기 최적화가 어떤 조건에서 수렴하는지는 알고리즘마다 다르다.

분산 시스템은 inter-robot loop closure, 좌표계 정렬, outlier 제거를 함께 풀어야 한다. 로봇 사이의 SE(3) 변환은 검증된 6자유도 포즈 제약으로 바로 받기도 하고, 3차원 점 대응쌍으로 계산하기도 한다. 점 대응으로 계산하려면 한 직선 위에 있지 않은 점이 적어도 셋 있어야 하고 인라이어도 충분해야 한다. PCM(Pairwise Consistency Maximization), GNC(Graduated Non-Convexity), distributed Gauss-Seidel, ADMM은 저마다 가정과 통신 비용이 다르다.

**대표 시스템**:
| 시스템 | 특징 |
|---|---|
| Kimera-Multi | Distributed, 3D mesh + semantic, Kimera 기반 |
| DOOR-SLAM | Distributed, outlier-robust, DGS 최적화 |
| Swarm-SLAM | ROS2 기반, 다양한 센서 지원, 경량 |

> **추천 자료**
> - [Lajoie et al., "DOOR-SLAM: Distributed, Online, and Outlier Resilient SLAM for Robotic Teams" (RA-L 2020)](https://arxiv.org/abs/1909.12198) — Distributed SLAM + robust optimization
> - [Tian et al., "Kimera-Multi: Robust, Distributed, Dense Metric-Semantic SLAM for Multi-Robot Systems" (T-RO 2022)](https://arxiv.org/abs/2106.14386) — Multi-robot semantic SLAM
> - [Cieslewski et al., "Data-Efficient Decentralized Visual SLAM" (ICRA 2018)](https://arxiv.org/abs/1710.05772) — 통신 효율적인 분산 SLAM의 초기 연구

### 14.14 심화: Place Recognition

Loop closure는 "지금 보는 장면을 전에 본 적 있는가?"를 묻는다. 이것은 이미지 검색(image retrieval) 문제다. 현재 프레임의 descriptor를 과거 키프레임 전체의 descriptor와 비교해 가장 비슷한 것을 찾는다. place recognition이 틀리면 loop closure가 틀리고, 그러면 SLAM 지도 전체가 틀어진다.

**전통적 방법: Bag of Visual Words (BoVW)**

ORB-SLAM2/3가 쓰는 DBoW2 라이브러리가 이 방식이다.
1. 대규모 이미지에서 local feature(ORB 등)를 추출
2. k-means clustering으로 visual vocabulary(단어 사전) 구축
3. 각 이미지를 "어떤 visual word가 몇 번 나타났는지"의 histogram(BoW vector)으로 표현
4. BoW vector 간의 유사도(L1-score 등)로 이미지 비교

inverted index를 써서 빠르고 오래 써 온 방법이다. 대신 시점과 조명이 바뀌면 약하고, vocabulary를 미리 학습해 둬야 한다.

**학습 기반 방법: Global Descriptor**

이미지 전체를 짧은 벡터 하나로 압축한다. **NetVLAD**(2016)는 CNN 특징 위에 VLAD 집계를 미분 가능한 층으로 붙여 end-to-end로 학습했고, 도시 규모 벤치마크에서 비교한 기존 방법보다 재현율(recall)이 높았다. **CosPlace**(2022)와 **MixVPR**(2023)은 descriptor 학습과 집계 방식을 고쳐 각 논문의 데이터셋과 평가 방식에서 성능을 쟀다. **AnyLoc**(2023)은 DINOv2 feature를 써서, place recognition용 fine-tuning 없이 실내·실외·항공 데이터셋에서 낸 결과를 보고했다. 이 결과가 어느 환경에서나 BoVW보다 낫다는 뜻은 아니므로, 쓸 환경에서 recall과 false positive를 다시 재야 한다.

**LiDAR 기반 Place Recognition**:

3D 구조만 보고 장소를 알아보므로 조명이 바뀌어도 괜찮다. 다만 날씨나 움직이는 물체 때문에 레이저가 되돌아오는 점이 달라지고, 긴 복도처럼 구조가 비슷한 곳은 헷갈린다.

- **Scan Context** (IROS 2018): 3D 포인트 클라우드를 위에서 내려다본 평면에 투영하고, 거리와 방위로 나눈 칸마다 가장 높은 점의 높이를 적어 2D descriptor를 만든다. 로봇이 제자리에서 돌아도 매칭할 수 있다
- **OverlapTransformer** (2022): Transformer 기반으로 LiDAR range image에서 global descriptor를 학습

**Cross-modal Place Recognition**: 카메라 이미지로 LiDAR 지도를 검색하거나, 거꾸로 LiDAR scan으로 이미지 지도를 검색한다. 센서가 서로 다른 로봇들로 multi-robot SLAM을 할 때 필요하다.

**Sequence Matching**: 이미지 한 장으로는 판단하기 어려울 때 연속된 프레임 여러 장을 묶어 비교한다.
- **SeqSLAM** (2012): 이미지 한 장끼리는 덜 비슷해도 연속된 이미지들의 유사도 패턴이 맞으면 같은 장소로 본다. 낮과 밤처럼 겉모습이 크게 바뀌어도 동작한다
- **SeqNet** (RA-L 2021): 이미지 시퀀스 전체를 sequence descriptor 하나로 학습해, SeqSLAM식 비교 전에 후보를 고른다

조명이나 계절이 크게 바뀌는 환경이라면 공개 기준선인 DBoW2와 함께 NetVLAD 이후의 학습 기반 descriptor도 같은 데이터로 비교해 본다. AnyLoc은 fine-tuning이 필요 없지만, backbone 계산량과 descriptor 메모리, 쓸 환경의 recall을 확인해야 한다.

> **추천 자료**
> - [Arandjelovic et al., "NetVLAD: CNN architecture for weakly supervised place recognition" (arXiv:1511.07247)](https://arxiv.org/abs/1511.07247) — CNN 기반 place recognition을 end-to-end로 학습한 논문
> - [Keetha et al., "AnyLoc: Towards Universal Visual Place Recognition" (arXiv:2308.00688)](https://arxiv.org/abs/2308.00688) — Foundation model 기반 zero-shot place recognition
> - [Kim & Kim, "Scan Context: Egocentric Spatial Descriptor for Place Recognition within 3D Point Cloud Map" (IROS 2018)](https://ieeexplore.ieee.org/document/8593953) — LiDAR place recognition의 대표 방법
> - [김기섭 블로그 — Scan Context-based LiDAR Pose-graph SLAM 구현](https://gisbi-kim.github.io/blog/2021/05/17/sclidarslam.html) — Scan Context를 LiDAR SLAM에 통합한 구현 해설
> - [다크 프로그래머 — Bag of Words 기법](https://darkpgmr.tistory.com/125) — BoW의 원리를 이미지 검색과 연결하여 설명

> **기술 흐름: SLAM & Odometry**
> - **~2007**: EKF-SLAM과 입자 필터 기반 FastSLAM이 주로 쓰임. MonoSLAM(2007)이 실시간 단안 SLAM을 보였고, PTAM(2007)이 Tracking과 Mapping을 나눈 구조를 내놓음
> - **2010~2015**: LSD-SLAM, SVO 등 direct method 등장. LOAM(2014)이 LiDAR odometry와 mapping의 영향력 있는 구조를 제시했고, ORB-SLAM(2015)이 공개 feature-based visual SLAM baseline이 됨
> - **2015~2020**: VINS-Mono(2018), MSCKF 계열 등 VIO system의 공개 구현. DSO가 direct sparse 방식을 제시했고, LIO-SAM 등 LiDAR-inertial system 등장
> - **2020~2023**: FAST-LIO/FAST-LIO2, ORB-SLAM3, DROID-SLAM, R3LIVE 등 filter·optimization·learning을 서로 다르게 조합한 공개 system 등장
> - **2024~**: 3DGS 기반 SLAM 발표(SplaTAM·MonoGS는 CVPR 2024, Gaussian-SLAM은 arXiv 2023 프리프린트)

### 14.15 심화: Long-term Mapping

실제 환경에서 로봇을 오래 쓰려면 한 번 만든 지도로 끝낼 수 없다. 같은 장소를 여러 번 다니며 지도를 고치고, 사람이나 차처럼 움직이는 물체를 지도에서 지우고, 여러 번 나눠 모은 데이터를 합쳐야 한다. 이 일을 long-term mapping이라 한다.

#### 14.15.1 Incremental Smoothing: iSAM에서 iSAM2까지

EKF 같은 필터 기반 SLAM은 상태가 늘어날수록 행렬이 커져 실시간으로 돌리기 어렵다. iSAM(Kaess et al., TRO 2008)은 QR 분해의 R 행렬을 Givens rotation으로 조금씩 갱신할 수 있음을 보였다. 새 측정이 들어오면 바뀐 부분만 고치면 된다. 다만 0이 아닌 원소가 쌓이면 주기적으로 변수 순서를 다시 정해야 한다.

iSAM2(Kaess et al., IJRR 2012)는 Bayes tree라는 자료구조로 이 문제를 풀었다. 새 측정의 영향을 받는 부분 트리만 다시 소거하므로 문제가 커져도 속도가 고르게 유지된다. GTSAM의 증분 최적화가 iSAM2다.

#### 14.15.2 Dynamic Object Removal

Long-term mapping을 하려면 지도에서 움직이는 물체를 지워야 한다.

**Removert** (Kim et al., 2020): 포인트 클라우드를 range image로 투영하고, 다른 시점에서 잰 거리와 비교해 움직인 점인지 판단한다. 먼저 정지한 점만 보수적으로 남기고, 그다음 잘못 지운 점을 되살리는 두 단계로 돈다. range image의 해상도를 여러 단계로 바꿔 가며 두 단계의 균형을 조절한다.

다른 방법들과 견주면, 복셀 레이캐스팅은 정밀하지만 계산이 무겁다. 가시성(visibility) 기반 방식은 뒤쪽에 가려진 정지한 점을 지키려면 따로 가정을 둬야 한다. semantic segmentation 기반 방식은 학습한 클래스에 없는 물체를 놓친다.

**SuMa++** (Chen et al., IROS 2019): 작은 원판(surfel)으로 지도를 만드는 SuMa에 semantic label을 더했다. semantic label로도, 실제 움직임으로도 움직인다고 판정된 surfel만 지운다. 주차된 차처럼 움직일 수 있는 클래스라도 실제로 멈춰 있으면 기하 제약으로 쓸 수 있기 때문이다.

#### 14.15.3 Multi-Session SLAM

같은 환경을 여러 날에 걸쳐 매핑하면 세션마다 만든 궤적을 하나로 합쳐야 한다. 세션마다 좌표계의 원점과 방향이 제각각이라(gauge freedom) 그냥 겹치면 맞지 않는다.

**LT-mapper** (Kim et al., ICRA 2022): Scan Context로 세션 사이의 loop를 찾고, 세션마다 좌표계를 나타내는 anchor node를 두어 여러 세션을 맞춘다. 변화는 빠른 변화(high dynamic)와 느린 변화(low dynamic)로 나누고, 느린 변화를 다시 새로 생긴 점(positive difference)과 사라진 점(negative difference)으로 갈라 변화분 지도(delta map)로 관리한다.

**Continuous-Time Estimation** (Furgale et al., ICRA 2012): 궤적을 시각마다 따로 두지 않고 B-spline 기저 함수로 나타내면, 측정 주기가 다른 센서들을 더 적은 변수로 묶을 수 있다. IMU처럼 빠른 센서와 LiDAR·카메라처럼 느린 센서 사이의 시간 오프셋과 외부 파라미터를 함께 추정할 때도 쓴다.

> **추천 자료**
> - [Kaess et al., "iSAM2: Incremental Smoothing and Mapping Using the Bayes Tree" (IJRR 2012)](https://www.cs.cmu.edu/~kaess/pub/Kaess12ijrr.pdf) — Bayes tree 기반 incremental SLAM의 원본 논문
> - [Kim et al., "Remove, then Revert: Static Point Cloud Map Construction using Multiresolution Range Images" (IROS 2020)](https://github.com/gisbi-kim/removert) — Dynamic point 제거의 실용적 방법. 코드 공개
> - [Kim et al., "LT-mapper: A Modular Framework for LiDAR-based Lifelong Mapping" (ICRA 2022)](https://github.com/gisbi-kim/lt-mapper) — Multi-session SLAM 프레임워크
> - [Chen et al., "SuMa++: Efficient LiDAR-based Semantic SLAM" (IROS 2019)](https://github.com/PRBonn/semantic_suma) — Semantic 정보를 활용한 LiDAR SLAM

---

### 14.16 심화: 정보형 SLAM의 역사

2005년 *Probabilistic Robotics*가 나올 무렵에는 SLAM의 불확실성을 어떻게 나타낼지를 두고 여러 방법이 함께 쓰였다. EKF-SLAM은 로봇 포즈와 랜드마크를 한 상태 벡터에 묶고 공분산 행렬을 갱신했다. EIF와 SEIF는 정보 행렬이 덧셈으로 갱신된다는 성질을 써서 계산을 줄였다. EM mapping은 모르는 데이터 연관을 기댓값 최대화(EM)로 풀었다. 2010년대의 factor graph와 GTSAM/iSAM2는 정보형의 덧셈 갱신, 변수 소거, 증분 갱신을 받아들였다(§14.9).

#### 14.16.1 EKF-SLAM (PR §10)

EKF-SLAM은 Smith, Self, Cheeseman의 "Estimating Uncertain Spatial Relationships in Robotics"(1986/1990)에서 시작한다. 이들은 로봇 포즈와 랜드마크를 한 확률 변수로 묶은 'stochastic map'을 제안했다. 1990년대 Leonard와 Durrant-Whyte, 그리고 Dissanayake et al.(2001, IEEE T-RA)이 이를 정식화했다.

**알고리즘 구조**: 포즈 $x_t = (x, y, \theta)$와 랜드마크 N개 $(m_{j,x}, m_{j,y}, s_j)$를 $(3N+3)$차원 상태 벡터 $y_t$로 묶고 EKF를 돌린다.

```
EKF_SLAM_known_correspondences(μ_{t-1}, Σ_{t-1}, u_t, z_t, c_t):
  // Motion: F_x로 3D motion을 (3N+3)D로 lift
  // F_x = [I_3 | 0_{3×3N}] — (3×(3N+3)) 프로젝션, F_x^T 는 (3N+3)×3
  // G_t = I_{3N+3} + F_x^T G_t^{pose} F_x — (3N+3)×(3N+3), G_t^{pose}는 포즈 3×3 Jacobian에서 I_3를 뺀 행렬
  μ̄_t = μ_{t-1} + F_x^T · g(u_t, μ_{t-1}[포즈 부분])
  Σ̄_t = G_t Σ_{t-1} G_t^T + F_x^T R_t F_x

  // Measurement loop
  for each observation z_t^i with j = c_t^i do
    if j is new landmark:
      μ̄_{j} ← range-bearing 역변환으로 초기화
    ẑ_t^i = h(μ̄_t, j),   H_t^i = Jacobian  // H_t^i: 3×(3N+3)
    K_t^i = Σ̄_t H_t^{i,T} (H_t^i Σ̄_t H_t^{i,T} + Q_t)^{-1}
  endfor

  // Update
  μ_t = μ̄_t + Σ_i K_t^i (z_t^i − ẑ_t^i)
  Σ_t = (I − Σ_i K_t^i H_t^i) Σ̄_t
  return μ_t, Σ_t
```

Kalman gain $K_t^i$는 $(3N+3) \times 3$ 행렬이라 랜드마크 하나를 관측해도 상태 전체가 갱신된다. 공분산의 비대각 원소 덕분에 한 관측이 다른 랜드마크 추정까지 고친다. 갱신 비용은 $O(N^2)$이다.

대응을 모르면(unknown correspondence) 가상의 $(N_t+1)$번째 랜드마크를 맵 끝에 잠시 붙이고, 모든 후보에 대해 마할라노비스 거리를 계산해 ML 대응을 고른다. 거리가 임계값 $\alpha$를 넘으면 새 랜드마크로 등록한다. 이렇게 그때그때 정한 대응이 틀리면 그 오류는 뒤에서 바로잡히지 않는다.

EKF-SLAM은 지도가 커지면 한계가 드러난다. 공분산 행렬 $\Sigma \in \mathbb{R}^{(3N+3) \times (3N+3)}$의 메모리가 $N^2$에 비례한다. 랜드마크가 100개면 303×303, 1000개면 3003×3003이다. 랜드마크가 늘수록 과거의 선형화 오차가 쌓여 추정이 일관성을 잃는다(Bailey et al. 2006). 과거 포즈는 marginalize되어 상태에서 빠지므로 전체 사후분포를 다시 최적화할 수 없다.

그래서 랜드마크가 많은 지도에서는 smoothing이나 factor graph 방식을 많이 쓴다. EKF-SLAM은 랜드마크가 적은 시스템이나 fiducial 마커로 지도를 만들 때 지금도 쓴다. MSCKF 같은 sliding-window 필터는 EKF의 선형화와 공분산 전파를 쓰면서 랜드마크를 곧바로 상태에서 marginalize한다. 데이터 연관에서는 측정을 하나씩 짝짓는 ML 방식 대신 여러 측정의 짝을 함께 검사하는 JCBB를 쓰기도 한다.

#### 14.16.2 GraphSLAM — 정보형 배치 SLAM (PR §11)

EKF-SLAM에서는 측정이 들어올 때마다 dense한 $\Sigma$ 전체를 갱신한다. 정보형 $\Omega = \Sigma^{-1}$에서는 서로 독립인 factor의 기여를 해당 칸에만 더하므로, SLAM 그래프가 sparse하다는 사실이 행렬에 그대로 드러난다. 다만 motion update와 marginalization은 fill-in을 만든다.

##### 정보형의 직관: 스프링-매스 비유

EIF SLAM은 정보를 더할 수 있는 양으로 다룬다. 공분산 $\Sigma$ 대신 정보 행렬 $\Omega = \Sigma^{-1}$과 정보 벡터 $\xi = \Omega \mu$를 쓴다.

스프링과 질점으로 비유하면, 변수(포즈, 랜드마크)는 질점이고 $\Omega$의 비대각 원소는 두 질점을 잇는 스프링이다.
- Control $u_t$: $x_{t-1}$과 $x_t$ 사이의 스프링. 강성은 $R_t^{-1}$이다(motion noise가 작을수록 단단하다).
- Measurement $z_t^i$: 포즈 $x_t$와 랜드마크 $m_j$ 사이의 스프링. 강성은 $Q_t^{-1}$이다.
- 랜드마크끼리는 직접 잰 적이 없으므로 둘 사이를 바로 잇는 스프링은 없다.

**정보형 갱신식**:
$$\Omega \leftarrow \Omega + H_t^{iT} Q_t^{-1} H_t^i, \qquad \xi \leftarrow \xi + H_t^{iT} Q_t^{-1}[z_t^i - h(\mu_t) + H_t^i \mu_t]$$

측정 정보는 해당 칸에 더하기만 하면 된다. motion update와 marginalization에서는 변수 소거와 fill-in이 생긴다. Factor graph에서도 factor 하나가 변수 몇 개만 잇는다는 점에서 같은 스프링 비유가 통한다.

##### 4단계 파이프라인

GraphSLAM은 전체 사후분포 $p(x_{0:t}, m | z_{1:t}, u_{1:t})$를 정보형으로 한꺼번에(offline) 푼다. EIF와 SEIF도 정보형이지만 과거 상태를 marginalize하며 도는 온라인 필터다.

```
GraphSLAM_known_correspondence(u_{1:t}, z_{1:t}, c_{1:t}):
  1. Initialize:  μ_{0:t} ← motion model만으로 초기 추정 (관측 무시)
  2. Construct:   Ω = 0, ξ = 0에서 출발,
                  prior·controls·measurements를 국소 덧셈으로 누적
  3. Reduce:      각 랜드마크 j에 대해 Schur complement로 소거
                  Ω̄ ← Ω̄ − Ω_{τ(j),j} Ω_{j,j}^{-1} Ω_{j,τ(j)}
                  ξ̄ ← ξ̄ − Ω_{τ(j),j} Ω_{j,j}^{-1} ξ_j
                  → 포즈만 남은 reduced Ω̄, ξ̄
  4. Solve:       Σ_{0:t} = Ω̄^{-1},  μ_{0:t} = Σ_{0:t} ξ̄
                  각 랜드마크: μ_j = Ω_{j,j}^{-1}(ξ_j − Ω_{j,τ(j)} μ_{τ(j)})
  전체 2-3회 반복 (linearization 개선)
  return μ_{0:t}, {μ_j}
```

$\tau(j)$는 랜드마크 $j$를 본 포즈 시점 전체다. Reduce 단계에서는 *랜드마크 하나를 본 포즈들끼리 새 스프링으로 잇고 그 랜드마크 노드를 떼어 낸다*. 수학으로는 Bundle Adjustment의 Schur complement(§14.9.2)와 같다.

**Marginalization Lemma**: 선형 Gaussian 정보형의 marginal은 Schur complement로 표현된다. 이 연산은 남은 변수 사이에 fill-in을 만들 수 있다.
$$\bar\Omega_{xx} = \Omega_{xx} - \Omega_{xy} \Omega_{yy}^{-1} \Omega_{yx}$$

Thrun & Montemerlo(2006, IJRR)의 GraphSLAM은 궤적 전체와 지도의 사후분포를 sparse한 정보 그래프로 한꺼번에 최적화한다. 포즈 사이의 상대 관계를 전역 최적화하는 방법은 Lu & Milios(1997)가 먼저 내놓았다.

대응을 모를 때 GraphSLAM은 두 feature $(m_j, m_k)$가 같은 랜드마크인지 후보를 따져 보고, 고른 제약을 그래프에 더한 뒤 다시 최적화한다. factor를 따로 저장해 두면 고른 제약을 빼고 다시 최적화할 수도 있다. Switchable constraints(Sünderhauf & Protzel 2012)는 이와 다르게 loop factor를 켜고 끄는 정도를 연속 변수로 두고 함께 최적화한다.

초기화, factor 구성, 변수 소거, 풀이로 이어지는 GraphSLAM의 단계는 지금의 batch SLAM과 비슷하다. iSAM(Kaess et al. 2008)은 새 factor가 들어오면 QR 분해를 증분으로 갱신해 바뀌는 항목만 다시 계산하는 증분 평활화(incremental smoothing)다. iSAM2(Kaess et al. 2012)는 이 분해 구조를 Bayes tree라는 트리 자료구조로 관리하며, 필요한 변수만 다시 선형화한다.

#### 14.16.3 SEIF — Sparse Extended Information Filter (PR §12)

EIF는 온라인 필터로 쓸 수 있지만, motion update를 정확히 하면 정보 행렬이 dense해진다. SEIF는 active feature 수를 제한하고 sparsification 근사를 넣어, 지도 크기와 상관없이 갱신 비용이 일정하도록 만든 온라인 필터다. Thrun et al. (2004, IJRR)은 Victoria Park 3.5 km 실험의 해당 구현에서 EKF-SLAM보다 두 배쯤 빠르고 메모리는 4분의 1 미만을 쓰면서 오차는 비슷했다고 보고했다.

##### 4단계 업데이트

```
SEIF_SLAM_known_correspondences(ξ_{t-1}, Ω_{t-1}, μ_{t-1}, u_t, z_t, c_t):
  1. Motion update:       ξ̄_t, Ω̄_t, μ̄_t ← u_t로 정보형 갱신
                          (active feature + robot pose만 변경, sparse 유지)
  2. Measurement update:  Ω_t ← Ω̄_t + Σ_i H_t^{iT} Q_t^{-1} H_t^i  [가산]
                          ξ_t ← ξ̄_t + 해당 항 가산
  3. Sparsification:      일부 active feature를 passive로 강제
                          — robot과의 link를 끊고 정보를 인접 노드에 재분배
  4. State estimate:      amortized coordinate descent로
                          active feature 추정만 incremental 갱신
  return ξ_t, Ω_t, μ_t
```

##### Sparsification

변수 $a, b$ 사이의 직접 의존을 끊는 근사로 $\Omega$에 0을 만든다.

$$\tilde p(a,b,c) = \frac{p(a,c)\, p(b,c)}{p(c)} \quad \Longrightarrow \quad \Omega_{a,b} = 0$$

이 근사는 $c$가 주어졌을 때 $a$와 $b$를 독립으로 만드는 KL projection이다. sparsification과 반복 선형화가 필터의 일관성을 얼마나 해치는지는 따로 평가해야 한다.

PR 12장의 SEIF는 랜드마크를 signature 없이 $(x, y)$ 두 성분으로 두어 상태가 $2N+3$차원이다. §14.16.1의 EKF-SLAM은 PR 10장을 따라 signature까지 세 성분을 두므로 $3N+3$차원이다. Active feature 수 $K$를 상수로 고정하면 motion·measurement update에서 다루는 행렬이 $(2K+3) \times (2K+3)$을 넘지 않는다. 그래서 갱신 비용은 지도 크기와 상관없는 상수다. 지도 전체의 상태를 복원하는 비용과 데이터 연관 비용은 여기에 들어가지 않는다.

*Probabilistic Robotics*의 예시는 active feature를 6개쯤 쓴다. 센서와 지도에 따라 일관성과 계산량을 확인해서 정할 값이다. Eustice et al.(2006, TRO)은 과거 포즈를 상태에 남기는 delayed-state 구조에서는 정보 행렬이 근사 없이 sparse하다는 것을 보였다(Exactly Sparse Delayed-State Filter). feature 기반 SLAM에서는 Walter, Eustice, Leonard(2007, IJRR)의 ESEIF가 일관성을 지키면서 sparsity를 유지한다. PR 12장의 그림(초고 기준 Figure 12.2 measurement, 12.3 motion, 12.4 sparsification)은 각 단계에서 연결이 어떻게 바뀌는지 보여 준다.

##### 트리 기반 데이터 연관

정보형은 덧셈으로 갱신되므로 data association에서도 대응 제약을 더했다가 뺄 수 있다. 두 feature $m_i, m_j$가 같다는 soft constraint를

$$\Omega \leftarrow \Omega + F_{m_i - m_j}^T C\, F_{m_i - m_j}$$

로 더하고, 그 factor를 따로 저장해 두었다면 나중에 뺄 수 있다. 이 성질로 data association 트리를 A* 같은 탐색으로 뒤질 수 있지만, 최악의 경우 가설 수가 지수적으로 늘어난다. 잘못된 loop closure에 대비하는 방법으로는 Switchable constraints(Sünderhauf & Protzel 2012)의 switch 변수와 Max-mixtures(Olson & Agarwal 2013)의 mixture factor도 있다.

##### 다중 로봇 맵 융합

정보형에서는 서로 독립인 관측의 기여를 더하면 되므로 여러 로봇의 지도를 합치기 쉽다. 두 로봇 $j, k$의 상태를 같은 좌표계로 맞췄고 두 추정에 겹치는 prior나 관측이 없다면 정보 항을 그대로 더한다.

$$\Omega^{\text{fused}} = \Omega^{j \leftarrow k\text{-aligned}} + \Omega^k, \qquad \xi^{\text{fused}} = \xi^{j \leftarrow k\text{-aligned}} + \xi^k$$

겹치는 정보를 빼지 않고 더하면 같은 관측을 두 번 세어 실제보다 확신이 커진다. 공분산형에서는 covariance intersection 같은 보수적인 융합 방법을 쓴다. Nettleton et al.(2003)이 정보형 분산 융합을 다뤘고, DDF-SAM, Kimera-Multi, Swarm-SLAM은 통신 방식과 겹치는 정보를 처리하는 방식이 저마다 다르다(§14.13).

SEIF는 근사 정보 필터이고 iSAM2는 증분 평활화다. iSAM2도 비선형 문제에서는 어디서 선형화하고 언제 다시 선형화하는지에 따라 결과가 달라진다. VINS-Mono·OKVIS의 sliding-window marginalization과 MSCKF의 feature 소거는 상태 크기가 계속 커지지 않게 막는 기법이다. 이 방법들은 모두 sparsity를 지켜 계산량을 줄이려 한다(§14.9.2).

#### 14.16.4 EM Mapping

EKF-SLAM, EIF SLAM, SEIF는 모두 데이터 연관을 안다고 가정하거나 ML로 그때그때 정했다. 모호한 연관을 확률로 다루는 방법도 있었다. MHT와 JPDA는 여러 가설과 확률적 연관을 썼고, FastSLAM은 입자마다 연관을 따로 두었다. EM Mapping은 모르는 데이터 연관을 EM의 잠재 변수로 두고, 지도 추정과 연관 추정을 번갈아 푼다. 그래서 모호한 데이터도 버리지 않고 쓴다. Thrun, Fox, Burgard(1998, Machine Learning)의 원형은 RHINO 박물관 가이드 로봇(Burgard et al. 1999)에 변형되어 쓰였다.

##### E-step / M-step 구조

```
EM_mapping(d):
  m ← uniform map 초기화
  repeat until satisfied:
    // E-step (forward α)
    α^(0) = δ(⟨0,0,0⟩)
    for t = 1 to T:
      α^(t) = η P(o^(t)|s^(t),m) ∫ P(s^(t)|a^(t-1),s^(t-1)) α^(t-1) ds^(t-1)

    // E-step (backward β)
    β^(T) = uniform
    for t = T-1 downto 0:
      β^(t) = ∫ P(o^(t+1)|s^(t+1),m) P(s^(t+1)|a^(t),s^(t)) β^(t+1) ds^(t+1)

    // E-step (combine)
    Bel(s^(t)) = α^(t) · β^(t)   [정규화]

    // M-step
    for each cell ⟨x,y⟩, property l:
      m_{⟨x,y⟩=l} ∝ Σ_t ∫ P(o^(t)|s^(t),m_{⟨x,y⟩}=l) · I_{⟨x,y⟩ ∈ range} · Bel(s^(t)) ds^(t)
    정규화
  return m
```

$\alpha$는 앞으로 진행하는 위치추정(Markov localization)이고, $\beta$는 미래 데이터로 과거 belief를 고치는 역방향 계산이다. $\beta$ 덕분에 루프를 닫으면 과거 belief가 거슬러 올라가며 고쳐진다. 이 forward-backward 계산은 HMM에서 Baum-Welch 알고리즘의 E-step이 하는 계산과 같다.

M-step은 셀이 속성 l로 관측된 횟수를 그 셀이 관측된 전체 횟수로 나눈다. 횟수는 belief로 가중해서 센다. 원전(PR 초고 13장)의 장난감 예제에서는 10회쯤 반복하자 거의 수렴했다. 필요한 반복 수는 데이터에 따라 다르다.

##### Layered EM Mapping

기본 EM_mapping의 M-step은 센서 원뿔 안에서 지도의 기하 일관성을 깨뜨린다. Layered EM은 이를 고친 변형이다. 짧게 움직인 구간마다 로컬 occupancy grid를 먼저 만들고, EM은 그 로컬 맵의 *위치*만 최적화한다. **Deterministic annealing**으로 $\sigma$를 1.0에서 시작해 0 쪽으로 줄여 가며 EM이 local maximum에 갇히지 않게 한다.

```
layered_EM_mapping(d):
  1. 각 t: m^(t) = occupancy_grid(o^(t))  [로컬 맵 생성]
     Bel(s^(t)) ← uniform 초기화
  2. repeat until satisfied  [σ = 1.0 → 0]:
     E-step (α, β)  [layered perceptual model 사용]
     M-step (annealed): Bel(s^(t)) = η (α^(t) β^(t))^{1/σ}
     σ ← 0.9σ
  3. 각 로컬 맵의 ML 포즈 추출 → occupancy_grid()로 글로벌 합성
  return m_global
```

GNC(Yang et al. 2020)와 robust kernel scheduling도 deterministic annealing처럼 쉬운 목적 함수에서 시작해 점점 원래 문제로 옮겨 가며 나쁜 국소해를 피한다.

##### EM Mapping이 주류에서 벗어난 이유

EM_mapping과 layered_EM_mapping은 지금 SLAM 시스템에서 거의 쓰지 않는다.
- pose와 map을 E-step과 M-step으로 나눠 푸는 대신, factor graph로 함께 최적화하게 됐다.
- batch·offline 방식이라 실시간 SLAM에 쓰기 어렵다.
- Cartographer (Hess et al. 2016)는 EM 없이 scan matching과 pose graph 최적화로 직접 풀었다. GMapping (Grisetti et al. 2007)은 scan matching으로 제안 분포를 개선한 Rao-Blackwellized particle filter다.

layered EM과 Cartographer의 local/global SLAM은 둘 다 로컬 지도를 먼저 만들고 나중에 전체를 맞춘다.

#### 14.16.5 Cycle Posterior

한 걸음씩 ML로 지도를 갱신하는 mapper는 odometry 오차가 크면 지도가 크게 어긋나고, 과거 자세를 거슬러 고치지도 못한다. cycle posterior 방식은 ML mapper 옆에서 자세 사후분포 추정기를 *함께* 돌려 이 두 문제를 푼다.

**알고리즘 구조**:
```
Incremental Mapping with Posterior Estimation:
  1. incremental_ML_mapping(o, a, s, m)  → ⟨m', s'⟩    [ML 갱신]
  2. Bel(s') = η P(o|s') ∫ P(s'|a,s) Bel(s) ds          [사후분포 한 스텝]
  3. s'' = argmax Bel(s')                                 [사후 mode]
  4. s'' ≠ s'  →  cycle closure 검출
                   s'' − s'를 cycle 경로 따라 선형 분배
  5. incremental_ML_mapping을 시간 역방향으로 재실행       [nested ML refinement]
```

사후분포가 갑자기 좁아지면 cycle closure로 보고, 좁아진 분포의 mode와 ML 추정값의 차이를 보정량으로 쓴다. 원전은 사후분포 추정기를 MCL로 구현했고, odometry를 전혀 쓰지 않은 데이터에서도 cycle을 닫은 결과를 보였다. cycle을 찾는 단계와 과거를 거슬러 고치는 단계를 한 시스템에 묶은 초기 사례다. 지금의 SLAM도 장소를 알아보는 부분(place recognition)과 고치는 부분(graph optimization)을 나눈다. §14.15.1의 증분 평활화는 루프가 확인되면 영향을 받는 부분만 골라 다시 최적화한다.

#### 14.16.6 정리: 무엇이 살아남았나

| PR 알고리즘 | 핵심 기여 | 비교할 현대 구조 | 관계의 범위 |
|---|---|---|---|
| EKF-SLAM | 통합 landmark state, off-diagonal covariance | MSCKF류 filter, fiducial mapping | EKF를 공유하지만 state 구성은 다름 |
| GraphSLAM | 정보형 factor 가산, variable elimination, full trajectory | GTSAM, g2o, Ceres, iSAM2 | sparse least squares와 smoothing 관점이 이어짐 |
| SEIF | bounded active set, sparsification | ESEIF, bounded-state estimator | ESEIF는 SEIF의 일관성 문제를 고친 변형이고, bounded-state estimator는 별도 방법임 |
| EM Mapping | latent data association, forward-backward localization, submap | EM 기반 mapping, submap system | 통계 기법과 구조가 비슷한 정도이고, 직접 이어진 방법은 아님 |
| Cycle Posterior | online closure detection과 correction 결합 | place recognition + graph optimization | 기능 분할이 유사함 |

> **참고 자료**
> - [Thrun et al., "Simultaneous Localization and Mapping with Sparse Extended Information Filters" (IJRR 2004)](https://journals.sagepub.com/doi/10.1177/0278364904045479) — SEIF 원본 논문
> - [Thrun & Montemerlo, "The GraphSLAM Algorithm with Applications to Large-Scale Mapping of Urban Structures" (IJRR 2006)](https://journals.sagepub.com/doi/10.1177/0278364906065387) — EIF/GraphSLAM 정식화
> - [Dissanayake et al., "A Solution to the Simultaneous Localization and Map Building (SLAM) Problem" (IEEE T-RA 2001)](https://ieeexplore.ieee.org/document/938381) — EKF-SLAM 고전 정식화
> - [Kaess et al., "iSAM2: Incremental Smoothing and Mapping Using the Bayes Tree" (IJRR 2012)](https://www.cs.cmu.edu/~kaess/pub/Kaess12ijrr.pdf) — incremental smoothing과 Bayes tree의 원 논문
