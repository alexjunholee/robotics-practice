# Ch.14 — SLAM & Odometry


SLAM은 낯선 환경에서 로봇의 위치를 추정하면서 주변 지도를 함께 만드는 문제다. 실내나 지하처럼 GPS를 사용할 수 없는 공간에서 자율 이동의 기반이 된다.

---

## Part 1. 기초와 시스템

### 14.1 개념 소개

내비게이션과 경로 계획에는 로봇의 현재 위치와 주변 환경 정보가 필요하다. SLAM은 두 정보를 함께 추정한다.

**SLAM (Simultaneous Localization and Mapping)**:
자신의 위치를 추정하면서 동시에 주변 환경의 지도를 작성하는 문제이다.

닭과 달걀 문제:
- 지도가 있어야 위치를 알 수 있음
- 위치를 알아야 지도를 만들 수 있음
→ 동시에 해결

센서는 항상 노이즈가 있다. 바퀴가 미끄러지기도 하고, 카메라 이미지가 흔들리기도 한다. 이런 불확실성이 시간이 지날수록 누적되어 위치 추정이 점점 틀어진다(drift). SLAM의 핵심 도전은 이 drift를 보정하면서 일관된 지도를 만드는 것이다.

**Odometry vs SLAM**:
| 특징 | Odometry | SLAM |
|---|---|---|
| 출력 | 상대적 이동 | 위치 + 지도 |
| Loop Closure | 없음 | 있음 |
| Drift | 누적 | 보정 가능 |
| 계산량 | 적음 | 많음 |

> **추천 자료**
> - [Cyrill Stachniss — SLAM Course (University of Bonn)](https://www.youtube.com/playlist?list=PLgnQpQtFTOGQrZ4O5QzbIHgl3b1JHimN_) — Bayes filter부터 graph-based SLAM까지 이어지는 공개 강의 시리즈
> - [Thrun, Burgard, Fox, "Probabilistic Robotics" (Textbook)](https://mitpress.mit.edu/9780262201629/probabilistic-robotics/) — SLAM의 수학적 기반을 다루는 교과서. 칼만 필터, 파티클 필터, EKF-SLAM 등
> - [Barfoot, "State Estimation for Robotics" (Free PDF)](http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) — 상태 추정의 수학을 깊이 있게 다루는 교재. 무료 PDF 제공
> - [Awesome-SLAM GitHub](https://github.com/SilenceOverflow/Awesome-SLAM) — SLAM 관련 논문, 라이브러리, 데이터셋을 모아놓은 목록
> - [정진용 블로그 — SLAM 강의 시리즈 (Freiburg Robot Mapping 기반)](https://jinyongjeong.github.io/2017/02/13/lec01_SLAM_bayes_filter/) — Bayes filter부터 EKF/UKF/Particle filter, Graph SLAM, Robust SLAM까지 잇는 15편의 한국어 시리즈
> - [김기섭 블로그 — SLAM Back-end 공부자료 5개 추천](https://gisbi-kim.github.io/blog/2021/10/03/slam-textbooks.html) — Error-state KF, Factor Graphs, Bundle Adjustment 등 핵심 자료 큐레이션
> - [Robot Mapping Course (Uni Freiburg, Cyrill Stachniss)](http://ais.informatik.uni-freiburg.de/teaching/ws13/mapping/) — SLAM 강의 슬라이드와 과제 자료. 영상과 함께 보면 좋다
> - [EKF-SLAM 슬라이드 (Freiburg)](http://ais.informatik.uni-freiburg.de/teaching/ws12/mapping/pdf/slam04-ekf-slam.pdf) — 위 강의 중 EKF-SLAM 파트. 수식 전개가 깔끔하게 정리되어 있다

> **실습**: [SE(2) Odometry](https://alexjunholee.github.io/robotics-practice/app.html#se2_odometry)
> 2D 평면에서의 odometry 누적 과정을 직접 조작하며, drift가 어떻게 발생하는지 확인할 수 있다.

### 14.2 Visual Odometry (VO)

카메라만으로 상대적 이동을 추정한다. SLAM의 "front-end"에 해당하며, 여기서 추정한 이동이 부정확하면 SLAM 전체가 무너진다.

#### 14.2.1 Feature-based vs Direct Method

두 방식은 장단점이 뚜렷하다. 운용 환경에 따라 선택이 달라진다.

**Feature-based** (ORB-SLAM 계열)는 이미지에서 변하지 않는 특징적인 점(코너, 블롭 등)을 추출하고, 프레임 간 매칭으로 카메라 움직임을 역추정한다. 선형대수적으로는 Essential Matrix 또는 Fundamental Matrix를 구하는 문제다. 조명 변화에 강건하고 방법론이 검증되어 있지만, 흰 벽·텍스처 없는 바닥처럼 특징점을 뽑기 어려운 환경에서는 한계가 있다.

```
이미지 → 특징점 추출 → 매칭 → 움직임 추정
```

**Direct Method** (DSO, LSD-SLAM 계열)는 픽셀 밝기를 직접 비교한다. "연속 프레임에서 같은 3D 점을 관측하면 밝기가 같아야 한다"는 가정(brightness constancy)을 이용하므로, 특징점을 뽑을 필요가 없어 텍스처가 적은 환경에서도 작동할 수 있다. 대신 조명 변화에 민감하다.

```
이미지 → 픽셀 밝기 직접 비교 → 움직임 추정
```

#### 14.2.2 Mono vs Stereo vs RGB-D

각 구성의 트레이드오프를 알아야 실제 로봇에 맞는 센서를 고를 수 있다.

| 구성 | Scale | 특징 | 적합 환경 |
|---|---|---|---|
| **Monocular** | 불가 (ambiguity) | 경량·단순, IMU 없이 스케일 복원 불가 | 저비용 드론, 모바일 |
| **Stereo** | 가능 | 베이스라인이 측정 범위를 제한 | 일반 실내·실외 |
| **RGB-D** | 가능 | 깊이 직접 측정, 실외·직사광선에 취약 | 실내 구조화 환경 |

Scale ambiguity를 보충하면: 단안 카메라로는 "가까이 있는 작은 물체"와 "멀리 있는 큰 물체"를 구분할 수 없다. 모노 SLAM의 지도는 임의의 스케일로 나오며, IMU나 다른 센서로 복원해야 한다. 초기화 때 충분한 이동이 필요한 이유도 여기에 있다.

> **추천 자료**
> - [Daniel Cremers — Multiple View Geometry (TUM)](https://www.youtube.com/playlist?list=PLTBdjV_4f-EJn6udZ34tht9EVIW7lbeo4) — Visual Odometry에 필요한 다중 뷰 기하를 다루는 공개 강의
> - [EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) — Visual(-Inertial) Odometry 벤치마크 데이터셋
> - [TUM RGB-D Benchmark](https://cvg.cit.tum.de/data/datasets/rgbd-dataset) — RGB-D SLAM/VO에서 널리 쓰이는 실내 데이터셋과 평가 도구

### 14.3 Visual SLAM

#### 14.3.1 ORB-SLAM2/3

ORB-SLAM 계열은 Visual SLAM 논문에서 자주 쓰이는 공개 baseline 중 하나다. 코드가 공개되어 있어 직접 빌드하고 입력 조건과 실패 사례를 확인할 수 있다.

**구성**:
1. **Tracking**: 현재 프레임에서 포즈 추정
2. **Local Mapping**: 키프레임 기반 지역 지도 관리
3. **Loop Closing**: 루프 감지 및 전역 최적화

이 세 스레드 구조가 ORB-SLAM의 핵심 설계다. Tracking은 매 프레임 실시간으로, Local Mapping은 키프레임이 들어올 때, Loop Closing은 루프가 감지될 때 동작한다. 각각 다른 주기로 병렬 실행되므로, 실시간 성능을 유지하면서도 글로벌 일관성을 확보할 수 있다.

**ORB-SLAM3 특징**:
- Visual-Inertial 모드 지원
- 멀티맵 지원
- Fish-eye 카메라 지원

ORB-SLAM의 역사적 맥락:
- **MonoSLAM (2007)**: 실시간 단안 SLAM을 대표한 초기 시스템. EKF 기반으로 작동했으나, 맵 크기가 커지면 계산량이 급증하는 한계가 있었다.
- **PTAM (Parallel Tracking and Mapping, 2007)**: Tracking과 Mapping을 병렬 thread로 분리한 영향력 있는 초기 시스템. 이 아키텍처가 이후 ORB-SLAM에 큰 영향을 미쳤다.
- **ORB-SLAM (2015)**: PTAM의 설계를 계승하면서 ORB 특징점, Loop Closure, 재위치 추정(relocalization)을 추가한 완전한 SLAM 시스템.
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
> - [정진용 블로그 — Visual SLAM 비교 실험 (KAIST Urban Dataset)](https://jinyongjeong.github.io/2019/10/22/visual_slam_compare/) — ORB-SLAM2 vs VINS-Fusion 실전 비교. 실제 데이터셋에서의 성능 차이 분석

#### 14.3.2 DSO (Direct Sparse Odometry)

**Direct Method** + **Sparse Points**

Direct Method는 dense(모든 픽셀)하게 쓰이는 경우가 많고, Sparse는 Feature-based에서 쓰이는 방식인데, DSO는 "Direct이면서 Sparse"라는 조합을 사용한다. 선별된 고품질 점들만 사용하면서 광도(photometric) 오차를 최소화한다.

- 특징점 추출 없이 픽셀 밝기 직접 사용
- 선별된 점들만 사용 (Sparse)
- Photometric bundle adjustment

> **추천 자료**
> - [Engel et al., "Direct Sparse Odometry" (2018)](https://arxiv.org/abs/1607.02565) — DSO 논문

#### 14.3.3 VINS-Mono/Fusion

단안 카메라 추적은 빠른 움직임이나 텍스처가 부족한 환경에서 실패할 수 있다. IMU는 영상 프레임 사이의 고주파 운동 제약을 보완한다. VINS-Mono는 카메라와 IMU를 결합한 드론·모바일 로봇용 Visual-Inertial SLAM 시스템이다.

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

VINS-Mono는 IMU preintegration을 사용해 두 키프레임 사이의 여러 IMU 측정을 하나의 상대 운동 제약으로 묶는다. 최적화에서는 각 원시 측정을 다시 적분하는 대신 이 제약을 사용한다.

> **추천 자료**
> - [Qin et al., "VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator" (2018)](https://arxiv.org/abs/1708.03852) — VINS-Mono 논문
> - [VINS-Mono GitHub](https://github.com/HKUST-Aerial-Robotics/VINS-Mono) — 공식 코드, ROS 지원

### 14.4 LiDAR Odometry & SLAM

카메라 기반 방법은 조명과 텍스처의 영향을 받는다. LiDAR는 3D 거리를 직접 측정하므로 영상의 밝기나 텍스처에 같은 방식으로 의존하지 않는다. 이 차이 때문에 자율주행과 실외 로봇에서 LiDAR SLAM을 사용한다.

#### 14.4.1 LOAM (Lidar Odometry and Mapping)

LOAM의 edge·planar feature 분리와 odometry-mapping 이중 주기 구조는 LeGO-LOAM, LIO-SAM 등 후속 LiDAR SLAM에 영향을 주었다.

- Edge points와 Planar points 분류
- Point-to-edge, point-to-plane 거리 최소화
- Odometry와 Mapping 분리 (주파수 다르게)

포인트 클라우드에서 모서리와 평면에 해당하는 점을 추려 사용한다. 모든 점을 매칭하는 대신 edge·planar 점으로 제약을 구성해 계산량을 줄인다.

#### 14.4.2 LeGO-LOAM

**Lightweight and Ground-Optimized LOAM**:
- 지면 분리로 계산량 감소
- 지면을 기반으로 초기 추정
- 모바일 로봇에 적합

#### 14.4.3 LIO-SAM

LIO-SAM은 Factor Graph 기반 최적화를 LiDAR-Inertial SLAM에 적용한 대표적인 방법이다. Factor Graph에서는 새 센서의 측정을 factor로 추가할 수 있어 시스템을 확장하기 쉽다.

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

Factor Graph가 뭐냐면: 변수(로봇 포즈, 랜드마크 위치)와 제약(센서 측정)의 관계를 그래프로 표현하는 것이다. IMU 측정이 factor 하나, LiDAR 매칭이 factor 하나, GPS가 factor 하나, Loop Closure가 factor 하나... 이런 식으로 센서를 추가하려면 해당 factor만 추가하면 된다. GTSAM 라이브러리가 이 최적화를 효율적으로 수행한다.

> **추천 자료**
> - [Shan et al., "LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping" (2020)](https://arxiv.org/abs/2007.00258) — LIO-SAM 논문
> - [Vizzo et al., "KISS-ICP: In Defense of Point-to-Point ICP" (RA-L 2023, arXiv:2209.15397)](https://arxiv.org/abs/2209.15397) — 잘 만든 vanilla ICP가 복잡한 LiDAR odometry와 동등한 성능. 단순함의 힘
> - [LIO-SAM GitHub](https://github.com/TixiaoShan/LIO-SAM) — 공식 코드, ROS 지원
> - [GTSAM Documentation](https://gtsam.org/) — Factor Graph 최적화 라이브러리. LIO-SAM, ORB-SLAM3 등 다양한 SLAM 시스템의 백엔드로 사용된다
> - [Frank Dellaert — Factor Graphs for Perception and Action (MIT Robotics)](https://www.youtube.com/watch?v=-yCC7mpgL4w) — GTSAM 개발자가 직접 설명하는 Factor Graph
> - [김기섭 블로그 — Scan Context-based LiDAR Pose-graph SLAM 구현](https://gisbi-kim.github.io/blog/2021/05/17/sclidarslam.html) — Scan Context를 LiDAR SLAM에 통합한 구현 해설

#### 14.4.4 FAST-LIO / FAST-LIO2

**Fast LiDAR-Inertial Odometry**:
- Kalman Filter 기반 (optimization 대신)
- ikd-Tree: 동적 KD-트리로 빠른 매핑
- 실시간 성능

FAST-LIO가 왜 빠른지: LIO-SAM이 Factor Graph 최적화(비선형 최소자승법)를 사용하는 반면, FAST-LIO는 Iterated Extended Kalman Filter(IEKF)를 사용한다. 최적화 문제를 풀지 않고 필터링으로 처리하니, 계산이 훨씬 가볍다. 또한 ikd-Tree라는 증분적 KD-트리를 사용해서 맵에 새 점을 추가할 때도 빠르다.

> **추천 자료**
> - [Xu & Zhang, "FAST-LIO: A Fast, Robust LiDAR-Inertial Odometry Package by Tightly-Coupled Iterated Kalman Filter" (2021)](https://arxiv.org/abs/2010.14709) — FAST-LIO 논문
> - [Xu et al., "FAST-LIO2: Fast Direct LiDAR-Inertial Odometry" (2022)](https://arxiv.org/abs/2107.06829) — FAST-LIO2 논문
> - [FAST-LIO2 GitHub](https://github.com/hku-mars/FAST_LIO) — 공식 코드

### 14.5 Multi-sensor Fusion

단일 센서로 모든 상황을 커버하기는 어렵다. 카메라는 어두우면 안 되고, LiDAR는 비가 오면 힘들고, IMU만으로는 drift가 커진다. 센서를 결합(fusion)하면 각 센서의 약점을 다른 센서가 보완한다.

#### 14.5.1 Camera + IMU (VIO)

Visual과 Inertial을 결합하는 방식에는 두 가지 전략이 있다. **Loosely-coupled**는 카메라와 IMU가 각자 상태를 추정한 뒤 결과를 covariance 기반으로 합친다. 구현이 단순하지만 정보를 충분히 활용하지 못한다. **Tightly-coupled**는 카메라 특징점의 재투영 오차와 IMU의 가속도/각속도 측정을 하나의 비용 함수에 넣고 동시에 최적화한다(VINS-Mono, MSCKF). 더 정확하지만 구현이 복잡하다.

**IMU Preintegration**:
두 키프레임 사이의 IMU 측정을 사전 적분하여 상대 변환 계산. 재선형화 없이 최적화 가능.

#### 14.5.2 LiDAR + IMU (LIO)

회전형 LiDAR와 IMU의 rate는 제품마다 다르지만 LiDAR scan 주기 안에 platform이 움직이면 motion distortion이 생긴다. LIO는 더 높은 rate의 IMU와 timestamp를 이용해 scan 안의 움직임을 보정(de-skewing)하고 LiDAR 관측과 함께 상태를 추정한다. 보정의 이득은 동기화, IMU bias, motion과 scan pattern에 달려 있다.

#### 14.5.3 Camera + LiDAR + IMU

**최신 트렌드**: 모든 센서 통합
- 예시: R3LIVE, LVI-SAM
- 각 센서의 장점 활용

R3LIVE는 LiDAR(기하 정보) + Camera(텍스처/색상 정보) + IMU(고속 움직임 보상)를 모두 결합한다. 정확한 위치 추정뿐 아니라 색상이 입혀진(colored) 고밀도 3D 맵까지 실시간으로 생성할 수 있다.

> **추천 자료**
> - [KITTI Odometry Benchmark](https://www.cvlibs.net/datasets/kitti/eval_odometry.php) — LiDAR/Visual Odometry 벤치마크의 표준
> - [EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) — VIO 벤치마크 데이터셋
> - [Lin & Zhang, "R3LIVE: A Robust, Real-time, RGB-colored, LiDAR-Inertial-Visual tightly-coupled state Estimation and mapping package" (2022)](https://arxiv.org/abs/2109.07982) — 3센서 융합의 대표작
> - [김기섭 블로그 — Filter-based VIO: MSCKF 계열 history 정리](https://gisbi-kim.github.io/blog/2021/04/27/msckf-history.html) — MSCKF 원본부터 stereo 확장까지 계보 정리

### 14.6 Loop Closure & Global Optimization

SLAM을 오래 실행하면 지도가 점차 뒤틀린다. 로봇이 큰 원을 그리며 출발점으로 돌아와도 지도에서는 시작점과 끝점이 어긋날 수 있다. Loop Closure는 이전에 방문한 장소를 다시 인식해 이 오차를 교정한다. 대규모 환경에서 누적 drift를 억제하려면 이 과정이 필요하다.

#### 14.6.1 Place Recognition

이전에 방문한 장소를 인식하여 drift를 보정한다.

같은 장소라도 시간, 조명, 계절이 바뀌면 아예 다르게 보인다. 비슷하게 생긴 다른 장소를 같은 장소로 오인하면(false positive) 지도가 오히려 더 망가진다. Place Recognition의 정밀도(precision)가 매우 높아야 하는 이유다.

**Bag of Words (BoW)**: visual vocabulary를 기반으로 이미지 간 유사도를 계산한다. DBoW2 라이브러리가 대표적이며 ORB-SLAM에서 사용된다. 빠르고 검증되어 있지만 조명·시점 변화에 취약하다.

**NetVLAD**: 딥러닝 기반 end-to-end 학습으로 조명·날씨 변화에 강건한 글로벌 디스크립터를 생성한다. (14.14절 참조)

**LiDAR Place Recognition**: Scan Context는 포인트 클라우드를 bird-eye view 2D 디스크립터로 압축하고, PointNetVLAD는 포인트 클라우드에서 직접 학습한다.

#### 14.6.2 Pose Graph Optimization

루프가 감지되면 전체 경로를 보정한다.

```
노드: 로봇 포즈
에지: 상대 변환 (odometry, loop closure)

목표: 모든 에지 제약을 만족하는 노드 위치 찾기
```

직관적으로 설명하면: Odometry가 만든 경로는 "각 구간은 대충 맞지만, 전체적으로는 뒤틀린" 상태이다. Loop Closure가 "이 위치와 저 위치가 같은 곳이다"라는 제약을 추가하면, Pose Graph Optimization이 "모든 제약을 최대한 만족하도록" 전체 경로를 부드럽게 조정한다. 이것은 비선형 최소자승법 문제이다.

주요 도구로는 경량 pose graph/BA 전용인 **g2o** (ORB-SLAM), factor graph와 iSAM2 기반의 **GTSAM** (LIO-SAM), Google이 개발한 범용 비선형 최소자승 라이브러리 **Ceres Solver**가 있다. 선택 기준은 14.9.4절의 비교표를 참고하라.

> **추천 자료**
> - [GTSAM Documentation & Tutorials](https://gtsam.org/) — Factor Graph 기반 최적화 라이브러리. Pose Graph Optimization 예제 포함
> - [Cyrill Stachniss — Graph-based SLAM](https://www.youtube.com/watch?v=uHbRKvD8TWg) — Pose Graph Optimization의 직관적 설명
> - [g2o GitHub](https://github.com/RainerKuemmerle/g2o) — Graph Optimization 프레임워크
> - [정진용 블로그 — Robust Graph SLAM](https://jinyongjeong.github.io/2017/03/04/lec15_Robust_Graph_SLAM/) — M-estimator, Max-mixture, DCS 등 robust SLAM 기법 한글 해설

> **실습**: [Pose Graph Optimization](https://alexjunholee.github.io/robotics-practice/app.html#pose_graph)
> Pose Graph의 노드(포즈)와 에지(제약)를 조작하고, loop closure 추가 시 전체 경로가 어떻게 보정되는지 확인할 수 있다.

### 14.7 Localization

사전 지도 기반으로 현재 위치를 추정한다. SLAM이 "지도를 만들면서 위치를 추정"하는 것이라면, Localization은 "이미 만들어진 지도에서 위치만 추정"하는 것이다. 실제 서비스 로봇은 SLAM으로 미리 지도를 만들고, 운용 중에는 Localization만 수행하는 경우가 많다.

MCL 알고리즘의 *원리·유도*는 §3.11 비모수 필터 (Ch.3 참조). EKF의 수학적 기반은 §3.10 (Ch.3 참조), IMU와 결합한 위치추정 확장은 §14.10 참조. 아래에서는 위치추정 시나리오 분류와 알고리즘 변형을 본다.

Map-based Localization은 미리 만들어진 지도를 사용하므로 SLAM보다 계산이 가볍지만, 환경이 바뀌면 지도 업데이트가 필요하다.

**Monte Carlo Localization (MCL)**:
- 파티클 필터 기반
- 2D LiDAR + 점유 격자 지도
- ROS AMCL 패키지

MCL의 직관: 수천 개의 "가상 로봇(파티클)"을 지도 위에 뿌린다. 각 파티클은 "나는 여기에 이런 방향으로 있다"라는 가설이다. 실제 센서 측정과 비교해서, 측정과 잘 맞는 파티클은 살아남고 안 맞는 파티클은 사라진다. 시간이 지나면 파티클들이 실제 위치 주변에 모이게 된다.

**LiDAR Localization**: 포인트 클라우드 맵에 ICP 또는 NDT 매칭으로 정밀하게 위치를 추정한다.

> **추천 자료**
> - [Cyrill Stachniss — Monte Carlo Localization](https://www.youtube.com/watch?v=MsYlueVDLI0) — MCL/파티클 필터의 직관적 설명
> - [ROS Navigation Stack — AMCL](http://wiki.ros.org/amcl) — ROS에서 MCL 사용하기

> **실습**: [Particle Filter](https://alexjunholee.github.io/robotics-practice/app.html#particle_filter)
> 파티클 필터 기반 로봇 위치 추정 과정을 시각화하며, 파티클의 수렴 과정을 인터랙티브하게 확인할 수 있다.

> **실습**: [Occupancy Grid](https://alexjunholee.github.io/robotics-practice/app.html#occupancy_grid)
> 2D 점유 격자 지도를 구축하는 과정을 시각화하며, 센서 측정이 어떻게 확률적 지도로 변환되는지 확인할 수 있다.

#### 14.7.1 위치추정 문제 분류

위치추정의 난이도는 단일 수치로 표현되지 않는다. 네 축이 교차하며 알고리즘 선택을 결정한다.

| 축 | 옵션 | 비고 |
|---|---|---|
| 사전지식 | position tracking → global localization → kidnapped robot | 난이도 상승 |
| 환경 | static (로봇만 이동) → dynamic (사람·문·조명) | 동적일수록 어려움 |
| 능동성 | passive (관찰만) → active (탐색 행동 선택) | active가 더 빠른 수렴 |
| 로봇 수 | single → multi (상호 관측으로 belief 공유) | multi는 정보 풍부 |

대표 시나리오는 position tracking과 global localization, 여기에 kidnapped robot이 확장으로 더해진다.

**Position tracking**: 초기 자세가 알려져 있고, belief가 좁은 단봉 Gaussian으로 유지된다. EKF Localization이 적합하다.

**Global localization**: 초기 자세를 모른다. 균등 분포에서 시작하여 측정이 쌓이면서 belief가 수렴해야 한다. 다봉(multi-modal) belief 표현이 필요하므로 Grid Localization 또는 MCL이 적합하다.

**Kidnapped robot**: 운용 중 로봇이 강제로 다른 위치로 옮겨진다. 로봇이 그 사실을 스스로 알아채지 못한다는 점에서 global localization보다 어렵다. 어떤 알고리즘도 언젠가 이 상황을 만나므로, 복구 능력 자체가 로봇 자율성의 척도가 된다.

ROS Nav2의 `recovery_alpha_slow/fast` 파라미터는 kidnapped 시나리오 대비 설계다. warehouse AGV·청소로봇의 부팅은 global localization에, 운용 중은 tracking에 해당한다.

#### 14.7.2 Markov Localization

Markov localization은 알고리즘이 아니라 **베이즈 필터를 위치추정 문제에 그대로 적용한 것**의 이름이다. EKF Localization·Grid Localization·MCL은 모두 이 베이즈 필터의 belief 표현 방식에서 갈라진다.

베이즈 필터(Ch.3 §3.9)와의 차이는 단 하나다: 운동 모델과 관측 모델에 **지도 m**이 추가 입력으로 들어간다.

```
Markov_localization(bel(x_{t-1}), u_t, z_t, m):
  for all x_t do
    bel̄(x_t) = ∫ p(x_t | u_t, x_{t-1}, m) bel(x_{t-1}) dx_{t-1}   // motion update
    bel(x_t) = η p(z_t | x_t, m) bel̄(x_t)                          // measurement update
  endfor
  return bel(x_t)
```

초기 belief bel(x_0)은 시나리오마다 다르게 초기화한다:
- Position tracking: $\text{bel}(x_0) = \mathcal{N}(x_0;\, \bar{x}_0, \Sigma)$ — 좁은 Gaussian
- Global localization: $\text{bel}(x_0) = 1/|X|$ — 모든 합법 자세에 균등
- Partial knowledge: 알려진 구역 근방에만 균등, 그 외 0

§14.7.3~§14.7.7의 알고리즘은 "위 박스의 bel 표현을 무엇으로 구현하는가"의 변주다.

#### 14.7.3 EKF Localization

EKF Localization은 Markov localization의 특수 케이스로 belief을 $(\mu_t, \Sigma_t)$ 가우시안으로 표현한다. **단봉(unimodal) 가정 → position tracking 전용**이다. Global localization과 kidnapped 문제는 다봉 belief을 요구하므로 EKF로 풀 수 없다.

§3.10.2의 EKF를 위치추정에 적용한 것이므로 (Ch.3 참조), 여기서는 feature 기반 지도 + 알려진 랜드마크 대응이라는 가정 구조와 구체 알고리즘을 본다.

**가정**: 지도 m이 feature 기반 (점 랜드마크 집합). 각 측정 $z_t^i = (r, \phi, s)^T$ (range, bearing, signature). Correspondence $c_t^i$는 알려짐 (ARTag·QR 코드·Eiffel Tower 같은 식별 가능 랜드마크).

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

**조건부 독립 가정** $p(z_t | x_t, m) = \prod_i p(z_t^i | x_t, m)$ 아래에서는 측정을 쌓아 한 번에 갱신하거나 순차적으로 conditioning할 수 있다. 위 코드는 각 측정 뒤의 $(\mu_t, \Sigma_t)$를 다음 측정에 사용하는 순차형이다. 비선형 모델에서는 재선형화 여부에 따라 stacked update와 작은 차이가 날 수 있다.

실용적 한계 — *Probabilistic Robotics*의 예시는 heading 불확실도가 약 ±20°를 넘는 경우를 선형화가 위험해지는 경험적 구간으로 든다. 이는 보편 임계값이 아니다. 관측 기하, motion, noise에 따라 NIS·NEES나 Monte Carlo consistency를 확인해야 한다. EKF localization 구조는 식별 가능한 ARTag·AprilTag landmark나 GNSS+IMU fusion처럼 belief가 단봉으로 유지되는 문제에 계속 적용할 수 있다.

**미지 대응(unknown correspondences)**: 실전에서 $c_t^i$는 보통 모른다. ML(maximum likelihood) data association은 마할라노비스 거리가 최소인 지도 랜드마크를 선택하는 방법이다.

$$j(i) = \arg\min_k (z_t^i - \hat{z}_t^k)^T \Psi_k^{-1} (z_t^i - \hat{z}_t^k), \quad \Psi_k = H_t^k \bar\Sigma_t H_t^{k,T} + Q_t$$

마할라노비스 거리 최소화는 공분산 determinant와 prior가 후보마다 같다는 조건에서 Gaussian log-likelihood 최대화와 대응한다. 실전에서는 (1) 측정 차원에 맞는 $\chi^2$ gate, (2) 한 프레임의 여러 측정에 대한 one-to-one assignment 같은 제약을 더한다. ORB-SLAM의 descriptor matching과 geometric verification도 후보 생성 뒤 outlier를 거른다는 점에서는 비교할 수 있지만, 이 EKF의 ML association을 그대로 구현한 것은 아니다.

#### 14.7.4 Multi-Hypothesis Tracking (MHT)

EKF는 단봉 Gaussian이라 데이터 연관 모호성을 표현하지 못한다. MHT는 belief를 **가우시안 혼합(Gaussian mixture)**으로 표현하여 여러 가설을 동시에 유지한다.

각 가설 $h$는 독립적인 EKF를 구동한다. 측정이 들어오면 각 가설을 확장하고, 가중치(사후확률)가 임계값 $\psi_{\min}$ 아래로 떨어진 가설은 가지치기한다. 가설 수가 폭발하는 것을 막으려면 가지치기 정책이 필수다.

자율주행 multi-object tracking(MOT)에서도 Mahalanobis gating과 Hungarian assignment를 자주 결합한다. 이는 단일 assignment를 고르는 방식이며, 여러 association 가설을 시간에 걸쳐 유지하는 MHT와는 구분해야 한다.

#### 14.7.5 Grid Localization

MHT가 가우시안 혼합으로 belief를 표현한다면, Grid Localization은 포즈 공간 전체를 격자로 나누고 셀마다 확률을 누산하는 더 직접적인 방법이다.

포즈 공간을 셀로 이산화한 **히스토그램 필터**다. EKF가 풀지 못하는 global·multi-modal belief을 표현할 수 있지만, 셀 수 $K$에 비례하는 계산 비용이 단점이다.

```
Grid_localization({p_{k,t-1}}, u_t, z_t, m):
  for all k do
    p̄_{k,t} = Σ_i p_{i,t-1} · motion_model(mean(x_k), u_t, mean(x_i))
    p_{k,t}  = η · measurement_model(z_t, mean(x_k), m) · p̄_{k,t}
  endfor
  return {p_{k,t}}
```

$\text{bel}(x_t) = \{p_{k,t}\}$이며 각 셀 $x_k$에 확률 하나, 합은 1이다.

**해상도 트레이드오프**: 셀이 작을수록 추정 오차 감소 — 셀이 5 cm이면 LiDAR 오차 4 cm, 65 cm이면 오차 25 cm (PR 실험 기준). 반면 셀이 작을수록 global localization의 CPU 시간이 급증한다. 실시간 트릭으로는 raycast 결과 캐싱, 스캔 서브샘플링, 선택적 업데이트(임계 이상 셀만)가 있다.

이산 격자로 global을 표현하면서도, "왜 파티클 필터가 더 나은가"를 이해하는 교육적 다리 역할을 한다. ROS `amcl` 노드는 Grid Localization의 격자를 파티클로 교체한 것이다.

#### 14.7.6 MCL 알고리즘 (보강)

MCL의 원리와 유도는 §3.11 (Ch.3 참조). 여기서는 위치추정 알고리즘으로서의 전체 골격을 명시한다.

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

세 단계: **predict (sample) → weight → resample**. 초기화는 시나리오에 따라 다르다: global localization이면 자유공간 균등 분포에서 $M$개 샘플, position tracking이면 좁은 Gaussian에서 샘플한다.

**계산 자원 적응성**: $M$을 고정하지 않고 "다음 측정이 도착할 때까지 가능한 한 많이 샘플"하면 CPU가 빠를수록 $M$ 증가, 정확도 자동 향상된다.

proposal이 motion model이므로, perfect sensor(측정 우도가 극도로 좁은) 환경에서는 거의 모든 입자의 가중치가 0에 가까워진다. Mixture MCL(§14.7.8)이 이 문제를 해결한다.

ROS2 Nav2의 `nav2_amcl`이 이 구조를 그대로 구현한다.

#### 14.7.7 Augmented MCL — 납치 복구

표준 MCL은 납치에 취약하다. 입자들이 하나의 자세로 수렴한 뒤 로봇이 강제로 옮겨지면, 어떤 입자도 새 위치 근방에 없어 복구 경로가 없다. Augmented MCL은 측정 우도의 **단기 평균이 장기 평균에 비해 갑자기 떨어지면 무작위 입자를 주입**한다. "센서가 갑자기 지도와 안 맞기 시작했다 = 길 잃었다"는 직관을 두 지수이동평균의 비율로 수치화한다.

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

평소에는 $w_{\text{fast}} \approx w_{\text{slow}}$ → 비율 $\approx 1$ → 주입 확률 $\approx 0$ → 표준 MCL과 동일. 납치 직후에는 측정이 어디에도 안 맞음 → $w_{\text{fast}}$ 급락 → 주입 확률 상승. 장기 평균이 따라잡으면 비율 다시 1 → 주입 멎음.

단순 노이즈 스파이크는 $w_{\text{slow}}$가 반응하지 않으므로 false positive가 억제된다.

ROS `amcl`의 `recovery_alpha_slow`·`recovery_alpha_fast` 파라미터가 이 적응형 random-pose 주입을 제어한다. 연구 시스템에서는 NetVLAD류 place recognition과 PnP가 낸 pose 후보를 particle proposal로 섞기도 하지만, 이는 기본 AMCL 동작이 아니며 목표 환경에서 별도로 검증해야 한다.

#### 14.7.8 Mixture MCL

Augmented MCL이 무작위 포즈를 주입하는 것과 달리, Mixture MCL은 **proposal 분포 자체를 바꾼다**. 일부 입자를 motion model이 아니라 **측정 모델**에서 직접 샘플한다.

$$x_t^{[k]} \sim \begin{cases} p(z_t | x_t, m) & \text{확률 } \rho \\ \text{sample\_motion\_model}(u_t, x_{t-1}^{[k]}) & \text{확률 } 1 - \rho \end{cases}$$

측정에서 바로 샘플한 입자는 센서 정보가 강한 곳에 집중되므로, 저잡음 센서 환경에서 기본 MCL의 proposal 비효율을 해결한다. 납치 복구와 low-noise 센서 실패 모두를 다룬다는 점이 Augmented MCL과 다른 강점이다. 단, $p(z_t | x_t, m)$에서 직접 샘플하려면 역방향 센서 모델이 필요하다는 구현 부담이 있다.

#### 14.7.9 동적 환경 필터링

동적 물체(사람, 차량)가 있는 환경에서는 빔의 일부가 지도에 없는 장애물을 관측한다. 빔 센서 모델의 short hit 성분 $p_{\text{short}}(z | x, m)$의 사후확률을 이용해 의심스러운 빔을 위치추정에서 제외한다.

각 빔 $z_t^k$에 대해 네 성분 혼합 모델(§2.7, Ch.2 참조)을 평가하고, short 성분의 사후확률이 높은 빔은 가중치 계산에서 배제한다. 이 필터링이 없으면 복도에 사람이 많을 때 MCL이 흔들린다.

#### 14.7.10 필터 비교 정리

| 알고리즘 | Belief 표현 | Position tracking | Global loc | Kidnapped | 계산 비용 |
|---|---|---|---|---|---|
| EKF Loc | Gaussian (μ, Σ) | 좋음 | 불가 | 불가 | O(N) |
| MHT | Gaussian 혼합 | 좋음 | 제한 | 제한 | O(H·N) |
| Grid Loc | 히스토그램 | 좋음 | 가능 | 가능 | O(K) |
| MCL | 파티클 집합 | 좋음 | 가능 | Augmented MCL로 가능 | O(M) |

N은 랜드마크 수, H는 가설 수, K는 격자 셀 수, M은 파티클 수다. EKF는 Gaussian 단봉 가정 때문에 global/kidnapped 문제를 다룰 수 없다. Grid와 MCL은 계산 자원을 조절해 정확도와 속도의 균형을 선택할 수 있다.

#### 14.7.11 실무 고려: 랜드마크 효율·Negative Information

실제 EKF Localization을 구현할 때 자주 부딪히는 문제들이 있다.

**효율적 랜드마크 검색**: 지도에 N개 landmark가 있을 때 매 관측마다 전수 검색은 O(N)이다. 낮은 차원에서 균형 잡힌 KD-tree의 평균 query는 O(log N)에 가깝지만 최악에는 O(N)이며, grid index의 비용은 cell occupancy와 검색 반경에 달려 있다.

**Mutual exclusion**: 한 프레임에서 두 측정이 동일 랜드마크에 대응될 수 없다. ML data association은 component-wise 최적화라 이 제약을 자동으로 강제하지 않는다. 충돌 쌍이 생기면 마할라노비스 거리가 더 작은 측정을 선택하고 나머지를 버리는 repair 단계가 필요하다.

**Outlier rejection**은 마할라노비스 거리가 $\chi^2_{95\%}$ 임계값을 초과하는 측정을 제거한다. 이 한 줄이 EKF의 brittleness를 크게 줄인다.

**Negative information**: "이 각도 범위에서 랜드마크가 관측되지 않았다"는 정보도 위치추정에 유용할 수 있지만, 정확한 확률 처리가 복잡하고 구현 부담이 크다. 대부분의 실용 시스템에서 negative information은 무시된다.

---

### 14.7B Occupancy Grid Mapping

§14.7은 지도가 *주어졌을 때* 위치를 추정했다. 여기서는 반대 방향, 즉 위치가 알려진 상태에서 지도를 *만드는* 문제인 Occupancy Grid Mapping을 다룬다. Occupancy Grid Mapping은 **위치가 알려진 상태에서 셀별 점유 확률을 추정**하며, pose graph 최적화로 얻은 자세 궤적에서 최종 지도를 만드는 후처리 단계에서 핵심으로 쓰인다. 실제 SLAM 파이프라인에서는 두 단계가 번갈아 작동한다: pose graph 최적화로 자세 궤적을 확정한 뒤, 이 절의 알고리즘으로 최종 지도를 완성한다. binary Bayes 필터의 기반은 §3.11.2 (Ch.3 참조).

#### 14.7B.1 도입: 지도 작성의 어려움

매핑이 위치추정보다 더 어렵다는 말이 있다. 위치는 연속적 $x_t \in \mathbb{R}^3$이지만, 지도 m은 수만~수백만 개의 셀로 이루어진 고차원 불연속 변수다. 가능한 지도의 수가 $2^{|m|}$이므로 직접 탐색은 불가능하다.

이 조합 폭발을 피하는 핵심 가정 둘: (1) **자세를 안다**(알려진 $x_{1:t}$), (2) **셀들은 조건부 독립**이다. 두 번째 가정 덕에 지도 사후확률을 셀별 marginal의 곱으로 분해하여, 전체 문제를 셀마다 독립적인 binary Bayes 필터로 쪼갤 수 있다.

$$p(m \mid z_{1:t}, x_{1:t}) = \prod_i p(m_i \mid z_{1:t}, x_{1:t})$$

추가 어려움: 센서 잡음·지각 모호성(같은 위치에서 다른 측정)·환경 동적 변화·닫힌 루프에서의 오차 누적이 있다.

#### 14.7B.2 표준 알고리즘: Log-Odds 누산

각 셀의 occupancy 사후확률을 **log-odds** 형태로 누산한다.

$$l_{t,i} = \log \frac{p(m_i \mid z_{1:t}, x_{1:t})}{1 - p(m_i \mid z_{1:t}, x_{1:t})}$$

prior log-odds: $l_0 = \log[p(m_i) / (1 - p(m_i))]$. binary Bayes 필터(§3.11.2, Ch.3 참조)의 유도로부터 갱신식은:

$$l_{t,i} = l_{t-1,i} + \text{inverse\_sensor\_model}(m_i, x_t, z_t) - l_0$$

직관: 새 측정이 셀 $m_i$에 hit 증거를 주면 log-odds가 오르고, free 증거를 주면 내린다. $-l_0$ 항이 prior의 이중 계상을 막는다.

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
  if 빔 밖이거나 z_t^k + α/2 너머:
    return l_0                   // 정보 없음
  if |r − z_t^k| < α/2:
    return l_occ                 // hit (> l_0)
  if r ≤ z_t^k:
    return l_free                // free (< l_0)
```

$\alpha$는 장애물 두께 파라미터, $\beta$는 빔 개구각. ROS Nav2의 `costmap_2d`, SLAM Toolbox, Cartographer의 submap representation이 이 log-odds 누산을 그대로 사용한다.

#### 14.7B.3 다중 센서 융합

카메라·LiDAR·소나·적외선이 서로 다른 inverse_sensor_model을 가진다. 융합 전략 중 가장 단순한 것은 **셀별 최대값(conservative max)**이다: 어떤 센서라도 hit을 보고하면 그 셀은 occupied로 분류한다. 이 보수적 정책은 충돌 회피에서 안전하지만, 자유 공간을 과소평가하는 경향이 있다.

각 센서의 log-odds 업데이트를 독립적으로 누산한 후 셀별 합계를 구하는 방법도 있다. 이 경우 센서마다 정보량이 다른 경우 가중 합산이 필요하다.

#### 14.7B.4 inverse_sensor_model 학습

수작업으로 설계한 inverse_sensor_model은 간단한 기하학적 모델이다. **forward 모델 $p(z | x, m)$을 이미 갖고 있다면 역방향을 학습으로 도출**할 수 있다.

절차: (자세, 측정, 점유) 삼중쌍 $\{(x^{(k)}, z^{(k)}, m_i^{(k)})\}$를 시뮬레이션으로 생성한 후, cross-entropy 손실로 함수 근사기를 학습한다.

$$\mathcal{L} = -\sum_k \left[m_i^{(k)} \log \hat{p}_i + (1 - m_i^{(k)}) \log(1 - \hat{p}_i)\right]$$

입력 $(x, z)$, 출력 $\hat{p}_i = p(m_i | x, z)$인 신경망이 inverse_sensor_model의 역할을 대체한다. 복잡한 센서 기하(sonar의 반사 패턴, 유리에서의 LiDAR 특성)를 사람이 명시적으로 모델링하기 어려울 때 유용하다.

#### 14.7B.5 MAP Occupancy Mapping (심화)

표준 알고리즘의 셀 독립 가정은 한 가지 모순을 만든다: 같은 빔 cone 안에 있는 인접 셀들이 실제로는 서로 연관된 증거를 공유하지만, 독립 가정 때문에 이 연관성이 무시된다. 소나처럼 빔 폭이 넓은 센서에서 이 문제가 두드러진다.

MAP Occupancy Mapping은 지도 사후확률의 mode를 직접 최대화한다.

$$m^* = \arg\max_m \left[\sum_t \log p(z_t \mid x_t, m) + \log p(m)\right]$$

inverse model 대신 **forward 모델** $p(z_t | x_t, m)$을 그대로 쓴다. 모든-free 지도에서 출발해 셀 하나씩 occupancy를 뒤집으면서 log-likelihood가 증가하는 방향으로 반복하는 hill-climbing이다.

```
MAP_occupancy_grid_mapping(x_{1:t}, z_{1:t}):
  m ← 모든 셀 free로 초기화
  repeat until convergence:
    for all cells m_i do
      m_i ← argmax_{k ∈ {0,1}} [k·l_0 + Σ_t log measurement_model(z_t, x_t, m | m_i=k)]
  return m
```

실용적 한계: batch라 incremental SLAM에 맞지 않고, hill-climbing이 local maximum에 갇힌다. 후방 불확실성도 사라진다. 그러나 **"셀 독립 가정을 깨야 한다"는 통찰은 이후에 이어진다**.

#### 14.7B.6 다른 공간 표현과의 비교: OctoMap·Voxblox·NeRF·3DGS

**OctoMap**은 3D occupancy를 octree에 저장하므로 occupancy grid의 직접적인 3D 확장으로 볼 수 있다. **Voxblox**와 **nvblox**는 별도의 거리장 계열로, TSDF(Truncated Signed Distance Function)에 표면까지의 부호 있는 거리를 저장한다. **NeRF**의 density field와 **3D Gaussian Splatting**의 opacity도 ray를 따라 투명도와 색을 합성하지만, 이들은 binary occupancy Bayes filter의 직계 후예가 아니라 novel-view rendering을 학습하는 서로 다른 장면 표현이다. 따라서 forward sensor model과 ray integration이라는 수학적 공통점을 비교할 수는 있어도 계보를 동일시해서는 안 된다. Occupancy grid는 SLAM Toolbox와 Nav2 costmap 같은 navigation 구성에서 계속 쓰인다.

> **실습**: [Occupancy Grid](https://alexjunholee.github.io/robotics-practice/app.html#occupancy_grid)
> Log-odds 누산 과정을 셀별로 시각화하며, inverse_sensor_model의 hit/free 영역이 어떻게 지도로 쌓이는지 확인할 수 있다.

---

## Part 2. 최신 트렌드

### 14.8 Learning-based & Neural SLAM

전통적인 SLAM은 수작업으로 설계된 특징점, 매칭 알고리즘, 최적화 파이프라인을 사용한다. 최근에는 이 과정 일부 또는 전체를 딥러닝으로 대체하는 연구가 이어지고 있다.

**DROID-SLAM (2021)**:
- Dense Recurrent Optical-flow 기반 SLAM
- 특징점 추출/매칭 없이, Dense optical flow를 반복적으로 정제하여 카메라 포즈와 깊이를 동시에 추정
- 텍스처 없는 환경·조명 변화 등 기존 방법이 실패하는 상황에서 robustness 향상
- Differentiable한 Dense Bundle Adjustment(DBA) 레이어를 사용하여 end-to-end 학습

DROID-SLAM이 왜 주목받는지: 기존 feature-based SLAM(ORB-SLAM)은 특징점이 없는 환경에서 실패하고, direct method(DSO)는 조명 변화에 약하다. DROID-SLAM은 학습된 representation을 사용하기 때문에 이런 한계를 상당 부분 극복한다. 다만 GPU가 필수이고 실시간 성능은 아직 기존 방법에 미치지 못하는 경우가 있다.

**3DGS-SLAM 융합**:
13.5.2에서 다룬 3D Gaussian Splatting을 SLAM의 맵 표현(map representation)으로도 쓴다. SplaTAM, MonoGS 등이 대표적이며, 기존 SLAM의 sparse/dense 포인트 맵 대신 3D Gaussian으로 환경을 표현한다. 장면의 시각적 충실도가 높아지고, 렌더링 기반의 새로운 응용(가상 뷰 생성, AR 오버레이 등)이 가능해진다.

> **추천 자료**
> - [Teed & Deng, "DROID-SLAM: Deep Visual SLAM for Monocular, Stereo, and RGB-D Cameras" (2021)](https://arxiv.org/abs/2108.10869) — DROID-SLAM 논문
> - [Keetha et al., "SplaTAM" (2024)](https://arxiv.org/abs/2312.02126) — 3DGS 기반 Dense SLAM
> - [Awesome-SLAM GitHub](https://github.com/SilenceOverflow/Awesome-SLAM) — 최신 SLAM 논문/프로젝트 모음

---

## Part 3. 심화

### 14.9 심화: SLAM 백엔드 최적화

factor graph 이전 시대의 정보형 SLAM (EKF-SLAM·EIF·SEIF·EM)은 §14.16 심화: 정보형 SLAM의 역사 참조.

SLAM 프론트엔드가 센서 데이터를 처리해서 제약 조건(constraint)을 만들어내면, 백엔드는 이 제약 조건들을 동시에 만족하는 최적의 상태(포즈, 랜드마크)를 찾는다. 이 과정은 비선형 최소제곱(nonlinear least squares) 문제다. 여기서 다루는 내용은 g2o, GTSAM, Ceres 같은 라이브러리를 "왜 그렇게 설정하는지" 이해하기 위한 수학적 배경이다.

**SLAM 백엔드가 푸는 문제의 직관**

SLAM 백엔드는 결국 **Ax = b를 푸는 문제**이다.

로봇이 주행하면서 얻는 데이터는 두 종류다:
1. **오도메트리**: "나는 1m 앞으로 갔다" (상대적 이동)
2. **관측**: "저 랜드마크가 3m 거리에 보인다"

이 측정값들을 모두 만족시키는 포즈와 랜드마크 위치를 찾고 싶지만, 센서 노이즈 때문에 완벽히 만족시키는 해는 없다. 대신 "모든 측정값과의 오차 제곱합을 최소화"하는 해를 찾는다. 이것이 nonlinear least squares 문제이고, 이걸 효율적으로 푸는 것이 SLAM 백엔드의 역할이다.

비선형이라 한 번에 못 풀고, 현재 추정값 근처에서 선형화(linearize)해서 반복적으로 갱신한다. 이 "선형화 → Ax=b 풀기 → 업데이트 → 반복"이 Gauss-Newton이다.

(참고: [김기섭 블로그 — SLAM back-end 시리즈](https://gisbi-kim.github.io/blog/2021/03/04/slambackend-1.html))

#### 14.9.1 Manifold 위의 Gauss-Newton

SLAM의 상태 변수(포즈)는 SE(3) 위에 있다. SE(3)는 유클리드 공간이 아니라 Lie group이므로, 일반적인 Gauss-Newton update `x ← x + δx`를 그대로 쓸 수 없다. 회전 행렬에 벡터를 더하면 더 이상 회전 행렬이 아니게 된다.

해법은 Lie algebra se(3) 위에서 perturbation을 정의하는 것이다.

**Update step (left perturbation)**:
```
T ← exp(δξ^) · T
```
여기서 `δξ ∈ R^6`는 se(3) 위의 작은 perturbation이고, `exp(·)`는 exponential map, `^`(hat operator)는 6-벡터를 4x4 행렬로 변환한다.

**Jacobian 계산**: 오차 함수 `e(T)`의 Jacobian을 `δξ`에 대해 계산한다.
```
J = ∂e / ∂δξ
```
이것은 chain rule로 `∂e/∂T · ∂T/∂δξ`가 되는데, `∂T/∂δξ`가 바로 SE(3)의 left Jacobian이다.

**Normal equation**:
```
(J^T Σ^{-1} J) δξ* = -J^T Σ^{-1} e
```
- `Σ`는 측정 노이즈 공분산
- `H = J^T Σ^{-1} J`가 Hessian의 Gauss-Newton 근사이고, 이것이 **information matrix**
- 여러 제약 조건이 있으면 각각의 `J^T Σ^{-1} J`를 합산한다 (additive property)

이 과정을 수렴할 때까지 반복한다. 매 iteration마다 현재 추정치에서 Jacobian을 재계산하고, update를 적용한다.

#### 14.9.2 Schur Complement (Marginalization)

Bundle Adjustment(BA)에서 상태 변수는 카메라 포즈(p)와 랜드마크(l) 두 종류이다. Normal equation의 Hessian `H`는 다음 block 구조를 갖는다:

```
[H_pp  H_pl] [δp]   [b_p]
[H_lp  H_ll] [δl] = [b_l]
```

포즈 수를 `m`, 랜드마크 수를 `n`이라 하면, 보통 `n >> m`이다. 이 큰 시스템을 직접 풀면 비싸다.

**Schur complement**로 랜드마크를 marginalize한다:

```
(H_pp - H_pl · H_ll^{-1} · H_lp) δp = b_p - H_pl · H_ll^{-1} · b_l
```

**`H_ll`은 block diagonal**이므로 이것이 가능하다. 각 랜드마크는 다른 랜드마크와 직접 연결되지 않으므로(랜드마크끼리는 공통 factor가 없다), `H_ll`의 역행렬은 각 block을 독립적으로 역산하면 된다. 계산 비용이 `O(n)`으로 싸다.

Schur complement 뒤 reduced camera system의 차원은 pose 수 `m`으로 정해지지만, landmark elimination과 back-substitution 비용은 여전히 관측 수와 landmark 수 `n`에 의존한다. 이 block sparsity 덕분에 큰 BA 문제를 효율적으로 풀 수 있지만 처리 속도는 graph 구조, solver, hardware에 달려 있다.

`δp`를 구한 뒤, `δl`은 back-substitution으로 복원한다:
```
δl = H_ll^{-1} (b_l - H_lp · δp)
```

#### 14.9.3 희소성과 Variable Ordering

Pose graph optimization의 `H`는 각 factor가 일부 pose만 연결하므로 보통 **sparse**하다. Odometry factor는 인접 pose를, loop factor는 떨어진 pose를 연결한다. 노드 차수는 dataset과 closure 수에 따라 달라지며 dense한 loop proposal이나 elimination ordering은 fill-in을 늘릴 수 있다.

sparse linear system을 풀 때 Cholesky factorization(`H = L L^T`)을 사용하는데, 여기서 **fill-in** 문제가 발생한다. 원래 0이었던 위치가 factorization 과정에서 non-zero가 되는 현상이다. Fill-in이 많으면 메모리와 계산 비용이 급증한다.

Fill-in을 최소화하려면 변수의 순서(variable ordering)를 잘 정해야 한다:
- **COLAMD** (Column Approximate Minimum Degree): sparse least-squares에서 흔히 쓰이는 heuristic. 예상 fill-in이 작도록 column ordering을 근사
- **AMD** (Approximate Minimum Degree): COLAMD와 유사하지만 symmetric 행렬에 특화
- **Nested dissection**: 그래프를 재귀적으로 분할하여 ordering을 결정. 대규모 문제에서 효과적

g2o, GTSAM, Ceres 같은 라이브러리에서 solver를 설정할 때 linear solver type(DENSE_SCHUR, SPARSE_NORMAL_CHOLESKY 등)과 ordering strategy를 함께 살펴야 한다. Ordering이 fill-in과 실행 시간에 미치는 영향은 graph structure에 따라 달라지므로, 대표 dataset에서 memory와 latency를 측정해 선택한다.

```python
# Ceres Solver에서 ordering 설정 예시 (Python binding)
options = ceres.SolverOptions()
options.linear_solver_type = ceres.LinearSolverType.SPARSE_NORMAL_CHOLESKY
options.sparse_linear_algebra_library_type = ceres.SparseLinearAlgebraLibraryType.SUITE_SPARSE
# ordering은 보통 자동으로 COLAMD를 사용하지만, 수동 설정도 가능
```

#### 14.9.4 최적화 라이브러리 비교

| 라이브러리 | 특징 | 주요 사용처 |
|---|---|---|
| **g2o** | Pose graph / BA 전용, 가벼움, C++ only | ORB-SLAM2/3, LSD-SLAM |
| **GTSAM** | Factor graph 기반, Bayes tree(iSAM2) 지원, incremental 최적화에 강함 | LIO-SAM, VINS-Fusion, 연구용 |
| **Ceres Solver** | 범용 nonlinear least squares, auto-diff 지원, Google 개발 | Cartographer, 다양한 프로젝트 |

선택 기준:
- SLAM 전용이고 가볍게 쓰고 싶다 → g2o
- Factor graph 모델링이 필요하고, incremental update(키프레임이 추가될 때마다 점진적으로 최적화)가 중요하다 → GTSAM (iSAM2)
- SLAM 이외의 범용 최적화도 해야 하고, Jacobian을 직접 유도하기 싫다 → Ceres (auto-diff)

> **추천 자료**
> - Barfoot, "State Estimation for Robotics" Ch.4 (Nonlinear Estimation) — Manifold 위의 최적화를 체계적으로 설명
> - [CMU 16-833 Robot Localization and Mapping Lecture Notes](https://www.cs.cmu.edu/~kaess/pub/Dellaert17fnt.pdf) — Factor graph와 SLAM 백엔드 이론
> - [g2o Tutorial](https://github.com/RainerKuemmerle/g2o) / [GTSAM Tutorial](https://gtsam.org/tutorials/intro.html) — 라이브러리별 실습
> - [김기섭 블로그 — Gauss-Newton Opt == IEKF update?](https://gisbi-kim.github.io/blog/2022/03/05/gn-iekf-same.html) — GN 최적화와 반복 칼만 필터의 수학적 동치성 설명. 필터 vs 최적화 논쟁에 대한 정리

> **실습**: [Bundle Adjustment 시각화](https://alexjunholee.github.io/robotics-practice/app.html#bundle_adjustment)
> 카메라 포즈와 3D 포인트를 동시에 최적화하는 Bundle Adjustment 과정을 인터랙티브하게 확인할 수 있다.

### 14.10 심화: IMU Preintegration

IEKF·MSCKF의 EKF 기반 구조는 §3.10.2 EKF (Ch.3 참조).

14.3.3에서 VINS-Mono를 소개할 때 IMU preintegration을 간단히 언급했다. 여기서는 그 수학적 배경을 본다.

**문제 정의**: IMU는 보통 200~1000 Hz로 가속도와 각속도를 출력한다. 반면 SLAM 최적화는 키프레임 단위(수 Hz~수십 Hz)로 수행한다. 키프레임 사이에 수백 개의 IMU 측정이 존재하는데, 최적화할 때 이것을 전부 상태 변수로 넣으면 문제 크기가 폭발한다.

**Preintegration의 아이디어**: 두 키프레임 `i`와 `j` 사이의 IMU 측정값들을 하나의 "상대 운동 측정(relative motion measurement)"으로 압축한다. 이 압축된 측정값이 최적화의 factor로 들어간다.

**Preintegrated measurements**: 키프레임 `i`에서 `j`까지의 상대 변화량 세 가지를 계산한다.

```
ΔR_ij = Π_{k=i}^{j-1} Exp((ω_k - b_g) · Δt)          # 상대 회전
Δv_ij = Σ_{k=i}^{j-1} ΔR_ik · (a_k - b_a) · Δt        # 상대 속도
Δp_ij = Σ_{k=i}^{j-1} (Δv_ik · Δt + 0.5 · ΔR_ik · (a_k - b_a) · Δt^2)  # 상대 위치
```

여기서 `ω_k`, `a_k`는 IMU 측정값, `b_g`, `b_a`는 gyroscope/accelerometer bias, `Δt`는 IMU 샘플링 간격이다.

이 preintegrated measurement들은 **키프레임 `i`의 좌표계를 기준으로** 계산된다. 따라서 키프레임 `i`의 절대 포즈가 최적화 과정에서 바뀌더라도, preintegrated measurement를 재계산할 필요가 없다.

**공분산 전파**: IMU 측정 노이즈가 preintegrated measurement에 어떻게 전파되는지 계산한다. Discrete-time propagation으로 각 IMU 측정마다 공분산을 업데이트한다.

```
Σ_{k+1} = A_k · Σ_k · A_k^T + B_k · Q · B_k^T
```
- `A_k`: 상태 전이 행렬 (현재 상태에서의 Jacobian)
- `B_k`: 노이즈 입력 행렬
- `Q`: IMU 노이즈 공분산 (데이터시트에서 확인)

이 공분산이 최적화에서 해당 factor의 information matrix(`Σ^{-1}`)로 사용된다.

**Bias 변화 시 보정**: 최적화 과정에서 IMU bias 추정치가 바뀔 수 있다. Bias가 바뀌면 원칙적으로 preintegration을 처음부터 다시 해야 한다. 하지만 이것은 비싸다. 대신 **first-order approximation**으로 보정한다:

```
ΔR_ij ≈ ΔR_ij^0 · Exp(∂ΔR/∂b_g · δb_g)
Δv_ij ≈ Δv_ij^0 + ∂Δv/∂b_g · δb_g + ∂Δv/∂b_a · δb_a
Δp_ij ≈ Δp_ij^0 + ∂Δp/∂b_g · δb_g + ∂Δp/∂b_a · δb_a
```

`^0`은 이전 bias 추정치로 계산한 값, `δb`는 bias 변화량, 편미분들은 preintegration 과정에서 함께 축적해둔다. Bias 변화가 크지 않은 한(보통 그렇다) 이 근사는 충분히 정확하다.

**왜 manifold에서 preintegration하는가**: 이전 방식은 IMU 측정을 가장 가까운 키프레임 timestamp에 interpolation해서 사용했다. 그런데 회전은 SO(3)에 있으므로 단순 선형 보간이 정확하지 않다. Lie group 위에서 미리 integration해두면, 1) 회전 누적이 수학적으로 정확하고, 2) 결과가 relative motion measurement로서 factor graph에 바로 들어갈 수 있다. Forster et al. (2015 RSS, 2017 TRO)의 핵심 기여가 이 부분이다.

**Tightly-coupled vs Loosely-coupled**: LIO-SAM을 예로 설명하면:
- **Loosely-coupled**: IMU를 다음 포즈의 초기값(initial guess)으로만 사용한다. LiDAR odometry와 IMU가 각자 독립적으로 상태를 추정하고, 나중에 covariance 기반으로 합친다. LeGO-LOAM이 이 방식이다.
- **Tightly-coupled**: IMU preintegration factor를 LiDAR odometry factor와 같은 factor graph 안에서 동시에 최적화한다. IMU가 단순 초기값이 아니라, 키프레임 사이의 상대 포즈에 대한 독립적 관측으로 작용한다. LIO-SAM이 이 방식이다.

Tightly-coupled의 장점은 모션이 심한 상황(빠른 회전, 급격한 가감속)에서 드러난다. LiDAR scan matching만으로는 잡기 어려운 빠른 변화를 IMU factor가 잡아주기 때문이다. Factor graph 형식이므로 GPS factor, loop closure factor 등을 모듈처럼 추가할 수 있다는 것도 실용적 장점이다.

**LIO-SAM의 구조**: GTSAM 기반으로, IMU preintegration factor + LiDAR odometry factor + GPS factor + loop closure factor를 하나의 그래프에서 최적화한다. LiDAR odometry는 edge feature와 planar feature를 따로 추출하고, 각각 다른 resolution의 voxel map으로 관리한다. Scan matching 시 planar feature는 point-to-plane distance, edge feature는 point-to-line distance를 최소화하는 상대 변환을 구한다.

VINS-Mono, ORB-SLAM3 (Visual-Inertial mode), LIO-SAM 등 현대 VIO/LIO 시스템이 이 기법을 IMU factor 구현에 그대로 쓴다.

> **추천 자료**
> - [Forster et al., "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry" (TRO 2017, arXiv:1512.02363)](https://arxiv.org/abs/1512.02363) — Preintegration의 원본 논문. 수식이 많지만 이 분야의 필수 논문
> - [Forster et al., "IMU Preintegration on Manifold for Efficient VIO" (2015 RSS)](https://rpg.ifi.uzh.ch/docs/RSS15_Forster.pdf) — 위 논문의 초기 버전으로, 방법을 더 간결하게 설명한다.
> - [Shan et al., "LIO-SAM" (IROS 2020)](https://github.com/TixiaoShan/LIO-SAM) — Tightly-coupled LIO의 레퍼런스 구현. 코드와 논문 모두 읽을 것
> - [Sola et al., "A micro Lie theory for state estimation in robotics" (arXiv:1812.01537)](https://arxiv.org/abs/1812.01537) — Lie group/algebra의 실용적 정리. Preintegration 읽기 전에 이것부터 보면 좋다
> - GTSAM의 `PreintegratedImuMeasurements` 클래스 소스코드 — 이론이 코드로 어떻게 구현되는지 확인
> - [IMU Preintegration MATLAB 구현](https://github.com/GentleDell/imu_preintegration_matlab) — KITTI에서 테스트한 MATLAB 코드. 수식과 코드를 대조하며 공부하기 좋다

### 14.11 심화: 관측가능성 분석 (Observability)

SLAM/VIO 시스템을 돌려보면 "왜 이 상황에서 drift가 심한가?", "왜 가만히 서있으면 위치가 흔들리는가?" 같은 현상을 겪게 된다. 이런 현상의 상당수는 시스템의 **관측가능성(observability)** 한계에서 비롯된다.

**Visual-Inertial 시스템의 관측 불가능한 상태**: VIO에서 추정할 수 없는(unobservable) 자유도가 4개 있다:

1. **Global position (3 DoF)** — 절대 위치를 알 수 없다. GPS 같은 절대 기준이 없으면 시작점을 원점으로 잡을 수밖에 없다
2. **Global yaw (1 DoF)** — 중력 방향 축 기준의 회전(heading). 나침반 없이는 "어느 쪽이 북쪽인지" 알 수 없다

반면 다음은 관측 가능하다:
- **Roll/pitch**: IMU accelerometer가 중력 방향을 감지하므로, 중력 기준 roll/pitch는 추정 가능
- **Scale** (stereo/IMU가 있는 경우): stereo 카메라의 baseline이나 IMU의 가속도 측정으로 스케일 복원 가능. 단, **monocular 카메라만으로는 scale이 관측 불가능**하다

**Degenerate motion** — 특정 움직임 패턴에서 추가적인 상태가 관측 불가능해진다:

- **순수 회전(pure rotation)**: Monocular VO에서 translation을 추정할 수 없다. Epipolar geometry에서 epipole이 무한원점으로 가기 때문이다. 실무에서 "카메라를 제자리에서 돌리면 트래킹이 깨진다"는 현상의 원인
- **등속 직진(constant velocity)**: IMU의 accelerometer가 중력과 가속도를 구분하는데, 가속이 없으면(등속이면) accelerometer bias와 중력 방향의 미세한 오차를 구분할 수 없다. IMU bias가 관측 불가능해진다
- **정지(stationary)**: 등속의 특수 케이스. 가만히 서있으면 visual feature의 parallax도 없고 IMU 가속도도 없어서, bias와 스케일 모두 관측 불가능. "왜 가만히 서있으면 VINS가 drift 하는가?"의 답

**EKF 기반 시스템에서의 문제**: 표준 EKF를 VIO에 적용하면, linearization 오차로 인해 이론적으로 관측 불가능한 방향에서도 공분산이 줄어드는(uncertainty가 인위적으로 감소하는) 현상이 발생한다. 이것은 inconsistency의 주요 원인이다.

**OC-EKF (Observability-Constrained EKF)**: 이 문제를 해결하기 위해, EKF의 Jacobian을 수정하여 관측 불가능한 방향의 null space를 보존한다. 추정기가 "모르는 것은 모른다고 유지"하도록 강제하는 것이다.

실무적 함의:
- VIO 시스템을 사용할 때는 **다양한 방향으로 움직이면서** 초기화해야 한다. 한 방향으로만 걸으면 IMU bias 추정이 제대로 되지 않는다
- Loop closure가 없는 VIO는 장시간 운용 시 반드시 drift 한다. 관측 불가능한 yaw 방향의 오차가 축적되기 때문
- Monocular VIO의 스케일은 가속/감속이 있어야 관측 가능. 일정 속도로만 움직이면 스케일 drift가 생긴다

> **추천 자료**
> - [Hesch et al., "Observability-constrained Vision-aided Inertial Navigation" (TRO 2014)](https://ieeexplore.ieee.org/document/6672119) — OC-EKF/OC-VINS의 원본 논문
> - Barfoot, "State Estimation for Robotics" Ch.9 — 관측가능성 분석의 이론적 토대
> - [Huang & Dissanayake, "A critique of current developments in Simultaneous Localization and Mapping" (IJRR 2016)](https://journals.sagepub.com/doi/10.1177/0278364916643566) — SLAM의 관측가능성/일관성 문제를 비판적으로 정리

> **실습**: [Odometry Uncertainty 시각화](https://alexjunholee.github.io/robotics-practice/app.html#odom_uncertainty)
> Odometry의 불확실성이 시간에 따라 어떻게 누적되는지, 공분산 타원이 어떻게 커지는지 인터랙티브하게 확인할 수 있다.

#### 14.11.1 필터 기반 vs 최적화 기반: 뭐가 더 나은가?

필터와 최적화 중 어느 쪽이 나은지는 SLAM/VIO 분야의 오래된 논쟁이다. 수학적으로 Gauss-Newton 최적화와 Iterated EKF (IEKF)는 같은 문제를 서로 다른 형태로 푼다.

- **필터 (EKF, MSCKF 등)**: 새 측정이 들어올 때마다 상태와 covariance를 갱신한다. 단순 EKF odometry는 현재 state만 둘 수 있지만, MSCKF는 일정 수의 과거 camera clone을 state에 유지한다. 보관 범위를 제한하므로 full smoothing보다 memory를 작게 만들 수 있다.
- **최적화 (BA, factor graph)**: 과거 상태를 전부 유지하고 한꺼번에 최적화한다. 과거 데이터를 relinearize할 수 있으므로 정확도가 높다. 하지만 상태 수가 늘어나면 계산량이 커진다 (sliding window나 iSAM2로 완화).

VINS-Mono(최적화)와 MSCKF(필터)의 성능 차이는 solver보다 **시스템 구조**에서 온다. 어떤 상태를 유지하고 어떤 측정을 사용하는지가 다르기 때문이다. 최적화 기반 시스템은 relinearization으로 과거의 linearization error를 줄일 수 있다는 이점이 있다.

실무적 선택:
- IMU 중심 + 경량 → 필터 (MSCKF, FAST-LIO2의 IEKF)
- 카메라 중심 + 정확도 → 최적화 (VINS-Mono, ORB-SLAM3)
- 둘 다 필요 → 하이브리드 (LIO-SAM: IMU preintegration을 factor로 넣은 최적화)

(참고: [김기섭 블로그 — Gauss-Newton Opt == IEKF update?](https://gisbi-kim.github.io/blog/2022/03/05/gn-iekf-same.html))

### 14.12 심화: Semantic SLAM

기존 SLAM은 순수하게 기하학적(geometric)인 지도를 만든다. 포인트 클라우드, 메쉬, occupancy grid 등 "공간의 형태"만 기록한다. 벽이 있다는 건 알지만 그것이 벽인지, 문인지, 책장인지는 모른다. Semantic SLAM은 지도에 의미(semantic) 정보를 결합한다.

접근 방식은 랜드마크 표현 방식에 따라 나뉜다. **Object-level SLAM** (CubeSLAM, QuadricSLAM)은 점(point) 대신 3D cuboid·dual quadric 같은 물체 단위를 랜드마크로 추정한다. 물체 검출기에 의존하지만, data association이 점 기반보다 견고하고 물체 수준의 추론이 가능하다. **Panoptic SLAM**은 panoptic segmentation 결과를 3D로 융합하여 모든 픽셀에 semantic label이 붙은 지도를 만든다. 로봇이 "이 방에 의자가 3개"를 지도에서 바로 쿼리할 수 있다. **Open-vocabulary SLAM** (ConceptGraphs)은 CLIP 같은 vision-language model의 feature를 지도에 저장하여 자연어로 장소를 검색할 수 있게 한다. 13장에서 다룬 3D Scene Graph (Hydra 등)와 직접 연결되는 주제다.

**동적 물체 처리**: Semantic label은 동적 환경에서 SLAM의 robustness를 높이는 데도 쓰인다. "사람", "차" 등 동적일 가능성이 높은 클래스의 feature를 tracking/mapping에서 빼면, 정적 환경만으로 깨끗한 SLAM이 가능하다.
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

한 대의 로봇이 넓은 환경을 탐색하려면 시간이 오래 걸린다. 여러 로봇이 동시에 나눠서 탐색하면 시간을 줄일 수 있지만, 각 로봇이 만든 부분 지도(submap)를 하나의 일관된 글로벌 지도로 합치는 것은 단순하지 않다.

**Centralized 접근**은 여러 로봇의 센서 데이터나 local map을 중앙 server로 보내 공동 최적화를 수행한다. global information에 접근하기 쉬운 대신 통신량과 server 계산량이 커질 수 있고, server가 단일 장애점이 된다. 비볼록 SLAM에서 중앙화 자체가 전역 최적해를 보장하지는 않는다.

**Distributed 접근**은 각 로봇이 local SLAM을 수행하고 rendezvous나 inter-robot loop closure에서 relative pose constraint를 만든다. 원본 data 대신 descriptor나 submap을 교환하면 통신량을 줄일 수 있지만 검증에 필요한 정보도 줄어든다. 각 robot이 어떤 변수와 factor를 보유·교환하는지, 그리고 asynchronous optimization이 어떤 조건에서 수렴하는지는 algorithm마다 다르다.

분산 시스템에서는 **inter-robot loop closure**, **좌표계 정렬**, **outlier rejection**을 함께 다뤄야 한다. Relative SE(3)는 하나의 검증된 6-DoF pose constraint로 주어질 수도 있고, 3D point correspondence로 구한다면 퇴화하지 않는 최소 세 점과 충분한 inlier가 필요하다. PCM(Pairwise Consistency Maximization), GNC(Graduated Non-Convexity), distributed Gauss-Seidel, ADMM 등은 각기 다른 가정과 통신 비용을 가지므로 system 조건에 맞춰 선택한다.

**대표 시스템**:
| 시스템 | 특징 |
|---|---|
| **Kimera-Multi** | Distributed, 3D mesh + semantic, Kimera 기반 |
| **DOOR-SLAM** | Distributed, outlier-robust, DGS 최적화 |
| **Swarm-SLAM** | ROS2 기반, 다양한 센서 지원, 경량 |

> **추천 자료**
> - [Lajoie et al., "DOOR-SLAM: Distributed, Online, and Outlier Resilient SLAM for Robotic Teams" (RA-L 2020)](https://arxiv.org/abs/1909.12198) — Distributed SLAM + robust optimization
> - [Tian et al., "Kimera-Multi: Robust, Distributed, Dense Metric-Semantic SLAM" (ICRA 2022)](https://arxiv.org/abs/2106.14386) — Multi-robot semantic SLAM
> - [Cieslewski et al., "Data-Efficient Decentralized Visual SLAM" (ICRA 2018)](https://arxiv.org/abs/1710.05772) — 통신 효율적인 분산 SLAM의 초기 연구

### 14.14 심화: Place Recognition

Loop closure의 핵심 문제는 "지금 보는 장면을 이전에 본 적 있는가?"이다. 이 질문은 이미지 검색(image retrieval) 문제다. 현재 프레임의 descriptor를 과거 모든 키프레임의 descriptor와 비교해서 가장 유사한 것을 찾는다. SLAM의 정확도는 loop closure에, loop closure는 place recognition에 달려 있다.

**전통적 방법: Bag of Visual Words (BoVW)**

DBoW2 라이브러리가 대표적이며 ORB-SLAM2/3에서 사용된다.
1. 대규모 이미지에서 local feature(ORB 등)를 추출
2. k-means clustering으로 visual vocabulary(단어 사전) 구축
3. 각 이미지를 "어떤 visual word가 몇 번 나타났는지"의 histogram(BoW vector)으로 표현
4. BoW vector 간의 유사도(L1-score 등)로 이미지 비교

장점: 빠르다 (inverted index 사용), 검증된 방법. 단점: 시점/조명 변화에 취약, vocabulary 학습이 필요.

**학습 기반 방법: Global Descriptor**

이미지 전체를 하나의 compact vector로 압축하는 방식이다. **NetVLAD** (2016)는 CNN feature와 VLAD aggregation을 결합했고, 논문의 도시 규모 benchmark에서 비교 방법보다 높은 recall을 보고했다. **CosPlace** (2022)와 **MixVPR** (2023)은 각 논문의 dataset·protocol에서 descriptor 학습과 aggregation의 개선을 평가했다. **AnyLoc** (2023)은 DINOv2 feature를 활용해 여러 indoor·outdoor·aerial dataset에서 별도 place-recognition fine-tuning 없는 결과를 보고했다. 이 결과는 모든 환경에서 BoVW보다 robust하다는 보장이 아니므로 target domain에서 recall과 false positive를 다시 측정해야 한다.

**LiDAR 기반 Place Recognition**:

시각 정보 없이 3D 구조만으로 장소를 인식하므로 영상 밝기 변화에 직접 의존하지 않는다. 다만 날씨와 동적 물체가 point return을 바꿀 수 있고, 구조적으로 유사한 환경(예: 긴 복도)에서는 혼동될 수 있다.

- **Scan Context** (IROS 2018): 3D 포인트 클라우드를 bird-eye view로 투영한 뒤, 거리/높이 기반의 2D descriptor 생성. Rotation-invariant한 매칭 가능
- **OverlapTransformer** (2022): Transformer 기반으로 LiDAR range image에서 global descriptor를 학습

**Cross-modal Place Recognition**: 카메라 이미지로 query하고 LiDAR 지도에서 검색하거나, 그 반대. 센서가 다른 로봇 간의 multi-robot SLAM에서 중요하다.

**Sequence Matching**: 단일 이미지 매칭의 한계를 극복하기 위해, 연속된 프레임의 시퀀스를 함께 매칭한다.
- **SeqSLAM** (2012): 이미지 개별 유사도는 낮아도, 시퀀스 패턴이 일치하면 같은 장소로 판단. 극적인 외관 변화(주간 vs 야간)에서도 동작
- 최근 방법: sequence descriptor를 학습하여 더 효율적으로 시퀀스 매칭

실무 팁: DBoW2는 ORB-SLAM 계열에서 쓰이는 공개 baseline이다. 조명·계절 변화가 크다면 NetVLAD 이후의 학습 기반 descriptor도 같은 데이터에서 비교하라. AnyLoc은 별도 fine-tuning 없이 feature를 구성하지만 backbone 연산량, descriptor memory, target-domain recall을 확인해야 한다.

§14.16.5의 cycle posterior도 place 일치 신호 뒤 과거 trajectory를 보정한다는 기능적 구조를 갖는다. 현대 loop closure가 그 알고리즘에서 직접 유래했다고 단정하지는 않는다.

> **추천 자료**
> - [Arandjelovic et al., "NetVLAD: CNN architecture for weakly supervised place recognition" (arXiv:1511.07247)](https://arxiv.org/abs/1511.07247) — 학습 기반 place recognition의 시작점
> - [Keetha et al., "AnyLoc: Towards Universal Visual Place Recognition" (arXiv:2308.00688)](https://arxiv.org/abs/2308.00688) — Foundation model 기반 zero-shot place recognition
> - [Kim & Kim, "Scan Context: Egocentric Spatial Descriptor for Place Recognition within 3D Point Cloud Map" (IROS 2018)](https://ieeexplore.ieee.org/document/8593953) — LiDAR place recognition의 대표 방법
> - [김기섭 블로그 — Scan Context-based LiDAR Pose-graph SLAM 구현](https://gisbi-kim.github.io/blog/2021/05/17/sclidarslam.html) — Scan Context를 LiDAR SLAM에 통합한 구현 해설
> - [다크 프로그래머 — Bag of Words 기법](https://darkpgmr.tistory.com/125) — BoW의 원리를 이미지 검색과 연결하여 설명

> **기술 흐름: SLAM & Odometry**
> - **~2007**: 고전기. EKF-SLAM, FastSLAM(파티클 필터 기반)이 주류. MonoSLAM(2007)이 실시간 단안 SLAM의 시작을 알림. PTAM(2007)이 Tracking/Mapping 분리 아키텍처를 제안
> - **2010~2015**: LSD-SLAM, SVO 등 direct method 등장. LOAM(2014)이 LiDAR odometry와 mapping의 영향력 있는 구조를 제시했고, ORB-SLAM(2015)이 공개 feature-based visual SLAM baseline을 제공
> - **2015~2020**: VINS-Mono, MSCKF 계열 등 VIO system의 공개 구현과 적용 확대. DSO가 direct sparse 방식을 제시했고, LIO-SAM 등 LiDAR-inertial system 등장
> - **2020~2023**: FAST-LIO/FAST-LIO2, ORB-SLAM3, DROID-SLAM, R3LIVE 등 filter·optimization·learning을 서로 다르게 조합한 공개 system 등장
> - **2024~**: 3DGS 기반 SLAM(SplaTAM, MonoGS, Gaussian-SLAM)이 Neural SLAM의 방향을 바꾸고 있다. Foundation Model과 SLAM의 결합(예: 자연어로 장소를 설명하여 위치를 찾는 등) 연구도 시작
> - **최근 흐름**: geometric SLAM은 ORB-SLAM3, LIO-SAM, FAST-LIO2처럼 공개 구현과 벤치마크가 축적된 방법을 중심으로 발전해 왔다. 한편 3DGS-SLAM과 learning-based 방법은 장면 표현과 front-end의 선택지를 넓히고 있다. KITTI, EuRoC, TUM RGB-D에서 두 계열의 입력 조건과 실패 사례를 비교할 수 있다.

### 14.15 심화: Long-term Mapping

실제 환경에서 로봇을 운용하면 "지도를 한 번 만들고 끝"이 아니다. 같은 장소를 여러 번 방문하면서 지도를 업데이트하고, 동적 물체(사람, 차량)를 제거하고, 여러 세션의 데이터를 통합해야 한다. 이것이 long-term mapping이고, 실용적인 로봇 시스템에서 피할 수 없는 문제다.

#### 14.15.1 Incremental Smoothing: iSAM에서 iSAM2까지

Filter-based SLAM(EKF 등)은 state 수가 늘어나면 Jacobian 행렬이 커져서 실시간 처리가 어렵다. iSAM(Kaess et al., TRO 2008)은 QR factorization의 R matrix를 Givens rotation으로 incremental하게 업데이트할 수 있음을 보여줬다. 새로운 measurement가 추가될 때 변경된 부분만 갱신하면 된다. 다만 non-zero element가 누적되면 주기적으로 re-ordering이 필요하다.

iSAM2(Kaess et al., IJRR 2012)는 Bayes tree 구조를 도입하여 이 한계를 극복했다. 영향받는 subtree만 re-elimination하므로, 대규모 문제에서도 일관된 성능을 보인다. GTSAM의 핵심 엔진이 바로 iSAM2다.

#### 14.15.2 Dynamic Object Removal

지도에서 동적 물체를 제거하는 것은 long-term mapping의 필수 과제다.

**Removert** (Kim et al., 2020): multi-resolution range image를 이용해 static/dynamic을 분류한다. Point cloud를 range image로 투영한 뒤, 다른 시점에서 관측한 range와 비교하여 동적 여부를 판단한다. 먼저 static point를 보수적으로 확보하고, 잘못 제거한 point를 복원하는 two-stage 방식을 사용한다. 여러 confidence level을 두어 이 두 단계의 trade-off를 조절할 수 있다.

기존 접근과 비교하면: voxel ray-casting은 정확하지만 연산이 비싸고, visibility-based는 뒤의 static point를 보존한다는 가정이 필요하며, segmentation-based는 unknown label에 약하고 scan-to-map 관계를 무시한다. Removert는 이 세 방법의 단점을 multi-resolution range image 비교로 보완한다.

**SuMa++** (Chen et al., IROS 2019): surfel-based mapping에 semantic label을 추가한 시스템이다. LiDAR point에 normal과 semantic 정보를 더하고, semantic과 motion 양쪽에서 dynamic으로 판정된 surfel만 제거한다. Motion-degenerate 환경에서는 움직이는 point도 geometric constraint로 유용할 수 있기 때문이다.

#### 14.15.3 Multi-Session SLAM

같은 환경을 여러 날에 걸쳐 매핑하면, 각 세션의 trajectory를 하나로 합쳐야 한다. 문제는 gauge freedom — 각 세션의 좌표계가 다르므로 단순히 합치면 안 맞는다.

**LT-mapper** (Kim et al., 2021): Scan Context 기반 anchor node로 multi-session을 정렬하고, positive/negative change detection으로 지도를 업데이트한다. 변화 정도에 따라 high-dynamic, low-dynamic, weak non-dynamic, strong positive-dynamic을 구분하여 delta map을 관리한다.

**Continuous-Time Estimation** (Furgale et al., ICRA 2012): discrete-time 대신 B-spline basis function으로 trajectory를 표현하면, 다른 Hz의 센서들을 더 적은 변수로 통합할 수 있다. 빠른 센서(IMU)와 느린 센서(LiDAR, 카메라) 사이의 self-calibration에도 활용 가능하다.

> **추천 자료**
> - [Kaess et al., "iSAM2: Incremental Smoothing and Mapping Using the Bayes Tree" (IJRR 2012)](https://www.cs.cmu.edu/~kaess/pub/Kaess12ijrr.pdf) — Bayes tree 기반 incremental SLAM의 원본 논문
> - [Kim et al., "Remove, then Revert: Static Point Cloud Map Construction using Multiresolution Range Images" (IROS 2020)](https://github.com/irapkaist/removert) — Dynamic point 제거의 실용적 방법. 코드 공개
> - [Kim et al., "LT-mapper: A Modular Framework for LiDAR-based Lifelong Mapping" (ICRA 2022)](https://github.com/gisbi-kim/lt-mapper) — Multi-session SLAM 프레임워크
> - [Chen et al., "SuMa++: Efficient LiDAR-based Semantic SLAM" (IROS 2019)](https://github.com/PRBonn/semantic_suma) — Semantic 정보를 활용한 LiDAR SLAM

---

### 14.16 심화: 정보형 SLAM의 역사

*§14.9 factor graph 최적화(양방향 참조).*

2005년 *Probabilistic Robotics*가 출간될 무렵에는 SLAM의 정보 표현을 두고 여러 접근이 경쟁했다. EKF-SLAM은 결합 상태와 공분산을 사용했고, EIF/SEIF는 정보형의 가산성을 활용했으며, EM mapping은 미지 data association을 통계적으로 다뤘다. 2010년대의 factor graph와 GTSAM/iSAM2는 이 계보에서 정보형의 가산성, 변수 소거, 증분 갱신을 이어받아 현대적인 최적화 구조로 정리했다.

#### 14.16.1 EKF-SLAM (PR §10)

**Smith, Self, Cheeseman (1986/1990)** "Estimating Uncertain Spatial Relationships in Robotics"에서 시작된 계보다. 이들이 제안한 'stochastic map' — 로봇 포즈와 랜드마크를 하나의 확률 변수로 묶는다는 발상 — 이 EKF-SLAM의 원형이다. 1990년대 Leonard·Durrant-Whyte, 그리고 Dissanayake et al. (2001, IEEE T-RA)이 정식화를 완성했다.

**알고리즘 골격**: 포즈 $x_t = (x, y, \theta)$와 N개 랜드마크 $(m_{j,x}, m_{j,y}, s_j)$를 $(3N+3)$차원 상태 벡터 $y_t$로 묶고 EKF를 굴린다.

```
EKF_SLAM_known_correspondences(μ_{t-1}, Σ_{t-1}, u_t, z_t, c_t):
  // Motion: F_x로 3D motion을 (3N+3)D로 lift
  // F_x = [I_3 | 0_{3×3N}] — (3×(3N+3)) 프로젝션, F_x^T 는 (3N+3)×3
  // G_t = I_{3N+3} + F_x^T G_t^{pose} F_x — (3N+3)×(3N+3), G_t^{pose}는 포즈 3×3 Jacobian
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

**Kalman gain $K_t^i$는 $(3N+3) \times 3$ 행렬**이다 — 단일 랜드마크 관측이 전체 상태를 갱신한다. 한 관측이 공분산 off-diagonal을 통해 다른 랜드마크 추정을 개선하지만, 업데이트 비용이 $O(N^2)$로 늘어난다는 대가가 따른다.

미지 대응(unknown correspondence)이면 가상의 $(N_t+1)$번째 랜드마크를 맵 끝에 잠시 추가하고, 모든 후보에 대해 마할라노비스 거리를 계산하여 ML 대응을 고른다. 임계값 $\alpha$ 초과면 신규 랜드마크로 등록한다. 이 greedy ML 결정이 한번 틀리면 회복할 수 없다는 것이 ML data association의 근본 약점이다.

EKF-SLAM의 한계는 세 방향에서 드러났다. 공분산 행렬 $\Sigma \in \mathbb{R}^{(3N+3) \times (3N+3)}$의 메모리는 $N^2$에 비례한다 — 100개 랜드마크면 303×303, 1000개면 3003×3003. 랜드마크가 추가될수록 과거의 선형화 오차가 쌓여 추정이 inconsistent해진다 (Bailey et al. 2006). 과거 포즈를 marginalize한 상태를 유지하므로 과거 포즈는 다시 볼 수 없어 full posterior 최적화가 불가능하다.

대규모 landmark map에서는 dense covariance와 누적 선형화 오차 때문에 고전적 full-state EKF-SLAM보다 smoothing·factor-graph 방식이 흔하다. 그렇다고 EKF-SLAM이 특정 landmark 수 아래에서만 유효하다는 보편 경계가 있는 것은 아니다. 계산 예산, 관측 sparsity, consistency 요구에 따라 소규모 landmark system이나 fiducial mapping에서 사용할 수 있다. MSCKF류 sliding-window filter는 EKF 선형화와 covariance propagation을 쓰지만 landmark를 상태에 영구 보관하지 않는 별도 정식화다. JCBB는 ML data association의 대안이다. GTSAM/iSAM2는 같은 SLAM 문제를 smoothing과 factor graph로 푸는 후속 세대이지만 EKF-SLAM의 직계 알고리즘 후예는 아니다.

#### 14.16.2 EIF SLAM / GraphSLAM (PR §11)

EKF-SLAM에서 $\Sigma$는 측정마다 dense한 상관관계를 갱신한다. 정보형 $\Omega = \Sigma^{-1}$에서는 독립 factor의 기여를 국소적으로 더할 수 있어 SLAM graph의 sparsity를 드러내기 쉽다. 다만 motion update와 marginalization은 fill-in을 만들 수 있으므로 정보 행렬이 자동으로 계속 sparse한 것은 아니다. 이것이 EIF와 sparse graph formulation을 살펴보는 동기다.

##### 정보형의 직관: 스프링-매스 비유

EIF SLAM의 핵심 발상은 **정보는 가산량이다**. 공분산 $\Sigma$ 대신 정보 행렬 $\Omega = \Sigma^{-1}$과 정보 벡터 $\xi = \Omega \mu$를 사용한다.

이것을 스프링-매스 시스템으로 보면: 각 변수(포즈, 랜드마크)는 노드, $\Omega$의 비대각 원소는 두 노드를 잇는 스프링이다.
- Control $u_t$: $x_{t-1}$과 $x_t$ 사이 스프링. Stiffness = $R_t^{-1}$ (motion noise가 작을수록 강한 결속).
- Measurement $z_t^i$: 포즈 $x_t$와 랜드마크 $m_j$ 사이 스프링. Stiffness = $Q_t^{-1}$.
- 두 다른 랜드마크 사이 직접 스프링은 없다 — 랜드마크끼리 직접 측정한 적이 없으니.

**정보형 갱신식**:
$$\Omega \leftarrow \Omega + H_t^{iT} Q_t^{-1} H_t^i, \qquad \xi \leftarrow \xi + H_t^{iT} Q_t^{-1}[z_t^i - h(\mu_t) + H_t^i \mu_t]$$

Measurement information은 **국소 덧셈**으로 추가할 수 있다. 다만 motion update와 marginalization에는 elimination과 fill-in이 생길 수 있으므로 모든 연산이 국소적인 것은 아니다. Factor graph도 각 factor가 일부 변수만 연결한다는 유사한 spring 직관을 쓸 수 있지만, factor graph의 역사나 정식화가 SEIF에서 시작한 것은 아니다.

##### 4단계 파이프라인

EIF SLAM (= GraphSLAM)은 full posterior $p(x_{0:t}, m | z_{1:t}, u_{1:t})$를 배치(offline)로 푼다.

```
EIF_SLAM_known_correspondence(u_{1:t}, z_{1:t}, c_{1:t}):
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

$\tau(j)$는 랜드마크 $j$를 본 모든 포즈 시점이다. Reduce 단계에서는 *각 랜드마크에 인접한 포즈끼리 새 스프링을 만들고 랜드마크 노드를 떼어낸다*. 이는 Bundle Adjustment의 **block diagonal Schur complement** 트릭과 수학적으로 동일하다 (§14.9.2 참조).

**Marginalization Lemma**: 선형 Gaussian 정보형의 marginal은 Schur complement로 표현된다. 이 연산은 남은 변수 사이에 fill-in을 만들 수 있다.
$$\bar\Omega_{xx} = \Omega_{xx} - \Omega_{xy} \Omega_{yy}^{-1} \Omega_{yx}$$

Thrun & Montemerlo (2006, IJRR)의 GraphSLAM은 전체 trajectory와 map posterior를 sparse information graph로 batch 최적화한다. EIF와 정보형 가산성을 공유하지만 online filtering과 batch smoothing은 구분해야 한다. Lu & Milios (1997)는 pose relation을 전역 최적화하는 앞선 연구다.

GraphSLAM의 한 unknown-correspondence 절차는 feature 쌍 $(m_j, m_k)$의 동일성 후보를 평가하고 선택한 제약을 graph에 추가한 뒤 다시 최적화한다. Factor를 명시적으로 보관하면 선택한 제약을 제거하고 재최적화할 수 있다. Switchable constraints (Sünderhauf & Protzel 2012)는 loop factor의 활성도를 연속 변수로 함께 최적화하는 별도 robust formulation이며, 단순한 계보 관계로 동일시해서는 안 된다.

Initialize → factor 구성 → variable elimination → solve라는 단계는 현대 batch SLAM과 비교할 수 있다. iSAM/iSAM2는 새 factor가 들어올 때 linearization과 elimination의 영향을 받는 부분을 갱신하는 incremental smoothing이고, Bayes tree는 그 factorization을 관리하는 자료구조다. GraphSLAM이 단순히 이름만 바뀌어 진화한 것으로 보아서는 안 된다.

#### 14.16.3 SEIF — Sparse Extended Information Filter (PR §12)

앞 절의 GraphSLAM은 batch smoothing formulation이다. 일반적인 EIF는 online filter로도 쓸 수 있지만, 정확한 motion update가 information matrix를 dense하게 만들 수 있다. **SEIF**는 active feature 수를 제한하고 sparsification 근사를 도입해 map-size-independent update를 목표로 한 online filter다. Thrun et al. (2004, IJRR)은 Victoria Park 3.5 km 실험의 해당 구현에서 EKF-SLAM 대비 약 절반의 시간과 4분의 1의 메모리로 유사한 오차를 보고했다.

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

SEIF의 핵심 메커니즘이다. 변수 $a, b$ 사이의 직접 의존성을 두 marginal의 곱으로 근사하여 $\Omega$에서 0 원소를 만든다.

$$\tilde p(a,b,c) = \frac{p(a,c)\, p(b,c)}{p(c)} \quad \Longrightarrow \quad \Omega_{a,b} = 0$$

이 근사는 $a \perp b | c$를 강제하는 KL projection으로 설명할 수 있다. 그러나 sparsification과 반복 선형화가 filter consistency에 미치는 영향은 별도로 평가해야 하며, 단순히 "분산이 절대 줄지 않는다"고 일반화할 수 없다.

Active feature 수 $K$를 상수로 고정하면 local motion·measurement update의 핵심 행렬 크기를 $(2K+3) \times (2K+3)$로 제한할 수 있다. 이는 map size에 대한 update 복잡도를 상수로 만드는 근거이지만, 전체 map state recovery나 data association 비용까지 자동으로 O(1)이 되는 것은 아니다.

*Probabilistic Robotics*의 예시는 약 6개 active feature를 사용한다. 이는 보편 권장값이 아니며 sensor geometry와 map에 맞춰 consistency와 계산량을 함께 확인해야 한다. Eustice et al. (2006)의 Exactly Sparse EIF는 특정 구조에서 sparsification approximation을 피하는 접근을 다룬다. PR Figure 12.3은 measurement, motion, sparsification 단계의 link 변화를 보여준다.

##### 트리 기반 데이터 연관

정보형의 가산성은 data association에서도 특별한 능력을 준다: **소프트 대응 제약을 더하거나 뺄 수 있다**. 두 feature $m_i, m_j$가 동일하다는 soft constraint를

$$\Omega \leftarrow \Omega + F_{m_i - m_j}^T C\, F_{m_i - m_j}$$

로 추가하고, 그 factor를 별도로 보관했다면 제거할 수 있다. 이 구조로 data association tree를 A*류 frontier search로 탐색할 수 있지만 worst-case 가설 수는 지수적으로 늘어난다. Switchable constraints (Sünderhauf & Protzel 2012)와 Max-mixtures (Olson & Agarwal 2013)도 잘못된 loop closure에 robust하도록 설계됐다는 문제의식을 공유하지만, 각각 switch variable과 mixture factor를 사용하는 별도 정식화다.

##### 다중 로봇 맵 융합

정보형에서는 서로 독립적인 관측 factor의 기여를 더할 수 있어 multi-robot fusion을 표현하기 편하다. 두 로봇 $j, k$의 상태가 같은 좌표계에 있고 두 estimate 사이에 중복된 prior·관측 정보가 없다는 조건에서 정보 항을 더할 수 있다.

$$\Omega^{\text{fused}} = \Omega^{j \leftarrow k\text{-aligned}} + \Omega^k, \qquad \xi^{\text{fused}} = \xi^{j \leftarrow k\text{-aligned}} + \xi^k$$

공통 정보를 추적하지 않고 더하면 같은 관측을 두 번 세어 over-confidence가 생긴다. 공분산형도 단순 덧셈은 할 수 없지만 covariance intersection 같은 보수적 fusion 방법이 있다. Nettleton et al. (2003)은 분산 정보 fusion을 다뤘고, DDF-SAM, Kimera-Multi, Swarm-SLAM도 multi-robot estimation을 다루지만 통신 모델과 중복 정보 처리 방식은 서로 다르다(§14.13 참조).

SEIF와 iSAM2는 각각 approximate information filtering과 incremental smoothing이라는 다른 문제 설정이다. iSAM2도 비선형 문제에서는 linearization point와 relinearization 정책의 영향을 받는다. ESEIF는 SEIF의 sparsity를 더 정확히 다루는 계열이지만, VINS-Mono·OKVIS의 sliding-window marginalization과 MSCKF의 feature elimination은 제한된 상태를 유지하기 위한 별도 기법이다. 공통점은 sparsity와 계산량을 관리한다는 데 있다(§14.9.2 참조).

#### 14.16.4 EM Mapping (PR §13)

정보형 가산성에 sparsification을 더한 것이 SEIF였다. 그러나 data association이 불확실할 때, 모호한 측정을 버리지 않고 통계적으로 다루는 방법은 아직 없었다. EM Mapping이 그 자리를 채운다.

EKF-SLAM, EIF SLAM, SEIF는 모두 data association이 알려졌거나, ML greedy하게 결정한다고 가정했다. EM Mapping은 **unknown data association을 EM의 latent variable로 다뤄서 ambiguous한 데이터까지 활용**한다. Thrun, Burgard, Fox(1998–2000, AAAI/JAIR)의 원형은 RHINO 박물관 가이드 로봇(Burgard et al. 1999)에 변형되어 쓰였다.

##### E-step / M-step 골격

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

$\alpha$는 forward localization (Markov localization), $\beta$는 backward (미래 데이터로 과거 belief 보정). $\beta$ 덕분에 루프를 닫을 때 과거 belief가 거꾸로 수정된다 — 이것이 EM mapping의 통계적 핵심이자, PR Figure 13.10-13.12에서 가장 명확히 드러나는 특성이다. forward-backward 구조는 HMM의 Baum-Welch와 동일하다.

M-step은 frequentist count: "셀이 property l로 관측된 횟수 / 무엇이든 관측된 횟수", belief로 가중. 3-5회 반복이면 보통 수렴한다.

##### Layered EM Mapping

기본 EM_mapping의 M-step이 sensor cone에서 기하학적 일관성을 깨뜨리는 문제를 해결한 변종이다. 짧은 motion segment마다 **로컬 occupancy grid**를 먼저 만들고, EM은 그 로컬 맵의 *위치*만 최적화한다. **Deterministic annealing** ($\sigma: 1.0 \to 0$으로 냉각)으로 EM의 local maxima 함정을 회피한다.

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

Deterministic annealing과 GNC (Yang et al. 2020)·robust kernel scheduling은 목적 함수의 난도를 단계적으로 높여 나쁜 국소해를 피한다는 설계 원리를 공유한다.

##### EM Mapping이 주류에서 벗어난 이유

EM_mapping과 layered_EM_mapping은 현대 SLAM 시스템의 주류가 아니다. 이유:
- pose와 map을 E/M-step으로 분리하려는 시도는 factor graph의 joint optimization이 대체하면서 설 자리를 잃었다.
- batch·offline 성격이 실시간 SLAM과 맞지 않는다.
- Cartographer (Hess et al. 2016), GMapping (Grisetti et al. 2007) 모두 EM 없이 scan matching + pose graph로 직접 풀었다.

다만 layered EM의 **submap + global alignment**와 Cartographer의 local/global SLAM은, 로컬 지도를 먼저 만들고 전역적으로 정렬한다는 구조적 공통점이 있다. 이것은 직접적인 계보를 뜻하기보다 같은 계산 문제를 나누는 두 설계로 이해하는 편이 정확하다.

#### 14.16.5 Cycle Posterior (PR §14)

PR §14.3의 stepwise ML mapper는 두 가지 한계를 가진다: (1) 큰 odometry error를 견디지 못하고, (2) 과거 자세를 시간 역방향으로 보정할 수 없다. §14.4는 ML mapper와 *동시에* 자세 사후분포 추정기를 병행 구동하여 두 결함을 해결한다.

**알고리즘 골격**:
```
Incremental Mapping with Posterior Estimation:
  1. incremental_ML_mapping(o, a, s, m)  → ⟨m', s'⟩    [ML 갱신]
  2. Bel(s') = P(o,s') ∫ P(s'|a,s) Bel(s) ds            [사후분포 한 스텝]
  3. s'' = argmax Bel(s')                                 [사후 mode]
  4. s'' ≠ s'  →  cycle closure 검출
                   s'' − s'를 cycle 경로 따라 선형 분배
  5. incremental_ML_mapping을 시간 역방향으로 재실행       [nested ML refinement]
```

사후분포가 급격히 좁아지는 사건을 cycle closure로 보고, 좁아진 mode와 ML 추정값의 차이를 보정 신호로 쓴다. ML mapper와 posterior estimator를 함께 구동하므로 MCL 기반 구현과 잘 맞으며, odometry 없이도 동작하도록 설계됐다. 명시적인 cycle detection과 backwards correction을 한 프레임워크에 묶은 초기 사례라는 점에서 이후의 online loop-closure 시스템과 비교해 볼 수 있다. 다만 현대의 place recognition과 incremental graph optimization이 이 알고리즘에서 직접 계승되었다고 단정할 수는 없다.

MCL, linear distribution, nested ML을 그대로 결합한 구현은 현재 널리 쓰이지 않는다. 그러나 detector(place recognition)와 corrector(graph optimization)를 나누고, closure가 확인되면 graph 전체의 상태를 다시 최적화하는 구조는 현대 SLAM에서도 볼 수 있다. iSAM (Kaess et al. 2008)·iSAM2 (2012)·GTSAM의 incremental smoothing은 변화의 영향을 받는 부분을 선택적으로 갱신한다.

#### 14.16.6 정리: 무엇이 살아남았나

*Probabilistic Robotics*(2005)의 정보형 SLAM에서 현재 시스템으로 이어진 요소를 정리하면 다음과 같다.

| PR 알고리즘 | 핵심 기여 | 비교할 현대 구조 | 관계의 범위 |
|---|---|---|---|
| EKF-SLAM | 통합 landmark state, off-diagonal covariance | MSCKF류 filter, fiducial mapping | EKF를 공유하지만 state 구성은 다름 |
| EIF/GraphSLAM | 정보형 factor 가산, variable elimination, full trajectory | GTSAM, g2o, Ceres, iSAM2 | sparse least squares와 smoothing 관점이 이어짐 |
| SEIF | bounded active set, sparsification | ESEIF, bounded-state estimator | SEIF 계열과 다른 sparse estimator를 구분해야 함 |
| EM Mapping | latent data association, forward-backward localization, submap | EM 기반 mapping, submap system | 통계 기법·구조의 비교이며 직접 계보는 아님 |
| Cycle Posterior | online closure detection과 correction 결합 | place recognition + graph optimization | 기능 분할이 유사함 |

정보형의 **가산성**은 현대 factor cost의 합과 직접 비교할 수 있다. 반면 SEIF의 **sparsification**, sliding-window marginalization, Bayes-tree elimination은 모두 계산량을 관리하지만 서로 다른 연산이다. Cycle posterior의 detector/corrector 구분과 EM mapping의 submap은 현대 시스템과 구조적으로 비교할 수 있으나, 이를 특정 구현의 직접 계보로 단정하지 않는다(§14.9 참조).

> **참고 자료**
> - [Thrun et al., "Simultaneous Localization and Mapping with Sparse Extended Information Filters" (IJRR 2004)](https://journals.sagepub.com/doi/10.1177/0278364904045026) — SEIF 원본 논문
> - [Thrun & Montemerlo, "The GraphSLAM Algorithm with Applications to Large-Scale Mapping of Urban Structures" (IJRR 2006)](https://journals.sagepub.com/doi/10.1177/0278364906065390) — EIF/GraphSLAM 정식화
> - [Dissanayake et al., "A Solution to the Simultaneous Localization and Map Building (SLAM) Problem" (IEEE T-RA 2001)](https://ieeexplore.ieee.org/document/938381) — EKF-SLAM 고전 정식화
> - [Kaess et al., "iSAM2: Incremental Smoothing and Mapping Using the Bayes Tree" (IJRR 2012)](https://www.cs.cmu.edu/~kaess/pub/Kaess12ijrr.pdf) — incremental smoothing과 Bayes tree의 원 논문

---

서비스 로봇의 localization은 belief를 어떻게 표현하느냐에 따라 방법이 갈리고, pose graph 최적화 결과는 별도의 map generation 과정을 거쳐 최종 지도가 된다. Factor graph가 널리 쓰이게 된 역사도 이 흐름 안에서 이해할 수 있다. 천장 마커나 계산 자원이 제한된 시스템에서는 EKF localization의 골격도 여전히 선택할 수 있다. 두 방식의 공존은 한 구조로의 수렴이 모든 운용 조건의 답은 아니라는 점을 보여준다.
