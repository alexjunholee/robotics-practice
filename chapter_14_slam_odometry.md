# Ch.14 — SLAM & Odometry


로봇이 낯선 환경에서 "나는 어디에 있고, 주변은 어떻게 생겼는가?"를 동시에 알아내는 문제가 SLAM이다. GPS가 안 되는 실내, 지하, 건물 내부에서 로봇이 자율적으로 움직이려면 SLAM은 선택이 아니라 필수이다. 로봇 소프트웨어 엔지니어에게 가장 자주 요구되는 기술 중 하나이므로, 이론과 실습 모두 탄탄히 잡아야 한다.

---

## Part 1. 기초와 시스템

### 14.1 개념 소개

지도 없이 로봇을 돌려보면 바로 느낄 수 있다. 로봇은 자기가 어디 있는지 모르면 아무것도 못 한다. 내비게이션, 장애물 회피, 경로 계획 — 모든 것의 전제 조건이 "현재 위치"와 "주변 환경 정보"이다.

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
> - [Cyrill Stachniss — SLAM Course (University of Bonn)](https://www.youtube.com/playlist?list=PLgnQpQtFTOGQrZ4O5QzbIHgl3b1JHimN_) — SLAM 강의의 정석. SLAM을 처음 배운다면 이 시리즈를 보는 걸 권한다
> - [Thrun, Burgard, Fox, "Probabilistic Robotics" (Textbook)](https://mitpress.mit.edu/9780262201629/probabilistic-robotics/) — SLAM의 수학적 기반을 다루는 교과서. 칼만 필터, 파티클 필터, EKF-SLAM 등
> - [Barfoot, "State Estimation for Robotics" (Free PDF)](http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) — 상태 추정의 수학을 깊이 있게 다루는 교재. 무료 PDF 제공
> - [Awesome-SLAM GitHub](https://github.com/SilenceOverflow/Awesome-SLAM) — SLAM 관련 논문, 라이브러리, 데이터셋을 모아놓은 목록
> - [정진용 블로그 — SLAM 강의 시리즈 (Freiburg Robot Mapping 기반)](https://jinyongjeong.github.io/2017/02/13/lec01_SLAM_bayes_filter/) — Bayes filter부터 EKF/UKF/Particle filter, Graph SLAM, Robust SLAM까지 15편 시리즈. 한글로 된 SLAM 입문 자료 중 가장 체계적
> - [김기섭 블로그 — SLAM Back-end 공부자료 5개 추천](https://gisbi-kim.github.io/blog/2021/10/03/slam-textbooks.html) — Error-state KF, Factor Graphs, Bundle Adjustment 등 핵심 자료 큐레이션
> - [Robot Mapping Course (Uni Freiburg, Cyrill Stachniss)](http://ais.informatik.uni-freiburg.de/teaching/ws13/mapping/) — SLAM 강의 슬라이드와 과제 자료. 영상과 함께 보면 좋다
> - [EKF-SLAM 슬라이드 (Freiburg)](http://ais.informatik.uni-freiburg.de/teaching/ws12/mapping/pdf/slam04-ekf-slam.pdf) — 위 강의 중 EKF-SLAM 파트. 수식 전개가 깔끔하게 정리되어 있다

> **실습**: [SE(2) Odometry](https://alexjunholee.github.io/robotics-practice/app.html#se2_odometry)
> 2D 평면에서의 odometry 누적 과정을 직접 조작하며, drift가 어떻게 발생하는지 확인할 수 있다.

### 14.2 Visual Odometry (VO)

카메라만으로 상대적 이동을 추정한다. SLAM의 "front-end"에 해당하며, 여기서 추정한 이동이 부정확하면 SLAM 전체가 무너진다.

#### 14.2.1 Feature-based vs Direct Method

이 두 방식은 장단점이 뚜렷하다. 어떤 환경에서 로봇을 운용하느냐에 따라 선택이 달라진다.

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
> - [Daniel Cremers — Multiple View Geometry (TUM)](https://www.youtube.com/playlist?list=PLTBdjV_4f-EJn6udZ34tht9EVIW7lbeo4) — Visual Odometry의 수학적 기반을 배우기에 최적
> - [EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) — Visual(-Inertial) Odometry 벤치마크 데이터셋
> - [TUM RGB-D Benchmark](https://cvg.cit.tum.de/data/datasets/rgbd-dataset) — RGB-D SLAM/VO 벤치마크의 표준

### 14.3 Visual SLAM

#### 14.3.1 ORB-SLAM2/3

ORB-SLAM은 Visual SLAM의 표준 베이스라인이다. 대부분의 Visual SLAM 논문이 ORB-SLAM과 비교하며, 코드가 공개되어 있어 직접 빌드해서 돌려볼 수 있다. SLAM을 공부한다면 한 번은 직접 해볼 것을 권한다.

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
- **MonoSLAM (2007)**: 최초의 실시간 단안(monocular) SLAM. EKF 기반으로 작동했으나, 맵 크기가 커지면 계산량이 급증하는 한계가 있었다.
- **PTAM (Parallel Tracking and Mapping, 2007)**: Tracking과 Mapping을 별도 스레드로 분리한 최초의 시스템. 이 아키텍처가 이후 ORB-SLAM에 큰 영향을 미쳤다.
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

직접 돌려보면 알겠지만, 카메라만으로는 빠른 움직임이나 텍스처 없는 환경에서 트래킹이 쉽게 실패한다. IMU를 결합하면 이런 상황에서도 안정적으로 동작한다. VINS-Mono는 실제 드론이나 모바일 로봇에서 가장 많이 쓰이는 Visual-Inertial SLAM 시스템이다.

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

VINS-Mono의 핵심 기여: IMU Preintegration이라는 기법을 활용해, 두 키프레임 사이의 수백 개 IMU 측정을 하나의 상대 변환으로 압축한다. 이렇게 하면 최적화할 때 IMU 데이터를 일일이 다룰 필요 없이, 압축된 제약 조건 하나만 추가하면 된다. 계산 효율이 크게 올라간다.

> **추천 자료**
> - [Qin et al., "VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator" (2018)](https://arxiv.org/abs/1708.03852) — VINS-Mono 논문
> - [VINS-Mono GitHub](https://github.com/HKUST-Aerial-Robotics/VINS-Mono) — 공식 코드, ROS 지원

### 14.4 LiDAR Odometry & SLAM

카메라 기반 방법이 조명 변화나 텍스처에 민감한 반면, LiDAR는 직접 3D 거리를 측정하므로 이런 문제에서 자유롭다. 자율주행, 실외 로봇에서는 LiDAR SLAM이 사실상 표준이다.

#### 14.4.1 LOAM (Lidar Odometry and Mapping)

LOAM은 LiDAR SLAM의 출발점이다. 이후 나온 LeGO-LOAM, LIO-SAM, FAST-LIO 등 거의 모든 LiDAR SLAM이 LOAM의 아이디어를 계승하거나 확장했다.

- Edge points와 Planar points 분류
- Point-to-edge, point-to-plane 거리 최소화
- Odometry와 Mapping 분리 (주파수 다르게)

포인트 클라우드에서 기하학적으로 의미 있는 점들(모서리, 평면)만 추려 사용한다. 모든 점을 다 매칭하면 느리고 노이즈에 취약하지만, edge/planar 점만 골라 쓰면 빠르고 정확하다.

#### 14.4.2 LeGO-LOAM

**Lightweight and Ground-Optimized LOAM**:
- 지면 분리로 계산량 감소
- 지면을 기반으로 초기 추정
- 모바일 로봇에 적합

#### 14.4.3 LIO-SAM

LIO-SAM은 Factor Graph 기반 최적화를 LiDAR-Inertial SLAM에 적용한 대표작이다. Factor Graph의 핵심은 확장성에 있다. 센서를 하나 더 추가하고 싶으면 factor 하나만 추가하면 된다.

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

LiDAR는 10–20Hz로 스캔하는데, 로봇이 빠르게 움직이면 한 스캔 내에서도 로봇이 이동한다(motion distortion). 200–400Hz로 측정하는 IMU로 스캔 중 움직임을 보정(de-skewing)한 뒤, LiDAR로 정밀하게 맞추는 것이 LIO의 기본 구조다. 고속 움직임 상황에서 LiDAR 단독보다 유리한 이유가 여기에 있다.

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

SLAM을 돌려보면 시간이 지날수록 지도가 뒤틀리는 걸 볼 수 있다. 로봇이 큰 원을 그리며 출발점으로 돌아왔는데, 지도에서는 출발점과 도착점이 안 맞는 것이다. Loop Closure가 바로 그 뒤틀림을 교정하는 핵심 메커니즘이다. 이것 없이는 대규모 환경에서 SLAM이 사실상 불가능하다.

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

*연구자가 되고 싶다면 여기서부터 읽어라.*

SLAM 프론트엔드가 센서 데이터를 처리해서 제약 조건(constraint)을 만들어내면, 백엔드는 이 제약 조건들을 동시에 만족하는 최적의 상태(포즈, 랜드마크)를 찾는다. 이 과정은 비선형 최소제곱(nonlinear least squares) 문제다. 여기서 다루는 내용은 g2o, GTSAM, Ceres 같은 라이브러리를 "왜 그렇게 설정하는지" 이해하기 위한 수학적 배경이다.

**SLAM 백엔드가 푸는 문제의 직관**

복잡한 수식을 보기 전에 한 가지만 기억하자: SLAM 백엔드는 결국 **Ax = b를 푸는 문제**이다.

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

풀어야 하는 시스템 크기가 `n`에 관계없이 포즈 수 `m`에만 비례하게 된다. BA가 수만 개의 랜드마크를 다루면서도 실시간에 가까운 성능을 내는 이유가 여기에 있다.

`δp`를 구한 뒤, `δl`은 back-substitution으로 복원한다:
```
δl = H_ll^{-1} (b_l - H_lp · δp)
```

#### 14.9.3 희소성과 Variable Ordering

Pose graph optimization에서 `H` 행렬은 **sparse**하다. 각 포즈는 시간적으로 인접한 포즈, 그리고 loop closure로 연결된 포즈와만 제약 관계를 갖는다. 전체 포즈가 1000개라 해도, 각 포즈가 연결된 포즈는 기껏해야 수 개~수십 개이다.

sparse linear system을 풀 때 Cholesky factorization(`H = L L^T`)을 사용하는데, 여기서 **fill-in** 문제가 발생한다. 원래 0이었던 위치가 factorization 과정에서 non-zero가 되는 현상이다. Fill-in이 많으면 메모리와 계산 비용이 급증한다.

Fill-in을 최소화하려면 변수의 순서(variable ordering)를 잘 정해야 한다:
- **COLAMD** (Column Approximate Minimum Degree): 가장 널리 쓰이는 heuristic. 연결이 적은 변수부터 제거하는 방식
- **AMD** (Approximate Minimum Degree): COLAMD와 유사하지만 symmetric 행렬에 특화
- **Nested dissection**: 그래프를 재귀적으로 분할하여 ordering을 결정. 대규모 문제에서 효과적

g2o, GTSAM, Ceres 같은 라이브러리에서 solver를 설정할 때 linear solver type(DENSE_SCHUR, SPARSE_NORMAL_CHOLESKY 등)과 ordering strategy를 선택해야 한다. 이 배경 지식 없이 기본 설정으로 돌리면 "느려서 다른 라이브러리로 갈아탔다"는 식의 비효율이 생긴다. ordering만 바꿔도 10배 이상 속도 차이가 날 수 있다.

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

*연구자가 되고 싶다면 여기서부터 읽어라.*

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

핵심 포인트: 이 preintegrated measurement들은 **키프레임 `i`의 좌표계를 기준으로** 계산된다. 따라서 키프레임 `i`의 절대 포즈가 최적화 과정에서 바뀌더라도, preintegrated measurement를 재계산할 필요가 없다.

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
> - [Forster et al., "IMU Preintegration on Manifold for Efficient VIO" (2015 RSS)](https://rpg.ifi.uzh.ch/docs/RSS15_Forster.pdf) — 위 논문의 초기 버전. 핵심 아이디어가 더 간결하게 정리되어 있다
> - [Shan et al., "LIO-SAM" (IROS 2020)](https://github.com/TixiaoShan/LIO-SAM) — Tightly-coupled LIO의 레퍼런스 구현. 코드와 논문 모두 읽을 것
> - [Sola et al., "A micro Lie theory for state estimation in robotics" (arXiv:1812.01537)](https://arxiv.org/abs/1812.01537) — Lie group/algebra의 실용적 정리. Preintegration 읽기 전에 이것부터 보면 좋다
> - GTSAM의 `PreintegratedImuMeasurements` 클래스 소스코드 — 이론이 코드로 어떻게 구현되는지 확인
> - [IMU Preintegration MATLAB 구현](https://github.com/GentleDell/imu_preintegration_matlab) — KITTI에서 테스트한 MATLAB 코드. 수식과 코드를 대조하며 공부하기 좋다

### 14.11 심화: 관측가능성 분석 (Observability)

*연구자가 되고 싶다면 여기서부터 읽어라.*

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

SLAM/VIO 분야에서 오래된 논쟁이다. 결론부터 말하면, 수학적으로 Gauss-Newton 최적화와 Iterated EKF (IEKF)는 동치이다. 같은 문제를 다른 관점에서 푸는 것이다.

- **필터 (EKF, MSCKF 등)**: 새 측정이 들어올 때마다 상태를 점진적으로 업데이트한다. 과거 상태는 marginalize하여 현재 상태만 유지한다. 메모리 효율적이고, proprioceptive 센서(IMU)와의 결합에 자연스럽다.
- **최적화 (BA, factor graph)**: 과거 상태를 전부 유지하고 한꺼번에 최적화한다. 과거 데이터를 relinearize할 수 있으므로 정확도가 높다. 하지만 상태 수가 늘어나면 계산량이 커진다 (sliding window나 iSAM2로 완화).

그러면 "VINS-Mono(최적화)가 MSCKF(필터)보다 낫다"는 건 solver 때문인가? 아니다. 차이는 solver가 아니라 **시스템 구조**(어떤 상태를 유지하는가, 어떤 측정을 사용하는가)에서 온다. 최적화 기반이 relinearize를 통해 과거 linearization error를 줄일 수 있다는 점이 실질적 이점이다.

실무적 선택:
- IMU 중심 + 경량 → 필터 (MSCKF, FAST-LIO2의 IEKF)
- 카메라 중심 + 정확도 → 최적화 (VINS-Mono, ORB-SLAM3)
- 둘 다 필요 → 하이브리드 (LIO-SAM: IMU preintegration을 factor로 넣은 최적화)

(참고: [김기섭 블로그 — Gauss-Newton Opt == IEKF update?](https://gisbi-kim.github.io/blog/2022/03/05/gn-iekf-same.html))

### 14.12 심화: Semantic SLAM

*연구자가 되고 싶다면 여기서부터 읽어라.*

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

*연구자가 되고 싶다면 여기서부터 읽어라.*

한 대의 로봇이 넓은 환경을 탐색하려면 시간이 오래 걸린다. 여러 로봇이 동시에 나눠서 탐색하면 시간을 줄일 수 있지만, 각 로봇이 만든 부분 지도(submap)를 하나의 일관된 글로벌 지도로 합치는 것은 단순하지 않다.

**Centralized 접근**은 모든 로봇의 센서 데이터 또는 로컬 지도를 중앙 서버로 전송하고 서버에서 전체 SLAM을 수행한다. 구현이 단순하고 최적해에 가깝지만, 원본 데이터를 전부 보내면 통신 대역폭이 병목이 되고 서버가 단일 장애점(single point of failure)이 된다.

**Distributed 접근**은 각 로봇이 독립적으로 로컬 SLAM을 수행하고, rendezvous 또는 inter-robot loop closure가 발생했을 때 상대 포즈를 추정하여 지도를 정렬한다. 원본 데이터 대신 compressed descriptor(NetVLAD vector, summary map 등)를 교환하므로 통신 효율이 높다. 각 로봇은 자신의 포즈만 최적화하면서도 이웃 로봇과의 제약을 통해 전체가 수렴하는 구조다.

분산 시스템에서 반드시 해결해야 할 문제가 셋 있다. **inter-robot loop closure**는 로봇 A가 방문한 장소를 로봇 B가 나중에 방문했을 때 인식하는 것으로, 14.14절의 place recognition이 핵심이다. **좌표계 정렬**은 각 로봇이 독립 좌표계에서 시작하기 때문에 필요하다. 최소 3개의 inter-robot correspondence에서 상대 SE(3) 변환을 추정해야 정렬이 가능하다. **outlier rejection**은 inter-robot loop closure의 false positive가 많을 수 있어 PCM(Pairwise Consistency Maximization)이나 GNC(Graduated Non-Convexity) 같은 robust 기법이 필요하다. 분산 최적화는 Distributed Gauss-Seidel, ADMM 등을 사용한다.

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

*연구자가 되고 싶다면 여기서부터 읽어라.*

Loop closure의 핵심 문제: "지금 보는 장면을 이전에 본 적 있는가?" 이것은 이미지 검색(image retrieval) 문제다. 현재 프레임의 descriptor를 과거 모든 키프레임의 descriptor와 비교해서 가장 유사한 것을 찾는다. SLAM의 정확도는 loop closure에, loop closure는 place recognition에 달려 있다.

**전통적 방법: Bag of Visual Words (BoVW)**

DBoW2 라이브러리가 대표적이며 ORB-SLAM2/3에서 사용된다.
1. 대규모 이미지에서 local feature(ORB 등)를 추출
2. k-means clustering으로 visual vocabulary(단어 사전) 구축
3. 각 이미지를 "어떤 visual word가 몇 번 나타났는지"의 histogram(BoW vector)으로 표현
4. BoW vector 간의 유사도(L1-score 등)로 이미지 비교

장점: 빠르다 (inverted index 사용), 검증된 방법. 단점: 시점/조명 변화에 취약, vocabulary 학습이 필요.

**학습 기반 방법: Global Descriptor**

이미지 전체를 하나의 compact vector로 압축하는 방식으로 BoVW보다 robust하다. **NetVLAD** (2016)는 CNN feature를 VLAD aggregation으로 결합하여 도시 규모 장소 인식에서 기존 방법을 크게 앞섰다. 이후 **CosPlace** (2022)는 contrastive learning으로 학습 파이프라인을 단순화하면서 성능을 높였다. **MixVPR** (2023)은 feature mixing으로 주간/야간·계절 변화 등 다양한 조건에서 robust하다. **AnyLoc** (2023)은 DINOv2 feature를 활용하여 별도 fine-tuning 없이 indoor/outdoor, aerial/ground 어떤 환경에서도 동작하는 zero-shot place recognition을 내놓았다.

**LiDAR 기반 Place Recognition**:

시각 정보 없이 3D 구조만으로 장소를 인식한다. 조명 변화에 완전히 면역이지만, 구조적으로 유사한 환경(예: 긴 복도)에서 혼동될 수 있다.

- **Scan Context** (IROS 2018): 3D 포인트 클라우드를 bird-eye view로 투영한 뒤, 거리/높이 기반의 2D descriptor 생성. Rotation-invariant한 매칭 가능
- **OverlapTransformer** (2022): Transformer 기반으로 LiDAR range image에서 global descriptor를 학습

**Cross-modal Place Recognition**: 카메라 이미지로 query하고 LiDAR 지도에서 검색하거나, 그 반대. 센서가 다른 로봇 간의 multi-robot SLAM에서 중요하다.

**Sequence Matching**: 단일 이미지 매칭의 한계를 극복하기 위해, 연속된 프레임의 시퀀스를 함께 매칭한다.
- **SeqSLAM** (2012): 이미지 개별 유사도는 낮아도, 시퀀스 패턴이 일치하면 같은 장소로 판단. 극적인 외관 변화(주간 vs 야간)에서도 동작
- 최근 방법: sequence descriptor를 학습하여 더 효율적으로 시퀀스 매칭

실무 팁: 대부분의 SLAM 시스템은 DBoW2를 기본으로 쓴다. 만약 조명/계절 변화가 큰 환경에서 운용해야 한다면, 학습 기반 방법(NetVLAD 이후의 방법들)으로 교체하는 것을 고려하라. AnyLoc은 fine-tuning 없이 쓸 수 있어서 진입 장벽이 낮다.

> **추천 자료**
> - [Arandjelovic et al., "NetVLAD: CNN architecture for weakly supervised place recognition" (arXiv:1511.07247)](https://arxiv.org/abs/1511.07247) — 학습 기반 place recognition의 시작점
> - [Keetha et al., "AnyLoc: Towards Universal Visual Place Recognition" (arXiv:2308.00688)](https://arxiv.org/abs/2308.00688) — Foundation model 기반 zero-shot place recognition
> - [Kim & Kim, "Scan Context: Egocentric Spatial Descriptor for Place Recognition within 3D Point Cloud Map" (IROS 2018)](https://ieeexplore.ieee.org/document/8593953) — LiDAR place recognition의 대표 방법
> - [김기섭 블로그 — Scan Context-based LiDAR Pose-graph SLAM 구현](https://gisbi-kim.github.io/blog/2021/05/17/sclidarslam.html) — Scan Context를 LiDAR SLAM에 통합한 구현 해설
> - [다크 프로그래머 — Bag of Words 기법](https://darkpgmr.tistory.com/125) — BoW의 원리를 이미지 검색과 연결하여 설명

> **기술 흐름: SLAM & Odometry**
> - **~2007**: 고전기. EKF-SLAM, FastSLAM(파티클 필터 기반)이 주류. MonoSLAM(2007)이 실시간 단안 SLAM의 시작을 알림. PTAM(2007)이 Tracking/Mapping 분리 아키텍처를 제안
> - **2010~2015**: LSD-SLAM, SVO 등 Direct Method 등장. LOAM(2014)이 LiDAR SLAM의 기틀 마련. ORB-SLAM(2015)이 Feature-based Visual SLAM의 완성형으로 등극
> - **2015~2020**: Visual-Inertial 시대. VINS-Mono(2018), MSCKF 등 VIO 시스템이 드론/모바일에서 표준으로 자리잡음. DSO(2018)가 Direct Sparse 방식 제안. LiDAR-Inertial 결합 본격화: LIO-SAM(2020)
> - **2020~2023**: FAST-LIO/FAST-LIO2(2021/2022)가 경량 LiDAR-Inertial 시스템의 새 표준. ORB-SLAM3(2021)가 Visual-Inertial + 멀티맵 지원. DROID-SLAM(2021)이 learning-based SLAM의 가능성을 보여줌. R3LIVE 등 멀티센서 통합 시스템 등장
> - **2024~**: 3DGS 기반 SLAM(SplaTAM, MonoGS, Gaussian-SLAM)이 Neural SLAM의 방향을 바꾸고 있다. Foundation Model과 SLAM의 결합(예: 자연어로 장소를 설명하여 위치를 찾는 등) 연구도 시작
> - **지금 주목할 것**: 기존 geometric SLAM(ORB-SLAM3, FAST-LIO2)은 이미 성숙한 기술이므로 꼭 익혀두고, 3DGS-SLAM과 learning-based 방법은 트렌드로 지켜보자. 실무에서는 LIO-SAM/FAST-LIO2(실외)와 ORB-SLAM3(실내)가 여전히 가장 많이 쓰인다. 벤치마크 데이터셋(KITTI, EuRoC, TUM RGB-D)으로 직접 돌려보는 것이 가장 빠른 학습 방법이다.

### 14.15 심화: Long-term Mapping

*연구자가 되고 싶다면 여기서부터 읽어라.*

실제 환경에서 로봇을 운용하면 "지도를 한 번 만들고 끝"이 아니다. 같은 장소를 여러 번 방문하면서 지도를 업데이트하고, 동적 물체(사람, 차량)를 제거하고, 여러 세션의 데이터를 통합해야 한다. 이것이 long-term mapping이고, 실용적인 로봇 시스템에서 피할 수 없는 문제다.

#### 14.15.1 Incremental Smoothing: iSAM에서 iSAM2까지

Filter-based SLAM(EKF 등)은 state 수가 늘어나면 Jacobian 행렬이 커져서 실시간 처리가 어렵다. iSAM(Kaess et al., TRO 2008)은 QR factorization의 R matrix를 Givens rotation으로 incremental하게 업데이트할 수 있음을 보여줬다. 새로운 measurement가 추가될 때 전체를 재계산하지 않고, 변경된 부분만 갱신한다. 다만 non-zero element가 누적되면 주기적으로 re-ordering이 필요하다.

iSAM2(Kaess et al., IJRR 2012)는 Bayes tree 구조를 도입하여 이 한계를 극복했다. 영향받는 subtree만 re-elimination하므로, 대규모 문제에서도 일관된 성능을 보인다. GTSAM의 핵심 엔진이 바로 iSAM2다.

#### 14.15.2 Dynamic Object Removal

지도에서 동적 물체를 제거하는 것은 long-term mapping의 필수 과제다.

**Removert** (Kim et al., 2020): multi-resolution range image를 이용해 static/dynamic을 분류한다. Point cloud를 range image로 투영한 뒤, 다른 시점에서 관측한 range와 비교하여 동적 여부를 판단한다. 보수적으로 static point를 확보한 후, false removed point를 복원하는 two-stage 방식을 사용한다. 핵심은 multiple confidence level로 trade-off를 조절할 수 있다는 점이다.

기존 접근과 비교하면: voxel ray-casting은 정확하지만 연산이 비싸고, visibility-based는 뒤의 static point를 보존한다는 가정이 필요하며, segmentation-based는 unknown label에 약하고 scan-to-map 관계를 무시한다. Removert는 이 세 방법의 단점을 multi-resolution range image 비교로 보완한다.

**SuMa++** (Chen et al., IROS 2019): surfel-based mapping에 semantic label을 추가한 시스템이다. LiDAR point에 normal과 semantic 정보를 더해서, semantic과 motion 모두에서 dynamic으로 판정된 경우에만 surfel을 제거한다. 단순히 다 지우지 않는 이유는, motion degenerate한 환경에서 dynamic이지만 geometric으로 유용한 point가 있을 수 있기 때문이다.

#### 14.15.3 Multi-Session SLAM

같은 환경을 여러 날에 걸쳐 매핑하면, 각 세션의 trajectory를 하나로 합쳐야 한다. 문제는 gauge freedom — 각 세션의 좌표계가 다르므로 단순히 합치면 안 맞는다.

**LT-mapper** (Kim et al., 2021): Scan Context 기반 anchor node로 multi-session을 정렬하고, positive/negative change detection으로 지도를 업데이트한다. 변화 정도에 따라 high-dynamic, low-dynamic, weak non-dynamic, strong positive-dynamic을 구분하여 delta map을 관리한다.

**Continuous-Time Estimation** (Furgale et al., ICRA 2012): discrete-time 대신 B-spline basis function으로 trajectory를 표현하면, 다른 Hz의 센서들을 더 적은 변수로 통합할 수 있다. 빠른 센서(IMU)와 느린 센서(LiDAR, 카메라) 사이의 self-calibration에도 활용 가능하다.

> **추천 자료**
> - [Kaess et al., "iSAM2: Incremental Smoothing and Mapping Using the Bayes Tree" (IJRR 2012)](https://www.cs.cmu.edu/~kaess/pub/Kaess12ijrr.pdf) — Bayes tree 기반 incremental SLAM의 원본 논문
> - [Kim et al., "Remove, then Revert: Static Point Cloud Map Construction using Multiresolution Range Images" (IROS 2020)](https://github.com/irapkaist/removert) — Dynamic point 제거의 실용적 방법. 코드 공개
> - [Kim et al., "LT-mapper: A Modular Framework for LiDAR-based Lifelong Mapping" (ICRA 2022)](https://github.com/gisbi-kim/lt-mapper) — Multi-session SLAM 프레임워크
> - [Chen et al., "SuMa++: Efficient LiDAR-based Semantic SLAM" (IROS 2019)](https://github.com/PRBonn/semantic_suma) — Semantic 정보를 활용한 LiDAR SLAM
