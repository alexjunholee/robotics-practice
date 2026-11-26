# 14장 SLAM과 오도메트리


GPS 신호가 닿지 않는 실내나 지하, 고층 빌딩 사이에서 로봇이 스스로 움직이려면 두 가지 질문에 동시에 답해야 한다. "나는 지금 어디에 있는가?"와 "내 주변은 어떻게 생겼는가?" 이 두 과제를 한꺼번에 푸는 문제가 바로 SLAM(Simultaneous Localization and Mapping)이다.

---

## 기초와 시스템

### 14.1 개념 소개

로봇이 목적지까지 경로를 짜고 안전하게 주행하려면 자기 위치와 주변 지도가 모두 필요하다. 그런데 이 둘은 서로를 전제로 삼는 닭과 달걀의 관계에 놓여 있다. 자기 위치를 정확히 알려면 주변 지도가 먼저 있어야 하고, 반대로 주변 지도를 일관되게 그리려면 지금 어느 위치에서 센서로 측정하고 있는지 알아야 하기 때문이다. SLAM은 위치 추정과 지도 작성을 따로 떼어 놓지 않고 동시에 처리해 이 순환 고리를 푼다.

만약 센서와 바퀴가 완벽하다면 출발점에서부터 이동량만 차곡차곡 더해도 현재 위치를 바로 알 수 있다. 하지만 현실에서는 센서 측정에 노이즈가 섞이고, 바퀴는 바닥에서 미끄러지며, 이동 중 카메라 영상은 흔들린다. 이 작은 오차들이 매 순간 누적되면 추정 위치가 실제 경로에서 점점 벌어지는 드리프트(drift)가 발생한다. SLAM의 핵심은 단순히 이동량을 쌓는 데 그치지 않고, 주변 환경의 관측을 이용해 이 누적 드리프트를 되돌려 보정하면서 일관된 지도를 유지하는 데 있다.

로봇의 이동만 추적하는 오도메트리(Odometry)와 지도를 함께 관리하는 SLAM의 차이는 다음 표로 요약된다.

**Odometry vs SLAM**:
| 특징 | Odometry | SLAM |
|---|---|---|
| 출력 | 상대적 이동 | 위치 + 지도 |
| Loop Closure | 없음 | 있음 |
| Drift | 누적 | 보정 가능 |
| 계산량 | 적음 | 많음 |

오도메트리는 계산이 가볍지만 한 번 생긴 드리프트를 되돌릴 방법이 없고, SLAM은 계산량이 늘어나는 대신 지도를 바탕으로 이전에 지나온 장소를 다시 알아보고(Loop Closure) 궤적 전체를 바로잡는다.

> **추천 자료**
> - [Cyrill Stachniss — SLAM Course (2013)](https://www.youtube.com/playlist?list=PLgnQpQtFTOGQrZ4O5QzbIHgl3b1JHimN_) — Bayes filter부터 graph-based SLAM까지 이어지는 공개 강의 시리즈
> - [Thrun, Burgard, Fox, "Probabilistic Robotics" (Textbook)](https://mitpress.mit.edu/9780262201629/probabilistic-robotics/) — 칼만 필터, 입자 필터, EKF-SLAM 등 SLAM의 수학적 토대를 다루는 표준 교과서
> - [Barfoot, "State Estimation for Robotics" (Free PDF)](https://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) — 상태 추정의 수학을 깊이 있게 다루는 교재(무료 PDF 공개)
> - [Awesome-SLAM GitHub](https://github.com/SilenceOverflow/Awesome-SLAM) — SLAM 관련 논문, 라이브러리, 데이터셋 큐레이션 목록
> - [정진용 블로그 — SLAM 강의 시리즈 (Freiburg Robot Mapping 기반)](https://jinyongjeong.github.io/2017/02/13/lec01_SLAM_bayes_filter/) — Bayes filter부터 EKF/UKF/Particle filter, Graph SLAM, Robust SLAM까지 잇는 15편의 한국어 시리즈
> - [김기섭 블로그 — SLAM Back-end 공부자료 5개 추천](https://gisbi-kim.github.io/blog/2021/10/03/slam-textbooks.html) — Error-state KF, Factor Graphs, Bundle Adjustment 등 핵심 자료 큐레이션
> - [Robot Mapping Course (Uni Freiburg, Cyrill Stachniss)](http://ais.informatik.uni-freiburg.de/teaching/ws13/mapping/) — SLAM 강의 슬라이드와 과제 자료로, 위 공개 강의 영상과 짝지어 보기 좋다
> - [EKF-SLAM 슬라이드 (Freiburg)](http://ais.informatik.uni-freiburg.de/teaching/ws12/mapping/pdf/slam04-ekf-slam.pdf) — 위 강의 중 EKF-SLAM 파트로, 수식 전개가 깔끔하게 정리되어 있다

> **실습**: [SE(2) Odometry](https://alexjunholee.github.io/robotics-practice/app.html#se2_odometry)
> 2D 평면에서 odometry가 쌓이는 과정을 직접 조작하며 drift가 어떻게 발생하는지 확인할 수 있다.

이제 SLAM의 앞단에서 연속된 센서 입력으로부터 로봇의 상대 이동을 뽑아내는 오도메트리부터 살펴보자. 가장 흔한 센서인 카메라로 시작한다.

### 14.2 Visual Odometry (VO)

Visual Odometry(VO)는 오직 카메라 영상의 연속적인 변화만으로 로봇이 얼마나 이동하고 회전했는지 추정하는 기술이다. SLAM 파이프라인에서 앞단(front-end)을 맡는 모듈인 만큼, 여기서 움직임을 잘못 계산하면 뒷단에서 아무리 정교하게 지도를 다듬어도 전체 추정이 흔들린다.

#### 14.2.1 Feature-based vs Direct Method

카메라 영상에서 움직임을 뽑아내는 방법은 크게 두 갈래로 나뉘며, 로봇이 다닐 환경의 질감과 조명 조건을 보고 선택한다.

먼저 **Feature-based** 방식(ORB-SLAM 계열)은 영상에서 코너나 블롭 같은 특징점(feature)을 추출한다. 시점이 달라져도 다시 찾아낼 수 있는 이 점들을 연속 프레임 사이에서 서로 짝지어 카메라 움직임을 역산한다. 계산 과정은 지도의 유무에 따라 두 단계로 달라진다. 아직 3D 지도가 없는 초기화 단계에서는 두 영상의 2D-2D 대응만으로 Essential 또는 Fundamental Matrix(장면이 평면이라면 Homography)를 풀어 초기 카메라 자세와 점들의 3D 좌표를 잡는다. 3D 맵 포인트가 구축되고 나면 현재 프레임의 2D 관측과 기존 3D 맵 포인트 사이의 재투영 오차(reprojection error)를 최소화하며 위치를 추적한다. 특징점 기술자가 밝기 변화에 꽤 강인해 오랫동안 널리 쓰였지만, 흰 벽이나 무늬 없는 바닥처럼 특징점을 뽑기 어려운 환경에서는 추적이 쉽게 끊긴다.

```
이미지 → 특징점 추출 → 매칭 → 움직임 추정
```

반면 **Direct Method**(DSO, LSD-SLAM 계열)는 특징점을 뽑고 짝짓는 중간 단계를 건너뛰고 픽셀 밝기 값을 직접 비교한다. 연속된 프레임에서 같은 3D 점을 바라본다면 픽셀 밝기도 같을 것이라는 광도 불변 가정(brightness constancy)에 바탕을 두고, 밝기 차이의 제곱합이 최소화되는 카메라 움직임을 찾는다. 특징점에 의존하지 않으므로 텍스처가 희박한 복도에서도 살아남을 때가 많지만, 카메라 노출이 바뀌거나 조명이 급변하면 밝기 가정이 깨지면서 오차가 빠르게 커진다.

```
이미지 → 픽셀 밝기 직접 비교 → 움직임 추정
```

#### 14.2.2 Mono vs Stereo vs RGB-D

영상을 어떻게 해석할지 정했다면, 그다음에는 카메라를 몇 대 쓰고 깊이 정보를 어떤 방식으로 얻을지 골라야 한다. 단안(Monocular), 스테레오(Stereo), RGB-D 구성은 스케일 복원 가능 여부와 적합한 운용 환경이 뚜렷하게 갈린다.

| 구성 | Scale | 특징 | 적합 환경 |
|---|---|---|---|
| Monocular | 불가 (ambiguity) | 경량·단순, IMU 없이 스케일 복원 불가 | 저비용 드론, 모바일 |
| Stereo | 가능 | 베이스라인이 측정 범위를 제한 | 일반 실내·실외 |
| RGB-D | 가능 | 깊이 직접 측정, 실외·직사광선에 취약 | 실내 구조화 환경 |

단안 카메라에서 생기는 scale ambiguity는 2D 투영의 본질적인 한계에서 비롯된다. 가까이 있는 작은 물체와 멀리 있는 큰 물체는 크기를 거리 비율에 맞춰 조절하면 이미지 평면에 완전히 똑같은 영상을 맺기 때문이다. 이 때문에 단안 SLAM으로 만든 지도는 실제 미터(m) 단위가 아니라 임의의 스케일로 그려지며, 절대 크기를 복원하려면 IMU나 바퀴 오도메트리 같은 다른 센서를 결합해야 한다. 또한 단안 카메라로 처음 3D 구조를 세우려면 카메라가 옆으로 이동해 충분한 시차(parallax)를 만들어야 한다. 제자리에서 회전만 하면 두 영상 사이의 기선이 0이라 깊이를 풀 수 없다. 카메라와 IMU를 함께 초기화할 때도 scale, IMU bias, 중력 방향이 수식에서 구분될 만큼 충분히 가속하고 회전해야 한다. IMU 없이 단안 카메라만 쓴다면 아무리 오래 이동해도 절대 scale은 끝내 정해지지 않는다.

> **추천 자료**
> - [Daniel Cremers — Multiple View Geometry (TUM)](https://www.youtube.com/playlist?list=PLTBdjV_4f-EJn6udZ34tht9EVIW7lbeo4) — Visual Odometry에 필요한 다중 뷰 기하를 다루는 공개 강의
> - [EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) — Visual(-Inertial) Odometry 벤치마크 데이터셋
> - [TUM RGB-D Benchmark](https://cvg.cit.tum.de/data/datasets/rgbd-dataset) — RGB-D SLAM/VO에서 널리 쓰이는 실내 데이터셋과 평가 도구

VO에서는 프레임 사이의 상대적인 이동을 추정하는 데 초점을 맞추는 반면, Visual SLAM에서는 여기에 지역 지도 관리와 루프 폐쇄(loop closure)를 더해 전역적으로 일관된 궤적과 지도를 완성한다. 이어지는 절에서는 대표적인 Visual SLAM 시스템 세 가지를 살펴본다.

### 14.3 Visual SLAM

#### 14.3.1 ORB-SLAM2/3

ORB-SLAM 계열은 Visual SLAM 논문에서 성능을 견줄 때 사실상의 기준선(baseline)으로 삼는 공개 시스템이다. 전체 소스 코드가 공개되어 있어 직접 빌드해 돌려보며 어떤 장면에서 강점을 보이고 어디서 추적을 놓치는지 확인하기 좋다.

시스템에서는 역할이 다른 모듈 세 개가 병렬 스레드로 맞물려 돌아간다.

1. **Tracking**: 프레임마다 특징점을 추출해 기존 지도와 맞춰 현재 카메라 포즈를 실시간으로 추정한다.
2. **Local Mapping**: 추적에서 선별된 키프레임이 들어올 때 새 3D 맵 포인트를 삼각측량으로 추가하고 지역 번들 조정(local BA)으로 주변 지도를 다듬는다.
3. **Loop Closing**: 새 키프레임이 추가될 때마다 이전에 지나온 장소인지 검사해 루프를 감지하고, 발견되면 그래프 전체의 드리프트를 한꺼번에 보정한다.

이처럼 수행 주기와 계산량이 다른 일을 스레드 세 개로 분리해 병렬로 처리하기 때문에, Tracking이 프레임 속도를 놓치지 않고 실시간으로 도는 동안에도 백그라운드에서 지도 전체의 일관성을 꾸준히 유지할 수 있다.

최신 버전인 **ORB-SLAM3**는 단안·스테레오·RGB-D 입력에 더해 Visual-Inertial 모드를 통합했고, 멀티맵(multi-map) 기능과 왜곡이 큰 Fish-eye 카메라 모델까지 지원한다. 멀티맵은 추적이 잠시 끊겨도 새 지도를 열어 주행을 이어가다 나중에 합치는 기능이다.

ORB-SLAM의 설계는 앞선 단안 SLAM 연구의 계보 위에서 발전해 왔다.
- **MonoSLAM (2007)**: 초기의 실시간 단안 SLAM으로, EKF 상태 벡터에 포즈와 랜드마크를 함께 넣어 지도가 커질수록 계산량이 급격히 늘어났다.
- **PTAM (Parallel Tracking and Mapping, 2007)**: Tracking과 Mapping을 별도 스레드로 분리해 계산 병목을 깼으며, ORB-SLAM이 이 병렬 구조를 그대로 이어받았다.
- **ORB-SLAM (2015)**: PTAM 구조에 회전·크기 변화에 강한 ORB 특징점과 Loop Closure를 결합하고, 추적을 잃고 시점이 크게 달라져도 다시 위치를 잡는 재위치 추정(relocalization)을 넣었다.
- **ORB-SLAM2 (2017)**: Stereo와 RGB-D 카메라 지원을 추가해 절대 스케일을 바로 복원할 수 있게 되었다.
- **ORB-SLAM3 (2021)**: Visual-Inertial 통합, 멀티맵, 비선형 왜곡 카메라 모델까지 확장했다.

실제 EuRoC 데이터셋으로 ORB-SLAM3의 단안 예제를 돌릴 때는 실행 바이너리에 어휘 사전(vocabulary), 카메라 설정 YAML 파일, 데이터셋 경로를 순서대로 넘긴다.

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

보통 Direct Method는 영상의 거의 모든 픽셀을 쓰는 dense 방식을, Feature-based는 코너 등 일부 점만 고르는 sparse 방식을 택한다. **DSO**는 이 통념을 비틀어 direct 방식의 광도 오차를 쓰면서도 계산 대상은 선별된 점들로 좁힌(sparse) 오도메트리다. 영상 전체에서 밝기 기울기(gradient)가 뚜렷한 픽셀만 추려내어 특징점 추출기 없이도 photometric bundle adjustment로 최적화하므로, 계산 효율과 텍스처가 적은 환경에서의 강인함을 함께 챙긴다.

- 특징점 추출 없이 픽셀 밝기 직접 사용
- 밝기 기울기가 큰 점들만 선별해 사용 (Sparse)
- Photometric bundle adjustment 기반 최적화

> **추천 자료**
> - [Engel et al., "Direct Sparse Odometry" (2018)](https://arxiv.org/abs/1607.02565) — DSO 논문

#### 14.3.3 VINS-Mono/Fusion

단안 카메라만으로는 드론처럼 빠르게 움직여 모션 블러가 심하거나 텍스처가 희박한 벽을 지날 때 추적이 끊기기 쉽다. 이럴 때 수백 Hz로 가속도와 각속도를 내보내는 IMU를 결합하면 영상 프레임 사이의 빈틈을 관성 측정으로 촘촘하게 메울 수 있다. **VINS-Mono**(및 스테레오·GPS 확장을 포함한 **VINS-Fusion**)는 카메라와 IMU를 밀접하게 묶는(tightly-coupled) 대표적인 Visual-Inertial Navigation System(VINS)으로, 드론과 모바일 로봇에 널리 쓰인다.

전체 흐름은 다음과 같이 IMU 사전 적분과 시각 추적이 슬라이딩 윈도 최적화기 안에서 만나는 구조다.

```
센서 입력 → IMU Preintegration →
Visual Feature Tracking →
Sliding Window Optimization →
Loop Closure (optional)
```

- Camera + IMU tight coupling으로 스케일과 중력 방향 복원
- 최근 키프레임 일정 개수만 유지하는 Sliding window optimization
- 누적 오차 보정을 위한 Loop closure 지원
- 드론 및 모바일 로봇 현장에서 검증된 범용성

여기서 핵심 역할을 하는 기술이 **IMU preintegration**이다. 키프레임 하나가 들어오는 짧은 시간 동안에도 IMU 측정값은 수십 개씩 쌓이는데, 최적화가 한 번 돌 때마다 이 측정값을 처음부터 다시 적분하면 계산량이 감당할 수 없이 불어난다. VINS-Mono는 두 키프레임 사이의 IMU 측정 전체를 하나의 상대 운동 제약으로 미리 압축해 두었다가, 최적화기 안에서 비선형 갱신이 일어날 때도 재적분 없이 바로 이 제약을 활용한다(구체적인 수식은 §14.10에서 다룬다).

> **추천 자료**
> - [Qin et al., "VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator" (2018)](https://arxiv.org/abs/1708.03852) — VINS-Mono 논문
> - [VINS-Mono GitHub](https://github.com/HKUST-Aerial-Robotics/VINS-Mono) — 공식 코드, ROS 지원

### 14.4 LiDAR Odometry & SLAM

카메라는 풍부한 질감 정보를 주지만 어두운 터널이나 급격한 역광, 무늬 없는 벽 앞에서는 쉽게 흔들린다. 반면 LiDAR는 능동적으로 레이저를 쏘아 3D 거리를 직접 측정하므로 조명에 구애받지 않고 정밀한 기하 구조를 얻는다. 이 특성 덕분에 자율주행 차량이나 야외 이동 로봇에서는 LiDAR 기반 SLAM이 주력으로 쓰인다.

#### 14.4.1 LOAM (Lidar Odometry and Mapping)

초당 수십만 개씩 쏟아지는 LiDAR 포인트를 매 스캔마다 전부 정합하려면 계산 비용이 너무 크다. **LOAM**은 포인트 클라우드에서 곡률을 계산해 날카로운 모서리(edge)와 평평한 면(planar)에 놓인 점만 추려내어 계산량을 크게 줄였다. 여기에 더해 빠른 속도로 대략적인 이동량을 뽑는 **Odometry** 스레드와, 낮은 주기로 정밀한 전역 지도를 다듬는 **Mapping** 스레드를 분리한 이중 주기 구조를 제안했다.

- 곡률 기반 Edge points와 Planar points 분류
- Point-to-edge, point-to-plane 거리 최소화
- Odometry와 Mapping 분리 (서로 다른 주파수로 병렬 실행)

이 특징점 분리 전략과 이중 주기 아키텍처는 LeGO-LOAM, LIO-SAM 같은 후속 LiDAR SLAM의 뼈대가 되었다.

#### 14.4.2 LeGO-LOAM

지상에서 주행하는 모바일 로봇은 매 스캔마다 바닥에서 수많은 점을 얻는데, 이를 일반 특징점과 섞어 정합하면 계산량만 늘고 수직 방향 제약을 제대로 쓰기 어렵다. **LeGO-LOAM**(Lightweight and Ground-Optimized LOAM)은 바닥 점을 먼저 분리해 내어(ground segmentation) 지면 점으로는 높이·롤·피치(z, roll, pitch)를, 나머지 점으로는 평면 이동과 회전(x, y, yaw)을 따로 추정한다. 덕분에 연산 부담이 가벼워져 임베디드 보드를 탑재한 소형 모바일 로봇에서도 실시간으로 동작한다.

#### 14.4.3 LIO-SAM

고속으로 회전하거나 요철을 넘을 때는 LiDAR 스캔 하나만으로는 움직임을 따라잡기 어렵다. **LIO-SAM**(LiDAR-Inertial Odometry via Smoothing and Mapping)은 LiDAR와 IMU를 **Factor Graph** 위에서 밀결합(tightly-coupled)해 최적화하는 시스템이다. Factor Graph를 쓰면 새로운 센서 측정을 그래프의 factor로 끼워 넣기만 하면 되므로, GPS나 루프 폐쇄(Loop Closure) 제약까지 자연스럽게 확장할 수 있다.

```
                    ┌──────────────┐
IMU ──────────────→ │              │
                    │ Factor Graph │ ──→ Pose
LiDAR ────────────→ │              │
                    │  iSAM2       │
GPS (optional) ───→ │              │
                    └──────────────┘
```

Factor Graph는 추정해야 할 변수(로봇 포즈, 랜드마크 위치)와 그 사이를 묶는 제약(센서 측정)을 이분 그래프로 나타낸다. IMU 사전 적분, LiDAR 스캔 매칭, GPS 수신값, 루프 폐쇄가 각각 하나의 factor로 들어가며, 새로운 센서를 추가할 때도 기존 수식을 갈아엎지 않고 새 factor만 붙이면 된다. 이렇게 쌓인 그래프의 비선형 최적화는 GTSAM 라이브러리의 iSAM2 엔진이 증분 방식으로 풀어낸다.

> **추천 자료**
> - [Shan et al., "LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping" (2020)](https://arxiv.org/abs/2007.00258) — LIO-SAM 논문
> - [Vizzo et al., "KISS-ICP: In Defense of Point-to-Point ICP" (RA-L 2023, arXiv:2209.15397)](https://arxiv.org/abs/2209.15397) — 잘 다듬은 point-to-point ICP로 복잡한 LiDAR odometry와 비슷한 성능을 보고
> - [LIO-SAM GitHub](https://github.com/TixiaoShan/LIO-SAM) — 공식 코드, ROS 지원
> - [GTSAM Documentation](https://gtsam.org/) — Factor Graph 최적화 라이브러리. LIO-SAM 같은 SLAM 시스템이 백엔드로 쓴다
> - [Frank Dellaert — Factor Graphs for Perception and Action (MIT Robotics)](https://www.youtube.com/watch?v=-yCC7mpgL4w) — GTSAM 개발자가 직접 설명하는 Factor Graph
> - [김기섭 블로그 — Scan Context-based LiDAR Pose-graph SLAM 구현](https://gisbi-kim.github.io/blog/2021/05/17/sclidarslam.html) — Scan Context를 LiDAR SLAM에 통합한 구현 해설

#### 14.4.4 FAST-LIO / FAST-LIO2

LIO-SAM이 비선형 최소자승 기반의 Factor Graph를 푼다면, **FAST-LIO**(Fast LiDAR-Inertial Odometry)는 반복 확장 칼만 필터(Iterated EKF, IEKF)를 택한다. 사실 IEKF의 반복 갱신은 사후 최대(MAP) 목적함수를 Gauss-Newton법으로 푸는 것과 수학적으로 동등하다. 그런데도 FAST-LIO가 압도적으로 빠른 비결은 Kalman gain 공식을 재구성한 데 있다. 표준 Kalman gain은 측정 차원이 커질수록 역행렬 계산이 무거워져 수천 개의 LiDAR 점을 한꺼번에 넣기 어렵지만, FAST-LIO는 행렬 항등식을 이용해 계산량이 측정 차원이 아닌 상태 차원(18~24차원 내외)에만 비례하도록 바꿨다. 게다가 과거 상태를 슬라이딩 윈도로 들고 다니지 않으므로 메모리와 연산 부담이 한층 줄어든다. 기존 KD-트리는 맵에 새 점을 넣을 때마다 트리를 통째로 다시 지었지만, 후속작 **FAST-LIO2**는 이를 개선해 점을 하나씩 동적으로 삽입·삭제할 수 있는 **ikd-Tree**를 도입했다. 덕분에 특징점 추출 과정 없이 원시 포인트 클라우드를 그대로 등록하면서도 높은 실시간성을 유지한다.

> **추천 자료**
> - [Xu & Zhang, "FAST-LIO: A Fast, Robust LiDAR-Inertial Odometry Package by Tightly-Coupled Iterated Kalman Filter" (2021)](https://arxiv.org/abs/2010.08196) — FAST-LIO 논문
> - [Xu et al., "FAST-LIO2: Fast Direct LiDAR-Inertial Odometry" (2022)](https://arxiv.org/abs/2107.06829) — FAST-LIO2 논문
> - [FAST-LIO2 GitHub](https://github.com/hku-mars/FAST_LIO) — 공식 코드

### 14.5 Multi-sensor Fusion

지금까지 살펴본 개별 센서는 저마다 뚜렷한 약점을 안고 있다. 카메라는 어둠과 역광에 취약하고, LiDAR는 비나 안개, 유리창 앞에서 산란·투과 오류를 내며, IMU는 혼자 적분하면 몇 초 만에 드리프트가 걷잡을 수 없이 커진다. 여러 센서를 결합(fusion)하면 한 센서가 눈을 감는 순간 다른 센서가 공백을 메워 시스템 전체의 강인함과 정확도를 함께 끌어올릴 수 있다.

#### 14.5.1 Camera + IMU (VIO)

카메라와 IMU를 묶는 전략은 크게 두 가지로 갈린다. **Loosely-coupled** 방식에서는 시각 추정기와 관성 추정기가 각자 독립적으로 상태를 계산한 뒤, 마지막에 공분산 가중 평균 등으로 결과를 합친다. 모듈을 조립하기는 쉽지만 두 센서의 오차 사이에 얽힌 상관관계를 버리기 때문에 정밀도에 한계가 있다. 반면 **Tightly-coupled** 방식에서는 카메라 특징점의 재투영 오차와 IMU 측정 잔차가 동일한 상태 변수를 직접 제약한다. 구현은 까다롭지만 정보를 남김없이 활용하므로 정확도가 높다. 예컨대 VINS-Mono는 두 오차를 하나의 비선형 비용 함수에 묶어 최적화하고, 필터 계열인 MSCKF는 슬라이딩 윈도 EKF 안에서 특징점 변수를 null-space 투영으로 소거해 가벼운 연산으로 밀결합을 이룬다.

#### 14.5.2 LiDAR + IMU (LIO)

회전형 LiDAR는 10~20 Hz 주기로 한 바퀴를 돌며 주변 점을 순차적으로 찍는다. 만약 스캔이 한 바퀴 도는 0.05~0.1초 사이에 로봇이 빠르게 이동하거나 회전하면, 스캔 초반에 찍힌 점과 후반에 찍힌 점이 서로 다른 로봇 포즈에서 관측되어 직선 벽이 휘어져 보이는 **모션 왜곡(motion distortion)**이 일어난다. LIO는 LiDAR보다 훨씬 주기가 빠른 IMU 측정과 점별 타임스탬프를 이용해 스캔 도중의 로봇 궤적을 보간하고, 모든 점을 기준 시점으로 되돌려 왜곡을 편 뒤(de-skewing) LiDAR 관측을 상태 추정에 결합한다. 이 보정 효과가 얼마나 두드러지는지는 센서 간 시간 동기화 정밀도, IMU 바이어스 안정성, 움직임의 격렬함, 스캔 패턴에 달려 있다.

#### 14.5.3 Camera + LiDAR + IMU

LiDAR는 기하 구조를 정밀하게 재고, 카메라는 색상과 풍부한 질감을 담으며, IMU는 빠른 회전과 가속을 놓치지 않는다. **R3LIVE**나 **LVI-SAM**처럼 이 세 센서를 모두 결합한 3센서 통합 시스템도 널리 연구된다. 세 센서가 서로의 사각지대를 보완하므로, 이런 시스템은 예컨대 한쪽 센서가 잠시 가려지거나 퇴화(degenerate) 환경에 들어가도 추정이 끊기지 않으며, R3LIVE처럼 위치를 추정하는 동시에 실제 색상이 입혀진 조밀한 3D 맵을 실시간으로 생성해 내기까지 한다.

> **추천 자료**
> - [KITTI Odometry Benchmark](https://www.cvlibs.net/datasets/kitti/eval_odometry.php) — LiDAR/Visual Odometry 벤치마크의 표준
> - [EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) — VIO 벤치마크 데이터셋
> - [Lin & Zhang, "R3LIVE: A Robust, Real-time, RGB-colored, LiDAR-Inertial-Visual tightly-coupled state Estimation and mapping package" (2022)](https://arxiv.org/abs/2109.07982) — 3센서 융합의 대표작
> - [김기섭 블로그 — Filter-based VIO: MSCKF 계열 history 정리](https://web.archive.org/web/20240820015803/https://gisbi-kim.github.io/blog/2021/04/27/msckf-history.html) — MSCKF 원본부터 stereo 확장까지 후속 연구 정리

### 14.6 Loop Closure & Global Optimization

아무리 정교한 오도메트리와 다중 센서 융합을 사용해도, 로봇이 오래 주행하다 보면 미세한 오차가 쌓여 지도가 조금씩 뒤틀린다. 큰 원을 그리며 출발점으로 되돌아왔는데도 지도 위에서는 시작점과 끝점이 몇 미터씩 어긋나 있는 일이 흔하다. **Loop Closure**는 "이곳은 아까 지나갔던 장소다"라고 알아본 뒤, 그 연결 고리를 이용해 그동안 누적된 드리프트를 궤적 전체에 걸쳐 한꺼번에 바로잡는 과정이다. 넓은 환경에서 일관된 지도를 만들려면 반드시 거쳐야 하는 단계다.

#### 14.6.1 Place Recognition

이전에 방문한 장소를 다시 알아보는 일(Place Recognition)은 생각보다 까다롭다. 같은 복도라도 시간, 조명, 계절, 주차된 차량이 달라지면 전혀 딴판으로 보이기 때문이다. 거꾸로 겉모습이 비슷한 서로 다른 두 방을 같은 장소로 오판하면(false positive), 잘못된 루프 제약이 그래프 전체를 엉뚱한 방향으로 잡아당겨 멀쩡하던 지도까지 순식간에 망가진다. 이 때문에 Place Recognition에서는 놓치지 않는 것(recall)만큼이나 오탐을 내지 않는 **정밀도(precision)**가 결정적이다.

전통적인 시각 기반 방법인 **Bag of Words (BoW)**는 이미지에서 뽑은 특징점을 미리 학습해 둔 시각 단어(visual vocabulary)로 치환한 뒤 단어 빈도 히스토그램의 유사도를 잰다. ORB-SLAM이 채택한 DBoW2 라이브러리가 대표적인데, 속도가 빠르고 가볍지만 조명이나 카메라 시점이 크게 바뀌면 매칭률이 떨어진다. 이를 극복하기 위해 등장한 **NetVLAD**는 합성곱 신경망을 end-to-end로 학습해 날씨와 조명이 바뀌어도 잘 무너지지 않는 이미지 전역 기술자(global descriptor)를 만든다(14.14절에서 자세히 다룬다). 한편 LiDAR 영역에서는 3D 포인트 클라우드를 위에서 내려다본(bird's-eye view) 2D 격자 기술자로 압축하는 **Scan Context**와, 원시 점 집합에서 직접 전역 임베딩을 학습하는 **PointNetVLAD** 등이 활발히 쓰인다.

#### 14.6.2 Pose Graph Optimization

장소 인식으로 루프가 확인되면 이제 뒤틀린 궤적을 실제로 펴 주어야 한다. 이를 위해 로봇의 전체 주행 이력을 다음과 같은 그래프 문제로 옮긴다.

```
노드: 로봇 포즈
에지: 상대 변환 (odometry, loop closure)

목표: 모든 에지 제약을 만족하는 노드 위치 찾기
```

오도메트리만으로 이어 붙인 경로는 이웃한 포즈 사이의 상대 변환은 그럴듯하지만 멀리 떨어질수록 오차가 불어난다. 여기에 루프 폐쇄가 만들어 준 "이 노드와 저 노드는 사실 같은 위치다"라는 에지가 추가되면, **Pose Graph Optimization**은 비선형 최소자승 문제를 풀어 모든 에지의 오차가 궤적 전체에 고르게 분산되도록 노드들의 포즈를 한꺼번에 조정한다.

이 그래프 최적화를 실제로 푸는 데는 주로 세 가지 오픈소스 라이브러리가 쓰인다. 경량 pose graph와 번들 조정에 특화된 **g2o**(ORB-SLAM에서 사용), factor graph 모델링과 증분 최적화(iSAM2)에 강한 **GTSAM**(LIO-SAM에서 사용), 그리고 Google이 개발한 범용 비선형 최소자승 솔버인 **Ceres Solver**다(구체적인 선택 기준은 14.9.4절 비교표에서 정리한다).

> **추천 자료**
> - [GTSAM Documentation & Tutorials](https://gtsam.org/) — Factor Graph 기반 최적화 라이브러리. Pose Graph Optimization 예제 포함
> - [Cyrill Stachniss — Graph-based SLAM](https://www.youtube.com/watch?v=uHbRKvD8TWg) — Pose Graph Optimization의 직관적 설명
> - [g2o GitHub](https://github.com/RainerKuemmerle/g2o) — Graph Optimization 프레임워크
> - [정진용 블로그 — Robust Graph SLAM](https://jinyongjeong.github.io/2017/03/04/lec15_Robust_Graph_SLAM/) — M-estimator, Max-mixture, DCS 등 robust SLAM 기법 한글 해설

> **실습**: [Pose Graph Optimization](https://alexjunholee.github.io/robotics-practice/app.html#pose_graph)
> Pose Graph의 노드(포즈)와 에지(제약)를 조작하고, loop closure 추가 시 전체 경로가 어떻게 보정되는지 확인할 수 있다.

### 14.7 Localization

지금까지는 미지의 환경에서 지도를 그리면서 동시에 위치를 찾는 SLAM을 다루었다. 반면 **Localization**은 이미 완성된 지도가 손에 있을 때, 그 지도 안에서 로봇의 현재 위치만 추정하는 문제다. 실제 서비스 로봇이나 물류 AGV 현장에서는 먼저 오프라인 SLAM으로 정밀한 기준 지도를 만들어 둔 뒤, 운용 중에는 계산이 가벼운 Localization만 돌리는 방식을 흔히 쓴다. 물론 공장 배치가 바뀌는 등 환경이 크게 변하면 지도를 갱신해 주어야 한다. (이 절에서 쓰는 MCL과 EKF의 수학적 기초는 각각 §3.11 비모수 필터와 §3.10에, IMU를 결합한 상태 추정은 §14.10에 정리되어 있다.)

2D 평면 지도에서 가장 널리 쓰이는 위치 추정 기법은 입자 필터 기반의 **Monte Carlo Localization (MCL)**으로, ROS의 `amcl` 패키지 등이 이를 구현한다. MCL은 2D LiDAR와 점유 격자 지도를 바탕으로 지도 위에 수천 개의 "가상 로봇(파티클)"을 흩뿌린다. 각 파티클은 "로봇이 지금 이 좌표에서 이 방향을 바라보고 있다"는 하나의 가설을 뜻한다. 로봇이 움직이고 새 센서 측정이 들어올 때마다 실제 관측과 잘 맞아떨어지는 파티클은 높은 가중치를 받아 살아남고, 벽을 뚫고 있거나 엉뚱한 거리를 예측한 파티클은 도태된다. 이 과정을 몇 차례 되풀이하면 흩어져 있던 파티클들이 자연스럽게 로봇의 실제 위치 부근으로 수렴한다. 3D 포인트 클라우드 환경에서는 이와 달리 현재 LiDAR 스캔을 미리 구축해 둔 3D 맵에 ICP나 NDT 알고리즘으로 직접 정합하는 LiDAR Localization을 주로 쓴다.

> **추천 자료**
> - [Cyrill Stachniss — Monte Carlo Localization](https://www.youtube.com/watch?v=MsYlueVDLI0) — MCL/입자 필터의 직관적 설명
> - [ROS Navigation Stack — AMCL](https://wiki.ros.org/amcl) — ROS 1에서 MCL 사용하기

> **실습**: [Particle Filter](https://alexjunholee.github.io/robotics-practice/app.html#particle_filter)
> 입자 필터 기반 로봇 위치 추정 과정을 시각화하며, 파티클의 수렴 과정을 인터랙티브하게 확인할 수 있다.

> **실습**: [Occupancy Grid](https://alexjunholee.github.io/robotics-practice/app.html#occupancy_grid)
> 2D 점유 격자 지도를 구축하는 과정을 시각화하며, 센서 측정이 어떻게 확률적 지도로 변환되는지 확인할 수 있다.

#### 14.7.1 위치 추정 문제 분류

같은 위치 추정이라도 초기 상태를 얼마나 아는지, 주변 환경이 얼마나 변하는지에 따라 문제의 성격이 크게 달라진다. 위치 추정의 난이도는 다음 네 축으로 나누어 볼 수 있다.

| 축 | 옵션 | 비고 |
|---|---|---|
| 사전지식 | position tracking → global localization → kidnapped robot | 난이도 상승 |
| 환경 | static (로봇만 이동) → dynamic (사람·문·조명) | 동적일수록 어려움 |
| 능동성 | passive (관찰만) → active (탐색 행동 선택) | active가 더 빠른 수렴 |
| 로봇 수 | single → multi (상호 관측으로 belief 공유) | multi는 정보 풍부 |

실전에서는 사전지식의 수준에 따라 세 가지 대표 시나리오를 구분하며, 특히 kidnapped robot은 global localization을 한 단계 더 꼬아 놓은 난제다.

**Position tracking**은 출발 자세를 이미 알고 시작하는 상황이다. 초기 불확실성이 작아 belief가 좁은 단봉(unimodal) 가우시안 분포로 유지되므로 EKF Localization만으로도 충분히 잘 추적한다. 반면 **Global localization**은 로봇이 지도의 어디에 있는지 전혀 모르는 상태에서 시작한다. 이때 belief는 균등 분포에서 출발해 센서 측정이 쌓이면서 여러 후보를 거쳐 결국 한 지점으로 수렴해야 한다. 따라서 봉우리가 여럿인 다봉(multi-modal) belief를 다룰 수 있는 Grid Localization이나 MCL이 필요하다. 가장 까다로운 **Kidnapped robot**은 멀쩡히 운용 중이던 로봇을 누군가 들어 올려 엉뚱한 곳에 내려놓는 상황이다. 로봇은 자신이 옮겨졌다는 사실을 모른 채 기존의 좁은 belief를 그대로 믿고 버티기 때문에, 처음부터 아무것도 모르고 시작하는 global localization보다 복구가 훨씬 어렵다. Thrun 등은 알고리즘이 이 위기에서 얼마나 잘 빠져나오는지를 localization 기법의 핵심 시험대로 삼았다.

현장의 물류 창고 AGV나 가정용 청소 로봇도 전원을 켜고 처음 자리를 잡을 때는 global localization을 풀고, 일단 위치를 찾은 뒤 주행할 때는 position tracking으로 전환한다. ROS Nav2의 `recovery_alpha_slow/fast` 파라미터 역시 운용 중 갑자기 위치를 잃는 납치 상황에 대비하기 위한 장치다.

#### 14.7.2 Markov Localization

**Markov localization**은 구체적인 위치 추정 알고리즘을 하나의 원리로 묶어 주는 틀이다. 이는 3장(§3.9)에서 다룬 베이즈 필터를 위치 추정 문제에 그대로 적용한 프레임워크다. 뒤이어 나올 EKF Localization, Grid Localization, MCL은 모두 같은 베이즈 필터에서 확률 분포(belief)를 무엇으로 근사하는지만 다를 뿐이다.

일반적인 베이즈 필터와 다른 점은 단 하나, 운동 모델과 관측 모델에 미리 주어진 지도 $m$이 조건으로 함께 들어간다는 사실이다. 시간 단계마다 예측(motion update)과 보정(measurement update)을 번갈아 수행하는 구조는 다음과 같다.

```
Markov_localization(bel(x_{t-1}), u_t, z_t, m):
  for all x_t do
    bel̄(x_t) = ∫ p(x_t | u_t, x_{t-1}, m) bel(x_{t-1}) dx_{t-1}   // motion update
    bel(x_t) = η p(z_t | x_t, m) bel̄(x_t)                          // measurement update
  endfor
  return bel(x_t)
```

시나리오에 따라 출발점의 초기 분포 $\text{bel}(x_0)$는 다르게 설정한다.
- Position tracking: $\text{bel}(x_0) = \mathcal{N}(x_0;\, \bar{x}_0, \Sigma)$ — 알려진 초기 위치 주변의 좁은 Gaussian
- Global localization: $\text{bel}(x_0) = 1/|X|$ — 로봇이 존재할 수 있는 모든 자세에 균등 분포
- Partial knowledge: 알려진 구역 근방에만 균등 분포를 두고 그 외 영역은 0으로 설정

이어지는 §14.7.3~§14.7.7의 알고리즘들은 위 의사코드의 $\text{bel}$을 가우시안, 가우시안 혼합, 이산 격자, 입자 집합 중 무엇으로 표현하느냐에 따라 갈린다.

#### 14.7.3 EKF Localization

**EKF Localization**은 Markov localization의 belief를 단봉 가우시안 $(\mu_t, \Sigma_t)$ 하나로 나타내는 방법이다(EKF 자체의 유도는 §3.10.2 참조). 단일 가우시안은 위치 가설을 하나만 품을 수 있으므로 position tracking에 적합하며, 여러 후보가 공존하는 global localization이나 kidnapped 상황에서는 MHT, Grid Localization, MCL을 써야 한다.

여기서는 지도 $m$이 점 랜드마크 집합으로 구성되어 있고, 관측한 랜드마크가 지도의 몇 번째 랜드마크인지 대응 $c_t^i$를 이미 알고 있다고 가정하자. 예를 들어 ARTag, QR 코드, 혹은 에펠탑처럼 고유한 식별자(signature)가 붙어 있어 혼동할 여지가 없는 경우다. 각 측정은 거리·방위각·식별자의 3차원 벡터 $z_t^i = (r, \phi, s)^T$로 들어온다.

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

측정이 서로 조건부 독립이라면($p(z_t | x_t, m) = \prod_i p(z_t^i | x_t, m)$) 한 프레임의 측정들을 한 벡터로 쌓아 한 번에 갱신해도 되고, 위 코드처럼 랜드마크를 하나씩 돌며 순차적으로 갱신해도 된다. 다만 모델이 비선형이기 때문에 측정마다 바뀐 $\mu_t$에서 Jacobian을 다시 계산하느냐에 따라 일괄 갱신 결과와 미세한 차이가 생긴다.

*Probabilistic Robotics*에서는 경험칙(rule of thumb)으로, 방향각(heading) $\theta$의 표준편차가 ±20°를 넘어가면 선형화 오차가 급격히 커져 EKF와 MHT 모두 실패하기 쉽다고 지적한다. 물론 이는 절대적인 기준은 아니며, 실제 환경에서는 관측 기하와 움직임, 노이즈 특성에 따라 NIS·NEES 지표나 몬테카를로 시뮬레이션으로 필터의 일관성을 검증해야 한다. 그럼에도 ARTag·AprilTag처럼 식별 가능한 마커를 쓰거나 GNSS와 IMU를 결합하는 등 belief가 단봉으로 유지되는 문제에서는 지금도 EKF localization이 탄탄한 선택지다.

**미지 대응(unknown correspondences)**: 현실에서는 관측한 랜드마크의 정체 $c_t^i$를 미리 알 수 없는 상황이 훨씬 흔하다. 이럴 때 가장 기본적인 전략은 최대 우도(ML) 데이터 연관이다. 예측 관측 $\hat{z}_t^k$와 실제 측정 $z_t^i$ 사이의 불확실성까지 반영한 **마할라노비스 거리(Mahalanobis distance)**를 구한 뒤, 그 거리가 가장 작은 지도 랜드마크를 고른다.

$$j(i) = \arg\min_k (z_t^i - \hat{z}_t^k)^T \Psi_k^{-1} (z_t^i - \hat{z}_t^k), \quad \Psi_k = H_t^k \bar\Sigma_t H_t^{k,T} + Q_t$$

여기서 $\Psi_k$는 혁신 공분산(innovation covariance)으로, 로봇 포즈의 불확실성과 센서 측정 노이즈를 모두 합친 값이다. 마할라노비스 거리 최소화는 후보마다 공분산 행렬식과 사전확률이 같을 때 가우시안 로그 우도 최대화와 일치한다. 실무에서는 엉뚱한 랜드마크와 억지로 짝지어지는 일을 막기 위해 측정 차원에 맞춘 $\chi^2$ gate로 임계값 밖의 후보를 거르고, 한 프레임 안에서 랜드마크 하나에 측정 하나만 붙도록 일대일 할당(one-to-one assignment) 제약을 함께 건다. 참고로 ORB-SLAM 역시 기술자(descriptor) 매칭으로 후보를 추린 뒤 기하학적 검증으로 outlier를 걸러내지만, 이산 특징점 매칭과 RANSAC 기반이라는 점에서 EKF의 ML association과는 결이 다르다.

#### 14.7.4 Multi-Hypothesis Tracking (MHT)

EKF는 단봉 가우시안으로 단 하나의 가설만 품지만, **MHT**는 가우시안 여러 개가 섞인 가우시안 혼합(Gaussian mixture)으로 belief를 표현해 여러 위치 가설을 동시에 유지한다.

가설 $h$마다 독립적인 EKF가 하나씩 돌아가는 셈이다. 새로운 측정이 들어올 때마다 가능한 데이터 연관에 따라 가설이 분기되어 늘어나고, 사후확률 가중치가 임계값 $\psi_{\min}$ 아래로 떨어진 가설은 가지치기(pruning)로 버린다. 이렇게 가설을 쳐내지 않으면 시간이 지날수록 후보가 기하급수적으로 불어나 계산을 감당할 수 없기 때문이다.

참고로 자율주행의 다중 객체 추적(MOT)에서도 Mahalanobis gating과 Hungarian assignment를 자주 함께 쓰지만, 이 조합은 프레임마다 가장 그럴듯한 단 하나의 매칭을 확정 짓는 방식이다. 여러 연관 가설을 시간 축을 따라 계속 살려 두고 추후 관측으로 판가름하는 MHT와는 구조적으로 다르다.

#### 14.7.5 Grid Localization

가우시안 기반 방법에서는 연속 공간 위의 분포를 가정하지만, **Grid Localization**은 포즈 공간 전체를 잘게 쪼갠 이산 격자(히스토그램)의 셀마다 확률 값을 저장한다. 덕분에 global localization처럼 넓게 퍼진 분포나 다봉 belief를 아무런 형태 제약 없이 그대로 표현할 수 있으며, 계산 비용은 전체 격자 셀 수 $K$에 비례한다.

```
Grid_localization({p_{k,t-1}}, u_t, z_t, m):
  for all k do
    p̄_{k,t} = Σ_i p_{i,t-1} · motion_model(mean(x_k), u_t, mean(x_i))
    p_{k,t}  = η · measurement_model(z_t, mean(x_k), m) · p̄_{k,t}
  endfor
  return {p_{k,t}}
```

여기서 $\text{bel}(x_t) = \{p_{k,t}\}$는 각 셀 $x_k$가 차지하는 확률이며 모든 셀의 합은 1이다.

**해상도 트레이드오프**: 격자를 촘촘하게 나눌수록 공간을 이산화하며 생기는 양자화 오차는 줄어들지만, 모든 셀을 훑어야 하는 global localization의 CPU 연산량은 기하급수적으로 늘어난다. 이 부담을 줄이고 실시간성을 확보하기 위해 실무에서는 레이캐스팅 결과를 미리 계산해 캐시에 담아 두거나, 스캔 빔을 일정 간격으로 솎아 쓰고, 확률이 임계값을 넘는 유망한 셀만 골라 갱신하는 기법을 쓴다.

Grid Localization은 공간 전체를 고정 격자로 쪼개어 global belief를 표현한다. 만약 이 고정 격자 대신 확률 밀도에 맞춰 동적으로 흩뿌린 샘플 집합으로 belief를 나타내면 곧 **MCL**이 된다. ROS의 `amcl` 노드는 이 방식에 KLD-sampling을 적용한 적응형 MCL 구현으로, 분포의 넓이에 따라 필요한 샘플 수를 자동으로 조절한다.

#### 14.7.6 MCL 알고리즘 (보강)

입자 필터의 확률적 원리와 수학적 유도는 §3.11에 자세히 나와 있다. 이를 지도 기반 위치 추정 알고리즘으로 옮겨 놓은 MCL의 기본 골격은 다음과 같다.

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

주기마다 MCL은 입자를 운동 모델로 퍼뜨리고(predict/sample), 지도와 대조해 가중치를 매긴 뒤(weight), 가중치에 비례해 입자를 다시 뽑는(resample) 세 단계를 반복한다. 처음 시작할 때 global localization이라면 빈 공간 전체의 균등 분포에서 샘플 $M$개를 뿌리고, position tracking이라면 초기 추정치 주변의 좁은 가우시안에서 샘플을 뽑는다.

**계산 자원에 맞추기**: 입자 수 $M$을 고정된 상수로 묶지 않고 유연하게 두어, 다음 센서 측정이 도착하기 전까지 하드웨어가 처리할 수 있는 만큼만 샘플을 뽑도록 하면 CPU 성능이 여유로울수록 더 많은 입자를 굴려 정밀도를 높일 수 있다.

다만 기본 MCL에는 한 가지 구조적 약점이 있다. 입자를 오직 motion model에서만 뽑기 때문에, 레이저 스캐너처럼 센서가 아주 정밀해 측정 우도(likelihood)가 바늘처럼 뾰족한 경우에는 정작 진짜 위치에 들어맞는 입자가 거의 없어 대부분 가중치가 0에 수렴해 버린다. 이 고갈 문제는 §14.7.8의 Mixture MCL이 해결한다.

현재 ROS2 Nav2의 `nav2_amcl` 패키지는 이 기본 MCL 골격을 바탕으로, 분포 크기에 맞춰 입자 수를 조절하는 KLD-sampling(§14.7.5)과 무작위 입자 주입 기법(§14.7.7)을 결합해 구현되어 있다.

#### 14.7.7 Augmented MCL — 납치 복구

표준 MCL은 일단 입자들이 한 지점으로 좁게 모이고 나면 로봇 납치(kidnapped) 상황에 속수무책이다. 로봇이 다른 방으로 옮겨졌을 때 새 위치 근처에는 살아남은 입자가 단 하나도 없어 아무리 재샘플링을 돌려도 위치를 되찾을 길이 없기 때문이다.

**Augmented MCL**은 이 막다른 골목을 뚫기 위해 센서 측정 우도의 추이를 감시한다. 측정 우도의 최근 단기 평균이 평소의 장기 평균에 비해 갑자기 뚝 떨어졌다면, 현재 파티클들이 현실과 어긋나 길을 잃었다는 신호다. 이때 필터는 그 비율에 맞춰 일정 확률로 지도 전체에 무작위 입자를 새로 주입해 탈출구를 열어 준다.

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

두 지수이동평균의 반응 속도를 가르기 위해 계수는 $0 \le \alpha_{\text{slow}} \ll \alpha_{\text{fast}}$ 조건(예: $\alpha_{\text{slow}} = 0.001$, $\alpha_{\text{fast}} = 0.1$)을 만족해야 하며, 무작위 입자 주입 확률은 두 평균의 비율로 정해진다.

$$p_{\text{inject}} = \max\!\left(0,\, 1 - \frac{w_{\text{fast}}}{w_{\text{slow}}}\right)$$

평소 정상 주행 중에는 $w_{\text{fast}} \approx w_{\text{slow}}$가 유지되어 비율이 1에 가깝고 주입 확률은 사실상 0이므로 표준 MCL과 똑같이 동작한다. 하지만 로봇이 납치된 직후에는 모든 입자의 측정 우도가 바닥을 치면서 반응이 빠른 $w_{\text{fast}}$가 먼저 급락하고, 그 결과 주입 확률이 치솟아 새 위치에 입자가 뿌려질 기회가 생긴다. 이후 새 위치로 수렴하거나 장기 평균 $w_{\text{slow}}$가 천천히 내려와 비율이 다시 1에 가까워지면 무작위 주입은 자연스레 멎는다.

물론 센서 노이즈가 잠깐 튀어도 $w_{\text{fast}}$가 출렁이며 불필요한 입자 주입을 부를 수 있다. 기준선 역할을 하는 $w_{\text{slow}}$가 묵직하게 버티는 상태에서, 일시적 잡음에 따른 오탐을 줄이려면 $\alpha_{\text{fast}}$ 값을 현장 센서의 잡음 수준에 맞춰 조율해야 한다.

§14.7.1에서 언급한 ROS Nav2의 `recovery_alpha_slow/fast` 파라미터가 바로 이 적응형 random-pose 주입을 제어한다. 최신 연구에서는 아무 곳에나 무작위 포즈를 뿌리는 대신, NetVLAD 같은 place recognition과 PnP로 추정한 후보 포즈를 입자 풀에 섞어 복구 속도를 높이는 방식도 쓰인다. 다만 기본 AMCL에는 들어 있지 않은 기능이므로 실제 운용 환경에서 별도로 검증해야 한다.

#### 14.7.8 Mixture MCL

Augmented MCL이 지도 전체에 눈을 감고 무작위 포즈를 흩뿌린다면, **Mixture MCL**은 아예 입자를 뽑는 제안(proposal) 분포 자체를 섞어, 일부 입자는 센서 측정 모델에서 직접 추출한다.

$$x_t^{[k]} \sim \begin{cases} p(z_t | x_t, m) & \text{확률 } \rho \\ \text{sample\_motion\_model}(u_t, x_{t-1}^{[k]}) & \text{확률 } 1 - \rho \end{cases}$$

측정 분포에서 바로 뽑아낸 입자들은 현재 센서 관측과 들어맞는 위치에 곧장 꽂힌다. 덕분에 Mixture MCL은 납치 복구 문제를 해결할 뿐 아니라, §14.7.6에서 짚었던 "센서 노이즈가 너무 작아 운동 모델 입자들이 전부 0으로 죽어 버리는" 고갈 문제까지 한 번에 풀어낸다. 그만큼 Augmented MCL보다 한 단계 나아간 해법이다. 다만 $p(z_t | x_t, m)$에서 직접 샘플링하려면 현재 센서 스캔을 입력받아 그럴듯한 로봇 포즈를 역으로 뱉어 내는 역방향 센서 모델(inverse sensor model)을 따로 설계해야 하는 부담이 따른다.

#### 14.7.9 동적 환경 필터링

복도를 오가는 사람이나 주차장을 지나는 차량처럼 지도에 없는 움직이는 물체가 많으면, LiDAR 빔 상당수가 원래 벽보다 훨씬 짧은 거리에서 반사된다. 이를 그대로 지도와 대조하면 로봇의 현재 위치가 틀렸다고 오해해 필터가 엉뚱한 곳으로 튈 수 있다. 빔 센서 모델의 4성분 혼합 모델(§2.7.2)에는 이런 예상 밖의 앞선 충돌을 설명하는 $p_{\text{short}}(z | x, m)$ 성분이 들어 있다. 빔 $z_t^k$마다 각 성분의 사후확률을 계산해 short 성분의 비중이 높게 나온 빔을 가중치 합산에서 제외하면, 주변에 사람이 북적이는 환경에서도 MCL이 흔들리지 않고 안정적으로 위치를 지킨다.

#### 14.7.10 필터 비교 정리

지금까지 살펴본 네 가지 위치 추정 필터의 표현 방식과 능력, 계산 비용을 한눈에 비교하면 다음과 같다.

| 알고리즘 | Belief 표현 | Position tracking | Global loc | Kidnapped | 계산 비용 |
|---|---|---|---|---|---|
| EKF Loc | Gaussian (μ, Σ) | 좋음 | 불가 | 불가 | O(N) |
| MHT | Gaussian 혼합 | 좋음 | 제한 | 제한 | O(H·N) |
| Grid Loc | 히스토그램 | 좋음 | 가능 | 가능 | O(K) |
| MCL | 파티클 집합 | 좋음 | 가능 | Augmented MCL로 가능 | O(M) |

표에서 N은 랜드마크 수, H는 가설 수, K는 격자 셀 수, M은 파티클 수다. 출발 자세를 확실히 알고 있는 position tracking이라면 가우시안 단봉 가정으로도 충분한 EKF가 가장 간결하며, 전역 탐색이나 납치 복구가 필요하다면 셀 수 K나 입자 수 M을 조절해 연산량과 정확도 사이의 균형을 맞출 수 있는 Grid Localization과 MCL이 유리하다.

#### 14.7.11 실무 고려: 랜드마크 효율·Negative Information

실제 시스템에 위치 추정 필터를 올릴 때는 알고리즘 골격 외에 다음 네 가지 구현 디테일이 안정성과 속도를 좌우한다.

- **랜드마크 빨리 찾기**: 지도에 랜드마크가 N개 등록되어 있을 때 매 관측마다 전체를 훑으면 $O(N)$ 시간이 걸린다. 저차원 공간에서 균형 잡힌 KD-tree를 구축하면 평균 $O(\log N)$으로 줄일 수 있지만(최악의 경우 $O(N)$), 아예 지도를 공간 격자로 색인(grid indexing)해 두면 탐색 비용이 반경 내 셀에 담긴 소수의 랜드마크 수에만 비례하도록 고정할 수 있다.
- **Mutual exclusion**: 한 프레임에서 지도의 랜드마크 하나에는 실제 측정 하나만 대응해야 물리적으로 말이 된다. 하지만 기본 ML data association은 측정마다 독립적으로 가장 가까운 랜드마크를 고르기 때문에 서로 다른 두 측정이 같은 랜드마크에 들러붙는 일이 생긴다. 이런 중복 할당이 일어나면 마할라노비스 거리가 더 작은 쪽만 남기고 나머지는 배제하는 상호 배제 제약을 따로 걸어야 한다.
- **Outlier rejection**: 센서가 반사광이나 동적 장애물 때문에 엉뚱한 값을 내놓았을 때, 마할라노비스 거리가 $\chi^2_{95\%}$ 임계값을 넘는 측정을 단호히 버린다. 이 간단한 게이팅 한 줄만 넣어도 EKF가 잘못된 측정 하나에 휘둘려 발산하는 사고를 크게 줄일 수 있다.
- **Negative information**: "로봇이 바라보는 각도 범위 안에 있어야 할 랜드마크가 보이지 않았다"는 사실 역시 유용한 단서다. 가려짐이 없는 상황에서 특정 랜드마크가 관측되지 않았다면 로봇이 그 위치에 있지 않을 확률이 높아지기 때문이다. 다만 가려짐(occlusion) 여부까지 확률적으로 엄밀하게 모델링하기가 까다로워, 실용 시스템 대부분은 구현 단순성을 위해 이 정보를 생략한다.

---

### 14.7B Occupancy Grid Mapping

앞선 §14.7에서는 지도가 *주어졌을 때* 로봇의 위치를 추정했다. 이번에는 문제를 정반대로 뒤집어 보자. **Occupancy Grid Mapping**은 로봇의 자세 궤적을 이미 안다고 가정하고, 공간을 잘게 나눈 격자 셀마다 장애물이 차 있을 확률을 추정해 지도를 *만든다*. 실제 SLAM 시스템에서도 pose graph 최적화로 로봇의 궤적을 먼저 확정한 뒤, 이 절의 알고리즘으로 내비게이션에 쓸 최종 점유 격자 지도를 빚어내는 흐름을 자주 쓴다(기초가 되는 binary Bayes 필터는 §3.11.2 참조).

#### 14.7B.1 도입: 지도 작성의 어려움

지도를 그리는 문제는 위치를 추정하는 문제보다 차원이 훨씬 거대하다. 평면 위 로봇의 자세 $x_t \in \mathbb{R}^3$(x, y, θ)은 겨우 연속 변수 세 개로 끝나지만, 지도 $m$은 수만에서 수백만 개에 이르는 격자 셀의 집합이다. 각 셀이 비었는지(0) 차 있는지(1)만 따져도 가능한 지도의 가짓수가 $2^{|m|}$개에 달하므로, 모든 지도의 확률을 일일이 계산하는 일은 원천적으로 불가능하다.

이 거대한 공간을 다루기 위해 표준 Occupancy Grid Mapping은 두 가지 가정을 도입한다. 첫째, 로봇의 궤적 $x_{1:t}$는 이미 정확히 안다. 둘째, 각 셀의 점유 여부는 서로 조건부 독립이다. 두 번째 가정 덕분에 지도 전체의 사후확률이 셀별 주변 확률(marginal)의 곱으로 깔끔하게 쪼개지며, 결국 수백만 셀마다 독립적인 binary Bayes 필터 하나씩을 돌리는 문제로 단순화된다.

$$p(m \mid z_{1:t}, x_{1:t}) = \prod_i p(m_i \mid z_{1:t}, x_{1:t})$$

*Probabilistic Robotics*는 이 밖에도 지도 작성을 어렵게 만드는 현실적인 난관으로 센서 측정 거리에 비해 환경이 지나치게 넓은 경우, 센서와 구동계의 잡음, 서로 다른 장소가 똑같이 보이는 지각 모호성(perceptual ambiguity), 그리고 지도의 크기를 키우는 순환 경로(cycle)를 꼽는다.

#### 14.7B.2 표준 알고리즘: Log-Odds 누산

셀마다 확률 값을 직접 곱해 나가면 값이 0이나 1 근처에서 부동소수점 언더플로(underflow)에 빠지기 쉽고, 매번 정규화 상수를 계산해야 하는 번거로움이 따른다. 이를 피하기 위해 점유 확률 $p$ 대신 **log-odds** 표현으로 바꾸어 더한다.

$$l_{t,i} = \log \frac{p(m_i \mid z_{1:t}, x_{1:t})}{1 - p(m_i \mid z_{1:t}, x_{1:t})}$$

초기 사전확률에 해당하는 사전 log-odds를 $l_0 = \log[p(m_i) / (1 - p(m_i))]$로 두면, binary Bayes 필터(§3.11.2)로부터 갱신 규칙이 단순한 덧셈식으로 유도된다.

$$l_{t,i} = l_{t-1,i} + \text{inverse\_sensor\_model}(m_i, x_t, z_t) - l_0$$

새로운 스캔에서 레이저가 셀 $m_i$에 부딪힌 것으로 관측되면(hit) log-odds가 올라가고, 레이저가 무사히 통과해 그 셀이 비어 있는 것으로 나타나면(free) log-odds가 내려간다. 식 마지막의 $-l_0$ 항은 매 관측마다 사전확률이 중복해서 더해지지 않도록 빼 주는 보정 항이다.

이를 전체 지도에 적용한 표준 알고리즘의 흐름은 다음과 같다.

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

시각화나 경로 계획에 확률 값이 필요해지면 언제든 $p(m_i | z_{1:t}, x_{1:t}) = 1 - 1/(1 + \exp\{l_{t,i}\})$ 식을 거쳐 0과 1 사이의 확률로 복원할 수 있다.

여기서 핵심 부품인 `inverse_sensor_model`은 거리 센서(range finder)의 경우 기하학적으로 매우 직관적이다. 로봇 위치 $x_t$에서 각 셀 중심까지의 거리 $r$과 방위각 $\phi$를 구한 뒤, 가장 가까운 레이저 빔과 비교해 충돌 지점(hit)이면 높은 log-odds를, 빔이 통과한 빈 구간(free)이면 낮은 log-odds를 돌려준다.

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

위 코드에서 $\alpha$는 벽이나 장애물의 물리적 두께, $\beta$는 단일 빔이 퍼지는 각도 폭을 뜻한다. 참고로 실제로 널리 쓰이는 시스템들은 이 원리를 조금씩 변형해 속도와 용도에 맞춘다. Cartographer의 submap probability grid는 log-odds 덧셈 대신 odds 곱을 미리 계산한 lookup table로 속도를 끌어올린다. ROS Nav2의 `costmap_2d`는 확률 대신 0~254의 정수 비용(cost)을 두고 raytracing으로 marking/clearing을 처리한다. SLAM Toolbox(Karto 계열)는 셀별로 관측(visit) 횟수 대비 충돌(hit) 횟수 비율에 임계값을 걸어 지도를 갱신한다.

#### 14.7B.3 다중 센서 융합

로봇에 카메라, LiDAR, 소나, 적외선 센서가 함께 달려 있다면 센서마다 물리적 특성과 `inverse_sensor_model`이 다르다. 여러 센서에서 얻은 정보를 하나의 점유 지도로 합칠 때는 셀마다 각 센서 모델의 최댓값을 취하는 방식(conservative max)이 가장 보수적이다. 센서 중 단 하나라도 장애물이 있다고(hit) 보고하면 그 셀은 막혀 있다고 간주하므로 충돌 회피에는 안전하지만, 반대로 센서 오탐 때문에 실제보다 주행 가능 공간이 좁게 잡힐 수 있다.

센서별로 독립적인 지도 레이어를 유지하면서 log-odds를 따로 쌓은 뒤 셀마다 합산하는 방법도 있다. 이때 센서마다 신뢰도나 정밀도가 다르다면 가중치를 달리 주어 더함으로써 훨씬 균형 잡힌 지도를 얻는다.

#### 14.7B.4 inverse_sensor_model 학습

위에서 작성한 `inverse_range_sensor_model`은 사람이 기하 규칙으로 짠 단순한 모델이다. 하지만 초음파 소나의 복잡한 다중 반사 패턴이나 유리벽에 부딪힌 LiDAR의 굴절처럼, 손으로 수식을 적기 까다로운 센서 특성을 다룰 때는 학습에 기대는 편이 낫다.

센서의 정방향 모델 $p(z | x, m)$을 알고 있거나 시뮬레이터가 준비되어 있다면, 수많은 (자세, 측정, 실제 점유 상태) 쌍 $\{(x^{(k)}, z^{(k)}, m_i^{(k)})\}$를 생성한 뒤 이진 분류용 교차 엔트로피(cross-entropy) 손실로 신경망을 학습시켜 역방향 모델을 근사할 수 있다.

$$\mathcal{L} = -\sum_k \left[m_i^{(k)} \log \hat{p}_i + (1 - m_i^{(k)}) \log(1 - \hat{p}_i)\right]$$

이렇게 학습된 함수 근사기는 $(x, z)$를 입력받아 점유 확률 $\hat{p}_i = p(m_i | x, z)$를 출력한다. 이 근사기를 기존의 수작업 `inverse_sensor_model` 자리에 그대로 끼워 넣을 수 있다.

#### 14.7B.5 MAP Occupancy Mapping (심화)

표준 log-odds 알고리즘이 기대는 "셀 간 조건부 독립" 가정에는 미묘한 맹점이 있다. 하나의 레이저나 소나 빔이 퍼져 나가는 원뿔(cone) 안에 놓인 이웃 셀들은 본질적으로 *동일한 하나의 측정*을 증거로 공유하지만, 독립 가정은 이 상관관계를 무시한다. 특히 빔 폭이 넓은 소나 센서에서는 한 측정이 여러 셀에 중복 반영되어 벽이 실제보다 두껍거나 어긋나게 그려지는 현상이 두드러진다.

**MAP Occupancy Mapping**은 이 왜곡을 피하기 위해 역방향 모델을 버리고, 정방향 측정 모델 $p(z_t | x_t, m)$을 그대로 사용해 전체 지도 사후확률을 최대화하는 최빈 지도(mode)를 직접 탐색한다.

$$m^* = \arg\max_m \left[\sum_t \log p(z_t \mid x_t, m) + \log p(m)\right]$$

구체적인 탐색 절차로 언덕 오르기(hill-climbing) 방식을 쓴다. 모든 셀이 비어 있는 백지 지도에서 출발해, 각 셀의 상태를 반전시켰을 때 전체 로그 우도가 커지면 채택하고 아니면 되돌린다.

```
MAP_occupancy_grid_mapping(x_{1:t}, z_{1:t}):
  m ← 모든 셀 free로 초기화
  repeat until convergence:
    for all cells m_i do
      m_i ← argmax_{k ∈ {0,1}} [k·l_0 + Σ_t log measurement_model(z_t, x_t, m | m_i=k)]
  return m
```

이 방식은 센서 원뿔 내의 기하학적 상관관계를 정직하게 반영하지만, 모든 측정 데이터를 한꺼번에 모아야 하는 오프라인 배치 방식이라 실시간 SLAM에 녹이기 어렵고 hill-climbing 특성상 지역 최댓값(local maximum)에 갇히기 쉽다. 또한 결과가 확정된 지도 한 장으로만 나오므로 셀마다 얼마나 불확실한지 확률 정보를 남겨 두지 못한다는 한계도 있다.

#### 14.7B.6 다른 공간 표현과 비교: OctoMap·Voxblox·NeRF·3DGS

2D 격자에서 한 걸음 더 나아가 3D 공간을 표현하는 방식도 목적에 따라 다양하게 분화해 왔다. **OctoMap**은 2D occupancy grid의 log-odds 갱신을 8진 트리(octree) 자료구조로 3D에 그대로 확장해 메모리를 아끼며, **Voxblox**와 GPU 가속 버전인 **nvblox**는 점유 확률 대신 가장 가까운 표면까지의 부호 있는 거리를 담는 TSDF(Truncated Signed Distance Function)를 저장해 정밀한 충돌 회피와 표면 복원에 쓰인다. 최근 주목받는 **NeRF**의 밀도장(density field)과 **3D Gaussian Splatting(3DGS)**의 불투명도(opacity)는 사실적인 새 시점 렌더링을 위해 학습하는 연속 장면 표현인데, 광선(ray)을 따라 값을 누적해 기하를 복원한다는 점에서는 고전적인 occupancy grid와 수학적 맥락이 닿아 있다. 물론 계산이 가볍고 장애물 유무가 명확해야 하는 SLAM Toolbox나 Nav2 costmap 같은 현장 내비게이션 스택에서는 여전히 고전적인 occupancy grid가 현역으로 쓰인다.

> **실습**: [Occupancy Grid](https://alexjunholee.github.io/robotics-practice/app.html#occupancy_grid)
> Log-odds 누산 과정을 셀별로 시각화하며, inverse_sensor_model의 hit/free 영역이 어떻게 지도로 쌓이는지 확인할 수 있다.

---

## 학습 기반 SLAM

### 14.8 Learning-based & Neural SLAM

지금까지 다룬 고전 SLAM 시스템은 사람이 직접 설계한 코너 추출기, 기술자 매칭, 그리고 비선형 최소자승 최적화기가 조립된 파이프라인이었다. 최근 등장한 **학습 기반 SLAM**은 이 수작업 모듈의 일부 또는 전체를 심층 신경망으로 대체해, 고전 방식이 무너지기 쉬운 텍스처 부족이나 급격한 조명 변화까지 돌파하려 한다.

대표 주자인 **DROID-SLAM (2021)**은 이름(Differentiable Recurrent Optimization-Inspired Design)이 말해주듯, 고전 최적화의 반복 구조를 신경망 아키텍처 안으로 끌어들였다. 특징점을 뽑아 일일이 짝짓는 대신, 순환 신경망(GRU)이 두 영상 사이의 조밀한 광류(dense optical flow)를 반복적으로 다듬으면서 카메라 포즈와 픽셀별 깊이를 함께 추정한다. 특히 반복 갱신의 마지막 단계에 미분 가능한 **Dense Bundle Adjustment(DBA)** 레이어를 심어 두어, 기하학적 제약을 지키면서도 전체 네트워크를 end-to-end로 학습시킬 수 있다. 그 결과 DROID-SLAM 논문은 TartanAir, EuRoC, TUM-RGBD, ETH3D-SLAM 등 여러 벤치마크에서 기존 방법보다 오차를 낮췄을 뿐 아니라 추적이 완전히 터지는 치명적 실패(catastrophic failure) 빈도를 극적으로 줄였다고 보고했다. 물론 추론에 GPU 연산 자원이 필수적이고, 하드웨어와 설정에 따라 경량 CPU 기반 SLAM보다 프레임 처리 속도가 느릴 수 있다는 현실적인 트레이드오프가 따른다.

또 다른 흐름은 맵의 표현 자체를 신경망이나 미분 가능한 렌더링 프리미티브로 바꾸는 **3DGS-SLAM 융합**이다. 13.5.2절에서 살펴본 3D Gaussian Splatting(3DGS)을 SLAM의 맵 표현으로 채택한 **SplaTAM**과 **MonoGS**가 대표적이다. 듬성듬성한 점군 대신 수많은 3D 가우시안 타원체로 공간을 채우기 때문에, 주행이 끝난 뒤 지도가 실제 장면과 흡사한 실사급 품질로 복원되며 임의의 가상 시점 영상을 즉시 렌더링해 AR 화면에 덧입히는 등 응용 범위도 넓어진다.

> **추천 자료**
> - [Teed & Deng, "DROID-SLAM: Deep Visual SLAM for Monocular, Stereo, and RGB-D Cameras" (2021)](https://arxiv.org/abs/2108.10869) — DROID-SLAM 논문
> - [Keetha et al., "SplaTAM" (CVPR 2024)](https://arxiv.org/abs/2312.02126) — 3DGS 기반 Dense SLAM
> - [Awesome-SLAM GitHub](https://github.com/SilenceOverflow/Awesome-SLAM) — SLAM 논문/프로젝트 모음

---

## 심화 주제

지금부터는 SLAM을 실제 시스템으로 구현하거나 연구 수준으로 파고들 때 반드시 마주치는 백엔드 최적화의 수학, 관성 센서 사전 적분, 관측가능성 한계, 그리고 의미·다중 로봇·장기 매핑으로 이어지는 심화 주제를 차례로 짚어 본다.

### 14.9 심화: SLAM 백엔드 최적화

프런트엔드가 카메라나 LiDAR 데이터에서 상대적인 이동 제약을 뽑아내면, 이제 백엔드가 바통을 이어받아 흩어진 제약을 한꺼번에 가장 잘 만족하는 전역 상태(로봇 포즈와 랜드마크 위치)를 찾아내야 한다. 전역 상태를 구하는 일은 수학적으로 **비선형 최소자승(nonlinear least squares)** 문제다. g2o, GTSAM, Ceres Solver 같은 최적화 라이브러리의 수많은 설정값을 제대로 다루려면 이 백엔드 수학의 뼈대를 이해하고 있어야 한다(factor graph 이전 세대의 정보형 SLAM인 EKF-SLAM, EIF, SEIF, EM은 §14.16에서 따로 정리한다).

**SLAM 백엔드가 푸는 문제**

복잡해 보이는 SLAM 백엔드도 결국 한 줄로 요약하면 선형 연립방정식 $Ax = b$를 반복해서 푸는 작업이다.

로봇이 주행하며 모으는 제약 데이터는 크게 두 종류다.
1. **Odometry**: "나는 직전 위치에서 1 m 앞으로 갔다" (연속 포즈 간 상대 이동 제약)
2. **관측**: "지금 위치에서 저 랜드마크가 3 m 거리에 보인다" (포즈와 랜드마크 간 관측 제약)

현실의 모든 측정에는 노이즈가 섞여 있으므로, 이 수많은 제약을 오차 0으로 동시에 만족하는 해는 존재하지 않는다. 따라서 백엔드는 모든 측정의 오차 제곱합이 최소가 되는 타협점을 찾는다. 게다가 회전과 투영이 얽힌 비선형 문제이기 때문에 한 번에 해를 구할 수 없고, 현재 추정값 근방에서 식을 선형화(linearize)해 작은 보정치를 구한 뒤 상태를 업데이트하는 과정을 수렴할 때까지 되풀이한다. 이 "선형화 → $Ax = b$ 풀이 → 업데이트 → 반복"의 순환이 바로 **Gauss-Newton** 최적화다.

(참고: [김기섭 블로그 — SLAM back-end 시리즈](https://gisbi-kim.github.io/blog/2021/03/04/slambackend-1.html))

#### 14.9.1 Manifold 위의 Gauss-Newton

일반 유클리드 공간의 최적화에서는 추정치에 보정 벡터를 그대로 더한다(`x ← x + δx`). 하지만 SLAM에서 다루는 로봇의 3D 포즈는 회전과 병진이 묶인 리 군(Lie group) $SE(3)$의 원소다. 만약 $3 \times 3$ 회전 행렬에 임의의 델타 행렬을 그냥 더해 버리면 행렬의 직교 조건과 $\det = 1$ 성질이 즉시 깨져 회전 행렬로서의 자격을 잃는다.

따라서 SLAM 백엔드는 유클리드 덧셈 대신 $SE(3)$ 다양체(manifold) 위에서 상태를 갱신한다. 작은 보정량(perturbation)은 탄젠트 공간에 해당하는 리 대수(Lie algebra) $\mathfrak{se}(3)$의 6차원 벡터 `δξ ∈ R^6`로 정의하고, 이를 지수 사상(exponential map)으로 $SE(3)$ 행렬로 변환한 뒤 기존 포즈에 곱하는 방식(**left perturbation**)을 쓴다.

```
T ← exp(δξ^) · T
```

여기서 `^`(hat 연산자)는 6차원 섭동 벡터 `δξ`를 $4 \times 4$ 행렬로 바꿔 주며, `exp(·)`는 이를 다시 $SE(3)$ 변환 행렬로 매핑한다.

선형화 단계에서는 이 미소 섭동 `δξ`에 대해 오차 함수 `e(T)`가 얼마나 변하는지 나타내는 야코비안(Jacobian)을 구해야 한다.

```
J = ∂e / ∂δξ
```

연쇄 법칙(chain rule)으로 풀어쓰면 `∂e/∂(Tp) · ∂(exp(δξ^)Tp)/∂δξ` 꼴이 된다. 앞 항은 3D 점의 오차 민감도이고, 뒤 항은 좌섭동을 점 `Tp`에 가했을 때의 미분이다. `δξ`를 병진·회전 순으로 배열하면 3차원 점에 대해 `[I, -(Tp)^]`라는 깔끔한 행렬이 나온다(이름이 비슷한 $SE(3)$의 left Jacobian은 BCH 공식에서 리 대수 섭동끼리 잇는 $6 \times 6$ 행렬이므로 이 항과 혼동하지 말아야 한다).

구한 Jacobian `J`와 측정 노이즈 공분산 `Σ`, 현재 잔차 `e`를 모으면 비선형 최소자승의 한 스텝을 푸는 정규 방정식(normal equation)이 완성된다.

```
(J^T Σ^{-1} J) δξ* = -J^T Σ^{-1} e
```
- `Σ`: 센서 측정의 노이즈 공분산 행렬
- `H = J^T Σ^{-1} J`: Hessian의 Gauss-Newton 근사이자 정보 행렬(information matrix)
- 여러 제약 조건이 모이면 각 `J^T Σ^{-1} J` 항을 해당 변수 블록에 더해서 쌓는다

이 정규 방정식을 풀어 최적의 섭동 `δξ*`를 얻고 `T ← exp(δξ*^) · T`로 갱신한 뒤, 다시 새 위치에서 Jacobian을 계산하고 풀기를 오차가 수렴할 때까지 반복한다.

#### 14.9.2 Schur Complement (Marginalization)

Visual SLAM의 핵심 최적화인 **Bundle Adjustment(BA)**에서는 카메라 포즈(`p`)와 3D 랜드마크(`l`)를 동시에 최적화한다. 이때 정규 방정식의 Hessian `H`는 변수 종류에 따라 다음과 같이 4개의 블록으로 나뉜다.

```
[H_pp  H_pl] [δp]   [b_p]
[H_lp  H_ll] [δl] = [b_l]
```

보통 카메라 포즈 수 `m`이 수십~수백 개인 데 비해, 지도 안의 랜드마크 수 `n`은 수만 개에 이른다(`n >> m`). 이 거대한 행렬 전체의 역행렬을 한 번에 구하려 들면 계산 비용이 감당할 수 없이 커진다.

이 병목을 뚫어 주는 기법이 **Schur complement(슈어 여수)**를 이용한 랜드마크 소거(marginalization)다. 식의 두 번째 행에서 `δl`을 소거해 첫 번째 행에 대입하면, 랜드마크 변수 없이 포즈 변수 `δp`만 남은 축소 시스템을 얻는다.

```
(H_pp - H_pl · H_ll^{-1} · H_lp) δp = b_p - H_pl · H_ll^{-1} · b_l
```

이 소거가 놀라울 만큼 싸게 먹히는 이유는 `H_ll`이 **블록 대각 행렬(block diagonal)**이기 때문이다. 랜드마크들끼리는 서로를 직접 관측하는 factor가 없으므로, `H_ll`의 역행렬은 $3 \times 3$ 크기인 랜드마크 블록마다 따로 구하기만 하면 되며 전체 비용도 랜드마크 수 `n`에 비례하는 `O(n)`에 불과하다.

그 결과 실제 풀어야 할 연립방정식의 행렬 크기는 포즈 수 `m`만으로 결정된다. 물론 랜드마크를 소거하고 나중에 다시 대입하는 과정의 연산량은 여전히 관측 수와 랜드마크 수 `n`에 비례해 늘어나지만, 이 블록 희소 구조 덕분에 수만 개의 3D 점을 포함한 대규모 BA도 실시간이나 준실시간으로 풀 수 있다(실제 실행 속도는 그래프 연결 구조, 솔버 종류, 하드웨어에 따라 달라진다).

축소된 방정식에서 포즈 변화량 `δp`를 먼저 구하고 나면, 랜드마크 보정치 `δl`은 단순한 역치환(back-substitution)으로 곧바로 복원한다.

```
δl = H_ll^{-1} (b_l - H_lp · δp)
```

#### 14.9.3 희소성과 Variable Ordering

Pose graph optimization에서도 Hessian `H`는 대부분이 0으로 채워진 **희소 행렬(sparse matrix)**이다. Factor 하나가 기껏해야 포즈 두세 개만 묶기 때문이다(오도메트리 factor는 이웃 포즈를, 루프 factor는 재방문한 두 포즈를 잇는다). 물론 노드 하나에 얼마나 많은 factor가 붙는지는 데이터셋과 루프 폐쇄 빈도에 따라 달라진다.

이 희소 선형 시스템은 주로 Cholesky 분해(`H = L L^T`)로 푼다. 그런데 분해 과정에서 원래 `H`에서는 0이었던 자리가 삼각 행렬 `L`에서는 0이 아닌 값으로 채워지는 현상이 일어나는데, 이를 **fill-in**이라 부른다. fill-in이 심해지면 희소 행렬의 이점이 사라져 메모리 사용량과 계산 시간이 급격히 불어나며, 특히 루프가 복잡하게 얽히거나 행렬에 변수를 나열하는 순서가 나쁠 때 fill-in이 폭증한다.

따라서 실행 속도를 끌어올리려면 분해 전에 fill-in을 최소화하는 **변수 순서(variable ordering)**를 잘 잡아야 한다.
- **COLAMD** (Column Approximate Minimum Degree): 희소 최소자승 문제에서 널리 쓰이는 휴리스틱으로, fill-in이 적게 생길 만한 열 순서를 빠르게 근사 탐색한다.
- **AMD** (Approximate Minimum Degree): COLAMD와 원리가 같으며, Hessian처럼 대칭인 행렬에 특화된 알고리즘이다.
- **Nested dissection**: 그래프를 재귀적으로 반씩 분할해 가며 경계 노드를 뒤로 미루는 방식으로, 규모가 아주 큰 그래프에서 효과가 뚜렷하다.

g2o, GTSAM, Ceres Solver를 실전에 적용할 때는 선형 솔버의 종류(`DENSE_SCHUR`, `SPARSE_NORMAL_CHOLESKY` 등)와 함께 어떤 ordering 알고리즘을 물릴지 점검해야 한다. Ordering 선택이 실제 메모리와 지연 시간에 미치는 영향은 그래프 토폴로지마다 다르므로, 목표 데이터셋에서 직접 프로파일링해 고르는 것이 정석이다.

```cpp
// Ceres Solver(C++)에서 linear solver와 ordering 설정 예시
ceres::Solver::Options options;
options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
// fill-reducing ordering 알고리즘(linear_solver_ordering_type)은 기본값이 AMD다.
// METIS를 넣어 빌드했다면 ceres::NESDIS(nested dissection)로 바꿀 수 있다
```

#### 14.9.4 최적화 라이브러리 비교

백엔드 수학을 직접 구현하는 대신 실전에서는 검증된 오픈소스 솔버를 가져다 쓴다. 대표적인 세 라이브러리의 성격과 알맞은 쓰임새를 비교하면 다음과 같다.

| 라이브러리 | 특징 | 주요 사용처 |
|---|---|---|
| g2o | Pose graph / BA 전용, 가벼움, C++ only | ORB-SLAM2/3, LSD-SLAM |
| GTSAM | Factor graph 기반, Bayes tree(iSAM2) 지원, incremental 최적화에 강함 | LIO-SAM, 연구용 |
| Ceres Solver | 범용 nonlinear least squares, auto-diff 지원, Google 개발 | Cartographer, VINS-Fusion, 다양한 프로젝트 |

프로젝트의 성격에 따라 선택 기준을 정리해 보면 다음과 같다.
- SLAM용 pose graph나 BA만 빠르고 가볍게 돌리고 싶다 → **g2o**
- 센서 퓨전을 factor graph로 깔끔하게 모델링하고, 새 키프레임이 들어올 때마다 전체를 다시 풀지 않고 점진적으로(incremental) 갱신해야 한다 → **GTSAM (iSAM2)**
- SLAM 이외의 범용 비선형 최적화까지 아우르고 싶고, 복잡한 수식의 Jacobian을 손으로 유도하기 번거롭다 → **Ceres Solver (auto-diff)**

> **추천 자료**
> - Barfoot, "State Estimation for Robotics" Ch.7 (Matrix Lie Groups, §7.1.9 Calculus and Optimization)와 Ch.8 — Manifold 위의 최적화를 체계적으로 설명
> - [Dellaert & Kaess, "Factor Graphs for Robot Perception" (Foundations and Trends in Robotics, 2017)](https://www.cs.cmu.edu/~kaess/pub/Dellaert17fnt.pdf) — Factor graph와 SLAM 백엔드 이론
> - [g2o Tutorial](https://github.com/RainerKuemmerle/g2o) / [GTSAM Tutorial](https://gtsam.org/tutorials/intro.html) — 라이브러리별 실습
> - [김기섭 블로그 — Gauss-Newton Opt == IEKF update?](https://gisbi-kim.github.io/blog/2022/03/05/gn-iekf-same.html) — GN 최적화와 반복 칼만 필터의 수학적 동치성 설명

> **실습**: [Bundle Adjustment 시각화](https://alexjunholee.github.io/robotics-practice/app.html#bundle_adjustment)
> 카메라 포즈와 3D 포인트를 동시에 최적화하는 Bundle Adjustment 과정을 인터랙티브하게 확인할 수 있다.

### 14.10 심화: IMU Preintegration

앞서 14.3.3절의 VINS-Mono와 14.4.3절의 LIO-SAM에서 핵심 요소로 언급한 **IMU preintegration**의 수학을 들여다보자(IEKF와 MSCKF의 바탕이 되는 EKF는 §3.10.2 참조).

**해결하려는 문제**: IMU는 보통 200~1000 Hz의 빠른 속도로 가속도와 각속도를 뿜어낸다. 반면 카메라나 LiDAR 백엔드는 수 Hz에서 수십 Hz 주기의 키프레임 단위로 최적화를 돌린다. 키프레임 두 장 사이에도 IMU 측정이 수십에서 수백 개씩 끼어드는 셈인데, IMU 측정 시각마다 로봇 상태(포즈, 속도, 바이어스)를 노드로 세우면 그래프의 변수 개수가 순식간에 감당할 수 없을 만큼 불어난다.

**Preintegration의 핵심 아이디어**는 두 키프레임 `i`와 `j` 사이에 쏟아진 수십~수백 개의 IMU 측정을 단 하나의 **상대 운동 제약(relative motion measurement)**으로 압축한 뒤, 이를 단일 factor로 최적화 그래프에 넣는 데 있다.

먼저 키프레임 `i`에서 `j`까지 이어진 IMU 샘플 `k = i, ..., j-1`을 적분해 회전, 속도, 위치의 세 가지 상대 변화량을 계산한다.

```
ΔR_ij = Π_{k=i}^{j-1} Exp((ω_k - b_g) · Δt)          # 상대 회전
Δv_ij = Σ_{k=i}^{j-1} ΔR_ik · (a_k - b_a) · Δt        # 상대 속도
Δp_ij = Σ_{k=i}^{j-1} (Δv_ik · Δt + 0.5 · ΔR_ik · (a_k - b_a) · Δt^2)  # 상대 위치
```

여기서 `ω_k`와 `a_k`는 IMU의 원시 각속도·가속도 측정값, `b_g`와 `b_a`는 자이로스코프·가속도계 바이어스, `Δt`는 IMU 샘플링 간격이다. 이 식의 진가는 모든 적분이 월드 좌표계가 아닌 **키프레임 `i`의 로컬 좌표계**를 기준으로 이루어진다는 데 있다. 덕분에 백엔드 최적화 과정에서 키프레임 `i`의 전역 포즈나 중력 방향 추정치가 계속 수정되더라도 이 상대 변화량을 처음부터 다시 적분할 필요가 없다.

**공분산 전파**: 상대 변화량만 구해서는 factor graph에 어느 정도의 신뢰도로(얼마나 세게) 반영해야 할지 알 수 없다. 따라서 IMU 측정 노이즈가 preintegrated measurement로 얼마나 전파되는지도 함께 추적해야 한다. 새 IMU 측정이 들어올 때마다 이산 시간 선형화 식으로 오차 공분산을 재귀적으로 갱신한다.

```
Σ_{k+1} = A_k · Σ_k · A_k^T + B_k · Q · B_k^T
```
- `A_k`: 현재 상태에서의 상태 전이 Jacobian 행렬
- `B_k`: 노이즈 입력 행렬
- `Q`: IMU 센서의 원시 노이즈 공분산 (데이터시트 또는 Allan variance 캘리브레이션으로 확인)

최적화 단계에서는 이렇게 얻은 공분산의 역행렬(`Σ^{-1}`)을 IMU preintegration factor의 information matrix로 사용한다.

**Bias가 바뀔 때의 1차 보정**: 한 가지 문제가 남는다. 위 식에서 보듯 적분값 `ΔR_ij, Δv_ij, Δp_ij`는 바이어스 추정치 `b_g, b_a`에 의존한다. 만약 백엔드 최적화 도중 바이어스 추정치가 바뀌면 원칙상 모든 IMU 측정을 다시 적분해야 하므로 계산 이점이 사라진다. 이를 피하기 위해 preintegration 과정에서 바이어스에 대한 편미분(Jacobian)을 미리 같이 쌓아 두었다가, 바이어스가 조금(`δb`) 변했을 때는 재적분 없이 다음 1차 테일러 근사로 값을 보정한다.

```
ΔR_ij ≈ ΔR_ij^0 · Exp(∂ΔR/∂b_g · δb_g)
Δv_ij ≈ Δv_ij^0 + ∂Δv/∂b_g · δb_g + ∂Δv/∂b_a · δb_a
Δp_ij ≈ Δp_ij^0 + ∂Δp/∂b_g · δb_g + ∂Δp/∂b_a · δb_a
```

여기서 위 첨자 `^0`은 이전 바이어스 추정치로 계산해 둔 기준값이고 `δb`는 바이어스 변화량이다. 바이어스 갱신 폭이 과도하게 크지 않은 한 이 1차 근사만으로 충분한 정확도를 얻는다.

**Manifold 위의 preintegration**: 사실 preintegration 개념 자체는 Lupton과 Sukkarieh가 먼저 제안했으나 당시에는 회전을 오일러 각(Euler angle)으로 다루어 짐벌 락(gimbal lock) 등의 한계가 있었다. 이후 Forster 등(RSS 2015, TRO 2017)이 이를 회전 군 $SO(3)$ 다양체 위에 정식화하면서 현대적인 형태로 자리 잡았다. 리 군 위에서 적분하면 회전 행렬의 직교 조건이 깨지지 않고 특이점도 피할 수 있으며, 최종 결과는 곧바로 factor graph의 상대 운동 factor로 들어간다(물론 이산 시간 적분에 따른 근사 오차와 센서 잡음 자체는 남는다).

**Tightly-coupled와 Loosely-coupled**: LiDAR-Inertial 시스템인 LIO-SAM을 예로 들면 두 결합 방식의 차이가 선명해진다.
- **Loosely-coupled**: 크게 두 형태로 나뉜다. 하나는 IMU를 보조 도구로만 써서 포인트 클라우드 모션 왜곡을 제거(de-skew)하고 스캔 매칭의 초기 추정치를 얻는 방식(LeGO-LOAM이 여기에 가깝다)이고, 다른 하나는 LiDAR 오도메트리와 IMU가 각자 상태를 추정한 뒤 나중에 공분산으로 합치는 방식이다.
- **Tightly-coupled**: IMU preintegration factor와 LiDAR 오도메트리 factor를 *동일한* factor graph 안에 넣어 함께 최적화한다. IMU가 단순히 초기값만 던져 주는 것이 아니라 키프레임 사이 상대 포즈의 독립적인 관측 제약으로 직접 참여하므로, 고속 회전이나 급가감속처럼 LiDAR 스캔 매칭만으로는 놓치기 쉬운 격렬한 움직임에서도 IMU factor가 궤적을 단단히 붙잡아 준다. **LIO-SAM**이 바로 이 구조를 따르며, factor graph 기반인 덕분에 GPS factor나 loop closure factor를 필요에 따라 손쉽게 덧붙일 수 있다는 실용적 장점도 크다.

구체적으로 LIO-SAM은 GTSAM 백엔드 안에서 IMU preintegration, LiDAR 오도메트리, GPS, loop closure의 네 가지 factor를 한 그래프에서 동시에 최적화한다. LiDAR 오도메트리는 매 스캔에서 모서리(edge)와 평면(planar) 특징을 분리해 해상도가 다른 voxel map에 쌓고, 스캔 매칭에서는 point-to-edge 거리와 point-to-plane 거리를 최소화하는 상대 변환을 계산한다. 오늘날 VINS-Mono, ORB-SLAM3(Visual-Inertial 모드), LIO-SAM 등 주요 VIO/LIO 시스템은 모두 이 on-manifold preintegration을 표준 부품처럼 채택한다.

> **추천 자료**
> - [Forster et al., "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry" (TRO 2017, arXiv:1512.02363)](https://arxiv.org/abs/1512.02363) — on-manifold preintegration의 원본 논문(사전 적분 개념 자체는 Lupton & Sukkarieh 2012). 수식이 많지만 이 분야의 필수 논문
> - [Forster et al., "IMU Preintegration on Manifold for Efficient Visual-Inertial Maximum-a-Posteriori Estimation" (RSS 2015)](https://rpg.ifi.uzh.ch/docs/RSS15_Forster.pdf) — 위 논문의 초기 버전으로, 방법을 더 간결하게 설명한다.
> - [Shan et al., "LIO-SAM" (IROS 2020)](https://github.com/TixiaoShan/LIO-SAM) — Tightly-coupled LIO의 레퍼런스 구현. 코드와 논문 모두 읽을 것
> - [Sola et al., "A micro Lie theory for state estimation in robotics" (arXiv:1812.01537)](https://arxiv.org/abs/1812.01537) — Lie group/algebra의 실용적 정리. Preintegration 논문을 읽기 전에 먼저 훑어보면 수식이 훨씬 쉽게 읽힌다
> - GTSAM의 `PreintegratedImuMeasurements` 클래스 소스코드 — 이론이 코드로 어떻게 구현되는지 확인
> - [IMU Preintegration MATLAB 구현](https://github.com/GentleDell/imu_preintegration_matlab) — KITTI에서 테스트한 MATLAB 코드로, 수식과 실제 구현을 대조하며 따라가기 좋다

### 14.11 심화: 관측가능성 분석 (Observability)

SLAM이나 VIO를 실제로 돌려 보면, 로봇이 멀쩡히 가만히 서 있는데도 추정 위치가 스멀스멀 움직이거나 특정 축 방향으로 유독 드리프트가 심하게 쌓이는 현상을 종종 겪는다. 이는 단순한 코드 버그가 아니라 센서와 물리 법칙이 원천적으로 구분할 수 없는 자유도, 즉 **관측가능성(observability)**의 한계에서 비롯되는 문제다.

**Visual-Inertial 시스템에서 관측할 수 없는 상태**: Hesch 등(2014)은 임의로 움직이며 점 특징 하나를 관측하는 이상적인 VINS 모델을 선형화해 관측가능성을 엄밀하게 분석했고, 센서 데이터만으로는 절대 알아낼 수 없는(unobservable) 방향이 정확히 **4자유도**임을 보였다.

1. **Global position (3 DoF)**: 지구상의 절대 좌표 $(x, y, z)$는 카메라 영상과 IMU 어디에도 들어 있지 않으므로 보통 출발점을 원점으로 삼는다. 절대 위치가 필요하면 GPS 같은 외부 기준 센서를 붙여야 한다.
2. **Global yaw (1 DoF)**: 가속도계가 중력 벡터를 감지하므로 지면과 수평인 roll과 pitch는 관측할 수 있지만, 중력축을 중심으로 회전하는 수평 방향각(heading/yaw)은 어느 쪽이 북쪽인지 알 도리가 없다. 이를 고정하려면 나침반(지자계)이나 외부 기준이 필요하다.

물론 이 4자유도 결론은 위에서 가정한 이상적 모델과 충분한 움직임 조건에서 성립한다. 나머지 상태인 roll/pitch, 미터 단위의 scale, IMU 바이어스, 센서 간 외부 보정(calibration) 파라미터가 실제로 관측 가능할지는 센서 구성과 로봇이 얼마나 다양하게 가속·회전하는지(excitation)에 달려 있다. 예를 들어 스테레오 카메라는 두 렌즈 사이의 고정된 기선(baseline) 길이 덕분에 metric scale을 바로 얻지만, 단안 카메라는 영상만으로 scale을 알 수 없으므로 충분히 가속하고 회전하는 구간의 IMU 측정을 엮어 scale을 복원해야 한다.

**Degenerate motion**: 로봇의 움직임 패턴 자체가 단조로우면 관측할 수 없는 자유도가 4개보다 더 늘어난다.

- **순수 회전(pure rotation)**: 로봇이 제자리에서 돌기만 하면 병진 시차(parallax)가 0이라 단안 영상에서 이동 거리와 새로 본 점의 깊이가 미정으로 남는다(회전 자체는 추정된다).
- **초기화 구간의 부족한 운동**: 단안 VIO를 켜고 등속 직진만 하거나 가만히 정지해 있으면 가속도가 변하지 않아 IMU 바이어스와 중력·스케일을 분리하는 방정식의 해가 여럿 남는다. 그 결과 초기화 단계에서 scale, 중력 방향, 바이어스 추정이 불안정해진다(정확한 퇴화 조건은 상태 정의와 측정 모델에 따라 달라진다).

**EKF 기반 시스템의 문제**: 관측할 수 없는 4자유도 방향은 정보가 없으니 이론상 불확실성(공분산)이 줄어들어서는 안 된다. 그러나 표준 EKF로 VIO를 풀면 매 스텝 다른 상태 추정치 위에서 선형화하다 보니 Jacobian의 영공간(null space)이 미세하게 어긋난다. 그 결과 필터는 "관측할 수 없는 global yaw까지 내가 정확히 알고 있다"고 착각하며 해당 방향의 공분산을 인위적으로 줄여 버린다. 이 거짓 확신이 누적되면 필터가 실제 오차보다 훨씬 좁은 공분산을 믿고 새 측정을 무시하는 **비일관성(inconsistency)** 문제로 이어져 발산을 부른다.

**OC-EKF (Observability-Constrained EKF)**: 이 병목을 고치기 위해 Hesch 등은 EKF의 Jacobian을 계산할 때 시스템이 원래 가져야 할 관측 불가능한 영공간을 강제로 유지하도록 제약(Observability-Constrained)을 걸었다. 덕분에 관측할 수 없는 방향의 불확실성이 필터 내부에서 거짓으로 쪼그라들지 않아 장기 일관성이 크게 높아진다.

실제 현장에서 VIO나 SLAM 시스템을 운용할 때는 이 이론적 한계를 염두에 두고 다음 세 가지를 챙겨야 한다.
- VIO를 초기화할 때는 사용 중인 구현이 요구하는 수준만큼 회전과 병진 이동을 충분히 시킨 뒤, scale·중력·IMU 바이어스가 안정적으로 수렴했는지 확인한다.
- Loop closure는 재방문한 장소를 연결해 상대적인 누적 드리프트를 펴 줄 뿐, 절대 좌표나 절대 북쪽(yaw) 기준을 새로 만들어 주지는 못한다.
- 단안 VIO의 미터 단위 scale은 영상만으로 정해지지 않으므로, 초기화 구간에 충분한 가속 운동과 IMU 정보가 들어갔는지 반드시 점검한다.

> **추천 자료**
> - [Hesch et al., "Consistency Analysis and Improvement of Vision-aided Inertial Navigation" (TRO 2014)](https://ieeexplore.ieee.org/document/6605544) — OC-EKF/OC-VINS의 원본 논문
> - [Hong & Lim, "Visual-Inertial Odometry with Robust Initialization and Online Scale Estimation" (Sensors 2018)](https://pmc.ncbi.nlm.nih.gov/articles/PMC6308559/) — 등속·정지 구간에서 초기화가 약해지는 조건을 다루는 구현 예
> - Barfoot, "State Estimation for Robotics" Ch.3 (§3.1.4, §3.3.7 Existence, Uniqueness, and Observability) — 관측가능성 분석의 이론적 토대
> - [Huang & Dissanayake, "A critique of current developments in Simultaneous Localization and Mapping" (IJARS 2016)](https://journals.sagepub.com/doi/10.1177/1729881416669482) — SLAM의 관측가능성/일관성 문제를 비판적으로 정리

> **실습**: [Odometry Uncertainty 시각화](https://alexjunholee.github.io/robotics-practice/app.html#odom_uncertainty)
> Odometry의 불확실성이 시간에 따라 어떻게 누적되는지, 공분산 타원이 어떻게 커지는지 인터랙티브하게 확인할 수 있다.

#### 14.11.1 필터 기반 vs 최적화 기반: 뭐가 더 나은가?

SLAM과 VIO 커뮤니티에서는 "칼만 필터 기반과 그래프 최적화 기반 중 어느 쪽이 더 우월한가?"라는 논쟁이 오랫동안 이어져 왔다. 흥미롭게도 14.4.4절에서 살짝 짚었듯, 순수 수학의 눈으로 보면 **Gauss-Newton 최적화와 Iterated EKF(IEKF)는 동일한 문제를 서로 다른 형태로 푸는 동치 관계**에 놓여 있다. 그럼에도 실제 시스템에서는 상태 변수를 얼마나 오래 쥐고 있는지와 재선형화(relinearization) 구조에 따라 성능과 특성이 갈린다.

- **필터 기반 (EKF, MSCKF 등)**: 새로운 측정이 들어올 때마다 현재 상태와 공분산을 즉시 갱신하고 과거 상태는 곧바로 marginalize해 버린다. 기본 EKF 오도메트리는 현재 시점의 상태만 들고 가고, MSCKF는 최근 카메라 포즈 몇 개만 복사해 슬라이딩 윈도에 남겨 둔다. 유지하는 상태 크기가 일정 범위 안에 묶여 있으므로 과거 전체를 쥐고 있는 최적화 방식보다 메모리와 연산이 훨씬 가볍다.
- **최적화 기반 (BA, factor graph)**: 과거의 키프레임 포즈와 측정 제약을 그래프에 남겨 두고 한꺼번에 비선형 최적화를 푼다. 필터와 달리 추정치가 개선된 뒤에 과거 시점의 Jacobian을 **다시 선형화(relinearize)**할 수 있으므로 초기 선형화 오차를 털어내고 높은 정확도를 얻는다. 상태가 쌓일수록 계산이 무거워지는 문제는 슬라이딩 윈도로 오래된 노드를 쳐내거나(VINS-Mono), 바뀐 부분만 다시 푸는 증분 평활화(iSAM2)로 해결한다.

따라서 VINS-Mono(최적화)와 MSCKF(필터)의 정확도 차이는 수학적 우열이라기보다 어떤 상태를 남기고 재선형화를 허용하느냐는 아키텍처 설계에서 나온다. 실무에서는 탑재할 하드웨어와 센서 구성을 보고 다음과 같이 가늠해 선택한다.
- IMU가 중심이고 극한의 경량화·저지연이 필요하다 → **필터 계열** (MSCKF, FAST-LIO2의 IEKF)
- 카메라가 중심이고 최고 수준의 전역 정확도가 중요하다 → **최적화 계열** (VINS-Mono, ORB-SLAM3)
- 높은 반응성과 다중 센서 확장성, 전역 정밀도가 모두 필요하다 → **하이브리드** (LIO-SAM처럼 IMU preintegration을 factor로 넣고 그래프 최적화 수행)

(참고: [김기섭 블로그 — Gauss-Newton Opt == IEKF update?](https://gisbi-kim.github.io/blog/2022/03/05/gn-iekf-same.html))

### 14.12 심화: Semantic SLAM

전통적인 SLAM은 공간의 기하학적 형상만 복원한다. 포인트 클라우드든 메시든 점유 격자든 "어디에 장애물이 있는지"는 기록하지만, 그 장애물이 벽인지 문인지 책상인지는 알지 못한다. **Semantic SLAM**은 이 기하 지도에 사물의 의미(semantic) 정보를 입혀 로봇이 공간을 이해하게 만든다.

의미를 지도에 녹이는 방식은 랜드마크를 무엇으로 표현하느냐에 따라 나뉜다. 먼저 **Object-level SLAM**(CubeSLAM, QuadricSLAM)은 점 특징 대신 3D 직육면체(cuboid)나 이중 이차곡면(dual quadric) 같은 물체 단위를 랜드마크로 추정한다. 물체 검출기를 돌려야 하는 부담은 있지만, 점 하나보다 데이터 연관(data association)이 훨씬 안정적이고 물체 수준에서 공간을 추론할 수 있다. 한 걸음 더 나아간 panoptic SLAM은 2D panoptic segmentation 결과를 3D 지도로 투영해 누적함으로써, 모든 점과 복셀에 의미 라벨과 개체 ID가 달린 지도를 만든다. 덕분에 로봇은 "지금 이 방에 의자가 몇 개인가?" 같은 질의에 지도만 뒤져 답할 수 있다. 최근의 **Open-vocabulary SLAM**(ConceptGraphs)은 여기서 CLIP 같은 vision-language model의 임베딩 특징까지 지도에 저장해, 사전에 학습하지 않은 임의의 자연어 문구로도 목표 물체나 장소를 검색하게 해 주며 13장의 3D Scene Graph(Hydra 등)와 곧바로 맞닿는다.

**움직이는 물체 처리**: 의미 라벨은 지도를 풍부하게 만들 뿐 아니라, 사람이나 차량이 북적이는 동적 환경에서 SLAM이 무너지지 않게 지키는 방패로도 쓰인다. 영상에서 움직일 가능성이 높은 클래스를 미리 가려내면, 정지한 벽과 바닥 등 정적 배경의 특징점만 남겨 깨끗하게 위치를 추적하고 지도를 쌓을 수 있다. 대표적인 **DynaSLAM**은 ORB-SLAM2에 Mask R-CNN을 결합해 동적 물체를 마스킹하고 배경을 복원하며, **DS-SLAM**은 시맨틱 세그멘테이션과 광류 검사를 엮어 동적 영역의 특징점을 걸러낸다.

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

로봇 한 대로 드넓은 건물이나 재난 현장을 탐색하려면 시간이 너무 오래 걸린다. 로봇 여러 대에 구역을 나누어 맡기면 탐색 속도는 빨라지지만, 각 로봇이 자기 기준 좌표계에서 만든 부분 지도(submap)를 하나의 일관된 전체 지도로 합치는 새로운 과제가 생긴다.

다중 로봇 협력 아키텍처는 크게 중앙 집중형과 분산형으로 갈린다. **Centralized** 방식은 모든 로봇이 원시 센서 데이터나 지역 지도를 중앙 서버로 올려보내 한꺼번에 최적화한다. 이 방식은 서버가 전체 상황을 조망하기는 편하지만 대역폭과 연산 부담이 한곳에 몰리고, 중앙 서버가 뻗으면 팀 전체가 마비되는 단일 장애점(single point of failure) 문제도 안는다. 게다가 SLAM 문제는 본질적으로 비볼록(non-convex)이기 때문에 중앙 서버에서 한꺼번에 푼다고 해서 반드시 전역 최적해가 보장되는 것도 아니다.

반면 **Distributed** 방식에서는 로봇 각자가 독립적으로 지역 SLAM을 돌리다가, 두 로봇이 서로 마주치거나(rendezvous) 상대가 앞서 지나간 장소를 알아봤을 때(inter-robot loop closure) 통신으로 상대 포즈 제약을 교환한다. 원시 포인트 클라우드 대신 가벼운 기술자(descriptor)나 요약된 부분 지도만 주고받으면 통신량은 크게 줄지만, 잘못된 매칭을 걸러낼 기하 검증 정보가 덩달아 줄어드는 트레이드오프가 따른다. 각 로봇이 어떤 변수와 factor를 유지·교환하는지, 통신 지연이 있는 비동기 환경에서 분산 최적화가 어떤 조건에서 수렴하는지는 알고리즘 설계마다 다르다.

분산 SLAM을 실제 현장에 올리려면 로봇 간 루프 폐쇄(inter-robot loop closure), 서로 다른 시작 좌표계 정렬, 치명적인 오탐(outlier) 제거라는 과제를 동시에 풀어야 한다. 로봇 사이의 $SE(3)$ 상대 변환은 이미 검증된 6자유도 포즈 제약으로 직접 주고받기도 하고, 공유한 3차원 특징점 대응쌍으로부터 역산하기도 한다(후자의 경우 한 직선 위에 놓이지 않은 점 대응이 최소 세 쌍 이상 필요하고 인라이어 비율도 충분해야 한다). 이 과정에서 잘못된 루프를 솎아 내고 분산 합의를 이루기 위해 PCM(Pairwise Consistency Maximization), GNC(Graduated Non-Convexity), 분산 Gauss-Seidel(DGS), ADMM 등이 저마다의 가정과 통신 비용을 바탕으로 쓰인다.

대표적인 분산 Multi-Robot SLAM 시스템을 아래 표에 비교해 정리했다.

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

단일 로봇의 Loop Closure든 다중 로봇 간의 조우든, 핵심은 "지금 눈앞의 장면이 이전에 본 적 있는 장소인가?"를 판별하는 **Place Recognition**이다. 본질적으로 대규모 데이터베이스에서 현재 프레임의 기술자(descriptor)와 가장 비슷한 과거 키프레임을 건져 올리는 **이미지·장면 검색(retrieval)** 문제다. 14.6.1절에서도 강조했듯 place recognition이 엉뚱한 장소를 연결하면 SLAM 지도 전체가 왜곡되므로, 높은 정밀도와 환경 변화에 대한 내성이 필수다.

**전통적 방법: Bag of Visual Words (BoVW)**

ORB-SLAM2/3에 내장된 DBoW2 라이브러리가 따르는 고전 방식은 문서 검색의 단어 주머니(Bag of Words) 모델을 영상에 옮겨 놓았다.
1. 방대한 훈련 이미지에서 local feature(ORB 등)를 대량 추출한다.
2. k-means clustering으로 특징 공간의 대표 군집들을 뽑아 시각 단어 사전(visual vocabulary)을 구축한다.
3. 새 이미지가 들어오면 특징점을 추출해 가장 가까운 시각 단어로 치환하고, 어떤 단어가 몇 번 등장했는지 세어 히스토그램(BoW vector)으로 요약한다.
4. 역색인(inverted index) 구조를 이용해 BoW vector 사이의 유사도(L1-score 등)가 높은 과거 프레임을 빠르게 찾는다.

DBoW2는 계산이 매우 가벼워 CPU에서도 밀리초 단위로 돌지만, 미리 학습한 어휘 사전에 얽매이고 조명이나 시점이 크게 바뀌면 특징점 자체가 달라져 매칭 성능이 급격히 떨어진다.

**학습 기반 방법: Global Descriptor**

이 한계를 넘기 위해 딥러닝으로 이미지 한 장 전체를 환경 변화에 둔감한 짧은 벡터 하나로 압축하는 연구가 발전했다. **NetVLAD**(2016)는 CNN 특징 맵 위에 고전적인 VLAD 집계를 미분 가능한 층으로 얹어 end-to-end로 학습해, 도시 규모 벤치마크에서 기존 기법들보다 눈에 띄게 높은 재현율(recall)을 달성했다. 뒤이어 **CosPlace**(2022)와 **MixVPR**(2023)은 학습 손실과 특징 집계 구조를 개량해 각자의 평가 벤치마크에서 검색 정확도와 효율을 끌어올렸다. 최근의 **AnyLoc**(2023)은 대규모 사전학습 비전 모델인 DINOv2의 특징을 그대로 활용해, 장소 인식용 추가 미세조정(fine-tuning) 없이도 실내·실외·항공 등 전혀 다른 도메인에서 준수한 성능을 보고했다. 다만 이러한 학습 기반 결과가 모든 현장에서 무조건 BoVW를 앞선다는 보장은 없으므로, 실제 도입할 때는 목표 환경의 데이터를 모아 재현율과 오탐률(false positive)을 직접 재어 보아야 한다.

**LiDAR 기반 Place Recognition**

레이저로 3D 형상을 재는 LiDAR는 밤낮이나 조명 변화에 전혀 흔들리지 않는다. 대신 비·눈 같은 기상 조건이나 주변을 지나는 차량에 따라 반사점이 달라지고, 구조가 좌우 대칭인 긴 복도나 터널에서는 전혀 다른 구간을 같은 곳으로 헷갈리기 쉽다.

- **Scan Context** (IROS 2018): 3D 포인트 클라우드를 위에서 내려다본 평면에 투영한 뒤, 거리와 방위각으로 나눈 극좌표 칸마다 가장 높은 점의 높이를 기록해 2D 행렬 기술자를 만든다. 방위각 축으로 열을 순환 이동(column shift)하며 비교할 수 있어 로봇이 반대 방향으로 진입해 제자리에서 돌아도 거뜬히 매칭한다.
- **OverlapTransformer** (2022): LiDAR 거리 영상(range image)에 Transformer 어텐션을 적용해 두 스캔 사이의 겹침 비율에 비례하는 전역 기술자를 학습한다.

**Cross-modal 및 Sequence Matching**

환경과 센서 구성이 다양해지면서 단일 프레임·단일 센서 검색을 넘어서는 기법도 쓰인다. **Cross-modal Place Recognition**은 카메라 영상으로 LiDAR 지도를 검색하거나 거꾸로 LiDAR 스캔으로 이미지 지도를 검색하는 기술로, 서로 다른 센서를 단 로봇들이 협력하는 multi-robot SLAM에서 요긴하다. 한편, 눈 덮인 겨울과 푸른 여름처럼 겉모습이 너무 달라 이미지 한 장만으로는 도저히 분간하기 힘들 때는 **Sequence Matching**으로 연속된 프레임 여러 장을 묶어 비교한다. **SeqSLAM**(2012)은 낱장 유사도는 낮더라도 일정 구간에 걸친 연속 이미지의 유사도 행렬에서 대각선 패턴이 맞으면 같은 경로로 판정해 낮과 밤을 넘나드는 장소 인식을 해냈다. **SeqNet**(RA-L 2021)은 이미지 시퀀스 전체를 하나의 sequence descriptor로 표현하도록 학습해 SeqSLAM식 정밀 비교에 앞서 후보군을 빠르게 추려낸다.

실제 현장에서 조명이나 계절 변화가 큰 환경을 다룬다면, 표준 기준선인 DBoW2와 함께 NetVLAD 계열 학습 기반 descriptor를 동일 데이터셋에서 나란히 비교해 보는 것이 안전하다. 특히 AnyLoc처럼 거대 백본(DINOv2 등)을 쓰는 방법은 별도 학습이 필요 없다. 대신 GPU 연산량과 descriptor 메모리 점유율을 감당할 수 있는지 따져 보아야 한다.

> **추천 자료**
> - [Arandjelovic et al., "NetVLAD: CNN architecture for weakly supervised place recognition" (arXiv:1511.07247)](https://arxiv.org/abs/1511.07247) — CNN 기반 place recognition을 end-to-end로 학습한 논문
> - [Keetha et al., "AnyLoc: Towards Universal Visual Place Recognition" (arXiv:2308.00688)](https://arxiv.org/abs/2308.00688) — Foundation model 기반 zero-shot place recognition
> - [Kim & Kim, "Scan Context: Egocentric Spatial Descriptor for Place Recognition within 3D Point Cloud Map" (IROS 2018)](https://ieeexplore.ieee.org/document/8593953) — LiDAR place recognition의 대표 방법
> - [김기섭 블로그 — Scan Context-based LiDAR Pose-graph SLAM 구현](https://gisbi-kim.github.io/blog/2021/05/17/sclidarslam.html) — Scan Context를 LiDAR SLAM에 통합한 구현 해설
> - [다크 프로그래머 — Bag of Words 기법](https://darkpgmr.tistory.com/125) — BoW의 원리를 이미지 검색과 연결하여 설명

> **기술 흐름: SLAM & Odometry**
> - **~2007**: EKF-SLAM과 입자 필터 기반 FastSLAM이 주로 쓰임. MonoSLAM(2007)이 실시간 단안 SLAM을 보였고, PTAM(2007)이 Tracking과 Mapping을 나눈 구조를 내놓음
> - **2010~2015**: LSD-SLAM, SVO 등 direct method 등장. LOAM(2014)이 LiDAR odometry와 mapping의 영향력 있는 구조를 제시했고, ORB-SLAM(2015)이 공개 feature-based visual SLAM 기준선이 됨
> - **2015~2020**: VINS-Mono(2018), MSCKF 계열 등 VIO system의 공개 구현. DSO가 direct sparse 방식을 제시했고, LIO-SAM 등 LiDAR-inertial system 등장
> - **2020~2023**: FAST-LIO/FAST-LIO2, ORB-SLAM3, DROID-SLAM, R3LIVE 등 filter·optimization·learning을 서로 다르게 조합한 공개 system 등장
> - **2024~**: 3DGS 기반 SLAM 발표(SplaTAM·MonoGS는 CVPR 2024, Gaussian-SLAM은 arXiv 2023 프리프린트)

### 14.15 심화: Long-term Mapping

로봇을 실험실에서 꺼내 실제 물류 창고나 도심에 몇 달씩 투입하려면, 한 번 그려 놓은 지도로 버틸 수 없다. 계절과 시간 흐름에 맞춰 지도를 계속 고쳐 쓰고, 지나다니는 사람이나 주차된 차처럼 일시적인 물체는 지도에서 털어내며, 어제 모은 데이터와 오늘 모은 데이터를 자연스럽게 이어 붙여야 한다. 이렇게 시간이 지나도 썩지 않는 지도를 유지·관리하는 기술 전체를 **Long-term mapping**이라 부른다.

#### 14.15.1 Incremental Smoothing: iSAM에서 iSAM2까지

장기 운용에서는 데이터가 끝없이 쌓이므로 매번 처음부터 지도를 다시 최적화할 수 없다. EKF 같은 고전 필터는 랜드마크가 늘어날수록 공분산 행렬이 커져 실시간성이 무너지고, 배치 최적화는 매 스텝 전체 그래프를 다시 푸느라 느려진다. **iSAM**(Kaess et al., TRO 2008)은 행렬을 QR 분해할 때 Givens rotation을 이용하면 새 측정이 들어온 행과 관련된 $R$ 행렬의 일부만 살짝 고쳐 증분 갱신(incremental update)할 수 있음을 보였다. 다만 0이 아닌 원소(fill-in)가 서서히 쌓이기 때문에 주기적으로 변수 순서를 다시 잡아 분해를 새로 해야 하는 숙제가 남았다.

후속 연구인 **iSAM2**(Kaess et al., IJRR 2012)는 행렬 대신 **Bayes tree**라는 트리 자료구조를 도입해 이 병목마저 돌파했다. 새로운 측정 factor가 추가되면 트리에서 영향을 받는 좁은 부분 트리(subtree)만 찾아 변수를 소거하고 재선형화하므로, 지도가 커져도 매 스텝 연산 속도가 일정하게 유지된다. GTSAM 라이브러리의 핵심 증분 최적화 엔진이 바로 이 iSAM2다.

#### 14.15.2 Dynamic Object Removal

며칠에 걸쳐 모은 데이터로 정밀한 기준 지도를 만들려 할 때, 주행 당시 앞을 지나가던 보행자나 도로 위 차량의 잔상이 지도에 남으면 정적 환경에 대한 위치 추정까지 방해한다. 따라서 장기 매핑에서는 동적 물체가 남긴 점을 깔끔하게 걷어내는 과정이 필수적이다.

**Removert**(Kim et al., 2020)는 계산이 무거운 3D 복셀 레이캐스팅 대신, 포인트 클라우드를 각 스캔 시점의 2D 거리 영상(range image)으로 투영해 다른 시점에서 관측된 거리와 대조하는 실용적인 해법을 제시했다. 만약 과거 스캔에서 레이저가 막힘없이 통과했던 허공에 어떤 점이 놓여 있다면 그 점은 당시 움직이던 물체였을 가능성이 높다. Removert는 오탐을 줄이기 위해 먼저 확실히 정지해 있는 점만 보수적으로 남긴 뒤(remove), 해상도가 다른 여러 range image 단계를 거치며 잘못 지워진 정적 구조물을 다시 되살리는(revert) 2단계 전략으로 균형을 맞춘다. 다른 기법들과 견주어 보면, 3D 복셀 레이캐스팅은 정교하지만 연산이 무겁고, 가시성(visibility) 기반 기법은 뒷배경에 가려진 정지 점을 보호하려면 추가 가정이 필요하며, 시맨틱 세그멘테이션 기반 방식은 사전에 학습하지 않은 클래스의 동적 물체를 놓치기 쉽다.

한편 **SuMa++**(Chen et al., IROS 2019)는 작은 원판(surfel)으로 3D 지도를 쌓는 SuMa 프레임워크에 시맨틱 정보를 엮었다. 단순히 "차량 클래스는 무조건 지운다"고 처리하지 않고, 시맨틱 라벨과 실제 프레임 간 기하학적 움직임 검사를 함께 활용해 진짜 움직인 것으로 판명된 surfel만 제거한다. 덕분에 주차장에 얌전히 서 있는 차량처럼 움직일 *가능성*은 있지만 실제로는 정지해 있는 물체는 버리지 않고 유용한 위치 추정 랜드마크로 살려 쓸 수 있다.

#### 14.15.3 Multi-Session SLAM

며칠에 걸쳐 구역을 나누어 주행한 뒤 각 세션에서 만든 궤적과 부분 지도를 하나로 합치려 하면 문제가 생긴다. 세션마다 전원을 켠 출발점이 달라 기준 좌표계의 원점과 방향이 제각각이기 때문이다(이를 게이지 자유도, gauge freedom이라 한다). 좌표를 맞추지 않고 그냥 겹쳐 놓으면 두 지도가 어긋난다.

**LT-mapper**(Kim et al., ICRA 2022)는 Scan Context로 서로 다른 세션 사이에 겹치는 장소를 찾아 루프를 연결하고, 각 세션의 기준 좌표계를 대표하는 앵커 노드(anchor node)를 그래프에 두어 여러 세션의 궤적을 한꺼번에 정렬한다. 정렬이 끝난 뒤에는 시간 흐름에 따른 환경 변화를 두 층위로 나눈다. 사람이나 차처럼 금방 지나가는 빠른 변화(high dynamic)는 지우고, 창고 적재물 이동이나 공사처럼 천천히 일어나는 변화(low dynamic)는 새로 생긴 점(positive difference)과 사라진 점(negative difference)으로 구분해 변화분 지도(delta map) 형태로 추적·관리한다.

한편 서로 다른 세션이나 측정 주기가 제각각인 다중 센서를 다룰 때 유용한 도구가 **Continuous-Time Estimation**(Furgale et al., ICRA 2012)이다. 이산적인 타임스탬프마다 포즈 노드를 따로 세우는 대신, 로봇의 연속 궤적을 매끄러운 B-spline 기저 함수의 결합으로 표현한다. 이렇게 하면 측정 속도가 수백 Hz인 IMU와 10 Hz인 LiDAR·카메라 측정을 훨씬 적은 수의 제어점(control point) 변수로 매끄럽게 묶을 수 있다. 덕분에 센서 간 시간 오프셋이나 외부 캘리브레이션 파라미터를 궤적과 동시에 정밀 추정할 때도 이 방식이 폭넓게 활용된다.

> **추천 자료**
> - [Kaess et al., "iSAM2: Incremental Smoothing and Mapping Using the Bayes Tree" (IJRR 2012)](https://www.cs.cmu.edu/~kaess/pub/Kaess12ijrr.pdf) — Bayes tree 기반 incremental SLAM의 원본 논문
> - [Kim et al., "Remove, then Revert: Static Point Cloud Map Construction using Multiresolution Range Images" (IROS 2020)](https://github.com/gisbi-kim/removert) — Dynamic point 제거의 실용적 방법. 코드 공개
> - [Kim et al., "LT-mapper: A Modular Framework for LiDAR-based Lifelong Mapping" (ICRA 2022)](https://github.com/gisbi-kim/lt-mapper) — Multi-session SLAM 프레임워크
> - [Chen et al., "SuMa++: Efficient LiDAR-based Semantic SLAM" (IROS 2019)](https://github.com/PRBonn/semantic_suma) — Semantic 정보를 활용한 LiDAR SLAM

---

### 14.16 심화: 정보형 SLAM의 역사

지금까지 현대 SLAM 백엔드의 주류인 factor graph와 증분 평활화를 살펴보았다. 그런데 GTSAM과 iSAM2가 구사하는 '제약의 덧셈 누적, 변수 소거, 희소성 유지'라는 핵심 아이디어는 하루아침에 나온 것이 아니다. 2005년 *Probabilistic Robotics*가 출간되던 무렵, 연구자들은 SLAM의 거대한 불확실성을 어떻게 표현하고 줄일지를 두고 다양한 해법을 실험했다. 로봇 포즈와 랜드마크를 하나의 공분산 행렬로 묶어 돌리던 **EKF-SLAM**, 공분산의 역행렬인 정보 행렬의 덧셈 성질을 파고든 **EIF·GraphSLAM·SEIF**, 그리고 모호한 데이터 연관을 잠재 변수로 풀어낸 **EM mapping**이 그 시절의 주역이었다. 2010년대의 factor graph는 바로 이 고전 정보형 SLAM의 유산을 흡수해 한 단계 진화한 결과물이다(§14.9).

#### 14.16.1 EKF-SLAM (PR §10)

EKF-SLAM의 뿌리는 '확률 지도(stochastic map)' 개념에 닿아 있다. Smith, Self, Cheeseman이 1986~1990년 논문에서 로봇 포즈와 모든 랜드마크 위치를 하나의 확률 변수 벡터로 묶어 제시한 개념이다. 이후 1990년대 Leonard와 Durrant-Whyte, 그리고 Dissanayake 등(2001, IEEE T-RA)이 이를 수학적으로 정식화하면서 2000년대 초반 SLAM의 표준 패러다임으로 자리 잡았다.

**알고리즘 구조**: 로봇 포즈 $x_t = (x, y, \theta)$와 지도 안의 랜드마크 N개 $(m_{j,x}, m_{j,y}, s_j)$를 길이 $(3N+3)$의 거대한 단일 상태 벡터 $y_t$로 이어 붙인 뒤 표준 EKF를 돌린다.

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

이 식에서 Kalman gain $K_t^i$는 $(3N+3) \times 3$ 행렬이므로, 랜드마크를 단 하나만 관측해도 상태 벡터 전체가 동시에 갱신된다. 공분산 행렬의 비대각 원소가 로봇 포즈와 모든 랜드마크 사이의 상관관계를 고스란히 품고 있기 때문에, 어느 한 지점을 정확히 관측하면 다른 랜드마크의 추정 위치까지 연쇄적으로 교정되는 장점이 있다. 하지만 그 대가로 매 스텝 갱신 비용이 $O(N^2)$으로 불어난다.

실전에서 대응 $c_t^i$를 모를 때는 가상의 $(N_t+1)$번째 랜드마크를 맵 끝에 임시로 덧붙여 둔 뒤, 기존 모든 후보와 마할라노비스 거리를 재어 최대 우도(ML) 대응을 고른다. 모든 기존 랜드마크와의 거리가 임계값 $\alpha$를 넘어서면 비로소 새 랜드마크로 정식 등록한다. 문제는 이처럼 매 순간 짝을 확정해 버리는 방식이라 한 번 데이터 연관이 어긋나면 뒤에서 되돌릴 길이 없다는 점이다.

결국 EKF-SLAM은 환경이 넓어지면 근본적인 벽에 부딪힌다. 공분산 행렬 $\Sigma \in \mathbb{R}^{(3N+3) \times (3N+3)}$의 메모리와 계산량이 $N^2$에 비례해, 랜드마크가 100개일 때는 $303 \times 303$ 수준이던 행렬이 1,000개만 되어도 $3003 \times 3003$으로 커진다. 게다가 랜드마크가 쌓일수록 과거 시점의 선형화 오차가 누적되어 필터가 실제보다 과신하며 일관성을 잃는다(Bailey et al. 2006). 과거 로봇 포즈가 매 스텝 상태에서 marginalize되어 사라지므로 루프를 닫아도 과거 궤적 전체를 다시 선형화해 최적화할 수 없기 때문이다.

이 때문에 대규모 지도 작성에서는 평활화(smoothing)나 factor graph 방식이 주류로 자리 잡았다. 다만 EKF-SLAM도 랜드마크 수가 적거나 fiducial 마커 몇 개로 소규모 지도를 꾸릴 때는 여전히 실용적이다. MSCKF 같은 현대의 sliding-window 필터는 EKF의 선형화와 공분산 전파 틀을 유지하되, 랜드마크를 상태에 쌓지 않고 곧바로 marginalize해 버려 $O(N^2)$ 병목을 피해 간다. 데이터 연관에서도 개별 측정마다 ML을 고르는 대신 여러 측정의 짝을 한꺼번에 검증하는 JCBB(Joint Compatibility Branch and Bound) 기법을 함께 쓰기도 한다.

#### 14.16.2 GraphSLAM — 정보형 배치 SLAM (PR §11)

EKF-SLAM은 공분산 표현의 특성 때문에 꽉 찬(dense) 공분산 행렬 $\Sigma$를 매번 통째로 갱신해야 했다. 반면 그 역행렬인 **정보 행렬(information matrix)** $\Omega = \Sigma^{-1}$로 시선을 돌리면, 서로 독립인 관측 factor의 기여가 행렬의 해당 원소에 단순 덧셈으로만 얹힌다. 본래 성긴(sparse) SLAM 그래프의 물리적 구조가 행렬의 모양에 그대로 살아나는 셈이다(물론 이후 motion update나 변수 marginalization을 거치면 앞서 본 fill-in이 생길 수 있다).

##### 정보형의 직관: 스프링-매스 비유

EIF(Extended Information Filter) 계열 SLAM은 불확실성 대신 '정보'를 더해 나가는 양으로 다룬다. 공분산 $\Sigma$와 평균 $\mu$ 대신 정보 행렬 $\Omega = \Sigma^{-1}$과 정보 벡터 $\xi = \Omega \mu$를 기본 변수로 쓴다.

이를 물리적인 **스프링과 질점(spring-mass) 시스템**에 빗대어 보면 수식이 한결 선명해진다. 추정하려는 변수(로봇 포즈와 랜드마크)를 공간에 놓인 질점이라 하면, $\Omega$의 비대각 원소는 두 질점 사이를 연결하는 스프링에 해당한다.
- **Control $u_t$**: 연속한 포즈 $x_{t-1}$과 $x_t$를 잇는 스프링이며, 스프링의 강성(stiffness)은 $R_t^{-1}$이다. 구동 오차(motion noise)가 작을수록 두 포즈를 단단하게 붙잡는 뻣뻣한 스프링이 걸린다.
- **Measurement $z_t^i$**: 관측 시점의 포즈 $x_t$와 랜드마크 $m_j$를 잇는 스프링이며, 강성은 $Q_t^{-1}$이다.
- 랜드마크끼리는 서로를 직접 측정할 일이 없으므로, 둘 사이를 직접 잇는 스프링은 존재하지 않는다.

새로운 관측이 하나 들어올 때마다 해당 포즈와 랜드마크 사이에 새 스프링을 하나 더 달아 주기만 하면 되므로, 정보형 갱신식은 다음과 같은 국소 덧셈으로 표현된다.

$$\Omega \leftarrow \Omega + H_t^{iT} Q_t^{-1} H_t^i, \qquad \xi \leftarrow \xi + H_t^{iT} Q_t^{-1}[z_t^i - h(\mu_t) + H_t^i \mu_t]$$

이처럼 측정 정보는 행렬의 해당 칸에 더하는 것으로 끝나며, 이후 motion update나 marginalization 단계에서만 변수 소거와 fill-in 처리가 일어난다. 오늘날의 factor graph 역시 factor 하나가 소수의 변수 노드만 연결한다는 점에서 이 스프링 비유와 정확히 맞아떨어진다.

##### 4단계 파이프라인

**GraphSLAM**은 이 정보형 표현을 바탕으로 전체 주행 이력의 사후분포 $p(x_{0:t}, m | z_{1:t}, u_{1:t})$를 한꺼번에(offline) 푸는 배치 알고리즘이다(EIF와 SEIF 역시 정보형을 쓰지만 과거 상태를 매 스텝 소거해 나가는 온라인 필터라는 점이 다르다). 전체 절차는 초기화, 그래프 구성, 축소, 풀이의 네 단계로 진행된다.

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

위 3단계의 $\tau(j)$는 랜드마크 $j$를 관측한 모든 포즈 시점의 집합이다. Reduce 단계는 스프링 비유로 해석하면 *랜드마크 $j$ 하나를 함께 바라본 포즈들끼리 새로운 등가 스프링으로 직접 연결한 뒤 그 랜드마크 노드를 그래프에서 떼어 내는* 작업이다. 수학적으로는 §14.9.2에서 다룬 Bundle Adjustment의 Schur complement와 완벽히 같다.

이 변수 소거를 뒷받침하는 핵심 정리가 **Marginalization Lemma**다. 선형 가우시안 모델의 정보형에서 변수 일부($y$)를 주변화(marginalize)해 제거하면, 나머지 변수($x$)의 정보 행렬은 다음과 같이 Schur complement로 닫힌 형태가 된다(이 과정에서 남아 있는 변수들 사이에 원래 없던 스프링, 즉 fill-in이 생길 수 있다).

$$\bar\Omega_{xx} = \Omega_{xx} - \Omega_{xy} \Omega_{yy}^{-1} \Omega_{yx}$$

역사적으로 포즈 간 상대 제약을 전역 최적화하는 틀은 Lu와 Milios(1997)가 먼저 열었고, Thrun과 Montemerlo(2006, IJRR)의 GraphSLAM이 궤적 전체와 지도의 사후분포를 희소 정보 그래프로 정식화해 완성했다.

실전에서 데이터 연관을 모를 때 GraphSLAM은 두 특징점 $(m_j, m_k)$가 같은 랜드마크인지 후보를 검정한 뒤 채택한 제약을 그래프에 더해 다시 최적화한다. 특히 각 관측 factor를 따로 보관해 두면 나중에 오탐으로 밝혀진 제약만 골라 그래프에서 빼고 재최적화할 수도 있다. Sünderhauf와 Protzel(2012)의 **Switchable constraints**는 이 아이디어를 연속 최적화로 확장한 방식이다. 루프 factor를 켜고 끄는 스위치를 연속 변수로 두어 최적화 도중 잘못된 루프를 스스로 꺼 버린다.

이처럼 초기화, factor 구성, 변수 소거, 풀이로 이어지는 GraphSLAM의 파이프라인은 현대 배치 SLAM의 원형이다. 여기에 새 factor가 들어올 때마다 QR 분해를 처음부터 다시 하지 않고 Givens rotation으로 증분 갱신하는 기법이 **iSAM**(Kaess et al. 2008)이며, 이를 Bayes tree 자료구조로 발전시켜 영향을 받는 부분만 다시 선형화하는 것이 **iSAM2**(Kaess et al. 2012)다.

#### 14.16.3 SEIF — Sparse Extended Information Filter (PR §12)

정보형 표현을 배치 방식이 아닌 온라인 필터(EIF)로 쓰려 하면 뜻밖의 장애물을 만난다. 측정 갱신은 희소성을 해치지 않고 깔끔하게 덧셈으로 끝나지만, 로봇이 이동해 과거 포즈를 소거하는 motion update를 정확하게 수행하는 순간 정보 행렬 $\Omega$에 수많은 fill-in이 생기며 행렬이 빽빽하게(dense) 변해 버리기 때문이다.

**SEIF**(Sparse Extended Information Filter)는 이 문제를 풀기 위한 온라인 필터다. 현재 활발히 상호작용하는 **활성 특징점(active feature)**의 개수를 일정 수 이하로 제한하고, 기준 밖으로 밀려난 링크를 강제로 끊어 근사하는 **희소화(sparsification)** 단계를 도입했다. 덕분에 지도가 아무리 커져도 매 스텝 갱신 비용이 상수 시간에 머문다. Thrun 등(2004, IJRR)은 Victoria Park 3.5 km 주행 실험에서 SEIF 구현이 EKF-SLAM보다 약 두 배 빠르고 메모리는 4분의 1 미만만 쓰면서도 비슷한 추정 정확도를 냈다고 보고했다.

##### 4단계 업데이트

SEIF는 시간 스텝마다 다음 네 단계를 차례로 밟으며 정보 행렬을 항상 희소하게 유지한다.

```
SEIF_SLAM_known_correspondences(ξ_{t-1}, Ω_{t-1}, μ_{t-1}, u_t, z_t, c_t):
  1. Motion update:       ξ̄_t, Ω̄_t, μ̄_t ← u_t로 정보형 갱신
                          (active feature + robot pose만 변경, sparse 유지)
  2. Measurement update:  Ω_t ← Ω̄_t + Σ_i H_t^{iT} Q_t^{-1} H_t^i  [가산]
                          ξ_t ← ξ̄_t + 해당 항 가산
  3. Sparsification:      일부 active feature를 passive로 강제
                          — robot과 link를 끊고 정보를 인접 노드에 재분배
  4. State estimate:      amortized coordinate descent로
                          active feature 추정만 incremental 갱신
  return ξ_t, Ω_t, μ_t
```

##### Sparsification

3단계 희소화의 핵심은 두 변수 $a, b$ 사이에 걸려 있던 직접적인 의존 고리(스프링)를 잘라 내어 정보 행렬의 원소 $\Omega_{a,b}$를 정확히 0으로 만드는 근사다.

$$\tilde p(a,b,c) = \frac{p(a,c)\, p(b,c)}{p(c)} \quad \Longrightarrow \quad \Omega_{a,b} = 0$$

이는 공통 이웃 $c$가 주어졌을 때 $a$와 $b$가 조건부 독립이 되도록 확률 분포를 투영하는 **KL projection**에 해당한다. 물론 이 강제 절단과 반복적인 선형화가 필터의 확률적 일관성을 얼마나 깎아먹는지는 실제 환경에서 별도로 평가해 보아야 한다.

한편 PR 12장의 SEIF는 랜드마크를 식별자(signature) 없이 $(x, y)$ 2성분으로 두어 전체 상태가 $2N+3$차원이다(§14.16.1의 EKF-SLAM은 PR 10장을 따라 식별자까지 3성분을 두므로 $3N+3$차원). 활성 특징점 수 $K$를 상수로 묶어 두면 motion과 measurement update에서 실제로 건드리는 부분 행렬 크기가 $(2K+3) \times (2K+3)$을 넘지 않으므로, 지도 크기 $N$과 무관하게 상수 시간 갱신이 가능해진다(단, 전역 상태 벡터 $\mu$ 전체를 복원하거나 데이터 연관을 탐색하는 비용은 별도다).

*Probabilistic Robotics*에서는 활성 특징점을 6개 안팎으로 두는 예시를 들지만, 구체적인 수치는 센서 시야각과 지도 밀도에 맞춰 일관성과 계산량을 저울질해 정해야 한다. 이후 Eustice 등(2006, TRO)은 과거 로봇 포즈를 지우지 않고 상태에 남겨 두는 지연 상태(delayed-state) 구조에서는 근사 없이도 정보 행렬이 정확히 희소함을 보였고(**ESDSF**, Exactly Sparse Delayed-State Filter), Walter, Eustice, Leonard(2007, IJRR)는 특징점 기반 SLAM에서도 일관성을 해치지 않으면서 희소성을 지키는 **ESEIF**를 제안했다. (각 단계에서 그래프 연결이 어떻게 바뀌는지는 PR 12장의 그림에 직관적으로 도식화되어 있으며, 초고 기준으로는 Figure 12.2 measurement, 12.3 motion, 12.4 sparsification에 해당한다.)

##### 트리 기반 데이터 연관

정보형의 덧셈 구조는 데이터 연관 문제를 풀 때도 독특한 무기가 된다. 두 특징점 $m_i, m_j$가 사실 같은 물체라는 가설을 다음과 같은 연성 제약(soft constraint)으로 정보 행렬에 살짝 더해 볼 수 있기 때문이다.

$$\Omega \leftarrow \Omega + F_{m_i - m_j}^T C\, F_{m_i - m_j}$$

만약 가설이 틀렸음이 드러나면, 미리 저장해 둔 항을 그대로 빼기만 해도 원래 상태로 깔끔하게 돌아간다. 이 성질을 활용하면 데이터 연관 가설 트리를 A* 탐색처럼 뒤져 볼 수 있지만, 가설이 많아지면 최악의 경우 탐색 공간이 지수적으로 커진다는 한계가 있다. 오늘날에는 잘못된 루프 폐쇄를 걸러내기 위해 앞서 언급한 Switchable constraints(Sünderhauf & Protzel 2012)나 여러 모드의 혼합 분포로 루프를 표현하는 Max-mixtures(Olson & Agarwal 2013)가 널리 쓰인다.

##### 다중 로봇 맵 융합

정보형의 또 다른 매력은 다중 로봇의 지도를 합칠 때 드러난다. 두 로봇 $j, k$의 좌표를 공통 기준계로 정렬했을 때, 두 로봇이 겹치는 사전 정보나 동일한 관측 데이터를 중복해서 사용하지 않았다면 각자의 정보 행렬과 정보 벡터를 단순히 더하는 것만으로 융합이 끝난다.

$$\Omega^{\text{fused}} = \Omega^{j \leftarrow k\text{-aligned}} + \Omega^k, \qquad \xi^{\text{fused}} = \xi^{j \leftarrow k\text{-aligned}} + \xi^k$$

만약 두 로봇 사이에 이미 공유한 관측 정보가 있는데도 이를 빼지 않고 그대로 더해 버리면, 같은 증거를 두 번 세는 셈이 되어 실제보다 지나치게 자신만만한(overconfident) 추정이 나오므로 주의해야 한다. 공분산 표현에서는 이를 막기 위해 covariance intersection 같은 보수적 융합 기법을 쓴다. Nettleton 등(2003)이 정보형 기반의 분산 융합을 체계적으로 다뤘으며, 현대의 DDF-SAM, Kimera-Multi, Swarm-SLAM은 통신 대역폭과 중복 정보 배제 방식에 따라 각기 다른 전략을 택한다(§14.13).

요컨대 SEIF는 계산량을 잡기 위해 링크를 끊는 근사를 택한 온라인 정보 필터이고, iSAM2는 정확한 구조를 Bayes tree로 관리하며 증분 평활화를 한다. 물론 iSAM2 역시 비선형 문제에서는 어느 시점에 어디를 선형화하고 재선형화하느냐에 따라 해가 미묘하게 달라진다. VINS-Mono·OKVIS의 sliding-window marginalization과 MSCKF의 특징점 소거 역시 상태가 무한정 커지지 않게 막으면서 행렬의 희소성과 계산 효율을 지키려는 같은 맥락의 해법들이다(§14.9.2).

#### 14.16.4 EM Mapping

앞선 EKF-SLAM, EIF, SEIF는 모두 데이터 연관을 이미 알거나 최대 우도(ML)로 매 순간 하나씩 확정 지었다. 하지만 비슷한 기둥이나 문이 늘어선 환경에서는 모호한 데이터 연관을 섣불리 단정 짓는 대신 확률 분포로 품고 가는 접근도 시도되었다. MHT와 JPDA가 복수 가설과 확률적 가중치를 썼고 FastSLAM이 입자마다 개별 연관을 들고 다녔다면, **EM Mapping**은 아예 모르는 데이터 연관을 기댓값 최대화(EM) 알고리즘의 잠재 변수(latent variable)로 놓았다. 지도 추정과 위치(연관) 추정을 번갈아 반복함으로써 당장 판가름하기 모호한 측정도 버리지 않고 끝까지 살려 쓰는 전략이며, Thrun, Fox, Burgard(1998, Machine Learning)의 원형은 실제 박물관 안내 로봇 RHINO(Burgard et al. 1999)에 적용되어 활약했다.

##### E-step / M-step 구조

EM Mapping은 현재 지도 추정치를 바탕으로 전 구간 궤적의 확률 분포를 구하는 E-step(forward-backward)과, 그 분포를 가중치로 삼아 지도를 다시 그리는 M-step을 수렴할 때까지 번갈아 실행한다.

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

E-step의 $\alpha$는 과거에서 현재로 나아가는 순방향 위치 추정(Markov localization)이고, $\beta$는 미래에 들어온 관측 데이터를 거꾸로 되돌려 과거의 belief를 수정하는 역방향 계산이다. 바로 이 $\beta$ 항 덕분에 로봇이 한 바퀴를 돌아 출발점으로 되돌아오면(루프 폐쇄), 과거 시점의 자세 belief까지 거슬러 올라가며 일제히 보정된다. 이 forward-backward 구조는 은닉 마르코프 모델(HMM)의 Baum-Welch 알고리즘 E-step과 정확히 같은 계산이다.

이어지는 M-step에서는 각 격자 셀이 특정 속성 $l$(예: 점유 또는 빈 공간)로 관측된 기대 횟수를 그 셀이 관측된 전체 횟수로 나누어 지도를 갱신하며, 이때 belief 확률을 가중치로 쓴다. 원전(PR 초고 13장)의 소규모 예제에서는 약 10회 반복 만에 거의 수렴하는 모습을 보였으나, 실제 필요한 반복 횟수는 데이터의 길이와 복잡도에 따라 달라진다.

##### Layered EM Mapping

기본 `EM_mapping`에는 미묘한 약점이 하나 있다. M-step에서 센서 빔 원뿔 안의 각 셀을 따로따로 추정하다 보니 지도의 국소적인 기하학적 일관성이 깨지기 쉽다. **Layered EM**은 이를 보완한 개량형이다. 짧은 구간을 움직이며 모은 관측으로 먼저 작은 로컬 점유 격자 지도(local map)를 각각 빚어 놓은 뒤, EM이 셀 하나하나가 아니라 그 로컬 맵들의 *상대 위치*만 최적화하도록 문제를 바꾼다. 이때 EM이 잘못된 국소 최댓값(local maximum)에 빠지지 않도록 온도 파라미터 $\sigma$를 1.0에서 출발해 0에 가깝게 서서히 식혀 가는 **Deterministic annealing**을 결합한다.

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

참고로 이처럼 처음에는 완만하고 풀기 쉬운 목적 함수에서 시작해 점차 원래의 날카로운 문제로 옮겨 가며 나쁜 국소해를 피하는 전략은, 현대 로보틱스 최적화의 **GNC**(Graduated Non-Convexity, Yang et al. 2020)나 robust kernel scheduling과 철학이 맞닿아 있다.

##### EM Mapping이 주류에서 벗어난 이유

이처럼 우아한 확률적 토대에도 불구하고 `EM_mapping`과 `layered_EM_mapping`은 오늘날의 실전 SLAM 시스템에서 거의 자취를 감추었다.
- 자세 추정(E-step)과 지도 추정(M-step)을 번갈아 푸는 대신, **factor graph** 위에서 포즈와 지도를 동시에 최적화하는 방식이 훨씬 빠르고 정확하기 때문이다.
- 전체 주행 데이터를 모아 놓고 여러 번 왕복해야 하는 batch·offline 방식이라 실시간 주행 중 매핑에 적용하기 어렵다.
- 실제로 2D LiDAR SLAM의 대표격인 **Cartographer**(Hess et al. 2016)는 EM 없이 빠른 scan matching과 pose graph 최적화를 결합해 직접 해를 구했고, **GMapping**(Grisetti et al. 2007)은 scan matching으로 입자 필터의 제안 분포를 정교하게 다듬은 Rao-Blackwellized particle filter(RBPF)로 실시간 2D 매핑을 해냈다.

흥미롭게도 layered EM에서 "작은 로컬 맵을 먼저 빚은 뒤 전체를 정렬한다"는 핵심 구조만큼은 현대 Cartographer의 local/global SLAM 이원화 설계와도 궤를 같이한다.

#### 14.16.5 Cycle Posterior

스텝마다 가장 그럴듯한 단일 추정치(ML)만 취해 지도를 그려 나가는 순차 매퍼(incremental ML mapper)는 구현이 가볍지만 결정적인 약점이 있다. 오도메트리 오차가 조금만 커져도 지도가 크게 어긋나며, 나중에 출발점으로 되돌아와도 이미 그려 버린 과거 궤적을 거슬러 고칠 재간이 없다. **Cycle Posterior** 방식은 ML 매퍼 바로 옆에 로봇 자세의 전체 사후분포를 추적하는 추정기를 *나란히* 돌려 이 두 한계를 돌파하려는 초기 시도다.

```
Incremental Mapping with Posterior Estimation:
  1. incremental_ML_mapping(o, a, s, m)  → ⟨m', s'⟩    [ML 갱신]
  2. Bel(s') = η P(o|s') ∫ P(s'|a,s) Bel(s) ds          [사후분포 한 스텝]
  3. s'' = argmax Bel(s')                                 [사후 mode]
  4. s'' ≠ s'  →  cycle closure 검출
                   s'' − s'를 cycle 경로 따라 선형 분배
  5. incremental_ML_mapping을 시간 역방향으로 재실행       [nested ML refinement]
```

로봇이 미지의 복도를 지날 때는 자세 사후분포 $\text{Bel}(s')$가 넓게 퍼져 있다. 그러다 이미 알고 있는 장소로 돌아오는 순간 사후분포가 갑자기 한 점으로 뾰족하게 모인다. 알고리즘은 이 급격한 수렴을 루프 폐쇄(cycle closure)의 신호로 읽고, 새로 잡힌 사후 최빈값 $s''$과 기존 ML 추정치 $s'$ 사이의 오차를 루프 경로 전체에 선형으로 배분한 뒤 지도를 시간 역방향으로 다시 다듬는다. 원전에서는 사후분포 추정기를 MCL로 구현해 오도메트리 센서를 전혀 쓰지 않은 데이터에서도 거대한 순환 경로를 닫는 데 성공했다. Cycle Posterior는 "장소를 알아보는 단계"와 "과거 궤적을 되감아 고치는 단계"를 한 루프 안에 묶어 낸 초기 사례이며, 그 문제의식은 지금도 이어진다. 오늘날의 SLAM 역시 place recognition(감지)과 graph optimization(보정)으로 역할을 분담하고, §14.15.1의 증분 평활화가 루프 확인 시 영향권만 골라 재최적화하기 때문이다.

#### 14.16.6 정리: 무엇이 살아남았나

고전 정보형·확률적 SLAM 알고리즘들이 남긴 핵심 아이디어가 현대 SLAM 아키텍처에 어떻게 이어지고 있는지 정리하면 다음과 같다.

| PR 알고리즘 | 핵심 기여 | 비교할 현대 구조 | 관계의 범위 |
|---|---|---|---|
| EKF-SLAM | 통합 landmark state, off-diagonal covariance | MSCKF류 filter, fiducial mapping | EKF를 공유하지만 state 구성은 다름 |
| GraphSLAM | 정보형 factor 가산, variable elimination, full trajectory | GTSAM, g2o, Ceres, iSAM2 | sparse least squares와 smoothing 관점이 이어짐 |
| SEIF | bounded active set, sparsification | ESEIF, bounded-state estimator | ESEIF는 SEIF의 일관성 문제를 고친 변형이고, bounded-state estimator는 별도 방법임 |
| EM Mapping | latent data association, forward-backward localization, submap | EM 기반 mapping, submap system | 통계 기법과 구조가 비슷한 정도이고, 직접 이어진 방법은 아님 |
| Cycle Posterior | online closure detection과 correction 결합 | place recognition + graph optimization | 기능 분할이 유사함 |

이처럼 오도메트리에서 출발해 Visual·LiDAR SLAM, 다중 센서 융합, 백엔드 최적화와 관측가능성, 그리고 고전 정보형 SLAM의 계보까지 짚어 보았다. 이름은 달라도 이 모든 기법은 결국 불확실한 센서 데이터 속에서 일관된 궤적과 지도를 동시에 세운다는 하나의 목표를 향해 수렴한다.

> **참고 자료**
> - [Thrun et al., "Simultaneous Localization and Mapping with Sparse Extended Information Filters" (IJRR 2004)](https://journals.sagepub.com/doi/10.1177/0278364904045479) — SEIF 원본 논문
> - [Thrun & Montemerlo, "The GraphSLAM Algorithm with Applications to Large-Scale Mapping of Urban Structures" (IJRR 2006)](https://journals.sagepub.com/doi/10.1177/0278364906065387) — EIF/GraphSLAM 정식화
> - [Dissanayake et al., "A Solution to the Simultaneous Localization and Map Building (SLAM) Problem" (IEEE T-RA 2001)](https://ieeexplore.ieee.org/document/938381) — EKF-SLAM 고전 정식화
> - [Kaess et al., "iSAM2: Incremental Smoothing and Mapping Using the Bayes Tree" (IJRR 2012)](https://www.cs.cmu.edu/~kaess/pub/Kaess12ijrr.pdf) — incremental smoothing과 Bayes tree의 원 논문
