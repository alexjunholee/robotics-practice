# Ch.20 — 추천 자료

로보틱스 연구를 시작할 때 도움이 될 교과서, 강의, 논문, 학습 경로를 정리했다. 자료가 너무 많아서 뭘 봐야 할지 모르겠다면, 맨 아래 **학습 경로** 섹션을 먼저 보자.

## 20.1 교과서

### Computer Vision

**Multiple View Geometry in Computer Vision** (Hartley & Zisserman)
- 다시점 기하학의 핵심 교재
- 카메라 모델, Epipolar Geometry, 3D 복원
- 수학적으로 엄밀 — 솔직히 처음부터 끝까지 읽기는 고통스럽지만, 필요한 챕터만 발췌해서 읽으면 된다
- 링크: [Cambridge University Press](https://www.cambridge.org/core/books/multiple-view-geometry-in-computer-vision/0B6F289C78B2B23F596CAA76D3D43F7A)
- 저자 홈페이지에서 일부 챕터 PDF 제공: https://www.robots.ox.ac.uk/~vgg/hzbook/

**Computer Vision: Algorithms and Applications** (Szeliski)
- 포괄적인 CV 교과서
- 최신 버전 (2022)에 딥러닝 포함
- **무료 PDF 제공** — 학생에게는 축복
- 무료 PDF: https://szeliski.org/Book/

### Robotics

**Probabilistic Robotics** (Thrun, Burgard, Fox)
- 확률적 로보틱스의 표준 교재
- Kalman Filter, Particle Filter, SLAM
- 필수 교재 — SLAM을 연구하려면 읽어야 한다
- 링크: [MIT Press](https://mitpress.mit.edu/9780262201629/probabilistic-robotics/)
- PDF는 공식적으로 무료가 아니지만, 저자의 강의 슬라이드가 대부분의 내용을 커버한다

**State Estimation for Robotics** (Tim Barfoot)
- 상태 추정 심화
- Lie Groups, Factor Graph — 수학적으로 깊지만 설명이 친절하다
- **무료 PDF 제공**
- 무료 PDF: http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf

### Deep Learning

**Deep Learning** (Goodfellow, Bengio, Courville)
- 딥러닝 이론 표준 교과서
- **무료 온라인** 제공
- 무료 PDF: https://www.deeplearningbook.org/

**Dive into Deep Learning** (d2l.ai)
- 실습 중심 — 코드와 함께 배운다
- **무료, 인터랙티브**
- 링크: https://d2l.ai/
- PyTorch, TensorFlow, JAX 버전 모두 지원

### 수학 보충

**Introduction to Linear Algebra** (Gilbert Strang)
- 선형대수를 직관적으로 설명하는 명저
- MIT OCW 강의와 함께 보면 효과 극대화
- 링크: https://math.mit.edu/~gs/linearalgebra/ila6/indexila6.html

**Convex Optimization** (Boyd & Vandenberghe)
- 최적화 이론의 표준 교재
- **무료 PDF 제공**
- 무료 PDF: https://web.stanford.edu/~boyd/cvxbook/

## 20.2 온라인 강의

### Computer Vision

**CS231n: Convolutional Neural Networks for Visual Recognition** (Stanford)
- 딥러닝 비전의 기초 — 이 분야를 시작하는 거의 모든 사람이 보는 강의
- 무료 강의 자료, 영상
- 강의 영상: https://www.youtube.com/playlist?list=PL3FW7Lu3i5JvHM8ljYj-zLfQRF3EO8sYv
- 강의 노트: https://cs231n.github.io/

**CS231A: Computer Vision, From 3D Reconstruction to Recognition** (Stanford)
- 3D Vision 중심
- 기하학 기반
- 강의 자료: https://web.stanford.edu/class/cs231a/

### SLAM

**Cyrill Stachniss SLAM Course** (YouTube)
- SLAM 이론 강의 — 독일어 억양이지만 설명이 정말 명확하다
- SLAM 입문자에게 가장 추천하는 강의
- YouTube: https://www.youtube.com/playlist?list=PLgnQpQtFTOGQrZ4O5QzbIHgl3b1JHimN_

**Multiple View Geometry** (TUM, Daniel Cremers 교수)
- YouTube 공개 — 수학적으로 탄탄한 강의
- YouTube: https://www.youtube.com/playlist?list=PLTBdjV_4f-EJn6udZ34tht9EVIW7lbeo4

**SLAM 입문 (한국어)**:
- SLAM KR 커뮤니티의 스터디 자료: https://github.com/slam-kr

### ROS

**ROS2 공식 튜토리얼**
- 가장 최신 정보
- 링크: https://docs.ros.org/en/humble/Tutorials.html (Humble 기준)
- ROS2 Iron/Jazzy 등 다른 버전은 상단 드롭다운에서 변경

**The Construct** (온라인 플랫폼)
- ROS 전문 강의
- 일부 무료
- 링크: https://www.theconstructsim.com/

### 딥러닝 기초

**CS229: Machine Learning** (Stanford, Andrew Ng)
- ML 기초 — 딥러닝 전에 이것부터 보는 것을 추천
- YouTube: https://www.youtube.com/playlist?list=PLoROMvodv4rMiGQp3WXShtMGgzqpfVfbU

**Neural Networks: Zero to Hero** (Andrej Karpathy)
- 신경망을 밑바닥부터 구현하면서 배우기
- 설명이 매우 직관적이고, 코드와 함께 진행
- YouTube: https://www.youtube.com/playlist?list=PLAqhIrjkxbuWI23v9cThsA9GvCAUhRvKZ

## 20.3 유튜브 채널 추천

교과서나 강의보다 가볍게 볼 수 있는 유튜브 채널들이다. 출퇴근길에, 밥 먹으면서, 쉬는 시간에 틀어놓으면 감각이 쌓인다.

| 채널 | 주제 | 특징 |
| --- | --- | --- |
| **Cyrill Stachniss** | SLAM, Robotics | SLAM의 정석 강의. 학부 수업 수준의 체계적 설명 |
| **First Principles of Computer Vision** (Shree Nayar) | Computer Vision | Columbia 교수가 CV 기초를 하나하나 설명. 정말 친절 |
| **Andrej Karpathy** | Deep Learning, AI | Tesla AI Director 출신. Neural Net을 밑바닥부터 구현 |
| **Yannic Kilcher** | 논문 리뷰 | 최신 ML/AI 논문을 매주 리뷰. 논문 읽는 법을 배울 수 있다 |
| **Two Minute Papers** | AI 연구 트렌드 | 최신 연구를 2-3분 영상으로 소개. "What a time to be alive!" |
| **3Blue1Brown** | 수학 시각화 | 선형대수, 미적분을 시각적으로 설명. 수학이 막힐 때 |
| **Computerphile** | CS 전반 | 컴퓨터과학의 다양한 주제를 쉽게 설명 |
| **sentdex** | Python, ML | Python으로 ML/로보틱스 실습. 코드 중심 |
| **The Coding Train** | 알고리즘 시각화 | 알고리즘을 시각적으로 이해. 에너지 넘치는 진행 |

**링크 모음**:
- Cyrill Stachniss: https://www.youtube.com/@CyrillStachniss
- First Principles of Computer Vision: https://www.youtube.com/@firstprinciplesofcomputerv3258
- Andrej Karpathy: https://www.youtube.com/@AndrejKarpathy
- Yannic Kilcher: https://www.youtube.com/@YannicKilcher
- Two Minute Papers: https://www.youtube.com/@TwoMinutePapers
- 3Blue1Brown: https://www.youtube.com/@3blue1brown
- Computerphile: https://www.youtube.com/@Computerphile
- sentdex: https://www.youtube.com/@sentdex
- The Coding Train: https://www.youtube.com/@TheCodingTrain

## 20.4 논문 읽기

### 어떻게 읽을 것인가?

논문 읽기는 별도 가이드에서 본격적으로 다룬다. 왜 읽는가, Keshav 3-pass, 5 Cs, reviewer 시점, CCC 렌즈, 독자 기대 진단까지 일곱 챕터에 걸쳐 [`../research-notes/part1_reading/`](../research-notes/part1_reading/) (ch01–ch07)에 정리되어 있다.

### 필독 논문 리스트

**Classical CV/SLAM**:
- ORB-SLAM: Mur-Artal et al., 2015 — [arXiv:1502.00956](https://arxiv.org/abs/1502.00956)
- LOAM: Zhang & Singh, 2014 — [RSS 2014](https://www.ri.cmu.edu/pub_files/2014/7/Ji_LidarMapping_RSS2014_v8.pdf)
- VINS-Mono: Qin et al., 2018 — [arXiv:1708.03852](https://arxiv.org/abs/1708.03852)

**Deep Learning 기초**:
- ResNet: He et al., 2015 — [arXiv:1512.03385](https://arxiv.org/abs/1512.03385)
- Transformer (Attention Is All You Need): Vaswani et al., 2017 — [arXiv:1706.03762](https://arxiv.org/abs/1706.03762)
- ViT: Dosovitskiy et al., 2020 — [arXiv:2010.11929](https://arxiv.org/abs/2010.11929)

**Object Detection**:
- Faster R-CNN: Ren et al., 2015 — [arXiv:1506.01497](https://arxiv.org/abs/1506.01497)
- YOLO (original): Redmon et al., 2015 — [arXiv:1506.02640](https://arxiv.org/abs/1506.02640)
- DETR: Carion et al., 2020 — [arXiv:2005.12872](https://arxiv.org/abs/2005.12872)

**Foundation Models**:
- CLIP: Radford et al., 2021 — [arXiv:2103.00020](https://arxiv.org/abs/2103.00020)
- SAM (Segment Anything): Kirillov et al., 2023 — [arXiv:2304.02643](https://arxiv.org/abs/2304.02643)
- DINOv2: Oquab et al., 2023 — [arXiv:2304.07193](https://arxiv.org/abs/2304.07193)

**최신 트렌드**:
- RT-2: Brohan et al., 2023 — [arXiv:2307.15818](https://arxiv.org/abs/2307.15818)
- 3D Gaussian Splatting: Kerbl et al., 2023 — [arXiv:2308.14737](https://arxiv.org/abs/2308.14737)
- Depth Anything: Yang et al., 2024 — [arXiv:2401.10891](https://arxiv.org/abs/2401.10891)

> 논문 검색은 [Google Scholar](https://scholar.google.com/), [Semantic Scholar](https://www.semanticscholar.org/), [Papers With Code](https://paperswithcode.com/), [arXiv](https://arxiv.org/)를 활용하자. Papers With Code는 특히 벤치마크 순위와 코드 링크를 같이 보여줘서 편하다.

### 논문 작성 도구

> **추천 자료**
> - [Overleaf](https://www.overleaf.com/) — 온라인 LaTeX 에디터. 논문 공동 작성의 사실상 표준
> - [Mathpix](https://mathpix.com/) — 수식 스크린샷을 LaTeX 코드로 변환
> - [Detexify](http://detexify.kirelabs.org/classify.html) — 손으로 그려서 LaTeX 기호를 검색
> - [Tables Generator](https://www.tablesgenerator.com/) — LaTeX/HTML 테이블 생성기
> - [QuillBot](https://quillbot.com/) — 영어 문장 paraphrasing 도구. 논문 영작에 유용
> - [Ludwig](https://ludwig.guru/) — 영어 표현 검색 엔진. 원어민이 실제로 쓰는 표현을 확인
> - [DL Monitor (deeplearn.org)](https://deeplearn.org/) — 주요 학회/arXiv의 딥러닝 논문을 자동 추적

## 20.5 주요 학회

분야별 학회 reference 표는 별도 가이드에 정리되어 있다 — [`../research-notes/chapter_34_conference_prep.md`](../research-notes/chapter_34_conference_prep.md) 끝의 *분야별 학회 reference* 섹션이 CV·로보틱스·자율주행 학회 일정과 성격을 다룬다.

본 가이드는 SLAM/CV/로보틱스 분야 본체에 집중한다. 학회 일반론(왜 가는가·발표 opener·첫 마디 화법)과 분야별 표는 위 가이드에서 다룬다.

## 20.6 유용한 GitHub 저장소

### SLAM

```
# ORB-SLAM3 — Visual(-Inertial) SLAM의 레퍼런스
https://github.com/UZ-SLAMLab/ORB_SLAM3

# VINS-Fusion — 다중 카메라+IMU 융합
https://github.com/HKUST-Aerial-Robotics/VINS-Fusion

# LIO-SAM — LiDAR-Inertial SLAM (factor graph 기반)
https://github.com/TixiaoShan/LIO-SAM

# FAST-LIO2 — 빠른 LiDAR-Inertial Odometry
https://github.com/hku-mars/FAST_LIO

# RTAB-Map — RGB-D SLAM, 대규모 환경 지원
https://github.com/introlab/rtabmap

# SplaTAM — 3D Gaussian Splatting 기반 SLAM
https://github.com/spla-tam/SplaTAM
```

### Deep Learning

```
# Ultralytics YOLO — YOLOv8/v11, 가장 쓰기 쉬운 detection 프레임워크
https://github.com/ultralytics/ultralytics

# HuggingFace Transformers — NLP/Vision 모델 허브
https://github.com/huggingface/transformers

# OpenMMLab — Detection, Segmentation, 3D 등 종합 프레임워크
https://github.com/open-mmlab

# PyTorch Lightning — 학습 코드 구조화
https://github.com/Lightning-AI/pytorch-lightning

# timm (PyTorch Image Models) — 사전학습된 Vision 모델 모음
https://github.com/huggingface/pytorch-image-models
```

### 3D Vision

```
# Open3D — 포인트 클라우드, 메쉬 처리
https://github.com/isl-org/Open3D

# 3D Gaussian Splatting — 원본 구현
https://github.com/graphdeco-inria/gaussian-splatting

# NeRF Studio — NeRF/3DGS 통합 프레임워크
https://github.com/nerfstudio-project/nerfstudio

# Depth Anything V2 — 범용 depth estimation
https://github.com/DepthAnything/Depth-Anything-V2

# COLMAP — Structure from Motion 파이프라인
https://github.com/colmap/colmap
```

### VFM/VLA

```
# Segment Anything (SAM) — Meta의 범용 세그멘테이션
https://github.com/facebookresearch/segment-anything

# SAM 2 — 비디오까지 확장
https://github.com/facebookresearch/sam2

# DINOv2 — Self-supervised vision features
https://github.com/facebookresearch/dinov2

# Grounded-SAM — 텍스트로 물체 찾기 + 세그멘테이션
https://github.com/IDEA-Research/Grounded-Segment-Anything

# OpenVLA — 오픈소스 Vision-Language-Action 모델
https://github.com/openvla/openvla
```

### ROS / 로봇 개발

```
# ROS2 공식 저장소
https://github.com/ros2

# Nav2 — ROS2 네비게이션 스택
https://github.com/ros-navigation/navigation2

# MoveIt2 — 로봇팔 모션 플래닝
https://github.com/moveit/moveit2

# micro-ROS — 마이크로컨트롤러용 ROS
https://github.com/micro-ROS
```

### 유용한 Awesome 리스트

```
# Awesome SLAM — SLAM 자료 종합
https://github.com/SilenceOverflow/Awesome-SLAM

# Awesome Robotics — 로보틱스 자료 종합
https://github.com/kiloreux/awesome-robotics

# Awesome 3D Gaussian Splatting — 3DGS 논문/코드 모음
https://github.com/MrNeRF/awesome-3D-gaussian-splatting
```

## 20.7 추천 학습 경로

아래는 기존 1.4절에 있던 학습 경로를 확장한 것이다. 각 단계별로 구체적인 자료와 링크를 달았으니, 자기 수준에 맞는 단계부터 시작하면 된다.

### 입문 단계 (1-3개월)

**목표**: 기초 도구 습득 — "일단 뭔가 돌려볼 수 있는 상태"가 되는 것

| 주제 | 학습 내용 | 추천 자료 |
| --- | --- | --- |
| Python 숙달 | 문법, 클래스, 파일 I/O | [점프 투 파이썬](https://wikidocs.net/book/1) (무료, 한국어) |
| NumPy, OpenCV 기초 | 배열 연산, 이미지 읽기/처리 | [OpenCV 공식 튜토리얼](https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html) |
| 선형대수 복습 | 행렬, 고유값, SVD | [3Blue1Brown: Essence of Linear Algebra](https://www.youtube.com/playlist?list=PLZHQObOWTQDPD3MizzM2xVFitgF8hE_ab) |
| 확률/통계 복습 | 베이즈 정리, 가우시안 | [StatQuest](https://www.youtube.com/@statquest) |
| ROS2 기본 | 노드, 토픽, 서비스 | [ROS2 공식 튜토리얼](https://docs.ros.org/en/humble/Tutorials.html) |
| Git 사용법 | commit, branch, PR | [Git 입문](https://backlog.com/git-tutorial/kr/) (한국어) |

**실습 과제**:
- OpenCV로 이미지 처리 (grayscale 변환, edge detection, feature 추출)
- 간단한 ROS2 노드 작성 (publisher/subscriber)
- 카메라 캘리브레이션 수행 — 본 문서 9장 참고
- 본 문서의 **3장, 9장**을 읽으면서 좌표 변환과 카메라 모델을 이해한다

**마일스톤**: Python으로 이미지를 읽어서 특징점을 추출하고, 두 이미지 간 매칭을 시각화할 수 있으면 입문 단계 졸업이다.

### 중급 단계 (3-6개월)

**목표**: 핵심 기술 이해 — "논문을 읽고 코드를 돌려볼 수 있는 상태"

| 주제 | 학습 내용 | 추천 자료 |
| --- | --- | --- |
| 딥러닝 기초 (PyTorch) | CNN, 학습, 역전파 | [CS231n](https://www.youtube.com/playlist?list=PL3FW7Lu3i5JvHM8ljYj-zLfQRF3EO8sYv) + [PyTorch 공식 튜토리얼](https://pytorch.org/tutorials/) |
| Object Detection | YOLO, Faster R-CNN | [Ultralytics 문서](https://docs.ultralytics.com/) + 본 문서 10장 |
| Visual SLAM 이해 | ORB-SLAM3 분석 | [Cyrill Stachniss SLAM 강의](https://www.youtube.com/playlist?list=PLgnQpQtFTOGQrZ4O5QzbIHgl3b1JHimN_) + 본 문서 14장 |
| 포인트 클라우드 처리 | Open3D 사용법 | [Open3D 튜토리얼](http://www.open3d.org/docs/release/tutorial/) + 본 문서 13장 |
| Depth Estimation | 단안 카메라 depth 추정 | 본 문서 10장 + [Depth Anything 코드](https://github.com/DepthAnything/Depth-Anything-V2) |

**실습 과제**:
- KITTI 데이터셋 다루기 — [KITTI 홈페이지](https://www.cvlibs.net/datasets/kitti/)
- YOLOv8 파인튜닝 — 커스텀 데이터셋으로 fine-tuning
- ORB-SLAM3 실행 및 분석 — TUM RGB-D 데이터셋으로 평가
- TUM RGB-D 벤치마크 — ATE, RPE 계산해보기
- 본 문서의 **9-14장**을 읽으면서 이론적 배경을 다진다

**마일스톤**: ORB-SLAM3를 직접 빌드하고 데이터셋으로 돌려서, trajectory를 ground truth와 비교할 수 있으면 중급 단계 졸업이다.

### 고급 단계 (6개월+)

**목표**: 연구 능력 개발 — "새로운 아이디어를 내고 실험할 수 있는 상태"

| 주제 | 학습 내용 | 추천 자료 |
| --- | --- | --- |
| VFM 이해 및 활용 | DINOv2, SAM, CLIP | 본 문서 10-11장 + 논문 직접 읽기 |
| 3D 재구성 심화 | NeRF, 3D Gaussian Splatting | [NeRF Studio](https://github.com/nerfstudio-project/nerfstudio) + 본 문서 13장 |
| 논문 읽기 및 구현 | 최신 논문 분석 | [Papers With Code](https://paperswithcode.com/) + [Yannic Kilcher 채널](https://www.youtube.com/@YannicKilcher) — *본격은 [`../research-notes/part1_reading/`](../research-notes/part1_reading/)* |
| 새로운 아이디어 실험 | 가설 수립, 실험 설계 | 연구실 세미나 + 학회 워크숍 참여 — *본격은 [`../research-notes/part0_starting/`](../research-notes/part0_starting/)* |
| 벤치마크 평가 | 정량적 비교 | 각 분야별 표준 벤치마크 (KITTI, ScanNet, Replica 등) — *결과 해석 frame은 [`../research-notes/part2_writing/E_after/`](../research-notes/part2_writing/E_after/)* |

**실습 과제**:
- 최신 논문 코드 분석 — GitHub에서 코드를 받아 직접 돌려보기
- 자체 개선 아이디어 실험 — "이 부분을 바꾸면 어떨까?" 시도
- 논문 작성 시도 — *본격 frame은 [`../research-notes/part2_writing/`](../research-notes/part2_writing/)* 참고
- 본 문서의 **10-13장**을 읽으면서 최신 연구 방향을 파악한다

**마일스톤**: 기존 논문의 방법을 수정/개선한 실험을 하고, 그 결과를 정량적으로 비교할 수 있으면 고급 단계에 진입한 것이다. 학회 워크숍에 제출할 수 있는 수준이 되는 것을 목표로 하자.

### 학습 순서 요약

```
입문 (1-3개월)                    중급 (3-6개월)                     고급 (6개월+)
─────────────                   ─────────────                    ─────────────
Python + NumPy                  PyTorch + CNN                    VFM (DINOv2, SAM)
OpenCV 기초                     YOLO 파인튜닝                     3DGS / NeRF
선형대수/확률 복습               ORB-SLAM3 분석                    논문 구현
ROS2 기초                       KITTI/TUM 벤치마크                 아이디어 실험
Git 사용법                      포인트 클라우드 (Open3D)           논문 작성
                                Depth Estimation
     ↓                                ↓                                ↓
 "코드를 돌릴 수 있다"          "논문을 읽고 재현한다"           "새 아이디어를 실험한다"
```

## 20.8 연구 실전 mindset (link)

*대학원 수준.* 연구자 마인드셋과 논문 쓰기·실험 설계·학회 발표·리뷰 같은 메타 운영은 별도 가이드 [research-notes](../research-notes/)·[grad-notes](../grad-notes/)에서 본격적으로 다룬다. 본 챕터에서는 분야 instantiation에 해당하는 자리만 짧게 남기고 메타 자리는 link로 안내한다.

핵심 frame 한 줄: 엔지니어는 정체성이고, 연구는 *방향·엔진·도구* 세 layer로 굴러간다 — 도구만 갈고닦으면 5년 후 *날카로운 칼 + 빈 방향*에 도착한다.

### 20.8.0 연구자 마인드셋 (link)

- 방향·엔진·도구 세 layer + 통합 frame → [`gradnotes/p4_ch04_integrated_life.md`](../grad-notes/chapter_17_integrated_life.md) § 5
- 자율성의 무게 + Hyun *모든 것이 optimization* + optimization horizon = 장기전 → [`gradnotes/p4_ch01_autonomy_weight.md`](../grad-notes/chapter_14_autonomy_weight.md) § 1
- 꾸준함 vs 폭발적 성장 + 옆 사람 속도 비교 함정 → [`gradnotes/p4_ch02_comparison_trap.md`](../grad-notes/chapter_15_comparison_trap.md) § 3
- 견고한 기초 — *리젝 이유가 없는 논문* frame → [`part2_writing/A_workflow/ch01_mindset.md`](../research-notes/chapter_16_mindset.md) § 1

### 20.8.1 논문 쓰기 (link)

Abstract → Introduction → Related Work → Method → Experiments → Conclusion 구조, Intro 4단 (문제 정의·기존 한계·접근·contribution), figure/table 먼저 그리기 — 본격 가이드는 [`part2_writing/A_workflow/`](../research-notes/part2_writing/A_workflow/) (mindset·outline·time budget) 및 [`part2_writing/C_sections/ch08_introduction.md`](../research-notes/chapter_23_introduction.md).

### 20.8.2 실험 설계와 Ablation

분야 본체 자리이므로 한 줄로 정리한다 — *Ablation·변인 통제·통계 유의성(최소 3회 반복)·공정 비교(같은 데이터·split·하드웨어)*는 분야 표준 작업 단위다. baseline 숫자를 다른 논문에서 그대로 가져오면 조건이 달라 reject 사유가 잡힌다.

### 20.8.3 학회 발표 (link)

발표 1+1+3+3+1 분 구조, 슬라이드 한 장 한 메시지, 포스터 3m 가독성, 데모 30초~1분 — 본격 가이드는 [`part3_presentations/`](../research-notes/part3_presentations/).

### 20.8.4 논문 리뷰 — Peer Review (link)

리뷰어 관점 체크리스트(novelty·soundness·experiments·clarity·reproducibility), 건설적 피드백, rebuttal — 본격 가이드는 [`part1_reading/ch05_reading_for_review.md`](../research-notes/chapter_10_reading_for_review.md) (reviewer로 읽기) + [`part2_writing/E_after/ch17_revision_rebuttal.md`](../research-notes/chapter_32_revision_rebuttal.md) (rebuttal 작성).

### 20.8.5 도구

- LaTeX: Overleaf 또는 로컬 (texlive + vscode)
- 참고 문헌: PDF 리더 + AI 조합이 효율적 — Acrobat 형광펜 + Claude/GPT로 요약·BibTeX 생성·related work 비교, Google Scholar Cite 버튼, Zotero + Better BibTeX는 100편 이상에서 유용
- 파이프라인 그림: TikZ (정밀), draw.io (빠른 제작), Inkscape (SVG)
- 테이블: booktabs (\toprule, \midrule, \bottomrule)
- 알고리즘: algorithm2e
- 수식: notation table을 따로 만들어 논문 전체에서 통일

표기법(LaTeX·notation·수식) 자리는 [`part2_writing/D_sentence/ch15_math_and_proofs.md`](../research-notes/chapter_30_math_and_proofs.md) (글쓰기 단위 통일)이 메타 layer로 다룬다.

> 추천 자료:
> - [How to Write a Great Research Paper (Simon Peyton Jones, Microsoft Research)](https://www.microsoft.com/en-us/research/academic-program/write-great-research-paper/) — 논문 쓰기의 고전 강연
> - [How to Read a Paper (S. Keshav)](http://ccr.sigcomm.org/online/files/p83-keshavA.pdf) — 3-pass reading method
> - [Tips for Writing Technical Papers (Jennifer Widom, Stanford)](https://cs.stanford.edu/people/widom/paper-writing.html) — 간결한 실전 조언
