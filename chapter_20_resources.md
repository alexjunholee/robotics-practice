# 19장 추천 자료

도구와 환경이 손에 익었다면 이제 본격적으로 배경지식을 쌓고 최신 연구를 따라갈 차례다. 이 장에서는 Spatial AI와 로보틱스를 공부할 때 길잡이가 되어 줄 교과서, 온라인 강의, 유튜브 채널, 필독 논문, GitHub 저장소를 주제별로 모았다. 목록이 방대하므로 처음부터 끝까지 훑으려 하기보다, 19.7절의 수준별 학습 경로를 먼저 확인해 지금 자신에게 필요한 자료부터 골라 보기를 권한다.

## 19.1 교과서

교과서는 첫 페이지부터 정독하기보다, 연구나 구현 도중 원리가 막힐 때마다 꺼내 보는 기준서로 삼을 때 빛을 발한다.

### Computer Vision

**Multiple View Geometry in Computer Vision** (Hartley & Zisserman)
- 다시점 기하학의 표준으로 꼽히는 핵심 교재다.
- 카메라 모델, Epipolar Geometry, 3D 복원의 수학적 토대를 엄밀하게 다룬다.
- 분량이 방대하고 수식이 깊으므로 처음부터 다 읽기보다 당장 필요한 장만 골라 읽어도 충분하다.
- 링크: [Cambridge University Press](https://www.cambridge.org/core/books/multiple-view-geometry-in-computer-vision/0B6F289C78B2B23F596CAA76D3D43F7A)
- 저자 홈페이지에서 일부 장의 PDF를 내려받을 수 있다: https://www.robots.ox.ac.uk/~vgg/hzbook/

**Computer Vision: Algorithms and Applications** (Szeliski)
- 컴퓨터 비전 전반을 폭넓게 훑는 교과서다.
- 2022년에 출간된 2판에는 딥러닝 기반 비전을 다룬 장도 보강되었다.
- 무료 PDF: https://szeliski.org/Book/

### Robotics

**Probabilistic Robotics** (Thrun, Burgard, Fox)
- 확률적 로보틱스의 고전이자 표준 교재다.
- Kalman Filter, Particle Filter, SLAM의 확률적 정식화를 차근차근 설명한다.
- SLAM을 본격적으로 연구한다면 반드시 한 번은 짚고 넘어갈 책이다.
- 링크: [MIT Press](https://mitpress.mit.edu/9780262201629/probabilistic-robotics/)
- 공식 무료 PDF는 제공되지 않는다.

**State Estimation for Robotics** (Tim Barfoot)
- 상태 추정 이론을 한 단계 깊게 파고드는 심화 교재다.
- Lie Groups와 Factor Graph처럼 수식이 까다로운 주제를 다루지만 전개가 친절해 따라가기 좋다.
- 무료 PDF: https://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf

### Deep Learning

**Deep Learning** (Goodfellow, Bengio, Courville)
- 딥러닝의 수학적·이론적 바탕을 정리한 표준 교과서다.
- 무료 온라인(장별 HTML): https://www.deeplearningbook.org/

**Dive into Deep Learning** (d2l.ai)
- 이론 설명과 실행 코드를 나란히 두고 배우는 실습 중심 교재다.
- 무료 인터랙티브 웹북 형태로 제공된다.
- 링크: https://d2l.ai/
- PyTorch, TensorFlow, JAX 버전을 모두 지원한다.

### 수학 보충

**Introduction to Linear Algebra** (Gilbert Strang)
- 선형대수의 핵심 개념을 기하학적 직관과 함께 풀어내는 교재다.
- MIT OCW 동영상 강의를 곁들이면 훨씬 수월하게 이해할 수 있다.
- 링크: https://math.mit.edu/~gs/linearalgebra/ila6/indexila6.html

**Convex Optimization** (Boyd & Vandenberghe)
- 최적화 이론의 표준 교과서로, SLAM과 제어의 최적화 수식을 이해할 때 든든한 바탕이 된다.
- 무료 PDF: https://web.stanford.edu/~boyd/cvxbook/

## 19.2 온라인 강의

책의 수식이 눈에 잘 들어오지 않거나 코드 실습 감각을 먼저 익히고 싶다면, 잘 구성된 대학 강의를 함께 들으면 훨씬 빨리 감을 잡을 수 있다.

### Computer Vision

**CS231n: Convolutional Neural Networks for Visual Recognition** (Stanford)
- 딥러닝 기반 컴퓨터 비전의 기초를 다지는 대표 강의다.
- 강의 자료와 영상이 모두 무료로 공개되어 있다.
- 강의 영상: https://www.youtube.com/playlist?list=PL3FW7Lu3i5JvHM8ljYj-zLfQRF3EO8sYv
- 강의 노트: https://cs231n.github.io/

**CS231A: Computer Vision, From 3D Reconstruction to Recognition** (Stanford)
- 3D 비전에 초점을 맞춘 강의다.
- 기하학적 컴퓨터 비전의 흐름을 잡기에 알맞다.
- 강의 자료: https://web.stanford.edu/class/cs231a/

### SLAM

**Cyrill Stachniss SLAM Course** (YouTube)
- Freiburg 대학의 Robot Mapping 강의(2013/14 겨울학기) 녹화본으로, Bayes filter부터 graph-based SLAM까지 한 흐름으로 꿰어 준다.
- YouTube: https://www.youtube.com/playlist?list=PLgnQpQtFTOGQrZ4O5QzbIHgl3b1JHimN_

**Multiple View Geometry** (TUM, Daniel Cremers 교수)
- YouTube에 공개된 강의로, 수학적 엄밀함을 놓치지 않으면서 다시점 기하학을 깊이 있게 다룬다.
- YouTube: https://www.youtube.com/playlist?list=PLTBdjV_4f-EJn6udZ34tht9EVIW7lbeo4

**SLAM 입문 (한국어)**:
- 국내 SLAM 연구자 모임인 SLAM KR에서 정리한 한국어 스터디 자료다.

### ROS

**ROS2 공식 튜토리얼**
- 링크: https://docs.ros.org/en/humble/Tutorials.html (Humble 기준)
- Jazzy나 Lyrical 등 다른 배포판을 쓴다면 문서 상단의 버전 메뉴에서 해당 배포판을 선택한다.

**The Construct** (온라인 플랫폼)
- 브라우저 환경에서 실습할 수 있는 ROS 전문 강의 플랫폼이다.
- 일부 기초 과정은 무료로 공개되어 있다.
- 링크: https://www.theconstruct.ai/

### 딥러닝 기초

**CS229: Machine Learning** (Stanford, Andrew Ng)
- 딥러닝으로 넘어가기 전에 머신러닝의 기본 원리를 다지기에 좋은 입문 강의다.
- YouTube: https://www.youtube.com/playlist?list=PLoROMvodv4rMiGQp3WXShtMGgzqpfVfbU

**Neural Networks: Zero to Hero** (Andrej Karpathy)
- 라이브러리에 기대지 않고 신경망을 밑바닥부터 직접 구현하며 원리를 체득하는 시리즈다.
- YouTube: https://www.youtube.com/playlist?list=PLAqhIrjkxbuWI23v9cThsA9GvCAUhRvKZ

## 19.3 유튜브 채널 추천

교과서나 한 학기 분량의 강의가 부담스러울 때는, 개념 하나를 10~30분 단위로 깔끔하게 짚어 주는 아래 유튜브 채널들이 좋은 길잡이가 된다.

| 채널 | 주제 | 특징 |
| --- | --- | --- |
| Cyrill Stachniss | SLAM, Robotics | SLAM과 로보틱스 핵심 개념을 강의 단위로 차근차근 설명 |
| First Principles of Computer Vision (Shree Nayar) | Computer Vision | 영상 형성부터 복원까지 CV 기초를 개념별로 명쾌하게 해설 |
| Andrej Karpathy | Deep Learning, AI | 전 Tesla AI Director가 신경망을 밑바닥부터 직접 구현하며 설명 |
| Yannic Kilcher | 논문 리뷰 | 주요 ML/AI 논문을 한 편씩 뜯어보며 맥락과 수식을 짚어 줌 |
| Two Minute Papers | AI 연구 동향 | 최신 연구 성과를 짧은 영상으로 빠르게 훑어볼 수 있음 |
| 3Blue1Brown | 수학 시각화 | 선형대수와 미적분을 시각적으로 풀어내 수학이 막힐 때 유용 |
| Computerphile | CS 전반 | 컴퓨터과학의 다양한 원리를 알기 쉽게 소개 |
| sentdex | Python, ML | Python을 활용한 ML·로보틱스 코드 중심 실습 영상 |
| The Coding Train | 알고리즘 시각화 | 여러 알고리즘을 실시간으로 코딩하며 직관적인 그림으로 보여 줌 |

각 채널의 바로가기 링크는 아래와 같다.
- Cyrill Stachniss: https://www.youtube.com/@CyrillStachniss
- First Principles of Computer Vision: https://www.youtube.com/@firstprinciplesofcomputerv3258
- Andrej Karpathy: https://www.youtube.com/@AndrejKarpathy
- Yannic Kilcher: https://www.youtube.com/@YannicKilcher
- Two Minute Papers: https://www.youtube.com/@TwoMinutePapers
- 3Blue1Brown: https://www.youtube.com/@3blue1brown
- Computerphile: https://www.youtube.com/@Computerphile
- sentdex: https://www.youtube.com/@sentdex
- The Coding Train: https://www.youtube.com/@TheCodingTrain

## 19.4 논문 읽기

### 어떻게 읽을 것인가?

논문 한 편을 붙잡고 첫 줄부터 마지막 줄까지 정독하다 보면 금세 지치기 쉽다. 논문을 고르는 기준부터 Keshav의 3-pass 읽기, 5 Cs 정리, reviewer 관점, CCC 렌즈까지 실전 독법은 [「연구노트」 6–15장 — 읽기](../research-notes/guide.html#chapter-6)에서 체계적으로 다룬다.

### 필독 논문 리스트

분야의 흐름을 이해하려면 최신 논문만 쫓기보다, 각 갈래의 전환점이 된 대표 논문을 먼저 짚어 보는 게 좋다. 아래 목록은 고전 SLAM부터 딥러닝, 객체 탐지, Foundation Model, 최근 3D·로봇 연구까지 핵심 이정표가 되는 논문들이다.

**Classical CV/SLAM**:
- ORB-SLAM: Mur-Artal et al., 2015 — [arXiv:1502.00956](https://arxiv.org/abs/1502.00956)
- LOAM: Zhang & Singh, 2014 — [RSS 2014](https://publications.ri.cmu.edu/storage/publications/pub_files/2014/7/Ji_LidarMapping_RSS2014_v8.pdf)
- VINS-Mono: Qin et al., 2018 — [arXiv:1708.03852](https://arxiv.org/abs/1708.03852)

**Deep Learning 기초**:
- ResNet: He et al., CVPR 2016 — [arXiv:1512.03385](https://arxiv.org/abs/1512.03385)
- Transformer (Attention Is All You Need): Vaswani et al., 2017 — [arXiv:1706.03762](https://arxiv.org/abs/1706.03762)
- ViT: Dosovitskiy et al., ICLR 2021 — [arXiv:2010.11929](https://arxiv.org/abs/2010.11929)

**Object Detection**:
- Faster R-CNN: Ren et al., 2015 — [arXiv:1506.01497](https://arxiv.org/abs/1506.01497)
- YOLO (original): Redmon et al., CVPR 2016 — [arXiv:1506.02640](https://arxiv.org/abs/1506.02640)
- DETR: Carion et al., 2020 — [arXiv:2005.12872](https://arxiv.org/abs/2005.12872)

**Foundation Models**:
- CLIP: Radford et al., 2021 — [arXiv:2103.00020](https://arxiv.org/abs/2103.00020)
- SAM (Segment Anything): Kirillov et al., 2023 — [arXiv:2304.02643](https://arxiv.org/abs/2304.02643)
- DINOv2: Oquab et al., TMLR 2024 — [arXiv:2304.07193](https://arxiv.org/abs/2304.07193)

**2023–2024 논문**:
- RT-2: Brohan et al., CoRL 2023 — [arXiv:2307.15818](https://arxiv.org/abs/2307.15818)
- 3D Gaussian Splatting: Kerbl et al., SIGGRAPH 2023 — [arXiv:2308.04079](https://arxiv.org/abs/2308.04079)
- Depth Anything: Yang et al., 2024 — [arXiv:2401.10891](https://arxiv.org/abs/2401.10891)

> 논문을 검색할 때는 주로 [Google Scholar](https://scholar.google.com/), [Semantic Scholar](https://www.semanticscholar.org/), [arXiv](https://arxiv.org/)를 활용한다. 벤치마크 순위와 코드 링크를 함께 묶어 보여주던 Papers With Code는 2025년에 문을 닫았으며, 현재 해당 주소로 접속하면 Hugging Face로 연결된다.

### 논문 작성 도구

직접 논문을 쓸 단계가 되면 수식·표 편집부터 영어 표현 다듬기까지 손이 많이 간다. 아래 도구들을 익혀 두면 작성과 퇴고에 드는 수고를 크게 줄일 수 있다.

> **추천 자료**
> - [Overleaf](https://www.overleaf.com/) — 여럿이 실시간으로 공동 편집할 수 있는 온라인 LaTeX 에디터
> - [Mathpix](https://mathpix.com/) — 수식 스크린샷을 곧바로 LaTeX 코드로 변환해 주는 도구
> - [Detexify](https://detexify.kirelabs.org/classify.html) — 손으로 기호를 그려서 해당 LaTeX 명령어를 찾는 사이트
> - [Tables Generator](https://www.tablesgenerator.com/) — LaTeX/HTML 표를 시각적으로 만들어 주는 생성기
> - [QuillBot](https://quillbot.com/) — 영어 문장을 다른 어조나 표현으로 다듬어 주는 도구
> - [Ludwig](https://ludwig.guru/) — 실제 원어민 문헌에서 쓰이는 영어 표현 용례 검색 엔진
> - [DL Monitor (deeplearn.org)](https://deeplearn.org/) — 주요 학회와 arXiv에 올라오는 딥러닝 논문을 자동으로 추적해 주는 서비스

## 19.5 주요 학회

논문을 읽고 투고할 곳을 정할 때는 각 학회의 성격과 심사 흐름을 먼저 파악해야 한다. 컴퓨터 비전·로보틱스·자율주행 주요 학회의 특징은 [「연구노트」 34장 학회 준비](../research-notes/guide.html#chapter-34)의 *분야별 학회 reference* 표에 정리해 두었고, 발표 준비 요령도 같은 장에서 함께 다룬다.

## 19.6 유용한 GitHub 저장소

논문만 읽어서는 구현의 세부가 잘 잡히지 않을 때가 많다. 널리 검증된 오픈소스 저장소의 코드 구조와 설정 파일을 직접 뜯어보면 이론이 실제 시스템으로 어떻게 옮겨지는지 빠르게 익힐 수 있다.

### SLAM

```
# ORB-SLAM3 — Visual(-Inertial) SLAM의 레퍼런스
https://github.com/UZ-SLAMLab/ORB_SLAM3

# VINS-Fusion — 단안+IMU, 스테레오, 스테레오+IMU 지원
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
# Ultralytics YOLO — YOLOv8·YOLO11 등, 가장 쓰기 쉬운 detection 프레임워크
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

## 19.7 추천 학습 경로

1.4절에서 짚은 전체 흐름을 바탕으로, 입문·중급·고급 세 단계에 맞춰 무엇을 어떤 순서로 공부할지 구체적인 자료와 함께 정리했다. 각 단계마다 목표와 추천 자료, 직접 손으로 익힐 실습 과제, 그리고 다음 단계로 넘어갈 기준(마일스톤)을 함께 두었다.

### 입문 단계 (1~3개월)

**목표**: 기초 도구와 수학적 언어를 익혀, 예제 코드를 스스로 실행하고 수정할 수 있다.

먼저 아래 여섯 가지 기초 주제를 차근차근 다진다.

| 주제 | 학습 내용 | 추천 자료 |
| --- | --- | --- |
| Python 숙달 | 문법, 클래스, 파일 I/O | [점프 투 파이썬](https://wikidocs.net/book/1) (무료, 한국어) |
| NumPy, OpenCV 기초 | 배열 연산, 이미지 읽기/처리 | [OpenCV 공식 튜토리얼](https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html) |
| 선형대수 복습 | 행렬, 고유값, SVD | [3Blue1Brown: Essence of Linear Algebra](https://www.youtube.com/playlist?list=PLZHQObOWTQDPD3MizzM2xVFitgF8hE_ab) |
| 확률/통계 복습 | 베이즈 정리, 가우시안 | [StatQuest](https://www.youtube.com/@statquest) |
| ROS2 기본 | 노드, 토픽, 서비스 | [ROS2 공식 튜토리얼](https://docs.ros.org/en/humble/Tutorials.html) |
| Git 사용법 | commit, branch, PR | [Git 입문](https://nulab.com/ko/learn/software-development/git-tutorial/) (한국어) |

기초를 익히면서 아래 실습 과제를 병행하면 개념이 훨씬 빨리 손에 잡힌다.
- OpenCV로 이미지 처리하기 (grayscale 변환, edge detection, feature 추출)
- 간단한 ROS2 노드 작성하기 (publisher/subscriber)
- 카메라 캘리브레이션 직접 해 보기 — 9장 참고
- 3장과 9장을 읽으면서 좌표 변환과 카메라 모델의 연결 고리 이해하기

**마일스톤**: Python으로 이미지를 읽어 특징점을 추출하고, 두 이미지 사이의 매칭 결과를 화면에 시각화할 수 있다면 입문 단계를 졸업하고 다음 단계로 넘어가도 좋다.

### 중급 단계 (3~6개월)

**목표**: 핵심 논문과 알고리즘의 구조를 이해하고, 공개된 구현을 직접 빌드해 벤치마크 데이터셋에서 재현할 수 있다.

입문기를 지나면 아래 표의 딥러닝과 3D·SLAM 핵심 주제를 본격적으로 다룬다.

| 주제 | 학습 내용 | 추천 자료 |
| --- | --- | --- |
| 딥러닝 기초 (PyTorch) | CNN, 학습, 역전파 | [CS231n](https://www.youtube.com/playlist?list=PL3FW7Lu3i5JvHM8ljYj-zLfQRF3EO8sYv) + [PyTorch 공식 튜토리얼](https://docs.pytorch.org/tutorials/) |
| Object Detection | YOLO, Faster R-CNN | [Ultralytics 문서](https://docs.ultralytics.com/) + 10장 |
| Visual SLAM 이해 | ORB-SLAM3 분석 | [Cyrill Stachniss SLAM 강의](https://www.youtube.com/playlist?list=PLgnQpQtFTOGQrZ4O5QzbIHgl3b1JHimN_) + 14장 |
| 포인트 클라우드 처리 | Open3D 사용법 | [Open3D 문서의 Tutorial 목차](https://www.open3d.org/docs/release/) + 13장 |
| Depth Estimation | 단안 카메라 depth 추정 | 10장 + [Depth Anything 코드](https://github.com/DepthAnything/Depth-Anything-V2) |

이론을 코드로 확인하기 위해 다음 실습들을 진행해 본다.
- KITTI 데이터셋 다루기 — [KITTI 홈페이지](https://www.cvlibs.net/datasets/kitti/)
- YOLOv8 파인튜닝 — 직접 모은 데이터셋으로 다시 학습시켜 보기
- ORB-SLAM3 실행 및 분석 — TUM RGB-D 데이터셋으로 궤적 평가하기
- TUM RGB-D 벤치마크 — ATE(Absolute Trajectory Error)와 RPE(Relative Pose Error) 직접 계산해 보기
- 9~14장을 읽으면서 인식과 기하학의 이론적 배경 다지기

**마일스톤**: ORB-SLAM3를 직접 빌드해 데이터셋에서 실행하고, 추정된 궤적을 ground truth와 정량적으로 비교할 수 있다면 중급 단계를 졸업할 수 있다.

### 고급 단계 (6개월+)

**목표**: 기존 연구의 한계를 짚어 새 아이디어를 제안하고, 통제된 실험으로 검증할 수 있다.

고급 단계에서는 최신 Foundation Model과 3D 재구성 기술을 익히며 실제 연구 사이클에 들어간다.

| 주제 | 학습 내용 | 추천 자료 |
| --- | --- | --- |
| VFM 심화 (1.4절 중급 항목의 확장) | DINOv2, SAM, CLIP | 10~11장 + 논문 직접 읽기 |
| 3D 재구성 심화 | NeRF, 3D Gaussian Splatting | [NeRF Studio](https://github.com/nerfstudio-project/nerfstudio) + 13장 |
| 논문 읽기 및 구현 | 최신 논문 분석 | [Hugging Face Papers](https://huggingface.co/papers/trending) + [Yannic Kilcher 채널](https://www.youtube.com/@YannicKilcher) — *본격은 [「연구노트」 6–15장 — 읽기](../research-notes/guide.html#chapter-6)* |
| 새로운 아이디어 실험 | 가설 수립, 실험 설계 | 연구실 세미나 + 학회 워크숍 참여 — *가설 수립은 [「연구노트」 1–5장 — 시작하기](../research-notes/guide.html#chapter-1)* |
| 벤치마크 평가 | 정량적 비교 | 분야별 표준 벤치마크 (KITTI, ScanNet, Replica 등) — *결과 해석은 [「연구노트」 26장 결과와 논의](../research-notes/guide.html#chapter-26)* |

실제 연구로 이어지도록 아래 과제를 목표로 삼는다.
- 최신 논문 코드 분석 — GitHub에서 코드를 받아 실행 흐름과 하이퍼파라미터 파악하기
- 자체 개선 아이디어 실험 — 특정 모듈이나 손실 함수를 바꿨을 때 결과가 어떻게 달라지는지 검증하기
- 논문 작성 시도 — *[「연구노트」 Part 2 — 쓰기](../research-notes/guide.html#chapter-16)* 참고
- 10~13장을 다시 훑으며 최신 연구 흐름과 내 주제의 접점 파악하기

**마일스톤**: 기존 논문의 방법을 수정하거나 개선한 실험을 설계해 정량적으로 비교하고, 학회 워크숍 제출을 목표로 초고를 완성한다.

### 학습 순서 요약

세 단계의 흐름과 각 단계에서 도달해야 할 감각을 한눈에 보면 다음과 같다.

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

## 19.8 연구 실전

*대학원 수준.* 본격적인 연구 단계에 들어가면 코드를 짜는 일만큼이나 논문을 읽고 쓰고, 실험을 설계하고, 학회에서 발표하며, 동료 심사에 참여하는 일이 중요해진다. 논문 작성과 학회 활동 전반은 [research-notes](../research-notes/guide.html)에서, 박사과정의 장기 운영 전략은 [grad-notes](../grad-notes/guide.html)에서 더 깊이 다룬다.

### 19.8.0 연구자 마인드셋

기술 스택만큼이나 오래 지치지 않고 연구를 이어 가는 태도가 중요하다. 아래 자매 가이드의 장들이 초기에 방향을 잡을 때 길잡이가 된다.

- 연구의 방향, 오래 버티는 작업 방식, 도구 → [「대학원노트」 16장 연구가 삶이 되는 조건](../grad-notes/guide.html#chapter-16) §16.5
- 스스로 정해야 하는 대학원 생활과 장기전 → [「대학원노트」 13장 자율성](../grad-notes/guide.html#chapter-13) §13.1
- 꾸준한 성장과 옆 사람과 속도를 비교하는 함정 → [「대학원노트」 14장 남과 비교하는 마음 다루기](../grad-notes/guide.html#chapter-14) §14.3
- 떨어뜨릴 이유가 없는 논문 → [「연구노트」 16장 쓰기 전에 정할 것](../research-notes/guide.html#chapter-16) §16.1

### 19.8.1 논문 쓰기

Abstract → Introduction → Related Work → Method → Experiments → Conclusion으로 이어지는 표준 논문 구조와, 서론에서 문제 정의·기존 한계·접근·기여(contribution)를 설득력 있게 엮는 법은 [「연구노트」 Part 2 — 쓰기](../research-notes/guide.html#chapter-16)와 [「연구노트」 23장 서론](../research-notes/guide.html#chapter-23)에 자세히 정리되어 있다.

### 19.8.2 실험 설계와 Ablation

SLAM과 컴퓨터 비전 실험은 핵심 아이디어 하나만 좋아서는 설득력을 얻기 어렵다. 각 구성 요소의 효과를 떼어 내 확인하는 ablation study, 변인 통제, 반복 실험이 반드시 뒤따라야 한다. 이때 비교 대상 모델들은 반드시 같은 데이터, 같은 split, 같은 하드웨어 조건에서 평가해야 한다. 다른 논문에 적힌 기준선(baseline) 숫자를 그대로 옮겨 적으면 실행 환경과 전처리 조건이 달라 공정한 비교로 인정받기 어렵다.

### 19.8.3 학회 발표

발표 길이에 따른 슬라이드 구성과 5분·20분 발표 전략은 [「연구노트」 34장 학회 준비](../research-notes/guide.html#chapter-34)에서, 학회장에서 다른 연구자에게 처음 말을 거는 30초 요령은 [35장](../research-notes/guide.html#chapter-35)에서 살펴볼 수 있다.

### 19.8.4 논문 리뷰 — Peer Review

남의 논문을 건설적으로 비평하는 눈은 내 논문을 단단하게 만드는 훈련이기도 하다. 심사위원 관점에서 논문을 읽고 피드백을 작성하는 법은 [「연구노트」 10장 심사위원으로 읽기](../research-notes/guide.html#chapter-10)에서, 리뷰어 질문에 논리적으로 대응하는 rebuttal 작성법은 [「연구노트」 32장 개정과 답변서](../research-notes/guide.html#chapter-32)에서 다룬다.

### 19.8.5 도구

논문 작성과 서지 관리, 시각 자료 제작에 주로 쓰는 도구들을 목적별로 추렸다.

- LaTeX: Overleaf 또는 로컬 환경 (texlive + vscode)
- 참고 문헌: PDF 리더로 원문에 직접 표시해 두고, Zotero + Better BibTeX로 서지 정보를 체계적으로 관리한다. AI가 만들어 준 요약이나 related work 비교, BibTeX 초안은 반드시 원문과 DOI 메타데이터를 대조해 검증한 뒤 반영한다.
- 파이프라인 그림: TikZ (정밀한 벡터 도식), draw.io (빠른 구조도 제작), Inkscape (SVG 편집)
- 테이블: booktabs (\toprule, \midrule, \bottomrule)
- 알고리즘: algorithm2e
- 수식: notation table을 따로 만들어 논문 전체의 기호 표기를 통일

수식 표기를 일관되게 맞추고 수식을 문장 흐름 안에 자연스럽게 녹여 쓰는 요령은 [「연구노트」 30장 수식·정리·증명 쓰기](../research-notes/guide.html#chapter-30)에서 확인할 수 있다.

논문 읽기와 글쓰기의 핵심 원칙을 압축한 고전 글과 강연도 함께 권한다.

> 추천 자료:
> - [How to Write a Great Research Paper (Simon Peyton Jones, Microsoft Research)](https://www.microsoft.com/en-us/research/academic-program/write-great-research-paper/) — 논문 쓰기의 고전 강연
> - [How to Read a Paper (S. Keshav)](http://ccr.sigcomm.org/online/files/p83-keshavA.pdf) — 3-pass reading method
> - [Tips for Writing Technical Papers (Jennifer Widom, Stanford)](https://cs.stanford.edu/people/widom/paper-writing.html) — 간결한 실전 조언
