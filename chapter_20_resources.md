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

논문 읽기는 처음에 너무 어렵다. 2시간 걸려서 한 편 읽었는데 뭔 소린지 모르겠는 경험, 누구나 한다. 그래서 **3-패스 방법**을 추천한다:

1. **첫 번째 패스** (5-10분)
    - Title, Abstract, Conclusion
    - Figure, Table 훑어보기
    - 핵심 contribution 파악
    - 이 단계에서 "이 논문이 나에게 필요한가?"를 판단한다
2. **두 번째 패스** (1시간)
    - 전체 읽기 (수식 스킵 가능)
    - 방법론 이해
    - 관련 연구 파악
    - Figure를 꼼꼼히 보라 — 저자가 가장 공들인 곳이다
3. **세 번째 패스** (수 시간~며칠)
    - 수식 유도 따라가기
    - 코드 분석
    - 재구현 시도
    - 여기까지 오면 그 논문의 전문가

> 처음에는 일주일에 1편을 3-패스로 완전히 이해하는 게, 매일 1편을 대충 읽는 것보다 낫다. 나중에 속도가 붙으면 자연스럽게 빨라진다.

> AI 활용: 1패스 후에 Claude나 GPT에게 논문 PDF를 주고 "이 논문의 contribution 3줄 요약", "이 수식(Eq.5)을 단계별로 설명", "이 논문과 [비교 논문]의 차이점"을 물어보면 2패스 시간을 크게 줄일 수 있다. 단, AI 요약만 읽고 원문을 안 읽는 건 안 된다 — AI가 미묘한 가정이나 한계를 놓치는 경우가 많다. AI는 이해 보조 도구이지 대체재가 아니다.

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

### Computer Vision

| 학회 | 수준 | 특징 |
| --- | --- | --- |
| **CVPR** | Top | 가장 큰 CV 학회. 매년 6월 |
| **ICCV** | Top | 2년 주기 (홀수년). CVPR과 함께 CV 양대 산맥 |
| **ECCV** | Top | 유럽 중심, 2년 주기 (짝수년) |
| **NeurIPS** | Top | ML 전반, Vision 포함. 매년 12월 |
| **ICML** | Top | ML 전반. 매년 7월 |
| **ICLR** | Top | 딥러닝 중심. 매년 5월 |

### Robotics

| 학회 | 수준 | 특징 |
| --- | --- | --- |
| **ICRA** | Top | IEEE 로보틱스. 가장 큰 로보틱스 학회 |
| **IROS** | Top | IEEE/RSJ. ICRA와 함께 양대 산맥 |
| **RSS** | Top | 소규모, 선별적. 질이 높다 |
| **CoRL** | Top | 로봇 학습 특화. 최근 급성장 |

### 자율주행

| 학회/저널 | 특징 |
| --- | --- |
| **CVPR Workshop** (WAD, OmniCV) | 자율주행 워크숍 |
| **T-IV** | 지능형 차량 저널 |
| **T-ITS** | 교통 시스템 저널 |

> **추천 자료**
> - [CV Conference Deadlines](http://conferences.visionbib.com/Iris-Conferences.html) — 주요 CV/로보틱스 학회 마감일 모음

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
| 논문 읽기 및 구현 | 최신 논문 분석 | [Papers With Code](https://paperswithcode.com/) + [Yannic Kilcher 채널](https://www.youtube.com/@YannicKilcher) |
| 새로운 아이디어 실험 | 가설 수립, 실험 설계 | 연구실 세미나 + 학회 워크숍 참여 |
| 벤치마크 평가 | 정량적 비교 | 각 분야별 표준 벤치마크 (KITTI, ScanNet, Replica 등) |

**실습 과제**:
- 최신 논문 코드 분석 — GitHub에서 코드를 받아 직접 돌려보기
- 자체 개선 아이디어 실험 — "이 부분을 바꾸면 어떨까?" 시도
- 논문 작성 시도 — LaTeX로 4-6페이지짜리 draft 써보기
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

## 20.8 연구 스킬

*대학원 수준.*

### 20.8.0 연구자로서의 마인드셋

논문 쓰는 법, 실험 설계 같은 기술적 스킬은 하면 늘지만, 마인드셋이 잘못 잡히면 몇 년을 해도 제자리다.

**엔지니어는 직업이 아니라 정체성이다**

로보틱스 엔지니어라는 말을 들으면 보통 "로봇 만드는 사람"을 떠올린다. 로봇 팔을 조립하고, SLAM 코드를 돌리고, 모터 드라이버를 디버깅하는 사람. 틀린 말은 아니지만, 그건 엔지니어가 하는 "일"이지 엔지니어가 "무엇인지"에 대한 답은 아니다.

엔지니어는 직업이 아니라 정체성이다. 세상을 보는 방식이다. 문제가 있으면 그걸 모델링하고, 모델링한 문제를 풀고, 가능한 최적의 해를 찾아낸다. 이게 엔지니어가 세상을 대하는 태도이고, 이 태도는 연구실을 나서도 꺼지지 않는다.

**모든 것은 최적화 문제다**

> "이 세상 모든 것이 optimization이다."
>
> — Dr. Dongjin Hyun, Head of Robotics LAB, Hyundai Motor Company

엔지니어링에는 100%가 없다. 0%도 없다. 사이언스는 자연 법칙을 발견하는 학문이다. "빛의 속도는 299,792,458 m/s이다" 같은 단정적인 진술을 만든다. 엔지니어링은 다르다. 우리가 다루는 건 현실 세계의 더럽고 애매한 문제들이다. 센서는 노이즈를 뱉고, 모터는 명령한 토크를 정확히 내지 않고, 환경은 예측 불가능하게 변한다. 이런 문제에는 "정답"이 없다. 있는 건 "이 조건 하에서 가장 나은 선택"뿐이다.

그래서 엔지니어가 하는 일의 본질은 최적화다. 불확실하고 확률적인(probabilistic) 세상을 기댓값으로 모델링하고, 적절한 수준에서 단순화(level of abstraction)를 결정하고, 그 위에서 최적화를 돌리는 것. SLAM에서 센서 노이즈를 가우시안으로 모델링하고 factor graph를 풀어서 최적의 포즈를 추정하는 것이 그렇고, MPC에서 유한 구간의 미래를 예측하고 비용 함수를 최소화하는 제어 입력을 구하는 것이 그렇다.

그런데 이건 로봇 문제에만 해당하는 이야기가 아니다. 점심을 어디서 먹을지 고르는 것도, 오늘 밤을 새서 논문을 쓸지 내일 집중해서 쓸지 결정하는 것도, 대학원에서 어떤 연구 주제를 선택할지도 — 구조가 같다. 불확실한 세상에서, 내가 관측할 수 있는 정보(observable state) 안에서, 기대 보상(expected reward)을 최대화하는 선택지를 찾는 것. 엔지니어의 눈으로 보면 세상의 거의 모든 의사결정이 이 프레임에 들어맞는다.

**근시안적 판단의 정체**

이 관점에서 보면 "근시안적인 판단"이 뭔지도 명확해진다. 우리가 누군가를 보고 "왜 저런 선택을 하지?"라고 생각할 때, 그 사람이 멍청해서가 아니다. 그 사람의 loss function에 장기적 state가 들어 있지 않아서 그런 것이다. 미래의 나에 대한 상태 — 3년 뒤의 커리어, 건강, 관계 — 가 의사결정 모델에 없으면, 지금 눈앞에 보이는 state만 가지고 greedy한 최적화를 하게 된다. 그리고 그 사람의 관점에서는, 자기가 구성한 상태 공간 안에서 최적의 결정을 내린 것이다. 틀린 게 아니라 모델이 부족한 것이다.

연구도 마찬가지다. 이번 주 실험 결과를 빨리 내려고 밤을 새는 것과, 일주일을 투자해서 코드를 제대로 구조화한 뒤 실험하는 것 — 단기 loss function에서는 밤새는 쪽이 최적이지만, 6개월 뒤까지 state를 확장하면 후자가 압도적으로 낫다. 코드가 깨끗하면 이후 실험을 10배 빨리 반복할 수 있으니까. 연구에서 "장기전이 중요하다"는 조언은, 수학적으로 말하면 "optimization horizon을 늘려라"는 이야기다.

그래서 좋은 연구자가 되려면 자기 의사결정 모델의 state space를 의식적으로 확장해야 한다. "지금 이 논문"이 아니라 "3년 뒤의 나", "이 분야의 5년 뒤"까지 state에 넣어야 한다. 물론 먼 미래의 state는 불확실하다. 하지만 불확실하다고 빼버리면 근시안이 되고, 넣되 큰 variance를 부여하면 합리적인 장기 계획이 된다. 칼만 필터에서 process noise를 키우는 것과 같은 원리다.

개인적으로는 이 horizon의 차이가 결국 애티튜드에서 온다고 생각한다. 똑똑한 사람이 빠른 건 맞다. 근데 몇 년 지나고 보면 그 차이가 생각보다 크진 않았다 (물론 여기서 super genius들은 제외하도록 하자). 오래 보면 갈리는 건... 틀린 걸 인정하고, 모르는 걸 모른다고 하고, 고집은 있되 고착되지 않는 것. 3개월 차이는 눈에 보이는데, 3년 지나면 순서가 바뀌어 있는 경우가 꽤 있었다. 애티튜드가 전부다 하는 이야기는 이런 맥락이라고 생각한다.

**그러면 방향은 어디서 오는가**

여기서 중요한 질문이 생긴다. 최적화를 하려면 비용 함수가 있어야 한다. "뭘 최소화할 것인가", "뭘 최대화할 것인가". 연구에서는 논문 수? citation? 졸업 시기? 연봉? 이런 것들은 측정 가능한 메트릭이지만, 그게 정말 최적화해야 할 목적 함수인지는 별개의 문제다.

내가 뭘 원하는지, 어디로 가야 하는지 — 이 질문에 대한 답은 엔지니어링 안에 없다. 선형대수를 아무리 잘 풀어도, SLAM 논문을 100편 읽어도, 이 질문에는 답이 안 나온다. 이건 인문학의 영역이다. 철학, 문학, 역사.

다른 사람들은 나의 거울이라는 말이 있다. 책을 읽는 것은 그 내용을 이해하는 것이기도 하지만, 그 글을 쓴 사람이 어떤 삶을 살았고, 무슨 고민을 했고, 왜 그런 생각에 도달했는지를 들여다보는 것이기도 하다. 그걸 통해 자기 자신을 되돌아볼 수 있다. 내가 이 글을 읽고 무엇을 느꼈는지를 인식하는 것 — 메타인지라는 게 별거 아니고 이게 전부다.

사람은 같은 실수를 반복하는 동물이다. 그건 어쩔 수 없다. 하지만 수천 년간 반복하면서도 여기까지 온 건 기록이라는 게 있었기 때문이다. 글을 쓰고, 지식을 전달하고, 그 지식을 받아들이는 자기 자신을 인지하는 것. 문명의 진보가 여기서 온다. 고전 문학을 읽어도 공감할 수 있는 건, 인간의 DNA가 수천 년 단위로는 거의 변하지 않았기 때문이다. 옛날 사람들이 겪은 고민 — 방향에 대한 불안, 동기의 상실, 비교에서 오는 좌절 — 은 지금 대학원생이 겪는 것과 같다.

그러니까 연구자로서 방향을 잡으려면, 가끔은 논문이 아니라 책을 읽어라. 엔지니어링 밖의 책을. 거기서 자기가 뭘 중요하게 여기는지, 어떤 키워드가 자기 삶에서 반복되는지 — 성장인지, 사람인지, 자유인지, 인정인지 — 힌트를 찾을 수 있다. 정답은 없고, 정답이 없다는 걸 아는 것 자체가 시작이다.

**세 가지: 방향, 엔진, 도구**

정리하면, 연구자로서 오래 가려면 세 가지가 필요하다.

첫째, **방향**. 내가 어디로 가야 하는지. 위에서 말했듯이 이건 엔지니어링 밖에서 찾아야 한다. 방향이 없으면 아무리 열심히 달려도 "열심히 달렸다"는 사실만 남는다.

둘째, **엔진**. 동기, 원동력. 내 마음속에 있는 엔진이 무엇을 연료로 해서 출력을 내는가? 순수한 호기심일 수도 있고, 성장에 대한 갈망일 수도 있고, 사람들과 나누는 교류에서 오는 에너지일 수도 있다. 어떤 사람은 "문제를 풀었을 때의 쾌감"이 연료이고, 어떤 사람은 "내가 만든 게 세상에서 돌아가는 걸 보는 것"이 연료이다. 이건 사람마다 다르고 정답이 없다. 다만 자기 엔진이 뭘로 돌아가는지는 알아야 한다. 모르면 연료가 떨어졌을 때 왜 멈췄는지도 모르고, 다시 시동을 걸 수도 없다.

셋째, **도구**. 학위, 프로그래밍 실력, 수학, 연구실 인프라, 동료, 가족의 지원. 이 문서에서 21개 챕터에 걸쳐 다루는 기술적 내용이 전부 여기에 해당한다. 도구는 중요하다. 칼이 날카로워야 나무를 벨 수 있다. 하지만 방향과 엔진 없이 도구만 갈고닦으면, 어디로 가야 할지 모르는 채로 칼만 날카로운 사람이 된다. 그리고 그런 사람은 생각보다 많다.

이 문서의 나머지 부분은 전부 "도구"에 대한 이야기다. 방향과 엔진은 각자가 찾아야 한다.

---

아래는 연구를 시작하는 단계에서의 구체적인 전략이다. (이 부분은 [Giseop Kim, "연구 초입자를 위한 지속가능한 성장 가이드"](https://gsk1m.github.io/productivity/2024/05/25/entering-research.html)의 내용을 참고하여 재구성했다.)

**꾸준함이 폭발적 성장보다 낫다**

연구 초반에는 성장이 느리다. 논문을 읽어도 이해가 안 되고, 코드를 짜도 안 돌아가고, 실험 결과가 기대와 다르다. 이게 정상이다. 한 주기에 하나씩, 일관된 속도로, 지치지 않게 밀고 나가는 것이 중요하다. 한 달에 논문 10편을 읽고 번아웃되는 것보다, 매주 1편을 6개월 동안 꾸준히 읽는 쪽이 낫다. 선형 성장이 복리로 돌아온다.

**옆 사람 속도에 흔들리지 마라**

같이 입학한 동기가 벌써 논문을 냈다고, 옆 연구실이 더 좋은 장비를 가졌다고 조급해지면 안 된다. 연구 분야마다 성과가 나오는 속도가 다르고, 같은 분야 안에서도 주제에 따라 천차만별이다. 비교 대상은 어제의 나 자신이어야 한다. 타인의 연구 철학과 노력 방식은 배우되, 숫자(논문 편수, citation)에 집착하면 방향을 잃는다.

**논문은 대작보다 견고한 기초가 먼저다**

첫 논문에서 Nature를 노리는 것보다, "리젝 이유가 없는 논문"을 목표로 하는 편이 현실적이다. 실험이 재현 가능하고, 비교가 공정하고, 주장이 데이터로 뒷받침되는 논문. 화려한 novelty보다 빈틈없는 완성도가 첫 논문에서는 더 중요하다. 하나의 논문에는 하나의 핵심 메시지만 담아라. 제목을 먼저 지어보고 — 그 제목이 논문의 contribution을 한 문장으로 요약하는지 확인하라.

**이론 공부보다 연구를 해야 연구를 잘하게 된다**

교과서를 다 읽고 시작하려면 영원히 시작하지 못한다. "이 개념을 완벽히 이해한 다음에 실험하겠다"는 마인드가 가장 위험하다. 필요한 만큼만 공부하고 바로 실험에 뛰어들어라. 모르는 건 하다가 막힐 때 찾아보면 된다. 농사는 씨를 뿌려야 시작이지, 농업 이론서를 다 읽는 게 시작이 아니다. 기술 부채는 쌓이겠지만, 그건 나중에 하나씩 갚으면 된다. 안 갚으면 안 되지만, 시작을 미루는 것보다는 낫다.

**지적 성실성**

좋은 연구자는 자기가 틀렸을 수 있다는 가능성을 항상 열어둔다. 실험 결과가 가설과 다르면, 결과를 의심하기 전에 가설을 의심하라. 자기 마음을 바꿀 수 있는 것이 지적으로 성실한 태도다. 메타인지는 다양한 관점의 논문을 읽어야 키울 수 있다. "내가 모르는 것이 무엇인지 아는 것"이 연구 역량의 핵심이다.

**20.8.1 논문 쓰기**
- 구조: Abstract → Introduction → Related Work → Method → Experiments → Conclusion
- Introduction 쓰는 법: (1) 문제 정의, (2) 기존 방법의 한계, (3) 우리의 접근, (4) contribution 목록
- 실험 섹션: baseline 비교, ablation study, 정성적 결과
- 흔한 실수: contribution이 불명확, 실험이 불공정한 비교, related work에서 핵심 논문 빠뜨림
- 논문을 쓰기 전에 figure/table을 먼저 그려라. 스토리가 잡힌다

**20.8.2 실험 설계와 Ablation Study**
- Ablation: 모델/시스템의 각 구성 요소를 하나씩 빼면서 기여도를 측정
- 변인 통제: 한 번에 하나만 바꿔라. 두 개 이상 동시에 바꾸면 어떤 것의 효과인지 모른다
- 통계적 유의성: 같은 실험을 여러 번(최소 3회) 반복하고 평균/표준편차 보고
- 공정한 비교: 같은 데이터, 같은 split, 같은 하드웨어에서 비교. 다른 논문의 숫자를 그대로 가져오면 조건이 다를 수 있다

**20.8.3 학회 발표**
- 발표 구조: 문제(1분) → 기존 한계(1분) → 제안 방법(3분) → 실험 결과(3분) → 결론(1분)
- 슬라이드: 글자 줄이고 그림/다이어그램 위주. 한 슬라이드에 하나의 메시지
- 포스터: 3m 거리에서 제목과 핵심 figure가 보여야 한다
- 데모 영상: 30초~1분. 처음에 결과 요약, 그다음 상세

**20.8.4 논문 리뷰 (Peer Review)**
- 리뷰어 관점에서 체크할 것: novelty, technical soundness, experiments, clarity, reproducibility
- 건설적 피드백: "이 부분이 잘못됐다" 대신 "이 부분은 X를 추가하면 더 강해질 것 같다"
- Rebuttal 쓰기: 리뷰어의 핵심 우려를 하나씩 짚어서 답변. 감정적 대응 금지

**20.8.5 도구**
- LaTeX: Overleaf 또는 로컬 (texlive + vscode)
- 참고 문헌 관리: 예전에는 Mendeley, Zotero 같은 전용 도구를 썼지만, 요즘은 **PDF 리더 + AI** 조합이 더 효율적이다. 예를 들어:
  - Adobe Acrobat으로 논문을 읽고 형광펜/메모를 남긴 뒤, Claude나 GPT에게 PDF를 던져서 핵심 요약, BibTeX 생성, related work 비교를 시킬 수 있다
  - Google Scholar에서 논문을 찾고, "Cite" 버튼으로 BibTeX를 직접 복사하는 것만으로도 충분한 경우가 많다
  - Zotero + Better BibTeX는 여전히 쓸만하지만 필수는 아니다. 논문 수가 100편을 넘어가면 관리 도구가 있는 게 편하다
  - Semantic Scholar의 Research Feed로 관련 논문을 구독하는 것도 좋다
- 파이프라인 그림: TikZ (정밀 제어), draw.io (빠른 제작), Inkscape (SVG 편집)
- 테이블: booktabs 패키지 (\toprule, \midrule, \bottomrule)
- 알고리즘: algorithm2e 패키지
- 수식: 표기법을 논문 전체에서 통일하라 (notation table 만들기)

> 추천 자료:
> - [How to Write a Great Research Paper (Simon Peyton Jones, Microsoft Research)](https://www.microsoft.com/en-us/research/academic-program/write-great-research-paper/) — 논문 쓰기의 고전 강연
> - [How to Read a Paper (S. Keshav)](http://ccr.sigcomm.org/online/files/p83-keshavA.pdf) — 3-pass reading method
> - [Tips for Writing Technical Papers (Jennifer Widom, Stanford)](https://cs.stanford.edu/people/widom/paper-writing.html) — 간결한 실전 조언
