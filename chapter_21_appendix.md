# 부록

이 부록은 연구와 개발 과정에서 막힐 때마다 바로 펼쳐 볼 수 있는 실용 참고서다. 용어 사전(A)과 자주 묻는 질문(B), 환경·빌드 트러블슈팅(C), 시작 전 점검표(D), 신입생을 위한 첫 주 생존 가이드(E)를 차례로 담았고, 마지막 부록 F에서는 우리 연구실이 실제로 파고드는 연구 방향을 소개한다.

## A. 용어 사전

로보틱스와 Spatial AI 논문에는 약어와 전문 용어가 쉼 없이 등장한다. 논문을 읽거나 세미나를 들을 때 뜻이 헷갈리는 표현이 나오면 아래 목록에서 빠르게 찾아보자.

### A.1 약어

논문과 코드베이스에서 자주 마주치는 핵심 약어를 분야와 상관없이 한곳에 모았다.

| 약어 | 풀이 | 설명 |
| --- | --- | --- |
| SLAM | Simultaneous Localization and Mapping | 위치 추정과 지도 작성을 동시에 하는 문제 |
| VO | Visual Odometry | 시각 주행거리계 |
| VIO | Visual-Inertial Odometry | 시각-관성 주행거리계 |
| LIO | LiDAR-Inertial Odometry | 라이다-관성 주행거리계 |
| IMU | Inertial Measurement Unit | 관성 측정 장치 |
| DoF | Degrees of Freedom | 자유도 |
| SE(3) | Special Euclidean Group (3D) | 3D 강체 변환 그룹 |
| SO(3) | Special Orthogonal Group (3D) | 3D 회전 그룹 |
| FoV | Field of View | 시야각 |
| ToF | Time of Flight | 비행 시간 (거리 측정 방식) |
| CNN | Convolutional Neural Network | 합성곱 신경망 |
| ViT | Vision Transformer | 비전 트랜스포머 |
| VFM | Vision Foundation Model | 비전 기반 파운데이션 모델 |
| VLA | Vision-Language-Action | 시각-언어-행동 모델 |
| VLM | Vision-Language Model | 시각-언어 모델 |
| LLM | Large Language Model | 대규모 언어 모델 |
| mAP | mean Average Precision | 클래스별 평균 정밀도(AP)의 평균 |
| ICP | Iterative Closest Point | 반복적 최근접점 정합 알고리즘 |
| NDT | Normal Distributions Transform | 정규분포 변환 기반 정합 기법 |
| NeRF | Neural Radiance Fields | 신경 방사장 |
| 3DGS | 3D Gaussian Splatting | 3D 가우시안 스플래팅 |
| BEV | Bird's Eye View | 조감도 (위에서 내려다본 시점) |
| TSDF | Truncated Signed Distance Function | 절단 부호 거리 함수 |
| BA | Bundle Adjustment | 번들 조정 |
| PGO | Pose Graph Optimization | 포즈 그래프 최적화 |
| DDS | Data Distribution Service | ROS2의 통신 미들웨어 |
| ONNX | Open Neural Network Exchange | 프레임워크 간 모델 교환 포맷 |
| TRT | TensorRT | NVIDIA 추론 최적화 엔진 |
| ATE | Absolute Trajectory Error | 절대 궤적 오차 |
| RPE | Relative Pose Error | 상대 포즈 오차 |

### A.2 용어

단순 약어 외에, 개념 자체가 문맥에 따라 헷갈리기 쉬운 핵심 용어들을 직관적인 비유와 함께 정리했다.

**Keyframe**: 추적과 지도 작성의 기준으로 삼기 위해 선별한 프레임. 모든 프레임을 매번 처리하면 연산량이 감당되지 않으므로, 카메라가 일정 거리 이상 움직이거나 시야가 크게 달라졌을 때만 골라 남긴다.

**Loop Closure**: 로봇이 이전에 지나왔던 장소를 다시 알아보고 그동안 쌓인 drift를 한꺼번에 바로잡는 과정. "아, 여기 아까 지나왔던 곳이네" 하고 인식하면서 누적 오차를 되감아 지도를 맞춘다.

**Drift**: 작은 추정 오차가 시간이나 이동 거리에 따라 조금씩 쌓여 궤적이 틀어지는 현상. 예컨대 보폭 1 m로 100 m를 걸으며 매 걸음 1 cm의 오차가 한쪽 방향으로만 쌓이면 도착 지점에서는 100 cm까지 벌어지고, 오차 방향이 서로 무상관이라 하더라도 걸음 수의 제곱근에 비례해 약 10 cm 규모의 오차가 남는다.

**Reprojection Error**: 3D 공간의 점을 카메라 이미지 평면에 다시 투영했을 때 생기는 픽셀 거리 오차. "이 3D 점이 이미지의 어디에 맺혀야 하는가"라는 기하학적 예측값과 실제 관측된 픽셀 좌표 사이의 차이를 뜻한다.

**Feature Descriptor**: 특징점 주변의 밝기와 기울기 패턴을 요약한 수치 벡터. 서로 다른 두 이미지에서 같은 3D 점을 가리키는 특징점을 짝지을 때 이 벡터의 거리를 비교한다.

**Homography**: 같은 평면을 찍은 두 이미지 사이의 2D 사영 변환 행렬. 책상 위나 건물 외벽처럼 한 평면 위에 놓인 장면을 서로 다른 각도에서 찍은 두 사진을 맞출 때 사용한다.

**Essential Matrix**: 내부 파라미터가 캘리브레이션된 두 카메라 사이의 에피폴라 기하 관계를 담는 행렬. 회전 3자유도와 크기(scale)를 뺀 이동 방향 2자유도를 합쳐 5DoF를 갖는다.

**Fundamental Matrix**: 캘리브레이션되지 않은 일반 두 카메라 사이의 에피폴라 기하 관계를 나타내는 행렬로, 7DoF를 갖는다.

**Epipole**: 한 카메라의 광학 중심이 다른 카메라의 이미지 평면에 투영된 점.

**Zero-shot**: 특정 태스크를 위한 별도 학습이나 예시 없이 모델을 바로 적용해 문제를 푸는 방식. 예컨대 고양이 탐지용으로 따로 파인튜닝하지 않았는데도 "고양이 찾아 줘"라는 지시에 곧장 반응하는 경우다.

**Few-shot**: 단 몇 개의 예시만 보여 주어 모델이 새로운 태스크에 적응하게 하는 학습 또는 추론 방식.

**Fine-tuning**: 대규모 데이터로 사전학습(pretraining)된 모델의 가중치를 출발점으로 삼아, 특정 태스크나 내 연구 데이터셋에 맞춰 추가로 학습시키는 과정.

**Domain Adaptation**: 한 도메인(소스)에서 학습한 모델이 분포가 다른 도메인(타깃)에서도 잘 동작하도록 간극을 줄이는 기법. 시뮬레이션 환경에서 학습한 모델을 실제 로봇 카메라 영상에 맞게 조정하는 작업이 대표적이다.

**Sim-to-Real**: 물리 시뮬레이터에서 학습시킨 인식 모델이나 제어 정책을 실제 로봇 하드웨어로 옮겨 적용하는 과정. [Zhao et al.(2020)의 서베이](https://arxiv.org/abs/2009.13303)에서는 Domain Randomization과 Domain Adaptation을 Sim-to-Real의 핵심 방법론으로 정리한다.

**Gaussian Splatting**: 3D 장면을 수많은 3차원 가우시안 타원체(primitive)의 집합으로 표현해 렌더링하는 기법. NeRF 계열보다 렌더링 속도가 훨씬 빠르고, 명시적 입자 기반이라 장면의 일부를 잘라내거나 편집하기도 수월하다.

**Factor Graph**: 추정하려는 상태 변수와 센서 측정값 사이의 제약 조건을 이분 그래프(bipartite graph)로 모델링한 자료구조. 현대 SLAM의 비선형 최적화 문제는 대부분 이 그래프 형태로 정식화해 푼다.

**Knowledge Distillation**: 성능 손실을 최소화하면서 추론 속도를 높이는 모델 경량화 기법. 크고 무거운 모델(teacher)이 내는 출력 분포나 중간 표현을 작고 가벼운 모델(student)이 모방하도록 학습시킨다.

## B. 자주 묻는 질문

연구실에 막 합류한 신입생들이 가장 자주 묻는 질문을 모았다. 언어 선택부터 GPU 사양 고르기, 논문 읽기와 학회 발표 준비까지 실전에서 바로 쓸 수 있는 답을 담았다.

**Q: Python과 C++ 중 어떤 것을 먼저 배워야 하나요?**

A: 지금 당장 읽고 실행해야 할 연구실 코드가 어느 언어로 쓰여 있는지부터 확인한다. SLAM, ROS 패키지, 실시간 제어 모듈은 대부분 C++로 작성되어 있으므로 기존 연구실 코드를 뜯어보고 수정하려면 C++ 실력이 필요하다. 반면 딥러닝 학습 스크립트와 데이터 전처리, 시각화에는 주로 Python을 쓴다. 요즘은 AI 코딩 에이전트가 코드 작성을 거들어 주긴 하지만, 결국 두 언어 모두 기존 코드를 읽고 동작이 맞는지 검증할 줄 알아야 한다.

**Q: GPU가 없는데 연구를 시작할 수 있나요?**

A: 기하 알고리즘 검증이나 데이터 전처리, 간단한 추론 테스트는 CPU만으로도 충분히 돌릴 수 있다. 딥러닝 모델을 직접 학습시킬 때는 GPU가 필요하므로, 개인 PC에 GPU가 없다면 Google Colab이나 연구실 공용 서버를 활용한다. Colab 무료 버전에서도 작은 YOLO 모델을 파인튜닝하는 수준의 입문 실습은 무리 없이 진행할 수 있다.

**Q: ROS1과 ROS2 중 어떤 것을 배워야 하나요?**

A: 새로 시작한다면 ROS2를 선택한다. ROS1은 2025년에 공식 지원이 종료되었다(EOL). 다만 프로젝트에서 꼭 써야 하는 기존 패키지나 하드웨어 드라이버가 ROS1만 지원한다면 당장 필요한 범위에서 ROS1을 먼저 익혀도 무방하다. 노드·토픽·서비스 같은 핵심 개념이 같기 때문에 ROS1을 먼저 다루어 보더라도 이후 ROS2로 넘어가는 장벽은 높지 않다.

**Q: 논문을 어디서 찾나요?**

A: 주로 [arXiv](https://arxiv.org/)(무료 프리프린트 서버)와 [Google Scholar](https://scholar.google.com/)에서 검색한다. 공식 구현 코드가 함께 공개되었는지 보려면 논문에 적힌 GitHub 링크나 [Hugging Face Papers](https://huggingface.co/papers/trending)를 확인한다. (오랫동안 코드·벤치마크 링크를 모아 주던 Papers With Code는 2025년에 운영을 마쳤다.) 학회별로 논문을 훑어보고 싶다면 [CVF Open Access](https://openaccess.thecvf.com/)나 [IEEE Xplore](https://ieeexplore.ieee.org/)를 이용하면 편리하다.

**Q: SLAM을 공부하려면 어디서 시작해야 하나요?**

A: Cyrill Stachniss의 [YouTube SLAM 강의](https://www.youtube.com/playlist?list=PLgnQpQtFTOGQrZ4O5QzbIHgl3b1JHimN_)로 전체 그림을 잡은 뒤 ORB-SLAM3 코드를 직접 분석해 보기를 권한다. 그 전에 9장의 카메라 모델과 14장의 SLAM 기초를 먼저 훑어 두면 강의 내용과 코드 구조가 훨씬 수월하게 들어온다.

**Q: 연구 아이디어는 어떻게 찾나요?**

A: 최근 주요 학회 논문의 Limitation 절을 눈여겨보자. 저자들이 스스로 한계로 짚은 지점이나 후속 과제로 남겨 둔 문제에서 출발하면 실제 연구로 이어질 실마리를 얻기 쉽다. 아직 잘 결합되지 않은 두 분야를 이어 보는 접근도 유효한데, 이때는 아이디어가 떠올랐다고 바로 구현에 뛰어들기보다 그 조합을 다룬 선행 연구가 이미 있는지부터 꼼꼼히 검색해야 한다. 예컨대 "3D Gaussian Splatting + Semantic SLAM"은 이미 여러 논문이 발표된 주제이며, 부록 F에서도 의미 정보를 담은 Gaussian 지도를 다룬다. 선행 연구의 범위를 정확히 파악한 뒤에야 아직 풀리지 않은 빈틈을 겨냥할 수 있다.

**Q: 어떤 GPU를 사야 하나요?**

A: 모델명이나 스펙표만 보고 고르기 전에, 먼저 내가 연구할 모델을 실제 학습 설정으로 돌려 보고 모델 가중치, 옵티마이저 상태, activation, 배치가 VRAM 안에 들어가는지부터 재야 한다. 그다음 지원하는 수치 정밀도, 메모리 대역폭, 소비 전력, 프레임워크 호환성, 실제 벤치마크 결과를 함께 저울질한다. 아래 두 표에는 VRAM 용량과 용도(데스크톱 vs 데이터센터)에 따라 검토해 볼 만한 대표 후보를 추려 담았다. 표의 위아래 순서는 성능 순위와 무관하다.

먼저 개인 작업용이나 로컬 프로토타이핑에 많이 쓰이는 데스크톱 GPU 후보다.

| VRAM | 카드 예시 | 검토할 역할 | 구매 전 확인 |
|------|-----------|-------------|--------------|
| 8GB | RTX 4060, RTX 5060 Ti 8GB | 작은 CNN 학습, 제한된 batch의 추론 | 돌릴 모델의 최대 메모리 사용량. VFM fine-tuning에는 설정에 따라 VRAM이 부족할 수 있음 |
| 12GB | RTX 3060 12GB, RTX 4070 | 중간 크기 inference·학습 실험 | 세대별 연산 차이와 중고 상태 |
| 16GB | RTX 5060 Ti 16GB, RTX 4070 Ti Super | 더 큰 batch, VFM inference, 중간 규모 학습 | 모델별 activation·optimizer 메모리 |
| 24GB | RTX 3090, RTX 4090 | 24GB 안에 드는 학습, 3DGS·VLA 실험 | 전력·냉각·중고 보증; 3090과 4090의 runtime 차이 |
| 32GB | RTX 5090 | 24GB를 넘는 로컬 실험 | 전력·케이스·PSU, 소프트웨어 지원 |

대규모 학습이나 다중 사용자 환경에 쓰이는 서버·연구실용 데이터센터 GPU는 아래와 같다.

| GPU memory | 카드 | 특징 | 공식 사양 |
|------------|------|------|-----------|
| 16/32GB | V100 SXM2 | 1세대 Tensor Core, TF32/BF16 미지원 | [V100 datasheet](https://images.nvidia.com/content/technologies/volta/pdf/tesla-volta-v100-datasheet.pdf) |
| 24GB | A10 | PCIe inference·graphics 계열 | [A10 datasheet](https://www.nvidia.com/content/dam/en-zz/Solutions/Data-Center/a10/pdf/a10-datasheet.pdf) |
| 40/80GB | A100 | TF32·BF16, MIG, PCIe/SXM 변형 | [A100 specifications](https://www.nvidia.com/en-us/data-center/a100/) |
| 80GB | H100 SXM | Hopper, Transformer Engine, NVLink | [H100 specifications](https://www.nvidia.com/en-us/data-center/h100/) |
| 141GB | H200 SXM | 141GB HBM3e, 4.8TB/s memory bandwidth | [H200 specifications](https://www.nvidia.com/en-us/data-center/h200/) |
| 180GB | B200 | Blackwell, 180GB HBM3e. DGX 같은 서버 구성으로 공급 | [DGX B200 specifications](https://www.nvidia.com/en-us/data-center/dgx-b200/) |

GPU 사양표를 읽을 때는 지원하는 precision(FP32·TF32·BF16·FP16·FP8), CUDA core와 Tensor Core 구성, dense와 structured sparsity 수치, 그리고 PCIe와 SXM 폼팩터 차이를 반드시 구분해서 본다. 제조사 표의 이론적 peak TFLOPS가 같더라도 실제 학습 시간은 memory bandwidth, 커널 효율, batch 크기, data loading 속도에 크게 좌우된다. `torch.amp`의 속도 향상과 메모리 절감 폭 역시 모델과 하드웨어마다 다르므로, 결정 전에 같은 repository·batch·precision 조건으로 짧은 benchmark를 직접 돌려 비교하는 게 가장 정확하다.

**구매 및 사용 전 참고 사항**:
- RTX 5060 Ti처럼 같은 모델명으로 8GB와 16GB 버전이 함께 나오는 카드에서는, 쓸 모델과 batch가 요구하는 VRAM을 먼저 계산해야 한다. VFM을 로컬에서 다룰 계획이라면 8GB는 금세 한계에 부딪힐 수 있다.
- AMD GPU를 검토한다면 주력 프레임워크와 라이브러리가 ROCm을 안정적으로 지원하는지부터 확인한다. CUDA 전용 확장이나 의존성이 있는 코드가 많다면 이식 비용까지 구매 조건에 포함해야 한다.
- 연구실 서버에 A100이나 H100이 이미 구축되어 있다면 개인 GPU의 역할은 디버깅과 작은 프로토타이핑 위주로 좁아진다. 구매 전에 서버의 실제 대기 시간과 할당 정책, 가용 사양을 먼저 파악한다.
- 중고 RTX 3090은 24GB VRAM을 비교적 저렴하게 확보할 수 있는 선택지지만, 매물마다 가격·보증 기간·냉각 상태 편차가 크다. 정격 전력과 파워서플라이(PSU)·케이스 규격도 반드시 함께 점검한다.
- Google Colab과 클라우드 GPU의 요금제, 할당되는 GPU 종류, 사용 시간 제한은 수시로 바뀐다. 장비를 사거나 장기 결제하기 전에 실제 workload를 대여 GPU에서 측정해 보고, 최신 요금과 quota는 제공업체 공식 페이지에서 확인한다.

**Q: 논문은 하루에 몇 편 읽어야 하나요?**

A: 정해진 할당량은 없다. 지금 논문을 읽는 목적에 맞춰 깊이를 조절하면 된다. 입문할 때는 19.4절에서 안내한 Keshav의 3-pass 방식으로 일주일에 한 편이라도 제대로 뜯어보는 연습부터 시작해 보자. 경험이 쌓이면 Abstract만 읽고도 논문의 성격과 내 연구와의 관련성을 빠르게 가늠할 수 있다. 랩미팅 발표를 위해 전체 논지를 파악할 때와, 내 연구의 베이스라인으로 삼아 코드까지 뜯어볼 때는 필요한 깊이와 투입해야 할 시간이 완전히 다르다.

**Q: 코딩이 서툰데 연구를 시작할 수 있나요?**

A: 요즘은 Claude나 Copilot 같은 코딩 에이전트에게 "KITTI 데이터셋 로더를 짜 줘"라거나 "이 학습 루프에 wandb 로깅을 붙여 줘"라고 요청해 작업 초안을 빠르게 얻을 수 있다. 다만 에이전트가 만들어 준 코드는 반드시 한 줄씩 직접 읽고 검토한 뒤 써야 한다.

생성된 코드가 제대로 동작하는지 가려내려면 결국 도메인 지식이 필요하다. `num_workers=0`일 때 DataLoader가 느려지는 병목, 학습 도중 loss가 NaN으로 튀는 이유, SLAM 코드에서 좌표계 축이 뒤집힌 지점 등은 실행 환경 맥락이 충분하지 않으면 에이전트도 쉽게 놓친다(14장 참고). 따라서 코드를 직접 실행해 보고 검증된 기존 구현의 출력과 대조한 뒤에야 믿고 받아들여야 한다.

ORB-SLAM3, Ultralytics, HuggingFace Transformers처럼 완성도 높은 오픈소스 저장소를 읽으며 각 모듈이 왜 그렇게 설계되었는지 따라가다 보면, 코드를 읽고 검토하는 눈이 자연스럽게 자란다.

**Q: 학회 발표는 어떻게 준비하나요?**

A: 학회 발표는 크게 구두 발표(oral)와 포스터 발표(poster)로 나뉜다. 실제 발표 시간과 포스터 규격, 사용 언어는 학회마다 다르므로 항상 공식 발표자 안내(presenter guidelines)를 최우선으로 확인한다.

- **포스터**: A0 세로 규격이 흔하지만 학회 지정 크기와 방향이 따로 있는지 먼저 본다. 멀리서도 한눈에 들어오도록 Figure를 크게 배치하고 본문 텍스트는 과감히 줄여, 지나가는 연구자가 짧은 시간 안에 핵심 문제와 결과를 알아볼 수 있게 구성한다. 행사 전에는 연구실 동료들 앞에서 리허설을 해 보며 질의응답 흐름을 다듬는다.
- **구두 발표**: 15~20분 발표가 흔한 편이긴 하지만 세션별 제한 시간을 정확히 확인해 맞춘다. 주어진 시간에 맞춰 슬라이드 장수를 조절하고, 한 장에는 하나의 핵심 메시지만 담는다. 예상 질문에 대비한 백업 슬라이드와 로봇 데모 영상은 필요에 따라 미리 챙겨 둔다.
- **공통**: 발표 언어를 확인한 뒤 대본으로 충분히 연습하되, 문장을 토씨 하나 안 틀리고 외우기보다 핵심 논지를 또렷이 전달하고 제한 시간을 지키는 데 집중한다.

**Q: 영어 논문 읽기가 너무 힘든데요?**

A: 처음에는 누구나 막막하지만, 반복해서 읽고 분야의 배경지식이 쌓이면 자연스럽게 속도가 붙는다. 아래 네 가지 습관을 들이면 초반 부담을 크게 덜 수 있다.

- **구조부터 훑기**: 많은 실험 논문이 Introduction → Related Work → Method → Experiments → Conclusion 순서를 따르지만, 핵심 기여가 문제 정의나 데이터셋, 평가 분석에 있는 경우도 많다. 문장을 곧장 해석하려 들기 전에 제목과 소제목(heading)부터 훑어 글의 뼈대를 먼저 잡는다.
- **자주 쓰이는 어휘 익히기**: "ablation study", "state-of-the-art", "we empirically show"처럼 논문마다 반복되는 관용 표현이 정해져 있다. 처음 몇 편만 꼼꼼히 정리해 두면 뒤로 갈수록 낯선 표현이 눈에 띄게 줄어든다.
- **번역 도구는 보조 수단으로 쓰기**: 잘 안 읽히는 문장은 DeepL이나 Google Translate의 도움을 받아도 좋다. 다만 번역본만 읽으면 영어 독해력이 늘지 않으므로, "원문 읽기 → 번역으로 뜻 확인 → 다시 원문으로 문장 구조 되짚기" 순서를 지킨다.
- **PDF 리더의 형광펜 활용하기**: 중요한 주장과 수식, 실험 조건을 색깔별로 표시해 두면 나중에 다시 찾아보기 한결 수월해진다. Adobe Acrobat이나 Zotero 내장 뷰어처럼 본인 손에 잘 맞는 도구를 고른다.

## C. 트러블슈팅 가이드

실험 코드를 돌리다 보면 알고리즘 자체보다 환경 설정과 의존성 충돌에서 막히는 일이 잦다. 여기서는 연구실에서 가장 흔히 마주치는 Ubuntu 시스템 명령과 CUDA·PyTorch, ROS2, Docker, OpenCV, C++ 빌드 에러의 대표적인 해법을 증상별로 모았다.

**자주 쓰는 apt 명령어**

Ubuntu 환경에서 패키지를 설치하거나 깨진 의존성을 복구할 때 기본으로 쓰는 명령어 모음이다.

```bash
sudo apt update                  # 패키지 목록 갱신
sudo apt upgrade                 # 설치된 패키지 업그레이드
sudo apt install <package>       # 패키지 설치
sudo apt remove <package>        # 패키지 제거 (설정 파일 유지)
sudo apt purge <package>         # 패키지 + 설정 파일 완전 제거
sudo apt autoremove              # 사용하지 않는 의존성 제거
apt list --installed             # 설치된 패키지 목록
apt search <keyword>             # 패키지 검색
sudo apt --fix-broken install    # 의존성 깨졌을 때 복구
```

(참고: [정진용 블로그](https://jinyongjeong.github.io/2016/06/07/Ubuntu_apt_get_commend/))

**SSH 키 설정 (비밀번호 없이 서버 접속)**

16장 §16.4.2의 절차로 생성한 공개키(`~/.ssh/id_ed25519.pub`)를 GitHub 계정에도 등록해 두면, SSH remote(`git@github.com:...`)로 연결된 저장소에서 `git push`를 할 때 비밀번호 입력 없이 인증할 수 있다. 반면 HTTPS로 clone한 저장소는 개인 액세스 토큰이나 credential helper로 인증한다. (참고: [정진용 블로그](https://jinyongjeong.github.io/2016/06/02/SSH_keygen_setting/))

**CPU 성능 모드 설정 (실험 시)**

SLAM이나 딥러닝 벤치마크를 돌릴 때 CPU 절전 기능(throttling)이 개입해 실행 속도가 들쭉날쭉해질 수 있다. 지연 시간을 재현 가능하게 측정하려면 아래 명령으로 CPU governor를 성능 모드로 고정한다.

```bash
# 현재 CPU governor 확인
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor

# performance 모드로 변경 (모든 코어)
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# 영구 설정 (재부팅 후에도 유지)
sudo apt install cpufrequtils
echo 'GOVERNOR="performance"' | sudo tee /etc/default/cpufrequtils
sudo systemctl restart cpufrequtils
```

노트북에서 이 설정을 켜 두면 배터리가 빠르게 소모되므로, 반드시 전원을 연결한 상태에서만 사용한다. (참고: [정진용 블로그](https://jinyongjeong.github.io/2020/02/04/Ubuntu_cpu_freq_change/))

### C.1 CUDA / PyTorch 관련

GPU 메모리 부족이나 CUDA 버전 불일치는 딥러닝 실험에서 가장 먼저 마주치는 오류다.

**문제**: `CUDA out of memory`

**해결**: 아래 코드처럼 먼저 배치 크기를 줄이고, 이어서 캐시를 정리한 뒤 gradient accumulation과 혼합 정밀도(`torch.autocast`) 조합을 적용한다.

```python
import torch

# 1. 배치 사이즈 줄이기 (가장 먼저 시도)
batch_size = 16  # → 8 또는 4

# 2. 메모리 정리
torch.cuda.empty_cache()

# 3. Gradient accumulation + AMP
accumulation_steps = 4
num_batches = len(dataloader)
scaler = torch.amp.GradScaler("cuda")
optimizer.zero_grad()

for i, (inputs, labels) in enumerate(dataloader):
    group_start = (i // accumulation_steps) * accumulation_steps
    group_size = min(accumulation_steps, num_batches - group_start)

    with torch.autocast(device_type="cuda", dtype=torch.float16):
        output = model(inputs)
        loss = criterion(output, labels) / group_size

    scaler.scale(loss).backward()

    is_update_step = ((i + 1) % accumulation_steps == 0) or ((i + 1) == num_batches)
    if is_update_step:
        scaler.step(optimizer)
        scaler.update()
        optimizer.zero_grad()
```

AMP(Automatic Mixed Precision)로 얻는 메모리 절감과 속도 향상 폭은 모델 구조와 GPU 세대, 연산자 지원 여부에 따라 달라진다. 적용 전후로 같은 배치 크기와 모델에서 peak memory와 학습 안정성을 직접 측정해 확인한다.

**문제**: `CUDA version mismatch` (주로 CUDA 확장을 직접 빌드할 때)

**해결**: 시스템 `nvcc`와 PyTorch가 인식하는 CUDA 런타임 버전을 함께 확인한다.

```bash
# 설치된 CUDA 버전 확인
nvcc --version

# PyTorch에서 인식하는 CUDA 버전 확인
python -c "import torch; print(torch.version.cuda)"

# 두 값은 달라도 정상이다. wheel이 자체 CUDA 런타임을 포함하므로
# 드라이버가 그 런타임을 지원하면 그대로 쓴다. 단 CUDA 확장을 직접 빌드할 때는
# nvcc와 torch.version.cuda의 메이저 버전을 맞춰야 하며, 그때만 재설치한다.
pip install torch --index-url https://download.pytorch.org/whl/cu126
```

**문제**: `RuntimeError: CUDA error: device-side assert triggered`

**해결**: 가장 흔한 원인은 분류 모델의 라벨 인덱스가 클래스 개수 범위를 벗어난 것이다. 비동기 GPU 실행 탓에 스택 트레이스가 엉뚱한 줄을 가리키기 쉬우므로, CPU에서 먼저 돌려 자세한 에러 메시지를 확인하거나 GPU에서 아래처럼 CUDA 호출을 동기화해 오류가 난 코드 줄이 정확히 찍히게 한다.

```bash
CUDA_LAUNCH_BLOCKING=1 python train.py
```

### C.2 ROS 관련

ROS2에서는 작업 공간(workspace) 환경 변수가 빠지거나 노드 간 통신 설정이 어긋나 데이터가 조용히 끊기는 일이 빈번하다.

**문제**: `Package not found`

**해결**: 빌드한 작업 공간의 `setup.bash`를 현재 셸에 소싱했는지부터 확인한다.

```bash
# Workspace 소싱 확인
source ~/ros2_ws/install/setup.bash

# 패키지 설치 확인
ros2 pkg list | grep package_name

# .bashrc에 소싱 추가 (매번 수동으로 안 해도 됨)
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

**문제**: `TF tree not connected`

**해결**: TF 트리를 시각화해 어느 링크 사이가 끊겼는지 찾고, 누락된 고정 변환이 있다면 static transform을 발행한다.

```bash
# TF 트리 확인
ros2 run tf2_tools view_frames

# Static transform 추가 (예시). 위치·자세 인자를 생략하면 0으로 둔다
ros2 run tf2_ros static_transform_publisher --frame-id base_link --child-frame-id camera_link
```

**문제**: `Topic not published` / 데이터가 안 들어옴

**해결**: 토픽이 실제로 떠 있는지, 메시지가 들어오는지 확인하고 ROS2에서 흔한 원인인 QoS 불일치 여부를 점검한다.

```bash
# 현재 활성 토픽 확인
ros2 topic list

# 특정 토픽 데이터 확인
ros2 topic echo /camera/image_raw --once

# QoS 설정 불일치 확인 (ROS2에서 흔한 문제)
ros2 topic info /camera/image_raw -v
```

### C.3 Docker 관련

**문제**: `Permission denied`

**해결**: 현재 사용자를 `docker` 그룹에 추가한 뒤 세션을 다시 연다.

```bash
# 도커 그룹에 사용자 추가
sudo usermod -aG docker $USER
# 로그아웃 후 재로그인
```

**문제**: GUI 프로그램 실행 안 됨

**해결**: 호스트의 X11 소켓과 디스플레이 환경을 컨테이너로 전달해 GUI 창이 열리게 한다.

```bash
# X11 forwarding
xhost +local:docker
docker run -it --env DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix ...
```

**문제**: Docker 안에서 GPU가 안 잡힘

**해결**: 16장 §16.3.3에서 안내한 순서대로 `nvidia-container-toolkit`을 설치해 Docker 런타임에 등록한 뒤, 컨테이너 실행 시 `--gpus all` 옵션을 붙여 띄운다.

### C.4 OpenCV 관련

**문제**: `cv2.imshow() not working`

**해결**: GUI 백엔드가 빠진 headless 패키지가 설치된 상태라면 이를 지우고 기본 패키지를 다시 설치한다.

```bash
# OpenCV headless 버전 제거 후 재설치
pip uninstall opencv-python-headless
pip install opencv-python
```

**문제**: OpenCV와 ROS의 cv_bridge 충돌

**해결**: conda나 venv 환경의 Python을 활성화한 채 `colcon build`를 실행하면 `cv_bridge`가 참조하는 OpenCV·Python 경로가 뒤섞일 수 있으므로 시스템 Python 경로를 명시한다.

```bash
# ROS의 cv_bridge가 시스템 OpenCV를 참조하는 경우
# conda/venv 환경의 OpenCV와 충돌할 수 있다
# 해결: ROS workspace 빌드 시 Python 경로 명시

colcon build --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

### C.5 빌드/컴파일 관련

**문제**: ORB-SLAM3 빌드 에러 (OpenCV 버전 충돌)

**해결**: `CMakeLists.txt` 버전을 OpenCV 4.x 환경에 맞추고, 시각화 라이브러리인 Pangolin을 빌드할 때는 저장소 안의 의존성 설치 스크립트를 활용한다.

```bash
# OpenCV 4.x에서는 일부 API가 변경됨
# CMakeLists.txt에서 OpenCV 버전 확인:
#   find_package(OpenCV 4 REQUIRED)

# Pangolin 빌드 에러 시: 저장소에 든 스크립트로 의존 패키지를 설치한다
# (Ubuntu 24.04 이후에는 libpython2.7-dev 패키지가 없다)
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
./scripts/install_prerequisites.sh recommended
cmake -B build
cmake --build build
```

**문제**: Eigen 버전 관련 에러

**해결**: 시스템에 설치된 Eigen3 버전을 확인하고, 누락되었다면 공식 개발 패키지를 설치한다.

```bash
# 시스템 Eigen 버전 확인
pkg-config --modversion eigen3

# 저장소에서 제공하는 Eigen 개발 패키지 설치
sudo apt-get install libeigen3-dev
```

## D. 체크리스트: 연구 시작 전 확인 사항

새로운 서버나 연구용 워크스테이션을 세팅한 뒤 본격적인 실험에 들어가기 전에, 빠진 항목이 없는지 아래 네 범주를 차례로 점검해 보자.

### D.1 환경 설정

- [ ] Ubuntu 설치 완료 (쓸 ROS2 배포판에 맞춘다. 15장 15.1.1, 16장 16.2.1 참고)
- [ ] NVIDIA 드라이버 설치 (`nvidia-smi`로 확인)
- [ ] CUDA toolkit 설치 (`nvcc --version`으로 확인)
- [ ] cuDNN 설치 (시스템 설치는 `cudnn_version.h`로 확인. `torch.backends.cudnn.version()`은 PyTorch에 번들된 cuDNN을 보여 준다)
- [ ] Conda 또는 venv 환경 설정
- [ ] PyTorch GPU 동작 확인 (`torch.cuda.is_available()`)
- [ ] ROS2 설치 (필요 시. Ubuntu 22.04이면 Humble, 24.04이면 Jazzy, 26.04이면 Lyrical)
- [ ] Git 설정 (`git config --global user.name/email`)
- [ ] Docker 설치 (선택, 재현성을 위해 권장)
- [ ] VS Code + 필수 확장 설치 (Python, Remote-SSH, Jupyter)

### D.2 기초 지식

- [ ] Python 기본 문법 (클래스, 데코레이터, 리스트 컴프리헨션)
- [ ] NumPy 배열 연산 (broadcasting, indexing, reshape)
- [ ] OpenCV 이미지 처리 (읽기, 변환, 필터, 특징점)
- [ ] 선형대수 기초 (행렬 곱셈, 고유값 분해, SVD)
- [ ] 확률/통계 기초 (베이즈 정리, 가우시안 분포, MLE/MAP)

### D.3 연구 도구

논문 읽기 도구(노트 양식과 인용 관리)는 [「연구노트」 15장](../research-notes/guide.html#chapter-15)에서, 아웃라인·단락 구조·피드백 받기 같은 논문 쓰기 방법은 [「연구노트」 Part 2 — 쓰기](../research-notes/guide.html#chapter-16)에서 다룬다. 학회 준비 체크리스트는 [「연구노트」 34장 학회 준비](../research-notes/guide.html#chapter-34)에, Spatial AI 분야의 추천 도구와 학습 순서는 이 가이드의 §19.4와 §19.7에 정리되어 있다.

### D.4 데이터셋 준비

- [ ] 연구 관련 데이터셋 다운로드
- [ ] 데이터 포맷 이해 (이미지 크기, depth 단위, 좌표계)
- [ ] DataLoader 구현 (PyTorch Dataset/DataLoader)
- [ ] 데이터 시각화 코드 작성 (디버깅용)

## E. 첫 주 생존 가이드

연구실에 들어온 첫 주에 모든 논문을 이해하거나 완벽한 실험 파이프라인을 돌릴 필요는 없다. 첫 주의 진짜 목표는 계정과 실행 환경을 마련하고, 앞으로 내가 다룰 코드·데이터·문서가 어디에 있는지 지도를 그리는 데 있다. 아래 Day 1~7 일정은 전형적인 예시이므로, 계정 발급 속도나 장비 일정, 연구실 온보딩 방식에 따라 순서를 유연하게 조정한다.

### Day 1~2: 환경 구축

처음 이틀은 연구실 인프라에 접속하고 기본 개발 도구를 손에 맞추는 데 집중한다.

```
[ ] 연구실 서버 계정 받기 (관리자에게 요청)
[ ] SSH로 서버 접속 확인
[ ] VS Code Remote-SSH 설정
[ ] 서버에 conda 환경 만들기
[ ] PyTorch + CUDA 동작 확인
[ ] 연구실 GitHub organization에 가입
[ ] Slack/Discord 채널 가입
```

> 팁: 서버 환경 설정에서 막혔을 때는 혼자 며칠씩 붙잡고 있기보다, 시도한 명령어와 예상했던 결과, 실제 출력 메시지를 함께 정리해 선배에게 묻자. 확인한 단서를 같이 보여 주면 문제를 훨씬 빨리 좁힐 수 있다.

### Day 3~4: 기존 코드 파악

환경이 열렸다면 이제 연구실이 실제로 돌리고 있는 코드베이스와 데이터를 내려받아 실행해 볼 차례다.

```
[ ] 연구실의 기존 코드/프로젝트 리포지토리 클론
[ ] README 읽기 (있다면)
[ ] 기존 코드 빌드/실행 해보기
[ ] 데이터셋 다운로드 및 경로 설정
[ ] 간단한 데모 돌려보기
```

> 팁: 남이 짜 둔 코드를 처음 실행했을 때 단번에 돌아가는 일은 드물다. 곧바로 당황하기보다 환경 변수, 데이터 경로, 패키지 버전 차이부터 확인하고, 에러 메시지로 공식 문서와 GitHub issue를 먼저 검색해 본다.

### Day 5: 논문 읽기 시작

코드가 돌아가는 모습을 확인했다면, 그 코드가 뿌리를 둔 논문들을 읽기 시작한다. 첫 논문을 추천받고 연구실 구성원과 대화를 트는 방법은 [「대학원노트」 4장 지도교수와의 관계](../grad-notes/guide.html#chapter-4)에서, 첫 주에 내 연구 방향의 가닥을 잡는 일은 [「대학원노트」 7장 내 연구 갖기](../grad-notes/guide.html#chapter-7)에서 자세히 다룬다.

> 팁: 처음 읽는 논문이 한 번에 이해되지 않는 건 지극히 정상이다. 첫 주에는 수식을 전부 증명하려 들지 말고, "이 논문이 대체 무슨 문제를 풀려고 하는가?"만 잡아내도 충분하다.

### Day 6~7: 연구 방향 파악

첫 주의 마무리는 연구실의 큰 그림을 파악하는 일이다. 이어지는 부록 F의 연구 방향을 읽고, 연구실의 최근 논문과 진행 중인 프로젝트가 어느 주제에 닿아 있는지 정리해 본다. 선배들이 맡고 있는 주제와 겹치는 지점까지 함께 표시해 두면 앞으로 누구에게 무엇을 물어봐야 할지도 한눈에 보인다.

### 첫 주에 하지 않아도 되는 것들

초반부터 스스로에게 과한 기준을 세워 지치지 않도록, 첫 주에 굳이 해내려 애쓰지 않아도 되는 일들을 미리 짚어 둔다.

- **논문을 완벽하게 이해하기** — 배경지식이 쌓이고 시간이 지나면서 자연스럽게 풀린다.
- **최신 연구 트렌드를 전부 파악하기** — 한 번에 다 훑기보다 내 주제에서 출발해 점진적으로 넓힌다.
- **코드를 처음부터 짜기** — 기존 코드를 실행하고 작은 부분을 고쳐 보는 것부터 시작한다.
- **GPU 서버를 완벽하게 세팅하기** — 연구실에 이미 검증된 환경 파일·container·설치 절차가 있다면 거기서 출발한다.
- **연구 아이디어를 완성된 형태로 내놓기** — 먼저 연구실이 다루는 문제와 도구의 맥락부터 익힌다.

### 생존을 위한 마인드셋

연구 초기의 불안과 시행착오에 대처하는 태도와 작업 습관은 「대학원노트」의 아래 장들에서 길게 다루고 있으니, 마음이 급해질 때마다 펼쳐 보기를 권한다.

- *모르는 건 당연하다* → [「대학원노트」 13장 자율성](../grad-notes/guide.html#chapter-13) §13.2 (대학원에서 오래 남는 것)
- *"안 돼요"는 보고가 아니다* + 보고 형식(예측·시도·결과) → [「대학원노트」 10장 한 메일에 한 질문](../grad-notes/guide.html#chapter-10) §10.3 (답하기 쉬운 메일의 형식)
- *기록하라 — 과거의 내가 미래의 나를 도와준다* → [「대학원노트」 8장 시간 배분](../grad-notes/guide.html#chapter-8) §8.5 (퇴근 전에 다음 시작점을 적는다)
- *작게 시작하라 — 작은 코드 조각부터* → [「대학원노트」 11장 환경 설정이 연구를 가릴 때](../grad-notes/guide.html#chapter-11) §11.1 (환경 설정 시간과 연구 시간을 구분한다)
- *비교하지 마라 — 3개월 후의 나* → [「대학원노트」 14장 남과 비교하는 마음 다루기](../grad-notes/guide.html#chapter-14) §14.3 (단거리와 장거리)
