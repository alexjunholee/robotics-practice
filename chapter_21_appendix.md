# Ch.21 — 부록


## A. 용어 사전

### A.1 약어

| 약어 | 풀이 | 설명 |
| --- | --- | --- |
| SLAM | Simultaneous Localization and Mapping | 동시적 위치추정 및 지도작성 |
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
| VFM | Vision Foundation Model | 비전 기반 모델 |
| VLA | Vision-Language-Action | 시각-언어-행동 모델 |
| VLM | Vision-Language Model | 시각-언어 모델 |
| LLM | Large Language Model | 대규모 언어 모델 |
| mAP | mean Average Precision | 평균 정밀도 |
| ICP | Iterative Closest Point | 반복적 최근접점 |
| NDT | Normal Distributions Transform | 정규분포 변환 |
| NeRF | Neural Radiance Fields | 신경 방사장 |
| 3DGS | 3D Gaussian Splatting | 3D 가우시안 스플래팅 |
| BEV | Bird's Eye View | 조감도 |
| TSDF | Truncated Signed Distance Function | 절단 부호 거리 함수 |
| BA | Bundle Adjustment | 번들 조정 |
| PGO | Pose Graph Optimization | 포즈 그래프 최적화 |
| DDS | Data Distribution Service | ROS2의 통신 미들웨어 |
| ONNX | Open Neural Network Exchange | 모델 변환 포맷 |
| TRT | TensorRT | NVIDIA 추론 최적화 엔진 |
| ATE | Absolute Trajectory Error | 절대 궤적 오차 |
| RPE | Relative Pose Error | 상대 포즈 오차 |

### A.2 용어

**Keyframe**: 중요 정보를 포함하는 선택된 프레임. 모든 프레임을 처리하면 너무 느리니까, 의미 있는 변화가 있는 프레임만 골라서 사용한다.

**Loop Closure**: 이전 방문 장소 재인식을 통한 드리프트 보정. "아, 여기 아까 왔던 곳이네" → 누적 오차를 한꺼번에 교정.

**Drift**: 오차의 누적. 100m를 걸어가면서 매 걸음마다 1cm씩 오차가 나면, 도착할 때는 100cm 오차가 된다.

**Reprojection Error**: 3D 점을 이미지에 재투영했을 때의 오차. "이 3D 점이 카메라 이미지 어디에 보여야 하는가"의 예측값과 실제값의 차이.

**Feature Descriptor**: 특징점 주변을 설명하는 벡터. 두 이미지에서 같은 점을 찾을 때, 이 벡터를 비교한다.

**Homography**: 평면 간의 변환. 책상 위를 찍은 두 사진을 정합할 때 사용.

**Essential Matrix**: 캘리브레이션된 카메라 쌍의 기하 관계. 5DoF(회전 3 + 이동 방향 2).

**Fundamental Matrix**: 캘리브레이션되지 않은 카메라 쌍의 기하 관계. 7DoF.

**Epipole**: 한 카메라의 중심이 다른 카메라 이미지에 투영된 점.

**Zero-shot**: 학습 없이 새로운 태스크 수행. "고양이"를 학습 안 했는데 "고양이 찾아줘"가 되는 것.

**Few-shot**: 적은 예제로 새로운 태스크 학습. 예시 3-5개만 주면 학습.

**Fine-tuning**: 사전학습 모델을 특정 태스크에 맞게 재학습. 대형 모델을 내 데이터에 맞게 조정.

**Domain Adaptation**: 소스 도메인에서 타겟 도메인으로 적응. 시뮬레이션에서 학습 → 실제 환경 적용.

**Sim-to-Real**: 시뮬레이션에서 실제 환경으로 전이. Domain Adaptation의 대표적 사례.

**Gaussian Splatting**: 3D 장면을 수백만 개의 3D 가우시안으로 표현하는 방법. NeRF보다 빠르고 편집 가능.

**Factor Graph**: 변수 간의 제약 조건을 그래프로 표현한 SLAM 최적화의 핵심 자료구조다.

**Knowledge Distillation**: 큰 모델(teacher)의 지식을 작은 모델(student)에 전달하는 기법.

## B. 자주 묻는 질문 (FAQ)

**Q: Python과 C++ 중 어떤 것을 먼저 배워야 하나요?**

A: SLAM, ROS 패키지, 실시간 제어 모듈에는 C++가 널리 쓰이므로 연구실 코드를 읽고 수정하려면 C++가 필요하다. Python은 딥러닝 스크립트와 데이터 전처리에 주로 쓴다. AI 코딩 에이전트가 작성을 도울 수는 있지만, 두 언어 모두 기존 코드를 읽고 동작을 검증할 수 있어야 한다.

**Q: GPU가 없으면 연구를 할 수 없나요?**

A: 간단한 실험은 CPU로 가능하다. 하지만 딥러닝 학습에는 GPU가 필수이다. Google Colab(무료)이나 연구실 서버를 활용하자. Colab 무료 버전으로도 YOLO fine-tuning 정도는 충분히 가능하다.

**Q: ROS1과 ROS2 중 어떤 것을 배워야 하나요?**

A: 새로 배운다면 ROS2를 권장한다. ROS1은 2025년에 공식 지원이 종료(EOL)되었다. 하지만 사용하려는 패키지가 ROS1만 지원하면 어쩔 수 없이 ROS1을 먼저 배울 수도 있다. 다만 ROS1을 알면 ROS2는 금방 익힌다.

**Q: 논문을 어디서 찾나요?**

A: [arXiv](https://arxiv.org/) (무료 프리프린트 서버)와 [Google Scholar](https://scholar.google.com/) (논문 검색)를 주로 활용하자. 코드가 같이 필요하면 [Papers With Code](https://paperswithcode.com/)가 편하다. 학회별로 보고 싶으면 [CVPR Open Access](https://openaccess.thecvf.com/)나 [IEEE Xplore](https://ieeexplore.ieee.org/)도 유용하다.

**Q: SLAM을 공부하려면 어디서 시작해야 하나요?**

A: Cyrill Stachniss의 [YouTube SLAM 강의](https://www.youtube.com/playlist?list=PLgnQpQtFTOGQrZ4O5QzbIHgl3b1JHimN_)로 시작하고, ORB-SLAM3 코드를 분석해보자. 그 전에 본 문서의 9장(카메라 모델)과 14장(Visual Odometry)을 읽으면 강의가 훨씬 잘 들릴 것이다.

**Q: 연구 아이디어는 어떻게 찾나요?**

A: 최신 학회 논문의 Limitation 섹션을 읽어보자. 해결되지 않은 문제에서 아이디어를 얻을 수 있다. 또 다른 방법은 아직 합쳐지지 않은 두 분야를 합치는 것이다. "3D Gaussian Splatting + Semantic SLAM"처럼.

**Q: 어떤 GPU를 사야 하나요?**

A: 먼저 모델·optimizer state·activation·batch가 VRAM에 들어가는지 실제 설정으로 측정한다. 그다음 precision 지원, memory bandwidth, 전력, framework 호환성, 실제 benchmark를 본다. 표의 용도는 구매 순위를 정하는 것이 아니라 후보를 VRAM 등급별로 좁히는 것이다.

개인용 (데스크톱)

| VRAM | 카드 예시 | 검토할 역할 | 구매 전 확인 |
|------|-----------|-------------|--------------|
| 8GB | RTX 4060, RTX 5060 Ti 8GB | 작은 CNN 학습, 제한된 batch의 추론 | intended model의 peak memory; VFM fine-tuning은 설정에 따라 부족할 수 있음 |
| 12GB | RTX 3060 12GB, RTX 4070 | 중간 크기 inference·학습 실험 | 세대별 연산 차이와 중고 상태 |
| 16GB | RTX 5060 Ti 16GB, RTX 4070 Ti Super | 더 큰 batch, VFM inference, 중간 규모 학습 | model별 activation·optimizer memory |
| 24GB | RTX 3090, RTX 4090 | 24GB 안에 드는 학습, 3DGS·VLA 실험 | 전력·냉각·중고 보증; 3090과 4090의 runtime 차이 |
| 32GB | RTX 5090 | 24GB를 넘는 로컬 실험 | 전력·케이스·PSU와 software support |

서버/연구실용 (데이터센터)

| GPU memory | 카드 | 특징 | 공식 사양 |
|------------|------|------|-----------|
| 16/32GB | V100 SXM2 | 1세대 Tensor Core, TF32/BF16 미지원 | [V100 data center GPU](https://www.nvidia.com/en-us/data-center/v100/) |
| 24GB | A10 | PCIe inference·graphics 계열 | [A10 datasheet](https://www.nvidia.com/content/dam/en-zz/Solutions/Data-Center/a10/pdf/a10-datasheet.pdf) |
| 40/80GB | A100 | TF32·BF16, MIG, PCIe/SXM 변형 | [A100 specifications](https://www.nvidia.com/en-us/data-center/a100/) |
| 80GB | H100 SXM | Hopper, Transformer Engine, NVLink | [H100 specifications](https://www.nvidia.com/en-us/data-center/h100/) |
| 141GB | H200 SXM | 141GB HBM3e, 4.8TB/s memory bandwidth | [H200 specifications](https://www.nvidia.com/en-us/data-center/h200/) |
| 180GB | B200 | Blackwell, 180GB HBM3e; server configuration에 따라 제공 | [DGX B200 specifications](https://www.nvidia.com/en-us/data-center/dgx-b200/) |

사양을 읽을 때는 precision(FP32·TF32·BF16·FP16·FP8), CUDA core와 Tensor Core, dense와 structured sparsity, PCIe와 SXM을 구분한다. 제조사 표의 peak TFLOPS가 같아도 memory bandwidth, kernel, batch, data loading 때문에 실제 학습 시간은 달라진다. `torch.amp`의 효과도 model과 hardware에 따라 달라지므로, 같은 repository·batch·precision으로 짧은 benchmark를 돌려 비교한다.

**참고 사항**:
- RTX 5060 Ti처럼 8GB와 16GB 모델이 함께 나오는 카드에서는 사용할 모델과 batch가 요구하는 VRAM을 먼저 계산한다. VFM을 로컬에서 다룬다면 8GB는 빠듯할 수 있다.
- AMD GPU를 고려할 때는 사용할 프레임워크와 라이브러리가 ROCm을 지원하는지 확인한다. CUDA 전용 의존성이 있다면 이식 비용도 구매 조건에 포함한다.
- 연구실 서버에 A100/H100이 있다면 개인 GPU의 역할은 디버깅과 프로토타이핑에 가까워진다. 구매 전에 서버의 사용 가능 시간과 사양을 확인한다.
- 중고 RTX 3090은 24GB 선택지지만 가격·보증·냉각 상태가 매물마다 다르다. 정격 전력과 PSU·케이스 조건도 확인한다.
- Colab과 cloud GPU의 요금·할당 GPU·사용 제한은 수시로 바뀐다. 구매 전 실제 workload를 대여 GPU에서 측정하되, 현재 provider 페이지의 가격과 quota를 확인한다.

**Q: 논문은 하루에 몇 편 읽어야 하나요?**

A: 하루에 몇 편을 읽는지보다 읽는 목적과 깊이가 중요하다. 처음에는 20.4절의 3-패스 방법에 따라 일주일에 한 편을 자세히 읽는 편이 도움이 된다. 경험이 쌓이면 Abstract만으로도 논문의 유형과 관련성을 빠르게 가늠할 수 있다. 랩미팅 발표를 위한 읽기와 자신의 연구를 위한 읽기도 깊이가 다르며, 후자는 코드 분석까지 이어질 수 있다.

**Q: 코딩을 잘 못하는데 연구를 할 수 있나요?**

A: 코딩 에이전트(Claude, Copilot 등)는 "KITTI 데이터셋 로더를 만들어줘"나 "이 학습 루프에 wandb 로깅을 추가해줘" 같은 요청으로 초안을 빠르게 만들 수 있다. 덕분에 직접 타이핑하는 시간은 줄었지만, 결과를 검토하는 일은 남는다.

생성된 코드를 판단하려면 도메인 지식이 필요하다. 에이전트가 `num_workers=0`일 때 DataLoader가 느린 이유, loss가 NaN이 되는 원인, SLAM 코드에서 좌표계가 뒤집힌 지점을 충분한 실행 정보 없이 찾기는 어렵다(14장 참고). 코드를 실행하고 기존 구현과 비교한 뒤 받아들여야 한다.

ORB-SLAM3, Ultralytics, HuggingFace Transformers 같은 오픈소스를 읽고 설계 이유를 추적하면 코드 검토 능력을 익히는 데 도움이 된다.

**Q: 학회 발표는 어떻게 준비하나요?**

A: 학회 발표는 크게 구두 발표(oral)와 포스터 발표(poster)로 나뉜다. 발표 시간·포스터 크기·언어는 학회마다 다르므로 공식 발표자 안내가 우선한다.

- 포스터: A0는 자주 보이는 크기지만 학회 지정 규격을 확인한다. Figure를 크게 두고 텍스트를 줄여, 지나가는 사람이 짧은 시간에 주제와 결과를 찾을 수 있게 한다. 발표 전에는 연구실 동료들 앞에서 연습한다.
- 구두 발표: 15-20분은 흔한 예일 뿐이며 세션별 제한 시간이 우선한다. 시간에 맞춰 슬라이드 수를 정하고 한 장의 메시지를 하나로 좁힌다. 데모 영상과 질문용 보충 슬라이드는 필요할 때 준비한다.
- 공통: 발표 언어를 확인한 뒤 스크립트로 연습하되 문장 암기보다 내용 전달과 시간 준수에 집중한다.

**Q: 영어 논문 읽기가 너무 힘든데요?**

A: 반복해서 읽고 분야 배경지식이 쌓이면 부담이 줄어든다.

- 구조를 먼저 파악하라: 많은 실험 논문은 Introduction → Related Work → Method → Experiments → Conclusion 순서를 쓰지만, 기여는 문제 정의·데이터·평가나 분석에도 놓일 수 있다. 제목과 헤딩으로 실제 구조부터 확인한다.
- 분야별 어휘를 먼저 익혀라: "ablation study", "state-of-the-art", "we empirically show" 같은 표현은 반복된다. 익숙해지는 데 필요한 논문 수는 배경지식과 분야에 따라 다르다.
- 번역 도구를 부끄러워하지 마라: DeepL, Google Translate로 모르는 문장을 번역하는 건 전혀 부끄러운 일이 아니다. 다만, 번역에만 의존하면 영어 실력이 안 는다. "원문 → 번역 확인 → 다시 원문" 순서로 읽자.
- PDF 리더의 형광펜을 활용하라: 중요한 문장을 표시하면 다시 찾기 쉽다. Adobe Acrobat이나 Zotero 내장 뷰어처럼 자신에게 편한 도구를 쓰면 된다.

## C. 트러블슈팅 가이드

**자주 쓰는 apt 명령어**

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

```bash
# 키 생성 (Enter 연타로 기본값 사용)
ssh-keygen -t ed25519

# 공개키를 서버에 복사
ssh-copy-id user@server_ip

# 이후 비밀번호 없이 접속 가능
ssh user@server_ip
```

GitHub에도 같은 공개키(`~/.ssh/id_ed25519.pub`)를 등록하면 `git push`에 비밀번호가 필요 없다. (참고: [정진용 블로그](https://jinyongjeong.github.io/2016/06/02/SSH_keygen_setting/))

**CPU 성능 모드 설정 (실험 시)**

SLAM이나 딥러닝 실험에서 CPU throttling 때문에 성능이 들쭉날쭉한 경우가 있다.

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

노트북에서는 배터리 소모가 커지니 전원 연결 상태에서만 사용할 것. (참고: [정진용 블로그](https://jinyongjeong.github.io/2020/02/04/Ubuntu_cpu_freq_change/))

### C.1 CUDA / PyTorch 관련

**문제**: `CUDA out of memory`

**해결**:

```python
# 1. 배치 사이즈 줄이기 (가장 먼저 시도)
batch_size = 16  # → 8 또는 4

# 2. 메모리 정리
torch.cuda.empty_cache()

# 3. Gradient accumulation 사용 (배치 효과는 유지하면서 메모리 절약)
accumulation_steps = 4
for i, (inputs, labels) in enumerate(dataloader):
    loss = model(inputs, labels) / accumulation_steps
    loss.backward()
    if (i + 1) % accumulation_steps == 0:
        optimizer.step()
        optimizer.zero_grad()

# 4. Mixed Precision Training (메모리 절반으로)
from torch.cuda.amp import autocast, GradScaler
scaler = GradScaler()
with autocast():
    output = model(input)
    loss = criterion(output, target)
```

**문제**: `CUDA version mismatch`

**해결**:

```bash
# 설치된 CUDA 버전 확인
nvcc --version

# PyTorch에서 인식하는 CUDA 버전 확인
python -c "import torch; print(torch.version.cuda)"

# 둘이 다르면 PyTorch 재설치 (CUDA 버전에 맞춰서)
pip install torch --index-url https://download.pytorch.org/whl/cu121
```

**문제**: `RuntimeError: CUDA error: device-side assert triggered`

**해결**: 이건 보통 라벨 인덱스가 범위를 벗어났을 때 발생한다. CPU에서 돌려보면 더 자세한 에러 메시지가 나온다.

```bash
CUDA_LAUNCH_BLOCKING=1 python train.py
```

### C.2 ROS 관련

**문제**: `Package not found`

**해결**:

```bash
# Workspace 소싱 확인
source ~/ros2_ws/install/setup.bash

# 패키지 설치 확인
ros2 pkg list | grep package_name

# .bashrc에 소싱 추가 (매번 수동으로 안 해도 됨)
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

**문제**: `TF tree not connected`

**해결**:

```bash
# TF 트리 확인
ros2 run tf2_tools view_frames

# Static transform 추가 (예시)
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link camera_link
```

**문제**: `Topic not published` / 데이터가 안 들어옴

**해결**:

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

**해결**:

```bash
# 도커 그룹에 사용자 추가
sudo usermod -aG docker $USER
# 로그아웃 후 재로그인
```

**문제**: GUI 프로그램 실행 안 됨

**해결**:

```bash
# X11 forwarding
xhost +local:docker
docker run -it --env DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix ...
```

**문제**: Docker 안에서 GPU가 안 잡힘

**해결**:

```bash
# nvidia-container-toolkit 설치
sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker

# GPU 옵션 추가해서 실행
docker run --gpus all -it nvidia/cuda:12.1.0-base-ubuntu22.04 nvidia-smi
```

### C.4 OpenCV 관련

**문제**: `cv2.imshow() not working`

**해결**:

```bash
# OpenCV headless 버전 제거 후 재설치
pip uninstall opencv-python-headless
pip install opencv-python
```

**문제**: OpenCV와 ROS의 cv_bridge 충돌

**해결**:

```bash
# ROS의 cv_bridge가 시스템 OpenCV를 참조하는 경우
# conda/venv 환경의 OpenCV와 충돌할 수 있다
# 해결: ROS workspace 빌드 시 Python 경로 명시

colcon build --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

### C.5 빌드/컴파일 관련

**문제**: ORB-SLAM3 빌드 에러 (OpenCV 버전 충돌)

**해결**:

```bash
# OpenCV 4.x에서는 일부 API가 변경됨
# CMakeLists.txt에서 OpenCV 버전 확인
find_package(OpenCV 4 REQUIRED)

# Pangolin 빌드 에러 시
sudo apt-get install libglew-dev libpython2.7-dev
```

**문제**: Eigen 버전 관련 에러

**해결**:

```bash
# 시스템 Eigen 버전 확인
pkg-config --modversion eigen3

# 특정 버전 필요 시 직접 설치
sudo apt-get install libeigen3-dev
```

## D. 체크리스트: 연구 시작 전 확인 사항

### D.1 환경 설정

- [ ] Ubuntu 설치 완료 (22.04 LTS 권장)
- [ ] NVIDIA 드라이버 설치 (`nvidia-smi`로 확인)
- [ ] CUDA / cuDNN 설치 (`nvcc --version`으로 확인)
- [ ] Conda 또는 venv 환경 설정
- [ ] PyTorch GPU 동작 확인 (`torch.cuda.is_available()`)
- [ ] ROS2 설치 (필요시, Humble 또는 Jazzy)
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

논문 읽기·쓰기 도구는 [「연구노트」 Part 2 — 쓰기](../research-notes/guide.html#chapter-16)에서, 학회 준비 도구는 [「연구노트」 Ch.34 — 학회 2-3주 전 체크리스트](../research-notes/guide.html#chapter-34)에서 확인할 수 있다. Spatial AI 분야의 도구와 학습 순서는 §20.4와 §20.7을 참고한다.

### D.4 데이터셋 준비

- [ ] 연구 관련 데이터셋 다운로드
- [ ] 데이터 포맷 이해 (이미지 크기, depth 단위, 좌표계)
- [ ] DataLoader 구현 (PyTorch Dataset/DataLoader)
- [ ] 데이터 시각화 코드 작성 (디버깅용)

## E. 첫 주 생존 가이드

첫 주에는 계정과 실행 환경을 준비하고, 연구 대상의 코드·데이터·문서를 찾는 최소 작업 목록이 필요하다. 아래 Day 1-7 배치는 예시이며 계정 발급, 장비 일정, 연구실 onboarding 방식에 맞춰 순서를 바꾼다.

### Day 1-2: 환경 구축

```
[ ] 연구실 서버 계정 받기 (관리자에게 요청)
[ ] SSH로 서버 접속 확인
[ ] VS Code Remote-SSH 설정
[ ] 서버에 conda 환경 만들기
[ ] PyTorch + CUDA 동작 확인
[ ] 연구실 GitHub organization에 가입
[ ] Slack/Discord 채널 가입
```

> 팁: 서버 환경 설정에서 막히면 시도한 명령과 예상·실제 결과를 함께 정리해 선배에게 묻는다. 확인한 내용을 보여 주면 문제를 훨씬 빨리 좁힐 수 있다.

### Day 3-4: 기존 코드 파악

```
[ ] 연구실의 기존 코드/프로젝트 리포지토리 클론
[ ] README 읽기 (있다면)
[ ] 기존 코드 빌드/실행 해보기
[ ] 데이터셋 다운로드 및 경로 설정
[ ] 간단한 데모 돌려보기
```

> 팁: 처음 실행한 코드가 바로 동작하지 않는 경우가 많다. 환경, 경로, 버전 차이를 확인하고 에러 메시지로 공식 문서와 issue를 먼저 찾아본다.

### Day 5: 논문 읽기 시작

첫 논문을 추천받고 연구실 구성원과 대화하는 방법은 [「대학원노트」 Ch.4 — 관계는 양방향](../grad-notes/guide.html#chapter-4)에서, 첫 주의 연구 방향 설정은 [「대학원노트」 Ch.7 — 내 연구를 갖기](../grad-notes/guide.html#chapter-7)에서 다룬다.

> 팁: 처음 읽는 논문은 이해가 안 되는 게 정상이다. "이 논문이 무슨 문제를 풀려고 하는가?"만 파악해도 첫 주로서는 충분하다.

### Day 6-7: 연구 방향 파악

Ch.18의 연구 방향을 읽고 연구실의 최근 논문과 프로젝트가 어느 주제에 해당하는지 정리한다. 선배들의 연구 주제와 겹치는 지점도 함께 표시한다.

### 첫 주에 하지 않아도 되는 것들

- 논문을 완벽하게 이해하기 — 시간이 해결해준다
- 최신 연구 트렌드를 전부 파악하기 — 점진적으로
- 코드를 처음부터 짜기 — 기존 코드를 수정하는 것부터
- GPU 서버를 완벽하게 세팅하기 — 연구실이 검증한 환경 파일·container·설치 절차에서 시작한다
- 연구 아이디어를 완성된 형태로 내놓기 — 먼저 연구실의 문제와 도구를 배워도 늦지 않다

### 생존을 위한 마인드셋

연구 초기에 필요한 태도와 작업 습관은 research-notes와 grad-notes에서 주제별로 다룬다.

관련 장은 다음과 같다.

- *모르는 건 당연하다* → [「대학원노트」 Ch.14 — 자율성의 무게](../grad-notes/guide.html#chapter-14) § 2 (대학원의 가치 재정의)
- *"안 돼요"는 보고가 아니다* + 보고 형식(예측·시도·결과) → [「대학원노트」 Ch.10 — 한 메일 한 질문](../grad-notes/guide.html#chapter-10) § 3 (형식의 사소한 표준)
- *기록하라 — 과거의 내가 미래의 나를 도와준다* → [「대학원노트」 Ch.8 — 시간 쓰는 법](../grad-notes/guide.html#chapter-8) § 5 (퇴근 전 포스트잇)
- *작게 시작하라 — 작은 코드 조각부터* → [「대학원노트」 Ch.11 — 도구의 함정](../grad-notes/guide.html#chapter-11) § 1 (셋업 시간은 연구 시간이 아니다)
- *비교하지 마라 — 3개월 후의 나* → [「대학원노트」 Ch.15 — 비교의 함정](../grad-notes/guide.html#chapter-15) § 3 (단거리 vs 장거리)
