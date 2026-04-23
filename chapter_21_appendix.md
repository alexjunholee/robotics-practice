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

**Factor Graph**: 변수 간의 제약 조건을 그래프로 표현. SLAM 최적화의 핵심 자료구조.

**Knowledge Distillation**: 큰 모델(teacher)의 지식을 작은 모델(student)에 전달하는 기법.

## B. 자주 묻는 질문 (FAQ)

**Q: Python과 C++ 중 어떤 것을 먼저 배워야 하나요?**

A: 연구실 코드의 핵심은 C++이다. SLAM, ROS 패키지, 실시간 제어 모듈 전부 C++로 되어 있고, 이 코드를 읽고 수정할 일이 많다. Python은 딥러닝 스크립트나 데이터 전처리에 쓰이지만, AI 코딩 에이전트가 잘 도와주는 영역이라 직접 숙달할 필요성은 줄었다. 둘 다 "코드를 읽고 이해하는 능력"이 핵심이고, 작성은 AI와 협업하면 된다.

**Q: GPU가 없으면 연구를 할 수 없나요?**

A: 간단한 실험은 CPU로 가능하다. 하지만 딥러닝 학습에는 GPU가 필수이다. Google Colab(무료)이나 연구실 서버를 활용하자. Colab 무료 버전으로도 YOLO fine-tuning 정도는 충분히 가능하다.

**Q: ROS1과 ROS2 중 어떤 것을 배워야 하나요?**

A: 새로 배운다면 ROS2를 권장한다. ROS1은 2025년에 공식 지원이 종료(EOL)되었다. 하지만 사용하려는 패키지가 ROS1만 지원하면 어쩔 수 없이 ROS1을 먼저 배울 수도 있다. 다만 ROS1을 알면 ROS2는 금방 익힌다.

**Q: 논문을 어디서 찾나요?**

A: [arXiv](https://arxiv.org/) (무료 프리프린트 서버), [Google Scholar](https://scholar.google.com/) (논문 검색), [Papers With Code](https://paperswithcode.com/) (코드 포함 논문 검색)를 활용하자. 학회별로 보고 싶으면 [CVPR Open Access](https://openaccess.thecvf.com/)나 [IEEE Xplore](https://ieeexplore.ieee.org/)도 유용하다.

**Q: SLAM을 공부하려면 어디서 시작해야 하나요?**

A: Cyrill Stachniss의 [YouTube SLAM 강의](https://www.youtube.com/playlist?list=PLgnQpQtFTOGQrZ4O5QzbIHgl3b1JHimN_)로 시작하고, ORB-SLAM3 코드를 분석해보자. 그 전에 본 문서의 9장(카메라 모델)과 14장(Visual Odometry)을 읽으면 강의가 훨씬 잘 들릴 것이다.

**Q: 연구 아이디어는 어떻게 찾나요?**

A: 최신 학회 논문의 Limitation 섹션을 읽어보자. 해결되지 않은 문제에서 아이디어를 얻을 수 있다. 또 다른 방법: "3D Gaussian Splatting + Semantic SLAM"처럼 아직 합쳐지지 않은 두 분야를 합치는 시도.

**Q: 어떤 GPU를 사야 하나요?**

A: GPU 선택에서 가장 중요한 스펙은 **VRAM**이다. 모델 크기가 VRAM에 안 들어가면 아예 못 돌린다. 그다음이 연산 속도(TFLOPS)이고, 학습 시간에 직접 영향을 준다.

**개인용 (데스크톱)**

| VRAM | 카드 | FP32 TFLOPS | FP16 TFLOPS | 용도 |
|------|------|-------------|-------------|------|
| 8GB | RTX 4060 | 15.1 | 15.1 | YOLOv8, ResNet 학습, 소규모 fine-tuning |
| 8GB | RTX 5060 Ti 8GB | ~30 | ~30 | 4070급 연산이지만 VRAM이 8GB라 VFM은 빠듯 |
| 12GB | RTX 3060 12GB | 12.7 | 12.7 | 연산은 느리지만 12GB VRAM이 이 가격대에서 유일. 중고로 저렴. 학생 엔트리용 |
| 12GB | RTX 4070 | 29.1 | 29.1 | Depth Anything, SegFormer. SAM은 batch 1로 겨우 가능 |
| 16GB | RTX 5060 Ti 16GB | ~30 | ~30 | SAM, DINOv2 inference. 중간 규모 학습. **가성비 추천** |
| 16GB | RTX 4070 Ti Super | 44.1 | 44.1 | 위와 VRAM 동일, 연산 속도가 1.5배 |
| 24GB | RTX 3090 (중고) | 35.6 | 35.6 | VRAM 24GB를 가장 싸게 확보. 학습 가능, 속도만 느림 |
| 24GB | RTX 4090 | 82.6 | 82.6 | 개인용 최고. VLA fine-tuning, 3DGS 대형 장면 |
| 32GB | RTX 5090 | 104.8 | 209.6 | 개인용 최대 VRAM. FP16에서 4090의 2.5배 |

**서버/연구실용 (데이터센터)**

| VRAM | 카드 | FP32 TFLOPS | TF32 TFLOPS | BF16 TFLOPS | 특징 |
|------|------|-------------|-------------|-------------|------|
| 16/32GB | V100 SXM2 | 15.7 | — | — | Tensor Core 1세대. TF32 미지원. 아직 많은 연구실에서 현역. 중고로 싸게 구할 수 있다 |
| 24GB | A10 | 31.2 | 62.5 | 125.0 | 추론 서버용. 학습에는 느림 |
| 40/80GB | A100 SXM | 19.5 | 156 | 312 | FP32는 느리지만 TF32/BF16에서 압도적. NVLink로 multi-GPU 확장 |
| 80GB | H100 SXM | 66.9 | 989 | 1979 | A100 대비 TF32 6배, BF16 6배. Transformer Engine 지원 |
| 80GB | H200 SXM | 66.9 | 989 | 1979 | H100과 동일 연산, HBM3e로 메모리 대역폭 1.5배 |
| 141GB | B200 | 90 | 2250 | 4500 | 최신. 단일 GPU로 70B+ 모델 학습 가능 |

숫자를 읽는 법:
- **FP32**: 전통적 부동소수점. OpenCV, 고전 SLAM 등에서 사용. 개인 GPU는 이 수치가 실질 성능.
- **TF32/BF16**: PyTorch에서 `torch.cuda.amp` (mixed precision) 사용 시 적용. 학습 속도가 2-6배 빨라진다. 데이터센터 GPU(A100, H100)는 이 모드에서 진가를 발휘하므로, FP32 TFLOPS만 보고 "A100이 4090보다 느리네?"라고 판단하면 안 된다.
- **TFLOPS**: Tera Floating Point Operations Per Second. 높을수록 빠르다.

**참고 사항**:
- RTX 5060 Ti는 8GB와 16GB 두 버전이 있다. 반드시 16GB를 사라. 8GB는 VRAM이 부족해서 금방 한계에 부딪힌다.
- AMD GPU (RX 7900 XTX 등)는 ROCm 지원이 개선되고 있으나, CUDA 생태계와의 호환성 문제가 아직 있다. 트러블슈팅에 시간 쓰기 싫으면 NVIDIA를 사라.
- 연구실 서버에 A100/H100이 있다면 개인 GPU는 디버깅/프로토타이핑용이다. 서버 사양을 먼저 확인하자.
- 중고 RTX 3090 (24GB)은 VRAM 대비 가격이 가장 낫다. 전력 소모(350W)가 크고 소음이 심하다는 점은 감안해야 한다.
- Google Colab Pro(월 $10)로 A100을 시간 단위로 쓸 수도 있다. GPU 구매 전에 먼저 시도해 볼 만하다.

**Q: 논문은 하루에 몇 편 읽어야 하나요?**

A: "하루에 N편" 같은 기준은 의미 없다. 처음에는 *일주일에 1편을 완벽하게* 이해하는 게 훨씬 낫다. 20.4절의 3-패스 방법을 따라서, 한 편을 깊이 파고들자. 6개월쯤 지나면 Abstract만 봐도 "아, 이런 류의 논문이구나" 감이 온다. 그때부터 속도가 붙는다. 참고로, 랩미팅 발표를 위해 논문을 읽는 것과 자기 연구를 위해 읽는 것은 깊이가 다르다. 후자는 코드까지 분석해야 한다.

**Q: 코딩을 잘 못하는데 연구를 할 수 있나요?**

A: 2026년 기준, 코드를 한 줄 한 줄 직접 쓰는 능력보다 *코딩 에이전트(Claude, Copilot 등)를 잘 활용하는 능력*이 더 중요해졌다. 에이전트에게 "KITTI 데이터셋 로더 만들어줘", "이 학습 루프에 wandb 로깅 추가해줘"라고 시키면 코드가 나온다. 직접 타이핑하는 시간은 크게 줄었다.

다만, 에이전트가 만든 코드가 맞는지 틀린지 판단하려면 도메인 지식이 있어야 한다. "이 DataLoader의 num_workers가 왜 0이면 느린지", "이 loss가 왜 NaN이 나오는지", "이 SLAM 코드에서 좌표계가 왜 뒤집혀 있는지" — 이런 건 에이전트가 알아서 못 잡는다(14장 참고). 결국 좋은 코드가 왜 좋은지, 나쁜 코드가 왜 나쁜지를 구분하는 눈이 필요하고, 그 눈은 좋은 코드를 많이 읽어야 생긴다.

추천 접근법: 유명 오픈소스(ORB-SLAM3, Ultralytics, HuggingFace Transformers 등)의 코드를 읽으면서 "왜 이렇게 짰는지"를 이해하라. 코딩 실력은 타이핑 속도가 아니라 코드를 읽고 판단하는 능력이다.

**Q: 학회 발표는 어떻게 준비하나요?**

A: 학회 발표는 크게 **구두 발표(oral)**와 **포스터 발표(poster)**로 나뉜다.

- **포스터**: 대부분의 첫 발표는 포스터다. A0 크기로 연구 내용을 요약한다. 핵심은 Figure를 크게, 텍스트를 적게. 지나가는 사람이 3초 안에 관심을 가지게 해야 한다. 발표 연습은 연구실 동료들 앞에서 최소 3번은 하자.
- **구두 발표**: 보통 15-20분이다. 슬라이드는 20장 이내, 한 슬라이드에 한 메시지. 데모 영상이 있으면 좋다. 질문에 대비해서 supplementary 슬라이드도 준비하자.
- 공통: 영어 발표가 대부분이니, 스크립트를 써서 연습하되 외우지는 말자. 자연스러운 영어보다 내용 전달이 중요하다.

**Q: 영어 논문 읽기가 너무 힘든데요?**

A: 이건 정말 시간이 해결해준다. 몇 가지 팁:

- **구조를 먼저 파악하라**: 대부분의 논문은 Introduction → Related Work → Method → Experiments → Conclusion 구조다. Method만 진짜 새로운 내용이고, 나머지는 패턴이 비슷하다.
- **분야별 어휘를 먼저 익혀라**: "ablation study", "state-of-the-art", "we empirically show" 같은 표현은 반복된다. 처음 20편쯤 읽으면 이런 표현에 익숙해진다.
- **번역 도구를 부끄러워하지 마라**: DeepL, Google Translate로 모르는 문장을 번역하는 건 전혀 부끄러운 일이 아니다. 다만, 번역에만 의존하면 영어 실력이 안 는다. "원문 → 번역 확인 → 다시 원문" 순서로 읽자.
- **PDF 리더에서 형광펜을 활용하라**: 핵심 문장을 색칠하면서 읽으면 집중도가 올라간다. Adobe Acrobat, Zotero 내장 뷰어 등 본인이 편한 도구를 쓰면 된다.

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

- [ ] 논문 읽기/관리 환경 세팅 (PDF 리더 + AI 요약 워크플로우, 또는 Zotero)
- [ ] 코드 에디터 (VS Code — 원격 서버 SSH 연결 가능)
- [ ] 실험 로깅 (Weights & Biases 또는 MLflow)
- [ ] GitHub 계정 (+ SSH 키 설정)
- [ ] LaTeX 환경 (Overleaf 추천 — 온라인, 협업 가능)
- [ ] 슬라이드 도구 (Google Slides, Keynote, 또는 LaTeX Beamer)

### D.4 데이터셋 준비

- [ ] 연구 관련 데이터셋 다운로드
- [ ] 데이터 포맷 이해 (이미지 크기, depth 단위, 좌표계)
- [ ] DataLoader 구현 (PyTorch Dataset/DataLoader)
- [ ] 데이터 시각화 코드 작성 (디버깅용)

## E. 첫 주 생존 가이드

연구실에 처음 들어왔을 때, 뭘 해야 할지 막막한 건 당연하다. 이 가이드는 "첫 주에 최소한 이것만은 하자"를 정리한 것이다.

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

> 팁: 서버 환경 설정에서 막히면 선배에게 물어보자. "이걸 해봤는데 예상과 다르게 이런 결과가 나왔다"라고 하면 거의 100% 도와준다. 아무것도 안 해보고 물어보면... 알아서 하라고 할 수도 있다.

### Day 3-4: 기존 코드 파악

```
[ ] 연구실의 기존 코드/프로젝트 리포지토리 클론
[ ] README 읽기 (있다면)
[ ] 기존 코드 빌드/실행 해보기
[ ] 데이터셋 다운로드 및 경로 설정
[ ] 간단한 데모 돌려보기
```

> 팁: 코드가 돌아가지 않는 건 정상이다. 환경이 다르고, 경로가 다르고, 버전이 다르니까. 에러 메시지를 복사해서 Google에 검색하면 대부분 Stack Overflow에 답이 있다.

### Day 5: 논문 읽기 시작

```
[ ] 지도교수/선배에게 "먼저 읽어야 할 논문 3편" 추천받기
[ ] 논문 관리 환경 세팅 (PDF 리더 + AI 워크플로우)
[ ] 추천받은 논문 3편의 Abstract와 Conclusion 읽기
[ ] 모르는 용어 정리 (본 문서의 부록 A 활용)
```

> 팁: 처음 읽는 논문은 이해가 안 되는 게 정상이다. "이 논문이 무슨 문제를 풀려고 하는가?"만 파악해도 첫 주로서는 충분하다.

### Day 6-7: 연구 방향 파악

```
[ ] 연구실 최근 논문/프로젝트 살펴보기
[ ] 선배들의 연구 주제 파악 (누가 어떤 주제를 하고 있는지)
[ ] 본 문서의 18장(연구실 연구 방향) 읽기
[ ] 관심 있는 연구 방향 2-3개 메모
[ ] 다음 주 랩미팅에서 발표할 자기소개 준비
```

### 첫 주에 하지 않아도 되는 것들

- 논문을 완벽하게 이해하기 — 시간이 해결해준다
- 최신 연구 트렌드를 전부 파악하기 — 점진적으로
- 코드를 처음부터 짜기 — 기존 코드를 수정하는 것부터
- GPU 서버를 완벽하게 세팅하기 — 선배 환경을 복사하자
- 연구 아이디어를 내기 — 최소 1-2개월은 배우는 시간이다

### 생존을 위한 마인드셋

1. **모르는 건 당연하다**: 3학년 학부생이 SLAM을 모르는 건 당연하다. 부끄러워하지 말고 물어보자.
2. **선배를 활용하라**: 선배들은 같은 고통을 겪었기에 도와주고 싶어한다. 다만 "안 돼요"는 보고가 아니다. "무엇을 예측해서, 무엇을 어떻게 했는데, 예상과 달리 어떤 결과가 나왔다"라고 말해야 한다. 교수에게도 마찬가지다.
3. **기록하라**: 오늘 뭘 했는지, 뭘 모르겠는지, 어떤 에러가 났는지 기록해두자. 나중에 같은 문제를 만났을 때 과거의 내가 도와준다.
4. **작게 시작하라**: 거대한 시스템을 이해하려 하지 말고, 작은 코드 조각부터 돌려보자. "이 함수가 뭘 하는지" 하나만 이해해도 진전이다.
5. **비교하지 마라**: 선배가 논문을 술술 읽는 건 그만큼 시간을 투자했기 때문이다. 3개월 후의 나는 지금보다 훨씬 잘할 것이다.
