# Ch.16 — 개발 환경 & 도구

로봇 연구를 하다 보면, 알고리즘 자체보다 "환경 세팅"에 시간을 더 많이 쓰는 경우가 흔하다. CUDA 버전이 안 맞아서 하루를 날리거나, 다른 사람 코드를 클론했는데 파이썬 버전 차이로 안 돌아가는 경험을 한 번쯤 하게 된다. 여기서 소개하는 도구들은 그런 삽질을 최소화해 주는 것들이다. 화려하지 않지만, 연구 생산성에 직접 영향을 미치는 실전 지식이다.

## 16.1 프로그래밍 언어

AI 코딩 에이전트가 코드 작성을 상당 부분 대신해 주는 시대이다. 이제 중요한 건 코드를 처음부터 짜는 능력보다, **기존 코드를 읽고 이해하는 능력**이다. 다른 사람의 연구 코드를 클론해서 구조를 파악하고, AI가 생성한 코드를 검증하고, 문제가 생겼을 때 어디를 고쳐야 하는지 판단할 수 있어야 한다.

## 16.1.1 C++

**용도**: 실시간 시스템, ROS 노드, SLAM, 성능 중요 모듈

연구실에서 다루는 핵심 코드 대부분이 C++이다. SLAM, 실시간 제어, ROS 패키지의 핵심 로직이 전부 C++로 작성되어 있고, 이 코드를 읽고 수정할 일이 많다. ORB-SLAM3, LOAM, VINS-Mono 같은 코드를 이해하려면 C++에 익숙해야 한다.

**장점**:
- 빠른 실행 속도
- 메모리 직접 제어
- ROS/SLAM 코드 대부분 C++

**단점**:
- 배우기 어려움
- 개발 속도 느림
- 메모리 관리 실수

**modern C++ (C++17/20)**:

```cpp
// 스마트 포인터
auto ptr = std::make_shared<MyClass>();

// Range-based for
for (const auto& item : container) { ... }

// Lambda
auto func = [&](int x) { return x * 2; };
```

> **추천 자료**
> - [C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines) — 모던 C++ 코딩 가이드 (Bjarne Stroustrup, Herb Sutter)
> - [The Cherno - C++ Playlist](https://www.youtube.com/playlist?list=PLlrATfBNZ98dudnM48yfGUldqGD0S4FFb) — C++ 기초부터 고급까지 영상 시리즈
> - [Modernes C++](https://www.modernescpp.com/index.php) — 모던 C++ (C++17/20/23) 기능을 체계적으로 정리한 블로그

> **⚠ AI 에이전트 주의**: AI가 생성한 C++ 코드가 x86에서는 빌드되지만 Jetson(ARM)에서 실패하는 경우가 많다. 크로스 컴파일 환경이나 타겟 아키텍처를 알려줘라.

## 16.1.2 Python

**용도**: 프로토타이핑, 딥러닝 학습/추론, 데이터 분석, 시각화

PyTorch 학습 스크립트나 데이터 전처리 같은 작업에 사용된다. 연구에서 자주 마주치긴 하지만, AI 에이전트가 가장 잘 다루는 언어이기도 해서 직접 작성하는 비중은 줄어들고 있다. 읽고 이해할 수 있으면 충분하다.

**자주 쓰는 라이브러리**:

```bash
pip install numpy scipy matplotlib
pip install opencv-python open3d
pip install torch torchvision
pip install transformers  # HuggingFace
```

> **추천 자료**
> - [Real Python](https://realpython.com/) — Python 기초부터 고급까지 체계적 튜토리얼
> - [Fireship - Python in 100 Seconds](https://www.youtube.com/watch?v=x7X9w_GIm1s) — Python 전체를 빠르게 훑어보는 영상

## 16.2 개발 환경 설정

## 16.2.1 Ubuntu

로보틱스 개발은 사실상 Ubuntu에서 한다. ROS가 Ubuntu를 1차 지원 플랫폼으로 삼고 있고, GPU 드라이버·CUDA·cuDNN 등의 호환성도 Ubuntu에서 가장 잘 검증됐기 때문이다. macOS나 Windows에서도 일부 개발이 가능하지만, 결국 실제 로봇에 올릴 때는 Ubuntu로 돌아오게 된다.

**권장 버전**:
- Ubuntu 22.04 LTS (ROS2 Humble)
- Ubuntu 24.04 LTS (ROS2 Jazzy)

**초기 설정**:

```bash
# 기본 도구
sudo apt update && sudo apt upgrade -y
sudo apt install -y build-essential cmake git curl wget

# Python 관련
sudo apt install -y python3-pip python3-venv

# 개발 도구
sudo apt install -y vim tmux htop
```

> **추천 자료**
> - [The Missing Semester of Your CS Education (MIT)](https://missing.csail.mit.edu/) — 셸, vim, tmux, Git 등 "수업에서는 안 가르치지만 매일 쓰는" 개발 도구를 체계적으로 정리. 추천한다
> - [Fireship - Linux in 100 Seconds](https://www.youtube.com/watch?v=rrB13utjYV4) — Linux가 뭔지 빠르게 감 잡기

## 16.2.2 CUDA / cuDNN

딥러닝 모델 학습은 CPU로는 현실적으로 불가능하다. GPU 가속을 위해 CUDA가 필요한데, PyTorch와 CUDA 버전이 안 맞으면 `import torch` 한 줄에서부터 에러가 난다. 로보틱스 연구자가 가장 많이 겪는 환경 문제 중 하나이다.

**설치 확인**:

```bash
nvidia-smi          # GPU 상태
nvcc --version      # CUDA 버전
```

**권장 버전**: CUDA 12.x, cuDNN 8.x

**주의**: PyTorch/TensorFlow 버전과 CUDA 버전 호환성 확인 필수

> **추천 자료**
> - [PyTorch - Previous Versions](https://pytorch.org/get-started/previous-versions/) — PyTorch와 CUDA 버전 매칭 확인. 설치 전 확인할 것
> - [NVIDIA CUDA Toolkit Documentation](https://docs.nvidia.com/cuda/) — CUDA 공식 문서

**NVIDIA 드라이버 설치 트러블슈팅**

Ubuntu에서 NVIDIA 드라이버 설치 시 가장 흔한 문제는 `nouveau` (오픈소스 드라이버)와의 충돌이다.

```bash
# nouveau 비활성화
sudo bash -c "echo blacklist nouveau > /etc/modprobe.d/blacklist-nvidia-nouveau.conf"
sudo bash -c "echo options nouveau modeset=0 >> /etc/modprobe.d/blacklist-nvidia-nouveau.conf"
sudo update-initramfs -u
sudo reboot

# 드라이버 설치 (권장: apt 사용)
sudo apt install nvidia-driver-535  # 버전은 GPU에 맞게 조정
sudo reboot

# 확인
nvidia-smi
```

설치 후 `nvidia-smi`에서 GPU가 안 보이면: (1) Secure Boot가 켜져 있는지 확인 (BIOS에서 끌 것), (2) `sudo dkms status`로 드라이버 모듈 상태 확인. (참고: [정진용 블로그](https://jinyongjeong.github.io/2016/11/22/ubuntu_graphic_driver_install/))

**CUDA/cuDNN 버전 호환성**

PyTorch와 CUDA 버전이 안 맞으면 `import torch` 한 줄에서 에러가 난다. 확인 순서:

```bash
# 1. GPU 확인
nvidia-smi  # 오른쪽 상단의 CUDA Version은 "드라이버가 지원하는 최대 버전"

# 2. 설치된 CUDA toolkit 확인
nvcc --version

# 3. PyTorch가 사용하는 CUDA 확인
python -c "import torch; print(torch.version.cuda)"

# 세 버전이 호환되어야 한다. PyTorch 공식 사이트에서 조합 확인:
# https://pytorch.org/get-started/locally/
```

(참고: [정진용 블로그](https://jinyongjeong.github.io/2016/09/19/cuda_setting/))

**OpenCV + CUDA 직접 빌드**

apt로 설치하는 `python3-opencv`나 `pip install opencv-python`은 CUDA 가속이 안 된다. GPU 가속이 필요하면 (DNN 모듈, optical flow 등) 소스에서 직접 빌드해야 한다.

```bash
# 의존성 설치
sudo apt install -y cmake git libgtk2.0-dev pkg-config \
    libavcodec-dev libavformat-dev libswscale-dev \
    libtbb-dev libjpeg-dev libpng-dev

# OpenCV + contrib 소스
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git

# 빌드 (CUDA 활성화)
cd opencv && mkdir build && cd build
cmake -D CMAKE_BUILD_TYPE=Release \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
      -D WITH_CUDA=ON \
      -D CUDA_ARCH_BIN="8.6"  \  # GPU에 맞게 조정 (RTX 3090=8.6, RTX 4090=8.9)
      -D WITH_CUDNN=ON \
      -D OPENCV_DNN_CUDA=ON \
      -D BUILD_opencv_python3=ON \
      ..
make -j$(nproc)
sudo make install
```

`CUDA_ARCH_BIN`은 자기 GPU에 맞춰야 한다. 틀리면 빌드는 되지만 런타임에 느리거나 에러가 난다. [NVIDIA GPU Compute Capability](https://developer.nvidia.com/cuda-gpus)에서 확인.

주의: ROS 환경에서 pip opencv와 apt cv_bridge가 충돌하는 문제가 있다 (19장 AI 에이전트 트러블슈팅 참고). CUDA OpenCV를 직접 빌드하면 이 문제가 더 복잡해질 수 있으니, Docker로 격리하는 것을 권장한다.

(참고: [다크 프로그래머 — OpenCV + CUDA 직접 빌드하기](https://darkpgmr.tistory.com/184))

## 16.2.3 환경 관리

프로젝트마다 필요한 Python 버전과 라이브러리 버전이 다르다. 환경 관리 도구 없이 `pip install`을 전역으로 하면 프로젝트 A에 필요한 라이브러리가 프로젝트 B와 충돌하는 "의존성 지옥(dependency hell)"에 빠진다. Conda나 venv로 프로젝트별 독립 환경을 만드는 것이 기본이다.

**Conda** (권장):

```bash
# Miniconda 설치
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh

# 환경 생성
conda create -n myenv python=3.10
conda activate myenv

# 패키지 설치
conda install pytorch torchvision pytorch-cuda=12.1 -c pytorch -c nvidia
```

**venv** (가벼움):

```bash
python3 -m venv myenv
source myenv/bin/activate
pip install -r requirements.txt
```

예를 들어 SLAM 연구에는 Python 3.8이 필요하고, 최신 트랜스포머 모델에는 Python 3.10이 필요할 수 있다. Conda를 쓰면 `conda activate slam_env`, `conda activate transformer_env`로 전환하면 끝이다.

> **추천 자료**
> - [Conda Documentation - Managing Environments](https://docs.conda.io/projects/conda/en/latest/user-guide/tasks/manage-environments.html) — Conda 환경 관리 공식 가이드
> - [Python venv Documentation](https://docs.python.org/3/library/venv.html) — 파이썬 공식 가상환경 문서

## 16.3 Docker

## 16.3.1 왜 Docker인가?

연구실에서 코드를 공유할 때 가장 많이 듣는 말이 "내 컴퓨터에선 돌아가는데..."이다. Docker는 그 문제를 해결한다. OS, 라이브러리, 환경 설정을 통째로 패키징해서 어디서든 동일한 실행 환경을 보장하기 때문이다. 논문 코드를 재현할 때도 Docker 이미지가 제공되면 훨씬 수월하다.

- **재현성**: 동일한 환경 보장
- **격리**: 시스템 오염 방지
- **배포**: 쉬운 공유 및 배포
- **의존성**: 복잡한 의존성 관리

## 16.3.2 기본 사용법

```bash
# 이미지 다운로드
docker pull nvidia/cuda:12.1.0-devel-ubuntu22.04

# 컨테이너 실행
docker run -it --rm \
    --gpus all \
    -v $(pwd):/workspace \
    nvidia/cuda:12.1.0-devel-ubuntu22.04 bash

# Dockerfile 빌드
docker build -t my_image .
```

**Dockerfile 예시**:

```dockerfile
FROM nvidia/cuda:12.1.0-devel-ubuntu22.04

RUN apt-get update && apt-get install -y \
    python3-pip git

COPY requirements.txt /tmp/
RUN pip3 install -r /tmp/requirements.txt

WORKDIR /workspace
```

> **추천 자료**
> - [Docker 공식 Getting Started Guide](https://docs.docker.com/get-started/) — Docker 처음이라면 여기부터. 컨테이너, 이미지, 볼륨 개념을 잘 설명
> - [NetworkChuck - Docker Tutorial](https://www.youtube.com/watch?v=eGz9DS-aIeY) — Docker를 재미있고 쉽게 설명하는 영상. 입문용으로 적합
> - [Fireship - Docker in 100 Seconds](https://www.youtube.com/watch?v=Gjnup-PuquQ) — Docker 핵심 개념을 빠르게 훑어보기

## 16.3.3 NVIDIA Container Toolkit

일반 Docker 컨테이너 안에서는 GPU가 보이지 않는다. 딥러닝 학습이나 CUDA 기반 연산을 하려면 nvidia-container-toolkit을 설치하고 `--gpus all` 플래그를 사용해야 한다. 로보틱스 연구에서 Docker를 쓴다면 거의 100% 필요하다.

참고: 과거에 쓰던 `nvidia-docker2`는 deprecated됐다. 현재는 `nvidia-container-toolkit`이 표준이고, `--runtime=nvidia` 대신 `--gpus all`을 쓴다.

```bash
# nvidia-container-toolkit 설치 (Ubuntu 22.04/24.04)
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
    sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit

# Docker 데몬에 NVIDIA 런타임 등록
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# 테스트
docker run --rm --gpus all nvidia/cuda:12.4.1-base-ubuntu22.04 nvidia-smi
```

## 16.3.4 실전 레시피: ROS2 + GPU + GUI + 센서

로보틱스에서 Docker를 쓸 때는 GPU, GUI(RViz/Gazebo), USB 센서를 동시에 써야 하는 경우가 대부분이다. 이걸 하나씩 붙이면 충돌하기 쉽고, 한 번에 설정하는 게 낫다.

[turlucode/ros-docker-gui](https://github.com/turlucode/ros-docker-gui)가 이 조합을 잘 정리해둔 프로젝트이니 참고할 것. 아래는 그 구조를 기반으로 현재 환경(nvidia-container-toolkit + ROS2 Humble)에 맞춘 레시피다.

**1단계: X11 포워딩 준비 (호스트)**

```bash
# 호스트에서 한 번만 실행
sudo apt-get install -y xauth
xhost +local:docker
```

`xhost +local:docker`는 Docker 컨테이너에서만 X 서버 접근을 허용한다. `xhost +`(전체 허용)보다 안전하지만, 보안이 중요한 환경이라면 `xauth` 기반 인증을 쓰는 게 맞다.

**2단계: 실행 스크립트**

```bash
#!/bin/bash
# run_ros2_docker.sh — GPU + GUI + USB 센서 풀 세팅

docker run --rm -it \
    --gpus all \
    --privileged \
    --net=host \
    --ipc=host \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e ROS_DOMAIN_ID=42 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $HOME/.Xauthority:/root/.Xauthority:ro \
    -v /dev:/dev \
    -v $HOME/catkin_ws:/root/catkin_ws \
    --name ros2_dev \
    osrf/ros:humble-desktop \
    bash
```

각 플래그가 뭘 하는지:

| 플래그 | 역할 |
|--------|------|
| `--gpus all` | GPU 패스스루 (nvidia-container-toolkit) |
| `--privileged` | USB/시리얼 디바이스 전체 접근. 프로덕션에서는 `--device`로 개별 매핑할 것 |
| `--net=host` | DDS multicast 통신을 위해 호스트 네트워크 공유. ROS2 노드 간 통신에 필수 |
| `--ipc=host` | 공유 메모리. Rviz 등 GUI 도구에서 필요 |
| `-e QT_X11_NO_MITSHM=1` | 이거 빠지면 RViz가 segfault로 죽는다. MIT-SHM이 Docker에서 안 됨 |
| `-e ROS_DOMAIN_ID=42` | 같은 네트워크의 다른 ROS2 시스템과 격리. 연구실에서 여러 명이 쓸 때 필수 |
| `-v /dev:/dev` | 센서 USB가 언제 꽂힐지 모르므로 /dev 전체 마운트. `--privileged`와 세트 |
| `-v .Xauthority` | X11 인증. 이걸 안 걸면 `cannot open display` |

**3단계: 작업 후 컨테이너 저장**

```bash
# 컨테이너 안에서 패키지 설치 등 작업을 했으면 커밋
docker commit ros2_dev my_ros2_workspace:v1

# 다음번에는 저장된 이미지로 실행
# run_ros2_docker.sh에서 이미지 이름만 바꾸면 된다
```

**Dockerfile로 관리하는 방식** (더 권장):

```dockerfile
FROM osrf/ros:humble-desktop

# 기본 도구
RUN apt-get update && apt-get install -y \
    python3-pip git wget curl vim \
    ros-humble-rviz2 \
    ros-humble-rqt* \
    && rm -rf /var/lib/apt/lists/*

# Python 패키지
RUN pip3 install torch torchvision numpy opencv-python-headless

# ROS2 워크스페이스
RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws

# 소스 빌드가 필요한 패키지가 있으면 여기서
# RUN cd src && git clone https://github.com/...
# RUN . /opt/ros/humble/setup.sh && colcon build

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
```

`docker commit`보다 Dockerfile이 나은 이유: 나중에 "이 이미지에 뭐가 깔려있지?"를 추적할 수 있다. commit으로 만든 이미지는 히스토리가 없어서 재현이 안 된다.

> **추천 자료**
> - [turlucode/ros-docker-gui](https://github.com/turlucode/ros-docker-gui) — ROS + NVIDIA + GUI Docker 설정 레퍼런스. Melodic부터 Humble까지 지원
> - [NVIDIA Container Toolkit Documentation](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/index.html) — 공식 설치 가이드
> - [OSRF Docker Images](https://hub.docker.com/r/osrf/ros) — ROS 공식 Docker 이미지. `humble-desktop`이 GUI 포함 버전

> **AI 에이전트 주의**: AI에게 Docker 설정을 물어볼 때는 "ROS2 + GPU + USB 센서 + GUI 시각화"를 동시에 쓸 건지 한 번에 알려줘라. 각각을 따로 물어보면 서로 충돌하는 설정을 준다. 특히 `QT_X11_NO_MITSHM=1`과 `ROS_DOMAIN_ID`는 AI가 빠뜨리는 대표적인 항목이다.

## 16.4 원격 관리: Git, SSH, 파일 전송

연구실 서버에 SSH로 접속해서 실험을 돌리고, 코드는 Git으로 관리하고, 데이터는 서버 간에 주고받는 것이 일상이다. 여기 나오는 도구들을 익혀 두면 이 과정이 매끄러워진다.

## 16.4.1 Git/GitHub

### 16.4.1.1 기본 워크플로우

코드를 관리하지 않으면, "어제 돌아가던 코드가 오늘은 왜 안 되지?"라는 상황이 반복된다. Git은 모든 변경 이력을 추적해 주므로, 언제든 과거 상태로 돌아갈 수 있다. 연구 코드라도 Git으로 관리하는 습관을 들이자.

```bash
# 저장소 클론
git clone https://github.com/user/repo.git

# 변경 사항 확인
git status
git diff

# 커밋
git add .
git commit -m "feat: add new feature"

# 푸시
git push origin main
```

> **추천 자료**
> - [GitHub's Git Handbook](https://docs.github.com/en/get-started/using-git/about-git) — Git 핵심 개념을 깔끔하게 정리한 공식 가이드
> - [The Missing Semester - Version Control (Git)](https://missing.csail.mit.edu/2020/version-control/) — MIT 강의. Git의 내부 모델(DAG)까지 설명해 줘서 깊이 있게 이해 가능

### 16.4.1.2 브랜치 전략

**Git Flow**:
- `main`: 안정 버전
- `develop`: 개발 버전
- `feature/*`: 기능 개발
- `hotfix/*`: 긴급 수정

**GitHub Flow** (간단):
- `main`: 항상 배포 가능
- `feature-branch`: 기능별 브랜치 → PR → Merge

연구실에서 여러 사람이 같은 코드를 수정할 때 브랜치 전략 없이 `main`에 직접 push하면 충돌이 끊이질 않는다. 연구 코드라면 GitHub Flow 정도면 충분하다. 기능 하나당 브랜치 하나를 만들고, PR을 통해 머지하는 습관을 들이자.

### 16.4.1.3 협업

**Pull Request (PR)**:
1. Fork 또는 브랜치 생성
2. 변경 사항 커밋
3. PR 생성 및 리뷰 요청
4. 코드 리뷰 후 머지

**커밋 메시지 규칙** (Conventional Commits):

```
feat: 새로운 기능
fix: 버그 수정
docs: 문서 변경
refactor: 리팩토링
test: 테스트 추가/수정
chore: 빌드/설정 변경
```

> **추천 자료**
> - [GitHub's Git Handbook](https://docs.github.com/en/get-started/using-git/about-git) — GitHub에서 직접 만든 Git 입문 가이드
> - [Conventional Commits Specification](https://www.conventionalcommits.org/) — 커밋 메시지 규칙 공식 스펙

## 16.4.2 SSH

연구실 GPU 서버에 접속하는 기본 도구이다. 비밀번호 대신 키 인증을 쓰면 편하고 안전하다.

```bash
# 키 생성 (처음 한 번)
ssh-keygen -t ed25519

# 서버에 공개키 등록
ssh-copy-id user@server_ip

# 접속
ssh user@server_ip

# 포트 포워딩 (서버의 Jupyter/TensorBoard를 로컬에서 보기)
ssh -L 8888:localhost:8888 user@server_ip
```

**~/.ssh/config**를 설정해 두면 매번 IP와 사용자명을 입력하지 않아도 된다:

```
Host lab-server
    HostName 192.168.1.100
    User junholee
    IdentityFile ~/.ssh/id_ed25519
```

이후 `ssh lab-server`로 바로 접속 가능. VS Code Remote-SSH도 이 설정을 읽는다.

## 16.4.3 SCP & rsync

서버와 로컬 간 파일 전송 도구다. **SCP**는 단순 파일 복사, **rsync**는 변경된 부분만 전송한다.

**SCP**:

```bash
# 로컬 → 서버
scp model.pth user@server:/home/user/weights/

# 서버 → 로컬
scp user@server:/home/user/results/log.txt ./

# 디렉토리 복사
scp -r dataset/ user@server:/data/
```

**rsync** — 대용량 데이터셋이나 반복 전송에 유리하다:

```bash
# 로컬 → 서버 (변경분만 전송, 진행률 표시)
rsync -avz --progress dataset/ user@server:/data/dataset/

# 서버 → 로컬
rsync -avz user@server:/home/user/results/ ./results/

# 삭제된 파일도 동기화 (미러링)
rsync -avz --delete source/ user@server:/data/source/
```

`scp`는 매번 전체를 복사하지만, `rsync`는 diff만 보내므로 수십 GB 데이터셋을 동기화할 때 차이가 크다.

## 16.4.4 Tailscale

연구실 서버가 NAT/방화벽 뒤에 있으면 외부에서 SSH 접속이 안 된다. Tailscale은 WireGuard 기반 VPN으로, 설치만 하면 어디서든 연구실 서버에 직접 접속할 수 있게 해 준다.

```bash
# 설치 (서버와 로컬 양쪽에)
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up

# 상태 확인 — 연결된 기기 목록과 IP 확인
tailscale status

# 이후 Tailscale IP로 SSH
ssh user@100.x.y.z
```

**장점**:
- 포트 포워딩이나 공유기 설정이 필요 없다
- 카페, 집, 학교 어디서든 같은 IP로 서버에 접속
- Tailscale SSH를 쓰면 SSH 키 관리도 자동화 가능

**~/.ssh/config**에 Tailscale IP를 등록해 두면 편하다:

```
Host lab-gpu
    HostName 100.x.y.z
    User junholee
```

> **추천 자료**
> - [Tailscale 공식 문서](https://tailscale.com/kb/) — 설치부터 ACL 설정까지
> - [The Missing Semester - Remote Machines](https://missing.csail.mit.edu/2020/command-line/#remote-machines) — SSH, 포트 포워딩, tmux 등 원격 작업 기초

## 16.5 실험 관리

## 16.5.1 Weights & Biases (wandb)

딥러닝 실험을 하다 보면 "어제 돌린 모델의 하이퍼파라미터가 뭐였지?"라는 상황이 매일 발생한다. 실험 관리 도구 없이 엑셀이나 노트로 기록하면 금방 한계에 부딪힌다. wandb는 학습 과정을 자동으로 로깅하고 시각화해 주며, 팀원과 결과를 공유하기도 쉽다.

**특징**:
- 실험 로깅 및 시각화
- 하이퍼파라미터 추적
- 모델 버전 관리
- 팀 협업

```python
import wandb

# 초기화
wandb.init(project="my-project", config={
    "learning_rate": 0.001,
    "epochs": 100
})

# 로깅
for epoch in range(epochs):
    loss = train_one_epoch()
    wandb.log({"loss": loss, "epoch": epoch})

# 완료
wandb.finish()
```

> **추천 자료**
> - [Weights & Biases 공식 문서 및 Quickstart](https://docs.wandb.ai/quickstart) — wandb 시작 가이드. 5분이면 첫 실험 로깅 가능
> - [Weights & Biases YouTube](https://www.youtube.com/@WeightsBiases) — 사용법 튜토리얼 및 MLOps 관련 강연

## 16.5.2 MLflow

wandb가 클라우드 기반 서비스라면, MLflow는 자체 서버에서 운영할 수 있는 오픈소스 대안이다. 데이터 보안이 중요한 환경에서 유용하다.

```python
import mlflow

mlflow.set_experiment("my-experiment")

with mlflow.start_run():
    mlflow.log_param("lr", 0.001)
    mlflow.log_metric("accuracy", 0.95)
    mlflow.pytorch.log_model(model, "model")
```

## 16.5.3 TensorBoard

```python
from torch.utils.tensorboard import SummaryWriter

writer = SummaryWriter("runs/experiment1")
writer.add_scalar("Loss/train", loss, epoch)
writer.add_image("Sample", image, epoch)
writer.close()
```

```bash
tensorboard --logdir runs
```

TensorBoard는 PyTorch에서도 바로 쓸 수 있고, 별도의 계정 생성 없이 로컬에서 바로 시각화가 가능하다는 장점이 있다. 간단한 실험이라면 wandb 대신 TensorBoard만으로도 충분하다.

> **추천 자료**
> - [PyTorch TensorBoard Tutorial](https://pytorch.org/tutorials/recipes/recipes/tensorboard_with_pytorch.html) — PyTorch에서 TensorBoard 사용하는 공식 가이드
> - [MLflow Documentation](https://mlflow.org/docs/latest/index.html) — MLflow 공식 문서

## 16.6 코드 포매팅

## 16.6.1 Linting & Formatting

코드 스타일이 사람마다 다르면 코드 리뷰에서 로직보다 스타일 논쟁에 시간을 더 쓰게 된다. 자동 포매터를 쓰면 이 문제가 사라진다. 혼자 연구할 때도 일관된 코드 스타일은 나중에 자기 코드를 다시 읽을 때 큰 도움이 된다.

**Python**:

```bash
# Ruff (빠른 린터, Black 호환 포맷터)
pip install ruff
ruff check .
ruff format .

# Black (포매터)
pip install black
black .

# 타입 체크
pip install mypy
mypy .
```

**C++**:

```bash
# clang-format
clang-format -i src/*.cpp
```

## 16.6.2 Testing

코드에 테스트를 작성하는 것은 연구 코드에서도 중요하다. "모델 forward pass가 제대로 되는지", "데이터 전처리 결과가 예상과 같은지" 같은 기본적인 테스트만 있어도 리팩토링할 때 훨씬 안심이 된다.

**Python (pytest)**:

```python
# test_module.py
def test_addition():
    assert 1 + 1 == 2

def test_function():
    result = my_function(input)
    assert result == expected
```

```bash
pytest tests/ -v
```

**C++ (gtest)**:

```cpp
#include <gtest/gtest.h>

TEST(MyTest, BasicTest) {
    EXPECT_EQ(1 + 1, 2);
}
```

> **추천 자료**
> - [Real Python - Python Testing with pytest](https://realpython.com/pytest-python-testing/) — pytest 사용법 상세 튜토리얼
> - [The Missing Semester (MIT)](https://missing.csail.mit.edu/) — 셸, 에디터, 디버깅, 프로파일링 등 개발 도구 전반. 연구실 입학 전에 한 번 쭉 보면 좋다
> - [Fireship YouTube](https://www.youtube.com/@Fireship) — 각종 개발 도구를 "100 Seconds" 시리즈로 빠르게 훑어볼 수 있다
> - [정진용 블로그 — 로봇 소프트웨어 개발 문화](https://jinyongjeong.github.io/2025/02/14/developmen_culture/) — 코드 리뷰, CI/CD, 스타일 가이드 등 로봇 개발팀의 개발 문화 정착 방법
> - [정진용 블로그 — 로봇 개발과 테스트 코드](https://jinyongjeong.github.io/2025/02/19/test_code/) — 로봇 소프트웨어에서 테스트 코드가 필수인 6가지 이유

## 기술 흐름: 개발 환경 & 도구의 과거 → 현재 → 미래

```
2005 ─── Git 탄생 (Linus Torvalds)
  │       분산 버전 관리의 시작
  │
2008 ─── GitHub 출시
  │       오픈소스 협업의 중심지가 됨
  │
2010 ─── Conda (Anaconda) 등장
  │       Python 환경 관리 표준으로 자리 잡음
  │
2013 ─── Docker 공개
  │       컨테이너 기반 가상화로 재현성 문제 해결
  │
2015 ─── TensorBoard (TensorFlow와 함께 공개)
  │       딥러닝 학습 시각화의 시작
  │
2017 ─── nvidia-docker 공개
  │       Docker 컨테이너에서 GPU 사용 가능
  │
2018 ─── Weights & Biases 출시
  │       실험 추적·시각화·팀 협업을 클라우드로
  │
2020 ─── Ruff 등장 (2022), Black 대중화
  │       Python 코드 품질 도구의 고속화
  │
2023+ ── AI-assisted 개발 도구 확산
          Copilot, Cursor 등 AI 코딩 보조 도구
          Dev Container 표준화 (VS Code Remote)
          재현 가능한 연구를 위한 Docker + wandb 조합 보편화
```
