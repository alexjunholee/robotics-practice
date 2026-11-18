# 16장 개발 환경과 도구

로보틱스 소프트웨어는 CUDA, Python, ROS, 시스템 라이브러리의 버전 조합에 유난히 민감하다. 논문 한 편의 코드를 돌려 보려다 시스템 전역 라이브러리를 건드려 기존 ROS 워크스페이스까지 망가지는 일도 흔하다. 그래서 본격적인 실험에 들어가기 전에 언어와 OS 조합을 정하고, 프로젝트마다 환경을 격리해 다른 컴퓨터에서도 그대로 재현할 수 있는 도구부터 갖춘다.

## 16.1 프로그래밍 언어

AI 코딩 에이전트가 보일러플레이트나 스크립트 작성을 상당 부분 대신해 주는 지금도, 코드를 직접 읽고 이해하는 능력은 여전히 중요하다. 다른 연구자의 코드를 클론해 구조를 파악하고, 에이전트가 짠 코드에 미묘한 버그가 없는지 검증하며, 로봇이 멈췄을 때 어느 지점을 고쳐야 할지 판단하는 일은 결국 사람의 몫이기 때문이다. 로보틱스 현장에서는 실행 성능이 필요한 코어에 C++를, 실험과 학습 파이프라인에 Python을 나누어 쓴다.

### 16.1.1 C++

실시간 제어 루프, ROS 노드, SLAM, 고빈도 센서 처리처럼 지연 시간에 민감한 모듈은 C++로 작성한다. 연구실에서 자주 마주치는 ORB-SLAM3, LOAM, VINS-Mono 같은 핵심 오픈소스 패키지도 대부분 C++ 기반이므로, 직접 밑바닥부터 짜지 않더라도 구조를 읽고 수정할 수 있을 만큼은 익숙해져야 한다(1장 §1.5).

C++는 실행 속도가 빠르고 메모리를 직접 제어할 수 있어 실시간 시스템에 적합하지만, 문법이 까다롭고 컴파일·디버깅 주기가 길며 포인터 실수로 메모리 오류가 나기 쉽다는 부담이 있다. 다행히 최근 로봇 오픈소스 코드는 과거 C 스타일의 날것 포인터 대신 스마트 포인터, 범위 기반 반복문, 람다 표현식을 쓰는 모던 C++(C++17/20) 관용구를 널리 채택해 가독성과 안전성이 한결 나아졌다. 실제 코드에서 가장 자주 만나는 세 가지 문법은 다음과 같다.

```cpp
// 스마트 포인터
auto ptr = std::make_shared<MyClass>();

// Range-based for
for (const auto& item : container) { ... }

// Lambda
auto func = [&](int x) { return x * 2; };
```

`std::make_shared`로 수명을 자동 관리해 메모리 누수를 줄이고, `const auto&` 범위 반복문으로 불필요한 복사를 막으며, 콜백이나 정렬 기준을 넘길 때 간결한 람다 함수를 활용하는 식이다. 모던 C++ 문법과 안전한 설계 패턴을 익힐 때는 아래 자료들이 기준점 역할을 한다.

> **추천 자료**
> - [C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines) — 모던 C++ 코딩 가이드 (Bjarne Stroustrup, Herb Sutter)
> - [The Cherno - C++ Playlist](https://www.youtube.com/playlist?list=PLlrATfBNZ98dudnM48yfGUldqGD0S4FFb) — C++ 기초부터 고급까지 영상 시리즈
> - [Modernes C++](https://www.modernescpp.com/index.php) — 모던 C++ (C++17/20/23) 기능을 체계적으로 정리한 블로그

> **⚠ 타겟 환경 점검**: C++ 코드를 요청할 때 x86인지 Jetson(ARM)인지, 크로스 컴파일을 쓰는지 함께 적는다. 생성된 의존성과 build flag가 타겟 아키텍처를 지원하는지도 확인한다.

### 16.1.2 Python

C++가 로봇의 실시간 심장을 담당한다면, Python은 빠른 프로토타이핑, 딥러닝 학습과 추론, 데이터 분석 및 시각화를 맡는다. PyTorch 학습 스크립트나 데이터 전처리 파이프라인을 짤 때 특히 생산성이 높다. 코딩 에이전트로 초안을 빠르게 뽑아내는 데도 잘 어울리지만, 생성된 코드의 텐서 차원과 실행 결과, 데이터 로딩 병목은 직접 확인하고 짚어낼 수 있어야 한다.

로보틱스와 비전 연구에서 거의 매번 설치하게 되는 핵심 패키지는 수치 연산, 영상·3D 처리, 딥러닝 스택이다.

```bash
pip install numpy scipy matplotlib
pip install opencv-python open3d
pip install torch torchvision
pip install transformers  # HuggingFace
```

문법 자체는 간결하지만 파이썬 특유의 제너레이터, 데코레이터, 벡터화 연산을 익혀 두면 연구 코드가 훨씬 깔끔해진다.

> **추천 자료**
> - [Real Python](https://realpython.com/) — Python 기초부터 고급까지 체계적 튜토리얼
> - [Fireship - Python in 100 Seconds](https://www.youtube.com/watch?v=x7X9w_GIm1s) — Python 전체를 빠르게 훑어보는 영상

## 16.2 개발 환경 설정

사용할 언어를 정했다면 이제 운영체제와 GPU 드라이버, 패키지 환경을 깔 차례다. 로보틱스 개발에서는 이 바닥층의 버전 궁합을 맞추는 일이 전체 삽질의 절반을 차지한다.

### 16.2.1 Ubuntu

Ubuntu는 ROS의 1차 지원 플랫폼이며 NVIDIA 드라이버, CUDA, cuDNN 조합에 관한 문서와 트러블슈팅 사례가 가장 풍부하다. macOS나 Windows에서도 Docker나 가상머신으로 일부 작업을 돌릴 수 있지만, 실제 로봇과 센서 드라이버까지 매끄럽게 붙이려면 사용할 ROS2 배포판과 짝이 맞는 Ubuntu LTS 버전을 기준으로 삼아야 한다.

- Ubuntu 22.04 LTS (ROS2 Humble)
- Ubuntu 24.04 LTS (ROS2 Jazzy)
- Ubuntu 26.04 LTS (ROS2 Lyrical, 2026년 5월 출시)

새 Ubuntu 머신을 받으면 컴파일러, 빌드 도구, 파이썬 가상환경, 터미널 유틸리티부터 먼저 설치해 둔다.

```bash
# 기본 도구
sudo apt update && sudo apt upgrade -y
sudo apt install -y build-essential cmake git curl wget

# Python 관련
sudo apt install -y python3-pip python3-venv

# 개발 도구
sudo apt install -y vim tmux htop
```

특히 원격 서버에 자주 붙는다면 창을 쪼개 쓰고 세션이 끊겨도 작업을 유지해 주는 `tmux`와 자원 점유율을 보는 `htop`을 초반에 손에 익혀 두면 작업 효율이 크게 오른다.

> **추천 자료**
> - [The Missing Semester of Your CS Education (MIT)](https://missing.csail.mit.edu/) — 셸, vim, tmux, Git 등 "수업에서는 안 가르치지만 매일 쓰는" 개발 도구를 체계적으로 정리한 MIT 강의
> - [Fireship - Linux in 100 Seconds](https://www.youtube.com/watch?v=rrB13utjYV4) — Linux가 뭔지 빠르게 감 잡기

### 16.2.2 CUDA / cuDNN

딥러닝 모델을 학습시키려면 GPU 가속이 필수다. 이때 많은 초보자가 호스트의 CUDA 버전과 PyTorch 안의 CUDA 버전이 똑같아야 한다고 오해하지만, 핵심은 설치된 NVIDIA GPU 드라이버가 PyTorch에 내장된 CUDA 런타임을 지원하느냐에 있다. 먼저 현재 머신의 드라이버와 CUDA toolkit 상태를 아래 명령으로 확인한다.

```bash
nvidia-smi          # GPU 상태
nvcc --version      # CUDA 버전
```

설치할 CUDA·cuDNN 조합은 쓰려는 PyTorch나 TensorFlow 릴리스의 공식 지원 표를 기준으로 고른다.

**NVIDIA 드라이버 설치 트러블슈팅**

새로 깐 Ubuntu에 NVIDIA 드라이버를 올릴 때 가장 흔하게 부딪히는 장애물은 리눅스 커널의 기본 오픈소스 드라이버인 `nouveau`와의 충돌이다. 드라이버를 설치하기 전에 `nouveau`를 먼저 비활성화하고 재부팅한 뒤 `apt`로 드라이버를 설치하면 충돌을 피할 수 있다.

```bash
# nouveau 비활성화
sudo bash -c "echo blacklist nouveau > /etc/modprobe.d/blacklist-nvidia-nouveau.conf"
sudo bash -c "echo options nouveau modeset=0 >> /etc/modprobe.d/blacklist-nvidia-nouveau.conf"
sudo update-initramfs -u
sudo reboot

# 드라이버 설치 (권장: apt 사용)
sudo apt install nvidia-driver-535  # 버전은 GPU에 맞게 조정
sudo reboot
```

재부팅 뒤에는 `nvidia-smi`를 실행해 GPU 모델명과 드라이버 버전이 정상 출력되는지 확인한다.

> **추천 자료**
> - [PyTorch - Previous Versions](https://pytorch.org/get-started/previous-versions/) — PyTorch와 CUDA 버전 매칭 확인. 설치 전 확인할 것
> - [NVIDIA CUDA Toolkit Documentation](https://docs.nvidia.com/cuda/) — CUDA 공식 문서

```bash
# 확인
nvidia-smi
```

설치 후에도 `nvidia-smi`에서 GPU가 잡히지 않는다면 두 가지를 점검한다. 첫째, `sudo dkms status`로 커널 모듈이 제대로 빌드·설치됐는지와 `lsmod | grep nvidia`로 실제 커널에 올라왔는지를 본다. 둘째, 바이오스의 Secure Boot가 켜져 있으면 서명되지 않은 커널 모듈 로드가 차단되므로 MOK 키를 등록하거나 배포판이 서명한 드라이버 패키지를 쓰고, 여의치 않으면 Secure Boot를 끈다. (참고: [정진용 블로그](https://jinyongjeong.github.io/2016/11/22/ubuntu_graphic_driver_install/))

**CUDA/cuDNN 버전 호환성**

PyTorch 공식 wheel은 자체 CUDA 런타임을 안에 품고 배포되므로, 호스트에 설치된 CUDA toolkit(`nvcc`) 버전과 꼭 일치하지 않아도 돌아간다. 반면 NVIDIA 드라이버가 PyTorch의 CUDA 런타임보다 구버전이면 `import torch` 자체는 성공해도 `torch.cuda.is_available()`이 `False`를 반환하거나 첫 CUDA 호출 순간 오류를 낸다. 호환성 점검 순서는 다음과 같다.

```bash
# 1. GPU 확인
nvidia-smi  # 오른쪽 상단의 CUDA Version은 "드라이버가 지원하는 최대 버전"

# 2. 설치된 CUDA toolkit 확인
nvcc --version

# 3. PyTorch가 사용하는 CUDA 확인
python -c "import torch; print(torch.version.cuda)"

# wheel을 쓰면 조건은 하나다: 드라이버(1)가 PyTorch의 CUDA 런타임(3)을 지원하는가.
# nvcc(2)는 소스 빌드나 커스텀 CUDA 확장을 컴파일할 때만 관여한다. 조합은 PyTorch 공식 사이트에서 확인:
# https://pytorch.org/get-started/locally/
```

요컨대 `nvidia-smi` 우측 상단의 숫자가 `torch.version.cuda`보다 크거나 같으면 wheel 기반 학습 코드는 정상 실행되며, `nvcc`는 직접 C++/CUDA 확장을 컴파일할 때만 신경 쓰면 된다. (참고: [정진용 블로그](https://jinyongjeong.github.io/2016/09/19/cuda_setting/))

**OpenCV + CUDA 직접 빌드**

`apt`로 받는 `python3-opencv`나 `pip install opencv-python` 패키지는 범용 호환성을 위해 CUDA 가속이 빠져 있다. GPU 기반 optical flow나 OpenCV DNN 모듈 가속이 꼭 필요하다면 `opencv`와 `opencv_contrib` 소스를 받아 직접 빌드해야 한다.

```bash
# 의존성 설치
sudo apt install -y build-essential cmake git libgtk2.0-dev pkg-config \
    libavcodec-dev libavformat-dev libswscale-dev \
    libtbb-dev libjpeg-dev libpng-dev \
    python3-dev python3-numpy   # BUILD_opencv_python3=ON에 필요. cmake 요약의 Python 3 항목을 확인한다

# OpenCV + contrib 소스
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git

# 빌드 (CUDA 활성화)
cd opencv && mkdir build && cd build
# CUDA_ARCH_BIN은 GPU에 맞게 조정 (RTX 3090=8.6, RTX 4090=8.9). 줄 연속(\) 뒤에는 주석을 둘 수 없다
cmake -D CMAKE_BUILD_TYPE=Release \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
      -D WITH_CUDA=ON \
      -D CUDA_ARCH_BIN="8.6" \
      -D WITH_CUDNN=ON \
      -D OPENCV_DNN_CUDA=ON \
      -D BUILD_opencv_python3=ON \
      ..
make -j$(nproc)
sudo make install
```

빌드할 때 `CUDA_ARCH_BIN` 값은 자기 GPU 아키텍처에 맞게 정확히 적어야 한다. 번호가 어긋나면 컴파일은 통과하더라도 실행 시 속도가 급락하거나 런타임 에러가 발생한다. 정확한 번호는 [NVIDIA GPU Compute Capability](https://developer.nvidia.com/cuda/gpus)에서 확인한다.

한 가지 더 주의할 점이 있다. ROS 환경에서는 `pip`로 설치한 OpenCV와 `apt` 기반 `cv_bridge`가 공유 라이브러리 충돌을 일으키는 문제가 잦다(→ 부록 C.4). CUDA OpenCV까지 호스트에 직접 빌드해 올리면 시스템 경로가 한층 더 복잡해지므로, 호스트 환경을 깔끔하게 지키려면 16.3절의 Docker 컨테이너 안에 격리해 빌드하는 편이 안전하다.

(참고: [다크 프로그래머 — OpenCV + CUDA 직접 빌드하기](https://darkpgmr.tistory.com/184))

### 16.2.3 환경 관리

프로젝트마다 요구하는 Python 버전과 라이브러리 버전이 제각각이다. 환경 분리 없이 전역에 `pip install`을 반복하다 보면 프로젝트 A를 살리려다 프로젝트 B의 패키지 호환성이 깨지는 "의존성 지옥(dependency hell)"에 빠진다. 이를 막기 위해 프로젝트 단위로 독립된 가상환경을 만든다.

파이썬 버전 자체를 프로젝트마다 다르게 써야 하거나 비파이썬 바이너리 의존성까지 깔끔히 묶고 싶다면 **Conda**가 편리하다.

```bash
# Miniconda 설치
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh
# 설치 마지막에 conda init을 허용했다면 셸을 다시 연다 (그래야 conda activate가 동작한다)

# 환경 생성
conda create -n myenv python=3.10
conda activate myenv

# 패키지 설치
# PyTorch는 2.6부터 pytorch Anaconda 채널 게시를 중단했다. conda 환경 안에서 pip 인덱스를 쓴다
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu126
```

별도 도구 설치 없이 가볍게 패키지 목록만 격리하고 싶다면 파이썬 표준 라이브러리에 들어 있는 **venv**만으로 충분하다.

```bash
python3 -m venv myenv
source myenv/bin/activate
pip install -r requirements.txt
```

가령 구형 SLAM 연구 코드에는 Python 3.8이, 최신 트랜스포머 모델 코드에는 Python 3.10이 필요하다면 `conda activate slam_env`와 `conda activate transformer_env`를 오가며 충돌 없이 두 프로젝트를 동시에 진행한다.

> **추천 자료**
> - [Conda Documentation - Managing Environments](https://docs.conda.io/projects/conda/en/latest/user-guide/tasks/manage-environments.html) — Conda 환경 관리 공식 가이드
> - [Python venv Documentation](https://docs.python.org/3/library/venv.html) — 파이썬 공식 가상환경 문서

## 16.3 Docker

가상환경이 파이썬 패키지 충돌을 막아 준다면, Docker는 그 아래의 리눅스 시스템 라이브러리와 ROS 배포판, CUDA 버전까지 통째로 격리해 주는 한 단계 더 큰 울타리다.

### 16.3.1 왜 Docker인가?

로보틱스 소프트웨어는 Python 패키지뿐 아니라 Ubuntu 시스템 라이브러리, C++ 빌드 도구, ROS 배포판, CUDA 런타임이 복잡하게 얽혀 있다. Conda나 venv만으로는 호스트 OS의 `apt` 패키지나 C++ 공유 라이브러리 충돌까지 막아 주지 못한다. Docker는 OS 사용자 공간, 시스템 라이브러리, 환경 설정을 하나의 이미지로 묶어 배포하므로, 같은 이미지를 쓰는 팀원이나 연구실 서버 어디에서나 동일한 실행 환경을 그대로 재현할 수 있고 논문 코드의 실행 환경을 통째로 아카이빙하기도 좋다.

### 16.3.2 기본 사용법

아래 명령과 예시는 CUDA 12.1과 Ubuntu 22.04 조합(`nvidia/cuda:12.1.0-devel-ubuntu22.04`)을 기준으로 작성했다. 다른 CUDA나 Ubuntu 버전을 쓰고 싶다면 이미지 태그만 바꾸면 된다.

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

`docker run`에서 `--rm`은 셸을 빠져나올 때 컨테이너를 자동 삭제해 찌꺼기를 남기지 않게 하고, `-v $(pwd):/workspace`는 호스트의 현재 작업 디렉터리를 컨테이너 내부 `/workspace`와 실시간으로 연결한다. 매번 컨테이너를 띄운 뒤 수동으로 패키지를 설치하지 않으려면 아래처럼 `Dockerfile`에 설치 절차를 적어 이미지로 굳혀 둔다.

**Dockerfile 예시**:

```dockerfile
FROM nvidia/cuda:12.1.0-devel-ubuntu22.04

RUN apt-get update && apt-get install -y \
    python3-pip git

COPY requirements.txt /tmp/
RUN pip3 install -r /tmp/requirements.txt

WORKDIR /workspace
```

이렇게 `Dockerfile`로 빌드 과정을 코드화해 두면 몇 달 뒤 새 서버로 옮기더라도 `docker build` 한 번으로 똑같은 환경을 복원할 수 있다.

> **추천 자료**
> - [Docker 공식 Getting Started Guide](https://docs.docker.com/get-started/) — Docker 처음이라면 여기부터. 컨테이너, 이미지, 볼륨 개념을 잘 설명
> - [NetworkChuck - Docker Tutorial](https://www.youtube.com/watch?v=eGz9DS-aIeY) — Docker를 재미있고 쉽게 설명하는 영상. 입문용으로 적합
> - [Fireship - Docker in 100 Seconds](https://www.youtube.com/watch?v=Gjnup-PuquQ) — Docker 핵심 개념을 빠르게 훑어보기

### 16.3.3 NVIDIA Container Toolkit

기본 Docker 컨테이너는 격리된 공간이므로 호스트의 GPU 장치를 바로 보지 못한다. 컨테이너 안에서 CUDA와 PyTorch를 돌리려면 호스트에 `nvidia-container-toolkit`을 설치해 Docker 데몬에 GPU 연결 기능을 등록해야 한다. 참고로 Docker 19.03부터는 예전의 `--runtime=nvidia` 옵션 대신 `--gpus all` 플래그로 GPU 접근을 요청한다.

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

마지막 테스트 명령에서 호스트와 똑같은 `nvidia-smi` 표가 출력되면 컨테이너 안에서 GPU를 쓸 준비가 끝난다.

### 16.3.4 실전 레시피: ROS2 + GPU + GUI + 센서

로보틱스 개발용 컨테이너에는 딥러닝 학습용 GPU, 시각화 창(RViz/Gazebo)을 띄우는 GUI, 그리고 USB 카메라나 LiDAR 같은 물리 센서 접근 권한을 한꺼번에 물려야 하는 상황이 잦다. 아래 레시피는 [turlucode/ros-docker-gui](https://github.com/turlucode/ros-docker-gui)의 구성을 바탕으로 `nvidia-container-toolkit`과 ROS2 Humble(Ubuntu 22.04 기반 `osrf/ros:humble-desktop` 이미지)에 맞춘 설정이다. Jazzy 등 다른 ROS2 배포판을 쓰려면 이미지 태그와 `humble`이 들어간 패키지 이름·경로를 함께 맞추면 된다.

**1단계: X11 포워딩 준비 (호스트)**

컨테이너 안의 RViz나 Gazebo 창을 호스트 모니터에 띄우려면 먼저 호스트 X 서버가 컨테이너의 화면 출력 요청을 받아 주도록 허용해야 한다.

```bash
# 호스트에서 한 번만 실행
sudo apt-get install -y xauth
xhost +local:docker
```

`xhost +local:docker`는 로컬 연결을 허용하는 접근 규칙을 추가한다. `xhost`는 호스트 단위 접근 제어이므로 이 설정을 켜면 Docker 컨테이너를 포함한 모든 로컬 연결에 X 서버가 열린다. 특정 컨테이너에만 권한을 제한하고 싶다면 `xauth` 쿠키 기반 인증을 사용한다.

**2단계: 실행 스크립트**

X11 준비가 끝났다면 GPU, GUI 소켓, USB 디바이스, ROS2 워크스페이스 볼륨을 한 번에 물려 컨테이너를 실행하는 스크립트를 만든다.

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
    -v $HOME/ros2_ws:/root/ros2_ws \
    --name ros2_dev \
    osrf/ros:humble-desktop \
    bash
```

옵션이 많아 보이지만 각 플래그가 맡은 역할은 분명하다.

| 플래그 | 역할 |
|--------|------|
| `--gpus all` | GPU 패스스루 (nvidia-container-toolkit) |
| `--privileged` | USB/시리얼 디바이스 전체 접근. 프로덕션에서는 `--device`로 개별 매핑할 것 |
| `--net=host` | DDS multicast 설정을 간단히 하려고 호스트 네트워크 공유. 통신 구성에 따라 선택 |
| `--ipc=host` | 공유 메모리. RViz 등 GUI 도구에서 필요 |
| `-e QT_X11_NO_MITSHM=1` | MIT-SHM 호환성 문제로 RViz 등이 종료될 때 사용하는 우회 설정 |
| `-e ROS_DOMAIN_ID=42` | 같은 네트워크의 다른 ROS2 시스템과 격리. 여러 시스템이 함께 있을 때 도메인 값을 분리 |
| `-v /dev:/dev` | 센서 USB가 언제 꽂힐지 모르므로 /dev 전체 마운트. `--privileged`와 세트 |
| `-v .Xauthority` | X11 인증. 1단계의 xhost 설정을 쓰지 않을 때 이것이 없으면 `cannot open display` |

실제 현장에 배포할 때는 보안과 격리 범위를 좁히기 위해 `--privileged`와 `/dev` 전체 마운트 대신 `--device=/dev/ttyUSB0`처럼 필요한 장치만 골라 넘기는 쪽으로 다듬는다.

**3단계: 작업 후 컨테이너 저장**

2단계를 `--rm` 옵션으로 띄우면 `exit`로 셸을 나가는 순간 컨테이너 안의 변경 사항이 사라진다. 컨테이너 안에서 새 패키지를 설치하고 그대로 보존하고 싶다면, 컨테이너가 살아 있는 동안 호스트의 다른 터미널에서 `commit`으로 이미지를 떠 두거나 애초에 `--rm`을 빼고 실행한다.

```bash
# 컨테이너 안에서 패키지 설치 등 작업을 했으면, 컨테이너를 띄운 채 호스트의 다른 셸에서 커밋
docker commit ros2_dev my_ros2_workspace:v1

# 다음번에는 저장된 이미지로 실행
# run_ros2_docker.sh에서 이미지 이름만 바꾸면 된다
```

**Dockerfile로 관리하는 방식** (더 권장):

임시로 `docker commit`을 쓸 수는 있지만, 팀과 공유하거나 몇 달 뒤 환경을 다시 손볼 때는 설치 과정을 처음부터 `Dockerfile`에 적어 두는 편이 훨씬 깔끔하다.

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

`docker commit`으로 만든 이미지에도 레이어 히스토리는 남지만, 정확히 어떤 명령과 옵션으로 깔았는지 사람이 읽을 수 있는 명세서가 남지 않아 나중에 재생성하거나 일부만 고치기가 까다롭다. `Dockerfile`은 환경 그 자체가 곧 문서가 된다.

> **추천 자료**
> - [turlucode/ros-docker-gui](https://github.com/turlucode/ros-docker-gui) — ROS + NVIDIA + GUI Docker 설정 레퍼런스. 현재 README는 ROS 1 Noetic과 ROS 2 Jazzy 등을 예로 든다
> - [NVIDIA Container Toolkit Documentation](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/index.html) — 공식 설치 가이드
> - [OSRF Docker Images](https://hub.docker.com/r/osrf/ros) — ROS 공식 Docker 이미지. `humble-desktop`이 GUI 포함 버전

> **Docker 요구사항 점검**: 설정을 요청할 때 ROS2, GPU, USB 센서, GUI 시각화를 함께 쓸지 한 번에 적는다. 따로 만든 설정을 합치면 권한과 네트워크 옵션이 충돌할 수 있다. 완성된 명령에는 `QT_X11_NO_MITSHM`, `ROS_DOMAIN_ID`, device mapping이 필요한지 확인한다.

## 16.4 원격 관리: Git, SSH, 파일 전송

로보틱스 실험은 내 노트북 한 대 안에서 끝나지 않는다. 모델 학습은 연구실 GPU 서버에서 돌리고, 로봇 제어 코드는 탑재된 온보드 PC로 옮겨 실행하며, 수정된 코드는 팀원들과 주고받아야 한다. 이 원격 작업 흐름을 지탱하는 도구가 Git, SSH, 파일 전송 도구, 그리고 Tailscale이다.

### 16.4.1 Git/GitHub

### 16.4.1.1 기본 워크플로우

Git은 코드 변경 이력과 실험 시점의 스냅샷을 남긴다. 실험 결과 로그에 당시 실행한 commit 해시를 함께 적어 두면, 며칠 뒤 성능이 갑자기 떨어졌을 때 어느 변경에서 문제가 시작됐는지 즉시 대조하고 이전 상태로 되돌릴 수 있다. 매일 반복하는 기본 흐름은 클론, 상태·차이 확인, 스테이징·커밋, 푸시다.

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

변경 사항을 `git add .`로 한꺼번에 올리기 전에 `git status`와 `git diff`로 불필요한 대용량 데이터나 임시 파일이 섞이지 않았는지 훑어보는 습관이 중요하다.

> **추천 자료**
> - [GitHub's Git Handbook](https://docs.github.com/en/get-started/using-git/about-git) — Git 핵심 개념을 깔끔하게 정리한 공식 가이드
> - [The Missing Semester - Version Control (Git)](https://missing.csail.mit.edu/2020/version-control/) — MIT 강의. Git의 내부 모델(DAG)까지 설명해 줘서 깊이 있게 이해 가능

### 16.4.1.2 브랜치 전략

혼자 실험할 때는 `main` 브랜치 하나만 써도 버틸 수 있지만, 여러 사람이 같은 저장소를 건드리기 시작하면 각자의 실험과 안정 버전이 뒤엉키지 않게 브랜치를 나누어야 한다.

규모가 큰 소프트웨어 배포에서는 릴리스 주기에 맞춰 역할을 엄격히 나눈 **Git Flow**를 쓰기도 한다. 안정 버전을 담는 `main`, 다음 릴리스를 통합하는 `develop`, 기능 개발용 `feature/*`, 긴급 수정용 `hotfix/*` 브랜치를 분리해 운영하는 방식이다. 반면 연구실 프로젝트나 빠른 실험 사이클에는 **GitHub Flow**가 훨씬 가볍고 실용적이다. 항상 동작하는 `main` 브랜치를 두고, 새 실험이나 기능을 추가할 때마다 `feature-branch`를 따서 작업한 뒤 Pull Request(PR)로 코드 리뷰를 거쳐 `main`에 합친다. 단순한 GitHub Flow만 잘 지켜도 충돌 범위와 변경 목적을 뚜렷하게 구분할 수 있다(PR 절차는 §16.4.1.3).

### 16.4.1.3 협업

여러 명이 함께 코드를 다듬을 때는 기능 브랜치를 바로 `main`에 덮어쓰지 않고 **Pull Request (PR)** 절차를 거친다. 저장소를 fork하거나 새 브랜치를 만든 뒤 변경 사항을 커밋하고, PR을 열어 동료에게 리뷰를 요청해 검토가 끝나면 머지하는 흐름이다.

이때 커밋 메시지에 일정한 규칙을 두면 나중에 히스토리만 훑어도 어떤 성격의 작업이 오갔는지 한눈에 파악된다. 널리 쓰이는 **Conventional Commits** 규칙은 다음과 같다.

```
feat: 새로운 기능
fix: 버그 수정
docs: 문서 변경
refactor: 리팩토링
test: 테스트 추가/수정
chore: 빌드/설정 변경
```

`feat:`나 `fix:` 같은 짧은 접두어만 통일해도 릴리스 노트를 정리하거나 특정 버그 수정 시점을 되짚는 속도가 눈에 띄게 빨라진다.

> **추천 자료**
> - [Conventional Commits Specification](https://www.conventionalcommits.org/) — 커밋 메시지 규칙 공식 스펙

### 16.4.2 SSH

연구실 GPU 서버나 로봇 온보드 컴퓨터에 접속하는 가장 기본적인 통로는 SSH다. 매번 비밀번호를 입력하는 대신 공개키 인증을 설정해 두면 접속이 간편할 뿐 아니라 무차별 대입 공격에 대한 보안도 높아진다. 아래 순서로 키를 만들고 서버에 등록한 뒤, 필요하다면 `-L` 옵션으로 서버의 Jupyter나 TensorBoard 포트를 내 로컬 브라우저로 끌어와 볼 수 있다.

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

서버 IP와 사용자명을 매번 외워서 치기 번거롭다면 `~/.ssh/config` 파일에 별칭을 등록해 둔다.

```
Host lab-server
    HostName 192.168.1.100
    User junholee
    IdentityFile ~/.ssh/id_ed25519
```

이렇게 적어 두면 터미널에서 `ssh lab-server`만 입력해 곧장 접속할 수 있고, VS Code의 Remote-SSH 확장도 이 설정을 자동으로 읽어 원격 서버에 바로 붙는다.

### 16.4.3 SCP & rsync

학습 가중치나 실험 결과 로그, 데이터셋을 로컬과 서버 사이에서 주고받을 때는 `scp`와 `rsync`를 용도에 맞게 골라 쓴다.

가중치 파일 하나나 짧은 결과 로그처럼 작은 파일을 단발로 복사할 때는 **SCP**가 간단하다.

```bash
# 로컬 → 서버
scp model.pth user@server:/home/user/weights/

# 서버 → 로컬
scp user@server:/home/user/results/log.txt ./

# 디렉토리 복사
scp -r dataset/ user@server:/data/
```

반면 수십 GB에 달하는 데이터셋을 옮기거나 중간에 끊긴 전송을 이어받고 싶을 때는 **rsync**가 압도적으로 유리하다.

```bash
# 로컬 → 서버 (변경분만 전송, 진행률 표시)
rsync -avz --progress dataset/ user@server:/data/dataset/

# 서버 → 로컬
rsync -avz user@server:/home/user/results/ ./results/

# 삭제된 파일도 동기화 (미러링)
rsync -avz --delete source/ user@server:/data/source/
```

`rsync`는 이미 상대편에 있는 파일을 건너뛰고 바뀐 부분만 압축(`-z`)해 보내므로, 매번 폴더 전체를 처음부터 다시 복사하는 `scp`보다 시간과 네트워크 대역폭을 크게 아껴 준다.

### 16.4.4 Tailscale

연구실 서버가 공유기나 기관 방화벽(NAT) 안쪽에 놓여 있으면 집이나 카페에서 곧바로 SSH 접속을 열 수 없다. **Tailscale**은 WireGuard 기반 메시 VPN으로, 서버와 내 노트북 양쪽에 설치해 로그인만 해 두면 별도의 공유기 포트 포워딩 없이도 사설 고정 IP로 직접 연결된다.

```bash
# 설치 (서버와 로컬 양쪽에)
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up

# 상태 확인 — 연결된 기기 목록과 IP 확인
tailscale status

# 이후 Tailscale IP로 SSH
ssh user@100.x.y.z
```

덕분에 카페·집·학교 어디서든 똑같은 IP로 연구실 GPU 서버에 접속하며, Tailscale SSH 기능을 켜면 별도의 SSH 키를 배포하지 않고도 Tailscale 계정 인증으로 접속을 관리한다.

자주 붙는 서버라면 `~/.ssh/config`에 Tailscale IP를 별칭으로 등록해 두어 `ssh lab-gpu` 한 줄로 연결되도록 만들어 둔다.

```
Host lab-gpu
    HostName 100.x.y.z
    User junholee
```

> **추천 자료**
> - [Tailscale 공식 문서](https://tailscale.com/docs) — 설치부터 ACL 설정까지
> - [The Missing Semester - Remote Machines](https://missing.csail.mit.edu/2020/command-line/#remote-machines) — SSH, 포트 포워딩, tmux 등 원격 작업 기초

## 16.5 실험 관리

원격 서버에서 딥러닝 실험을 여러 번 돌리다 보면 "어제 가장 잘 나온 모델의 학습률과 배치 크기가 뭐였지?" 하고 헤매는 순간이 반드시 온다. 엑셀 표나 개인 노트에 손으로 적는 방식은 실험이 열 개만 넘어가도 금방 누락이 생긴다. 실험 설정과 손실 곡선, 모델 체크포인트를 자동으로 묶어 기록해 두는 도구가 필요한 이유다.

### 16.5.1 Weights & Biases (wandb)

**Weights & Biases(wandb)**는 학습 지표와 하이퍼파라미터, 시스템 자원 사용량을 클라우드 대시보드에 실시간으로 기록하고 팀원과 링크 하나로 공유하기 좋은 도구다. 학습 스크립트 앞뒤에 초기화·로깅·종료 호출만 몇 줄 넣어 주면 모든 실행 이력이 웹 화면에 쌓인다.

```python
import wandb

# 초기화
wandb.init(project="my-project", config={
    "learning_rate": 0.001,
    "epochs": 100
})

# 로깅
for epoch in range(wandb.config.epochs):
    loss = train_one_epoch()
    wandb.log({"loss": loss, "epoch": epoch})

# 완료
wandb.finish()
```

팀 프로젝트에서 여러 사람이 돌린 실험 곡선을 한 그래프 위에 겹쳐 비교할 때 특히 편리하다.

> **추천 자료**
> - [Weights & Biases 공식 문서 및 Quickstart](https://docs.coreweave.com/products/wandb/quickstart) — wandb 시작 가이드. 5분이면 첫 실험 로깅 가능
> - [Weights & Biases YouTube](https://www.youtube.com/@CoreWeaveYT) — 사용법 튜토리얼 및 MLOps 관련 강연. 옛 W&B 공식 채널 주소(youtube.com/c/weights-biases)가 지금은 CoreWeave 채널로 이어진다

### 16.5.2 MLflow

외부 클라우드 서비스에 학습 로그나 모델 가중치를 올리기 어려운 보안 환경이나 사내 폐쇄망에서는 자체 서버에 설치해 운영하는 오픈소스 도구인 **MLflow**가 대안이 된다. `with mlflow.start_run():` 블록 안에서 파라미터, 메트릭, PyTorch 모델 아티팩트를 로컬이나 사내 추적 서버에 저장한다.

```python
import mlflow

mlflow.set_experiment("my-experiment")

with mlflow.start_run():
    mlflow.log_param("lr", 0.001)
    mlflow.log_metric("accuracy", 0.95)
    mlflow.pytorch.log_model(model, "model")
```

### 16.5.3 TensorBoard

외부 계정 가입이나 별도 서버 구축 없이 내 컴퓨터에서 가볍게 학습 곡선과 샘플 이미지만 확인하고 싶다면 **TensorBoard**가 가장 손쉽다. PyTorch 기본 유틸리티(`torch.utils.tensorboard`)에 내장되어 있어 `SummaryWriter`로 로그 디렉터리에 스칼라와 이미지를 써 둔 뒤 `tensorboard` 명령으로 웹 UI를 띄우면 된다.

```python
from torch.utils.tensorboard import SummaryWriter

writer = SummaryWriter("runs/experiment1")
writer.add_scalar("Loss/train", loss, epoch)
writer.add_image("Sample", image, epoch)
writer.close()
```

로그가 쌓이는 동안 별도 터미널이나 백그라운드에서 아래 명령을 실행하면 브라우저로 실시간 변화를 확인한다.

```bash
tensorboard --logdir runs
```

빠른 단일 실험이나 로컬 디버깅에는 TensorBoard만으로 충분하고, 수십 번의 하이퍼파라미터 탐색과 팀 공유가 잦아지면 wandb나 MLflow로 옮겨 가는 흐름이 자연스럽다.

> **추천 자료**
> - [PyTorch TensorBoard Tutorial](https://docs.pytorch.org/tutorials/recipes/recipes/tensorboard_with_pytorch.html) — PyTorch에서 TensorBoard 사용하는 공식 가이드
> - [MLflow Documentation](https://mlflow.org/docs/latest/index.html) — MLflow 공식 문서

## 16.6 코드 포매팅

코드가 쌓이고 협업자가 늘어나면 "어떻게 동작하는가"만큼이나 "얼마나 일관되게 읽히고 깨지지 않는가"가 중요해진다. 자동 포매터와 린터, 그리고 최소한의 단위 테스트를 초반에 붙여 두면 유지보수 비용이 크게 줄어든다.

### 16.6.1 Linting & Formatting

팀원마다 들여쓰기나 따옴표, 줄바꿈 스타일이 다르면 코드 리뷰에서 정작 중요한 알고리즘 로직 대신 스타일 논쟁으로 시간을 허비하게 된다. 자동 포매터를 도입하면 이런 소모전이 사라지고, 혼자 연구할 때도 몇 달 뒤 자기 코드를 다시 읽을 때 한결 수월하다. Python에서는 속도가 빠르고 린팅과 Black 호환 포매팅을 한 번에 처리하는 `ruff`, 정적 타입 검사를 맡는 `mypy`를 자주 조합하고, C++에서는 `clang-format`을 쓴다.

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

저장소 루트에 설정 파일(`pyproject.toml`이나 `.clang-format`)을 함께 커밋해 두면 에디터 저장 시 자동으로 맞춰져 따로 신경 쓸 일이 없다.

### 16.6.2 Testing

연구용 코드는 수시로 뜯어고치기 때문에 아예 테스트를 안 적고 넘어가는 일이 흔하다. 하지만 "모델의 forward pass 입출력 텐서 shape가 맞는지", "데이터 전처리 함수가 경계 조건에서 예상값을 내는지" 같은 아주 가벼운 테스트 몇 개만 두어도 코드를 리팩토링하거나 새 기능을 붙일 때 심리적 안전망이 생긴다. Python에서는 `pytest`, C++과 ROS2 패키지에서는 `gtest`가 표준처럼 쓰인다.

**Python (pytest)**:

```python
# tests/test_module.py
def test_addition():
    assert 1 + 1 == 2

def test_function():
    result = my_function(input)
    assert result == expected
```

테스트 파일을 `tests/` 아래에 모아 두고 아래 한 줄만 실행하면 모든 검증 항목이 자동으로 돈다.

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

특히 로봇 소프트웨어처럼 하드웨어와 맞물려 돌아가는 시스템에서는 시뮬레이션이나 현장 테스트에 올리기 전에 순수 로직을 단위 테스트로 먼저 걸러 내는 습관이 디버깅 시간을 극적으로 줄여 준다.

> **추천 자료**
> - [Real Python - Python Testing with pytest](https://realpython.com/pytest-python-testing/) — pytest 사용법 상세 튜토리얼
> - [Fireship YouTube](https://www.youtube.com/@Fireship) — 각종 개발 도구를 "100 Seconds" 시리즈로 빠르게 훑어볼 수 있다
> - [정진용 블로그 — 로봇 소프트웨어 개발 문화](https://jinyongjeong.github.io/2025/02/14/developmen_culture/) — 코드 리뷰, CI/CD, 스타일 가이드 등 로봇 개발팀의 개발 문화 정착 방법
> - [정진용 블로그 — 로봇 개발과 테스트 코드](https://jinyongjeong.github.io/2025/02/19/test_code/) — 로봇 소프트웨어에서 테스트 코드가 필수인 6가지 이유

실행 환경과 협업 파이프라인이 단단히 갖춰졌다면, 이제 알고리즘의 성능을 좌우하고 서로 다른 연구 결과를 공정하게 비교할 기준이 되는 데이터셋과 벤치마크를 살펴볼 차례다(→ 17장).

## 발전 과정

```
2005 ─── Git 탄생 (Linus Torvalds)
  │
2008 ─── GitHub 출시
  │
2012 ─── Conda (Anaconda) 등장
  │
2013 ─── Docker 공개
  │       컨테이너 이미지로 사용자 공간 의존성 재현이 쉬워짐
  │
2015 ─── TensorBoard (TensorFlow와 함께 공개)
  │
2016 ─── nvidia-docker 공개 (1.0 정식은 2017년 1월)
  │       Docker 컨테이너에서 GPU 사용 가능
  │
2018 ─── Weights & Biases 출시
  │       실험 추적·시각화·팀 협업을 클라우드로
  │
2022 ─── GitHub Copilot 정식 출시 (2022년 6월, 기술 미리보기는 2021년)
  │       에디터 안에서 코드를 제안하는 AI 코딩 보조 도구
  │
2022 ─── Ruff 공개 (2022년 8월, Black은 2018년 공개)
          Rust로 작성한 Python 린터
```
