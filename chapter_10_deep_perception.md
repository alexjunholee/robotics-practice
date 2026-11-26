# 10장 딥러닝 기반 인식

9장에서 다룬 기하 기반 컴퓨터 비전이 이미지 속 모서리와 3차원 점들의 좌표를 짚어 준다면, 딥러닝 기반 인식은 그 점들이 도대체 **무엇**인지 의미를 읽어 낸다. 데이터로부터 직접 표현을 학습한 신경망은 장면을 보고 물체의 종류를 맞히고(분류), 어디에 놓여 있는지 찾아 사각형으로 감싸고(물체 탐지), 심지어 픽셀 하나하나가 어떤 물체에 속하는지 경계를 갈라 낸다(분할). 로봇이 어지러운 작업대 위에서 원하는 부품만 집어 올리거나 보행자를 피해 주행하려면 바로 이 의미 정보가 기하 정보와 맞물려야 한다.

---

## 10.1 프레임워크 선택

인식 모델을 직접 돌리고 튜닝하려면 먼저 딥러닝 프레임워크를 하나 손에 익혀야 한다. 어떤 프레임워크를 주력으로 잡느냐에 따라 논문에 딸려 나오는 공개 연구 코드와 사전학습 모델(pretrained model)을 얼마나 손쉽게 가져다 쓸 수 있는지가 갈린다. 물론 한 프레임워크로 모델 정의와 학습, 추론, 디버깅 사이클을 한 번 제대로 익혀 두면, 다른 프레임워크로 짜인 코드베이스도 금세 구조를 읽고 따라갈 수 있다.

### 10.1.1 PyTorch (권장)

로보틱스와 컴퓨터 비전 연구에 입문한다면 **PyTorch**로 시작하기를 권한다. 코드가 쓰인 순서 그대로 한 줄씩 실행되는 동적 그래프(eager execution) 방식을 따르기 때문에 중간 텐서 모양이나 값을 일반 Python 디버거로 곧바로 찍어 보며 디버깅하기 편하다. 여기에 `torchvision`과 `timm` 같은 생태계 덕분에 검증된 사전학습 모델을 몇 줄 만에 불러올 수 있고, 최신 논문의 공개 구현도 대부분 PyTorch로 작성되어 있어 연구 코드를 실험에 곧장 적용하고 고치기가 수월하다.

CUDA 12.1 환경 기준으로 설치 명령은 다음과 같다.

```bash
# CUDA 12.1 버전
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121
```

텐서를 만들고 GPU 가속기가 잡히는지 확인해 장치를 옮기는 기본 패턴은 아래와 같다.

```python
import torch
import torch.nn as nn

# 텐서 생성
x = torch.randn(32, 3, 224, 224)  # (batch, channel, height, width)

# GPU 사용
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
x = x.to(device)
```

> **추천 자료**
> - [PyTorch 공식 튜토리얼](https://docs.pytorch.org/tutorials/) — 입문부터 고급 주제까지 다룬다
> - [d2l.ai (Dive into Deep Learning)](https://d2l.ai/) — 인터랙티브 교과서. PyTorch 코드와 수학이 함께 나온다
> - [Andrej Karpathy — Neural Networks: Zero to Hero](https://www.youtube.com/playlist?list=PLAqhIrjkxbuWI23v9cThsA9GvCAUhRvKZ) — 전 Tesla AI Director가 신경망을 밑바닥부터 설명
> - [Jaejun Yoo's Playground](https://jaejunyoo.blogspot.com/search/label/kr) — GAN, VAE 같은 생성 모델을 한국어로 설명한 블로그

### 10.1.2 TensorFlow / JAX

실무와 연구 현장에서는 PyTorch 외에 **TensorFlow**와 **JAX**도 자주 마주친다. TensorFlow는 프로덕션 서버 배포 도구가 잘 갖춰져 있을 뿐 아니라, TF Lite(2024년 9월 **LiteRT**로 이름이 바뀜)를 앞세워 모바일 기기나 라즈베리파이 같은 소형 엣지 디바이스에 경량 모델을 올릴 때 강점을 발휘한다. 반면 JAX는 함수형 프로그래밍 패러다임을 바탕으로 자동 미분과 XLA 컴파일을 결합한 고성능 수치 연산 라이브러리다. 주로 연구 목적으로 활용되거나 Google DeepMind의 공개 코드베이스에서 많이 쓰인다. 처음부터 셋을 다 파고들 필요는 없고 PyTorch를 기본 무기로 삼되, 두 프레임워크가 프로젝트나 논문에 등장할 때 성격과 용도를 구분할 수 있으면 충분하다.

> **추천 자료**
> - [TensorFlow 공식 가이드](https://www.tensorflow.org/guide) — TFLite 변환까지 다룬다
> - [JAX 공식 문서](https://docs.jax.dev/en/latest/) — 함수형 딥러닝 프레임워크

---

## 10.2 딥러닝 기초 개념

이 장 뒷부분에서 살펴볼 탐지·분할 모델은 겉보기엔 복잡해 보여도, 결국 두 가지 핵심 엔진으로 돌아간다. 하나는 ResNet처럼 지역 패턴을 훑어 올리는 **CNN의 합성곱 구조**이고, 다른 하나는 ViT와 DETR처럼 화면 전체의 문맥을 한 번에 엮는 **Transformer의 attention 구조**다. 두 작동 원리부터 분명히 짚고 넘어가자.

### 10.2.1 CNN (Convolutional Neural Network)

9장에서 다룬 SIFT나 ORB는 사람이 수학적 직관으로 설계한 고정 필터를 써서 코너와 모서리를 찾았다(→ 9장). 반면 **CNN**은 작은 필터(kernel)를 이미지 격자 위에서 미끄러지듯 움직이며 공간 특징을 뽑아낸다. 이때 필터의 가중치 숫자는 사람이 정하지 않고, 학습 목적함수를 최적화하는 과정에서 CNN이 데이터로부터 스스로 찾아낸다. 덕분에 얕은 층에서는 단순한 에지나 색상 대비를 잡아내고, 깊은 층으로 올라갈수록 바퀴나 손잡이 같은 고수준 부품을 포착하면서 계층적 표현이 자연스럽게 만들어진다.

전형적인 CNN에서는 다음 네 가지 요소가 한 블록으로 맞물려 동작한다.

- **Convolution Layer**: 학습 가능한 필터로 국소 영역의 공간 특징을 추출한다.
- **Batch Normalization**: 각 층의 출력 분포를 정규화해 학습 중 기울기가 튀거나 사라지는 현상을 줄이고 수렴을 안정시킨다.
- **Activation**: ReLU나 GELU 같은 비선형 함수를 걸어 단순 선형 변환의 반복으로 주저앉지 않게 한다.
- **Pooling Layer**: Max나 Average 연산으로 특징 맵의 공간 해상도를 줄여 연산량을 낮추고 작은 위치 흔들림에 둔감하게 만든다.

실제 PyTorch 코드에서 가장 자주 마주치는 기본 합성곱 블록은 대개 `Conv2d → BatchNorm2d → ReLU`의 세 줄로 이루어진다.

```python
# 간단한 CNN 블록
class ConvBlock(nn.Module):
    def __init__(self, in_ch, out_ch):
        super().__init__()
        self.conv = nn.Conv2d(in_ch, out_ch, kernel_size=3, padding=1)
        self.bn = nn.BatchNorm2d(out_ch)
        self.relu = nn.ReLU(inplace=True)

    def forward(self, x):
        return self.relu(self.bn(self.conv(x)))
```

선형대수 관점에서 보면 convolution은 필터 가중치와 이미지 패치 사이의 내적을 위치마다 반복 계산하는 연산이다. 위 코드처럼 `stride=1`일 때 `kernel_size=3, padding=1`로 맞추면 가장자리 픽셀에 1칸씩 여백(zero padding)이 붙어 출력 특징 맵의 가로·세로 크기가 입력과 똑같이 유지된다. 해상도를 그대로 유지한 채 채널 수만 늘려 가며 특징을 뽑고 싶을 때 CNN 코드에 단골로 등장하는 조합이다.

> **추천 자료**
> - [Stanford CS231n — Convolutional Neural Networks for Visual Recognition](https://www.youtube.com/playlist?list=PLoROMvodv4rMFqRtEuo6SGjY4XbRIVRd4) — CNN을 다루는 Stanford 강의
> - [d2l.ai — CNN 챕터](https://d2l.ai/chapter_convolutional-neural-networks/index.html) — 코드와 수학 설명이 함께 나온다
> - [3Blue1Brown — But what is a Neural Network?](https://www.youtube.com/watch?v=aircAruvnKk) — 신경망을 그림으로 설명하는 영상

### 10.2.2 Attention & Transformer

CNN의 합성곱은 한 층에서 바로 이웃한 3×3 같은 좁은 범위만 바라보기 때문에, 화면 왼쪽 끝과 오른쪽 끝처럼 멀리 떨어진 영역의 관계를 엮으려면 층을 아주 깊게 쌓아 **receptive field**(출력 픽셀 하나가 영향을 받는 입력 영역의 범위)를 넓혀야 한다. 이와 달리 Transformer의 **self-attention**은 단 한 층 안에서도 입력 시퀀스의 모든 위치 쌍을 서로 직접 비교해 장거리 문맥을 한꺼번에 계산한다.

```
Attention(Q, K, V) = softmax(QK^T / √d_k) V
```

수식에서 Query(`Q`)와 Key(`K`)의 내적으로 모든 위치 쌍의 관련도 점수를 구한 뒤 `√d_k`로 스케일을 맞추고 `softmax`로 가중치를 정규화해 Value(`V`)를 가중 합산한다. **Vision Transformer(ViT)**는 이 구조를 이미지에 그대로 이식하기 위해, 이미지를 16×16 같은 고정 크기의 정사각형 패치로 잘게 자른 뒤 각 패치를 자연어 처리의 단어 토큰(token) 하나처럼 펼쳐 Transformer encoder에 밀어 넣는다. 원 논문에서는 대규모 데이터셋으로 사전학습했을 때 순수 Transformer만으로도 CNN과 동등하거나 더 높은 분류 성능을 낼 수 있음을 보였다. 2020년 이후에는 분류·탐지·분할 모델 전반에 걸쳐 순수 Transformer 구조와 CNN-Transformer 하이브리드 구조가 함께 주류를 이루고 있다.

> **추천 자료**
> - [Vaswani et al., "Attention Is All You Need" (2017)](https://arxiv.org/abs/1706.03762) — Transformer 원논문
> - [Dosovitskiy et al., "An Image is Worth 16x16 Words" (2020)](https://arxiv.org/abs/2010.11929) — ViT 원논문
> - [Yannic Kilcher — Vision Transformer 설명](https://www.youtube.com/watch?v=TrdevFK_am4) — 논문을 풀어 설명하는 영상
> - [Andrej Karpathy — Let's build GPT from scratch](https://www.youtube.com/watch?v=kCc8FmEb1nY) — Transformer를 밑바닥부터 구현한다. 예제는 NLP지만 ViT도 같은 encoder 구조를 쓴다

---

## 10.3 Image Classification

로봇이 실제 현장에서 단순히 "이 사진은 머그컵 사진이다"라는 딱지 하나만 얻는 데서 멈추는 일은 드물다. 그럼에도 이미지 분류(classification)가 모든 비전 태스크의 출발점이 되는 이유는 분명하다. 물체 탐지기든 픽셀 분할 모델이든 내부에는 클래스 이름을 판별하는 분류기가 들어가며, 무엇보다 ImageNet 같은 대규모 분류 데이터셋으로 미리 학습해 둔 **backbone**(특징 추출기)이 다른 모든 비전 모델의 척추 역할을 맡기 때문이다.

시대에 따라 널리 쓰이는 대표 분류 백본은 구조와 계산 예산 면에서 각기 성격이 뚜렷하다.

| 모델 | 특징 | 용도 |
| --- | --- | --- |
| ResNet | Residual connection으로 깊은 네트워크도 학습이 안정적 | 백본 네트워크 |
| EfficientNet | 깊이·너비·해상도를 함께 키우는 compound scaling | 모바일, 연산량이 제한된 환경 |
| ViT | Transformer 기반 | 대규모 데이터로 사전학습할 때 |
| ConvNeXt | ViT의 설계 요소를 CNN에 옮김 | ViT 대신 쓰는 CNN 백본 |

그중에서도 2015년에 등장한 ResNet의 **residual connection**(잔차 연결)은 딥러닝 설계의 판도를 바꾸어 놓았다. 블록의 입력 `x`를 출력 `F(x)`에 곧바로 더해 `F(x) + x`를 내보내는 이 단순한 지름길 덕분에, 수십~백수십 개 층을 쌓아도 역전파 기울기가 소실되지 않고 네트워크가 안정적으로 학습된다. 이후 등장한 Transformer를 비롯한 현대 아키텍처 대부분에서도 이 잔차 연결을 기본 골격으로 채택했다.

실무에서 새 로봇 작업에 맞는 인식기를 만들 때는 처음부터 무작위 가중치로 학습하지 않고 **전이 학습(transfer learning)**을 쓴다. ImageNet 등으로 이미 시각적 패턴을 배운 모델을 가져와 마지막 분류 층만 떼어 내고 앞단의 특징 벡터를 재활용하는 방식이다. 새 도메인의 라벨 데이터가 몇백~몇천 장밖에 없을 때도 탄탄한 초기 표현을 곧바로 빌려 올 수 있기 때문이다.

```python
import torchvision.models as models

# Pretrained ResNet50
model = models.resnet50(weights='IMAGENET1K_V2')

# Feature extractor로 사용
model.fc = nn.Identity()  # 마지막 FC 제거
features = model(x)  # (batch, 2048)
```

> **추천 자료**
> - [He et al., "Deep Residual Learning for Image Recognition" (2015)](https://arxiv.org/abs/1512.03385) — ResNet 원논문
> - [Papers With Code — Image Classification (2025년 서비스 종료 전 Internet Archive 보관본)](https://web.archive.org/web/20241228230436/https://paperswithcode.com/task/image-classification) — 공개 구현과 과거 leaderboard를 찾는 출발점. 최신 수치와 protocol은 벤치마크 공식 페이지와 원 논문에서 다시 확인
> - [timm (PyTorch Image Models) 라이브러리](https://github.com/huggingface/pytorch-image-models) — 수백 개의 사전학습 모델을 한 줄로 로드
> - [Stanford CS231n — Training Neural Networks](https://www.youtube.com/playlist?list=PLoROMvodv4rMFqRtEuo6SGjY4XbRIVRd4) — 학습 기법과 트릭

---

## 10.4 Object Detection

테이블 위의 컵을 집어 올리려면 화면에 컵이 있다는 사실만으로는 부족하고, 그 컵이 정확히 어느 위치에 있는지 좌표까지 알아내야 한다. Object detection(물체 탐지)은 이미지 속에서 물체의 클래스와 함께 그 물체를 감싸는 직사각형 경계 상자(bounding box)를 동시에 예측하는 과제로, 로봇 조작(manipulation)과 자율주행의 눈 역할을 맡는다.

### 10.4.1 Two-Stage Detectors

초기 딥러닝 탐지기들은 정확도를 높이기 위해 "물체가 있을 법한 후보를 먼저 추리고, 각 후보를 다시 들여다보는" 2단계 전략을 택했다. 그 대표 모델인 **Faster R-CNN**은 다음 세 단계로 동작한다.

1. **Region Proposal Network (RPN)**: 특징 맵 위에서 물체가 들어 있을 법한 후보 영역(proposal)을 추려 낸다.
2. **ROI Pooling**: 크기와 비율이 제각각인 후보 영역마다 고정 크기의 특징 맵을 잘라낸다.
3. **Classification + Bounding Box Regression**: 각 후보가 어떤 클래스인지 판별하고 경계 상자의 네 좌표를 미세하게 다듬는다.

두 번에 나누어 검증하는 덕분에 탐지 정확도는 높지만, 후보마다 분류와 회귀를 반복해야 해서 뒤에 나올 one-stage 방식보다 추론 속도가 느리다. 그럼에도 이 "후보 제안 후 정밀 예측" 구조는 이후 픽셀 마스크까지 뽑아내는 Mask R-CNN의 토대가 되었다.

### 10.4.2 One-Stage Detectors

로봇이 움직이며 실시간으로 장애물을 피하거나 컨베이어 벨트 위 물건을 잡으려면 정확도를 조금 양보하더라도 추론 지연을 크게 줄여야 한다. **YOLO(You Only Look Once)**는 이미지를 격자(grid)로 나눈 뒤 단 한 번의 forward pass로 모든 칸의 경계 상자와 클래스를 한꺼번에 예측하는 one-stage 구조로 이 병목을 깼다. 원 논문 기준으로 초당 45프레임(45 FPS)을 달성해 실시간 물체 탐지의 문을 열었고, 이후 YOLOv5, YOLOv8, YOLO11(Ultralytics) 등으로 꾸준히 진화해 왔다. 특히 Ultralytics의 YOLOv8과 YOLO11은 설치부터 커스텀 데이터 학습, 추론까지 절차가 매우 짧아 로봇 프로토타이핑 현장에서 가장 먼저 손이 가는 모델이다.

```python
from ultralytics import YOLO

# 모델 로드 및 추론
model = YOLO('yolov8n.pt')  # nano 모델
results = model('image.jpg')

# 결과 시각화
results[0].show()
```

또 다른 대표 one-stage 모델인 **SSD(Single Shot Detector)**는 해상도가 다른 여러 층의 feature map에서 각각 박스를 예측해 다양한 크기의 물체에 대응하려 했다. 하지만 얕은 층과 깊은 층의 특징 결합이 약해 작은 물체를 놓치는 약점이 드러났고, 이후 YOLOv3 등이 Feature Pyramid Network(FPN)와 비슷한 다중 스케일 예측 구조를 본격 도입하면서 작은 물체 탐지 성능을 크게 끌어올렸다.

> **추천 자료**
> - [Redmon et al., "You Only Look Once: Unified, Real-Time Object Detection" (2016)](https://arxiv.org/abs/1506.02640) — YOLO 원논문
> - [Ultralytics YOLOv8 문서](https://docs.ultralytics.com/) — 설치부터 직접 만든 데이터로 학습하는 방법까지 다룬다
> - [Papers With Code — Object Detection (2025년 서비스 종료 전 Internet Archive 보관본)](https://web.archive.org/web/20250115141015/https://paperswithcode.com/task/object-detection) — 공개 구현과 과거 leaderboard를 찾는 출발점. 최신 수치와 protocol은 벤치마크 공식 페이지와 원 논문에서 다시 확인
> - [다크 프로그래머 — precision, recall의 이해](https://darkpgmr.tistory.com/162) — detection 평가 지표를 한국어로 설명

### 10.4.3 Transformer-based

전통적인 one-stage·two-stage 탐지기들은 수작업 규칙에 크게 의존했다. 수많은 anchor box를 미리 깔아 두고, 겹친 예측 상자를 NMS(non-maximum suppression) 후처리로 추려 내는 방식이었다. **DETR(Detection Transformer)**은 물체 탐지를 "고정된 개수의 물체 집합을 맞히는 문제"로 다시 정의해 이 수작업을 걷어 냈다. 학습 가능한 벡터인 `object query`들이 각자 물체 하나씩을 맡아 이미지 특징과 cross-attention을 주고받으며 최종 박스와 클래스를 바로 내놓으므로, 별도의 anchor나 NMS 없이 완전한 end-to-end 학습이 가능하다.

초창기 DETR은 attention 가중치가 수렴하는 데 학습 epoch이 아주 오래 걸리는 단점이 있었다. 하지만 관심 위치 주변만 집중해 보는 Deformable DETR, 탐지 성능과 수렴 속도를 끌어올린 DINO, 그리고 Co-DETR 같은 후속 연구들이 이어지면서 실시간 탐지 분야까지 Transformer 계열의 영역이 넓어졌다. (여기서 DINO는 DETR 계열 탐지기로, §10.9의 자기지도 백본 DINOv2와는 다른 모델이다.)

> **추천 자료**
> - [Carion et al., "End-to-End Object Detection with Transformers" (2020)](https://arxiv.org/abs/2005.12872) — DETR 원논문
> - [Yannic Kilcher — DETR 설명](https://www.youtube.com/watch?v=T35ba_VXkMY) — 논문을 풀어 설명하는 영상
> - [HuggingFace — Object Detection 가이드](https://huggingface.co/docs/transformers/tasks/object_detection) — Transformers 라이브러리로 DETR 사용하기
> - [Zhao et al., "DETRs Beat YOLOs on Real-time Object Detection" (RT-DETR, CVPR 2024, arXiv:2304.08069)](https://arxiv.org/abs/2304.08069) — 논문의 비교 설정에서 실시간 DETR의 속도·정확도 개선을 보고
> - [Cheng et al., "YOLO-World: Real-Time Open-Vocabulary Object Detection" (CVPR 2024, arXiv:2401.17270)](https://arxiv.org/abs/2401.17270) — YOLO에 텍스트 프롬프트로 클래스를 지정하는 open-vocabulary detection을 더했다

---

## 10.5 Semantic Segmentation

Bounding box는 직사각형 안에 물체뿐 아니라 주변 배경 픽셀까지 함께 가두기 때문에, 가느다란 도구 손잡이를 잡거나 도로와 인도의 경계를 따라 주행해야 하는 로봇에게는 너무 거친 표현일 때가 많다. **Semantic segmentation**은 이미지의 모든 픽셀에 클래스를 하나씩 매겨 물체와 배경의 경계를 세밀하게 갈라 낸다. 자율주행 차량이 주행 가능한 도로, 인도, 차선을 구분하는 작업이 전형적인 쓰임새다. 다만 예측한 분할 경계가 실제 물리적 윤곽과 항상 완벽히 포개지는 것은 아니므로, 로봇 파지(grasping)에 활용할 때는 깊이 정보, 개체 분리 결과, 예측 불확실성을 함께 따져 안전 여백을 두어야 한다.

픽셀 단위 분할을 구현하는 대표 아키텍처들은 공간 해상도를 복원하고 문맥을 넓히는 전략에 따라 다음과 같이 발전해 왔다.

| 모델 | 특징 |
| --- | --- |
| FCN | fully connected 층 없이 convolution만으로 end-to-end 분할을 한 초기 모델 |
| U-Net | Encoder-Decoder 구조, 의료 영상 분할용으로 제안 |
| DeepLab v3+ | Atrous(dilated) convolution으로 여러 스케일의 문맥을 모음 |
| SegFormer | Transformer encoder와 가벼운 MLP 디코더 |

그중 **U-Net**의 구조는 분할을 넘어 비전 전반에서 교과서처럼 쓰인다. Encoder가 해상도를 줄여 가며 "화면에 무엇이 있는가"라는 전역 문맥을 압축하면, Decoder가 해상도를 다시 키워 원래 픽셀 격자로 되돌린다. 이때 해상도를 키우는 과정에서 잃어버리기 쉬운 날카로운 경계 정보를 되살리려고, Encoder의 같은 해상도 층에서 나온 특징 맵을 Decoder로 직접 건네주는 **Skip Connection**을 둔다. 이 Encoder-Decoder와 Skip Connection 조합은 semantic segmentation뿐 아니라 단안 깊이 추정과 이미지 생성 모델의 핵심 뼈대로도 두루 쓰인다.

HuggingFace `transformers` 라이브러리를 이용하면 경량 Transformer 분할 모델인 SegFormer를 곧바로 불러와 추론할 수 있다.

```python
# Segmentation 모델 사용 (transformers 라이브러리)
from transformers import SegformerForSemanticSegmentation

model = SegformerForSemanticSegmentation.from_pretrained(
    "nvidia/segformer-b0-finetuned-ade-512-512"
)
```

> **추천 자료**
> - [Papers With Code — Semantic Segmentation (2025년 서비스 종료 전 Internet Archive 보관본)](https://web.archive.org/web/20241226120222/https://paperswithcode.com/task/semantic-segmentation) — 공개 구현과 과거 leaderboard를 찾는 출발점
> - [HuggingFace — Image Segmentation](https://huggingface.co/docs/transformers/tasks/semantic_segmentation) — SegFormer 등 사용법
> - [Two Minute Papers — Semantic Segmentation 관련 영상들](https://www.youtube.com/@TwoMinutePapers) — 연구 결과를 짧게 소개하는 채널

---

## 10.6 Instance & Panoptic Segmentation

Semantic segmentation은 픽셀마다 클래스 이름("의자")만 붙일 뿐, 화면에 의자 세 개가 나란히 놓여 있어도 이를 전부 한 덩어리의 "의자 픽셀"로 뭉뚱그린다. 하지만 로봇에게 "왼쪽 의자를 집어" 또는 "앞에 있는 두 번째 상자를 옮겨"라고 지시하려면 같은 클래스 안에서도 개별 물체를 따로 떼어 내야 한다.

- **Instance Segmentation**: 같은 클래스에 속하더라도 개체 하나하나("의자 1, 2, 3")를 구분해 각각의 마스크를 예측한다. 대표적인 모델인 **Mask R-CNN**은 Faster R-CNN의 후보 영역 파이프라인에 픽셀 마스크를 예측하는 작은 브랜치를 덧붙여 박스 탐지와 개체 분할을 동시에 해결했다.
- **Panoptic Segmentation**: Semantic segmentation과 Instance segmentation을 단일 출력으로 통합한 과제다. 의자나 자동차처럼 셀 수 있는 대상(**Things**)은 개체별로 번호를 매겨 구분하고, 하늘이나 도로처럼 셀 수 없는 배경(**Stuff**)은 클래스 단위 영역으로 묶어 장면의 모든 픽셀을 빠짐없이 설명한다.

> **추천 자료**
> - [He et al., "Mask R-CNN" (2017)](https://arxiv.org/abs/1703.06870) — Mask R-CNN 원논문
> - [Detectron2](https://github.com/facebookresearch/detectron2) — Meta의 detection/segmentation 프레임워크. Mask R-CNN 같은 모델이 들어 있다

---

## 10.7 Depth Estimation

지금까지의 탐지와 분할이 "무엇이 화면의 어느 픽셀에 있는가"를 읽는 2차원 인식이었다면, 로봇이 실제로 팔을 뻗고 장애물을 피하려면 그 물체까지 거리가 얼마나 되는지 알아야 하므로 3차원 깊이가 필요하다. **Monocular depth estimation(단안 깊이 추정)**은 스테레오 카메라나 무거운 LiDAR 없이 단 한 장의 RGB 이미지에서 픽셀별 깊이를 예측한다. 센서 가격과 무게를 아낄 수 있어 탑재 중량(payload)이 빠듯한 드론이나 소형 이동 로봇에 특히 요긴하며, 2024년에 발표된 Depth Anything처럼 대규모 데이터로 학습해 처음 보는 실내외 장면에서도 일관된 깊이를 뽑아내는 모델들이 실전성을 높이고 있다.

대표적인 단안 깊이 모델은 학습 데이터 규모와 출력 단위에 따라 성격을 나눌 수 있다.

- **MiDaS**: 성격이 다른 여러 깊이 데이터셋을 섞어 학습해 범용적인 단안 깊이 추정의 토대를 닦았다.
- **Depth Anything**: 라벨이 없는 이미지 약 6,200만 장을 대규모 pseudo-labeling 학습에 더해 처음 보는 환경에서의 일반화 성능을 크게 끌어올렸다.
- **ZoeDepth**: 상대적 순서에 머물지 않고 실제 미터 단위 깊이(metric depth)를 직접 추정한다.

```python
# Depth Anything 사용
from transformers import pipeline

pipe = pipeline("depth-estimation", model="LiheYoung/depth-anything-base-hf")
result = pipe("image.jpg")
depth = result['depth']
```

실제 로봇 제어에 단안 깊이 모델을 연결할 때는 출력의 **척도(scale)**를 반드시 확인해야 한다. MiDaS와 Depth Anything의 기본 모델이 내놓는 값은 절대 거리가 아니라 **상대적 깊이(relative depth)**다. 즉 "A가 B보다 가깝다"는 순서와 상대적 비율은 알 수 있지만, 정확히 몇 미터 떨어져 있는지 숫자를 곧바로 쓰면 스케일 오차로 충돌할 수 있다. 경로 계획이나 파지에 쓸 미터 단위 거리가 필요하다면 ZoeDepth나 Depth Anything V2의 metric 버전을 사용하거나, 별도의 센서·기하 단서로 스케일을 보정해야 한다.

> **추천 자료**
> - [Yang et al., "Depth Anything: Unleashing the Power of Large-Scale Unlabeled Data" (2024)](https://arxiv.org/abs/2401.10891) — Depth Anything 원논문
> - [Godard et al., "Digging Into Self-Supervised Monocular Depth Estimation" (Monodepth2, ICCV 2019, arXiv:1806.01260)](https://arxiv.org/abs/1806.01260) — 자기지도 단안 깊이 추정에서 비교 기준으로 자주 쓰는 모델
> - [HuggingFace — Monocular Depth Estimation](https://huggingface.co/docs/transformers/tasks/monocular_depth_estimation) — 바로 돌려 볼 수 있는 코드
> - [Papers With Code — Monocular Depth Estimation (2025년 서비스 종료 전 Internet Archive 보관본)](https://web.archive.org/web/20241128105333/https://paperswithcode.com/task/monocular-depth-estimation) — 공개 구현과 과거 leaderboard를 찾는 출발점

---

## 10.8 학습 레시피 [심화]

사전학습된 모델을 곧장 가져다 쓰는 것만으로 부족해 로봇 현장 데이터에 맞게 모델을 다시 학습하거나 파인튜닝해야 할 때가 오면, 똑같은 네트워크 아키텍처를 쓰더라도 학습 설정에 따라 수렴 여부와 최종 정확도가 크게 달라진다. 논문 수치를 제대로 재현하고 싶다면 모델 구조뿐 아니라 learning rate 스케줄, 데이터 증강(augmentation), 정규화 기법까지 한 세트로 맞추어야 한다.

학습 초기에는 가중치가 아직 불안정해 큰 step으로 갱신하면 기울기가 튀기 쉽고, 후반부에는 손실 함수의 골짜기에 차분히 내려앉아야 한다. 이를 위해 **Learning Rate Schedule**을 설계한다.

- **Cosine Annealing with Warm-up**: 처음 몇 epoch 동안은 learning rate를 0에서 목표값까지 천천히 선형으로 끌어올려(warm-up) 초기 불안정을 넘긴 뒤에는 코사인 곡선을 따라 완만하게 바닥까지 낮춘다. warm-up이 끝난 뒤의 감쇠 구간은 다음 식으로 나타낸다.

$$\eta_t = \eta_{min} + \frac{1}{2}(\eta_{max} - \eta_{min})\left(1 + \cos\left(\frac{t \cdot \pi}{T}\right)\right)$$

여기서 `η_t`는 `t`번째 epoch의 learning rate, `η_max`와 `η_min`은 시작값과 최솟값, `T`는 코사인 감쇠가 진행되는 전체 epoch 수(아래 코드의 `T_max`)다.

- **OneCycleLR**: 전체 학습 기간 동안 learning rate를 한 번 최댓값까지 끌어올렸다가 서서히 내리는 단일 사이클 스케줄이다. Smith는 이 방식으로 보통보다 훨씬 적은 epoch 만에 수렴하는 현상을 super-convergence라 불렀다.

```python
import torch.optim as optim

# Cosine Annealing (warm-up 없음). warm-up이 필요하면 아래 OneCycleLR의 pct_start처럼 넣는다
optimizer = optim.AdamW(model.parameters(), lr=1e-4, weight_decay=0.05)
scheduler = optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=100)

# OneCycleLR
scheduler = optim.lr_scheduler.OneCycleLR(
    optimizer, max_lr=1e-3, total_steps=len(dataloader) * num_epochs,
    pct_start=0.1  # 처음 10%를 warm-up에 사용
)
```

데이터 양이 제한된 비전 문제에서는 **Data Augmentation**으로 모델이 조명이나 위치, 배경 변화에 과적합되지 않도록 입력을 다양하게 비틀어 준다. 태스크에 따라 자주 쓰이는 조합이 다르다.

| 기법 | 설명 | 주 용도 |
|------|------|---------|
| RandAugment | 변환 N개를 세기 M으로 무작위로 고른다 | 분류 전반 |
| CutMix | 이미지 한 영역을 다른 이미지로 바꾸고, 라벨도 면적 비율대로 섞는다 | 분류 |
| MixUp | 두 이미지와 라벨을 선형 보간한다 | 분류 |
| Mosaic | 이미지 4장을 한 장으로 붙인다 | Detection (YOLO 계열) |

`torchvision.transforms.v2` API를 쓰면 RandAugment와 정규화 파이프라인을 몇 줄로 깔끔하게 묶을 수 있다.

```python
import torch
import torchvision.transforms.v2 as T

# RandAugment
transform = T.Compose([
    T.RandAugment(num_ops=2, magnitude=9),
    T.ToImage(),                           # v2에서는 ToTensor 대신 ToImage + ToDtype
    T.ToDtype(torch.float32, scale=True),
    T.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
])
```

여기에 모델이 학습 데이터의 우연한 패턴까지 외워 버리지 않도록 **Regularization** 기법들을 더한다.

- **Label Smoothing**: 정답 1, 오답 0인 날카로운 hard label 대신, 정답 확률에서 `ε`(예: `0.1`)만큼을 덜어내 모든 클래스에 고르게 나누어 준 soft label을 쓴다. 모델의 과신을 억제하는 효과가 있으며, PyTorch에서는 `nn.CrossEntropyLoss(label_smoothing=0.1)`로 바로 적용한다.
- **Stochastic Depth**: 학습할 때 일부 residual 블록을 확률적으로 건너뛰어 앙상블과 비슷한 효과를 내고, 깊은 ResNet 계열의 overfitting을 줄인다.
- **Weight Decay**: optimizer에 `weight_decay=0.01~0.05`를 주어 가중치가 비대해지는 것을 억제한다. 특히 `AdamW`는 weight decay를 adaptive gradient 갱신 식에서 분리해(decoupled) 적용한다.

또한 Transformer 계열처럼 층이 깊고 attention 점수가 급변할 수 있는 구조를 학습할 때는 **Gradient Clipping**으로 기울기 벡터의 노름(norm)에 상한을 걸어 학습 중 가중치가 폭발하는 사고를 막는다.

```python
torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
```

학습을 돌려 놓고 로그를 지켜볼 때, loss 곡선이 그리는 모양만 잘 읽어도 문제의 원인과 처방을 빠르게 짚어 낼 수 있다.

| 패턴 | 진단 | 대응 |
|------|------|------|
| train loss 감소, val loss 증가 | Overfitting | augmentation 추가, dropout/weight decay 증가, 데이터 확보 |
| train loss가 높은 채로 멈춤 | Underfitting | 모델 크기 증가, learning rate 조정, augmentation 감소 |
| train loss가 심하게 출렁임 | Learning rate 과다 | learning rate 감소 |
| train loss NaN 발생 | Gradient explosion | gradient clipping, learning rate 대폭 감소, 데이터 검증 |
| val loss 초반에 급감 후 완전 정체 | Learning rate 부족 또는 스케줄 문제 | warm-up 추가, cosine schedule 적용 |

모델과 데이터셋 규모가 커져 GPU 한 장으로는 실험 한 번에 며칠씩 걸린다면 여러 GPU로 학습을 나누어야 한다. **DistributedDataParallel(DDP)**은 PyTorch의 기본 분산 학습 방식으로, 각 GPU에 모델을 복제해 올린 뒤 서로 다른 미니배치에서 계산한 gradient만 동기화한다.

```python
# DDP 최소 구조 (torchrun으로 실행)
import os
import torch
import torch.distributed as dist
from torch.nn.parallel import DistributedDataParallel as DDP

dist.init_process_group("nccl")
local_rank = int(os.environ["LOCAL_RANK"])
torch.cuda.set_device(local_rank)
model = model.to(local_rank)
model = DDP(model, device_ids=[local_rank])

# 실행: torchrun --nproc_per_node=4 train.py
```

> **추천 자료**
> - [Goyal et al., "Accurate, Large Minibatch SGD" (2017)](https://arxiv.org/abs/1706.02677) — 대규모 학습의 learning rate scaling rule
> - [PyTorch DDP 튜토리얼](https://docs.pytorch.org/tutorials/intermediate/ddp_tutorial.html) — 분산 학습 공식 가이드
> - [Wightman et al., "ResNet strikes back" (2021)](https://arxiv.org/abs/2110.00476) — ResNet-50 구조는 그대로 두고 학습 기법만 바꿔 ImageNet top-1 정확도를 80.4%까지 올렸다

> **실습**: [Data Augmentation 시각화](https://alexjunholee.github.io/robotics-practice/app.html#data_augmentation)
> RandAugment, CutMix, MixUp 같은 augmentation이 이미지를 어떻게 바꾸는지 직접 조작하며 본다.

> **실습**: [Learning Rate Schedule 시각화](https://alexjunholee.github.io/robotics-practice/app.html#lr_schedule)
> Cosine Annealing, OneCycleLR 같은 learning rate schedule의 곡선을 하이퍼파라미터를 바꿔 가며 비교한다.

---

## 10.9 자기지도 학습과 대조 학습 [심화]

로봇을 하루만 굴려도 카메라는 이미지를 수천, 수만 장씩 쏟아 내지만, 사람이 일일이 경계 상자와 픽셀 마스크를 그려 라벨을 붙이는 일에는 막대한 비용과 시간이 든다. **자기지도 학습(self-supervised learning)**은 사람이 붙인 라벨 없이도 방대한 이미지 데이터 자체에서 학습 신호를 만들어 내어 쓸모 있는 시각 표현을 미리 익히게 한다.

대표적인 갈래 중 하나인 **Contrastive Learning(대조 학습)**은 "같은 장면을 다르게 변형한 두 이미지는 임베딩 공간에서 서로 끌어당기고, 다른 이미지끼리는 밀어낸다"는 단순한 원리로 표현을 배운다. 같은 원본에 서로 다른 augmentation을 건 두 장을 **positive pair**, 다른 이미지와 짝지은 쌍을 **negative pair**라고 부른다.

- **SimCLR**: 한 이미지에서 변형된 이미지 두 장을 만들어 positive pair로 삼고, 같은 미니배치 안에 들어 있는 나머지 이미지들을 전부 negative로 쓴다. 풍부한 negative를 확보하려면 배치 크기를 아주 크게 잡아야 한다.
- **MoCo (Momentum Contrast)**: momentum encoder와 별도의 queue를 두어 지난 배치들의 임베딩을 계속 쌓아 두기 때문에, 작은 배치 크기에서도 수많은 negative와 비교할 수 있다.

대조 학습의 핵심 목적함수인 **InfoNCE Loss**는 이 끌어당김과 밀어냄을 소프트맥스 분류 문제처럼 수식으로 옮긴 형태다.

$$\mathcal{L} = -\log \frac{\exp(\text{sim}(z_i, z_j) / \tau)}{\sum_{k=1}^{2N} \mathbb{1}_{[k \neq i]} \exp(\text{sim}(z_i, z_k) / \tau)}$$

미니배치 이미지 `N`장에 augmentation을 두 번씩 걸면 총 `2N`개의 임베딩이 나온다. `z_i`와 `z_j`는 같은 원본 이미지에서 파생된 두 임베딩이고, `sim`은 코사인 유사도, `τ`는 분포의 날카로움을 조절하는 temperature다. 이 손실을 최소화하면 분자에 놓인 positive pair 유사도는 커지고, 분모에 합산된 수많은 negative pair 유사도는 작아진다.

대조 학습과 달리 negative pair 없이 입력의 일부를 가리고 복원하면서 표현을 배우는 방식도 있다. **MAE(Masked Autoencoders)**는 ViT 입력 패치의 75%를 무작위로 가린 뒤, 남은 25%의 조각만으로 가려진 영역의 픽셀을 복원하도록 학습한다. 자연어 처리의 BERT가 문장 속 빈칸을 맞히던 아이디어를 비전에 옮긴 셈이다. 이미지는 텍스트보다 인접 픽셀 사이의 정보 중복이 훨씬 심해, 조금만 가리면 주변 색깔을 베껴 쉽게 메워 버린다. 그래서 마스킹 비율을 75%까지 높여 과제를 어렵게 만들어야 비로소 물체의 구조적 표현을 배운다. 또한 원 논문에서는 무거운 인코더가 가리지 않은 25% 패치만 처리하도록 설계한 덕분에 사전학습 속도가 3배 이상 빨라졌다고 보고한다(다만 attention 연산 비용이 토큰 수에 따라 초선형으로 늘고 디코더 연산이 더해지므로, 실제 연산량 절감 비율이 마스킹 비율과 꼭 비례하지는 않는다).

오늘날 로보틱스 백본으로 각광받는 **DINOv2**는 또 다른 축인 **self-distillation** 계열에 뿌리를 두고 있다. 같은 구조의 teacher와 student 네트워크를 두고, teacher의 가중치가 지수 이동 평균(EMA, exponential moving average)으로 student 가중치를 천천히 따라가게 만든다. 이때 모든 입력에 똑같은 상수 벡터를 내뱉으며 학습이 무너지는 collapse 현상을 막기 위해, teacher 출력에는 낮은 temperature로 분포를 뾰족하게 만드는 **sharpening**과 편향을 누르는 **centering**을 건다(DINO v1은 teacher 출력에서 평균을 빼는 centering을, DINOv2는 그 대신 SwAV의 Sinkhorn-Knopp centering을 적용했다). DINOv2 논문에서는 이렇게 라벨 없이 학습한 backbone의 가중치를 통째로 고정(freeze)한 채, ImageNet 분류는 선형 head만 학습하는 linear probe와 학습 없는 k-NN으로, ADE20K 의미 분할은 linear probe의 mIoU로 평가했다. 그렇게 얻은 성능이 supervised 사전학습 모델에 견줄 만하다고 보고했다(결과를 인용할 때는 이처럼 평가 방식과 모델 크기를 함께 밝혀야 공정하다).

실무에서는 사전학습된 DINOv2 backbone을 HuggingFace에서 내려받아 가중치를 얼린 뒤, 내가 가진 소량의 데이터에 맞는 가벼운 분류 head만 학습해 빠르게 인식기를 만들 수 있다.

```python
from transformers import AutoModel, AutoImageProcessor
import torch.nn as nn

# DINOv2 backbone 로드
processor = AutoImageProcessor.from_pretrained("facebook/dinov2-base")
backbone = AutoModel.from_pretrained("facebook/dinov2-base")

# Backbone freeze 후 classification head만 학습
for param in backbone.parameters():
    param.requires_grad = False

class MyClassifier(nn.Module):
    def __init__(self, backbone, num_classes):
        super().__init__()
        self.backbone = backbone
        self.head = nn.Linear(768, num_classes)  # DINOv2-base dim = 768

    def forward(self, pixel_values):
        features = self.backbone(pixel_values).last_hidden_state[:, 0]  # CLS token
        return self.head(features)
```

> **추천 자료**
> - [Chen et al., "A Simple Framework for Contrastive Learning of Visual Representations (SimCLR)" (2020)](https://arxiv.org/abs/2002.05709) — Contrastive learning의 대표작
> - [He et al., "Masked Autoencoders Are Scalable Vision Learners" (2022)](https://arxiv.org/abs/2111.06377) — MAE 원논문
> - [Oquab et al., "DINOv2: Learning Robust Visual Features without Supervision" (TMLR 2024)](https://arxiv.org/abs/2304.07193) — DINOv2 원논문

---

## 10.10 Knowledge Distillation [심화]

자기지도 학습이나 대규모 사전학습으로 거대한 모델의 성능은 눈부시게 좋아졌지만, 수억 개가 넘는 파라미터를 실은 모델은 전력과 GPU 메모리가 빠듯한 로봇 온보드 컴퓨터에서 실시간으로 돌아가기 어렵다. **Knowledge distillation(지식 증류)**은 무거운 모델의 능력을 엣지 디바이스에 옮겨 담는 압축 기법으로, 이미 성능이 검증된 거대한 모델(**teacher**)이 내는 풍부한 출력 신호를 작고 빠른 모델(**student**)이 그대로 따라 하도록 가르친다. 가령 MobileSAM은 거대한 SAM의 image encoder를 훨씬 작은 encoder로 증류해 실행 속도를 극적으로 줄였다.

Teacher가 단순한 정답 라벨(`hard label`)보다 더 좋은 선생님이 되는 이유는 **soft prediction** 안에 담긴 클래스 간 유사도 정보 때문이다. 예를 들어 (고양이, 개, 자동차) 세 클래스를 구분할 때 고양이 사진의 hard label은 `[1, 0, 0]`인데, 정답만 1로 두고 나머지는 0으로 잘라 버린다. 반면 잘 학습된 teacher는 `[0.85, 0.10, 0.05]` 같은 확률 분포를 내놓아, "이 사진은 고양이이지만 자동차보다는 개와 훨씬 닮았다"는 미묘한 관계(dark knowledge)까지 student에게 들려준다.

이 오답 클래스들 사이의 작은 확률 차이까지 또렷이 전달하려면 logits를 temperature `τ`(`τ > 1`)로 나누어 분포를 완만하게 편 뒤 KL divergence로 차이를 잰다(아래 코드 예제는 `τ = 4`를 쓴다).

$$\mathcal{L}_{KD} = \text{KL}\left(\sigma\left(\frac{z_t}{\tau}\right) \| \sigma\left(\frac{z_s}{\tau}\right)\right)$$

여기서 `z_t`와 `z_s`는 각각 teacher와 student의 logits이고, `σ`는 softmax 함수다. 실제 학습에서는 이 distillation loss를 원래의 정답 라벨과 비교하는 hard label loss와 가중 합해 최적화한다.

$$\mathcal{L} = \alpha \cdot \mathcal{L}_{CE}(y, \sigma(z_s)) + (1 - \alpha) \cdot \tau^2 \cdot \mathcal{L}_{KD}$$

`y`는 정답 라벨, `α`는 두 손실의 비중을 정하는 가중치다. logit을 `τ`로 나누면서 soft target loss의 역전파 gradient 크기가 `1/τ^2`만큼 줄어들기 때문에, 이를 되살려 hard label loss와 균형을 맞추려고 두 번째 항에 `τ^2`을 곱해 준다.

최종 출력 logit뿐 아니라 중간 은닉층의 공간 특징까지 닮게 하고 싶다면 **Feature-based Distillation(FitNets)**을 쓴다.

$$\mathcal{L}_{feat} = \|f_t(x) - r(f_s(x))\|^2$$

`f_t(x)`와 `f_s(x)`는 같은 입력 `x`에 대해 teacher와 student의 중간 layer가 뽑아낸 feature map이며, `r`은 작은 student feature의 채널·차원 크기를 teacher에 맞추는 projection layer다. 덕분에 최종 분류기 직전에는 버려질 수 있는 풍부한 중간 공간 표현까지 student가 물려받을 수 있다.

최근에는 이 원리를 활용해 거대한 Vision Foundation Model(VFM)을 로봇에 올릴 수 있는 크기로 줄이는 연구가 활발하다.

| Teacher | Student | 방법 |
|---------|---------|------|
| SAM (ViT-H) | MobileSAM | Image encoder를 작은 ViT로 바꾸고 distillation |
| SAM | FastSAM | SAM의 데이터셋(SA-1B) 일부로 YOLOv8-seg를 학습 |
| DINOv2 ViT-g | DINOv2 ViT-S/B/L | 큰 모델을 같은 계열의 작은 ViT로 distillation |

PyTorch로 Soft target과 Hard target을 결합한 기본 distillation 손실 함수를 구현하면 다음과 같다.

```python
import torch
import torch.nn.functional as F

def distillation_loss(student_logits, teacher_logits, labels,
                      temperature=4.0, alpha=0.5):
    # Soft target loss (KL divergence)
    soft_loss = F.kl_div(
        F.log_softmax(student_logits / temperature, dim=-1),
        F.softmax(teacher_logits / temperature, dim=-1),
        reduction="batchmean"
    ) * (temperature ** 2)

    # Hard target loss
    hard_loss = F.cross_entropy(student_logits, labels)

    return alpha * hard_loss + (1 - alpha) * soft_loss
```

> **추천 자료**
> - [Hinton et al., "Distilling the Knowledge in a Neural Network" (2015)](https://arxiv.org/abs/1503.02531) — Knowledge distillation 원논문
> - [Zhang et al., "Faster Segment Anything (MobileSAM)" (2023)](https://arxiv.org/abs/2306.14289) — SAM을 distillation한 사례
> - [Romero et al., "FitNets: Hints for Thin Deep Nets" (2015)](https://arxiv.org/abs/1412.6550) — Feature-based distillation 원논문

---

## 10.11 도메인 적응 [심화]

경량화까지 마친 모델을 막상 실제 로봇에 올려 현장에 내보내면, 연구실 데이터셋에서 99%를 찍던 인식기가 햇빛이 비치는 창가나 처음 보는 공장 바닥 앞에서 맥없이 무너지는 일이 잦다. 시뮬레이션에서 학습한 모델을 실제 로봇에 옮겼을 때나 실내 데이터로 학습한 모델을 실외에 투입했을 때처럼, 학습 환경과 실제 배포 환경의 데이터 분포가 어긋나는 현상을 **domain shift**라 부른다. 로보틱스에서 부딪히는 sim-to-real gap(8장 §8.5)이 바로 이 문제의 전형이며, 이를 메우는 기술이 **domain adaptation(도메인 적응)**이다.

문제 구도는 다음과 같이 정리된다.

- **Source domain `D_s` (라벨 있음)**: 시뮬레이터 생성 데이터나 미리 구축된 학습 데이터셋
- **Target domain `D_t` (라벨 없음 또는 소량)**: 실제 로봇이 동작할 배포 환경 데이터
- **목표**: `D_s`에서 배운 모델이 분포가 다른 `D_t`에서도 성능을 잃지 않고 동작하게 만드는 것

현장에서는 쓸 수 있는 데이터와 배포 제약에 따라 크게 세 가지 접근법을 쓴다.

**Domain Randomization**:

시뮬레이터에서 합성 학습 데이터를 뽑을 때, 배경과 센서 조건을 현실과 똑같이 맞추려 애쓰는 대신 아예 환경 파라미터를 넓은 범위에서 무작위로 마구 흔들어 놓는다.

- **텍스처**: 벽, 바닥, 물체의 재질과 무늬를 에피소드마다 랜덤 교체
- **조명**: 광원의 위치, 색상, 강도를 랜덤 변경
- **카메라 파라미터**: focal length, 장착 위치, 각도에 노이즈 추가
- **물리 파라미터**: 마찰 계수, 질량, 관성 등을 넓은 범위 안에서 랜덤 설정

Tobin et al.(2017)은 시뮬레이션이 보여 주는 변종이 충분히 넓고 다양하다면, 모델 입장에서 **현실 세계 역시 그 수많은 변종 가운데 하나로 자연스럽게 포섭될 수 있다**는 통찰을 제시했다.

**Adversarial Domain Adaptation**:

실제 환경에서 라벨은 없더라도 이미지만 조금이라도 모을 수 있다면, feature extractor가 도메인 차이를 지운 표현(domain-invariant feature)을 배우도록 적대적 학습을 걸 수 있다. 구조는 주 과제를 푸는 분류기 옆에 "이 특징이 시뮬레이션(`D_s`)에서 왔는지 현실(`D_t`)에서 왔는지" 맞히는 **domain discriminator**를 하나 더 달아 놓는 형태다.

```
Input --> Feature Extractor --> [Task Classifier]      --> Task Loss
                              \--> [Domain Discriminator] --> Domain Loss (GRL)
```

여기서 핵심 장치는 두 네트워크 사이에 끼운 **Gradient Reversal Layer(GRL)**다. 순전파 때는 입력을 그대로 통과시키지만, 역전파로 돌아오는 gradient의 부호만 반대로 뒤집는다. 그 결과 task classifier는 라벨이 있는 `D_s` 데이터로 원래 과제를 충실히 학습하는 동시에, feature extractor는 domain discriminator가 `D_s`와 `D_t`를 전혀 구별하지 못하게 만드는 쪽으로 가중치를 고친다.

$$\mathcal{L} = \mathcal{L}_{task}(D_s) - \lambda \cdot \mathcal{L}_{domain}(D_s, D_t)$$

`λ`는 두 손실의 균형을 잡는 가중치다. 식의 마이너스 부호가 뜻하듯 feature extractor는 `L_domain`을 키우는 방향으로, discriminator는 같은 `L_domain`을 줄이는 방향으로 맞서며 마치 GAN처럼 미니맥스 게임을 벌인다. 결국 과제 수행에는 유용하면서도 도메인 색채가 빠진 표현만 살아남는다.

**Test-Time Adaptation (TTA)**:

모델을 이미 로봇에 실어 배포한 뒤에도 조명이 어두워지거나 비가 오는 등 환경이 계속 바뀐다면 **Test-Time Adaptation(TTA)**을 고려할 수 있다. TTA는 학습 데이터를 다시 꺼내지 않고 추론 중에 들어오는 테스트 배치만으로 모델을 조금씩 조율하는 기법이다. 대표적으로 **TENT**는 모델이 타겟 환경에서 내놓는 예측 분포가 자신감 있고 또렷해지도록(즉 prediction entropy가 줄어들도록) Batch Normalization 층의 affine 파라미터(`weight`, `bias`)만 실시간으로 업데이트한다. **CoTTA**는 이를 확장해 분포가 시간에 따라 끊임없이 변하는 연속(continual) 환경을 다룬다.

```python
# TENT 업데이트 과정 (간략화)
model.eval()
model.requires_grad_(False)          # 먼저 전체를 얼린다
params = []
for m in model.modules():
    if isinstance(m, nn.BatchNorm2d):
        m.requires_grad_(True)       # BN의 affine parameter만 다시 켠다
        m.track_running_stats = False
        m.running_mean = None        # eval 모드에서 batch statistics를 쓰려면 running 버퍼를 비운다
        m.running_var = None
        params += [m.weight, m.bias]

optimizer = optim.SGD(params, lr=1e-4)  # BN 파라미터만 최적화

# 추론 시 adaptation
for batch in test_loader:
    output = model(batch)
    loss = entropy(output)  # prediction entropy 최소화
    loss.backward()
    optimizer.step()
    optimizer.zero_grad()
```

**Sim-to-Real Gap과의 연결**:

실제 프로젝트에서는 목적과 쓸 수 있는 자원, 안전 요구 수준에 따라 이 기법들을 적절히 조합해 쓴다. 전형적인 파이프라인을 그려 보자면 다음과 같다.

1. 시뮬레이터에서 **domain randomization**을 걸어 다양한 합성 데이터를 대량으로 만든다.
2. 실제 로봇 환경에서 소량의 데이터를 모아 **adversarial adaptation**으로 분포 차이를 줄인다.
3. 현장에 배포한 뒤에는 **TTA**로 시시각각 변하는 환경 조건을 따라간다.

물론 세 단계를 늘 모두 묶어 써야 하는 것은 아니다. domain randomization만으로 충분해 곧장 실제 소량 데이터로 파인튜닝을 더하기도 하고, 안전 검증이 엄격한 로봇에서는 배포 중에 가중치가 변하는 것을 막으려고 TTA를 쓰지 않기도 한다. 결국 안전 제약과 온보드 계산 예산을 함께 따져 알맞은 조합을 고른다.

여기까지 10장에서는 딥러닝 인식 파이프라인을 다루었다. 이 파이프라인에서는 특정 태스크와 데이터셋을 중심으로 모델을 학습시켜 분류, 탐지, 분할, 단안 깊이 추정을 하고, 그 모델을 경량화해 현장 환경에 적응시킨다. 하지만 작업 현장에 새 물체가 나타날 때마다 라벨을 새로 모으고 모델을 다시 학습시키는 데는 한계가 있다. 다음 11장에서는 **비전 파운데이션 모델(VFM)**로 시야를 넓혀 본다. VFM은 인터넷 규모의 방대한 데이터로 사전학습해, 추가 학습 없이도(zero-shot) 새로운 물체와 장면을 곧바로 인식하고 분할한다.

> **추천 자료**
> - [Tobin et al., "Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World" (2017)](https://arxiv.org/abs/1703.06907) — Domain randomization 원논문
> - [Ganin et al., "Domain-Adversarial Training of Neural Networks" (2016)](https://arxiv.org/abs/1505.07818) — GRL을 쓰는 adversarial domain adaptation 논문 (JMLR)
> - [Wang et al., "TENT: Fully Test-Time Adaptation by Entropy Minimization" (2021)](https://arxiv.org/abs/2006.10726) — TENT 원논문 (ICLR 2021)
> - [Wen et al., "FoundationPose: Unified 6D Pose Estimation and Tracking of Novel Objects" (CVPR 2024, arXiv:2312.08344)](https://arxiv.org/abs/2312.08344) — 처음 보는 물체의 6D 포즈를 추정하고 추적한다. CAD 모델이나 참조 이미지 몇 장만 있으면 된다

---

> **기술 흐름: 딥러닝 기반 인식 (Deep Learning for Perception)**
> - 2012: AlexNet이 ILSVRC 2012 분류 부문에서 top-5 오류율 15.3%로 2위(26.2%)를 크게 앞서 우승했다. 이후 손으로 설계한 특징 대신 학습한 특징을 쓰는 연구가 늘었다.
> - 2014~2016: VGGNet, GoogLeNet, ResNet이 나왔다. ResNet (CVPR 2016)은 residual connection으로 152층 네트워크를 학습했다. Faster R-CNN (2015)과 YOLO (CVPR 2016)가 나오면서 object detection이 빨라졌다.
> - 2017: "Attention Is All You Need"가 Transformer를 발표했다. 처음에는 기계 번역용이었다.
> - 2020~2021: DETR (2020)이 detection에 Transformer를 썼다. ViT (ICLR 2021)는 이미지를 패치 시퀀스로 처리했다. Swin Transformer (2021) 원 논문은 당시 COCO와 ADE20K에서 비교 모델보다 높은 점수를 보고했다.
> - 2022~2023: ConvNeXt (2022)는 CNN도 ViT와 비슷한 성능을 낼 수 있음을 보였다. Segment Anything (SAM, 2023)은 프롬프트로 가리킨 물체를 종류와 상관없이 분할한다. DINOv2 (arXiv 2023, TMLR 2024) 논문은 backbone을 고정한 채 분류·의미 분할(§10.9)에 더해 깊이 추정도 평가했다.
