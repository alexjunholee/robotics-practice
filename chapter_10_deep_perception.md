# Ch.10 — 딥러닝 기반 인식 (Deep Learning for Perception)

로봇이 "무엇을 보고 있는지"를 이해하는 핵심 기술들이다. 고전 CV가 "이미지를 어떻게 처리하고, 기하학적 관계를 어떻게 추출하는가"에 집중했다면, 여기서는 "이미지 안에 뭐가 있는지"를 인식하는 데 집중한다. 물체 탐지, 분류, 분할 — 로봇이 "저기 빨간 컵이 있으니 집어" 같은 명령을 수행하려면, 이 기술들이 필수이다.

---

## 10.1 프레임워크 선택

어떤 딥러닝 프레임워크를 쓸지는 생각보다 중요한 결정이다. 연구 코드를 읽고 재현하려면 그 코드가 쓰는 프레임워크를 알아야 하고, 자기 모델을 만들려면 하나는 제대로 알아야 한다.

### 10.1.1 PyTorch (권장)

**장점**:
- 직관적인 동적 그래프 (eager execution)
- 디버깅 용이
- 연구 커뮤니티에서 표준
- 풍부한 사전학습 모델 (torchvision, timm)

현실적인 이유가 있다. 2024년 기준 NeurIPS, CVPR, ICLR 등 주요 학회에서 발표되는 논문의 코드 공개 중 80% 이상이 PyTorch이다. 최신 논문을 읽고 코드를 돌려보고 수정하려면, PyTorch를 모르면 사실상 진행이 안 된다.

**설치**:

```bash
# CUDA 12.1 버전
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121
```

**기본 사용**:

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
> - [PyTorch 공식 튜토리얼](https://pytorch.org/tutorials/) — 입문부터 고급까지 체계적으로 정리되어 있다
> - [d2l.ai (Dive into Deep Learning)](https://d2l.ai/) — 인터랙티브 교과서. PyTorch 코드와 수학이 함께 나온다
> - [Andrej Karpathy — Neural Networks: Zero to Hero](https://www.youtube.com/playlist?list=PLAqhIrjkxbuWI23v9cThsA9GvCAUhRvKZ) — 전 Tesla AI Director가 신경망을 밑바닥부터 설명
> - [Jaejun Yoo's Playground](http://jaejunyoo.blogspot.com/search/label/kr) — 한국어로 GAN, VAE 등 생성 모델을 잘 설명한 블로그

### 10.1.2 TensorFlow / JAX

**TensorFlow**: 프로덕션 배포에 강점, TF Lite 모바일 지원
**JAX**: 고성능 연산, 함수형 프로그래밍, 연구용

대부분의 최신 연구 코드가 PyTorch로 공개되므로, PyTorch를 먼저 배우길 권장한다.

단, TensorFlow Lite는 로봇의 엣지 디바이스(Jetson, 라즈베리파이 등)에 모델을 배포할 때 여전히 많이 쓰이고, JAX는 Google DeepMind 계열 연구에서 많이 사용되므로 존재는 알아두자.

> **추천 자료**
> - [TensorFlow 공식 가이드](https://www.tensorflow.org/guide) — TFLite 변환까지 커버
> - [JAX 공식 문서](https://jax.readthedocs.io/) — 함수형 딥러닝 프레임워크

---

## 10.2 딥러닝 기초 개념

이 섹션의 개념들을 모르면 "왜 딥러닝이 이미지 인식에서 고전 방법을 압도하는가"를 이해할 수 없다. CNN의 구조를 모르면 ResNet이 왜 중요한지 모르고, Transformer를 모르면 ViT와 DETR이 왜 기존 접근을 대체하는지 이해할 수 없다.

### 10.2.1 CNN (Convolutional Neural Network)

이미지에서 공간적 특징을 추출하는 핵심 구조이다.

CNN은 "이미지의 지역적 패턴(에지, 코너, 텍스처)을 자동으로 학습"하는 구조이다. 앞서 배운 고전 CV에서는 SIFT, ORB 같은 특징을 사람이 설계(hand-craft)했지만, CNN은 데이터에서 자동으로 최적의 특징을 배운다. 딥러닝의 전환점이 여기다.

**주요 구성 요소**:
- **Convolution Layer**: 필터로 특징 추출
- **Pooling Layer**: 공간 크기 축소 (Max, Average)
- **Activation**: nonlinearity 도입 (ReLU, GELU)
- **Batch Normalization**: 학습 안정화

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

Convolution은 선형대수적으로 보면 "필터(kernel)와 이미지 패치의 내적"이다. 수업에서 배운 행렬 곱이 여기서 바로 쓰인다. `kernel_size=3, padding=1`이면 출력 크기가 입력과 같게 유지된다 — 이 패턴은 매우 자주 나온다.

> **추천 자료**
> - [Stanford CS231n — Convolutional Neural Networks for Visual Recognition](https://www.youtube.com/playlist?list=PLoROMvodv4rMFqRtEuo6SGjY4XbRIVRd4) — CNN을 이해하기 위한 대표 강의. 보는 걸 권한다
> - [d2l.ai — CNN 챕터](https://d2l.ai/chapter_convolutional-neural-networks/index.html) — 코드와 수학 설명이 동시에
> - [3Blue1Brown — But what is a Neural Network?](https://www.youtube.com/watch?v=aircAruvnKk) — 신경망의 직관적 이해

### 10.2.2 Attention & Transformer

**Self-Attention**: 시퀀스 내 모든 위치 간의 관계를 학습

```
Attention(Q, K, V) = softmax(QK^T / √d_k) V
```

CNN과의 차이를 알면 왜 Transformer가 뜨는지 바로 이해된다. CNN은 "가까운 픽셀끼리만" 정보를 교환하지만 (local receptive field), Transformer의 Self-Attention은 "이미지의 어떤 부분이든 서로 참조"할 수 있다 (global attention). 이 덕분에 물체의 전체적인 맥락을 파악하는 데 유리하다. 2020년 이후 비전 분야에서 Transformer가 CNN을 빠르게 대체하는 추세이다.

**Vision Transformer (ViT)**는 이미지를 16×16 같은 고정 크기 패치로 나누고, 각 패치를 NLP에서 "단어"처럼 취급해서 Transformer encoder에 넣는다. 아이디어 자체는 단순하지만, 대규모 데이터에서 CNN을 넘어서는 성능을 보여주면서 최근 비전 태스크의 주력이 되었다.

> **추천 자료**
> - [Vaswani et al., "Attention Is All You Need" (2017)](https://arxiv.org/abs/1706.03762) — Transformer 원논문. 모든 것의 시작
> - [Dosovitskiy et al., "An Image is Worth 16x16 Words" (2020)](https://arxiv.org/abs/2010.11929) — ViT 원논문
> - [Yannic Kilcher — Vision Transformer 설명](https://www.youtube.com/watch?v=TrdevFK_am4) — 논문을 쉽게 풀어서 설명
> - [Andrej Karpathy — Let's build GPT from scratch](https://www.youtube.com/watch?v=kCc8FmEb1nY) — Transformer 구현을 밑바닥부터. NLP지만 ViT 이해에 직결된다

---

## 10.3 Image Classification

분류(classification)는 "이 이미지에 뭐가 있는가?"라는 가장 기본적인 질문이다. 복잡한 detection, segmentation 모델도 내부적으로 분류기를 포함하고 있으므로, 분류 모델을 이해하는 것이 기본 중의 기본이다. 사전학습된 분류 모델의 backbone (ResNet, ViT 등)은 다른 태스크의 feature extractor로 널리 쓰인다.

**대표 모델**:

| 모델 | 특징 | 용도 |
| --- | --- | --- |
| ResNet | Residual connection, 안정적 학습 | 백본 네트워크 |
| EfficientNet | Compound scaling, 효율적 | 모바일, 효율성 중시 |
| ViT | Transformer 기반 | 대규모 데이터, 고성능 |
| ConvNeXt | CNN의 현대화 | ViT와 경쟁 |

ResNet의 Residual Connection은 "입력을 출력에 더해주는" 단순한 아이디어인데, 이 하나가 수십 층 깊은 네트워크의 학습을 가능하게 만들었다. 2015년 발표 이후 거의 모든 딥러닝 아키텍처의 기본 빌딩 블록이 되었다.

**사전학습 모델 사용**:

```python
import torchvision.models as models

# Pretrained ResNet50
model = models.resnet50(weights='IMAGENET1K_V2')

# Feature extractor로 사용
model.fc = nn.Identity()  # 마지막 FC 제거
features = model(x)  # (batch, 2048)
```

이 패턴은 매우 자주 쓰인다. ImageNet으로 사전학습된 모델의 마지막 분류 레이어만 제거하고, 그 전까지의 출력을 "feature"로 사용하는 것이다. 이를 transfer learning이라 하고, 로보틱스에서 새로운 물체를 인식시킬 때 거의 항상 이 방식을 쓴다.

> **추천 자료**
> - [He et al., "Deep Residual Learning for Image Recognition" (2015)](https://arxiv.org/abs/1512.03385) — ResNet 원논문. 딥러닝 역사상 가장 많이 인용된 논문 중 하나
> - [Papers With Code — Image Classification](https://paperswithcode.com/task/image-classification) — 최신 벤치마크와 SOTA 모델 확인
> - [timm (PyTorch Image Models) 라이브러리](https://github.com/huggingface/pytorch-image-models) — 수백 개의 사전학습 모델을 한 줄로 로드. 실무에서 매우 유용
> - [Stanford CS231n — Training Neural Networks](https://www.youtube.com/playlist?list=PLoROMvodv4rMFqRtEuo6SGjY4XbRIVRd4) — 학습 기법과 트릭

---

## 10.4 Object Detection

로봇이 "저 테이블 위에 컵이 어디 있지?"를 알려면, 단순 분류가 아니라 "어디에 뭐가 있는지"를 알아야 한다. 그것이 object detection이다. Bounding box로 물체의 위치와 클래스를 동시에 예측하는 태스크이며, 로봇 manipulation, 자율주행 등 거의 모든 응용에서 쓰인다.

### 10.4.1 Two-Stage Detectors

**Faster R-CNN**:
1. Region Proposal Network (RPN): 후보 영역 제안
2. ROI Pooling: 각 영역에서 특징 추출
3. Classification + Bounding Box Regression

장점: 높은 정확도
단점: 느린 속도

Faster R-CNN은 two-stage detection의 대표작이고, 정확도가 중요한 경우(예: 산업용 검사)에서 여전히 쓰인다. "먼저 후보를 뽑고, 그 후보를 정밀 분석한다"는 구조는 직관적이고, 이후 Mask R-CNN 등으로 이어졌다.

### 10.4.2 One-Stage Detectors

**YOLO (You Only Look Once)**:
- 이미지를 그리드로 나누고 한 번에 예측
- 실시간 처리 가능 (30+ FPS)
- 버전: YOLOv5, YOLOv8, YOLOv11 (Ultralytics)

YOLO는 이름 그대로 "한 번만 본다" — two-stage처럼 후보를 먼저 뽑지 않고, 이미지 전체를 한 번에 처리해서 모든 물체를 탐지한다. 로봇 시스템에서 실시간성이 중요할 때 가장 먼저 고려하는 모델이다. Ultralytics의 YOLOv8/v11은 설치와 사용이 매우 간편해서 프로토타이핑에 최적이다.

```python
from ultralytics import YOLO

# 모델 로드 및 추론
model = YOLO('yolov8n.pt')  # nano 모델
results = model('image.jpg')

# 결과 시각화
results[0].show()
```

**SSD (Single Shot Detector)**:
- 다양한 스케일의 feature map에서 예측
- YOLO보다 작은 객체 탐지에 유리

> **추천 자료**
> - [Redmon et al., "You Only Look Once: Unified, Real-Time Object Detection" (2016)](https://arxiv.org/abs/1506.02640) — YOLO 원논문. 간결하고 읽기 좋다
> - [Ultralytics YOLOv8 문서](https://docs.ultralytics.com/) — 설치부터 커스텀 학습까지 잘 정리
> - [Papers With Code — Object Detection](https://paperswithcode.com/task/object-detection) — 최신 SOTA 확인
> - [다크 프로그래머 — precision, recall의 이해](https://darkpgmr.tistory.com/162) — detection 평가 지표를 직관적으로 설명

### 10.4.3 Transformer-based

**DETR (Detection Transformer)**은 detection을 "집합 예측 문제"로 재정의했다. Object Query라는 고정 개수의 학습 가능한 벡터가 각 물체에 대응하고, NMS 없이 end-to-end로 학습한다. 기존 방법들이 수천 개의 anchor box를 만들고 NMS로 중복을 제거하는 복잡한 파이프라인을 썼던 것과 대조적이다. 초기 학습이 느리다는 단점이 있었지만, 구조가 깔끔해서 Deformable DETR, DINO, Co-DETR 등 많은 후속 연구로 이어졌다.

> **추천 자료**
> - [Carion et al., "End-to-End Object Detection with Transformers" (2020)](https://arxiv.org/abs/2005.12872) — DETR 원논문
> - [Yannic Kilcher — DETR 설명](https://www.youtube.com/watch?v=T35ba_VXkMY) — 논문을 잘 풀어서 설명
> - [HuggingFace — Object Detection 가이드](https://huggingface.co/docs/transformers/tasks/object_detection) — Transformers 라이브러리로 DETR 사용하기
> - [Zhao et al., "DETRs Beat YOLOs on Real-time Object Detection" (RT-DETR, CVPR 2024, arXiv:2304.08069)](https://arxiv.org/abs/2304.08069) — DETR 계열 최초로 YOLO급 실시간 속도 달성. 실시간 detection의 새로운 방향
> - [Cheng et al., "YOLO-World: Real-Time Open-Vocabulary Object Detection" (CVPR 2024, arXiv:2401.17270)](https://arxiv.org/abs/2401.17270) — YOLO에 텍스트 프롬프트 기반 open-vocabulary detection 추가. 로보틱스에서 임의 물체 탐지에 실용적

---

## 10.5 Semantic Segmentation

픽셀 단위로 클래스를 예측하는 태스크이다.

로봇 manipulation을 생각하면 차이가 바로 보인다. detection은 bounding box로 대략적인 위치만 알려주지만, segmentation은 물체의 정확한 경계를 알려준다. 로봇이 물체를 잡으려면 bounding box가 아니라 정확한 윤곽이 필요하고, 자율주행에서 도로/인도/차선을 구분하려면 픽셀 단위의 분류가 필수이다.

**대표 모델**:

| 모델 | 특징 |
| --- | --- |
| FCN | 최초의 end-to-end segmentation |
| U-Net | Encoder-Decoder 구조, 의료 영상에서 시작 |
| DeepLab v3+ | Atrous convolution, 다중 스케일 |
| SegFormer | Transformer 기반, 경량 디코더 |

U-Net의 Encoder-Decoder + Skip Connection 구조는 segmentation의 기본 패턴이 되었다. Encoder에서 특징을 추출하면서 해상도를 줄이고, Decoder에서 다시 해상도를 복원하면서 Skip Connection으로 세부 정보를 보충한다. 이 패턴은 depth estimation, image generation 등 다른 태스크에서도 널리 쓰인다.

```python
# Segmentation 모델 사용 (transformers 라이브러리)
from transformers import SegformerForSemanticSegmentation

model = SegformerForSemanticSegmentation.from_pretrained(
    "nvidia/segformer-b0-finetuned-ade-512-512"
)
```

> **추천 자료**
> - [Papers With Code — Semantic Segmentation](https://paperswithcode.com/task/semantic-segmentation) — 최신 벤치마크
> - [HuggingFace — Image Segmentation](https://huggingface.co/docs/transformers/tasks/semantic_segmentation) — SegFormer 등 사용법
> - [Two Minute Papers — Semantic Segmentation 관련 영상들](https://www.youtube.com/@TwoMinutePapers) — 최신 연구를 2분으로 요약

---

## 10.6 Instance & Panoptic Segmentation

**Instance Segmentation**: 각 객체 인스턴스를 구분
- Mask R-CNN: Faster R-CNN + Mask 브랜치

**Panoptic Segmentation**: Semantic + Instance 통합
- "Things" (객체): 인스턴스 구분
- "Stuff" (배경): 인스턴스 구분 없음

한 단계 더 생각해 보면, semantic segmentation은 "여기가 의자 영역"이라고만 알려주지, "의자가 3개 있는데 각각 어디까지인지"는 구분하지 못한다. 로봇이 "왼쪽 의자를 집어"라는 명령을 수행하려면, instance segmentation이 필요하다. Panoptic segmentation은 이 둘을 통합한 것으로, 장면 전체를 완전하게 이해하는 데 쓰인다.

> **추천 자료**
> - [He et al., "Mask R-CNN" (2017)](https://arxiv.org/abs/1703.06870) — Instance segmentation의 대표작
> - [Detectron2](https://github.com/facebookresearch/detectron2) — Meta의 detection/segmentation 프레임워크. Mask R-CNN 등을 쉽게 사용 가능

---

## 10.7 Depth Estimation

단일 이미지에서 깊이를 예측하는 태스크이다.

스테레오 카메라나 LiDAR 없이 단안(monocular) 카메라 하나로 깊이 정보를 얻을 수 있다면, 하드웨어 비용과 무게를 크게 줄일 수 있다. 특히 드론이나 소형 로봇처럼 payload가 제한적인 시스템에서 매우 유용하다. 최근 foundation model 수준의 일반화 성능을 보여주는 모델들이 나오면서 실용성이 크게 높아졌다.

**대표 모델**:
- **MiDaS**: 다양한 데이터셋 학습, 범용성
- **Depth Anything**: Foundation model 수준의 일반화
- **ZoeDepth**: 메트릭 깊이 추정

```python
# Depth Anything 사용
from transformers import pipeline

pipe = pipeline("depth-estimation", model="LiheYoung/depth-anything-base-hf")
result = pipe("image.jpg")
depth = result['depth']
```

주의할 점: MiDaS와 Depth Anything은 기본적으로 **상대적 깊이(relative depth)**를 추정한다. 즉, "A가 B보다 가깝다"는 알 수 있지만, "A까지 정확히 몇 미터"인지는 알 수 없다. 메트릭 깊이가 필요하면 ZoeDepth나 Depth Anything V2의 metric 버전을 사용해야 한다.

> **추천 자료**
> - [Yang et al., "Depth Anything: Unleashing the Power of Large-Scale Unlabeled Data" (2024)](https://arxiv.org/abs/2401.10891) — Depth Anything 원논문
> - [Godard et al., "Digging Into Self-Supervised Monocular Depth Estimation" (Monodepth2, ICCV 2019, arXiv:1806.01260)](https://arxiv.org/abs/1806.01260) — self-supervised depth의 기준선
> - [HuggingFace — Monocular Depth Estimation](https://huggingface.co/docs/transformers/tasks/monocular_depth_estimation) — 바로 돌려볼 수 있는 코드
> - [Papers With Code — Monocular Depth Estimation](https://paperswithcode.com/task/monocular-depth-estimation) — 최신 벤치마크 확인

---

## 10.8 심화: 학습 레시피

*연구자가 되고 싶다면 여기서부터 읽어라.*

모델 아키텍처만 알면 연구할 수 있을 것 같지만, 실제로 "학습이 잘 되게 하는 것"이 전체 연구 시간의 절반 이상을 차지한다. 같은 모델이라도 learning rate 하나 잘못 잡으면 수렴하지 않고, augmentation 하나 추가하면 정확도가 몇 % 뛸 수 있다. 이 절에서는 실무에서 반복적으로 쓰이는 학습 테크닉을 정리한다.

**Learning Rate Schedule**:

- **Cosine Annealing with Warm-up**: 가장 널리 쓰이는 스케줄. 초기 몇 epoch 동안 learning rate를 0에서 목표값까지 선형으로 올리고(warm-up), 이후 cosine 곡선으로 감쇠한다.

$$\eta_t = \eta_{min} + \frac{1}{2}(\eta_{max} - \eta_{min})\left(1 + \cos\left(\frac{t \cdot \pi}{T}\right)\right)$$

- **OneCycleLR**: learning rate를 한 번 올렸다가 내리는 정책. Super-convergence를 달성할 수 있어서 적은 epoch으로 빠르게 수렴한다.

```python
import torch.optim as optim

# Cosine Annealing with Warm-up (manual)
optimizer = optim.AdamW(model.parameters(), lr=1e-4, weight_decay=0.05)
scheduler = optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=100)

# OneCycleLR
scheduler = optim.lr_scheduler.OneCycleLR(
    optimizer, max_lr=1e-3, total_steps=len(dataloader) * num_epochs,
    pct_start=0.1  # 처음 10%를 warm-up에 사용
)
```

**Data Augmentation**:

| 기법 | 설명 | 주 용도 |
|------|------|---------|
| **RandAugment** | N개의 변환을 magnitude M으로 랜덤 적용 | 분류 전반 |
| **CutMix** | 이미지 영역을 다른 이미지로 대체, 라벨도 비율 혼합 | 분류 |
| **MixUp** | 두 이미지와 라벨을 선형 보간 | 분류 |
| **Mosaic** | 4개 이미지를 하나로 합성 | Detection (YOLO 계열) |

```python
import torchvision.transforms.v2 as T

# RandAugment
transform = T.Compose([
    T.RandAugment(num_ops=2, magnitude=9),
    T.ToTensor(),
    T.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
])
```

**Regularization**:

- **Label Smoothing**: hard label (0 또는 1) 대신 soft label (0.1, 0.9 등)을 사용. overconfidence를 방지한다. `nn.CrossEntropyLoss(label_smoothing=0.1)`
- **Stochastic Depth**: 학습 시 일부 layer를 랜덤으로 건너뛴다. ResNet 계열에서 overfitting 방지에 효과적이다.
- **Weight Decay**: optimizer에서 `weight_decay=0.01~0.05` 설정. AdamW에서는 decoupled weight decay를 사용한다.

**Gradient Clipping**: gradient가 폭발하는 것을 방지한다. Transformer 학습에서는 거의 필수이다.

```python
torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
```

**Loss Curve로 문제 진단**:

| 패턴 | 진단 | 대응 |
|------|------|------|
| train loss 감소, val loss 증가 | Overfitting | augmentation 추가, dropout/weight decay 증가, 데이터 확보 |
| train loss 높은 상태 정체 | Underfitting | 모델 크기 증가, learning rate 조정, augmentation 감소 |
| train loss 진동이 심함 | Learning rate 과다 | learning rate 감소 |
| train loss NaN 발생 | Gradient explosion | gradient clipping, learning rate 대폭 감소, 데이터 검증 |
| val loss 초반에 급감 후 완전 정체 | Learning rate 부족 또는 스케줄 문제 | warm-up 추가, cosine schedule 적용 |

**Distributed Training — PyTorch DDP 기본**:

모델이 커지면 GPU 1개로는 시간이 부족하다. DistributedDataParallel (DDP)은 여러 GPU에 모델을 복제하고 gradient를 동기화하는 가장 기본적인 병렬 학습 방법이다.

```python
# DDP 최소 구조 (torchrun으로 실행)
import torch.distributed as dist
from torch.nn.parallel import DistributedDataParallel as DDP

dist.init_process_group("nccl")
local_rank = int(os.environ["LOCAL_RANK"])
model = model.to(local_rank)
model = DDP(model, device_ids=[local_rank])

# 실행: torchrun --nproc_per_node=4 train.py
```

> **추천 자료**
> - [Goyal et al., "Accurate, Large Minibatch SGD" (2017)](https://arxiv.org/abs/1706.02677) — 대규모 학습의 learning rate scaling rule
> - [PyTorch DDP 튜토리얼](https://pytorch.org/tutorials/intermediate/ddp_tutorial.html) — 분산 학습 공식 가이드
> - [Wightman et al., "ResNet strikes back" (2021)](https://arxiv.org/abs/2110.00476) — 학습 레시피의 중요성을 보여주는 논문. 같은 ResNet으로 학습 기법만 바꿔 정확도를 크게 향상

> **실습**: [Data Augmentation 시각화](https://alexjunholee.github.io/robotics-practice/app.html#data_augmentation)
> RandAugment, CutMix, MixUp 등 다양한 augmentation 기법이 이미지를 어떻게 변형하는지 인터랙티브하게 확인할 수 있다.

> **실습**: [Learning Rate Schedule 시각화](https://alexjunholee.github.io/robotics-practice/app.html#lr_schedule)
> Cosine Annealing, OneCycleLR 등 다양한 learning rate schedule의 곡선을 비교하며 하이퍼파라미터의 영향을 확인할 수 있다.

---

## 10.9 심화: 자기지도 학습과 대조 학습

*연구자가 되고 싶다면 여기서부터 읽어라.*

로보틱스 데이터는 라벨이 부족하다. 로봇이 수집하는 이미지는 수천, 수만 장이지만, 이것에 일일이 바운딩 박스나 세그멘테이션 마스크를 다는 것은 비현실적이다. 자기지도 학습(self-supervised learning)은 라벨 없이 데이터 자체에서 학습 신호를 만들어내는 방법이다.

**Contrastive Learning**:

핵심 아이디어는 단순하다: 같은 이미지의 다른 augmentation은 가깝게(positive pair), 다른 이미지는 멀게(negative pair) 임베딩 공간에 배치한다.

- **SimCLR**: 같은 이미지에 서로 다른 augmentation을 적용하여 positive pair를 만든다. 배치 내 다른 이미지들이 negative pair. 큰 배치 사이즈가 필요하다.
- **MoCo (Momentum Contrast)**: momentum encoder와 queue를 사용해서 큰 배치 없이도 많은 negative를 확보한다.

**InfoNCE Loss**:

$$\mathcal{L} = -\log \frac{\exp(\text{sim}(z_i, z_j) / \tau)}{\sum_{k=1}^{2N} \mathbb{1}_{[k \neq i]} \exp(\text{sim}(z_i, z_k) / \tau)}$$

여기서 sim은 코사인 유사도, τ는 temperature이다. 분자는 positive pair의 유사도를 크게 하고, 분모는 negative pair와 구분하도록 학습한다.

**Masked Image Modeling — MAE**:

ViT 기반으로, 이미지 패치의 75%를 랜덤 마스킹하고 나머지 25%에서 마스킹된 부분을 복원하는 방식이다. NLP의 BERT가 단어를 마스킹하고 복원하는 것과 같은 원리이다.

- 마스킹 비율이 75%나 되는 이유: 이미지는 텍스트보다 redundancy가 크기 때문에, 높은 마스킹 비율이 더 어려운 과제를 만들어 좋은 표현을 학습시킨다.
- 인코더는 visible 패치만 처리하므로 학습이 효율적이다 (75% 연산량 감소).

**DINOv2와의 연결**:

DINOv2는 self-distillation 방식으로 학습한다. Teacher-student 구조이되, teacher는 student의 EMA(exponential moving average)이다.

- **Self-distillation**: student와 teacher가 같은 구조. teacher의 weight는 student weight의 EMA.
- **Centering + Sharpening**: teacher output에 centering(평균 빼기)과 sharpening(낮은 temperature)을 적용하여 mode collapse를 방지한다.
- 결과물인 DINOv2 feature는 별도 학습 없이도 ImageNet k-NN 83.0%, ADE20K linear probe 82.0% 등 supervised 방법에 필적하는 성능을 보인다.

**실무 — HuggingFace에서 self-supervised backbone fine-tune**:

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
> - [Oquab et al., "DINOv2: Learning Robust Visual Features without Supervision" (2024)](https://arxiv.org/abs/2304.07193) — DINOv2 원논문

---

## 10.10 심화: Knowledge Distillation

*연구자가 되고 싶다면 여기서부터 읽어라.*

큰 모델(teacher)의 "지식"을 작은 모델(student)에 전달하는 기법이다. 로보틱스에서 특히 중요한 이유는, VFM 같은 거대 모델을 edge device에서 돌려야 하기 때문이다. SAM을 Jetson에서 실시간으로 돌리고 싶다면, distillation이 거의 유일한 방법이다.

**Teacher-Student 구조**:

Teacher 모델(큰 모델, 이미 학습됨)의 출력을 student 모델(작은 모델)이 모방하도록 학습한다. Hard label(정답)보다 teacher의 soft prediction이 더 많은 정보를 담고 있다는 점이 핵심이다.

예를 들어, "고양이" 이미지에 대해 hard label은 [1, 0, 0]이지만, teacher의 soft prediction은 [0.85, 0.10, 0.05]일 수 있다. 이 soft prediction에는 "고양이와 개가 어느 정도 유사하다"는 정보가 담겨 있고, student는 이 정보까지 학습한다.

**Soft Targets와 Temperature Scaling**:

$$\mathcal{L}_{KD} = \text{KL}\left(\sigma\left(\frac{z_t}{\tau}\right) \| \sigma\left(\frac{z_s}{\tau}\right)\right)$$

여기서 z_t, z_s는 각각 teacher, student의 logits이고, τ는 temperature이다. τ > 1이면 probability distribution이 더 "부드러워"져서 클래스 간 관계 정보가 더 많이 전달된다. 일반적으로 τ = 3~5를 사용한다.

전체 loss는 hard label loss와 distillation loss의 가중 합이다:

$$\mathcal{L} = \alpha \cdot \mathcal{L}_{CE}(y, \sigma(z_s)) + (1 - \alpha) \cdot \tau^2 \cdot \mathcal{L}_{KD}$$

τ^2를 곱하는 이유: temperature scaling 때문에 gradient 크기가 1/τ^2로 줄어드는 것을 보상한다.

**Feature-based Distillation (FitNets)**:

Logit뿐 아니라 중간 layer의 feature map도 teacher와 유사하게 만든다.

$$\mathcal{L}_{feat} = \|f_t(x) - r(f_s(x))\|^2$$

여기서 r은 student feature 차원을 teacher 차원에 맞추는 projection layer이다. Logit distillation만으로는 전달하기 어려운 중간 표현까지 학습시킬 수 있다.

**VFM 경량화 응용**:

| Teacher | Student | 방법 |
|---------|---------|------|
| SAM (ViT-H) | MobileSAM | Image encoder를 경량 ViT로 교체, distillation |
| SAM (ViT-H) | FastSAM | YOLO 아키텍처로 전체 파이프라인 대체 |
| DINOv2-giant | DINOv2-small | 같은 구조의 작은 버전으로 distillation |

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
> - [Zhang et al., "Faster Segment Anything (MobileSAM)" (2023)](https://arxiv.org/abs/2306.14289) — SAM distillation 사례
> - [Romero et al., "FitNets: Hints for Thin Deep Nets" (2015)](https://arxiv.org/abs/1412.6550) — Feature-based distillation 원논문

---

## 10.11 심화: 도메인 적응

*연구자가 되고 싶다면 여기서부터 읽어라.*

시뮬레이션에서 학습한 모델을 실제 로봇에 배포하면, 성능이 급격히 떨어진다. 실내 데이터로 학습해서 실외에 배포해도 마찬가지이다. 이 문제를 **domain shift**라 하고, 이를 해결하는 연구가 domain adaptation이다. 로보틱스에서는 sim-to-real gap 문제와 직결된다.

**문제 정의**:
- Source domain D_s (라벨 있음): 시뮬레이션 데이터 또는 기존 데이터셋
- Target domain D_t (라벨 없음 또는 소량): 실제 배포 환경
- 목표: D_s에서 학습한 모델이 D_t에서도 잘 작동하도록 한다.

**Domain Randomization**:

가장 단순하지만 효과적인 접근법이다. 시뮬레이터에서 학습 데이터를 생성할 때, 환경 파라미터를 극단적으로 랜덤화한다.

- 텍스처: 벽, 바닥, 물체의 텍스처를 매 에피소드마다 랜덤 변경
- 조명: 위치, 색상, 강도를 랜덤 변경
- 카메라 파라미터: focal length, 위치, 각도에 noise 추가
- 물리 파라미터: 마찰 계수, 질량, 관성 등을 범위 내에서 랜덤 설정

아이디어: 충분히 다양한 시뮬레이션 환경을 보면, 실제 환경도 "또 하나의 변종"으로 처리될 수 있다.

**Adversarial Domain Adaptation**:

Domain discriminator를 도입하여, feature extractor가 source와 target을 구분할 수 없는 domain-invariant feature를 학습하도록 만든다.

```
Input --> Feature Extractor --> [Task Classifier]      --> Task Loss
                              \--> [Domain Discriminator] --> Domain Loss (GRL)
```

- Gradient Reversal Layer (GRL): domain discriminator의 gradient를 반전시켜서, domain discriminator가 source와 target을 구분하지 못하는 방향으로 feature extractor를 학습시킨다.
- Task classifier는 source domain에서 정상적으로 학습한다.
- feature extractor는 task에 유용하면서도 domain에 불변인 표현을 학습한다.

$$\mathcal{L} = \mathcal{L}_{task}(D_s) - \lambda \cdot \mathcal{L}_{domain}(D_s, D_t)$$

마이너스 부호가 핵심이다. Domain loss를 "최대화"하는 방향으로 feature extractor를 학습시킨다 (GAN과 유사한 적대적 학습).

**Test-Time Adaptation (TTA)**:

배포 후에도 모델이 새로운 환경에 적응하는 방법이다. 학습 데이터에 접근하지 않고, 추론 시 들어오는 데이터만으로 모델을 조정한다.

- **TENT**: batch normalization의 affine parameter를 entropy minimization으로 조정한다.
- **CoTTA**: continual TTA. 시간에 따라 distribution이 변하는 경우에도 적응한다.

```python
# TENT 핵심 아이디어 (간략화)
model.eval()
# BN의 affine parameter만 학습 가능하게
for m in model.modules():
    if isinstance(m, nn.BatchNorm2d):
        m.requires_grad_(True)
        m.track_running_stats = False  # batch statistics 사용

optimizer = optim.SGD(model.parameters(), lr=1e-4)

# 추론 시 adaptation
for batch in test_loader:
    output = model(batch)
    loss = entropy(output)  # prediction entropy 최소화
    loss.backward()
    optimizer.step()
    optimizer.zero_grad()
```

**Sim-to-Real Gap과의 연결**:

실제 로보틱스에서는 이 기법들을 조합한다:
1. 시뮬레이터에서 domain randomization으로 다양한 데이터를 생성한다.
2. 실제 환경의 소량 데이터로 adversarial adaptation을 수행한다.
3. 배포 후 TTA로 환경 변화에 지속 적응한다.

이 조합이 현재 sim-to-real transfer에서 가장 널리 쓰이는 접근법이다.

> **추천 자료**
> - [Tobin et al., "Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World" (2017)](https://arxiv.org/abs/1703.06907) — Domain randomization 원논문
> - [Ganin et al., "Domain-Adversarial Training of Neural Networks" (2016)](https://arxiv.org/abs/1505.07818) — Adversarial domain adaptation 원논문 (GRL 제안)
> - [Wang et al., "TENT: Fully Test-Time Adaptation by Entropy Minimization" (2021)](https://arxiv.org/abs/2006.10726) — TTA 대표작
> - [Wen et al., "FoundationPose: Unified 6D Pose Estimation and Tracking of Novel Objects" (CVPR 2024, arXiv:2312.08344)](https://arxiv.org/abs/2312.08344) — 새로운 물체의 6D pose 추정. CAD 모델 또는 참조 이미지 몇 장으로 동작

---

> **기술 흐름: 딥러닝 기반 인식 (Deep Learning for Perception)**
> - **2012**: AlexNet이 ImageNet 대회에서 기존 방법을 큰 차이로 이기며 우승. "딥러닝 혁명"의 시작. hand-crafted feature의 시대가 저물기 시작
> - **2014~2015**: VGGNet, GoogLeNet, ResNet 등장. 특히 ResNet (2015)의 residual connection은 수백 층의 네트워크를 학습 가능하게 만듦. 이 시기에 Faster R-CNN (2015), YOLO (2016)로 실시간 object detection이 가능해짐
> - **2017**: "Attention Is All You Need" — Transformer 발표. 원래 NLP용이었지만, 이후 비전까지 확장
> - **2020~2021**: ViT (Vision Transformer) 등장. 이미지를 패치 시퀀스로 처리하는 새 패러다임. DETR로 detection에도 Transformer 적용. Swin Transformer가 다양한 비전 태스크에서 SOTA 달성
> - **2022~**: ConvNeXt가 "CNN도 아직 죽지 않았다"를 보여줌. Segment Anything(SAM)이 segmentation을 foundation model로 끌어올림. Spatial AI로의 확장 — depth estimation, 3D scene understanding이 딥러닝의 다음 전장
> - **지금 주목할 것**: 단일 태스크 모델에서 foundation model로의 전환. DINOv2 하나로 detection, segmentation, depth feature를 모두 추출하는 파이프라인이 실험적으로 등장하고 있다
