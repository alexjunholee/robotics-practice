# Ch.10 — 딥러닝 기반 인식 (Deep Learning for Perception)

학습 기반 인식은 데이터에서 표현을 학습해 물체의 클래스, 위치, 영역을 예측한다. 분류는 무엇이 있는지, 물체 탐지는 어디에 있는지, 분할은 어느 픽셀이 그 물체인지 알려 준다. 로봇은 이 출력으로 장면 속 대상을 찾아 조작한다.

---

## 10.1 프레임워크 선택

어떤 프레임워크를 고르느냐에 따라 바로 돌려 볼 수 있는 연구 코드와 사전학습 모델이 달라진다. 프레임워크 하나로 모델 정의, 학습, 추론, 디버깅을 익혀 두면 다른 코드베이스도 읽기 쉽다.

### 10.1.1 PyTorch (권장)

**장점**:
- 코드를 한 줄씩 바로 실행하는 동적 그래프 (eager execution)
- 일반 Python 디버거로 디버깅할 수 있다
- 사전학습 모델이 많다 (torchvision, timm)

공개 연구 코드도 PyTorch로 짠 것이 많아서, PyTorch에 익숙하면 최신 논문의 구현을 돌려 보고 고치기 쉽다.

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
> - [PyTorch 공식 튜토리얼](https://docs.pytorch.org/tutorials/) — 입문부터 고급 주제까지 다룬다
> - [d2l.ai (Dive into Deep Learning)](https://d2l.ai/) — 인터랙티브 교과서. PyTorch 코드와 수학이 함께 나온다
> - [Andrej Karpathy — Neural Networks: Zero to Hero](https://www.youtube.com/playlist?list=PLAqhIrjkxbuWI23v9cThsA9GvCAUhRvKZ) — 전 Tesla AI Director가 신경망을 밑바닥부터 설명
> - [Jaejun Yoo's Playground](https://jaejunyoo.blogspot.com/search/label/kr) — GAN, VAE 같은 생성 모델을 한국어로 설명한 블로그

### 10.1.2 TensorFlow / JAX

**TensorFlow**: 프로덕션 배포 도구가 많고, TF Lite(2024년 9월 LiteRT로 이름이 바뀜)로 모바일 기기나 라즈베리파이 같은 엣지 디바이스에 모델을 올린다.
**JAX**: 함수형 프로그래밍 방식의 고성능 수치 연산 라이브러리로, 주로 연구에 쓰고 Google DeepMind 연구 코드에 자주 나온다.

PyTorch부터 배우되, 이 두 프레임워크도 이름과 쓰임새는 알아 둔다.

> **추천 자료**
> - [TensorFlow 공식 가이드](https://www.tensorflow.org/guide) — TFLite 변환까지 다룬다
> - [JAX 공식 문서](https://docs.jax.dev/en/latest/) — 함수형 딥러닝 프레임워크

---

## 10.2 딥러닝 기초 개념

뒤에 나올 ResNet은 CNN의 합성곱 구조를, ViT와 DETR은 Transformer의 attention 구조를 바탕으로 한다.

### 10.2.1 CNN (Convolutional Neural Network)

CNN은 학습한 필터를 영상 전체에 밀어 가며 적용해 공간 특징을 뽑는다. 에지, 코너, 텍스처 같은 지역 패턴을 찾는 필터도 데이터로 학습한다. 앞 장의 SIFT와 ORB는 사람이 특징을 설계했지만, CNN의 필터 값은 학습 목적함수를 최적화하면서 정해진다.

**주요 구성 요소**:
- Convolution Layer: 필터로 특징을 뽑는다
- Pooling Layer: 공간 크기를 줄인다 (Max, Average)
- Activation: 비선형성을 넣는다 (ReLU, GELU)
- Batch Normalization: 층 출력을 정규화해 학습을 안정시킨다

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

선형대수로 보면 convolution은 필터(kernel)와 이미지 패치의 내적을 위치마다 계산하는 연산이다. stride가 1일 때 `kernel_size=3, padding=1`로 두면 출력 크기가 입력과 같다. 이 설정은 CNN 코드에 자주 나온다.

> **추천 자료**
> - [Stanford CS231n — Convolutional Neural Networks for Visual Recognition](https://www.youtube.com/playlist?list=PLoROMvodv4rMFqRtEuo6SGjY4XbRIVRd4) — CNN을 다루는 Stanford 강의
> - [d2l.ai — CNN 챕터](https://d2l.ai/chapter_convolutional-neural-networks/index.html) — 코드와 수학 설명이 함께 나온다
> - [3Blue1Brown — But what is a Neural Network?](https://www.youtube.com/watch?v=aircAruvnKk) — 신경망을 그림으로 설명하는 영상

### 10.2.2 Attention & Transformer

**Self-Attention**: 시퀀스 안의 모든 위치 쌍 사이의 관계를 계산한다

```
Attention(Q, K, V) = softmax(QK^T / √d_k) V
```

CNN의 convolution은 한 층에서 가까운 이웃만 모으고, 층을 쌓아 receptive field(출력 하나가 보는 입력 범위)를 넓힌다. Transformer의 self-attention은 한 층 안에서도 멀리 떨어진 위치 사이의 관계를 계산한다. 2020년 이후 분류·탐지·분할 모델에는 Transformer만 쓴 구조와 CNN과 Transformer를 섞은 구조가 함께 쓰인다.

Vision Transformer (ViT)는 이미지를 16×16 같은 고정 크기 패치로 나누고, 패치 하나를 NLP의 token 하나처럼 다뤄 Transformer encoder에 넣는다. 원 논문은 대규모 데이터로 사전학습했을 때 비교한 CNN과 같거나 더 높은 분류 성능을 보고했다. 그 뒤로 Transformer 구조는 여러 비전 태스크에 쓰인다.

> **추천 자료**
> - [Vaswani et al., "Attention Is All You Need" (2017)](https://arxiv.org/abs/1706.03762) — Transformer 원논문
> - [Dosovitskiy et al., "An Image is Worth 16x16 Words" (2020)](https://arxiv.org/abs/2010.11929) — ViT 원논문
> - [Yannic Kilcher — Vision Transformer 설명](https://www.youtube.com/watch?v=TrdevFK_am4) — 논문을 풀어 설명하는 영상
> - [Andrej Karpathy — Let's build GPT from scratch](https://www.youtube.com/watch?v=kCc8FmEb1nY) — Transformer를 밑바닥부터 구현한다. 예제는 NLP지만 ViT도 같은 encoder 구조를 쓴다

---

## 10.3 Image Classification

분류(classification)는 이미지에 무엇이 있는지 알아맞히는 태스크다. Detection과 segmentation 모델도 안에 분류기가 들어 있다. 분류용으로 사전학습한 backbone(ResNet, ViT 등)은 다른 태스크에서 feature extractor로 쓴다.

**대표 모델**:

| 모델 | 특징 | 용도 |
| --- | --- | --- |
| ResNet | Residual connection으로 깊은 네트워크도 학습이 안정적 | 백본 네트워크 |
| EfficientNet | 깊이·너비·해상도를 함께 키우는 compound scaling | 모바일, 연산량이 제한된 환경 |
| ViT | Transformer 기반 | 대규모 데이터로 사전학습할 때 |
| ConvNeXt | ViT의 설계 요소를 CNN에 옮김 | ViT 대신 쓰는 CNN 백본 |

ResNet의 residual connection은 블록의 입력을 출력에 더해 깊은 네트워크도 안정적으로 학습하게 한다. 2015년 발표 뒤 Transformer를 비롯한 여러 아키텍처가 이 연결을 쓴다.

**사전학습 모델 사용**:

```python
import torchvision.models as models

# Pretrained ResNet50
model = models.resnet50(weights='IMAGENET1K_V2')

# Feature extractor로 사용
model.fc = nn.Identity()  # 마지막 FC 제거
features = model(x)  # (batch, 2048)
```

ImageNet으로 사전학습한 모델에서 마지막 분류 레이어를 떼어 내면 앞단의 출력을 feature로 다시 쓸 수 있다. 이런 transfer learning(전이 학습)은 새 데이터가 적은 인식 문제에서 학습을 시작할 표현을 준다.

> **추천 자료**
> - [He et al., "Deep Residual Learning for Image Recognition" (2015)](https://arxiv.org/abs/1512.03385) — ResNet 원논문
> - [Papers With Code — Image Classification (2025년 서비스 종료 전 Internet Archive 보관본)](https://web.archive.org/web/20241228230436/https://paperswithcode.com/task/image-classification) — 공개 구현과 과거 leaderboard를 찾는 출발점. 최신 수치와 protocol은 벤치마크 공식 페이지와 원 논문에서 다시 확인
> - [timm (PyTorch Image Models) 라이브러리](https://github.com/huggingface/pytorch-image-models) — 수백 개의 사전학습 모델을 한 줄로 로드
> - [Stanford CS231n — Training Neural Networks](https://www.youtube.com/playlist?list=PLoROMvodv4rMFqRtEuo6SGjY4XbRIVRd4) — 학습 기법과 트릭

---

## 10.4 Object Detection

컵을 집으려면 컵이 있다는 사실과 함께 컵의 위치도 알아야 한다. Object detection은 물체의 클래스와 bounding box(물체를 둘러싼 직사각형)를 함께 예측한다. 로봇 manipulation과 자율주행에 쓴다.

### 10.4.1 Two-Stage Detectors

**Faster R-CNN**:
1. Region Proposal Network (RPN): 물체가 있을 만한 후보 영역을 뽑는다
2. ROI Pooling: 후보 영역마다 특징을 뽑는다
3. Classification + Bounding Box Regression: 클래스를 정하고 박스를 다듬는다

장점: 정확도가 높다
단점: one-stage 방식보다 느리다

Faster R-CNN은 two-stage detector의 대표 모델이다. 후보를 먼저 뽑고 후보마다 다시 분류하는 이 구조는 Mask R-CNN으로 이어졌다.

### 10.4.2 One-Stage Detectors

**YOLO (You Only Look Once)**:
- 이미지를 그리드로 나누고 모든 칸의 박스를 한 번에 예측한다
- 실시간으로 돌아간다 (원 논문 기준 45 FPS)
- 버전: YOLOv5, YOLOv8, YOLO11 (Ultralytics)

YOLO는 이미지 전체를 forward pass 한 번으로 처리해 박스와 클래스를 함께 예측한다. 실시간 처리가 필요한 로봇 시스템에 쓴다. Ultralytics의 YOLOv8과 YOLO11은 설치하고 돌리는 절차가 짧아 프로토타이핑에 자주 쓴다.

```python
from ultralytics import YOLO

# 모델 로드 및 추론
model = YOLO('yolov8n.pt')  # nano 모델
results = model('image.jpg')

# 결과 시각화
results[0].show()
```

**SSD (Single Shot Detector)**:
- 해상도가 다른 여러 feature map에서 박스를 예측한다
- 작은 물체를 놓치기 쉽다. YOLOv3는 FPN과 비슷한 다중 스케일 예측을 넣어 이 문제를 줄였다

> **추천 자료**
> - [Redmon et al., "You Only Look Once: Unified, Real-Time Object Detection" (2016)](https://arxiv.org/abs/1506.02640) — YOLO 원논문
> - [Ultralytics YOLOv8 문서](https://docs.ultralytics.com/) — 설치부터 직접 만든 데이터로 학습하는 방법까지 다룬다
> - [Papers With Code — Object Detection (2025년 서비스 종료 전 Internet Archive 보관본)](https://web.archive.org/web/20250115141015/https://paperswithcode.com/task/object-detection) — 공개 구현과 과거 leaderboard를 찾는 출발점. 최신 수치와 protocol은 벤치마크 공식 페이지와 원 논문에서 다시 확인
> - [다크 프로그래머 — precision, recall의 이해](https://darkpgmr.tistory.com/162) — detection 평가 지표를 한국어로 설명

### 10.4.3 Transformer-based

DETR (Detection Transformer)은 detection을 집합 예측 문제로 바꿔 풀었다. 학습 가능한 벡터인 object query가 정해진 개수만큼 있고, query 하나가 물체 하나를 맡는다. 겹친 박스를 지우는 NMS(non-maximum suppression) 후처리와 미리 깔아 두는 anchor box 없이 end-to-end로 학습한다. 원래 DETR은 수렴이 느려 학습 epoch이 많이 필요했다. Deformable DETR, DINO(DETR 계열 탐지기로, §10.9의 DINOv2와 다른 모델), Co-DETR 같은 후속 연구가 이 구조를 이어받았다.

> **추천 자료**
> - [Carion et al., "End-to-End Object Detection with Transformers" (2020)](https://arxiv.org/abs/2005.12872) — DETR 원논문
> - [Yannic Kilcher — DETR 설명](https://www.youtube.com/watch?v=T35ba_VXkMY) — 논문을 풀어 설명하는 영상
> - [HuggingFace — Object Detection 가이드](https://huggingface.co/docs/transformers/tasks/object_detection) — Transformers 라이브러리로 DETR 사용하기
> - [Zhao et al., "DETRs Beat YOLOs on Real-time Object Detection" (RT-DETR, CVPR 2024, arXiv:2304.08069)](https://arxiv.org/abs/2304.08069) — 논문의 비교 설정에서 실시간 DETR의 속도·정확도 개선을 보고
> - [Cheng et al., "YOLO-World: Real-Time Open-Vocabulary Object Detection" (CVPR 2024, arXiv:2401.17270)](https://arxiv.org/abs/2401.17270) — YOLO에 텍스트 프롬프트로 클래스를 지정하는 open-vocabulary detection을 더했다

---

## 10.5 Semantic Segmentation

Semantic segmentation은 픽셀마다 클래스를 예측한다.

박스는 물체 주변의 배경까지 포함하지만, 픽셀마다 클래스를 붙이면 물체와 배경의 경계를 더 세밀하게 나눌 수 있다. 예측한 경계가 실제 윤곽과 꼭 맞지는 않으므로 grasping에 쓸 때는 depth, 개체 구분, 예측 불확실성도 함께 확인한다. 자율주행에서 도로, 인도, 차선을 나눌 때도 픽셀 단위 분류를 쓴다.

**대표 모델**:

| 모델 | 특징 |
| --- | --- |
| FCN | fully connected 층 없이 convolution만으로 end-to-end 분할을 한 초기 모델 |
| U-Net | Encoder-Decoder 구조, 의료 영상 분할용으로 제안 |
| DeepLab v3+ | Atrous(dilated) convolution으로 여러 스케일의 문맥을 모음 |
| SegFormer | Transformer encoder와 가벼운 MLP 디코더 |

U-Net은 Encoder에서 특징을 뽑으며 해상도를 줄이고, Decoder에서 해상도를 되살린다. 이때 Skip Connection으로 Encoder의 같은 해상도 특징을 Decoder에 넘겨 세부 정보를 채운다. 이 Encoder-Decoder와 Skip Connection 구조는 segmentation뿐 아니라 depth estimation, 이미지 생성 모델에도 쓰인다.

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

**Instance Segmentation**: 같은 클래스 안에서도 개체를 하나씩 구분한다
- Mask R-CNN: Faster R-CNN에 mask를 예측하는 브랜치를 더했다

**Panoptic Segmentation**: Semantic과 Instance 결과를 하나로 합친다
- "Things" (셀 수 있는 물체): 개체를 구분한다
- "Stuff" (하늘, 도로 같은 배경): 개체를 구분하지 않는다

Semantic segmentation은 픽셀마다 클래스("의자")를 붙인다. Instance segmentation은 같은 클래스 안에서 개체("의자 1, 2, 3")를 나눈다. "왼쪽 의자를 집어" 같은 명령에는 instance 단위 결과를 쓴다. Panoptic segmentation은 두 결과를 한 출력으로 묶는다.

> **추천 자료**
> - [He et al., "Mask R-CNN" (2017)](https://arxiv.org/abs/1703.06870) — Mask R-CNN 원논문
> - [Detectron2](https://github.com/facebookresearch/detectron2) — Meta의 detection/segmentation 프레임워크. Mask R-CNN 같은 모델이 들어 있다

---

## 10.7 Depth Estimation

Monocular depth estimation은 이미지 한 장에서 깊이를 예측한다.

스테레오 카메라나 LiDAR 없이 카메라 하나로 깊이를 얻으면 센서 비용과 무게가 준다. 드론이나 소형 로봇처럼 실을 수 있는 무게(payload)가 작은 시스템에 쓸모가 있다. 2024년에 나온 Depth Anything처럼 대규모 데이터로 학습해 처음 보는 장면에서도 깊이를 예측하는 모델도 있다.

**대표 모델**:
- MiDaS: 여러 데이터셋을 섞어 학습했다
- Depth Anything: 라벨 없는 이미지 약 6,200만 장을 학습에 더했다
- ZoeDepth: 미터 단위 깊이(metric depth)를 추정한다

```python
# Depth Anything 사용
from transformers import pipeline

pipe = pipeline("depth-estimation", model="LiheYoung/depth-anything-base-hf")
result = pipe("image.jpg")
depth = result['depth']
```

MiDaS와 Depth Anything의 기본 모델은 상대적 깊이(relative depth)를 추정한다. 출력에는 A가 B보다 가깝다는 순서와 상대적인 비율만 들어 있다. 미터 단위 깊이가 필요하면 ZoeDepth나 Depth Anything V2의 metric 버전을 쓴다.

> **추천 자료**
> - [Yang et al., "Depth Anything: Unleashing the Power of Large-Scale Unlabeled Data" (2024)](https://arxiv.org/abs/2401.10891) — Depth Anything 원논문
> - [Godard et al., "Digging Into Self-Supervised Monocular Depth Estimation" (Monodepth2, ICCV 2019, arXiv:1806.01260)](https://arxiv.org/abs/1806.01260) — 자기지도 단안 깊이 추정에서 비교 기준으로 자주 쓰는 모델
> - [HuggingFace — Monocular Depth Estimation](https://huggingface.co/docs/transformers/tasks/monocular_depth_estimation) — 바로 돌려 볼 수 있는 코드
> - [Papers With Code — Monocular Depth Estimation (2025년 서비스 종료 전 Internet Archive 보관본)](https://web.archive.org/web/20241128105333/https://paperswithcode.com/task/monocular-depth-estimation) — 공개 구현과 과거 leaderboard를 찾는 출발점

---

## 10.8 심화: 학습 레시피

같은 아키텍처도 learning rate와 augmentation 설정에 따라 수렴 속도와 성능이 달라진다. 논문 결과를 재현하려면 학습 설정까지 맞춰야 한다. 아래는 실무에서 자주 쓰는 학습 기법이다.

**Learning Rate Schedule**:

- Cosine Annealing with Warm-up: 처음 몇 epoch 동안 learning rate를 0에서 목표값까지 선형으로 올리고(warm-up), 그다음 cosine 곡선을 따라 줄인다.

$$\eta_t = \eta_{min} + \frac{1}{2}(\eta_{max} - \eta_{min})\left(1 + \cos\left(\frac{t \cdot \pi}{T}\right)\right)$$

η_t는 epoch t의 learning rate, η_max와 η_min은 시작값과 최솟값, T는 cosine으로 줄이는 전체 epoch 수(아래 코드의 `T_max`)다. 이 식은 warm-up이 끝난 뒤의 구간이다.

- OneCycleLR: 학습 동안 learning rate를 한 번 올렸다가 내린다. Smith는 이 방식으로 적은 epoch 만에 수렴하는 현상을 super-convergence라 불렀다.

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

**Data Augmentation**:

| 기법 | 설명 | 주 용도 |
|------|------|---------|
| RandAugment | 변환 N개를 세기 M으로 무작위로 고른다 | 분류 전반 |
| CutMix | 이미지 한 영역을 다른 이미지로 바꾸고, 라벨도 면적 비율대로 섞는다 | 분류 |
| MixUp | 두 이미지와 라벨을 선형 보간한다 | 분류 |
| Mosaic | 이미지 4장을 한 장으로 붙인다 | Detection (YOLO 계열) |

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

**Regularization**:

- Label Smoothing: 정답 1, 오답 0인 hard label 대신, 정답 확률에서 ε(예: 0.1)만큼 덜어 모든 클래스에 고르게 나눠 준 soft label을 쓴다. 모델이 지나치게 확신하지 않게 한다. `nn.CrossEntropyLoss(label_smoothing=0.1)`
- Stochastic Depth: 학습할 때 일부 residual 블록을 무작위로 건너뛴다. 깊은 ResNet 계열의 overfitting을 줄인다.
- Weight Decay: optimizer에 `weight_decay=0.01~0.05`를 준다. AdamW는 weight decay를 gradient 갱신과 분리해서(decoupled) 적용한다.

**Gradient Clipping**: gradient 크기에 상한을 두어 gradient가 폭발하지 않게 한다. Transformer를 학습할 때 자주 쓴다.

```python
torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
```

**Loss Curve로 문제 진단**:

| 패턴 | 진단 | 대응 |
|------|------|------|
| train loss 감소, val loss 증가 | Overfitting | augmentation 추가, dropout/weight decay 증가, 데이터 확보 |
| train loss가 높은 채로 멈춤 | Underfitting | 모델 크기 증가, learning rate 조정, augmentation 감소 |
| train loss가 심하게 출렁임 | Learning rate 과다 | learning rate 감소 |
| train loss NaN 발생 | Gradient explosion | gradient clipping, learning rate 대폭 감소, 데이터 검증 |
| val loss 초반에 급감 후 완전 정체 | Learning rate 부족 또는 스케줄 문제 | warm-up 추가, cosine schedule 적용 |

**Distributed Training — PyTorch DDP 기본**:

모델과 데이터가 커지면 GPU 한 장으로는 학습이 너무 오래 걸린다. DistributedDataParallel (DDP)은 GPU마다 모델을 복제하고 gradient를 동기화하는 기본 병렬 학습 방법이다.

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

## 10.9 심화: 자기지도 학습과 대조 학습

로봇은 이미지를 수천, 수만 장씩 모으지만, 그 모두에 bounding box나 segmentation mask를 손으로 달기는 어렵다. 자기지도 학습(self-supervised learning)은 라벨 없이 데이터 자체에서 학습 신호를 만든다.

**Contrastive Learning**:

같은 이미지에 다른 augmentation을 적용한 두 장(positive pair)은 임베딩 공간에서 가깝게 두고, 다른 이미지와 짝지은 쌍(negative pair)은 멀리 둔다.

- SimCLR: 같은 이미지에 서로 다른 augmentation을 적용해 positive pair를 만들고, 배치 안의 다른 이미지를 negative로 쓴다. negative가 많아야 해서 배치가 커야 한다.
- MoCo (Momentum Contrast): momentum encoder와 queue로 지난 배치의 임베딩을 모아 두어, 배치가 작아도 negative를 많이 쓴다.

**InfoNCE Loss**:

$$\mathcal{L} = -\log \frac{\exp(\text{sim}(z_i, z_j) / \tau)}{\sum_{k=1}^{2N} \mathbb{1}_{[k \neq i]} \exp(\text{sim}(z_i, z_k) / \tau)}$$

배치의 이미지 N장에 augmentation을 두 번씩 적용하면 임베딩이 2N개 나온다. z_i와 z_j는 같은 이미지에서 나온 두 임베딩이고, sim은 코사인 유사도, τ는 temperature다. 이 loss를 줄이면 분자의 positive pair 유사도는 커지고, 분모에 들어 있는 negative pair 유사도는 작아진다.

**Masked Image Modeling — MAE**:

MAE는 ViT 입력 패치의 75%를 무작위로 가리고, 남은 25%로 가린 부분을 복원하도록 학습한다. NLP의 BERT가 단어를 가리고 맞히게 하는 방식을 이미지에 옮겼다.

- 마스킹 비율이 75%로 높은 이유: 이미지는 텍스트보다 중복된 정보가 많아서, 조금만 가리면 주변 패치로 쉽게 메울 수 있다. 많이 가려야 과제가 어려워지고 쓸모 있는 표현을 배운다.
- 인코더는 가리지 않은 25% 패치만 처리한다. 원 논문은 이 덕분에 사전학습이 3배 이상 빨라졌다고 보고한다. attention 비용은 토큰 수에 초선형이고 decoder 비용이 더해지므로, 연산량 감소율은 마스킹 비율과 다르다.

**DINOv2와의 연결**:

DINOv2는 self-distillation으로 학습한다. teacher와 student가 같은 구조이고, teacher의 weight는 student weight의 EMA(exponential moving average, 지수 이동 평균)다.

- Sharpening과 centering: teacher 출력에 낮은 temperature를 적용해 분포를 뾰족하게 만든다. DINO v1은 teacher 출력에서 평균을 빼는 centering을 썼고, DINOv2는 그 자리에 SwAV의 Sinkhorn-Knopp centering을 쓴다. 둘 다 모든 입력이 같은 출력으로 무너지는 collapse를 막는다.
- DINOv2 논문은 backbone을 고정한 채로 여러 벤치마크에서 supervised 사전학습 모델에 견줄 만한 성능을 보고했다. ImageNet 분류는 linear probe와 학습이 필요 없는 k-NN으로, ADE20K 의미 분할은 선형 head를 학습하는 linear probe의 mIoU로 평가했다. 수치를 인용할 때는 평가 방식과 모델 크기를 함께 적는다.

**실무 — HuggingFace에서 self-supervised backbone을 얼리고 분류 head만 학습**:

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

## 10.10 심화: Knowledge Distillation

Knowledge distillation은 큰 모델(teacher)의 출력을 작은 모델(student)이 따라 하도록 학습시킨다. 로보틱스에서는 VFM 같은 큰 모델을 edge device에서 돌리려고 쓴다. MobileSAM은 SAM의 image encoder를 작은 encoder로 distillation해 가볍게 만들었다.

**Teacher-Student 구조**:

이미 학습한 teacher가 내는 확률 분포(soft prediction)에는 정답 라벨(hard label)보다 많은 정보가 들어 있다. 클래스가 (고양이, 개, 자동차)일 때 고양이 이미지의 hard label은 [1, 0, 0]이지만, teacher는 [0.85, 0.10, 0.05]처럼 답할 수 있다. 이 분포에는 고양이가 자동차보다 개와 더 닮았다는 정보가 담겨 있고, student는 이 정보까지 배운다.

**Soft Targets와 Temperature Scaling**:

$$\mathcal{L}_{KD} = \text{KL}\left(\sigma\left(\frac{z_t}{\tau}\right) \| \sigma\left(\frac{z_s}{\tau}\right)\right)$$

여기서 z_t, z_s는 각각 teacher, student의 logits, σ는 softmax, τ는 temperature이다. τ > 1이면 확률 분포가 평평해져서 정답이 아닌 클래스 사이의 차이도 student에 전해진다. 아래 예시 코드는 τ = 4를 쓴다.

전체 loss는 hard label loss와 distillation loss의 가중 합이다:

$$\mathcal{L} = \alpha \cdot \mathcal{L}_{CE}(y, \sigma(z_s)) + (1 - \alpha) \cdot \tau^2 \cdot \mathcal{L}_{KD}$$

y는 정답 라벨, α는 두 loss의 가중치다. τ^2를 곱하는 이유: logit을 τ로 나누면 soft target loss의 gradient가 1/τ^2로 작아지므로, 이를 되돌려 hard label loss와 균형을 맞춘다.

**Feature-based Distillation (FitNets)**:

FitNets는 logit에 더해 중간 layer의 feature map도 teacher와 비슷해지도록 학습한다.

$$\mathcal{L}_{feat} = \|f_t(x) - r(f_s(x))\|^2$$

f_t(x)와 f_s(x)는 입력 x에 대한 teacher와 student의 중간 layer feature이고, r은 student feature 차원을 teacher 차원에 맞추는 projection layer다. 이렇게 하면 logit만 맞출 때는 전해지지 않는 중간 표현도 student가 배운다.

**VFM 경량화 응용**:

| Teacher | Student | 방법 |
|---------|---------|------|
| SAM (ViT-H) | MobileSAM | Image encoder를 작은 ViT로 바꾸고 distillation |
| SAM | FastSAM | SAM의 데이터셋(SA-1B) 일부로 YOLOv8-seg를 학습 |
| DINOv2 ViT-g | DINOv2 ViT-S/B/L | 큰 모델을 같은 계열의 작은 ViT로 distillation |

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

## 10.11 심화: 도메인 적응

시뮬레이션에서 학습한 모델을 실제 로봇에 올리면 성능이 크게 떨어지는 일이 많다. 실내 데이터로 학습한 모델을 실외에서 써도 그렇다. 학습 데이터와 배포 환경의 분포가 다른 이 문제를 **domain shift**라 하고, 이를 줄이는 연구를 domain adaptation이라 한다. 로보틱스에서는 sim-to-real gap(Ch.8 §8.5)이 대표적인 domain shift다.

**문제 정의**:
- Source domain D_s (라벨 있음): 시뮬레이션 데이터 또는 기존 데이터셋
- Target domain D_t (라벨 없음 또는 소량): 실제 배포 환경
- 목표: D_s에서 학습한 모델이 D_t에서도 잘 동작하게 한다.

**Domain Randomization**:

시뮬레이터에서 학습 데이터를 만들 때 환경 파라미터를 넓은 범위에서 무작위로 바꾼다.

- 텍스처: 벽, 바닥, 물체의 텍스처를 매 에피소드마다 랜덤 변경
- 조명: 위치, 색상, 강도를 랜덤 변경
- 카메라 파라미터: focal length, 위치, 각도에 noise 추가
- 물리 파라미터: 마찰 계수, 질량, 관성 등을 범위 내에서 랜덤 설정

Tobin et al.(2017)은 시뮬레이션 환경을 충분히 다양하게 보여 주면 모델이 실제 환경도 그 변종 가운데 하나로 받아들일 수 있다고 보았다.

**Adversarial Domain Adaptation**:

Domain discriminator(입력이 어느 domain에서 왔는지 맞히는 분류기)를 두고, feature extractor는 이 분류기가 source와 target을 구분하지 못하는 feature를 배우게 한다. 이런 feature를 domain-invariant feature라 한다.

```
Input --> Feature Extractor --> [Task Classifier]      --> Task Loss
                              \--> [Domain Discriminator] --> Domain Loss (GRL)
```

- Gradient Reversal Layer (GRL): domain discriminator에서 feature extractor로 돌아오는 gradient의 부호를 뒤집는다. 그래서 feature extractor는 source와 target feature가 discriminator에게 같아 보이는 쪽으로 학습된다.
- Task classifier는 라벨이 있는 source domain 데이터로 평소처럼 학습한다.
- 결과적으로 feature extractor는 task에 쓸모 있고 domain에 따라 달라지지 않는 표현을 배운다.

$$\mathcal{L} = \mathcal{L}_{task}(D_s) - \lambda \cdot \mathcal{L}_{domain}(D_s, D_t)$$

D_s와 D_t는 source와 target 데이터, λ는 두 loss의 비중을 정하는 가중치다. 마이너스 부호 때문에 feature extractor는 domain loss를 키우는 쪽으로 학습된다. discriminator는 같은 loss를 줄이려 하므로, GAN처럼 두 네트워크가 서로 반대 목표로 학습한다.

**Test-Time Adaptation (TTA)**:

Test-time adaptation은 배포한 뒤에 모델을 새 환경에 맞춘다. 학습 데이터는 쓰지 않고, 추론할 때 들어오는 데이터만으로 모델을 조정한다.

- TENT: 예측 분포의 entropy를 줄이도록 batch normalization의 affine parameter만 갱신한다.
- CoTTA: 데이터 분포가 시간에 따라 계속 바뀌는 상황(continual TTA)을 다룬다.

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

목표와 쓸 수 있는 데이터에 따라 이 기법들을 섞어 쓴다. 예를 들면 이렇다.
1. 시뮬레이터에서 domain randomization으로 다양한 데이터를 만든다.
2. 실제 환경에서 모은 적은 데이터로 adversarial adaptation을 한다.
3. 배포한 뒤에는 TTA로 환경 변화를 따라간다.

세 단계를 늘 함께 쓰지는 않는다. Domain randomization만 쓰기도 하고, 실제 데이터로 파인튜닝을 더하기도 한다. 안전 때문에 배포 중에는 모델을 바꾸지 않기도 한다. 안전 요구와 계산 예산을 보고 조합을 정한다.

> **추천 자료**
> - [Tobin et al., "Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World" (2017)](https://arxiv.org/abs/1703.06907) — Domain randomization 원논문
> - [Ganin et al., "Domain-Adversarial Training of Neural Networks" (2016)](https://arxiv.org/abs/1505.07818) — GRL을 쓰는 adversarial domain adaptation 논문 (JMLR)
> - [Wang et al., "TENT: Fully Test-Time Adaptation by Entropy Minimization" (2021)](https://arxiv.org/abs/2006.10726) — TENT 원논문 (ICLR 2021)
> - [Wen et al., "FoundationPose: Unified 6D Pose Estimation and Tracking of Novel Objects" (CVPR 2024, arXiv:2312.08344)](https://arxiv.org/abs/2312.08344) — 처음 보는 물체의 6D pose를 추정하고 추적한다. CAD 모델이나 참조 이미지 몇 장만 있으면 된다

---

> **기술 흐름: 딥러닝 기반 인식 (Deep Learning for Perception)**
> - 2012: AlexNet이 ILSVRC 2012 분류 부문에서 top-5 오류율 15.3%로 2위(26.2%)를 크게 앞서 우승했다. 이후 손으로 설계한 특징 대신 학습한 특징을 쓰는 연구가 늘었다.
> - 2014~2016: VGGNet, GoogLeNet, ResNet이 나왔다. ResNet (CVPR 2016)은 residual connection으로 152층 네트워크를 학습했다. Faster R-CNN (2015)과 YOLO (CVPR 2016)가 나오면서 object detection이 빨라졌다.
> - 2017: "Attention Is All You Need"가 Transformer를 발표했다. 처음에는 기계 번역용이었다.
> - 2020~2021: DETR (2020)이 detection에 Transformer를 썼다. ViT (ICLR 2021)는 이미지를 패치 시퀀스로 처리했다. Swin Transformer (2021) 원 논문은 당시 COCO와 ADE20K에서 비교 모델보다 높은 점수를 보고했다.
> - 2022~2023: ConvNeXt (2022)는 CNN도 ViT와 비슷한 성능을 낼 수 있음을 보였다. Segment Anything (SAM, 2023)은 프롬프트로 가리킨 물체를 종류와 상관없이 분할한다. DINOv2 (arXiv 2023, TMLR 2024) 논문은 backbone을 고정한 채 분류·의미 분할(§10.9)에 더해 깊이 추정도 평가했다.
