# Ch.11 — Vision Foundation Models (VFM)


컴퓨터 비전의 패러다임이 바뀌고 있다. 앞서 배운 딥러닝 모델들이 특정 데이터셋에서 특정 태스크를 잘 하도록 학습된 specialist였다면, foundation model은 거대한 데이터로 범용적 시각 능력을 학습한 generalist이다. 이 차이가 보이면, 2023년 이후 ICRA/IROS에서 VFM 기반 인식 논문이 빠르게 늘어난 이유도 보인다.

---

## 11.1 Foundation Model이란?

**Foundation Model**은 대규모 데이터로 사전학습되어 다양한 downstream task에 적용 가능한 모델이다.

기존 접근을 생각해 보자. "새로운 환경 → 데이터 수집 → 라벨링 → 학습"의 사이클을 반복해야 했다. 공장이 바뀌면, 로봇이 인식해야 할 물체가 바뀌면, 처음부터 다시 해야 했다. Foundation model은 이 사이클을 끊는다. 한 번 학습된 모델이 본 적 없는 물체, 본 적 없는 환경에서도 작동한다. 로봇의 범용성(generalization) 문제를 푸는 가장 유력한 접근이다.

**특징**:
- **Scale**: 수억~수십억 파라미터
- **Pretraining**: 대규모 데이터 (수억 이미지)
- **Zero-shot / Few-shot**: 학습 없이 또는 적은 예제로 새 태스크 수행
- **Transfer**: 다양한 도메인으로 전이

**왜 중요한가?**
- 새로운 환경에서도 일반화 능력
- 특정 데이터셋에 대한 annotation 없이도 사용 가능
- 연구실의 Global Module에서 핵심 역할

여기서 Scale Law라는 개념이 중요하다. 모델 크기, 데이터 크기, 연산량을 키우면 성능이 power law로 향상된다는 경험적 법칙이다. GPT, CLIP, SAM 등 최근의 대형 모델들이 모두 이 법칙을 활용한다. "더 크면 더 좋다"가 일정 범위에서 성립한다 (Kaplan et al., 2020; Zhai et al., 2022 for ViT). 단, Hoffmann et al. (2022, Chinchilla)이 보여줬듯 모델 크기만 키우는 것보다 데이터와 연산량의 균형이 더 중요하다.

> **추천 자료**
> - [Bommasani et al., "On the Opportunities and Risks of Foundation Models" (2021)](https://arxiv.org/abs/2108.07258) — Foundation Model이라는 용어를 정의한 Stanford 보고서
> - [Two Minute Papers — Foundation Models 관련 영상들](https://www.youtube.com/@TwoMinutePapers) — 최신 VFM 연구를 빠르게 따라잡기
> - [HuggingFace Model Hub](https://huggingface.co/models) — 수천 개의 사전학습 모델을 바로 사용 가능

---

## 11.2 주요 VFM

로보틱스에서 가장 많이 활용되는 Vision Foundation Model들을 본다. 각 모델이 어떤 문제를 풀고, 왜 로보틱스에서 중요하며, 어떻게 사용하는지를 중심으로 다룬다.

### 11.2.1 DINOv2

**Self-Supervised Vision Transformer**로, 라벨 없이 이미지에서 풍부한 특징을 학습한다.

DINOv2는 라벨 없이도 범용적인 시각 특징을 학습한다. 이 특징은 분류, 분할, 매칭 등 다양한 태스크에 그대로 쓸 수 있다. 특히 로보틱스에서는 DINOv2의 dense feature가 텍스처 없는 영역에서도 안정적인 매칭을 제공하여, SLAM이나 Visual Odometry에서 textureless 환경의 tracking failure rate를 줄이는 데 쓰인다.

**특징**:
- Contrastive learning + Self-distillation
- 다양한 태스크에서 우수한 전이 성능
- Dense visual features 제공

**활용**:
- Image retrieval
- Semantic segmentation (linear probe)
- Feature matching for SLAM/VO
- 3D reconstruction의 feature backbone

```python
import torch
from transformers import AutoModel, AutoImageProcessor

processor = AutoImageProcessor.from_pretrained('facebook/dinov2-base')
model = AutoModel.from_pretrained('facebook/dinov2-base')

inputs = processor(images=image, return_tensors="pt")
outputs = model(**inputs)
features = outputs.last_hidden_state  # (1, num_patches+1, 768): CLS + 패치 feature
```

`last_hidden_state`의 첫 번째 토큰은 [CLS] 토큰으로 이미지 전체의 요약이고, 나머지는 각 패치의 feature이다. [CLS]는 분류에, 패치 features는 dense prediction(segmentation, matching 등)에 쓰인다.

> **추천 자료**
> - [Oquab et al., "DINOv2: Learning Robust Visual Features without Supervision" (2023)](https://arxiv.org/abs/2304.07193) — DINOv2 원논문
> - [DINOv2 GitHub](https://github.com/facebookresearch/dinov2) — 공식 코드 및 사전학습 모델
> - [HuggingFace — DINOv2](https://huggingface.co/docs/transformers/model_doc/dinov2) — HuggingFace에서 바로 사용
> - [Yannic Kilcher — DINO 설명](https://www.youtube.com/watch?v=h3ij3F3cPIk) — Self-distillation의 핵심 아이디어를 설명 (DINOv1 기반이지만 DINOv2 이해에 필수)

### 11.2.2 SAM (Segment Anything Model)

**Promptable Segmentation**: 점, 박스, 텍스트 등의 프롬프트로 어떤 객체든 분할한다.

기존 segmentation 모델은 학습에 사용된 클래스만 분할할 수 있었다. "의자, 테이블, 사람"으로 학습하면 "컵"은 분할하지 못한다. SAM은 1.1B개의 마스크로 학습되어, 본 적 없는 어떤 물체든 분할할 수 있다. 로봇이 새로운 환경에서 처음 보는 물체를 조작해야 할 때, SAM 이후로 segmentation 접근 방식이 달라졌다.

**구성**:
- Image Encoder: ViT로 이미지 임베딩
- Prompt Encoder: 포인트, 박스, 마스크 등
- Mask Decoder: 경량 디코더로 마스크 생성

**SAM2**: 비디오 지원, 더 빠른 속도

SAM2는 단일 이미지 뿐 아니라 비디오에서도 동작한다. 첫 프레임에서 포인트/박스로 물체를 지정하면, 이후 프레임에서 자동으로 추적하며 분할한다. 이는 로봇이 실시간으로 물체를 추적하며 조작하는 시나리오에 직접 적용할 수 있다.

```python
from segment_anything import sam_model_registry, SamPredictor

sam = sam_model_registry["vit_h"](checkpoint="sam_vit_h.pth")
predictor = SamPredictor(sam)

predictor.set_image(image)
masks, scores, logits = predictor.predict(
    point_coords=np.array([[500, 375]]),
    point_labels=np.array([1]),  # 1: 전경(foreground)
    multimask_output=True,
)
```

`multimask_output=True`로 하면 3개의 마스크 후보가 나온다(전체 물체, 부분, 더 작은 부분). `scores`로 가장 적합한 마스크를 고르면 된다.

> **추천 자료**
> - [Kirillov et al., "Segment Anything" (2023)](https://arxiv.org/abs/2304.02643) — SAM 원논문
> - [Ravi et al., "SAM 2: Segment Anything in Images and Videos" (2024)](https://arxiv.org/abs/2408.00714) — SAM2 원논문. 비디오 segmentation으로 확장
> - [Segment Anything GitHub](https://github.com/facebookresearch/segment-anything) — 공식 코드
> - [Segment Anything Explained](https://www.youtube.com/watch?v=KRAJd4_rNrc) — SAM의 구조와 임팩트를 이해
> - [HuggingFace — SAM](https://huggingface.co/docs/transformers/model_doc/sam) — HuggingFace에서 바로 사용

> **실습**: [SAM2 Interactive Segmentation](https://alexjunholee.github.io/robotics-practice/app.html#hf_sam)
> SAM2 모델을 직접 사용하여 이미지에서 프롬프트 기반 세그멘테이션을 체험할 수 있다 (HuggingFace Space).

### 11.2.3 CLIP

**Vision-Language Model**: 이미지와 텍스트를 공유 임베딩 공간에 매핑한다.

CLIP 이전에는 이미지를 분류하려면 미리 정한 클래스 목록이 필요했다. CLIP은 이미지와 텍스트를 같은 공간에 놓으므로, 임의의 텍스트로 이미지를 검색하거나 분류할 수 있다. "red mug on a wooden table" 같은 자연어로 로봇에게 목표 물체를 지시할 수 있게 된 것이다. open-vocabulary의 시작이고, 로봇이 자연어를 이해하는 기반이 된다.

**학습**: 4억 쌍의 이미지-텍스트 데이터로 contrastive learning

**활용**:
- Zero-shot image classification
- Image-text retrieval
- Open-vocabulary detection의 기반

```python
import clip
import torch

model, preprocess = clip.load("ViT-B/32", device="cuda")

image = preprocess(Image.open("image.jpg")).unsqueeze(0).to("cuda")
text = clip.tokenize(["a dog", "a cat", "a car"]).to("cuda")

with torch.no_grad():
    image_features = model.encode_image(image)
    text_features = model.encode_text(text)

    similarity = (image_features @ text_features.T).softmax(dim=-1)
    print(similarity)  # 각 텍스트와 이미지 간 유사도
```

`@`는 행렬 곱(dot product)이다. 이미지 feature와 텍스트 feature의 코사인 유사도를 계산하는 것인데, 이미지와 각 텍스트가 얼마나 의미적으로 가까운지를 수치로 나타낸다. 이것이 zero-shot classification의 원리이다.

> **추천 자료**
> - [Radford et al., "Learning Transferable Visual Models From Natural Language Supervision" (2021)](https://arxiv.org/abs/2103.00020) — CLIP 원논문
> - [OpenAI CLIP GitHub](https://github.com/openai/CLIP) — 공식 코드 및 사전학습 모델
> - [Yannic Kilcher — CLIP 설명](https://www.youtube.com/watch?v=T9XSU0pKX2E) — CLIP의 아이디어를 잘 풀어서 설명
> - [HuggingFace — CLIP](https://huggingface.co/docs/transformers/model_doc/clip) — HuggingFace에서 다양한 CLIP 변형 사용

### 11.2.4 Depth Anything

**Monocular Depth Foundation Model**: 단일 이미지에서 상대적 깊이를 추정한다.

10.7에서 depth estimation을 다뤘는데, Depth Anything은 그것을 foundation model 수준으로 끌어올린 모델이다. 1.5M 라벨 데이터에 62M 비라벨 데이터까지 활용해서, 실내(NYU), 실외(KITTI), zero-shot 도메인에서 안정적으로 깊이를 추정한다. 단, 학습 데이터와 크게 다른 도메인(내시경, 수중 등)에서는 정확도가 떨어질 수 있다. 로봇이 새로운 환경에 배치될 때 추가 학습 없이 바로 깊이 정보를 얻을 수 있다는 장점이 있다.

**특징**:
- 1.5M 라벨 데이터 + 62M 비라벨 데이터 학습
- 다양한 도메인에서 강건
- V2: 더 정확한 절대 깊이

Depth Anything V2는 V1에서 한 걸음 더 나아가, metric depth (절대 깊이)를 추정할 수 있는 버전도 제공한다. 로보틱스에서는 상대 깊이보다 "정확히 몇 미터 떨어져 있는가"가 중요한 경우가 많으므로, V2의 metric 버전에 주목하자.

> **추천 자료**
> - [Yang et al., "Depth Anything: Unleashing the Power of Large-Scale Unlabeled Data" (2024)](https://arxiv.org/abs/2401.10891) — Depth Anything 원논문
> - [Yang et al., "Depth Anything V2" (2024)](https://arxiv.org/abs/2406.09414) — V2 원논문. Metric depth 지원
> - [Depth Anything GitHub](https://github.com/LiheYoung/Depth-Anything) — 공식 코드
> - [HuggingFace — Depth Anything](https://huggingface.co/docs/transformers/model_doc/depth_anything) — HuggingFace에서 바로 사용

> **실습**: [Depth Anything V2](https://alexjunholee.github.io/robotics-practice/app.html#hf_depth)
> Depth Anything V2 모델로 이미지에서 깊이를 추정하는 과정을 직접 체험할 수 있다 (HuggingFace Space).

### 11.2.5 GroundingDINO

**Open-Vocabulary Object Detection**: 텍스트 프롬프트로 임의의 객체를 탐지한다.

기존 detection 모델(YOLO, Faster R-CNN 등)은 학습에 사용된 클래스만 탐지할 수 있었다. "person, car, dog"으로 학습하면 "coffee mug"은 찾지 못한다. GroundingDINO는 CLIP처럼 텍스트로 어떤 물체든 지정해서 탐지할 수 있다. 로봇에게 "저기 있는 빨간 컵 찾아"라고 자연어로 지시하면, 학습 없이 바로 찾을 수 있다.

```
입력: 이미지 + "person. car. traffic light."
출력: 해당 객체들의 bounding box
```

**Grounded-SAM**: GroundingDINO + SAM 결합
→ 텍스트 프롬프트로 객체 탐지 + 세그멘테이션

Grounded-SAM은 로보틱스에서 실용적인 조합이다. "red cup"이라고 텍스트를 넣으면 GroundingDINO가 bounding box를 찾고, SAM이 그 안에서 정확한 마스크를 생성한다. 별도 학습 없이도 임의의 물체를 탐지하고 분할할 수 있어서, manipulation 파이프라인에서 활발히 쓰인다.

> **추천 자료**
> - [Liu et al., "Grounding DINO: Marrying DINO with Grounded Pre-Training for Open-Set Object Detection" (2023)](https://arxiv.org/abs/2303.05499) — GroundingDINO 원논문
> - [Grounded-SAM GitHub](https://github.com/IDEA-Research/Grounded-Segment-Anything) — 텍스트 기반 탐지+분할 파이프라인
> - [HuggingFace — Grounding DINO](https://huggingface.co/docs/transformers/model_doc/grounding-dino) — HuggingFace에서 사용

> **실습**: [Grounding DINO Demo](https://alexjunholee.github.io/robotics-practice/app.html#hf_grounding_dino)
> 텍스트 프롬프트로 이미지에서 임의의 객체를 탐지하는 Open-Vocabulary Detection을 직접 체험할 수 있다 (HuggingFace Space).

---

## 11.3 VFM의 Spatial AI 응용

앞서 배운 VFM들이 실제 로보틱스 시스템에서 어떻게 결합되어 쓰이는지를 본다. 개별 모델의 능력도 중요하지만, 이것들을 조합해서 공간을 이해하는 AI를 만드는 것이 로보틱스의 목표이다.

**Open-vocabulary Scene Understanding**:
- 사전 정의된 클래스 없이 장면 이해
- "navigate to the red chair" 같은 자연어 명령 처리

로봇이 실제 환경에서 동작하려면 미리 정해둔 물체 목록에 의존하면 안 된다. 사람의 자연어 명령을 이해하고 그에 해당하는 물체를 찾아서 행동해야 한다. CLIP + SAM + GroundingDINO 조합으로 이 파이프라인을 구현할 수 있다.

**Zero-shot Semantic Segmentation**:
- 새로운 환경에서 라벨링 없이 segmentation
- CLIP + SAM 조합으로 구현

**Dense Feature for SLAM**:
- DINOv2 features를 특징점 대신 사용
- 텍스처 없는 영역에서도 매칭 가능
- 최근 연구: DROID-SLAM + DINOv2

현장에서 겪는 문제와 직결된다. 고전 SLAM은 ORB, SIFT 같은 특징점에 의존하는데, 텍스처가 없는 벽면이나 바닥에서는 특징점이 잘 잡히지 않는다. DINOv2의 dense feature는 시맨틱 정보를 포함하고 있어서 하얀 벽이라도 이 부분과 저 부분을 구분할 수 있다. SLAM의 robustness가 높아지는 이유다.

**3D Scene Understanding**:
- 2D VFM features를 3D로 리프팅
- Semantic NeRF, Feature 3DGS

2D에서 추출한 VFM feature를 3D 표현(NeRF, 3D Gaussian Splatting)에 심어넣으면 3D 공간 자체에 시맨틱 정보를 담을 수 있다. "이 3D 맵에서 의자는 어디에 있지?"라는 질문에 텍스트 쿼리로 답할 수 있게 된다. 이 방향의 연구가 Spatial AI에서 늘고 있다(LERF, LangSplat, ConceptGraphs 등).

> **추천 자료**
> - [Kerr et al., "LERF: Language Embedded Radiance Fields" (2023)](https://arxiv.org/abs/2303.09553) — CLIP feature를 NeRF에 심는 연구. Spatial AI의 대표적 예
> - [Tschernezki et al., "Neural Feature Fusion Fields: 3D Distillation of Self-Supervised 2D Image Representations" (2022)](https://arxiv.org/abs/2209.03494) — 2D feature를 3D로 올리는 초기 연구
> - [Papers With Code — 3D Scene Understanding](https://paperswithcode.com/task/3d-scene-understanding) — 최신 연구 동향

---

## 11.4 경량화 및 Edge 배포

로봇의 Local Module에서 사용하려면 경량화가 필요하다.

VFM은 성능은 좋지만, 수억 개의 파라미터가 있어서 GPU 서버가 아니면 실시간으로 돌리기 어렵다. 반면 로봇은 Jetson이나 임베디드 보드에서 30FPS로 돌려야 한다. 이 간극을 메우는 것이 경량화와 Edge 배포 기술이다. 아무리 좋은 모델도 로봇에서 실시간으로 돌릴 수 없으면 논문 안에서만 빛난다.

**경량화 기법**:
| 기법 | 설명 |
|------|------|
| **Distillation** | 큰 모델의 지식을 작은 모델로 전이 |
| **Quantization** | FP32 → INT8/INT4로 precision 감소 |
| **Pruning** | 중요하지 않은 weight 제거 |

각 기법의 trade-off를 파악하는 것이 중요하다. Quantization은 모델 구조를 바꾸지 않으므로 적용이 가장 쉽고, Pruning은 실제 연산량을 줄이지만 정확도 손실이 따를 수 있다. Distillation은 아예 작은 모델을 새로 학습시키므로 효과가 가장 크지만 비용도 높다.

**경량 VFM**:
- **FastSAM**: SAM의 경량 버전 (YOLO 기반)
- **MobileSAM**: 모바일용 SAM
- **EfficientViT-SAM**: 효율적인 ViT 백본

**Edge 배포 도구**:
- **TensorRT**: NVIDIA GPU 최적화
- **ONNX Runtime**: 크로스 플랫폼
- **TFLite**: 모바일/임베디드

```python
# TensorRT 변환 예시 (PyTorch → ONNX → TensorRT)
import torch

# 1. ONNX 내보내기
torch.onnx.export(model, dummy_input, "model.onnx")

# 2. TensorRT 변환 (trtexec 사용)
# trtexec --onnx=model.onnx --saveEngine=model.trt --fp16
```

NVIDIA Jetson을 쓴다면 TensorRT가 거의 필수이다. FP16 변환만으로도 속도가 2-3배 빨라지면서 정확도 손실은 거의 없다. INT8까지 가면 더 빨라지지만 calibration 데이터가 필요하다.

> **추천 자료**
> - [NVIDIA TensorRT 문서](https://docs.nvidia.com/deeplearning/tensorrt/) — TensorRT 사용법과 최적화 가이드
> - [ONNX Runtime](https://onnxruntime.ai/) — 크로스 플랫폼 추론 최적화
> - [MobileSAM GitHub](https://github.com/ChaoningZhang/MobileSAM) — SAM의 모바일 경량화 버전
> - [FastSAM GitHub](https://github.com/CASIA-IVA-Lab/FastSAM) — YOLO 기반 SAM 경량화
> - [NVIDIA Jetson AI Courses](https://developer.nvidia.com/embedded/learn/jetson-ai-certification-programs) — 엣지 배포 실습

---

## 11.5 심화: VFM Fine-tuning과 Adaptation

*연구자가 되고 싶다면 여기서부터 읽어라.*

VFM을 그대로 쓰면 zero-shot 성능이 나오지만, 특정 도메인(의료, 위성, 수중 등)에서는 성능이 떨어진다. fine-tuning이 필요한데, 수억 개 파라미터를 전부 학습시키는 것은 비용이 크다. Parameter-efficient fine-tuning(PEFT)은 모델의 극소수 파라미터만 학습하면서도 full fine-tuning에 근접한 성능을 얻는 방법이다.

**Fine-tuning 전략 비교**:

| 전략 | 학습 파라미터 비율 | 성능 | GPU 메모리 | 적용 난이도 |
|------|-------------------|------|-----------|------------|
| **Full fine-tuning** | 100% | 최고 (데이터 충분 시) | 매우 높음 | 낮음 |
| **Linear probing** | <1% (head만) | 낮음 | 낮음 | 매우 낮음 |
| **LoRA** | 0.1~1% | 높음 | 낮음 | 보통 |
| **Adapter** | 1~5% | 높음 | 보통 | 보통 |
| **Prompt tuning** | <0.1% | 보통 | 낮음 | 높음 |

**LoRA (Low-Rank Adaptation)**:

핵심 아이디어: 사전학습된 weight matrix $\mathbf{W}$에 low-rank update를 추가한다.

$$\mathbf{W}' = \mathbf{W} + \Delta\mathbf{W} = \mathbf{W} + \mathbf{B}\mathbf{A}$$

여기서 $\mathbf{W}$는 $d \times d$ 행렬이고, $\mathbf{B}$는 $d \times r$, $\mathbf{A}$는 $r \times d$ 행렬이다 ($r \ll d$). 원래 $\mathbf{W}$의 파라미터 수 $d^2$ 대신 $2dr$개만 학습한다.

예를 들어 $d = 768$, $r = 8$이면, 원래 589,824개의 파라미터 대신 12,288개만 학습한다(약 2%).

```python
from peft import LoraConfig, get_peft_model
from transformers import AutoModelForImageClassification

# 기본 모델 로드
model = AutoModelForImageClassification.from_pretrained(
    "facebook/dinov2-base",
    num_labels=10
)

# LoRA 설정
lora_config = LoraConfig(
    r=16,                      # rank (저랭크 행렬 차원)
    lora_alpha=32,             # scaling factor
    target_modules=["query", "value"],  # attention Q, V에만 적용
    lora_dropout=0.1,
    bias="none",
)

# PEFT 모델 생성
model = get_peft_model(model, lora_config)
model.print_trainable_parameters()
# 출력 예: trainable params: 294,912 || all params: 86,567,178 || trainable%: 0.34%
```

**Adapter**:

Transformer block 사이에 작은 bottleneck layer를 삽입한다. 원래 weight는 고정하고 adapter layer만 학습한다.

```
Input → [Frozen Attention] → [Adapter: down_proj → ReLU → up_proj] → [Frozen FFN] → Output
```

LoRA는 기존 weight에 합쳐지므로 추론 시 추가 비용이 없고, Adapter는 추가 layer이므로 약간의 추론 지연이 생긴다.

**Prompt Tuning**:

입력에 학습 가능한 가상 토큰을 추가한다. 모델 자체는 전혀 건드리지 않고 입력만 조작한다.

- Visual Prompt Tuning(VPT): ViT의 각 layer 입력에 학습 가능한 토큰을 추가한다.
- 파라미터 효율이 가장 높지만, 성능은 LoRA보다 약간 낮은 경향이 있다.

**SAM을 특정 도메인에 맞추기**:

SAM의 prompt encoder에 도메인 특화 자동 prompt 생성기를 붙이는 전략이 자주 쓰인다.

1. **Grid prompt**: 이미지를 NxN 그리드로 나누고 각 교차점을 point prompt로 사용한다.
2. **학습된 prompt generator**: 이미지를 입력받아 자동으로 point/box prompt를 생성하는 경량 네트워크를 학습한다.
3. **LoRA + SAM**: image encoder에 LoRA를 적용하여 도메인 특화 feature를 학습한다.

```python
# SAM + LoRA 적용 예시 (개념적)
from segment_anything import sam_model_registry
from peft import LoraConfig, get_peft_model

sam = sam_model_registry["vit_b"](checkpoint="sam_vit_b.pth")

# Image encoder에만 LoRA 적용
lora_config = LoraConfig(
    r=4,
    lora_alpha=8,
    target_modules=["qkv"],  # SAM attention qkv projection
)

sam.image_encoder = get_peft_model(sam.image_encoder, lora_config)
# mask decoder는 full fine-tuning (파라미터가 적으므로)
for param in sam.mask_decoder.parameters():
    param.requires_grad = True
```

**평가 방법론**:

VFM adaptation 연구에서는 다음 프로토콜로 비교하는 것이 표준이다.

| 프로토콜 | 설명 | 비교 목적 |
|---------|------|----------|
| **Zero-shot** | 학습 없이 바로 평가 | VFM의 기본 범용성 확인 |
| **Few-shot (1/5/10-shot)** | 클래스당 소수 샘플로 학습 | 데이터 효율성 비교 |
| **Full fine-tune** | 전체 학습 데이터 사용 | 상한선 확인 |
| **PEFT (LoRA 등)** | 소수 파라미터로 학습 | 효율성-성능 trade-off |

비교할 때는 동일 backbone, 동일 데이터 split, 동일 augmentation을 써야 공정하다. Few-shot에서는 seed에 따라 결과 분산이 크므로 3~5회 반복 후 평균과 표준편차를 보고해야 한다.

> **추천 자료**
> - [Hu et al., "LoRA: Low-Rank Adaptation of Large Language Models" (2022)](https://arxiv.org/abs/2106.09685) — LoRA 원논문 (LLM용이지만 ViT에도 그대로 적용 가능)
> - [HuggingFace PEFT 라이브러리](https://github.com/huggingface/peft) — LoRA, Adapter 등 PEFT 구현체
> - [Chen et al., "SAM Fails to Segment Anything? — SAM-Adapter" (2023)](https://arxiv.org/abs/2304.09148) — SAM의 도메인 adaptation 사례

---

> **기술 흐름: Vision Foundation Models**
> - **2021**: CLIP (OpenAI) 발표. 이미지-텍스트 공유 임베딩으로 zero-shot 인식의 가능성을 열다. 4억 쌍의 이미지-텍스트 데이터로 학습. open-vocabulary 시대의 시작
> - **2022**: Masked Autoencoders (MAE) 등 self-supervised 사전학습 방법이 주목받기 시작. DINO가 self-supervised ViT의 가능성을 보여줌
> - **2023**: SAM (Segment Anything Model) 발표. 11M 이미지, 1.1B 마스크로 학습. "어떤 물체든 분할"이라는 foundation model 수준의 범용성 달성. 같은 해 DINOv2 발표 — self-supervised 비전 feature의 새 기준
> - **2024**: SAM2 (비디오 segmentation 확장), Depth Anything V2 (metric depth 지원), Florence-2 (통합 비전 모델) 등 VFM이 빠르게 진화. 모델들의 경량화와 edge 배포가 활발해짐
> - **2025~**: VFM들의 3D 확장과 멀티모달 통합이 가속. 하나의 foundation model이 detection, segmentation, depth, tracking을 통합 처리하는 방향. 로보틱스에서는 VFM이 perception의 표준 백본으로 자리잡는 추세
> - **지금 주목할 것**: Foundation model의 핵심 가치는 **zero-shot 능력**이다. 새 환경, 새 물체에서도 추가 학습 없이 작동하므로 로봇의 범용성을 높인다. CLIP+SAM+DINOv2 조합은 NLMap, ConceptGraphs 등에서 open-vocabulary 로봇 인식의 대표적 파이프라인으로 쓰이고 있다. 경량화(FastSAM, MobileSAM)를 통해 실제 로봇에 올리는 것까지가 완전한 파이프라인이다
