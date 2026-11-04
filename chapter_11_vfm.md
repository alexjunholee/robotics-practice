# Ch.11 — Vision Foundation Models (VFM)


앞 장의 모델은 대부분 특정 데이터셋과 태스크에 맞춰 학습했다. Vision Foundation Model(VFM)은 대규모 데이터로 사전학습한 표현을 여러 태스크에 옮겨 쓴다.

---

## 11.1 Foundation Model이란?

Foundation Model은 대규모 데이터로 사전학습해 여러 downstream task에 쓰는 모델이다.

태스크별 모델은 환경이나 인식 대상이 바뀔 때마다 데이터를 다시 모으고 라벨을 붙여 학습해야 한다. Foundation model은 사전학습한 표현을 다시 써서 이 비용을 줄인다. 처음 보는 물체나 환경에서 얼마나 잘 되는지는 모델과 도메인마다 다르다. 그래서 성능을 적을 때는 추가 학습 없이 얻은 zero-shot 결과와 추가 학습한 결과를 나눠 적는다.

특징:
- Scale: 파라미터가 수억~수십억 개다
- Pretraining: 이미지(또는 이미지-텍스트 쌍) 천만~수억 개 규모의 데이터로 사전학습한다
- Zero-shot / Few-shot: 추가 학습 없이, 또는 예제 몇 개만으로 새 태스크를 푼다
- Transfer: 여러 도메인에 옮겨 쓴다

Scaling law는 성능과 모델 크기·데이터·연산량 사이에서 관찰되는 power-law 관계를 가리킨다(Kaplan et al., 2020; ViT는 Zhai et al., 2022). GPT, CLIP, SAM 같은 대형 모델의 규모를 정할 때 이 경험적 관계를 참고한다. Hoffmann et al.(2022)은 정해진 연산량에서 모델 크기와 학습 토큰 수를 비슷한 비율로 함께 늘릴 때 성능이 가장 좋다고 보고했다.

> **추천 자료**
> - [Bommasani et al., "On the Opportunities and Risks of Foundation Models" (2021)](https://arxiv.org/abs/2108.07258) — Foundation Model이라는 용어를 정의한 Stanford 보고서
> - [Two Minute Papers — Foundation Models 관련 영상들](https://www.youtube.com/@TwoMinutePapers) — VFM 연구 결과를 짧게 소개하는 채널
> - [HuggingFace Model Hub](https://huggingface.co/models) — 사전학습 모델을 내려받아 바로 쓸 수 있다

---

## 11.2 주요 VFM

DINOv2는 표현 학습, SAM은 분할, CLIP은 이미지-텍스트 정렬, Depth Anything은 단안 깊이 추정, GroundingDINO는 open-vocabulary 검출에 쓴다.

### 11.2.1 DINOv2

DINOv2는 Meta가 2023년에 공개한 자기지도 학습 Vision Transformer다. 라벨 없는 이미지 가운데 골라 모은 1억 4,200만 장(LVD-142M)으로 학습했다. 이 모델의 feature를 고정하고 거기에 선형 층만 붙여 분류, 분할, 깊이 추정을 한다. 로보틱스에서는 패치마다 나오는 dense feature를 매칭에 써서, 텍스처가 적은 장면에서 SLAM이나 Visual Odometry의 추적을 도우려는 연구가 있다.

특징:
- 학습 목표: DINO의 self-distillation, iBOT의 masked image modeling, feature를 고르게 퍼뜨리는 KoLeo 정규화
- backbone을 고정한 채 여러 태스크에 쓴다
- 패치마다 dense visual feature를 낸다

활용:
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

`last_hidden_state`의 인덱스 0은 이미지 전체를 요약한 [CLS] 토큰이고, 분류에 쓴다. 그 뒤의 토큰에는 패치마다 국소 feature가 하나씩 들어 있고, segmentation이나 feature 매칭 같은 dense prediction에 쓴다.

> **추천 자료**
> - [Oquab et al., "DINOv2: Learning Robust Visual Features without Supervision" (TMLR 2024)](https://arxiv.org/abs/2304.07193) — DINOv2 원논문
> - [DINOv2 GitHub](https://github.com/facebookresearch/dinov2) — 공식 코드와 사전학습 모델
> - [HuggingFace — DINOv2](https://huggingface.co/docs/transformers/model_doc/dinov2) — HuggingFace 사용법
> - [Yannic Kilcher — DINO 설명](https://www.youtube.com/watch?v=h3ij3F3cPIk) — DINOv2도 쓰는 DINO v1의 self-distillation을 설명한다

### 11.2.2 SAM (Segment Anything Model)

Promptable Segmentation: 점, 박스, 마스크 프롬프트를 받아 그 자리의 물체를 분할한다. 텍스트 프롬프트는 원 논문에서 탐색 실험으로만 다뤘다. 공개 checkpoint와 API가 받는 프롬프트는 점·박스·마스크다. 텍스트로 지정하려면 뒤에 나오는 GroundingDINO로 텍스트를 박스로 바꿔 SAM에 넘긴다.

기존 segmentation 모델은 학습한 클래스 목록 안에서만 분할한다. "의자, 테이블, 사람"으로 학습한 모델은 "컵"을 출력하지 못한다. SAM은 클래스 라벨이 없는 마스크 11억 개(1.1B)로 학습해, 프롬프트가 가리키는 물체라면 종류와 상관없이 분할한다. 로봇이 처음 보는 물체를 조작할 때도 SAM으로 물체 마스크를 바로 얻는다. 의료·위성·수중 영상처럼 학습 데이터와 분포가 다른 도메인에서는 성능이 떨어진다(§11.5).

구성:
- Image Encoder: ViT로 이미지를 임베딩한다
- Prompt Encoder: 점, 박스, 마스크 프롬프트를 임베딩한다
- Mask Decoder: 가벼운 디코더로 마스크를 만든다

SAM2는 이미지와 비디오를 함께 처리하고, 원 논문 기준으로 이미지에서 SAM보다 6배 빠르다. 한 프레임에서 포인트나 박스로 물체를 지정하면 비디오의 다른 프레임에서도 그 물체를 추적하며 분할한다. 로봇이 물체를 추적하며 조작하는 작업에 쓴다.

```python
import numpy as np
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

`multimask_output=True`로 두면 마스크 후보가 3개(전체 물체, 부분, 더 작은 부분) 나온다. `scores`는 모델이 예측한 마스크 품질(IoU) 추정치다. 원하는 범위의 마스크는 프롬프트를 바꾸거나 세 후보를 직접 비교해 고른다.

> **추천 자료**
> - [Kirillov et al., "Segment Anything" (2023)](https://arxiv.org/abs/2304.02643) — SAM 원논문
> - [Ravi et al., "SAM 2: Segment Anything in Images and Videos" (2024)](https://arxiv.org/abs/2408.00714) — SAM2 원논문. 비디오 segmentation까지 다룬다
> - [Segment Anything GitHub](https://github.com/facebookresearch/segment-anything) — 공식 코드
> - [Segment Anything Explained](https://www.youtube.com/watch?v=KRAJd4_rNrc) — SAM의 구조를 설명하는 영상
> - [HuggingFace — SAM](https://huggingface.co/docs/transformers/model_doc/sam) — HuggingFace 사용법

> **실습**: [SAM2 Interactive Segmentation](https://alexjunholee.github.io/robotics-practice/app.html#hf_sam)
> SAM2 모델에 프롬프트를 직접 넣어 이미지를 분할해 본다 (HuggingFace Space).

### 11.2.3 CLIP

Vision-Language Model: 이미지와 텍스트를 같은 임베딩 공간에 놓는다.

CLIP 이전의 이미지 분류기는 미리 정한 클래스 목록 안에서 답을 골랐다. CLIP의 공유 공간에서는 아무 문장으로나 이미지를 검색하거나 분류할 수 있다. 그래서 "red mug on a wooden table" 같은 자연어로 로봇에게 목표 물체를 알려 줄 수 있다.

학습: 이미지-텍스트 쌍 4억 개로 contrastive learning을 했다

활용:
- Zero-shot image classification
- Image-text retrieval
- Open-vocabulary detection 모델의 텍스트·이미지 인코더

```python
import clip
import torch
from PIL import Image

model, preprocess = clip.load("ViT-B/32", device="cuda")

image = preprocess(Image.open("image.jpg")).unsqueeze(0).to("cuda")
text = clip.tokenize(["a dog", "a cat", "a car"]).to("cuda")

with torch.no_grad():
    image_features = model.encode_image(image)
    text_features = model.encode_text(text)

    # CLIP은 L2 정규화 후 내적해야 코사인 유사도가 된다
    image_features /= image_features.norm(dim=-1, keepdim=True)
    text_features /= text_features.norm(dim=-1, keepdim=True)
    similarity = (100.0 * image_features @ text_features.T).softmax(dim=-1)  # 100.0 ≈ model.logit_scale.exp(), 학습된 온도 스케일
    print(similarity)  # 각 텍스트와 이미지 간 유사도
```

`@`는 행렬 곱이다. 두 feature를 L2 정규화한 뒤 내적하면 그 값이 코사인 유사도다. 여기에 학습된 스케일을 곱하고 클래스 문장들에 대해 softmax를 취하면 zero-shot classification 확률이 나온다. 정규화를 빼먹으면 벡터 길이가 점수에 섞인다.

> **추천 자료**
> - [Radford et al., "Learning Transferable Visual Models From Natural Language Supervision" (2021)](https://arxiv.org/abs/2103.00020) — CLIP 원논문
> - [OpenAI CLIP GitHub](https://github.com/openai/CLIP) — 공식 코드와 사전학습 모델
> - [Yannic Kilcher — CLIP 설명](https://www.youtube.com/watch?v=T9XSU0pKX2E) — CLIP 논문을 풀어 설명하는 영상
> - [HuggingFace — CLIP](https://huggingface.co/docs/transformers/model_doc/clip) — 여러 CLIP 변형의 HuggingFace 사용법

### 11.2.4 Depth Anything

Monocular Depth Foundation Model: 이미지 한 장에서 상대 깊이를 추정한다.

Depth Anything은 라벨 있는 이미지 150만 장(1.5M)과 라벨 없는 이미지 6,200만 장(62M)으로 학습한 단안 깊이 모델이다. 논문은 실내(NYU)와 실외(KITTI)를 포함해 학습에 쓰지 않은 여러 데이터셋에서 zero-shot으로 평가했다. 내시경·수중 영상처럼 학습 데이터와 크게 다른 영상에서는 정확도가 떨어질 수 있다.

특징:
- V1은 NYUv2·KITTI로, V2는 합성 데이터셋 Hypersim(실내)·Virtual KITTI 2(실외)로 파인튜닝한 metric depth 모델을 공개했다
- V2는 합성 이미지로 teacher를 다시 학습해 상대 깊이의 세부 품질을 높였다

물리 단위의 거리가 필요한 로봇 시스템에는 상대 깊이 모델 대신 metric depth(절대 깊이) 모델을 쓴다.

> **추천 자료**
> - [Yang et al., "Depth Anything: Unleashing the Power of Large-Scale Unlabeled Data" (2024)](https://arxiv.org/abs/2401.10891) — Depth Anything 원논문
> - [Yang et al., "Depth Anything V2" (2024)](https://arxiv.org/abs/2406.09414) — V2 원논문
> - [Depth Anything GitHub](https://github.com/LiheYoung/Depth-Anything) — 공식 코드
> - [HuggingFace — Depth Anything](https://huggingface.co/docs/transformers/model_doc/depth_anything) — HuggingFace 사용법

> **실습**: [Depth Anything V2](https://alexjunholee.github.io/robotics-practice/app.html#hf_depth)
> Depth Anything V2로 이미지의 깊이를 직접 추정해 본다 (HuggingFace Space).

### 11.2.5 GroundingDINO

Open-Vocabulary Object Detection: 텍스트 프롬프트로 지정한 물체를 탐지한다.

YOLO나 Faster R-CNN 같은 closed-set detector는 학습할 때 정한 클래스 안에서만 예측한다. GroundingDINO는 텍스트 질의를 받아, 그 표현에 해당하는 이미지 영역의 bounding box를 돌려준다. "red cup" 같은 표현을 그대로 질의로 넣으면 되고, 클래스를 늘릴 때 출력 head를 다시 학습하지 않는다.

```
입력: 이미지 + "person. car. traffic light."
출력: 해당 객체들의 bounding box
```

Grounded-SAM은 GroundingDINO와 SAM을 이어 텍스트 프롬프트로 탐지와 분할을 함께 한다. GroundingDINO가 "red cup" 같은 질의에 해당하는 bounding box를 찾으면, SAM이 그 상자를 프롬프트로 받아 mask를 만든다. 이 mask를 manipulation pipeline에 넘긴다.

> **추천 자료**
> - [Liu et al., "Grounding DINO: Marrying DINO with Grounded Pre-Training for Open-Set Object Detection" (2023)](https://arxiv.org/abs/2303.05499) — GroundingDINO 원논문
> - [Grounded-SAM GitHub](https://github.com/IDEA-Research/Grounded-Segment-Anything) — 텍스트 기반 탐지+분할 파이프라인
> - [HuggingFace — Grounding DINO](https://huggingface.co/docs/transformers/model_doc/grounding-dino) — HuggingFace 사용법

> **실습**: [Grounding DINO Demo](https://alexjunholee.github.io/robotics-practice/app.html#hf_grounding_dino)
> 텍스트 프롬프트를 넣어 이미지에서 물체를 탐지해 본다 (HuggingFace Space).

---

## 11.3 VFM의 Spatial AI 응용

로봇 시스템에서는 위 VFM들을 조합해 쓴다.

Open-vocabulary Scene Understanding:
- 미리 정한 클래스 목록 없이 장면 속 물체를 알아본다
- "navigate to the red chair" 같은 자연어 명령을 처리한다

가정이나 사무실의 로봇은 미리 정한 목록에 없는 물체를 자주 만난다. 자연어 명령에 맞는 물체를 찾아 움직이는 파이프라인은 CLIP, SAM, GroundingDINO를 조합해 만든다.

Zero-shot Semantic Segmentation:
- 새 환경에서 라벨을 더 달지 않고 분할한다
- SAM으로 마스크를 만들고 CLIP으로 마스크마다 클래스 이름을 붙인다

Dense Feature for SLAM:
- DINOv2 feature를 특징점 대신 쓴다
- 텍스처가 적은 영역의 매칭에 쓴다

고전 SLAM은 ORB, SIFT 같은 특징점을 쓰는데, 텍스처가 적은 벽면이나 바닥에서는 특징점이 적게 나온다. DINOv2의 dense feature에는 주변 넓은 영역의 의미 정보가 섞여 있어서, 이런 곳에서도 위치마다 값이 달라진다. 이 feature로 매칭하면 텍스처가 적은 장면에서 추적이 끊기는 일을 줄일 수 있다.

3D Scene Understanding:
- 2D VFM feature를 3D 표현으로 옮긴다
- Semantic NeRF, Feature 3DGS

2D 이미지에서 뽑은 VFM feature를 3D 표현(NeRF, 3D Gaussian Splatting, 3D scene graph)에 넣으면 3D 맵의 각 위치에 의미 정보가 붙는다. 그러면 "이 맵에서 의자는 어디 있지?"를 텍스트 질의로 찾을 수 있다. LERF, LangSplat, ConceptGraphs가 이렇게 한다.

> **추천 자료**
> - [Kerr et al., "LERF: Language Embedded Radiance Fields" (2023)](https://arxiv.org/abs/2303.09553) — CLIP feature를 NeRF에 함께 학습시켜 텍스트로 3D 위치를 찾는다
> - [Tschernezki et al., "Neural Feature Fusion Fields: 3D Distillation of Self-Supervised 2D Image Representations" (2022)](https://arxiv.org/abs/2209.03494) — 2D 자기지도 feature를 3D 표현으로 distillation한 연구
> - [Papers With Code — 3D Scene Understanding](https://paperswithcode.com/task/3d-scene-understanding) — Papers With Code는 2025년에 문을 닫았다(Ch.20 §20.4). 이 주소는 지금 Hugging Face의 trending 논문 목록으로 넘어간다

---

## 11.4 경량화 및 Edge 배포

로봇에 탑재한 연산 장치에서 VFM을 돌리려면 모델을 가볍게 만들어야 한다.

VFM은 파라미터가 수억 개라 연산과 메모리를 많이 쓴다. ViT-B급 모델은 데스크톱 GPU에서는 무리 없이 돌지만, 현장 로봇은 같은 모델을 온보드 연산 장치에서 다른 모듈과 자원을 나눠 쓰며 돌린다. 초당 몇 장을 처리해야 하는지는 제어 루프와 작업이 정하고, 허용 지연도 검색·지도 갱신·조작마다 다르다. 이 지연 안에 맞추려고 아래 경량화와 Edge 배포 기법을 쓴다.

경량화 기법:
| 기법 | 설명 |
|------|------|
| Distillation | 큰 모델의 출력을 작은 모델이 따라 하도록 학습 |
| Quantization | 수치 정밀도를 FP32에서 INT8/INT4로 낮춤 |
| Pruning | 기여가 작은 weight를 지움 |

Quantization은 네트워크 구조를 그대로 두고 정밀도만 낮추므로 적용하기 쉽다. Pruning은 어떤 weight를 지울지 기준을 정해야 하고, 채널이나 블록 단위로 지우는 구조적 pruning이어야 실제 연산량이 줄어든다. Distillation은 작은 student 모델의 구조를 새로 정할 수 있어 크기를 많이 줄일 수 있지만, student를 학습하는 비용이 따로 든다.

경량 VFM:
- FastSAM: YOLOv8-seg로 SAM의 작업을 대신한다
- MobileSAM: SAM의 image encoder를 작은 ViT로 바꿨다
- EfficientViT-SAM: SAM의 image encoder를 EfficientViT로 바꿨다

Edge 배포 도구:
- TensorRT: NVIDIA GPU용 추론 최적화
- ONNX Runtime: 여러 하드웨어와 OS에서 ONNX 모델을 실행
- TFLite(2024년 9월 LiteRT로 개명): 모바일·임베디드 기기용

```python
# TensorRT 변환 예시 (PyTorch → ONNX → TensorRT)
import torch

# 1. ONNX 내보내기
torch.onnx.export(model, dummy_input, "model.onnx")

# 2. TensorRT 변환 (trtexec 사용)
# trtexec --onnx=model.onnx --saveEngine=model.trt --fp16
```

NVIDIA Jetson에서는 추론 backend로 TensorRT를 많이 쓴다. FP16·INT8로 바꿨을 때 latency, 메모리, 태스크 정확도가 얼마나 달라지는지는 model graph, 입력 크기, batch, Jetson power mode, TensorRT·CUDA 버전에 따라 다르다. 그래서 실제로 쓸 장치에서 FP32 기준 모델과 같은 validation set으로 잰다. INT8은 실제 입력을 대표하는 calibration 데이터로 scale을 정한 뒤 정확도를 다시 잰다.

> **추천 자료**
> - [NVIDIA TensorRT 문서](https://docs.nvidia.com/deeplearning/tensorrt/) — TensorRT 사용법과 최적화 가이드
> - [ONNX Runtime](https://onnxruntime.ai/) — 여러 플랫폼에서 쓰는 추론 엔진
> - [MobileSAM GitHub](https://github.com/ChaoningZhang/MobileSAM) — 모바일용으로 줄인 SAM
> - [FastSAM GitHub](https://github.com/CASIA-LMC-Lab/FastSAM) — YOLO 기반으로 SAM을 대신하는 모델
> - [NVIDIA Jetson Tutorials](https://developer.nvidia.com/embedded/learn/tutorials) — Jetson 튜토리얼 모음

---

## 11.5 심화: VFM Fine-tuning과 Adaptation

의료, 위성, 수중 영상에서는 VFM의 zero-shot 성능이 떨어져서 파인튜닝을 한다. 파라미터 수억 개를 전부 학습하면 비용이 크다. Parameter-efficient fine-tuning(PEFT)은 파라미터 일부만 학습해 full fine-tuning에 가까운 성능을 얻는다.

Fine-tuning 전략 비교:

| 전략 | 학습 파라미터 비율 | 성능 | GPU 메모리 | 적용 난이도 |
|------|-------------------|------|-----------|------------|
| Full fine-tuning | 100% | 최고 (데이터 충분 시) | 매우 높음 | 낮음 |
| Linear probing | <1% (head만) | 낮음 | 낮음 | 매우 낮음 |
| LoRA | 0.1~1% | 높음 | 낮음 | 보통 |
| Adapter | 1~5% | 높음 | 보통 | 보통 |
| Prompt tuning | <0.1% | 보통 | 낮음 | 높음 |

LoRA (Low-Rank Adaptation):

LoRA는 사전학습한 weight matrix $\mathbf{W}$는 고정하고, 거기에 더할 low-rank 행렬만 학습한다.

$$\mathbf{W}' = \mathbf{W} + \Delta\mathbf{W} = \mathbf{W} + \mathbf{B}\mathbf{A}$$

$\mathbf{W} \in \mathbb{R}^{d \times d}$이고, $\mathbf{B} \in \mathbb{R}^{d \times r}$, $\mathbf{A} \in \mathbb{R}^{r \times d}$다 ($r \ll d$). 원래 $\mathbf{W}$의 파라미터 수 $d^2$ 대신 $2dr$개만 학습한다.

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
# 출력 예: trainable params: 589,824 || all params: 87,185,674 || trainable%: 0.68%
```

Adapter:

Adapter는 Transformer block 안에 차원을 줄였다 늘리는 작은 bottleneck layer를 끼워 넣는다. 원래 weight는 고정하고 adapter layer만 학습한다.

```
Input → [Frozen Attention] → [Adapter: down_proj → ReLU → up_proj] → [Frozen FFN] → Output
```

LoRA는 학습 후 $\mathbf{B}\mathbf{A}$를 $\mathbf{W}$에 더해 병합하므로, 추론 때의 연산량은 원래 모델과 같다. Adapter는 추가 레이어를 거치므로 추론 지연이 조금 늘어난다.

Prompt Tuning:

Prompt tuning은 입력에 학습 가능한 가상 토큰을 덧붙이고, 모델 weight는 고정한 채 그 토큰만 학습한다.

- Visual Prompt Tuning(VPT): VPT-Shallow는 첫 layer 입력에만, VPT-Deep은 ViT의 모든 layer 입력에 학습 가능한 토큰을 붙인다.
- 위 표의 전략 가운데 학습하는 파라미터가 가장 적다.

SAM을 특정 도메인에 맞추기:

SAM을 새 도메인에 맞출 때는 prompt를 주는 방식을 바꾸거나 image encoder를 파인튜닝한다.

1. Grid prompt: 이미지에 N×N 격자를 놓고 격자점마다 point prompt를 준다.
2. 학습한 prompt generator: 이미지를 받아 point·box prompt를 자동으로 만드는 작은 네트워크를 학습한다.
3. LoRA + SAM: image encoder에 LoRA를 붙여 그 도메인에 맞는 feature를 학습한다.

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

평가 방법론:

VFM adaptation 방법을 비교할 때는 아래 평가 프로토콜을 쓴다.

| 프로토콜 | 설명 | 비교 목적 |
|---------|------|----------|
| Zero-shot | 학습 없이 바로 평가 | 추가 학습 없이 얼마나 되는지 확인 |
| Few-shot (1/5/10-shot) | 클래스당 샘플 몇 개로 학습 | 적은 데이터로 얼마나 되는지 비교 |
| Full fine-tune | 학습 데이터 전체로 모든 파라미터 학습 | 성능 상한 확인 |
| PEFT (LoRA 등) | 파라미터 일부만 학습 | 학습 비용과 성능의 trade-off |

비교할 때는 backbone, 데이터 split, augmentation을 같게 맞춘다. Few-shot 결과는 어떤 샘플을 뽑았는지(seed)에 따라 크게 달라지므로, 3~5번 반복해 평균과 표준편차를 적는다.

> **추천 자료**
> - [Hu et al., "LoRA: Low-Rank Adaptation of Large Language Models" (2022)](https://arxiv.org/abs/2106.09685) — LoRA 원논문. LLM을 대상으로 했지만 같은 방식을 ViT의 attention 층에도 쓴다
> - [HuggingFace PEFT 라이브러리](https://github.com/huggingface/peft) — LoRA, Adapter 같은 PEFT 구현
> - [Chen et al., "SAM Fails to Segment Anything? — SAM-Adapter" (2023)](https://arxiv.org/abs/2304.09148) — SAM의 도메인 adaptation 사례

---

> **기술 흐름: Vision Foundation Models**
> - 2021: OpenAI가 CLIP을 발표했다(§11.2.3). 같은 해 DINO(ICCV 2021)는 자기지도 학습한 ViT의 attention map에 물체 경계가 드러난다고 보고했다.
> - 2022: MAE(CVPR 2022) 같은 masked image modeling 사전학습이 나왔다.
> - 2023: SAM이 나왔다(§11.2.2). 학습 이미지는 1,100만 장이다. 같은 해 DINOv2도 나왔다. Florence-2(여러 비전 태스크를 텍스트 프롬프트로 처리하는 모델)는 2023년 11월 arXiv에 공개됐고 CVPR 2024에 실렸다.
> - 2024: SAM2(비디오 segmentation)와 Depth Anything V2가 나왔다.
> - 2025: Meta가 DINOv3(8월, 이미지 17억 장으로 자기지도 학습)와 SAM 3(11월, 짧은 명사구 텍스트나 예시 이미지로 지정한 개념의 모든 개체를 이미지·비디오에서 분할·추적)를 공개했다.
> - 로봇 인식: NLMap과 ConceptGraphs 같은 시스템은 CLIP, SAM 같은 모델의 출력을 엮어 open-vocabulary 인식을 한다. 로봇에 올릴 때는 FastSAM, MobileSAM 같은 경량 모델의 지연 시간과 정확도를 함께 잰다.
