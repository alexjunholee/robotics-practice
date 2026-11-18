# 11장 비전 파운데이션 모델


10장에서 살펴본 인식 모델은 대부분 특정 데이터셋과 태스크에 맞춰 학습한다. 하지만 로봇이 실제로 활동하는 가정이나 물류 창고, 야외 현장에는 학습 과정에서 한 번도 보지 못한 물체와 조명, 배경이 수시로 등장한다. 환경이 바뀔 때마다 새 데이터를 모아 라벨을 붙이고 모델을 다시 학습하는 방식으로는 이러한 변화에 대응하기 어렵다. Vision Foundation Model(VFM)은 인터넷 규모의 방대한 데이터로 시각 표현을 미리 학습해 두고, 그 표현을 여러 다운스트림 태스크에 그대로 옮겨 쓰는 접근으로 이 한계를 넘는다.

---

## 11.1 Foundation Model의 정의

Foundation Model은 대규모 데이터로 사전학습(pretraining)한 뒤 다양한 다운스트림 태스크(downstream task)에 폭넓게 활용하는 범용 모델이다. 기존 태스크별 모델이 인식 대상이나 작업 환경이 달라질 때마다 데이터 수집과 라벨링, 재학습을 처음부터 반복해야 했다면, Foundation Model은 이미 풍부한 시각 구조를 익힌 표현을 재사용해 적응 비용을 크게 낮춘다. 물론 낯선 물체나 새로운 환경에서도 곧바로 제 성능을 발휘할지는 모델 구조와 도메인 차이에 따라 달라진다. 그래서 연구나 실무에서는 추가 학습 없이 바로 평가한 zero-shot 성능과 소량의 도메인 데이터로 추가 학습한 성능을 구분해 보고하는 것이 원칙이다.

이처럼 한 번의 사전학습으로 여러 작업을 감당할 수 있는 비전 파운데이션 모델은 크게 네 가지 특성으로 요약된다.

- **규모(Scale)**: 수억 개에서 수십억 개에 이르는 방대한 파라미터를 지닌다.
- **사전학습(Pretraining)**: 수천만에서 수억 장 규모의 이미지 또는 이미지-텍스트 쌍으로 범용 표현을 익힌다.
- **Zero-shot / Few-shot 적응**: 별도의 추가 학습 없이, 혹은 몇 개의 예시만으로도 새로운 태스크를 풀어낸다.
- **범용 전이(Transfer)**: 분류나 분할뿐 아니라 로봇 제어, 3D 복원 등 여러 도메인으로 표현을 옮겨 쓴다.

단순히 모델과 데이터를 키우는 데 그치지 않고, 얼마나 키웠을 때 성능이 얼마나 오르는지를 가늠하는 경험적 기준도 확립되었다. 바로 **scaling law**다. 이는 모델 크기, 학습 데이터 규모, 연산량과 테스트 성능 사이에 성립하는 멱법칙(power-law) 관계를 가리킨다(Kaplan et al., 2020; ViT 분야는 Zhai et al., 2022). GPT, CLIP, SAM 같은 초대형 모델의 크기를 설계할 때도 이 경험적 관계가 나침반 역할을 한다. 특히 Hoffmann et al.(2022)은 주어진 연산 예산 안에서 최대 성능을 얻으려면 모델 크기만 키우기보다 모델 파라미터 수와 학습 토큰 수를 비슷한 비율로 함께 늘려야 한다고 보고했다.

> **추천 자료**
> - [Bommasani et al., "On the Opportunities and Risks of Foundation Models" (2021)](https://arxiv.org/abs/2108.07258) — Foundation Model이라는 용어를 정의한 Stanford 보고서
> - [Two Minute Papers — Foundation Models 관련 영상들](https://www.youtube.com/@TwoMinutePapers) — VFM 연구 결과를 짧게 소개하는 채널
> - [HuggingFace Model Hub](https://huggingface.co/models) — 사전학습 모델을 내려받아 바로 쓸 수 있는 공개 저장소

---

## 11.2 주요 VFM

현장의 로봇 시스템에서 파운데이션 모델을 활용할 때는 한 모델로 모든 문제를 풀기보다 역할에 따라 대표 모델들을 골라 쓰거나 조합한다. 범용 시각 표현을 뽑아 주는 **DINOv2**, 어떤 물체든 프롬프트로 오려 내는 **SAM**, 이미지와 자연어를 같은 공간에 연결하는 **CLIP**, 한 장의 사진에서 입체적인 거리감을 읽는 **Depth Anything**, 그리고 텍스트 질의만으로 물체 위치를 찾아내는 **GroundingDINO**가 대표적이다. 각각의 원리와 사용법을 차례로 살펴보자.

### 11.2.1 DINOv2

DINOv2는 Meta가 2023년에 공개한 자기지도 학습(self-supervised learning) 기반 Vision Transformer다. 사람의 라벨링 없이 정제해 모은 1억 4,200만 장의 이미지(LVD-142M)만으로 학습했음에도, 가중치(backbone)를 완전히 고정한 채 가벼운 선형 층만 얹어 분류, 분할, 깊이 추정까지 두루 풀어낸다. 로보틱스에서는 특히 이미지 패치마다 추출되는 조밀한 특징(dense feature)이 주목받는다. 특징점이 잘 잡히지 않는 밋밋한 벽면이나 바닥에서도 패치별 시각 문맥이 뚜렷해, SLAM이나 Visual Odometry의 영상 추적 안정성을 높이는 데 활용된다.

DINOv2가 이처럼 정교한 표현을 배울 수 있었던 비결은 학습 목표와 출력 구조에 있다.

- **학습 목표**: DINO의 self-distillation과 iBOT의 masked image modeling을 결합하고, 특징 벡터가 한곳으로 쏠리지 않고 공간에 고르게 퍼지도록 KoLeo 정규화를 더했다.
- **고정 백본 활용**: 사전학습된 backbone을 얼린 상태에서도 여러 다운스트림 태스크에 바로 투입할 수 있다.
- **패치별 밀집 특징**: 이미지 전체 요약뿐 아니라 개별 패치마다 세밀한 시각 특징 벡터를 출력한다.

이 특성 덕분에 이미지 검색(image retrieval), 선형 탐색(linear probe) 기반의 의미 분할(semantic segmentation), SLAM/VO의 특징점 매칭, 3D 재구성 파이프라인의 특징 추출 백본 등 다양한 곳에 쓰인다. 실제로 DINOv2가 이미지 한 장에서 어떤 구조의 텐서를 내놓는지 HuggingFace 코드로 확인해 보자.

```python
import torch
from transformers import AutoModel, AutoImageProcessor

processor = AutoImageProcessor.from_pretrained('facebook/dinov2-base')
model = AutoModel.from_pretrained('facebook/dinov2-base')

inputs = processor(images=image, return_tensors="pt")
outputs = model(**inputs)
features = outputs.last_hidden_state  # (1, num_patches+1, 768): CLS + 패치 feature
```

출력 텐서 `last_hidden_state`의 0번 인덱스는 이미지 전체의 맥락을 압축한 `[CLS]` 토큰으로, 이미지 분류나 전체 장면 검색에 쓰인다. 그 뒤로 이어지는 토큰들에는 각 이미지 패치에 대응하는 국소 특징이 담겨 있어, 픽셀 단위 분할이나 서로 다른 두 시점 영상 사이의 특징 매칭 같은 밀집 예측(dense prediction) 작업에 바로 활용할 수 있다.

> **추천 자료**
> - [Oquab et al., "DINOv2: Learning Robust Visual Features without Supervision" (TMLR 2024)](https://arxiv.org/abs/2304.07193) — DINOv2 원논문
> - [DINOv2 GitHub](https://github.com/facebookresearch/dinov2) — 공식 코드와 사전학습 모델
> - [HuggingFace — DINOv2](https://huggingface.co/docs/transformers/model_doc/dinov2) — HuggingFace 사용법
> - [Yannic Kilcher — DINO 설명](https://www.youtube.com/watch?v=h3ij3F3cPIk) — DINOv2도 쓰는 DINO v1의 self-distillation을 설명한다

### 11.2.2 SAM (Segment Anything Model)

기존의 의미 분할(semantic segmentation) 모델은 학습 때 정해 둔 클래스 목록 안에서만 답을 내놓았다. 예컨대 "의자, 테이블, 사람"만 학습한 모델은 눈앞의 "컵"을 따로 분할해 내지 못한다. SAM은 이 제약을 **프롬프트 기반 분할(Promptable Segmentation)**이라는 개념으로 풀었다. 점이나 사각형 박스, 혹은 대략적인 마스크로 "바로 이 위치의 물체"를 가리키면, 모델이 그 자리에 있는 물체의 경계를 오려 내 준다. 참고로 텍스트 프롬프트는 원 논문에서 탐색적 실험으로만 다루었을 뿐이며, 공개된 체크포인트와 공식 API가 실제로 받는 입력은 점·박스·마스크 세 가지다. 따라서 텍스트로 물체를 지정해 분할하려면 뒤에서 볼 GroundingDINO로 먼저 텍스트를 바운딩 박스로 바꾼 뒤 SAM에 넘겨주는 방식을 쓴다.

SAM이 물체의 이름을 몰라도 경계를 정확히 가려낼 수 있는 이유는 클래스 라벨을 떼어 버린 11억 개(1.1B)의 대규모 마스크 데이터로 학습했기 때문이다. 덕분에 로봇이 난생처음 보는 물체를 집어 올려야 할 때도 별도 학습 없이 곧바로 물체 마스크를 얻을 수 있다. 다만 의료 영상, 위성 사진, 수중 카메라처럼 학습 데이터와 시각적 분포가 크게 다른 도메인에서는 분할 정확도가 떨어질 수 있다(§11.5).

SAM은 상호작용적인 프롬프트 처리 속도를 확보하기 위해 세 부분으로 나뉘어 동작한다.

- **Image Encoder**: 무거운 ViT가 이미지를 한 번 인코딩해 특징 맵을 만들어 둔다.
- **Prompt Encoder**: 사용자가 넘긴 점, 박스, 마스크 프롬프트를 가벼운 임베딩 벡터로 바꾼다.
- **Mask Decoder**: 이미지 특징과 프롬프트 임베딩을 결합하는 가벼운 디코더가 최종 분할 마스크를 즉시 생성한다.

이후 등장한 **SAM2**는 정지 이미지와 비디오를 하나의 아키텍처로 통합 처리하며, 원 논문 기준 이미지 분할 속도도 기존 SAM보다 6배 빨라졌다. 첫 프레임에서 점이나 박스로 물체를 한 번만 지정해 주면 비디오가 이어지는 동안 그 물체를 끝까지 추적하며 분할하므로, 움직이는 물체를 실시간으로 따라가며 조작하는 로봇 작업에 특히 유용하다.

이제 SAM 모델에 이미지와 전경 포인트 좌표를 하나 넘겨 마스크를 얻는 기본 흐름을 코드로 살펴보자.

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

점 하나만 찍으면 "모자만 가리킨 건지, 사람 전체를 가리킨 건지" 모호할 때가 있다. 이때 `multimask_output=True` 옵션을 켜 두면 모델이 전체 물체, 부분, 더 작은 부분에 해당하는 마스크 후보 3개를 함께 반환하고, `scores`에는 각 마스크의 품질(예상 IoU) 추정치가 담긴다. 실제 파이프라인에서는 프롬프트를 추가해 모호성을 줄이거나 세 후보의 점수와 크기를 비교해 원하는 범위의 마스크를 선택한다.

> **추천 자료**
> - [Kirillov et al., "Segment Anything" (2023)](https://arxiv.org/abs/2304.02643) — SAM 원논문
> - [Ravi et al., "SAM 2: Segment Anything in Images and Videos" (2024)](https://arxiv.org/abs/2408.00714) — SAM2 원논문. 비디오 segmentation까지 다룬다
> - [Segment Anything GitHub](https://github.com/facebookresearch/segment-anything) — 공식 코드
> - [Segment Anything Explained](https://www.youtube.com/watch?v=KRAJd4_rNrc) — SAM의 구조를 설명하는 영상
> - [HuggingFace — SAM](https://huggingface.co/docs/transformers/model_doc/sam) — HuggingFace 사용법

> **실습**: [SAM2 Interactive Segmentation](https://alexjunholee.github.io/robotics-practice/app.html#hf_sam)
> SAM2 모델에 프롬프트를 직접 넣어 이미지를 분할해 본다 (HuggingFace Space).

### 11.2.3 CLIP

DINOv2와 SAM이 시각 정보 자체의 구조와 경계를 잘 잡는다면, 로봇에게 "저기 나무 테이블 위 빨간 머그컵을 찾아봐"처럼 말로 지시를 내리려면 이미지와 언어가 서로 통하는 다리가 필요하다. **CLIP(Contrastive Language-Image Pre-training)**은 이미지와 텍스트를 하나의 공유 임베딩 공간(Vision-Language embedding space)에 나란히 배치해 이 문제를 해결한다.

CLIP 이전의 이미지 분류기는 학습 때 정해 둔 제한된 클래스 목록 안에서만 답을 골랐다. 반면 CLIP은 이미지-텍스트 쌍 4억 개를 대조 학습(contrastive learning)으로 익혀, 짝이 맞는 이미지와 문장은 가까운 벡터로, 관련 없는 쌍은 먼 벡터로 밀어낸다. 그 덕분에 미리 정한 클래스 목록 없이도 `"red mug on a wooden table"` 같은 자유로운 자연어 문장으로 이미지를 검색하거나 분류할 수 있다. 로봇 공학에서는 다음과 같은 곳에 폭넓게 쓰인다.

- 추가 학습 없는 Zero-shot 이미지 분류
- 자연어 기반 이미지-텍스트 검색(retrieval)
- Open-vocabulary 물체 검출 모델의 텍스트·이미지 인코더

CLIP이 실제로 이미지와 여러 텍스트 후보 사이의 유사도를 어떻게 계산하는지 아래 코드로 확인해 보자.

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

코드에서 `@` 연산자는 행렬 곱을 뜻한다. 이미지 특징과 텍스트 특징을 각각 L2 정규화해 길이를 1로 맞춘 뒤 내적하면 두 벡터 사이의 코사인 유사도가 얻어진다. 여기에 모델이 학습한 온도 스케일(위 코드의 `100.0`)을 곱하고 문장 후보 차원에 softmax를 적용하면, 입력 이미지가 각 설명과 얼마나 잘 어울리는지를 나타내는 zero-shot 분류 확률이 나온다. 만약 정규화 단계를 빼먹으면 방향 유사도 대신 각 벡터의 크기(norm)가 점수에 뒤섞여 결과가 왜곡되므로 주의해야 한다.

> **추천 자료**
> - [Radford et al., "Learning Transferable Visual Models From Natural Language Supervision" (2021)](https://arxiv.org/abs/2103.00020) — CLIP 원논문
> - [OpenAI CLIP GitHub](https://github.com/openai/CLIP) — 공식 코드와 사전학습 모델
> - [Yannic Kilcher — CLIP 설명](https://www.youtube.com/watch?v=T9XSU0pKX2E) — CLIP 논문을 풀어 설명하는 영상
> - [HuggingFace — CLIP](https://huggingface.co/docs/transformers/model_doc/clip) — 여러 CLIP 변형의 HuggingFace 사용법

### 11.2.4 Depth Anything

로봇이 물체를 보고 이름까지 알아냈더라도, 카메라 한 장만으로는 그 물체가 손이 닿을 50 cm 거리에 있는지 5 m 밖에 있는지 곧바로 알기 어렵다. **Depth Anything**은 일반 RGB 이미지 한 장에서 장면의 상대적인 원근(단안 깊이, monocular depth)을 추정하는 파운데이션 모델이다.

이 모델은 깊이 정답 라벨이 있는 이미지 150만 장(1.5M)에 더해 라벨이 없는 이미지 6,200만 장(62M)을 함께 활용해 방대한 시각적 단서를 학습했다. 원 논문은 실내(NYU)와 실외(KITTI)를 비롯해 학습에 전혀 쓰지 않은 다양한 데이터셋에서 뛰어난 zero-shot 깊이 추정 성능을 입증했다. 물론 내시경이나 수중 영상처럼 학습 분포와 이질적인 특수 도메인에서는 정확도가 떨어질 수 있다.

버전이 올라가면서 실용성도 한층 다듬어졌다. V1은 NYUv2와 KITTI 데이터셋으로, V2는 정밀한 합성 데이터셋인 Hypersim(실내)과 Virtual KITTI 2(실외)로 파인튜닝한 **metric depth(절대 깊이)** 모델을 별도로 공개했다. 특히 V2는 실제 센서 노이즈가 없는 합성 이미지로 teacher 모델을 다시 학습시켜, 얇은 구조물이나 경계 부근의 상대 깊이 세부 묘사를 크게 끌어올렸다.

로봇 시스템에 적용할 때는 한 가지 구분이 꼭 필요하다. 일반 상대 깊이 모델은 어느 물체가 더 앞뒤에 있는지는 잘 가려내지만 미터(m) 단위의 실제 스케일은 알지 못한다. 따라서 충돌 회피나 로봇 팔 파지처럼 물리적 거리 값이 직접 필요한 제어 루프에는 반드시 metric depth 모델을 사용하거나 별도의 스케일 보정 과정을 거쳐야 한다.

> **추천 자료**
> - [Yang et al., "Depth Anything: Unleashing the Power of Large-Scale Unlabeled Data" (2024)](https://arxiv.org/abs/2401.10891) — Depth Anything 원논문
> - [Yang et al., "Depth Anything V2" (2024)](https://arxiv.org/abs/2406.09414) — V2 원논문
> - [Depth Anything GitHub](https://github.com/LiheYoung/Depth-Anything) — 공식 코드
> - [HuggingFace — Depth Anything](https://huggingface.co/docs/transformers/model_doc/depth_anything) — HuggingFace 사용법

> **실습**: [Depth Anything V2](https://alexjunholee.github.io/robotics-practice/app.html#hf_depth)
> Depth Anything V2로 이미지의 깊이를 직접 추정해 본다 (HuggingFace Space).

### 11.2.5 GroundingDINO

CLIP은 이미지 전체와 문장을 연결해 주지만 물체가 화면 어디에 있는지 정확한 박스를 쳐 주지는 않고, SAM은 물체를 정교하게 오려 내 주지만 먼저 위치를 가리킬 점이나 박스가 필요하다. 이 빈자리를 채우는 모델이 바로 Open-Vocabulary Object Detection 계열의 **GroundingDINO**다.

YOLO나 Faster R-CNN 같은 전통적인 closed-set detector는 학습 당시 정해 둔 클래스 집합 안에서만 물체를 찾는다. 반면 GroundingDINO는 이미지와 함께 자유로운 텍스트 질의를 입력받아, 해당 문구가 가리키는 이미지 영역의 바운딩 박스를 직접 돌려준다. `"red cup"`처럼 수식어가 붙은 표현을 그대로 넣어도 인식하며, 새로운 물체 클래스를 추가할 때마다 출력 헤드를 뜯어고쳐 다시 학습할 필요가 없다.

```
입력: 이미지 + "person. car. traffic light."
출력: 그 객체들의 bounding box
```

이 특성 덕분에 로보틱스 현장에서는 GroundingDINO와 SAM을 하나로 이은 **Grounded-SAM** 파이프라인이 자주 쓰인다. 먼저 GroundingDINO가 `"red cup"`이라는 텍스트 질의에 해당하는 바운딩 박스를 찾아내면, SAM이 그 박스를 프롬프트로 받아 물체의 정밀한 픽셀 마스크를 생성한다. 이렇게 얻은 마스크를 깊이 영상과 결합하면 로봇 조작(manipulation) 파이프라인이 바로 집어 올릴 수 있는 3D 목표 영역이 완성된다.

> **추천 자료**
> - [Liu et al., "Grounding DINO: Marrying DINO with Grounded Pre-Training for Open-Set Object Detection" (2023)](https://arxiv.org/abs/2303.05499) — GroundingDINO 원논문
> - [Grounded-SAM GitHub](https://github.com/IDEA-Research/Grounded-Segment-Anything) — 텍스트 기반 탐지+분할 파이프라인
> - [HuggingFace — Grounding DINO](https://huggingface.co/docs/transformers/model_doc/grounding-dino) — HuggingFace 사용법

> **실습**: [Grounding DINO Demo](https://alexjunholee.github.io/robotics-practice/app.html#hf_grounding_dino)
> 텍스트 프롬프트를 넣어 이미지에서 물체를 탐지해 본다 (HuggingFace Space).

---

## 11.3 VFM의 Spatial AI 응용

앞서 살펴본 다섯 가지 모델은 각자 강점이 뚜렷하다. 실제 로봇의 공간 지능(Spatial AI) 시스템에서는 이들을 단독으로 쓰기보다 작업 목적에 맞게 유기적으로 엮어 쓴다. 대표적인 네 가지 결합 패턴을 살펴보자.

먼저 **Open-vocabulary Scene Understanding**이다. 가정이나 사무실처럼 예측하기 어려운 생활 공간에서 로봇은 미리 정의해 둔 목록에 없는 온갖 물체를 마주친다. 이때 CLIP, GroundingDINO, SAM을 조합하면 고정된 클래스 목록 없이도 장면 속 물체를 자유롭게 인식하고, `"navigate to the red chair"` 같은 자연어 명령을 받아 목표 위치로 이동하는 파이프라인을 꾸릴 수 있다.

비슷한 원리로 **Zero-shot Semantic Segmentation**도 구현할 수 있다. 새로운 작업 현장에 들어갈 때마다 픽셀 단위 라벨을 새로 달지 않고, SAM으로 장면 속 모든 물체의 경계 마스크를 먼저 뽑은 뒤 각 마스크 영역을 잘라 CLIP으로 클래스 이름이나 설명을 붙이는 식이다.

한편 기하학적 추적 안정성을 높이는 데도 VFM이 쓰인다. **Dense Feature for SLAM**이 대표적이다. 고전 SLAM은 ORB나 SIFT 같은 지역 특징점에 의존하는데, 무늬가 거의 없는 흰 벽면이나 단조로운 바닥에서는 특징점이 턱없이 부족해 추적이 끊기기 쉽다. 반면 DINOv2의 dense feature에는 픽셀 주변의 넓은 시각적 맥락과 의미 정보가 함께 녹아 있어, 텍스처가 희박한 영역에서도 위치마다 구별되는 특징 값을 내놓는다. 이를 특징점 매칭에 활용하면 어려운 장면에서도 SLAM의 추적 강건성을 높일 수 있다.

마지막으로 **3D Scene Understanding**은 2D 이미지에서 뽑아낸 VFM의 의미 정보를 3차원 공간으로 끌어올린다. 여러 시점의 이미지에서 추출한 CLIP이나 DINOv2 특징을 NeRF, 3D Gaussian Splatting(3DGS), 혹은 3D Scene Graph 같은 3차원 지도 표현에 투영해 함께 심어 두는 방식이다. 이렇게 의미가 담긴 3D 지도를 만들어 두면 로봇은 `"이 공간에서 의자가 어디에 있지?"`라는 텍스트 질의만으로도 3차원 좌표를 곧바로 찾아낼 수 있다. LERF, LangSplat, ConceptGraphs가 바로 이 아이디어로 구현된 대표 연구들이다.

> **추천 자료**
> - [Kerr et al., "LERF: Language Embedded Radiance Fields" (2023)](https://arxiv.org/abs/2303.09553) — CLIP feature를 NeRF에 함께 학습시켜 텍스트로 3D 위치를 찾는다
> - [Tschernezki et al., "Neural Feature Fusion Fields: 3D Distillation of Self-Supervised 2D Image Representations" (2022)](https://arxiv.org/abs/2209.03494) — 2D 자기지도 feature를 3D 표현으로 distillation한 연구
> - [Papers With Code — 3D Scene Understanding](https://paperswithcode.com/task/3d-scene-understanding) — Papers With Code는 2025년에 문을 닫았다(19장 §19.4). 이 주소는 지금 Hugging Face의 trending 논문 목록으로 넘어간다

---

## 11.4 경량화 및 Edge 배포

VFM이 아무리 다재다능하더라도 로봇에 올라간 제한된 온보드 컴퓨터에서 제때 돌아가지 못하면 현장에서 쓸 수 없다. VFM은 보통 수억 개의 파라미터를 품고 있어 연산량과 메모리 점유율이 매우 높다. ViT-B급 모델만 해도 데스크톱 워크스테이션 GPU에서는 가볍게 돌아가지만, 현장 로봇은 한정된 전력과 메모리 예산 안에서 제어·경로 계획·센서 처리 등 다른 모듈과 자원을 나눠 써야 한다. 초당 몇 프레임을 처리해야 하는지는 제어 루프와 작업 성격에 따라 갈리며, 허용 지연 시간 역시 물체 검색, 지도 갱신, 고속 조작마다 다르다. 이 실시간 제약을 맞추기 위해 모델 경량화와 Edge 배포 최적화 기법을 사용한다.

대표적인 모델 경량화 접근법은 크게 세 가지로 나뉜다.

| 기법 | 설명 |
|------|------|
| Distillation | 큰 모델의 출력을 작은 모델이 따라 하도록 학습 |
| Quantization | 수치 정밀도를 FP32에서 INT8/INT4로 낮춤 |
| Pruning | 기여가 작은 weight를 지움 |

세 기법은 적용 난이도와 효과가 다르다. **Quantization(양자화)**은 기존 네트워크 구조를 건드리지 않고 가중치와 활성화 값의 수치 정밀도만 낮추므로 가장 손쉽게 적용할 수 있다. **Pruning(가지치기)**은 어떤 가중치를 덜어낼지 기준을 정해야 하며, 개별 가중치를 듬성듬성 지우기보다 채널이나 블록 단위로 통째로 잘라 내는 구조적(structured) pruning이어야 실제 하드웨어 연산량이 줄어든다. **Distillation(지식 증류)**은 작고 빠른 student 모델 구조를 완전히 새로 설계할 수 있어 압축 폭이 가장 크지만, student 모델을 따로 학습시켜야 하는 비용이 따른다.

실제 로보틱스에서는 이런 기법을 적용해 미리 가볍게 만들어 공개한 **경량 VFM**을 가져다 쓰는 일도 잦다. SAM 계열에서는 YOLOv8-seg 구조로 SAM의 분할 역할을 대신해 속도를 크게 높인 **FastSAM**, SAM의 무거운 image encoder를 작은 ViT로 교체해 증류한 **MobileSAM**, 그리고 효율적인 EfficientViT 백본으로 인코더를 바꾼 **EfficientViT-SAM** 등이 널리 쓰인다.

모델을 가볍게 줄였다면 로봇의 임베디드 보드에서 최대 속도를 끌어내는 추론 엔진에 올릴 차례다. 주요 Edge 배포 도구로는 NVIDIA GPU에 맞춰 그래프 융합과 정밀도 최적화를 수행하는 **TensorRT**, 다양한 하드웨어와 운영체제에서 ONNX 포맷 모델을 일관되게 실행하는 **ONNX Runtime**, 그리고 모바일과 임베디드 기기에 특화된 **TFLite**(2024년 9월 **LiteRT**로 개명)가 있다. 아래는 PyTorch 모델을 ONNX로 내보낸 뒤 TensorRT 엔진으로 컴파일하는 전형적인 변환 절차다.

```python
# TensorRT 변환 예시 (PyTorch → ONNX → TensorRT)
import torch

# 1. ONNX 내보내기
torch.onnx.export(model, dummy_input, "model.onnx")

# 2. TensorRT 변환 (trtexec 사용)
# trtexec --onnx=model.onnx --saveEngine=model.trt --fp16
```

NVIDIA Jetson 플랫폼에서는 이처럼 TensorRT를 추론 백엔드로 삼는 경우가 표준적이다. 다만 FP16이나 INT8로 정밀도를 낮췄을 때 지연 시간, 메모리 사용량, 태스크 정확도가 실제로 얼마나 변하는지는 모델 연산 그래프와 입력 해상도, 배치 크기, Jetson 전력 모드(power mode), 그리고 TensorRT와 CUDA 버전에 따라 크게 달라진다. 따라서 스펙 표의 숫자만 믿지 말고 실제 배포할 장비 위에서 FP32 기준 모델과 동일한 검증 데이터셋(validation set)으로 성능을 직접 재야 한다. 특히 INT8 양자화를 쓸 때는 실제 현장 입력 분포를 잘 대표하는 calibration 데이터를 준비해 활성화 스케일을 보정한 뒤 정확도 하락 여부를 다시 확인해야 안전하다.

> **추천 자료**
> - [NVIDIA TensorRT 문서](https://docs.nvidia.com/deeplearning/tensorrt/) — TensorRT 사용법과 최적화 가이드
> - [ONNX Runtime](https://onnxruntime.ai/) — 여러 플랫폼에서 쓰는 추론 엔진
> - [MobileSAM GitHub](https://github.com/ChaoningZhang/MobileSAM) — 모바일용으로 줄인 SAM
> - [FastSAM GitHub](https://github.com/CASIA-LMC-Lab/FastSAM) — YOLO 기반으로 SAM을 대신하는 모델
> - [NVIDIA Jetson Tutorials](https://developer.nvidia.com/embedded/learn/tutorials) — Jetson 튜토리얼 모음

---

## 11.5 VFM Fine-tuning과 Adaptation [심화]

경량화까지 마쳐 로봇에 올릴 준비를 끝냈더라도, 막상 의료 영상이나 위성 사진, 탁한 수중 환경처럼 인터넷 자연 이미지와 분포가 크게 다른 현장에 투입하면 VFM의 zero-shot 성능이 기대에 못 미칠 수 있다. 그렇다고 수억 개의 파라미터를 통째로 다시 학습(full fine-tuning)하자니 GPU 메모리와 데이터 비용이 너무 크다. 이 딜레마를 푸는 방법이 전체 파라미터 중 극히 일부만 업데이트하면서도 full fine-tuning에 버금가는 성능을 끌어내는 **Parameter-Efficient Fine-Tuning(PEFT)**이다.

다양한 fine-tuning 전략을 학습 비용과 성능 관점에서 비교하면 다음과 같다.

| 전략 | 학습 파라미터 비율 | 성능 | GPU 메모리 | 적용 난이도 |
|------|-------------------|------|-----------|------------|
| Full fine-tuning | 100% | 최고 (데이터 충분 시) | 매우 높음 | 낮음 |
| Linear probing | <1% (head만) | 낮음 | 낮음 | 매우 낮음 |
| LoRA | 0.1~1% | 높음 | 낮음 | 보통 |
| Adapter | 1~5% | 높음 | 보통 | 보통 |
| Prompt tuning | <0.1% | 보통 | 낮음 | 높음 |

이 가운데 가장 널리 쓰이는 기법이 **LoRA(Low-Rank Adaptation)**다. LoRA는 거대한 사전학습 가중치 행렬 $\mathbf{W}$를 그대로 동결(freeze)해 둔 채, 파인튜닝 과정에서 바뀌어야 할 가중치 변화량 전체를 직접 구하지 않고 두 개의 얇은 low-rank 행렬을 곱한 형태로 근사해 학습한다.

$$\mathbf{W}' = \mathbf{W} + \Delta\mathbf{W} = \mathbf{W} + \mathbf{B}\mathbf{A}$$

여기서 $\mathbf{W} \in \mathbb{R}^{d \times d}$일 때 $\mathbf{B} \in \mathbb{R}^{d \times r}$과 $\mathbf{A} \in \mathbb{R}^{r \times d}$로 두며, 랭크는 차원보다 훨씬 작게 잡는다($r \ll d$). 이렇게 분해하면 원래 $\mathbf{W}$의 파라미터 수 $d^2$개 대신 $2dr$개만 학습하면 된다. 예를 들어 $d = 768$, $r = 8$이면 원래 589,824개였던 학습 파라미터가 12,288개로 줄어 약 2% 수준에 불과하다.

HuggingFace의 `peft` 라이브러리를 사용하면 DINOv2 같은 ViT 모델의 attention 층에 LoRA를 몇 줄만으로 붙일 수 있다.

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

LoRA와 함께 자주 비교되는 방식이 **Adapter**다. Adapter는 Transformer 블록 내부에 입력 차원을 잠깐 줄였다가 다시 키우는 작은 병목 층(bottleneck layer)을 끼워 넣고, 원래 가중치는 고정한 채 이 adapter 층만 학습한다.

```
Input → [Frozen Attention] → [Adapter: down_proj → ReLU → up_proj] → [Frozen FFN] → Output
```

두 기법은 추론 속도에서 뚜렷한 차이를 보인다. LoRA는 학습이 끝난 뒤 $\mathbf{B}\mathbf{A}$를 원래 가중치 $\mathbf{W}$에 그대로 더해 병합(merge)해 버릴 수 있으므로 추론 시 추가 연산이 전혀 없다. 반면 Adapter는 네트워크 경로에 새로운 층이 직렬로 덧붙어 추론 지연이 조금 늘어난다.

더 극단적으로 학습 파라미터를 줄이고 싶다면 **Prompt Tuning**을 쓴다. 모델의 가중치는 손대지 않고 입력 단계에 학습 가능한 가상 토큰(learnable prompt token)을 덧붙여 그 토큰만 최적화하는 방식이다. 비전 분야의 대표적 방법인 **Visual Prompt Tuning(VPT)**은 첫 번째 layer 입력에만 토큰을 붙이는 VPT-Shallow와 ViT의 모든 layer 입력마다 토큰을 끼워 넣는 VPT-Deep으로 나뉘며, 앞선 표의 기법들 가운데 학습하는 파라미터 수가 가장 적다.

SAM처럼 구조가 특수한 모델은 **도메인 적응(adaptation)** 방식도 조금 다르다. SAM을 새로운 도메인에 맞출 때는 프롬프트를 생성하는 방식을 자동화하거나, 핵심 특징을 추출하는 image encoder를 파인튜닝한다.

1. **Grid prompt**: 이미지 위에 N×N 격자를 깔고 각 격자점마다 point prompt를 넣어 전체 물체를 빠짐없이 분할한다.
2. **학습한 prompt generator**: 이미지를 입력받아 유효한 point·box prompt를 자동으로 예측하는 작은 보조 네트워크를 학습한다.
3. **LoRA + SAM**: 무거운 image encoder에 LoRA를 붙여 새 도메인에 맞는 시각 특징을 학습하고, 파라미터가 적은 mask decoder는 직접 업데이트한다.

아래 코드는 SAM의 image encoder에 LoRA를 적용하고 가벼운 mask decoder는 전체 학습하도록 설정하는 개념적 예시다.

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

이렇게 다양한 적응 기법 가운데 내 로봇 환경에 가장 알맞은 방법을 고르려면 공정한 **평가 프로토콜** 아래 성능을 비교해야 한다.

| 프로토콜 | 설명 | 비교 목적 |
|---------|------|----------|
| Zero-shot | 학습 없이 바로 평가 | 추가 학습 없이 얼마나 되는지 확인 |
| Few-shot (1/5/10-shot) | 클래스당 샘플 몇 개로 학습 | 적은 데이터로 얼마나 되는지 비교 |
| Full fine-tune | 학습 데이터 전체로 모든 파라미터 학습 | 성능 상한 확인 |
| PEFT (LoRA 등) | 파라미터 일부만 학습 | 학습 비용과 성능의 trade-off |

기법을 공정하게 비교하려면 backbone 모델, 데이터 split, augmentation 조건을 모두 동일하게 고정해야 한다. 특히 데이터가 극소량인 few-shot 평가에서는 어떤 샘플 몇 개가 뽑혔는지(random seed)에 따라 점수가 크게 출렁이므로, 최소 3~5번 반복 실험해 평균과 표준편차를 함께 보고하는 습관이 중요하다.

지금까지 살펴본 VFM과 경량화·적응 기법은 로봇이 낯선 세상을 정교하게 *보고 이해하는* 눈을 만들어 준다. 하지만 로봇이 실제로 임무를 완수하려면 본 것을 바탕으로 팔을 뻗고 바퀴를 굴려야 한다. 다음 12장에서는 시각과 언어 이해를 곧바로 로봇의 물리적 행동으로 연결하는 VLA(Vision-Language-Action) 모델과 체화 AI를 다룬다.

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
