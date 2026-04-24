# Ch.11 — Vision Foundation Models (VFM)


The paradigm of computer vision is shifting. The deep learning models covered so far were specialists trained to do a specific task well on a specific dataset; foundation models are generalists that learn general-purpose visual capabilities from massive data. Once this difference is clear, so is the reason VFM-based perception papers have grown rapidly at ICRA/IROS since 2023.

---

## 11.1 What Is a Foundation Model?

A **foundation model** is a model pretrained on large-scale data and applicable to a wide range of downstream tasks.

Consider the conventional approach. It required repeating the cycle "new environment → data collection → labeling → training". When the factory changed, or when the objects the robot had to recognize changed, everything had to be redone from scratch. Foundation models break this cycle. A model trained once works on objects it has never seen, in environments it has never seen. It is the most promising approach for solving the generalization problem in robotics.

**Characteristics**:
- **Scale**: hundreds of millions to billions of parameters
- **Pretraining**: large-scale data (hundreds of millions of images)
- **Zero-shot / Few-shot**: performs new tasks with no training or only a few examples
- **Transfer**: transfers to diverse domains

**Why it matters**:
- generalization to new environments
- usable without annotations for a specific dataset
- a core role in the lab's Global Module

Here the concept of the scaling law is important. It is the empirical law that performance improves as a power law when model size, data size, and compute are scaled up. Recent large models such as GPT, CLIP, and SAM all exploit this law. "Bigger is better" holds within a certain range (Kaplan et al., 2020; Zhai et al., 2022 for ViT). However, as Hoffmann et al. (2022, Chinchilla) showed, balancing data and compute matters more than simply scaling model size.

> **Further reading**
> - [Bommasani et al., "On the Opportunities and Risks of Foundation Models" (2021)](https://arxiv.org/abs/2108.07258) — Stanford report that defined the term "foundation model".
> - [Two Minute Papers — videos on foundation models](https://www.youtube.com/@TwoMinutePapers) — a quick way to keep up with the latest VFM research.
> - [HuggingFace Model Hub](https://huggingface.co/models) — thousands of pretrained models ready to use.

---

## 11.2 Major VFMs

We look at the vision foundation models most commonly used in robotics. The focus is on what problem each model solves, why it matters in robotics, and how to use it.

### 11.2.1 DINOv2

A **self-supervised Vision Transformer** that learns rich features from images without labels.

DINOv2 learns general-purpose visual features without labels. These features can be used as-is for diverse tasks such as classification, segmentation, and matching. In robotics in particular, DINOv2's dense features provide stable matching even in textureless regions, and are used in SLAM and visual odometry to reduce tracking failure rates in textureless environments.

**Characteristics**:
- contrastive learning + self-distillation
- strong transfer performance across diverse tasks
- provides dense visual features

**Uses**:
- image retrieval
- semantic segmentation (linear probe)
- feature matching for SLAM/VO
- feature backbone for 3D reconstruction

```python
import torch
from transformers import AutoModel, AutoImageProcessor

processor = AutoImageProcessor.from_pretrained('facebook/dinov2-base')
model = AutoModel.from_pretrained('facebook/dinov2-base')

inputs = processor(images=image, return_tensors="pt")
outputs = model(**inputs)
features = outputs.last_hidden_state  # (1, num_patches+1, 768): CLS + patch features
```

The first token of `last_hidden_state` is the [CLS] token summarizing the whole image, and the rest are per-patch features. The [CLS] token is used for classification, while the patch features are used for dense prediction (segmentation, matching, etc.).

> **Further reading**
> - [Oquab et al., "DINOv2: Learning Robust Visual Features without Supervision" (2023)](https://arxiv.org/abs/2304.07193) — the original DINOv2 paper.
> - [DINOv2 GitHub](https://github.com/facebookresearch/dinov2) — official code and pretrained models.
> - [HuggingFace — DINOv2](https://huggingface.co/docs/transformers/model_doc/dinov2) — ready to use on HuggingFace.
> - [Yannic Kilcher — DINO explained](https://www.youtube.com/watch?v=h3ij3F3cPIk) — explains the core idea of self-distillation (based on DINOv1, but essential for understanding DINOv2).

### 11.2.2 SAM (Segment Anything Model)

**Promptable segmentation**: segments any object from prompts such as points, boxes, or text.

Conventional segmentation models could only segment the classes used during training. Trained on "chair, table, person", they cannot segment "cup". SAM is trained on 1.1B masks and can segment any object it has never seen. For robots that have to manipulate objects they encounter for the first time in a new environment, the approach to segmentation has changed since SAM.

**Components**:
- Image Encoder: embeds images with a ViT
- Prompt Encoder: points, boxes, masks, etc.
- Mask Decoder: a lightweight decoder that produces masks

**SAM2**: video support, higher speed

SAM2 works not only on single images but also on videos. Specify an object with a point or box on the first frame, and it is automatically tracked and segmented in subsequent frames. This applies directly to scenarios in which a robot tracks and manipulates objects in real time.

```python
from segment_anything import sam_model_registry, SamPredictor

sam = sam_model_registry["vit_h"](checkpoint="sam_vit_h.pth")
predictor = SamPredictor(sam)

predictor.set_image(image)
masks, scores, logits = predictor.predict(
    point_coords=np.array([[500, 375]]),
    point_labels=np.array([1]),  # 1: foreground
    multimask_output=True,
)
```

With `multimask_output=True`, three mask candidates are returned (whole object, part, smaller part). Use `scores` to pick the most suitable mask.

> **Further reading**
> - [Kirillov et al., "Segment Anything" (2023)](https://arxiv.org/abs/2304.02643) — the original SAM paper.
> - [Ravi et al., "SAM 2: Segment Anything in Images and Videos" (2024)](https://arxiv.org/abs/2408.00714) — the original SAM2 paper. Extension to video segmentation.
> - [Segment Anything GitHub](https://github.com/facebookresearch/segment-anything) — official code.
> - [Segment Anything Explained](https://www.youtube.com/watch?v=KRAJd4_rNrc) — understand SAM's architecture and impact.
> - [HuggingFace — SAM](https://huggingface.co/docs/transformers/model_doc/sam) — ready to use on HuggingFace.

> **Exercise**: [SAM2 Interactive Segmentation](https://alexjunholee.github.io/robotics-practice/app.html#hf_sam)
> Try prompt-based segmentation on images using the SAM2 model directly (HuggingFace Space).

### 11.2.3 CLIP

**Vision-language model**: maps images and text into a shared embedding space.

Before CLIP, classifying an image required a predefined list of classes. CLIP places images and text in the same space, so arbitrary text can be used to retrieve or classify images. It becomes possible to tell a robot its target object in natural language, such as "red mug on a wooden table". This is the start of open-vocabulary, and the foundation for robots understanding natural language.

**Training**: contrastive learning on 400M image-text pairs

**Uses**:
- zero-shot image classification
- image-text retrieval
- basis for open-vocabulary detection

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
    print(similarity)  # similarity between each text and the image
```

`@` is matrix multiplication (dot product). It computes the cosine similarity between image features and text features, expressing numerically how semantically close the image is to each text. This is the principle of zero-shot classification.

> **Further reading**
> - [Radford et al., "Learning Transferable Visual Models From Natural Language Supervision" (2021)](https://arxiv.org/abs/2103.00020) — the original CLIP paper.
> - [OpenAI CLIP GitHub](https://github.com/openai/CLIP) — official code and pretrained models.
> - [Yannic Kilcher — CLIP explained](https://www.youtube.com/watch?v=T9XSU0pKX2E) — a clear walk-through of the CLIP idea.
> - [HuggingFace — CLIP](https://huggingface.co/docs/transformers/model_doc/clip) — use various CLIP variants on HuggingFace.

### 11.2.4 Depth Anything

**Monocular depth foundation model**: estimates relative depth from a single image.

Section 10.7 covered depth estimation; Depth Anything raises it to the level of a foundation model. By leveraging 1.5M labeled images plus 62M unlabeled images, it estimates depth stably indoors (NYU), outdoors (KITTI), and in zero-shot domains. Accuracy can drop, though, in domains that differ greatly from the training data (endoscopy, underwater, etc.). Its advantage is that a robot deployed to a new environment can obtain depth information immediately without additional training.

**Characteristics**:
- trained on 1.5M labeled + 62M unlabeled images
- robust across diverse domains
- V2: more accurate absolute depth

Depth Anything V2 goes a step beyond V1 and also provides a version that estimates metric depth (absolute depth). In robotics, "exactly how many meters away" often matters more than relative depth, so the metric version of V2 deserves attention.

> **Further reading**
> - [Yang et al., "Depth Anything: Unleashing the Power of Large-Scale Unlabeled Data" (2024)](https://arxiv.org/abs/2401.10891) — the original Depth Anything paper.
> - [Yang et al., "Depth Anything V2" (2024)](https://arxiv.org/abs/2406.09414) — the original V2 paper. Metric depth support.
> - [Depth Anything GitHub](https://github.com/LiheYoung/Depth-Anything) — official code.
> - [HuggingFace — Depth Anything](https://huggingface.co/docs/transformers/model_doc/depth_anything) — ready to use on HuggingFace.

> **Exercise**: [Depth Anything V2](https://alexjunholee.github.io/robotics-practice/app.html#hf_depth)
> Try the Depth Anything V2 model to estimate depth from images directly (HuggingFace Space).

### 11.2.5 GroundingDINO

**Open-vocabulary object detection**: detects arbitrary objects from text prompts.

Conventional detection models (YOLO, Faster R-CNN, etc.) could only detect the classes used during training. Trained on "person, car, dog", they cannot find "coffee mug". Like CLIP, GroundingDINO can specify and detect any object with text. Give a robot a natural-language instruction such as "find the red cup over there", and it can locate the cup immediately without training.

```
Input: image + "person. car. traffic light."
Output: bounding boxes for the corresponding objects
```

**Grounded-SAM**: GroundingDINO + SAM combined
→ text-prompted object detection + segmentation

Grounded-SAM is a practical combination for robotics. Feed in the text "red cup", and GroundingDINO finds the bounding box, then SAM produces a precise mask inside it. Because arbitrary objects can be detected and segmented without separate training, it is widely used in manipulation pipelines.

> **Further reading**
> - [Liu et al., "Grounding DINO: Marrying DINO with Grounded Pre-Training for Open-Set Object Detection" (2023)](https://arxiv.org/abs/2303.05499) — the original GroundingDINO paper.
> - [Grounded-SAM GitHub](https://github.com/IDEA-Research/Grounded-Segment-Anything) — text-based detection+segmentation pipeline.
> - [HuggingFace — Grounding DINO](https://huggingface.co/docs/transformers/model_doc/grounding-dino) — use on HuggingFace.

> **Exercise**: [Grounding DINO Demo](https://alexjunholee.github.io/robotics-practice/app.html#hf_grounding_dino)
> Try open-vocabulary detection that finds arbitrary objects in images from text prompts (HuggingFace Space).

---

## 11.3 Spatial AI Applications of VFMs

We look at how the VFMs covered above are combined in real robotics systems. The capability of each individual model matters, but the goal in robotics is to combine them to build an AI that understands space.

**Open-vocabulary scene understanding**:
- scene understanding without predefined classes
- handling natural-language commands such as "navigate to the red chair"

For robots to operate in real environments, they cannot rely on a predetermined list of objects. They must understand a person's natural-language command, find the corresponding object, and act accordingly. This pipeline can be implemented with a CLIP + SAM + GroundingDINO combination.

**Zero-shot semantic segmentation**:
- segmentation in new environments without labeling
- implemented with a CLIP + SAM combination

**Dense features for SLAM**:
- use DINOv2 features in place of keypoints
- matching is possible even in textureless regions
- recent work: DROID-SLAM + DINOv2

This is directly tied to problems encountered in the field. Classical SLAM relies on keypoints such as ORB and SIFT, but on textureless walls or floors, keypoints are hard to obtain. DINOv2's dense features carry semantic information, so even on a white wall one part can be distinguished from another. This is why SLAM robustness improves.

**3D scene understanding**:
- lift 2D VFM features into 3D
- Semantic NeRF, Feature 3DGS

Embedding 2D-extracted VFM features into a 3D representation (NeRF, 3D Gaussian Splatting) carries semantic information in the 3D space itself. A question like "where is the chair in this 3D map?" can be answered with a text query. Research in this direction is growing in Spatial AI (LERF, LangSplat, ConceptGraphs, etc.).

> **Further reading**
> - [Kerr et al., "LERF: Language Embedded Radiance Fields" (2023)](https://arxiv.org/abs/2303.09553) — work that embeds CLIP features into NeRF. A representative example of Spatial AI.
> - [Tschernezki et al., "Neural Feature Fusion Fields: 3D Distillation of Self-Supervised 2D Image Representations" (2022)](https://arxiv.org/abs/2209.03494) — early work on lifting 2D features into 3D.
> - [Papers With Code — 3D Scene Understanding](https://paperswithcode.com/task/3d-scene-understanding) — latest research trends.

---

## 11.4 Lightweight Models and Edge Deployment

Using VFMs in a robot's Local Module requires making them lightweight.

VFMs perform well, but with hundreds of millions of parameters they are hard to run in real time without a GPU server. Robots, on the other hand, must run at 30 FPS on Jetson or embedded boards. Bridging this gap is the job of lightweight modeling and edge deployment. No matter how good a model is, if it cannot run in real time on a robot, it only shines inside a paper.

**Lightweight techniques**:
| Technique | Description |
|------|------|
| **Distillation** | transfer knowledge from a large model to a small one |
| **Quantization** | reduce precision from FP32 → INT8/INT4 |
| **Pruning** | remove unimportant weights |

Understanding the trade-offs of each technique matters. Quantization does not change the model structure, so it is easiest to apply; pruning reduces actual compute but can incur accuracy loss. Distillation trains a small model from scratch, so its effect is largest but its cost is also highest.

**Lightweight VFMs**:
- **FastSAM**: lightweight version of SAM (YOLO-based)
- **MobileSAM**: SAM for mobile
- **EfficientViT-SAM**: efficient ViT backbone

**Edge deployment tools**:
- **TensorRT**: optimization for NVIDIA GPUs
- **ONNX Runtime**: cross-platform
- **TFLite**: mobile/embedded

```python
# TensorRT conversion example (PyTorch → ONNX → TensorRT)
import torch

# 1. Export to ONNX
torch.onnx.export(model, dummy_input, "model.onnx")

# 2. Convert to TensorRT (using trtexec)
# trtexec --onnx=model.onnx --saveEngine=model.trt --fp16
```

If you are using NVIDIA Jetson, TensorRT is almost mandatory. FP16 conversion alone yields a 2-3x speedup with almost no accuracy loss. Going to INT8 is faster still, but calibration data is required.

> **Further reading**
> - [NVIDIA TensorRT documentation](https://docs.nvidia.com/deeplearning/tensorrt/) — TensorRT usage and optimization guide.
> - [ONNX Runtime](https://onnxruntime.ai/) — cross-platform inference optimization.
> - [MobileSAM GitHub](https://github.com/ChaoningZhang/MobileSAM) — mobile-lightweight version of SAM.
> - [FastSAM GitHub](https://github.com/CASIA-IVA-Lab/FastSAM) — YOLO-based lightweight SAM.
> - [NVIDIA Jetson AI Courses](https://developer.nvidia.com/embedded/learn/jetson-ai-certification-programs) — edge deployment practice.

---

## 11.5 Advanced: VFM Fine-tuning and Adaptation

*If you want to become a researcher, start reading from here.*

Using a VFM as-is yields zero-shot performance, but performance drops in specific domains (medical, satellite, underwater, etc.). Fine-tuning is needed, but training all of its hundreds of millions of parameters is costly. Parameter-efficient fine-tuning (PEFT) trains only a tiny fraction of the model's parameters while achieving performance close to full fine-tuning.

**Comparison of fine-tuning strategies**:

| Strategy | Fraction of trained params | Performance | GPU memory | Application difficulty |
|------|-------------------|------|-----------|------------|
| **Full fine-tuning** | 100% | best (given enough data) | very high | low |
| **Linear probing** | <1% (head only) | low | low | very low |
| **LoRA** | 0.1~1% | high | low | moderate |
| **Adapter** | 1~5% | high | moderate | moderate |
| **Prompt tuning** | <0.1% | moderate | low | high |

**LoRA (Low-Rank Adaptation)**:

Core idea: add a low-rank update to a pretrained weight matrix $\mathbf{W}$.

$$\mathbf{W}' = \mathbf{W} + \Delta\mathbf{W} = \mathbf{W} + \mathbf{B}\mathbf{A}$$

Here $\mathbf{W}$ is a $d \times d$ matrix, $\mathbf{B}$ is $d \times r$, and $\mathbf{A}$ is $r \times d$ ($r \ll d$). Instead of the $d^2$ parameters of the original $\mathbf{W}$, only $2dr$ parameters are trained.

For example, with $d = 768$ and $r = 8$, instead of the original 589,824 parameters, only 12,288 are trained (about 2%).

```python
from peft import LoraConfig, get_peft_model
from transformers import AutoModelForImageClassification

# Load the base model
model = AutoModelForImageClassification.from_pretrained(
    "facebook/dinov2-base",
    num_labels=10
)

# LoRA configuration
lora_config = LoraConfig(
    r=16,                      # rank (low-rank matrix dimension)
    lora_alpha=32,             # scaling factor
    target_modules=["query", "value"],  # apply only to attention Q, V
    lora_dropout=0.1,
    bias="none",
)

# Create the PEFT model
model = get_peft_model(model, lora_config)
model.print_trainable_parameters()
# Example output: trainable params: 294,912 || all params: 86,567,178 || trainable%: 0.34%
```

**Adapter**:

Insert small bottleneck layers between Transformer blocks. The original weights are frozen, and only the adapter layers are trained.

```
Input → [Frozen Attention] → [Adapter: down_proj → ReLU → up_proj] → [Frozen FFN] → Output
```

LoRA merges into the existing weights, so there is no extra cost at inference; adapters are extra layers, so they add a small inference latency.

**Prompt tuning**:

Add learnable virtual tokens to the input. The model itself is left untouched; only the input is manipulated.

- Visual Prompt Tuning (VPT): adds learnable tokens to the input of each ViT layer.
- Parameter efficiency is the highest, but performance tends to be slightly below LoRA.

**Adapting SAM to specific domains**:

A common strategy is to attach a domain-specific automatic prompt generator to SAM's prompt encoder.

1. **Grid prompt**: split the image into an NxN grid and use each intersection as a point prompt.
2. **Learned prompt generator**: train a lightweight network that takes an image as input and automatically generates point/box prompts.
3. **LoRA + SAM**: apply LoRA to the image encoder to learn domain-specific features.

```python
# SAM + LoRA application example (conceptual)
from segment_anything import sam_model_registry
from peft import LoraConfig, get_peft_model

sam = sam_model_registry["vit_b"](checkpoint="sam_vit_b.pth")

# Apply LoRA only to the image encoder
lora_config = LoraConfig(
    r=4,
    lora_alpha=8,
    target_modules=["qkv"],  # SAM attention qkv projection
)

sam.image_encoder = get_peft_model(sam.image_encoder, lora_config)
# Full fine-tuning for the mask decoder (since it has few parameters)
for param in sam.mask_decoder.parameters():
    param.requires_grad = True
```

**Evaluation methodology**:

The standard protocol for comparison in VFM adaptation research is the following.

| Protocol | Description | Purpose of comparison |
|---------|------|----------|
| **Zero-shot** | evaluate without training | check the baseline generality of the VFM |
| **Few-shot (1/5/10-shot)** | train with a small number of samples per class | compare data efficiency |
| **Full fine-tune** | use the full training set | check the upper bound |
| **PEFT (LoRA, etc.)** | train with few parameters | efficiency-performance trade-off |

For fair comparison, the same backbone, the same data split, and the same augmentation must be used. In few-shot settings the variance across seeds is large, so results should be reported as the mean and standard deviation over 3-5 repetitions.

> **Further reading**
> - [Hu et al., "LoRA: Low-Rank Adaptation of Large Language Models" (2022)](https://arxiv.org/abs/2106.09685) — the original LoRA paper (for LLMs, but directly applicable to ViTs).
> - [HuggingFace PEFT library](https://github.com/huggingface/peft) — implementations of LoRA, Adapter, and other PEFT methods.
> - [Chen et al., "SAM Fails to Segment Anything? — SAM-Adapter" (2023)](https://arxiv.org/abs/2304.09148) — a case study of SAM domain adaptation.

---

> **Technical Timeline: Vision Foundation Models**
> - **2021**: CLIP (OpenAI) released. A shared image-text embedding opens up zero-shot recognition. Trained on 400M image-text pairs. The beginning of the open-vocabulary era.
> - **2022**: Self-supervised pretraining methods such as Masked Autoencoders (MAE) begin to draw attention. DINO demonstrates the potential of self-supervised ViTs.
> - **2023**: SAM (Segment Anything Model) released. Trained on 11M images and 1.1B masks. Achieves foundation-model-level generality with "segment anything". DINOv2 released the same year — a new standard for self-supervised vision features.
> - **2024**: Rapid evolution of VFMs including SAM2 (extension to video segmentation), Depth Anything V2 (metric depth support), and Florence-2 (unified vision model). Lightweight modeling and edge deployment become active.
> - **2025~**: 3D extensions of VFMs and multimodal unification accelerate. The direction is a single foundation model that jointly handles detection, segmentation, depth, and tracking. In robotics, VFMs are on course to become the standard perception backbone.
> - **What to watch now**: The core value of a foundation model is its **zero-shot ability**. Working in new environments and on new objects without additional training raises a robot's generalization. The CLIP+SAM+DINOv2 combination is used as a representative pipeline for open-vocabulary robot perception in NLMap, ConceptGraphs, and others. Making it lightweight (FastSAM, MobileSAM) and putting it on an actual robot completes the pipeline.
