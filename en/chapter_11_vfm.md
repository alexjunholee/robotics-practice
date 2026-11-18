# Ch.11 — Vision Foundation Models (VFM)


The perception networks in Ch.10 were built for a fixed world: a model trained on a specific dataset predicts a closed set of classes or masks, and moving to new objects or environments usually means collecting fresh labels and retraining from scratch. Vision foundation models (VFMs) invert that workflow by pretraining a general-purpose visual representation on massive corpora so it can adapt across many downstream tasks. Since 2023, ICRA and IROS papers have increasingly adopted these pretrained backbones as the core of robot perception.

---

## 11.1 What Is a Foundation Model?

A **foundation model** is a large model pretrained on broad data and adaptable to a wide range of downstream tasks. In conventional robot perception, changing the target objects or deployment site often forces engineers to repeat the expensive cycle of data collection, annotation, and retraining. Foundation models aim to amortize that cost by reusing a single pretrained representation across environments. Even so, transfer to unseen scenes varies substantially by model architecture and target domain, so zero-shot performance on a benchmark must always be distinguished from performance after domain adaptation.

Four traits define this paradigm and explain why it behaves differently from standard task-specific networks:

- **Scale**: architectures spanning hundreds of millions to billions of parameters.
- **Pretraining**: broad exposure to hundreds of millions of images (often paired with text or unlabeled).
- **Zero-shot / Few-shot**: capability to tackle new tasks with no extra training or only a handful of examples.
- **Transfer**: representations that generalize across diverse visual domains without rebuilding labels from scratch for every environment.

Underlying these models are **scaling laws**—empirical power-law relationships linking downstream performance to model size, dataset size, and training compute (Kaplan et al., 2020; Zhai et al., 2022 for ViT). Those scaling trends guided the development of GPT, CLIP, and SAM, though raw parameter count is not enough on its own: Hoffmann et al. (2022) showed that increasing model size without scaling data and compute in proportion yields diminishing returns.

> **Further reading**
> - [Bommasani et al., "On the Opportunities and Risks of Foundation Models" (2021)](https://arxiv.org/abs/2108.07258) — Stanford report that defined the term "foundation model".
> - [Two Minute Papers — videos on foundation models](https://www.youtube.com/@TwoMinutePapers) — a quick way to keep up with the latest VFM research.
> - [HuggingFace Model Hub](https://huggingface.co/models) — thousands of pretrained models ready to use.

---

## 11.2 Major VFMs

Modern robotics relies on five complementary VFM families, each solving a different piece of the perception puzzle: DINOv2 for self-supervised representation learning, SAM for promptable segmentation, CLIP for image-text alignment, Depth Anything for monocular depth estimation, and GroundingDINO for open-vocabulary detection.

### 11.2.1 DINOv2

DINOv2 is a **self-supervised Vision Transformer** that learns general-purpose visual features directly from images without human labels. By combining self-distillation (DINO), masked image modeling (iBOT), and KoLeo regularization, it produces both global scene descriptors and fine-grained dense patch features that transfer cleanly across tasks—from image retrieval and linear-probe semantic segmentation to serving as a feature backbone for 3D reconstruction. In robotics, those dense features are particularly valuable for SLAM and visual odometry: whereas classical keypoints fail on blank walls or uniform floors, DINOv2 features remain discriminative enough to support stable matching and reduce tracking failures in textureless environments.

The following snippet extracts DINOv2 features with HuggingFace `transformers`:

```python
import torch
from transformers import AutoModel, AutoImageProcessor

processor = AutoImageProcessor.from_pretrained('facebook/dinov2-base')
model = AutoModel.from_pretrained('facebook/dinov2-base')

inputs = processor(images=image, return_tensors="pt")
outputs = model(**inputs)
features = outputs.last_hidden_state  # (1, num_patches+1, 768): CLS + patch features
```

In `last_hidden_state`, the leading `[CLS]` token summarizes the entire image for global tasks such as classification or retrieval, whereas the remaining tokens encode per-patch representations suited to dense prediction tasks such as segmentation and geometric matching.

> **Further reading**
> - [Oquab et al., "DINOv2: Learning Robust Visual Features without Supervision" (2023)](https://arxiv.org/abs/2304.07193) — the original DINOv2 paper.
> - [DINOv2 GitHub](https://github.com/facebookresearch/dinov2) — official code and pretrained models.
> - [HuggingFace — DINOv2](https://huggingface.co/docs/transformers/model_doc/dinov2) — ready to use on HuggingFace.
> - [Yannic Kilcher — DINO explained](https://www.youtube.com/watch?v=h3ij3F3cPIk) — explains self-distillation in DINOv1 and also helps with understanding DINOv2.

### 11.2.2 SAM (Segment Anything Model)

Conventional segmentation networks can only outline categories present in their training set: a model trained on "chair, table, person" has no way to segment a "cup." **SAM** breaks that closed-set constraint through **promptable segmentation**, outputting object masks from geometric prompts such as points, bounding boxes, or rough masks. Trained on 1.1B masks, it separates objectness from fixed class labels—a major shift for robots that must isolate and manipulate unfamiliar objects on the fly. Note that text prompts were only an exploratory experiment in the original paper and are supported by neither the public checkpoints nor the API; text-driven segmentation is instead achieved by composing GroundingDINO with SAM (§11.2.5). As with other foundation models, SAM's accuracy also degrades under strong distribution shifts such as medical, satellite, or underwater imagery (§11.5).

Architecturally, SAM splits inference into three components so a heavy image pass can be amortized across multiple interactive prompts:

- **Image Encoder**: embeds the input image once using a ViT backbone.
- **Prompt Encoder**: maps point, box, or mask prompts into prompt embeddings.
- **Mask Decoder**: a lightweight decoder that fuses image and prompt embeddings to predict masks.

**SAM2** extends this promptable design from static images to video at higher speed. A single point or box marked in the first frame is propagated across subsequent frames, allowing a robot to track and maintain an object mask continuously even while the camera or object moves.

You can run point-prompted segmentation with SAM in a few lines:

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

Setting `multimask_output=True` returns three candidate masks corresponding to nested granularities (whole object, part, and smaller sub-part) to resolve ambiguity from a single point click. Because `scores` estimates predicted mask quality (IoU) rather than which granularity the user intended, selecting a specific part or whole object requires either adjusting the prompt or inspecting all three candidates.

> **Further reading**
> - [Kirillov et al., "Segment Anything" (2023)](https://arxiv.org/abs/2304.02643) — the original SAM paper.
> - [Ravi et al., "SAM 2: Segment Anything in Images and Videos" (2024)](https://arxiv.org/abs/2408.00714) — the original SAM2 paper. Extension to video segmentation.
> - [Segment Anything GitHub](https://github.com/facebookresearch/segment-anything) — official code.
> - [Segment Anything Explained](https://www.youtube.com/watch?v=KRAJd4_rNrc) — understand SAM's architecture and impact.
> - [HuggingFace — SAM](https://huggingface.co/docs/transformers/model_doc/sam) — ready to use on HuggingFace.

> **Exercise**: [SAM2 Interactive Segmentation](https://alexjunholee.github.io/robotics-practice/app.html#hf_sam)
> Try prompt-based segmentation on images using the SAM2 model directly (HuggingFace Space).

### 11.2.3 CLIP

While DINOv2 and SAM operate purely on pixels and geometric prompts, **CLIP** bridges vision and language by mapping images and text into a shared embedding space through contrastive learning on 400M image-text pairs. Before CLIP, classification required a predefined label list baked into the final network head. By placing images and text in the same metric space, CLIP enables zero-shot image classification, image-text retrieval, and open-vocabulary detection from arbitrary phrases—making it possible to specify a robot's target in natural language, such as "red mug on a wooden table."

The snippet below computes zero-shot class probabilities with CLIP:

```python
import clip
import torch

model, preprocess = clip.load("ViT-B/32", device="cuda")

image = preprocess(Image.open("image.jpg")).unsqueeze(0).to("cuda")
text = clip.tokenize(["a dog", "a cat", "a car"]).to("cuda")

with torch.no_grad():
    image_features = model.encode_image(image)
    text_features = model.encode_text(text)

    # CLIP only yields a cosine similarity if both features are L2-normalized before the inner product
    image_features /= image_features.norm(dim=-1, keepdim=True)
    text_features /= text_features.norm(dim=-1, keepdim=True)
    similarity = (100.0 * image_features @ text_features.T).softmax(dim=-1)  # 100.0 ≈ model.logit_scale.exp(), the learned temperature scale
    print(similarity)  # similarity between each text and the image
```

Here `@` denotes matrix multiplication. L2-normalizing both `image_features` and `text_features` before taking their inner product ensures the output measures cosine similarity rather than an unnormalized dot product; scaling by the learned temperature and applying softmax then converts those similarities into zero-shot class probabilities.

> **Further reading**
> - [Radford et al., "Learning Transferable Visual Models From Natural Language Supervision" (2021)](https://arxiv.org/abs/2103.00020) — the original CLIP paper.
> - [OpenAI CLIP GitHub](https://github.com/openai/CLIP) — official code and pretrained models.
> - [Yannic Kilcher — CLIP explained](https://www.youtube.com/watch?v=T9XSU0pKX2E) — a clear walk-through of the CLIP idea.
> - [HuggingFace — CLIP](https://huggingface.co/docs/transformers/model_doc/clip) — use various CLIP variants on HuggingFace.

### 11.2.4 Depth Anything

Semantic recognition alone cannot tell a robot how far away an obstacle sits. **Depth Anything** addresses geometric perception as a **monocular depth foundation model**, estimating relative depth from a single RGB image. Trained on 1.5M labeled images and 62M unlabeled images, it generalizes strongly across indoor (NYU), outdoor (KITTI), and zero-shot benchmarks, though accuracy still drops in regimes far outside its training distribution such as endoscopic or underwater imagery—again separating zero-shot inference from results fine-tuned to a target domain.

**Depth Anything V2** sharpens relative-depth quality substantially by retraining on synthetic labels, and both V1 and V2 offer **metric-depth** variants alongside relative-depth checkpoints. For robotics pipelines that need distances in physical meters—such as collision checking or grasp planning—rather than only near-to-far ordering, those metric-depth models are the relevant choice.

> **Further reading**
> - [Yang et al., "Depth Anything: Unleashing the Power of Large-Scale Unlabeled Data" (2024)](https://arxiv.org/abs/2401.10891) — the original Depth Anything paper.
> - [Yang et al., "Depth Anything V2" (2024)](https://arxiv.org/abs/2406.09414) — the original V2 paper. Metric depth support.
> - [Depth Anything GitHub](https://github.com/LiheYoung/Depth-Anything) — official code.
> - [HuggingFace — Depth Anything](https://huggingface.co/docs/transformers/model_doc/depth_anything) — ready to use on HuggingFace.

> **Exercise**: [Depth Anything V2](https://alexjunholee.github.io/robotics-practice/app.html#hf_depth)
> Try the Depth Anything V2 model to estimate depth from images directly (HuggingFace Space).

### 11.2.5 GroundingDINO

Closed-set detectors such as standard YOLO and Faster R-CNN models can only localize categories fixed during training. **GroundingDINO** brings **open-vocabulary object detection** to localization by pairing an image with arbitrary text queries and returning bounding boxes for matching regions, allowing a phrase like "red cup" to drive detection without training a new classification head:

```
Input: image + "person. car. traffic light."
Output: bounding boxes for the corresponding objects
```

Combining GroundingDINO with SAM yields **Grounded-SAM**, a widely used two-stage composition for text-prompted detection and segmentation. GroundingDINO first predicts a bounding box for a natural-language query such as "red cup," and SAM segments the pixel-accurate mask inside that box—supplying text-conditioned object regions and masks directly to downstream manipulation or mapping pipelines.

> **Further reading**
> - [Liu et al., "Grounding DINO: Marrying DINO with Grounded Pre-Training for Open-Set Object Detection" (2023)](https://arxiv.org/abs/2303.05499) — the original GroundingDINO paper.
> - [Grounded-SAM GitHub](https://github.com/IDEA-Research/Grounded-Segment-Anything) — text-based detection+segmentation pipeline.
> - [HuggingFace — Grounding DINO](https://huggingface.co/docs/transformers/model_doc/grounding-dino) — use on HuggingFace.

> **Exercise**: [Grounding DINO Demo](https://alexjunholee.github.io/robotics-practice/app.html#hf_grounding_dino)
> Try open-vocabulary detection that finds arbitrary objects in images from text prompts (HuggingFace Space).

---

## 11.3 Spatial AI Applications of VFMs

Individual VFMs each solve one narrow capability, yet real robotic systems gain their leverage by composing those models into a coherent spatial-understanding pipeline. Three integration patterns appear most frequently in Spatial AI:

- **Open-vocabulary scene understanding and zero-shot segmentation**: Robots operating in unstructured human environments cannot rely on a static, pre-enumerated object catalog; they have to parse commands such as "navigate to the red chair" in novel rooms without per-environment labeling. Combining CLIP, GroundingDINO, and SAM supports both open-vocabulary object localization and zero-shot semantic segmentation under natural-language queries.
- **Dense features for SLAM**: Classical SLAM pipelines depend on hand-crafted keypoints such as ORB and SIFT, which become scarce or ambiguous on blank walls and uniform floors. Replacing or augmenting those keypoints with DINOv2's dense semantic features allows reliable correspondence matching across textureless regions and lowers tracking failure rates—an idea explored in recent systems pairing DROID-SLAM with DINOv2.
- **3D scene understanding**: Lifting 2D VFM features into 3D scene representations such as NeRF or 3D Gaussian Splatting (e.g., Semantic NeRF, Feature 3DGS) embeds semantic meaning directly into 3D geometry. Once features live in 3D space, a system such as LERF, LangSplat, or ConceptGraphs can answer spatial text queries like "where is the chair in this 3D map?"—a thread explored further in Ch.13.

> **Further reading**
> - [Kerr et al., "LERF: Language Embedded Radiance Fields" (2023)](https://arxiv.org/abs/2303.09553) — work that embeds CLIP features into NeRF. A representative example of Spatial AI.
> - [Tschernezki et al., "Neural Feature Fusion Fields: 3D Distillation of Self-Supervised 2D Image Representations" (2022)](https://arxiv.org/abs/2209.03494) — early work on lifting 2D features into 3D.
> - [Papers With Code — 3D Scene Understanding](https://paperswithcode.com/task/3d-scene-understanding) — latest research trends.

---

## 11.4 Lightweight Models and Edge Deployment

Deploying VFMs inside a robot's Local Module runs straight into hardware limits. Because foundation models carry hundreds of millions of parameters, their memory footprint and compute cost are high: a ViT-B-class backbone runs comfortably on a desktop GPU, yet hitting real-time deadlines on embedded onboard hardware alongside planning and control is far harder. Every task also carries a different latency budget—image retrieval, map updates, and high-rate manipulation cannot share the same timing allowance—so model compression and edge deployment determine whether a strong paper model actually runs on a physical robot.

Three primary compression techniques trade off accuracy, engineering effort, and runtime speedup in different ways:

| Technique | Description |
|------|------|
| **Distillation** | transfer knowledge from a large model to a small one |
| **Quantization** | reduce precision from FP32 → INT8/INT4 |
| **Pruning** | remove unimportant weights |

Quantization preserves the existing network structure and is therefore the easiest to apply. Pruning removes redundant weights to reduce actual compute, though it can degrade accuracy if pushed too aggressively. Distillation achieves the largest speedups by training a compact student architecture from scratch, at the cost of higher training overhead.

In practice, teams pair these techniques with purpose-built lightweight variants—such as **FastSAM** (a YOLO-based re-implementation of SAM), **MobileSAM**, and **EfficientViT-SAM**—and compile them onto target runtimes including **TensorRT** (optimized for NVIDIA GPUs), **ONNX Runtime** (cross-platform), and **TFLite** (mobile and embedded targets). A typical deployment workflow exports a PyTorch model to ONNX before building a TensorRT engine:

```python
# TensorRT conversion example (PyTorch → ONNX → TensorRT)
import torch

# 1. Export to ONNX
torch.onnx.export(model, dummy_input, "model.onnx")

# 2. Convert to TensorRT (using trtexec)
# trtexec --onnx=model.onnx --saveEngine=model.trt --fp16
```

On NVIDIA Jetson platforms, TensorRT is one standard inference backend, though FP16 or INT8 latency, memory consumption, and task-metric shifts depend heavily on the model graph, input resolution, batch size, Jetson power mode, and TensorRT/CUDA versions. Always benchmark against an FP32 baseline on the exact target hardware and validation split, and when using INT8, run calibration on representative data followed by a fresh accuracy check.

> **Further reading**
> - [NVIDIA TensorRT documentation](https://docs.nvidia.com/deeplearning/tensorrt/) — TensorRT usage and optimization guide.
> - [ONNX Runtime](https://onnxruntime.ai/) — cross-platform inference optimization.
> - [MobileSAM GitHub](https://github.com/ChaoningZhang/MobileSAM) — mobile-lightweight version of SAM.
> - [FastSAM GitHub](https://github.com/CASIA-IVA-Lab/FastSAM) — YOLO-based lightweight SAM.
> - [NVIDIA Jetson AI Courses](https://developer.nvidia.com/embedded/learn/jetson-ai-certification-programs) — edge deployment practice.

---

## 11.5 Advanced: VFM Fine-tuning and Adaptation

Zero-shot VFM inference works remarkably well on natural web-like scenes, yet accuracy drops sharply in specialized domains such as medical, satellite, or underwater imagery. Adapting a model by updating all hundreds of millions of parameters is expensive in both GPU memory and labeled data. **Parameter-efficient fine-tuning (PEFT)** resolves this bottleneck by freezing most of the pretrained backbone and updating only a tiny subset of parameters while approaching full fine-tuning quality.

The table below compares five common adaptation strategies across parameter budget, accuracy, memory footprint, and implementation complexity:

| Strategy | Fraction of trained params | Performance | GPU memory | Application difficulty |
|------|-------------------|------|-----------|------------|
| **Full fine-tuning** | 100% | best (given enough data) | very high | low |
| **Linear probing** | <1% (head only) | low | low | very low |
| **LoRA** | 0.1~1% | high | low | moderate |
| **Adapter** | 1~5% | high | moderate | moderate |
| **Prompt tuning** | <0.1% | moderate | low | high |

**LoRA (Low-Rank Adaptation)** freezes a pretrained weight matrix $\mathbf{W}$ and learns a compact low-rank update alongside it:

$$\mathbf{W}' = \mathbf{W} + \Delta\mathbf{W} = \mathbf{W} + \mathbf{B}\mathbf{A}$$

Here $\mathbf{W}$ is a $d \times d$ matrix, $\mathbf{B}$ is $d \times r$, and $\mathbf{A}$ is $r \times d$ with rank $r \ll d$. Instead of updating all $d^2$ entries of $\mathbf{W}$, training updates only $2dr$ parameters: for $d = 768$ and $r = 8$, the trainable count drops from 589,824 to 12,288 (about 2%). Using HuggingFace `peft`, LoRA attaches cleanly to a DINOv2 backbone:

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
# Example output: trainable params: 589,824 || all params: 87,185,674 || trainable%: 0.68%
```

Two alternative PEFT mechanisms trade off parameter count and inference cost differently:

- **Adapter**: inserts small bottleneck modules between frozen Transformer blocks:

```
Input → [Frozen Attention] → [Adapter: down_proj → ReLU → up_proj] → [Frozen FFN] → Output
```

  Unlike LoRA—whose low-rank matrices merge directly into the pretrained weights after training for zero inference overhead—adapters introduce extra layers and therefore add slight latency at runtime.
- **Prompt tuning**: leaves the network weights untouched and prepends learnable virtual tokens to the input—for example, Visual Prompt Tuning (VPT), which injects learnable tokens into every ViT layer. It offers the highest parameter efficiency, though accuracy typically trails LoRA.

When adapting **SAM** specifically to a new visual domain, practitioners either alter how prompts are supplied or adapt the encoder weights themselves:

1. **Grid prompt**: partition the image into an NxN grid and query every intersection as a point prompt.
2. **Learned prompt generator**: train a lightweight auxiliary network that predicts point or box prompts directly from the input image.
3. **LoRA + SAM**: apply LoRA to the heavy image encoder to learn domain-specific features while fine-tuning the lightweight mask decoder directly, as illustrated below:

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

To compare adaptation strategies fairly, evaluate them across a consistent protocol matrix covering baseline generality, data efficiency, upper-bound accuracy, and parameter efficiency:

| Protocol | Description | Purpose of comparison |
|---------|------|----------|
| **Zero-shot** | evaluate without training | check the baseline generality of the VFM |
| **Few-shot (1/5/10-shot)** | train with a small number of samples per class | compare data efficiency |
| **Full fine-tune** | use the full training set | check the upper bound |
| **PEFT (LoRA, etc.)** | train with few parameters | efficiency-performance trade-off |

Controlled comparisons must hold the backbone, data split, and augmentation pipeline fixed across methods. Because few-shot runs exhibit high variance across random seeds, report mean and standard deviation over 3–5 independent repetitions.

> **Further reading**
> - [Hu et al., "LoRA: Low-Rank Adaptation of Large Language Models" (2022)](https://arxiv.org/abs/2106.09685) — the original LoRA paper (for LLMs, but directly applicable to ViTs).
> - [HuggingFace PEFT library](https://github.com/huggingface/peft) — implementations of LoRA, Adapter, and other PEFT methods.
> - [Chen et al., "SAM Fails to Segment Anything? — SAM-Adapter" (2023)](https://arxiv.org/abs/2304.09148) — a case study of SAM domain adaptation.

---

> **Technical Timeline: Vision Foundation Models**
> - **2021**: CLIP (OpenAI) trains a shared embedding on 400M image-text pairs and demonstrates zero-shot classification, while DINO (ICCV 2021) establishes the potential of self-supervised ViTs.
> - **2022**: Self-supervised pretraining methods such as Masked Autoencoders (MAE, CVPR 2022) gain wide adoption.
> - **2023**: SAM (Segment Anything Model), trained on 11M images and 1.1B masks, brings foundation-model generality to promptable segmentation, and DINOv2 sets a new standard for self-supervised vision features.
> - **2024**: VFM capabilities broaden rapidly through SAM2 (video segmentation), Depth Anything V2 (metric depth), and Florence-2 (unified vision tasks), alongside active work on lightweight compression and edge deployment.
> - **2025~**: 3D extensions of VFMs and multimodal unification accelerate toward single foundation models spanning detection, segmentation, depth, and tracking, establishing VFMs as the standard robotics perception backbone.
> - **Recent direction**: Zero-shot transfer motivates adopting foundation models in robot perception, with systems such as NLMap and ConceptGraphs combining CLIP, SAM, and DINOv2 for open-vocabulary scene understanding; on physical robots, lightweight variants such as FastSAM and MobileSAM must still be benchmarked jointly for latency and accuracy.

With VFMs supplying open-vocabulary visual perception, the next step is closing the loop from perception and language instructions to physical motion—the domain of Vision-Language-Action models and embodied AI in Ch.12.
