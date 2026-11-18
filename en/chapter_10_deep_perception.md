# Ch.10 — Deep Learning for Perception

Classical computer vision (Ch.9) builds explicit image-processing pipelines and geometric estimators from first principles, ending at the boundary where neural networks began replacing hand-crafted local features. Deep perception crosses that boundary completely: instead of designing filters and descriptors by hand, models learn hierarchical visual representations directly from data to predict an object's category, bounding box, pixel mask, or depth across an entire scene.

---

## 10.1 Choosing a Framework

Before training or fine-tuning a perception model, you have to choose a deep-learning framework. That choice determines which research repositories and pretrained checkpoints you can run out of the box, and mastering model definition, training, and debugging in one ecosystem makes it far easier to read and compare other codebases.

### 10.1.1 PyTorch (Recommended)

**PyTorch** is the practical default for robotics perception research because it pairs a clean Pythonic workflow with the largest open-source model ecosystem:

- **Dynamic computation graph (eager execution)**: control flow executes line by line like standard Python, making shapes and gradients straightforward to inspect.
- **Easy debugging**: standard Python debuggers and print checks work directly inside `forward()`.
- **Research ecosystem**: most newly released perception papers publish official PyTorch code, making it easy to reproduce and adapt recent work. (Reported conference usage shares vary with how repositories are sampled, so avoid citing fixed percentages without a documented census.)
- **Pretrained model hubs**: libraries such as `torchvision` and `timm` provide ready-to-use backbones.

To install a CUDA 12.1 build of PyTorch along with its vision and audio companions, run:

```bash
# CUDA 12.1 build
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121
```

In PyTorch, images are typically represented as 4D tensors in `(batch, channel, height, width)` order and moved explicitly onto a GPU when available:

```python
import torch
import torch.nn as nn

# Create a tensor
x = torch.randn(32, 3, 224, 224)  # (batch, channel, height, width)

# Use GPU
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
x = x.to(device)
```

> **Further reading**
> - [PyTorch Official Tutorials](https://pytorch.org/tutorials/) — Structured guide progressing from tensor basics to custom training loops.
> - [d2l.ai (Dive into Deep Learning)](https://d2l.ai/) — Interactive textbook pairing mathematical derivations with runnable PyTorch code.
> - [Andrej Karpathy — Neural Networks: Zero to Hero](https://www.youtube.com/playlist?list=PLAqhIrjkxbuWI23v9cThsA9GvCAUhRvKZ) — Hands-on lecture series building neural networks from scratch.
> - [Jaejun Yoo's Playground](http://jaejunyoo.blogspot.com/search/label/kr) — Korean-language blog with clear explanations of generative models such as GANs and VAEs.

### 10.1.2 TensorFlow / JAX

While PyTorch is the primary tool to learn first for perception research, two sibling frameworks play distinct roles in robotics:

- **TensorFlow**: mature production deployment tooling, especially **TF Lite** for running models on edge hardware such as NVIDIA Jetson or Raspberry Pi boards.
- **JAX**: functional, XLA-compiled array computing designed for high-throughput research, widely used in Google DeepMind-style codebases.

Even if your day-to-day experiments run in PyTorch, understanding where TFLite and JAX fit will save time when deploying onto embedded controllers or reading simulation-heavy research code.

> **Further reading**
> - [TensorFlow Official Guide](https://www.tensorflow.org/guide) — Documentation covering training pipelines and TFLite conversion.
> - [JAX Official Docs](https://jax.readthedocs.io/) — Reference for functional automatic differentiation and JIT compilation.

---

## 10.2 Deep Learning Fundamentals

Modern visual perception architectures rest on two core building blocks: **Convolutional Neural Networks (CNNs)**, which explain classic backbones like ResNet, and **Transformers**, which power newer architectures such as ViT and DETR.

### 10.2.1 Convolutional Neural Network (CNN)

A **CNN** extracts hierarchical spatial features by sliding learned filters across an image. Rather than relying on fixed hand-crafted filters like the Sobel operator or ORB descriptors in Ch.9, a CNN optimizes its filters end to end alongside the rest of the network—learning edges and textures in early layers and object parts in deeper ones.

Four operations form the backbone of standard CNN architectures:
- **Convolution layer**: extracts local spatial features using learnable kernels
- **Pooling layer**: downsamples spatial resolution (`Max` or `Average` pooling)
- **Activation function**: introduces nonlinearity (`ReLU`, `GELU`)
- **Batch normalization**: normalizes intermediate activations to stabilize training

The `ConvBlock` below illustrates the standard convolution–normalization–activation pattern:

```python
# Simple CNN block
class ConvBlock(nn.Module):
    def __init__(self, in_ch, out_ch):
        super().__init__()
        self.conv = nn.Conv2d(in_ch, out_ch, kernel_size=3, padding=1)
        self.bn = nn.BatchNorm2d(out_ch)
        self.relu = nn.ReLU(inplace=True)

    def forward(self, x):
        return self.relu(self.bn(self.conv(x)))
```

From a linear-algebra viewpoint, each spatial step of a convolution computes the inner product between a filter kernel and a local image patch—putting matrix multiplication directly to work. Setting `kernel_size=3, padding=1` keeps the output height and width identical to the input, which is why this configuration recurs across convolutional backbones.

> **Further reading**
> - [Stanford CS231n — Convolutional Neural Networks for Visual Recognition](https://www.youtube.com/playlist?list=PLoROMvodv4rMFqRtEuo6SGjY4XbRIVRd4) — Classic lecture series covering CNN foundations and visual recognition.
> - [d2l.ai — CNN chapter](https://d2l.ai/chapter_convolutional-neural-networks/index.html) — Code and math side by side.
> - [3Blue1Brown — But what is a Neural Network?](https://www.youtube.com/watch?v=aircAruvnKk) — Visual intuition for layered neural representations.

### 10.2.2 Attention & Transformer

Whereas a convolutional layer aggregates only a local neighborhood—needing many stacked layers to build a global receptive field—**self-attention** computes pairwise interactions across all positions in a sequence within a single layer:

```
Attention(Q, K, V) = softmax(QK^T / √d_k) V
```

Since 2020, vision systems have increasingly adopted pure Transformers as well as CNN–Transformer hybrids for classification, detection, and segmentation.

The **Vision Transformer (ViT)** adapts the Transformer directly to images by splitting an image into fixed-size patches (such as 16×16 pixels), projecting each patch into a token embedding, and feeding the resulting sequence into a standard Transformer encoder. Under large-scale pretraining, the original ViT paper outperformed its comparison CNN baselines on image classification, establishing Transformer backbones as a mainstay across modern vision tasks.

> **Further reading**
> - [Vaswani et al., "Attention Is All You Need" (2017)](https://arxiv.org/abs/1706.03762) — The original Transformer paper.
> - [Dosovitskiy et al., "An Image is Worth 16x16 Words" (2020)](https://arxiv.org/abs/2010.11929) — The original ViT paper.
> - [Yannic Kilcher — Vision Transformer explanation](https://www.youtube.com/watch?v=TrdevFK_am4) — Accessible walk-through of the ViT architecture.
> - [Andrej Karpathy — Let's build GPT from scratch](https://www.youtube.com/watch?v=kCc8FmEb1nY) — Builds a Transformer from the ground up; NLP-focused, yet directly clarifies ViT's attention mechanics.

---

## 10.3 Image Classification

**Image classification** predicts what category an image depicts. Even when a robot ultimately needs bounding boxes or pixel masks, classification remains foundational because detection and segmentation networks embed classifiers inside their heads—and pretrained classification backbones such as ResNet or ViT serve as general feature extractors for downstream tasks.

Four model families capture the main architectural trade-offs in classification backbones:

| Model | Characteristics | Use |
| --- | --- | --- |
| ResNet | Residual connection, stable training | Backbone network |
| EfficientNet | Compound scaling, efficient | Mobile, efficiency-focused |
| ViT | Transformer-based | Large-scale data, high performance |
| ConvNeXt | Modernized CNN | Competes with ViT |

In particular, **ResNet** (2015) solved gradient degradation in very deep networks by adding each block's input directly to its output via a residual skip connection, a design pattern adopted across almost every subsequent vision architecture.

To reuse an ImageNet-pretrained backbone as a fixed or fine-tunable feature extractor in PyTorch, strip its final linear classification head (`fc`) by replacing it with `nn.Identity()`:

```python
import torchvision.models as models

# Pretrained ResNet50
model = models.resnet50(weights='IMAGENET1K_V2')

# Use as a feature extractor
model.fc = nn.Identity()  # remove the last FC
features = model(x)  # (batch, 2048)
```

This **transfer-learning** pattern supplies a rich visual representation from millions of labeled images, reducing how much task-specific data a robot needs when learning a new recognition problem.

> **Further reading**
> - [He et al., "Deep Residual Learning for Image Recognition" (2015)](https://arxiv.org/abs/1512.03385) — The original ResNet paper, foundational to modern vision architectures.
> - [Papers With Code — Image Classification](https://paperswithcode.com/task/image-classification) — Starting point for public implementations and historical leaderboards; verify current numbers and protocols on each benchmark's official page and source paper.
> - [timm (PyTorch Image Models) library](https://github.com/huggingface/pytorch-image-models) — Unified interface for hundreds of pretrained vision backbones.
> - [Stanford CS231n — Training Neural Networks](https://www.youtube.com/playlist?list=PLoROMvodv4rMFqRtEuo6SGjY4XbRIVRd4) — Practical guidance on optimization, normalization, and regularization.

---

## 10.4 Object Detection

Knowing that a cup appears somewhere in an image is not enough for a robot arm to grasp it; manipulation and autonomous driving require knowing both *what* each object is and *where* it sits. **Object detection** solves both simultaneously by predicting a class label and a 2D bounding box for every detected instance.

### 10.4.1 Two-Stage Detectors

Two-stage detectors split localization and recognition into a coarse proposal step followed by fine region refinement. **Faster R-CNN** exemplifies this paradigm:
1. A **Region Proposal Network (RPN)** scans feature maps to propose candidate object regions.
2. **RoI Pooling** extracts fixed-size feature crops for each candidate box.
3. Separate heads classify each region and refine its bounding-box coordinates.

This "propose first, inspect closely second" design achieves high localization and classification accuracy—making it common in quality-critical settings such as industrial inspection—at the cost of slower inference than single-pass architectures, and it later served as the base for Mask R-CNN.

### 10.4.2 One-Stage Detectors

When a robot needs real-time perception at 30+ FPS, one-stage detectors skip the separate proposal stage and predict classes and bounding boxes in a single forward pass:

- **YOLO (You Only Look Once)**: partitions the image into a spatial grid and regresses boxes and class probabilities simultaneously in real time. Modern Ultralytics releases (`YOLOv5`, `YOLOv8`, `YOLOv11`) streamline setup, training, and inference, making them convenient for rapid robotics prototyping:

```python
from ultralytics import YOLO

# Load the model and run inference
model = YOLO('yolov8n.pt')  # nano model
results = model('image.jpg')

# Visualize the results
results[0].show()
```

- **SSD (Single Shot Detector)**: predicts detections across feature maps at multiple spatial scales. Early SSD struggled with small objects, a limitation later mitigated in the YOLOv3-and-subsequent generations by adopting Feature Pyramid Networks (FPN).

> **Further reading**
> - [Redmon et al., "You Only Look Once: Unified, Real-Time Object Detection" (2016)](https://arxiv.org/abs/1506.02640) — The concise, readable original YOLO paper.
> - [Ultralytics YOLOv8 docs](https://docs.ultralytics.com/) — End-to-end documentation from installation to custom dataset training.
> - [Papers With Code — Object Detection](https://paperswithcode.com/task/object-detection) — Starting point for public code and historical leaderboards; verify current metrics and protocols in the official benchmarks and papers.
> - [Dark Programmer — Understanding precision and recall](https://darkpgmr.tistory.com/162) — Clear walkthrough of detection evaluation metrics.

### 10.4.3 Transformer-based

**DETR (Detection Transformer)** reformulated object detection as a **set prediction** problem. Rather than generating thousands of hand-designed anchor boxes and pruning duplicates via Non-Maximum Suppression (NMS), DETR passes a fixed set of learnable **object queries** through a Transformer decoder and matches predictions to ground truth end to end. Although vanilla DETR suffered from slow convergence during training, its clean architecture inspired faster, higher-accuracy successors including Deformable DETR, DINO, and Co-DETR.

> **Further reading**
> - [Carion et al., "End-to-End Object Detection with Transformers" (2020)](https://arxiv.org/abs/2005.12872) — The original DETR paper.
> - [Yannic Kilcher — DETR explanation](https://www.youtube.com/watch?v=T35ba_VXkMY) — Visual walkthrough of bipartite matching and object queries.
> - [HuggingFace — Object Detection guide](https://huggingface.co/docs/transformers/tasks/object_detection) — Running DETR models with the `transformers` library.
> - [Zhao et al., "DETRs Beat YOLOs on Real-time Object Detection" (RT-DETR, CVPR 2024, arXiv:2304.08069)](https://arxiv.org/abs/2304.08069) — Reports speed-accuracy gains for real-time DETR variants under the paper's benchmark protocol.
> - [Cheng et al., "YOLO-World: Real-Time Open-Vocabulary Object Detection" (CVPR 2024, arXiv:2401.17270)](https://arxiv.org/abs/2401.17270) — Extends YOLO with text-prompted open-vocabulary detection, well suited to locating arbitrary objects in robotics.

---

## 10.5 Semantic Segmentation

Bounding boxes remain coarse approximations for irregular objects or drivable surfaces. **Semantic segmentation** assigns a class label to every single pixel, tracing object–background boundaries far more finely—essential when distinguishing roads, sidewalks, and lane markings in autonomous driving, or isolating graspable surfaces on a cluttered workbench. Note, however, that a predicted semantic mask is not an exact physical contour; reliable robotic grasping still requires depth measurements, instance separation, and uncertainty checks.

Across CNN and Transformer eras, four architectures represent the major design milestones:

| Model | Characteristics |
| --- | --- |
| FCN | An early representative of fully convolutional end-to-end segmentation |
| U-Net | Encoder-Decoder structure, originated in medical imaging |
| DeepLab v3+ | Atrous convolution, multi-scale |
| SegFormer | Transformer-based, lightweight decoder |

Among these, **U-Net** established the standard encoder–decoder template with skip connections: the encoder downsamples resolution to capture semantic context, while the decoder restores full spatial resolution and merges fine detail across skip connections—a pattern reused heavily in depth estimation and generative modeling.

Using modern pretrained segmentation models such as SegFormer requires only a few lines via HuggingFace `transformers`:

```python
# Using a segmentation model (transformers library)
from transformers import SegformerForSemanticSegmentation

model = SegformerForSemanticSegmentation.from_pretrained(
    "nvidia/segformer-b0-finetuned-ade-512-512"
)
```

> **Further reading**
> - [Papers With Code — Semantic Segmentation](https://paperswithcode.com/task/semantic-segmentation) — Current benchmarks and open-source baselines.
> - [HuggingFace — Image Segmentation](https://huggingface.co/docs/transformers/tasks/semantic_segmentation) — Practical guide to training and evaluating models like SegFormer.
> - [Two Minute Papers — videos on semantic segmentation](https://www.youtube.com/@TwoMinutePapers) — Short visual summaries of recent segmentation papers.

---

## 10.6 Instance & Panoptic Segmentation

Semantic segmentation answers "which pixels belong to the class *chair*," yet it merges adjacent chairs into one blob. If a robot receives the command "pick up the chair on the left," it must separate individual object instances—and ideally understand background regions at the same time:

- **Instance Segmentation**: detects and masks each individual object instance separately. **Mask R-CNN** extends Faster R-CNN by adding a parallel pixel-mask prediction branch alongside box regression and classification.
- **Panoptic Segmentation**: unifies semantic and instance segmentation into a single coherent scene parse, distinguishing countable **things** (individual objects such as chairs, cars, or cups) by instance while labeling uncountable **stuff** (background regions such as floor, sky, or road) semantically.

> **Further reading**
> - [He et al., "Mask R-CNN" (2017)](https://arxiv.org/abs/1703.06870) — The landmark paper for instance segmentation.
> - [Detectron2](https://github.com/facebookresearch/detectron2) — Meta's modular detection and segmentation library with clean Mask R-CNN implementations.

---

## 10.7 Depth Estimation

While stereo cameras and LiDAR measure distance directly, payload- and cost-constrained robots—such as micro aerial vehicles or compact mobile platforms—often carry only a single RGB camera. **Monocular depth estimation** predicts a dense depth map from one image, using learned visual cues (perspective, occlusion, relative scale) to infer 3D layout without extra range hardware. Recent foundation-scale training has dramatically boosted cross-domain generalization.

Three model families illustrate the progression from general relative depth to metric predictions:
- **MiDaS**: trained across diverse multi-domain datasets for robust general-purpose depth estimation
- **Depth Anything**: leverages large-scale unlabeled data for foundation-model-level generalization
- **ZoeDepth**: bridges relative pretraining with metric depth prediction in meters

Running monocular depth estimation out of the box takes just a single HuggingFace `pipeline` call:

```python
# Using Depth Anything
from transformers import pipeline

pipe = pipeline("depth-estimation", model="LiheYoung/depth-anything-base-hf")
result = pipe("image.jpg")
depth = result['depth']
```

**Caveat**: by default, MiDaS and Depth Anything output **relative depth**—telling you that object A is closer than object B, not how many meters away A lies. Whenever downstream planning or grasping requires absolute scale, use a metric model such as ZoeDepth or the metric variant of Depth Anything V2, or align predictions against a physical scale sensor.

> **Further reading**
> - [Yang et al., "Depth Anything: Unleashing the Power of Large-Scale Unlabeled Data" (2024)](https://arxiv.org/abs/2401.10891) — The original Depth Anything paper.
> - [Godard et al., "Digging Into Self-Supervised Monocular Depth Estimation" (Monodepth2, ICCV 2019, arXiv:1806.01260)](https://arxiv.org/abs/1806.01260) — Foundational baseline for self-supervised monocular depth.
> - [HuggingFace — Monocular Depth Estimation](https://huggingface.co/docs/transformers/tasks/monocular_depth_estimation) — Ready-to-run inference and fine-tuning examples.
> - [Papers With Code — Monocular Depth Estimation](https://paperswithcode.com/task/monocular-depth-estimation) — Benchmarks across indoor and outdoor datasets.

---

## 10.8 Advanced: Training Recipes

Model architecture is only half of a perception experiment. Even with the network fixed, learning-rate schedules, data augmentation, and regularization strongly govern convergence speed and final accuracy—often separating a mediocre baseline from a competitive result.

**Learning Rate Schedule**:
- **Cosine Annealing with Warm-up**: ramps the learning rate linearly from zero to the target peak over the initial warm-up epochs (preventing early gradient spikes, especially in Transformers), then decays it smoothly along a half-cosine curve:

$$\eta_t = \eta_{min} + \frac{1}{2}(\eta_{max} - \eta_{min})\left(1 + \cos\left(\frac{t \cdot \pi}{T}\right)\right)$$

- **OneCycleLR**: raises the learning rate once before annealing it down, often enabling faster convergence ("super-convergence") within fewer epochs.

In PyTorch, both schedules wrap the optimizer cleanly:

```python
import torch.optim as optim

# Cosine Annealing (no warm-up). If you need warm-up, add it like pct_start in OneCycleLR below
optimizer = optim.AdamW(model.parameters(), lr=1e-4, weight_decay=0.05)
scheduler = optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=100)

# OneCycleLR
scheduler = optim.lr_scheduler.OneCycleLR(
    optimizer, max_lr=1e-3, total_steps=len(dataloader) * num_epochs,
    pct_start=0.1  # use the first 10% for warm-up
)
```

**Data Augmentation**: diversifying training inputs reduces overfitting to lighting, viewpoint, and background clutter. Four augmentation strategies appear frequently in vision recipes:

| Technique | Description | Main use |
|------|------|---------|
| **RandAugment** | applies N transformations at magnitude M at random | general classification |
| **CutMix** | replaces an image region with another image and mixes the labels proportionally | classification |
| **MixUp** | linearly interpolates two images and their labels | classification |
| **Mosaic** | composes 4 images into one | detection (YOLO family) |

`torchvision.transforms.v2` provides a built-in `RandAugment` transform ready to compose into a preprocessing pipeline:

```python
import torchvision.transforms.v2 as T

# RandAugment
transform = T.Compose([
    T.RandAugment(num_ops=2, magnitude=9),
    T.ToTensor(),
    T.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
])
```

**Regularization**: complementary constraints keep networks from memorizing training noise:
- **Label Smoothing**: replaces hard `0`/`1` targets with soft probabilities (e.g., `0.1`, `0.9`) via `nn.CrossEntropyLoss(label_smoothing=0.1)`, curbing overconfident logits.
- **Stochastic Depth**: randomly drops residual blocks during training, improving generalization in deep ResNet- and Transformer-style architectures.
- **Weight Decay**: typically set to `weight_decay=0.01~0.05`, using decoupled weight decay in `AdamW`.

**Gradient Clipping**: caps gradient norms before each optimizer step to prevent destabilizing spikes, a routine safeguard when training Transformers:

```python
torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
```

**Diagnosing Problems from the Loss Curve**: watching training and validation losses side by side quickly reveals which hyperparameter to adjust next:

| Pattern | Diagnosis | Response |
|------|------|------|
| train loss decreasing, val loss increasing | Overfitting | add augmentation, increase dropout/weight decay, get more data |
| train loss stuck at a high value | Underfitting | increase model size, adjust learning rate, reduce augmentation |
| train loss oscillates heavily | Learning rate too high | decrease learning rate |
| train loss becomes NaN | Gradient explosion | gradient clipping, sharply reduce learning rate, validate data |
| val loss drops early then completely plateaus | Learning rate too low or schedule issue | add warm-up, apply cosine schedule |

**Distributed Training — PyTorch DDP basics**: when datasets or backbones outgrow a single GPU's time budget, **DistributedDataParallel (DDP)** replicates the model across multiple GPUs and synchronizes gradients cleanly via `torchrun`:

```python
# Minimal DDP structure (launch with torchrun)
import os
import torch
import torch.distributed as dist
from torch.nn.parallel import DistributedDataParallel as DDP

dist.init_process_group("nccl")
local_rank = int(os.environ["LOCAL_RANK"])
torch.cuda.set_device(local_rank)
model = model.to(local_rank)
model = DDP(model, device_ids=[local_rank])

# Launch: torchrun --nproc_per_node=4 train.py
```

> **Further reading**
> - [Goyal et al., "Accurate, Large Minibatch SGD" (2017)](https://arxiv.org/abs/1706.02677) — Establishes the linear learning-rate scaling rule for large-batch distributed training.
> - [PyTorch DDP Tutorial](https://pytorch.org/tutorials/intermediate/ddp_tutorial.html) — Official guide to multi-GPU distributed training.
> - [Wightman et al., "ResNet strikes back" (2021)](https://arxiv.org/abs/2110.00476) — Demonstrates how modern training recipes alone substantially boost a classic ResNet's accuracy.

> **Exercise**: [Data Augmentation visualization](https://alexjunholee.github.io/robotics-practice/app.html#data_augmentation)
> Inspect interactively how RandAugment, CutMix, and MixUp alter training images and labels.

> **Exercise**: [Learning Rate Schedule visualization](https://alexjunholee.github.io/robotics-practice/app.html#lr_schedule)
> Compare Cosine Annealing and OneCycleLR curves and explore how warm-up and decay parameters shape the schedule.

---

## 10.9 Advanced: Self-Supervised and Contrastive Learning

Robotics data is inherently label-scarce: a deployed robot can record tens of thousands of camera frames effortlessly, yet annotating every frame with bounding boxes or segmentation masks is prohibitive. **Self-supervised learning** extracts supervisory signals directly from the raw images themselves, training strong visual representations without manual labels.

**Contrastive Learning** trains an encoder so that differently augmented views of the same image (**positive pairs**) cluster together in embedding space while views from distinct images (**negative pairs**) are pushed apart:
- **SimCLR**: generates positive pairs by applying random augmentations to each image and treats all other images in the minibatch as negatives, which requires large batch sizes for strong representations.
- **MoCo (Momentum Contrast)**: maintains a dynamic queue of encoded keys updated by a momentum encoder, decoupling the number of available negatives from the current batch size.

Both approaches optimize variants of the **InfoNCE loss**:

$$\mathcal{L} = -\log \frac{\exp(\text{sim}(z_i, z_j) / \tau)}{\sum_{k=1}^{2N} \mathbb{1}_{[k \neq i]} \exp(\text{sim}(z_i, z_k) / \tau)}$$

Here `sim` denotes cosine similarity and `τ` is a temperature hyperparameter; the numerator pulls positive pairs together while the denominator pushes them away from all negative pairs in the pool.

**Masked Image Modeling — MAE**: adapting BERT's masked language modeling to Vision Transformers, Masked Autoencoders (**MAE**) randomly mask 75% of an image's patches and train the network to reconstruct missing pixels from the visible 25%:
- **Why 75% masking**: images carry far more spatial redundancy than text, so masking most patches prevents trivial local interpolation and forces the encoder to learn global semantic structure.
- **Efficiency**: because the heavy ViT encoder processes only the unmasked 25% of tokens, pretraining speeds up substantially—the original MAE paper reports a pretraining time reduction of more than 3×. (Compute savings do not equal the masking ratio directly, since self-attention scales superlinearly in token count and the lightweight decoder adds overhead.)

**Connection to DINOv2**: **DINOv2** learns general-purpose dense features through **self-distillation**, pairing a student network with a teacher whose weights are updated as an exponential moving average (EMA) of the student:
- **Self-distillation**: student and teacher share an identical ViT architecture, with the teacher tracking a slow-moving EMA of the student.
- **Sharpening + batch normalization**: sharpening (low temperature) on the teacher's output combined with SwAV's Sinkhorn-Knopp batch normalization prevents representation collapse, replacing the mean-centering used in DINO v1.
- **Frozen-backbone evaluation**: frozen DINOv2 features perform competitively with supervised backbones, though evaluation protocols differ across tasks—ImageNet classification uses a training-free `k`-NN classifier, whereas ADE20K semantic segmentation trains a linear probe and reports mIoU. Always check the paper's exact metric and model size when citing numbers.

Because frozen self-supervised backbones already capture rich semantics, adapting DINOv2 to a new classification task often requires training only a lightweight linear head:

```python
from transformers import AutoModel, AutoImageProcessor
import torch.nn as nn

# Load the DINOv2 backbone
processor = AutoImageProcessor.from_pretrained("facebook/dinov2-base")
backbone = AutoModel.from_pretrained("facebook/dinov2-base")

# Freeze the backbone and train only the classification head
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

> **Further reading**
> - [Chen et al., "A Simple Framework for Contrastive Learning of Visual Representations (SimCLR)" (2020)](https://arxiv.org/abs/2002.05709) — Foundational contrastive learning framework.
> - [He et al., "Masked Autoencoders Are Scalable Vision Learners" (2022)](https://arxiv.org/abs/2111.06377) — The original MAE paper.
> - [Oquab et al., "DINOv2: Learning Robust Visual Features without Supervision" (2024)](https://arxiv.org/abs/2304.07193) — The original DINOv2 paper.

---

## 10.10 Advanced: Knowledge Distillation

Large perception backbones and vision foundation models often exceed the latency and memory limits of onboard robotics compute such as an NVIDIA Jetson. **Knowledge distillation** compresses a large pretrained **teacher** model into a compact **student** model capable of real-time edge inference—for instance, distilling SAM into lightweight real-time variants.

Rather than training the student solely on one-hot ground-truth labels, distillation trains it to match the teacher's richer **soft targets**. For an image of a cat, a hard label is simply `[1, 0, 0]`, whereas a teacher might output `[0.85, 0.10, 0.05]`, encoding informative similarity structure (that a cat resembles a dog far more than an airplane).

**Soft Targets and Temperature Scaling**: dividing logits `z_t` (teacher) and `z_s` (student) by a temperature `τ > 1` (typically `τ = 3~5`) flattens the softmax distribution `σ(·)` so dark-knowledge inter-class relationships stand out more clearly in the KL-divergence loss:

$$\mathcal{L}_{KD} = \text{KL}\left(\sigma\left(\frac{z_t}{\tau}\right) \| \sigma\left(\frac{z_s}{\tau}\right)\right)$$

The overall training objective blends standard cross-entropy on ground-truth labels with temperature-scaled distillation loss:

$$\mathcal{L} = \alpha \cdot \mathcal{L}_{CE}(y, \sigma(z_s)) + (1 - \alpha) \cdot \tau^2 \cdot \mathcal{L}_{KD}$$

Multiplying the distillation loss by `τ^2` compensates for the `1/τ^2` scaling of gradient magnitudes introduced by dividing the logits by `τ`.

**Feature-based Distillation (FitNets)**: when matching output logits alone is insufficient—especially for dense prediction tasks—feature distillation aligns intermediate representations directly through a projection layer `r` that matches student and teacher channel dimensions:

$$\mathcal{L}_{feat} = \|f_t(x) - r(f_s(x))\|^2$$

In practice, distillation has produced fast, deployable counterparts to major vision foundation models:

| Teacher | Student | Method |
|---------|---------|------|
| SAM (ViT-H) | MobileSAM | Replace the image encoder with a lightweight ViT, distillation |
| SAM (ViT-H) | FastSAM | Replace the entire pipeline with a YOLO architecture |
| DINOv2-giant | DINOv2-small | Distill into a smaller version of the same architecture |

The function below implements standard logit-level knowledge distillation in PyTorch:

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

> **Further reading**
> - [Hinton et al., "Distilling the Knowledge in a Neural Network" (2015)](https://arxiv.org/abs/1503.02531) — The foundational knowledge distillation paper.
> - [Zhang et al., "Faster Segment Anything (MobileSAM)" (2023)](https://arxiv.org/abs/2306.14289) — Practical encoder distillation of SAM.
> - [Romero et al., "FitNets: Hints for Thin Deep Nets" (2015)](https://arxiv.org/abs/1412.6550) — Feature-based intermediate-layer distillation.

---

## 10.11 Advanced: Domain Adaptation

Perception models trained in simulation—or even on curated indoor benchmarks—frequently suffer sharp accuracy drops when deployed on a real outdoor robot. This mismatch between training and deployment distributions is called **domain shift**, and **domain adaptation** develops methods to bridge it—sitting directly at the heart of robotics' **sim-to-real gap**.

Formally, domain adaptation starts with a labeled **source domain** `D_s` (such as synthetic simulator renders or an existing public dataset) and an unlabeled or sparsely labeled **target domain** `D_t` (the physical deployment environment), aiming to preserve task performance on `D_t`.

**Domain Randomization**: the simplest and often most effective strategy randomizes simulator parameters aggressively during data generation so that real-world conditions look like just another variation within the training distribution:
- **Texture**: randomize wall, floor, and object surface textures on every episode
- **Lighting**: vary light positions, colors, and intensities
- **Camera parameters**: perturb focal length, mounting pose, and viewing angle
- **Physical parameters**: sample friction coefficients, masses, and inertias across plausible ranges

**Adversarial Domain Adaptation**: introduces a domain discriminator alongside the main task head so that the shared feature extractor learns representations useful for the task yet indistinguishable across `D_s` and `D_t`:

```
Input --> Feature Extractor --> [Task Classifier]      --> Task Loss
                              \--> [Domain Discriminator] --> Domain Loss (GRL)
```

A **Gradient Reversal Layer (GRL)** flips the sign of gradients flowing back from the domain discriminator, encouraging the feature extractor to confuse the discriminator while the task classifier trains normally on `D_s`:

$$\mathcal{L} = \mathcal{L}_{task}(D_s) - \lambda \cdot \mathcal{L}_{domain}(D_s, D_t)$$

The minus sign reflects this minimax game: the feature extractor maximizes domain confusion (analogous to GAN adversarial training) while minimizing task error.

**Test-Time Adaptation (TTA)**: updates a model online after deployment using only incoming unlabeled test frames, without touching the original training dataset:
- **TENT**: adapts batch-normalization affine parameters online by minimizing prediction entropy.
- **CoTTA**: extends TTA to continually shifting target distributions over long deployments.

The simplified TENT loop below freezes all network weights except batch-normalization scale and shift parameters (`weight` and `bias`), updating them against test-batch statistics:

```python
# Simplified TENT update
model.eval()
model.requires_grad_(False)          # freeze everything first
params = []
for m in model.modules():
    if isinstance(m, nn.BatchNorm2d):
        m.requires_grad_(True)       # re-enable only the BN affine parameters
        m.track_running_stats = False
        m.running_mean = None        # clear the running buffers so batch statistics are used in eval mode
        m.running_var = None
        params += [m.weight, m.bias]

optimizer = optim.SGD(params, lr=1e-4)  # optimize only the BN parameters

# Adaptation at inference time
for batch in test_loader:
    output = model(batch)
    loss = entropy(output)  # minimize prediction entropy
    loss.backward()
    optimizer.step()
    optimizer.zero_grad()
```

**Connection to the sim-to-real gap**: in practice, teams combine these tools according to their target environment and available real-world data:
1. Generate diverse synthetic training data via domain randomization.
2. Bridge residual visual gaps with adversarial adaptation (or targeted fine-tuning) on a small set of real frames.
3. Apply test-time adaptation after deployment if conditions drift continually.

Running all three stages is not a mandatory recipe—many deployed systems rely on domain randomization alone, fine-tune directly on real data, or disallow post-deployment weight updates under strict safety and compute budgets.

> **Further reading**
> - [Tobin et al., "Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World" (2017)](https://arxiv.org/abs/1703.06907) — The foundational domain randomization paper.
> - [Ganin et al., "Domain-Adversarial Training of Neural Networks" (2016)](https://arxiv.org/abs/1505.07818) — Introduces the Gradient Reversal Layer (GRL) for adversarial adaptation.
> - [Wang et al., "TENT: Fully Test-Time Adaptation by Entropy Minimization" (2021)](https://arxiv.org/abs/2006.10726) — Foundational entropy-minimization TTA method.
> - [Wen et al., "FoundationPose: Unified 6D Pose Estimation and Tracking of Novel Objects" (CVPR 2024, arXiv:2312.08344)](https://arxiv.org/abs/2312.08344) — Unified 6D pose estimation and tracking for novel objects from a CAD model or a few reference images.

---

> **Technical Timeline: Deep Learning for Perception**
> - **2012**: AlexNet wins the ImageNet challenge by a wide margin, launching the deep-learning revolution and signaling the shift away from hand-crafted features.
> - **2014~2016**: VGGNet, GoogLeNet, and ResNet emerge; ResNet's (2015) residual connections unlock networks hundreds of layers deep, while Faster R-CNN (2015) and YOLO (2016) bring real-time object detection to practice.
> - **2017**: "Attention Is All You Need" introduces the Transformer for NLP, laying the architectural foundation later adopted by computer vision.
> - **2020~2021**: Vision Transformer (ViT) popularizes patch-token processing and DETR brings Transformers to detection, while Swin Transformer reports gains over contemporary baselines across major vision benchmarks.
> - **2022~**: ConvNeXt shows modernized CNNs remain competitive, Segment Anything (SAM) turns segmentation into a foundation model, and Spatial AI—depth estimation and 3D scene understanding—becomes the next frontier.
> - **Recent direction**: rather than training separate task-specific models, robotics perception increasingly builds on general-purpose **Vision Foundation Models**—such as reusing DINOv2 features across detection, segmentation, and depth—the focus of Ch.11.
