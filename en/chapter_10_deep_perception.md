# Ch.10 — Deep Learning for Perception

These are the core techniques for a robot to understand "what it is looking at." Classical CV focused on "how to process the image and extract geometric relationships," whereas here the focus is on recognizing "what is in the image." Object detection, classification, segmentation — for a robot to carry out commands like "there's a red cup over there, pick it up," these techniques are essential.

---

## 10.1 Choosing a Framework

Which deep learning framework to use is a more important decision than it seems. To read and reproduce research code, you need to know the framework it uses; to build your own model, you have to know at least one properly.

### 10.1.1 PyTorch (Recommended)

**Strengths**:
- Intuitive dynamic graph (eager execution)
- Easy to debug
- Standard in the research community
- Rich pretrained models (torchvision, timm)

There is a practical reason. As of 2024, more than about 80% of code released with papers at major conferences such as NeurIPS, CVPR, and ICLR is in PyTorch. Without PyTorch, you essentially cannot read, run, and modify the latest papers.

**Install**:

```bash
# CUDA 12.1 build
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121
```

**Basic usage**:

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
> - [PyTorch Official Tutorials](https://pytorch.org/tutorials/) — organized systematically from beginner to advanced.
> - [d2l.ai (Dive into Deep Learning)](https://d2l.ai/) — interactive textbook. PyTorch code and math appear together.
> - [Andrej Karpathy — Neural Networks: Zero to Hero](https://www.youtube.com/playlist?list=PLAqhIrjkxbuWI23v9cThsA9GvCAUhRvKZ) — the former Tesla AI Director explains neural networks from scratch.
> - [Jaejun Yoo's Playground](http://jaejunyoo.blogspot.com/search/label/kr) — a Korean blog that explains generative models like GAN and VAE well.

### 10.1.2 TensorFlow / JAX

**TensorFlow**: strong for production deployment, TF Lite mobile support
**JAX**: high-performance computation, functional programming, for research

Since most recent research code is released in PyTorch, learn PyTorch first.

That said, TensorFlow Lite is still widely used when deploying models to a robot's edge devices (Jetson, Raspberry Pi, etc.), and JAX is heavily used in Google DeepMind-style research, so at least be aware they exist.

> **Further reading**
> - [TensorFlow Official Guide](https://www.tensorflow.org/guide) — covers TFLite conversion.
> - [JAX Official Docs](https://jax.readthedocs.io/) — functional deep learning framework.

---

## 10.2 Deep Learning Fundamentals

Without knowing the concepts in this section, you cannot understand "why deep learning overwhelms classical methods in image recognition." Without knowing the CNN structure, you don't see why ResNet matters; without knowing the Transformer, you can't understand why ViT and DETR replace earlier approaches.

### 10.2.1 Convolutional Neural Network (CNN)

This is the core architecture for extracting spatial features from images.

A CNN is an architecture that "automatically learns local patterns (edges, corners, textures) in images." In the classical CV covered earlier, features like SIFT and ORB were hand-crafted by humans, whereas a CNN learns the optimal features automatically from data. This is the turning point of deep learning.

**Main components**:
- **Convolution Layer**: extract features with filters
- **Pooling Layer**: reduce spatial size (Max, Average)
- **Activation**: introduce nonlinearity (ReLU, GELU)
- **Batch Normalization**: stabilize training

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

From a linear algebra perspective, a convolution is "the inner product of a filter (kernel) with an image patch." The matrix multiplication taught in class is used directly here. With `kernel_size=3, padding=1` the output size is kept the same as the input — this pattern shows up very often.

> **Further reading**
> - [Stanford CS231n — Convolutional Neural Networks for Visual Recognition](https://www.youtube.com/playlist?list=PLoROMvodv4rMFqRtEuo6SGjY4XbRIVRd4) — the canonical lecture series for understanding CNNs. Worth watching.
> - [d2l.ai — CNN chapter](https://d2l.ai/chapter_convolutional-neural-networks/index.html) — code and math explained together.
> - [3Blue1Brown — But what is a Neural Network?](https://www.youtube.com/watch?v=aircAruvnKk) — intuitive understanding of neural networks.

### 10.2.2 Attention & Transformer

**Self-Attention**: learns relationships between all positions within a sequence

```
Attention(Q, K, V) = softmax(QK^T / √d_k) V
```

Once you see the difference from a CNN, it is immediately clear why the Transformer is rising. CNNs exchange information only between "nearby pixels" (local receptive field), but the Transformer's Self-Attention lets "any part of the image reference any other part" (global attention). This is advantageous for grasping the overall context of an object. Since 2020, Transformers have rapidly replaced CNNs in vision.

**Vision Transformer (ViT)** splits the image into fixed-size patches such as 16×16, treats each patch like a "word" as in NLP, and feeds them into a Transformer encoder. The idea itself is simple, but by outperforming CNNs on large-scale data, it has recently become the mainstay of vision tasks.

> **Further reading**
> - [Vaswani et al., "Attention Is All You Need" (2017)](https://arxiv.org/abs/1706.03762) — the original Transformer paper. The start of everything.
> - [Dosovitskiy et al., "An Image is Worth 16x16 Words" (2020)](https://arxiv.org/abs/2010.11929) — the original ViT paper.
> - [Yannic Kilcher — Vision Transformer explanation](https://www.youtube.com/watch?v=TrdevFK_am4) — accessible walk-through of the paper.
> - [Andrej Karpathy — Let's build GPT from scratch](https://www.youtube.com/watch?v=kCc8FmEb1nY) — builds a Transformer from scratch. NLP-focused but directly relevant to understanding ViT.

---

## 10.3 Image Classification

Classification is the most basic question: "what is in this image?" Object detection and segmentation models internally contain a classifier, so understanding classification models is the fundamental of fundamentals. The backbone of pretrained classification models (ResNet, ViT, etc.) is widely used as the feature extractor for other tasks.

**Representative models**:

| Model | Characteristics | Use |
| --- | --- | --- |
| ResNet | Residual connection, stable training | Backbone network |
| EfficientNet | Compound scaling, efficient | Mobile, efficiency-focused |
| ViT | Transformer-based | Large-scale data, high performance |
| ConvNeXt | Modernized CNN | Competes with ViT |

ResNet's residual connection is the simple idea of "adding the input to the output," and this one thing made it possible to train networks dozens of layers deep. Since its 2015 release, it has become a basic building block of almost every deep learning architecture.

**Using a pretrained model**:

```python
import torchvision.models as models

# Pretrained ResNet50
model = models.resnet50(weights='IMAGENET1K_V2')

# Use as a feature extractor
model.fc = nn.Identity()  # remove the last FC
features = model(x)  # (batch, 2048)
```

This pattern is used very often. Remove only the final classification layer of a model pretrained on ImageNet and use the output up to that point as "features." This is called transfer learning, and in robotics it is almost always the approach taken when teaching a system to recognize new objects.

> **Further reading**
> - [He et al., "Deep Residual Learning for Image Recognition" (2015)](https://arxiv.org/abs/1512.03385) — the original ResNet paper. One of the most cited papers in the history of deep learning.
> - [Papers With Code — Image Classification](https://paperswithcode.com/task/image-classification) — check the latest benchmarks and SOTA models.
> - [timm (PyTorch Image Models) library](https://github.com/huggingface/pytorch-image-models) — loads hundreds of pretrained models in a single line. Extremely useful in practice.
> - [Stanford CS231n — Training Neural Networks](https://www.youtube.com/playlist?list=PLoROMvodv4rMFqRtEuo6SGjY4XbRIVRd4) — training techniques and tricks.

---

## 10.4 Object Detection

For a robot to know "where is the cup on that table," it needs not just classification but "where is what." That is object detection — the task of predicting the location and class of objects simultaneously via bounding boxes, used in almost every application: robot manipulation, autonomous driving, and so on.

### 10.4.1 Two-Stage Detectors

**Faster R-CNN**:
1. Region Proposal Network (RPN): proposes candidate regions
2. ROI Pooling: extracts features from each region
3. Classification + Bounding Box Regression

Strength: high accuracy
Weakness: slow speed

Faster R-CNN is the representative two-stage detector and is still used where accuracy matters (e.g., industrial inspection). The structure of "propose candidates first, then analyze them in detail" is intuitive, and it later led to Mask R-CNN and others.

### 10.4.2 One-Stage Detectors

**YOLO (You Only Look Once)**:
- Divides the image into a grid and predicts in one pass
- Real-time processing (30+ FPS)
- Versions: YOLOv5, YOLOv8, YOLOv11 (Ultralytics)

YOLO "only looks once" as its name says — instead of proposing candidates first like two-stage methods, it processes the whole image in one pass and detects every object. When real-time performance matters in a robot system, it is the first model to consider. Ultralytics' YOLOv8/v11 are very simple to install and use, making them well suited for prototyping.

```python
from ultralytics import YOLO

# Load the model and run inference
model = YOLO('yolov8n.pt')  # nano model
results = model('image.jpg')

# Visualize the results
results[0].show()
```

**SSD (Single Shot Detector)**:
- Predicts from feature maps at various scales
- Better at detecting small objects than YOLO

> **Further reading**
> - [Redmon et al., "You Only Look Once: Unified, Real-Time Object Detection" (2016)](https://arxiv.org/abs/1506.02640) — the original YOLO paper. Concise and a good read.
> - [Ultralytics YOLOv8 docs](https://docs.ultralytics.com/) — well organized from installation to custom training.
> - [Papers With Code — Object Detection](https://paperswithcode.com/task/object-detection) — check the latest SOTA.
> - [Dark Programmer — Understanding precision and recall](https://darkpgmr.tistory.com/162) — an intuitive explanation of detection evaluation metrics.

### 10.4.3 Transformer-based

**DETR (Detection Transformer)** redefined detection as a "set prediction problem." A fixed number of learnable vectors called Object Queries correspond to each object, and training is end-to-end without NMS. This contrasts with prior methods, which used a complex pipeline of generating thousands of anchor boxes and removing duplicates with NMS. It had the drawback of slow initial training, but thanks to its clean structure, it spawned many follow-up works such as Deformable DETR, DINO, and Co-DETR.

> **Further reading**
> - [Carion et al., "End-to-End Object Detection with Transformers" (2020)](https://arxiv.org/abs/2005.12872) — the original DETR paper.
> - [Yannic Kilcher — DETR explanation](https://www.youtube.com/watch?v=T35ba_VXkMY) — accessible walk-through of the paper.
> - [HuggingFace — Object Detection guide](https://huggingface.co/docs/transformers/tasks/object_detection) — using DETR through the Transformers library.
> - [Zhao et al., "DETRs Beat YOLOs on Real-time Object Detection" (RT-DETR, CVPR 2024, arXiv:2304.08069)](https://arxiv.org/abs/2304.08069) — first DETR-family model to reach YOLO-level real-time speed. A new direction for real-time detection.
> - [Cheng et al., "YOLO-World: Real-Time Open-Vocabulary Object Detection" (CVPR 2024, arXiv:2401.17270)](https://arxiv.org/abs/2401.17270) — adds text-prompt-based open-vocabulary detection to YOLO. Practical for detecting arbitrary objects in robotics.

---

## 10.5 Semantic Segmentation

This is the task of predicting a class for every pixel.

Robot manipulation makes the difference obvious. Object detection only gives you a rough location via a bounding box, but semantic segmentation gives you the exact boundary of the object. A robot needs the precise contour, not a bounding box, to grasp an object, and for autonomous driving to distinguish road, sidewalk, and lane markings, pixel-level classification is essential.

**Representative models**:

| Model | Characteristics |
| --- | --- |
| FCN | First end-to-end segmentation |
| U-Net | Encoder-Decoder structure, originated in medical imaging |
| DeepLab v3+ | Atrous convolution, multi-scale |
| SegFormer | Transformer-based, lightweight decoder |

U-Net's encoder-decoder plus skip-connection structure has become the default pattern for segmentation. The encoder extracts features while reducing resolution, and the decoder restores the resolution while using skip connections to add back fine detail. This pattern is also widely used in other tasks such as depth estimation and image generation.

```python
# Using a segmentation model (transformers library)
from transformers import SegformerForSemanticSegmentation

model = SegformerForSemanticSegmentation.from_pretrained(
    "nvidia/segformer-b0-finetuned-ade-512-512"
)
```

> **Further reading**
> - [Papers With Code — Semantic Segmentation](https://paperswithcode.com/task/semantic-segmentation) — latest benchmarks.
> - [HuggingFace — Image Segmentation](https://huggingface.co/docs/transformers/tasks/semantic_segmentation) — how to use SegFormer and others.
> - [Two Minute Papers — videos on semantic segmentation](https://www.youtube.com/@TwoMinutePapers) — summarizes recent research in two minutes.

---

## 10.6 Instance & Panoptic Segmentation

**Instance Segmentation**: distinguishes each object instance
- Mask R-CNN: Faster R-CNN + Mask branch

**Panoptic Segmentation**: unifies semantic + instance
- "Things" (objects): instances distinguished
- "Stuff" (background): no instance distinction

Going one step further, semantic segmentation only tells you "this area is chair," not "there are three chairs and here is where each one ends." For a robot to carry out a command like "pick up the chair on the left," instance segmentation is required. Panoptic segmentation unifies the two and is used to understand the entire scene completely.

> **Further reading**
> - [He et al., "Mask R-CNN" (2017)](https://arxiv.org/abs/1703.06870) — the representative work of instance segmentation.
> - [Detectron2](https://github.com/facebookresearch/detectron2) — Meta's detection/segmentation framework. Makes it easy to use Mask R-CNN and related models.

---

## 10.7 Depth Estimation

This is the task of predicting depth from a single image.

If you can obtain depth information from a single monocular camera without a stereo camera or LiDAR, you can greatly cut hardware cost and weight. It is especially useful for systems with limited payload, such as drones or small robots. Recently, with models that show foundation-model-level generalization, practicality has improved considerably.

**Representative models**:
- **MiDaS**: trained on diverse datasets, general-purpose
- **Depth Anything**: foundation-model-level generalization
- **ZoeDepth**: metric depth estimation

```python
# Using Depth Anything
from transformers import pipeline

pipe = pipeline("depth-estimation", model="LiheYoung/depth-anything-base-hf")
result = pipe("image.jpg")
depth = result['depth']
```

A caveat: MiDaS and Depth Anything by default estimate **relative depth**. You can tell "A is closer than B," but you cannot tell "exactly how many meters to A." When metric depth is required, use ZoeDepth or the metric version of Depth Anything V2.

> **Further reading**
> - [Yang et al., "Depth Anything: Unleashing the Power of Large-Scale Unlabeled Data" (2024)](https://arxiv.org/abs/2401.10891) — the original Depth Anything paper.
> - [Godard et al., "Digging Into Self-Supervised Monocular Depth Estimation" (Monodepth2, ICCV 2019, arXiv:1806.01260)](https://arxiv.org/abs/1806.01260) — the baseline for self-supervised depth.
> - [HuggingFace — Monocular Depth Estimation](https://huggingface.co/docs/transformers/tasks/monocular_depth_estimation) — runnable code out of the box.
> - [Papers With Code — Monocular Depth Estimation](https://paperswithcode.com/task/monocular-depth-estimation) — check the latest benchmarks.

---

## 10.8 Advanced: Training Recipes

*If you want to become a researcher, start reading from here.*

Knowing the model architecture seems enough to do research, but in practice "making the training work well" takes more than half of the total research time. Even with the same model, a bad learning rate prevents convergence, and adding one augmentation can bump accuracy by several percent. This section summarizes training techniques used repeatedly in practice.

**Learning Rate Schedule**:

- **Cosine Annealing with Warm-up**: the most widely used schedule. For the first few epochs, the learning rate is ramped linearly from 0 up to the target value (warm-up), then decayed along a cosine curve.

$$\eta_t = \eta_{min} + \frac{1}{2}(\eta_{max} - \eta_{min})\left(1 + \cos\left(\frac{t \cdot \pi}{T}\right)\right)$$

- **OneCycleLR**: a policy that raises the learning rate once and then lowers it. It can achieve super-convergence and converges quickly in few epochs.

```python
import torch.optim as optim

# Cosine Annealing with Warm-up (manual)
optimizer = optim.AdamW(model.parameters(), lr=1e-4, weight_decay=0.05)
scheduler = optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=100)

# OneCycleLR
scheduler = optim.lr_scheduler.OneCycleLR(
    optimizer, max_lr=1e-3, total_steps=len(dataloader) * num_epochs,
    pct_start=0.1  # use the first 10% for warm-up
)
```

**Data Augmentation**:

| Technique | Description | Main use |
|------|------|---------|
| **RandAugment** | applies N transformations at magnitude M at random | general classification |
| **CutMix** | replaces an image region with another image and mixes the labels proportionally | classification |
| **MixUp** | linearly interpolates two images and their labels | classification |
| **Mosaic** | composes 4 images into one | detection (YOLO family) |

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

- **Label Smoothing**: use soft labels (e.g., 0.1, 0.9) instead of hard labels (0 or 1). Prevents overconfidence. `nn.CrossEntropyLoss(label_smoothing=0.1)`
- **Stochastic Depth**: randomly skip some layers during training. Effective at preventing overfitting in ResNet-family models.
- **Weight Decay**: set `weight_decay=0.01~0.05` in the optimizer. With AdamW, use decoupled weight decay.

**Gradient Clipping**: prevents gradients from exploding. Almost mandatory in Transformer training.

```python
torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
```

**Diagnosing Problems from the Loss Curve**:

| Pattern | Diagnosis | Response |
|------|------|------|
| train loss decreasing, val loss increasing | Overfitting | add augmentation, increase dropout/weight decay, get more data |
| train loss stuck at a high value | Underfitting | increase model size, adjust learning rate, reduce augmentation |
| train loss oscillates heavily | Learning rate too high | decrease learning rate |
| train loss becomes NaN | Gradient explosion | gradient clipping, sharply reduce learning rate, validate data |
| val loss drops early then completely plateaus | Learning rate too low or schedule issue | add warm-up, apply cosine schedule |

**Distributed Training — PyTorch DDP basics**:

When the model gets large, a single GPU runs out of time. DistributedDataParallel (DDP) is the most basic parallel training method; it replicates the model across multiple GPUs and synchronizes gradients.

```python
# Minimal DDP structure (launch with torchrun)
import torch.distributed as dist
from torch.nn.parallel import DistributedDataParallel as DDP

dist.init_process_group("nccl")
local_rank = int(os.environ["LOCAL_RANK"])
model = model.to(local_rank)
model = DDP(model, device_ids=[local_rank])

# Launch: torchrun --nproc_per_node=4 train.py
```

> **Further reading**
> - [Goyal et al., "Accurate, Large Minibatch SGD" (2017)](https://arxiv.org/abs/1706.02677) — the learning rate scaling rule for large-scale training.
> - [PyTorch DDP Tutorial](https://pytorch.org/tutorials/intermediate/ddp_tutorial.html) — the official guide to distributed training.
> - [Wightman et al., "ResNet strikes back" (2021)](https://arxiv.org/abs/2110.00476) — a paper that shows the importance of training recipes. Using the same ResNet, changing only the training techniques improves accuracy substantially.

> **Exercise**: [Data Augmentation visualization](https://alexjunholee.github.io/robotics-practice/app.html#data_augmentation)
> Interactively see how various augmentation techniques such as RandAugment, CutMix, and MixUp transform images.

> **Exercise**: [Learning Rate Schedule visualization](https://alexjunholee.github.io/robotics-practice/app.html#lr_schedule)
> Compare curves of various learning rate schedules such as Cosine Annealing and OneCycleLR, and see the effect of hyperparameters.

---

## 10.9 Advanced: Self-Supervised and Contrastive Learning

*If you want to become a researcher, start reading from here.*

Robotics data is label-scarce. A robot collects thousands or tens of thousands of images, but labeling each of them with bounding boxes or segmentation masks is impractical. Self-supervised learning creates a training signal from the data itself without labels.

**Contrastive Learning**:

The core idea is simple: place different augmentations of the same image close together in the embedding space (positive pair) and different images far apart (negative pair).

- **SimCLR**: apply different augmentations to the same image to form positive pairs. Other images in the batch form the negative pairs. Requires a large batch size.
- **MoCo (Momentum Contrast)**: uses a momentum encoder and a queue to secure many negatives without needing a large batch.

**InfoNCE Loss**:

$$\mathcal{L} = -\log \frac{\exp(\text{sim}(z_i, z_j) / \tau)}{\sum_{k=1}^{2N} \mathbb{1}_{[k \neq i]} \exp(\text{sim}(z_i, z_k) / \tau)}$$

Here, sim is cosine similarity and τ is the temperature. The numerator raises the similarity of positive pairs, while the denominator trains the model to distinguish them from negative pairs.

**Masked Image Modeling — MAE**:

Based on ViT, this approach randomly masks 75% of image patches and reconstructs the masked portions from the remaining 25%. It follows the same principle as BERT in NLP masking and recovering words.

- Why the masking ratio is as high as 75%: images have much more redundancy than text, so a high masking ratio makes the task harder and forces better representations.
- Since the encoder only processes the visible patches, training is efficient (75% reduction in compute).

**Connection to DINOv2**:

DINOv2 is trained via self-distillation. It uses a teacher-student structure, but the teacher is the EMA (exponential moving average) of the student.

- **Self-distillation**: student and teacher share the same architecture. The teacher's weights are an EMA of the student's weights.
- **Centering + Sharpening**: centering (subtracting the mean) and sharpening (low temperature) are applied to the teacher output to prevent mode collapse.
- The resulting DINOv2 features achieve performance comparable to supervised methods without additional training — e.g., 83.0% ImageNet k-NN and 82.0% ADE20K linear probe.

**Practice — Fine-tuning a self-supervised backbone on HuggingFace**:

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
> - [Chen et al., "A Simple Framework for Contrastive Learning of Visual Representations (SimCLR)" (2020)](https://arxiv.org/abs/2002.05709) — the representative work of contrastive learning.
> - [He et al., "Masked Autoencoders Are Scalable Vision Learners" (2022)](https://arxiv.org/abs/2111.06377) — the original MAE paper.
> - [Oquab et al., "DINOv2: Learning Robust Visual Features without Supervision" (2024)](https://arxiv.org/abs/2304.07193) — the original DINOv2 paper.

---

## 10.10 Advanced: Knowledge Distillation

*If you want to become a researcher, start reading from here.*

This is the technique of transferring the "knowledge" of a large model (teacher) to a small model (student). It is especially important in robotics because giant models such as VFMs must run on edge devices. To run SAM in real time on a Jetson, distillation is almost the only way.

**Teacher-Student Structure**:

The student model (small model) is trained to imitate the output of the teacher model (large model, already trained). Crucially, the teacher's soft prediction carries more information than a hard label (ground truth).

For example, for an image of "cat" the hard label is [1, 0, 0], but the teacher's soft prediction might be [0.85, 0.10, 0.05]. This soft prediction contains the information that "cat and dog are somewhat similar," and the student learns that too.

**Soft Targets and Temperature Scaling**:

$$\mathcal{L}_{KD} = \text{KL}\left(\sigma\left(\frac{z_t}{\tau}\right) \| \sigma\left(\frac{z_s}{\tau}\right)\right)$$

Here z_t and z_s are the teacher and student logits, respectively, and τ is the temperature. When τ > 1, the probability distribution becomes "softer," so more information about inter-class relationships is conveyed. Typically τ = 3~5 is used.

The total loss is a weighted sum of the hard label loss and the distillation loss:

$$\mathcal{L} = \alpha \cdot \mathcal{L}_{CE}(y, \sigma(z_s)) + (1 - \alpha) \cdot \tau^2 \cdot \mathcal{L}_{KD}$$

The reason for multiplying by τ^2: it compensates for the fact that gradient magnitudes shrink by 1/τ^2 due to temperature scaling.

**Feature-based Distillation (FitNets)**:

Not only logits, but also the intermediate layer feature maps are made similar to the teacher's.

$$\mathcal{L}_{feat} = \|f_t(x) - r(f_s(x))\|^2$$

Here r is a projection layer that matches the student feature dimension to the teacher's. This trains intermediate representations that are hard to transfer with logit distillation alone.

**Applications to VFM lightweighting**:

| Teacher | Student | Method |
|---------|---------|------|
| SAM (ViT-H) | MobileSAM | Replace the image encoder with a lightweight ViT, distillation |
| SAM (ViT-H) | FastSAM | Replace the entire pipeline with a YOLO architecture |
| DINOv2-giant | DINOv2-small | Distill into a smaller version of the same architecture |

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
> - [Hinton et al., "Distilling the Knowledge in a Neural Network" (2015)](https://arxiv.org/abs/1503.02531) — the original knowledge distillation paper.
> - [Zhang et al., "Faster Segment Anything (MobileSAM)" (2023)](https://arxiv.org/abs/2306.14289) — a case of SAM distillation.
> - [Romero et al., "FitNets: Hints for Thin Deep Nets" (2015)](https://arxiv.org/abs/1412.6550) — the original feature-based distillation paper.

---

## 10.11 Advanced: Domain Adaptation

*If you want to become a researcher, start reading from here.*

When a model trained in simulation is deployed to a real robot, performance drops sharply. The same goes for training on indoor data and deploying outdoors. This problem is called **domain shift**, and the research that addresses it is domain adaptation. In robotics, it is directly tied to the sim-to-real gap problem.

**Problem Setup**:
- Source domain D_s (labeled): simulation data or an existing dataset
- Target domain D_t (unlabeled or small): the real deployment environment
- Goal: make a model trained on D_s also work well on D_t.

**Domain Randomization**:

The simplest yet effective approach. When generating training data in the simulator, randomize environment parameters to the extreme.

- Texture: randomly change the textures of walls, floor, and objects every episode
- Lighting: randomize position, color, and intensity
- Camera parameters: add noise to focal length, position, and angle
- Physical parameters: randomly set friction coefficient, mass, inertia, etc. within a range

The idea is that after seeing a sufficiently diverse set of simulated environments, the real environment can be treated as "just another variant."

**Adversarial Domain Adaptation**:

Introduces a domain discriminator so that the feature extractor learns domain-invariant features that the discriminator cannot distinguish between source and target.

```
Input --> Feature Extractor --> [Task Classifier]      --> Task Loss
                              \--> [Domain Discriminator] --> Domain Loss (GRL)
```

- Gradient Reversal Layer (GRL): reverses the gradient from the domain discriminator so that the feature extractor trains in the direction of not being able to distinguish domains.
- The task classifier trains normally on the source domain.
- The feature extractor learns representations that are useful for the task yet invariant to the domain.

$$\mathcal{L} = \mathcal{L}_{task}(D_s) - \lambda \cdot \mathcal{L}_{domain}(D_s, D_t)$$

The minus sign matters. The feature extractor is trained in the direction of "maximizing" the domain loss (adversarial training analogous to GANs).

**Test-Time Adaptation (TTA)**:

A way for the model to adapt to new environments even after deployment. Without accessing the training data, it adjusts the model using only the data that arrives at inference time.

- **TENT**: adjusts the affine parameters of batch normalization via entropy minimization.
- **CoTTA**: continual TTA. Adapts even when the distribution changes over time.

```python
# Core idea of TENT (simplified)
model.eval()
# Make only the BN affine parameters trainable
for m in model.modules():
    if isinstance(m, nn.BatchNorm2d):
        m.requires_grad_(True)
        m.track_running_stats = False  # use batch statistics

optimizer = optim.SGD(model.parameters(), lr=1e-4)

# Adaptation at inference time
for batch in test_loader:
    output = model(batch)
    loss = entropy(output)  # minimize prediction entropy
    loss.backward()
    optimizer.step()
    optimizer.zero_grad()
```

**Connection to the sim-to-real gap**:

In real-world robotics these techniques are combined:
1. Generate diverse data in the simulator with domain randomization.
2. Perform adversarial adaptation with a small amount of real-environment data.
3. After deployment, continually adapt to environmental changes via TTA.

This combination is currently the most widely used approach to sim-to-real transfer.

> **Further reading**
> - [Tobin et al., "Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World" (2017)](https://arxiv.org/abs/1703.06907) — the original domain randomization paper.
> - [Ganin et al., "Domain-Adversarial Training of Neural Networks" (2016)](https://arxiv.org/abs/1505.07818) — the original adversarial domain adaptation paper (proposes GRL).
> - [Wang et al., "TENT: Fully Test-Time Adaptation by Entropy Minimization" (2021)](https://arxiv.org/abs/2006.10726) — the representative TTA work.
> - [Wen et al., "FoundationPose: Unified 6D Pose Estimation and Tracking of Novel Objects" (CVPR 2024, arXiv:2312.08344)](https://arxiv.org/abs/2312.08344) — 6D pose estimation for novel objects. Operates from a CAD model or a few reference images.

---

> **Technical Timeline: Deep Learning for Perception**
> - **2012**: AlexNet wins the ImageNet competition by a large margin over prior methods. The start of the "deep learning revolution." The era of hand-crafted features begins to end.
> - **2014~2015**: VGGNet, GoogLeNet, and ResNet appear. In particular, ResNet's (2015) residual connection makes it possible to train networks hundreds of layers deep. During this period, Faster R-CNN (2015) and YOLO (2016) make real-time object detection possible.
> - **2017**: "Attention Is All You Need" — Transformer is announced. Originally for NLP, but later extended to vision.
> - **2020~2021**: ViT (Vision Transformer) appears. A new paradigm of processing images as patch sequences. DETR applies Transformer to detection as well. Swin Transformer achieves SOTA on various vision tasks.
> - **2022~**: ConvNeXt shows that "CNNs are not dead yet." Segment Anything (SAM) elevates segmentation to a foundation model. Extension to Spatial AI — depth estimation and 3D scene understanding become the next battleground for deep learning.
> - **What to watch now**: the shift from single-task models to foundation models. Pipelines that use a single DINOv2 to extract detection, segmentation, and depth features are beginning to appear experimentally.
