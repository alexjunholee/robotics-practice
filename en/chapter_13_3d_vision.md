# Ch.13 — 3D Vision


With only 2D images, a robot has trouble answering "how far is that object?" or "what is behind that wall?" 3D vision is the field that gives a robot spatial awareness. Point cloud processing, 3D object detection, and scene reconstruction fall under it. Neither SLAM nor robot manipulation is fully understandable without this material. It is the foundation of the next chapter (SLAM), so study it carefully.

## 13.1 Point Cloud Basics

The data coming out of a LiDAR or a depth camera is a point cloud. An image is 2D data aligned on a pixel grid, whereas a point cloud is a set of points irregularly scattered in 3D space. How to handle this unstructured data is the starting point of 3D vision.

A **point cloud** is a set of points in 3D space. Each point has at least (x, y, z) coordinates, and may additionally carry attributes such as color (RGB), reflectance (intensity), or a normal.

### 13.1.1 Data Structures and Formats

**Typical structure**:

```
Point: [x, y, z, r, g, b, intensity, ...]
Point Cloud: N × D matrix (N points, D-dimensional attributes)
```

In linear-algebra terms, a point cloud is just an N×D matrix. N ranges from tens of thousands to millions of points, and D is the attribute dimension per point. A transformation (rotation, translation) amounts to multiplying each point by a 4×4 transformation matrix.

**Major file formats**:
| Format | Characteristics |
|---|---|
| **PCD** | PCL standard, ASCII/Binary |
| **PLY** | General-purpose, also supports meshes |
| **LAS/LAZ** | Geospatial standard, LAZ is compressed |
| **XYZ** | Simple text, coordinates only |
| **BIN** | Used by KITTI and others, binary |

> **Further reading**
> - [Open3D Documentation](http://www.open3d.org/docs/release/) — modern library for point cloud processing.
> - [PCL (Point Cloud Library) Tutorials](https://pcl.readthedocs.io/projects/tutorials/en/latest/) — classic library for point cloud processing.

### 13.1.2 Libraries

**PCL (Point Cloud Library)**:
- C++ based, the most comprehensive
- Integrated with ROS
- Filtering, segmentation, registration, and more

```cpp
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::io::loadPCDFile<pcl::PointXYZ>("cloud.pcd", *cloud);
```

**Open3D**:
- Python/C++, modern API
- Strong on visualization
- Deep-learning friendly

```python
import open3d as o3d

# Load point cloud
pcd = o3d.io.read_point_cloud("cloud.pcd")

# Visualize
o3d.visualization.draw_geometries([pcd])

# Convert to NumPy
points = np.asarray(pcd.points)  # (N, 3)
```

In practice, Open3D is good for prototyping because you can use it directly from Python, while PCL is mostly used when building C++ ROS nodes. If you are just starting out, I recommend starting with Open3D.

> **Further reading**
> - [Open3D Getting Started](http://www.open3d.org/docs/release/getting_started.html) — introduction to handling point clouds in Python.
> - [PCL Tutorials — Basic Usage](https://pcl.readthedocs.io/projects/tutorials/en/latest/#basic-usage) — C++-based point cloud processing.
> - [Open3D YouTube Channel](https://www.youtube.com/@Open3D) — visualization and processing tutorials.

## 13.2 Point Cloud Processing

### 13.2.1 Filtering

Raw point clouds are noisy and have uneven point density. Using them as-is slows down downstream algorithms (registration, segmentation, and so on) or degrades their results. Filtering is the first stage of every point cloud pipeline.

**Voxel grid downsampling**:
Partition the space into a grid and collapse the points in each cell into one.

```python
# Open3D
voxel_pcd = pcd.voxel_down_sample(voxel_size=0.05)  # 5cm grid
```

**Statistical outlier removal**:
Remove outliers based on distance statistics to neighbors.

```python
# Outlier removal
cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
filtered_pcd = pcd.select_by_index(ind)
```

**Radius outlier removal**:
Remove points that have too few neighbors within a given radius.

> **Further reading**
> - [Open3D — Point Cloud Filtering Tutorial](http://www.open3d.org/docs/release/tutorial/geometry/pointcloud.html) — examples of voxel downsampling and outlier removal.
> - [PCL — Filtering Tutorial](https://pcl.readthedocs.io/projects/tutorials/en/latest/passthrough.html) — PCL-based filtering.

### 13.2.2 Normal Estimation

Estimate the surface normal vector at each point. This is a preprocessing step for many algorithms.

ICP (registration), surface reconstruction, lighting computation — nearly every 3D processing step requires normals. Without normals, there is no way to tell "whether a point is part of a plane or part of an edge."

```python
# Normal estimation
pcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
)
```

Internally, for each point you gather its k nearest neighbors, compute the covariance matrix, and take the eigenvector corresponding to the smallest eigenvalue as the normal. This is exactly the same principle as PCA (Principal Component Analysis) from linear algebra class.

### 13.2.3 Registration

The process of aligning two point clouds. SLAM needs it to stitch consecutive frames together, or to merge scans captured from multiple viewpoints.

**ICP (Iterative Closest Point)**:
1. Find the closest point pairs
2. Compute the transformation (least squares)
3. Apply the transformation
4. Repeat until convergence

```python
# Point-to-Point ICP
reg = o3d.pipelines.registration.registration_icp(
    source, target, max_correspondence_distance=0.05,
    estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint()
)
transformation = reg.transformation
```

The intuition behind ICP: "find the closest point pairs between the two point clouds, then solve for the rotation + translation that makes those pairs overlap as much as possible. One pass is not perfect, so iterate." In linear-algebra terms, you use SVD (singular value decomposition) to solve for the optimal rotation matrix R and translation vector t.

**Point-to-Plane ICP**: minimizes point-to-plane distance (more accurate).

**GICP (Generalized ICP)**: accounts for point distributions.

**NDT (Normal Distributions Transform)**: partitions space into cells and matches the normal distribution of each cell.

**Feature-based registration**:
- Extract features such as FPFH or SHOT
- Initial registration via RANSAC
- Refine with ICP

> **Further reading**
> - [Open3D — ICP Registration Tutorial](http://www.open3d.org/docs/release/tutorial/pipelines/icp_registration.html) — hands-on ICP code.
> - [Open3D — Global Registration (RANSAC + Feature)](http://www.open3d.org/docs/release/tutorial/pipelines/global_registration.html) — feature-based registration.
> - [Cyrill Stachniss — ICP & Point Cloud Registration](https://www.youtube.com/watch?v=dhzLQfDBx2Q) — intuitive explanation of the ICP algorithm.

> **Exercise**: [Step-by-step 2D ICP visualization](https://alexjunholee.github.io/robotics-practice/app.html#icp_steps) | [3D ICP](https://alexjunholee.github.io/robotics-practice/app.html#icp_3d)
> Inspect ICP registering two point clouds iteration by iteration, and compare convergence in 2D and 3D environments.

## 13.3 3D Object Detection

Predict 3D bounding boxes from a point cloud. In autonomous driving, this is the core technology for knowing "where that car is and how big it is."

### 13.3.1 Point-based Methods

The core idea of PointNet is to apply deep learning **directly to raw points** without converting the point cloud to voxels or images. Previously there was no answer to "how do we apply a CNN to irregular points?" — PointNet solved this.

**PointNet (2017)**:
- Applied directly to raw points
- Permutation invariant (independent of point order)
- Global feature via max pooling

**PointNet++ (2017)**:
- Hierarchical feature learning
- Set Abstraction: extracts features per region
- Can learn local patterns

```python
# PointNet++ conceptual structure
# 1. Sampling: pick centers via FPS
# 2. Grouping: gather neighbors via ball query
# 3. PointNet: extract features in each group
```

PointNet's core idea: the result must be the same even if the point order changes (permutation invariance). To achieve this, each point is passed independently through an MLP and then aggregated via max pooling. Mathematically, it takes the form f({x1, ..., xn}) = g(MAX(h(x1), ..., h(xn))).

### 13.3.2 Voxel-based Methods

**VoxelNet (2018)**:
- Converts the point cloud into 3D voxels
- Voxel Feature Encoding
- Processed by a 3D CNN

**SECOND (Sparsely Embedded Convolutional Detection)**:
- Uses sparse convolution
- Much faster than VoxelNet
- A widely used baseline

**PointPillars (2019)**:
- Processes data in pillars (vertical columns)
- Converts to a 2D CNN for high speed
- Real-time capable

PointPillars' core idea: if you divide 3D space into vertical pillars, you can compress the points inside each pillar into a single feature vector and arrange them like a 2D image. You can then reuse well-validated 2D CNNs as-is, which is much faster than a 3D CNN.

### 13.3.3 Multi-modal Methods

Combining multiple sensors lets each one cover the others' weaknesses. Cameras are rich in color and texture but lack depth, while LiDAR has accurate 3D information but no texture. How to fuse the two is the key question.

**BEVFusion**:
- Camera + LiDAR fusion
- Integrated in Bird's Eye View (BEV) space

**TransFusion**:
- Transformer-based fusion
- Query-based detection

> **Further reading**
> - [Qi et al., "PointNet: Deep Learning on Point Sets" (2017)](https://arxiv.org/abs/1612.00593) — the starting point of 3D deep learning.
> - [Lang et al., "PointPillars" (2019)](https://arxiv.org/abs/1812.05784) — real-time 3D detection.
> - [Liu et al., "BEVFusion" (2023)](https://arxiv.org/abs/2205.13542) — a landmark work in multi-modal fusion.
> - [MMDetection3D GitHub](https://github.com/open-mmlab/mmdetection3d) — unified 3D object detection framework.

> **Exercise**: [BEV Projection Visualization](https://alexjunholee.github.io/robotics-practice/app.html#bev_projection)
> Interactively inspect the process of converting a camera image into BEV, and grasp the principle of BEV-based 3D detection.

## 13.4 3D Reconstruction

Generate a 3D model from multiple views or depth information. A robot needs this technology to "remember" the environment in 3D.

### 13.4.1 Structure from Motion (SfM)

You can recover 3D structure from just a handful of 2D photographs. A few smartphone photos can yield a 3D model of a building. It is also the preprocessing step that produces the input data (camera poses) for NeRF or 3D Gaussian Splatting (3DGS).

Recover camera poses and 3D structure simultaneously from multiple images.

**Pipeline**:
1. Feature extraction and matching
2. Triangulation from an initial two views
3. Incremental addition of cameras
4. Bundle adjustment (BA)

Bundle adjustment "jointly optimizes all camera poses and 3D point positions." It uses nonlinear least squares (Levenberg-Marquardt and the like), and the number of variables can reach tens of thousands to hundreds of thousands. Think of it as a large-scale nonlinear extension of the least squares you learned in linear algebra.

**Tools**:
- **COLMAP**: the most widely used, GUI/CLI
- **OpenMVG**: library-style

```bash
# Using COLMAP
colmap feature_extractor --database_path db.db --image_path ./images
colmap exhaustive_matcher --database_path db.db
colmap mapper --database_path db.db --image_path ./images --output_path ./sparse
```

> **Further reading**
> - [COLMAP Documentation](https://colmap.github.io/) — the de facto standard tool for SfM/MVS.
> - [Daniel Cremers — Multiple View Geometry (TUM)](https://www.youtube.com/playlist?list=PLTBdjV_4f-EJn6udZ34tht9EVIW7lbeo4) — core lectures on multiple view geometry.
> - [Schönberger & Frahm, "Structure-from-Motion Revisited" (2016)](https://openaccess.thecvf.com/content_cvpr_2016/papers/Schonberger_Structure-From-Motion_Revisited_CVPR_2016_paper.pdf) — the COLMAP paper.

### 13.4.2 Multi-View Stereo (MVS)

Generate a dense point cloud from the SfM result.

Where SfM recovers "where the cameras were" and "sparse 3D points," MVS uses those camera poses to build a **dense** 3D point cloud. SfM → MVS → mesh generation is the canonical 3D reconstruction pipeline.

**Tools**: COLMAP (dense reconstruction), OpenMVS

### 13.4.3 Volumetric Reconstruction

**TSDF (Truncated Signed Distance Function)**:
- Partition space into voxels and store the distance to the surface in each voxel
- Integrate multiple views
- Extract a mesh via Marching Cubes

TSDF's core idea: each voxel stores "the signed distance to the nearest surface." Positive means outside the surface, negative means inside. Taking a weighted average of depths observed from multiple views reduces noise and yields a clean surface. The location where the sign flips (passes through zero) is the surface.

```python
# Open3D TSDF Integration
volume = o3d.pipelines.integration.ScalableTSDFVolume(
    voxel_length=0.01,
    sdf_trunc=0.04,
    color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8
)

for i, (color, depth, pose) in enumerate(frames):
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth)
    volume.integrate(rgbd, intrinsic, np.linalg.inv(pose))

mesh = volume.extract_triangle_mesh()
```

> **Further reading**
> - [Open3D — TSDF Integration Tutorial](http://www.open3d.org/docs/release/tutorial/pipelines/rgbd_integration.html) — hands-on TSDF code.
> - [Curless & Levoy, "A Volumetric Method for Building Complex Models from Range Images" (1996)](https://graphics.stanford.edu/papers/volrange/volrange.pdf) — the original TSDF paper (a classic, but still worth reading).

## 13.5 Neural Rendering

A new deep-learning-based approach to 3D representation and rendering. Prior methods (mesh, point cloud) had limits in representing complex scenes (reflections, transparent objects, thin structures). Neural rendering represents a scene as a learnable function, handling such effects naturally. Recently it has also been combined with SLAM for real-time mapping.

### 13.5.1 NeRF (Neural Radiance Fields)

**Concept**: represent a 3D scene as a continuous function.

```
F: (x, y, z, θ, φ) → (r, g, b, σ)
- Position (x, y, z) and viewing direction (θ, φ)
- Outputs color (r, g, b) and density (σ)
```

Intuitively: NeRF learns, via a neural network, "for every point in 3D space, what color and density it has when viewed from a given direction." Once training is done, you can synthesize novel views from arbitrary camera positions (novel view synthesis).

**Rendering**: integrate color and density along a ray (volume rendering).

**Pros**:
- Photorealistic novel view synthesis
- Handles complex effects such as reflection and transparency

**Cons**:
- Training takes a long time
- Dynamic scenes are hard

**Extensions**:
- Instant-NGP: fast training via hash encoding (minutes)
- Mip-NeRF: anti-aliasing
- Block-NeRF: large-scale scenes

> **Further reading**
> - [Mildenhall et al., "NeRF: Representing Scenes as Neural Radiance Fields" (2020)](https://arxiv.org/abs/2003.08934) — the original NeRF paper.
> - [NeRFStudio Documentation](https://docs.nerf.studio/) — a unified framework that makes NeRF experiments easy. Start here if you want to run NeRF yourself.
> - [Yannic Kilcher — NeRF Explained](https://www.youtube.com/watch?v=CRlN-cYFxTk) — an intuitive explanation of NeRF's core idea.
> - [Jon Barron — Understanding NeRF (ECCV 2022 Tutorial)](https://www.youtube.com/watch?v=HfJpQCBTqZs) — from a NeRF author.

### 13.5.2 3D Gaussian Splatting (3DGS)

3DGS was adopted quickly because it fixed NeRF's fatal weakness, slow rendering. NeRF takes seconds to render a single frame, whereas 3DGS renders in real time at 100+ FPS. This speed makes it directly usable in robot applications such as SLAM and real-time mapping.

**Concept**: represent a scene with millions of 3D Gaussians.

Each Gaussian has:
- Position (mean)
- Covariance (shape/size/orientation)
- Color (spherical harmonics)
- Opacity

Recall the covariance matrix from linear algebra. The eigenvectors of a 3×3 covariance matrix determine an ellipsoid's axis directions, and the eigenvalues determine the axis lengths. 3DGS uses this idea directly to represent each Gaussian's shape and size.

**Rendering**: project Gaussians onto the image (splatting).

**Pros**:
- **Real-time rendering** (100+ FPS) compared with NeRF
- Fast training (minutes)
- Explicit representation (easy to edit)

**Applications**:
- SLAM: SplaTAM, Gaussian Splatting SLAM
- Mapping: large-scale environment representation
- Dynamic scenes: extensions to dynamic scenes

```python
# 3DGS basic idea (pseudo-code)
# Each Gaussian: position, covariance, color, opacity
# Rendering: project to camera view to generate the image
```

**3DGS + SLAM (current trend)**:

As 3D Gaussian Splatting merges with SLAM, a new direction for neural SLAM is emerging. Whereas traditional SLAM built sparse point maps or voxel maps, 3DGS-SLAM builds photorealistic 3D maps in real time.

- **SplaTAM (2024)**: runs 3DGS-based dense SLAM from RGB-D camera input. It alternates between tracking (camera pose estimation) and mapping (adding/updating Gaussians), and greatly improves both rendering quality and speed compared with prior neural SLAM.
- **MonoGS (2024)**: runs 3DGS-based SLAM using only a monocular camera. It is drawing attention because it can build a dense 3D map without a depth sensor.
- **Gaussian-SLAM (2024)**: runs 3DGS SLAM at large scale via a sub-map approach.

If a robot can build photorealistic 3D maps in real time while moving around, applications such as AR/VR content creation, digital twins, and building inspection open up.

> **Further reading**
> - [Kerbl et al., "3D Gaussian Splatting for Real-Time Radiance Field Rendering" (2023)](https://arxiv.org/abs/2308.14737) — the original 3DGS paper.
> - [Huang et al., "2D Gaussian Splatting for Geometrically Accurate Radiance Fields" (SIGGRAPH 2024, arXiv:2403.17888)](https://arxiv.org/abs/2403.17888) — improves surface reconstruction quality via 2D Gaussians.
> - [Keetha et al., "SplaTAM: Splat, Track & Map 3D Gaussians for Dense RGB-D SLAM" (2024)](https://arxiv.org/abs/2312.02126) — a landmark for 3DGS + SLAM.
> - [Matsuki et al., "Gaussian Splatting SLAM" (2024)](https://arxiv.org/abs/2312.06741) — the MonoGS paper.
> - [Wang et al., "DUSt3R: Geometric 3D Vision Made Easy" (CVPR 2024, arXiv:2312.14132)](https://arxiv.org/abs/2312.14132) — dense 3D reconstruction from image pairs without camera intrinsics/extrinsics. A paradigm shift for 3D reconstruction.
> - [Leroy et al., "MASt3R: Matching And Stereo 3D Reconstruction" (ECCV 2024, arXiv:2406.09756)](https://arxiv.org/abs/2406.09756) — adds local feature matching to DUSt3R. Provides reconstruction and precise correspondences simultaneously.
> - [NeRFStudio Documentation](https://docs.nerf.studio/) — unified framework for NeRF/3DGS experiments.
> - [3DGS Original Implementation (GitHub)](https://github.com/graphdeco-inria/gaussian-splatting) — the official code.

> **Exercise**: [3D Gaussian Splatting Visualization](https://alexjunholee.github.io/robotics-practice/app.html#gaussian_splatting)
> Manipulate the position, covariance, and color of 3D Gaussians to interactively understand the splatting rendering process.

## 13.6 Advanced: Neural Implicit Representations

*If you want to become a researcher, read from here.*

Section 13.5 covered NeRF and 3DGS. NeRF uses a density field to perform volume rendering, but extracting a clear surface from the density is hard. For robotics — grasping objects or checking collisions — an accurate surface is required. This is where signed-distance-function (SDF) based approaches come in.

**SDF (Signed Distance Function)**

A function that returns, at each point `x` in space, the signed distance to the nearest surface.

```
f(x) > 0  : outside the surface
f(x) < 0  : inside the surface
f(x) = 0  : on the surface (zero level set)
```

The key property of an SDF: its gradient has unit magnitude everywhere (Eikonal equation).

```
||∇f(x)|| = 1
```

Only functions satisfying this condition are proper distance functions. When learning an SDF with a neural network, this condition is added as a regularization term, called the **Eikonal loss**.

```
L_eikonal = E_x[ (||∇f_θ(x)|| - 1)^2 ]
```

**DeepSDF**

An early landmark work on learning SDFs with neural networks. It uses a decoder-only architecture and represents each object's shape with a latent code `z`.

```
f_θ(z, x) → SDF value
```

For a new object, `z` is estimated via test-time optimization.

**NeuS**

Combines the rendering quality of NeRF's volume rendering with the clean surfaces of SDFs. It introduces a function that converts SDF values into densities, so SDFs can be learned within the volume rendering framework.

```
density σ(x) = max(-dΦ_s(f(x))/dt, 0) / Φ_s(f(x))
```

Here `Φ_s` is a sigmoid-like function controlled by a learnable parameter `s`. As training progresses, `s` shrinks and density concentrates near the surface.

**VolSDF**

A similar approach, but defines density as the CDF of a Laplace distribution of the SDF.

```
σ(x) = (1/β) · Ψ_β(-f(x))
```

As `β` decreases, density concentrates on the surface.

**Surface extraction**

The standard method for converting the iso-surface `f(x) = 0` of a learned SDF into a mesh is the **Marching Cubes** algorithm. It partitions space into a grid, checks the SDF sign at each grid vertex, and interpolates to determine where the surface crosses.

**Comparison table**

| Representation | Pros | Cons | Examples |
|------|------|------|------|
| NeRF (density) | High rendering quality | Surface extraction is hard | Instant-NGP |
| SDF (neural) | Clean surfaces | Hard to train | NeuS, VolSDF |
| 3DGS (explicit) | Real-time rendering | High memory usage | Gaussian Splatting |
| Occupancy | Simple via binary classification | Limited surface detail | ConvONet |

> **Further reading**
> - [Wang et al., "NeuS: Learning Neural Implicit Surfaces by Volume Rendering" (NeurIPS 2021)](https://arxiv.org/abs/2106.10689) — the original NeuS paper.
> - [Yariv et al., "Volume Rendering of Neural Implicit Surfaces" (NeurIPS 2021)](https://arxiv.org/abs/2106.12052) — the VolSDF paper.
> - [Park et al., "DeepSDF: Learning Continuous Signed Distance Functions for Shape Representation" (CVPR 2019)](https://arxiv.org/abs/1901.05103) — the original DeepSDF paper.
> - [Mescheder et al., "Occupancy Networks" (CVPR 2019)](https://arxiv.org/abs/1812.03828) — a landmark for occupancy-based approaches.

## 13.7 Advanced: Differentiable Rendering

*If you want to become a researcher, read from here.*

NeRF, 3DGS, NeuS, and other recent core 3D vision techniques share one principle: **make the rendering process differentiable, so the difference between the rendered result and the real image optimizes the 3D representation**. This paradigm is called analysis-by-synthesis.

**Volume rendering equation**

The basic rendering formula used by the NeRF family. It integrates color along a ray `r(t) = o + td` cast from the camera.

```
C(r) = ∫ T(t) · σ(t) · c(t) dt

where T(t) = exp( -∫_{t_n}^{t} σ(s) ds )
```

- `σ(t)`: density (opacity) at location `t`
- `c(t)`: color (RGB) at location `t`
- `T(t)`: accumulated transmittance (the probability that the ray reaches `t`)

In practice, this continuous integral is discretized and approximated at N samples along the ray (ray marching).

```
C(r) ≈ Σ_i T_i · α_i · c_i
where α_i = 1 - exp(-σ_i · δ_i),  T_i = Π_{j<i} (1 - α_j)
```

**Rasterization-based rendering in 3DGS**

Instead of the ray marching above, 3DGS uses a rasterization approach that projects each Gaussian onto the image plane (splatting). Each pixel's color is the weighted sum of the Gaussians that influence that pixel.

```
C(p) = Σ_i c_i · α_i · Π_{j<i} (1 - α_j)
```

The equation is structurally similar to volume rendering, but instead of sampling along a ray, Gaussians are sorted by depth and composited in order. This compatibility with the GPU rasterization pipeline is why real-time rendering works.

**Differentiable rasterization libraries**

Tools for rendering mesh-based 3D representations in a differentiable way.

- **PyTorch3D** (Meta): provides a differentiable mesh renderer and point cloud renderer.
- **nvdiffrast** (NVIDIA): high-performance CUDA-based differentiable rasterizer.
- **Kaolin** (NVIDIA): a library for 3D deep learning in general.

**Analysis-by-synthesis pipeline**

```
1. Set up a hypothesized 3D scene (NeRF, 3DGS, mesh, etc.)
2. Render that scene from the camera viewpoint → predicted image
3. Compare with the actually observed image → compute loss
4. Backpropagate the loss to update 3D scene parameters
5. Iterate
```

The advantage of this paradigm is that, regardless of the 3D representation form, you can use the common supervision of "images." No separate 3D ground truth is needed.

**Connection to SLAM**

When differentiable rendering is applied to SLAM, both tracking and mapping can be optimized through rendering loss.

- **NeRF-SLAM**: uses NeRF as the map representation. When a new frame comes in, it renders the current map to estimate the pose (tracking), then updates the map (mapping).
- **3DGS-SLAM** (SplaTAM, MonoGS, and so on): uses 3DGS as the map. Real-time rendering makes it faster than NeRF-SLAM.

```
[new frame] → [render current map] → [compare render vs. real]
                                      ↓
                              [update pose (tracking)]
                              [update map (mapping)]
```

> **Further reading**
> - [Tewari et al., "Advances in Neural Rendering" (EGSR 2022)](https://arxiv.org/abs/2111.05849) — survey of differentiable rendering.
> - [Ravi et al., "Accelerating 3D Deep Learning with PyTorch3D" (2020)](https://arxiv.org/abs/2007.08501) — the PyTorch3D paper.
> - [Laine et al., "Modular Primitives for High-Performance Differentiable Rendering" (2020)](https://arxiv.org/abs/2011.03277) — the nvdiffrast paper.

## 13.8 Advanced: 3D Scene Graph

*If you want to become a researcher, read from here.*

If you tell a robot "bring me the red cup in the kitchen," a point cloud or mesh alone cannot carry out the command. The robot has to understand where "the kitchen" is, which object is "the red cup," and the relation that it is "inside" the kitchen. A 3D scene graph represents the environment as a semantic relation graph beyond a purely geometric representation.

**Structure**

- **Node**: objects, rooms, buildings, etc. — a hierarchical structure
  - building → floor → room → object
  - each node carries a 3D position, bounding box, and semantic label
- **Edge**: relations between nodes
  - "on", "in", "near", "support", and so on

```
[Building]
  └── [Floor 1]
        ├── [Kitchen]
        │     ├── [Table]  ──(on)──  [Red Cup]
        │     ├── [Sink]
        │     └── [Chair]
        └── [Living Room]
              ├── [Sofa]
              └── [TV]
```

**Hydra**

A real-time 3D scene graph construction system developed at MIT. It takes RGB-D or LiDAR input and incrementally builds a hierarchical scene graph as the robot moves.

Pipeline:
1. Build a metric-semantic mesh (TSDF + semantic segmentation)
2. Partition into rooms (free-space clustering)
3. Extract object nodes and establish relations
4. Connect the hierarchy

The core of Hydra is that this entire process runs online in real time. The scene graph is updated while the robot explores.

**ConceptGraphs**

Research that builds open-vocabulary scene graphs using foundation models (CLIP, LLM).

Prior scene graphs relied on a predefined set of categories (chair, table, and so on). ConceptGraphs uses CLIP to find objects matching arbitrary natural-language queries, and uses an LLM to infer relations between objects.

```
1. Detect objects in RGB-D frames with an open-vocabulary detector
2. Extract object embeddings with CLIP features
3. Merge identical objects in 3D space (multi-view association)
4. Infer inter-object relations with an LLM
5. Build the scene graph
```

The key point is that it can handle queries like "red cup" that were never seen at training time.

**Why is this needed?**

| Representation | Can execute "bring me the red cup in the kitchen"? | Reason |
|------|------|------|
| Point Cloud | No | No semantic information |
| Semantic Map | Partial | Can find "cup" but struggles with the relation "in the kitchen" |
| 3D Scene Graph | Yes | Expresses objects, relations, and hierarchy together |

In task planning, natural-language-driven navigation, and human-robot interaction, a scene graph bridges 3D representations and high-level reasoning.

> **Further reading**
> - [Hughes et al., "Hydra: A Real-time Spatial Perception System for 3D Scene Graph Construction and Optimization" (RSS 2022)](https://arxiv.org/abs/2201.13360) — the original Hydra paper.
> - [Gu et al., "ConceptGraphs: Open-Vocabulary 3D Scene Graphs for Perception and Planning" (2023)](https://arxiv.org/abs/2309.16650) — the ConceptGraphs paper.
> - [Rosinol et al., "3D Dynamic Scene Graphs: Actionable Spatial Perception with Places, Objects, and Humans" (RSS 2020)](https://arxiv.org/abs/2002.06289) — introduces the Dynamic Scene Graph concept.
> - [Armeni et al., "3D Scene Graph: A Structure for Unified Semantics, 3D Space, and Camera" (ICCV 2019)](https://arxiv.org/abs/1910.02527) — early work on 3D scene graphs.

> **Technical Timeline: 3D Vision**
> - **~2010**: the classical era of point cloud processing. The PCL library, ICP registration, and TSDF-based volumetric reconstruction dominate.
> - **2012~**: the release of the Kinect popularized RGB-D-based 3D reconstruction. KinectFusion (TSDF + ICP) opened the door to real-time 3D reconstruction.
> - **2017~**: point cloud deep learning begins with PointNet/PointNet++. 3D object detection research such as VoxelNet and PointPillars also surges in this period.
> - **2020~**: the arrival of NeRF brings neural rendering into the spotlight. A handful of photos can yield a photorealistic 3D scene, and follow-ups such as Instant-NGP and Mip-NeRF arrive in quick succession.
> - **2023~**: 3D Gaussian Splatting overcomes NeRF's speed limits. It combines real-time rendering with the advantages of explicit representation, and multi-modal 3D detection such as BEVFusion becomes the reference in autonomous driving.
> - **2024~**: the combination of 3DGS + SLAM (SplaTAM, MonoGS, Gaussian-SLAM) is opening up a new direction for neural SLAM. Robots can build photorealistic 3D maps in real time as they move.
> - **Worth watching now**: 3DGS-based methods are rapidly proliferating in SLAM/robotics applications. NeRFStudio lets you experiment with both, so I recommend comparing them directly.
