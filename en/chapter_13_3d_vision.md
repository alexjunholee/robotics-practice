# Ch.13 — 3D Vision


Whether an embodied policy from Ch.12 is grasping a mug or planning an obstacle-free path, 2D images alone leave distance, surface geometry, and occlusion ambiguous. **3D vision** recovers the spatial structure needed to answer "how far away is that object?" or "what lies behind that wall?" Spanning point cloud processing, 3D object detection, multi-view reconstruction, neural rendering, and semantic scene graphs, these techniques form the shared geometric backbone for both manipulation and SLAM (Ch.14).

## 13.1 Point Cloud Basics

Depth cameras and LiDAR sensors output **point clouds**: sets of 3D points scattered irregularly through space rather than arranged on a uniform 2D pixel grid. Every point stores at least `(x, y, z)` coordinates and often carries extra attributes such as RGB color, LiDAR reflectance (intensity), or a surface normal. Learning how to represent and manipulate this unstructured data is the entry point to 3D vision.

### 13.1.1 Data Structures and Formats

In linear-algebra terms, a point cloud is simply an N × D matrix of N points (ranging from tens of thousands to millions) with D attributes per point:

```
Point: [x, y, z, r, g, b, intensity, ...]
Point Cloud: N × D matrix (N points, D-dimensional attributes)
```

Applying a rigid-body rotation and translation amounts to converting each `(x, y, z)` coordinate into homogeneous form and multiplying by a 4 × 4 transformation matrix. Color and reflectance attributes remain unchanged under rigid transforms, while surface normals transform by the 3 × 3 rotation component alone.

Across robotics and geospatial pipelines, five file formats appear most often depending on tooling and storage needs:

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

Two open-source libraries dominate point cloud workflows in robotics, splitting along language and workflow preferences. **PCL (Point Cloud Library)** is C++-based, deeply integrated with ROS, and provides comprehensive classical filtering, segmentation, and registration algorithms:

```cpp
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::io::loadPCDFile<pcl::PointXYZ>("cloud.pcd", *cloud);
```

**Open3D** pairs a modern Python/C++ API with interactive visualization and clean NumPy/PyTorch interoperability, making it well suited to rapid prototyping and deep-learning pipelines:

```python
import open3d as o3d

# Load point cloud
pcd = o3d.io.read_point_cloud("cloud.pcd")

# Visualize
o3d.visualization.draw_geometries([pcd])

# Convert to NumPy
points = np.asarray(pcd.points)  # (N, 3)
```

In practice, teams prototype algorithms quickly in Python with Open3D and move to PCL when deploying production C++ ROS nodes; starting with Open3D is usually the fastest way to build intuition.

> **Further reading**
> - [Open3D Getting Started](http://www.open3d.org/docs/release/getting_started.html) — introduction to handling point clouds in Python.
> - [PCL Tutorials — Basic Usage](https://pcl.readthedocs.io/projects/tutorials/en/latest/#basic-usage) — C++-based point cloud processing.
> - [Open3D YouTube Channel](https://www.youtube.com/@Open3D) — visualization and processing tutorials.

## 13.2 Point Cloud Processing

Raw sensor scans arrive noisy, unevenly sampled, and expressed only in the sensor's instantaneous frame. Before a robot can detect objects or build a map, a standard preprocessing pipeline filters the points, estimates local surface orientation, and aligns successive scans into a common coordinate frame.

### 13.2.1 Filtering

Dense raw point clouds inflate compute time and harbor stray measurements from specular surfaces or sensor edges that corrupt downstream registration and segmentation. Three filtering operations form the standard first stage:

- **Voxel grid downsampling**: partitions 3D space into cubic voxels and replaces all points inside each occupied cell with a single representative point, bounding density uniformly:

```python
# Open3D
voxel_pcd = pcd.voxel_down_sample(voxel_size=0.05)  # 5cm grid
```

- **Statistical outlier removal**: discards isolated points whose average distance to their nearest neighbors deviates significantly from the global neighborhood distribution:

```python
# Outlier removal
cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
filtered_pcd = pcd.select_by_index(ind)
```

- **Radius outlier removal**: removes points that have fewer than a minimum number of neighbors within a specified metric radius, catching sparse flying pixels cleanly.

> **Further reading**
> - [Open3D — Point Cloud Filtering Tutorial](http://www.open3d.org/docs/release/tutorial/geometry/pointcloud.html) — examples of voxel downsampling and outlier removal.
> - [PCL — Filtering Tutorial](https://pcl.readthedocs.io/projects/tutorials/en/latest/passthrough.html) — PCL-based filtering.

### 13.2.2 Normal Estimation

Many geometric algorithms—including surface reconstruction, shading, and point-to-plane registration—need local surface orientation in addition to raw 3D coordinates. Without estimated **surface normals**, a pipeline cannot distinguish a flat wall from a sharp edge. In Open3D, hybrid neighbor search estimates normals directly:

```python
# Normal estimation
pcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
)
```

Under the hood, normal estimation gathers the k nearest neighbors around each query point, builds their 3 × 3 covariance matrix, and selects the eigenvector corresponding to the smallest eigenvalue—the exact same principal-axis decomposition used in Principal Component Analysis (PCA).

### 13.2.3 Registration

**Registration** aligns two point clouds into a shared coordinate frame, allowing SLAM systems to stitch consecutive LiDAR or RGB-D frames together and merge scans captured from multiple viewpoints.

The classic workhorse is **ICP (Iterative Closest Point)**, which alternates between finding nearest neighbors and solving for a rigid transform until convergence:

1. Find the closest point pairs between source and target clouds.
2. Solve for the optimal rotation and translation in least squares.
3. Apply the transform to the source cloud.
4. Repeat until alignment converges.

```python
# Point-to-Point ICP
reg = o3d.pipelines.registration.registration_icp(
    source, target, max_correspondence_distance=0.05,
    estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint()
)
transformation = reg.transformation
```

Intuitively, ICP repeatedly pairs nearest points across the two clouds and uses Singular Value Decomposition (SVD) to compute the rigid rotation R and translation t that minimize squared correspondence distance; because initial pairings are imperfect when the clouds are misaligned, iterating refines both correspondences and pose together. Several variants address ICP's failure modes:

- **Point-to-Plane ICP**: minimizes distance along target surface normals rather than point-to-point distance, yielding faster convergence and higher accuracy on planar scenes.
- **GICP (Generalized ICP)**: models local planar covariance around each point to unify point-to-point and point-to-plane behavior.
- **NDT (Normal Distributions Transform)**: partitions space into voxels and aligns Gaussian point distributions per cell rather than discrete nearest neighbors.
- **Feature-based global registration**: extracts local 3D descriptors such as FPFH or SHOT, computes an initial coarse alignment via RANSAC when the initial pose offset is large, and refines the result with ICP.

> **Further reading**
> - [Open3D — ICP Registration Tutorial](http://www.open3d.org/docs/release/tutorial/pipelines/icp_registration.html) — hands-on ICP code.
> - [Open3D — Global Registration (RANSAC + Feature)](http://www.open3d.org/docs/release/tutorial/pipelines/global_registration.html) — feature-based registration.
> - [Cyrill Stachniss — ICP & Point Cloud Registration](https://www.youtube.com/watch?v=dhzLQfDBx2Q) — intuitive explanation of the ICP algorithm.

> **Exercise**: [Step-by-step 2D ICP visualization](https://alexjunholee.github.io/robotics-practice/app.html#icp_steps) | [3D ICP](https://alexjunholee.github.io/robotics-practice/app.html#icp_3d)
> Inspect ICP registering two point clouds iteration by iteration, and compare convergence in 2D and 3D environments.

## 13.3 3D Object Detection

For autonomous driving and mobile manipulation, knowing that an object exists in 2D pixels is not enough—a planner needs **3D bounding boxes** specifying each object's 3D position, dimensions, and heading. Deep 3D detectors generally operate on raw points, regularize space into voxels or pillars, or fuse LiDAR with camera imagery.

### 13.3.1 Point-based Methods

Standard CNNs assume a dense 2D grid and cannot ingest an unordered set of 3D coordinates directly. **PointNet (2017)** bypassed voxelization by applying a shared per-point MLP `h` followed by symmetric max pooling `MAX`, giving `f({x1, ..., xn}) = g(MAX(h(x1), ..., h(xn)))`. Because max pooling is symmetric, the output is **permutation invariant**—unaffected by how the input points are ordered—while producing a global shape descriptor.

Because a single global pooling step discards fine local geometry, **PointNet++ (2017)** introduced hierarchical **Set Abstraction** layers to capture multi-scale local patterns:

```python
# PointNet++ conceptual structure
# 1. Sampling: pick centers via FPS
# 2. Grouping: gather neighbors via ball query
# 3. PointNet: extract features in each group
```

### 13.3.2 Voxel-based Methods

Neighbor queries on raw points become expensive at outdoor LiDAR scales. Voxel-based detectors instead discretize 3D space onto structured grids so convolutional backbones can run efficiently:

- **VoxelNet (2018)**: groups points into 3D voxels, encodes each voxel via Voxel Feature Encoding (VFE), and applies dense 3D convolutions.
- **SECOND (Sparsely Embedded Convolutional Detection)**: exploits the fact that most 3D voxels in a LiDAR scan are empty, replacing dense 3D convolutions with sparse convolutions for substantially faster inference and serving as a standard baseline.
- **PointPillars (2019)**: eliminates 3D convolutions altogether by collapsing height into vertical columns ("pillars"), encoding the points inside each pillar into a feature vector, and laying those vectors onto a 2D pseudo-image so a fast real-time 2D CNN can process the scan.

### 13.3.3 Multi-modal Methods

Each sensing modality covers the other's blind spots: cameras supply dense color and semantic texture without direct metric depth, whereas LiDAR supplies accurate 3D geometry without texture or fine appearance cues. Multi-modal 3D detectors combine both streams into a shared spatial representation:

- **BEVFusion**: projects both camera features and LiDAR features into a unified Bird's Eye View (BEV) plane so downstream heads operate seamlessly across modalities.
- **TransFusion**: uses Transformer cross-attention queries to fuse image and LiDAR features adaptively during detection.

> **Further reading**
> - [Qi et al., "PointNet: Deep Learning on Point Sets" (2017)](https://arxiv.org/abs/1612.00593) — the starting point of 3D deep learning.
> - [Lang et al., "PointPillars" (2019)](https://arxiv.org/abs/1812.05784) — real-time 3D detection.
> - [Liu et al., "BEVFusion" (2023)](https://arxiv.org/abs/2205.13542) — a widely used reference in multi-modal fusion.
> - [MMDetection3D GitHub](https://github.com/open-mmlab/mmdetection3d) — unified 3D object detection framework.

> **Exercise**: [BEV Projection Visualization](https://alexjunholee.github.io/robotics-practice/app.html#bev_projection)
> Interactively inspect the process of converting a camera image into BEV, and grasp the principle of BEV-based 3D detection.

## 13.4 3D Reconstruction

Beyond detecting isolated objects, a robot often needs a persistent 3D model of its entire environment built from multi-view imagery or depth streams.

### 13.4.1 Structure from Motion (SfM)

**Structure from Motion (SfM)** recovers both camera poses and sparse 3D scene structure simultaneously from a collection of overlapping 2D photographs. Alongside standalone modeling—turning smartphone photos into a 3D building scan—SfM is the standard preprocessing step that provides calibrated camera poses for NeRF and 3D Gaussian Splatting (3DGS).

An incremental SfM pipeline proceeds in four stages:

1. Feature extraction and matching across image pairs.
2. Triangulation from an initial well-conditioned two-view pair.
3. Incremental registration of additional cameras and 3D points.
4. **Bundle adjustment (BA)**—jointly refining all camera poses and 3D point coordinates via large-scale nonlinear least squares (such as Levenberg-Marquardt) spanning tens of thousands to hundreds of thousands of variables.

In practice, **OpenMVG** offers a modular library interface, while **COLMAP** serves as a widely adopted public SfM/MVS reference implementation via both GUI and CLI:

```bash
# Using COLMAP
colmap feature_extractor --database_path db.db --image_path ./images
colmap exhaustive_matcher --database_path db.db
mkdir -p sparse
colmap mapper --database_path db.db --image_path ./images --output_path ./sparse
```

> **Further reading**
> - [COLMAP Documentation](https://colmap.github.io/) — documentation for a widely used public SfM/MVS implementation.
> - [Daniel Cremers — Multiple View Geometry (TUM)](https://www.youtube.com/playlist?list=PLTBdjV_4f-EJn6udZ34tht9EVIW7lbeo4) — core lectures on multiple view geometry.
> - [Schönberger & Frahm, "Structure-from-Motion Revisited" (2016)](https://openaccess.thecvf.com/content_cvpr_2016/papers/Schonberger_Structure-From-Motion_Revisited_CVPR_2016_paper.pdf) — the COLMAP paper.

### 13.4.2 Multi-View Stereo (MVS)

While SfM recovers camera trajectories and a sparse cloud of matched keypoints, **Multi-View Stereo (MVS)** uses those calibrated poses to compute dense depth maps and reconstruct a **dense** 3D point cloud. Together, SfM → MVS → surface meshing form the classical multi-view reconstruction pipeline, implemented in tools such as COLMAP's dense mode and OpenMVS.

### 13.4.3 Volumetric Reconstruction

When depth sensors (such as RGB-D cameras) are available, **TSDF (Truncated Signed Distance Function)** fusion provides a clean volumetric alternative. Space is discretized into a 3D voxel grid where each voxel stores a truncated signed distance to the nearest observed surface—positive outside the surface, negative inside, and zero directly on the surface. Averaging weighted depth observations across multiple viewpoints suppresses sensor noise, and the **Marching Cubes** algorithm extracts a triangle mesh along the zero crossing:

```python
# Open3D TSDF Integration
volume = o3d.pipelines.integration.ScalableTSDFVolume(
    voxel_length=0.01,
    sdf_trunc=0.04,
    color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8
)

for i, (color, depth, pose) in enumerate(frames):
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color, depth, depth_trunc=4.0, convert_rgb_to_intensity=False)
    volume.integrate(rgbd, intrinsic, np.linalg.inv(pose))

mesh = volume.extract_triangle_mesh()
```

> **Further reading**
> - [Open3D — TSDF Integration Tutorial](http://www.open3d.org/docs/release/tutorial/pipelines/rgbd_integration.html) — hands-on TSDF code.
> - [Curless & Levoy, "A Volumetric Method for Building Complex Models from Range Images" (1996)](https://graphics.stanford.edu/papers/volrange/volrange.pdf) — the original TSDF paper (a classic, but still worth reading).

## 13.5 Neural Rendering

Classical meshes and point clouds struggle with view-dependent reflections, translucent materials, and thin geometric structures. **Neural rendering** replaces rigid discrete primitives with learnable scene representations optimized directly from images, enabling photorealistic novel view synthesis and opening new avenues for dense online SLAM.

### 13.5.1 NeRF (Neural Radiance Fields)

**NeRF** models an entire 3D scene as a continuous neural function mapping 3D position `(x, y, z)` and viewing direction `(θ, φ)` to emitted RGB color and volume density `σ`:

```
F: (x, y, z, θ, φ) → (r, g, b, σ)
- Position (x, y, z) and viewing direction (θ, φ)
- Outputs color (r, g, b) and density (σ)
```

To render a pixel, NeRF marches a camera ray through this continuous field and integrates color and density via classical volume rendering. Once fitted to a set of posed photos, the network synthesizes photorealistic novel views from arbitrary camera viewpoints and naturally captures specular reflections and transparency.

Its primary drawbacks are slow optimization and difficulty modeling dynamic scenes, though follow-up variants address several limits: **Instant-NGP** accelerates training from hours to minutes with multiresolution hash encoding, **Mip-NeRF** eliminates aliasing across scales, and **Block-NeRF** scales representation to city-block environments.

> **Further reading**
> - [Mildenhall et al., "NeRF: Representing Scenes as Neural Radiance Fields" (2020)](https://arxiv.org/abs/2003.08934) — the original NeRF paper.
> - [NeRFStudio Documentation](https://docs.nerf.studio/) — a unified framework that makes NeRF experiments easy. Start here if you want to run NeRF yourself.
> - [Yannic Kilcher — NeRF Explained](https://www.youtube.com/watch?v=CRlN-cYFxTk) — an intuitive explanation of NeRF's representation.
> - [Jon Barron — Understanding NeRF (ECCV 2022 Tutorial)](https://www.youtube.com/watch?v=HfJpQCBTqZs) — from a NeRF author.

### 13.5.2 3D Gaussian Splatting (3DGS)

Although NeRF produces striking visuals, ray-marching neural networks can take seconds per frame at inference time. **3D Gaussian Splatting (3DGS)** was adopted rapidly across vision and robotics because it trains in minutes and renders trained scenes at more than 100 FPS—fast enough to revisit dense photorealistic mapping and SLAM.

Rather than querying an implicit MLP along rays, 3DGS represents the scene explicitly using millions of anisotropic 3D Gaussians. Each Gaussian stores:

- **Position (mean)** in 3D space.
- **Covariance** defining its shape, scale, and orientation—just as in linear algebra, the eigenvectors of the 3 × 3 covariance matrix orient the ellipsoid's principal axes while the eigenvalues set their lengths.
- **Color** parameterized via spherical harmonics for view-dependent appearance.
- **Opacity** controlling transparency during compositing.

Instead of expensive volumetric ray marching, rendering projects these Gaussians directly onto the image plane (**splatting**):

```python
# 3DGS basic idea (pseudo-code)
# Each Gaussian: position, covariance, color, opacity
# Rendering: project to camera view to generate the image
```

Because Gaussians are explicit and compatible with GPU rasterization, 3DGS supports straightforward scene editing alongside real-time rendering—driving applications across mapping, dynamic scenes, and neural SLAM.

**3DGS + SLAM**: Combining 3DGS with SLAM lets a mobile robot build photorealistic dense maps online instead of sparse point clouds. Keep in mind, however, that quoted 100+ FPS figures reflect rendering throughput on an already-trained scene; end-to-end SLAM throughput—including camera tracking and iterative Gaussian optimization—runs substantially slower:

- **SplaTAM (2024)**: performs dense RGB-D 3DGS SLAM by alternating between camera pose tracking and map updates, improving rendering fidelity and speed over earlier neural SLAM systems.
- **MonoGS (2024)**: builds dense 3DGS maps from a monocular RGB camera alone without requiring a depth sensor.
- **Gaussian-SLAM (2024)**: scales 3DGS SLAM to larger environments by partitioning maps into local sub-maps.

Together, these systems point toward real-time digital twins, inspection, and AR/VR scene capture directly from a moving robot.

> **Further reading**
> - [Kerbl et al., "3D Gaussian Splatting for Real-Time Radiance Field Rendering" (2023)](https://arxiv.org/abs/2308.04079) — the original 3DGS paper.
> - [Huang et al., "2D Gaussian Splatting for Geometrically Accurate Radiance Fields" (SIGGRAPH 2024, arXiv:2403.17888)](https://arxiv.org/abs/2403.17888) — improves surface reconstruction quality via 2D Gaussians.
> - [Keetha et al., "SplaTAM: Splat, Track & Map 3D Gaussians for Dense RGB-D SLAM" (2024)](https://arxiv.org/abs/2312.02126) — an early 3DGS + SLAM system.
> - [Matsuki et al., "Gaussian Splatting SLAM" (2024)](https://arxiv.org/abs/2312.06741) — the MonoGS paper.
> - [Wang et al., "DUSt3R: Geometric 3D Vision Made Easy" (CVPR 2024, arXiv:2312.14132)](https://arxiv.org/abs/2312.14132) — reconstructs dense 3D from image pairs without camera intrinsics or extrinsics.
> - [Leroy et al., "Grounding Image Matching in 3D with MASt3R" (ECCV 2024, arXiv:2406.09756)](https://arxiv.org/abs/2406.09756) — adds local feature matching to DUSt3R. Provides reconstruction and precise correspondences simultaneously.
> - [NeRFStudio Documentation](https://docs.nerf.studio/) — unified framework for NeRF/3DGS experiments.
> - [3DGS Original Implementation (GitHub)](https://github.com/graphdeco-inria/gaussian-splatting) — the official code.

> **Exercise**: [3D Gaussian Splatting Visualization](https://alexjunholee.github.io/robotics-practice/app.html#gaussian_splatting)
> Manipulate the position, covariance, and color of 3D Gaussians to interactively understand the splatting rendering process.

## 13.6 Advanced: Neural Implicit Representations

While NeRF excels at novel view synthesis, extracting crisp geometric surfaces from an unconstrained density field is difficult. For robotics tasks such as contact-rich grasping and collision checking, an explicit, well-defined boundary is essential—motivating **Signed Distance Function (SDF)** representations.

An **SDF** maps every 3D point `x` to its signed Euclidean distance to the nearest surface:

```
f(x) > 0  : outside the surface
f(x) < 0  : inside the surface
f(x) = 0  : on the surface (zero level set)
```

A defining mathematical property of any valid SDF is that its spatial gradient has unit norm wherever `f` is differentiable (the **Eikonal equation**):

```
||∇f(x)|| = 1
```

When fitting a neural SDF, enforcing this unit-gradient constraint as a regularization penalty—the **Eikonal loss**—prevents degenerate zero solutions and keeps the field metric:

```
L_eikonal = E_x[ (||∇f_θ(x)|| - 1)^2 ]
```

Several influential architectures build on this formulation to balance geometry and appearance:

- **DeepSDF**: pioneers neural SDF shape representation using an auto-decoder architecture that encodes each object shape in a latent vector `z` recovered at test time via optimization:

```
f_θ(z, x) → SDF value
```

- **NeuS**: bridges NeRF's photorealistic volume rendering with clean SDF surfaces by converting SDF values into unbiased volume density along rays `r(t) = o + t·d`:

```
density ρ(t) = max(-dΦ_s(f(r(t)))/dt, 0) / Φ_s(f(r(t))),   r(t) = o + t·d is a point along the ray
```

  Here `Φ_s` is a sigmoid scaled by a learnable sharpness parameter `s`; its derivative forms an S-density of width `1/s`, so as `s` grows during optimization, density concentrates tightly around the true surface `f = 0`.
- **VolSDF**: takes an analogous approach by modeling volume density through the cumulative distribution function `Ψ_β` of a Laplace distribution applied to the signed distance:

```
σ(x) = (1/β) · Ψ_β(-f(x))
```

  As the scale parameter `β` shrinks during training, density collapses sharply onto the surface boundary.

Once an SDF is trained, the **Marching Cubes** algorithm extracts an explicit triangle mesh by evaluating the sign of `f(x)` on a 3D grid and interpolating vertices wherever edges cross the zero level set `f(x) = 0`.

The table below summarizes how the major 3D scene representations trade off rendering fidelity, surface precision, runtime speed, and memory:

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
> - [Mescheder et al., "Occupancy Networks" (CVPR 2019)](https://arxiv.org/abs/1812.03828) — an early reference for occupancy-based approaches.

## 13.7 Advanced: Differentiable Rendering

NeRF, 3DGS, and NeuS all rest on a shared paradigm called **analysis-by-synthesis**: make the forward rendering pipeline differentiable so that photometric differences between rendered images and real camera observations backpropagate directly into the 3D scene parameters—requiring no 3D ground-truth supervision at all.

In the NeRF family, **volume rendering** integrates radiance along a camera ray `r(t) = o + td`:

```
C(r) = ∫ T(t) · σ(t) · c(t) dt

where T(t) = exp( -∫_{t_n}^{t} σ(s) ds )
```

Here `σ(t)` is the volume density (in units of inverse length), `c(t)` is the RGB color at distance `t`, and `T(t)` is the accumulated transmittance—the probability that the ray travels from near bound to `t` without being absorbed. Numerically, ray marching discretizes this integral over N quadrature samples along the ray:

```
C(r) ≈ Σ_i T_i · α_i · c_i
where α_i = 1 - exp(-σ_i · δ_i),  T_i = Π_{j<i} (1 - α_j)
```

**3DGS** keeps essentially the same alpha-compositing algebra, but swaps slow per-ray neural sampling for **tile-based GPU rasterization**: Gaussians are projected onto the image plane, sorted by depth, and composited per pixel:

```
C(p) = Σ_i c_i · α_i · Π_{j<i} (1 - α_j)
```

Aligning alpha compositing with hardware rasterization is what unlocks 100+ FPS rendering while preserving end-to-end differentiability. For explicit meshes and point clouds, dedicated libraries—**PyTorch3D** (Meta, supporting meshes and point clouds), **nvdiffrast** (NVIDIA, high-performance CUDA rasterizer), and **Kaolin** (NVIDIA, general 3D deep learning)—expose differentiable rasterizers directly inside PyTorch graphs.

Across all of these representations, optimization follows the same closed analysis-by-synthesis loop:

```
1. Set up a hypothesized 3D scene (NeRF, 3DGS, mesh, etc.)
2. Render that scene from the camera viewpoint → predicted image
3. Compare with the actually observed image → compute loss
4. Backpropagate the loss to update 3D scene parameters
5. Iterate
```

When embedded into **SLAM**, differentiable rendering turns photometric loss into a unified objective for both tracking and mapping—driving **NeRF-SLAM** systems and faster **3DGS-SLAM** pipelines such as SplaTAM and MonoGS:

```
[new frame] → [render current map] → [compare render vs. real]
                                      ↓
                              [update pose (tracking)]
                              [update map (mapping)]
```

> **Further reading**
> - [Tewari et al., "Advances in Neural Rendering" (EUROGRAPHICS 2022 STAR)](https://arxiv.org/abs/2111.05849) — survey of differentiable rendering.
> - [Ravi et al., "Accelerating 3D Deep Learning with PyTorch3D" (2020)](https://arxiv.org/abs/2007.08501) — the PyTorch3D paper.
> - [Laine et al., "Modular Primitives for High-Performance Differentiable Rendering" (2020)](https://arxiv.org/abs/2011.03277) — the nvdiffrast paper.

## 13.8 Advanced: 3D Scene Graph

Geometric maps alone cannot satisfy high-level commands such as "bring me the red cup in the kitchen." Fulfilling that request requires knowing which region constitutes "the kitchen," which object is "the red cup," and that the cup is currently "in" that room. A **3D scene graph** bridges geometry and high-level task reasoning by organizing the environment into hierarchical semantic nodes and relational edges:

- **Nodes**: multi-scale entities arranged hierarchically (`building → floor → room → object`), each storing a 3D position, bounding box, and semantic label.
- **Edges**: spatial and semantic relations linking nodes, such as `"on"`, `"in"`, `"near"`, or `"support"`.

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

Two complementary systems illustrate how robotics builds and queries these graphs:

- **Hydra** (MIT) constructs hierarchical 3D scene graphs online in real time from RGB-D or LiDAR streams as the robot explores. Its pipeline builds a metric-semantic TSDF mesh, clusters free space into rooms, extracts object nodes and spatial relations, and links the full hierarchy continuously during motion.
- **ConceptGraphs** removes closed-set category limits by pairing open-vocabulary vision foundation models with LLMs. Rather than locking nodes to predefined classes like "chair" or "table," it uses CLIP embeddings to match arbitrary phrases such as "red cup" and queries an LLM to infer inter-object relations:

```
1. Detect objects in RGB-D frames with an open-vocabulary detector
2. Extract object embeddings with CLIP features
3. Merge identical objects in 3D space (multi-view association)
4. Infer inter-object relations with an LLM
5. Build the scene graph
```

Comparing geometric, semantic, and relational representations side by side makes clear why task-level planning and human-robot interaction demand scene graphs:

| Representation | Can execute "bring me the red cup in the kitchen"? | Reason |
|------|------|------|
| Point Cloud | No | No semantic information |
| Semantic Map | Partial | Can find "cup" but struggles with the relation "in the kitchen" |
| 3D Scene Graph | Yes | Expresses objects, relations, and hierarchy together |

> **Further reading**
> - [Hughes et al., "Hydra: A Real-time Spatial Perception System for 3D Scene Graph Construction and Optimization" (RSS 2022)](https://arxiv.org/abs/2201.13360) — the original Hydra paper.
> - [Gu et al., "ConceptGraphs: Open-Vocabulary 3D Scene Graphs for Perception and Planning" (2023)](https://arxiv.org/abs/2309.16650) — the ConceptGraphs paper.
> - [Rosinol et al., "3D Dynamic Scene Graphs: Actionable Spatial Perception with Places, Objects, and Humans" (RSS 2020)](https://arxiv.org/abs/2002.06289) — introduces the Dynamic Scene Graph concept.
> - [Armeni et al., "3D Scene Graph: A Structure for Unified Semantics, 3D Space, and Camera" (ICCV 2019)](https://arxiv.org/abs/1910.02527) — early work on 3D scene graphs.

> **Technical Timeline: 3D Vision**
> - **~2010**: Classical point cloud processing dominates, anchored by PCL, ICP registration, and TSDF volumetric reconstruction.
> - **2010~**: The release of Microsoft Kinect (2010) and KinectFusion (2011, TSDF + ICP) makes real-time RGB-D 3D reconstruction widely accessible.
> - **2017~**: Deep learning enters raw point cloud processing with PointNet and PointNet++, alongside rapid advances in 3D detection such as VoxelNet and PointPillars.
> - **2020~**: NeRF ignites neural rendering from posed photos, quickly followed by Instant-NGP and Mip-NeRF.
> - **2023~**: 3D Gaussian Splatting removes NeRF's rendering bottleneck with explicit real-time splatting, while multi-modal architectures such as BEVFusion become standard references in autonomous driving.
> - **2024~**: Coupling 3DGS with SLAM (SplaTAM, MonoGS, Gaussian-SLAM) opens a new front in dense neural SLAM; rendering reaches real time, though end-to-end throughput including tracking and mapping optimization remains slower.
> - **Recent direction**: More robotics and SLAM pipelines are adopting 3DGS scene representations, while frameworks such as NeRFStudio let researchers benchmark NeRF and 3DGS representations and rendering speed side by side on shared data.

Turning these 3D representations into consistent online pose estimates and drift-corrected maps as a robot explores unfamiliar environments is the central problem of SLAM and odometry in Ch.14.
