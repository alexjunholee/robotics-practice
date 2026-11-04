# Ch.13 — 3D 비전 (3D Vision)


2D 이미지 한 장으로는 로봇이 물체까지 거리가 얼마인지 바로 알기 어렵다. 3D 비전은 포인트 클라우드를 처리하고, 3D 물체를 감지하고, 장면을 복원해서 공간의 형상과 거리를 구한다. SLAM과 로봇 조작(manipulation)이 이 결과를 쓴다.

## 13.1 Point Cloud 기초

LiDAR나 깊이 카메라는 포인트 클라우드를 출력한다. 이미지는 픽셀 격자에 정렬된 2D 데이터다. 포인트 클라우드는 3D 공간에 불규칙하게 흩어진 점들의 집합이다.

각 점에는 적어도 (x, y, z) 좌표가 있고, 색상(RGB), 반사도(intensity), 법선(normal) 같은 속성이 더 붙기도 한다.

### 13.1.1 데이터 구조 및 포맷

**일반적인 구조**:

```
Point: [x, y, z, r, g, b, intensity, ...]
Point Cloud: N × D 행렬 (N개 점, D차원 속성)
```

선형대수로 보면 포인트 클라우드는 N×D 행렬이다. N은 점의 개수로 수만에서 수백만에 이르고, D는 점 하나에 붙은 속성의 차원이다. 회전하고 이동할 때는 각 점의 위치 좌표를 동차 좌표로 바꾼 뒤 4×4 변환 행렬을 곱한다. 색상과 반사도는 그대로 둔다. 법선에는 회전만 적용한다.

**주요 파일 포맷**:
| 포맷 | 특징 |
|---|---|
| PCD | PCL 표준, ASCII/Binary |
| PLY | 다목적, mesh도 지원 |
| LAS/LAZ | 지리정보 표준, LAZ는 압축 |
| XYZ | 단순 텍스트, 좌표만 |
| BIN | KITTI 등에서 사용, 바이너리 |

> **추천 자료**
> - [Open3D Documentation](https://www.open3d.org/docs/release/) — Python과 C++로 쓰는 포인트 클라우드 라이브러리
> - [PCL (Point Cloud Library) Tutorials](https://pcl.readthedocs.io/projects/tutorials/en/latest/) — 오래전부터 써 온 C++ 포인트 클라우드 라이브러리

### 13.1.2 라이브러리

**PCL (Point Cloud Library)**:
- C++ 기반, 가장 포괄적
- ROS와 통합
- 필터링, 세그멘테이션, 정합 등

```cpp
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::io::loadPCDFile<pcl::PointXYZ>("cloud.pcd", *cloud);
```

**Open3D**:
- Python/C++, 현대적 API
- 시각화 강점
- 딥러닝 친화적

```python
import numpy as np
import open3d as o3d

# 포인트 클라우드 읽기
pcd = o3d.io.read_point_cloud("cloud.pcd")

# 시각화
o3d.visualization.draw_geometries([pcd])

# NumPy 변환
points = np.asarray(pcd.points)  # (N, 3)
```

Open3D는 파이썬 바인딩이 있어 알고리즘 프로토타이핑에 편하다. C++ ROS 파이프라인에서는 PCL도 많이 쓴다. 처음이라면 Open3D부터 시작한다.

> **추천 자료**
> - [Open3D Getting Started](https://www.open3d.org/docs/release/getting_started.html) — Python으로 포인트 클라우드 다루기 입문
> - [PCL Tutorials — Basic Usage](https://pcl.readthedocs.io/projects/tutorials/en/latest/#basic-usage) — C++ 기반 포인트 클라우드 처리
> - [Open3D YouTube Channel](https://www.youtube.com/channel/UCRJBlASPfPBtPXJSPffJV-w) — 시각화 및 처리 튜토리얼

## 13.2 Point Cloud 처리

### 13.2.1 필터링 (Filtering)

원시 포인트 클라우드에는 노이즈가 많고 점의 밀도가 위치마다 다르다. 그대로 쓰면 정합이나 세그멘테이션 같은 후속 알고리즘이 느려지거나 결과가 나빠지므로, 파이프라인 첫 단계에서 필터링한다.

**Voxel Grid Downsampling**:
공간을 격자로 나누고, 한 칸에 든 점들을 점 하나로 합친다.

```python
# Open3D
voxel_pcd = pcd.voxel_down_sample(voxel_size=0.05)  # 5cm 격자
```

**Statistical Outlier Removal**:
이웃 점까지 거리의 통계를 보고 이상치를 지운다.

```python
# 이상치 제거
cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
filtered_pcd = pcd.select_by_index(ind)
```

**Radius Outlier Removal**:
정한 반경 안에 이웃이 너무 적은 점을 지운다.

> **추천 자료**
> - [Open3D — Point Cloud Filtering Tutorial](https://www.open3d.org/docs/release/tutorial/geometry/pointcloud.html) — Voxel downsampling, outlier removal 예제
> - [PCL — Filtering Tutorial](https://pcl.readthedocs.io/projects/tutorials/en/latest/passthrough.html) — PCL 기반 필터링

### 13.2.2 Normal Estimation

각 점에서 표면의 법선 벡터를 추정하는 전처리 단계다.

표면 재구성(reconstruction), 조명 계산, point-to-plane ICP가 법선 벡터를 쓴다. point-to-point ICP는 점 좌표만 쓴다. 이웃 점들의 법선 방향을 비교하면 그 점이 평면 위에 있는지 모서리에 있는지 판단할 수 있다.

```python
# 법선 추정
pcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
)
```

Open3D는 각 점 주변 이웃 점들(위 예제에서는 반경 0.1 m 안의 최대 30개)로 공분산 행렬을 만든다. 이 행렬의 최소 고유값에 대응하는 고유벡터를 법선 방향으로 쓴다. 주성분 분석(PCA)에서 분산이 가장 작은 축을 고르는 셈이다.

### 13.2.3 Registration (정합)

두 포인트 클라우드가 겹치도록 맞추는 과정이다. SLAM에서 연속된 프레임을 이어 붙이거나 여러 위치에서 스캔한 데이터를 합칠 때 쓴다.

**ICP (Iterative Closest Point)**:
1. 가장 가까운 점 쌍 찾기
2. 변환 계산 (최소자승법)
3. 변환 적용
4. 수렴할 때까지 반복

```python
# Point-to-Point ICP
reg = o3d.pipelines.registration.registration_icp(
    source, target, max_correspondence_distance=0.05,
    estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint()
)
transformation = reg.transformation
```

ICP는 두 포인트 클라우드에서 가장 가까운 점 쌍을 찾고, 그 쌍의 거리를 최소로 만드는 회전·이동을 구하는 과정을 반복한다. point-to-point ICP에서는 SVD(특이값 분해)로 회전 행렬 R과 이동 벡터 t를 닫힌 형태로 구한다.

Point-to-Plane ICP: 점과 평면 사이 거리를 최소화. 평면이 많은 장면에서 더 빨리 수렴

**GICP (Generalized ICP)**: 각 점 주변의 분포(공분산)를 함께 써서 정합

NDT (Normal Distributions Transform): 공간을 셀로 나누고 각 셀의 정규분포에 점을 매칭

**Feature-based Registration**:
- FPFH, SHOT 등 특징 추출
- RANSAC으로 초기 정합
- ICP로 정밀화

> **추천 자료**
> - [Open3D — ICP Registration Tutorial](https://www.open3d.org/docs/release/tutorial/pipelines/icp_registration.html) — ICP 실습 코드
> - [Open3D — Global Registration (RANSAC + Feature)](https://www.open3d.org/docs/release/tutorial/pipelines/global_registration.html) — Feature 기반 정합
> - [Cyrill Stachniss — ICP & Point Cloud Registration](https://www.youtube.com/watch?v=dhzLQfDBx2Q) — ICP 알고리즘의 직관적 설명

> **실습**: [ICP 2D 단계별 시각화](https://alexjunholee.github.io/robotics-practice/app.html#icp_steps) | [ICP 3D](https://alexjunholee.github.io/robotics-practice/app.html#icp_3d)
> ICP 알고리즘이 두 포인트 클라우드를 정합하는 과정을 iteration별로 확인하고, 2D와 3D 환경에서 수렴 과정을 비교할 수 있다.

## 13.3 3D Object Detection

포인트 클라우드에서 물체를 감싸는 3D bounding box를 예측한다. 자율주행에서는 주변 차량의 위치와 크기를 이렇게 구한다.

### 13.3.1 Point-based Methods

PointNet은 raw 포인트 집합을 신경망 입력으로 바로 받는다. CNN은 격자 구조의 입력을 받으므로, 이전 방법들은 점을 복셀이나 이미지로 바꿔 넣었다.

**PointNet (2017)**:
- Raw 포인트에 직접 적용
- Permutation invariant (점 순서 무관)
- Max pooling으로 global feature

**PointNet++ (2017)**:
- Hierarchical feature learning
- Set Abstraction: 영역별 특징 추출
- 로컬 패턴 학습 가능

```python
# PointNet++ 개념 구조
# 1. Sampling: FPS로 중심점 선택
# 2. Grouping: Ball query로 이웃 수집
# 3. PointNet: 각 그룹에서 특징 추출
```

PointNet의 출력은 포인트 클라우드의 점 순서가 바뀌어도 같아야 한다(permutation invariance). 그래서 각 점을 같은 MLP에 따로 통과시킨 뒤 max pooling으로 모은다. 식으로 쓰면 f({x1, ..., xn}) = g(MAX(h(x1), ..., h(xn)))다.

### 13.3.2 Voxel-based Methods

**VoxelNet (2018)**:
- 포인트 클라우드를 3D 복셀로 변환
- Voxel Feature Encoding
- 3D CNN으로 처리

**SECOND (Sparsely Embedded Convolutional Detection)**:
- Sparse Convolution 사용
- VoxelNet 대비 훨씬 빠름
- 널리 쓰는 베이스라인

**PointPillars (2019)**:
- Pillar (수직 기둥) 단위 처리
- 2D CNN으로 변환하여 빠른 속도
- 실시간 가능

PointPillars는 3D 공간을 수직 기둥(pillar)으로 나누고, 각 pillar 안의 점을 특징 벡터 하나로 압축해 2D 이미지처럼 배열한다. 그 위에 2D CNN을 돌리므로 3D 합성곱보다 계산이 빠르다.

### 13.3.3 Multi-modal Methods

카메라는 색상과 텍스처를 준다. 라이다는 정확한 3차원 좌표를 측정한다. 융합 방법은 두 정보를 한 표현(예: BEV)에 모아 함께 쓴다.

**BEVFusion**:
- Camera + LiDAR 융합
- 조감도(Bird's Eye View, BEV) 공간에서 통합

**TransFusion**:
- Transformer 기반 융합
- Query 기반 detection

> **추천 자료**
> - [Qi et al., "PointNet: Deep Learning on Point Sets" (2017)](https://arxiv.org/abs/1612.00593) — 3D 딥러닝의 시작점
> - [Lang et al., "PointPillars" (2019)](https://arxiv.org/abs/1812.05784) — 실시간 3D 감지
> - [Liu et al., "BEVFusion" (ICRA 2023)](https://arxiv.org/abs/2205.13542) — 카메라·라이다 특징을 BEV에서 융합
> - [MMDetection3D GitHub](https://github.com/open-mmlab/mmdetection3d) — 3D Object Detection 통합 프레임워크

> **실습**: [BEV Projection 시각화](https://alexjunholee.github.io/robotics-practice/app.html#bev_projection)
> 카메라 이미지를 BEV로 변환하는 과정을 인터랙티브하게 확인하며, BEV 기반 3D 감지의 원리를 이해할 수 있다.

## 13.4 3D Reconstruction

여러 시점의 이미지나 깊이 정보로 3D 모델을 만든다. 로봇은 이 모델을 환경 지도로 쓴다.

### 13.4.1 Structure from Motion (SfM)

여러 장의 2D 사진으로 카메라 포즈와 3D 구조를 함께 복원한다. 스마트폰 사진 여러 장으로 건물의 3D 모델을 만들 때 이 방법을 쓴다. NeRF나 3D Gaussian Splatting에 넣을 카메라 포즈도 SfM으로 구한다.

**파이프라인**:
1. 특징점 추출 및 매칭
2. 초기 두 뷰로 삼각측량
3. 점진적 카메라 추가
4. Bundle Adjustment (BA)

Bundle Adjustment는 모든 카메라 포즈와 3D 점 위치를 동시에 최적화한다. Levenberg-Marquardt 같은 비선형 최소자승법으로 푼다. 변수가 수만에서 수십만 개에 이르기도 한다.

**도구**:
- COLMAP: 널리 쓰이는 공개 SfM/MVS 구현, GUI/CLI
- **OpenMVG**: 라이브러리 형태

```bash
# COLMAP 사용
colmap feature_extractor --database_path db.db --image_path ./images
colmap exhaustive_matcher --database_path db.db
mkdir -p sparse
colmap mapper --database_path db.db --image_path ./images --output_path ./sparse
```

> **추천 자료**
> - [COLMAP Documentation](https://colmap.github.io/) — 널리 쓰이는 공개 SfM/MVS 참조 구현
> - [Daniel Cremers — Multiple View Geometry (TUM)](https://www.youtube.com/playlist?list=PLTBdjV_4f-EJn6udZ34tht9EVIW7lbeo4) — 다중 뷰 기하학 핵심 강의
> - [Schönberger & Frahm, "Structure-from-Motion Revisited" (2016)](https://openaccess.thecvf.com/content_cvpr_2016/papers/Schonberger_Structure-From-Motion_Revisited_CVPR_2016_paper.pdf) — COLMAP 논문

### 13.4.2 Multi-View Stereo (MVS)

SfM은 카메라 포즈와 듬성듬성한(sparse) 3D 점을 복원한다. MVS는 그 카메라 포즈로 조밀한(dense) 3D 포인트 클라우드를 만든다. SfM → MVS → Mesh 생성이 전형적인 3D 복원 파이프라인이다.

**도구**: COLMAP (dense reconstruction), OpenMVS

### 13.4.3 Volumetric Reconstruction

**TSDF (Truncated Signed Distance Function)**:
- 공간을 복셀로 나누고 각 복셀에 표면까지의 거리 저장
- 여러 뷰 통합
- Marching Cubes로 mesh 추출

TSDF는 각 복셀에 가장 가까운 표면까지의 부호 있는 거리(signed distance)를 저장한다. 양수는 표면 바깥, 음수는 표면 안쪽을 뜻한다. 여러 뷰에서 관측한 깊이를 가중 평균하면 노이즈가 줄어들고, 부호가 바뀌는 지점에서 표면을 추출할 수 있다.

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

> **추천 자료**
> - [Open3D — TSDF Integration Tutorial](https://www.open3d.org/docs/release/tutorial/pipelines/rgbd_integration.html) — TSDF 실습 코드
> - [Curless & Levoy, "A Volumetric Method for Building Complex Models from Range Images" (1996)](https://graphics.stanford.edu/papers/volrange/volrange.pdf) — TSDF 원 논문 (고전이지만 읽어볼 가치가 있다)

## 13.5 Neural Rendering

신경망이나 학습으로 정하는 파라미터로 3D 장면을 표현하고 렌더링한다. mesh나 point cloud로는 반사, 투명체, 가는 구조를 사실적으로 표현하기 어렵다. Neural Rendering은 장면을 학습된 함수로 표현해 이런 효과를 재현한다. SLAM과 결합해 로봇이 움직이는 동안 지도를 만드는 데도 쓴다.

### 13.5.1 NeRF (Neural Radiance Fields)

**개념**: 3D 장면을 continuous 함수로 표현

```
F: (x, y, z, θ, φ) → (r, g, b, σ)
- 위치 (x, y, z)와 시점 방향 (θ, φ)
- 색상 (r, g, b)과 밀도 (σ) 출력
```

NeRF는 3D 공간의 각 점이 어느 방향에서 어떤 색으로 보이고 밀도가 얼마인지 신경망으로 학습한다. 학습이 끝나면 찍지 않은 카메라 위치에서 본 영상도 합성한다(novel view synthesis).

렌더링: 광선을 따라 색상과 밀도를 적분 (volume rendering)

**장점**:
- 사실적인 novel view synthesis
- 반사, 투명 등 복잡한 효과 처리

**단점**:
- 학습 시간 오래 걸림
- 동적 장면 어려움

**발전**:
- Instant-NGP: 해시 인코딩으로 빠른 학습 (수 분)
- Mip-NeRF: 안티앨리어싱
- Block-NeRF: 대규모 장면

> **추천 자료**
> - [Mildenhall et al., "NeRF: Representing Scenes as Neural Radiance Fields" (ECCV 2020)](https://arxiv.org/abs/2003.08934) — NeRF 원 논문
> - [NeRFStudio Documentation](https://docs.nerf.studio/) — NeRF 실험용 통합 프레임워크
> - [Yannic Kilcher — NeRF Explained](https://www.youtube.com/watch?v=CRlN-cYFxTk) — NeRF의 표현 방식을 직관적으로 설명
> - [Jon Barron — Understanding and Extending Neural Radiance Fields](https://www.youtube.com/watch?v=HfJpQCBTqZs) — MIT Vision & Graphics Seminar 강연. 연사는 NeRF 공저자다

### 13.5.2 3D Gaussian Splatting (3DGS)

원래 NeRF는 한 프레임을 렌더링하는 데 수 초씩 걸리기도 한다. 3DGS 논문은 1080p 해상도에서 30 FPS 이상으로 실시간 렌더링했다고 보고했다. 이 속도 덕분에 SLAM과 온라인 매핑에도 쓴다.

개념: 장면을 수백만 개의 3D Gaussian으로 표현

각 Gaussian:
- 위치 (mean)
- 공분산 (모양/크기/방향)
- 색상 (Spherical Harmonics)
- 불투명도

3×3 공분산 행렬의 고유벡터가 타원체의 축 방향이다. 축 길이는 고유값의 제곱근에 비례한다. 3DGS는 각 Gaussian의 모양과 크기를 이 공분산으로 표현한다.

**렌더링**: Gaussian을 이미지에 투영 (splatting)

**장점**:
- 실시간 렌더링 (1080p에서 30 FPS 이상)
- 학습이 비교적 빠름 (원 논문 Mip-NeRF360 기준 7K iteration 약 6분, 최고 품질 30K iteration 약 42분)
- 명시적 표현 (편집 용이)

**응용**:
- SLAM: SplaTAM, Gaussian Splatting SLAM
- Mapping: 대규모 환경 표현
- Dynamic scenes: 동적 장면 확장

```python
# 3DGS 기본 개념 (pseudo-code)
# 각 Gaussian: position, covariance, color, opacity
# 렌더링: 카메라 뷰로 투영하여 이미지 생성
```

**3DGS + SLAM**:

3DGS-SLAM은 3D Gaussian으로 사진처럼 보이는 맵을 만든다. 앞의 실시간 수치는 학습이 끝난 장면을 렌더링할 때의 속도다. tracking과 mapping 최적화까지 넣은 전체 속도는 훨씬 낮다. MonoGS 논문은 시스템이 3 fps로 돈다고 보고했다.

- SplaTAM (CVPR 2024): RGB-D 카메라 입력으로 3DGS 기반 dense SLAM을 한다. Tracking(카메라 포즈 추정)과 Mapping(Gaussian 추가·갱신)을 번갈아 돌린다. 논문은 카메라 포즈 추정, 지도 복원, 새 시점 합성에서 기존 방법보다 최대 2배 좋은 성능을 보고했다.
- MonoGS (CVPR 2024): 단안(monocular) 카메라만으로도 3DGS 기반 SLAM을 한다. RGB-D 입력도 지원한다.
- Gaussian-SLAM (arXiv 2023 프리프린트): RGB-D 입력을 서브맵(sub-map)으로 나눠 관리해 더 큰 장면을 다룬다.

이렇게 만든 맵은 디지털 트윈이나 AR/VR 콘텐츠에도 쓴다.

> **추천 자료**
> - [Kerbl et al., "3D Gaussian Splatting for Real-Time Radiance Field Rendering" (SIGGRAPH 2023)](https://arxiv.org/abs/2308.04079) — 3DGS 원 논문
> - [Huang et al., "2D Gaussian Splatting for Geometrically Accurate Radiance Fields" (SIGGRAPH 2024, arXiv:2403.17888)](https://arxiv.org/abs/2403.17888) — 2D Gaussian으로 표면 복원 품질 향상
> - [Keetha et al., "SplaTAM: Splat, Track & Map 3D Gaussians for Dense RGB-D SLAM" (CVPR 2024)](https://arxiv.org/abs/2312.02126) — 3DGS + SLAM의 대표작
> - [Matsuki et al., "Gaussian Splatting SLAM" (CVPR 2024)](https://arxiv.org/abs/2312.06741) — MonoGS 논문
> - [Wang et al., "DUSt3R: Geometric 3D Vision Made Easy" (CVPR 2024, arXiv:2312.14132)](https://arxiv.org/abs/2312.14132) — 카메라 내부/외부 파라미터 없이 이미지 쌍에서 dense 3D를 복원한다.
> - [Leroy et al., "Grounding Image Matching in 3D with MASt3R" (ECCV 2024, arXiv:2406.09756)](https://arxiv.org/abs/2406.09756) — DUSt3R에 local feature matching을 붙여 3D 복원과 정밀한 대응점을 함께 낸다
> - [NeRFStudio Documentation](https://docs.nerf.studio/) — NeRF/3DGS 실험 통합 프레임워크
> - [3DGS Original Implementation (GitHub)](https://github.com/graphdeco-inria/gaussian-splatting) — 공식 코드

> **실습**: [3D Gaussian Splatting 시각화](https://alexjunholee.github.io/robotics-practice/app.html#gaussian_splatting)
> 3D Gaussian의 위치, 공분산, 색상을 조작하며 splatting 렌더링 과정을 인터랙티브하게 이해할 수 있다.

## 13.6 심화: Neural Implicit Representations

NeRF는 density field로 volume rendering을 한다. density field에서는 표면 위치가 흐릿하게 퍼져 있어 깨끗한 surface를 뽑기 어렵다. 물체를 잡거나 충돌을 판단하려면 정확한 surface가 필요하므로, 부호 있는 거리 함수(Signed Distance Function, SDF)로 장면을 표현한다.

**SDF (Signed Distance Function)**

공간의 각 점 `x`에서 가장 가까운 표면까지 부호 있는 거리를 반환하는 함수다.

```
f(x) > 0  : 표면 바깥
f(x) < 0  : 표면 안쪽
f(x) = 0  : 표면 위 (zero level set)
```

SDF는 미분 가능한 점에서 gradient의 크기가 1이다 (Eikonal equation).

```
||∇f(x)|| = 1
```

거리 함수는 이 조건을 만족한다. Neural network로 SDF를 학습할 때 이 조건을 정규화 항(regularization)으로 넣는데, 이를 Eikonal loss라 한다.

```
L_eikonal = E_x[ (||∇f_θ(x)|| - 1)^2 ]
```

**DeepSDF**

DeepSDF(2019)는 SDF를 신경망으로 학습한 초기 연구다. 인코더 없이 decoder만 쓰고, 물체마다 형상을 latent code `z` 하나로 나타낸다.

```
f_θ(z, x) → SDF value
```

새 물체의 `z`는 test-time optimization으로 추정한다.

**NeuS**

NeuS(2021)는 SDF 값을 density로 바꾸는 함수를 두어, volume rendering으로 SDF를 학습한다. 렌더링 품질은 NeRF에 가깝고 surface는 SDF의 0-level set에서 바로 뽑는다.

```
density ρ(t) = max(-dΦ_s(f(r(t)))/dt, 0) / Φ_s(f(r(t))),   r(t) = o + t·d 는 광선 위의 점
```

여기서 `Φ_s`는 learnable parameter `s`가 제어하는 시그모이드다. 그 도함수로 정의되는 S-density의 표준편차가 `1/s`이므로, 학습이 수렴하면서 `s`가 커지고 density가 surface 근처로 좁아진다.

**VolSDF**

VolSDF(2021)는 density를 SDF에 Laplace 분포 CDF를 적용한 값으로 정의한다.

```
σ(x) = (1/β) · Ψ_β(-f(x))
```

`β`가 줄어들수록 density가 surface에 집중된다.

**Surface 추출**

학습된 SDF에서 `f(x) = 0`인 iso-surface를 mesh로 바꿀 때는 Marching Cubes 알고리즘을 쓴다. 공간을 격자로 나누고, 각 격자 꼭짓점에서 SDF 부호를 확인해 surface가 지나가는 위치를 보간으로 결정한다.

**비교표**

| 표현 | 장점 | 단점 | 예시 |
|------|------|------|------|
| NeRF (density) | 렌더링 품질 높음 | surface 추출 어려움 | Instant-NGP |
| SDF (neural) | 깨끗한 surface | 학습 어려움 | NeuS, VolSDF |
| 3DGS (explicit) | 실시간 렌더링 | 메모리 사용량 | Gaussian Splatting |
| Occupancy | 이진 분류로 단순 | 표면 디테일 한계 | ConvONet |

> **추천 자료**
> - [Wang et al., "NeuS: Learning Neural Implicit Surfaces by Volume Rendering" (NeurIPS 2021)](https://arxiv.org/abs/2106.10689) — NeuS 원 논문
> - [Yariv et al., "Volume Rendering of Neural Implicit Surfaces" (NeurIPS 2021)](https://arxiv.org/abs/2106.12052) — VolSDF 논문
> - [Park et al., "DeepSDF: Learning Continuous Signed Distance Functions for Shape Representation" (CVPR 2019)](https://arxiv.org/abs/1901.05103) — DeepSDF 원 논문
> - [Mescheder et al., "Occupancy Networks" (CVPR 2019)](https://arxiv.org/abs/1812.03828) — Occupancy 기반 접근의 대표작

## 13.7 심화: Differentiable Rendering

NeRF, 3DGS, NeuS는 렌더링 과정을 미분 가능하게 만들고, 렌더링 결과와 실제 이미지의 차이로 3D 표현을 최적화한다. 관측 이미지를 가장 잘 재현하는 장면 표현을 찾는 방식을 analysis-by-synthesis라 한다.

**Volume Rendering Equation**

NeRF 계열이 쓰는 기본 렌더링 공식이다. 카메라에서 발사한 ray `r(t) = o + td` 위의 색상을 적분한다.

```
C(r) = ∫_{t_n}^{t_f} T(t) · σ(t) · c(t) dt

where T(t) = exp( -∫_{t_n}^{t} σ(s) ds )
```

- `t_n`, `t_f`: ray 위 적분 구간의 가까운 끝과 먼 끝
- `σ(t)`: 위치 `t`의 density (부피 밀도, 길이의 역수 차원)
- `c(t)`: 위치 `t`의 색상 (RGB)
- `T(t)`: 누적 투과도 (ray가 `t`까지 도달할 확률)

구현에서는 ray 위의 N개 샘플 점으로 이 적분을 근사한다 (ray marching).

```
C(r) ≈ Σ_i T_i · α_i · c_i
where α_i = 1 - exp(-σ_i · δ_i),  T_i = Π_{j<i} (1 - α_j)
```

`δ_i = t_{i+1} - t_i`는 이웃한 두 샘플 점 사이의 거리다.

**3DGS의 Rasterization 기반 렌더링**

3DGS는 위의 ray marching 대신 각 Gaussian을 이미지 평면에 투영(splatting)하는 rasterization 방식을 쓴다. 각 픽셀의 색상은 해당 픽셀에 영향을 주는 Gaussian들의 가중 합이다.

```
C(p) = Σ_i c_i · α_i · Π_{j<i} (1 - α_j)
```

수식 구조는 volume rendering과 비슷하다. 3DGS는 ray를 따라 샘플링하는 대신 Gaussian을 깊이 순으로 정렬해 차례로 합성한다. 화면을 작은 타일로 나누고 타일마다 이 정렬과 합성을 GPU에서 병렬로 처리하므로 실시간으로 렌더링된다.

**Differentiable Rasterization 라이브러리**

mesh 기반 3D 표현을 미분 가능하게 렌더링하는 도구들이다.

- PyTorch3D (Meta): 미분 가능한 mesh renderer와 point cloud renderer
- nvdiffrast (NVIDIA): CUDA 기반 고성능 differentiable rasterizer
- Kaolin (NVIDIA): 3D 딥러닝 전반을 위한 라이브러리

**Analysis-by-Synthesis 파이프라인**

```
1. 가설적 3D 장면을 설정 (NeRF, 3DGS, mesh 등)
2. 해당 장면을 카메라 시점에서 렌더링 → 예측 이미지
3. 실제 관측 이미지와 비교 → loss 계산
4. loss를 역전파하여 3D 장면 파라미터 업데이트
5. 반복
```

3D 표현이 어떤 형태든 supervision은 이미지에서만 온다.

**SLAM과의 연결**

Differentiable rendering을 SLAM에 적용하면 tracking과 mapping을 모두 렌더링 loss로 최적화한다.

- iMAP, NICE-SLAM: 신경망 implicit 표현을 맵으로 쓴다. 새 프레임이 들어오면 현재 맵을 렌더링해 포즈를 추정(tracking)하고 맵을 갱신(mapping)한다.
- 3DGS-SLAM (SplaTAM, MonoGS 등): 3DGS를 맵으로 쓴다. SplaTAM 논문에 따르면 rasterization으로 렌더링이 빨라서, implicit 표현을 쓰는 SLAM보다 훨씬 많은 픽셀로 loss를 계산하면서도 프레임당 처리 시간은 비슷하다.

```
[새 프레임] → [현재 맵 렌더링] → [렌더링 vs 실제 비교]
                                      ↓
                              [포즈 업데이트 (tracking)]
                              [맵 업데이트 (mapping)]
```

> **추천 자료**
> - [Tewari et al., "Advances in Neural Rendering" (EUROGRAPHICS 2022 STAR)](https://arxiv.org/abs/2111.05849) — Differentiable rendering 서베이
> - [Ravi et al., "Accelerating 3D Deep Learning with PyTorch3D" (2020)](https://arxiv.org/abs/2007.08501) — PyTorch3D 논문
> - [Laine et al., "Modular Primitives for High-Performance Differentiable Rendering" (2020)](https://arxiv.org/abs/2011.03277) — nvdiffrast 논문

## 13.8 심화: 3D Scene Graph

"주방에 있는 빨간 컵을 가져와"를 해내려면 로봇은 주방의 위치, 빨간 컵이라는 물체, 컵이 주방 안에 있다는 관계를 알아야 한다. 포인트 클라우드나 mesh에는 이 정보가 없다. 3D Scene Graph는 물체와 장소를 노드로, 그 관계를 엣지로 두어 환경을 표현한다.

**구조**

- **노드(Node)**: 물체, 방, 건물 등 — 계층적(hierarchical) 구조
  - 건물 → 층 → 방 → 물체
  - 노드마다 3D 위치, 바운딩 박스, 의미 라벨이 붙음
- **엣지(Edge)**: 노드 간 관계
  - "위에 있다" (on), "안에 있다" (in), "가까이 있다" (near), "지지한다" (support) 등

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

Hydra는 MIT에서 개발한 실시간 3D scene graph 구축 시스템이다. 로봇이 이동하는 동안 센서 입력으로 계층적 scene graph를 점진적으로 만든다.

파이프라인:
1. Metric-semantic mesh 구축 (TSDF + semantic segmentation)
2. 물체 노드 추출 및 관계 설정
3. 방(room) 단위 분할 (free-space clustering)
4. 계층 구조 연결

**ConceptGraphs**

ConceptGraphs는 Foundation model(CLIP, LLM)로 open-vocabulary scene graph를 만든다.

이전 scene graph는 의자, 테이블처럼 미리 정한 카테고리로 노드를 만들었다. ConceptGraphs는 CLIP으로 자연어 쿼리에 맞는 물체를 찾고, LLM으로 물체 간 관계를 추론한다.

```
1. RGB-D 프레임에서 class-agnostic segmentation으로 물체 후보 분할
2. CLIP feature로 물체 임베딩 추출
3. 3D 공간에서 동일 물체 병합 (multi-view association)
4. LLM으로 물체 간 관계 추론
5. Scene graph 구축
```

"빨간 컵"처럼 학습 카테고리에 없는 쿼리도 넣을 수 있다.

**왜 필요한가?**

| 표현 | "주방의 빨간 컵을 가져와"를 처리할 수 있나 | 이유 |
|------|------|------|
| Point Cloud | 불가 | 의미 정보 없음 |
| Semantic Map | 부분적 | "컵"은 찾지만 "주방에 있는"이라는 관계 처리 어려움 |
| 3D Scene Graph | 가능 | 물체 + 관계 + 계층 모두 표현 |

task planning이나 자연어 기반 내비게이션에서는 scene graph를 3D 맵과 고수준 추론 사이의 중간 표현으로 쓴다.

> **추천 자료**
> - [Hughes et al., "Hydra: A Real-time Spatial Perception System for 3D Scene Graph Construction and Optimization" (RSS 2022)](https://arxiv.org/abs/2201.13360) — Hydra 원 논문
> - [Gu et al., "ConceptGraphs: Open-Vocabulary 3D Scene Graphs for Perception and Planning" (2023)](https://arxiv.org/abs/2309.16650) — ConceptGraphs 논문
> - [Rosinol et al., "3D Dynamic Scene Graphs: Actionable Spatial Perception with Places, Objects, and Humans" (RSS 2020)](https://arxiv.org/abs/2002.06289) — Dynamic Scene Graph 개념 제시
> - [Armeni et al., "3D Scene Graph: A Structure for Unified Semantics, 3D Space, and Camera" (ICCV 2019)](https://arxiv.org/abs/1910.02527) — 3D Scene Graph 초기 연구

> **기술 흐름: 3D Vision**
> - ~2010: ICP 정합(1992), TSDF 기반 볼류메트릭 복원(1996).
> - 2010~: Kinect 출시(2010). PCL 공개(Rusu & Cousins, ICRA 2011). KinectFusion(2011)이 TSDF와 ICP로 실시간 RGB-D 3D 복원을 보였다.
> - 2017~: PointNet/PointNet++가 raw 포인트를 신경망에 직접 넣었다. VoxelNet(2018), PointPillars(2019) 등 LiDAR 3D Object Detection이 이어졌다.
> - 2020~: NeRF 발표. 여러 장의 사진으로 새 시점 영상을 사진처럼 사실적으로 합성했다. Mip-NeRF(2021), Instant-NGP(2022) 등이 뒤따랐다.
> - 2023~: 3D Gaussian Splatting이 명시적 Gaussian 표현으로 실시간 렌더링을 보였다. BEVFusion(ICRA 2023)은 카메라와 LiDAR 특징을 BEV에서 융합했다.
> - 2024~: 3DGS를 맵으로 쓰는 SLAM 발표(SplaTAM·MonoGS는 CVPR 2024, Gaussian-SLAM은 arXiv 2023 프리프린트).
> - 도구: NeRFStudio에서는 NeRF와 3DGS의 표현과 렌더링 속도를 같은 데이터로 비교할 수 있다.
