# Ch.13 — 3D 비전 (3D Vision)


로봇이 2D 이미지만으로는 "저 물체가 얼마나 멀리 있는지", "저 벽 뒤에 뭐가 있는지"를 알기 어렵다. 3D 비전은 로봇에게 공간 감각을 부여하는 분야다. 포인트 클라우드 처리, 3D 물체 감지, 장면 복원이 여기에 속한다. SLAM도, 로봇 조작(manipulation)도 이 내용 없이는 온전히 이해하기 어렵다.

## 13.1 Point Cloud 기초

LiDAR나 깊이 카메라에서 나오는 데이터가 바로 포인트 클라우드이다. 이미지는 픽셀 격자에 정렬된 2D 데이터인 반면, 포인트 클라우드는 3D 공간에 불규칙하게 흩어진 점들이다. 이 비정형 데이터를 어떻게 다루는지가 3D 비전의 출발점이다.

**Point Cloud (포인트 클라우드)**는 3D 공간의 점들의 집합이다. 각 점은 최소한 (x, y, z) 좌표를 가지며, 추가로 색상(RGB), 반사도(intensity), 법선(normal) 등의 속성을 가질 수 있다.

### 13.1.1 데이터 구조 및 포맷

**일반적인 구조**:

```
Point: [x, y, z, r, g, b, intensity, ...]
Point Cloud: N × D 행렬 (N개 점, D차원 속성)
```

선형대수로 생각하면, 포인트 클라우드는 그냥 N×D 행렬이다. N은 수만~수백만 개의 점, D는 각 점의 속성 차원이다. 변환(회전, 이동)은 각 점에 4×4 변환 행렬을 곱하는 것과 같다.

**주요 파일 포맷**:
| 포맷 | 특징 |
|---|---|
| **PCD** | PCL 표준, ASCII/Binary |
| **PLY** | 다목적, mesh도 지원 |
| **LAS/LAZ** | 지리정보 표준, LAZ는 압축 |
| **XYZ** | 단순 텍스트, 좌표만 |
| **BIN** | KITTI 등에서 사용, 바이너리 |

> **추천 자료**
> - [Open3D Documentation](http://www.open3d.org/docs/release/) — 포인트 클라우드 처리의 현대적 라이브러리
> - [PCL (Point Cloud Library) Tutorials](https://pcl.readthedocs.io/projects/tutorials/en/latest/) — 포인트 클라우드 처리의 고전 라이브러리

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
import open3d as o3d

# 포인트 클라우드 읽기
pcd = o3d.io.read_point_cloud("cloud.pcd")

# 시각화
o3d.visualization.draw_geometries([pcd])

# NumPy 변환
points = np.asarray(pcd.points)  # (N, 3)
```

실제로 작업할 때는 Open3D가 Python에서 바로 쓸 수 있어서 프로토타이핑에 좋고, PCL은 C++ ROS 노드를 만들 때 주로 쓴다. 처음 시작한다면 Open3D부터 보면 된다.

> **추천 자료**
> - [Open3D Getting Started](http://www.open3d.org/docs/release/getting_started.html) — Python으로 포인트 클라우드 다루기 입문
> - [PCL Tutorials — Basic Usage](https://pcl.readthedocs.io/projects/tutorials/en/latest/#basic-usage) — C++ 기반 포인트 클라우드 처리
> - [Open3D YouTube Channel](https://www.youtube.com/@Open3D) — 시각화 및 처리 튜토리얼

## 13.2 Point Cloud 처리

### 13.2.1 필터링 (Filtering)

원시 포인트 클라우드는 노이즈가 많고 점의 밀도가 불균일하다. 그대로 쓰면 이후 알고리즘(정합, 세그멘테이션 등)이 느려지거나 결과가 나빠진다. 필터링은 모든 포인트 클라우드 파이프라인의 첫 번째 단계이다.

**Voxel Grid Downsampling**:
공간을 격자로 나누고 각 격자 내 점들을 하나로 축소한다.

```python
# Open3D
voxel_pcd = pcd.voxel_down_sample(voxel_size=0.05)  # 5cm 격자
```

**Statistical Outlier Removal**:
이웃과의 거리 통계를 기반으로 이상치를 제거한다.

```python
# 이상치 제거
cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
filtered_pcd = pcd.select_by_index(ind)
```

**Radius Outlier Removal**:
주어진 반경 내 이웃 수가 부족한 점을 제거한다.

> **추천 자료**
> - [Open3D — Point Cloud Filtering Tutorial](http://www.open3d.org/docs/release/tutorial/geometry/pointcloud.html) — Voxel downsampling, outlier removal 예제
> - [PCL — Filtering Tutorial](https://pcl.readthedocs.io/projects/tutorials/en/latest/passthrough.html) — PCL 기반 필터링

### 13.2.2 Normal Estimation

각 점의 표면 법선 벡터를 추정한다. 3D 처리 파이프라인 전반의 전처리 단계이다.

ICP (정합), 표면 재구성(reconstruction), 조명 계산 등 거의 모든 3D 처리에서 법선 벡터를 요구한다. 법선이 없으면 "이 점이 평면의 일부인지 모서리의 일부인지"를 알 수 없다.

```python
# 법선 추정
pcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
)
```

내부적으로는 각 점의 이웃 k개를 모아 공분산 행렬을 구하고, 그 최소 고유값에 대응하는 고유벡터가 법선이 된다. 선형대수 시간에 배운 PCA(주성분 분석)와 정확히 같은 원리이다.

### 13.2.3 Registration (정합)

두 포인트 클라우드를 정렬하는 과정이다. SLAM에서 연속 프레임을 이어 붙이거나, 여러 뷰에서 스캔한 데이터를 합칠 때 필요하다.

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

ICP의 직관: "두 포인트 클라우드에서 가장 가까운 점 쌍을 찾고, 그 쌍들이 최대한 겹치도록 회전+이동 변환을 구한다. 한 번에 완벽하지 않으니 이 과정을 반복한다." 선형대수적으로는 SVD(특이값 분해)를 이용해 최적의 회전 행렬 R과 이동 벡터 t를 구하는 것이다.

**Point-to-Plane ICP**: 점과 평면 거리 최소화 (더 정확)

**GICP (Generalized ICP)**: 점 분포 고려

**NDT (Normal Distributions Transform)**: 공간을 셀로 나누고 각 셀의 정규분포 매칭

**Feature-based Registration**:
- FPFH, SHOT 등 특징 추출
- RANSAC으로 초기 정합
- ICP로 정밀화

> **추천 자료**
> - [Open3D — ICP Registration Tutorial](http://www.open3d.org/docs/release/tutorial/pipelines/icp_registration.html) — ICP 실습 코드
> - [Open3D — Global Registration (RANSAC + Feature)](http://www.open3d.org/docs/release/tutorial/pipelines/global_registration.html) — Feature 기반 정합
> - [Cyrill Stachniss — ICP & Point Cloud Registration](https://www.youtube.com/watch?v=dhzLQfDBx2Q) — ICP 알고리즘의 직관적 설명

> **실습**: [ICP 2D 단계별 시각화](https://alexjunholee.github.io/robotics-practice/app.html#icp_steps) | [ICP 3D](https://alexjunholee.github.io/robotics-practice/app.html#icp_3d)
> ICP 알고리즘이 두 포인트 클라우드를 정합하는 과정을 iteration별로 확인하고, 2D와 3D 환경에서 수렴 과정을 비교할 수 있다.

## 13.3 3D Object Detection

포인트 클라우드에서 3D bounding box를 예측하는 분야다. 자율주행에서 "저 차가 어디 있고 얼마나 큰지"를 파악하는 핵심 기술이다.

### 13.3.1 Point-based Methods

PointNet의 핵심 아이디어는 포인트 클라우드를 복셀이나 이미지로 변환하지 않고 raw 포인트에 직접 딥러닝을 적용한다는 것이다. 이전에는 "불규칙한 점들에 어떻게 CNN을 쓰지?"라는 질문에 답이 없었는데, PointNet이 이를 해결했다.

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

PointNet의 핵심 아이디어: 포인트 클라우드의 점 순서가 바뀌어도 결과가 같아야 한다(permutation invariance). 이를 위해 각 점을 독립적으로 MLP에 통과시킨 뒤 max pooling으로 집계한다. 수학적으로 f({x1, ..., xn}) = g(MAX(h(x1), ..., h(xn))) 형태이다.

### 13.3.2 Voxel-based Methods

**VoxelNet (2018)**:
- 포인트 클라우드를 3D 복셀로 변환
- Voxel Feature Encoding
- 3D CNN으로 처리

**SECOND (Sparsely Embedded Convolutional Detection)**:
- Sparse Convolution 사용
- VoxelNet 대비 훨씬 빠름
- 널리 사용되는 베이스라인

**PointPillars (2019)**:
- Pillar (수직 기둥) 단위 처리
- 2D CNN으로 변환하여 빠른 속도
- 실시간 가능

PointPillars의 핵심 아이디어: 3D 공간을 수직 기둥(pillar)으로 나누면, 각 pillar 안의 점들을 하나의 특징 벡터로 압축한 뒤, 이를 2D 이미지처럼 배열할 수 있다. 잘 검증된 2D CNN을 그대로 쓸 수 있어서, 3D CNN보다 훨씬 빠르다.

### 13.3.3 Multi-modal Methods

여러 센서를 결합하면 각 센서의 단점을 서로 보완할 수 있다. 카메라는 색상과 텍스처 정보가 풍부하지만 깊이가 없고, LiDAR는 정확한 3D 정보가 있지만 텍스처가 없다. 이 둘을 어떻게 합치느냐가 핵심이다.

**BEVFusion**:
- Camera + LiDAR 융합
- 조감도(Bird's Eye View, BEV) 공간에서 통합

**TransFusion**:
- Transformer 기반 융합
- Query 기반 detection

> **추천 자료**
> - [Qi et al., "PointNet: Deep Learning on Point Sets" (2017)](https://arxiv.org/abs/1612.00593) — 3D 딥러닝의 시작점
> - [Lang et al., "PointPillars" (2019)](https://arxiv.org/abs/1812.05784) — 실시간 3D 감지
> - [Liu et al., "BEVFusion" (2023)](https://arxiv.org/abs/2205.13542) — 멀티모달 융합의 대표작
> - [MMDetection3D GitHub](https://github.com/open-mmlab/mmdetection3d) — 3D Object Detection 통합 프레임워크

> **실습**: [BEV Projection 시각화](https://alexjunholee.github.io/robotics-practice/app.html#bev_projection)
> 카메라 이미지를 BEV로 변환하는 과정을 인터랙티브하게 확인하며, BEV 기반 3D 감지의 원리를 이해할 수 있다.

## 13.4 3D Reconstruction

여러 뷰 또는 깊이 정보로부터 3D 모델을 생성한다. 로봇이 환경을 3D로 "기억"하려면 이 기술이 필요하다.

### 13.4.1 Structure from Motion (SfM)

여러 장의 2D 사진만으로 3D 구조를 복원할 수 있다. 스마트폰 사진 몇 장으로 건물의 3D 모델을 만들 수 있다는 얘기다. NeRF나 3D Gaussian Splatting의 입력 데이터(카메라 포즈)를 만드는 전처리 단계이기도 하다.

여러 이미지에서 카메라 포즈와 3D 구조를 동시에 복원한다.

**파이프라인**:
1. 특징점 추출 및 매칭
2. 초기 두 뷰로 삼각측량
3. 점진적 카메라 추가
4. Bundle Adjustment (BA)

Bundle Adjustment는 "모든 카메라 포즈와 3D 점 위치를 동시에 최적화"하는 것이다. 비선형 최소자승법(Levenberg-Marquardt 등)을 사용하며, 변수가 수만~수십만 개가 될 수 있다. 선형대수에서 배운 최소자승법의 대규모 비선형 확장이라고 보면 된다.

**도구**:
- **COLMAP**: 분야 사실상 표준, GUI/CLI
- **OpenMVG**: 라이브러리 형태

```bash
# COLMAP 사용
colmap feature_extractor --database_path db.db --image_path ./images
colmap exhaustive_matcher --database_path db.db
colmap mapper --database_path db.db --image_path ./images --output_path ./sparse
```

> **추천 자료**
> - [COLMAP Documentation](https://colmap.github.io/) — SfM/MVS의 사실상 표준 도구
> - [Daniel Cremers — Multiple View Geometry (TUM)](https://www.youtube.com/playlist?list=PLTBdjV_4f-EJn6udZ34tht9EVIW7lbeo4) — 다중 뷰 기하학 핵심 강의
> - [Schönberger & Frahm, "Structure-from-Motion Revisited" (2016)](https://openaccess.thecvf.com/content_cvpr_2016/papers/Schonberger_Structure-From-Motion_Revisited_CVPR_2016_paper.pdf) — COLMAP 논문

### 13.4.2 Multi-View Stereo (MVS)

SfM 결과를 기반으로 dense 포인트 클라우드를 생성한다.

SfM이 "카메라가 어디 있었는지"와 "sparse한 3D 점들"을 복원한다면, MVS는 그 카메라 포즈를 이용해 조밀한(dense) 3D 포인트 클라우드를 만든다. SfM → MVS → Mesh 생성이 전형적인 3D 복원 파이프라인이다.

**도구**: COLMAP (dense reconstruction), OpenMVS

### 13.4.3 Volumetric Reconstruction

**TSDF (Truncated Signed Distance Function)**:
- 공간을 복셀로 나누고 각 복셀에 표면까지의 거리 저장
- 여러 뷰 통합
- Marching Cubes로 mesh 추출

TSDF의 핵심 아이디어: 각 복셀에 "가장 가까운 표면까지의 부호 있는 거리(signed distance)"를 저장한다. 양수면 표면 바깥, 음수면 표면 안쪽이다. 여러 뷰에서 관측한 깊이를 가중 평균하면, 노이즈가 줄어들고 깨끗한 표면을 얻는다. 부호가 바뀌는 지점(0을 지나는 곳)이 바로 표면이다.

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

> **추천 자료**
> - [Open3D — TSDF Integration Tutorial](http://www.open3d.org/docs/release/tutorial/pipelines/rgbd_integration.html) — TSDF 실습 코드
> - [Curless & Levoy, "A Volumetric Method for Building Complex Models from Range Images" (1996)](https://graphics.stanford.edu/papers/volrange/volrange.pdf) — TSDF 원 논문 (고전이지만 읽어볼 가치가 있다)

## 13.5 Neural Rendering

딥러닝을 이용한 새로운 3D 표현 및 렌더링 방식이다. 기존 방법(mesh, point cloud)은 복잡한 장면(반사, 투명체, 가는 구조)을 표현하는 데 한계가 있었다. Neural Rendering은 장면을 학습 가능한 함수로 표현해서, 이런 복잡한 효과를 자연스럽게 처리한다. 최근에는 SLAM과 결합되어 실시간 매핑에까지 활용 범위가 넓어지고 있다.

### 13.5.1 NeRF (Neural Radiance Fields)

**개념**: 3D 장면을 continuous 함수로 표현

```
F: (x, y, z, θ, φ) → (r, g, b, σ)
- 위치 (x, y, z)와 시점 방향 (θ, φ)
- 색상 (r, g, b)과 밀도 (σ) 출력
```

직관적으로 설명하면: NeRF는 "3D 공간의 모든 점에 대해, 어떤 방향에서 보면 어떤 색과 밀도를 가지는지"를 신경망으로 학습하는 것이다. 학습이 끝나면 어떤 카메라 위치에서든 새로운 뷰를 합성(novel view synthesis)할 수 있다.

**렌더링**: 광선을 따라 색상과 밀도를 적분 (volume rendering)

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
> - [Mildenhall et al., "NeRF: Representing Scenes as Neural Radiance Fields" (2020)](https://arxiv.org/abs/2003.08934) — NeRF 원 논문
> - [NeRFStudio Documentation](https://docs.nerf.studio/) — NeRF 실험을 쉽게 할 수 있는 통합 프레임워크. NeRF를 직접 돌려보고 싶다면 여기서 시작하자.
> - [Yannic Kilcher — NeRF Explained](https://www.youtube.com/watch?v=CRlN-cYFxTk) — NeRF의 핵심 아이디어를 직관적으로 설명
> - [Jon Barron — Understanding NeRF (ECCV 2022 Tutorial)](https://www.youtube.com/watch?v=HfJpQCBTqZs) — NeRF 저자 직강

### 13.5.2 3D Gaussian Splatting (3DGS)

3DGS가 빠르게 채택된 이유는 NeRF의 치명적 단점인 느린 렌더링 속도를 해결했기 때문이다. NeRF는 한 프레임을 렌더링하는 데 수 초가 걸리지만, 3DGS는 100 FPS 이상으로 실시간 렌더링이 가능하다. 이 속도 덕분에 SLAM, 실시간 매핑 등 로봇 응용에 직접 쓸 수 있게 되었다.

**개념**: 장면을 수백만 개의 3D Gaussian으로 표현

각 Gaussian:
- 위치 (mean)
- 공분산 (모양/크기/방향)
- 색상 (Spherical Harmonics)
- 불투명도

선형대수에서 배운 공분산 행렬을 떠올려보자. 3×3 공분산 행렬의 고유벡터가 타원체의 축 방향을, 고유값이 축의 길이를 결정한다. 3DGS는 이 개념을 그대로 활용해서, 각 Gaussian의 모양과 크기를 표현한다.

**렌더링**: Gaussian을 이미지에 투영 (splatting)

**장점**:
- NeRF 대비 실시간 렌더링 (100+ FPS)
- 빠른 학습 (수 분)
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

**3DGS + SLAM (최신 트렌드)**:

3D Gaussian Splatting이 SLAM과 결합되면서 Neural SLAM의 새로운 방향이 열리고 있다. 기존 SLAM이 sparse한 포인트 맵이나 복셀 맵을 만들었다면, 3DGS-SLAM은 포토리얼리스틱한 3D 맵을 실시간으로 구축한다.

- **SplaTAM (2024)**: RGB-D 카메라 입력으로 3DGS 기반 dense SLAM을 수행한다. Tracking(카메라 포즈 추정)과 Mapping(Gaussian 추가/업데이트)을 번갈아 수행하며, 기존 Neural SLAM 대비 렌더링 품질과 속도를 크게 높인다.
- **MonoGS (2024)**: 단안(monocular) 카메라만으로 3DGS 기반 SLAM을 수행한다. 깊이 센서 없이도 dense한 3D 맵을 구축할 수 있다.
- **Gaussian-SLAM (2024)**: 서브맵(sub-map) 기반으로 대규모 환경에서도 3DGS SLAM을 돌릴 수 있다.

로봇이 돌아다니면서 포토리얼리스틱한 3D 맵을 실시간으로 만들 수 있다면, 디지털 트윈이나 AR/VR 콘텐츠 생성 같은 응용이 열린다.

> **추천 자료**
> - [Kerbl et al., "3D Gaussian Splatting for Real-Time Radiance Field Rendering" (2023)](https://arxiv.org/abs/2308.14737) — 3DGS 원 논문
> - [Huang et al., "2D Gaussian Splatting for Geometrically Accurate Radiance Fields" (SIGGRAPH 2024, arXiv:2403.17888)](https://arxiv.org/abs/2403.17888) — 2D Gaussian으로 표면 복원 품질 향상
> - [Keetha et al., "SplaTAM: Splat, Track & Map 3D Gaussians for Dense RGB-D SLAM" (2024)](https://arxiv.org/abs/2312.02126) — 3DGS + SLAM의 대표작
> - [Matsuki et al., "Gaussian Splatting SLAM" (2024)](https://arxiv.org/abs/2312.06741) — MonoGS 논문
> - [Wang et al., "DUSt3R: Geometric 3D Vision Made Easy" (CVPR 2024, arXiv:2312.14132)](https://arxiv.org/abs/2312.14132) — 카메라 내부/외부 파라미터 없이 이미지 쌍에서 dense 3D 복원. 3D reconstruction 패러다임 전환
> - [Leroy et al., "MASt3R: Matching And Stereo 3D Reconstruction" (ECCV 2024, arXiv:2406.09756)](https://arxiv.org/abs/2406.09756) — DUSt3R에 local feature matching 추가. 복원 + 정밀 대응점 동시 제공
> - [NeRFStudio Documentation](https://docs.nerf.studio/) — NeRF/3DGS 실험 통합 프레임워크
> - [3DGS Original Implementation (GitHub)](https://github.com/graphdeco-inria/gaussian-splatting) — 공식 코드

> **실습**: [3D Gaussian Splatting 시각화](https://alexjunholee.github.io/robotics-practice/app.html#gaussian_splatting)
> 3D Gaussian의 위치, 공분산, 색상을 조작하며 splatting 렌더링 과정을 인터랙티브하게 이해할 수 있다.

## 13.6 심화: Neural Implicit Representations

*연구자가 되고 싶다면 여기서부터 읽어라.*

13.5에서 NeRF와 3DGS를 다뤘다. NeRF는 density field를 사용하여 volume rendering을 수행하지만, density에서 명확한 surface를 추출하기 어렵다는 한계가 있다. 로보틱스에서 물체를 잡거나 충돌을 판단하려면 정확한 surface가 필요하다. 여기서 부호 있는 거리 함수(Signed Distance Function, SDF) 기반 접근이 등장한다.

**SDF (Signed Distance Function)**

공간의 각 점 `x`에서 가장 가까운 표면까지 부호 있는 거리를 반환하는 함수다.

```
f(x) > 0  : 표면 바깥
f(x) < 0  : 표면 안쪽
f(x) = 0  : 표면 위 (zero level set)
```

SDF의 핵심 성질: gradient의 크기가 어디서나 1이다 (Eikonal equation).

```
||∇f(x)|| = 1
```

이 조건을 만족하는 함수만이 올바른 거리 함수이다. Neural network로 SDF를 학습할 때 이 조건을 정규화 항(regularization)으로 추가하는데, 이를 **Eikonal loss**라 한다.

```
L_eikonal = E_x[ (||∇f_θ(x)|| - 1)^2 ]
```

**DeepSDF**

SDF를 신경망으로 학습하는 초기 대표 연구다. Decoder-only architecture를 사용하며, 각 물체의 형상을 latent code `z`로 표현한다.

```
f_θ(z, x) → SDF value
```

새로운 물체에 대해서는 test-time optimization으로 `z`를 추정한다.

**NeuS**

NeRF의 volume rendering 품질과 SDF의 깨끗한 surface를 결합한 연구다. SDF 값을 density로 변환하는 함수를 도입하여, volume rendering framework 안에서 SDF를 학습한다.

```
density σ(x) = max(-dΦ_s(f(x))/dt, 0) / Φ_s(f(x))
```

여기서 `Φ_s`는 learnable parameter `s`가 제어하는 sigmoid-like 함수다. 학습이 진행될수록 `s`가 작아지면서 density가 surface 근처에 집중된다.

**VolSDF**

유사한 접근이지만, density를 SDF의 Laplace 분포 CDF로 정의한다.

```
σ(x) = (1/β) · Ψ_β(-f(x))
```

`β`가 줄어들수록 density가 surface에 집중된다.

**Surface 추출**

학습된 SDF에서 `f(x) = 0`인 iso-surface를 mesh로 변환하는 표준 방법이 **Marching Cubes** 알고리즘이다. 공간을 격자로 나누고, 각 격자 꼭짓점에서 SDF 부호를 확인해 surface가 지나가는 위치를 보간으로 결정한다.

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

*연구자가 되고 싶다면 여기서부터 읽어라.*

NeRF, 3DGS, NeuS 등 최근 3D 비전의 핵심 기술들은 하나의 공통 원리를 공유한다: 렌더링 과정을 미분 가능하게 만들어서, 렌더링 결과와 실제 이미지의 차이로 3D 표현을 최적화하는 것이다. 이 패러다임을 analysis-by-synthesis라 한다.

**Volume Rendering Equation**

NeRF 계열에서 사용하는 기본 렌더링 공식이다. 카메라에서 발사한 ray `r(t) = o + td` 위의 색상을 적분한다.

```
C(r) = ∫ T(t) · σ(t) · c(t) dt

where T(t) = exp( -∫_{t_n}^{t} σ(s) ds )
```

- `σ(t)`: 위치 `t`에서의 density (불투명도)
- `c(t)`: 위치 `t`에서의 색상 (RGB)
- `T(t)`: 누적 투과도 (ray가 `t`까지 도달할 확률)

실제로는 이 연속 적분을 discretize하여 ray 위의 N개 샘플 점에서 근사한다 (ray marching).

```
C(r) ≈ Σ_i T_i · α_i · c_i
where α_i = 1 - exp(-σ_i · δ_i),  T_i = Π_{j<i} (1 - α_j)
```

**3DGS의 Rasterization 기반 렌더링**

3DGS는 위의 ray marching 대신 각 Gaussian을 이미지 평면에 투영(splatting)하는 rasterization 방식을 쓴다. 각 픽셀의 색상은 해당 픽셀에 영향을 주는 Gaussian들의 가중 합이다.

```
C(p) = Σ_i c_i · α_i · Π_{j<i} (1 - α_j)
```

수식 구조는 volume rendering과 유사하지만, ray를 따라 샘플링하는 대신 Gaussian을 정렬(depth sorting)하여 순서대로 합성한다. 이것이 GPU rasterization pipeline과 호환되어 실시간 렌더링이 가능한 이유이다.

**Differentiable Rasterization 라이브러리**

mesh 기반 3D 표현을 미분 가능하게 렌더링하는 도구들이다.

- **PyTorch3D** (Meta): differentiable mesh renderer, point cloud renderer 제공
- **nvdiffrast** (NVIDIA): CUDA 기반 고성능 differentiable rasterizer
- **Kaolin** (NVIDIA): 3D 딥러닝 전반을 위한 라이브러리

**Analysis-by-Synthesis 파이프라인**

```
1. 가설적 3D 장면을 설정 (NeRF, 3DGS, mesh 등)
2. 해당 장면을 카메라 시점에서 렌더링 → 예측 이미지
3. 실제 관측 이미지와 비교 → loss 계산
4. loss를 역전파하여 3D 장면 파라미터 업데이트
5. 반복
```

이 패러다임은 3D 표현 형태에 관계없이 "이미지"라는 공통 supervision을 쓸 수 있다. 별도의 3D ground truth가 필요 없다.

**SLAM과의 연결**

Differentiable rendering을 SLAM에 적용하면, tracking과 mapping을 모두 렌더링 loss로 최적화할 수 있다.

- **NeRF-SLAM**: NeRF를 map representation으로 사용. 새 프레임이 들어오면 현재 맵을 렌더링하여 포즈를 추정(tracking)하고, 맵을 업데이트(mapping).
- **3DGS-SLAM** (SplaTAM, MonoGS 등): 3DGS를 맵으로 사용. 실시간 렌더링이 가능하므로 NeRF-SLAM보다 빠르다.

```
[새 프레임] → [현재 맵 렌더링] → [렌더링 vs 실제 비교]
                                      ↓
                              [포즈 업데이트 (tracking)]
                              [맵 업데이트 (mapping)]
```

> **추천 자료**
> - [Tewari et al., "Advances in Neural Rendering" (EGSR 2022)](https://arxiv.org/abs/2111.05849) — Differentiable rendering 서베이
> - [Ravi et al., "Accelerating 3D Deep Learning with PyTorch3D" (2020)](https://arxiv.org/abs/2007.08501) — PyTorch3D 논문
> - [Laine et al., "Modular Primitives for High-Performance Differentiable Rendering" (2020)](https://arxiv.org/abs/2011.03277) — nvdiffrast 논문

## 13.8 심화: 3D Scene Graph

*연구자가 되고 싶다면 여기서부터 읽어라.*

로봇에게 "주방에 있는 빨간 컵을 가져와"라고 명령하면, 포인트 클라우드나 mesh만으로는 이 명령을 수행하기 어렵다. "주방"이 어디인지, "빨간 컵"이 어떤 물체인지, 그것이 주방 "안에 있다"는 관계를 이해해야 한다. 3D Scene Graph는 환경을 기하학적 표현을 넘어 의미론적 관계 그래프로 표현하는 방법이다.

**구조**

- **노드(Node)**: 물체, 방, 건물 등 — 계층적(hierarchical) 구조
  - 건물 → 층 → 방 → 물체
  - 각 노드는 3D 위치, 바운딩 박스, 의미론적 라벨을 가짐
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

MIT에서 개발한 실시간 3D scene graph 구축 시스템이다. RGB-D 또는 LiDAR 입력을 받아, 로봇이 이동하면서 계층적 scene graph를 점진적으로 구축한다.

파이프라인:
1. Metric-semantic mesh 구축 (TSDF + semantic segmentation)
2. 방(room) 단위 분할 (free-space clustering)
3. 물체 노드 추출 및 관계 설정
4. 계층 구조 연결

Hydra는 이 모든 과정을 실시간(online)으로 수행한다. 로봇이 탐색하면서 동시에 scene graph가 갱신된다.

**ConceptGraphs**

Foundation model(CLIP, LLM)을 활용하여 open-vocabulary scene graph를 구축하는 연구다.

기존 scene graph는 미리 정의된 카테고리(의자, 테이블 등)에 의존했다. ConceptGraphs는 CLIP으로 임의의 자연어 쿼리에 대응하는 물체를 찾고, LLM으로 물체 간 관계를 추론한다.

```
1. RGB-D 프레임에서 open-vocabulary detector로 물체 감지
2. CLIP feature로 물체 임베딩 추출
3. 3D 공간에서 동일 물체 병합 (multi-view association)
4. LLM으로 물체 간 관계 추론
5. Scene graph 구축
```

훈련 시 본 적 없는 "빨간 컵" 같은 쿼리도 처리할 수 있다.

**왜 필요한가?**

| 표현 | "주방의 빨간 컵을 가져와" 수행 가능? | 이유 |
|------|------|------|
| Point Cloud | 불가 | 의미 정보 없음 |
| Semantic Map | 부분적 | "컵"은 찾지만 "주방에 있는"이라는 관계 처리 어려움 |
| 3D Scene Graph | 가능 | 물체 + 관계 + 계층 모두 표현 |

task planning이나 자연어 기반 내비게이션에서 scene graph는 3D 표현과 고수준 추론을 연결하는 다리 역할을 한다.

> **추천 자료**
> - [Hughes et al., "Hydra: A Real-time Spatial Perception System for 3D Scene Graph Construction and Optimization" (RSS 2022)](https://arxiv.org/abs/2201.13360) — Hydra 원 논문
> - [Gu et al., "ConceptGraphs: Open-Vocabulary 3D Scene Graphs for Perception and Planning" (2023)](https://arxiv.org/abs/2309.16650) — ConceptGraphs 논문
> - [Rosinol et al., "3D Dynamic Scene Graphs: Actionable Spatial Perception with Places, Objects, and Humans" (RSS 2020)](https://arxiv.org/abs/2002.06289) — Dynamic Scene Graph 개념 제시
> - [Armeni et al., "3D Scene Graph: A Structure for Unified Semantics, 3D Space, and Camera" (ICCV 2019)](https://arxiv.org/abs/1910.02527) — 3D Scene Graph 초기 연구

> **기술 흐름: 3D Vision**
> - **~2010**: 포인트 클라우드 처리 고전기. PCL 라이브러리, ICP 정합, TSDF 기반 볼류메트릭 복원이 주류.
> - **2012~**: Kinect 출시로 RGB-D 기반 3D 복원이 대중화되었다. KinectFusion(TSDF + ICP)이 실시간 3D 복원의 문을 열었다.
> - **2017~**: PointNet/PointNet++로 포인트 클라우드 딥러닝이 시작되었다. VoxelNet, PointPillars 등 3D Object Detection 연구도 이 시기에 급증했다.
> - **2020~**: NeRF 등장으로 Neural Rendering이 주목받았다. 사진 몇 장으로 포토리얼리스틱한 3D 장면을 만들 수 있게 되었고, Instant-NGP, Mip-NeRF 등 후속 연구가 빠르게 이어졌다.
> - **2023~**: 3D Gaussian Splatting이 NeRF의 속도 한계를 극복했다. 실시간 렌더링과 명시적 표현의 장점을 동시에 갖추었고, BEVFusion 등 멀티모달 3D 감지가 자율주행에서 기준점이 되었다.
> - **2024~**: 3DGS + SLAM 결합(SplaTAM, MonoGS, Gaussian-SLAM)으로 Neural SLAM의 새 방향이 열리고 있다. 로봇이 이동하면서 실시간으로 포토리얼리스틱 3D 맵을 구축하는 방식이다.
> - **지금 주목할 것**: SLAM/로보틱스 응용에서 3DGS 기반 방법이 빠르게 늘고 있다. NeRFStudio에서 두 방법 모두 실험해볼 수 있으니 직접 비교해보자.
