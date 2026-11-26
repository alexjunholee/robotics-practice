# 13장 3D 비전


11장과 12장에서 다룬 VFM과 VLA는 주로 2D 이미지와 언어에서 의미와 행동을 끌어냈다. 하지만 실제 물리 세계에서 로봇이 물체를 정확히 집어 올리고(manipulation), 장애물과 부딪히지 않고 주행하려면(SLAM·내비게이션) 눈앞 공간의 입체 형상과 절대 거리를 알아야 한다. 2D 사진 한 장만으로는 물체까지의 정확한 거리와 뒷면의 기하 구조를 바로 재기 어렵다. **3D 비전**은 포인트 클라우드를 다루고, 3차원 물체를 검출하며, 여러 시점의 관측으로 공간 전체를 복원해 로봇이 쓸 수 있는 기하학적·의미적 3차원 표현을 만들어 낸다.

## 13.1 Point Cloud 기초

카메라가 찍은 2D 이미지는 픽셀이 행과 열 격자 위에 가지런히 놓인 규칙적 데이터다. 반면 LiDAR나 깊이 카메라(depth camera)가 뱉어 내는 **포인트 클라우드(Point Cloud)**는 3차원 공간 안에 불규칙하게 흩어져 있는 점들의 집합이다. 인접한 픽셀을 인덱스 `(u+1, v)`로 곧장 찾을 수 있는 이미지와 달리, 포인트 클라우드에서는 점들의 저장 순서에 특별한 기하학적 의미가 없으므로 데이터를 다루는 사고방식부터 달라져야 한다.

각 점은 최소한 3차원 공간 좌표 `(x, y, z)`를 지니며, 센서 종류에 따라 색상(RGB), 레이저 반사 세기(intensity), 표면 법선 벡터(normal) 같은 부가 속성이 함께 붙는다.

### 13.1.1 데이터 구조 및 포맷

메모리에서 포인트 클라우드는 보통 각 점의 속성을 한 행으로 묶어 쌓은 2차원 배열로 표현한다.

```
Point: [x, y, z, r, g, b, intensity, ...]
Point Cloud: N × D 행렬 (N개 점, D차원 속성)
```

선형대수 관점에서 보면 포인트 클라우드는 `N×D` 행렬 하나다. 여기서 `N`은 점의 개수로 수만 개에서 수백만 개에 이르며, `D`는 한 점에 붙은 속성 차원이다. 로봇 좌표계 변환을 위해 포인트 클라우드 전체를 회전시키고 평행 이동할 때는 속성의 물리적 성격에 따라 처리가 갈린다. 위치 좌표 `(x, y, z)`는 마지막 성분에 `1`을 붙인 동차 좌표로 바꾼 뒤 `4×4` 강체 변환 행렬을 곱해 위치와 방향을 함께 옮긴다. 반면 색상이나 반사 세기 같은 고유 속성은 좌표계가 바뀌어도 변하지 않으므로 그대로 두고, 방향만 나타내는 표면 법선 벡터에는 이동 성분을 뺀 회전 행렬만 적용한다.

파일로 저장할 때는 목적과 생태계에 따라 여러 표준 포맷을 쓴다.

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

포인트 클라우드를 직접 구현해 다루기보다 검증된 오픈소스 라이브러리를 쓰는 편이 훨씬 효율적이다. 로보틱스에서는 **PCL**과 **Open3D**가 양대 산맥을 이룬다.

**PCL(Point Cloud Library)**은 C++ 기반의 가장 포괄적인 3D 처리 라이브러리로, ROS 생태계와 깊게 통합되어 있으며 필터링, 평면 분할, 정합 등 고전 기하 알고리즘을 폭넓게 제공한다.

```cpp
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::io::loadPCDFile<pcl::PointXYZ>("cloud.pcd", *cloud);
```

**Open3D**는 Python과 C++를 모두 지원하는 현대적인 API를 갖추고 있으며, 직관적인 3D 시각화가 강점이고 NumPy·PyTorch 같은 딥러닝 생태계와 매끄럽게 이어진다.

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

Python 바인딩 덕분에 알고리즘을 빠르게 시험해 보는 프로토타이핑 단계에서는 Open3D를 쓰는 편이 훨씬 수월하다. 반면 실시간성이 엄격히 요구되는 C++ ROS 파이프라인을 짤 때는 PCL을 마주칠 일이 잦으므로, 입문할 때는 Open3D로 감을 잡은 뒤 필요에 따라 PCL로 확장하는 흐름이 자연스럽다.

> **추천 자료**
> - [Open3D Getting Started](https://www.open3d.org/docs/release/getting_started.html) — Python으로 포인트 클라우드 다루기 입문
> - [PCL Tutorials — Basic Usage](https://pcl.readthedocs.io/projects/tutorials/en/latest/#basic-usage) — C++ 기반 포인트 클라우드 처리
> - [Open3D YouTube Channel](https://www.youtube.com/channel/UCRJBlASPfPBtPXJSPffJV-w) — 시각화 및 처리 튜토리얼

## 13.2 Point Cloud 처리

센서에서 막 들어온 원시 포인트 클라우드를 곧바로 인식이나 지도 작성에 넣는 경우는 드물다. 먼저 불필요한 잡음과 과도한 점 개수를 정리하고(필터링), 표면의 방향 정보를 계산한 뒤(법선 추정), 서로 다른 시점에서 찍은 점 구름을 하나의 좌표계로 맞춰야(정합) 비로소 쓸 만한 3D 데이터가 된다.

### 13.2.1 필터링 (Filtering)

원시 포인트 클라우드에는 센서 측정 노이즈로 공중에 뜬 점들이 섞여 있고, 센서에서 가까운 곳은 점이 빽빽한 반면 먼 곳은 듬성듬성해 밀도가 고르지 않다. 이를 그대로 두면 뒤이은 정합이나 세그멘테이션 연산이 급격히 느려지고 정확도도 떨어지므로, 파이프라인 맨 앞에서 먼저 걸러 낸다.

가장 널리 쓰는 밀도 균일화 기법은 **Voxel Grid Downsampling**이다. 3차원 공간을 일정한 크기의 정육면체 격자(voxel)로 나눈 뒤, 같은 칸 안에 들어온 점들을 무게중심 점 하나로 합쳐 점 수를 줄이면서도 전체 형상과 균일한 간격을 유지한다.

```python
# Open3D
voxel_pcd = pcd.voxel_down_sample(voxel_size=0.05)  # 5cm 격자
```

노이즈로 생긴 이상치(outlier)를 지울 때는 **Statistical Outlier Removal**을 쓴다. 점마다 가장 가까운 이웃 점들까지의 평균 거리를 계산한 뒤, 전체 분포에서 평균과 표준편차 기준을 크게 벗어나는 외딴 점들을 제거한다.

```python
# 이상치 제거
cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
filtered_pcd = pcd.select_by_index(ind)
```

이와 비슷하게 **Radius Outlier Removal**은 지정한 반경의 구 안에 이웃 점이 일정 개수 미만으로 들어 있는 고립된 점을 걸러 내는 방식이다.

> **추천 자료**
> - [Open3D — Point Cloud Filtering Tutorial](https://www.open3d.org/docs/release/tutorial/geometry/pointcloud.html) — Voxel downsampling, outlier removal 예제
> - [PCL — Filtering Tutorial](https://pcl.readthedocs.io/projects/tutorials/en/latest/passthrough.html) — PCL 기반 필터링

### 13.2.2 Normal Estimation

필터링을 마친 점 구름에서 한 단계 더 나아가 표면의 기울기, 즉 각 점에서의 **법선 벡터(normal vector)**를 추정하면 훨씬 풍부한 기하 추론이 가능해진다. 법선 벡터는 3D 표면 재구성(reconstruction)과 그래픽스 조명 계산은 물론, 평면 정보를 활용하는 point-to-plane ICP 정합에서도 핵심 입력으로 쓰인다(단순 point-to-point ICP는 점 좌표만 쓴다). 또한 한 점 주변 이웃 점들의 법선 방향이 나란한지 갑자기 꺾이는지를 비교하면, 그 점이 매끈한 평면 위인지 모서리나 경계인지도 가려낼 수 있다.

```python
# 법선 추정
pcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
)
```

내부적으로 Open3D는 각 점 주변의 이웃 점들(위 코드에서는 반경 0.1 m 안에서 최대 30개)을 모아 위치 공분산 행렬을 만든다. 점들이 국소적으로 작은 평면 위에 퍼져 있다면 평면 방향으로는 분산이 크고 표면에 수직인 방향으로는 분산이 가장 작을 것이므로, 이 공분산 행렬에서 최소 고유값에 대응하는 고유벡터를 곧장 표면 법선 방향으로 채택한다. 주성분 분석(PCA)에서 분산이 가장 작은 축을 찾아내는 원리와 같다.

### 13.2.3 정합 (Registration)

로봇이 이동하면서 스캔한 연속 프레임이나 여러 각도에서 찍은 부분 포인트 클라우드를 하나의 일관된 3차원 모델로 합치려면, 두 점 구름이 정확히 포개지도록 상대 회전과 이동을 찾는 **정합(Registration)**이 필요하다.

가장 고전적이면서도 핵심적인 알고리즘이 **ICP(Iterative Closest Point)**다. 두 포인트 클라우드가 어느 정도 가까이 놓여 있을 때, "가장 가까운 점끼리 짝을 짓고 → 그 쌍들의 거리를 줄이는 변환을 구해 움직이는" 과정을 수렴할 때까지 되풀이한다.

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

각 반복 단계에서 대응점 쌍이 정해지면, 그 쌍 사이의 유클리드 거리 제곱합을 최소화하는 회전 행렬 `R`과 이동 벡터 `t`를 구해야 한다. Point-to-point ICP에서는 이 문제를 특이값 분해(SVD)를 이용해 반복 탐색 없는 닫힌 형태(closed-form)로 단번에 풀어낸다.

현장에서는 장면 특성과 요구 정밀도에 따라 여러 변형 정합 기법을 고른다.

- **Point-to-Plane ICP**: 점과 점 사이의 직선 거리 대신, 점에서 상대 표면의 접평면(tangent plane)까지의 수직 거리를 최소화한다. 벽이나 바닥처럼 평면이 많은 실내·도로 환경에서 훨씬 빠르게 수렴한다.
- **GICP(Generalized ICP)**: 점 하나를 단순 좌표가 아니라 주변 이웃 점들의 분포(공분산)로 모델링해 함께 맞추므로 노이즈에 더 강인하다.
- **NDT(Normal Distributions Transform)**: 3차원 공간을 격자 셀로 쪼갠 뒤 각 셀 안의 점 분포를 정규분포로 근사하고, 소스 점들이 이 확률 분포에 가장 잘 들어맞는 변환을 찾는다.
- **Feature-based Registration**: 두 점 구름이 처음에 너무 멀리 떨어져 있으면 ICP는 잘못된 지역 최솟값(local minimum)에 빠진다. 이럴 때는 먼저 FPFH나 SHOT 같은 기하 특징을 뽑아 매칭한 뒤 RANSAC으로 대략적인 초기 정합(global registration)을 맞추고, 마지막으로 ICP를 돌려 정밀하게 다듬는다.

> **추천 자료**
> - [Open3D — ICP Registration Tutorial](https://www.open3d.org/docs/release/tutorial/pipelines/icp_registration.html) — ICP 실습 코드
> - [Open3D — Global Registration (RANSAC + Feature)](https://www.open3d.org/docs/release/tutorial/pipelines/global_registration.html) — Feature 기반 정합
> - [Cyrill Stachniss — ICP & Point Cloud Registration](https://www.youtube.com/watch?v=dhzLQfDBx2Q) — ICP 알고리즘의 직관적 설명

> **실습**: [ICP 2D 단계별 시각화](https://alexjunholee.github.io/robotics-practice/app.html#icp_steps) | [ICP 3D](https://alexjunholee.github.io/robotics-practice/app.html#icp_3d)
> ICP 알고리즘이 두 포인트 클라우드를 정합하는 과정을 iteration별로 확인하고, 2D와 3D 환경에서 수렴 과정을 비교할 수 있다.

## 13.3 3D Object Detection

필터링과 정합으로 깨끗하게 모은 포인트 클라우드 위에서, 로봇이나 자율주행차는 다음으로 주변 물체의 정확한 위치·크기·방향을 **3D 바운딩 박스(3D bounding box)**로 찾아내야 한다. 2D 박스 검출과 달리 깊이, 높이, 회전 각도(yaw 등)까지 예측해야 하므로 점들의 불규칙한 구조를 어떻게 신경망에 태울지가 핵심 설계 과제가 된다. 접근 방식은 크게 점 자체를 다루는 법, 격자로 바꾸는 법, 카메라와 융합하는 법 세 갈래로 나뉜다.

### 13.3.1 Point-based Methods

일반 합성곱 신경망(CNN)은 픽셀이 규칙적인 격자에 놓여 있어야 필터를 밀며 연산할 수 있다. 그래서 초기 3D 딥러닝 연구에서는 어쩔 수 없이 포인트 클라우드를 3차원 복셀이나 2차원 깊이 이미지로 억지로 변환해 넣곤 했다. **PointNet(2017)**은 이 관행을 깨고 원시(raw) 점 집합을 신경망에 그대로 입력하는 길을 열었다.

PointNet이 충족해야 했던 가장 중요한 수학적 조건은 **순서 불변성(permutation invariance)**이다. 같은 공간에 찍힌 점들이 파일에 어떤 순서로 나열되어 있든 신경망의 출력은 똑같아야 한다. PointNet은 각 점에 똑같은 MLP `h`를 독립적으로 적용해 특징을 뽑은 뒤, 순서의 영향을 받지 않는 대칭 연산인 **max pooling**으로 모든 점의 특징을 하나로 합친다. 수식으로 적으면 `f({x1, ..., xn}) = g(MAX(h(x1), ..., h(xn)))` 꼴이 되며, 이렇게 얻은 벡터가 점 집합 전체를 요약하는 global feature다.

다만 PointNet은 점들을 한꺼번에 max pooling해 버리므로 국소적인 미세 구조를 잡는 데 약했다. 이를 보완한 **PointNet++(2017)**는 CNN이 작은 수용 영역에서 큰 영역으로 계층적으로 특징을 키워 나가듯, **Set Abstraction** 블록을 여러 층 쌓아 지역 패턴을 단계적으로 학습한다.

```python
# PointNet++ 개념 구조
# 1. Sampling: FPS로 중심점 선택
# 2. Grouping: Ball query로 이웃 수집
# 3. PointNet: 각 그룹에서 특징 추출
```

즉, Farthest Point Sampling(FPS)으로 공간 전체를 고르게 대표하는 중심점들을 고르고, 각 중심점 반경 안의 이웃 점들을 ball query로 모은 뒤, 작은 영역마다 미니 PointNet을 돌려 국소 특징을 뽑아 올리는 구조다.

### 13.3.2 Voxel-based Methods

수십만 개의 점이 찍히는 야외 LiDAR 스캔에서 매번 FPS와 이웃 탐색을 수행하는 point-based 방식은 연산량이 만만치 않다. 그래서 자율주행용 3D 물체 검출에서는 공간을 규칙적인 3D 격자로 나누어 합성곱 연산의 속도를 되찾으려는 **voxel-based** 계열이 크게 발전했다.

- **VoxelNet (2018)**: 포인트 클라우드를 3D 복셀 격자로 나눈 뒤 각 복셀 안의 점들을 Voxel Feature Encoding으로 요약하고, 3D CNN을 통과시켜 물체를 검출한다.
- **SECOND (Sparsely Embedded Convolutional Detection)**: 야외 포인트 클라우드의 복셀은 대부분 텅 비어 있다는 점에 착안해, 점이 들어 있는 복셀에서만 연산하는 **Sparse Convolution**을 도입했다. VoxelNet보다 속도가 훨씬 빨라져 이후 수많은 3D 검출기의 기준선(baseline)으로 자리 잡았다.
- **PointPillars (2019)**: 계산 비용이 큰 3D CNN마저 없애 실시간성을 극대화했다. 3D 공간을 높이 방향으로 자르지 않고 수직 기둥(pillar) 단위로 나눈 뒤, 각 기둥 안의 점들을 하나의 특징 벡터로 압축해 위에서 내려다본 2D 이미지처럼 배열한다. 그 위에 가벼운 2D CNN만 돌리면 되므로 연산이 매우 빠르다.

### 13.3.3 Multi-modal Methods

실제 자율주행차나 로봇에는 LiDAR와 카메라가 함께 실려 있는 경우가 흔하다. 카메라는 먼 거리의 신호등 색상이나 차선, 물체의 세밀한 텍스처를 선명하게 보여 주는 대신 거리 정보가 약하고, LiDAR는 빛의 유무와 무관하게 정확한 3차원 좌표를 재는 대신 멀어질수록 점이 희박해져 텍스처 구분이 어렵다. 최근의 멀티모달 검출기들은 이 두 센서의 강점을 하나의 통합 표현 공간으로 끌어와 결합한다.

- **BEVFusion**: 카메라 이미지 특징과 LiDAR 포인트 클라우드 특징을 모두 위에서 내려다본 **조감도(Bird's Eye View, BEV)** 공간으로 투영해 한데 묶는다. 두 센서 정보가 동일한 평면 좌표계에서 만나므로 융합이 자연스럽고 후속 주행 계획 모듈과도 바로 이어진다.
- **TransFusion**: Transformer 아키텍처를 바탕으로 LiDAR 특징에서 초기 물체 후보(query)를 뽑은 뒤, 이를 이미지 특징과 cross-attention으로 융합해 위치를 정교하게 다듬는다.

> **추천 자료**
> - [Qi et al., "PointNet: Deep Learning on Point Sets" (2017)](https://arxiv.org/abs/1612.00593) — 3D 딥러닝의 시작점
> - [Lang et al., "PointPillars" (2019)](https://arxiv.org/abs/1812.05784) — 실시간 3D 감지
> - [Liu et al., "BEVFusion" (ICRA 2023)](https://arxiv.org/abs/2205.13542) — 카메라·라이다 특징을 BEV에서 융합
> - [MMDetection3D GitHub](https://github.com/open-mmlab/mmdetection3d) — 3D Object Detection 통합 프레임워크

> **실습**: [BEV Projection 시각화](https://alexjunholee.github.io/robotics-practice/app.html#bev_projection)
> 카메라 이미지를 BEV로 변환하는 과정을 인터랙티브하게 확인하며, BEV 기반 3D 감지의 원리를 이해할 수 있다.

## 13.4 3D Reconstruction

개별 물체를 바운딩 박스로 찾아내는 수준을 넘어, 로봇이 활동하는 방이나 건물 전체의 형상을 연속적인 3D 모델로 복원하면 이를 곧장 충돌 없는 경로 계획에 활용하고 환경 지도로도 쓸 수 있다. 여러 각도에서 찍은 2D 사진이나 연속된 RGB-D 깊이 영상으로부터 3차원 기하 구조를 빚어내는 고전적·표준적 복원 파이프라인을 살펴보자.

### 13.4.1 Structure from Motion (SfM)

**Structure from Motion(SfM)**은 서로 다른 위치에서 찍은 여러 장의 2D 사진만으로 각 사진을 찍었을 때의 **카메라 포즈(위치와 방향)**와 장면의 **3차원 구조**를 동시에 역산하는 기술이다. 스마트폰으로 건물을 빙 둘러 찍은 사진들로 3D 모델을 만들 때, 그리고 뒤에서 배울 NeRF나 3D Gaussian Splatting에 들어갈 정확한 카메라 포즈를 사전에 구할 때 가장 먼저 거치는 관문이다.

SfM은 보통 다음 네 단계로 진행된다.

1. 각 이미지에서 SIFT 같은 국소 특징점을 추출하고 서로 다른 사진 사이의 대응점을 매칭한다.
2. 기하학적으로 좋은 초기 두 뷰(view)를 골라 삼각측량으로 첫 3차원 점들을 복원한다.
3. 나머지 카메라를 하나씩 점진적으로 추가하며 3D 점 구름을 확장한다.
4. 누적된 오차를 잡기 위해 **Bundle Adjustment(BA)**로 전체 구조를 다듬는다.

이 중 마지막 단계인 **Bundle Adjustment**가 정밀도를 결정짓는다. 모든 카메라의 포즈와 모든 3D 점의 좌표를 한꺼번에 변수로 놓고, 3D 점들을 각 카메라 화면에 다시 투영했을 때 원래 관측된 픽셀 좌표와 어긋나는 정도(재투영 오차)를 최소화한다. 이 비선형 최소자승 문제는 보통 Levenberg-Marquardt 알고리즘으로 풀며, 사진과 점이 많아지면 최적화해야 할 변수가 수만에서 수십만 개에 달한다.

현장에서는 그래픽 사용자 인터페이스(GUI)와 명령행(CLI)을 모두 갖춘 공개 표준 구현 **COLMAP**이나 라이브러리 형태로 설계된 **OpenMVG**가 널리 쓰인다. 아래는 COLMAP CLI로 이미지 폴더에서 특징점 추출, 매칭, 희소 3D 복원까지 한 번에 돌리는 기본 명령 예시다.

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

SfM이 내놓는 결과물은 카메라 포즈와 듬성듬성한(sparse) 특징점들의 3차원 좌표다. 윤곽만 겨우 알아볼 수 있는 이 뼈대 위에 빽빽하고 빈틈없는 표면 점 구름을 채워 넣는 후속 단계가 **Multi-View Stereo(MVS)**다. SfM으로 각 사진의 정확한 카메라 포즈가 이미 확정되었으므로, MVS는 에피폴라 기하 제약을 활용해 픽셀 단위로 촘촘히 대응점을 찾고 조밀한(dense) 3D 포인트 클라우드를 생성한다. 이렇게 `SfM → MVS → Mesh 생성`으로 이어지는 흐름이 사진 기반 3D 복원의 정석적인 파이프라인이며, 대표 도구로는 COLMAP의 dense reconstruction 기능과 **OpenMVS**가 쓰인다.

### 13.4.3 Volumetric Reconstruction

RGB-D 카메라나 LiDAR처럼 이미 프레임마다 깊이 정보가 주어지는 로봇 환경에서는 사진 매칭 대신 공간을 3차원 복셀 격자로 쪼개 깊이를 직접 누적하는 **Volumetric Reconstruction**이 훨씬 빠르고 깔끔하다. 그 중심에 **TSDF(Truncated Signed Distance Function)**가 있다.

TSDF는 공간의 각 복셀에 가장 가까운 물체 표면까지의 **부호 있는 거리(signed distance)**를 저장한다. 표면 바깥쪽(카메라 쪽 빈 공간)은 양수, 물체 안쪽은 음수로 기록하며, 표면에서 일정 거리 이상 멀어지면 값을 잘라내(truncated) 계산과 메모리를 아낀다. 로봇이 움직이며 여러 시점에서 찍은 깊이 프레임을 같은 복셀 격자에 가중 평균으로 계속 누적하면 센서 노이즈가 자연스럽게 상쇄된다. 최종적으로 거리의 부호가 양에서 음으로 바뀌는 0 지점을 **Marching Cubes** 알고리즘으로 이어 붙이면 매끈한 삼각형 메쉬(mesh)가 추출된다.

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

앞서 본 포인트 클라우드나 삼각형 메쉬는 기하 구조를 명확하게 담지만, 금속 표면의 은은한 반사, 투명한 유리잔, 나뭇잎이나 전선처럼 가느다란 구조를 사진처럼 사실적으로 표현하는 데는 한계가 있다. **Neural Rendering**은 고정된 삼각형 꼭짓점 대신 신경망이나 학습 가능한 3D 프리미티브로 장면을 표현하고, 카메라 시점에 따른 빛과 색의 상호작용을 학습해 이 한계를 돌파했다. 최근에는 단순히 예쁜 영상을 합성하는 데 그치지 않고 SLAM과 결합해 로봇이 움직이며 실시간으로 실사급 지도를 구축하는 데에도 적극 쓰이고 있다.

### 13.5.1 NeRF (Neural Radiance Fields)

**NeRF**는 3차원 장면 전체를 하나의 연속적인 신경망 함수로 통째로 외워 버리는 발상으로 3D 비전 분야에 큰 전환점을 가져왔다.

```
F: (x, y, z, θ, φ) → (r, g, b, σ)
- 위치 (x, y, z)와 시점 방향 (θ, φ)
- 색상 (r, g, b)과 밀도 (σ) 출력
```

3차원 공간상의 좌표 `(x, y, z)`와 그 점을 바라보는 시선 방향 `(θ, φ)`를 다층 퍼셉트론(MLP)에 넣으면, 그 지점의 시점 의존적 색상 `(r, g, b)`과 빛을 얼마나 가로막는지 나타내는 부피 밀도 `σ`가 출력된다. 장면을 렌더링할 때는 카메라 픽셀마다 광선(ray)을 쏘아 지나가는 점들의 색상과 밀도를 적분(volume rendering)해 최종 픽셀 색을 만든다. 학습이 끝나면 한 번도 카메라를 대 본 적 없는 새로운 위치와 각도에서도 놀라울 만큼 사실적인 영상을 합성해 낸다(novel view synthesis).

반사나 반투명 같은 복잡한 광학 효과까지 훌륭하게 재현하는 대신, 초기 NeRF는 픽셀마다 수많은 점을 샘플링해 MLP를 통과시켜야 해서 한 장면을 학습하고 렌더링하는 데 시간이 오래 걸렸고 움직이는 동적 장면을 다루기도 까다로웠다. 이후 이 약점을 보완하기 위한 후속 모델이 잇달아 등장했다. 예컨대 **Instant-NGP**는 다중 해상도 해시 인코딩으로 학습 시간을 수 분 이내로 단축했고, **Mip-NeRF**는 원뿔 형태 광선 추적으로 계단 현상과 흐림을 잡았으며, **Block-NeRF**는 도시 블록 같은 대규모 공간을 분할 학습한다.

> **추천 자료**
> - [Mildenhall et al., "NeRF: Representing Scenes as Neural Radiance Fields" (ECCV 2020)](https://arxiv.org/abs/2003.08934) — NeRF 원 논문
> - [NeRFStudio Documentation](https://docs.nerf.studio/) — NeRF 실험용 통합 프레임워크
> - [Yannic Kilcher — NeRF Explained](https://www.youtube.com/watch?v=CRlN-cYFxTk) — NeRF의 표현 방식을 직관적으로 설명
> - [Jon Barron — Understanding and Extending Neural Radiance Fields](https://www.youtube.com/watch?v=HfJpQCBTqZs) — MIT Vision & Graphics Seminar 강연. 연사는 NeRF 공저자다

### 13.5.2 3D Gaussian Splatting (3DGS)

NeRF가 뛰어난 화질을 얻은 대가로 치러야 했던 느린 렌더링 속도(초기 모델 기준 프레임당 수 초 이상)를 단숨에 해결한 기술이 **3D Gaussian Splatting(3DGS)**이다. 원 논문은 1080p 고해상도에서도 30 FPS 이상의 실시간 렌더링을 달성했다고 보고했으며, 이 압도적인 속도 덕분에 온라인 3D 매핑과 SLAM 분야에서 폭발적인 관심을 모으고 있다.

3DGS는 장면을 암묵적인 신경망 가중치 속에 감추는 대신, 공간에 수백만 개의 **반투명한 3차원 타원체(3D Gaussian)**를 직접 띄워 놓는 명시적(explicit) 표현을 택한다. 각 Gaussian은 네 가지 속성을 지닌다.

- **위치 (mean)**: 3D 공간에서의 중심 좌표
- **공분산 (covariance)**: 타원체의 모양과 크기, 회전 방향
- **색상 (Spherical Harmonics)**: 바라보는 방향에 따라 변하는 빛깔을 표현하는 구면 조화 함수 계수
- **불투명도 (opacity)**: 빛을 통과시키거나 가리는 정도

여기서 모양과 방향을 결정짓는 `3×3` 공분산 행렬을 고유값 분해해 보면 기하학적 의미가 한눈에 들어온다. 공분산 행렬의 세 고유벡터가 곧 3D 타원체의 직교하는 세 축 방향이며, 각 축의 반경 길이는 대응하는 고유값의 제곱근에 비례한다. 학습 과정에서 3DGS는 장면을 잘 메우도록 이 Gaussian들의 위치·크기·색·투명도를 조절할 뿐 아니라, 세밀한 묘사가 필요한 곳에서는 Gaussian을 쪼개거나 복제하고 불필요한 것은 지우며 개수를 스스로 최적화한다.

렌더링할 때는 NeRF처럼 광선을 따라 일일이 점들을 샘플링하는 대신, 3D Gaussian들을 카메라 이미지 평면으로 한꺼번에 투영해 겹쳐 그리는 **래스터화(splatting)** 방식을 쓴다. 그 덕분에 1080p에서 30 FPS 이상으로 매우 빠르게 그려지며, 원 논문 Mip-NeRF360 벤치마크 기준 7K iteration이면 약 6분, 최고 품질의 30K iteration도 42분 내외로 학습을 마친다. 또한 3D 공간에 실체가 놓인 명시적 표현이므로 특정 물체만 지우거나 옮기는 편집도 직관적이다.

```python
# 3DGS 기본 개념 (pseudo-code)
# 각 Gaussian: position, covariance, color, opacity
# 렌더링: 카메라 뷰로 투영하여 이미지 생성
```

실제 로보틱스에서는 이 고속 표현을 SLAM과 묶어 **3DGS + SLAM** 시스템으로 확장하고 있다. 다만 한 가지 짚고 넘어가야 할 숫자가 있다. 앞서 말한 `30 FPS 이상`은 이미 학습이 끝난 정적 3DGS 맵을 화면에 *그려 내기만* 할 때의 속도다. 로봇이 움직이면서 매 프레임 카메라 포즈를 추적(tracking)하고 동시에 새 Gaussian을 추가·최적화(mapping)하는 전체 SLAM 루프의 속도는 그보다 훨씬 낮다. 예컨대 MonoGS 논문에서는 전체 시스템이 약 3 fps로 동작한다고 보고했다.

대표적인 3DGS 기반 SLAM 연구로는 다음과 같은 시스템들이 있다.

- **SplaTAM (CVPR 2024)**: RGB-D 카메라 입력을 받아 3DGS로 조밀한 지도를 그리는 dense SLAM이다. Tracking(카메라 포즈 추정)과 Mapping(Gaussian 추가·갱신)을 번갈아 실행한다. 논문에서는 포즈 추정 정확도와 지도 복원, 새 시점 합성 품질에서 기존 방법보다 최대 2배 우수한 성능을 보고했다.
- **MonoGS (CVPR 2024)**: 깊이 센서 없이 단안(monocular) 카메라 영상만으로도 3DGS 기반 SLAM을 구동하며, 필요에 따라 RGB-D 입력도 지원한다.
- **Gaussian-SLAM (arXiv 2023 프리프린트)**: RGB-D 입력을 여러 개의 서브맵(sub-map)으로 쪼개어 관리함으로써 넓은 공간에서도 메모리와 드리프트 문제를 줄인다.

이렇게 만든 실사급 3D 맵은 로봇 내비게이션을 넘어 디지털 트윈 구축이나 AR/VR 콘텐츠 제작에도 곧바로 활용된다.

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

## 13.6 Neural Implicit Representations [심화]

NeRF와 3DGS가 눈으로 보기에 감쪽같은 장면을 그려 주기는 하지만, 로봇 공학에서는 또 다른 요구가 있다. NeRF의 부피 밀도장(density field)이나 공중에 흩뿌려진 3D Gaussian으로는 **정확한 물체 표면(surface)**이 어디인지 날카롭게 짚어 내기가 까다롭다. 로봇 그리퍼가 물체에 부딪히지 않고 이를 정확히 짚으려면 매끈하고 빈틈없는 표면 경계 정보가 필수적이다. 이 정보를 신경망으로 정밀하게 표현하는 도구가 **부호 있는 거리 함수(Signed Distance Function, SDF)** 기반의 Neural Implicit Representation이다.

**SDF(Signed Distance Function)**는 3차원 공간의 임의의 점 `x`를 입력받아 가장 가까운 물체 표면까지의 거리를 부호와 함께 돌려주는 연속 함수다.

```
f(x) > 0  : 표면 바깥
f(x) < 0  : 표면 안쪽
f(x) = 0  : 표면 위 (zero level set)
```

이 함수 값이 정확히 `0`이 되는 등위면(zero level set)이 곧 우리가 찾는 물체의 깨끗한 표면이다. 또한 유클리드 공간에서 유효한 거리 함수라면, 표면에 수직인 방향으로 `1`만큼 움직였을 때 거리 값도 정확히 `1`만큼 변해야 하므로 미분 가능한 모든 지점에서 기울기(gradient)의 크기가 항상 `1`이라는 기하학적 성질(**Eikonal equation**)을 만족한다.

```
||∇f(x)|| = 1
```

신경망으로 SDF를 학습할 때 네트워크가 이상한 값을 내지 않고 진짜 거리 함수처럼 거동하도록 이 성질을 정규화 손실 항으로 강제한다. 이 손실 항이 **Eikonal loss**다.

```
L_eikonal = E_x[ (||∇f_θ(x)|| - 1)^2 ]
```

이 SDF를 딥러닝으로 학습하는 방식은 크게 두 세대로 발전했다. 먼저 초기 연구인 **DeepSDF(2019)**는 별도의 인코더 없이 디코더 MLP만 두고, 각 물체의 고유한 형상 정보를 작은 잠재 벡터 `z` 하나에 압축해 표현했다. 새로운 물체가 주어지면 테스트 시점에 디코더 가중치는 고정한 채 `z`만 최적화(test-time optimization)해 형상을 맞춰 나간다.

```
f_θ(z, x) → SDF value
```

하지만 DeepSDF는 3차원 정답 SDF 값을 직접 알아야 학습할 수 있다는 한계가 있었다. 반면 **NeuS(2021)**와 **VolSDF(2021)**는 SDF 값을 NeRF의 부피 밀도(density)로 변환해, 3D 형상 정답 없이 일반 2D 사진만 보고도 volume rendering을 거쳐 SDF를 학습할 수 있게 했다.

**NeuS**는 광선 위의 점 `r(t) = o + t·d`에서의 SDF 값에 시그모이드 함수 `Φ_s` 기반의 변환을 적용해 밀도 `ρ(t)`를 구한다.

```
density ρ(t) = max(-dΦ_s(f(r(t)))/dt, 0) / Φ_s(f(r(t))),   r(t) = o + t·d 는 광선 위의 점
```

여기서 `s`는 `Φ_s`의 가파른 정도를 조절하는 학습 가능 파라미터다. 그 역수인 `1/s`가 도함수 분포(S-density)의 표준편차에 해당하므로, 학습 과정에서 `s`가 점차 커지면 밀도가 `f(x) = 0`인 표면 부근에 종잇장처럼 날카롭게 모인다. **VolSDF** 역시 비슷한 철학으로, SDF 값에 규모 파라미터가 `β`인 Laplace 분포의 누적분포함수(CDF)를 적용해 밀도를 정의하며, `β`가 작아질수록 밀도가 표면에 바싹 달라붙는다.

```
σ(x) = (1/β) · Ψ_β(-f(x))
```

이렇게 2D 이미지만으로 신경망 SDF 학습이 끝나면, 3차원 공간을 격자로 나눈 뒤 각 꼭짓점에서 `f(x)`의 부호가 바뀌는 지점을 찾는다. 이어서 **Marching Cubes** 알고리즘으로 보간해 깔끔한 삼각형 메쉬(surface)를 추출한다.

주요 3D 신경망 표현들의 성격을 한눈에 비교하면 다음과 같다.

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

## 13.7 Differentiable Rendering [심화]

그런데 잠깐, NeRF나 3DGS, NeuS는 어떻게 3차원 형상 정답지 없이 2D 사진 몇십 장만 보고도 3D 구조를 이토록 정교하게 복원할 수 있을까? 그 비밀은 **렌더링 과정 자체를 미분 가능(differentiable)하게 만든 데** 있다. 현재 머릿속에 가정해 둔 3D 장면 표현으로 카메라 이미지를 직접 그려 보고(synthesis), 그 예측 이미지와 실제 카메라가 찍은 사진 사이의 픽셀 차이를 계산해 미분 오차를 3D 파라미터로 거꾸로 흘려보낸다. 이처럼 이미지를 합성해 관측과 맞춰 가며 3D 장면을 찾아내는 패러다임을 **analysis-by-synthesis**라 부른다.

NeRF와 NeuS 계열은 이 미분 가능한 렌더링의 뼈대로 고전적인 **Volume Rendering Equation**을 쓴다. 카메라 원점 `o`에서 픽셀 방향 `d`로 쏜 광선 `r(t) = o + td`를 따라, 가까운 끝 `t_n`부터 먼 끝 `t_f`까지 놓인 모든 위치의 색상과 밀도를 적분해 최종 픽셀 색을 얻는다.

```
C(r) = ∫_{t_n}^{t_f} T(t) · σ(t) · c(t) dt

where T(t) = exp( -∫_{t_n}^{t} σ(s) ds )
```

수식 속 각 기호의 물리적 의미는 직관적이다. `t_n`과 `t_f`는 광선 위 적분 구간의 시작과 끝 거리, `σ(t)`는 위치 `t`에서 빛이 입자에 부딪혀 멈추거나 반사되는 부피 밀도(길이의 역수 차원), `c(t)`는 그 위치의 RGB 색상이다. 그리고 `T(t)`는 광선이 시작점부터 `t` 지점까지 중간의 짙은 안개나 물체에 가로막히지 않고 무사히 도달할 확률인 **누적 투과도(transmittance)**다. 즉, 어떤 지점이 최종 픽셀 색에 기여하려면 그 지점 앞쪽이 투명해서 빛이 닿아야 하고(`T(t)`), 그 지점 자체에 물질이 존재해야 하며(`σ(t)`), 그 물질의 고유 색(`c(t)`)이 더해져야 한다는 뜻이다.

실제 컴퓨터 구현에서는 연속 적분을 직접 계산할 수 없으므로, 광선을 `N`개의 이산 점으로 잘게 쪼개 근사하는 **ray marching**을 쓴다.

```
C(r) ≈ Σ_i T_i · α_i · c_i
where α_i = 1 - exp(-σ_i · δ_i),  T_i = Π_{j<i} (1 - α_j)
```

여기서 `δ_i = t_{i+1} - t_i`는 이웃한 두 샘플 점 사이의 간격이며, `α_i`는 그 작은 구간에서 빛이 불투명하게 걸릴 확률을 나타낸다. 덧셈과 곱셈, 지수 함수만으로 이루어져 있으므로 자연스럽게 미분이 가능하다.

한편 **3DGS**는 이 무거운 ray marching 대신 각 Gaussian을 이미지 평면에 투영해 겹쳐 그리는 **rasterization** 방식을 취한다. 최종 픽셀 색상 `C(p)`의 합성 공식 자체는 놀랍게도 volume rendering과 같은 꼴이다.

```
C(p) = Σ_i c_i · α_i · Π_{j<i} (1 - α_j)
```

다만 3DGS는 광선을 따라 빈 공간까지 일일이 샘플링하며 신경망을 부르는 대신, 시야 안에 들어오는 Gaussian들을 깊이 순서대로 정렬한 뒤 화면을 작은 타일(tile)로 쪼개 GPU에서 타일별로 병렬 합성(alpha blending)해 버리므로 실시간 렌더링 속도를 낼 수 있다.

만약 신경망 밀도나 Gaussian이 아니라 전통적인 삼각형 메쉬를 이미지 오차로 직접 최적화하고 싶다면 **Differentiable Rasterization** 라이브러리를 쓰면 된다. Meta의 **PyTorch3D**(미분 가능한 메쉬 및 포인트 클라우드 렌더러), NVIDIA의 CUDA 기반 고속 래스터라이저 **nvdiffrast**, 그리고 3D 딥러닝 전반을 다루는 NVIDIA **Kaolin**이 대표적이다.

이러한 미분 가능한 렌더러를 활용한 **Analysis-by-Synthesis 파이프라인**은 어떤 3D 표현을 쓰든 다음 다섯 단계의 루프로 돌아가며, 오직 2D 이미지 비교만으로 3D 장면을 학습시킨다.

```
1. 가설적 3D 장면을 설정 (NeRF, 3DGS, mesh 등)
2. 그 장면을 카메라 시점에서 렌더링 → 예측 이미지
3. 실제 관측 이미지와 비교 → loss 계산
4. loss를 역전파하여 3D 장면 파라미터 업데이트
5. 반복
```

흥미롭게도 이 루프는 그대로 **SLAM**의 핵심 엔진이 된다. 새 카메라 프레임이 들어왔을 때 현재까지 만든 지도를 렌더링해 실제 카메라 영상과 비교하면 두 가지를 동시에 미분으로 풀 수 있기 때문이다. 카메라 포즈를 움직여 영상 차이를 줄이면 **tracking**이 되고, 지도 파라미터를 움직여 차이를 줄이면 **mapping**이 된다.

```
[새 프레임] → [현재 맵 렌더링] → [렌더링 vs 실제 비교]
                                      ↓
                              [포즈 업데이트 (tracking)]
                              [맵 업데이트 (mapping)]
```

신경망 implicit 표현을 지도로 삼은 초기 연구인 **iMAP**과 **NICE-SLAM**이 이 구조를 개척했다. 이어 등장한 **3DGS-SLAM(SplaTAM, MonoGS 등)**은 훨씬 빠른 rasterization 덕분에 훨씬 더 많은 픽셀의 렌더링 오차를 계산하면서도, 프레임당 처리 시간은 비슷하거나 더 짧은 수준으로 유지하며 한 단계 진화했다.

> **추천 자료**
> - [Tewari et al., "Advances in Neural Rendering" (EUROGRAPHICS 2022 STAR)](https://arxiv.org/abs/2111.05849) — Differentiable rendering 서베이
> - [Ravi et al., "Accelerating 3D Deep Learning with PyTorch3D" (2020)](https://arxiv.org/abs/2007.08501) — PyTorch3D 논문
> - [Laine et al., "Modular Primitives for High-Performance Differentiable Rendering" (2020)](https://arxiv.org/abs/2011.03277) — nvdiffrast 논문

## 13.8 3D Scene Graph [심화]

여기까지 우리는 점 구름에서 복셀과 메쉬, NeRF와 SDF, 3DGS에 이르기까지 물리 공간의 기하학적 형상과 외형을 3D로 복원하는 다양한 표현을 살펴보았다. 하지만 로봇에게 `"주방에 있는 빨간 컵을 가져와"`라고 시켰을 때, 단순한 포인트 클라우드나 메쉬 지도만으로는 이 명령을 이해하기 어렵다. 로봇은 어디가 '주방'이라는 방인지, 무엇이 '빨간 컵'인지, 그리고 그 컵이 주방 안의 테이블 '위에' 놓여 있는지와 같은 **계층적 의미와 관계**를 구조적으로 파악해야 한다. **3D Scene Graph**는 공간 속 물체와 장소를 **노드(Node)**로, 그 사이의 공간적·의미적 관계를 **엣지(Edge)**로 엮어 이 문제를 푸는 고수준 3D 지도 표현이다.

- **노드(Node)**: 물체, 방, 층, 건물처럼 크기가 다른 공간 개체를 계층적(hierarchical) 구조로 담는다(`건물 → 층 → 방 → 물체`). 각 노드에는 3차원 중심 좌표, 바운딩 박스, 의미 라벨이 붙는다.
- **엣지(Edge)**: 노드 사이를 관계로 연결한다(`"위에 있다"(on)`, `"안에 있다"(in)`, `"가까이 있다"(near)`, `"지지한다"(support)` 등).

실제 건물 내부를 3D Scene Graph로 표현하면 다음과 같은 트리·그래프 계층 구조를 이룬다.

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

로봇이 환경 속을 주행하면서 실시간으로 이러한 3D Scene Graph를 점진적으로 구축하는 대표 시스템이 MIT에서 개발한 **Hydra**다. Hydra는 다음 네 단계 파이프라인으로 센서 입력을 계층 그래프로 끌어올린다.

1. TSDF 복원과 semantic segmentation을 결합해 메트릭-시맨틱 메쉬(metric-semantic mesh)를 구축한다.
2. 메쉬에서 개별 물체 노드를 추출하고 인접 관계를 설정한다.
3. 로봇이 다닐 수 있는 빈 공간의 연결성(free-space clustering)을 분석해 방(room) 단위로 공간을 분할한다.
4. 물체, 장소, 방, 건물 노드를 계층 구조로 연결한다.

Hydra가 미리 학습해 둔 고정 클래스 목록을 바탕으로 그래프를 짰다면, 최근에 나온 **ConceptGraphs**는 11장에서 본 비전 파운데이션 모델(CLIP)과 LLM을 결합해 **open-vocabulary 3D scene graph**로 나아갔다. 그 덕분에 학습 목록에 없던 `"빨간 컵"` 같은 자유로운 자연어 질의로도 대상을 그대로 찾아낼 수 있다.

```
1. RGB-D 프레임에서 class-agnostic segmentation으로 물체 후보 분할
2. CLIP feature로 물체 임베딩 추출
3. 3D 공간에서 동일 물체 병합 (multi-view association)
4. LLM으로 물체 간 관계 추론
5. Scene graph 구축
```

이처럼 3D 비전 표현은 단순 좌표 집합에서 관계 그래프로 진화해 왔다. 그에 따라 가능한 추론의 수준을 비교해 보면 3D Scene Graph가 왜 필요한지 분명해진다.

| 표현 | "주방의 빨간 컵을 가져와"를 처리할 수 있나 | 이유 |
|------|------|------|
| Point Cloud | 불가 | 의미 정보 없음 |
| Semantic Map | 부분적 | "컵"은 찾지만 "주방에 있는"이라는 관계 처리 어려움 |
| 3D Scene Graph | 가능 | 물체 + 관계 + 계층 모두 표현 |

오늘날 작업 계획(task planning)이나 자연어 기반 내비게이션 시스템에서는 거친 3D 기하 지도와 LLM의 고수준 추론 사이를 잇는 핵심 중간 표현으로 이 3D Scene Graph를 널리 채택하고 있다.

이처럼 3D 비전은 포인트 클라우드와 TSDF부터 3DGS, SDF, 그리고 3D Scene Graph까지 로봇이 주변 세계를 정밀하고 입체적으로 이해할 수 있는 다층적인 지도를 제공한다. 그런데 정지된 데이터셋에서 장면을 복원하는 것과 달리, 실제 로봇은 낯선 환경 속을 계속 이동하면서 자신의 현재 위치와 자세가 틀어지지 않도록 추적하는 동시에 이 3D 지도를 실시간으로 쌓아 올려야 한다. 다음 14장에서는 바로 이 문제를 푸는 **SLAM과 오도메트리**의 세계로 들어간다.

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
