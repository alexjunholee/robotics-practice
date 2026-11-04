# Ch.9 — 컴퓨터 비전 기초 (Computer Vision Fundamentals)


컴퓨터 비전은 카메라의 원시 데이터를 로봇이 쓸 수 있는 정보로 바꾼다. 이미지 처리와 카메라 기하를 이해해야 SLAM이나 물체 조작 파이프라인에서 생기는 오류를 추적할 수 있다.

---

## 9.1 이미지 처리 기초 (Image Processing)

카메라의 raw 이미지에는 노이즈와 불필요한 변화가 섞여 있다. 필터링, 에지 검출, 형태학적 연산으로 입력을 정리하면 후속 파이프라인의 이상이 전처리에서 생겼는지 구분할 수 있다.

### 9.1.1 OpenCV 소개

OpenCV(Open Source Computer Vision Library)는 널리 쓰이는 공개 CV 라이브러리이다.

OpenCV는 C++ 라이브러리이고 Python 바인딩도 있다. 영상 입출력부터 필터링·특징 추출·기하 계산까지 자주 쓰는 연산을 묶어 둔다.

설치:

```bash
pip install opencv-python opencv-contrib-python
```

기본 사용:

```python
import cv2
import numpy as np

# 이미지 읽기
img = cv2.imread('image.jpg')

# 그레이스케일 변환
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# 이미지 표시
cv2.imshow('Image', img)
cv2.waitKey(0)
cv2.destroyAllWindows()
```

주의: OpenCV는 BGR 순서를 사용한다. Matplotlib처럼 RGB를 쓰는 라이브러리에 이미지를 넘길 때는 `cv2.cvtColor(img, cv2.COLOR_BGR2RGB)`로 채널 순서를 바꾼다.

> 추천 자료
> - [OpenCV 공식 튜토리얼](https://docs.opencv.org/4.x/d9/df8/tutorial_root.html) — Python/C++ 예제가 잘 정리되어 있다
> - [First Principles of Computer Vision](https://www.youtube.com/channel/UCf0WB91t8Ky6AuYcQV0CcLw) — Columbia의 Shree Nayar 교수의 채널. 이미지 처리 원리를 직관적으로 설명한다
> - [Szeliski, "Computer Vision: Algorithms and Applications"](https://szeliski.org/Book/) — 저자 사이트에서 PDF를 무료로 받을 수 있다
> - [Stanford CS131 — Computer Vision: Foundations and Applications](http://vision.stanford.edu/teaching/cs131_fall1415/schedule.html) — CS231n보다 기초적인 CV 강의. 이미지 처리부터 시작하고 싶다면 여기서

### 9.1.2 필터링 (Filtering)

필터링은 이미지에서 원하는 정보를 남기고 노이즈를 줄인다. 에지 검출이나 segmentation(영역 분할) 전에는 blur를 걸어 입력의 고주파 변동을 줄인다.

Blur (흐림):

```python
# Gaussian Blur
blurred = cv2.GaussianBlur(img, (5, 5), 0)

# Median Blur (노이즈 제거에 효과적)
median = cv2.medianBlur(img, 5)
```

Edge Detection (에지 검출):

```python
# Canny Edge Detection
edges = cv2.Canny(gray, threshold1=50, threshold2=150)

# Sobel Operator
sobel_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
sobel_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
```

에지는 물체의 윤곽과 밝기 변화가 큰 경계를 드러낸다. Canny 검출 결과는 threshold 값에 민감하므로, 같은 이미지에서 값을 바꾸며 누락되는 경계와 남는 노이즈를 비교한다.

> 추천 자료
> - [First Principles of Computer Vision — Edge Detection](https://www.youtube.com/playlist?list=PL2zRqk16wsdoCCLpouGuRbcJFBVVJlvgr) — 에지 검출의 수학적 원리를 시각적으로 설명
> - [OpenCV 필터링 튜토리얼](https://docs.opencv.org/4.x/d4/d13/tutorial_py_filtering.html) — 코드와 함께 바로 따라할 수 있다

> 실습: [Canny Edge Detection](https://alexjunholee.github.io/robotics-practice/app.html#canny_edge)
> Canny 에지 검출기의 threshold 파라미터를 실시간으로 조절하며 결과 변화를 확인할 수 있다.

> 실습: [Convolution 시각화](https://alexjunholee.github.io/robotics-practice/app.html#convolution)
> 여러 커널을 이미지에 적용하며 convolution(합성곱)으로 필터링이 되는 과정을 확인한다.

### 9.1.3 Morphology

Morphology(형태학적 연산)는 이진 이미지(binary image)의 형태를 다듬는다. Segmentation 결과에서 작은 노이즈 점을 제거하거나 끊어진 영역을 잇는 데 쓴다.

```python
kernel = np.ones((5, 5), np.uint8)

# Erosion (침식)
eroded = cv2.erode(binary_img, kernel, iterations=1)

# Dilation (팽창)
dilated = cv2.dilate(binary_img, kernel, iterations=1)

# Opening (침식 → 팽창): 노이즈 제거
opening = cv2.morphologyEx(binary_img, cv2.MORPH_OPEN, kernel)

# Closing (팽창 → 침식): 구멍 채우기
closing = cv2.morphologyEx(binary_img, cv2.MORPH_CLOSE, kernel)
```

Opening은 먼저 깎고(erosion) 다시 키우므로(dilation) 작은 돌기와 노이즈를 없앤다. Closing은 먼저 키우고 다시 깎아 작은 구멍을 메운다.

> 추천 자료
> - [OpenCV Morphological Operations](https://docs.opencv.org/4.x/d9/d61/tutorial_py_morphological_ops.html) — 시각적 예제와 함께 설명
> - [First Principles of Computer Vision — Binary Image Processing](https://www.youtube.com/watch?v=IcBzsP-fvPo) — 형태학적 연산의 원리

---

## 9.2 카메라 모델 (Camera Model)

2D 이미지에서 3D를 복원하려면 카메라가 3D 점을 픽셀로 옮기는 방식을 알아야 한다. SLAM과 3D reconstruction(3차원 복원)은 카메라 모델에서 시작한다.

### 9.2.1 Pinhole Model

Pinhole Model(핀홀 모델)은 렌즈 없이 작은 구멍 하나로 3D 점을 2D 이미지에 투영하는 이상적인 카메라 모델이다. 픽셀 좌표 (u, v)에서 3D 위치를 거꾸로 계산하려면 이 투영 관계를 알아야 한다.

투영 방정식:

```
[u]   [f_x  0   c_x] [X/Z]
[v] = [0   f_y  c_y] [Y/Z]
[1]   [0    0    1 ] [ 1 ]
```

(X, Y, Z)는 카메라 좌표계로 나타낸 3D 점이다.

Intrinsic Parameters (내부 파라미터):
- f_x, f_y: Focal length (픽셀 단위)
- c_x, c_y: Principal point (광축이 영상면과 만나는 점이며, 이미지 중심과 다를 수 있음)
- Intrinsic Matrix K (3×3)

Extrinsic Parameters (외부 파라미터):
- R: 회전 행렬 (3×3)
- t: 이동 벡터 (3×1)
- World → Camera 변환

K는 카메라의 렌즈 특성을, [R|t]는 카메라가 세상 어디에 어떤 방향으로 놓여 있는지를 나타낸다. 이 둘을 곱하면 3D 점이 2D 픽셀로 매핑된다.

> 추천 자료
> - [Stanford CS231A — Camera Models](https://web.stanford.edu/class/cs231a/) — 기하 기반 CV의 핵심 강의
> - [First Principles of CV — Camera and Imaging](https://www.youtube.com/playlist?list=PL2zRqk16wsdoYzrWStQ2SQHXXS2K6ofd4) — Pinhole부터 실제 렌즈까지 차근차근 설명
> - [Szeliski Ch.2 — Image Formation](https://szeliski.org/Book/) — 카메라 모델의 수학적 기초
> - [정진용 블로그 — Camera Models and Distortion (Perspective, Fisheye, Omni)](https://jinyongjeong.github.io/2020/06/15/Camera_and_distortion_model/) — Perspective, Equidistant, Omni 카메라 모델 비교 정리
> - [정진용 블로그 — OpenCV Camera model 정리](https://jinyongjeong.github.io/2020/06/19/SLAM-Opencv-Camera-model-%EC%A0%95%EB%A6%AC/) — OpenCV의 핀홀/어안 카메라 모델 구현 기준 정리

> 실습: [Camera Projection](https://alexjunholee.github.io/robotics-practice/app.html#camera_projection)
> 3D 공간의 점이 카메라 내부/외부 파라미터로 2D 이미지에 투영되는 과정을 인터랙티브하게 확인할 수 있다.

### 9.2.2 Distortion Models

실제 렌즈는 Pinhole Model에서 벗어난다. 광각 렌즈나 fisheye 렌즈에서는 직선이 곡선으로 휘어 보인다. 왜곡을 보정하지 않은 영상을 쓰면 SLAM 정확도가 떨어지고 3D reconstruction이 휜다.

빛은 렌즈를 지나며 굴절하고, 이 굴절이 이미지에 왜곡으로 나타난다.

Radial distortion (방사 왜곡): 이미지 중심에서 멀어질수록 심해진다. 파라미터 k1, k2, k3로 모델링한다. k1 < 0이면 barrel distortion (직선이 바깥으로 볼록), k1 > 0이면 pincushion distortion (직선이 안쪽으로 오목). 광각 렌즈일수록 barrel distortion이 심하다.

Tangential distortion (접선 왜곡): 렌즈가 이미지 센서와 완전히 평행하지 않을 때 생긴다. 파라미터는 p1, p2이고, 보통 radial보다 영향이 작다. 렌즈와 센서의 정렬 공차가 큰 모듈에서는 p1, p2도 추정한다.

왜곡 보정:

```python
# 단순 보정 (매 프레임 계산 — 느림)
undistorted = cv2.undistort(distorted, K, dist_coeffs)

# 보정 맵 미리 계산 후 재사용 (SLAM 파이프라인의 일반적 패턴)
map1, map2 = cv2.initUndistortRectifyMap(K, dist_coeffs, None, K, (w, h), cv2.CV_32FC1)
undistorted = cv2.remap(distorted, map1, map2, cv2.INTER_LINEAR)
```

고정된 카메라 파라미터라면 매 프레임 `cv2.undistort()`를 호출하는 대신 `initUndistortRectifyMap()`으로 맵을 미리 계산하고 `cv2.remap()`으로 재사용할 수 있다. 실시간 파이프라인에서 계산량을 줄일 때 흔히 쓰는 방법이다.

**어안 렌즈 (Fisheye)**: 화각이 매우 넓은 어안 렌즈는 저차 radial-tangential 핀홀 모델로 근사하기 어렵다. 어안 모델은 빛의 입사각 θ로 투영을 표현한다. equidistant 모델은 r = f·θ다. OpenCV에는 이 모델용 `cv2.fisheye` 모듈이 있다. 모델은 렌즈 투영 방식과 별도 검증 영상의 재투영 잔차를 보고 고른다.

(참고: [다크 프로그래머 — 카메라 왜곡보정](https://darkpgmr.tistory.com/31), [정진용 블로그 — Camera Models and Distortion](https://jinyongjeong.github.io/2020/06/15/Camera_and_distortion_model/))

> 추천 자료
> - [OpenCV Camera Calibration and 3D Reconstruction](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html) — 왜곡 모델의 수식이 잘 정리되어 있다
> - [First Principles of CV — Lens Related Issues](https://www.youtube.com/watch?v=hzOeqCb2Fg4) — 렌즈 왜곡이 왜 생기는지 물리적 직관 설명

> 실습: [Lens Distortion 시각화](https://alexjunholee.github.io/robotics-practice/app.html#lens_distortion)
> Radial/Tangential 왜곡 파라미터를 조절하며 이미지가 어떻게 변형되는지 직접 확인할 수 있다.

### 9.2.3 캘리브레이션 (Calibration)

캘리브레이션은 카메라의 내부·외부 파라미터를 추정하는 과정이다. K (intrinsic matrix)와 왜곡 계수를 알아야 카메라 모델을 실제 영상에 쓸 수 있다. 캘리브레이션이 부정확하면 이 값을 쓰는 SLAM, 스테레오 깊이 추정, hand-eye calibration(로봇 손과 카메라 사이 변환 추정)의 정확도가 모두 떨어진다.

체커보드 방식:

```python
# 체커보드 코너 검출
ret, corners = cv2.findChessboardCorners(gray, (9, 6), None)

# 코너 정밀화
corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

# 캘리브레이션
ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
    object_points, image_points, gray.shape[::-1], None, None
)
```

캘리브레이션 영상은 장수보다 관측 가능성으로 평가한다. 타겟이 이미지 중앙과 가장자리를 덮고, 거리와 여러 축의 기울기가 달라지도록 촬영한다. 전체 RMS 하나뿐 아니라 이미지별·위치별 잔차와 별도 검증 영상의 오차를 확인한다.

체커보드 패턴을 여러 각도에서 촬영하면, 체커보드의 3D 좌표(알고 있음)와 이미지의 2D 좌표(검출함)의 대응 쌍이 수십~수백 개 생긴다. 이 대응 쌍으로부터:

1. Intrinsic parameters (f_x, f_y, c_x, c_y): 초점거리와 주점. 초점·줌·해상도·crop, 온도와 기계적 조립이 바뀌면 다시 확인한다.
2. Distortion coefficients (k1, k2, p1, p2, k3): 선택한 투영 모델에서 렌즈와 조립의 왜곡을 근사하는 계수다. 크기는 캘리브레이션으로 확인한다.
3. Extrinsic parameters (R, t): 영상마다 체커보드에 대한 카메라 자세. 캘리브레이션 자체에서는 부산물이지만, hand-eye calibration 등에서 별도로 쓰인다.

필요한 장수는 parameter uncertainty와 conditioning, 검출 품질에 따라 달라진다. Reprojection error의 기대 범위도 해상도, 렌즈 모델, 타겟, corner detector에 의존하므로 카메라마다 기준을 따로 잡는다. 큰 residual 이미지는 자동 삭제하기 전에 blur·반사·검출 실패를 확인하고, 제외 전후의 parameter 안정성과 held-out error를 비교한다.

(참고: [다크 프로그래머 — 카메라 캘리브레이션](https://darkpgmr.tistory.com/32))

Kalibr: 멀티 카메라, Camera-IMU 캘리브레이션 도구
- ROS 기반
- 권장 타겟은 AprilGrid(AprilTag 격자)이고, 체커보드와 원형 격자도 지원
- 카메라와 IMU 사이 시간 오프셋까지 추정

> 추천 자료
> - [OpenCV 카메라 캘리브레이션 튜토리얼](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html) — 체커보드 캘리브레이션 step-by-step
> - [Kalibr 공식 Wiki](https://github.com/ethz-asl/kalibr/wiki) — 널리 쓰이는 공개 Camera-IMU 캘리브레이션 도구
> - [Zhang, "A Flexible New Technique for Camera Calibration" (2000)](https://www.microsoft.com/en-us/research/publication/a-flexible-new-technique-for-camera-calibration/) — 현재 OpenCV 캘리브레이션의 기반이 되는 논문
> - [Tangram Vision Blog](https://www.tangramvision.com/blog) — 카메라 캘리브레이션, 센서 퓨전 등 실전 엔지니어링 글 모음

---

## 9.3 특징점 (Features)

특징점은 이미지에서 구별되는 점(keypoint)과 그 주변을 설명하는 벡터(descriptor)를 말한다. SLAM과 Visual Odometry(시각 주행거리 추정)는 카메라가 움직이는 동안 이미지 사이에서 같은 점을 찾아야 한다. 특징점은 그 대응점을 안정적으로 찾는 수단이다.

### 9.3.1 Keypoint Detection

Harris Corner:
- 코너 검출의 고전적 방법
- 속도 느림, 스케일 변화에 취약

FAST (Features from Accelerated Segment Test):
- 매우 빠른 코너 검출
- 실시간 시스템에 적합
- 스케일 불변 아님

ORB (Oriented FAST and Rotated BRIEF):
- FAST 검출 + BRIEF 디스크립터 + 방향 정보
- 특허 제약 없음
- 실시간 SLAM에서 널리 사용

ORB-SLAM 시리즈는 ORB를 사용한다.

SIFT (Scale-Invariant Feature Transform):
- 스케일, 회전 불변
- 높은 반복성
- 계산 비용 높음 (특허 US6711293이 2020년 3월 만료, OpenCV 4.4.0부터 main 모듈에 포함)

Lowe는 1999년 ICCV 논문에서 SIFT를 처음 발표했고, 2004년 IJCV 논문으로 정리했다. SURF와 ORB 논문은 SIFT를 비교 대상으로 삼는다.

SuperPoint (딥러닝 기반):
- Self-supervised 학습
- 높은 반복성과 정확도
- GPU 필요

> 추천 자료
> - [Lowe, "Distinctive Image Features from Scale-Invariant Keypoints" (2004)](https://www.cs.ubc.ca/~lowe/papers/ijcv04.pdf) — SIFT 원논문
> - [Rublee et al., "ORB: An efficient alternative to SIFT or SURF" (2011)](https://ieeexplore.ieee.org/document/6126544) — ORB 원논문
> - [First Principles of CV — Feature Detection](https://www.youtube.com/playlist?list=PL2zRqk16wsdqXEMpHrc4Qnb5rA1Cylrhx) — 특징점 검출의 원리를 시각적으로
> - [DeTone et al., "SuperPoint: Self-Supervised Interest Point Detection and Description" (2018)](https://arxiv.org/abs/1712.07629) — 자기지도 학습 기반 특징점 검출·기술 논문
> - [다크 프로그래머 — 영상 특징점(keypoint) 추출방법](https://darkpgmr.tistory.com/131) — SIFT, HOG, Haar, Ferns, LBP, MCT 등 특징점 비교 정리

### 9.3.2 Descriptor

Descriptor는 keypoint 주변 패턴을 숫자로 적은 벡터다. 두 이미지에서 같은 물리적 점을 찾을 때 이 벡터끼리 비교한다.

BRIEF (Binary Robust Independent Elementary Features):
- 이진 디스크립터 (0 or 1)
- 빠른 매칭 (Hamming distance)
- 회전 불변 아님

ORB Descriptor:
- BRIEF + 방향 정보
- 256비트 이진 벡터

이진 디스크립터의 장점은 매칭 속도이다. 두 디스크립터 간 거리를 Hamming distance (XOR 연산)로 계산하기 때문에 SIFT의 유클리드 거리 비교보다 빠르다. 연산 자원이 적은 임베디드 보드에서는 이 차이가 처리 속도를 좌우한다.

SuperGlue (딥러닝 기반):
- Graph Neural Network 기반 매칭
- 반복 패턴, 적은 텍스처에서도 강건
- LightGlue: 경량화 버전

> 추천 자료
> - [Sarlin et al., "SuperGlue: Learning Feature Matching with Graph Neural Networks" (2020)](https://arxiv.org/abs/1911.11763) — SuperGlue 원논문

### 9.3.3 Feature Matching

Feature matching 결과를 보면 `tracking lost`가 대응점 부족에서 시작됐는지, 잘못된 대응에서 시작됐는지 구분할 수 있다.

```python
# ORB 특징점 및 디스크립터 추출
orb = cv2.ORB_create()
kp1, des1 = orb.detectAndCompute(img1, None)
kp2, des2 = orb.detectAndCompute(img2, None)

# BFMatcher (Brute-Force)
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
matches = bf.match(des1, des2)

# Ratio Test (Lowe's ratio)
bf = cv2.BFMatcher(cv2.NORM_HAMMING)
matches = bf.knnMatch(des1, des2, k=2)
good = [p[0] for p in matches if len(p) == 2 and p[0].distance < 0.75 * p[1].distance]
```

Lowe's ratio test는 kNN으로 가장 가까운 매치 두 개를 찾고, 첫째와 둘째 거리의 비가 threshold 이하인 것만 남긴다. 두 거리가 비슷한 애매한 매치를 이 과정에서 걸러낸다. Lowe(2004)는 비가 0.8보다 큰 매치를 버렸고, 위 예제는 0.75를 쓴다. 값은 데이터에 맞춰 0.6~0.8 사이에서 조절한다.

> 추천 자료
> - [OpenCV Feature Matching](https://docs.opencv.org/4.x/dc/dc3/tutorial_py_matcher.html) — BFMatcher, FLANN, Ratio Test 예제
> - [Computerphile — SIFT Features](https://www.youtube.com/watch?v=ram-jbLJjFg) — 특징점 매칭의 직관적 설명

> 실습: [Feature Matching](https://alexjunholee.github.io/robotics-practice/app.html#feature_matching)
> 두 이미지 간 특징점 매칭과 Lowe's ratio test 적용 과정을 인터랙티브하게 실험할 수 있다.

---

## 9.4 에피폴라 기하학 (Epipolar Geometry)

에피폴라 기하학은 두 카메라 시점 사이의 기하 관계다. 두 장의 사진에서 같은 물체를 보고 카메라의 상대 자세를 구하고, 이어서 3D 구조를 복원한다. Visual Odometry와 SfM(Structure from Motion, 여러 영상에서 3D 구조와 카메라 위치를 함께 추정)이 이 관계를 쓴다. 계산에는 SVD와 고유값 분해가 들어간다.

### 9.4.1 Essential Matrix (E)

정의: 캘리브레이션된 카메라 쌍의 상대 자세를 담은 3×3 행렬

```
x2^T E x1 = 0
```

- x1, x2: 정규화된 이미지 좌표
- E = [t]_× R (t의 skew-symmetric 행렬 × R)
- R, t: 카메라 1 좌표의 점을 카메라 2 좌표로 옮기는 회전과 이동 (X2 = R X1 + t)

5-point 알고리즘: 최소 5쌍의 대응점으로 E 추정 (RANSAC과 함께 사용)

Essential Matrix를 R과 t로 분해(decompose)한 뒤 양의 깊이 조건으로 후보를 고르면 두 카메라 사이의 상대 회전과 이동 방향을 알 수 있다. 이동의 크기를 알려면 별도의 척도 정보가 필요하다. 단안 Visual Odometry는 이 분해로 프레임 간 움직임을 구한다.

> 실습: [Epipolar Geometry 시각화](https://alexjunholee.github.io/robotics-practice/app.html#epipolar)
> 두 카메라 시점의 에피폴라 선과 에피폴을 조작하며 Essential/Fundamental Matrix가 무엇을 나타내는지 확인한다.

### 9.4.2 Fundamental Matrix (F)

정의: 캘리브레이션되지 않은 카메라 쌍의 관계

```
p2^T F p1 = 0
```

- p1, p2: 픽셀 좌표
- F = K2^(-T) E K1^(-1)

8-point 알고리즘: 8쌍 이상의 대응점으로 F를 선형 추정한다. F의 자유도가 7이라 최소 해법은 7점 알고리즘이다(OpenCV `cv2.FM_7POINT`).

F는 픽셀 좌표에 바로 쓰고, E는 내부 파라미터 K로 정규화한 좌표에 쓴다. 캘리브레이션을 했다면 E를, 안 했다면 F를 쓴다.

### 9.4.3 Triangulation

Triangulation(삼각측량)은 두 카메라, 또는 움직인 카메라 하나에서 같은 점을 관측했을 때 그 점의 3D 위치를 계산한다.

```python
# OpenCV triangulation
points_4d = cv2.triangulatePoints(P1, P2, pts1, pts2)
points_3d = points_4d[:3] / points_4d[3]  # Homogeneous → Cartesian
```

주의: baseline (두 카메라 간 거리)이 너무 작으면 삼각측량 정확도가 떨어지고, 너무 크면 같은 점을 양쪽에서 동시에 관측하기 어려워진다.

> 추천 자료
> - [Stanford CS231A — Epipolar Geometry](https://web.stanford.edu/class/cs231a/) — 수학적 유도가 잘 정리된 강의 자료
> - [Hartley & Zisserman, "Multiple View Geometry in Computer Vision"](https://www.robots.ox.ac.uk/~vgg/hzbook/) — 다중 시점 기하학 교재
> - [First Principles of CV — Stereo Vision](https://www.youtube.com/playlist?list=PL2zRqk16wsdoYzrWStQ2SQHXXS2K6ofd4) — Epipolar geometry를 직관적으로 설명
> - [다크 프로그래머 — 영상 Geometry 시리즈 (7편: 좌표계~Epipolar)](https://darkpgmr.tistory.com/77) — 좌표계, Homogeneous, 2D/3D 변환, Homography, Imaging, Epipolar Geometry를 한글로 체계적 정리

> 실습: [Homography 시각화](https://alexjunholee.github.io/robotics-practice/app.html#homography)
> 평면 간 호모그래피 변환을 인터랙티브하게 조작하며, 4개의 대응점으로 투영 변환이 어떻게 결정되는지 확인할 수 있다.

---

## 9.5 광학 흐름 (Optical Flow)

Optical flow(광학 흐름)는 연속 프레임 사이에서 각 픽셀이 어디로 움직였는지 추정한다. Visual Odometry 자세 추정과 움직이는 물체 검출에 쓴다. Feature matching이 sparse한 점만 다루는 반면, dense optical flow는 모든 픽셀의 움직임을 추정한다.

### 9.5.1 Lucas-Kanade Method

- Sparse optical flow (특정 점들만)
- 밝기 불변 가정
- 작은 움직임 가정

```python
# 광류 계산
p1, status, err = cv2.calcOpticalFlowPyrLK(
    prev_gray, curr_gray, p0, None, **lk_params
)
```

"PyrLK"에서 "Pyr"는 Pyramid를 뜻한다. 저해상도 피라미드 단계에서는 큰 움직임도 몇 픽셀로 줄어들어, Lucas-Kanade의 작은 움직임 가정이 맞는다.

### 9.5.2 Dense Optical Flow

- 모든 픽셀의 움직임 계산
- Farneback, RAFT (딥러닝)

```python
# Farneback dense flow
flow = cv2.calcOpticalFlowFarneback(prev_gray, curr_gray, None, 0.5, 3, 15, 3, 5, 1.2, 0)
```

RAFT(Recurrent All-Pairs Field Transforms)는 학습 기반 dense optical flow 연구에서 자주 비교 대상이 된다. 2020년 원 논문은 KITTI와 Sintel 벤치마크에서 당시 기존 방법보다 낮은 광류 추정 오류를 보고했다.

> 추천 자료
> - [First Principles of CV — Optical Flow](https://www.youtube.com/playlist?list=PL2zRqk16wsdp8KbDfHKvPYNGF2L-zQASc) — 광류의 수학적 원리
> - [Teed & Deng, "RAFT: Recurrent All-Pairs Field Transforms for Optical Flow" (2020)](https://arxiv.org/abs/2003.12039) — RAFT 원논문
> - [Huang et al., "FlowFormer: A Transformer Architecture for Optical Flow" (ECCV 2022, arXiv:2203.16194)](https://arxiv.org/abs/2203.16194) — Transformer 기반 optical flow
> - [OpenCV Optical Flow 튜토리얼](https://docs.opencv.org/4.x/d4/dee/tutorial_optical_flow.html) — Lucas-Kanade, Farneback 코드 예제

> 실습: [Optical Flow 시각화](https://alexjunholee.github.io/robotics-practice/app.html#optical_flow)
> Lucas-Kanade와 Dense Optical Flow 알고리즘의 동작을 인터랙티브하게 비교하며 픽셀 이동 추정 과정을 확인할 수 있다.

---

## 9.6 심화: PnP 문제

Perspective-n-Point (PnP)는 3D 공간의 점과 2D 이미지의 대응점이 주어졌을 때 카메라 포즈(회전 R + 이동 t)를 추정하는 문제다. SLAM의 프레임별 카메라 tracking과 AR의 마커 기반 위치 추정이 PnP를 푼다.

문제 정의: n개의 3D-2D 대응 {(X_i, x_i)}가 주어졌을 때, 카메라 외부 파라미터 [R|t]를 추정한다.

$$x_i = K [R | t] X_i$$

여기서 K는 카메라 내부 파라미터(intrinsics)이다. x_i와 X_i는 동차 좌표이고, 양변은 스케일 배 차이를 빼고 같다.

P3P (3-Point Problem):
- 최소 3개의 대응점으로 풀 수 있다.
- 3점으로 최대 4개의 해가 나온다. 4번째 점으로 그중 하나를 고른다.
- RANSAC과 함께 쓰면 outlier(잘못된 대응)에 강하다.

EPnP (Efficient PnP):
- O(n) 복잡도로, 대응점이 많을 때 효율적이다.
- 3D 점들을 4개의 가상 제어점(virtual control points)으로 표현하고, 이 제어점의 카메라 좌표를 추정하는 방식이다.
- 원 논문 실험에서 다른 비반복 해법(AD, Clamped DLT)보다 정확했고, 반복 해법(LHM)보다 훨씬 빠르면서 정확도는 조금만 낮았다. Gauss-Newton으로 한 번 다듬으면(EPnP+GN) 그 차이도 거의 없어진다.

실무 사용법:

```python
import cv2
import numpy as np

# 3D 월드 좌표 (n x 3)
object_points = np.array([...], dtype=np.float64)
# 대응하는 2D 이미지 좌표 (n x 2)
image_points = np.array([...], dtype=np.float64)
# 카메라 내부 파라미터
camera_matrix = np.array([[fx, 0, cx],
                          [0, fy, cy],
                          [0,  0,  1]], dtype=np.float64)
dist_coeffs = np.zeros(4)

# EPnP: 비반복 폐쇄형 해법이라 초기값이 필요 없다
success, rvec, tvec = cv2.solvePnP(
    object_points, image_points, camera_matrix, dist_coeffs,
    flags=cv2.SOLVEPNP_EPNP
)

# RANSAC 버전 — outlier가 있을 때 필수
success, rvec, tvec, inliers = cv2.solvePnPRansac(
    object_points, image_points, camera_matrix, dist_coeffs,
    iterationsCount=1000, reprojectionError=3.0
)
```

Visual SLAM은 프레임마다 다음 과정을 거친다.
1. 이전 프레임에서 삼각측량(triangulation)으로 3D 맵 포인트를 만든다.
2. 새 프레임에서 해당 맵 포인트의 2D 재투영(reprojection)을 예측한다.
3. 실제 관측된 2D 키포인트와 매칭한다.
4. 이 3D-2D 대응으로 PnP를 풀어 새 프레임의 카메라 포즈를 구한다.

ORB-SLAM3의 Tracking도 대체로 이 흐름을 따른다. 다만 단안 모드에서 키프레임 사이의 삼각측량으로 새 맵 포인트를 만드는 일은 주로 Local Mapping이 담당한다(스테레오·RGB-D는 키프레임 삽입 시 Tracking이 근거리 점을 바로 만든다).

> 추천 자료
> - [Lepetit et al., "EPnP: An Accurate O(n) Solution to the PnP Problem" (2009)](https://doi.org/10.1007/s11263-008-0152-6) — EPnP 원논문
> - [OpenCV solvePnP 문서](https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html) — 다양한 PnP 알고리즘 flag 설명
> - [Multiple View Geometry — Ch. 7](https://www.robots.ox.ac.uk/~vgg/hzbook/) — PnP 문제의 수학적 배경

solvePnP 실전 팁

OpenCV의 `cv2.solvePnP()`는 3D-2D 대응점으로 카메라 포즈를 추정한다. 반환하는 `rvec`은 Rodrigues 벡터(축-각 표현)이다.

```python
# rvec → 회전 행렬 변환
R, _ = cv2.Rodrigues(rvec)

# 카메라의 월드 좌표 위치
camera_position = -R.T @ tvec
```

주의할 점:
- `solvePnP`의 결과는 세계→카메라 변환이다. 카메라의 세계 좌표 위치를 구하려면 역변환을 해야 한다.
- `solvePnP`는 일반적으로 최소 4점을 받는다(P3P도 해를 하나로 고르려면 4번째 점이 필요하다). 기본값 `SOLVEPNP_ITERATIVE`는 초기값을 DLT로 구하므로, 3D 점들이 한 평면 위에 있지 않으면 6점 이상이 필요하다. 점이 많을수록 노이즈에 강건하다. RANSAC 버전인 `cv2.solvePnPRansac()`을 쓰면 outlier를 자동으로 걸러준다.
- `flags` 파라미터로 알고리즘을 고른다: `cv2.SOLVEPNP_ITERATIVE` (기본, LM), `cv2.SOLVEPNP_P3P` (`solvePnP`에서는 정확히 4점), `cv2.SOLVEPNP_EPNP` (빠르고 안정적, 많은 점에 적합).

Rodrigues 벡터의 방향이 회전축, 크기(norm)가 회전 각도다. Ch.3 §3.5.2–3.5.3의 so(3)에서 다룬 축-각 표현과 같다. `cv2.Rodrigues()`는 exp/log map의 구현이다.

(참고: [다크 프로그래머 — solvePnP 함수 사용법과 Rodrigues 표현법](https://darkpgmr.tistory.com/99))

---

## 9.7 심화: RANSAC 변종

RANSAC(RANdom SAmple Consensus)의 기본 알고리즘과 변종 비교표는 Ch.3 §3.7.3에 있다. OpenCV는 이 변종들을 USAC 프레임워크로 묶어 두었다.

Lo-RANSAC (Locally Optimized RANSAC):
- 좋은 모델을 찾으면, 그 모델의 inlier들로 다시 모델을 추정(local optimization)한다.
- inlier 비율이 높으면 local optimization이 실행 시간의 대부분을 차지할 수 있다(Lebeda, Matas, Chum, BMVC 2012).

PROSAC (Progressive Sample Consensus):
- 매칭 스코어가 높은 대응점부터 우선적으로 샘플링한다.
- 좋은 매칭이 앞에 많으면 초기 iteration에서 바로 좋은 모델을 찾는다.

MAGSAC++ (Marginalizing Sample Consensus):
- 노이즈 스케일 σ(inlier 임계값을 정하는 값)를 하나로 정하지 않고 여러 값에 대해 marginalize(주변화)한다.
- 여러 σ 값에 걸쳐 적분하므로 임계값 선택에 덜 민감하다. σ의 상한은 지정한다.
- OpenCV에서는 `USAC_MAGSAC` 옵션으로 사용할 수 있다.

OpenCV에서 MAGSAC++ 사용:

```python
import cv2

# Fundamental matrix 추정에 MAGSAC++ 사용
F, mask = cv2.findFundamentalMat(
    pts1, pts2,
    method=cv2.USAC_MAGSAC,
    ransacReprojThreshold=1.0,
    confidence=0.999,
    maxIters=10000
)

# Homography 추정에도 같은 방식으로 쓴다
H, mask = cv2.findHomography(
    src_pts, dst_pts,
    method=cv2.USAC_MAGSAC,
    ransacReprojThreshold=3.0
)
```

실무 팁:
- Iteration 수: `confidence` 파라미터로 제어한다. 0.999는 주어진 inlier 비율에서 "전부 inlier인 최소 표본을 적어도 한 번 뽑을 확률이 99.9%"가 되도록 반복 수를 정한다는 뜻이며, 최종 채택 모델이 옳을 확률은 아니다. 필요한 반복 수는 N = log(1−p)/log(1−w^s) (w: inlier 비율, s: 최소 표본 크기)이므로 w가 낮을수록 빠르게 늘어난다.
- Threshold: MAGSAC++를 쓰면 threshold에 덜 민감하지만, 초기값은 여전히 줘야 한다. 예시 코드의 값은 시작점일 뿐이며 영상 해상도와 노이즈에 맞춰 검증해야 한다.
- PROSAC과 MAGSAC++의 속도·정확도 관계는 매칭 점수의 품질, outlier 비율, 구현에 따라 달라진다. 같은 데이터와 시간 예산으로 비교해 고른다.

> 추천 자료
> - [Barath et al., "MAGSAC++, a Fast, Reliable and Accurate Robust Estimator" (2020)](https://arxiv.org/abs/1912.05909) — MAGSAC++ 원논문
> - [OpenCV USAC 문서](https://docs.opencv.org/4.x/de/d3e/tutorial_usac.html) — OpenCV의 universal RANSAC 프레임워크
> - [Chum & Matas, "Matching with PROSAC" (2005)](https://doi.org/10.1109/CVPR.2005.221) — PROSAC 원논문

---

## 9.8 심화: 학습 기반 특징 매칭

ORB, SIFT 같은 hand-crafted feature(사람이 설계한 특징)는 반복 패턴, 텍스처 부족, 극단적 조명 변화에서 자주 실패한다. 2018년 SuperPoint 이후 학습 기반 특징 추출과 매칭 연구가 늘었다.

파이프라인 발전 과정:

```
SuperPoint (2018) → SuperGlue (2020) → LightGlue (2023)
  [키포인트 검출+기술]    [그래프 신경망 매칭]     [경량화된 매칭]
```

SuperPoint:
- 자기지도 학습으로 키포인트 검출기와 디스크립터를 동시에 학습한다.
- Homographic adaptation: 합성 변환을 적용하고 역변환해 pseudo ground truth를 생성한다.
- 원 논문의 평가 조건에서는 여러 고전 방법보다 높은 repeatability를 보였다.

SuperGlue:
- 두 이미지의 키포인트를 그래프로 보고, attention mechanism으로 매칭한다.
- Self-attention으로 같은 이미지 안 키포인트 관계를, cross-attention으로 두 이미지 사이 관계를 학습한다.
- Sinkhorn algorithm으로 최적 할당(optimal assignment) 문제를 푼다.
- 원 논문의 평가에서는 강한 매칭 성능을 보였지만, 연산량과 지연은 입력 특징 수와 하드웨어에 따라 달라진다.

LightGlue:
- SuperGlue의 경량 버전. Adaptive early stopping으로 쉬운 이미지 쌍은 빨리, 어려운 쌍은 더 많은 layer를 통과시킨다.
- 원 논문의 평가에서는 adaptive depth·point pruning을 사용해 SuperGlue보다 낮은 지연으로 유사한 정확도를 보였다. 배수는 하드웨어와 설정에 따라 달라진다.

LoFTR (Detector-Free Local Feature Matching):
- 키포인트 검출 단계 없이 이미지 전체에서 dense matching을 한다.
- Transformer 기반으로 coarse-to-fine 매칭을 한다.
- 텍스처가 부족한 영역에서도 매칭이 가능하다는 것이 가장 큰 장점이다.
- 단점: 속도가 느리고 GPU 메모리를 많이 사용한다.

고전 방법 vs 학습 기반 비교:

| 항목 | ORB/SIFT | SuperPoint+LightGlue | LoFTR |
|------|----------|---------------------|-------|
| 속도 (CPU) | 빠름 | 느림 | 매우 느림 |
| 속도 (GPU) | 해당 없음 | 보통 | 느림 |
| 텍스처 부족 영역 | 자주 실패 | 보통 | 강함 |
| 반복 패턴 | 약함 | 강함 | 강함 |
| GPU 의존성 | 없음 | 높음 | 매우 높음 |
| 실시간 로봇 적용 | 용이 | 조건부 가능 | 어려움 |

코드 예시 — LightGlue (kornia 사용):

```python
import torch
import kornia
from kornia.feature import LightGlueMatcher, KeyNetAffNetHardNet

# 특징 추출기 + 매칭기 구성
extractor = KeyNetAffNetHardNet(num_features=2048).eval()
matcher = LightGlueMatcher("keynet_affnet_hardnet").eval()

# GPU로 이동
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
extractor = extractor.to(device)
matcher = matcher.to(device)

# 이미지 로드 (kornia 형식: B x C x H x W, 0-1 범위; KeyNet은 1채널 grayscale 입력)
img0 = kornia.io.load_image(path0, kornia.io.ImageLoadType.GRAY32).unsqueeze(0).to(device)
img1 = kornia.io.load_image(path1, kornia.io.ImageLoadType.GRAY32).unsqueeze(0).to(device)

# 특징 추출
with torch.no_grad():
    lafs0, resp0, desc0 = extractor(img0)   # (lafs, responses, descriptors) 튜플
    lafs1, resp1, desc1 = extractor(img1)

    # 매칭 — LightGlue는 디스크립터와 함께 키포인트 기하(lafs)를 받는다
    dists, match_idxs = matcher(desc0[0], desc1[0], lafs0, lafs1)
```

LightGlue 계열(SuperPoint+LightGlue, 위 예제의 KeyNetAffNetHardNet+LightGlue)은 GPU가 있고 학습 기반 매칭의 강건성이 필요할 때 검토한다. GPU가 없거나 지연·전력 예산이 작다면 ORB 같은 고전 특징이 더 단순한 기준선이 된다. 최종 선택은 목표 데이터에서 정확도, 지연, 메모리를 함께 측정해 결정한다.

> 추천 자료
> - [DeTone et al., "SuperPoint: Self-Supervised Interest Point Detection and Description" (2018)](https://arxiv.org/abs/1712.07629) — SuperPoint 원논문
> - [Lindenberger et al., "LightGlue: Local Feature Matching at Light Speed" (2023)](https://arxiv.org/abs/2306.13643) — LightGlue 원논문
> - [Sun et al., "LoFTR: Detector-Free Local Feature Matching with Transformers" (2021)](https://arxiv.org/abs/2104.00680) — LoFTR 원논문

---

> 기술 흐름: 컴퓨터 비전 기초 (Classical Methods)
> - **~2004**: 고전적 특징점의 시대. Harris Corner (1988), SIFT (1999/2004) 등 hand-crafted feature가 주류. 수학적으로 정교하지만 계산이 무거움
> - **2006~2011**: 실시간을 위한 경량화. SURF (2006), FAST (2006), BRIEF (2010), ORB (2011) 등이 등장. ORB는 특허 제약이 없는 SIFT·SURF 대안으로 제안됨
> - **2015~2019**: 학습 기반 특징점. SuperPoint (2018)가 등장
> - **2020~**: 기하와 학습의 결합. SuperGlue (2020) 같은 학습 기반 매처, LoFTR (2021) 같은 detector-free matching, LightGlue (2023) 같은 경량 학습 매칭이 등장. SLAM/VO 백엔드는 여전히 고전 기하를 씀
