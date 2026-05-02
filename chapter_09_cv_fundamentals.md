# Ch.9 — 컴퓨터 비전 기초 (Computer Vision Fundamentals)


로봇이 카메라로 들어오는 원시 데이터를 의미 있는 정보로 바꾸는 모든 과정의 뿌리가 여기에 있다. SLAM을 하든 물체를 집든, 이 기초가 흔들리면 "왜 안 되지?"에서 한참을 헤매게 된다.

---

## 9.1 이미지 처리 기초 (Image Processing)

카메라에서 들어오는 raw 이미지는 노이즈가 많고 정보가 정리되지 않은 상태다. 어떤 알고리즘이든 그 위에서 동작하려면 먼저 이미지를 정제해야 한다. 필터링, 에지 검출, 형태학적 연산이 전처리의 기본 도구다. 이걸 모르면 후속 파이프라인에서 결과가 왜 이상한지 잡을 수 없다.

### 9.1.1 OpenCV 소개

OpenCV(Open Source Computer Vision Library)는 가장 널리 사용되는 CV 라이브러리이다.

논문의 알고리즘을 직접 구현하든, 빠르게 프로토타입을 만들든, OpenCV는 거의 항상 거치게 되는 도구다. C++/Python 바인딩이 모두 있어서 연구에서 프로덕션까지 커버한다.

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

주의: OpenCV는 BGR 순서를 사용한다 (RGB 아님). Matplotlib이나 다른 라이브러리와 섞어 쓸 때 색이 뒤집히는 원인이 여기에 있다. `cv2.cvtColor(img, cv2.COLOR_BGR2RGB)`를 습관처럼 쓰자.

> 추천 자료
> - [OpenCV 공식 튜토리얼](https://docs.opencv.org/4.x/d9/df8/tutorial_root.html) — Python/C++ 예제가 잘 정리되어 있다
> - [First Principles of Computer Vision](https://www.youtube.com/channel/UCf0WB91t8Ky6AuYcQV0CcLw) — Columbia의 Shree Nayar 교수의 채널. 이미지 처리 원리를 직관적으로 설명한다
> - [Szeliski, "Computer Vision: Algorithms and Applications"](https://szeliski.org/Book/) — 무료 PDF 제공. CV 분야의 표준 교과서
> - [Stanford CS131 — Computer Vision: Foundations and Applications](http://vision.stanford.edu/teaching/cs131_fall1415/schedule.html) — CS231n보다 기초적인 CV 강의. 이미지 처리부터 시작하고 싶다면 여기서

### 9.1.2 필터링 (Filtering)

이미지에서 원하는 정보를 뽑아내거나, 원치 않는 노이즈를 제거하는 가장 기본적인 도구가 필터링이다. 필터링을 모르면 에지 검출 결과가 지저분해도 원인을 모르고, segmentation 전처리에서 왜 blur를 거치는지 감이 안 온다.

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

에지는 이미지에서 정보량이 가장 많은 부분이다. 사람이 물체를 인식할 때 가장 먼저 보는 것도 에지다. Canny는 가장 널리 쓰이는 에지 검출기인데, threshold 값에 따라 결과가 크게 달라지므로 직접 파라미터를 바꿔가며 실험해봐야 한다.

> 추천 자료
> - [First Principles of Computer Vision — Edge Detection](https://www.youtube.com/playlist?list=PL2zRqk16wsdoCCLpouGuRbcJFBVVJlvgr) — 에지 검출의 수학적 원리를 시각적으로 설명
> - [OpenCV 필터링 튜토리얼](https://docs.opencv.org/4.x/d4/d13/tutorial_py_filtering.html) — 코드와 함께 바로 따라할 수 있다
> - [Papers With Code — Edge Detection](https://paperswithcode.com/task/edge-detection) — 에지 검출 최신 벤치마크와 논문 모음

> 실습: [Canny Edge Detection](https://alexjunholee.github.io/robotics-practice/app.html#canny_edge)
> Canny 에지 검출기의 threshold 파라미터를 실시간으로 조절하며 결과 변화를 확인할 수 있다.

> 실습: [Convolution 시각화](https://alexjunholee.github.io/robotics-practice/app.html#convolution)
> 다양한 커널을 이미지에 적용하며 convolution 연산이 어떻게 필터링을 수행하는지 직관적으로 이해할 수 있다.

### 9.1.3 Morphology

이진 이미지(binary image)를 다룰 때 필수적인 도구이다. 예를 들어, segmentation 결과에서 작은 노이즈 점들을 제거하거나, 끊어진 영역을 이어 붙이거나 할 때 morphology를 쓴다. 이걸 모르면 이진화 결과를 후처리할 때 막막하다.

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

Opening과 Closing의 순서를 헷갈리는 사람이 많은데 — Opening은 "먼저 깎고(erosion) 다시 키우는(dilation)" 것이라 작은 돌기나 노이즈가 사라지고, Closing은 "먼저 키우고 다시 깎는" 것이라 작은 구멍이 메워진다. 직관적으로 기억하자.

> 추천 자료
> - [OpenCV Morphological Operations](https://docs.opencv.org/4.x/d9/d61/tutorial_py_morphological_ops.html) — 시각적 예제와 함께 설명
> - [First Principles of Computer Vision — Binary Image Processing](https://www.youtube.com/watch?v=IcBzsP-fvPo) — 형태학적 연산의 원리

---

## 9.2 카메라 모델 (Camera Model)

카메라가 세상을 어떻게 읽는지 모르면, 2D 이미지에서 3D를 복원하는 건 불가능하다. SLAM과 3D reconstruction의 출발점이 카메라 모델이다. 선형대수를 배웠다면 여기서 행렬이 어떻게 쓰이는지 체감할 수 있다.

### 9.2.1 Pinhole Model

이상적인 카메라 모델로, 3D 점을 2D 이미지로 투영한다.

픽셀 좌표 (u, v)에서 실제 세상의 3D 위치를 역으로 계산하려면 이 투영 관계를 정확히 알아야 한다. 이 관계를 수식으로 표현한 것이 Pinhole Model이다.

투영 방정식:

```
[u]   [f_x  0   c_x] [X/Z]
[v] = [0   f_y  c_y] [Y/Z]
[1]   [0    0    1 ] [ 1 ]
```

Intrinsic Parameters (내부 파라미터):
- f_x, f_y: Focal length (픽셀 단위)
- c_x, c_y: Principal point (이미지 중심)
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
> 3D 공간의 점이 카메라 내부/외부 파라미터를 통해 2D 이미지로 투영되는 과정을 인터랙티브하게 확인할 수 있다.

### 9.2.2 Distortion Models

실제 렌즈에서는 왜곡이 발생한다.

실제 카메라로 찍은 이미지는 Pinhole Model이 가정하는 것처럼 깔끔하지 않다. 특히 광각 렌즈나 fisheye 렌즈를 쓰면 직선이 곡선으로 보이는 왜곡이 심하다. 왜곡 보정을 빠뜨리면 SLAM 정확도가 뚝 떨어지고 3D reconstruction 결과가 찌그러진다.

카메라 렌즈는 완벽한 핀홀이 아니다. 렌즈를 통과하면서 빛이 휘어지고, 이 휘어짐이 이미지에 왜곡으로 나타난다.

Radial distortion (방사 왜곡): 이미지 중심에서 멀어질수록 심해진다. 파라미터 k1, k2, k3로 모델링한다. k1 < 0이면 barrel distortion (직선이 바깥으로 볼록), k1 > 0이면 pincushion distortion (직선이 안쪽으로 오목). 대부분의 렌즈는 barrel distortion을 가진다. 광각 렌즈일수록 심하다.

Tangential distortion (접선 왜곡): 렌즈가 이미지 센서와 완벽하게 평행하지 않을 때 발생한다. 파라미터 p1, p2. 보통 radial보다 영향이 작지만, 저가 카메라에서는 무시할 수 없다.

왜곡 보정:

```python
# 단순 보정 (매 프레임 계산 — 느림)
undistorted = cv2.undistort(distorted, K, dist_coeffs)

# 보정 맵 미리 계산 후 재사용 (빠름 — SLAM 파이프라인 표준)
map1, map2 = cv2.initUndistortRectifyMap(K, dist_coeffs, None, K, (w, h), cv2.CV_32FC1)
undistorted = cv2.remap(distorted, map1, map2, cv2.INTER_LINEAR)
```

`cv2.undistort()`는 매 프레임마다 호출하면 느리다. `initUndistortRectifyMap()`으로 맵을 계산해두고 `cv2.remap()`으로 적용하는 것이 실시간 시스템의 표준이다.

**어안 렌즈 (Fisheye)**: 일반 핀홀 왜곡 모델로는 보정이 안 된다. 어안 렌즈는 빛의 입사각 θ에 대해 왜곡을 모델링한다 (equidistant model: r = f·θ). OpenCV의 `cv2.fisheye` 모듈을 별도로 사용해야 한다. 혼동하면 보정 결과가 오히려 나빠지니 주의.

(참고: [다크 프로그래머 — 카메라 왜곡보정](https://darkpgmr.tistory.com/31), [정진용 블로그 — Camera Models and Distortion](https://jinyongjeong.github.io/2020/06/15/Camera_and_distortion_model/))

> 추천 자료
> - [OpenCV Camera Calibration and 3D Reconstruction](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html) — 왜곡 모델의 수식이 잘 정리되어 있다
> - [First Principles of CV — Lens Related Issues](https://www.youtube.com/watch?v=hzOeqCb2Fg4) — 렌즈 왜곡이 왜 생기는지 물리적 직관 설명

> 실습: [Lens Distortion 시각화](https://alexjunholee.github.io/robotics-practice/app.html#lens_distortion)
> Radial/Tangential 왜곡 파라미터를 조절하며 이미지가 어떻게 변형되는지 직접 확인할 수 있다.

### 9.2.3 캘리브레이션 (Calibration)

카메라의 내부/외부 파라미터를 추정하는 과정이다.

K (intrinsic matrix)와 왜곡 계수를 실제로 알아내야 카메라 모델을 쓸 수 있다. 캘리브레이션이 부정확하면 그 위에 쌓는 모든 것 — SLAM, 스테레오 깊이 추정, hand-eye calibration — 전부 정확도가 떨어진다. "garbage in, garbage out"의 대표적인 사례다.

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

팁: 캘리브레이션 퀄리티를 높이려면 (1) 다양한 각도에서 20장 이상 촬영하고, (2) 체커보드가 이미지 전체를 고르게 커버하게 하고, (3) reprojection error가 0.5 픽셀 이하인지 확인하자.

캘리브레이션이 하는 일을 직관적으로 이해하기

카메라 캘리브레이션은 결국 "이 카메라가 3D 세상을 2D 이미지로 어떻게 변환하는지"의 파라미터를 알아내는 것이다. 체커보드 패턴을 여러 각도에서 촬영하면, 체커보드의 3D 좌표(알고 있음)와 이미지의 2D 좌표(검출함)의 대응 쌍이 수십~수백 개 생긴다. 이 대응 쌍으로부터:

1. Intrinsic parameters (fx, fy, cx, cy): 렌즈의 초점거리와 이미지 중심. 카메라 고유 속성이므로 한 번 구하면 렌즈를 바꾸지 않는 한 변하지 않는다.
2. Distortion coefficients (k1, k2, p1, p2, k3): 렌즈의 왜곡 정도. 저가 렌즈일수록 크다.
3. Extrinsic parameters (R, t): 각 촬영 위치에서의 카메라 자세. 캘리브레이션 자체에서는 부산물이지만, hand-eye calibration 등에서 별도로 쓰인다.

체커보드를 최소 10장 이상, 다양한 각도와 거리에서 촬영해야 한다. 한쪽으로 치우치면 해당 영역의 왜곡만 보정되고 나머지는 부정확하다. 이미지 전체에 걸쳐 체커보드가 고르게 분포하도록 촬영하는 것이 핵심이다.

reprojection error가 0.5 픽셀 이하면 양호, 0.1 이하면 매우 좋다. 1.0 이상이면 촬영을 다시 하거나 outlier 이미지를 제거해야 한다.

(참고: [다크 프로그래머 — 카메라 캘리브레이션](https://darkpgmr.tistory.com/32))

Kalibr: 멀티 카메라, Camera-IMU 캘리브레이션 도구
- ROS 기반
- AprilTag 보드 사용
- 시간 오프셋까지 추정

> 추천 자료
> - [OpenCV 카메라 캘리브레이션 튜토리얼](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html) — 체커보드 캘리브레이션 step-by-step
> - [Kalibr 공식 Wiki](https://github.com/ethz-asl/kalibr/wiki) — Camera-IMU 캘리브레이션의 사실상 표준 도구
> - [Zhang, "A Flexible New Technique for Camera Calibration" (2000)](https://www.microsoft.com/en-us/research/publication/a-flexible-new-technique-for-camera-calibration/) — 현재 OpenCV 캘리브레이션의 기반이 되는 논문
> - [Tangram Vision Blog](https://www.tangramvision.com/blog) — 카메라 캘리브레이션, 센서 퓨전 등 실전 엔지니어링 글 모음

---

## 9.3 특징점 (Features)

이미지에서 구별 가능한 점(keypoint)과 그 주변을 설명하는 벡터(descriptor)이다.

SLAM을 이해하려면 특징점을 먼저 알아야 한다. 로봇이 카메라를 움직이면서 "지금 보는 장면이 아까 봤던 그곳인지"를 판단하려면 이미지 간에 같은 점을 찾아야 한다. 특징점은 그 대응점을 안정적으로 찾기 위한 핵심 도구다. SLAM과 Visual Odometry는 물론 거의 모든 시각 기반 로보틱스 알고리즘이 특징점에 의존한다.

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
- 특허 무료
- 실시간 SLAM에서 널리 사용

ORB는 ORB-SLAM 시리즈의 핵심이다. 특허 무료라서 상업적으로도 자유롭게 쓸 수 있고, 속도가 빨라 실시간 시스템에 적합하다. 로보틱스에서 가장 먼저 접하게 될 feature이다.

SIFT (Scale-Invariant Feature Transform):
- 스케일, 회전 불변
- 높은 반복성
- 계산 비용 높음 (과거 특허 문제, 현재 해제)

SIFT는 2004년 Lowe가 발표한 알고리즘으로, CV 분야에서 가장 많이 인용된 논문 중 하나다. 스케일과 회전에 불변하는 특징점 추출 원리를 이해하면, 이후 나온 SURF, ORB가 SIFT를 어떻게 개선했는지 자연스럽게 보인다.

SuperPoint (딥러닝 기반):
- Self-supervised 학습
- 높은 반복성과 정확도
- GPU 필요

> 추천 자료
> - [Lowe, "Distinctive Image Features from Scale-Invariant Keypoints" (2004)](https://www.cs.ubc.ca/~lowe/papers/ijcv04.pdf) — SIFT 원논문. 한 번쯤은 읽어볼 가치가 있다
> - [Rublee et al., "ORB: An efficient alternative to SIFT or SURF" (2011)](https://ieeexplore.ieee.org/document/6126544) — ORB 원논문
> - [First Principles of CV — Feature Detection](https://www.youtube.com/playlist?list=PL2zRqk16wsdqXEMpHrc4Qnb5rA1Cylrhx) — 특징점 검출의 원리를 시각적으로
> - [DeTone et al., "SuperPoint: Self-Supervised Interest Point Detection and Description" (2018)](https://arxiv.org/abs/1712.07629) — 딥러닝 기반 특징점의 시작
> - [다크 프로그래머 — 영상 특징점(keypoint) 추출방법](https://darkpgmr.tistory.com/131) — SIFT, HOG, Haar, Ferns, LBP, MCT 등 특징점 비교 정리

### 9.3.2 Descriptor

Keypoint를 찾았으면, 그 주변을 "어떻게 설명할 것인가"가 descriptor이다. 두 이미지에서 같은 물리적 점을 찾으려면, 그 점 주변의 패턴을 숫자로 표현해서 비교해야 한다.

BRIEF (Binary Robust Independent Elementary Features):
- 이진 디스크립터 (0 or 1)
- 빠른 매칭 (Hamming distance)
- 회전 불변 아님

ORB Descriptor:
- BRIEF + 방향 정보
- 256비트 이진 벡터

이진 디스크립터의 장점은 매칭 속도이다. 두 디스크립터 간 거리를 Hamming distance (XOR 연산)로 계산하기 때문에 SIFT의 유클리드 거리 비교보다 훨씬 빠르다. 임베디드 시스템에서 이 차이는 크다.

SuperGlue (딥러닝 기반):
- Graph Neural Network 기반 매칭
- 반복 패턴, 적은 텍스처에서도 강건
- LightGlue: 경량화 버전

> 추천 자료
> - [Sarlin et al., "SuperGlue: Learning Feature Matching with Graph Neural Networks" (2020)](https://arxiv.org/abs/1911.11763) — 딥러닝 기반 매칭의 대표작
> - [OpenCV Feature Matching 튜토리얼](https://docs.opencv.org/4.x/dc/dc3/tutorial_py_matcher.html) — BFMatcher, FLANN 사용법

### 9.3.3 Feature Matching

SLAM에서 카메라가 움직일 때 이전 프레임과 현재 프레임에서 같은 점을 찾아야 한다. feature matching이 바로 이 과정이고, 제대로 이해하지 못하면 왜 SLAM이 tracking lost를 뱉는지 감이 안 온다.

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
good = [m for m, n in matches if m.distance < 0.75 * n.distance]
```

Lowe's ratio test가 핵심이다. kNN으로 가장 가까운 2개의 매치를 찾고, 1등과 2등의 거리 비가 일정 threshold 이하인 것만 "좋은 매치"로 남긴다. 이렇게 하면 애매한 매치(1등과 2등 거리가 비슷한)를 걸러낼 수 있다. 0.75라는 값은 Lowe가 원논문에서 제안한 것인데, 상황에 따라 0.6~0.8 사이에서 조절하면 된다.

> 추천 자료
> - [OpenCV Feature Matching](https://docs.opencv.org/4.x/dc/dc3/tutorial_py_matcher.html) — BFMatcher, FLANN, Ratio Test 예제
> - [Computerphile — SIFT Features](https://www.youtube.com/watch?v=ram-jbLJjFg) — 특징점 매칭의 직관적 설명

> 실습: [Feature Matching](https://alexjunholee.github.io/robotics-practice/app.html#feature_matching)
> 두 이미지 간 특징점 매칭과 Lowe's ratio test 적용 과정을 인터랙티브하게 실험할 수 있다.

---

## 9.4 에피폴라 기하학 (Epipolar Geometry)

두 카메라 시점 사이의 기하학적 관계를 다룬다.

두 장의 사진에서 같은 물체를 봤을 때, 카메라가 어떻게 움직였는지(상대 자세)를 알아내고 나아가 3D 구조를 복원하는 것이 목표다. Visual Odometry와 SfM(Structure from Motion)의 수학적 기반이 에피폴라 기하학이다. 선형대수에서 배운 SVD, eigenvalue 분해 등이 직접 쓰이는 부분이기도 하다.

### 9.4.1 Essential Matrix (E)

정의: 캘리브레이션된 카메라 쌍의 상대 자세를 인코딩

```
x2^T E x1 = 0
```

- x1, x2: 정규화된 이미지 좌표
- E = [t]_× R (t의 skew-symmetric 행렬 × R)

5-point 알고리즘: 최소 5쌍의 대응점으로 E 추정 (RANSAC과 함께 사용)

Essential Matrix에서 R과 t를 분해(decompose)하면 두 카메라 간의 상대 회전과 이동을 알 수 있다. Visual Odometry의 핵심 원리다.

> 실습: [Epipolar Geometry 시각화](https://alexjunholee.github.io/robotics-practice/app.html#epipolar)
> 두 카메라 시점 간의 에피폴라 선과 에피폴을 인터랙티브하게 확인하며, Essential/Fundamental Matrix의 기하학적 의미를 이해할 수 있다.

### 9.4.2 Fundamental Matrix (F)

정의: 캘리브레이션되지 않은 카메라 쌍의 관계

```
p2^T F p1 = 0
```

- p1, p2: 픽셀 좌표
- F = K2^(-T) E K1^(-1)

8-point 알고리즘: 최소 8쌍의 대응점으로 F 추정

E와 F의 관계를 정리하면: F는 "픽셀 좌표에서 바로 쓸 수 있는" 버전이고, E는 "카메라 내부 파라미터를 이미 알고 있을 때 쓰는" 버전이다. 캘리브레이션을 했다면 E를, 안 했다면 F를 쓴다.

### 9.4.3 Triangulation

두 시점에서 동일 점을 관측했을 때, 3D 위치를 계산한다.

두 눈으로 깊이를 느끼는 것과 같은 원리다. 두 카메라(또는 움직인 하나의 카메라)에서 같은 점을 관측하면 기하학적으로 그 점의 3D 위치를 계산할 수 있다.

```python
# OpenCV triangulation
points_4d = cv2.triangulatePoints(P1, P2, pts1, pts2)
points_3d = points_4d[:3] / points_4d[3]  # Homogeneous → Cartesian
```

주의: baseline (두 카메라 간 거리)이 너무 작으면 삼각측량 정확도가 떨어지고, 너무 크면 같은 점을 양쪽에서 동시에 관측하기 어려워진다. 이 trade-off를 잘 이해해야 한다.

> 추천 자료
> - [Stanford CS231A — Epipolar Geometry](https://web.stanford.edu/class/cs231a/) — 수학적 유도가 잘 정리된 강의 자료
> - [Hartley & Zisserman, "Multiple View Geometry in Computer Vision"](https://www.robots.ox.ac.uk/~vgg/hzbook/) — 다중 시점 기하학의 핵심 교재. 깊이 들어가려면 필독
> - [First Principles of CV — Stereo Vision](https://www.youtube.com/playlist?list=PL2zRqk16wsdoYzrWStQ2SQHXXS2K6ofd4) — Epipolar geometry를 직관적으로 설명
> - [다크 프로그래머 — 영상 Geometry 시리즈 (7편: 좌표계~Epipolar)](https://darkpgmr.tistory.com/77) — 좌표계, Homogeneous, 2D/3D 변환, Homography, Imaging, Epipolar Geometry를 한글로 체계적 정리

> 실습: [Homography 시각화](https://alexjunholee.github.io/robotics-practice/app.html#homography)
> 평면 간 호모그래피 변환을 인터랙티브하게 조작하며, 4개의 대응점으로 투영 변환이 어떻게 결정되는지 확인할 수 있다.

---

## 9.5 광학 흐름 (Optical Flow)

연속 프레임 간 픽셀 이동을 추정한다.

로봇이 카메라로 세상을 보면서 움직일 때, 각 픽셀이 다음 프레임에서 어디로 갔는지 아는 것은 유용하다. Visual Odometry 자세 추정과 동적 물체 감지에 직접 쓰인다. Feature matching이 sparse한 점만 다루는 반면, dense optical flow는 모든 픽셀의 움직임을 추정한다.

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

"PyrLK"에서 "Pyr"는 Pyramid를 뜻한다. 이미지 피라미드를 사용해서 큰 움직임도 잡을 수 있게 한 것이다. Lucas-Kanade의 "작은 움직임 가정"을 극복하기 위한 기법이다.

### 9.5.2 Dense Optical Flow

- 모든 픽셀의 움직임 계산
- Farneback, RAFT (딥러닝)

```python
# Farneback dense flow
flow = cv2.calcOpticalFlowFarneback(prev_gray, curr_gray, None, 0.5, 3, 15, 3, 5, 1.2, 0)
```

최근에는 RAFT(Recurrent All-Pairs Field Transforms)가 dense optical flow의 기준점이 됐다. 딥러닝 기반이지만 정확도가 크게 높아서 품질이 중요한 경우에 쓰인다.

> 추천 자료
> - [First Principles of CV — Optical Flow](https://www.youtube.com/playlist?list=PL2zRqk16wsdp8KbDfHKvPYNGF2L-zQASc) — 광류의 수학적 원리
> - [Teed & Deng, "RAFT: Recurrent All-Pairs Field Transforms for Optical Flow" (2020)](https://arxiv.org/abs/2003.12039) — 딥러닝 기반 optical flow의 대표작
> - [Huang et al., "FlowFormer: A Transformer Architecture for Optical Flow" (ECCV 2022, arXiv:2203.16194)](https://arxiv.org/abs/2203.16194) — Transformer 기반 optical flow
> - [OpenCV Optical Flow 튜토리얼](https://docs.opencv.org/4.x/d4/dee/tutorial_optical_flow.html) — Lucas-Kanade, Farneback 코드 예제

> 실습: [Optical Flow 시각화](https://alexjunholee.github.io/robotics-practice/app.html#optical_flow)
> Lucas-Kanade와 Dense Optical Flow 알고리즘의 동작을 인터랙티브하게 비교하며 픽셀 이동 추정 과정을 확인할 수 있다.

---

## 9.6 심화: PnP 문제

*연구자가 되고 싶다면 여기서부터 읽어라.*

**Perspective-n-Point (PnP)**은 3D 공간의 점과 2D 이미지의 대응점이 주어졌을 때 카메라 포즈(회전 R + 이동 t)를 추정하는 문제다. SLAM에서 매 프레임 카메라 tracking이 곧 PnP 문제이며, AR에서 마커 기반 위치 추정도 PnP로 푼다.

문제 정의: n개의 3D-2D 대응 {(X_i, x_i)}가 주어졌을 때, 카메라 외부 파라미터 [R|t]를 추정한다.

$$x_i = K [R | t] X_i$$

여기서 K는 카메라 내부 파라미터(intrinsics)이다.

P3P (3-Point Problem):
- 최소 3개의 대응점으로 풀 수 있다.
- 3점으로 최대 4개의 해가 나온다. 4번째 점을 사용해 disambiguation한다.
- RANSAC과 결합하여 outlier에 robust하게 풀 수 있다.

EPnP (Efficient PnP):
- O(n) 복잡도로, 대응점이 많을 때 효율적이다.
- 3D 점들을 4개의 가상 제어점(virtual control points)으로 표현하고, 이 제어점의 카메라 좌표를 추정하는 방식이다.
- 많은 점이 있는 경우 P3P+RANSAC보다 빠르고 안정적이다.

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

# 기본 PnP (iterative, 초기값 필요 없음)
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

SLAM과의 연결: Visual SLAM에서 매 프레임마다 수행하는 과정은 다음과 같다.
1. 이전 프레임에서 삼각측량(triangulation)으로 3D 맵 포인트를 만든다.
2. 새 프레임에서 해당 맵 포인트의 2D 재투영(reprojection)을 예측한다.
3. 실제 관측된 2D 키포인트와 매칭한다.
4. 이 3D-2D 대응으로 PnP를 풀어 새 프레임의 카메라 포즈를 구한다.

ORB-SLAM3의 Tracking 단계가 정확히 이 네 단계다.

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
- `solvePnP`의 결과는 **세계→카메라 변환**이다. 카메라의 세계 좌표 위치를 구하려면 역변환을 해야 한다.
- 최소 4점이 필요하지만, 점이 많을수록 노이즈에 강건하다. RANSAC 버전인 `cv2.solvePnPRansac()`을 쓰면 outlier를 자동으로 걸러준다.
- `flags` 파라미터로 알고리즘을 선택할 수 있다: `cv2.SOLVEPNP_ITERATIVE` (기본, LM), `cv2.SOLVEPNP_P3P` (최소 3점), `cv2.SOLVEPNP_EPNP` (빠르고 안정적, 많은 점에 적합).

Rodrigues 벡터의 방향이 회전축, 크기(norm)가 회전 각도다. 3장의 Lie algebra so(3)에서 다룬 축-각 표현과 정확히 같다. `cv2.Rodrigues()`는 exp/log map의 구현이다.

(참고: [다크 프로그래머 — solvePnP 함수 사용법과 Rodrigues 표현법](https://darkpgmr.tistory.com/99))

---

## 9.7 심화: RANSAC 변종

*연구자가 되고 싶다면 여기서부터 읽어라.*

3장의 robust estimation에서 RANSAC을 소개했다. 실제 연구에서는 vanilla RANSAC을 그대로 쓰는 경우가 드물다. 수렴 속도와 정확도를 개선한 여러 변종이 존재하며, 어떤 것을 쓰느냐에 따라 결과가 크게 달라질 수 있다.

주요 변종:

| 방법 | 핵심 아이디어 | 특징 |
|------|-------------|------|
| Lo-RANSAC | inlier로 로컬 최적화 수행 | vanilla보다 적은 iteration으로 수렴 |
| PROSAC | 매칭 신뢰도 순으로 샘플링 | 좋은 매칭부터 먼저 시도하여 수렴 가속 |
| MAGSAC++ | σ(inlier threshold)를 marginalization | threshold 설정 불필요, 자동 적응 |

Lo-RANSAC (Locally Optimized RANSAC):
- 좋은 모델을 찾으면, 그 모델의 inlier들로 다시 모델을 추정(local optimization)한다.
- 단순한 아이디어지만 효과가 크다. 특히 inlier ratio가 낮을 때 유용하다.

PROSAC (Progressive Sample Consensus):
- 매칭 스코어가 높은 대응점부터 우선적으로 샘플링한다.
- 좋은 매칭이 앞에 많으면 초기 iteration에서 바로 좋은 모델을 찾는다.

MAGSAC++ (Marginalizing Sample Consensus):
- 가장 골치 아픈 하이퍼파라미터인 inlier threshold σ를 marginalize한다.
- Threshold를 고정하지 않고 여러 σ 값에 대해 적분하므로, 수동 튜닝이 거의 필요 없다.
- 현재 OpenCV에서 권장하는 방법이다.

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

# Homography 추정에도 동일하게 적용 가능
H, mask = cv2.findHomography(
    src_pts, dst_pts,
    method=cv2.USAC_MAGSAC,
    ransacReprojThreshold=3.0
)
```

실무 팁:
- Iteration 수: `confidence` 파라미터로 제어한다. 0.999면 "99.9% 확률로 올바른 모델을 찾겠다"는 의미. inlier ratio가 낮을수록 필요한 iteration이 기하급수적으로 증가한다.
- Threshold: MAGSAC++를 쓰면 threshold에 덜 민감하지만, 초기값은 여전히 줘야 한다. Fundamental matrix는 1.0~3.0 pixel, Homography는 3.0~5.0 pixel이 일반적이다.
- 속도가 중요하면 PROSAC, 정확도가 중요하면 MAGSAC++를 쓴다.

> 추천 자료
> - [Barath et al., "MAGSAC++, a Fast, Reliable and Accurate Robust Estimator" (2020)](https://arxiv.org/abs/1912.05909) — MAGSAC++ 원논문
> - [OpenCV USAC 문서](https://docs.opencv.org/4.x/d1/df1/md__build_4rdparty_ippicv_ippicv_lnx_doc_USAC.html) — OpenCV의 universal RANSAC 프레임워크
> - [Chum & Matas, "Matching with PROSAC" (2005)](https://doi.org/10.1109/CVPR.2005.221) — PROSAC 원논문

---

## 9.8 심화: 학습 기반 특징 매칭

*연구자가 되고 싶다면 여기서부터 읽어라.*

ORB, SIFT 같은 hand-crafted feature는 수십 년간 잘 작동해왔지만 반복 패턴, 텍스처 부족, 극단적 조명 변화 등에서 실패한다. 2018년 이후 딥러닝 기반 특징 추출과 매칭이 고전 방법을 넘어서기 시작했다.

파이프라인 발전 과정:

```
SuperPoint (2018) → SuperGlue (2020) → LightGlue (2023)
  [키포인트 검출+기술]    [그래프 신경망 매칭]     [경량화된 매칭]
```

SuperPoint:
- 자기지도 학습으로 키포인트 검출기와 디스크립터를 동시에 학습한다.
- Homographic adaptation: 합성 변환을 적용하고 역변환해 pseudo ground truth를 생성한다.
- 반복 패턴에 강하고 고전 방법 대비 repeatability가 높다.

SuperGlue:
- 두 이미지의 키포인트를 그래프로 보고, attention mechanism으로 매칭한다.
- Self-attention으로 같은 이미지 내 키포인트 관계를 학습하고, cross-attention으로 두 이미지 간 매칭을 수행한다.
- Sinkhorn algorithm으로 최적 할당(optimal assignment) 문제를 푼다.
- 정확도는 매우 높지만 속도가 느리다 (GPU 필수).

LightGlue:
- SuperGlue의 경량 버전. Adaptive early stopping으로 쉬운 이미지 쌍은 빨리, 어려운 쌍은 더 많은 layer를 통과시킨다.
- SuperGlue 대비 속도가 수 배 빠르면서 정확도는 유사하다.

LoFTR (Detector-Free Local Feature Matching):
- 키포인트 검출 단계 자체를 제거한다. 이미지 전체에서 dense matching을 수행한다.
- Transformer 기반으로 coarse-to-fine 매칭을 한다.
- 텍스처가 부족한 영역에서도 매칭이 가능하다는 것이 가장 큰 장점이다.
- 단점: 속도가 느리고 GPU 메모리를 많이 사용한다.

고전 방법 vs 학습 기반 비교:

| 항목 | ORB/SIFT | SuperPoint+LightGlue | LoFTR |
|------|----------|---------------------|-------|
| 속도 (CPU) | 빠름 | 느림 | 매우 느림 |
| 속도 (GPU) | 해당 없음 | 보통 | 느림 |
| 텍스처 부족 영역 | 실패 | 보통 | 강함 |
| 반복 패턴 | 약함 | 강함 | 강함 |
| GPU 의존성 | 없음 | 높음 | 매우 높음 |
| 실시간 로봇 적용 | 용이 | 조건부 가능 | 어려움 |

코드 예시 — LightGlue (kornia 사용):

```python
import kornia
from kornia.feature import LightGlueMatcher, KeyNetAffNetHardNet

# 특징 추출기 + 매칭기 구성
extractor = KeyNetAffNetHardNet(num_features=2048).eval()
matcher = LightGlueMatcher("keynetaffnethardnet").eval()

# GPU로 이동
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
extractor = extractor.to(device)
matcher = matcher.to(device)

# 이미지 로드 (kornia 형식: B x C x H x W, 0-1 범위)
img0 = kornia.io.load_image(path0).unsqueeze(0).to(device)
img1 = kornia.io.load_image(path1).unsqueeze(0).to(device)

# 특징 추출
with torch.no_grad():
    feats0 = extractor(img0)
    feats1 = extractor(img1)

# 매칭
dists, match_idxs = matcher(feats0["descriptors"], feats1["descriptors"])
```

GPU가 있으면 SuperPoint+LightGlue 조합이 가장 균형이 좋다. GPU 없이 embedded에서 돌려야 하면 여전히 ORB가 현실적이다.

> 추천 자료
> - [DeTone et al., "SuperPoint: Self-Supervised Interest Point Detection and Description" (2018)](https://arxiv.org/abs/1712.07629) — SuperPoint 원논문
> - [Lindenberger et al., "LightGlue: Local Feature Matching at Light Speed" (2023)](https://arxiv.org/abs/2306.13643) — LightGlue 원논문
> - [Sun et al., "LoFTR: Detector-Free Local Feature Matching with Transformers" (2021)](https://arxiv.org/abs/2104.00680) — LoFTR 원논문

---

> 기술 흐름: 컴퓨터 비전 기초 (Classical Methods)
> - **~2004**: 고전적 특징점의 시대. Harris Corner (1988), SIFT (2004) 등 hand-crafted feature가 주류. 수학적으로 정교하지만 계산이 무거움
> - **2006~2011**: 실시간을 위한 경량화. SURF (2006), FAST (2006), BRIEF (2010), ORB (2011) 등이 등장. 특허 문제와 속도 문제를 해결하면서 실시간 SLAM이 가능해짐
> - **2015~2019**: 딥러닝의 침투. SuperPoint (2018), SuperGlue (2020) 등 학습 기반 특징점이 고전 방법의 성능을 넘어서기 시작
> - **2020~**: Geometry + Learning의 융합. LoFTR (2021) 같은 detector-free matching, LightGlue (2023) 같은 경량 학습 매칭이 등장. 고전 기하학은 여전히 SLAM/VO의 백엔드에서 핵심
> - **지금 주목할 것**: 딥러닝이 front-end(특징점 추출, 매칭)를 대체하는 추세이지만, back-end의 수학은 그대로이다. 고전 기하학(epipolar geometry, triangulation)과 학습 기반 방법, 둘 다 필요하다
