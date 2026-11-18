# 9장 컴퓨터 비전 기초

카메라는 로봇이 세상을 바라보는 가장 풍부한 창이지만, 센서에서 막 빠져나온 이미지는 그저 거대한 숫자 격자일 뿐이다. 컴퓨터 비전은 이 원시 픽셀 배열에서 윤곽선, 특징점, 카메라의 움직임, 3차원 기하 구조처럼 로봇이 곧바로 쓸 수 있는 정보를 끌어낸다. 요즘은 딥러닝 모델이 인식을 많이 맡지만, SLAM이 갑자기 길을 잃거나 로봇 팔이 물체를 빗겨 잡을 때 원인이 전처리 노이즈인지, 렌즈 왜곡인지, 대응점 오매칭인지 추적하려면 이미지 처리와 카메라 기하의 바닥 원리를 짚고 넘어가야 한다.

---

## 9.1 이미지 처리 기초

카메라가 찍은 raw 이미지에는 센서 열 노이즈와 조명 변화, 불필요한 배경 질감이 고스란히 섞여 들어온다. 필터링으로 잡음을 덜어 내고 에지 검출과 형태학적 연산으로 윤곽과 영역을 정돈해 두면, 뒤쪽 파이프라인에서 문제가 생겼을 때 입력 전처리 탓인지 알고리즘 자체의 한계인지 가려내기가 훨씬 쉬워진다.

### 9.1.1 OpenCV 소개

OpenCV(Open Source Computer Vision Library)는 로보틱스와 비전 분야에서 사실상 표준처럼 쓰이는 오픈소스 라이브러리다. 내부 연산은 빠른 C++로 구현되어 있고 Python 바인딩도 탄탄해, 영상 입출력부터 필터링, 특징점 추출, 카메라 캘리브레이션, 기하 계산까지 실무에 필요한 기능을 두루 갖추고 있다.

먼저 `pip`로 기본 패키지와 추가 모듈(`contrib`)을 함께 설치한다.

```bash
pip install opencv-python opencv-contrib-python
```

이미지를 불러와 그레이스케일로 바꾼 뒤 화면에 띄우는 가장 기본적인 흐름은 다음과 같다.

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

여기서 한 가지 꼭 짚고 갈 습관이 있다. OpenCV는 컬러 채널을 RGB가 아니라 **BGR** 순서로 저장한다. 이 사실을 잊은 채 Matplotlib이나 딥러닝 프레임워크처럼 RGB를 기대하는 라이브러리로 이미지를 넘기면 파란색과 빨간색이 뒤바뀐 엉뚱한 화면을 보게 되므로, 넘기기 전에 `cv2.cvtColor(img, cv2.COLOR_BGR2RGB)`로 채널 순서를 맞춰 주어야 한다.

> 추천 자료
> - [OpenCV 공식 튜토리얼](https://docs.opencv.org/4.x/d9/df8/tutorial_root.html) — Python/C++ 예제가 잘 정리되어 있다
> - [First Principles of Computer Vision](https://www.youtube.com/channel/UCf0WB91t8Ky6AuYcQV0CcLw) — Columbia의 Shree Nayar 교수의 채널. 이미지 처리 원리를 직관적으로 설명한다
> - [Szeliski, "Computer Vision: Algorithms and Applications"](https://szeliski.org/Book/) — 저자 사이트에서 PDF를 무료로 받을 수 있다
> - [Stanford CS131 — Computer Vision: Foundations and Applications](http://vision.stanford.edu/teaching/cs131_fall1415/schedule.html) — CS231n보다 기초적인 CV 강의. 이미지 처리부터 시작하고 싶다면 여기서

### 9.1.2 필터링 (Filtering)

필터링은 이미지에서 필요한 신호만 남기고 거슬리는 노이즈를 눌러 주는 작업이다. 특히 에지 검출이나 영역 분할(segmentation)처럼 픽셀 값의 변화량을 따지는 연산은 작은 고주파 노이즈에도 민감하게 흔들리므로, 연산에 들어가기 전에 먼저 blur(흐림 처리)로 표면을 부드럽게 고르는 과정이 뒤따른다.

부드러운 가중 평균으로 전반적인 노이즈를 누를 때는 Gaussian blur를, 주변 픽셀의 중앙값을 취해 톡톡 튀는 점 노이즈(salt-and-pepper noise)를 경계를 덜 뭉개며 지울 때는 Median blur를 쓴다.

```python
# Gaussian Blur
blurred = cv2.GaussianBlur(img, (5, 5), 0)

# Median Blur (노이즈 제거에 효과적)
median = cv2.medianBlur(img, 5)
```

노이즈를 가라앉혔다면 이제 밝기가 급격히 변하는 경계, 즉 에지를 뽑아낼 차례다. Sobel 연산자는 가로와 세로 방향의 밝기 기울기(gradient)를 각각 계산하고, Canny 검출기는 이렇게 구한 기울기에 비최대 억제와 이중 임계값(hysteresis thresholding)을 적용해 깔끔한 1픽셀 두께의 윤곽선을 추출한다.

```python
# Canny Edge Detection
edges = cv2.Canny(gray, threshold1=50, threshold2=150)

# Sobel Operator
sobel_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
sobel_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
```

에지는 물체의 윤곽과 표면 경계를 또렷하게 드러내 주지만, Canny 검출 결과는 `threshold1`과 `threshold2` 설정에 크게 좌우된다. 임계값을 너무 높이면 옅은 경계가 끊어지고 너무 낮추면 배경 질감까지 에지로 잡히므로, 실제 카메라 영상에서 두 값을 바꿔 가며 놓치는 경계와 남는 노이즈 사이의 균형을 맞춰 보아야 한다.

> 추천 자료
> - [First Principles of Computer Vision — Edge Detection](https://www.youtube.com/playlist?list=PL2zRqk16wsdoCCLpouGuRbcJFBVVJlvgr) — 에지 검출의 수학적 원리를 시각적으로 설명
> - [OpenCV 필터링 튜토리얼](https://docs.opencv.org/4.x/d4/d13/tutorial_py_filtering.html) — 코드와 함께 바로 따라할 수 있다

> 실습: [Canny Edge Detection](https://alexjunholee.github.io/robotics-practice/app.html#canny_edge)
> Canny 에지 검출기의 threshold 파라미터를 실시간으로 조절하며 결과 변화를 확인할 수 있다.

> 실습: [Convolution 시각화](https://alexjunholee.github.io/robotics-practice/app.html#convolution)
> 여러 커널을 이미지에 적용하며 convolution(합성곱)으로 필터링이 되는 과정을 확인한다.

### 9.1.3 Morphology

이미지를 임계값 처리하거나 분할해 흑백의 이진 이미지(binary image)를 얻고 나면, 배경에 좁쌀만 한 흰 점들이 흩뿌려져 있거나 물체 안쪽에 작은 검은 구멍이 뚫려 있는 일이 흔하다. Morphology(형태학적 연산)는 이런 이진 마스크의 모양을 다듬어 잡음을 지우고 끊어진 영역을 이어 주는 후처리 도구다.

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

단일 연산만 쓰면 물체 전체의 크기가 줄거나(erosion) 커져 버리므로(dilation), 실무에서는 두 연산을 차례로 짝지어 쓴다. Opening은 먼저 침식으로 자잘한 돌기와 잡음 점을 깎아 낸 뒤 다시 팽창해 본래 물체 크기를 되돌리고, 반대로 Closing은 먼저 팽창해 영역 안쪽의 작은 구멍과 틈을 메운 뒤 다시 침식해 외곽선 크기를 원래대로 맞춘다.

> 추천 자료
> - [OpenCV Morphological Operations](https://docs.opencv.org/4.x/d9/d61/tutorial_py_morphological_ops.html) — 시각적 예제와 함께 설명
> - [First Principles of Computer Vision — Binary Image Processing](https://www.youtube.com/watch?v=IcBzsP-fvPo) — 형태학적 연산의 원리

---

## 9.2 카메라 모델

앞절의 필터링과 형태학적 연산이 2차원 이미지 평면 위에서 픽셀을 다듬는 작업이었다면, 이제부터는 이미지 속 2차원 좌표가 3차원 공간의 어느 위치에서 투영되었는지 따져 볼 차례다. SLAM이든 3차원 복원(3D reconstruction)이든 로봇이 카메라로 공간을 이해하려면 먼저 카메라가 3D 점을 픽셀로 옮기는 수학적 규칙, 즉 카메라 모델부터 세워야 한다.

### 9.2.1 Pinhole Model

Pinhole Model(핀홀 모델)은 렌즈 없이 아주 작은 구멍 하나로 3차원 공간의 점을 2차원 영상면에 투영하는 이상적인 카메라 모델이다. 3D 점 `(X, Y, Z)`에서 나온 빛이 핀홀을 지나 영상면에 맺힐 때, 원근 투영(perspective projection)에 따라 좌표는 깊이 `Z`로 나눈 정규화 좌표 `(X/Z, Y/Z)`가 된다. 여기에 초점 거리와 이미지 원점 오프셋을 곱해 더하면 실제 픽셀 좌표 `(u, v)`를 얻는다.

```
[u]   [f_x  0   c_x] [X/Z]
[v] = [0   f_y  c_y] [Y/Z]
[1]   [0    0    1 ] [ 1 ]
```

여기서 `(X, Y, Z)`는 카메라를 원점으로 삼은 카메라 좌표계의 3D 점이다. 투영 식에 등장하는 파라미터는 카메라 고유의 내부 특성과 공간상의 놓임새에 따라 두 묶음으로 나뉜다.

- **Intrinsic Parameters (내부 파라미터)**: 3×3 행렬 `K`로 묶이며, 렌즈와 센서 자체가 결정하는 성질이다. `f_x`, `f_y`는 픽셀 단위로 환산한 초점 거리(focal length)이고, `c_x`, `c_y`는 광축이 영상면과 만나는 주점(principal point)으로 센서 조립 오차 때문에 이미지의 정중앙과 다를 수 있다.
- **Extrinsic Parameters (외부 파라미터)**: 3×3 회전 행렬 `R`과 3×1 이동 벡터 `t`의 쌍 `[R|t]`로 나타내며, 월드 좌표계의 점을 카메라 좌표계로 옮기는 변환(World → Camera)을 담당한다.

요컨대 `[R|t]`가 "세상 어딘가에 놓인 카메라가 어느 방향을 바라보는가"를 정해 주고, `K`가 "그 카메라의 렌즈와 센서가 빛을 어떤 픽셀 격자에 모으는가"를 정해 준다. 이 둘을 차례로 적용하면 어떤 3D 점도 2D 픽셀 위치로 정확히 매핑할 수 있다.

> 추천 자료
> - [Stanford CS231A — Camera Models](https://web.stanford.edu/class/cs231a/) — 기하 기반 CV의 핵심 강의
> - [First Principles of CV — Camera and Imaging](https://www.youtube.com/playlist?list=PL2zRqk16wsdoYzrWStQ2SQHXXS2K6ofd4) — Pinhole부터 실제 렌즈까지 차근차근 설명
> - [Szeliski Ch.2 — Image Formation](https://szeliski.org/Book/) — 카메라 모델의 수학적 기초
> - [정진용 블로그 — Camera Models and Distortion (Perspective, Fisheye, Omni)](https://jinyongjeong.github.io/2020/06/15/Camera_and_distortion_model/) — Perspective, Equidistant, Omni 카메라 모델 비교 정리
> - [정진용 블로그 — OpenCV Camera model 정리](https://jinyongjeong.github.io/2020/06/19/SLAM-Opencv-Camera-model-%EC%A0%95%EB%A6%AC/) — OpenCV의 핀홀/어안 카메라 모델 구현 기준 정리

> 실습: [Camera Projection](https://alexjunholee.github.io/robotics-practice/app.html#camera_projection)
> 3D 공간의 점이 카메라 내부/외부 파라미터로 2D 이미지에 투영되는 과정을 인터랙티브하게 확인할 수 있다.

### 9.2.2 Distortion Models

실제 카메라에는 작은 구멍 대신 여러 겹의 유리 렌즈가 들어간다. 빛이 렌즈를 통과하면서 굴절하기 때문에, 특히 화각이 넓은 광각 렌즈나 어안 렌즈로 찍은 영상에서는 직선이 이미지 가장자리로 갈수록 곡선처럼 휘어 보인다. 이 렌즈 왜곡을 풀지 않은 채 핀홀 모델만 믿고 영상을 넘기면, SLAM의 궤적이 틀어지고 복원한 3차원 지도가 활처럼 휘어 버린다.

렌즈 왜곡은 크게 두 가지 항으로 나누어 모델링한다.

- **Radial distortion (방사 왜곡)**: 이미지 중심에서 바깥으로 멀어질수록 심해지는 왜곡으로, 계수 `k1`, `k2`, `k3`로 표현한다. `k1 < 0`이면 직선이 바깥쪽으로 불룩하게 휘는 barrel distortion(광각 렌즈에서 두드러짐)이 나타나고, `k1 > 0`이면 안쪽으로 오목하게 말리는 pincushion distortion이 나타난다.
- **Tangential distortion (접선 왜곡)**: 렌즈와 이미지 센서가 완벽하게 평행하지 않고 미세하게 기울어져 조립될 때 생긴다. 계수 `p1`, `p2`로 모델링하며 보통 방사 왜곡보다 영향이 작지만, 센서 정렬 공차가 큰 저가 모듈에서는 함께 추정해 보정한다.

OpenCV에서 왜곡을 보정하는 코드는 간단하지만, 실시간 로봇 파이프라인에서는 호출 방식에 따라 속도 차이가 크게 난다. 고정된 카메라 파라미터를 쓴다면 매 프레임 다항식 왜곡을 새로 계산하는 `cv2.undistort()` 대신, 초기화 단계에서 보정 맵을 한 번만 만들어 두고 `cv2.remap()`으로 픽셀을 재사용하는 편이 CPU 부담을 크게 줄여 준다.

```python
# 단순 보정 (매 프레임 계산 — 느림)
undistorted = cv2.undistort(distorted, K, dist_coeffs)

# 보정 맵 미리 계산 후 재사용 (SLAM 파이프라인의 일반적 패턴)
map1, map2 = cv2.initUndistortRectifyMap(K, dist_coeffs, None, K, (w, h), cv2.CV_32FC1)
undistorted = cv2.remap(distorted, map1, map2, cv2.INTER_LINEAR)
```

화각이 180도 가까이 넓어지는 **어안 렌즈(Fisheye)**는 일반적인 저차 radial-tangential 핀홀 모델로 근사하기 어렵다. 어안 모델은 영상면 위 거리를 직선 탄젠트가 아니라 빛의 입사각 θ로 표현하며, 대표적으로 equidistant 모델은 `r = f·θ` 관계를 따른다. OpenCV에는 이 모델용 `cv2.fisheye` 모듈이 따로 있으며, 어떤 모델을 쓸지는 렌즈의 실제 투영 특성과 별도 검증 영상에서 측정한 재투영 잔차를 비교해 결정한다.

(참고: [다크 프로그래머 — 카메라 왜곡보정](https://darkpgmr.tistory.com/31), [정진용 블로그 — Camera Models and Distortion](https://jinyongjeong.github.io/2020/06/15/Camera_and_distortion_model/))

> 추천 자료
> - [OpenCV Camera Calibration and 3D Reconstruction](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html) — 왜곡 모델의 수식이 잘 정리되어 있다
> - [First Principles of CV — Lens Related Issues](https://www.youtube.com/watch?v=hzOeqCb2Fg4) — 렌즈 왜곡이 왜 생기는지 물리적 직관 설명

> 실습: [Lens Distortion 시각화](https://alexjunholee.github.io/robotics-practice/app.html#lens_distortion)
> Radial/Tangential 왜곡 파라미터를 조절하며 이미지가 어떻게 변형되는지 직접 확인할 수 있다.

### 9.2.3 캘리브레이션 (Calibration)

카메라 모델과 왜곡 식을 세웠더라도, 눈앞에 놓인 실제 카메라의 `K`와 왜곡 계수 숫자를 모르면 3차원 계산에 쓸 수 없다. 캘리브레이션은 이 미지의 내부·외부 파라미터를 촬영 데이터로부터 추정하는 절차다. 이 값이 어긋나면 이를 물려받는 SLAM, 스테레오 깊이 추정, 그리고 로봇 손과 카메라 사이의 변환을 구하는 hand-eye calibration까지 연쇄적으로 정확도가 무너진다.

가장 널리 쓰이는 체커보드 캘리브레이션은 격자 크기를 정확히 아는 체커보드를 여러 각도에서 찍어 3D 코너 좌표와 이미지 속 2D 코너 좌표의 대응 쌍을 수십~수백 개 모은 뒤, 재투영 오차가 최소가 되도록 파라미터를 최적화하는 순서로 진행한다.

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

이 과정을 거치면 다음 세 가지가 함께 구해진다.

1. **Intrinsic parameters (`f_x, f_y, c_x, c_y`)**: 초점 거리와 주점이다. 렌즈 초점이나 줌을 돌리거나 해상도·crop 설정을 바꿨을 때, 혹은 온도 변화와 충격으로 기구 조립이 틀어졌을 때는 다시 추정해야 한다.
2. **Distortion coefficients (`k1, k2, p1, p2, k3`)**: 선택한 투영 모델 아래에서 렌즈와 조립이 만드는 왜곡을 근사하는 계수다.
3. **Extrinsic parameters (`R, t`)**: 촬영한 영상 한 장 한 장마다 체커보드 좌표계 기준으로 카메라가 어디에 놓여 있었는지 나타낸다. 단일 카메라 캘리브레이션만 할 때는 중간 부산물이지만, hand-eye calibration 등에서는 이 값을 그대로 활용한다.

캘리브레이션 데이터의 품질은 단순히 사진 장수가 아니라 파라미터가 얼마나 잘 드러나는지(관측 가능성)로 따져야 한다. 비슷한 구도로 수십 장을 찍기보다, 타겟이 이미지 중앙과 네 귀퉁이 가장자리까지 골고루 덮고 거리와 여러 축의 기울기가 다양하게 변하도록 찍어야 왜곡 계수와 초점 거리가 서로 얽히지 않고 안정적으로 풀린다.

필요한 촬영 장수는 파라미터의 불확실성과 수치 조건(conditioning), 코너 검출 품질에 따라 달라지며, 재투영 오차(reprojection error)의 기대 범위 역시 해상도·렌즈 모델·타겟·코너 검출기에 좌우되므로 카메라마다 기준을 따로 세운다. 검증할 때는 전체 RMS 수치 하나만 보지 말고 이미지별·화면 위치별 잔차와 별도 검증 영상(held-out set)의 오차를 함께 살핀다. 잔차가 유독 큰 영상이 나와도 무조건 지워 버리기 전에 모션 블러, 빛 반사, 코너 오검출이 있었는지 먼저 확인하고, 제외한 전후의 파라미터 안정성과 검증 오차를 비교해 판단한다.

(참고: [다크 프로그래머 — 카메라 캘리브레이션](https://darkpgmr.tistory.com/32))

단일 카메라를 넘어 여러 대의 카메라나 Camera-IMU 센서 묶음을 함께 보정해야 할 때는 ROS 기반의 **Kalibr** 도구가 표준처럼 쓰인다. 체커보드와 원형 격자도 지원하지만 추적이 안정적인 AprilGrid(AprilTag 격자)를 권장하며, 센서 간 공간 변환뿐 아니라 카메라와 IMU 사이의 미세한 시간 오프셋까지 한꺼번에 추정해 준다.

> 추천 자료
> - [OpenCV 카메라 캘리브레이션 튜토리얼](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html) — 체커보드 캘리브레이션 step-by-step
> - [Kalibr 공식 Wiki](https://github.com/ethz-asl/kalibr/wiki) — 널리 쓰이는 공개 Camera-IMU 캘리브레이션 도구
> - [Zhang, "A Flexible New Technique for Camera Calibration" (2000)](https://www.microsoft.com/en-us/research/publication/a-flexible-new-technique-for-camera-calibration/) — 현재 OpenCV 캘리브레이션의 기반이 되는 논문
> - [Tangram Vision Blog](https://www.tangramvision.com/blog) — 카메라 캘리브레이션, 센서 융합 등 실전 엔지니어링 글 모음

---

## 9.3 특징점

한 대의 카메라가 3D 공간을 어떻게 투영하는지 파악했다면, 이제 움직이는 로봇의 시선에서 두 장의 이미지를 연결해 볼 차례다. SLAM이나 Visual Odometry(시각 주행거리 추정)가 카메라의 이동량을 계산하려면, 연속된 프레임 속에서 물리적으로 똑같은 3D 지점이 어느 픽셀로 옮겨 갔는지 알아내야 한다. 이때 화면 전체에서 변별력이 높은 지점을 뽑은 것이 keypoint(특징점)이고, 그 주변 밝기 패턴을 숫자로 요약해 서로 비교할 수 있게 만든 벡터가 descriptor(디스크립터)다.

### 9.3.1 Keypoint Detection

특징점 검출기는 이미지가 회전하거나 카메라가 앞뒤로 움직여 크기가 달라져도 같은 모서리나 무늬를 반복해서 찾아내야 한다.

- **Harris Corner**: 밝기가 모든 방향으로 급격히 변하는 모서리를 찾는 고전적 방법이지만, 연산이 비교적 무겁고 카메라 거리에 따른 스케일 변화에 취약하다.
- **FAST (Features from Accelerated Segment Test)**: 후보 픽셀 둘레의 원형 픽셀 몇 개만 검사해 코너 여부를 판별하므로 속도가 매우 빠르고 실시간 시스템에 잘 맞는다. 다만 스케일 불변성은 갖추지 못했다.
- **ORB (Oriented FAST and Rotated BRIEF)**: FAST 검출기에 방향 정보를 보태고 회전 보정을 거친 BRIEF 디스크립터를 결합한 알고리즘이다. 특허 제약이 없으면서도 실시간 성능이 뛰어나 ORB-SLAM 시리즈를 비롯한 실시간 SLAM 시스템에서 널리 쓰인다.
- **SIFT (Scale-Invariant Feature Transform)**: 이미지 피라미드와 방향 히스토그램을 활용해 스케일과 회전 변화에 모두 강하고 반복성도 높다. 연산 비용이 크다는 점이 단점이며, Lowe가 1999년 ICCV 논문에서 처음 발표한 뒤 2004년 IJCV 논문으로 정리했다. SURF와 ORB 논문도 SIFT를 기준 비교 대상으로 삼는다. 과거에는 특허(US6711293) 걸림돌이 있었으나 2020년 3월에 만료되면서 OpenCV 4.4.0부터 기본(main) 모듈에 포함되었다.
- **SuperPoint (딥러닝 기반)**: Self-supervised 학습으로 특징점과 디스크립터를 함께 뽑는 신경망 기반 방식이다. 반복성과 정확도가 높지만 실시간으로 돌리려면 GPU 자원이 필요하다.

> 추천 자료
> - [Lowe, "Distinctive Image Features from Scale-Invariant Keypoints" (2004)](https://www.cs.ubc.ca/~lowe/papers/ijcv04.pdf) — SIFT 원논문
> - [Rublee et al., "ORB: An efficient alternative to SIFT or SURF" (2011)](https://ieeexplore.ieee.org/document/6126544) — ORB 원논문
> - [First Principles of CV — Feature Detection](https://www.youtube.com/playlist?list=PL2zRqk16wsdqXEMpHrc4Qnb5rA1Cylrhx) — 특징점 검출의 원리를 시각적으로
> - [DeTone et al., "SuperPoint: Self-Supervised Interest Point Detection and Description" (2018)](https://arxiv.org/abs/1712.07629) — 자기지도 학습 기반 특징점 검출·기술 논문
> - [다크 프로그래머 — 영상 특징점(keypoint) 추출방법](https://darkpgmr.tistory.com/131) — SIFT, HOG, Haar, Ferns, LBP, MCT 등 특징점 비교 정리

### 9.3.2 Descriptor

Keypoint가 "이미지의 어디에 주목할 것인가"를 정한다면, Descriptor는 "그 점 주변이 어떻게 생겼는가"를 숫자 벡터로 적어 두는 단계다. 두 영상에서 같은 물리적 지점을 찾으려면 결국 이 벡터 사이의 거리를 비교해야 한다.

- **BRIEF (Binary Robust Independent Elementary Features)**: 주변 픽셀 쌍의 밝기 대소 비교 결과를 `0`과 `1`의 비트열로 묶은 이진 디스크립터다. Hamming distance로 매우 빠르게 매칭할 수 있지만, 패치 방향을 맞추지 않으므로 회전 불변성은 없다.
- **ORB Descriptor**: BRIEF에 특징점의 주방향 정보를 반영해 256비트 이진 벡터로 만든 디스크립터다.

이진 디스크립터가 임베디드 로보틱스에서 각광받는 결정적 이유는 매칭 속도다. 두 벡터 사이의 거리를 부동소수점 유클리드 거리 대신 비트 단위 XOR 연산인 Hamming distance로 계산하기 때문에, SIFT 같은 실수형 디스크립터보다 비교 속도가 월등히 빠르다. 연산 자원이 빠듯한 온보드 컴퓨터에서는 이 차이가 전체 파이프라인의 프레임 속도를 좌우한다.

- **SuperGlue (딥러닝 기반)**: Graph Neural Network로 두 이미지의 특징점 관계를 통째로 보고 매칭하므로 반복 패턴이나 텍스처가 적은 환경에서도 잘 버틴다. 경량화 버전인 LightGlue도 나와 있다(→ §9.8에서 상세히 다룬다).

> 추천 자료
> - [Sarlin et al., "SuperGlue: Learning Feature Matching with Graph Neural Networks" (2020)](https://arxiv.org/abs/1911.11763) — SuperGlue 원논문

### 9.3.3 Feature Matching

실제 영상에는 비슷하게 생긴 모서리가 수두룩하기 때문에, 단순히 가장 가까운 디스크립터끼리 짝을 지으면 엉뚱한 매칭이 대거 섞여 든다. SLAM에서 `tracking lost`가 났을 때 매칭 결과를 화면에 띄워 보면, 대응점 자체가 모자라서 끊겼는지 아니면 잘못된 대응점이 너무 많아 포즈가 튀었는지 바로 구분할 수 있다.

다음 예제는 ORB 특징점을 뽑아 전수 비교(Brute-Force) 매칭을 수행한 뒤, 모호한 매치를 걸러내는 Lowe's ratio test를 적용하는 흐름이다.

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

Lowe's ratio test의 아이디어는 명쾌하다. `knnMatch(..., k=2)`로 가장 가까운 후보 둘을 찾았을 때, 1등 후보와 2등 후보의 거리 차이가 크지 않다면(비율이 임계값보다 크다면) 반복 무늬처럼 헷갈리는 점일 가능성이 높으니 과감히 걸러 낸다. Lowe(2004) 원 논문에서는 비가 0.8보다 큰 매치를 탈락시켰고, 위 코드에서는 조금 더 엄격한 0.75를 썼다. 실제 프로젝트에서는 데이터 특성에 맞춰 0.6~0.8 사이에서 조정한다.

> 추천 자료
> - [OpenCV Feature Matching](https://docs.opencv.org/4.x/dc/dc3/tutorial_py_matcher.html) — BFMatcher, FLANN, Ratio Test 예제
> - [Computerphile — SIFT Features](https://www.youtube.com/watch?v=ram-jbLJjFg) — 특징점 매칭의 직관적 설명

> 실습: [Feature Matching](https://alexjunholee.github.io/robotics-practice/app.html#feature_matching)
> 두 이미지 간 특징점 매칭과 Lowe's ratio test 적용 과정을 인터랙티브하게 실험할 수 있다.

---

## 9.4 에피폴라 기하학

두 프레임 사이에서 믿을 만한 2D 대응점 쌍을 확보했다면, 이제 이 점들을 지렛대 삼아 카메라가 두 시점 사이를 어떻게 이동했는지, 그리고 각 점이 3차원 공간의 어디에 놓여 있는지 복원할 수 있다. 에피폴라 기하학(Epipolar Geometry)은 바로 두 카메라 시점 사이에 성립하는 기하학적 제약 조건을 다룬다. Visual Odometry와 SfM(Structure from Motion, 여러 영상에서 3D 구조와 카메라 위치를 함께 추정)의 뼈대가 여기 놓여 있으며, 실제 해를 구하는 과정에는 SVD와 고유값 분해가 핵심 도구로 들어간다.

### 9.4.1 Essential Matrix (E)

첫 번째 카메라 이미지의 한 점 `x1`에 대응하는 실제 3D 점은 카메라 중심에서 뻗어 나가는 광선(ray) 위 어딘가에 놓여 있다. 이 광선을 두 번째 카메라 이미지에 투영하면 한 직선(에피폴라 선)이 되는데, 같은 3D 점을 찍은 두 번째 이미지의 대응점 `x2`는 반드시 이 직선 위에 놓여야 한다. 캘리브레이션된 카메라 쌍에서 두 시점의 상대 회전 `R`과 이동 `t`를 3×3 행렬 하나로 압축해 이 공면(coplanar) 조건을 나타낸 것이 **Essential Matrix(E)**다.

```
x2^T E x1 = 0
```

- `x1`, `x2`: 내부 파라미터 `K`로 정규화한 이미지 좌표
- `E = [t]_× R` (`t`의 skew-symmetric 행렬과 `R`의 곱)
- `R`, `t`: 카메라 1 좌표계의 점을 카메라 2 좌표계로 옮기는 회전과 이동 (`X2 = R X1 + t`)

`E`를 추정하려면 최소 5쌍의 대응점을 쓰는 5-point 알고리즘을 사용하며, 현업에서는 오매칭(outlier)을 걸러 내기 위해 반드시 RANSAC과 짝지어 돌린다. 구한 `E`를 `R`과 `t`로 분해(decompose)하면 네 쌍의 후보가 나오고, 그중 삼각측량한 점들이 양쪽 카메라 앞(양의 깊이)에 놓이는 후보를 고르면 상대 회전과 이동 방향을 확정할 수 있다. 다만 단안 카메라에서는 이동 벡터 `t`의 방향만 나올 뿐 미터 단위의 실제 이동 거리는 알 수 없으므로, 단안 Visual Odometry에서 절대 크기를 맞추려면 별도의 척도(scale) 정보가 필요하다.

> 실습: [Epipolar Geometry 시각화](https://alexjunholee.github.io/robotics-practice/app.html#epipolar)
> 두 카메라 시점의 에피폴라 선과 에피폴을 조작하며 Essential/Fundamental Matrix가 무엇을 나타내는지 확인한다.

### 9.4.2 Fundamental Matrix (F)

만약 카메라의 내부 파라미터 `K`를 모른다면 정규화 좌표 `x1, x2` 대신 픽셀 좌표 `p1, p2`를 그대로 써야 한다. 이때 캘리브레이션되지 않은 카메라 쌍의 에피폴라 제약을 담는 행렬이 **Fundamental Matrix(F)**다.

```
p2^T F p1 = 0
```

- `p1`, `p2`: 픽셀 좌표
- `F = K2^(-T) E K1^(-1)`

`F`는 8쌍 이상의 대응점으로 선형 추정하는 8-point 알고리즘으로 푸는 것이 일반적이며, 행렬의 자유도가 7이기 때문에 최소 해법으로는 7쌍만 쓰는 7점 알고리즘(`cv2.FM_7POINT`)도 쓴다. 정리하자면 `F`는 픽셀 좌표에 곧바로 적용하고 `E`는 `K`로 정규화한 좌표에 적용하므로, 카메라 캘리브레이션을 마쳤다면 `E`를, 미지의 카메라 영상을 다룬다면 `F`를 선택한다.

### 9.4.3 Triangulation

상대 자세 `[R|t]`를 알아내 두 카메라가 공간에서 어떻게 마주 보고 있는지 확정했다면, 이제 대응하는 2D 픽셀 쌍에서 3차원 좌표를 역산할 수 있다. Triangulation(삼각측량)은 두 시점에서 뻗어 나온 두 광선이 3D 공간에서 만나는(혹은 가장 가깝게 스치는) 지점을 계산해 3D 점 위치를 복원한다.

```python
# OpenCV triangulation
points_4d = cv2.triangulatePoints(P1, P2, pts1, pts2)
points_3d = points_4d[:3] / points_4d[3]  # Homogeneous → Cartesian
```

`cv2.triangulatePoints`는 동차 좌표(homogeneous coordinates)를 반환하므로, 마지막 원소인 `points_4d[3]`으로 나누어 일반 3차원 직교 좌표(`points_3d`)로 변환한다. 이때 두 카메라 사이의 거리인 baseline 설정에는 뚜렷한 트레이드오프가 있다. baseline이 너무 좁으면 두 광선이 거의 평행하게 만나 픽셀 오차 몇 칸에도 깊이 오차가 크게 튀고, 반대로 너무 넓으면 두 시점의 시야각 차이가 커져 같은 특징점을 양쪽 영상에서 동시에 찾기가 어려워진다.

> 추천 자료
> - [Stanford CS231A — Epipolar Geometry](https://web.stanford.edu/class/cs231a/) — 수학적 유도가 잘 정리된 강의 자료
> - [Hartley & Zisserman, "Multiple View Geometry in Computer Vision"](https://www.robots.ox.ac.uk/~vgg/hzbook/) — 다중 시점 기하학 교재
> - [First Principles of CV — Stereo Vision](https://www.youtube.com/playlist?list=PL2zRqk16wsdoYzrWStQ2SQHXXS2K6ofd4) — Epipolar geometry를 직관적으로 설명
> - [다크 프로그래머 — 영상 Geometry 시리즈 (7편: 좌표계~Epipolar)](https://darkpgmr.tistory.com/77) — 좌표계, Homogeneous, 2D/3D 변환, Homography, Imaging, Epipolar Geometry를 한글로 체계적 정리

> 실습: [Homography 시각화](https://alexjunholee.github.io/robotics-practice/app.html#homography)
> 평면 간 호모그래피 변환을 인터랙티브하게 조작하며, 4개의 대응점으로 투영 변환이 어떻게 결정되는지 확인할 수 있다.

---

## 9.5 광학 흐름

앞절의 특징점 매칭은 시점이 제법 달라진 두 이미지 사이에서도 드문드문 떨어져 있는(sparse) 핵심 지점들을 짝지어 준다. 반면 비디오 스트림처럼 연속된 프레임이 짧은 간격으로 들어올 때는 매번 디스크립터를 뽑아 비교하지 않고도, 픽셀 밝기 패턴이 바로 직전 프레임에서 어디로 흘러갔는지 직접 추적할 수 있다. 이것이 optical flow(광학 흐름)이며, Visual Odometry의 프레임 간 추적과 동적 물체 검출에 자주 활용된다.

### 9.5.1 Lucas-Kanade Method

Lucas-Kanade 방법은 고른 특징점들만 골라 추적하는 **sparse optical flow**의 대표 주자다. 같은 물리적 지점은 짧은 시간 동안 밝기가 변하지 않는다는 **밝기 불변 가정**과, 프레임 사이 이동 폭이 작아 국소 테일러 근사가 통한다는 **작은 움직임 가정** 위에서 해를 구한다.

```python
# 광류 계산
p1, status, err = cv2.calcOpticalFlowPyrLK(
    prev_gray, curr_gray, p0, None, **lk_params
)
```

함수 이름 `calcOpticalFlowPyrLK`에서 `Pyr`은 이미지 피라미드(Pyramid)를 가리킨다. 카메라가 빠르게 회전해 실제 픽셀 이동량이 수십 픽셀에 달하더라도, 해상도를 단계적으로 낮춘 저해상도 피라미드 층에서는 움직임이 몇 픽셀 수준으로 줄어든다. 덕분에 거친 층에서 대략의 이동량을 먼저 잡은 뒤 고해상도 층으로 내려오며 정밀화하면, 작은 움직임 가정을 깨지 않고도 큰 변위를 안정적으로 추적할 수 있다.

### 9.5.2 Dense Optical Flow

특정 코너만 따라가는 대신 화면 안 **모든 픽셀**의 2차원 이동 벡터장을 구해야 할 때는 dense optical flow를 쓴다. 장면 전체에서 움직이는 물체를 배경과 분리하거나 픽셀 단위의 조밀한 움직임 패턴을 분석할 때 쓰이며, 고전적인 Farneback 알고리즘과 딥러닝 기반 모델인 RAFT가 대표적이다.

```python
# Farneback dense flow
flow = cv2.calcOpticalFlowFarneback(prev_gray, curr_gray, None, 0.5, 3, 15, 3, 5, 1.2, 0)
```

학습 기반 dense optical flow 연구에서는 **RAFT(Recurrent All-Pairs Field Transforms)**가 핵심 비교 기준으로 자주 등장한다. 2020년 원 논문은 모든 픽셀 쌍의 상관 볼륨을 반복적으로 갱신하는 구조로 KITTI와 Sintel 벤치마크에서 기존 방법보다 낮은 광류 추정 오류를 보고했다.

> 추천 자료
> - [First Principles of CV — Optical Flow](https://www.youtube.com/playlist?list=PL2zRqk16wsdp8KbDfHKvPYNGF2L-zQASc) — 광류의 수학적 원리
> - [Teed & Deng, "RAFT: Recurrent All-Pairs Field Transforms for Optical Flow" (2020)](https://arxiv.org/abs/2003.12039) — RAFT 원논문
> - [Huang et al., "FlowFormer: A Transformer Architecture for Optical Flow" (ECCV 2022, arXiv:2203.16194)](https://arxiv.org/abs/2203.16194) — Transformer 기반 optical flow
> - [OpenCV Optical Flow 튜토리얼](https://docs.opencv.org/4.x/d4/dee/tutorial_optical_flow.html) — Lucas-Kanade, Farneback 코드 예제

> 실습: [Optical Flow 시각화](https://alexjunholee.github.io/robotics-practice/app.html#optical_flow)
> Lucas-Kanade와 Dense Optical Flow 알고리즘의 동작을 인터랙티브하게 비교하며 픽셀 이동 추정 과정을 확인할 수 있다.

---

## 9.6 PnP 문제 [심화]

앞선 에피폴라 기하학이 두 장의 2D 이미지로부터 상대 자세와 3D 점들을 처음부터 세우는 방법이었다면, 이미 3D 지도가 만들어져 있는 다음 순간부터는 문제가 달라진다. 새 프레임이 들어올 때마다 2D-2D로만 다시 풀면 척도(scale)가 흔들리기 쉽지만, 앞선 삼각측량으로 얻은 3D 맵 포인트와 현재 프레임에서 찾은 2D 픽셀을 직접 연결하면 카메라의 절대 위치와 방향을 한 번에 구할 수 있다. **Perspective-n-Point(PnP)**가 바로 이 3D-2D 대응으로부터 카메라 포즈(회전 `R`과 이동 `t`)를 추정하는 문제이며, SLAM의 프레임별 카메라 추적과 AR의 마커 기반 위치 추정이 모두 이 연산을 핵심 엔진으로 삼는다.

수학적으로 정리하면, `n`개의 3D-2D 대응 쌍 `{(X_i, x_i)}`와 알려진 카메라 내부 파라미터 `K`가 주어졌을 때 다음 투영 식을 만족하는 외부 파라미터 `[R|t]`를 찾는 문제다.

$$x_i = K [R | t] X_i$$

여기서 `x_i`와 `X_i`는 동차 좌표이며, 투영 방정식 특성상 양변은 전체 스케일 상수 배의 차이를 제외하고 일치한다. 대응점의 개수와 잡음 수준에 따라 주로 다음 두 해법이 쓰인다.

- **P3P (3-Point Problem)**: 최소 3개의 대응점만으로 풀 수 있는 최소 해법(minimal solver)이다. 기하학적으로 3점만 쓰면 최대 4개의 후보 해가 나오므로 4번째 점을 대입해 그중 올바른 하나를 가려낸다. 최소 표본 크기가 작기 때문에 RANSAC 루프 안에서 오매칭(outlier)을 걸러 내는 용도로 궁합이 좋다.
- **EPnP (Efficient PnP)**: `n`개의 3D 점들을 4개의 가상 제어점(virtual control points)의 가중합으로 나타낸 뒤, 제어점의 카메라 좌표를 먼저 구해 전체 포즈를 `O(n)` 복잡도로 계산한다. 대응점이 수십~수백 개로 많을 때 특히 빠르다. 원 논문 실험에서는 다른 비반복 해법(AD, Clamped DLT)보다 정확했고, 반복 해법(LHM)보다 훨씬 빠르면서도 정확도 차이는 작았으며, 해를 구한 뒤 Gauss-Newton 정밀화를 한 번 더하면(`EPnP+GN`) 그 차이마저 거의 사라진다.

OpenCV에서는 `cv2.solvePnP()`와 outlier 제거가 결합된 `cv2.solvePnPRansac()`으로 PnP를 곧바로 풀 수 있다.

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

실제 Visual SLAM 파이프라인은 프레임이 들어올 때마다 다음 순환을 반복하며 카메라 위치를 추적한다.

1. 이전 키프레임들에서 삼각측량(triangulation)으로 3D 맵 포인트를 만든다.
2. 새 프레임이 들어오면 직전 움직임을 바탕으로 해당 맵 포인트들이 이미지 어디에 맺힐지 2D 재투영(reprojection) 위치를 예측한다.
3. 예측 위치 주변에서 실제 관측된 2D 키포인트와 매칭한다.
4. 확보한 3D-2D 대응으로 PnP를 풀어 새 프레임의 카메라 포즈를 구한다.

ORB-SLAM3의 Tracking 모듈도 대체로 이 흐름을 따른다. 다만 단안 모드에서는 키프레임 사이의 삼각측량으로 새 맵 포인트를 만드는 작업을 주로 Local Mapping 스레드가 맡아 처리하고, 깊이를 바로 알 수 있는 스테레오·RGB-D 모드에서는 Tracking이 새 키프레임을 넣는 시점에 근거리 점을 곧바로 생성한다는 차이가 있다.

**solvePnP 실전 팁**

`cv2.solvePnP()`가 반환하는 `rvec`은 3×3 회전 행렬이 아니라 방향이 회전축, 크기(norm)가 회전 각도를 뜻하는 **Rodrigues 벡터**(축-각 표현)다. 3장 §3.5.2–3.5.3의 `so(3)`에서 다룬 축-각 표현과 정확히 같으며, `cv2.Rodrigues()`가 바로 exp/log map 변환을 수행한다. 또한 반환된 `(R, tvec)`는 월드 좌표를 카메라 좌표로 옮기는 변환이므로, 월드 공간에서 카메라가 서 있는 좌표를 얻으려면 역변환을 취해야 한다.

```python
# rvec → 회전 행렬 변환
R, _ = cv2.Rodrigues(rvec)

# 카메라의 월드 좌표 위치
camera_position = -R.T @ tvec
```

실무에서 `solvePnP`를 호출할 때는 몇 가지 조건을 함께 챙겨야 한다.

- `solvePnP`의 결과는 **세계 → 카메라** 변환이므로, 지도 위에 카메라 궤적을 찍으려면 위처럼 `-R.T @ tvec`로 역변환해 월드 좌표를 얻는다.
- 함수는 보통 최소 4개 이상의 대응점을 요구한다. P3P 역시 4개까지 나오는 후보 해 가운데 진짜 하나를 고르려면 4번째 점이 필요하고, 기본값인 `SOLVEPNP_ITERATIVE`는 초기값을 DLT로 잡기 때문에 3D 점들이 한 평면 위에 놓여 있지 않다면 6점 이상이 필요하다. 실제 환경에서는 대응점이 많을수록 노이즈에 안정적이며, 잘못된 매치가 섞일 여지가 있다면 `cv2.solvePnPRansac()`으로 outlier를 걸러 내야 한다.
- `flags` 인자로는 반복 최적화 기반의 `cv2.SOLVEPNP_ITERATIVE`(기본값, LM), 최소 4점을 쓰는 `cv2.SOLVEPNP_P3P`, 많은 점에서 빠르고 안정적인 `cv2.SOLVEPNP_EPNP` 가운데 문제 상황에 맞는 해법을 고른다.

(참고: [다크 프로그래머 — solvePnP 함수 사용법과 Rodrigues 표현법](https://darkpgmr.tistory.com/99))

> 추천 자료
> - [Lepetit et al., "EPnP: An Accurate O(n) Solution to the PnP Problem" (2009)](https://doi.org/10.1007/s11263-008-0152-6) — EPnP 원논문
> - [OpenCV solvePnP 문서](https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html) — 다양한 PnP 알고리즘 flag 설명
> - [Multiple View Geometry — Ch. 7](https://www.robots.ox.ac.uk/~vgg/hzbook/) — PnP 문제의 수학적 배경

---

## 9.7 RANSAC 변종 [심화]

앞선 Essential Matrix, Fundamental Matrix, PnP 추정에는 공통의 적이 하나 있다. 바로 **오매칭(outlier)**이다. Lowe's ratio test를 통과한 대응점이라도 반복 무늬나 동적 물체 때문에 엉뚱한 짝이 남기 마련이고, 최소자승법 기반 추정기는 이런 오매칭 단 한두 개만 섞여도 카메라 포즈를 엉뚱한 방향으로 꺾어 버린다. RANSAC(RANdom SAmple Consensus)의 기본 알고리즘과 변종 비교표는 3장 §3.7.3에서 다루었으며, OpenCV는 현대적인 RANSAC 개선안들을 **USAC** 프레임워크 아래 하나로 묶어 제공한다.

- **Lo-RANSAC (Locally Optimized RANSAC)**: 무작위 최소 표본으로 괜찮은 모델을 하나 찾으면 곧바로 멈추지 않고, 그 모델이 끌어모은 inlier 전체를 써서 모델을 다시 정밀하게 최적화(local optimization)한다. 다만 inlier 비율이 매우 높은 장면에서는 이 지역 최적화 단계가 전체 실행 시간의 상당 부분을 차지할 수 있다(Lebeda, Matas, Chum, BMVC 2012).
- **PROSAC (Progressive Sample Consensus)**: 모든 대응점을 똑같은 확률로 뽑는 대신, 디스크립터 매칭 점수가 높은 점부터 우선적으로 샘플링한다. 신뢰할 만한 매치가 상위권에 몰려 있다면 초기 몇 번의 반복만으로도 좋은 모델을 빠르게 찾아낸다.
- **MAGSAC++ (Marginalizing Sample Consensus)**: 기존 RANSAC은 "재투영 오차 1픽셀 이하만 inlier"처럼 경계 임계값(노이즈 스케일 `σ`)을 딱 잘라 정해야 해서 임계값 설정에 민감했다. MAGSAC++는 `σ`를 단일 값으로 고정하지 않고 상한선 안의 여러 `σ` 값에 대해 주변화(marginalize)해 적분하므로 임계값 튜닝 부담이 훨씬 적다. OpenCV에서는 `USAC_MAGSAC` 플래그로 바로 쓸 수 있다.

Fundamental matrix나 Homography를 추정할 때 `method=cv2.USAC_MAGSAC`만 넘기면 기본 RANSAC을 대체할 수 있다.

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

실제 파이프라인에 적용할 때는 파라미터의 정확한 의미를 염두에 두고 조정해야 한다.

- **Iteration 수와 `confidence`**: `confidence=0.999`는 최종 채택된 모델이 물리적으로 참일 확률이 아니라, 주어진 inlier 비율 아래에서 "모두 inlier로만 이루어진 최소 표본을 적어도 한 번 뽑을 확률이 99.9%"가 되도록 반복 횟수를 계산한다는 뜻이다. 필요한 반복 횟수는 `N = log(1−p)/log(1−w^s)` (`w`: inlier 비율, `s`: 최소 표본 크기)를 따르므로, 오매칭이 많아 `w`가 낮아질수록 요구 반복 수가 가파르게 늘어난다.
- **Threshold 설정**: MAGSAC++가 임계값에 덜 민감하기는 하지만, 노이즈 상한 역할을 하는 `ransacReprojThreshold` 초기값은 여전히 건네주어야 한다. 위 코드의 `1.0`, `3.0`은 어디까지나 출발점일 뿐이며, 사용하는 카메라의 해상도와 픽셀 노이즈 수준에 맞추어 검증 후 정해야 한다.
- **알고리즘 선택**: PROSAC과 MAGSAC++ 가운데 어느 쪽이 더 빠르고 정확한지는 매칭 점수의 신뢰도, outlier 비율, 구현체에 따라 달라지므로 같은 데이터와 시간 예산 안에서 직접 비교해 고른다.

> 추천 자료
> - [Barath et al., "MAGSAC++, a Fast, Reliable and Accurate Robust Estimator" (2020)](https://arxiv.org/abs/1912.05909) — MAGSAC++ 원논문
> - [OpenCV USAC 문서](https://docs.opencv.org/4.x/de/d3e/tutorial_usac.html) — OpenCV의 universal RANSAC 프레임워크
> - [Chum & Matas, "Matching with PROSAC" (2005)](https://doi.org/10.1109/CVPR.2005.221) — PROSAC 원논문

---

## 9.8 학습 기반 특징 매칭 [심화]

지금까지 살펴본 ORB나 SIFT 같은 hand-crafted feature(사람이 수식으로 설계한 특징)는 빠르고 해석하기 좋지만, 무늬 없는 흰 벽이나 똑같은 타일이 끝없이 이어지는 복도, 혹은 낮과 밤처럼 극단적으로 조명이 바뀌는 현장에서는 매칭이 급격히 무너진다. 2018년 SuperPoint가 등장한 이후, 특징점 추출과 매칭 자체를 데이터로 학습하는 신경망 기반 접근이 이 빈틈을 메우고 있다.

학습 기반 매칭 파이프라인은 특징점 하나를 뽑아 개별적으로 비교하던 방식에서 점차 두 이미지 전체의 문맥을 함께 보는 방향으로 발전해 왔다.

```
SuperPoint (2018) → SuperGlue (2020) → LightGlue (2023)
  [키포인트 검출+기술]    [그래프 신경망 매칭]     [경량화된 매칭]
```

- **SuperPoint**: 자기지도 학습으로 키포인트 검출기와 디스크립터를 동시에 학습한다. 합성 원근 변환(homographic adaptation)을 영상에 여러 번 걸고 역변환해 의사 정답(pseudo ground truth)을 만들어 내며, 원 논문 평가 조건에서는 여러 고전 방법보다 높은 반복성(repeatability)을 기록했다.
- **SuperGlue**: 두 이미지의 키포인트 집합을 그래프로 묶고 attention mechanism으로 매칭한다. 같은 이미지 내부 키포인트의 공간 관계를 살피는 self-attention과 두 이미지 사이의 대응 단서를 교환하는 cross-attention을 번갈아 거친 뒤, Sinkhorn algorithm으로 최적 할당(optimal assignment) 문제를 푼다. 원 논문 평가에서 뛰어난 매칭 성능을 보였으나, 연산량과 지연 시간은 입력 특징 수와 하드웨어에 따라 달라진다.
- **LightGlue**: SuperGlue를 실시간 응용에 맞게 가볍게 다듬은 후속 모델이다. 쉬운 이미지 쌍은 얕은 층에서 adaptive early stopping으로 일찍 끝내고 어려운 쌍만 깊은 층까지 통과시키며(adaptive depth), 매칭 가망이 없는 점은 중간에 쳐낸다(point pruning). 원 논문 평가에서는 이를 바탕으로 SuperGlue와 비슷한 정확도를 유지하면서 더 낮은 지연을 달성했다(실제 배수는 하드웨어와 설정에 따라 달라진다).
- **LoFTR (Detector-Free Local Feature Matching)**: 아예 키포인트 검출 단계를 없애고 Transformer 기반의 coarse-to-fine 방식으로 이미지 전체에 대해 조밀한(dense) 매칭을 수행한다. 특징점을 찍기 힘든 텍스처 부족 영역에서도 대응점을 뽑아내는 것이 가장 큰 강점이지만, 그 대가로 추론 속도가 느리고 GPU 메모리를 많이 차지한다.

실제 로봇에 올릴 때는 각 접근법의 계산 자원과 현장 난이도를 나란히 견주어 보아야 한다.

| 항목 | ORB/SIFT | SuperPoint+LightGlue | LoFTR |
|------|----------|---------------------|-------|
| 속도 (CPU) | 빠름 | 느림 | 매우 느림 |
| 속도 (GPU) | 해당 없음 | 보통 | 느림 |
| 텍스처 부족 영역 | 자주 실패 | 보통 | 강함 |
| 반복 패턴 | 약함 | 강함 | 강함 |
| GPU 의존성 | 없음 | 높음 | 매우 높음 |
| 실시간 로봇 적용 | 쉬움 | 조건부 가능 | 어려움 |

다음은 `kornia` 라이브러리를 이용해 KeyNetAffNetHardNet 특징 추출기와 LightGlue 매처(`LightGlueMatcher`)를 묶어 두 이미지의 대응점을 찾는 코드다. LightGlue는 디스크립터 벡터뿐 아니라 키포인트의 위치와 스케일·방향을 담은 기하 정보(`lafs`)를 함께 입력으로 받아 매칭을 수행한다.

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

SuperPoint+LightGlue나 위 예제의 KeyNetAffNetHardNet+LightGlue 같은 LightGlue 계열은 GPU 자원이 확보되어 있고 고전 특징이 무너지는 까다로운 환경에서 매칭 강건성이 꼭 필요할 때 좋은 선택지가 된다. 반면 GPU를 싣기 어렵거나 전력·지연 예산이 빠듯한 소형 로봇에서는 ORB 같은 고전 특징이 여전히 가장 단단한 기준선이다. 결국 최종 선택은 목표 데이터 위에서 정확도, 추론 지연, 메모리 사용량을 함께 측정해 결정해야 한다.

이처럼 9장에서는 픽셀 필터링에서 카메라 모델, 특징점, 에피폴라 기하학, PnP와 RANSAC, 그리고 학습 기반 매칭까지 이미지 속 **기하 정보**를 뽑아내는 법을 살펴보았다. 하지만 로봇이 물리 세계에서 물체를 집고 피해 다니려면 "모서리가 어디 있는가"를 넘어 "저기 놓인 것이 머그컵인지 사람인지", "어느 픽셀까지가 주행 가능한 바닥인지"라는 **의미(semantics)**를 알아야 한다. 이어지는 10장에서는 딥러닝을 이용해 영상에서 물체의 종류, 위치, 픽셀 영역, 그리고 단안 깊이까지 읽어 내는 방법을 다룬다.

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
