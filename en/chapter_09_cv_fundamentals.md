# Ch.9 — Computer Vision Fundamentals

While Ch.8 explored how robots learn policies directly from experience and demonstrations, most manipulation and navigation pipelines still rely on vision to turn raw camera pixels into geometric measurements. Understanding classical image processing and multi-view camera geometry makes it possible to recover 3D structure from 2D images—and to trace whether a tracking failure in visual SLAM or manipulation stems from noisy preprocessing, uncalibrated optics, or broken feature correspondences.

---

## 9.1 Image Processing

Raw camera frames contain sensor noise, lighting variations, and hardware-specific byte layouts that must be normalized before geometric estimation can succeed. Filtering, edge detection, and morphological operations clean up the input signal and make it far easier to diagnose whether a downstream perception error began during preprocessing.

### 9.1.1 Introduction to OpenCV

**OpenCV (Open Source Computer Vision Library)** is the standard open-source library for classical computer vision in robotics. It provides both C++ and Python bindings covering everything from image I/O and filtering to feature extraction, camera calibration, and multi-view geometry.

To install OpenCV along with extra algorithms (such as SIFT and additional feature descriptors) in Python, run:

```bash
pip install opencv-python opencv-contrib-python
```

The snippet below illustrates the minimal OpenCV workflow for loading an image from disk, converting it to grayscale, and displaying it in a window:

```python
import cv2
import numpy as np

# Read an image
img = cv2.imread('image.jpg')

# Convert to grayscale
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Display the image
cv2.imshow('Image', img)
cv2.waitKey(0)
cv2.destroyAllWindows()
```

**Caveat**: OpenCV stores color images in BGR rather than RGB channel order. Before passing an OpenCV array to an RGB-based library such as Matplotlib, convert it with `cv2.cvtColor(img, cv2.COLOR_BGR2RGB)` so colors are not swapped.

> **Further reading**
> - [OpenCV official tutorials](https://docs.opencv.org/4.x/d9/df8/tutorial_root.html) — Well-organized Python and C++ walkthroughs across core vision modules.
> - [First Principles of Computer Vision](https://www.youtube.com/channel/UCf0WB91t8Ky6AuYcQV0CcLw) — Columbia University Prof. Shree Nayar's lectures offering intuitive derivations of image-processing fundamentals.
> - [Szeliski, "Computer Vision: Algorithms and Applications"](https://szeliski.org/Book/) — Free PDF of the standard computer vision reference textbook.
> - [Stanford CS131 — Computer Vision: Foundations and Applications](http://vision.stanford.edu/teaching/cs131_fall1415/schedule.html) — More introductory than CS231n; a practical starting point if you want to build up from image processing.

### 9.1.2 Filtering

Image filtering suppresses unwanted sensor noise while preserving meaningful spatial structure. Because computing intensity derivatives amplifies high-frequency noise, smoothing (blurring) is typically applied before edge detection or thresholding.

OpenCV offers linear Gaussian smoothing for general noise reduction and nonlinear median filtering, which is particularly effective against salt-and-pepper noise without washing out sharp boundaries:

```python
# Gaussian Blur
blurred = cv2.GaussianBlur(img, (5, 5), 0)

# Median Blur (effective for noise removal)
median = cv2.medianBlur(img, 5)
```

Once high-frequency noise is attenuated, gradient-based operators such as Sobel and multi-stage detectors such as Canny extract intensity discontinuities that trace object boundaries:

```python
# Canny Edge Detection
edges = cv2.Canny(gray, threshold1=50, threshold2=150)

# Sobel Operator
sobel_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
sobel_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
```

Edge maps highlight object contours and sharp illumination transitions. Because Canny output is sensitive to its hysteresis thresholds (`threshold1` and `threshold2`), inspect how real boundaries drop out or spurious noise reappears as those values vary.

> **Further reading**
> - [First Principles of Computer Vision — Edge Detection](https://www.youtube.com/playlist?list=PL2zRqk16wsdoCCLpouGuRbcJFBVVJlvgr) — Visual explanation of the mathematics behind edge detection.
> - [OpenCV filtering tutorial](https://docs.opencv.org/4.x/d4/d13/tutorial_py_filtering.html) — Hands-on code examples for smoothing kernels.
> - [Papers With Code — Edge Detection](https://paperswithcode.com/task/edge-detection) — Benchmarks and recent papers on edge detection.

> **Exercise**: [Canny Edge Detection](https://alexjunholee.github.io/robotics-practice/app.html#canny_edge)
> Adjust the Canny edge detector's thresholds in real time and observe how the extracted contours respond.

> **Exercise**: [Convolution Visualization](https://alexjunholee.github.io/robotics-practice/app.html#convolution)
> Apply different convolution kernels to an image to build intuition for how linear spatial filtering works.

### 9.1.3 Morphology

After thresholding or semantic segmentation produces a binary mask, isolated noise specks and tiny interior gaps often remain. Morphological operations reshape binary regions using a small structuring element (`kernel`) to clean up those artifacts.

The four core operations combine two primitives—erosion (shrinking foreground regions) and dilation (expanding them)—into compound filters for cleaning masks:

```python
kernel = np.ones((5, 5), np.uint8)

# Erosion
eroded = cv2.erode(binary_img, kernel, iterations=1)

# Dilation
dilated = cv2.dilate(binary_img, kernel, iterations=1)

# Opening (erosion -> dilation): removes noise
opening = cv2.morphologyEx(binary_img, cv2.MORPH_OPEN, kernel)

# Closing (dilation -> erosion): fills holes
closing = cv2.morphologyEx(binary_img, cv2.MORPH_CLOSE, kernel)
```

Opening applies erosion followed by dilation, stripping away small foreground specks and thin protrusions while preserving the approximate size of larger objects. Closing reverses the order—dilation followed by erosion—to bridge narrow gaps and fill small holes inside a mask.

> **Further reading**
> - [OpenCV Morphological Operations](https://docs.opencv.org/4.x/d9/d61/tutorial_py_morphological_ops.html) — Illustrated guide to erosion, dilation, opening, and closing.
> - [First Principles of Computer Vision — Binary Image Processing](https://www.youtube.com/watch?v=IcBzsP-fvPo) — Principles of binary region analysis and morphology.

---

## 9.2 Camera Model

Recovering 3D geometry from a 2D image is impossible without modeling how light rays from the world project onto the image sensor. SLAM, 3D reconstruction, and visual servoing all begin with this projection model—and this is where the linear algebra and rigid-body transforms introduced in Ch.3 become concrete engineering tools.

### 9.2.1 Pinhole Model

The pinhole model idealizes a camera as an infinitesimally small aperture that projects every 3D point onto a 2D image plane along a straight ray. Inverting that projection—turning a pixel coordinate `(u, v)` back into a 3D ray or position—requires expressing the imaging geometry mathematically.

In normalized camera coordinates `(X/Z, Y/Z, 1)`, perspective projection maps a 3D point in the camera frame to pixel coordinates `(u, v)` through the 3×3 **intrinsic matrix** `K`:

```
[u]   [f_x  0   c_x] [X/Z]
[v] = [0   f_y  c_y] [Y/Z]
[1]   [0    0    1 ] [ 1 ]
```

Two sets of parameters govern the full world-to-pixel mapping:

**Intrinsic Parameters** (internal camera geometry, forming the 3×3 matrix `K`):
- `f_x, f_y`: focal length expressed in horizontal and vertical pixel units
- `c_x, c_y`: principal point, where the optical axis pierces the image plane (often offset slightly from the geometric center of the sensor)

**Extrinsic Parameters** (camera pose in the environment):
- `R`: 3×3 rotation matrix
- `t`: 3×1 translation vector
- Together `[R|t]` defines the rigid-body transform from the world frame to the camera frame

In short, `K` captures the internal optics and sensor geometry, while `[R|t]` specifies where the camera sits in the world and which way it faces. Multiplying them projects any 3D world point onto a 2D pixel.

> **Further reading**
> - [Stanford CS231A — Camera Models](https://web.stanford.edu/class/cs231a/) — Core lecture notes on geometric computer vision.
> - [First Principles of CV — Camera and Imaging](https://www.youtube.com/playlist?list=PL2zRqk16wsdoYzrWStQ2SQHXXS2K6ofd4) — Step-by-step progression from ideal pinholes to physical lenses.
> - [Szeliski Ch.2 — Image Formation](https://szeliski.org/Book/) — Mathematical foundations of image projection.
> - [Jinyong Jeong blog — Camera Models and Distortion (Perspective, Fisheye, Omni)](https://jinyongjeong.github.io/2020/06/15/Camera_and_distortion_model/) — Detailed comparison of Perspective, Equidistant, and Omni camera models.
> - [Jinyong Jeong blog — OpenCV Camera model notes](https://jinyongjeong.github.io/2020/06/19/SLAM-Opencv-Camera-model-%EC%A0%95%EB%A6%AC/) — Notes on OpenCV's pinhole and fisheye implementations.

> **Exercise**: [Camera Projection](https://alexjunholee.github.io/robotics-practice/app.html#camera_projection)
> Interactively adjust intrinsic and extrinsic parameters to see how a 3D point projects onto a 2D image plane.

### 9.2.2 Distortion Models

Real cameras do not behave like ideal pinholes. As light refracts through curved glass elements—and especially through wide-angle or fisheye optics—straight lines in the physical world bow into curves on the sensor. Skipping distortion correction degrades SLAM trajectory accuracy and warps 3D reconstructions.

Lens distortion is commonly decomposed into two geometric effects:

- **Radial distortion**: grows stronger with distance from the image center and is modeled by coefficients `k1, k2, k3`. When `k1 < 0`, straight lines bow outward (**barrel distortion**, typical of wide-angle lenses); when `k1 > 0`, they pinch inward (**pincushion distortion**).
- **Tangential distortion**: arises when the lens assembly is not perfectly parallel to the sensor plane and is modeled by `p1, p2`. Although usually smaller than radial distortion, it becomes significant in low-cost modules with loose mechanical alignment tolerances.

OpenCV can remove distortion frame by frame with `cv2.undistort()`, or precompute remap tables once when the intrinsics stay fixed:

```python
# Simple correction (computed per frame - slow)
undistorted = cv2.undistort(distorted, K, dist_coeffs)

# Precompute correction maps and reuse (a common SLAM pipeline pattern)
map1, map2 = cv2.initUndistortRectifyMap(K, dist_coeffs, None, K, (w, h), cv2.CV_32FC1)
undistorted = cv2.remap(distorted, map1, map2, cv2.INTER_LINEAR)
```

In real-time SLAM pipelines where `K` and `dist_coeffs` are static, precomputing `map1` and `map2` with `initUndistortRectifyMap()` and applying `cv2.remap()` avoids recomputing the polynomial warp on every frame, saving substantial CPU time.

**Fisheye lenses**: for ultra-wide fields of view, a low-order radial-tangential pinhole model breaks down. Fisheye models instead express projection as a function of incidence angle θ (for instance, the equidistant ideal `r = f·θ`), and OpenCV provides dedicated routines under `cv2.fisheye`. Rather than picking a model from a single field-of-view cutoff, compare held-out reprojection residuals across candidate lens models.

(See: [Dark Programmer — Camera distortion correction](https://darkpgmr.tistory.com/31), [Jinyong Jeong blog — Camera Models and Distortion](https://jinyongjeong.github.io/2020/06/15/Camera_and_distortion_model/))

> **Further reading**
> - [OpenCV Camera Calibration and 3D Reconstruction](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html) — Reference documentation with the exact distortion model equations.
> - [First Principles of CV — Lens Related Issues](https://www.youtube.com/watch?v=hzOeqCb2Fg4) — Physical intuition for optical aberrations and lens distortion.

> **Exercise**: [Lens Distortion Visualization](https://alexjunholee.github.io/robotics-practice/app.html#lens_distortion)
> Vary radial and tangential distortion coefficients interactively and observe how the image grid deforms.

### 9.2.3 Calibration

A camera model is only useful once you actually know its intrinsic matrix `K` and distortion coefficients. Any error in calibration propagates straight into stereo depth, visual SLAM, and hand-eye transforms—a classic case of "garbage in, garbage out."

**Camera calibration** estimates these parameters by observing a planar target of known geometry (most often a checkerboard) from diverse viewpoints, generating dozens or hundreds of known 3D-to-2D point correspondences:

```python
# Detect checkerboard corners
ret, corners = cv2.findChessboardCorners(gray, (9, 6), None)

# Refine corners
corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

# Calibration
ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
    object_points, image_points, gray.shape[::-1], None, None
)
```

From these 3D-2D pairs, the solver jointly estimates three parameter groups:

1. **Intrinsic parameters (`fx, fy, cx, cy`)**: focal length and principal point. Recalibrate whenever focus, zoom, resolution, cropping, operating temperature, or mechanical mounting changes.
2. **Distortion coefficients (`k1, k2, p1, p2, k3`)**: lens and assembly distortion parameters under the chosen projection model; camera price alone does not predict their magnitude.
3. **Extrinsic parameters (`R, t`)**: the camera pose at each target view. Although produced as a byproduct of calibration, these poses are directly useful in downstream workflows such as hand-eye calibration.

Judge a calibration dataset by parameter observability rather than raw image count. Capture the target across both the center and outer edges of the sensor while varying distance and tilt about multiple axes; the exact number of views needed depends on parameter uncertainty, numerical conditioning, and corner-detection quality. Inspect per-image residuals, spatial residual patterns, and held-out view error rather than trusting a single global RMS number—and because expected pixel residuals vary with resolution, lens model, target, and detector, no universal grading threshold applies. Before discarding a high-residual frame, inspect it for motion blur, glare, or corner misdetection, and verify how excluding it affects parameter stability and held-out error.

(See: [Dark Programmer — Camera calibration](https://darkpgmr.tistory.com/32))

For multi-camera rigs or tightly coupled visual-inertial sensors, **Kalibr** extends calibration beyond single-camera checkerboards:
- ROS-based calibration framework for multi-camera and camera-IMU systems
- Uses AprilTag calibration boards for robust partial-view detection
- Jointly estimates spatial extrinsics and inter-sensor time offsets

> **Further reading**
> - [OpenCV camera calibration tutorial](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html) — Step-by-step checkerboard calibration walkthrough.
> - [Kalibr official Wiki](https://github.com/ethz-asl/kalibr/wiki) — Documentation for the widely used camera-IMU calibration suite.
> - [Zhang, "A Flexible New Technique for Camera Calibration" (2000)](https://www.microsoft.com/en-us/research/publication/a-flexible-new-technique-for-camera-calibration/) — The foundational planar-target calibration paper behind OpenCV's implementation.
> - [Tangram Vision Blog](https://www.tangramvision.com/blog) — Practical engineering articles on camera calibration and multi-sensor fusion.

---

## 9.3 Features

Once a camera is calibrated, visual SLAM and visual odometry still need a reliable way to recognize the same physical 3D points as the camera moves across frames. Local **features** solve this correspondence problem by pairing a repeatable **keypoint** location with a numerical **descriptor** vector summarizing its surrounding patch.

### 9.3.1 Keypoint Detection

Keypoint detectors locate salient image points—typically corners or blobs—that remain detectable across changes in viewpoint, scale, and lighting. Over the past three decades, detectors have evolved to trade off invariance, speed, and licensing:

- **Harris Corner**: a classical corner detector based on local intensity gradients. It is accurate on small shifts, but slow and sensitive to scale changes.
- **FAST (Features from Accelerated Segment Test)**: tests intensity differences along a circular pixel ring for very fast corner detection. It suits real-time pipelines, though it is not inherently scale-invariant.
- **ORB (Oriented FAST and Rotated BRIEF)**: pairs multi-scale FAST detection with orientation estimation and a rotated BRIEF descriptor. Because it is fast and patent-free, the ORB-SLAM family adopts it as its real-time workhorse.
- **SIFT (Scale-Invariant Feature Transform)**: introduced by Lowe (2004), SIFT provides strong scale and rotation invariance with high repeatability at higher computational cost. Previously patented (now expired), it remains the standard reference baseline when comparing newer detectors such as SURF and ORB.
- **SuperPoint**: a self-supervised deep neural network that jointly predicts keypoints and descriptors. It achieves high repeatability and strong matching accuracy, at the cost of requiring a GPU for fast inference.

> **Further reading**
> - [Lowe, "Distinctive Image Features from Scale-Invariant Keypoints" (2004)](https://www.cs.ubc.ca/~lowe/papers/ijcv04.pdf) — The original SIFT paper.
> - [Rublee et al., "ORB: An efficient alternative to SIFT or SURF" (2011)](https://ieeexplore.ieee.org/document/6126544) — The original ORB paper.
> - [First Principles of CV — Feature Detection](https://www.youtube.com/playlist?list=PL2zRqk16wsdqXEMpHrc4Qnb5rA1Cylrhx) — Visual explanation of corner and scale-space keypoint detection.
> - [DeTone et al., "SuperPoint: Self-Supervised Interest Point Detection and Description" (2018)](https://arxiv.org/abs/1712.07629) — Landmark paper in learned keypoint extraction.
> - [Dark Programmer — Image keypoint extraction methods](https://darkpgmr.tistory.com/131) — Comparative overview of SIFT, HOG, Haar, Ferns, LBP, MCT, and related descriptors.

### 9.3.2 Descriptor

Finding a keypoint only specifies *where* an interest point lies; a **descriptor** encodes *what* the surrounding image patch looks like so the same physical point can be matched numerically across two frames:

- **BRIEF (Binary Robust Independent Elementary Features)**: builds a compact binary string (`0` or `1`) by comparing pairs of pixel intensities around a keypoint. Matching reduces to fast Hamming distance, though plain BRIEF is not rotation-invariant.
- **ORB Descriptor**: steers BRIEF using the keypoint's dominant orientation, yielding a rotation-aware 256-bit binary vector. Because comparing binary descriptors requires only bitwise XOR and population count (Hamming distance) rather than floating-point Euclidean distance as in SIFT, ORB runs dramatically faster on resource-constrained embedded CPUs.
- **SuperGlue**: replaces isolated descriptor comparison with a Graph Neural Network that reasons jointly over keypoints across both images, improving robustness on repetitive patterns and low-texture surfaces (with **LightGlue** offering a faster, lighter successor).

> **Further reading**
> - [Sarlin et al., "SuperGlue: Learning Feature Matching with Graph Neural Networks" (2020)](https://arxiv.org/abs/1911.11763) — Foundational work on learned feature matching.
> - [OpenCV Feature Matching tutorial](https://docs.opencv.org/4.x/dc/dc3/tutorial_py_matcher.html) — Practical guide to `BFMatcher` and `FLANN`.

### 9.3.3 Feature Matching

Matching descriptors between consecutive frames provides the point correspondences needed to estimate camera motion. Visualizing those raw matches is also one of the fastest ways to debug tracking failures—separating scenes that simply lack features from scenes dominated by false matches.

The example below extracts ORB features from two images and compares brute-force cross-checked matching with k-nearest-neighbor filtering via Lowe's ratio test:

```python
# Extract ORB keypoints and descriptors
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

Lowe's ratio test queries the two closest descriptor matches (`k=2`) and keeps the top match `m` only when its distance is distinctly smaller than the runner-up `n`, discarding ambiguous matches on repetitive textures where two candidates score nearly the same. Lowe used `0.8` in the original SIFT paper, while the snippet above uses `0.75`; in practice, tuning between `0.6` and `0.8` trades recall against precision for a given scene.

> **Further reading**
> - [OpenCV Feature Matching](https://docs.opencv.org/4.x/dc/dc3/tutorial_py_matcher.html) — Code examples for `BFMatcher`, `FLANN`, and the ratio test.
> - [Computerphile — SIFT Features](https://www.youtube.com/watch?v=ram-jbLJjFg) — Intuitive walkthrough of feature extraction and matching.

> **Exercise**: [Feature Matching](https://alexjunholee.github.io/robotics-practice/app.html#feature_matching)
> Test feature matching between two views interactively and observe how Lowe's ratio test filters ambiguous correspondences.

---

## 9.4 Epipolar Geometry

Once feature matching links pixels across two viewpoints, **epipolar geometry** describes the exact geometric constraint tying those two views together. Given two images of a static scene, epipolar geometry lets us recover the relative motion between the cameras and triangulate 3D scene structure—forming the mathematical backbone of Visual Odometry and Structure from Motion (SfM), and putting SVD and eigendecomposition from linear algebra directly to work.

### 9.4.1 Essential Matrix (E)

The **essential matrix (`E`)** encodes the relative rotation `R` and translation `t` between a pair of **calibrated** cameras whose intrinsics `K` are known:

```
x2^T E x1 = 0
```

Here `x1` and `x2` are normalized image coordinates (obtained by multiplying pixel coordinates by `K^(-1)`), and `E = [t]_× R` combines the skew-symmetric cross-product matrix of `t` with the rotation `R`. Because `E` has five degrees of freedom (three for rotation and two for translation direction up to scale), the **5-point algorithm**—paired with RANSAC to reject outliers—can estimate `E` from as few as five correspondence pairs.

Decomposing `E` via SVD yields candidate `(R, t)` pairs for the relative camera motion. Enforcing the positive-depth (cheirality) constraint—requiring triangulated 3D points to lie in front of both cameras—selects the physically valid configuration, though the overall metric scale of `t` cannot be recovered from a monocular pair alone without external scale cues. This decomposition is the core geometric step of monocular Visual Odometry.

> **Exercise**: [Epipolar Geometry Visualization](https://alexjunholee.github.io/robotics-practice/app.html#epipolar)
> Explore epipolar lines and epipoles across two viewpoints interactively to build geometric intuition for the essential and fundamental matrices.

### 9.4.2 Fundamental Matrix (F)

When camera intrinsics are unknown, the **fundamental matrix (`F`)** generalizes the epipolar constraint directly to raw pixel coordinates `p1` and `p2` between a pair of **uncalibrated** cameras:

```
p2^T F p1 = 0
```

It relates to the essential matrix through the two cameras' intrinsic matrices via `F = K2^(-T) E K1^(-1)` and can be estimated from at least eight point correspondences using the **8-point algorithm** (typically with coordinate normalization and RANSAC).

In practice, the distinction between `E` and `F` comes down to calibration: use `F` when operating directly in pixel coordinates without known intrinsics, and use `E` once the cameras are calibrated so normalized rays isolate pure rigid-body motion.

### 9.4.3 Triangulation

With relative camera poses `(P1, P2)` and matched 2D points `(pts1, pts2)` established, **triangulation** computes each observed point's 3D coordinates by intersecting its back-projected viewing rays from both viewpoints—the same geometric principle behind human binocular depth perception.

OpenCV's `cv2.triangulatePoints()` solves this intersection in homogeneous coordinates, which are then divided by their fourth component to yield Cartesian 3D coordinates:

```python
# OpenCV triangulation
points_4d = cv2.triangulatePoints(P1, P2, pts1, pts2)
points_3d = points_4d[:3] / points_4d[3]  # Homogeneous -> Cartesian
```

In practice, triangulation requires balancing the **baseline** (distance between the two camera centers): if the baseline is too narrow, small pixel noise causes large depth errors; if it is too wide, viewpoint changes make it difficult to observe and match the same points across both views.

> **Further reading**
> - [Stanford CS231A — Epipolar Geometry](https://web.stanford.edu/class/cs231a/) — Lecture notes with clear derivations of `E`, `F`, and triangulation.
> - [Hartley & Zisserman, "Multiple View Geometry in Computer Vision"](https://www.robots.ox.ac.uk/~vgg/hzbook/) — The definitive reference on multi-view geometry.
> - [First Principles of CV — Stereo Vision](https://www.youtube.com/playlist?list=PL2zRqk16wsdoYzrWStQ2SQHXXS2K6ofd4) — Visual walkthrough of stereo and epipolar geometry.
> - [Dark Programmer — Image Geometry series (7 parts: coordinate frames to Epipolar)](https://darkpgmr.tistory.com/77) — A systematic Korean-language series spanning coordinate frames, homogeneous coordinates, 2D/3D transforms, homography, imaging, and epipolar geometry.

> **Exercise**: [Homography Visualization](https://alexjunholee.github.io/robotics-practice/app.html#homography)
> Manipulate a planar homography interactively and see how four point correspondences determine a projective transformation.

---

## 9.5 Optical Flow

Whereas feature matching pairs discrete descriptors across frames—even across wide viewpoint jumps—**optical flow** estimates the apparent 2D motion of pixels between consecutive frames in a video stream. Knowing where pixels shift from frame `t` to `t+1` directly supports visual odometry, moving-obstacle detection, and collision avoidance, whether computed sparsely at selected points or densely at every pixel.

### 9.5.1 Lucas-Kanade Method

The **Lucas-Kanade** method tracks sparse keypoints by assuming **brightness constancy** (a point's intensity stays unchanged between frames) and **small inter-frame motion** within a local patch:

```python
# Compute optical flow
p1, status, err = cv2.calcOpticalFlowPyrLK(
    prev_gray, curr_gray, p0, None, **lk_params
)
```

In `calcOpticalFlowPyrLK`, `Pyr` stands for **image pyramid**: by solving flow first on coarse downsampled images and refining upward to full resolution, pyramidal Lucas-Kanade tracks larger pixel displacements that would otherwise violate the small-motion assumption.

### 9.5.2 Dense Optical Flow

Dense optical flow estimates a 2D displacement vector for every pixel in the image, trading higher computation for a complete motion field useful in segmentation and dynamic-scene analysis. Classical polynomial-expansion methods such as Farneback run directly on CPU:

```python
# Farneback dense flow
flow = cv2.calcOpticalFlowFarneback(prev_gray, curr_gray, None, 0.5, 3, 15, 3, 5, 1.2, 0)
```

For higher accuracy under large displacements and motion blur, **RAFT (Recurrent All-Pairs Field Transforms)** has become a standard deep-learning baseline for dense optical flow, reporting lower estimation errors than prior methods on both the KITTI and Sintel benchmarks in its 2020 paper.

> **Further reading**
> - [First Principles of CV — Optical Flow](https://www.youtube.com/playlist?list=PL2zRqk16wsdp8KbDfHKvPYNGF2L-zQASc) — Mathematical derivations of brightness constancy and Lucas-Kanade.
> - [Teed & Deng, "RAFT: Recurrent All-Pairs Field Transforms for Optical Flow" (2020)](https://arxiv.org/abs/2003.12039) — The foundational deep optical-flow architecture.
> - [Huang et al., "FlowFormer: A Transformer Architecture for Optical Flow" (ECCV 2022, arXiv:2203.16194)](https://arxiv.org/abs/2203.16194) — Transformer-based dense optical flow.
> - [OpenCV Optical Flow tutorial](https://docs.opencv.org/4.x/d4/dee/tutorial_optical_flow.html) — Complete code examples for Lucas-Kanade and Farneback.

> **Exercise**: [Optical Flow Visualization](https://alexjunholee.github.io/robotics-practice/app.html#optical_flow)
> Compare sparse Lucas-Kanade and dense optical flow interactively and inspect how pixel motion vectors are estimated.

---

## 9.6 Advanced: PnP Problem

Epipolar geometry recovers relative pose from two 2D views, leaving absolute scale unobservable on a monocular camera. Once a 3D map has already been triangulated, however, subsequent frames can localize directly against those known 3D landmarks. **Perspective-n-Point (PnP)** solves this exact problem: estimating a calibrated camera's 6-DoF pose (`R` and `t`) from `n` correspondences between known 3D points and their observed 2D image projections. Frame-to-frame camera tracking in visual SLAM and marker-based localization in AR both rely on PnP at their core.

Given `n` 3D-2D correspondences `{(X_i, x_i)}` and known camera intrinsics `K`, PnP solves for the extrinsics `[R|t]` satisfying the perspective projection equation:

$$x_i = K [R | t] X_i$$

Depending on how many correspondences are available and whether outliers are present, two classical solvers dominate:

- **P3P (3-Point Problem)**: a minimal closed-form solver requiring just 3 correspondences. Three points yield up to four geometrically valid pose candidates, so a 4th point is evaluated to disambiguate the true pose. Because its minimal sample size is only 3 (plus 1 check point), P3P pairs naturally with RANSAC to reject outliers.
- **EPnP (Efficient PnP)**: expresses all `n` 3D points as weighted combinations of 4 virtual control points and estimates those control points' coordinates in the camera frame in `O(n)` time. When dozens or hundreds of correspondences are available, EPnP is both faster and numerically cleaner than running P3P inside many RANSAC iterations.

OpenCV exposes both non-iterative solvers and RANSAC-wrapped robust pose estimation through `cv2.solvePnP()` and `cv2.solvePnPRansac()`:

```python
import cv2
import numpy as np

# 3D world coordinates (n x 3)
object_points = np.array([...], dtype=np.float64)
# Corresponding 2D image coordinates (n x 2)
image_points = np.array([...], dtype=np.float64)
# Camera intrinsics
camera_matrix = np.array([[fx, 0, cx],
                          [0, fy, cy],
                          [0,  0,  1]], dtype=np.float64)
dist_coeffs = np.zeros(4)

# EPnP: a non-iterative closed-form solver, so no initial guess is needed
success, rvec, tvec = cv2.solvePnP(
    object_points, image_points, camera_matrix, dist_coeffs,
    flags=cv2.SOLVEPNP_EPNP
)

# RANSAC version - essential when outliers are present
success, rvec, tvec, inliers = cv2.solvePnPRansac(
    object_points, image_points, camera_matrix, dist_coeffs,
    iterationsCount=1000, reprojectionError=3.0
)
```

In a typical Visual SLAM pipeline, PnP sits inside the per-frame tracking loop:
1. Triangulate 3D map points from earlier keyframes.
2. Project those 3D map points into the incoming frame using the predicted camera motion.
3. Match the projected points against newly detected 2D keypoints.
4. Solve PnP over the resulting 3D-2D correspondences to recover the current frame's pose.

Tracking in ORB-SLAM3 follows this overall structure, with an important architectural distinction: in monocular mode, creating new map points via keyframe triangulation is handled mainly by the Local Mapping thread, whereas in stereo and RGB-D modes, Tracking can instantiate nearby 3D points immediately whenever a new keyframe is inserted.

> **Further reading**
> - [Lepetit et al., "EPnP: An Accurate O(n) Solution to the PnP Problem" (2009)](https://doi.org/10.1007/s11263-008-0152-6) — The original EPnP paper.
> - [OpenCV solvePnP documentation](https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html) — Detailed guide to OpenCV's PnP solver flags and requirements.
> - [Multiple View Geometry — Ch. 7](https://www.robots.ox.ac.uk/~vgg/hzbook/) — Mathematical treatment of camera resectioning and PnP.

**Practical tips for solvePnP**

`cv2.solvePnP()` returns orientation as a 3-vector `rvec` in Rodrigues (axis-angle) form rather than a 3×3 matrix. Its direction defines the rotation axis and its Euclidean norm gives the rotation angle—precisely the `so(3)` Lie algebra representation from Ch.3, with `cv2.Rodrigues()` implementing the exponential/logarithmic map between `rvec` and `R`:

```python
# rvec -> rotation matrix
R, _ = cv2.Rodrigues(rvec)

# Camera position in world coordinates
camera_position = -R.T @ tvec
```

Keep three practical details in mind when calling `solvePnP`:
- **Coordinate direction**: `solvePnP` returns the **world-to-camera** transform `(R, tvec)`. To plot the camera's position in the world frame, invert it via `-R.T @ tvec`.
- **Minimum points and outliers**: at least 4 non-coplanar points are typically required (and more correspondences improve noise resilience). Whenever mismatched keypoints are possible, use `cv2.solvePnPRansac()` to filter outliers automatically.
- **Solver flags**: choose `cv2.SOLVEPNP_ITERATIVE` (default Levenberg-Marquardt refinement), `cv2.SOLVEPNP_P3P` (expects 4 points in `solvePnP`, using the 4th for disambiguation), or `cv2.SOLVEPNP_EPNP` (fast and stable for large point sets).

(See: [Dark Programmer — solvePnP usage and Rodrigues representation](https://darkpgmr.tistory.com/99))

---

## 9.7 Advanced: RANSAC Variants

Both epipolar estimation (`E`, `F`) and PnP degrade rapidly if even a handful of mismatched keypoints slip past Lowe's ratio test. Ch.3 introduced vanilla RANSAC for robust estimation; in real perception pipelines, modern variants substantially improve convergence speed and threshold stability:

| Method | Core idea | Characteristics |
|------|-------------|------|
| **Lo-RANSAC** | local optimization on inliers | converges in fewer iterations than vanilla |
| **PROSAC** | samples in order of matching confidence | tries good matches first, accelerating convergence |
| **MAGSAC++** | marginalizes over σ (inlier threshold) | less sensitive to threshold tuning; still requires validation on the data |

- **Lo-RANSAC (Locally Optimized RANSAC)**: whenever a promising hypothesis is found from a minimal sample, it immediately re-estimates the model using all current inliers—a simple refinement step that pays off strongly when the inlier ratio is low.
- **PROSAC (Progressive Sample Consensus)**: instead of sampling correspondences uniformly at random, it prioritizes pairs with higher descriptor matching scores. When strong matches cluster near the top of the ranking, PROSAC often finds an accurate model within the first few iterations.
- **MAGSAC++ (Marginalizing Sample Consensus)**: avoids committing to a single brittle inlier threshold `σ` by marginalizing over a range of noise scales up to a specified upper bound, making estimation far less sensitive to exact threshold tuning. OpenCV integrates it under the `USAC_MAGSAC` flag.

Switching from classical RANSAC to MAGSAC++ in OpenCV requires changing only the `method` argument in `findFundamentalMat()` or `findHomography()`:

```python
import cv2

# Use MAGSAC++ for fundamental matrix estimation
F, mask = cv2.findFundamentalMat(
    pts1, pts2,
    method=cv2.USAC_MAGSAC,
    ransacReprojThreshold=1.0,
    confidence=0.999,
    maxIters=10000
)

# The same applies to homography estimation
H, mask = cv2.findHomography(
    src_pts, dst_pts,
    method=cv2.USAC_MAGSAC,
    ransacReprojThreshold=3.0
)
```

When tuning these calls in production, keep three practical caveats in mind:
- **Iteration count and `confidence`**: setting `confidence=0.999` chooses the iteration budget so that, under the estimated inlier ratio, there is a 99.9% chance of drawing at least one outlier-free minimal sample—it is *not* the probability that the final model is physically exact. As the inlier ratio drops, required iterations grow exponentially.
- **Threshold initialization**: although MAGSAC++ reduces sensitivity to `ransacReprojThreshold`, it still requires a sensible upper-bound starting value. Treat `1.0` and `3.0` above as initial defaults to validate against your camera's resolution and pixel noise.
- **Empirical selection**: whether PROSAC or MAGSAC++ wins on speed versus accuracy depends on match-score reliability, outlier fraction, and implementation details, so benchmark both under the same data and runtime budget.

> **Further reading**
> - [Barath et al., "MAGSAC++, a Fast, Reliable and Accurate Robust Estimator" (2020)](https://arxiv.org/abs/1912.05909) — The original MAGSAC++ paper.
> - [OpenCV USAC documentation](https://docs.opencv.org/4.x/d1/df1/md__build_4rdparty_ippicv_ippicv_lnx_doc_USAC.html) — Overview of OpenCV's Universal RANSAC framework.
> - [Chum & Matas, "Matching with PROSAC" (2005)](https://doi.org/10.1109/CVPR.2005.221) — The original PROSAC paper.

---

## 9.8 Advanced: Learning-Based Feature Matching

Hand-crafted features such as ORB and SIFT are fast and interpretable, yet they struggle in environments dominated by repetitive tiles, textureless walls, or severe day-to-night illumination shifts. Starting in 2018, neural networks began replacing hand-crafted detectors and matchers—bridging classical multi-view geometry with the deep perception methods explored in Ch.10.

Learned correspondence pipelines have steadily evolved from independent local extraction toward global image-context reasoning and adaptive speed:

```
SuperPoint (2018) -> SuperGlue (2020) -> LightGlue (2023)
  [keypoint detection+description]    [graph neural network matching]     [lightweight matching]
```

- **SuperPoint**: trains a joint keypoint detector and descriptor via self-supervised learning using **homographic adaptation**—applying synthetic warps to images and inverting them to generate pseudo ground truth. In its paper's evaluation, SuperPoint achieved higher repeatability than several classical baselines.
- **SuperGlue**: models the keypoints of both images as a graph and matches them with attention—combining intra-image self-attention and cross-image cross-attention before solving optimal assignment via the Sinkhorn algorithm. It delivered strong matching accuracy in the paper's evaluation, with runtime and memory scaling with feature count and GPU hardware.
- **LightGlue**: streamlines SuperGlue for real-time applications by pruning unmatchable points early and exiting easy image pairs at shallower layers (**adaptive early stopping**). In the paper's evaluation, adaptive depth and point pruning lowered latency relative to SuperGlue at comparable accuracy, with exact speedups depending on hardware and configuration.
- **LoFTR (Detector-Free Local Feature Matching)**: eliminates keypoint detection altogether, using a Transformer in a coarse-to-fine pipeline to establish dense matches across the entire image. Its main advantage is matching across low-texture surfaces where corner detectors fire sparsely, at the expense of slower inference and higher GPU memory consumption.

Choosing between classical and learned matchers on a robot platform depends on compute budget and scene difficulty:

| Item | ORB/SIFT | SuperPoint+LightGlue | LoFTR |
|------|----------|---------------------|-------|
| Speed (CPU) | fast | slow | very slow |
| Speed (GPU) | not applicable | moderate | slow |
| Texture-poor regions | fails | moderate | strong |
| Repetitive patterns | weak | strong | strong |
| GPU dependence | none | high | very high |
| Real-time robot use | easy | conditionally feasible | difficult |

The `kornia` example below pairs a `KeyNetAffNetHardNet` extractor with `LightGlueMatcher`, passing both local descriptors and keypoint geometry (`lafs`) to establish matches across an image pair:

```python
import torch
import kornia
from kornia.feature import LightGlueMatcher, KeyNetAffNetHardNet

# Build the extractor and matcher
extractor = KeyNetAffNetHardNet(num_features=2048).eval()
matcher = LightGlueMatcher("keynet_affnet_hardnet").eval()

# Move to GPU
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
extractor = extractor.to(device)
matcher = matcher.to(device)

# Load images (kornia format: B x C x H x W, 0-1 range; KeyNet takes 1-channel grayscale input)
img0 = kornia.io.load_image(path0, kornia.io.ImageLoadType.GRAY32).unsqueeze(0).to(device)
img1 = kornia.io.load_image(path1, kornia.io.ImageLoadType.GRAY32).unsqueeze(0).to(device)

# Feature extraction
with torch.no_grad():
    lafs0, resp0, desc0 = extractor(img0)   # (lafs, responses, descriptors) tuple
    lafs1, resp1, desc1 = extractor(img1)

    # Matching — LightGlue takes the keypoint geometry (lafs) along with the descriptors
    dists, match_idxs = matcher(desc0[0], desc1[0], lafs0, lafs1)
```

When an onboard GPU is available and robust matching in repetitive or low-light scenes matters, the LightGlue family (paired with SuperPoint or KeyNetAffNetHardNet) is a strong candidate. Under strict CPU-only, latency, or power constraints, classical features such as ORB remain the simpler baseline—always verify the final choice by profiling accuracy, latency, and memory on target data.

> **Further reading**
> - [DeTone et al., "SuperPoint: Self-Supervised Interest Point Detection and Description" (2018)](https://arxiv.org/abs/1712.07629) — The original SuperPoint paper.
> - [Lindenberger et al., "LightGlue: Local Feature Matching at Light Speed" (2023)](https://arxiv.org/abs/2306.13643) — The original LightGlue paper.
> - [Sun et al., "LoFTR: Detector-Free Local Feature Matching with Transformers" (2021)](https://arxiv.org/abs/2104.00680) — The original LoFTR paper.

---

> **Technical Timeline: Computer Vision Fundamentals (Classical Methods)**
> - **~2004**: the era of classical features. Hand-crafted detectors and descriptors such as Harris Corner (1988) and SIFT (2004) set the standard—mathematically principled, though computationally heavy.
> - **2006~2011**: lightweighting for real-time systems. SURF (2006), FAST (2006), BRIEF (2010), and ORB (2011) dramatically cut runtime, while the ORB family bypassed SIFT and SURF patents to make real-time visual SLAM practical.
> - **2015~2019**: deep learning enters local perception. Learned feature pipelines such as SuperPoint (2018) begin outperforming classical descriptors in repeatability and challenging lighting.
> - **2020~**: fusion of geometry and learning. Graph-based matchers such as SuperGlue (2020), detector-free matchers such as LoFTR (2021), and lightweight matchers such as LightGlue (2023) reshape front-end correspondence while classical geometry remains central to the SLAM/VO back-end.
> - **Recent direction**: learned features and matchers increasingly power the front-end, while epipolar geometry, triangulation, and bundle adjustment anchor the geometric back-end—and the next chapter turns fully to deep neural perception across whole scenes.
