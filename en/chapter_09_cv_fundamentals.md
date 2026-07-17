# Ch.9 — Computer Vision Fundamentals


Computer vision turns raw camera data into information a robot can use. Understanding image processing and camera geometry makes it possible to trace failures in SLAM and manipulation pipelines.

---

## 9.1 Image Processing

Raw camera images contain noise and irrelevant variation. Filtering, edge detection, and morphological operations prepare the input and help determine whether an error in a downstream pipeline began during preprocessing.

### 9.1.1 Introduction to OpenCV

**OpenCV (Open Source Computer Vision Library)** is a widely used public CV library.

OpenCV provides C++ and Python bindings for common operations ranging from image I/O and filtering to feature extraction and geometric computation.

**Installation**:

```bash
pip install opencv-python opencv-contrib-python
```

**Basic usage**:

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

**Caveat**: OpenCV uses BGR rather than RGB channel order. Before passing an image to an RGB-based library such as Matplotlib, convert it with `cv2.cvtColor(img, cv2.COLOR_BGR2RGB)`.

> **Further reading**
> - [OpenCV official tutorials](https://docs.opencv.org/4.x/d9/df8/tutorial_root.html) — Python/C++ examples, well organized.
> - [First Principles of Computer Vision](https://www.youtube.com/channel/UCf0WB91t8Ky6AuYcQV0CcLw) — Columbia's Prof. Shree Nayar channel. Intuitive explanations of image processing principles.
> - [Szeliski, "Computer Vision: Algorithms and Applications"](https://szeliski.org/Book/) — Free PDF. The standard textbook in CV.
> - [Stanford CS131 — Computer Vision: Foundations and Applications](http://vision.stanford.edu/teaching/cs131_fall1415/schedule.html) — More introductory than CS231n. Start here if you want to begin from image processing.

### 9.1.2 Filtering

Filtering retains selected image structure while reducing unwanted noise. Blur is often applied before edge detection or segmentation to suppress high-frequency variation in the input.

**Blur**:

```python
# Gaussian Blur
blurred = cv2.GaussianBlur(img, (5, 5), 0)

# Median Blur (effective for noise removal)
median = cv2.medianBlur(img, 5)
```

**Edge Detection**:

```python
# Canny Edge Detection
edges = cv2.Canny(gray, threshold1=50, threshold2=150)

# Sobel Operator
sobel_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
sobel_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
```

Edges highlight object contours and locations with large intensity changes. Canny output is sensitive to its thresholds, so compare which boundaries disappear and which noise remains as the values change.

> **Further reading**
> - [First Principles of Computer Vision — Edge Detection](https://www.youtube.com/playlist?list=PL2zRqk16wsdoCCLpouGuRbcJFBVVJlvgr) — Visual explanation of the mathematics of edge detection.
> - [OpenCV filtering tutorial](https://docs.opencv.org/4.x/d4/d13/tutorial_py_filtering.html) — Follow along immediately with code.
> - [Papers With Code — Edge Detection](https://paperswithcode.com/task/edge-detection) — Latest benchmarks and papers on edge detection.

> **Exercise**: [Canny Edge Detection](https://alexjunholee.github.io/robotics-practice/app.html#canny_edge)
> Adjust the threshold parameters of the Canny edge detector in real time and observe how the output changes.

> **Exercise**: [Convolution Visualization](https://alexjunholee.github.io/robotics-practice/app.html#convolution)
> Apply various kernels to an image and build intuition for how the convolution operation performs filtering.

### 9.1.3 Morphology

Morphological operations modify the shape of regions in a binary image. They can remove small noise specks from a segmentation output or reconnect broken regions.

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

Opening first erodes and then dilates, removing small protrusions and noise. Closing first dilates and then erodes, filling small holes.

> **Further reading**
> - [OpenCV Morphological Operations](https://docs.opencv.org/4.x/d9/d61/tutorial_py_morphological_ops.html) — Explained with visual examples.
> - [First Principles of Computer Vision — Binary Image Processing](https://www.youtube.com/watch?v=IcBzsP-fvPo) — Principles of morphological operations.

---

## 9.2 Camera Model

If you do not understand how a camera reads the world, recovering 3D from a 2D image is impossible. SLAM, 3D reconstruction, visual servoing — all of these start from the camera model. If you have learned linear algebra, this is where you can feel how matrices are actually used.

### 9.2.1 Pinhole Model

The pinhole model is an idealized camera model that projects a 3D point onto a 2D image.

To invert the projection and recover a real-world 3D position from a pixel coordinate (u, v), you need to know this projection relation precisely. The Pinhole Model expresses this relation as equations.

**Projection equation**:

```
[u]   [f_x  0   c_x] [X/Z]
[v] = [0   f_y  c_y] [Y/Z]
[1]   [0    0    1 ] [ 1 ]
```

**Intrinsic Parameters**:
- f_x, f_y: Focal length (in pixels)
- c_x, c_y: Principal point (image center)
- Intrinsic Matrix K (3x3)

**Extrinsic Parameters**:
- R: rotation matrix (3x3)
- t: translation vector (3x1)
- World -> Camera transform

K represents the lens characteristics of the camera, and [R|t] represents where the camera sits in the world and how it is oriented. Multiplying the two maps a 3D point to a 2D pixel.

> **Further reading**
> - [Stanford CS231A — Camera Models](https://web.stanford.edu/class/cs231a/) — Core lectures on geometry-based CV.
> - [First Principles of CV — Camera and Imaging](https://www.youtube.com/playlist?list=PL2zRqk16wsdoYzrWStQ2SQHXXS2K6ofd4) — From pinhole to real lenses, explained step by step.
> - [Szeliski Ch.2 — Image Formation](https://szeliski.org/Book/) — Mathematical foundations of the camera model.
> - [Jinyong Jeong blog — Camera Models and Distortion (Perspective, Fisheye, Omni)](https://jinyongjeong.github.io/2020/06/15/Camera_and_distortion_model/) — Comparison of Perspective, Equidistant, and Omni camera models.
> - [Jinyong Jeong blog — OpenCV Camera model notes](https://jinyongjeong.github.io/2020/06/19/SLAM-Opencv-Camera-model-%EC%A0%95%EB%A6%AC/) — Notes on OpenCV's pinhole/fisheye camera model implementation.

> **Exercise**: [Camera Projection](https://alexjunholee.github.io/robotics-practice/app.html#camera_projection)
> Check interactively how a point in 3D space is projected onto a 2D image through the intrinsic and extrinsic parameters.

### 9.2.2 Distortion Models

Real lenses introduce distortion.

An image taken with a real camera is not as clean as the Pinhole Model assumes. In particular, with wide-angle or fisheye lenses, the distortion that bends straight lines into curves is severe. Skipping distortion correction drops SLAM accuracy sharply and warps 3D reconstruction output.

Camera lenses are not perfect pinholes. Light bends as it passes through the lens, and this bending appears in the image as distortion.

**Radial distortion**: gets worse as you move away from the image center. Modeled by parameters k1, k2, k3. When k1 < 0, you get barrel distortion (straight lines bulge outward); when k1 > 0, pincushion distortion (straight lines curve inward). Most lenses have barrel distortion, and it is more pronounced in wider lenses.

**Tangential distortion**: occurs when the lens is not perfectly parallel to the image sensor. Parameters p1, p2. Usually smaller in effect than radial distortion, but with cheap cameras it is not negligible.

**Distortion correction**:

```python
# Simple correction (computed per frame - slow)
undistorted = cv2.undistort(distorted, K, dist_coeffs)

# Precompute correction maps and reuse (a common SLAM pipeline pattern)
map1, map2 = cv2.initUndistortRectifyMap(K, dist_coeffs, None, K, (w, h), cv2.CV_32FC1)
undistorted = cv2.remap(distorted, map1, map2, cv2.INTER_LINEAR)
```

With fixed camera parameters, precomputing maps through `initUndistortRectifyMap()` and reusing them with `cv2.remap()` avoids rebuilding the mapping every frame. This is a common way to reduce work in a real-time pipeline.

**Fisheye lenses**: a low-order radial-tangential pinhole model may not represent a wide-angle lens adequately. Fisheye models express projection as a function of incidence angle θ (the equidistant ideal is r = f·θ); OpenCV provides matching routines in `cv2.fisheye`. Select the model from the lens projection and held-out reprojection residuals rather than from a single FoV cutoff.

(See: [Dark Programmer — Camera distortion correction](https://darkpgmr.tistory.com/31), [Jinyong Jeong blog — Camera Models and Distortion](https://jinyongjeong.github.io/2020/06/15/Camera_and_distortion_model/))

> **Further reading**
> - [OpenCV Camera Calibration and 3D Reconstruction](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html) — The equations of the distortion models are well organized.
> - [First Principles of CV — Lens Related Issues](https://www.youtube.com/watch?v=hzOeqCb2Fg4) — Physical intuition for why lens distortion occurs.

> **Exercise**: [Lens Distortion Visualization](https://alexjunholee.github.io/robotics-practice/app.html#lens_distortion)
> Adjust radial and tangential distortion parameters and see directly how the image deforms.

### 9.2.3 Calibration

Calibration is the process of estimating a camera's intrinsic and extrinsic parameters.

You can only use the camera model once you actually know K (the intrinsic matrix) and the distortion coefficients. If calibration is inaccurate, everything built on top of it — SLAM, stereo depth estimation, hand-eye calibration — loses accuracy. A textbook case of "garbage in, garbage out".

**Checkerboard method**:

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

Evaluate a calibration set by observability rather than image count. Cover the center and edges of the image and vary range and tilt about several axes. Inspect per-image and image-space residuals and error on held-out views, not only one global RMS value.

Camera calibration estimates the parameters that map points in the 3D world to positions in a 2D image. Capturing a checkerboard from several angles produces dozens to hundreds of correspondences between its known 3D coordinates and the detected 2D image points. From these pairs, calibration estimates:

1. **Intrinsic parameters** (fx, fy, cx, cy): focal length and principal point. Recheck them when focus, zoom, resolution, crop, temperature, or mechanical assembly changes.
2. **Distortion coefficients** (k1, k2, p1, p2, k3): coefficients that approximate lens and assembly distortion under the chosen projection model. Price alone does not predict their magnitude.
3. **Extrinsic parameters** (R, t): the camera pose at each capture position. These are a byproduct of calibration itself, but are used separately in settings like hand-eye calibration.

Capture the checkerboard at several angles and distances, with observations spread across the image. The required count depends on parameter uncertainty, conditioning, and detection quality. Expected reprojection error also depends on resolution, lens model, target, and corner detector, so a universal pixel grading table is inappropriate. Before removing a high-residual image, inspect blur, glare, and detection failure; compare parameter stability and held-out error before and after exclusion.

(See: [Dark Programmer — Camera calibration](https://darkpgmr.tistory.com/32))

**Kalibr**: Multi-camera and Camera-IMU calibration tool
- ROS-based
- Uses AprilTag boards
- Estimates time offsets as well

> **Further reading**
> - [OpenCV camera calibration tutorial](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html) — Step-by-step checkerboard calibration.
> - [Kalibr official Wiki](https://github.com/ethz-asl/kalibr/wiki) — a widely used public Camera-IMU calibration tool.
> - [Zhang, "A Flexible New Technique for Camera Calibration" (2000)](https://www.microsoft.com/en-us/research/publication/a-flexible-new-technique-for-camera-calibration/) — The paper behind OpenCV's current calibration.
> - [Tangram Vision Blog](https://www.tangramvision.com/blog) — Practical engineering posts on camera calibration, sensor fusion, and more.

---

## 9.3 Features

A distinguishable point (keypoint) in an image together with a vector (descriptor) describing its surroundings.

SLAM and Visual Odometry must find the same points across images as the camera moves. Features provide a repeatable way to establish those correspondences.

### 9.3.1 Keypoint Detection

**Harris Corner**:
- Classical method for corner detection
- Slow, not robust to scale changes

**FAST (Features from Accelerated Segment Test)**:
- Very fast corner detection
- Suitable for real-time systems
- Not scale-invariant

**ORB (Oriented FAST and Rotated BRIEF)**:
- FAST detection + BRIEF descriptor + orientation
- Patent-free
- Widely used in real-time SLAM

The ORB-SLAM family uses ORB. It carries no patent restriction, and its speed suits real-time systems.

**SIFT (Scale-Invariant Feature Transform)**:
- Scale- and rotation-invariant
- High repeatability
- High computational cost (previously patented, now released)

Lowe published SIFT in 2004. Its scale- and rotation-invariant keypoints provide a useful reference for comparing later methods such as SURF and ORB.

**SuperPoint** (deep-learning-based):
- Self-supervised training
- High repeatability and accuracy
- Requires GPU

> **Further reading**
> - [Lowe, "Distinctive Image Features from Scale-Invariant Keypoints" (2004)](https://www.cs.ubc.ca/~lowe/papers/ijcv04.pdf) — The original SIFT paper.
> - [Rublee et al., "ORB: An efficient alternative to SIFT or SURF" (2011)](https://ieeexplore.ieee.org/document/6126544) — The original ORB paper.
> - [First Principles of CV — Feature Detection](https://www.youtube.com/playlist?list=PL2zRqk16wsdqXEMpHrc4Qnb5rA1Cylrhx) — Principles of keypoint detection, visually.
> - [DeTone et al., "SuperPoint: Self-Supervised Interest Point Detection and Description" (2018)](https://arxiv.org/abs/1712.07629) — The starting point of deep-learning-based features.
> - [Dark Programmer — Image keypoint extraction methods](https://darkpgmr.tistory.com/131) — Comparison of SIFT, HOG, Haar, Ferns, LBP, MCT, and other features.

### 9.3.2 Descriptor

Once you have a keypoint, the descriptor is about "how to describe" its surroundings. To find the same physical point across two images, you have to express the pattern around that point as numbers so they can be compared.

**BRIEF (Binary Robust Independent Elementary Features)**:
- Binary descriptor (0 or 1)
- Fast matching (Hamming distance)
- Not rotation-invariant

**ORB Descriptor**:
- BRIEF + orientation
- 256-bit binary vector

The advantage of binary descriptors is matching speed. Because the distance between two descriptors is computed as Hamming distance (an XOR operation), it is much faster than SIFT's Euclidean distance comparison. On embedded systems, this difference is large.

**SuperGlue** (deep-learning-based):
- Graph Neural Network-based matching
- Robust to repetitive patterns and low texture
- LightGlue: a lightweight version

> **Further reading**
> - [Sarlin et al., "SuperGlue: Learning Feature Matching with Graph Neural Networks" (2020)](https://arxiv.org/abs/1911.11763) — Representative work of deep-learning-based matching.
> - [OpenCV Feature Matching tutorial](https://docs.opencv.org/4.x/dc/dc3/tutorial_py_matcher.html) — How to use BFMatcher and FLANN.

### 9.3.3 Feature Matching

As the camera moves, SLAM must find the same points in the previous and current frames. Inspecting the feature matches helps distinguish tracking loss caused by too few correspondences from loss caused by incorrect ones.

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

Lowe's ratio test uses kNN to find the two nearest matches and keeps a match only when the ratio of the first distance to the second falls below a threshold. This removes ambiguous cases in which the two distances are similar. Lowe proposed 0.75 in the original paper; the threshold can be adjusted between 0.6 and 0.8 for the application.

> **Further reading**
> - [OpenCV Feature Matching](https://docs.opencv.org/4.x/dc/dc3/tutorial_py_matcher.html) — Examples of BFMatcher, FLANN, and ratio test.
> - [Computerphile — SIFT Features](https://www.youtube.com/watch?v=ram-jbLJjFg) — Intuitive explanation of feature matching.

> **Exercise**: [Feature Matching](https://alexjunholee.github.io/robotics-practice/app.html#feature_matching)
> Experiment interactively with feature matching between two images and the application of Lowe's ratio test.

---

## 9.4 Epipolar Geometry

Epipolar geometry deals with the geometric relation between two camera viewpoints.

Given two photos of the same object, the goal is to recover how the camera moved (relative pose) and from there reconstruct the 3D structure. This is the mathematical foundation of Visual Odometry and Structure from Motion (SfM). It is also where SVD and eigenvalue decomposition from linear algebra are directly used.

### 9.4.1 Essential Matrix (E)

**Definition**: encodes the relative pose between a pair of calibrated cameras

```
x2^T E x1 = 0
```

- x1, x2: normalized image coordinates
- E = [t]_× R (skew-symmetric matrix of t × R)

**5-point algorithm**: estimates E from at least 5 correspondence pairs (used with RANSAC)

Decomposing the essential matrix into R and t gives the relative rotation and translation between the two cameras. This is the core principle of Visual Odometry.

> **Exercise**: [Epipolar Geometry Visualization](https://alexjunholee.github.io/robotics-practice/app.html#epipolar)
> Examine interactively the epipolar lines and epipoles between two camera viewpoints, and understand the geometric meaning of the essential/fundamental matrix.

### 9.4.2 Fundamental Matrix (F)

**Definition**: the relation between a pair of uncalibrated cameras

```
p2^T F p1 = 0
```

- p1, p2: pixel coordinates
- F = K2^(-T) E K1^(-1)

**8-point algorithm**: estimates F from at least 8 correspondence pairs

For the relation between E and F: F is the version "you can use directly on pixel coordinates", while E is the version "you use when you already know the camera intrinsics". If you have calibrated, use E; if not, use F.

### 9.4.3 Triangulation

Triangulation computes the 3D position of a point observed from two viewpoints.

It is the same principle as how you perceive depth with two eyes. Observing the same point from two cameras (or one camera after it has moved) allows you to compute its 3D position geometrically.

```python
# OpenCV triangulation
points_4d = cv2.triangulatePoints(P1, P2, pts1, pts2)
points_3d = points_4d[:3] / points_4d[3]  # Homogeneous -> Cartesian
```

Caveat: if the baseline (distance between the two cameras) is too small, triangulation accuracy drops; if it is too large, it becomes hard to observe the same point from both sides at once. You have to understand this trade-off well.

> **Further reading**
> - [Stanford CS231A — Epipolar Geometry](https://web.stanford.edu/class/cs231a/) — Lecture material with clear mathematical derivations.
> - [Hartley & Zisserman, "Multiple View Geometry in Computer Vision"](https://www.robots.ox.ac.uk/~vgg/hzbook/) — The key reference on multi-view geometry. A must-read if you want to go deep.
> - [First Principles of CV — Stereo Vision](https://www.youtube.com/playlist?list=PL2zRqk16wsdoYzrWStQ2SQHXXS2K6ofd4) — Intuitive explanation of epipolar geometry.
> - [Dark Programmer — Image Geometry series (7 parts: coordinate frames to Epipolar)](https://darkpgmr.tistory.com/77) — A systematic Korean-language treatment of coordinate frames, homogeneous coordinates, 2D/3D transforms, homography, imaging, and epipolar geometry.

> **Exercise**: [Homography Visualization](https://alexjunholee.github.io/robotics-practice/app.html#homography)
> Manipulate the homography between planes interactively and see how four correspondence points determine the projective transform.

---

## 9.5 Optical Flow

Optical flow estimates pixel motion between consecutive frames.

As a robot sees the world through its camera while moving, knowing where each pixel goes in the next frame is useful. It is used directly in Visual Odometry pose estimation, dynamic object detection, collision avoidance, and so on. While feature matching only handles sparse points, dense optical flow estimates the motion of every pixel.

### 9.5.1 Lucas-Kanade Method

- Sparse optical flow (specific points only)
- Brightness constancy assumption
- Small-motion assumption

```python
# Compute optical flow
p1, status, err = cv2.calcOpticalFlowPyrLK(
    prev_gray, curr_gray, p0, None, **lk_params
)
```

In "PyrLK", "Pyr" stands for Pyramid. It uses an image pyramid to capture large motions too — a technique for overcoming the small-motion assumption of Lucas-Kanade.

### 9.5.2 Dense Optical Flow

- Computes motion for every pixel
- Farneback, RAFT (deep learning)

```python
# Farneback dense flow
flow = cv2.calcOpticalFlowFarneback(prev_gray, curr_gray, None, 0.5, 3, 15, 3, 5, 1.2, 0)
```

RAFT (Recurrent All-Pairs Field Transforms) is a common learned baseline for dense optical flow. Its paper reports strong benchmark accuracy, but latency and accuracy relative to newer methods depend on resolution, hardware, training data, and evaluation protocol.

> **Further reading**
> - [First Principles of CV — Optical Flow](https://www.youtube.com/playlist?list=PL2zRqk16wsdp8KbDfHKvPYNGF2L-zQASc) — Mathematical principles of optical flow.
> - [Teed & Deng, "RAFT: Recurrent All-Pairs Field Transforms for Optical Flow" (2020)](https://arxiv.org/abs/2003.12039) — Representative work of deep-learning-based optical flow.
> - [Huang et al., "FlowFormer: A Transformer Architecture for Optical Flow" (ECCV 2022, arXiv:2203.16194)](https://arxiv.org/abs/2203.16194) — Transformer-based optical flow.
> - [OpenCV Optical Flow tutorial](https://docs.opencv.org/4.x/d4/dee/tutorial_optical_flow.html) — Code examples for Lucas-Kanade and Farneback.

> **Exercise**: [Optical Flow Visualization](https://alexjunholee.github.io/robotics-practice/app.html#optical_flow)
> Compare the behavior of Lucas-Kanade and Dense Optical Flow algorithms interactively and observe the pixel-motion estimation process.

---

## 9.6 Advanced: PnP Problem

**Perspective-n-Point (PnP)** is the problem of estimating the camera pose (rotation R and translation t) given 3D points in space and their 2D correspondences in the image. In SLAM, per-frame camera tracking is precisely a PnP problem, and in AR, marker-based localization is also solved with PnP.

**Problem statement**: given n 3D-2D correspondences {(X_i, x_i)}, estimate the camera extrinsics [R|t].

$$x_i = K [R | t] X_i$$

Here K is the camera intrinsics.

**P3P (3-Point Problem)**:
- Solvable with at least 3 correspondences.
- 3 points yield up to 4 solutions; a 4th point is used for disambiguation.
- Combined with RANSAC, it can be solved robustly in the presence of outliers.

**EPnP (Efficient PnP)**:
- O(n) complexity, efficient when there are many correspondences.
- Represents the 3D points as 4 virtual control points and estimates those control points' camera coordinates.
- When there are many points, it is faster and more stable than P3P+RANSAC.

**Practical usage**:

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

# Basic PnP (iterative, no initial guess needed)
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

**Connection to SLAM**: the per-frame procedure in Visual SLAM is as follows.
1. From the previous frame, create 3D map points via triangulation.
2. In the new frame, predict the 2D reprojection of those map points.
3. Match them against the observed 2D keypoints.
4. Solve PnP on these 3D-2D correspondences to obtain the camera pose of the new frame.

The Tracking stage of ORB-SLAM3 is exactly this process.

> **Further reading**
> - [Lepetit et al., "EPnP: An Accurate O(n) Solution to the PnP Problem" (2009)](https://doi.org/10.1007/s11263-008-0152-6) — The original EPnP paper.
> - [OpenCV solvePnP documentation](https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html) — Explanation of the various PnP algorithm flags.
> - [Multiple View Geometry — Ch. 7](https://www.robots.ox.ac.uk/~vgg/hzbook/) — Mathematical background of the PnP problem.

**Practical tips for solvePnP**

OpenCV's `cv2.solvePnP()` estimates the camera pose from 3D-2D correspondences. The returned `rvec` is a Rodrigues vector (axis-angle representation).

```python
# rvec -> rotation matrix
R, _ = cv2.Rodrigues(rvec)

# Camera position in world coordinates
camera_position = -R.T @ tvec
```

Things to watch out for:
- The result of `solvePnP` is the **world-to-camera transform**. To get the camera's world position, you need to invert it.
- At least 4 points are required, but the more points, the more robust to noise. The RANSAC version `cv2.solvePnPRansac()` filters outliers automatically.
- The `flags` parameter selects the algorithm: `cv2.SOLVEPNP_ITERATIVE` (default, LM), `cv2.SOLVEPNP_P3P` (minimum 3 points), `cv2.SOLVEPNP_EPNP` (fast and stable, good when there are many points).

The direction of the Rodrigues vector is the rotation axis, and its magnitude (norm) is the rotation angle. This is exactly the axis-angle representation from the Lie algebra so(3) in Ch.3. `cv2.Rodrigues()` is an implementation of the exp/log map.

(See: [Dark Programmer — solvePnP usage and Rodrigues representation](https://darkpgmr.tistory.com/99))

---

## 9.7 Advanced: RANSAC Variants

RANSAC was introduced in the robust estimation section of Ch.3. In actual research, vanilla RANSAC is rarely used as is. There are several variants that improve convergence speed and accuracy, and which one you pick can change the result a lot.

**Main variants**:

| Method | Core idea | Characteristics |
|------|-------------|------|
| **Lo-RANSAC** | local optimization on inliers | converges in fewer iterations than vanilla |
| **PROSAC** | samples in order of matching confidence | tries good matches first, accelerating convergence |
| **MAGSAC++** | marginalizes over σ (inlier threshold) | no threshold tuning needed, adapts automatically |

**Lo-RANSAC (Locally Optimized RANSAC)**:
- When a good model is found, it re-estimates the model from that model's inliers (local optimization).
- A simple idea with a large effect. Particularly useful when the inlier ratio is low.

**PROSAC (Progressive Sample Consensus)**:
- Samples correspondences in order of matching score, highest first.
- When good matches are concentrated at the top, it finds a good model in the very first iterations.

**MAGSAC++ (Marginalizing Sample Consensus)**:
- Marginalizes the most troublesome hyperparameter, the inlier threshold σ.
- Instead of fixing the threshold, it integrates over multiple σ values, so manual tuning is almost unnecessary.
- OpenCV exposes it through the `USAC_MAGSAC` option.

**Using MAGSAC++ in OpenCV**:

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

**Practical tips**:
- Iteration count: controlled by the `confidence` parameter. 0.999 means "find the correct model with 99.9% probability". The lower the inlier ratio, the more iterations are needed, growing exponentially.
- Threshold: with MAGSAC++ you are less sensitive to the threshold, but you still need to provide an initial value. The values in the example are starting points and should be validated for the image resolution and noise level.
- The speed-accuracy relationship between PROSAC and MAGSAC++ depends on match-score quality, outlier ratio, and implementation. Compare them on the same data and time budget.

> **Further reading**
> - [Barath et al., "MAGSAC++, a Fast, Reliable and Accurate Robust Estimator" (2020)](https://arxiv.org/abs/1912.05909) — The original MAGSAC++ paper.
> - [OpenCV USAC documentation](https://docs.opencv.org/4.x/d1/df1/md__build_4rdparty_ippicv_ippicv_lnx_doc_USAC.html) — OpenCV's universal RANSAC framework.
> - [Chum & Matas, "Matching with PROSAC" (2005)](https://doi.org/10.1109/CVPR.2005.221) — The original PROSAC paper.

---

## 9.8 Advanced: Learning-Based Feature Matching

Hand-crafted features like ORB and SIFT have worked well for decades, but they fail on repetitive patterns, lack of texture, or extreme illumination changes. Since 2018, deep-learning-based feature extraction and matching have started to surpass classical methods.

**Pipeline evolution**:

```
SuperPoint (2018) -> SuperGlue (2020) -> LightGlue (2023)
  [keypoint detection+description]    [graph neural network matching]     [lightweight matching]
```

**SuperPoint**:
- Trains a keypoint detector and descriptor jointly via self-supervised learning.
- Homographic adaptation: applies synthetic transforms and inverts them to generate pseudo ground truth.
- In the paper's evaluation, it achieved higher repeatability than several classical baselines.

**SuperGlue**:
- Treats the keypoints of two images as a graph and matches them via an attention mechanism.
- Self-attention learns keypoint relations within the same image, and cross-attention performs matching between the two images.
- Solves the optimal assignment problem with the Sinkhorn algorithm.
- It achieved strong matching results in the paper's evaluation, while its compute cost and latency depend on feature count and hardware.

**LightGlue**:
- A lightweight version of SuperGlue. Adaptive early stopping pushes easy image pairs through quickly, while harder pairs go through more layers.
- In the paper's evaluation, adaptive depth and point pruning reduced latency relative to SuperGlue at comparable accuracy. The ratio depends on hardware and configuration.

**LoFTR (Detector-Free Local Feature Matching)**:
- Removes the keypoint detection step altogether. Performs dense matching across the whole image.
- Transformer-based, coarse-to-fine matching.
- Its biggest advantage is being able to match even in texture-poor regions.
- Downside: slow and uses a lot of GPU memory.

**Classical vs. learning-based comparison**:

| Item | ORB/SIFT | SuperPoint+LightGlue | LoFTR |
|------|----------|---------------------|-------|
| Speed (CPU) | fast | slow | very slow |
| Speed (GPU) | not applicable | moderate | slow |
| Texture-poor regions | fails | moderate | strong |
| Repetitive patterns | weak | strong | strong |
| GPU dependence | none | high | very high |
| Real-time robot use | easy | conditionally feasible | difficult |

**Code example — LightGlue (using kornia)**:

```python
import kornia
from kornia.feature import LightGlueMatcher, KeyNetAffNetHardNet

# Build the extractor and matcher
extractor = KeyNetAffNetHardNet(num_features=2048).eval()
matcher = LightGlueMatcher("keynetaffnethardnet").eval()

# Move to GPU
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
extractor = extractor.to(device)
matcher = matcher.to(device)

# Load images (kornia format: B x C x H x W, 0-1 range)
img0 = kornia.io.load_image(path0).unsqueeze(0).to(device)
img1 = kornia.io.load_image(path1).unsqueeze(0).to(device)

# Feature extraction
with torch.no_grad():
    feats0 = extractor(img0)
    feats1 = extractor(img1)

# Matching
dists, match_idxs = matcher(feats0["descriptors"], feats1["descriptors"])
```

SuperPoint+LightGlue is worth evaluating when a GPU is available and learned-matching robustness matters. When there is no GPU or the latency and power budgets are tight, a classical feature such as ORB is a simpler baseline. Make the final choice by measuring accuracy, latency, and memory on the target data.

> **Further reading**
> - [DeTone et al., "SuperPoint: Self-Supervised Interest Point Detection and Description" (2018)](https://arxiv.org/abs/1712.07629) — The original SuperPoint paper.
> - [Lindenberger et al., "LightGlue: Local Feature Matching at Light Speed" (2023)](https://arxiv.org/abs/2306.13643) — The original LightGlue paper.
> - [Sun et al., "LoFTR: Detector-Free Local Feature Matching with Transformers" (2021)](https://arxiv.org/abs/2104.00680) — The original LoFTR paper.

---

> **Technical Timeline: Computer Vision Fundamentals (Classical Methods)**
> - **~2004**: the era of classical features. Hand-crafted features like Harris Corner (1988) and SIFT (2004) dominate. Mathematically precise but computationally heavy.
> - **2006~2011**: lightweighting for real time. SURF (2006), FAST (2006), BRIEF (2010), ORB (2011) arrive. Patent and speed issues get resolved, and real-time SLAM becomes feasible.
> - **2015~2019**: deep learning seeps in. Learning-based features like SuperPoint (2018) and SuperGlue (2020) begin to surpass classical methods in performance.
> - **2020~**: fusion of geometry and learning. Detector-free matching such as LoFTR (2021) and lightweight learned matching such as LightGlue (2023) appear. Classical geometry remains central in the SLAM/VO back-end.
> - **Recent direction**: learned features and matchers are entering the front-end, while the back-end still uses epipolar geometry and triangulation. The two families therefore coexist in the same systems.
