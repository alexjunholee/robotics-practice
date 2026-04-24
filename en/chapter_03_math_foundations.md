# Ch.3 — Mathematical Foundations

Understanding Spatial AI properly requires a mathematical foundation. This chapter touches on the core concepts briefly; for deeper study, consult the recommended references.

The urge to skip the math is understandable. But when you read papers, you end up stuck at the equations. A SLAM paper says "optimization on SE(3)" and if you don't know what SE(3) is, you miss the paper's core idea; when a single line says "we derived the Jacobian and solved with Gauss-Newton," if that doesn't parse, the whole methodology is out of reach. The math here is not "math for a math exam" but "math to read and implement robotics papers." A third-year engineering student will have taken linear algebra, so the focus here is on connecting what you learned as an undergraduate to how it gets used in robotics.

Classical mathematical tools are still central, but Differentiable Programming and Auto-Differentiation are changing how we approach optimization problems. Jacobians used to be derived by hand; now auto-diff in PyTorch or JAX computes gradients for complex pipelines automatically. This is the background that made end-to-end learning-based SLAM and Differentiable Rendering (NeRF, 3D Gaussian Splatting) possible. To understand what auto-diff does internally, you still need the fundamentals covered here.

## 3.1 Linear Algebra

Linear algebra is the base tool across Spatial AI. Coordinate transformations, camera models, optimization, deep learning — all are expressed with matrices and vectors. "I took linear algebra as an undergrad" and "I can apply linear algebra to robotics" are different levels. This section covers the concepts used most in robotics.

### 3.1.1 Vectors and Matrices

**Vector**: a quantity with magnitude and direction.

```
v = [v_x, v_y, v_z]^T  (column vector)
```

In robotics, vectors represent points in 3D space, forces, velocities, and so on. "The robot is at (3, 2, 1) in the world frame" expresses a position as a vector.

**Matrix operations**:
- Addition/subtraction: element-wise
- Multiplication: row-by-column inner product
- Transpose: A^T
- Inverse: A^(-1), AA^(-1) = I

Coordinate transformation, rotation, and projection are all expressed as matrix multiplications. A camera projecting a 3D point to a 2D image, a robot transforming between coordinate frames — all of it is matrix multiplication.

> **Further reading**
> - [3Blue1Brown — Essence of Linear Algebra](https://www.youtube.com/playlist?list=PLZHQObOWTQDPD3MizzM2xVFitgF8hE_ab) — If you haven't watched this series, watch it. It shows visually what matrix multiplication means geometrically and why eigenvalues matter. It helps you understand linear algebra as "transformation" rather than "computation."
> - [Introduction to Applied Linear Algebra (Boyd & Vandenberghe) — free PDF](https://web.stanford.edu/~boyd/vmls/) — Applied linear algebra textbook by Stanford's Professor Boyd. Practical perspective, includes Python examples.
> - [Dark Programmer — Linear Algebra series (6 posts: basic formulas to PCA)](https://darkpgmr.tistory.com/103) — Summarizes key terms, inverse, eigenvalues, SVD, linear systems, and PCA in Korean.
> - [Dark Programmer — Vector and Matrix Calculus](https://darkpgmr.tistory.com/141) — Rules for vector/matrix differentiation. Foundation needed for Jacobian computation.

### 3.1.2 Eigenvalue Decomposition

```
Av = λv
```

- v: eigenvector
- λ: eigenvalue

**Uses**: PCA, covariance matrix analysis, stability analysis.

You'll use this the moment you handle a point cloud. When PCA (Principal Component Analysis) finds the principal axes of a point cloud, the eigenvectors of the covariance matrix are the principal axis directions and the eigenvalues are the variances along them. Deciding whether "this point cloud is a plane or a line" also comes from the ratio of eigenvalues. Normal vector estimation uses the eigenvector corresponding to the smallest eigenvalue.

> **Further reading**
> - [3Blue1Brown — Eigenvectors and Eigenvalues](https://www.youtube.com/watch?v=PFDu9oVAE-g) — Intuitive explanation of the geometric meaning of eigenvalues.
> - [MIT 18.06 Linear Algebra — Gilbert Strang (YouTube)](https://www.youtube.com/playlist?list=PLE7DDD91010BC51F8) — A widely known linear algebra lecture series. Covers all of linear algebra in depth, including eigenvalue decomposition.

> **Exercise**: [PCA 3D · Dimensionality Reduction](https://alexjunholee.github.io/robotics-practice/app.html#pca_3d)
> Manipulate the process by which the eigenvectors of the covariance matrix become the principal axes of a 3D distribution, and simultaneously visualize dimensionality reduction onto the PC1·PC2 plane (3D→2D) and the PC1 axis (2D→1D).

### 3.1.3 Singular Value Decomposition (SVD)

```
A = UΣV^T
```

- U: left singular vectors (m×m orthogonal matrix)
- Σ: diagonal matrix of singular values (m×n)
- V: right singular vectors (n×n orthogonal matrix)

**Uses**: least squares solutions, matrix approximation, fundamental matrix computation.

SVD shows up constantly in robotics. It is the most numerically stable way to compute the least-squares solution of an overdetermined system. Camera calibration for the fundamental matrix, point cloud registration for the optimal transformation — all use SVD. The final step of the "8-point algorithm," which computes the fundamental matrix from eight or more correspondences, is SVD.

> **Further reading**
> - [Steve Brunton — Singular Value Decomposition (YouTube)](https://www.youtube.com/watch?v=nbBvuuNVfco) — Lectures by a University of Washington professor that clearly explain the mathematical meaning of SVD and its applications.
> - [Linear Algebra and Its Applications (Gilbert Strang)](https://math.mit.edu/~gs/linearalgebra/ila6/indexila6.html) — Standard linear algebra textbook. The SVD chapter is particularly well written.

## 3.2 3D Geometry

3D geometry is at the heart of Spatial AI. It expresses mathematically "where is the robot in 3D space, where is the camera looking, and where is that object?" Without this part, the first page of a SLAM paper is already a wall.

### 3.2.1 Coordinate Frames

In Spatial AI you move between multiple coordinate frames. The **World Frame (W)** is the globally fixed frame, the **Camera Frame (C)** is centered on the camera, the **Body Frame (B)** is centered on the robot, and the **IMU Frame (I)** is the IMU sensor's frame.

Once you build a robot system yourself, this becomes tangible. A single piece of data has to pass through several frames before it is meaningful. "The location of the object the camera sees" is expressed in the camera frame, but for the robot to approach the object, that position must be transformed into the robot or world frame. Each sensor has its own frame, and sensor fusion is possible only when you know the transformation between them accurately (extrinsic calibration).

**Coordinate transformation**:

```
p_W = T_WC × p_C
```

T_WC: Camera → World transformation matrix (4×4).

> **Further reading**
> - [State Estimation for Robotics, Ch.6 — Coordinate Frames (Tim Barfoot) — free PDF](http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) — The best robotics-oriented treatment of coordinate frame transformations.
> - [Stanford CS231A — Camera Models](https://web.stanford.edu/class/cs231a/) — The part of Stanford's CV course that covers camera frames and projection models.

### 3.2.2 Rotation Representations

Multiple rotation representations exist because each has different strengths and weaknesses. In SLAM optimization, the choice of representation affects convergence speed and stability. Without this, you can't tell "why does this code use quaternions while that code uses a rotation matrix?"

**Rotation Matrix R** is a 3×3 orthogonal matrix (det(R) = 1, R^T = R^(-1)) with 9 parameters and 6 constraints, giving 3 actual degrees of freedom.

**Euler Angles** express rotation with three angles: Roll (φ), Pitch (θ), Yaw (ψ). Intuitive, but has the **Gimbal Lock** problem, and results depend on the application order (ZYX, XYZ, etc.).

**Quaternion q = [w, x, y, z]** (||q|| = 1) expresses 3 DoF with 4 parameters. It has no Gimbal Lock and supports smooth interpolation (Slerp), so it is the most widely used.

**Axis-Angle** combines a rotation axis n and angle θ into a 3-parameter representation. It converts to a rotation matrix via Rodrigues' formula.

Practical tips: ROS uses quaternions as the default rotation representation, OpenCV mostly uses Rodrigues vectors (axis-angle), and optimization libraries (Ceres, GTSAM) often use Lie group-based representations (so(3) → SO(3) mapping). You need to convert between them freely.

> **Further reading**
> - [3Blue1Brown — Quaternions and 3D Rotation](https://www.youtube.com/watch?v=zjMuIxRvygQ) — Visualizes the geometric meaning of quaternions. An intuitive answer to why four dimensions are needed for 3D rotation.
> - [State Estimation for Robotics, Ch.7 — Rotation (Tim Barfoot)](http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) — Clean treatment of every rotation representation and the conversions between them.
> - [Sola — Quaternion Kinematics for the Error-State Kalman Filter (Tech Report)](https://arxiv.org/abs/1711.02508) — Mathematical foundations of quaternion-based error-state Kalman filtering for VIO/INS implementations. A very practical technical report.
> - [3D Rotation Converter](https://www.andre-gaschler.com/rotationconverter/) — Online tool for checking conversions between quaternions, Euler angles, and rotation matrices.

> **Exercise**: [Rotation Representations and Gimbal Lock](https://alexjunholee.github.io/robotics-practice/app.html#rotation_gimbal) | [6DoF Pose Visualization](https://alexjunholee.github.io/robotics-practice/app.html#xyzrpy_6dof)
> Manipulate and compare Euler-angle Gimbal Lock and quaternion rotation directly, and interactively explore a 6-DoF pose (x, y, z, roll, pitch, yaw).

### 3.2.3 Homogeneous Coordinates

Extend a 3D point to 4D so that transformations become a single matrix:

```
[X, Y, Z, 1]^T  (3D point)

T = | R   t |   (4×4 transformation matrix)
    | 0   1 |
```

Why use homogeneous coordinates: rotation and translation can be expressed as one matrix multiplication. In ordinary coordinates p' = Rp + t (multiplication + addition), but in homogeneous coordinates it becomes p' = Tp (multiplication only). When chaining multiple transformations you just multiply the matrices, which is convenient for things like the chain of joint transformations in a robot arm.

### 3.2.4 SE(3) and SO(3)

**SE(3)** (Special Euclidean Group) is the set of all 3D rigid-body transformations (rotation + translation) with 6 DoF. **SO(3)** (Special Orthogonal Group) is the set of rotations alone with 3 DoF.

SE(3) and SO(3) are **Lie groups**. When optimizing, you need to "update while satisfying the rotation matrix constraints (orthogonality, determinant 1)," and Lie group theory solves this elegantly. You optimize without constraints on the corresponding **Lie algebra** (se(3), so(3)) and then map back to the Lie group via the exponential map. This concept is central to pose graph optimization in SLAM.

> **Further reading**
> - [State Estimation for Robotics, Ch.7 (Tim Barfoot) — free PDF](http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) — The best robotics-oriented treatment of SE(3), SO(3), and Lie groups/algebras. Essential reading if you dig into this area.
> - [Sola — A Micro Lie Theory for State Estimation in Robotics (arXiv:1812.01537)](https://arxiv.org/abs/1812.01537) — A paper that summarizes Lie group theory just as far as state estimation in robotics requires. Very practical.

## 3.3 Probability & Statistics

Sensor data always has noise, and the robot's state always has uncertainty. Probability and statistics express and manipulate that uncertainty mathematically. "The sensor value is exactly 3.0 m" is not meaningful; "3.0 m ± 0.05 m (95% confidence interval)" is. Propagating and updating this uncertainty is the basis of state estimation.

### 3.3.1 Gaussian Distribution

```
p(x) = (1 / √(2πσ²)) × exp(-(x-μ)²/(2σ²))
```

**Multivariate Gaussian**:

```
p(x) = N(μ, Σ)
```

- μ: mean vector
- Σ: covariance matrix

Widely used for modeling sensor noise and position uncertainty.

The Gaussian is used this heavily because of mathematical convenience. Thanks to the central limit theorem, many natural phenomena follow a Gaussian, and operations between Gaussians (product, sum) are closed, which makes analysis easy. The Kalman filter assumes a Gaussian for the same reason.

> **Further reading**
> - [3Blue1Brown — But what is the Central Limit Theorem?](https://www.youtube.com/watch?v=zeJD6dqJ5lo) — Visual explanation of the central limit theorem. An intuitive answer to why the Gaussian appears everywhere.
> - [Kalman Filter — How it works, in pictures](http://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/) — Visual explanation of how the Kalman filter works. Good for building intuition before the equations.

> **Exercise**: [Kalman Filter](https://alexjunholee.github.io/robotics-practice/app.html#kalman_filter)
> Interactively manipulate the Kalman filter's predict-update cycle and observe Gaussian-based state estimation in action.

**Mahalanobis Distance**

Euclidean distance treats every direction equally. But sensor data has different uncertainty in different directions. GPS, for example, has much larger error vertically (tens of meters) than horizontally (a few meters).

Mahalanobis distance is distance that accounts for covariance:

```
d_M = sqrt((x - μ)^T Σ^{-1} (x - μ))
```

If Σ is the identity, it reduces to Euclidean distance. If Σ is diagonal, it is a per-axis scaled distance. For a general Σ, distance is redefined along the principal axes of the covariance.

Use in SLAM: for data association, Mahalanobis distance decides "did this observation come from this landmark?" Something close in Euclidean but far in Mahalanobis (not aligned with the uncertainty direction) is likely a wrong association.

(See: [Dark Programmer — Mean, Standard Deviation, Variance, and Mahalanobis Distance](https://darkpgmr.tistory.com/41))

### 3.3.2 Bayes' Rule

```
P(A|B) = P(B|A) × P(A) / P(B)
```

Bayes' rule is the mathematical basis of state estimation. It is the formula answering "given the sensor measurement, what is the robot's actual state?" Without it, you cannot understand Kalman filters, particle filters, or factor graph-based SLAM.

**Recursive state estimation**:

```
P(x_t | z_{1:t}) ∝ P(z_t | x_t) × P(x_t | z_{1:t-1})
```

- P(z_t | x_t): measurement model — "given the robot is at this pose, what is the probability the sensor outputs this value?"
- P(x_t | z_{1:t-1}): prior — "based on all prior information, what is the probability the robot is here?"

Once you see this recursive structure, the Kalman filter follows immediately. Every time new sensor data arrives, update the existing belief (prior) to get a more accurate estimate (posterior). The Kalman filter's predict-update cycle is exactly this structure.

> **Further reading**
> - [3Blue1Brown — Bayes' Theorem](https://www.youtube.com/watch?v=HZGCoVF3YvM) — A good visual introduction to Bayes' theorem.
> - [Probabilistic Robotics (Thrun, Burgard, Fox)](https://www.probabilistic-robotics.org/) — The essential textbook on probabilistic robotics. Organizes Bayes filters, Kalman filters, particle filters, and SLAM cleanly from a probabilistic viewpoint.
> - [Giseop Kim's blog — Bayesian Filtering series (2 posts)](https://gisbi-kim.github.io/blog/2021/03/09/bayesfiltering-1.html) — Korean-language walkthrough of Bayes filtering. A foundation that leads into the Kalman filter.

### 3.3.3 MLE and MAP

**MLE (Maximum Likelihood Estimation)**:

```
x* = argmax P(z | x)
```

The parameter most likely given the data.

**MAP (Maximum A Posteriori)**:

```
x* = argmax P(x | z) = argmax P(z | x) × P(x)
```

An estimate that takes the prior into account.

The difference in SLAM is between "find the optimal position from observations alone (MLE)" and "find the optimal position using prior position information too (MAP)." Real SLAM systems mostly use MAP. A prior makes estimation stable even with noisy observations. Mathematically, taking the log of MAP turns it into "sum of squared observation errors + regularization term," which is the same form as regularized least squares from an optimization standpoint.

> **Further reading**
> - [Probabilistic Robotics, Ch.2 — Recursive State Estimation (Thrun)](https://www.probabilistic-robotics.org/) — Explains the relationships between MLE, MAP, and Bayes filtering in a robotics context.
> - [Cyrill Stachniss — Maximum Likelihood and MAP Estimation](https://www.youtube.com/watch?v=XepXtl9YKwc) — A clear, example-driven explanation of the difference between MLE and MAP.

**Intuitive difference between MLE and MAP**

Both share the goal of "find the most plausible parameter," but their approaches differ.

MLE (Maximum Likelihood) asks "which parameter maximizes the probability of observing this data?" It looks only at the data. MAP (Maximum A Posteriori) adds a prior: "given the data and combining prior knowledge, which value maximizes the posterior probability of the parameter?"

In formulas: MAP = MLE + prior. With a Gaussian prior, MAP equals MLE with L2 regularization added. Weight decay in deep learning can be seen as an implementation of MAP.

In SLAM: multiply the likelihood of odometry measurements by the likelihood of sensor observations, combine with the prior from the previous state, and do MAP estimation. Each factor in a factor graph corresponds to one of these likelihoods or priors.

(See: [Dark Programmer — Bayes' Rule, ML and MAP, and Image Processing](https://darkpgmr.tistory.com/62))

## 3.4 Optimization Basics

Optimization is the final stage of Spatial AI algorithms. SLAM bundle adjustment, camera calibration, and deep-learning training are all optimization problems. Without this section you can run the code but cannot debug why it fails to converge or why the result is wrong.

### 3.4.1 Least Squares

```
x* = argmin ||Ax - b||²
```

**Normal equation**:

```
x* = (A^T A)^(-1) A^T b
```

Least squares is the most basic way to "find the best-fitting model parameters from noisy measurements." From line fitting to camera calibration, it is the starting point of almost every estimation problem.

> **Further reading**
> - [Cyrill Stachniss — Least Squares for Robotics](https://www.youtube.com/watch?v=r2cyMQ5NB1o) — Concrete explanation of applying least squares to robotics problems.
> - [Giseop Kim's blog — SLAM back-end series (3 posts)](https://gisbi-kim.github.io/blog/2021/03/04/slambackend-1.html) — An introduction to the back-end that starts from "SLAM is solving Ax=b." Three-part series leading up to factor graphs.
> - [Giseop Kim's blog — Iterative Optimization, Part 1](https://gisbi-kim.github.io/blog/2021/03/16/leastsquare-1.html) — An intuitive Korean-language walkthrough of nonlinear optimization.

**Intuition for least squares**

"Why minimize the square of the error?" The reason for square rather than absolute value is twofold: it is differentiable, and it penalizes large errors more. As a bonus, under a Gaussian noise assumption it gives the same solution as Maximum Likelihood Estimation.

In an over-determined system (more equations than unknowns), no x exactly satisfies Ax = b. Instead, find the x that minimizes ||Ax - b||², which gives the normal equation `A^T A x = A^T b`. That is all there is to least squares.

Caveat: if `A^T A` is singular (rank deficient), there is no unique solution. In that case, instead of the pseudo-inverse `x = (A^T A)^{-1} A^T b`, use SVD for numerical stability.

(See: [Dark Programmer — Understanding Least Squares and Various Uses](https://darkpgmr.tistory.com/56))

### 3.4.2 Gradient Descent

```
x_{k+1} = x_k - α × ∇f(x_k)
```

- α: learning rate
- ∇f: gradient

Gradient descent is used daily in deep learning, but it is also the baseline for optimization in robotics. It is the intuitive method of stepping opposite to the gradient to find a minimum. The limits are that tuning the learning rate is difficult, it can get stuck in local minima, and convergence is slow, so robotics typically uses more efficient methods (Gauss-Newton, LM).

**Relationships between gradient, Jacobian, and Hessian**

Three easily confused concepts, laid out:

The **gradient** ∇f is the first derivative of a scalar function f and outputs an n×1 vector. It tells you "in which direction does f increase fastest?" The **Jacobian** J extends this to a vector function f: R^n → R^m as an m×n matrix. It holds the partial derivatives of each output with respect to each input. The **Hessian** H is the second derivative of a scalar function f, an n×n symmetric matrix that carries curvature information and is used in Newton's method.

Relationships:
```
For cost function C(x) = ||r(x)||²:
  Gradient:  ∇C = J^T r           (J is the Jacobian of r)
  Hessian:   H ≈ J^T J            (Gauss-Newton approximation: drop the second-derivative term)
  Update:    δx = -(J^T J)^{-1} J^T r
```

Why Gauss-Newton uses J^T J as the Hessian approximation: the exact Hessian is expensive to compute, and in regions where the residual r is small, the second-order term is negligible.

(See: [Dark Programmer — Gradient, Jacobian, Hessian, Laplacian](https://darkpgmr.tistory.com/132))

### 3.4.3 Gauss-Newton

Solves nonlinear least squares problems by iteratively linearizing:

```
(J^T J) Δx = -J^T r
x_{k+1} = x_k + Δx
```

- J: Jacobian matrix
- r: residual

Why Gauss-Newton is preferred over gradient descent in robotics: it uses second-order information (J^T J as a Hessian approximation) and converges much faster. When SLAM optimizes thousands to tens of thousands of variables, gradient descent takes far too long to converge, whereas Gauss-Newton can converge in a handful of iterations.

### 3.4.4 Levenberg-Marquardt (LM)

A hybrid of Gauss-Newton and gradient descent:

```
(J^T J + λI) Δx = -J^T r
```

- λ: damping factor
- small λ → Gauss-Newton (fast convergence)
- large λ → gradient descent (stable)

The core algorithm for bundle adjustment and pose graph optimization in SLAM.

Why LM dominates in practice: Gauss-Newton converges very quickly with a good initial value but can diverge with a bad one. LM adjusts λ automatically, starting safely like gradient descent early on and converging quickly like Gauss-Newton once near the solution. Ceres Solver, g2o, and GTSAM all adopt it as the default algorithm for robotics optimization libraries.

> **Further reading**
> - [Cyrill Stachniss — Gauss-Newton and Levenberg-Marquardt for SLAM](https://www.youtube.com/watch?v=hRyL5KwFLAE) — Step-by-step explanation of how Gauss-Newton and LM are used in SLAM.
> - [State Estimation for Robotics, Ch.4 — Nonlinear Optimization (Tim Barfoot) — free PDF](http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) — Solid treatment of nonlinear optimization from a robotics state estimation perspective.
> - [Ceres Solver Tutorial](http://ceres-solver.org/tutorial.html) — Google's nonlinear least squares optimization library. Hands-on practice with how to use the LM algorithm in code.
> - [Dark Programmer — Intuitive Understanding of Optimization Methods](https://darkpgmr.tistory.com/149) — Geometric intuition for gradient descent, Newton, LM, and others.
> - [Giseop Kim's blog — Five recommended resources for SLAM back-end study](https://gisbi-kim.github.io/blog/2021/10/03/slam-textbooks.html) — A curated list covering error-state KF, factor graphs, bundle adjustment, and more.
> - [Derivative Calculator](https://www.derivative-calculator.net/) — Online tool that shows step-by-step symbolic differentiation. Useful for checking Jacobian derivations.

**Intuition for LM: switching between Gauss-Newton and gradient descent**

In nonlinear least squares, Gauss-Newton converges fast but can diverge with a bad initial value. Gradient descent is slow but stable. LM switches between them automatically via the damping factor λ.

Small λ is close to Gauss-Newton and converges fast near the solution; large λ is close to gradient descent and is stable in the early stages far from the solution. If an update reduces the cost, decrease λ; if it increases, increase λ. This adaptive switching is the core of LM, and it is why the default solver in Ceres Solver is LM.

(See: [Dark Programmer — Summary of Function Optimization Methods (LM, etc.)](https://darkpgmr.tistory.com/142))

## 3.5 Advanced: Lie Group and Lie Algebra

*If you want to become a researcher, read from here.*

One of the most frequent mathematical hurdles in robotics is "how do you optimize rotations?" Lie groups and Lie algebras provide a systematic framework for handling rotations and rigid-body transformations. They are essential for understanding SLAM back-ends, visual-inertial odometry, and bundle adjustment.

### 3.5.1 Why We Need Lie Groups

Section 3.2 covered several ways to represent rotations. Problems arise when you try to optimize with them.

- **Rotation matrix R**: 3x3 with 9 parameters but only 3 actual degrees of freedom, because of the constraints R^T R = I and det(R) = 1. Applying ordinary unconstrained optimization means that after an update, R is no longer a valid rotation matrix.
- **Quaternion**: 4 parameters with a normalization constraint (||q|| = 1). You must re-normalize on every update, and numerical errors accumulate in the process.
- **Euler angles**: have the gimbal lock problem, and angle wrapping is tricky.

The core issue: rotations live on a nonlinear manifold, but the optimization algorithms we know (Gauss-Newton, LM) work in Euclidean space. Lie group theory bridges this gap. It defines a tangent space (the Lie algebra) at a point (a rotation matrix) on the manifold, runs Euclidean optimization in that tangent space, and lifts the result back onto the manifold.

Background: a "group" here is a set equipped with an operation satisfying closure, associativity, identity, and inverse. For example, the set of invertible n x n matrices forms a group under matrix multiplication, called the general linear group GL(n). Its subgroup with det = 1 is the special linear group SL(n). The orthogonal group O(n) is the set of matrices preserving the inner product; collecting only those with det = 1 gives SO(n) — that is, the rotation group. Those with det = -1 include reflections, and because they are not closed under the group operation, they do not form a subgroup.

### 3.5.2 SO(3): The 3D Rotation Group

**Definition:**
```
SO(3) = { R in R^{3x3} | R^T R = I, det(R) = 1 }
```

SO(3) is a group. The group operation is matrix multiplication, and the composition R_1 R_2 of two rotations R_1, R_2 is again in SO(3). The identity is the identity matrix I, and the inverse is R^T (= R^{-1}). Because it is orthogonal, the transpose equals the inverse. Matrix multiplication is associative but not commutative (in general R_1 R_2 != R_2 R_1).

**Lie algebra so(3):**

The Lie algebra of SO(3) is the space of 3x3 skew-symmetric matrices, which is 3-dimensional.

The **hat operator** `[.]x` converts a 3D vector into a skew-symmetric matrix:

```
w = [w1, w2, w3]^T  (in R^3)

        [  0   -w3   w2 ]
[w]x =  [  w3   0   -w1 ]   in so(3)
        [ -w2   w1   0  ]
```

This matrix corresponds to the cross product of vectors: `[w]x v = w x v`.

The **vee operator** `(.)v` is the inverse: it extracts a 3D vector from a skew-symmetric matrix.

Intuitively, an element w of so(3) encodes "rotation axis direction" and "rotation magnitude" as a single vector. It corresponds directly to the axis-angle representation.

### 3.5.3 Exponential Map and Logarithmic Map

**Exponential map**: so(3) -> SO(3)

The map that sends an element (vector) of the Lie algebra to an element (rotation matrix) of the Lie group. Let's derive where it comes from.

Suppose we have a matrix R(t) that rotates continuously over time (R(0) = I). R(t) is always in SO(3), so `R(t) R(t)^T = I`. Differentiating both sides with respect to t:

```
d/dt (R R^T) = R_dot R^T + R R_dot^T = 0
→  R_dot R^T = -(R R_dot^T)^T
```

So `R_dot R^T` is skew-symmetric. It can be written as the hat form of some vector w(t):

```
R_dot(t) R^T(t) = [w(t)]x
→  R_dot(t) = [w(t)]x R(t)
```

When w is constant (constant angular velocity), the solution to this differential equation is:

```
R(t) = exp([w]x * t) = sum_{n=0}^{inf} ([w]x * t)^n / n!
```

Here `exp([w]x)` is the matrix that rotates by ||w|| radians around the axis w. Concretely, setting theta = ||w|| yields the closed form via **Rodrigues' formula**:

```
exp([w]x) = I + (sin(theta) / theta) [w]x + ((1 - cos(theta)) / theta^2) [w]x^2
```

This formula is derived by substituting the Taylor series of `sin(t)` and `cos(t)` into powers of `[w]x`. Using the identity `[w]x^3 = -theta^2 [w]x`, the series collapses into sin and cos terms.

When theta is small (|theta| < eps), sin(theta)/theta ≈ 1 and (1-cos(theta))/theta^2 ≈ 1/2, so:

```
exp([w]x) ≈ I + [w]x + (1/2)[w]x^2   (first-order approximation)
```

Caveat: for a given rotation matrix R, the w satisfying `R = exp([w]x)` is not unique. ||w|| + 2*pi*k (integer k) give the same R. This is the subtle point in the logarithmic map.

**Logarithmic map**: SO(3) -> so(3)

The inverse. Recover the axis-angle vector w from a given rotation matrix R.

```
theta = arccos((tr(R) - 1) / 2)
[w]x = (theta / (2 sin(theta))) (R - R^T)
```

Special handling is needed near theta = 0 (identity rotation) or theta = pi (180-degree rotation).

**Intuition**: the Lie algebra is the tangent space at a point on the group (usually the identity I). "Small rotations" can be expressed as vectors in the tangent space, and the exponential map sends such a vector to an actual rotation on the manifold. This is why it is central to optimization: compute the update dw in the tangent space (R^3), then multiply exp([dw]x) onto the current rotation to move along the manifold.

### 3.5.4 SE(3): The 3D Rigid-Body Transformation Group

A robot's pose includes not only rotation but also translation. SE(3) handles this.

**Definition:**
```
SE(3) = { T = [ R  t ] | R in SO(3), t in R^3 }
              [ 0  1 ]
```

T is a 4x4 homogeneous transformation matrix. SE(3) is also a group. The group operation is matrix multiplication T_1 T_2, the identity is the 4x4 identity matrix, and the inverse is T^{-1} = [ R^T  -R^T t ; 0  1 ].

**Lie algebra se(3):**

The Lie algebra of SE(3) is 6-dimensional. Its elements are called **twist** vectors:

```
xi = [rho; w] in R^6     (rho in R^3: translation part, w in R^3: rotation part)
```

The **hat operator** converts a 6D vector into a 4x4 matrix:

```
        [ [w]x  rho ]
xi^ =   [  0     0  ]   in se(3)    (4x4 matrix)
```

**Exponential map**: se(3) -> SE(3)

```
exp(xi^) = [ exp([w]x)   J rho ]   in SE(3)
           [    0           1   ]
```

Here J is the left Jacobian of SO(3):

```
J = I + ((1 - cos(theta)) / theta^2) [w]x + ((theta - sin(theta)) / theta^3) [w]x^2
```

**Key point**: a 6-DoF pose (3 rotation + 3 translation) can be parameterized by a 6D vector xi in R^6. Run optimization in unconstrained 6D Euclidean space and lift the result onto the SE(3) manifold via the exponential map. This is why Lie groups are used in SLAM optimization.

> **Exercise**: [SE(3) Pose Composition](https://alexjunholee.github.io/robotics-practice/app.html#pose_composition_3d)
> Manipulate the composition of SE(3) transformations in 3D and see how combined rotation-and-translation rigid-body transformations chain together.

### 3.5.5 Perturbation Model and Jacobian

When optimizing a pose with Gauss-Newton or LM, there are two ways to apply a small perturbation d_xi to the current estimate T.

**Left perturbation (global frame):**
```
T' = exp(d_xi^) * T
```

**Right perturbation (body frame):**
```
T' = T * exp(d_xi^)
```

Either works, as long as you stay mathematically consistent. Conventions differ across the literature, so pay attention. Barfoot's textbook uses left predominantly, and Strasdat's Sophus uses right by default.

**Jacobian computation:**

For an error function e(T), the Jacobian with respect to the perturbation is:

```
de/d(d_xi) = lim_{d_xi->0}  (e(exp(d_xi^) * T) - e(T)) / d_xi     (for left perturbation)
```

This Jacobian is a 6-column matrix (error dimension x 6).

**Why this matters:**

In ordinary optimization the update is `x <- x + dx` (Euclidean addition). But on SE(3), addition is not defined. Instead:

1. Compute d_xi in R^6 (the tangent space) via Gauss-Newton: `d_xi = -(J^T J)^{-1} J^T e`.
2. Update on the manifold: `T <- exp(d_xi^) * T`.

This guarantees that T remains a valid SE(3) element after the update. No separate constraint handling is needed.

**Practical note**: g2o, GTSAM, and Ceres (with local parameterization / manifold) use this approach internally. GTSAM's `Pose3` implements SE(3) directly and provides `Pose3::Expmap()` and `Pose3::Logmap()`. In Ceres, the same concept is implemented through `LocalParameterization` (or `Manifold` in the newer API).

### 3.5.6 Adjoint Representation

Use the adjoint when you need to transform a twist into a different coordinate frame.

For an element T of SE(3), the adjoint matrix Ad_T is a 6x6 matrix:

```
Ad_T = [ R    [t]x R ]   in R^{6x6}
       [ 0      R    ]
```

Twist transformation:
```
xi_a = Ad_{T_ab} * xi_b
```

**Practical meaning**: the velocity a sensor (e.g., an IMU) measures (angular velocity, linear velocity) is expressed in the sensor frame. Use the adjoint to convert it to the body or world frame. In a VIO system fusing multiple sensors, coordinate frame conversions happen frequently, so you need to understand what the adjoint means.

### 3.5.7 Use in Practice

**Sophus (C++)**: A Lie group library written by Strasdat. It implements SO(3), SE(3), their exponential/logarithmic maps, the adjoint, and more. Major SLAM systems like ORB-SLAM3 and Kimera use it.

```cpp
#include <sophus/se3.hpp>

// Initialize an SE(3) pose (identity transformation)
Sophus::SE3d T_world_body;

// se(3) perturbation (6-vector): [translation; rotation]
Sophus::SE3d::Tangent delta;
delta << 0.01, 0.0, 0.0, 0.0, 0.0, 0.001;  // small x-translation + small z-rotation

// Left perturbation update
T_world_body = Sophus::SE3d::exp(delta) * T_world_body;

// Log map: SE(3) -> se(3)
Sophus::SE3d::Tangent xi = T_world_body.log();
```

**Jaxlie (Python/JAX)**: A JAX-based Lie group library by Brent Yi. Because automatic differentiation works, you don't need to derive Jacobians by hand. Useful for research prototyping.

```python
import jaxlie
import jax.numpy as jnp

T = jaxlie.SE3.identity()
delta = jnp.array([0.01, 0.0, 0.0, 0.0, 0.0, 0.001])
T_updated = jaxlie.SE3.exp(delta) @ T
```

**GTSAM**: `gtsam::Pose3` uses SE(3) internally. It automatically handles perturbations on the Lie group during factor graph optimization.

> **Further reading**
> - [State Estimation for Robotics, Ch.7-8 (Barfoot)](http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) — The key reference for Lie groups in robotics state estimation.
> - [A micro Lie theory for state estimation in robotics (Sola et al., arXiv:1812.01537)](https://arxiv.org/abs/1812.01537) — Summarizes the essentials of Lie groups in 20 pages. Read this before the papers.
> - [TUM Multiple View Geometry, Ch.2 -- Rigid Body Motion](https://cvg.cit.tum.de/teaching/online/mvg) — Lectures by Professor Daniel Cremers. Visual explanation of SO(3) and SE(3).
> - [Sophus GitHub](https://github.com/strasdat/Sophus) — C++ Lie group library. Reading the code accelerates understanding.
> - [Jinyong Jeong's blog — SE(3) and SO(3) transformation](https://jinyongjeong.github.io/2016/06/07/se3_so3_transformation/) — Korean-language summary of SE(3) and SO(3) transformations. Explains systematically starting from GL(3) and O(3).
> - [T-Robotics: Lie Group Formulation for Robot Mechanics](http://t-robotics.blogspot.com/2015/07/lie-group-formulation-for-robot.html) — Korean-language explanation of Lie groups. Summarizes the use of Lie groups in robot dynamics.

## 3.6 Advanced: Factor Graph

*If you want to become a researcher, read from here.*

Factor graph is the framework for systematically defining and efficiently solving SLAM problems. The back-end of modern SLAM systems is almost without exception based on factor graphs.

### 3.6.1 What Is a Factor Graph

A factor graph is a bipartite graph made of two kinds of nodes:

- **Variable nodes**: the states to be estimated. Robot poses (x_1, x_2, ...), landmark positions (l_1, l_2, ...), etc.
- **Factor nodes**: constraints or measurements among variables. Each factor defines a cost function over the variables it connects.

Probabilistically, the full posterior decomposes as a product of factors:

```
p(X | Z) proportional to  prod_i  f_i(X_i)
```

Here X_i is the subset of variables connected to factor f_i.

**MAP estimation** = maximize the product of all factors = take the negative log to minimize the sum = a **nonlinear least squares** problem:

```
X* = argmin_X  sum_i  ||e_i(X_i)||^2_{Sigma_i}
```

e_i is the error function and Sigma_i is the covariance (uncertainty weighting) of the measurement.

### 3.6.2 Expressing SLAM as a Factor Graph

Factor types commonly used in SLAM:

| Factor | Role |
|---|---|
| Prior factor | Prior information about the initial pose. Example: "the start point is the origin" |
| Odometry factor | Relative transformation between two consecutive poses. Comes from IMU preintegration or wheel odometry |
| Landmark observation factor | Measurement of a landmark observed from a pose. Reprojection error is the classic example |
| Loop closure factor | Added when a previously visited place is recognized again. Core mechanism for correcting drift across the whole trajectory |
| IMU preintegration factor | Summarizes IMU measurements between two keyframes as a single factor |

A simple ASCII sketch:

```
 [prior]---x1---[odom]---x2---[odom]---x3
                  |                      |
              [landmark]            [landmark]
                  |                      |
                  l1                     l2

            x3 ---[loop closure]--- x1
```

Each factor carries a measurement and a covariance (noise model). Once the graph is built, Gauss-Newton or LM optimizes all variables simultaneously.

### 3.6.3 Solving: Variable Elimination and the Bayes Tree

Optimizing a factor graph requires solving the normal equation `H d = -b` (H is the Hessian approximation, b is the gradient). Understanding the structure of this system is the key to efficient solutions.

**Variable elimination**: the process of eliminating variables one by one. This is mathematically equivalent to sparse Cholesky factorization. The elimination order changes the fill-in (originally zero entries becoming non-zero), which directly affects computational cost.

**Variable ordering**: optimizing the elimination order matters. Heuristics like COLAMD (Column Approximate Minimum Degree) are widely used. Intuitively, eliminating variables with few connections first keeps fill-in low.

**Bayes tree**: a data structure proposed by Kaess et al. (2012), the core of iSAM2. Eliminating a factor graph yields a Bayes net, and reorganizing it into a tree gives the Bayes tree. When a new measurement arrives, only the affected subtree needs re-elimination.

In real-time SLAM, new factors are added every frame. Re-solving the entire system from scratch is O(n^3), but incremental updates via the Bayes tree refresh only the affected part, making real-time processing possible.

> **Further reading**
> - [Factor Graphs and GTSAM (Dellaert & Kaess)](https://gtsam.org/tutorials/intro.html) — The official GTSAM tutorial. Explains the connection from factor graphs to SLAM.
> - [Factor Graphs for Robot Perception (Dellaert & Kaess, 2017)](https://www.cs.cmu.edu/~kaess/pub/Dellaert17fnt.pdf) — A 100-page comprehensive reference.
> - [CMU 16-833 Lecture Notes](https://www.cs.cmu.edu/~kaess/teaching/16833/) — Professor Michael Kaess's SLAM course. Covers factor graphs and iSAM2 in depth.

> **Exercise**: [Factor Graph Visualization](https://alexjunholee.github.io/robotics-practice/app.html#factor_graph_viz)
> Construct variable nodes and factor nodes of a factor graph directly, and see how the graph structure affects optimization.

### 3.6.4 Implementing Pose Graph Optimization with Ceres Solver

Beyond GTSAM, Google's Ceres Solver can also implement factor graph-based optimization. Ceres is a general-purpose nonlinear least squares solver with no SLAM-specific features, which makes it a good way to understand the internals directly. The following is an analysis based on the official Ceres example `pose_graph_3d`.

**Error Term definition:**

Given a relative transformation measurement `T_ab_measured` between two poses `x_a` and `x_b`, the residual is the difference between the estimated relative transformation and the measurement.

```cpp
class PoseGraph3dErrorTerm {
 public:
  PoseGraph3dErrorTerm(Pose3d t_ab_measured,
                       Eigen::Matrix<double, 6, 6> sqrt_information)
      : t_ab_measured_(std::move(t_ab_measured)),
        sqrt_information_(std::move(sqrt_information)) {}

  template <typename T>
  bool operator()(const T* const p_a_ptr, const T* const q_a_ptr,
                  const T* const p_b_ptr, const T* const q_b_ptr,
                  T* residuals_ptr) const {
    // Compute the estimated relative transformation
    Eigen::Quaternion<T> q_a_inverse = q_a.conjugate();
    Eigen::Quaternion<T> q_ab_estimated = q_a_inverse * q_b;
    Eigen::Matrix<T, 3, 1> p_ab_estimated = q_a_inverse * (p_b - p_a);

    // Difference from the measurement
    Eigen::Quaternion<T> delta_q =
        t_ab_measured_.q.cast<T>() * q_ab_estimated.conjugate();

    // residual = [position_error; orientation_error]
    residuals.block<3,1>(0,0) = p_ab_estimated - t_ab_measured_.p.cast<T>();
    residuals.block<3,1>(3,0) = T(2.0) * delta_q.vec();

    // Apply the information matrix (inverse of covariance)
    residuals.applyOnTheLeft(sqrt_information_.cast<T>());
    return true;
  }
};
```

- **template \<typename T\>**: inside Ceres, `T=double` is used when residual values are needed, and `T=Jet<double>` when Jacobians are needed, switching automatically. That is the mechanism of AutoDiff.
- **sqrt_information**: the Cholesky decomposition of the covariance. Computed as `information.llt().matrixL()`.
- **AutoDiffCostFunction dimensions**: `<PoseGraph3dErrorTerm, 6, 3, 4, 3, 4>` — residual 6D, pos_a 3D, quat_a 4D, pos_b 3D, quat_b 4D.
- **SetManifold**: a quaternion has 4 parameters but only 3 DoF, so specify `EigenQuaternionManifold` to optimize on the manifold. In the older API this was `LocalParameterization`.

**Problem setup:**

```cpp
ceres::Problem problem;
ceres::LossFunction* loss_function = nullptr;  // HuberLoss etc. if robust loss is needed
ceres::Manifold* quaternion_manifold = new EigenQuaternionManifold;

for (const auto& constraint : constraints) {
    ceres::CostFunction* cost_function =
        PoseGraph3dErrorTerm::Create(constraint.t_be, sqrt_information);
    problem.AddResidualBlock(cost_function, loss_function,
                              pose_begin.p.data(), pose_begin.q.coeffs().data(),
                              pose_end.p.data(), pose_end.q.coeffs().data());
    problem.SetManifold(pose_begin.q.coeffs().data(), quaternion_manifold);
    problem.SetManifold(pose_end.q.coeffs().data(), quaternion_manifold);
}

// Fix the first pose (remove gauge freedom)
problem.SetParameterBlockConstant(poses.begin()->second.p.data());
problem.SetParameterBlockConstant(poses.begin()->second.q.coeffs().data());
```

**Solve:**

```cpp
ceres::Solver::Options options;
options.max_num_iterations = 200;
options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
ceres::Solver::Summary summary;
ceres::Solve(options, &problem, &summary);
```

`SPARSE_NORMAL_CHOLESKY` suits sparse problems like pose graphs. As the number of variables grows, `SPARSE_SCHUR` is also worth considering.

**GTSAM vs Ceres comparison**

| | GTSAM | Ceres |
|---|---|---|
| Character | SLAM-specialized | general-purpose nonlinear least squares |
| Built-ins | predefined factors like `BetweenFactor`, `PriorFactor` | none; all cost functions defined manually |
| Incremental optimization | supported via iSAM2 | not supported |
| Manifold | built-in Lie group support | set manually via `LocalParameterization` / `Manifold` |
| Best suited for | building SLAM systems | when flexible structure is needed, or large-scale BA |

> **Further reading**
> - [Official Ceres Solver pose_graph_3d example](https://ceres-solver.googlesource.com/ceres-solver/+/master/examples/slam/pose_graph_3d/) — Full version of the code above.
> - [Ceres Solver Tutorial](http://ceres-solver.org/tutorial.html) — Explains AutoDiff and Manifold concepts.
> - [Jinyong Jeong's blog — Ceres Solver Tutorial](https://jinyongjeong.github.io/2023/07/22/Ceres_tutorial/) — Ceres Solver presentation slides and GitHub exercise code. A good entry point to nonlinear optimization.

## 3.7 Advanced: Robust Estimation

*If you want to become a researcher, read from here.*

Real-world data is not clean. False data associations, dynamic objects, and sensor failures produce outliers, and outliers seriously distort optimization results. Robust estimation is the set of techniques for producing sensible estimates even in such situations.

### 3.7.1 Why It's Needed

Standard least squares minimizes the sum of squared errors: `rho(r) = r^2`. Because this function weights large residuals heavily, a single outlier can drag the whole solution.

Concrete cases in SLAM:
- A single wrong loop closure twists the entire map.
- A false positive in visual feature matching ruins the BA result.
- Features attached to dynamic objects (people, cars) violate the static-scene assumption.

### 3.7.2 M-Estimator

An M-estimator uses a different cost function rho instead of `rho(r) = r^2` to reduce the influence of outliers.

| M-Estimator | rho(r) | Characteristics |
|---|---|---|
| **L2 (standard)** | r^2 | Vulnerable to outliers |
| **Huber** | r^2 (abs(r) <= k), 2k*abs(r) - k^2 (abs(r) > k) | L2 for small residuals, L1 for large ones. Most widely used |
| **Cauchy** | c^2 * log(1 + (r/c)^2) | Suppresses outliers more strongly than Huber |
| **Geman-McClure** | r^2 / (1 + r^2) | Effectively ignores extreme outliers |

Huber is the safe default in most cases. With high outlier ratios or extreme cases, consider Cauchy or Geman-McClure. The parameter (k or c) must be tuned to the statistical distribution of the residuals.

In practice, Ceres Solver applies `ceres::HuberLoss`, `ceres::CauchyLoss`, and so on by wrapping the cost function. GTSAM uses `gtsam::noiseModel::mEstimator::Huber`.

> **Exercise**: [M-Estimator comparison](https://alexjunholee.github.io/robotics-practice/app.html#m_estimator)
> Interactively compare how various cost functions such as L2, Huber, Cauchy, and Geman-McClure respond to outliers.

### 3.7.3 RANSAC and Variants

RANSAC (Random Sample Consensus) is an iterative algorithm for fitting a model to data that contains outliers. Unlike M-estimators, it classifies data explicitly as inlier or outlier.

**Basic RANSAC algorithm:**
1. Randomly pick a minimal sample.
2. Fit the model with that sample.
3. Count inliers across the full data (points with residuals within a threshold).
4. Iterate → pick the model with the most inliers.
5. Finally, re-fit the model using all inliers.

**Variants:**

| Variant | Core idea | Trade-off |
|---|---|---|
| RANSAC (basic) | random samples → iterate | Simple and easy to implement but sensitive to threshold and iteration count |
| PROSAC | try good samples first by matching score | Converges fast, but depends on the quality of the prior score |
| Lo-RANSAC | add a local optimization when a good model is found | Higher accuracy, lower speed |
| MAGSAC++ | auto-estimates noise scale sigma, soft inlier/outlier | Close to parameter-free, but computationally expensive |

In OpenCV, you can use MAGSAC++ via the `cv::USAC_MAGSAC` flag in functions like `cv::findHomography` and `cv::findFundamentalMat`.

> **Further reading**
> - [State Estimation for Robotics, Ch.5 (Barfoot)](http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) — A practical chapter on biases, correspondence problems, and outliers.
> - [Hartley & Zisserman, Ch.4 -- Estimation: 2D Projective Transforms](https://www.robots.ox.ac.uk/~vgg/hzbook/) — The original treatment of RANSAC and robust estimation theory.
> - [Dark Programmer — Understanding RANSAC and Its Use in Image Processing](https://darkpgmr.tistory.com/61) — Explains the principle of RANSAC, threshold setting, and iteration-count computation in Korean.
> - [Jinyong Jeong's blog — Jacobian Computation in Bundle Adjustment](https://jinyongjeong.github.io/2020/03/01/Jacobian_of_BA/) — Derives the BA reprojection error Jacobian with Lie algebra and quaternions. Includes handwritten equations.

> **Exercise**: [RANSAC Visualization](https://alexjunholee.github.io/robotics-practice/app.html#ransac)
> Step through how RANSAC classifies inliers and outliers and fits a model on data with outliers.

## 3.8 Advanced: Information Theory Basics

*If you want to become a researcher, read from here.*

Information-theoretic concepts are used in active SLAM, exploration, and uncertainty-based decision making. Just the essentials.

**Shannon entropy**: measures the uncertainty of a random variable X.

```
H(X) = -sum  p(x) log p(x)
```

Higher entropy means greater uncertainty. For a Gaussian, larger covariance means higher entropy.

**KL divergence (Kullback-Leibler divergence)**: measures the "difference" between two probability distributions p and q.

```
D_KL(p || q) = sum  p(x) log(p(x) / q(x))
```

It is asymmetric: D_KL(p||q) != D_KL(q||p). It can be read as "the information loss when you assumed p but the truth is q."

**Mutual information**: measures how much you learn about X by observing Y.

```
I(X; Y) = H(X) - H(X|Y)
```

H(X) is the uncertainty of X before observing Y, and H(X|Y) is the uncertainty after. The difference is the amount of information Y provides about X.

**Active SLAM application**: when a robot decides where to go next, mutual information quantifies "by how much would this action reduce uncertainty in the map/pose?" Choosing the action with the largest expected information gain is the core of information-theoretic exploration.

```
a* = argmax_a  I(X; Z_a)  =  argmax_a  [ H(Z_a) - H(Z_a | X) ]
```

Here a is the action, Z_a is the observation obtained through that action, and X is the environment state.

> **Further reading**
> - [Elements of Information Theory (Cover & Thomas)](https://onlinelibrary.wiley.com/doi/book/10.1002/047174882X) — An information theory textbook.
> - [Placed: An exploration planner using information gain (2022)](https://arxiv.org/abs/2206.05193) — An example of using information theory in active SLAM.

> **Technical Timeline: robotics math and optimization**
> - **~2005**: State estimation centered on the Kalman filter (EKF). Linear-approximation-based, suited to small-scale problems. Real-time processing was difficult, which constrained problem size.
> - **2006~2015**: Factor graph-based optimization (iSAM, g2o, GTSAM) emerged. Sparse matrix structure made large-scale SLAM efficient. Lie groups/algebras became standard tools in the SLAM community.
> - **2016~2020**: Real-time large-scale optimization became practical. Incremental optimization enabled per-frame real-time updates. Ceres Solver became an industry standard.
> - **2021~**: The era of differentiable programming. End-to-end optimization using auto-differentiation (Auto-Diff) in PyTorch/JAX. With the rise of differentiable rendering such as NeRF and 3D Gaussian Splatting, Jacobians that used to be derived by hand were replaced by auto-diff. Differentiable optimization libraries like Theseus (Meta) also appeared.
> - **Now**: Classical math (Lie groups, probability, optimization) is still essential. Differentiable programming is changing how we approach optimization problems, but understanding what auto-diff does internally still requires the foundations covered here. Knowing only the tools means you can't debug.
