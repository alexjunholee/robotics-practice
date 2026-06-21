# Ch.3 — Mathematical Foundations

Understanding Spatial AI properly requires a mathematical foundation. The core concepts here are brief; for deeper study, use the recommended references.

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

---

## 3.9 Advanced: Bayes Filter

Source: Thrun, Burgard, Fox (2005) *Probabilistic Robotics*, Ch.2 (Recursive State Estimation).

§3.8 information theory gave us a tool to *measure* uncertainty. The question now is how to *update* that uncertainty as new observations accumulate over time. Running the Bayes' rule of §3.3.2 recursively along the time axis produces the structure that answers "where is the robot right now?" Where the factor graph of §3.6 assembled spatial constraints into a graph, the Bayes filter extends that estimation structure along the time direction.

"Where is the robot right now?" is not a single-shot estimation problem but a **recursive estimation** problem. The Bayes filter is the most general form of that recursive structure; the Kalman filter and particle filter both operate within this framework.

### 3.9.1 State and the Markov Assumption

**State $x_t$** is the collection of variables that contains all information needed to predict the robot's and environment's future. A "complete state" is a sufficient statistic on its own for predicting the future. That property is the **Markov property**.

$$p(x_{t+1} \mid x_t,\, x_{0:t-1},\, z_{1:t},\, u_{1:t}) = p(x_{t+1} \mid x_t)$$

Under the complete-state assumption, the future depends only on the current $x_t$. The past is irrelevant.

State variables fall into several categories. Along the time axis: dynamic states that change (robot position, velocity) and static states that do not (wall positions, landmarks). By value type: continuous states (pose), discrete states (whether a sensor has failed), and hybrid states (both). In practice, a complete state is almost never achievable, so filters always run on partial approximations. The main sources of Markov assumption violations are model inaccuracies and unmodeled dynamics, along with errors arising from the approximation itself.

### 3.9.2 Environment Interaction: Measurements and Controls

Interactions between the robot and the environment decompose into two data streams.

- **Measurement data** $z_t$: information the environment gives the robot (LiDAR range, camera image). Increases the robot's knowledge over the interval $(t-1, t]$.
- **Control data** $u_t$: actions the robot applies to the environment (motor commands). Applying control increases state uncertainty.

$$z_{t_1:t_2} = z_{t_1},\, z_{t_1+1},\, \ldots,\, z_{t_2} \qquad u_{t_1:t_2} = u_{t_1},\, \ldots,\, u_{t_2}$$

**Odometry is treated as control data.** Wheel encoders are physically sensors, but because they carry information about state change (how far the robot moved), they are classified as $u_t$. A do-nothing command also counts as control: time passing is itself control information.

### 3.9.3 Defining Belief

Belief is the robot's internal posterior distribution over the true state $x_t$, which cannot be measured directly.

$$\text{bel}(x_t) = p(x_t \mid z_{1:t},\, u_{1:t})$$

The predicted belief *before* incorporating measurement $z_t$ is written separately:

$$\overline{\text{bel}}(x_t) = p(x_t \mid z_{1:t-1},\, u_{1:t})$$

The transition $\overline{\text{bel}} \to \text{bel}$ is called the **correction** or **measurement update**. Even GPS does not directly hand the robot its pose — belief is always the result of indirect inference. This $\text{bel}/\overline{\text{bel}}$ distinction is the conceptual foundation of the two phases of the Bayes filter.

### 3.9.4 Generative Laws: Motion Model and Measurement Model

The complete-state assumption yields two conditional independences:

$$p(x_t \mid x_{0:t-1},\, z_{1:t-1},\, u_{1:t}) = p(x_t \mid x_{t-1},\, u_t) \quad \text{(motion model)}$$

$$p(z_t \mid x_{0:t},\, z_{1:t-1},\, u_{1:t}) = p(z_t \mid x_t) \quad \text{(measurement model)}$$

Because $x_{t-1}$ is a sufficient statistic for all past data, the next state depends only on the immediately preceding state and control, and measurements depend only on the current state. Under a time-invariant assumption these collapse to $p(x' \mid u, x)$ and $p(z \mid x)$.

The complete generative model = motion model + measurement model + initial distribution $p(x_0)$. This structure is a Hidden Markov Model / Dynamic Bayes Network.

### 3.9.5 The General Bayes Filter

The most general form of any belief calculator. It alternates a **prediction** step and a **correction** step.

$$\overline{\text{bel}}(x_t) = \int p(x_t \mid u_t,\, x_{t-1})\, \text{bel}(x_{t-1})\, dx_{t-1} \tag{prediction}$$

$$\text{bel}(x_t) = \eta\, p(z_t \mid x_t)\, \overline{\text{bel}}(x_t) \tag{correction}$$

$\eta$ is a normalization constant corresponding to the total-probability denominator ($P(B)$) in the Bayes rule of §3.3.2.

```
# Algorithm Bayes_filter (Table 2.1, adapted)
# Input:  bel(x_{t-1}), u_t, z_t
# Output: bel(x_t)

for all x_t do
    # prediction: integrate over x_{t-1} via the motion model
    bel_bar(x_t) = ∫ p(x_t | u_t, x_{t-1}) · bel(x_{t-1}) dx_{t-1}

    # correction: weight by measurement model and normalize
    bel(x_t) = η · p(z_t | x_t) · bel_bar(x_t)
endfor
return bel(x_t)
```

In a discrete state space the integral becomes a summation. An initial belief $\text{bel}(x_0)$ is required — set it to a point mass if the initial state is known exactly, or a uniform distribution if not. This general form is only directly implementable when the integral has a closed form or the state space is small enough. The Kalman filter (§3.10) and the particle filter (§3.11) each approximate this general form in different ways.

### 3.9.6 Door Estimation: A Worked Example

A two-state (open/closed) door is used to trace by hand how belief updates.

**Model setup:**

- Measurement model: $p(\text{sense\_open} \mid \text{is\_open}) = 0.6$, $p(\text{sense\_open} \mid \text{is\_closed}) = 0.2$
- Motion model push: if already open stays open (probability 1); if closed opens with probability 0.8
- Motion model do_nothing: deterministic identity (state unchanged)
- Initial: $\text{bel}(X_0 = \text{open}) = \text{bel}(X_0 = \text{closed}) = 0.5$

**Step 1: $u_1$ = do_nothing (apply control)**

do_nothing is the identity transform, so $\overline{\text{bel}}(X_1) = (0.5,\; 0.5)$, unchanged.

**Step 2: $z_1$ = sense_open (incorporate measurement)**

$$\overline{\text{bel}}(X_1 = \text{open}) = 0.5, \quad p(\text{sense\_open} \mid \text{open}) = 0.6$$
$$\overline{\text{bel}}(X_1 = \text{closed}) = 0.5, \quad p(\text{sense\_open} \mid \text{closed}) = 0.2$$

Unnormalized posterior: $(0.6 \times 0.5,\; 0.2 \times 0.5) = (0.30,\; 0.10)$. Normalization constant $\eta = 1/(0.30 + 0.10) = 2.5$.

$$\text{bel}(X_1 = \text{open}) = 0.75, \quad \text{bel}(X_1 = \text{closed}) = 0.25$$

**Step 3: $u_2$ = push (apply control)**

$$\overline{\text{bel}}(X_2 = \text{open}) = 1 \cdot 0.75 + 0.8 \cdot 0.25 = 0.95$$
$$\overline{\text{bel}}(X_2 = \text{closed}) = 0 \cdot 0.75 + 0.2 \cdot 0.25 = 0.05$$

**Step 4: $z_2$ = sense_open (incorporate measurement)**

Unnormalized: $(0.6 \times 0.95,\; 0.2 \times 0.05) = (0.570,\; 0.010)$. $\eta = 1/0.580 \approx 1.724$.

$$\text{bel}(X_2 = \text{open}) \approx 0.983, \quad \text{bel}(X_2 = \text{closed}) \approx 0.017$$

| Step | $\text{bel(open)}$ | $\text{bel(closed)}$ |
|------|:-----------------:|:-------------------:|
| Initial | 0.500 | 0.500 |
| After $z_1$ | 0.750 | 0.250 |
| After $u_2$ | 0.950 | 0.050 |
| After $z_2$ | 0.983 | 0.017 |

Even with substantial sensor noise (60% / 20%) and nondeterministic control, accumulated measurements and controls drive belief rapidly toward one hypothesis. Whether 0.983 is a sufficient threshold for autonomous decision-making is a question this example deliberately leaves open.

### 3.9.7 Mathematical Derivation

The two update equations of the Bayes filter follow from three tools alone: Bayes' rule, the law of total probability, and the Markov (complete-state) assumption.

**Correction step derivation:**

Apply Bayes' rule:
$$p(x_t \mid z_{1:t},\, u_{1:t}) = \eta\, p(z_t \mid x_t,\, z_{1:t-1},\, u_{1:t})\, p(x_t \mid z_{1:t-1},\, u_{1:t})$$

The complete-state assumption gives $p(z_t \mid x_t,\, z_{1:t-1},\, u_{1:t}) = p(z_t \mid x_t)$, so:
$$\text{bel}(x_t) = \eta\, p(z_t \mid x_t)\, \overline{\text{bel}}(x_t)$$

**Prediction step derivation:**

Expand $\overline{\text{bel}}$ using the law of total probability:
$$\overline{\text{bel}}(x_t) = \int p(x_t \mid x_{t-1},\, z_{1:t-1},\, u_{1:t})\, p(x_{t-1} \mid z_{1:t-1},\, u_{1:t})\, dx_{t-1}$$

The complete-state assumption reduces the first factor to $p(x_t \mid x_{t-1},\, u_t)$. In the second factor, $u_t$ arrives later than $x_{t-1}$ and can be dropped:
$$\overline{\text{bel}}(x_t) = \int p(x_t \mid u_t,\, x_{t-1})\, \text{bel}(x_{t-1})\, dx_{t-1}$$

The entire derivation rests on the Markov assumption. When the Markov assumption breaks, the equations themselves become inaccurate.

The two lines of the algorithm in §3.9.5 are consequences of Bayes' rule, the law of total probability, and the Markov assumption — nothing else. Knowing where each assumption enters tells you exactly where this filter will fail.

> **Further reading**
> - [Thrun, Burgard, Fox — Probabilistic Robotics (2005)](https://www.probabilistic-robotics.org/) — Ch.2 is the primary source for this section. Algorithm, examples, and derivation are covered completely.
> - [Cyrill Stachniss — Bayes Filter Lecture (YouTube)](https://www.youtube.com/watch?v=0lKHFJpaZkI) — Lecture from Freiburg University. A clear slide-based walkthrough of the Bayes filter.

---

## 3.10 Advanced: Gaussian Filters (KF, EKF, IF)

Source: Thrun, Burgard, Fox (2005) *Probabilistic Robotics*, Ch.3 (Gaussian Filters).

The Bayes filter of §3.9 handles arbitrary beliefs, but the integrals cannot be solved in closed form, which makes direct implementation difficult. The Gaussian filter family resolves this by restricting belief to a Gaussian $\mathcal{N}(\mu_t, \Sigma_t)$. The Kalman filter (KF), extended Kalman filter (EKF), and information filter (IF) all belong to this family, and all three carry over the prediction-correction structure of §3.9 unchanged.

### 3.10.1 Kalman Filter

#### Linear Gaussian System Assumptions

For the KF to be an exact Bayes filter, belief must remain Gaussian at every step. Three assumptions guarantee this.

**State transition (motion model):**
$$x_t = A_t x_{t-1} + B_t u_t + \varepsilon_t, \quad \varepsilon_t \sim \mathcal{N}(0, R_t)$$

$A_t$ is the $n \times n$ state-transition matrix, $B_t$ is the $n \times m$ control-input matrix, and $R_t$ is the process-noise covariance.

**Measurement model:**
$$z_t = C_t x_t + \delta_t, \quad \delta_t \sim \mathcal{N}(0, Q_t)$$

$C_t$ is the $k \times n$ measurement matrix and $Q_t$ is the measurement-noise covariance.

**Initial belief:**
$$\text{bel}(x_0) = \mathcal{N}(\mu_0, \Sigma_0)$$

Under these three assumptions, belief at every time step remains Gaussian:
$$p(x_t \mid u_t, x_{t-1}) = \mathcal{N}(x_t;\; A_t x_{t-1} + B_t u_t,\; R_t)$$
$$p(z_t \mid x_t) = \mathcal{N}(z_t;\; C_t x_t,\; Q_t)$$

#### Kalman Filter Algorithm

The KF represents belief with two quantities $(\mu_t, \Sigma_t)$ and completes one cycle in five steps: two lines of prediction followed by three lines of update.

```
# Algorithm Kalman_filter (Table 3.1, adapted)
# Input:  μ_{t-1}, Σ_{t-1}, u_t, z_t
# Output: μ_t, Σ_t

# --- prediction ---
1: μ̄_t = A_t μ_{t-1} + B_t u_t          # state prediction: apply motion model
2: Σ̄_t = A_t Σ_{t-1} A_t^T + R_t        # covariance prediction: uncertainty grows

# --- correction ---
3: K_t  = Σ̄_t C_t^T (C_t Σ̄_t C_t^T + Q_t)^{-1}   # Kalman gain
4: μ_t  = μ̄_t + K_t (z_t - C_t μ̄_t)    # correct mean using innovation
5: Σ_t  = (I - K_t C_t) Σ̄_t             # covariance shrinks

return μ_t, Σ_t
```

Lines 1–2 are prediction (incorporating control $u_t$; uncertainty grows), and lines 3–5 are the measurement update (incorporating observation $z_t$; uncertainty shrinks).

**Meaning of Kalman gain $K_t$:** $K_t$ sets the confidence balance between prediction and measurement. Large measurement noise $Q_t$ shrinks $K_t$, reducing trust in the measurement; large prediction uncertainty $\bar\Sigma_t$ grows $K_t$, increasing trust in the measurement.

**Innovation:** $z_t - C_t \bar\mu_t$ is the difference between the predicted measurement and the actual measurement. When this is zero, no new information has arrived.

#### 1D KF Illustration: How Information Combines

Visualizing each step of the KF in a 1D position estimation problem makes the intuition clear.

- **Prior $\text{bel}(x_{t-1})$**: a narrow Gaussian — high confidence from the previous estimate.
- **After prediction**: motion adds variance ($\bar\Sigma_t = A_t \Sigma_{t-1} A_t^T + R_t$). The Gaussian flattens.
- **Measurement $z_t$**: represented as a separate Gaussian. Sensor precision $Q_t$ determines the width of this curve.
- **After correction**: multiplying the two Gaussians produces a result narrower than either one — the effect of information combination. The mean sits at the weighted average of the two Gaussians.
- Next motion: variance grows again. Next measurement: variance shrinks again.

Core intuition: **measurements shrink variance; motion grows it.** This alternation is the essence of state estimation. The same intuition carries through §3.10.2 EKF, §3.11.3 particle filter, Ch.14 §14.7 EKF localization, and §14.10 IMU preintegration.

<!-- DEMO: kalman_1d_illustration.html -->

#### Mathematical Derivation of the KF (Key Steps)

The five lines of the KF are the closed-form solution to the two integrals of the Bayes filter (§3.9.5) under the linear Gaussian assumption.

**Part 1 (Prediction).** In the prediction integral of the Bayes filter, the exponent $L_t$ is quadratic in both $x_{t-1}$ and $x_t$. Splitting $L_t$ into a part quadratic in $x_{t-1}$ and a part depending only on $x_t$ makes the $x_{t-1}$ integral a constant, absorbed into normalization. The first- and second-order coefficients in the remaining $x_t$ quadratic yield directly $\bar\mu_t = A_t \mu_{t-1} + B_t u_t$ and $\bar\Sigma_t = A_t \Sigma_{t-1} A_t^T + R_t$.

**Part 2 (Measurement update).** From the correction integral $\text{bel}(x_t) \propto \exp\{-J_t\}$, the first and second derivatives of $J_t$ give $\Sigma_t^{-1} = C_t^T Q_t^{-1} C_t + \bar\Sigma_t^{-1}$. Inverting this directly requires $n \times n$ operations, but the **inversion lemma** (Woodbury identity; see §3.1 linear algebra) transforms it into:
$$K_t = \bar\Sigma_t C_t^T (C_t \bar\Sigma_t C_t^T + Q_t)^{-1}$$
which needs only a $k \times k$ ($k$ = measurement dimension) inversion. When $k \ll n$, the computational cost drops considerably.

**Complexity:** $O(k^{2.8} + n^2)$ per cycle ($k$: measurement dimension, $n$: state dimension).

The derivation pattern — quadratic splitting plus the inversion lemma — recurs identically in the EKF derivation of §3.10.2 and the Gauss-Newton update of the factor graph in §3.6.

The brevity of the five KF lines rests on the linear Gaussian assumption. Relaxing that assumption to the nonlinear case leads to §3.10.2 EKF.

### 3.10.2 Extended Kalman Filter (EKF)

#### Extension to Nonlinear Systems

Real robot systems are not linear. The motion model $g$ for a robot rotating while moving, and the measurement model $h$ for a range sensor, are both inherently nonlinear.

$$x_t = g(u_t, x_{t-1}) + \varepsilon_t, \quad \varepsilon_t \sim \mathcal{N}(0, R_t)$$
$$z_t = h(x_t) + \delta_t, \quad \delta_t \sim \mathcal{N}(0, Q_t)$$

A Gaussian passed through a nonlinear $g$ is no longer Gaussian. The EKF addresses this with a **first-order Taylor expansion**: it linearizes $g$ and $h$ around the most likely point (the predicted mean) to force Gaussian closure.

$$g(u_t, x_{t-1}) \approx g(u_t, \mu_{t-1}) + G_t (x_{t-1} - \mu_{t-1})$$

$$G_t := \frac{\partial g(u_t, x_{t-1})}{\partial x_{t-1}}\bigg|_{\mu_{t-1}} \quad (n \times n \text{ Jacobian})$$

$$h(x_t) \approx h(\bar\mu_t) + H_t (x_t - \bar\mu_t)$$

$$H_t := \frac{\partial h(x_t)}{\partial x_t}\bigg|_{\bar\mu_t} \quad (k \times n \text{ Jacobian})$$

Linearization quality depends on two factors: how nonlinear the function is, and how wide the belief is. Larger variance makes the tangent-plane approximation break down sooner — EKF works well when variance is small.

#### EKF Algorithm

Going from KF to EKF requires only replacing the linear terms with nonlinear functions and their Jacobians.

```
# Algorithm Extended_Kalman_filter (Table 3.3, adapted)
# Input:  μ_{t-1}, Σ_{t-1}, u_t, z_t
# Output: μ_t, Σ_t

# --- prediction ---
1: μ̄_t = g(u_t, μ_{t-1})                         # nonlinear motion model
2: Σ̄_t = G_t Σ_{t-1} G_t^T + R_t                  # covariance propagated through Jacobian

# --- correction ---
3: K_t  = Σ̄_t H_t^T (H_t Σ̄_t H_t^T + Q_t)^{-1}  # Kalman gain (H_t substituted in)
4: μ_t  = μ̄_t + K_t (z_t - h(μ̄_t))               # nonlinear predicted measurement
5: Σ_t  = (I - K_t H_t) Σ̄_t

return μ_t, Σ_t
```

The KF and EKF differ in two lines: (line 1) $A_t \mu_{t-1} + B_t u_t \to g(u_t, \mu_{t-1})$, and (line 4) $C_t \bar\mu_t \to h(\bar\mu_t)$. In the covariance propagation, $A_t \to G_t$ and $C_t \to H_t$.

#### Derivation Summary and Practical Comparison

The derivation parallels §3.10.1 KF: replace the nonlinear $g$ and $h$ with their first-order Taylor expansions, then run the same quadratic-splitting plus inversion-lemma procedure. The result:
$$\bar\mu_t = g(u_t, \mu_{t-1}), \quad \bar\Sigma_t = G_t \Sigma_{t-1} G_t^T + R_t$$
$$\mu_t = \bar\mu_t + K_t (z_t - h(\bar\mu_t)), \quad \Sigma_t = (I - K_t H_t) \bar\Sigma_t, \quad K_t = \bar\Sigma_t H_t^T (H_t \bar\Sigma_t H_t^T + Q_t)^{-1}$$

**Practical comparison:**

The EKF was widely used in SLAM, VIO, and IMU fusion through the mid-2010s. Several alternatives now compete.

- **UKF (Unscented KF):** Uses sigma points to propagate nonlinearity more accurately. No hand-computed Jacobians needed. Useful when state dimension is low.
- **IEKF (Iterated EKF):** Re-computes the Jacobian by repeating the update point. More accurate than EKF under strong nonlinearity.
- **LIEKF (Left-Invariant EKF):** For SO(3)/SE(3) states, uses manifold linearization in place of Taylor linearization. Improves rotation estimation accuracy.

The EKF/IEKF used in Ch.14 §14.7 EKF localization and §14.10 IMU preintegration takes this algorithm box directly. IMU preintegration in §14.10 uses IEKF to reduce rotation drift; localization in §14.7 feeds odometry into EKF prediction and landmark observations into correction. Understanding what $g$, $h$, $G_t$, and $H_t$ are here means the algorithm skeleton in Ch.14 does not need to be rederived when you encounter specific sensor models there.

### 3.10.3 Information Filter (IF)

#### Canonical Form: $(\Omega, \xi)$

The KF and EKF represented Gaussians with $(\mu, \Sigma)$. Writing the same Gaussian in different coordinates swaps the computational complexity of prediction and measurement update. When measurements from multiple robots or sensors must be fused independently, this coordinate system is substantially more convenient.

There is a second way to represent a Gaussian: instead of mean and covariance $(\mu, \Sigma)$, use the **information matrix** $\Omega$ and the **information vector** $\xi$.

$$\Omega = \Sigma^{-1}, \quad \xi = \Sigma^{-1} \mu$$

Inverse: $\Sigma = \Omega^{-1}$, $\mu = \Omega^{-1} \xi$.

In these coordinates the negative log-likelihood of the Gaussian is quadratic in $\Omega$ and $\xi$:

$$p(x) = \eta \exp\!\left\{-\tfrac{1}{2} x^T \Omega x + x^T \xi\right\}$$

$$-\log p(x) = \mathrm{const} + \tfrac{1}{2} x^T \Omega x - x^T \xi$$

The minimum is at $\Omega x = \xi$, i.e., $x = \Omega^{-1} \xi = \mu$. This has exactly the same structure as the normal equation $H \delta x = -b$ of the Gauss-Newton method in §3.4.

**Intuition:** $\Omega = 0$ means total absence of information (complete uncertainty, uniform distribution). In the moments representation $\Sigma = \infty$ is unrepresentable, but the information representation handles it naturally. It is as if you were directly measuring certainty.

#### Information Filter Algorithm

The information filter is the dual of the KF. Where the KF's prediction step was additive, the information filter's **measurement update is additive**.

```
# Algorithm Information_filter (Table 3.4, adapted)
# Input:  ξ_{t-1}, Ω_{t-1}, u_t, z_t
# Output: ξ_t, Ω_t

# --- prediction (requires two matrix inversions) ---
1: Ω̄_t = (A_t Ω_{t-1}^{-1} A_t^T + R_t)^{-1}
2: ξ̄_t = Ω̄_t (A_t Ω_{t-1}^{-1} ξ_{t-1} + B_t u_t)

# --- correction (simple addition!) ---
3: Ω_t  = C_t^T Q_t^{-1} C_t + Ω̄_t       # one measurement = one term added to Ω
4: ξ_t  = C_t^T Q_t^{-1} z_t + ξ̄_t       # one measurement = one term added to ξ

return ξ_t, Ω_t
```

**Complexity duality between KF and IF:**

| Step | KF | IF |
|------|:---:|:---:|
| Prediction | $O(n^2)$ additive | $O(n^{2.8})$, two inversions |
| Measurement update | $O(k^{2.8})$, inversion needed | $O(n^2)$ additive |

$k$: measurement dimension, $n$: state dimension. When measurements touch only part of the state (sparse $C_t$), the IF's measurement update is even cheaper.

#### EIF (Extended Information Filter)

As with the EKF, replacing linear $g, h$ with Jacobians $G_t, H_t$ for nonlinear systems yields the EIF. Substituting $A_t \to G_t$ in prediction and $C_t \to H_t$ in correction gives the EIF algorithm.

```
# Algorithm Extended_Information_filter (key changes only)
# prediction  (μ_{t-1} = Ω_{t-1}^{-1} ξ_{t-1})
Ω̄_t = (G_t Ω_{t-1}^{-1} G_t^T + R_t)^{-1}
ξ̄_t = Ω̄_t · g(u_t, μ_{t-1})       # replaces linear IF's A_t μ_{t-1}+B_t u_t

# correction
Ω_t  = H_t^T Q_t^{-1} H_t + Ω̄_t
ξ_t  = H_t^T Q_t^{-1} z_t + ξ̄_t - H_t^T Q_t^{-1} h(μ̄_t) + H_t^T Q_t^{-1} H_t μ̄_t
```

#### Additivity and the SLAM Connection

The additivity of the measurement update $\Omega_t = \bar\Omega_t + C_t^T Q_t^{-1} C_t$ is a powerful property. One measurement = one term added to $\Omega$. When multiple robots fuse independent measurements, $\Omega_{\text{total}} = \sum_i \Omega_i$ and $\xi_{\text{total}} = \sum_i \xi_i$ combine directly.

This additivity generalizes in §3.6 factor graphs to "one measurement factor = one term $H^T Q^{-1} H$ and one term $H^T Q^{-1} z$ added." The reason factor graphs solve efficiently via sparse Cholesky, and the reason the information matrices in GTSAM and iSAM2 are sparse, both trace back to this additive structure.

The additivity of the information form is the direct foundation of the SLAM information-form representation in EIF SLAM and SEIF — the historical details are in Ch.14 §14.16.

> **Further reading**
> - [Thrun, Burgard, Fox — Probabilistic Robotics (2005)](https://www.probabilistic-robotics.org/) — Ch.3 is the primary source. KF, EKF, and IF derivations appear side by side.
> - [Cyrill Stachniss — Kalman Filter and EKF Lectures](https://www.youtube.com/watch?v=PiCC-SxWlH8) — Freiburg lectures. Strong visual explanations.
> - [Welch & Bishop — An Introduction to the Kalman Filter (2006)](https://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf) — A standard KF introduction. Derivation and intuition are balanced throughout.

---

## 3.11 Advanced: Nonparametric Filters

Source: Thrun, Burgard, Fox (2005) *Probabilistic Robotics*, Ch.4 (Nonparametric Filters).

The Gaussian filters of §3.10 compress belief into two numbers $(\mu, \Sigma)$, but at the cost of being unable to handle nonlinearity or multi-modal distributions properly. Nonparametric filters lift this restriction and can represent arbitrary distributions. The price is computational cost.

### 3.11.1 Histogram Filter / Discrete Bayes Filter

#### Finite State Spaces: Integral Becomes Summation

When the Bayes filter integral of §3.9.5 cannot be solved in closed form, the most direct escape is to make the state space finite. If the state takes only $K$ discrete values $\{x_1, x_2, \ldots, x_K\}$, the integral of §3.9.5 becomes a summation:

$$\bar p_{k,t} = \sum_i p(x_k \mid u_t, x_i)\, p_{i,t-1} \quad \text{(prediction)}$$
$$p_{k,t} = \eta\, p(z_t \mid x_k)\, \bar p_{k,t} \quad \text{(correction)}$$

```
# Algorithm Discrete_Bayes_filter (Table 4.1, adapted)
# Input:  {p_{k,t-1}}, u_t, z_t
# Output: {p_{k,t}}

for all k do
    # prediction: sum transitions from all prior states to x_k
    p̄_{k,t} = Σ_i p(X_t = x_k | u_t, X_{t-1} = x_i) · p_{i,t-1}

    # correction: weight by measurement likelihood
    p_{k,t} = η · p(z_t | X_t = x_k) · p̄_{k,t}
endfor
return {p_{k,t}}
```

This algorithm has the same structure as the HMM forward algorithm in speech recognition. For problems where the state space is naturally discrete (door open/closed, semantic class) it remains the shortest path.

#### Continuous State: Histogram Filter

Partition the continuous state space into a finite collection of regions $\{\mathbf{x}_{k,t}\}$ and assume belief is piecewise uniform within each region:

$$p(x_t) = \frac{p_{k,t}}{|\mathbf{x}_{k,t}|} \quad x_t \in \mathbf{x}_{k,t}$$

Approximate the model using a representative value (mean state) $\hat x_{k,t}$ per region:

$$p(z_t \mid \mathbf{x}_{k,t}) \approx p(z_t \mid \hat x_{k,t})$$
$$p(\mathbf{x}_{k,t} \mid u_t, \mathbf{x}_{i,t-1}) \approx \frac{\eta}{|\mathbf{x}_{k,t}|}\, p(\hat x_{k,t} \mid u_t, \hat x_{i,t-1})$$

When all regions have equal size, the $|\mathbf{x}_{k,t}|$ factors are absorbed into normalization. The resulting discrete Bayes filter is called a **histogram filter**.

**Limitations:** The curse of dimensionality makes it impractical above five dimensions or so. It is unsuitable for 6-DoF pose estimation. Decomposition approaches include density trees (non-uniform partitioning by state density), selective updating (updating only regions where change occurred), and mixed topological-metric representations.

Occupancy grid mapping in Ch.14 is the direct descendant of the histogram filter.

### 3.11.2 Binary Bayes Filter (Log-Odds Form)

#### Binary Estimation of a Static State

When estimating a binary state that does not change over time (e.g., "is this cell occupied?"), there is no state-transition model and the prediction step disappears. Only the correction step repeats.

Directly computing the posterior $p(x \mid z_{1:t})$ at every measurement risks numerical underflow as likelihood products accumulate, and the $[0, 1]$ clamping also needs handling. **Log-odds representation** solves both.

$$l(x) := \log \frac{p(x)}{1 - p(x)} \in (-\infty, +\infty)$$

Log-odds has the entire real line as its range, so clamping is not an issue. The multiplicative Bayes update becomes **additive**:

$$l_t = l_{t-1} + \log\frac{p(x \mid z_t)}{1 - p(x \mid z_t)} - l_0$$

Here $l_0 = \log\frac{p(x)}{1-p(x)}$ is the prior log-odds.

Recovering belief: $\text{bel}_t(x) = 1 - \dfrac{1}{1 + \exp(l_t)}$

```
# Algorithm Binary_Bayes_filter (Table 4.2, adapted)
# Input:  l_{t-1}, z_t
# Output: l_t
# (static state assumed: no prediction step)

1: l_t = l_{t-1}
         + log( p(x|z_t) / (1 - p(x|z_t)) )   # incorporate measurement via inverse sensor model
         - log( p(x) / (1 - p(x)) )            # subtract prior (avoid double-counting)
return l_t
```

#### Inverse Sensor Model

The **inverse sensor model** $p(x \mid z)$ is the reverse of the forward measurement model $p(z \mid x)$. When the camera "sees an open door," the probability that a cell is empty is an example of an inverse model that can be easier to specify than the forward direction. The binary Bayes filter takes this inverse model directly as input.

The log-odds update equation is applied cell by cell in Ch.14 Occupancy Grid Mapping.

### 3.11.3 Particle Filter

#### Core Idea of Nonparametric Representation

The histogram filter's grid grows exponentially with dimension. The particle filter sidesteps this by approximating the distribution with samples instead of a grid.

The particle filter represents belief with $M$ random samples (particles):

$$\mathcal{X}_t = \{x_t^{[1]},\, x_t^{[2]},\, \ldots,\, x_t^{[M]}\}$$

Each particle $x_t^{[m]}$ is more densely concentrated where belief is high. Without any Gaussian assumption, arbitrary distribution shapes — multi-modal, heavy-tailed — can be represented.

#### Particle Filter Algorithm: Sampling, Weighting, Resampling

```
# Algorithm Particle_filter (Table 4.3, adapted)
# Input:  X_{t-1}, u_t, z_t
# Output: X_t (M particles)

X̄_t = X_t = ∅
for m = 1 to M do
    # Step 1: sampling — propagate each particle through the motion model
    x_t^[m] ~ p(x_t | u_t, x_{t-1}^[m])

    # Step 2: weighting — compute importance weight via measurement likelihood
    w_t^[m] = p(z_t | x_t^[m])

    X̄_t = X̄_t ∪ {x_t^[m], w_t^[m]}
endfor

for m = 1 to M do
    # Step 3: resampling — draw M new particles proportional to weights
    draw i with probability ∝ w_t^[i] from X̄_t
    add x_t^[i] to X_t
endfor
return X_t
```

In the limit $M \to \infty$, $x_t^{[m]} \sim p(x_t \mid z_{1:t}, u_{1:t})$.

#### Importance Sampling Intuition

When direct sampling from a target distribution $f$ is difficult, draw from a proposal distribution $g$ and correct with weights $w = f/g$:

$$w^{[m]} = \frac{f(x^{[m]})}{g(x^{[m]})}$$

The weighted empirical distribution converges, for any Borel set $A$:
$$\left[\sum_{m=1}^M w^{[m]}\right]^{-1} \sum_{m=1}^M \mathbf{1}(x^{[m]} \in A)\, w^{[m]} \;\longrightarrow\; \int_A f(x)\, dx$$

The convergence rate is $O(1/\sqrt{M})$. The closer the proposal is to the target, the smaller the constant.

In the particle filter, the proposal propagates each particle through the motion model $p(x_t \mid u_t, x_{t-1})$, and the target is $\text{bel}(x_t)$ which also incorporates the measurement. The "missing information" between a proposal that has not seen $z_t$ and a target that has is $p(z_t \mid x_t^{[m]})$, which gives the Step 2 weights their intuitive justification.

Why this falls out to exactly $p(z_t \mid x_t^{[m]})$ becomes clear when the argument is made rigorously in sequence space.

#### Convergence and Implementation

The rigorous derivation works by treating each particle not as a single state $x_t^{[m]}$ but as a state sequence $x_{0:t}^{[m]}$. Two applications of Bayes plus the Markov property factorize the target; the proposal factorizes inductively; their ratio reduces to exactly $\eta\, p(z_t \mid x_t^{[m]})$. This holds exactly only as $M \to \infty$.

Without resampling, weights concentrate on a small number of particles — **weight degeneracy** — which is why Step 3 is needed. §3.11.4 covers the new problems that resampling itself introduces.

<!-- DEMO: particle_filter_1d.html -->

### 3.11.4 Four Sources of Error in Particle Filters

The particle filter is an approximation and carries structural errors. Resampling is a weight-based selection, similar to how low-weight candidates get pruned in the optimization of §3.4. Understanding the four error sources makes particle filter debugging systematic.

#### (1) Systematic Bias from Finite $M$

Imagine $M = 1$. The single weight normalizes against itself: $w/w = 1$. The measurement is completely ignored. With finite $M$, weights are confined to the $M-1$-dimensional simplex and random errors accumulate. Bias decreases as $M$ grows, but computational cost grows linearly.

#### (2) Sample Impoverishment from Resampling

In a static state ($x_t = x_{t-1}$) with no motion, all particles follow identical trajectories. Unbounded resampling drives particle diversity to zero and the filter collapses to a single state.

Mitigation: hold resampling when the robot is stationary. Alternatively, resample only when weight variance is high and otherwise accumulate weights multiplicatively:

$$w_t^{[m]} = \begin{cases} 1 & \text{(immediately after resampling)} \\ p(z_t \mid x_t^{[m]})\, w_{t-1}^{[m]} & \text{(when not resampling)} \end{cases}$$

#### (3) Proposal-Target Divergence

When sensors are very accurate but motion is imprecise, the target belief is narrow while the proposal is wide, and efficiency collapses. In the extreme, a noiseless range sensor would confine the support of $p(z \mid x)$ to a low-dimensional manifold, leaving most particles with weight $\approx 0$.

Mitigation: deliberately inflate measurement noise (at the cost of some precision) or use a measurement-aware proposal that incorporates measurement information at the sampling stage.

#### (4) Particle Deprivation

In a high-dimensional space, there may be no particle near the true state. The randomness of resampling has a nonzero probability every cycle of sweeping out all particles near the true state. Once lost, they are hard to recover.

Mitigation: inject a small number of **random particles** drawn from the prior every cycle. This slightly distorts the posterior but prevents catastrophic failure.

#### Low-Variance Sampler

The standard resampling implementation is the **low-variance (systematic) sampler**. A single random number draws $M$ particles at regular intervals, achieving $O(M)$ complexity.

```
# Algorithm Low_variance_sampler (Table 4.4, adapted)
# Input:  X̄_t (weighted particles), W_t (weight array)
# Output: X̄_t (resampled particles)

r = rand(0, M^{-1})    # single uniform random number in [0, 1/M)
c = w_t^[1]             # cumulative weight
i = 1
X̄_t = ∅
for m = 1 to M do
    u = r + (m-1) · M^{-1}   # advance at regular intervals
    while u > c do
        i = i + 1
        c = c + w_t^[i]       # accumulate weight
    endwhile
    add x_t^[i] to X̄_t       # select particle at this position
endfor
return X̄_t
```

When all weights are equal, the output is identical to the input — no particles are lost in steps with no measurement. $O(M)$ versus $O(M \log M)$ for independent sampling.

With this, both why the particle filter works and where it breaks down are clear. Each of the four error sources has a mitigation, and choosing the right tradeoff for the situation is the core of practical implementation. The four errors are addressed by augmented MCL and mixture MCL in Ch.14 §14.7.

> **Further reading**
> - [Thrun, Burgard, Fox — Probabilistic Robotics (2005)](https://www.probabilistic-robotics.org/) — Ch.4 is the primary source. Algorithms and analysis for histogram, binary Bayes, and particle filters are covered completely.
> - [Arulampalam et al. — A Tutorial on Particle Filters (IEEE Trans. Signal Processing 2002)](https://ieeexplore.ieee.org/document/978374) — A standard tutorial on particle filter theory and applications.
> - [Thrun — Particle Filters in Robotics (UAI 2002)](https://www.aaai.org/Papers/UAI/2002/UAI02-079.pdf) — A short paper explaining the connection to Rao-Blackwellized PF and FastSLAM.
> - [ROS AMCL package](https://wiki.ros.org/amcl) — The theory of §3.11.3–3.11.4 implemented in a production package. Augmented MCL and the low-variance sampler are applied directly.

---

The Bayes filter is the framework. The KF, EKF, and IF are closed-form implementations of that framework under the Gaussian assumption. The histogram filter and particle filter buy flexibility without the Gaussian assumption, at the cost of computation. Which filter to choose is determined by state-space dimension and whether the distribution is multi-modal.

When you encounter EKF localization (§14.7), MCL (§14.7), and IMU preintegration (§14.10) in Ch.14, there is no need to follow each algorithm's derivation from scratch. The filter vocabulary built here makes it possible to identify what $g$, $h$, and the proposal are in each algorithm, and the skeleton becomes immediately visible.
