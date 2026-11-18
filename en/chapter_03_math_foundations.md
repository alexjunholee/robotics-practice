# Ch.3 — Mathematical Foundations

In Ch.2, we saw how cameras, LiDAR, and IMUs observe the physical world and how sensor models compare predicted observations against noisy measurements. Turning those raw streams into a reliable estimate of where a robot is—and how it should move next, as we will see when we turn to kinematics in Ch.4—requires a shared mathematical language. Without a working command of concepts like "optimization on SE(3)" or "deriving the Jacobian and solving the system with Gauss-Newton," the core arguments of Spatial AI and SLAM papers remain opaque and their implementations hard to reconstruct. The mathematics in this chapter serves as a practical toolkit for robotics, bridging undergraduate fundamentals to the formulations used directly in research papers and production code.

Differentiable programming and automatic differentiation have reshaped how these mathematical tools enter modern pipelines. Frameworks such as PyTorch and JAX now compute Jacobians and gradients that once required painstaking hand derivation, enabling end-to-end learned SLAM and differentiable rendering with NeRF or 3D Gaussian Splatting. Even so, automatic differentiation does not absolve you from understanding the underlying math: interpreting what those derivatives mean, choosing the right parameterization, and debugging why an optimization fails still depend on the linear algebra, geometry, probability, and filtering theory developed here.

## 3.1 Linear Algebra

Linear algebra is the common language of coordinate transformations, camera projection models, numerical optimization, and deep learning. Rather than treating it abstractly, this section connects standard matrix operations to the geometric calculations that appear in everyday robotics code.

### 3.1.1 Vectors and Matrices

A **vector** represents a quantity that carries both magnitude and direction. In robotics, column vectors are the standard representation for 3D point coordinates, forces, linear and angular velocities, and state variables—saying "the robot sits at (3, 2, 1) in the world frame" simply packages its position as a 3D column vector:

```
v = [v_x, v_y, v_z]^T  (column vector)
```

Matrices act on these vectors to transform, project, and combine quantities across spaces. Four basic matrix operations recur constantly:
- **Addition and subtraction**: element-wise combination of matrices of matching dimensions.
- **Multiplication**: row-by-column inner products that compose linear maps.
- **Transpose** (`A^T`): swaps rows and columns, turning column vectors into row vectors and reversing orthogonal transforms.
- **Inverse** (`A^(-1)`): satisfies `AA^(-1) = I`, undoing an invertible linear transformation.

Nearly every geometric operation in perception and control reduces to matrix multiplication: projecting a 3D landmark onto a 2D camera image, rotating a velocity vector, or transforming coordinates from a sensor frame into the robot body frame.

> **Further reading**
> - [3Blue1Brown — Essence of Linear Algebra](https://www.youtube.com/playlist?list=PLZHQObOWTQDPD3MizzM2xVFitgF8hE_ab) — Visualizes matrix multiplication and eigenvalues geometrically, framing linear algebra as spatial transformations rather than mere arithmetic rules.
> - [Introduction to Applied Linear Algebra (Boyd & Vandenberghe) — free PDF](https://web.stanford.edu/~boyd/vmls/) — An applied linear algebra textbook by Stanford's Stephen Boyd, offering a practical perspective complete with Python examples.
> - [Dark Programmer — Linear Algebra series (6 posts: basic formulas to PCA)](https://darkpgmr.tistory.com/103) — Summarizes core concepts including inverses, eigenvalues, SVD, linear systems, and PCA (in Korean).
> - [Dark Programmer — Vector and Matrix Calculus](https://darkpgmr.tistory.com/141) — Covers vector and matrix differentiation rules needed for Jacobian derivations (in Korean).

### 3.1.2 Eigenvalue Decomposition

Most vectors change both length and direction when multiplied by a square matrix A. For certain special directions, however, the matrix acts purely as a scalar stretch or compression without rotating the vector at all. Those invariant directions are **eigenvectors**, and their corresponding scaling factors are **eigenvalues**:

```
Av = λv
```

- v: eigenvector
- λ: eigenvalue

In robotics, eigenvalue decomposition underpins Principal Component Analysis (PCA), covariance matrix analysis, and system stability checks. It appears immediately when processing 3D point clouds: running PCA on a local neighborhood of points extracts the eigenvectors of the sample covariance matrix as the principal axes of the distribution, while the eigenvalues measure the variance along each axis. The ratios among those eigenvalues distinguish whether a local patch behaves like a flat plane, an edge line, or an isotropic corner, and the eigenvector paired with the smallest eigenvalue gives the surface normal vector.

> **Further reading**
> - [3Blue1Brown — Eigenvectors and Eigenvalues](https://www.youtube.com/watch?v=PFDu9oVAE-g) — Builds geometric intuition for what eigenvalues and eigenvectors represent.
> - [MIT 18.06 Linear Algebra — Gilbert Strang (YouTube)](https://www.youtube.com/playlist?list=PLE7DDD91010BC51F8) — A classic lecture series covering linear algebra in depth, including eigenvalue decomposition.

> **Exercise**: [PCA 3D · Dimensionality Reduction](https://alexjunholee.github.io/robotics-practice/app.html#pca_3d)
> Manipulate the process by which the eigenvectors of the covariance matrix become the principal axes of a 3D distribution, and simultaneously visualize dimensionality reduction onto the PC1·PC2 plane (3D→2D) and the PC1 axis (2D→1D).

### 3.1.3 Singular Value Decomposition (SVD)

Eigenvalue decomposition applies only to square matrices and is not always numerically well-behaved. Sensor measurements in robotics, by contrast, usually yield rectangular, overdetermined, or noisy matrices. **Singular Value Decomposition (SVD)** generalizes spectral decomposition to any m×n matrix A by factoring it into an orthogonal rotation/reflection, a non-negative axis-wise scaling, and another orthogonal transform:

```
A = UΣV^T
```

- U: left singular vectors (m×m orthogonal matrix)
- Σ: diagonal matrix of singular values (m×n)
- V: right singular vectors (n×n orthogonal matrix)

**Uses**: least squares solutions, matrix approximation, and fundamental matrix computation.

Because orthogonal matrices preserve norms and never amplify rounding errors, SVD provides the standard numerically stable workhorse for solving overdetermined linear systems, computing low-rank approximations, and recovering rigid alignments in point cloud registration. In multi-view geometry, for example, the final step of the classic 8-point algorithm extracts the fundamental matrix from eight or more point correspondences by taking the right singular vector corresponding to the smallest singular value.

> **Further reading**
> - [Steve Brunton — Singular Value Decomposition (YouTube)](https://www.youtube.com/watch?v=nbBvuuNVfco) — Clear University of Washington lectures explaining the mathematical meaning of SVD and its engineering applications.
> - [Linear Algebra and Its Applications (Gilbert Strang)](https://math.mit.edu/~gs/linearalgebra/ila6/indexila6.html) — A standard linear algebra textbook whose SVD chapter is particularly illuminating.

## 3.2 3D Geometry

Linear algebra supplies the operations; 3D geometry gives them physical meaning in space. To fuse sensor readings, track motion, and build maps across viewpoints, a robot needs a consistent mathematical language for describing positions and orientations.

### 3.2.1 Coordinate Frames

Every Spatial AI system constantly translates quantities among multiple **coordinate frames**. Typical frames include the globally fixed **World Frame (W)**, the **Camera Frame (C)** centered at the optical origin, the **Body Frame (B)** attached to the robot base, and the **IMU Frame (I)** aligned with the inertial sensor axes.

An object detected in an image is initially known only in the camera frame. Before a planner or controller can steer the robot around it, those coordinates must be mapped into the body or world frame. Because each sensor sits at a different physical mounting location and orientation, multi-sensor fusion relies on the calibrated extrinsic transformations connecting these frames.

Transforming a point `p_C` from the camera frame into the world frame takes a single 4×4 matrix product:

```
p_W = T_WC × p_C
```

Here `T_WC` is the 4×4 homogeneous transformation matrix from the camera frame to the world frame.

> **Further reading**
> - [State Estimation for Robotics, Ch.6 — Coordinate Frames (Tim Barfoot) — free PDF](http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) — Treats coordinate-frame transformations rigorously from the perspective of robotics state estimation.
> - [Stanford CS231A — Camera Models](https://web.stanford.edu/class/cs231a/) — Covers camera coordinate frames and perspective projection models in Stanford's computer vision course.

### 3.2.2 Rotation Representations

While 3D position has a straightforward vector representation, 3D orientation is trickier because rotations live on a curved space rather than a flat Euclidean grid. Robotics and vision rely on four standard rotation representations, each trading off parameter count, algebraic constraints, singularities, and interpolation behavior:

- **Rotation Matrix R**: A 3×3 orthogonal matrix satisfying `R^T R = I` and `det(R) = 1`. Although it uses 9 numbers, the 6 orthogonality constraints leave exactly 3 true degrees of freedom. It composes cleanly by matrix multiplication and acts directly on vectors, though its constraints require special care during optimization.
- **Euler Angles**: Represent orientation with three sequential rotations—Roll (φ), Pitch (θ), and Yaw (ψ). They are intuitive to inspect and configure, but suffer from **gimbal lock** (loss of a degree of freedom when two axes align), and their meaning depends on the rotation order (such as ZYX vs. XYZ).
- **Quaternion q = [w, x, y, z]** (`||q|| = 1`): Encodes 3 DoF using 4 unit-norm parameters. Quaternions avoid gimbal lock entirely and support smooth spherical linear interpolation (Slerp), making them the standard attitude representation in robotics.
- **Axis-Angle**: Combines a unit rotation axis n and a rotation angle θ into a compact 3-parameter vector, converting to a rotation matrix via Rodrigues' formula.

In practice, software ecosystems pick different defaults: ROS messages standardize on quaternions, OpenCV returns Rodrigues axis-angle vectors, and optimization libraries such as Ceres and GTSAM support Lie-group manifolds directly. Whenever you bridge these libraries, verify the quaternion component ordering (`[w, x, y, z]` vs. `[x, y, z, w]`) and maintain unit normalization.

> **Further reading**
> - [3Blue1Brown — Quaternions and 3D Rotation](https://www.youtube.com/watch?v=zjMuIxRvygQ) — Visualizes the 4D geometry of quaternions and builds intuition for why 3D rotations benefit from four parameters.
> - [State Estimation for Robotics, Ch.7 — Rotation (Tim Barfoot)](http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) — Provides a unified treatment of all rotation representations and conversions between them.
> - [Sola — Quaternion Kinematics for the Error-State Kalman Filter (Tech Report)](https://arxiv.org/abs/1711.02508) — A practical reference detailing quaternion kinematics for error-state Kalman filters in VIO and INS pipelines.
> - [3D Rotation Converter](https://www.andre-gaschler.com/rotationconverter/) — Interactive web tool for verifying conversions among quaternions, Euler angles, and rotation matrices.

> **Exercise**: [Rotation Representations and Gimbal Lock](https://alexjunholee.github.io/robotics-practice/app.html#rotation_gimbal) | [6DoF Pose Visualization](https://alexjunholee.github.io/robotics-practice/app.html#xyzrpy_6dof)
> Manipulate and compare Euler-angle Gimbal Lock and quaternion rotation directly, and interactively explore a 6-DoF pose (x, y, z, roll, pitch, yaw).

### 3.2.3 Homogeneous Coordinates

A general rigid-body motion combines a rotation R and a translation t as `p' = Rp + t`. Because translation is an addition rather than a linear map, rigid motions cannot be chained purely by multiplying 3×3 matrices. Extending 3D points into 4D **homogeneous coordinates** by appending a trailing 1 folds both rotation and translation into a single 4×4 transformation matrix:

```
[X, Y, Z, 1]^T  (3D point)

T = | R   t |   (4×4 transformation matrix)
    | 0   1 |
```

In homogeneous coordinates, `p' = Rp + t` becomes a single matrix multiplication `p' = Tp`. This uniformity pays off immediately whenever multiple transforms are composed: chaining transformations from link to link along a robot arm (which we will formalize in Ch.4) or from camera to body to world reduces to multiplying 4×4 matrices in sequence.

### 3.2.4 SE(3) and SO(3)

Collecting all valid 3D rotations gives the **Special Orthogonal Group SO(3)** (3 DoF), while combining 3D rotations and translations forms the **Special Euclidean Group SE(3)** (6 DoF).

Both SO(3) and SE(3) are **Lie groups**—smooth curved manifolds equipped with a group multiplication. During SLAM optimization, you cannot simply add an increment to a rotation matrix without breaking orthogonality and `det(R) = 1`. Lie theory resolves this cleanly by computing unconstrained updates in the flat tangent space—the corresponding **Lie algebra** (`so(3)` or `se(3)`)—and mapping those increments back onto the curved group via the exponential map. We develop this machinery in detail in §3.5.

> **Further reading**
> - [State Estimation for Robotics, Ch.7 (Tim Barfoot) — free PDF](http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) — Develops SE(3), SO(3), and Lie groups and algebras for robotics state estimation.
> - [Sola — A Micro Lie Theory for State Estimation in Robotics (arXiv:1812.01537)](https://arxiv.org/abs/1812.01537) — A concise, highly practical tutorial summarizing just the Lie theory needed for robotics state estimation.

## 3.3 Probability & Statistics

Geometry describes where a robot could be, but real sensors from Ch.2 never yield exact numbers. Every measurement carries noise, and every state estimate carries uncertainty. Saying "the range is 3.0 m" is incomplete without a spread; saying "3.0 m ± 0.05 m (95% confidence interval)" captures how much we should trust that reading. Probability and statistics formalize how uncertainty is represented, propagated, and refined over time.

### 3.3.1 Gaussian Distribution

When modeling continuous sensor noise or pose uncertainty, the **Gaussian (normal) distribution** is the default starting point. In one dimension, its density is parameterized by mean μ and variance σ²:

```
p(x) = (1 / √(2πσ²)) × exp(-(x-μ)²/(2σ²))
```

In higher dimensions—such as a 3D position or 6-DoF pose—this generalizes to the **multivariate Gaussian**:

```
p(x) = N(μ, Σ)
```

- μ: mean vector
- Σ: covariance matrix

The Gaussian owes its ubiquity both to nature and to algebra. Under broad conditions, the central limit theorem explains why the sum of many small, independent disturbances approaches a bell curve. Algebraically, Gaussians are closed under key operations: linear combinations of independent Gaussian random variables remain Gaussian, and multiplying two Gaussian densities over the same variable yields another Gaussian after normalization. That analytical closure is precisely what makes the Kalman filter tractable.

> **Further reading**
> - [3Blue1Brown — But what is the Central Limit Theorem?](https://www.youtube.com/watch?v=zeJD6dqJ5lo) — Visualizes the central limit theorem to build intuition for why Gaussians emerge across independent noise sources.
> - [Kalman Filter — How it works, in pictures](http://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/) — An illustrated walkthrough of Kalman filtering intuition before diving into matrix equations.

> **Exercise**: [Kalman Filter](https://alexjunholee.github.io/robotics-practice/app.html#kalman_filter)
> Interactively manipulate the Kalman filter's predict-update cycle and observe Gaussian-based state estimation in action.

**Mahalanobis Distance**

Ordinary Euclidean distance treats all directions identically, which breaks down when sensor uncertainty is anisotropic. A GPS receiver, for instance, typically exhibits two to three times larger error vertically than horizontally: a 1-meter discrepancy vertically may be routine noise, whereas 1 meter horizontally could indicate a genuine jump.

The **Mahalanobis distance** measures deviation relative to the covariance Σ, rescaling each direction by its uncertainty:

```
d_M = sqrt((x - μ)^T Σ^{-1} (x - μ))
```

When Σ = I, Mahalanobis distance reduces to ordinary Euclidean distance; when Σ is diagonal, it scales each coordinate axis independently; and for a full covariance matrix Σ, it measures distance along the covariance ellipsoid's principal axes. In SLAM data association, `d_M` answers whether an observation plausibly matches a predicted landmark: a candidate that looks nearby in Euclidean space but lies far off along a tight covariance axis is flagged as an unlikely match.

(See: [Dark Programmer — Mean, Standard Deviation, Variance, and Mahalanobis Distance](https://darkpgmr.tistory.com/41))

### 3.3.2 Bayes' Rule

State estimation is fundamentally about revising beliefs as new evidence arrives. **Bayes' rule** expresses how a prior belief `P(A)` updates to a posterior belief `P(A|B)` once measurement `B` is observed:

```
P(A|B) = P(B|A) × P(A) / P(B)
```

Applied sequentially over time, this gives the core recursion of **recursive state estimation**—both Kalman filters and particle filters implement this same update under different distribution approximations:

```
P(x_t | z_{1:t}) ∝ P(z_t | x_t) × P(x_t | z_{1:t-1})
```

- `P(z_t | x_t)`: **measurement model** (likelihood)—how probable is sensor reading `z_t` if the robot is at state `x_t`?
- `P(x_t | z_{1:t-1})`: **prior**—given all past observations up to `t-1`, how likely is the robot to be at `x_t` before incorporating `z_t`?

At each step, multiplying the prior by the measurement likelihood produces the updated posterior. Under linear-Gaussian assumptions, the Kalman filter's predict-and-update cycle implements this exact recursion in closed form.

> **Further reading**
> - [3Blue1Brown — Bayes' Theorem](https://www.youtube.com/watch?v=HZGCoVF3YvM) — A crisp visual introduction to Bayesian updating.
> - [Probabilistic Robotics (Thrun, Burgard, Fox)](http://probabilistic-robotics.org/) — The foundational textbook on probabilistic robotics, deriving Bayes filters, Kalman filters, particle filters, and SLAM from first principles.
> - [Giseop Kim's blog — Bayesian Filtering series (2 posts)](https://gisbi-kim.github.io/blog/2021/03/09/bayesfiltering-1.html) — Walks through Bayesian filtering leading up to the Kalman filter (in Korean).

### 3.3.3 MLE and MAP

When turning probabilistic models into concrete point estimates, two closely related objectives arise depending on whether prior knowledge is included.

**Maximum Likelihood Estimation (MLE)** chooses the parameter `x*` that makes the observed measurements `z` most probable, ignoring any prior:

```
x* = argmax P(z | x)
```

**Maximum A Posteriori (MAP)** estimation maximizes the posterior probability instead, combining measurement likelihood with a prior `P(x)`:

```
x* = argmax P(x | z) = argmax P(z | x) × P(x)
```

In SLAM, MLE asks which trajectory best explains the sensor observations alone, whereas MAP combines those observations with prior state information (such as an initial pose anchor or motion model). Practical SLAM systems almost universally solve MAP problems because priors prevent drift or ill-conditioned solutions when measurements are noisy or sparse. Moreover, under Gaussian noise with fixed covariance, minimizing the negative log-posterior of MAP produces a sum of covariance-weighted squared observation residuals plus a quadratic prior penalty—turning probabilistic inference directly into regularized nonlinear least squares.

> **Further reading**
> - [Probabilistic Robotics, Ch.2 — Recursive State Estimation (Thrun)](http://probabilistic-robotics.org/) — Explains the connections among MLE, MAP, and recursive Bayes filtering in robotics.
> - [Cyrill Stachniss — Maximum Likelihood and MAP Estimation](https://www.youtube.com/watch?v=XepXtl9YKwc) — An example-driven lecture contrasting MLE and MAP.

**Intuitive difference between MLE and MAP**

While both methods seek the most plausible parameter values, they differ in how they weigh external knowledge. MLE trusts only the data at hand, asking which parameter maximizes `P(z | x)`. MAP tempers the data with prior belief `P(x)`, asking which parameter maximizes the full posterior `P(x | z) ∝ P(z | x) P(x)`.

Expressed in log space, MAP simply adds a log-prior term to the MLE objective. When that prior is Gaussian, MAP reduces to MLE plus L2 regularization—the same mechanism behind weight decay in deep learning. In a SLAM back-end, multiplying odometry likelihoods, landmark observation likelihoods, and an initial state prior turns each measurement and prior into a single factor inside a factor graph (§3.6).

(See: [Dark Programmer — Bayes' Rule, ML and MAP, and Image Processing](https://darkpgmr.tistory.com/62))

## 3.4 Optimization Basics

Once MAP estimation casts state estimation as minimizing a sum of weighted squared errors, finding the best state becomes a numerical optimization problem. SLAM bundle adjustment, camera calibration, and neural-network training all minimize objective functions of this kind. Keeping the residual, Jacobian, and update step distinct makes it far easier to diagnose whether a convergence failure stems from the model, a bad initialization, or the solver itself.

### 3.4.1 Least Squares

When a system of sensor equations has more measurements than unknowns—an overdetermined system—noise makes an exact solution `Ax = b` impossible. Instead, **least squares** finds the parameter vector `x*` that minimizes the sum of squared residuals:

```
x* = argmin ||Ax - b||²
```

Setting the derivative with respect to x to zero gives the closed-form **normal equation**:

```
x* = (A^T A)^(-1) A^T b
```

Least squares is the standard starting point for fitting models to noisy measurements, from line fitting and plane segmentation to camera calibration.

> **Further reading**
> - [Cyrill Stachniss — Least Squares for Robotics](https://www.youtube.com/watch?v=r2cyMQ5NB1o) — Concrete walkthrough of applying least squares to robotics problems.
> - [Giseop Kim's blog — SLAM back-end series (3 posts)](https://gisbi-kim.github.io/blog/2021/03/04/slambackend-1.html) — Introduces SLAM back-ends from "SLAM is solving Ax=b" through factor graphs.
> - [Giseop Kim's blog — Iterative Optimization, Part 1](https://gisbi-kim.github.io/blog/2021/03/16/leastsquare-1.html) — Intuitive walkthrough of nonlinear optimization (in Korean).

**Intuition for least squares**

Why minimize squared error rather than absolute error? Squaring offers two key advantages: it is smooth and differentiable everywhere, and it penalizes large errors more heavily than small ones. Moreover, under Gaussian measurement noise, minimizing the sum of squared errors yields the exact same estimate as MLE.

In an overdetermined linear system, solving the normal equation `A^T A x = A^T b` produces the unique best-fit vector whenever `A^T A` is invertible. If `A^T A` is singular (rank-deficient), however, no unique inverse exists. In that case, computing the SVD `A = U Σ V^T` gives the Moore–Penrose pseudo-inverse `A^+ = V Σ^+ U^T`, which yields the minimum-norm solution `x = A^+ b`.

(See: [Dark Programmer — Understanding Least Squares and Various Uses](https://darkpgmr.tistory.com/56))

### 3.4.2 Gradient Descent

Most geometric models in robotics—such as camera projection or 3D rigid motion—are nonlinear, so no single closed-form normal equation can solve them in one shot. **Gradient descent** tackles nonlinear objectives iteratively by taking small steps opposite to the local gradient `∇f(x_k)`, the direction of steepest descent:

```
x_{k+1} = x_k - α × ∇f(x_k)
```

- α: step size (learning rate)
- ∇f: gradient

While gradient descent drives deep-learning training, it serves mainly as a baseline in geometric robotics. Tuning the step size α is delicate, progress slows to a crawl in narrow valleys, and iterations can stall in poor local minima. Consequently, SLAM and calibration pipelines prefer second-order-informed methods such as Gauss-Newton and Levenberg-Marquardt.

**Relationships between gradient, Jacobian, and Hessian**

Three derivative objects organize all first- and second-order optimization methods:
- The **gradient** `∇f` is the first derivative of a scalar function `f`, forming an n×1 column vector that points in the direction of steepest ascent.
- The **Jacobian** `J` generalizes this to a vector-valued function `f: R^n → R^m` as an m×n matrix holding the partial derivative of every output component with respect to every input variable.
- The **Hessian** `H` is the n×n symmetric matrix of second derivatives of a scalar function `f`, encoding local curvature used in Newton-style updates.

For a nonlinear least-squares cost built from a residual vector `r(x)`, these three quantities link together cleanly:

```
For cost function C(x) = (1/2)||r(x)||²:
  Gradient:  ∇C = J^T r           (J is the Jacobian of r)
  Hessian:   H ≈ J^T J            (Gauss-Newton approximation: drop the second-derivative term)
  Update:    δx = -(J^T J)^{-1} J^T r
```

Exact Hessians require expensive second-order derivatives of the residual. Near a good fit where the residual r is small, that second-order term becomes negligible, making `J^T J` an accurate and much cheaper curvature approximation.

(See: [Dark Programmer — Gradient, Jacobian, Hessian, Laplacian](https://darkpgmr.tistory.com/132))

### 3.4.3 Gauss-Newton

Rather than taking tiny steps along the gradient alone, **Gauss-Newton** exploits the special structure of squared-error objectives. At each iteration, it replaces the nonlinear residual with its first-order linear approximation around the current estimate `x_k` and solves the resulting linear least-squares subproblem:

```
(J^T J) Δx = -J^T r
x_{k+1} = x_k + Δx
```

- J: Jacobian matrix of the residual
- r: residual vector

By incorporating curvature information through `J^T J`, Gauss-Newton converges dramatically faster than gradient descent. When a SLAM back-end optimizes thousands or tens of thousands of variables simultaneously, first-order gradient descent is far too slow, whereas Gauss-Newton often reaches convergence in just a handful of iterations.

### 3.4.4 Levenberg-Marquardt (LM)

Gauss-Newton converges rapidly near the solution, but if initialized far away—or if `J^T J` is nearly singular—its full Newton-like step can overshoot and diverge. **Levenberg-Marquardt (LM)** stabilizes the update by blending Gauss-Newton with gradient descent through a damping factor λ:

```
(J^T J + λI) Δx = -J^T r
```

- λ: damping factor
- small λ → Gauss-Newton (fast local convergence)
- large λ → gradient descent (cautious, stable steps)

This blend makes LM the workhorse solver for bundle adjustment and pose graph optimization in SLAM, and the default algorithm inside libraries such as Ceres Solver, g2o, and GTSAM.

> **Further reading**
> - [Cyrill Stachniss — Gauss-Newton and Levenberg-Marquardt for SLAM](https://www.youtube.com/watch?v=hRyL5KwFLAE) — Step-by-step explanation of how Gauss-Newton and LM operate inside SLAM.
> - [State Estimation for Robotics, Ch.4 — Nonlinear Optimization (Tim Barfoot) — free PDF](http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) — Rigorous treatment of nonlinear optimization from a state-estimation perspective.
> - [Ceres Solver Tutorial](http://ceres-solver.org/tutorial.html) — Official guide to Google's nonlinear least-squares library, demonstrating LM in practice.
> - [Dark Programmer — Intuitive Understanding of Optimization Methods](https://darkpgmr.tistory.com/149) — Builds geometric intuition for gradient descent, Newton's method, and LM (in Korean).
> - [Giseop Kim's blog — Five recommended resources for SLAM back-end study](https://gisbi-kim.github.io/blog/2021/10/03/slam-textbooks.html) — Curated guide covering error-state KF, factor graphs, and bundle adjustment.
> - [Derivative Calculator](https://www.derivative-calculator.net/) — Interactive symbolic differentiation tool useful for verifying hand-derived Jacobians.

**Intuition for LM: switching between Gauss-Newton and gradient descent**

In nonlinear least squares, the trade-off between speed and stability is governed dynamically by λ. When a step successfully lowers the cost, the solver gains confidence in the local quadratic approximation and decreases λ toward Gauss-Newton for rapid convergence. When a step increases the cost, the solver rejects it and increases λ, shrinking the step size and turning toward the safer gradient-descent direction. That adaptive feedback loop explains why LM remains the default solver in Ceres Solver and across modern SLAM back-ends.

(See: [Dark Programmer — Summary of Function Optimization Methods (LM, etc.)](https://darkpgmr.tistory.com/142))

## 3.5 Advanced: Lie Group and Lie Algebra

Every update rule in §3.4 assumes an additive step `x_{k+1} = x_k + Δx` in flat Euclidean space. The moment a state contains a 3D rotation or rigid-body pose, that assumption breaks down. One of the most common mathematical hurdles in robotics is figuring out how to optimize orientations without violating their geometric constraints. **Lie groups** and **Lie algebras** resolve this cleanly and underpin modern SLAM back-ends, visual-inertial odometry, and bundle adjustment.

### 3.5.1 Why We Need Lie Groups

Each rotation representation introduced in §3.2 runs into trouble under standard Euclidean optimization:

- **Rotation matrix R**: Uses 9 parameters for 3 DoF because of the constraints `R^T R = I` and `det(R) = 1`. An unconstrained additive step `R + ΔR` immediately leaves the space of valid rotations.
- **Quaternion**: Carries 4 parameters subject to `||q|| = 1`. Every additive update requires re-normalization, and repeated projection distorts steps and accumulates numerical error.
- **Euler angles**: Avoid explicit algebraic constraints, but break down near gimbal lock and introduce discontinuities across ±π angle wrap-around.

The root cause is geometric: rotations live on a curved nonlinear manifold rather than a flat Euclidean vector space. Lie theory bridges that gap by attaching a flat tangent space—the **Lie algebra**—at a point on the manifold. Optimization computes an unconstrained step inside that tangent vector space, and the exponential map projects the step cleanly back onto the curved manifold.

Formally, a **group** is a set paired with a binary operation satisfying four axioms: closure, associativity, existence of an identity element, and existence of inverses. Under matrix multiplication, the set of invertible n×n matrices forms the general linear group GL(n), and those with determinant 1 form the special linear group SL(n). Matrices that preserve inner products form the orthogonal group O(n), and restricting to det = 1 isolates the proper rotation group SO(n). Matrices with det = -1 represent reflections; since multiplying two reflections gives a proper rotation (det = +1), reflections alone fail closure and do not form a group.

### 3.5.2 SO(3): The 3D Rotation Group

Specializing SO(n) to three dimensions gives the **3D rotation group SO(3)**, the set of all 3×3 orthogonal matrices with unit determinant:

```
SO(3) = { R in R^{3x3} | R^T R = I, det(R) = 1 }
```

SO(3) satisfies all group axioms under matrix multiplication: the product `R_1 R_2` of two rotations remains in SO(3), the identity matrix I serves as the identity element, and orthogonality makes the transpose `R^T` equal to the inverse `R^{-1}`. Note that 3D rotations are associative but not commutative—in general, `R_1 R_2 != R_2 R_1`.

To optimize over SO(3) without constraints, we work in its tangent space at the identity—the **Lie algebra so(3)**, which consists of 3×3 skew-symmetric matrices and has dimension 3. The **hat operator** `[.]x` maps any 3D vector `w` in `R^3` to its corresponding skew-symmetric matrix:

```
w = [w1, w2, w3]^T  (in R^3)

        [  0   -w3   w2 ]
[w]x =  [  w3   0   -w1 ]   in so(3)
        [ -w2   w1   0  ]
```

This matrix representation turns the 3D cross product into ordinary matrix multiplication: `[w]x v = w x v`. Its inverse, the **vee operator** `(.)v`, extracts the 3D vector back out of a skew-symmetric matrix. Physically, a vector w in `so(3)` encodes an axis of rotation (its direction) and a rotation angle or angular rate (its magnitude), matching the axis-angle representation directly.

### 3.5.3 Exponential Map and Logarithmic Map

How do we move back and forth between flat tangent vectors in `so(3)` and curved rotation matrices in SO(3)? The **exponential map** (`so(3) -> SO(3)`) sends a Lie algebra vector to a rotation matrix, and its origin is most easily seen from continuous-time kinematics.

Consider a rotation matrix `R(t)` evolving smoothly over time with `R(0) = I`. Because `R(t)` remains in SO(3) at all times, `R(t) R(t)^T = I` holds identically. Differentiating both sides with respect to time gives:

```
d/dt (R R^T) = R_dot R^T + R R_dot^T = 0
→  R_dot R^T = -(R_dot R^T)^T
```

Because `R_dot R^T` equals the negative of its own transpose, it is skew-symmetric and can therefore be written as the hat matrix of some angular velocity vector `w(t)`:

```
R_dot(t) R^T(t) = [w(t)]x
→  R_dot(t) = [w(t)]x R(t)
```

When angular velocity w is constant, this linear matrix differential equation integrates directly via the matrix exponential:

```
R(t) = exp([w]x * t) = sum_{n=0}^{inf} ([w]x * t)^n / n!
```

Evaluating at `t = 1`, `exp([w]x)` produces the matrix that rotates by `||w||` radians about axis w. Setting `theta = ||w||` and applying the cyclic identity `[w]x^3 = -theta^2 [w]x` collapses the infinite series into the sine and cosine Taylor expansions, yielding **Rodrigues' formula** in closed form:

```
exp([w]x) = I + (sin(theta) / theta) [w]x + ((1 - cos(theta)) / theta^2) [w]x^2
```

When theta is very small (`|theta| < eps`), `sin(theta)/theta ≈ 1` and `(1 - cos(theta))/theta^2 ≈ 1/2`, simplifying the exponential map to a second-order Taylor approximation:

```
exp([w]x) ≈ I + [w]x + (1/2)[w]x^2   (second-order approximation)
```

One subtlety arises when inverting this mapping: the exponential map is many-to-one, since adding any integer multiple of `2*pi` to `||w||` leaves the resulting rotation matrix unchanged.

The **logarithmic map** (`SO(3) -> so(3)`) performs the inverse operation, recovering an axis-angle vector w from a rotation matrix R:

```
theta = arccos((tr(R) - 1) / 2)
[w]x = (theta / (2 sin(theta))) (R - R^T)
```

Near `theta = 0` (identity rotation) or `theta = pi` (180-degree rotation), `sin(theta)` vanishes, so production implementations branch to series approximations or eigenvector extraction to stay numerically stable.

**Intuition**: Think of the Lie algebra as the flat tangent plane attached to the curved rotation manifold at the identity I. Small rotation increments behave like ordinary vectors in `R^3`, and the exponential map wraps each tangent vector cleanly onto the manifold. During optimization, solvers compute a 3D update `dw` in the tangent space and compose `exp([dw]x)` onto the current rotation—moving smoothly along SO(3) without ever leaving it.

### 3.5.4 SE(3): The 3D Rigid-Body Transformation Group

A robot's full 6-DoF pose couples 3D rotation with 3D translation. Collecting all rigid-body motions defines the **Special Euclidean Group SE(3)**, represented by 4×4 homogeneous transformation matrices:

```
SE(3) = { T = [ R  t ] | R in SO(3), t in R^3 }
              [ 0  1 ]
```

Like SO(3), SE(3) forms a Lie group under matrix multiplication `T_1 T_2`, with the 4×4 identity matrix as its identity element and `T^{-1} = [ R^T  -R^T t ; 0  1 ]` as its inverse.

Its tangent space at the identity—the **Lie algebra se(3)**—is 6-dimensional. Elements of `se(3)` are parameterized by 6D **twist** vectors combining a translational component `rho` in `R^3` and a rotational component `w` in `R^3`:

```
xi = [rho; w] in R^6     (rho in R^3: translation part, w in R^3: rotation part)
```

Note that conventions vary across libraries on whether translation or rotation comes first inside `xi`. The **hat operator** maps this 6D vector into a 4×4 matrix in `se(3)`:

```
        [ [w]x  rho ]
xi^ =   [  0     0  ]   in se(3)    (4x4 matrix)
```

Exponentiating `xi^` lifts the twist onto SE(3). Crucially, because rotation and translation interact along a curved helical path, the translation block is not simply `rho`, but `rho` transformed by the left Jacobian J of SO(3):

```
exp(xi^) = [ exp([w]x)   J rho ]   in SE(3)
           [    0           1   ]
```

where J is given in closed form by:

```
J = I + ((1 - cos(theta)) / theta^2) [w]x + ((theta - sin(theta)) / theta^3) [w]x^2
```

Through this exponential map, a 6-DoF pose can be perturbed cleanly using an unconstrained vector `xi` in `R^6`, enabling standard Gauss-Newton or LM steps in Euclidean tangent space while keeping every iterate strictly on the SE(3) manifold.

> **Exercise**: [SE(3) Pose Composition](https://alexjunholee.github.io/robotics-practice/app.html#pose_composition_3d)
> Manipulate the composition of SE(3) transformations in 3D and see how combined rotation-and-translation rigid-body transformations chain together.

### 3.5.5 Perturbation Model and Jacobian

When computing a Jacobian for Gauss-Newton or LM around a current pose estimate `T` in SE(3), a small tangent-space perturbation `d_xi` can be applied on either side, depending on which coordinate frame you perturb in:

**Left perturbation (global frame):**
```
T' = exp(d_xi^) * T
```

**Right perturbation (body frame):**
```
T' = T * exp(d_xi^)
```

Both conventions are valid provided residuals and updates stay mutually consistent, though mixed conventions cause subtle bugs: Barfoot's textbook predominantly uses left perturbations, whereas Strasdat's Sophus defaults to right perturbations.

With a perturbation convention fixed, the **Jacobian** of an error function `e(T)` measures how the residual changes with respect to the 6D tangent increment:

```
de/d(d_xi) = lim_{d_xi->0}  (e(exp(d_xi^) * T) - e(T)) / d_xi     (for left perturbation)
```

This produces a matrix with as many rows as the error dimension and 6 columns. Instead of Euclidean addition `x <- x + dx`, each optimization iteration proceeds in two steps:

1. Solve for the unconstrained 6D tangent step via Gauss-Newton: `d_xi = -(J^T J)^{-1} J^T e`.
2. Retract onto the manifold via group multiplication: `T <- exp(d_xi^) * T` (for left perturbation).

Because `exp(d_xi^)` always lies in SE(3) and SE(3) is closed under multiplication, the updated pose T stays valid automatically with no external constraints. Libraries such as g2o, GTSAM, and Ceres implement this exact mechanism: GTSAM's `Pose3` exposes `Pose3::Expmap()` and `Pose3::Logmap()` natively, while Ceres models tangent-space updates through `Manifold` (formerly `LocalParameterization`).

### 3.5.6 Adjoint Representation

In multi-sensor systems, a twist or perturbation expressed in one coordinate frame (such as an IMU or camera frame) often needs to be transformed into another (such as the body or world frame). The **adjoint representation** `Ad_T` performs this 6×6 linear transformation on elements of `se(3)`:

```
Ad_T = [ R    [t]x R ]   in R^{6x6}
       [ 0      R    ]
```

Given a rigid transformation `T_ab` in SE(3), a twist `xi_b` in frame b maps to frame a via:

```
xi_a = Ad_{T_ab} * xi_b
```

**Practical meaning**: Because linear velocity and angular velocity couple through the lever arm `[t]x R` whenever the origin shifts, transforming a 6D twist across frames is not a block-diagonal rotation. In visual-inertial odometry (VIO), where an IMU measures angular velocity and specific force in the IMU frame while the state tracks the body or camera frame, the adjoint captures how perturbations and covariances propagate across sensor extrinsics.

### 3.5.7 Use in Practice

Modern robotics libraries package Lie-group algebra behind clean APIs so you rarely implement Rodrigues' formula by hand.

**Sophus (C++)** is a widely used Lie-group library written by Hauke Strasdat that implements SO(3), SE(3), exponential and logarithmic maps, and adjoint matrices. Systems such as ORB-SLAM3 rely on Sophus directly, whereas Kimera uses GTSAM's `Pose3` and `Rot3` equivalents.

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

**Jaxlie (Python/JAX)**, developed by Brent Yi, brings Lie groups into JAX so automatic differentiation computes exact manifold Jacobians without hand derivation—especially convenient for research prototyping and differentiable pipelines.

```python
import jaxlie
import jax.numpy as jnp

T = jaxlie.SE3.identity()
delta = jnp.array([0.01, 0.0, 0.0, 0.0, 0.0, 0.001])
T_updated = jaxlie.SE3.exp(delta) @ T
```

**GTSAM** embeds SE(3) directly inside `gtsam::Pose3`, handling tangent-space perturbations seamlessly during factor graph optimization.

> **Further reading**
> - [State Estimation for Robotics, Ch.7-8 (Barfoot)](http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) — The standard reference for Lie groups in robotics state estimation.
> - [A micro Lie theory for state estimation in robotics (Sola et al., arXiv:1812.01537)](https://arxiv.org/abs/1812.01537) — A 20-page, self-contained summary of Lie theory tailored to robotics.
> - [TUM Multiple View Geometry, Ch.2 -- Rigid Body Motion](https://cvg.cit.tum.de/teaching/online/mvg) — Lectures by Daniel Cremers visually introducing SO(3) and SE(3).
> - [Sophus GitHub](https://github.com/strasdat/Sophus) — Source code for the C++ Lie-group library; inspecting the headers clarifies exact conventions.
> - [Jinyong Jeong's blog — SE(3) and SO(3) transformation](https://jinyongjeong.github.io/2016/06/07/se3_so3_transformation/) — Systematic walkthrough of SE(3) and SO(3) starting from GL(3) and O(3) (in Korean).
> - [T-Robotics: Lie Group Formulation for Robot Mechanics](http://t-robotics.blogspot.com/2015/07/lie-group-formulation-for-robot.html) — Summarizes Lie-group formulations in robot mechanics and dynamics (in Korean).

## 3.6 Advanced: Factor Graph

With Lie groups in place to represent and perturb 3D poses, the next question is how to organize an entire SLAM problem—thousands of poses, landmarks, and sensor constraints—into a solvable mathematical structure. **Factor graphs** provide that unifying framework, forming the back-end representation of most modern SLAM systems.

### 3.6.1 What Is a Factor Graph

A factor graph is a bipartite graph containing two distinct types of nodes:

- **Variable nodes**: the unknown states to be estimated, such as robot poses (`x_1, x_2, ...`) and 3D landmark positions (`l_1, l_2, ...`).
- **Factor nodes**: probabilistic constraints or measurements connecting subsets of variables, where each factor defines a local error term over the variables it touches.

Rather than treating the joint posterior as one monolithic density, a factor graph factorizes it into a product over all local factors:

```
p(X | Z) proportional to  prod_i  f_i(X_i)
```

where `X_i` denotes the subset of variables connected to factor `f_i`.

Finding the MAP estimate means maximizing this product of factors—or equivalently, taking the negative logarithm to minimize a sum of squared residuals. Under Gaussian noise models, MAP inference directly becomes a structured **nonlinear least squares** problem:

```
X* = argmin_X  sum_i  ||e_i(X_i)||^2_{Sigma_i}
```

Here `e_i` is the factor's error function and `Sigma_i` is the measurement covariance weighting its contribution.

### 3.6.2 Expressing SLAM as a Factor Graph

Different sensor streams and geometric constraints map cleanly onto specific factor types in a SLAM back-end:

| Factor | Role |
|---|---|
| Prior factor | Encodes prior knowledge about an initial state (for instance, anchoring the starting pose at the origin). |
| Odometry factor | Constrains the relative transformation between two consecutive poses using wheel odometry or IMU preintegration. |
| Landmark observation factor | Ties a pose to an observed landmark, typically through pixel reprojection error or range-bearing residuals. |
| Loop closure factor | Connects a pose to a previously visited pose upon place recognition, eliminating accumulated trajectory drift. |
| IMU preintegration factor | Compresses high-rate IMU measurements between two keyframes into a single relative motion factor. |

Connecting these pieces produces a sparse graph like the following, where poses advance along a chain while landmarks and loop closures weave across non-consecutive states:

```
 [prior]---x1---[odom]---x2---[odom]---x3
                  |                      |
              [landmark]            [landmark]
                  |                      |
                  l1                     l2

            x3 ---[loop closure]--- x1
```

Each factor stores its measurement and noise covariance, and once the graph is constructed, Gauss-Newton or Levenberg-Marquardt optimizes all variables jointly.

### 3.6.3 Solving: Variable Elimination and the Bayes Tree

Optimizing a factor graph requires repeatedly solving the linear normal equation `H d = -b`, where `H` is the Gauss-Newton Hessian approximation and `b` is the gradient vector. In a SLAM problem with thousands of variables, solving this system densely would be prohibitively slow; real-time performance depends on exploiting the graph's sparsity.

- **Variable elimination**: Eliminating unknown variables one by one solves the normal system, and is algebraically equivalent to sparse Cholesky factorization. Crucially, the order in which variables are eliminated governs **fill-in**—the creation of new non-zero entries where zeros originally stood—which directly determines runtime and memory cost.
- **Variable ordering**: Choosing a good elimination order is therefore critical. Heuristics such as **COLAMD** (Column Approximate Minimum Degree) keep fill-in small by eliminating sparsely connected variables before densely connected hubs.
- **Bayes tree**: Introduced by Kaess et al. (WAFR 2010) and powering **iSAM2** (Kaess et al., IJRR 2012), the Bayes tree reorganizes the Bayes net produced by variable elimination into a tree structure. When new measurements arrive, only the affected subtree requires re-elimination rather than the entire graph.

Because real-time SLAM appends new factors at every frame or keyframe, re-solving from scratch at `O(n^3)` cost quickly falls behind. Incremental updates via the Bayes tree localize computation to the changing subgraph, keeping optimization real-time even over long trajectories.

The factor graph is closely tied to the information filter of §3.10 — both representations exploit the sparse structure of the information matrix $\Omega$.

> **Further reading**
> - [Factor Graphs and GTSAM (Dellaert & Kaess)](https://gtsam.org/tutorials/intro.html) — Official GTSAM tutorial explaining how factor graphs map onto SLAM problems.
> - [Factor Graphs for Robot Perception (Dellaert & Kaess, 2017)](https://www.cs.cmu.edu/~kaess/pub/Dellaert17fnt.pdf) — A comprehensive 100-page monograph on factor graphs and incremental smoothing.
> - [CMU 16-833 Lecture Notes](https://www.cs.cmu.edu/~kaess/teaching/16833/) — Michael Kaess's SLAM course covering factor graphs, variable ordering, and iSAM2 in depth.

> **Exercise**: [Factor Graph Visualization](https://alexjunholee.github.io/robotics-practice/app.html#factor_graph_viz)
> Construct variable nodes and factor nodes of a factor graph directly, and see how the graph structure affects optimization.

### 3.6.4 Implementing Pose Graph Optimization with Ceres Solver

While GTSAM is built specifically for SLAM, Google's **Ceres Solver** is a general-purpose nonlinear least-squares library with no built-in SLAM primitives—making it an ideal vehicle for seeing how factor graphs work under the hood. The walkthrough below follows Ceres' official `pose_graph_3d` example.

First, define the **error term** (factor) connecting two poses `x_a` and `x_b`. Given a measured relative transformation `T_ab_measured`, the residual compares the relative pose predicted from the current estimates against that measurement, weighted by the square-root information matrix:

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

Four implementation details make this functor work cleanly inside Ceres:
- **`template <typename T>`**: Ceres evaluates the same templated functor with `T = double` when computing residual costs and with `T = Jet<double>` (dual numbers) when automatic differentiation needs exact Jacobians.
- **`sqrt_information`**: The Cholesky factor of the information matrix (`information.llt().matrixL()`), which whitens the residual so minimizing its squared L2 norm matches the Mahalanobis cost.
- **`AutoDiffCostFunction` dimensions**: `<PoseGraph3dErrorTerm, 6, 3, 4, 3, 4>` specifies a 6D residual evaluated over four parameter blocks: `pos_a` (3D), `quat_a` (4D), `pos_b` (3D), and `quat_b` (4D).
- **`SetManifold`**: Because a quaternion packs 3 rotation DoF into 4 parameters, attaching `EigenQuaternionManifold` ensures updates stay on the unit quaternion manifold (replacing `LocalParameterization` in older Ceres releases).

Next, assemble the optimization **problem** by adding a residual block for every relative pose constraint, assigning the quaternion manifold, and locking the first pose to remove global gauge freedom:

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

Finally, configure the sparse linear solver and run the optimization loop:

```cpp
ceres::Solver::Options options;
options.max_num_iterations = 200;
options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
ceres::Solver::Summary summary;
ceres::Solve(options, &problem, &summary);
```

`SPARSE_NORMAL_CHOLESKY` fits sparse pose-graph problems well. For bundle adjustment problems where a large block of 3D point variables can be eliminated via the Schur complement, `SPARSE_SCHUR` is usually preferable.

Choosing between GTSAM and Ceres depends on whether you want SLAM-specific abstractions or maximum modeling flexibility:

| | GTSAM | Ceres |
|---|---|---|
| Character | SLAM-specialized | General-purpose nonlinear least squares |
| Built-ins | Predefined factors such as `BetweenFactor` and `PriorFactor` | None; all cost functions are defined by the user |
| Incremental optimization | Supported natively via iSAM2 | Not supported |
| Manifold | Built-in Lie-group support | Configured manually via `LocalParameterization` / `Manifold` |
| Best suited for | Building end-to-end SLAM systems | Custom cost structures or large-scale bundle adjustment |

> **Further reading**
> - [Official Ceres Solver pose_graph_3d example](https://ceres-solver.googlesource.com/ceres-solver/+/master/examples/slam/pose_graph_3d/) — Complete source code for the 3D pose-graph solver excerpted above.
> - [Ceres Solver Tutorial](http://ceres-solver.org/tutorial.html) — Explains AutoDiff, loss functions, and Manifold concepts in detail.
> - [Jinyong Jeong's blog — Ceres Solver Tutorial](https://jinyongjeong.github.io/2023/07/22/Ceres_tutorial/) — Slide deck and hands-on GitHub exercises introducing nonlinear optimization with Ceres.

## 3.7 Advanced: Robust Estimation

Factor graphs and least squares silently assume that residuals follow a Gaussian distribution. Real-world sensor data regularly breaks that assumption: mismatched features, moving pedestrians, and transient sensor glitches inject **outliers** whose errors are orders of magnitude larger than nominal noise. **Robust estimation** protects optimization from being dragged off course by these corrupted measurements.

### 3.7.1 Why It's Needed

Standard least squares penalizes errors quadratically through `rho(r) = r^2`. Because the gradient of `r^2` grows linearly without bound as `r` increases, even a single large outlier exerts enormous pull on the solution. In SLAM, that sensitivity leads to catastrophic failures:
- A single false loop closure can twist an entire trajectory map out of alignment.
- Outlier feature correspondences corrupt camera pose and 3D point estimates during bundle adjustment.
- Keypoints tracked on dynamic objects (such as vehicles or pedestrians) violate the static-world assumption and bias ego-motion estimates.

### 3.7.2 M-Estimator

One way to limit outlier influence inside continuous optimization is to replace the quadratic loss `rho(r) = r^2` with a sub-quadratic **M-estimator** `rho(r)` whose slope grows slowly—or vanishes—for large residuals:

| M-Estimator | rho(r) | Characteristics |
|---|---|---|
| **L2 (standard)** | r^2 | Vulnerable to outliers |
| **Huber** | r^2 (abs(r) <= k), 2k*abs(r) - k^2 (abs(r) > k) | Quadratic near zero, linear in the tails; widely supported across solvers |
| **Cauchy** | c^2 * log(1 + (r/c)^2) | Suppresses large outliers more aggressively than Huber |
| **Geman-McClure** | r^2 / (1 + r^2) | Saturates asymptotically, effectively ignoring extreme outliers |

For mild contamination, **Huber** loss is a reliable default because it remains convex and preserves L2 behavior on clean inliers. When outlier rates are high or extreme mismatches occur, **Cauchy** or **Geman-McClure** offers stronger suppression, at the cost of introducing non-convexity; in all cases, the scale threshold (`k` or `c`) should reflect the expected inlier residual distribution. Both Ceres (`ceres::HuberLoss`, `ceres::CauchyLoss`) and GTSAM (`gtsam::noiseModel::mEstimator::Huber`) apply M-estimators directly by wrapping factor residuals.

> **Exercise**: [M-Estimator comparison](https://alexjunholee.github.io/robotics-practice/app.html#m_estimator)
> Interactively compare how various cost functions such as L2, Huber, Cauchy, and Geman-McClure respond to outliers.

### 3.7.3 RANSAC and Variants

When outliers make up a large fraction of the measurements—often 30% to 70% in raw visual feature matching—continuous loss functions alone can get trapped in bad local minima. **RANSAC (Random Sample Consensus)** tackles this front-end problem by explicitly separating **inliers** from **outliers** before any final refinement:

1. Draw a minimal random subset of measurements needed to instantiate a model (for example, 2 points for a line or 5–8 correspondences for camera geometry).
2. Fit a candidate model to that minimal sample.
3. Score the candidate across all measurements by counting how many fall within an inlier residual threshold.
4. Repeat for a fixed or adaptive number of trials, keeping the candidate with the largest consensus set.
5. Re-fit the final model using all identified inliers via least squares or nonlinear optimization.

Several widely used variants improve RANSAC's speed, robustness to thresholds, and accuracy:

| Variant | Core idea | Trade-off |
|---|---|---|
| RANSAC (basic) | Random minimal samples across iterations | Simple and general, but sensitive to the inlier threshold and iteration budget |
| PROSAC | Prioritizes high-scoring matches during sampling | Converges much faster, though quality depends on the matching scores |
| Lo-RANSAC | Runs local optimization whenever a new best model is found | Higher geometric accuracy at modest extra compute |
| MAGSAC++ | Marginalizes over noise scale sigma with soft inlier weights | Nearly threshold-free, at higher computational cost per iteration |

In OpenCV, MAGSAC++ is available out of the box via the `cv::USAC_MAGSAC` flag in functions such as `cv::findHomography` and `cv::findFundamentalMat`.

> **Further reading**
> - [State Estimation for Robotics, Ch.5 (Barfoot)](http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf) — Practical coverage of biases, correspondence ambiguities, and outlier handling.
> - [Hartley & Zisserman, Ch.4 -- Estimation: 2D Projective Transforms](https://www.robots.ox.ac.uk/~vgg/hzbook/) — Classic textbook exposition of RANSAC (originally introduced by Fischler & Bolles, 1981) and robust geometric estimation.
> - [Dark Programmer — Understanding RANSAC and Its Use in Image Processing](https://darkpgmr.tistory.com/61) — Explains RANSAC mechanics, threshold selection, and stopping criteria (in Korean).
> - [Jinyong Jeong's blog — Jacobian Computation in Bundle Adjustment](https://jinyongjeong.github.io/2020/03/01/Jacobian_of_BA/) — Derives BA reprojection-error Jacobians with Lie algebra and quaternions, including handwritten derivations.

> **Exercise**: [RANSAC Visualization](https://alexjunholee.github.io/robotics-practice/app.html#ransac)
> Step through how RANSAC classifies inliers and outliers and fits a model on data with outliers.

## 3.8 Advanced: Information Theory Basics

Up to this point, our mathematical tools have focused on estimating a robot's state and protecting that estimate against noise and outliers. Yet an autonomous robot also needs to decide *where to move or look next* to learn as much as possible about its surroundings. **Information theory** supplies the scalar quantities needed to measure uncertainty and guide active exploration.

**Shannon entropy** quantifies the average uncertainty inherent in a random variable `X`:

```
H(X) = -sum  p(x) log p(x)
```

Spread-out distributions carry higher entropy than sharp, confident ones; for a Gaussian distribution, larger covariance directly corresponds to higher entropy.

To compare two probability distributions `p` and `q`, **Kullback-Leibler (KL) divergence** measures how much information is lost when approximating `p` by `q`:

```
D_KL(p || q) = sum  p(x) log(p(x) / q(x))
```

Note that KL divergence is not a symmetric distance metric (`D_KL(p||q) != D_KL(q||p)`), reflecting the asymmetry between true distribution `p` and approximating model `q`.

Finally, **mutual information** measures how much observing a random variable `Y` reduces our uncertainty about `X`:

```
I(X; Y) = H(X) - H(X|Y)
```

Here `H(X)` is the prior entropy of `X` before observing `Y`, and `H(X|Y)` is the expected posterior entropy afterward; their difference is the information gain provided by `Y`.

In **active SLAM**, mutual information turns exploration into an optimization problem. When choosing its next trajectory or viewpoint, the robot evaluates which candidate action `a` maximizes expected information gain over the map and pose state `X`:

```
a* = argmax_a  I(X; Z_a)  =  argmax_a  [ H(Z_a) - H(Z_a | X) ]
```

where `Z_a` is the sensor observation expected along action `a`.

> **Further reading**
> - [Elements of Information Theory (Cover & Thomas)](https://onlinelibrary.wiley.com/doi/book/10.1002/047174882X) — The standard reference textbook on information theory.
> - Placed et al., "A Survey on Active Simultaneous Localization and Mapping: State of the Art and New Frontiers" (IEEE T-RO 2023) — Surveys how information-theoretic objectives drive modern active SLAM.

> **Technical Timeline: robotics math and optimization**
> - **~2005**: State estimation centered on the extended Kalman filter (EKF). Because linearizations scaled poorly to large state vectors, real-time computation constrained maps to relatively small environments.
> - **2006~2015**: Factor-graph optimization (iSAM, g2o, GTSAM) reshaped SLAM back-ends. Exploiting sparse matrix structure made large-scale mapping practical, while Lie groups and Lie algebras became standard tools across the community.
> - **2016~2020**: Incremental smoothing and sparse solvers matured into real-time, per-frame back-ends, supported by open nonlinear least-squares libraries such as Ceres Solver across both academia and industry.
> - **2021~**: Differentiable programming entered geometric robotics, with auto-differentiation in PyTorch and JAX powering differentiable rendering (NeRF, 3D Gaussian Splatting) and differentiable optimization libraries such as Meta's Theseus.
> - **Now**: Classical foundations—Lie groups, probability, and sparse optimization—remain indispensable. Auto-diff automates Jacobian code generation, yet diagnosing numerical conditioning, parameterization bugs, and local minima still relies on understanding the mathematics underneath.

---

## 3.9 Advanced: Bayes Filter

Source: Thrun, Burgard, Fox (2005) *Probabilistic Robotics*, Ch.2 (Recursive State Estimation).

The information-theoretic metrics of §3.8 tell us how to *measure* uncertainty. The next question is how to *update* that uncertainty sequentially as controls are executed and new sensor readings arrive over time. Applying the Bayes' rule of §3.3.2 recursively along the time axis yields the general framework that answers "where is the robot right now?" Unlike the factor graph of §3.6—which retains the full trajectory of past poses and optimizes them jointly—the **Bayes filter** marginalizes out past states at every step so that only the current belief distribution needs to be maintained.

Estimating a robot's current state is a **recursive estimation** problem because each new belief builds directly on the prior belief and the latest control and measurement inputs. The Bayes filter expresses this recursion in its purest form, and both Gaussian filters (§3.10) and nonparametric filters (§3.11) are concrete realizations of it under different approximations.

### 3.9.1 State and the Markov Assumption

The **state** $x_t$ collects all variables required to predict how the robot and its environment will evolve next. When $x_t$ forms a complete sufficient statistic—meaning no older observations or controls add predictive power once $x_t$ is known—the system satisfies the **Markov property**:

$$p(x_{t+1} \mid x_t,\, x_{0:t-1},\, z_{1:t},\, u_{1:t}) = p(x_{t+1} \mid x_t)$$

Under this complete-state assumption, the future depends solely on the present state $x_t$, rendering the earlier history conditionally irrelevant.

In robotics, state variables span multiple categories. Along the time axis, states can be dynamic (robot pose, velocity) or static (wall geometry, landmark coordinates); by value type, they can be continuous (3D pose), discrete (sensor failure flags), or hybrid combinations of both. In practice, a truly complete state is unattainable, so every deployed filter operates on an approximation. Violations of the Markov assumption typically stem from unmodeled dynamics, calibration errors, and discretization or linearization approximations.

### 3.9.2 Environment Interaction: Measurements and Controls

A robot interacts with its environment through two distinct streams of data:

- **Measurement data** $z_t$: observations delivered by the environment to the robot (such as LiDAR scans or camera images), which sharpen knowledge of the state over the interval $(t-1, t]$.
- **Control data** $u_t$: actions commanded by the robot onto the environment (such as wheel velocities or motor torques); predicting state evolution under noisy motion typically increases uncertainty.

$$z_{t_1:t_2} = z_{t_1},\, z_{t_1+1},\, \ldots,\, z_{t_2} \qquad u_{t_1:t_2} = u_{t_1},\, \ldots,\, u_{t_2}$$

Notably, **odometry is treated as control data**. Although wheel encoders are physically sensors, they measure incremental state change (how far the robot moved) rather than absolute environmental landmarks, so filtering treats them as $u_t$. Even a do-nothing command counts as a control input, since the mere passage of time allows the state to evolve.

### 3.9.3 Defining Belief

Because the true state $x_t$ cannot be observed directly, the robot maintains an internal posterior probability distribution over it, called its **belief**:

$$\text{bel}(x_t) = p(x_t \mid z_{1:t},\, u_{1:t})$$

It is useful to separate the intermediate **predicted belief** computed *before* incorporating the latest measurement $z_t$:

$$\overline{\text{bel}}(x_t) = p(x_t \mid z_{1:t-1},\, u_{1:t})$$

Turning $\overline{\text{bel}}(x_t)$ into $\text{bel}(x_t)$ is the **correction** (or **measurement update**) step. Even a GPS receiver does not hand the robot its true state directly—every state estimate remains an indirect probabilistic inference built from these two alternating beliefs.

### 3.9.4 Generative Laws: Motion Model and Measurement Model

The complete-state (Markov) assumption implies two fundamental conditional independences that govern how states evolve and generate observations:

$$p(x_t \mid x_{0:t-1},\, z_{1:t-1},\, u_{1:t}) = p(x_t \mid x_{t-1},\, u_t) \quad \text{(motion model)}$$

$$p(z_t \mid x_{0:t},\, z_{1:t-1},\, u_{1:t}) = p(z_t \mid x_t) \quad \text{(measurement model)}$$

Because $x_{t-1}$ summarizes all earlier history, the next state depends only on $x_{t-1}$ and the current control $u_t$, while the current measurement $z_t$ depends only on the current state $x_t$. Under time-invariance, these models simplify to $p(x' \mid u, x)$ and $p(z \mid x)$. Together with an initial prior $p(x_0)$, they define a complete generative model structured as a Hidden Markov Model or Dynamic Bayes Network.

### 3.9.5 The General Bayes Filter

Combining the motion and measurement models yields the **general Bayes filter**, which recursively tracks a dynamic state by alternating a **prediction** step (propagating the previous belief forward through motion) and a **correction** step (reweighting the prediction by the latest measurement likelihood):

$$\overline{\text{bel}}(x_t) = \int p(x_t \mid u_t,\, x_{t-1})\, \text{bel}(x_{t-1})\, dx_{t-1} \tag{prediction}$$

$$\text{bel}(x_t) = \eta\, p(z_t \mid x_t)\, \overline{\text{bel}}(x_t) \tag{correction}$$

Here $\eta$ is the normalization constant ensuring the posterior integrates to 1, corresponding to $1/P(B)$ in the Bayes' rule formula of §3.3.2. Written as an abstract recursive procedure, one filter cycle looks like this:

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

In discrete state spaces, the integral over $x_{t-1}$ becomes a summation. Every recursion begins from an initial belief $\text{bel}(x_0)$—initialized to a point mass if the starting state is known exactly, or a uniform distribution if it is unknown. Because the prediction integral rarely admits a closed-form solution in continuous high-dimensional spaces, the general Bayes filter serves primarily as a mathematical blueprint; the Kalman filter (§3.10) and particle filter (§3.11) make it computationally tractable through different approximations.

### 3.9.6 Door Estimation: A Worked Example

To see the prediction–correction cycle in action without calculus, consider a simple two-state world where a door is either `open` or `closed`, and a robot estimates its state over two steps.

**Model setup:**

- Measurement model: $p(\text{sense\_open} \mid \text{is\_open}) = 0.6$, $p(\text{sense\_open} \mid \text{is\_closed}) = 0.2$
- Motion model push: if already open stays open (probability 1); if closed opens with probability 0.8
- Motion model do_nothing: deterministic identity (state unchanged)
- Initial: $\text{bel}(X_0 = \text{open}) = \text{bel}(X_0 = \text{closed}) = 0.5$

**Step 1: $u_1$ = do_nothing (apply control)**

Because `do_nothing` leaves the door untouched, the prediction step preserves the initial belief: $\overline{\text{bel}}(X_1) = (0.5,\; 0.5)$.

**Step 2: $z_1$ = sense_open (incorporate measurement)**

Next, the sensor reports `sense_open`. Multiplying each hypothesis's prior by its measurement likelihood gives:

$$\overline{\text{bel}}(X_1 = \text{open}) = 0.5, \quad p(\text{sense\_open} \mid \text{open}) = 0.6$$
$$\overline{\text{bel}}(X_1 = \text{closed}) = 0.5, \quad p(\text{sense\_open} \mid \text{closed}) = 0.2$$

This yields an unnormalized posterior of $(0.6 \times 0.5,\; 0.2 \times 0.5) = (0.30,\; 0.10)$, so normalizing by $\eta = 1/(0.30 + 0.10) = 2.5$ updates the belief to:

$$\text{bel}(X_1 = \text{open}) = 0.75, \quad \text{bel}(X_1 = \text{closed}) = 0.25$$

**Step 3: $u_2$ = push (apply control)**

The robot now pushes the door. Summing over both possible previous states weighted by the `push` transition probabilities produces the new predicted belief:

$$\overline{\text{bel}}(X_2 = \text{open}) = 1 \cdot 0.75 + 0.8 \cdot 0.25 = 0.95$$
$$\overline{\text{bel}}(X_2 = \text{closed}) = 0 \cdot 0.75 + 0.2 \cdot 0.25 = 0.05$$

**Step 4: $z_2$ = sense_open (incorporate measurement)**

Finally, a second `sense_open` reading arrives. Reweighting $(0.95,\; 0.05)$ gives unnormalized values $(0.6 \times 0.95,\; 0.2 \times 0.05) = (0.570,\; 0.010)$, with $\eta = 1/0.580 \approx 1.724$:

$$\text{bel}(X_2 = \text{open}) \approx 0.983, \quad \text{bel}(X_2 = \text{closed}) \approx 0.017$$

| Step | $\text{bel(open)}$ | $\text{bel(closed)}$ |
|------|:-----------------:|:-------------------:|
| Initial | 0.500 | 0.500 |
| After $z_1$ | 0.750 | 0.250 |
| After $u_2$ | 0.950 | 0.050 |
| After $z_2$ | 0.983 | 0.017 |

Even with a noisy sensor (60% true positive vs. 20% false positive) and a stochastic actuator, combining controls and observations concentrates probability mass rapidly onto the true state. Whether 0.983 is confident enough to trigger an autonomous action, however, depends on the task's risk threshold rather than the filter equations themselves.

### 3.9.7 Mathematical Derivation

Both update equations of the Bayes filter follow directly from three elementary tools: Bayes' rule, the law of total probability, and the Markov (complete-state) assumption.

**Correction step derivation:**

Applying Bayes' rule to the posterior $p(x_t \mid z_{1:t},\, u_{1:t})$ separates the latest measurement $z_t$ from earlier history:
$$p(x_t \mid z_{1:t},\, u_{1:t}) = \eta\, p(z_t \mid x_t,\, z_{1:t-1},\, u_{1:t})\, p(x_t \mid z_{1:t-1},\, u_{1:t})$$

Invoking the complete-state assumption simplifies the measurement likelihood to $p(z_t \mid x_t,\, z_{1:t-1},\, u_{1:t}) = p(z_t \mid x_t)$, yielding the correction step:
$$\text{bel}(x_t) = \eta\, p(z_t \mid x_t)\, \overline{\text{bel}}(x_t)$$

**Prediction step derivation:**

For the prediction step, expanding $\overline{\text{bel}}(x_t)$ by marginalizing over the previous state $x_{t-1}$ via the law of total probability gives:
$$\overline{\text{bel}}(x_t) = \int p(x_t \mid x_{t-1},\, z_{1:t-1},\, u_{1:t})\, p(x_{t-1} \mid z_{1:t-1},\, u_{1:t})\, dx_{t-1}$$

Under the complete-state assumption, the first factor reduces to $p(x_t \mid x_{t-1},\, u_t)$. In the second factor, the future control $u_t$ carries no information about the earlier state $x_{t-1}$ and drops out, leaving $\text{bel}(x_{t-1})$:
$$\overline{\text{bel}}(x_t) = \int p(x_t \mid u_t,\, x_{t-1})\, \text{bel}(x_{t-1})\, dx_{t-1}$$

Because the entire derivation hinges on the Markov assumption, any unmodeled dependency across time directly degrades the filter's accuracy. Seeing exactly where Bayes' rule, total probability, and the Markov property enter §3.9.5 makes it clear why—and where—practical filters break down.

> **Further reading**
> - [Thrun, Burgard, Fox — Probabilistic Robotics (2005)](http://probabilistic-robotics.org/) — Ch.2 is the primary source for this section, presenting the algorithm, examples, and full derivation.
> - [Cyrill Stachniss — Bayes Filter Lecture (YouTube)](https://www.youtube.com/watch?v=0lKHFJpaZkI) — Freiburg lecture offering a clear slide-based walkthrough of the Bayes filter.

---

## 3.10 Advanced: Gaussian Filters (KF, EKF, IF)

Source: Thrun, Burgard, Fox (2005) *Probabilistic Robotics*, Ch.3 (Gaussian Filters).

While the Bayes filter of §3.9 accommodates arbitrary distributions, its continuous integrals rarely evaluate in closed form. **Gaussian filters** overcome this by restricting belief at every time step to a multivariate Gaussian $\mathcal{N}(\mu_t, \Sigma_t)$. The Kalman filter (KF), extended Kalman filter (EKF), and information filter (IF) all live within this family, preserving the exact prediction–correction structure of §3.9 while replacing general integrals with matrix algebra.

### 3.10.1 Kalman Filter

#### Linear Gaussian System Assumptions

For the Bayes recursion to preserve Gaussianity exactly at every step, the system must satisfy three **linear-Gaussian assumptions**:

**State transition (motion model):**
$$x_t = A_t x_{t-1} + B_t u_t + \varepsilon_t, \quad \varepsilon_t \sim \mathcal{N}(0, R_t)$$

Here $A_t$ is the $n \times n$ state-transition matrix, $B_t$ is the $n \times m$ control-input matrix, and $R_t$ is the process-noise covariance.

**Measurement model:**
$$z_t = C_t x_t + \delta_t, \quad \delta_t \sim \mathcal{N}(0, Q_t)$$

Here $C_t$ is the $k \times n$ measurement matrix and $Q_t$ is the measurement-noise covariance.

**Initial belief:**
$$\text{bel}(x_0) = \mathcal{N}(\mu_0, \Sigma_0)$$

Under these three conditions, both transition and observation densities are Gaussian:
$$p(x_t \mid u_t, x_{t-1}) = \mathcal{N}(x_t;\; A_t x_{t-1} + B_t u_t,\; R_t)$$
$$p(z_t \mid x_t) = \mathcal{N}(z_t;\; C_t x_t,\; Q_t)$$

and by Gaussian closure, $\text{bel}(x_t)$ remains strictly Gaussian forever.

#### Kalman Filter Algorithm

Because a Gaussian is completely specified by its mean and covariance $(\mu_t, \Sigma_t)$, the Kalman filter executes one full Bayes recursion in just five matrix equations—two lines for prediction and three for measurement correction:

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

Lines 1–2 propagate the prior through the motion model, expanding covariance via $R_t$. Lines 3–5 fuse the incoming measurement $z_t$ to contract covariance and shift the mean:
- **Kalman gain $K_t$**: Acts as the optimal weighting factor between prediction and observation. High measurement noise $Q_t$ shrinks $K_t$ so the filter discounts noisy sensors, whereas large prediction uncertainty $\bar\Sigma_t$ increases $K_t$ so the filter leans more heavily on the new measurement.
- **Innovation**: The residual $z_t - C_t \bar\mu_t$ measures the discrepancy between the actual reading and what the predicted state expected. Even when the innovation happens to be zero (leaving the mean unchanged), fusing a new measurement still shrinks the posterior covariance $\Sigma_t$.

#### 1D KF Illustration: How Information Combines

Tracking a robot's 1D position across one cycle illustrates the geometry behind these five equations:

- **Prior $\text{bel}(x_{t-1})$**: Starts as a relatively narrow Gaussian representing confidence from the previous step.
- **After prediction**: Applying motion and adding process noise ($\bar\Sigma_t = A_t \Sigma_{t-1} A_t^T + R_t$) shifts and flattens the Gaussian.
- **Measurement $z_t$**: Arrives as a separate Gaussian centered on the sensor reading, with width governed by $Q_t$.
- **After correction**: Multiplying the predicted and measurement Gaussians yields a posterior curve narrower than *either* input alone, with its mean at their precision-weighted average.
- As the robot continues moving and sensing, motion repeatedly widens variance while measurements repeatedly narrow it.

In short, **motion grows uncertainty while measurements shrink it**, and their alternation forms the core estimation loop. The same intuition carries over to the EKF (§3.10.2), the particle filter (§3.11.3), EKF localization in Ch.14 §14.7, and (at the factor level) IMU preintegration in §14.10.

<!-- DEMO: kalman_1d_illustration.html -->

#### Mathematical Derivation of the KF (Key Steps)

The five Kalman filter equations are the exact closed-form evaluation of the Bayes filter integrals (§3.9.5) under linear-Gaussian assumptions:

**Part 1 (Prediction).** In the prediction integral, the combined exponent $L_t$ is quadratic in both $x_{t-1}$ and $x_t$. Completing the square to isolate the terms quadratic in $x_{t-1}$ allows the integral over $x_{t-1}$ to evaluate to a constant absorbed into $\eta$. Reading off the linear and quadratic coefficients in the remaining $x_t$ exponent directly recovers $\bar\mu_t = A_t \mu_{t-1} + B_t u_t$ and $\bar\Sigma_t = A_t \Sigma_{t-1} A_t^T + R_t$.

**Part 2 (Measurement update).** In the correction step, multiplying two Gaussians gives $\text{bel}(x_t) \propto \exp\{-J_t\}$, where differentiating $J_t$ yields the posterior precision $\Sigma_t^{-1} = C_t^T Q_t^{-1} C_t + \bar\Sigma_t^{-1}$. Inverting an $n \times n$ matrix directly scales poorly in high-dimensional states, so applying the **matrix inversion lemma** (Woodbury identity) rewrites the gain as:
$$K_t = \bar\Sigma_t C_t^T (C_t \bar\Sigma_t C_t^T + Q_t)^{-1}$$
which requires inverting only a $k \times k$ matrix in measurement space—saving substantial compute whenever $k \ll n$.

**Complexity:** Each cycle runs in $O(k^{2.8} + n^2)$ operations, where $k$ is the measurement dimension and $n$ is the state dimension. Notice how this same two-step pattern—completing the square followed by the Woodbury identity—reappears in the EKF (§3.10.2) and underlies Gauss-Newton updates in factor graphs (§3.6).

Because the five-line KF depends strictly on linear models, handling real rotating robots and range sensors requires relaxing linearity—leading to the EKF in §3.10.2.

### 3.10.2 Extended Kalman Filter (EKF)

#### Extension to Nonlinear Systems

Physical robots rarely obey linear equations: rotating while translating introduces trigonometric motion models $g$, and range-bearing or camera projections produce nonlinear measurement models $h$:

$$x_t = g(u_t, x_{t-1}) + \varepsilon_t, \quad \varepsilon_t \sim \mathcal{N}(0, R_t)$$
$$z_t = h(x_t) + \delta_t, \quad \delta_t \sim \mathcal{N}(0, Q_t)$$

Passing a Gaussian through a nonlinear function generally produces a skewed, non-Gaussian distribution. The **extended Kalman filter (EKF)** keeps the distribution Gaussian by taking a **first-order Taylor expansion** around the current mean—linearizing $g$ around $\mu_{t-1}$ and $h$ around $\bar\mu_t$:

$$g(u_t, x_{t-1}) \approx g(u_t, \mu_{t-1}) + G_t (x_{t-1} - \mu_{t-1})$$

$$G_t := \frac{\partial g(u_t, x_{t-1})}{\partial x_{t-1}}\bigg|_{\mu_{t-1}} \quad (n \times n \text{ Jacobian})$$

$$h(x_t) \approx h(\bar\mu_t) + H_t (x_t - \bar\mu_t)$$

$$H_t := \frac{\partial h(x_t)}{\partial x_t}\bigg|_{\bar\mu_t} \quad (k \times n \text{ Jacobian})$$

How accurate this tangent-plane approximation stays depends on two factors: the curvature (nonlinearity) of $g$ and $h$, and the spread of the belief. When covariance is small, the local linear tangent fits well; when uncertainty grows wide, the linear approximation breaks down and the EKF can become overconfident or diverge.

#### EKF Algorithm

Once $g$ and $h$ are linearized locally, turning the KF into the EKF simply substitutes the nonlinear functions into the mean updates and their Jacobians $G_t, H_t$ into the covariance updates:

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

Comparing line by line against §3.10.1, only the mean predictions change from linear products to nonlinear evaluations ($A_t \mu_{t-1} + B_t u_t \to g(u_t, \mu_{t-1})$ in line 1, and $C_t \bar\mu_t \to h(\bar\mu_t)$ in line 4), while covariance updates swap $A_t \to G_t$ and $C_t \to H_t$.

#### Derivation Summary and Practical Comparison

Following the exact same quadratic-splitting and inversion-lemma recipe as §3.10.1 on the Taylor-linearized models yields:
$$\bar\mu_t = g(u_t, \mu_{t-1}), \quad \bar\Sigma_t = G_t \Sigma_{t-1} G_t^T + R_t$$
$$\mu_t = \bar\mu_t + K_t (z_t - h(\bar\mu_t)), \quad \Sigma_t = (I - K_t H_t) \bar\Sigma_t, \quad K_t = \bar\Sigma_t H_t^T (H_t \bar\Sigma_t H_t^T + Q_t)^{-1}$$

Although the EKF served as the workhorse of SLAM, VIO, and inertial fusion through the mid-2010s, several stronger variants address its first-order linearization limits:

- **UKF (Unscented KF):** Propagates a deterministic cloud of sigma points through the true nonlinear functions, capturing higher-order moments without needing hand-derived Jacobians—particularly effective when the state dimension is modest.
- **IEKF (Iterated EKF):** Re-linearizes $h$ iteratively around the updated estimate, improving accuracy when measurements are strongly nonlinear.
- **LIEKF (Left-Invariant EKF):** For states living on $SO(3)$ or $SE(3)$, linearizes errors directly on the Lie group rather than in flat coordinates, improving consistency and rotation accuracy.

Both EKF localization and VIO filters such as MSCKF and IEKF in Ch.14 §14.7 share this exact algorithmic skeleton. (By contrast, IMU preintegration in §14.10 is not a recursive filter, but a measurement-compression scheme that turns high-rate inertial readings between keyframes into a single factor-graph constraint.) Understanding $g$, $h$, $G_t$, and $H_t$ here lets you read specific sensor filters in Ch.14 directly without rederiving the framework from scratch.

### 3.10.3 Information Filter (IF)

#### Canonical Form: $(\Omega, \xi)$

Both the KF and EKF parameterize a Gaussian via its moments $(\mu, \Sigma)$. Writing the exact same distribution in **canonical (information) coordinates** flips the computational cost between prediction and measurement update—a duality that proves invaluable when fusing independent measurements across multiple sensors or robots.

Instead of mean and covariance, the **information filter (IF)** tracks the **information matrix** $\Omega$ (precision) and the **information vector** $\xi$:

$$\Omega = \Sigma^{-1}, \quad \xi = \Sigma^{-1} \mu$$

Converting back to moments simply inverts $\Omega$: $\Sigma = \Omega^{-1}$ and $\mu = \Omega^{-1} \xi$.

In canonical coordinates, the negative log-likelihood of the Gaussian becomes a clean quadratic form in the state $x$:

$$p(x) = \eta \exp\!\left\{-\tfrac{1}{2} x^T \Omega x + x^T \xi\right\}$$

$$-\log p(x) = \mathrm{const} + \tfrac{1}{2} x^T \Omega x - x^T \xi$$

Setting the derivative to zero locates the mean at $\Omega x = \xi$, or $x = \Omega^{-1} \xi = \mu$—matching the exact algebraic structure of the Gauss-Newton normal equation $H \delta x = -b$ in §3.4.

**Intuition:** Whereas complete ignorance is awkward to express in covariance form ($\Sigma = \infty$), in information form it is simply $\Omega = 0$ (zero certainty). Information coordinates directly measure how much certainty has accumulated along each direction of the state space.

#### Information Filter Algorithm

As the mathematical dual of the Kalman filter, the information filter swaps which step is easy: where the KF's prediction step was simple addition, the IF's **measurement update is purely additive**, while prediction incurs matrix inversions:

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

This creates a clean computational duality between the two Gaussian representations:

| Step | KF | IF |
|------|:---:|:---:|
| Prediction | $O(n^2)$ additive | $O(n^{2.8})$, two inversions |
| Measurement update | $O(k^{2.8})$, inversion needed | $O(n^2)$ additive |

Here $k$ is the measurement dimension and $n$ is the state dimension. Whenever a sensor observes only a small subset of variables (making $C_t$ sparse), the IF measurement update touches only those corresponding entries.

#### EIF (Extended Information Filter)

Just as the EKF generalizes the KF to nonlinear systems, replacing the linear maps $g, h$ with their Jacobians $G_t, H_t$ yields the **extended information filter (EIF)**:

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

Because each measurement update $\Omega_t = \bar\Omega_t + C_t^T Q_t^{-1} C_t$ simply adds a symmetric term to $\Omega$, independent observations from multiple sensors or robots fuse cleanly by summation: $\Omega_{\text{total}} = \sum_i \Omega_i$ and $\xi_{\text{total}} = \sum_i \xi_i$.

This exact additivity reappears in the factor graphs of §3.6: each factor contributes one $H^T Q^{-1} H$ block to the Hessian/information matrix and one $H^T Q^{-1} z$ term to the right-hand side. Note, however, that additivity alone does not make $\Omega$ sparse—*locality* does. Because each SLAM factor links only a handful of poses or landmarks, $H$ has very few non-zero columns, so the accumulated outer products $H^T Q^{-1} H$ remain sparse; preserving that sparsity during solving then depends on choosing a low-fill-in variable elimination order (§3.6.3). Systems such as EIF-SLAM and SEIF directly exploit this canonical structure, whose historical evolution we revisit in Ch.14 §14.16.

> **Further reading**
> - [Thrun, Burgard, Fox — Probabilistic Robotics (2005)](http://probabilistic-robotics.org/) — Ch.3 is the primary reference, presenting KF, EKF, and IF derivations side by side.
> - [Cyrill Stachniss — Kalman Filter and EKF Lectures](https://www.youtube.com/watch?v=PiCC-SxWlH8) — Freiburg University lectures with strong geometric visualizations.
> - [Welch & Bishop — An Introduction to the Kalman Filter (2006)](https://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf) — Classic introductory monograph balancing derivation and intuition.

---

## 3.11 Advanced: Nonparametric Filters

Source: Thrun, Burgard, Fox (2005) *Probabilistic Robotics*, Ch.4 (Nonparametric Filters).

Gaussian filters (§3.10) gain speed and elegance by compressing belief into two quantities $(\mu, \Sigma)$, but that single-peak assumption fails whenever a robot faces global localization ambiguity ("am I in hallway A or identical hallway B?") or severe nonlinearities. **Nonparametric filters** drop the Gaussian restriction altogether to represent arbitrary, multi-modal distributions—trading closed-form simplicity for representational flexibility and higher compute cost.

### 3.11.1 Histogram Filter / Discrete Bayes Filter

#### Finite State Spaces: Integral Becomes Summation

When the continuous Bayes filter integral of §3.9.5 cannot be evaluated analytically, the most direct approach is to discretize the state space into $K$ distinct states $\{x_1, x_2, \ldots, x_K\}$. The prediction integral then turns into a finite summation over prior states:

$$\bar p_{k,t} = \sum_i p(x_k \mid u_t, x_i)\, p_{i,t-1} \quad \text{(prediction)}$$
$$p_{k,t} = \eta\, p(z_t \mid x_k)\, \bar p_{k,t} \quad \text{(correction)}$$

Written in algorithmic form, the **discrete Bayes filter** loops directly over every discrete hypothesis:

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

This matches the forward algorithm for Hidden Markov Models in speech recognition, and remains the cleanest solution when a state is intrinsically discrete (such as door status or semantic class).

#### Continuous State: Histogram Filter

To apply the same discrete machinery to a continuous state space, we partition the domain into a finite grid of regions $\{\mathbf{x}_{k,t}\}$ and approximate the belief as piecewise uniform inside each cell:

$$p(x_t) = \frac{p_{k,t}}{|\mathbf{x}_{k,t}|} \quad x_t \in \mathbf{x}_{k,t}$$

Evaluating the motion and measurement models at a single representative point (such as the cell centroid $\hat x_{k,t}$) gives:

$$p(z_t \mid \mathbf{x}_{k,t}) \approx p(z_t \mid \hat x_{k,t})$$
$$p(\mathbf{x}_{k,t} \mid u_t, \mathbf{x}_{i,t-1}) \approx \eta\,|\mathbf{x}_{k,t}|\, p(\hat x_{k,t} \mid u_t, \hat x_{i,t-1})$$

When every cell shares the same volume, $|\mathbf{x}_{k,t}|$ is absorbed into the normalization constant $\eta$, yielding the **histogram filter**.

**Limitations:** Because the number of grid cells grows exponentially with dimension, the curse of dimensionality makes uniform histogram filters impractical above roughly five dimensions—ruling them out for full 6-DoF pose tracking. Common workarounds include adaptive density trees (splitting cells finely only where probability mass concentrates), selective updating (recomputing only cells that change significantly), and hybrid topological-metric maps. In Ch.14, occupancy grid mapping builds directly on this discrete-cell formulation.

### 3.11.2 Binary Bayes Filter (Log-Odds Form)

#### Binary Estimation of a Static State

Many mapping problems estimate a static binary variable—most famously, "is this grid cell occupied or free?" Because the world cell does not move on its own, there is no motion model and the prediction step disappears, leaving only repeated measurement updates.

Multiplying raw probabilities $p(x \mid z_{1:t})$ over hundreds of scans invites floating-point underflow near 0 or 1 and requires awkward clamping. Switching to **log-odds** solves both problems cleanly:

$$l(x) := \log \frac{p(x)}{1 - p(x)} \in (-\infty, +\infty)$$

Because log-odds maps $[0, 1]$ smoothly onto the entire real line $(-\infty, +\infty)$, probabilities never underflow or clip, and multiplicative Bayesian updates turn into simple **additions**:

$$l_t = l_{t-1} + \log\frac{p(x \mid z_t)}{1 - p(x \mid z_t)} - l_0$$

where $l_0 = \log\frac{p(x)}{1-p(x)}$ is the prior log-odds (subtracted so the prior is not counted twice). Whenever an actual probability is needed, it is recovered via the sigmoid inverse: $\text{bel}_t(x) = 1 - \dfrac{1}{1 + \exp(l_t)}$.

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

Notice that this update uses the **inverse sensor model** $p(x \mid z)$—mapping from observation back to state—rather than the forward measurement model $p(z \mid x)$. When mapping with range finders or cameras, it is often much easier to specify the probability that a cell along a ray is occupied or free given a hit than to model the entire 3D sensor scan forward. Occupancy grid mapping in Ch.14 applies this exact additive log-odds loop independently to every cell in the map.

### 3.11.3 Particle Filter

#### Principle of Nonparametric Representation

While histogram filters allocate grid cells uniformly across the entire state space—even in empty regions with near-zero probability—the **particle filter** avoids the curse of dimensionality by concentrating computation only where probability mass actually lives. It approximates the belief using a finite set of $M$ random state samples (**particles**):

$$\mathcal{X}_t = \{x_t^{[1]},\, x_t^{[2]},\, \ldots,\, x_t^{[M]}\}$$

Regions of high belief attract dense clusters of particles, allowing the particle set to capture skewed, heavy-tailed, or multi-modal distributions without any Gaussian assumption.

#### Particle Filter Algorithm: Sampling, Weighting, Resampling

Each iteration of the particle filter advances the particle cloud through three stages—propagating particles via the motion model (**sampling**), scoring each particle against the observation (**weighting**), and duplicating high-scoring particles while pruning unlikely ones (**resampling**):

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

As the particle count grows ($M \to \infty$), the resampled distribution converges to the exact Bayesian posterior $p(x_t \mid z_{1:t}, u_{1:t})$.

#### Importance Sampling Intuition

Why does multiplying by $p(z_t \mid x_t^{[m]})$ turn samples from the motion model into samples from the posterior? The answer comes from **importance sampling**. When sampling directly from a target distribution $f$ is intractable, we draw samples from an easier **proposal distribution** $g$ instead and reweight each sample by the ratio $w = f/g$:

$$w^{[m]} = \frac{f(x^{[m]})}{g(x^{[m]})}$$

Under this correction, the weighted empirical distribution converges to $f$ over any measurable (Borel) region $A$:
$$\left[\sum_{m=1}^M w^{[m]}\right]^{-1} \sum_{m=1}^M \mathbf{1}(x^{[m]} \in A)\, w^{[m]} \;\longrightarrow\; \int_A f(x)\, dx$$

at a Monte Carlo rate of $O(1/\sqrt{M})$—with a smaller constant whenever the proposal $g$ closely resembles the target $f$.

In the particle filter, Step 1 draws particles from a proposal governed by the motion model $p(x_t \mid u_t, x_{t-1})$, which has not yet seen the measurement $z_t$, whereas the target is the full posterior $\text{bel}(x_t)$. The missing factor separating the unconditioned proposal from the measurement-conditioned target is precisely the likelihood $p(z_t \mid x_t^{[m]})$, explaining the Step 2 weight formula intuitively. Making that argument airtight, however, requires viewing particles over full state trajectories rather than single time slices.

#### Convergence and Implementation

In the rigorous derivation, each particle represents an entire state sequence $x_{0:t}^{[m]}$ rather than a single instant $x_t^{[m]}$. Factorizing the target sequence posterior via two applications of Bayes' rule and the Markov property, and comparing it against the inductively factorized motion proposal, cancels all past terms so that the importance ratio collapses to $\eta\, p(z_t \mid x_t^{[m]})$—an equality that becomes exact as $M \to \infty$.

Without Step 3 (resampling), repeated weighting causes probability mass to collapse onto a handful of lucky particles—a failure mode called **weight degeneracy**. Resampling refreshes the cloud by focusing particles on high-likelihood regions, though finite-$M$ sampling introduces its own structural pitfalls, examined next in §3.11.4.

<!-- DEMO: particle_filter_1d.html -->

### 3.11.4 Four Sources of Error in Particle Filters

Because a particle filter approximates a continuous density with a finite sample set, it introduces structural approximation errors. Resampling acts as survival-of-the-fittest selection—favoring high-weight particles and pruning low-weight ones—which creates trade-offs of its own. Knowing the four primary failure modes makes diagnosing particle-filter behavior systematic.

#### (1) Systematic Bias from Finite $M$

Consider the extreme case $M = 1$: a single particle's weight always normalizes to $w/w = 1$, causing the filter to ignore measurements completely. Even for larger finite $M$, normalizing weights confines them to an $(M-1)$-dimensional simplex, injecting systematic variance and bias that shrinks only as $M$ grows at linear computational cost.

#### (2) Sample Impoverishment from Resampling

In a static state ($x_t = x_{t-1}$) with zero transition noise, particles never move on their own. Repeating resampling step after step eliminates distinct particles through pure sampling variance—known as **sample impoverishment**—until every particle collapses onto a single identical copy.

To prevent this collapse, skip resampling while the robot is stationary, or resample adaptively only when weight variance exceeds a threshold, accumulating weights multiplicatively between resamples:

$$w_t^{[m]} = \begin{cases} 1 & \text{(immediately after resampling)} \\ p(z_t \mid x_t^{[m]})\, w_{t-1}^{[m]} & \text{(when not resampling)} \end{cases}$$

#### (3) Proposal-Target Divergence

When sensors are far more accurate than the motion model, the true posterior (target) becomes razor-thin while the motion-predicted particles (proposal) scatter widely, causing efficiency to collapse. With an ultra-low-noise range sensor, for instance, $p(z \mid x)$ concentrates along a thin manifold and assigns nearly every particle weight $\approx 0$.

Practical mitigations include deliberately inflating the sensor noise covariance during weighting (trading slight precision for particle survival) or designing a measurement-aware proposal that steers sampling toward high-likelihood regions.

#### (4) Particle Deprivation

In high-dimensional state spaces, finite sampling may leave zero particles near the true state—and random resampling always carries a small risk of deleting the last surviving particle in the right mode. Once lost, the filter cannot recover on its own.

To guard against catastrophic deprivation, inject a small fraction of **random particles** drawn from a global prior at each cycle; this introduces slight posterior bias in exchange for reliable recovery from kidnapping or particle loss.

#### Low-Variance Sampler

Naive independent multinomial resampling costs $O(M \log M)$ and injects high sampling variance. In practice, filters use the **low-variance (systematic) sampler**, which draws a single random offset $r \in [0, 1/M)$ and steps uniformly around the cumulative weight circle in $O(M)$ time:

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

A key property of systematic sampling is zero distortion under uniform weights: when all particles have equal weight (such as during steps without measurements), the output set equals the input set identically with zero particles lost, all in $O(M)$ rather than $O(M \log M)$ time.

With both the convergence theory and these four failure modes in hand, tuning a particle filter becomes a principled engineering trade-off. In Ch.14 §14.7, augmented MCL and mixture MCL apply these exact mitigations to production mobile-robot localization.

> **Further reading**
> - [Thrun, Burgard, Fox — Probabilistic Robotics (2005)](http://probabilistic-robotics.org/) — Ch.4 is the primary source, covering histogram, binary Bayes, and particle filters in full.
> - [Arulampalam et al. — A Tutorial on Particle Filters (IEEE Trans. Signal Processing 2002)](https://ieeexplore.ieee.org/document/978374) — Standard tutorial on sequential Monte Carlo methods and particle filter variants.
> - [Thrun — Particle Filters in Robotics (UAI 2002)](https://www.aaai.org/Papers/UAI/2002/UAI02-079.pdf) — Concise paper linking particle filters to Rao-Blackwellized particle filters and FastSLAM.
> - [ROS AMCL package](https://wiki.ros.org/amcl) — Production localization package implementing the theory of §3.11.3–3.11.4, including augmented MCL and the low-variance sampler.

---

Taken together, the state estimation methods in the second half of this chapter form a single family tree: the **Bayes filter** provides the overarching recursive framework; the **KF, EKF, and IF** evaluate it in closed form under Gaussian assumptions; and **histogram and particle filters** trade extra compute for the ability to represent arbitrary multi-modal distributions. Choosing among them comes down to state dimensionality, nonlinearity, and whether multiple pose hypotheses must be tracked simultaneously.

Every tool built in this chapter now feeds directly into the chapters ahead. Starting immediately in **Ch.4 (Kinematics & Mechatronics)**, the 4×4 homogeneous transformations, rotation matrices, and Jacobians from §3.1–3.4 become the language for chaining robot manipulator links and mapping joint velocities to end-effector motion. Later, when you reach **Ch.14** for EKF localization (§14.7) and Monte Carlo localization (§14.7), you will not need to rederive each filter from scratch—recognizing $g$, $h$, and the proposal distribution makes every algorithm's structure transparent (while IMU preintegration in §14.10 compresses inertial streams into the optimization factors of §3.6).
