# Ch.4 — Kinematics & Mechatronics


Ch.3 introduced the mathematical machinery of coordinate frames, rigid-body transformations, Lie groups, and probabilistic state estimation. This chapter brings that machinery onto physical hardware. Consider a six-axis robot arm resting on a desk: calculating the exact angle each motor must hold for the fingertip to reach a coffee cup is the domain of **kinematics**, while actually driving those motors, reading sensors, and closing a deterministic control loop at 1 kHz is **mechatronics**.

From rigid-body equations to actuator sizing, fieldbus protocols, and real-time software, the sections that follow trace how geometry becomes motion on a real robot—and then extend that deterministic geometry into the probabilistic motion models used in mobile localization.

---

## 4.1 Why Study Kinematics

A robot manipulator consists of rigid links connected by actuated joints. In practice, tasks are specified in Cartesian space—the position and orientation (pose) of the end-effector—whereas the controller only commands individual joint angles or linear displacements. **Kinematics** is the mathematical bridge between these two representations, split into two complementary problems:

- **Forward Kinematics (FK)**: given the joint angles, determine the end-effector pose.
- **Inverse Kinematics (IK)**: given a desired end-effector pose, find the joint angles that achieve it.

Unlike dynamics, kinematics ignores masses, inertias, and forces. It asks *"where is the mechanism?"* rather than *"what torques are needed to move it?"*—a physical layer that Ch.5 takes up next.

Even without dynamics, kinematics underpins much of practical robotics:
- **Motion planning**: generating collision-free Cartesian or joint-space trajectories for an arm.
- **Teleoperation**: mapping a human operator's hand or leader-arm pose onto a follower robot in real time.
- **Calibration**: identifying and correcting geometric discrepancies between the nominal CAD model and the assembled machine.
- **Collision avoidance**: evaluating where every link occupies 3D space so the arm clears both obstacles and itself.

---

## 4.2 Forward Kinematics

### 4.2.1 Homogeneous Transformation Matrix

Because a manipulator is a serial chain of rigid bodies, tracking where each link sits in space requires bundling rotation and translation into a single algebraic object. Drawing on the $SE(3)$ transformations from Ch.3, kinematics relies on the 4×4 homogeneous transformation matrix:

```
T = | R  p |
    | 0  1 |
```

Here, `R` is a 3×3 rotation matrix describing orientation and `p` is a 3×1 position vector describing translation. Packing both into a 4×4 matrix turns frame composition into ordinary matrix multiplication: if `T_01` expresses frame 1 relative to frame 0 and `T_12` expresses frame 2 relative to frame 1, their product gives frame 2 directly in frame 0:

```
T_02 = T_01 * T_12
```

Forward kinematics applies this chain rule from the robot's base through every link out to the end-effector, multiplying each joint transformation in sequence.


### 4.2.2 DH Parameters (Denavit-Hartenberg)

An arbitrary rigid-body transformation in 3D has six degrees of freedom, yet a standard single-axis robot joint introduces only one variable. In 1955, Denavit and Hartenberg proposed a systematic frame-assignment convention that reduces the relative transform between any two adjacent links to four scalar parameters, summarized below:

| Parameter | Meaning |
|---------|------|
| **a_i** (link length) | distance from z_{i-1} to z_i along the x_i axis |
| **α_i** (link twist) | rotation angle from z_{i-1} to z_i about the x_i axis |
| **d_i** (link offset) | distance from x_{i-1} to x_i along the z_{i-1} axis |
| **θ_i** (joint angle) | rotation angle from x_{i-1} to x_i about the z_{i-1} axis |

For a revolute joint, `θ_i` acts as the joint variable while the other three parameters remain fixed geometric constants; for a prismatic joint, `d_i` varies instead. Composing the rotation and translation along `z_{i-1}` with the translation and rotation along `x_i` yields the single-joint transformation matrix:

```
T_i = Rot_z(θ_i) * Trans_z(d_i) * Trans_x(a_i) * Rot_x(α_i)

    = | cos(θ)  -sin(θ)cos(α)   sin(θ)sin(α)   a*cos(θ) |
      | sin(θ)   cos(θ)cos(α)  -cos(θ)sin(α)   a*sin(θ) |
      | 0        sin(α)          cos(α)          d        |
      | 0        0               0               1        |
```

One practical catch is that the DH convention exists in two distinct variants: **standard DH** and **modified DH (Craig's convention)**. Craig's textbook adopts modified DH, whereas many other references use standard DH. Because the two conventions attach link frames at different joints, mixing parameters from one with matrix formulas from the other produces incorrect kinematics—always verify and state which convention a model assumes.


### 4.2.3 Example: FK of a 2-link Planar Arm

To make these transformations concrete, consider the simplest nontrivial manipulator: a two-link arm moving in the plane.

```
       q1         q2
  O────────O────────O → end-effector
  (base)   L1       L2
```

Because both joint axes point out of the page in parallel (`α = 0`) and there is no out-of-plane offset (`d = 0`), the standard DH table simplifies to two rows:

| Link | a    | α   | d   | θ    |
|------|------|-----|-----|------|
| 1    | L1   | 0   | 0   | θ_1  |
| 2    | L2   | 0   | 0   | θ_2  |

Multiplying `T_1 * T_2` reproduces what elementary trigonometry already shows—the end-effector coordinates equal the vector sum of the two links:

```
x = L1*cos(θ_1) + L2*cos(θ_1 + θ_2)
y = L1*sin(θ_1) + L2*sin(θ_1 + θ_2)
```

In Python, `fk_2link` evaluates this position along with the cumulative end-effector orientation `phi`:

```python
import numpy as np

def fk_2link(theta1, theta2, L1=1.0, L2=1.0):
    """Forward kinematics of a 2-link planar arm."""
    x = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
    y = L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2)
    phi = theta1 + theta2  # absolute orientation of the end-effector
    return x, y, phi

# θ_1=30°, θ_2=45°, link lengths of 1m each
x, y, phi = fk_2link(np.radians(30), np.radians(45))
print(f"End-effector position: ({x:.3f}, {y:.3f}), orientation: {np.degrees(phi):.1f}°")
# Output: End-effector position: (1.125, 1.466), orientation: 75.0°
```

If that derivation feels almost too simple, that is reassuring: the forward kinematics of a full six-axis industrial arm follows the exact same principle, merely chaining six 4×4 matrices instead of two.


### 4.2.4 Product of Exponentials (PoE)

While DH parameters are compact, placing intermediate frames along common normals can feel arbitrary and error-prone when joint axes are skewed or parallel. An alternative formulation—the **Product of Exponentials (PoE)** method, rooted in Lie group and Lie algebra theory and popularized in Lynch and Park's *Modern Robotics*—eliminates intermediate link frames altogether.

PoE imagines the robot resting in a fixed zero (home) configuration `M`, then treats each joint as a screw motion (a twist) that rotates and translates the distal links via the matrix exponential:

```
T(θ) = e^{[S_1]θ_1} * e^{[S_2]θ_2} * ... * e^{[S_n]θ_n} * M
```

In this expression, each term has a direct geometric interpretation:
- `S_i` is the screw axis of the i-th joint (a 6×1 vector combining angular axis and linear velocity component).
- `[S_i]` is the 4×4 matrix representation of `S_i` (an element of `se(3)`, whose upper-left 3×3 block is skew-symmetric).
- `M` is the end-effector pose when all joints sit at their zero (home) configuration.
- `θ_i` is the joint variable.

The table below contrasts the practical trade-offs between the DH and PoE conventions:

**DH vs PoE comparison:**

| Item | DH | PoE |
|------|-----|-----|
| Frame attachment | a frame needed on each link | only the base frame and end-effector frame needed |
| Convention confusion | standard vs. modified variants | none (though space form vs. body form exists) |
| Mathematical basis | matrix multiplication | Lie group, matrix exponential |
| Singularity analysis | requires separate treatment | naturally integrated |
| Industry adoption | very high | academia-centered, spreading |
| Textbook | Craig, Siciliano | Lynch & Park |

DH parameters remain entrenched in classical textbooks and industrial robot manuals, whereas URDF files encode link and joint offsets directly without DH rules. PoE offers a cleaner Lie-group formulation that dominates modern research derivations. Knowing both makes it straightforward to move between vendor datasheets, URDF models, and control papers. For a hands-on example of the DH approach in Python, `robotics-toolbox-python` builds a classic six-axis Puma 560 model and computes its pose and Roll-Pitch-Yaw angles in a few lines:

```python
# Example of DH-based FK with robotics-toolbox-python (Puma 560)
import roboticstoolbox as rtb

puma = rtb.models.DH.Puma560()
q = [0, -np.pi/4, np.pi/4, 0, np.pi/6, 0]  # six joint angles
T = puma.fkine(q)
print(T)  # print the 4x4 SE(3) homogeneous transformation matrix
print(f"Position: {T.t}")  # end-effector position
print(f"RPY angles: {T.rpy()}")  # Roll-Pitch-Yaw
```

> **Further reading**
> - Lynch & Park, *Modern Robotics*, Chapter 4 — a PoE-centered treatment with a free PDF and Coursera course: https://modernrobotics.org
> - Craig, *Introduction to Robotics*, Chapter 3 — a textbook treatment using the Modified DH convention.
> - Peter Corke, *Robotics, Vision and Control* — lets you practice FK together with Python code: https://github.com/petercorke/robotics-toolbox-python

---

## 4.3 Inverse Kinematics

Forward kinematics is straightforward: multiplying joint transforms produces a unique end-effector pose. **Inverse kinematics (IK)** reverses the question—*"To place the end-effector at a target pose, what angle must each joint take?"*—and turns out to be dramatically harder for four structural reasons:

1. **Nonlinear equations**: nested trigonometric functions couple the joint angles together and resist simple linear algebra.
2. **Multiple solutions**: several distinct postures (such as elbow-up versus elbow-down) can place the end-effector at the exact same pose.
3. **No solution**: any target outside the mechanism's reach has no valid joint configuration at all.
4. **Infinitely many solutions**: whenever the arm has more degrees of freedom than the task requires (a redundant manipulator), a continuous manifold of configurations achieves the same pose.


### 4.3.1 Analytical IK

Analytical IK derives closed-form trigonometric expressions for the joint variables. When a closed-form solution exists, every candidate posture can be evaluated directly without iterative optimization, though numerical precision and runtime still depend on careful singularity and branch handling.

**IK of a 2-link planar arm:**

For a planar two-link arm reaching toward $(x, y)$, applying the law of cosines to the triangle formed by the two links and the target radius gives the elbow angle $\theta_2$, after which subtracting the inner link angle from the target bearing yields the shoulder angle $\theta_1$:

```
cos(θ_2) = (x² + y² - L1² - L2²) / (2 * L1 * L2)
θ_2 = atan2(±√(1 - cos²(θ_2)), cos(θ_2))

θ_1 = atan2(y, x) - atan2(L2*sin(θ_2), L1 + L2*cos(θ_2))
```

Notice the $\pm$ sign in $\theta_2$: it exposes the two valid branches—elbow-up and elbow-down—that reach the same point, illustrating why IK rarely yields a single answer. The Python script below implements this closed-form solver, checks workspace reachability, and verifies the solution by feeding it back through forward kinematics:

```python
def ik_2link(x, y, L1=1.0, L2=1.0, elbow_up=True):
    """Inverse kinematics of a 2-link planar arm. Returns None if no solution."""
    d_sq = x**2 + y**2
    # check reachability
    if d_sq > (L1 + L2)**2 or d_sq < (L1 - L2)**2:
        return None

    cos_q2 = (d_sq - L1**2 - L2**2) / (2 * L1 * L2)
    cos_q2 = np.clip(cos_q2, -1.0, 1.0)  # numerical safety

    if elbow_up:
        q2 = np.arctan2(np.sqrt(1 - cos_q2**2), cos_q2)
    else:
        q2 = np.arctan2(-np.sqrt(1 - cos_q2**2), cos_q2)

    q1 = np.arctan2(y, x) - np.arctan2(L2 * np.sin(q2), L1 + L2 * np.cos(q2))
    return q1, q2

# Verify: FK → IK → FK
target_x, target_y = 1.2, 0.8
result = ik_2link(target_x, target_y)
if result:
    q1, q2 = result
    x_check, y_check, _ = fk_2link(q1, q2)
    print(f"Target: ({target_x}, {target_y})")
    print(f"IK solution: q1={np.degrees(q1):.2f}°, q2={np.degrees(q2):.2f}°")
    print(f"FK check: ({x_check:.6f}, {y_check:.6f})")
    print(f"Error: {np.sqrt((x_check-target_x)**2 + (y_check-target_y)**2):.2e}")
```

**Analytical IK of a 6R manipulator:**

Extending closed-form IK to six-axis robots relies on **Pieper's condition**: if three consecutive joint axes intersect at a single point—most commonly the last three axes forming a **spherical wrist**—the 6-DOF problem decouples cleanly. Because rotating the wrist does not translate that intersection point, the first three joints alone place the wrist center in space, and the last three joints orient the end-effector around it. Classical industrial six-axis arms (such as KUKA and ABB manipulators) adopt this spherical-wrist geometry specifically to guarantee fast analytical IK. Universal Robots (UR) arms, by contrast, use an offset wrist whose last three axes do not intersect at one point; a closed-form solution still exists, but it requires a dedicated algebraic derivation.

Even with a spherical wrist, decoupling typically yields up to eight distinct configurations (combinations of shoulder, elbow, and wrist flips). Controllers normally filter out solutions that violate joint limits and pick the valid branch closest to the robot's current joint angles to avoid sudden jumps.


### 4.3.2 Numerical IK

When no closed-form expression is practical—as with complex non-standard linkages or 7-DOF redundant arms—IK is framed as an iterative numerical optimization problem. By linearizing the forward kinematics around the current joint configuration $q$, a small Cartesian pose error $\Delta x$ relates locally to a joint increment $\Delta q$ through the Jacobian $J(q)$ (examined in detail in §4.4).

**Jacobian pseudo-inverse method:**

Inverting that local linear relationship with the Moore-Penrose pseudo-inverse $J^\dagger(q)$ produces an update step that drives the pose error toward zero:

```
Δq = J†(q) * Δx
```

Repeating this step in a loop iteratively walks the arm to the target pose, as demonstrated for the two-link planar arm below:

```python
def numerical_ik_2link(target_x, target_y, L1=1.0, L2=1.0,
                        max_iter=100, tol=1e-6):
    """Numerical IK based on the Jacobian pseudo-inverse."""
    # initial guess (random or current joint angles)
    q = np.array([0.5, 0.5])

    for i in range(max_iter):
        # current FK
        x = L1 * np.cos(q[0]) + L2 * np.cos(q[0] + q[1])
        y = L1 * np.sin(q[0]) + L2 * np.sin(q[0] + q[1])

        # error
        error = np.array([target_x - x, target_y - y])
        if np.linalg.norm(error) < tol:
            print(f"Converged: {i+1} iterations")
            return q

        # Jacobian
        J = np.array([
            [-L1*np.sin(q[0]) - L2*np.sin(q[0]+q[1]), -L2*np.sin(q[0]+q[1])],
            [ L1*np.cos(q[0]) + L2*np.cos(q[0]+q[1]),  L2*np.cos(q[0]+q[1])]
        ])

        # update joint angles via pseudo-inverse
        dq = np.linalg.pinv(J) @ error
        q += dq

    print("Failed to converge")
    return q
```

**Damped Least Squares (DLS, Levenberg-Marquardt):**

The pure pseudo-inverse fails badly near kinematic singularities: as the arm approaches a posture where $J$ loses rank, its smallest singular value approaches zero, causing $J^\dagger$ to amplify tiny Cartesian errors into enormous joint jumps. **Damped Least Squares (DLS)** stabilizes the step by adding a positive damping factor $\lambda$ to the normal equations:

```
Δq = J^T (J * J^T + λ²I)^{-1} * Δx
```

Adding $\lambda^2 I$ keeps the inverted matrix well-conditioned even when $J$ is rank-deficient. A large $\lambda$ guarantees smooth, bounded joint velocities near singularities at the cost of slower convergence, whereas a small $\lambda$ recovers the fast behavior of the pure pseudo-inverse. In practice, adaptive schedules that scale $\lambda$ up only as the robot nears a singularity (Nakamura & Hanafusa, 1986) give the best of both worlds.


### 4.3.3 Singularity

A joint configuration at which the Jacobian loses rank is called a **singularity**. Physically, two or more joint axes align so that their instantaneous motions become linearly dependent, stripping the end-effector of mobility in at least one spatial direction. At or near a singularity, three pathologies emerge:

1. **Loss of a degree of freedom**: the end-effector cannot produce instantaneous velocity along a particular direction.
2. **Exploding joint speeds**: demanding even a tiny Cartesian motion along the blocked direction drives required joint velocities toward infinity, far beyond real motor limits.
3. **Discontinuous IK branches**: passing near a singular posture can cause analytical or numerical solvers to snap abruptly between distant joint configurations.

In the two-link planar arm, singularities occur intuitively at $\theta_2 = 0$ (arm fully stretched out) and $\theta_2 = \pi$ (arm folded completely back on itself). At either configuration, the tip can sweep tangentially along a circle, but no joint motion can generate instantaneous radial velocity.

On a six-axis manipulator, singularities typically appear in three forms:
- **Wrist singularity**: joint axes 4 and 6 become collinear ($q_5 \approx 0$), collapsing one rotational degree of freedom at the wrist.
- **Shoulder singularity**: the wrist center sits directly above the base rotation axis (axis 1).
- **Elbow singularity**: the arm reaches full extension, matching the planar two-link case.

In real deployments, engineers manage singularities by planning paths that steer clear of singular regions, clamping joint speeds with DLS regularization when traversing near them, or adding kinematic redundancy (extra joints) so the arm can reconfigure without losing end-effector mobility.


### 4.3.4 IK Solvers

Outside of learning exercises, production systems rarely build an IK solver from scratch; relying on a well-tested library avoids subtle failures around joint limits, wrap-around angles, and singularities. The table below summarizes the most common solvers in robotics workflows:

| Solver | Method | Notes |
|------|------|------|
| **KDL** | Numerical (Newton-Raphson) | available in the ROS ecosystem; sensitive to joint limits, initialization, and singularities |
| **IKFast** (OpenRAVE) | Analytical (code generation) | auto-generates C++ code for specific structures. Fast |
| **TRAC-IK** | KDL + SQP dual | higher solve rate than stock KDL on the paper's tested chains; ROS package available |
| **MoveIt2 IK** | Integrates the solvers above | ROS2 ecosystem, integrated collision avoidance |
| **pinocchio** | Jacobian-based numerical iteration (CLIK) | rigid-body dynamics library, fast, provides analytical derivatives |

To put the numerical trade-offs in perspective, the benchmark note below summarizes the empirical comparison from the TRAC-IK paper:

```python
# Beeson & Ames (2015) tested 10,000 reachable poses per robot model
# on five models, with a 5 ms limit per solve.
# In that experiment TRAC-IK had a higher solve rate than stock KDL,
# but the rate varied with the chain, seed, and error tolerance.
```

> **Further reading**
> - [Beeson & Ames, "TRAC-IK: An Open-Source Library for Improved Solving of Generic Inverse Kinematics" (2015)](https://doi.org/10.1109/HUMANOIDS.2015.7363472) — see the paper's tables for per-model conditions and solve rates
> - MoveIt2 IK documentation: https://moveit.picknik.ai/main/doc/concepts/inverse_kinematics.html
> - Pinocchio (rigid body dynamics library): https://github.com/stack-of-tasks/pinocchio

---

## 4.4 Jacobian

Whereas forward and inverse kinematics relate static joint angles to static poses, real-time motion control needs to know how *velocities* and *forces* transform between joint space and task space. Both mappings revolve around one matrix: the **Jacobian**.


### 4.4.1 Joint Velocity → End-Effector Velocity

Differentiating the forward kinematics at configuration $q$ yields the linear map from joint velocities $\dot{q}$ to the end-effector's spatial velocity $\dot{x}$ (combining linear velocity $v$ and angular velocity $\omega$):

```
ẋ = J(q) * q̇

where ẋ = [v; ω] ∈ ℝ^6 (for a 6-axis case)
      q̇ ∈ ℝ^n
      J(q) ∈ ℝ^{6×n}
```

For a full six-dimensional end-effector task, $n < 6$ leaves too few joint degrees of freedom to span all spatial velocities, $n = 6$ matches the task dimension, and $n > 6$ provides kinematic redundancy. What motions the arm can actually execute at a given posture depends on the rank of $J(q)$, whereas underactuation is a separate question comparing the number of independent control inputs against the system's degrees of freedom.


### 4.4.2 Force/Torque Relation (Duality)

Remarkably, the exact same Jacobian—transposed—maps static forces and moments $F$ applied at the end-effector back to the joint torques $\tau$ required to balance them:

```
τ = J^T(q) * F
```

This relationship, known as **static duality**, connects velocity kinematics and statics directly through conservation of mechanical power (or virtual work). Because instantaneous power must match whether measured at the end-effector or summed across the joints, substituting $\dot{x} = J\dot{q}$ yields:

```
P = F^T * ẋ = F^T * J * q̇ = (J^T * F)^T * q̇ = τ^T * q̇
```

Static duality sits at the heart of force and impedance control: to exert a desired wrench $F$ on the environment with the end-effector, a torque-controlled arm simply commands $\tau = J^T(q) F$ at its joints.


### 4.4.3 Manipulability Ellipsoid

Beyond converting velocities and forces, the Jacobian quantifies how effortlessly a robot can move in different directions from its current posture. Imagine all joint-velocity vectors inside the unit sphere ($\|\dot{q}\| \le 1$); pushing that sphere through $\dot{x} = J\dot{q}$ stretches it into a task-space **manipulability ellipsoid** whose principal axes align with the eigenvectors of $J J^T$. Its overall volume is captured by Yoshikawa's manipulability index:

```
manipulability index = √det(J * J^T)
```

For an arm whose Jacobian normally has full row rank for the six-dimensional task, a manipulability index of zero signals a singularity. Larger index values indicate a larger ellipsoid volume, though directional uniformity depends on the ratios of the ellipsoid's semi-axes: large eigenvalues of $J J^T$ mark directions along which the end-effector moves rapidly, whereas small eigenvalues mark sluggish directions. When all eigenvalues match, motion is isotropic; when they diverge sharply, the posture is highly anisotropic.

The script below evaluates the base-frame Jacobian of a Puma 560, computes its manipulability index, extracts the linear-velocity ellipsoid semi-axes, and reports the singular-value condition number:

```python
import roboticstoolbox as rtb
import numpy as np

# Jacobian and manipulability of the Puma 560
puma = rtb.models.DH.Puma560()
q = [0, -np.pi/4, np.pi/4, 0, np.pi/6, 0]

J = puma.jacob0(q)  # 6x6 Jacobian (in the base frame)

# Manipulability index
m = np.sqrt(np.linalg.det(J @ J.T))
print(f"Manipulability index: {m:.4f}")

# Principal axes of the velocity ellipsoid (eigenvalue analysis)
JJT = J[:3, :] @ J[:3, :].T  # linear-velocity part only
eigenvalues, eigenvectors = np.linalg.eigh(JJT)
print(f"Velocity ellipsoid semi-axes: {np.sqrt(eigenvalues)}")

# Condition number: isotropy indicator (closer to 1 is better)
sigma = np.linalg.svd(J, compute_uv=False)
cond = sigma[0] / sigma[-1]
print(f"Condition number: {cond:.2f}")
# cond = 1 is perfect isotropy; infinite means a singularity
```


### 4.4.4 Practical Code: Jacobian-Based Velocity Control

In real-time Cartesian velocity control—often called *resolved-rate control*—a controller reads the current joint angles, solves for joint rates using a damped pseudo-inverse of the Jacobian to stay safe near singularities, clamps those rates within motor limits, and integrates forward over a 1 ms step (`dt=0.001`):

```python
import numpy as np

def jacobian_velocity_control(robot_fk, robot_jacob, q_current,
                               desired_twist, dt=0.001):
    """
    Jacobian-based resolved rate control.

    Args:
        robot_fk: FK function (q -> SE3)
        robot_jacob: Jacobian function (q -> 6xn matrix)
        q_current: current joint angles
        desired_twist: desired end-effector velocity [vx, vy, vz, wx, wy, wz]
        dt: control period
    Returns:
        q_new: new joint angles
    """
    J = robot_jacob(q_current)

    # Damped least squares
    lambda_dls = 0.01
    n = J.shape[1]
    JJT = J @ J.T
    J_dls = J.T @ np.linalg.inv(JJT + lambda_dls**2 * np.eye(JJT.shape[0]))

    q_dot = J_dls @ desired_twist

    # joint velocity limits (essential on a real robot)
    max_qdot = 2.0  # rad/s
    scale = np.max(np.abs(q_dot)) / max_qdot
    if scale > 1.0:
        q_dot /= scale

    q_new = q_current + q_dot * dt
    return q_new
```

> **Further reading**
> - Siciliano et al., *Robotics: Modelling, Planning and Control*, Chapter 3 — a broad treatment of Jacobians in kinematics and dynamics
> - Corke, *Robotics, Vision and Control*, Chapter 8 — includes code examples and visualization: https://petercorke.com/rvc/
> - robotics-toolbox-python documentation: https://github.com/petercorke/robotics-toolbox-python

---

## 4.5 Mechatronics Basics

Computing joint angles and velocities only solves half the problem. Turning mathematical commands into physical motion requires motors, gearboxes, sensors, deterministic communication buses, and real-time software—the engineering stack of **mechatronics**.


### 4.5.1 Actuators

Electric motors form the muscle of most modern robots, spanning everything from simple brushed units to integrated smart servos and high-transparency direct-drive assemblies.

**DC motor:**
A brushed DC motor is the simplest electric actuator: applying a voltage across its terminals spins the rotor, with output torque proportional to current ($\tau = K_t i$) and back-EMF proportional to angular speed ($V_{\text{emf}} = K_e \omega$). Brushed motors are inexpensive and straightforward to drive, though mechanical brush wear limits their operating lifetime.

**BLDC (Brushless DC) motor:**
A brushless DC motor replaces physical brushes with electronic commutation, delivering longer service life, higher torque density, and superior efficiency—traits that make it the standard choice on modern robotic platforms. Pairing a BLDC motor with Field-Oriented Control (FOC) keeps stator currents aligned with the rotor flux, minimizing torque ripple at both low and high speeds.

**Servo motors (Dynamixel series):**
Rather than assembling motor, gearbox, encoder, and driver electronics separately, all-in-one smart servos package all four into a single modular housing. Robotis's Dynamixel family is ubiquitous across research and educational arms, with representative models summarized below:

| Model | Example advertised maximum torque (Nm) | Communication | Use |
|------|-----------|------|------|
| XL330 | 0.5 | TTL | small grippers, small low-cost arms |
| XM540 | 10.0 | RS-485 | mid-sized robot arms |
| PH54  | 44.7 | RS-485 | large manipulators, mobile robots |

Because advertised maximum torque varies with supply voltage and operating conditions, real actuator sizing must rely on each e-Manual's rated, stall, and continuous-duty curves. Beyond compact packaging, Dynamixel actuators simplify cabling via daisy-chain wiring and expose position, velocity, and current-based control modes with tunable PID gains; however, communication rates, control-cycle timing, and thermal headroom vary by model, so check that the stock firmware supports the required control mode and bandwidth.

**Quasi-Direct Drive (QDD):**

Traditional robot arms rely on high-ratio gearboxes—often 100:1 or higher using harmonic drives—to multiply torque from compact, high-speed motors. That amplification carries a hidden mechanical cost: reflected rotor inertia scales with the *square* of the gear ratio ($N^2 I_{\text{motor}}$). Through a 100:1 reducer, a light spinning rotor feels 10,000 times heavier at the joint output, turning external impacts into harsh shock loads and masking subtle contact forces behind gearbox friction.

**Quasi-Direct Drive (QDD)** actuation, popularized by the MIT Mini Cheetah (2019), flips that trade-off by pairing a high-torque-density motor with a **low gear ratio** of roughly 6:1 to 10:1 (typically a single-stage planetary gearbox or belt):

- **High backdrivability**: the joint yields readily under external forces, simplifying safe collision response and compliant control.
- **High transparency**: with lower gearbox friction and damping, joint output torque can be estimated accurately from motor phase current once basic friction is modeled.
- **High torque bandwidth**: minimal reducer compliance and stiction leave room for rapid, clean torque tracking.

The trade-off is lower torque multiplication per unit volume: achieving high output torques without a large reduction ratio demands a larger-diameter, higher-torque motor. Platforms such as the MIT Mini Cheetah, Cheetah 3, and the Unitree quadruped series all rely on this QDD architecture. The arithmetic below illustrates just how sharply gear ratio governs reflected inertia:

```
# Torque-control comparison: QDD vs traditional reducer
#
# Traditional (gear ratio 100:1, harmonic drive):
#   reflected inertia = N² × I_motor
#   → motor inertia 0.001 kg·m² × 100² = 10 kg·m²
#   → motor inertia reflected to the joint output is very large
#   → precise force control is difficult
#
# QDD (gear ratio 8:1):
#   with the same motor, reflected inertia = 8² × 0.001 = 0.064 kg·m²
#   → about 156× smaller in this gear-ratio-only example
#   → actual force-control performance also depends on link inertia, friction, and control
```


**Reducer types:**

Choosing an actuator almost always involves selecting its mechanical reducer alongside the motor. The three most common gearbox families balance gear ratio, backlash, efficiency, and cost in distinct ways:

| Type | Gear ratio | Backlash | Efficiency | Price | Use |
|------|--------|--------|------|------|------|
| Planetary | 3~100:1 | medium | 85-95% | cheap | general-purpose, suited to QDD |
| Harmonic Drive | 30~320:1 | very low | 65-85% | expensive | industrial robots, precision |
| Cycloidal | 6~120:1 | low | 85-93% | medium | emerging as a recent alternative |

Because actual gear ratios, efficiencies, and backlash vary considerably across specific geometries and manufacturers, treat the table above as a rough orientation and base final selections on rated-load datasheets.

**Actuator selection criteria:**

When sizing a joint actuator, combine static holding torque, dynamic acceleration torque, and expected impact loads, then apply a design margin tailored to load uncertainty, fatigue life, and the consequences of failure (the factor of 2 in the code below serves purely as an illustration, not a universal rule). Maximum joint angular speed must also map through the gear ratio into feasible motor RPM, alongside checks on backdrivability, actuator mass, continuous thermal rating, and cost. Ultimately, choosing between a QDD and a harmonic drive comes down to whether the application prizes torque density and positional precision or backdrivable force transparency. The script below walks through a back-of-the-envelope sizing calculation for a single horizontal arm link:

```python
# Simple actuator-sizing example
import numpy as np

# Goal: lift a 1 kg object at the arm tip (arm length 0.5 m)
m_payload = 1.0  # kg
m_link = 0.5     # weight of the link itself
L = 0.5          # m
g = 9.81         # m/s²

# Worst-case torque (arm fully horizontal)
tau_static = (m_payload * L + m_link * L/2) * g
print(f"Static torque: {tau_static:.2f} Nm")

# Acceleration torque (max angular acceleration 10 rad/s²)
alpha_max = 10.0  # rad/s²
I_total = m_payload * L**2 + m_link * (L/2)**2  # moment of inertia (simplified)
tau_dynamic = I_total * alpha_max
print(f"Dynamic torque: {tau_dynamic:.2f} Nm")

# Total required torque (safety factor 2)
tau_required = (tau_static + tau_dynamic) * 2.0
print(f"Required torque (safety factor 2): {tau_required:.2f} Nm")

# Max angular velocity → motor RPM
omega_max = 3.0  # rad/s (joint)
gear_ratio = 8    # QDD
motor_rpm = omega_max * gear_ratio * 60 / (2 * np.pi)
print(f"Required motor RPM: {motor_rpm:.0f}")
```

> **Further reading**
> - Katz, "A Low Cost Modular Actuator for Dynamic Robots" (MIT, 2018) — the core QDD paper: https://dspace.mit.edu/handle/1721.1/118671
> - Dynamixel product lineup and documentation: https://emanual.robotis.com/
> - Seok et al., "Design Principles for Energy-Efficient Legged Locomotion and Implementation on the MIT Cheetah Robot" (2015)


### 4.5.2 Sensor Interfacing

Closing a feedback loop around those actuators requires accurate measurements of joint position, interaction forces, and body motion.

**Encoder:**

Encoders are the foundational sensors for measuring joint angle, coming in two main varieties:

*Incremental encoders* track relative rotation by counting pulses across two quadrature channels (A and B). They are inexpensive and readily achieve high resolutions (10,000 PPR or more), though they lose their absolute reference whenever power is cut and therefore require a homing routine on startup.

*Absolute encoders* report the true angular position immediately upon power-up, and multi-turn variants track full revolutions across power cycles as well. Although more expensive, eliminating startup homing makes them standard on industrial manipulators that must resume safely after a reboot.

Notice how gearbox reduction multiplies the effective angular precision seen at the output link:

```
Resolution example:
  Incremental encoder, 4096 PPR, quadrature decoding (x4)
  → resolution = 360° / (4096 × 4) = 0.022° ≈ 0.38 mrad
  → at a 100:1 reduced joint → output resolution 0.0038 mrad
```

**Torque sensors:**

When current-based torque estimation is masked by gearbox friction, strain-gauge sensors measure mechanical loads directly at two typical locations:

*Joint Torque Sensors (JTS)* sit on the output side of each reducer so they capture true joint torque free of transmission friction. The KUKA LBR iiwa set the industry benchmark for compliant manipulation by integrating a JTS into all seven of its joints.

*Force/Torque sensors (F/T sensors)* mount at the wrist or end-effector to resolve all six wrench components ($F_x, F_y, F_z, T_x, T_y, T_z$). Vendors such as ATI Industrial Automation supply widely used research units, which should be selected based on measurement range, resolution, overload limits, electrical interface, and a current vendor quote.

**Inertial sensors (IMU):**

Covered in detail in Ch.2, an inertial measurement unit combines an accelerometer, a gyroscope, and often a magnetometer. On mobile and legged platforms, IMUs anchor body-pose estimation; on manipulators, link-mounted IMUs are occasionally added to sense and damp structural vibrations.


### 4.5.3 Communication Protocols

Connecting sensors and motor drivers back to a microcontroller or host PC often causes more headaches in real robot builds than the kinematics equations themselves: excessive latency erodes control stability, while insufficient bandwidth drops telemetry packets.

**Basic protocols:**

On a single circuit board or across short internal harnesses, three serial interfaces cover most microcontroller-level peripherals:

| Protocol | Wiring | Speed | Distance | Notes |
|---------|------|------|------|------|
| **UART** | 2-wire (TX, RX) | ~1 Mbps | ~15m | simplest, 1:1 communication |
| **SPI** | 4-wire (MOSI, MISO, SCK, CS) | ~50 Mbps | ~1m (on-PCB) | fast; multiple slaves need extra CS lines |
| **I2C** | 2-wire (SDA, SCL) | 100k~3.4 Mbps | ~1m | address-based, convenient for sensor buses |

While UART, SPI, and I2C excel inside a PCB enclosure, multi-joint robots spanning meters of cabling near switching motor currents demand differential signaling, deterministic timing, and multi-node topologies.

**CAN Bus:**

Developed originally for automotive electronics, the Controller Area Network (CAN) bus is widely adopted for motor and sensor backbones in legged and mobile robots. Differential signaling makes it resilient to electromagnetic noise, while its multi-master design resolves bus collisions through priority-based arbitration:

- **Speed**: up to 1 Mbps (CAN 2.0) or 5 Mbps (CAN FD)
- **Distance**: up to about 1 km at 50 kbps (dropping to ~500 m at 125 kbps and ~40 m at 1 Mbps)
- **Topology**: linear bus supporting daisy-chained nodes

In practice, CAN typically links a central controller to distributed motor drivers—a topology used on the MIT Cheetah and many modern quadrupeds. Because standard CAN 2.0 frames hold at most 8 bytes, firmware usually packs floating-point setpoints into scaled 16-bit integers, as shown in the STM32 HAL pseudo-code below:

```cpp
// Example of sending a motor command over CAN bus (pseudo-code, STM32 HAL)
#include "can.h"

struct MotorCommand {
    float position;   // rad
    float velocity;   // rad/s
    float torque;     // Nm
    float kp;         // position gain
    float kd;         // velocity gain
};

void send_motor_command(CAN_HandleTypeDef* hcan, uint8_t motor_id,
                        MotorCommand cmd) {
    CAN_TxHeaderTypeDef header;
    header.StdId = motor_id;   // unique CAN ID for each motor
    header.DLC = 8;            // 8 bytes (CAN 2.0 standard)
    header.RTR = CAN_RTR_DATA;

    // pack floats as integers (typical for robot motor drivers)
    uint8_t data[8];
    int16_t pos_int = (int16_t)(cmd.position / 0.001f);   // 0.001 rad units
    int16_t vel_int = (int16_t)(cmd.velocity / 0.01f);    // 0.01 rad/s units
    int16_t tau_int = (int16_t)(cmd.torque / 0.01f);      // 0.01 Nm units
    int16_t kp_int  = (int16_t)(cmd.kp / 0.01f);

    data[0] = pos_int >> 8;  data[1] = pos_int & 0xFF;
    data[2] = vel_int >> 8;  data[3] = vel_int & 0xFF;
    data[4] = tau_int >> 8;  data[5] = tau_int & 0xFF;
    data[6] = kp_int >> 8;   data[7] = kp_int & 0xFF;

    uint32_t mailbox;
    HAL_CAN_AddTxMessage(hcan, &header, data, &mailbox);
}
```

**EtherCAT:**

When dozens of high-rate joints need tighter synchronization than CAN can provide, **EtherCAT** brings deterministic microsecond-scale timing onto standard Ethernet hardware. Robots such as KUKA arms and many research humanoids rely on EtherCAT for three main reasons:

- **High bandwidth**: 100 Mbps links synchronize dozens to hundreds of nodes at sub-millisecond cycles.
- **Deterministic timing**: predictable, tightly bounded packet delays keep real-time feedback loops stable.
- **On-the-fly frame processing**: each slave reads its command and inserts sensor telemetry dynamically as a single master frame passes through, maximizing bus efficiency.

The price of that performance is software and configuration complexity: running EtherCAT requires a dedicated master stack (such as SOEM or IgH EtherCAT Master) and careful topology setup, making it unnecessary overhead for hobbyist builds.

**RS-485 / Dynamixel Protocol:**

For modular smart servos like Dynamixel units, **RS-485** offers a simpler differential serial alternative capable of several Mbps over short daisy-chained harnesses (configured at 1 Mbps in the example below). Using the Dynamixel SDK, a host script can open a USB-to-RS485 adapter, enable motor torque, command a target position, and read back the encoder value in a few lines of Python:

```python
# Example of servo control using the Dynamixel SDK
from dynamixel_sdk import *

PROTOCOL_VERSION = 2.0
BAUDRATE = 1000000
DEVICENAME = '/dev/ttyUSB0'
DXL_ID = 1

# open the port
port = PortHandler(DEVICENAME)
packet = PacketHandler(PROTOCOL_VERSION)
port.openPort()
port.setBaudRate(BAUDRATE)

# enable torque
ADDR_TORQUE_ENABLE = 64
packet.write1ByteTxRx(port, DXL_ID, ADDR_TORQUE_ENABLE, 1)

# move to target position (units: 0~4095, 0~360 degrees)
ADDR_GOAL_POSITION = 116
goal_position = 2048  # center (180 degrees)
packet.write4ByteTxRx(port, DXL_ID, ADDR_GOAL_POSITION, goal_position)

# read current position
ADDR_PRESENT_POSITION = 132
pos, _, _ = packet.read4ByteTxRx(port, DXL_ID, ADDR_PRESENT_POSITION)
print(f"Current position: {pos} (= {pos * 360 / 4096:.1f}°)")
```


### 4.5.4 Real-Time Systems

In robot control, **"real-time" does not mean "fast on average"—it means guaranteed completion before a strict deadline.** On a 1 kHz control loop, every 1 ms cycle must finish reading sensors, computing the control law, and transmitting actuator commands before the clock expires; missing even a single deadline introduces phase lag that can destabilize a high-gain controller.

**RTOS (Real-Time Operating System):**

When closing low-level loops directly on a microcontroller, a Real-Time Operating System (RTOS) lets you assign strict task priorities so background logging or telemetry never preempts the motor control loop. Three common choices span lightweight embedded targets to aerospace hardware:

| RTOS | Notes | Use |
|------|------|------|
| **FreeRTOS** | lightweight, for microcontrollers, free | STM32, ESP32, etc. |
| **Zephyr** | modern, broad hardware support, Linux Foundation | IoT, robot embedded |
| **VxWorks** | commercial, used by NASA | aerospace, industrial |

**PREEMPT_RT Linux:**

Higher-level autonomy stacks such as ROS2 typically run on desktop Linux, yet a standard Linux kernel is optimized for throughput rather than determinism—its scheduler can stall a control thread for several milliseconds during disk I/O or interrupts. Patching the kernel with **PREEMPT_RT** makes nearly all kernel code paths preemptible, bringing worst-case scheduling latency down into the deterministic regime. Setting up and verifying a real-time Linux host follows the five-step workflow below:

```bash
# 1. Install a kernel with PREEMPT_RT (Debian example)
sudo apt install linux-image-rt-amd64   # Debian metapackage. Ubuntu has no package by this name; it uses a separate path such as the Ubuntu Pro real-time kernel

# 2. Configure GRUB to boot the RT kernel

# 3. Give the control thread real-time priority
sudo chrt -f 99 ./my_robot_controller

# 4. CPU isolation (optional but recommended)
#    add isolcpus=2,3 in /etc/default/grub
#    → isolate CPUs 2, 3 from ordinary processes
#    → pin the control thread to those CPUs (affinity)

# 5. Verify performance
sudo cyclictest -m -p 99 -t 1 -n
# compare worst-case latency with the target control period and timing margin
```

**Choosing a control period:**

Although 1 kHz (1 ms) is a familiar baseline for torque and impedance control, it is a design point rather than a universal law. An appropriate loop rate depends on the target closed-loop bandwidth, structural resonances, sensor and actuator latency, solver execution time, and acceptable jitter margin. Remember that the Nyquist rate (twice the bandwidth) is merely an anti-aliasing lower bound—not a guarantee of stable closed-loop performance; in practice, sample comfortably faster than the desired closed-loop bandwidth and verify both frequency response and delay margins. Similarly, CAN bus capacity cannot be judged from motor count alone without accounting for frame overhead, arbitration delays, total bus load, and feedback frequency.

Lightweight arms, rapid collision-reaction reflexes, and tactile controllers sometimes push into multi-kilohertz loops. Even then, EtherCAT or an FPGA is not mandatory by default: select among CAN, EtherCAT, an MCU, or an FPGA based on the determinism, aggregate bandwidth, and I/O topology the mechanism actually demands.

> **Further reading**
> - FreeRTOS official documentation: https://www.freertos.org/
> - PREEMPT_RT Wiki: https://wiki.linuxfoundation.org/realtime/start
> - Dynamixel SDK: https://github.com/ROBOTIS-GIT/DynamixelSDK
> - IgH EtherCAT Master (open-source for Linux): https://etherlab.org/en/ethercat/
> - SOEM (Simple Open EtherCAT Master): https://github.com/OpenEtherCATsociety/SOEM

---

## 4.6 Advanced: Workspace Analysis and Optimal Design

Beyond controlling an existing robot, kinematics also serves as a yardstick for designing and comparing mechanisms. Workspace boundaries, directional conditioning, and redundant degrees of freedom all come straight out of the forward map and Jacobian developed above.


### 4.6.1 Reachable Workspace vs Dexterous Workspace

Not every point a robot can touch can be approached from arbitrary angles. Kinematic design distinguishes two nested volumes:

- **Reachable workspace**: the set of all 3D points the end-effector can attain in *at least one* orientation—roughly, how far the arm can stretch.
- **Dexterous workspace**: the subset of points the end-effector can reach in *every* orientation—where the wrist remains free to rotate without constraint, which is typically much smaller.

On a six-axis arm, joint limits and wrist singularities often shrink the dexterous workspace to a tight core, motivating the move to seven-axis manipulators. Rather than deriving workspace boundaries analytically, engineers routinely map them via Monte Carlo sampling: draw thousands of random configurations within the joint limits, evaluate FK, and plot the resulting end-effector point cloud, as shown below for the Puma 560:

```python
import numpy as np
import roboticstoolbox as rtb

# Workspace visualization of the Puma 560 (Monte Carlo)
puma = rtb.models.DH.Puma560()
n_samples = 50000
positions = []

for _ in range(n_samples):
    # random sample within each joint's range
    q = puma.random_q()
    T = puma.fkine(q)
    positions.append(T.t)  # [x, y, z]

positions = np.array(positions)

# Visualization (matplotlib)
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
ax.scatter(positions[:, 0], positions[:, 1], positions[:, 2],
           s=0.1, alpha=0.1, c='blue')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title('Puma 560 Reachable Workspace')
plt.savefig('workspace.png', dpi=150)
```


### 4.6.2 Condition Number and Isotropy

Whereas workspace plots show *where* an arm can reach, the Jacobian's **condition number** ($\kappa$) measures *how cleanly* it can move at a given configuration:

```
κ(J) = σ_max / σ_min
```

Here $\sigma_{\max}$ and $\sigma_{\min}$ are the largest and smallest singular values of $J$:

- **$\kappa = 1$**: perfect isotropy, meaning the arm can move with uniform sensitivity in every direction (achievable for well-chosen link geometries and postures).
- **$\kappa \to \infty$**: a kinematic singularity, where mobility vanishes completely along at least one direction.

Minimizing the condition number across a target workspace is a central objective of **kinematic optimization** (optimal mechanical design). One important caveat applies whenever linear and angular rows appear together in $J$: translational velocity ($\text{m/s}$) and angular velocity ($\text{rad/s}$) carry different physical units, making raw singular values scale-dependent and physically inconsistent. Always normalize the rows by a characteristic link length or evaluate the translational and rotational sub-Jacobians separately—a classic subtlety in kinematic optimization.


### 4.6.3 Redundancy Resolution (7-DOF Arms)

A seven-axis manipulator—such as the Kinova Gen3, KUKA LBR iiwa, or Franka Emika Panda—carries one extra degree of freedom beyond a six-dimensional end-effector pose. Because of this **kinematic redundancy**, the arm can reshape its intermediate posture without shifting the end-effector at all, much like raising or lowering your elbow while keeping your fist pinned to a fixed point in space.

Controllers exploit that internal freedom to pursue secondary goals while tracking the primary task:
1. **Singularity avoidance**: steer the extra joint away from postures where manipulability collapses.
2. **Joint-limit avoidance**: bias joints back toward the center of their travel ranges before they hit hard stops.
3. **Obstacle avoidance**: swing the elbow clear of clutter in the workspace.
4. **Torque or energy minimization**: settle into postures that carry gravity or task loads with less motor effort.

Mathematically, those internal motions live in the **null space** of the Jacobian—the subspace of joint velocities $\dot{q}$ that map to zero end-effector motion ($J\dot{q} = 0$):

```
q̇ = J† * ẋ + (I - J† * J) * q̇_0
```

The first term, $J^\dagger \dot{x}$, produces the minimum-norm joint velocity that realizes the desired end-effector twist $\dot{x}$. The second term multiplies an arbitrary secondary velocity $\dot{q}_0$ (such as the gradient of a manipulability or joint-centering cost) by the null-space projector $(I - J^\dagger J)$, altering the internal posture without disturbing the end-effector trajectory. The function below implements this null-space projection for a 7-DOF arm, using the secondary task to pull joints gently toward their midpoint `q_center`:

```python
def redundancy_resolution(J, x_dot, q, q_center, k_null=0.5):
    """
    Redundancy resolution for a 7-DOF robot.

    Args:
        J: 6x7 Jacobian
        x_dot: 6x1 desired end-effector velocity
        q: 7x1 current joint angles
        q_center: 7x1 joint center values (null-space target)
        k_null: null-space gain
    Returns:
        q_dot: 7x1 joint velocities
    """
    # Damped pseudo-inverse
    lam = 0.01
    J_pinv = J.T @ np.linalg.inv(J @ J.T + lam**2 * np.eye(6))

    # primary objective: track end-effector velocity
    q_dot_primary = J_pinv @ x_dot

    # secondary objective: return toward joint center (null space)
    null_projector = np.eye(7) - J_pinv @ J
    q_dot_null = null_projector @ (k_null * (q_center - q))

    return q_dot_primary + q_dot_null
```

> **Further reading**
> - Siciliano et al., *Robotics: Modelling, Planning and Control*, Chapter 3.9 — detailed treatment of redundancy resolution
> - Nakamura, "Advanced Robotics: Redundancy and Optimization" (1991) — a classic
> - Dietrich et al., "An Overview of Null Space Projections for Redundant, Torque-Controlled Robots" (2015)
> - Franka Emika research interface: https://frankaemika.github.io/docs/

Every model up to this point has treated kinematics as deterministic: given a joint vector, a single exact pose follows. Once a wheeled robot rolls across a real floor, however, wheel slip and unmodeled terrain turn that single pose into a probability distribution—the subject of §4.7, which connects kinematics directly back to the Bayesian filtering framework of Ch.3.

---

## 4.7 Advanced: Probabilistic Motion Models

### 4.7.1 From Determinism to Probability

Forward kinematics (§4.2) and inverse kinematics (§4.3) are deterministic mappings: supply joint angles and a single end-effector pose emerges; specify a target pose and a discrete set of joint solutions follows. Even for rigid manipulators, however, that determinism belongs to the idealized equations rather than the physical machine—real arms suffer from gear backlash, link compliance, and assembly tolerances, which is why §4.1 listed kinematic calibration among core applications.

On wheeled mobile robots, the gap between command and reality widens dramatically. Wheels slip against the floor, tire radii shift as treads wear, and slight left-right asymmetries curve supposedly straight paths. Consequently, after issuing a motion command $u_t$ from pose $x_{t-1}$, the next pose $x_t$ is no longer a single point but a probability distribution over possible outcomes. A **probabilistic motion model** formalizes this transition uncertainty.

Representing the planar pose as $x_t = (x, y, \theta)^T \in SE(2)$, the motion model defines the conditional transition density of the next pose given the previous pose $x_{t-1}$ and the motion input $u_t$:

$$p(x_t \mid u_t, x_{t-1})$$

In mobile robotics, two complementary formulations represent this transition distribution:

- **Velocity model**: takes commanded linear and angular velocities $u_t = (v, \omega)^T$ as input and treats discrepancies between commanded and executed speed as stochastic noise. Because it depends only on commands, it works forward in time during motion planning.
- **Odometry model**: takes a pair of encoder-integrated poses $u_t = (\bar{x}_{t-1}, \bar{x}_t)$ as if it were the control input. Because encoder readings arrive only after the robot has moved, this model is retrospective and cannot guide open-loop planning; despite relying on kinematic wheel integration rather than direct pose sensing, it usually captures real-world displacement more faithfully than trusting raw velocity commands.

Each model also supports two distinct operational modes. **Closed-form density evaluation** outputs a scalar likelihood answering *"how plausible is a candidate pose $x_t$?"*—exactly what grid localization needs when evaluating the Bayes filter integral directly (note that EKF and UKF prediction instead propagate the mean and covariance via Jacobians or sigma points). **Sampling** runs forward in simulation to draw one concrete next pose $x_t$, which particle filters such as Monte Carlo Localization (MCL) invoke once per particle. Sections §4.7.2–§4.7.5 work through all four combinations.


### 4.7.2 Velocity Motion Model — Closed-Form

**Intuition.** Under ideal noise-free motion, a differential-drive robot holding constant linear velocity $v$ and angular velocity $\omega$ travels along a circular arc (degenerating to a straight line when $\omega = 0$). Real wheel noise bends that arc into a slightly different trajectory. Closed-form evaluation runs the geometry in reverse: given a starting pose $x_{t-1}$ and a hypothesized endpoint $x_t$, it solves for the circle center $(x_c, y_c)$ and radius $r^*$ connecting them, recovers the effective velocities $(\hat{v}, \hat{\omega})$ that would have traced that arc, and scores how likely the discrepancy from the commanded $(v, \omega)$ is under the noise model.

**Equations.** Starting from $x_{t-1} = (x, y, \theta)^T$ and a hypothesized $x_t = (x', y', \theta')^T$, the arc center, radius, sweep angle, and implied velocities are:

$$\mu = \frac{1}{2} \cdot \frac{(x - x')\cos\theta + (y - y')\sin\theta}{(y - y')\cos\theta - (x - x')\sin\theta}$$

$$x_c = \frac{x + x'}{2} + \mu(y - y'), \quad y_c = \frac{y + y'}{2} + \mu(x' - x)$$

$$r^* = \sqrt{(x - x_c)^2 + (y - y_c)^2}$$

$$\Delta\theta = \text{atan2}(y' - y_c,\ x' - x_c) - \text{atan2}(y - y_c,\ x - x_c)$$

$$\hat{v} = \frac{\Delta\theta \cdot r^*}{\Delta t}, \quad \hat{\omega} = \frac{\Delta\theta}{\Delta t}, \quad \hat{\gamma} = \frac{\theta' - \theta}{\Delta t} - \hat{\omega}$$

Motion errors are modeled as zero-mean additive noise whose variances grow linearly with the magnitude of the commanded speeds—capturing the physical intuition that driving or turning faster introduces greater uncertainty:

$$b_v = \alpha_1|v| + \alpha_2|\omega|, \quad b_\omega = \alpha_3|v| + \alpha_4|\omega|, \quad b_\gamma = \alpha_5|v| + \alpha_6|\omega|$$

Here $b_v$ and $b_\omega$ govern the noise variances on linear and angular velocity, respectively. The extra term $\hat{\gamma}$ acts as a final-heading perturbation: with only two noise variables $(v, \omega)$, reachable poses would collapse onto a 2D surface inside 3D $(x, y, \theta)$ space (*degeneracy*), so adding $\hat{\gamma}$ restores full 3D support. Physically, $\alpha_1, \alpha_2$ scale linear-velocity uncertainty, $\alpha_3, \alpha_4$ scale angular-velocity uncertainty, and $\alpha_5, \alpha_6$ scale final-heading jitter; all six parameters are calibrated empirically for each robot across straight, circular, and figure-eight runs.

Putting the inverse arc geometry together with the three noise likelihoods gives the standard closed-form algorithm (`prob(a, b)` denotes a zero-mean normal or triangular density with variance $b$):

**Algorithm box (PR Table 5.1: `motion_model_velocity`).**

```
Algorithm motion_model_velocity(x_t, u_t, x_{t-1}):
  # inputs: x_t=(x',y',θ'), u_t=(v,ω), x_{t-1}=(x,y,θ)
  # output: p(x_t | u_t, x_{t-1}) probability density

  μ = 0.5 * ((x − x')cosθ + (y − y')sinθ) / ((y − y')cosθ − (x − x')sinθ)
  x* = (x + x')/2 + μ(y − y')
  y* = (y + y')/2 + μ(x' − x)
  r* = sqrt((x − x*)² + (y − y*)²)
  Δθ = atan2(y' − y*, x' − x*) − atan2(y − y*, x − x*)
  v̂ = Δθ·r*/Δt
  ω̂ = Δθ/Δt
  γ̂ = (θ' − θ)/Δt − ω̂

  p1 = prob(v − v̂,  α₁|v| + α₂|ω|)
  p2 = prob(ω − ω̂,  α₃|v| + α₄|ω|)
  p3 = prob(γ̂,       α₅|v| + α₆|ω|)

  return p1 · p2 · p3
```

While `motion_model_velocity` works backward from a candidate pose to score its density, sampling runs the exact same noise model forward in time.

### 4.7.3 Velocity Motion Model — Sampling

Instead of inverting an arc between two fixed poses, **sampling** starts by drawing random perturbations around the commanded velocities and integrating the resulting arc forward over $\Delta t$ to generate one plausible next pose. Particle filters rely directly on this forward simulation to propagate particles, and avoiding the inverse arc calculation makes the implementation markedly simpler.

First, draw perturbed linear, angular, and final-heading velocities using the same six noise parameters:

$$\hat{v} = v + \text{sample}(\alpha_1|v| + \alpha_2|\omega|)$$
$$\hat{\omega} = \omega + \text{sample}(\alpha_3|v| + \alpha_4|\omega|)$$
$$\hat{\gamma} = \text{sample}(\alpha_5|v| + \alpha_6|\omega|)$$

Integrating those perturbed controls along a circular arc gives the sampled pose:

$$x' = x - \frac{\hat{v}}{\hat{\omega}}\sin\theta + \frac{\hat{v}}{\hat{\omega}}\sin(\theta + \hat{\omega}\Delta t)$$
$$y' = y + \frac{\hat{v}}{\hat{\omega}}\cos\theta - \frac{\hat{v}}{\hat{\omega}}\cos(\theta + \hat{\omega}\Delta t)$$
$$\theta' = \theta + \hat{\omega}\Delta t + \hat{\gamma}\Delta t$$

Because dividing by $\hat{\omega}$ blows up near zero rotation, any practical implementation must fall back to straight-line integration—$x' = x + \hat{v}\cos\theta\,\Delta t,\ y' = y + \hat{v}\sin\theta\,\Delta t$—whenever $|\hat{\omega}| < \epsilon$. In the algorithm below, `sample(b)` draws a zero-mean variate with variance $b$, commonly approximated via the central limit theorem using 12 uniform draws: $\frac{\sqrt{b}}{2}\sum_{i=1}^{12}\text{rand}(-1,1)$.

**Algorithm box (PR Table 5.3: `sample_motion_model_velocity`).**

```
Algorithm sample_motion_model_velocity(u_t, x_{t-1}):
  # inputs: u_t=(v,ω), x_{t-1}=(x,y,θ)
  # output: sample x_t ~ p(x_t | u_t, x_{t-1})

  v̂ = v + sample(α₁|v| + α₂|ω|)
  ω̂ = ω + sample(α₃|v| + α₄|ω|)
  γ̂ = sample(α₅|v| + α₆|ω|)

  if |ω̂| < ε:   # straight-line fallback
    x' = x + v̂·cosθ·Δt
    y' = y + v̂·sinθ·Δt
  else:
    x' = x − (v̂/ω̂)sinθ + (v̂/ω̂)sin(θ + ω̂Δt)
    y' = y + (v̂/ω̂)cosθ − (v̂/ω̂)cos(θ + ω̂Δt)
  θ' = θ + ω̂Δt + γ̂Δt

  return (x', y', θ')ᵀ
```

**Closed-form vs. sampling.** Although both velocity algorithms share the exact same noise parameters $\alpha_1..\alpha_6$, they serve opposite directions of inference. `motion_model_velocity` *evaluates* the probability density of a given hypothesis—needed, for example, in grid localization when integrating the Bayes filter directly—whereas `sample_motion_model_velocity` *generates* a concrete successor state to advance each particle in Monte Carlo Localization (MCL, §14.7). The Python implementation below packages the 12-draw normal sampler and the straight-line fallback into a ready-to-run function:

```python
import numpy as np

def sample_normal(b):
    """Zero-mean normal approximation sample with variance b (12 uniform draws)."""
    return (np.sqrt(b) / 2.0) * sum(np.random.uniform(-1, 1) for _ in range(12))

def sample_motion_model_velocity(v, omega, x, y, theta, dt,
                                  alpha, eps=1e-6):
    """
    Velocity motion model sampling.
    alpha: [α₁, α₂, α₃, α₄, α₅, α₆]
    """
    v_hat   = v     + sample_normal(alpha[0]*abs(v) + alpha[1]*abs(omega))
    w_hat   = omega + sample_normal(alpha[2]*abs(v) + alpha[3]*abs(omega))
    g_hat   =         sample_normal(alpha[4]*abs(v) + alpha[5]*abs(omega))

    if abs(w_hat) < eps:
        x_new = x + v_hat * np.cos(theta) * dt
        y_new = y + v_hat * np.sin(theta) * dt
    else:
        r = v_hat / w_hat
        x_new = x - r * np.sin(theta) + r * np.sin(theta + w_hat * dt)
        y_new = y + r * np.cos(theta) - r * np.cos(theta + w_hat * dt)
    theta_new = theta + w_hat * dt + g_hat * dt

    return x_new, y_new, theta_new
```

Both velocity variants assume that only the commanded speed $(v, \omega)$ is available. Once wheel encoders have recorded how far the wheels actually turned, the **odometry model** replaces raw commands with measured relative displacement.

### 4.7.4 Odometry Motion Model — Closed-Form

**Intuition.** Rather than predicting motion from velocity commands, the odometry model takes the pair of poses $u_t = (\bar{x}_{t-1}, \bar{x}_t)$ integrated from wheel encoders and treats that step as its control input. Any planar displacement between two poses can be decomposed cleanly into three elementary moves—rotate toward the endpoint ($\delta_{\text{rot1}}$), translate straight across ($\delta_{\text{trans}}$), and rotate to the final heading ($\delta_{\text{rot2}}$)—which together span every planar motion.

Strictly speaking, wheel odometry is a sensor measurement rather than a control signal. Treating it as a pseudo-control is a deliberate modeling shortcut: handling odometry as a formal sensor measurement would force us to augment the state vector with velocities, inflating filter dimensionality.

**Equations.** From the odometry pose pair $u_t = (\bar{x}_{t-1}, \bar{x}_t)$, the three motion primitives are extracted directly:

$$\delta_{\text{rot1}} = \text{atan2}(\bar{y}' - \bar{y},\ \bar{x}' - \bar{x}) - \bar{\theta}$$
$$\delta_{\text{trans}} = \sqrt{(\bar{x} - \bar{x}')^2 + (\bar{y} - \bar{y}')^2}$$
$$\delta_{\text{rot2}} = \bar{\theta}' - \bar{\theta} - \delta_{\text{rot1}}$$

The same decomposition is computed for the hypothesized state transition $(x_{t-1}, x_t)$ to obtain $(\hat\delta_{\text{rot1}}, \hat\delta_{\text{trans}}, \hat\delta_{\text{rot2}})$, which parameterize four noise coefficients $\alpha_1..\alpha_4$:

$$b_{\text{rot1}} = \alpha_1|\hat\delta_{\text{rot1}}| + \alpha_2|\hat\delta_{\text{trans}}|$$
$$b_{\text{trans}} = \alpha_3|\hat\delta_{\text{trans}}| + \alpha_4(|\hat\delta_{\text{rot1}}| + |\hat\delta_{\text{rot2}}|)$$
$$b_{\text{rot2}} = \alpha_1|\hat\delta_{\text{rot2}}| + \alpha_2|\hat\delta_{\text{trans}}|$$

Each parameter has a clear physical role: $\alpha_1$ captures rotational error caused by rotation (rotational slip), $\alpha_2$ captures rotational drift caused by translation, $\alpha_3$ scales translational error with distance traveled, and $\alpha_4$ scales translational error induced by turning. Unlike the velocity model, no auxiliary $\alpha_5, \alpha_6$ trick is needed because the three independent perturbations $(\delta_{\text{rot1}}, \delta_{\text{trans}}, \delta_{\text{rot2}})$ already span the full 3D pose space. One critical implementation detail: always wrap angular differences such as $\delta_{\text{rot1}} - \hat{\delta}_{\text{rot1}}$ into $[-\pi, \pi]$, because forgetting to normalize across the $\pm\pi$ seam causes likelihoods to collapse spuriously.

Comparing the measured primitives against the inverse-model primitives yields the closed-form odometry density:

**Algorithm box (PR Table 5.5: `motion_model_odometry`).**

```
Algorithm motion_model_odometry(x_t, u_t, x_{t-1}):
  # inputs: x_t=(x',y',θ'), u_t=(x̄_{t-1}, x̄_t), x_{t-1}=(x,y,θ)
  # output: p(x_t | u_t, x_{t-1}) probability density

  # extract (δ_rot1, δ_trans, δ_rot2) from odometry measurement
  δ_rot1  = atan2(ȳ' − ȳ, x̄' − x̄) − θ̄
  δ_trans = sqrt((x̄ − x̄')² + (ȳ − ȳ')²)
  δ_rot2  = θ̄' − θ̄ − δ_rot1

  # same decomposition from the hypothesized pose pair (inverse model)
  δ̂_rot1  = atan2(y' − y, x' − x) − θ
  δ̂_trans = sqrt((x − x')² + (y − y')²)
  δ̂_rot2  = θ' − θ − δ̂_rot1

  # evaluate the three parameter differences as independent noise
  p1 = prob(δ_rot1  − δ̂_rot1,  α₁|δ̂_rot1|  + α₂|δ̂_trans|)
  p2 = prob(δ_trans − δ̂_trans, α₃|δ̂_trans| + α₄(|δ̂_rot1| + |δ̂_rot2|))
  p3 = prob(δ_rot2  − δ̂_rot2,  α₁|δ̂_rot2|  + α₂|δ̂_trans|)

  return p1 · p2 · p3
```

Because all three noise components act independently along the three decomposed degrees of freedom, the resulting probability density remains non-degenerate across $(x, y, \theta)$, letting a grid filter score arbitrary candidate poses directly.

### 4.7.5 Odometry Model — Sampling for Particle Filters

Just as velocity sampling inverted closed-form velocity evaluation, odometry sampling reverses `motion_model_odometry`: extract $(\delta_{\text{rot1}}, \delta_{\text{trans}}, \delta_{\text{rot2}})$ from the encoder step, perturb each primitive with random noise, and compose the noisy rotate–translate–rotate sequence forward from $x_{t-1}$. Eliminating the inverse pose decomposition keeps the algorithm concise and fast.

**Equations.** Composing the perturbed primitives forward gives the standard update (PR eq. 5.40):

$$\begin{pmatrix}x'\\y'\\\theta'\end{pmatrix} = \begin{pmatrix}x\\y\\\theta\end{pmatrix} + \begin{pmatrix}\hat{\delta}_{\text{trans}}\cos(\theta + \hat{\delta}_{\text{rot1}})\\\hat{\delta}_{\text{trans}}\sin(\theta + \hat{\delta}_{\text{rot1}})\\\hat{\delta}_{\text{rot1}} + \hat{\delta}_{\text{rot2}}\end{pmatrix}$$

Modeling each step as *two rotations and a straight translation* serves as a first-order approximation to a curved arc over small $\Delta t$, with the bonus that straight-line motion requires no singular $\omega \to 0$ fallback branch.

**Algorithm box (PR Table 5.6: `sample_motion_model_odometry`).**

```
Algorithm sample_motion_model_odometry(u_t, x_{t-1}):
  # inputs: u_t=(x̄_{t-1}, x̄_t), x_{t-1}=(x,y,θ)
  # output: sample x_t ~ p(x_t | u_t, x_{t-1})

  # extract relative motion from odometry
  δ_rot1  = atan2(ȳ' − ȳ, x̄' − x̄) − θ̄
  δ_trans = sqrt((x̄ − x̄')² + (ȳ − ȳ')²)
  δ_rot2  = θ̄' − θ̄ − δ_rot1

  # perturb with noise
  δ̂_rot1  = δ_rot1  − sample(α₁|δ_rot1|  + α₂|δ_trans|)
  δ̂_trans = δ_trans − sample(α₃|δ_trans| + α₄(|δ_rot1| + |δ_rot2|))
  δ̂_rot2  = δ_rot2  − sample(α₁|δ_rot2|  + α₂|δ_trans|)

  # forward composition
  x' = x + δ̂_trans · cos(θ + δ̂_rot1)
  y' = y + δ̂_trans · sin(θ + δ̂_rot1)
  θ' = θ + δ̂_rot1 + δ̂_rot2

  return (x', y', θ')ᵀ
```

In production robotics, ROS2 Nav2's `nav2_amcl` package implements this exact algorithm as its `differential` motion model for wheeled autonomous mobile robot (AMR) localization. Below is a compact Python version ready to plug into a particle filter:

```python
import numpy as np

def sample_motion_model_odometry(bar_x_prev, bar_x_curr, x, y, theta,
                                  alpha):
    """
    Odometry motion model sampling.
    bar_x_prev, bar_x_curr: odometry pose pair (x̄,ȳ,θ̄)
    alpha: [α₁, α₂, α₃, α₄]
    """
    bx,  by,  bt  = bar_x_prev
    bx_, by_, bt_ = bar_x_curr

    d_rot1  = np.arctan2(by_ - by, bx_ - bx) - bt
    d_trans = np.sqrt((bx - bx_)**2 + (by - by_)**2)
    d_rot2  = bt_ - bt - d_rot1

    def sample_normal(b):
        return (np.sqrt(b) / 2.0) * sum(np.random.uniform(-1, 1) for _ in range(12))

    dh_rot1  = d_rot1  - sample_normal(alpha[0]*abs(d_rot1)  + alpha[1]*abs(d_trans))
    dh_trans = d_trans - sample_normal(alpha[2]*abs(d_trans) + alpha[3]*(abs(d_rot1) + abs(d_rot2)))
    dh_rot2  = d_rot2  - sample_normal(alpha[0]*abs(d_rot2)  + alpha[1]*abs(d_trans))

    x_new     = x + dh_trans * np.cos(theta + dh_rot1)
    y_new     = y + dh_trans * np.sin(theta + dh_rot1)
    theta_new = theta + dh_rot1 + dh_rot2

    return x_new, y_new, theta_new
```

<!-- DEMO: probabilistic_motion_model.html -->

All four algorithms so far model open-space kinematics without considering obstacles. Yet in localization problems, a map $m$ is already available and can rule out physically impossible trajectories.

### 4.7.6 Motion + Map: Map-Conditioned Motion Model

When a map $m$ is given, conditioning the transition model on map geometry prevents particles or grid beliefs from drifting through walls.

**Equations.** Because computing the exact map-constrained transition distribution along every continuous path is intractable, localization systems rely on a practical factored approximation:

$$p(x_t \mid u_t, x_{t-1}, m) \propto p(x_t \mid u_t, x_{t-1}) \cdot p(x_t \mid m)$$

The first factor $p(x_t \mid u_t, x_{t-1})$ is simply the free-space motion model from §4.7.2–§4.7.5. The second factor $p(x_t \mid m)$ encodes map consistency: on an occupancy grid, it sits near 1 when $x_t$ lands in a free cell and drops near 0 inside an obstacle or wall.

**Effect.** In a particle filter, this factor suppresses particles that step into walls—most simply by sampling a candidate pose, checking its occupancy cell, and zeroing (or heavily penalizing) its weight if occupied. Note two subtle limitations of this shortcut: first, checking only the endpoint $x_t$ misses paths that tunnel straight across a thin wall during a large time step $\Delta t$; second, $p(x_t \mid m)$ behaves mathematically as a state prior rather than a sensor likelihood, so the product factorization is exact only under the conditional independence assumptions developed in §3.3 (Bayes' theorem and conditional independence, Ch.3).

Real occupancy grids also contain *unknown* cells alongside free and occupied space. Assigning $p(x_t \mid m) = 1$ versus an intermediate probability to unknown cells noticeably shifts localization behavior in partially mapped environments—for reference, ROS2 Nav2 treats unknown cells as free by default.

### 4.7.7 What Survived

On wheeled platforms, the sampled velocity and odometry models remain standard motion priors for particle-filter localization, with ROS2 Nav2's `nav2_amcl` (`differential` mode) directly embodying the odometry formulation. In deployment, particle counts and update rates should be tuned empirically against the target map, sensor frequency, CPU budget, beam count, and calibrated noise parameters.

**Platforms where VIO and IMU preintegration are standard.** Humanoids, aerial drones, and legged robots cannot be modeled cleanly by planar body velocities $(v, \omega)$: legged contacts follow complex intermittent slip dynamics, and drones maneuver in full $SE(3)$ rather than $SE(2)$. On these platforms, **IMU preintegration** (§14.10) supplies the motion prior—preserving the probabilistic structure $p(x_t \mid u_t, x_{t-1})$ while replacing the wheel-kinematic content entirely.

**Limits of the model.** Every formulation in this section is strictly $SE(2)$-planar and non-holonomic. Omnidirectional platforms (such as Mecanum-wheel bases) and high-speed vehicles governed by lateral tire dynamics require dedicated motion models.

**Where these models are used next.** Later chapters plug these motion models straight into full localization pipelines: §14.7 Monte Carlo Localization (MCL) invokes `sample_motion_model_odometry` inside the particle filter prediction step (Ch.14), and §14.10 compares wheeled odometry directly against IMU preintegration (Ch.14). Across both settings, $p(x_t \mid u_t, x_{t-1})$ supplies the prediction term for the Gaussian and nonparametric filter families introduced in §3.10 and §3.11 (Ch.3).

---

Across this chapter, kinematics has progressed from deterministic geometry to hardware execution and stochastic motion. Forward kinematics maps joint angles to a unique pose, inverse kinematics recovers joint configurations for a target pose, and probabilistic motion models map commands or encoder steps to distributions over $SE(2)$ poses. Combining two motion representations (velocity versus odometry) with two inference modes (closed-form density evaluation versus forward sampling) yields the four building blocks of classical mobile localization: odometry cannot plan ahead because it relies on retrospective encoder readings, yet captures physical displacement more accurately than raw velocity commands; sampling drives particle filters, whereas closed-form density evaluation serves grid-based Bayes filters.

One open question lingers over every wheel model above: each formulation layers Gaussian or triangular noise onto a nominal *no-slip* kinematic constraint. When mud, gravel, or steep inclines violate that no-slip assumption altogether, calibrating $\alpha_i$ can only compensate so far. Section §4.8 gathers the textbooks, courses, and libraries referenced throughout this chapter before Ch.5 layers mass, inertia, and forces onto our geometric foundation.

---

## 4.8 Further Reading

To build lasting fluency in kinematics and mechatronics, nothing beats working systematically through a complete textbook alongside real code.

**Textbooks:**

- **Craig, "Introduction to Robotics: Mechanics and Control"** — a classic introduction to DH parameters and manipulator kinematics using the Modified DH convention (confirm convention and prerequisites against your course syllabus).

- **Lynch & Park, "Modern Robotics: Mechanics, Planning, and Control"** — PoE-based throughout, accompanied by a free PDF and Coursera specialization. Mathematically unified, though demanding on a first pass. https://modernrobotics.org

- **Corke, "Robotics, Vision and Control"** — pairs kinematic theory directly with executable code via `robotics-toolbox-python`; the 3rd edition uses Python throughout. https://petercorke.com/rvc/

- **Siciliano et al., "Robotics: Modelling, Planning and Control"** — a comprehensive graduate reference spanning kinematics, dynamics, and control.

**Online courses:**

- Modern Robotics, Coursera (Northwestern University): https://www.coursera.org/specializations/modernrobotics
- Introduction to Robotics, Stanford CS223A (Khatib): https://see.stanford.edu/Course/CS223A

**Software / libraries:**

- robotics-toolbox-python: https://github.com/petercorke/robotics-toolbox-python
- Pinocchio (fast dynamics, differentiable kinematics): https://github.com/stack-of-tasks/pinocchio
- MoveIt2 (ROS2 motion planning): https://moveit.picknik.ai/
- Drake (simulation + optimization + control): https://drake.mit.edu/

---

## Technical Timeline

```
1955 ── DH parameters proposed (Denavit & Hartenberg)
1969 ── Stanford Arm (an early electric, computer-controlled robot arm)
1970s–80s ── Harmonic Drive becomes the standard reducer for industrial articulated-robot joints
1985 ── Product of Exponentials formalized
2019 ── MIT Mini Cheetah: QDD actuator
2019 ── MoveIt2 (ROS2-based motion planning framework)
2023 ── ALOHA: low-cost bimanual teleoperation platform
2024 ── SO-ARM100: open-source five-axis arm with a published BOM and assembly documentation
```

---

*Layering mass, inertia, and external forces onto this kinematic geometry brings us to rigid-body dynamics and control in Ch.5—shifting the question from "what angles place the joints at a target pose?" to "what torques must the actuators apply to drive motion?"*
