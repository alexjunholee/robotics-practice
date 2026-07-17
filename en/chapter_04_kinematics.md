# Ch.4 — Kinematics & Mechatronics


Place a single robot arm on a desk. What angle must each of the six motors take so that the fingertip reaches a coffee cup? Kinematics is the discipline that answers this question. And the real-world problem of actually spinning those motors, reading sensors, and running a control loop at 1kHz is mechatronics.

The equations connect to hardware selection and communication protocols, and eventually to motion on a physical robot.

---

## 4.1 Why Study Kinematics

A robot manipulator is built from multiple joints and links. What we want is the position and pose of the end-effector. What we directly control, however, is the angle (or displacement) of each joint.

The mathematical description of the relationship between these two is **kinematics**.

- **Forward Kinematics (FK)**: joint angles → end-effector position/pose
- **Inverse Kinematics (IK)**: end-effector position/pose → joint angles

This is different from dynamics. Kinematics does not consider forces and masses. It is the question of "where is it," not "what force is required." Dynamics is the subject of the next chapter.

Kinematics is used in the following tasks:
- Robot arm path planning (motion planning)
- Teleoperation (master-slave mapping in remote control)
- Calibration (correcting errors between the real robot and the model)
- Collision avoidance (you must know where each link sits in space to avoid it)

---

## 4.2 Forward Kinematics

### 4.2.1 Homogeneous Transformation Matrix

The basic tool of kinematics is the 4×4 homogeneous transformation matrix:

```
T = | R  p |
    | 0  1 |
```

Here R is a 3×3 rotation matrix and p is a 3×1 position vector. A homogeneous transformation matrix represents a rigid body's position and orientation together, and matrix multiplication chains multiple transformations.

Given a transformation T_01 between two frames and another transformation T_12:

```
T_02 = T_01 * T_12
```

Forward kinematics applies this composition from the base to the end-effector, multiplying each joint transformation in order.


### 4.2.2 DH Parameters (Denavit-Hartenberg)

A convention proposed in 1955 by Denavit and Hartenberg. It remains widely used in robot kinematics, with four parameters defining the relationship between adjacent links:

| Parameter | Meaning |
|---------|------|
| **a_i** (link length) | distance from z_{i-1} to z_i along the x_i axis |
| **α_i** (link twist) | rotation angle from z_{i-1} to z_i about the x_i axis |
| **d_i** (link offset) | distance from x_{i-1} to x_i along the z_{i-1} axis |
| **θ_i** (joint angle) | rotation angle from x_{i-1} to x_i about the z_{i-1} axis |

For a revolute joint, θ_i is the variable and the other three are constants.
For a prismatic joint, d_i is the variable.

The transformation matrix for each joint:

```
T_i = Rot_z(θ_i) * Trans_z(d_i) * Trans_x(a_i) * Rot_x(α_i)

    = | cos(θ)  -sin(θ)cos(α)   sin(θ)sin(α)   a*cos(θ) |
      | sin(θ)   cos(θ)cos(α)  -cos(θ)sin(α)   a*sin(θ) |
      | 0        sin(α)          cos(α)          d        |
      | 0        0               0               1        |
```

Caveat: the DH convention comes in two flavors — "standard" and "modified (Craig convention)." If you use Craig's textbook you will see modified DH; many other texts use standard DH. The two differ in how frames are attached. Mixing them yields wrong results, so always state which convention you are using.


### 4.2.3 Example: FK of a 2-link Planar Arm

Start with the simplest example. A 2-link robot arm in the plane.

```
       q1         q2
  O────────O────────O → end-effector
  (base)   L1       L2
```

DH table (standard convention):

| Link | a    | α   | d   | θ    |
|------|------|-----|-----|------|
| 1    | L1   | 0   | 0   | θ_1  |
| 2    | L2   | 0   | 0   | θ_2  |

The end-effector position follows simply from trigonometry:

```
x = L1*cos(θ_1) + L2*cos(θ_1 + θ_2)
y = L1*sin(θ_1) + L2*sin(θ_1 + θ_2)
```

Implemented in Python:

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
# Output: End-effector position: (0.259, 1.366), orientation: 75.0°
```

If this looks overly simple, that is normal. The FK of a real 6-axis arm works on the same principle — it just multiplies six 4×4 matrices.


### 4.2.4 Product of Exponentials (PoE)

As an alternative to DH parameters, there is the PoE (Product of Exponentials) method, based on Lie group / Lie algebra. This is the method adopted in Lynch & Park's "Modern Robotics."

PoE represents each joint as a twist (screw motion) and computes the transformation via the matrix exponential.

```
T(θ) = e^{[S_1]θ_1} * e^{[S_2]θ_2} * ... * e^{[S_n]θ_n} * M
```

Where:
- S_i is the screw axis of the i-th joint (6×1 vector)
- [S_i] is the 4×4 skew-symmetric matrix representation of S_i (an element of se(3))
- M is the end-effector pose when all joints are at the zero (home) configuration
- θ_i is the joint variable

**DH vs PoE comparison:**

| Item | DH | PoE |
|------|-----|-----|
| Frame attachment | a frame needed on each link | only the base frame and end-effector frame needed |
| Convention confusion | beware standard vs modified | none (though space form vs body form exists) |
| Mathematical basis | matrix multiplication | Lie group, matrix exponential |
| Singularity analysis | requires separate treatment | naturally integrated |
| Industry adoption | very high | academia-centered, spreading |
| Textbook | Craig, Siciliano | Lynch & Park |

DH parameters remain common in textbooks and industrial robot manuals, while URDF encodes the equivalent link and joint transformations directly. PoE provides a cleaner Lie-group formulation and is widely used in research. Familiarity with both conventions makes it easier to move between manuals, robot descriptions, and derivations.

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

FK is easy. Matrix multiplication suffices. The problem is IK.

"I want to place the end-effector at (x, y, z). What must each joint angle be?"

Why this problem is hard:
1. **Nonlinear equations** — trigonometric functions are tangled together
2. **Multiple solutions** — several combinations of joint angles may reach the same end-effector position (elbow-up, elbow-down, etc.)
3. **No solution may exist** — points outside the workspace are unreachable
4. **Infinitely many solutions** — if degrees of freedom remain (a redundant manipulator), the number of solutions is infinite


### 4.3.1 Analytical IK

This method derives a closed-form solution. When one exists, candidate solutions can be computed without iterative optimization, but numerical accuracy and runtime still depend on the implementation and singularity handling.

**IK of a 2-link planar arm:**

Given a target position (x, y):

```
cos(θ_2) = (x² + y² - L1² - L2²) / (2 * L1 * L2)
θ_2 = atan2(±√(1 - cos²(θ_2)), cos(θ_2))

θ_1 = atan2(y, x) - atan2(L2*sin(θ_2), L1 + L2*cos(θ_2))
```

The ± sign reveals two solutions (elbow-up and elbow-down). The existence of multiple solutions is one source of difficulty in IK.

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

Among 6-axis robots, those satisfying Pieper's condition — where the last three axes meet at a single point (a spherical wrist) — can be solved analytically. Most industrial 6-axis robots (UR, KUKA, ABB, etc.) have this structure.

In this case the position problem (first three axes) and the orientation problem (last three axes) can be decoupled and solved. Up to eight solutions exist, and it is common to pick the one that respects joint limits and stays close to the previous joint angles.


### 4.3.2 Numerical IK

When an analytical solution is not available (complex structures, 7 or more axes, non-standard structures), one must solve it numerically. This is an iterative optimization problem.

**Jacobian pseudo-inverse method:**

```
Δq = J†(q) * Δx
```

Here J† is the pseudo-inverse of the Jacobian. Iterating this converges to the target.

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

The problem with the pseudo-inverse is that joint velocities blow up near singularities. DLS mitigates this by adding a damping factor λ:

```
Δq = J^T (J * J^T + λ²I)^{-1} * Δx
```

Large λ is stable near singularities but slow to converge; small λ approaches the pseudo-inverse. Adaptive adjustment of λ (Nakamura & Hanafusa, 1986) is widely used in practice.


### 4.3.3 Singularity

A joint configuration where the rank of the Jacobian drops is called a singularity. At a singularity:

1. **The end-effector cannot move in a particular direction** — loss of a degree of freedom
2. **Joint velocities go to infinity for infinitesimal motion** — real motors cannot follow
3. **IK solutions are discontinuous** — abrupt joint jumps during path following

Singularities of the 2-link arm are simple: θ_2 = 0 (arm fully extended) or θ_2 = π (fully folded). Here the end-effector can only move in the radial direction; no tangential velocity is achievable.

Representative singularities of 6-axis robots:
- **Wrist singularity**: axes 4 and 6 are aligned (q5 ≈ 0)
- **Shoulder singularity**: the end-effector lies on axis 1
- **Elbow singularity**: the arm is fully extended

Practical countermeasures:
- Path planning that avoids the neighborhood of singularities
- Velocity limiting while passing through singularities with the DLS method
- Use of redundancy (extra degrees of freedom)


### 4.3.4 IK Solvers

It is rare to implement IK from scratch. Using a proven solver is the sensible choice.

| Solver | Method | Notes |
|------|------|------|
| **KDL** | Numerical (Newton-Raphson) | available in the ROS ecosystem; sensitive to joint limits, initialization, and singularities |
| **IKFast** (OpenRAVE) | Analytical (code generation) | auto-generates C++ code for specific structures. Fast |
| **TRAC-IK** | KDL + SQP dual | higher solve rate than stock KDL on the paper's tested chains; ROS package available |
| **MoveIt2 IK** | Integrates the solvers above | ROS2 ecosystem, integrated collision avoidance |
| **pinocchio** | PoE-based | modern, fast, differentiable |

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

The Jacobian is one of the most heavily used tools in kinematics. If FK is the problem of "position," the Jacobian is the problem of "velocity."


### 4.4.1 Joint Velocity → End-Effector Velocity

The relationship between end-effector velocity (linear v, angular ω) and joint velocity q̇:

```
ẋ = J(q) * q̇

where ẋ = [v; ω] ∈ ℝ^6 (for a 6-axis case)
      q̇ ∈ ℝ^n
      J(q) ∈ ℝ^{6×n}
```

n < 6 is under-actuated, n = 6 is fully-actuated, n > 6 is redundant.


### 4.4.2 Force/Torque Relation (Duality)

The transpose of the Jacobian maps end-effector force to joint torque:

```
τ = J^T(q) * F
```

Here τ is joint torque and F is the force/moment acting on the end-effector.

This is **static duality**. Velocity and force are dual through the Jacobian and its transpose. It follows naturally from the principle of power conservation:

```
P = F^T * ẋ = F^T * J * q̇ = (J^T * F)^T * q̇ = τ^T * q̇
```

This relation is central to force control. To apply a desired force F at the end-effector, apply joint torques τ = J^T * F.


### 4.4.3 Manipulability Ellipsoid

The Jacobian also tells how "well" the robot can move at its current configuration.

```
manipulability index = √det(J * J^T)
```

When this value is zero, the robot is at a singularity. The larger it is, the more evenly the robot can move in all directions.

The eigenvalues and eigenvectors of J * J^T trace out an ellipsoid. Large eigenvalues mean fast motion in that direction; small ones mean slow. If the eigenvalues are all similar the motion is isotropic; if they differ greatly it is anisotropic.

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

Specifying joint angles does not make the robot move. It also needs motors, sensors, and the electronics and communication that connect them. This is mechatronics.


### 4.5.1 Actuators

**DC motor:**
The most basic actuator. Apply a voltage and it spins. Torque is proportional to current (τ = K_t * i), and back-EMF is proportional to speed (V_emf = K_e * ω). Easy to control and cheap, but the brushes wear.

**BLDC (Brushless DC) motor:**
Switches current electronically without brushes. Long life, high torque density, and good efficiency make it a common choice in modern robots. FOC (Field-Oriented Control) is used to reduce torque ripple.

**Servo motors (Dynamixel series):**
A product that bundles motor + reducer + encoder + controller into a single unit. Robotis's Dynamixel is a widely used servo family on research and educational platforms.

| Model | Example advertised maximum torque (Nm) | Communication | Use |
|------|-----------|------|------|
| XL330 | 0.5 | TTL | small grippers, SO-ARM100, etc. |
| XM540 | 10.0 | RS-485 | mid-sized robot arms |
| PH54  | 44.7 | RS-485 | large manipulators, mobile robots |

Torque in the table depends on model and supply voltage, so actual selection must use each e-Manual's rated or stall conditions and continuous-duty limits. Dynamixel strengths include daisy-chain wiring, position/velocity/current-based control modes, and adjustable PID gains. Its communication, control-cycle, and thermal limits are product-specific; verify that the required bandwidth and control mode are supported by the stock firmware.

**Quasi-Direct Drive (QDD):**

The approach drew attention with the MIT Mini Cheetah (2019) and uses a **lower gear ratio**.

Typical robot joint: gear ratio of 100:1 or higher (harmonic drive)
QDD: gear ratio of 6:1 to 10:1 (planetary gears or belt)

Advantages of a low gear ratio:
- **High backdrivability**: the joint yields more readily under external force, which can simplify collision-response and force-control design.
- **High transparency**: with a suitable friction model, joint torque can be approximated from motor current more readily.
- **Potential for high bandwidth**: lower reducer friction and compliance leave more room for a fast torque response.

Drawbacks: lower output torque for the same size. For large torques, you must use a larger motor.

Recent systems using QDD:
- MIT Mini Cheetah / Cheetah 3
- ALOHA (low-cost bimanual teleop)
- Unitree robot series

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

| Type | Gear ratio | Backlash | Efficiency | Price | Use |
|------|--------|--------|------|------|------|
| Planetary | 3~100:1 | medium | 85-95% | cheap | general-purpose, suited to QDD |
| Harmonic Drive | 30~320:1 | very low | 65-85% | expensive | industrial robots, precision |
| Cycloidal | 6~120:1 | low | 85-93% | medium | emerging as a recent alternative |

Gear ratio, efficiency, and backlash vary substantially by design and product. Treat these ranges as a starting point and use the manufacturer's rated-load data for selection.

**Actuator selection criteria:**

For a robot joint, combine static torque, dynamic torque, and impact loads, then choose a design margin appropriate to load uncertainty, lifetime, and the consequence of failure. The factor of two in the code below is an illustration, not a universal rule. Convert maximum joint speed to motor RPM through the gear ratio, and evaluate backdrivability, package mass, continuous torque, and thermal limits together. The choice between QDD and a harmonic drive depends on torque density, transparency, precision, and cost requirements.

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

**Encoder:**

An encoder is the most basic sensor for measuring joint angle.

*Incremental encoder*: counts pulses on two channels (A, B) to measure relative rotation. Loses position when power is cut (requires homing). Cheap, with high resolution (10,000 PPR or higher is common).

*Absolute encoder*: outputs the current position as an absolute value. Knows its position the moment power is applied. Multi-turn absolute encoders remember multiple revolutions. They cost more but avoid homing, so they are widely used on industrial robots that must recover position after a restart.

```
Resolution example:
  Incremental encoder, 4096 PPR, quadrature decoding (x4)
  → resolution = 360° / (4096 × 4) = 0.022° ≈ 0.38 mrad
  → at a 100:1 reduced joint → output resolution 0.0038 mrad
```

**Torque sensors:**

Directly measure joint torque or end-effector force. Most are based on strain gauges.

*Joint Torque Sensor (JTS)*: mounted on the output side of the reducer. The KUKA LBR iiwa set the benchmark for force control by fitting a JTS to all seven joints.

*Force/Torque sensor (F/T sensor)*: mounted at the end-effector to measure six axes (Fx, Fy, Fz, Tx, Ty, Tz). ATI Industrial Automation and other vendors supply research sensors; select one by measurement range, resolution, overload limit, interface, and a current quotation.

**Inertial sensors (IMU):**

Already covered in Ch.2, so only briefly noted here. Accelerometer + gyroscope + (magnetometer). Used for body-pose estimation in mobile robots and legged robots. In manipulators, per-link IMUs are sometimes used for vibration damping.


### 4.5.3 Communication Protocols

How sensors and actuators connect to a microcontroller or PC. Communication causes more trouble in robot systems than one might expect. Too much latency destabilizes control; too little bandwidth drops data.

**Basic protocols:**

| Protocol | Wiring | Speed | Distance | Notes |
|---------|------|------|------|------|
| **UART** | 2-wire (TX, RX) | ~1 Mbps | ~15m | simplest, 1:1 communication |
| **SPI** | 4-wire (MOSI, MISO, SCK, CS) | ~50 Mbps | ~1m (on-PCB) | fast; multiple slaves need extra CS lines |
| **I2C** | 2-wire (SDA, SCL) | 100k~3.4 Mbps | ~1m | address-based, convenient for sensor buses |

These three are microcontroller-level basics. Robot systems need more robust protocols.

**CAN Bus:**

Originating in the automotive industry, it is also used for motor and sensor networks in robots. Differential signaling makes it noise-resistant, and the multi-master architecture supports priority-based arbitration.

- Speed: up to 1 Mbps (CAN 2.0), 5 Mbps (CAN FD)
- Distance: up to 1 km (at 125 kbps)
- Topology: bus (daisy-chain possible)

Usage in robots: communication between motor drivers and the main controller. The MIT Cheetah and many legged robots use CAN.

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

An industrial real-time Ethernet protocol. It uses standard Ethernet hardware while providing deterministic communication at the microsecond scale.

Why robots use it:
- **Speed**: 100 Mbps, synchronizing dozens to hundreds of nodes on microsecond cycles
- **Deterministic timing**: constant packet delay → suited to real-time control
- **Processing model**: slaves read and write on-the-fly as the master's frame passes through (processed as the frame flows by). Extremely high bandwidth efficiency.

KUKA, Beckhoff, and many recent research robot platforms use EtherCAT.

Drawbacks: a dedicated master stack is required (SOEM, IgH EtherCAT Master, etc.), and configuration is complicated. It is overkill at the hobbyist level.

**RS-485 / Dynamixel Protocol:**

The communication scheme used by Dynamixel servos. RS-485 is differential-signal serial communication, up to 1 Mbps, with multiple devices daisy-chained.

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

In robot control, "real-time" does not mean "fast" but **"guaranteed to complete within a specified time."** For a 1kHz control loop, every 1ms the sequence of sensor read → control computation → motor command transmission must complete. A single missed deadline can destabilize the robot.

**RTOS (Real-Time Operating System):**

| RTOS | Notes | Use |
|------|------|------|
| **FreeRTOS** | lightweight, for microcontrollers, free | STM32, ESP32, etc. |
| **Zephyr** | modern, broad hardware support, Linux Foundation | IoT, robot embedded |
| **VxWorks** | commercial, used by NASA | aerospace, industrial |

When driving motors directly from a microcontroller, use an RTOS. Set task priorities so the control loop is not pre-empted by other tasks.

**PREEMPT_RT Linux:**

The problem: ROS2 runs on Linux. But a stock Linux kernel is not real-time. The scheduler can interrupt the control thread at any time, and delays of several milliseconds can occur.

The solution: a Linux kernel patched with PREEMPT_RT. Most of the kernel's code paths are made preemptible, delivering performance close to real-time.

Setup overview:
```bash
# 1. Install a kernel with PREEMPT_RT (Ubuntu example)
sudo apt install linux-image-rt-amd64   # Debian/Ubuntu

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

1 kHz (1 ms) is a common design point for torque and impedance control, not a universal standard. Choose the period from closed-loop bandwidth, mechanical resonances, sensor and actuator latency, solver time, and jitter margin. Twice the bandwidth in the Nyquist criterion is only an anti-aliasing lower bound; it does not guarantee control performance. In practice, sample sufficiently faster than the target closed-loop bandwidth and verify the frequency response and delay margins. CAN bandwidth cannot be inferred from motor count alone: include frame size, arbitration, bus load, and feedback rate in the calculation.

Some lightweight robots, high-speed collision responses, and tactile controllers use multi-kilohertz loops. EtherCAT or an FPGA is not automatically required; choose a fieldbus, MCU, or FPGA according to the required determinism, bandwidth, and I/O structure.

> **Further reading**
> - FreeRTOS official documentation: https://www.freertos.org/
> - PREEMPT_RT Wiki: https://wiki.linuxfoundation.org/realtime/start
> - Dynamixel SDK: https://github.com/ROBOTIS-GIT/DynamixelSDK
> - IgH EtherCAT Master (open-source for Linux): https://etherlab.org/en/ethercat/
> - SOEM (Simple Open EtherCAT Master): https://github.com/OpenEtherCATsociety/SOEM

---

## 4.6 Advanced: Workspace Analysis and Optimal Design

Kinematics can also be used to compare robot designs. The following topics relate workspace geometry and manipulability to design optimization.


### 4.6.1 Reachable Workspace vs Dexterous Workspace

**Reachable workspace**: the set of all points that the end-effector can reach in at least one orientation. "How far the hand can reach."

**Dexterous workspace**: the set of points the end-effector can reach in any orientation. "Where it can move freely." Naturally a subset of the reachable workspace, and usually much smaller.

For a 6-DOF robot, the dexterous workspace can be quite limited. This is one reason 7-DOF robots exist.

Workspace analysis can be performed with a Monte Carlo method: randomly sample the joint space and compute end-effector positions via FK to build a point cloud.

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

The Jacobian's condition number (κ) indicates how "well" the robot can move at a given configuration.

```
κ(J) = σ_max / σ_min
```

σ_max and σ_min are the maximum and minimum singular values of the Jacobian.

- κ = 1: perfect isotropy. Uniform motion in every direction. Unattainable but ideal.
- κ → ∞: singularity. No motion at all in one direction.

In robot design, minimizing the condition number across the entire workspace can be the goal. This is called **kinematic optimization** or **optimal design**.

Caveat: when computing the Jacobian's condition number, linear velocity (m/s) and angular velocity (rad/s) have different units, so comparing them directly is meaningless. Normalize by a characteristic length, or analyze linear and angular velocities separately. This issue is a long-standing debate in the optimization of robot kinematics.


### 4.6.3 Redundancy Resolution (7-DOF Arms)

A 7-DOF robot arm (Kinova Gen3, KUKA LBR iiwa, Franka Emika Panda, etc.) has one extra degree of freedom relative to a 6-DOF task space. This extra freedom is called **kinematic redundancy**.

The overall arm configuration can be changed while holding the same end-effector pose. It is like a human raising or lowering the elbow while keeping the fist in place.

Strategies for exploiting this freedom:
1. **Singularity avoidance**: use the extra freedom to maximize the Jacobian's manipulability
2. **Joint-limit avoidance**: as a joint nears its limit, use the extra freedom to return toward center
3. **Obstacle avoidance**: adjust the configuration so the elbow does not collide with obstacles
4. **Energy optimization**: choose the pose that minimizes torque

Mathematically, the extra freedom corresponds to the null space of the Jacobian:

```
q̇ = J† * ẋ + (I - J† * J) * q̇_0
```

The first term is the minimum-norm joint velocity that achieves the end-effector velocity. The second term (I - J†J) is the null-space projector — it moves the joints without affecting the end-effector velocity. q̇_0 is the gradient of a secondary objective (e.g., maximizing manipulability).

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

Everything so far has been deterministic kinematics. §4.7 layers probability on top of it.

---

## 4.7 Advanced: Probabilistic Motion Models

### 4.7.1 From Determinism to Probability

§4.2 forward kinematics and §4.3 inverse kinematics are deterministic. Feed in joint angles and one end-effector position comes out; feed in an end-effector position and a set of joint angles comes out. The output is a point estimate given the input. It is the world of manipulators where the math tells you exactly where the fingertip is.

Wheeled mobile robots are different. The wheels do not roll at precisely the commanded speed. There is slip, effective radius changes with wear, and asymmetric wear on left and right wheels generates straight-line error. The result: even after issuing a control command $u_t$, the next pose $x_t$ is not a single point but a probability distribution. Formalizing that distribution is what a **probabilistic motion model** does.

The state is a planar pose, $x_t = (x, y, \theta)^T \in SE(2)$. The motion model defines the conditional probability distribution of the next pose given the previous pose $x_{t-1}$ and the control input $u_t$:

$$p(x_t \mid u_t, x_{t-1})$$

Two representations exist for this distribution.

**Velocity model**: the control input is given as linear and angular velocity, $u_t = (v, \omega)^T$. Usable at the planning stage. The error between the robot's commanded speed and its actual speed is modeled as noise.

**Odometry model**: the control input is a pair of poses measured by the wheel encoder, $u_t = (\bar{x}_{t-1}, \bar{x}_t)$. It is retrospective, so it cannot be used for planning, but since the encoder measures directly, it is more accurate than the velocity model.

For each of the two models there are two ways to use it: **closed-form density evaluation** and **sampling**. Closed-form returns a probability density value answering "how plausible is this hypothesized pose $x_t$?" That is what EKF and UKF need in their prediction step. Sampling is forward simulation that generates the next pose. A particle filter (MCL) uses this form directly. The four combinations are covered in §4.7.2–§4.7.5.


### 4.7.2 Velocity Motion Model — Closed-Form

**Intuition.** Without noise, a robot moving at linear velocity $v$ and angular velocity $\omega$ traces a circular arc. At $\omega = 0$ the arc degenerates to a straight line. Noise means the arc actually traced differs from the commanded one. Closed-form evaluation inverts this logic: given two poses $x_{t-1}$ and $x_t$, it back-computes the center $(x_c, y_c)$ and radius $r^*$ of the arc connecting them, recovers the hypothetical velocity $(\hat{v}, \hat{\omega})$ that would have produced that arc, and then evaluates the difference from the commanded velocity $(v, \omega)$ under a noise distribution.

**Equations.** Given $x_{t-1} = (x, y, \theta)^T$ and a hypothesized $x_t = (x', y', \theta')^T$:

$$\mu = \frac{1}{2} \cdot \frac{(x - x')\cos\theta + (y - y')\sin\theta}{(y - y')\cos\theta - (x - x')\sin\theta}$$

$$x_c = \frac{x + x'}{2} + \mu(y - y'), \quad y_c = \frac{y + y'}{2} + \mu(x' - x)$$

$$r^* = \sqrt{(x - x_c)^2 + (y - y_c)^2}$$

$$\Delta\theta = \text{atan2}(y' - y_c,\ x' - x_c) - \text{atan2}(y - y_c,\ x - x_c)$$

$$\hat{v} = \frac{\Delta\theta \cdot r^*}{\Delta t}, \quad \hat{\omega} = \frac{\Delta\theta}{\Delta t}, \quad \hat{\gamma} = \frac{\theta' - \theta}{\Delta t} - \hat{\omega}$$

The noise model is additive, with variance proportional to command magnitude. The second argument $b$ (variance) of `prob(a, b)` is:

$$b_v = \alpha_1|v| + \alpha_2|\omega|, \quad b_\omega = \alpha_3|v| + \alpha_4|\omega|, \quad b_\gamma = \alpha_5|v| + \alpha_6|\omega|$$

$b_v$ is the noise variance on linear velocity, $b_\omega$ on angular velocity. $\hat{\gamma}$ is a final-heading correction term. With only two noise variables $(v, \omega)$, hypothesized poses are confined to a 2D manifold within 3D pose space — a *degeneracy* problem. Adding $\hat{\gamma}$ secures full 3D support.

The six parameters physically: $\alpha_1, \alpha_2$ weight the variance of linear-velocity noise; $\alpha_3, \alpha_4$ weight angular-velocity noise; $\alpha_5, \alpha_6$ weight final-heading noise. Because variance scales linearly with command magnitude, faster motion becomes more uncertain — which matches intuition. Each robot needs its $\alpha_i$ calibrated from straight-line, circular, and figure-eight trajectories.

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

`prob(a, b)` is the density of a zero-mean normal or triangular distribution with variance $b$.

Generating a pose sample from the same noise parameters works in the opposite direction, covered next.

### 4.7.3 Velocity Motion Model — Sampling

Closed-form evaluated a hypothesized pose by inverting the arc. Sampling runs in the opposite direction: draw noise first, perturb the commanded velocity, then integrate the arc forward to produce one pose sample. A particle filter needs exactly this one sample per particle, and the implementation is simpler than the closed-form version.

Perturbed controls:

$$\hat{v} = v + \text{sample}(\alpha_1|v| + \alpha_2|\omega|)$$
$$\hat{\omega} = \omega + \text{sample}(\alpha_3|v| + \alpha_4|\omega|)$$
$$\hat{\gamma} = \text{sample}(\alpha_5|v| + \alpha_6|\omega|)$$

Forward arc integration:

$$x' = x - \frac{\hat{v}}{\hat{\omega}}\sin\theta + \frac{\hat{v}}{\hat{\omega}}\sin(\theta + \hat{\omega}\Delta t)$$
$$y' = y + \frac{\hat{v}}{\hat{\omega}}\cos\theta - \frac{\hat{v}}{\hat{\omega}}\cos(\theta + \hat{\omega}\Delta t)$$
$$\theta' = \theta + \hat{\omega}\Delta t + \hat{\gamma}\Delta t$$

Note: when $|\hat{\omega}| < \epsilon$ the above diverges. The implementation must fall back to a straight-line approximation: $x' = x + \hat{v}\cos\theta\,\Delta t,\ y' = y + \hat{v}\sin\theta\,\Delta t$.

`sample(b)` draws a zero-mean sample with variance $b$. Normal approximation: $\frac{b}{6}\sum_{i=1}^{12}\text{rand}(-1,1)$ (central-limit-theorem approximation using 12 uniform draws).

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

**Closed-form vs. sampling.** `motion_model_velocity` returns a probability density value, used alongside the Jacobian in the EKF/UKF prediction step. `sample_motion_model_velocity` produces one pose and is called directly to propagate each particle in a particle filter (MCL, §14.7). Both algorithms share the same noise parameters $\alpha_1..\alpha_6$, but their directions are opposite: closed-form *evaluates* a hypothesized pose, sampling *generates* the next pose.

```python
import numpy as np

def sample_normal(b):
    """Zero-mean normal approximation sample with variance b (12 uniform draws)."""
    return (b / 6.0) * sum(np.random.uniform(-1, 1) for _ in range(12))

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

The two velocity model variants share the same noise parameters $\alpha_1..\alpha_6$ and the same assumption that the control input is a commanded velocity $(v, \omega)$. Changing that assumption leads to the second model family.

### 4.7.4 Odometry Motion Model — Closed-Form

**Intuition.** The velocity model estimates motion from commanded velocity. The odometry model goes the other way: it treats the pair of poses $u_t = (\bar{x}_{t-1}, \bar{x}_t)$ measured by the wheel encoder as if they were the control. The relative motion between these two poses is decomposed into three parameters $(\delta_{\text{rot1}}, \delta_{\text{trans}}, \delta_{\text{rot2}})$: first rotate toward the destination, then translate straight, then correct the final heading. This decomposition can represent any planar motion.

Strictly speaking, the odometry measurement is a sensor reading, but here it is treated as a control input. Treating it as a genuine measurement model would require adding velocity to the state space, which enlarges the dimension. This is a practical simplification.

**Equations.** Extract relative motion from the odometry measurement $u_t = (\bar{x}_{t-1}, \bar{x}_t)$:

$$\delta_{\text{rot1}} = \text{atan2}(\bar{y}' - \bar{y},\ \bar{x}' - \bar{x}) - \bar{\theta}$$
$$\delta_{\text{trans}} = \sqrt{(\bar{x} - \bar{x}')^2 + (\bar{y} - \bar{y}')^2}$$
$$\delta_{\text{rot2}} = \bar{\theta}' - \bar{\theta} - \delta_{\text{rot1}}$$

Noise model (four parameters $\alpha_1..\alpha_4$). The variance argument to `prob()` depends on the $(\hat\delta_{\text{rot1}}, \hat\delta_{\text{trans}}, \hat\delta_{\text{rot2}})$ back-computed from the hypothesized pose:

$$b_{\text{rot1}} = \alpha_1|\hat\delta_{\text{rot1}}| + \alpha_2|\hat\delta_{\text{trans}}|$$
$$b_{\text{trans}} = \alpha_3|\hat\delta_{\text{trans}}| + \alpha_4(|\hat\delta_{\text{rot1}}| + |\hat\delta_{\text{rot2}}|)$$
$$b_{\text{rot2}} = \alpha_1|\hat\delta_{\text{rot2}}| + \alpha_2|\hat\delta_{\text{trans}}|$$

$\alpha_1$: how much rotation disturbs rotation (rotational slip); $\alpha_2$: how much translation disturbs rotation; $\alpha_3$: translation's own variance; $\alpha_4$: how much rotation disturbs translation. The final-heading correction trick from the velocity model ($\alpha_5, \alpha_6$) is unnecessary here. Three independent noise variables naturally secure 3D support.

One implementation note: angular differences must be wrapped to $[-\pi, \pi]$. Omitting this wrap is a common bug that causes the distribution to blow up.

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

The three parameters $(\delta_{\text{rot1}}, \delta_{\text{trans}}, \delta_{\text{rot2}})$ are treated as independent noise variables, so the result can be used directly to evaluate a hypothesized pose in the EKF/UKF prediction step.

### 4.7.5 Odometry Model — Sampling for Particle Filters

The odometry closed-form used an inverse model to evaluate a hypothesized pose. Sampling reverses the direction: add noise to the $(\delta_{\text{rot1}}, \delta_{\text{trans}}, \delta_{\text{rot2}})$ extracted from the odometry, then compose the perturbed values forward to generate a new pose. No inverse model is needed at all, making the implementation considerably simpler than the closed-form version.

**Equations.** Forward composition (PR eq. 5.40):

$$\begin{pmatrix}x'\\y'\\\theta'\end{pmatrix} = \begin{pmatrix}x\\y\\\theta\end{pmatrix} + \begin{pmatrix}\hat{\delta}_{\text{trans}}\cos(\theta + \hat{\delta}_{\text{rot1}})\\\hat{\delta}_{\text{trans}}\sin(\theta + \hat{\delta}_{\text{rot1}})\\\hat{\delta}_{\text{rot1}} + \hat{\delta}_{\text{rot2}}\end{pmatrix}$$

This approximates motion as *two rotations plus one translation* rather than a circular arc — a first-order arc approximation for small $\Delta t$. Unlike velocity sampling, there is no need for an $\omega \to 0$ branch.

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

ROS2 Nav2's `nav2_amcl` implements this as the `differential` motion model. It is the direct application to wheeled AMR localization.

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
        return (b / 6.0) * sum(np.random.uniform(-1, 1) for _ in range(12))

    dh_rot1  = d_rot1  - sample_normal(alpha[0]*abs(d_rot1)  + alpha[1]*abs(d_trans))
    dh_trans = d_trans - sample_normal(alpha[2]*abs(d_trans) + alpha[3]*(abs(d_rot1) + abs(d_rot2)))
    dh_rot2  = d_rot2  - sample_normal(alpha[0]*abs(d_rot2)  + alpha[1]*abs(d_trans))

    x_new     = x + dh_trans * np.cos(theta + dh_rot1)
    y_new     = y + dh_trans * np.sin(theta + dh_rot1)
    theta_new = theta + dh_rot1 + dh_rot2

    return x_new, y_new, theta_new
```

<!-- DEMO: probabilistic_motion_model.html -->

All four algorithms so far model motion alone, without a map. In a localization problem, a map $m$ is present.

### 4.7.6 Motion + Map: Map-Conditioned Motion Model

The models above ignored map information. In localization, a map $m$ is available, and it can be used to filter out physically impossible poses.

**Equations.** Computing the map-conditioned transition distribution exactly is hard. A practical approximate factorization:

$$p(x_t \mid u_t, x_{t-1}, m) \propto p(x_t \mid u_t, x_{t-1}) \cdot p(x_t \mid m)$$

The first factor $p(x_t \mid u_t, x_{t-1})$ is the motion model from §4.7.2–§4.7.5. The second factor $p(x_t \mid m)$ is the map-conditioned probability: in an occupancy grid, it is close to 1 when $x_t$ lies in a free cell and close to 0 in a wall or occupied cell.

**Effect.** Particles no longer walk through walls in the particle filter. The simplest implementation samples a new pose, queries the map, and sets that particle's weight to 0 (or very low) when the cell is occupied. Strictly, $p(x_t \mid m)$ acts as a prior rather than a likelihood here, so the approximation assumes the motion model and the map information are independent — a limitation worth noting.

In practice an occupancy grid has unknown regions in addition to free space. Whether $p(x_t \mid m)$ is set to 1 or to some intermediate value for unknown cells affects localization quality. ROS2 Nav2's default treats unknown cells as free.

The mathematical basis for this factorization is in §3.3 (Bayes' theorem and conditional independence, Ch.3). The product factorization is exact only when the motion model and the map prior are independent.

### 4.7.7 What Survived

The sampled velocity and odometry formulations describe motion priors for particle-filter localization on wheeled robots. ROS2 Nav2 `nav2_amcl`'s `differential` motion model corresponds to the odometry-based formulation. Choose the particle count and update rate by measurement on the target map, sensor update rate, CPU, beam count, and error parameters.

**Platforms where VIO and IMU preintegration are standard.** Humanoids, drones, and legged robots are hard to describe in terms of body velocity $(v, \omega)$. Legs bring a different slip model entirely, and drones move in SE(3) rather than SE(2). On these platforms, IMU preintegration provides the motion prior. That topic is in §14.10. The formal framework — $p(x_t \mid u_t, x_{t-1})$ — is the same; the content is entirely different.

**Limits of the model.** Every model here is SE(2)-only. Holonomic robots (Mecanum wheels) or vehicles with lateral dynamics require separate models.

**Where these models are used next.** §14.7 Monte Carlo Localization (MCL) calls `sample_motion_model_odometry` directly in the prediction step of the particle filter (Ch.14). §14.10 IMU preintegration shows how the wheeled odometry model and the IMU model compare (Ch.14). In both contexts, the $p(x_t \mid u_t, x_{t-1})$ derived here enters the prediction term of the Gaussian filter and nonparametric filter families from §3.10 and §3.11 (Ch.3).

---

Deterministic FK/IK maps input to a single output point. A probabilistic motion model maps input to an output distribution. Two models (velocity, odometry) times two uses (density evaluation, sampling) yields four combinations that are the basic building blocks of real localization systems. The odometry model, reading encoder data directly, is more accurate but cannot be used for planning; the velocity model can be used for planning but does not capture actual slip. Sampling serves the particle filter; closed-form serves the EKF/UKF.

One question remains. Every model here stacks noise on top of the kinematic constraint that wheels do not slip. In mud or on inclines — environments where that constraint itself breaks down — how far can $\alpha_i$ calibration compensate? §4.8 collects the textbooks and software referenced across this chapter.

---

## 4.8 Further Reading

To study kinematics and mechatronics seriously, the most effective approach is to work through one textbook cover to cover.

**Textbooks:**

- **Craig, "Introduction to Robotics: Mechanics and Planning"** — covers DH parameters and kinematics using the Modified DH convention. Check its prerequisites and examples against the course in which it will be used.

- **Lynch & Park, "Modern Robotics: Mechanics, Planning, and Control"** — PoE-based. Provides a free PDF and Coursera course, making it highly accessible. Mathematically cleaner but hard on first read. https://modernrobotics.org

- **Corke, "Robotics, Vision and Control"** — practice kinematics alongside MATLAB/Python code. robotics-toolbox-python is the companion library for this book. The 3rd edition is Python-based. https://petercorke.com/rvc/

- **Siciliano et al., "Robotics: Modelling, Planning and Control"** — a broad graduate text covering kinematics, dynamics, and control.

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
1985 ── Product of Exponentials formalized
1998 ── Harmonic Drive adoption spreads in robots
2019 ── MIT Mini Cheetah: QDD actuator
2019 ── MoveIt2 (ROS2-based motion planning framework)
2023 ── ALOHA: low-cost bimanual teleoperation platform
2024 ── SO-ARM100: open-source five-axis arm with a published BOM and assembly documentation
```

---

*Layering force and mass on top of this kinematics leads to dynamics and control. The viewpoint shifts from "sending joint angles to a desired value" to "applying a desired torque."*
