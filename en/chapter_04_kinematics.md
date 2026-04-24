# Ch.4 — Kinematics & Mechatronics


Place a single robot arm on a desk. What angle must each of the six motors take so that the fingertip reaches a coffee cup? Kinematics is the discipline that answers this question. And the real-world problem of actually spinning those motors, reading sensors, and running a control loop at 1kHz is mechatronics.

This chapter covers everything from the math to real hardware selection and communication protocols. Equations show up, but the goal is "getting a robot to actually move."

---

## 4.1 Why Study Kinematics

A robot manipulator is built from multiple joints and links. What we want is the position and pose of the end-effector. What we directly control, however, is the angle (or displacement) of each joint.

The mathematical description of the relationship between these two is **kinematics**.

- **Forward Kinematics (FK)**: joint angles → end-effector position/pose
- **Inverse Kinematics (IK)**: end-effector position/pose → joint angles

This is different from dynamics. Kinematics does not consider forces and masses. It is the question of "where is it," not "what force is required." Dynamics is the subject of the next chapter.

Without kinematics, you get stuck in the following situations:
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

Here R is a 3×3 rotation matrix and p is a 3×1 position vector. The key point is that this single matrix expresses the position and pose of a rigid body simultaneously, and multiple transformations can be chained through matrix multiplication.

Given a transformation T_01 between two frames and another transformation T_12:

```
T_02 = T_01 * T_12
```

This is the essence of forward kinematics. Multiply the transformation of each joint in order from the base to the end-effector.


### 4.2.2 DH Parameters (Denavit-Hartenberg)

A method proposed in 1955 by Denavit and Hartenberg. Seventy years on, it remains the industry standard. Four parameters define the relationship between two adjacent links:

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

Core idea: represent each joint as a twist (screw motion) and compute the transformation via the matrix exponential.

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

Practical advice: you must know DH parameters. The parameters that go into a URDF (robot description file) are ultimately DH-based, and all industrial robot manuals provide DH tables. PoE is theoretically cleaner and preferred in research, but not knowing DH in the field will cause trouble. Learn both.

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
> - Lynch & Park, *Modern Robotics*, Chapter 4 — the textbook with the best exposition of PoE. Free PDF and Coursera course: https://modernrobotics.org
> - Craig, *Introduction to Robotics*, Chapter 3 — the standard reference for DH parameters. Uses the Modified DH convention.
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

The method of deriving a closed-form solution. When possible, it is the fastest and most accurate.

**IK of a 2-link planar arm:**

Given a target position (x, y):

```
cos(θ_2) = (x² + y² - L1² - L2²) / (2 * L1 * L2)
θ_2 = atan2(±√(1 - cos²(θ_2)), cos(θ_2))

θ_1 = atan2(y, x) - atan2(L2*sin(θ_2), L1 + L2*cos(θ_2))
```

The ± shows that there are two solutions (elbow-up, elbow-down). This is the essential difficulty of IK.

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
| **KDL** | Numerical (Newton-Raphson) | ROS default, slow, fragile near singularities |
| **IKFast** (OpenRAVE) | Analytical (code generation) | auto-generates C++ code for specific structures. Fast |
| **TRAC-IK** | KDL + SQP dual | higher success rate than KDL, ROS package available |
| **MoveIt2 IK** | Integrates the solvers above | ROS2 ecosystem, integrated collision avoidance |
| **pinocchio** | PoE-based | modern, fast, differentiable |

```python
# Why TRAC-IK beats KDL: probability of finding a solution within the time budget
# Benchmark (Beeson & Ames, 2015):
#   KDL:     ~50-70% success rate (5ms time limit)
#   TRAC-IK: ~95-99% success rate (same condition)
```

> **Further reading**
> - Beeson & Ames, "TRAC-IK: An Open-Source Library for Improved Solving of Generic Inverse Kinematics" (2015): https://traclabs.com/projects/trac-ik/
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
> - Siciliano et al., *Robotics: Modelling, Planning and Control*, Chapter 3 — the most comprehensive treatment of the Jacobian
> - Corke, *Robotics, Vision and Control*, Chapter 8 — includes code examples and visualization: https://petercorke.com/rvc/
> - robotics-toolbox-python documentation: https://github.com/petercorke/robotics-toolbox-python

---

## 4.5 Mechatronics Basics

The math stops here. Back to reality. "Deciding" joint angles does not make the robot move. There must be motors, there must be sensors, and there must be the electronics and communication that connect them. This is mechatronics.


### 4.5.1 Actuators

**DC motor:**
The most basic actuator. Apply a voltage and it spins. Torque is proportional to current (τ = K_t * i), and back-EMF is proportional to speed (V_emf = K_e * ω). Easy to control and cheap, but the brushes wear.

**BLDC (Brushless DC) motor:**
Switches current electronically without brushes. Long life, high torque density, good efficiency. The standard for modern robots. With FOC (Field-Oriented Control), torque ripple can be minimized.

**Servo motors (Dynamixel series):**
A product that bundles motor + reducer + encoder + controller into a single unit. Robotis's Dynamixel series is the most widely used in research.

| Model | Torque (Nm) | Communication | Use |
|------|-----------|------|------|
| XL330 | 0.5 | TTL | small grippers, SO-ARM100, etc. |
| XM540 | 10.0 | RS-485 | mid-sized robot arms |
| PH54  | 44.7 | RS-485 | large manipulators, mobile robots |

Dynamixel strengths: daisy-chain wiring, position/velocity/torque control modes, adjustable PID gains, good performance for the price. Weaknesses: a ceiling on communication speed; for advanced control, custom firmware is sometimes required.

**Quasi-Direct Drive (QDD):**

The approach that drew attention with the MIT Mini Cheetah (2019). The core idea is simple: **lower the gear ratio.**

Typical robot joint: gear ratio of 100:1 or higher (harmonic drive)
QDD: gear ratio of 6:1 to 10:1 (planetary gears or belt)

Advantages of a low gear ratio:
- **High backdrivability**: when external force is applied, the joint follows naturally. Safer under collision and easier for force control.
- **High transparency**: end-effector force can be estimated from motor current alone, without a torque sensor.
- **High bandwidth**: with less friction and elasticity in the reducer, fast torque response is possible.

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
#   → the inertia felt at the end-effector is very large
#   → precise force control is difficult
#
# QDD (gear ratio 8:1):
#   reflected inertia = 8² × 0.01 = 0.64 kg·m²
#   → more than 15× lighter
#   → force control is much easier
```


**Reducer types:**

| Type | Gear ratio | Backlash | Efficiency | Price | Use |
|------|--------|--------|------|------|------|
| Planetary | 3~100:1 | medium | 85-95% | cheap | general-purpose, suited to QDD |
| Harmonic Drive | 30~320:1 | very low | 65-85% | expensive | industrial robots, precision |
| Cycloidal | 6~120:1 | low | 85-93% | medium | emerging as a recent alternative |

**Actuator selection criteria:**

Things to consider when selecting the actuator for a robot joint:

1. **Required torque**: static torque (pose holding) + dynamic torque (acceleration). Safety factor of 2-3×.
2. **Required speed**: the joint's maximum angular velocity. Determine motor RPM accounting for the gear ratio.
3. **Backdrivability**: QDD for collaborative robots or when force control is needed, otherwise harmonic drive.
4. **Size and weight**: since the actuator mounts on a link, physical constraints exist.
5. **Thermal**: check the continuous torque rating. Peak torque is only available for short bursts.

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

The most basic sensor for measuring joint angle.

*Incremental encoder*: counts pulses on two channels (A, B) to measure relative rotation. Loses position when power is cut (requires homing). Cheap, with high resolution (10,000 PPR or higher is common).

*Absolute encoder*: outputs the current position as an absolute value. Knows its position the moment power is applied. Multi-turn absolute encoders remember multiple revolutions. Expensive but no homing required. Standard on industrial robots.

```
Resolution example:
  Incremental encoder, 4096 PPR, quadrature decoding (x4)
  → resolution = 360° / (4096 × 4) = 0.022° ≈ 0.38 mrad
  → at a 100:1 reduced joint → output resolution 0.0038 mrad
```

**Torque sensors:**

Directly measure joint torque or end-effector force. Most are based on strain gauges.

*Joint Torque Sensor (JTS)*: mounted on the output side of the reducer. The KUKA LBR iiwa set the benchmark for force control by fitting a JTS to all seven joints.

*Force/Torque sensor (F/T sensor)*: mounted at the end-effector to measure six axes (Fx, Fy, Fz, Tx, Ty, Tz). Sensors from ATI Industrial Automation are the research standard. They are expensive ($3,000–$20,000).

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

Originating in the automotive industry, it has become a standard in robotics as well. Differential signaling makes it noise-resistant, and the multi-master architecture supports priority-based arbitration.

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
# max latency under 50μs is good
```

**Why 1kHz:**

Reasons the standard robot control loop runs at 1kHz (1ms):

1. **Impedance/force control**: the control frequency must be well above the mechanical resonance. Most robot arms have natural frequencies of a few to tens of Hz, so at least 10× higher (→ hundreds of Hz to 1kHz) is needed for stable control.
2. **Nyquist theorem**: controlling a 100 Hz dynamic phenomenon requires sampling at a minimum of 200 Hz; in practice 5–10× (→ 1kHz) is preferred.
3. **Communication bandwidth**: CAN bus at 1 Mbps driving ten motors at 1kHz already saturates the link. Anything beyond requires EtherCAT.
4. **Convention**: after the MIT Cheetah demonstrated dynamic walking with a CAN + 1kHz configuration, QDD + 1kHz became the de facto standard.

When higher-frequency control (5–10 kHz) is needed: very light robots (low inertia), high-speed collision response, some tactile control. These cases call for EtherCAT or FPGA-based control.

> **Further reading**
> - FreeRTOS official documentation: https://www.freertos.org/
> - PREEMPT_RT Wiki: https://wiki.linuxfoundation.org/realtime/start
> - Dynamixel SDK: https://github.com/ROBOTIS-GIT/DynamixelSDK
> - IgH EtherCAT Master (open-source for Linux): https://etherlab.org/en/ethercat/
> - SOEM (Simple Open EtherCAT Master): https://github.com/OpenEtherCATsociety/SOEM

---

## 4.6 Advanced: Workspace Analysis and Optimal Design

*If you want to become a researcher, start reading from here.*

Kinematics addresses "how to move a given robot," but also "what robot should be designed." This section covers advanced topics related to design optimization.


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

---

## 4.7 Further Reading

To study kinematics and mechatronics seriously, the most effective approach is to work through one textbook cover to cover.

**Textbooks:**

- **Craig, "Introduction to Robotics: Mechanics and Planning"** — the canonical text on DH parameters and kinematics. Uses the Modified DH convention. Best suited to the undergraduate level.

- **Lynch & Park, "Modern Robotics: Mechanics, Planning, and Control"** — PoE-based. Provides a free PDF and Coursera course, making it highly accessible. Mathematically cleaner but hard on first read. https://modernrobotics.org

- **Corke, "Robotics, Vision and Control"** — practice kinematics alongside MATLAB/Python code. robotics-toolbox-python is the companion library for this book. The 3rd edition is Python-based. https://petercorke.com/rvc/

- **Siciliano et al., "Robotics: Modelling, Planning and Control"** — the most comprehensive graduate textbook. Covers kinematics, dynamics, and control together. Thick, but correspondingly thorough.

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
1969 ── Stanford Arm (first electric computer-controlled robot arm)
1985 ── Product of Exponentials formalized
1998 ── Harmonic Drive adoption spreads in robots
2019 ── MIT Mini Cheetah: QDD actuator
2019 ── MoveIt2 (ROS2-based motion planning framework)
2023 ── ALOHA: low-cost bimanual teleoperation platform
2024 ── SO-ARM100: open-source 5-axis robot arm (under $200)
```

---

*The next chapter layers force and mass on top of this kinematics: dynamics and control. The viewpoint shifts from "sending joint angles to a desired value" to "applying a desired torque."*
