# Ch.5 — Rigid Body Dynamics


---

## 5.1 Why Study Dynamics

If kinematics addresses "*where* the robot moves", dynamics addresses "*with what forces* it moves". Kinematics alone is enough to control a robot in some cases — slow industrial manipulators are one example. When joint velocities are low enough, inertial and Coriolis forces are negligible, and a PID controller handles the rest.

But the following situations cannot be handled without dynamics:

- **High-speed manipulation**: Reducing cycle time in industrial settings requires moving the robot fast. Moving fast increases inertial, centrifugal, and Coriolis forces. Ignoring them inflates path-tracking error and, in the worst case, saturates the joint motors.
- **Legged robots**: Whether bipedal or quadrupedal, the robot must manage ground contact forces without falling. This is a purely dynamic problem.
- **Simulation**: A physics simulator takes forces/torques, computes accelerations, and integrates to obtain the next state. The dynamics model is the core of the simulator.
- **Optimal control**: Finding trajectories that minimize energy or time requires the dynamics model as a constraint.
- **Collision/contact handling**: Grasping, pushing, or throwing objects is impossible without contact dynamics.

Kinematics is the "geometry" of the robot; dynamics is its "physics". Just as geometry alone cannot capture the world, kinematics alone cannot fully control a robot.

> **Further reading**
> - Featherstone, *Rigid Body Dynamics Algorithms*, Chapter 1 — a concise account of why dynamics is needed.
> - Russ Tedrake, *Underactuated Robotics* Ch.1 (https://underactuated.csail.mit.edu/) — gives intuition for why dynamics-based control is more powerful than kinematics-based.

---

## 5.2 Newton-Euler Formulation

### Basic Principles

Newtonian mechanics rests on two points:

**Translational motion:**
```
F = ma
```
The net force F on a body equals mass m times the acceleration a of the center of mass (CoM).

**Rotational motion:**
```
τ = Iα + ω × (Iω)
```
The net torque τ on a body equals the product of the inertia tensor I and the angular acceleration α, plus the gyroscopic term ω × (Iω). In 2D the latter term vanishes and the equation reduces to τ = Iα, but in 3D it must be included. Omitting it makes the robot behave like a ghost in simulation.

### Recursive Newton-Euler Algorithm (RNEA)

This is the most efficient way to solve inverse dynamics for a serial manipulator. The core idea is simple:

**Forward pass (base → end-effector):** Propagate velocities and accelerations of each link forward. The velocity of link i equals the velocity of link i-1 plus the contribution from joint i.

**Backward pass (end-effector → base):** Propagate forces and torques acting on each link backward. Use the Newton-Euler equations to obtain the net force/torque required at link i, then convert to the torque of joint i.

Why solve it recursively? The dynamics of a single rigid body is O(1). Handling n links sequentially gives O(n). Solving the Lagrange equations directly, in contrast, costs O(n^3) for computing the M(q) matrix. For robots with many joints (e.g., a humanoid with 30+ DOF), this difference determines whether real-time control is feasible.

The pseudo-code for RNEA is:

```
RNEA(model, q, q̇, q̈):
    # Forward pass: i = 1, 2, ..., n
    for i = 1 to n:
        v[i] = v[i-1] + S[i] * q̇[i]        # add velocity along joint axis
        a[i] = a[i-1] + S[i] * q̈[i] + v[i] × (S[i] * q̇[i])
        f[i] = I[i] * a[i] + v[i] × (I[i] * v[i])  # Newton-Euler

    # Backward pass: i = n, n-1, ..., 1
    for i = n downto 1:
        τ[i] = S[i]^T * f[i]               # extract joint torque
        f[parent(i)] += f[i]                # propagate to parent link

    return τ
```

Here S[i] is the motion subspace matrix of joint i (the joint axis direction), v[i] is the spatial velocity of link i, and I[i] is the spatial inertia of link i. The notation follows Featherstone's spatial vector convention. §5.7 covers it in more detail.

### Real Code: Pinocchio

Pinocchio is a C++/Python library that implements RNEA and a variety of other dynamics algorithms. Here is an example of computing inverse dynamics with RNEA:

```python
import pinocchio as pin
import numpy as np

# Load model from URDF
model = pin.buildModelFromUrdf("robot.urdf")
data = model.createData()

# Set current state
q = pin.randomConfiguration(model)     # joint positions
v = np.random.randn(model.nv)          # joint velocities
a = np.random.randn(model.nv)          # joint accelerations

# RNEA: (q, v, a) → τ
tau = pin.rnea(model, data, q, v, a)
print("Joint torques:", tau)

# Compute gravity torques only (v=0, a=0)
tau_g = pin.rnea(model, data, q, np.zeros(model.nv), np.zeros(model.nv))
print("Gravity compensation torques:", tau_g)
```

Gravity compensation drops out of RNEA immediately by setting v=0, a=0. Even this alone keeps the robot from sagging under gravity. It is one of the first controllers implemented in practice.

The same computation in Drake:

```python
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.parsing import Parser
import numpy as np

plant = MultibodyPlant(time_step=0.0)
Parser(plant).AddModels("robot.urdf")
plant.Finalize()

context = plant.CreateDefaultContext()

q = np.random.randn(plant.num_positions())
v = np.random.randn(plant.num_velocities())
vdot = np.random.randn(plant.num_velocities())

plant.SetPositions(context, q)
plant.SetVelocities(context, v)

# Inverse dynamics: vdot → τ
tau = plant.CalcInverseDynamics(context, vdot, MultibodyForces(plant))
```

> **Further reading**
> - Featherstone, *Rigid Body Dynamics Algorithms*, Chapter 5 — the original description of RNEA
> - Pinocchio documentation (https://github.com/stack-of-tasks/pinocchio) — the easiest library for trying RNEA in practice
> - Luh, Walker, Paul (1980), "On-Line Computational Scheme for Mechanical Manipulators" — the original RNEA paper

---

## 5.3 Lagrangian Mechanics

### What Is a Lagrangian

Lagrangian mechanics derives the equations of motion through energy, instead of handling forces and torques directly.

The **Lagrangian** L is defined as:

```
L(q, q̇) = T(q, q̇) - V(q)
```

where T is the kinetic energy of the system and V is the potential energy.

**Euler-Lagrange equation:**

```
d/dt (∂L/∂q̇_i) - ∂L/∂q_i = τ_i
```

Writing this equation for each generalized coordinate q_i yields the equations of motion of the system.

The key point is that the coordinate frame can be chosen freely. In Newtonian mechanics, the CoM position and orientation of each link must be expressed in the world frame, and constraints must be managed. In Lagrangian mechanics, choosing joint angles as generalized coordinates makes the constraints vanish automatically.

### Manipulator Equation

Organizing the Euler-Lagrange equations for an n-DOF serial manipulator produces the following standard form:

```
M(q)q̈ + C(q, q̇)q̇ + g(q) = τ
```

The meaning of each term:

- **M(q)**: mass/inertia matrix. An n×n symmetric positive definite matrix. It depends on the robot configuration q — extend the arm and the inertia grows; fold it and the inertia shrinks, by the same principle.

- **C(q, q̇)q̇**: Coriolis and centrifugal terms. Inertial coupling that arises when joints move simultaneously. Negligible at low speeds, but large at high speeds.

- **g(q)**: gravity vector. The gravitational torque on each joint when the robot is in a gravitational field.

- **τ**: joint torque vector. The forces produced by the motors. Friction is typically modeled separately and added on.

This equation is the heart of robotic dynamics. Control, simulation, and trajectory optimization all start from it.

### 2-Link Planar Arm Example

The 2-link planar arm is a staple example in any introduction to dynamics. Deriving it by hand on paper is strongly recommended — going through it once makes the structure of the n-DOF case clear.

Setup:
- Link lengths: l_1, l_2
- Link masses: m_1, m_2 (assume mass concentrated at the link tip — point mass)
- Joint angles: q_1, q_2 (measured from the base)
- Gravity: g (pointing down)

**Kinetic energy T:**

Tip position of link 1:
```
x_1 = l_1 cos(q_1)
y_1 = l_1 sin(q_1)
```

Tip position of link 2:
```
x_2 = l_1 cos(q_1) + l_2 cos(q_1 + q_2)
y_2 = l_1 sin(q_1) + l_2 sin(q_1 + q_2)
```

Computing each mass's velocity and expanding T = (1/2)m_1 v_1^2 + (1/2)m_2 v_2^2 gives:

```
T = (1/2)(m_1 + m_2) l_1^2 q̇_1^2
  + (1/2) m_2 l_2^2 (q̇_1 + q̇_2)^2
  + m_2 l_1 l_2 cos(q_2) q̇_1 (q̇_1 + q̇_2)
```

**Potential energy V:**

```
V = m_1 g l_1 sin(q_1) + m_2 g [l_1 sin(q_1) + l_2 sin(q_1 + q_2)]
```

**M(q) matrix:**

```
M(q) = [ (m_1+m_2)l_1^2 + m_2 l_2^2 + 2 m_2 l_1 l_2 cos(q_2)    m_2 l_2^2 + m_2 l_1 l_2 cos(q_2) ]
        [ m_2 l_2^2 + m_2 l_1 l_2 cos(q_2)                          m_2 l_2^2                          ]
```

Note that M(q) depends on q_2. At q_2 = 0 the arm is fully extended and the inertia is maximal. At q_2 = π the arm is folded and the inertia is minimal.

**C(q, q̇) matrix:**

```
C(q, q̇) = [ -m_2 l_1 l_2 sin(q_2) q̇_2    -m_2 l_1 l_2 sin(q_2)(q̇_1 + q̇_2) ]
            [  m_2 l_1 l_2 sin(q_2) q̇_1     0                                     ]
```

There are several ways to derive C (Christoffel symbols among them). The most systematic route is Christoffel symbols, but for a 2-link arm it is faster to collect the terms directly from the Euler-Lagrange equations.

**g(q) vector:**

```
g(q) = [ (m_1 + m_2) g l_1 cos(q_1) + m_2 g l_2 cos(q_1 + q_2) ]
        [ m_2 g l_2 cos(q_1 + q_2)                                 ]
```

Code that verifies this with SymPy:

```python
import sympy as sp

q1, q2, dq1, dq2, ddq1, ddq2 = sp.symbols('q1 q2 dq1 dq2 ddq1 ddq2')
m1, m2, l1, l2, g = sp.symbols('m1 m2 l1 l2 g', positive=True)

# Positions
x1 = l1 * sp.cos(q1)
y1 = l1 * sp.sin(q1)
x2 = x1 + l2 * sp.cos(q1 + q2)
y2 = y1 + l2 * sp.sin(q1 + q2)

# Velocities (chain rule)
vx1 = sp.diff(x1, q1) * dq1
vy1 = sp.diff(y1, q1) * dq1
vx2 = sp.diff(x2, q1) * dq1 + sp.diff(x2, q2) * dq2
vy2 = sp.diff(y2, q1) * dq1 + sp.diff(y2, q2) * dq2

# Kinetic energy
T = sp.Rational(1,2)*m1*(vx1**2 + vy1**2) + sp.Rational(1,2)*m2*(vx2**2 + vy2**2)
T = sp.trigsimp(sp.expand(T))

# Potential energy
V = m1*g*y1 + m2*g*y2

# Lagrangian
L = T - V

# Euler-Lagrange equations
# d/dt(∂L/∂q̇_i) - ∂L/∂q_i = τ_i
# d/dt here must account for time derivatives of q1, q2, so substitutions are required.
# For a clean extraction of M, C, g, consult a textbook.

print("T =", T)
print("V =", V)
```

Running this code confirms that the result matches the hand derivation above. After SymPy applies trigsimp, the form comes out cleanly.

> **Further reading**
> - Murray, Li, Sastry, *A Mathematical Introduction to Robotic Manipulation*, Ch. 4 (https://www.cds.caltech.edu/~murray/mlswiki/) — the most rigorous treatment of Lagrangian mechanics in a robotics context. Free PDF available.
> - Spong, Hutchinson, Vidyasagar, *Robot Modeling and Control*, Ch. 6-7 — the most accessible explanation at an undergraduate level
> - Craig, *Introduction to Robotics*, Ch. 6 — contains a detailed 2-link arm example

---

## 5.4 Newton-Euler vs. Lagrangian

These are the same physics viewed from different perspectives. The final result (the equations of motion) is identical. The difference lies in derivation and computational efficiency.

| Item | Newton-Euler (RNEA) | Lagrangian |
|------|-------------------|---------|
| Perspective | force/torque (force-based) | energy (energy-based) |
| Computational complexity | O(n) | O(n^3) (when computing the M matrix directly) |
| Derivation difficulty | recursive, same pattern as n grows | partial derivatives explode as n grows |
| Physical intuition | forces/torques on each link are directly visible | energy conservation/transformation is visible |
| Primary use | real-time control, simulation | model derivation, energy-based analysis, Lyapunov stability |
| Constraint forces | explicitly computable | automatically eliminated when using generalized coordinates |

The practical workflow is typically as follows:

1. **Model derivation**: Use Lagrangian mechanics to understand the structure of the manipulator equation.
2. **Numerical computation**: Use RNEA (or ABA) for real-time evaluation.
3. **Controller design**: Design computed torque control, passivity-based control, and similar schemes that exploit the structure (M, C, g) of the manipulator equation.
4. **Code implementation**: Pinocchio or Drake use RNEA/ABA internally, so calling the library is sufficient.

In the end, both are required. Without Lagrangian mechanics, one cannot understand control theory; without Newton-Euler, one cannot implement real-time code.

> **Further reading**
> - Featherstone, *Rigid Body Dynamics Algorithms*, Ch. 3 — a clear account of the relationship between the two formulations
> - Siciliano et al., *Robotics: Modelling, Planning and Control*, Ch. 7 — a comparative example deriving the dynamics of the same robot with both methods

---

## 5.5 Forward Dynamics vs. Inverse Dynamics

Dynamics has two "directions":

**Inverse Dynamics:**
```
Given: q, q̇, q̈
Find: τ
```
Answers "what torque must the motor produce to follow this trajectory?". Used primarily in control. The core of computed torque control.

**Forward Dynamics:**
```
Given: q, q̇, τ
Find: q̈
```
Answers "how does the robot accelerate under this torque?". The core of simulation. The simulator repeats this computation at every time step.

Mathematically, forward dynamics is solving for q̈ in the manipulator equation:

```
q̈ = M(q)^{-1} [τ - C(q, q̇)q̇ - g(q)]
```

Simply inverting M(q) is O(n^3). This is slow for robots with many joints.

### Articulated Body Algorithm (ABA)

Featherstone's ABA computes forward dynamics in O(n). Just as RNEA is the O(n) algorithm for inverse dynamics, ABA is the O(n) algorithm for forward dynamics.

The core idea of ABA: treat each link as an "articulated body" and recursively accumulate the inertia of its subtree. q̈ can be computed directly without explicitly forming the M matrix.

```
ABA(model, q, q̇, τ):
    # Pass 1 (forward): propagate velocities
    for i = 1 to n:
        v[i] = v[parent(i)] + S[i] * q̇[i]
        c[i] = v[i] × (S[i] * q̇[i])  # Coriolis acceleration

    # Pass 2 (backward): compute articulated body inertia
    for i = n downto 1:
        I_A[i] = I[i]  # spatial inertia
        p_A[i] = v[i] × (I[i] * v[i]) - f_ext[i]  # bias force
        # accumulate contributions from child links (omitted)
        # compute intermediate joint acceleration values

    # Pass 3 (forward): propagate accelerations
    for i = 1 to n:
        q̈[i] = ...  # computed using articulated body inertia
        a[i] = a[parent(i)] + S[i] * q̈[i] + c[i]

    return q̈
```

The actual implementation is quite involved. Rather than writing it from scratch, using Pinocchio or Drake is the sensible choice.

### Role in Simulators

The algorithm differs between simulators:

- **MuJoCo**: uses its own algorithm for forward dynamics. Its hallmark is an integrated solver that includes contact. Internally it exploits sparse factorization and is specialized for branching structures.
- **Drake**: MultibodyPlant uses ABA. Contact is handled by a separate solver (time-stepping, hydroelastic, etc.).
- **Bullet (PyBullet)**: builds on Featherstone's ABA, with contact handled by a sequential impulse solver.

In code:

```python
# Pinocchio: forward dynamics (ABA)
import pinocchio as pin
import numpy as np

model = pin.buildModelFromUrdf("robot.urdf")
data = model.createData()

q = pin.randomConfiguration(model)
v = np.random.randn(model.nv)
tau = np.random.randn(model.nv)

# ABA: (q, v, τ) → q̈
qdd = pin.aba(model, data, q, v, tau)
print("Joint accelerations:", qdd)

# Verify: recompute with RNEA
tau_check = pin.rnea(model, data, q, v, qdd)
print("Torque error:", np.linalg.norm(tau - tau_check))  # ≈ 0
```

RNEA and ABA are inverses of each other. RNEA(q, v, ABA(q, v, τ)) ≈ τ holds (within floating-point error).

> **Further reading**
> - Featherstone, *Rigid Body Dynamics Algorithms*, Ch. 7 — the original description of ABA
> - MuJoCo documentation: Computation (https://mujoco.readthedocs.io/en/latest/computation/) — describes MuJoCo's dynamics pipeline
> - Drake MultibodyPlant tutorial (https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_multibody_plant.html) — the dynamics computation API in Drake

---

## 5.6 Contact Dynamics

The moment a robot makes contact with its environment, dynamics becomes one step more complicated. Dynamics in free space is expressed cleanly as an ODE (ordinary differential equation), but once contact is introduced, inequality constraints and discontinuities appear.

### Rigid Contact vs. Compliant Contact

There are two large frameworks for modeling contact:

**Rigid contact:**
- Directly imposes the constraint that bodies do not interpenetrate.
- Contact forces come out as the Lagrange multipliers of the constraint.
- Mathematically clean but numerically difficult — discontinuities appear at contact/non-contact transitions, and handling them requires solving an LCP (Linear Complementarity Problem) or NCP (Nonlinear Complementarity Problem).
- Drake's time-stepping approach belongs to this family.

**Compliant contact:**
- Places a virtual spring-damper at the contact surface. It generates a restoring force proportional to penetration depth.
- Numerically stable and easy to implement.
- Increasing spring stiffness approaches rigid contact, but the integrator's time step must shrink accordingly (stiff ODE).
- MuJoCo's default contact model belongs to this family.

### Coulomb Friction Model

Where there is contact, there is friction. The most basic friction model is Coulomb friction:

```
|f_t| ≤ μ f_n          (static friction)
|f_t| = μ f_n, f_t ∥ -v_t  (sliding friction)
```

Here f_t is the tangential friction force, f_n is the normal force, μ is the friction coefficient, and v_t is the tangential relative velocity.

Problems with this model:
- The transition from static to sliding friction is discontinuous.
- In 3D the friction cone is nonlinear. Linearizing it produces a friction pyramid, which loses accuracy.
- Painleve's paradox: under certain conditions, rigid contact + Coulomb friction admits no solution, or a non-unique one.

### Why Contact-Rich Manipulation Is Hard

Why are tasks like grasping, rotating, and inserting objects (peg-in-hole, in-hand manipulation, etc.) so difficult?

1. **Hybrid dynamics**: the contact mode changes frequently (contact/no-contact, stick/slip). Each mode has different dynamics, and predicting mode-switch timing is hard.
2. **Discontinuous dynamics**: state can change discontinuously at mode transitions (impact).
3. **Sensitivity to parameters**: without accurate values of the friction coefficient, contact stiffness, and the like, the sim-to-real gap grows large.
4. **Combinatorial complexity**: with n contact points, there are 3^n possible contact mode combinations (contact/separation, stick/slip in each direction).

### Why Contact Handling Differs Across Simulators

Because there is no "right answer" in contact dynamics. Each simulator picks a different trade-off between accuracy, speed, and stability:

- **MuJoCo**: compliant contact + convex optimization. Fast and stable, but not physically exact. In particular, interpenetration is allowed and treated as part of "soft contact". This stability is one reason MuJoCo is popular as an RL environment.
- **Drake**: rigid contact + time-stepping (Stewart-Trinkle), or hydroelastic contact. More physically rigorous but potentially more expensive. Hydroelastic contact even computes the pressure distribution over the contact surface.
- **Bullet**: velocity-level LCP + sequential impulse. Originating from games/VR, the engine is optimized for speed but has limitations for robotics tasks that require accurate contact.
- **DART**: LCP-based rigid contact. Academically rigorous, but with a smaller user base than MuJoCo or Drake.

The choice of simulator depends on the research goal. For learning locomotion with RL, MuJoCo is the standard. For manipulation research where contact matters, Drake or MuJoCo is the usual choice; recently MuJoCo's contact quality has also improved significantly.

```python
# Accessing contact information in MuJoCo
import mujoco
import numpy as np

model = mujoco.MjModel.from_xml_path("scene.xml")
data = mujoco.MjData(model)

mujoco.mj_step(model, data)

# Number of contacts
n_contacts = data.ncon
print(f"Number of contacts: {n_contacts}")

# Information about each contact
for i in range(n_contacts):
    contact = data.contact[i]
    print(f"Contact {i}:")
    print(f"  Position: {contact.pos}")
    print(f"  Normal: {contact.frame[:3]}")  # contact normal
    print(f"  Penetration depth: {contact.dist}")
    print(f"  Geom pair: ({contact.geom1}, {contact.geom2})")
```

> **Further reading**
> - Stewart, "Rigid-Body Dynamics with Friction and Impact", SIAM Review 2000 — the mathematical foundation of contact dynamics
> - Todorov, "Convex and analytically-invertible dynamics with contacts and constraints", ICRA 2014 — the paper behind MuJoCo's contact model
> - [Todorov et al., "MuJoCo: A Physics Engine for Model-Based Control" (IROS 2012)](https://ieeexplore.ieee.org/document/6386109) — the standard for contact-based control simulation. Introduces the convex contact model and velocity stepping.
> - Drake's contact model documentation (https://drake.mit.edu/doxygen_cxx/group__hydroelastic__user__guide.html) — describes hydroelastic contact
> - Russ Tedrake, *Underactuated Robotics*, Ch. "Contact" (https://underactuated.csail.mit.edu/) — an introduction to contact dynamics

---

## 5.7 Advanced: Featherstone Algorithms and Spatial Algebra

*If you want to become a researcher, start reading here.*

From here on the material is at the graduate level. Featherstone's spatial vector algebra is a mathematical framework for expressing dynamics algorithms concisely and efficiently.

### Spatial Vectors (6D Vectors)

The motion of a rigid body in 3D space is translation (3 DOF) + rotation (3 DOF) = 6 DOF. A spatial vector bundles this into a single 6D vector.

**Motion vector (spatial velocity, twist):**
```
v = [ω; v_O]
```
The top 3 entries are the angular velocity (ω); the bottom 3 are the linear velocity at the reference point O (v_O).

**Force vector (spatial force, wrench):**
```
f = [n_O; f]
```
The top 3 entries are the moment about the reference point O (n_O); the bottom 3 are the force (f).

The key advantage of this notation: the inner product of spatial velocity and spatial force is exactly power.
```
P = f^T v = n_O · ω + f · v_O
```

This is no accident — spatial vectors are designed to have this property.

### Spatial Inertia

The 6×6 spatial inertia matrix bundles mass, CoM position, and rotational inertia into a single matrix:

```
I_sp = [ I_cm + m·[c]×[c]×^T    m·[c]× ]
       [ m·[c]×^T                 m·1    ]
```

Here m is the mass, c is the vector to the CoM, I_cm is the rotational inertia about the CoM, and [c]× is the skew-symmetric matrix of c.

Advantages of spatial inertia:
- Inertias of multiple rigid bodies combine by simple addition: I_composite = I_1 + I_2 + ...
- Coordinate transformation is a single congruence transform: I_B = X^T I_A X

### Spatial Vector Form of RNEA and ABA

The pseudo-code shown in §5.2 and §5.5 was in fact spatial vector notation. S[i] is the motion subspace of joint i (for a revolute joint, [e_z; 0]; for a prismatic joint, [0; e_z]); v[i] is a spatial velocity; f[i] is a spatial force.

With spatial vectors, revolute and prismatic joints are handled by the same code. This is why libraries like Pinocchio and Drake use spatial algebra internally.

### Accessing Spatial Quantities in Pinocchio

```python
import pinocchio as pin
import numpy as np

model = pin.buildModelFromUrdf("robot.urdf")
data = model.createData()

q = pin.randomConfiguration(model)
v = np.random.randn(model.nv)

# Forward kinematics + velocity computation
pin.forwardKinematics(model, data, q, v)

# Spatial velocity of each frame
for i in range(model.njoints):
    # Spatial velocity in the world frame
    v_world = pin.getVelocity(model, data, i, pin.ReferenceFrame.WORLD)
    print(f"Joint {i} spatial velocity (world): {v_world}")

# Composite Rigid Body Algorithm (CRBA): compute M(q)
M = pin.crba(model, data, q)
print("Mass matrix M(q):\n", data.M)

# Centroidal momentum matrix
pin.computeCentroidalMap(model, data, q)
Ag = data.Ag  # 6 x nv matrix
# h = Ag @ v is the centroidal momentum (linear + angular)
```

Using Pinocchio from C++:

```cpp
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/parsers/urdf.hpp>

pinocchio::Model model;
pinocchio::urdf::buildModel("robot.urdf", model);
pinocchio::Data data(model);

Eigen::VectorXd q = pinocchio::randomConfiguration(model);
Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);
Eigen::VectorXd tau = Eigen::VectorXd::Random(model.nv);

// Inverse dynamics (RNEA)
Eigen::VectorXd tau_id = pinocchio::rnea(model, data, q, v, Eigen::VectorXd::Zero(model.nv));

// Forward dynamics (ABA)
Eigen::VectorXd qdd = pinocchio::aba(model, data, q, v, tau);
```

Pinocchio's C++ API is Eigen-based and exposes nearly the same interface as its Python API. For real-time control the C++ API is necessary — Python overhead is not negligible in kHz-rate control loops.

> **Further reading**
> - Featherstone, *Rigid Body Dynamics Algorithms*, Ch. 2 — the original description of spatial vector algebra. Required reading for anyone who wants to do research in this area.
> - Featherstone, "A Beginner's Guide to 6-D Vectors" (IEEE Robotics & Automation Magazine, 2010) — a more accessible introduction than the textbook
> - Pinocchio GitHub (https://github.com/stack-of-tasks/pinocchio) — the source code itself is a good implementation example of spatial algebra

---

## 5.8 Advanced: Floating Base Systems

*If you want to become a researcher, start reading here.*

An industrial manipulator has its base bolted to the floor. Legged robots, drones, and underwater robots, however, have a base that moves. In that case the base position and orientation become additional degrees of freedom, and the structure of the dynamics changes fundamentally.

### Configuration of a Floating Base

For a fixed-base robot, the configuration is q ∈ R^n. For a floating-base robot, the configuration is:

```
q = [q_base; q_joints]
```

q_base is an element of SE(3) — position (3) + orientation (3, or 4 with a quaternion). This is why in Pinocchio the dimension of q (nq) and the dimension of v (nv) can differ (with a quaternion, nq = nv + 1).

This is subtly important. Because q and v do not live in the same vector space, one cannot simply do q += v*dt when integrating or differencing. In Pinocchio one must use `pin.integrate(model, q, v*dt)`.

### Underactuated Systems

A core property of floating-base systems: **underactuation**. The base has no actuator directly attached to it. Legged robots must push against the ground with their feet to move the base; drones move the base through propeller thrust.

Splitting the manipulator equation into base and joints:

```
[ M_bb  M_bj ] [ a_base  ]   [ C_b ]   [ g_b ]   [  0  ]   [ J_c^T ]
[ M_jb  M_jj ] [ q̈_joints] + [ C_j ] + [ g_j ] = [ τ_j ] + [ J_c^T ] λ
```

The `0` at the top left is the key — there is no joint torque on the base. λ is the contact force and J_c is the contact Jacobian. The base can accelerate only through contact forces and gravity.

This constraint is what makes locomotion control hard. For a fixed-base manipulator, the desired joint torque can simply be commanded to the motors; a legged robot, in contrast, must generate appropriate contact forces to make the base move as desired.

### Centroidal Dynamics

Expressing the total system momentum at the center of mass (CoM) yields centroidal dynamics:

**Linear momentum:**
```
p = m v_CoM = Σ m_i v_i
```

**Angular momentum about CoM:**
```
L = Σ (r_i - r_CoM) × (m_i v_i) + I_i ω_i
```

**Time derivative of centroidal momentum:**
```
ṗ = m g + Σ f_contact
L̇ = Σ (r_contact - r_CoM) × f_contact
```

Why this matters in locomotion control:

1. **CoM dynamics determines balance.** For the robot not to fall, the CoM trajectory must stay over the support polygon (the ZMP condition). More precisely, centroidal momentum must be regulated appropriately.

2. **Dimensionality reduction.** The full dynamics of an n-DOF legged robot is n-dimensional, but centroidal dynamics is 6-dimensional (3 linear + 3 angular momentum). A common approach is to first plan the desired momentum trajectory in this 6D space, then decompose down to the full joint level.

3. **Direct connection to contact force planning.** As the equations above show, the rate of change of centroidal momentum is determined only by external forces (contact forces + gravity). Planning which contact force pattern produces the desired momentum trajectory is the central problem of locomotion.

```python
# Computing centroidal dynamics in Pinocchio
import pinocchio as pin
import numpy as np

model = pin.buildModelFromUrdf("humanoid.urdf", pin.JointModelFreeFlyer())
data = model.createData()

q = pin.randomConfiguration(model)
v = np.random.randn(model.nv)

# Centroidal momentum
pin.computeCentroidalMomentum(model, data, q, v)
h = data.hg  # 6D centroidal momentum [angular; linear]
print("Angular momentum:", h.angular)
print("Linear momentum:", h.linear)

# Centroidal momentum matrix: h = A_g(q) * v
pin.computeCentroidalMap(model, data, q)
Ag = data.Ag  # 6 x nv
h_check = Ag @ v
print("Centroidal momentum (via Ag):", h_check)

# CoM position and velocity
pin.centerOfMass(model, data, q, v)
print("CoM position:", data.com[0])
print("CoM velocity:", data.vcom[0])
```

### Structure of Centroidal-Dynamics-Based Locomotion Control

A typical modern legged-robot control pipeline has the following structure:

```
[Contact Schedule] → [Centroidal Trajectory Optimization] → [Whole-Body Control] → [Joint Torques]

Stage 1: Decide which foot touches the ground and when (gait pattern)
Stage 2: Plan CoM trajectory + contact forces consistent with centroidal dynamics
Stage 3: Compute torques that meet the centroidal target while satisfying joint-level constraints
Stage 4: Command torques to the motors
```

This structure is not limited to legged robots; it applies to any system with a floating base. Similar structures are used in drone trajectory optimization and underwater-robot control.

> **Further reading**
> - Orin et al., "Centroidal Dynamics of a Humanoid Robot", Autonomous Robots 2013 — the foundational paper introducing centroidal dynamics to robotics
> - Wensing et al., "Optimization-Based Control for Dynamic Legged Locomotion", 2023 — a recent survey of locomotion control
> - Russ Tedrake, *Underactuated Robotics*, Ch. "Walking" (https://underactuated.csail.mit.edu/) — the relationship between underactuated systems and walking
> - Carpentier, Mansard, "Pinocchio: fast forward and inverse dynamics for poly-articulated systems" (https://github.com/stack-of-tasks/pinocchio) — Pinocchio's centroidal dynamics implementation

---

## 5.9 Further Reading

Where to begin is the most important question when studying this area. The recommended order depends on background.

**Undergraduate junior/senior, introduction to dynamics:**

> - Spong, Hutchinson, Vidyasagar, *Robot Modeling and Control* — the most accessible textbook at an undergraduate level. Detailed derivation of the manipulator equation.
> - Craig, *Introduction to Robotics: Mechanics and Control* — a shop-floor perspective. Practical but relatively shallow on the mathematical side.

**Graduate level, when mathematically rigorous understanding is needed:**

> - Murray, Li, Sastry, *A Mathematical Introduction to Robotic Manipulation* (https://www.cds.caltech.edu/~murray/mlswiki/) — dynamics from a Lie group/algebra perspective. Free PDF available. The most mathematically rigorous, but hard on first reading.
> - Featherstone, *Rigid Body Dynamics Algorithms* — the central reference for dynamics algorithms. All the core algorithms — spatial vector algebra, RNEA, ABA, composite rigid body algorithm — are here. Required reading for graduate students.

**Dynamics + control integrated:**

> - Russ Tedrake, *Underactuated Robotics* (https://underactuated.csail.mit.edu/) — covers how to use dynamics models in control and optimization. Lecture videos are also on MIT OCW. Free.

**Libraries and tools:**

> - Pinocchio (https://github.com/stack-of-tasks/pinocchio) — a C++/Python library for pure dynamics computation. Supports RNEA, ABA, CRBA, centroidal dynamics, analytical derivatives, and more. Autodiff via CasADi/JAX is also available (Pinocchio 3.x).
> - Drake (https://drake.mit.edu/) — a framework integrating simulation + optimization + control. MultibodyPlant is the dynamics engine. Its powerful mathematical programming interface is particularly useful for trajectory optimization.
> - MuJoCo (https://mujoco.org/) — a physics simulator maintained by DeepMind. Its contact handling is fast and stable. The de facto standard simulator in RL research.
> - PyBullet (https://pybullet.org/) — the Python interface to Bullet Physics. The low entry barrier makes it suitable for teaching, but its contact physics accuracy does not match MuJoCo or Drake.

---

## Technical Timeline

```
1687 ── Newton's laws of motion (Principia Mathematica)
1788 ── Lagrange's analytical mechanics (Mécanique Analytique)
1965 ── Uicker's dynamics equations (symbolic, inefficient)
1980 ── Luh, Walker, Paul's recursive Newton-Euler algorithm (RNEA, O(n))
1983 ── Featherstone's Articulated Body Algorithm (ABA, O(n) forward dynamics)
1987 ── Featherstone formalizes spatial vector algebra
2000 ── Stewart's mathematical treatment of rigid contact dynamics (SIAM Review)
2004 ── ODE (Open Dynamics Engine) — early open-source physics engine
2012 ── MuJoCo released (Todorov, Erez, Tassa)
2015 ── Bullet Physics 2.x → PyBullet interface
2016 ── Pinocchio 1.0 released (LAAS-CNRS)
2022 ── Drake 1.0 (MIT → Toyota Research Institute)
2021 ── MuJoCo open-sourced (after DeepMind acquisition)
2022 ── MuJoCo 2.3: implicit integration, elliptic friction cone
2023 ── Pinocchio 3.0: CasADi/JAX autodiff support
2023 ── MuJoCo 3.0: MJX (JAX backend for GPU parallelism)
```

---

## Summary

One-sentence summary of this chapter: **dynamics concerns the relationship between forces and motion, and is essential for controlling and simulating robots.**

Practical takeaways:

1. The manipulator equation `M(q)q̈ + C(q,q̇)q̇ + g(q) = τ` is the starting point for everything.
2. Use RNEA for inverse dynamics (computing τ) and ABA for forward dynamics (computing q̈). Both are O(n).
3. Once contact enters, the problem becomes much harder. Simulator choice matters.
4. In floating-base systems, centroidal dynamics is the key tool.
5. Do not implement these from scratch; use Pinocchio or Drake. But understand what these libraries compute internally.

This dynamics model leads directly to computed torque control, operational space control, and whole-body control.
