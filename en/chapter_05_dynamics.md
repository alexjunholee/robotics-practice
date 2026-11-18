# Ch.5 — Rigid Body Dynamics

Kinematics in Ch. 4 describes how joint angles map to link poses, velocities, and workspaces without asking what actually moves the joints. To command real motors, simulate physical contact, or keep a legged robot upright, we need to bring in mass, inertia, and force—the domain of rigid body dynamics.

---

## 5.1 Why Study Dynamics

Where kinematics asks *where* a robot moves, dynamics asks *what forces* produce that motion. For slow industrial manipulators, kinematics alone can take you surprisingly far: when joint speeds stay low, inertial and Coriolis forces are negligible, and a basic PID loop can absorb what little remains.

Once you leave that slow, fixed-base regime, dynamics becomes indispensable:

- **High-speed manipulation**: Shortening industrial cycle times means accelerating joints rapidly. High speeds amplify inertial, centrifugal, and Coriolis forces; ignoring them widens tracking error and can saturate the actuators.
- **Legged robots**: Walking and running—whether on two legs or four—require regulating ground contact forces at every step so the body does not tip over. Balancing and locomotion are inherently dynamic problems.
- **Simulation**: Every physics engine runs forward dynamics at its core—taking applied forces and torques, solving for link accelerations, and integrating forward to the next state.
- **Optimal control**: Planning trajectories that minimize energy consumption or travel time requires the dynamics model directly as a physical constraint.
- **Collision and contact handling**: Tasks like grasping, pushing, or throwing objects depend on how forces transfer across making and breaking contacts.

In short, kinematics captures the geometry of motion, whereas dynamics captures its physics—connecting applied torques to resulting accelerations.

> **Further reading**
> - Featherstone, *Rigid Body Dynamics Algorithms*, Chapter 1 — a concise account of why dynamics is needed.
> - Russ Tedrake, *Underactuated Robotics* Ch.1 (https://underactuated.csail.mit.edu/) — gives intuition for why dynamics-based control is more capable than kinematics-based control.

---

## 5.2 Newton-Euler Formulation

### Basic Principles

Every link in a robot manipulator is a rigid body coupled to its neighbors through joints. Before analyzing an entire kinematic chain, we start with the equations of motion for a single rigid body, where Newtonian mechanics treats translation and rotation separately.

**Translational motion:**
```
F = ma
```
The net force `F` acting on the body equals its mass `m` times the linear acceleration `a` of its center of mass (CoM).

**Rotational motion:**
```
τ = Iα + ω × (Iω)
```
The net torque `τ` equals the inertia tensor `I` times the angular acceleration `α`, plus the gyroscopic term `ω × (Iω)`. In 2D, the rotation axis stays fixed so the gyroscopic term vanishes and the equation reduces to `τ = Iα`. In 3D, however, a spinning body's angular momentum changes direction whenever the body rotates about another axis—omitting `ω × (Iω)` produces unphysical rotational behavior in simulation.

### Recursive Newton-Euler Algorithm (RNEA)

To compute the joint torques required to execute a given motion on an `n`-link serial arm, notice the natural dependency: a joint near the wrist only supports the hand, whereas a joint at the shoulder must accelerate every link downstream. The Recursive Newton-Euler Algorithm (RNEA) exploits this chain structure in two linear sweeps:

- **Forward pass (base → end-effector):** Starting from the fixed base, propagate velocities and accelerations outward link by link. Each link's velocity is its parent's velocity plus the motion added by its own joint.
- **Backward pass (end-effector → base):** Starting from the tip, propagate forces and torques inward. Using the Newton-Euler balance on each link, find the net wrench needed to produce its acceleration, project it onto the joint axis to read off the motor torque, and pass the reaction wrench back to the parent link.

Why favor this recursive structure? Evaluating the dynamics of a single rigid body takes O(1) work, so marching through `n` links sequentially costs O(n) time overall. By contrast, expanding the Euler-Lagrange equations to build the dense inertia matrix `M(q)` directly scales as O(n^3) or worse—a difference that determines whether a multi-DOF controller can hit a kilohertz real-time loop.

Putting both sweeps together yields the RNEA pseudocode:

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

Here `S[i]` is the motion subspace matrix of joint `i` (specifying the joint axis direction), `v[i]` is the spatial velocity of link `i`, and `I[i]` is its spatial inertia, following Featherstone's spatial vector convention (covered in detail in §5.7).

### Real Code: Pinocchio

Pinocchio is a C++/Python library that implements RNEA and related rigid-body algorithms. In the snippet below, we load a URDF model, sample a joint state `(q, v, a)`, compute full inverse dynamics torques, and then isolate gravity compensation torques by setting velocities and accelerations to zero:

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

Why does zeroing `v` and `a` yield gravity torques? Internally, RNEA initializes the base acceleration to `a[0] = -g` (an detail omitted from the simplified pseudocode above): accelerating the base upward at `g` produces the exact inertial forces of a downward gravitational field. Commanding `tau_g` alone prevents a manipulator from sagging under its own weight, making gravity compensation one of the first building blocks implemented on real hardware.

If your workflow uses Drake rather than Pinocchio, the equivalent inverse-dynamics call lives on `MultibodyPlant`:

```python
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.parsing import Parser
from pydrake.multibody.tree import MultibodyForces
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

Tracking reaction forces and moments across every joint quickly becomes tedious when deriving closed-form equations of motion. Lagrangian mechanics sidesteps those internal constraint forces by working with scalar energies instead of vector forces: because energy simply sums across links, you only need the total kinetic and potential energy in whatever coordinates best describe the robot.

The **Lagrangian** `L` is defined as the difference between total kinetic energy `T` and total potential energy `V`:

```
L(q, q̇) = T(q, q̇) - V(q)
```

Applying the **Euler-Lagrange equation** to each generalized coordinate `q_i` yields the system's equations of motion:

```
d/dt (∂L/∂q̇_i) - ∂L/∂q_i = τ_i
```

Unlike the Newton-Euler formulation—which tracks internal forces between adjacent links and requires choosing frames carefully—Lagrangian mechanics lets you pick generalized coordinates freely. For open kinematic chains with holonomic constraints, choosing joint angles as generalized coordinates eliminates joint constraint forces from the equations altogether. (Closed kinematic chains and nonholonomic constraints still require Lagrange multipliers.)

### Manipulator Equation

Expanding the Euler-Lagrange equations for an `n`-DOF serial manipulator and grouping terms by acceleration, velocity, and position gives the canonical manipulator equation:

```
M(q)q̈ + C(q, q̇)q̇ + g(q) = τ
```

Each term has a clear physical role:

- **M(q)**: the `n×n` symmetric, positive-definite mass/inertia matrix. Because `M(q)` depends on configuration `q`, extending an arm increases its effective inertia while folding it in reduces it.
- **C(q, q̇)q̇**: Coriolis and centrifugal forces arising from inertial coupling when multiple joints move simultaneously. Negligible during slow motion, these terms grow quadratically with speed.
- **g(q)**: the gravity torque vector required at each joint to hold the robot stationary against gravity.
- **τ**: the applied joint torque vector produced by the actuators (with joint friction typically modeled separately and added on).

This equation is the backbone of robot dynamics: model-based control, forward simulation, and trajectory optimization all build directly on it.

### 2-Link Planar Arm Example

The 2-link planar arm is the classic testbed for dynamics—working through it once by hand reveals exactly how `M(q)`, `C(q, q̇)`, and `g(q)` arise in the general `n`-DOF case.

Consider a planar arm with the following parameters:
- Link lengths: `l_1`, `l_2`
- Link masses: `m_1`, `m_2` (modeled as point masses concentrated at each link tip)
- Joint angles: `q_1`, `q_2` (measured relative to the horizontal and the previous link, respectively)
- Gravity: `g` (pointing downward)

**Kinetic energy T:**

From forward kinematics, the tip position of link 1 is:
```
x_1 = l_1 cos(q_1)
y_1 = l_1 sin(q_1)
```

And the tip position of link 2 is:
```
x_2 = l_1 cos(q_1) + l_2 cos(q_1 + q_2)
y_2 = l_1 sin(q_1) + l_2 sin(q_1 + q_2)
```

Differentiating each position with respect to time to get `v_1` and `v_2`, then substituting into `T = (1/2)m_1 v_1^2 + (1/2)m_2 v_2^2`, yields:

```
T = (1/2)(m_1 + m_2) l_1^2 q̇_1^2
  + (1/2) m_2 l_2^2 (q̇_1 + q̇_2)^2
  + m_2 l_1 l_2 cos(q_2) q̇_1 (q̇_1 + q̇_2)
```

**Potential energy V:**

Summing the gravitational potential energy `m_i g y_i` of both point masses gives:

```
V = m_1 g l_1 sin(q_1) + m_2 g [l_1 sin(q_1) + l_2 sin(q_1 + q_2)]
```

Applying the Euler-Lagrange equation and collecting coefficients of `q̈`, `q̇`, and `g` separates the dynamics into the standard manipulator matrices.

**M(q) matrix:**

```
M(q) = [ (m_1+m_2)l_1^2 + m_2 l_2^2 + 2 m_2 l_1 l_2 cos(q_2)    m_2 l_2^2 + m_2 l_1 l_2 cos(q_2) ]
        [ m_2 l_2^2 + m_2 l_1 l_2 cos(q_2)                          m_2 l_2^2                          ]
```

Notice how `M(q)` depends on the elbow angle `q_2`. In particular, `M_{11}`—the effective inertia felt by the base joint—peaks at `q_2 = 0` when the arm is fully extended and reaches its minimum at `q_2 = π` when the arm is folded back.

**C(q, q̇) matrix:**

```
C(q, q̇) = [ -m_2 l_1 l_2 sin(q_2) q̇_2    -m_2 l_1 l_2 sin(q_2)(q̇_1 + q̇_2) ]
            [  m_2 l_1 l_2 sin(q_2) q̇_1     0                                     ]
```

In general `n`-DOF derivations, `C(q, q̇)` is systematically constructed via Christoffel symbols of the first kind from `M(q)`, though for a 2-link arm it is faster to read the velocity-product terms directly off the Euler-Lagrange expansion.

**g(q) vector:**

```
g(q) = [ (m_1 + m_2) g l_1 cos(q_1) + m_2 g l_2 cos(q_1 + q_2) ]
        [ m_2 g l_2 cos(q_1 + q_2)                                 ]
```

You can verify the symbolic kinetic and potential energy expressions in Python with SymPy before carrying out the time derivatives:

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

Once SymPy runs `trigsimp`, the expanded trigonometric terms collapse cleanly into the compact `T` and `V` expressions derived above.

> **Further reading**
> - Murray, Li, Sastry, *A Mathematical Introduction to Robotic Manipulation*, Ch. 4 (https://www.cds.caltech.edu/~murray/mlswiki/) — the most rigorous treatment of Lagrangian mechanics in a robotics context. Free PDF available.
> - Spong, Hutchinson, Vidyasagar, *Robot Modeling and Control*, Ch. 6-7 — the most accessible explanation at an undergraduate level
> - Craig, *Introduction to Robotics*, Ch. 6 — contains a detailed 2-link arm example

---

## 5.4 Newton-Euler vs. Lagrangian

Both formulations express the exact same physical laws and arrive at identical equations of motion. Where they diverge is how you derive those equations, what physical quantities they surface, and how fast they run as the number of joints grows:

| Item | Newton-Euler (RNEA) | Lagrangian |
|------|-------------------|---------|
| Perspective | force/torque (force-based) | energy (energy-based) |
| Computational complexity | O(n) | O(n^3) (when computing the M matrix directly) |
| Derivation difficulty | recursive, same pattern as n grows | partial derivatives explode as n grows |
| Physical intuition | forces/torques on each link are directly visible | energy conservation/transformation is visible |
| Primary use | real-time control, simulation | model derivation, energy-based analysis, Lyapunov stability |
| Constraint forces | explicitly computable | automatically eliminated when using generalized coordinates |

In practice, robotics engineers pair the two rather than picking one exclusively:

1. **Model derivation**: Use Lagrangian mechanics to understand the analytical structure of the manipulator equation (`M`, `C`, `g`).
2. **Numerical computation**: Evaluate dynamics in real time using recursive Newton-Euler algorithms (RNEA or ABA).
3. **Controller design**: Formulate computed-torque or passivity-based controllers around the structural properties of `M(q)` and `C(q, q̇)`.
4. **Code implementation**: Call libraries such as Pinocchio or Drake, which run RNEA and ABA under the hood.

Viewed together, the two formulations play complementary roles: Lagrangian mechanics gives you the algebraic structure needed for control theory and stability proofs, while Newton-Euler recursion delivers the speed required inside real-time control loops.

> **Further reading**
> - Featherstone, *Rigid Body Dynamics Algorithms*, Ch. 3 — a clear account of the relationship between the two formulations
> - Siciliano et al., *Robotics: Modelling, Planning and Control*, Ch. 7 — a comparative example deriving the dynamics of the same robot with both methods

---

## 5.5 Forward Dynamics vs. Inverse Dynamics

Every rigid-body dynamics computation runs in one of two directions, depending on whether you are controlling a physical robot or stepping a simulator:

**Inverse Dynamics:**
```
Given: q, q̇, q̈
Find: τ
```
Inverse dynamics asks: *what motor torques must we apply to follow a desired trajectory?* It sits at the core of model-based controllers such as computed torque control.

**Forward Dynamics:**
```
Given: q, q̇, τ
Find: q̈
```
Forward dynamics asks the opposite question: *given the current torques, how will the robot accelerate?* Physics simulators solve this problem at every integration time step.

Algebraically, forward dynamics means isolating `q̈` in the manipulator equation:

```
q̈ = M(q)^{-1} [τ - C(q, q̇)q̇ - g(q)]
```

Forming `M(q)` explicitly and inverting it costs O(n^3) operations, which quickly becomes a bottleneck as joint counts rise toward humanoids or multi-arm systems.

### Articulated Body Algorithm (ABA)

Just as RNEA solves inverse dynamics in O(n) time, Featherstone's Articulated Body Algorithm (ABA) solves forward dynamics in O(n) time without ever building or inverting the full `M(q)` matrix.

The intuition is to look at each link together with its entire downstream subtree as a single composite "articulated body." In three passes—sweeping velocities forward, accumulating effective subtree inertias and bias forces backward from the tips to the base, and finally resolving joint accelerations forward—ABA computes `q̈` in linear time:

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

Because the bookkeeping in Pass 2 and Pass 3 is intricate to implement cleanly by hand, production code relies on battle-tested libraries such as Pinocchio or Drake rather than custom ABA implementations.

### Role in Simulators

Each major physics engine builds its forward-dynamics pipeline around a different strategy for branching chains and contacts:

- **MuJoCo**: uses its own forward-dynamics algorithm featuring an integrated solver that treats contact forces alongside joint dynamics, exploiting sparse factorization tailored to branching kinematic trees.
- **Drake**: `MultibodyPlant` uses ABA for articulated inertia, delegating contact to separate solvers (time-stepping, hydroelastic, and related formulations).
- **Bullet (PyBullet)**: builds on Featherstone's ABA and resolves contact via a sequential impulse solver.

In Pinocchio, you can verify directly that ABA and RNEA are exact mathematical inverses of each other—feeding ABA's output acceleration back into RNEA recovers the original torque vector up to floating-point precision:

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

> **Further reading**
> - Featherstone, *Rigid Body Dynamics Algorithms*, Ch. 7 — the original description of ABA
> - MuJoCo documentation: Computation (https://mujoco.readthedocs.io/en/latest/computation/) — describes MuJoCo's dynamics pipeline
> - Drake MultibodyPlant tutorial (https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_multibody_plant.html) — the dynamics computation API in Drake

---

## 5.6 Contact Dynamics

Free-space robot motion evolves according to smooth ordinary differential equations (ODEs). The moment a link touches a table, wall, or floor, however, unilateral inequality constraints and sudden velocity discontinuities enter the dynamics.

### Rigid Contact vs. Compliant Contact

Contact models fall into two broad families depending on whether they treat bodies as perfectly hard or slightly deformable:

**Rigid contact:**
- Enforces a strict non-penetration constraint so colliding geometries cannot overlap.
- Recovers normal contact forces as Lagrange multipliers associated with that constraint.
- Clean mathematically, but numerically challenging: impact and separation introduce discontinuities that require solving a Linear Complementarity Problem (LCP) or Nonlinear Complementarity Problem (NCP) at each step.
- Time-stepping LCP formulations in the Stewart-Trinkle family belong to this camp.

**Compliant contact:**
- Places a virtual spring-damper at the contact interface, generating a restoring force proportional to penetration depth and indentation velocity.
- Straightforward to implement and numerically well-behaved for optimization and learning.
- Raising spring stiffness approaches rigid behavior, at the cost of a stiff ODE that forces smaller integration time steps.
- MuJoCo's default contact formulation falls into this family.

### Coulomb Friction Model

Contact almost always brings friction along with it. The classical baseline is Coulomb friction, which distinguishes sticking from sliding using a single coefficient `μ`:

```
|f_t| ≤ μ f_n          (static friction)
|f_t| = μ f_n, f_t ∥ -v_t  (sliding friction)
```

Here `f_t` is the tangential friction force, `f_n` is the normal force, `μ` is the friction coefficient, and `v_t` is the tangential relative slip velocity.

Despite its simplicity, Coulomb friction introduces thorny numerical and theoretical issues:
- Switching between static sticking and kinetic sliding creates a sharp discontinuity.
- In 3D, the feasible friction region forms a nonlinear circular cone; approximating it as a faceted pyramid keeps constraints linear at the cost of directional bias.
- Under certain geometric and inertial configurations—known as Painleve's paradox—rigid contact paired with Coulomb friction can have no solution or multiple solutions.

### Why Contact-Rich Manipulation Is Hard

Tasks like grasping, in-hand reorientation, and peg-in-hole assembly remain among the hardest problems in robotics for four compounding reasons:

1. **Hybrid dynamics**: Each combination of contacts (making/breaking contact, sticking/slipping) acts as a distinct dynamical mode, and predicting exactly when the system switches modes is difficult.
2. **Discontinuous dynamics**: Impacts cause near-instantaneous jumps in velocity, breaking smooth gradient-based reasoning.
3. **Sensitivity to parameters**: Small mismatches in friction coefficients, surface compliance, or mass distribution can flip a successful grasp into a slip, widening the sim-to-real gap.
4. **Combinatorial complexity**: With multiple contact points, the number of active stick/slip/separation combinations grows exponentially (with the exact count depending on the friction model and tangential discretization).

### Why Contact Handling Differs Across Simulators

Because exact rigid contact with 3D friction is computationally expensive and non-smooth, each physics simulator adopts a different numerical approximation trading off accuracy, speed, and solver stability:

- **MuJoCo**: combines compliant soft contact with a convex optimization solver. Fast and numerically smooth, it permits small interpenetrations by design—one reason it became a staple for reinforcement learning.
- **Drake**: provides both compliant point contact (via its TAMSI and SAP discrete solvers) and hydroelastic contact, which integrates pressure across overlapping contact patches for higher physical fidelity at greater compute cost.
- **Bullet**: relies on velocity-level LCP with a sequential impulse solver, reflecting its origins in games and real-time graphics; actual accuracy and speed vary strongly with solver settings and task geometry, so benchmarking on the target system is essential.
- **DART**: implements LCP-based rigid contact with strong academic rigor, though with a smaller ecosystem than MuJoCo or Drake.

No single simulator wins across every metric: MuJoCo is a common default for locomotion RL, while both Drake and MuJoCo serve contact-rich manipulation. Choose based on the contact physics, differentiability, throughput, hardware targets, and validation benchmarks your problem demands.

Once a MuJoCo simulation step finishes, you can inspect every active contact—its world position, normal frame, signed penetration distance, and colliding geometry IDs—directly through `MjData`:

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
    print(f"  Signed distance (negative means penetration): {contact.dist}")
    print(f"  Geom pair: ({contact.geom1}, {contact.geom2})")
```

> **Further reading**
> - Stewart, "Rigid-Body Dynamics with Friction and Impact", SIAM Review 2000 — the mathematical foundation of contact dynamics
> - Todorov, "Convex and analytically-invertible dynamics with contacts and constraints", ICRA 2014 — the paper behind MuJoCo's contact model
> - [Todorov et al., "MuJoCo: A Physics Engine for Model-Based Control" (IROS 2012)](https://ieeexplore.ieee.org/document/6386109) — the original paper describing MuJoCo's convex contact formulation and velocity stepping.
> - Drake's contact model documentation (https://drake.mit.edu/doxygen_cxx/group__hydroelastic__user__guide.html) — describes hydroelastic contact
> - Russ Tedrake, *Underactuated Robotics*, Ch. "Contact" (https://underactuated.csail.mit.edu/) — an introduction to contact dynamics

---

## 5.7 Advanced: Featherstone Algorithms and Spatial Algebra

The remaining two sections enter graduate-level territory. Earlier, in §5.2 and §5.5, we wrote the RNEA and ABA loops using compact symbols `v[i]`, `f[i]`, and `I[i]`. Featherstone's spatial vector algebra is the mathematical machinery behind that notation—packing linear and angular quantities into unified 6D vectors so rigid-body algorithms collapse to a handful of clean matrix operations.

### Spatial Vectors (6D Vectors)

A rigid body moving in 3D space has 3 translational and 3 rotational degrees of freedom—6 DOF in total. Rather than carrying separate 3D equations for translation and rotation, spatial algebra bundles them into dual 6D vectors:

**Motion vector (spatial velocity, twist):**
```
v = [ω; v_O]
```
The top three entries store the body's angular velocity `ω`; the bottom three store the linear velocity `v_O` of the body point momentarily passing through reference point `O`.

**Force vector (spatial force, wrench):**
```
f = [n_O; f]
```
The top three entries store the moment `n_O` about reference point `O`; the bottom three store the linear force `f`.

Notice the deliberate ordering: pairing `[ω; v_O]` with `[n_O; f]` makes their inner product equal instantaneous mechanical power:
```
P = f^T v = n_O · ω + f · v_O
```

Spatial vectors live in dual vector spaces specifically so that this coordinate-invariant power pairing holds everywhere.

### Spatial Inertia

To map 6D spatial acceleration to 6D spatial force in a single step, the `6×6` spatial inertia matrix packages mass `m`, center-of-mass offset `c`, and rotational inertia `I_cm` into one block matrix:

```
I_sp = [ I_cm + m·[c]×[c]×^T    m·[c]× ]
       [ m·[c]×^T                 m·1    ]
```

Here `[c]×` denotes the skew-symmetric cross-product matrix of the vector `c` from reference point `O` to the CoM.

Treating inertia as a single `6×6` operator brings two major algebraic payoffs:
- **Simple composition**: Inertias of multiple bodies expressed in the same frame and reference point add directly (`I_composite = I_1 + I_2 + ...`), which is exactly what the Composite Rigid Body Algorithm (CRBA) uses to build `M(q)`.
- **Clean frame transforms**: Shifting a spatial inertia between frames `A` and `B` reduces to a single congruence transform, `I_B = X^T I_A X`.

### Spatial Vector Form of RNEA and ABA

With 6D motion and force vectors defined, the pseudocode in §5.2 and §5.5 becomes exact rather than schematic. The joint motion subspace `S[i]` encodes the joint type (`[e_z; 0]` for a revolute joint along `z`, `[0; e_z]` for a prismatic joint along `z`), while `v[i]` and `f[i]` carry full 6D twists and wrenches.

Because joint geometry lives entirely inside `S[i]`, one unified code path handles revolute, prismatic, spherical, and free-flyer joints alike—the architectural reason Pinocchio and Drake build their cores on spatial algebra.

### Accessing Spatial Quantities in Pinocchio

Pinocchio exposes these spatial velocities, mass matrices, and centroidal momentum maps directly from Python. The script below computes frame twists, builds the full joint-space inertia matrix `M(q)` via CRBA, and extracts the centroidal momentum matrix `A_g`:

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

For real-time control loops running at 1 kHz or faster, Pinocchio provides a mirror Eigen-based C++ API that avoids Python interpreter overhead while keeping nearly identical function signatures:

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

> **Further reading**
> - Featherstone, *Rigid Body Dynamics Algorithms*, Ch. 2 — the original description of spatial vector algebra.
> - Featherstone, "A Beginner's Guide to 6-D Vectors" (IEEE Robotics & Automation Magazine, 2010) — a more accessible introduction than the textbook
> - Pinocchio GitHub (https://github.com/stack-of-tasks/pinocchio) — the source code itself is a good implementation example of spatial algebra

---

## 5.8 Advanced: Floating Base Systems

An industrial manipulator sits bolted to the floor. Legged robots, aerial drones, and underwater vehicles have no fixed anchor—their base moves freely in 3D space, adding six unactuated degrees of freedom that fundamentally reshape the equations of motion.

### Configuration of a Floating Base

For a fixed-base robot, every joint coordinate lives in `R^n`. Once the base is free to translate and rotate, the configuration vector splits into base pose and internal joints:

```
q = [q_base; q_joints]
```

Here `q_base` belongs to `SE(3)`—3 coordinates for translation plus 3 for orientation (or 4 when parameterized by a unit quaternion). Because a quaternion lives on the unit 3-sphere rather than in Euclidean space, Pinocchio's configuration dimension `nq` exceeds its velocity dimension `nv` (`nq = nv + 1` for a single free-flyer base).

Since `q` and `v` no longer occupy the same vector space, naive Euler updates like `q += v*dt` break unit-quaternion normalization. Instead, integration and differencing must respect the manifold structure via `pin.integrate(model, q, v*dt)`.

### Underactuated Systems

Most floating-base robots are **underactuated**—they have fewer independent control inputs than degrees of freedom. A legged robot has no thrusters bolted to its pelvis, so it can only accelerate its torso by pushing against the ground with its feet; similarly, a standard quadrotor must control 6-DOF pose using only 4 rotor thrusts. Underactuation depends on both available actuators and active contacts, not the floating base itself: an omnidirectional multirotor or a spacecraft with 6-axis thrusters is fully actuated, and a legged robot with sufficient multi-limb contacts can likewise control all 6 base DOFs.

Partitioning the manipulator equation into base rows (`b`) and joint rows (`j`) makes this structure explicit:

```
[ M_bb  M_bj ] [ a_base  ]   [ C_b ]   [ g_b ]   [  0  ]   [ J_{c,b}^T ]
[ M_jb  M_jj ] [ q̈_joints] + [ C_j ] + [ g_j ] = [ τ_j ] + [ J_{c,j}^T ] λ
```

The `0` in the top block of the torque vector reflects the hard physical constraint that no motor acts directly on the base. Instead, `λ` (dimension `k`) denotes the external contact forces, mapped into generalized forces through the base and joint blocks `J_{c,b}` (`k×6`) and `J_{c,j}` (`k×n`) of the contact Jacobian `J_c` (`k×(6+n)`). Only contact forces and gravity can accelerate the base.

This is precisely why legged locomotion is much harder to control than fixed-base manipulation: on a bolted arm, you can command joint torques directly toward a target; on a quadruped or humanoid, you must synthesize joint torques that produce physically valid ground contact forces (`λ`) to steer the unactuated base without slipping or tipping.

### Centroidal Dynamics

Looking at all `6+n` DOFs simultaneously can obscure the forest for the trees. Summing linear and angular momentum across every link and expressing the total momentum at the robot's overall center of mass (CoM) yields **centroidal dynamics**—a compact 6D summary of the entire system's momentum:

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

Three properties make centroidal dynamics central to modern locomotion control:

1. **Balance reduces to CoM and momentum regulation.** When stationary, static stability requires the CoM's vertical projection to stay inside the support polygon. During dynamic walking, the Zero-Moment Point (ZMP, or center of pressure over the contact patch) must remain inside that polygon, and more broadly, centroidal linear and angular momentum must be kept bounded.
2. **Sharp dimensionality reduction.** Rather than optimizing over `30+` joint coordinates directly, planners can shape desired motion in the 6D centroidal space (3 linear + 3 angular momentum components) and map the result back to full joint trajectories afterward.
3. **Direct link to contact forces.** Because internal joint torques cancel out pairwise by Newton's third law, the rate of change of centroidal momentum depends *exclusively* on gravity and external contact forces `f_contact`—turning gait generation into a contact-force planning problem.

In Pinocchio, initializing a model with `pin.JointModelFreeFlyer()` lets you evaluate centroidal momentum `h`, the `6 × nv` centroidal momentum matrix `A_g(q)`, and the CoM position and velocity directly:

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
h = data.hg  # 6D centroidal momentum; Pinocchio orders it [linear; angular]
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

Combining contact scheduling, centroidal trajectory optimization, and whole-body control (covered in §6.7) yields the standard hierarchical architecture used across modern legged robots:

```
[Contact Schedule] → [Centroidal Trajectory Optimization] → [Whole-Body Control] → [Joint Torques]

Stage 1: Decide which foot touches the ground and when (gait pattern)
Stage 2: Plan CoM trajectory + contact forces consistent with centroidal dynamics
Stage 3: Compute torques that meet the centroidal target while satisfying joint-level constraints
Stage 4: Command torques to the motors
```

Variants of this same pipeline generalize across floating-base platforms, from aerial vehicle trajectory optimization to underwater vehicle control.

> **Further reading**
> - Orin et al., "Centroidal Dynamics of a Humanoid Robot", Autonomous Robots 2013 — an analysis of centroidal dynamics for humanoids
> - Wensing et al., "Optimization-Based Control for Dynamic Legged Robots", IEEE T-RO 2024 (arXiv 2022) — a survey of locomotion control
> - Russ Tedrake, *Underactuated Robotics*, Ch. "Walking" (https://underactuated.csail.mit.edu/) — the relationship between underactuated systems and walking
> - Carpentier, Mansard, "Pinocchio: fast forward and inverse dynamics for poly-articulated systems" (https://github.com/stack-of-tasks/pinocchio) — Pinocchio's centroidal dynamics implementation

---

## 5.9 Further Reading

The best entry point depends on your mathematical background and goals:

**Undergraduate introduction to dynamics:**

> - Spong, Hutchinson, Vidyasagar, *Robot Modeling and Control* — an accessible undergraduate text with a thorough step-by-step derivation of the manipulator equation.
> - Craig, *Introduction to Robotics: Mechanics and Control* — takes a practical engineering perspective, though lighter on mathematical rigor.

**Graduate-level mathematical foundations and algorithms:**

> - Murray, Li, Sastry, *A Mathematical Introduction to Robotic Manipulation* (https://www.cds.caltech.edu/~murray/mlswiki/) — develops dynamics rigorously via Lie groups and Lie algebras. Free PDF available, though steep as a first text.
> - Featherstone, *Rigid Body Dynamics Algorithms* — the authoritative reference for spatial vector algebra, RNEA, ABA, and the composite rigid body algorithm; essential reading for graduate researchers.

**Unifying dynamics with control and optimization:**

> - Russ Tedrake, *Underactuated Robotics* (https://underactuated.csail.mit.edu/) — connects rigid-body dynamics directly to trajectory optimization and feedback control, with free lecture videos on MIT OCW.

**Core libraries and simulators:**

> - Pinocchio (https://github.com/stack-of-tasks/pinocchio) — fast C++/Python rigid-body dynamics supporting RNEA, ABA, CRBA, centroidal dynamics, analytical derivatives, and autodiff via CasADi and CppAD (Pinocchio 3.x).
> - Drake (https://drake.mit.edu/) — an integrated simulation, optimization, and control framework built around `MultibodyPlant` and a rich mathematical programming interface suited to trajectory optimization.
> - MuJoCo (https://mujoco.org/) — a fast physics engine maintained by DeepMind and widely adopted for contact-rich robot learning.
> - PyBullet (https://pybullet.org/) — the Python interface to Bullet Physics. Easy to install and teach with, though contact fidelity and speed depend heavily on articulated-body tuning, contact parameters, and time step, requiring task-specific benchmarking.

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
2021 ── DeepMind acquires MuJoCo and makes it free (source released May 2022 under Apache 2.0)
2022 ── Drake 1.0 (MIT → Toyota Research Institute)
2022 ── MuJoCo 2.3 released (the implicitfast integrator arrived in 2.3.x in 2023; the elliptic friction cone was available earlier)
2023 ── MuJoCo 3.0: MJX (JAX backend for GPU parallelism)
2024 ── Pinocchio 3.0 stable release: CasADi and CppAD autodiff support
```

---

## Summary

Five takeaways to carry into the next chapter:

1. The manipulator equation `M(q)q̈ + C(q,q̇)q̇ + g(q) = τ` is the standard template for rigid-body dynamics and controller design.
2. RNEA solves inverse dynamics (finding `τ`) and ABA solves forward dynamics (finding `q̈`), both in `O(n)` time.
3. Contact introduces unilateral constraints and velocity discontinuities, making contact modeling central to simulator selection and sim-to-real transfer.
4. For floating-base systems, centroidal dynamics compresses full-body motion into 6D momentum equations driven solely by gravity and contact forces.
5. In practice, rely on established libraries like Pinocchio or Drake while verifying their coordinate conventions, quaternion orderings, and contact assumptions.

With a model linking joint torques and contact forces to accelerations in hand, Ch. 6 turns to the control problem: how to choose those torques continuously in real time—using PID, LQR, MPC, impedance control, and whole-body control—while guaranteeing stability despite disturbances and model error.
