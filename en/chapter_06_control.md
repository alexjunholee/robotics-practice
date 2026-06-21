# Ch.6 — Control Theory


Perceiving the world and actually affecting it are two entirely different problems for a robot. No matter how sophisticated the perception pipeline, a single miscalculated motor current makes the robot tip over, break things, or injure people. Control theory is the discipline of "how to actually realize a desired motion."

---

## 6.1 Why Learn Control

If perception is "understanding the world," then control is "affecting the world." Without both, a robot is just a pile of sensors or a pile of motors.

The reasons to learn control are simple:

- **Motors are dumber than expected.** Command "send the joint to 30 degrees," and the motor slams in maximum current, overshoots 30 degrees, and oscillates. Smoothly bringing it to the desired position is control.
- **Disturbances are always present.** The floor is slippery, the wind blows, the payload mass differs from expectation. The sensor-actuator loop must be closed (feedback) to cope with such uncertainty.
- **Safety is on the line.** When an industrial robot arm works next to a person, the lack of force control can break the person's arm. No exaggeration.

Control theory is broad; robotics practice usually needs the path from PID to state-space control, MPC, impedance control, and whole-body control.

One thing upfront: control theory involves a lot of math. If you are not comfortable with linear algebra and differential equations, review at least matrix operations and the eigenvalue concept before reading this chapter.

---

## 6.2 PID Control

PID (Proportional-Integral-Derivative), proposed by Minorsky in 1922 for a ship steering system, has been the most widely used controller in industry for over 100 years. Every control engineer in the world learns it first and keeps using it until retirement.

### Basic Structure

Define the error e(t) = r(t) - y(t). r(t) is the reference (target) and y(t) is the current output.

```
u(t) = Kp * e(t) + Ki * integral(e(τ)dτ, 0, t) + Kd * de(t)/dt
```

Role of each term:

- **P (Proportional)**: generates the control input in proportion to the current error. A large Kp gives a fast response but causes more overshoot and oscillation. With the P term alone, a steady-state error remains. This is because as the error becomes small near the target, the control input also becomes small.

- **I (Integral)**: proportional to the accumulated error. Its role is to eliminate steady-state error. It is essential when there are constant disturbances like gravity or friction. However, excessive use produces wind-up. The issue is that when the error has been accumulating for a long time and the control input is saturated, the accumulated integral value causes a large overshoot even after reaching the target. In practice, anti-windup logic must be implemented.

- **D (Derivative)**: proportional to the rate of change of the error. If the error is decreasing quickly, it reduces the control input to suppress overshoot. A kind of "brake." The problem is that differentiation is extremely sensitive to noise. On real systems with sensor noise, the D term must be passed through a low-pass filter. For this reason, the field often drops the D term entirely and uses PI control only.

### Python Implementation

```python
class PIDController:
    """Discrete-time PID controller. Includes anti-windup."""

    def __init__(self, kp: float, ki: float, kd: float, dt: float,
                 output_limit: tuple[float, float] = (-float('inf'), float('inf')),
                 d_filter_coeff: float = 0.1):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.output_limit = output_limit
        self.d_filter_coeff = d_filter_coeff  # Low-pass filter coefficient for the D term

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_d_filtered = 0.0

    def compute(self, error: float) -> float:
        # Proportional
        p_term = self.kp * error

        # Integral (trapezoidal integration)
        self.integral += 0.5 * (error + self.prev_error) * self.dt
        i_term = self.ki * self.integral

        # Derivative (with low-pass filter)
        d_raw = (error - self.prev_error) / self.dt
        d_filtered = (self.d_filter_coeff * d_raw
                      + (1.0 - self.d_filter_coeff) * self.prev_d_filtered)
        d_term = self.kd * d_filtered

        # Control output
        output = p_term + i_term + d_term

        # Output saturation + anti-windup (clamping)
        lo, hi = self.output_limit
        if output > hi:
            output = hi
            # Anti-windup: back out the integral on saturation
            self.integral -= 0.5 * (error + self.prev_error) * self.dt
        elif output < lo:
            output = lo
            self.integral -= 0.5 * (error + self.prev_error) * self.dt

        self.prev_error = error
        self.prev_d_filtered = d_filtered
        return output


# Usage example: 1-DOF position control
import numpy as np

dt = 0.001  # 1 kHz control period
pid = PIDController(kp=100.0, ki=10.0, kd=5.0, dt=dt,
                    output_limit=(-50.0, 50.0))

position = 0.0
velocity = 0.0
mass = 1.0
target = 1.0

positions = []
for step in range(5000):
    error = target - position
    force = pid.compute(error)

    # Simple first-order dynamics: F = ma, with damping
    acceleration = (force - 0.5 * velocity) / mass
    velocity += acceleration * dt
    position += velocity * dt
    positions.append(position)
```

### Tuning Methods

**Ziegler-Nichols method**: a classical tuning method. Set Ki = 0 and Kd = 0, raise Kp, and find the critical gain Ku at which the system exhibits sustained oscillation and its oscillation period Tu. Then set the gains according to the following table.

```
PID:  Kp = 0.6 * Ku,  Ki = 2 * Kp / Tu,  Kd = Kp * Tu / 8
PI:   Kp = 0.45 * Ku,  Ki = 1.2 * Kp / Tu
P:    Kp = 0.5 * Ku
```

Frankly, Ziegler-Nichols tuning produces fairly large overshoot. Fine as a starting point, but manual fine-tuning is essential afterward.

**Empirical tuning in practice**: most people do it like this.

1. Set D and I to 0.
2. Raise P. Stop at the point where the system reacts quickly but does not oscillate.
3. If a steady-state error remains, raise I little by little. Watch out for wind-up.
4. If overshoot is large, add a little D. Check the noise filter.

A professor would not like to hear that this is done "by feel," but in the field it is done this way most of the time. If the system model is accurate, tuning first in simulation and then applying on the real hardware is far more efficient.

### Limits of PID

PID is powerful but has clear limits:

- **It is SISO (Single-Input Single-Output) only.** In systems with inter-joint coupling like a 6-axis robot arm, applying independent PID to each joint degrades performance. The motion of one joint acts as a disturbance on the others.
- **It is weak on nonlinear systems.** PID is fundamentally a linear controller. Robot dynamics are nonlinear. It only performs well near an operating point.
- **It cannot handle constraints.** Within the PID structure there is no way to explicitly handle physical constraints such as torque limits, joint angle limits, or velocity limits.
- **It does not predict the future.** It reacts only to the current error. Without feedforward, tracking performance is limited.

The reason PID is still in use after 100 years is simple: easy to implement, easy to understand, and works "reasonably well" on most systems. If the plant is simple and the performance requirements are not extreme, PID is enough. Per-joint servo control on industrial robots is still mostly PID-based today.

---

## 6.3 State-Space Representation

PID sees only the input-output relationship. It does not know what is happening "inside" the system. The state-space representation explicitly describes the internal state of the system.

### Basic Form

Continuous-time linear system:

```
x_dot(t) = A * x(t) + B * u(t)    (state equation)
y(t)     = C * x(t) + D * u(t)    (output equation)
```

- x(t): state vector (n x 1). The minimal set of variables needed to fully describe the system.
- u(t): input vector (m x 1). The control input.
- y(t): output vector (p x 1). The measurable outputs.
- A: system matrix (n x n). Determines the intrinsic dynamics of the system.
- B: input matrix (n x m). The influence of the input on the state.
- C: output matrix (p x n). The mapping from state to output.
- D: direct transmission matrix (p x m). Zero in most physical systems.

For example, for the mass-spring-damper system (m * x_ddot + c * x_dot + k * x = F), taking the state as x1 = position and x2 = velocity gives:

```
A = [[0, 1], [-k/m, -c/m]]
B = [[0], [1/m]]
C = [[1, 0]]   (position measured only)
D = [[0]]
```

### Relationship to Transfer Functions

The transfer function is G(s) = C * (sI - A)^(-1) * B + D. Transfer functions are convenient for SISO systems, but for MIMO (Multi-Input Multi-Output) systems state-space is much more natural. Robots are almost always MIMO systems, so state-space is the standard representation.

### Controllability

A system is controllable if, from any initial state, it can reach any final state in finite time. The controllability matrix:

```
C_ctrl = [B, A*B, A^2*B, ..., A^(n-1)*B]
```

If this matrix has rank n, the system is controllable. If the rank is less than n, there exist states unreachable by the control input. LQR should not be applied to such a system.

### Observability

A system is observable if the initial state x(0) can be uniquely determined by observing the output y(t). The observability matrix:

```
O = [C; C*A; C*A^2; ...; C*A^(n-1)]
```

If the rank is n, the system is observable. If it is not observable, state estimation (observer, Kalman filter) will not work properly.

### Why Move from PID to State-Space

Controlling each joint independently with PID ignores the dynamic coupling between joints. Even in a 2-DOF robot arm, when one joint moves quickly, centrifugal and Coriolis forces act on the other. Treating this as a disturbance makes the I term in PID work hard to compensate, but the response is slow and performance is poor.

In state-space, the entire system is described by a single model, and the control input is computed by considering all state variables simultaneously. This becomes the foundation of LQR and MPC in the next sections.

```python
import numpy as np
from scipy import signal
import control  # pip install control

# Inverted pendulum state-space model
# State: [x, x_dot, theta, theta_dot]
# x: cart position, theta: pendulum angle (from vertical)
M = 1.0    # Cart mass (kg)
m = 0.1    # Pendulum mass (kg)
l = 0.5    # Pendulum length (m)
g = 9.81   # Gravity (m/s^2)

# Linearized state-space matrices (around theta ≈ 0)
A = np.array([
    [0, 1, 0, 0],
    [0, 0, -m * g / M, 0],
    [0, 0, 0, 1],
    [0, 0, (M + m) * g / (M * l), 0]
])
B = np.array([[0], [1 / M], [0], [-1 / (M * l)]])
C = np.array([[1, 0, 0, 0],
              [0, 0, 1, 0]])  # Measure cart position and pendulum angle
D = np.zeros((2, 1))

# Check controllability
ctrb_matrix = control.ctrb(A, B)
print(f"Controllability matrix rank: {np.linalg.matrix_rank(ctrb_matrix)}")  # 4 = controllable

# Check observability
obsv_matrix = control.obsv(A, C)
print(f"Observability matrix rank: {np.linalg.matrix_rank(obsv_matrix)}")  # 4 = observable

# System poles (eigenvalues of A)
eigenvalues = np.linalg.eigvals(A)
print(f"System poles: {eigenvalues}")
# If there is a pole with positive real part → unstable system (the inverted pendulum is such a case)
```

---

## 6.4 LQR (Linear-Quadratic Regulator)

If PID relies on "experience and tuning," LQR is a controller based on "optimization." The control input that minimizes a given cost function is obtained analytically.

### Cost Function

```
J = integral_0^inf (x(t)^T * Q * x(t) + u(t)^T * R * u(t)) dt
```

- Q (n x n, positive semi-definite): penalty on state error. "How much is the state departing from zero disliked."
- R (m x m, positive definite): penalty on the control input. "How much is control energy to be saved."

Increasing Q makes the state converge to zero quickly, but the control input grows. Increasing R makes the control input smaller, but state convergence slows down. This is the essential trade-off of LQR.

### Tuning Q and R

Practical method: make Q and R diagonal matrices, and set each diagonal entry to the inverse square of the allowable range of the corresponding state or input.

```
Q_ii = 1 / (maximum allowable value of x_i)^2
R_jj = 1 / (maximum allowable value of u_j)^2
```

Example: cart position within 0.5 m, pendulum angle within 0.1 rad, force within 20 N:

```
Q = diag(1/0.5^2, 0, 1/0.1^2, 0) = diag(4, 0, 100, 0)
R = [1/20^2] = [0.0025]
```

This is only a starting point. Adjust afterward by running simulations.

### Algebraic Riccati Equation (ARE)

The optimal LQR gain K is obtained from the solution P of the following Algebraic Riccati Equation:

```
A^T * P + P * A - P * B * R^(-1) * B^T * P + Q = 0
```

Optimal state feedback gain: K = R^(-1) * B^T * P

Control law: u(t) = -K * x(t)

The key point of this result is that all eigenvalues of the closed-loop system (A - BK) are guaranteed to lie in the left half-plane. That is, stability is mathematically proven.

### Python Implementation

```python
import numpy as np
from scipy.linalg import solve_continuous_are

# Use the inverted pendulum model from the previous section
M, m, l, g = 1.0, 0.1, 0.5, 9.81

A = np.array([
    [0, 1, 0, 0],
    [0, 0, -m * g / M, 0],
    [0, 0, 0, 1],
    [0, 0, (M + m) * g / (M * l), 0]
])
B = np.array([[0], [1 / M], [0], [-1 / (M * l)]])

# Cost function weights
Q = np.diag([4.0, 0.0, 100.0, 0.0])  # Position, velocity, angle, angular velocity
R = np.array([[0.0025]])

# Solve the ARE
P = solve_continuous_are(A, B, Q, R)

# Compute the optimal gain
K = np.linalg.inv(R) @ B.T @ P
print(f"LQR gain K: {K}")

# Check closed-loop poles
A_cl = A - B @ K
eigenvalues_cl = np.linalg.eigvals(A_cl)
print(f"Closed-loop poles: {eigenvalues_cl}")
# All real parts are negative → stable


def simulate_lqr(A, B, K, x0, dt=0.001, t_final=5.0):
    """Closed-loop LQR simulation (Euler integration)."""
    n_steps = int(t_final / dt)
    n = A.shape[0]
    x_history = np.zeros((n_steps, n))
    u_history = np.zeros((n_steps, 1))

    x = x0.copy()
    for i in range(n_steps):
        u = -K @ x
        x_history[i] = x.flatten()
        u_history[i] = u.flatten()
        x_dot = A @ x + B @ u
        x = x + x_dot * dt

    return x_history, u_history


# Initial condition: pendulum tilted 10 degrees
x0 = np.array([[0.0], [0.0], [np.radians(10)], [0.0]])
x_hist, u_hist = simulate_lqr(A, B, K, x0)

# Success if x_hist[:, 2] converges to 0
print(f"Final pendulum angle: {np.degrees(x_hist[-1, 2]):.4f} deg")
```

### Limits of LQR

- **A linear model is required.** Nonlinear systems have to be linearized around an operating point. Performance drops sharply away from the operating point.
- **Constraints cannot be handled explicitly.** Physical constraints such as torque limits or velocity limits cannot be encoded in the cost function. Once the control input saturates, optimality breaks down.
- **The full state is required.** Since u = -Kx, every state variable must be either measured or estimated (by an observer).
- **Not directly applicable to tracking.** Basic LQR is a regulator; it only solves the problem of driving the state to zero. Extensions are needed to track a time-varying target.

MPC emerges to overcome these limits.

---

## 6.5 MPC (Model Predictive Control)

MPC (Model Predictive Control) is a method that solves a finite-horizon optimization problem at every control cycle to compute the control input. It is one of the most widely used control techniques in robotics in the 2020s.

### Basic Concept

At every time step k, perform the following:

1. Measure or estimate the current state x(k).
2. Predict N steps ahead using the model.
3. Find the input sequence {u(k), u(k+1), ..., u(k+N-1)} that minimizes the cost function. Constraints are explicitly included at this step.
4. Apply only the first input u(k); discard the rest.
5. At the next time step, return to step 1.

This is the "receding horizon" strategy. Since the optimization is re-solved each time, feedback effects against model error and disturbances arise naturally.

### Why MPC Dominates in Robotics

- **Constraint handling**: torque limits, joint angle limits, velocity limits, collision avoidance — all go directly into the optimization as constraints. Impossible with PID or LQR.
- **Nonlinear models**: Nonlinear MPC uses the nonlinear dynamics model as-is.
- **Future prediction**: rather than reacting to the current error, MPC predicts the future trajectory and responds proactively. A legged robot shifting its center of mass before taking the next step is based on this principle.
- **Multi-objective optimization**: multiple objectives fit into the cost function simultaneously. "Track the target trajectory while saving energy and respecting torque limits."

### Linear MPC vs Nonlinear MPC

**Linear MPC**: uses a linear model (x(k+1) = A*x(k) + B*u(k)). When the cost function is quadratic and the constraints are linear, the problem becomes a QP (Quadratic Program). QP is convex, so the global optimum is found quickly. Suitable for real-time control.

**Nonlinear MPC (NMPC)**: uses a nonlinear dynamics model. The problem becomes non-convex, making it hard to solve with no guarantee of the global optimum. However, it reflects robot dynamics accurately and performs well. CasADi + IPOPT is the standard toolkit.

Choice in practice: if the system is sufficiently close to linear or the control period is very short, Linear MPC; if the nonlinearity is large and there is slack in the control period, NMPC.

### Real-Time Issues

The greatest obstacle of MPC is that optimization must be solved every control cycle. A legged robot controlled at 1 kHz must solve a QP within 1 ms.

Major QP solvers:
- **OSQP** (https://osqp.org/): operator splitting based, strong on sparse QPs. First choice for most Linear MPC setups.
- **qpOASES**: active-set based, supports warm-starting, efficient for sequences of QPs.
- **ECOS/Clarabel**: handles up to second-order cone programming.

For NMPC:
- **CasADi** + **IPOPT**: automatic differentiation + interior-point method. The de facto standard for NMPC implementation.
- **acados** (https://docs.acados.org/): CasADi-based but optimized for real time. Generates C code.

Solver speed determines the control rate. A solver taking 5 ms caps throughput at 200 Hz. This is why MPC engineers care so much about the solver.

### Linear MPC Python Example

```python
import numpy as np
from scipy import sparse
import osqp

def linear_mpc(A, B, Q, R, Q_f, x0, N, x_min, x_max, u_min, u_max):
    """
    Linear MPC: convert to QP and solve with OSQP.

    A, B: discrete-time system matrices
    Q: state cost (stage)
    R: input cost
    Q_f: terminal cost
    x0: current state
    N: prediction horizon
    x_min, x_max: state constraints
    u_min, u_max: input constraints
    """
    n = A.shape[0]  # State dimension
    m = B.shape[1]  # Input dimension

    # Decision variable: z = [x(0), x(1), ..., x(N), u(0), ..., u(N-1)]
    n_var = (N + 1) * n + N * m

    # --- Cost function matrices (P, q) ---
    # min 0.5 * z^T P z + q^T z
    P_blocks = [sparse.kron(sparse.eye(N), Q)]      # x(0) ~ x(N-1)
    P_blocks.append(Q_f)                              # x(N) terminal cost
    P_blocks.append(sparse.kron(sparse.eye(N), R))   # u(0) ~ u(N-1)
    P = sparse.block_diag(P_blocks, format='csc')
    q = np.zeros(n_var)

    # --- Equality constraints: dynamics ---
    # x(k+1) = A*x(k) + B*u(k)
    # → A*x(k) + B*u(k) - x(k+1) = 0
    Ax_eq = sparse.kron(sparse.eye(N + 1), -sparse.eye(n))
    Au_shift = sparse.kron(sparse.eye(N, N + 1, 1), sparse.eye(n))
    # Fix: add A in the lower-left block
    for i in range(N):
        row_start = (i + 1) * n
        col_start = i * n
        Ax_eq[row_start:row_start + n, col_start:col_start + n] = A

    Bu_eq = sparse.lil_matrix(((N + 1) * n, N * m))
    for i in range(N):
        Bu_eq[(i + 1) * n:(i + 2) * n, i * m:(i + 1) * m] = B
    Bu_eq = sparse.csc_matrix(Bu_eq)

    A_eq = sparse.hstack([Ax_eq, Bu_eq], format='csc')
    l_eq = np.zeros((N + 1) * n)
    l_eq[:n] = -x0.flatten()  # Initial condition
    u_eq = l_eq.copy()

    # --- Inequality constraints: state and input bounds ---
    A_ineq = sparse.eye(n_var, format='csc')
    l_ineq = np.concatenate([
        np.tile(x_min, N + 1),
        np.tile(u_min, N)
    ])
    u_ineq = np.concatenate([
        np.tile(x_max, N + 1),
        np.tile(u_max, N)
    ])

    # --- Combine all constraints ---
    A_total = sparse.vstack([A_eq, A_ineq], format='csc')
    l_total = np.concatenate([l_eq, l_ineq])
    u_total = np.concatenate([u_eq, u_ineq])

    # --- Solve with OSQP ---
    solver = osqp.OSQP()
    solver.setup(P, q, A_total, l_total, u_total,
                 warm_starting=True, verbose=False,
                 eps_abs=1e-6, eps_rel=1e-6)
    result = solver.solve()

    if result.info.status != 'solved':
        print(f"MPC solve failed: {result.info.status}")
        return None, None

    # Return only the first input
    u_opt = result.x[(N + 1) * n:(N + 1) * n + m]
    x_pred = result.x[:(N + 1) * n].reshape(N + 1, n)
    return u_opt, x_pred


# Usage example: 2D double integrator
dt = 0.1
A_d = np.array([[1, dt], [0, 1]])   # Discrete time
B_d = np.array([[0.5 * dt**2], [dt]])
n, m_ctrl = 2, 1

Q_mpc = sparse.diags([10.0, 1.0])
R_mpc = sparse.diags([0.1])
Q_f_mpc = sparse.diags([100.0, 10.0])  # Make terminal cost large

x0 = np.array([5.0, 0.0])  # Initial position 5 m, velocity 0
N_horizon = 20

x_min_val = np.array([-10.0, -5.0])
x_max_val = np.array([10.0, 5.0])
u_min_val = np.array([-1.0])   # Force limit
u_max_val = np.array([1.0])

u_opt, x_pred = linear_mpc(
    A_d, B_d,
    Q_mpc, R_mpc, Q_f_mpc,
    x0, N_horizon,
    x_min_val, x_max_val,
    u_min_val, u_max_val
)
print(f"Optimal control input: {u_opt}")
print(f"Predicted trajectory (position): {x_pred[:5, 0]}")
```

### Industry Cases

- **Boston Dynamics Atlas (2019~)**: a combination of MPC + Whole-Body Control. Nonlinear MPC predicts contact sequences, and WBC distributes joint torques in real time.
- **Unitree H1/G1 (2023~)**: a hybrid structure where a learning-based policy (reinforcement learning) generates high-level commands and MPC handles low-level trajectory tracking.
- **Figure 01 (2024)**: an LLM does task-level planning and MPC optimizes manipulation trajectories. An example of combining control with AI.

---

## 6.6 Impedance/Admittance Control

The control techniques covered so far focus mostly on "sending the position to a desired place." But the moment the robot physically contacts the environment, position control alone is not enough.

### Position Control vs Force Control vs Impedance Control

- **Position Control**: tracks a target position. Suitable when there is no environment or when the environment is highly rigid. But when a robot arm tries to pick up a cup from a table and the table height differs by even 1 mm — the position controller does not know and tries to push in, so excessive force is generated.

- **Force Control**: tracks a target force. Needed in contact tasks such as grinding and assembly. But pure force control is unstable when not in contact. It is also sensitive to force sensor noise.

- **Impedance Control**: controls the relationship between position and force. Makes the robot behave like a virtual spring-damper system. On contact with the environment, force arises naturally; in non-contact it behaves like position control.

### Virtual Spring-Damper Model

Core idea of impedance control:

```
F = M_d * (x_ddot_d - x_ddot) + D_d * (x_dot_d - x_dot) + K_d * (x_d - x)
```

Or a simplified version ignoring the inertia term:

```
F = K_d * (x_d - x) + D_d * (x_dot_d - x_dot)
```

- K_d: virtual stiffness. Large values give accurate position tracking but large forces on contact.
- D_d: virtual damping. Suppresses oscillation.
- M_d: virtual inertia. Usually hard to tune, so the inertia term is often omitted.

The key is tuning K_d and D_d for the task:
- Picking up a glass: low K_d (gentle), high D_d (stable).
- Tightening a bolt: high K_d (precise).
- Collaborating with a person: very low K_d (safe).

```python
import numpy as np

class ImpedanceController:
    """Cartesian-space impedance controller (1-DOF simplified)."""

    def __init__(self, k_d: float, d_d: float, m_d: float = 0.0):
        self.k_d = k_d   # Virtual stiffness (N/m)
        self.d_d = d_d   # Virtual damping (N*s/m)
        self.m_d = m_d   # Virtual inertia (kg)

    def compute_force(self, x_d, x, x_dot_d, x_dot,
                      x_ddot_d=0.0, x_ddot=0.0) -> float:
        """Compute force according to the target impedance relation."""
        f = (self.k_d * (x_d - x)
             + self.d_d * (x_dot_d - x_dot)
             + self.m_d * (x_ddot_d - x_ddot))
        return f


# Simulation: robot approaches a wall and makes contact
dt = 0.001
controller = ImpedanceController(k_d=500.0, d_d=50.0)

# Robot + environment
robot_mass = 2.0
position = 0.0
velocity = 0.0
target_position = 0.15  # Target position
wall_position = 0.10    # Wall position (closer than the target)
wall_stiffness = 10000.0  # Wall stiffness

positions = []
forces = []
contact_forces = []

for step in range(10000):
    # Environment contact force
    if position > wall_position:
        f_env = -wall_stiffness * (position - wall_position)
    else:
        f_env = 0.0

    # Impedance control output
    f_ctrl = controller.compute_force(
        x_d=target_position, x=position,
        x_dot_d=0.0, x_dot=velocity
    )

    # Dynamics
    acceleration = (f_ctrl + f_env) / robot_mass
    velocity += acceleration * dt
    position += velocity * dt

    positions.append(position)
    forces.append(f_ctrl)
    contact_forces.append(-f_env)

# Result: position stabilizes near wall_position
# Without crushing the wall, pushing with an appropriate contact force
print(f"Final position: {positions[-1]:.4f} m (wall: {wall_position} m)")
print(f"Final contact force: {contact_forces[-1]:.2f} N")
# Pure position control would have hit the wall with 10000 N/m * 0.05 m = 500 N
```

### Admittance Control

If impedance control is "position deviation → force output," admittance control is the opposite: "force input → position output."

```
x_d_new = x_d + (1 / K_d) * F_ext + (1 / D_d) * F_ext_dot
```

More precisely, the measured external force F_ext is fed into a virtual impedance model to modify the target position, and the modified target is passed to the existing (high-stiffness) position controller.

Why admittance control is widely used on industrial robots: industrial robots already have very precise position controllers built in, and torque cannot usually be commanded directly from outside. So measuring the external force with a force sensor (F/T sensor) and modifying the position command — the admittance approach — is more practical.

On research robots with torque control (such as Franka Emika Panda), impedance control is more natural.

---

## 6.7 Advanced: Whole-Body Control

*If you want to become a researcher, read from here.*

Humanoid and quadruped robots have dozens of joints, must manage multiple contact points (feet, hands) simultaneously, and must maintain balance. In such systems, "put a PID on each joint" is practically meaningless. Integrated control at the whole-body level is required.

### Task-Space vs Joint-Space

- **Joint-space control**: controls the joint angles q directly. Simple, but to achieve task-level goals (end-effector position, center-of-mass position) inverse kinematics (IK) must be solved first.

- **Task-space control**: controls directly in task coordinates (Cartesian position, orientation). Task goals are described naturally. Mapping to joint space is handled inside the controller.

### Operational Space Control (Khatib, 1987)

Khatib's Operational Space Framework is the foundation of task-space control. Core idea: derive the dynamics directly in the task space.

Joint-space dynamics:

```
M(q) * q_ddot + C(q, q_dot) * q_dot + g(q) = tau + J^T * F_ext
```

Conversion to task space:

```
Lambda(q) * x_ddot + mu(q, q_dot) * x_dot + p(q) = F + F_ext
```

Here Lambda = (J * M^(-1) * J^T)^(-1) is the task-space inertia matrix.

Joint torques to achieve a desired task-space acceleration x_ddot_d:

```
tau = J^T * Lambda * x_ddot_d + C * q_dot + g(q)
```

Combining impedance control on top of this framework realizes desired dynamic behavior (impedance) in task space.

### QP-Based Whole-Body Control

Modern WBC handles multiple tasks simultaneously by solving a QP (Quadratic Program) at every control cycle.

Basic structure:

```
minimize    || J_task * q_ddot - x_ddot_d ||^2  (task tracking)
subject to  M(q)*q_ddot + h(q,q_dot) = S^T*tau + J_c^T*F_c  (dynamics)
            F_c ∈ friction cone                   (contact force constraint)
            tau_min ≤ tau ≤ tau_max               (torque limits)
```

Here:
- J_task: task Jacobian
- J_c: contact Jacobian
- F_c: contact force
- S: selection matrix (removes underactuated DoFs)

**Multi-task priority**: on real robots multiple tasks conflict. For example, "send the right hand to a target position" + "maintain balance" + "respect joint limits." Priorities are assigned:

1. Highest priority: contact constraints (feet must stay on the ground), joint limits.
2. High priority: balance maintenance (CoM control).
3. Medium priority: end-effector position control.
4. Low priority: posture maintenance (null-space).

To implement this as a strict hierarchy, use null-space projection, or solve the QP at each priority level sequentially (hierarchical QP). Alternatively, soft priorities combine them into a single QP with different weights.

### Contact-Consistent Control

On legged robots, contact forces must be physically plausible:

- **Unilateral contact**: a foot cannot pull the ground. F_z >= 0.
- **Friction cone**: the tangential force must be less than the normal force times the friction coefficient. sqrt(F_x^2 + F_y^2) <= mu * F_z.
- **ZMP/CoP constraint**: the center of pressure must lie within the support polygon to avoid tipping over.

Putting all these constraints into the QP yields physically feasible control inputs. The friction cone is originally nonlinear (second-order cone), but approximated as a polyhedron (linearized friction cone) it fits into a QP.

```python
import numpy as np

def linearized_friction_cone(mu, n_edges=8):
    """
    Polyhedral approximation of the friction cone.
    Returns: constraint matrix in the form A_cone * F <= 0.
    F = [fx, fy, fz]^T
    """
    A_rows = []
    for i in range(n_edges):
        theta = 2 * np.pi * i / n_edges
        # mu * fz >= cos(theta)*fx + sin(theta)*fy
        # → cos(theta)*fx + sin(theta)*fy - mu*fz <= 0
        row = [np.cos(theta), np.sin(theta), -mu]
        A_rows.append(row)
    # fz >= 0 → -fz <= 0
    A_rows.append([0, 0, -1])
    return np.array(A_rows)

# Friction coefficient 0.7, octagonal approximation
A_friction = linearized_friction_cone(mu=0.7)
print(f"Friction cone constraint matrix shape: {A_friction.shape}")
# (9, 3) → 9 linear inequalities approximate the 3D friction cone
```

---

## 6.8 Advanced: Lyapunov Stability and Adaptive Control

*If you want to become a researcher, read from here.*

Once a controller has been designed, "does this controller really make the system stable?" must be proven. Working in simulation and mathematically guaranteed stability are entirely different matters. Lyapunov theory is the central tool for this proof.

### Lyapunov Stability

For a nonlinear system x_dot = f(x), let the origin be an equilibrium (f(0) = 0).

Lyapunov's direct method: if a function V(x) satisfies the following, the origin is stable.

1. V(0) = 0
2. V(x) > 0 for all x != 0 (positive definite)
3. V_dot(x) = dV/dx * f(x) <= 0 (non-increasing)

If V_dot(x) < 0, the origin is asymptotically stable — the state converges to the origin over time.

Physical intuition: V(x) is energy. If the energy is always positive and decreases over time, the system converges to the equilibrium that minimizes the energy.

The hard part: finding V(x). There is no general methodology. For mechanical systems, mechanical energy (kinetic + potential) is a natural Lyapunov function candidate. For linear systems, V(x) = x^T * P * x (where P is the solution of the ARE) serves as a Lyapunov function. This is where the LQR stability proof comes from.

### Adaptive Control

Used when the model parameters are not precisely known. For example, the payload mass carried by a robot arm is unknown, or the friction coefficient changes over time.

Basic idea: embed a parameter estimator inside the controller and run control and estimation simultaneously.

Robot dynamics can be written in a form linear in the parameters:

```
M(q)*q_ddot + C(q,q_dot)*q_dot + g(q) = Y(q, q_dot, q_ddot) * theta
```

Here Y is the regressor matrix and theta is the dynamics parameter vector (mass, inertia, friction, etc.).

Adaptive control law:

```
tau = Y * theta_hat - K_d * s
theta_hat_dot = -Gamma * Y^T * s
```

Here s is the sliding variable, theta_hat is the parameter estimate, and Gamma is the adaptation gain matrix.

With a suitable Lyapunov function (V = 0.5*s^T*M*s + 0.5*theta_tilde^T*Gamma^(-1)*theta_tilde), V_dot <= 0 can be shown and the tracking error proven to converge to zero. Note that theta_hat is not guaranteed to converge to the true theta. Only the tracking error converges.

### Robust Control

Used when there is model uncertainty but the bound is known.

- **H-infinity control**: optimizes performance against the worst-case disturbance. Provides a guarantee of the form "whatever disturbance enters, the output error stays below this level." The math is heavy (Riccati inequalities, LMI) and tends to be conservative. Widely applied in industry in the 1990s.

- **Sliding Mode Control**: drives the state onto a sliding surface in finite time, then follows the desired dynamics on the sliding surface. Very robust to model uncertainty. The issue is chattering: high-frequency switching near the sliding surface stresses the actuator. Mitigated with a boundary layer approach or higher-order sliding mode.

### When to Use, When Not to Use

| Situation | Recommended | Not recommended |
|------|------|--------|
| Accurate model, sufficiently linear | LQR, MPC | Adaptive control (overdesign) |
| Large parameter uncertainty | Adaptive control | Relying on PID alone |
| Known uncertainty bound | Robust control (H-inf) | Adaptive control (unnecessary) |
| Safety certification required | Lyapunov-based proofs | "It worked in simulation so OK" |
| Rapid prototyping | PID + feedforward | H-infinity from the start |

Frankly, unless writing a paper, adaptive control or sliding mode is rarely used on real systems. MPC is powerful and intuitive enough. But when "why is this controller stable?" has to be explained, Lyapunov theory is unavoidable. Especially in safety-critical systems (medical robots, autonomous driving), mathematical stability proofs are essential.

---

## 6.9 Further Reading

> **Åström & Murray, "Feedback Systems: An Introduction for Scientists and Engineers"**
> https://fbswiki.org/
> Free PDF available. The most suitable introduction to control theory. Hits the core accurately without going overboard on math. Covers PID, state-space, and frequency response. Undergraduates should start here.

> **Steve Brunton, "Control Bootcamp" (YouTube)**
> https://www.youtube.com/playlist?list=PLMrJAkhIeNNR20Mz-VpzgfQs5zrYi085m
> Explains state-space, controllability, observability, and LQR intuitively. Each video is around 15 minutes, short and dense. Watching before reading a textbook makes understanding much faster.

> **Slotine & Li, "Applied Nonlinear Control"**
> The standard text on nonlinear control, Lyapunov stability, and adaptive control. The book for studying the content of Section 6.8 seriously. Out of print, but PDFs are around (find it yourself).

> **Russ Tedrake, "Underactuated Robotics" (MIT OCW)**
> https://underactuated.csail.mit.edu/
> Free online textbook and lectures. Goes deep on MPC, trajectory optimization, and the connection between control and planning. Also the theoretical background of the Drake library.

> **python-control library**
> https://python-control.readthedocs.io/
> A Python library for analyzing and designing control systems. The Python alternative to MATLAB's Control System Toolbox. Supports Bode plots, root locus, and state-space analysis.

> **CasADi**
> https://web.casadi.org/
> The de facto standard tool for implementing Nonlinear MPC. Supports automatic differentiation and multiple NLP solvers (IPOPT, SNOPT). Offers Python, MATLAB, and C++ interfaces.

> **OSQP (Operator Splitting Quadratic Program)**
> https://osqp.org/
> A QP solver for Linear MPC. Fast, robust, and capable of code generation, allowing deployment on embedded systems. C implementation with bindings for Python, MATLAB, Julia.

> **Key papers**
> - [Hogan, "Impedance Control: An Approach to Manipulation" (ASME JDSMC 1985)](https://doi.org/10.1115/1.3140702) — the original paper on impedance control. Presents a framework that unifies position control and force control.
> - [Khatib, "A Unified Approach for Motion and Force Control of Robot Manipulators: The Operational Space Formulation" (IEEE RA 1987)](https://doi.org/10.1109/JRA.1987.1087068) — the original paper on Operational Space Control. Foundations of task-space dynamics derivation and control.
> - [Khazoom et al., "Tailoring Solution Accuracy for Fast Whole-Body MPC" (RA-L 2024, arXiv:2407.10789)](https://arxiv.org/abs/2407.10789) — a recent approach to real-time whole-body MPC.

---

## Technical Timeline

```
1922 ── PID control concept formalized (Minorsky)
1960 ── State-space theory (Kalman)
1960 ── LQR (Kalman)
1985 ── Impedance Control concept (Hogan)
1987 ── Operational Space Control (Khatib)
1990s ─ Robust control (H-infinity) applied in industry
2004 ── Real-time MPC becomes practical
2019 ── Boston Dynamics Atlas: MPC + WBC
2023 ── Unitree H1/G1: learning-based + MPC hybrid
2024 ── Figure 01: LLM + MPC + manipulation
```

---

Robotics uses only part of control theory. For the mathematical details of each technique, supplement with the further reading. One piece of advice: control theory is hard to understand without simulation. Run the code, change the parameters, and watch the system response. One simulation beats reading a textbook three times.
