# Ch.6 — Control Theory

Ch. 5 established how applied torques and contact forces accelerate a rigid-body system. Control theory turns that relationship around in real time: given sensor estimates of the robot's state and environment, what actuator commands will drive the system along a desired motion safely, accurately, and without going unstable?

---

## 6.1 Why Learn Control

Even with an exact dynamics model, commanding raw open-loop torques fails the moment a real robot meets model mismatch, sensor noise, or external disturbances. Closing a feedback loop between sensors and actuators addresses three practical requirements:

- **Accurate trajectory tracking**: Open-loop drive easily overshoots the target or breaks into oscillation when inertia or friction varies. Feedback continuously compares measured state against the reference and corrects the actuator command.
- **Disturbance rejection**: Slippery floors, wind gusts, and unmodeled payloads shift the physical dynamics away from the nominal model. Real-time feedback compensates for these unpredicted forces on the fly.
- **Safe physical interaction**: Robots operating around fragile objects or human coworkers must regulate contact forces, not just geometric positions, so small calibration errors never turn into crushing impacts.

Rather than touring all of classical and modern control theory, this chapter traces the working progression robotics engineers rely on in practice: from PID and state-space feedback (LQR) to constrained receding-horizon optimization (MPC), compliant interaction (impedance control), whole-body coordination, and formal stability guarantees (Lyapunov, adaptive, and robust control).

The sections ahead assume basic familiarity with matrix algebra, eigenvalues, and ordinary differential equations.

---

## 6.2 PID Control

Proposed by Nicolas Minorsky in 1922 for automated ship steering, Proportional-Integral-Derivative (PID) control remains the workhorse of industrial motion control. Its three intuitive terms and modest tuning requirements also make it the standard starting point in control courses and joint-level servos.

### Basic Structure

Define the tracking error `e(t) = r(t) - y(t)` as the difference between the target reference `r(t)` and the measured output `y(t)`. Intuitively, PID blends three complementary actions—a virtual spring pulling toward the target (`P`), a memory of persistent offset (`I`), and a damper resisting rapid error changes (`D`):

```
u(t) = Kp * e(t) + Ki * integral(e(τ)dτ, 0, t) + Kd * de(t)/dt
```

Each term shapes the closed-loop response differently:

- **P (Proportional)**: scales the command linearly with the current error. Raising `Kp` speeds up the response, but pushing it too high invites overshoot and oscillation. On a plant without a free integrator, proportional control alone leaves a non-zero steady-state error under a step command—because the corrective input shrinks right alongside the error near the target. (Even when the plant contains an integrator so step-tracking error vanishes, constant loads like gravity still produce steady-state sag under pure P control.)

- **I (Integral)**: accumulates past error over time to wipe out persistent steady-state offsets caused by constant disturbances such as gravity or Coulomb friction. The catch is **integrator wind-up**: if the actuator saturates while the error stays large, the integral keeps growing unchecked, causing massive overshoot once the system finally reaches the target. Real implementations always pair integral action with anti-windup clamping or back-calculation.

- **D (Derivative)**: reacts to the rate of change of the error, acting like a mechanical damper or brake that trims the command when the system approaches the target too fast. Because numerical differentiation amplifies high-frequency sensor noise, raw D action can chatter violently on real hardware. In practice, the derivative term is passed through a low-pass filter—or omitted altogether in noisy loops, leaving a PI controller.

### Python Implementation

A production-ready discrete-time PID loop differs from the textbook formula in three crucial ways: it uses trapezoidal integration for the I term, applies an exponential low-pass filter to the noisy D term, and backs out integral accumulation whenever the output hits actuator limits (anti-windup clamping). The class below demonstrates this on a simple 1-DOF damped mass:

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

    # Simple second-order position dynamics: F = ma, with damping
    acceleration = (force - 0.5 * velocity) / mass
    velocity += acceleration * dt
    position += velocity * dt
    positions.append(position)
```

### Tuning Methods

**Ziegler-Nichols method**: a classic heuristic for choosing baseline gains. Start with `Ki = 0` and `Kd = 0`, raise `Kp` until the system enters sustained marginal oscillation, and record both that critical gain `Ku` and the oscillation period `Tu`. Then compute the initial gains:

```
PID:  Kp = 0.6 * Ku,  Ki = 2 * Kp / Tu,  Kd = Kp * Tu / 8
PI:   Kp = 0.45 * Ku,  Ki = 1.2 * Kp / Tu
P:    Kp = 0.5 * Ku
```

Ziegler-Nichols rules tend to yield aggressive responses with noticeable overshoot, so treat them as a starting point to refine against measured step responses.

**Empirical tuning in practice**: when tuning directly on a plant (or preferably first in simulation), adjust one gain at a time:

1. Zero out `Ki` and `Kd`.
2. Increase `Kp` until the response is crisp without sustained ringing.
3. If steady-state error persists, nudge `Ki` upward gradually while verifying anti-windup behavior.
4. If overshoot or sluggish settling appears, add `Kd` carefully and check the derivative low-pass filter cutoff.

Whenever a reliable dynamics model is available, dial in initial gains in simulation before running on physical actuators.

### Limits of PID

Despite its ubiquity, PID hits structural limits on complex robots:

- **Single-input, single-output (SISO) design**: Running independent PID loops on each joint of a 6-axis arm ignores dynamic coupling—accelerating the shoulder injects centrifugal and Coriolis disturbances directly into the elbow.
- **Linearity around an operating point**: Because PID applies fixed linear gains to inherently nonlinear rigid-body dynamics, gains tuned for one pose can feel sluggish or oscillatory in another.
- **No native constraint handling**: Torque limits, joint stops, and velocity caps sit outside the PID law, handled only after the fact through ad-hoc clamping.
- **Purely reactive**: PID responds only to present, past, and instantaneous-rate errors; without model-based feedforward, fast trajectory tracking inevitably lags behind.

When inter-joint coupling is mild—such as geared industrial servos running at moderate speeds—independent joint PID remains simple, fast, and effective. To coordinate coupled multi-DOF dynamics systematically, however, we move from scalar error loops to state-space models.

---

## 6.3 State-Space Representation

Classical PID treats the plant as a single input-output channel, hiding internal dynamics. The state-space representation opens up that black box, packing every position, velocity, and internal variable into a state vector `x(t)` so multi-input, multi-output (MIMO) coupling can be modeled and controlled directly.

### Basic Form

A continuous-time linear time-invariant (LTI) system takes the first-order matrix form:

```
x_dot(t) = A * x(t) + B * u(t)    (state equation)
y(t)     = C * x(t) + D * u(t)    (output equation)
```

- `x(t)`: state vector (`n × 1`) — the minimal set of variables whose current values uniquely determine future evolution given future inputs.
- `u(t)`: input vector (`m × 1`) — the actuator commands.
- `y(t)`: output vector (`p × 1`) — the measured sensor signals.
- `A`: system matrix (`n × n`) — governs the plant's natural, unforced dynamics.
- `B`: input matrix (`n × m`) — maps actuator inputs into state rates of change.
- `C`: output matrix (`p × n`) — maps internal states to sensor measurements.
- `D`: feedthrough matrix (`p × m`) — captures any direct input-to-output coupling (zero in most physical mechanical systems).

Consider a 1-DOF mass-spring-damper (`m * x_ddot + c * x_dot + k * x = F`). Choosing position `x1` and velocity `x2` as the state vector converts the second-order ODE into first-order state-space form:

```
A = [[0, 1], [-k/m, -c/m]]
B = [[0], [1/m]]
C = [[1, 0]]   (position measured only)
D = [[0]]
```

### Relationship to Transfer Functions

Taking the Laplace transform links state-space matrices directly to the transfer function matrix `G(s) = C * (sI - A)^(-1) * B + D`. Transfer functions shine in SISO frequency-domain design, whereas state-space equations expose internal coupling naturally for multi-joint robots with many sensors and actuators. Which representation you choose depends on the system's structure and the analysis tools you need.

### Controllability

Before designing a controller, ask a fundamental question: *do our actuators actually have enough authority to steer every state variable?* A system is **controllable** if an input signal exists to drive it from any initial state to any target state in finite time. We test this by checking the rank of the controllability matrix:

```
C_ctrl = [B, A*B, A^2*B, ..., A^(n-1)*B]
```

If `rank(C_ctrl) = n`, every mode of the system can be steered by `u`. If `rank(C_ctrl) < n`, some subspace of the state cannot be reached by any control input. (In infinite-horizon LQR design, full controllability can actually be relaxed to **stabilizability**—meaning any uncontrollable modes must already decay on their own—paired with detectability through the state cost and a positive-definite input cost.)

### Observability

The dual question applies to sensing: *can we reconstruct the full internal state from our measurements alone?* A system is **observable** if the initial state `x(0)` can be uniquely inferred from future outputs `y(t)`. We test this with the observability matrix:

```
O = [C; C*A; C*A^2; ...; C*A^(n-1)]
```

Full rank (`rank(O) = n`) guarantees that a state observer or Kalman filter can reconstruct hidden states such as unmeasured velocities from position readings.

### Why Move from PID to State-Space

Driving each joint with an isolated PID loop ignores physical cross-coupling: even on a 2-DOF arm, swinging one link rapidly induces centrifugal and Coriolis torques on the other. An independent PID loop perceives that coupling purely as an unknown disturbance for its integral term to chase after the fact.

State-space methods model all joints and velocities inside one coupled system and compute coordinated actuator commands across every state simultaneously—setting the stage for optimal feedback (LQR) and predictive control (MPC) below.

To make these matrices concrete, the script below linearizes a cart-pole (inverted pendulum) around its upright equilibrium `θ ≈ 0`, verifies full controllability and observability, and inspects the open-loop eigenvalues showing its natural instability:

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

Where PID relies on manual gain tuning, the Linear-Quadratic Regulator (LQR) frames state feedback as an optimization problem: specify what matters—state accuracy versus actuator effort—and solve analytically for the globally optimal feedback matrix `K`.

### Cost Function

LQR seeks the control sequence `u(t)` that minimizes an infinite-horizon quadratic cost balancing regulation error against control effort:

```
J = integral_0^inf (x(t)^T * Q * x(t) + u(t)^T * R * u(t)) dt
```

- `Q` (`n × n`, positive semi-definite): state-error penalty, encoding how strongly deviations of each state variable from zero are penalized.
- `R` (`m × m`, positive definite): input-effort penalty, encoding how costly it is to expend actuator energy.

Weighting `Q` more heavily drives the state to zero faster at the cost of larger actuator spikes; weighting `R` more heavily saves control effort with gentler, slower convergence.

### Tuning Q and R

A reliable starting heuristic (Bryson's rule) initializes `Q` and `R` as diagonal matrices using the inverse square of each variable's maximum acceptable deviation, normalizing mismatched units such as meters, radians, and Newtons:

```
Q_ii = 1 / (maximum allowable value of x_i)^2
R_jj = 1 / (maximum allowable value of u_j)^2
```

For the cart-pole, bounding cart travel within `0.5 m`, pendulum tilt within `0.1 rad`, and force within `20 N` gives:

```
Q = diag(1/0.5^2, 0, 1/0.1^2, 0) = diag(4, 0, 100, 0)
R = [1/20^2] = [0.0025]
```

Treat Bryson's rule as an initial baseline, then refine the weights against closed-loop simulations.

### Algebraic Riccati Equation (ARE)

Because the dynamics are linear and the cost is quadratic over an infinite horizon, the optimal control law turns out to be constant linear state feedback `u(t) = -K * x(t)`. The gain `K` is computed from the unique positive semi-definite solution `P` of the continuous-time Algebraic Riccati Equation:

```
A^T * P + P * A - P * B * R^(-1) * B^T * P + Q = 0
```

From `P`, the optimal gain is `K = R^(-1) * B^T * P`, yielding `u(t) = -K * x(t)`. Under standard stabilizability and detectability conditions, every eigenvalue of the closed-loop matrix `(A - BK)` is guaranteed to have a strictly negative real part—providing an automatic mathematical proof of stability.

### Python Implementation

Applying `scipy.linalg.solve_continuous_are` to our linearized cart-pole computes `K`, shifts the unstable open-loop pole into the left half-plane, and stabilizes a 10-degree initial tilt back to upright:

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

LQR solves unconstrained linear stabilization cleanly, yet four limitations push modern robots beyond it:

- **Local linearity assumption**: Nonlinear systems must be linearized around an equilibrium or nominal trajectory; far from that point, performance degrades rapidly.
- **No explicit constraints**: Torque ceilings, joint stops, and obstacle boundaries cannot be enforced inside standard LQR—if actuators saturate, both optimality and the stability guarantee break down.
- **Full-state feedback required**: Computing `u = -Kx` assumes every state variable is either directly measured or reconstructed by an observer.
- **Regulation, not native tracking**: Standard LQR drives the state to zero; tracking a moving reference trajectory requires reformulating the error dynamics or adding feedforward (as in time-varying LQR).

Overcoming those constraint and prediction limits is precisely why Model Predictive Control (MPC) became central to modern robotics.

---

## 6.5 MPC (Model Predictive Control)

Model Predictive Control (MPC) bridges planning and feedback by solving a finite-horizon constrained optimization problem at every control tick. Think of driving down a winding road: you look ahead several seconds to plan a smooth path that stays inside the lane boundaries, execute only the steering command for the current instant, and re-plan from scratch as soon as you see where the car actually moved.

### Basic Concept

At each discrete time step `k`, MPC runs a five-step receding-horizon loop:

1. Measure or estimate the current state `x(k)`.
2. Roll the dynamics model forward `N` steps into the future.
3. Solve for the optimal control sequence `{u(k), u(k+1), ..., u(k+N-1)}` minimizing the cost function subject to state and input constraints.
4. Apply only the first command `u(k)` to the actuators and discard the rest of the plan.
5. Advance one step, acquire the new state `x(k+1)`, and repeat from step 1.

Because the entire optimization is re-solved at every cycle using fresh measurements, this **receding horizon** strategy turns open-loop trajectory optimization into closed-loop feedback capable of rejecting disturbances and model error.

### Why MPC Is Useful in Robotics

MPC addresses each of the major blind spots in PID and standard LQR:

- **Native constraint handling**: Torque limits, joint range limits, velocity bounds, friction cones, and obstacle avoidance enter directly as inequality constraints—unlike basic PID or LQR, which require add-ons such as saturation clamping, reference governors, or constrained-LQR extensions.
- **Nonlinear models**: Nonlinear MPC (NMPC) embeds the full nonlinear rigid-body dynamics directly rather than relying on a single fixed linearization.
- **Proactive lookahead**: Instead of reacting after error accumulates, MPC anticipates upcoming transitions—for instance, a legged robot shifting its center of mass *before* lifting a foot for the next step.
- **Multi-objective trade-offs**: Competing priorities such as tracking accuracy, energy efficiency, and actuation smoothness coexist inside a single scalar objective alongside hard physical limits.

### Linear MPC vs Nonlinear MPC

Depending on how the dynamics are modeled, MPC takes one of two forms:

- **Linear MPC**: uses a linear discrete-time model (`x(k+1) = A*x(k) + B*u(k)`). Paired with a quadratic cost and linear constraints, the optimization becomes a convex Quadratic Program (QP)—every feasible problem has a unique global optimum, though actual solve time still depends on horizon length, sparsity, solver implementation, and hardware.
- **Nonlinear MPC (NMPC)**: keeps the nonlinear dynamics directly. The resulting Nonlinear Program (NLP) is generally non-convex without a global-optimality guarantee. Toolchains such as CasADi paired with IPOPT—or specialized real-time solvers like acados, FORCESPRO, and SNOPT—provide automatic differentiation and fast numerical solutions.

In practice, choose Linear MPC when the dynamics are mildly nonlinear around a trajectory or when update rates must be very high; turn to NMPC when strong nonlinearities dominate and the control budget allows slightly slower updates.

### Real-Time Issues

The price of online optimization is compute time: every solve must finish inside the control budget. Running MPC synchronously inside a `1 kHz` loop leaves less than `1 ms` for state acquisition, problem setup, and QP/NLP convergence. Consequently, many robotic stacks run MPC as an outer planner at `50–200 Hz` feeding reference trajectories or contact forces to a faster inner tracking loop.

Popular solvers for real-time deployment include:

For Linear MPC (QPs):
- **OSQP** (https://osqp.org/): operator-splitting ADMM solver tailored to sparse QPs; a common first choice for Linear MPC.
- **qpOASES**: active-set solver with warm-starting, well suited to dense or small-to-medium QP sequences.
- **ECOS/Clarabel**: interior-point solvers supporting second-order cone constraints (useful for exact 3D friction cones).

For Nonlinear MPC (NLPs):
- **CasADi** + **IPOPT**: flexible symbolic automatic differentiation coupled to a general-purpose interior-point NLP solver.
- **acados** (https://docs.acados.org/): builds on CasADi models but generates tailored C code for real-time embedded execution.

As a back-of-the-envelope bound, a synchronous solve taking `5 ms` caps the loop at `200 Hz` before accounting for preprocessing, IPC overhead, and OS jitter—which is why asynchronous execution or a separate inner loop is standard on dynamic hardware.

### Linear MPC Python Example

To solve Linear MPC with OSQP, we transcribe the `N`-step horizon into a single sparse QP over the stacked decision vector `z = [x(0), ..., x(N), u(0), ..., u(N-1)]`. Dynamics enter as block-banded equality constraints, state and force limits become box bounds, and only the first optimal input `u(0)` is returned:

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
    Ax_eq = sparse.kron(sparse.eye(N + 1), -sparse.eye(n), format='lil')  # format that allows slice assignment
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

- **Boston Dynamics Atlas**: according to Boston Dynamics' published technical descriptions, MPC sits at the core of Atlas's dynamic behaviors. For whole-body manipulation tasks such as picking up and throwing heavy objects, the predictive model coupled whole-body joint kinematics, link momenta, and object interaction forces directly.

---

## 6.6 Impedance/Admittance Control

PID, LQR, and basic MPC all focus primarily on moving a robot to a target pose in free space. The moment a robot pushes against a rigid table, wipes a surface, or inserts a peg, pure position control turns hazardous—making it necessary to regulate the mechanical interaction itself.

### Position Control vs Force Control vs Impedance Control

Physical interaction highlights a sharp contrast among three control paradigms:

- **Position Control**: drives tracking error to zero. That works well in free space or against soft materials, but against a stiff surface any tiny calibration offset explodes into huge forces (`F = k_env * Δx`). If a robot reaches for a cup on a table whose height is off by even `1 mm`, a stiff position loop blindly tries to crush through the surface.
- **Force Control**: regulates a desired contact force directly, which is essential for polishing, grinding, or precision assembly. In free space with zero reaction force, however, pure force control accelerates uncontrollably, and on contact it remains sensitive to force-sensor noise.
- **Impedance Control**: rather than choosing position *or* force in isolation, impedance control shapes the dynamic *relationship* between them—making the robot behave like a programmable virtual spring-damper. In free space it tracks the desired pose smoothly; on contact, force rises gently in proportion to deflection rather than spiking.

### Virtual Spring-Damper Model

Impedance control commands actuator forces so the end-effector obeys a target mass-spring-damper relation around the desired trajectory `(x_d, x_dot_d, x_ddot_d)`:

```
F = M_d * (x_ddot_d - x_ddot) + D_d * (x_dot_d - x_dot) + K_d * (x_d - x)
```

Because shaping virtual inertia `M_d` cleanly requires noisy acceleration feedback or force sensing, practical implementations often drop the inertia term and run pure stiffness-damping impedance:

```
F = K_d * (x_d - x) + D_d * (x_dot_d - x_dot)
```

- `K_d`: virtual stiffness—higher values tighten free-space tracking at the cost of stiffer contact forces.
- `D_d`: virtual damping—absorbs kinetic energy to prevent ringing on impact.
- `M_d`: virtual inertia—shapes apparent mass during fast impacts, though often omitted unless force/acceleration feedback is clean.

Tuning `K_d` and `D_d` directly tailors the robot's mechanical personality to the task:
- Grasping a fragile glass: low `K_d` for gentle contact, higher `D_d` for well-damped settling.
- Tightening a bolt: high `K_d` for crisp alignment.
- Physical human-robot collaboration: very low `K_d` so accidental bumps remain harmless.

The script below simulates a 1-DOF impedance-controlled robot commanded to reach `0.15 m` while an unmodeled stiff wall (`10,000 N/m`) sits in the way at `0.10 m`—showing how virtual compliance settles calmly against the wall instead of generating a `500 N` impact spike:

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

Impedance and admittance control are causal duals of each other: impedance takes motion deviation in and outputs a force (`position → force`), whereas **admittance control** measures external contact force and outputs a modified position or velocity reference (`force → position`):

```
x_d_new = x_d + (1 / K_d) * F_ext      # stiffness term: force -> position offset
x_dot_d = (1 / D_d) * F_ext             # damping term: force -> desired velocity (not the force derivative)
```

In practice, an external wrist force/torque (F/T) sensor reads `F_ext`, integrates or scales it through the virtual compliance law to shift `x_d`, and hands the compliant target to an existing inner position controller.

This explains the hardware split seen across robotics: commercial industrial arms ship with locked, high-precision inner position servos and rarely expose raw joint-torque commands, making admittance control via a wrist F/T sensor the practical choice. Torque-controlled research manipulators (such as the Franka Emika Panda), by contrast, implement impedance control directly in torque space.

---

## 6.7 Advanced: Whole-Body Control

Humanoids and quadrupeds pack dozens of actuated joints onto an unbolted floating base (Ch. 5, §5.8), juggling multiple foot and hand contacts while staying balanced. Running independent joint PID loops on such a machine breaks down immediately—coordinating balance, manipulation, and contact constraints demands whole-body control (WBC).

### Task-Space vs Joint-Space

Whole-body controllers work natively in task space rather than joint space:

- **Joint-space control**: regulates joint angles `q` directly. Simple when targets are defined per joint, but achieving Cartesian goals (hand poses, center-of-mass trajectories) requires solving inverse kinematics upstream first.
- **Task-space control**: regulates task coordinates directly (end-effector pose, CoM position, torso orientation), letting the controller handle the mapping to joint torques internally using the robot's Jacobian and inertia matrix.

### Operational Space Control (Khatib, 1987)

Khatib's Operational Space Framework provides the analytical bridge from joint torques to task-space behavior by projecting the manipulator equation directly into task coordinates.

Starting from the joint-space dynamics with external contact forces `F_ext`:

```
M(q) * q_ddot + C(q, q_dot) * q_dot + g(q) = tau + J^T * F_ext
```

Projecting through the task Jacobian `J` rewrites the motion directly in Cartesian task coordinates `x`:

```
Lambda(q) * x_ddot + mu(q, q_dot) * x_dot + p(q) = F + F_ext
```

Here `Lambda = (J * M^(-1) * J^T)^(-1)` represents the apparent **task-space inertia matrix** felt at the end-effector. Commanding the joint torques:

```
tau = J^T * Lambda * (x_ddot_d - J_dot * q_dot) + C * q_dot + g(q)
```

realizes any desired task-space acceleration `x_ddot_d`—and plugging an impedance law into `x_ddot_d` yields compliant Cartesian impedance control across a multi-link arm.

### QP-Based Whole-Body Control

Modern legged and humanoid robots extend operational space control into a Quadratic Program (QP) solved at every tick (often `500–1000 Hz`). Why a QP? Because unlike unconstrained operational space control, a QP enforces the full floating-base dynamics, joint torque limits, and unilateral contact constraints simultaneously while juggling competing task objectives:

```
minimize    || J_task * q_ddot + J_task_dot * q_dot - x_ddot_d ||^2  (task tracking)
subject to  M(q)*q_ddot + h(q,q_dot) = S^T*tau + J_c^T*F_c  (dynamics)
            F_c ∈ friction cone                   (contact force constraint)
            tau_min ≤ tau ≤ tau_max               (torque limits)
```

Here `J_task` is the task Jacobian, `J_c` is the contact Jacobian, `F_c` denotes the ground contact forces, and `S` is the actuator selection matrix zeroing out the unactuated floating-base DOFs.

**Multi-task priority**: Real robots constantly face conflicting demands—"reach the doorknob with the right hand," "keep the CoM balanced," and "stay clear of joint limits." A typical strict priority ladder orders them as:

1. Highest priority: physical contact constraints (stance feet must not slip or lift) and joint limits.
2. High priority: balance and CoM / centroidal momentum regulation.
3. Medium priority: end-effector trajectory tracking.
4. Low priority: nominal posture regularization in the remaining null space.

Strict priorities can be enforced via null-space projections or sequential **hierarchical QPs** solved top-down; alternatively, **soft priorities** stack all objectives into a single weighted QP for faster, simpler execution.

### Contact-Consistent Control

For a legged robot's stance feet to push reliably without slipping or tipping, the QP restricts every contact wrench `F_c` to physically realizable bounds:

- **Unilateral contact**: feet can push into the ground, never pull (`F_z >= 0`).
- **Friction cone**: tangential shear cannot exceed normal force scaled by friction (`sqrt(F_x^2 + F_y^2) <= mu * F_z`).
- **ZMP/CoP constraint**: the center of pressure must stay inside the foot's support polygon to avoid rolling onto an edge.

Because the Euclidean friction cone is a quadratic second-order cone rather than linear, QP-based WBC typically approximates the circular cone with an inscribed `n`-sided polyhedron (`n_edges = 8` below), turning it into simple linear inequalities `A_cone * F <= 0`:

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

Watching a controller succeed across ten simulation runs is encouraging, yet it offers no mathematical guarantee that the next initial condition won't blow up. Lyapunov theory supplies that missing guarantee—proving stability for nonlinear systems without having to solve their differential equations in closed form.

### Lyapunov Stability

The intuition comes directly from physical energy: if a damped pendulum starts with positive energy and loses energy continuously whenever it moves, it must eventually settle at the bottom equilibrium.

Formally, consider an autonomous nonlinear system `x_dot = f(x)` with equilibrium at the origin (`f(0) = 0`). Under **Lyapunov's direct method**, the origin is stable if you can construct a scalar function `V(x)` ("generalized energy") satisfying three conditions:

1. `V(0) = 0`
2. `V(x) > 0` for all `x != 0` (positive definite)
3. `V_dot(x) = dV/dx * f(x) <= 0` (non-increasing along system trajectories)

If `V_dot(x) < 0` strictly away from the origin, the equilibrium is **asymptotically stable**—every trajectory in its basin actually converges to zero over time.

The hard part is discovering `V(x)`, since no universal recipe exists for arbitrary nonlinear systems. Fortunately, physical structure often hands you a candidate: total mechanical energy (kinetic plus potential) works naturally for mechanical manipulators, while quadratic forms `V(x) = x^T * P * x` (using the Riccati solution `P`) serve as exact Lyapunov functions proving closed-loop LQR stability.

### Adaptive Control

Even a well-proven model-based controller degrades when physical parameters are unknown or shift mid-task—such as picking up an unweighed payload or encountering changing joint friction. **Adaptive control** handles structured uncertainty by pairing feedback control with an online parameter estimator running simultaneously.

Crucially, rigid-body manipulator dynamics are linear in the unknown inertial parameters `theta` (link masses, moments of inertia, friction coefficients), allowing us to factor out a known kinematic **regressor matrix** `Y`:

```
M(q)*q_ddot + C(q,q_dot)*q_dot + g(q) = Y(q, q_dot, q_ddot) * theta
```

Using a sliding tracking variable `s`, the adaptive control law updates the parameter estimate `theta_hat` continuously from tracking errors:

```
tau = Y * theta_hat - K_d * s
theta_hat_dot = -Gamma * Y^T * s
```

where `Gamma` is a positive-definite adaptation gain matrix. Analyzing the composite Lyapunov candidate `V = 0.5*s^T*M*s + 0.5*theta_tilde^T*Gamma^(-1)*theta_tilde` establishes `V_dot <= 0`, proving that tracking error converges to zero—even though `theta_hat` itself need not converge to the true physical `theta` unless the trajectory sufficiently excites every parameter.

### Robust Control

When parameters are uncertain but lie within known bounded intervals, **robust control** designs a fixed controller guaranteed to hold across the worst-case mismatch without online parameter estimation:

- **H-infinity control**: minimizes worst-case disturbance amplification by bounding the induced `L2` gain from disturbance energy to tracking error energy below a prescribed level $\gamma$. (Note that this bounds the *energy ratio* rather than absolute error—if disturbance energy grows, error scales proportionally—and solving the underlying Riccati inequalities or LMIs often yields conservative gains.)
- **Sliding Mode Control**: forces the system state onto a chosen lower-dimensional "sliding surface" in finite time and holds it there with discontinuous switching feedback. Highly insensitive to bounded disturbances and model error, its classic drawback is high-frequency **chattering** that wears actuators—typically softened using a smooth boundary layer or higher-order sliding modes.

### When to Use, When Not to Use

Choosing among LQR, MPC, adaptive, and robust control comes down to how well you know the plant and what guarantees the application demands:

| Situation | Recommended | Not recommended |
|------|------|--------|
| Accurate model, sufficiently linear | LQR, MPC | Adaptive control (overdesign) |
| Large parameter uncertainty | Adaptive control | Relying on PID alone |
| Known uncertainty bound | Robust control (H-inf) | Adaptive control (unnecessary) |
| Safety certification required | Lyapunov-based proofs | "It worked in simulation so OK" |
| Rapid prototyping | PID + feedforward | H-infinity from the start |

In short, pick adaptive or sliding-mode schemes according to the structure of your uncertainty and actuation hardware. Whenever safety or formal guarantees matter, back up empirical simulation tests with a Lyapunov argument: while functional safety standards mandate hazard analysis, safety integrity levels, and structured verification and validation rather than a single mathematical proof method, Lyapunov certificates provide strong analytical evidence for that safety case.

---

## 6.9 Further Reading

> **Åström & Murray, "Feedback Systems: An Introduction for Scientists and Engineers"**
> https://fbswiki.org/
> Free PDF. An introductory text that connects PID, state-space methods, and frequency response.

> **Steve Brunton, "Control Bootcamp" (YouTube)**
> https://www.youtube.com/playlist?list=PLMrJAkhIeNNR20Mz-VpzgfQs5zrYi085m
> Explains state-space, controllability, observability, and LQR in videos of roughly 15 minutes each. Useful for orienting the concepts before following a textbook derivation.

> **Slotine & Li, "Applied Nonlinear Control"**
> A textbook on nonlinear control, Lyapunov stability, and adaptive control. It extends the material in Section 6.8 and is currently out of print.

> **Russ Tedrake, "Underactuated Robotics" (MIT OCW)**
> https://underactuated.csail.mit.edu/
> Free online textbook and lectures. Goes deep on MPC, trajectory optimization, and the connection between control and planning. Also the theoretical background of the Drake library.

> **python-control library**
> https://python-control.readthedocs.io/
> A Python library for analyzing and designing control systems. The Python alternative to MATLAB's Control System Toolbox. Supports Bode plots, root locus, and state-space analysis.

> **CasADi**
> https://web.casadi.org/
> A widely used framework connecting automatic differentiation to NLP solvers such as IPOPT and SNOPT. It offers Python, MATLAB, and C++ interfaces.

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
1990s ─ Robust control (H-infinity) theory established; early application attempts
2004 ── Real-time MPC becomes practical
2020s ─ Boston Dynamics Atlas: publicly demonstrated MPC covering whole-body motion and forces applied to objects
```

---

The references above develop the formal proofs and solver internals behind each method; experiment by running the code snippets and varying gains, weights, and horizons to see the closed-loop response shift directly. Notice one common assumption across PID, LQR, impedance control, and WBC: each takes a reference state or desired motion as given (and short-horizon MPC still looks only a few steps ahead). Where do globally collision-free, dynamically feasible trajectories from A to B come from? Answering that question is the subject of Ch. 7, on motion planning and trajectory optimization.
