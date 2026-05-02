# Ch.7 — Motion Planning & Trajectory Optimization

For a robot to go from A to B, it needs a path. You might think a straight line is enough, but obstacles, joint limits, and dynamic constraints apply at once. Finding a path that satisfies all of these is motion planning; following that path optimally over time is trajectory optimization.

---

## 7.1 Why Study Motion Planning

Suppose you tell a 6-axis robot arm, "pick up that cup." IK gives the target joint angles. But linearly interpolating the joints from the current pose to the target pose can drive the arm through the table or into its own body. A straight line in joint space is not a straight line in task space.

Motion planning answers:
- Does a collision-free path to the goal exist?
- If so, what is the shortest / fastest / smoothest path?
- Can that path be followed while satisfying dynamic constraints (torque limits, velocity limits)?

---

## 7.2 Configuration Space (C-space)

Represent all possible states of the robot as a single space.

**Joint space = Configuration space**: for an n-DOF robot, the configuration is q = (q1, q2, ..., qn). The n-dimensional space where q lives is the C-space.

**C-space obstacle**: task-space (3D) obstacles transformed into the C-space. Configurations that fall inside the obstacle region in C-space are in collision.

Why think in C-space: the robot is not a point. Checking in 3D space that every link avoids the obstacles means computing FK at each configuration and running a collision check. In C-space the robot becomes a "point," and obstacle avoidance reduces to finding a path for a point.

The problem is that computing the exact shape of a C-space obstacle is hard. In practice you do not obtain C-space obstacles explicitly; you use a collision checker that tests whether a given configuration is in collision.

---

## 7.3 Graph Search-Based Planning

The most classical approach: discretize the C-space and find a path with a graph search algorithm.

### Dijkstra's Algorithm

Finds the shortest path in a weighted graph. It explores every edge, so it guarantees the optimum. Time complexity O((V + E) log V).

### A* Algorithm

Dijkstra plus a heuristic. An estimated distance to the goal (heuristic) guides the search direction. If the heuristic is admissible (no greater than the true distance), A* guarantees the optimum while running faster than Dijkstra.

```python
import heapq
import numpy as np

def astar_2d(grid, start, goal):
    """A* path search on a 2D grid.
    grid: 0=free, 1=obstacle
    """
    rows, cols = grid.shape
    open_set = [(0, start)]  # (f_score, node)
    came_from = {}
    g_score = {start: 0}

    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])  # Manhattan distance

    neighbors = [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]

    while open_set:
        f, current = heapq.heappop(open_set)
        if current == goal:
            # reconstruct the path
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]

        for dx, dy in neighbors:
            neighbor = (current[0] + dx, current[1] + dy)
            if (0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols
                    and grid[neighbor] == 0):
                cost = np.sqrt(dx**2 + dy**2)
                tentative_g = g_score[current] + cost
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, neighbor))

    return None  # no path
```

### Pros and Cons

Grid-based planning guarantees **completeness** — if a solution exists, it is found. But it suffers from the **curse of dimensionality**. Discretizing the C-space of a 6-DOF robot arm into 100 cells per axis gives 100^6 = 10^12 cells. Effectively impossible.

Sampling-based planners emerged to address this.

---

## 7.4 Sampling-Based Planners

Rather than discretizing the C-space, sample it randomly and search for a path. In high-dimensional C-spaces this is the only practical approach.

### RRT (Rapidly-exploring Random Tree)

Proposed by LaValle (1998). The idea is simple:

```
1. Initialize the tree at the start.
2. Sample a random point q_rand in the C-space.
3. Find the node q_near in the tree closest to q_rand.
4. Extend from q_near toward q_rand by step_size to produce q_new.
5. If the path q_near → q_new is collision-free, add it to the tree.
6. If q_new is near the goal, terminate. Otherwise go back to 2.
```

```python
import numpy as np

class RRT:
    def __init__(self, start, goal, obstacle_fn, bounds, step_size=0.3, max_iter=5000):
        self.start = np.array(start)
        self.goal = np.array(goal)
        self.obstacle_fn = obstacle_fn  # config → bool (True if in collision)
        self.bounds = np.array(bounds)  # [[min_q1, max_q1], ...]
        self.step_size = step_size
        self.max_iter = max_iter
        self.nodes = [self.start]
        self.parents = {0: -1}

    def sample_random(self):
        # sample the goal with probability 10% (goal bias)
        if np.random.random() < 0.1:
            return self.goal
        return np.random.uniform(self.bounds[:, 0], self.bounds[:, 1])

    def nearest(self, q):
        dists = [np.linalg.norm(node - q) for node in self.nodes]
        return np.argmin(dists)

    def steer(self, q_near, q_rand):
        direction = q_rand - q_near
        dist = np.linalg.norm(direction)
        if dist < self.step_size:
            return q_rand
        return q_near + (direction / dist) * self.step_size

    def collision_free(self, q1, q2, n_checks=10):
        for t in np.linspace(0, 1, n_checks):
            q = q1 + t * (q2 - q1)
            if self.obstacle_fn(q):
                return False
        return True

    def plan(self):
        for i in range(self.max_iter):
            q_rand = self.sample_random()
            idx_near = self.nearest(q_rand)
            q_near = self.nodes[idx_near]
            q_new = self.steer(q_near, q_rand)

            if self.collision_free(q_near, q_new):
                idx_new = len(self.nodes)
                self.nodes.append(q_new)
                self.parents[idx_new] = idx_near

                if np.linalg.norm(q_new - self.goal) < self.step_size:
                    # reconstruct the path
                    path = [q_new]
                    idx = idx_new
                    while self.parents[idx] != -1:
                        idx = self.parents[idx]
                        path.append(self.nodes[idx])
                    return path[::-1]

        return None  # failure
```

### RRT* (Optimal RRT)

Karaman & Frazzoli (2011). RRT finds a solution but not an optimal one. RRT* re-wires nearby nodes when adding a new node, guaranteeing asymptotic optimality. As the number of samples goes to infinity, it converges to the optimal path.

In practice, RRT* finds better paths than RRT but converges slowly. Under real-time deadlines, RRT-Connect is often more practical.

### PRM (Probabilistic Roadmap)

Kavraki et al. (1996). RRT is single-query (one start-goal pair at a time); PRM is suited to multi-query settings.

Phase 1 (offline): sample many points in the C-space and connect nearby points with collision-free edges to build a roadmap (graph).
Phase 2 (online): connect start and goal to the roadmap and find a path by graph search (A*, etc.).

When many path queries are needed in the same environment (e.g., an industrial robot cell), PRM is efficient.

### RRT-Connect

Kuffner & LaValle (2000). Grow trees from the start and the goal at the same time, and connect the paths when the two trees meet. The most widely used variant in practice; MoveIt2's default planner is also RRT-Connect (via OMPL).

### The OMPL Library

Open Motion Planning Library (https://ompl.kavrakilab.org/). A C++ library from the Kavraki Lab (Rice University) providing dozens of sampling-based planners — RRT, RRT*, RRT-Connect, PRM, EST, KPIECE, and more.

OMPL itself does not perform collision checking. The user supplies a state validity checker. MoveIt2 combines OMPL with FCL (Flexible Collision Library) to form a complete motion planning pipeline.

```python
# OMPL-based motion planning in MoveIt2 (ROS2 Python API, simplified)
from moveit_py import MoveItPy

moveit = MoveItPy(node_name="motion_planner")
arm = moveit.get_planning_component("manipulator")

# set the goal
arm.set_goal_state(configuration_name="home")

# plan (default: OMPL RRT-Connect)
plan_result = arm.plan()

if plan_result:
    # execute
    arm.execute()
```

> **Further reading**
> - [LaValle, "Planning Algorithms"](http://lavalle.pl/planning/) — free online textbook. The standard reference for motion planning.
> - [OMPL](https://ompl.kavrakilab.org/) — open-source motion planning library.
> - [MoveIt2 Tutorials](https://moveit.picknik.ai/) — hands-on motion planning guide on ROS2.

---

## 7.5 Trajectory Optimization

Sampling-based planners hand back a "collision-free path." But that path is:
- jagged (because of random sampling)
- oblivious to dynamics (only the kinematic path)
- without timing (no speed to follow it at)

Trajectory optimization fills the gap. It finds a trajectory that minimizes a cost function (time, energy, smoothness) while satisfying dynamic constraints, collision avoidance, and joint limits.

### Direct Collocation

Partition the trajectory into time intervals and treat the state and input at each interval as decision variables. Dynamics equations are handled as equality constraints.

```
minimize    Σ_k L(x_k, u_k) * dt                    (cost)
subject to  x_{k+1} = f(x_k, u_k)   for all k       (dynamics)
            g(x_k) <= 0              for all k       (inequality constraints: collisions, joint limits)
            x_0 = x_init                             (initial condition)
            x_N = x_goal                             (terminal condition)
```

Cast this as one large nonlinear program (NLP) and solve it with a solver such as IPOPT.

Pros: handles dynamics and constraints at the same time, smooth trajectories.
Cons: sensitive to the initial guess; non-convex, so it can fall into local optima.

### Direct Shooting

Drop the state from the decision variables and keep only the input sequence {u_0, u_1, ..., u_{N-1}} as decision variables. States are computed by dynamics simulation.

Fewer decision variables than collocation, but if the simulation is unstable (e.g., an inverted pendulum) the optimization becomes unstable too.

### CHOMP (Covariant Hamiltonian Optimization for Motion Planning)

Ratliff et al. (2009). Start from an initial trajectory (usually linear interpolation) and iteratively improve it by following the gradient of a collision cost plus a smoothness cost. A covariant gradient keeps the updates smooth.

Pros: intuitive, improves an existing trajectory incrementally.
Cons: struggles with narrow passages, local optima.

### TrajOpt

Schulman et al. (2014). Based on sequential convex optimization: at each iteration the nonlinear problem is replaced by a linear/quadratic approximation solved as a QP, and a trust region ensures convergence. Collision avoidance uses a signed distance function to yield a continuous gradient.

### Trajectory Optimization with CasADi

CasADi is a framework that combines symbolic computation, automatic differentiation, and connections to NLP solvers. It is the de facto standard tool for trajectory optimization.

```python
import casadi as ca
import numpy as np

# Simple example: time-optimal trajectory for a 1D double integrator
# x = [position, velocity], u = force
# x_dot = [velocity, force/mass]

N = 50           # number of intervals
dt = 0.1         # time step
mass = 1.0

opti = ca.Opti()

# decision variables
X = opti.variable(2, N + 1)  # state trajectory
U = opti.variable(1, N)      # input trajectory

# cost: minimize energy + time penalty
cost = 0
for k in range(N):
    cost += U[0, k]**2 * dt  # energy
opti.minimize(cost)

# dynamics constraint (Euler integration)
for k in range(N):
    x_next = X[:, k] + ca.vertcat(X[1, k], U[0, k] / mass) * dt
    opti.subject_to(X[:, k + 1] == x_next)

# boundary conditions
opti.subject_to(X[:, 0] == ca.vertcat(0, 0))    # start: position 0, velocity 0
opti.subject_to(X[:, N] == ca.vertcat(1, 0))     # end: position 1, velocity 0

# input constraint
opti.subject_to(opti.bounded(-5.0, U, 5.0))

# state constraint (velocity limit)
opti.subject_to(opti.bounded(-2.0, X[1, :], 2.0))

# solver setup
opti.solver('ipopt', {'print_time': False}, {'print_level': 0})
sol = opti.solve()

x_opt = sol.value(X)
u_opt = sol.value(U)
print(f"Optimal trajectory - final position: {x_opt[0, -1]:.4f}")
print(f"Max force: {np.max(np.abs(u_opt)):.4f} N")
```

> **Further reading**
> - [Matthew Kelly, "An Introduction to Trajectory Optimization" (SIAM Review 2017)](https://www.matthewpeterkelly.com/research/MatthewKelly_IntroTrajectoryOptimization_SIAM_Review_2017.pdf) — solid tutorial comparing collocation and shooting.
> - [CasADi](https://web.casadi.org/) — standard tool for NLP implementation.
> - [Drake Trajectory Optimization](https://drake.mit.edu/) — includes direct collocation examples.

---

## 7.6 MoveIt2: Motion Planning in Practice

MoveIt2 is a ROS2-based motion planning framework. It is the most widely used robot arm planning tool in industry and research alike.

**Architecture:**
- **Planning Scene**: manages the 3D model of the robot plus the environment (obstacles). The basis for collision checking.
- **Planning Pipeline**: call a planner such as OMPL → validate the path → time parameterization.
- **Move Group Interface**: the user-facing API. Abstracts goal setting, planning, and execution.

**OMPL integration**: MoveIt2 uses OMPL as its default planning backend. Planner type and parameters are set in `ompl_planning.yaml`.

```yaml
# ompl_planning.yaml example
manipulator:
  planner_configs:
    - RRTConnectkConfigDefault
    - RRTstarkConfigDefault
    - PRMkConfigDefault
  default_planner_config: RRTConnectkConfigDefault
  projection_evaluator: joints(joint1, joint2)
  longest_valid_segment_fraction: 0.01
```

**Pick-and-Place pipeline:**
1. Object recognition (Perception) → estimate the object's 6-DoF pose.
2. Grasp planning → decide the grasp location/pose.
3. Approach trajectory → plan motion to the approach point above the object.
4. Grasp → close the gripper.
5. Retreat trajectory → lift the object.
6. Place trajectory → plan motion to the placement location.
7. Release → open the gripper.

At every stage, MoveIt2 handles collision avoidance and joint limits automatically.

---

## 7.7 Advanced: Optimization-Based Planning

*If you want to become a researcher, start reading here.*

### Constrained Nonlinear Optimization

Trajectory optimization for real robots is mostly a constrained NLP:

```
minimize    Σ L(x_k, u_k) + Φ(x_N)
subject to  x_{k+1} = f(x_k, u_k)           (dynamics)
            h(x_k, u_k) = 0                  (equality constraints)
            g(x_k, u_k) <= 0                 (inequality constraints: collisions, torque limits, etc.)
```

IPOPT (Interior Point Optimizer) is the standard solver for this problem. CasADi uses IPOPT by default.

### Contact-Implicit Trajectory Optimization

Rather than fixing the contact mode in advance (what touches what, what is separated), let the optimization decide automatically. Useful for tasks with contact transitions, such as walking and grasping.

Include contact forces in the decision variables and add complementarity constraints:

```
F_n >= 0                   (contact force cannot pull)
d >= 0                     (object cannot go below the floor)
F_n * d = 0                (zero force when separated, zero distance when in contact)
```

Mathematically this is an MPCC (Mathematical Program with Complementarity Constraints), and it is hard to solve. Relaxation techniques or smoothed contact models are used.

Drake's `ContactImplicitDirectCollocation` implements this method.

### Connection to Real-Time Re-planning and MPC

In a static environment, planning once is enough; in a dynamic environment you must re-plan in real time. Trajectory optimization and MPC meet here.

MPC can be viewed as trajectory optimization over a short horizon. At every control cycle, optimize the trajectory over a short interval, apply only the first input, then optimize again. The MPC of the previous chapter is exactly this.

The difference: motion-planning trajectory optimization usually computes the full trajectory offline in one pass, while MPC recomputes a short interval online.

---

## 7.8 Advanced: Task and Motion Planning (TAMP)

*If you want to become a researcher, start reading here.*

To carry out "place the cup on the shelf":

1. Recognize where the cup is.
2. Decide a grasp pose that can pick up the cup.
3. Plan the sequence approach → grasp → lift → move → place.
4. Motion plan each stage.

Steps 1-3 are **symbolic planning** (which actions, in what order); step 4 is **motion planning** (which concrete trajectory to move along). TAMP combines the two.

### PDDLStream

A TAMP framework developed at MIT. Symbolic actions are defined in PDDL (Planning Domain Definition Language); streams generate continuous parameters (grasp pose, placement pose).

### LLM-Based Task Planning

Recently, attempts to replace the symbolic planner with an LLM have been active:

- **SayCan** (Google, 2022): the LLM scores natural-language descriptions of possible actions, and an affordance model filters for actions executable in the current state. The product of the two picks the next action.
- **Code as Policies** (Google, 2023): the LLM generates robot control code directly. Natural-language command → Python code → robot execution.
- **Inner Monologue** (Google, 2023): completes a task through iterative dialogue between the LLM and environment feedback.

Practical limits: LLM-based TAMP is still experimental. Complex geometric constraints (manipulation in tight spaces, precision assembly) are hard for LLMs to handle, and traditional motion planners are still needed in the end. A realistic division of labor: LLM for high-level planning, motion planner for low-level execution.

TAMP assumes the environment dynamics and actions are deterministic. When both dynamics and observations are stochastic, see §7.9 Advanced: POMDP.

---

## 7.9 Advanced: Decision-Making Under Uncertainty (POMDP and Belief Space Planning)

*If you want to become a researcher, start reading here.*

§7.1–§7.8 assumed the robot knows its own state and the environment exactly. Real robots observe only partial information through noisy sensors. A robot that does not know which side of a symmetric corridor it is on, a situation where it is uncertain whether a door is open or closed — planning from the "current best-estimate state" fails in these cases. The plan must be built directly over the belief (the posterior distribution). Thrun, Burgard, and Fox's *Probabilistic Robotics* §15.2 and §16 address this problem.

### 7.9.1 Introduction: Three Paradigms

Three planners give different answers in the same environment. Take a left-right symmetric corridor with a Goal, a Pit, and a Robot.

Classical planning assumes the state is fully known and actions are deterministic. A* from §7.3 belongs here: compute the shortest path once, and no sensing is needed during execution.

**MDP (Markov Decision Process)**: full state observability, stochastic actions. A policy $\pi: s \to a$ maps every state to an action. In a narrow passage the planner can choose a wider route to reduce the risk of hitting a wall. Ch.8 §8.2 falls in this category.

**POMDP**: both actions and observations are stochastic. The policy $\pi: b \to a$ is defined over belief $b$. In the symmetric corridor the robot starts without knowing its position, so it deliberately detours into an asymmetric region to gather information before heading for the goal. This is **active information gathering**.

The three paradigms are nested: classical $\subset$ MDP $\subset$ POMDP. Uncertainty has two axes: action uncertainty (where you tried to go versus where you actually went) and perceptual uncertainty (where you actually are versus what the sensor read). MDP handles the first; POMDP handles both.

Ch.3's filters *tracked* the belief; this section addresses what to *do* with the tracked belief.

<!-- DEMO: pomdp_three_paradigms.html -->

### 7.9.2 Value Iteration over Belief

Comparing the equations of the three paradigms shows immediately where POMDP becomes hard. The core equation of MDP value iteration is the Bellman equation:

$$C^T(s) = \max_a \int \left[ c(s') + C^{T-1}(s') \right] P(s' \mid a, s)\, ds'$$

Replace state $s$ with belief $b$ and the POMDP value iteration follows:

$$C^T(b) = \max_a \int \left[ c(b') + C^{T-1}(b') \right] P(b' \mid a, b)\, db' \tag{16.2}$$

The policy is:

$$\pi^T(b) = \arg\max_a \int \left[ c(b') + C^{T-1}(b') \right] P(b' \mid a, b)\, db' \tag{16.3}$$

The problem is that $b'$ is a distribution over distributions. $b$ is a probability distribution over the state space $\mathcal{S}$, and $b'$ is itself a distribution over that space of distributions. The integral dimension diverges.

In the infinite-horizon limit, if this recursion converges, we get the standard Bellman equation:

$$V(b) = \max_a \left[ r(b, a) + \gamma \sum_{o'} P(o' \mid b, a)\, V(B(b, a, o')) \right]$$

where $r(b,a) = \sum_s b(s)\, c(s,a)$ is the expected immediate reward over the belief. Written as a finite-horizon recursion this is eq. (16.2).

There is a key trick for handling this. Once observation $o'$ is determined, the posterior belief $B(b, a, o')$ is *uniquely* determined by the Bayes filter. So the integral over the entire belief space can be recast as an integral over the observation space:

$$C^T(b) = \max_a \int \left[ c(B(b, a, o')) + C^{T-1}(B(b, a, o')) \right] P(o' \mid a, b)\, do' \tag{16.34}$$

The belief update operator is:

$$B(b, a, o')(s') = \frac{1}{P(o' \mid a, b)}\, P(o' \mid s') \int P(s' \mid a, s)\, b(s)\, ds$$

In discrete state and observation spaces the integrals become sums. This reformulation is the starting point for all modern POMDP solvers.

### 7.9.3 Four-State Toy Example

To see the PWLC (piecewise-linear convex) structure directly, a small example helps. Work through a 4-state, 2-action, 2-observation problem by hand.

**Setup:**
- States $s_1, s_2, s_3, s_4$. Initially in one of $(s_1, s_2)$.
- Action $a_1$: information gathering. Swaps $s_1 \leftrightarrow s_2$ with probability 0.9.
- Action $a_2$: termination. Moves to $s_3$ (reward +80) or $s_4$ (reward −80).
- Observations $o_1, o_2$: probabilities $(0.7, 0.3)$ from $s_1$, $(0.4, 0.6)$ from $s_2$.
- Belief $b = (p_1, p_2)$ with $p_1 + p_2 = 1$, so it is one-dimensional.

**Horizon 1 computation:**

The immediate reward is linear in belief: $c(b) = \sum_i c(s_i) p_i$.

Taking $a_2$ gives the $T=1$ value ($\gamma = 0.9$):
$$C^1(b, a_2) = \gamma(80 p_1 - 80 p_2) = 72 p_1 - 72 p_2$$

Taking $a_1$ yields no termination, so only the immediate reward: $C^1(b, a_1) \approx 0$.

Therefore:
$$C^1(b) = \max\{ 0,\; 72p_1 - 72p_2 \}$$

$C^1(b)$ is the max of two linear functions. It bends at $p_1 = 0.5$. If $p_1 > 0.5$, take $a_2$; otherwise take $a_1$.

**Horizon 2 computation:**

Integrating over the probabilities of observing $o_1, o_2$ after $a_1$:
$$C^2(b, a_1) \approx \max\{0,\; -33.05 p_1 + 7.78 p_2\}$$

(Coefficients are computed through the observation probabilities and the belief update.)

Full $T=2$:
$$C^2(b) = \max\{ 0,\; -33.05 p_1 + 7.78 p_2,\; 72 p_1 - 72 p_2 \}$$

The max of three linear pieces. As the horizon grows, more pieces are added.

"Knowledge always helps" — $\beta C(b) + (1-\beta) C(b') \geq C(\beta b + (1-\beta) b')$. The value at a certain belief is always at least as high as at an uncertain one.

<!-- DEMO: pomdp_toy_pwlc.html -->

### 7.9.4 PWLC Structure and Alpha-Vectors

The four-state example showed that the value function takes the form of a *max of linear pieces*. This is not a coincidence; the following inductive argument shows it holds in general.

Base case ($T=1$): the immediate reward $c(b) = \sum_i c(s_i) p_i$ is linear in belief. So $C^1(b) = \max_a \sum_i C^1_{a,i}\, p_i$ — one linear function per action.

Inductive step: suppose $C^{T-1}(b)$ is PWLC. Expanding $C^{T-1}(B(b,a,o'))$ as a function of $b$ in eq. (16.34): the nonlinear normalization factor $1/P(o'\mid a, b)$ in the belief update cancels with the weight $P(o'\mid a, b)$ in eq. (16.34), so each inner product $\langle \phi, B(b,a,o') \rangle \cdot P(o'\mid a, b)$ reduces to a linear function of $b$. The max of a max of linear functions is still a max of linear functions. So $C^T(b)$ is PWLC.

Each coefficient vector of a linear piece is called an **alpha-vector** $\phi$. The value function is:

$$V(b) = \max_\phi \langle \phi, b \rangle$$

With $\Phi$ the set of alpha-vectors, $V(b) = \max_{\phi \in \Phi} \sum_i \phi_i\, p_i$.

Each alpha-vector corresponds to one *conditional policy* (current action plus subsequent policy contingent on observations). The count $|\Phi^T| = |A| \cdot |\mathcal{O}|^{|\Phi^{T-1}|}$ grows doubly exponentially. Starting from $|\Phi^1| = 1$: $|\Phi^2| = 2 \cdot 2^1 = 4$, $|\Phi^3| = 2 \cdot 2^4 = 32$, $|\Phi^4| = 2 \cdot 2^{32} \approx 10^{10}$. By horizon 4 we are already above ten billion vectors. This is why exact solutions are impractical.

### 7.9.5 LP Solution

If the doubly exponential growth in alpha-vector count is the problem, there is a way to reduce the max–sum–max structure to a linear program (LP) directly, computing an exact solution without enumerating alpha-vectors.

**Reduction principle**: $C = \max_a x(a)$ is solved as $\min C$ subject to $\{C \geq x(a) \;\forall a\}$. For $C = \sum_i \max_a x(a,i)$, introduce a function $a(\cdot)$ selecting an action per state and add $\{C \geq \sum_i x(a(i),i)\}$ for every combination. The number of constraints is $|A|^{|\mathcal{S}|}$.

POMDP horizon $T$ constraints (eq. 16.67):

$$\bigcup_a \bigcup_{k(o'):1 \leq k(o') \leq |\Phi^{T-1}|} \left\{ C^T(b) \geq \gamma \sum_{o'} \sum_i \left(c_i + C^{T-1}_{k(o'),i}\right) P(o' \mid s_i') \sum_j P(s_i' \mid a, s_j)\, p_j \right\}$$

The number of constraints is $|\Phi^T| = |A| \cdot |\mathcal{O}|^{|\Phi^{T-1}|}$.

---

**Algorithm: finite_world_POMDP** (Thrun et al., Table 16.1, adapted)

```
Algorithm finite_world_POMDP(T):
  Φ¹ = { φ : C¹(b) = γ Σᵢ c(sᵢ) pᵢ }     # single alpha-vector for horizon 1

  for t = 2 to T:
    Φᵗ = ∅
    for each action a:
      for each assignment k(o') ∈ {1, …, |Φᵗ⁻¹|} for each o':
        # compute new alpha-vector
        for each state sⱼ:
          φⱼ = γ Σₒ' Σᵢ (cᵢ + Φᵗ⁻¹[k(o'), i]) · P(o'|sᵢ') · P(sᵢ'|a, sⱼ)
        Φᵗ = Φᵗ ∪ { ⟨a, φ⟩ }

  # remove dominated alpha-vectors (pruning)
  Φᵀ = prune(Φᵀ)
  return Φᵀ
```

---

$|\Phi^T|$ grows doubly exponentially. With a horizon of 5, 3 actions, and 5 observations, millions of alpha-vectors are already needed, and even with pruning the number is unmanageable in realistic domains. The exact solution is a proof of concept; approximation is essential in practice.

### 7.9.6 General POMDP

If the LP solution is already impractical for discrete finite-state problems, continuous state spaces make things worse.

With a continuous state space, alpha-vectors become continuous functions. Eq. (16.34) still holds in principle, but $\Phi^{T-1}$ becomes a set of functions — infinite-dimensional.

---

**Algorithm: POMDP(T)** (Thrun et al., Table 16.2, adapted, compressed)

```
Algorithm POMDP(T):
  initialize: Φ¹ ← value function at horizon 1 (continuous)

  for t = 2 to T:
    for each action a:
      for each "conditional plan" k(·) mapping observations to Φᵗ⁻¹ elements:
        new function φ(b) = γ ∫ₒ' [ c(B(b,a,o')) + Φᵗ⁻¹[k(o')](B(b,a,o')) ] P(o'|a,b) do'
        Φᵗ ← Φᵗ ∪ { φ }

  return Φᵀ
```

---

In continuous spaces, storing and comparing sets of functions is itself impractical. This algorithm is an in-principle solution; practical algorithms (MC-POMDP, AMDP) emerge as alternatives.

### 7.9.7 MC-POMDP

With exact solutions blocked, the next step is to approximate the belief with samples and bring computation down to a tractable level.

Represent the belief with a particle filter and approximate the value-iteration update on a sample basis. In ch.3 §3.11, the particle filter served for estimation; here it serves for *planning*.

Belief $\theta$ is a set of weighted particles $\langle s^{(i)}, w^{(i)} \rangle$. The belief update $B(b, a, o')$ is implemented in particle form:

```
Algorithm particle_filter_belief_update(θ, a, o'):
  θ' = ∅
  for i = 1 to N:
    s ~ θ                         # sample a particle
    s' ~ P(s'|a, s)               # motion model
    w' = P(o'|s')                 # measurement model
    θ' ← θ' ∪ { ⟨s', w'⟩ }
  normalize weights in θ'
  return θ'
```

The value-iteration update learns a Q-value $Q(\theta, a)$ per action for each belief $\theta$. Sample $N$ times at each belief, take the max Q from the next belief, and average:

---

**Algorithm: MC-POMDP** (Thrun et al., Table 16.3, skeleton adapted)

```
Algorithm MCPOMDP(belief_database):
  for each belief θ in database:
    V(θ) = −∞
    for each action a:
      Q(θ, a) = 0
      for i = 1 to N:
        s ~ θ
        s' ~ P(s'|a, s)
        o' ~ P(o'|s')
        θ' = particle_filter_belief_update(θ, a, o')
        Q(θ, a) += (1/N) · γ · [V(θ') + c(s')]
      if Q(θ, a) > V(θ):
        V(θ) = Q(θ, a)

  return V, policy σ(θ) = argmax_a Q(θ, a)
```

---

Q-function update (eq. 16.78):

$$Q(\theta_t, a_t) \leftarrow \mathbb{E}\left[ R(o_{t+1}) + \gamma \max_{\bar{a}} Q(\theta_{t+1}, \bar{a}) \right]$$

Policy (eq. 16.79):

$$\sigma^Q(\theta) = \arg\max_{\bar{a}} Q(\theta, \bar{a})$$

Q-value function approximation uses nearest-neighbor lookup. Because belief $\theta$ is a particle set, it cannot be fed directly into a feedforward network. Instead a database of $\langle \theta, a, Q \rangle$ tuples is maintained, and for a new belief $\theta'$ the $k$ nearest neighbors by KL divergence yield the average Q-value.

KL divergence between two beliefs is approximated with Gaussian KDE. KL-based kNN acts as the function approximator. Modern implementations replace this with neural function approximation, but the algorithmic skeleton is the same.

The outer loop either holds a static belief database or generates beliefs naturally through $\varepsilon$-greedy simulation trials — the latter concentrates computation on beliefs the real robot is likely to visit.

### 7.9.8 Experiments: Heaven/Hell and Find-and-Fetch

**Heaven/Hell problem**: in a T-shaped corridor, one end is heaven (+1) and the other is hell (−1). Only a priest near the entrance knows which is which. The robot must first ask the priest (information gathering), then head in the correct direction. The POMDP planner automatically learns a policy that detours to the priest. Going directly on the shortest path reaches hell 50% of the time; consulting the priest guarantees the right direction.

**Find-and-Fetch (monocular camera)**: the robot must find and retrieve a target object using a monocular camera. The camera gives the object's direction but not its distance. MC-POMDP learns a policy that actively changes viewpoint to reduce distance uncertainty, observing the object from multiple angles to narrow down its position before approaching.

Both experiments track the belief and include *information-gathering actions* in the plan. State estimation followed by greedy action selection alone never produces these detour policies.

### 7.9.9 AMDP — Dimensionality Reduction via Belief Statistics

MC-POMDP tracks the belief directly as a particle set. AMDP (Augmented MDP) rests on the observation that the same uncertainty can be summarized with far fewer statistics. The two extremes of POMDP are MDP (polynomial in $|S|$) and exact POMDP (doubly exponential). AMDP sits in between.

The idea: along real robot trajectories, belief does not fill the entire belief space but occupies a narrow manifold. Summarize that manifold with *low-dimensional statistics* $\bar{b} = f(b)$ and apply standard MDP value iteration over $\bar{b}$.

**Standard statistics** (eq. 16.80):

$$\bar{b} = \langle \arg\max_s b(s),\; H[b] \rangle$$

Most likely state plus belief entropy. Entropy:

$$H[b] = -\int b(s) \ln b(s)\, ds \tag{16.81}$$

An infinite-dimensional belief summarized by a single scalar. Whether this is a *sufficient statistic* is not guaranteed — Thrun et al. explicitly note that "the sufficient statistic assumption rarely holds" — but coastal navigation and heaven/hell experiments confirm it is enough to select reasonable actions.

Using $\arg\max_s b(s)$ alone is a standard MDP. Adding entropy to the state encodes "how much I do not know."

---

**Algorithm: Augmented_MDP_value_iteration** (Thrun et al., Table 16.4, adapted)

```
Algorithm Augmented_MDP_value_iteration():
  for all b̄:
    Ĉ(b̄) = 0

  repeat until convergence:
    for all b̄:
      Ĉ(b̄) ← max_a ∫ [c(b̄') + Ĉ(b̄')] P(b̄'|a, b̄) db̄'

  return Ĉ
  policy: π(b̄) = argmax_a ∫ [c(b̄') + Ĉ(b̄')] P(b̄'|a, b̄) db̄'
```

---

The form is identical to MDP_value_iteration (§15.3.3). The only difference is that the state is $\bar{b}$ instead of $s$.

Transition probability $P(\bar{b}' \mid a, \bar{b})$ (eq. 16.85):

$$P(\bar{b}' \mid a, \bar{b}) = \int\!\!\int\!\!\int I_{f(b)=\bar{b}}\, I_{f(B(o',a,b))=\bar{b}'}\, P(o' \mid s') P(s' \mid a, s) P(s \mid b)\, ds\, ds'\, do'\, db$$

In practice this is approximated by simulation with a lookup-table cache, estimating transitions statistically over many random trials.

### 7.9.10 Coastal Navigation Example

Coastal navigation is the easiest emergent behavior produced by AMDP to describe.

The motivation: crossing a wide open space, a conventional MDP planner takes the straight-line path because it is short. But in open space, lidar or a camera sees only featureless walls, so the entropy of the position belief grows substantially. The robot reaches the destination without knowing where it is.

In the same environment, an AMDP planner chooses a **curved path that follows the wall**. Near the wall, lidar measurements tightly constrain the position and entropy stays low. Entropy is part of the cost function, so preferring "information-rich" paths comes out automatically.

Analogy: a ship navigating without GPS follows the coastline. Landmarks are plentiful near the coast, which keeps the position estimate sharp.

In Thrun et al.'s Figure 16.5, as sensor range decreases the arrival entropy of the conventional planner rises steeply, while the coastal planner's arrival entropy barely changes. This is where the robustness of information-aware path planning shows up.

Selecting paths to reduce position uncertainty in Active SLAM, and moving toward the highest-information viewpoint in next-best-view planning, are both modern forms of coastal navigation.

<!-- DEMO: coastal_navigation_amdp.html -->

### 7.9.11 What Survived

Coastal navigation is the conclusion a planner reaches automatically once entropy is included in the cost function. The value iteration over belief that started in §7.9.2 produces coastal navigation as its answer, in concrete form.

The exact solutions (§7.9.5 and §7.9.6) are impractical but remain useful as conceptual tools. Modern POMDP solvers have branched into three families.

Point-based value iteration (SARSOP, HSVI, PBVI) performs alpha-vector backup only at sampled belief points, not across the full belief space. The alpha-vector structure from §7.9.4 is intact; restricting the search range prevents the explosion.

**MCTS family**: POMCP (Silver & Veness, 2010) and DESPOT. Rollouts estimate Q-values and belief trees are searched with MCTS. This combines the Q estimation structure of §7.9.7's MC-POMDP with tree search.

**Deep POMDP**: DRQN (Recurrent Q-network), DVRL (Igl et al.). The RNN's hidden state serves as an implicit belief. MC-POMDP's nearest-neighbor function approximation replaced by neural function approximation.

AMDP's core idea lives on under other names. Bayes-adaptive MDP (BAMDP) uses the posterior distribution over model parameters as an augmented state. Active SLAM includes the variance of the position belief in the cost function. In NeRF-based active perception, entropy-augmented planning has become a standard tool.

Ch.8 §8.3's deep RL methods (PPO, SAC) learn from experience and need no model (transition probabilities, observation model). POMDP planning computes the optimal policy when the model is known. Without a model, MC-POMDP does not run either. MC-POMDP sits at the intersection: belief tracked with the model, Q-values learned from experience.

Value iteration over belief is mathematically clean but scales doubly exponentially in the number of states — that is why approximate solvers exist. MC-POMDP represents belief with a particle filter and learns Q-values from samples. AMDP summarizes belief as a (most likely state, entropy) pair and reduces to a standard MDP. The methods differ, but the goal is one: active information gathering. Its modern forms are MCTS-based POMCP, Deep POMDP, and entropy-augmented planning in Active SLAM.

---

## 7.10 Further Reading

> **LaValle, "Planning Algorithms"**
> http://lavalle.pl/planning/
> Free online. The most comprehensive textbook on motion planning. Written by the originator of RRT, so naturally strong.

> **Russ Tedrake, "Underactuated Robotics" Ch.10: Trajectory Optimization**
> https://underactuated.csail.mit.edu/trajopt.html
> Hands-on trajectory optimization with Drake. Code and theory together.

> **Matthew Kelly, "An Introduction to Trajectory Optimization" (SIAM Review 2017)**
> https://www.matthewpeterkelly.com/research/MatthewKelly_IntroTrajectoryOptimization_SIAM_Review_2017.pdf
> Solid tutorial comparing direct collocation and shooting. Example code included.

> **OMPL**
> https://ompl.kavrakilab.org/
> Open-source motion planning library. Implements dozens of algorithms including RRT, RRT*, and PRM.

> **MoveIt2 Tutorials**
> https://moveit.picknik.ai/
> Hands-on motion planning on ROS2. From pick-and-place to advanced configuration.

> **Drake**
> https://drake.mit.edu/
> Integrates trajectory optimization with simulation. Contact-implicit support.

> **CasADi**
> https://web.casadi.org/
> Standard tool for implementing nonlinear trajectory optimization.

> **Additional papers**
> - [Garrett et al., "Integrated Task and Motion Planning" (2021, arXiv:2010.01083)](https://arxiv.org/abs/2010.01083) — the standard TAMP survey paper.
> - [Janner et al., "Planning with Diffusion for Flexible Behavior Synthesis" (ICML 2022, arXiv:2205.09991)](https://arxiv.org/abs/2205.09991) — the start of trajectory-level diffusion-based planning.

---

## Technical Timeline

```
1979 ── Visibility graph-based path planning
1996 ── PRM (Kavraki et al.) — the start of sampling-based planning
1998 ── RRT (LaValle) — the standard for single-query planning
2000 ── RRT-Connect (Kuffner & LaValle) — the most widely used variant in practice
2009 ── CHOMP (Ratliff et al.) — gradient-based trajectory optimization
2011 ── RRT* (Karaman & Frazzoli) — asymptotic optimality guarantee
2012 ── OMPL 1.0 released — unified library of sampling-based planners
2014 ── TrajOpt (Schulman et al.) — sequential convex optimization
2019 ── MoveIt2 (ROS2) — industrial/research standard framework
2022 ── SayCan (Google) — LLM + motion planning
2023 ── Contact-implicit trajectory optimization becomes practical
2024 ── LLM-based TAMP research spreads
```
