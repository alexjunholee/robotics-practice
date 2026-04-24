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

---

## 7.9 Further Reading

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
