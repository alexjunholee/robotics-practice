# Ch.7 — Motion Planning & Trajectory Optimization

Controllers from Ch.6 can track a reference path and regulate contact forces, yet they still need a feasible reference to follow. Motion planning and trajectory optimization supply that reference: motion planning searches for a collision-free geometric path from A to B around obstacles and joint limits, while trajectory optimization refines that geometry into a smooth, time-parameterized trajectory that respects the robot's dynamics and minimizes a chosen cost.

---

## 7.1 Why Study Motion Planning

Suppose you command a 6-axis robot arm to pick up a cup. Inverse kinematics (Ch.4) yields the target joint angles, yet linearly interpolating every joint from the current pose to the target pose does not produce a straight line in 3D workspace—the links swing along curved arcs that can drive the arm straight through the table or into its own base.

Before the controller executes a single torque command, motion planning must answer three questions:
- Does a collision-free path from the start pose to the goal exist?
- If so, which path minimizes distance, travel time, or jerk?
- Can the robot follow that path while obeying dynamic constraints such as velocity and torque limits?

---

## 7.2 Configuration Space (C-space)

Planning directly in 3D workspace is awkward because an articulated robot is not a single point—at every candidate pose, every link occupies a different volume in space. **Configuration space (C-space)** flips that geometry on its head by encoding the robot's entire pose as one coordinate vector.

For an n-DOF robot, the **configuration** is the joint vector `q = (q1, q2, ..., qn)`, and the n-dimensional space of all such vectors is the **C-space** (for a standard manipulator, C-space coincides with joint space). Meanwhile, 3D obstacles in the workspace map onto **C-space obstacles**: the set of all configurations `q` where any link collides with the environment or another link.

Thinking in C-space reduces an articulated rigid-body avoidance problem to navigating a single dimensionless point through free C-space. The catch is that computing the explicit boundary of a C-space obstacle in high dimensions is intractable. Practical planners therefore never construct C-space obstacles explicitly; instead, they sample or search over configurations and query a fast geometric collision checker to test whether each candidate `q` is valid.

---

## 7.3 Graph Search-Based Planning

Once the problem is navigating a point through free C-space, the most direct strategy is to discretize the space into a grid or graph and run a classical shortest-path algorithm.

### Dijkstra's Algorithm

Dijkstra's algorithm finds minimum-cost paths on any graph with nonnegative edge weights by expanding vertices in order of cumulative cost from the start. Implemented with a binary heap and adjacency lists, its time complexity is $O((V+E)\log V)$, and a single-goal query can terminate as soon as the goal node is settled rather than traversing the entire graph.

### A* Algorithm

Dijkstra's algorithm expands outward symmetrically in every direction, including away from the target. A* speeds up search by adding a heuristic estimate of the remaining distance to the goal, steering node expansion toward promising regions. Graph-search A* guarantees an optimal path whenever the heuristic is admissible (never overestimates) and consistent (satisfies the triangle inequality). Although a strong heuristic typically cuts down node expansions dramatically, A* is not guaranteed to outperform Dijkstra on every graph topology.

The implementation below demonstrates A* on an 8-connected 2D occupancy grid, where Euclidean distance provides a natural consistent heuristic matching diagonal step costs.

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
        return np.hypot(a[0] - b[0], a[1] - b[1])  # Euclidean distance consistent with diagonal step costs

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

A search algorithm that explores every reachable cell on a finite grid is resolution-complete for that **discrete problem**, yet two limitations restrict grid search in robotics. First, a coarse discretization can seal off narrow passages that exist in continuous space. Second, uniform grids succumb to the **curse of dimensionality**: discretizing a 6-DOF robot arm into just 100 cells per axis produces 100^6 = 10^12 cells—far beyond practical memory or compute budgets.

Sampling-based planners emerged specifically to break through this dimensional barrier.

---

## 7.4 Sampling-Based Planners

Instead of partitioning the entire C-space into a fixed grid upfront, sampling-based planners probe free space with random configurations and link collision-free pairs into a tree or graph. Because they check validity only at sampled points and connecting segments, they scale gracefully to high-DOF manipulators and frequently pair with graph search or optimization-based smoothing.

### RRT (Rapidly-exploring Random Tree)

Proposed by LaValle (1998), RRT explores high-dimensional spaces by anchoring a tree at the start configuration and repeatedly pulling it toward random samples. Because samples drawn in large unexplored regions pull the nearest existing node outward (a property known as Voronoi bias), the tree expands rapidly across free space without exhaustive gridding:

```
1. Initialize the tree at the start.
2. Sample a random point q_rand in the C-space.
3. Find the node q_near in the tree closest to q_rand.
4. Extend from q_near toward q_rand by step_size to produce q_new.
5. If the path q_near → q_new is collision-free, add it to the tree.
6. If q_new is near the goal, terminate. Otherwise go back to 2.
```

The Python implementation below turns this loop into a runnable planner, adding a 10% goal bias so the tree steers toward the target whenever a clear corridor opens up, and validating each segment by testing interpolated points along the edge.

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

Standard RRT finds a feasible path quickly, yet its random branches often meander well beyond the shortest route. Karaman & Frazzoli (2011) resolved this with **RRT***: whenever a new node joins the tree, the planner inspects nearby vertices and rewires parent connections if routing through the new node yields a shorter path from the start. This local rewiring guarantees asymptotic optimality—as sample counts approach infinity, the solution converges toward the true minimum-cost path. In practice, however, RRT* converges slowly, so under tight real-time deadlines engineers often reach for RRT-Connect instead and smooth the path afterward.

### PRM (Probabilistic Roadmap)

While RRT solves a single start-to-goal query from scratch, **PRM** (Kavraki et al., 1996) amortizes planning cost across repeated queries in the same environment:

- **Phase 1 (offline)**: Sample configurations across the C-space and connect nearby collision-free pairs to construct a persistent roadmap graph.
- **Phase 2 (online)**: Attach the current start and goal to the roadmap and extract a path with standard graph search such as A*.

When a robot performs hundreds of picks inside a static industrial workcell, PRM avoids rebuilding the free-space representation on every cycle.

### RRT-Connect

Proposed by Kuffner & LaValle (2000), **RRT-Connect** grows two trees simultaneously—one rooted at the start and one at the goal—and greedily extends them toward each other until they meet. Because bidirectional search shrinks the unexplored gap from both ends, it finds an initial feasible path much faster than single-tree RRT, and several MoveIt2 OMPL example configurations select `RRTConnect` as the default planner config (though the actual default depends on the distribution and user configuration).

### The OMPL Library

The Open Motion Planning Library ([OMPL](https://ompl.kavrakilab.org/)), developed by the Kavraki Lab at Rice University, is a C++ library implementing dozens of sampling-based planners including RRT, RRT*, RRT-Connect, PRM, EST, and KPIECE. OMPL intentionally separates graph/tree exploration from geometry: it does not perform collision checking itself, instead delegating validity tests to a user-supplied state validity checker. MoveIt2 pairs OMPL with the Flexible Collision Library (FCL) to assemble an end-to-end motion planning pipeline, as shown in the simplified ROS2 snippet below.

```python
# OMPL-based motion planning in MoveIt2 (ROS2 Python API, simplified)
from moveit.planning import MoveItPy

moveit = MoveItPy(node_name="motion_planner")
arm = moveit.get_planning_component("manipulator")

# set the goal
arm.set_goal_state(configuration_name="home")

# plan (default: OMPL RRT-Connect)
plan_result = arm.plan()

if plan_result:
    # execute: PlanningComponent has no execute(); the MoveItPy object executes the trajectory
    moveit.execute(plan_result.trajectory, controllers=[])
```

> **Further reading**
> - [LaValle, "Planning Algorithms"](http://lavalle.pl/planning/) — free online textbook. The standard reference for motion planning.
> - [OMPL](https://ompl.kavrakilab.org/) — open-source motion planning library.
> - [MoveIt2 Tutorials](https://moveit.picknik.ai/) — hands-on motion planning guide on ROS2.

---

## 7.5 Trajectory Optimization

Sampling-based planners return a collision-free geometric path, yet that raw path has three drawbacks for real hardware:
- It is jagged because vertices come from random sampling.
- It ignores robot dynamics, checking only static kinematics.
- It carries no timing, leaving velocity and acceleration profiles unspecified.

Trajectory optimization bridges geometry and physics. Rather than merely connecting waypoints, it solves for a continuous, time-parameterized trajectory that minimizes an objective—such as duration, energy, or jerk—while satisfying dynamic equations, collision avoidance, and joint or torque limits.

### Direct Collocation

The most general way to transcribe a continuous trajectory optimization problem for a numerical solver is **direct collocation**. Instead of integrating dynamics sequentially, collocation discretizes the horizon into `N` time intervals and treats both the states `x_k` and control inputs `u_k` at every knot point as simultaneous decision variables, enforcing the system dynamics as algebraic equality constraints between adjacent steps:

```
minimize    Σ_k L(x_k, u_k) * dt                    (cost)
subject to  x_{k+1} = f(x_k, u_k)   for all k       (dynamics)
            g(x_k) <= 0              for all k       (inequality constraints: collisions, joint limits)
            x_0 = x_init                             (initial condition)
            x_N = x_goal                             (terminal condition)
```

This transcription turns the problem into a single large, sparse nonlinear program (NLP) that interior-point solvers such as IPOPT handle efficiently. Direct collocation handles dynamics and path constraints simultaneously and yields smooth trajectories, though its non-convexity makes it sensitive to the initial guess and vulnerable to local optima.

### Direct Shooting

**Direct shooting** takes the opposite trade-off: it eliminates the state variables `x_k` from the optimization vector, leaving only the control sequence `{u_0, u_1, ..., u_{N-1}}` as decision variables while computing states through forward simulation. Although this shrinks the number of decision variables, long-horizon rollouts of unstable systems (such as an inverted pendulum) amplify tiny input perturbations exponentially, making gradients poorly conditioned and optimization unstable.

### CHOMP (Covariant Hamiltonian Optimization for Motion Planning)

Proposed by Ratliff et al. (2009), **CHOMP** starts from a naive initial trajectory—typically a straight-line interpolation—and refines it via gradient descent on a combined collision and smoothness objective. Using a covariant inner product propagates local obstacle gradients smoothly along neighboring waypoints rather than kinking individual points. CHOMP excels at smoothing paths in moderately cluttered scenes, though it struggles with narrow passages and can stall in local minima.

### TrajOpt

**TrajOpt** (Schulman et al., 2014) replaces gradient descent with sequential convex optimization (SQP): each iteration builds a linear or quadratic approximation of the costs and constraints around the current trajectory and optimizes within a trust region where that approximation remains valid. Because collision avoidance uses signed-distance costs that remain informative even inside penetration, TrajOpt escapes shallow collisions cleanly—though like all local optimization on non-convex problems, it offers no global optimality guarantee.

### Trajectory Optimization with CasADi

CasADi is a widely used framework that pairs symbolic expression graphs and automatic differentiation with NLP solvers such as IPOPT, sitting alongside Drake, direct solver APIs, and JAX-based implementations in practice. The script below uses CasADi's `Opti` stack to implement direct collocation on a 1D double integrator, minimizing squared control effort over a fixed horizon subject to force and velocity bounds.

```python
import casadi as ca
import numpy as np

# Simple example: minimize squared control effort for a 1D double integrator over a fixed duration
# x = [position, velocity], u = force
# x_dot = [velocity, force/mass]

N = 50           # number of intervals
dt = 0.1         # time step
mass = 1.0

opti = ca.Opti()

# decision variables
X = opti.variable(2, N + 1)  # state trajectory
U = opti.variable(1, N)      # input trajectory

# cost: minimize squared control effort over a fixed duration
cost = 0
for k in range(N):
    cost += U[0, k]**2 * dt  # squared control effort integral
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
> - [CasADi](https://web.casadi.org/) — automatic differentiation and NLP-solver integration.
> - [Drake Trajectory Optimization](https://drake.mit.edu/) — includes direct collocation examples.

---

## 7.6 MoveIt2: Motion Planning in Practice

In real ROS2 systems, **MoveIt2** connects collision geometry, sampling-based planners, and trajectory time-parameterization into a single manipulation stack widely used in research and industrial arm applications.

Its architecture divides the problem into three clear layers:
- **Planning Scene**: maintains the 3D kinematic model of the robot alongside environment obstacles and attached objects, serving as the single source of truth for collision checking.
- **Planning Pipeline**: invokes a planner such as OMPL, validates geometric feasibility, and applies time parameterization to turn waypoints into a timed trajectory.
- **Move Group Interface**: exposes the user-facing API for setting goal constraints, triggering planning, and dispatching execution.

By default, OMPL plugs into the planning pipeline and reads its planner parameters from `ompl_planning.yaml`, though alternative pipelines (such as CHOMP or Pilz industrial motion planners) can also be selected. The configuration snippet below shows how a manipulator group registers RRT-Connect, RRT*, and PRM:

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

In a complete pick-and-place application, perception and planning interleave across seven stages:
1. **Object recognition**: estimate the target object's 6-DoF pose from perception sensors.
2. **Grasp planning**: select a feasible gripper pose and pre-grasp offset around the object.
3. **Approach trajectory**: plan a collision-free motion to the pre-grasp approach point above the object.
4. **Grasp**: close the gripper and attach the object to the planning scene.
5. **Retreat trajectory**: lift the grasped object clear of the support surface.
6. **Place trajectory**: plan motion to the target placement pose while checking collisions for both arm and carried object.
7. **Release**: open the gripper and detach the object.

Across every motion stage, MoveIt2 enforces joint limits and checks self-collisions as well as obstacle clearance automatically.

---

## 7.7 Advanced: Optimization-Based Planning

### Constrained Nonlinear Optimization

Once we move beyond geometric paths to dynamic feasibility, most robot trajectory optimization problems share a single mathematical template—a constrained nonlinear program (NLP):

```
minimize    Σ L(x_k, u_k) + Φ(x_N)
subject to  x_{k+1} = f(x_k, u_k)           (dynamics)
            h(x_k, u_k) = 0                  (equality constraints)
            g(x_k, u_k) <= 0                 (inequality constraints: collisions, torque limits, etc.)
```

Here `L` captures running stage costs (such as control effort or tracking error) and `Φ` scores the terminal state, while equality and inequality constraints enforce dynamics, actuator bounds, and obstacle avoidance. IPOPT (Interior Point Optimizer) is the workhorse solver for problems of this form, and CasADi wires into IPOPT by default.

### Contact-Implicit Trajectory Optimization

Standard trajectory optimization assumes the contact sequence—which foot or finger touches which surface, and at what exact time step—is fixed in advance. For tasks with rich, unpredictable contact transitions such as legged locomotion over uneven terrain or dexterous in-hand manipulation, hardcoding the contact schedule is brittle. **Contact-implicit trajectory optimization** lets the solver discover when and where contacts form and break on its own.

To give the optimizer that freedom, we include normal contact forces `F_n` alongside the state and input variables and couple them to the signed separation distance `d` through **complementarity constraints**:

```
F_n >= 0                   (contact force cannot pull)
d >= 0                     (object cannot go below the floor)
F_n * d = 0                (zero force when separated, zero distance when in contact)
```

Physically, these three lines encode rigid unilateral contact: surfaces only push (`F_n >= 0`), bodies never interpenetrate (`d >= 0`), and at least one of force or gap must vanish (`F_n * d = 0`). Mathematically, that bilinear product makes the problem an **MPCC** (Mathematical Program with Complementarity Constraints), which violates standard constraint qualifications and stalls naive NLP solvers. In practice, solvers rely on relaxation schemes (replacing `F_n * d = 0` with `F_n * d <= eps` and annealing `eps`) or smoothed compliant contact models. Drake offers no single pre-wrapped class for this method out of the box; instead, practitioners construct it by adding contact complementarity constraints directly to a `MathematicalProgram`.

### Connection to Real-Time Re-planning and MPC

In a static workcell, planning a full trajectory offline once is enough. When humans, moving targets, or modeling errors disturb the scene, however, the robot must continuously re-plan online—which is precisely where trajectory optimization and Ch.6's Model Predictive Control (MPC) converge.

MPC operates as short-horizon trajectory optimization inside a feedback loop: at every control cycle, the controller solves a trajectory problem over a brief lookahead window, executes only the first control input, measures the actual resulting state, and re-solves from there. Thus, offline motion-planning trajectory optimization computes an entire trajectory in one pass before execution, whereas online MPC repeatedly re-solves a truncated horizon in real time.

---

## 7.8 Advanced: Task and Motion Planning (TAMP)

Motion planning assumes someone has already chosen *what* to move, *where* to grasp it, and *in what order*. Real manipulation tasks—even something as ordinary as "place the cup on the shelf"—intertwine discrete task choices with continuous geometry. Executing that instruction breaks down into four coupled steps:

1. Recognize where the cup sits in the scene.
2. Choose a continuous grasp pose that can lift the cup without colliding with the table or shelf.
3. Sequence the discrete high-level actions: approach → grasp → lift → move → place.
4. Plan a collision-free joint trajectory for each continuous motion stage.

Step 1 belongs to perception, step 2 is a continuous geometric choice (which grasp pose to use), step 3 is **symbolic planning** (which actions to execute, and in what order), and step 4 is **motion planning** (how the joints move during each step). **Task and Motion Planning (TAMP)** couples the symbolic and geometric layers so an infeasible grasp forces the symbolic planner to pick a different action sequence (for example, moving an obstructing mug aside first). Frameworks such as PDDLStream factor out continuous parameter choices like step 2 into modular geometric samplers.

### PDDLStream

Developed at MIT, **PDDLStream** extends classical symbolic planning in PDDL (Planning Domain Definition Language) by attaching declarative **streams**—conditional generators that sample continuous parameters such as valid grasp poses, placement poses, and collision-free kinematic trajectories on demand while the symbolic search looks for a feasible plan skeleton.

### LLM-Based Task Planning

Large language models have spurred active research aimed at replacing hand-authored PDDL domains with open-vocabulary reasoning:

- **SayCan** (Google, 2022): pairs an LLM that scores candidate natural-language skills by task relevance with a learned affordance value function that scores whether each skill is physically executable in the current scene; their product selects the next action.
- **Code as Policies** (Google, 2023): prompts an LLM to synthesize executable Python control code directly from natural-language instructions, bridging high-level logic and perception/motion primitives.
- **Inner Monologue** (Google, 2022): closes the loop by feeding scene descriptions, success detectors, and failure messages back to the LLM so it replans iteratively when actions fail.

Despite rapid progress, LLM-based TAMP remains largely experimental for tight physical constraints. Language models struggle with fine millimeter clearances, cluttered collision geometry, and precision assembly, so reliable systems still pair LLMs for high-level semantic sequencing with classical geometric motion planners at the execution layer.

Both classical TAMP and these LLM pipelines assume deterministic dynamics and fully known state. Once both motion and sensing become stochastic, planning must move into belief space, as covered in §7.9 Advanced: POMDP.

---

## 7.9 Advanced: Decision-Making Under Uncertainty (POMDP and Belief Space Planning)

Everything in §7.1–§7.8 assumed the robot knows its exact state and the environment geometry. Real robots never enjoy that certainty: wheel encoders drift, cameras lose texture, and sensors return noisy, partial measurements. When a robot cannot tell which end of a symmetric corridor it occupies, or whether a door ahead is latched or ajar, planning solely from its "most likely state" fails—it may drive straight into a hazard it thought was elsewhere. Instead, the planner must reason directly over the **belief** (the posterior probability distribution over states). Thrun, Burgard, and Fox's *Probabilistic Robotics* §15.2 and §16 develop this framework in detail. (Section, table, equation, and figure numbers cited below follow the 1999–2000 early draft; in the 2005 MIT Press edition, MDPs appear in Chapter 14, POMDPs in Chapter 15, and approximate solvers such as QMDP, AMDP, and MC-POMDP in Chapter 16.)

### 7.9.1 Introduction: Three Paradigms

Consider a single left-right symmetric corridor containing a Goal at one end, a Pit at the other, and a Robot that starts near the center. Depending on what uncertainty they model, three planning paradigms prescribe completely different trajectories:

- **Classical planning** assumes full state observability and deterministic transitions. A* from §7.3 belongs here: it computes one shortest path offline and expects no surprises during execution.
- **MDP (Markov Decision Process)** keeps full state observability while allowing stochastic actions. Its policy $\pi: s \to a$ maps every concrete state to an action, so near a narrow passage or pit an MDP planner steers wider to hedge against motion noise. Ch.8 §8.2 builds directly on this formulation.
- **POMDP (Partially Observable MDP)** drops both assumptions: actions *and* observations are stochastic, so the true state is hidden behind noisy measurements. Its policy $\pi: b \to a$ therefore maps a belief $b$ to an action. In the symmetric corridor, the robot initially holds a bimodal belief split across both sides, so instead of gambling on a straight dash toward the goal, it deliberately detours into an asymmetric landmark region to collapse its uncertainty before committing—a hallmark behavior called **active information gathering**.

Notice the strict nesting: classical $\subset$ MDP $\subset$ POMDP. Robot uncertainty splits along two axes—**action uncertainty** (where the robot meant to go versus where it landed) and **perceptual uncertainty** (where the robot actually is versus what its sensors report). MDPs address only the first; POMDPs address both. Whereas Ch.3's Bayes filters *tracked* the belief distribution from sensor streams, this section determines what actions to *choose* with that tracked belief.

<!-- DEMO: pomdp_three_paradigms.html -->

### 7.9.2 Value Iteration over Belief

Comparing Bellman backups side by side reveals immediately why POMDPs are so much harder than MDPs. In an MDP, value iteration computes the finite-horizon value over states $s$:

$$C^T(s) = \max_a \int \left[ c(s') + C^{T-1}(s') \right] P(s' \mid a, s)\, ds'$$

To handle partial observability, we elevate the state $s$ to the belief $b$, yielding the POMDP value backup:

$$C^T(b) = \max_a \int \left[ c(b') + C^{T-1}(b') \right] P(b' \mid a, b)\, db' \tag{16.2}$$

with the corresponding optimal policy:

$$\pi^T(b) = \arg\max_a \int \left[ c(b') + C^{T-1}(b') \right] P(b' \mid a, b)\, db' \tag{16.3}$$

Here both $b$ and $b'$ are probability distributions over the state space $\mathcal{S}$, and the integral runs over the entire space of beliefs under the belief transition kernel $P(b' \mid a,b)$. Even for a finite state space, that belief simplex has dimension $|\mathcal{S}|-1$; for a continuous state space, belief space is infinite-dimensional, making a direct integral over $db'$ intractable.

In the infinite-horizon limit, when the recursion converges, it satisfies the belief-space Bellman equation:

$$V(b) = \max_a \left[ r(b, a) + \gamma \sum_{o'} P(o' \mid b, a)\, V(B(b, a, o')) \right]$$

where $r(b,a) = \sum_s b(s)\, c(s,a)$ is the expected immediate reward under belief $b$, and eq. (16.2) expresses its finite-horizon counterpart.

A key algebraic observation makes this formulation workable. Once the robot takes action $a$ and receives observation $o'$, the resulting posterior belief $B(b, a, o')$ is *uniquely* determined by the Bayes filter—no extra randomness remains in the belief transition itself. Consequently, the daunting integral over all possible next beliefs $b'$ collapses into an integral (or sum) over possible observations $o'$:

$$C^T(b) = \max_a \int \left[ c(B(b, a, o')) + C^{T-1}(B(b, a, o')) \right] P(o' \mid a, b)\, do' \tag{16.34}$$

where the Bayes filter belief update operator is:

$$B(b, a, o')(s') = \frac{1}{P(o' \mid a, b)}\, P(o' \mid s') \int P(s' \mid a, s)\, b(s)\, ds$$

In discrete state and observation spaces these integrals reduce to finite sums, and this observation-space reformulation underpins nearly every modern POMDP solver.

### 7.9.3 Four-State Toy Example

Before proving the general geometry of POMDP value functions, it helps to trace a tiny 1D belief problem by hand so the **piecewise-linear convex (PWLC)** structure appears concretely. Consider a problem with 4 states, 2 actions, and 2 observations:

**Setup:**
- States $s_1, s_2, s_3, s_4$, with the robot starting in either $s_1$ or $s_2$.
- Action $a_1$ (information gathering): swaps $s_1 \leftrightarrow s_2$ with probability 0.9 while yielding an informative observation.
- Action $a_2$ (termination): moves to terminal state $s_3$ (reward +80) from $s_1$ or $s_4$ (reward −80) from $s_2$.
- Observations $o_1, o_2$: emitted with probabilities $(0.7, 0.3)$ from $s_1$ and $(0.4, 0.6)$ from $s_2$.
- Belief $b = (p_1, p_2)$ with $p_1 + p_2 = 1$, leaving a single free scalar parameter $p_1 \in [0, 1]$.

**Horizon 1 computation:**

Because expected immediate reward weights state rewards by belief, $c(b) = \sum_i c(s_i) p_i$ is linear in $b$. Choosing $a_2$ immediately with discount $\gamma = 0.9$ yields:
$$C^1(b, a_2) = \gamma(80 p_1 - 80 p_2) = 72 p_1 - 72 p_2$$

Choosing $a_1$ does not terminate, leaving only its near-zero immediate reward: $C^1(b, a_1) \approx 0$. Maximizing over both actions gives the horizon-1 value function:
$$C^1(b) = \max\{ 0,\; 72p_1 - 72p_2 \}$$

Visually, $C^1(b)$ is the upper envelope of two straight lines, bending sharply at $p_1 = 0.5$: when $p_1 > 0.5$ the robot gambles on $a_2$, whereas below 0.5 it prefers $a_1$.

**Horizon 2 computation:**

With two steps remaining, taking $a_1$ first lets the robot observe $o_1$ or $o_2$, update its belief via Bayes' rule, and then act optimally on the second step. Summing the expected values over both observations gives:
$$C^2(b, a_1) = \max\{0,\; -18.79 p_1 + 38.23 p_2\} + \max\{0,\; -33.05 p_1 + 13.61 p_2\} = \max\{0,\; -18.79 p_1 + 38.23 p_2,\; -51.84 p_1 + 51.84 p_2\}$$

(Coefficients follow directly from substituting the transition/observation probabilities into the belief update; among the four linear combinations formed by pairing per-observation choices, $-33.05 p_1 + 13.61 p_2$ lies strictly below $-18.79 p_1 + 38.23 p_2$ for all beliefs and is pruned.)

Combining $a_1$ with the immediate termination line for $a_2$ (which dominates the constant 0 piece) yields the full horizon-2 value function:
$$C^2(b) = \max\{ -51.84 p_1 + 51.84 p_2,\; -18.79 p_1 + 38.23 p_2,\; 72 p_1 - 72 p_2 \}$$

Now three linear segments form the upper envelope, and each additional horizon step adds more segments. Crucially, the resulting value function is always **convex** over belief space: $\beta C(b) + (1-\beta) C(b') \geq C(\beta b + (1-\beta) b')$. Intuitively, uncertainty is costly—a blended belief forces the robot to choose one action for an ambiguous mixture of states, earning less than the weighted average of acting with certainty in each state separately.

<!-- DEMO: pomdp_toy_pwlc.html -->

### 7.9.4 PWLC Structure and Alpha-Vectors

The upper-envelope geometry we just saw is not an accident of four states; an inductive argument proves that finite-state POMDP value functions are always **piecewise-linear convex (PWLC)**.

In the base case ($T=1$), expected immediate reward $c(b) = \sum_i c(s_i) p_i$ is linear in belief, so $C^1(b) = \max_a \sum_i C^1_{a,i}\, p_i$ is the maximum over one linear function per action. For the inductive step, assume $C^{T-1}(b)$ is PWLC and substitute it into eq. (16.34). When we expand $C^{T-1}(B(b,a,o'))$, the nonlinear normalization denominator $1/P(o'\mid a, b)$ inside the Bayes update cancels cleanly against the weighting factor $P(o'\mid a, b)$ in eq. (16.34), leaving each term $\langle \phi, B(b,a,o') \rangle \cdot P(o'\mid a, b)$ strictly linear in $b$. Because the maximum of sums of maxima of linear functions remains a maximum of linear functions, $C^T(b)$ stays PWLC for every finite horizon $T$.

Each linear segment is parameterized by a coefficient vector called an **alpha-vector** $\phi$, allowing the value function to be written compactly as the upper envelope over a finite set $\Phi$ of alpha-vectors:

$$V(b) = \max_\phi \langle \phi, b \rangle$$

or explicitly $V(b) = \max_{\phi \in \Phi} \sum_i \phi_i\, p_i$.

Each alpha-vector represents the state-by-state value of committing to one specific *conditional policy*—a root action followed by observation-contingent future plans. That combinatorial branching is where exact planning explodes: before pruning, the number of candidate alpha-vectors grows doubly exponentially as $|\Phi^T| = |A| \cdot |\Phi^{T-1}|^{|\mathcal{O}|}$. Even in our minimal example with two actions, two observations, and $|\Phi^0| = 1$, the count jumps from $|\Phi^1| = 2$ and $|\Phi^2| = 2 \cdot 2^2 = 8$ to $|\Phi^3| = 2 \cdot 8^2 = 128$ and $|\Phi^4| = 2 \cdot 128^2 = 32768$, rendering exact enumeration impractical for realistic horizons.

### 7.9.5 LP Solution

To tame that doubly exponential explosion as much as possible in an exact solver, we can cast the nested max–sum–max backup as a linear program (LP) that detects and prunes dominated alpha-vectors before they multiply across subsequent steps.

The **reduction principle** converts any maximum into linear inequalities: $C = \max_a x(a)$ becomes $\min C$ subject to $\{C \geq x(a) \;\forall a\}$. Extending that trick to $C = \sum_i \max_a x(a,i)$ requires introducing a selector $a(\cdot)$ that assigns an action to every state $i$ and imposing $\{C \geq \sum_i x(a(i),i)\}$ over all $|A|^{|\mathcal{S}|}$ combinations.

Applying this reduction to the POMDP horizon-$T$ backup yields the exact LP constraint set (eq. 16.67):

$$\bigcup_a \bigcup_{k(o'):1 \leq k(o') \leq |\Phi^{T-1}|} \left\{ C^T(b) \geq \gamma \sum_{o'} \sum_i \left(c_i + C^{T-1}_{k(o'),i}\right) P(o' \mid s_i') \sum_j P(s_i' \mid a, s_j)\, p_j \right\}$$

Before pruning, the constraint count still scales as $|\Phi^T| = |A| \cdot |\Phi^{T-1}|^{|\mathcal{O}|}$. The resulting exact algorithm builds these conditional vectors layer by layer and strips away dominated ones:

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

Even with LP pruning, $|\Phi^T|$ grows doubly exponentially. With just a horizon of 3, 3 actions, and 5 observations, the pre-pruning alpha-vector count already reaches roughly $10^{14}$, far exceeding what any realistic domain can tolerate. Exact POMDP solvers therefore serve mainly as a theoretical benchmark, motivating the approximate methods below.

### 7.9.6 General POMDP

If exact LP solutions already choke on small finite-state problems, moving to continuous state spaces—such as 2D or 3D robot poses—pushes exact backups completely out of reach. In continuous space, each alpha-vector becomes a continuous function rather than a finite array; eq. (16.34) still holds formally, yet $\Phi^{T-1}$ turns into an infinite-dimensional set of functions:

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

Because storing, integrating, and pruning arbitrary sets of continuous functions is intractable in practice, this algorithm remains an idealized blueprint. Practical continuous-space solvers replace exact function sets with sample-based or statistical approximations—most notably MC-POMDP and AMDP.

### 7.9.7 MC-POMDP

Once exact belief representations hit a wall, the natural next step is Monte Carlo approximation: represent the belief with a cloud of weighted particles and estimate value backups from simulated rollouts. Whereas Ch.3 §3.11 used the particle filter purely for state estimation, **MC-POMDP** moves that same particle filter inside the planning loop.

Here a belief $\theta$ is stored as a set of weighted particles $\langle s^{(i)}, w^{(i)} \rangle$. When the robot takes action $a$ and observes $o'$, the Bayes filter update $B(b, a, o')$ operates directly on the particle cloud:

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

Instead of integrating over all possible observations and states analytically, MC-POMDP learns an action-value function $Q(\theta, a)$ across a database of sampled beliefs. For each stored belief $\theta$ and action $a$, it simulates $N$ state-observation rollouts, propagates the particle filter to the next belief $\theta'$, and averages the resulting returns:

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

This sample-based backup implements the Bellman Q-function update (eq. 16.78):

$$Q(\theta_t, a_t) \leftarrow \mathbb{E}\left[ R(o_{t+1}) + \gamma \max_{\bar{a}} Q(\theta_{t+1}, \bar{a}) \right]$$

from which the greedy policy follows directly (eq. 16.79):

$$\sigma^Q(\theta) = \arg\max_{\bar{a}} Q(\theta, \bar{a})$$

A subtle implementation detail is how to generalize $Q(\theta, a)$ to unseen beliefs $\theta'$. Because $\theta$ is an unordered particle set, it could not be fed directly into fixed-order feedforward neural networks of that era. Thrun et al. instead maintained a database of $\langle \theta, a, Q \rangle$ tuples and evaluated unseen beliefs via $k$-nearest neighbors under KL divergence approximated with Gaussian kernel density estimation (KDE). Modern architectures can substitute permutation-invariant set encoders and neural value networks, yet the underlying algorithmic skeleton remains the same.

To populate the belief database efficiently, the outer loop can either hold a fixed dataset or generate beliefs online through $\varepsilon$-greedy simulation trials—focusing computation on the narrow subset of beliefs the robot actually encounters in motion.

### 7.9.8 Experiments: Heaven/Hell and Find-and-Fetch

Two classic experiments in *Probabilistic Robotics* illustrate how MC-POMDP discovers active information gathering automatically without handcrafted heuristics:

- **Heaven/Hell problem**: In a T-shaped corridor, one end holds heaven (+1) and the other holds hell (−1), and only a priest stationed near the entrance knows which is which. Acting greedily on the initial ambiguous belief sends the robot into hell half the time, whereas MC-POMDP learns a detour that first visits the priest to gather a reliable observation before committing to the goal.
- **Find-and-Fetch (monocular camera)**: Tasked with retrieving a target using only a monocular camera—which measures bearing to the object but not range—MC-POMDP learns to translate sideways across multiple viewpoints, shrinking depth uncertainty through triangulation before moving in to grasp.

In both domains, tracking the full belief distribution inside the planner produces deliberate sensing detours that decoupled "estimate-then-act-greedily" pipelines never discover.

### 7.9.9 AMDP — Dimensionality Reduction via Belief Statistics

MC-POMDP retains the full particle cloud, which keeps belief representation faithful at the cost of sample-based Q lookup. **Augmented MDP (AMDP)** takes an even simpler route between the two extremes of tractable MDPs (polynomial in $|S|$) and exact POMDPs (doubly exponential): along real robot trajectories, beliefs rarely wander across the entire infinite-dimensional belief space, instead concentrating along a low-dimensional manifold.

AMDP compresses each belief $b$ into a compact summary vector $\bar{b} = f(b)$ and runs ordinary MDP value iteration directly over $\bar{b}$. The standard summary pairs the robot's most likely state with the belief's differential entropy (eq. 16.80):

$$\bar{b} = \langle \arg\max_s b(s),\; H[b] \rangle$$

where the entropy

$$H[b] = -\int b(s) \ln b(s)\, ds \tag{16.81}$$

quantifies uncertainty ("how lost I am"). Keeping only $\arg\max_s b(s)$ reduces the problem back to an ordinary MDP that ignores localization risk; appending $H[b]$ explicitly informs the planner when its state estimate is deteriorating. Although this two-number summary is not guaranteed to be a *sufficient statistic*—Thrun et al. explicitly caution that "the sufficient statistic assumption rarely holds"—empirical navigation results confirm that it captures enough uncertainty structure to drive sensible decisions.

Replacing $s$ with $\bar{b}$ turns belief planning back into standard value iteration:

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

Its structure mirrors `MDP_value_iteration` (*Probabilistic Robotics* §15.3.3), substituting $\bar{b}$ for $s$. The induced transition model $P(\bar{b}' \mid a, \bar{b})$ integrates over all underlying beliefs mapping to $\bar{b}$ and resulting updated beliefs mapping to $\bar{b}'$ (eq. 16.85):

$$P(\bar{b}' \mid a, \bar{b}) = \int\!\!\int\!\!\int I_{f(b)=\bar{b}}\, I_{f(B(o',a,b))=\bar{b}'}\, P(o' \mid s') P(s' \mid a, s) P(s \mid b)\, ds\, ds'\, do'\, db$$

Because evaluating that integral analytically is prohibitive, practical implementations estimate $P(\bar{b}' \mid a, \bar{b})$ empirically via Monte Carlo simulation and cache the transitions in a lookup table.

### 7.9.10 Coastal Navigation Example

**Coastal navigation** provides the clearest demonstration of how AMDP reshapes robot paths once uncertainty enters the state. Consider crossing a wide open room: a conventional MDP planner takes the shortest straight line straight across the center, where a short-range LiDAR or camera sees only blank, distant walls, causing position uncertainty (entropy) to balloon until the robot arrives near the goal without knowing where it is.

In the exact same room, an AMDP planner deliberately chooses a **curved path hugging the walls**. Near walls and corners, sensor measurements anchor the pose estimate and keep entropy low; because entropy enters the augmented state and cost, the planner favors information-rich corridors automatically—just as a pre-GPS ship hugs the coastline to stay oriented by visible landmarks. As shown in Thrun et al.'s Figure 16.5, shrinking sensor range drives the arrival entropy of a conventional planner sharply upward, whereas the coastal AMDP planner keeps arrival entropy nearly flat. The same principle drives modern Active SLAM—which penalizes pose covariance along candidate trajectories—and next-best-view planning.

<!-- DEMO: coastal_navigation_amdp.html -->

### 7.9.11 What Survived

Once uncertainty enters the planner's objective, coastal navigation and active information gathering follow naturally from the belief-space value iteration introduced in §7.9.2. Although exact solvers (§7.9.5 and §7.9.6) remain computationally intractable, modern POMDP planning has flourished across three practical families:

- **Point-based value iteration (SARSOP, HSVI, PBVI)**: preserves the alpha-vector PWLC representation from §7.9.4 while restricting backups to a reachable sample of belief points, preventing combinatorial explosion.
- **MCTS family (POMCP, DESPOT)**: pairs Monte Carlo rollouts—echoing §7.9.7's MC-POMDP Q estimation—with online belief-tree search (Silver & Veness, 2010).
- **Deep POMDP (DRQN, DVRL)**: replaces MC-POMDP's kNN belief lookup with recurrent or variational neural encoders (Igl et al.) whose hidden states serve as learned implicit beliefs.

The same belief-augmented principle appears in Bayes-adaptive MDPs (BAMDP), which maintain a posterior over unknown model parameters as part of the state, as well as Active SLAM and NeRF-based active perception that route trajectories toward high-information viewpoints.

Notice the boundary separating these planners from Ch.8 §8.3: POMDP planning computes policies *given* known transition and observation models—without a model, even MC-POMDP cannot simulate belief rollouts—whereas deep RL algorithms such as PPO and SAC learn policies purely from experience without an explicit dynamics model. MC-POMDP bridges the two worlds by tracking beliefs with a known model while estimating Q-values from sampled experience.

Across exact value iteration, particle-based MC-POMDP, and entropy-compressed AMDP, the core goal remains identical: balancing task progress against active information gathering whenever sensors see only part of the truth.

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
1998 ── RRT (LaValle) — an influential single-query sampling planner
2000 ── RRT-Connect (Kuffner & LaValle) — a variant widely provided by practical motion-planning libraries
2009 ── CHOMP (Ratliff et al.) — gradient-based trajectory optimization
2011 ── RRT* (Karaman & Frazzoli) — asymptotic optimality guarantee
2012 ── OMPL introductory paper (IEEE RAM) — unified library of sampling-based planners (1.0 release in 2014)
2014 ── TrajOpt (Schulman et al.) — sequential convex optimization
2019 ── MoveIt2 (ROS2) — an open motion-planning framework used in research and industry
2022 ── SayCan (Google) — LLM + motion planning
2023 ── Contact-implicit trajectory optimization becomes practical
2024 ── LLM-based TAMP research spreads
```

---

Everything in this chapter relied on an explicit model of geometry, dynamics, or sensor likelihoods. When contact transitions, deformable objects, or unstructured homes resist clean analytical modeling, robots must instead acquire behavior from interaction and demonstration—the domain of robot learning in Ch.8.
