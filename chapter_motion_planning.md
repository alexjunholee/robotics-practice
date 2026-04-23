## 8. 모션 플래닝 & 궤적 최적화 (Motion Planning & Trajectory Optimization)

로봇이 A에서 B로 가려면 경로가 필요하다. 단순히 직선으로 가면 되지 않냐고 생각할 수 있지만, 장애물이 있고, 관절 한계가 있고, 동역학 제약이 있다. 이 모든 조건을 만족하는 경로를 찾는 것이 모션 플래닝이고, 그 경로를 시간에 따라 최적으로 추종하는 것이 궤적 최적화다.

---

### 8.1 왜 모션 플래닝을 배우는가

6축 로봇 팔에게 "저 컵을 집어라"라고 명령했다고 하자. IK로 목표 관절 각도를 구했다. 그런데 현재 자세에서 목표 자세로 관절을 직선으로 보간(interpolation)하면, 팔이 테이블을 관통하거나 자기 몸체에 충돌할 수 있다. 관절 공간에서의 직선이 작업 공간에서의 직선이 아니기 때문이다.

모션 플래닝은 다음 질문에 답한다:
- 충돌 없이 목표에 도달하는 경로가 존재하는가?
- 존재한다면, 가장 짧은/빠른/부드러운 경로는 무엇인가?
- 동역학 제약(토크 한계, 속도 한계)을 만족하면서 그 경로를 따라갈 수 있는가?

---

### 8.2 Configuration Space (C-space)

모션 플래닝의 핵심 개념이다. 로봇의 모든 가능한 상태를 하나의 공간으로 표현한다.

**Joint space = Configuration space**: n-DOF 로봇의 configuration은 q = (q1, q2, ..., qn)이다. 이 q가 살고 있는 n차원 공간이 C-space이다.

**C-space obstacle**: 작업 공간(3D)의 장애물을 C-space로 변환한 것이다. C-space에서 장애물 영역에 속하는 configuration은 충돌 상태이다.

왜 C-space에서 생각해야 하는가: 로봇은 점이 아니다. 3D 공간에서 로봇의 모든 링크가 장애물과 충돌하지 않는지 확인하려면, 각 configuration에서 FK를 계산하고 충돌 검사를 해야 한다. C-space에서는 로봇을 "점"으로 취급할 수 있고, 장애물을 피하는 문제가 점의 경로 찾기 문제로 환원된다.

문제는 C-space obstacle의 정확한 형태를 계산하는 것이 일반적으로 어렵다는 것이다. 그래서 실무에서는 C-space obstacle을 명시적으로 구하지 않고, 특정 configuration에서의 충돌 여부를 검사하는 collision checker를 사용한다.

---

### 8.3 그래프 탐색 기반 플래닝

가장 고전적인 접근이다. C-space를 이산화(discretize)하고 그래프 탐색 알고리즘으로 경로를 찾는다.

#### Dijkstra 알고리즘

가중 그래프에서 최단 경로를 찾는다. 모든 간선을 탐색하므로 최적해를 보장한다. 시간 복잡도 O((V + E) log V).

#### A* 알고리즘

Dijkstra에 휴리스틱을 추가한 것이다. 목표까지의 추정 거리(heuristic)를 이용하여 탐색 방향을 유도한다. 휴리스틱이 admissible(실제 거리 이하)이면 최적해를 보장하면서 Dijkstra보다 빠르다.

```python
import heapq
import numpy as np

def astar_2d(grid, start, goal):
    """2D 격자에서의 A* 경로 탐색.
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
            # 경로 복원
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

    return None  # 경로 없음
```

#### 장단점

격자 기반 플래닝은 **완전성(completeness)**을 보장한다 — 해가 존재하면 반드시 찾는다. 하지만 **차원의 저주(curse of dimensionality)**에 시달린다. 6-DOF 로봇 팔의 C-space를 각 축 100개로 이산화하면 100^6 = 10^12개의 셀이 된다. 사실상 불가능하다.

이 문제를 해결하기 위해 샘플링 기반 플래너가 등장했다.

---

### 8.4 샘플링 기반 플래너

C-space를 이산화하지 않고, 무작위로 샘플링하여 경로를 탐색한다. 고차원 C-space에서 실용적인 유일한 방법이다.

#### RRT (Rapidly-exploring Random Tree)

LaValle (1998)이 제안한 알고리즘이다. 핵심 아이디어는 단순하다:

```
1. 시작점에서 트리를 초기화한다.
2. C-space에서 무작위 점 q_rand를 샘플링한다.
3. 트리에서 q_rand에 가장 가까운 노드 q_near를 찾는다.
4. q_near에서 q_rand 방향으로 step_size만큼 확장하여 q_new를 만든다.
5. q_near → q_new 경로가 충돌하지 않으면 트리에 추가한다.
6. q_new가 목표 근처면 종료. 아니면 2로 돌아간다.
```

```python
import numpy as np

class RRT:
    def __init__(self, start, goal, obstacle_fn, bounds, step_size=0.3, max_iter=5000):
        self.start = np.array(start)
        self.goal = np.array(goal)
        self.obstacle_fn = obstacle_fn  # config -> bool (True if collision)
        self.bounds = np.array(bounds)  # [[min_q1, max_q1], ...]
        self.step_size = step_size
        self.max_iter = max_iter
        self.nodes = [self.start]
        self.parents = {0: -1}

    def sample_random(self):
        # 10% 확률로 goal을 샘플링 (goal bias)
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
                    # 경로 복원
                    path = [q_new]
                    idx = idx_new
                    while self.parents[idx] != -1:
                        idx = self.parents[idx]
                        path.append(self.nodes[idx])
                    return path[::-1]

        return None  # 실패
```

#### RRT* (Optimal RRT)

Karaman & Frazzoli (2011)이 제안. RRT는 해를 찾지만 최적이 아니다. RRT*는 새 노드 추가 시 근처 노드들과의 re-wiring을 수행하여 점근적 최적성(asymptotic optimality)을 보장한다. 샘플 수가 무한대로 가면 최적 경로에 수렴한다.

실무적으로 RRT*는 RRT보다 좋은 경로를 찾지만, 수렴이 느리다. 시간 제한이 있는 실시간 상황에서는 RRT-Connect가 더 실용적인 경우가 많다.

#### PRM (Probabilistic Roadmap)

Kavraki et al. (1996)이 제안. RRT가 single-query(한 번에 하나의 start-goal 쌍)인 반면, PRM은 multi-query에 적합하다.

1단계 (offline): C-space에 많은 점을 샘플링하고, 가까운 점들을 충돌 없는 간선으로 연결하여 로드맵(graph)을 구축한다.
2단계 (online): start와 goal을 로드맵에 연결하고, 그래프 탐색(A* 등)으로 경로를 찾는다.

같은 환경에서 여러 경로 쿼리가 필요한 경우(예: 산업용 로봇 셀) PRM이 효율적이다.

#### RRT-Connect

Kuffner & LaValle (2000)이 제안. 시작점과 목표점에서 동시에 트리를 성장시키고, 두 트리가 만나면 경로를 연결한다. 실무에서 가장 많이 쓰이는 변종이다. MoveIt2의 기본 플래너도 RRT-Connect(OMPL 내장)이다.

#### OMPL 라이브러리

Open Motion Planning Library (https://ompl.kavrakilab.org/). Kavraki Lab (Rice University)에서 개발한 C++ 라이브러리로, RRT, RRT*, RRT-Connect, PRM, EST, KPIECE 등 수십 가지 샘플링 기반 플래너를 제공한다.

OMPL 자체는 충돌 검사를 하지 않는다. State validity checker를 사용자가 제공해야 한다. MoveIt2는 OMPL + FCL(Flexible Collision Library)를 결합하여 완전한 모션 플래닝 파이프라인을 구성한다.

```python
# MoveIt2에서 OMPL 기반 모션 플래닝 (ROS2 Python API, 간략화)
from moveit_py import MoveItPy

moveit = MoveItPy(node_name="motion_planner")
arm = moveit.get_planning_component("manipulator")

# 목표 설정
arm.set_goal_state(configuration_name="home")

# 플래닝 (OMPL RRT-Connect가 기본)
plan_result = arm.plan()

if plan_result:
    # 실행
    arm.execute()
```

> **추천 자료**
> - [LaValle, "Planning Algorithms"](http://lavalle.pl/planning/) — 무료 온라인 교재. 모션 플래닝의 바이블
> - [OMPL](https://ompl.kavrakilab.org/) — 오픈소스 모션 플래닝 라이브러리
> - [MoveIt2 Tutorials](https://moveit.picknik.ai/) — ROS2 기반 실전 모션 플래닝 가이드

---

### 8.5 궤적 최적화 (Trajectory Optimization)

샘플링 기반 플래너는 "충돌 없는 경로"를 찾아준다. 하지만 그 경로는:
- 울퉁불퉁하다 (random sampling이므로)
- 동역학을 무시한다 (기구학적 경로만 제공)
- 시간 정보가 없다 (어떤 속도로 따라가야 하는지 모른다)

궤적 최적화는 이 한계를 보완한다. 비용 함수(시간, 에너지, 부드러움)를 최소화하면서, 동역학 제약, 충돌 회피, 관절 한계를 모두 만족하는 궤적을 찾는다.

#### Direct Collocation

궤적을 시간 구간으로 나누고, 각 구간의 상태와 입력을 결정 변수(decision variable)로 둔다. 동역학 방정식은 등식 제약(equality constraint)으로 처리한다.

```
minimize    Σ_k L(x_k, u_k) * dt                    (비용)
subject to  x_{k+1} = f(x_k, u_k)   for all k       (동역학)
            g(x_k) <= 0              for all k       (부등식 제약: 충돌, 관절 한계)
            x_0 = x_init                             (초기 조건)
            x_N = x_goal                             (종단 조건)
```

이것을 하나의 큰 nonlinear program(NLP)으로 만들고, IPOPT 같은 솔버로 푼다.

장점: 동역학과 제약을 동시에 처리, 부드러운 궤적
단점: 초기 추측(initial guess)에 민감, 비볼록이므로 지역 최적해에 빠질 수 있음

#### Direct Shooting

상태를 결정 변수에서 제거하고, 입력 시퀀스 {u_0, u_1, ..., u_{N-1}}만을 결정 변수로 둔다. 상태는 동역학 시뮬레이션으로 계산한다.

collocation보다 결정 변수가 적지만, 시뮬레이션이 불안정하면 (예: 도립진자) 최적화도 불안정해진다.

#### CHOMP (Covariant Hamiltonian Optimization for Motion Planning)

Ratliff et al. (2009)이 제안. 초기 궤적(보통 직선 보간)에서 시작하여, 충돌 비용 + 부드러움 비용의 gradient를 따라 궤적을 반복적으로 개선한다. 공변 gradient(covariant gradient)를 사용하여 업데이트가 부드럽다.

장점: 직관적, 기존 궤적을 점진적으로 개선
단점: 좁은 통로(narrow passage)를 통과하기 어려움, 지역 최적해

#### TrajOpt

Schulman et al. (2014)이 제안. Sequential convex optimization 기반이다. 매 반복에서 비선형 문제를 선형/이차 근사로 바꿔서 QP로 풀고, trust region으로 수렴을 보장한다. 충돌 회피를 signed distance function으로 처리하여 연속적인 gradient를 제공한다.

#### CasADi를 이용한 Trajectory Optimization

CasADi는 symbolic computation + automatic differentiation + NLP solver 연결을 제공하는 프레임워크다. Trajectory optimization의 사실상 표준 도구이다.

```python
import casadi as ca
import numpy as np

# 간단한 예: 1D 더블 인티그레이터의 시간 최적 궤적
# x = [position, velocity], u = force
# x_dot = [velocity, force/mass]

N = 50           # 구간 수
dt = 0.1         # 시간 간격
mass = 1.0

opti = ca.Opti()

# 결정 변수
X = opti.variable(2, N + 1)  # 상태 궤적
U = opti.variable(1, N)      # 입력 궤적

# 비용: 에너지 최소화 + 시간 패널티
cost = 0
for k in range(N):
    cost += U[0, k]**2 * dt  # 에너지
opti.minimize(cost)

# 동역학 제약 (Euler integration)
for k in range(N):
    x_next = X[:, k] + ca.vertcat(X[1, k], U[0, k] / mass) * dt
    opti.subject_to(X[:, k + 1] == x_next)

# 경계 조건
opti.subject_to(X[:, 0] == ca.vertcat(0, 0))    # 시작: 위치 0, 속도 0
opti.subject_to(X[:, N] == ca.vertcat(1, 0))     # 종료: 위치 1, 속도 0

# 입력 제약
opti.subject_to(opti.bounded(-5.0, U, 5.0))

# 상태 제약 (속도 제한)
opti.subject_to(opti.bounded(-2.0, X[1, :], 2.0))

# 솔버 설정
opti.solver('ipopt', {'print_time': False}, {'print_level': 0})
sol = opti.solve()

x_opt = sol.value(X)
u_opt = sol.value(U)
print(f"최적 궤적 - 최종 위치: {x_opt[0, -1]:.4f}")
print(f"최대 힘: {np.max(np.abs(u_opt)):.4f} N")
```

> **추천 자료**
> - [Matthew Kelly, "An Introduction to Trajectory Optimization" (SIAM Review 2017)](https://www.matthewpeterkelly.com/research/MatthewKelly_IntroTrajectoryOptimization_SIAM_Review_2017.pdf) — collocation과 shooting을 비교하는 최고의 튜토리얼
> - [CasADi](https://web.casadi.org/) — NLP 구현 표준 도구
> - [Drake Trajectory Optimization](https://drake.mit.edu/) — direct collocation 예제 포함

---

### 8.6 MoveIt2: 실전 모션 플래닝

MoveIt2는 ROS2 기반의 모션 플래닝 프레임워크이다. 산업/연구 양쪽에서 가장 널리 쓰이는 로봇 팔 플래닝 도구다.

**아키텍처:**
- **Planning Scene**: 로봇 + 환경(장애물)의 3D 모델 관리. 충돌 검사의 기반.
- **Planning Pipeline**: OMPL 등 플래너 호출 → 경로 검증 → 시간 매개변수화(time parameterization)
- **Move Group Interface**: 사용자 API. 목표 설정, 플래닝, 실행을 추상화.

**OMPL 통합**: MoveIt2는 OMPL을 기본 플래닝 백엔드로 사용한다. `ompl_planning.yaml`에서 플래너 종류와 파라미터를 설정한다.

```yaml
# ompl_planning.yaml 예시
manipulator:
  planner_configs:
    - RRTConnectkConfigDefault
    - RRTstarkConfigDefault
    - PRMkConfigDefault
  default_planner_config: RRTConnectkConfigDefault
  projection_evaluator: joints(joint1, joint2)
  longest_valid_segment_fraction: 0.01
```

**Pick-and-Place 파이프라인:**
1. 물체 인식 (Perception) → 물체의 6-DoF 포즈 추정
2. Grasp planning → 잡을 위치/자세 결정
3. Approach trajectory → 물체 위 접근점까지 모션 플래닝
4. Grasp → 그리퍼 닫기
5. Retreat trajectory → 물체를 들어올림
6. Place trajectory → 놓을 위치까지 모션 플래닝
7. Release → 그리퍼 열기

각 단계에서 MoveIt2가 충돌 회피와 관절 한계를 자동으로 처리한다.

---

### 8.7 심화: Optimization-Based Planning

*연구자가 되고 싶다면 여기서부터 읽어라.*

#### Constrained Nonlinear Optimization

실제 로봇의 궤적 최적화는 대부분 constrained NLP이다:

```
minimize    Σ L(x_k, u_k) + Φ(x_N)
subject to  x_{k+1} = f(x_k, u_k)           (동역학)
            h(x_k, u_k) = 0                  (등식 제약)
            g(x_k, u_k) <= 0                 (부등식 제약: 충돌, 토크 한계 등)
```

IPOPT(Interior Point Optimizer)가 이 문제를 푸는 표준 솔버다. CasADi에서 IPOPT를 기본으로 사용한다.

#### Contact-Implicit Trajectory Optimization

접촉 모드(어디가 닿아 있고 어디가 떨어져 있는지)를 미리 지정하지 않고, 최적화가 자동으로 결정하게 하는 방법이다. 걷기, 잡기 같은 접촉 전환이 필요한 태스크에서 유용하다.

접촉력을 결정 변수에 포함하고, 상보성 조건(complementarity constraint)을 추가한다:

```
F_n >= 0                   (접촉력은 당기지 못함)
d >= 0                     (물체가 바닥 아래로 못 감)
F_n * d = 0                (떨어져 있으면 힘 0, 닿아 있으면 거리 0)
```

이 문제는 수학적으로 MPCC(Mathematical Program with Complementarity Constraints)이고, 풀기 어렵다. Relaxation 기법이나 smoothed contact model을 쓴다.

Drake의 `ContactImplicitDirectCollocation`이 이 방법을 구현한다.

#### 실시간 Re-planning과 MPC의 연결

정적 환경에서 한 번 계획하면 끝이지만, 동적 환경에서는 실시간으로 재계획(re-plan)해야 한다. 여기서 궤적 최적화와 MPC가 만난다.

MPC는 짧은 horizon의 trajectory optimization이다. 매 제어 주기마다 짧은 구간의 궤적을 최적화하고, 첫 입력만 적용한 뒤 다시 최적화한다. 이전 장의 MPC가 정확히 이것이다.

차이점: 모션 플래닝의 trajectory optimization은 보통 오프라인으로 전체 궤적을 한 번에 계산하고, MPC는 온라인으로 짧은 구간을 반복 계산한다.

---

### 8.8 심화: Task and Motion Planning (TAMP)

*연구자가 되고 싶다면 여기서부터 읽어라.*

"컵을 선반 위에 놓아라"라는 명령을 수행하려면:

1. 컵이 어디 있는지 인식
2. 컵을 잡을 수 있는 grasp pose 결정
3. 접근 → 잡기 → 들기 → 이동 → 놓기 순서 계획
4. 각 단계의 모션 플래닝

1-3은 **symbolic planning** (어떤 순서로 어떤 action을 할지), 4는 **motion planning** (구체적으로 어떤 궤적으로 움직일지). TAMP는 이 둘을 결합한다.

#### PDDLStream

MIT에서 개발한 TAMP 프레임워크. PDDL(Planning Domain Definition Language)로 symbolic action을 정의하고, stream을 통해 연속적 파라미터(grasp pose, placement pose)를 생성한다.

#### LLM 기반 Task Planning

최근에는 LLM이 symbolic planner를 대체하는 시도가 활발하다:

- **SayCan** (Google, 2022): LLM이 가능한 action들의 자연어 설명을 평가하고, affordance model이 현재 상태에서 실행 가능한 action을 필터링한다. 둘의 곱으로 다음 action을 선택한다.
- **Code as Policies** (Google, 2023): LLM이 직접 로봇 제어 코드를 생성한다. 자연어 명령 → Python 코드 → 로봇 실행.
- **Inner Monologue** (Google, 2023): LLM + 환경 피드백의 반복적 대화로 태스크를 완수한다.

현실적 한계: LLM 기반 TAMP는 아직 실험 단계이다. 복잡한 기하학적 제약(좁은 공간에서의 조작, 정밀 조립)은 LLM이 처리하기 어렵고, 결국 전통적 motion planner가 필요하다. LLM은 high-level 계획, motion planner는 low-level 실행이라는 역할 분담이 현실적이다.

---

### 8.9 추천 자료

> **LaValle, "Planning Algorithms"**
> http://lavalle.pl/planning/
> 무료 온라인. 모션 플래닝의 가장 포괄적인 교과서. RRT의 원저자가 쓴 책이니 당연히 좋다.

> **Russ Tedrake, "Underactuated Robotics" Ch.10: Trajectory Optimization**
> https://underactuated.csail.mit.edu/trajopt.html
> Drake를 이용한 trajectory optimization 실습. 코드와 이론이 함께 제공된다.

> **Matthew Kelly, "An Introduction to Trajectory Optimization" (SIAM Review 2017)**
> https://www.matthewpeterkelly.com/research/MatthewKelly_IntroTrajectoryOptimization_SIAM_Review_2017.pdf
> Direct collocation과 shooting을 비교하는 최고의 튜토리얼. 예제 코드도 제공.

> **OMPL**
> https://ompl.kavrakilab.org/
> 오픈소스 모션 플래닝 라이브러리. RRT, RRT*, PRM 등 수십 가지 알고리즘 구현.

> **MoveIt2 Tutorials**
> https://moveit.picknik.ai/
> ROS2 기반 실전 모션 플래닝. Pick-and-place부터 고급 설정까지.

> **Drake**
> https://drake.mit.edu/
> Trajectory optimization + 시뮬레이션 통합 프레임워크. Contact-implicit 지원.

> **CasADi**
> https://web.casadi.org/
> Nonlinear trajectory optimization 구현 표준 도구.

---

### 기술 흐름

```
1979 ── Visibility graph 기반 path planning
1996 ── PRM (Kavraki et al.) — 샘플링 기반 플래닝의 시작
1998 ── RRT (LaValle) — single-query 플래닝의 표준
2000 ── RRT-Connect (Kuffner & LaValle) — 실무에서 가장 많이 쓰이는 변종
2009 ── CHOMP (Ratliff et al.) — gradient 기반 궤적 최적화
2011 ── RRT* (Karaman & Frazzoli) — 점근적 최적성 보장
2012 ── OMPL 1.0 공개 — 샘플링 기반 플래너 통합 라이브러리
2014 ── TrajOpt (Schulman et al.) — sequential convex optimization
2019 ── MoveIt2 (ROS2) — 산업/연구 표준 프레임워크
2022 ── SayCan (Google) — LLM + motion planning
2023 ── Contact-implicit trajectory optimization 실용화
2024 ── LLM 기반 TAMP 연구 확산
```
