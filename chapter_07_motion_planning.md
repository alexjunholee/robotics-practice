# Ch.7 — 모션 플래닝 & 궤적 최적화 (Motion Planning & Trajectory Optimization)

로봇이 A에서 B로 가려면 장애물·관절 한계·동역학 제약을 모두 만족하는 경로가 필요하다. 이런 경로를 찾는 과정이 모션 플래닝이고, 경로를 시간에 따라 최적으로 추종하도록 만드는 과정이 궤적 최적화다.

---

## 7.1 왜 모션 플래닝을 배우는가

6축 로봇 팔에게 "저 컵을 집어라"라고 명령했다고 하자. IK로 목표 관절 각도를 구했다. 그런데 현재 자세에서 목표 자세로 관절을 직선으로 보간(interpolation)하면, 팔이 테이블을 관통하거나 자기 몸체에 충돌할 수 있다. 관절 공간에서의 직선이 작업 공간에서의 직선이 아니기 때문이다.

모션 플래닝은 다음 질문에 답한다:
- 충돌 없이 목표에 도달하는 경로가 존재하는가?
- 존재한다면, 가장 짧은/빠른/부드러운 경로는 무엇인가?
- 동역학 제약(토크 한계, 속도 한계)을 만족하면서 그 경로를 따라갈 수 있는가?

---

## 7.2 Configuration Space (C-space)

로봇의 모든 가능한 상태를 하나의 공간으로 표현한다.

**Joint space = Configuration space**: n-DOF 로봇의 configuration은 q = (q1, q2, ..., qn)이다. 이 q가 살고 있는 n차원 공간이 C-space이다.

**C-space obstacle**: 작업 공간(3D)의 장애물을 C-space로 변환한 것이다. C-space에서 장애물 영역에 속하는 configuration은 충돌 상태이다.

왜 C-space에서 생각해야 하는가: 로봇은 점이 아니다. 3D 공간에서 로봇의 모든 링크가 장애물과 충돌하지 않는지 확인하려면, 각 configuration에서 FK를 계산하고 충돌 검사를 해야 한다. C-space에서는 로봇을 "점"으로 취급할 수 있고, 장애물을 피하는 문제가 점의 경로 찾기 문제로 환원된다.

문제는 C-space obstacle의 정확한 형태를 계산하기가 어렵다는 데 있다. 실무에서는 C-space obstacle을 명시적으로 구하지 않고, 특정 configuration에서의 충돌 여부를 검사하는 collision checker를 사용한다.

---

## 7.3 그래프 탐색 기반 플래닝

C-space를 이산화(discretize)하고 그래프 탐색 알고리즘으로 경로를 찾는, 가장 고전적인 접근이다.

### Dijkstra 알고리즘

음이 아닌 간선 가중치를 가진 그래프에서 최단 경로를 찾는다. binary heap과 adjacency list를 쓰면 시간 복잡도는 $O((V+E)\log V)$이며, 목표 노드가 확정되면 전체 간선을 모두 처리하기 전에 멈출 수 있다.

### A* 알고리즘

Dijkstra에 휴리스틱을 추가한 것이다. 목표까지의 추정 거리(heuristic)를 이용하여 탐색 방향을 유도한다. graph-search A*는 admissible하면서 consistent한 휴리스틱에서 최적해를 찾는다. 좋은 휴리스틱은 탐색 노드를 줄일 수 있지만, 실행 시간이 항상 Dijkstra보다 짧은 것은 아니다.

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
        return abs(a[0] - b[0]) + abs(a[1] - b[1])  # 맨해튼 거리

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

### 장단점

유한 격자에서 모든 도달 가능한 셀을 탐색하는 알고리즘은 그 **이산 문제**에 대해 완전하다. 그러나 격자 해상도 때문에 연속 공간의 좁은 통로를 놓칠 수 있고, **차원의 저주(curse of dimensionality)**에도 시달린다. 6-DOF 로봇 팔의 C-space를 각 축 100개로 이산화하면 100^6 = 10^12개의 셀이 된다. 이 한계 때문에 샘플링 기반 플래너가 등장했다.

---

## 7.4 샘플링 기반 플래너

C-space를 균일한 고정 격자로 모두 나누지 않고 표본을 뽑아 경로를 탐색한다. 고차원 C-space에서 중요한 선택지이며, optimization-based planning이나 search와 결합하기도 한다.

### RRT (Rapidly-exploring Random Tree)

LaValle (1998)이 제안한 알고리즘이다. 아이디어는 단순하다:

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
        self.obstacle_fn = obstacle_fn  # config → bool (충돌이면 True)
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

### RRT* (Optimal RRT)

Karaman & Frazzoli (2011). RRT는 해를 찾지만 최적이 아니다. RRT*는 새 노드 추가 시 근처 노드들과 re-wiring을 수행하여 점근적 최적성(asymptotic optimality)을 보장한다. 샘플 수가 무한대로 가면 최적 경로에 수렴한다.

실무적으로 RRT*는 RRT보다 좋은 경로를 찾지만, 수렴이 느리다. 시간 제한이 있는 실시간 상황에서는 RRT-Connect가 더 실용적인 경우가 많다.

### PRM (Probabilistic Roadmap)

Kavraki et al. (1996). RRT가 single-query(한 번에 하나의 start-goal 쌍)인 반면, PRM은 multi-query에 적합하다.

1단계 (offline): C-space에 많은 점을 샘플링하고, 가까운 점들을 충돌 없는 간선으로 연결하여 로드맵(graph)을 구축한다.
2단계 (online): start와 goal을 로드맵에 연결하고, 그래프 탐색(A* 등)으로 경로를 찾는다.

같은 환경에서 여러 경로 쿼리가 필요한 경우(예: 산업용 로봇 셀) PRM이 효율적이다.

### RRT-Connect

Kuffner & LaValle (2000). 시작점과 목표점에서 동시에 트리를 성장시키고, 두 트리가 만나면 경로를 연결한다. 빠르게 초기 경로를 찾는 용도로 널리 쓰이며, MoveIt2의 여러 OMPL 설정 예시에서도 `RRTConnect`를 기본 planner config로 지정한다. 실제 기본값은 배포판과 사용자 설정에 따라 달라진다.

### OMPL 라이브러리

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
> - [LaValle, "Planning Algorithms"](http://lavalle.pl/planning/) — 무료 온라인 교재. 모션 플래닝의 표준 교재
> - [OMPL](https://ompl.kavrakilab.org/) — 오픈소스 모션 플래닝 라이브러리
> - [MoveIt2 Tutorials](https://moveit.picknik.ai/) — ROS2 기반 실전 모션 플래닝 가이드

---

## 7.5 궤적 최적화 (Trajectory Optimization)

샘플링 기반 플래너는 "충돌 없는 경로"를 찾아준다. 하지만 그 경로는:
- 울퉁불퉁하다 (random sampling이므로)
- 동역학을 무시한다 (기구학적 경로만 제공)
- 시간 정보가 없다 (어떤 속도로 따라가야 하는지 모른다)

궤적 최적화는 이 한계를 보완한다. 비용 함수(시간, 에너지, 부드러움)를 최소화하면서, 동역학 제약, 충돌 회피, 관절 한계를 모두 만족하는 궤적을 찾는다.

### Direct Collocation

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

### Direct Shooting

상태를 결정 변수에서 제거하고, 입력 시퀀스 {u_0, u_1, ..., u_{N-1}}만을 결정 변수로 둔다. 상태는 동역학 시뮬레이션으로 계산한다.

collocation보다 결정 변수가 적지만, 시뮬레이션이 불안정하면 (예: 도립진자) 최적화도 불안정해진다.

### CHOMP (Covariant Hamiltonian Optimization for Motion Planning)

Ratliff et al. (2009). 초기 궤적(보통 직선 보간)에서 시작하여, 충돌 비용 + 부드러움 비용의 gradient를 따라 궤적을 반복적으로 개선한다. 공변 gradient(covariant gradient)를 사용하여 업데이트가 부드럽다.

장점: 직관적, 기존 궤적을 점진적으로 개선
단점: 좁은 통로(narrow passage)를 통과하기 어려움, 지역 최적해

### TrajOpt

Schulman et al. (2014). Sequential convex optimization 기반으로, 매 반복에서 비선형 문제를 선형/이차 근사로 바꿔서 풀고 trust region으로 근사의 유효 범위를 제한한다. 비볼록 문제이므로 전역 최적해는 보장하지 않는다. 충돌 회피를 signed distance 기반 비용으로 다뤄 gradient를 사용한다.

### CasADi를 이용한 Trajectory Optimization

CasADi는 symbolic computation, automatic differentiation, NLP solver 연결을 제공하는 널리 쓰이는 프레임워크다. Trajectory optimization에서는 Drake, direct solver API, JAX 기반 구현 등과 함께 선택지 중 하나다.

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
> - [Matthew Kelly, "An Introduction to Trajectory Optimization" (SIAM Review 2017)](https://www.matthewpeterkelly.com/research/MatthewKelly_IntroTrajectoryOptimization_SIAM_Review_2017.pdf) — collocation과 shooting을 비교하는 좋은 튜토리얼
> - [CasADi](https://web.casadi.org/) — automatic differentiation과 NLP solver 연결 도구
> - [Drake Trajectory Optimization](https://drake.mit.edu/) — direct collocation 예제 포함

---

## 7.6 MoveIt2: 실전 모션 플래닝

MoveIt2는 ROS2 기반의 공개 모션 플래닝 프레임워크로, 로봇 팔 연구와 응용에서 널리 쓰인다.

**아키텍처:**
- **Planning Scene**: 로봇 + 환경(장애물)의 3D 모델 관리. 충돌 검사의 기반.
- **Planning Pipeline**: OMPL 등 플래너 호출 → 경로 검증 → 시간 매개변수화(time parameterization)
- **Move Group Interface**: 사용자 API. 목표 설정, 플래닝, 실행을 추상화.

**OMPL 통합**: OMPL은 MoveIt2에서 사용할 수 있는 대표적인 planning pipeline plugin이다. `ompl_planning.yaml`에서 플래너 종류와 파라미터를 설정하며, 다른 pipeline도 구성할 수 있다.

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

## 7.7 심화: Optimization-Based Planning

### Constrained Nonlinear Optimization

실제 로봇의 궤적 최적화는 대부분 constrained NLP이다:

```
minimize    Σ L(x_k, u_k) + Φ(x_N)
subject to  x_{k+1} = f(x_k, u_k)           (동역학)
            h(x_k, u_k) = 0                  (등식 제약)
            g(x_k, u_k) <= 0                 (부등식 제약: 충돌, 토크 한계 등)
```

IPOPT(Interior Point Optimizer)가 이 문제를 푸는 표준 솔버다. CasADi에서 IPOPT를 기본으로 사용한다.

### Contact-Implicit Trajectory Optimization

접촉 모드(어디가 닿아 있고 어디가 떨어져 있는지)를 미리 지정하지 않고, 최적화가 자동으로 결정하게 하는 방법이다. 걷기, 잡기 같은 접촉 전환이 필요한 태스크에서 유용하다.

접촉력을 결정 변수에 포함하고, 상보성 조건(complementarity constraint)을 추가한다:

```
F_n >= 0                   (접촉력은 당기지 못함)
d >= 0                     (물체가 바닥 아래로 못 감)
F_n * d = 0                (떨어져 있으면 힘 0, 닿아 있으면 거리 0)
```

이 문제는 수학적으로 MPCC(Mathematical Program with Complementarity Constraints)이고, 풀기 어렵다. Relaxation 기법이나 smoothed contact model을 쓴다.

Drake의 `ContactImplicitDirectCollocation`이 이 방법을 구현한다.

### 실시간 Re-planning과 MPC의 연결

정적 환경에서 한 번 계획하면 끝이지만, 동적 환경에서는 실시간으로 재계획(re-plan)해야 한다. 궤적 최적화와 MPC가 여기서 만난다. MPC를 짧은 horizon의 trajectory optimization으로 볼 수 있다. 매 제어 주기마다 짧은 구간의 궤적을 최적화하고, 첫 입력만 적용한 뒤 다시 최적화한다. 이전 장의 MPC가 정확히 이것이다.

차이점: 모션 플래닝의 trajectory optimization은 보통 오프라인으로 전체 궤적을 한 번에 계산하고, MPC는 온라인으로 짧은 구간을 반복 계산한다.

---

## 7.8 심화: Task and Motion Planning (TAMP)

"컵을 선반 위에 놓아라"라는 명령을 수행하려면:

1. 컵이 어디 있는지 인식
2. 컵을 잡을 수 있는 grasp pose 결정
3. 접근 → 잡기 → 들기 → 이동 → 놓기 순서 계획
4. 각 단계의 모션 플래닝

1-3은 **symbolic planning** (어떤 순서로 어떤 action을 할지), 4는 **motion planning** (구체적으로 어떤 궤적으로 움직일지). TAMP는 이 둘을 결합한다.

### PDDLStream

MIT에서 개발한 TAMP 프레임워크. PDDL(Planning Domain Definition Language)로 symbolic action을 정의하고, stream을 통해 연속적 파라미터(grasp pose, placement pose)를 생성한다.

### LLM 기반 Task Planning

최근에는 LLM이 symbolic planner를 대체하는 시도가 활발하다:

- **SayCan** (Google, 2022): LLM이 가능한 action들의 자연어 설명을 평가하고, affordance model이 현재 상태에서 실행 가능한 action을 필터링한다. 둘의 곱으로 다음 action을 선택한다.
- **Code as Policies** (Google, 2023): LLM이 직접 로봇 제어 코드를 생성한다. 자연어 명령 → Python 코드 → 로봇 실행.
- **Inner Monologue** (Google, 2023): LLM + 환경 피드백의 반복적 대화로 태스크를 완수한다.

LLM 기반 TAMP는 아직 실험 단계이다. 복잡한 기하학적 제약(좁은 공간에서의 조작, 정밀 조립)은 LLM이 처리하기 어렵고, 결국 전통적 motion planner가 필요하다. LLM은 high-level 계획, motion planner는 low-level 실행이라는 역할 분담이 현실적이다.

TAMP는 환경과 행동이 결정론적이라고 가정한다. 환경 동역학과 관측이 확률적이라면 §7.9 심화: POMDP를 본다.

---

## 7.9 심화: 불확실성 하 의사결정 (POMDP와 belief space planning)

§7.1~§7.8은 로봇이 자신의 상태와 환경을 정확히 안다고 가정했다. 하지만 실제 로봇은 노이즈 있는 센서로 부분적인 정보만 관측한다. 대칭 복도에서 어느 쪽에 있는지 모르는 로봇, 문이 열려 있는지 닫혀 있는지 불확실한 상황 — 이럴 때 "현재 최선 추정 상태"에서 계획하면 틀린다. belief(사후 분포) 위에서 직접 계획해야 한다. 이 절의 내용은 Thrun, Burgard, Fox의 *Probabilistic Robotics* §15.2·§16을 기반으로 한다.

### 7.9.1 도입: 세 패러다임

같은 환경에서 세 가지 플래너가 다른 답을 낸다. Goal·Pit·Robot이 놓인 좌우 대칭 복도를 예로 들자.

Classical planning은 상태를 완전히 알고 행동도 결정론적이다. §7.3의 A*가 이 범주로, 최단 경로를 한 번 계산하면 실행 중 센싱이 필요 없다.

**MDP(Markov Decision Process)**: 상태는 완전히 관측되고 행동은 확률적이다. 정책 $\pi: s \to a$로 모든 상태에 행동을 매핑한다. 좁은 길에서 벽과 충돌 위험을 고려해 더 넓은 경로를 택할 수 있다. ch.8 §8.2가 이 범주이다.

**POMDP**: 행동·관측 모두 확률적. belief $b$ 위에 정책 $\pi: b \to a$를 정의한다. 대칭 복도에서 처음엔 위치를 모르기 때문에, 일부러 비대칭 영역으로 우회해 정보를 수집한 뒤 목표로 향한다. 이것이 **능동적 정보 수집(active information gathering)**이다.

세 패러다임은 classical $\subset$ MDP $\subset$ POMDP 순으로 포함된다. 불확실성의 축은 두 가지다. 행동 불확실성(어디로 가려 했는데 실제로 어디로 갔나)과 지각 불확실성(실제로 어디 있는데 센서가 뭐라 읽었나)이 그것이다. MDP는 전자만, POMDP는 둘 다 다룬다.

ch.3의 필터들이 belief를 *추적*했다면, 이 절은 추적된 belief로 *무엇을 할 것인가*를 본다.

<!-- DEMO: pomdp_three_paradigms.html -->

### 7.9.2 belief 위 가치 반복

세 패러다임의 수식을 비교하면 POMDP가 어디서 어려워지는지 바로 보인다. MDP 가치 반복의 핵심 식은 다음과 같다 (Bellman 방정식):

$$C^T(s) = \max_a \int \left[ c(s') + C^{T-1}(s') \right] P(s' \mid a, s)\, ds'$$

상태 $s$를 belief $b$로 바꾸면 POMDP의 가치 반복이 된다:

$$C^T(b) = \max_a \int \left[ c(b') + C^{T-1}(b') \right] P(b' \mid a, b)\, db' \tag{16.2}$$

정책은:

$$\pi^T(b) = \arg\max_a \int \left[ c(b') + C^{T-1}(b') \right] P(b' \mid a, b)\, db' \tag{16.3}$$

문제는 $b'$가 분포 위의 분포라는 점이다. $b$는 상태 공간 $\mathcal{S}$ 위의 확률 분포이고, $b'$는 그 분포들의 공간 위에 다시 분포한다. 적분 차원이 발산한다.

무한 horizon 극한에서 이 재귀가 수렴하면 표준 Bellman 방정식을 얻는다:

$$V(b) = \max_a \left[ r(b, a) + \gamma \sum_{o'} P(o' \mid b, a)\, V(B(b, a, o')) \right]$$

여기서 $r(b,a) = \sum_s b(s)\, c(s,a)$는 belief에 대한 기대 즉시 보상이다. 유한 horizon 재귀 형태로 쓰면 식 (16.2)가 된다.

관측 $o'$가 결정되면 사후 belief $B(b, a, o')$가 Bayes 필터로 *유일하게* 결정된다. 이 점을 이용하면 belief 공간 전체 적분을 관측 공간 위 적분으로 재구성할 수 있다:

$$C^T(b) = \max_a \int \left[ c(B(b, a, o')) + C^{T-1}(B(b, a, o')) \right] P(o' \mid a, b)\, do' \tag{16.34}$$

belief update operator는:

$$B(b, a, o')(s') = \frac{1}{P(o' \mid a, b)}\, P(o' \mid s') \int P(s' \mid a, s)\, b(s)\, ds$$

이산 상태·관측 공간에서는 적분이 합으로 대체된다. 이 재구성이 모든 현대 POMDP solver의 출발점이다.

### 7.9.3 4상태 toy 예시

PWLC(piecewise-linear convex) 구조를 직접 보려면 작은 예제가 필요하다. 4상태·2행동·2관측 문제를 손으로 계산해 보자.

**설정:**
- 상태 $s_1, s_2, s_3, s_4$. 초기에 $(s_1, s_2)$ 중 하나.
- 행동 $a_1$: 정보 수집. $s_1 \leftrightarrow s_2$를 0.9 확률로 교환.
- 행동 $a_2$: 종결. $s_3$(보상 +80) 또는 $s_4$(보상 -80)로 이동.
- 관측 $o_1, o_2$: $s_1$에서 확률 (0.7, 0.3), $s_2$에서 확률 (0.4, 0.6).
- belief는 $b = (p_1, p_2)$이고 $p_1 + p_2 = 1$이므로 1차원.

**horizon 1 계산:**

즉시 보상은 belief에 대해 선형이다: $c(b) = \sum_i c(s_i) p_i$.

$a_2$를 택하면 $T=1$ 가치 ($\gamma = 0.9$):
$$C^1(b, a_2) = \gamma(80 p_1 - 80 p_2) = 72 p_1 - 72 p_2$$

$a_1$을 택하면 종결 없으므로 즉시 보상만: $C^1(b, a_1) = 0$에 가깝다.

따라서:
$$C^1(b) = \max\{ 0,\; 72p_1 - 72p_2 \}$$

$C^1(b)$는 두 선형 함수의 max다. $p_1 = 0.5$에서 꺾인다. $p_1 > 0.5$이면 $a_2$, 아니면 $a_1$.

**horizon 2 계산:**

$a_1$ 후 관측 $o_1, o_2$가 올 확률을 적분하면:
$$C^2(b, a_1) \approx \max\{0,\; -33.05 p_1 + 7.78 p_2\}$$

(계수는 관측 확률과 belief update를 통해 계산.)

$T=2$ 전체:
$$C^2(b) = \max\{ 0,\; -33.05 p_1 + 7.78 p_2,\; 72 p_1 - 72 p_2 \}$$

세 선형 조각의 max. horizon이 늘수록 조각이 추가된다.

가치 함수는 belief 공간에서 볼록(convex)이다: $\beta C(b) + (1-\beta) C(b') \geq C(\beta b + (1-\beta) b')$. 불확실한 belief보다 확실한 belief에서 가치가 항상 높거나 같다.

<!-- DEMO: pomdp_toy_pwlc.html -->

### 7.9.4 PWLC 구조와 alpha-vectors

4상태 예제에서 가치 함수가 *선형 조각의 max* 형태임을 보았다. 이것이 우연이 아님을 귀납으로 보인다.

베이스 케이스 ($T=1$): 즉시 보상 $c(b) = \sum_i c(s_i) p_i$는 belief에 대해 선형이다. 따라서 $C^1(b) = \max_a \sum_i C^1_{a,i}\, p_i$이고, 각 행동에 대해 선형 함수 하나씩이 나온다.

귀납 단계: $C^{T-1}(b)$가 PWLC라고 하자. 식 (16.34)에서 $C^{T-1}(B(b,a,o'))$를 $b$의 함수로 전개하면: belief update의 비선형 정규화 인자 $1/P(o'\mid a, b)$가 식 (16.34)의 가중치 $P(o'\mid a, b)$와 상쇄되어, 각 alpha-vector와의 내적 $\langle \phi, B(b,a,o') \rangle \cdot P(o'\mid a, b)$이 $b$의 선형 함수로 정리된다. 선형 함수들의 max의 max는 여전히 선형 함수들의 max다. 따라서 $C^T(b)$도 PWLC.

각 선형 조각의 계수 벡터를 **alpha-vector** $\phi$라 한다. 가치 함수는:

$$V(b) = \max_\phi \langle \phi, b \rangle$$

$\Phi$가 alpha-vector 집합이면 $V(b) = \max_{\phi \in \Phi} \sum_i \phi_i\, p_i$.

각 alpha-vector는 하나의 *조건부 정책*(현재 행동 + 관측에 따른 후속 정책)에 대응한다. $|\Phi^T| = |A| \cdot |\mathcal{O}|^{|\Phi^{T-1}|}$으로 이중 지수적으로 폭발한다. $|\Phi^1| = 1$에서 시작해 $|\Phi^2| = 2 \cdot 2^1 = 4$, $|\Phi^3| = 2 \cdot 2^4 = 32$, $|\Phi^4| = 2 \cdot 2^{32} \approx 10^{10}$으로 horizon 4에서 이미 100억 개를 넘는다. 정확 해법이 비실용적인 이유다.

### 7.9.5 LP 해법

alpha-vector 수가 이중지수적으로 증가한다는 것이 문제라면, 그 max·sum·max 구조를 LP(linear program)로 직접 환원하여 alpha-vector를 열거하지 않고 정확 해를 구하는 방법이 있다.

**변환 원리**: $C = \max_a x(a)$는 $\{C \geq x(a) \;\forall a\}$ 제약에서 $\min C$로 풀린다. $C = \sum_i \max_a x(a,i)$는 각 $i$마다 행동을 선택하는 함수 $a(\cdot)$의 모든 조합에 대해 $\{C \geq \sum_i x(a(i),i)\}$ 제약을 만들면 된다. 제약 수는 $|A|^{|\mathcal{S}|}$.

POMDP horizon $T$의 제약 (식 16.67):

$$\bigcup_a \bigcup_{k(o'):1 \leq k(o') \leq |\Phi^{T-1}|} \left\{ C^T(b) \geq \gamma \sum_{o'} \sum_i \left(c_i + C^{T-1}_{k(o'),i}\right) P(o' \mid s_i') \sum_j P(s_i' \mid a, s_j)\, p_j \right\}$$

제약 수는 $|\Phi^T| = |A| \cdot |\mathcal{O}|^{|\Phi^{T-1}|}$.

---

**알고리즘: finite_world_POMDP** (Thrun et al., Table 16.1 의역)

```
Algorithm finite_world_POMDP(T):
  Φ¹ = { φ : C¹(b) = γ Σᵢ c(sᵢ) pᵢ }     # horizon 1 단일 alpha-vector

  for t = 2 to T:
    Φᵗ = ∅
    for each action a:
      for each assignment k(o') ∈ {1, …, |Φᵗ⁻¹|} for each o':
        # 새 alpha-vector 계산
        for each state sⱼ:
          φⱼ = γ Σₒ' Σᵢ (cᵢ + Φᵗ⁻¹[k(o'), i]) · P(o'|sᵢ') · P(sᵢ'|a, sⱼ)
        Φᵗ = Φᵗ ∪ { ⟨a, φ⟩ }

  # dominated alpha-vectors 제거 (pruning)
  Φᵀ = prune(Φᵀ)
  return Φᵀ
```

---

$|\Phi^T|$는 이중지수적으로 증가한다. horizon 5, 행동 3개, 관측 5개면 이미 수백만 개의 alpha-vector가 필요하고, pruning을 해도 현실적인 도메인에서는 감당이 안 된다. 정확 해법은 개념 증명이고, 실제로는 근사가 필수다.

### 7.9.6 일반 POMDP

이산 finite-state 문제에서 LP 해법이 이미 비실용적이라면, 연속 상태 공간에서는 어떤 일이 벌어지는가.

상태 공간이 연속이면 alpha-vector 표현도 연속 함수가 된다. 식 (16.34)는 원칙적으로 여전히 성립하지만, $\Phi^{T-1}$가 함수들의 집합으로 무한차원이 된다.

---

**알고리즘: POMDP(T)** (Thrun et al., Table 16.2 의역, 압축)

```
Algorithm POMDP(T):
  초기화: Φ¹ ← horizon 1 가치 함수 (연속)

  for t = 2 to T:
    for each action a:
      for each "conditional plan" k(·) mapping observations to Φᵗ⁻¹ elements:
        새 함수 φ(b) = γ ∫ₒ' [ c(B(b,a,o')) + Φᵗ⁻¹[k(o')](B(b,a,o')) ] P(o'|a,b) do'
        Φᵗ ← Φᵗ ∪ { φ }

  return Φᵀ
```

---

연속 공간에서는 함수들의 집합을 저장·비교하는 것 자체가 비실용적이다. 이 알고리즘은 in-principle 해법이고, 실용 알고리즘(MC-POMDP, AMDP)이 대안으로 등장한다.

### 7.9.7 MC-POMDP

정확 해법이 막혔으니, belief를 표본으로 근사하여 계산을 현실적인 수준으로 낮추는 방향을 택한다.

particle filter로 belief를 표현하고 가치 반복 갱신을 표본 기반으로 근사한다. ch.3 §3.11의 파티클 필터가 추정용으로 쓰였다면, 여기서는 *플래닝용*으로 쓰인다.

belief $\theta$는 가중 입자 집합 $\langle s^{(i)}, w^{(i)} \rangle$이다. belief update $B(b, a, o')$를 입자 형태로 구현한다:

```
Algorithm particle_filter_belief_update(θ, a, o'):
  θ' = ∅
  for i = 1 to N:
    s ~ θ                         # 입자 샘플링
    s' ~ P(s'|a, s)               # 운동 모델 (motion model)
    w' = P(o'|s')                 # 측정 모델 (measurement model)
    θ' ← θ' ∪ { ⟨s', w'⟩ }
  normalize weights in θ'
  return θ'
```

가치 반복 갱신은 belief $\theta$마다 행동 $a$별 Q값 $Q(\theta, a)$를 학습한다. 각 belief에서 $N$번 샘플링하고, 다음 belief에서 max Q를 가져와 평균낸다.

---

**알고리즘: MC-POMDP** (Thrun et al., Table 16.3 골격 의역)

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

Q함수 갱신 (식 16.78):

$$Q(\theta_t, a_t) \leftarrow \mathbb{E}\left[ R(o_{t+1}) + \gamma \max_{\bar{a}} Q(\theta_{t+1}, \bar{a}) \right]$$

정책 (식 16.79):

$$\sigma^Q(\theta) = \arg\max_{\bar{a}} Q(\theta, \bar{a})$$

Q값 함수 근사는 nearest-neighbor 방식을 쓴다. belief $\theta$가 입자 집합이라 feedforward 네트워크의 입력으로 직접 쓸 수 없다. 대신 $\langle \theta, a, Q \rangle$ 데이터베이스를 유지하고, 새 belief $\theta'$가 들어오면 KL divergence로 $k$-nearest neighbor를 찾아 Q값 평균을 쓴다.

두 belief 사이의 KL divergence는 Gaussian KDE로 근사한다. KL 기반 kNN이 함수 근사기 역할을 한다. 현대에서는 neural function approximation으로 대체되었지만 알고리즘 골격은 동일하다.

Outer loop는 belief 데이터베이스를 정적으로 유지하거나, $\varepsilon$-greedy 시뮬레이션 trial로 belief를 자연스럽게 방문하며 생성한다. 후자가 실제 로봇 궤적에서 마주칠 belief에 집중하기 때문에 계산 예산을 아낄 수 있다.

### 7.9.8 실험: heaven/hell과 find-and-fetch

**Heaven/Hell 문제**: T자 복도에서 한쪽 끝은 천국(+1), 반대쪽은 지옥(-1)이다. 오직 입구 근처 priest만이 어느 쪽이 천국인지 안다. 로봇은 먼저 priest에게 물어보고(정보 수집) 올바른 방향으로 가야 한다. POMDP planner는 priest로 우회하는 정책을 자동으로 학습한다. 직접 최단 경로로 가면 50%의 확률로 지옥에 도달하지만, priest를 거치면 올바른 방향으로 갈 수 있다.

**Find-and-Fetch (단안 카메라)**: 로봇이 단안 카메라로 목표 물체를 찾고 가져오는 태스크다. 카메라로는 물체의 방향은 알지만 거리를 정확히 모른다. MC-POMDP는 능동적으로 시점을 바꿔 거리 불확실성을 줄이는 정책을 학습한다. 물체를 여러 각도에서 관찰해 위치를 좁힌 뒤 접근한다.

두 실험 모두 belief를 추적하며 *정보 수집 행동*을 계획에 포함시킨다. 상태 추정 후 greedy action selection만 하면 이런 우회 경로는 나오지 않는다.

### 7.9.9 AMDP — belief 통계로 차원 축소

MC-POMDP가 입자 집합으로 belief를 직접 추적한다면, 같은 불확실성을 훨씬 적은 수의 통계량으로 요약할 수 있다. 이 발상이 AMDP(Augmented MDP)의 출발점이다. POMDP의 두 극단은 MDP($|S|$에 polynomial)와 정확 POMDP(이중지수)이고, AMDP는 그 사이 절충이다.

아이디어: 실제 로봇 궤적에서 belief는 belief 공간 전체를 채우지 않고 좁은 manifold만 점유한다. 그 manifold를 *저차원 통계* $\bar{b} = f(b)$로 요약하고, $\bar{b}$ 위에서 표준 MDP 가치 반복을 적용한다.

**표준 통계 선택** (식 16.80):

$$\bar{b} = \langle \arg\max_s b(s),\; H[b] \rangle$$

최대 가능 상태 + belief 엔트로피. 엔트로피는:

$$H[b] = -\int b(s) \ln b(s)\, ds \tag{16.81}$$

무한 차원 belief를 스칼라 하나로 요약한다. 이것이 *충분 통계*인지는 보장되지 않지만 ("충분 통계라는 가정이 거의 성립하지 않는다"고 Thrun et al.이 명시한다), coastal navigation·heaven/hell 실험에서 합리적 행동 선택에 충분함이 확인된다.

$\arg\max_s b(s)$만 쓰면 표준 MDP 그대로다. 거기에 엔트로피를 더해 "내가 얼마나 모르는가"를 상태에 포함시켰다.

---

**알고리즘: Augmented_MDP_value_iteration** (Thrun et al., Table 16.4 의역)

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

MDP_value_iteration (§15.3.3)과 외형이 동일하다. 상태가 $s$ 대신 $\bar{b}$라는 점만 다르다.

전이 확률 $P(\bar{b}' \mid a, \bar{b})$ 계산 (식 16.85):

$$P(\bar{b}' \mid a, \bar{b}) = \int\!\!\int\!\!\int I_{f(b)=\bar{b}}\, I_{f(B(o',a,b))=\bar{b}'}\, P(o' \mid s') P(s' \mid a, s) P(s \mid b)\, ds\, ds'\, do'\, db$$

실용에서는 시뮬레이션 + lookup table 캐시로 근사한다. 여러 랜덤 시도로 전이를 통계적으로 추정한다.

### 7.9.10 Coastal Navigation 예시

coastal navigation은 AMDP가 낳는 emergent 행동 중 설명이 가장 쉬운 사례다.

동기: 넓은 open space를 가로지를 때 conventional MDP planner는 직선 경로를 택한다. 거리가 짧아서다. 하지만 open space에서는 라이더나 카메라가 특징이 없는 벽만 보이므로 위치 belief의 엔트로피가 크게 증가한다. 목적지에 도착해도 어디 있는지 모른다.

AMDP planner는 같은 환경에서 벽을 따라 도는 곡선 경로를 선택한다. 벽 근처에서 라이더 측정이 위치를 잘 제약하여 엔트로피가 낮게 유지된다. 비용 함수에 엔트로피가 포함되어 있으므로, 정보가 많은 경로를 선호하는 동작이 자동으로 나온다.

비유: 선박이 GPS 없이 항해할 때 해안선을 따라간다(coast = 해안). 랜드마크가 많은 경로가 위치 유지에 유리하기 때문이다.

Thrun et al.의 그림 16.5에서 센서 range를 줄일수록 conventional planner의 도착 엔트로피는 급격히 커지지만, coastal planner의 도착 엔트로피는 거의 변화가 없다. 정보를 고려한 경로 계획의 강건성이 여기서 드러난다.

Active SLAM에서 위치 불확실성을 줄이도록 경로를 선택하는 것, next-best-view planning에서 정보량이 큰 시점으로 이동하는 것이 모두 coastal navigation의 현대적 형태다.

<!-- DEMO: coastal_navigation_amdp.html -->

### 7.9.11 무엇이 살아남았나

coastal navigation은 비용 함수에 엔트로피를 포함했을 때 계획기가 자동으로 도달하는 결론이다. §7.9.2의 belief 위 가치 반복이 실제 경로 선택에서 어떻게 드러나는지 보여주는 가장 직관적인 사례이기도 하다.

정확 해법(§7.9.5·§7.9.6)은 비실용적이지만 개념 도구로 살아있고, 현대 POMDP solver는 이를 기반으로 세 방향으로 발전했다.

Point-based value iteration(SARSOP, HSVI, PBVI)은 belief 공간 전체가 아니라 샘플된 belief 점에서만 alpha-vector backup을 수행한다. §7.9.4의 alpha-vector 구조는 그대로이고, 탐색 범위를 제한하여 폭발을 막는다.

**MCTS 계열**: POMCP(Silver & Veness, 2010), DESPOT. rollout으로 Q값을 추정하고 belief tree를 MCTS로 탐색한다. §7.9.7 MC-POMDP의 Q 추정 구조를 tree search에 결합했다.

**Deep POMDP**: DRQN(Recurrent Q-network), DVRL(Igl et al.). RNN의 hidden state가 implicit belief 역할을 한다. MC-POMDP의 nearest-neighbor 함수 근사가 neural function approximation으로 대체된 형태다.

AMDP에서 사용한 원리는 다른 방법에도 나타난다. Bayes-adaptive MDP(BAMDP)는 모델 파라미터의 사후 분포를 증강 상태로 쓰고, Active SLAM은 위치 belief의 분산을 비용 함수에 포함한다. NeRF 기반 능동 인식에서도 entropy-augmented planning을 사용한다.

ch.8 §8.3의 PPO·SAC 등 deep RL은 경험에서 학습하고, 모델(전이 확률, 관측 모델)을 알 필요가 없다. POMDP planning은 모델을 알 때 최적 정책을 계산한다. 모델이 없으면 MC-POMDP도 돌아가지 않는다. MC-POMDP는 그 교집합에 있다 — belief는 모델로 추적하고, Q값은 경험에서 학습한다.

belief 위의 가치 반복은 상태 수에 이중지수적으로 폭발하기 때문에 근사 solver가 등장했다. MC-POMDP는 particle filter로 belief를 표현하고 Q값을 표본으로 학습한다. AMDP는 belief를 (최대 가능 상태, 엔트로피) 쌍으로 요약해 표준 MDP로 환원한다. 방법은 다르지만 목적은 같다 — 능동적 정보 수집. 그 현대적 형태가 MCTS 기반 POMCP, Deep POMDP, Active SLAM의 entropy-augmented planning이다.

---

## 7.10 추천 자료

> **LaValle, "Planning Algorithms"**
> http://lavalle.pl/planning/
> 무료 온라인. 모션 플래닝의 가장 포괄적인 교과서. RRT의 원저자가 쓴 책이니 당연히 좋다.

> **Russ Tedrake, "Underactuated Robotics" Ch.10: Trajectory Optimization**
> https://underactuated.csail.mit.edu/trajopt.html
> Drake를 이용한 trajectory optimization 실습. 코드와 이론이 함께 제공된다.

> **Matthew Kelly, "An Introduction to Trajectory Optimization" (SIAM Review 2017)**
> https://www.matthewpeterkelly.com/research/MatthewKelly_IntroTrajectoryOptimization_SIAM_Review_2017.pdf
> Direct collocation과 shooting을 비교하는 좋은 튜토리얼. 예제 코드도 제공.

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

> **추가 논문**
> - [Garrett et al., "Integrated Task and Motion Planning" (2021, arXiv:2010.01083)](https://arxiv.org/abs/2010.01083) — TAMP의 표준 서베이 논문
> - [Janner et al., "Planning with Diffusion for Flexible Behavior Synthesis" (ICML 2022, arXiv:2205.09991)](https://arxiv.org/abs/2205.09991) — trajectory-level diffusion 기반 planning의 시작

---

## 기술 흐름

```
1979 ── Visibility graph 기반 path planning
1996 ── PRM (Kavraki et al.) — 샘플링 기반 플래닝의 시작
1998 ── RRT (LaValle) — 영향력 큰 single-query sampling planner
2000 ── RRT-Connect (Kuffner & LaValle) — 실무 motion-planning 라이브러리에서 널리 제공되는 변종
2009 ── CHOMP (Ratliff et al.) — gradient 기반 궤적 최적화
2011 ── RRT* (Karaman & Frazzoli) — 점근적 최적성 보장
2012 ── OMPL 1.0 공개 — 샘플링 기반 플래너 통합 라이브러리
2014 ── TrajOpt (Schulman et al.) — sequential convex optimization
2019 ── MoveIt2 (ROS2) — 산업·연구에서 쓰이는 공개 motion-planning framework
2022 ── SayCan (Google) — LLM + motion planning
2023 ── Contact-implicit trajectory optimization 실용화
2024 ── LLM 기반 TAMP 연구 확산
```
