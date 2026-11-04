# Ch.7 — 모션 플래닝 & 궤적 최적화 (Motion Planning & Trajectory Optimization)

로봇이 A에서 B로 가려면 장애물·관절 한계·동역학 제약을 모두 만족하는 경로가 필요하다. 모션 플래닝은 충돌 없는 기하학적 경로를 찾는다. 궤적 최적화는 동역학 한계와 목적 함수를 반영해 시간에 따른 궤적을 만든다.

---

## 7.1 왜 모션 플래닝을 배우는가

6축 로봇 팔에게 "저 컵을 집어라"라고 명령했다고 하자. IK로 목표 관절 각도를 구했다. 그런데 현재 자세에서 목표 자세로 관절을 직선으로 보간(interpolation)하면, 팔이 테이블을 관통하거나 자기 몸에 부딪힐 수 있다. 관절 공간의 직선은 작업 공간에서 곡선을 그리기 때문이다.

모션 플래닝은 다음 질문에 답한다:
- 충돌 없이 목표에 도달하는 경로가 존재하는가?
- 존재한다면, 가장 짧은/빠른/부드러운 경로는 무엇인가?
- 동역학 제약(토크 한계, 속도 한계)을 만족하면서 그 경로를 따라갈 수 있는가?

---

## 7.2 Configuration Space (C-space)

로봇이 취할 수 있는 모든 자세를 한 공간의 점으로 나타낸다.

Joint space = Configuration space: n-DOF 로봇의 configuration은 q = (q1, q2, ..., qn)이다. q가 속한 n차원 공간이 C-space다.

C-space obstacle: 작업 공간(3D)의 장애물과 충돌하는 configuration들의 집합이다.

로봇은 부피가 있는 링크를 이어 놓은 것이다. 3D 공간에서 충돌을 확인하려면 각 configuration에서 FK를 계산해 모든 링크를 검사한다. C-space에서는 로봇 전체가 점 하나가 되므로, 장애물 회피는 점의 경로를 찾는 문제가 된다.

C-space obstacle의 정확한 형태는 계산하기 어렵다. 그래서 실무에서는 주어진 configuration 하나의 충돌 여부를 답하는 collision checker를 반복 호출한다.

---

## 7.3 그래프 탐색 기반 플래닝

C-space를 이산화(discretize)하고 그래프 탐색 알고리즘으로 경로를 찾는다.

### Dijkstra 알고리즘

간선 가중치가 음수가 아닌 그래프에서 최단 경로를 찾는다. binary heap과 adjacency list를 쓰면 시간 복잡도는 $O((V+E)\log V)$이며, 목표 노드가 확정되면 전체 간선을 모두 처리하기 전에 멈출 수 있다.

### A* 알고리즘

A*는 Dijkstra에 목표까지의 추정 거리(heuristic)를 더해 탐색 방향을 목표 쪽으로 끈다. graph-search A*는 admissible하면서 consistent한 휴리스틱에서 최적해를 찾는다. 좋은 휴리스틱은 탐색 노드 수를 줄인다. 다만 휴리스틱 계산 비용이 크면 전체 실행 시간은 Dijkstra보다 길어질 수 있다.

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
        return np.hypot(a[0] - b[0], a[1] - b[1])  # 대각선 이동 비용과 일치하는 유클리드 거리

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

유한 격자에서 도달 가능한 셀을 모두 탐색하는 알고리즘은 그 이산 문제에서 완전(complete)하다. 해가 있으면 반드시 찾는다는 뜻이다. 격자 해상도보다 좁은 통로는 격자에서 막힌 칸으로 바뀌어 사라진다. 차원이 늘면 셀 수가 지수적으로 늘어난다(차원의 저주, curse of dimensionality). 6-DOF 로봇 팔의 C-space를 각 축 100개로 이산화하면 100^6 = 10^12개의 셀이 된다. 그래서 고차원 C-space에서는 샘플링 기반 플래너를 쓴다.

---

## 7.4 샘플링 기반 플래너

C-space에서 무작위 표본을 뽑아 그 표본들로 경로를 탐색한다. optimization-based planning이나 search와 결합하기도 한다.

### RRT (Rapidly-exploring Random Tree)

LaValle (1998)이 제안했다.

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

Karaman & Frazzoli (2011). RRT가 찾은 경로는 최적 경로보다 훨씬 길 수 있다. RRT*는 새 노드를 추가할 때 근처 노드의 부모를 더 짧은 경로 쪽으로 바꿔 잇는 rewiring으로 점근적 최적성(asymptotic optimality)을 얻는다. 샘플 수가 무한대로 가면 최적 경로에 수렴한다.

RRT*는 RRT보다 좋은 경로를 찾는 대신 계산이 더 든다. 시간 제한이 빡빡하면 RRT-Connect(아래 절)로 빨리 경로를 얻고 나중에 다듬는다.

### PRM (Probabilistic Roadmap)

Kavraki et al. (1996). RRT는 single-query(한 번에 하나의 start-goal 쌍)용이다. PRM은 한 번 만든 로드맵을 여러 쿼리에 다시 쓴다(multi-query).

1단계 (offline): C-space에 많은 점을 샘플링한다. 가까운 점끼리 충돌 없는 간선으로 이어 로드맵(graph)을 만든다.
2단계 (online): start와 goal을 로드맵에 연결하고, 그래프 탐색(A* 등)으로 경로를 찾는다.

산업용 로봇 셀처럼 같은 환경에서 경로 쿼리가 반복되면 PRM이 효율적이다.

### RRT-Connect

Kuffner & LaValle (2000). 시작점과 목표점에서 동시에 트리를 키우고, 두 트리가 만나면 두 경로를 잇는다. 초기 경로를 빨리 찾는 용도로 널리 쓰이며, MoveIt2의 여러 OMPL 설정 예시에서도 `RRTConnect`를 기본 planner config로 지정한다. 실제 기본값은 배포판과 사용자 설정에 따라 달라진다.

### OMPL 라이브러리

Open Motion Planning Library (https://ompl.kavrakilab.org/). Kavraki Lab (Rice University)에서 개발한 C++ 라이브러리다. RRT, RRT*, RRT-Connect, PRM, EST, KPIECE를 비롯한 수십 가지 샘플링 기반 플래너가 들어 있다.

충돌 검사는 사용자가 state validity checker로 OMPL에 넘긴다. MoveIt2는 OMPL과 FCL(Flexible Collision Library)을 묶어 모션 플래닝 파이프라인을 구성한다.

```python
# MoveIt2에서 OMPL 기반 모션 플래닝 (ROS2 Python API, 간략화)
from moveit.planning import MoveItPy

moveit = MoveItPy(node_name="motion_planner")
arm = moveit.get_planning_component("manipulator")

# 목표 설정
arm.set_goal_state(configuration_name="home")

# 플래닝 (기본 planner는 설정에 따름. 흔히 RRTConnect)
plan_result = arm.plan()

if plan_result:
    # 실행: PlanningComponent에는 execute()가 없고 MoveItPy 객체가 궤적을 실행한다
    moveit.execute(plan_result.trajectory, controllers=[])
```

> **추천 자료**
> - [LaValle, "Planning Algorithms"](https://lavalle.pl/planning/) — 무료 온라인 교재. 모션 플래닝 강의에서 널리 쓰는 교재
> - [OMPL](https://ompl.kavrakilab.org/) — 오픈소스 모션 플래닝 라이브러리
> - [MoveIt2 Tutorials](https://moveit.picknik.ai/) — ROS2 기반 실전 모션 플래닝 가이드

---

## 7.5 궤적 최적화 (Trajectory Optimization)

샘플링 기반 플래너가 주는 충돌 없는 경로에는 다음 문제가 남는다.
- 무작위로 샘플링해 얻은 경로라 울퉁불퉁하다
- 기구학적 경로만 주고, 동역학 제약은 사용자가 따로 확인한다
- 시간 정보, 곧 각 점을 지날 속도가 비어 있다

궤적 최적화는 비용 함수(시간, 에너지, 부드러움)를 최소화하면서 동역학 제약, 충돌 회피, 관절 한계를 모두 만족하는 궤적을 찾는다.

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

장점: 동역학과 제약을 한 문제에서 처리, 부드러운 궤적
단점: 초기 추측(initial guess)에 민감, 비볼록이므로 지역 최적해에 빠질 수 있음

### Direct Shooting

상태를 결정 변수에서 제거하고, 입력 시퀀스 {u_0, u_1, ..., u_{N-1}}만을 결정 변수로 둔다. 상태는 동역학 시뮬레이션으로 계산한다.

collocation보다 결정 변수가 적다. 대신 도립진자처럼 시뮬레이션이 불안정한 시스템에서는 최적화도 불안정해진다.

### CHOMP (Covariant Hamiltonian Optimization for Motion Planning)

Ratliff et al. (2009). 초기 궤적(보통 직선 보간)에서 시작해 충돌 비용 + 부드러움 비용의 gradient를 따라 궤적을 반복적으로 개선한다. 공변 gradient(covariant gradient)를 써서 궤적 전체가 매끄럽게 갱신된다.

장점: 구현이 단순, 기존 궤적을 점진적으로 개선
단점: 좁은 통로(narrow passage)를 통과하기 어려움, 지역 최적해

### TrajOpt

Schulman et al. (2014). Sequential convex optimization(순차 볼록 최적화)을 쓴다. 매 반복에서 비선형 문제를 선형·이차 근사로 바꿔 풀고, trust region으로 근사를 믿을 수 있는 범위를 제한한다. 문제가 비볼록이라 결과는 지역 최적해다. 충돌 회피는 signed distance 기반 비용의 gradient로 처리한다.

### CasADi를 이용한 Trajectory Optimization

CasADi는 기호 계산(symbolic computation), 자동 미분(automatic differentiation), NLP solver 연결 기능을 갖춘 프레임워크다. Trajectory optimization에는 Drake, JAX 기반 구현 등도 쓴다.

```python
import casadi as ca
import numpy as np

# 간단한 예: 고정 시간 동안 1D 더블 인티그레이터의 제어 입력 제곱 적분 최소화
# x = [position, velocity], u = force
# x_dot = [velocity, force/mass]

N = 50           # 구간 수
dt = 0.1         # 시간 간격
mass = 1.0

opti = ca.Opti()

# 결정 변수
X = opti.variable(2, N + 1)  # 상태 궤적
U = opti.variable(1, N)      # 입력 궤적

# 비용: 고정 시간 동안 제어 입력 제곱 적분 최소화
cost = 0
for k in range(N):
    cost += U[0, k]**2 * dt  # 제어 입력 제곱 적분
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

MoveIt2는 ROS2용 오픈소스 모션 플래닝 프레임워크로, 로봇 팔 연구와 응용에서 널리 쓰인다.

**아키텍처:**
- Planning Scene: 로봇 + 환경(장애물)의 3D 모델 관리. 충돌 검사의 기반.
- Planning Pipeline: OMPL 등 플래너 호출 → 경로 검증 → 시간 매개변수화(time parameterization)
- Move Group Interface: 사용자 API. 목표 설정, 플래닝, 실행을 추상화.

OMPL 통합: OMPL은 MoveIt2의 대표적인 planning pipeline plugin이다. `ompl_planning.yaml`에서 플래너 종류와 파라미터를 설정하며, 다른 pipeline도 구성할 수 있다.

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

모션 플래닝 단계에서는 MoveIt2가 충돌 회피와 관절 한계를 처리한다.

---

## 7.7 심화: Optimization-Based Planning

### Constrained Nonlinear Optimization

실제 로봇의 궤적 최적화는 다음 형태의 constrained NLP로 쓴다.

```
minimize    Σ L(x_k, u_k) + Φ(x_N)
subject to  x_{k+1} = f(x_k, u_k)           (동역학)
            h(x_k, u_k) = 0                  (등식 제약)
            g(x_k, u_k) <= 0                 (부등식 제약: 충돌, 토크 한계 등)
```

이 문제는 흔히 IPOPT(Interior Point Optimizer)로 푼다. 앞의 CasADi 예제도 IPOPT를 썼다.

### Contact-Implicit Trajectory Optimization

접촉 모드(어디가 닿아 있고 어디가 떨어져 있는지)를 최적화가 직접 정하게 하는 방법이다. 걷기, 잡기 같은 접촉 전환이 필요한 태스크에서 유용하다.

접촉력을 결정 변수에 포함하고, 상보성 조건(complementarity constraint)을 추가한다:

```
F_n >= 0                   (접촉력은 당기지 못함)
d >= 0                     (물체가 바닥 아래로 못 감)
F_n * d = 0                (떨어져 있으면 힘 0, 닿아 있으면 거리 0)
```

이 문제는 MPCC(Mathematical Program with Complementarity Constraints)다. MPCC도 NLP의 한 종류지만, 상보성 제약 때문에 문제가 ill-posed해서 그대로 풀면 수렴이 잘 안 된다. Posa, Cantu & Tedrake(2014)는 상보성 조건을 $F_n \cdot d \le 0$ 꼴로 바꾸고 SQP 솔버 SNOPT의 elastic mode로 풀었다. 어려운 예제에서는 $F_n \cdot d \le \epsilon$으로 잠시 완화한 뒤 $\epsilon$을 0까지 줄였다. 상보성 조건 대신 smoothed contact model을 쓰는 방법도 있다.

Drake에서는 `MathematicalProgram`에 접촉 상보성 제약을 직접 추가해 문제를 만든다.

### 실시간 Re-planning과 MPC의 연결

정적 환경에서는 한 번 계획한 궤적을 그대로 따라간다. 동적 환경에서는 실시간으로 재계획(re-plan)한다. MPC는 짧은 horizon의 trajectory optimization이다. 매 제어 주기마다 짧은 구간의 궤적을 최적화하고, 첫 입력만 적용한 뒤 다시 최적화한다. Ch.6 §6.5의 MPC가 이 방식이다.

---

## 7.8 심화: Task and Motion Planning (TAMP)

"컵을 선반 위에 놓아라"라는 명령은 다음 단계로 나뉜다.

1. 컵이 어디 있는지 인식
2. 컵을 잡을 수 있는 grasp pose 결정
3. 접근 → 잡기 → 들기 → 이동 → 놓기 순서 계획
4. 각 단계의 모션 플래닝

1은 인식, 2는 연속 기하 결정(어느 grasp pose를 잡을지), 3은 symbolic planning(어떤 순서로 어떤 action을 할지), 4는 motion planning(어떤 궤적으로 움직일지)이다. TAMP는 기호 계획과 모션 플래닝을 한 문제로 함께 푼다. 아래 PDDLStream은 2번처럼 연속 파라미터를 고르는 일을 stream으로 분리한다.

### PDDLStream

MIT에서 개발한 TAMP 프레임워크다(Garrett et al., 2020). PDDL(Planning Domain Definition Language)로 symbolic action을 정의하고, stream으로 연속 파라미터(grasp pose, placement pose)를 만든다.

### LLM 기반 Task Planning

LLM을 symbolic planner 자리에 쓰는 연구도 있다.

- SayCan (Google, CoRL 2022): LLM이 각 skill의 자연어 설명이 지시에 얼마나 맞는지 점수를 매기고, value function이 현재 상태에서 그 skill의 성공 가능성을 매긴다. 두 값의 곱이 가장 큰 skill을 고른다.
- Code as Policies (Google, arXiv 2022 / ICRA 2023): LLM이 자연어 명령을 받아 로봇 제어용 Python 코드를 생성한다.
- Inner Monologue (Google, CoRL 2022): 성공 감지, 장면 설명 같은 환경 피드백을 텍스트로 LLM에 다시 넣어 계획을 고친다.

이 시스템들은 LLM으로 high-level 순서를 정한다. 좁은 공간에서 하는 조작이나 정밀 조립 같은 기하학적 제약은 motion planner와 학습된 skill에 맡긴다.

여기서 다룬 TAMP는 환경과 행동이 결정론적이라고 가정한다. 동역학과 관측이 확률적인 경우는 §7.9에서 다룬다.

---

## 7.9 심화: 불확실성하 의사결정 (POMDP와 belief space planning)

§7.1~§7.8은 로봇이 자신의 상태와 환경을 정확히 안다고 가정했다. 실제 로봇은 노이즈 있는 센서로 일부 정보만 관측한다. 대칭 복도에서 자기가 어느 쪽에 있는지 모르는 로봇이나, 문이 열렸는지 반반으로 추정하는 로봇이 "현재 최선 추정 상태"만 보고 계획하면 잘못된 행동을 고를 수 있다. 이런 경우 belief(상태에 대한 사후 분포)를 놓고 계획한다. 이 절은 Thrun, Burgard, Fox의 *Probabilistic Robotics* §15.2·§16을 따른다. 절·표·식·그림 번호는 1999–2000년 초고 번호다. 2005년 MIT Press 출간본에서는 MDP가 14장, POMDP가 15장, QMDP·AMDP·MC-POMDP 같은 근사 기법이 16장이다.

### 7.9.1 도입: 세 패러다임

Goal·Pit·Robot이 놓인 좌우 대칭 복도에서 세 종류의 플래너는 서로 다른 답을 낸다.

Classical planning은 상태를 완전히 알고 행동 결과도 결정론적이라고 가정한다. §7.3의 A*가 이 범주로, 최단 경로를 한 번 계산하면 실행 중 센싱이 필요 없다.

MDP(Markov Decision Process): 상태는 완전히 관측되고 행동 결과는 확률적이다. 정책 $\pi: s \to a$가 모든 상태에 행동을 하나씩 정해 준다. 좁은 길에서 벽과 충돌 위험을 고려해 더 넓은 경로를 택할 수 있다. Ch.8 §8.2가 이 범주이다.

POMDP: 행동 결과와 관측이 모두 확률적이다. belief $b$ 위에 정책 $\pi: b \to a$를 정의한다. 대칭 복도에서 로봇은 처음에 자기 위치를 모르므로, 비대칭 영역으로 우회해 정보를 모은 뒤 목표로 향한다. 이 행동을 능동적 정보 수집(active information gathering)이라 한다.

세 패러다임은 classical $\subset$ MDP $\subset$ POMDP 순으로 포함된다. MDP는 행동 불확실성(가려던 곳과 실제로 간 곳의 차이)을 다룬다. POMDP는 여기에 지각 불확실성(실제 위치와 센서 값의 차이)까지 다룬다.

Ch.3의 필터는 belief를 추적했다. 여기서는 그 belief를 보고 행동을 고른다.

<!-- DEMO: pomdp_three_paradigms.html -->

### 7.9.2 belief 위 가치 반복

MDP 가치 반복의 Bellman 방정식은 다음과 같다.

$$C^T(s) = \max_a \int \left[ c(s') + C^{T-1}(s') \right] P(s' \mid a, s)\, ds'$$

이 절은 PR 15·16장의 기호를 따른다. $c$는 payoff(즉시 보상), $C^T$는 남은 horizon이 $T$일 때의 가치 함수, $P(s' \mid a, s)$는 전이 확률이고, 각각 Ch.8 §8.2의 $r$, $V$, $p(x' \mid x, a)$에 해당한다.

상태 $s$를 belief $b$로 바꾸면 POMDP의 가치 반복이 된다:

$$C^T(b) = \max_a \int \left[ c(b') + C^{T-1}(b') \right] P(b' \mid a, b)\, db' \tag{16.2}$$

정책은:

$$\pi^T(b) = \arg\max_a \int \left[ c(b') + C^{T-1}(b') \right] P(b' \mid a, b)\, db' \tag{16.3}$$

$b$와 $b'$는 모두 상태 공간 $\mathcal{S}$ 위의 확률 분포다. 위 식의 belief 전이 분포 $P(b' \mid a,b)$는 분포들의 공간 위에 정의되고, 적분도 이 belief 공간 전체에 걸친다. belief 공간은 유한 상태 공간에서 $|\mathcal{S}|-1$차원이다. 연속 상태 공간에서는 무한차원이 된다.

무한 horizon 극한에서 이 재귀가 수렴하면 표준 Bellman 방정식을 얻는다:

$$V(b) = \max_a \left[ r(b, a) + \gamma \sum_{o'} P(o' \mid a, b)\, V(B(b, a, o')) \right]$$

여기서 $r(b,a) = \sum_s b(s)\, c(s,a)$는 belief $b$에서 행동 $a$의 기대 즉시 보상이다. 유한 horizon 재귀 형태로 쓰면 식 (16.2)가 된다.

관측 $o'$가 결정되면 사후 belief $B(b, a, o')$가 Bayes 필터로 *유일하게* 결정된다. 이 점을 이용하면 belief 공간 전체 적분을 관측 공간 위 적분으로 재구성할 수 있다:

$$C^T(b) = \max_a \int \left[ c(B(b, a, o')) + C^{T-1}(B(b, a, o')) \right] P(o' \mid a, b)\, do' \tag{16.34}$$

belief update operator는:

$$B(b, a, o')(s') = \frac{1}{P(o' \mid a, b)}\, P(o' \mid s') \int P(s' \mid a, s)\, b(s)\, ds$$

이산 상태·관측 공간에서는 적분이 합으로 바뀐다. 이 재구성이 아래 alpha-vector 해법의 출발점이다.

### 7.9.3 4상태 toy 예시

PWLC(piecewise-linear convex) 구조를 4상태·2행동·2관측 문제로 손으로 계산해 본다.

**설정:**
- 상태 $s_1, s_2, s_3, s_4$. 초기에 $(s_1, s_2)$ 중 하나.
- 행동 $a_1$: 정보 수집. $s_1 \leftrightarrow s_2$를 0.9 확률로 교환.
- 행동 $a_2$: 종결. $s_3$(보상 +100) 또는 $s_4$(보상 -100)로 이동한다. $s_1$에서는 0.9 확률로 $s_3$, $s_2$에서는 0.9 확률로 $s_4$에 가므로 기대 보상은 각각 +80, -80이다.
- 관측 $o_1, o_2$: $s_1$에서 확률 (0.7, 0.3), $s_2$에서 확률 (0.4, 0.6).
- belief는 $b = (p_1, p_2)$이고 $p_1 + p_2 = 1$이므로 1차원.

**horizon 1 계산:**

즉시 보상은 belief에 대해 선형이다: $c(b) = \sum_i c(s_i) p_i$.

$a_2$를 택하면 $T=1$ 가치 ($\gamma = 0.9$):
$$C^1(b, a_2) = \gamma(80 p_1 - 80 p_2) = 72 p_1 - 72 p_2$$

$a_1$로는 보상이 있는 상태($s_3$, $s_4$)에 가지 않으므로 $C^1(b, a_1) = 0$이다.

따라서:
$$C^1(b) = \max\{ 0,\; 72p_1 - 72p_2 \}$$

$C^1(b)$는 두 선형 함수의 max다. $p_1 = 0.5$에서 꺾인다. $p_1 > 0.5$이면 $a_2$, 아니면 $a_1$.

**horizon 2 계산:**

$a_1$ 후 관측 $o_1, o_2$가 올 확률을 적분하면:
$$C^2(b, a_1) = \max\{0,\; -18.79 p_1 + 38.23 p_2\} + \max\{0,\; -33.05 p_1 + 13.61 p_2\} = \max\{0,\; -18.79 p_1 + 38.23 p_2,\; -51.84 p_1 + 51.84 p_2\}$$

(계수는 관측 확률과 belief update로 계산. 관측별 선택을 조합한 네 선형 함수 가운데 $-33.05 p_1 + 13.61 p_2$는 모든 belief에서 $-18.79 p_1 + 38.23 p_2$보다 작아 가지치기된다. PR 원 예제는 $a_1$의 교환 확률이 $s_1$에서 0.9, $s_2$에서 0.8이라 계수가 다르고, 식 (16.27)은 $C^2(b) = \max\{0,\; -33.048 p_1 + 7.776 p_2,\; 72 p_1 - 72 p_2\}$이다. 여기서는 교환 확률을 둘 다 0.9로 단순화해 다시 계산했다.)

$T=2$ 전체($a_2$ 조각까지 넣으면 상수 0 조각도 지배된다):
$$C^2(b) = \max\{ -51.84 p_1 + 51.84 p_2,\; -18.79 p_1 + 38.23 p_2,\; 72 p_1 - 72 p_2 \}$$

세 선형 조각의 max다. horizon이 늘면 조각이 더 생긴다.

가치 함수는 belief 공간에서 볼록(convex)이다: $\beta C(b) + (1-\beta) C(b') \geq C(\beta b + (1-\beta) b')$. 따라서 두 belief를 섞은 belief의 가치는 두 가치의 가중평균 이하다.

<!-- DEMO: pomdp_toy_pwlc.html -->

### 7.9.4 PWLC 구조와 alpha-vectors

4상태 예제의 가치 함수는 선형 조각의 max였다. 이 형태가 모든 horizon에서 유지됨을 귀납으로 보인다.

베이스 케이스 ($T=1$): 즉시 보상 $c(b) = \sum_i c(s_i) p_i$는 belief에 대해 선형이다. Thrun et al. Table 16.1처럼 보상이 상태에만 달려 있으면 $C^1(b) = \gamma \sum_i c(s_i)\, p_i$로, 선형 함수 하나다(Table 16.1 2행, 경계 조건 $|\Phi^1| = 1$). 앞의 4상태 예제는 $a_2$를 택할 때만 보상 상태로 가므로 $C^1(b)$가 행동별 선형 함수 두 개의 max다(식 16.11). 어느 경우든 $C^1(b)$는 PWLC다.

귀납 단계: $C^{T-1}(b)$가 PWLC라고 하자. 식 (16.34)에서 $C^{T-1}(B(b,a,o'))$를 $b$의 함수로 전개하면: belief update의 비선형 정규화 인자 $1/P(o'\mid a, b)$가 식 (16.34)의 가중치 $P(o'\mid a, b)$와 상쇄되어, 각 alpha-vector와의 내적 $\langle \phi, B(b,a,o') \rangle \cdot P(o'\mid a, b)$이 $b$의 선형 함수로 정리된다. 선형 함수들의 max를 관측별로 더하고 다시 행동별로 max를 취해도 선형 함수들의 max다. 따라서 $C^T(b)$도 PWLC.

각 선형 조각의 계수 벡터를 alpha-vector $\phi$라 한다. 가치 함수는:

$$V(b) = \max_\phi \langle \phi, b \rangle$$

$\Phi$가 alpha-vector 집합이면 $V(b) = \max_{\phi \in \Phi} \sum_i \phi_i\, p_i$.

각 alpha-vector는 하나의 *조건부 정책*(현재 행동 + 관측에 따른 후속 정책)에 대응한다. 가지치기 전 후보 수는 $|\Phi^T| = |A| \cdot |\Phi^{T-1}|^{|\mathcal{O}|}$으로 이중 지수적으로 증가한다. 행동과 관측이 각각 2개이면, $|\Phi^1| = 1$에서 시작해 $|\Phi^2| = 2 \cdot 1^2 = 2$, $|\Phi^3| = 2 \cdot 2^2 = 8$, $|\Phi^4| = 2 \cdot 8^2 = 128$, $|\Phi^5| = 2 \cdot 128^2 = 32768$이다. 그래서 긴 horizon에서는 정확 해법을 쓰기 어렵다.

### 7.9.5 LP 해법

정확 해법은 max·sum·max 구조를 LP(linear program)로 바꿔, 지배되는 alpha-vector를 판별하고 가지치기한다.

변환 원리: $C = \max_a x(a)$는 $\{C \geq x(a) \;\forall a\}$ 제약에서 $\min C$로 풀린다. $C = \sum_i \max_a x(a,i)$는 각 $i$마다 행동을 선택하는 함수 $a(\cdot)$의 모든 조합에 대해 $\{C \geq \sum_i x(a(i),i)\}$ 제약을 만들면 된다. 제약 수는 $|A|^{|\mathcal{S}|}$.

POMDP horizon $T$의 제약 (식 16.67):

$$\bigcup_a \bigcup_{k(o'):1 \leq k(o') \leq |\Phi^{T-1}|} \left\{ C^T(b) \geq \gamma \sum_{o'} \sum_i \left(c_i + C^{T-1}_{k(o'),i}\right) P(o' \mid s_i') \sum_j P(s_i' \mid a, s_j)\, p_j \right\}$$

가지치기 전 제약 수는 $|\Phi^T| = |A| \cdot |\Phi^{T-1}|^{|\mathcal{O}|}$.

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
  # Table 16.1에는 없는 줄. 본문이 설명하는 가지치기를 덧붙였다
  Φᵀ = prune(Φᵀ)
  return Φᵀ
```

---

$|\Phi^1|=1$에서 시작하면 행동 3개, 관측 5개, horizon 4에서 가지치기 전 alpha-vector가 $3 \cdot 729^5 \approx 6 \times 10^{14}$개다($|\Phi^2|=3$, $|\Phi^3|=3 \cdot 3^5=729$). 현실적인 도메인에서는 pruning을 해도 계산량이 감당할 수준을 넘으므로 근사 기법을 쓴다.

### 7.9.6 일반 POMDP

상태 공간이 연속이면 alpha-vector 표현도 연속 함수가 된다. 식 (16.34)는 원칙적으로 여전히 성립하지만, $\Phi^{T-1}$가 함수들의 집합으로 무한차원이 된다.

---

**알고리즘: POMDP(T)** (Thrun et al., 식 (16.34)를 Table 16.1 꼴로 정리)

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

연속 공간에서는 함수들의 집합을 저장하고 비교하는 비용이 너무 크다. 이 알고리즘은 원리를 보이는 용도다. 실제로는 MC-POMDP나 AMDP를 쓴다.

### 7.9.7 MC-POMDP

particle filter로 belief를 표현하고 가치 반복 갱신을 표본 기반으로 근사한다. Ch.3 §3.11.3에서 추정에 쓴 입자 필터를 여기서는 플래닝에 쓴다.

belief $\theta$는 가중 입자 집합 $\langle s^{(i)}, w^{(i)} \rangle$이다. belief update $B(b, a, o')$를 입자 형태로 구현한다:

```
Algorithm particle_filter_belief_update(θ, a, o'):   # Thrun et al., Table 16.3 의역
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

**알고리즘: MC-POMDP** (Thrun et al., §16.4 서술을 정리)

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

$R(o_{t+1})$은 다음 시점에 받는 payoff다.

정책 (식 16.79):

$$\sigma^Q(\theta) = \arg\max_{\bar{a}} Q(\theta, \bar{a})$$

Q값 함수 근사는 nearest-neighbor 방식을 쓴다. belief $\theta$가 순서가 없는 입자 집합이라, 고정 순서의 벡터를 받는 당시의 feedforward 네트워크에 그대로 넣을 수 없었다. Thrun et al.은 $\langle \theta, a, Q \rangle$ 데이터베이스를 유지하고, 새 belief $\theta'$가 들어오면 KL divergence로 $k$-nearest neighbor를 찾아 Q값 평균을 쓴다. 오늘날에는 집합을 다루는 인코더로 신경망 입력을 만들 수도 있다.

두 belief 사이의 KL divergence는 Gaussian KDE로 근사한다.

Outer loop는 belief 데이터베이스를 정적으로 유지하거나, $\varepsilon$-greedy 시뮬레이션 trial을 돌리며 방문한 belief를 쌓는다. 후자는 실제 로봇이 마주칠 belief에 계산을 모은다.

### 7.9.8 실험: heaven/hell과 find-and-fetch

Heaven/Hell 문제: 2차원 합성 환경에서 에이전트는 왼쪽 아래 구석에서 출발한다. 천국(heaven)은 왼쪽 위나 오른쪽 아래 구석에 있고, 반대쪽 구석은 지옥(hell)이다. 에이전트는 천국이 어디인지 모르지만 오른쪽 위 구석의 사제(priest)에게 물어볼 수 있다. 그래서 최적 해는 먼저 사제에게 가서 정보를 얻고 천국으로 가는 것이다. 에이전트는 자기 좌표도 관측하지 못하고, 매 이동에 5% 무작위 잡음이 들어간다. 13번의 실험 모두에서 MC-POMDP는 17 episode 안에 늘 천국에 도착하는 정책을 찾았다.

Find-and-Fetch (단안 카메라): 시뮬레이션 속 RWI B21 로봇이 주변 어딘가(바닥이나 탁자 높이)에 있는 물체를 찾아 그리퍼로 잡는 태스크다. 단안 카메라로는 한 자리에서 본 영상만으로 물체의 3차원 위치를 정할 수 없고, 처음에는 물체가 시야 밖에 있을 수도 있다. 약 4,000번 반복한 뒤 잡기 성공률이 80%를 넘었다. 성공한 궤적에서 로봇은 먼저 물체를 찾아 둘러본 뒤 다가가 잡았다.

두 실험 모두 belief를 추적하며 *정보 수집 행동*을 계획에 넣는다.

### 7.9.9 AMDP — belief 통계로 차원 축소

AMDP(Augmented MDP)는 belief를 적은 수의 통계량으로 요약한다. 계산량은 MDP($|S|$에 polynomial)와 정확 POMDP(이중지수) 사이에 있다.

아이디어: 실제 로봇 궤적에서 belief는 belief 공간의 좁은 manifold 위에 모여 있다. 그 manifold를 *저차원 통계* $\bar{b} = f(b)$로 요약하고, $\bar{b}$ 위에서 표준 MDP 가치 반복을 적용한다.

**흔히 쓰는 통계** (식 16.80):

$$\bar{b} = \langle \arg\max_s b(s),\; H[b] \rangle$$

가능성이 가장 높은 상태와 belief의 엔트로피를 쓴다. 엔트로피는:

$$H[b] = -\int b(s) \ln b(s)\, ds \tag{16.81}$$

무한 차원 belief를 최빈 상태와 엔트로피의 쌍으로 요약한다. Thrun et al.은 이 쌍이 충분 통계라는 가정이 거의 성립하지 않는다고 적었다. 그래도 coastal navigation 실험에서는 이 요약으로 적절한 경로를 골랐다.

$\arg\max_s b(s)$만 쓰면 표준 MDP다. AMDP는 여기에 엔트로피를 더해 불확실성의 크기를 상태에 넣는다.

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

MDP_value_iteration (원전 Probabilistic Robotics §15.3.3)에서 상태 $s$를 $\bar{b}$로 바꾼 형태다.

전이 확률 $P(\bar{b}' \mid a, \bar{b})$ 계산 (식 16.85):

$$P(\bar{b}' \mid a, \bar{b}) = \int\!\!\int\!\!\int I_{f(b)=\bar{b}}\, I_{f(B(o',a,b))=\bar{b}'}\, P(o' \mid s') P(s' \mid a, s) P(s \mid b)\, ds\, ds'\, do'\, db$$

구현에서는 여러 번의 랜덤 시뮬레이션으로 전이 확률을 추정하고 lookup table에 저장한다.

### 7.9.10 Coastal Navigation 예시

넓은 open space를 가로지를 때 conventional MDP planner는 거리가 짧은 직선 경로를 택한다. open space 한가운데서는 센서 범위 안에 특징이 거의 들어오지 않아 위치 belief의 엔트로피가 커진다. 목적지에 도착했을 때 로봇은 자기 위치를 정확히 모른다.

AMDP planner는 같은 환경에서 장애물 가까이 지나는 경로를 고른다. 이동 시간은 늘지만 장애물 근처에서는 센서 측정이 위치를 잘 잡아 주어 엔트로피가 낮게 유지된다. 비용 함수에 엔트로피가 들어 있으므로 planner는 정보가 많은 경로를 고른다. 이름은 위치를 잃지 않으려고 해안선(coast) 가까이 항해하는 배에서 왔다.

Thrun et al.의 그림 16.5는 폭 약 40m인 박물관의 넓은 공간에서 두 planner가 만든 경로를 보여 준다. 그림 16.6은 레이저 거리 센서의 최대 거리를 바꿔 가며 목표 지점에서 잰 belief 엔트로피를 비교한다. 센서 거리가 짧을수록 coastal planner가 더 유리하고, 센서 거리가 길면 두 planner의 차이가 사라진다.

Active SLAM은 위치 불확실성을 줄이는 경로를 고른다. next-best-view planning은 정보량이 큰 시점으로 이동한다. 둘 다 coastal navigation과 같은 원리를 쓴다.

<!-- DEMO: coastal_navigation_amdp.html -->

### 7.9.11 무엇이 살아남았나

이후 POMDP solver는 다음 방향으로 발전했다.

Point-based value iteration(PBVI, HSVI, SARSOP)은 샘플링한 belief 점에서만 alpha-vector backup을 한다. §7.9.4의 alpha-vector 구조를 그대로 쓰고 backup할 점의 수를 제한해 계산량을 줄인다.

MCTS 계열: POMCP(Silver & Veness, 2010), DESPOT. rollout으로 Q값을 추정하고 belief tree를 MCTS로 탐색한다. §7.9.7 MC-POMDP의 Q 추정 구조를 tree search에 결합했다.

Deep POMDP: DRQN(Deep Recurrent Q-Network), DVRL(Igl et al.). RNN의 hidden state가 implicit belief 역할을 하고, Q 함수는 신경망으로 근사한다.

상태에 불확실성 정보를 더하는 방식은 다른 연구에도 있다. Bayes-adaptive MDP(BAMDP)는 모델 파라미터의 사후 분포를 상태에 넣는다. Active SLAM은 위치 belief의 분산 지표를 비용 함수에 넣는다.

Ch.8 §8.3의 PPO·SAC 같은 model-free deep RL은 경험만으로 학습한다. POMDP planning은 전이 확률과 관측 모델을 알 때 최적 정책을 계산한다. MC-POMDP는 둘 사이에 있다. belief는 모델로 추적하고, Q값은 시뮬레이션 경험에서 학습한다.

---

## 7.10 추천 자료

> **LaValle, "Planning Algorithms"**
> https://lavalle.pl/planning/
> 무료 온라인. RRT를 제안한 LaValle의 모션 플래닝 교과서.

> **Russ Tedrake, "Underactuated Robotics" Ch.10: Trajectory Optimization**
> https://underactuated.csail.mit.edu/trajopt.html
> Drake로 하는 trajectory optimization 실습. 코드와 이론을 함께 다룬다.

> **Matthew Kelly, "An Introduction to Trajectory Optimization" (SIAM Review 2017)**
> https://www.matthewpeterkelly.com/research/MatthewKelly_IntroTrajectoryOptimization_SIAM_Review_2017.pdf
> Direct collocation 튜토리얼. 예제 코드가 함께 있다.

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
> Nonlinear trajectory optimization 구현에 널리 쓰는 도구.

> **추가 논문**
> - [Garrett et al., "Integrated Task and Motion Planning" (Annual Review of Control, Robotics, and Autonomous Systems 2021, arXiv:2010.01083)](https://arxiv.org/abs/2010.01083) — TAMP 서베이 논문
> - [Janner et al., "Planning with Diffusion for Flexible Behavior Synthesis" (ICML 2022, arXiv:2205.09991)](https://arxiv.org/abs/2205.09991) — 궤적 전체를 diffusion 모델로 생성해 계획하는 방법

---

## 기술 흐름

```
1979 ── Lozano-Pérez & Wesley, visibility graph 기반 경로 계획 (CACM)
1996 ── PRM (Kavraki et al.) — 샘플링 기반 로드맵
1998 ── RRT (LaValle) — single-query sampling planner
2000 ── RRT-Connect (Kuffner & LaValle) — OMPL 같은 motion-planning 라이브러리에 들어 있는 변종
2009 ── CHOMP (Ratliff et al.) — gradient 기반 궤적 최적화
2011 ── RRT* (Karaman & Frazzoli) — 점근적 최적성 보장
2012 ── OMPL 소개 논문 (IEEE RAM) — 샘플링 기반 플래너 통합 라이브러리 (1.0 릴리스는 2014)
2014 ── TrajOpt (Schulman et al.) — sequential convex optimization
2014 ── Contact-implicit trajectory optimization (Posa, Cantu & Tedrake, IJRR) — 접촉 모드 순서를 미리 정하지 않는 직접법
2019 ── MoveIt 2 alpha 릴리스 (ROS 2 Dashing)
2022 ── SayCan (Google) — LLM 점수와 skill 가치 함수를 곱해 실행할 skill 선택
```
