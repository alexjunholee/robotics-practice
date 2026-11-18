# 7장 모션 플래닝과 궤적 최적화

6장에서 다룬 제어기가 주어진 목표나 궤적을 정확히 따라가는 역할을 한다면, 7장은 애초에 로봇이 따라갈 **충돌 없는 경로와 물리적으로 실행 가능한 궤적**을 만드는 문제를 다룬다. 로봇이 A 지점에서 B 지점으로 이동하려면 주변 장애물을 피해야 할 뿐 아니라 관절 가동 범위와 모터의 토크·속도 한계까지 모두 만족해야 한다. 이 장에서는 먼저 기하학적으로 부딪히지 않는 경로를 찾는 **모션 플래닝(motion planning)**으로 움직임의 큰 뼈대를 잡은 뒤, 여기에 시간 정보와 동역학 제약, 비용 함수를 반영해 부드럽고 효율적인 움직임으로 다듬는 **궤적 최적화(trajectory optimization)**까지 차근차근 살펴본다.

---

## 7.1 모션 플래닝이 쓰이는 곳

6축 로봇 팔에 "테이블 위의 컵을 집어라"라는 명령을 내렸다고 하자. 역기구학(IK)을 풀면 목표 지점에 닿는 관절 각도는 금방 구할 수 있다. 하지만 현재 관절 각도에서 목표 관절 각도까지 단순한 직선 보간(interpolation)으로 움직이면 팔이 테이블 모서리를 들이받거나 자기 몸체와 충돌하기 십상이다. 관절 공간에서 직선으로 움직이더라도 3차원 작업 공간에서는 로봇 링크들이 크게 휘는 곡선 궤적을 그리기 때문이다. 반대로 작업 공간에서 손끝만 직선으로 옮기려 해도 중간에 관절 한계나 특이점(singularity)에 걸릴 수 있다.

모션 플래닝은 바로 이 지점에서 다음 세 가지 핵심 질문에 답한다.
- 장애물이나 자기 몸체와 부딪히지 않고 목표 자세까지 갈 수 있는 경로가 실제로 존재하는가?
- 그런 경로가 여러 개라면, 그중 가장 짧거나 빠르고 움직임이 부드러운 경로는 무엇인가?
- 로봇의 토크 한계와 속도·가속도 제한 같은 동역학 제약을 지키면서 그 경로를 따라갈 수 있는가?

이 질문들을 수학적으로 다루려면, 먼저 부피가 있는 로봇 팔의 자세와 충돌 여부를 한눈에 표현할 수 있는 좌표 공간부터 정의해야 한다.

---

## 7.2 Configuration Space (C-space)

3차원 작업 공간(workspace)에서 로봇은 여러 개의 링크가 관절로 이어진 부피 있는 물체다. 작업 공간 안에서 팔 전체가 장애물과 부딪히는지 따지려면 모든 링크의 3차원 위치와 자세를 매번 따로 검사해야 해서 문제를 다루기가 까다롭다. 발상을 뒤집어 로봇이 취할 수 있는 모든 자세를 하나의 추상적인 공간 속 **점(point)**으로 압축하면 어떨까?

$n$자유도(n-DOF) 로봇의 자세는 $n$개의 관절 값 $q = (q_1, q_2, \dots, q_n)$으로 완전히 결정된다(관절 공간, joint space). 이 벡터 $q$가 놓이는 $n$차원 공간 전체를 **형상 공간(Configuration Space, C-space)**이라 부른다. 작업 공간의 장애물이나 자기 몸체와 충돌을 일으키는 형상 $q$들을 모두 모아 놓은 영역이 **C-space obstacle**이고, 반대로 충돌이 전혀 없는 안전한 영역이 free C-space다.

C-space로 문제를 옮기면 거대하고 복잡한 로봇 팔의 충돌 회피 문제가 "장애물 영역을 피해 C-space 안의 한 점을 시작점에서 목표점까지 이동시키는 경로 탐색 문제"로 깔끔하게 바뀐다. 로봇의 부피는 이미 C-space obstacle의 경계 안에 흡수되어 있기 때문이다.

다만 C-space obstacle의 정확한 기하학적 경계는 고차원 비선형 변환(정기구학, FK)을 거쳐야 해서 해석적으로 미리 구하기가 매우 어렵다. 그래서 실무에서는 C-space 장애물 지도를 통째로 계산해 두는 대신, 후보 형상 $q$ 하나를 넘기면 FK로 각 링크의 3D 포즈를 계산한 뒤 충돌 여부만 참·거짓으로 판정해 주는 **collision checker**를 반복 호출하는 방식을 쓴다.

이제 C-space 위에서 점 하나를 움직여 경로를 찾으면 된다. 가장 먼저 떠올릴 수 있는 방법은 공간을 촘촘한 격자(grid)나 그래프로 나눈 뒤 고전적인 그래프 탐색 알고리즘을 적용하는 전략이다.

---

## 7.3 그래프 탐색 기반 플래닝

연속적인 C-space를 일정 간격으로 이산화(discretize)하면 각 상태가 노드(vertex), 인접 상태로의 이동이 간선(edge)인 그래프가 된다. 여기에 최단 경로 탐색 알고리즘을 적용하면 충돌 없는 경로를 체계적으로 찾아낼 수 있다.

### Dijkstra 알고리즘

Dijkstra 알고리즘은 간선 가중치가 음수가 아닌 그래프에서 시작 노드부터 다른 노드까지의 최단 경로를 보장한다. 우선순위 큐(binary heap)와 인접 리스트(adjacency list)를 함께 쓰면 시간 복잡도는 $O((V+E)\log V)$이며, 목표 노드가 큐에서 꺼내져 최단 거리가 확정되는 순간 전체 그래프를 끝까지 돌지 않고도 탐색을 멈출 수 있다. 다만 목표 지점이 어느 방향에 있는지 전혀 고려하지 않고 시작점 주변부터 동심원 형태로 넓혀 나가므로, 고차원이나 넓은 맵에서는 불필요한 노드를 많이 방문한다.

### A* 알고리즘

A*는 Dijkstra의 누적 비용 $g(n)$에 목표 지점까지의 추정 비용(heuristic) $h(n)$을 더한 $f(n) = g(n) + h(n)$ 기준으로 노드를 꺼내 탐색 방향을 목표 쪽으로 바짝 끌어당긴다. 휴리스틱 $h(n)$이 실제 남은 비용을 절대 과대평가하지 않고(admissible) 삼각 부등식을 만족하면(consistent), graph-search A*는 여전히 최적 경로를 보장하면서도 탐색하는 노드 수를 크게 줄여 준다. 물론 휴리스틱 자체를 계산하는 비용이 지나치게 무겁다면 탐색 노드 수가 줄어도 전체 실행 시간은 오히려 Dijkstra보다 길어질 수 있으므로, 계산이 가벼운 거리 함수를 고르는 균형이 중요하다.

아래 코드는 0(빈 칸)과 1(장애물)로 이루어진 2차원 격자에서 대각선 이동까지 허용하고, 유클리드 거리를 휴리스틱으로 써서 최단 경로를 찾는 기본 A* 구현이다.

```python
import heapq
import numpy as np

def astar_2d(grid, start, goal):
    """2D 격자에서 A* 경로 탐색.
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

유한 격자에서 도달 가능한 셀을 빠짐없이 살피는 그래프 탐색은 그 이산화된 문제 안에서 **완전성(completeness)**을 가진다. 즉, 격자 위에 해가 존재하기만 하면 반드시 찾아낸다. 그러나 격자 해상도보다 좁은 통로는 격자 칸이 장애물로 채워지면서 지도상에서 통째로 사라져 버린다(해상도 완전성, resolution completeness의 한계).

더 결정적인 병목은 **차원의 저주(curse of dimensionality)**다. 관절 수가 늘어날수록 격자 셀 수가 기하급수적으로 불어나기 때문이다. 예를 들어 6-DOF 로봇 팔의 C-space를 각 축마다 100칸으로만 나누어도 전체 셀 수는 $100^6 = 10^{12}$개에 달한다. 이처럼 거대한 그래프는 메모리에 올리는 것조차 불가능하므로, 다관절 로봇 팔 같은 고차원 C-space에서는 공간을 미리 잘게 쪼개는 대신 무작위로 표본을 뽑아 연결하는 **샘플링 기반 플래너**로 방향을 튼다.

---

## 7.4 샘플링 기반 플래너

고차원 C-space를 통째로 격자로 쪼개는 대신, 공간에서 무작위로 점(표본)을 뽑아 충돌이 없는 점들을 선분으로 이어 나가면 계산량을 크게 줄일 수 있다. 이 방식은 격자처럼 모든 공간을 샅샅이 뒤지지는 못하지만, 샘플 수를 늘리면 해가 존재할 때 결국 찾아낸다는 **확률적 완전성(probabilistic completeness)**을 만족하며, 필요에 따라 최적화 기반 플래닝이나 그래프 탐색과 결합해 쓰기도 한다.

### RRT (Rapidly-exploring Random Tree)

LaValle(1998)이 제안한 RRT는 시작점에서 출발해 빈 공간 쪽으로 빠르게 뻗어 나가는 트리를 키우는 알고리즘이다. 핵심은 트리에 없는 영역에서 무작위 점 $q_{\text{rand}}$를 뽑을수록, 기존 트리에서 가장 가까운 노드 $q_{\text{near}}$가 아직 개척되지 않은 넓은 공간 쪽으로 당겨진다는 점이다(Voronoi bias).

```
1. 시작점에서 트리를 초기화한다.
2. C-space에서 무작위 점 q_rand를 샘플링한다.
3. 트리에서 q_rand에 가장 가까운 노드 q_near를 찾는다.
4. q_near에서 q_rand 방향으로 step_size만큼 확장하여 q_new를 만든다.
5. q_near → q_new 경로가 충돌하지 않으면 트리에 추가한다.
6. q_new가 목표 근처면 종료. 아니면 2로 돌아간다.
```

실제 구현에서는 순수 무작위 샘플만 뽑으면 목표 근처까지 다가가기까지 시간이 오래 걸리므로, 일정 확률(예: 10%)로 목표점을 직접 뽑아 트리가 목표 방향으로 수렴하도록 유도하는 goal bias를 함께 넣는다.

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

기본 RRT는 해를 빠르게 찾지만, 한 번 연결한 부모 노드를 바꾸지 않기 때문에 빙 돌아가는 비효율적인 경로가 나올 수 있다. Karaman & Frazzoli(2011)가 제안한 RRT*는 새 노드 $q_{\text{new}}$를 추가할 때 주변 반경 안의 기존 노드들을 살펴 가장 비용이 적은 부모를 고르고, 반대로 주변 노드가 $q_{\text{new}}$를 거쳐 가는 편이 더 짧다면 부모를 다시 잇는 **재배선(rewiring)** 과정을 거친다. 덕분에 샘플 수가 늘어날수록 최적 경로에 수렴하는 **점근적 최적성(asymptotic optimality)**을 얻는다.

물론 공짜는 아니다. 매번 반경 내 이웃을 탐색하고 충돌 검사를 다시 수행해야 하므로 기본 RRT보다 계산량이 크다. 그래서 실시간 응답이 빠듯한 현장에서는 아래의 RRT-Connect로 일단 실행 가능한 경로를 재빨리 확보한 뒤, 후처리(path shortcutting이나 궤적 최적화)로 경로를 다듬는 전략이 흔히 쓰인다.

### PRM (Probabilistic Roadmap)

Kavraki et al.(1996)이 제안한 PRM은 RRT와 달리 **다중 질의(multi-query)** 환경에 맞춘 알고리즘이다. RRT는 시작점과 목표점이 바뀔 때마다 트리를 새로 키우는 single-query 방식이지만, 작업장 구조가 고정된 산업용 로봇 셀처럼 같은 환경에서 시작-목표 쌍만 바꿔 가며 경로를 반복 요청해야 한다면 매번 공간을 새로 탐색하는 편이 비효율적이다.

PRM은 이 문제를 두 단계로 나누어 푼다.
1. **오프라인 단계(roadmap 구축)**: C-space에 수많은 점을 미리 뿌려 충돌 없는 점만 남긴 뒤, 가까운 점들끼리 충돌 없는 간선으로 이어 지도(그래프)를 한 번 만들어 둔다.
2. **온라인 단계(경로 질의)**: 실제 요청이 들어오면 현재 시작점과 목표점만 기존 로드맵에 연결하고, A* 같은 그래프 탐색으로 최단 경로를 즉시 찾아낸다.

### RRT-Connect

Kuffner & LaValle(2000)이 제안한 RRT-Connect는 single-query 탐색 속도를 극대화한 변종이다. 시작점에서만 트리를 키우는 대신 시작점과 목표점 양쪽에서 동시에 트리를 키워 나가다 두 트리가 마주치면 곧바로 경로를 이어 붙인다. 초기 실행 가능 경로를 매우 빠르게 찾아내기 때문에 실무에서 널리 쓰이며, MoveIt2의 여러 OMPL 설정 예시에서도 `RRTConnect`를 기본 planner config로 지정한다(실제 기본값은 배포판과 사용자 설정에 따라 달라진다).

### OMPL 라이브러리

지금까지 살펴본 알고리즘을 연구나 프로젝트에서 매번 밑바닥부터 구현할 필요는 없다. Rice 대학 Kavraki Lab에서 개발한 **OMPL(Open Motion Planning Library, https://ompl.kavrakilab.org/)**은 C++로 작성된 표준 오픈소스 라이브러리로, RRT, RRT*, RRT-Connect, PRM, EST, KPIECE 등 수십 가지 샘플링 기반 플래너를 일관된 인터페이스로 제공한다.

OMPL은 기구학이나 충돌 검사 자체를 직접 수행하지 않고, 사용자가 넘겨주는 `StateValidityChecker` 콜백으로 특정 형상 $q$의 충돌 여부만 물어보는 구조다. MoveIt2는 바로 이 OMPL의 탐색 엔진에 FCL(Flexible Collision Library) 기반의 충돌 검사를 묶어 모션 플래닝 파이프라인을 구성한다. 아래는 ROS2 MoveIt2 Python API(`MoveItPy`)로 플래너를 호출해 궤적을 실행하는 간단한 흐름이다.

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

## 7.5 궤적 최적화

샘플링 기반 플래너로 충돌 없는 경로를 얻었다고 해서 로봇을 곧바로 움직일 수 있는 상태가 되지는 않는다. 무작위 탐색으로 이어 붙인 경로에는 다음과 같은 빈틈이 남기 때문이다.
- 무작위로 뽑은 경유점들을 직선으로 이은 탓에 꺾임이 많고 울퉁불퉁하다.
- 순수하게 기하학적 충돌 여부만 따졌을 뿐, 로봇의 질량·관성이나 모터 토크 한계 같은 동역학 제약은 반영되어 있지 않다.
- "어디를 지나는가"만 있을 뿐 "각 경유점을 몇 초에 어떤 속도와 가속도로 지나가는가"라는 시간 정보가 비어 있다.

**궤적 최적화(trajectory optimization)**는 바로 이 빈틈을 메운다. 시간·에너지·부드러움(예: 가속도나 저크 제곱) 같은 비용 함수를 최소화하는 동시에 동역학 방정식, 충돌 회피, 관절·토크 한계를 모두 만족하는 시간 기반 궤적 $x(t), u(t)$를 수치 최적화로 풀어낸다.

### Direct Collocation

궤적 최적화를 푸는 대표적인 방법이 **Direct Collocation**이다. 연속적인 궤적을 $N$개의 시간 구간으로 나눈 뒤, 각 시점의 상태 $x_k$와 제어 입력 $u_k$를 모두 최적화의 결정 변수(decision variable)로 올린다. 그리고 시스템의 물리 동역학은 인접한 두 시점 사이의 등식 제약(equality constraint)으로 묶는다.

```
minimize    Σ_k L(x_k, u_k) * dt                    (비용)
subject to  x_{k+1} = f(x_k, u_k)   for all k       (동역학)
            g(x_k) <= 0              for all k       (부등식 제약: 충돌, 관절 한계)
            x_0 = x_init                             (초기 조건)
            x_N = x_goal                             (종단 조건)
```

이렇게 정식화한 문제는 하나의 큰 비선형 계획 문제(Nonlinear Program, NLP)가 되며, IPOPT 같은 수치 최적화 솔버에 넘겨 한꺼번에 푼다. 동역학과 복잡한 제약을 단일 문제 안에서 깔끔하게 다루어 부드러운 궤적을 얻는다는 장점이 있지만, 문제가 비볼록(non-convex)이기 때문에 초기 추측(initial guess)에 따라 결과가 크게 달라지고 지역 최적해(local optimum)에 빠질 수 있다.

### Direct Shooting

Direct Collocation이 상태와 입력을 모두 변수로 두는 반면, **Direct Shooting**은 상태 $x_k$를 결정 변수에서 빼고 입력 시퀀스 $\{u_0, u_1, \dots, u_{N-1}\}$만 변수로 둔다. 각 시점의 상태는 초기 상태 $x_0$에서 시작해 동역학 시뮬레이션을 앞으로 돌리며 직접 계산한다.

결정 변수 수가 훨씬 적다는 장점이 있지만, 도립진자처럼 조그만 입력 변화에도 상태가 발산하기 쉬운 불안정한 시스템에서는 시뮬레이션 자체가 요동치면서 최적화의 기울기(gradient)가 폭발해 수렴이 어려워진다. 그래서 불안정한 비선형 동역학이나 긴 시간 구간을 다룰 때는 상태를 직접 변수로 붙잡아 두는 Direct Collocation이나 Multiple Shooting이 더 안정적이다.

### CHOMP (Covariant Hamiltonian Optimization for Motion Planning)

단순 직선 보간이나 RRT 경로처럼 거친 초기 궤적이 이미 있을 때, 이를 점진적으로 매끄럽게 펴고 장애물에서 밀어내는 데 특화된 방법도 있다. Ratliff et al.(2009)이 제안한 **CHOMP**는 초기 궤적에서 출발해 "장애물 충돌 비용 + 궤적 부드러움 비용"의 gradient를 따라 궤적을 반복 갱신한다. 이때 유클리드 gradient가 아니라 궤적의 속도와 가속도 구조를 반영한 공변 gradient(covariant gradient)를 쓰기 때문에, 한 점만 툭 튀어나오지 않고 궤적 전체가 탄력 있게 매끄러워진다. 구현이 비교적 단순하고 기존 궤적을 다듬기 좋지만, 역시 지역 최적해 기반이라 좁은 통로(narrow passage) 반대편으로 궤적을 크게 돌아가게 만드는 데는 한계가 있다.

### TrajOpt

Schulman et al.(2014)이 제안한 **TrajOpt**는 **순차 볼록 최적화(sequential convex optimization)** 전략을 쓴다. 매 반복마다 원래의 비선형 목적 함수와 제약을 현재 궤적 주변에서 선형·이차 함수로 근사해 볼록 최적화 문제로 바꾸어 푼다. 이때 근사가 지나치게 먼 영역에서 틀어지는 일을 막기 위해 신뢰 영역(trust region)으로 한 번에 움직일 수 있는 폭을 제한한다. 충돌 회피는 로봇 링크와 장애물 사이의 부호 거리(signed distance) 기반 비용과 그 gradient로 처리하므로 3차원 메쉬 장애물도 자연스럽게 다룬다. 다만 원 문제가 비볼록인 이상, TrajOpt 역시 전역 최적해가 아닌 지역 최적해를 찾는다.

### CasADi를 이용한 Trajectory Optimization

실제로 Direct Collocation 스타일의 궤적 최적화를 구현할 때는 자동 미분(automatic differentiation)과 NLP 솔버 연결을 대신해 주는 프레임워크를 쓰는 편이 훨씬 빠르다. **CasADi**는 기호 계산(symbolic computation), 자동 미분, 그리고 IPOPT·SNOPT 같은 솔버 인터페이스를 하나로 묶어 둔 도구다(이 밖에도 Drake나 JAX 기반 구현을 많이 활용한다).

아래 예제는 질량 1 kg인 물체를 5초($N=50$, $\Delta t=0.1\text{ s}$) 동안 위치 0, 속도 0에서 위치 1, 속도 0으로 옮기되, 속도는 $\pm 2\text{ m/s}$, 힘은 $\pm 5\text{ N}$ 이내로 제한하면서 제어 입력 제곱의 적분을 최소화하는 1차원 더블 인티그레이터 문제를 CasADi의 `Opti` 인터페이스로 푼 코드다.

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

앞에서 살펴본 샘플링 기반 플래너와 궤적 스무딩, 충돌 검사를 실제 로봇 팔에 하나하나 직접 엮으려면 손이 많이 간다. ROS2 생태계에서는 이 전체 흐름을 하나로 묶어 둔 오픈소스 프레임워크 **MoveIt2**가 사실상의 표준으로 쓰인다.

MoveIt2는 크게 세 계층으로 움직인다.
- **Planning Scene**: 로봇의 현재 관절 상태와 작업 환경의 3D 장애물 지도를 실시간으로 관리하며 모든 충돌 검사의 기준이 된다.
- **Planning Pipeline**: OMPL 같은 모션 플래너를 호출해 기하학적 경로를 뽑은 뒤, 경로 검증과 시간 매개변수화(time parameterization)를 거쳐 속도·가속도 제약에 맞는 실행 가능한 궤적으로 완성한다.
- **Move Group Interface**: 사용자가 목표 포즈를 지정하고 플래닝과 실행을 요청하는 상위 API다.

OMPL은 MoveIt2에서 가장 널리 쓰는 planning pipeline 플러그인이며, 아래와 같이 `ompl_planning.yaml` 파일에서 로봇 그룹별로 사용할 플래너 목록과 기본 플래너를 지정한다. 물론 필요에 따라 CHOMP나 Pilz 산업용 플래너 등 다른 파이프라인으로 갈아끼울 수도 있다.

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

실제 작업에서 MoveIt2가 어떻게 쓰이는지 감을 잡기 위해 대표적인 **Pick-and-Place 파이프라인**을 따라가 보자.
1. **물체 인식(Perception)**: 카메라나 깊이 센서로 대상 물체의 6-DoF 포즈를 추정한다.
2. **Grasp planning**: 물체의 기하학적 형태를 보고 충돌 없이 안정적으로 집을 수 있는 그리퍼 위치와 자세를 정한다.
3. **Approach trajectory**: 현재 자세에서 물체 바로 위 접근점(pre-grasp pose)까지 모션 플래닝으로 이동한다.
4. **Grasp**: 목표 지점으로 조심스럽게 내려가 그리퍼를 닫는다.
5. **Retreat trajectory**: 물체를 손에 쥔 채로 테이블에서 들어 올린다(이때 물체도 로봇의 일부로 Planning Scene에 붙여 충돌을 함께 검사한다).
6. **Place trajectory**: 놓을 위치 위까지 다시 모션 플래닝으로 이동한다.
7. **Release**: 물체를 내려놓고 그리퍼를 연다.

이 가운데 3·5·6단계의 이동 경로를 생성하면서 충돌 회피와 관절 한계를 자동으로 챙기는 일이 바로 MoveIt2의 주특기다.

---

## 7.7 Optimization-Based Planning [심화]

MoveIt2의 기본 파이프라인은 주로 팔이 장애물을 피해 빈 공간을 이동하는 기하학적 경로에 시간 프로파일을 얹는 구조다. 하지만 보행 로봇이 발을 내딛거나 로봇 팔이 물체를 밀고 당기는 것처럼 **동역학과 접촉력이 궤적 자체를 결정하는 문제**에서는 기하학적 경로 탐색만으로는 부족하다. 이때는 문제 전체를 제약 있는 비선형 최적화로 직접 세워 푸는 방법이 힘을 발휘한다.

### Constrained Nonlinear Optimization

실제 로봇의 전신 동역학과 물리 제약을 모두 녹여낸 궤적 최적화는 대체로 다음과 같은 **제약 비선형 최적화(constrained NLP)** 꼴로 정식화한다.

```
minimize    Σ L(x_k, u_k) + Φ(x_N)
subject to  x_{k+1} = f(x_k, u_k)           (동역학)
            h(x_k, u_k) = 0                  (등식 제약)
            g(x_k, u_k) <= 0                 (부등식 제약: 충돌, 토크 한계 등)
```

목적 함수는 구간별 비용 $L(x_k, u_k)$와 종단 비용 $\Phi(x_N)$의 합이며, 시스템 동역학과 기하학적 제약·토크 한계가 각각 등식과 부등식 제약으로 들어간다. 앞의 CasADi 예제처럼 이 문제는 주로 IPOPT(Interior Point Optimizer)나 SNOPT 같은 솔버로 푼다.

### Contact-Implicit Trajectory Optimization

보행 로봇이 걷거나 뛰고, 로봇 손이 물체를 돌려 잡을 때는 발이나 손가락이 환경과 닿았다 떨어지는 **접촉 모드(contact mode)**가 시시각각 바뀐다. 어느 발이 몇 초에 땅에 닿을지 사람이 미리 정해 주지 않고, 최적화 알고리즘이 접촉 순서와 타이밍까지 스스로 찾아내게 만드는 방법이 **Contact-Implicit Trajectory Optimization**이다.

핵심 아이디어는 접촉력 $F_n$을 결정 변수에 직접 넣고, 물리 법칙을 다음 세 줄의 **상보성 조건(complementarity constraint)**으로 적어 넣는 데 있다.

```
F_n >= 0                   (접촉력은 당기지 못함)
d >= 0                     (물체가 바닥 아래로 못 감)
F_n * d = 0                (떨어져 있으면 힘 0, 닿아 있으면 거리 0)
```

바닥과의 거리 $d$가 0보다 커서 떨어져 있으면 접촉력 $F_n$은 반드시 0이어야 하고, 반대로 밀어내는 힘 $F_n > 0$이 작용하려면 반드시 바닥에 닿아($d = 0$) 있어야 한다는 뜻이다. 이렇게 정식화한 문제는 **MPCC(Mathematical Program with Complementarity Constraints)**라 부른다. MPCC도 형식상 NLP의 한 종류지만, $F_n \cdot d = 0$이라는 상보성 제약 때문에 실행 가능 영역이 특이한 형태를 띠어(ill-posed) 일반 NLP 솔버에 그대로 넣으면 좀처럼 수렴하지 않는다.

Posa, Cantu & Tedrake(2014)는 이 난점을 돌파하기 위해 상보성 조건을 $F_n \cdot d \le 0$ 꼴로 바꾸고 SQP 솔버 SNOPT의 elastic mode를 활용해 풀었다. 수렴이 까다로운 문제에서는 아예 $F_n \cdot d \le \epsilon$으로 제약을 살짝 느슨하게 풀어 시작점을 잡은 뒤 $\epsilon$을 0까지 줄여 가는 연속화(homotopy) 전략을 썼으며, 상보성 제약 대신 접촉력을 매끄러운 함수로 근사하는 smoothed contact model을 쓰는 방식도 널리 연구된다. Drake에서는 `MathematicalProgram` API를 이용해 이런 접촉 상보성 제약을 직접 추가할 수 있다.

### 실시간 Re-planning과 MPC의 연결

정적 환경에서는 출발 전에 궤적을 한 번만 계산해 두고 그대로 따라가면 된다. 하지만 사람이나 움직이는 장애물이 불쑥 나타나는 동적 환경에서는 계획한 궤적이 순식간에 쓸모없어지므로, 현재 상태에 맞춰 궤적을 **실시간으로 계속 다시 짜야(re-plan)** 한다.

이 지점에서 모션 플래닝과 제어가 만난다. 6장 §6.5에서 다룬 **MPC(Model Predictive Control)**는 사실상 매 제어 주기마다 짧은 구간(horizon)에 대해 궤적 최적화를 다시 풀어 첫 입력만 실행하고, 다음 주기에 새 측정값을 받아 다시 최적화하는 구조다. 궤적 최적화가 "전체 여정의 긴 궤적"을 한 번에 그린다면, MPC는 "눈앞의 짧은 궤적을 초당 수십~수백 번 다시 그리는" 온라인 플래너 역할을 하는 셈이다.

---

## 7.8 Task and Motion Planning [심화]

지금까지는 "현재 관절 각도에서 저 목표 포즈로 가라"처럼 기하학적 목표가 이미 주어졌다고 가정했다. 그렇다면 로봇에게 **"부엌 테이블 위의 컵을 거실 선반 위에 올려놓아라"** 같은 일상적인 명령을 내리면 어떤 일이 벌어질까? 로봇은 이 한 문장을 풀기 위해 다음 네 층위의 일을 동시에 해내야 한다.

1. 카메라로 컵과 선반이 어디 있는지 **인식**한다.
2. 컵을 어느 방향에서 잡아야 할지 연속적인 **파지 자세(grasp pose)**를 정한다.
3. `접근 → 잡기 → 들어 올리기 → 이동 → 놓기`라는 **이산적인 행동 순서(symbolic planning)**를 세운다.
4. 각 동작 단계가 장애물과 부딪히지 않도록 **연속적인 궤적(motion planning)**을 구한다.

여기서 까다로운 점은 3번(작업 순서 계획)과 2·4번(기하·모션 플래닝)이 서로 얽혀 있다는 사실이다. 예를 들어 위쪽 선반 공간이 낮다면 컵의 위쪽을 덮쳐 잡는 자세로는 선반 안에 컵을 밀어 넣을 수 없으니, 아예 처음부터 컵의 옆면을 잡거나 중간에 컵을 한 번 내려놓고 다시 잡아야 한다. 이처럼 논리적인 행동 순서(task planning)와 연속적인 공간상의 움직임(motion planning)을 따로 떼지 않고 하나의 문제로 묶어 푸는 분야가 **TAMP(Task and Motion Planning)**다.

### PDDLStream

TAMP의 대표 프레임워크가 MIT에서 개발한 **PDDLStream**(Garrett et al., 2020)이다. 고전 인공지능에서 행동 순서와 전제 조건을 기술할 때 쓰는 PDDL(Planning Domain Definition Language)로 이산적인 심볼릭 액션을 정의하되, grasp pose나 placement pose, 충돌 없는 궤적 같은 연속 파라미터를 찾아내는 연산은 **stream**이라는 선언적 샘플러로 분리해 기호 플래너가 필요할 때만 연속 공간 값을 요청해 조합할 수 있게 만든다.

### LLM 기반 Task Planning

최근에는 전통적인 PDDL 대신 대규모 언어 모델(LLM)의 상식 추론 능력을 high-level task planner 자리에 올리는 접근도 활발하다.
- **SayCan** (Google, CoRL 2022): LLM이 자연어 지시와 잘 맞는 후보 skill들에 언어적 점수를 매기면, 학습된 가치 함수(value function)가 현재 장면에서 그 skill이 실제로 성공할 확률(affordance)을 평가한다. 두 점수를 곱해 가장 높은 skill을 실행함으로써 "말은 되지만 현재 물리적으로 불가능한 행동"을 걸러 낸다.
- **Code as Policies** (Google, arXiv 2022 / ICRA 2023): LLM이 자연어 명령을 받아 조건문과 반복문, 기하 API를 호출하는 로봇 제어용 Python 코드를 직접 생성한다.
- **Inner Monologue** (Google, CoRL 2022): 한 번 계획을 세우고 끝내지 않고, 행동의 성공·실패 감지나 현재 장면 묘사 같은 환경 피드백을 텍스트로 다시 LLM에 넣어 실시간으로 계획을 수정하게 한다.

다만 이들 LLM 기반 시스템이 맡는 역할은 어디까지나 high-level 행동 순서를 짜는 부분이다. 좁은 틈 사이를 빠져나가는 충돌 회피나 정밀 조립 같은 기하학적 제약과 물리적 실행은 여전히 하위 모션 플래너와 학습된 제어 skill이 책임진다.

여기서 한 가지 중요한 가정이 남아 있다. §7.8까지 다룬 TAMP와 모든 플래너는 기본적으로 **로봇이 자신의 상태와 환경 지도를 정확히 알고 있고, 명령을 내리면 행동 결과도 확정적으로 따라온다**는 결정론적(deterministic) 세계를 전제했다. 하지만 센서는 늘 노이즈가 섞인 일부 정보만 보여 주고, 바퀴는 미끄러지기 일쑤다. 동역학과 관측이 확률적인 세계에서는 어떻게 계획을 세워야 할까? 이 질문이 다음 절의 주제다.

---

## 7.9 불확실성하 의사결정 [심화]

§7.1~§7.8까지는 로봇이 자신의 정확한 상태와 환경 지도를 모두 알고 있다고 가정했다. 하지만 현실의 로봇은 노이즈가 섞인 센서로 세상의 일부만 어렴풋이 관측할 뿐이다. 좌우가 똑같이 생긴 대칭 복도에서 자신이 왼쪽에 있는지 오른쪽에 있는지 모르는 로봇이나, 눈앞의 문이 열려 있는지 닫혀 있는지 반반으로 추정하는 로봇이 "가장 그럴듯한 상태 하나"만 찍어 계획을 세우면 엉뚱한 행동을 고르기 쉽다. 불확실성이 클 때는 단일 상태가 아니라 가능한 모든 상태에 대한 확률 분포, 즉 **belief** 자체를 놓고 계획을 세워야 한다.

이 절은 Thrun, Burgard, Fox의 *Probabilistic Robotics* §15.2·§16을 따른다. 본문에 적힌 절·표·식·그림 번호는 1999–2000년 초고 기준이며, 2005년 MIT Press 출간본에서는 MDP가 14장, POMDP가 15장, QMDP·AMDP·MC-POMDP 같은 근사 기법이 16장에 실려 있다.

### 7.9.1 도입: 세 패러다임

목표(Goal), 함정(Pit), 로봇(Robot)이 놓인 좌우 대칭 복도를 떠올려 보자. 똑같은 환경에서도 불확실성을 어디까지 인정하느냐에 따라 세 종류의 플래너는 전혀 다른 움직임을 보여 준다.

- **Classical planning**: 현재 상태를 완벽히 알고 행동 결과도 100% 결정론적이라고 가정한다. §7.3의 A*가 이 범주에 속하며, 출발 전에 최단 경로를 한 번 계산해 두면 실행 도중에는 센서를 다시 볼 필요조차 없다.
- **MDP(Markov Decision Process)**: 상태는 완전히 관측되지만 행동의 결과가 확률적이다. 모든 상태마다 최적 행동을 하나씩 지정해 주는 정책 $\pi: s \to a$를 찾으며, 좁은 통로에서 미끄러져 벽이나 함정에 빠질 위험이 크다면 거리가 조금 더 멀더라도 안전한 우회로를 택한다. 8장 §8.2에서 다룰 강화학습의 기본 틀이 바로 MDP다.
- **POMDP(Partially Observable MDP)**: 행동의 결과뿐 아니라 센서 관측까지 확률적이다. 로봇이 실제 상태를 직접 볼 수 없으므로, 사후 확률 분포인 belief $b$ 위에 정책 $\pi: b \to a$를 정의한다. 대칭 복도처럼 처음에 자신의 위치를 확신할 수 없을 때, POMDP 플래너는 곧장 목표로 돌진하지 않고 먼저 비대칭적인 랜드마크가 있는 쪽으로 우회해 자신의 위치를 확실히 알아낸 뒤 목표로 향한다. 불확실성을 줄이기 위해 일부러 정보를 얻으러 움직이는 이런 행동을 **능동적 정보 수집(active information gathering)**이라 부른다.

세 패러다임은 `classical ⊂ MDP ⊂ POMDP` 순으로 점점 넓은 문제를 포괄한다. MDP가 행동의 불확실성(가려던 곳과 실제로 간 곳의 차이)을 다룬다면, POMDP는 여기에 지각의 불확실성(실제 위치와 센서 측정값의 차이)까지 한꺼번에 끌어안는다. 3장에서 다룬 베이즈 필터가 센서 측정으로 belief를 추적하는 도구였다면, POMDP는 그렇게 추적한 belief를 보고 지금 무엇을 해야 할지 고르는 도구다(→ 3장).

<!-- DEMO: pomdp_three_paradigms.html -->

### 7.9.2 belief 위 가치 반복

MDP에서는 다음 상태 $s'$로의 전이 확률과 보상을 알면 가치 반복(value iteration)의 Bellman 방정식으로 각 상태의 가치를 계산할 수 있다.

$$C^T(s) = \max_a \int \left[ c(s') + C^{T-1}(s') \right] P(s' \mid a, s)\, ds'$$

여기서 이 절은 *Probabilistic Robotics* 15·16장의 표기를 따른다. $c$는 즉시 보상(payoff), $C^T$는 남은 horizon이 $T$일 때의 가치 함수, $P(s' \mid a, s)$는 상태 전이 확률이며, 각각 8장 §8.2의 $r$, $V$, $p(x' \mid x, a)$에 대응한다.

이제 POMDP로 넘어가 보자. 로봇은 진짜 상태 $s$를 모르고 오직 belief $b$만 안다. 발상을 조금만 바꾸면, POMDP는 상태 $s$ 대신 **belief $b$를 하나의 새로운 상태로 간주하는 연속 상태 MDP**가 된다. 위 Bellman 방정식에서 $s$를 $b$로 갈아끼우면 그대로 POMDP의 가치 반복식이 된다.

$$C^T(b) = \max_a \int \left[ c(b') + C^{T-1}(b') \right] P(b' \mid a, b)\, db' \tag{16.2}$$

이 식에서 기댓값을 가장 크게 만드는 행동을 고르면 남은 horizon $T$에 대한 최적 정책을 얻는다.

$$\pi^T(b) = \arg\max_a \int \left[ c(b') + C^{T-1}(b') \right] P(b' \mid a, b)\, db' \tag{16.3}$$

하지만 이 식을 곧바로 계산하기는 만만치 않다. $b$와 $b'$가 모두 상태 공간 $\mathcal{S}$ 위의 확률 분포이기 때문이다. 식 (16.2)의 $P(b' \mid a,b)$는 분포들 사이의 전이 확률이고, 적분 역시 belief 공간 전체를 훑어야 한다. 상태가 유한한 이산 공간에서도 belief 공간은 $|\mathcal{S}|-1$차원 연속 공간이 되며, 상태 공간 자체가 연속이면 belief 공간은 무한차원으로 치솟는다.

무한 horizon 극한에서 이 재귀가 수렴하면 다음과 같은 표준 Bellman 방정식 형태를 얻는다.

$$V(b) = \max_a \left[ r(b, a) + \gamma \sum_{o'} P(o' \mid a, b)\, V(B(b, a, o')) \right]$$

여기서 $r(b,a) = \sum_s b(s)\, c(s,a)$는 현재 belief $b$에서 행동 $a$를 택했을 때 기대되는 즉시 보상이다(유한 horizon 재귀 형태로 쓰면 앞의 식 (16.2)로 돌아간다).

다행히 belief 공간 전체를 적분해야 하는 부담을 덜어 주는 결정적인 열쇠가 있다. 행동 $a$ 뒤에 어떤 관측 $o'$가 들어왔는지만 정해지면, 다음 belief $b' = B(b, a, o')$가 베이즈 필터에 따라 **오직 하나로(유일하게)** 정해진다는 사실이다. 덕분에 막막한 belief 공간 전체 적분 대신, 실제로 들어올 수 있는 관측 $o'$의 공간 위에서 적분하도록 식을 다시 쓸 수 있다.

$$C^T(b) = \max_a \int \left[ c(B(b, a, o')) + C^{T-1}(B(b, a, o')) \right] P(o' \mid a, b)\, do' \tag{16.34}$$

여기서 belief update 연산자 $B(b, a, o')$는 3장 베이즈 필터의 예측·보정 단계를 그대로 옮긴 식이다.

$$B(b, a, o')(s') = \frac{1}{P(o' \mid a, b)}\, P(o' \mid s') \int P(s' \mid a, s)\, b(s)\, ds$$

이산 상태·관측 공간에서는 위 적분들이 모두 합(summation)으로 바뀐다. 바로 이 관측 공간으로의 재구성이 다음에 볼 alpha-vector 해법의 출발점이 된다.

### 7.9.3 4상태 toy 예시

belief 위 가치 함수가 실제로 어떤 모양을 띠는지 감을 잡기 위해, 아주 작은 4상태·2행동·2관측 문제를 손으로 직접 풀어 보자. 계산을 따라가다 보면 가치 함수가 선형 조각들의 최댓값, 즉 **PWLC(piecewise-linear convex)** 구조를 이룬다는 놀라운 성질을 눈으로 확인할 수 있다.

**설정:**
- 상태 $s_1, s_2, s_3, s_4$. 로봇은 초기에 $(s_1, s_2)$ 중 하나에 놓여 있다.
- 행동 $a_1$: 정보 수집 행동. $s_1 \leftrightarrow s_2$ 상태를 확률 0.9로 서로 바꾼다(확률 0.1로 제자리).
- 행동 $a_2$: 종결 행동. $s_3$(보상 +100) 또는 $s_4$(보상 -100)로 이동해 에피소드를 끝낸다. $s_1$에서 $a_2$를 쓰면 0.9 확률로 $s_3$, 0.1 확률로 $s_4$에 가고, $s_2$에서는 반대로 0.9 확률로 $s_4$에 간다. 따라서 기대 보상은 $s_1$에서 $+80$, $s_2$에서 $-80$이다.
- 관측 $o_1, o_2$: $s_1$에서는 각각 확률 (0.7, 0.3)으로, $s_2$에서는 확률 (0.4, 0.6)으로 관측된다.
- 초기 belief는 $b = (p_1, p_2)$이며 $p_1 + p_2 = 1$이므로 사실상 1차원 선분($p_1 \in [0, 1]$) 위에서 움직인다.

**horizon 1 계산:**

먼저 즉시 보상 $c(b) = \sum_i c(s_i) p_i$는 belief $p_i$에 대한 일차 선형 함수다. 할인 인자 $\gamma = 0.9$일 때, $T=1$에서 $a_2$를 고르면 기대 보상이 선형 함수 하나로 나온다.
$$C^1(b, a_2) = \gamma(80 p_1 - 80 p_2) = 72 p_1 - 72 p_2$$

반면 $a_1$을 고르면 보상이 있는 종결 상태($s_3$, $s_4$)로 가지 않으므로 $C^1(b, a_1) = 0$이다. 두 행동 중 더 큰 값을 고르면 $T=1$의 최적 가치 함수가 나온다.
$$C^1(b) = \max\{ 0,\; 72p_1 - 72p_2 \}$$

$C^1(b)$는 두 직선의 최댓값(upper envelope)이며, 두 직선이 만나는 $p_1 = 0.5$에서 꺾인다. 즉 $s_1$일 확률 $p_1 > 0.5$이면 과감히 $a_2$로 종결하고, 확신이 부족하면($p_1 \le 0.5$) 일단 $a_1$을 택해 한 스텝 버티는 셈이다.

**horizon 2 계산:**

이제 한 스텝의 여유가 더 생긴 $T=2$를 계산해 보자. 먼저 $a_1$을 실행한 뒤 들어오는 관측 $o_1, o_2$의 확률에 따라 $C^1$을 적분(합산)하면 다음 식을 얻는다.
$$C^2(b, a_1) = \max\{0,\; -18.79 p_1 + 38.23 p_2\} + \max\{0,\; -33.05 p_1 + 13.61 p_2\} = \max\{0,\; -18.79 p_1 + 38.23 p_2,\; -51.84 p_1 + 51.84 p_2\}$$

(각 계수는 관측 확률과 belief update 결과를 대입해 얻은 값이다. 관측 $o_1, o_2$별 선택을 조합하면 네 개의 선형 함수가 나오는데, 그중 $-33.05 p_1 + 13.61 p_2$는 구간 전체에서 $-18.79 p_1 + 38.23 p_2$보다 작아 지배되므로 가지치기된다. 참고로 *Probabilistic Robotics* 원 예제는 $a_1$의 교환 확률이 $s_1$에서 0.9, $s_2$에서 0.8이라 계수가 다르며, 원전의 식 (16.27)은 $C^2(b) = \max\{0,\; -33.048 p_1 + 7.776 p_2,\; 72 p_1 - 72 p_2\}$다. 여기서는 교환 확률을 둘 다 0.9로 단순화해 다시 계산했다.)

여기에 처음부터 $a_2$를 고르는 조각 $72 p_1 - 72 p_2$까지 합치면 상수 0 조각마저 다른 조각들에 지배되어 사라지고, $T=2$의 전체 가치 함수는 다음 세 선형 조각의 최댓값이 된다.
$$C^2(b) = \max\{ -51.84 p_1 + 51.84 p_2,\; -18.79 p_1 + 38.23 p_2,\; 72 p_1 - 72 p_2 \}$$

horizon이 1에서 2로 늘어나자 선형 조각이 2개에서 3개로 늘었다. 직관적으로 보면 가운데 선형 조각($-18.79 p_1 + 38.23 p_2$)이 바로 "먼저 $a_1$으로 상태 정보를 수집한 뒤, 관측 결과에 따라 $a_2$ 여부를 결정하는" 정보 수집 정책에 해당한다.

이처럼 선형 함수들의 최댓값으로 이루어진 가치 함수는 belief 공간에서 **볼록(convex)**하다. 즉 $\beta C(b) + (1-\beta) C(b') \geq C(\beta b + (1-\beta) b')$이 성립한다. 물리적 직관도 분명하다. 어떤 상태인지 확실히 아는 두 belief($b, b'$)를 반반 섞어 애매하게 만든 belief의 가치는 두 확실한 belief 가치의 평균을 절대 넘을 수 없기 때문이다(확실히 알수록 더 좋은 결정을 내릴 수 있다).

<!-- DEMO: pomdp_toy_pwlc.html -->

### 7.9.4 PWLC 구조와 alpha-vectors

앞선 4상태 예제에서 가치 함수는 유한 개의 선형 조각 중 최댓값을 취하는 형태였다. 사실 유한 상태·행동·관측 공간의 POMDP라면 모든 유한 horizon $T$에서 가치 함수가 언제나 **PWLC(piecewise-linear convex)** 구조를 유지한다. 수학적 귀납법으로 이 사실을 간단히 확인할 수 있다.

- **베이스 케이스($T=1$)**: 즉시 보상 $c(b) = \sum_i c(s_i) p_i$는 belief $b$에 대해 선형이다. Thrun et al. Table 16.1처럼 보상이 상태에만 달려 있으면 $C^1(b) = \gamma \sum_i c(s_i)\, p_i$로 단 하나의 선형 함수가 된다(Table 16.1 2행, 경계 조건 $|\Phi^1| = 1$). 앞선 4상태 예제는 $a_2$를 택할 때만 보상 상태로 가므로 $C^1(b)$가 행동별 선형 함수 두 개의 max가 된다(식 16.11). 어느 쪽이든 $C^1(b)$는 분명한 PWLC다.
- **귀납 단계**: $C^{T-1}(b)$가 PWLC라고 가정하자. 식 (16.34)에서 $C^{T-1}(B(b,a,o'))$를 $b$의 함수로 펼쳐 보면 놀라운 상쇄가 일어난다. belief update $B(b,a,o')$ 분모에 있던 비선형 정규화 인자 $1/P(o'\mid a, b)$가 식 (16.34) 바깥에 곱해지는 관측 확률 가중치 $P(o'\mid a, b)$와 정확히 약분되기 때문이다. 그 결과 각 alpha-vector와의 내적 $\langle \phi, B(b,a,o') \rangle \cdot P(o'\mid a, b)$이 깔끔하게 $b$의 선형 함수로 정리된다. 선형 함수들의 최댓값을 관측별로 더하고 다시 행동별로 최댓값을 취해도 결과는 여전히 선형 함수들의 최댓값이다. 따라서 $C^T(b)$도 PWLC가 된다.

이때 각 선형 조각을 이루는 계수 벡터를 **alpha-vector** $\phi$라 부른다. alpha-vector들의 집합을 $\Phi$라 두면 가치 함수는 믿을 수 없을 만큼 간결하게 쓰인다.

$$V(b) = \max_\phi \langle \phi, b \rangle$$

구체적으로 풀어서 쓰면 $V(b) = \max_{\phi \in \Phi} \sum_i \phi_i\, p_i$다. 각 alpha-vector $\phi$는 단순한 숫자 벡터가 아니라 하나의 **조건부 정책(conditional plan)**—즉 "지금 이 행동을 하고, 다음에 관측 $o_1$이 오면 이 후속 계획을, $o_2$가 오면 저 후속 계획을 따른다"는 트리—에 대응하며, $\phi_i$는 실제 상태가 $s_i$일 때 그 조건부 정책을 끝까지 실행해서 얻는 기대 누적 보상을 뜻한다.

문제는 이 alpha-vector의 개수다. 불필요한 벡터를 쳐내기(가지치기) 전에 만들어지는 후보 개수는 $|\Phi^T| = |A| \cdot |\Phi^{T-1}|^{|\mathcal{O}|}$으로 **이중 지수적(doubly exponential)**으로 폭발한다. 행동이 2개, 관측이 2개인 아주 작은 문제조차 $|\Phi^1| = 1$에서 시작해 $|\Phi^2| = 2 \cdot 1^2 = 2$, $|\Phi^3| = 2 \cdot 2^2 = 8$, $|\Phi^4| = 2 \cdot 8^2 = 128$, $|\Phi^5| = 2 \cdot 128^2 = 32{,}768$개로 순식간에 불어난다. 그래서 긴 horizon 문제에는 정확한 해법을 그대로 쓰기 어렵다.

### 7.9.5 LP 해법

폭발하는 alpha-vector 가운데 모든 belief에서 다른 벡터보다 값이 작아 결코 선택될 일이 없는 벡터(dominated alpha-vector)를 골라내 버리면 계산량을 크게 줄일 수 있다. 정확한 POMDP 해법은 앞의 `max · sum · max` 구조를 **선형 계획법(Linear Program, LP)**으로 옮겨, 지배되는 alpha-vector를 판별하고 가지치기한다.

변환 원리는 단순하다. $C = \max_a x(a)$라는 최댓값 연산은 $\{C \geq x(a) \;\forall a\}$ 제약 아래에서 $\min C$를 푸는 문제와 같고, $C = \sum_i \max_a x(a,i)$는 각 $i$마다 행동을 고르는 함수 $a(\cdot)$의 모든 조합에 대해 $\{C \geq \sum_i x(a(i),i)\}$ 제약을 세우는 꼴이 된다(제약 수는 $|A|^{|\mathcal{S}|}$). 이를 POMDP의 horizon $T$에 적용하면 다음과 같은 제약 집합(식 16.67)을 얻는다.

$$\bigcup_a \bigcup_{k(o'):1 \leq k(o') \leq |\Phi^{T-1}|} \left\{ C^T(b) \geq \gamma \sum_{o'} \sum_i \left(c_i + C^{T-1}_{k(o'),i}\right) P(o' \mid s_i') \sum_j P(s_i' \mid a, s_j)\, p_j \right\}$$

가지치기 전 제약 수는 앞서 본 $|\Phi^T| = |A| \cdot |\Phi^{T-1}|^{|\mathcal{O}|}$개다. 아래 알고리즘은 이 과정을 유한 상태·행동·관측 POMDP의 가치 반복 절차로 정리했다.

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

$|\Phi^1|=1$에서 출발해도 행동 3개, 관측 5개, horizon 4만 되면 가지치기 전 alpha-vector가 $3 \cdot 729^5 \approx 6 \times 10^{14}$개에 달한다($|\Phi^2|=3$, $|\Phi^3|=3 \cdot 3^5=729$). 현실적인 로봇 문제에서는 매 단계 LP로 pruning을 동원하더라도 계산량이 감당할 수 있는 선을 훌쩍 넘기 때문에, 실전에서는 아래의 근사 기법으로 눈을 돌린다.

### 7.9.6 일반 POMDP

로봇의 위치와 자세처럼 상태 공간이 연속일 때는 유한 차원의 alpha-vector 대신 연속 상태 공간 위에 정의된 **연속 함수**로 가치 함수를 표현해야 한다. 이론적으로 관측 기반 재귀식 (16.34)는 여전히 성립하지만, 이제 $\Phi^{T-1}$는 유한 개 벡터의 묶음이 아니라 무한차원 함수들의 집합이 된다.

---

**알고리즘: POMDP(T)** (Thrun et al. 식 (16.34)를 함수 집합 꼴로 정리. 원전의 일반 알고리즘은 Table 16.2)

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

연속 공간에서는 이 함수 집합을 그대로 저장하고 비교하는 일 자체가 불가능에 가깝다. 따라서 위 일반 알고리즘은 어디까지나 이론적 원리를 보여 주는 청사진 역할을 하며, 실제 연속 공간 문제를 풀 때는 입자 필터 기반의 **MC-POMDP**나 저차원 통계량으로 belief를 압축하는 **AMDP** 같은 실용적인 근사 해법을 쓴다.

### 7.9.7 MC-POMDP

**MC-POMDP(Monte Carlo POMDP)**는 상태 추정에 쓰던 **입자 필터(particle filter)**를 플래닝으로 확장한 알고리즘이다(→ §3.11.3). 연속 공간 위의 복잡한 확률 분포를 다루기 어렵다면, 아예 가중치를 가진 유한 개의 입자 집합으로 belief $\theta = \{\langle s^{(i)}, w^{(i)} \rangle\}$를 근사하고 가치 반복 갱신도 시뮬레이션 표본으로 대체하자는 발상이다.

먼저 어떤 belief $\theta$에서 행동 $a$를 하고 관측 $o'$를 얻었을 때 다음 belief $\theta' = B(b, a, o')$로 갱신하는 과정은 표준 입자 필터 알고리즘 그대로다.

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

이렇게 입자로 belief를 갱신할 수 있으면, 데이터베이스에 모아 둔 여러 belief $\theta$마다 각 행동 $a$의 가치 $Q(\theta, a)$를 몬테카를로 롤아웃으로 추정한다. 각 belief에서 상태 $s$, 다음 상태 $s'$, 관측 $o'$를 $N$번 무작위로 뽑아 다음 belief $\theta'$를 만들고, 거기서 얻을 최대 가치 $V(\theta')$와 보상을 평균 내어 $Q(\theta, a)$를 갱신하는 구조다.

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

이를 기대 보상 형태로 쓰면 Q함수 갱신식 (식 16.78)이 된다.

$$Q(\theta_t, a_t) \leftarrow \mathbb{E}\left[ R(o_{t+1}) + \gamma \max_{\bar{a}} Q(\theta_{t+1}, \bar{a}) \right]$$

여기서 $R(o_{t+1})$은 다음 시점에 받는 보상(payoff)이며, 실제 행동은 가장 큰 Q값을 주는 행동을 선택한다(식 16.79).

$$\sigma^Q(\theta) = \arg\max_{\bar{a}} Q(\theta, \bar{a})$$

구현에서 흥미로운 대목은 $Q(\theta, a)$를 근사하는 방식이다. belief $\theta$는 입자들의 순서 없는 집합(set)이기 때문에, 고정된 차원과 순서를 요구하는 당시의 feedforward 신경망에 그대로 집어넣기가 까다로웠다. 그래서 Thrun et al.은 $\langle \theta, a, Q \rangle$ 튜플들을 데이터베이스로 쌓아 두고, 새 belief $\theta'$가 들어오면 KL divergence 기준으로 가장 가까운 $k$-nearest neighbor들을 찾아 그 Q값의 평균을 취했다(오늘날에는 PointNet이나 DeepSets, Attention처럼 집합을 바로 인코딩하는 신경망으로 입력을 만들 수도 있다).

두 입자 집합 사이의 KL divergence는 Gaussian KDE(kernel density estimation)로 매끄러운 밀도를 씌워 근사한다. 또한 바깥 루프에서는 정적인 belief 목록만 돌리는 대신, $\varepsilon$-greedy 시뮬레이션 에피소드를 직접 실행하며 실제로 마주치는 belief들을 데이터베이스에 모으면 실제 로봇이 방문할 법한 영역에 계산 자원을 집중할 수 있다.

### 7.9.8 실험: heaven/hell과 find-and-fetch

MC-POMDP가 실제로 "모르는 정보를 먼저 알아낸 뒤 행동하는" 지능적인 정책을 배울 수 있는지 보여 주는 두 가지 고전 실험이 있다.

- **Heaven/Hell 문제**: 2차원 합성 환경에서 에이전트는 왼쪽 아래 구석에서 출발한다. 천국(heaven)은 왼쪽 위 또는 오른쪽 아래 구석 중 한 곳에, 지옥(hell)은 반대쪽에 숨어 있다. 에이전트는 둘 중 어디가 천국인지 모른 채 시작하지만, 오른쪽 위 구석에 있는 사제(priest)에게 찾아가면 힌트를 들을 수 있다. 게다가 에이전트는 자기 좌표도 직접 관측하지 못하고 매 이동마다 5%의 무작위 잡음이 끼어든다. 13번의 실험 모두에서 MC-POMDP는 17 episode 안에 빠짐없이 먼저 사제에게 들러 정보를 확인한 뒤 천국으로 직행하는 최적 정책을 학습했다.
- **Find-and-Fetch (단안 카메라)**: 시뮬레이션 속 RWI B21 로봇이 주변 어딘가(바닥이나 탁자 높이)에 놓인 물체를 찾아 그리퍼로 집는 작업이다. 단안 카메라 한 대로 한 자리에서 찍은 영상만으로는 물체까지의 3차원 거리를 확정할 수 없고, 처음에는 아예 물체가 시야 밖에 있을 수도 있다. 약 4,000번의 반복 학습 뒤 잡기 성공률이 80%를 넘어섰으며, 성공한 궤적을 보면 로봇이 먼저 주위를 둘러보며 물체를 찾아 시야에 담고 위치를 좁힌 뒤 다가가 집어 올렸다.

두 실험 모두 belief를 명시적으로 추적함으로써, 보상과 직접 무관해 보이는 **정보 수집 행동**을 스스로 계획에 녹여 냈다는 점이 핵심이다.

### 7.9.9 AMDP — belief 통계로 차원 축소

MC-POMDP가 입자 표본으로 일반 POMDP를 풀었다면, **AMDP(Augmented MDP)**는 전혀 다른 방향에서 속도를 확보한다. 실제 로봇이 주행하거나 조작할 때 마주치는 belief들은 거대한 belief 공간 전체에 고르게 퍼져 있지 않고, 대체로 좁고 규칙적인 저차원 다양체(manifold) 위에 모여 있다는 관찰에서 출발한다. AMDP는 이 belief 분포를 소수의 핵심 통계량 $\bar{b} = f(b)$로 압축해 상태에 덧붙인 뒤, 그 압축된 공간 $\bar{b}$ 위에서 표준 MDP 가치 반복을 적용한다. 덕분에 계산 복잡도가 MDP($|\mathcal{S}|$에 대한 다항 시간)와 정확 POMDP(이중지수 시간) 사이의 실용적인 타협점에 놓인다.

**가장 널리 쓰이는 요약 통계**는 최빈 상태(가장 가능성이 높은 상태)와 belief의 엔트로피 쌍이다(식 16.80).

$$\bar{b} = \langle \arg\max_s b(s),\; H[b] \rangle$$

여기서 belief의 불확실성 크기를 재는 엔트로피 $H[b]$는 다음과 같이 정의된다.

$$H[b] = -\int b(s) \ln b(s)\, ds \tag{16.81}$$

$\arg\max_s b(s)$ 하나만 쓰면 불확실성을 무시하는 표준 MDP로 돌아간다. AMDP는 여기에 엔트로피 $H[b]$를 덧붙여 "내가 지금 어디 있다고 믿는지"뿐 아니라 "그 믿음을 얼마나 확신하는지"까지 상태 변수에 넣는 셈이다. Thrun et al.도 이 두 값의 쌍이 수학적으로 엄밀한 충분 통계(sufficient statistic)라는 가정은 거의 성립하지 않는다고 인정했지만, 이어질 coastal navigation 실험에서는 이 단순한 요약만으로도 훨씬 영리한 경로를 골라냈다.

상태 $s$ 자리에 압축된 통계 상태 $\bar{b}$를 넣으면, 알고리즘 구조는 표준 MDP 가치 반복(*Probabilistic Robotics* §15.3.3)과 완전히 똑같다.

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

압축 상태 사이의 전이 확률 $P(\bar{b}' \mid a, \bar{b})$는 원래 belief 공간의 전이와 관측 모델, 그리고 사후 갱신을 모두 적분해 구한다(식 16.85).

$$P(\bar{b}' \mid a, \bar{b}) = \int\!\!\int\!\!\int I_{f(b)=\bar{b}}\, I_{f(B(o',a,b))=\bar{b}'}\, P(o' \mid s') P(s' \mid a, s) P(s \mid b)\, ds\, ds'\, do'\, db$$

실제 구현에서는 이 삼중 적분을 해석적으로 풀기 어려우므로, 랜덤 시뮬레이션을 여러 번 돌려 전이 횟수를 센 뒤 lookup table에 저장해 두고 쓴다.

### 7.9.10 Coastal Navigation 예시

AMDP가 왜 강력한지 가장 극적으로 보여 주는 사례가 **coastal navigation**이다. 넓은 빈 공간을 가로질러 반대편 목표로 가야 하는 로봇을 생각해 보자.

일반적인 MDP 플래너는 최단 거리인 한가운데 직선 경로를 택한다. 하지만 빈 공간 한가운데로 들어가면 벽이나 기둥 같은 특징물이 레이저 센서 범위에 전혀 잡히지 않기 때문에, 주행 오차가 쌓이면서 위치 belief의 엔트로피가 걷잡을 수 없이 커진다. 결국 목적지 근처에 닿고도 자신이 정확히 어디에 있는지 몰라 헤매게 된다.

반면 AMDP 플래너는 상태에 엔트로피 $H[b]$가 들어 있고 비용 함수가 높은 엔트로피(불확실성)를 피하도록 설계되어 있어서, 같은 환경에서도 장애물과 벽면 가까이 스치듯 돌아가는 경로를 고른다. 이동 거리는 조금 늘어나지만 벽과 장애물이 센서 시야에 계속 들어와 위치를 정확히 보정해 주므로 엔트로피를 낮게 유지할 수 있다. 이 이름은 먼바다로 나가면 길을 잃기 쉬워 해안선(coast)을 보며 항해하던 옛 항해술에서 유래했다.

Thrun et al.의 그림 16.5는 폭 약 40m인 박물관 홀을 가로지를 때 두 플래너가 만들어 내는 극명한 경로 차이를 보여 주며, 그림 16.6은 레이저 거리 센서의 최대 측정 거리를 바꿔 가며 목표 지점에서의 belief 엔트로피를 비교한다. 센서 사거리가 짧을수록 빈 공간에서 눈이 멀기 쉬워 coastal planner의 우위가 뚜렷하고, 센서가 홀 전체를 한눈에 볼 만큼 길어지면 두 플래너의 차이가 사라진다.

이처럼 행동 결과의 물리적 거리뿐 아니라 "앞으로 불확실성이 어떻게 줄어들 것인가"까지 경로 비용에 넣는 원리는 오늘날 **Active SLAM**(위치 불확실성을 줄이는 경로를 스스로 고르는 SLAM)과 **next-best-view planning**(정보 이득이 가장 큰 다음 시점으로 이동하는 탐색)으로 그대로 이어지고 있다.

<!-- DEMO: coastal_navigation_amdp.html -->

### 7.9.11 무엇이 살아남았나

이후 POMDP 솔버들은 이중 지수적 폭발을 피하면서 실전 크기의 문제를 풀기 위해 세 가지 큰 줄기로 발전해 왔다.
- **Point-based value iteration (PBVI, HSVI, SARSOP)**: belief 공간 전체를 훑는 대신 도달 가능한 대표 belief 점들만 몇 개 뽑아 두고, 그 점들 위에서만 §7.9.4의 alpha-vector backup을 수행해 계산량을 극적으로 낮췄다.
- **온라인 MCTS 계열 (POMCP, DESPOT)**: POMCP(Silver & Veness, 2010)와 DESPOT은 미리 전체 정책을 다 구해 두는 대신, 현재 belief에서 출발해 시뮬레이션 rollout으로 Q값을 추정하며 belief tree를 몬테카를로 트리 탐색(MCTS)으로 파고든다. §7.9.7 MC-POMDP의 표본 기반 Q 추정 아이디어가 트리 탐색과 만난 셈이다.
- **Deep POMDP (DRQN, DVRL 등)**: RNN이나 트랜스포머의 은닉 상태(hidden state)가 과거 관측과 행동을 압축한 암묵적 belief 역할을 맡고, 그 위에서 신경망이 Q함수나 정책을 직접 근사한다.

상태에 불확실성 요약을 덧붙이는 AMDP의 아이디어 역시 모델 파라미터의 사후 분포를 상태에 넣는 **Bayes-adaptive MDP(BAMDP)**나, 위치 공분산 지표를 비용 함수에 얹는 Active SLAM 형태로 계속 살아남았다.

정리하자면 POMDP 플래닝은 전이 확률과 관측 모델을 미리 알고 있을 때 불확실성 아래에서의 최적 정책을 수학적으로 계산하는 틀이고, 8장 §8.3에서 다룰 PPO·SAC 같은 model-free 심층 강화학습은 모델을 모르는 상태에서 순수 경험만으로 정책을 배우는 틀이다(MC-POMDP는 모델로 belief를 추적하면서 시뮬레이션 경험으로 Q값을 배운다는 점에서 둘의 징검다리에 놓인다).

이제 6장의 제어기와 7장의 모션 플래닝·POMDP를 지나, 사람이 일일이 물리 모델과 전이 확률을 적어 주기 어려운 복잡한 세계에서 로봇이 **데이터와 경험으로 스스로 행동을 배우는 방법**을 다루는 8장 로봇 러닝으로 넘어가자.

---

## 참고 자료

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

## 발전 과정

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
