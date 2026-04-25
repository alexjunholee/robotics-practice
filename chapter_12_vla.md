# Ch.12 — Vision-Language-Action (VLA) & Embodied AI

로봇이 "빨간 컵을 집어서 테이블 위에 놓아줘"라는 자연어 명령을 받아 실행하는 것이 VLA의 목표다. 비전, 언어 모델, 제어를 하나로 합치는 분야이며, "왜 ChatGPT가 로봇을 움직이지 못하는지", "왜 시뮬레이션에서 잘 되던 정책이 실제 로봇에서 망하는지"를 이해하려면 이 챕터의 개념이 필요하다. CoRL, ICRA 2024-2025에서 VLA 관련 논문 비중이 크게 늘었다.

## 12.1 VLA 개념

기존 로봇 시스템은 시각 인식, 언어 이해, 행동 생성이 각각 독립 파이프라인으로 분리되어 있었다.

**VLA (Vision-Language-Action)**는 이 세 역할을 단일 모델로 처리한다.

```
입력: 이미지 + 자연어 명령 ("pick up the red cup")
출력: 로봇 행동 (관절 각도, gripper 명령 등)
```

**Embodied AI**: 물리적 환경에서 상호작용하며 학습하는 AI
- 단순 인식을 넘어 행동까지 포함
- 시뮬레이션과 실제 환경의 간극 (Sim-to-Real)

Embodied AI가 기존 AI와 다른 점은, 모델이 단순히 "이것은 컵이다"라고 분류하는 데 그치지 않고, 실제로 컵을 집어 올리는 물리적 행동을 수행해야 한다는 것이다. 이 과정에서 중력, 마찰, 충돌 같은 물리 법칙을 모두 고려해야 하므로, 단순 이미지 분류보다 훨씬 어렵다.

> **추천 자료**
> - [Google DeepMind Robotics Blog](https://deepmind.google/discover/blog/) — RT-1, RT-2, PaLM-E 등의 공식 블로그 포스트
> - [Brohan et al., "RT-2: Vision-Language-Action Models" (2023)](https://arxiv.org/abs/2307.15818) — VLA의 핵심 논문

## 12.2 주요 모델 및 연구

### 12.2.1 RT-1, RT-2 (Google DeepMind)

RT-1과 RT-2는 "대규모 데이터로 학습한 범용 로봇 정책"이라는 개념을 처음으로 실증한 모델이다. RT-1 이전에는 로봇 하나당 하나의 태스크를 학습시키는 방식이 표준이었다. RT-1/RT-2는 하나의 모델로 수백 가지 태스크를 수행할 수 있음을 보여줬다.

**RT-1 (Robotics Transformer 1)**:
- 대규모 로봇 데모 데이터로 학습
- 130K 에피소드, 700+ 태스크
- Tokenized action output

**RT-2 (Robotics Transformer 2)**:
- VLM (PaLI-X, PaLM-E)을 로봇 행동으로 파인튜닝
- Web-scale 데이터의 지식을 로봇에 전이
- "chain of thought" 추론 가능

RT-2의 아이디어는 단순하다. 인터넷에서 학습한 거대 언어/비전 모델이 이미 "세상에 대한 지식"을 갖고 있으니, 그 지식을 로봇 행동으로 파인튜닝하면 제로샷(zero-shot)으로 새로운 물체나 상황에도 대응할 수 있다. 예를 들어, RT-2는 학습 데이터에 없던 물체도 언어 지식을 활용해 집어 올릴 수 있다.

> **추천 자료**
> - [Google DeepMind — RT-2 Demo Video](https://deepmind.google/discover/blog/rt-2-new-model-translates-vision-and-language-into-action/) — RT-2의 실제 동작 영상과 설명
> - [Brohan et al., "RT-1: Robotics Transformer" (2022)](https://arxiv.org/abs/2212.06817) — RT-1 원 논문
> - [Brohan et al., "RT-2" (2023)](https://arxiv.org/abs/2307.15818) — RT-2 원 논문

### 12.2.2 PaLM-E

**Embodied Multimodal Language Model**:
- PaLM (Language) + ViT (Vision) + 로봇 상태
- 562B 파라미터
- "다목적" 로봇 태스크 수행

PaLM-E가 흥미로운 이유는 "positive transfer"를 보여줬기 때문이다. 로봇 데이터, 웹 이미지, 텍스트를 함께 학습하면 오히려 각각을 따로 학습할 때보다 로봇 태스크 성능이 올라간다. 범용 지식이 로봇 행동에도 도움이 된다는 점을 실증한 것이다.

> **추천 자료**
> - [Driess et al., "PaLM-E: An Embodied Multimodal Language Model" (2023)](https://arxiv.org/abs/2303.03378) — PaLM-E 원 논문

### 12.2.3 OpenVLA

RT-2나 PaLM-E는 Google 내부 인프라 없이는 쓸 수 없다. OpenVLA는 오픈소스로 공개된 VLA 모델로, 실제로 연구실에서 다운로드 받아 파인튜닝하고 로봇에 올릴 수 있는 모델이다.

**Open-source VLA**:
- 7B 파라미터 (Llama 2 기반)
- 970K 에피소드 학습
- 다양한 로봇 embodiment에 적용 가능

```python
# OpenVLA 사용 예시 (개념)
from openvla import OpenVLAModel

model = OpenVLAModel.from_pretrained("openvla/openvla-7b")

action = model.predict(
    image=current_image,
    instruction="pick up the blue block and place it on the red target"
)
```

**RT-X 프로젝트**: OpenVLA와 함께 알아둘 것이 RT-X이다. 여러 연구 기관이 수집한 로봇 데이터를 모아 하나의 거대한 데이터셋(Open X-Embodiment)을 만들고, 이를 기반으로 범용 로봇 정책을 학습하는 프로젝트다. 22개 이상의 로봇 유형에서 수집된 데이터를 포함한다.

**Octo**: RT-X 데이터로 학습된 또 다른 오픈소스 모델로, OpenVLA보다 작은 사이즈(93M 파라미터)로 더 가볍게 활용할 수 있다. 다양한 로봇 플랫폼에 빠르게 파인튜닝할 수 있도록 설계한 모델이다.

> **추천 자료**
> - [OpenVLA GitHub](https://github.com/openvla/openvla) — 코드와 모델 가중치 공개
> - [Kim et al., "OpenVLA" (2024)](https://arxiv.org/abs/2406.09246) — OpenVLA 논문
> - [Open X-Embodiment Collaboration, "Open X-Embodiment" (2023)](https://arxiv.org/abs/2310.08864) — RT-X 데이터셋 논문
> - [Octo GitHub](https://github.com/octo-models/octo) — 경량 오픈소스 로봇 정책 모델

### 12.2.4 Navigation 관련

로봇이 물체를 조작(manipulation)하는 것뿐 아니라, 환경 내에서 이동(navigation)하는 것도 Embodied AI의 핵심 과제이다. 아래 연구들은 LLM의 언어 이해 능력을 navigation에 활용하는 접근이다.

**LINGO**: Language-guided Indoor Navigation

**SayCan**: LLM이 "할 수 있는 것"과 "해야 하는 것"을 분리
- Affordance function: 로봇이 현재 할 수 있는 행동
- LLM: 목표 달성을 위해 해야 하는 행동

SayCan의 핵심 아이디어를 조금 풀어보면: LLM에게 "커피 만들어줘"라고 하면, LLM은 "1. 컵을 잡아 2. 커피 머신으로 가 3. 버튼을 눌러..."처럼 계획을 세울 수 있다. 하지만 로봇이 현재 컵 근처에 없다면 "컵을 잡아"는 실행 불가능하다. SayCan은 LLM의 계획(해야 하는 것)과 로봇의 현재 가능 행동(할 수 있는 것)을 곱해서, 실행 가능하면서도 목표에 가까운 행동을 선택한다.

> **추천 자료**
> - [Ahn et al., "Do As I Can, Not As I Say: Grounding Language in Robotic Affordances" (2022)](https://arxiv.org/abs/2204.01691) — SayCan 논문
> - [SayCan project page](https://say-can.github.io/) — 데모 영상 포함

## 12.3 World Models

실제 로봇으로 수만 번의 시행착오를 하는 것은 시간과 비용 면에서 불가능에 가깝다. World Model은 로봇이 "머릿속에서 시뮬레이션"을 돌려보고, 그 결과를 바탕으로 행동을 결정할 수 있게 해준다. 자율주행에서 특히 각광받고 있는데, "앞 차가 갑자기 멈추면 어떻게 될까?"를 실제로 경험하지 않고도 예측할 수 있기 때문이다.

**World Model**: 환경의 동작을 예측하는 모델

**왜 필요한가?**
- 실제 로봇 없이 model-based RL 가능
- 위험한 탐색을 시뮬레이션 안에서 처리

**자율주행 분야**:
- **GAIA-1 (Wayve)**: Video prediction + Action conditioning. 실제 주행 영상을 학습해서 "이렇게 핸들을 돌리면 이런 장면이 될 것"을 예측하는 생성 모델이다.
- **DriveDreamer**: 주행 시나리오 생성. 텍스트 조건으로 다양한 시나리오를 생성해 학습 데이터를 증강하는 데 활용된다.
- **MILE**: World model 기반 End-to-End driving. 미래 상태를 예측하는 implicit world model을 학습하고, 이를 기반으로 주행 정책을 도출한다.

**구조**:

```
z_{t+1} = f(z_t, a_t)     # Dynamics model (현재 상태 + 행동 → 다음 상태)
o_t = g(z_t)              # Observation model (잠재 상태 → 관측)
r_t = h(z_t, a_t)         # Reward model (보상 예측)
```

선형대수 시간에 배운 상태 공간 모델(state-space model)과 비슷한 구조다. x_{t+1} = Ax_t + Bu_t 가 비선형 신경망 버전으로 확장된 것이라고 보면 된다.

> **추천 자료**
> - [Hu et al., "GAIA-1: A Generative World Model for Autonomous Driving" (2023)](https://arxiv.org/abs/2309.17080) — Wayve의 World Model 논문
> - [Wang et al., "DriveDreamer" (2023)](https://arxiv.org/abs/2309.09777) — 주행 시나리오 생성 논문
> - [Yannic Kilcher — World Models Explained](https://www.youtube.com/watch?v=dPsXxLyqpfs) — World Model 개념 설명 영상

## 12.4 End-to-End vs Modular

로봇 시스템을 설계할 때 가장 먼저 내려야 하는 아키텍처 결정이다. 모르면 논문을 읽을 때 "이 시스템이 왜 이렇게 설계되었는지"를 파악할 수 없다.

로봇 시스템 설계의 두 가지 철학이다.

**End-to-End**:

```
센서 입력 → [단일 신경망] → 행동 출력
```

- 장점: 간단한 파이프라인, 중간 표현의 bottleneck 없음
- 단점: Interpretability 부족, 대규모 데이터 필요
- 예시: NVIDIA PilotNet, Tesla FSD (추정)

**자율주행에서의 End-to-End 최신 연구**:
- **UniAD (Unified Autonomous Driving, 2023)**: End-to-End이면서도 내부에 detection, tracking, mapping, prediction, planning 모듈을 두어, 해석 가능성을 유지한 통합 모델이다. CVPR 2023 Best Paper.
- **VAD (Vectorized Scene Representation for Efficient Autonomous Driving, 2023)**: 장면을 벡터화된 표현으로 변환하여 효율적인 End-to-End 주행 정책을 학습한다.
- **GenAD (Generalized Autonomous Driving, 2024)**: 생성 모델 기반으로 다양한 주행 시나리오에 일반화 가능한 End-to-End 시스템이다.

**Modular**:

```
센서 → [인식] → [예측] → [계획] → [제어] → 행동
```

- 장점: 각 모듈 독립적 개발/디버깅, 해석 가능
- 단점: 모듈 간 정보 손실, 최적화 어려움
- 예시: Apollo, Autoware

**최근 트렌드**: 하이브리드 접근. 인식은 학습 기반, 계획·제어는 모델 기반으로 안전성을 확보하되, UniAD처럼 End-to-End 프레임워크 안에 명시적 모듈을 배치한다.

> **추천 자료**
> - [Hu et al., "Planning-oriented Autonomous Driving (UniAD)" (2023)](https://arxiv.org/abs/2212.10156) — CVPR 2023 Best Paper
> - [Jiang et al., "VAD" (2023)](https://arxiv.org/abs/2303.12077) — 벡터화 기반 자율주행
> - [Andrej Karpathy — Tesla AI Day 2022 Presentation](https://www.youtube.com/watch?v=ODSJsviD_SU) — End-to-End 자율주행 실무 관점

### End-to-End vs Modular: 2026년 현실

학회에서는 end-to-end가 화제지만, 실제 배포된 로봇 시스템의 대부분은 모듈형이다. 왜 그런가?

- **디버깅**: end-to-end 모델이 실패했을 때 원인을 찾기 어렵다. "왜 로봇이 컵을 놓쳤는가?"에 대해, 모듈형이면 "depth estimation이 틀렸다" 또는 "grasp planning이 잘못됐다"로 좁힐 수 있지만, end-to-end에서는 어디서 틀렸는지 모른다.
- **안전 보장**: 모듈형은 각 모듈에 safety check를 넣을 수 있다 (속도 제한, 충돌 감지 등). End-to-end에서 이런 보장을 넣기 어렵다.
- **부분 업데이트**: perception 모듈만 개선하고 싶을 때, 모듈형이면 해당 모듈만 교체하면 된다. End-to-end는 전체를 재학습해야 한다.
- **데이터 효율**: end-to-end 학습은 대규모 데이터가 필요하다. RT-2는 130k 에피소드, OpenVLA는 970k 에피소드를 사용했다. 대부분의 연구실은 이 규모의 데이터를 수집할 여력이 없다.

현실적 방향은 **하이브리드**다. 인식은 VFM(foundation model)으로 범용화하고, 계획/제어는 모듈형으로 안전성을 확보한다. 연구실의 Local/Global Module 설계(18장)가 이 방향이다.

end-to-end가 모듈형을 완전히 대체하려면 두 가지가 선행되어야 한다: 디버깅 가능한 end-to-end 구조, 그리고 소규모 데이터로도 학습되는 few-shot 정책. safety guarantee 문제도 여전히 열려 있다.

## 12.5 Spatial AI + VLA 통합

VLA 모델이 아무리 좋아도 실시간으로 장애물을 피하지 못하면 로봇은 벽에 부딪힌다. 반대로, 장애물 회피만 잘 하는 로봇은 "커피 가져와"라는 복잡한 명령을 수행하지 못한다. 두 레벨을 통합해야 실제 로봇 시스템이 돌아간다.

연구실의 2-Module Architecture와 연결:

**Local (Fast) Perception**:
- Geometric understanding: 깊이, 장애물, 자세
- 실시간 반응 (10-100Hz)
- Classical 또는 경량 학습 모델

**Global (Heavy) Understanding**:
- Semantic understanding: 객체, 관계, 맥락
- VFM/VLA 기반
- 서버 또는 클라우드 처리 (1-10Hz)

**통합 시나리오**:

```
1. Local: 실시간 obstacle avoidance, odometry
2. Global: "kitchen에서 cup을 찾아서 table로 가져와"
   - VLM으로 cup 인식
   - Semantic map에서 경로 계획
3. Local이 Global의 waypoint를 받아 실제 이동 수행
```

## 12.6 Sim-to-Real & Simulation Platforms

실제 로봇으로 데이터를 모으는 건 느리고 비싸고 위험하다. 그래서 시뮬레이션에서 먼저 학습하고 실제 로봇에 전이(transfer)하는 Sim-to-Real이 사실상 기본이 되었다. 하지만 시뮬레이션과 현실 사이에는 "Reality Gap"이 존재한다. 이 간극을 좁히는 주요 기법들을 아래에 정리한다.

**Domain Randomization**: 시뮬레이션에서 텍스처, 조명, 물리 파라미터 등을 무작위로 변경하여 학습한다. 모델이 다양한 조건에 노출되면, 현실 환경도 그 중 하나의 변형(variation)으로 처리할 수 있게 된다.

**주요 시뮬레이션 플랫폼**:
- **NVIDIA Isaac Sim/Lab**: GPU 가속 물리 시뮬레이션. 수천 개의 환경을 병렬로 돌릴 수 있어 대규모 강화학습에 적합하다. Isaac Lab은 로봇 학습 연구를 위한 통합 프레임워크이다.
- **AI2-THOR (Allen Institute)**: 실내 환경 시뮬레이터. 주방, 거실 등 가정 환경에서 물체 조작(manipulation)을 연습할 수 있다. Embodied AI 연구에서 가장 많이 쓰이는 플랫폼 중 하나이다.
- **Habitat (Meta)**: 대규모 3D 스캔 환경(Matterport3D, Gibson 등)에서 내비게이션 학습이 가능하다. Habitat Challenge를 통해 매년 벤치마크를 제공한다.
- **MuJoCo**: 접촉(contact) 물리에 강점이 있는 시뮬레이터. 로봇 팔 조작이나 보행 학습에 널리 사용된다. DeepMind가 인수 후 오픈소스로 전환했다.

> **추천 자료**
> - [NVIDIA Isaac Lab Documentation](https://isaac-sim.github.io/IsaacLab/) — 로봇 학습을 위한 시뮬레이션 프레임워크
> - [AI2-THOR Documentation](https://ai2thor.allenai.org/) — 실내 환경 시뮬레이터
> - [Habitat Documentation](https://aihabitat.org/) — Meta의 Embodied AI 플랫폼
> - [Tobin et al., "Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World" (2017)](https://arxiv.org/abs/1703.06907) — Domain Randomization 원 논문

## 12.7 심화: Imitation Learning

*연구자가 되고 싶다면 여기서부터 읽어라.*

VLA와 Embodied AI에서 정책(policy)을 학습하는 방법은 크게 강화학습(RL)과 모방학습(IL)으로 나뉜다. 로보틱스에서는 IL이 RL보다 훨씬 자주 쓰인다. 그 이유를 이해하려면 각 접근법의 구조를 알아야 한다.

**Behavioral Cloning (BC)**

가장 단순한 IL 방법이다. 전문가(사람 또는 스크립트)의 시연 데이터 `{(s_t, a_t)}`를 수집하고, 상태 `s_t`에서 행동 `a_t`를 예측하는 supervised learning을 수행한다.

```
Loss = E[ || π_θ(s_t) - a_t ||^2 ]
```

간단하고 구현이 쉽지만 치명적인 문제가 있다: **distribution shift**. 학습 시에는 전문가의 상태 분포를 따르지만, 추론 시에는 자기 자신의 (불완전한) 행동이 다음 상태를 결정한다. 작은 오차가 누적되면서 전문가가 방문한 적 없는 상태로 빠지고, 거기서 어떻게 해야 할지 모른다.

**DAgger (Dataset Aggregation)**

Distribution shift를 완화하는 대표적 방법이다. 핵심 아이디어는 학습된 정책으로 데이터를 수집하되, 전문가의 라벨을 받아서 데이터셋에 추가하는 것이다.

```
1. 초기 데이터 D = {전문가 시연}으로 정책 π_1 학습
2. for i = 1, 2, ...
     π_i로 rollout 수행 → 방문한 상태 {s_t} 수집
     전문가에게 {s_t}에서의 행동 {a_t^*}를 질의
     D = D ∪ {(s_t, a_t^*)}
     D로 π_{i+1} 학습
```

전문가에게 매번 질의하는 건 비싸기 때문에, human-in-the-loop 변형이나 DAgger의 근사 버전(HG-DAgger, ThriftyDAgger 등)이 쓰인다.

**왜 RL보다 IL이 로보틱스에서 자주 쓰이는가?**

| 기준 | RL | IL |
|------|----|----|
| Sample efficiency | 수백만 에피소드 필요 | 수백~수천 시연으로 충분 |
| 보상 함수 | 직접 설계해야 함 (reward engineering) | 불필요 |
| 안전성 | 탐색(exploration) 중 위험한 행동 가능 | 전문가 행동 모방이므로 상대적으로 안전 |
| Sim-to-Real | 보상 함수의 sim-real gap도 문제 | 실제 시연 데이터를 쓰면 gap 감소 |

로보틱스에서 보상 함수를 제대로 설계하는 것은 매우 어렵다. "컵을 잡아라"의 보상을 어떻게 정의할 것인가? 컵과 그리퍼 사이의 거리? 그러면 로봇이 컵 옆에만 가서 멈출 수 있다. 잡았는지 여부? 그러면 sparse reward 문제가 생긴다. IL은 이 문제를 우회한다.

> **추천 자료**
> - [Ross et al., "A Reduction of Imitation Learning and Structured Prediction to No-Regret Online Learning" (2011)](https://arxiv.org/abs/1011.0686) — DAgger 원 논문
> - [Florence et al., "Implicit Behavioral Cloning" (CoRL 2021)](https://arxiv.org/abs/2109.00137) — BC의 한계를 극복하기 위한 implicit 접근
> - [Zare et al., "A Survey of Imitation Learning" (2024)](https://arxiv.org/abs/2309.15894) — IL 전반 서베이

## 12.8 심화: Diffusion Policy

*연구자가 되고 싶다면 여기서부터 읽어라.*

Chi et al.(RSS 2023)이 제안한 Diffusion Policy는 로봇 조작(manipulation) 분야에서 BC 계열 방법을 빠르게 대체하고 있는 정책 표현 방식이다. 핵심 아이디어는 행동 시퀀스(action trajectory)를 denoising diffusion 과정으로 생성하는 것이다.

**왜 Diffusion인가?**

기존 BC는 `π_θ(s) → a`로 단일 action을 결정론적으로 예측한다. 하지만 현실에서는 같은 상태에서도 여러 가능한 행동이 있다(multi-modality). 예를 들어 테이블 위의 컵을 잡을 때 왼쪽에서 잡아도 되고 오른쪽에서 잡아도 된다. 결정론적 BC는 이 두 행동의 평균을 출력해서 둘 다 실패한다. Gaussian Mixture Model 같은 방법도 있지만, 모드 수를 미리 정해야 한다.

Diffusion Policy는 이 multi-modal 분포를 자연스럽게 표현한다.

**동작 원리**

```
1. 랜덤 노이즈 a_T ~ N(0, I)에서 시작 (T = diffusion steps)
2. 현재 관측 s를 조건으로 반복적으로 denoising:
   a_{t-1} = denoise_θ(a_t, s, t)    for t = T, T-1, ..., 1
3. 최종 a_0가 실행할 action trajectory
```

Action trajectory는 단일 action이 아니라 미래 수 스텝의 action 시퀀스 `[a_0, a_1, ..., a_H]`이다. 이 중 처음 몇 스텝만 실행하고(receding horizon), 다시 새 관측으로 다음 trajectory를 생성한다.

**장점**:
- Multi-modal action distribution을 명시적 가정 없이 표현
- Action sequence를 한 번에 생성하므로 temporally coherent한 행동
- 학습이 안정적 (denoising score matching은 잘 수렴함)

**단점**:
- Inference 시 여러 번의 denoising step이 필요하므로 느림 (10~100 steps)
- 실시간 제어(>100Hz)에는 부적합할 수 있음. DDIM 같은 가속 기법이나 consistency distillation으로 완화 가능

**실무 참고**: Chi et al.의 원 논문 실험에서 연속 action space 태스크 12개 중 11개에서 BC 계열을 앞섰다. 접촉이 많은 삽입·조립 태스크에서 차이가 특히 크다.

> **추천 자료**
> - [Chi et al., "Diffusion Policy: Visuomotor Policy Learning via Action Diffusion" (RSS 2023)](https://arxiv.org/abs/2303.04137) — Diffusion Policy 원 논문
> - [Diffusion Policy 프로젝트 페이지](https://diffusion-policy.cs.columbia.edu/) — 코드, 데모, 영상
> - [Ho et al., "Denoising Diffusion Probabilistic Models" (NeurIPS 2020)](https://arxiv.org/abs/2006.11239) — Diffusion model 기초 논문

## 12.9 심화: Sim-to-Real Transfer

*연구자가 되고 싶다면 여기서부터 읽어라.*

12.6에서 시뮬레이션 플랫폼과 Domain Randomization을 간략히 다뤘다. 여기서는 Sim-to-Real transfer의 구체적 기법들을 체계적으로 정리한다.

**1. Domain Randomization (DR)**

시뮬레이션 환경의 파라미터를 학습 시마다 무작위로 변경한다. 모델이 충분히 다양한 조건에서 학습하면, 현실 환경이 그 변형 중 하나에 포함될 것이라는 가정이다.

랜덤화 대상:
- **시각적(Visual)**: 텍스처, 조명 방향/세기, 카메라 위치/시야각, 배경
- **물리적(Physical)**: 마찰 계수, 관성 모멘트, 링크 질량, 관절 감쇠(damping)
- **동역학(Dynamics)**: actuator 지연, 센서 노이즈, 제어 주기

DR의 한계: 랜덤화 범위를 너무 넓히면 학습 자체가 어려워지고, 너무 좁히면 현실을 커버하지 못한다. 적절한 범위를 찾는 것이 실무적으로 중요하다.

**2. System Identification (SysID)**

실제 시스템의 물리 파라미터를 측정하거나 추정하여 시뮬레이터를 보정하는 방식이다.

```
1. 실제 로봇에서 특정 trajectory를 실행하여 데이터 수집
2. 시뮬레이터의 파라미터 φ를 최적화:
   φ* = argmin_φ || f_sim(φ) - f_real ||^2
3. 보정된 시뮬레이터에서 정책 학습
```

전통적이고 효과적이지만, 모든 파라미터를 정확히 추정하기는 어렵고, 시뮬레이터가 모델링하지 않는 현상(케이블의 유연함, 접촉면의 미세 변형 등)에는 무력하다.

**3. Real-to-Sim-to-Real (R2S2R)**

DR과 SysID의 장점을 결합한 최근 접근이다.

```
1. 실제 데이터를 소량 수집
2. 실제 데이터로 시뮬레이터를 교정 (SysID) 또는 차이를 모델링
3. 교정된 시뮬레이터에서 정책 학습
4. 학습된 정책을 실제 로봇에 적용
5. (반복) 실제 결과로 시뮬레이터를 다시 교정
```

**4. Transfer 성공 여부 판단**

정량적으로 판단하는 가장 직접적인 방법: sim과 real에서 동일 태스크의 성공률을 비교한다.

- **Sim 성공률 ≈ Real 성공률**: transfer 성공. 시뮬레이터가 현실을 잘 반영.
- **Sim >> Real**: reality gap이 큼. DR 범위 확장 또는 SysID 보정 필요.
- **Sim < Real**: 드물지만 발생. 시뮬레이터가 오히려 더 어려운 조건(보수적)으로 설정된 경우.

추가 지표로 행동 궤적(trajectory)의 유사도, 접촉력(contact force) 비교 등을 사용하기도 한다.

> **추천 자료**
> - [Tobin et al., "Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World" (2017)](https://arxiv.org/abs/1703.06907) — DR 원 논문
> - [Muratore et al., "Robot Learning from Randomized Simulations" (2022)](https://arxiv.org/abs/2111.00137) — DR 체계적 정리
> - [Hanna & Stone, "Grounded Action Transformation for Sim-to-Real" (AAAI 2017)](https://arxiv.org/abs/1511.07461) — transfer 방법론
> - [NVIDIA Isaac Lab Tutorials](https://isaac-sim.github.io/IsaacLab/) — 실습용 DR/SysID 파이프라인

> **추가 논문 (3D/Spatial 이해 + 벤치마크)**
> - [Hong et al., "3D-LLM: Injecting the 3D World into Large Language Models" (NeurIPS 2023, arXiv:2307.12981)](https://arxiv.org/abs/2307.12981) — LLM에 3D 공간 이해 능력을 부여. 3D captioning, QA, navigation
> - [Chen et al., "SpatialVLM: Endowing Vision-Language Models with Spatial Reasoning" (CVPR 2024, arXiv:2401.12168)](https://arxiv.org/abs/2401.12168) — VLM에 거리/크기 등 공간 추론 능력 추가
> - [Nasiriany et al., "RoboCasa: Large-Scale Simulation of Everyday Tasks for Generalist Robots" (RSS 2024, arXiv:2406.02523)](https://arxiv.org/abs/2406.02523) — 100개 주방 태스크, 150+ 물체 카테고리. 가정용 로봇 벤치마크
> - [Szot et al., "Habitat 3.0: A Co-Habitat for Humans, Avatars, and Robots" (ICLR 2024, arXiv:2310.13724)](https://arxiv.org/abs/2310.13724) — 사람-로봇 공존 시뮬레이션. Social navigation, 협업 태스크

> **기술 흐름: VLA & Embodied AI**
> - **~2015**: 개별 태스크별 모방학습(imitation learning), 단일 물체 grasping 연구 중심
> - **2017~**: Domain Randomization을 통한 Sim-to-Real 전이 본격화, MuJoCo/PyBullet 기반 연구
> - **2020~**: 대규모 언어 모델(LLM)과 비전의 결합 시도. CLIPort, SayCan 등 언어 기반 로봇 제어 등장
> - **2022~**: RT-1, RT-2, PaLM-E 등 Foundation Model 기반 로봇 정책 등장. Open X-Embodiment 데이터셋 구축
> - **2024~**: OpenVLA, Octo 등 오픈소스 VLA 모델 공개. World Model 기반 계획(planning)이 자율주행과 조작 모두에서 주목. End-to-End 자율주행(UniAD, VAD, GenAD)이 modular 방식을 대체하기 시작
> - **지금 주목할 것**: Foundation Model을 로봇에 적용하는 연구가 2023년 이후 빠르게 늘고 있다 (RT-2, OpenVLA, Octo, pi0 등). OpenVLA/Octo처럼 오픈소스 모델을 자기 로봇에 파인튜닝할 수 있으니, 직접 실험해보는 것을 추천한다.
