# Ch.12 — Vision-Language-Action (VLA) & Embodied AI

VLA의 목표는 로봇이 "빨간 컵을 집어서 테이블 위에 놓아줘" 같은 자연어 명령을 받아 실행하는 것이다. 비전, 언어 모델, 제어를 하나로 묶는 분야이며, "왜 ChatGPT가 로봇을 움직이지 못하는지", "왜 시뮬레이션에서 잘 되던 정책이 실제 로봇에서 실패하는지"를 이해하려면 이 장의 개념이 필요하다.

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

Embodied AI는 시각적 객체 분류를 넘어 실제 환경에서 컵을 집어 올리는 물리적 상호작용까지 포괄한다. 중력, 마찰, 충돌 같은 물리 법칙이 직접 작용하므로 단순 인식 태스크보다 고려할 변수가 훨씬 복잡하다.

> **추천 자료**
> - [Google DeepMind Robotics Blog](https://deepmind.google/discover/blog/) — RT-1, RT-2, PaLM-E 등의 공식 블로그 포스트
> - [Brohan et al., "RT-2: Vision-Language-Action Models" (2023)](https://arxiv.org/abs/2307.15818) — VLA의 핵심 논문

## 12.2 주요 모델 및 연구

### 12.2.1 RT-1, RT-2 (Google DeepMind)

RT-1과 RT-2는 대규모 로봇 데이터와 웹 규모의 시각·언어 지식을 하나의 정책에 결합한 사례다. RT-1은 하나의 모델이 수백 가지 태스크를 수행할 수 있음을 논문의 실험 환경에서 보였다.

RT-1(Robotics Transformer 1)은 대규모 로봇 데모 데이터, 즉 130K 에피소드와 700개 이상의 태스크로 학습했다. 출력은 tokenized action이다.

RT-2(Robotics Transformer 2)는 PaLI-X와 PaLM-E 같은 VLM을 로봇 행동 출력에 맞게 파인튜닝한다. 원 논문은 웹 규모 데이터에서 얻은 지식의 전이와 별도의 robot-chain-of-thought 실험을 보고한다.

RT-2는 인터넷에서 학습한 거대 언어·비전 모델의 "세상에 대한 지식"을 로봇 행동에 맞춰 파인튜닝한다. 원 논문의 실험에서는 이 구조가 제로샷(zero-shot)으로 새로운 물체나 상황에 대응할 수 있음을 보였다. RT-2는 학습 데이터에 없던 물체도 언어 지식을 활용해 집어 올리는 식이다.

> **추천 자료**
> - [Google DeepMind — RT-2 Demo Video](https://deepmind.google/discover/blog/rt-2-new-model-translates-vision-and-language-into-action/) — RT-2의 실제 동작 영상과 설명
> - [Brohan et al., "RT-1: Robotics Transformer" (2022)](https://arxiv.org/abs/2212.06817) — RT-1 원 논문
> - [Brohan et al., "RT-2" (2023)](https://arxiv.org/abs/2307.15818) — RT-2 원 논문

### 12.2.2 PaLM-E

PaLM-E는 Embodied Multimodal Language Model로, PaLM(언어)과 ViT(비전)에 로봇 상태를 결합한 562B 파라미터 모델이다. 다양한 로봇 태스크를 단일 모델로 처리한다.

PaLM-E는 "positive transfer"를 보였다는 점에서 주목할 만하다. 로봇 데이터, 웹 이미지, 텍스트를 함께 학습하면 각각을 따로 학습했을 때보다 로봇 태스크 성능이 올라간다. 범용 지식이 로봇 행동에도 도움이 될 수 있음을 실증한 사례다.

> **추천 자료**
> - [Driess et al., "PaLM-E: An Embodied Multimodal Language Model" (2023)](https://arxiv.org/abs/2303.03378) — PaLM-E 원 논문

### 12.2.3 OpenVLA

RT-2와 PaLM-E의 전체 가중치는 공개되지 않았다. OpenVLA는 코드와 가중치가 공개되어, 필요한 연산 자원이 있는 연구실에서 다운로드해 파인튜닝하거나 로봇에 배포할 수 있다.

OpenVLA는 7B 파라미터(Llama 2 기반)로, 970K 에피소드로 학습됐다. 다양한 로봇 embodiment에 적용할 수 있다.

```python
# OpenVLA 사용 예시 (개념)
from openvla import OpenVLAModel

model = OpenVLAModel.from_pretrained("openvla/openvla-7b")

action = model.predict(
    image=current_image,
    instruction="pick up the blue block and place it on the red target"
)
```

RT-X 프로젝트도 OpenVLA와 함께 살펴볼 만하다. 여러 연구 기관이 수집한 로봇 데이터를 모은 Open X-Embodiment 데이터셋으로 범용 로봇 정책을 학습한다. 이 데이터셋에는 22개 이상의 로봇 유형에서 나온 데이터가 포함된다.

**Octo**: RT-X 데이터로 학습한 또 다른 오픈소스 모델이다. OpenVLA보다 작은 93M 파라미터 규모라 더 가볍게 활용할 수 있으며, 다양한 로봇 플랫폼에 빠르게 파인튜닝하도록 설계됐다.

> **추천 자료**
> - [OpenVLA GitHub](https://github.com/openvla/openvla) — 코드와 모델 가중치 공개
> - [Kim et al., "OpenVLA" (2024)](https://arxiv.org/abs/2406.09246) — OpenVLA 논문
> - [Open X-Embodiment Collaboration, "Open X-Embodiment" (2023)](https://arxiv.org/abs/2310.08864) — RT-X 데이터셋 논문
> - [Octo GitHub](https://github.com/octo-models/octo) — 경량 오픈소스 로봇 정책 모델

### 12.2.4 Navigation 관련

물체 조작(manipulation)뿐 아니라 환경 안에서의 이동(navigation)도 Embodied AI의 핵심 과제다. 아래 연구들은 LLM의 언어 이해 능력을 navigation에 활용한다.

LINGO는 Language-guided Indoor Navigation이다.

**SayCan**: LLM이 "할 수 있는 것"과 "해야 하는 것"을 분리
- Affordance function: 로봇이 현재 할 수 있는 행동
- LLM: 목표 달성을 위해 해야 하는 행동

예를 들어 LLM은 "커피 만들어줘"라는 요청에 "1. 컵을 잡아 2. 커피 머신으로 가 3. 버튼을 눌러..."처럼 계획을 세울 수 있다. 그러나 로봇이 컵 근처에 없다면 "컵을 잡아"는 실행할 수 없다. SayCan은 LLM의 계획(해야 하는 것)과 로봇이 현재 할 수 있는 행동을 함께 고려해, 실행 가능하면서 목표에 가까운 행동을 고른다.

> **추천 자료**
> - [Ahn et al., "Do As I Can, Not As I Say: Grounding Language in Robotic Affordances" (2022)](https://arxiv.org/abs/2204.01691) — SayCan 논문
> - [SayCan project page](https://say-can.github.io/) — 데모 영상 포함

## 12.3 World Models

실제 로봇에서 시행착오를 거듭하면 시간과 장비 비용이 커진다. **World Model**은 현재 상태와 행동으로부터 다음 상태나 관측을 예측한다. 이 예측으로 실제 장비에서 모든 후보 행동을 실행하지 않고 model-based policy를 평가할 수 있다.

World model은 model-based RL의 rollout과 위험한 행동의 사전 평가에 사용할 수 있다.

자율주행에서는 GAIA-1(Wayve)이 행동 조건부 주행 영상을 예측하고, DriveDreamer가 텍스트 조건의 주행 장면을 생성하며, MILE이 implicit world model에서 미래 상태와 주행 정책을 함께 학습한다.

구조는 상태 공간 모델과 비슷하다.

```
z_{t+1} = f(z_t, a_t)     # Dynamics model (현재 상태 + 행동 → 다음 상태)
o_t = g(z_t)              # Observation model (잠재 상태 → 관측)
r_t = h(z_t, a_t)         # Reward model (보상 예측)
```

선형대수 시간에 배운 x_{t+1} = Ax_t + Bu_t 가 비선형 신경망 버전으로 확장된 것이라고 보면 된다.

> **추천 자료**
> - [Hu et al., "GAIA-1: A Generative World Model for Autonomous Driving" (2023)](https://arxiv.org/abs/2309.17080) — Wayve의 World Model 논문
> - [Wang et al., "DriveDreamer" (2023)](https://arxiv.org/abs/2309.09777) — 주행 시나리오 생성 논문
> - [Yannic Kilcher — World Models Explained](https://www.youtube.com/watch?v=dPsXxLyqpfs) — World Model 개념 설명 영상

## 12.4 End-to-End vs Modular

End-to-end와 modular architecture는 인식, 계획, 제어를 어디에서 분리할지에 대한 두 가지 설계 방식이다.

End-to-End:

```
센서 입력 → [단일 신경망] → 행동 출력
```

- 장점: 간단한 파이프라인, 중간 표현의 bottleneck 없음
- 단점: Interpretability 부족, 대규모 데이터 필요
- 예시: NVIDIA PilotNet, Tesla FSD (추정)

자율주행에서는 End-to-End 방식이 여러 형태로 발전했다. UniAD(Unified Autonomous Driving, 2023)는 End-to-End이면서도 내부에 detection, tracking, mapping, prediction, planning 모듈을 두어 해석 가능성을 유지한다. CVPR 2023 Best Paper. VAD(2023)는 장면을 벡터화된 기하 표현으로 변환하여 효율적인 주행 정책을 유도하며, GenAD(2024)는 생성형 파이프라인을 바탕으로 다채로운 도로 시나리오에 걸친 일반화 성능을 확보한다.

Modular:

```
센서 → [인식] → [예측] → [계획] → [제어] → 행동
```

- 장점: 각 모듈 독립적 개발/디버깅, 해석 가능
- 단점: 모듈 간 정보 손실, 최적화 어려움
- 예시: Apollo, Autoware

최근 트렌드는 하이브리드다. 인식은 학습 기반, 계획·제어는 모델 기반으로 안전성을 확보하되, UniAD처럼 End-to-End 프레임워크 안에 명시적 모듈을 배치한다.

> **추천 자료**
> - [Hu et al., "Planning-oriented Autonomous Driving (UniAD)" (2023)](https://arxiv.org/abs/2212.10156) — CVPR 2023 Best Paper
> - [Jiang et al., "VAD" (2023)](https://arxiv.org/abs/2303.12077) — 벡터화 기반 자율주행
> - [Andrej Karpathy — Tesla AI Day 2022 Presentation](https://www.youtube.com/watch?v=ODSJsviD_SU) — End-to-End 자율주행 실무 관점

### End-to-End vs Modular: 2026년 현실

End-to-end와 모듈형 중 어느 쪽이 우세한지는 응용과 검증 조건에 따라 달라진다. 실제 시스템에서 모듈형이나 하이브리드 구성을 선택하는 이유는 다음과 같다.

- 디버깅: end-to-end 모델이 실패했을 때 원인을 찾기 어렵다. "왜 로봇이 컵을 놓쳤는가?"에 대해, 모듈형이면 "depth estimation이 틀렸다" 또는 "grasp planning이 잘못됐다"로 좁힐 수 있지만, end-to-end에서는 어디서 틀렸는지 모른다.
- 안전 보장: 모듈형은 각 모듈에 safety check를 넣을 수 있다 (속도 제한, 충돌 감지 등). End-to-end에서 이런 보장을 넣기 어렵다.
- 부분 업데이트: perception 모듈만 개선하고 싶을 때, 모듈형이면 해당 모듈만 교체할 수 있다. End-to-end에서는 변경 범위에 따라 여러 구성요소를 함께 재학습하거나 재검증해야 할 수 있다.
- 데이터 효율: 이 장에서 다룬 범용 정책은 대규모 데이터를 사용했다. RT-1은 약 13만 에피소드의 시연 데이터를 활용했으며, OpenVLA에 이르러서는 97만 에피소드 규모의 방대한 데이터셋으로 학습이 이뤄졌다. 대부분의 연구실이 같은 규모의 데이터를 직접 수집하기는 어렵다.

현실적인 선택지 가운데 하나는 하이브리드 구성이다. 인식에는 VFM을 쓰되 계획·제어에는 명시적 모듈과 안전 검사를 둔다. 연구실의 Local/Global Module 설계(18장)도 이 방향에 속한다.

end-to-end가 모듈형을 넓게 대체하려면 디버깅 가능성, 소규모 데이터에서의 학습 효율, safety guarantee를 응용별 검증 기준으로 입증해야 한다. 현재 공개 시스템은 이 조건들을 서로 다른 범위에서 다루며, 하나의 구조가 모두 해결했다고 보기는 어렵다.

## 12.5 Spatial AI + VLA 통합

VLA가 "커피 가져와" 같은 고수준 장기 작업을 의미론적으로 분해한다면, 로컬 제어기는 실시간 동적 장애물 회피와 차체 자세 안정화를 전담한다. 실제 시스템에서는 서로 다른 시간척도의 두 출력을 연결해야 한다.

연구실의 2-Module Architecture와 연결:

Local(Fast) Perception은 geometric understanding을 담당한다. 깊이, 장애물, 자세를 실시간(10–100 Hz)으로 처리하며, classical 또는 경량 학습 모델을 쓴다.

Global(Heavy) Understanding은 semantic understanding 담당이다. 객체, 관계, 맥락을 VFM/VLA 기반으로 처리하고, 서버 또는 클라우드에서 1–10 Hz로 실행된다.

통합 시나리오:

```
1. Local: 실시간 obstacle avoidance, odometry
2. Global: "kitchen에서 cup을 찾아서 table로 가져와"
   - VLM으로 cup 인식
   - Semantic map에서 경로 계획
3. Local이 Global의 waypoint를 받아 실제 이동 수행
```

## 12.6 Sim-to-Real & Simulation Platforms

실제 로봇으로 데이터를 모으는 일은 시간과 비용이 많이 들고 위험하다. 그래서 시뮬레이션에서 먼저 학습한 뒤 실제 로봇으로 옮기는 Sim-to-Real을 많이 쓴다. 다만 시뮬레이션과 현실 사이에는 "Reality Gap"이 있다. 다음 기법들은 이 간극을 줄이는 데 쓰인다.

**Domain Randomization**: 시뮬레이션에서 텍스처, 조명, 물리 파라미터 등을 무작위로 변경하여 학습한다. 모델이 다양한 조건에 노출되면, 현실 환경도 그 중 하나의 변형(variation)으로 처리할 수 있게 된다.

대표적인 시뮬레이션 플랫폼은 네 가지다. NVIDIA Isaac Sim/Lab은 GPU 가속 물리 시뮬레이션으로 수천 개의 환경을 병렬로 돌릴 수 있어 대규모 강화학습에 맞는다. Isaac Lab은 로봇 학습 연구를 위한 통합 프레임워크다. AI2-THOR(Allen Institute)는 주방·거실 같은 가정 환경에서 물체 조작(manipulation)을 연습할 수 있는 실내 시뮬레이터다. Habitat(Meta)는 Matterport3D·Gibson 같은 대규모 3D 스캔 환경에서 내비게이션을 학습하며, Habitat Challenge를 통해 매년 벤치마크를 제공한다. MuJoCo는 접촉(contact) 물리에 강점이 있어 로봇 팔 조작과 보행 학습에 널리 쓰이며, DeepMind가 인수한 뒤 오픈소스로 전환됐다.

> **추천 자료**
> - [NVIDIA Isaac Lab Documentation](https://isaac-sim.github.io/IsaacLab/) — 로봇 학습을 위한 시뮬레이션 프레임워크
> - [AI2-THOR Documentation](https://ai2thor.allenai.org/) — 실내 환경 시뮬레이터
> - [Habitat Documentation](https://aihabitat.org/) — Meta의 Embodied AI 플랫폼
> - [Tobin et al., "Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World" (2017)](https://arxiv.org/abs/1703.06907) — Domain Randomization 원 논문

## 12.7 심화: Imitation Learning

VLA와 Embodied AI의 정책(policy) 학습은 크게 강화학습(RL)과 모방학습(IL)으로 나뉜다. 두 접근은 데이터 수집 방식, 보상 설계, 실제 로봇에서의 탐색 비용이 다르다.

**Behavioral Cloning (BC)**

가장 단순한 IL 방법이다. 전문가(사람 또는 사전 설계된 스크립트) 시연 데이터 `{(s_t, a_t)}`를 확보한 뒤, 임의 상태 `s_t`에 대응하는 행동 `a_t`를 예측하도록 지도학습(supervised learning)을 수행한다.

```
Loss = E[ || π_θ(s_t) - a_t ||^2 ]
```

간단하고 구현하기 쉽지만 **distribution shift**라는 한계가 있다. 학습 때는 전문가의 상태 분포를 따르지만, 추론 때는 정책의 불완전한 행동이 다음 상태를 결정한다. 작은 오차가 누적되면 전문가가 방문하지 않은 상태에 이르고, 정책은 그 상태에서 적절한 행동을 고르지 못한다.

**DAgger (Dataset Aggregation)**

DAgger는 distribution shift를 완화하는 대표적 방법이다. 학습된 정책으로 데이터를 수집하면서 전문가에게 라벨을 받아 데이터셋에 추가한다.

```
1. 초기 데이터 D = {전문가 시연}으로 정책 π_1 학습
2. for i = 1, 2, ...
     π_i로 rollout 수행 → 방문한 상태 {s_t} 수집
     전문가에게 {s_t}에서의 행동 {a_t^*}를 질의
     D = D ∪ {(s_t, a_t^*)}
     D로 π_{i+1} 학습
```

전문가에게 매번 질의하는 건 비싸기 때문에, human-in-the-loop 변형이나 DAgger의 근사 버전(HG-DAgger, ThriftyDAgger 등)이 쓰인다.

두 접근인 RL과 IL의 전형적 차이는 다음과 같다. 실제 표본 수와 안전성은 알고리즘, 시뮬레이터, 전문가 데이터의 품질에 따라 달라진다.

| 기준 | RL | IL |
|------|----|----|
| Sample efficiency | 환경 상호작용이 많이 필요할 수 있음 | 전문가 시연 수와 다양성에 좌우됨 |
| 보상 함수 | 직접 설계해야 함 (reward engineering) | 불필요 |
| 안전성 | 탐색(exploration) 중 위험한 행동 가능 | 전문가 행동 모방이므로 상대적으로 안전 |
| Sim-to-Real | 보상 함수의 sim-real gap도 문제 | 실제 시연 데이터를 쓰면 gap 감소 |

로보틱스에서 보상 함수를 제대로 설계하는 것은 매우 어렵다. "컵을 잡아라"의 보상을 어떻게 정의할 것인가? 컵과 그리퍼 사이의 거리? 그러면 로봇이 컵 옆에만 가서 멈출 수 있다. 잡았는지 여부? 그러면 sparse reward 문제가 생긴다. IL은 이 문제를 우회한다.

> **추천 자료**
> - [Ross et al., "A Reduction of Imitation Learning and Structured Prediction to No-Regret Online Learning" (2011)](https://arxiv.org/abs/1011.0686) — DAgger 원 논문
> - [Florence et al., "Implicit Behavioral Cloning" (CoRL 2021)](https://arxiv.org/abs/2109.00137) — BC의 한계를 극복하기 위한 implicit 접근
> - [Zare et al., "A Survey of Imitation Learning" (2024)](https://arxiv.org/abs/2309.15894) — IL 전반 서베이

## 12.8 심화: Diffusion Policy

Chi et al.(RSS 2023)이 제안한 Diffusion Policy는 로봇 조작(manipulation) 분야에서 BC 계열 방법의 대안으로 쓰이는 정책 표현 방식이다. 행동 시퀀스(action trajectory)를 denoising diffusion 과정으로 생성한다.

기존 BC는 `π_θ(s) → a`로 단일 action을 결정론적으로 예측한다. 그러나 현실에서는 같은 상태에서도 여러 행동이 가능하다(multi-modality). 테이블 위의 컵은 왼쪽에서도, 오른쪽에서도 잡을 수 있다. 결정론적 BC는 이 두 행동의 평균을 출력해 어느 쪽도 성공하지 못할 수 있다. Gaussian Mixture Model 같은 방법도 있지만 모드 수를 미리 정해야 한다.

Diffusion Policy는 denoising 과정을 이용해 이 multi-modal 분포를 표현한다.

동작은 다음 순서를 따른다.

```
1. 랜덤 노이즈 a_T ~ N(0, I)에서 시작 (T = diffusion steps)
2. 현재 관측 s를 조건으로 반복적으로 denoising:
   a_{t-1} = denoise_θ(a_t, s, t)    for t = T, T-1, ..., 1
3. 최종 a_0가 실행할 action trajectory
```

Action trajectory는 단일 action이 아니라 미래 수 스텝의 action 시퀀스 `[a_0, a_1, ..., a_H]`이다. 이 중 처음 몇 스텝만 실행하고(receding horizon), 다시 새 관측으로 다음 trajectory를 생성한다.

이 방식은 multi-modal action distribution의 모드 수를 미리 정할 필요가 없고, action sequence를 한 번에 생성해 시간적으로 이어진 행동을 만들 수 있다. 학습에는 denoising score matching을 쓴다.

단점은 속도다. Inference 때는 denoising 과정을 10~100 steps 반복해야 하므로 실시간 제어(>100 Hz)에는 맞지 않을 수 있다. DDIM 같은 가속 기법이나 consistency distillation으로 이를 완화할 수 있다.

Chi et al.의 원 논문 프로젝트 페이지는 4개 벤치마크의 12개 태스크에서 기존 로봇 학습 방법보다 평균 46.9% 향상됐다고 보고한다. 이 수치는 논문이 사용한 태스크·평가지표·baseline에 한정해 해석해야 한다.

> **추천 자료**
> - [Chi et al., "Diffusion Policy: Visuomotor Policy Learning via Action Diffusion" (RSS 2023)](https://arxiv.org/abs/2303.04137) — Diffusion Policy 원 논문
> - [Diffusion Policy 프로젝트 페이지](https://diffusion-policy.cs.columbia.edu/) — 코드, 데모, 영상
> - [Ho et al., "Denoising Diffusion Probabilistic Models" (NeurIPS 2020)](https://arxiv.org/abs/2006.11239) — Diffusion model 기초 논문

## 12.9 심화: Sim-to-Real Transfer

12.6에서 시뮬레이션 플랫폼과 Domain Randomization을 간략히 다뤘다. 여기서는 Sim-to-Real transfer의 구체적 기법들을 살펴본다.

**1. Domain Randomization (DR)**

시뮬레이션 환경의 파라미터를 학습 시마다 무작위로 변경한다. 모델이 충분히 다양한 조건에서 학습하면, 현실 환경이 그 변형 중 하나에 포함될 것이라는 가정이다.

랜덤화 대상:
- **시각적(Visual)**: 텍스처, 조명 방향/세기, 카메라 위치/시야각, 배경
- **물리적(Physical)**: 마찰 계수, 관성 모멘트, 링크 질량, 관절 감쇠(damping)
- **동역학(Dynamics)**: actuator 지연, 센서 노이즈, 제어 주기

DR의 한계: 랜덤화 범위를 너무 넓히면 학습 자체가 어려워지고, 너무 좁히면 현실을 커버하지 못한다. 적절한 범위를 찾는 것이 중요하다.

**2. System Identification (SysID)**

실제 시스템의 물리 파라미터를 측정하거나 추정해 시뮬레이터를 보정한다.

```
1. 실제 로봇에서 특정 trajectory를 실행하여 데이터 수집
2. 시뮬레이터의 파라미터 φ를 최적화:
   φ* = argmin_φ || f_sim(φ) - f_real ||^2
3. 보정된 시뮬레이터에서 정책 학습
```

전통적이고 효과적이지만, 모든 파라미터를 정확히 추정하기는 어렵고, 시뮬레이터가 모델링하지 않는 현상(케이블의 유연함, 접촉면의 미세 변형 등)에는 무력하다.

**3. Real-to-Sim-to-Real (R2S2R)**

DR의 다양성과 SysID의 정밀도를 결합하는 접근이다.

```
1. 실제 데이터를 소량 수집
2. 실제 데이터로 시뮬레이터를 교정 (SysID) 또는 차이를 모델링
3. 교정된 시뮬레이터에서 정책 학습
4. 학습된 정책을 실제 로봇에 적용
5. (반복) 실제 결과로 시뮬레이터를 다시 교정
```

**4. Transfer 성공 여부 판단**

정량적 판단 기준은 sim과 real에서 동일 태스크의 성공률 비교다.

- Sim 성공률 ≈ Real 성공률: transfer 성공. 시뮬레이터가 현실을 잘 반영.
- Sim >> Real: reality gap이 큼. DR 범위 확장 또는 SysID 보정 필요.
- Sim < Real: 드물지만 발생. 시뮬레이터가 오히려 더 어려운 조건(보수적)으로 설정된 경우.

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
> - **2024~**: OpenVLA, Octo 등 오픈소스 VLA 모델 공개. World Model 기반 계획(planning), End-to-End 자율주행(UniAD, VAD, GenAD), modular·hybrid 설계가 함께 연구됨
> - **최근 흐름**: 2023년 이후 RT-2, OpenVLA, Octo, pi0 등 Foundation Model 기반 로봇 정책이 발표됐다. 이 가운데 OpenVLA와 Octo는 공개된 코드와 가중치로 추가 학습을 실험할 수 있다.
