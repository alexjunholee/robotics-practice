# Ch.12 — Vision-Language-Action (VLA) & Embodied AI

VLA 모델은 "빨간 컵을 집어서 테이블 위에 놓아줘" 같은 자연어 명령과 카메라 영상을 받아 로봇 행동을 출력한다.

## 12.1 VLA 개념

기존 로봇 시스템은 시각 인식, 언어 이해, 행동 생성을 각각 별도 파이프라인으로 구현했다. VLA (Vision-Language-Action) 모델은 세 역할을 한 모델에서 처리한다.

```
입력: 이미지 + 자연어 명령 ("pick up the red cup")
출력: 로봇 행동 (관절 각도, gripper 명령 등)
```

**Embodied AI**: 물리적 환경에서 상호작용하며 학습하는 AI
- 인식과 행동을 함께 다룸
- 시뮬레이션과 실제 환경의 간극 (Sim-to-Real)

컵을 집어 올리는 것처럼 로봇이 물체를 건드리는 순간 중력, 마찰, 충돌이 결과에 끼어들어 고려할 변수가 늘어난다.

> **추천 자료**
> - [Google DeepMind Robotics Blog](https://deepmind.google/blog/) — RT-1, RT-2, PaLM-E 등의 공식 블로그 포스트

## 12.2 주요 모델 및 연구

### 12.2.1 RT-1, RT-2 (Google DeepMind)

RT-1(Robotics Transformer 1)은 700개 이상의 태스크를 담은 약 13만 에피소드의 로봇 시연 데이터로 학습했고, 행동을 토큰으로 출력한다.

RT-2(Robotics Transformer 2)는 PaLI-X와 PaLM-E 같은 VLM(시각-언어 모델)을 로봇 행동을 출력하도록 파인튜닝(fine-tuning)한다. 원 논문은 웹 데이터에서 배운 지식이 로봇 작업으로 옮겨 가는지 실험했고, 행동 전에 계획을 글로 먼저 쓰게 하는 robot chain-of-thought 실험도 따로 실었다.

원 논문의 실험에서 RT-2는 로봇 데이터에 없던 물체와 지시도 웹 데이터에서 얻은 지식을 써서 제로샷(zero-shot)으로 처리했다.

> **추천 자료**
> - [Google DeepMind — RT-2 Demo Video](https://deepmind.google/blog/rt-2-new-model-translates-vision-and-language-into-action/) — RT-2의 실제 동작 영상과 설명
> - [Brohan et al., "RT-1: Robotics Transformer" (RSS 2023)](https://arxiv.org/abs/2212.06817) — RT-1 원 논문
> - [Brohan et al., "RT-2: Vision-Language-Action Models" (CoRL 2023)](https://arxiv.org/abs/2307.15818) — RT-2 원 논문

### 12.2.2 PaLM-E

PaLM-E는 PaLM(540B, 언어)과 ViT-22B(비전)를 결합하고 로봇 상태를 입력으로 받는 562B 파라미터 Embodied Multimodal Language Model이다. 여러 로봇 태스크를 한 모델로 처리한다. 저자들은 로봇 데이터, 웹 이미지, 텍스트를 함께 학습했을 때 로봇 태스크 성능이 로봇 데이터만으로 학습했을 때보다 올라가는 "positive transfer"를 보고했다.

> **추천 자료**
> - [Driess et al., "PaLM-E: An Embodied Multimodal Language Model" (2023)](https://arxiv.org/abs/2303.03378) — PaLM-E 원 논문

### 12.2.3 OpenVLA

RT-2와 PaLM-E는 가중치를 공개하지 않았다. OpenVLA는 코드와 가중치를 공개해서, 연산 자원을 갖춘 연구실은 내려받아 파인튜닝하거나 로봇에 배포할 수 있다. OpenVLA는 Llama 2 기반 7B 파라미터 모델이고, Open X-Embodiment의 970K 에피소드로 학습했다.

```python
# OpenVLA 추론 예시 (공식 README의 HuggingFace 인터페이스)
import torch
from transformers import AutoModelForVision2Seq, AutoProcessor

processor = AutoProcessor.from_pretrained("openvla/openvla-7b", trust_remote_code=True)
vla = AutoModelForVision2Seq.from_pretrained(
    "openvla/openvla-7b",
    torch_dtype=torch.bfloat16,
    low_cpu_mem_usage=True,
    trust_remote_code=True,
).to("cuda:0")

instruction = "pick up the blue block and place it on the red target"
prompt = f"In: What action should the robot take to {instruction}?\nOut:"
inputs = processor(prompt, current_image).to("cuda:0", dtype=torch.bfloat16)  # current_image: PIL.Image
# 7-DoF 행동. unnorm_key는 학습 데이터셋의 정규화 통계를 고른다(예: BridgeData V2)
action = vla.predict_action(**inputs, unnorm_key="bridge_orig", do_sample=False)
```

RT-X 프로젝트는 21개 기관이 22종의 로봇으로 모은 데이터를 Open X-Embodiment 데이터셋으로 묶고, 이 데이터로 범용 로봇 정책을 학습했다.

Octo: Open X-Embodiment의 800K 궤적으로 학습한 오픈소스 정책이다. 큰 버전이 93M 파라미터로 OpenVLA보다 훨씬 작고, 새 로봇의 센서와 행동 공간에 맞춰 파인튜닝하도록 설계됐다.

> **추천 자료**
> - [OpenVLA GitHub](https://github.com/openvla/openvla) — 코드와 모델 가중치 공개
> - [Kim et al., "OpenVLA" (2024)](https://arxiv.org/abs/2406.09246) — OpenVLA 논문
> - [Open X-Embodiment Collaboration, "Open X-Embodiment" (ICRA 2024)](https://arxiv.org/abs/2310.08864) — RT-X 데이터셋 논문
> - [Octo GitHub](https://github.com/octo-models/octo) — 경량 오픈소스 로봇 정책 모델

### 12.2.4 Navigation 관련

Embodied AI는 물체 조작(manipulation) 말고도 공간을 옮겨 다니는 일(navigation)과 긴 작업의 계획도 다룬다. 아래 연구는 LLM의 언어 이해를 주행과 작업 계획에 쓴다.

LINGO는 Wayve가 공개한 주행용 모델 계열이다. LINGO-1은 주행 장면을 언어로 해설하는 개방 루프 모델이다. LINGO-2는 언어 지시를 받아 차량을 직접 제어하는 폐쇄 루프 모델이다.

SayCan: 행동 후보마다 두 점수를 곱해 고른다
- Affordance function: 로봇이 현재 할 수 있는 행동
- LLM: 목표 달성을 위해 해야 하는 행동

LLM은 "커피 만들어줘"라는 요청에 "1. 컵을 잡아 2. 커피 머신으로 가 3. 버튼을 눌러..."처럼 계획을 세운다. 로봇이 컵에서 멀리 있으면 "컵을 잡아"를 실행해도 실패한다. SayCan은 LLM이 매긴 점수(목표에 얼마나 도움이 되는가)와 affordance value function이 매긴 점수(지금 상태에서 성공할 확률)를 곱해, 둘 다 높은 행동을 고른다.

> **추천 자료**
> - [Ahn et al., "Do As I Can, Not As I Say: Grounding Language in Robotic Affordances" (CoRL 2022)](https://arxiv.org/abs/2204.01691) — SayCan 논문
> - [SayCan project page](https://say-can.github.io/) — 데모 영상 포함

## 12.3 World Models

World Model은 현재 상태와 행동을 받아 다음 상태나 관측을 예측한다. 후보 행동을 이 모델 안에서 먼저 시험해 보면 실제 장비를 움직이는 횟수가 줄어든다. model-based RL에서 rollout(모델 안에서 미리 굴려 본 상태·행동 궤적)을 만들 때와 위험한 행동을 실행 전에 평가할 때 world model을 쓴다.

자율주행에서도 world model을 쓴다. Wayve의 GAIA-1은 차량 행동을 조건으로 받아 앞으로 이어질 주행 영상을 예측한다. DriveDreamer는 HD 지도와 3D 박스 같은 도로 구조 정보와 차량 행동을 조건으로 받아 주행 영상을 만들고, 텍스트로 날씨나 시간대를 바꾼다. MILE은 잠재 공간의 world model 안에서 미래 상태와 주행 정책을 함께 학습한다.

식으로 쓰면 상태 공간 모델과 같은 꼴이다.

```
z_{t+1} = f(z_t, a_t)     # Dynamics model (현재 상태 + 행동 → 다음 상태)
o_t = g(z_t)              # Observation model (잠재 상태 → 관측)
r_t = h(z_t, a_t)         # Reward model (보상 예측)
```

선형 상태 공간 모델 x_{t+1} = Ax_t + Bu_t에서 A, B 자리에 신경망을 넣고, 상태를 학습된 잠재 공간에서 다룬다.

> **추천 자료**
> - [Hu et al., "GAIA-1: A Generative World Model for Autonomous Driving" (2023)](https://arxiv.org/abs/2309.17080) — Wayve의 World Model 논문
> - [Wang et al., "DriveDreamer" (2023)](https://arxiv.org/abs/2309.09777) — 주행 시나리오 생성 논문
> - [Yannic Kilcher — World Models Explained](https://www.youtube.com/watch?v=dPsXxLyqpfs) — World Model 개념 설명 영상

## 12.4 End-to-End vs Modular

인식, 계획, 제어를 한 신경망에 넣을지 모듈로 나눌지에 따라 설계가 갈린다.

End-to-End:

```
센서 입력 → [단일 신경망] → 행동 출력
```

- 장점: 간단한 파이프라인, 사람이 정한 중간 표현에서 생기는 정보 손실을 피함
- 단점: Interpretability(해석 가능성) 부족, 대규모 데이터 필요
- 예시: NVIDIA PilotNet, Tesla FSD v12 이후(Tesla 릴리스 노트가 도심 주행 스택을 단일 end-to-end 신경망으로 바꿨다고 밝힘)

자율주행의 End-to-End 모델은 여러 형태로 나왔다. UniAD(CVPR 2023 Best Paper)는 detection, tracking, mapping, prediction, planning 모듈을 한 네트워크 안에 두고 planning을 목표로 함께 학습한다. 중간 모듈의 출력을 볼 수 있어 해석하기 쉽다. VAD(2023)는 장면을 벡터화된 기하 표현으로 바꿔 planning 연산량을 줄였다. GenAD(2024)는 주변 차량과 자차의 미래 궤적을 생성 모델로 함께 예측한다.

Modular:

```
센서 → [인식] → [예측] → [계획] → [제어] → 행동
```

- 장점: 각 모듈 독립적 개발/디버깅, 해석 가능
- 단점: 모듈 간 정보 손실, 최적화 어려움
- 예시: Apollo, Autoware

하이브리드 구성도 많이 쓴다. 인식에 학습 모델을 쓰고 계획·제어에 모델 기반 방법을 둔다. UniAD처럼 End-to-End 네트워크 안에 명시적 모듈을 넣는 방식도 있다.

> **추천 자료**
> - [Hu et al., "Planning-oriented Autonomous Driving (UniAD)" (2023)](https://arxiv.org/abs/2212.10156) — CVPR 2023 Best Paper
> - [Jiang et al., "VAD" (2023)](https://arxiv.org/abs/2303.12077) — 벡터화 기반 자율주행
> - [Tesla AI Day 2022 Presentation](https://www.youtube.com/watch?v=ODSJsviD_SU) — FSD v12 이전의 인식(occupancy network)·계획 모듈 구조를 설명한 발표. 위 Modular·하이브리드 구성과 비교해 볼 수 있다

### End-to-End vs Modular: 2026년 현실

어느 구조를 고를지는 응용과 검증 조건에 따라 정한다. 실제 시스템에서 모듈형이나 하이브리드 구성을 고르는 이유는 다음과 같다.

- 디버깅: 로봇이 컵을 놓쳤을 때 모듈형 시스템에서는 각 모듈 출력을 확인해 "depth estimation이 틀렸다"나 "grasp planning이 잘못됐다"로 원인을 좁힌다. end-to-end 모델에서는 입력과 출력만 보이므로 원인을 찾는 데 시간이 더 든다.
- 안전 보장: 모듈형은 각 모듈 사이에 safety check(속도 제한, 충돌 감지 등)를 넣는다. End-to-end 모델에는 출력 뒤에 별도의 안전 계층을 붙여야 한다.
- 부분 업데이트: 모듈형에서는 perception 모듈만 교체한다. End-to-end에서는 변경 범위에 따라 네트워크 전체를 다시 학습하고 다시 검증한다.
- 데이터 효율: RT-1은 약 13만 에피소드, OpenVLA는 97만 에피소드로 학습했다. 연구실 하나가 이 규모의 데이터를 직접 모으려면 로봇과 사람 시간이 많이 든다.

연구실의 Local/Global Module 설계(18장)도 앞의 하이브리드 구성처럼 인식에 VFM을 쓰고, 계획·제어에 명시적 모듈과 안전 검사를 둔다. end-to-end 모델을 넓게 쓰려면 디버깅할 수 있는지, 데이터가 적을 때도 잘 배우는지, 안전을 보장하는지를 응용마다 정한 검증 기준으로 보여야 한다.

## 12.5 Spatial AI + VLA 통합

VLA는 "커피 가져와" 같은 장기 작업을 하위 단계로 나눈다. 장애물 회피와 자세 안정화는 로컬 제어기가 실시간으로 처리한다. 실제 시스템에서는 시간척도가 다른 두 출력을 이어 붙인다.

연구실의 2-Module Architecture와 연결:

Local(Fast) Perception은 깊이, 장애물, 자세 같은 기하 정보를 10–100 Hz로 처리한다. classical 방법이나 경량 학습 모델을 쓴다.

Global(Heavy) Understanding은 객체, 관계, 맥락 같은 의미 정보를 VFM/VLA로 처리한다. 서버나 클라우드에서 1–10 Hz로 돈다.

통합 시나리오:

```
1. Local: 실시간 obstacle avoidance, odometry
2. Global: "kitchen에서 cup을 찾아서 table로 가져와"
   - VLM으로 cup 인식
   - Semantic map에서 경로 계획
3. Local이 Global의 waypoint를 받아 실제 이동 수행
```

## 12.6 Sim-to-Real & Simulation Platforms

실제 로봇으로 데이터를 모으는 일은 시간과 비용이 많이 들고 위험하다. 그래서 시뮬레이션에서 먼저 학습한 뒤 실제 로봇으로 옮기는 Sim-to-Real을 많이 쓴다. 시뮬레이션과 현실의 차이(Reality Gap)는 아래 기법으로 줄인다.

Domain Randomization: 시뮬레이션의 텍스처, 조명, 물리 파라미터 등을 무작위로 바꿔 가며 학습한다. Ch.8 §8.5에서 처음 다뤘고, 이 장에서는 §12.9에서 랜덤화 대상과 범위를 정리한다.

NVIDIA Isaac Sim/Lab은 GPU 가속 물리 시뮬레이션으로 수천 개의 환경을 병렬로 돌려 대규모 강화학습에 쓴다. Isaac Lab은 로봇 학습 연구를 위한 통합 프레임워크다. AI2-THOR(Allen Institute)는 주방·거실 같은 가정 환경에서 물체 조작(manipulation)을 연습할 수 있는 실내 시뮬레이터다. Habitat(Meta)에서는 Matterport3D·Gibson 같은 대규모 3D 스캔 환경에서 내비게이션을 학습하고, Habitat Challenge로 성능을 비교한다. MuJoCo는 접촉(contact) 물리를 빠르고 안정적으로 계산해 로봇 팔 조작과 보행 학습에 널리 쓴다. DeepMind가 2021년에 인수했고 2022년에 오픈소스로 공개했다.

> **추천 자료**
> - [NVIDIA Isaac Lab Documentation](https://isaac-sim.github.io/IsaacLab/) — 로봇 학습을 위한 시뮬레이션 프레임워크
> - [AI2-THOR Documentation](https://ai2thor.allenai.org/) — 실내 환경 시뮬레이터
> - [Habitat Documentation](https://aihabitat.org/) — Meta의 Embodied AI 플랫폼
> - [Tobin et al., "Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World" (2017)](https://arxiv.org/abs/1703.06907) — Domain Randomization 원 논문

## 12.7 심화: Imitation Learning

VLA와 Embodied AI의 정책(policy)은 주로 강화학습(RL)이나 모방학습(IL)으로 학습한다. 데이터 수집 방식, 보상 설계, 실제 로봇으로 탐색할 때 드는 비용이 방법마다 다르다.

**Behavioral Cloning (BC)**

가장 단순한 IL 방법이다. 전문가(사람 또는 사전 설계된 스크립트)의 시연 데이터 `{(s_t, a_t)}`를 모은 뒤, 상태 `s_t`에서 행동 `a_t`를 예측하도록 지도학습(supervised learning)한다.

```
Loss = E[ || π_θ(s_t) - a_t ||^2 ]
```

구현은 쉽지만 distribution shift 문제가 있다. 학습 데이터의 상태는 전문가가 지나간 상태다. 실행할 때는 정책 자신의 행동이 다음 상태를 정한다. 작은 오차가 쌓이면 로봇은 전문가가 가 본 적 없는 상태에 들어가고, 정책은 그 상태에서 엉뚱한 행동을 낸다.

**DAgger (Dataset Aggregation)**

DAgger는 distribution shift를 줄이는 방법이다. 학습된 정책으로 데이터를 수집하면서 전문가에게 라벨을 받아 데이터셋에 추가한다.

```
1. 초기 데이터 D = {전문가 시연}으로 정책 π_1 학습
2. for i = 1, 2, ...
     π_i로 rollout 수행 → 방문한 상태 {s_t} 수집
     전문가에게 {s_t}에서의 행동 {a_t^*}를 질의
     D = D ∪ {(s_t, a_t^*)}
     D로 π_{i+1} 학습
```

전문가에게 매번 묻는 비용을 줄이려고 HG-DAgger, ThriftyDAgger 같은 human-in-the-loop 변형을 쓴다.

RL과 IL을 기준별로 비교하면 아래와 같다. 실제 표본 수와 안전성은 알고리즘, 시뮬레이터, 전문가 데이터의 품질에 따라 달라진다.

| 기준 | RL | IL |
|------|----|----|
| Sample efficiency | 환경 상호작용이 많이 필요할 수 있음 | 전문가 시연 수와 다양성에 좌우됨 |
| 보상 함수 | 직접 설계해야 함 (reward engineering) | 전문가 시연으로 대신함 |
| 안전성 | 탐색(exploration) 중 위험한 행동 가능 | 전문가 행동 모방이므로 상대적으로 안전 |
| Sim-to-Real | 보상 함수의 sim-real gap도 문제 | 실제 시연 데이터를 쓰면 gap 감소 |

로보틱스에서는 보상 함수를 설계하기가 어렵다. "컵을 잡아라"의 보상을 컵과 그리퍼 사이의 거리로 정하면 로봇이 컵 옆에 가서 멈추는 정책을 배울 수 있다. 잡았는지 여부로 정하면 보상이 드물게 주어지는 sparse reward 문제가 생긴다. IL은 보상 대신 시연을 학습 신호로 쓴다.

> **추천 자료**
> - [Ross et al., "A Reduction of Imitation Learning and Structured Prediction to No-Regret Online Learning" (2011)](https://arxiv.org/abs/1011.0686) — DAgger 원 논문
> - [Florence et al., "Implicit Behavioral Cloning" (CoRL 2021)](https://arxiv.org/abs/2109.00137) — energy-based 모델로 BC의 multi-modality 문제를 다룬 논문
> - Zare et al., "A Survey of Imitation Learning: Algorithms, Recent Developments, and Challenges" (IEEE Trans. Cybernetics, 2024) — IL 전반 서베이

## 12.8 심화: Diffusion Policy

Chi et al.(RSS 2023)의 Diffusion Policy는 행동 시퀀스(action trajectory)를 denoising diffusion 과정으로 생성하는 정책 표현이다. 학습 방식은 BC와 같고, 정책이 행동을 내는 방식만 바꿨다. 로봇 조작(manipulation) 연구에서 널리 쓴다.

기존 BC는 `π_θ(s) → a`로 action 하나를 결정론적으로 예측한다. 같은 상태에서 여러 행동이 모두 맞을 때가 있다(multi-modality). 테이블 위의 컵은 왼쪽으로 돌아가서 잡아도 되고 오른쪽으로 돌아가서 잡아도 된다. MSE로 학습한 BC는 두 행동의 평균, 곧 컵 정면으로 들이받는 행동을 낼 수 있다. Gaussian Mixture Model은 모드 수를 미리 정해야 한다.

Diffusion Policy는 모드 수를 미리 정하지 않고 denoising 과정으로 이 multi-modal 분포를 표현한다.

```
1. 랜덤 노이즈 a_T ~ N(0, I)에서 시작 (T = diffusion steps)
2. 현재 관측 s를 조건으로 반복적으로 denoising:
   a_{t-1} = denoise_θ(a_t, s, t)    for t = T, T-1, ..., 1
3. 최종 a_0가 실행할 action trajectory
```

Action trajectory는 미래 몇 스텝의 action 시퀀스 `[a(0), a(1), ..., a(H)]`이다. 괄호 안 숫자는 실행 시간 인덱스다. 앞 의사코드의 아래첨자 t는 diffusion step 번호라서 서로 다른 축이다. 이 중 처음 몇 스텝만 실행하고(receding horizon), 다시 새 관측으로 다음 trajectory를 생성한다.

Diffusion Policy는 action sequence를 한 번에 생성하므로 시간적으로 이어진 행동이 나온다. 학습에는 노이즈 예측(denoising) 손실을 쓴다.

Inference 때 denoising을 10~100 steps 반복하므로 100 Hz를 넘는 실시간 제어 루프에 그대로 넣기에는 느리다. DDIM 같은 가속 샘플러나 consistency distillation으로 step 수를 줄인다.

Chi et al.은 4개 벤치마크의 12개 태스크에서 기존 방법보다 평균 46.9% 높은 성공률을 보고했다. 이 수치는 논문이 고른 태스크·평가지표·baseline에서 잰 값이다.

> **추천 자료**
> - [Chi et al., "Diffusion Policy: Visuomotor Policy Learning via Action Diffusion" (RSS 2023)](https://arxiv.org/abs/2303.04137) — Diffusion Policy 원 논문
> - [Diffusion Policy 프로젝트 페이지](https://diffusion-policy.cs.columbia.edu/) — 코드, 데모, 영상
> - [Ho et al., "Denoising Diffusion Probabilistic Models" (NeurIPS 2020)](https://arxiv.org/abs/2006.11239) — Diffusion model 기초 논문

## 12.9 심화: Sim-to-Real Transfer

**1. Domain Randomization (DR)**

정책 학습에서는 Ch.8 §8.5의 물리 파라미터와 Ch.10 §10.11의 시각 요소를 함께, 에피소드마다 무작위로 바꾼다. 충분히 넓은 범위에서 학습하면 현실 환경도 그 범위 안에 들어간다고 가정한다.

랜덤화 대상:
- **시각적(Visual)**: 텍스처, 조명 방향/세기, 카메라 위치/시야각, 배경
- **물리적(Physical)**: 마찰 계수, 관성 모멘트, 링크 질량, 관절 감쇠(damping)
- **동역학(Dynamics)**: actuator 지연, 센서 노이즈, 제어 주기

DR의 범위 설정: 범위를 넓힐수록 학습이 어려워지고 정책이 보수적으로 바뀐다. 범위가 좁으면 현실 조건이 범위 밖으로 나간다. 실제 로봇에서 잰 값 주변으로 범위를 잡고 넓혀 가며 조정한다.

**2. System Identification (SysID)**

실제 시스템의 물리 파라미터를 측정하거나 추정해 시뮬레이터를 보정한다.

```
1. 실제 로봇에서 특정 trajectory를 실행하여 데이터 수집
2. 시뮬레이터의 파라미터 φ를 최적화:
   φ* = argmin_φ || f_sim(φ) - f_real ||^2
3. 보정된 시뮬레이터에서 정책 학습
```

SysID는 시뮬레이터가 모델링하는 파라미터만 맞춘다. 케이블의 유연함이나 접촉면의 미세 변형처럼 시뮬레이터 모델 밖에 있는 현상은 남는다.

**3. Real-to-Sim-to-Real (R2S2R)**

실제 데이터로 시뮬레이터를 교정하고, 그 시뮬레이터에서 학습한 정책을 다시 실제 로봇에 옮긴다. Domain Randomization을 함께 쓸 때는 어느 파라미터를 어느 범위에서 바꿨는지 단계마다 기록해 둔다.

```
1. 실제 데이터를 소량 수집
2. 실제 데이터로 시뮬레이터를 교정 (SysID) 또는 차이를 모델링
3. 교정된 시뮬레이터에서 정책 학습
4. 학습된 정책을 실제 로봇에 적용
5. (반복) 실제 결과로 시뮬레이터를 다시 교정
```

**4. Transfer 성공 여부 판단**

가장 기본적인 정량 기준은 성공률이다. 같은 태스크를 sim과 real에서 돌려 성공률을 비교한다.

- Sim 성공률 ≈ Real 성공률: 두 값이 모두 충분히 높고 실패 양상도 비슷하면 transfer가 된 것으로 본다. 둘 다 낮으면 두 값이 가까워도 판단 근거가 약하므로 절대 성능과 시행 수를 함께 본다.
- Sim >> Real: reality gap이 큼. DR 범위 확장 또는 SysID 보정 필요.
- Sim < Real: 시뮬레이터 조건을 현실보다 어렵게(보수적으로) 잡았을 가능성이 크다.

행동 궤적(trajectory)이 얼마나 비슷한지, 접촉력(contact force)이 얼마나 다른지도 함께 본다.

> **추천 자료**
> - [Tobin et al., "Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World" (2017)](https://arxiv.org/abs/1703.06907) — DR 원 논문
> - Muratore et al., "Robot Learning from Randomized Simulations: A Review" (Frontiers in Robotics and AI, 2022) — DR 체계적 정리
> - [Hanna & Stone, "Grounded Action Transformation for Robot Learning in Simulation" (AAAI 2017)](https://ojs.aaai.org/index.php/AAAI/article/view/11044) — transfer 방법론
> - [NVIDIA Isaac Lab Tutorials](https://isaac-sim.github.io/IsaacLab/) — 실습용 DR/SysID 파이프라인

> **추가 논문 (3D/Spatial 이해 + 벤치마크)**
> - [Hong et al., "3D-LLM: Injecting the 3D World into Large Language Models" (NeurIPS 2023, arXiv:2307.12981)](https://arxiv.org/abs/2307.12981) — 3D 장면을 LLM 입력으로 넣어 3D captioning, QA, navigation을 학습
> - [Chen et al., "SpatialVLM: Endowing Vision-Language Models with Spatial Reasoning" (CVPR 2024, arXiv:2401.12168)](https://arxiv.org/abs/2401.12168) — VLM이 거리·크기 같은 공간 관계를 추론하도록 학습
> - [Nasiriany et al., "RoboCasa: Large-Scale Simulation of Everyday Tasks for Generalist Robots" (RSS 2024, arXiv:2406.02523)](https://arxiv.org/abs/2406.02523) — 100개 주방 태스크, 150+ 물체 카테고리. 가정용 로봇 벤치마크
> - [Puig et al., "Habitat 3.0: A Co-Habitat for Humans, Avatars, and Robots" (ICLR 2024, arXiv:2310.13724)](https://arxiv.org/abs/2310.13724) — 사람-로봇 공존 시뮬레이션. Social navigation, 협업 태스크

> **기술 흐름: VLA & Embodied AI**
> - ~2015: 개별 태스크별 모방학습(imitation learning), 단일 물체 grasping 연구 중심
> - 2017~: Domain Randomization으로 Sim-to-Real 전이 본격화, MuJoCo/PyBullet 기반 연구
> - 2021~: 언어 지시로 로봇을 제어하는 연구. CLIPort(CoRL 2021)는 비전-언어 모델 CLIP을, SayCan(CoRL 2022)은 대규모 언어 모델(LLM)을 썼다
> - 2022~: RT-1, RT-2, PaLM-E 등 Foundation Model 기반 로봇 정책 등장. Open X-Embodiment 데이터셋 구축
> - 2024~: OpenVLA, Octo, π0 등 코드와 가중치를 공개한 VLA 모델 등장. World Model 기반 계획(planning), End-to-End 자율주행(UniAD·VAD 2023, GenAD 2024), modular·hybrid 설계가 함께 연구됨
