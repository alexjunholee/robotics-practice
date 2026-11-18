# 12장 VLA와 체화 AI


11장에서는 VFM으로 장면을 분할하고 깊이를 추정하며, 이미지와 자연어를 같은 의미 공간에 연결해 로봇이 주변 환경을 폭넓게 이해하는 방법을 살펴보았다. 하지만 보는 것만으로는 임무를 끝낼 수 없다. 이제 로봇에게 `"빨간 컵을 집어서 테이블 위에 놓아줘"` 같은 자연어 명령과 카메라 영상을 함께 건네고, 곧바로 물리적 행동까지 출력하게 만드는 **VLA(Vision-Language-Action)** 모델과 **체화 AI(Embodied AI)**로 시선을 옮겨 보자.

## 12.1 VLA 개념

전통적인 로봇 소프트웨어는 시각 인식, 언어 이해, 행동 생성을 서로 다른 독립 모듈로 쪼개어 설계했다. 카메라 영상에서 물체 좌표를 찾고, 언어 파서로 명령을 해석한 뒤, 별도의 계획기와 제어기가 움직임을 계산하는 식이다. **VLA(Vision-Language-Action)** 모델은 이 세 역할을 하나의 통합 신경망 안에서 함께 처리한다.

```
입력: 이미지 + 자연어 명령 ("pick up the red cup")
출력: 로봇 행동 (관절 각도, gripper 명령 등)
```

이처럼 물리적 몸체를 가진 에이전트가 현실 공간과 상호작용하며 지능을 발휘하고 학습하는 패러다임을 **체화 AI(Embodied AI)**라 부른다. 체화 AI는 인식과 행동을 떼어 놓고 보지 않으며, 시뮬레이션에서 배운 지식을 실제 하드웨어로 옮길 때 부딪히는 현실 간극(Sim-to-Real gap)까지 핵심 문제로 다룬다. 화면 속에서 고양이를 분류할 때는 픽셀만 보면 그만이지만, 컵을 집어 올리려고 로봇 팔이 물체를 건드리는 순간 중력과 마찰, 충돌 반력이 결과에 즉각 개입하므로 고려해야 할 물리 변수가 훨씬 많아진다.

> **추천 자료**
> - [Google DeepMind Robotics Blog](https://deepmind.google/blog/) — RT-1, RT-2, PaLM-E 등의 공식 블로그 포스트

## 12.2 주요 모델 및 연구

대규모 언어·비전 모델의 추론 능력이 로봇 제어와 결합하기 시작하면서 VLA 연구는 빠르게 발전했다. Google DeepMind의 RT-1·RT-2와 PaLM-E가 웹 지식의 전이 가능성을 열어젖혔다면, 가중치를 공개한 OpenVLA와 Octo는 일반 연구실에서도 범용 로봇 정책을 직접 다루고 파인튜닝할 수 있는 토대를 놓았다. 나아가 이러한 접근은 물체 조작뿐 아니라 주행과 장기 작업 계획(LINGO, SayCan)으로 확장되었다.

### 12.2.1 RT-1, RT-2 (Google DeepMind)

**RT-1(Robotics Transformer 1)**은 700개 이상의 태스크를 담은 약 13만 에피소드의 로봇 시연 데이터를 학습해, 영상과 지시를 받아 로봇 행동을 이산 토큰(action token) 형태로 직접 출력하는 트랜스포머 모델이다. 이어 나온 **RT-2(Robotics Transformer 2)**는 아예 웹 규모의 이미지-텍스트 데이터로 사전학습된 PaLI-X나 PaLM-E 같은 VLM(시각-언어 모델)을 기반으로 삼고, 여기에 로봇 시연 데이터를 함께 파인튜닝해 행동을 출력하도록 만들었다. 원 논문은 인터넷 웹 데이터에서 익힌 의미 지식이 실제 로봇 작업으로 얼마나 옮겨 가는지 검증했으며, 행동을 내놓기 전에 중간 단계 계획을 텍스트로 먼저 풀어쓰게 하는 *robot chain-of-thought* 실험도 함께 실었다.

그 결과 RT-2는 로봇 학습 데이터에 한 번도 등장하지 않은 물체나 추상적인 지시(예: `"멸종된 동물을 가리키는 물체를 집어"`처럼 상식 추론이 필요한 요청)도 웹 데이터에서 배운 지식을 끌어와 제로샷(zero-shot)으로 처리해 보였다.

> **추천 자료**
> - [Google DeepMind — RT-2 Demo Video](https://deepmind.google/blog/rt-2-new-model-translates-vision-and-language-into-action/) — RT-2의 실제 동작 영상과 설명
> - [Brohan et al., "RT-1: Robotics Transformer" (RSS 2023)](https://arxiv.org/abs/2212.06817) — RT-1 원 논문
> - [Brohan et al., "RT-2: Vision-Language-Action Models" (CoRL 2023)](https://arxiv.org/abs/2307.15818) — RT-2 원 논문

### 12.2.2 PaLM-E

**PaLM-E**는 거대 언어 모델 PaLM(540B)과 거대 비전 인코더 ViT-22B를 결합하고, 여기에 로봇의 현재 상태 벡터까지 입력으로 받아들이는 총 562B 파라미터 규모의 Embodied Multimodal Language Model이다. 다양한 로봇 태스크를 하나의 모델로 소화할 뿐 아니라, 저자들은 로봇 시연 데이터와 웹 이미지·텍스트 데이터를 함께 학습시켰을 때 로봇 데이터만으로 학습한 경우보다 오히려 로봇 태스크 성능이 높아지는 **"긍정적 전이(positive transfer)"** 현상을 보고했다.

> **추천 자료**
> - [Driess et al., "PaLM-E: An Embodied Multimodal Language Model" (2023)](https://arxiv.org/abs/2303.03378) — PaLM-E 원 논문

### 12.2.3 OpenVLA

RT-2와 PaLM-E는 거대 모델의 가능성을 입증했지만 모델 가중치를 공개하지 않아 외부 연구자가 직접 써 보기는 어려웠다. **OpenVLA**는 학습 코드와 모델 가중치를 모두 공개해, 일정 수준의 GPU 자원을 갖춘 연구실이라면 누구나 내려받아 자기 로봇 환경에 맞게 파인튜닝하거나 배포할 수 있도록 열어 주었다. OpenVLA는 Llama 2 기반의 7B 파라미터 모델로, 대규모 다종 로봇 데이터셋인 Open X-Embodiment의 970K 에피소드를 학습했다.

아래 코드는 HuggingFace 인터페이스로 OpenVLA 모델을 불러와 현재 카메라 이미지와 자연어 지시로부터 7자유도(7-DoF) 로봇 행동을 추론하는 예시다. 마지막 줄의 `unnorm_key`는 모델이 출력한 정규화 행동을 특정 데이터셋의 실제 물리 스케일로 되돌릴 때 쓰인다.

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

OpenVLA의 밑바탕이 된 **RT-X** 프로젝트는 전 세계 21개 기관이 22종의 서로 다른 로봇으로 모은 데이터를 **Open X-Embodiment**라는 단일 데이터셋으로 통합하고, 이 위에서 여러 로봇을 아우르는 범용 정책을 학습한 공동 노력이다. 한편 같은 Open X-Embodiment의 800K 궤적으로 학습한 또 다른 오픈소스 정책 **Octo**는 가장 큰 버전도 93M 파라미터로 OpenVLA보다 훨씬 가벼우며, 새로운 로봇의 센서 구성이나 행동 공간에 맞춰 빠르게 파인튜닝할 수 있도록 설계되었다.

> **추천 자료**
> - [OpenVLA GitHub](https://github.com/openvla/openvla) — 코드와 모델 가중치 공개
> - [Kim et al., "OpenVLA" (2024)](https://arxiv.org/abs/2406.09246) — OpenVLA 논문
> - [Open X-Embodiment Collaboration, "Open X-Embodiment" (ICRA 2024)](https://arxiv.org/abs/2310.08864) — RT-X 데이터셋 논문
> - [Octo GitHub](https://github.com/octo-models/octo) — 경량 오픈소스 로봇 정책 모델

### 12.2.4 Navigation 관련

체화 AI의 무대는 테이블 위 물체 조작(manipulation)에만 머물지 않는다. 공간을 가로질러 이동하는 주행(navigation)과 여러 단계가 얽힌 장기 작업 계획에도 언어 이해가 접목된다.

자율주행 분야에서는 Wayve가 공개한 **LINGO** 모델 계열이 대표적이다. **LINGO-1**은 운전 장면을 보고 차량의 판단과 주변 상황을 자연어로 해설해 주는 개방 루프(open-loop) 모델이고, 한 단계 더 나아간 **LINGO-2**는 언어 지시와 시각 입력을 받아 차량의 조향과 가속을 직접 제어하는 폐쇄 루프(closed-loop) 모델이다.

장기 가정 작업 계획에서는 **SayCan**이 고전적이면서도 명쾌한 해법을 보여 주었다. `"커피 만들어줘"`라는 추상적 요청을 받으면, LLM은 `"1. 컵을 잡아 2. 커피 머신으로 가 3. 버튼을 눌러..."`처럼 논리적인 순서를 잘 세운다. 하지만 LLM은 지금 로봇이 컵에서 3 m 떨어져 있는지, 바로 앞에 서 있는지 물리적 맥락을 알지 못해 실행 불가능한 단계를 내놓기 쉽다. SayCan은 이를 해결하기 위해 후보 행동마다 두 가지 점수를 곱해 다음 행동을 고른다.

- **Affordance function**: 지금 로봇의 물리적 상태에서 해당 행동을 성공적으로 수행할 확률
- **LLM 점수**: 그 행동이 최종 목표를 이루는 데 얼마나 기여하는지 평가한 언어적 타당성

두 점수를 곱하면 "목표 달성에도 필요하고, 지금 당장 로봇이 실제로 해낼 수도 있는" 행동만이 높은 점수를 받아 선택된다.

> **추천 자료**
> - [Ahn et al., "Do As I Can, Not As I Say: Grounding Language in Robotic Affordances" (CoRL 2022)](https://arxiv.org/abs/2204.01691) — SayCan 논문
> - [SayCan project page](https://say-can.github.io/) — 데모 영상 포함

## 12.3 World Models

앞선 VLA 모델들이 관측에서 곧바로 행동을 내놓는다면, 한 걸음 물러서서 `"내가 이 행동을 취하면 다음 순간 세상이 어떻게 변할까?"`를 내면에서 먼저 시뮬레이션해 보는 접근도 있다. **World Model**은 현재 상태와 행동을 입력받아 다음 상태나 미래의 관측 장면을 예측하는 모델이다. 후보 행동을 실제 로봇에서 시험하기 전에 World Model 안에서 미리 굴려 보면(rollout), 고가의 실제 장비를 움직이거나 파손시킬 위험을 크게 줄일 수 있다. 그래서 모델 기반 강화학습(model-based RL)에서 가상 궤적을 생성하거나, 위험한 행동을 실행 직전에 걸러 내는 안전 검증 도구로 널리 쓰인다.

자율주행 분야에서도 World Model 연구가 활발하다. Wayve의 **GAIA-1**은 차량의 조향·속도 행동을 조건으로 받아 앞으로 펼쳐질 주행 영상을 생성한다. **DriveDreamer**는 HD 지도와 3D 바운딩 박스 같은 도로 구조 정보에 차량 행동을 결합해 현실적인 주행 영상을 만들어 내며, 텍스트 프롬프트로 날씨나 시간대까지 바꾼다. **MILE**은 고차원 픽셀 전체가 아니라 압축된 잠재 공간(latent space) 위의 World Model 안에서 미래 상태 예측과 주행 정책 학습을 동시에 수행한다.

수학적 구조로 보면 World Model은 제어공학에서 다루는 상태 공간 모델(state-space model)의 비선형·고차원 확장판과 같다. 선형 상태 공간 모델 `x_{t+1} = Ax_t + Bu_t`에서 행렬 A와 B 자리에 심층 신경망을 넣고, 원시 센서 대신 학습된 잠재 상태(latent state)를 다룬다고 생각하면 이해하기 쉽다.

```
z_{t+1} = f(z_t, a_t)     # Dynamics model (현재 상태 + 행동 → 다음 상태)
o_t = g(z_t)              # Observation model (잠재 상태 → 관측)
r_t = h(z_t, a_t)         # Reward model (보상 예측)
```

여기서 동역학 모델 `f`가 시간 흐름에 따른 상태 전이를 예측하고, 관측 모델 `g`와 보상 모델 `h`가 각각 그 상태에서 보일 센서 영상과 얻게 될 보상을 복원한다. 이 세 요소가 갖춰지면 로봇은 실제 환경을 건드리지 않고도 잠재 공간 속에서 수천 번의 시뮬레이션을 돌려 최선의 행동을 고를 수 있다.

> **추천 자료**
> - [Hu et al., "GAIA-1: A Generative World Model for Autonomous Driving" (2023)](https://arxiv.org/abs/2309.17080) — Wayve의 World Model 논문
> - [Wang et al., "DriveDreamer" (2023)](https://arxiv.org/abs/2309.09777) — 주행 시나리오 생성 논문
> - [Yannic Kilcher — World Models Explained](https://www.youtube.com/watch?v=dPsXxLyqpfs) — World Model 개념 설명 영상

## 12.4 End-to-End vs Modular

로봇이나 자율주행 시스템의 전체 아키텍처를 짤 때 가장 먼저 마주치는 설계 갈림길은 인식·예측·계획·제어를 하나의 거대한 신경망으로 엮을 것인가(**End-to-End**), 아니면 역할별 블록으로 나눌 것인가(**Modular**)다.

**End-to-End** 방식은 센서 원시 입력을 단일 신경망에 통과시켜 곧바로 조향이나 관절 토크 같은 행동 출력을 얻는다.

```
센서 입력 → [단일 신경망] → 행동 출력
```

사람이 인위적으로 정한 중간 표현(예: 바운딩 박스나 점유 격자)을 거치면서 생기는 정보 손실을 피할 수 있고 파이프라인 자체도 간결하다. 대신 내부 판단 과정을 사람이 뜯어보기 어려워 해석 가능성(interpretability)이 떨어지고, 만족스러운 수준에 도달하려면 막대한 학습 데이터가 필요하다. 초기 자율주행 연구인 NVIDIA PilotNet이나, 도심 주행 스택을 단일 end-to-end 신경망으로 전환했다고 릴리스 노트에서 밝힌 Tesla FSD v12 이후 버전이 이 갈래를 대표한다.

다만 자율주행의 End-to-End 모델도 내부를 들여다보면 다양한 형태를 띤다. **UniAD**(CVPR 2023 Best Paper)는 detection, tracking, mapping, prediction, planning 모듈을 하나의 네트워크 안에 통합해 최종 주행 계획(planning)을 목표로 함께 학습(joint training)시키되, 중간 모듈의 출력도 밖에서 들여다볼 수 있어 해석이 한결 수월하다. **VAD**(2023)는 장면을 빽빽한 래스터 지도 대신 벡터화된 기하 표현으로 압축해 planning 연산량을 크게 줄였고, **GenAD**(2024)는 주변 차량과 자차의 미래 궤적을 생성 모델로 함께 예측한다.

반대로 **Modular** 방식은 전통적인 공학 설계처럼 시스템 책임을 뚜렷하게 나눈다.

```
센서 → [인식] → [예측] → [계획] → [제어] → 행동
```

각 모듈을 독립적으로 개발하고 단위 테스트하며 디버깅하기 좋고 동작 원리도 투명하다. 반면 모듈과 모듈 사이를 정해진 인터페이스(예: 객체 좌표 목록)로만 연결하므로 미세한 시각 정보가 중간에 버려질 수 있고, 전체 시스템 성능을 한꺼번에 끝단까지 최적화하기는 어렵다. 오픈소스 자율주행 스택인 Apollo와 Autoware가 전형적인 모듈형 구조를 따른다.

실제 현장에서는 양 극단보다 **하이브리드(Hybrid)** 구성을 택하는 사례가 많다. 인식 단계에는 강력한 딥러닝·VFM 모델을 쓰되 계획과 제어는 검증된 모델 기반 최적화에 맡기거나, 앞서 본 UniAD처럼 end-to-end로 미분 가능하게 학습되는 신경망 내부에 명시적 모듈 경계를 두어 두 장점을 절충한다.

> **추천 자료**
> - [Hu et al., "Planning-oriented Autonomous Driving (UniAD)" (2023)](https://arxiv.org/abs/2212.10156) — CVPR 2023 Best Paper
> - [Jiang et al., "VAD" (2023)](https://arxiv.org/abs/2303.12077) — 벡터화 기반 자율주행
> - [Tesla AI Day 2022 Presentation](https://www.youtube.com/watch?v=ODSJsviD_SU) — FSD v12 이전의 인식(occupancy network)·계획 모듈 구조를 설명한 발표. 위 Modular·하이브리드 구성과 비교해 볼 수 있다

### End-to-End vs Modular: 2026년 현실

학계의 눈부신 end-to-end 데모에도 불구하고, 실제 로봇이나 상용 시스템을 구축할 때 모듈형이나 하이브리드 구조를 고집하는 데에는 분명한 공학적 이유가 있다. 어떤 구조를 택할지는 결국 응용 분야와 안전 검증 조건이 결정한다.

- **디버깅**: 로봇이 테이블 위 컵을 놓쳤을 때 모듈형 시스템에서는 중간 출력을 순서대로 열어 `"depth estimation이 5 cm 어긋났다"`거나 `"grasp planning 자세가 틀렸다"`처럼 원인을 즉시 좁혀 낸다. 반면 end-to-end 모델에서는 입력 영상과 최종 관절 값만 보이므로 실패 원인을 추적하는 데 훨씬 많은 시간이 든다.
- **안전 보장**: 모듈형 아키텍처는 각 단계 사이에 속도 제한이나 충돌 감지 같은 명시적 safety check를 끼워 넣기 쉽다. End-to-end 모델을 쓸 때는 신경망 출력 뒷단에 별도의 안전 감시 계층을 반드시 덧붙여야 한다.
- **부분 업데이트**: 인식 성능만 개선하고 싶다면 모듈형에서는 perception 모듈만 갈아 끼우면 된다. End-to-end 구조에서는 변경이 전체 동작에 파급되므로 수정 범위에 따라 네트워크 전체를 다시 학습하고 처음부터 검증해야 한다.
- **데이터 효율**: RT-1이 약 13만 에피소드, OpenVLA가 97만 에피소드를 학습에 투입했다는 사실이 말해 주듯, 순수 end-to-end 학습은 데이터 갈증이 크다. 개별 연구실이 이 규모의 실제 로봇 시연을 직접 모으려면 막대한 장비와 인력 시간이 필요하다.

연구실의 Local/Global Module 설계(부록 F) 역시 이러한 현실적 고려를 반영한 하이브리드 구성이다. 의미 인식에는 VFM을 활용하되, 실시간 계획과 제어는 명시적 모듈과 안전 검사 위에 올려 안정성을 확보한다. End-to-end 모델이 실제 현장에 더 널리 자리 잡으려면, 적은 데이터로도 잘 학습할 뿐 아니라 응용 현장이 요구하는 검증 기준 아래에서 디버깅 가능성과 안전 보장을 입증해 보여야 한다.

## 12.5 Spatial AI + VLA 통합

앞 절에서 확인한 하이브리드 설계 철학을 시간척도(timescale) 관점에서 구체화한 구조가 바로 **2-Module Architecture**다. VLA 모델은 `"부엌에서 커피 가져와"` 같은 장기 작업을 하위 목표들로 쪼개어 지시하는 데 강하지만 추론 속도가 느리다. 반대로 로봇이 사람이나 문틀에 부딪히지 않고 균형을 잡으려면 밀리초 단위의 빠른 반응이 필요하다. 실제 시스템에서는 서로 다른 주파수로 도는 두 모듈을 계층적으로 연결해 이 간극을 메운다.

연구실의 2-Module Architecture와 연결해 보면 각 계층의 역할은 다음과 같다.

- **Local(Fast) Perception**: 깊이, 장애물 거리, 로봇 자세 같은 기하 정보를 **10–100 Hz**의 빠른 주기로 처리한다. 지연을 최소화하기 위해 고전 기하 알고리즘이나 가벼운 온보드 학습 모델을 사용한다.
- **Global(Heavy) Understanding**: 물체의 종류, 공간 맥락, 작업 순서 같은 고수준 의미 정보를 VFM과 VLA로 처리한다. 연산량이 크므로 서버나 클라우드 환경에서 **1–10 Hz** 주기로 실행된다.

두 계층이 실제 주행·조작 시나리오에서 어떻게 호흡을 맞추는지 흐름으로 따라가 보자.

```
1. Local: 실시간 obstacle avoidance, odometry
2. Global: "kitchen에서 cup을 찾아서 table로 가져와"
   - VLM으로 cup 인식
   - Semantic map에서 경로 계획
3. Local이 Global의 waypoint를 받아 실제 이동 수행
```

이처럼 Global 모듈이 무거운 추론으로 전체 전략과 경유지(waypoint)를 잡아 주면, Local 모듈이 그 지시를 받아 실시간 장애물 회피와 안전한 추종을 책임진다.

## 12.6 Sim-to-Real & Simulation Platforms

VLA든 모듈형 정책이든 학습하려면 엄청난 양의 경험 데이터가 필요하다. 하지만 실제 로봇을 수만 번씩 굴려 데이터를 모으려면 시간과 비용이 막대할뿐더러, 넘어지거나 부딪혀 하드웨어가 부서질 위험도 크다. 그래서 연구 현장에서는 가상 시뮬레이터에서 먼저 정책을 충분히 학습시킨 뒤 실제 로봇으로 옮기는 **Sim-to-Real** 패러다임을 표준처럼 활용한다. 이때 시뮬레이션과 현실 사이의 차이(Reality Gap)를 좁히는 대표 기법이 시뮬레이션의 텍스처, 조명, 물리 파라미터를 에피소드마다 무작위로 흔들어 학습하는 **Domain Randomization**이며, 구체적인 대상과 범위 설정법은 §12.9에서 자세히 다룬다(→ §8.5).

목적과 연구 성격에 따라 주로 선택하는 시뮬레이션 플랫폼도 뚜렷하게 나뉜다.

- **NVIDIA Isaac Sim/Lab**: GPU 가속 물리 시뮬레이션을 바탕으로 수천 개의 가상 환경을 병렬로 동시에 돌려 대규모 강화학습을 빠르게 수행한다. 특히 **Isaac Lab**은 이러한 대규모 로봇 학습 연구를 위해 정돈된 통합 프레임워크다.
- **AI2-THOR** (Allen Institute): 주방이나 거실처럼 정교하게 꾸며진 가정 실내 환경에서 물체를 열고 집고 옮기는 조작(manipulation) 과제를 실험하기에 좋다.
- **Habitat** (Meta): Matterport3D나 Gibson 같은 대규모 실제 3D 스캔 실내 데이터 위에서 고속 내비게이션을 학습하고, Habitat Challenge에서 성능을 표준화된 기준으로 비교할 수 있다.
- **MuJoCo**: 다관절 로봇의 접촉(contact) 역학을 빠르고 수치적으로 안정되게 계산해 로봇 팔 조작과 사족·이족 보행 제어 연구에서 오랫동안 표준으로 쓰여 왔다. 2021년 DeepMind가 인수한 뒤 2022년 오픈소스로 공개되었다.

> **추천 자료**
> - [NVIDIA Isaac Lab Documentation](https://isaac-sim.github.io/IsaacLab/) — 로봇 학습을 위한 시뮬레이션 프레임워크
> - [AI2-THOR Documentation](https://ai2thor.allenai.org/) — 실내 환경 시뮬레이터
> - [Habitat Documentation](https://aihabitat.org/) — Meta의 Embodied AI 플랫폼
> - [Tobin et al., "Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World" (2017)](https://arxiv.org/abs/1703.06907) — Domain Randomization 원 논문

## 12.7 Imitation Learning [심화]

시뮬레이터나 실제 환경에서 VLA와 Embodied AI 정책(policy)을 학습하는 주된 방법은 크게 **강화학습(RL)**과 **모방학습(IL)** 두 가지다. 데이터를 모으는 방식부터 학습 신호를 설계하는 법, 실제 로봇에서 탐색할 때 감수해야 하는 위험까지 두 접근은 성격이 사뭇 다르다.

가장 기본적인 모방학습인 행동 복제(Behavioral Cloning, BC)의 정의와 분포 이동으로 오차가 눈덩이처럼 불어나는 compounding error, 그리고 이를 교정하는 DAgger 알고리즘의 절차는 8장 §8.6에서 이미 살펴보았다. 실제 연구에서는 매 스텝 전문가에게 정답 행동을 묻는 부담을 덜기 위해, 로봇이 위험하거나 불확실한 순간에만 사람이 개입해 교정해 주는 **HG-DAgger**나 **ThriftyDAgger** 같은 human-in-the-loop 변형 기법이 자주 쓰인다.

두 패러다임의 실무적 장단점을 네 가지 기준으로 나란히 비교하면 다음과 같다. 물론 실제 필요한 표본 수와 안전성은 세부 알고리즘과 시뮬레이터 정밀도, 전문가 시연의 품질에 따라 달라진다.

| 기준 | RL | IL |
|------|----|----|
| Sample efficiency | 환경 상호작용이 많이 필요할 수 있음 | 전문가 시연 수와 다양성에 좌우됨 |
| 보상 함수 | 직접 설계해야 함 (reward engineering) | 전문가 시연으로 대신함 |
| 안전성 | 탐색(exploration) 중 위험한 행동 가능 | 전문가 행동 모방이므로 상대적으로 안전 |
| Sim-to-Real | 보상 함수의 sim-real gap도 문제 | 실제 시연 데이터를 쓰면 gap 감소 |

로보틱스에서 유독 모방학습이 각광받는 큰 이유는 **보상 함수(reward function) 설계의 어려움** 때문이다. 예컨대 `"컵을 잡아라"`라는 작업에서 보상을 단순히 컵과 그리퍼 사이의 거리로 주면, 로봇은 컵 바로 옆까지만 다가가서 멈춰 버리는 꼼수 정책을 배울 수 있다. 반대로 컵을 완전히 들어 올렸을 때만 보상을 주면, 우연히 성공하기 전까지 아무런 학습 신호도 받지 못하는 희소 보상(sparse reward) 문제에 빠진다. 모방학습은 이 까다로운 보상 설계 대신 사람의 시연 궤적 자체를 직접적인 학습 신호로 삼아 문제를 우회한다.

> **추천 자료**
> - [Ross et al., "A Reduction of Imitation Learning and Structured Prediction to No-Regret Online Learning" (2011)](https://arxiv.org/abs/1011.0686) — DAgger 원 논문
> - [Florence et al., "Implicit Behavioral Cloning" (CoRL 2021)](https://arxiv.org/abs/2109.00137) — energy-based 모델로 BC의 multi-modality 문제를 다룬 논문
> - Zare et al., "A Survey of Imitation Learning: Algorithms, Recent Developments, and Challenges" (IEEE Trans. Cybernetics, 2024) — IL 전반 서베이

## 12.8 Diffusion Policy [심화]

모방학습이 보상 설계의 수고를 덜어 주기는 하지만, 단순한 지도학습 방식의 행동 복제(BC)에는 치명적인 함정이 하나 있다. Chi et al.(RSS 2023)이 제안한 **Diffusion Policy**는 바로 이 지점을 파고들어 로봇 조작(manipulation) 연구의 표준 도구로 자리 잡았다. 학습 데이터로 전문가 시연을 쓰는 점은 BC와 같지만, 정책 신경망이 행동을 표현하고 생성하는 방식을 **denoising diffusion** 과정으로 바꾼 것이 핵심이다.

무엇이 문제였을까? 기존 BC는 `π_θ(s) → a`처럼 주어진 상태 `s`에서 하나의 행동 `a`를 결정론적으로 예측한다. 그런데 실제 로봇 작업에서는 똑같은 상황에서도 정답 행동이 여러 갈래로 나뉘는 **다봉성(multi-modality)**이 흔하다. 테이블 중앙에 놓인 컵을 피해서 이동할 때 왼쪽으로 돌아가도 정답이고 오른쪽으로 돌아가도 정답인 경우가 그렇다. 만약 정책을 평균 제곱 오차(MSE) 손실로 학습하면, 모델은 왼쪽 경로와 오른쪽 경로의 산술 평균을 출력해 컵 정면으로 그대로 들이받는 엉뚱한 행동을 내놓고 만다. Gaussian Mixture Model(GMM)을 써서 여러 모드를 표현할 수도 있지만, 상황에 따라 모드가 몇 개나 될지 미리 정해 주어야 하는 번거로움이 남는다.

Diffusion Policy는 모드 개수를 미리 못 박지 않고도, 무작위 노이즈에서 출발해 점진적으로 노이즈를 걷어 내는 denoising 과정으로 복잡한 다봉 분포(multi-modal distribution) 자체를 자연스럽게 그려 낸다.

```
1. 랜덤 노이즈 a_T ~ N(0, I)에서 시작 (T = diffusion steps)
2. 현재 관측 s를 조건으로 반복적으로 denoising:
   a_{t-1} = denoise_θ(a_t, s, t)    for t = T, T-1, ..., 1
3. 최종 a_0가 실행할 action trajectory
```

이 과정에서 모델이 한 번에 생성하는 `a_0`는 단일 시점의 행동이 아니라 앞으로 이어질 몇 스텝 분량의 행동 궤적(action trajectory) `[a(0), a(1), ..., a(H)]`이다. 여기서 괄호 안의 숫자는 실제 로봇이 움직이는 실행 시간 인덱스를 뜻하며, 앞선 의사코드의 아래첨자 `t`는 노이즈를 제거하는 diffusion step 번호이므로 서로 다른 축을 가리킨다. 실제 로봇 제어에서는 생성된 궤적 중 앞쪽 몇 스텝만 실제로 실행하고(receding horizon), 바뀐 최신 관측을 다시 입력받아 다음 궤적을 새로 생성하는 식으로 반응성을 유지한다.

이처럼 행동 시퀀스 전체를 한꺼번에 뽑아내면 스텝마다 끊김 없이 시간적으로 부드럽게 이어진 동작이 나오며, 학습 역시 이미지 생성 확산 모델과 마찬가지로 노이즈 예측(denoising) 손실을 사용해 안정적이다. 다만 추론할 때마다 10~100번의 denoising step을 반복해야 하므로 100 Hz를 훌쩍 넘는 고속 실시간 제어 루프에 그대로 넣기에는 다소 무겁다. 이를 보완하기 위해 추론 시에는 DDIM 같은 가속 샘플러나 consistency distillation 기법으로 반복 step 수를 대폭 줄여 사용한다.

Chi et al.은 4개 벤치마크에 걸친 12개 조작 태스크에서 기존 비교 방법들보다 평균 46.9% 높은 성공률을 달성했다고 보고했다(물론 이 수치는 논문이 선정한 태스크와 평가지표, 기준선 조건에서 측정된 값이다).

> **추천 자료**
> - [Chi et al., "Diffusion Policy: Visuomotor Policy Learning via Action Diffusion" (RSS 2023)](https://arxiv.org/abs/2303.04137) — Diffusion Policy 원 논문
> - [Diffusion Policy 프로젝트 페이지](https://diffusion-policy.cs.columbia.edu/) — 코드, 데모, 영상
> - [Ho et al., "Denoising Diffusion Probabilistic Models" (NeurIPS 2020)](https://arxiv.org/abs/2006.11239) — Diffusion model 기초 논문

## 12.9 Sim-to-Real Transfer [심화]

Diffusion Policy나 강화학습으로 가상 환경에서 매끄럽게 동작하는 정책을 얻었더라도, 그 정책을 실제 로봇 하드웨어에 올리는 순간 예상치 못한 오차와 떨림이 나타난다. 이 **Sim-to-Real Gap**을 체계적으로 건너기 위해 현장에서 활용하는 네 가지 핵심 전략을 정리해 보자.

**1. Domain Randomization (DR)**

시뮬레이터가 완벽히 현실을 흉내 내지 못한다면, 차라리 에피소드마다 시뮬레이션 환경을 다채롭게 흔들어 현실 환경도 그 수많은 변형 중 하나처럼 느껴지게 만드는 전략이다. 정책 학습에서는 8장 §8.5에서 다룬 물리 파라미터와 10장 §10.11에서 본 시각적 요소를 함께 무작위로 변화시킨다.

- **시각적(Visual)**: 텍스처, 조명 방향과 세기, 카메라 위치와 시야각, 배경 배치
- **물리적(Physical)**: 마찰 계수, 관성 모멘트, 링크 질량, 관절 감쇠(damping)
- **동역학(Dynamics)**: 구동기(actuator) 지연 시간, 센서 노이즈, 제어 루프 주기

무작위화 범위를 잡을 때는 균형이 중요하다. 범위를 지나치게 넓히면 정책이 최악의 조건까지 모두 버티려다 지나치게 보수적으로 변하거나 학습 자체가 어려워지고, 반대로 좁게 잡으면 실제 환경의 물리 특성이 범위 밖으로 빠져나가 전이에 실패한다. 실무에서는 실제 로봇에서 측정한 공칭 값 주변으로 좁게 시작한 뒤 점차 범위를 넓혀 가며 최적점을 조율한다.

**2. System Identification (SysID)**

환경을 무작위로 흔드는 대신 시뮬레이터 자체를 실제 로봇과 똑같이 닮게 정밀 보정하는 접근이다. 실제 로봇에서 정해진 궤적을 움직여 데이터를 모은 뒤, 시뮬레이터의 물리 반응이 실제 계측과 가장 일치하도록 파라미터 `φ`를 최적화한다.

```
1. 실제 로봇에서 특정 trajectory를 실행하여 데이터 수집
2. 시뮬레이터의 파라미터 φ를 최적화:
   φ* = argmin_φ || f_sim(φ) - f_real ||^2
3. 보정된 시뮬레이터에서 정책 학습
```

다만 SysID는 시뮬레이터 물리 엔진이 수식으로 모델링하고 있는 파라미터만 맞출 수 있다. 전선이나 케이블의 휘어짐, 접촉면의 미세한 탄성 변형처럼 시뮬레이터 모델 바깥에 있는 비모델링 현상은 여전히 간극으로 남는다.

**3. Real-to-Sim-to-Real (R2S2R)**

SysID와 정책 학습을 한 번의 일방향 과정으로 끝내지 않고 선순환 고리로 엮는 방법이다. 먼저 소량의 실제 데이터를 모아 시뮬레이터를 교정하거나 현실과의 차이를 학습한 뒤, 다듬어진 시뮬레이터에서 정책을 학습시켜 실제 로봇에 투입하고, 거기서 얻은 실제 실패 사례와 주행 기록으로 다시 시뮬레이터를 다듬는 과정을 반복한다. 이때 Domain Randomization을 함께 적용한다면 실험의 재현성을 잃지 않도록 단계마다 어떤 파라미터를 어느 범위로 흔들었는지 꼼꼼히 기록해 두어야 한다.

```
1. 실제 데이터를 소량 수집
2. 실제 데이터로 시뮬레이터를 교정 (SysID) 또는 차이를 모델링
3. 교정된 시뮬레이터에서 정책 학습
4. 학습된 정책을 실제 로봇에 적용
5. (반복) 실제 결과로 시뮬레이터를 다시 교정
```

**4. Transfer 성공 여부 판단**

실제 로봇에 정책을 올린 뒤 전이가 제대로 되었는지 판단하는 가장 기본적인 정량 지표는 **성공률**이다. 동일한 태스크를 시뮬레이션과 실제 환경에서 각각 충분히 반복 실행한 뒤 두 성공률을 비교한다.

- **Sim 성공률 ≈ Real 성공률**: 두 성공률이 모두 충분히 높은 수준이면서 실패하는 양상까지 닮았다면 전이가 성공적으로 이루어진 것으로 본다. 단, 둘 다 20%처럼 낮다면 숫자가 비슷하더라도 의미가 약하므로 절대 성능과 충분한 시행 횟수를 함께 확인해야 한다.
- **Sim >> Real**: 아직 reality gap이 크게 남아 있다는 신호다. DR 범위를 넓히거나 SysID로 물리 파라미터를 다시 맞춰야 한다.
- **Sim < Real**: 시뮬레이터의 랜덤화나 외란 조건을 실제 환경보다 더 혹독하고 보수적으로 설정했을 가능성이 크다.

정밀한 평가에서는 최종 성공 여부뿐 아니라 관절의 움직임 궤적(trajectory)이 시뮬레이션과 얼마나 닮았는지, 물체를 짚을 때 발생하는 접촉력(contact force)의 분포가 얼마나 어긋나는지도 함께 점검한다.

지금까지 VLA와 모방학습, 그리고 Sim-to-Real 전이를 거쳐 로봇이 시각·언어 지시를 물리적 행동으로 옮기는 전체 흐름을 살펴보았다. 그런데 로봇이 컵을 비켜 가며 정확히 손잡이를 잡거나 넓은 건물을 헤매지 않고 이동하려면, 2차원 화면 속 픽셀 좌표를 넘어 눈앞 세계의 정확한 3차원 위치와 표면 형상, 입체적 공간 구조를 읽어야 한다. 다음 13장에서는 포인트 클라우드 기초부터 3D Gaussian Splatting, 3D Scene Graph까지 로봇의 공간 지능을 떠받치는 3D 비전 기술을 본격적으로 다룬다.

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
