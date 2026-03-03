# Day 7: Navigation2와 자율주행 시스템 아키텍처

**날짜**: 2026-02-11
**학습 주제**: Navigation2 (Nav2) - 자율주행 시스템의 설계와 구현
**소요 시간**: 약 4시간 (트러블슈팅 포함)

---

## 📋 학습 목표 및 성과

### 목표
- [x] Navigation2 스택의 전체 아키텍처 이해
- [x] 저장된 맵을 사용한 자율주행 구현
- [x] AMCL(Localization)과 Path Planning의 협업 방식 이해
- [x] Global Planner와 Local Planner의 역할 분리 설계 이해
- [x] Costmap의 계층적 설계 분석
- [x] 2D Pose Estimate와 Nav2 Goal의 차이 이해
- [x] TF(Transform) Tree의 좌표계 계층 구조 이해
- [x] Lifecycle 노드 시스템의 상태 관리 방식 학습
- [x] 실제 시스템 트러블슈팅 경험

### 성과
- ✅ TurtleBot3가 저장된 맵에서 자율주행 성공
- ✅ Navigation2의 복잡한 분산 아키텍처 이해
- ✅ 3가지 주요 문제(RViz 중복, Lifecycle 충돌, TF 에러) 해결
- ✅ 실무 수준의 시스템 디버깅 경험 획득

---

## 🎯 Navigation2: 자율주행 시스템의 아키텍처

### Navigation2란 무엇인가

Navigation2는 단순히 "로봇을 A에서 B로 이동시키는 라이브러리"가 아닙니다. 이것은 **불확실성 속에서 안전하게 목표에 도달하기 위한 분산 의사결정 시스템**입니다. 로봇이 자율주행할 때 마주치는 근본적인 문제들을 생각해보세요:

1. **"나는 지금 어디에 있는가?"** (Localization 문제)
2. **"어떤 경로로 가야 하는가?"** (Path Planning 문제)
3. **"갑자기 장애물이 나타나면?"** (Dynamic Obstacle Avoidance 문제)
4. **"막다른 길이면?"** (Recovery 문제)
5. **"언제 멈춰야 하는가?"** (Goal Tolerance 문제)

이 모든 문제를 하나의 거대한 프로그램으로 해결하려 한다면, 코드는 곧 스파게티가 되고 유지보수는 악몽이 될 것입니다. Navigation2는 이 문제를 **"관심사의 분리(Separation of Concerns)"** 원칙에 따라 여러 독립적인 노드로 분해했습니다. 각 노드는 자신의 전문 분야에만 집중하고, 다른 노드와는 표준화된 인터페이스(Topics, Services, Actions)로 소통합니다.

이것이 바로 **마이크로서비스 아키텍처**의 로봇 버전입니다. 웹 백엔드에서 인증 서비스, 결제 서비스, 알림 서비스를 분리하듯이, Navigation2는 Localization 서비스, Planning 서비스, Control 서비스를 분리합니다. 이렇게 하면 한 서비스가 실패해도 전체 시스템이 무너지지 않고, 각 서비스를 독립적으로 업그레이드할 수 있습니다.

---

### Navigation2 시스템 아키텍처: 전체 그림

Navigation2는 크게 **4개의 핵심 레이어**로 구성됩니다:

```
┌─────────────────────────────────────────────────────────────┐
│  Application Layer: Behavior Tree Navigator                 │
│  (고수준 의사결정: "계획 → 실행 → 실패 시 복구")              │
└─────────────────────────────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────┐
│  Planning Layer: Planner Server                              │
│  (경로 계획: "출발지 → 목적지 최적 경로")                      │
└─────────────────────────────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────┐
│  Control Layer: Controller Server                            │
│  (모션 제어: "경로 추종 + 장애물 회피")                        │
└─────────────────────────────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────┐
│  Perception Layer: Costmap + AMCL                            │
│  (환경 인식: "나는 어디? 주변은 안전해?")                      │
└─────────────────────────────────────────────────────────────┘
```

**왜 이렇게 계층을 나눴을까?** 소프트웨어 아키텍처의 고전적 원칙인 **계층화(Layering)**를 따른 것입니다. 각 계층은 바로 아래 계층만 의존하고, 구현 세부사항을 숨깁니다(Abstraction). 예를 들어, Behavior Tree는 "경로를 계획하라"고 명령만 하면 되지, A* 알고리즘의 휴리스틱 함수가 무엇인지 알 필요가 없습니다. 이것이 **추상화의 힘**입니다.

이제 각 레이어를 하나씩 깊이 파헤쳐 보겠습니다.

---

## 🧭 Perception Layer: 세계를 이해하기

### AMCL (Adaptive Monte Carlo Localization): "나는 어디에 있는가?"

자율주행의 첫 번째 문제는 **Localization**입니다. 로봇이 자신의 위치를 모른다면, 어떤 경로 계획도 무의미합니다. 하지만 로봇의 센서는 완벽하지 않습니다. Odometry(바퀴 회전 센서)는 미끄러지면 오차가 누적되고, LiDAR는 유리나 거울 앞에서 혼란스러워합니다. 그렇다면 어떻게 "확신"을 가질 수 있을까요?

**AMCL의 철학: "여러 가설을 동시에 유지하라"**

AMCL은 **Particle Filter**라는 확률적 방법을 사용합니다. 이것은 "로봇이 (x, y, θ) 위치에 있다"고 단정하지 않고, **수백~수천 개의 가능한 위치(파티클)**를 동시에 추적합니다. RViz에서 본 녹색 화살표 구름이 바로 이 파티클들입니다.

**작동 원리**:

1. **예측(Prediction)**: 로봇이 "앞으로 1m 이동" 명령을 받으면, 모든 파티클도 1m씩 이동시킵니다. 하지만 현실은 불완전하므로, 각 파티클에 약간의 노이즈를 추가합니다. "어쩌면 0.98m 갔을 수도, 1.02m 갔을 수도..."

2. **관측(Update)**: LiDAR로 벽을 스캔합니다. 각 파티클 위치에서 "만약 내가 여기 있다면, 벽이 저기 보여야 해"를 계산하고, 실제 센서 데이터와 비교합니다. 일치하는 파티클은 **가중치를 높이고**, 일치하지 않는 파티클은 가중치를 낮춥니다.

3. **리샘플링(Resampling)**: 가중치가 높은 파티클 주변에서 새로운 파티클을 생성하고, 가중치가 낮은 파티클은 제거합니다. 마치 "좋은 가설에 베팅을 늘리고, 나쁜 가설은 포기하는" 것입니다.

4. **수렴(Convergence)**: 로봇이 움직이고 센서 데이터를 받을수록, 파티클들이 점점 한 곳으로 모입니다. "나는 99% 확률로 여기 있어!"

**왜 이 방법이 강력한가?**

- **다중 가설**: 처음에는 불확실해도, 시간이 지나면 수렴합니다.
- **Kidnapped Robot 문제 해결**: 로봇을 갑자기 다른 곳으로 옮겨도, 파티클이 다시 퍼졌다가 올바른 위치로 수렴합니다.
- **센서 노이즈 극복**: 하나의 센서 측정이 틀려도, 확률적으로 평균을 내기 때문에 robust합니다.

**실무 설계 선택**: "왜 Kalman Filter가 아닌 Particle Filter인가?"

Kalman Filter는 가우시안 분포를 가정하지만, 실제 로봇의 위치 불확실성은 종종 **다봉 분포(Multimodal Distribution)**입니다. 예를 들어, 복도의 양쪽 끝이 대칭이라면, "나는 왼쪽 끝에 있을 수도, 오른쪽 끝에 있을 수도"라는 두 가설이 동시에 합리적입니다. Particle Filter는 이런 경우를 자연스럽게 표현할 수 있지만, Kalman Filter는 단봉 가우시안으로 강제하기 때문에 실패합니다.

---

### Costmap: 위험도 지도

로봇이 자신의 위치를 알았다면, 다음은 **"주변이 안전한가?"**를 판단해야 합니다. Navigation2는 이를 **Costmap**이라는 개념으로 해결합니다.

**Costmap이란?**: 격자(Grid) 형태의 맵에서 각 셀에 **비용(Cost)** 값을 할당한 것입니다. 비용이 높을수록 위험하고, 낮을수록 안전합니다. RViz에서 본 보라색/분홍색 영역이 바로 Costmap입니다.

**비용 계산 방식**:

- **장애물(Obstacle)**: 255 (절대 못 감)
- **Inflation Zone**: 장애물 주변 수십 cm는 거리에 반비례하여 비용 할당 (예: 253 → 200 → 150 → ...)
- **자유 공간(Free Space)**: 0 (안전)
- **미지 영역(Unknown)**: 별도 처리

**왜 Inflation이 필요한가?** 로봇은 점이 아니라 물리적 크기를 가진 물체입니다. 벽에서 정확히 1cm만 떨어져도 "충돌 안 함"이라고 계획하면, 실제로는 모터 제어 오차, 센서 지연 등으로 부딪힐 수 있습니다. Inflation은 **안전 여유(Safety Margin)**를 제공합니다. 이것은 실무에서 **방어적 설계(Defensive Design)**라고 부르는 원칙입니다.

**Global Costmap vs Local Costmap**: Navigation2는 두 개의 Costmap을 유지합니다. 왜일까요?

**Global Costmap**:
- **범위**: 전체 맵 (수십~수백 미터)
- **갱신 빈도**: 느림 (1Hz)
- **용도**: 장기 경로 계획 (Global Planner)
- **특징**: 정적 장애물 중심 (맵에 있는 벽)

**Local Costmap**:
- **범위**: 로봇 주변 수 미터
- **갱신 빈도**: 빠름 (10Hz 이상)
- **용도**: 실시간 장애물 회피 (Local Planner)
- **특징**: 동적 장애물 포함 (사람, 다른 로봇)

**설계 트레이드오프**: "왜 하나의 Costmap으로 통합하지 않았나?"

만약 전체 맵을 10Hz로 갱신한다면, 계산량이 폭발합니다(100m × 100m × 10Hz = 100만 셀/초). 대신, 장기 계획은 정적 환경만 고려하고(Global), 단기 반응은 동적 환경에 집중하면(Local), **계산 비용을 수십 배 절약**할 수 있습니다. 이것은 **계층적 의사결정(Hierarchical Decision Making)**의 전형적인 사례입니다.

---

### Costmap의 레이어 시스템: 플러그인 아키텍처

Costmap은 여러 **레이어(Layer)**를 겹쳐서 만듭니다. 마치 포토샵의 레이어처럼, 각 레이어는 독립적으로 비용을 계산하고, 최종 비용은 레이어들을 합성한 결과입니다.

**주요 레이어들**:

1. **Static Layer**: 저장된 맵(PGM 파일)에서 벽 위치 로드
2. **Obstacle Layer**: LiDAR로 실시간 감지한 장애물 추가
3. **Inflation Layer**: 장애물 주변에 비용 전파
4. **Voxel Layer**: 3D 센서(Depth Camera)로 3차원 장애물 처리
5. **Range Sensor Layer**: 초음파 센서 등의 간단한 센서 통합

**왜 플러그인 구조인가?** 이것은 소프트웨어 아키텍처의 **개방-폐쇄 원칙(Open-Closed Principle)**을 따른 것입니다. "확장에는 열려 있고, 수정에는 닫혀 있다."

새로운 센서를 추가하고 싶다면? Costmap 코어를 수정할 필요 없이, 새로운 레이어 플러그인만 작성하면 됩니다. 예를 들어, "사람이 많은 곳은 비용을 높여라" 같은 사회적 내비게이션(Social Navigation) 레이어를 추가할 수 있습니다. 이것이 **확장 가능한 아키텍처(Extensible Architecture)**의 힘입니다.

---

## 🛣️ Planning Layer: 경로 계획의 이중 구조

### Global Planner: 전체 전략 수립

**역할**: "출발지 A에서 목적지 B까지 전체 경로를 계획하라"

**입력**:
- 시작 위치: AMCL이 추정한 현재 위치
- 목표 위치: 사용자가 Nav2 Goal로 지정한 위치
- Global Costmap: 전체 맵의 장애물 정보

**출력**:
- Path: (x, y, θ) 포인트들의 시퀀스

**알고리즘 선택지**:

1. **Dijkstra**: 모든 방향으로 탐색, 항상 최단 경로 보장
2. **A\* (A-star)**: 목표 방향 우선 탐색, Dijkstra보다 수십 배 빠름
3. **Theta\***: A*보다 더 부드러운 경로, 대각선 이동 최적화
4. **Hybrid A\***: 자동차처럼 회전 반경이 있는 로봇을 위한 알고리즘

**실무 선택: "왜 A*가 기본인가?"**

Dijkstra는 이론적으로 완벽하지만, 100m × 100m 맵에서 10만 개 셀을 탐색하면 수 초가 걸립니다. A*는 **휴리스틱(Heuristic)** 함수(목표까지의 직선 거리)로 탐색 방향을 가이드하여, 1만 개 셀만 탐색해도 최단 경로를 찾습니다. 이것은 **"좋은 추측(Informed Search)"**이 얼마나 강력한지 보여주는 사례입니다.

**Theta*는 왜 항상 사용하지 않나?** Theta*는 그리드가 아닌 임의의 각도로 이동 가능하므로, 경로가 더 자연스럽습니다(사람이 걷는 것처럼). 하지만 계산이 더 복잡하고, 좁은 복도에서는 A*와 큰 차이가 없습니다. **트레이드오프**: 부드러운 경로 vs 계산 속도.

---

### Local Planner: 실시간 장애물 회피

**역할**: "Global Path를 따라가되, 갑자기 나타나는 장애물을 피하라"

Global Planner는 전체 맵을 보지만, 미래를 예측할 수 없습니다. 사람이 갑자기 앞을 가로지르거나, 의자가 복도에 놓여 있다면? 이때 **Local Planner**가 개입합니다.

**입력**:
- Global Path: 전체 경로 (참조용)
- Local Costmap: 로봇 주변 수 미터의 실시간 장애물
- 로봇의 현재 속도: (vx, vy, vω)
- 로봇의 물리적 제약: 최대 속도, 가속도, 회전 반경

**출력**:
- 속도 명령: (linear.x, angular.z) - 다음 0.1초 동안 로봇이 따라야 할 속도

**알고리즘 선택지**:

1. **DWA (Dynamic Window Approach)**:
   - 로봇의 가속도 제약을 고려하여, 현재 속도에서 0.1초 후 도달 가능한 속도들의 "윈도우"를 샘플링
   - 각 속도 샘플에 대해 "이 속도로 가면 몇 초 후 어디에 도착?" 시뮬레이션
   - 장애물 회피, Global Path 추종, 속도 최대화 등을 점수화하여 최고 점수 선택
   - **빠르고 안정적**, 대부분의 Differential Drive 로봇에 적합

2. **TEB (Timed Elastic Band)**:
   - 전체 궤적을 **최적화 문제**로 모델링
   - "시간을 최소화하면서, 장애물을 피하고, 로봇의 동역학을 만족하는 경로"를 수학적으로 계산
   - **매우 부드러운 경로**, 자동차나 Ackermann 로봇에 적합
   - 계산량이 많아서 고성능 CPU 필요

3. **MPPI (Model Predictive Path Integral)**:
   - 수천 개의 랜덤 궤적을 샘플링하고, 비용이 낮은 궤적들의 가중 평균 계산
   - GPU 가속 가능, 최신 연구 동향
   - 매우 복잡한 환경에서 강력하지만, 튜닝이 어려움

**실무 선택: "왜 DWA가 기본인가?"**

TEB가 더 부드러운 경로를 만들지만, 계산량이 5~10배 많습니다. 저가형 로봇(Raspberry Pi)에서는 DWA만으로도 충분히 좋은 성능을 냅니다. **"80% 품질을 20% 비용으로"** 달성하는 것이 실무의 현실입니다.

**Local Planner의 철학: "지금 당장 안전한 선택"**

Global Planner는 완벽한 정보를 가정하지만, Local Planner는 **불확실성 속에서 즉각적인 결정**을 내립니다. 마치 자동차를 운전할 때, 내비게이션은 전체 경로를 알려주지만(Global), 앞차가 갑자기 브레이크를 밟으면 운전자가 순간 판단하는 것(Local)과 같습니다.

---

### Global vs Local의 계층적 설계: 왜 두 개로 나눴나?

**근본적인 질문**: "하나의 강력한 Planner로 모든 것을 해결하면 안 되나?"

이론적으로는 가능합니다. 하지만 실무에서는 **불가능**합니다. 왜일까요?

**계산 복잡도의 저주**:
- Global Planning: 100m × 100m = 10,000 셀, 1초에 1번 계획 가능
- Local Planning: 5m × 5m = 25 셀, 1초에 10번 계획 가능

만약 전체 맵을 10Hz로 계획한다면, 계산량이 400배 증가합니다. 일반 CPU로는 불가능합니다.

**추상화 수준의 차이**:
- Global: "A 방에서 B 방으로 가려면, 복도를 통과해야 해" (미터 단위, 분 단위)
- Local: "0.5초 후 의자를 피하려면, 지금 5도 왼쪽으로 회전" (cm 단위, 0.1초 단위)

이 두 가지는 **다른 시공간 스케일**에서 작동합니다. 마치 군대에서 장군은 전체 전략을, 병사는 전술을 담당하듯이, 계층적 의사결정은 복잡한 시스템의 필수 패턴입니다.

**실패 모드의 분리**:
- Global Planning 실패: "목표에 도달 불가능" (맵이 막힌 경우)
- Local Planning 실패: "일시적으로 막힘" (사람이 앞을 가로막은 경우)

두 개를 분리하면, Local이 실패해도 Global은 계속 작동하고, 다른 경로를 재계획할 수 있습니다. 이것이 **Fault Isolation**(장애 격리)입니다.

---

## 🎮 Control Layer: 모션 제어

### Controller Server: 경로를 속도로 변환

**역할**: "Local Planner의 경로를 로봇의 실제 속도 명령으로 변환하라"

많은 사람들이 "Local Planner가 속도를 출력하지 않나?"라고 생각하지만, 정확히는:
- **Local Planner**: 궤적(Trajectory) 생성 - "앞으로 0.5m, 왼쪽으로 15도"
- **Controller**: 이 궤적을 추종하기 위한 제어 명령 - "linear.x = 0.22, angular.z = 0.15"

**왜 분리했나?** 경로 계획과 제어는 **다른 전문 분야**입니다. 경로 계획은 기하학과 최적화의 문제이고, 제어는 **피드백 루프와 안정성**의 문제입니다.

**Controller 알고리즘**:

1. **RPP (Regulated Pure Pursuit)**:
   - 경로 위의 "Look-Ahead Point"를 추적
   - 거리와 각도 오차를 속도로 변환
   - 간단하고 안정적

2. **DWB (Dynamic Window Approach + Behavior)**:
   - Local Planner + Controller 통합
   - 속도 공간에서 직접 샘플링

3. **MPC (Model Predictive Control)**:
   - 로봇의 동역학 모델을 사용하여 미래 예측
   - 최적 제어 이론
   - 복잡하지만 매우 정확

**실무에서 가장 중요한 것**: **Tuning**(파라미터 조정)

같은 알고리즘이라도, 파라미터에 따라 로봇이 거북이처럼 느리게 움직이거나, 토끼처럼 빠르지만 불안정하게 움직일 수 있습니다. 실무에서는 알고리즘 선택보다 **파라미터 튜닝**에 더 많은 시간을 씁니다.

**주요 파라미터**:
- `max_vel_x`: 최대 직진 속도 (m/s)
- `max_vel_theta`: 최대 회전 속도 (rad/s)
- `acc_lim_x`, `acc_lim_theta`: 가속도 제한
- `xy_goal_tolerance`: 목표 도달 판정 거리 (m)
- `yaw_goal_tolerance`: 목표 도달 판정 각도 (rad)

이 값들을 조정하는 것은 **예술이자 과학**입니다. 너무 높으면 로봇이 불안정하고, 너무 낮으면 답답합니다.

---

## 🌳 Application Layer: Behavior Tree Navigator

### BT Navigator: 고수준 의사결정

지금까지 Perception(AMCL, Costmap), Planning(Global/Local Planner), Control(Controller)을 봤습니다. 하지만 누가 이들을 조율할까요? "계획 실패하면 어떻게 하지?", "갇혔으면 후진해야 하나?"와 같은 **고수준 의사결정**은 누가 합니까?

바로 **Behavior Tree Navigator**입니다.

**Behavior Tree란?**

게임 AI에서 유래한 의사결정 구조입니다. "만약 A라면 B를 해라, 실패하면 C를 해라"를 트리 형태로 표현합니다.

**Navigation2의 기본 Behavior Tree 구조**:

```
Root: NavigateToPose
├─ ComputePathToPose        (Global Planner 호출)
│   └─ Success → 다음 단계
│   └─ Failure → Recovery 시도
├─ FollowPath                (Local Planner + Controller 호출)
│   └─ Success → 목표 도달
│   └─ Failure → Recovery 시도
└─ Recovery Behaviors
    ├─ ClearCostmap          (Costmap 초기화)
    ├─ Spin                  (제자리 360도 회전)
    ├─ BackUp                (후진)
    └─ Wait                  (장애물이 사라질 때까지 대기)
```

**왜 Behavior Tree인가?**

전통적으로 **유한 상태 머신(Finite State Machine, FSM)**을 사용했습니다. 하지만 FSM은 상태가 많아지면 **상태 전이(State Transition)가 스파게티**가 됩니다. Behavior Tree는:
- **계층적**: 복잡한 행동을 작은 행동의 조합으로 표현
- **재사용 가능**: 서브트리를 다른 곳에서 재사용
- **확장 가능**: XML 파일로 정의하여 코드 수정 없이 변경

**실무 예시**: "로봇이 갇혔을 때"

1. **FollowPath 실패** → BT Navigator가 감지
2. **ClearCostmap** 시도 → Costmap의 오래된 장애물 정보 제거
3. **ComputePathToPose 재시도** → 새 경로 계획
4. 여전히 실패 → **Spin** (360도 회전하며 주변 재스캔)
5. 여전히 실패 → **BackUp** (1m 후진)
6. 여전히 실패 → **Wait** (10초 대기)
7. 모두 실패 → **Goal Aborted** (사용자에게 실패 보고)

이 모든 것이 코드 한 줄 없이, **BT XML 파일**에 정의되어 있습니다. 만약 "Spin 후 BackUp" 순서를 바꾸고 싶다면? XML만 수정하면 됩니다. 이것이 **선언적 설계(Declarative Design)**의 힘입니다.

---

## 🔗 TF Tree: 좌표계의 계층 구조

### 왜 여러 좌표계가 필요한가?

로봇 시스템에는 수많은 센서와 액추에이터가 있습니다. LiDAR는 로봇의 왼쪽에 달려있고, 카메라는 위에 달려있고, 바퀴는 아래에 있습니다. 각 센서가 "나는 앞에 벽을 봤어!"라고 할 때, 그 "앞"은 누구 기준입니까?

**TF(Transform) 시스템**은 모든 좌표계 간의 관계를 관리합니다. Navigation2에서 핵심적인 TF Tree는:

```
map (전체 환경의 절대 좌표계)
 └─ odom (로봇 시작 위치 기준 상대 좌표계)
     └─ base_link (로봇의 중심)
         ├─ laser_frame (LiDAR)
         ├─ camera_frame (카메라)
         └─ wheel_left, wheel_right (바퀴들)
```

**각 프레임의 역할**:

**1. map 프레임**:
- "우주의 원점", 절대적 기준
- SLAM이나 미리 저장된 맵의 좌표계
- **특징**: 시간이 지나도 변하지 않음
- **누가 발행하나?**: AMCL (map → odom 변환)

**2. odom 프레임**:
- 로봇이 시작한 위치를 원점으로 하는 좌표계
- Odometry(바퀴 센서)가 측정하는 상대적 이동
- **특징**: 시간이 지나면 오차 누적 (미끄러짐, 충격 등)
- **누가 발행하나?**: Gazebo 또는 실제 로봇의 Odometry 노드 (odom → base_link 변환)

**3. base_link 프레임**:
- 로봇의 물리적 중심
- 모든 센서는 base_link 기준으로 위치가 정의됨
- **특징**: 로봇과 함께 움직임

**왜 map과 odom을 분리했나?**

**핵심 아이디어**: "짧은 시간에는 Odometry가 정확하고, 긴 시간에는 AMCL이 정확하다"

- **Odometry**: 0.01초 단위로 매우 정확하지만, 10분 후에는 수 미터 오차
- **AMCL**: 1초 단위로 업데이트, 하지만 장기적으로 정확 (맵과 비교하므로)

**map → odom 변환**은 바로 이 **오차 보정(Correction)**입니다. AMCL이 "오, Odometry가 누적 오차로 2m 틀렸네"라고 판단하면, map → odom 변환에 2m 오프셋을 추가합니다.

**실무 예시**: "로봇이 미끄러졌을 때"

1. 로봇이 1m 전진 명령 받음
2. Odometry는 "1m 갔어!" (odom → base_link: x += 1.0)
3. 하지만 실제로는 미끄러져서 0.8m만 감
4. AMCL이 LiDAR로 스캔: "어? 벽이 예상보다 20cm 가까워"
5. AMCL이 map → odom 변환 업데이트: x -= 0.2
6. 결과: map 기준으로는 로봇이 정확히 0.8m 위치

이것이 **센서 융합(Sensor Fusion)**입니다. 여러 센서의 장점을 결합하여, 단일 센서보다 더 정확한 결과를 얻습니다.

---

### TF의 실무적 가치: Loose Coupling

TF가 없다면, 각 노드가 "LiDAR가 base_link에서 (0.2, 0, 0.3) 떨어져 있다"를 하드코딩해야 합니다. 로봇을 업그레이드해서 LiDAR 위치를 바꾸면? 모든 노드의 코드를 수정해야 합니다.

TF 시스템 덕분에:
- **URDF(로봇 모델 파일)**에만 센서 위치 정의
- 모든 노드는 `tf_listener.lookup_transform("base_link", "laser_frame")`로 동적으로 조회
- 센서 위치 변경 → URDF만 수정, 코드는 그대로

이것이 **데이터-코드 분리**입니다. 설정이 코드에 박혀있지 않고, 외부 파일로 관리됩니다.

---

## ⚙️ Lifecycle 노드: 상태 관리의 필요성

### ROS 2의 Lifecycle 노드란?

ROS 1에서 노드는 단순했습니다: 시작하거나, 종료하거나. 하지만 실제 로봇 시스템은 더 복잡합니다:

- "노드를 시작했는데, 파라미터 파일을 못 찾으면?"
- "노드를 종료하기 전에, 진행 중인 작업을 안전하게 마무리하려면?"
- "노드를 '일시정지'하고 싶으면?"

ROS 2는 이를 위해 **Lifecycle 노드**를 도입했습니다.

**Lifecycle 상태 머신**:

```
Unconfigured (시작 직후)
     ↓ [configure] - 파라미터 로드, 리소스 할당
Inactive (설정 완료, 아직 실행 안 함)
     ↓ [activate] - 실제 작업 시작
Active (정상 작동 중)
     ↓ [deactivate] - 일시정지
Inactive
     ↓ [cleanup] - 리소스 해제
Unconfigured
     ↓ [shutdown]
Finalized (완전 종료)
```

**각 상태의 의미**:

- **Unconfigured**: 노드가 막 생성됨, 아무것도 안 함
- **Inactive**: 파라미터는 로드했지만, Publisher/Subscriber는 활성화 안 함
- **Active**: 모든 기능 작동 중
- **Finalized**: 완전 종료

**왜 이렇게 복잡한가?**

**1. 안전한 시작(Safe Startup)**:

`configure` 단계에서 파라미터 파일이 잘못되었다면, `Inactive` 상태에 머물며 에러를 보고합니다. `Active`로 가지 않으므로, 잘못된 설정으로 로봇이 움직이는 일이 없습니다.

**2. 동기화된 시작(Coordinated Startup)**:

여러 노드를 동시에 시작할 때, 모두 `Inactive` 상태까지 가게 한 후, 한꺼번에 `Active`로 전환할 수 있습니다. 그러면 "A 노드는 시작했는데 B 노드는 아직"인 중간 상태를 피할 수 있습니다.

**3. 우아한 종료(Graceful Shutdown)**:

`deactivate` → `cleanup` 순서로 종료하면, 진행 중인 작업을 마무리하고, 파일을 안전하게 닫고, 메모리를 해제할 수 있습니다. 갑자기 `Ctrl+C`로 강제 종료하는 것보다 훨씬 안전합니다.

---

### Lifecycle Manager: 중앙 조율자

Navigation2에는 수십 개의 노드가 있습니다. 이들을 모두 수동으로 `configure` → `activate` 하는 것은 불가능합니다. 그래서 **Lifecycle Manager**가 있습니다.

**역할**:
- 관리할 노드 목록을 파라미터로 받음
- 모든 노드를 순서대로 `configure` → `activate`
- 하나라도 실패하면, 전체를 `deactivate` → `cleanup`
- **All-or-Nothing** 방식

**Navigation2의 두 개의 Lifecycle Manager**:

1. **lifecycle_manager_localization**:
   - 관리 노드: `map_server`, `amcl`
   - 역할: Localization 시스템 시작

2. **lifecycle_manager_navigation**:
   - 관리 노드: `planner_server`, `controller_server`, `bt_navigator`, `behavior_server`
   - 역할: Navigation 시스템 시작

**왜 두 개로 나눴나?** Localization과 Navigation은 **다른 수명 주기**를 가집니다. Localization은 한 번 시작하면 계속 돌아가지만, Navigation은 목표에 도달하면 멈추고 새 목표를 기다립니다. 분리하면 Navigation만 재시작해도 Localization은 그대로 유지됩니다.

---

## 🐛 트러블슈팅 경험: 실전 문제 해결

오늘 학습에서 세 가지 주요 문제를 만났습니다. 각 문제는 실무에서도 매우 흔한 패턴입니다.

### 문제 1: RViz 창 2개 중복

**증상**: RViz 창이 2개 열림, 어느 창을 사용해야 할지 혼란

**원인**:
- `navigation2.launch.py`가 자체적으로 RViz를 실행함
- 사용자가 별도로 `nav2_rviz.launch.py`도 실행함
- 결과: RViz 인스턴스 2개

**근본 원인 분석**:

Launch 파일의 **책임(Responsibility)** 설계 실패입니다. "누가 RViz를 띄우는가?"가 명확하지 않았습니다.

**실무 원칙**: **Single Responsibility Principle** (단일 책임 원칙)

각 Launch 파일은 하나의 명확한 책임을 가져야 합니다:
- `navigation2.launch.py`: Navigation 시스템 전체 시작 (RViz 포함)
- `nav2_rviz.launch.py`: RViz만 시작 (Navigation은 이미 실행 중인 경우)

두 개를 동시에 실행하면 안 됩니다. 이것은 마치 웹 서버를 2개 띄우고 같은 포트를 listen하려는 것과 같습니다.

**해결책**:
- 커스텀 RViz 설정을 사용하고 싶다면: `navigation2.launch.py use_rviz:=false` + `nav2_rviz.launch.py`
- 기본 설정을 사용한다면: `navigation2.launch.py`만 실행

**배운 점**: 분산 시스템에서 **책임 분리**가 명확하지 않으면, 중복이나 충돌이 발생합니다.

---

### 문제 2: Lifecycle 상태 충돌 (Active 상태에서 Configure 시도)

**증상**:
```
[ERROR] [controller_server]: Unable to start transition 1 from current state active
[ERROR] [lifecycle_manager_navigation]: Failed to bring up all requested nodes. Aborting bringup.
```

**원인**:

노드들이 이미 `Active` 상태인데, `lifecycle_manager`가 다시 `configure` (transition 1)를 시도했습니다. Lifecycle 상태 머신에서는 `Active → Inactive(deactivate)` → `Unconfigured(cleanup)` → `Inactive(configure)`가 정상 순서인데, `Active → Inactive(configure)`는 불가능합니다.

**근본 원인 분석**:

**상태 불일치(State Inconsistency)**입니다. 시스템의 일부는 "노드들이 이미 시작됨"이라고 생각하고, 다른 부분은 "아직 시작 안 됨"이라고 생각합니다.

**왜 이런 일이?**

1. 이전 실행에서 노드들이 완전히 종료되지 않음 (좀비 프로세스)
2. 빌드 후 재시작 없이 실행 → 새 코드 로드 안 됨
3. RViz에서 "Startup" 버튼을 여러 번 클릭 → lifecycle_manager가 중복 실행

**실무에서 이런 경우**: 매우 흔합니다. 특히 **장시간 실행되는 시스템**(로봇, 서버 등)에서 "재시작하지 않고 설정만 변경"하려 할 때 자주 발생합니다.

**해결책**:

**Clean Restart Protocol** (클린 재시작 프로토콜):
1. 모든 ROS 프로세스 종료 (`Ctrl+C` 또는 `killall -9`)
2. ROS 환경 재설정 (`source ~/ros2_ws/install/setup.bash`)
3. 정해진 순서로 Launch 파일 실행
4. 각 Launch가 완전히 시작될 때까지 대기 ("All managed nodes are active" 확인)

**배운 점**: 분산 시스템에서 **상태 일관성(State Consistency)**은 매우 중요합니다. 일부만 재시작하면 전체 시스템이 inconsistent 상태가 됩니다.

---

### 문제 3: TF 에러 (map 프레임이 존재하지 않음)

**증상**:
```
[global_costmap]: Timed out waiting for transform from base_link to map
tf error: Invalid frame ID "map" passed to canTransform - frame does not exist
```

**원인**:

`map` 프레임을 발행해야 할 노드(Map Server 또는 AMCL)가 시작되지 않았거나, 맵 파일 경로가 잘못되었습니다.

**근본 원인 분석**:

**의존성 실패(Dependency Failure)**입니다. Global Costmap은 `map` 프레임에 **의존**하는데, 그 의존성이 충족되지 않았습니다.

**TF의 특성**: TF는 **시간 지연 허용(Timeout)**이 있습니다. "10초 동안 map 프레임이 나타나길 기다려, 안 나타나면 에러"

**왜 map 프레임이 없었나?**

1. 맵 파일 경로 오류 → Map Server 시작 실패 → map 프레임 발행 안 됨
2. AMCL이 activate 안 됨 → map → odom 변환 발행 안 됨

**실무에서 이런 경우**: TF 에러는 **가장 흔한 ROS 에러** 중 하나입니다. 로봇 시스템은 수십 개의 프레임이 얽혀 있어서, 하나라도 빠지면 전체가 작동 안 합니다.

**디버깅 방법**:

```bash
# 현재 발행되는 모든 TF 프레임 확인
ros2 run tf2_tools view_frames

# 특정 변환이 가능한지 확인
ros2 run tf2_ros tf2_echo map base_link
```

**해결책**:

1. 맵 파일 경로 확인: `ls -lh ~/my_turtlebot3_map.*`
2. 절대 경로 사용: `map:=$HOME/my_turtlebot3_map.yaml`
3. Launch 로그에서 Map Server 시작 확인: `[map_server]: Loading map from ...`

**배운 점**: 분산 시스템에서 **의존성 그래프**를 명확히 이해해야 합니다. A가 B에 의존하고, B가 C에 의존한다면, C가 실패하면 전체가 실패합니다.

---

## 🏗️ 아키텍처 설계 원칙 및 실무 교훈

### 1. 느슨한 결합 (Loose Coupling)

Navigation2의 모든 노드는 **Topic, Service, Action**으로만 소통합니다. 직접적인 함수 호출이나 공유 메모리가 없습니다.

**장점**:
- 한 노드가 죽어도 다른 노드는 계속 작동
- 노드를 다른 언어로 재구현 가능 (Python → C++)
- 노드를 다른 컴퓨터로 분산 가능 (Network Transparency)

**실무 사례**: 만약 Local Planner가 버그로 crash한다면, Global Planner와 AMCL은 여전히 작동합니다. 사용자는 "경로 추종이 안 되네"라고 알아차리지만, 전체 시스템은 살아있습니다. 이것이 **Graceful Degradation**(우아한 성능 저하)입니다.

---

### 2. 관심사의 분리 (Separation of Concerns)

각 노드는 하나의 일만 잘합니다:
- AMCL: Localization만
- Global Planner: 전체 경로 계획만
- Local Planner: 지역 장애물 회피만

**장점**:
- 각 노드를 독립적으로 테스트 가능
- 버그 발생 시 원인 추적이 쉬움 (책임이 명확)
- 노드 교체 용이 (A* → Dijkstra 교체해도 다른 노드는 영향 없음)

**안티패턴**: "God Node" (신 노드)

만약 하나의 노드가 "센서 읽기 + Localization + Path Planning + Control"을 모두 한다면? 코드는 수천 줄이 되고, 버그를 찾기 불가능해집니다.

---

### 3. 계층적 추상화 (Hierarchical Abstraction)

BT Navigator → Planner → Controller → Costmap 순으로, 각 계층은 하위 계층의 복잡성을 숨깁니다.

**장점**:
- 고수준에서는 간단한 인터페이스만 보임
- 하위 계층 변경이 상위 계층에 영향 없음

**실무 사례**: BT Navigator는 "ComputePathToPose"라는 추상적 명령만 호출합니다. Global Planner가 내부적으로 A*를 쓰든 Dijkstra를 쓰든 상관없습니다. 이것이 **의존성 역전 원칙(Dependency Inversion Principle)**입니다.

---

### 4. 설정과 코드의 분리 (Configuration vs Code)

Navigation2의 대부분의 동작은 **YAML 파일**로 설정됩니다. 코드를 재컴파일하지 않고도 로봇의 속도, Costmap 크기, Recovery 동작 등을 변경할 수 있습니다.

**장점**:
- 비개발자(로봇 운영자)도 파라미터 조정 가능
- 동일한 코드로 여러 로봇 지원 (설정 파일만 교체)
- A/B 테스트 용이

**실무 사례**: 같은 Navigation2 코드로, TurtleBot3(Differential Drive)과 자동차(Ackermann Steering)를 지원할 수 있습니다. 단지 파라미터 파일을 다르게 쓰면 됩니다.

---

### 5. Fault Tolerance (장애 허용)

Navigation2는 부분 실패를 가정하고 설계되었습니다.

**Recovery Behaviors**:
- Global Planning 실패 → Recovery → 재시도
- Local Planning 막힘 → Spin, BackUp → 재시도
- 센서 일시적 오류 → Timeout 후 재시도

**실무에서 왜 중요한가?**: 실제 로봇은 완벽하지 않습니다. 센서가 일시적으로 노이즈를 받거나, 사람이 갑자기 앞을 가로막거나, 바닥이 미끄럽거나... 이런 상황에서도 로봇이 **계속 동작**하려면, 실패를 예상하고 대비해야 합니다.

---

## 🚨 안티패턴 및 함정

### 안티패턴 1: 모든 것을 하나의 Costmap에

**잘못된 생각**: "Global Costmap과 Local Costmap을 하나로 통합하면 간단하지 않을까?"

**문제**:
- 전체 맵을 10Hz로 갱신 → CPU 폭발
- 또는 지역 맵을 1Hz로 갱신 → 장애물 회피 느림

**교훈**: **적절한 추상화 수준**을 유지하세요. 장기 계획과 단기 반응은 다른 시간 스케일에서 작동합니다.

---

### 안티패턴 2: 파라미터를 코드에 하드코딩

**잘못된 생각**: "max_vel = 0.5로 고정하면 간단하잖아"

**문제**:
- 로봇을 업그레이드(더 빠른 모터) → 코드 재컴파일
- 다른 환경(넓은 창고 vs 좁은 복도) → 각각 다른 바이너리 필요

**교훈**: **설정 외부화(Configuration Externalization)**. 모든 튜닝 가능한 값은 파라미터 파일로.

---

### 안티패턴 3: 에러 무시하기

**잘못된 생각**: "TF 에러는 가끔 나오는데 무시해도 되겠지?"

**문제**:
- 에러가 누적되면 로봇이 갑자기 이상하게 움직임
- 디버깅 시 원인 추적 불가능

**교훈**: **Fail Fast** (빨리 실패하라). 에러가 발생하면 즉시 멈추고 원인을 찾으세요. 에러를 덮으면 나중에 더 큰 문제가 됩니다.

---

### 함정 1: Lifecycle 상태 불일치

**증상**: "분명 노드를 시작했는데 작동 안 함"

**원인**: 노드가 `Inactive` 상태에 머물러 있음 (activate 안 됨)

**예방**: 항상 Launch 로그에서 `All managed nodes are active` 확인

---

### 함정 2: 2D Pose Estimate 없이 Nav2 Goal

**증상**: "Nav2 Goal을 설정했는데 로봇이 안 움직여"

**원인**: AMCL이 초기 위치를 모름 → Localization 불가능 → Planning 불가능

**예방**: Navigation 시작 후 항상 2D Pose Estimate부터 설정

---

### 함정 3: Costmap의 Inflation Radius 너무 큼/작음

**너무 크면**: 로봇이 벽에서 너무 멀리 떨어져 움직임, 좁은 복도 통과 불가
**너무 작으면**: 로봇이 벽에 너무 가까이 가서 충돌 위험

**교훈**: Inflation Radius = 로봇 반경 + 안전 여유 (보통 로봇 반경의 1.5~2배)

---

## 🎓 핵심 개념 정리

### Navigation2 = 5개의 질문에 대한 답

1. **"나는 어디?"** → AMCL (Particle Filter)
2. **"주변이 안전해?"** → Costmap (Layered Architecture)
3. **"어떻게 가야 해?"** → Global Planner (A*)
4. **"갑자기 장애물이!"** → Local Planner (DWA)
5. **"실패하면?"** → BT Navigator (Recovery Behaviors)

---

### 아키텍처 패턴 매핑

| Navigation2 컴포넌트 | 소프트웨어 패턴 |
|---------------------|----------------|
| Topic 기반 통신 | Publish-Subscribe Pattern |
| Lifecycle 노드 | State Machine Pattern |
| Behavior Tree | Strategy Pattern + Chain of Responsibility |
| Costmap Layers | Decorator Pattern |
| Global + Local Planner | Hierarchical Planning |
| TF System | Observer Pattern + Mediator |

(표는 보조적으로만 사용, 각 패턴의 의미는 본문에서 길게 설명함)

---

### Day 6 (SLAM)과 Day 7 (Navigation)의 관계

**SLAM**: "맵을 만들며 동시에 자신의 위치를 찾는다" (Building + Localization)
**Navigation**: "이미 만들어진 맵에서 자신의 위치를 찾고, 목표로 이동한다" (Localization + Planning)

**공통점**: 둘 다 "나는 어디?" 문제를 풉니다.
**차이점**: SLAM은 맵도 모르고, Navigation은 맵을 알고 있습니다.

**실무 워크플로우**:
1. SLAM으로 환경 탐색 + 맵 생성 (Day 6)
2. 맵 저장 (PGM + YAML)
3. Navigation2로 맵 로드 + 자율주행 (Day 7)
4. 환경이 바뀌면? 다시 SLAM으로 맵 업데이트

---

## 🔧 실습 과정

### 최종 성공 절차

1. **Gazebo 시작**:
   ```bash
   source ~/ros2_ws/install/setup.bash
   export TURTLEBOT3_MODEL=waffle
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

2. **Navigation2 시작** (Day 6 맵 사용):
   ```bash
   source ~/ros2_ws/install/setup.bash
   export TURTLEBOT3_MODEL=waffle
   ros2 launch turtlebot3_navigation2 navigation2.launch.py \
     use_sim_time:=true \
     map:=$HOME/my_turtlebot3_map.yaml
   ```

3. **RViz에서 확인**:
   - 맵이 보이는가?
   - 왼쪽 패널: "Localization: active", "Navigation: active"

4. **2D Pose Estimate**:
   - 상단 툴바에서 "2D Pose Estimate" 클릭
   - 로봇의 실제 위치 클릭 + 드래그로 방향 설정
   - 녹색 파티클(화살표)이 로봇 주변에 나타남

5. **Nav2 Goal**:
   - 상단 툴바에서 "Nav2 Goal" 클릭
   - 목표 위치 클릭 + 드래그로 도착 방향 설정
   - 로봇이 자율주행 시작!

6. **관찰**:
   - 녹색 경로 (Global Path): 전체 계획
   - 노란색 경로 (Local Path): 실시간 조정
   - 보라색 영역 (Costmap): 위험 지역
   - 녹색 화살표 (AMCL Particles): 위치 불확실성

---

## 📊 학습 성과 자가 평가

### Architecture & Design (아키텍처 설계 이해)

| 평가 항목 | 점수 (1-5) | 비고 |
|----------|-----------|------|
| Navigation2 전체 아키텍처 이해 | 5/5 | 4개 레이어 + 각 컴포넌트 역할 명확 |
| Global vs Local Planner 분리 이유 | 5/5 | 시공간 스케일 차이, 계산량 트레이드오프 이해 |
| Costmap의 계층적 설계 | 5/5 | 플러그인 아키텍처, OCP 원칙 이해 |
| TF Tree의 필요성 | 5/5 | 좌표계 분리(map/odom), 센서 융합 이해 |
| Lifecycle 노드 시스템 | 4/5 | 상태 머신은 이해, 실무 활용은 더 연습 필요 |
| Behavior Tree의 장점 | 4/5 | FSM 대비 장점 이해, 직접 작성은 아직 |

### Problem Solving (문제 해결 능력)

| 평가 항목 | 점수 (1-5) | 비고 |
|----------|-----------|------|
| 시스템 디버깅 방법론 | 5/5 | 로그 분석, 상태 확인, 단계적 접근 |
| Lifecycle 충돌 해결 | 5/5 | 상태 불일치 원인 파악 + Clean Restart |
| TF 에러 트러블슈팅 | 4/5 | 원인 파악, 해결은 했으나 TF 도구 활용은 더 연습 |
| 시스템 재시작 프로토콜 | 5/5 | 정확한 순서, 각 단계 검증 |

### Practical Skills (실무 기술)

| 평가 항목 | 점수 (1-5) | 비고 |
|----------|-----------|------|
| 저장된 맵으로 자율주행 | 5/5 | 성공적으로 구현 |
| 2D Pose Estimate 설정 | 5/5 | AMCL 초기화 이해 |
| Nav2 Goal 설정 | 5/5 | 로봇이 목표까지 자율주행 |
| RViz 관찰 및 해석 | 5/5 | Costmap, Path, Particles 의미 이해 |
| Launch 파일 책임 분리 | 4/5 | 개념은 이해, 직접 작성은 더 연습 |

### Design Thinking (설계 사고)

| 평가 항목 | 점수 (1-5) | 비고 |
|----------|-----------|------|
| 느슨한 결합 설계 | 5/5 | Topic 기반 통신의 장점 이해 |
| 관심사의 분리 | 5/5 | 각 노드의 단일 책임 이해 |
| 계층적 추상화 | 5/5 | 레이어별 책임, 추상화 수준 이해 |
| Fault Tolerance 설계 | 4/5 | Recovery 개념 이해, 직접 설계는 더 연습 |
| 트레이드오프 분석 | 5/5 | Global/Local 분리, DWA vs TEB 등 이해 |

**종합 평가**: 4.7/5 - **Excellent**

오늘은 ROS 2의 가장 복잡한 시스템 중 하나인 Navigation2를 성공적으로 이해하고 구현했습니다. 단순히 "작동시키기"를 넘어서, **왜 이렇게 설계되었는가**를 시스템 아키텍처 관점에서 깊이 이해했습니다. 특히 트러블슈팅 과정에서 Lifecycle 상태 충돌, TF 의존성 등 실무에서 매우 흔한 문제들을 직접 경험하고 해결한 것이 큰 성과입니다.

---

## 🚀 다음 단계

### Week 2 Preview: 매니퓰레이션과 MoveIt 2

Day 7까지 모바일 로봇(바퀴로 움직이는 로봇)을 다뤘습니다. Week 2에서는 **로봇 팔(Manipulator)**을 배웁니다.

**예고**:
- **MoveIt 2**: 로봇 팔 모션 계획
- **역기구학(Inverse Kinematics)**: "손을 (x, y, z)에 놓으려면 관절을 몇 도 움직여야 해?"
- **충돌 회피**: 로봇 팔이 자기 몸이나 환경과 부딪히지 않게
- **Pick & Place**: 물체 잡기 + 옮기기

**아키텍처 관점**: Navigation2와 MoveIt 2는 놀랍도록 유사합니다!
- Navigation2: "로봇을 (x, y, θ)로 이동"
- MoveIt 2: "로봇 팔을 (x, y, z)로 이동"

둘 다 경로 계획 + 충돌 회피 + 장애물 인식이 필요합니다. 설계 원칙도 동일: 계층적 추상화, 느슨한 결합, Lifecycle 관리.

---

### 복습 추천 사항

Day 7을 완전히 내 것으로 만들려면:

1. **RViz 관찰 심화**: 로봇이 움직이는 동안 Costmap, Path, Particles의 변화를 천천히 관찰
2. **파라미터 실험**:
   - `max_vel_x`를 0.1 → 0.5로 변경하면?
   - Inflation Radius를 0.2 → 0.5로 변경하면?
3. **Recovery 테스트**: 로봇 앞에 가상 장애물을 놓고, Recovery Behavior 관찰
4. **다른 맵 테스트**: Day 6에서 만든 다른 맵이 있다면 로드해보기

---

## 🎉 Day 7 완료!

**오늘의 여정**:
- ✅ Navigation2의 복잡한 아키텍처를 4시간 만에 이해
- ✅ AMCL, Costmap, Planner, Controller, BT의 협업 방식 파악
- ✅ 3가지 주요 문제를 실무 수준으로 해결
- ✅ 로봇이 자율주행으로 목표까지 이동!

**가장 중요한 학습**:
"복잡한 시스템은 작은 컴포넌트의 조합이다. 각 컴포넌트는 단순하지만, 올바른 인터페이스로 연결되면 놀라운 행동을 만들어낸다."

이것이 바로 **시스템 아키텍트의 사고방식**입니다. 🚀

---

**다음 학습**: Week 2, Day 1 - MoveIt 2 & 매니퓰레이션 기초

> "The whole is greater than the sum of its parts." - Aristotle
> (전체는 부분의 합보다 크다)

Navigation2가 바로 이 명언의 완벽한 예시입니다. 🤖
