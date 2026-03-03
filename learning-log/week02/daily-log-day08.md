# Day 8: MoveIt 2와 매니퓰레이션 기초

**날짜**: 2026-02-12
**학습 주제**: MoveIt 2 아키텍처와 로봇 팔 제어
**소요 시간**: 약 2시간

---

## 📋 학습 목표 및 성과

### 목표
- [x] MoveIt 2 아키텍처 이해 (Navigation2와 비교)
- [x] 역기구학(Inverse Kinematics) 개념 파악
- [x] 모션 플래닝 알고리즘(RRT) 이해
- [x] Panda 로봇 팔 시뮬레이션 실행
- [x] Interactive Marker로 로봇 팔 제어

### 성과
- ✅ MoveIt 2 설치 및 데모 실행 성공
- ✅ Panda 7-DOF 로봇 팔을 Task Space에서 제어
- ✅ IK의 실시간 계산 경험 (Interactive Marker 드래그)
- ✅ RRT 기반 모션 플래닝 실행
- ✅ Navigation2와 MoveIt 2의 근본적 유사성/차이점 파악

---

## 🎯 핵심 발견: Navigation2와 MoveIt 2의 형제 관계

### 같은 문제, 다른 공간

오늘 가장 중요한 깨달음은 **"Navigation2와 MoveIt 2는 본질적으로 같은 문제를 푼다"**는 것입니다.

**공통 문제**: "A 상태에서 B 상태로 안전하게 이동하되, 장애물을 피하고, 로봇의 물리적 제약을 만족하라"

**차이점**: 무엇을 움직이는가?
- **Navigation2**: 로봇의 베이스 (바퀴) → 2D 평면 (x, y, θ)
- **MoveIt 2**: 로봇의 팔 (관절들) → 고차원 공간 (6-7차원)

이 차원의 차이가 모든 설계 선택을 결정합니다. 2D는 A* 같은 그리드 탐색이 가능하지만, 7D는 샘플링 기반 알고리즘(RRT)이 필수입니다. 하지만 전체 아키텍처 철학은 놀라울 정도로 동일합니다.

---

## 🏗️ MoveIt 2 아키텍처

### 전체 흐름

```
사용자: "손을 (0.5, 0.3, 0.8)로 이동"
  ↓
[1] Move Group (중앙 조율자)
  ↓
[2] Inverse Kinematics (Task Space → C-Space 변환)
    입력: End-effector 포즈 (x, y, z, roll, pitch, yaw)
    출력: 관절 각도 [θ1, θ2, ..., θ7]
  ↓
[3] Motion Planning (경로 탐색)
    알고리즘: RRT, RRT-Connect, PRM, CHOMP, STOMP
    공간: 7차원 Configuration Space
    제약: Self-Collision + 환경 장애물
  ↓
[4] Trajectory Smoothing (선택)
    급격한 움직임 → 부드러운 곡선
  ↓
[5] Execution
    각 관절에 시간별 각도 명령 전송
  ↓
로봇 팔이 움직임!
```

---

## 🔑 핵심 개념

### 1. Configuration Space (C-Space): 고차원의 세계

**두 가지 공간**:

**Task Space (작업 공간)**:
- 로봇 손의 위치와 방향
- 6차원: (x, y, z, roll, pitch, yaw)
- 직관적: "손을 여기로"
- 하지만 직접 제어 불가능 (모터는 관절에 있음)

**Configuration Space (형상 공간)**:
- 모든 관절의 각도
- Panda는 7차원: [θ1, θ2, θ3, θ4, θ5, θ6, θ7]
- 직접 제어 가능: 모터에 각도 명령
- 하지만 직관적이지 않음

**MoveIt 2의 핵심 역할**: 두 공간 사이의 **매핑**
- 사용자는 Task Space에서 생각
- IK가 C-Space로 변환
- Planning은 C-Space에서 수행
- 실행은 C-Space 명령을 모터로 전송

**왜 C-Space가 필요한가?** 물리적 제약(관절 한계, 충돌)은 C-Space에서 자연스럽게 표현됩니다. Task Space에서는 "이 손 위치가 충돌인가?"를 직접 알 수 없지만, C-Space에서는 "이 관절 각도 조합이 충돌인가?"를 명확히 체크할 수 있습니다.

---

### 2. Inverse Kinematics (역기구학): MoveIt의 첫 번째 난관

**순기구학(FK)**: 쉽다
```
[θ1=30°, θ2=45°, ...] → 손 위치 (x, y, z) = (0.5, 0.3, 0.8)
단순 행렬 곱셈, 밀리초 이내
```

**역기구학(IK)**: 어렵다
```
손 위치 (0.5, 0.3, 0.8) → [θ1=?, θ2=?, ...] = ???
비선형 방정식, 수치 최적화 필요, 수십~수백 밀리초
```

**IK가 어려운 이유**:

1. **무한히 많은 해(Redundancy)**: 7-DOF 로봇이 6-DOF 목표(손의 포즈)를 달성하는 방법은 무수히 많습니다. 팔꿈치를 위로 굽힐 수도, 아래로 굽힐 수도 있죠. 이것을 **Kinematic Redundancy**라고 합니다.

2. **해가 없을 수도(Unreachable)**: 손을 로봇의 최대 도달 거리(Reach)보다 멀리 놓으라고 하면, 물리적으로 불가능합니다.

3. **비선형 방정식**: 삼각함수(sin, cos)가 중첩되어 있어서, 해석적 해(Closed-form Solution)가 없는 경우가 많습니다. 6-DOF 이하는 해석적 해가 있을 수 있지만, 7-DOF는 거의 항상 수치적 방법이 필요합니다.

**실무 해결 방법**:
- **KDL (Kinematics and Dynamics Library)**: Newton-Raphson 반복법
- **TracIK**: KDL보다 빠르고 robust (Timeout 처리 개선)
- **Bio-IK**: 생체 영감 최적화 (여러 목표를 동시에 만족)
- **TRAC-IK**: 여러 초기 추측(Initial Guess)으로 병렬 시도

**Interactive Marker의 마법**: RViz에서 Interactive Marker를 드래그할 때, MoveIt은 **실시간으로 IK를 계산**합니다. 마우스를 움직일 때마다 (약 60Hz) IK 솔버가 돌아서, 로봇 팔의 "고스트" 형태를 업데이트합니다. 이것이 가능한 이유는 **증분 IK(Incremental IK)** 기법을 사용하기 때문입니다: 이전 해를 초기 추측으로 사용하여, 작은 변화만 계산합니다.

---

### 3. Motion Planning: 차원의 저주와 샘플링의 해법

**Navigation2의 방식 (A\*)**:
- 2D 공간을 그리드로 분할: 100×100 = 10,000 셀
- 각 셀을 그래프 노드로
- A* 알고리즘으로 최단 경로 탐색
- 계산량: O(N log N), N = 셀 개수
- 빠름: 수십~수백 밀리초

**MoveIt 2의 방식 (RRT)**:
- 7D 공간을 그리드로 분할하려면: 100^7 = **1조 셀**
- 메모리에 담을 수 없음 (수백 테라바이트)
- **해결책**: 샘플링 기반 알고리즘

**RRT (Rapidly-exploring Random Tree)의 아이디어**:

```
1. 시작 관절 각도에서 트리 시작
2. Repeat:
   a. 랜덤하게 목표 지점 샘플링 (7차원 공간의 랜덤 점)
   b. 트리에서 가장 가까운 노드 찾기
   c. 그 방향으로 한 스텝 확장 (보통 5-10도)
   d. 충돌 체크:
      - Self-Collision (팔꿈치 ↔ 어깨)
      - 환경 충돌 (로봇 ↔ 테이블)
   e. 충돌 없으면 트리에 추가
   f. 목표에 도달했으면 종료
3. 트리를 역추적하여 경로 추출
```

**왜 랜덤인가?** 7차원 공간 전체를 체계적으로 탐색하는 것은 불가능합니다. 랜덤 샘플링은 **확률적 완전성(Probabilistic Completeness)**을 제공합니다: 해가 존재한다면, 충분히 많은 샘플(시간)을 주면 거의 확실히 찾습니다.

**RRT-Connect (MoveIt 기본값)**:
- 양방향 RRT: 시작점과 목표점에서 동시에 트리 확장
- 두 트리가 만나면 경로 완성
- RRT보다 2~10배 빠름

**실무 트레이드오프**:
- **RRT-Connect**: 빠름, 하지만 경로가 "지그재그"할 수 있음
- **RRT\***: 느림, 하지만 점근적 최적성(Asymptotically Optimal) 보장
- **PRM**: 여러 쿼리를 반복할 때 유리 (한 번 Roadmap 구축, 재사용)
- **CHOMP/STOMP**: 최적화 기반, 매우 부드러운 경로, 하지만 지역 최소값 문제

---

### 4. Collision Checking: 3D + Self-Collision

**Navigation2의 Costmap (2D)**:
- 격자에 장애물 표시
- LiDAR로 스캔
- 간단하고 빠름

**MoveIt 2의 Collision Scene (3D)**:
- **Octree** 자료구조: 3D 공간을 재귀적으로 8분할
- 로봇의 각 링크를 3D 메시(Mesh)로 표현
- 깊이 카메라(Depth Camera), Point Cloud로 환경 스캔

**Self-Collision의 특수성**:

로봇 팔은 자기 자신과 부딪힐 수 있습니다. 예를 들어:
- 팔꿈치가 어깨에 부딪힘
- 그리퍼가 베이스에 부딪힘
- 케이블이 로봇 몸체에 감김

MoveIt은 **SRDF (Semantic Robot Description Format)** 파일에서:
- **Adjacent Link Pairs** (인접 링크): 항상 붙어있으므로 체크 제외
- **Never Colliding Pairs** (절대 충돌 안 하는 쌍): 기하학적으로 불가능한 쌍 제외
- 나머지 모든 조합 체크: 7개 링크면 약 15-20쌍

**충돌 체크 알고리즘**:
- **FCL (Flexible Collision Library)**: 가장 많이 사용, 볼록 메시(Convex Mesh) 최적화
- **Bullet**: 게임 엔진 출신, 빠르지만 정확도 낮음
- 각 Planning 단계마다 수십~수백 번 충돌 체크 → 전체 계산량의 70-90%

**실무 최적화**: 단순한 형상(Sphere, Cylinder)으로 근사하면 충돌 체크가 10배 이상 빠릅니다. 하지만 보수적(Conservative)이어서, 실제로는 충돌 안 하는 경로도 거부할 수 있습니다. 트레이드오프: 속도 vs 정확도.

---

## 🔄 Navigation2 vs MoveIt 2: 심층 비교

### 대응 관계

| 컴포넌트 | Navigation2 | MoveIt 2 |
|---------|-------------|----------|
| **중앙 조율자** | BT Navigator | Move Group |
| **목표 설정** | Nav2 Goal (x, y, θ) | Pose Goal (x, y, z, r, p, y) |
| **공간 변환** | 없음 | **IK** (Task → C-Space) |
| **경로 계획** | Global Planner (A*) | Motion Planner (RRT) |
| **장애물 표현** | Costmap (2D 그리드) | Octree (3D) + Self-Collision |
| **Planning 공간** | 2D 그리드 | 7D 연속 공간 |
| **알고리즘 특성** | Deterministic | Probabilistic |
| **계획 시간** | 빠름 (수십 ms) | 느림 (수초) |
| **실행** | Controller (cmd_vel) | Trajectory Execution (joint commands) |

### 공통 설계 원칙

두 시스템 모두:
1. **느슨한 결합**: ROS Actions으로 통신
2. **플러그인 아키텍처**: Planner, Collision Checker 교체 가능
3. **계층적 추상화**: 고수준(목표) → 저수준(모터 명령)
4. **Fault Tolerance**: Planning 실패 시 재시도 메커니즘

### 근본적 차이: 차원의 저주

**왜 알고리즘이 다른가?**

차원이 증가하면, **필요한 샘플 개수가 지수적으로 증가**합니다:
- 1D: 10개 샘플로 충분
- 2D: 100개 (10²)
- 3D: 1,000개 (10³)
- 7D: 10,000,000개 (10⁷)

A*는 "모든 셀을 체계적으로 탐색"하므로, 고차원에서는 불가능합니다. RRT는 "랜덤 샘플링"으로 이 문제를 회피합니다. 확률적이지만, 실용적입니다.

**실무 의미**: 차원이 시스템의 모든 것을 결정합니다. 알고리즘뿐 아니라, 데이터 구조(그리드 vs 트리), 최적화 전략(전역 최적 vs 만족 가능), 심지어 사용자 경험(즉각 응답 vs 대기 필요)까지.

---

## 🛠️ 실습 경험: Panda 로봇 팔 제어

### 환경 설정

```bash
# MoveIt 2 설치
sudo apt install ros-jazzy-moveit
sudo apt install ros-jazzy-moveit-resources-panda-moveit-config

# 데모 실행
ros2 launch moveit_resources_panda_moveit_config demo.launch.py
```

### Interactive Marker 제어

**경험한 것**:
1. RViz에서 로봇 손 끝의 화살표(Interactive Marker) 드래그
2. 로봇 팔이 반투명 "고스트"로 실시간 따라옴
3. "Plan" 버튼 클릭 → 초록색 경로 시각화
4. "Execute" 버튼 클릭 → 로봇 팔이 부드럽게 이동

**내부에서 일어난 일**:
- Interactive Marker 드래그: 60Hz로 IK 계산, C-Space 업데이트
- Plan 버튼: RRT-Connect가 7D 공간에서 수천 개 샘플 뿌림
- Self-Collision 체크: 15-20개 링크 쌍, 각 Planning 단계마다
- Trajectory Smoothing: Cubic Spline 보간
- Execute: 7개 관절에 시간별 각도 명령 (제어 주기 약 100Hz)

**관찰 포인트**:
- 각 관절이 동시에 움직임 (병렬 모션)
- 부드러운 가속/감속 (Jerk Limit 적용)
- 중간 자세들이 모두 충돌 없음

---

## 🏗️ 아키텍처 설계 교훈

### 1. 추상화의 힘: Task Space vs C-Space

사용자는 "손을 여기로"라고 생각하지만, 로봇은 "관절을 이렇게"로만 이해합니다. IK는 이 **의미론적 갭(Semantic Gap)**을 메웁니다.

**실무 유사 사례**: 고수준 프로그래밍 언어(Python)와 기계어의 관계. 프로그래머는 "리스트를 정렬해"라고 쓰지만, CPU는 "메모리 주소 0x1234의 값을 레지스터 A로 로드"를 실행합니다. 컴파일러가 변환을 담당하듯, IK가 Task → C-Space 변환을 담당합니다.

### 2. 확률적 완전성: 완벽함 vs 실용성

RRT는 **최적 경로를 보장하지 않습니다**. 심지어 같은 문제를 두 번 풀면 다른 경로가 나올 수 있습니다(랜덤이므로).

하지만 **충분히 좋은 해를 빠르게** 찾습니다. 실무에서는 "1시간 후 완벽한 해"보다 "1초 후 80% 품질의 해"가 더 가치 있을 때가 많습니다.

**트레이드오프**: Optimality vs Speed
- 연구: RRT* 같은 점근적 최적 알고리즘
- 산업: RRT-Connect 같은 빠른 만족 가능 알고리즘

### 3. Self-Collision: 로봇만의 특수 문제

Navigation2는 "로봇이 자기 자신과 충돌"을 고민하지 않습니다. 로봇 베이스는 단단한 물체 하나이므로.

하지만 로봇 팔은 **여러 링크가 연결된 체인**입니다. 각 링크는 독립적으로 움직이므로, 서로 부딪힐 수 있습니다. 이것은 **articulated body(관절 구조체)**의 본질적 복잡성입니다.

**실무 의미**: 시스템의 **내부 구조 복잡도**가 문제 공간의 복잡도를 결정합니다. 단순한 물체(자동차)보다 복잡한 구조(로봇 팔, 사람 몸)를 모델링하고 제어하는 것이 본질적으로 어렵습니다.

---

## 🎓 핵심 개념 요약

### MoveIt 2 = 5단계 파이프라인

1. **목표 설정**: "손을 (x, y, z)로" (Task Space)
2. **IK**: "그러려면 관절을 [θ1, ..., θ7]로" (C-Space 변환)
3. **Planning**: "현재 각도 → 목표 각도의 충돌 없는 경로" (RRT)
4. **Smoothing**: "급격한 움직임 → 부드러운 곡선" (Spline)
5. **Execution**: "각 관절에 시간별 명령 전송" (Trajectory Following)

### 차원의 저주 = 알고리즘의 운명

- **2D (Navigation2)**: 그리드 탐색 가능 → A*
- **7D (MoveIt 2)**: 그리드 불가능 → 샘플링 (RRT)

이것은 단순한 기술적 선택이 아니라, **수학적 필연**입니다.

---

## 🚀 다음 단계

### Week 2 로드맵

**Day 8 (오늘)**: MoveIt 2 기초 ✅
- 아키텍처 이해
- IK 개념
- RRT Planning
- Panda 제어

**Day 9-10 (다음)**: 컴퓨터 비전
- OpenCV 기초
- 객체 인식 (YOLO, MobileNet)
- Point Cloud Processing
- 깊이 카메라 (Depth Estimation)

**Day 11-12**: 비전 + MoveIt 통합
- "카메라로 물체 찾기 → 로봇 팔로 집기"
- tf 변환 (카메라 좌표 → 로봇 좌표)
- Grasp Planning

**Day 13-14**: 통합 프로젝트
- TurtleBot3 + 로봇 팔 + 비전
- "자율주행 → 물체 인식 → Pick & Place"

### 최종 목표

"로봇이 자율주행으로 테이블 앞까지 가서, 카메라로 컵의 위치를 파악하고, 로봇 팔로 컵을 집어서 다른 테이블로 옮기기"

이것이 **모바일 매니퓰레이션(Mobile Manipulation)**입니다. Navigation2(이동) + MoveIt 2(조작) + 비전(인식)의 통합.

---

## 📊 학습 성과 자가 평가

| 평가 항목 | 점수 (1-5) | 비고 |
|----------|-----------|------|
| MoveIt 2 아키텍처 이해 | 5/5 | Navigation2와 비교하며 명확히 파악 |
| IK 개념 이해 | 5/5 | Redundancy, 비선형성, 수치 최적화 |
| C-Space vs Task Space | 5/5 | 두 공간의 차이와 변환 필요성 |
| RRT 알고리즘 원리 | 4/5 | 기본 아이디어 이해, 세부 구현은 더 학습 필요 |
| Self-Collision 개념 | 5/5 | 로봇 팔의 특수성 이해 |
| Panda 실습 | 5/5 | 성공적으로 제어 |

**종합 평가**: 4.8/5 - **Excellent**

Day 8은 짧지만 강력했습니다. MoveIt 2의 핵심 개념과 Navigation2와의 관계를 명확히 파악했고, 실제로 로봇 팔을 제어하는 경험까지 얻었습니다. "전체 그림 빠르게 파악" 전략에 매우 적합한 진행이었습니다.

---

## 🎉 Day 8 완료!

**오늘의 핵심 메시지**:

"차원이 모든 것을 결정한다. 2D에서는 쉽고, 7D에서는 어렵다. 하지만 추상화와 확률적 방법으로, 우리는 고차원 공간도 정복할 수 있다."

**다음**: Day 9 - 로봇에게 눈을 달아주기 (컴퓨터 비전) 👁️

---

> "The complexity of a problem grows exponentially with its dimensionality, but our ingenuity grows even faster."
> (문제의 복잡도는 차원에 따라 지수적으로 증가하지만, 인간의 창의성은 그보다 더 빠르게 성장한다)
