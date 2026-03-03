# Daily Learning Log - Week 1, Day 5

**Date**: 2026-02-11
**Phase**: Phase 1 → Phase 2 전환
**Focus**: Gazebo 3D 시뮬레이션, TurtleBot3, RViz2, 실전 환경 구축
**Hours Logged**: ~4 hours

---

## 📋 Daily Goals - 완료!

- [x] WSL2 Ubuntu 환경 접근 및 ROS 2 실행
- [x] Gazebo Harmonic 설치 및 검증
- [x] TurtleBot3 Burger 시뮬레이션 실행
- [x] RViz2 설정 및 Lidar 센서 시각화
- [x] TurtleBot3 World 탐색 (장애물 환경)
- [x] 실전 센서 데이터 분석 (LaserScan)
- [x] Agile 방법론 심층 학습

---

## 🎓 What I Learned Today

### 큰 그림: Turtlesim에서 실전 환경으로

#### 왜 Gazebo가 필요한가?

Day 1-4에서는 **Turtlesim**이라는 2D 교육용 도구로 ROS 2의 핵심 개념들을 배웠습니다. Publisher/Subscriber, Parameters, Launch Files, State Machine 등 모든 기본기를 익혔습니다. 하지만 Turtlesim은 말 그대로 "거북이 그림"을 움직이는 것일 뿐, 실제 로봇과는 거리가 멉니다.

**실제 로봇 개발에서 필요한 것들**:
- **3D 공간**: 실제 로봇은 평면이 아닌 3차원에서 움직입니다
- **물리 법칙**: 중력, 마찰, 관성, 충돌이 존재합니다
- **실제 센서**: Lidar, Camera, IMU 등의 센서가 실제처럼 작동해야 합니다
- **복잡한 환경**: 벽, 장애물, 좁은 공간 등 현실적인 상황

**Gazebo는 이 모든 것을 시뮬레이션**합니다. 더 중요한 것은, Gazebo에서 작성한 코드가 **실제 TurtleBot3 하드웨어에 그대로 배포**될 수 있다는 점입니다. 이것이 바로 **Simulation-to-Reality (Sim2Real)** 전환입니다.

---

### ROS-Gazebo Bridge: 두 세계를 연결하는 아키텍처

#### 왜 별도의 Bridge가 필요한가?

Gazebo와 ROS 2는 **완전히 다른 시스템**입니다:

**Gazebo의 세계**:
- 게임 엔진 기반 (Ogre3D 렌더러)
- 자체 메시지 형식: `gz.msgs.*`
- 물리 엔진: DART, Bullet
- 독립적인 시뮬레이터 (ROS 없이도 작동)

**ROS 2의 세계**:
- 로봇 미들웨어 (DDS 기반)
- 표준 메시지: `sensor_msgs`, `geometry_msgs`
- 하드웨어 추상화
- 실제 로봇과 시뮬레이터 모두 지원

**Bridge의 역할**:

```
[Gazebo Lidar Plugin]
    ↓
gz.msgs.LaserScan 발행
    ↓
[ros_gz_bridge] ← 실시간 번역!
    ↓
sensor_msgs/msg/LaserScan 발행
    ↓
[ROS 2 노드들]
```

이 Bridge 덕분에:
1. **ROS 2 노드는 Gazebo를 몰라도 됨**: `/scan` 토픽만 구독하면 됨
2. **실제 로봇과 코드 공유**: 같은 ROS 2 코드가 시뮬레이션과 실제 로봇에서 작동
3. **독립적 발전**: Gazebo와 ROS 2가 각각 업데이트되어도 Bridge만 업데이트하면 됨

**설계 원칙**: Adapter Pattern (서로 다른 인터페이스를 호환되게 만듦)

---

### RViz2: 로봇의 주관적 세계 시각화

#### Gazebo vs RViz2의 철학적 차이

이 구분은 **로봇 개발에서 가장 중요한 개념** 중 하나입니다:

**Gazebo - 객관적 진실 (Ground Truth)**:
- 물리 세계를 시뮬레이션
- "벽이 정확히 (5.0, 3.0)에 있다"
- 센서 없이도 모든 것을 알 수 있음
- 신의 시점 (God's View)

**RViz2 - 로봇의 인식 (Robot's Perception)**:
- ROS 토픽 데이터만 시각화
- "로봇이 센서로 측정한 결과: 벽이 (4.98, 3.02)에 있는 것 같다"
- 센서 없으면 아무것도 모름
- 로봇의 시점 (Robot's View)

**왜 이 구분이 중요한가?**

실제 로봇은 세계를 "있는 그대로" 볼 수 없습니다. 오직 **센서를 통해서만** 세계를 인식합니다. 센서에는 항상 노이즈, 오차, 한계가 있습니다. 따라서:

- Gazebo: 디버깅 시 "진짜 무슨 일이 일어났나?" 확인
- RViz2: "로봇은 무엇을 보고 있나? 왜 이렇게 행동했나?" 이해

**실무 예시**:

```
[상황]
로봇이 벽에 부딪힘

[Gazebo 확인]
"벽이 명확히 보임. 로봇이 회피 안 함."

[RViz2 확인]
"Lidar 데이터가 벽을 감지 못함! (센서 문제)"

→ 원인: 센서 필터링 알고리즘 버그
```

**아키텍처 교훈**: Observer Pattern (관찰자는 시스템에 영향을 주지 않음)

---

### Lidar 센서: 로봇의 눈

#### LaserScan 메시지 구조 심층 분석

오늘 처음으로 실제 센서 데이터를 다뤘습니다:

```yaml
header:
  stamp: {sec: 465, nanosec: 0}
  frame_id: base_scan
angle_min: 0.0              # 시작 각도 (0도)
angle_max: 6.28             # 끝 각도 (360도)
angle_increment: 0.0175     # 각도 간격 (~1도)
range_min: 0.12m            # 최소 측정 거리
range_max: 3.5m             # 최대 측정 거리
ranges: [.inf, .inf, 2.1, ...]  # 360개의 거리값
```

**360도 스캔의 의미**:

로봇은 매 0.1초마다 (10Hz):
1. 레이저를 360도 회전시키며 발사
2. 반사되어 돌아오는 시간으로 거리 계산
3. 360개의 측정값을 하나의 메시지로 발행

**왜 배열로 묶어서 발행하는가?**

만약 각 측정값을 개별 메시지로 보낸다면:
- 360개 메시지 × 10Hz = 초당 3600개 메시지!
- 시간 동기화 문제 (각 측정이 "다른 순간"이 됨)
- 통신 오버헤드 증가

**데이터 구조 설계 원칙**: "의미있는 단위"로 그룹화
- 360개 측정 = "한 번의 스캔" (의미론적 단위)
- 시간 일관성 보장 (모든 측정이 "같은 순간")
- 효율적 통신 (한 번의 메시지)

**실무 적용**:
- 자율주행차: Lidar + Radar + Camera 데이터를 "한 프레임"으로 묶음
- 드론: IMU + GPS + Barometer를 "한 상태"로 묶음

---

### Sense-Think-Act 사이클: 로봇 공학의 기본

#### 모든 로봇의 공통 구조

오늘 체험한 것은 **모든 지능형 로봇의 기본 사이클**입니다:

```
1. SENSE (감지)
   ↓
   센서가 환경 데이터 수집
   예: Lidar → /scan 토픽 발행
   ↓
2. THINK (판단)
   ↓
   알고리즘이 데이터 분석하여 의사결정
   예: "앞에 벽이 0.3m → 회전해야 함"
   ↓
3. ACT (행동)
   ↓
   제어 명령 실행
   예: /cmd_vel로 회전 명령 발행
   ↓
   (다시 SENSE로 돌아가 반복)
```

**실무 복잡도 증가**:

**간단한 로봇** (오늘 체험):
- SENSE: Lidar
- THINK: 거리 임계값 비교
- ACT: 회전 명령

**자율주행차**:
- SENSE: Lidar + Camera × 8 + Radar × 6 + GPS + IMU
- THINK: 딥러닝 객체 인식 + SLAM + 경로 계획 + 행동 예측
- ACT: 조향 + 가속 + 브레이크 + 신호등

**하지만 기본 구조는 동일**합니다!

**아키텍처 설계에서의 적용**:

ROS 2는 이 사이클을 **노드로 분리**하도록 장려합니다:

```
[Lidar Driver Node]
    ↓ /scan
[Obstacle Detector Node]
    ↓ /obstacle_alert
[Path Planner Node]
    ↓ /planned_path
[Controller Node]
    ↓ /cmd_vel
[Motor Driver Node]
```

**장점**:
- **Single Responsibility**: 각 노드는 한 가지만 잘 함
- **교체 가능성**: Lidar를 Radar로 교체해도 다른 노드는 변경 불필요
- **병렬 처리**: 각 노드가 독립적으로 다른 CPU 코어에서 실행
- **테스트 가능성**: 각 노드를 독립적으로 테스트

---

## 💻 오늘 작성한 코드

오늘은 코드를 작성하지 않았습니다. 대신 **시스템 구축**에 집중했습니다:

### 환경 설정 명령어들

```bash
# ROS 2 환경 활성화
source /opt/ros/jazzy/setup.bash

# TurtleBot3 모델 설정
export TURTLEBOT3_MODEL=burger

# Gazebo 실행
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# RViz2 실행
rviz2

# Teleop 제어
ros2 run turtlebot3_teleop teleop_keyboard
```

**설계 관점에서의 가치**:

코드를 작성하지 않았지만, **시스템 통합**을 경험했습니다:
- 여러 프로세스가 동시에 실행 (Gazebo, RViz2, Teleop)
- ROS 2 토픽으로 서로 통신
- Launch Files로 복잡한 시스템을 한 번에 시작

이것이 바로 **분산 시스템 아키텍처**입니다!

---

## 🐛 Errors & Solutions

### Error 1: ros2 명령어 실행 안 됨 (Git Bash 환경)

**Error Message**:
```bash
ros2: command not found
```

**Root Cause**:
- 현재 환경이 Git Bash (Windows)
- ROS 2는 Linux 전용
- WSL2 Ubuntu 환경이 필요

**Solution**:
```bash
# Git Bash에서 WSL 명령 실행
wsl -e bash -c "source /opt/ros/jazzy/setup.bash && ros2 ..."
```

**Lesson**:
- 환경 의존성의 중요성
- 로봇 개발 = Linux 필수
- WSL2는 "Windows에서 Linux 커널 실행"

---

### Error 2: gz-harmonic 패키지를 찾을 수 없음

**Error Message**:
```bash
E: Unable to locate package gz-harmonic
```

**Root Cause**:
- Gazebo는 별도의 APT 저장소 사용
- Ubuntu 기본 저장소에는 없음

**Solution**:
```bash
# ROS 2 통합 패키지 사용 (더 간단)
sudo apt install -y ros-jazzy-ros-gz
```

**Lesson**:
- 모듈화된 시스템의 저장소 관리
- Gazebo는 ROS와 독립적으로 발전
- 통합 패키지 (`ros-jazzy-ros-gz`)의 가치

**아키텍처 교훈**:
- 큰 시스템은 독립적인 컴포넌트로 구성
- 각 컴포넌트는 독립적으로 릴리스
- 하지만 통합 패키지로 사용자 편의성 제공

---

## 💡 Key Insights

### Insight 1: Simulation은 "거짓"이 아니라 "안전한 실험장"

많은 사람들이 "시뮬레이션은 가짜다"라고 생각합니다. 하지만 실무에서 시뮬레이션은:

**리스크 없는 실험**:
- 실제 로봇: 벽에 부딪히면 하드웨어 손상 (수백만 원)
- 시뮬레이션: 무한 충돌 가능, 비용 0원

**빠른 반복**:
- 실제 로봇: 충전, 설치, 테스트 → 1시간
- 시뮬레이션: 재시작 → 10초

**재현 가능성**:
- 실제 환경: "어제는 됐는데 오늘은 안 됨" (조명, 바닥 상태 변화)
- 시뮬레이션: 항상 동일한 조건

**극한 상황 테스트**:
- 실제: 위험한 시나리오 테스트 불가
- 시뮬레이션: 절벽 끝, 폭발, 극한 기후 모두 가능

**실무 워크플로우**:
1. 시뮬레이션에서 알고리즘 개발 (빠르고 안전)
2. 시뮬레이션에서 99% 완성
3. 실제 로봇에서 최종 검증 (1%)

---

### Insight 2: RViz2는 디버깅의 핵심 도구

**"왜 로봇이 이상하게 행동하지?"**

이 질문에 답하려면 **로봇이 무엇을 보고 있는지** 알아야 합니다. RViz2는 바로 그 창문입니다.

**디버깅 시나리오**:

```
[문제]
로봇이 장애물을 피하지 않고 충돌함

[Gazebo 확인]
장애물이 명확히 보임. 로봇이 회피를 안 함.

[RViz2 확인 - 여러 레이어]
1. /scan 토픽: Lidar 데이터 정상
2. /obstacle_alert: 장애물 감지 됨
3. /planned_path: 경로 계획 됨 (회피 경로)
4. /cmd_vel: 제어 명령 발행 안 됨! ← 문제 발견!

→ 원인: Controller 노드의 버그
```

**실무에서**:
- 시니어 개발자는 RViz2를 열자마자 문제 원인을 파악
- 주니어는 "어디서 문제인지" 모름
- **차이**: 어떤 토픽을 봐야 하는지 아는 경험

---

### Insight 3: Agile 방법론 - 작은 것부터 빠르게

**오늘 배운 Agile의 핵심**:

"완벽한 계획"보다 "작동하는 제품"이 중요합니다.

**우리의 학습 방식이 Agile입니다**:
- Day 1: Publisher 하나 → 작동 확인
- Day 2: Parameters 추가 → 작동 확인
- Day 3: Multi-Node → 작동 확인
- Day 4: State Machine → 작동 확인
- Day 5: Gazebo → 작동 확인

**매일 "작동하는 것"이 있었습니다!**

만약 Waterfall이었다면:
- Week 1-2: ROS 2 이론 공부만
- Week 3: 설계 문서 작성
- Week 4: 드디어 첫 코드...
- Week 5: "지루해서 포기"

**실무 적용**:
- 로봇 프로젝트: "6개월 후 완벽한 자율주행"보다 "매주 작동하는 버전"
- 스타트업: "완벽한 제품"보다 "MVP (최소 기능 제품) → 피드백 → 개선"

---

## 🎯 Progress Tracking

### Completed Today
- ✅ WSL2 환경 문제 해결 (Git Bash → WSL Ubuntu)
- ✅ Gazebo + ROS 2 통합 (ros-jazzy-ros-gz)
- ✅ TurtleBot3 World 실행 (복잡한 장애물 환경)
- ✅ RViz2 설정 (LaserScan, RobotModel 플러그인)
- ✅ 실시간 Lidar 센서 데이터 분석
- ✅ Sense-Think-Act 사이클 체험
- ✅ Agile 방법론 심층 이해

### Key Achievements
- **Turtlesim → Gazebo 전환 완료** (교육용 → 실전용)
- **센서 데이터 시각화** (RViz2 활용)
- **실전 환경 체험** (복잡한 월드, 장애물 회피)

---

## 📝 Action Items for Tomorrow (Day 6)

Day 5는 "Gazebo 환경 구축"에 집중했습니다. 다음은 **SLAM (맵 생성)**으로 넘어갑니다.

### 다음 학습 주제: SLAM (Simultaneous Localization and Mapping)

- [ ] SLAM이란 무엇인가? (개념 및 알고리즘)
- [ ] SLAM Toolbox 또는 Cartographer 설치
- [ ] TurtleBot3로 실시간 맵 생성
- [ ] 맵 저장 및 로드
- [ ] Localization (맵 상에서 로봇 위치 추정)

**목표**: 로봇이 스스로 지도를 만들고, 자신의 위치를 파악하는 능력

---

## 📊 Self-Assessment

| Aspect | Score (1-5) | Notes |
|--------|-------------|-------|
| Concept Understanding | 5/5 | ROS-Gazebo Bridge, RViz2 철학 완전 이해 |
| System Integration | 5/5 | 여러 프로세스 동시 실행 및 통합 |
| Problem Solving | 5/5 | 환경 문제 해결 (WSL2, 패키지 설치) |
| Sensor Data Analysis | 4/5 | LaserScan 구조 이해, 실시간 관찰 |
| Agile Mindset | 5/5 | 점진적 학습의 가치 체득 |

**Overall Feeling**: 😊 매우 만족

오늘은 **큰 도약**이었습니다. 교육용 도구에서 실전 환경으로 전환했고, 실제 로봇 개발자들이 사용하는 도구들(Gazebo, RViz2)을 마스터했습니다. 앞으로 배울 SLAM, Navigation은 모두 이 기반 위에서 작동합니다.

---

**End of Day 5 Log**

**Next Session**: Week 1/Week 2, Day 6
**First Task**: SLAM 개념 이해 및 첫 맵 생성
**Goal**: 로봇이 스스로 환경을 인식하고 지도를 만드는 능력 확보

---

**Tags**: #ROS2 #Week1 #Day5 #Gazebo #TurtleBot3 #RViz2 #SLAM-Preview #Agile
