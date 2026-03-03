# Daily Learning Log - Week 1, Day 4

**Date**: 2026-02-11
**Phase**: Phase 1 - ROS 2 Fundamentals
**Focus**: Autonomous Control, State Machines, Closed-Loop Systems
**Hours Logged**: ~5 hours

---

## 📋 Daily Goals - 완료!

- [x] Turtlesim 자동 제어 시스템 구현
- [x] State Machine 패턴 학습 및 적용
- [x] Closed-Loop Control 구현
- [x] Point-to-Point Navigation 완성
- [x] 실무 디버깅 프로세스 체험 (Overshoot 문제)
- [x] Feedback Control 시스템 설계 원칙 이해

---

## 🎓 What I Learned Today

### 큰 그림: 제어 시스템의 본질

#### 로봇 제어의 핵심 질문

로봇 제어는 단순히 "명령을 보내는 것"이 아닙니다. 핵심은 **"목표 상태에 도달하고 유지하는 것"**입니다. 이를 위해서는:

1. **현재 상태를 알아야 합니다** (Sensing/Perception)
2. **목표 상태를 정의해야 합니다** (Goal)
3. **차이를 계산해야 합니다** (Error = Goal - Current)
4. **차이를 줄이는 명령을 계산해야 합니다** (Control)
5. **명령을 실행하고 다시 1번부터 반복** (Feedback Loop)

이 5단계 사이클이 **Closed-Loop Control**의 핵심이며, 모든 로봇 시스템의 기반입니다.

---

### Open-Loop vs Closed-Loop: 실패를 통한 학습

#### Open-Loop의 한계

**Open-Loop Control**:
```
목표 설정 → 명령 계산 → 명령 실행 → 끝
           (피드백 없음)
```

**오늘의 첫 시도**:
1. 거북이를 목표 방향으로 회전
2. 직진 명령 전송
3. 일정 시간 후 정지

**결과**: 벽 충돌! 🐢💥

**왜 실패했나?**
- 회전 완료 시 -3.1° 오차가 남음
- 직진하면서 각도 보정 없음
- 목표를 비스듬히 지나침
- 계속 직진 → 벽에 도달

**교훈**: 실세계는 완벽하지 않습니다
- 모터는 정확히 멈추지 못함
- 바닥 마찰이 균일하지 않음
- 센서 노이즈 존재
- 외부 교란 (사람이 로봇을 밀 수도 있음)

Open-Loop는 이러한 불확실성을 **보정할 방법이 없습니다**.

#### Closed-Loop의 우아함

**Closed-Loop Control**:
```
┌─────────────────────────────────────┐
│ 목표 설정 → 명령 계산 → 명령 실행 │
│      ↑                        │     │
│      └────── 센서 피드백 ──────┘     │
└─────────────────────────────────────┘
```

**최종 구현**:
1. 목표 방향으로 회전
2. 직진하면서 **지속적으로**:
   - 현재 위치 확인
   - 목표까지 거리 계산
   - 각도 오차 계산
   - **실시간 보정** (각도가 틀어지면 약간 회전)
   - Overshoot 감지 (거리가 증가하면 즉시 정지)

**결과**: 정확히 목표 도달! 🎯

**왜 성공했나?**
- 피드백이 오차를 지속적으로 보정
- Overshoot을 감지하고 대응
- 외부 교란에도 강건함

---

### State Machine: 복잡한 행동의 구조화

#### 왜 State Machine이 필요한가?

로봇의 행동은 본질적으로 **순차적이고 조건부**입니다:

```
"목표로 이동"이라는 단순한 작업도:
1. 먼저 목표 방향으로 회전해야 하고
2. 회전이 완료되면 직진하고
3. 목표에 도착하면 정지해야 합니다
```

이를 if-else로 구현하면:
```python
def control_loop(self):
    if not self.is_aligned():
        rotate()
    elif not self.is_arrived():
        if self.overshoot_detected():
            stop()
        else:
            move_forward()
            if angle_error_too_large():
                correct_angle()
    else:
        stop()
```

**문제점**:
- 중첩된 조건문으로 로직 파악 어려움
- 현재 "상태"가 명시적이지 않음
- 디버깅 시 "지금 뭐 하는 중인지" 알기 어려움
- 새로운 상태 추가 시 조건문 수정 필요

#### State Machine 패턴

```python
STATE_ROTATING = 0
STATE_MOVING = 1
STATE_ARRIVED = 2

def control_loop(self):
    if self.state == STATE_ROTATING:
        if rotation_complete():
            self.state = STATE_MOVING
        else:
            continue_rotating()

    elif self.state == STATE_MOVING:
        if overshoot_detected():
            self.state = STATE_ARRIVED
        elif goal_reached():
            self.state = STATE_ARRIVED
        else:
            move_with_correction()

    elif self.state == STATE_ARRIVED:
        stop_and_stay()
```

**장점**:
- **명확성**: 현재 상태가 명시적 (로그에서 "STATE_MOVING" 확인 가능)
- **확장성**: 새 상태 추가 쉬움 (예: STATE_OBSTACLE_AVOIDING)
- **디버깅**: 상태 전환을 로그로 추적 가능
- **테스트**: 각 상태를 독립적으로 테스트
- **문서화**: 상태 다이어그램으로 시각화 가능

#### 실무 State Machine 설계 원칙

**1. 상태는 명사, 전환은 동사**
- 상태: ROTATING, MOVING, ARRIVED (명사)
- 전환: "rotation_complete", "goal_reached" (동사/조건)

**2. 각 상태는 명확한 진입/종료 조건**
```python
# ROTATING 진입: 항상 시작 시
# ROTATING 종료: angle_error < threshold

# MOVING 진입: rotation_complete
# MOVING 종료: distance < threshold OR overshoot
```

**3. 모든 상태 전환을 로그로 기록**
```python
self.get_logger().info('✅ Rotation complete → MOVING')
```

**4. 예외 상태 고려**
실무에서는 `STATE_ERROR`, `STATE_EMERGENCY_STOP` 같은 예외 상태도 필요합니다.

---

### Overshoot 문제: 실전 디버깅 경험

#### 문제 발견 과정

**증상**:
```
[WARN] Oh no! I hit the wall! (Clamping from [x=11.104908, y=-0.017871])
```

목표는 (8, 8)인데 거북이가 벽(x=11+)까지 갔습니다.

#### 체계적 디버깅

**1단계: 데이터 수집**
```python
# 초기 상태 로그 추가
self.get_logger().info(
    f'Current: ({x:.2f}, {y:.2f}), theta={degrees(theta):.1f}°\n'
    f'Goal: ({goal_x:.2f}, {goal_y:.2f})\n'
    f'Distance: {distance:.2f}m\n'
    f'Angle error: {degrees(angle_error):.1f}°'
)
```

**2단계: 로그 분석**
```
🔄 ROTATING: angle_error=-3.1°  ← 회전 종료, 하지만 -3°오차
✅ Rotation complete → MOVING

➡️  MOVING: distance=2.95m       ← 목표로 접근 중
➡️  MOVING: distance=1.42m
➡️  MOVING: distance=0.59m
➡️  MOVING: distance=0.23m       ← 가장 가까웠던 순간
➡️  MOVING: distance=0.81m       ← 거리 증가 시작! (Overshoot)
➡️  MOVING: distance=1.97m
➡️  MOVING: distance=3.54m
➡️  MOVING: distance=4.37m
[WARN] I hit the wall!
```

**3단계: 근본 원인 파악**
- 회전 시 -3.1° 오차 누적
- MOVING 상태에서 각도 보정 없음 (Open-Loop 직진)
- 목표를 비스듬히 통과하면서 거리가 다시 멀어짐
- Overshoot 감지 메커니즘 부재

**4단계: 해결책 설계**

**Fix 1: Overshoot 감지**
```python
self.previous_distance = float('inf')

if distance > self.previous_distance + 0.05:  # 거리가 증가하면
    self.state = STATE_ARRIVED
    self.get_logger().warn('Overshoot detected! Stopping.')
```

**Fix 2: 이동 중 각도 보정 (Closed-Loop)**
```python
if abs(angle_error) > 0.2:  # ~11도 이상 오차
    cmd.angular.z = 0.5 * self.angular_speed  # 절반 속도로 보정
```

**5단계: 검증**
```
➡️  MOVING: distance=2.83m
➡️  MOVING: distance=1.39m
➡️  MOVING (correcting): angle_error=5.2°  ← 보정 작동!
➡️  MOVING: distance=0.57m
➡️  MOVING: distance=0.10m
🎯 ARRIVED at goal!  ← 성공!
```

#### 디버깅 경험에서 배운 것

**1. 로그는 최고의 디버깅 도구**
- 초기 상태, 중간 과정, 최종 결과를 모두 기록
- `throttle_duration_sec`로 로그 양 조절
- 핵심 변수(distance, angle_error) 집중 모니터링

**2. 가설 → 검증 → 수정 사이클**
- 가설: "회전 오차가 누적되어 Overshoot 발생"
- 검증: 로그에서 angle_error 추적
- 수정: 각도 보정 추가
- 재검증: 성공 확인

**3. Edge Case의 중요성**
- "정상적으로 도착"만 테스트하면 안 됨
- "Overshoot", "각도 오차 큼", "목표 근처에서 시작" 등 다양한 시나리오 필요

---

## 💻 오늘 작성한 코드

### Turtle Controller 최종 구조

```python
class TurtleController(Node):
    STATE_ROTATING = 0
    STATE_MOVING = 1
    STATE_ARRIVED = 2

    def __init__(self):
        super().__init__('turtle_controller')

        # Parameters
        self.declare_parameter('goal_x', 8.0)
        self.declare_parameter('goal_y', 8.0)
        self.declare_parameter('linear_speed', 1.5)
        self.declare_parameter('angular_speed', 1.5)
        self.declare_parameter('distance_tolerance', 0.1)
        self.declare_parameter('angle_tolerance', 0.05)

        # State
        self.state = self.STATE_ROTATING
        self.current_pose = None
        self.previous_distance = float('inf')  # Overshoot 감지용

        # Subscriber: Pose 피드백
        self.pose_subscriber = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10
        )

        # Publisher: 제어 명령
        self.cmd_publisher = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10
        )

        # Control Loop (10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

    def control_loop(self):
        if self.current_pose is None:
            return

        distance = self.get_distance_to_goal()
        angle_to_goal = self.get_angle_to_goal()
        angle_error = self.normalize_angle(
            angle_to_goal - self.current_pose.theta
        )

        cmd = Twist()

        if self.state == self.STATE_ROTATING:
            if abs(angle_error) > self.angle_tolerance:
                # 계속 회전
                cmd.angular.z = (
                    self.angular_speed if angle_error > 0
                    else -self.angular_speed
                )
            else:
                # 회전 완료 → MOVING
                self.state = self.STATE_MOVING
                self.get_logger().info('✅ Rotation complete → MOVING')

        elif self.state == self.STATE_MOVING:
            # Overshoot 감지
            if distance > self.previous_distance + 0.05:
                self.state = self.STATE_ARRIVED
                self.get_logger().warn('Overshoot detected!')

            elif distance > self.distance_tolerance:
                # 직진
                cmd.linear.x = self.linear_speed

                # 각도 보정 (Closed-Loop)
                if abs(angle_error) > 0.2:
                    cmd.angular.z = 0.5 * (
                        self.angular_speed if angle_error > 0
                        else -self.angular_speed
                    )

                self.previous_distance = distance

            else:
                # 목표 도달
                self.state = self.STATE_ARRIVED
                self.get_logger().info('🎯 ARRIVED at goal!')

        elif self.state == self.STATE_ARRIVED:
            # 정지
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_publisher.publish(cmd)

    def get_distance_to_goal(self):
        dx = self.goal_x - self.current_pose.x
        dy = self.goal_y - self.current_pose.y
        return math.sqrt(dx**2 + dy**2)

    def get_angle_to_goal(self):
        dx = self.goal_x - self.current_pose.x
        dy = self.goal_y - self.current_pose.y
        return math.atan2(dy, dx)

    def normalize_angle(self, angle):
        """[-π, π] 범위로 정규화"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
```

#### 설계 하이라이트

**1. Proportional Control**
```python
cmd.angular.z = self.angular_speed if angle_error > 0 else -self.angular_speed
```
오차의 방향에 비례하여 제어 출력. 실무에서는 PID(Proportional-Integral-Derivative) 컨트롤러를 사용하지만, 간단한 P 제어만으로도 충분히 작동합니다.

**2. 각도 정규화의 중요성**
```python
def normalize_angle(self, angle):
    while angle > math.pi:
        angle -= 2 * math.pi
```

각도는 주기적이므로 (360° = 0°), 정규화하지 않으면 359°에서 1°로 가는 것을 "-358° 회전"으로 잘못 계산할 수 있습니다.

**3. Hysteresis 부족 (개선 가능 지점)**
현재 구현은 `distance_tolerance` 한 개만 사용합니다. 실무에서는:
```python
if distance < 0.08:  # 진입 임계값
    arrived = True
elif distance > 0.15:  # 이탈 임계값
    arrived = False
```

이렇게 두 개의 임계값을 사용하여 "노이즈로 인한 상태 떨림"을 방지합니다.

---

## 💡 Key Insights

### Insight 1: 에러는 최고의 학습 도구

오늘의 Overshoot 문제는 좌절스러웠지만, 다음을 배웠습니다:
- Closed-Loop 제어의 필요성을 **직접 체감**
- 체계적 디버깅 프로세스 (로그 → 분석 → 가설 → 수정 → 검증)
- 실세계는 완벽하지 않다는 것을 코드에 반영해야 함

만약 첫 시도가 바로 성공했다면, 왜 Closed-Loop가 중요한지 깊이 이해하지 못했을 것입니다.

### Insight 2: State Machine은 사람의 사고방식

"거북이를 목표로 이동시켜"라는 작업을 사람에게 설명하면:
1. "먼저 그쪽을 봐"
2. "그 다음 직진해"
3. "도착하면 멈춰"

이것이 바로 State Machine입니다. 코드를 사람의 직관적 사고방식으로 구조화하면, 버그가 줄고 협업이 쉬워집니다.

### Insight 3: Parameters는 실험을 가능하게 함

오늘 여러 번 parameters를 조정했습니다:
- `angular_speed: 1.5` → 회전이 너무 빠름 → `1.2`
- `distance_tolerance: 0.2` → 너무 관대함 → `0.1`

코드 수정 없이 parameters만 바꾸면서 최적 값을 찾는 과정이 **튜닝**입니다. 실무에서는 이런 튜닝이 성능의 80%를 좌우합니다.

---

## 🎯 Progress Tracking

### Completed Today
- ✅ State Machine 패턴 구현 (ROTATING → MOVING → ARRIVED)
- ✅ Closed-Loop Control 시스템 구축
- ✅ Overshoot 문제 발견 및 해결
- ✅ 실시간 각도 보정 구현
- ✅ Point-to-Point Navigation 완성
- ✅ 실무 디버깅 프로세스 체험

### Key Achievements
- Open-Loop vs Closed-Loop의 차이를 **실패를 통해** 체감
- State Machine의 가치를 복잡한 행동 구현으로 확인
- 체계적 디버깅 능력 향상 (로그 기반 분석)

---

## 🚀 Week 1 회고

### 1주차 전체 성과

**Day 1**: ROS 2 기초 (Node, Topic, Publisher)
**Day 2**: Parameters, Launch Files (설정과 코드 분리)
**Day 3**: Multi-Node 시스템 (아키텍처 패턴)
**Day 4**: 자동 제어 (State Machine, Closed-Loop)

**4일 만에 달성**:
- 단일 노드 → 3-노드 시스템 → 자율 제어 시스템
- 코드 작성 → 설계 사고 → 시스템 아키텍처
- 튜토리얼 따라하기 → 문제 해결 → 실전 디버깅

### 실무 능력 평가

**아키텍처 설계**: ⭐⭐⭐⭐⭐
- Fan-Out, Pipeline, State Machine 패턴 이해
- 노드 책임 분리, 토픽 설계 능력

**문제 해결**: ⭐⭐⭐⭐☆
- 체계적 디버깅 (로그 → 분석 → 수정)
- 근본 원인 파악 능력

**코드 품질**: ⭐⭐⭐⭐☆
- 깔끔한 구조, 명확한 네이밍
- 개선 가능: 에러 처리, 테스트 코드

---

## 📝 Action Items for Week 2

### 계획된 주제
- [ ] 커스텀 메시지 타입 (TemperatureStatistics.msg)
- [ ] Services 시스템
- [ ] Actions (장시간 작업 + 피드백)
- [ ] Gazebo 시뮬레이션 시작

### 심화 학습 방향
- State Machine 확장 (장애물 회피 상태 추가?)
- PID 컨트롤러 구현
- 더 복잡한 Navigation 시나리오

---

## 📊 Self-Assessment

| Aspect | Score (1-5) | Notes |
|--------|-------------|-------|
| Control Theory Understanding | 5/5 | Open-Loop vs Closed-Loop 완전 이해 |
| State Machine Design | 5/5 | 실무 수준의 State Machine 구현 |
| Debugging Skills | 5/5 | Overshoot 문제를 체계적으로 해결 |
| Problem Solving | 5/5 | 실패 → 분석 → 해결의 완전한 사이클 |

**Overall Feeling**: 🎉 Week 1 완벽 마무리!

오늘은 단순히 거북이를 움직인 것이 아니라, **실세계 로봇 제어의 본질**을 배운 날입니다. 이론으로만 배웠다면 "Closed-Loop가 좋다"는 것을 머리로만 알았을 테지만, Overshoot으로 벽에 부딪힌 후에는 **몸으로 체감**했습니다.

이것이 프로젝트 기반 학습의 힘입니다.

---

**End of Day 4 Log**

**Next Session**: Week 1, Day 5 or Week 2, Day 1
**Next Goal**: 커스텀 메시지 & Services 시스템

---

**Tags**: #ROS2 #Week1 #Day4 #StateMachine #ClosedLoop #Control #Debugging #Overshoot #Turtlesim
