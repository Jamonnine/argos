# Day 13: ROS 2 Actions — 장기 작업의 통신 패턴

**날짜**: 2026-02-18
**학습 주제**: ROS 2 Action Server/Client — Topic·Service로 할 수 없는 것
**소요 시간**: 약 2시간

---

## 📋 학습 목표 및 성과

### 목표
- [x] ROS 2 세 가지 통신 방식 완성 (Topic ✅ Service ✅ Action ✅)
- [x] 커스텀 Action 타입 정의 (.action 파일)
- [x] Action Server 구현 (arm_controller_sim.py 변환)
- [x] Action Client 구현 (vision_moveit_bridge.py 변환)
- [x] 전체 Goal → Feedback → Result 사이클 동작 확인

### 성과
- ✅ `MoveToTarget.action` 타입 정의 및 빌드 (my_robot_interfaces 패키지)
- ✅ Action Server: PLANNING → MOVING → REACHED 사이클 + Feedback 발행
- ✅ Action Client: send_goal_async + on_feedback + on_result 콜백 체계
- ✅ `SUCCESS: Successfully reached target pose` Result 수신 확인

---

## 🎯 핵심 개념

### 1. 세 가지 통신 방식 완전 정리

| | Topic | Service | Action |
|--|--|--|--|
| 방향 | 단방향 | 양방향 | 양방향 |
| 패턴 | 지속 발행 | 요청-응답 | 목표-피드백-결과 |
| 대기 여부 | 없음 | 완료까지 대기 | 비동기 |
| 취소 가능 | 불가 | 불가 | 가능 |
| 용도 | 센서 스트림 | 설정 변경 | 장기 작업 |
| ROS 2 예시 | /camera/points | /set_parameters | /move_to_target |

"장기 작업 + 진행 상황 보고 + 취소 가능" 세 가지가 동시에 필요할 때 Action을 사용합니다.

---

### 2. Action = Topic + Service의 조합

Action은 독립 프로토콜이 아니라 내부적으로 다음 세 채널의 조합입니다:
- Goal 전송: Service (요청-응답)
- Feedback: Topic (단방향 스트림)
- Cancel: Service (요청-응답)
- Result: Service (요청-응답)

이것이 **Facade 패턴**: 복잡한 내부를 단순한 인터페이스로 감쌈.

---

### 3. 커스텀 인터페이스 패키지 분리 이유

`ament_python` 패키지(순수 Python)는 커스텀 msg/action을 직접 정의할 수 없습니다.
별도 `ament_cmake` 패키지를 만들어야 합니다. 이것이 ROS 2 표준 패턴입니다.

```
my_robot_interfaces/  ← CMake 패키지 (인터페이스 정의 전용)
    action/MoveToTarget.action

my_robot_bringup/     ← Python 패키지 (실제 노드)
    → my_robot_interfaces 의존
```

**장점**: 인터페이스 정의와 구현이 분리되어, 다른 패키지에서도 재사용 가능.

---

### 4. MultiThreadedExecutor 필요 이유

Action Server의 `execute_goal`은 long-running 함수입니다 (3~4초 동안 실행 중).
기본 SingleThreadedExecutor로는 실행 중에 cancel 요청이 들어와도 처리 불가합니다.

```python
executor = rclpy.executors.MultiThreadedExecutor()
```

여러 콜백이 동시에 실행 가능하므로, execute_goal 실행 중에도 cancel_callback이 동작합니다.

---

### 5. 비동기(Async) 패턴

```python
# 동기 방식 (잘못된 예): 결과 나올 때까지 전체 노드 블로킹
result = action_client.send_goal(goal)  # 4초 동안 멈춤

# 비동기 방식 (올바른 예): 즉시 반환, 완료 시 콜백 호출
future = action_client.send_goal_async(goal, feedback_callback=on_feedback)
future.add_done_callback(on_goal_response)
```

비동기 방식을 써야 노드가 goal 대기 중에도 다른 토픽을 구독하고 처리할 수 있습니다.

---

## 🏗️ 생성된 파일

```
ros2_ws/src/my_robot_interfaces/           (신규 패키지)
├── CMakeLists.txt
├── package.xml
└── action/MoveToTarget.action

ros2_ws/src/my_robot_bringup/scripts/
├── arm_controller_sim.py    (수정 - Action Server로 변환)
└── vision_moveit_bridge.py  (수정 - Action Client로 변환)
```

---

## ✅ 동작 확인 로그

```
vision_moveit_bridge → Sending goal: (-0.52, 0.04, 2.37)
arm_controller_sim   → Goal received — accepted
vision_moveit_bridge → Goal accepted — waiting for result...
arm_controller_sim   → Executing goal: (-0.52, 0.04, 2.37)
vision_moveit_bridge → Feedback: [MOVING] 3%  | dist: 1.04m
vision_moveit_bridge → Feedback: [MOVING] 37% | dist: 0.72m
vision_moveit_bridge → Feedback: [MOVING] 71% | dist: 0.21m
arm_controller_sim   → Target REACHED!
vision_moveit_bridge → SUCCESS: Successfully reached target pose
```

---

## 🚀 다음 단계

### Day 14: Gazebo 시뮬레이션 환경 구축

- 물리 시뮬레이터 Gazebo에서 실제 로봇 모델(URDF) 실행
- TurtleBot3 또는 커스텀 모바일 로봇으로 실제 Navigation 연결
- Nav2(Navigation2) 스택 구조 이해

---

## 🎉 Day 13 완료!

오늘의 핵심 메시지:

"Action은 '전화 통화'입니다. 처음에 요청을 하면(Goal), 상대방이 진행 상황을 중간중간 알려주고(Feedback), 마지막에 완료 여부를 알려줍니다(Result). 그리고 원하면 언제든 끊을 수 있습니다(Cancel). Topic이 '방송', Service가 '문자', Action이 '전화'라면, 로봇 팔이 물체를 집는 것처럼 시간이 걸리는 작업은 당연히 전화가 맞습니다."
