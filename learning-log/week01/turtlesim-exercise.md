# Turtlesim 실습 가이드 - ROS 2 핵심 개념 체험

## 🎯 실습 목표
Turtlesim을 통해 ROS 2의 핵심 개념을 **손으로 직접 느끼며** 이해하기

---

## 📝 실습 1: Turtlesim 실행 및 노드 이해

### 터미널 1: Turtlesim 노드 실행
```bash
wsl -d Ubuntu
source /opt/ros/jazzy/setup.bash
ros2 run turtlesim turtlesim_node
```

**질문**: 창에 거북이가 보이나요? 🐢

---

### 터미널 2: 실행 중인 노드 확인
```bash
wsl -d Ubuntu
source /opt/ros/jazzy/setup.bash

# 현재 실행 중인 노드 확인
ros2 node list
```

**예상 출력**:
```
/turtlesim
```

**개념**: `/turtlesim`이 바로 **Node**입니다. Node는 ROS 2의 실행 단위입니다.

---

## 📝 실습 2: Topics 탐색

```bash
# 모든 토픽 목록 보기
ros2 topic list

# 더 자세한 정보 보기 (-t: 타입 포함)
ros2 topic list -t
```

**예상 출력**:
```
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/rosout [rcl_interfaces/msg/Log]
/turtle1/cmd_vel [geometry_msgs/msg/Twist]
/turtle1/color_sensor [turtlesim/msg/Color]
/turtle1/pose [turtlesim/msg/Pose]
```

**개념**:
- Topic = 노드들이 메시지를 주고받는 **통신 채널**
- `/turtle1/cmd_vel` = 거북이에게 움직임 명령을 보내는 토픽
- `/turtle1/pose` = 거북이의 현재 위치 정보를 받는 토픽

---

## 📝 실습 3: 거북이 위치 실시간 모니터링

```bash
# 거북이의 현재 위치를 실시간으로 확인
ros2 topic echo /turtle1/pose
```

**거북이를 마우스로 드래그해보세요!** 숫자가 변하는 것을 확인할 수 있습니다.

**개념**:
- `ros2 topic echo` = 토픽에서 발행되는 메시지를 **구독(Subscribe)**
- 실시간으로 데이터를 받아서 화면에 출력

---

## 📝 실습 4: 거북이 제어 (Publisher/Subscriber 체험)

### 터미널 3: Teleop 키보드 제어
```bash
wsl -d Ubuntu
source /opt/ros/jazzy/setup.bash
ros2 run turtlesim turtle_teleop_key
```

**화살표 키를 눌러서 거북이를 움직여보세요!**

**개념 이해**:
1. `turtle_teleop_key` 노드가 키보드 입력을 받음
2. `/turtle1/cmd_vel` 토픽에 **발행(Publish)**
3. `turtlesim` 노드가 `/turtle1/cmd_vel`을 **구독(Subscribe)**
4. 거북이가 움직임

```
[turtle_teleop_key] --발행--> [/turtle1/cmd_vel] --구독--> [turtlesim]
   (Publisher)                    (Topic)                (Subscriber)
```

---

## 📝 실습 5: 토픽 상세 정보 분석

```bash
# 토픽의 메시지 타입 확인
ros2 topic info /turtle1/cmd_vel

# 메시지 구조 확인
ros2 interface show geometry_msgs/msg/Twist

# 토픽의 발행/구독 관계 확인
ros2 topic info /turtle1/cmd_vel -v
```

**출력 분석**:
- `Publisher count: 1` → 1개의 노드가 발행 중
- `Subscription count: 1` → 1개의 노드가 구독 중

---

## 📝 실습 6: 명령어로 거북이 직접 제어하기

```bash
# 거북이를 앞으로 이동시키기
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 거북이를 회전시키기
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"

# 지속적으로 원을 그리게 하기 (1Hz)
ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```

**개념**:
- 우리가 직접 **Publisher**가 되어 메시지를 발행할 수 있음
- `--once`: 한 번만 발행
- `--rate 1`: 1Hz(초당 1번)로 지속 발행

**중지**: `Ctrl+C`

---

## 📝 실습 7: 노드 정보 상세 분석

```bash
# turtlesim 노드의 모든 정보 확인
ros2 node info /turtlesim
```

**출력에서 확인할 것**:
- **Subscribers**: 이 노드가 구독하는 토픽들
- **Publishers**: 이 노드가 발행하는 토픽들
- **Services**: 이 노드가 제공하는 서비스들
- **Actions**: 이 노드가 제공하는 액션들

---

## 📝 실습 8: Service 호출 (거북이 텔레포트)

```bash
# 사용 가능한 서비스 목록
ros2 service list

# 거북이를 특정 위치로 순간이동
ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{x: 5.5, y: 5.5, theta: 0.0}"

# 새 거북이 생성
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0, y: 2.0, theta: 0.0, name: 'turtle2'}"
```

**개념**:
- **Service** = 요청-응답 방식의 통신
- Topic과 다르게 "요청하면 응답을 받음"
- 일회성 작업에 적합

---

## 📝 실습 9: RQt Graph로 시각화

```bash
# RQt Graph 실행 (노드와 토픽 관계를 그래프로 보기)
ros2 run rqt_graph rqt_graph
```

**확인할 것**:
- 노드들이 원(○)으로 표시됨
- 토픽들이 화살표로 연결됨
- Publisher → Topic → Subscriber 관계 시각화

---

## 🎓 핵심 개념 정리

### 1. Node (노드)
- ROS 2의 **실행 단위**
- 각자의 역할을 수행하는 독립적인 프로세스
- 예: `turtlesim`, `turtle_teleop_key`

### 2. Topic (토픽)
- 노드 간 **비동기 통신 채널**
- Publisher가 발행, Subscriber가 구독
- 1:N, N:1, N:N 통신 가능
- 예: `/turtle1/cmd_vel`, `/turtle1/pose`

### 3. Message (메시지)
- 토픽을 통해 전달되는 **데이터 구조**
- 예: `geometry_msgs/msg/Twist` (속도 명령)

### 4. Service (서비스)
- 노드 간 **동기 요청-응답 통신**
- 일회성 작업에 적합
- 예: `/spawn` (거북이 생성)

---

## ✅ 실습 완료 체크리스트

- [ ] Turtlesim 실행 및 거북이 확인
- [ ] `ros2 node list`로 노드 목록 확인
- [ ] `ros2 topic list`로 토픽 목록 확인
- [ ] `ros2 topic echo`로 실시간 데이터 모니터링
- [ ] 키보드로 거북이 제어 (Publisher/Subscriber 체험)
- [ ] 명령어로 거북이 직접 제어
- [ ] Service로 거북이 텔레포트
- [ ] RQt Graph로 노드 관계 시각화

---

## 🤔 스스로에게 질문하기

1. Node와 Topic의 차이는 무엇인가?
2. Publisher와 Subscriber는 어떻게 연결되는가?
3. Topic과 Service의 차이는 무엇인가?
4. 한 노드가 여러 토픽을 발행/구독할 수 있는가?

**답을 모르면 AI에게 질문하세요!**

---

**다음 단계**: AI와 함께 개념 심화 → 직접 Python 노드 작성
