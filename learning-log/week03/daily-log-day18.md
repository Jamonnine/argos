# Day 18 - Custom Interfaces + Services: AI-Robotics 통합 파이프라인

**날짜**: 2026-02-19
**학습 시간**: ~1시간
**핵심 주제**: Custom .msg/.srv 생성, Service Server/Client, AI-Robotics 통합 아키텍처

---

## 🎯 오늘의 목표

ROS 2의 3대 통신 기본(Topic, Service, Action) 중 Service를 실전으로 완성하고, 커스텀 인터페이스(.msg/.srv)를 직접 설계해서 AI 인식 결과를 Nav2 이동 명령으로 연결하는 실무 패턴을 구현하는 것이었습니다.

---

## ✅ 달성 결과

- `DetectedObject.msg`, `DetectionArray.msg` 커스텀 메시지 생성
- `NavigateToObject.srv` 커스텀 서비스 생성
- `my_robot_interfaces` 패키지 빌드 및 인터페이스 확인
- `PerceptionBridgeNode` 구현 (Service Server + Action Client + Subscriber)
- `ros2 service call`로 서비스 호출 성공
- LiDAR → 가짜 감지 결과 → 서비스 응답 파이프라인 동작 확인

---

## 📐 왜 커스텀 인터페이스가 필요한가

표준 메시지만 사용할 때의 문제는 도메인 언어가 사라진다는 것입니다. AI 감지 결과를 `geometry_msgs/PoseArray`로 표현하면 confidence, class_name, track_id 같은 의미 있는 필드가 사라지고 좌표만 남습니다. 코드를 읽는 사람이 "이 PoseArray가 AI 감지 결과인지 경로 웨이포인트인지"를 context 없이 알 수 없게 됩니다.

커스텀 인터페이스는 **도메인 모델을 코드로 표현**하는 것입니다. `.msg` 파일이 곧 팀 간 API 계약서가 됩니다. AI 팀은 `DetectedObject`를 발행하는 노드를 만들고, 로봇 팀은 `DetectedObject`를 구독하는 노드를 만들면 됩니다. 서로의 내부 구현을 몰라도 됩니다.

### 인터페이스 분리 패키지의 이유

인터페이스를 `my_robot_interfaces`라는 별도 패키지로 분리한 이유가 있습니다. 만약 `my_robot_bringup`에 인터페이스를 넣으면, AI 감지 노드도 `my_robot_bringup`에 의존해야 합니다. AI 팀이 로봇 제어 코드 전체를 갖고 있어야 하는 것입니다. 인터페이스 패키지를 분리하면 의존 그래프가 단순해집니다.

```
my_robot_ai    ──→  my_robot_interfaces  ←──  my_robot_bringup
(AI 팀)              (계약서)                  (로봇 팀)
```

### Header는 항상 포함해야 하는 이유

`DetectedObject.msg`에 `std_msgs/Header`를 넣은 것이 핵심 설계 결정입니다. Header는 두 가지를 제공합니다. `stamp`는 이 데이터가 언제 생성됐는지 알려주고, `frame_id`는 어느 좌표계를 기준으로 표현됐는지 알려줍니다. 카메라 인식 결과는 `camera_optical_frame` 기준이고, LiDAR 인식 결과는 `base_link` 기준입니다. 이것을 명시하지 않으면 TF2로 좌표 변환이 불가능합니다. Header 없는 Pose는 "어디의 좌표인지 모르는 좌표"입니다.

---

## 🔄 Service vs Action 설계 결정

오늘 `NavigateToObject.srv`를 Service로 설계하면서 중요한 아키텍처 결정이 있었습니다.

Service는 "수락 여부"만 즉시 반환합니다. `accepted: True`는 "이동하겠다"는 약속이지 "이동 완료"가 아닙니다. 실제 이동은 Nav2 Action으로 비동기 처리됩니다. 이 분리가 중요한 이유는, Service를 블로킹으로 만들어서 이동 완료까지 기다리게 하면 그 시간 동안 다른 모든 서비스 요청이 대기해야 하기 때문입니다. 로봇이 3분 동안 이동 중에는 어떤 명령도 받을 수 없는 상태가 됩니다.

실무 패턴:
- `NavigateToObject.srv` (Service): "이 객체 앞으로 갈 수 있어? → 수락/거부"
- `NavigateToPose.action` (Action): 실제 이동 + 피드백 + 취소 가능

---

## 🏗️ PerceptionBridgeNode 아키텍처

```
[LiDAR /scan] ──→ _on_scan_simulate()
                       ↓ DetectedObject 생성
                  _detection_cache (deque, maxlen=10)
                       ↑
[/detections] ──→ _on_detections()  ← 실제 AI 모델 연결 지점

[/navigate_to_object Service]
  요청: target_class, min_confidence
       ↓ _find_best_detection() → 캐시에서 최적 객체 선택
       ↓ _compute_approach_pose() → approach_distance 앞 좌표 계산
       ↓ _send_nav2_goal() → Nav2 NavigateToPose Action 전송
  응답: accepted, message, target_pose, estimated_distance
```

### ReentrantCallbackGroup + MultiThreadedExecutor

Service 핸들러 안에서 Nav2 Action을 호출할 때 `ReentrantCallbackGroup`이 필요합니다. 기본 `MutuallyExclusiveCallbackGroup`에서는 하나의 콜백이 실행 중일 때 다른 콜백이 블로킹됩니다. Service 안에서 Action 응답을 기다리면 교착 상태(deadlock)가 발생합니다. `ReentrantCallbackGroup`은 이것을 허용합니다.

---

## 🛠️ 인터페이스 빌드 프로세스

```bash
# 1. .msg/.srv/.action 파일 작성
# 2. CMakeLists.txt에 등록
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedObject.msg"
  "srv/NavigateToObject.srv"
  DEPENDENCIES geometry_msgs std_msgs sensor_msgs
)
# 3. package.xml에 의존성 추가
<depend>rosidl_default_generators</depend>
<depend>rosidl_default_runtime</depend>
# 4. 빌드
colcon build --packages-select my_robot_interfaces
# 5. 확인
ros2 interface show my_robot_interfaces/msg/DetectedObject
```

`rosidl_generate_interfaces`가 `.msg` 파일을 Python 클래스(`from my_robot_interfaces.msg import DetectedObject`)와 C++ 헤더로 자동 변환합니다. 팀이 어떤 언어를 쓰든 동일한 인터페이스를 사용합니다.

---

## 🧪 서비스 호출 결과

```bash
ros2 service call /navigate_to_object \
  my_robot_interfaces/srv/NavigateToObject \
  "{target_class: 'obstacle', min_confidence: 0.5, approach_distance: 0.5}"

# 응답:
# accepted: True
# message: "'obstacle' 객체 발견 (confidence=0.85, distance=0.50m)"
# target_pose: x=-0.081, y=-0.058
# estimated_distance: 0.499m
```

---

## 🗂️ 생성된 파일

```
my_robot_interfaces/
├── msg/
│   ├── DetectedObject.msg     ← AI 감지 결과 단일 항목
│   └── DetectionArray.msg     ← 감지 결과 배열 (프레임 단위)
├── srv/
│   └── NavigateToObject.srv   ← 객체 앞으로 이동 요청

my_robot_bringup/my_robot_bringup/
└── perception_bridge_node.py  ← AI-Robotics 브릿지 노드
```

---

*Day 18 완료 - AI 인식과 로봇 제어를 연결하는 인터페이스 계층 설계 완성.*
