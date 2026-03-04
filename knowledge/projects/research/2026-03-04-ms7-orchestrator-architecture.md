# 리서치 보고서: MS-7 오케스트레이터 아키텍처
- 조사일: 2026년 3월 4일 (수)
- 키워드: multi-robot orchestration, ROS2 Jazzy, BehaviorTree.CPP, Open-RMF, CBBA, Lifecycle Node, task allocation, DARPA SubT, fire robot
- 연관 마일스톤: MS-7 (Orchestrator Node)

---

## 핵심 요약

ARGOS MS-7 오케스트레이터는 **감독 자율성(Supervised Autonomy)** 아키텍처를 채택해야 한다. 완전 중앙집중도, 완전 분산도 아닌 중간 지점이다. 오케스트레이터가 임무를 할당하고 상황을 종합하되, 각 로봇은 독립적으로 판단하고 통신 두절 시 자율 폴백(fallback)을 실행한다. DARPA SubT CERBERUS 우승팀과 JPL CoSTAR 팀 모두 이 방식을 채택했다. BehaviorTree.CPP가 오케스트레이터의 상태 흐름을 제어하고, 임무 할당은 거리 기반 + 능력 필터링 방식이 ARGOS 2~3대 규모에 가장 적합하다.

---

## 1. 멀티로봇 오케스트레이션 아키텍처

### 1-1. 중앙집중 vs 분산: 소방 현장 적합성 비교

소방 현장은 통신 품질이 불안정하고, 인간 지휘관의 개입이 필수적이며, 임무 우선순위가 실시간으로 변한다. 이 세 가지 조건이 아키텍처 선택을 결정한다.

| 항목 | 완전 중앙집중 | 감독 자율성 (권장) | 완전 분산 |
|------|------------|-----------------|---------|
| 임무 일관성 | 높음 | 높음 | 낮음 (충돌 가능) |
| 통신 두절 대응 | 전체 마비 | **로봇 독립 폴백** | 유지 |
| 인간 지휘관 개입 | 쉬움 | **쉬움** | 어려움 |
| 구현 복잡도 | 낮음 | 중간 | 높음 |
| 소방 현장 적합성 | 낮음 | **높음** | 중간 |

**결론**: 감독 자율성 방식이 소방 현장에 최적이다.

오케스트레이터가 "어디를 탐색할지", "어떤 로봇이 담당할지"를 결정하되, 각 로봇은 Nav2 스택을 통해 스스로 장애물을 회피하고 경로를 계획한다. 통신 두절 시 각 로봇이 정해진 폴백 행동(탐색 지속 → 귀환)을 실행한다.

---

### 1-2. DARPA SubT 우승팀 오케스트레이션 구조

#### Team CERBERUS (1위, 2021)
- ETH Zurich 주도, ANYmal 보행 로봇 4대 + 드론으로 구성
- **핵심 구조**: "단일 인간 감독관 + 고수준 지시"
  - 감독관은 로봇에 영역 탐색 명령만 내림
  - 세부 경로 계획은 로봇 자율 판단
  - 운영자가 로봇의 자율성 수준을 4단계로 조절 가능
    - 수동 조종 → 스마트 조이스틱 → 웨이포인트 → 완전 자율
- **MUX(멀티플렉서) + BehaviorTree 구조**
  - BehaviorTree가 현재 자율성 수준 결정
  - MUX가 해당 채널(제어 소스)만 활성화
  - 충돌하는 지시를 단일 인터페이스에서 차단
- **통신 두절 대응**: 링크 가용 시 고수준 명령 수신, 두절 시 마지막 할당된 탐색 영역 자율 수행

#### Team CoSTAR / NeBula (JPL NASA, 2위~3위)
- **NeBula(Networked Belief-aware Perceptual Autonomy)** 프레임워크
- 불확실성 인식 기반 계획: 센서 오차, 통신 상태, 시스템 건강 상태를 확률 분포로 표현
- 이종 로봇 플랫폼 지원: 바퀴형, 보행형, 트랙형, 비행형
- **리스크 인식 임무 계획**: 배터리 부족 시 귀환, 위험 지형 시 우회를 자동 판단
- 네트워크화된 분산 추론: 직접 연결 없어도 체인 중계 방식으로 정보 전파

#### DARPA SubT 이후 레슨 러닝 (2024 논문)
핵심 설계 원칙 5가지:
1. 운영자가 자율성 수준을 실시간 조절 가능해야 함
2. 이종 로봇 간 인터페이스 표준화가 확장성의 핵심
3. 중앙화된 제어 흐름으로 충돌 명령 방지
4. 맥락에 따른 적응형 UI (인지 부하 최소화)
5. 모듈식 설계로 로봇 추가/교체 용이

---

### 1-3. Open-RMF 입찰 기반 태스크 디스패칭

Open-RMF는 이종 로봇 플릿 관리 오픈소스 프레임워크다. ARGOS가 직접 사용하기보다 설계 패턴을 참고할 가치가 있다.

**입찰(Bidding) 프로세스**
```
새 태스크 요청
      |
Dispatcher → BidNotice 브로드캐스트 (모든 fleet adapter에)
      |
각 fleet adapter → 처리 가능 여부 판단 → BidProposal 반환
  (비용 = 완료 시간 + 이동 거리 + 배터리 소모)
      |
Dispatcher → 최저 비용 제안 선택 → DispatchRequest 전송
      |
해당 fleet adapter → 로봇에 태스크 할당
```

**ARGOS 적용 방식**: 공식 RMF 패키지 대신 동일한 입찰 논리를 오케스트레이터 노드 내에 Python으로 구현한다. 2~3대 규모에서는 단순 구현으로 충분하다.

---

### 1-4. CBBA / ACBBA 알고리즘 (경매 기반 할당)

**용어 설명**: CBBA는 "합의 기반 묶음 알고리즘(Consensus-Based Bundle Algorithm)"의 약자. 각 로봇이 자신이 처리할 임무 묶음을 스스로 결정하고, 이후 로봇끼리 합의하여 중복 할당을 제거하는 분산 알고리즘이다.

**CBBA 2단계 구조**
```
Phase 1: 포함 단계 (Inclusion)
  각 로봇이 탐욕(greedy) 방식으로 임무 묶음 구성
  자신이 가장 잘 할 수 있는 임무부터 선택

Phase 2: 합의 단계 (Consensus)
  로봇 간 임무 할당 목록 공유
  충돌(같은 임무를 두 로봇이 선택) 발견 시
  → 더 높은 입찰가 제시한 로봇이 유지
  → 다른 로봇은 다음 순위 임무 선택
```

**ACBBA (Asynchronous CBBA)**: 통신이 비동기적(로봇 간 연결이 끊겼다 붙었다)인 환경에서도 수렴성(결국 충돌 없는 해결책 도달)을 보장한다. 소방 현장 무선 통신 환경에 이론적으로 적합하다.

**ARGOS 규모 판단**: 2~3대에서 CBBA의 복잡도는 과도하다. 거리 기반 할당(가장 가까운 로봇에 임무 배정)이 MVP에 충분하고, 3대 이상으로 확장 시 CBBA 도입을 고려한다.

---

## 2. ROS 2 설계 패턴

### 2-1. Lifecycle Node: 오케스트레이터에 적합한가?

**Lifecycle Node란**: ROS 2 노드를 단순히 시작/종료하는 것이 아니라, 상태 기계(State Machine)처럼 관리하는 패턴이다.

```
상태 기계 구조:
  [Unconfigured]
       |  configure()
  [Inactive]
       |  activate()
  [Active]   ←── 실제 동작 상태
       |  deactivate()
  [Inactive]
       |  cleanup()
  [Unconfigured]
```

**오케스트레이터에의 적합성**

| 사용 이유 | 구체적 이점 |
|---------|-----------|
| 시작 순서 보장 | SLAM → Nav2 → 탐색 노드 → 오케스트레이터 순서 활성화 |
| 긴급 정지 | deactivate() 한 번으로 전체 임무 중단 |
| 상태 모니터링 | 외부에서 오케스트레이터 상태 확인 가능 |
| Nav2 호환성 | Nav2 lifecycle_manager와 동일 패턴 → 연동 자연스러움 |

**결론**: 오케스트레이터 노드에 Lifecycle Node 적용을 권장한다. Nav2 스택과 동일한 상태 관리 패턴을 사용하므로 일관성이 높다.

---

### 2-2. BehaviorTree.CPP: 고수준 미션 BT

BehaviorTree.CPP는 Nav2가 내부적으로 사용하는 동일 라이브러리다. Nav2의 저수준 항법 BT와 별개로, 오케스트레이터는 고수준 임무 BT를 구성할 수 있다.

**BT 계층 구조 (2계층)**
```
[고수준 임무 BT]  ← 오케스트레이터가 관리
    |
    |─ Sequence: 화재 대응 임무
    |    |─ Action: 로봇 상태 확인 (RosServiceNode)
    |    |─ Action: 임무 할당 실행 (커스텀 Action)
    |    |─ Action: 탐색 모니터링
    |    └─ Action: 화점 보고 → 귀환 명령

[저수준 항법 BT]  ← Nav2가 관리
    |
    |─ Sequence: NavigateToPose
         |─ ComputePathToPose
         |─ FollowPath
         └─ RecoveryFallback
```

**BehaviorTree.ROS2 연동 패턴**
```python
# ROS2 Action 노드 래핑 예시
class AssignMissionAction(RosActionNode):
    def setGoal(self, goal):
        goal.robot_id = self.getInput("robot_id")
        goal.mission_type = self.getInput("mission_type")
        goal.target_area = self.getInput("target_area")
        return True

    def onResultReceived(self, result):
        if result.success:
            return NodeStatus.SUCCESS
        return NodeStatus.FAILURE

    def onFeedback(self, feedback):
        # 실시간 진행 상황 BT 블랙보드에 업데이트
        self.setOutput("progress", feedback.completion_percent)
```

**XML로 BT 정의하는 방식의 장점**: 코드 수정 없이 XML 파일만 변경해서 임무 흐름을 수정할 수 있다. 소방 시나리오별로 BT XML을 다르게 로드하는 방식으로 확장 가능하다.

---

### 2-3. Custom Action/Service 인터페이스 설계

오케스트레이터는 ROS 2 표준 인터페이스를 확장한 커스텀 인터페이스를 정의해야 한다.

**필요한 인터페이스 목록**

```
my_robot_interfaces/
├── action/
│   ├── AssignMission.action      ← 로봇에 임무 할당 (핵심)
│   └── EmergencyReturn.action    ← 긴급 귀환 명령
├── msg/
│   ├── RobotStatus.msg           ← 로봇 상태 보고
│   ├── FireAlert.msg             ← 화점 발견 알림 (MS-4에서 이미 일부 정의)
│   ├── MissionState.msg          ← 전체 임무 상태
│   └── RobotCapabilities.msg     ← 로봇 능력 기술
└── srv/
    ├── GetRobotStatus.srv        ← 로봇 상태 조회
    └── UpdatePriority.srv        ← 임무 우선순위 변경
```

**AssignMission.action 상세 설계**
```
# Goal: 오케스트레이터 → 로봇
string robot_id              # 대상 로봇 ID (예: "ugv_1", "drone_1")
uint8 mission_type           # 0=탐색, 1=화점감시, 2=귀환, 3=대기
geometry_msgs/Polygon target_area  # 담당 탐색 영역 (다각형)
uint8 priority               # 1=높음, 2=중간, 3=낮음
float64 timeout_sec          # 임무 제한 시간 (0 = 무제한)
---
# Result: 로봇 → 오케스트레이터 (임무 완료/실패 시)
bool success
string result_summary        # "탐색 완료: 화점 미발견" 또는 "배터리 부족으로 귀환"
geometry_msgs/PoseArray visited_poses  # 방문한 위치 목록
float64 coverage_percent     # 할당 영역 탐색 완료율
---
# Feedback: 로봇 → 오케스트레이터 (진행 중 주기적)
float64 completion_percent   # 현재 진행률
geometry_msgs/Pose current_pose  # 현재 위치
float64 battery_percent      # 배터리 잔량
string status_message        # 상태 메시지
bool fire_detected           # 화점 감지 여부
geometry_msgs/Point fire_location  # 화점 위치 (감지된 경우)
```

**RobotCapabilities.msg 설계**
```
string robot_id
string platform_type         # "ugv", "drone", "legged"
float64 max_speed_mps
float64 max_payload_kg
bool has_thermal_camera
bool has_water_cannon
bool can_climb_stairs        # 계단 등반 가능 여부
float64 battery_capacity_wh
float64 max_range_m          # 최대 작전 반경
```

---

### 2-4. DDS QoS 전략: 소방 현장 통신 열악 환경

소방 현장은 금속 구조물, 콘크리트 벽, 연기로 인해 무선 통신 품질이 급격히 저하된다. 토픽별로 다른 QoS 프로파일을 적용해야 한다.

**토픽별 QoS 설정 권장안**

| 토픽 종류 | 예시 | Reliability | Durability | History Depth | 이유 |
|---------|------|------------|-----------|--------------|------|
| 임무 명령 | `/assign_mission` | **RELIABLE** | TRANSIENT_LOCAL | 10 | 명령 유실 불가 |
| 긴급 귀환 | `/emergency_return` | **RELIABLE** | TRANSIENT_LOCAL | 1 | 절대 유실 불가 |
| 화점 알림 | `/fire_alert` | **RELIABLE** | TRANSIENT_LOCAL | 10 | 유실 불가 |
| 로봇 상태 | `/robot_status` | BEST_EFFORT | VOLATILE | 5 | 최신 상태만 필요 |
| 센서 스트림 | `/thermal/image` | BEST_EFFORT | VOLATILE | 1 | 지연 최소화 우선 |
| SLAM 지도 | `/map` | RELIABLE | TRANSIENT_LOCAL | 1 | 재연결 시 최신 맵 전달 |
| 오도메트리 | `/odom` | BEST_EFFORT | VOLATILE | 10 | 지연 최소화 |

**핵심 원칙**
- 제어/명령 토픽: RELIABLE + TRANSIENT_LOCAL (재연결 시 마지막 명령 전달)
- 센서 스트림: BEST_EFFORT + VOLATILE (최신 데이터, 손실 허용)
- Heartbeat 토픽: BEST_EFFORT + Deadline QoS 조합 (일정 주기 내 수신 없으면 타임아웃 이벤트 발생)

**Deadline QoS 활용 (Heartbeat 대용)**
```python
# 로봇 상태 토픽에 Deadline QoS 적용
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.qos import QoSDeadlinePolicy
import rclpy.duration

heartbeat_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    deadline=rclpy.duration.Duration(seconds=2.0),  # 2초 내 수신 없으면 이벤트
    depth=5
)
```

---

## 3. 임무 할당 시스템

### 3-1. 능력(Capabilities) 기반 매칭

각 로봇의 물리적 능력을 `RobotCapabilities.msg`로 등록하고, 임무 유형과 매칭한다.

```
임무 유형별 로봇 우선순위:

탐색 임무 (EXPLORE):
  1순위: UGV (안정적, 지속 시간 김)
  2순위: 드론 (빠르지만 배터리 제한)
  3순위: 보행형 (계단 통과 가능)

화점 감시 (FIRE_WATCH):
  1순위: 열화상 카메라 보유 로봇
  2순위: 드론 (상공에서 넓은 시야)

계단 통과 요구 구역:
  1순위: 보행형 로봇
  제외: UGV (계단 불가)

구조 지원 (RESCUE_SUPPORT):
  1순위: 배터리 여유 있고 카메라 보유 로봇
  조건: battery_percent > 40
```

**매칭 로직 (Python 의사코드)**
```python
def find_best_robot(mission_type, available_robots):
    candidates = []
    for robot in available_robots:
        cap = robot.capabilities
        score = 0

        # 기본 능력 필터링
        if mission_type == FIRE_WATCH and not cap.has_thermal_camera:
            continue  # 열화상 없으면 제외
        if mission_type == STAIR_AREA and not cap.can_climb_stairs:
            continue  # 계단 불가면 제외

        # 점수 계산
        score += cap.battery_percent * 0.4       # 배터리 40% 비중
        score += (1 / robot.dist_to_target) * 0.4  # 거리 40% 비중 (가까울수록 높은 점수)
        score += cap.max_speed_mps * 0.2           # 속도 20% 비중

        candidates.append((robot, score))

    # 최고 점수 로봇 선택
    return max(candidates, key=lambda x: x[1])[0]
```

---

### 3-2. 실시간 재할당 (Dynamic Reallocation)

소방 현장에서 상황은 급변한다. 오케스트레이터는 세 가지 트리거에서 임무를 재할당해야 한다.

**재할당 트리거와 대응**

| 트리거 | 감지 방법 | 대응 |
|--------|---------|------|
| 로봇 장애 | Heartbeat 타임아웃 (2~5초) | 해당 로봇 임무를 다른 로봇에 재할당 |
| 화점 발견 | `/fire_alert` 수신 | 탐색 임무 중인 로봇 일부를 화점 감시로 전환 |
| 통신 두절 | DDS Deadline 이벤트 | 자율 폴백 모드 전환 (로봇이 독자 판단) |
| 배터리 임계 | RobotStatus.battery < 20% | 귀환 임무 자동 삽입 |
| 탐색 완료 | 탐색 커버리지 100% | 다음 단계 임무 (귀환 또는 대기) 할당 |

**재할당 프로세스**
```
트리거 감지
    |
현재 임무 목록 갱신 (장애 로봇 임무를 UNASSIGNED 큐로)
    |
사용 가능 로봇 목록 갱신
    |
UNASSIGNED 임무를 우선순위 순으로 정렬
    |
find_best_robot() 호출하여 재할당
    |
새 AssignMission.action 전송
    |
임무 상태 로그 갱신
```

---

### 3-3. 우선순위 기반 임무 큐

소방 임무는 명확한 우선순위 계층이 있다.

```
PRIORITY_1 (긴급): 인명 구조 지원
  - 생존자 위치 확인 신호 감지 시
  - 부상 대원 발생 시
  - 즉시 모든 자원 집중

PRIORITY_2 (높음): 화점 위치 확인 및 보고
  - 화점 발견 즉시
  - 지휘관에게 좌표 전송
  - 화점 감시 로봇 지정

PRIORITY_3 (중간): 영역 탐색
  - 미탐색 구역 체계적 순환
  - 프론티어 탐색 알고리즘 적용

PRIORITY_4 (낮음): 귀환 대기
  - 배터리 임계 미만 로봇
  - 임무 완료 로봇
```

**선점(Preemption) 메커니즘**: 낮은 우선순위 임무 수행 중에 높은 우선순위 임무 발생 시, `cancel_goal_async()` 호출 후 새 임무 할당. Nav2 액션 서버는 취소를 즉시 처리한다.

---

## 4. 소방 지휘체계 매핑

### 4-1. 지휘 계층 구조

```
[현장 지휘관] (인간)
      |
      | 고수준 명령: "1층 동편 탐색", "화점 감시 유지", "전원 철수"
      |
[ARGOS 오케스트레이터] (MS-7 노드)
      |
      |── 임무 할당: AssignMission.action
      |── 상태 수집: RobotStatus.msg 구독
      |── 지도 통합: merged_map 관리
      |
      |──────────────────────────────────
      |             |              |
  [UGV_1]       [UGV_2]        [Drone_1]
  (1층 서편)    (1층 동편)     (건물 상공)
      |             |              |
  Nav2 Stack   Nav2 Stack    PX4/Nav2 Stack
  SLAM         SLAM           GPS+LiDAR
  열화상       열화상          열화상
```

### 4-2. 지휘관 오버라이드 (Human Override)

감독 자율성의 핵심은 언제든 인간이 개입할 수 있어야 한다는 것이다.

**오버라이드 토픽 설계**
```
/commander_override (std_msgs/String, RELIABLE QoS)
  명령 형식: JSON 문자열
  예시:
    {"robot": "ugv_1", "action": "goto", "x": 10.5, "y": 3.2}
    {"robot": "all", "action": "emergency_return"}
    {"robot": "ugv_2", "action": "pause"}
    {"robot": "drone_1", "action": "watch_fire", "location": {...}}
```

**오케스트레이터 오버라이드 처리 로직**
```python
def commander_override_callback(self, msg):
    cmd = json.loads(msg.data)

    if cmd["action"] == "emergency_return":
        # 모든 임무 취소, 즉시 귀환
        self.cancel_all_missions()
        self.assign_return_all()

    elif cmd["action"] == "pause":
        # 특정 로봇 일시 정지
        self.pause_robot(cmd["robot"])

    elif cmd["action"] == "goto":
        # 임무 큐를 무시하고 즉시 해당 위치로
        self.assign_direct_goto(cmd["robot"], cmd["x"], cmd["y"])

    # 오버라이드 기록 (감사 추적)
    self.log_override(cmd)
```

---

### 4-3. 소방 시나리오: 전체 임무 흐름

```
시나리오: 3층 건물 화재 신고, ARGOS 2대 UGV 투입

STAGE 1: 초기화 (0~30초)
  - 오케스트레이터 활성화 (Lifecycle: Unconfigured → Active)
  - 로봇 능력 등록 (RobotCapabilities 수신)
  - 초기 임무 할당: UGV_1 → 1층 서편, UGV_2 → 1층 동편

STAGE 2: 진입 탐색 (30초~5분)
  - 각 UGV 프론티어 탐색 실행 (MS-6 연동)
  - 오케스트레이터: 30초 주기로 merged_map 갱신 확인
  - 오케스트레이터: 5초 주기로 로봇 상태 수신 및 건강 체크

STAGE 3: 화점 발견 (이벤트)
  - UGV_1에서 /fire_alert 발행 (confidence > 0.8)
  - 오케스트레이터: PRIORITY_2 임무 즉시 생성
  - UGV_1: 화점 감시 유지 명령 수신
  - UGV_2: 나머지 구역 탐색 계속

STAGE 4: 지휘관 보고 및 결정 (이벤트)
  - 오케스트레이터: 화점 위치와 신뢰도를 /mission_summary에 발행
  - 지휘관: 정보 수신 후 오버라이드 명령 가능
  - 예: "UGV_2도 화점 감시로 전환" 또는 "전원 철수"

STAGE 5: 철수 (명령)
  - EmergencyReturn.action 전송 (모든 로봇)
  - 각 로봇: 현재 임무 취소 → 출발점으로 Nav2 항법
  - 오케스트레이터: 모든 로봇 귀환 확인 후 임무 종료
  - 최종 보고: 탐색 커버리지, 화점 위치, 임무 기록 저장
```

---

## 5. 통신 두절 대응

### 5-1. Heartbeat 메커니즘

**구현 방식**: ROS 2의 DDS Deadline QoS를 활용하면 별도 Heartbeat 노드 없이 통신 두절을 감지할 수 있다.

```python
class OrchestratorNode(LifecycleNode):
    def setup_robot_monitoring(self):
        # 각 로봇의 상태 토픽 구독 (Deadline QoS)
        for robot_id in self.robot_ids:
            qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                deadline=Duration(seconds=5.0),  # 5초 내 수신 없으면 타임아웃
                depth=1
            )
            self.create_subscription(
                RobotStatus,
                f'/{robot_id}/status',
                lambda msg, rid=robot_id: self.robot_status_callback(msg, rid),
                qos
            )
            # Deadline 이벤트 핸들러 등록
            self.register_deadline_callback(
                f'/{robot_id}/status',
                lambda rid=robot_id: self.robot_timeout_callback(rid)
            )

    def robot_timeout_callback(self, robot_id):
        self.get_logger().warn(f"{robot_id} 응답 없음: 5초 타임아웃")
        # 1. 해당 로봇을 OFFLINE 상태로 마킹
        self.robot_states[robot_id] = "OFFLINE"
        # 2. 해당 로봇의 임무를 UNASSIGNED로 전환
        self.reassign_robot_missions(robot_id)
        # 3. 지휘관에게 알림
        self.publish_alert(f"로봇 {robot_id} 통신 두절")
```

---

### 5-2. 로봇 측 자율 폴백

각 로봇은 오케스트레이터와 통신이 끊겨도 독립적으로 동작해야 한다.

**폴백 행동 트리 (각 로봇 내부)**
```
폴백 BT:
  Fallback
    |── [오케스트레이터 연결 확인]
    |      ↓ SUCCESS → 정상 임무 수행
    |
    └── [독립 폴백 모드] (통신 두절 시)
           |── Sequence
           |      |── [마지막 할당 임무 계속 수행]
           |      |── [배터리 > 30% 확인]
           |      └── [마지막 알려진 프론티어 탐색]
           |
           └── [배터리 임계 폴백]
                  |── [배터리 < 20%]
                  └── [출발점으로 자율 귀환]
```

**구현 핵심**: 각 로봇의 Nav2 스택은 독립적으로 동작하므로, 오케스트레이터 없이도 `NavigateToPose`를 직접 호출하여 항법이 가능하다. 폴백 노드가 마지막 목표 좌표를 로컬 파라미터에 저장해두고, 타임아웃 시 이를 사용한다.

---

### 5-3. 재연결 시 상태 동기화

**문제**: 5분간 통신 두절 후 로봇이 재연결되면, 오케스트레이터는 로봇의 현재 상태(위치, 배터리, 탐색 커버리지)를 모른다.

**동기화 프로토콜**
```
재연결 감지 (DDS Deadline 이벤트 해제 또는 상태 토픽 수신 재개)
      |
오케스트레이터 → /request_state_sync (RobotID) 발행
      |
로봇 → /state_sync_response 응답:
  {
    current_pose: geometry_msgs/PoseStamped,
    battery_percent: 65.0,
    last_mission_status: "EXPLORING",
    coverage_map: nav_msgs/OccupancyGrid (탐색한 영역),
    fire_alerts: [...] (두절 기간 중 감지된 화점 목록)
  }
      |
오케스트레이터: 전체 지도 갱신, 임무 상태 갱신
      |
임무 재할당 결정 (배터리, 탐색 완료율 기준)
```

**DDS TRANSIENT_LOCAL 활용**: `/merged_map`과 `/mission_state` 토픽에 TRANSIENT_LOCAL 내구성을 설정하면, 재연결된 로봇이 구독 즉시 최신 지도와 임무 상태를 자동 수신한다. 별도 동기화 서비스 없이도 최신 정보를 전달할 수 있다.

---

## 6. 참고 오픈소스

### 6-1. 핵심 참고 구현체

| 프로젝트 | GitHub | 설명 | ARGOS 활용 |
|--------|--------|------|-----------|
| **BehaviorTree.ROS2** | [BehaviorTree/BehaviorTree.ROS2](https://github.com/BehaviorTree/BehaviorTree.ROS2) | ROS2 Action/Service 노드 래퍼 | 오케스트레이터 BT 구현 |
| **Open-RMF rmf_task** | [open-rmf/rmf_task](https://github.com/open-rmf/rmf_task) | 태스크 플래너, 입찰 시스템 | 임무 할당 로직 참고 |
| **Multi-Robot Coordination** | [JayDS22/Multi-Robot-Coordination-Framework](https://github.com/JayDS22/Multi-Robot-Coordination-Framework) | 경매 알고리즘 + ROS2 | CBBA 구현 참고 |
| **CBBA Python** | [zehuilu/CBBA-Python](https://github.com/zehuilu/CBBA-Python) | CBBA 순수 Python 구현 | 알고리즘 이해용 |
| **RobotFleet** | [arxiv 2510.10379](https://arxiv.org/html/2510.10379v1) | 중앙집중식 태스크 플래닝 | 아키텍처 설계 참고 |

### 6-2. nav2_behavior_tree 플러그인 활용

Nav2에서 제공하는 기존 BT 노드를 오케스트레이터 BT에서 직접 재사용할 수 있다.

```xml
<!-- 오케스트레이터 고수준 BT XML 예시 -->
<root BTCPP_format="4">
  <BehaviorTree ID="ArgosOrchestratorBT">
    <Sequence name="FireResponseMission">
      <!-- Nav2 기존 BT 노드 재사용 -->
      <RateController hz="0.5">
        <Action ID="CheckRobotStatuses"/>
      </RateController>

      <!-- 커스텀 오케스트레이터 액션 -->
      <Action ID="AllocateMissions"
              available_robots="{available_robots}"
              pending_missions="{pending_missions}"/>

      <!-- 화점 감지 모니터링 -->
      <ReactiveFallback>
        <Condition ID="NoFireDetected"/>
        <Action ID="HandleFireAlert"
                fire_location="{fire_location}"/>
      </ReactiveFallback>

    </Sequence>
  </BehaviorTree>
</root>
```

---

## 7. 권장 ARGOS 오케스트레이터 아키텍처

### 7-1. 전체 아키텍처 설계 (ARGOS Orchestrator v1)

```
╔══════════════════════════════════════════════════════════╗
║              ARGOS Orchestrator Node                     ║
║              (LifecycleNode + BehaviorTree.CPP)          ║
║                                                          ║
║  ┌─────────────────────────────────────────────────┐     ║
║  │  Mission Manager                                │     ║
║  │  - 임무 큐 관리 (우선순위 기반)                  │     ║
║  │  - 로봇-임무 매칭 (능력 기반)                    │     ║
║  │  - 재할당 로직                                   │     ║
║  └──────────────────┬──────────────────────────────┘     ║
║                     │                                    ║
║  ┌──────────────────▼──────────────────────────────┐     ║
║  │  Robot Registry                                 │     ║
║  │  - 로봇 상태 추적 (ONLINE/OFFLINE/BUSY/IDLE)     │     ║
║  │  - Heartbeat 모니터링 (Deadline QoS)             │     ║
║  │  - 능력 데이터베이스                             │     ║
║  └──────────────────┬──────────────────────────────┘     ║
║                     │                                    ║
║  ┌──────────────────▼──────────────────────────────┐     ║
║  │  Situation Awareness                            │     ║
║  │  - merged_map 관리                              │     ║
║  │  - 화점 목록 추적                               │     ║
║  │  - 탐색 커버리지 계산                            │     ║
║  └─────────────────────────────────────────────────┘     ║
║                                                          ║
╠══════════════════════════════════════════════════════════╣
║  입력                         출력                        ║
║  /robot_*/status              /assign_mission (Action)   ║
║  /fire_alert                  /emergency_return (Action) ║
║  /merged_map                  /mission_summary           ║
║  /commander_override          /orchestrator_state        ║
╚══════════════════════════════════════════════════════════╝
            |                          |
     ┌──────┘                          └──────┐
     │                                        │
┌────▼─────┐  AssignMission.action  ┌────────▼─────┐
│  UGV_1   │◄─────────────────────►│  UGV_2       │
│  Nav2    │                       │  Nav2        │
│  SLAM    │                       │  SLAM        │
│  열화상  │                       │  열화상      │
└──────────┘                       └──────────────┘
```

### 7-2. 노드 구조 (Python)

```python
class ArgosOrchestrator(LifecycleNode):

    def __init__(self):
        super().__init__('argos_orchestrator')
        # 모듈 초기화는 on_activate에서
        self.robot_registry = {}     # robot_id → RobotState
        self.mission_queue = []      # 우선순위 큐
        self.fire_alerts = []        # 화점 목록
        self.bt_executor = None      # BehaviorTree 실행기

    def on_configure(self, state):
        # 파라미터 로드
        self.declare_parameter('robot_ids', ['ugv_1', 'ugv_2'])
        self.declare_parameter('heartbeat_timeout', 5.0)
        self.declare_parameter('mission_bt_xml', 'fire_response.xml')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        # 구독자 생성
        self._setup_robot_subscribers()
        # 액션 클라이언트 생성
        self._setup_action_clients()
        # BehaviorTree 로드
        self._load_mission_bt()
        # BT 실행 타이머 (10Hz)
        self.create_timer(0.1, self._tick_behavior_tree)
        return TransitionCallbackReturn.SUCCESS
```

---

## 8. 필요한 Custom Interface 전체 목록

### Action 인터페이스 (`.action`)
```
AssignMission.action
  └── 오케스트레이터 → 로봇: 임무 할당
  └── 피드백: 진행률, 현재 위치, 배터리, 화점 감지

EmergencyReturn.action
  └── 오케스트레이터 → 로봇: 즉시 귀환 명령
  └── 결과: 귀환 성공/실패, 귀환 시간
```

### Message 인터페이스 (`.msg`)
```
RobotStatus.msg
  └── 로봇 → 오케스트레이터: 주기적 상태 보고
  └── 필드: 위치, 배터리, 현재 임무, 통신 품질

RobotCapabilities.msg
  └── 시작 시 1회 등록
  └── 필드: 플랫폼 유형, 속도, 센서, 계단 등반 여부

MissionState.msg
  └── 오케스트레이터 → UI: 전체 임무 현황
  └── 필드: 활성 임무 목록, 로봇별 상태 요약

FireAlert.msg (MS-4에서 이미 일부 정의 → 확장)
  └── 로봇 → 오케스트레이터: 화점 감지 알림
  └── 필드: 위치, 신뢰도, 감지 시각, 열화상 이미지
```

### Service 인터페이스 (`.srv`)
```
GetRobotStatus.srv
  └── 요청: robot_id
  └── 응답: 현재 상태 전체

UpdateMissionPriority.srv
  └── 요청: mission_id, new_priority
  └── 응답: 성공 여부

RequestStateSync.srv
  └── 재연결 시 상태 동기화
  └── 응답: 로봇의 전체 상태 스냅샷
```

---

## 9. 구현 우선순위

### MVP (최소 기능 제품): MS-7 핵심

```
Phase 1: 뼈대 구현 (1~2일)
  [x] LifecycleNode 기반 오케스트레이터 노드 생성
  [x] RobotRegistry 구현 (등록/상태 추적)
  [x] Heartbeat 모니터링 (Deadline QoS 5초)
  [x] 기본 AssignMission.action 인터페이스 정의

Phase 2: 임무 할당 (2~3일)
  [x] 거리 기반 로봇 선택 (find_best_robot)
  [x] 우선순위 큐 구현 (PRIORITY_1~4)
  [x] MS-6 프론티어 탐색 연동 (탐색 임무 할당)

Phase 3: 이벤트 대응 (2~3일)
  [x] 화점 발견 → 임무 재할당 로직
  [x] 지휘관 오버라이드 처리
  [x] 통신 두절 → 폴백 모드 전환

Phase 4: BehaviorTree 통합 (선택, 2~3일)
  [ ] BehaviorTree.CPP 도입
  [ ] 임무 흐름 BT XML 작성
  [ ] 시뮬레이션 시나리오 테스트
```

### 확장 (MS-8 이후)
- 드론 플랫폼 추가 시 RobotCapabilities.platform_type = "drone" 필터 추가
- 3대 이상 시 CBBA 알고리즘 도입 고려
- MS-10 Web UI 연동: `/mission_state` 토픽을 rosbridge로 브라우저에 표시

---

## 비교 분석: 오케스트레이터 구현 방식

| 접근 방식 | 복잡도 | 확장성 | ARGOS 2~3대 적합성 | 권장 시기 |
|---------|--------|--------|-----------------|---------|
| 단순 상태 기계 (Python) | 낮음 | 낮음 | **MVP 적합** | Phase 1~3 |
| BehaviorTree.CPP | 중간 | 높음 | 좋음 | Phase 4 이후 |
| Open-RMF 직접 사용 | 높음 | 매우 높음 | 과도함 | MS-8+ (드론 추가 후) |
| CBBA 완전 구현 | 높음 | 높음 | 2~3대엔 과도 | 5대 이상 |

---

## 출처

- [Team CERBERUS DARPA SubT Technical Overview (arXiv 2207.04914)](https://arxiv.org/abs/2207.04914) — CERBERUS 아키텍처 전체 개요
- [Modular Resilient Scalable Lessons from DARPA SubT (arXiv 2404.17759)](https://arxiv.org/html/2404.17759) — SubT 이후 설계 레슨 러닝 5가지
- [NeBula/CoSTAR arXiv 2103.11470](https://arxiv.org/abs/2103.11470) — JPL 불확실성 인식 멀티로봇 프레임워크
- [Open-RMF Task Dispatching 공식 문서](https://osrf.github.io/ros2multirobotbook/task.html) — 입찰 기반 태스크 할당 패턴
- [Open-RMF ROS2 Multi-Robot Book](https://osrf.github.io/ros2multirobotbook/) — 멀티로봇 ROS2 설계 패턴 전반
- [BehaviorTree.CPP ROS2 Integration](https://www.behaviortree.dev/docs/ros2_integration/) — RosActionNode, RosServiceNode 구현 가이드
- [BehaviorTree.ROS2 GitHub](https://github.com/BehaviorTree/BehaviorTree.ROS2) — 즉시 사용 가능한 ROS2 BT 래퍼
- [CBBA MIT Aerospace Controls Lab](https://acl.mit.edu/projects/consensus-based-bundle-algorithm) — CBBA 원본 알고리즘
- [CBBA-Python GitHub](https://github.com/zehuilu/CBBA-Python) — Python CBBA 구현체
- [ROS2 QoS Policies 공식 문서](https://design.ros2.org/articles/qos.html) — Deadline, Reliability, Durability 설정 가이드
- [RobotFleet 오픈소스 프레임워크 (arXiv 2510.10379)](https://arxiv.org/html/2510.10379v1) — 중앙집중식 이종 로봇 태스크 플래닝
- [DARPA SubT 운영자 역할 분석](https://spectrum.ieee.org/darpa-subterranean-challenge-operator) — 인간-로봇 협업 인터페이스
- [Multi-Robot Coordination Framework GitHub](https://github.com/JayDS22/Multi-Robot-Coordination-Framework) — 경매 알고리즘 + ROS2 구현체
- [ROS2 Lifecycle Node 공식 설계 문서](https://design.ros2.org/articles/node_lifecycle.html) — 상태 기계 기반 노드 관리
