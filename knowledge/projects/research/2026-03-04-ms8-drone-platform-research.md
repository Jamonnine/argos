# 리서치 보고서: ARGOS MS-8 드론 플랫폼 (이종 군집 시뮬레이션)

- 조사일: 2026년 3월 4일 (수)
- 키워드: Gazebo Harmonic 쿼드콥터, MulticopterMotorModel, MulticopterVelocityControl, PX4 SITL, ArduPilot SITL, ROS 2 Jazzy 드론, 이종 군집, UGV+드론 통합, WSL2 성능
- 대상: ARGOS MS-8 마일스톤 — 이종(heterogeneous) 군집 시뮬레이션 (UGV + 쿼드콥터)

---

## 핵심 요약

**결론: Gazebo 내장 플러그인 방식(Option A)을 권장한다.**

Gazebo Harmonic(gz-sim8)은 `MulticopterMotorModel` + `MulticopterVelocityControl` 두 플러그인을 자체 내장하고 있으며, PX4/ArduPilot 같은 외부 비행 제어 소프트웨어 없이도 쿼드콥터를 시뮬레이션할 수 있다. `ros_gz_bridge`를 통해 `geometry_msgs/msg/Twist`(cmd_vel)로 직접 드론을 제어할 수 있어, 기존 UGV 오케스트레이터와 **동일한 인터페이스**로 통합이 가능하다.

RTX 4050 노트북 + WSL2 환경에서는 PX4 SITL 풀스택(추가 프로세스 3~4개)보다 내장 플러그인 방식이 CPU/메모리 부하가 현저히 낮아 UGV 2대 + 드론 1대 동시 구동에 적합하다.

---

## 상세 내용

### 1. Gazebo Harmonic 쿼드콥터 시뮬레이션 방법 비교

#### Option A: Gazebo 내장 플러그인 (권장 MVP)

Gazebo Harmonic(gz-sim8)에는 두 가지 쿼드콥터 관련 시스템 플러그인이 기본 내장되어 있다.

**플러그인 1: MulticopterMotorModel**
- 역할: 로터(프로펠러) 하나에 실제 물리 추력(Thrust Force)과 토크(Torque)를 적용
- 입력: 모터 속도 명령 (`gz.msgs.Actuators` 메시지, 토픽: `/{model}/gazebo/command/motor_speed`)
- 로터 4개에 각각 인스턴스 생성 (0번, 1번 CCW / 2번, 3번 CW)
- Gazebo Fuel의 X3 UAV 모델에 공식 예제가 있음 (`examples/worlds/quadcopter.sdf`)

**주요 물리 파라미터 (공식 예제 기준):**

| 파라미터 | 값 | 설명 |
|---------|-----|------|
| maxRotVelocity | 800.0 rad/s | 최대 회전 속도 |
| motorConstant | 8.54858e-06 | 추력 계수 (모터 특성) |
| momentConstant | 0.016 | 반작용 토크 계수 |
| timeConstantUp | 0.0125 s | 출력 상승 시간 상수 |
| timeConstantDown | 0.025 s | 출력 하강 시간 상수 |
| rotorDragCoefficient | 8.06428e-05 | 공기 저항 계수 |

> **용어 설명**: "시간 상수(Time Constant)"란 모터가 목표 RPM의 63%에 도달하는 데 걸리는 시간이다. 실제 모터는 관성 때문에 즉각 반응하지 않는데, 이 값이 클수록 드론이 더 천천히 반응한다.

**플러그인 2: MulticopterVelocityControl**
- 역할: Twist 메시지를 받아 4개 모터 속도를 자동 계산하는 속도 제어기
- 입력 토픽: `/{robotNamespace}/cmd_vel` (타입: `gz.msgs.Twist`)
- 출력: 내부적으로 `components::Actuators` 컴포넌트를 업데이트 → MulticopterMotorModel이 읽음
- 기반 알고리즘: RotorS의 LeePositionController에서 영감을 받은 PID 제어기

> **핵심 포인트**: MulticopterVelocityControl의 commandSubTopic 파라미터 기본값이 `"cmd_vel"`이다. 즉, `ros_gz_bridge`로 ROS 2의 `geometry_msgs/msg/Twist`를 브릿지하면, 기존 UGV의 `cmd_vel`과 **동일한 ROS 2 인터페이스**로 드론을 제어할 수 있다.

**ros_gz_bridge 브릿지 설정 예시:**
```yaml
# bridge.yaml
- ros_topic_name: /drone/cmd_vel
  gz_topic_name: /drone/cmd_vel
  ros_type_name: geometry_msgs/msg/Twist
  gz_type_name: gz.msgs.Twist
  direction: ROS_TO_GZ
```

**알려진 이슈**: 2024년 11월에 Ubuntu 24.04 + Gazebo Harmonic에서 MulticopterMotorModel 세그폴트(segfault) 버그가 보고되었다. 최신 gz-sim8 패치 버전 적용 필수. `sudo apt upgrade gz-sim8` 후 재시도 권장.

---

#### Option B: PX4 SITL + Gazebo Harmonic

**동작 방식:**
1. `make px4_sitl gz_x500` 명령으로 PX4 SITL 인스턴스 시작
2. PX4 내부에서 Gazebo 시뮬레이션 소켓으로 센서/액추에이터 데이터 교환
3. uXRCE-DDS 에이전트를 별도 실행하여 PX4 ↔ ROS 2 토픽 브릿지
4. PX4 offboard 모드로 `/fmu/in/trajectory_setpoint` 토픽에 목표 위치 발행

**장점:**
- 실제 PX4 비행 제어기와 동일한 알고리즘 (현실적 비행 특성)
- MAVLink/QGroundControl 연동 (모니터링·지상국 연동 가능)
- x500, x500_camera 등 다양한 기체 모델 공식 지원

**단점:**
- PX4 빌드 자체가 복잡 (ROS 2 Jazzy + Ubuntu 24.04 조합에서 Known Bug 존재: Issue #24159)
- 추가 프로세스: PX4 SITL 데몬 + uXRCE-DDS 에이전트 + QGroundControl
- WSL2에서 네트워크 소켓 통신 불안정 가능성
- ROS 2 인터페이스가 PX4 전용 (`px4_msgs` 패키지 의존)

---

#### Option C: ArduPilot SITL + Gazebo Harmonic

**동작 방식:**
1. `ardupilot_gazebo` 플러그인 빌드 (`GZ_VERSION=harmonic`)
2. ArduPilot SITL과 Gazebo가 UDP 소켓으로 통신
3. MAVROS 또는 ArduPilot-ROS2 브릿지로 ROS 2 연동

**장점:**
- ArduPilot이 Gazebo Harmonic을 공식 지원 (Garden/Harmonic 지원 명시)
- PX4보다 설정이 다소 유연

**단점:**
- Ubuntu 24.04 호환성 명시적 미언급 (Ubuntu 22.04까지만 확인)
- ROS 2 연동이 PX4 대비 복잡 ("ArduPilot Gazebo 플러그인은 ROS에 의존하지 않는다"고 문서 명시 → 별도 브릿지 구현 필요)
- 커스텀 플러그인 빌드 단계 필수 (바이너리 배포 없음)

---

### 2. 드론 SDF/URDF 구조

#### Gazebo Harmonic 공식 예제 기반 쿼드콥터 SDF 구조

```
quadcopter.sdf
├── world
│   ├── plugin: Physics (gz-sim-physics-system)
│   ├── plugin: SceneBroadcaster (gz-sim-scene-broadcaster-system)
│   ├── plugin: UserCommands (gz-sim-user-commands-system)
│   └── plugin: Sensors (gz-sim-sensors-system, renderer=ogre2)
└── model: X3 UAV
    ├── link: base_link (동체)
    ├── link: rotor_0 (CCW) ← MulticopterMotorModel
    ├── link: rotor_1 (CCW) ← MulticopterMotorModel
    ├── link: rotor_2 (CW)  ← MulticopterMotorModel
    ├── link: rotor_3 (CW)  ← MulticopterMotorModel
    ├── plugin: MulticopterMotorModel x4 (로터별 1개)
    └── plugin: MulticopterVelocityControl (속도→모터속도 변환)
```

**프로펠러 모델링 방식:**
- 회전 조인트(Revolute Joint) + MulticopterMotorModel 조합
- 플러그인이 조인트에 추력 힘(Force)을 직접 인가
- 가시적 회전 애니메이션도 포함 (시각화 목적)

**ARGOS 스타일 xacro 모듈화 가능성:**
SDF는 xacro를 직접 지원하지 않지만, `sdformat_urdf` 라이브러리를 사용하면 URDF ↔ SDF 변환이 가능하다. 또는 URDF(xacro)로 작성 후 Gazebo의 `urdf2sdf` 변환을 활용할 수 있다.

**권장 센서 구성 (ARGOS 드론용):**

| 센서 | SDF 타입 | 역할 | 필요 여부 |
|-----|--------|------|--------|
| 하방 카메라 | `camera` | 하방 정찰 영상 | 필수 |
| IMU | `imu` | 자세 추정 | 필수 |
| GPS | `navsat` | 실외 위치 추정 | 선택 |
| Odometry | `odometry` (플러그인) | 내부 위치 추정 | 선택 |
| 전방 RGB-D | `rgbd_camera` | 장애물 탐지 | 추후 |

---

### 3. ROS 2 비행 제어 연동

#### MulticopterVelocityControl + ros_gz_bridge 방식 (권장)

```
ROS 2 오케스트레이터
    ↓ geometry_msgs/msg/Twist
ros_gz_bridge (parameter_bridge)
    ↓ gz.msgs.Twist  [토픽: /argos_drone_0/cmd_vel]
MulticopterVelocityControl (Gazebo 플러그인)
    ↓ 모터 속도 계산 (PID)
MulticopterMotorModel x4 (Gazebo 플러그인)
    ↓ 물리 추력 인가
Gazebo 물리 엔진 (드론 비행)
```

**ROS 2에서 드론에 cmd_vel 발행 예시:**
```python
from geometry_msgs.msg import Twist

# 상승
twist = Twist()
twist.linear.z = 1.0   # m/s 상승
drone_pub.publish(twist)

# 전진
twist.linear.x = 0.5   # m/s 전진
twist.linear.z = 0.0   # 고도 유지
drone_pub.publish(twist)
```

> **주의**: MulticopterVelocityControl은 속도 명령을 **바디 프레임(Body Frame)** 기준으로 받는다. linear.x=전진, linear.y=좌우, linear.z=상승/하강, angular.z=요(Yaw) 회전.

#### Nav2와 3D 네비게이션 관계

**결론: Nav2는 드론에 직접 사용하기 어렵다.**

Nav2는 2D 평면(x,y) 기반 네비게이션 스택이다. 3D 네비게이션 지원은 커뮤니티의 "스트레치 골(Stretch Goal)"로만 언급되며, 현재 공식 지원하지 않는다.

**드론 경로 계획 대안:**

| 방법 | 설명 | 복잡도 |
|-----|------|-------|
| 웨이포인트 직접 발행 | 오케스트레이터가 cmd_vel 시퀀스 직접 계산 | 낮음 (MVP) |
| nav_drone 패키지 | Nav2 기반 3D 드론 네비게이션 커뮤니티 패키지 | 중간 |
| PX4 offboard | `/fmu/in/trajectory_setpoint`로 3D 웨이포인트 | 높음 |
| 커스텀 경로 플래너 | 3D 그리드맵 + A* | 높음 (MS-9이후) |

**ARGOS MS-8 권장**: 오케스트레이터가 드론에게 "웨이포인트 좌표"를 전달하면, 드론 전용 노드가 현재 위치와 차이를 계산하여 cmd_vel을 발행하는 단순 P 제어기를 구현. 이는 3D SLAM 없이도 동작한다.

---

### 4. 이종 로봇 통합 패턴

#### ARGOS 오케스트레이터 통합 설계안

기존 오케스트레이터가 UGV를 지휘하는 인터페이스를 그대로 활용하되, 드론 전용 capabilites를 추가한다.

**로봇 capabilities 정의:**

```python
# 기존 UGV
UGV_CAPABILITIES = {
    "ground_explore": True,   # 지상 탐색
    "thermal_imaging": True,  # 열화상 촬영
    "firefighting": True,     # 소화 지원
    "max_speed": 1.0,         # m/s
    "dimension": "3D-limited" # 계단 불가
}

# 신규 드론
DRONE_CAPABILITIES = {
    "aerial_recon": True,     # 공중 정찰
    "downward_camera": True,  # 하방 카메라
    "gps_navigation": True,   # GPS 기반 이동
    "max_altitude": 20.0,     # m
    "dimension": "3D-full"    # 3D 자유 이동
}
```

**통합 인터페이스 패턴 (권장: 공통 추상화):**

```
오케스트레이터 (Orchestrator Node)
    ├── UGV 에이전트 노드 (/argos_ugv_0)
    │       └── cmd_vel: geometry_msgs/msg/Twist
    │       └── odom:    nav_msgs/msg/Odometry
    │       └── status:  argos_interfaces/msg/RobotStatus
    │
    └── 드론 에이전트 노드 (/argos_drone_0)
            └── cmd_vel: geometry_msgs/msg/Twist  ← 동일 인터페이스!
            └── pose:    geometry_msgs/msg/PoseStamped
            └── status:  argos_interfaces/msg/RobotStatus  ← 동일 메시지
```

> **설계 핵심**: UGV와 드론이 **동일한 cmd_vel 인터페이스**를 사용하면, 오케스트레이터 코드 수정 없이 드론을 추가할 수 있다. 차이점은 드론이 linear.z(고도)를 추가로 제어한다는 것뿐이다.

**임무 할당 흐름:**

```
화재 감지 이벤트
    ↓
오케스트레이터: "외부 정찰 필요"
    ├── aerial_recon capability 보유 로봇 선택 → 드론
    └── ground_explore capability 보유 로봇 선택 → UGV

드론 임무: 건물 외부 2층 높이에서 열화상 촬영
UGV 임무: 1층 내부 탐색, 요구조자 위치 파악
```

**드론 위치 추정:**
드론은 SLAM 없이 **IMU + Odometry 기반 Dead Reckoning** 또는 **GPS**를 사용한다. Gazebo에서는 `odometry` 플러그인이 완벽한 위치를 제공하므로 MVP에서는 Ground Truth 사용 가능.

---

### 5. 가장 간단한 MVP 접근법

#### 3단계 MVP 구현 계획

**Phase 1 (MS-8 MVP): Gazebo 내장 플러그인만 사용**
- 목표: Gazebo에서 드론이 날아다니고, ROS 2에서 제어 가능한 상태
- 드론 기능: 이착륙, 고도 유지, 수평 이동, 하방 카메라
- 제어: cmd_vel (Twist) 직접 발행
- 위치: Ground Truth Odometry (완벽한 위치 정보)

**Phase 2 (MS-9): 오케스트레이터 통합**
- 드론 에이전트 노드 구현 (오케스트레이터 ↔ 드론 인터페이스)
- capabilities 기반 임무 할당 추가
- UGV + 드론 동시 임무 시연

**Phase 3 (MS-10+): 고급 기능**
- 웨이포인트 자동 경로 추종
- 드론 → UGV 정보 릴레이 (공중에서 발견한 요구조자 위치 전달)
- (선택) GPS 기반 실외 시뮬레이션

---

## 비교 분석

| 항목 | Option A (내장 플러그인) | Option B (PX4 SITL) | Option C (ArduPilot SITL) |
|-----|-------------------|----------------|-------------------|
| **설정 복잡도** | 낮음 | 높음 | 중간 |
| **추가 의존성** | 없음 (gz-sim8 기본) | PX4 빌드, uXRCE-DDS | ardupilot_gazebo 빌드 |
| **ROS 2 인터페이스** | geometry_msgs/Twist (표준) | px4_msgs (전용) | MAVROS (복잡) |
| **WSL2 성능 영향** | 최소 | 높음 (+3~4 프로세스) | 중간 |
| **Ubuntu 24.04 지원** | 공식 지원 | Known Bug 존재 | 미확인 |
| **비행 현실성** | 중간 (간단한 PID) | 높음 (실제 PX4) | 높음 (실제 ArduPilot) |
| **ARGOS 통합 용이성** | 높음 (표준 인터페이스) | 낮음 (전용 메시지) | 낮음 (별도 브릿지) |
| **MVP 구현 시간** | 1~2일 | 3~5일 | 3~4일 |

---

## 권장 사항

### 권장 MVP 구현 방법

**Gazebo 내장 플러그인 방식 (Option A)** 으로 구현한다. 이유:

1. **ARGOS 아키텍처 적합성**: 기존 UGV의 cmd_vel 인터페이스와 완전히 동일 → 오케스트레이터 변경 최소화
2. **WSL2 RTX 4050 성능**: PX4 SITL 추가 없이 Gazebo 단독으로 동작 → 리소스 여유
3. **Ubuntu 24.04 호환**: gz-sim8 기본 내장이므로 별도 빌드 불필요
4. **개발 속도**: 복잡한 SITL 설정 없이 SDF 파일 + 브릿지 설정만으로 완성

### 필요한 패키지/의존성

```bash
# 이미 설치된 것 (ROS 2 Jazzy + Gazebo Harmonic 환경)
# gz-sim8 (gz-harmonic) → MulticopterMotorModel, MulticopterVelocityControl 내장
# ros-jazzy-ros-gz-bridge → Twist 브릿지
# ros-jazzy-ros-gz-sim → Gazebo ↔ ROS 2 연동

# 추가 설치 필요
sudo apt install ros-jazzy-actuator-msgs   # Actuators 메시지 타입 (모터 직접 제어 시)
```

**검증 명령 (내장 플러그인 확인):**
```bash
gz sim --gui examples/worlds/quadcopter.sdf
```

### 구현 순서

1. **[Day 1] 드론 SDF 작성**
   - Gazebo Fuel에서 X3 UAV 모델 기반으로 ARGOS 드론 SDF 작성
   - 하방 카메라 센서 추가
   - MulticopterMotorModel + MulticopterVelocityControl 플러그인 설정

2. **[Day 1] ros_gz_bridge 연동**
   - `bridge.yaml`: `/argos_drone_0/cmd_vel` (Twist) 브릿지 설정
   - `/argos_drone_0/pose` (PoseStamped) 브릿지 설정 (위치 피드백)
   - Launch 파일에 bridge 노드 추가

3. **[Day 2] 드론 에이전트 노드 작성**
   - 오케스트레이터로부터 웨이포인트 수신 (`geometry_msgs/msg/PoseStamped`)
   - 현재 위치 → 목표 위치 차이로 cmd_vel 계산 (P 제어기)
   - 이착륙 서비스 (`argos_interfaces/srv/DroneCommand`) 구현

4. **[Day 2] 세계 SDF 통합**
   - 기존 UGV 2대 + 드론 1대를 포함하는 world.sdf 작성
   - 각 모델 네임스페이스 분리 (namespace: argos_ugv_0, argos_ugv_1, argos_drone_0)

5. **[Day 3] 오케스트레이터 capabilities 확장**
   - 드론의 capabilities 등록 (aerial_recon, downward_camera)
   - 임무 할당 로직에 드론 포함

6. **[Day 3] 통합 테스트**
   - UGV 2대 + 드론 1대 동시 구동 성능 측정
   - 오케스트레이터가 드론에 정찰 임무 할당 → 드론이 지정 위치로 이동하는 시나리오 검증

---

## 성능 전망 (RTX 4050 노트북, WSL2)

WSL2 + Gazebo Harmonic 환경에서 일반적으로:
- 데스크톱: 6~10배 실시간 속도
- 노트북: 3~4배 실시간 속도

UGV 2대 + 드론 1대 구성에서 내장 플러그인 방식은 PX4 SITL 대비 CPU 30~40% 절감 예상. WSL2에서 Gazebo Garden/Harmonic이 WSLg GPU 가속을 지원하므로, RTX 4050의 GPU 렌더링을 활용한다.

단, VmmemWSL 프로세스가 CPU 70%, 메모리 70% 점유하는 기존 WSL2 이슈는 여전히 존재. 시뮬레이션 실행 전 `wsl --shutdown` 후 재시작으로 메모리 정리 권장.

---

## 출처

- [gz-sim: MulticopterVelocityControl 공식 문서](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1MulticopterVelocityControl.html) — 플러그인 파라미터, 구독 토픽, 내부 동작 구조
- [gz-sim8: quadcopter.sdf 공식 예제](https://github.com/gazebosim/gz-sim/blob/gz-sim8/examples/worlds/quadcopter.sdf) — X3 UAV 완전한 SDF 구조, 물리 파라미터
- [MulticopterVelocityControl 소스 코드 (gz-sim8)](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/multicopter_control/MulticopterVelocityControl.cc) — Twist 메시지 구독, 모터 속도 계산 알고리즘
- [PX4 Gazebo Harmonic 공식 문서](https://docs.px4.io/main/en/sim_gazebo_gz/) — PX4 SITL 연동 방법, Ubuntu 24.04 호환성
- [ArduPilot SITL with Gazebo](https://ardupilot.org/dev/docs/sitl-with-gazebo.html) — ArduPilot Gazebo Harmonic 플러그인 빌드 방법
- [PX4 + Ubuntu 24.04 + ROS2 Jazzy Known Bug](https://github.com/PX4/PX4-Autopilot/issues/24159) — 호환성 이슈 트래킹
- [ros_gz_bridge Jazzy 문서](https://docs.ros.org/en/jazzy/p/ros_gz_bridge/) — ROS 2 ↔ Gazebo 토픽 브릿지 설정
- [Gazebo Harmonic Discourse: MulticopterMotorModel 문제 해결](https://discourse.openrobotics.org/t/gazebo-harmonic-plug-in-for-motor-control/48171) — ros_gz_bridge 브릿지 타입 매핑 해결법
- [Toward a Generic Framework for Mission Planning with Heterogeneous Multi-Robot System](https://pmc.ncbi.nlm.nih.gov/articles/PMC11548481/) — 이종 군집 임무 계획 아키텍처 참고 논문
- [A Modular and Scalable System Architecture for Heterogeneous UAV Swarms Using ROS 2](https://arxiv.org/html/2510.27327v1) — ROS 2 기반 이종 UAV 군집 시스템 설계 패턴
- [WSLg GPU support for Gazebo Garden/Harmonic](https://discourse.openrobotics.org/t/wslg-with-gpu-support-available-on-latest-version-of-gazebo-garden-and-harmonic/48128) — WSL2에서 GPU 가속 지원 현황
- [sjtu_drone ROS 2 쿼드콥터 시뮬레이터](https://github.com/NovoG93/sjtu_drone) — 대안 경량 드론 시뮬레이터 (Gazebo 11 기반, Harmonic 미지원)
