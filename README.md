# ARGOS

> **A**utonomous **R**obot **G**roup **O**rchestration **S**ystem

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange)
![License](https://img.shields.io/badge/License-Apache_2.0-green)
![Robots](https://img.shields.io/badge/Robots-UGV_×3_+_Drone_×2_+_Sherpa-purple)
![Tests](https://img.shields.io/badge/Tests-785_passed-brightgreen)
![Portal](https://img.shields.io/badge/Portal-daegufire.ai.kr/argos-teal)
![Code](https://img.shields.io/badge/Code-15K+_lines-informational)

이종 군집 소방 로봇 오케스트레이션 플랫폼.
드론 + UGV(차량형) + 보행형 로봇이 **한 팀**으로 화재 현장을 자율 탐색하고, 화점을 감지하며, 실시간으로 정보를 공유하는 시스템.

현직 소방관이 실제 소방 현장 운영 경험을 바탕으로 설계한 **Supervised Autonomy** 아키텍처 — 오케스트레이터가 "무엇을" 결정하고, 각 로봇이 "어떻게"를 자율 수행합니다.

**[HEAT Portal](https://daegufire.ai.kr/argos)** 에서 실시간 모니터링 + 인터랙티브 전술 체험이 가능합니다.

## Highlights

- **이종 군집 로봇**: UGV 3대 + 드론 2대 + HR-셰르파(소방로봇) — CBBA 경매 기반 최적 임무 할당
- **자율 프론티어 탐색**: SLAM 맵에서 미탐색 경계를 자동 감지, 최적 프론티어로 이동
- **열화상 화점 감지**: L8 카메라 시뮬레이션 + 적응형 임계값 기반 화점 분류 (low/medium/high/critical)
- **중앙 지휘 시스템**: 오케스트레이터가 로봇 등록, 임무 할당, 긴급 정지, 단계 전환을 관리
- **HEAT Portal 통합**: [daegufire.ai.kr/argos](https://daegufire.ai.kr/argos)에서 실시간 모니터링 + 인터랙티브 제어
- **인터랙티브 전술 체험**: 맵 클릭 화재 생성, 로봇 개별 이동, 4종 진압 전술(직선/측방/포위/릴레이), 4종 시나리오 프리셋
- **소방서 PC 호환**: Python 독립 서버로 WSL/ROS2 없이도 시뮬레이션 실행 가능
- **소방 시나리오 자동 시연**: 7단계 상태 머신으로 이륙→탐색→화재감지→긴급정지→재개→착륙 자동 실행
- **8중 센싱 체계**: 가스(CO/O2/LEL/CO2/HCN) + 피해자(열화상 인체) + 구조물(LiDAR 변위) + 음향(8종)
- **LifecycleNode 동기 제어**: 핵심 4노드 lifecycle 전환 — lifecycle_manager로 클린 시작/종료 보장
- **MCP 로봇 서버**: Claude/MCP 클라이언트에서 자연어로 로봇 제어 가능 (SkillLibrary 동적 쿼리)
- **지휘관 승인 워크플로우**: PAUSED 상태에서 `/orchestrator/resume` 명시 승인 후 RETURNING 전환
- **HR-셰르파 소방로봇**: NFRI 공식 제원(3.1×2.0×1.9m, 6WD, 2650LPM) 기반 URDF + 호스 물리 모델 (100m 릴, 꺾임 감지, 충수 후진 금지)
- **CBBA 태스크 할당**: Consensus-Based Bundle Algorithm — 이종 로봇 capabilities 기반 경매 할당, 번들 크기 N 지원
- **드론→UGV 화재 핸드오프**: 드론 열화상 감지 → FireAlert → CBBA → UGV 진압 + 드론 감시 자동 배정
- **PX4 드론 통합**: uXRCE-DDS Agent 연결, PlatformInterface 추상화로 UGV/드론/셰르파 통합 제어
- **편대 패턴**: 횡대/종대/제대/포위 4종 편대 + 충돌 검사
- **YOLOv8 화재 탐지**: D-Fire+SYN-FIRE 혼합 학습 (mAP50=0.718, 실데이터 검증, AGPL 분리 패키지)

## Experimental Results

| Metric | Value | Note |
|--------|-------|------|
| Unit Tests | **813 passed** | 37 test files, pure Python |
| Nav2 Goal | **SUCCEEDED** | SLAM 311×217 map, 0.50m accuracy |
| PX4↔ROS2 | **24 topics connected** | uXRCE-DDS Agent colcon build |
| YOLOv8 mAP50 | **0.718** | D-Fire 14K + SYN-FIRE 2K mixed |
| FLAME Diffuser | **1,000 images** | SD v1.5, 339MB |
| Sherpa Spawn | **Entity creation successful** | 875-line URDF, 6WD |
| UGVPlatform | **move_to 1.068m** | Nav2 → MPPI → Gazebo |
| Code | **15,176 lines** | 126 Python files |

## Architecture

```
┌─────────────────────────────────────────────────────┐
│                   Web Dashboard                      │
│     robot trail · click detail · stage timeline      │
│              (rosbridge + roslibjs)                   │
└───────────────────────┬─────────────────────────────┘
                        │ WebSocket :9090
┌───────────────────────┴─────────────────────────────┐
│               Orchestrator Node                      │
│   ┌─────────┬──────────┬──────────┬──────────┐      │
│   │ Robot   │ Mission  │ Fire     │ Emergency│      │
│   │Registry │ Stage    │ Response │ Stop     │      │
│   │         │ Machine  │ Escalate │          │      │
│   └─────────┴──────────┴──────────┴──────────┘      │
│   Deadline QoS 5s · Heartbeat 10s · Fire Expiry 5m  │
└──────┬──────────────┬──────────────┬────────────────┘
       │              │              │
  ┌────┴────┐   ┌─────┴─────┐  ┌────┴─────┐
  │  UGV 1  │   │   UGV 2   │  │  Drone 1 │
  │(argos1) │   │ (argos2)  │  │ (drone1) │
  ├─────────┤   ├───────────┤  ├──────────┤
  │Nav2+SLAM│   │Nav2+SLAM  │  │Waypoint  │
  │Frontier │   │Frontier   │  │P+Yaw     │
  │Explorer │   │Explorer   │  │Controller│
  │Hotspot  │   │Hotspot    │  │          │
  │Detector │   │Detector   │  │          │
  │Status   │   │Status     │  │Status    │
  │Publisher│   │Publisher  │  │Publisher │
  └─────────┘   └───────────┘  └──────────┘
       │              │              │
  ┌────┴──────────────┴──────────────┴────┐
  │          Gazebo Harmonic               │
  │   indoor_test world · ros_gz bridge    │
  └────────────────────────────────────────┘
```

### Node Communication

```
                    /orchestrator/mission_state (MissionState, 2Hz)
                              ▲
                    ┌─────────┴─────────┐
                    │   Orchestrator    │◄──── /orchestrator/emergency_stop (Trigger)
                    │                   │◄──── /orchestrator/resume (Trigger)
                    └──┬──────┬─────┬──┘
                       │      │     │
     robot_status ─────┘      │     └───── fire_alert
     (Deadline QoS 5s)        │            (RELIABLE + TRANSIENT_LOCAL)
                              │
          ┌───────────────────┼───────────────────┐
          │                   │                   │
    ┌─────┴──────┐     ┌─────┴──────┐     ┌──────┴─────┐
    │ RobotStatus│     │ RobotStatus│     │ RobotStatus│
    │  (argos1)  │     │  (argos2)  │     │  (drone1)  │
    └─────┬──────┘     └─────┬──────┘     └──────┬─────┘
          │                  │                    │
    ┌─────┴──────┐     ┌─────┴──────┐     ┌──────┴─────┐
    │ Frontier   │     │ Frontier   │     │   Drone    │
    │ Explorer   │     │ Explorer   │     │ Controller │
    └─────┬──────┘     └─────┬──────┘     └────────────┘
          │                  │
    ┌─────┴──────┐     ┌─────┴──────┐
    │  Hotspot   │     │  Hotspot   │
    │ Detector   │     │ Detector   │
    └────────────┘     └────────────┘
```

### 4-Layer Design

| Layer | Role | Implementation |
|-------|------|----------------|
| **Orchestrator** | 임무 할당, 상황 종합, 전략 판단 | `orchestrator_node.py` |
| **Mission** | 정찰 / 진압보조 / 구조지원 / 위험물 | `scenario_runner_node.py` |
| **Core Services** | 위치추정 · 경로계획 · 군집통신 · 상태관리 | Nav2, slam_toolbox, TF2, ros_gz_bridge |
| **Platform** | Drone / UGV / Legged — 동일 인터페이스 | URDF/SDF + ros2_control |

## Tech Stack

| Component | Technology |
|-----------|-----------|
| ROS | **ROS 2 Jazzy** (Ubuntu 24.04 / WSL2) |
| Simulator | **Gazebo Harmonic** + ros_gz bridge |
| Navigation | **Nav2** (MPPI Controller + SmacPlanner2D) |
| SLAM | **slam_toolbox** (online async) |
| Drive | **ros2_control** + diff_drive_controller (4WD skid-steer) |
| Drone | Gazebo MulticopterVelocityControl + P controller |
| Vision | Thermal L8 camera (mono8) + OpenCV contour detection |
| Web | **rosbridge_server** + roslibjs (WebSocket) |
| Model | URDF/xacro (3-layer modular: platform/sensors/control) |

## Prerequisites

```bash
# 환경 확인
wsl --list --verbose        # WSL2 Ubuntu 24.04 확인
lsb_release -a              # Ubuntu 24.04 확인
ros2 --version              # ROS 2 Jazzy 확인 (0.20.x)
```

```bash
# Ubuntu 24.04 (WSL2 or native)
# ROS 2 Jazzy (https://docs.ros.org/en/jazzy/Installation.html)

sudo apt install \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-controller-manager \
  ros-jazzy-diff-drive-controller \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-topic-tools \
  ros-jazzy-rosbridge-server \
  python3-opencv python3-numpy
```

## Quick Start

```bash
# 1. 클론 + 빌드
git clone https://github.com/Jamonnine/argos.git
cd argos/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

# 2. 로봇 모델 확인 (RViz)
ros2 launch argos_description display.launch.py

# 3. 단일 로봇 시뮬레이션
ros2 launch argos_description gazebo.launch.py

# 4. 네비게이션 + SLAM + 열화상
ros2 launch argos_description navigation.launch.py

# 5. 자율 프론티어 탐색
ros2 launch argos_description navigation.launch.py explore:=true

# 6. 이종 군집 탐색 (UGV 2 + Drone 1)
ros2 launch argos_description exploration.launch.py

# 7. 전체 소방 시나리오 시연 (rosbridge + 웹 대시보드 포함)
ros2 launch argos_description demo.launch.py
# 별도 터미널에서 웹 서버:
cd $(ros2 pkg prefix argos_description)/share/argos_description/web
python3 -m http.server 8080
# 브라우저: http://localhost:8080
```

### Launch Files

| Launch | Description | Robots |
|--------|-------------|--------|
| `display.launch.py` | RViz에서 URDF 모델 확인 | 1 UGV |
| `gazebo.launch.py` | Gazebo에 로봇 스폰 + 컨트롤러 | 1 UGV |
| `navigation.launch.py` | Nav2 + SLAM + 열화상 + (선택) 프론티어 탐색 | 1 UGV |
| `multi_robot.launch.py` | 멀티로봇 네임스페이스 분리 스폰 | N UGV |
| `exploration.launch.py` | 이종 군집 자율 탐색 + 오케스트레이터 | 2 UGV + 1 Drone |
| `demo.launch.py` | 전체 스택 + 시나리오 러너 + rosbridge | 2 UGV + 1 Drone |
| `monitor.launch.py` | rosbridge 단독 실행 (기존 시뮬에 연결) | — |

## Project Structure

```
argos/
├── ros2_ws/src/
│   ├── argos_bringup/              # 노드 20개, launch 10개, 테스트 511개
│   │   └── argos_bringup/
│   │       ├── frontier_explorer_node.py   # 프론티어 자율 탐색 (LifecycleNode)
│   │       ├── hotspot_detector_node.py    # 열화상 화점 감지 (LifecycleNode)
│   │       ├── orchestrator_node.py        # 중앙 지휘 시스템 (LifecycleNode)
│   │       ├── robot_status_node.py        # 상태 보고
│   │       ├── drone_controller_node.py    # 드론 웨이포인트 비행
│   │       ├── scenario_runner_node.py     # 시나리오 자동 실행
│   │       ├── perception_bridge_node.py   # AI → Nav2 브릿지
│   │       ├── gas_sensor_node.py          # 가스 센서 (CO/O2/LEL/CO2/HCN, LifecycleNode)
│   │       ├── situation_assessor_node.py  # compute_situation_score() 5센서 퓨전
│   │       ├── acoustic_detector_node.py   # 음향 탐지 8종
│   │       ├── structural_monitor_node.py  # LiDAR 구조물 변위 감지
│   │       ├── kalman_fire_tracker_node.py # Kalman 화점 시계열 추적 + 확산 추세
│   │       ├── skill_library.py            # 로봇 능력 동적 등록/쿼리/MCP 변환
│   │       └── mcp_robot_server.py         # MCP 로봇 서버 (자연어 로봇 제어)
│   │
│   ├── argos_description/          # 로봇 모델 + 시뮬레이션 설정
│   │   ├── urdf/                   # URDF/xacro (base + sensors + control)
│   │   ├── models/argos_drone/     # 드론 SDF 모델
│   │   ├── launch/                 # 10개 launch 파일
│   │   ├── config/                 # Nav2 + controllers 파라미터
│   │   ├── worlds/                 # Gazebo indoor_test 월드
│   │   └── web/                    # 웹 대시보드 (index.html)
│   │
│   ├── argos_interfaces/           # 커스텀 메시지/서비스/액션 (msg 11 + srv 1 + action 2)
│   │   ├── msg/                    # RobotStatus, MissionState, FireAlert, ThermalDetection,
│   │   │                           # DetectedObject, GasSensorReading, SituationScore
│   │   │                           # AcousticEvent, StructuralAlert, KalmanFireTrack,
│   │   │                           # SkillCapability
│   │   ├── srv/                    # SetStage, NavigateToObject
│   │   └── action/                 # PatrolArea, ExploreArea
│   │
│   └── argos_fire_ai/              # YOLOv8 화재 탐지 (AGPL 분리 패키지)
│
├── scripts/                        # 유틸리티 (진단, Gazebo, SLAM, Nav2)
└── docs/                           # 가이드 문서
```

## Nodes

| Node | Description | Key Topics |
|------|-------------|------------|
| `frontier_explorer` | OccupancyGrid 프론티어 감지 → Nav2 자율 이동. 멀티로봇 대상 회피 + 열화상 일시정지. Thread-safe lock. **LifecycleNode** | `map` → `navigate_to_pose` |
| `hotspot_detector` | L8 열화상 mono8 적응형 임계값 화점 감지. 프레임당 worst-only + 1초 쿨다운. **LifecycleNode** | `thermal/image_raw` → `thermal/detections` |
| `orchestrator` | 로봇 등록, heartbeat(10s), Deadline QoS(5s), 자동 단계 전환, 화재 에스컬레이션, 배터리 자동귀환. **지휘관 승인(PAUSED→resume→RETURNING)**. **LifecycleNode** | `/orchestrator/mission_state` |
| `robot_status_publisher` | TF2 위치 + 탐색 상태 + 배터리(시간 감쇠) + 커버리지. TRANSIENT_LOCAL map QoS. **TF frame_prefix 멀티로봇 격리** | `/orchestrator/robot_status` |
| `drone_controller` | P+Yaw 제어기 웨이포인트 비행. 큐 기반 경로. 이륙/착륙/호버링 서비스 | `odom` → `cmd_vel` |
| `scenario_runner` | 7단계 상태 머신 소방 시나리오. 페이즈별 상대 시간 관리 | service clients |
| `perception_bridge` | AI 감지 결과 캐시 → Nav2 목표 변환. 비블로킹 서버 체크 | `/detections` → `navigate_to_pose` |
| `gas_sensor` | 가스 센서 퓨전 (CO/O2/LEL/CO2/HCN). 위험 임계값 초과 시 경보. **LifecycleNode** | `/gas/readings` → `/gas/alert` |
| `situation_assessor` | `compute_situation_score()` — 5개 센서 가중 합산으로 현장 위험도 종합 평가 | 다수 센서 토픽 → `/situation/score` |
| `acoustic_detector` | 음향 탐지 8종 (비명/폭발/붕괴 등). BackPressure LATEST 모드 스트리밍 | `audio/raw` → `/acoustic/events` |
| `structural_monitor` | LiDAR 포인트클라우드 변위 분석 → 구조물 붕괴 위험 조기 감지 | `/lidar/points` → `/structural/alert` |
| `kalman_fire_tracker` | Kalman 필터로 화점 위치 시계열 추적. 확산 속도·방향 추세 판정 | `/fire_alert` → `/fire/tracked` |
| `skill_library` | 로봇 능력 동적 등록/쿼리. `SkillCapability` → MCP 변환으로 Claude에서 직접 호출 | ROS2 서비스 |
| `mcp_robot_server` | MCP 프로토콜 로봇 제어 서버. 자연어 명령 → SkillLibrary → ROS2 액션/서비스 변환 | MCP ↔ ROS2 브릿지 |

## Custom Interfaces

```
# .msg (11개)
RobotStatus.msg       — 로봇 상태 (ID, type, state, pose, battery, coverage, capabilities, nav_error_count)
MissionState.msg      — 임무 상태 (stage, robot counts, coverage, fire locations, primary_responder)
FireAlert.msg         — 화점 알림 (robot_id, location, temperature, severity, active)
ThermalDetection.msg  — 열화상 감지 (temperature, bbox, centroid, severity, area_ratio)
DetectedObject.msg    — AI 감지 객체 (class, confidence, pose, distance)
GasSensorReading.msg  — 가스 센서 측정값 (CO, O2, LEL, CO2, HCN, timestamp)
SituationScore.msg    — 현장 위험도 종합 점수 (score, breakdown, recommendation)
AcousticEvent.msg     — 음향 이벤트 (event_type, confidence, location_estimate)
StructuralAlert.msg   — 구조물 경보 (displacement_mm, risk_level, zone_id)
KalmanFireTrack.msg   — Kalman 추적 화점 (position, velocity, spread_rate, direction)
SkillCapability.msg   — 로봇 능력 기술 (skill_id, robot_id, params_schema, mcp_tool_def)

# .srv (1개)
SetStage.srv          — 임무 단계 전환
NavigateToObject.srv  — AI 감지 객체로 이동 (argos_bringup 내부 헬퍼)

# .action (2개)
PatrolArea.action     — 구역 순찰 (waypoints → progress → result)
ExploreArea.action    — 자율 탐색 구역 지정 (boundary → coverage_progress → map_snapshot)
```

## Web Dashboard (HEAT Portal)

**[daegufire.ai.kr/argos](https://daegufire.ai.kr/argos)** 에서 실시간 임무 모니터링 + 인터랙티브 제어:

### 모니터링
- **미션 패널**: 현재 단계, 경과 시간(M:SS), 로봇/탐색률/화재 상태
- **스테이지 타임라인**: 7단계 진행 표시 (색상 코딩)
- **로봇 상태 카드**: ID, 타입, 상태, 배터리, 속도, 현재 임무
- **2D 전술 맵**: 로봇 위치(UGV 원형/드론 다이아몬드), 화점 마커, 이동 궤적(trail), 건물/기지/구역
- **화재 알림**: 심각도별 색상(위험/경고/주의), NEW 펄스 애니메이션

### 인터랙티브 제어
- **맵 클릭 화재 생성**: 원하는 위치에 화재 발생 → 로봇 자동 대응
- **로봇 개별 이동**: 로봇 선택 → 맵 클릭으로 웨이포인트 지정
- **진압 전술 선택**: 직선 접근 / 측방 우회 / 포위 진압 / 릴레이 교대
- **시나리오 프리셋**: 건물화재 / 다중화점 / 창고화재 / 수색구조
- **시뮬레이션 제어**: 속도(0.5x~4x), 통신 두절 토글, 화재 확산 토글
- **긴급정지/재개**: 전 로봇 즉시 정지 + 위치 고정
- **드론 이착륙**: 수동 이륙/착륙 명령

### 연결 방식
1. **런처 다운로드**: 포털에서 `.bat` 파일 다운로드 → 더블클릭
2. **자동 감지**: WSL+ROS2 → Gazebo 풀 시뮬 / Python만 → 독립 서버
3. **자동 연결**: rosbridge ws://localhost:9090 → 포털 자동 감지 + 연결

## Milestones

| # | Milestone | Description | Status |
|---|-----------|-------------|--------|
| 1 | PatrolArea Action | Action server/client 기본 구조 | Done |
| 2 | UGV URDF | 3계층 모듈식 xacro (platform/sensors/control) | Done |
| 3 | Nav2 + SLAM | 자율 네비게이션 + 실시간 지도 작성 | Done |
| 4 | Thermal Camera | L8 열화상 시뮬레이션 + 화점 감지 | Done |
| 5 | Multi-Robot | 네임스페이스 분리로 N대 동시 스폰 | Done |
| 6 | Exploration | 프론티어 기반 자율 탐색 (멀티로봇 조율) | Done |
| 7 | Orchestrator | 중앙 지휘 시스템 (Supervised Autonomy) | Done |
| 8 | Drone Platform | 이종 로봇 확장 (UGV + Drone) | Done |
| 9 | Demo Scenario | 소방 시나리오 7단계 자동 시연 | Done |
| 10 | Web Dashboard | rosbridge + roslibjs 실시간 모니터링 | Done |
| 11 | LifecycleNode | 핵심 4노드 Lifecycle 전환 + lifecycle_manager 동기 제어 | Done |
| 12 | 8중 센싱 체계 | 가스·피해자·구조물·음향 센서 통합 | Done |
| 13 | Sensor Fusion | compute_situation_score() 5센서 가중 합산 | Done |
| 14 | Commander Approval | PAUSED → resume 지휘관 승인 워크플로우 | Done |
| 15 | TF frame_prefix | 멀티로봇 TF 네임스페이스 격리 | Done |
| 16 | SkillLibrary | 로봇 능력 동적 등록/쿼리/MCP 변환 | Done |
| 17 | MCP Robot Server | 자연어 로봇 제어 (Claude/MCP 클라이언트 연동) | Done |
| 18 | Kalman Tracker | 화점 위치 시계열 추적 + 확산 추세 판정 | Done |
| 19 | ReactiveStream | 센서 BackPressure (LATEST/DROP/BUFFER 정책) | Done |

## Design Decisions

- **Supervised Autonomy** (DARPA SubT CERBERUS 패턴): 오케스트레이터가 할당, 로봇이 자율 실행. 통신 두절 시 독립 수행
- **네임스페이스 분리**: 모든 노드가 상대경로 토픽 사용 → launch에서 `namespace=robot_name`으로 N대 확장
- **적응형 열화상 임계값**: L8 AGC 특성상 고정 임계값 불가 → 상위 N% 적응형 방식
- **페이즈별 상대 시간**: 시나리오 러너가 각 단계의 시작 시점 기준으로 타이밍 관리
- **Deadline QoS**: 로봇 상태 5초 미수신 시 통신 두절 판정 (DDS 표준 품질 보장)
- **화재 에스컬레이션**: 대응 중 더 심각한 화재 감지 시 대응 대상 갱신
- **시간 기반 FireAlert 만료**: 5분 경과 시 자동 비활성화 → 탐색 재개
- **LifecycleNode 채택 (v2.0)**: 핵심 4노드(orchestrator/frontier_explorer/hotspot_detector/gas_sensor)를 LifecycleNode로 전환 → lifecycle_manager로 시작/정지 순서 동기화, 하드웨어 이상 시 클린 셧다운 보장
- **TF frame_prefix (v2.0)**: 로봇별 `frame_prefix` 파라미터로 TF 트리 격리 → 동일 URDF를 N대에 재사용하면서 TF 충돌 방지
- **ReactiveStream BackPressure (v2.0)**: 고주파 센서(음향, LiDAR)는 LATEST 정책으로 최신 데이터만 유지. 처리 지연 시 DROP으로 큐 포화 방지
- **SkillLibrary + MCP 이중 인터페이스 (v2.0)**: 로봇 능력을 ROS2 서비스와 MCP 툴 정의 양쪽으로 동시 노출 → 운용자(대시보드)와 AI 에이전트(Claude) 모두 동일 SkillLibrary 참조
- **AGPL 분리 패키지 (v2.0)**: YOLOv8 의존 argos_fire_ai를 별도 패키지로 분리 → 본체(Apache-2.0) 라이선스 오염 방지

## Verification

빌드 후 정상 동작 확인:

```bash
# 빌드 확인
ros2 pkg list | grep argos   # argos_bringup, argos_description, argos_interfaces

# 테스트 (511개)
cd ros2_ws/src/argos_bringup
python3 -m pytest test/ --ignore=test/test_copyright.py --ignore=test/test_flake8.py --ignore=test/test_pep257.py -q

# 노드 확인 (시뮬레이션 기동 후)
ros2 node list | grep -E 'orchestrator|frontier|gas_sensor'

# 토픽 확인
ros2 topic list | grep orchestrator
```

## Troubleshooting

| 증상 | 원인 | 해결 |
|------|------|------|
| `colcon build` 실패 | 의존성 누락 | `rosdep install --from-paths src --ignore-src -r -y` |
| Gazebo 검은 화면 | GPU 드라이버 | `export LIBGL_ALWAYS_SOFTWARE=1` 또는 headless 모드 |
| `/map` 토픽 없음 | SLAM 미시작 | `navigation.launch.py` 먼저 실행 |
| `frame "odom" does not exist` | TF 미발행 | 컨트롤러 매니저 시작 대기 (5초) |

## Roadmap

v1.0 (MS-1~10) 완료 → **v2.0 (MS-11~19) 완료** — 노드 20+개, 테스트 511개, 인터페이스 16개.

소방청 10대 전략과제(2026.01.16)의 **5번(이종 무인장비 조합)**, **4번(AI 현장지휘체계)**과 정확히 정합.

상세 로드맵, 정책 정합성 분석, 외부 생태계, 전략 타임라인: **[ROADMAP.md](ROADMAP.md)**

## References

- **DARPA SubT Challenge** — CERBERUS, CoSTAR, CSIRO (Supervised Autonomy 아키텍처 원형)
- **Open-RMF** — Heterogeneous fleet management standard
- **GLIDE** (arXiv 2509.14210) — ROS2 UAV+UGV 협업 search & rescue
- **MARL Firefighting** (Cyborg/Griffith, 2026.02) — 99.67% 소방 로봇 자율 성공률
- **Hyundai HR-Sherpa** — 무인소방로봇 100대 보급 추진 (2027~2030)
- **CMU M-TARE** — Multi-robot exploration planner
- **소방청 10대 전략과제** (2026.01.16) — AI·첨단기술 활용 미래 소방체계

## Author

**민발** — 대구 강북소방서 소방관 / 1인 개발자
AI·로보틱스 전문 소방관을 지향하며, 현장 경험을 로봇 시스템 설계에 반영합니다.

## License

Apache-2.0
