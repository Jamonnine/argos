# ARGOS API Reference

> Version: v3.7 (Phase D+) | ROS 2 Jazzy | Updated: 2026-03-23

이 문서는 ARGOS 시스템의 모든 노드, 토픽, 서비스, 액션, 커스텀 메시지, 주요 파라미터를 정의합니다.
기여자 및 통합 개발자를 위한 공식 참조 문서입니다.

---

## 1. Node List

모든 노드는 `argos_bringup` 패키지에 포함됩니다. 단, 화재 탐지 추론 노드는 `argos_fire_ai`에 있습니다.

| Node | 소스 파일 | 설명 |
|------|----------|------|
| `orchestrator` | `orchestrator_node.py` | 중앙 지휘 노드. CBBA 경매 기반 임무 할당, 4계층 상태 관리. LifecycleNode |
| `frontier_explorer` | `frontier_explorer_node.py` | 프론티어 기반 미탐색 영역 자율 탐색. 멀티로봇 중복 목표 방지. LifecycleNode |
| `hotspot_detector` | `hotspot_detector_node.py` | 열화상 이미지 분석, 화점 위치 추정 및 FireAlert 발행. LifecycleNode |
| `drone_controller` | `drone_controller_node.py` | Gazebo 네이티브 드론 제어. 웨이포인트 순회, 호버링, 착륙 |
| `scenario_runner` | `scenario_runner_node.py` | 정의된 소방 시나리오 순차 실행. SIL 테스트용 |
| `robot_status` | `robot_status_node.py` | 각 로봇의 배터리·위치·상태를 집계하여 발행 |
| `gas_sensor` | `gas_sensor_node.py` | 유독가스 농도(CO, CO₂, NO₂) 시뮬레이션 및 경보. LifecycleNode |
| `victim_detector` | `victim_detector_node.py` | RGB-D 카메라 + 열화상 기반 인명 탐지 |
| `structural_monitor` | `structural_monitor_node.py` | 건물 구조 안전도 평가 (LiDAR 포인트클라우드 분석) |
| `audio_detector` | `audio_detector_node.py` | 음향 이벤트 감지 (비명, 폭발음, 구조 요청 신호) |
| `smoke_effect` | `smoke_effect_node.py` | Gazebo Harmonic 연기 파티클 시뮬레이션 제어 |
| `step_detector` | `step_detector_node.py` | 계단·단차 감지. UGV 이동 가능 여부 판단 |
| `px4_bridge` | `px4_bridge_node.py` | ARGOS ↔ PX4 Offboard 브릿지. ENU↔NED 좌표 변환, TrajectorySetpoint 발행 |
| `mcp_robot_server` | `mcp_robot_server.py` | Model Context Protocol 서버. DimOS 벤치마크용 |
| `scan_frame_relay` | `scan_frame_relay.py` | LiDAR scan 프레임 ID 변환 릴레이 (멀티로봇 TF 격리용) |
| `keepout_manager` | `keepout_manager.py` | Nav2 Costmap keepout zone 동적 관리 |
| `kalman_fire_tracker` | `kalman_fire_tracker.py` | Kalman 필터 기반 화점 위치 시계열 추정 |
| `ugv_platform` | `ugv_platform.py` | UGV Platform 구현체. Nav2 래핑, PlatformInterface 상향 |
| `px4_platform` | `px4_platform.py` | PX4 Drone Platform 구현체. TrajectorySetpoint 래핑 |
| `hose_tether` | `hose_tether_node.py` | 소방 호스 물리 시뮬레이션. 장력 계산, 경로 제약 적용 |
| `water_curtain` | `water_curtain_node.py` | 방수포(Water Curtain) 제어. 차열 구역 생성 |
| `lidar_degradation` | `lidar_degradation_node.py` | 연기·열기로 인한 LiDAR 성능 저하 시뮬레이션 |
| `drone_fire_relay` | `drone_fire_relay_node.py` | 드론 감지 화재 정보를 UGV 및 오케스트레이터로 릴레이 |
| `perception_bridge` | `perception_bridge_node.py` | 카메라/열화상 데이터를 ROS 2 표준 토픽으로 변환 |

---

## 2. Topics

### 핵심 오케스트레이션 토픽

| 토픽 | 타입 | 발행 노드 | 구독 노드 | 설명 |
|------|------|----------|----------|------|
| `/orchestrator/robot_status` | `argos_interfaces/RobotStatus` | `robot_status` (각 로봇) | `orchestrator` | 로봇별 상태 보고 (배터리, 위치, 능력) |
| `/orchestrator/fire_alert` | `argos_interfaces/FireAlert` | `hotspot_detector`, `drone_fire_relay` | `orchestrator` | 화점 감지 경보. 5분 TTL |
| `/orchestrator/mission_state` | `argos_interfaces/MissionState` | `orchestrator` | 전체 | 현재 임무 단계, 할당 상태 |
| `/orchestrator/autonomy_mode` | `std_msgs/String` | `orchestrator` | 전체 | `"CENTRALIZED"` \| `"LOCAL_AUTONOMY"` |
| `/orchestrator/hose_conflict` | `std_msgs/String` | `orchestrator` | `hose_tether` | JSON: `{"rid_a", "rid_b", "resolution"}` |

### 이동 제어 토픽

| 토픽 | 타입 | 발행 노드 | 구독 노드 | 설명 |
|------|------|----------|----------|------|
| `/cmd_vel` | `geometry_msgs/TwistStamped` | `orchestrator`, Nav2 | UGV 플랫폼 | 속도 명령 (**Jazzy 필수**: TwistStamped, header 포함) |
| `/odom` | `nav_msgs/Odometry` | UGV 플랫폼, Gazebo | Nav2, `orchestrator` | 로봇 위치 추정 |
| `/<ns>/cmd_vel` | `geometry_msgs/TwistStamped` | Nav2 (per namespace) | UGV 플랫폼 | 멀티로봇: 네임스페이스 분리 |

### 센싱 토픽

| 토픽 | 타입 | 발행 노드 | 구독 노드 | 설명 |
|------|------|----------|----------|------|
| `/scan` | `sensor_msgs/LaserScan` | Gazebo, `lidar_degradation` | Nav2, `structural_monitor` | 2D LiDAR 스캔 |
| `/map` | `nav_msgs/OccupancyGrid` | `slam_toolbox` | Nav2, `frontier_explorer` | SLAM 생성 맵 |
| `/camera/image_raw` | `sensor_msgs/Image` | Gazebo | `victim_detector`, `perception_bridge` | RGB 카메라 |
| `/thermal/image_raw` | `sensor_msgs/Image` | Gazebo | `hotspot_detector`, `victim_detector` | 열화상 카메라 (32×24 시뮬) |
| `/depth_camera/image_raw` | `sensor_msgs/Image` | Gazebo | `perception_bridge` | 깊이 카메라 이미지 |
| `/depth_camera/points` | `sensor_msgs/PointCloud2` | Gazebo | `structural_monitor` | 포인트클라우드 |
| `/depth_camera/camera_info` | `sensor_msgs/CameraInfo` | Gazebo | `perception_bridge` | 카메라 캘리브레이션 |
| `/gas_concentration` | `argos_interfaces/GasReading` | `gas_sensor` | `orchestrator` | 유독가스 농도 (CO/CO₂/NO₂ ppm) |

### 진단/상태 토픽

| 토픽 | 타입 | 발행 노드 | 설명 |
|------|------|----------|------|
| `/battery_state` | `sensor_msgs/BatteryState` | UGV/Drone 플랫폼 | 배터리 잔량 및 충전 상태 |
| `/network_diagnostics` | `argos_interfaces/NetworkDiagnostics` | Zenoh 브릿지 | 통신 품질 모니터링 |

> **QoS 규칙**: 센서 토픽 → `BEST_EFFORT / VOLATILE`. 명령·상태 토픽 → `RELIABLE / TRANSIENT_LOCAL`.

---

## 3. Services

| 서비스 | 타입 | 제공 노드 | 설명 |
|--------|------|----------|------|
| `/orchestrator/emergency_stop` | `std_srvs/Trigger` | `orchestrator` | 전체 로봇 즉시 정지. E-STOP 레벨 1 |
| `/orchestrator/resume` | `std_srvs/Trigger` | `orchestrator` | E-STOP 해제 후 임무 재개 |
| `/<drone_ns>/arm` | `std_srvs/Trigger` | `px4_bridge` | 드론 ARM (오프보드 진입 전 100회 선발행 필수) |
| `/<drone_ns>/disarm` | `std_srvs/Trigger` | `px4_bridge` | 드론 DISARM |
| `/<drone_ns>/takeoff` | `std_srvs/Trigger` | `px4_bridge` | 이륙 (첫 웨이포인트 z 고도로 상승) |
| `/<drone_ns>/land` | `std_srvs/Trigger` | `px4_bridge` | 현재 위치에서 착륙 |
| `/<drone_ns>/start_mission` | `std_srvs/Trigger` | `px4_bridge` | 웨이포인트 자동 순회 시작 |
| `/<drone_ns>/stop_mission` | `std_srvs/Trigger` | `px4_bridge` | 미션 정지 (호버링 유지) |
| `/navigate_to_object` | `argos_interfaces/NavigateToObject` | `ugv_platform` | 감지된 객체 좌표로 이동 |

---

## 4. Actions

| 액션 | 타입 | 제공 노드 | 설명 |
|------|------|----------|------|
| `/navigate_to_pose` | `nav2_msgs/NavigateToPose` | Nav2 `bt_navigator` | 목표 포즈로 이동. Nav2 표준 액션 |
| `/patrol_area` | `argos_interfaces/PatrolArea` | `patrol_action_server` | 지정 구역 순찰. 웨이포인트 리스트 순회 |
| `/move_to_target` | `argos_interfaces/MoveToTarget` | `ugv_platform` | 목표 좌표로 이동 (Orchestrator 호출용) |
| `/report_fire_alert` | `argos_interfaces/ReportFireAlert` | `orchestrator` | 화재 경보 처리 액션. 재할당 트리거 |

---

## 5. Custom Messages (`argos_interfaces`)

### `RobotStatus.msg`
```
# 개별 로봇 상태 보고 (주기: 1 Hz)
string robot_id          # 로봇 고유 ID (예: "ugv_0", "drone_1")
string robot_type        # "ugv" | "drone" | "sherpa"
float32 battery_level    # 0.0 ~ 1.0 (1.0 = 100%)
geometry_msgs/Pose pose  # map 프레임 기준 현재 위치
string[] capabilities    # ["has_thermal", "can_fly", "has_gas_sensor", ...]
string status            # "IDLE" | "PATROLLING" | "FIRE_RESPONSE" | "RETURNING" | "OFFLINE"
bool is_alive            # heartbeat (3초 이상 미수신 시 OFFLINE 전환)
```

### `FireAlert.msg`
```
# 화점 감지 경보
std_msgs/Header header
geometry_msgs/Point location   # map 프레임 기준 화점 좌표 (x, y, z)
float32 confidence             # 0.0 ~ 1.0 탐지 신뢰도
float32 severity               # 0.0 ~ 10.0 화재 규모 (소방청 등급 매핑)
string source_robot_id         # 최초 감지 로봇 ID
bool is_active                 # TTL 5분 초과 시 false (오케스트레이터 자동 만료)
```

### `MissionState.msg`
```
# 오케스트레이터 임무 상태
string stage             # "INIT" | "EXPLORING" | "FIRE_RESPONSE" | "RETURNING" | "COMPLETE"
int32 robots_active      # 현재 활성 로봇 수
int32 robots_total       # 등록된 전체 로봇 수
float32 map_coverage     # 탐색 완료 맵 커버리지 (0.0 ~ 1.0)
string[] fire_locations  # 활성 화점 ID 목록
```

### `GasReading.msg`
```
# 유독가스 농도 측정값
std_msgs/Header header
string robot_id
float32 co_ppm           # 일산화탄소 농도 (ppm). 경보: 200 ppm
float32 co2_ppm          # 이산화탄소 농도 (ppm). 위험: 5000 ppm
float32 no2_ppm          # 이산화질소 농도 (ppm). 경보: 3 ppm
float32 danger_level     # 0.0 ~ 1.0 종합 위험도
```

### `VictimDetection.msg`
```
# 인명 탐지
std_msgs/Header header
string robot_id
geometry_msgs/Point location    # 추정 위치 (map 프레임)
float32 confidence              # 탐지 신뢰도 0.0 ~ 1.0
bool thermal_confirmed          # 열화상 교차 확인 여부
string status                   # "DETECTED" | "CONFIRMED" | "RESCUED"
```

### `StructuralAlert.msg`
```
# 건물 구조 안전 경보
std_msgs/Header header
string robot_id
float32 risk_score       # 0.0 ~ 1.0 붕괴 위험도
string affected_zone     # 위험 구역 식별자
bool evacuation_required # 즉시 대피 필요 여부
```

### `AudioEvent.msg`
```
# 음향 이벤트 탐지
std_msgs/Header header
string robot_id
string event_type        # "SCREAM" | "EXPLOSION" | "SOS" | "COLLAPSE"
float32 confidence       # 0.0 ~ 1.0
geometry_msgs/Point estimated_location  # 음원 추정 위치
```

---

## 6. Key Parameters

### `nav2_params_sim.yaml` (Nav2 설정)

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `controller_server.controller_frequency` | `10.0` | 컨트롤러 업데이트 주기 (Hz). 낮추면 CPU 절약, 높이면 반응성 향상 |
| `local_costmap.resolution` | `0.05` | 로컬 코스트맵 해상도 (m/cell) |
| `global_costmap.resolution` | `0.05` | 글로벌 코스트맵 해상도 (m/cell) |
| `planner_server.GridBased.plugin` | `"nav2_smac_planner/SmacPlannerHybrid"` | SmacPlanner2D 사용 권장 (NavFn 대비 장애물 회피 우수) |
| `controller_server.FollowPath.plugin` | `"nav2_mppi_controller::MPPIController"` | DWB 대비 좁은 통로 성능 우수 |
| `amcl.max_particles` | `2000` | 위치 추정 파티클 최대 수. 정확도 vs. CPU 트레이드오프 |
| `amcl.min_particles` | `500` | 파티클 최솟값 |

### `slam_params.yaml` (slam_toolbox 설정)

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `solver_plugin` | `"solver_plugins::CeresSolver"` | Ceres 솔버 (정확도 높음) |
| `scan_topic` | `"/scan"` | 입력 LiDAR 토픽 |
| `map_frame` | `"map"` | 맵 프레임 ID |
| `base_frame` | `"base_footprint"` | 로봇 기준 프레임 |
| `map_update_interval` | `5.0` | 맵 갱신 주기 (초) |

### `sensors.yaml` (센서 노드 공통)

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `publish_rate` | `1.0` | 센서 데이터 발행 주기 (Hz). gas_sensor, thermal 공통 |
| `noise_stddev` | `0.01` | 센서 노이즈 표준편차 (시뮬 전용) |

### `orchestrator_node` 런타임 파라미터

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `heartbeat_timeout` | `3.0` | 로봇 오프라인 판정 기준 (초) |
| `return_battery_threshold` | `0.2` | 귀환 트리거 배터리 임계값 |
| `fire_alert_ttl` | `300.0` | FireAlert 유효 기간 (초, 5분) |
| `autonomy_fallback_timeout` | `10.0` | 통신 두절 후 LOCAL_AUTONOMY 전환 대기 시간 (초) |

---

> 자세한 아키텍처 설명은 `docs/architecture/` 참조.
> 포팅 및 하드웨어 통합은 `docs/hardware-porting-guide.md` 참조.
