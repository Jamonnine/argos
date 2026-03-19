# ARGOS Phase F — 시뮬→실세계 하드웨어 포팅 가이드

> 작성: 2026-03-19 | 대상 Phase: F (v5.0)
> 전제: Phase A~E 시뮬레이션 검증 완료, PlatformInterface 추상화 완료 (53 tests)
> 관련 문서: `docs/hardware-design-spec.md` | `memory/argos-tutor.md`

---

## 개요

ARGOS v2.0~v4.0까지 Gazebo Harmonic 시뮬레이션으로 검증한 아키텍처를 실물 하드웨어로
전환하는 절차를 기술한다. 4계층 아키텍처(Orchestrator → Mission → Core → Platform)의
Platform 계층만 교체하는 원칙에 따라 상위 레이어 코드 변경을 최소화한다.

---

## 1. UGV 포팅 — TurtleBot4

### 1.1 플랫폼 선정 근거

| 항목 | 근거 |
|------|------|
| 플랫폼 | TurtleBot4 Standard (iRobot Create3 + RPi CM4) |
| 선정 이유 | ROS 2 Jazzy 공식 지원, Nav2 최적화, 소방청 리빙랩 호환 |
| 대안 | Clearpath Jackal (야외 강점, 단가 높음) — Phase G 고려 |
| 소방 현장 제한 | 옥내 탐색 중심 → TurtleBot4 족하 (HR-셰르파 대체 아님) |

### 1.2 센서 매핑 (Gazebo → 실물)

| 역할 | Gazebo 가상 센서 | 실물 센서 | 드라이버 패키지 |
|------|----------------|----------|----------------|
| LiDAR (수평) | `gpu_lidar` plugin | **RPLiDAR A2M12** (12m, 10Hz) 또는 A3 (25m) | `rplidar_ros` (ROS2 지원) |
| RGB/Depth | Gazebo `depth_camera` | **OAK-D Pro** (720p, 스테레오, ToF) | `depthai-ros` |
| IMU | Gazebo `imu` plugin | **BNO055** (Create3 내장) | Create3 펌웨어 내장 |
| 열화상 | Gazebo thermal 커스텀 | **FLIR Lepton 3.5** (160×120, 8.6Hz) | `lepton_ros2` (PureThermal 보드 필요) |
| 오도메트리 | Gazebo ground truth | Create3 휠 인코더 + IMU 융합 | Create3 내장 |

> **주의**: RPLiDAR A2는 `/scan` 토픽을 `sensor_msgs/LaserScan`으로 발행하며
> Gazebo와 동일한 인터페이스. OAK-D Pro는 `/color/image_raw`와 `/stereo/depth`를
> 각각 발행 — `argos_bringup`의 `perception_bridge_node.py` remapping 확인 필수.

### 1.3 ros2_control 전환

```
시뮬: GazeboSystem (gz_ros2_control)
실물: TurtleBot4Hardware (Create3 기반 diff_drive_controller)
```

`argos_description/urdf/argos_ugv.urdf.xacro` 수정 포인트:

```xml
<!-- 시뮬용 (기존) -->
<plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
  <parameters>$(find argos_description)/config/ros2_control.yaml</parameters>
</plugin>

<!-- 실물용 (교체) -->
<ros2_control name="TurtleBot4Hardware" type="system">
  <hardware>
    <plugin>turtlebot4_hardware/TurtleBot4Hardware</plugin>
  </hardware>
  <!-- joint 정의는 동일 유지 -->
</ros2_control>
```

`config/ros2_control.yaml`에서 `max_velocity`를 실측 기반으로 재설정 (Create3 최대 0.306 m/s).

### 1.4 URDF 실측 수정 항목

TurtleBot4 실물 치수 기준으로 `argos_ugv_base.urdf.xacro` 수정:

| 파라미터 | Gazebo 기본값 | TurtleBot4 실측값 | 비고 |
|---------|-------------|-----------------|------|
| `wheel_radius` | 0.033 | **0.0352** m | Create3 실측 |
| `wheel_separation` | 0.287 | **0.233** m | 휠 중심 간격 |
| `base_footprint` (r) | 0.105 | **0.175** m | TurtleBot4 반경 |
| LiDAR mount z | 0.12 | 실측 후 기입 | RPLiDAR 마운트 위치 |
| Camera mount | (0.05, 0, 0.15) | 실측 후 기입 | OAK-D 브라켓 기준 |
| Thermal mount | (0.0, 0, 0.20) | 실측 후 기입 | Lepton 마운트 |

### 1.5 Nav2 파라미터 실환경 튜닝

`argos_description/config/nav2_params.yaml` — 실물 적용 시 조정 필수:

```yaml
# 속도 제한 (Create3 물리 한계 반영)
controller_server:
  FollowPath:
    max_vel_x: 0.26          # 시뮬 0.5 → 실물 0.26 (Create3 한계)
    max_vel_theta: 1.8        # 시뮬 2.5 → 실물 1.8
    min_vel_x: 0.0
    acc_lim_x: 2.0            # 시뮬 3.0 → 보수적으로 낮춤

# Costmap footprint (TurtleBot4 실측)
local_costmap:
  robot_radius: 0.18          # 시뮬 0.12 → 실물 0.175+여유

# AMCL (실내 SLAM 위치 추정)
amcl:
  min_particles: 1000         # 실내 다중 경로 환경 대응
  max_particles: 5000
  laser_model_type: "likelihood_field"
```

---

## 2. 드론 포팅 — PX4 실드론

### 2.1 플랫폼 선정

| 항목 | 내용 |
|------|------|
| 프레임 | **Holybro X500 v2** (대각 500mm, 페이로드 800g+) |
| FC (비행 컨트롤러) | **Pixhawk 6C** (STM32H753, PX4 공식 지원) |
| 선정 이유 | PX4 공식 reference design, X500 SDF 모델이 PX4 SITL 표준 |
| 대안 | DJI F450 프레임 (저렴) — FC 교체 시 동일 적용 가능 |

### 2.2 센서 구성

| 역할 | 드론 탑재 | 연결 | 비고 |
|------|----------|------|------|
| 비행 IMU | Pixhawk 6C 내장 (ICM-42688-P) | 내장 | FC 자체 처리 |
| GPS | M10 GPS (uBlox M10) | UART | Pixhawk 6C 번들 |
| 하방 카메라 | OAK-D Pro W (광각) | USB3 → Companion | 피해자 탐색 |
| 열화상 | FLIR Lepton 3.5 | SPI → Companion | 화점 감지 (UGV와 동일 모듈) |
| Companion 컴퓨터 | **Raspberry Pi 5** (4GB) | USB-C 전원 + UART | uXRCE-DDS 브리지 |

### 2.3 uXRCE-DDS 전환 (SITL → 실드론)

```
시뮬: UDP 통신 (PX4 SITL ↔ uXRCE-DDS Agent)
실물: Serial/UART 통신 (Pixhawk 6C ↔ RPi5 ↔ uXRCE-DDS Agent)
```

`scripts/test_px4_offboard.py` 실물 적용 시 변경사항:

```python
# 시뮬 (기존)
# PX4 SITL은 UDP 14550 자동 연결

# 실물 — uXRCE-DDS Agent 실행 방법 변경
# RPi5에서:
# sudo MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600
# (Pixhawk 6C TELEM2 ↔ RPi UART0, 921600 baud)
```

`argos_bringup/launch/multi_px4.launch.py` 추가 파라미터:

```python
# 실물 드론별 namespace는 PX4_SYS_AUTOSTART 기반으로 자동 설정
# 멀티 드론 시 각 Pixhawk에 고유 MAV_SYS_ID 설정 필수 (1, 2, 3...)
```

### 2.4 PX4 파라미터 설정 (QGroundControl)

실물 첫 기동 전 QGroundControl에서 설정 필수:

| 파라미터 | 값 | 비고 |
|---------|----|------|
| `MAV_SYS_ID` | 드론별 고유 (1,2,3...) | 멀티 드론 식별 |
| `UXRCE_DDS_CFG` | 101 (TELEM2) | Companion UART 연결 |
| `SER_TEL2_BAUD` | 921600 | uXRCE-DDS 속도 |
| `COM_RCL_EXCEPT` | 4 (Offboard + Mission) | RC 없이 offboard 허용 |
| `NAV_RCL_ACT` | 0 (none) | RC 두절 시 홀드 |
| `NAV_DLL_ACT` | 0 (disabled) | 데이터링크 두절 홀드 |

---

## 3. Sim-to-Real 체크리스트

Phase F 착수 전 단계별 검증 게이트.

### 3.1 하드웨어 준비

- [ ] URDF 실측 수정 완료 (wheel_radius, footprint, 센서 위치)
- [ ] 센서 드라이버 ROS2 노드 개별 테스트 (`ros2 topic echo` 확인)
  - [ ] `rplidar_ros` → `/scan` 정상 발행
  - [ ] `depthai-ros` → `/color/image_raw`, `/stereo/depth` 정상
  - [ ] `lepton_ros2` → `/thermal/image_raw` 정상
  - [ ] Create3 → `/odom`, `/cmd_vel` 정상 응답
- [ ] TF tree 검증 (`rviz2` — `map → odom → base_link → sensor_frames` 연결)
- [ ] Emergency stop 물리 버튼 연결 및 동작 확인
  - 버튼 → RPi GPIO → `ros2 topic pub /emergency_stop std_msgs/Bool "{data: true}"`
  - `orchestrator_node.py`의 `_handle_emergency_stop()` 실제 발동 확인

### 3.2 드론 사전 검증

- [ ] QGroundControl 파라미터 설정 완료
- [ ] Pixhawk 6C 센서 캘리브레이션 (가속도계, 자이로, 나침반, 수평)
- [ ] uXRCE-DDS Serial 연결 확인 (`ros2 topic list`에 `/fmu/out/*` 등장)
- [ ] Offboard 모드 전환 테스트 (지상 0.1m 호버링)
- [ ] Emergency kill switch (RC 채널 5 또는 GCS MAVLink CMD) 동작 확인

### 3.3 소프트웨어 통합

- [ ] Nav2 파라미터 실환경 튜닝 (첫 실내 주행 후 오차 분석)
- [ ] 실내 SLAM 지도 품질 확인 (slam_toolbox, 15×15m 이상 구역)
  - `map_saver_cli -f /tmp/indoor_map` 후 pgm 파일로 품질 육안 확인
- [ ] PlatformInterface 실물 구현체 테스트
  - `UGVPlatform` → Create3 cmd_vel 응답 확인
  - `PX4Platform` → TrajectorySetpoint 실드론 반응 확인
- [ ] 멀티로봇 TF 충돌 없음 확인 (robot1_map, robot2_map 격리 확인)
- [ ] Zenoh → 실물 WiFi 환경 연결 테스트 (uXRCE-DDS는 별도 직렬)

### 3.4 야외 실증 전 필수

- [ ] 달성군 실증공역 비행 승인 (항공안전법 제127조, 비행금지구역 확인)
- [ ] 드론 보험 가입 확인 (2kg 이상 대인·대물)
- [ ] 안전요원 2인 이상 동행
- [ ] 배터리 충전 상태 100% 확인 + 예비 배터리 2세트
- [ ] 비상 착륙 지점 사전 지정

---

## 4. 단계별 전환 전략

Phase F는 점진적으로 진행한다. 한 번에 전체를 실물로 바꾸지 않는다.

```
Step 1: UGV 1대 실내 주행 (복도/연습장)
        → Nav2 파라미터 튜닝 + 센서 TF 검증

Step 2: UGV 2대 실내 협업 탐색
        → 멀티 네임스페이스 TF 실물 검증 + SLAM 분산 확인

Step 3: 드론 옥내 호버링 + offboard 제어
        → PX4Platform 실드론 동작 확인

Step 4: UGV + 드론 실내 협업 (소규모)
        → 오케스트레이터 실물 이종 편대 첫 실행

Step 5: 달성군 야외 실증 (outdoor_test_launcher.sh)
        → 실전 환경 전체 파이프라인 검증
```

---

## 5. 예상 이슈 및 대응

| 이슈 | 원인 | 대응 |
|------|------|------|
| TF 불일치 (센서 → base_link) | URDF 실측 미적용 | xacro 수정 후 `robot_state_publisher` 재기동 |
| Nav2 진동/과조향 | 실물 속도 한계 미반영 | `max_vel_x`, `acc_lim_x` 낮춤 |
| 열화상 FPS 저하 (실내) | USB 대역폭 경합 | OAK-D와 Lepton USB 허브 분리 |
| uXRCE-DDS 연결 끊김 | UART 속도/전압 불안정 | 921600→460800 낮춤, 전원 안정화 |
| SLAM 지도 왜곡 | 반사 바닥, 유리 벽 | `slam_toolbox` reflective_threshold 조정 |
| 멀티드론 MAV_SYS_ID 충돌 | ID 미설정 | 첫 기동 전 QGC에서 개별 확인 |

---

## 참고 링크

- TurtleBot4 공식 문서: https://turtlebot.github.io/turtlebot4-user-manual/
- PX4 uXRCE-DDS: https://docs.px4.io/main/en/middleware/uxrce_dds.html
- OAK-D ROS2: https://github.com/luxonis/depthai-ros
- RPLIDAR ROS2: https://github.com/Slamtec/rplidar_ros/tree/ros2
- 관련 리서치: `knowledge/projects/research/2026-03-15-px4-ros2-drone-integration-deep-dive.md`
