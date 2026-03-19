# ARGOS PX4 SITL Docker 환경

ARGOS Phase D — PX4 드론 통합을 위한 격리 Docker 환경.

## 배경

ROS2 Jazzy + PX4 v1.16 조합의 `gz_bridge` 클락 타임아웃 버그(PX4 Issue #24159, 미해결)를 우회하기 위해
PX4 레이어를 Docker 내 Ubuntu 22.04 + Humble 환경으로 분리한다.

```
[WSL2 호스트: Ubuntu 24.04 + Jazzy]  ←→  [Docker: Ubuntu 22.04 + Humble]
  argos_bringup, argos_description            PX4 SITL + Gazebo Harmonic
  UGV 스택 (Nav2, SLAM, 오케스트레이터)      ↕ UDP 8888 (uXRCE-DDS)
                                           uXRCE-DDS Agent
                                              ↓ /fmu/in/* /fmu/out/* 토픽
```

## 구조

```
docker/px4-sitl/
├── docker-compose.yml    # 서비스 정의 (px4-sitl + uxrce-dds)
├── Dockerfile.px4        # PX4 v1.16.0 SITL 빌드 이미지
└── README.md             # 이 파일
```

## 빠른 시작

```bash
cd projects/argos/docker/px4-sitl

# 빌드 및 시작 (첫 실행: PX4 소스 빌드로 20~40분 소요)
docker compose up -d

# 로그 확인
docker compose logs -f px4-sitl
docker compose logs -f uxrce-dds

# 종료
docker compose down
```

## 브릿지 연결 확인

PX4 + uXRCE-DDS Agent 정상 기동 후 호스트 ROS2에서 확인:

```bash
# px4_msgs 패키지가 워크스페이스에 빌드되어 있어야 함
source ~/ros2_ws/install/setup.bash

# PX4 발행 토픽 확인 (/fmu/out/* 형태)
ros2 topic list | grep fmu

# 드론 상태 확인
ros2 topic echo /fmu/out/vehicle_status --once

# 드론 위치 확인 (NED 좌표계)
ros2 topic echo /fmu/out/vehicle_local_position --once
```

## px4_msgs 설치 (호스트 ROS2 워크스페이스)

PX4 토픽 타입을 호스트 ROS2에서 사용하려면 `px4_msgs`를 워크스페이스에 추가:

```bash
cd ~/ros2_ws/src
git clone --branch release/1.16 https://github.com/PX4/px4_msgs.git

cd ~/ros2_ws
colcon build --packages-select px4_msgs
source install/setup.bash
```

## 좌표계 주의사항

PX4는 **NED(North-East-Down)** 좌표계를 사용한다.
ARGOS 현재 스택은 **ENU(East-North-Up)** 기준이므로 변환 필요:

```
x_enu =  y_ned
y_enu =  x_ned
z_enu = -z_ned
```

`TrajectorySetpoint`에서 고도 5m 상승 = `position = [0.0, 0.0, -5.0]`

## 오프보드 제어 최소 패턴

```python
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand

# OffboardControlMode는 2Hz 이상 반드시 발행해야 오프보드 모드 유지됨
offboard_msg = OffboardControlMode()
offboard_msg.position = True
offboard_msg.timestamp = node.get_clock().now().nanoseconds // 1000

# 목표 위치 (NED, 미터)
setpoint = TrajectorySetpoint()
setpoint.position = [0.0, 0.0, -5.0]   # 고도 5m
setpoint.yaw = 0.0
setpoint.timestamp = node.get_clock().now().nanoseconds // 1000
```

## 멀티드론 확장

두 번째 드론 추가 시 `PX4_UXRCE_DDS_NS` 환경변수로 네임스페이스 분리:

```yaml
# docker-compose.yml에 추가
px4-sitl-2:
  build: .
  environment:
    - PX4_SIM_MODEL=gz_x500
    - PX4_UXRCE_DDS_NS="-n px4_1"    # 두 번째 드론 네임스페이스
  network_mode: host
```

토픽: `/px4_1/fmu/in/trajectory_setpoint`, `/px4_1/fmu/out/vehicle_status`

## 다음 단계 (Phase D 세션)

- [ ] `argos_bringup/argos_bringup/drone_controller_node.py`에 PX4 어댑터 레이어 추가
- [ ] ENU ↔ NED 변환 유틸리티 (`argos_bringup/argos_bringup/coordinate_utils.py`)
- [ ] 단일 드론 오프보드 Arm → 이륙 → 웨이포인트 → 착륙 시퀀스 검증
- [ ] `aerial-autonomy-stack` 멀티드론 패턴 참조 통합

## 참고

- [PX4 ROS2 User Guide](https://docs.px4.io/main/en/ros2/user_guide)
- [PX4 uXRCE-DDS 문서](https://docs.px4.io/main/en/middleware/uxrce_dds)
- [PX4 Offboard Control Example](https://docs.px4.io/main/en/ros2/offboard_control)
- [aerial-autonomy-stack GitHub](https://github.com/JacopoPan/aerial-autonomy-stack)
- [PX4 Issue #24159](https://github.com/PX4/PX4-Autopilot/issues/24159) — Jazzy 버그 추적
- 리서치 상세: `knowledge/projects/research/2026-03-15-px4-ros2-drone-integration-deep-dive.md`
