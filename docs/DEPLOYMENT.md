# ARGOS Deployment Guide

> Version: v3.7 (Phase D+) | ROS 2 Jazzy | Gazebo Harmonic | Updated: 2026-03-23

이 문서는 ARGOS 시스템의 배포 방법을 다룹니다: 로컬 시뮬레이션, HEAT Portal 연동, Docker(예정), 실물 하드웨어 포팅, PX4 드론 설정 순으로 기술합니다.

---

## 1. Local Simulation

### 사전 준비

WSL2 Ubuntu 24.04 + ROS 2 Jazzy 환경이 필요합니다. 초기 설정은 `docs/wsl2-setup-guide.md`를 참조하세요.

```bash
# WSL2 환경에서 워크스페이스 소싱
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

### 단일 로봇 시뮬레이션

기본 Nav2 + SLAM + Gazebo 통합 실행:

```bash
# Gazebo Harmonic + Nav2 + SLAM (단일 UGV)
ros2 launch argos_bringup slam_nav2_sim.launch.py

# 별도 터미널: RViz2 시각화
ros2 launch argos_bringup nav2_rviz.launch.py
```

오케스트레이터 포함 전체 시스템:

```bash
# 오케스트레이터 + 센서 시스템 전체
ros2 launch argos_bringup multi_node_system.launch.py
```

화재 현장 시나리오 (fire_building.sdf 월드):

```bash
ros2 launch argos_bringup multi_node_system.launch.py \
  world:=fire_building \
  enable_smoke:=true \
  enable_thermal:=true
```

### 멀티 로봇 시뮬레이션 (UGV 3대)

```bash
# 3대 UGV 병렬 탐색 (네임스페이스: robot_0, robot_1, robot_2)
ros2 launch argos_bringup multi_node_system.launch.py \
  num_robots:=3 \
  enable_frontier:=true
```

> **리소스 참고**: RTX 4050 기준 풀센서 구성 3~5대, 센서 간소화 시 8~10대까지 안정 운용 가능합니다.

### 드론 포함 이종 시뮬레이션 (UGV + 드론 편대)

```bash
# PX4 SITL 없이 Gazebo 네이티브 드론 (기본 모드)
ros2 launch argos_bringup multi_node_system.launch.py \
  num_ugv:=2 \
  num_drones:=2 \
  use_px4:=false

# PX4 SITL 설치 후 (use_px4:=true)
ros2 launch argos_bringup multi_node_system.launch.py \
  num_ugv:=2 \
  num_drones:=2 \
  use_px4:=true
```

### 시스템 종료

```bash
# ROS 2 노드 전체 종료
Ctrl+C (launch 터미널)

# WSL2 메모리 반환 (VmmemWSL 프로세스 해제) — 매 세션 종료 시 필수
wsl --shutdown
```

---

## 2. HEAT Portal Integration

ARGOS는 `daegufire.ai.kr/argos` 를 통해 웹 대시보드와 연동됩니다.

### 아키텍처

```
ARGOS ROS 2 (WSL2)
    │
    │  rosbridge_websocket (port 9090)
    ▼
HEAT Portal (daegufire.ai.kr)
    │  argos/ArgosMonitor.tsx
    │  2D Canvas 맵, 미션 패널, 로봇 제어, 화재 로그
    ▼
소방관 (웹 브라우저)
```

### rosbridge 실행

```bash
# rosbridge WebSocket 서버 시작
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# 확인: ws://localhost:9090 접속 가능 여부
```

### 포털 연동 설정

포털 프론트엔드(`ArgosMonitor.tsx`)는 아래 환경변수로 rosbridge 엔드포인트를 지정합니다:

```env
VITE_ROSBRIDGE_URL=ws://localhost:9090
```

프로덕션 배포 시 WSL2의 포트를 외부로 노출하려면 Windows 측 포트 포워딩이 필요합니다:

```powershell
# Windows PowerShell (관리자 권한)
netsh interface portproxy add v4tov4 listenport=9090 listenaddress=0.0.0.0 \
  connectport=9090 connectaddress=$(wsl hostname -I | awk '{print $1}')
```

### 포털에서 제공하는 기능

| 기능 | 설명 | 관련 토픽/서비스 |
|------|------|----------------|
| 2D Canvas 맵 | 실시간 OccupancyGrid 렌더링 + 로봇 위치 오버레이 | `/map`, `/orchestrator/robot_status` |
| 미션 패널 | 임무 단계 표시, 화점 마커, 커버리지 % | `/orchestrator/mission_state` |
| 로봇 제어 | E-STOP 버튼, 임무 재개 | `/orchestrator/emergency_stop`, `/orchestrator/resume` |
| 화재 로그 | 화점 감지 이력, TTL 타이머 | `/orchestrator/fire_alert` |

---

## 3. Docker (Placeholder)

> **현재 상태**: 기본 Dockerfile 및 docker-compose.yml 존재. 완전한 컨테이너 배포는 Phase F에서 구현 예정.

프로젝트 루트의 `Dockerfile` 및 `docker-compose.yml`은 향후 아래 구조로 확장될 예정입니다:

```yaml
# docker-compose.yml (향후 계획)
services:
  ros2_base:       # ROS 2 Jazzy + 의존성
  gazebo_sim:      # Gazebo Harmonic 시뮬레이션
  argos_nodes:     # ARGOS 노드 클러스터
  rosbridge:       # WebSocket 브릿지
  rviz2:           # 시각화 (X11 포워딩)
```

컨테이너 배포가 완성되면 이 섹션을 갱신합니다.

현재 Docker 관련 파일:
- `Dockerfile` — 기반 이미지 정의
- `docker-compose.yml` — 서비스 구성
- `docker-entrypoint.sh` — 컨테이너 진입점

---

## 4. Real Robot Porting

시뮬레이션에서 실물 하드웨어로 전환할 때는 **Platform 계층(Layer 4)만 교체**합니다. Orchestrator 이상의 코드는 변경하지 않습니다.

### 포팅 원칙

ARGOS의 4계층 아키텍처에서 실물 로봇은 `PlatformInterface`(ABC)를 구현하는 새 클래스를 작성하면 됩니다:

```python
# 새 플랫폼 구현 예시
class TurtleBot4Platform(PlatformInterface):
    def move_to(self, goal: PoseStamped) -> None: ...
    def get_pose(self) -> PoseStamped: ...
    def get_battery(self) -> float: ...
    def emergency_stop(self) -> None: ...
```

### 검증된 플랫폼

| 플랫폼 | 구현 파일 | 상태 |
|--------|----------|------|
| UGV (Gazebo) | `ugv_platform.py` | 검증 완료 (V-6, 1.068m 이동 확인) |
| PX4 드론 (SITL) | `px4_platform.py` | 검증 완료 (V-5, uXRCE-DDS 연결 성공) |
| HR-셰르파 (시뮬) | `sherpa_platform.py` | 검증 완료 (Phase D+, 6WD + 호스 물리) |
| TurtleBot4 (실물) | — | Phase F 예정 |

### 상세 포팅 절차

전체 하드웨어 포팅 절차, 센서 캘리브레이션, 필드 테스트 체크리스트는 다음을 참조하세요:

```
docs/hardware-porting-guide.md
docs/hardware-design-spec.md
```

---

## 5. PX4 Drone Setup

### uXRCE-DDS Agent 설치 및 실행

ARGOS의 `px4_bridge_node`는 PX4 SITL과 uXRCE-DDS를 통해 통신합니다.

```bash
# uXRCE-DDS Agent 설치 (빌드)
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install

# uXRCE-DDS Agent 실행 (UDP, PX4 기본 포트)
MicroXRCEAgent udp4 -p 8888
```

### PX4 SITL 실행 (Gazebo Harmonic)

```bash
# PX4 저장소 클론 (별도 디렉토리)
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot

# x500 쿼드콥터 SITL 시작 (Gazebo Harmonic)
make px4_sitl gz_x500

# 멀티드론: 인스턴스 번호로 네임스페이스 분리 (drone_0, drone_1, ...)
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,1,0,0,0,0" \
  PX4_MICRODDS_NS=px4_1 ./build/px4_sitl_default/bin/px4 -i 1
```

### ARGOS와 PX4 SITL 연동

```bash
# 1. uXRCE-DDS Agent 실행 (터미널 1)
MicroXRCEAgent udp4 -p 8888

# 2. PX4 SITL 실행 (터미널 2)
make px4_sitl gz_x500

# 3. ARGOS 드론 포함 launch (터미널 3)
source ~/ros2_ws/install/setup.bash
ros2 launch argos_bringup multi_node_system.launch.py \
  use_px4:=true \
  num_drones:=1
```

### Offboard 모드 진입 시퀀스

PX4 Offboard 모드는 순서를 지켜야 합니다:

1. `OffboardControlMode` + `TrajectorySetpoint` **100회 이상 선발행** (arm 전 필수)
2. `/px4_0/arm` 서비스 호출 → ARM
3. `/px4_0/takeoff` 서비스 호출 → 이륙
4. `/px4_0/start_mission` 서비스 호출 → 웨이포인트 순회 시작

> **주의**: `px4_msgs` 패키지가 설치되지 않은 환경에서는 `px4_bridge_node`가 자동으로 passthrough 모드로 동작합니다.

### 멀티드론 편대 (4대)

```bash
# 4대 편대 launch (CBBA 경매 기반 임무 할당)
ros2 launch argos_bringup multi_node_system.launch.py \
  use_px4:=true \
  num_drones:=4 \
  formation_pattern:=diamond
```

지원 편대 패턴: `line`, `diamond`, `v_shape`, `circle`

---

> 트러블슈팅은 `docs/troubleshooting/` 참조.
> PX4-ROS 2 심층 분석: `knowledge/projects/research/2026-03-15-px4-ros2-drone-integration-deep-dive.md`
