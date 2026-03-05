# ARGOS 시니어 수준 지식 통합 — 4대 리서치 교차 분석

조사일: 2026-03-05
출처: 4개 병렬 리서치 에이전트 (ROS2 멀티로봇, Nav2/SLAM, Gazebo Harmonic, SW 아키텍처)

---

## 1. 핵심 아키텍처 결정 (교차 분석 결론)

### 통신: DDS → Zenoh 전환 필수
- **근거**: Springer 2024 논문 실측 — 무선 메시 환경에서 Zenoh가 DDS 대비 발견 트래픽 97~99% 감소, 지연·손실·CPU 모두 최우수
- **소방 현장**: 건물 내부 WiFi 불안정, 연기로 인한 신호 감쇠 → DDS 멀티캐스트 플러딩이 치명적
- **적용**: `export RMW_IMPLEMENTATION=rmw_zenoh_cpp` (코드 변경 0)
- **3대 이상**: CycloneDDS `MaxAutoParticipantIndex` 120으로 올려야 함

### 오케스트레이션: 하이브리드 (Open-RMF 패턴)
- **DARPA SubT CERBERUS 교훈**: "로컬 자율성 우선, 통신은 보너스"
  - 각 로봇은 오케스트레이터 없어도 마지막 임무를 독립 수행
  - 통신 가능 시에만 서브맵/화점 공유
- **Open-RMF FleetAdapter**: 이종 플릿 등록 → 능력 선언 → 태스크 경매
- **CBBA**: 분산 합의 기반 태스크 할당 — 통신 두절 내성, 이종 능력 매칭

### 미션 제어: BehaviorTree.CPP v4
- **SMACH 대비 장점**: 리액티브 안전 조건(매 틱 재평가), 서브트리 모듈 재사용
- **Nav2 공식 표준**: BtActionNode 템플릿으로 커스텀 액션 연동
- **소방 특화**: ReactiveSequence 루트에 배터리/O2/온도 안전 조건 배치

### 안전: 3계층 E-STOP + Lifecycle + Watchdog
1. **SW E-STOP**: 오케스트레이터 브로드캐스트 (QoS Reliable+KeepAll)
2. **드라이버 E-STOP**: Lifecycle on_deactivate() → 모터 정지 → 브레이크
3. **HW E-STOP**: 물리 버튼 + 안전 릴레이 (소프트웨어 무관)
- `software_watchdogs`: Heartbeat 200ms + Watchdog lease 220ms

---

## 2. 멀티로봇 TF2 핵심 패턴 (가장 빠지기 쉬운 함정)

### 문제
- `/tf`, `/tf_static`은 절대경로 하드코딩 → 네임스페이스 무시 → 로봇 간 TF 충돌

### 해결: 하이브리드 전략 (ARGOS 채택)
```python
# 각 robot_state_publisher에:
remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
# + frame_prefix로 프레임명 분리
parameters=[{'frame_prefix': f'{robot_name}/'}]
```

### 토픽 경로 함정
- costmap 플러그인 안에서 `topic: scan`은 `/robot1/local_costmap/local_costmap/scan`으로 풀림
- **반드시 절대경로**: `topic: /robot1/scan` 또는 SetRemap으로 처리

---

## 3. Gazebo Harmonic 핵심 (Classic과의 차이)

### 필수 플러그인 3+2
| 플러그인 | 역할 | 없으면 |
|---------|------|--------|
| physics | DART 물리 엔진 | 물체 정지 |
| user_commands | 모델 생성/삭제 | 스폰 불가 |
| scene_broadcaster | 씬 상태 브로드캐스트 | GUI/RViz 불가 |
| sensors (센서 사용 시) | 센서 렌더링 | LiDAR/카메라 불가 |
| imu (IMU 사용 시) | IMU 시뮬레이션 | IMU 데이터 없음 |

### Building Editor 없음 → SDF 직접 작성
### 렌더링: OGRE2 (PBR, 파티클 지원)
### 물리: DART (ODE 미지원)

### 열화상 카메라 3중 구조
1. `gz-sim-thermal-system` → 물체에 온도 부여
2. `gz-sim-thermal-sensor-system` → 카메라 온도 범위 설정
3. `gz-sim-sensors-system` + `ogre2` → 렌더링

### 드론: cmd_vel = UGV와 동일 Twist
- `MulticopterMotorModel` × 4 (로터별 추력)
- `MulticopterVelocityControl` × 1 (Lee 컨트롤러 → 모터 RPM 역산)
- linear.z = 고도 제어 (UGV는 0)

### 헤드리스/CI
```bash
DISPLAY= gz sim -s -r --headless-rendering world.sdf
# real_time_factor: 0 → CI 최대속도
```

---

## 4. Nav2/SLAM 고급 전략

### SLAM 전략: 단일 SLAM + 나머지 AMCL (권장)
- UGV1: `slam_toolbox` (async 모드) → `/map` 발행
- UGV2, Drone1: AMCL → 공유 맵 기반 위치추정
- 장기: `map_merge`로 분산 SLAM (피처 매칭 한계 주의)

### 경로 계획: NavFn → Smac (레거시 교체)
| 플래너 | 용도 |
|--------|------|
| SmacPlanner2D | 차동구동 UGV (표준) |
| SmacPlannerHybrid | 방향 중요 플랫폼 |
| MPPI Controller | DWB 대체 — 동적 장애물 회피 최우수 |

### Keepout Zones → 화재 현장 핵심
- 열화상 → 고온 구역 자동 keepout mask 생성
- `nav2_costmap_2d::KeepoutFilter` 플러그인
- 오케스트레이터가 실시간 갱신 가능

### 피어 로봇 장애물 인식
- FleetObstaclePublisher가 각 로봇 위치 → PointCloud2로 변환
- 각 로봇 costmap의 obstacle_layer로 구독

### 탐사: 정보이득 + 열원 우선
- `score = info_gain / travel_cost`
- 열원 근처 frontier에 가중치 부여 (안전거리 내는 패널티)

---

## 5. 테스팅 피라미드

```
         [SIL: Gazebo headless]        ← 주요 시스템 검증
            /              \
    [Integration: launch_testing]       ← 노드 간 통신 검증
        /                      \
[Unit: pytest]                          ← 알고리즘 로직 검증 (현재 85개)
```

### launch_testing 핵심
- `@launch_testing.parametrize` → 로봇 수 파라미터화
- `ReadyToTest()` → 노드 실행 대기
- `post_shutdown_test` → 종료 후 에러 로그 확인
- CMakeLists: `run_test_isolated` → 고유 ROS_DOMAIN_ID 부여

---

## 6. 패키지 구조 권장 (현재 vs 목표)

### 현재 ARGOS
```
argos_description/ — URDF + launch + world + web
my_robot_bringup/  — 모든 노드 + 테스트
my_robot_interfaces/ — msg/srv/action
```

### 목표 (시니어 수준)
```
argos_description/  — URDF/xacro + meshes만
argos_interfaces/   — msg/srv/action
argos_core/         — 오케스트레이터 + 태스크 할당
argos_ugv/          — UGV 전용 (Lifecycle Node)
argos_drone/        — 드론 전용
argos_bt_nodes/     — 커스텀 BT 플러그인
argos_behavior_trees/ — BT XML 파일
argos_simulation/   — Gazebo worlds + models
argos_bringup/      — launch 파일 (최상위)
```

---

## 7. 레퍼런스 리포 핵심 요약

| 리포 | 핵심 배움 | ARGOS 적용 |
|------|----------|-----------|
| tb3_multi_robot | Jazzy+Harmonic 표준 launch 패턴 | 이미 유사하게 구현 |
| m-explore-ros2 | frontier 할당 + map_merge | FrontierCoordinator로 중복 방지 |
| Aerostack2 | 이종 드론 플러그인 아키텍처 | PlatformInterface 추상화 |
| Multi-Robot-Graph-SLAM | 분산 3D SLAM | 장기 목표 |
| Open-RMF | FleetAdapter 이종 로봇 조율 | 중기 목표 |
| DARPA SubT CERBERUS | 로컬 자율성 + breadcrumb | 통신 두절 대응 |
| CSIRO CatPack | 이종 플랫폼 동질적 센싱 | 센서 토픽 통일 |

---

## 8. ARGOS 현재 코드 vs 시니어 수준 GAP 분석

### 이미 올바르게 구현된 것 ✅
- SDF 월드: 3개 필수 플러그인 + sensors + imu ✅
- TF remapping: `('/tf', 'tf')` 패턴 ✅
- 드론 SDF: MulticopterMotorModel + VelocityControl ✅
- 오케스트레이터: 감독 자율성 패턴 ✅
- 네임스페이스별 로봇 스택 분리 ✅
- 열화상 센서 + 화점 감지 ✅
- 프론티어 탐사 + 멀티로봇 대상 공유 ✅

### 개선 필요 (리서치 기반) ⚠️
1. costmap 토픽 경로: `scan` → `/robot_name/scan` (절대경로)
2. NavFn → SmacPlanner2D 전환
3. DWB → MPPI Controller 전환 검토
4. Lifecycle Node 미적용 (현재 일반 Node)
5. 패키지 구조: 모든 노드가 my_robot_bringup에 혼재
6. BT 미도입 (현재 if/elif 상태 머신)
7. Watchdog/E-STOP 계층 미구현
8. Keepout zone 미구현 (열화상 → costmap 자동 갱신)
9. 피어 로봇 장애물 인식 미구현
10. launch_testing 통합 테스트 미작성

---

## 출처 (전 4개 보고서 통합)
- tb3_multi_robot (arshadlab/tb3_multi_robot) — Jazzy+Harmonic
- Multi-Robot-Graph-SLAM (aserbremen) — 분산 SLAM
- m-explore-ros2 (robo-friends) — frontier 탐사
- Aerostack2 (aerostack2) — 드론 아키텍처
- Open-RMF (open-rmf.org) — 이종 로봇 조율
- Nav2 공식 문서 — costmap, Smac, MPPI
- Gazebo Harmonic 공식 문서 — SDF, sensors
- DARPA SubT CERBERUS — Science Robotics 2022
- CSIRO Data61 CatPack — arXiv:2104.09053
- Springer 2024 Zenoh vs DDS 비교 논문
- ros-safety/software_watchdogs — QoS watchdog
- BehaviorTree.ROS2 — BT.CPP v4 ROS2 연동
- CBBA-Python (zehuilu) — 분산 태스크 할당
