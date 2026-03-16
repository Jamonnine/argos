# Changelog

## [2.0.0] — 2026-03-16 (개발 중)

### Added
- **8중 센싱 체계**: 가스(CO/O2/LEL/CO2/HCN) + 피해자(열화상 인체) + 구조물(LiDAR 변위) + 음향(8종 이벤트)
- **센서 퓨전 가중 합산**: `compute_situation_score()` — 5개 센서 가중치 기반 종합 위험도
- **지휘관 승인 체계**: critical 시 PAUSED → 지휘관 승인 → RETURNING (Supervised Autonomy 준수)
- **입력 검증 모듈**: `validation_utils.py` — robot_id, severity, 센서값 범위, 타임스탬프 검증
- **심각도 공통 모듈**: `severity_utils.py` — DRY 원칙 준수
- **웹 대시보드 센싱 패널**: Environment Sensors (가스/피해자/구조물/음향 실시간 표시)
- **CI/CD**: GitHub Actions `build-and-test.yml` + Dockerfile + .dockerignore
- **드론 PID 제어**: P→PID 전환 (I/D gain 추가) + 배터리 RTL 모델
- 4개 새 메시지: GasReading, VictimDetection, StructuralAlert, AudioEvent
- 4개 새 노드: gas_sensor, victim_detector, structural_monitor, audio_detector
- 테스트 +260개 (239→511): 센싱 48 + 시나리오 27 + 보안 25 + launch_testing 41 + Kalman 24 + 기타
- `sensing.launch.py` + `sensors.yaml` 설정 파일
- 15명 전문가 리뷰 문서 (`docs/expert-review-2026-03-16.md`)
- **LifecycleNode 전환**: orchestrator, frontier_explorer, hotspot_detector, gas_sensor — 4노드 전환 (on_configure/activate/deactivate/cleanup/shutdown)
- **TF frame_prefix**: multi_robot + exploration launch — 멀티로봇 TF 격리 방식 전환 (namespace remapping → frame_prefix)
- **launch_testing 통합 테스트 3개**: 화재 대응(377줄), 멀티로봇 경합(377줄), 배터리 lifecycle(401줄)
- **pytest-cov 설정**: pyproject.toml coverage 섹션 + setup.py pytest-cov 의존성
- **Kalman 시계열 강화**: 142→380줄 (시간기반 추적, 화점-피해자 근접경고, 통계, 확산추세)
- **rosbridge TLS**: demo.launch.py에 ssl/certfile/keyfile launch 인자 추가
- **gz_bridge 헬스체크**: `check_gz_bridge_health.sh` — 토픽 검증 + 자동 재시작
- **Nav2 DDC 체이닝**: navigation.launch.py — TimerAction(5s) → OnProcessExit(ddc_spawner) + Timer(45s)
- **FastDDS SHM 프로파일**: `fastdds_no_shm.xml` — WSL2 SHM 실패 우회용

### Changed
- 오케스트레이터: 8중 센싱 콜백 4개 추가 (gas/victim/structural/audio)
- MissionState.msg: gas_danger_level, victims_count, blocked_areas, audio_alerts 필드 추가
- E-Stop: 1회→3회 연속 발행 (네트워크 손실 대비)
- rosbridge: localhost 바인딩 + 토픽/서비스 화이트리스트 + max_message_size 1MB
- package.xml: 버전 2.0.0, 라이선스 Apache-2.0, 설명 갱신
- TF2 lookup: 타임아웃 0.1초 명시 + 구체적 예외 로깅
- 드론 고도 공차: 0.3m → 0.15m
- slam_toolbox transform_timeout: 0.2 → 2.0초 (WSL2 RTF 보상)
- nav2_params: `<robot_namespace>/` frame_prefix 치환 + slam_toolbox use_sim_time 명시

### Fixed
- agent-call-log-hook: 백그라운드 에이전트 완료 알림 시 에러 방지
- ExternalShutdownException: 4개 센싱 노드 그레이스풀 셧다운
- Nav2 DDC 타이밍: OnProcessExit 체이닝으로 DDC 완료 후 Nav2 기동 보장
- launch_testing ROS2 메시지 필드명: `pos_x` → `pose.pose.position.x`, `fire_x` → `location.point.x`

## [1.0.0] — 2026-03-15

### Milestones (MS-1~10 완료)
- MS-1: PatrolArea Action Server/Client
- MS-2: UGV URDF (3계층 모듈식 xacro)
- MS-3: Nav2 + SLAM (SmacPlanner2D + MPPI)
- MS-4: 열화상 시뮬레이션 + 화점 감지
- MS-5: 멀티로봇 네임스페이스 분리
- MS-6: 프론티어 기반 자율 탐색
- MS-7: 오케스트레이터 (Supervised Autonomy)
- MS-8: 드론 플랫폼 (이종 로봇)
- MS-9: 소방 시나리오 7단계 시연
- MS-10: 웹 대시보드 (HEAT Portal 통합)

### Stats
- 239 tests, 7 nodes, 10 launch files, 10 custom interfaces
- ~110 hours development
