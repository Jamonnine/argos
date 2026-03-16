# ARGOS 잔여 개선 항목 (코드 수준 불가)

> 기준: 15명 전문가 리뷰 (2026-03-16)
> 현재 평균: 83점 / 목표: 전원 100점
> 잔여 갭: ~17점/전문가 평균 (총 ~255점)

---

## 1. WSL2 개발 세션 필요 (다음 ARGOS 세션)

> 예상 소요: 비번 3~4회 (각 1~2시간)

| # | 항목 | 전문가 | 점수 영향 | 난이도 | 소요 | 상태 |
|---|------|--------|---------|--------|------|------|
| W1 | **Lifecycle Node 전환** (핵심 4개: orchestrator, frontier, hotspot, gas_sensor) | ROS2 92→97 | +5 | 중 | 2시간 | ✅ 완료 (2026-03-16) |
| W2 | **멀티로봇 TF frame_prefix 전환** + 3대 동시 기동 테스트 | Nav 85→90 | +5 | 중 | 1.5시간 | ✅ 코드 완료 (WSL2 기동 테스트 필요) |
| W3 | **Gazebo 연기 시뮬** (OpenCV 후처리 기반 가시거리 제한) | 시뮬 80→88 | +8 | 중 | 1시간 | 대기 (WSL2 필요) |
| W4 | **Gazebo headless RTF 검증** (0.28x → 1.0x 목표) | 시뮬 +3 | +3 | 하 | 30분 | 대기 (WSL2 필요) |
| W5 | **launch_testing 통합 테스트** 3개 (화재 대응, 멀티로봇, 배터리) | QA 89→95, DevOps 74→79 | +11 | 중 | 2시간 | ✅ 완료 (491 테스트 통과) |
| W6 | **pytest-cov 커버리지 리포트** (80% 목표) | QA +3, DevOps +3 | +6 | 하 | 30분 | ✅ 완료 |
| W7 | **PX4 offboard 인터페이스** 기본 구현 (aerial-autonomy-stack 참조) | 드론 78→88 | +10 | 상 | 3시간 | 대기 (WSL2+PX4 필요) |
| W8 | **Kalman 필터** 화점 시간 추적 구현 | 퓨전 84→90 | +6 | 중 | 1시간 | ✅ 완료 (142→380줄, 시계열+근접경고+통계) |
| W9 | **depth gradient 계단 감지** | Nav 85→89 | +4 | 중 | 1시간 | ✅ 기존 구현 확인 (step_detector_node.py) |

**완료 6/9 — 잔여 3건(W3/W4/W7)은 WSL2 세션 필요**
**코드 레벨 예상 평균: 83 → ~88점 (WSL2 검증 후 ~90점)**

---

## 2. 인프라/환경 설정 필요

> 예상 소요: 비번 2회

| # | 항목 | 전문가 | 점수 영향 | 전제 조건 | 상태 |
|---|------|--------|---------|---------|------|
| E1 | **Zenoh 통신 전환** (rmw_zenoh 설치 + DDS 대비 벤치마크) | 통신 76→85 | +9 | ros-jazzy-rmw-zenoh-cpp 설치 | 대기 |
| E2 | **DDS 보안 활성화** (ROS_SECURITY_ENABLED + 인증서 생성) | 보안 78→86 | +8 | SROS2 CLI 도구 | 대기 |
| E3 | **rosbridge TLS** (SSL 인증서 + certfile/keyfile 설정) | 보안 +5 | +5 | openssl 인증서 생성 | ✅ launch 인자 추가 (2026-03-16) |
| E4 | **네트워크 에뮬레이션** (tc netem delay 200ms loss 10%) | 통신 +5 | +5 | 시뮬 스크립트 | ✅ 4종 프리셋 구현 완료 |
| E5 | **TensorRT 변환 파이프라인** (pt→ONNX→TensorRT) | AI 92→96 | +4 | tensorrt 패키지 설치 | 대기 |

**완료 2/5 — 잔여 3건(E1/E2/E5)은 WSL2 패키지 설치 필요**

---

## 3. 하드웨어 설계 필요

> 예상 소요: 1개월 (설계 문서 중심)

| # | 항목 | 전문가 | 점수 영향 | 산출물 |
|---|------|--------|---------|--------|
| H1 | **내열/방수 설계 프레임워크** | HW 64→80 | +16 | thermal_hardening.urdf.xacro + IP54 요구사항서 |
| H2 | **UGV 배터리 시스템 설계** | HW +5 | +5 | 배터리 URDF 링크 + 전력 예산 문서 |
| H3 | **질량 재검증** (base_mass 15kg → 실측 기반) | HW +3 | +3 | 부품별 질량 분석서 |
| H4 | **모터 토크 역산** (바퀴 속도 → 모터 스펙) | HW +4 | +4 | 모터 선정 보고서 |
| H5 | **Legged 플랫폼 인터페이스 설계** | HW +3 | +3 | legged_base.urdf.xacro 초안 |
| H6 | **드론 열화상 카메라 SDF 추가** | 드론 78→83 | +5 | model.sdf 수정 |
| H7 | **센서 온도 모니터링 노드** (과열 시 자동 셧다운) | HW +2 | +2 | thermal_monitor_node.py |

**HW 설계 완료 시 예상 평균: 93 → ~95점**

---

## 4. 외부 기관 활동 필요

> 예상 소요: 3~6개월

| # | 항목 | 전문가 | 점수 영향 | 대상 기관 | 시기 |
|---|------|--------|---------|---------|------|
| X1 | **NFRI 방문** — 연구 데이터 확보 + 시험 환경 경험 | AI 96→98 | +2 | 국립소방연구원 | 2026 Q2 |
| X2 | **NFRI 119리빙랩 적용성 검증** | 규정 73→85 | +12 | 국립소방연구원 | 2026 Q3 |
| X3 | **FMEA 워크샵 실행** (15개 failure mode 실분석) | 규정 +5 | +5 | 내부 (소방서 기술자 참여) | 2026 Q2 |
| X4 | **KFI 형식승인 신청** | 규정 85→95 | +10 | 한국소방산업기술원 | 2027 Q1 |
| X5 | **ISO 13482 컨설팅** | 규정 +5 | +5 | 한국로봇산업진흥원 | 2027 Q2 |
| X6 | **국민안전발명챌린지 출품** | 소방 94→97 | +3 | 특허청+소방청 | 2026.05~06 |
| X7 | **하드웨어 프로토타입 제작** (TurtleBot4 + PX4 드론) | HW 95→98, 드론 88→95 | +10 | 자체 | 2026 Q3 |
| X8 | **달성군 무인비행시험공역 야외 테스트** | 전체 +3 | +3 | ETRI + 대구시 | 2026 Q4 |
| X9 | **소방서 훈련장 실내 테스트** | 전체 +2 | +2 | 강북소방서 | 2026 Q3 |

**외부 활동 완료 시 예상 평균: 95 → ~100점**

---

## 5. 실행 타임라인

```
2026-03-16 (완료) ─── 코드 레벨 개선
  ├── ✅ W1: Lifecycle 4노드, W2: TF frame_prefix, W5: launch_testing 3개
  ├── ✅ W6: pytest-cov, W8: Kalman 강화, W9: step_detector 확인
  ├── ✅ E3: rosbridge TLS, E4: 네트워크 에뮬레이션
  ├── ✅ Gazebo cmd_vel +6.35m 이동 성공, Nav2 DDC 체이닝
  ├── ✅ gz_bridge 간헐적 실패 근본 원인 확정 + 헬스체크 스크립트
  └── 현재: 83 → ~89점

2026 Q2 (4~6월) ─── WSL2 실동작 + 환경 설정
  ├── W3: 연기 시뮬, W4: RTF 검증, W7: PX4 offboard
  ├── E1: Zenoh, E2: DDS 보안, E5: TensorRT
  ├── CycloneDDS 설치 (sudo 필요)
  ├── GUI 모드 Nav2 실동작 검증 + OBS 녹화
  ├── X3: FMEA 워크샵, X6: 국민안전발명챌린지 출품
  └── 예상: 89 → 93점

2026 Q3 (7~9월) ─── HW 설계 + NFRI
  ├── H1~H7: 내열, 배터리, 질량, 드론 열화상
  ├── X1: NFRI 방문 + 데이터 확보
  ├── X2: 119리빙랩 검증
  ├── X7: 하드웨어 프로토타입
  ├── X9: 소방서 훈련장 테스트
  └── 예상: 93 → 97점

2026 Q4 ~ 2027 Q1 ─── 현장 실증 + 인증
  ├── X4: KFI 형식승인
  ├── X5: ISO 13482 컨설팅
  ├── X8: 달성군 야외 테스트
  └── 예상: 97 → 100점
```

---

## 6. 연락처 / 참조

| 기관 | 담당자 | 연락처 | 비고 |
|------|--------|--------|------|
| NFRI 대응기술연구과 | 이지향 팀장 | ljh1028@korea.kr | 무인소방차, 이메일 발송 완료 (3/15) |
| NFRI 대응기술연구과 | 김태동 연구원 | bigbronze1004@korea.kr | 드론 객체인식, 이메일 발송 완료 (3/15) |
| KFI | — | — | 형식승인 문의: nfa.go.kr |
| ETRI 대경연구센터 | — | — | AI 군집드론 R&D 40억 |
| 무지개연구소 | — | 대구 수성알파시티 | NVIDIA GPU 드론 두뇌 |
| 달성군 비행시험공역 | — | — | 관제센터+격납고+이착륙장 |
| 소방청 R&D (IRIS) | KEIT | 053-718-8632 | 2027년 2월 공모 예상 |
| 국민안전발명챌린지 | — | idearo.kr | 2026년 5~6월 예상 |

---

---

## 7. 모범 답안 매핑 (이미 수집된 리서치/참조 모델)

> 각 개선 항목에 "어떻게 하는지"의 구체적 참조가 존재합니다.

### WSL2 작업 모범 답안

| 항목 | 모범 답안 | 참조 위치 |
|------|---------|---------|
| **W1 Lifecycle Node** | Nav2의 lifecycle_manager가 이미 구현되어 있음. orchestrator를 LifecycleNode로 전환 시 `on_activate/on_deactivate` 패턴은 Nav2 소스 참조 | `knowledge/projects/research/2026-03-05-ros2-robotics-software-architecture-senior.md` (Lifecycle 섹션) |
| **W2 멀티로봇 TF** | DARPA SubT CERBERUS 팀: `frame_prefix` 방식 + TF aggregator. CSIRO CatPack: 이종 플랫폼 동질적 센싱 | `knowledge/projects/research/2026-03-05-ros2-multirobot-senior-deep-dive.md` (TF2 하이브리드 전략) |
| **W3 연기 시뮬** | Gazebo Harmonic `ParticleEmitter` 플러그인. 또는 OpenCV `GaussianBlur` + alpha 후처리 (경량 대안) | `knowledge/projects/research/2026-03-05-gazebo-harmonic-advanced-guide.md` |
| **W5 launch_testing** | ROS2 공식 `launch_testing` 라이브러리. Gazebo headless SIL 패턴은 Nav2 테스트 스위트 참조 | `knowledge/projects/dev-experience/ros2.md` (테스팅 피라미드 섹션) |
| **W7 PX4 통합** | **uXRCE-DDS가 유일한 공식 브릿지**. ETH Zurich `px4-offboard` 예제 + `aerial-autonomy-stack` Docker 3-컨테이너 구조 | `knowledge/projects/research/2026-03-15-px4-ros2-drone-integration-deep-dive.md` (전체) |
| **W8 Kalman 필터** | Fire SLAM 논문: YOLOv8n + Kalman 필터로 3D 화점 위치 실시간 추정. ARGOS B-3 구현 직접 참조 가능 | `knowledge/projects/research/2026-03-15-fire-detection-ai-models-datasets.md` (Fire SLAM 섹션) |
| **W9 계단 감지** | depth image gradient 계산 → 수직 픽셀 점프 감지. Nav2 costmap custom layer로 `step_hazard` 추가 | Nav 전문가 리뷰 (에이전트 출력 파일) |

### 환경 설정 모범 답안

| 항목 | 모범 답안 | 참조 위치 |
|------|---------|---------|
| **E1 Zenoh** | `rmw_zenoh`: DDS 대비 WiFi 환경에서 97~99% 발견 트래픽 감소. DARPA SubT에서 검증. PX4 ROS2 Interface Library (Auterion)도 Jazzy에서 rmw_zenoh 사용 | `memory/argos-tutor.md` (Zenoh 항목) + PX4 리서치 |
| **E2 DDS 보안** | SROS2 CLI: `ros2 security create_keystore` → 인증서 자동 생성. `ROS_SECURITY_STRATEGY=Enforce` 환경변수 | ROS2 공식 문서 (docs.ros.org/en/jazzy/Tutorials/Security/) |
| **E3 rosbridge TLS** | rosbridge launch 인자: `ssl:=true certfile:=/path/to/cert.pem keyfile:=/path/to/key.pem`. 자체 서명 인증서: `openssl req -x509 -newkey rsa:4096 -nodes` | demo.launch.py (이미 인자 구조 확인됨) |
| **E5 TensorRT** | `model.export(format='engine')` 한 줄. ultralytics가 TensorRT 변환 자동 지원. Jetson에서는 JetPack SDK 포함 | `knowledge/projects/research/2026-03-15-fire-detection-ai-models-datasets.md` (FCMI-YOLO TensorRT 섹션) |

### HW 설계 모범 답안

| 항목 | 모범 답안 | 참조 위치 |
|------|---------|---------|
| **H1 내열/방수** | **현대 HR-셰르파**: 내열 800°C, 6륜 인휠모터. 차열판 = 세라믹 섬유 + 공기 gap. IP65 방수. 로봇 몸체 알루미늄+스테인리스 | `knowledge/projects/research/2026-03-15-nfa-drone-robot-policy-2025-2026.md` (HR-셰르파 상세) |
| **H2 배터리** | TurtleBot4: 4S LiPo 14.8V 6800mAh, ~2.5시간 연속. ARGOS UGV 예상: 13W 소비 → 2600mAh@48V = 2시간 | HW 전문가 리뷰 (전력 예산 표) |
| **H3 질량** | TurtleBot3 Waffle: 4.5kg (센서 제외). ARGOS 0.6×0.4m 스케일: 예상 8~10kg (섀시) + 4kg (배터리) + 1.5kg (전자) = ~13.5kg | HW 전문가 리뷰 (질량 재검증 섹션) |
| **H6 드론 열화상** | Gazebo `thermal_camera` 센서 타입. FOV 90° 광각, 256×192 해상도. 하단 장착 (RGB 아래 8cm offset) | HW 전문가 리뷰 (드론 플랫폼 섹션) |

### 외부 활동 모범 답안

| 항목 | 모범 답안 | 참조 위치 |
|------|---------|---------|
| **X1 NFRI 방문** | 이메일 발송 완료 (3/15). 이지향 팀장: "연구 같이하자" 제안. 방문 시 ARGOS 데모(daegufire.ai.kr/argos) + 테스트 계정 제공 | `knowledge/firefighter/nfri-connection.md` (액션 플랜 Phase 2) |
| **X2 119리빙랩** | 접수: nfire.go.kr 상시 운영. 소방기관 또는 기업이 기술/제품 실증 의뢰 → 국립소방연구원이 검토 후 지원 | `knowledge/projects/research/2026-03-14-nfri-national-fire-research-institute.md` (119리빙랩 섹션) |
| **X3 FMEA** | 기초 FMEA 15개 failure mode + RPN 이미 작성됨. 워크샵에서 소방서 기술자와 함께 RPN 재산정 + Mitigation 보완 | `docs/fmea-initial.md` (이번 세션 생성) |
| **X4 KFI 인증** | 소방청 소관 R&D → IRIS 공모 (매년 2월). 개인 불가, **NFRI 공동연구자** 경로. 국민안전발명챌린지(5월)는 개인 직접 참여 가능 | `knowledge/projects/research/2026-03-15-nfa-iris-rd-application-guide.md` |
| **X6 발명챌린지** | idearo.kr 신청. 2025년 대상 = 부여소방서 소방장. ARGOS를 "이종 로봇 오케스트레이션 아이디어"로 재구성 | 위 리서치 (대안 경로 4가지 표) |
| **X7 하드웨어** | 비용 0원 원칙이지만, R&D 과제 예산 확보 시: TurtleBot4 ~$1,200 + Holybro X500 V2 드론 ~$500 | `knowledge/projects/research/2026-03-15-firefighting-robot-heterogeneous-swarm-trends.md` (HW 비교 섹션) |
| **X8 달성군 실증** | 달성군 구지면 — 격납고+이착륙장+관제센터 완비. ETRI 대경연구센터 연계. 전국 4개 드론 시험공역 중 하나 | `knowledge/projects/research/2026-03-15-daegu-drone-ai-firefighting-trends.md` (달성군 섹션) |

---

*이 문서는 ARGOS 100점 달성을 위한 잔여 로드맵입니다.*
*코드 수준 개선은 `docs/expert-review-2026-03-16.md`에 기록.*
*세션별 진행 시 해당 항목 완료 체크 후 리뷰 문서 갱신.*
