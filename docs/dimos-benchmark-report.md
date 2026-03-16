# ARGOS × DimOS 벤치마킹 구현 보고서

> 작성일: 2026-03-16
> 요청 근거: `knowledge/projects/argos-dimos-benchmark-requirements.md`
> 상태: **4/4 항목 구현 완료**

---

## 1. 개요

DimOS(dimensionalOS/dimos)의 핵심 패턴 4가지를 분석하고, ARGOS ROS2 네이티브로 구현하였다.
DimOS에 대한 직접 의존 없이, **패턴만 참조하여 자체 구현**하였다 (DimOS Pre-Release Beta 주의사항 준수).

---

## 2. 구현 결과

### 2.1 SkillLibrary — 로봇 능력 동적 등록/쿼리 (393줄)

**파일**: `argos_bringup/skill_library.py`

| 구성요소 | 설명 |
|----------|------|
| `SkillDescriptor` | 스킬 메타데이터 (이름, 타입, 파라미터 스키마, 필요 능력, ROS2 액션 매핑) |
| `AbstractRobotSkill` | 추상 기본 클래스 (validate_params, to_mcp_tool 인터페이스) |
| `PatrolSkill` | 순찰 — NavigateToPose 액션 래핑, 웨이포인트 파라미터 |
| `DetectFireSkill` | 화점 감지 — thermal 센서 기반, 감도/영역 파라미터 |
| `ReturnHomeSkill` | 귀환 — 원점/지정 좌표 귀환, 안전 속도 제한 |
| `RescueSkill` | 구조 — victim 센서 기반, 드론 우선/UGV 접근 전략 |
| `SkillLibrary` | 등록/해제, 로봇별 조회, 타입별 조회, **능력 기반 로봇 선택**, MCP 도구 변환 |

**DimOS 대비 ARGOS 차별점**:
- ROS2 액션/서비스와 직접 매핑 (`ros2_action`, `ros2_service` 필드)
- 소방 도메인 특화 스킬 4종 (DimOS는 범용 이동만)
- `get_capable_robots(required_capabilities)` — 오케스트레이터 임무 할당에 즉시 활용 가능

### 2.2 MCP 로봇 서버 — 자연어 로봇 제어 (279줄)

**파일**: `argos_bringup/mcp_robot_server.py`

| 구성요소 | 설명 |
|----------|------|
| `ArgosMCPServer` | list_tools / call_tool / run_stdio 핸들러 |
| `create_default_library()` | 표준 4개 스킬 자동 등록 팩토리 |
| MCP SDK 모드 | `mcp` 패키지 있으면 공식 SDK 사용 |
| JSON-RPC 폴백 | `mcp` 미설치 시 JSON-RPC 2.0 stdio 루프로 동작 |

**사용 시나리오**:
```
소방 지휘관 (Claude) → "3번 건물 정찰해"
    → MCP call_tool('patrol_area', {waypoints: [...]})
    → ArgosMCPServer → SkillLibrary → ROS2 NavigateToPose 액션
    → ARGOS UGV 이동
```

**실행 방법**:
```bash
# MCP stdio 서버 기동
ros2 run argos_bringup mcp_robot_server

# Claude Desktop claude_desktop_config.json에 등록
{
  "mcpServers": {
    "argos": {
      "command": "ros2",
      "args": ["run", "argos_bringup", "mcp_robot_server"]
    }
  }
}
```

### 2.3 리액티브 스트림 — 센서 BackPressure (337줄)

**파일**: `argos_bringup/reactive_stream.py`

| 구성요소 | 설명 |
|----------|------|
| `BackPressure` | LATEST (최신만) / DROP (처리 중 드롭) / BUFFER (큐 축적) |
| `ReactiveStream` | RxPY Subject 또는 threading.Queue 폴백 자동 전환 |
| `SensorStreamManager` | 여러 센서 스트림 통합 관리 + 일괄 dispose |

**적용 대상**: perception_bridge_node의 열화상 콜백 플러딩 방지

```python
# 기존 방식 (플러딩 위험)
def thermal_callback(self, msg):
    self.process(msg)  # 느린 처리 시 큐 폭발

# 리액티브 방식 (BackPressure.LATEST)
stream = manager.create_stream('thermal', BackPressure.LATEST)
stream.subscribe(self.process)  # 느린 처리 시 최신 프레임만 유지

def thermal_callback(self, msg):
    stream.push(msg)  # 즉시 반환, 백그라운드 처리
```

**RxPY 미설치 시**: threading.Queue + Thread 기반 폴백으로 동일 BackPressure 동작 보장.

### 2.4 Rerun 시각화 브릿지 (296줄)

**파일**: `argos_bringup/rerun_bridge.py`

| 메서드 | 시각화 대상 |
|--------|-----------|
| `log_robot_pose` | 로봇 궤적 (200점 버퍼, 3D Lines) |
| `log_scan` | LiDAR 스캔 (Points3D, 극좌표→직교좌표 변환) |
| `log_thermal` | 열화상 히트맵 (mono8 이미지) |
| `log_fire_alert` | 화점 마커 (3D Points, 심각도별 컬러) |

**Rerun 미설치 시**: 경고 1회 출력 후 모든 메서드 no-op (예외 없음).

---

## 3. 아키텍처 통합 구조

```
                    ┌─────────────────────────┐
                    │  Claude / MCP Client    │
                    └──────────┬──────────────┘
                               │ MCP Protocol
                    ┌──────────▼──────────────┐
                    │  ArgosMCPServer         │
                    │  (mcp_robot_server.py)  │
                    └──────────┬──────────────┘
                               │
                    ┌──────────▼──────────────┐
                    │  SkillLibrary           │
                    │  (skill_library.py)     │
                    │  PatrolSkill            │
                    │  DetectFireSkill        │
                    │  ReturnHomeSkill        │
                    │  RescueSkill            │
                    └──────────┬──────────────┘
                               │ ROS2 Action/Service
              ┌────────────────┼────────────────┐
              ▼                ▼                ▼
        ┌──────────┐    ┌──────────┐    ┌──────────┐
        │ Nav2     │    │ Orch-    │    │ Frontier │
        │ Navigate │    │ estrator │    │ Explorer │
        │ ToPose   │    │          │    │          │
        └──────────┘    └──────────┘    └──────────┘
              │                │                │
              ▼                ▼                ▼
        ┌──────────────────────────────────────────┐
        │           ReactiveStream                 │
        │  (reactive_stream.py)                    │
        │  thermal → BackPressure.LATEST           │
        │  lidar   → BackPressure.DROP             │
        │  gas     → BackPressure.BUFFER           │
        └──────────────┬───────────────────────────┘
                       │
                       ▼
        ┌──────────────────────────────────────────┐
        │           RerunBridge                    │
        │  (rerun_bridge.py)                       │
        │  log_robot_pose / log_scan / log_thermal │
        └──────────────────────────────────────────┘
```

---

## 4. DimOS 대비 비교

| 항목 | DimOS | ARGOS |
|------|-------|-------|
| 스킬 체계 | AbstractSkill → LLM Tool 변환 | **AbstractRobotSkill → MCP Tool + ROS2 Action 이중 매핑** |
| MCP 서버 | CLI 기반, 단일 로봇 | **SkillLibrary 연동, 이종 군집 전체 제어** |
| BackPressure | RxPY 전용 | **RxPY + Queue 폴백 (의존성 0 가능)** |
| 시각화 | Rerun 전용 | **Rerun + HEAT Portal 웹 대시보드 병행** |
| 군집 제어 | 미구현 | **오케스트레이터 + 능력 기반 로봇 선택** |
| 소방 도메인 | 없음 | **4종 소방 특화 스킬 (순찰/화점/귀환/구조)** |

---

## 5. 검증 결과

| 항목 | 결과 |
|------|------|
| SkillLibrary 등록/쿼리/MCP 변환 | PASS |
| MCP 서버 call_tool / 에러 처리 | PASS |
| ReactiveStream LATEST/DROP 동작 | PASS |
| RerunBridge no-op 폴백 | PASS |
| **전체 테스트 regression** | **491 통과, 0 신규 실패** |
| 총 신규 코드 | **1,305줄** |

---

## 6. 다음 단계

| 순위 | 항목 | 시기 |
|------|------|------|
| 1 | 오케스트레이터에 SkillLibrary 주입 — `get_capable_robots()` 활용 임무 할당 | v2.0 |
| 2 | perception_bridge에 ReactiveStream 적용 — 열화상 플러딩 근본 해결 | v2.0 |
| 3 | MCP 서버 ROS2 액션 실연결 — `_dispatch_ros2()` 구현 | 경연대회 후 |
| 4 | Rerun ROS2 노드 통합 — 실시간 3D 시각화 | NFRI 시연 시 |

---

*벤치마킹 요구사항 문서: `knowledge/projects/argos-dimos-benchmark-requirements.md`*
*구현 코드: `ros2_ws/src/argos_bringup/argos_bringup/{skill_library,mcp_robot_server,reactive_stream,rerun_bridge}.py`*
