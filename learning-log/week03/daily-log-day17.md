# Day 17 - Behavior Trees · Costmap · SLAM+Nav2 통합

**날짜**: 2026-02-19
**학습 시간**: ~1시간
**핵심 주제**: Nav2 Behavior Trees, Costmap 레이어 아키텍처, SLAM + Nav2 동시 실행 (완전 자율 탐색)

---

## 🎯 오늘의 목표 (3-in-1)

오늘은 세 가지 주제를 하나의 완성된 자율 주행 아키텍처 레이어 모델로 이해하는 것이 목표였습니다.

- **Phase 1**: Behavior Trees — Nav2의 의사결정 엔진
- **Phase 2**: Costmap 레이어 아키텍처 — 환경 인식과 비용 계산
- **Phase 3**: SLAM + Nav2 동시 실행 — 진짜 자율 탐색 로봇

---

## ✅ 달성 결과

- Nav2 BT XML 파일 분석 (`navigate_to_pose_w_replanning_and_recovery.xml`)
- Global/Local Costmap 파라미터 구조 이해
- `slam_nav2_sim.launch.py` 작성 (slam=True 파라미터 사용)
- SLAM + Nav2 + RViz2 동시 실행 성공
- RViz2 코스트맵 시각화 확인 (Inflation Layer 동심원 그라데이션)
- Nav2 Goal 자율 주행 성공 (SLAM 지도 기반)

---

## 📐 Phase 1: Behavior Trees — State Machine의 한계 극복

### State Explosion Problem

Day 4에서 배운 State Machine은 상태 수가 늘어나면 전환(transition) 수가 기하급수적으로 증가합니다. N개 상태 → 최악 N² 전환. 자율 주행 로봇의 복잡한 행동을 State Machine으로 표현하면 10개 상태에 100개 전환이 필요해 유지보수가 불가능해집니다.

Behavior Tree는 **계층적 구조**로 이 문제를 해결합니다.

### BT 4가지 노드 타입

```
Sequence (→)  : 모두 성공해야 성공 ("A AND B AND C")
Fallback (?)  : 하나만 성공해도 성공 ("A OR B OR C")
Action        : 실제 작업 (성공/실패 반환)
Condition     : 상태 확인 (성공/실패 반환)
```

### 실제 Nav2 BT 분석 (`navigate_to_pose_w_replanning_and_recovery.xml`)

```
RecoveryNode (최대 6회 재시도)
├── PipelineSequence "NavigateWithReplanning"
│   ├── ControllerSelector / PlannerSelector
│   ├── RateController (1Hz) → ComputePathToPose
│   │                          실패 시 → ClearGlobalCostmap
│   └── FollowPath
│       실패 시 → ClearLocalCostmap
│
└── 복구 시퀀스 (6회 모두 실패 시)
    └── RoundRobin
        ├── ClearLocalCostmap + ClearGlobalCostmap
        ├── Spin 90도 (주변 스캔)
        ├── Wait 5초
        └── BackUp 30cm
```

이 구조는 소프트웨어 공학 패턴과 1:1 대응됩니다.
- `RecoveryNode` = Try-Catch
- `PipelineSequence` = 데이터 파이프라인 패턴
- `RoundRobin` = Round-Robin 스케줄링

**Nav2 BT XML 파일 목록** (10개 이상 제공됨):
- `navigate_to_pose_w_replanning_and_recovery.xml` — 기본값
- `navigate_to_pose_w_replanning_goal_patience_and_recovery.xml` — 목표 인내심 추가
- `navigate_w_replanning_only_if_goal_is_updated.xml` — 목표 변경 시만 재계획
- `navigate_w_replanning_time.xml` — 시간 기반 재계획
- `navigate_w_replanning_distance.xml` — 거리 기반 재계획

**핵심**: BT XML 파일만 교체하면 로봇의 고수준 행동 전략 전체를 바꿀 수 있습니다. 코드 수정 없이.

---

## 🗺️ Phase 2: Costmap 레이어 아키텍처

### Global vs Local Costmap: 전략 vs 전술 분리

| | Global Costmap | Local Costmap |
|---|---|---|
| 프레임 | `map` | `odom` |
| 업데이트 | 1Hz | 5Hz |
| 범위 | 전체 지도 | 3×3m (rolling) |
| 역할 | 전체 경로 계획 | 실시간 장애물 회피 |

이것은 소프트웨어 아키텍처의 **전략(Strategy) vs 전술(Tactics)** 분리 원칙과 동일합니다. 두 레이어를 분리함으로써, 계산 비용을 최소화하면서 전역 경로 계획과 즉각적 반응을 동시에 달성합니다.

### 레이어 플러그인 스택

```
[static_layer]    ← YAML 지도 → 고정 장애물
[obstacle_layer]  ← /scan 실시간 → 동적 장애물 감지/제거
[voxel_layer]     ← /scan 3D → 높이 있는 장애물
[inflation_layer] ← 장애물 팽창 → 안전 여유 (0.5m 반경)
─────────────────────────────────────────────
최종 비용 지도: 0=자유, 100=장애물, 중간=비용
```

### RViz2에서 확인한 Costmap 색상

- **검은색/짙은 보라** = 실제 장애물 (cost 100)
- **빨간색 그라데이션** = 고비용 (장애물 근처)
- **하늘색** = 저비용 (안전 영역)
- **흰색** = cost 0 (자유 공간)
- **빨간 점선** = 실시간 LiDAR 스캔 포인트

동심원 그라데이션이 `inflation_layer`의 실체이며, 경로 플래너가 자연스럽게 장애물에서 멀리 떨어진 경로를 선택하도록 유도합니다. 이것이 하드코딩 없이 경로가 장애물을 피하는 이유입니다.

---

## 🔗 Phase 3: SLAM + Nav2 동시 실행

### 핵심 발견: `bringup_launch.py`의 `slam:=True` 파라미터

Nav2의 `bringup_launch.py`가 이미 SLAM 모드를 지원합니다:

```python
# slam=True → slam_launch.py 포함 (slam_toolbox)
# slam=False → localization_launch.py 포함 (AMCL + map_server)
# 둘은 절대 동시에 실행되지 않음
```

### slam_nav2_sim.launch.py 핵심

```python
nav2 = IncludeLaunchDescription(
    ...,
    launch_arguments={
        'slam': 'True',    # AMCL 대신 slam_toolbox 사용
        # 'map': 생략        # 지도 파일 불필요!
        'autostart': 'True',
    }.items(),
)
```

`map` 파라미터를 제거한 것이 핵심입니다. Day 15에서는 `map.yaml`을 반드시 제공해야 했지만, SLAM 모드에서는 slam_toolbox가 실시간으로 `/map`을 제공하므로 불필요합니다.

### AMCL vs slam_toolbox 역할 비교

두 컴포넌트가 Nav2에 제공하는 것은 동일합니다:
- `/map` 토픽 (OccupancyGrid)
- `map → odom` TF

차이는 AMCL은 고정된 지도 위에서 위치를 추정하고, slam_toolbox는 지도를 실시간으로 갱신하면서 위치를 추정한다는 것입니다. Nav2 입장에서는 "map 토픽이 계속 변한다"는 점만 다릅니다.

### autostart가 Lifecycle을 자동 처리

Day 16에서 `ros2 lifecycle set /slam_toolbox configure/activate`를 수동으로 실행했던 것이, `autostart: True` + `lifecycle_manager_slam`에 의해 자동으로 처리됩니다.

### 실행 결과

- `slam_toolbox`, `controller_server`, `planner_server`, `bt_navigator` 등 전체 스택 동작
- `/map` Publisher count: 1 확인
- RViz2 코스트맵 시각화 정상
- **Nav2 Goal 자율 주행 성공** — SLAM으로 실시간 생성되는 지도 기반

---

## 🏗️ 오늘의 아키텍처 레이어 모델

```
┌─────────────────────────────────────────┐
│  Layer 3: 의사결정 (Behavior Trees)       │
│  navigate_to_pose_w_replanning.xml       │
│  → 언제 재계획? 언제 복구? 어떤 순서로?   │
├─────────────────────────────────────────┤
│  Layer 2: 환경 인식 (Costmap)             │
│  Global (1Hz, 전체) + Local (5Hz, 3m)    │
│  → 어디가 안전한가? 어디가 위험한가?      │
├─────────────────────────────────────────┤
│  Layer 1: 환경 구성 (SLAM)                │
│  slam_toolbox → /map + map→odom TF       │
│  → 지도가 없어도 스스로 만든다             │
└─────────────────────────────────────────┘
```

각 레이어는 독립적으로 교체 가능합니다:
- SLAM 알고리즘 교체: slam_toolbox → cartographer, rtabmap
- BT 전략 교체: XML 파일 하나 교체
- Costmap 플러그인 교체: yaml 파라미터만 수정

---

## 🗂️ 생성된 파일

```
ros2_ws/src/my_robot_bringup/launch/
└── slam_nav2_sim.launch.py    ← SLAM + Nav2 통합 실행 (Day 17)
```

---

## 📊 학습 진행도

- Week 3 Day 15: Nav2 완전 통합 (pre-built map)
- Week 3 Day 16: SLAM으로 지도 생성
- **Week 3 Day 17: BT + Costmap + SLAM+Nav2 통합 ✅**

---

*Day 17 완료 - 자율 주행 아키텍처의 세 레이어를 이해하고 완전 자율 탐색 로봇을 구현했습니다.*
