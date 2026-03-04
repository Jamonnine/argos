# 리서치 보고서: MS-6 멀티로봇 분산 탐색 (Frontier Exploration)
- 조사일: 2026년 3월 4일 (수)
- 키워드: frontier exploration, multi-robot, ROS2 Jazzy, Gazebo Harmonic, map merge, Nav2, ARGOS
- 연관 마일스톤: MS-6 (멀티로봇 자율 탐색)

---

## 핵심 요약

ROS 2 Jazzy + Gazebo Harmonic 환경에서 멀티로봇 분산 탐색을 구현하는 데 있어 가장 현실적인 조합은 **m-explore-ros2 (explore 모듈) + slam_toolbox 독립 SLAM + 커스텀 영역 분할 로직 + Nav2 namespace 구성**이다. 주요 패키지들은 공식적으로 Jazzy 지원을 선언하지 않았지만, Humble 기반으로 소스 빌드하면 Jazzy에서도 동작 가능하다. CMU M-TARE는 가장 정교하지만 Docker 의존성과 ROS1 브리지 구조로 인해 ARGOS 환경에서 오버킬(overkill)이다. 2~3대 UGV 규모에서는 **단순 코스트맵 마킹 기반 영역 분할**이 가장 빠르게 구현 가능하다.

---

## 1. Frontier Exploration 오픈소스 패키지

### 1-1. m-explore-ros2 (explore_lite ROS 2 포팅)

| 항목 | 내용 |
|------|------|
| GitHub | [robo-friends/m-explore-ros2](https://github.com/robo-friends/m-explore-ros2) |
| 최신 커밋 | 공식 릴리즈 없음, 커밋 38개 (날짜 불명확) |
| Jazzy 호환 | **공식 미지원** — 테스트 기준: Eloquent, Dashing, Foxy, Galactic |
| 실질적 Jazzy 사용 | Humble 소스 빌드 후 Jazzy에서 동작 사례 있음 (커뮤니티 확인) |

**구조 (3개 패키지)**
- `explore` — 프론티어 기반 자율 탐색, `explore/resume` 토픽으로 중단/재개
- `explore_lite_msgs` — 커스텀 메시지 정의
- `map_merge` — 멀티로봇 지도 병합 (known/unknown 초기 위치 모두 지원)

**장점**
- explore + map_merge 한 패키지에 통합
- unknown initial pose 지원 (로봇 간 충분히 가까울 때 특징점 매칭으로 자동 정렬)
- TurtleBot3, JetBot 실하드웨어 검증 사례 있음

**단점**
- Jazzy 공식 지원 없음 → 소스 빌드 + 의존성 수동 해결 필요
- 릴리즈 버전 없음 → 안정성 미보장
- 멀티로봇 시 영역 분할 로직 없음 (중복 탐색 가능)
- slam_toolbox 연동은 experimental 브랜치 수준

**구현 난이도**: 3/5 (Jazzy 빌드 과정 포함)
**ARGOS 적합도**: 3/5

---

### 1-2. frontier_exploration (ROS 2 버전)

| 항목 | 내용 |
|------|------|
| 상태 | ROS 2 버전 **공식 존재하지 않음** |
| 대안 | nav2_wavefront_frontier_exploration (비공식 포팅) |
| GitHub | [SeanReg/nav2_wavefront_frontier_exploration](https://github.com/SeanReg/nav2_wavefront_frontier_exploration) |
| 최신 커밋 | 2020년 11월 21일 (사실상 **유지보수 중단**) |
| Jazzy 호환 | Foxy 기준 개발, Jazzy 미지원 |

**판단**: 사용 비권장. 2020년 이후 업데이트 없음. 단독 레퍼런스 코드로는 유용하나 실제 프로젝트 적용 불가.

**구현 난이도**: 4/5 (대규모 수정 필요)
**ARGOS 적합도**: 1/5

---

### 1-3. nav2_frontier_exploration (Nav2 공식 플러그인)

| 항목 | 내용 |
|------|------|
| 상태 | Nav2 공식 프론티어 플러그인 **존재하지 않음** |
| Nav2 공식 입장 | 프론티어 탐색은 Nav2 외부 노드에서 NavigateToPose 호출 방식 권장 |

**판단**: 공식 플러그인 형태가 아닌, 외부 노드(explore_lite 등)가 Nav2 액션 서버를 호출하는 구조가 표준이다.

---

### 1-4. CMU M-TARE (mtare_planner)

| 항목 | 내용 |
|------|------|
| GitHub | [caochao39/mtare_planner](https://github.com/caochao39/mtare_planner) |
| 공식 사이트 | [cmu-exploration.com/m-tare-planner](https://www.cmu-exploration.com/m-tare-planner) |
| 최신 커밋 | 2024년 1월 8일 (noetic 브랜치) |
| ROS 2 지원 | **간접 지원** — Docker 내 ros1_bridge 방식 |
| Jazzy 호환 | Ubuntu 24.04 + Jazzy 환경 Docker 이미지 제공 |

**핵심 아키텍처**
```
2계층 계획:
  ┌─────────────────────────────────────────┐
  │  Global Layer (희소 격자)               │
  │  └─ 탐색 상태 공유 (로봇 간 통신)       │
  │  └─ VRP(차량 경로 문제) 기반 목표 배분  │
  └────────────────┬────────────────────────┘
                   │
  ┌────────────────▼────────────────────────┐
  │  Local Layer (밀집 격자)                │
  │  └─ 정밀 경로 계획                      │
  │  └─ 장애물 회피                         │
  └─────────────────────────────────────────┘

로봇 조정 전략: "Pursuit" — 정보 교환 이득이 클 때
              로봇들이 서로 접근하여 맵 공유
```

**장점**
- 멀티로봇 자체 내장 (2~20대 지원, 통신 범위 10~300m)
- 실내 환경 전용 모드 지원
- Docker 기반 환경 일관성 보장
- CMU 논문 수준의 알고리즘 품질

**단점**
- ROS1 기반 핵심 코드 + ros1_bridge 래퍼 구조 → 복잡도 높음
- Docker + NVIDIA 컨테이너 의존성 필수
- 자체 경로 계획기 사용 → Nav2 직접 연동 불가
- 2대 소규모 UGV에는 과도한 복잡성

**구현 난이도**: 5/5 (아키텍처 이해 + Docker 환경 구성)
**ARGOS 적합도**: 2/5 (오버킬, Nav2 비연동)

---

## 2. 멀티로봇 탐색 영역 분할 알고리즘 비교

### 용어 설명
- **프론티어(Frontier)**: 알려진 자유 공간과 미지 공간의 경계선. 로봇이 향해야 할 "다음 탐색 지점"
- **코스트맵(Costmap)**: 지도 위에 비용(cost) 수치를 입힌 격자. 높은 수치 = 가기 힘든 곳
- **Voronoi 분할**: 각 로봇이 담당할 영역을 "자신과 가장 가까운 점들의 집합"으로 나누는 방법

| 방법 | 원리 | 장점 | 단점 | 2~3대 적합성 |
|------|------|------|------|------------|
| **Voronoi 기반 분할** | 공간을 각 로봇 기준 Voronoi 셀로 나누고, 각 로봇은 자기 셀 내 프론티어만 담당 | 이론적으로 균형 분할 | 실시간 재계산 비용 높음, 구현 복잡 | 보통 |
| **경매(Auction) 기반** | 각 로봇이 프론티어 가치를 입찰, 최고 입찰자가 담당 | 효율적 동적 할당 | 통신 오버헤드, 구현 복잡 | 보통 |
| **코스트맵 마킹** | 담당 로봇이 프론티어 방문 시 코스트맵에 "이미 탐색 중" 마킹, 다른 로봇은 회피 | 구현 단순, Nav2 친화적 | 완전한 분리 보장 불가 | **높음** |
| **거리 우선** | 각 프론티어에서 가장 가까운 로봇이 담당 | 매우 단순 | 로봇 집중 현상 가능 | **높음** |

### ARGOS 2~3대 UGV 권장 방식
**거리 기반 + 코스트맵 마킹 하이브리드**

```
1. 프론티어 목록 계산 (전역 지도 기준)
2. 각 로봇의 현재 위치에서 프론티어까지 거리 계산
3. 가장 가까운 프론티어를 해당 로봇에 할당
4. 할당된 프론티어 주변 반경 r을 코스트맵에 "탐색 중" 마킹
5. 다른 로봇은 마킹된 영역의 프론티어는 건너뜀
6. 로봇 도착 후 마킹 해제
```

구현 코드 규모: 약 200~300 라인 Python ROS 2 노드

---

## 3. 지도 병합 (Map Merge)

### 3-1. multirobot_map_merge (m-explore-ros2 내장)

| 항목 | 내용 |
|------|------|
| 패키지 위치 | m-explore-ros2 내 `map_merge` 모듈 |
| Jazzy 공식 지원 | **없음** — 소스 빌드 필요 |
| 초기 위치 없이 병합 | 지원 (`known_init_poses:=False`) |
| 제약 조건 | 로봇 간 시작 거리 **3미터 이내** 권장 (특징점 매칭 기준) |

**unknown pose 병합 원리**
```
각 로봇 독립 맵 생성
         ↓
맵 겹침 영역에서 특징점(feature) 추출
         ↓
특징점 매칭으로 상대적 위치 계산
         ↓
변환 행렬 적용하여 통합 지도 퍼블리시
```

**ARGOS 시나리오 제약**: 건물 입구에서 2~3대가 동시 출발하는 시나리오에서는 초기 위치를 미리 알고 있으므로, `known_init_poses:=True`로 설정하면 더 안정적이다.

---

### 3-2. slam_toolbox 자체 멀티로봇 지원

| 항목 | 내용 |
|------|------|
| GitHub | [SteveMacenski/slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) |
| Jazzy 버전 | **공식 지원** (v2.8.2 — 2024년 12월 13일 Jazzy 동기화) |
| 멀티로봇 방식 | "클라우드 분산 매핑" — 포즈 그래프 공유 방식 |
| 성숙도 | **고도로 실험적** (highly experimental) |

**두 가지 접근법 비교**

| 방식 | 구조 | 특징 |
|------|------|------|
| **독립 SLAM + 통합 병합** | 각 로봇 slam_toolbox 독립 실행 → map_merge로 합침 | 구현 쉬움, 통신 부하 낮음 |
| **공유 포즈 그래프** | 하나의 slam_toolbox 서버, 여러 로봇 데이터 집중 | 이론적으로 더 정확, 구현 복잡, 실험적 |

**ARGOS 권장**: 독립 SLAM + 통합 병합 방식. 실시간 통신 부하 없고 구현이 검증되어 있음.

---

## 4. Nav2 연동 방법

### 4-1. explore 노드 → Nav2 목표 설정 구조

```
[explore 노드]                [Nav2 Stack]
     │                              │
     │  1. 점유 격자 지도 구독       │
     │  /robot_1/map ──────────────►│
     │                              │
     │  2. 프론티어 계산             │
     │  (미탐색 경계 셀 추출)        │
     │                              │
     │  3. 최적 프론티어 선택         │
     │  (거리 + 정보 이득 기준)      │
     │                              │
     │  4. NavigateToPose 호출       │
     │  /robot_1/navigate_to_pose ──►│
     │                              │
     │  5. 피드백 수신               │
     │◄─ 진행률, 잔여 거리 ─────────│
     │                              │
     │  6. 도착 후 → 2번 반복        │
```

### 4-2. NavigateToPose vs NavigateThroughPoses 선택

| 액션 | 특징 | 탐색 시나리오 |
|------|------|-------------|
| **NavigateToPose** | 단일 목표 → 도착 → 다음 목표 | **프론티어 탐색에 적합** |
| **NavigateThroughPoses** | 경유지 목록 한 번에 전달 | 미리 경로가 확정된 순찰에 적합 |

**결론**: 프론티어 탐색은 실시간으로 다음 목표가 바뀌므로 `NavigateToPose`가 적합하다. 탐색하면서 새 프론티어가 발견되면 기존 목표를 취소하고 새 목표로 재설정.

### 4-3. 멀티로봇 Nav2 네임스페이스 구성

```yaml
# 각 로봇별 네임스페이스 분리
robot_1:
  - /robot_1/navigate_to_pose    # NavigateToPose 액션 서버
  - /robot_1/map                 # 개별 점유 격자
  - /robot_1/odom                # 오도메트리
  - /robot_1/scan                # LiDAR 데이터
  - /robot_1/tf                  # 좌표 변환

robot_2:
  - /robot_2/navigate_to_pose
  - /robot_2/map
  # ... 동일 구조

공유:
  - /merged_map                  # 통합 지도 (map_merge 출력)
  - /exploration_state           # 탐색 상태 공유 (커스텀)
```

### 4-4. 화점 발견 시 탐색 중단 로직

```python
# explore 노드 내 중단 로직 예시
class ExplorationNode(Node):
    def __init__(self):
        # 열화상 감지 구독
        self.fire_sub = self.create_subscription(
            FireDetection,
            '/thermal/fire_detected',
            self.fire_callback,
            10
        )

    def fire_callback(self, msg):
        if msg.confidence > 0.8:
            # 1. 현재 NavGoal 취소
            self.nav_client.cancel_goal_async(self.current_goal_handle)
            # 2. 화점 위치 퍼블리시
            self.fire_pub.publish(msg.location)
            # 3. 탐색 모드 전환
            self.exploration_active = False
            self.get_logger().info(f"화점 감지! 탐색 중단. 위치: {msg.location}")
```

---

## 5. ARGOS 시나리오 적합성 분석

### 5-1. 환경 특성

```
실내 화재 건물 특성:
  ┌──────────────────────────────────┐
  │  복도 (폭 1.5~2m)               │
  │  ┌────┐  ┌────┐  ┌────┐        │
  │  │ 방 │  │ 방 │  │ 방 │        │
  │  │    │  │    │  │    │        │
  │  └────┘  └────┘  └────┘        │
  │      ↑ 프론티어 다량 발생 예상   │
  └──────────────────────────────────┘

제약 조건:
  - 좁은 복도: 로봇 간 교행 어려움 → 영역 분할 필수
  - 문: 열린 문만 통과 가능 → Nav2 footprint 설정 중요
  - 연기: 카메라 인식 저하 → LiDAR 기반 SLAM 필수
```

### 5-2. RTX 4050 노트북 성능 제약

| 항목 | 요구 사양 | RTX 4050 추정 |
|------|----------|---------------|
| 로봇 2대 시뮬레이션 | 중간 GPU | **적합** |
| 로봇 3대 시뮬레이션 | 높은 GPU | 가능하나 프레임 저하 예상 |
| slam_toolbox 2개 동시 | CPU 집중 | i5/i7 이상 필요 |
| Gazebo Harmonic + 복잡 환경 | GPU 메모리 8GB+ | RTX 4050 6GB → **VRAM 부족 위험** |
| 열화상 처리 + 탐색 동시 | GPU 공유 | 성능 분산 필요 |

**권장 완화 전략**:
- Gazebo 환경 단순화 (메시 폴리곤 수 최소화)
- 로봇 2대로 시작, 안정화 후 3대 시도
- slam_toolbox는 CPU 스레드 수 제한 (`solver_plugin: solver_plugins::CeresSolver`, `max_laser_range: 12.0`)
- 열화상 처리는 별도 경량 모델 (YOLOv8n 수준)

---

## 6. 권장 조합 (Best Stack)

### 권장 스택 구성

```
[탐색 레이어]
m-explore-ros2 (explore 모듈)
  - 소스 빌드 (Jazzy 호환 패치 포함)
  - `explore/resume` 토픽으로 화점 감지 시 중단

[SLAM 레이어]
slam_toolbox (Jazzy 공식 지원 v2.8.4)
  - 각 로봇 독립 실행 (namespace 분리)
  - mode: mapping → 탐색 완료 후 mode: localization 전환

[지도 병합]
m-explore-ros2 (map_merge 모듈)
  - known_init_poses:=True (시작 위치 알고 있을 때)
  - 통합 지도를 explore 노드에 공급

[영역 분할]
커스텀 Python 노드 (~200줄)
  - 거리 기반 프론티어 할당
  - 코스트맵 마킹으로 중복 탐색 방지

[항법]
Nav2 (Jazzy 공식 지원)
  - 각 로봇별 namespace
  - NavigateToPose 액션 클라이언트

[화점 감지 연동]
커스텀 FireDetection → explore 중단 → 위치 보고
```

### 구현 순서 (단계별)

```
Phase 1: 단일 로봇 탐색 검증 (1~2주)
  └─ slam_toolbox + Nav2 + m-explore-ros2 단일 로봇

Phase 2: 지도 병합 검증 (1주)
  └─ 2대 로봇 독립 SLAM + map_merge 통합

Phase 3: 멀티로봇 탐색 (1~2주)
  └─ 영역 분할 노드 추가 + 2대 동시 탐색

Phase 4: 화점 연동 (1주)
  └─ 열화상 감지 신호 → 탐색 중단 → 위치 보고
```

---

## 7. 패키지별 종합 평가표

| 패키지 | GitHub | 최신 커밋 | Jazzy 호환 | 구현 난이도 | ARGOS 적합도 |
|--------|--------|----------|-----------|------------|-------------|
| m-explore-ros2 | [링크](https://github.com/robo-friends/m-explore-ros2) | 불명확 | 소스 빌드 | 3/5 | 3/5 |
| nav2_wavefront | [링크](https://github.com/SeanReg/nav2_wavefront_frontier_exploration) | 2020.11 | 불가 | 4/5 | 1/5 |
| CMU M-TARE | [링크](https://github.com/caochao39/mtare_planner) | 2024.01 | Docker 간접 | 5/5 | 2/5 |
| slam_toolbox | [링크](https://github.com/SteveMacenski/slam_toolbox) | 2024.12 | **공식 지원** | 2/5 | 5/5 |
| Nav2 | [링크](https://navigation.ros.org/) | 지속 업데이트 | **공식 지원** | 2/5 | 5/5 |
| 커스텀 분할 노드 | 직접 구현 | — | Jazzy 네이티브 | 2/5 | 4/5 |

---

## 8. 리스크 및 완화 방안

| 리스크 | 가능성 | 영향 | 완화 방안 |
|--------|--------|------|----------|
| m-explore Jazzy 빌드 실패 | 중간 | 높음 | 미리 Humble 환경에서 검증 후 포팅 |
| map_merge 정렬 실패 | 낮음 | 중간 | known_init_poses 사용, 시작 위치 명시 |
| RTX 4050 VRAM 부족 | 높음 | 중간 | 환경 단순화, 로봇 2대로 시작 |
| 좁은 복도 Nav2 플래너 실패 | 중간 | 높음 | footprint 최소화, Smac Hybrid-A* 사용 |
| 멀티 네임스페이스 TF 충돌 | 중간 | 높음 | robot_id prefix 철저히 적용 |

---

## 출처

- [robo-friends/m-explore-ros2](https://github.com/robo-friends/m-explore-ros2) — explore_lite ROS 2 포팅, explore + map_merge 통합 패키지
- [CMU M-TARE 공식 사이트](https://www.cmu-exploration.com/m-tare-planner) — 멀티로봇 계층적 탐색 플래너 설명
- [caochao39/mtare_planner](https://github.com/caochao39/mtare_planner) — M-TARE GitHub, 2024.01 최신 커밋
- [slam_toolbox 공식 문서 (Jazzy)](https://docs.ros.org/en/ros2_packages/jazzy/api/slam_toolbox/) — Jazzy 공식 지원 v2.8.4
- [Nav2 공식 문서](https://navigation.ros.org/) — NavigateToPose, NavigateThroughPoses 설명
- [NavigateToPose vs NavigateThroughPoses](https://answers.ros.org/question/409833/navigatetopose-vs-navigatethroughpose/) — 액션 비교 ROS Answers
- [SeanReg/nav2_wavefront_frontier_exploration](https://github.com/SeanReg/nav2_wavefront_frontier_exploration) — Nav2 연동 웨이브프론트 탐색 (2020, 유지보수 중단)
- [abdulkadrtr/ROS2-FrontierBaseExplorationForAutonomousRobot](https://github.com/abdulkadrtr/ROS2-FrontierBaseExplorationForAutonomousRobot) — Humble 기반 프론티어 탐색 단일로봇 구현체
- [AniArka/Autonomous-Explorer-and-Mapper-ros2-nav2](https://github.com/AniArka/Autonomous-Explorer-and-Mapper-ros2-nav2) — Nav2 + SLAM Toolbox 자율 탐색 패키지
- [Husarion 탐색 튜토리얼](https://husarion.com/tutorials/ros2-tutorials/10-exploration/) — explore_lite + Nav2 + slam_toolbox 통합 예제
- [Voronoi-GRU 멀티로봇 탐색 논문 (MDPI 2025)](https://www.mdpi.com/2076-3417/15/6/3313) — Voronoi 기반 DRL 탐색 최신 연구
- [ROS 2 멀티로봇 내비게이션 가이드](https://medium.com/@arshad.mehmood/a-guide-to-multi-robot-navigation-utilizing-turtlebot3-and-nav2-cd24f96d19c6) — TurtleBot3 + Nav2 멀티로봇 실전 예제
