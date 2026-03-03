# Day 16 - SLAM: 로봇이 스스로 지도를 만들다

**날짜**: 2026-02-19
**학습 시간**: ~40분
**핵심 주제**: SLAM (Simultaneous Localization And Mapping), slam_toolbox, Lifecycle Node 활성화, 지도 저장

---

## 🎯 오늘의 목표

Day 15에서 사전 제작된 지도(pre-built map)로 Nav2 자율 주행을 성공시켰다면, Day 16의 목표는 그 지도를 **로봇이 직접 만드는 것**이었습니다. SLAM 알고리즘을 사용해서 Gazebo 시뮬레이션 안에서 로봇이 환경을 탐색하며 `turtlebot3_world.pgm` + `turtlebot3_world.yaml`을 생성하는 것이 목표였습니다.

---

## ✅ 달성 결과

- slam_toolbox 설치 확인 및 launch 파일 구성
- Lifecycle Node configure → activate 전환 성공
- `/map` 토픽 발행 확인 (Publisher count: 1)
- `map → odom TF` 제공 확인
- teleop_twist_keyboard로 turtlebot3_world 한 바퀴 탐색
- `map_saver_cli`로 지도 파일 저장 성공: 112×103 픽셀, 0.05m/픽셀

---

## 📐 SLAM이란 무엇인가: 닭과 달걀 문제

SLAM이 왜 어려운지 이해하려면 먼저 자율 로봇이 갖고 있는 근본적인 딜레마를 이해해야 합니다. 로봇이 "나는 지금 어디에 있는가?"를 알려면 지도가 필요하고, "이 환경의 지도를 그리려면?"이라는 질문에 답하려면 현재 위치를 알아야 합니다. 이것이 바로 닭과 달걀 문제(chicken-and-egg problem)입니다.

고전적인 로봇공학에서는 이 두 문제를 분리해서 풀었습니다. 먼저 사람이 직접 환경을 측량해서 지도를 만들고, 그 지도를 로봇에 제공한 다음 AMCL 같은 위치추정 알고리즘으로 "나는 이 지도 위 어디에 있는가?"를 계산하는 방식입니다. Day 15에서 우리가 했던 것이 바로 이 패턴이었습니다.

SLAM은 이 두 문제를 **동시에** 풀겠다는 아이디어입니다. 수학적으로는 "불완전한 정보로 자기 위치와 환경 지도를 동시에 추정"하는 문제인데, 이것이 가능한 이유는 확률론적 추정(probabilistic estimation)을 사용하기 때문입니다. 완벽한 지도가 없어도 "아마 여기쯤이겠지"로 시작해서, 새로운 센서 데이터가 들어올 때마다 그 추정을 점진적으로 정확하게 갱신해 나갑니다.

---

## 🏗️ slam_toolbox의 아키텍처: 포즈 그래프 기반 SLAM

slam_toolbox는 **Graph-Based SLAM**을 구현합니다. 필터 기반 SLAM(Kalman Filter, Particle Filter)과 비교했을 때 그래프 기반이 갖는 근본적인 장점이 있는데, 바로 과거의 추정을 수정할 수 있다는 것입니다.

필터 기반 SLAM은 스트리밍 방식으로 작동합니다. 새 데이터가 들어오면 현재 상태 추정을 업데이트하고 이전 데이터는 버립니다. 이것은 메모리 효율적이지만, 과거에 내린 잘못된 판단을 수정할 수 없다는 치명적 약점이 있습니다.

그래프 기반 SLAM은 다르게 작동합니다. 로봇이 이동할 때마다 "노드(Node)"를 생성하고, 인접한 노드들 사이에 "엣지(Edge)"로 공간 관계를 기록합니다. 이 전체 그래프가 메모리에 유지됩니다. 로봇이 이전에 방문했던 장소로 돌아왔을 때(Loop Closure 감지), "아, 지금 센서 데이터가 50번 노드와 비슷하다"는 것을 알 수 있고, 이때 1번 노드부터 현재 노드까지 전체 그래프를 재최적화합니다. 이것이 **Loop Closure(루프 폐쇄)**이고, SLAM의 핵심입니다.

```
이동 경로:  A → B → C → D → E → F
                              ↑
Loop Closure: F에서 A를 다시 만남
→ 전체 경로 A~F를 일관되게 재최적화
```

slam_params.yaml에서 `do_loop_closing: true`, `loop_match_minimum_chain_size: 10`으로 설정한 것이 바로 이 기능을 활성화한 것입니다.

---

## 🔄 Lifecycle Node: 왜 slam_toolbox는 활성화가 필요한가

오늘 가장 중요한 디버깅 포인트는 slam_toolbox가 실행 중임에도 `/map`을 발행하지 않았던 것이었습니다. 원인은 slam_toolbox가 **Lifecycle Node**로 구현되어 있어서 `unconfigured [1]` 상태였기 때문입니다.

ROS 2의 Lifecycle Node는 일반 노드와 다르게 **관리된 상태 전환(managed state transitions)**을 요구합니다. 노드가 실행(spin)되더라도 즉시 동작하지 않고, 외부에서 명시적으로 상태 전환을 해줘야 합니다.

```
unconfigured [1]
    ↓ ros2 lifecycle set /slam_toolbox configure
inactive [2]       ← 파라미터 로드, 자료구조 초기화 완료
    ↓ ros2 lifecycle set /slam_toolbox activate
active [3]         ← /scan 구독 시작, /map 발행 시작
```

이 설계가 SLAM에 특히 중요한 이유는, Ceres Solver의 포즈 그래프 자료구조가 초기화되기 전에 센서 데이터를 받으면 안 되기 때문입니다. Lifecycle을 통해 "준비 완료" 신호를 명시적으로 보내야 구독이 시작되는 구조가 이 요구사항을 구조적으로 보장합니다.

### 실행 순서

```bash
# 1. slam_toolbox 실행 (unconfigured 상태로 시작)
ros2 launch my_robot_bringup slam_sim.launch.py

# 2. configure (파라미터 로드, 자료구조 초기화)
ros2 lifecycle set /slam_toolbox configure

# 3. activate (센서 구독 시작, 지도 발행 시작)
ros2 lifecycle set /slam_toolbox activate
```

---

## 🔧 TwistStamped vs Twist: ROS 2 Jazzy의 변화

오늘 두 번째 디버깅 포인트는 teleop_twist_keyboard로 `i`를 눌러도 로봇이 움직이지 않는 문제였습니다.

**원인**: `turtlebot3_burger_bridge.yaml`에서 `/cmd_vel` 브릿지 타입이 `geometry_msgs/msg/TwistStamped`로 설정되어 있는데, teleop_twist_keyboard의 기본값이 `geometry_msgs/msg/Twist`였습니다.

ROS 2 Jazzy와 Gazebo Harmonic에서는 `/cmd_vel`이 `TwistStamped` 타입으로 전환되었습니다. `Stamped` 버전은 헤더(타임스탬프 + frame_id)를 포함하는데, 이것이 `use_sim_time: true` 환경에서 시뮬레이션 시간 동기화에 중요하기 때문입니다.

**해결책**: teleop 실행 시 `stamped:=true` 파라미터 추가

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/cmd_vel -p stamped:=true
```

---

## 💾 지도 저장: 두 파일의 역할

지도를 저장하면 두 파일이 생성됩니다.

```bash
ros2 run nav2_map_server map_saver_cli \
  -f /path/to/maps/turtlebot3_world \
  --ros-args -p use_sim_time:=true
```

결과:
- `turtlebot3_world.pgm` (11,551 bytes) - 112×103 픽셀의 흑백 이미지
- `turtlebot3_world.yaml` (138 bytes) - 지도 메타데이터

`.pgm` 파일은 각 픽셀이 점유 상태를 나타냅니다. 흰색(255)은 빈 공간(free), 검은색(0)은 장애물(occupied), 회색(중간값)은 미탐색 영역(unknown)입니다. 그런데 이미지 파일 자체에는 "픽셀 하나가 현실의 몇 미터인가"라는 정보가 없습니다.

`.yaml` 파일이 이 정보를 제공합니다:
```yaml
resolution: 0.050          # 1픽셀 = 5cm
origin: [-0.928, -2.063, 0]  # 이미지 좌하단 = 실제 좌표 (-0.928m, -2.063m)
occupied_thresh: 0.65      # 65% 이상 확률로 점유 = 장애물
free_thresh: 0.196         # 19.6% 이하 = 빈 공간
```

이 두 파일을 함께 제공해야 Nav2의 `map_server`가 지도를 올바르게 로드할 수 있습니다. Day 15에서 `nav2_params_sim.yaml`에 이 파일들의 경로를 지정했던 것이 바로 이 이유입니다.

---

## 🗂️ 생성된 파일 목록

```
ros2_ws/src/my_robot_bringup/
├── config/
│   └── slam_params.yaml           ← slam_toolbox 파라미터
├── launch/
│   └── slam_sim.launch.py         ← SLAM + RViz2 실행
└── maps/
    ├── turtlebot3_world.pgm       ← SLAM으로 직접 생성한 지도 이미지
    └── turtlebot3_world.yaml      ← 지도 메타데이터

Desktop/
├── launch_slam.sh                 ← SLAM 스택 실행
├── activate_slam.sh               ← Lifecycle configure + activate
├── check_slam_status.sh           ← /map, TF 상태 확인
└── check_slam_proc.sh             ← 프로세스 확인
```

---

## 🎓 아키텍처 인사이트: SLAM vs Localization의 설계 트레이드오프

SLAM과 Localization(AMCL)은 서로 다른 가정을 기반으로 합니다. 이 차이가 실무에서 어떤 의사결정으로 이어지는지 이해하는 것이 중요합니다.

**AMCL(Day 15 방식)**은 "환경이 고정되어 있고 지도가 정확하다"고 가정합니다. 사무실이나 창고처럼 레이아웃이 변하지 않는 환경에서 매우 효율적입니다. 지도가 이미 있으므로 시작과 동시에 정확한 위치 추정이 가능하고, 계산 비용도 낮습니다.

**SLAM(Day 16 방식)**은 "환경을 모른다, 또는 환경이 변할 수 있다"고 가정합니다. 처음 방문하는 환경, 또는 가구 배치가 자주 바뀌는 공간에서 필요합니다. 포즈 그래프 유지와 최적화에 더 많은 연산이 필요합니다.

실무에서는 보통 이 두 가지를 단계적으로 사용합니다. 새 환경에 배포할 때는 SLAM으로 지도를 생성하고, 이후 운영 단계에서는 그 지도를 사용하는 AMCL로 전환합니다. 이것이 Day 15와 Day 16이 연결되는 실무적 이유입니다.

---

## 📊 학습 진행도

- Week 1: ROS 2 기초 (Node, Topic, Service, Action)
- Week 2: Gazebo 시뮬레이션, TF, URDF
- Week 3 Day 15: Nav2 완전 통합 (지도 + 위치추정 + 경로계획)
- **Week 3 Day 16: SLAM으로 지도 직접 생성 ✅**

---

*Day 16 완료 - 로봇이 스스로 지도를 만들었습니다.*
