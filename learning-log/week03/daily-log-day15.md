# Day 15: Nav2 자율 주행 완성 - use_sim_time 디버깅 여정

**날짜**: 2026-02-18
**소요 시간**: 약 6시간
**핵심 성과**: TurtleBot3 + Gazebo + Nav2 완전 통합, "Nav2 Goal" 자율 주행 성공

---

## 오늘의 목표
- Gazebo 시뮬레이션 환경에서 Nav2 완전 활성화
- RViz2 "Nav2 Goal" 버튼으로 로봇 자율 주행 테스트
- `use_sim_time` 관련 버그 완전 해결

---

## 최종 시스템 구성

```
Gazebo (turtlebot3_world)
  ├── gz sim server (물리 시뮬레이션)
  ├── gz sim client (3D 렌더링)
  ├── ros_gz_bridge (Gazebo ↔ ROS 2)
  │   ├── /clock (sim time)
  │   ├── /odom (odometry)
  │   ├── /tf (odom → base_footprint)
  │   ├── /scan (LiDAR)
  │   └── /cmd_vel (velocity commands)
  └── robot_state_publisher (URDF → TF)
      └── /tf_static (base_footprint → base_link → base_scan)

Nav2 Stack (nav2_sim.launch.py)
  ├── map_server (static map 제공)
  ├── amcl (particle filter localization)
  │   └── map → odom TF 제공
  ├── controller_server (DWB local planner)
  ├── planner_server (NavFn global planner)
  ├── bt_navigator (behavior tree 실행)
  ├── behavior_server (recovery behaviors)
  ├── smoother_server (velocity smoothing)
  ├── velocity_smoother (cmd_vel smoothing)
  ├── collision_monitor (안전 장치)
  ├── waypoint_follower (waypoint 추적)
  ├── lifecycle_manager_localization
  ├── lifecycle_manager_navigation
  └── rviz2 (시각화)
```

---

## 해결한 버그들 (아키텍처 교훈과 함께)

### Bug 1: `use_sim_time: false` - 분산 시스템의 시간 동기화

**증상**: Nav2 실행 후 "Data time: 1771408602s" (벽시계) vs sim time 4514s 불일치
**결과**: 로봇이 즉시 "Reached the goal!" 없이 이동

**근본 원인**: 이전 실패한 launch 시도들이 좀비 프로세스로 남아서 `use_sim_time: false`로 실행되고 있었음

**설계 교훈**: 분산 시스템에서 "시간"은 가장 기본적인 공유 자원이다. ROS 2의 `use_sim_time`은 모든 노드가 동일한 시계를 보도록 하는 메커니즘이다. 하나라도 다른 시계를 쓰면 TF lookup 실패, 타임아웃 오류, 이상한 동작이 발생한다.

**해결**: `nav2_sim.launch.py`에서 `use_sim_time: True` 명시적으로 설정, 좀비 프로세스 정리

---

### Bug 2: `map.yaml` 상대 경로 - ROS 2 파라미터 우선순위

**증상**: `lifecycle_manager_localization: Failed` - `bad file: map.yaml`

**근본 원인**: `burger.yaml`에 `map_server.ros__parameters.yaml_filename: "map.yaml"` (노드 수준 파라미터)가 하드코딩되어 있었고, launch 파일에서 절대 경로를 글로벌 파라미터로 넘겨도 **노드 수준 > 글로벌** 우선순위에 의해 상대 경로가 적용됨

**ROS 2 파라미터 우선순위 (높음 → 낮음)**:
1. 명령줄 `--ros-args -p`
2. 노드별 파라미터 파일 (`node_name.ros__parameters`)
3. 글로벌 파라미터 파일 (`/**.ros__parameters`)
4. 기본값 (코드 내)

**해결**: `nav2_params_sim.yaml` 커스텀 파일 생성, `map_server.ros__parameters.yaml_filename`에 절대 경로 직접 기입

```yaml
map_server:
  ros__parameters:
    yaml_filename: "/opt/ros/jazzy/share/turtlebot3_navigation2/map/map.yaml"
```

---

### Bug 3: `set_initial_pose: false` - AMCL의 의도적 설계

**증상**: `global_costmap: transform from base_link to map did not become available` → planner activation timeout

**근본 원인**: AMCL은 기본적으로 초기 위치를 모르는 상태에서 시작한다. `set_initial_pose: false`이면 운영자가 RViz2에서 "2D Pose Estimate"를 눌러줄 때까지 `map → odom` TF를 브로드캐스트하지 않는다. TF가 없으면 global_costmap이 초기화 실패.

**왜 이렇게 설계되었나**: 실제 로봇은 전원 켤 때 자신의 위치를 모른다. 잘못된 위치를 가정하면 오히려 localization 수렴이 어려워진다. 운영자가 명시적으로 초기 위치를 알려주는 것이 안전하다.

**시뮬레이션에서의 해결**: spawn 위치가 항상 고정이므로 자동화 가능
```yaml
amcl:
  ros__parameters:
    set_initial_pose: true
    initial_pose:
      x: -2.0
      y: -0.5
      z: 0.0
      yaw: 0.0
```

---

### Bug 4: ros_gz_bridge 중복 실행 - Launch 파일 의존성 이해

**증상**: Gazebo 시작 후 데이터 없음 (clock, odom, scan 모두 빈 값)

**근본 원인**: `turtlebot3_world.launch.py`는 내부적으로 `ros_gz_bridge`를 포함한다. 별도로 `restart_bridge.sh`를 먼저 실행하면 같은 DDS participant 이름이 두 개 생겨 충돌.

**교훈**: Launch 파일을 사용할 때는 그 파일이 무엇을 실행하는지 먼저 확인해야 한다.
```bash
# launch 파일 내용 확인하는 명령
ros2 launch --print-description turtlebot3_gazebo turtlebot3_world.launch.py
```

---

## 생성된 파일들

### `ros2_ws/src/my_robot_bringup/launch/nav2_sim.launch.py`
Nav2 시뮬레이션 전용 launch 파일. `use_composition:=False`, `use_sim_time:=True`, `nav2_params_sim.yaml` 사용.

### `ros2_ws/src/my_robot_bringup/config/nav2_params_sim.yaml`
`burger.yaml` 기반 커스텀 파라미터 파일. 두 가지 핵심 수정:
1. `yaml_filename` 절대 경로
2. `set_initial_pose: true` + 초기 위치 (-2.0, -0.5)

### Desktop 유틸리티 스크립트
- `start_gazebo.sh`: Gazebo 시작 (turtlebot3_world)
- `launch_nav2_clean.sh`: Nav2 + RViz2 시작
- `kill_all_ros.sh`: 모든 ROS/Gazebo 프로세스 정리
- `check_nav2_lifecycle.sh`: Nav2 상태 확인
- `send_nav_goal.sh`: CLI로 navigation goal 전송 (GUI 없이 테스트)

---

## Navigation 성공 확인

```
Goal accepted with ID: aa88b1f1224847fcb8ca2c75d6ae457c

Feedback:
  current_pose: x: 1.014, y: 0.128
  distance_remaining: 0.0

Result:
  error_code: 0
  error_msg: ''

Goal finished with status: SUCCEEDED ✅
```

RViz2 "Nav2 Goal" 버튼으로도 정상 작동 확인 ✅

---

## Nav2 아키텍처 복습: 요청 → 이동 전체 흐름

```
사용자 (RViz2 Nav2 Goal 클릭)
    ↓ /navigate_to_pose Action Goal
bt_navigator (Behavior Tree 실행)
    ↓ ComputePathToPose 요청
planner_server (NavFn A* 전역 경로 계획)
    ↓ global path 반환
bt_navigator
    ↓ FollowPath 요청
controller_server (DWB 지역 경로 추적)
    ↓ /cmd_vel_nav
velocity_smoother
    ↓ /cmd_vel_smoothed
collision_monitor (안전 장치)
    ↓ /cmd_vel
ros_gz_bridge
    ↓ Gazebo TwistStamped
TurtleBot3 실제 이동 ✅
```

이 체인에서 하나라도 끊기면 로봇이 멈춘다.
오늘 디버깅한 것들이 바로 이 체인의 각 연결 고리들이었다.

---

## 핵심 명령어

```bash
# Nav2 lifecycle 상태 확인
ros2 lifecycle list

# navigation goal 전송 (CLI)
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{ pose: { header: { frame_id: 'map' }, pose: { position: { x: 1.0, y: 0.0, z: 0.0 }, orientation: { w: 1.0 } } } }" \
  --feedback

# TF 체인 확인
ros2 run tf2_tools view_frames

# launch 파일 내용 확인
ros2 launch --print-description <package> <launch_file>
```

---

## 다음 단계 (Day 16+)
- Navigation 중 장애물 회피 테스트
- 여러 waypoint 연속 이동 (WaypointFollower)
- SLAM으로 맵 직접 생성하기
