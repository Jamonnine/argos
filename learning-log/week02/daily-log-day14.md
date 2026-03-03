# Day 14: Gazebo 시뮬레이션 — 물리 세계와 ROS 2의 연결

**날짜**: 2026-02-18
**학습 주제**: Gazebo Harmonic + TurtleBot3 + LiDAR 시각화
**소요 시간**: 약 2시간

---

## 📋 학습 목표 및 성과

### 목표
- [x] Gazebo 시뮬레이터 개념 이해 (왜 시뮬레이터가 필요한가?)
- [x] TurtleBot3 Burger를 turtlebot3_world에서 실행
- [x] LiDAR 센서 데이터를 RViz2로 시각화
- [x] cmd_vel로 로봇 이동 및 LiDAR 패턴 변화 확인

### 성과
- ✅ Gazebo Harmonic (gz sim) + ros_gz_bridge로 ROS 2 연결
- ✅ `/scan` 토픽으로 LaserScan 데이터 4Hz 수신 확인
- ✅ RViz2에서 빨간 점군으로 360° LiDAR 스캔 시각화
- ✅ cmd_vel 명령으로 로봇 이동, 이동 후 LiDAR 패턴 변화 확인

---

## 🎯 핵심 개념

### 1. 왜 시뮬레이터인가? — 실제 로봇 없이 개발하는 이유

로봇 개발에서 시뮬레이터는 단순한 "연습 도구"가 아닙니다. 실제 로봇 개발 현장에서 시뮬레이터를 쓰는 이유는 다음과 같습니다.

첫째, 비용과 안전의 문제입니다. TurtleBot3 한 대 가격이 약 50만원, 산업용 로봇팔은 수천만원입니다. 코드 한 줄 실수로 벽에 충돌하거나 테이블에서 떨어지면 수리비가 발생합니다. 시뮬레이터에서는 아무리 충돌해도 무료입니다. 더 중요한 것은, 실제 로봇이 사람 근처에서 예상치 못한 움직임을 보이면 부상 위험이 있습니다. 검증되지 않은 코드는 반드시 시뮬레이터에서 먼저 테스트하는 것이 업계 표준입니다.

둘째, 반복성과 재현성입니다. 실제 환경에서 실험하면 매번 조건이 달라집니다. 바닥의 마찰, 조명, 배터리 상태 등 변수가 너무 많습니다. 시뮬레이터는 완전히 동일한 조건에서 수천 번 반복 실험이 가능합니다. 강화학습(Reinforcement Learning) 기반 로봇 학습은 시뮬레이터 없이는 불가능합니다.

셋째, 센서 조합의 자유입니다. 실제 로봇에 LiDAR, 카메라, IMU를 동시에 장착하려면 추가 비용과 하드웨어 통합 작업이 필요합니다. 시뮬레이터에서는 URDF 파일에 한 줄만 추가하면 어떤 센서든 붙일 수 있습니다.

---

### 2. Gazebo Architecture — ROS 2와 어떻게 연결되는가?

Gazebo는 독립적인 물리 시뮬레이터입니다. ROS 2와 직접 통신하지 않습니다. 이 둘을 연결하는 것이 `ros_gz_bridge`라는 브리지 노드입니다.

```
Gazebo (gz sim)              ROS 2 (DDS)
   |                            |
   | gz::msgs::LaserScan        |
   |         →                  |
   |    ros_gz_bridge           |
   |         →                  |
   |              sensor_msgs/LaserScan
   |                            |
   |                     /scan topic
```

이 구조가 중요한 이유는 **관심사의 분리(Separation of Concerns)** 때문입니다. Gazebo는 물리 엔진(충돌, 중력, 마찰)에만 집중하고, ROS 2는 소프트웨어 통신에만 집중합니다. 브리지가 이 둘을 연결하므로, 나중에 Gazebo를 Isaac Sim이나 Webots로 교체해도 ROS 2 코드를 수정할 필요가 없습니다.

---

### 3. LiDAR 데이터의 좌표계 — 로봇 중심 세계관

LiDAR 센서가 발행하는 `/scan` 토픽의 데이터는 항상 "센서 중심 좌표계"로 표현됩니다. 즉, 모든 거리값은 "LiDAR 센서로부터 얼마나 떨어져 있는가"입니다.

로봇이 이동하면 무슨 일이 일어날까요? 외부 세계의 벽과 장애물은 실제로 이동하지 않았지만, 로봇의 관점에서는 그것들이 다른 각도와 거리에서 보입니다. 이것이 RViz2에서 확인한 현상입니다. 로봇이 앞으로 이동하고 왼쪽으로 회전하자, 오른쪽 벽은 더 가까워졌고(점들이 더 조밀하게 오른쪽 상단에 집중), 왼쪽 장애물은 다른 각도에서 보이게 되었습니다.

네비게이션 시스템(Nav2)이 해결해야 하는 핵심 문제가 바로 이것입니다. "로봇이 계속 이동해도 지도(map)에서의 절대 위치는 고정되어야 한다." 이것이 SLAM(Simultaneous Localization and Mapping)의 핵심입니다. 로봇의 odometry(바퀴 회전 기반 위치 추정)와 LiDAR 스캔을 융합하여 "내가 지도상의 어디에 있는지"를 실시간으로 추정합니다.

---

### 4. turtlebot3_world 환경 구조

turtlebot3_world는 TurtleBot3 공식 테스트 환경으로, 다음 특징을 가집니다:
- 사각형 외벽 (LiDAR로 보이는 긴 직선 패턴)
- 육각형 기둥 장애물 5개 (LiDAR로 보이는 점군 클러스터들)
- Nav2 알고리즘 검증에 최적화된 간단하지만 완전한 환경

이 환경에서 SLAM을 돌리면 약 30초 만에 전체 지도가 완성되며, 이 지도를 기반으로 "A 지점에서 B 지점으로 장애물 회피 경로"를 계획할 수 있습니다.

---

## 🏗️ 시스템 구성

### 실행된 컴포넌트들

```
Gazebo Harmonic
  └─ TurtleBot3 Burger 모델
       ├─ LiDAR 센서 (360°, 3.5m 범위)
       ├─ Differential Drive 플러그인 (바퀴 제어)
       └─ IMU 센서

ros_gz_bridge (parameter_bridge)
  ├─ /scan: gz→ros (LaserScan)
  ├─ /cmd_vel: ros→gz (TwistStamped)
  ├─ /odom: gz→ros (Odometry)
  └─ /tf: gz→ros (TF frames)

RViz2
  └─ /scan LaserScan 디스플레이 (빨간 점군)
```

### LiDAR 데이터 특성 (확인된 값)
- **토픽**: `/scan` (sensor_msgs/LaserScan)
- **발행 주기**: 4Hz (0.25초마다 업데이트)
- **각도 범위**: 0 ~ 360° (2π 라디안)
- **측정 범위**: 0.12m ~ 3.5m
- **해상도**: 360개 레이 (1° 간격)

---

## ✅ 동작 확인

### LiDAR 데이터 수신 확인
```
[ros2 topic echo /scan 결과 요약]
header:
  frame_id: base_scan
angle_min: 0.0
angle_max: 6.28 (2π)
angle_increment: 0.0174 (1°)
ranges: [1.23, 1.22, 1.21, ..., 0.89, 0.88]  # 360개 값
```

### 로봇 이동 확인
```
# 전진: linear.x = 0.2 m/s, 2초
ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.2}}}" --rate 10

# 좌회전: angular.z = 0.5 rad/s, 2초
ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped \
  "{twist: {angular: {z: 0.5}}}" --rate 10
```

**이동 결과**: LiDAR 점군 패턴이 변경됨 (오른쪽 벽 접근, 장애물 각도 변화)

---

## 🚀 다음 단계

### Day 15: SLAM + Nav2 기초

- `slam_toolbox`로 실시간 지도 생성
- `nav2_bringup`으로 자율 주행 스택 실행
- Rviz2에서 2D Nav Goal로 목적지 클릭 → 자동 경로 계획 및 이동
- 이것이 실제 자율주행 로봇의 기본 스택입니다

---

## 🎉 Day 14 완료!

오늘의 핵심 메시지:

"Gazebo는 물리법칙을 계산하는 엔진이고, ROS 2는 소프트웨어 버스입니다. 이 둘은 서로 다른 언어를 사용하기 때문에 ros_gz_bridge라는 번역가가 필요합니다. LiDAR가 '내 주변 1미터에 장애물이 있다'고 말하면, TF2가 그 데이터를 '세계 좌표계의 (3.2, 1.5, 0) 위치에 장애물이 있다'로 번역합니다. 이 번역 과정이 없으면 로봇은 자기가 어디 있는지도 모르고, 목적지로 가는 길도 계획할 수 없습니다."
