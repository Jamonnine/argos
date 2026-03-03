# 리서치 보고서: ROS 2 Jazzy 환경 4WD UGV URDF/xacro 설계
- 조사일: 2026년 3월 4일 (수)
- 키워드: ROS2 Jazzy, 4WD UGV, URDF, xacro, Gazebo Harmonic, ros2_control, diff_drive_controller, sensor plugin, HR-Sherpa

---

## 핵심 요약

ROS 2 Jazzy(2024년 LTS)와 호환되는 시뮬레이터는 **Gazebo Harmonic**이며, 두 가지를 연결하는 브리지 패키지는 `ros_gz`이다. 4WD UGV의 URDF 설계에 바로 참고할 수 있는 오픈소스 프로젝트로는 Linorobot2(Apache 2.0), Husarion Panther(Apache 2.0), MOGI-ROS 강의 시리즈(MIT)가 핵심 3대 레퍼런스로 꼽힌다. xacro 모듈식 설계는 Platform / Sensor / Control 3계층으로 분리하는 것이 실무 표준이며, ros2_control의 `diff_drive_controller`는 바퀴 쌍 수를 자유롭게 지정할 수 있어 4WD 스킵 스티어 구성도 단일 컨트롤러로 구현 가능하다. 현대 HR-Sherpa의 6륜 인휠모터 구조는 표준 URDF로 완전 재현이 가능하며, 내열 구조는 시뮬레이션에서 시각적 mesh와 collision geometry를 분리하여 표현하면 된다.

---

## 1. 오픈소스 UGV URDF 템플릿 / 참고 프로젝트

### 1-1. Linorobot2 — 가장 완성도 높은 4WD 교과서
- **GitHub**: https://github.com/linorobot/linorobot2
- **라이선스**: Apache 2.0
- **ROS 2 지원**: Humble, Iron, Jazzy 모두 지원
- **4WD 특징**: `linorobot2_description` 패키지 안에 `4wd.properties.urdf.xacro` 파일이 있으며, 바퀴 위치(wheel_pos_x, wheel_pos_y)와 크기를 파라미터 하나로 전체 URDF에 전파하는 구조이다. 바퀴 번호 배치가 아래와 같이 명확하게 정의되어 있다.

```
WHEEL1  WHEEL2   ← 전면
WHEEL3  WHEEL4   ← 후면
```

- **지원 센서**: LiDAR(RPLIDAR A1/A2/A3/S1/S2/S3, LD06, LD19, STL27L, YDLIDAR, XV11), 깊이 카메라(Intel RealSense D435/D435i, ZED, OAK-D)
- **패키지 구조**:
  - `linorobot2_description/` — URDF/xacro 파일
  - `linorobot2_bringup/` — 실기 로봇 실행 런치파일
  - `linorobot2_navigation/` — SLAM + Nav2 설정
  - `linorobot2_gazebo/` — 시뮬레이션 환경
- **ARGOS 적합도**: 최상. 2WD→4WD→메카넘 전환이 설정 파일 변경만으로 가능하여, ARGOS가 초기 4WD 플랫폼으로 시작해 나중에 구동계를 변경하더라도 코드 재사용성이 높다.

### 1-2. Husarion Panther / Lynx UGV — 실제 제품급 구조의 레퍼런스
- **GitHub**: https://github.com/husarion/husarion_ugv_ros
- **라이선스**: Apache 2.0
- **특징**: 실제 판매 중인 산업용 4WD UGV(야외 험지 대응, IP54/66)의 공식 ROS 2 패키지이다. xacro 파일로 URDF를 YAML 설정 파일 하나로 완전 제어할 수 있으며, Gazebo 시뮬레이션과 실 하드웨어 양쪽에서 동일한 URDF를 사용한다. `panther.urdf.xacro`(최상위 조합 파일)와 `panther_macro.urdf.xacro`(컴포넌트 매크로)가 분리되어 있어 모듈식 설계의 모범 사례가 된다.
- **ros2_control 통합**: `DiffDriveController`를 핵심으로 사용하며, 4개의 BLDC 인휠모터를 좌우 쌍으로 묶어 속도 명령을 정확한 바퀴 회전으로 변환한다.
- **공식 문서**: https://husarion.com/manuals/panther/overview/
- **ARGOS 적합도**: 매우 높음. 소방 현장 투입 로봇의 산업급 설계 패턴을 그대로 참고할 수 있다.

### 1-3. MOGI-ROS 강의 시리즈 — 학습용 최고 품질 튜토리얼
- **Week 3-4 (URDF + Gazebo Harmonic 기초)**: https://github.com/MOGI-ROS/Week-3-4-Gazebo-basics
- **Week 5-6 (Gazebo 센서 통합)**: https://github.com/MOGI-ROS/Week-5-6-Gazebo-sensors
- **라이선스**: MIT
- **환경**: ROS 2 Jazzy + Gazebo Harmonic (완전 호환, 2025년 기준 현행)
- **특징**: 부다페스트 공과대학교(BME MOGI) 공식 강의 자료로, Jazzy + Harmonic 조합을 다루는 가장 체계적인 오픈소스 커리큘럼이다. URDF 기초부터 Gazebo 플러그인 추가, 센서 통합까지 단계별로 설명한다.
- **ARGOS 적합도**: 학습 단계에서 최적. ARGOS가 현재 Phase 2(Gazebo 시뮬레이션)로 진입할 때 Week 3-4부터 순서대로 따라가면 된다.

### 1-4. RoverRobotics ROS2 — 실제 4WD 로버 드라이버
- **GitHub**: https://github.com/RoverRobotics/roverrobotics_ros2
- **라이선스**: Apache 2.0
- **특징**: `roverrobotics_description` 패키지에 2wd_rover와 4wd_rover URDF가 각각 존재한다. 4wd_rover는 스킵 스티어(skid steer) 구성이다. `urdf/accessories/` 폴더에 센서 통합 예시가 별도로 들어 있어, 센서를 부속품(accessory)으로 붙이는 패턴을 확인할 수 있다.

### 1-5. ROS 2 Control Demos — 컨트롤러 통합 공식 예시
- **GitHub**: https://github.com/ros-controls/ros2_control_demos
- **라이선스**: Apache 2.0
- **특징**: ros2_control 공식 팀이 제공하는 데모 모음이다. 각 예시 폴더 안에 `description/`(URDF/xacro), `bringup/`(런치파일), `hardware/`(하드웨어 인터페이스) 3개가 항상 함께 들어 있어, 완전한 통합 패턴을 볼 수 있다. DiffBot 예시가 4WD 구성에 바로 적용 가능하다.

---

## 2. xacro 모듈식 설계 패턴

### 2-1. Platform / Sensor / Control 3계층 분리 원칙

실무에서 유지보수 가능한 UGV URDF는 반드시 아래 세 계층으로 파일을 분리한다. 한 파일에 모든 것을 넣으면 센서 하나 교체할 때 전체 파일을 수정해야 하는 강결합(tight coupling) 문제가 발생한다.

```
robot.urdf.xacro             ← 최상위 조합 파일 (include 전용, 로직 없음)
├── platform/
│   ├── base.urdf.xacro      ← 섀시, 바퀴, 조인트 정의
│   └── dimensions.urdf.xacro ← 치수 파라미터 (바퀴 반지름, 간격 등)
├── sensors/
│   ├── lidar.urdf.xacro     ← LiDAR 매크로 정의
│   ├── rgb_camera.urdf.xacro
│   ├── depth_camera.urdf.xacro
│   └── imu.urdf.xacro
└── control/
    └── ros2_control.urdf.xacro ← <ros2_control> 태그 및 하드웨어 인터페이스
```

### 2-2. 센서 교체 가능한 xacro 매크로 패턴

센서를 교체 가능하게 만드는 핵심은 **매크로(macro)로 정의하고 파라미터로 위치를 주입**하는 것이다. 아래가 LiDAR를 교체 가능하게 만드는 표준 패턴이다.

```xml
<!-- sensors/lidar.urdf.xacro -->
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- 매크로 정의: 이름과 부착 위치를 파라미터로 받음 -->
  <xacro:macro name="lidar_sensor" params="parent_link xyz rpy topic_name">

    <link name="lidar_link">
      <visual>
        <geometry><cylinder radius="0.05" length="0.07"/></geometry>
      </visual>
      <collision>
        <geometry><cylinder radius="0.05" length="0.07"/></geometry>
      </collision>
      <inertial>
        <mass value="0.2"/>
        <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
      </inertial>
    </link>

    <joint name="lidar_joint" type="fixed">
      <parent link="${parent_link}"/>
      <child link="lidar_link"/>
      <origin xyz="${xyz}" rpy="${rpy}"/>
    </joint>

    <!-- Gazebo Harmonic 플러그인 (ros_gz 방식) -->
    <gazebo reference="lidar_link">
      <sensor name="gpu_lidar" type="gpu_lidar">
        <topic>${topic_name}</topic>
        <update_rate>10</update_rate>
        <lidar>
          <scan>
            <horizontal>
              <samples>360</samples>
              <resolution>1</resolution>
              <min_angle>-3.14159</min_angle>
              <max_angle>3.14159</max_angle>
            </horizontal>
          </scan>
          <range>
            <min>0.1</min>
            <max>30.0</max>
            <resolution>0.01</resolution>
          </range>
        </lidar>
        <always_on>true</always_on>
        <visualize>true</visualize>
      </sensor>
    </gazebo>

  </xacro:macro>
</robot>
```

최상위 파일에서 이 매크로를 호출하면 된다.

```xml
<!-- robot.urdf.xacro (최상위) -->
<xacro:include filename="$(find my_robot_description)/urdf/sensors/lidar.urdf.xacro"/>

<!-- 전면 LiDAR 배치 -->
<xacro:lidar_sensor
  parent_link="base_link"
  xyz="0.2 0 0.15"
  rpy="0 0 0"
  topic_name="/scan"/>
```

나중에 LiDAR 모델을 교체하거나 열화상 카메라를 추가할 때는 이 include 줄만 바꾸면 된다. 섀시 파일을 건드릴 필요가 없다.

### 2-3. ros2_control과의 통합 패턴 (4WD 스킵 스티어)

`ros2_control.urdf.xacro` 파일 안에 `<ros2_control>` 태그를 넣는다. 4WD 스킵 스티어는 diff_drive_controller에 좌우 바퀴 쌍을 2쌍 지정하는 방식으로 구현한다.

```xml
<!-- control/ros2_control.urdf.xacro -->
<ros2_control name="GazeboSystem" type="system">
  <hardware>
    <!-- Gazebo Harmonic에서는 gz_ros2_control 플러그인 사용 -->
    <plugin>gz_ros2_control/GazeboSimSystem</plugin>
  </hardware>

  <!-- 4개 바퀴 조인트 각각을 velocity 인터페이스로 등록 -->
  <joint name="front_left_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="velocity"/>
    <state_interface name="position"/>
  </joint>
  <joint name="front_right_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="velocity"/>
    <state_interface name="position"/>
  </joint>
  <joint name="rear_left_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="velocity"/>
    <state_interface name="position"/>
  </joint>
  <joint name="rear_right_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="velocity"/>
    <state_interface name="position"/>
  </joint>
</ros2_control>
```

그리고 컨트롤러 설정 YAML에서 diff_drive_controller가 4개 바퀴를 인식하도록 좌우 쌍을 나열한다.

```yaml
# config/controllers.yaml
controller_manager:
  ros__parameters:
    update_rate: 50

diff_drive_controller:
  ros__parameters:
    left_wheel_names:  ["front_left_wheel_joint",  "rear_left_wheel_joint"]
    right_wheel_names: ["front_right_wheel_joint", "rear_right_wheel_joint"]
    wheel_separation: 0.45     # 좌우 바퀴 중심 간격 (m)
    wheel_radius: 0.10         # 바퀴 반지름 (m)
    publish_odom: true
    odom_frame_id: odom
    base_frame_id: base_link
```

이것이 스킵 스티어의 핵심 원리이다. 왼쪽 2개를 동일 속도로 묶고 오른쪽 2개를 동일 속도로 묶어, 실제로는 좌/우 두 축처럼 동작시킨다. 현실 마찰로 인한 오차는 odometry 보정으로 처리한다.

---

## 3. Gazebo Sim (Harmonic) 호환성

### 3-1. ROS 2 Jazzy와 호환되는 Gazebo 버전

ROS 2 버전별 공식 지원 Gazebo 버전 관계는 아래와 같다.

| ROS 2 배포판 | 권장 Gazebo 버전 | 상태 |
|---|---|---|
| Jazzy (2024 LTS) | **Harmonic** | 현재 사용 권장 |
| Humble (2022 LTS) | Fortress, Garden | 구버전 |
| Iron (2023) | Garden, Harmonic | 비LTS |
| Rolling | Ionic | 최신 실험판 |

ARGOS는 Jazzy + Harmonic 조합을 사용하고 있으므로 올바른 선택이다. "Ignition"이라는 이름은 구버전 Gazebo(Fortress, Edifice 등)의 브랜드 이름이었고, 현재는 "Gazebo"로 통합 브랜드화되었다.

### 3-2. URDF에 Gazebo 플러그인 추가하는 최신 방법 (ros_gz vs gazebo_ros)

이것은 초보자가 가장 많이 혼동하는 부분이다. 사용하는 Gazebo 버전에 따라 완전히 다른 패키지를 써야 한다.

- **구 Gazebo Classic (11 이하)**: `gazebo_ros_pkgs`의 `libgazebo_ros_diff_drive.so` 플러그인 사용. URDF의 `<plugin name="..." filename="libgazebo_ros_diff_drive.so">` 태그 방식.
- **신 Gazebo (Fortress/Garden/Harmonic/Ionic)**: `ros_gz` 패키지 사용. 플러그인 이름이 `gz-sim-xxx-system` 형태로 바뀌었다. 특히 Harmonic에서는 센서 타입 이름도 바뀌어서 `ray` 대신 반드시 `gpu_lidar`를 사용해야 한다.

```xml
<!-- ARGOS가 사용해야 하는 방식: Gazebo Harmonic + ros_gz -->
<!-- Gazebo 세계(world) 파일에 필요한 시스템 플러그인 -->
<plugin filename="gz-sim-sensors-system"  name="gz::sim::systems::Sensors">
  <render_engine>ogre2</render_engine>
</plugin>
<plugin filename="gz-sim-imu-system"      name="gz::sim::systems::Imu"/>
<plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive"/>
<plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher"/>
```

ros2_control을 사용하는 경우에는 별도의 Gazebo 드라이브 플러그인이 필요 없다. 대신 gz_ros2_control 패키지가 그 역할을 대신한다.

```bash
# 설치 (Jazzy + Harmonic)
sudo apt install ros-jazzy-ros-gz
sudo apt install ros-jazzy-gz-ros2-control
```

### 3-3. 센서 시뮬레이션 플러그인 정리

아래는 Gazebo Harmonic + ros_gz 조합에서 ARGOS 프로젝트에 필요한 센서별 플러그인 설정이다.

**2D LiDAR (예: RPLIDAR 계열)**

```xml
<gazebo reference="lidar_link">
  <sensor name="lidar" type="gpu_lidar">
    <!-- 주의: 구버전 'ray' 타입은 Harmonic에서 지원 안 됨, 반드시 'gpu_lidar' 사용 -->
    <topic>/scan</topic>
    <update_rate>10</update_rate>
    <lidar>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159265</min_angle>
          <max_angle>3.14159265</max_angle>
        </horizontal>
      </scan>
      <range><min>0.12</min><max>12.0</max><resolution>0.01</resolution></range>
    </lidar>
    <always_on>true</always_on>
    <visualize>true</visualize>
  </sensor>
</gazebo>
```

**RGB 카메라**

```xml
<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <topic>/camera/image_raw</topic>
    <update_rate>30</update_rate>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>  <!-- 60도 -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip><near>0.1</near><far>100</far></clip>
    </camera>
    <always_on>true</always_on>
    <visualize>true</visualize>
  </sensor>
</gazebo>
```

**깊이 카메라 (Depth Camera)**

```xml
<gazebo reference="depth_camera_link">
  <sensor name="depth_camera" type="depth_camera">
    <topic>/depth_camera</topic>
    <update_rate>15</update_rate>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
      </image>
      <clip><near>0.2</near><far>5.0</far></clip>
    </camera>
    <always_on>true</always_on>
    <visualize>true</visualize>
  </sensor>
</gazebo>
```

**IMU (관성 측정 장치)**

```xml
<gazebo reference="imu_link">
  <sensor name="imu" type="imu">
    <topic>/imu/data</topic>
    <update_rate>200</update_rate>
    <imu>
      <angular_velocity>
        <x><noise type="gaussian"><mean>0</mean><stddev>0.009</stddev></noise></x>
        <y><noise type="gaussian"><mean>0</mean><stddev>0.009</stddev></noise></y>
        <z><noise type="gaussian"><mean>0</mean><stddev>0.009</stddev></noise></z>
      </angular_velocity>
      <linear_acceleration>
        <x><noise type="gaussian"><mean>0</mean><stddev>0.021</stddev></noise></x>
        <y><noise type="gaussian"><mean>0</mean><stddev>0.021</stddev></noise></y>
        <z><noise type="gaussian"><mean>0</mean><stddev>0.021</stddev></noise></z>
      </linear_acceleration>
    </imu>
    <always_on>true</always_on>
  </sensor>
</gazebo>
```

**ros_gz 브리지 런치파일 (센서 토픽을 ROS 2로 전달)**

```python
# launch/sensor_bridge.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/depth_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        output='screen',
    )
    return LaunchDescription([bridge])
```

---

## 4. 현대 HR-Sherpa 스펙 참고 — URDF 설계 시사점

### 4-1. HR-Sherpa 핵심 스펙 (소방청 기증 공식 발표 기준, 2025년)

| 항목 | 스펙 |
|---|---|
| 구동 방식 | 6×6 인휠모터 독립구동 |
| 최고 속도 | 50 km/h |
| 등판 능력 | 종방향 60%, 횡방향 40% |
| 자기 냉각 | 외부 살수 커튼 (800°C 환경에서 본체 50~60°C 유지) |
| 탑재 센서 | AI 비전 카메라, 원격운전 시스템 |
| 임무 장비 | 고압 자가발광 호스릴, 수동/분무 전환 수포 |

출처: 현대자동차그룹 공식 뉴스룸 (2025년 발표)

### 4-2. 6륜 인휠모터 구조를 URDF로 구현하는 방법

6륜 독립구동은 URDF 상에서 바퀴 조인트 6개를 독립적으로 정의하고, 스킵 스티어 방식으로 묶는 것이다. 표준 diff_drive_controller는 바퀴 쌍의 개수 제한이 없으므로 6륜 구성(좌 3개, 우 3개)도 가능하다.

```xml
<!-- base.urdf.xacro: 6륜 조인트 정의 -->
<!-- 전면 좌/우 -->
<xacro:wheel_macro prefix="front_left"  parent="base_link" xyz=" 0.25  0.25 0" />
<xacro:wheel_macro prefix="front_right" parent="base_link" xyz=" 0.25 -0.25 0" />
<!-- 중간 좌/우 -->
<xacro:wheel_macro prefix="mid_left"    parent="base_link" xyz=" 0.00  0.25 0" />
<xacro:wheel_macro prefix="mid_right"   parent="base_link" xyz=" 0.00 -0.25 0" />
<!-- 후면 좌/우 -->
<xacro:wheel_macro prefix="rear_left"   parent="base_link" xyz="-0.25  0.25 0" />
<xacro:wheel_macro prefix="rear_right"  parent="base_link" xyz="-0.25 -0.25 0" />
```

컨트롤러 설정에서 좌 3개, 우 3개를 묶는다.

```yaml
diff_drive_controller:
  ros__parameters:
    left_wheel_names:
      - "front_left_wheel_joint"
      - "mid_left_wheel_joint"
      - "rear_left_wheel_joint"
    right_wheel_names:
      - "front_right_wheel_joint"
      - "mid_right_wheel_joint"
      - "rear_right_wheel_joint"
    wheel_separation: 0.50
    wheel_radius: 0.15
```

### 4-3. 내열 구조의 시뮬레이션 모델링 방법

시뮬레이션에서 "내열"이라는 재료 특성을 물리 엔진이 직접 계산하지는 않는다. 다만 다음 두 가지 방법으로 개념적으로 모델링한다.

**방법 A: 외관 분리 (visual / collision 분리)**

내열 패널, 살수 노즐 등의 물리적 존재를 visual mesh로만 정의하고 collision은 단순 박스로 처리한다. 실제 소방 로봇 URDF에서 흔히 쓰이는 방식이다.

```xml
<link name="heat_shield_link">
  <!-- 시각적 표현: 내열 패널 mesh -->
  <visual>
    <geometry>
      <mesh filename="package://argos_description/meshes/heat_shield.dae"/>
    </geometry>
    <material name="heat_resistant">
      <color rgba="0.8 0.3 0.1 0.9"/>  <!-- 주황색 계열로 내열 시각화 -->
    </material>
  </visual>
  <!-- 충돌 모델: 단순 박스 (연산 효율) -->
  <collision>
    <geometry><box size="0.6 0.5 0.02"/></geometry>
  </collision>
</link>
```

**방법 B: ROS 2 커스텀 메시지로 온도 상태 퍼블리시**

시뮬레이션에서 온도 데이터 자체는 커스텀 노드가 계산하여 `/robot/temperature` 토픽으로 퍼블리시한다. ARGOS 프로젝트의 `temperature_sensor.py`가 이미 이 패턴을 구현하고 있어서 그대로 확장 가능하다.

---

## ARGOS 프로젝트 권장 적용 조합

### 권장 스택 (즉시 적용 가능)

| 계층 | 선택 | 이유 |
|---|---|---|
| URDF 기반 참고 | Linorobot2 (4WD xacro 구조) | Apache 2.0, Jazzy 지원, 파라미터 설계 모범 |
| 산업급 설계 참고 | Husarion Panther (구조 패턴) | Apache 2.0, 실제 소방 현장 투입 수준 UGV |
| 학습 커리큘럼 | MOGI-ROS Week 3-4, 5-6 | MIT, Jazzy+Harmonic 완전 호환, 가장 최신 |
| 구동 컨트롤러 | diff_drive_controller (ros2_control) | 4WD 스킵 스티어 단일 컨트롤러로 처리 가능 |
| Gazebo 연동 | gz_ros2_control + ros_gz bridge | Jazzy+Harmonic 공식 방식 |

### 권장 패키지 구조 (ARGOS 적용안)

```
ros2_ws/src/
├── argos_description/        ← URDF/xacro 파일
│   ├── urdf/
│   │   ├── robot.urdf.xacro          ← 최상위 조합
│   │   ├── platform/
│   │   │   ├── base.urdf.xacro       ← 섀시 + 바퀴
│   │   │   └── dimensions.xacro      ← 치수 파라미터
│   │   ├── sensors/
│   │   │   ├── lidar.urdf.xacro
│   │   │   ├── rgb_camera.urdf.xacro
│   │   │   ├── depth_camera.urdf.xacro
│   │   │   ├── thermal_camera.urdf.xacro  ← 소방용 열화상
│   │   │   └── imu.urdf.xacro
│   │   └── control/
│   │       └── ros2_control.urdf.xacro
│   ├── meshes/               ← 3D 모델 파일 (.dae, .stl)
│   └── config/
│       └── controllers.yaml
├── argos_bringup/            ← 런치파일
│   └── launch/
│       ├── sim.launch.py
│       └── sensor_bridge.launch.py
└── argos_gazebo/             ← Gazebo 월드 파일
    └── worlds/
        └── fire_scene.world
```

### 단계별 구현 로드맵

1단계 (이번 주): MOGI-ROS Week 3-4 코드를 그대로 실행하면서 xacro 파일 구조 파악. `argos_description` 패키지 빈 껍데기 생성.

2단계 (다음 주): Linorobot2의 4wd.properties.urdf.xacro를 참고하여 ARGOS 섀시 base.urdf.xacro 작성. Rviz에서 시각화 확인.

3단계: MOGI-ROS Week 5-6 참고하여 LiDAR + IMU 센서 xacro 매크로 추가. Gazebo Harmonic에서 센서 데이터 확인.

4단계: gz_ros2_control + diff_drive_controller로 실제 속도 명령(/cmd_vel) 수신 및 바퀴 회전 확인.

5단계 (ARGOS 고유화): 열화상 카메라 매크로 추가. HR-Sherpa 참고한 내열 패널 visual 추가.

---

## 출처

- [Linorobot2 — GitHub (Apache 2.0)](https://github.com/linorobot/linorobot2) — 4WD xacro 파라미터 설계, RPLIDAR/RealSense 통합
- [Husarion UGV ROS — GitHub (Apache 2.0)](https://github.com/husarion/husarion_ugv_ros) — 산업급 4WD UGV, ros2_control DiffDriveController 통합
- [Husarion Panther 공식 문서](https://husarion.com/manuals/panther/overview/) — 하드웨어 스펙 및 ROS 2 패키지 구조
- [MOGI-ROS Week 3-4 Gazebo basics (MIT)](https://github.com/MOGI-ROS/Week-3-4-Gazebo-basics) — Jazzy + Harmonic URDF/xacro 기초
- [MOGI-ROS Week 5-6 Gazebo sensors (MIT)](https://github.com/MOGI-ROS/Week-5-6-Gazebo-sensors) — LiDAR, 카메라, IMU 플러그인
- [ROS2 Control Demos — GitHub (Apache 2.0)](https://github.com/ros-controls/ros2_control_demos) — diff_drive_controller 공식 예시
- [gz_ros2_control — ROS2 Control 공식 문서 (Jazzy)](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html) — Gazebo Harmonic 연동 공식 문서
- [ros_gz — GitHub](https://github.com/gazebosim/ros_gz) — ROS 2와 Gazebo Harmonic 브리지 패키지
- [RoverRobotics ROS2 — GitHub (Apache 2.0)](https://github.com/RoverRobotics/roverrobotics_ros2) — 4WD rover URDF, 센서 accessory 패턴
- [현대자동차그룹 뉴스룸 — HR-Sherpa 소방로봇 (2025)](https://www.hyundai.com/worldwide/en/newsroom/detail/0000001129) — 6×6 인휠모터 스펙, 내열 구조 공식 발표
- [diff_drive_controller 공식 문서 (Rolling)](https://control.ros.org/rolling/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html) — 4바퀴 스킵 스티어 설정 방법
- [Gazebo Harmonic 마이그레이션 가이드](https://gazebosim.org/docs/latest/migrating_gazebo_classic_ros2_packages/) — gazebo_ros → ros_gz 전환 공식 가이드
- [automaticaddison — Gazebo ROS 2 Jazzy 시뮬레이션 튜토리얼](https://automaticaddison.com/how-to-simulate-a-mobile-robot-in-gazebo-ros-2-jazzy/) — 실용 튜토리얼
