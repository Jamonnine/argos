# 리서치 보고서: ARGOS MS-4 열화상 시뮬레이션 + 화점 감지
- 조사일: 2026년 3월 4일 (수)
- 키워드: Gazebo Harmonic thermal camera, gz-sim8 thermal sensor, ROS2 thermal bridge, hotspot detection, fire simulation SDF
- 대상: ARGOS 프로젝트 MS-4 구현 준비

---

## 핵심 요약

Gazebo Harmonic(gz-sim 8.x)은 `thermal_camera` 센서를 **공식 지원**한다. ARGOS의 `thermal_camera.urdf.xacro`는 이미 올바른 구조로 작성되어 있으나, **온도 데이터 형식 미지정(L8/L16)** 과 **ThermalSensor 플러그인 누락**, **월드 파일 열원(heat source) 미정의**라는 세 가지 보완이 필요하다. 화점 감지는 OpenCV 임계값 + 윤곽선(contour) 기반으로 구현하며, 파티클 이미터(불꽃 파티클)는 열화상 카메라에 **감지되지 않으므로** 열원을 별도 박스 모델로 정의해야 한다.

---

## 상세 내용

### 1. Gazebo Harmonic 열화상 카메라 센서 지원

#### 1-1. 지원 현황

Gazebo Harmonic(gz-sim 8)은 `gz-sensors` 라이브러리를 통해 `thermal_camera` 센서 타입을 공식 지원한다.

- **센서 타입**: `type="thermal"` (SDF), `type="thermal_camera"` (xacro/URDF)
- **렌더 엔진**: `ogre2` 필수 (`gz-sim-sensors-system` 플러그인에서 지정)
- **이미지 포맷**: L8 (8비트) 또는 L16 (16비트, 기본값)

#### 1-2. SDF 센서 정의 — 공식 예제 (gz-sim8 브랜치 기준)

```xml
<!-- 월드 필수 플러그인 (Sensors 시스템) -->
<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
  <render_engine>ogre2</render_engine>
</plugin>

<!-- ---- 16비트 열화상 카메라 (고정밀, 기본) ---- -->
<sensor name="thermal_camera" type="thermal">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>320</width>
      <height>240</height>
      <format>L16</format>   <!-- 16비트: 0.01 K/unit, 최대 ~655.35 K -->
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
  </camera>
  <always_on>1</always_on>
  <update_rate>10</update_rate>
  <visualize>true</visualize>
  <topic>/thermal/image_raw</topic>
</sensor>

<!-- ---- 8비트 열화상 카메라 (경량, 처리 용이) ---- -->
<sensor name="thermal_camera_8bit" type="thermal">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>160</width>
      <height>120</height>
      <format>L8</format>    <!-- 8비트: 3.0 K/unit, 최대 ~765 K -->
    </image>
    <clip>
      <near>0.1</near>
      <far>50</far>
    </clip>
  </camera>
  <always_on>1</always_on>
  <update_rate>10</update_rate>
  <topic>/thermal/image_raw</topic>
  <!-- ThermalSensor 플러그인: 8비트 시 온도 범위 명시 권장 -->
  <plugin filename="gz-sim-thermal-sensor-system"
          name="gz::sim::systems::ThermalSensor">
    <min_temp>253.15</min_temp>  <!-- -20°C -->
    <max_temp>1273.15</max_temp> <!-- 1000°C, 화재 대응 -->
    <resolution>3.0</resolution> <!-- K/unit (8비트) -->
  </plugin>
</sensor>
```

#### 1-3. 온도 데이터 형식 및 변환 공식

| 포맷 | 비트 | 분해능(기본) | 최대 온도 | 용도 |
|------|------|------------|----------|------|
| L16  | 16비트 | 0.01 K/unit | ~655.35 K (~382°C) | 고정밀, 기본값 |
| L8   | 8비트  | 3.0 K/unit  | ~765 K (~492°C)   | 경량, 빠른 처리 |

**온도 → 픽셀값 변환 공식:**
```
픽셀값 = 온도(K) / resolution
온도(K) = 픽셀값 × resolution

예) L16, 분해능 0.01:
  600 K (327°C) → 픽셀값 = 600 / 0.01 = 60,000
  300 K (27°C)  → 픽셀값 = 300 / 0.01 = 30,000

예) L8, 분해능 3.0:
  600 K (327°C) → 픽셀값 = 600 / 3.0 = 200
  300 K (27°C)  → 픽셀값 = 300 / 3.0 = 100
```

**중요**: L8 카메라는 뷰 내 최고 온도 객체가 항상 흰색(255), 나머지는 비례 정규화. 즉 절대온도 비교가 아닌 상대 비교.

---

### 2. 열원(Heat Source) 정의 방법

#### 2-1. 균일 온도 객체 (단순 열원)

모델의 `<visual>` 태그 내 `gz-sim-thermal-system` 플러그인으로 온도 지정:

```xml
<model name="fire_source_room_a">
  <static>true</static>
  <pose>1.5 6.5 0.5 0 0 0</pose>
  <link name="link">
    <visual name="visual">
      <geometry>
        <box><size>0.3 0.3 1.0</size></box>
      </geometry>
      <material>
        <ambient>1 0.3 0 1</ambient>  <!-- 주황색: 시각적 표시 -->
        <diffuse>1 0.3 0 1</diffuse>
      </material>
      <!-- 열화상 카메라에 고온으로 보이게 하는 핵심 플러그인 -->
      <plugin filename="gz-sim-thermal-system"
              name="gz::sim::systems::Thermal">
        <temperature>873.15</temperature>  <!-- 600°C: 초기 화재 온도 -->
      </plugin>
    </visual>
  </link>
</model>
```

#### 2-2. 열 서명 텍스처 (heat_signature, 비균일 온도)

표면마다 온도가 다른 복잡한 열원에 사용. PNG 텍스처의 밝기(0~255)가 min~max 온도에 선형 매핑됨:

```xml
<visual name="visual">
  <plugin filename="gz-sim-thermal-system"
          name="gz::sim::systems::Thermal">
    <heat_signature>model://fire_source/textures/heat_sig.png</heat_signature>
    <min_temp>300.0</min_temp>  <!-- 27°C: 차가운 부분 -->
    <max_temp>1273.15</max_temp> <!-- 1000°C: 가장 뜨거운 부분 -->
  </plugin>
</visual>
```

#### 2-3. 파티클 이미터(불꽃 파티클)와 열화상의 관계 — 중요 제약

> **Gazebo 공식 설계 결정**: 파티클 이미터는 열화상 카메라에 **감지되지 않는다.**
>
> 이유: 현실에서 열화상 카메라는 연기·안개 등 파티클을 투과하여 실제 열원을 보기 때문에, 이를 시뮬레이션에 그대로 반영한 설계.

**결론**: 불꽃 시각 효과는 파티클 이미터로 구현하되, 열원 감지용 데이터는 별도의 투명 박스 모델(`temperature` 플러그인 적용)로 정의해야 한다.

```xml
<!-- 시각 효과: 불꽃 파티클 (RGB 카메라용) -->
<model name="fire_visual">
  <pose>2 3 0 0 0 0</pose>
  <link name="link">
    <visual name="flame">
      <plugin filename="gz-sim-particle-emitter-system"
              name="gz::sim::systems::ParticleEmitter">
        <!-- 파티클 설정 -->
      </plugin>
    </visual>
  </link>
</model>

<!-- 열원: 투명 박스 (열화상 카메라용) -->
<model name="fire_thermal_source">
  <static>true</static>
  <pose>2 3 0.5 0 0 0</pose>
  <link name="link">
    <visual name="visual">
      <geometry>
        <box><size>0.5 0.5 1.0</size></box>
      </geometry>
      <transparency>1.0</transparency>  <!-- 눈에 안 보이게 -->
      <plugin filename="gz-sim-thermal-system"
              name="gz::sim::systems::Thermal">
        <temperature>1073.15</temperature>  <!-- 800°C -->
      </plugin>
    </visual>
  </link>
</model>
```

---

### 3. ROS 2 브리지 (gz.msgs → sensor_msgs)

#### 3-1. 토픽 타입 매핑

열화상 카메라는 일반 카메라와 동일한 메시지 타입을 사용한다:

| Gazebo 타입 | ROS 2 타입 | 방향 |
|-------------|-----------|------|
| `gz.msgs.Image` | `sensor_msgs/msg/Image` | GZ → ROS |
| `gz.msgs.CameraInfo` | `sensor_msgs/msg/CameraInfo` | GZ → ROS |

L16 포맷의 경우 `sensor_msgs/msg/Image`의 `encoding` 필드가 `mono16`으로 설정됨.

#### 3-2. 브리지 설정 방법

**방법 A: 커맨드라인**
```bash
ros2 run ros_gz_bridge parameter_bridge \
  /thermal/image_raw@sensor_msgs/msg/Image@gz.msgs.Image \
  /thermal/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo
```

**방법 B: YAML 설정 파일 (권장)**
```yaml
# config/thermal_bridge.yaml
- ros_topic_name: "/thermal/image_raw"
  gz_topic_name: "/thermal/image_raw"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS
  lazy: false

- ros_topic_name: "/thermal/camera_info"
  gz_topic_name: "/thermal/camera_info"
  ros_type_name: "sensor_msgs/msg/CameraInfo"
  gz_type_name: "gz.msgs.CameraInfo"
  direction: GZ_TO_ROS
  lazy: false
```

```python
# launch 파일에서 브리지 노드 실행
bridge_node = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=['--ros-args', '-p',
               'config_file:=/path/to/thermal_bridge.yaml'],
    output='screen'
)
```

#### 3-3. frame_id 주의사항

Gazebo의 카메라 토픽 `frame_id`와 URDF의 `optical_link`를 일치시켜야 RViz2에서 올바르게 표시됨:

```bash
# frame_id 재정의 옵션 사용
ros2 run ros_gz_bridge parameter_bridge \
  /thermal/image_raw@sensor_msgs/msg/Image@gz.msgs.Image \
  --ros-args -p override_frame_id:=thermal_camera_optical_link
```

---

### 4. 화점 감지 알고리즘

#### 4-1. 임계값 기반 화점 감지 (Threshold-based Hotspot Detection)

열화상 이미지에서 고온 영역을 분리하는 가장 기본적인 방법:

```python
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ThermalHotspotDetector(Node):
    """
    열화상 이미지에서 화점(hotspot)을 감지하는 ROS 2 노드.

    알고리즘:
    1. L8/mono16 이미지 수신
    2. 임계값으로 이진화 (온도 기준)
    3. 윤곽선(contour) 검출
    4. 크기 필터링 (노이즈 제거)
    5. 화점 위치/크기 퍼블리시
    """

    def __init__(self):
        super().__init__('thermal_hotspot_detector')

        # 파라미터 선언
        self.declare_parameter('threshold_kelvin', 500.0)   # 화점 판단 기준 온도
        self.declare_parameter('resolution', 3.0)            # K/unit (L8=3.0, L16=0.01)
        self.declare_parameter('min_area_pixels', 50)        # 최소 화점 면적(픽셀)
        self.declare_parameter('image_format', 'L8')         # L8 또는 L16

        self.bridge = CvBridge()

        # 구독/퍼블리시
        self.sub = self.create_subscription(
            Image, '/thermal/image_raw',
            self.thermal_callback, 10)

        self.pub_debug = self.create_publisher(Image, '/thermal/hotspot_viz', 10)

    def thermal_callback(self, msg: Image):
        """열화상 이미지 콜백"""
        threshold_k = self.get_parameter('threshold_kelvin').value
        resolution  = self.get_parameter('resolution').value
        min_area    = self.get_parameter('min_area_pixels').value
        img_format  = self.get_parameter('image_format').value

        # ROS Image → OpenCV
        if img_format == 'L16':
            # mono16: 16비트 unsigned
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono16')
            # 픽셀값 → 온도(K)
            temp_k = cv_img.astype(np.float32) * resolution  # resolution=0.01
            # 임계값을 픽셀값으로 변환
            threshold_pixel = int(threshold_k / resolution)
            _, binary = cv2.threshold(
                cv_img, threshold_pixel, 65535, cv2.THRESH_BINARY)
            binary = (binary / 256).astype(np.uint8)  # 8비트로 변환 (contour용)
        else:
            # L8: 8비트
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            threshold_pixel = int(threshold_k / resolution)
            threshold_pixel = min(threshold_pixel, 255)
            _, binary = cv2.threshold(
                cv_img, threshold_pixel, 255, cv2.THRESH_BINARY)

        # 윤곽선(contour) 검출
        # cv2.findContours: 이진 이미지에서 외곽 경계선 추출
        # RETR_EXTERNAL: 가장 바깥 윤곽선만 (내부 구멍 무시)
        # CHAIN_APPROX_SIMPLE: 선분 끝점만 저장 (메모리 효율)
        contours, _ = cv2.findContours(
            binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        hotspots = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area:
                continue  # 노이즈 제거

            # 중심점 계산 (모멘트 이용)
            M = cv2.moments(contour)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                hotspots.append({'center': (cx, cy), 'area': area})

        # 디버그 시각화 퍼블리시
        self._publish_debug(msg, contours, hotspots)

        if hotspots:
            self.get_logger().info(
                f"화점 감지: {len(hotspots)}개, "
                f"위치: {[h['center'] for h in hotspots]}")

    def _publish_debug(self, original_msg, contours, hotspots):
        """감지 결과를 컬러 이미지로 시각화"""
        # 원본을 컬러맵으로 변환 (COLORMAP_INFERNO: 열화상 표준 컬러맵)
        cv_img = self.bridge.imgmsg_to_cv2(original_msg, desired_encoding='mono8')
        colored = cv2.applyColorMap(cv_img, cv2.COLORMAP_INFERNO)

        # 감지된 화점 윤곽선 그리기
        cv2.drawContours(colored, contours, -1, (0, 255, 0), 2)
        for h in hotspots:
            cv2.circle(colored, h['center'], 5, (0, 0, 255), -1)

        debug_msg = self.bridge.cv2_to_imgmsg(colored, encoding='bgr8')
        debug_msg.header = original_msg.header
        self.pub_debug.publish(debug_msg)
```

#### 4-2. 적응형 임계값 (Adaptive Threshold)

단순 임계값의 문제: L8 카메라는 뷰 내 최고 온도에 정규화되므로, 절대 온도 기준이 불안정함.
해결책: 적응형 임계값 또는 상위 N% 픽셀 선택:

```python
def detect_hotspot_adaptive(cv_img_8bit: np.ndarray, top_percent: float = 0.05):
    """
    상위 5% 고온 픽셀을 화점으로 판단 (정규화 문제 해결).
    L8 카메라 특성에 적합.
    """
    flat = cv_img_8bit.flatten()
    threshold = np.percentile(flat, 100 * (1 - top_percent))
    _, binary = cv2.threshold(cv_img_8bit, int(threshold), 255, cv2.THRESH_BINARY)
    return binary
```

#### 4-3. Blob Detection (SimpleBlobDetector)

OpenCV의 SimpleBlobDetector로 화점 원형 영역 검출:

```python
def setup_blob_detector():
    """화점 감지용 Blob Detector 설정"""
    params = cv2.SimpleBlobDetector_Params()

    # 밝기 기반 필터 (밝은 = 뜨거운)
    params.filterByColor = True
    params.blobColor = 255  # 흰색(고온) 블롭

    # 면적 필터
    params.filterByArea = True
    params.minArea = 50     # 최소 50 픽셀
    params.maxArea = 5000   # 최대 5000 픽셀

    # 원형도 필터 (화점은 대체로 원형)
    params.filterByCircularity = True
    params.minCircularity = 0.3

    detector = cv2.SimpleBlobDetector_create(params)
    return detector
```

---

### 5. 관련 오픈소스 프로젝트

| 프로젝트 | 설명 | 관련성 |
|---------|------|--------|
| [gazebosim/gz-sim thermal_camera.sdf](https://github.com/gazebosim/gz-sim/blob/gz-sim8/examples/worlds/thermal_camera.sdf) | 공식 예제 (gz-sim8 브랜치) | 최고 |
| [gazebosim/gz-sensors](https://github.com/gazebosim/gz-sensors) | 열화상 카메라 센서 구현 소스 | 높음 |
| [kyriakosar/Autonomous-robot-for-fire-detection](https://github.com/kyriakosar/Autonomous-robot-for-fire-detection) | YOLO + Gazebo + ROS 화재 감지 로봇 | 중간 |
| [gazebosim/ros_gz](https://github.com/gazebosim/ros_gz) | ROS-Gazebo 브리지 공식 저장소 | 높음 |
| [PyImageSearch Thermal Vision Series](https://pyimagesearch.com/2022/10/24/thermal-vision-fever-detector-with-python-and-opencv-starter-project/) | OpenCV 열화상 처리 튜토리얼 | 중간 |

---

## 비교 분석

### L8 vs L16 센서 선택

| 항목 | L8 (8비트) | L16 (16비트) |
|------|-----------|-------------|
| 픽셀 범위 | 0~255 | 0~65535 |
| 기본 분해능 | 3.0 K/unit | 0.01 K/unit |
| 최대 온도(기본) | ~765 K (492°C) | ~655 K (382°C) |
| 처리 용이성 | 쉬움 (표준 OpenCV) | 별도 변환 필요 |
| 절대온도 정밀도 | 낮음 (상대 정규화) | 높음 |
| 화재 감지 용도 | 충분 (고온 감지) | 과도할 수 있음 |
| **ARGOS 권장** | **1차 개발용 (단순)** | **정밀 측정 필요 시** |

### 화점 감지 방법 비교

| 방법 | 장점 | 단점 | 권장 상황 |
|------|------|------|----------|
| 고정 임계값 | 구현 단순, 절대온도 기준 | L8 정규화 문제 | L16 센서 사용 시 |
| 적응형 임계값 (상위 N%) | L8 정규화 문제 해결 | 항상 화점이 있다고 판단 | L8 센서 사용 시 |
| Blob Detection | 원형 화점에 최적 | 불규칙 형태 약함 | 점형 열원 감지 |
| YOLOv8 (딥러닝) | 복잡한 패턴 인식 | 학습 데이터 필요, 무거움 | 실제 배포 시 |

---

## 현재 ARGOS 코드 갭 분석

### thermal_camera.urdf.xacro 현황

```
현재 상태                     보완 필요
---                           ---
type="thermal_camera" ✓       format 미지정 (L8/L16 기본값 불명확)
update_rate=10 ✓              ThermalSensor 플러그인 없음 (온도 범위 미지정)
160x120 해상도 ✓              화재 대응 max_temp 미설정 (기본값 낮을 수 있음)
topic 지정 ✓                  브리지 설정 파일 없음
gz_frame_id ✓                 -
```

### indoor_test.sdf 현황

```
현재 상태                     보완 필요
---                           ---
벽/가구/잔해 ✓                열원(fire_source) 모델 없음
Sensors 플러그인 ✓            <atmosphere> 온도 미지정 (기본값 사용)
ogre2 렌더 엔진 ✓             -
```

---

## 권장 사항

### 단계별 구현 계획

**Phase 1: 센서 수정 (1시간)**
1. `thermal_camera.urdf.xacro` 수정:
   - `<format>L8</format>` 추가 (8비트, 화재 감지 용도 충분)
   - ThermalSensor 플러그인 추가 (min: 253K/-20°C, max: 1273K/1000°C, resolution: 3.0)
2. `indoor_test.sdf` 수정:
   - `<atmosphere>` 주변 온도 설정 (300K)
   - 방 A에 열원 모델 추가 (`gz-sim-thermal-system` 적용, 873K/600°C)

**Phase 2: 브리지 설정 (30분)**
1. `config/thermal_bridge.yaml` 작성
2. `gazebo.launch.py`에 브리지 노드 추가

**Phase 3: 화점 감지 노드 (2~3시간)**
1. `my_robot_bringup/hotspot_detector.py` 작성
   - L8 이미지 수신
   - 적응형 임계값 (상위 5~10% 픽셀)
   - contour 기반 화점 위치 추출
   - `/thermal/hotspot_viz` 디버그 토픽 퍼블리시
2. `CMakeLists.txt`, `package.xml`에 opencv 의존성 추가

**Phase 4: 통합 테스트**
1. Gazebo 실행 → 열원 모델 존재 확인 → RViz2에서 열화상 이미지 확인
2. 화점 감지 노드 실행 → `/thermal/hotspot_viz` 확인

### 핵심 주의사항

1. **파티클 이미터 ≠ 열원**: 불꽃 파티클 시각 효과와 열원 데이터는 별도 모델로 분리 필수
2. **L8 카메라의 상대 정규화**: 뷰 안의 가장 뜨거운 물체가 항상 255. 절대온도 측정은 L16 사용
3. **ogre2 렌더 엔진 필수**: `gz-sim-sensors-system` 플러그인에 `<render_engine>ogre2</render_engine>` 명시
4. **열 시스템 플러그인 위치**: `<temperature>` 는 `<visual>` 태그 안에 있어야 함 (`<link>` 직접 하위 불가)

---

## 출처

- [Gazebo Sensors: Thermal Camera in Gazebo Sim](https://gazebosim.org/api/sensors/6/thermalcameraigngazebo.html) — 공식 API 문서, L8/L16 포맷, temperature 설정
- [gz-sim/examples/worlds/thermal_camera.sdf (gz-sim8)](https://github.com/gazebosim/gz-sim/blob/gz-sim8/examples/worlds/thermal_camera.sdf) — 공식 예제 SDF, 가장 신뢰도 높음
- [gazebosim/ros_gz README (jazzy 브랜치)](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_bridge/README.md) — 브리지 타입 매핑
- [ros_gz_bridge Jazzy 공식 문서](https://docs.ros.org/en/jazzy/p/ros_gz_bridge/) — ROS 2 Jazzy 기준 브리지 설정
- [Why particle emitter cannot be detected by Thermal Camera](https://discourse.openrobotics.org/t/why-particle-emitter-cannot-be-detected-by-the-thermal-camera/47489) — 파티클 제약 공식 확인
- [Gazebo Sim (Harmonic) Plugins and Sensors for ROS2 - Medium](https://medium.com/@alitekes1/gazebo-sim-plugin-and-sensors-for-acquire-data-from-simulation-environment-681d8e2ad853) — 2025년 2월 실용 예제
- [PyImageSearch: Thermal Vision Fever Detector](https://pyimagesearch.com/2022/10/24/thermal-vision-fever-detector-with-python-and-opencv-starter-project/) — OpenCV 열화상 처리 방법론
- [Gazebo Release Features - Harmonic](https://gazebosim.org/docs/harmonic/release-features/) — Harmonic 신규 기능 목록
