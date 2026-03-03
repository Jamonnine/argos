# Day 9: 컴퓨터 비전과 ROS 2 통합

**날짜**: 2026-02-13 (00:09 ~ 01:10)
**학습 주제**: OpenCV 기초 + ROS 2 cv_bridge 통합
**소요 시간**: 약 1시간

---

## 📋 학습 목표 및 성과

### 목표
- [x] 컴퓨터 비전의 큰 그림 이해
- [x] OpenCV 기초 (HSV, 윤곽선, 모멘트)
- [x] ROS 2 cv_bridge 통합
- [x] 카메라 Publisher 노드 작성
- [x] 물체 감지 Subscriber 노드 작성
- [x] 실시간 이미지 처리 파이프라인 구축

### 성과
- ✅ OpenCV로 색상 기반 물체 감지 성공
- ✅ ROS 2 Topic으로 이미지 실시간 전송
- ✅ 30Hz 이미지 스트림 처리
- ✅ Publisher-Subscriber 패턴 실무 경험
- ✅ 센서 추상화 설계 이해

---

## 🎯 핵심 개념

### 1. 컴퓨터 비전의 본질

**"픽셀 배열 → 의미 있는 정보"**

카메라는 단순히 숫자 배열을 출력합니다 (640×480×3 = 약 92만 개 숫자). 컴퓨터 비전은 이것에서:
- "저기 빨간 컵이 있다"
- "컵의 3D 좌표는 (0.5, 0.3, 0.8)이다"

같은 **고수준 의미**를 추출하는 과정입니다.

**3-Layer 아키텍처**:
```
Application: 의사결정 (MoveIt, Navigation)
     ↑
Processing: 인식 & 해석 (OpenCV, 딥러닝)
     ↑
Sensor: 데이터 획득 (카메라, Gazebo)
```

---

### 2. HSV vs BGR: 색상 감지의 핵심

**BGR (카메라 원본)**:
- Blue-Green-Red 채널
- 조명에 민감: 밝은 곳 [0, 0, 255], 어두운 곳 [0, 0, 120]

**HSV (색상 분리)**:
- Hue(색상) - Saturation(채도) - Value(명도)
- **Hue는 조명 독립적**: 밝든 어두운든 빨간색은 H=0

**실무 원칙**: 색상 기반 필터링은 항상 HSV로 변환 후 처리.

**코드 패턴**:
```python
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
mask = cv2.inRange(hsv, lower_red, upper_red)
```

---

### 3. cv_bridge: 두 세계의 다리

**OpenCV**: `numpy.ndarray`, shape=(480, 640, 3)
**ROS 2**: `sensor_msgs/Image`, 직렬화된 메시지

**변환**:
```python
# ROS → OpenCV
cv_image = bridge.imgmsg_to_cv2(ros_msg, 'bgr8')

# OpenCV → ROS
ros_msg = bridge.cv2_to_imgmsg(cv_image, 'bgr8')
```

**왜 변환이 필요한가?**

ROS 2는 **분산 시스템**입니다. 노드들이 다른 프로세스, 다른 컴퓨터에서 실행 가능. 데이터를 **직렬화(Serialization)**해서 전송해야 합니다.

OpenCV Mat는 메모리 포인터 → 네트워크 전송 불가능
`sensor_msgs/Image`는 자기 완결적 메시지 → 전송 가능

**성능 고려**: 640×480×3 = 약 1MB, 30Hz면 초당 30MB 복사. 고해상도에서는 압축 필수.

---

### 4. Publisher-Subscriber 설계

**왜 2개의 노드로 분리?**

**단일 책임 원칙 (SRP)**:
- 카메라 노드: "이미지 획득" 만
- 감지 노드: "이미지 처리" 만

**재사용성**:
- 카메라 노드는 다른 알고리즘(얼굴 인식, QR 코드)에도 재사용
- Fan-Out 패턴: 하나의 카메라 → 여러 처리 노드

**테스트 가능성**:
- 카메라 없이도 감지 노드 테스트 가능 (`ros2 bag play`)
- 알고리즘만 독립적으로 벤치마크

**확장성**:
- 감지 노드가 느리면? 여러 개 띄워서 병렬 처리
- 카메라 여러 개? 각각 Publisher, 하나의 Subscriber

**실무 사례**: Tesla Autopilot - 8개 카메라, 각각 독립 Publisher, 하나의 센서 융합 노드.

---

## 🏗️ 시스템 아키텍처

### 데이터 흐름

```
[카메라 노드] (30Hz 타이머)
      ↓
  generate_test_image() → OpenCV Mat
      ↓
  cv_bridge.cv2_to_imgmsg() → sensor_msgs/Image
      ↓
  publisher.publish() → /camera/image_raw
      ↓
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  (ROS 2 DDS: 프로세스 간 통신)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
      ↓
[감지 노드] (Subscriber Callback)
      ↓
  image_callback(ros_msg)
      ↓
  cv_bridge.imgmsg_to_cv2() → OpenCV Mat
      ↓
  detect_red_objects() → HSV → 마스크 → 윤곽선
      ↓
  [(cx, cy, area), ...] → 감지 결과
      ↓
  (추후) MoveIt 2로 전달
```

**Navigation2와 비교**:

| 컴포넌트 | Navigation2 | 비전 시스템 |
|---------|-------------|-----------|
| **Sensor** | LiDAR → `/scan` | Camera → `/image_raw` |
| **Data Type** | LaserScan (거리 배열) | Image (픽셀 배열) |
| **Processing** | Costmap | 물체 감지 |
| **Frequency** | 10-20Hz | 10-30Hz |
| **Data Size** | ~10KB | ~1MB (압축 전) |

---

## 🔑 핵심 설계 원칙

### 1. 센서 추상화 (Sensor Abstraction)

카메라 노드는 **센서 추상화** 역할:
- 실제 카메라든, Gazebo든, 녹화 파일이든
- 모두 같은 인터페이스 (`/camera/image_raw`)

**Dependency Inversion Principle**: Downstream 노드(감지)는 구체적인 센서 종류가 아닌, 추상적인 토픽에 의존.

**테스트 전략**: 실제 카메라 없이도 개발 가능! 시뮬레이션, 테스트 이미지, 녹화 데이터 모두 동일한 방식.

---

### 2. 이벤트 기반 처리 (Event-Driven)

감지 노드는 **폴링(Polling) 없음**. 이미지가 도착하면 자동으로 콜백 호출.

**Reactive Architecture** (반응형 아키텍처):
- CPU 낭비 없음 (이미지 없을 때는 대기)
- 지연 최소화 (이미지 도착 즉시 처리)

**성능 최적화**:
- QoS 조정 (오래된 프레임 버리기)
- 프레임 스킵 (매 N번째만 처리)
- 병렬화 (여러 Subscriber)

---

### 3. 표준 준수 (ROS REP 표준)

**토픽 이름**: `/camera/image_raw` (표준)

**왜 표준을 따르는가?**

1. **Interoperability**: RViz, rqt_image_view 자동 인식
2. **Documentation**: 암묵적 문서화 (이름만 봐도 의미 파악)
3. **Ecosystem**: `image_transport` 같은 라이브러리가 자동 최적화

**안티패턴**: `/my_image`, `/cam1` 같은 비표준 이름
- 도구 자동 인식 실패
- 추가 설정 필요
- 다른 개발자 혼란

---

## 🛠️ 실습 코드

### 카메라 Publisher 핵심

```python
class CameraPublisher(Node):
    def __init__(self):
        # Publisher 생성 (표준 토픽)
        self.publisher_ = self.create_publisher(
            Image, '/camera/image_raw', 10
        )

        # 30Hz 타이머
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)

        # cv_bridge 초기화
        self.bridge = CvBridge()

    def timer_callback(self):
        image = self.generate_test_image()  # OpenCV Mat

        # OpenCV → ROS 변환
        ros_image = self.bridge.cv2_to_imgmsg(image, 'bgr8')

        # Timestamp 설정 (중요!)
        ros_image.header.stamp = self.get_clock().now().to_msg()
        ros_image.header.frame_id = 'camera_frame'

        # 발행
        self.publisher_.publish(ros_image)
```

**설계 포인트**:
- **Timestamp**: 센서 융합 시 시간 동기화 필수
- **frame_id**: TF Tree에서 카메라 위치 정의
- **타이머 정확성**: ROS 2 타이머는 wall-clock 기반 (실시간성 보장)

---

### 물체 감지 Subscriber 핵심

```python
class ObjectDetector(Node):
    def __init__(self):
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw',
            self.image_callback, 10
        )
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # ROS → OpenCV 변환
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # 물체 감지
        detections = self.detect_red_objects(cv_image)

        # 결과 처리
        self.process_detections(detections, msg.header.stamp)

    def detect_red_objects(self, image):
        # BGR → HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # 빨간색 마스크
        mask = cv2.inRange(hsv, lower_red, upper_red)

        # 윤곽선
        contours, _ = cv2.findContours(mask, ...)

        # 중심점 계산
        for contour in contours:
            M = cv2.moments(contour)
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            # ...
```

**설계 포인트**:
- **알고리즘 캡슐화**: `detect_red_objects()`는 재사용 가능
- **노이즈 필터링**: 면적 < 500 픽셀 제외
- **성능 측정**: FPS 계산으로 실시간성 확인

---

## 📊 학습 성과 자가 평가

| 평가 항목 | 점수 (1-5) | 비고 |
|----------|-----------|------|
| 컴퓨터 비전 개념 | 5/5 | 3-Layer 아키텍처 명확 |
| HSV vs BGR | 5/5 | 색상 감지에 왜 HSV인지 이해 |
| cv_bridge 역할 | 5/5 | 직렬화 필요성, 성능 고려 |
| Publisher-Subscriber | 5/5 | SRP, 재사용성, 확장성 |
| 센서 추상화 | 5/5 | DIP, 테스트 전략 |
| OpenCV 기초 | 4/5 | 기본 동작 이해, 고급 기법은 추후 |
| 실시간 처리 | 5/5 | 30Hz 이미지 스트림 성공 |

**종합 평가**: 4.9/5 - **Excellent**

Day 9는 "빠르게 전체 그림"이라는 전략에 매우 적합했습니다. 컴퓨터 비전의 기초부터 ROS 2 통합까지, 실무에서 바로 활용 가능한 파이프라인을 구축했습니다.

---

## 🚀 다음 단계

### Day 10: 깊이 카메라와 3D 인식

**목표**:
- RGB-D 카메라 (깊이 + 색상)
- Point Cloud Processing
- 2D 이미지 → 3D 좌표 변환
- `image_geometry` 패키지

**최종 목표**: "빨간 공의 3D 위치는 (0.5, 0.3, 0.8)이다" → MoveIt으로 집기

---

### Week 2 로드맵

**Day 8 (완료)**: MoveIt 2 기초 ✅
**Day 9 (완료)**: 컴퓨터 비전 + ROS 2 ✅
**Day 10 (다음)**: 깊이 카메라, Point Cloud
**Day 11-12**: 비전 + MoveIt 통합 (Pick & Place)
**Day 13-14**: 통합 프로젝트 (Navigation + Manipulation + Vision)

---

## 🎉 Day 9 완료!

**오늘의 핵심 메시지**:

"컴퓨터 비전은 로봇의 눈이다. 하지만 단순히 보는 것이 아니라, '의미를 이해하는' 것이다. OpenCV는 도구이고, cv_bridge는 통역자이며, ROS 2는 이 모든 것을 연결하는 신경계다."

**다음**: Day 10 - 2D를 넘어 3D로 (깊이 카메라) 🌐

---

> "Vision is the art of seeing what is invisible to others."
> (비전은 다른 사람들이 보지 못하는 것을 보는 예술이다) - Jonathan Swift

이제 로봇도 "보지 못하는 것"을 볼 수 있습니다: 물체의 3D 위치를! 🤖👁️
