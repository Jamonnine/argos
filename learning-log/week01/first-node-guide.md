# 첫 Python 노드 작성 완벽 가이드

> 모든 단계를 "왜?"와 함께 자세히 설명합니다.

---

## 🎯 무엇을 만들 것인가?

### 시스템 구조
```
[Temperature Sensor Node] → /temperature Topic → [Monitor Node]
      (Publisher)                                  (Subscriber)
```

**시나리오**:
- 온도 센서 노드가 1초마다 온도 데이터 발행
- 모니터 노드가 이를 받아서 경고 메시지 출력 (임계값 초과 시)

**실무 연결**:
- 실제 로봇: 센서 → 처리 → 제어 파이프라인의 축소판
- 이 패턴 = 모든 ROS 시스템의 기본

---

## 📂 Step 1: ROS 2 Workspace 생성

### 명령어
```bash
wsl -d Ubuntu
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### 왜 이렇게 하나?

**Q: 왜 `ros2_ws` 디렉토리를 만드나?**
- A: ROS 2는 "workspace" 개념 사용
- Workspace = 여러 패키지를 모아두는 작업 공간
- 빌드 결과물도 여기에 생성됨

**Q: 왜 `src` 폴더가 필요한가?**
- A: ROS 2 workspace 구조:
  ```
  ros2_ws/
  ├── src/           ← 소스 코드 (우리가 작성)
  ├── build/         ← 빌드 중간 파일 (자동 생성)
  ├── install/       ← 설치된 실행 파일 (자동 생성)
  └── log/           ← 로그 파일 (자동 생성)
  ```
- 소스와 빌드 결과를 분리 → 깔끔한 관리

**실무 Tip**:
- 한 workspace에 여러 패키지 관리 가능
- 회사/프로젝트별로 workspace 분리하는 것이 일반적

---

## 📦 Step 2: Python 패키지 생성

### 명령어
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_robot_bringup --dependencies rclpy std_msgs
```

### 명령어 분해 설명

**`ros2 pkg create`**: ROS 2 패키지 생성 도구
- 자동으로 필요한 파일 구조 생성 (boilerplate)
- 수동으로 만들면 실수하기 쉬움

**`--build-type ament_python`**: Python 패키지로 생성
- 선택지: `ament_python` (Python) vs `ament_cmake` (C++)
- Python = 빠른 프로토타이핑, 스크립팅
- C++ = 성능 중요한 부분 (센서 드라이버, 제어기)

**`my_robot_bringup`**: 패키지 이름
- 네이밍 컨벤션: `프로젝트명_용도`
- 예: `turtlebot3_bringup`, `robot_navigation`
- `bringup` = 시스템 시작/초기화용 패키지 (ROS 관례)

**`--dependencies rclpy std_msgs`**: 의존성 자동 추가
- `rclpy` = ROS 2 Python 라이브러리 (필수!)
- `std_msgs` = 표준 메시지 타입 (String, Int32 등)
- 이걸 명시하면 `package.xml`에 자동 추가됨

### 생성된 파일 구조
```
my_robot_bringup/
├── my_robot_bringup/          ← Python 모듈 디렉토리
│   └── __init__.py
├── resource/
│   └── my_robot_bringup       ← 리소스 마커 파일
├── test/                      ← 테스트 파일
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── package.xml                ← 패키지 메타데이터
├── setup.cfg                  ← 설정 파일
└── setup.py                   ← Python 설치 스크립트
```

**각 파일의 역할**:
- `package.xml`: 패키지 정보, 의존성 선언 (다른 ROS 패키지가 읽음)
- `setup.py`: Python 패키지 설치 방법 (entry points 정의)
- `setup.cfg`: 실행 파일이 설치될 위치 지정

---

## 🔧 Step 3: package.xml 이해하기

### 파일 열기
```bash
cd ~/ros2_ws/src/my_robot_bringup
cat package.xml
```

### 주요 섹션 설명

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_bringup</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>TODO: License declaration</license>

  <!-- 의존성 선언 -->
  <depend>rclpy</depend>
  <depend>std_msgs</depend>

  <!-- 빌드 도구 -->
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

**핵심 요소**:

1. **`<depend>`**: 런타임 + 빌드타임 의존성
   - 이 패키지가 작동하려면 필요한 다른 패키지들
   - `rclpy`: ROS 2 Python 클라이언트 라이브러리
   - `std_msgs`: 표준 메시지 타입

2. **`<test_depend>`**: 테스트 시에만 필요
   - 코드 스타일 검사 도구들
   - 실무: CI/CD 파이프라인에서 자동 검사

3. **`<build_type>`**: 빌드 시스템 지정
   - `ament_python` = Python 패키지
   - colcon이 이를 보고 빌드 방법 결정

**실무 중요도**: ⭐⭐⭐⭐⭐
- 의존성 관리 = 프로젝트 성공의 핵심
- 의존성 누락 → 다른 사람이 빌드 못 함
- 불필요한 의존성 → 설치 시간 낭비

---

## 📝 Step 4: 첫 번째 노드 작성 (Publisher)

### 파일 생성
```bash
cd ~/ros2_ws/src/my_robot_bringup/my_robot_bringup
touch temperature_sensor.py
chmod +x temperature_sensor.py
```

**Q: 왜 `chmod +x`?**
- A: 실행 권한 부여 (나중에 직접 실행 가능하게)
- Linux에서는 명시적으로 권한 설정 필요

### 코드 작성 (한 줄씩 설명)

```python
#!/usr/bin/env python3
# ↑ Shebang: 이 파일을 Python3로 실행하라는 의미
# Linux에서 ./temperature_sensor.py 로 직접 실행 가능하게 함

"""
Temperature Sensor Node
실제 센서 대신 랜덤 온도 데이터를 생성하여 발행
"""

# 1. 필수 라이브러리 임포트
import rclpy  # ROS 2 Python 라이브러리
from rclpy.node import Node  # Node 클래스 (모든 노드의 기본)
from std_msgs.msg import Float32  # Float32 메시지 타입
import random  # 랜덤 온도 생성용

# 2. Node 클래스 상속받아 커스텀 노드 정의
class TemperatureSensor(Node):
    """
    온도 센서 시뮬레이터 노드

    실무에서는: 실제 센서 드라이버가 센서 하드웨어에서 데이터 읽음
    여기서는: random으로 가상 온도 생성 (학습용)
    """

    def __init__(self):
        # 3. 부모 클래스 초기화 (Node 이름 지정)
        super().__init__('temperature_sensor')
        # ↑ 'temperature_sensor' = 노드 이름 (ros2 node list에 표시됨)

        # 4. Publisher 생성
        self.publisher_ = self.create_publisher(
            Float32,           # 메시지 타입: Float32
            'temperature',     # 토픽 이름: /temperature
            10                 # QoS (Queue Size): 10
        )
        # ↑ QoS(Quality of Service) = 통신 품질 설정
        #   10 = 최대 10개 메시지를 큐에 보관
        #   네트워크 지연 시 메시지 손실 방지

        # 5. 타이머 생성 (주기적 실행)
        timer_period = 1.0  # 1초 (1Hz)
        self.timer = self.create_timer(
            timer_period,           # 실행 주기 (초)
            self.timer_callback     # 실행할 함수
        )
        # ↑ 1초마다 timer_callback 함수가 자동으로 호출됨
        #   실무: 센서 샘플링 레이트와 동일하게 설정

        # 6. 카운터 초기화 (메시지 번호 추적용)
        self.count = 0

        # 7. 노드 시작 로그
        self.get_logger().info('Temperature Sensor Node Started')
        # ↑ ROS 2 로깅 시스템 사용 (print 대신!)
        #   이유: ros2 topic echo /rosout 로 모든 로그 중앙 관리 가능

    def timer_callback(self):
        """
        타이머가 호출하는 콜백 함수
        1초마다 실행되어 온도 데이터 발행
        """
        # 8. 메시지 객체 생성
        msg = Float32()

        # 9. 랜덤 온도 생성 (20~30도 범위)
        msg.data = 20.0 + random.random() * 10.0
        # ↑ 실무에서는: sensor.read_temperature() 같은 함수 호출

        # 10. 메시지 발행 (Publish)
        self.publisher_.publish(msg)
        # ↑ /temperature 토픽으로 전송
        #   모든 Subscriber가 이 메시지를 받음

        # 11. 로그 출력 (디버깅용)
        self.get_logger().info(f'Publishing: {msg.data:.2f}°C (#{self.count})')

        # 12. 카운터 증가
        self.count += 1


# 13. main 함수 (프로그램 시작점)
def main(args=None):
    """
    노드 실행 엔트리포인트
    """
    # 14. ROS 2 초기화
    rclpy.init(args=args)
    # ↑ ROS 2 시스템 초기화 (필수!)
    #   DDS 미들웨어 설정, 로깅 시스템 시작 등

    # 15. 노드 인스턴스 생성
    node = TemperatureSensor()

    # 16. 노드 실행 (무한 루프)
    try:
        rclpy.spin(node)
        # ↑ spin = "계속 돌려라"
        #   콜백 함수들이 호출될 수 있도록 대기
        #   Ctrl+C 누르기 전까지 계속 실행
    except KeyboardInterrupt:
        # 17. Ctrl+C 처리
        node.get_logger().info('Keyboard Interrupt (Ctrl+C)')
    finally:
        # 18. 정리 작업
        node.destroy_node()  # 노드 종료
        rclpy.shutdown()     # ROS 2 시스템 종료


# 19. 이 파일이 직접 실행될 때만 main 호출
if __name__ == '__main__':
    main()
```

---

## 🎓 코드 핵심 개념 설명

### 1️⃣ 왜 Node 클래스를 상속받나?

```python
class TemperatureSensor(Node):  # Node 상속
```

**이유**:
- `Node` 클래스가 ROS 2 기능 제공:
  - `create_publisher()`: Publisher 생성
  - `create_subscription()`: Subscriber 생성
  - `create_timer()`: 타이머 생성
  - `get_logger()`: 로깅 시스템
  - `create_service()`, `create_client()` 등

- 상속 = 이 모든 기능을 사용할 수 있음

**실무 패턴**:
```python
# 하나의 노드에 여러 Publisher/Subscriber 가능
class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # 여러 Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'status', 10)

        # 여러 Subscriber
        self.laser_sub = self.create_subscription(LaserScan, 'scan', ...)
        self.odom_sub = self.create_subscription(Odometry, 'odom', ...)
```

### 2️⃣ QoS (Queue Size) = 10의 의미

```python
self.create_publisher(Float32, 'temperature', 10)
                                               ↑
                                            QoS 설정
```

**간단 설명**:
- Queue = 메시지 대기열
- 크기 10 = 최대 10개 메시지 보관

**시나리오**:
```
Publisher가 빠르게 발행: 초당 100개
Subscriber가 느리게 처리: 초당 10개

→ Queue 없으면: 90개 메시지 손실
→ Queue 10이면: 최신 10개는 보관, 나머지 손실
```

**실무 선택 기준**:
- **센서 데이터** (고속): QoS 10~100
  - 오래된 데이터는 버려도 됨 (최신 데이터만 중요)
- **명령 데이터** (중요): QoS 100+ 또는 Reliable QoS
  - 모든 명령이 전달되어야 함
- **로그 데이터** (낮은 우선순위): QoS 1
  - 손실되어도 큰 문제 없음

### 3️⃣ 타이머 vs While 루프

**우리 코드**:
```python
self.create_timer(1.0, self.timer_callback)  # ✅ 권장
```

**안 좋은 방법**:
```python
# ❌ 이렇게 하지 마세요!
while True:
    publish_data()
    time.sleep(1.0)
```

**왜 타이머가 좋은가?**
1. **Non-blocking**: 다른 콜백 실행 가능
   - 타이머: 1초 기다리는 동안 Subscriber 콜백 실행 가능
   - While: 1초 동안 완전히 멈춤

2. **정확한 주기**: ROS 2가 내부적으로 타이밍 관리
   - 타이머: 정확히 1Hz 유지
   - While: sleep 오차 누적 (0.99Hz, 1.01Hz...)

3. **ROS 2 통합**: Ctrl+C로 깔끔하게 종료
   - 타이머: rclpy.spin()과 통합, 자동 종료
   - While: 수동 종료 처리 필요

### 4️⃣ rclpy.spin()의 역할

```python
rclpy.spin(node)  # 이게 뭐하는 거?
```

**실제 동작**:
```python
# spin()은 내부적으로 이렇게 동작:
while rclpy.ok():  # Ctrl+C 누르기 전까지
    # 1. 들어온 메시지 확인
    # 2. Subscriber 콜백 호출
    # 3. 타이머 확인
    # 4. 타이머 콜백 호출
    # 5. 잠깐 대기 (CPU 낭비 방지)
    pass
```

**왜 필요한가?**
- Publisher만 있어도 spin 필요!
- 이유: 타이머 콜백이 실행되려면 spin 돌아야 함
- spin 없으면: 타이머 등록만 하고 실행 안 됨

---

## 🔄 다음: Subscriber 노드 작성

이제 Publisher를 이해했으니, 다음은 **Subscriber 노드**를 작성하겠습니다.

**준비됐나요?** 계속 진행할까요? 🚀
