# Daily Learning Log - Week 1, Day 2

**Date**: 2026-02-09 (예상)
**Phase**: Phase 1 - ROS 2 Fundamentals
**Focus**: Parameters, Launch Files, 다중 노드 시스템 기초
**Hours Logged**: ~4 hours

---

## 📋 Daily Goals - 완료!

- [x] Parameters 시스템 이해 및 활용
- [x] Launch Files 작성 (Python 기반)
- [x] YAML 설정 파일 통합
- [x] Temperature Monitor 노드 작성 (Subscriber)
- [x] Publisher-Subscriber 완전한 파이프라인 구축
- [x] 환경별 설정 분리 전략 학습

---

## 🎓 What I Learned Today

### 큰 그림: 설정과 코드의 분리

#### 왜 Parameters가 필요한가?

소프트웨어 공학의 핵심 원칙 중 하나는 **"설정과 코드의 분리"**입니다. 같은 코드를 다른 환경에서 다른 설정으로 실행해야 하는 경우가 매우 흔합니다:

- **개발 환경 vs 운영 환경**: 테스트 중에는 센서 업데이트 주기가 느려도 되지만, 실제 로봇은 빠른 응답이 필요합니다
- **로봇마다 다른 하드웨어**: 같은 소프트웨어를 다른 크기의 로봇에 적용할 때 속도 제한이 달라집니다
- **상황별 동작 변경**: 실내 주행과 실외 주행에서 다른 안전 임계값이 필요합니다

만약 이러한 설정을 코드에 하드코딩하면:
1. 환경이 바뀔 때마다 코드를 수정해야 합니다
2. 수정할 때마다 다시 빌드해야 합니다 (시간 소비)
3. 수정 과정에서 버그가 들어갈 위험이 있습니다
4. 여러 설정을 동시에 관리하기 어렵습니다

ROS 2의 **Parameters** 시스템은 이 문제를 해결합니다. 코드는 그대로 두고, 런타임에 설정을 주입하는 방식입니다.

---

### Parameters의 실무 가치

#### 설계 패턴: Dependency Injection

Parameters는 사실 **의존성 주입(Dependency Injection)** 패턴의 구현입니다. 노드는 "어떤 값이 필요하다"고 선언만 하고, 실제 값은 외부에서 주입받습니다.

```python
# 노드는 threshold_high가 필요하다고 선언
self.declare_parameter('threshold_high', 30.0)  # 기본값 30.0

# 실제 값은 런타임에 결정됨
threshold = self.get_parameter('threshold_high').value
```

이렇게 하면:
- **테스트 용이성**: 테스트 시 다른 임계값으로 쉽게 테스트 가능
- **재사용성**: 같은 노드를 다른 설정으로 여러 번 실행 가능
- **유지보수성**: 설정 변경 시 코드 수정 불필요

#### 계층적 설정 관리

실무에서는 설정을 계층적으로 관리합니다:

1. **코드 내 기본값**: 최소한의 동작을 보장
2. **YAML 파일**: 환경별 설정 (dev.yaml, prod.yaml)
3. **Launch File**: 특정 실행 시나리오
4. **런타임 동적 변경**: `ros2 param set` 명령어

이러한 계층 구조는 **우선순위**를 가집니다. 런타임 변경이 가장 높고, 코드 내 기본값이 가장 낮습니다. 이로 인해 유연성과 안정성을 동시에 확보할 수 있습니다.

---

### Launch Files: 시스템 오케스트레이션

#### 단일 노드에서 시스템으로

Day 1에서는 노드 하나를 `ros2 run`으로 실행했습니다. 하지만 실제 로봇 시스템은:
- 수십 개의 노드가 동시에 실행됩니다
- 각 노드마다 다른 parameters가 필요합니다
- 노드 간 의존성이 있을 수 있습니다 (A 노드가 실행된 후 B 노드 시작)

이 모든 것을 수동으로 관리하는 것은 불가능합니다. **Launch File**은 이러한 복잡한 시스템을 하나의 명령어로 시작할 수 있게 해주는 도구입니다.

#### Launch File의 실무 패턴

**환경별 Launch Files**:
```
launch/
├── robot.launch.py          # 공통 로직
├── robot_dev.launch.py      # 개발 환경
├── robot_prod.launch.py     # 운영 환경
└── robot_sim.launch.py      # 시뮬레이션
```

각 환경은 다른 parameters를 로드합니다:
- 개발: 느린 업데이트, 많은 로그, 안전 제한 낮음
- 운영: 빠른 업데이트, 필수 로그만, 안전 제한 높음
- 시뮬레이션: 가상 센서, 디버깅 활성화

이러한 분리는 **실수를 줄이고**, **명확한 의도를 전달**하며, **팀 협업을 쉽게** 만듭니다.

---

### 설계 전략: 설정의 범위

#### 무엇을 Parameter로 만들 것인가?

모든 값을 parameter로 만드는 것은 과도한 추상화입니다. 다음 기준으로 판단합니다:

**Parameter로 만들어야 할 것**:
- 환경에 따라 변할 수 있는 값 (임계값, 속도 제한)
- 하드웨어 스펙 (센서 업데이트 주기, 모터 최대 속도)
- 튜닝 가능한 값 (PID 게인, 필터 계수)

**하드코딩해도 되는 것**:
- 물리 상수 (중력 가속도, 파이 값)
- 프로토콜 규격 (메시지 타입, 토픽 이름)
- 알고리즘 로직

**실무 판단 기준**: "이 값이 코드 수정 없이 변경될 필요가 있는가?"

---

## 💻 오늘 작성한 코드

### Temperature Monitor Node (Subscriber)

```python
class TemperatureMonitor(Node):
    def __init__(self):
        super().__init__('temperature_monitor')

        # Parameters 선언
        self.declare_parameter('threshold_high', 30.0)
        self.declare_parameter('threshold_low', 15.0)

        # Subscriber 생성
        self.subscription = self.create_subscription(
            Float32,
            'temperature',
            self.temperature_callback,
            10
        )

    def temperature_callback(self, msg):
        temp = msg.data
        threshold_high = self.get_parameter('threshold_high').value
        threshold_low = self.get_parameter('threshold_low').value

        if temp > threshold_high:
            self.get_logger().warn(f'🔥 HIGH TEMP: {temp:.2f}°C')
        elif temp < threshold_low:
            self.get_logger().info(f'❄️  LOW TEMP: {temp:.2f}°C')
```

#### 설계 원칙 분석

**Separation of Concerns (관심사의 분리)**:
- 온도 모니터링 로직 (callback)
- 설정 관리 (parameters)
- 통신 (subscription)

이 세 가지가 명확히 분리되어 있어, 각각을 독립적으로 수정할 수 있습니다.

**Single Responsibility**:
이 노드는 오직 "온도 모니터링"만 합니다. 온도를 발행하지도 않고, 제어 명령을 보내지도 않습니다. 이러한 집중된 책임은 노드를 이해하기 쉽고 테스트하기 쉽게 만듭니다.

---

### Launch File 구조

```python
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_bringup',
            executable='temperature_sensor',
            name='temperature_sensor',
            output='screen',
        ),
        Node(
            package='my_robot_bringup',
            executable='temperature_monitor',
            name='temperature_monitor',
            output='screen',
            parameters=[{
                'threshold_high': 28.0,
                'threshold_low': 18.0,
            }]
        ),
    ])
```

#### 이 패턴의 강점

**선언적 설정 (Declarative Configuration)**:
"어떻게" 실행하는지가 아닌 "무엇을" 실행하는지를 선언합니다. 내부적으로 ROS 2가 노드 시작, 연결, 모니터링을 모두 처리합니다.

**버전 관리 가능**:
Launch file은 코드이므로 Git으로 버전 관리됩니다. "6개월 전 시스템 설정"을 정확히 재현할 수 있습니다.

**문서화 효과**:
Launch file을 보면 전체 시스템 구조를 한눈에 파악할 수 있습니다. 어떤 노드들이 실행되고, 어떤 parameters를 받는지 명확합니다.

---

## 🐛 Errors & Solutions

### Error 1: Parameter 타입 불일치

**Error Message**:
```
[ERROR] Parameter 'threshold_high' has wrong type: expected float, got int
```

**Root Cause**:
YAML 파일에서 `threshold_high: 30`으로 작성하면 정수로 해석됩니다. Python에서는 `30.0`으로 선언했으므로 타입이 맞지 않습니다.

**Solution**:
YAML에서 명시적으로 소수점 표기: `threshold_high: 30.0`

**Lesson**: ROS 2의 parameter 시스템은 타입에 엄격합니다. 이는 런타임 에러를 방지하는 안전 장치입니다.

---

### Error 2: Launch File 경로 문제

**Error Message**:
```
[ERROR] Package 'my_robot_bringup' not found
```

**Root Cause**:
Launch file은 `install/` 폴더에 설치되어야 하는데, `setup.py`에서 launch 파일을 등록하지 않았습니다.

**Solution**:
```python
data_files=[
    # Launch files 등록
    (os.path.join('share', package_name, 'launch'),
     glob('launch/*.launch.py')),
]
```

**Lesson**: ROS 2의 빌드 시스템은 명시적입니다. 파일이 존재한다고 자동으로 포함되지 않으며, `setup.py`에서 명시해야 합니다.

---

## 💡 Key Insights

### Insight 1: 설정은 코드보다 자주 변한다

실무에서 코드는 안정화되면 잘 바뀌지 않지만, 설정은 계속 조정됩니다. 로봇을 새로운 환경에 배포할 때마다, 센서를 교체할 때마다, 성능을 튜닝할 때마다 설정이 변경됩니다.

Parameters 시스템은 이러한 변경을 **안전하고 빠르게** 만듭니다. 코드를 수정하지 않으므로 버그 유입 위험이 없고, 빌드하지 않으므로 시간을 절약하며, 실행 중에도 변경할 수 있으므로 유연합니다.

### Insight 2: Launch Files는 시스템의 청사진

Launch file을 보면:
- 어떤 노드들이 협업하는가?
- 각 노드의 설정은 무엇인가?
- 노드 간 의존성은 어떤가?

이 모든 것을 알 수 있습니다. 새로운 팀원이 프로젝트에 합류했을 때, launch file을 보는 것만으로도 전체 시스템을 이해할 수 있습니다.

### Insight 3: YAML은 비개발자도 수정 가능

Parameters를 YAML 파일로 관리하면, 프로그래밍 지식이 없는 도메인 전문가(예: 로봇 운영자)도 설정을 조정할 수 있습니다. 이는 **협업의 범위를 확장**하고, 개발자가 단순 설정 변경에 시간을 쓰지 않게 합니다.

---

## 🎯 Progress Tracking

### Completed Today
- ✅ Parameters 시스템 이해 및 구현
- ✅ Temperature Monitor 노드 작성 (첫 Subscriber)
- ✅ Launch File 작성 및 테스트
- ✅ YAML 설정 파일 통합
- ✅ 환경별 설정 분리 전략 학습
- ✅ Publisher-Subscriber 완전한 파이프라인 구축

### Key Achievements
- 두 노드가 토픽을 통해 통신하는 시스템 완성
- 설정과 코드 분리를 통한 유연한 시스템 설계
- 실무 패턴 (환경별 설정, Launch Files) 학습

---

## 📝 Action Items for Tomorrow (Day 3)

- [ ] 세 번째 노드 추가: Data Processor
  - 온도 데이터의 통계 계산 (평균, 최대, 최소)
  - 슬라이딩 윈도우 기법 적용

- [ ] Multi-node 아키텍처 패턴
  - Fan-Out 패턴 (1 Publisher → N Subscribers)
  - Pipeline 패턴 (Sensor → Processor → Visualizer)

- [ ] 실시간 데이터 시각화
  - rqt_plot으로 온도 그래프
  - rqt_graph로 노드 관계 시각화

---

## 📊 Self-Assessment

| Aspect | Score (1-5) | Notes |
|--------|-------------|-------|
| Concept Understanding | 5/5 | Parameters와 Launch Files의 설계 의도 완전 이해 |
| Code Quality | 5/5 | 깨끗하고 재사용 가능한 코드 |
| Problem Solving | 4/5 | 타입 에러 해결, setup.py 이해 |
| System Design | 5/5 | 설정 계층 구조 설계 능력 향상 |

**Overall Feeling**: 😊 매우 만족

오늘은 "코드를 덜 쓰고 더 많이 이루는" 경험이었습니다. Parameters와 Launch Files는 코드 자체는 간단하지만, 시스템의 유연성과 유지보수성을 극적으로 향상시킵니다. 이것이 좋은 아키텍처의 힘입니다.

---

**End of Day 2 Log**

**Next Session**: Week 1, Day 3
**First Task**: Data Processor 노드 추가
**Goal**: 세 개 이상의 노드가 협업하는 실무 수준의 시스템 완성

---

**Tags**: #ROS2 #Week1 #Day2 #Parameters #LaunchFiles #Subscriber #Configuration
