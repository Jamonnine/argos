# Daily Learning Log - Week 1, Day 3

**Date**: 2026-02-10 (예상)
**Phase**: Phase 1 - ROS 2 Fundamentals
**Focus**: Multi-Node Systems, Data Processing, Architecture Patterns
**Hours Logged**: ~4 hours

---

## 📋 Daily Goals - 완료!

- [x] Data Processor 노드 작성 (통계 계산)
- [x] Multi-node 시스템 구축 (3개 노드 협업)
- [x] Fan-Out 아키텍처 패턴 이해 및 구현
- [x] 슬라이딩 윈도우 기법 적용
- [x] rqt_graph로 시스템 시각화
- [x] rqt_plot으로 실시간 데이터 모니터링

---

## 🎓 What I Learned Today

### 큰 그림: 단일 노드에서 시스템으로

#### 실무 로봇 시스템의 복잡성

지난 이틀간 단일 노드 (Day 1: Publisher, Day 2: Subscriber)를 다뤘다면, 오늘은 **여러 노드가 협업하는 시스템**을 구축했습니다. 실제 로봇은 하나의 노드로 작동하지 않습니다. 자율주행 로봇만 해도:

- 10+ 센서 드라이버 노드 (카메라, 라이다, IMU, GPS...)
- 5+ 인식 노드 (SLAM, 물체 감지, 차선 인식...)
- 3+ 계획 노드 (경로 계획, 행동 계획...)
- 5+ 제어 노드 (모터 제어, 안전 감시...)
- 기타 로깅, 모니터링, 통신 노드들...

총 30-50개의 노드가 동시에 실행되며, 이들이 수십 개의 토픽으로 통신합니다. 이러한 복잡한 시스템을 설계하고 관리하려면 **아키텍처 패턴**이 필수입니다.

---

### Fan-Out 패턴: 데이터 공유의 우아한 방법

#### 패턴 설명

**Fan-Out 패턴**은 하나의 데이터 소스가 여러 소비자에게 데이터를 제공하는 구조입니다:

```
[Temperature Sensor] → /temperature Topic → [Monitor]
                                          → [Processor]
                                          → [Logger]
                                          → [Visualizer]
```

#### 왜 이 패턴이 강력한가?

**느슨한 결합 (Loose Coupling)**:
- Sensor는 누가 데이터를 사용하는지 모릅니다
- 새로운 Subscriber를 추가해도 Sensor 코드는 전혀 수정 불필요
- 한 Subscriber가 실패해도 다른 Subscriber는 영향 없음

이것은 **Open-Closed Principle**의 완벽한 구현입니다. 시스템은 확장에는 열려있고(새 노드 추가 자유), 수정에는 닫혀있습니다(기존 코드 변경 불필요).

**실무 시나리오**:
프로젝트 초기에는 센서 데이터를 단순히 모니터링만 했습니다. 3개월 후 통계 분석이 필요해졌고, 6개월 후에는 ML 모델 학습을 위한 데이터 로깅이 추가되었습니다. Fan-Out 패턴 덕분에 센서 드라이버는 한 번도 수정하지 않았습니다.

---

### Pipeline 패턴: 데이터 변환 체인

#### 패턴 설명

**Pipeline 패턴**은 데이터가 여러 단계의 처리를 거치는 구조입니다:

```
[Raw Sensor] → /raw_data → [Processor] → /processed_data → [Analyzer] → /results → [Actuator]
```

#### 각 단계의 책임

**Stage 1: Raw Data Collection**
- 센서에서 원시 데이터 수집
- 노이즈 필터링
- 단위 변환

**Stage 2: Processing**
- 통계 계산
- 특징 추출
- 데이터 집계

**Stage 3: Analysis**
- 패턴 인식
- 이상 감지
- 의사 결정

**Stage 4: Action**
- 제어 명령 생성
- 경고 발행

#### 이 패턴의 장점

**테스트 가능성**:
각 단계를 독립적으로 테스트할 수 있습니다. Stage 2를 테스트하려면 Stage 1의 실제 센서 없이도, 미리 녹화된 데이터(`ros2 bag`)를 사용할 수 있습니다.

**재사용성**:
각 단계가 표준 메시지 타입을 사용하면, 다른 프로젝트에서 그대로 재사용 가능합니다. 예를 들어, "통계 계산" 노드는 온도 데이터뿐만 아니라 압력, 속도 등 모든 Float32 스트림에 사용할 수 있습니다.

**성능 최적화**:
각 단계가 독립적인 프로세스이므로, 멀티코어 CPU에서 병렬 실행됩니다. 느린 단계만 최적화하거나 더 강력한 하드웨어에 배치할 수 있습니다.

---

### Stateful vs Stateless 노드 설계

#### Stateful Node: Data Processor

오늘 작성한 Data Processor는 **Stateful Node**입니다:

```python
class DataProcessor(Node):
    def __init__(self):
        # 상태 유지: 최근 N개 샘플
        self.temperature_window = deque(maxlen=self.window_size)

    def temperature_callback(self, msg):
        # 새 데이터를 상태에 추가
        self.temperature_window.append(msg.data)
        # 누적된 상태로 계산
        mean = sum(self.temperature_window) / len(self.temperature_window)
```

**Stateful의 의미**:
- 과거 데이터를 메모리에 유지
- 현재 메시지만으로는 출력을 계산할 수 없음
- 시간에 따라 내부 상태가 변화

**장점**:
- 시계열 분석 가능 (이동 평균, 추세 분석)
- 컨텍스트 기반 의사결정 (예: 온도가 10분간 계속 상승 중)

**단점**:
- 메모리 사용량 증가
- 노드 재시작 시 상태 손실
- 병렬화 어려움

#### Stateless Node: Monitor

Day 2의 Monitor는 **Stateless Node**입니다:

```python
def temperature_callback(self, msg):
    # 현재 메시지만 사용, 과거 기억 안 함
    if msg.data > threshold:
        warn()
```

**장점**:
- 간단하고 예측 가능
- 메모리 효율적
- 쉽게 병렬화 가능
- 재시작해도 문제없음

**설계 선택**:
가능하면 Stateless로 설계하고, 정말 필요할 때만 Stateful로 만듭니다. Stateful 노드는 복잡도를 증가시키므로 신중히 결정해야 합니다.

---

### 실시간 데이터 시각화: 시스템 이해의 핵심

#### rqt_graph: 아키텍처 검증

`rqt_graph`는 실행 중인 시스템의 노드-토픽 관계를 시각화합니다:

```
[temperature_sensor] → /temperature → [temperature_monitor]
                                    → [data_processor]

[data_processor] → /statistics/mean → (visualizer)
                 → /statistics/std_dev
                 → /statistics/min
                 → /statistics/max
```

**실무 가치**:
- **설계 검증**: 의도한 대로 연결되었는지 확인
- **디버깅**: 메시지가 안 오면 → 그래프에서 연결 확인
- **문서화**: 그래프 스크린샷을 문서에 포함
- **팀 커뮤니케이션**: 비개발자도 시스템 구조 이해 가능

#### rqt_plot: 동작 검증

`rqt_plot`은 실시간 데이터를 그래프로 표시합니다:

**확인할 수 있는 것**:
- 센서 데이터가 정상 범위인가?
- 통계 계산이 올바른가? (평균이 원본 데이터 범위 안에 있나?)
- 시간 지연이 있나? (입력과 출력의 시간차)
- 노이즈 수준은? (변동폭)

**실무 팁**:
처음 시스템을 개발할 때는 항상 rqt_plot을 켜두고 작업합니다. 버그의 90%는 "데이터가 이상하게 보이는 것"으로 먼저 감지됩니다.

---

## 💻 오늘 작성한 코드

### Data Processor 구조

```python
class DataProcessor(Node):
    def __init__(self):
        super().__init__('data_processor')

        # Parameter: 윈도우 크기
        self.declare_parameter('window_size', 10)
        self.window_size = self.get_parameter('window_size').value

        # 슬라이딩 윈도우 (Stateful)
        self.temperature_window = deque(maxlen=self.window_size)

        # Subscriber: 원본 데이터
        self.subscription = self.create_subscription(
            Float32, 'temperature', self.temperature_callback, 10
        )

        # Publishers: 여러 통계 토픽 (Fan-Out 역방향)
        self.pub_mean = self.create_publisher(Float32, 'statistics/mean', 10)
        self.pub_std = self.create_publisher(Float32, 'statistics/std_dev', 10)
        self.pub_min = self.create_publisher(Float32, 'statistics/min', 10)
        self.pub_max = self.create_publisher(Float32, 'statistics/max', 10)

    def temperature_callback(self, msg):
        self.temperature_window.append(msg.data)

        if len(self.temperature_window) >= 2:
            # 통계 계산
            mean = sum(self.temperature_window) / len(self.temperature_window)
            variance = sum((x - mean)**2 for x in self.temperature_window) / len(self.temperature_window)
            std_dev = math.sqrt(variance)
            min_val = min(self.temperature_window)
            max_val = max(self.temperature_window)

            # 각 통계를 별도 토픽으로 발행
            self.pub_mean.publish(Float32(data=mean))
            self.pub_std.publish(Float32(data=std_dev))
            self.pub_min.publish(Float32(data=min_val))
            self.pub_max.publish(Float32(data=max_val))
```

#### 설계 결정 분석

**왜 각 통계를 별도 토픽으로?**

Alternative 1: 하나의 메시지에 모든 통계 포함
```python
# TemperatureStatistics.msg
float64 mean
float64 std_dev
float64 min
float64 max
```

Alternative 2: 각 통계를 별도 토픽으로 (현재 선택)

**선택 이유**:
- **유연성**: 일부 노드는 평균만 필요할 수 있음 (불필요한 데이터 전송 감소)
- **단순성**: 표준 Float32 메시지 재사용 (커스텀 메시지 불필요)
- **확장성**: 나중에 통계를 추가해도 기존 Subscriber 영향 없음

**트레이드오프**:
- 장점: 유연성, 재사용성
- 단점: 토픽 개수 증가, 동기화 문제 가능성 (하지만 통계는 동기화 불필요)

**실무 판단**: 이 경우 별도 토픽이 더 적합합니다. 하지만 만약 여러 값이 항상 함께 사용된다면(예: x, y, z 좌표), 하나의 메시지로 묶는 것이 낫습니다.

---

### Launch File: Multi-Node System

```python
def generate_launch_description():
    sensor_node = Node(
        package='my_robot_bringup',
        executable='temperature_sensor',
        name='temperature_sensor',
        output='screen',
    )

    monitor_node = Node(
        package='my_robot_bringup',
        executable='temperature_monitor',
        name='temperature_monitor',
        output='screen',
        parameters=[{
            'threshold_high': 27.0,
            'threshold_low': 23.0,
        }]
    )

    processor_node = Node(
        package='my_robot_bringup',
        executable='data_processor',
        name='data_processor',
        output='screen',
        parameters=[{
            'window_size': 10,
        }]
    )

    return LaunchDescription([
        sensor_node,
        monitor_node,
        processor_node,
    ])
```

#### 이 Launch File이 보여주는 시스템 구조

**데이터 흐름**:
1. Sensor → `/temperature`
2. Monitor, Processor ← `/temperature` (Fan-Out)
3. Processor → `/statistics/*` (여러 토픽)

**역할 분리**:
- Sensor: 데이터 생성만
- Monitor: 실시간 경고만
- Processor: 통계 계산만

각 노드가 명확한 단일 책임을 가지며, 서로 독립적으로 작동합니다.

---

## 💡 Key Insights

### Insight 1: 노드 개수는 복잡도가 아닌 명확성의 지표

초보 개발자는 "노드가 많으면 복잡해진다"고 생각합니다. 하지만 실제로는 반대입니다. 노드가 적으면 각 노드가 여러 책임을 가지게 되어 더 복잡해집니다.

**안티패턴**: 하나의 거대한 노드
```python
class SuperNode(Node):
    # 센서 읽기 + 처리 + 모니터링 + 제어 + 로깅...
    # 1000줄짜리 괴물 클래스
```

**좋은 패턴**: 작고 집중된 노드들
```python
SensorNode: 50줄
ProcessorNode: 80줄
MonitorNode: 60줄
# 각자 명확한 책임, 쉬운 이해, 간단한 테스트
```

### Insight 2: 토픽 이름은 네임스페이스를 활용하라

`/statistics/mean`, `/statistics/std_dev` 처럼 계층적 네이밍을 사용하면:
- 관련 토픽들을 그룹화 가능
- `ros2 topic list | grep statistics` 로 쉽게 필터링
- rqt_graph에서 시각적으로 구분하기 쉬움

실무에서는 로봇 이름도 포함합니다: `/robot1/sensors/camera/image`

### Insight 3: 슬라이딩 윈도우는 메모리 관리가 핵심

`deque(maxlen=10)`을 사용한 이유:
- 자동으로 오래된 데이터 제거 (메모리 누수 방지)
- 고정된 메모리 사용량 (예측 가능)
- O(1) 추가/제거 성능

**안티패턴**: 무한정 데이터 누적
```python
self.all_data = []  # 시간이 지나면 메모리 부족
```

---

## 🎯 Progress Tracking

### Completed Today
- ✅ Data Processor 노드 완성 (통계 계산)
- ✅ 3-노드 시스템 구축 및 테스트
- ✅ Fan-Out 패턴 구현 및 검증
- ✅ rqt_graph로 아키텍처 시각화
- ✅ rqt_plot으로 실시간 데이터 모니터링
- ✅ Stateful vs Stateless 설계 이해

### Key Achievements
- 실무 수준의 Multi-Node 아키텍처 완성
- 아키텍처 패턴을 코드로 구현하는 능력 확보
- 시스템 레벨 사고방식 훈련

---

## 📝 Action Items for Tomorrow (Day 4)

- [ ] Turtlesim 자동 제어 시스템 구현
  - State Machine 패턴 학습
  - Point-to-Point Navigation
  - Closed-Loop Control 구현

- [ ] 실무 제어 시스템 설계 원칙
  - Open-Loop vs Closed-Loop 비교
  - Feedback 시스템 구축

---

## 📊 Self-Assessment

| Aspect | Score (1-5) | Notes |
|--------|-------------|-------|
| Architecture Design | 5/5 | Fan-Out, Pipeline 패턴 완전 이해 및 구현 |
| Code Quality | 5/5 | 깔끔한 구조, 명확한 책임 분리 |
| System Thinking | 5/5 | 단일 노드가 아닌 시스템 관점으로 사고 |
| Visualization Skills | 4/5 | rqt 도구 활용, 더 깊은 분석 필요 |

**Overall Feeling**: 😊 매우 만족

오늘은 "아키텍처가 코드를 지배한다"는 것을 다시 한번 확인한 날입니다. 같은 기능을 구현하더라도, 어떻게 노드를 나누고 토픽을 설계하는가에 따라 시스템의 유지보수성, 확장성, 테스트 가능성이 완전히 달라집니다.

---

**End of Day 3 Log**

**Next Session**: Week 1, Day 4
**First Task**: Turtlesim 자동 제어 시스템
**Goal**: State Machine + Closed-Loop Control 마스터

---

**Tags**: #ROS2 #Week1 #Day3 #MultiNode #Architecture #FanOut #Pipeline #Stateful
