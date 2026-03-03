# Daily Learning Log - Week 1, Day 1

**Date**: 2026-02-08
**Phase**: Phase 1 - ROS 2 Fundamentals
**Focus**: 환경 구축, 핵심 개념 이해, 첫 Publisher 노드 작성
**Hours Logged**: ~5 hours

---

## 📋 Daily Goals - 완료!

- [x] 현재 환경 확인 (Ubuntu 24.04, WSL2)
- [x] ROS 2 Jazzy 설치 검증
- [x] Turtlesim으로 ROS 2 첫 경험
- [x] 핵심 개념 이해 (Node, Topic, Publisher/Subscriber)
- [x] ros2_ws를 ARGOS로 이동 (포트폴리오 구조)
- [x] 첫 Python Publisher 노드 작성
- [x] 빌드 및 실행 성공

---

## 🎓 What I Learned Today

### 큰 그림: ROS 2 아키텍처 이해

#### ROS 2의 설계 철학
로봇 시스템은 본질적으로 복잡한 분산 시스템입니다. 수많은 센서, 알고리즘, 제어기가 실시간으로 협업해야 하는데, 만약 이 모든 것을 하나의 거대한 프로그램으로 만든다면 다음과 같은 심각한 문제에 직면하게 됩니다:

1. **단일 실패점**: 한 부분에 버그가 있으면 전체 시스템이 멈춥니다
2. **병렬 처리 불가**: 최신 멀티코어 CPU의 성능을 활용할 수 없습니다
3. **유지보수 악몽**: 코드가 서로 얽혀있어 수정이 거의 불가능합니다
4. **팀 협업 불가**: 여러 개발자가 동시에 작업할 수 없습니다
5. **재사용 불가**: 다른 프로젝트에서 코드를 재사용할 수 없습니다

ROS 2는 이러한 문제를 해결하기 위해 **마이크로서비스 아키텍처**를 채택했습니다. 이는 현대 웹 개발에서 사용하는 것과 동일한 설계 패턴으로, 하나의 거대한 시스템을 작고 독립적인 서비스들로 분해하는 접근 방식입니다.

#### 분산 시스템의 핵심 개념

**Node (노드)**: 독립적으로 실행되는 프로세스
- 각 노드는 하나의 명확한 책임을 가집니다 (Single Responsibility Principle)
- 노드는 다른 노드의 존재를 몰라도 됩니다 (Loose Coupling)
- 하나의 노드가 실패해도 다른 노드는 계속 작동합니다 (Fault Isolation)

**Topic (토픽)**: 노드 간 비동기 통신 채널
- Publisher-Subscriber 패턴을 구현한 것입니다
- 발행자는 "누가 이 데이터를 사용할지" 몰라도 됩니다
- 구독자는 "누가 이 데이터를 보내는지" 몰라도 됩니다
- 이러한 분리(Decoupling)가 시스템의 확장성을 만듭니다

#### DDS 미들웨어: 숨은 영웅

ROS 2는 DDS(Data Distribution Service)라는 산업 표준 미들웨어 위에 구축되어 있습니다. DDS는 다음과 같은 복잡한 작업들을 자동으로 처리해줍니다:

- **서비스 발견**: 새로운 노드가 시작되면 자동으로 다른 노드들에게 알립니다
- **데이터 직렬화**: Python 객체를 네트워크로 전송 가능한 바이트로 변환합니다
- **QoS 관리**: 네트워크 상황에 따라 메시지 전달 방식을 조정합니다
- **멀티캐스트**: 하나의 메시지를 여러 구독자에게 효율적으로 전달합니다

이 덕분에 우리 개발자는 복잡한 네트워킹 코드를 작성할 필요 없이, 단순히 `publisher.publish(msg)` 한 줄로 데이터를 전송할 수 있습니다.

---

### 설계 원리: 왜 이렇게 만들어졌는가?

#### Publisher-Subscriber 패턴의 깊은 의미

이 패턴이 강력한 이유는 **시간적 분리(Temporal Decoupling)**와 **공간적 분리(Spatial Decoupling)**를 동시에 제공하기 때문입니다.

**공간적 분리**: Publisher와 Subscriber가 서로를 몰라도 됩니다
- Camera 노드는 어떤 노드가 이미지를 사용하는지 알 필요가 없습니다
- SLAM 노드는 이미지가 어디서 오는지 알 필요가 없습니다
- 이로 인해 시스템에 새로운 노드를 추가할 때 기존 코드를 전혀 수정하지 않아도 됩니다

**시간적 분리**: Publisher와 Subscriber가 동시에 실행될 필요가 없습니다
- Publisher가 먼저 시작해서 데이터를 발행할 수 있습니다
- 나중에 Subscriber가 시작되면 자동으로 연결됩니다
- 한쪽이 잠시 죽었다가 다시 살아나도 자동으로 재연결됩니다

이러한 특성 덕분에 로봇 시스템은 매우 유연하고 견고해집니다. 실제 로봇 운영 중에 일부 노드가 실패하거나 재시작되어도 전체 시스템은 계속 작동할 수 있습니다.

#### 계층적 아키텍처 (Layered Architecture)

실무 로봇 시스템은 일반적으로 다음과 같은 계층 구조를 가집니다:

**Hardware Layer (하드웨어 계층)**
- 센서 드라이버, 모터 컨트롤러 등 물리적 하드웨어와 직접 통신
- 이 계층은 하드웨어에 종속적이지만, 나머지 계층은 독립적입니다

**Control Layer (제어 계층)**
- PID 컨트롤러, 모터 제어 알고리즘
- 저수준 제어 로직을 담당합니다

**Perception Layer (인식 계층)**
- SLAM, 물체 인식, 센서 융합
- 센서 데이터를 의미 있는 정보로 변환합니다

**Application Layer (응용 계층)**
- 경로 계획, 작업 스케줄링, 의사결정
- 로봇의 "뇌" 역할을 합니다

이러한 계층 구조는 다음과 같은 설계 규칙을 따릅니다:
- 상위 계층은 하위 계층에 의존할 수 있습니다 (예: 경로 계획이 SLAM 결과를 사용)
- 하위 계층은 상위 계층을 알아서는 안 됩니다 (예: 카메라 드라이버가 경로 계획을 알 필요 없음)

이 규칙을 지키면 시스템의 각 부분을 독립적으로 개발하고 테스트할 수 있으며, 나중에 쉽게 교체하거나 업그레이드할 수 있습니다.

---

### 실무 설계 전략

#### 확장성 (Scalability) 설계

**수평 확장 (Horizontal Scaling)**:
동일한 노드를 여러 개 실행하여 부하를 분산시키는 방법입니다. 예를 들어, 로봇에 카메라가 3개 있다면 각각에 대해 독립적인 이미지 처리 노드를 실행할 수 있습니다. ROS 2의 토픽 시스템은 이를 자연스럽게 지원합니다.

**수직 확장 (Vertical Scaling)**:
더 강력한 알고리즘으로 노드를 교체하는 방법입니다. 예를 들어, 초기에는 간단한 물체 감지 알고리즘을 사용하다가, 나중에 YOLO 같은 딥러닝 모델로 업그레이드할 수 있습니다. 중요한 점은 토픽 인터페이스가 동일하다면 다른 노드들은 전혀 영향을 받지 않는다는 것입니다.

#### 데이터 흐름 중심 설계 (Data Flow Driven Design)

실무에서 새로운 로봇 시스템을 설계할 때는 코드를 먼저 작성하지 않습니다. 대신 다음 순서를 따릅니다:

1. **데이터 흐름 다이어그램 그리기**: 종이나 화이트보드에 센서부터 액추에이터까지 데이터가 어떻게 흐르는지 그립니다
2. **각 처리 단계를 박스로 표현**: 각 박스가 하나의 노드가 됩니다
3. **화살표로 데이터 흐름 표시**: 각 화살표가 하나의 토픽이 됩니다
4. **토픽 인터페이스 정의**: 어떤 메시지 타입을 사용할지 결정합니다
5. **그 후에야 코드 작성**: 각 노드를 독립적으로 구현합니다

이 접근 방식의 장점은 전체 시스템을 먼저 이해하고, 각 부분의 역할과 인터페이스가 명확해진 후에 코드를 작성하기 때문에 나중에 큰 리팩토링이 필요 없다는 것입니다.

---

## 💻 오늘 작성한 코드

### Temperature Sensor Node 구조

```python
class TemperatureSensor(Node):
    def __init__(self):
        super().__init__('temperature_sensor')
        self.publisher_ = self.create_publisher(Float32, 'temperature', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = Float32()
        msg.data = 20.0 + random.random() * 10.0
        self.publisher_.publish(msg)
```

#### 이 코드가 보여주는 설계 원칙들

**Single Responsibility (단일 책임)**:
이 노드는 오직 하나의 일만 합니다 - 온도 데이터를 발행하는 것입니다. 데이터를 어떻게 처리할지, 누가 사용할지는 전혀 신경 쓰지 않습니다. 이러한 집중된 책임은 코드를 이해하기 쉽게 만들고, 테스트하기 쉽게 만들며, 재사용하기 쉽게 만듭니다.

**Open-Closed Principle (개방-폐쇄 원칙)**:
이 노드는 확장에는 열려있고 수정에는 닫혀있습니다. 새로운 노드가 온도 데이터를 사용하고 싶다면, 이 노드를 수정할 필요 없이 그냥 `/temperature` 토픽을 구독하기만 하면 됩니다. 반대로, 온도 센서를 실제 하드웨어 센서로 교체하려면 `random.random()` 부분만 센서 읽기 코드로 바꾸면 되고, 다른 부분은 그대로 유지됩니다.

**Dependency Inversion (의존성 역전)**:
이 노드는 구체적인 구현이 아닌 추상적인 인터페이스(Topic)에 의존합니다. Float32라는 표준 메시지 타입을 사용하기 때문에, 이 데이터를 받는 쪽도 같은 표준을 따르기만 하면 어떤 노드든 상관없이 작동합니다.

---

## 🤖 AI와의 협업 경험

### 문제 해결 과정: WSL 경로 문제

오늘 가장 많은 시간을 소비한 것은 WSL 환경에서의 경로 문제였습니다. 이 경험을 통해 배운 것:

**문제의 본질**:
WSL2 환경에서는 세 가지 다른 경로 시스템이 공존합니다:
1. Windows 경로 (`C:\Users\...`)
2. WSL 마운트 경로 (`/mnt/c/Users/...`)
3. WSL 내부 경로 (`/home/username/...`)

여기에 심볼릭 링크까지 추가되면 상황이 복잡해집니다. 다양한 도구들(Write, Edit, Bash)이 각각 이 경로들을 다르게 해석하면서 문제가 발생했습니다.

**해결 과정**:
- Write 도구: WSL 경로 실패 → Windows 경로로 성공
- Bash 도구: Git Bash PATH 혼동 문제
- 최종 해결: Windows 절대 경로 (`C:/Users/...`) 사용

**실무 교훈**:
크로스 플랫폼 개발에서는 항상 경로 문제가 발생합니다. 중요한 것은:
1. 각 도구가 어떤 환경에서 실행되는지 파악
2. 물리적 경로와 논리적 경로를 구분
3. 가장 호환성 높은 경로 형식 사용
4. 문제 발생 시 체계적으로 진단 (`pwd -P`, `file` 명령어 등)

이러한 "환경 문제"는 실무에서 매우 흔하며, 이를 효율적으로 해결하는 능력이 시니어 개발자와 주니어 개발자를 구분하는 중요한 요소입니다.

---

## 🐛 Errors & Solutions

### Error 1: 깨진 심볼릭 링크
**Context**: ros2_ws 생성 시도

**Error Message**:
```
mkdir: File exists
cd: No such file or directory
```

**Root Cause**:
이전에 누군가(아마도 초기 설정 시도)가 `~/ros2_ws`를 ARGOS 폴더로 연결하는 심볼릭 링크를 만들었지만, 실제 대상 폴더는 생성하지 않았습니다. 심볼릭 링크는 단순히 "바로가기"일 뿐이므로, 가리키는 대상이 없으면 깨진 링크가 됩니다.

**Solution**:
```bash
rm ~/ros2_ws  # 깨진 링크 제거
mkdir -p ~/ros2_ws/src  # 실제 디렉토리 생성
```

**Lesson Learned**:
심볼릭 링크는 편리하지만, 링크와 실제 파일/폴더의 관계를 명확히 이해해야 합니다. `file` 명령어와 `pwd -P`는 심볼릭 링크 문제를 진단하는 데 매우 유용한 도구입니다.

---

### Error 2: WSL 경로 접근 실패
**Context**: Claude 도구로 파일 작성 시도

**Error Message**:
```
/usr/bin/bash: cd: /home/jamonnine/...: No such file or directory
C:/Program Files/Git/home/...
```

**Root Cause**:
Bash 도구가 Windows의 Git Bash 환경에서 실행되면서, WSL 내부 경로(`/home/jamonnine/`)를 Git Bash 경로로 잘못 해석했습니다. Git Bash는 자체적인 경로 변환 로직을 가지고 있어서 `/home`을 Git 설치 경로 아래로 해석하려고 시도했습니다.

**Solution**:
Windows 절대 경로를 직접 사용: `C:/Users/USER/Desktop/ARGOS/ros2_ws/...`

**Lesson Learned**:
크로스 플랫폼 개발에서는 각 도구가 어떤 환경에서 실행되는지 이해하는 것이 중요합니다. WSL 환경이라고 해서 모든 도구가 WSL 경로를 이해하는 것은 아닙니다. 특히 Windows에서 실행되는 CLI 도구들은 Windows 경로를 사용해야 할 때가 많습니다.

---

### Error 3: 패키지 중복
**Context**: colcon build 시도

**Error Message**:
```
Duplicate package names not supported:
- my_robot_bringup
- src/my_robot_bringup
```

**Root Cause**:
패키지 생성을 시도하는 과정에서 잘못된 위치(ros2_ws 루트)에 패키지 폴더가 생성되었습니다. ROS 2의 빌드 시스템인 colcon은 workspace를 재귀적으로 탐색하여 모든 패키지를 찾는데, 같은 이름의 패키지가 두 곳에서 발견되자 에러를 발생시켰습니다.

**Solution**:
```bash
rm -rf ~/ros2_ws/my_robot_bringup  # 잘못된 위치의 패키지 제거
```

**Lesson Learned**:
ROS 2의 workspace 구조를 이해하는 것이 중요합니다:
- `src/`: 모든 소스 코드 패키지가 여기에 위치
- `build/`: 빌드 중간 파일 (자동 생성)
- `install/`: 설치된 실행 파일 (자동 생성)
- `log/`: 빌드 로그 (자동 생성)

이 구조는 관례이자 표준이며, 이를 따르지 않으면 빌드 시스템이 혼란을 겪습니다.

---

## 💡 Key Insights

### Insight 1: 아키텍처가 코드보다 중요하다

오늘 가장 큰 깨달음은 "좋은 아키텍처가 좋은 코드보다 중요하다"는 것입니다. 우리가 작성한 temperature_sensor 노드는 코드 자체로는 매우 간단합니다 - 50줄도 안 됩니다. 하지만 이 노드가 강력한 이유는 코드의 복잡도 때문이 아니라, ROS 2가 제공하는 아키텍처 패턴을 올바르게 따랐기 때문입니다.

이 노드는:
- 독립적으로 실행 가능합니다 (다른 노드 필요 없음)
- 다른 노드와 느슨하게 결합되어 있습니다 (Topic을 통해서만 통신)
- 확장 가능합니다 (Monitor 노드를 추가해도 수정 불필요)
- 재사용 가능합니다 (다른 로봇 프로젝트에서도 사용 가능)

이 모든 장점은 아키텍처 레벨의 결정에서 나옵니다. 만약 이 기능을 ROS 2 없이 구현했다면, 같은 수준의 유연성을 얻기 위해 수백 줄의 추가 코드가 필요했을 것입니다.

### Insight 2: 도구는 환경을 탄다

WSL, Git Bash, Windows CMD, PowerShell, Python, Bash - 오늘 하루만 해도 수많은 도구와 환경을 오갔습니다. 각 도구는 자신만의 경로 해석 방식, 환경 변수, 기본 설정을 가지고 있습니다.

실무 개발자가 되려면 단순히 코드를 작성하는 능력뿐만 아니라, 이러한 도구들의 특성을 이해하고 적절히 활용하는 능력이 필요합니다. 오늘 겪은 경로 문제는 좌절스러웠지만, 이를 해결하는 과정에서 WSL 환경에 대한 깊은 이해를 얻었습니다.

### Insight 3: 에러는 학습의 기회다

오늘 만난 모든 에러 - 깨진 심볼릭 링크, 경로 문제, 패키지 중복 - 는 처음에는 장애물처럼 보였지만, 결과적으로는 시스템을 더 깊이 이해하게 만든 학습 기회였습니다.

특히 중요한 것은 "에러를 체계적으로 디버깅하는 능력"입니다. 에러 메시지를 주의 깊게 읽고, 근본 원인을 파악하고, 여러 가능한 해결책을 시도하고, 최종적으로 작동하는 해결책을 찾는 이 전체 과정이 실무 능력입니다.

### Insight 4: AI는 도구이지 대체재가 아니다

Claude와 함께 작업하면서 느낀 것은, AI가 많은 것을 도와주지만 결국 이해는 사람이 해야 한다는 것입니다. AI가 코드를 작성해주고, 에러를 해결해주고, 개념을 설명해줘도, 최종적으로 "왜 이렇게 작동하는가?"를 이해하는 것은 학습자의 책임입니다.

오늘의 접근 방식 - "큰 그림을 먼저 이해하고, 원리를 파고들고, 그 다음 코드를 보는 것" - 은 이러한 깊은 이해를 만드는 데 매우 효과적입니다.

---

## 🎯 Progress Tracking

### Completed Today
- ✅ WSL2 환경 검증 및 ROS 2 Jazzy 확인
- ✅ Turtlesim으로 ROS 2 Publisher-Subscriber 패턴 직접 체험
- ✅ ros2 CLI 도구 사용법 학습 (node, topic, interface 명령어)
- ✅ ros2_ws를 ARGOS 프로젝트로 통합 (포트폴리오 구조 완성)
- ✅ 첫 ROS 2 Python 패키지 생성 (my_robot_bringup)
- ✅ Temperature Sensor Publisher 노드 작성
- ✅ setup.py 설정 및 entry point 등록
- ✅ colcon build 성공
- ✅ 노드 실행 및 실시간 데이터 확인
- ✅ ROS 2 아키텍처와 설계 원리 깊이 이해
- ✅ MEMORY.md에 핵심 개념 및 에러 해결 방법 저장

### Challenges Faced & Overcome

**Challenge 1: 깨진 심볼릭 링크**
초기에 ros2_ws 생성 시 기존의 깨진 심볼릭 링크 때문에 혼란스러웠습니다. `file` 명령어로 진단하고, 링크와 실제 폴더의 관계를 이해한 후 해결했습니다.

**Challenge 2: WSL 경로 복잡성**
여러 도구가 경로를 다르게 해석하는 문제로 많은 시간을 소비했습니다. 각 도구의 특성을 이해하고, Windows 절대 경로를 사용하는 것으로 해결했습니다. 이 과정에서 크로스 플랫폼 개발의 복잡성을 실감했습니다.

**Challenge 3: 개념의 깊이**
Node, Topic, Publisher/Subscriber 같은 개념들이 처음에는 단순해 보였지만, 실제로는 분산 시스템, 소프트웨어 아키텍처, 설계 패턴 등 깊은 배경 지식을 요구했습니다. 단순히 "어떻게 사용하는가"가 아닌 "왜 이렇게 설계되었는가"를 이해하는 데 집중했습니다.

### Still Learning / Questions for Tomorrow

**질문 1: QoS의 고급 설정**
오늘은 Queue Size (10)만 사용했는데, ROS 2의 QoS는 훨씬 더 복잡한 설정을 제공합니다 (Reliability, Durability, Deadline 등). 실무에서 이러한 설정들을 언제 어떻게 사용하는지 더 배워야 합니다.

**질문 2: 대규모 시스템에서의 노드 관리**
실제 로봇에 노드가 수십 개 있을 때, 이들을 어떻게 조직하고 관리하는지 궁금합니다. Launch file이 그 해답일 것 같은데, 내일 배울 예정입니다.

**질문 3: 에러 처리와 Fault Tolerance**
오늘 만든 노드는 에러 처리가 거의 없습니다. 실무에서는 센서 읽기 실패, 네트워크 단절 등 다양한 예외 상황을 어떻게 처리하는지 배워야 합니다.

---

## 📝 Action Items for Tomorrow

### Week 1, Day 2 계획

- [ ] Subscriber 노드 작성 (temperature_monitor)
  - 온도 데이터를 구독하여 임계값 체크
  - 경고 메시지 발행 (새로운 Topic)
  - Publisher ↔ Subscriber 완전한 파이프라인 완성

- [ ] Parameters 학습
  - 노드 설정을 동적으로 변경하는 방법
  - 임계값을 parameter로 설정
  - 런타임에 parameter 변경 실습

- [ ] Launch Files 기초
  - 여러 노드를 한 번에 실행
  - Python launch file 작성
  - Parameter 전달 방법

- [ ] RViz2 시각화
  - 온도 데이터를 그래프로 시각화
  - Marker로 상태 표시

---

## 📊 Self-Assessment

| Aspect | Score (1-5) | Notes |
|--------|-------------|-------|
| Concept Understanding | 5/5 | 아키텍처와 설계 원리를 깊이 이해함 |
| Code Quality | 4/5 | 동작하는 코드 완성, 에러 처리는 아직 |
| Problem Solving | 5/5 | WSL 경로 문제를 체계적으로 해결함 |
| Time Management | 4/5 | 경로 문제에 시간 많이 소비, 하지만 학습 가치 있었음 |
| AI Utilization | 5/5 | AI와 효과적으로 협업, 깊은 이해에 집중 |

**Overall Feeling**: 😊 매우 만족

오늘은 단순히 코드를 작성한 것이 아니라, ROS 2의 설계 철학과 아키텍처를 깊이 이해한 날이었습니다. 처음에는 환경 설정과 경로 문제로 좌절할 뻔했지만, 이를 하나씩 해결하면서 시스템에 대한 이해가 깊어졌습니다.

특히 "왜 ROS 2는 이렇게 설계되었는가?"라는 질문에 대한 답을 찾은 것이 가장 큰 수확입니다. 분산 시스템, 마이크로서비스 아키텍처, Publisher-Subscriber 패턴 - 이 모든 것이 복잡한 로봇 시스템을 관리 가능하게 만드는 핵심 원리라는 것을 이해했습니다.

---

## 🧠 Memory Updates

**Added to MEMORY.md**:
- WSL 경로 문제 해결 전략
- 심볼릭 링크 사용법 및 주의사항
- Git 저장소 구성 전략 (통합 vs 분리)
- Claude 도구 사용 시 경로 처리 방법
- 학습 선호도 (큰 그림 중심, 자세한 설명)

---

## 📷 Screenshots/Artifacts

- Terminal 1: temperature_sensor 노드 실행 중
- Terminal 2: `ros2 topic echo /temperature` 실시간 데이터
- `ros2 node list`: /temperature_sensor 확인
- `ros2 topic list`: /temperature, /rosout, /parameter_events 확인

---

## 🎨 Personal Reflections

### What went well today

1. **문제 해결 태도**: 여러 에러에 직면했지만 포기하지 않고 체계적으로 접근했습니다

2. **학습 방향 설정**: 코드 문법보다 아키텍처와 설계 원리에 집중하기로 한 것이 올바른 선택이었습니다. 이것이 장기적으로 더 큰 가치를 만들 것입니다.

3. **AI 협업**: Claude와의 협업 방식을 확립했습니다. AI에게 큰 그림과 원리를 질문하고, 코드 작성은 맡기는 방식이 효율적이었습니다.

4. **포트폴리오 의식**: ros2_ws를 ARGOS 프로젝트로 통합한 것은 미래를 생각한 좋은 결정입니다. 학습 과정 전체가 하나의 스토리가 될 것입니다.

### What could be improved

1. **초기 환경 파악**: 처음부터 WSL 환경과 경로 구조를 명확히 이해했다면 시간을 절약할 수 있었을 것입니다.

2. **단계별 검증**: 각 단계마다 더 세심하게 검증했다면 중복 패키지 같은 실수를 방지할 수 있었을 것입니다.

3. **문서 읽기**: ROS 2 공식 문서를 더 먼저 읽었다면 일부 개념을 더 빨리 이해할 수 있었을 것입니다.

### Energy level throughout the day

- Morning: High - 새로운 시작에 대한 흥분
- Afternoon: Medium - 경로 문제로 약간의 좌절
- Evening: High - 문제 해결 후 성취감, 노드 실행 성공으로 에너지 회복

---

**End of Day 1 Log**

**Next Session**: Week 1, Day 2
**First Task**: Subscriber 노드 작성 (temperature_monitor)
**Goal**: Publisher와 Subscriber가 통신하는 완전한 시스템 완성

---

**Tags**: #ROS2 #Week1 #Day1 #Publisher #Architecture #WSL #FirstNode #Foundation
