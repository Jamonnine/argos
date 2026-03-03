# Week 1: 환경 설정 + ROS 2 기본 개념

**Duration**: 5-7 days (풀타임 학습 기준)
**Phase**: Phase 1 - ROS 2 Fundamentals
**Goal**: ROS 2 개발 환경을 완벽하게 구축하고 핵심 개념(Nodes, Topics, Pub/Sub)을 실전에서 사용할 수 있는 수준까지 학습

---

## 📅 Day-by-Day Breakdown

### Day 1: 개발 환경 구축 🛠️

**시간 배분**: 6-8시간

#### Morning (3-4h): 시스템 설치
- [ ] Ubuntu 24.04 설치 확인 (WSL2 또는 네이티브)
- [ ] `setup-ros2-jazzy.sh` 스크립트 실행
- [ ] VS Code 설치 + Extensions 설정
  - ROS extension
  - Python extension
  - C++ extension (optional)
- [ ] Git 설정
  ```bash
  git config --global user.name "Your Name"
  git config --global user.email "your.email@example.com"
  ```
- [ ] GitHub 계정 연동 (SSH key 생성)

#### Afternoon (3-4h): 환경 검증 및 첫 실행
- [ ] ROS 2 설치 확인
  ```bash
  ros2 --version
  ros2 run demo_nodes_cpp talker
  ```
- [ ] Turtlesim 실행 (첫 ROS 경험!)
  ```bash
  # Terminal 1
  ros2 run turtlesim turtlesim_node

  # Terminal 2
  ros2 run turtlesim turtle_teleop_key
  ```
- [ ] ROS 2 CLI 도구 탐색
  ```bash
  ros2 topic list
  ros2 topic echo /turtle1/pose
  ros2 node list
  ros2 node info /turtlesim
  ```

#### Evening (1-2h): AI 도구 설정 및 개념 학습
- [ ] Claude Code CLI 사용법 익히기
  ```bash
  claude "Explain ROS 2 nodes in simple terms"
  ```
- [ ] AI에게 질문하며 개념 이해
  - "What is a ROS 2 node?"
  - "What's the difference between a topic and a service?"
  - "Why do we need publishers and subscribers?"

**학습 목표**:
- ✅ ROS 2 명령어 없이 실행 가능
- ✅ Turtlesim으로 Topic/Node 개념 직관적 이해
- ✅ AI 도구를 편하게 사용할 수 있는 워크플로우 확립

**일일 기록**: `daily-log-day01.md` 작성

---

### Day 2: Nodes & Topics 이해 📡

**시간 배분**: 6-8시간

#### Morning (3-4h): 공식 튜토리얼 + AI 학습
- [ ] [공식 튜토리얼: Understanding Nodes](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html) 완료
- [ ] [공식 튜토리얼: Understanding Topics](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html) 완료
- [ ] AI와 개념 심화
  - "What happens if a publisher starts before a subscriber?"
  - "Can one node have multiple publishers?"
  - "What is QoS (Quality of Service) in ROS 2?"

#### Afternoon (3-4h): 실습 - 첫 Python 노드 작성
- [ ] 워크스페이스 생성
  ```bash
  mkdir -p ~/ros2_ws/src
  cd ~/ros2_ws/src
  ros2 pkg create --build-type ament_python my_first_package
  ```
- [ ] Publisher 노드 작성 (`simple_publisher.py`)
  - String 메시지를 /chatter 토픽에 발행
  - 1Hz 주기
- [ ] Subscriber 노드 작성 (`simple_subscriber.py`)
  - /chatter 토픽 구독
  - 받은 메시지 출력

#### Evening (1-2h): 빌드, 실행, 디버깅
- [ ] 패키지 빌드
  ```bash
  cd ~/ros2_ws
  colcon build --packages-select my_first_package
  source install/setup.bash
  ```
- [ ] 노드 실행 및 검증
- [ ] 에러 발생 시 AI와 함께 디버깅
- [ ] MEMORY.md 업데이트: "Nodes와 Topics 핵심 요약"

**학습 목표**:
- ✅ Python으로 기본 Publisher/Subscriber 작성 가능
- ✅ 패키지 생성, 빌드, 실행 워크플로우 이해
- ✅ 에러 로그 읽고 AI와 함께 해결하는 경험

**일일 기록**: `daily-log-day02.md` 작성

---

### Day 3: 메시지 타입 & 커스텀 메시지 📦

**시간 배분**: 6-8시간

#### Morning (3-4h): 표준 메시지 타입 학습
- [ ] [공식 튜토리얼: Understanding Parameters](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html)
- [ ] 다양한 메시지 타입 탐색
  ```bash
  ros2 interface list
  ros2 interface show std_msgs/msg/String
  ros2 interface show geometry_msgs/msg/Twist
  ros2 interface show sensor_msgs/msg/LaserScan
  ```
- [ ] AI와 학습
  - "When would I use Twist vs Pose?"
  - "What's the structure of LaserScan message?"

#### Afternoon (3-4h): 실습 - 센서 데이터 Publisher
- [ ] 새 노드 작성: `sensor_publisher.py`
  - 가상의 센서 데이터 생성 (예: 온도, 거리)
  - `sensor_msgs/Temperature` 또는 `sensor_msgs/Range` 사용
  - 주기적으로 랜덤 데이터 발행
- [ ] Subscriber 작성: `sensor_monitor.py`
  - 센서 데이터 수신
  - 임계값 초과 시 경고 출력

#### Evening (1-2h): 커스텀 메시지 타입 (선택)
- [ ] [공식 튜토리얼: Creating Custom Interfaces](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)
- [ ] 간단한 커스텀 메시지 정의 (예: SensorData.msg)
- [ ] 커스텀 메시지 사용하는 Publisher/Subscriber 작성

**학습 목표**:
- ✅ 다양한 메시지 타입 이해 및 사용
- ✅ 실제 센서 데이터 시뮬레이션 경험
- ✅ (선택) 커스텀 메시지 정의 및 사용

**일일 기록**: `daily-log-day03.md` 작성

---

### Day 4: Services & Parameters 🔧

**시간 배분**: 6-8시간

#### Morning (3-4h): Services 개념 및 실습
- [ ] [공식 튜토리얼: Understanding Services](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)
- [ ] [공식 튜토리얼: Writing a Simple Service and Client (Python)](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)
- [ ] AI와 학습
  - "When should I use a service vs a topic?"
  - "What are synchronous vs asynchronous service calls?"

#### Afternoon (3-4h): 실습 - Calculator Service
- [ ] Service Server 작성: `calculator_server.py`
  - 두 숫자를 받아 더하는 서비스
  - `example_interfaces/srv/AddTwoInts` 사용
- [ ] Service Client 작성: `calculator_client.py`
  - 서비스 호출 및 결과 출력
- [ ] 실행 및 테스트
  ```bash
  ros2 run my_first_package calculator_server
  ros2 run my_first_package calculator_client
  ```

#### Evening (1-2h): Parameters 실습
- [ ] Parameter 추가한 노드 작성
  - 예: Publisher의 주기를 parameter로 설정
- [ ] 런타임에 parameter 변경
  ```bash
  ros2 param list
  ros2 param get /node_name param_name
  ros2 param set /node_name param_name value
  ```

**학습 목표**:
- ✅ Service의 request-response 패턴 이해
- ✅ Service Server/Client 작성 가능
- ✅ Parameter를 통한 노드 설정 이해

**일일 기록**: `daily-log-day04.md` 작성

---

### Day 5: Launch Files & Multi-Node Systems 🚀

**시간 배분**: 6-8시간

#### Morning (3-4h): Launch Files 학습
- [ ] [공식 튜토리얼: Creating a Launch File](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)
- [ ] AI와 학습
  - "Why use launch files instead of running nodes manually?"
  - "What are the advantages of Python launch files over XML?"

#### Afternoon (3-4h): 실습 - Multi-Node System
- [ ] Launch file 작성: `sensor_system.launch.py`
  - 여러 노드를 동시에 실행
  - Parameter 전달
  - Remapping 사용
- [ ] 예: 센서 데이터 파이프라인
  - sensor_publisher → data_processor → monitor
  - 3개 노드를 launch file로 관리

#### Evening (1-2h): RViz2로 시각화
- [ ] RViz2 기본 사용법 학습
- [ ] 간단한 마커 발행 노드 작성
- [ ] RViz2에서 시각화

**학습 목표**:
- ✅ Python launch file 작성 가능
- ✅ 다중 노드 시스템을 launch file로 관리
- ✅ RViz2 기본 사용법 이해

**일일 기록**: `daily-log-day05.md` 작성

---

### Day 6-7: Week 1 프로젝트 - "AI와 함께 만드는 첫 ROS 노드" 🎯

**목표**: 이번 주 학습한 모든 내용을 통합하는 프로젝트 완성

#### 프로젝트 요구사항
1. **센서 데이터 수집 노드** (`sensor_node.py`)
   - 가상의 환경 센서 시뮬레이션 (온도, 습도, CO2 레벨)
   - 각각 다른 토픽에 발행 (1Hz)

2. **데이터 처리 노드** (`processor_node.py`)
   - 센서 데이터 구독
   - 평균, 최대/최소값 계산
   - 처리된 데이터 발행

3. **경고 노드** (`alert_node.py`)
   - 처리된 데이터 구독
   - 임계값 초과 시 경고 메시지 발행

4. **제어 노드** (`control_node.py`)
   - 경고 메시지 구독
   - Service로 시스템 리셋 제공

5. **시각화 노드** (`visualizer_node.py`)
   - 데이터를 RViz2용 마커로 변환

6. **Launch File** (`week1_project.launch.py`)
   - 모든 노드 실행
   - Parameter 설정

#### AI 활용 체크리스트
- [ ] 프로젝트 아키텍처를 Claude와 함께 설계
- [ ] 각 노드의 역할과 토픽 구조 논의
- [ ] 코드 작성 중 막히면 즉시 AI에게 질문
- [ ] 완성 후 코드 리뷰 요청
- [ ] 개선 사항 적용

#### 제출물
- [ ] GitHub repository 생성 (`ros2-week1-project`)
- [ ] README.md 작성 (프로젝트 설명, 실행 방법)
- [ ] 코드에 주석 추가 (설명 포함)
- [ ] 스크린샷/동영상 (RViz2 시각화)

**학습 목표**:
- ✅ 여러 노드가 협업하는 시스템 설계 및 구현
- ✅ AI와 페어 프로그래밍 경험
- ✅ Git/GitHub로 프로젝트 관리
- ✅ 포트폴리오 첫 작품 완성

**일일 기록**: `daily-log-day06.md`, `daily-log-day07.md` 작성

---

## ✅ Week 1 완료 체크리스트

### 환경 설정
- [ ] Ubuntu 24.04 + ROS 2 Jazzy 설치 완료
- [ ] VS Code + Extensions 설정 완료
- [ ] Git/GitHub 설정 완료
- [ ] Claude Code CLI 사용 가능

### 기술 역량
- [ ] ROS 2 CLI 도구 사용 가능 (topic, node, service, param)
- [ ] Python으로 Publisher/Subscriber 노드 작성 가능
- [ ] Service Server/Client 작성 가능
- [ ] Launch file 작성 가능
- [ ] RViz2로 데이터 시각화 가능

### AI 활용
- [ ] AI에게 개념 질문하며 깊이 이해한 경험 10회 이상
- [ ] AI와 함께 에러 해결한 경험 3회 이상
- [ ] AI에게 코드 리뷰 받은 경험 2회 이상

### 프로젝트
- [ ] Week 1 통합 프로젝트 완성
- [ ] GitHub에 코드 업로드
- [ ] README 작성

---

## 📚 추천 학습 리소스

### 공식 문서
- [ROS 2 Jazzy Tutorials](https://docs.ros.org/en/jazzy/Tutorials.html)
- [ROS 2 Concepts](https://docs.ros.org/en/jazzy/Concepts.html)

### 추가 학습 (선택)
- [ROS 2 Design](https://design.ros2.org/)
- [ROS 2 Command Line Tools Cheat Sheet](https://github.com/ros2/ros2/wiki/Command-Line-Tools)

---

## 🔜 Next: Week 2 Preview

**주제**: ROS 2 중급 개념 (TF, Parameters, ROS Bags, Actions)
**프로젝트**: 다중 노드 시스템 구축 (좌표 변환 포함)

**준비사항**:
- Week 1 프로젝트 복습
- TF (Transform) 개념 미리 읽어보기
- ROS 2 Actions 공식 문서 훑어보기

---

**마지막 업데이트**: 2026-02-08
**예상 완료일**: Week 1 종료 후
