# ROS 로봇 개발자 실무 능력 향상 학습 계획

## Context

**목적**: 완전 초보부터 시작하여 ROS를 활용한 로봇 개발자로서의 실무 능력과 문제해결 능력을 키우는 것

**배경**:
- 현재 수준: ROS 완전 초보 (처음 접함)
- 학습 시간: 주 30시간+ (풀타임)
- 관심 분야: 자율주행, 매니퓰레이션, 비전 등 전반적 학습
- 시뮬레이션 전략: Gazebo로 시작 → Isaac Sim으로 전환 (Brev GPU 렌탈)
- 학습 방식: AI 도구 활용 + 프로젝트 중심 + 최신 워크플로우

**차별화 포인트**: 전통적인 튜토리얼 따라하기가 아닌, AI 도구(Claude, GPT)를 적극 활용하여 실전 문제를 해결하면서 학습하는 "AI-Augmented Learning" 방식

---

## 학습 로드맵 (총 10-12주)

### Phase 1: ROS 2 기초 + 개발 환경 구축 (2-3주)

**목표**: ROS 2 핵심 개념 이해 및 AI 기반 개발 워크플로우 확립

**Week 1: 환경 설정 + ROS 2 기본 개념**
- Ubuntu 24.04 설치 (WSL2 또는 듀얼부팅)
- ROS 2 Jazzy 설치 (2026 최신 버전)
- 개발 도구 설정:
  - VS Code + ROS extension
  - Git + GitHub 설정
  - Docker 기본 이해 (재현 가능한 환경 구축)
  - Claude Code CLI 활용 시작

**학습 방법 (AI 활용)**:
- 공식 튜토리얼을 따라하되, 각 개념을 Claude/GPT에게 질문하며 깊이 이해
- 에러가 발생하면 즉시 AI에게 디버깅 요청 (스택 오버플로우 검색 대신)
- 예: "이 ROS 2 노드가 왜 실행되지 않는지 분석해줘" + 에러 로그 붙여넣기

**핵심 개념**:
- Nodes, Topics, Services, Actions
- Publisher/Subscriber 패턴
- ROS 2 패키지 구조
- Python & C++ 노드 작성

**실습 프로젝트 1: "AI와 함께 만드는 첫 ROS 노드"**
- 센서 데이터 수집 노드 작성
- 데이터 처리 노드 작성
- 시각화 (RViz2)
- **AI 활용**: Claude에게 코드 리뷰 요청, 베스트 프랙티스 학습

**Week 2-3: 중급 개념 + Launch Files**
- Parameters & Remapping
- Launch Files (Python 기반)
- TF (Transform) 시스템
- ROS 2 Bag (데이터 기록/재생)
- ROS 2 Control 기초

**실습 프로젝트 2: "다중 노드 시스템 구축"**
- 여러 노드가 협업하는 시스템 설계
- Launch file로 시스템 관리
- **AI 활용**: 아키텍처 설계를 Claude와 함께 논의

---

### Phase 2: Gazebo 시뮬레이션 + 로봇 제어 (3-4주)

**목표**: Gazebo에서 로봇을 시뮬레이션하고 제어하는 실무 능력 확보

**Week 4: Gazebo 기초**
- Gazebo Sim (Ignition) 설치
- URDF/SDF 파일 이해
- ros_gz_bridge 사용법
- Gazebo 플러그인 활용

**학습 방법**:
- 기존 로봇 모델 (TurtleBot, Carter 등) 분석
- URDF를 수정하면서 각 태그의 의미를 AI에게 질문
- "이 URDF에서 joint를 추가하려면 어떻게 해야 해?"

**실습 프로젝트 3: "간단한 모바일 로봇 시뮬레이션"**
- Differential drive 로봇 URDF 작성
- Gazebo에서 시뮬레이션
- 키보드/조이스틱으로 제어
- 센서 데이터 (Lidar, Camera) 수집

**Week 5-6: 자율주행 기초 (Navigation)**
- Nav2 스택 이해
- SLAM (Cartographer 또는 SLAM Toolbox)
- Path Planning & Obstacle Avoidance
- Costmap 설정

**실습 프로젝트 4: "자율주행 로봇 구현"**
- Gazebo 환경에서 맵 생성 (SLAM)
- Nav2로 자율주행 구현
- 동적 장애물 회피
- **AI 활용**: "Nav2 파라미터 튜닝을 어떻게 해야 더 부드러운 경로를 얻을 수 있어?"

**Week 7: 매니퓰레이션 기초**
- MoveIt 2 설치 및 개념
- 역기구학(Inverse Kinematics)
- Motion Planning
- 그리퍼 제어

**실습 프로젝트 5: "로봇 팔 시뮬레이션"**
- Panda 또는 UR5 로봇 팔 Gazebo 시뮬레이션
- MoveIt 2로 경로 계획
- Pick & Place 작업
- **AI 활용**: "이 충돌 에러를 어떻게 해결할 수 있을까?" + 로그 공유

---

### Phase 3: 컴퓨터 비전 + 통합 프로젝트 (3-4주)

**목표**: 비전 기반 로봇 시스템 구축 및 통합 능력 향상

**Week 8: Computer Vision with ROS 2**
- OpenCV + ROS 2 통합
- cv_bridge 사용
- 객체 인식 (YOLO, Detectron2)
- 3D 인식 (Point Cloud Processing)

**실습 프로젝트 6: "비전 기반 객체 인식"**
- Gazebo 카메라 데이터 수집
- YOLO로 객체 인식
- 인식된 객체로 로봇 제어

**Week 9-10: 통합 프로젝트**
**대형 프로젝트: "창고 자동화 로봇 시스템"** (AI와 함께 설계)
- 요구사항 정의 (Claude와 함께 브레인스토밍)
- 시스템 아키텍처 설계
- 구현:
  - 자율주행으로 목표 지점 이동
  - 객체 인식으로 물체 찾기
  - 로봇 팔로 Pick & Place
  - 복귀 및 보고
- **AI 활용의 핵심**:
  - "이 시스템을 어떻게 모듈화해야 유지보수가 쉬울까?"
  - "상태 머신(State Machine)을 어떻게 설계할까?"
  - 디버깅 시 AI에게 전체 시스템 로그 분석 요청

**Week 11: 코드 품질 & 테스트**
- Unit Testing (pytest, gtest)
- Integration Testing
- CI/CD 기초 (GitHub Actions)
- 코드 리팩토링
- **AI 활용**: "이 코드를 더 Pythonic하게 만들려면?" / "테스트 케이스를 추가해줘"

---

### Phase 4: Isaac Sim 전환 + 고급 시뮬레이션 (2-3주)

**목표**: 고성능 GPU 시뮬레이션 환경에서 실전 프로젝트 수행

**Week 12: Isaac Sim 환경 구축**
- Brev에서 GPU 인스턴스 렌탈
- Isaac Sim 설치 (Omniverse)
- ROS 2 Bridge 설정 (Jazzy 권장)
- Isaac ROS 패키지 탐색

**Week 13: Isaac Sim 고급 기능**
- Synthetic Data Generation (합성 데이터 생성)
- Domain Randomization (다양한 환경 생성)
- Isaac ROS DNN Stereo Disparity
- Isaac ROS Visual SLAM

**최종 프로젝트: "Gazebo 프로젝트를 Isaac Sim으로 마이그레이션"**
- Phase 3의 창고 자동화 로봇을 Isaac Sim으로 이식
- 성능 비교 (렌더링 속도, 물리 시뮬레이션 정확도)
- Isaac Sim 전용 기능 활용 (Synthetic Data로 비전 모델 학습)
- **AI 활용**: "Gazebo URDF를 Isaac Sim USD로 변환하는 방법"

---

## AI 시대 학습 방법론

### 1. AI-Pair Programming 워크플로우
- **코드 작성 전**: Claude에게 설계 리뷰 요청
  - "이 기능을 구현하려면 어떤 ROS 패키지를 사용해야 할까?"
  - "이 아키텍처의 장단점은?"
- **코드 작성 중**: Claude Code CLI로 실시간 코드 생성/수정
  - 반복 작업 자동화 (boilerplate 코드)
  - 에러 즉시 해결
- **코드 작성 후**: 코드 리뷰 및 개선
  - "이 코드의 성능을 개선하려면?"
  - "ROS 2 베스트 프랙티스를 따르고 있나?"

### 2. 문제 해결 프로세스 (AI 기반)
1. **문제 발생**: 에러 로그 수집
2. **AI에게 질문**: 전체 컨텍스트 + 에러 로그 제공
3. **해결책 실행**: AI가 제안한 방법 시도
4. **학습**: "왜 이 방법이 작동하는지" 추가 질문으로 깊이 이해
5. **문서화**: 해결 과정을 `MEMORY.md`에 기록 (재사용 가능)

### 3. 프로젝트 중심 학습
- 튜토리얼만 따라하지 말고, 각 개념을 배우자마자 작은 프로젝트에 적용
- "이론 30%, 실습 70%" 비율 유지
- 매주 1개 이상의 동작하는 프로젝트 완성

### 4. 커뮤니티 & 오픈소스 활용
- GitHub에서 우수한 ROS 프로젝트 분석
- AI에게 "이 프로젝트의 코드 구조를 분석해줘" 요청
- 작은 기여부터 시작 (문서화, 버그 수정)

### 5. 매일 학습 루틴 (풀타임 기준)
- **오전 (3-4시간)**: 새로운 개념 학습
  - 공식 문서 읽기
  - 튜토리얼 따라하기
  - AI에게 개념 질문
- **오후 (3-4시간)**: 프로젝트 작업
  - 배운 내용을 프로젝트에 적용
  - AI와 페어 프로그래밍
- **저녁 (1-2시간)**: 복습 및 정리
  - 오늘 배운 내용 MEMORY.md에 기록
  - 내일 계획 수립
  - 커뮤니티 참여 (Reddit, ROS Discourse)

---

## 개발 환경 & 도구 스택

### 필수 도구
- **OS**: Ubuntu 24.04 LTS (WSL2 또는 네이티브)
- **ROS**: ROS 2 Jazzy (2026 최신)
- **시뮬레이터**:
  - Gazebo Sim (Ignition) - Phase 1-3
  - Isaac Sim (Omniverse) - Phase 4
- **IDE**: VS Code + Extensions
  - ROS extension
  - Python extension
  - C++ extension
  - GitHub Copilot (선택)
- **버전 관리**: Git + GitHub
- **컨테이너**: Docker (환경 격리 및 재현성)
- **AI 도구**:
  - Claude Code CLI (주력 도구)
  - ChatGPT / Claude Web (개념 학습)
  - GitHub Copilot (코드 자동완성, 선택)

### 추천 ROS 패키지
- **Navigation**: Nav2
- **SLAM**: SLAM Toolbox, Cartographer
- **Manipulation**: MoveIt 2
- **Vision**: cv_bridge, OpenCV, Isaac ROS
- **Control**: ros2_control
- **Testing**: pytest, gtest

---

## 학습 리소스

### 공식 문서
- [ROS 2 Documentation](https://docs.ros.org/en/rolling/Tutorials.html)
- [Gazebo Documentation](https://gazebosim.org/docs/latest/ros2_integration/)
- [Isaac Sim Documentation](https://docs.isaacsim.omniverse.nvidia.com/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [MoveIt 2 Documentation](https://moveit.picknik.ai/)

### 온라인 코스 (선택)
- [The Construct - ROS 2 for Beginners](https://www.theconstruct.ai/robotigniteacademy_learnros/ros-courses-library/ros2-basics-course/)
- [Udemy - ROS 2 for Beginners](https://www.udemy.com/course/ros2-for-beginners/)

### 커뮤니티
- ROS Discourse: https://discourse.ros.org/
- Reddit: r/ROS
- GitHub: ROS 2 repositories

---

## 검증 및 마일스톤

### Phase 1 완료 기준
- [ ] 간단한 Publisher/Subscriber 노드를 처음부터 작성 가능
- [ ] Launch file로 다중 노드 시스템 실행 가능
- [ ] RViz2로 데이터 시각화 가능
- [ ] AI 도구를 활용하여 에러를 독립적으로 해결한 경험 3회 이상

### Phase 2 완료 기준
- [ ] Gazebo에서 커스텀 로봇 시뮬레이션 가능
- [ ] Nav2로 자율주행 구현 및 맵 생성
- [ ] MoveIt 2로 로봇 팔 제어
- [ ] URDF 파일을 직접 작성/수정 가능

### Phase 3 완료 기준
- [ ] 카메라 데이터로 객체 인식 구현
- [ ] 통합 프로젝트 (창고 자동화) 완성
- [ ] Git으로 프로젝트 관리 및 GitHub에 공개
- [ ] 코드에 테스트 코드 포함

### Phase 4 완료 기준
- [ ] Isaac Sim에서 로봇 시뮬레이션 실행
- [ ] Gazebo 프로젝트를 Isaac Sim으로 마이그레이션
- [ ] Synthetic Data 생성 및 활용
- [ ] Brev 환경에서 원격 개발 가능

---

## 예상 타임라인

| 주차 | Phase | 주요 내용 | 결과물 |
|------|-------|-----------|--------|
| 1-3 | Phase 1 | ROS 2 기초 | 2개의 작은 프로젝트 |
| 4-7 | Phase 2 | Gazebo + Navigation + Manipulation | 3개의 시뮬레이션 프로젝트 |
| 8-11 | Phase 3 | Vision + 통합 프로젝트 | 창고 자동화 로봇 (포트폴리오) |
| 12-13 | Phase 4 | Isaac Sim 전환 | Isaac Sim 최종 프로젝트 |

**총 기간**: 12-13주 (3개월)
**풀타임 학습 시 기대 결과**: 실무 투입 가능한 ROS 로봇 개발자 수준
