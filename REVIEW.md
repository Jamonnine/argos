# Code Review Guidelines — ARGOS

> 이종 군집 소방 로봇 시스템. ROS 2 Jazzy + Gazebo Sim. 안전 최우선.

## Critical
- **로봇 안전**: 속도/토크 제한 없는 명령, 비상정지 미구현
- **토픽 타입 불일치**: 퍼블리셔-서브스크라이버 간 메시지 타입 불일치
- **QoS 불일치**: 퍼블리셔-서브스크라이버 QoS 프로파일 불일치 (통신 실패)
- **데드락**: 콜백 내부에서 동기 서비스 호출 (단일 스레드 executor 데드락)
- **메모리 해제**: ROS 노드 소멸자에서 리소스 미정리

## High
- **하드코딩 파라미터**: 로봇 속도·PID 게인 등이 코드에 직접 입력 (파라미터 서버 사용 필수)
- **에러 핸들링**: 센서 데이터 None/NaN 체크 없이 연산
- **타이머 주기**: 제어 루프 주기가 불안정하거나 너무 느림
- **네임스페이스 충돌**: 멀티 로봇 환경에서 토픽/서비스 이름 충돌
- **launch 파일**: 파라미터 기본값 누락, 필수 노드 미포함

## Medium
- **미사용 의존성**: package.xml에 선언되었지만 실제 미사용
- **로깅**: get_logger 미사용 (print 사용 시 ROS 로깅 체계 이탈)
- **코드 구조**: 단일 노드에 과다 책임 (SRP 위반)
- **테스트**: launch_test 미작성

## Skip
- Gazebo 모델 파일 (.sdf, .urdf 생성 코드 제외)
- meshes/, textures/ 리소스 파일
- 빌드 산출물 (build/, install/, log/)

## 프로젝트 특수 규칙
- **TwistStamped**: Twist 대신 TwistStamped 사용 (Jazzy 표준)
- **라이선스**: Apache 2.0, 사용 오픈소스 LICENSE 파일 명시
- **시뮬레이션 우선**: 실기체 배포 전 Gazebo 시뮬 검증 필수
