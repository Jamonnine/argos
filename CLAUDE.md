# ARGOS - 이종 군집 소방 로봇 플랫폼

## 프로젝트 컨텍스트
- **비전**: 드론 + UGV + 보행형 등 이종 로봇이 한 팀으로 소방 현장을 탐색·진압 지원하는 오케스트레이션 플랫폼
- **개발자**: 민발 (대구 강북소방서 소방관 / 1인 개발자)
- **환경**: WSL2 Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic
- **라이선스**: Apache 2.0 (공공기관·수상·상업 사용 가능)
- **부모 프로젝트**: SUPERAGENT (`C:\Users\USER\Desktop\SUPERAGENT\projects\argos\`)

## 4계층 아키텍처
1. **Orchestrator** — 임무 할당, 상황 종합, 전략 판단
2. **Mission Modules** — 정찰 / 진압보조 / 구조지원 / 위험물 (교체식)
3. **Core Services** — 위치추정·경로계획·군집통신·상태관리·상황공유
4. **Platform** — Drone / UGV / Legged 각각 다른 구현, 동일 인터페이스 상향

## 개발 방식
- **개발 실행 모드**: AI가 코드 작성·빌드·테스트. 긴 설명 대신 개발 중 인사이트 한두마디
- **오픈소스 적극 활용**: 바퀴 재발명 금지. 검증된 오픈소스로 시간 단축
- **리서치 선행**: 새 기능 개발 전 관련 오픈소스·논문 자동 조사
- **소방관 기준**: "현장에서 이게 진짜 쓸모 있나?" 판단 유지
- **진도 관리**: `memory/argos-tutor.md` (SUPERAGENT 메모리 디렉토리)
- **로드맵**: `ROADMAP.md` — 정책 정합성, 외부 생태계, 전략 타임라인, 참조 연구 통합

## 프로젝트 구조
```
argos/
├── CLAUDE.md              ← 이 파일
├── ros2_ws/src/
│   ├── argos_description/ ← URDF/xacro (3계층 모듈식)
│   ├── argos_bringup/     ← 노드, launch, 스크립트
│   └── argos_interfaces/  ← .msg/.srv/.action
├── knowledge/projects/research/ ← 리서치 결과
├── learning-log/          ← Day 1-19 학습 기록 (아카이브)
├── scripts/               ← 유틸리티 (진단, Gazebo, SLAM, Nav2)
└── docs/                  ← 가이드 문서
```

## 심볼릭 링크
- `~/ros2_ws` → `/mnt/c/Users/USER/Desktop/SUPERAGENT/projects/argos/ros2_ws`
- 터미널: `~/ros2_ws` 사용 / AI 도구: Windows 경로 사용

## 세션 종료 시 필수
```bash
wsl --shutdown   # WSL 메모리 반환 (VmmemWSL 프로세스 해제)
```
