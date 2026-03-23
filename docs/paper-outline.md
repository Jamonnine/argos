# ARGOS 논문 아웃라인 (초안)

> **대상**: IROS/ICRA 워크숍 또는 국내 학회 (로봇학회, 소방방재학회)
> **형식**: 6~8페이지, 실험 결과 중심

---

## 제목 (후보)

1. ARGOS: Autonomous Robot Group Orchestration System for Heterogeneous Firefighting Robots
2. Hose-Aware Path Planning for Tethered Firefighting Robot Swarms
3. CBBA-Based Task Allocation for Heterogeneous Firefighting Robot Teams with Physical Constraints

---

## 1. Introduction

- 소방 로봇 현황: 단일 원격 조종 (HR-셰르파 20억/대, 100대 보급 계획)
- 문제: 다수 로봇 동시 운용 불가, 통신 두절 시 정지, 이종 로봇 협업 부재
- 기여:
  1. **이종 군집 오케스트레이션 아키텍처** (UGV+드론+셰르파)
  2. **호스 제약 반영 경로 계획** (세계 최초)
  3. **CBBA 번들 할당 + 드론→UGV 핸드오프**
  4. **합성+실제 혼합 데이터 파인튜닝** (mAP50=0.718)

## 2. Related Work

- DARPA SubT (CERBERUS, CoSTAR): 이종 로봇 탐색, 통신 두절 대응
- HR-셰르파 (현대로템/NFRI): 원격 조종, 단일 운용
- CBBA (Choi et al. 2009): 분산 경매 태스크 할당
- SYN-FIRE (Arlović et al. 2025): 합성 화재 데이터
- MARL 소방 로봇 (Cyborg/Griffith 2026): 99.67% 자율 성공률

## 3. System Architecture

- 4계층: Orchestrator → Mission → Core → Platform
- PlatformInterface 추상화 (UGV/PX4/Sherpa)
- LifecycleNode 패턴 + 8중 센싱

## 4. Hose-Aware Planning (핵심 기여)

- 100m 호스 릴 물리 모델 (잔여 길이, 곡률, 꺾임 위험)
- 5대 규칙: 급커브 금지, 진입 깊이 제한, 역할 분리, 호스 충돌 회피, 충수 후진 금지
- Hose-Aware Planner: Nav2 경로 필터
- 멀티 셰르파 호스 간섭 감지 (CCW 교차 알고리즘)

## 5. CBBA Task Allocation

- 3단계: 비용 행렬 → 경매 → 합의
- 번들 크기 N 확장 (로봇당 다수 임무)
- 드론 FireAlert → CBBA → UGV 진압 + 드론 감시

## 6. Fire Detection with Mixed Data

- SYN-FIRE 2,030장 (합성) + D-Fire 14,122장 (실제) 혼합 (87:13)
- FLAME Diffuser (SD v1.5) 1,000장 증강
- YOLOv8s 파인튜닝: mAP50=0.718 (실데이터 val)
- 도메인 갭 분석: 합성 단독 vs 혼합

## 7. Experiments

### 7.1 Gazebo 시뮬레이션
- Nav2 Goal SUCCEEDED (SLAM 311×217, 0.50m)
- UGVPlatform.move_to() Gazebo 실증 (1.068m)
- PX4 uXRCE-DDS 24토픽 연결
- 셰르파 URDF 스폰 성공

### 7.2 단위 테스트
- 813개 PASS (CBBA 82 + 핸드오프 41 + 편대 32 + E2E 28 + ...)

### 7.3 AI 탐지 성능
| Model | Data | mAP50 | Val Set |
|-------|------|-------|---------|
| YOLOv8s (COCO) | baseline | 0.370 | D-Fire |
| + SYN-FIRE only | 2,030 synthetic | 0.715 | SYN-FIRE |
| + Mixed (87:13) | 16,152 mixed | **0.718** | **D-Fire (real)** |

## 8. Conclusion & Future Work

- 이종 군집 소방 로봇 오케스트레이션 플랫폼 ARGOS 제시
- 호스 제약 경로 계획 세계 최초 구현
- 합성+실제 혼합 데이터로 도메인 갭 해소
- Future: TurtleBot4 실물 테스트, Isaac Sim 마이그레이션, IRIS 2027 R&D

---

## 예상 Figure 목록

1. 4계층 아키텍처 다이어그램
2. 호스 제약 5대 규칙 시각화
3. CBBA 경매 할당 흐름도
4. 드론→UGV 핸드오프 시퀀스
5. mAP50 학습 곡선 (합성 vs 혼합)
6. Gazebo 시뮬레이션 스크린샷
7. 편대 패턴 4종 (횡대/종대/제대/포위)
