# ARGOS 로드맵 업데이트 브리프 (2026-03-20)

> 이 문서는 2026-03-20 리서치 세션에서 수집된 정보를 ARGOS 개발 세션에 전달하기 위한 종합 브리프입니다.
> 로드맵(ROADMAP.md) 수정 시 이 문서를 근거로 사용하세요.

---

## 1. 외부 환경 변화 (로드맵에 반영 필요)

### 1-1. GTC 2026 발표 (3/17~19) — Gazebo→Isaac 전환 장벽 대폭 하락

| 발표 | ARGOS 영향 | 우선순위 |
|------|-----------|---------|
| **Isaac Sim 6.0** — ROS2 Jazzy 네이티브 + Newton 물리 엔진 | Gazebo 마이그레이션 장벽 크게 낮아짐. ARGOS ROS2 노드 거의 그대로 이식 가능 | ★★★ |
| **Isaac Lab 3.0** — DGX 스케일 RL, Warp Core API | MARL 훈련 환경 대폭 개선. marl_env.py → DirectRLEnv 전환 시 수백~수천 환경 병렬 가능 | ★★ |
| **Cosmos 3** — Predict+Reason+Transfer 통합 WFM | Gazebo 영상→포토리얼 변환 자동화. 화재 데이터 부족 문제 해결 경로 | ★★★ |
| **Cosmos Transfer2.5-2B** — 12GB VRAM 경량 모델 | RTX 4050(6GB)에서도 배치 축소 시 실험 가능. **"GPU 없어서 못 한다" 전제 수정** | ★★★ |
| **Physical AI Data Factory Blueprint** — 4월 GitHub 공개 예정 | Curator+Transfer+Evaluator+OSMO 4모듈 오픈소스. 합성 데이터 파이프라인 표준 | ★★ |
| **Feynman 아키텍처** (2028) | 단기 무관. 2029년 이후 온보드 컴퓨팅 로드맵 기준점 | ★ |

**상세 리서치**: `knowledge/projects/research/2026-03-20-gtc-2026-summary-for-argos.md`

### 1-2. 소방청 차세대 AI·클라우드 119 체계 — 2,300억 사업 착수

- **사업 규모**: 총 2,300억원, 전국 19개 시·도 119 시스템 단일 클라우드 통합
- **현재 단계**: ISMP(정보화전략계획) 수립 중 (KT 컨소시엄, ~2026년 9월 완료)
- **본사업 발주**: 2026 하반기~2027년 예상
- **핵심 설계 항목**: "현장 대원 활동을 돕는 **현장 대응 기반 설계**" 명시
- **ARGOS 정합성**: 소방청 10대 전략과제 4번(AI 현장지휘) + 5번(이종 무인장비)과 정확 일치

**의미**: ARGOS가 만드는 것(이종 로봇 오케스트레이션)이 소방청이 제도적으로 추진하는 것과 같은 방향. 단독 프로젝트가 아닌 국가 정책 정합 프로젝트로 포지셔닝 가능.

**무인소방로봇 병행 추진**: HR-셰르파 2027~2030년 36대→100대 보급. ARGOS = 이들을 통합 지휘하는 상위 레이어.

**상세 리서치**: `knowledge/projects/research/2026-03-20-fire-agency-ai-119-system.md`

### 1-3. NFRI 방침 변경 — 적극 추진 → 수동 대기

- 이메일 발송 완료 (3/15), 응답 미수신 (D+5)
- **결정 (3/20)**: NFRI에서 먼저 연락하지 않는 한 적극 추진 중단
- IRIS R&D, 119리빙랩, 국민안전발명챌린지 등 공식 채널은 NFRI와 무관하게 독립 유지
- 로드맵에서 "NFRI 공동연구" 의존 항목은 독립 경로로 전환 필요

---

## 2. 합성 데이터 파이프라인 — 핵심 리서치 결과

### 2-1. 효과는 검증됨 (R1 결과)

| 연구 | 합성 방법 | 성능 향상 | 비고 |
|------|---------|---------|------|
| SYN-FIRE (2025) | NVIDIA Omniverse | DiceScore +2~16% | 실내 화재, 데이터 공개 |
| UE5 산림화재 (2024) | Unreal Engine 5 | mAP +4.6% | 산불 탐지 |
| 소방관 탐지 (2025) | CLAHE + 스톡 합성 | mAP +8% | 연기 속 소방관 |
| Cosmos Transfer1 (2025) | Gazebo→포토리얼 | 성공률 +68.5% | 로봇 내비게이션 |

### 2-2. 주의사항 (R1, R4 결과)

- **합성 단독 학습은 위험**: arXiv 2024에서 합성만 학습 시 OOD 성능 최하위 확인
- **실제:합성 = 8:2 이상 비율 필수** + BTL 전략(합성 선학습→실제 파인튜닝)
- **Gazebo 영상 직접 사용 금지**: RoboCup 2024에서 Gazebo가 시각 성능 "크게 과대평가" 확인
- ARGOS는 이미 mAP50=0.953이므로, 수치 향상보다 **미탐지 엣지케이스 recall 향상**이 목표

### 2-3. Cosmos Curator는 AI-Hub에 적용 불가 (R2 결과)

- **Cosmos Curator(cosmos-curate)는 비디오 전용**. AI-Hub 238만 장 이미지에 적용 불가능
- NeMo Curator는 이미지 지원하나 bbox 정제 기능 없음 + WebDataset 포맷 불일치
- **대안**: CleanVision + Cleanlab (0원, CPU 동작, 기존 best.pt 활용)

### 2-4. RTX 4050(6GB)으로 즉시 실행 가능한 경로 (R4 결과)

```
Phase E 신규 작업으로 추가 권장:

[즉시 가능 — 비용 0원, RTX 4050]
1. SYN-FIRE 데이터셋 다운로드 (FigShare, 2,030장 실내 화재)
2. Gazebo fire_building.sdf에 세그멘테이션 카메라 추가 (자동 어노테이션)
3. D-Fire 21,527장 + SYN-FIRE 혼합
4. FLAME Diffuser (SD v1.5, 6GB OK)로 조명·연기 조건 다양화
5. AI-Hub best.pt 파인튜닝 → mAP50 0.953→0.96+ 목표

[Cosmos Transfer — Phase F 이후, 클라우드 GPU 필요]
6. Cosmos Transfer2.5-2B (12GB) 또는 API 테스트
7. Gazebo 화재 영상→포토리얼 변환 파이프라인
8. 포토리얼 5,000장 목표
```

---

## 3. GPU 접근 전략 (R3 결과)

| 순위 | 방법 | GPU | 비용 | 실현 조건 |
|------|------|-----|------|----------|
| 1 | **Kaggle** | P100 16GB, 30h/주 | 0원 | 전화번호 인증만 (5분) |
| 2 | **NVIDIA NIM API** | 클라우드 추론 | 0원 (무료 한도) | build.nvidia.com 키 발급 |
| 3 | **KISTI 슈퍼컴 6호기** | A100 80GB | 0원 | 연구 계획서 제출, 2026년 6월 이후 |
| 4 | **Google Colab** | T4 16GB | 0원 | 12h 세션 제한 |
| 5 | **HuggingFace ZeroGPU** | H200 | 0원 | Spaces 앱으로만 사용 가능 |

**전제 수정**: Cosmos Transfer2.5-2B(12GB)가 존재하므로, 반드시 24GB+ GPU가 아니어도 실험 시작 가능.

**상세 리서치**: `knowledge/projects/research/2026-03-20-free-gpu-access-programs-for-argos.md`

---

## 4. 로드맵 수정 권고사항

### 4-1. Phase E (지능화) 신규 세션 추가 권고

| 세션 | 내용 | 의존 | 비용 |
|------|------|------|------|
| **S-E-SYN-1** | SYN-FIRE + D-Fire 혼합 학습 실험 | 없음 | 0원 |
| **S-E-SYN-2** | fire_building.sdf 세그멘테이션 카메라 추가 | 없음 | 0원 |
| **S-E-SYN-3** | 혼합 데이터셋 파인튜닝 + mAP 측정 | S-E-SYN-1,2 | 0원 |
| **S-E-SYN-4** | FLAME Diffuser 환경 구성 (SD v1.5) | 없음 | 0원 |
| **S-E-SYN-5** | 합성 vs 실제 도메인 갭 정량 평가 | S-E-SYN-3,4 | 0원 |

### 4-2. Phase F (실세계 전환) 수정 권고

| 항목 | 기존 | 수정 |
|------|------|------|
| Isaac Lab 전환 | "검토" | Isaac Sim 6.0 ROS2 Jazzy 지원으로 **구체적 마이그레이션 경로 확보**. Phase D 완료 후 착수 |
| Cosmos Transfer | 미포함 | Gazebo 영상→포토리얼 변환 파이프라인 추가 (Transfer2.5-2B부터) |
| NFRI 공동연구 | Phase 3 핵심 | **독립 경로로 전환** (119리빙랩, IRIS R&D, 발명챌린지) |
| GPU 전략 | 미포함 | Kaggle P100 즉시 + KISTI 6호기(6월~) |

### 4-3. 정책 정합성 섹션 갱신 권고

추가 항목:
- **소방청 차세대 AI 119 (2,300억)**: ISMP 진행 중, 본사업 2027. "현장 대응 기반 설계" 명시 → ARGOS 포지셔닝 근거
- **무인소방로봇 100대 보급**: 2027~2030, HR-셰르파 기반 → ARGOS = 통합 지휘 상위 레이어
- **소방청 10대 전략과제 4·5번**: 이미 기록됨, 차세대 119와 연계하여 강화

---

## 5. 리서치 Phase 2~3 계획 (미래 참조)

리서치 계획서: `C:\Users\USER\.claude\plans\snug-plotting-key.md`

| Phase | 트리거 | 리서치 |
|-------|--------|--------|
| **Phase 2** | 4월 NVIDIA GitHub Blueprint 공개 | R5(API 상세), R6(Gazebo 호환성 실증), R7(OSMO 비용), R10(프롬프트), R11(세그 우회) |
| **Phase 3** | Phase E 착수 시 | R8(Isaac Lab 전환 경로), R9(이종 MARL 구축) |

---

## 6. 참고 자료 (오늘 생성된 리서치 파일 7건)

| 파일 | 핵심 내용 |
|------|----------|
| `2026-03-20-gtc-2026-summary-for-argos.md` | GTC 2026 7개 영역 ARGOS 맞춤 요약 |
| `2026-03-20-fire-agency-ai-119-system.md` | 소방청 차세대 AI 119 체계 (2,300억) 상세 |
| `2026-03-20-nvidia-blueprint-argos-application.md` | Blueprint 4모듈 × ARGOS 매핑 |
| `2026-03-20-cosmos-curator-ai-hub-applicability.md` | Curator=비디오전용, CleanVision 대안 |
| `2026-03-20-synthetic-fire-data-performance-survey.md` | 합성 데이터 효과 논문 비교표 |
| `2026-03-20-free-gpu-access-programs-for-argos.md` | 무료 GPU 프로그램 비교표 |
| `2026-03-20-fire-synthetic-data-deep-dive.md` | 핵심 논문 재현성 평가 + 즉시 실행 파이프라인 |
