# Day 19 - QoS (Quality of Service)

**날짜**: 2026-02-19
**학습 시간**: ~40분
**핵심 주제**: QoS 4대 정책, TRANSIENT_LOCAL Late Joiner, QoS 불일치 진단

---

## ✅ 달성 결과

- QoS 4대 정책 (Reliability, Durability, History, Deadline) 이해
- `qos_demo_node.py` 구현 — BEST_EFFORT + TRANSIENT_LOCAL 동시 실험
- `/map` TRANSIENT_LOCAL: 노드 시작 0.048초 만에 지도 수신 확인
- `/scan` BEST_EFFORT: 5초간 22~23회 수신 확인
- `ros2 topic info -v`로 QoS 불일치 진단 방법 실습

---

## 📐 왜 QoS가 필요한가

ROS 1은 TCP 기반으로 항상 신뢰성 있는 전달을 보장했습니다. 그런데 카메라가 30fps로 발행하는데 AI 처리 노드가 5fps밖에 처리 못하면, 신뢰성 보장 때문에 처리 못한 프레임이 큐에 쌓입니다. 결국 AI 노드는 1초 전 낡은 이미지로 현재 결정을 내립니다. 자율 주행에서 이것은 충돌입니다.

ROS 2는 DDS 기반으로 QoS를 개발자가 명시적으로 선택할 수 있게 합니다.

---

## 🔧 QoS 4대 정책

### Reliability
- `RELIABLE`: 재전송 보장, 지연 발생 가능 → 지도, 제어 명령
- `BEST_EFFORT`: 재전송 없음, 빠름, 손실 가능 → 센서 데이터

### Durability
- `VOLATILE`: 구독자 없을 때 발행된 메시지 사라짐
- `TRANSIENT_LOCAL`: 마지막 메시지 보관 → Late Joiner도 즉시 수신

### History
- `KEEP_LAST(N)`: 최근 N개만 유지 (실시간 데이터에 적합)
- `KEEP_ALL`: 전체 보관 (메모리 위험)

### Deadline / Lifespan
- `Deadline`: 이 주기 안에 메시지 안 오면 에러 (센서 고장 감지)
- `Lifespan`: 발행 후 N ms 지난 데이터는 폐기 (낡은 데이터 차단)

---

## 🧪 실험 결과

### TRANSIENT_LOCAL 동작
```
[0.048초 후] 지도 수신 성공! (TRANSIENT_LOCAL 덕분에 늦게 시작해도 수신)
  크기: 80x102 픽셀, 해상도: 0.05m/픽셀, 실제 크기: 4.0m x 5.1m
```
SLAM이 수 분 전부터 실행 중이었지만, qos_demo가 시작되자마자 0.048초 만에 지도를 수신했습니다. Nav2가 시작되자마자 즉시 지도를 받는 이유가 이것입니다.

### BEST_EFFORT 센서 수신
```
[QoS 통계] 5초간 /scan 수신: 23회 | 지도: ✅ | 최근접 장애물: 0.50m
```
5초 22~23회 ≈ 4.5Hz. BEST_EFFORT ↔ BEST_EFFORT 정상 동작.

### QoS 호환성 규칙 (ros2 topic info -v 확인)
```
Publisher  (ros_gz_bridge): Reliability: RELIABLE
Subscriber (qos_demo):      Reliability: BEST_EFFORT
→ 호환 ✅ (Publisher가 더 강한 보장 제공)
```
Publisher가 BEST_EFFORT이고 Subscriber가 RELIABLE이면 비호환 → 조용히 연결 끊김.

---

## 🛠️ 실무 QoS 조합 패턴

| 데이터 종류 | Reliability | Durability | History |
|---|---|---|---|
| 센서 (LiDAR, IMU) | BEST_EFFORT | VOLATILE | KEEP_LAST(1) |
| 지도, 로봇 상태 | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST(1) |
| 제어 명령 (/cmd_vel) | RELIABLE | VOLATILE | KEEP_LAST(10) |

---

## 🐛 QoS 불일치 디버깅

QoS 불일치는 에러 메시지 없이 조용히 실패합니다. 진단 방법:

```bash
ros2 topic info /토픽명 -v
# Publisher와 Subscriber의 Reliability/Durability를 각각 확인
```

"토픽은 있는데 내 노드만 못 받는다" → QoS 불일치 의심.

---

*Day 19 완료 - ROS 2가 ROS 1과 다른 핵심 이유인 QoS를 실험으로 확인.*
