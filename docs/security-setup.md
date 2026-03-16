# ARGOS 보안 설정 가이드

> 보안 전문가 권고: DDS 보안 + rosbridge TLS 설정 절차

---

## 1. DDS 보안 활성화 (SROS2)

```bash
# 1. 키스토어 생성
ros2 security create_keystore ~/argos_keystore

# 2. 노드별 인증서 생성
ros2 security create_enclave ~/argos_keystore /argos/orchestrator
ros2 security create_enclave ~/argos_keystore /argos/argos1/gas_sensor
ros2 security create_enclave ~/argos_keystore /argos/argos1/frontier_explorer
# ... 각 노드별 반복

# 3. 환경변수 설정
export ROS_SECURITY_KEYSTORE=~/argos_keystore
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce  # Permissive(테스트) or Enforce(배포)

# 4. 노드 실행 (보안 활성화)
ros2 run argos_bringup orchestrator \
  --ros-args --enclave /argos/orchestrator
```

### 검증
```bash
# 인증서 없는 외부 노드가 토픽 접근 시도 → 거부됨
ros2 topic echo /orchestrator/mission_state
# → "Access denied" (보안 활성화 시)
```

---

## 2. rosbridge TLS (SSL)

```bash
# 1. 자체 서명 인증서 생성
openssl req -x509 -newkey rsa:4096 -nodes \
  -keyout ~/argos_ssl/key.pem \
  -out ~/argos_ssl/cert.pem \
  -days 365 \
  -subj "/CN=argos-mission-control"

# 2. launch 인자에 SSL 설정
ros2 launch argos_description demo.launch.py \
  ssl:=true \
  certfile:=/home/jamonnine/argos_ssl/cert.pem \
  keyfile:=/home/jamonnine/argos_ssl/key.pem

# 3. 웹 대시보드 접속
# http → https 전환
# wss://localhost:9090 (WebSocket Secure)
```

### 웹 대시보드 수정 (index.html)
```javascript
// 현재
const WS_URL = 'ws://localhost:9090';

// TLS 적용 후
const WS_URL = location.protocol === 'https:'
  ? 'wss://localhost:9090'
  : 'ws://localhost:9090';
```

---

## 3. 현재 적용된 보안 조치

| 조치 | 상태 | 파일 |
|------|------|------|
| rosbridge localhost 바인딩 | ✅ | demo.launch.py |
| 토픽 화이트리스트 | ✅ | demo.launch.py (topics_glob) |
| 서비스 화이트리스트 | ✅ | demo.launch.py (services_glob) |
| max_message_size 1MB | ✅ | demo.launch.py |
| 입력 검증 (robot_id/severity) | ✅ | validation_utils.py |
| 타임스탬프 스큐 검증 (30초) | ✅ | orchestrator_node.py |
| E-Stop 3회 연속 발행 | ✅ | orchestrator_node.py |
| CSP/X-Frame-Options 헤더 | ✅ | index.html |
| DDS 보안 (SROS2) | 📋 가이드 작성 | 이 문서 |
| rosbridge TLS | 📋 가이드 작성 | 이 문서 |

---

## 4. 배포 전 보안 체크리스트

- [ ] DDS 보안 키스토어 생성 + 노드별 인증서
- [ ] ROS_SECURITY_STRATEGY=Enforce 설정
- [ ] rosbridge SSL 인증서 생성 + 적용
- [ ] 웹 대시보드 wss:// 전환
- [ ] 방화벽: 포트 9090만 허용 (localhost 또는 VPN)
- [ ] 로그 데이터 미션 종료 후 24시간 내 삭제 (FMEA 문서 참조)
