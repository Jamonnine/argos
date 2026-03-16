# ARGOS 코딩 스타일 가이드

## Python
- **포매터**: Black (line-length=100)
- **린터**: Flake8 (max-line=100, ignore E501,W503)
- **타입 체크**: mypy (선택)
- **docstring**: Google 스타일
- **import 순서**: isort (black 프로필)

## ROS2 노드
- **클래스명**: `XxxNode` (PascalCase)
- **토픽명**: `sensor/reading` (소문자/슬래시)
- **파라미터**: `snake_case` (declare_parameter)
- **콜백**: `_xxx_cb` 또는 `xxx_callback`
- **QoS**: 센서=BEST_EFFORT, 명령=RELIABLE+TRANSIENT_LOCAL

## 메시지 (.msg)
- **주석**: 파일 상단에 용도 설명 + 각 필드에 단위/범위 주석
- **필드명**: `snake_case`
- **상수**: `UPPER_CASE` (uint8 STATE_IDLE = 0)

## 테스트
- **파일명**: `test_xxx_logic.py` (단위), `test_xxx_integration.py` (통합)
- **클래스명**: `TestXxx`
- **메서드명**: `test_구체적_동작_기대결과`
- **assert**: 구체적 (`assert x == 5`, not `assert x`)

## 커밋 메시지
```
<type>(<scope>): <subject>

type: feat, fix, docs, test, refactor, chore
scope: orchestrator, gas_sensor, drone, ci, docs
```

## pre-commit
```bash
pip install pre-commit
pre-commit install
# 커밋 시 자동 검사: black, flake8, trailing-whitespace
```
