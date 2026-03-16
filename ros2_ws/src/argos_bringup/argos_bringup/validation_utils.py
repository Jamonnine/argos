"""validation_utils.py — ARGOS 입력 검증 공통 모듈

ROS2 메시지의 외부 입력(robot_id, severity, 센서값 등)을
검증하여 오염/위변조/오류를 방지하는 유틸리티.

보안 리뷰 (2026-03-16) 결과 신규 생성.
"""

import re
import math
from typing import Optional

# ── 문자열 검증 ──

ROBOT_ID_PATTERN = re.compile(r'^[a-zA-Z0-9_-]{1,32}$')
VALID_SEVERITIES = frozenset({'low', 'medium', 'high', 'critical'})
VALID_DANGER_LEVELS = frozenset({'safe', 'caution', 'danger', 'critical'})
VALID_DRONE_STATES = frozenset({'grounded', 'taking_off', 'hovering', 'flying', 'landing'})


def validate_robot_id(robot_id: str) -> bool:
    """로봇 ID 유효성 검증 (영숫자 + 하이픈/밑줄, 1~32자)."""
    return bool(ROBOT_ID_PATTERN.match(robot_id))


def validate_severity(severity: str) -> bool:
    """심각도 등급 유효성 검증."""
    return severity in VALID_SEVERITIES


def validate_danger_level(level: str) -> bool:
    """위험도 등급 유효성 검증."""
    return level in VALID_DANGER_LEVELS


# ── 숫자 범위 검증 ──

# 물리적 한계값 (센서 고장/위변조 감지용)
SENSOR_LIMITS = {
    'co_ppm': (0.0, 5000.0),           # CO: 5000ppm 이상은 센서 한계 초과
    'o2_percent': (0.0, 25.0),          # O2: 대기 20.9%, 산소 과잉 23.5%까지
    'lel_percent': (0.0, 100.0),        # LEL: 0~100%
    'co2_ppm': (0.0, 100000.0),         # CO2: 10% = 100,000ppm
    'hcn_ppm': (0.0, 500.0),            # HCN: 500ppm = 즉사
    'temperature_k': (200.0, 2000.0),   # 온도: -73°C ~ 1727°C
    'confidence': (0.0, 1.0),           # 신뢰도
    'battery_percent': (0.0, 100.0),    # 배터리
    'distance_m': (0.0, 1000.0),        # 거리: 1km 이내
    'intensity_db': (-10.0, 200.0),     # 소리 강도
    'ambient_co': (0.0, 50.0),          # 대기 CO 기저값
    'ambient_o2': (0.0, 21.0),          # 대기 O2 기저값
    'publish_rate': (0.1, 20.0),        # 발행 빈도 상한 20Hz
}


def clamp_sensor(value: float, sensor_type: str,
                 logger=None) -> float:
    """센서 값을 물리적 한계 내로 클램핑.

    Returns:
        클램핑된 값. 범위 밖이면 로거로 경고.
    """
    limits = SENSOR_LIMITS.get(sensor_type)
    if limits is None:
        return value

    lo, hi = limits
    if value < lo or value > hi:
        clamped = max(lo, min(hi, value))
        if logger:
            logger.warn(
                f'Sensor value out of range: {sensor_type}={value} '
                f'(valid: {lo}~{hi}), clamped to {clamped}')
        return clamped
    return value


def validate_sensor_range(value: float, sensor_type: str) -> bool:
    """센서 값이 물리적 한계 내인지 확인."""
    limits = SENSOR_LIMITS.get(sensor_type)
    if limits is None:
        return True
    return limits[0] <= value <= limits[1]


# ── 타임스탬프 검증 ──

MAX_TIMESTAMP_SKEW_SEC = 30.0  # 최대 허용 클록 스큐


def validate_timestamp(msg_stamp_sec: float, now_sec: float,
                       max_skew: float = MAX_TIMESTAMP_SKEW_SEC) -> bool:
    """메시지 타임스탬프의 합리성 검증.

    Args:
        msg_stamp_sec: 메시지 헤더의 타임스탬프 (초)
        now_sec: 현재 시간 (초)
        max_skew: 최대 허용 시간차 (초)

    Returns:
        True면 유효, False면 의심스러운 타임스탬프
    """
    skew = abs(now_sec - msg_stamp_sec)
    return skew <= max_skew


# ── NaN/Inf 검증 ──

def is_finite(value: float) -> bool:
    """NaN/Inf가 아닌 유한한 값인지 확인."""
    return math.isfinite(value)


def sanitize_float(value: float, default: float = 0.0) -> float:
    """NaN/Inf를 기본값으로 대체."""
    return value if math.isfinite(value) else default
