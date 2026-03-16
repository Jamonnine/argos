"""
입력 검증 유틸리티 단위 테스트.
보안 리뷰 (2026-03-16) 결과 추가.
"""
import pytest
import math

# 테스트 대상 함수 직접 구현 (rclpy 없이)
import re

ROBOT_ID_PATTERN = re.compile(r'^[a-zA-Z0-9_-]{1,32}$')
VALID_SEVERITIES = frozenset({'low', 'medium', 'high', 'critical'})
VALID_DANGER_LEVELS = frozenset({'safe', 'caution', 'danger', 'critical'})

SENSOR_LIMITS = {
    'co_ppm': (0.0, 5000.0),
    'o2_percent': (0.0, 25.0),
    'confidence': (0.0, 1.0),
    'temperature_k': (200.0, 2000.0),
    'publish_rate': (0.1, 20.0),
    'ambient_o2': (0.0, 21.0),
}


def validate_robot_id(rid): return bool(ROBOT_ID_PATTERN.match(rid))
def validate_severity(s): return s in VALID_SEVERITIES
def validate_danger_level(l): return l in VALID_DANGER_LEVELS
def clamp_sensor(v, t):
    lo, hi = SENSOR_LIMITS.get(t, (v, v))
    return max(lo, min(hi, v))
def validate_timestamp(msg_sec, now_sec, max_skew=30.0):
    return abs(now_sec - msg_sec) <= max_skew
def is_finite(v): return math.isfinite(v)
def sanitize_float(v, d=0.0): return v if math.isfinite(v) else d


class TestRobotIdValidation:
    def test_valid_ids(self):
        assert validate_robot_id('argos1') is True
        assert validate_robot_id('argos_1') is True
        assert validate_robot_id('drone-1') is True
        assert validate_robot_id('UGV_Alpha_01') is True

    def test_invalid_ids(self):
        assert validate_robot_id('') is False
        assert validate_robot_id(' ') is False
        assert validate_robot_id('a' * 33) is False  # 33자 초과
        assert validate_robot_id("'; DROP TABLE;--") is False
        assert validate_robot_id('robot id') is False  # 공백
        assert validate_robot_id('robot/../../etc') is False  # 경로 traversal
        assert validate_robot_id('<script>') is False  # XSS

    def test_boundary_length(self):
        assert validate_robot_id('a') is True  # 1자 최소
        assert validate_robot_id('a' * 32) is True  # 32자 최대
        assert validate_robot_id('a' * 33) is False  # 33자 초과


class TestSeverityValidation:
    def test_valid(self):
        for s in ['low', 'medium', 'high', 'critical']:
            assert validate_severity(s) is True

    def test_invalid(self):
        assert validate_severity('fuzzbuzz') is False
        assert validate_severity('') is False
        assert validate_severity('CRITICAL') is False  # 대소문자 구분
        assert validate_severity('normal') is False


class TestDangerLevelValidation:
    def test_valid(self):
        for l in ['safe', 'caution', 'danger', 'critical']:
            assert validate_danger_level(l) is True

    def test_invalid(self):
        assert validate_danger_level('ok') is False
        assert validate_danger_level('SAFE') is False


class TestSensorClamping:
    def test_within_range(self):
        assert clamp_sensor(100.0, 'co_ppm') == 100.0
        assert clamp_sensor(20.9, 'o2_percent') == 20.9

    def test_below_minimum(self):
        assert clamp_sensor(-10.0, 'co_ppm') == 0.0
        assert clamp_sensor(-5.0, 'confidence') == 0.0

    def test_above_maximum(self):
        assert clamp_sensor(999999.0, 'co_ppm') == 5000.0
        assert clamp_sensor(500.0, 'o2_percent') == 25.0

    def test_publish_rate_limit(self):
        """DoS 방지: 발행 빈도 상한 20Hz."""
        assert clamp_sensor(1000.0, 'publish_rate') == 20.0
        assert clamp_sensor(10.0, 'publish_rate') == 10.0

    def test_ambient_o2_limit(self):
        assert clamp_sensor(500.0, 'ambient_o2') == 21.0


class TestTimestampValidation:
    def test_valid_timestamp(self):
        assert validate_timestamp(100.0, 100.5) is True  # 0.5초 차이
        assert validate_timestamp(100.0, 129.9) is True  # 29.9초 차이

    def test_invalid_timestamp(self):
        assert validate_timestamp(100.0, 131.0) is False  # 31초 차이
        assert validate_timestamp(100.0, 0.0) is False  # 100초 차이

    def test_future_timestamp(self):
        """미래 타임스탬프도 거부."""
        assert validate_timestamp(200.0, 100.0) is False  # 100초 미래

    def test_exact_boundary(self):
        assert validate_timestamp(100.0, 130.0) is True  # 정확히 30초


class TestFloatSanitization:
    def test_finite(self):
        assert is_finite(1.0) is True
        assert is_finite(0.0) is True
        assert is_finite(-999.9) is True

    def test_nan(self):
        assert is_finite(float('nan')) is False
        assert sanitize_float(float('nan')) == 0.0
        assert sanitize_float(float('nan'), -1.0) == -1.0

    def test_inf(self):
        assert is_finite(float('inf')) is False
        assert is_finite(float('-inf')) is False
        assert sanitize_float(float('inf')) == 0.0


class TestSecurityScenarios:
    """보안 공격 시나리오 테스트."""

    def test_sql_injection_robot_id(self):
        assert validate_robot_id("'; DROP TABLE robots;--") is False

    def test_xss_robot_id(self):
        assert validate_robot_id('<script>alert(1)</script>') is False

    def test_path_traversal_robot_id(self):
        assert validate_robot_id('../../etc/passwd') is False

    def test_severity_injection(self):
        assert validate_severity("critical'; --") is False

    def test_extreme_sensor_values(self):
        """극단적 센서값으로 시스템 오동작 유도 시도."""
        assert clamp_sensor(float('inf'), 'co_ppm') == 5000.0
        assert clamp_sensor(float('-inf'), 'o2_percent') == 0.0

    def test_clock_skew_attack(self):
        """타임스탬프 조작으로 화점 만료 우회 시도."""
        # 100초 과거 타임스탬프 → 거부
        assert validate_timestamp(0.0, 100.0) is False
        # 100초 미래 타임스탬프 → 거부
        assert validate_timestamp(200.0, 100.0) is False
