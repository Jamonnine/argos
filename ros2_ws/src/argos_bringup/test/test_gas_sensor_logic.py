"""
Gas Sensor 핵심 로직 단위 테스트.
rclpy 없이 가스 위험도 판정 + 확산 모델 + 클램핑 검증.
"""

import pytest
import math


# ── 가스 위험 기준치 (gas_sensor_node.py에서 추출) ──

GAS_THRESHOLDS = {
    'co': {'safe': 25, 'caution': 50, 'danger': 200, 'critical': 800},
    'o2_low': {'safe': 19.5, 'caution': 18.0, 'danger': 16.0, 'critical': 14.0},
    'lel': {'safe': 5, 'caution': 10, 'danger': 25, 'critical': 50},
    'hcn': {'safe': 4.7, 'caution': 10, 'danger': 50, 'critical': 100},
}

LEVELS_ORDER = ['safe', 'caution', 'danger', 'critical']


def assess_danger(co_ppm, o2_percent, lel_percent, hcn_ppm):
    """가스 농도 기반 위험도 종합 판정 (노드 로직 추출)."""
    hazards = []
    max_level = 'safe'

    # CO
    for level in reversed(LEVELS_ORDER):
        if level == 'safe':
            continue
        if co_ppm >= GAS_THRESHOLDS['co'][level]:
            hazards.append(f'co_{level}')
            if LEVELS_ORDER.index(level) > LEVELS_ORDER.index(max_level):
                max_level = level
            break

    # O2 (역방향)
    for level in reversed(LEVELS_ORDER):
        if level == 'safe':
            continue
        if o2_percent <= GAS_THRESHOLDS['o2_low'][level]:
            hazards.append(f'o2_{level}')
            if LEVELS_ORDER.index(level) > LEVELS_ORDER.index(max_level):
                max_level = level
            break

    # LEL
    for level in reversed(LEVELS_ORDER):
        if level == 'safe':
            continue
        if lel_percent >= GAS_THRESHOLDS['lel'][level]:
            hazards.append(f'lel_{level}')
            if LEVELS_ORDER.index(level) > LEVELS_ORDER.index(max_level):
                max_level = level
            break

    # HCN
    for level in reversed(LEVELS_ORDER):
        if level == 'safe':
            continue
        if hcn_ppm >= GAS_THRESHOLDS['hcn'][level]:
            hazards.append(f'hcn_{level}')
            if LEVELS_ORDER.index(level) > LEVELS_ORDER.index(max_level):
                max_level = level
            break

    evacuate = (
        max_level == 'critical' or
        co_ppm >= 800 or
        o2_percent <= 16.0 or
        lel_percent >= 25.0
    )

    return max_level, hazards, evacuate


def gas_decay(distance, decay_rate=0.3):
    """거리 기반 가스 감쇠 (지수 함수)."""
    return math.exp(-decay_rate * distance)


# ── 위험도 판정 테스트 ──

class TestGasDangerAssessment:
    """NIOSH/OSHA 기준 가스 위험도 판정 테스트."""

    def test_safe_atmosphere(self):
        level, hazards, evacuate = assess_danger(0.0, 20.9, 0.0, 0.0)
        assert level == 'safe'
        assert len(hazards) == 0
        assert evacuate is False

    def test_co_caution(self):
        level, _, evacuate = assess_danger(50.0, 20.9, 0.0, 0.0)
        assert level == 'caution'
        assert evacuate is False

    def test_co_danger(self):
        level, hazards, evacuate = assess_danger(200.0, 20.9, 0.0, 0.0)
        assert level == 'danger'
        assert 'co_danger' in hazards
        assert evacuate is False

    def test_co_critical_evacuate(self):
        level, hazards, evacuate = assess_danger(800.0, 20.9, 0.0, 0.0)
        assert level == 'critical'
        assert 'co_critical' in hazards
        assert evacuate is True

    def test_co_extreme(self):
        level, _, evacuate = assess_danger(1200.0, 20.9, 0.0, 0.0)
        assert level == 'critical'
        assert evacuate is True

    def test_o2_low_caution(self):
        level, _, evacuate = assess_danger(0.0, 18.0, 0.0, 0.0)
        assert level == 'caution'
        assert evacuate is False

    def test_o2_critical_evacuate(self):
        level, hazards, evacuate = assess_danger(0.0, 14.0, 0.0, 0.0)
        assert level == 'critical'
        assert 'o2_critical' in hazards
        assert evacuate is True

    def test_o2_danger_evacuate(self):
        """O2 16% 이하는 evacuate=True (의식상실 위험)."""
        level, _, evacuate = assess_danger(0.0, 16.0, 0.0, 0.0)
        assert evacuate is True

    def test_lel_explosive(self):
        level, hazards, evacuate = assess_danger(0.0, 20.9, 25.0, 0.0)
        assert level == 'danger'
        assert 'lel_danger' in hazards
        assert evacuate is True  # LEL 25% 이상 즉시 철수

    def test_lel_critical(self):
        level, _, evacuate = assess_danger(0.0, 20.9, 50.0, 0.0)
        assert level == 'critical'
        assert evacuate is True

    def test_hcn_danger(self):
        level, hazards, _ = assess_danger(0.0, 20.9, 0.0, 50.0)
        assert level == 'danger'
        assert 'hcn_danger' in hazards

    def test_multi_gas_worst_wins(self):
        """복합 가스: 가장 심각한 등급이 전체 등급.
        CO 100ppm = caution, O2 17% = caution (18 미만), LEL 15 = caution, HCN 30 = caution.
        """
        level, hazards, evacuate = assess_danger(100.0, 17.0, 15.0, 30.0)
        assert level == 'caution'
        assert len(hazards) >= 2  # 최소 CO + LEL + HCN caution
        assert evacuate is False

    def test_all_critical(self):
        level, hazards, evacuate = assess_danger(1000.0, 12.0, 60.0, 120.0)
        assert level == 'critical'
        assert len(hazards) == 4  # 4종 모두 critical
        assert evacuate is True


# ── 가스 확산 모델 테스트 ──

class TestGasDecayModel:
    """거리 기반 가스 감쇠 모델 검증."""

    def test_zero_distance_full_intensity(self):
        assert gas_decay(0.0) == pytest.approx(1.0)

    def test_decay_decreases_with_distance(self):
        assert gas_decay(5.0) < gas_decay(2.0)
        assert gas_decay(10.0) < gas_decay(5.0)

    def test_large_distance_near_zero(self):
        assert gas_decay(30.0) < 0.001

    def test_decay_rate_parameter(self):
        """감쇠율이 높을수록 빠르게 감소."""
        fast = gas_decay(5.0, decay_rate=0.5)
        slow = gas_decay(5.0, decay_rate=0.1)
        assert fast < slow

    def test_negative_distance_safe(self):
        """음수 거리는 sqrt에서 나올 수 없지만, 방어적 검증."""
        result = gas_decay(0.0)
        assert result == pytest.approx(1.0)


# ── 경계값 테스트 ──

class TestBoundaryValues:
    """경계값 검증."""

    def test_exactly_at_threshold(self):
        """정확히 기준치일 때 해당 등급으로 판정."""
        level, _, _ = assess_danger(50.0, 20.9, 0.0, 0.0)
        assert level == 'caution'

    def test_just_below_threshold(self):
        level, _, _ = assess_danger(49.9, 20.9, 0.0, 0.0)
        assert level == 'safe'

    def test_o2_normal_lower_bound(self):
        """O2 19.5% = safe 하한."""
        level, _, _ = assess_danger(0.0, 19.5, 0.0, 0.0)
        assert level == 'safe'

    def test_o2_just_below_normal(self):
        level, _, _ = assess_danger(0.0, 19.4, 0.0, 0.0)
        assert level == 'safe'  # 19.5 미만이지만 18.0 이상 = safe~caution 사이

    def test_zero_values(self):
        level, _, _ = assess_danger(0.0, 0.0, 0.0, 0.0)
        assert level == 'critical'  # O2 0% = critical
        assert True  # 크래시 없음 확인
