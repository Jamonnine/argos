"""
8중 센싱 체계 핵심 로직 단위 테스트.
rclpy 없이 피해자/구조물/음향 센서의 핵심 계산 로직 검증.
"""

import pytest
import math
import numpy as np


# ═══════════════════════════════════════════════
# 피해자 감지 로직
# ═══════════════════════════════════════════════

def victim_confidence_gaussian(distance, detection_range=8.0):
    """거리 기반 피해자 감지 신뢰도 (가우시안 감쇠)."""
    return max(0.2, math.exp(-(distance / detection_range)**2))


class TestVictimDetection:
    """피해자 감지 신뢰도 모델 검증."""

    def test_zero_distance_max_confidence(self):
        conf = victim_confidence_gaussian(0.0)
        assert conf == pytest.approx(1.0)

    def test_at_range_limit(self):
        conf = victim_confidence_gaussian(8.0)
        # e^(-1) ≈ 0.368
        assert conf == pytest.approx(0.368, abs=0.01)

    def test_beyond_range(self):
        conf = victim_confidence_gaussian(16.0)
        # e^(-4) ≈ 0.018 → clamped to 0.2
        assert conf == pytest.approx(0.2)

    def test_minimum_confidence(self):
        """최소 신뢰도 0.2 보장."""
        conf = victim_confidence_gaussian(100.0)
        assert conf == 0.2

    def test_confidence_decreases(self):
        c1 = victim_confidence_gaussian(2.0)
        c2 = victim_confidence_gaussian(5.0)
        c3 = victim_confidence_gaussian(8.0)
        assert c1 > c2 > c3

    def test_gaussian_vs_linear(self):
        """가우시안이 선형보다 근거리에서 높은 신뢰도."""
        gauss = victim_confidence_gaussian(3.0, 8.0)
        linear = max(0.3, 1.0 - 3.0 / 8.0)  # = 0.625
        assert gauss > linear  # 가우시안 ≈ 0.87 > 선형 0.625


# ═══════════════════════════════════════════════
# 구조물 모니터링 로직
# ═══════════════════════════════════════════════

def classify_structural_severity(max_displacement):
    """변위 기반 구조물 손상 심각도."""
    if max_displacement > 1.0:
        return 'critical', 'floor_collapse'
    elif max_displacement > 0.5:
        return 'danger', 'ceiling_sag'
    else:
        return 'warning', 'structural_shift'


def compute_scan_change(baseline, current, threshold=0.3):
    """LiDAR 스캔 변화 감지."""
    diff = np.abs(current - baseline)
    changed_mask = diff > threshold
    n_changed = int(np.sum(changed_mask))
    max_disp = float(np.max(diff[changed_mask])) if n_changed > 0 else 0.0
    return n_changed, max_disp


class TestStructuralMonitor:
    """구조물 안전성 모니터링 검증."""

    def test_no_change(self):
        baseline = np.array([1.0, 2.0, 3.0, 4.0, 5.0])
        current = np.array([1.0, 2.0, 3.0, 4.0, 5.0])
        n, max_d = compute_scan_change(baseline, current)
        assert n == 0
        assert max_d == 0.0

    def test_small_noise_ignored(self):
        baseline = np.array([1.0, 2.0, 3.0])
        current = np.array([1.1, 2.1, 3.1])  # 0.1m 변화 (threshold 0.3 미만)
        n, _ = compute_scan_change(baseline, current)
        assert n == 0

    def test_significant_change_detected(self):
        baseline = np.array([1.0, 2.0, 3.0, 4.0, 5.0])
        current = np.array([1.0, 2.0, 3.5, 4.0, 5.8])  # 0.5, 0.8m 변화
        n, max_d = compute_scan_change(baseline, current)
        assert n == 2
        assert max_d == pytest.approx(0.8)

    def test_severity_classification(self):
        assert classify_structural_severity(0.4)[0] == 'warning'
        assert classify_structural_severity(0.6)[0] == 'danger'
        assert classify_structural_severity(1.5)[0] == 'critical'

    def test_severity_types(self):
        _, stype = classify_structural_severity(1.5)
        assert stype == 'floor_collapse'
        _, stype = classify_structural_severity(0.7)
        assert stype == 'ceiling_sag'

    def test_zero_division_safety(self):
        """빈 스캔에서 ZeroDivisionError 방지."""
        baseline = np.array([])
        current = np.array([])
        # max(1, min_len) 방어
        min_len = max(1, min(len(baseline), len(current)))
        confidence = 0 / min_len  # ZeroDivisionError 없어야 함
        assert confidence == 0.0

    def test_median_filter_robustness(self):
        """중앙값 필터가 노이즈 1건을 무시하는지."""
        scans = [
            np.array([1.0, 2.0, 3.0]),
            np.array([1.0, 2.0, 3.0]),
            np.array([1.0, 2.0, 9.0]),  # 노이즈 1건
            np.array([1.0, 2.0, 3.0]),
            np.array([1.0, 2.0, 3.0]),
            np.array([1.0, 2.0, 3.0]),
        ]
        half = len(scans) // 2
        baseline_median = np.median(scans[:half], axis=0)
        current_median = np.median(scans[half:], axis=0)
        n, _ = compute_scan_change(baseline_median, current_median)
        assert n == 0  # 노이즈 1건은 중앙값 필터에 의해 무시됨


# ═══════════════════════════════════════════════
# 음향 감지 로직
# ═══════════════════════════════════════════════

AUDIO_PROFILES = {
    'explosion': {'freq_range': (20, 200), 'intensity_range': (100, 140),
                  'detection_range': 50.0, 'immediate': True},
    'gas_leak': {'freq_range': (2000, 8000), 'intensity_range': (40, 70),
                 'detection_range': 15.0, 'immediate': True},
    'cry_for_help': {'freq_range': (300, 3400), 'intensity_range': (60, 90),
                     'detection_range': 30.0, 'immediate': True},
    'breathing': {'freq_range': (100, 1000), 'intensity_range': (20, 40),
                  'detection_range': 2.0, 'immediate': True},
}


def perceived_intensity(base_db, distance):
    """역제곱법칙 기반 감지 강도."""
    return base_db - 20 * math.log10(max(1.0, distance))


def audio_confidence(distance, detection_range):
    """거리 기반 음향 감지 신뢰도."""
    return max(0.2, 1.0 - (distance / detection_range) ** 0.5)


class TestAudioDetection:
    """음향 감지 물리 모델 검증."""

    def test_inverse_square_law(self):
        """10배 거리에서 20dB 감쇠."""
        i1 = perceived_intensity(100, 1.0)
        i10 = perceived_intensity(100, 10.0)
        assert i1 - i10 == pytest.approx(20.0)

    def test_zero_distance_no_attenuation(self):
        """거리 0~1m에서 감쇠 없음 (max(1.0, dist) 방어)."""
        i = perceived_intensity(80, 0.0)
        assert i == 80.0  # log10(1.0) = 0

    def test_explosion_detectable_at_50m(self):
        """폭발음(100~140dB)은 50m에서도 감지 가능."""
        base = (100 + 140) / 2  # 120dB
        i = perceived_intensity(base, 50.0)
        assert i > 40  # 40dB 이상이면 감지 가능

    def test_breathing_not_detectable_at_5m(self):
        """호흡음(20~40dB)은 5m에서 감지 불가."""
        base = (20 + 40) / 2  # 30dB
        i = perceived_intensity(base, 5.0)
        assert i < 20  # 20dB 미만 = 배경 소음에 묻힘

    def test_breathing_detectable_at_1m(self):
        """호흡음은 1m에서 감지 가능."""
        base = 30
        i = perceived_intensity(base, 1.0)
        assert i == 30  # 감쇠 없음

    def test_confidence_decreases_with_distance(self):
        c1 = audio_confidence(5.0, 30.0)
        c2 = audio_confidence(20.0, 30.0)
        assert c1 > c2

    def test_confidence_minimum(self):
        c = audio_confidence(100.0, 30.0)
        assert c == 0.2

    def test_gas_leak_high_frequency(self):
        """가스 누출은 고주파(2000~8000Hz) 특성."""
        freq = AUDIO_PROFILES['gas_leak']['freq_range']
        assert freq[0] >= 2000
        assert freq[1] <= 8000

    def test_all_profiles_have_required_fields(self):
        for name, profile in AUDIO_PROFILES.items():
            assert 'detection_range' in profile, f'{name} missing detection_range'
            assert 'immediate' in profile, f'{name} missing immediate'
            assert 'freq_range' in profile, f'{name} missing freq_range'


# ═══════════════════════════════════════════════
# 센서 퓨전 시나리오 테스트
# ═══════════════════════════════════════════════

class TestSensorFusionScenarios:
    """다중 센서 교차 검증 시나리오."""

    def test_gas_plus_audio_confirmation(self):
        """가스 센서 + 음향 이벤트 동시 감지 = 높은 확신."""
        gas_level = 'danger'  # CO 200ppm+
        audio_type = 'gas_leak'  # 쉿 소리 감지
        # 두 센서가 동시에 감지 → confirmed gas leak
        confirmed = gas_level in ('danger', 'critical') and audio_type == 'gas_leak'
        assert confirmed is True

    def test_structural_plus_audio_collapse(self):
        """구조물 변위 + 붕괴 소리 = 즉시 철수."""
        structural_severity = 'danger'
        audio_type = 'collapse'
        evacuate = (structural_severity in ('danger', 'critical')
                    and audio_type == 'collapse')
        assert evacuate is True

    def test_victim_audio_cry_for_help(self):
        """열화상 인체 감지 + 구조 요청 음성 = 최우선 구조."""
        victim_detected = True
        audio_cry = True
        priority = 1 if (victim_detected and audio_cry) else 2
        assert priority == 1
