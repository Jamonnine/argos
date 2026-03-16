"""센서 노이즈 유틸 + Nav temporal filtering 테스트."""
import pytest
import numpy as np


# 센서 노이즈 함수 (sensor_noise_utils 생성 대기, 여기서 직접 정의)
def add_lidar_noise(ranges, sigma=0.02):
    """LiDAR 스캔에 가우시안 노이즈 추가."""
    noise = np.random.normal(0, sigma, len(ranges))
    return np.clip(ranges + noise, 0.0, None)


def temporal_filter(scan_history, min_consistent=3):
    """시간적 필터링: N프레임 일관된 장애물만 인정.

    Nav 전문가 권고: 연기 속 LiDAR 산란(false positive) 방지.
    """
    if len(scan_history) < min_consistent:
        return scan_history[-1] if scan_history else np.array([])

    stacked = np.array(scan_history[-min_consistent:])
    # 각 빔에서 N프레임 중 중앙값 사용 (노이즈 제거)
    median_scan = np.median(stacked, axis=0)
    return median_scan


def detect_smoke_interference(scan_history, variance_threshold=0.1):
    """연기 간섭 감지: 스캔 분산이 높으면 연기 속."""
    if len(scan_history) < 3:
        return False
    stacked = np.array(scan_history[-3:])
    variance = np.var(stacked, axis=0)
    high_variance_ratio = np.mean(variance > variance_threshold)
    return high_variance_ratio > 0.3  # 30% 이상 빔이 불안정


class TestLidarNoise:

    def test_noise_shape(self):
        ranges = np.array([1.0, 2.0, 3.0, 4.0])
        noisy = add_lidar_noise(ranges, sigma=0.02)
        assert noisy.shape == ranges.shape

    def test_noise_magnitude(self):
        ranges = np.ones(100) * 5.0
        noisy = add_lidar_noise(ranges, sigma=0.02)
        diff = np.abs(noisy - ranges)
        assert np.mean(diff) < 0.1  # 평균 오차 10cm 미만
        assert np.max(diff) < 0.2   # 최대 오차 20cm 미만

    def test_no_negative(self):
        ranges = np.array([0.01, 0.02, 0.03])
        noisy = add_lidar_noise(ranges, sigma=0.05)
        assert np.all(noisy >= 0.0)


class TestTemporalFilter:

    def test_single_frame(self):
        history = [np.array([1.0, 2.0, 3.0])]
        result = temporal_filter(history, min_consistent=3)
        assert len(result) == 3

    def test_consistent_frames(self):
        """일관된 스캔 → 원본 유지."""
        scan = np.array([1.0, 2.0, 3.0])
        history = [scan, scan, scan]
        result = temporal_filter(history)
        np.testing.assert_array_almost_equal(result, scan)

    def test_noisy_frame_filtered(self):
        """1프레임 노이즈 → 중앙값으로 제거."""
        normal = np.array([1.0, 2.0, 3.0])
        noisy = np.array([1.0, 9.0, 3.0])  # 2번째 빔 노이즈
        history = [normal, noisy, normal]
        result = temporal_filter(history)
        assert abs(result[1] - 2.0) < 1.0  # 중앙값 = 2.0 (노이즈 9.0 제거)

    def test_all_noisy_passes_through(self):
        """모든 프레임 노이즈 → 중앙값이라도 노이즈."""
        history = [
            np.array([1.0, 8.0, 3.0]),
            np.array([1.0, 9.0, 3.0]),
            np.array([1.0, 7.0, 3.0]),
        ]
        result = temporal_filter(history)
        assert result[1] == pytest.approx(8.0)  # 중앙값 = 8.0


class TestSmokeInterference:

    def test_clear_air(self):
        """맑은 공기 → 간섭 없음."""
        scan = np.array([5.0, 5.0, 5.0, 5.0])
        history = [scan, scan, scan]
        assert not detect_smoke_interference(history)

    def test_smoke_detected(self):
        """연기 속 → 높은 분산 → 간섭 감지."""
        history = [
            np.array([5.0, 3.0, 5.0, 2.0]),
            np.array([5.0, 7.0, 5.0, 8.0]),
            np.array([5.0, 1.0, 5.0, 4.0]),
        ]
        assert detect_smoke_interference(history, variance_threshold=0.5)

    def test_partial_smoke(self):
        """부분 연기 (일부 빔만 불안정)."""
        history = [
            np.array([5.0, 5.0, 3.0, 5.0]),
            np.array([5.0, 5.0, 8.0, 5.0]),
            np.array([5.0, 5.0, 1.0, 5.0]),
        ]
        # 25% 빔만 불안정 → 30% 미만 → False
        assert not detect_smoke_interference(history, variance_threshold=0.5)

    def test_insufficient_history(self):
        history = [np.array([1.0, 2.0])]
        assert not detect_smoke_interference(history)
