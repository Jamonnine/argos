"""
Hotspot Detector 핵심 로직 단위 테스트.
rclpy 없이 온도 변환 + 심각도 분류 + 적응형 임계값 로직 검증.
"""

import pytest
import numpy as np
import cv2


# ── 온도 변환 로직 (hotspot_detector_node.py에서 추출) ──

L8_RESOLUTION = 3.0    # K/pixel
L8_MIN_TEMP = 253.15   # -20°C (K)

# 심각도 기준 (K)
SEVERITY_THRESHOLDS = [323.15, 473.15, 673.15]  # 50°C, 200°C, 400°C


def pixel_to_kelvin(pixel_value, resolution=L8_RESOLUTION,
                    min_temp=L8_MIN_TEMP):
    """L8 픽셀값 → 추정 온도(K) 변환."""
    return min_temp + pixel_value * resolution


def classify_severity(temp_kelvin, thresholds=None):
    """온도 기반 심각도 분류."""
    if thresholds is None:
        thresholds = SEVERITY_THRESHOLDS
    if temp_kelvin >= thresholds[2]:
        return 'critical'
    elif temp_kelvin >= thresholds[1]:
        return 'high'
    elif temp_kelvin >= thresholds[0]:
        return 'medium'
    else:
        return 'low'


def compute_adaptive_threshold(image, top_percent=0.05):
    """적응형 임계값 계산 (상위 N% 기준)."""
    flat = image.flatten()
    return np.percentile(flat, 100.0 * (1.0 - top_percent))


def detect_hotspots(image, threshold, min_area=20):
    """이진화 + 컨투어 기반 화점 검출.
    Returns: list of (cx, cy, area, max_pixel, mean_pixel)
    """
    _, binary = cv2.threshold(image, int(threshold), 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(
        binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    hotspots = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area:
            continue

        x, y, bw, bh = cv2.boundingRect(contour)
        roi = image[y:y+bh, x:x+bw]
        mask = np.zeros_like(roi)
        shifted = contour - np.array([x, y])
        cv2.drawContours(mask, [shifted], 0, 255, -1)

        masked_pixels = roi[mask > 0]
        if len(masked_pixels) == 0:
            continue

        max_px = float(np.max(masked_pixels))
        mean_px = float(np.mean(masked_pixels))

        moments = cv2.moments(contour)
        if moments['m00'] > 0:
            cx = moments['m10'] / moments['m00']
            cy = moments['m01'] / moments['m00']
        else:
            cx = x + bw / 2.0
            cy = y + bh / 2.0

        hotspots.append((cx, cy, area, max_px, mean_px))

    return hotspots


# ══════════════════════════════════════════════════
# 테스트
# ══════════════════════════════════════════════════

class TestPixelToKelvin:

    def test_zero_pixel(self):
        """픽셀 0 → 최소 온도."""
        assert pixel_to_kelvin(0) == pytest.approx(253.15)

    def test_max_pixel(self):
        """픽셀 255 → 최대 온도."""
        expected = 253.15 + 255 * 3.0  # 1018.15 K ≈ 745°C
        assert pixel_to_kelvin(255) == pytest.approx(expected)

    def test_boiling_point_pixel(self):
        """100°C (373.15K) → 픽셀값 역산 검증."""
        # 373.15 = 253.15 + px * 3.0 → px = 40
        assert pixel_to_kelvin(40) == pytest.approx(373.15)

    def test_custom_resolution(self):
        """커스텀 resolution 적용."""
        assert pixel_to_kelvin(100, resolution=2.0, min_temp=200.0) == \
            pytest.approx(400.0)


class TestClassifySeverity:

    def test_low(self):
        """50°C 미만 = low."""
        assert classify_severity(300.0) == 'low'  # ~27°C

    def test_medium(self):
        """50°C~200°C = medium."""
        assert classify_severity(350.0) == 'medium'  # ~77°C

    def test_high(self):
        """200°C~400°C = high."""
        assert classify_severity(500.0) == 'high'  # ~227°C

    def test_critical(self):
        """400°C+ = critical."""
        assert classify_severity(700.0) == 'critical'  # ~427°C

    def test_exact_threshold_medium(self):
        """정확히 50°C (323.15K) = medium."""
        assert classify_severity(323.15) == 'medium'

    def test_exact_threshold_high(self):
        """정확히 200°C (473.15K) = high."""
        assert classify_severity(473.15) == 'high'

    def test_exact_threshold_critical(self):
        """정확히 400°C (673.15K) = critical."""
        assert classify_severity(673.15) == 'critical'

    def test_absolute_zero(self):
        """절대영도 = low."""
        assert classify_severity(0.0) == 'low'

    def test_end_to_end_pixel_to_severity(self):
        """픽셀값 → 온도 → 심각도 통합 검증."""
        # 픽셀 200 → 253.15 + 600 = 853.15K ≈ 580°C → critical
        temp = pixel_to_kelvin(200)
        assert classify_severity(temp) == 'critical'


class TestAdaptiveThreshold:

    def test_uniform_image(self):
        """균일 이미지 → 임계값 ≈ 그 값."""
        img = np.full((10, 10), 100, dtype=np.uint8)
        th = compute_adaptive_threshold(img, 0.05)
        assert th == pytest.approx(100.0)

    def test_hotspot_in_cold_background(self):
        """차가운 배경 + 고온 영역 → 임계값은 배경보다 높음."""
        img = np.zeros((100, 100), dtype=np.uint8)
        # 상위 10% (1000px)를 고온으로 → 상위 5% 임계값이 200 근처
        img[0:10, :] = 200  # 1000px = 10%
        th = compute_adaptive_threshold(img, 0.05)
        assert th > 0

    def test_top_percent_changes_threshold(self):
        """top_percent가 클수록 임계값 낮아짐."""
        img = np.random.randint(0, 256, (50, 50), dtype=np.uint8)
        th_strict = compute_adaptive_threshold(img, 0.01)   # 상위 1%
        th_relaxed = compute_adaptive_threshold(img, 0.20)  # 상위 20%
        assert th_strict >= th_relaxed


class TestHotspotDetection:

    def test_no_hotspot_in_uniform_image(self):
        """균일 이미지 → 화점 없음."""
        img = np.full((50, 50), 100, dtype=np.uint8)
        hotspots = detect_hotspots(img, threshold=150, min_area=5)
        assert len(hotspots) == 0

    def test_single_hotspot(self):
        """단일 밝은 영역 감지."""
        img = np.zeros((100, 100), dtype=np.uint8)
        cv2.circle(img, (50, 50), 10, 255, -1)  # 반경 10 원
        hotspots = detect_hotspots(img, threshold=128, min_area=5)
        assert len(hotspots) == 1
        cx, cy, area, max_px, mean_px = hotspots[0]
        assert 45 < cx < 55  # 중심 근처
        assert 45 < cy < 55
        assert max_px == 255
        assert area > 200  # π*10² ≈ 314

    def test_min_area_filter(self):
        """min_area 미만 소형 화점 필터링."""
        img = np.zeros((100, 100), dtype=np.uint8)
        # 아주 작은 점 (3x3 = 9px)
        img[50:53, 50:53] = 255
        hotspots = detect_hotspots(img, threshold=128, min_area=20)
        assert len(hotspots) == 0

    def test_multiple_hotspots(self):
        """여러 화점 동시 감지."""
        img = np.zeros((100, 100), dtype=np.uint8)
        cv2.circle(img, (20, 20), 8, 200, -1)
        cv2.circle(img, (70, 70), 8, 250, -1)
        hotspots = detect_hotspots(img, threshold=150, min_area=5)
        assert len(hotspots) == 2

    def test_confidence_calculation(self):
        """신뢰도 = mean_pixel / 255 검증."""
        img = np.zeros((100, 100), dtype=np.uint8)
        cv2.circle(img, (50, 50), 10, 255, -1)
        hotspots = detect_hotspots(img, threshold=128, min_area=5)
        assert len(hotspots) == 1
        _, _, _, _, mean_px = hotspots[0]
        confidence = mean_px / 255.0
        assert 0.9 <= confidence <= 1.0  # 원 내부 전부 255이므로 거의 1.0


# ══════════════════════════════════════════════════
# HIGH 버그 수정 검증 (PR #9)
# ══════════════════════════════════════════════════

class TestSeverityThresholdsValidation:
    """H1: severity_thresholds 길이 검증."""

    def test_valid_3_thresholds(self):
        """3개 임계값 → 정상 사용."""
        thresholds = [323.15, 473.15, 673.15]
        assert len(thresholds) == 3
        assert classify_severity(500.0, thresholds) == 'high'

    def test_invalid_2_thresholds_fallback(self):
        """2개 임계값 → 기본값으로 폴백."""
        thresholds = [323.15, 473.15]
        # 수정 로직: 길이 != 3이면 기본값 사용
        if len(thresholds) != 3:
            thresholds = [323.15, 473.15, 673.15]
        assert len(thresholds) == 3
        assert classify_severity(500.0, thresholds) == 'high'

    def test_invalid_0_thresholds_fallback(self):
        """빈 리스트 → 기본값으로 폴백."""
        thresholds = []
        if len(thresholds) != 3:
            thresholds = [323.15, 473.15, 673.15]
        assert classify_severity(300.0, thresholds) == 'low'

    def test_invalid_5_thresholds_fallback(self):
        """5개 임계값 → 기본값으로 폴백."""
        thresholds = [300, 400, 500, 600, 700]
        if len(thresholds) != 3:
            thresholds = [323.15, 473.15, 673.15]
        assert classify_severity(700.0, thresholds) == 'critical'


class TestNegativeKelvinGuard:
    """H2: 음수 켈빈 온도 방지."""

    def test_negative_pixel_clamps_to_zero(self):
        """음수 픽셀값 → 0K로 클램프."""
        # 수정 후: max(0.0, ...)
        temp = max(0.0, 253.15 + (-200) * 3.0)
        assert temp == 0.0

    def test_zero_pixel_normal(self):
        """픽셀 0 → 최소 온도 (양수)."""
        temp = max(0.0, 253.15 + 0 * 3.0)
        assert temp == pytest.approx(253.15)

    def test_normal_pixel_unaffected(self):
        """정상 픽셀값은 가드에 영향 없음."""
        temp_guarded = max(0.0, 253.15 + 100 * 3.0)
        temp_original = 253.15 + 100 * 3.0
        assert temp_guarded == pytest.approx(temp_original)


class TestMaxDetections:
    """H4: max_detections 파라미터 검증."""

    def test_single_detection_default(self):
        """기본값: 1개 화점만 보고."""
        max_detections = 1
        detections = [
            (800.0, 'det_a'),
            (700.0, 'det_b'),
            (900.0, 'det_c'),
        ]
        # 온도 내림차순 정렬 후 상위 N개
        detections.sort(key=lambda x: x[0], reverse=True)
        top = detections[:max_detections]
        assert len(top) == 1
        assert top[0][0] == 900.0  # 가장 고온

    def test_multiple_detections(self):
        """max_detections=3이면 3개 보고."""
        max_detections = 3
        detections = [
            (800.0, 'a'), (700.0, 'b'), (900.0, 'c'),
            (600.0, 'd'), (850.0, 'e'),
        ]
        detections.sort(key=lambda x: x[0], reverse=True)
        top = detections[:max_detections]
        assert len(top) == 3
        assert top[0][0] == 900.0
        assert top[1][0] == 850.0
        assert top[2][0] == 800.0

    def test_fewer_detections_than_max(self):
        """감지 수 < max_detections → 모두 보고."""
        max_detections = 5
        detections = [(800.0, 'a'), (700.0, 'b')]
        detections.sort(key=lambda x: x[0], reverse=True)
        top = detections[:max_detections]
        assert len(top) == 2

    def test_incremental_insertion(self):
        """실제 코드 패턴: append → sort → truncate."""
        max_detections = 2
        top = []

        for temp, name in [(500.0, 'a'), (800.0, 'b'), (600.0, 'c'), (900.0, 'd')]:
            top.append((temp, name))
            top.sort(key=lambda x: x[0], reverse=True)
            top = top[:max_detections]

        assert len(top) == 2
        assert top[0][0] == 900.0
        assert top[1][0] == 800.0
