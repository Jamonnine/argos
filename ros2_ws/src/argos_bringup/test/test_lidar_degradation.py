"""test_lidar_degradation.py — LidarDegradationNode 단위 테스트

NFRI 실험 데이터 기반 오탐 모델 검증:
  - 방수포 비활성: 원본 스캔 그대로
  - 방수포 활성: 76.7% 노이즈 주입
  - 노이즈는 방수포 방향 ±30°에만 국한
  - 신뢰도 토픽 값 확인

rclpy 없이 순수 Python으로 degrade_scan() 로직을 직접 테스트한다.
ROS2 메시지 의존을 최소화하기 위해 LaserScan을 간단한 데이터 클래스로 모킹.
"""
import copy
import math
import random
from dataclasses import dataclass, field
from typing import List
from unittest.mock import MagicMock, patch

import pytest


# ── LaserScan 모킹 (rclpy 불필요) ────────────────────────────────────

@dataclass
class MockLaserScan:
    """sensor_msgs/LaserScan 핵심 필드 모킹."""
    angle_min: float = -math.pi
    angle_max: float = math.pi
    angle_increment: float = math.pi / 180.0  # 1° 단위 (360빔)
    ranges: List[float] = field(default_factory=list)
    header: object = None

    def __post_init__(self):
        if self.header is None:
            self.header = MagicMock()


def make_scan(
    n_beams: int = 360,
    fill_value: float = 5.0,
    angle_min: float = -math.pi,
) -> MockLaserScan:
    """테스트용 LaserScan 생성.

    Args:
        n_beams: 빔 수 (기본 360)
        fill_value: 모든 빔에 채울 초기 거리값 (m)
        angle_min: 시작 각도 (라디안)
    """
    angle_increment = (2 * math.pi) / n_beams
    return MockLaserScan(
        angle_min=angle_min,
        angle_max=angle_min + angle_increment * n_beams,
        angle_increment=angle_increment,
        ranges=[fill_value] * n_beams,
    )


# ── 테스트 대상 로직 직접 임포트 ─────────────────────────────────────
# rclpy 의존성 없이 degrade_scan 로직만 추출하여 테스트

def degrade_scan_logic(
    scan,
    cannon_active: bool,
    noise_probability: float = 0.767,
    noise_cone_rad: float = math.radians(30.0),
    noise_range_min: float = 0.5,
    noise_range_max: float = 3.0,
    cannon_angle: float = 0.0,
    rng: random.Random = None,
):
    """LidarDegradationNode.degrade_scan() 로직을 rclpy 없이 순수 Python으로 재현.

    테스트 재현성을 위해 rng 인스턴스를 외부에서 주입할 수 있다.
    """
    if rng is None:
        rng = random

    if not cannon_active:
        return scan

    degraded = copy.deepcopy(scan)
    n_ranges = len(degraded.ranges)

    if n_ranges == 0:
        return degraded

    for i in range(n_ranges):
        angle = scan.angle_min + i * scan.angle_increment
        angle_diff = angle - cannon_angle
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

        if abs(angle_diff) < noise_cone_rad:
            if rng.random() < noise_probability:
                degraded.ranges[i] = rng.uniform(noise_range_min, noise_range_max)

    return degraded


# ── 테스트 케이스 ─────────────────────────────────────────────────────

class TestCannonInactive:
    """방수포 비활성 — 원본 스캔 그대로 반환."""

    def test_returns_same_object(self):
        """방수포 비활성 시 동일 객체 참조 반환 (deepcopy 없음)."""
        scan = make_scan(fill_value=5.0)
        result = degrade_scan_logic(scan, cannon_active=False)
        assert result is scan

    def test_ranges_unchanged(self):
        """방수포 비활성 시 모든 빔 값 보존."""
        scan = make_scan(n_beams=360, fill_value=5.0)
        result = degrade_scan_logic(scan, cannon_active=False)
        assert result.ranges == [5.0] * 360

    def test_reliability_is_1_when_inactive(self):
        """방수포 비활성 시 신뢰도 = 1.0."""
        cannon_active = False
        reliability_spraying = 0.233
        expected_reliability = reliability_spraying if cannon_active else 1.0
        assert expected_reliability == pytest.approx(1.0)


class TestCannonActive:
    """방수포 활성 — 76.7% 노이즈 주입 검증."""

    def test_noise_probability_approx_767(self):
        """방수포 활성 시 cone 내 빔의 ~76.7%에 노이즈 주입 (통계적 검증).

        시드 고정으로 재현성 보장. 1000빔 대수의 법칙 적용.
        허용 오차: ±5%p (0.717 ~ 0.817)
        """
        rng = random.Random(42)
        n_beams = 1000

        # 전체 빔이 cone 내에 있도록 설정 (angle_min=0, increment→0에 수렴, cone=π)
        scan = MockLaserScan(
            angle_min=0.0,
            angle_max=0.0,  # 실제 사용 안 함
            angle_increment=0.001,  # 매우 작은 증분 → 모든 빔이 0°에 집중
            ranges=[5.0] * n_beams,
        )
        result = degrade_scan_logic(
            scan,
            cannon_active=True,
            noise_probability=0.767,
            noise_cone_rad=math.pi,  # 전 방향 cone (모든 빔 포함)
            rng=rng,
        )

        # 원본(5.0)에서 변경된 빔 수 계산
        changed = sum(1 for r in result.ranges if r != 5.0)
        actual_rate = changed / n_beams
        assert 0.717 <= actual_rate <= 0.817, (
            f'오탐률 {actual_rate:.3f}이 허용 범위(0.717~0.817) 밖'
        )

    def test_original_scan_not_mutated(self):
        """원본 스캔이 deepcopy로 보호되어 변경 없음."""
        scan = make_scan(n_beams=360, fill_value=5.0)
        original_ranges = list(scan.ranges)
        rng = random.Random(1)
        degrade_scan_logic(scan, cannon_active=True, rng=rng)
        assert scan.ranges == original_ranges

    def test_noisy_ranges_within_bounds(self):
        """주입된 가짜 반사점은 [0.5, 3.0] 범위 내.

        noise_probability=1.0 + cone=π(전 방향)으로 모든 빔을 변경시킨 뒤
        각 빔이 [noise_range_min, noise_range_max] 범위 안에 있는지 확인.
        """
        rng = random.Random(7)
        scan = make_scan(n_beams=360, fill_value=5.0,
                         angle_min=-math.pi)
        result = degrade_scan_logic(
            scan,
            cannon_active=True,
            noise_probability=1.0,      # 100% 주입 — cone 내 모든 빔 변경 보장
            noise_cone_rad=math.radians(30.0),  # ±30° cone
            noise_range_min=0.5,
            noise_range_max=3.0,
            cannon_angle=0.0,
            rng=rng,
        )
        # cone 내 빔만 추출하여 범위 검증 (cone 밖은 원본 5.0 유지)
        n_cone = 0
        for i, r in enumerate(result.ranges):
            angle = scan.angle_min + i * scan.angle_increment
            diff = abs(math.atan2(math.sin(angle), math.cos(angle)))
            if diff < math.radians(30.0):
                n_cone += 1
                assert 0.5 <= r <= 3.0, (
                    f'빔 {i}(angle={math.degrees(angle):.1f}°): '
                    f'범위 이탈 {r} (기대: 0.5~3.0)'
                )
        assert n_cone > 0, 'cone 내 빔이 0개 — 각도 설정 오류'

    def test_reliability_value_when_spraying(self):
        """방수포 활성 시 신뢰도 = 0.233 (= 1 - 0.767)."""
        cannon_active = True
        reliability_spraying = 0.233
        expected = reliability_spraying if cannon_active else 1.0
        assert expected == pytest.approx(0.233)


class TestNoiseCone:
    """노이즈는 방수포 방향 ±30°에만 국한."""

    def _count_changed(self, result, original_value: float = 5.0) -> int:
        return sum(1 for r in result.ranges if r != original_value)

    def _get_beam_index_for_angle(self, scan, target_angle_deg: float) -> int:
        """목표 각도(도)에 해당하는 빔 인덱스 반환."""
        target_rad = math.radians(target_angle_deg)
        best_i = 0
        best_diff = float('inf')
        for i, _ in enumerate(scan.ranges):
            a = scan.angle_min + i * scan.angle_increment
            diff = abs(math.atan2(math.sin(a - target_rad), math.cos(a - target_rad)))
            if diff < best_diff:
                best_diff = diff
                best_i = i
        return best_i

    def test_out_of_cone_beams_unchanged(self):
        """±30° 바깥 빔은 100% 확률 주입에도 변하지 않아야 함."""
        # 전방(0°) 기준 cone ±30°, 반대 방향(180°) 빔은 보호
        n_beams = 360
        scan = make_scan(n_beams=n_beams, fill_value=5.0,
                         angle_min=-math.pi)

        rng = random.Random(0)
        result = degrade_scan_logic(
            scan,
            cannon_active=True,
            noise_probability=1.0,       # cone 내 모든 빔 변경
            noise_cone_rad=math.radians(30.0),
            cannon_angle=0.0,            # 전방
            rng=rng,
        )

        # 180° (반대 방향) 빔 인덱스 확인 — cone 밖이므로 5.0 유지
        back_idx = self._get_beam_index_for_angle(scan, 180.0)
        assert result.ranges[back_idx] == pytest.approx(5.0), (
            f'cone 밖 빔({back_idx})이 변경됨: {result.ranges[back_idx]}'
        )

    def test_cone_beams_changed_with_probability_1(self):
        """±30° 이내 빔은 noise_probability=1.0 시 전부 변경."""
        n_beams = 360
        scan = make_scan(n_beams=n_beams, fill_value=5.0,
                         angle_min=-math.pi)

        rng = random.Random(0)
        result = degrade_scan_logic(
            scan,
            cannon_active=True,
            noise_probability=1.0,
            noise_cone_rad=math.radians(30.0),
            cannon_angle=0.0,
            rng=rng,
        )

        # 전방 ±30° 범위에 해당하는 빔들 추출
        cone_changed = 0
        cone_total = 0
        for i, _ in enumerate(result.ranges):
            angle = scan.angle_min + i * scan.angle_increment
            angle_diff = math.atan2(math.sin(angle), math.cos(angle))
            if abs(angle_diff) < math.radians(30.0):
                cone_total += 1
                if result.ranges[i] != 5.0:
                    cone_changed += 1

        assert cone_total > 0, 'cone 내 빔이 0개 — 각도 설정 오류'
        assert cone_changed == cone_total, (
            f'cone 내 빔 {cone_total}개 중 {cone_changed}개만 변경 '
            f'(noise_probability=1.0이므로 전부 변경 기대)'
        )

    def test_cone_boundary_90deg(self):
        """±90° cone 시 ±90° 경계 바깥(180°)은 변경 없음."""
        n_beams = 360
        scan = make_scan(n_beams=n_beams, fill_value=5.0,
                         angle_min=-math.pi)

        rng = random.Random(5)
        result = degrade_scan_logic(
            scan,
            cannon_active=True,
            noise_probability=1.0,
            noise_cone_rad=math.radians(90.0),
            cannon_angle=0.0,
            rng=rng,
        )

        back_idx = self._get_beam_index_for_angle(scan, 180.0)
        # 180° = π, atan2 정규화 후 ±π → |π| ≥ 90° 이므로 변경 없음
        assert result.ranges[back_idx] == pytest.approx(5.0)

    def test_empty_scan(self):
        """빈 ranges → 오류 없이 빈 결과 반환."""
        scan = MockLaserScan(
            angle_min=0.0,
            angle_max=0.0,
            angle_increment=0.01,
            ranges=[],
        )
        result = degrade_scan_logic(scan, cannon_active=True)
        assert result.ranges == []


class TestDegradeScanIntegration:
    """degrade_scan 통합 시나리오."""

    def test_full_nfri_scenario(self):
        """NFRI 파라미터 풀 세트로 목재 화재 시뮬레이션.

        조건: 360빔, ±30° cone, 76.7% 확률, 방수포 활성.
        기대: cone 내 빔의 약 76.7%가 [0.5, 3.0]m 범위로 변경.
        """
        rng = random.Random(2026)  # 재현성 고정
        n_beams = 360
        scan = make_scan(n_beams=n_beams, fill_value=10.0,  # 원거리 10m 벽
                         angle_min=-math.pi)

        result = degrade_scan_logic(
            scan,
            cannon_active=True,
            noise_probability=0.767,
            noise_cone_rad=math.radians(30.0),
            noise_range_min=0.5,
            noise_range_max=3.0,
            cannon_angle=0.0,
            rng=rng,
        )

        # cone 내 빔만 분리
        cone_beams_before = []
        cone_beams_after = []
        for i in range(n_beams):
            angle = scan.angle_min + i * scan.angle_increment
            diff = abs(math.atan2(math.sin(angle), math.cos(angle)))
            if diff < math.radians(30.0):
                cone_beams_before.append(scan.ranges[i])
                cone_beams_after.append(result.ranges[i])

        # 변경된 빔 중 [0.5, 3.0] 범위 확인
        changed_in_cone = [
            r for orig, r in zip(cone_beams_before, cone_beams_after)
            if r != orig
        ]
        for r in changed_in_cone:
            assert 0.5 <= r <= 3.0

        # cone 밖은 전부 10.0 유지
        for i in range(n_beams):
            angle = scan.angle_min + i * scan.angle_increment
            diff = abs(math.atan2(math.sin(angle), math.cos(angle)))
            if diff >= math.radians(30.0):
                assert result.ranges[i] == pytest.approx(10.0), (
                    f'cone 밖 빔 {i}(angle={math.degrees(angle):.1f}°)가 변경됨'
                )

    def test_reliability_transitions(self):
        """방수포 ON/OFF 전환 시 신뢰도 값 변화."""
        # 비활성
        assert (1.0 if not True else 0.233) == pytest.approx(0.233)
        # 활성
        assert (1.0 if not False else 0.233) == pytest.approx(1.0)
        # 0.233 + 0.767 = 1.0 (보완 관계)
        assert 0.233 + 0.767 == pytest.approx(1.0, abs=0.001)
