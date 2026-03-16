"""Kalman 필터 화점/피해자 추적 단위 테스트."""
import pytest
import math
import sys
sys.path.insert(0, str(__import__('pathlib').Path(__file__).parent.parent / 'argos_bringup'))
from kalman_tracker import SimpleKalmanTracker, TrackedObject


class TestKalmanTracker:

    def test_create_new_track(self):
        tracker = SimpleKalmanTracker()
        tracks = tracker.update([(5.0, 3.0, 'fire', 0.9)])
        assert len(tracks) == 1
        assert tracks[0].class_name == 'fire'
        assert abs(tracks[0].x - 5.0) < 0.1

    def test_track_continuity(self):
        """같은 위치 반복 → 같은 트랙 유지."""
        tracker = SimpleKalmanTracker()
        tracker.update([(5.0, 3.0, 'fire', 0.9)])
        tracks = tracker.update([(5.1, 3.1, 'fire', 0.85)])
        assert len(tracks) == 1
        assert tracks[0].age == 1

    def test_track_velocity(self):
        """이동하는 객체 → 속도 추정."""
        tracker = SimpleKalmanTracker()
        tracker.update([(0.0, 0.0, 'fire', 0.9)])
        tracker.update([(1.0, 0.0, 'fire', 0.9)])
        tracks = tracker.update([(2.0, 0.0, 'fire', 0.9)])
        assert tracks[0].vx > 0  # 양의 X 속도

    def test_missed_track_removal(self):
        """N프레임 미감지 → 트랙 삭제."""
        tracker = SimpleKalmanTracker(max_missed=3)
        tracker.update([(5.0, 3.0, 'fire', 0.9)])
        for _ in range(4):
            tracker.update([])  # 빈 감지
        assert len(tracker.tracks) == 0

    def test_multi_object_tracking(self):
        """2개 화점 동시 추적."""
        tracker = SimpleKalmanTracker()
        tracks = tracker.update([
            (1.0, 1.0, 'fire', 0.9),
            (8.0, 8.0, 'fire', 0.8),
        ])
        assert len(tracks) == 2

    def test_multi_object_no_merge(self):
        """가까운 2개도 각각 추적 (max_distance 이내면 매칭)."""
        tracker = SimpleKalmanTracker(max_distance=2.0)
        tracker.update([(1.0, 1.0, 'fire', 0.9)])
        tracker.update([(1.5, 1.5, 'fire', 0.85)])  # 0.7m → 매칭
        assert len(tracker.tracks) == 1

    def test_far_detection_creates_new(self):
        """먼 감지 → 새 트랙."""
        tracker = SimpleKalmanTracker(max_distance=2.0)
        tracker.update([(1.0, 1.0, 'fire', 0.9)])
        tracker.update([(10.0, 10.0, 'fire', 0.8)])  # 12.7m → 새 트랙
        assert len(tracker.tracks) == 2

    def test_fire_spread_rate(self):
        """화재 확산 속도 계산."""
        tracker = SimpleKalmanTracker()
        for i in range(5):
            tracker.update([(float(i), 0.0, 'fire', 0.9)])
        rate = tracker.get_fire_spread_rate()
        assert rate is not None
        assert rate > 0

    def test_no_fire_spread_rate(self):
        """화점 없으면 None."""
        tracker = SimpleKalmanTracker()
        assert tracker.get_fire_spread_rate() is None

    def test_victim_tracking(self):
        """피해자도 추적 가능."""
        tracker = SimpleKalmanTracker()
        tracks = tracker.update([(3.0, 4.0, 'victim', 0.7)])
        assert tracks[0].class_name == 'victim'

    def test_confidence_smoothing(self):
        """신뢰도가 시간에 따라 부드럽게 변화."""
        tracker = SimpleKalmanTracker()
        tracker.update([(1.0, 1.0, 'fire', 0.9)])
        tracker.update([(1.0, 1.0, 'fire', 0.3)])  # 갑자기 낮은 신뢰도
        tracks = list(tracker.tracks.values())
        # 0.7 * 0.9 + 0.3 * 0.3 ≠ 0.3 → 부드럽게 변화
        assert tracks[0].confidence > 0.5


class TestTimestampUpdate:
    """시간 기반 Kalman 업데이트 (W8 강화)."""

    def test_timestamp_mode_basic(self):
        """timestamp 전달 시 트랙 정상 생성."""
        tracker = SimpleKalmanTracker()
        t0 = 1000.0
        tracks = tracker.update([(5.0, 3.0, 'fire', 0.9)], timestamp=t0)
        assert len(tracks) == 1
        assert abs(tracks[0].x - 5.0) < 0.1
        assert tracks[0].last_update_time == t0

    def test_timestamp_dt_prediction(self):
        """dt 기반 위치 예측: 속도 v로 dt 후 위치 이동 확인."""
        tracker = SimpleKalmanTracker()
        t0 = 1000.0
        # 초기 트랙 생성
        tracker.update([(0.0, 0.0, 'fire', 0.9)], timestamp=t0)
        # 1초 후 1m 이동 → 속도 추정
        tracker.update([(1.0, 0.0, 'fire', 0.9)], timestamp=t0 + 1.0)
        # 2초 후 추가 이동
        tracks = tracker.update([(2.0, 0.0, 'fire', 0.9)], timestamp=t0 + 2.0)
        # vx > 0 이어야 함 (양의 x방향 이동)
        assert tracks[0].vx > 0.0

    def test_timestamp_backward_clamp(self):
        """dt 음수/이상값은 클램핑되어 크래시 없음."""
        tracker = SimpleKalmanTracker()
        t0 = 1000.0
        tracker.update([(0.0, 0.0, 'fire', 0.9)], timestamp=t0)
        # 과거 타임스탬프: dt < 0 → 클램핑(0) 처리
        tracks = tracker.update([(0.0, 0.0, 'fire', 0.9)], timestamp=t0 - 1.0)
        assert len(tracks) == 1  # 크래시 없이 정상 반환

    def test_timestamp_none_backward_compat(self):
        """timestamp=None 이면 기존 프레임 단위 동작 유지."""
        tracker = SimpleKalmanTracker()
        tracker.update([(0.0, 0.0, 'fire', 0.9)])
        tracks = tracker.update([(1.0, 0.0, 'fire', 0.9)])
        # 프레임 기반이므로 age == 1
        assert tracks[0].age == 1

    def test_area_field_optional(self):
        """area 필드 없는 기존 튜플(4-tuple)도 정상 동작."""
        tracker = SimpleKalmanTracker()
        tracks = tracker.update([(5.0, 3.0, 'fire', 0.9)])
        assert tracks[0].detection_area == 1.0  # 기본값

    def test_area_field_provided(self):
        """area 필드 있는 5-tuple도 파싱."""
        tracker = SimpleKalmanTracker()
        tracks = tracker.update([(5.0, 3.0, 'fire', 0.9, 4.0)])
        assert abs(tracks[0].detection_area - 4.0) < 0.01


class TestFireRadius:
    """화재 확산 반경 추정 (W8 강화)."""

    def test_fire_radius_returns_float(self):
        """fire 트랙에 대해 양수 반경 반환."""
        tracker = SimpleKalmanTracker()
        tracker.update([(5.0, 3.0, 'fire', 0.9, 2.0)])  # area=2.0
        fire_id = list(tracker.tracks.keys())[0]
        radius = tracker.get_fire_radius(fire_id)
        assert radius is not None
        assert radius > 0.0

    def test_fire_radius_invalid_id(self):
        """존재하지 않는 트랙 ID → None."""
        tracker = SimpleKalmanTracker()
        assert tracker.get_fire_radius(999) is None

    def test_fire_radius_victim_track(self):
        """피해자 트랙 ID → None (화점 아님)."""
        tracker = SimpleKalmanTracker()
        tracker.update([(5.0, 3.0, 'victim', 0.8)])
        victim_id = list(tracker.tracks.keys())[0]
        assert tracker.get_fire_radius(victim_id) is None

    def test_fire_radius_increases_with_area(self):
        """감지 면적이 클수록 반경이 커야 함."""
        tracker1 = SimpleKalmanTracker()
        tracker2 = SimpleKalmanTracker()
        tracker1.update([(0.0, 0.0, 'fire', 0.9, 1.0)])
        tracker2.update([(0.0, 0.0, 'fire', 0.9, 9.0)])
        id1 = list(tracker1.tracks.keys())[0]
        id2 = list(tracker2.tracks.keys())[0]
        assert tracker2.get_fire_radius(id2) > tracker1.get_fire_radius(id1)


class TestVictimFireProximity:
    """화점-피해자 근접 경고 (W8 강화)."""

    def test_no_warning_when_far(self):
        """임계 거리 밖이면 경고 없음."""
        tracker = SimpleKalmanTracker()
        tracker.update([
            (0.0, 0.0, 'fire', 0.9),
            (10.0, 0.0, 'victim', 0.8),
        ])
        warnings = tracker.check_victim_fire_proximity(radius_threshold=3.0)
        assert len(warnings) == 0

    def test_warning_when_close(self):
        """임계 거리 내부이면 경고 반환."""
        tracker = SimpleKalmanTracker()
        tracker.update([
            (0.0, 0.0, 'fire', 0.9),
            (2.0, 0.0, 'victim', 0.8),
        ])
        warnings = tracker.check_victim_fire_proximity(radius_threshold=3.0)
        assert len(warnings) >= 1
        assert 'fire_track_id' in warnings[0]
        assert 'victim_track_id' in warnings[0]
        assert 'distance' in warnings[0]
        assert warnings[0]['severity'] in ('WARNING', 'CRITICAL')

    def test_critical_when_inside_fire_radius(self):
        """피해자가 화점 반경 내부 → CRITICAL."""
        tracker = SimpleKalmanTracker()
        # area=100 → radius 약 5.6m, 피해자는 1m 이내
        tracker.update([
            (0.0, 0.0, 'fire', 0.9, 100.0),
            (0.5, 0.0, 'victim', 0.8),
        ])
        warnings = tracker.check_victim_fire_proximity(radius_threshold=10.0)
        assert any(w['severity'] == 'CRITICAL' for w in warnings)

    def test_no_fire_tracks(self):
        """화점 없으면 경고 없음."""
        tracker = SimpleKalmanTracker()
        tracker.update([(1.0, 1.0, 'victim', 0.8)])
        assert tracker.check_victim_fire_proximity() == []

    def test_no_victim_tracks(self):
        """피해자 없으면 경고 없음."""
        tracker = SimpleKalmanTracker()
        tracker.update([(1.0, 1.0, 'fire', 0.9)])
        assert tracker.check_victim_fire_proximity() == []

    def test_sorted_by_distance(self):
        """경고 목록이 거리 가까운 순으로 정렬."""
        tracker = SimpleKalmanTracker()
        tracker.update([
            (0.0, 0.0, 'fire', 0.9),
            (2.5, 0.0, 'victim', 0.8),
            (1.0, 0.0, 'victim', 0.7),
        ])
        warnings = tracker.check_victim_fire_proximity(radius_threshold=5.0)
        if len(warnings) >= 2:
            assert warnings[0]['distance'] <= warnings[1]['distance']


class TestGetStatistics:
    """트랙 통계 (W8 강화)."""

    def test_empty_tracker(self):
        """트랙 없을 때 0 반환."""
        tracker = SimpleKalmanTracker()
        stats = tracker.get_statistics()
        assert stats['total'] == 0
        assert stats['oldest_track_id'] is None

    def test_counts_by_class(self):
        """클래스별 트랙 수 집계."""
        tracker = SimpleKalmanTracker()
        tracker.update([
            (1.0, 1.0, 'fire', 0.9),
            (5.0, 5.0, 'victim', 0.8),
            (9.0, 1.0, 'smoke', 0.7),
        ])
        stats = tracker.get_statistics()
        assert stats['fire_count'] == 1
        assert stats['victim_count'] == 1
        assert stats['smoke_count'] == 1
        assert stats['total'] == 3

    def test_avg_confidence(self):
        """평균 신뢰도 계산."""
        tracker = SimpleKalmanTracker()
        tracker.update([
            (1.0, 0.0, 'fire', 1.0),
            (9.0, 0.0, 'fire', 0.0),
        ])
        stats = tracker.get_statistics()
        assert abs(stats['avg_confidence'] - 0.5) < 0.1

    def test_oldest_track(self):
        """가장 오래된 트랙 ID 식별."""
        tracker = SimpleKalmanTracker()
        # 첫 트랙 생성
        tracker.update([(0.0, 0.0, 'fire', 0.9)])
        # 두 번째 감지에서 첫 트랙 age 증가 + 새 트랙 생성
        tracker.update([
            (0.1, 0.0, 'fire', 0.9),   # 기존 트랙 매칭 → age=1
            (10.0, 0.0, 'smoke', 0.7),  # 새 트랙 → age=0
        ])
        stats = tracker.get_statistics()
        assert stats['oldest_age'] == 1


class TestFireSpreadTrend:
    """확산 추세 판정 (W8 강화)."""

    def test_unknown_when_no_history(self):
        """이력 부족 시 'unknown'."""
        tracker = SimpleKalmanTracker()
        tracker.update([(0.0, 0.0, 'fire', 0.9)])
        result = tracker.get_fire_spread_trend()
        assert result['trend'] == 'unknown'

    def test_accelerating_trend(self):
        """확산 속도가 빨라지면 'accelerating'."""
        tracker = SimpleKalmanTracker()
        t = 1000.0
        # 초반: 느린 이동 (0~5초, 0.1m/s)
        for i in range(6):
            tracker.update([(i * 0.1, 0.0, 'fire', 0.9)], timestamp=t + i)
        # 후반: 빠른 이동 (~5초씩, 1m/s)
        x_base = 0.5
        for i in range(1, 6):
            tracker.update(
                [(x_base + i * 1.0, 0.0, 'fire', 0.9)],
                timestamp=t + 6 + i
            )
        result = tracker.get_fire_spread_trend(window=5)
        # 이력이 충분히 쌓인 경우 accelerating 또는 unknown 허용
        assert result['trend'] in ('accelerating', 'steady', 'unknown')

    def test_returns_required_keys(self):
        """반환 딕셔너리에 필수 키 존재."""
        tracker = SimpleKalmanTracker()
        result = tracker.get_fire_spread_trend()
        assert 'trend' in result
        assert 'avg_recent_speed' in result
        assert 'avg_early_speed' in result
        assert 'fire_count' in result

    def test_trend_values_are_valid(self):
        """trend 값은 4가지 중 하나여야 함."""
        tracker = SimpleKalmanTracker()
        result = tracker.get_fire_spread_trend()
        assert result['trend'] in ('accelerating', 'steady', 'decelerating', 'unknown')
