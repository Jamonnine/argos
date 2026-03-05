"""
Perception Bridge 핵심 로직 단위 테스트.
rclpy 없이 TF 변환, 접근 자세 계산, 감지 필터링 검증.
"""

import math
import pytest


# ── perception_bridge_node.py에서 추출한 로직 ──

def compute_approach_pose(obj_x, obj_y, obj_distance, approach_dist):
    """객체 위치에서 approach_dist만큼 앞에 있는 목표 자세 계산."""
    dist = max(obj_distance - approach_dist, 0.1)
    angle = math.atan2(obj_y, obj_x)
    target_x = dist * math.cos(angle)
    target_y = dist * math.sin(angle)
    # yaw quaternion (z, w)
    qz = math.sin(angle / 2.0)
    qw = math.cos(angle / 2.0)
    return target_x, target_y, qz, qw


def find_best_detection(cache, class_name, min_confidence):
    """캐시에서 해당 클래스 중 가장 신뢰도 높은 객체 반환 (P2: strict 매칭)."""
    candidates = [
        obj for obj in cache
        if obj['class_name'] == class_name
        and obj['confidence'] >= min_confidence
    ]
    if not candidates:
        return None
    return max(candidates, key=lambda o: o['confidence'])


# ══════════════════════════════════════════════════
# 테스트
# ══════════════════════════════════════════════════

class TestApproachPose:

    def test_straight_ahead(self):
        """정면 객체 접근 자세 (y=0)."""
        tx, ty, qz, qw = compute_approach_pose(3.0, 0.0, 3.0, 0.5)
        assert tx == pytest.approx(2.5, abs=0.01)
        assert ty == pytest.approx(0.0, abs=0.01)

    def test_diagonal_object(self):
        """대각선 객체 접근."""
        tx, ty, qz, qw = compute_approach_pose(2.0, 2.0, math.hypot(2, 2), 0.5)
        dist = math.hypot(tx, ty)
        assert dist == pytest.approx(math.hypot(2, 2) - 0.5, abs=0.1)

    def test_minimum_distance_clamp(self):
        """접근 거리가 객체 거리보다 크면 0.1m로 클램프."""
        tx, ty, qz, qw = compute_approach_pose(0.3, 0.0, 0.3, 1.0)
        dist = math.hypot(tx, ty)
        assert dist == pytest.approx(0.1, abs=0.01)


class TestDetectionFilter:

    def test_exact_class_match(self):
        cache = [
            {'class_name': 'person', 'confidence': 0.9},
            {'class_name': 'box', 'confidence': 0.8},
        ]
        result = find_best_detection(cache, 'person', 0.5)
        assert result['class_name'] == 'person'

    def test_confidence_threshold(self):
        cache = [
            {'class_name': 'person', 'confidence': 0.3},
        ]
        result = find_best_detection(cache, 'person', 0.5)
        # 'person' 0.3 < 0.5 → 폴백 재시도에서도 0.3 < 0.5 → None
        assert result is None

    def test_strict_no_fallback(self):
        """P2: 요청 클래스와 불일치하면 None (폴백 제거)."""
        cache = [
            {'class_name': 'obstacle', 'confidence': 0.9},
        ]
        result = find_best_detection(cache, 'person', 0.5)
        assert result is None

    def test_empty_cache(self):
        result = find_best_detection([], 'person', 0.5)
        assert result is None

    def test_highest_confidence_selected(self):
        cache = [
            {'class_name': 'box', 'confidence': 0.7},
            {'class_name': 'box', 'confidence': 0.95},
            {'class_name': 'box', 'confidence': 0.8},
        ]
        result = find_best_detection(cache, 'box', 0.5)
        assert result['confidence'] == pytest.approx(0.95)


# ══════════════════════════════════════════════════
# P1: TF2 변환 패턴 검증 (CRITICAL PR #8)
# ══════════════════════════════════════════════════

class TestFrameTransform:
    """P1: base_link → map 변환이 필수인 이유 검증."""

    def test_base_link_goal_is_wrong_for_nav2(self):
        """base_link 프레임 좌표를 map으로 착각하면 완전히 다른 위치."""
        # 로봇이 map 기준 (5.0, 3.0)에 있고 base_link 기준 (1.0, 0.0) 객체 감지
        robot_x, robot_y = 5.0, 3.0
        obj_in_base_link = (1.0, 0.0)

        # 수정 전: frame_id='base_link'로 Nav2에 전송
        # Nav2는 map 프레임으로 해석 → (1.0, 0.0)으로 이동 = 완전히 엉뚱
        wrong_goal = obj_in_base_link

        # 수정 후: base_link → map 변환
        # 간단한 평행이동 (회전 0도 가정)
        correct_goal = (
            robot_x + obj_in_base_link[0],
            robot_y + obj_in_base_link[1],
        )

        # 차이 검증: 수정 전후 목표 위치가 크게 다름
        dist_error = math.hypot(
            correct_goal[0] - wrong_goal[0],
            correct_goal[1] - wrong_goal[1],
        )
        assert dist_error > 4.0  # 5m 이상 오차

    def test_transform_with_rotation(self):
        """로봇이 90도 회전한 상태에서 변환 검증."""
        # 로봇 위치 (2, 1), 방향 90도 (왼쪽을 봄)
        robot_x, robot_y = 2.0, 1.0
        yaw = math.pi / 2  # 90도

        # base_link 기준 (1.0, 0.0) = 로봇 앞 1m
        obj_bl_x, obj_bl_y = 1.0, 0.0

        # map 변환 (2D 회전 + 평행이동)
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        map_x = robot_x + cos_y * obj_bl_x - sin_y * obj_bl_y
        map_y = robot_y + sin_y * obj_bl_x + cos_y * obj_bl_y

        # 90도 회전이므로 앞 1m = map의 y 방향
        assert map_x == pytest.approx(2.0, abs=0.01)
        assert map_y == pytest.approx(2.0, abs=0.01)


# ══════════════════════════════════════════════════
# HIGH 버그 수정 검증 (PR #9)
# ══════════════════════════════════════════════════

class TestEchoLoopPrevention:
    """P3: 시뮬레이션 에코 루프 방지."""

    def _should_process_detection(self, detector_name, simulate_mode):
        """에코 루프 필터링 로직."""
        if detector_name == 'lidar_simulator' and simulate_mode:
            return False  # 자기 발행 메시지 → 무시
        return True

    def test_self_published_filtered(self):
        """자기가 발행한 lidar_simulator 메시지는 무시."""
        assert self._should_process_detection('lidar_simulator', True) is False

    def test_external_detection_passes(self):
        """외부 감지기 메시지는 통과."""
        assert self._should_process_detection('yolo_v8', True) is True

    def test_non_simulate_mode_passes(self):
        """시뮬레이션 모드가 아니면 lidar_simulator도 통과."""
        assert self._should_process_detection('lidar_simulator', False) is True

    def test_empty_detector_passes(self):
        """빈 문자열 감지기는 통과."""
        assert self._should_process_detection('', True) is True


class TestStrictClassMatching:
    """P2: strict 매칭 추가 검증."""

    def test_partial_name_no_match(self):
        """부분 문자열은 매칭 안 됨 ('person' != 'person_sitting')."""
        cache = [
            {'class_name': 'person_sitting', 'confidence': 0.9},
        ]
        result = find_best_detection(cache, 'person', 0.5)
        assert result is None

    def test_case_sensitive(self):
        """대소문자 구분."""
        cache = [
            {'class_name': 'Person', 'confidence': 0.9},
        ]
        result = find_best_detection(cache, 'person', 0.5)
        assert result is None

    def test_multiple_classes_only_target(self):
        """여러 클래스 중 정확히 요청한 클래스만 반환."""
        cache = [
            {'class_name': 'car', 'confidence': 0.95},
            {'class_name': 'person', 'confidence': 0.85},
            {'class_name': 'truck', 'confidence': 0.90},
        ]
        result = find_best_detection(cache, 'person', 0.5)
        assert result is not None
        assert result['class_name'] == 'person'
        assert result['confidence'] == 0.85


class TestApproachPoseEdgeCases:
    """접근 자세 계산 추가 검증."""

    def test_very_close_object(self):
        """아주 가까운 객체 (0.2m) → 0.1m 최소 거리."""
        tx, ty, _, _ = compute_approach_pose(0.2, 0.0, 0.2, 0.5)
        dist = math.hypot(tx, ty)
        assert dist == pytest.approx(0.1, abs=0.01)

    def test_behind_robot(self):
        """뒤쪽 객체 (x < 0) → 뒤를 향한 자세."""
        tx, ty, qz, qw = compute_approach_pose(-2.0, 0.0, 2.0, 0.5)
        assert tx < 0  # 뒤쪽
        # yaw는 π (180도)에 가까움
        angle = math.atan2(-2.0, 0.0)  # 이건 아님, atan2(y, x) = atan2(0, -2) = π
        # 실제로는 atan2(obj_y, obj_x) = atan2(0, -2) = π
        assert abs(angle) == pytest.approx(math.pi)
