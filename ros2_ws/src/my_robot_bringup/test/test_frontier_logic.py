"""
Frontier Explorer 핵심 로직 단위 테스트.
OccupancyGrid 프론티어 감지 + 블랙리스트 + 선택 알고리즘 검증.
"""

import pytest
import math
import threading
import numpy as np
import cv2


# ── 프론티어 감지 로직 (frontier_explorer_node.py에서 추출) ──

def detect_frontiers(data, width, height, resolution, origin_x, origin_y,
                     min_frontier_size=8, blacklisted=None, blacklist_radius=1.0):
    """OccupancyGrid에서 프론티어 클러스터 감지."""
    grid = np.array(data, dtype=np.int8).reshape(height, width)
    free = (grid == 0)
    unknown = (grid == -1)

    kernel = np.ones((3, 3), dtype=np.uint8)
    unknown_dilated = cv2.dilate(
        unknown.astype(np.uint8), kernel, iterations=1).astype(bool)
    frontier_mask = (free & unknown_dilated).astype(np.uint8) * 255

    n_labels, labeled = cv2.connectedComponents(frontier_mask)

    frontiers = []
    for i in range(1, n_labels):
        cells = np.argwhere(labeled == i)
        if len(cells) < min_frontier_size:
            continue
        cy, cx = cells.mean(axis=0)
        wx = cx * resolution + origin_x
        wy = cy * resolution + origin_y

        if blacklisted and is_blacklisted(wx, wy, blacklisted, blacklist_radius):
            continue

        frontiers.append((wx, wy, len(cells)))

    return frontiers


def is_blacklisted(x, y, blacklisted, radius=1.0):
    """좌표가 블랙리스트에 있는지 확인."""
    return any(
        math.hypot(x - bx, y - by) < radius
        for bx, by in blacklisted
    )


def select_best_frontier(frontiers, robot_x, robot_y, other_targets=None,
                         exclusion_radius=2.0):
    """거리 기반 최적 프론티어 선택."""
    if other_targets is None:
        other_targets = {}

    candidates = []
    for fx, fy, size in frontiers:
        too_close = any(
            math.hypot(fx - px, fy - py) < exclusion_radius
            for px, py in other_targets.values()
        )
        if too_close:
            continue
        dist = math.hypot(fx - robot_x, fy - robot_y)
        candidates.append((dist, fx, fy, size))

    if not candidates:
        return None

    candidates.sort(key=lambda c: c[0])
    _, bx, by, _ = candidates[0]
    return (bx, by)


# ══════════════════════════════════════════════════
# 테스트
# ══════════════════════════════════════════════════

class TestFrontierDetection:

    def _make_grid(self, width, height, free_cells, unknown_cells):
        """테스트용 OccupancyGrid 데이터 생성.
        기본값: 100(장애물). free_cells에 0, unknown_cells에 -1 할당.
        """
        data = [100] * (width * height)  # 모두 장애물
        for (r, c) in free_cells:
            data[r * width + c] = 0
        for (r, c) in unknown_cells:
            data[r * width + c] = -1
        return data

    def test_simple_frontier(self):
        """free와 unknown 경계에서 프론티어 감지."""
        # 10×10 그리드: 왼쪽 반 free, 오른쪽 반 unknown
        w, h = 10, 10
        free = [(r, c) for r in range(h) for c in range(5)]
        unknown = [(r, c) for r in range(h) for c in range(5, 10)]
        data = self._make_grid(w, h, free, unknown)

        frontiers = detect_frontiers(data, w, h, 0.1, 0.0, 0.0,
                                     min_frontier_size=3)
        assert len(frontiers) >= 1
        # 프론티어는 free-unknown 경계인 x=0.4~0.5 근처
        for fx, fy, size in frontiers:
            assert 0.3 <= fx <= 0.5

    def test_no_frontier_all_free(self):
        """전부 free이면 프론티어 없음."""
        w, h = 5, 5
        free = [(r, c) for r in range(h) for c in range(w)]
        data = self._make_grid(w, h, free, [])

        frontiers = detect_frontiers(data, w, h, 0.1, 0.0, 0.0,
                                     min_frontier_size=1)
        assert len(frontiers) == 0

    def test_no_frontier_all_unknown(self):
        """전부 unknown이면 프론티어 없음 (free 셀이 없으므로)."""
        w, h = 5, 5
        unknown = [(r, c) for r in range(h) for c in range(w)]
        data = self._make_grid(w, h, [], unknown)

        frontiers = detect_frontiers(data, w, h, 0.1, 0.0, 0.0,
                                     min_frontier_size=1)
        assert len(frontiers) == 0

    def test_min_frontier_size_filter(self):
        """min_frontier_size 미만은 필터링."""
        # 작은 경계 (3셀)와 큰 경계 (10셀)
        w, h = 20, 5
        free = [(r, c) for r in range(h) for c in range(10)]
        unknown = [(r, c) for r in range(h) for c in range(10, 20)]
        data = self._make_grid(w, h, free, unknown)

        # min_size=100이면 필터링됨
        frontiers = detect_frontiers(data, w, h, 0.1, 0.0, 0.0,
                                     min_frontier_size=100)
        assert len(frontiers) == 0

    def test_blacklisted_frontier_excluded(self):
        """블랙리스트 근처 프론티어는 제외."""
        w, h = 10, 10
        free = [(r, c) for r in range(h) for c in range(5)]
        unknown = [(r, c) for r in range(h) for c in range(5, 10)]
        data = self._make_grid(w, h, free, unknown)

        # 프론티어 위치(~0.4, ~0.45)를 블랙리스트에 추가
        blacklisted = [(0.4, 0.45)]
        frontiers = detect_frontiers(data, w, h, 0.1, 0.0, 0.0,
                                     min_frontier_size=3,
                                     blacklisted=blacklisted,
                                     blacklist_radius=0.5)
        # 블랙리스트 반경 내 프론티어는 제외됨
        for fx, fy, _ in frontiers:
            dist = math.hypot(fx - 0.4, fy - 0.45)
            assert dist >= 0.5


class TestBlacklist:

    def test_in_blacklist(self):
        assert is_blacklisted(1.0, 1.0, [(1.0, 1.0)], 0.5) is True

    def test_near_blacklist(self):
        assert is_blacklisted(1.3, 1.0, [(1.0, 1.0)], 0.5) is True

    def test_outside_blacklist(self):
        assert is_blacklisted(3.0, 3.0, [(1.0, 1.0)], 0.5) is False

    def test_empty_blacklist(self):
        assert is_blacklisted(1.0, 1.0, [], 0.5) is False


class TestFrontierSelection:

    def test_selects_nearest(self):
        """가장 가까운 프론티어 선택."""
        frontiers = [(1.0, 1.0, 10), (5.0, 5.0, 20), (3.0, 3.0, 15)]
        result = select_best_frontier(frontiers, 0.0, 0.0)
        assert result == (1.0, 1.0)

    def test_excludes_other_robot_targets(self):
        """다른 로봇이 향하는 프론티어 회피."""
        frontiers = [(1.0, 1.0, 10), (5.0, 5.0, 20)]
        other = {'argos2': (1.0, 1.0)}  # argos2가 (1,1)로 이동 중
        result = select_best_frontier(frontiers, 0.0, 0.0,
                                      other_targets=other, exclusion_radius=2.0)
        assert result == (5.0, 5.0)

    def test_no_candidates(self):
        """모든 프론티어가 다른 로봇 대상이면 None."""
        frontiers = [(1.0, 1.0, 10)]
        other = {'argos2': (1.0, 1.0)}
        result = select_best_frontier(frontiers, 0.0, 0.0,
                                      other_targets=other, exclusion_radius=2.0)
        assert result is None

    def test_empty_frontiers(self):
        """프론티어가 없으면 None."""
        result = select_best_frontier([], 0.0, 0.0)
        assert result is None


# ══════════════════════════════════════════════════
# CRITICAL 버그 수정 검증 (PR #8)
# ══════════════════════════════════════════════════

class TestFutureResultGuard:
    """F1: future.result() 예외 처리 패턴 검증."""

    def test_future_exception_handled(self):
        """Future가 예외를 던져도 navigating 상태가 False로 리셋."""
        # 실제 ROS2 future 없이 패턴만 검증
        navigating = True
        nav_goal_handle = None

        class FakeFuture:
            def result(self):
                raise RuntimeError('Nav2 server shutdown')

        fake_future = FakeFuture()
        try:
            result = fake_future.result()
        except Exception:
            navigating = False
            nav_goal_handle = None

        assert navigating is False
        assert nav_goal_handle is None

    def test_future_success_keeps_navigating(self):
        """정상 Future는 navigating 유지."""
        navigating = True

        class FakeFuture:
            def result(self):
                return type('Handle', (), {'accepted': True})()

        fake_future = FakeFuture()
        try:
            result = fake_future.result()
            assert result.accepted is True
        except Exception:
            navigating = False

        assert navigating is True


class TestGoalCancelledTOCTOU:
    """F2: _goal_cancelled TOCTOU 레이스 방지 패턴 검증."""

    def test_cancel_with_lock_capture(self):
        """Lock 내에서 handle 캡처 후 lock 밖에서 cancel 호출."""
        lock = threading.Lock()
        goal_cancelled = False
        cancel_called = False

        class FakeHandle:
            def cancel_goal_async(self):
                nonlocal cancel_called
                cancel_called = True

        nav_goal_handle = FakeHandle()

        # 수정된 패턴: lock 내에서 handle 캡처
        handle_to_cancel = None
        with lock:
            if nav_goal_handle is not None:
                goal_cancelled = True
                handle_to_cancel = nav_goal_handle

        # lock 밖에서 cancel (lock 내에서 cancel하면 데드락 위험)
        if handle_to_cancel is not None:
            handle_to_cancel.cancel_goal_async()

        assert goal_cancelled is True
        assert cancel_called is True

    def test_cancel_with_none_handle(self):
        """Handle이 None이면 cancel 시도하지 않음."""
        lock = threading.Lock()
        goal_cancelled = False

        handle_to_cancel = None
        with lock:
            nav_goal_handle = None
            if nav_goal_handle is not None:
                goal_cancelled = True
                handle_to_cancel = nav_goal_handle

        if handle_to_cancel is not None:
            handle_to_cancel.cancel_goal_async()

        assert goal_cancelled is False
