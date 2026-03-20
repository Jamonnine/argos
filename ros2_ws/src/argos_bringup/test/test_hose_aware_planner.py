"""test_hose_aware_planner.py — HoseAwarePlanner 단위 테스트.

ROS 의존성 없이 순수 Python 로직만 추출하여 검증.
Windows CI 환경에서도 동작한다.

검증 시나리오:
  1. 직선 경로 → 통과 (길이·커브·후진 모두 정상)
  2. 100m 초과 경로 → 호스 한계까지 트림
  3. U턴 경로 → 급커브 감지 (외접원 반경 < min_bend_radius)
  4. 충수 + 후진 구간 → 거부
  5. 트림 안전계수 적용 검증 (90%)
  6. 호스 잔여량이 경로보다 충분한 경우 → 트림 없음
  7. 포즈 1개 경로 → 검증 통과 (edge case)
  8. 직선 경로에서 후진 없음 확인
"""

import math
import copy
import pytest

# ──────────────────────────────────────────────────────────────────────────────
# ROS 없는 환경용 — hose_aware_planner.py 핵심 로직 추출
# ──────────────────────────────────────────────────────────────────────────────


class _Point:
    """nav_msgs/Path 내 포즈의 position 모의 클래스."""

    def __init__(self, x: float, y: float, z: float = 0.0):
        self.x = x
        self.y = y
        self.z = z


class _Pose:
    def __init__(self, x: float, y: float):
        self.position = _Point(x, y)


class _PoseStamped:
    def __init__(self, x: float, y: float):
        self.pose = _Pose(x, y)


class _Path:
    def __init__(self, poses=None):
        self.header = None
        self.poses = poses if poses is not None else []


# ── 테스트 대상 로직 (hose_aware_planner.py에서 추출) ─────────────────────


def calc_path_length(path) -> float:
    """경로 총 길이 계산 (m)."""
    total = 0.0
    poses = path.poses
    for i in range(len(poses) - 1):
        p0 = poses[i].pose.position
        p1 = poses[i + 1].pose.position
        total += math.hypot(p1.x - p0.x, p1.y - p0.y)
    return total


def trim_path_to_length(path, max_length: float):
    """경로를 max_length(m)까지 트림."""
    trimmed = _Path()
    trimmed.header = path.header
    poses = path.poses
    if not poses:
        return trimmed

    trimmed.poses.append(poses[0])
    accumulated = 0.0

    for i in range(1, len(poses)):
        p0 = poses[i - 1].pose.position
        p1 = poses[i].pose.position
        seg = math.hypot(p1.x - p0.x, p1.y - p0.y)

        if accumulated + seg > max_length:
            break

        accumulated += seg
        trimmed.poses.append(poses[i])

    return trimmed


def circumradius(p0, p1, p2):
    """세 점의 외접원 반경 (m). 일직선이면 None."""
    ax, ay = p0.x, p0.y
    bx, by = p1.x, p1.y
    cx, cy = p2.x, p2.y

    a = math.hypot(bx - cx, by - cy)
    b = math.hypot(ax - cx, ay - cy)
    c = math.hypot(ax - bx, ay - by)

    area = abs((bx - ax) * (cy - ay) - (cx - ax) * (by - ay)) / 2.0

    if area < 1e-9:
        return None

    return (a * b * c) / (4.0 * area)


def find_sharp_turns(path, min_radius: float) -> list:
    """급커브 인덱스 리스트 반환."""
    sharp = []
    poses = path.poses

    for i in range(1, len(poses) - 1):
        p0 = poses[i - 1].pose.position
        p1 = poses[i].pose.position
        p2 = poses[i + 1].pose.position

        radius = circumradius(p0, p1, p2)

        if radius is not None and radius < min_radius:
            sharp.append((i, radius))

    return sharp


def find_reverse_segments(path) -> list:
    """후진 구간 인덱스 리스트 반환."""
    poses = path.poses
    if len(poses) < 2:
        return []

    start = poses[0].pose.position
    end = poses[-1].pose.position
    overall_dx = end.x - start.x
    overall_dy = end.y - start.y
    overall_len = math.hypot(overall_dx, overall_dy)

    if overall_len < 1e-9:
        return []

    dir_x = overall_dx / overall_len
    dir_y = overall_dy / overall_len

    reverse_indices = []
    for i in range(len(poses) - 1):
        p0 = poses[i].pose.position
        p1 = poses[i + 1].pose.position
        seg_dx = p1.x - p0.x
        seg_dy = p1.y - p0.y
        dot = seg_dx * dir_x + seg_dy * dir_y
        if dot < 0.0:
            reverse_indices.append(i)

    return reverse_indices


def validate_path(path, hose_remaining: float, hose_charged: bool,
                  min_bend_radius: float = 0.5,
                  path_trim_safety_factor: float = 0.9) -> tuple:
    """경로 검증 통합 함수."""
    if len(path.poses) < 2:
        return True, 'OK', path

    # 제약 1: 길이
    total_length = calc_path_length(path)
    if total_length > hose_remaining:
        safe_limit = hose_remaining * path_trim_safety_factor
        trimmed = trim_path_to_length(path, safe_limit)
        return (
            False,
            f'경로 {total_length:.1f}m > 호스 {hose_remaining:.1f}m',
            trimmed,
        )

    # 제약 2: 급커브
    sharp_turns = find_sharp_turns(path, min_bend_radius)
    if sharp_turns:
        # 스무딩: 급커브 포즈를 전후 중간점으로 교체
        smoothed = _Path()
        smoothed.poses = [copy.deepcopy(p) for p in path.poses]
        for idx, _ in sharp_turns:
            poses = smoothed.poses
            if 0 < idx < len(poses) - 1:
                prev_p = poses[idx - 1].pose.position
                next_p = poses[idx + 1].pose.position
                poses[idx].pose.position.x = (prev_p.x + next_p.x) / 2.0
                poses[idx].pose.position.y = (prev_p.y + next_p.y) / 2.0
        return False, f'급커브 {len(sharp_turns)}곳 완화', smoothed

    # 제약 3: 충수 후진
    if hose_charged:
        reverse_segments = find_reverse_segments(path)
        if reverse_segments:
            return (
                False,
                f'충수 후진 구간 {len(reverse_segments)}곳 감지 — 경로 거부',
                path,
            )

    return True, 'OK', path


# ──────────────────────────────────────────────────────────────────────────────
# 헬퍼: 경로 생성
# ──────────────────────────────────────────────────────────────────────────────


def make_straight_path(length_m: float, steps: int = 10) -> _Path:
    """X축 방향 직선 경로 생성."""
    poses = []
    for i in range(steps + 1):
        x = length_m * i / steps
        poses.append(_PoseStamped(x, 0.0))
    return _Path(poses)


def make_uturn_path() -> _Path:
    """급커브 경로: 0.3m 스케일 직각 꺾임 — 외접원 반경 약 0.21m (< min_bend_radius 0.5m).

    (0,0)→(0.3,0)→(0.3,0.3)→(0,0.3) 형태.
    실제 소방 현장에서 문틀·모서리 회전 시 발생하는 급커브를 모의.
    """
    return _Path([
        _PoseStamped(0.0, 0.0),
        _PoseStamped(0.3, 0.0),
        _PoseStamped(0.3, 0.3),
        _PoseStamped(0.0, 0.3),
    ])


def make_reverse_path() -> _Path:
    """전진 후 후진: (0,0)→(10,0)→(5,0) — 뒤로 돌아오는 구간 포함."""
    return _Path([
        _PoseStamped(0.0, 0.0),
        _PoseStamped(5.0, 0.0),
        _PoseStamped(10.0, 0.0),
        _PoseStamped(5.0, 0.0),   # 후진 구간
    ])


# ──────────────────────────────────────────────────────────────────────────────
# 테스트
# ──────────────────────────────────────────────────────────────────────────────


class TestCalcPathLength:
    """경로 길이 계산 검증."""

    def test_직선_10m(self):
        path = make_straight_path(10.0)
        assert abs(calc_path_length(path) - 10.0) < 0.01

    def test_단일_포즈_길이_0(self):
        path = _Path([_PoseStamped(0.0, 0.0)])
        assert calc_path_length(path) == 0.0

    def test_빈_경로_길이_0(self):
        path = _Path([])
        assert calc_path_length(path) == 0.0


class TestTrimPath:
    """경로 트림 검증."""

    def test_50m_경로_45m로_트림(self):
        """호스 50m, 안전계수 0.9 → 45m까지 트림."""
        path = make_straight_path(50.0, steps=50)
        trimmed = trim_path_to_length(path, 45.0)
        length = calc_path_length(trimmed)
        # 트림 후 길이는 45m 이하여야 함
        assert length <= 45.0 + 0.1

    def test_트림_후_최소_1_포즈(self):
        """트림 한도가 0이어도 첫 포즈는 유지."""
        path = make_straight_path(10.0, steps=5)
        trimmed = trim_path_to_length(path, 0.0)
        assert len(trimmed.poses) >= 1

    def test_한도_충분하면_트림_없음(self):
        """max_length가 경로보다 크면 전체 포즈 유지."""
        path = make_straight_path(10.0, steps=10)
        trimmed = trim_path_to_length(path, 100.0)
        assert len(trimmed.poses) == len(path.poses)


class TestCircumradius:
    """외접원 반경 계산 검증."""

    def test_직각삼각형(self):
        """직각삼각형 (3-4-5): 외접원 반경 = 빗변/2 = 2.5."""
        p0 = _Point(0.0, 0.0)
        p1 = _Point(3.0, 0.0)
        p2 = _Point(0.0, 4.0)
        r = circumradius(p0, p1, p2)
        assert r is not None
        assert abs(r - 2.5) < 0.01

    def test_일직선_반경_None(self):
        """세 점이 일직선이면 None 반환."""
        p0 = _Point(0.0, 0.0)
        p1 = _Point(1.0, 0.0)
        p2 = _Point(2.0, 0.0)
        r = circumradius(p0, p1, p2)
        assert r is None

    def test_소반경_급커브(self):
        """작은 삼각형 → 작은 외접원 반경 (급커브)."""
        # 거의 직각에 가까운 0.1m 스케일 삼각형
        p0 = _Point(0.0, 0.0)
        p1 = _Point(0.1, 0.0)
        p2 = _Point(0.0, 0.1)
        r = circumradius(p0, p1, p2)
        assert r is not None
        assert r < 0.5  # min_bend_radius=0.5 미만 → 급커브


class TestFindSharpTurns:
    """급커브 감지 검증."""

    def test_직선_경로_급커브_없음(self):
        """직선 경로에서 급커브 인덱스가 비어 있어야 함."""
        path = make_straight_path(20.0, steps=20)
        sharp = find_sharp_turns(path, min_radius=0.5)
        assert len(sharp) == 0

    def test_u턴_경로_급커브_감지(self):
        """U턴 경로에서 꺾임 지점이 감지되어야 함."""
        path = make_uturn_path()
        sharp = find_sharp_turns(path, min_radius=0.5)
        # (5,0)→(5,2) 꺾임: 0.5m 이하 반경으로 감지
        # (실제 반경은 삼각형 스케일에 따라 달라지므로 0.5m 이하 여부만 검증)
        # U턴 꺾임 반경은 작아야 함 — 5m×2m 삼각형의 예각 꼭짓점
        assert len(sharp) > 0

    def test_포즈_2개_급커브_없음(self):
        """포즈가 2개이면 급커브 계산 불가 → 빈 리스트."""
        path = _Path([_PoseStamped(0.0, 0.0), _PoseStamped(10.0, 0.0)])
        sharp = find_sharp_turns(path, min_radius=0.5)
        assert len(sharp) == 0


class TestFindReverseSegments:
    """후진 구간 감지 검증."""

    def test_직선_전진_후진_없음(self):
        """순수 전진 경로에서 후진 구간이 없어야 함."""
        path = make_straight_path(20.0)
        reverse = find_reverse_segments(path)
        assert len(reverse) == 0

    def test_후진_포함_경로_감지(self):
        """전진 후 되돌아오는 구간이 있으면 후진 감지."""
        path = make_reverse_path()
        reverse = find_reverse_segments(path)
        assert len(reverse) > 0

    def test_포즈_1개_후진_없음(self):
        """포즈가 1개이면 후진 판단 불가 → 빈 리스트."""
        path = _Path([_PoseStamped(0.0, 0.0)])
        reverse = find_reverse_segments(path)
        assert len(reverse) == 0


class TestValidatePath:
    """validate_path 통합 시나리오 검증."""

    def test_직선_경로_통과(self):
        """짧은 직선 경로: 길이·커브·후진 모두 정상 → 통과."""
        path = make_straight_path(20.0)
        passed, reason, out_path = validate_path(
            path, hose_remaining=100.0, hose_charged=False
        )
        assert passed is True
        assert reason == 'OK'
        assert len(out_path.poses) == len(path.poses)

    def test_100m_초과_경로_트림(self):
        """110m 경로, 호스 100m → 90m로 트림."""
        path = make_straight_path(110.0, steps=110)
        passed, reason, out_path = validate_path(
            path, hose_remaining=100.0, hose_charged=False
        )
        assert passed is False
        assert '트림' in reason or '호스' in reason
        # 트림 후 길이는 90m(100*0.9) 이하여야 함
        trimmed_length = calc_path_length(out_path)
        assert trimmed_length <= 90.0 + 0.1

    def test_트림_안전계수_90퍼센트(self):
        """트림 한도가 hose_remaining * 0.9 인지 확인."""
        path = make_straight_path(60.0, steps=60)
        passed, reason, out_path = validate_path(
            path, hose_remaining=50.0, hose_charged=False,
            path_trim_safety_factor=0.9,
        )
        assert passed is False
        trimmed_length = calc_path_length(out_path)
        # 50m * 0.9 = 45m 이하
        assert trimmed_length <= 45.0 + 0.1

    def test_u턴_급커브_감지(self):
        """U턴 경로: 급커브 감지 → False 반환."""
        path = make_uturn_path()
        passed, reason, out_path = validate_path(
            path, hose_remaining=100.0, hose_charged=False
        )
        assert passed is False
        assert '급커브' in reason

    def test_충수_후진_거부(self):
        """충수 상태에서 후진 구간이 있으면 거부."""
        path = make_reverse_path()
        passed, reason, out_path = validate_path(
            path, hose_remaining=100.0, hose_charged=True
        )
        assert passed is False
        assert '후진' in reason

    def test_건수_후진_허용(self):
        """건수(hose_charged=False) 상태에서는 후진이 있어도 통과."""
        path = make_reverse_path()
        # 후진 경로지만 건수 상태 → 제약 3 미적용
        passed, reason, out_path = validate_path(
            path, hose_remaining=100.0, hose_charged=False
        )
        # 직선 경로이므로 급커브도 없고 길이도 20m 이내 → 통과
        assert passed is True

    def test_포즈_1개_엣지_케이스(self):
        """포즈가 1개인 경로 → 검증 없이 통과."""
        path = _Path([_PoseStamped(0.0, 0.0)])
        passed, reason, out_path = validate_path(
            path, hose_remaining=100.0, hose_charged=False
        )
        assert passed is True

    def test_호스_잔여_충분_트림_없음(self):
        """경로 20m, 호스 잔여 100m → 트림 없이 원본 유지."""
        path = make_straight_path(20.0, steps=20)
        passed, reason, out_path = validate_path(
            path, hose_remaining=100.0, hose_charged=False
        )
        assert passed is True
        assert len(out_path.poses) == len(path.poses)
