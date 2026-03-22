# Copyright 2026 민발 (Minbal), 대구강북소방서
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""S-D5 편대 조율 매니저 단위 테스트.

테스트 범위:
  1. 패턴 생성 (LINE_ABREAST / COLUMN / ECHELON / SURROUND × 기본 + 엣지)
  2. 충돌 감지
  3. 드론 고도 검증
  4. 혼합 편대 (2UGV + 2드론)
  5. 단일 로봇
  6. 간격 클램핑 (MIN_SPACING 강제)
  7. heading 회전 검증
  8. SURROUND 대칭성

rclpy 없이 순수 Python으로 실행 가능.
"""
import math
import pytest

from argos_bringup.formation_manager import (
    FormationManager,
    FormationPattern,
    RobotConfig,
)

# ── 픽스처 & 헬퍼 ─────────────────────────────────────────────────────────────

ORIGIN = (0.0, 0.0)
CENTER = (5.0, 3.0)
DEFAULT_ALT = FormationManager.DEFAULT_DRONE_ALTITUDE
MIN_SP = FormationManager.MIN_SPACING


def make_ugv(robot_id, x=0.0, y=0.0) -> RobotConfig:
    """UGV RobotConfig 생성 헬퍼."""
    return RobotConfig(robot_id=robot_id, robot_type='ugv', current_pos=(x, y))


def make_drone(robot_id, x=0.0, y=0.0, z=5.0) -> RobotConfig:
    """드론 RobotConfig 생성 헬퍼."""
    return RobotConfig(robot_id=robot_id, robot_type='drone', current_pos=(x, y, z))


@pytest.fixture
def fleet_2u2d():
    """2UGV + 2드론 혼합 편대 (S-D5 기준)."""
    return FormationManager([
        make_ugv('ugv1', x=0.0, y=0.0),
        make_ugv('ugv2', x=2.0, y=0.0),
        make_drone('drone1', x=1.0, y=0.0, z=5.0),
        make_drone('drone2', x=3.0, y=0.0, z=5.0),
    ])


@pytest.fixture
def fleet_ugv_only():
    """UGV 2대만."""
    return FormationManager([
        make_ugv('ugv1'),
        make_ugv('ugv2'),
    ])


@pytest.fixture
def fleet_drone_only():
    """드론 2대만."""
    return FormationManager([
        make_drone('d1'),
        make_drone('d2'),
    ])


@pytest.fixture
def fleet_single_ugv():
    """UGV 1대."""
    return FormationManager([make_ugv('ugv1')])


@pytest.fixture
def fleet_single_drone():
    """드론 1대."""
    return FormationManager([make_drone('d1')])


@pytest.fixture
def fleet_empty():
    """로봇 없음."""
    return FormationManager([])


# ── 1. 패턴 생성: LINE_ABREAST ────────────────────────────────────────────────

class TestLineAbreast:

    def test_returns_all_robot_ids(self, fleet_2u2d):
        """결과에 등록된 모든 로봇 ID가 포함되어야 한다."""
        wp = fleet_2u2d.compute_formation(ORIGIN, FormationPattern.LINE_ABREAST)
        assert set(wp.keys()) == {'ugv1', 'ugv2', 'drone1', 'drone2'}

    def test_ugv_positions_symmetric_about_center(self, fleet_ugv_only):
        """heading=0 횡대에서 UGV 2대는 Y축 대칭 배치."""
        spacing = 4.0
        wp = fleet_ugv_only.compute_formation(
            ORIGIN, FormationPattern.LINE_ABREAST, spacing=spacing, heading=0.0
        )
        # heading=0 → perp = pi/2 → Y 방향으로 분산
        y1, y2 = wp['ugv1'][1], wp['ugv2'][1]
        assert abs(y1 + y2) < 1e-9, "두 UGV의 Y 좌표 합이 0이어야 한다 (대칭)"
        assert abs(abs(y1) - spacing / 2) < 1e-9

    def test_drone_paired_with_ugv(self, fleet_2u2d):
        """드론은 동일 인덱스 UGV와 같은 X/Y 좌표에 배치된다."""
        wp = fleet_2u2d.compute_formation(ORIGIN, FormationPattern.LINE_ABREAST)
        ugv1_xy = wp['ugv1'][:2]
        drone1_xy = wp['drone1'][:2]
        assert ugv1_xy == pytest.approx(drone1_xy, abs=1e-9)

    def test_excess_drone_placed_at_center(self):
        """UGV보다 드론이 많으면 여분 드론은 편대 중심 상공."""
        fm = FormationManager([
            make_ugv('ugv1'),
            make_drone('d1'),
            make_drone('d2'),   # 여분 드론
        ])
        wp = fm.compute_formation(CENTER, FormationPattern.LINE_ABREAST)
        assert wp['d2'][:2] == pytest.approx(CENTER, abs=1e-9)

    def test_ugv_only_no_crash(self, fleet_ugv_only):
        """UGV 전용 편대에서도 결과가 정상 반환된다."""
        wp = fleet_ugv_only.compute_formation(ORIGIN, FormationPattern.LINE_ABREAST)
        assert len(wp) == 2

    def test_heading_rotates_formation(self, fleet_ugv_only):
        """heading 값이 달라지면 UGV 배치도 달라진다."""
        wp0 = fleet_ugv_only.compute_formation(ORIGIN, FormationPattern.LINE_ABREAST, heading=0.0)
        wp90 = fleet_ugv_only.compute_formation(
            ORIGIN, FormationPattern.LINE_ABREAST, heading=math.pi / 2
        )
        # heading=0: Y 분산, heading=pi/2: X 분산
        assert wp0['ugv1'] != wp90['ugv1']


# ── 2. 패턴 생성: COLUMN ─────────────────────────────────────────────────────

class TestColumn:

    def test_ugv_aligned_along_heading(self, fleet_ugv_only):
        """heading=0 종대에서 UGV는 X축으로 일렬 배치."""
        wp = fleet_ugv_only.compute_formation(ORIGIN, FormationPattern.COLUMN, heading=0.0)
        # Y 좌표는 동일해야 한다
        assert wp['ugv1'][1] == pytest.approx(wp['ugv2'][1], abs=1e-9)
        # X 좌표는 다르다
        assert wp['ugv1'][0] != pytest.approx(wp['ugv2'][0], abs=1e-9)

    def test_drone_altitude_set(self, fleet_drone_only):
        """종대 드론의 고도는 drone_altitude 값과 일치해야 한다."""
        alt = 12.0
        wp = fleet_drone_only.compute_formation(
            ORIGIN, FormationPattern.COLUMN, drone_altitude=alt
        )
        for did in ['d1', 'd2']:
            assert wp[did][2] == pytest.approx(alt, abs=1e-9)

    def test_mixed_fleet_all_present(self, fleet_2u2d):
        """혼합 편대 종대에서 모든 로봇 ID가 포함된다."""
        wp = fleet_2u2d.compute_formation(CENTER, FormationPattern.COLUMN)
        assert len(wp) == 4


# ── 3. 패턴 생성: ECHELON ────────────────────────────────────────────────────

class TestEchelon:

    def test_robots_ordered_diagonally(self, fleet_2u2d):
        """제대에서 로봇들이 사선(heading+45도) 방향으로 순차 배치된다."""
        wp = fleet_2u2d.compute_formation(ORIGIN, FormationPattern.ECHELON, heading=0.0)
        # UGV 먼저 — ugv1이 ugv2보다 중심에 가까워야 한다
        d1 = math.hypot(wp['ugv1'][0], wp['ugv1'][1])
        d2 = math.hypot(wp['ugv2'][0], wp['ugv2'][1])
        assert d1 < d2, "ugv1이 ugv2보다 중심에 가까워야 한다"

    def test_drone_has_altitude(self, fleet_2u2d):
        """제대 드론의 Z 좌표가 drone_altitude 와 일치한다."""
        alt = 10.0
        wp = fleet_2u2d.compute_formation(
            ORIGIN, FormationPattern.ECHELON, drone_altitude=alt
        )
        assert wp['drone1'][2] == pytest.approx(alt, abs=1e-9)
        assert wp['drone2'][2] == pytest.approx(alt, abs=1e-9)

    def test_ugv_no_z_coordinate(self, fleet_2u2d):
        """제대 UGV 좌표는 2D (x, y) 튜플이어야 한다."""
        wp = fleet_2u2d.compute_formation(ORIGIN, FormationPattern.ECHELON)
        assert len(wp['ugv1']) == 2
        assert len(wp['ugv2']) == 2


# ── 4. 패턴 생성: SURROUND ───────────────────────────────────────────────────

class TestSurround:

    def test_robots_equidistant_from_center(self, fleet_2u2d):
        """포위 패턴에서 모든 로봇은 중심으로부터 동일 거리(= spacing)."""
        spacing = 4.0
        wp = fleet_2u2d.compute_formation(CENTER, FormationPattern.SURROUND, spacing=spacing)
        for robot_id, pos in wp.items():
            dist = math.hypot(pos[0] - CENTER[0], pos[1] - CENTER[1])
            assert dist == pytest.approx(spacing, abs=1e-9), f"{robot_id}: 거리={dist}"

    def test_surround_angles_uniform(self, fleet_2u2d):
        """포위 패턴 각도 간격이 360/n 도로 균등해야 한다."""
        spacing = 3.0
        wp = fleet_2u2d.compute_formation(CENTER, FormationPattern.SURROUND, spacing=spacing)
        angles = []
        for pos in wp.values():
            a = math.atan2(pos[1] - CENTER[1], pos[0] - CENTER[0])
            angles.append(a % (2 * math.pi))
        angles.sort()
        n = len(angles)
        expected_gap = 2 * math.pi / n
        for i in range(n):
            gap = (angles[(i + 1) % n] - angles[i]) % (2 * math.pi)
            assert gap == pytest.approx(expected_gap, abs=1e-6), f"간격 불균등: {gap}"

    def test_surround_empty_fleet(self, fleet_empty):
        """로봇 없는 편대에서 포위 패턴은 빈 딕셔너리를 반환한다."""
        wp = fleet_empty.compute_formation(CENTER, FormationPattern.SURROUND)
        assert wp == {}

    def test_surround_single_robot(self, fleet_single_ugv):
        """단일 UGV 포위: 중심에서 spacing 만큼 떨어진 위치 1개."""
        spacing = 5.0
        wp = fleet_single_ugv.compute_formation(CENTER, FormationPattern.SURROUND, spacing=spacing)
        pos = wp['ugv1']
        dist = math.hypot(pos[0] - CENTER[0], pos[1] - CENTER[1])
        assert dist == pytest.approx(spacing, abs=1e-9)


# ── 5. 단일 로봇 ──────────────────────────────────────────────────────────────

class TestSingleRobot:

    def test_single_ugv_line_abreast(self, fleet_single_ugv):
        """UGV 1대 횡대는 중심 좌표에 배치된다."""
        wp = fleet_single_ugv.compute_formation(CENTER, FormationPattern.LINE_ABREAST)
        assert wp['ugv1'] == pytest.approx(CENTER, abs=1e-9)

    def test_single_drone_has_altitude(self, fleet_single_drone):
        """드론 1대 편대에서 고도가 반드시 포함된다."""
        alt = 15.0
        wp = fleet_single_drone.compute_formation(
            CENTER, FormationPattern.LINE_ABREAST, drone_altitude=alt
        )
        assert len(wp['d1']) == 3
        assert wp['d1'][2] == pytest.approx(alt, abs=1e-9)


# ── 6. 간격 클램핑 ────────────────────────────────────────────────────────────

class TestSpacingClamp:

    def test_spacing_below_min_is_clamped(self, fleet_ugv_only):
        """spacing < MIN_SPACING 이면 MIN_SPACING 으로 강제 적용된다."""
        tiny = 0.1  # MIN_SPACING(1.5) 미만
        wp = fleet_ugv_only.compute_formation(
            ORIGIN, FormationPattern.LINE_ABREAST, spacing=tiny
        )
        # heading=0 → perp=pi/2 → Y 방향 분산
        y1, y2 = wp['ugv1'][1], wp['ugv2'][1]
        actual_spacing = abs(y1 - y2)
        assert actual_spacing == pytest.approx(MIN_SP, abs=1e-9), \
            f"클램핑 미적용: actual={actual_spacing}"

    def test_spacing_at_min_boundary(self, fleet_ugv_only):
        """spacing == MIN_SPACING 은 그대로 사용된다."""
        wp = fleet_ugv_only.compute_formation(
            ORIGIN, FormationPattern.LINE_ABREAST, spacing=MIN_SP
        )
        y1, y2 = wp['ugv1'][1], wp['ugv2'][1]
        assert abs(y1 - y2) == pytest.approx(MIN_SP, abs=1e-9)


# ── 7. heading 회전 검증 ──────────────────────────────────────────────────────

class TestHeadingRotation:

    def test_column_heading_90_aligns_y(self, fleet_ugv_only):
        """heading=pi/2 종대에서 UGV는 Y축으로 배열된다."""
        wp = fleet_ugv_only.compute_formation(
            ORIGIN, FormationPattern.COLUMN, spacing=3.0, heading=math.pi / 2
        )
        # heading=pi/2 → cos=0, sin=1 → X 동일, Y 다름
        assert wp['ugv1'][0] == pytest.approx(wp['ugv2'][0], abs=1e-9)
        assert wp['ugv1'][1] != pytest.approx(wp['ugv2'][1], abs=1e-9)

    def test_echelon_heading_zero_vs_pi(self, fleet_2u2d):
        """heading=0 과 heading=pi 제대는 대칭 배치여야 한다.

        ugv1 (인덱스 0)은 항상 center에 배치되므로 ugv2 (인덱스 1)로 비교한다.
        """
        wp0 = fleet_2u2d.compute_formation(ORIGIN, FormationPattern.ECHELON, heading=0.0)
        wppi = fleet_2u2d.compute_formation(ORIGIN, FormationPattern.ECHELON, heading=math.pi)
        # ugv2 는 사선 방향 1번째 오프셋 위치 — heading 반전 시 반대 방향
        assert wp0['ugv2'] != wppi['ugv2']
        # x 좌표 부호가 반전되어야 한다 (대칭)
        assert wp0['ugv2'][0] == pytest.approx(-wppi['ugv2'][0], abs=1e-9)


# ── 8. 충돌 감지 ──────────────────────────────────────────────────────────────

class TestCollisionRisk:

    def test_no_risk_when_spacing_large(self, fleet_ugv_only):
        """충분한 간격에서 충돌 위험 없음."""
        wp = fleet_ugv_only.compute_formation(
            ORIGIN, FormationPattern.LINE_ABREAST, spacing=10.0
        )
        risks = fleet_ugv_only.check_collision_risk(wp, min_distance=1.5)
        assert risks == []

    def test_risk_detected_when_ugvs_overlap(self):
        """UGV 2대가 최소 거리 미만이면 위험 감지."""
        fm = FormationManager([make_ugv('u1'), make_ugv('u2')])
        # 의도적으로 가까운 웨이포인트 주입
        wp = {'u1': (0.0, 0.0), 'u2': (0.5, 0.0)}
        risks = fm.check_collision_risk(wp, min_distance=1.0)
        assert len(risks) == 1
        assert risks[0][0] in ('u1', 'u2')
        assert risks[0][2] == pytest.approx(0.5, abs=1e-9)

    def test_ugv_drone_no_risk(self, fleet_2u2d):
        """UGV-드론 쌍은 고도 차이로 충돌 불가 — 위험 목록에서 제외."""
        # 동일 X/Y 로 강제 주입해도 이종 간 충돌 검사는 skip
        wp = {
            'ugv1': (0.0, 0.0),
            'drone1': (0.0, 0.0, 8.0),
        }
        risks = fleet_2u2d.check_collision_risk(wp, min_distance=1.5)
        # ugv1-drone1 쌍은 이종이므로 제외, 위험 없음
        assert all('ugv' not in pair[0] or 'drone' not in pair[1] for pair in risks)
        # 더 직관적: 이 wp에는 UGV 1대, 드론 1대 → 충돌 없음
        assert risks == []

    def test_multiple_risks_detected(self):
        """3대 UGV 밀집 배치에서 복수 위험 쌍 반환."""
        fm = FormationManager([make_ugv('u1'), make_ugv('u2'), make_ugv('u3')])
        wp = {'u1': (0.0, 0.0), 'u2': (0.3, 0.0), 'u3': (0.6, 0.0)}
        risks = fm.check_collision_risk(wp, min_distance=1.0)
        # u1-u2, u2-u3, u1-u3 모두 위험
        assert len(risks) == 3

    def test_exact_min_distance_not_risk(self, fleet_ugv_only):
        """거리 == min_distance 는 위험이 아니다 (경계 조건)."""
        wp = {'ugv1': (0.0, 0.0), 'ugv2': (1.5, 0.0)}
        risks = fleet_ugv_only.check_collision_risk(wp, min_distance=1.5)
        assert risks == []


# ── 9. 프로퍼티 ───────────────────────────────────────────────────────────────

class TestProperties:

    def test_robot_count(self, fleet_2u2d):
        assert fleet_2u2d.robot_count == 4

    def test_ugv_count(self, fleet_2u2d):
        assert fleet_2u2d.ugv_count == 2

    def test_drone_count(self, fleet_2u2d):
        assert fleet_2u2d.drone_count == 2

    def test_empty_counts(self, fleet_empty):
        assert fleet_empty.robot_count == 0
        assert fleet_empty.ugv_count == 0
        assert fleet_empty.drone_count == 0


# ── 10. 알 수 없는 패턴 ───────────────────────────────────────────────────────

class TestUnknownPattern:

    def test_raises_value_error(self, fleet_2u2d):
        """알 수 없는 패턴 문자열은 ValueError를 발생시킨다."""
        with pytest.raises(ValueError, match="알 수 없는 편대 패턴"):
            fleet_2u2d.compute_formation(ORIGIN, pattern='DIAMOND')
