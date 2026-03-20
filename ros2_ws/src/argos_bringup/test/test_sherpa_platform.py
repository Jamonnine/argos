"""test_sherpa_platform.py — SherpaPlatform 단위 테스트.

ROS 의존성 없이 순수 Python 로직만 추출하여 검증.
Windows CI 환경에서도 동작한다.

검증 영역:
  1. 호스 제약 로직 (진입 깊이 초과, 급커브, 충수 후진)
  2. capabilities 확인 (셰르파 extra 필드)
  3. 배터리 모델 (20kWh / 5시간)
  4. 오프라인 모드 (호스 토픽 없이 graceful 동작)
  5. 좌표 연산 (유클리드 거리, 꺾임 각도, 후진 판단)
"""

import math
import pytest
from dataclasses import dataclass, field


# ──────────────────────────────────────────────────────────────────────────────
# ROS 없는 환경용 — sherpa_platform.py 핵심 로직 추출
# (platform_interface.py RobotCapabilities 복제 포함)
# ──────────────────────────────────────────────────────────────────────────────

@dataclass
class RobotCapabilities:
    """테스트용 RobotCapabilities 복제 (platform_interface.py와 동일 구조)."""
    can_fly: bool = False
    can_drive: bool = False
    has_thermal: bool = False
    has_lidar: bool = False
    max_speed: float = 0.0
    battery_capacity: float = 100.0
    platform_type: str = 'unknown'
    extra: dict = field(default_factory=dict)


# ── 배터리 상수 (sherpa_platform.py와 동일) ──────────────────────────────────
_SHERPA_ENDURANCE_HOURS = 5.0
_SHERPA_DRAIN_RATE_PCT_PER_MIN = 100.0 / (_SHERPA_ENDURANCE_HOURS * 60.0)
_SHERPA_IDLE_FACTOR = 0.25

# ── 호스 꺾임 임계값 (sherpa_platform.py와 동일) ────────────────────────────
_KINK_RISK_THRESHOLD = 0.7


def calc_distance(x1: float, y1: float, x2: float, y2: float) -> float:
    """두 좌표 사이 유클리드 거리 (m)."""
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def calc_turn_angle(
    prev_x: float, prev_y: float,
    cur_x: float, cur_y: float,
    target_x: float, target_y: float,
) -> float:
    """세 점의 꺾임 각도 계산 (도). prev→cur→target."""
    dx1 = cur_x - prev_x
    dy1 = cur_y - prev_y
    dx2 = target_x - cur_x
    dy2 = target_y - cur_y

    len1 = math.sqrt(dx1 ** 2 + dy1 ** 2)
    len2 = math.sqrt(dx2 ** 2 + dy2 ** 2)

    if len1 < 1e-6 or len2 < 1e-6:
        return 0.0

    cos_theta = (dx1 * dx2 + dy1 * dy2) / (len1 * len2)
    cos_theta = max(-1.0, min(1.0, cos_theta))
    return math.degrees(math.acos(cos_theta))


def requires_reverse(
    home_x: float, home_y: float,
    cur_x: float, cur_y: float,
    target_x: float, target_y: float,
) -> bool:
    """목표 이동이 후진(홈 방향 접근)인지 판단."""
    dist_cur = calc_distance(cur_x, cur_y, home_x, home_y)
    dist_target = calc_distance(target_x, target_y, home_x, home_y)
    return dist_target < dist_cur


def simulate_sherpa_battery(
    start: float,
    elapsed_min: float,
    moving: bool,
) -> float:
    """셰르파 배터리 소비 시뮬레이션 (sherpa_platform._update_battery와 동일 로직)."""
    drain_factor = 1.0 if moving else _SHERPA_IDLE_FACTOR
    drain = _SHERPA_DRAIN_RATE_PCT_PER_MIN * elapsed_min * drain_factor
    return max(0.0, start - drain)


class _MockSherpaHoseChecker:
    """호스 제약 체크 로직 단독 테스트용 헬퍼.

    SherpaPlatform._check_hose_constraints 로직을 ROS 없이 재현.
    """

    def __init__(
        self,
        cur_x: float = 10.0,
        cur_y: float = 0.0,
        home_x: float = 0.0,
        home_y: float = 0.0,
        hose_remaining_m: float = 100.0,
        hose_kink_risk: float = 0.0,
        hose_charged: bool = False,
        hose_mode: str = 'active',
        hose_topic_received: bool = True,
    ):
        self.cur_x = cur_x
        self.cur_y = cur_y
        self.home_x = home_x
        self.home_y = home_y
        self.hose_remaining_m = hose_remaining_m
        self.hose_kink_risk = hose_kink_risk
        self.hose_charged = hose_charged
        self._hose_mode = hose_mode
        self._hose_topic_received = hose_topic_received

    def check_hose_constraints(
        self, target_x: float, target_y: float
    ) -> tuple[bool, str]:
        """sherpa_platform._check_hose_constraints와 동일 로직."""
        # offline 모드 또는 토픽 미수신: 통과
        if self._hose_mode == 'offline' or not self._hose_topic_received:
            return True, ''

        # 제약 1: 진입 깊이
        depth = calc_distance(self.cur_x, self.cur_y, target_x, target_y)
        if depth > self.hose_remaining_m:
            return (
                False,
                f'진입 깊이 {depth:.1f}m > 호스 잔여 {self.hose_remaining_m:.1f}m',
            )

        # 제약 2: 급커브
        if self._would_cause_kink(target_x, target_y):
            return False, f'급커브 — 호스 꺾임 위험 (risk={self.hose_kink_risk:.2f})'

        # 제약 3: 충수 후진
        if self.hose_charged and requires_reverse(
            self.home_x, self.home_y,
            self.cur_x, self.cur_y,
            target_x, target_y,
        ):
            return False, '충수 상태 후진 금지 — 배관 손상 위험'

        return True, ''

    def _would_cause_kink(self, target_x: float, target_y: float) -> bool:
        """급커브 판단 (sherpa_platform._would_cause_kink와 동일).

        토픽 기반 hose_kink_risk 임계값만 사용한다.
        기하학적 U턴 계산은 직선 후퇴를 오탐하므로 사용 안 함.
        """
        return self.hose_kink_risk >= _KINK_RISK_THRESHOLD


# ──────────────────────────────────────────────────────────────────────────────
# 1. 호스 제약 — 진입 깊이
# ──────────────────────────────────────────────────────────────────────────────

class TestHoseDepthConstraint:
    """제약 1: 진입 깊이 초과 시 이동 거부."""

    def test_within_hose_length_allowed(self):
        """목표까지 거리가 호스 잔여 이내면 통과."""
        checker = _MockSherpaHoseChecker(
            cur_x=0.0, cur_y=0.0,
            hose_remaining_m=50.0,
        )
        ok, reason = checker.check_hose_constraints(40.0, 0.0)
        assert ok is True
        assert reason == ''

    def test_exact_hose_length_allowed(self):
        """거리 == 호스 잔여: 경계값 허용 (폐구간)."""
        checker = _MockSherpaHoseChecker(
            cur_x=0.0, cur_y=0.0,
            hose_remaining_m=50.0,
        )
        ok, reason = checker.check_hose_constraints(50.0, 0.0)
        assert ok is True

    def test_exceed_hose_length_rejected(self):
        """목표까지 거리가 호스 잔여 초과 → 거부."""
        checker = _MockSherpaHoseChecker(
            cur_x=0.0, cur_y=0.0,
            hose_remaining_m=30.0,
        )
        ok, reason = checker.check_hose_constraints(50.0, 0.0)
        assert ok is False
        assert '진입 깊이' in reason
        assert '호스 잔여' in reason

    def test_hose_nearly_depleted_short_move_allowed(self):
        """호스 잔여 5m일 때 3m 이동은 허용."""
        checker = _MockSherpaHoseChecker(
            cur_x=0.0, cur_y=0.0,
            hose_remaining_m=5.0,
        )
        ok, _ = checker.check_hose_constraints(3.0, 0.0)
        assert ok is True

    def test_hose_depleted_any_move_rejected(self):
        """호스 잔여 0m → 1m 이동도 거부."""
        checker = _MockSherpaHoseChecker(
            cur_x=0.0, cur_y=0.0,
            hose_remaining_m=0.0,
        )
        ok, reason = checker.check_hose_constraints(1.0, 0.0)
        assert ok is False
        assert '진입 깊이' in reason

    def test_diagonal_distance_calculation(self):
        """대각선 이동: 피타고라스 거리 정확히 계산."""
        # 3-4-5 직각삼각형: 거리 = 5
        checker = _MockSherpaHoseChecker(
            cur_x=0.0, cur_y=0.0,
            hose_remaining_m=4.9,
        )
        ok, reason = checker.check_hose_constraints(3.0, 4.0)
        assert ok is False  # 5.0 > 4.9

    def test_diagonal_within_hose_allowed(self):
        """대각선 이동 5m, 호스 잔여 6m → 허용."""
        checker = _MockSherpaHoseChecker(
            cur_x=0.0, cur_y=0.0,
            hose_remaining_m=6.0,
        )
        ok, _ = checker.check_hose_constraints(3.0, 4.0)
        assert ok is True


# ──────────────────────────────────────────────────────────────────────────────
# 2. 호스 제약 — 급커브
# ──────────────────────────────────────────────────────────────────────────────

class TestHoseKinkConstraint:
    """제약 2: 급커브(호스 꺾임) 위험 시 이동 거부."""

    def test_low_kink_risk_allowed(self):
        """꺾임 위험도 낮음(0.3) + 부드러운 커브 → 통과."""
        checker = _MockSherpaHoseChecker(
            cur_x=10.0, cur_y=0.0,
            home_x=0.0, home_y=0.0,
            hose_kink_risk=0.3,
        )
        # 직진 방향 (꺾임 없음)
        ok, _ = checker.check_hose_constraints(20.0, 0.0)
        assert ok is True

    def test_high_kink_risk_rejected(self):
        """꺾임 위험도 높음(0.8 ≥ 0.7) → 급커브 거부."""
        checker = _MockSherpaHoseChecker(
            cur_x=10.0, cur_y=0.0,
            hose_kink_risk=0.8,
        )
        ok, reason = checker.check_hose_constraints(20.0, 5.0)
        assert ok is False
        assert '급커브' in reason

    def test_kink_risk_threshold_boundary(self):
        """꺾임 위험도 정확히 0.7 → 거부 (임계값 이상)."""
        checker = _MockSherpaHoseChecker(
            cur_x=10.0, cur_y=0.0,
            hose_kink_risk=0.7,
        )
        ok, reason = checker.check_hose_constraints(20.0, 0.0)
        assert ok is False
        assert '급커브' in reason

    def test_kink_risk_just_below_threshold(self):
        """꺾임 위험도 0.699 → 토픽 기준 통과 (각도 판단으로 위임)."""
        checker = _MockSherpaHoseChecker(
            cur_x=10.0, cur_y=0.0,
            home_x=0.0, home_y=0.0,
            hose_kink_risk=0.699,
        )
        # 직진 방향 (각도 ~0도) → 각도 기준도 통과
        ok, _ = checker.check_hose_constraints(20.0, 0.0)
        assert ok is True

    def test_sharp_angle_turn_kink_risk_rejected(self):
        """급커브: hose_kink_risk ≥ 0.7 토픽 기반 → 거부.

        실제 호스 꺾임은 U턴 방향 이동 시 /hose/status 센서가 감지한다.
        """
        checker = _MockSherpaHoseChecker(
            cur_x=10.0, cur_y=0.0,
            home_x=0.0, home_y=0.0,
            hose_kink_risk=0.75,     # 센서가 꺾임 위험 감지
            hose_remaining_m=100.0,
        )
        ok, reason = checker.check_hose_constraints(20.0, 0.0)
        assert ok is False
        assert '급커브' in reason

    def test_gentle_curve_allowed(self):
        """완만한 커브(45도 이내) → 통과."""
        # home(0,0) → cur(10,0) → target(15,3): 약 17도 꺾임
        checker = _MockSherpaHoseChecker(
            cur_x=10.0, cur_y=0.0,
            home_x=0.0, home_y=0.0,
            hose_kink_risk=0.0,
            hose_remaining_m=100.0,
        )
        ok, _ = checker.check_hose_constraints(15.0, 3.0)
        assert ok is True


# ──────────────────────────────────────────────────────────────────────────────
# 3. 호스 제약 — 충수 후진 금지
# ──────────────────────────────────────────────────────────────────────────────

class TestHoseChargedReverseConstraint:
    """제약 3: 충수 상태에서 후진(홈 방향 접근) 금지."""

    def test_charged_forward_allowed(self):
        """충수 상태 + 전진(홈에서 멀어짐) → 통과."""
        checker = _MockSherpaHoseChecker(
            cur_x=10.0, cur_y=0.0,
            home_x=0.0, home_y=0.0,
            hose_charged=True,
            hose_remaining_m=100.0,
        )
        # 목표 20m: 홈에서 더 멀어짐 → 전진
        ok, _ = checker.check_hose_constraints(20.0, 0.0)
        assert ok is True

    def test_charged_reverse_rejected(self):
        """충수 상태 + 후진(홈 방향 이동) → 거부.

        테스트 설계 주의: U턴(각도>135도)은 급커브 제약(제약2)이 먼저 걸린다.
        충수 후진 제약(제약3)을 독립 검증하려면 U턴이 아닌 방향으로 설정.
        cur(0,10): home(0,0)에서 북쪽 10m 진입. target(0,5): 남쪽으로 후진 (각도=0도).
        """
        checker = _MockSherpaHoseChecker(
            cur_x=0.0, cur_y=10.0,
            home_x=0.0, home_y=0.0,
            hose_charged=True,
            hose_remaining_m=100.0,
            hose_kink_risk=0.0,
        )
        # target(0,5): 홈에 더 가까워짐 → 후진. 각도 0도 → 급커브 아님.
        ok, reason = checker.check_hose_constraints(0.0, 5.0)
        assert ok is False
        assert '충수 상태 후진' in reason

    def test_uncharged_reverse_allowed(self):
        """건수(미충수) 상태 + 후진 → 호스 후진 제약 없음.

        U턴이 아닌 단순 후진 (cur 북쪽에서 남쪽으로 소폭 후퇴).
        """
        checker = _MockSherpaHoseChecker(
            cur_x=0.0, cur_y=10.0,
            home_x=0.0, home_y=0.0,
            hose_charged=False,   # 건수
            hose_remaining_m=100.0,
            hose_kink_risk=0.0,
        )
        # 후진 방향이지만 건수이므로 허용 (급커브 아님, 깊이 충분)
        ok, _ = checker.check_hose_constraints(0.0, 5.0)
        assert ok is True

    def test_charged_sideways_allowed(self):
        """충수 상태 + 측방 이동(홈과 동일 거리) → 통과."""
        # cur(10,0): 홈(0,0) 기준 거리 = 10
        # target(0,10): 홈 기준 거리 = 10 (동일 → 후진 아님)
        checker = _MockSherpaHoseChecker(
            cur_x=10.0, cur_y=0.0,
            home_x=0.0, home_y=0.0,
            hose_charged=True,
            hose_remaining_m=100.0,
        )
        ok, _ = checker.check_hose_constraints(0.0, 10.0)
        assert ok is True

    def test_charged_slight_retreat_rejected(self):
        """충수 상태 + 소폭 후진(1m) → 거부.

        cur(0,20): 북쪽 20m 진입. target(0,19): 1m 후퇴 (직선, U턴 아님).
        """
        checker = _MockSherpaHoseChecker(
            cur_x=0.0, cur_y=20.0,
            home_x=0.0, home_y=0.0,
            hose_charged=True,
            hose_remaining_m=100.0,
            hose_kink_risk=0.0,
        )
        # 목표(0,19): 홈(0,0) 기준 거리 19 < 현재 20 → 후진. 각도=0 → 급커브 아님.
        ok, reason = checker.check_hose_constraints(0.0, 19.0)
        assert ok is False
        assert '충수 상태 후진' in reason


# ──────────────────────────────────────────────────────────────────────────────
# 4. 오프라인 모드 (호스 토픽 없음)
# ──────────────────────────────────────────────────────────────────────────────

class TestHoseOfflineMode:
    """호스 토픽 미수신 또는 offline 모드 — graceful 동작."""

    def test_offline_mode_all_pass(self):
        """hose_mode=offline: 모든 이동 허용 (호스 제약 우회)."""
        checker = _MockSherpaHoseChecker(
            hose_mode='offline',
            hose_topic_received=False,
            hose_remaining_m=0.0,   # 극단값이어도
            hose_charged=True,
        )
        # 어떤 이동도 호스 제약으로 거부되지 않음
        ok, reason = checker.check_hose_constraints(200.0, 0.0)
        assert ok is True
        assert reason == ''

    def test_active_mode_no_topic_passes(self):
        """active 모드이지만 토픽 미수신 → 통과 (offline 간주)."""
        checker = _MockSherpaHoseChecker(
            hose_mode='active',
            hose_topic_received=False,  # 토픽 아직 안 옴
            hose_remaining_m=5.0,
        )
        ok, reason = checker.check_hose_constraints(100.0, 0.0)
        assert ok is True
        assert reason == ''

    def test_active_mode_with_topic_applies_constraints(self):
        """active 모드 + 토픽 수신 → 제약 적용."""
        checker = _MockSherpaHoseChecker(
            hose_mode='active',
            hose_topic_received=True,
            cur_x=0.0, cur_y=0.0,
            hose_remaining_m=10.0,
        )
        # 20m 목표, 호스 10m → 거부
        ok, reason = checker.check_hose_constraints(20.0, 0.0)
        assert ok is False
        assert '진입 깊이' in reason


# ──────────────────────────────────────────────────────────────────────────────
# 5. Capabilities — 셰르파 extra 필드
# ──────────────────────────────────────────────────────────────────────────────

class TestSherpaCapabilities:
    """셰르파 특화 능력 명세 검증."""

    def _make_caps(self, battery: float = 80.0, hose_total: float = 100.0) -> RobotCapabilities:
        """테스트용 셰르파 capabilities 생성."""
        return RobotCapabilities(
            can_drive=True,
            can_fly=False,
            has_thermal=True,
            has_lidar=True,
            max_speed=1.4,
            battery_capacity=battery,
            platform_type='sherpa',
            extra={
                'hose_length_m': hose_total,
                'hose_diameter_mm': 65,
                'max_flow_lpm': 2650,
                'max_pressure_bar': 35,
                'water_curtain_nozzles': 24,
                'thermal_dual': True,
                'weight_kg': 2000,
                'wheel_count': 6,
                'hose_remaining_m': hose_total,
                'hose_charged': False,
            }
        )

    def test_platform_type_sherpa(self):
        """platform_type이 'sherpa'여야 한다."""
        caps = self._make_caps()
        assert caps.platform_type == 'sherpa'

    def test_can_drive_true(self):
        """셰르파는 지상 주행 가능."""
        caps = self._make_caps()
        assert caps.can_drive is True

    def test_can_fly_false(self):
        """셰르파는 비행 불가."""
        caps = self._make_caps()
        assert caps.can_fly is False

    def test_has_thermal_and_lidar(self):
        """열화상 + LiDAR 탑재 확인."""
        caps = self._make_caps()
        assert caps.has_thermal is True
        assert caps.has_lidar is True

    def test_max_speed_1_4_mps(self):
        """최대 속도 1.4 m/s (5kph)."""
        caps = self._make_caps()
        assert caps.max_speed == pytest.approx(1.4)

    def test_extra_hose_length(self):
        """호스 총 길이 100m."""
        caps = self._make_caps()
        assert caps.extra['hose_length_m'] == pytest.approx(100.0)

    def test_extra_hose_diameter(self):
        """호스 구경 65mm."""
        caps = self._make_caps()
        assert caps.extra['hose_diameter_mm'] == 65

    def test_extra_max_flow(self):
        """최대 방수량 2650 L/min."""
        caps = self._make_caps()
        assert caps.extra['max_flow_lpm'] == 2650

    def test_extra_max_pressure(self):
        """최대 압력 35 bar."""
        caps = self._make_caps()
        assert caps.extra['max_pressure_bar'] == 35

    def test_extra_water_curtain_nozzles(self):
        """수막 노즐 24개."""
        caps = self._make_caps()
        assert caps.extra['water_curtain_nozzles'] == 24

    def test_extra_thermal_dual(self):
        """SWIR + LWIR 이중 열화상."""
        caps = self._make_caps()
        assert caps.extra['thermal_dual'] is True

    def test_extra_weight_and_wheels(self):
        """차량 중량 2000kg, 6륜 구동."""
        caps = self._make_caps()
        assert caps.extra['weight_kg'] == 2000
        assert caps.extra['wheel_count'] == 6

    def test_orchestrator_can_assign_hose_mission(self):
        """오케스트레이터가 extra['hose_length_m']으로 호스 임무 할당 판단 가능."""
        caps = self._make_caps()
        # 오케스트레이터 로직 시뮬레이션: 호스 50m 이상 필요 임무
        required_hose = 50.0
        can_do = caps.extra.get('hose_length_m', 0.0) >= required_hose
        assert can_do is True

    def test_orchestrator_rejects_insufficient_hose(self):
        """호스 30m만 있을 때 50m 필요 임무 → 거부."""
        caps = self._make_caps(hose_total=30.0)
        required_hose = 50.0
        can_do = caps.extra.get('hose_length_m', 0.0) >= required_hose
        assert can_do is False


# ──────────────────────────────────────────────────────────────────────────────
# 6. 배터리 모델 (20kWh / 5시간)
# ──────────────────────────────────────────────────────────────────────────────

class TestSherpaBatteryModel:
    """셰르파 배터리 소비 모델 검증."""

    def test_drain_rate_approx(self):
        """drain_rate ≈ 0.333 %/분 (100% / 300분)."""
        expected = 100.0 / (5.0 * 60.0)
        assert _SHERPA_DRAIN_RATE_PCT_PER_MIN == pytest.approx(expected)

    def test_full_endurance_depletes_to_zero(self):
        """300분(5시간) 이동 → 배터리 완전 소진."""
        result = simulate_sherpa_battery(100.0, elapsed_min=300.0, moving=True)
        assert result == pytest.approx(0.0)

    def test_idle_lower_drain_than_moving(self):
        """정지 중 소비 < 이동 중 소비."""
        moving = simulate_sherpa_battery(100.0, 60.0, moving=True)
        idle = simulate_sherpa_battery(100.0, 60.0, moving=False)
        assert idle > moving

    def test_idle_factor_025(self):
        """정지 소비율: 이동의 25%."""
        moving = 100.0 - simulate_sherpa_battery(100.0, 60.0, moving=True)
        idle = 100.0 - simulate_sherpa_battery(100.0, 60.0, moving=False)
        assert idle == pytest.approx(moving * _SHERPA_IDLE_FACTOR)

    def test_battery_floor_zero(self):
        """배터리는 0% 미만으로 내려가지 않는다."""
        result = simulate_sherpa_battery(1.0, elapsed_min=1000.0, moving=True)
        assert result == pytest.approx(0.0)

    def test_no_drain_elapsed_zero(self):
        """경과 시간 0 → 소비 없음."""
        result = simulate_sherpa_battery(80.0, elapsed_min=0.0, moving=True)
        assert result == pytest.approx(80.0)

    def test_1hour_moving_drain(self):
        """1시간(60분) 이동 → 20% 소진 (100% / 5시간)."""
        result = simulate_sherpa_battery(100.0, elapsed_min=60.0, moving=True)
        assert result == pytest.approx(80.0)

    def test_battery_warning_threshold(self):
        """배터리 경고 임계값: 20% 이하."""
        # 4시간(240분) 이동 후 배터리
        result = simulate_sherpa_battery(100.0, elapsed_min=240.0, moving=True)
        assert result == pytest.approx(20.0)
        # 경고 조건 확인
        assert result <= 20.0

    def test_battery_stop_threshold(self):
        """이동 거부 임계값: 10% 미만 (275분 이동 → 8.33% 남음)."""
        result = simulate_sherpa_battery(100.0, elapsed_min=275.0, moving=True)
        assert result < 10.0


# ──────────────────────────────────────────────────────────────────────────────
# 7. 좌표 연산 함수 단위 검증
# ──────────────────────────────────────────────────────────────────────────────

class TestCoordinateUtils:
    """좌표 연산 유틸리티 함수 검증."""

    def test_calc_distance_origin(self):
        """원점 사이 거리 = 0."""
        assert calc_distance(0.0, 0.0, 0.0, 0.0) == pytest.approx(0.0)

    def test_calc_distance_3_4_5(self):
        """3-4-5 직각삼각형 거리 = 5."""
        assert calc_distance(0.0, 0.0, 3.0, 4.0) == pytest.approx(5.0)

    def test_calc_distance_symmetric(self):
        """거리는 방향 무관 (대칭)."""
        d1 = calc_distance(1.0, 2.0, 5.0, 6.0)
        d2 = calc_distance(5.0, 6.0, 1.0, 2.0)
        assert d1 == pytest.approx(d2)

    def test_calc_turn_angle_straight(self):
        """직진(꺾임 없음) → 각도 0도."""
        angle = calc_turn_angle(0.0, 0.0, 5.0, 0.0, 10.0, 0.0)
        assert angle == pytest.approx(0.0)

    def test_calc_turn_angle_uturn(self):
        """U턴(완전 반전) → 각도 180도."""
        angle = calc_turn_angle(0.0, 0.0, 5.0, 0.0, 0.0, 0.0)
        assert angle == pytest.approx(180.0)

    def test_calc_turn_angle_90deg(self):
        """직각 꺾임 → 각도 90도."""
        # (0,0) → (5,0) → (5,5): 90도 꺾임
        angle = calc_turn_angle(0.0, 0.0, 5.0, 0.0, 5.0, 5.0)
        assert angle == pytest.approx(90.0)

    def test_calc_turn_angle_same_points(self):
        """동일 점 → 0도 반환 (벡터 길이 0 방어)."""
        angle = calc_turn_angle(0.0, 0.0, 0.0, 0.0, 5.0, 0.0)
        assert angle == pytest.approx(0.0)

    def test_requires_reverse_true(self):
        """목표가 홈에 더 가까우면 후진."""
        # home(0,0), cur(10,0), target(5,0): target이 홈에 더 가까움
        assert requires_reverse(0.0, 0.0, 10.0, 0.0, 5.0, 0.0) is True

    def test_requires_reverse_false(self):
        """목표가 홈에서 더 멀면 전진."""
        # home(0,0), cur(10,0), target(20,0): target이 홈에서 더 멀어짐
        assert requires_reverse(0.0, 0.0, 10.0, 0.0, 20.0, 0.0) is False

    def test_requires_reverse_equidistant_false(self):
        """목표와 현재가 홈에서 동일 거리 → 후진 아님."""
        # home(0,0), cur(10,0), target(0,10): 둘 다 홈에서 10m
        assert requires_reverse(0.0, 0.0, 10.0, 0.0, 0.0, 10.0) is False


# ──────────────────────────────────────────────────────────────────────────────
# 8. 복합 시나리오 (실제 소방 현장 시뮬레이션)
# ──────────────────────────────────────────────────────────────────────────────

class TestSherpaScenariosFireScene:
    """소방 현장 복합 시나리오."""

    def test_scenario_deep_penetration(self):
        """시나리오: 70m 진입 후 추가 40m 이동 시도 → 호스 부족으로 거부.

        셰르파가 이미 70m 진입 후 (잔여 30m), 40m 더 들어가려는 상황.
        """
        checker = _MockSherpaHoseChecker(
            cur_x=70.0, cur_y=0.0,
            home_x=0.0, home_y=0.0,
            hose_remaining_m=30.0,   # 이미 70m 호스 풀림
            hose_charged=False,
        )
        ok, reason = checker.check_hose_constraints(110.0, 0.0)  # 40m 더
        assert ok is False
        assert '진입 깊이' in reason

    def test_scenario_water_attack_advance(self):
        """시나리오: 충수 상태로 화재 방향 전진 → 허용.

        방수 개시 후 화재 방향으로 계속 전진하는 정상 운용.
        """
        checker = _MockSherpaHoseChecker(
            cur_x=30.0, cur_y=0.0,
            home_x=0.0, home_y=0.0,
            hose_remaining_m=80.0,
            hose_charged=True,       # 방수 중
            hose_kink_risk=0.1,
        )
        ok, _ = checker.check_hose_constraints(40.0, 5.0)  # 전진
        assert ok is True

    def test_scenario_emergency_retreat_blocked(self):
        """시나리오: 충수 상태 긴급 후퇴 시도 → 제약으로 거부.

        실제 운용에서는 emergency_stop 후 인력이 호스를 처리해야 함.
        cur(0,50): 북쪽 50m 진입 후 충수 완료. target(0,20): 직선 30m 후퇴.
        직선 경로이므로 U턴 각도가 0도 → 급커브 아님 → 충수 후진 제약에서 정확히 거부.
        """
        checker = _MockSherpaHoseChecker(
            cur_x=0.0, cur_y=50.0,
            home_x=0.0, home_y=0.0,
            hose_remaining_m=50.0,
            hose_charged=True,
            hose_kink_risk=0.0,
        )
        # 목표(0,20): 홈에 더 가까워짐 → 후진 거부
        ok, reason = checker.check_hose_constraints(0.0, 20.0)
        assert ok is False
        assert '충수 상태 후진' in reason

    def test_scenario_kink_during_navigation(self):
        """시나리오: 내비게이션 중 호스 꺾임 감지 → 이동 거부.

        /hose/status가 꺾임 위험 0.8을 보고하는 상황.
        """
        checker = _MockSherpaHoseChecker(
            cur_x=20.0, cur_y=0.0,
            home_x=0.0, home_y=0.0,
            hose_remaining_m=80.0,
            hose_kink_risk=0.8,      # 꺾임 위험 높음
        )
        ok, reason = checker.check_hose_constraints(30.0, 0.0)
        assert ok is False
        assert '급커브' in reason

    def test_scenario_all_constraints_clear(self):
        """시나리오: 모든 제약 클리어 — 정상 이동 허용.

        충수 미개시, 꺾임 안전, 호스 충분한 초기 진입.
        """
        checker = _MockSherpaHoseChecker(
            cur_x=10.0, cur_y=0.0,
            home_x=0.0, home_y=0.0,
            hose_remaining_m=100.0,
            hose_kink_risk=0.1,
            hose_charged=False,
        )
        ok, reason = checker.check_hose_constraints(20.0, 2.0)
        assert ok is True
        assert reason == ''
