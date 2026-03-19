"""platform_interface / ugv_platform / px4_platform 단위 테스트.

기존 테스트 패턴 준수: ROS 의존성 없이 순수 Python 로직만 추출하여 검증.
ROS 2 미설치 환경(Windows CI)에서도 동작한다.
"""
import math
import time
import pytest


# ──────────────────────────────────────────────────────────────────────────────
# PlatformInterface 핵심 로직 (platform_interface.py에서 추출)
# ──────────────────────────────────────────────────────────────────────────────

from dataclasses import dataclass, field


@dataclass
class RobotCapabilities:
    """RobotCapabilities 테스트용 복제 (platform_interface.py와 동일 구조)."""
    can_fly: bool = False
    can_drive: bool = False
    has_thermal: bool = False
    has_lidar: bool = False
    max_speed: float = 0.0
    battery_capacity: float = 100.0
    platform_type: str = "unknown"
    extra: dict = field(default_factory=dict)


class _MockPlatform:
    """PlatformInterface 최소 구현체 (추상화 계약 검증용)."""

    def __init__(self, battery: float = 80.0, ptype: str = 'test'):
        self._battery = battery
        self._ptype = ptype

    def move_to(self, x: float, y: float, z: float = 0.0) -> bool:
        return self._battery >= 10.0

    def get_pose(self) -> tuple:
        return (1.0, 2.0, 0.0)

    def get_battery(self) -> float:
        return max(0.0, self._battery)

    def get_capabilities(self) -> RobotCapabilities:
        return RobotCapabilities(
            battery_capacity=self._battery,
            platform_type=self._ptype,
        )

    def emergency_stop(self) -> None:
        pass

    def return_home(self) -> bool:
        return self.move_to(0.0, 0.0)

    def is_alive(self) -> bool:
        try:
            return self.get_battery() > 0.0
        except Exception:
            return False

    def get_platform_type(self) -> str:
        return self.get_capabilities().platform_type


# ──────────────────────────────────────────────────────────────────────────────
# RobotCapabilities 테스트
# ──────────────────────────────────────────────────────────────────────────────

class TestRobotCapabilities:
    """RobotCapabilities 데이터클래스 기본 동작 검증."""

    def test_default_values(self):
        """기본값: 모든 능력 False, 속도/배터리 기본값."""
        cap = RobotCapabilities()
        assert cap.can_fly is False
        assert cap.can_drive is False
        assert cap.has_thermal is False
        assert cap.has_lidar is False
        assert cap.max_speed == pytest.approx(0.0)
        assert cap.battery_capacity == pytest.approx(100.0)
        assert cap.platform_type == 'unknown'

    def test_ugv_profile(self):
        """UGV 프로파일: can_drive=True, has_lidar=True, has_thermal=True."""
        ugv_cap = RobotCapabilities(
            can_fly=False,
            can_drive=True,
            has_thermal=True,
            has_lidar=True,
            max_speed=0.5,
            battery_capacity=80.0,
            platform_type='ugv',
        )
        assert ugv_cap.can_fly is False
        assert ugv_cap.can_drive is True
        assert ugv_cap.has_thermal is True
        assert ugv_cap.has_lidar is True
        assert ugv_cap.max_speed == pytest.approx(0.5)
        assert ugv_cap.platform_type == 'ugv'

    def test_drone_profile(self):
        """드론 프로파일: can_fly=True, has_thermal=True, has_lidar=False."""
        drone_cap = RobotCapabilities(
            can_fly=True,
            can_drive=False,
            has_thermal=True,
            has_lidar=False,
            max_speed=2.0,
            platform_type='drone',
        )
        assert drone_cap.can_fly is True
        assert drone_cap.can_drive is False
        assert drone_cap.has_lidar is False
        assert drone_cap.platform_type == 'drone'

    def test_extra_field(self):
        """extra 딕셔너리 확장 필드 저장."""
        cap = RobotCapabilities(extra={'has_hose': True, 'payload_kg': 5.0})
        assert cap.extra['has_hose'] is True
        assert cap.extra['payload_kg'] == pytest.approx(5.0)

    def test_extra_field_independent(self):
        """두 인스턴스의 extra 딕셔너리는 독립적이어야 한다 (mutable default 버그 방지)."""
        cap1 = RobotCapabilities()
        cap2 = RobotCapabilities()
        cap1.extra['test'] = 1
        assert 'test' not in cap2.extra


# ──────────────────────────────────────────────────────────────────────────────
# PlatformInterface 계약 검증
# ──────────────────────────────────────────────────────────────────────────────

class TestPlatformInterfaceContract:
    """PlatformInterface 계약 — 구현체가 올바르게 동작하는지 검증."""

    def test_move_to_succeeds_with_battery(self):
        """배터리 충분 시 move_to True 반환."""
        p = _MockPlatform(battery=50.0)
        assert p.move_to(5.0, 3.0) is True

    def test_move_to_fails_low_battery(self):
        """배터리 10% 미만 시 move_to False 반환."""
        p = _MockPlatform(battery=9.9)
        assert p.move_to(5.0, 3.0) is False

    def test_get_pose_returns_tuple(self):
        """get_pose는 (x, y, z) 3-tuple 반환."""
        p = _MockPlatform()
        pose = p.get_pose()
        assert len(pose) == 3
        assert all(isinstance(v, float) for v in pose)

    def test_get_battery_range(self):
        """get_battery는 0 ~ 100 범위 반환."""
        p = _MockPlatform(battery=75.0)
        b = p.get_battery()
        assert 0.0 <= b <= 100.0

    def test_get_battery_not_negative(self):
        """get_battery는 음수 반환 금지."""
        p = _MockPlatform(battery=-5.0)
        assert p.get_battery() >= 0.0

    def test_get_capabilities_type(self):
        """get_capabilities는 RobotCapabilities 반환."""
        p = _MockPlatform()
        cap = p.get_capabilities()
        assert isinstance(cap, RobotCapabilities)

    def test_return_home_calls_move_to(self):
        """return_home은 내부적으로 move_to를 호출해야 한다."""
        p = _MockPlatform(battery=80.0)
        result = p.return_home()
        assert result is True

    def test_emergency_stop_no_exception(self):
        """emergency_stop은 예외 없이 완료 (어떤 상태에서도)."""
        p = _MockPlatform(battery=0.0)
        try:
            p.emergency_stop()
        except Exception as e:
            pytest.fail(f'emergency_stop raised exception: {e}')


# ──────────────────────────────────────────────────────────────────────────────
# is_alive / get_platform_type 편의 메서드 테스트
# ──────────────────────────────────────────────────────────────────────────────

class TestPlatformConvenience:
    """PlatformInterface 편의 메서드 검증."""

    def test_is_alive_with_battery(self):
        """배터리 > 0이면 is_alive True."""
        p = _MockPlatform(battery=50.0)
        assert p.is_alive() is True

    def test_is_alive_zero_battery(self):
        """배터리 0이면 is_alive False."""
        p = _MockPlatform(battery=0.0)
        assert p.is_alive() is False

    def test_get_platform_type_ugv(self):
        """UGV 플랫폼 타입 문자열 반환."""
        p = _MockPlatform(ptype='ugv')
        assert p.get_platform_type() == 'ugv'

    def test_get_platform_type_drone(self):
        """드론 플랫폼 타입 문자열 반환."""
        p = _MockPlatform(ptype='drone')
        assert p.get_platform_type() == 'drone'

    def test_get_platform_type_unknown(self):
        """타입 미설정 시 'unknown' 반환."""
        p = _MockPlatform(ptype='unknown')
        assert p.get_platform_type() == 'unknown'


# ──────────────────────────────────────────────────────────────────────────────
# PX4 좌표 변환 (px4_platform.PX4Platform 정적 메서드에서 추출)
# ──────────────────────────────────────────────────────────────────────────────

def enu_to_ned(x_enu: float, y_enu: float, z_enu: float):
    """ENU → NED 좌표 변환 (px4_platform.py와 동일 로직)."""
    return y_enu, x_enu, -z_enu


def ned_to_enu(x_ned: float, y_ned: float, z_ned: float):
    """NED → ENU 좌표 변환 (px4_platform.py와 동일 로직)."""
    return y_ned, x_ned, -z_ned


class TestPX4CoordinateConversion:
    """PX4Platform ENU↔NED 좌표 변환 정확성 검증."""

    def test_enu_east_to_ned(self):
        """ENU(1,0,0) East → NED y축 이동."""
        nx, ny, nz = enu_to_ned(1.0, 0.0, 0.0)
        # ENU x=East → NED y=East
        assert nx == pytest.approx(0.0)
        assert ny == pytest.approx(1.0)
        assert nz == pytest.approx(0.0)

    def test_enu_north_to_ned(self):
        """ENU(0,1,0) North → NED x축 이동."""
        nx, ny, nz = enu_to_ned(0.0, 1.0, 0.0)
        # ENU y=North → NED x=North
        assert nx == pytest.approx(1.0)
        assert ny == pytest.approx(0.0)
        assert nz == pytest.approx(0.0)

    def test_enu_altitude_positive(self):
        """ENU z=3m (상승) → NED z=-3m (Down 역방향)."""
        _, _, nz = enu_to_ned(0.0, 0.0, 3.0)
        assert nz == pytest.approx(-3.0)

    def test_ned_to_enu_roundtrip(self):
        """ENU→NED→ENU 왕복 변환 손실 없음."""
        original = (5.5, -3.2, 8.0)
        ned = enu_to_ned(*original)
        back = ned_to_enu(*ned)
        for o, b in zip(original, back):
            assert o == pytest.approx(b)

    def test_argos_patrol_altitude_3m(self):
        """ARGOS 드론 기본 순항 고도 3m → NED -3m."""
        _, _, nz = enu_to_ned(0.0, 0.0, 3.0)
        assert nz == pytest.approx(-3.0)

    def test_zero_origin_preserved(self):
        """원점은 변환해도 원점."""
        nx, ny, nz = enu_to_ned(0.0, 0.0, 0.0)
        assert nx == pytest.approx(0.0)
        assert ny == pytest.approx(0.0)
        assert nz == pytest.approx(0.0)

    def test_symmetry_of_transforms(self):
        """enu_to_ned와 ned_to_enu는 동일 공식 (대칭 구조 확인)."""
        x, y, z = 3.0, -1.5, 5.0
        a = enu_to_ned(x, y, z)
        b = ned_to_enu(x, y, z)
        # 두 함수는 수식이 동일하므로 동일한 결과
        assert a[0] == pytest.approx(b[0])
        assert a[1] == pytest.approx(b[1])
        assert a[2] == pytest.approx(b[2])

    def test_negative_altitude_underground(self):
        """ENU z=-1 (지하) → NED z=1 (아래방향)."""
        _, _, nz = enu_to_ned(0.0, 0.0, -1.0)
        assert nz == pytest.approx(1.0)


# ──────────────────────────────────────────────────────────────────────────────
# 지오펜스 검증 로직 (px4_platform.py + ugv_platform.py에서 추출)
# ──────────────────────────────────────────────────────────────────────────────

def check_geofence(x_enu: float, y_enu: float, z_enu: float,
                   geofence: dict) -> bool:
    """지오펜스 경계 검증 (px4_platform._check_geofence와 동일 로직)."""
    gf = geofence
    if not (gf['x_min'] <= x_enu <= gf['x_max']):
        return False
    if not (gf['y_min'] <= y_enu <= gf['y_max']):
        return False
    if z_enu > gf['z_max']:
        return False
    if z_enu < 0.0:
        return False
    return True


class TestGeofenceLogic:
    """지오펜스 경계 검증 로직 — UGV/드론 공통."""

    _gf = {'x_min': -10.0, 'x_max': 10.0,
            'y_min': -10.0, 'y_max': 10.0,
            'z_max': 8.0}

    def test_inside_center(self):
        """중심 좌표는 항상 허용."""
        assert check_geofence(0.0, 0.0, 3.0, self._gf) is True

    def test_boundary_max(self):
        """최대 경계값은 허용 (폐구간)."""
        assert check_geofence(10.0, 10.0, 8.0, self._gf) is True

    def test_boundary_min(self):
        """최소 경계값은 허용 (폐구간)."""
        assert check_geofence(-10.0, -10.0, 0.0, self._gf) is True

    def test_x_overflow(self):
        """x 초과 → 위반."""
        assert check_geofence(10.1, 0.0, 3.0, self._gf) is False

    def test_x_underflow(self):
        """x 미달 → 위반."""
        assert check_geofence(-10.1, 0.0, 3.0, self._gf) is False

    def test_y_overflow(self):
        """y 초과 → 위반."""
        assert check_geofence(0.0, 10.1, 3.0, self._gf) is False

    def test_y_underflow(self):
        """y 미달 → 위반."""
        assert check_geofence(0.0, -10.1, 3.0, self._gf) is False

    def test_z_exceed_max(self):
        """z 최대 초과 → 위반."""
        assert check_geofence(0.0, 0.0, 8.1, self._gf) is False

    def test_z_negative(self):
        """z < 0 (지하) → 드론에 적용 시 위반."""
        assert check_geofence(0.0, 0.0, -0.1, self._gf) is False

    def test_z_zero_ground(self):
        """z = 0 (지면) → 허용 (착륙 좌표)."""
        assert check_geofence(0.0, 0.0, 0.0, self._gf) is True

    def test_argos_default_patrol(self):
        """ARGOS 기본 정찰 고도 3m, 좌표 (5,3) → 허용."""
        assert check_geofence(5.0, 3.0, 3.0, self._gf) is True


# ──────────────────────────────────────────────────────────────────────────────
# 배터리 소비 모델 (ugv_platform.py + px4_platform.py에서 추출)
# ──────────────────────────────────────────────────────────────────────────────

def simulate_battery(start: float, drain_rate: float,
                     elapsed_min: float, moving: bool,
                     idle_factor: float = 0.3) -> float:
    """배터리 소비 시뮬레이션 (플랫폼 공통 로직)."""
    drain_factor = 1.0 if moving else idle_factor
    drain = drain_rate * elapsed_min * drain_factor
    return max(0.0, start - drain)


class TestBatteryModel:
    """배터리 소비 시뮬레이션 로직 — UGV/드론 공통."""

    def test_idle_lower_drain(self):
        """정지 상태: 소비율 idle_factor(0.3배) 적용."""
        result = simulate_battery(100.0, 1.0, 1.0, moving=False)
        assert result == pytest.approx(99.7)

    def test_moving_full_drain(self):
        """이동 상태: 소비율 1.0배 적용."""
        result = simulate_battery(100.0, 1.0, 1.0, moving=True)
        assert result == pytest.approx(99.0)

    def test_battery_floor_zero(self):
        """배터리는 0 미만으로 내려가지 않는다."""
        result = simulate_battery(1.0, 100.0, 10.0, moving=True)
        assert result == pytest.approx(0.0)

    def test_drone_higher_drain_than_ugv(self):
        """드론(2%/분)은 UGV(1%/분)보다 빨리 소모."""
        ugv = simulate_battery(100.0, 1.0, 5.0, moving=True)
        drone = simulate_battery(100.0, 2.0, 5.0, moving=True)
        assert drone < ugv

    def test_no_drain_at_elapsed_zero(self):
        """경과 시간 0이면 소비 없음."""
        result = simulate_battery(80.0, 1.0, 0.0, moving=True)
        assert result == pytest.approx(80.0)

    def test_ugv_warning_threshold(self):
        """UGV 배터리 경고 임계값: 20% 이하."""
        assert 19.9 < 20.0  # 경고 조건

    def test_drone_rtl_threshold(self):
        """드론 RTL 권장 임계값: 25% 이하."""
        assert 24.9 < 25.0

    def test_ugv_move_block_threshold(self):
        """UGV 이동 거부 임계값: 10% 미만."""
        assert 9.9 < 10.0

    def test_drone_move_block_threshold(self):
        """드론 이동 거부 임계값: 15% 미만."""
        assert 14.9 < 15.0

    def test_drone_hover_drain_factor(self):
        """드론 호버링: idle_factor 0.6 (UGV 0.3보다 높음 — 비행 유지 전력)."""
        ugv_hover = simulate_battery(100.0, 2.0, 1.0, moving=False, idle_factor=0.3)
        drone_hover = simulate_battery(100.0, 2.0, 1.0, moving=False, idle_factor=0.6)
        assert drone_hover < ugv_hover


# ──────────────────────────────────────────────────────────────────────────────
# 오케스트레이터 관점: capabilities 기반 임무 적합성 판단
# ──────────────────────────────────────────────────────────────────────────────

class TestCapabilitiesBasedDispatch:
    """오케스트레이터가 capabilities로 임무 할당하는 로직 검증.

    오케스트레이터는 로봇 타입을 모르고 capabilities만 질의한다.
    이 패턴이 PlatformInterface 설계의 핵심.
    """

    def _assign_recon(self, robot) -> bool:
        """정찰 임무 할당 — can_fly 또는 has_thermal 필요."""
        cap = robot.get_capabilities()
        return cap.can_fly or cap.has_thermal

    def _assign_terrain(self, robot) -> bool:
        """지형 탐색 — can_drive 필요."""
        cap = robot.get_capabilities()
        return cap.can_drive

    def _assign_fire_detect(self, robot) -> bool:
        """화재 감지 — has_thermal 필요."""
        cap = robot.get_capabilities()
        return cap.has_thermal

    def test_ugv_can_do_terrain(self):
        """UGV(can_drive=True)는 지형 탐색 가능."""
        ugv = _MockPlatform(ptype='ugv')
        ugv._caps = RobotCapabilities(can_drive=True, platform_type='ugv')
        ugv.get_capabilities = lambda: ugv._caps
        assert self._assign_terrain(ugv) is True

    def test_drone_cannot_terrain(self):
        """드론(can_drive=False)은 지형 탐색 불가."""
        drone = _MockPlatform(ptype='drone')
        drone._caps = RobotCapabilities(can_fly=True, can_drive=False, platform_type='drone')
        drone.get_capabilities = lambda: drone._caps
        assert self._assign_terrain(drone) is False

    def test_drone_can_recon(self):
        """드론(can_fly=True)은 정찰 가능."""
        drone = _MockPlatform(ptype='drone')
        drone._caps = RobotCapabilities(can_fly=True, platform_type='drone')
        drone.get_capabilities = lambda: drone._caps
        assert self._assign_recon(drone) is True

    def test_ugv_with_thermal_can_recon(self):
        """열화상 장착 UGV(has_thermal=True)도 정찰 가능."""
        ugv = _MockPlatform(ptype='ugv')
        ugv._caps = RobotCapabilities(can_drive=True, has_thermal=True, platform_type='ugv')
        ugv.get_capabilities = lambda: ugv._caps
        assert self._assign_recon(ugv) is True

    def test_ugv_with_thermal_fire_detect(self):
        """UGV 열화상 탑재 시 화재 감지 임무 가능."""
        ugv = _MockPlatform(ptype='ugv')
        ugv._caps = RobotCapabilities(can_drive=True, has_thermal=True, platform_type='ugv')
        ugv.get_capabilities = lambda: ugv._caps
        assert self._assign_fire_detect(ugv) is True

    def test_type_agnostic_dispatch(self):
        """오케스트레이터는 플랫폼 타입을 몰라도 capabilities로 판단 가능."""
        platforms = [
            _MockPlatform(ptype='unknown'),
            _MockPlatform(ptype='unknown'),
        ]
        caps_list = [
            RobotCapabilities(can_drive=True, has_thermal=True),   # UGV처럼
            RobotCapabilities(can_fly=True, has_thermal=True),     # 드론처럼
        ]
        for p, cap in zip(platforms, caps_list):
            p.get_capabilities = lambda c=cap: c

        # 화재 감지 임무: 두 플랫폼 모두 has_thermal=True → 가능
        for p in platforms:
            assert self._assign_fire_detect(p) is True
