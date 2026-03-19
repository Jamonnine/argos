"""keepout_manager.py 핵심 로직 단위 테스트 (ROS2 없이 실행 가능).

ROS2 Node 생성 없이 내부 로직만 분리 테스트합니다.
- zone 추가/중복 방지/만료 제거 로직
- OccupancyGrid 패치 셀 계산 로직
"""
import math
import time
import sys
from pathlib import Path

# ROS2 없이 실행하기 위해 mock 모듈 주입
import types

# ── ROS2 stub 모듈 (rclpy 없이 로직 테스트) ──
def _make_stub_rclpy():
    rclpy_mod = types.ModuleType('rclpy')
    node_mod = types.ModuleType('rclpy.node')

    class FakeNode:
        def __init__(self, *a, **kw):
            pass
        def declare_parameter(self, name, value):
            pass
        def get_parameter(self, name):
            class P:
                def __init__(self, v):
                    self.value = v
            return P(0.0)
        def get_clock(self):
            class Clock:
                def now(self):
                    class T:
                        def to_msg(self):
                            return None
                    return T()
            return Clock()
        def get_logger(self):
            class L:
                def info(self, *a, **kw): pass
                def warn(self, *a, **kw): pass
                def error(self, *a, **kw): pass
                def debug(self, *a, **kw): pass
            return L()
        def create_subscription(self, *a, **kw): pass
        def create_publisher(self, *a, **kw): pass
        def create_timer(self, *a, **kw): pass

    node_mod.Node = FakeNode
    rclpy_mod.node = node_mod
    return rclpy_mod

sys.modules['rclpy'] = _make_stub_rclpy()
sys.modules['rclpy.node'] = sys.modules['rclpy'].node
for mod in ['rclpy.qos', 'nav_msgs', 'nav_msgs.msg',
            'std_msgs', 'std_msgs.msg', 'geometry_msgs',
            'geometry_msgs.msg', 'argos_interfaces',
            'argos_interfaces.msg']:
    sys.modules[mod] = types.ModuleType(mod)

# geometry_msgs.msg stub
gm = sys.modules['geometry_msgs.msg']
class FakePose:
    def __init__(self):
        self.position = type('P', (), {'x': 0.0, 'y': 0.0, 'z': 0.0})()
        self.orientation = type('Q', (), {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0})()
gm.Pose = FakePose

# nav_msgs.msg stub
nm = sys.modules['nav_msgs.msg']
class FakeOccupancyGrid:
    def __init__(self):
        self.header = type('H', (), {'stamp': None, 'frame_id': ''})()
        self.info = None
        self.data = []
nm.OccupancyGrid = FakeOccupancyGrid

class FakeMapMeta:
    def __init__(self):
        self.resolution = 0.05
        self.width = 0
        self.height = 0
        self.origin = FakePose()
nm.MapMetaData = FakeMapMeta

# std_msgs.msg stub
sm = sys.modules['std_msgs.msg']
class FakeString:
    def __init__(self):
        self.data = ''
sm.String = FakeString

# argos_interfaces.msg stub
ai = sys.modules['argos_interfaces.msg']
class FakeGasReading:
    pass
class FakeStructuralAlert:
    pass
ai.GasReading = FakeGasReading
ai.StructuralAlert = FakeStructuralAlert

# QoS stub
qos = sys.modules['rclpy.qos']
class FakeQoS:
    def __init__(self, **kw): pass
class FakeReliability:
    RELIABLE = 'RELIABLE'
class FakeDurability:
    TRANSIENT_LOCAL = 'TRANSIENT_LOCAL'
qos.QoSProfile = FakeQoS
qos.ReliabilityPolicy = FakeReliability
qos.DurabilityPolicy = FakeDurability

# ── 실제 로직 임포트 ──
sys.path.insert(0, str(Path(__file__).parent.parent / 'argos_bringup'))
from keepout_manager import KeeputZone, CAUSE_GAS, CAUSE_STRUCTURAL, LETHAL_COST, FREE_COST


# ─────────────────── 순수 로직 헬퍼 (Node 의존 없음) ───────────────────

def _zone_exists_nearby(zones, cx, cy, cause, radius_limit):
    """KeepoutManager._zone_exists_nearby 로직 추출."""
    for zone in zones.values():
        if zone.cause != cause:
            continue
        dist = math.sqrt((zone.center_x - cx)**2 + (zone.center_y - cy)**2)
        if dist < radius_limit:
            return True
    return False


def _remove_expired_zones(zones, now):
    """만료 zone 제거 로직 추출."""
    to_remove = []
    for zone_id, zone in zones.items():
        if now < zone.expires_at:
            continue
        if zone.cause == CAUSE_GAS:
            if zone.sensor_cleared:
                to_remove.append(zone_id)
        else:
            to_remove.append(zone_id)
    for zid in to_remove:
        zones.pop(zid)
    return to_remove


def _build_circle_cells(radius_m, resolution, map_size_m):
    """원형 keepout zone의 LETHAL_COST 셀 수를 계산."""
    import numpy as np
    cells = int(math.ceil(map_size_m / resolution))
    data = np.zeros(cells * cells, dtype=np.int8)
    radius_cells = int(math.ceil(radius_m / resolution))
    cx_cell = cells // 2
    cy_cell = cells // 2
    count = 0
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            if dx * dx + dy * dy > radius_cells * radius_cells:
                continue
            xi = cx_cell + dx
            yi = cy_cell + dy
            if 0 <= xi < cells and 0 <= yi < cells:
                data[yi * cells + xi] = LETHAL_COST
                count += 1
    return count, data


# ─────────────────── 테스트 ───────────────────

class TestKeeputZoneLogic:
    """Zone 추가·중복 방지·만료 로직."""

    def _make_zone(self, zone_id, cx, cy, cause=CAUSE_GAS,
                   expires_offset=1800.0, cleared=False) -> KeeputZone:
        now = time.time()
        return KeeputZone(
            zone_id=zone_id,
            cause=cause,
            center_x=cx,
            center_y=cy,
            radius_m=2.0,
            created_at=now,
            expires_at=now + expires_offset,
            sensor_cleared=cleared,
        )

    def test_no_zones_initially(self):
        zones = {}
        assert len(zones) == 0

    def test_add_zone(self):
        zones = {}
        zones[0] = self._make_zone(0, 5.0, 3.0)
        assert len(zones) == 1
        assert zones[0].cause == CAUSE_GAS

    def test_nearby_duplicate_prevention(self):
        """1.5m 이내 동일 원인 zone → 중복 방지."""
        zones = {0: self._make_zone(0, 5.0, 3.0)}
        # 1.5m 이내에 같은 원인 zone 존재 → True (추가 금지)
        assert _zone_exists_nearby(zones, 5.5, 3.5, CAUSE_GAS, radius_limit=2.0)

    def test_faraway_zone_allowed(self):
        """5m 이상 떨어진 같은 원인 zone → 추가 허용."""
        zones = {0: self._make_zone(0, 5.0, 3.0)}
        assert not _zone_exists_nearby(zones, 15.0, 3.0, CAUSE_GAS, radius_limit=2.0)

    def test_different_cause_no_block(self):
        """같은 위치라도 다른 원인 zone → 중복 방지 대상 아님."""
        zones = {0: self._make_zone(0, 5.0, 3.0, cause=CAUSE_GAS)}
        assert not _zone_exists_nearby(zones, 5.0, 3.0, CAUSE_STRUCTURAL, radius_limit=2.0)

    def test_gas_zone_not_removed_without_sensor_clear(self):
        """가스 zone: 시간 만료했지만 sensor_cleared=False → 제거 안 함."""
        zones = {0: self._make_zone(0, 5.0, 3.0, expires_offset=-1.0, cleared=False)}
        removed = _remove_expired_zones(zones, time.time())
        assert len(removed) == 0
        assert len(zones) == 1

    def test_gas_zone_removed_when_expired_and_cleared(self):
        """가스 zone: 시간 만료 + sensor_cleared=True → 제거."""
        zones = {0: self._make_zone(0, 5.0, 3.0, expires_offset=-1.0, cleared=True)}
        removed = _remove_expired_zones(zones, time.time())
        assert 0 in removed
        assert len(zones) == 0

    def test_structural_zone_removed_on_expiry(self):
        """구조물 zone: 시간 만료만으로 제거 (sensor_cleared 불필요)."""
        zones = {
            0: self._make_zone(0, 5.0, 3.0, cause=CAUSE_STRUCTURAL,
                               expires_offset=-1.0, cleared=False)
        }
        removed = _remove_expired_zones(zones, time.time())
        assert 0 in removed
        assert len(zones) == 0

    def test_not_expired_zone_kept(self):
        """만료 시간 미도달 zone → 유지."""
        zones = {0: self._make_zone(0, 5.0, 3.0, expires_offset=3600.0, cleared=True)}
        removed = _remove_expired_zones(zones, time.time())
        assert len(removed) == 0
        assert len(zones) == 1

    def test_multiple_zones_partial_removal(self):
        """여러 zone 중 일부만 만료 → 해당 것만 제거."""
        zones = {
            0: self._make_zone(0, 1.0, 1.0, expires_offset=-1.0, cleared=True),
            1: self._make_zone(1, 5.0, 5.0, expires_offset=3600.0, cleared=False),
        }
        removed = _remove_expired_zones(zones, time.time())
        assert 0 in removed
        assert 1 not in removed
        assert len(zones) == 1


class TestCostmapPatch:
    """OccupancyGrid 패치 셀 계산 로직."""

    def test_lethal_cells_in_radius(self):
        """반경 2m 내 셀은 모두 LETHAL_COST(100)."""
        count, data = _build_circle_cells(radius_m=2.0, resolution=0.05, map_size_m=10.0)
        # 반경 2m 원의 이론적 셀 수: pi * (2/0.05)^2 ≈ 5027셀
        assert count > 4000  # 넉넉한 하한

    def test_no_lethal_outside_radius(self):
        """패치 외곽(중심에서 멀리)은 FREE_COST."""
        _, data = _build_circle_cells(radius_m=2.0, resolution=0.05, map_size_m=10.0)
        # 패치 모서리 셀 (0,0) → FREE
        assert data[0] == FREE_COST

    def test_center_cell_is_lethal(self):
        """패치 중앙 셀은 LETHAL_COST."""
        cells_count = int(math.ceil(10.0 / 0.05))
        _, data = _build_circle_cells(radius_m=2.0, resolution=0.05, map_size_m=10.0)
        center_idx = (cells_count // 2) * cells_count + (cells_count // 2)
        assert data[center_idx] == LETHAL_COST

    def test_larger_radius_more_cells(self):
        """반경 클수록 lethal 셀 수 증가."""
        count_small, _ = _build_circle_cells(1.0, 0.1, 10.0)
        count_large, _ = _build_circle_cells(3.0, 0.1, 10.0)
        assert count_large > count_small


class TestFirePrediction:
    """kalman_tracker.py 확산 예측 기능 테스트."""

    def setup_method(self):
        sys.path.insert(0, str(Path(__file__).parent.parent / 'argos_bringup'))

    def test_predict_stationary_fire(self):
        """정지 화점 → 예측 위치 = 현재 위치."""
        from kalman_tracker import SimpleKalmanTracker
        tracker = SimpleKalmanTracker()
        now = time.time()
        # 여러 프레임으로 정지 화점 학습
        for _ in range(10):
            tracker.update([(5.0, 3.0, 'fire', 0.9)], timestamp=now)
            now += 1.0

        preds = tracker.predict_fire_positions(horizon_sec=300.0)
        assert len(preds) == 1
        # 속도가 거의 0이므로 예측 위치 ≈ 현재 위치 (±1m 허용)
        assert abs(preds[0]['predicted_x'] - preds[0]['current_x']) < 1.0
        assert abs(preds[0]['predicted_y'] - preds[0]['current_y']) < 1.0

    def test_predict_moving_fire_direction(self):
        """X축 이동 화점 → 예측이 현재보다 X 방향으로 앞서야 함."""
        from kalman_tracker import SimpleKalmanTracker
        tracker = SimpleKalmanTracker()
        now = time.time()
        # X축 방향으로 1m/s로 이동하는 화점
        for i in range(10):
            tracker.update([(float(i) * 0.1, 0.0, 'fire', 0.9)], timestamp=now)
            now += 1.0

        preds = tracker.predict_fire_positions(horizon_sec=60.0)
        assert len(preds) == 1
        # 양의 X 방향 이동 → 예측 X > 현재 X
        assert preds[0]['predicted_x'] > preds[0]['current_x']

    def test_evacuation_direction_opposite_fire(self):
        """대피 방향은 화점 반대 방향이어야 함."""
        from kalman_tracker import SimpleKalmanTracker
        tracker = SimpleKalmanTracker()
        now = time.time()
        # 화점: (10, 0) — 로봇 기준 오른쪽
        for _ in range(10):
            tracker.update([(10.0, 0.0, 'fire', 0.9)], timestamp=now)
            now += 1.0

        # 로봇 위치: (0, 0)
        evac = tracker.get_evacuation_direction(0.0, 0.0, safe_distance=5.0)
        assert evac is not None
        # 대피 목적지 X는 음수여야 함 (화점 반대 방향)
        assert evac['safe_x'] < 0

    def test_evacuation_no_fire_returns_none(self):
        """화점 없을 때 대피 경로 계산 → None."""
        from kalman_tracker import SimpleKalmanTracker
        tracker = SimpleKalmanTracker()
        evac = tracker.get_evacuation_direction(0.0, 0.0)
        assert evac is None

    def test_predict_horizon_scales_distance(self):
        """지평선이 길수록 예측 이동 거리도 커야 함."""
        from kalman_tracker import SimpleKalmanTracker
        tracker = SimpleKalmanTracker()
        now = time.time()
        for i in range(10):
            tracker.update([(float(i) * 0.05, 0.0, 'fire', 0.9)], timestamp=now)
            now += 1.0

        preds_short = tracker.predict_fire_positions(horizon_sec=60.0)
        preds_long = tracker.predict_fire_positions(horizon_sec=600.0)
        if preds_short and preds_long:
            # 긴 지평선 = 더 먼 예측
            short_dist = abs(preds_short[0]['predicted_x'] - preds_short[0]['current_x'])
            long_dist = abs(preds_long[0]['predicted_x'] - preds_long[0]['current_x'])
            assert long_dist >= short_dist
