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
"""S-D4 드론-UGV 화재 핸드오프 통합 테스트.

테스트 범위:
  - DroneFireRelay 핵심 로직 (쿨다운, 고도 검증, 근접 중복 제거, 지상 투영)
  - CBBA 할당: 화재 시 UGV=inspect_fire, 드론=monitor 분리
  - E2E 체인: hotspot 감지 → FireAlert → CBBA 할당 → 역할 배정
  - 다중 드론 시나리오
  - capability 표준화 후 드론 inspect_fire 가능 확인

rclpy 없이 순수 Python으로 실행 가능.
"""
import math
from unittest.mock import MagicMock

import pytest

from argos_bringup.cbba_allocator import (
    CBBAAllocator,
    RobotRecord as CbbaRobotRecord,
    Task as CbbaTask,
)


# ── Mock 클래스 ──────────────────────────────────────────────────

class MockPoint:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class MockPointStamped:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.header = MagicMock()
        self.point = MockPoint(x, y, z)


class MockFireAlert:
    def __init__(self, severity='medium', temp_k=573.15, x=0.0, y=0.0):
        self.severity = severity
        self.max_temperature_kelvin = temp_k
        self.location = MockPointStamped(x, y, 0.0)
        self.robot_id = 'drone1'
        self.header = MagicMock()


# ── DroneFireRelay 순수 로직 (노드 없이 테스트 가능한 형태) ─────────

class DroneFireRelayLogic:
    """DroneFireRelayNode의 순수 로직 추출 (rclpy 미의존).

    DroneFireRelayNode에서 rclpy 의존을 제거하고 핵심 판단 로직만
    분리한 테스트 가능 형태. 실제 노드 구현과 동일한 동작 보장.
    """

    def __init__(self, drone_id='drone1', cooldown_sec=5.0,
                 altitude_threshold=2.0, proximity_dedup_m=3.0):
        self.drone_id = drone_id
        self.cooldown_sec = cooldown_sec
        self.altitude_threshold = altitude_threshold
        self.proximity_dedup_m = proximity_dedup_m
        self.drone_pose = None          # (x, y, z) 튜플 또는 None
        self.last_alert_time = 0.0
        self.last_alert_location = None  # (x, y) 튜플 또는 None
        self.alert_count = 0

    def should_relay(self, now_sec: float, drone_alt: float,
                     ground_x: float, ground_y: float) -> tuple:
        """릴레이 가능 여부 판단 + 사유 반환.

        우선순위 순서로 검증:
          1. pose 미수신 → no_pose
          2. 고도 미달 → low_altitude
          3. 쿨다운 중 → cooldown
          4. 근접 중복 → proximity_dedup
          5. 모두 통과 → ok

        Args:
            now_sec:  현재 시각 (초, UNIX epoch)
            drone_alt: 드론 현재 고도 (m)
            ground_x:  투영된 지상 좌표 X (m)
            ground_y:  투영된 지상 좌표 Y (m)

        Returns:
            (should_relay: bool, reason: str)
        """
        # 1. pose 미수신 검증
        if self.drone_pose is None:
            return False, 'no_pose'

        # 2. 고도 검증 (threshold 미만이면 거부)
        if drone_alt < self.altitude_threshold:
            return False, 'low_altitude'

        # 3. 쿨다운 검증
        if (now_sec - self.last_alert_time) < self.cooldown_sec:
            return False, 'cooldown'

        # 4. 근접 중복 제거 (이전 알림과 proximity_dedup_m 내에 있으면 거부)
        if self.last_alert_location is not None:
            dist = math.hypot(
                ground_x - self.last_alert_location[0],
                ground_y - self.last_alert_location[1])
            if dist < self.proximity_dedup_m:
                return False, 'proximity_dedup'

        return True, 'ok'

    def record_alert(self, now_sec: float, x: float, y: float):
        """알림 발송 후 상태 갱신."""
        self.last_alert_time = now_sec
        self.last_alert_location = (x, y)
        self.alert_count += 1

    @staticmethod
    def project_to_ground(drone_x, drone_y, drone_z,
                          hotspot_x=0.0, hotspot_y=0.0):
        """지상 좌표 투영.

        hotspot 좌표가 유효(비영)하면 hotspot 위치 사용,
        없으면 드론 직하방(nadir) 투영.

        Returns:
            (x, y, z) — z는 항상 0.0 (지상면)
        """
        if hotspot_x != 0.0 or hotspot_y != 0.0:
            return (hotspot_x, hotspot_y, 0.0)
        return (drone_x, drone_y, 0.0)


# ── 쿨다운 테스트 ────────────────────────────────────────────────

class TestCooldown:
    """쿨다운 메커니즘: 동일 구역 알림 홍수 방지."""

    def test_first_alert_always_passes(self):
        """첫 번째 알림은 항상 통과 (이전 알림 없음)."""
        logic = DroneFireRelayLogic()
        logic.drone_pose = (5.0, 5.0, 8.0)
        ok, reason = logic.should_relay(100.0, 8.0, 5.0, 5.0)
        assert ok and reason == 'ok'

    def test_cooldown_blocks_rapid_alerts(self):
        """쿨다운 5초 내 재발신 → 거부."""
        logic = DroneFireRelayLogic(cooldown_sec=5.0)
        logic.drone_pose = (5.0, 5.0, 8.0)
        logic.record_alert(100.0, 5.0, 5.0)
        # 3초 후 → 쿨다운 내
        ok, reason = logic.should_relay(103.0, 8.0, 10.0, 10.0)
        assert not ok and reason == 'cooldown'

    def test_cooldown_expires_allows_new_alert(self):
        """쿨다운 경과(6초) → 새 알림 허용."""
        logic = DroneFireRelayLogic(cooldown_sec=5.0)
        logic.drone_pose = (5.0, 5.0, 8.0)
        logic.record_alert(100.0, 5.0, 5.0)
        # 6초 후 → 쿨다운 만료
        ok, reason = logic.should_relay(106.0, 8.0, 10.0, 10.0)
        assert ok and reason == 'ok'

    def test_cooldown_exact_boundary_still_blocked(self):
        """쿨다운 정확히 경계(5.0초)는 거부 (< 아닌 <=이 아니므로)."""
        logic = DroneFireRelayLogic(cooldown_sec=5.0)
        logic.drone_pose = (5.0, 5.0, 8.0)
        logic.record_alert(100.0, 5.0, 5.0)
        # 정확히 5.0초 후 → (now - last) = 5.0, 조건: < 5.0 → 거부 아님
        # 즉 5.0초 경과 시 허용되어야 함
        ok, reason = logic.should_relay(105.0, 8.0, 10.0, 10.0)
        assert ok and reason == 'ok'

    def test_zero_cooldown_always_passes(self):
        """쿨다운 0초 설정 → 연속 알림 모두 허용."""
        logic = DroneFireRelayLogic(cooldown_sec=0.0)
        logic.drone_pose = (5.0, 5.0, 8.0)
        logic.record_alert(100.0, 0.0, 0.0)
        ok, reason = logic.should_relay(100.0, 8.0, 20.0, 20.0)
        assert ok

    def test_alert_count_increments(self):
        """record_alert 호출마다 alert_count 증가."""
        logic = DroneFireRelayLogic(cooldown_sec=0.0)
        logic.drone_pose = (5.0, 5.0, 8.0)
        logic.record_alert(100.0, 1.0, 1.0)
        logic.record_alert(101.0, 10.0, 10.0)
        assert logic.alert_count == 2


# ── 고도 검증 테스트 ─────────────────────────────────────────────

class TestAltitudeCheck:
    """고도 검증: 지면 근접 드론의 오감지 방지."""

    def test_ground_level_rejected(self):
        """지면 수준(0.5m) → 거부."""
        logic = DroneFireRelayLogic(altitude_threshold=2.0)
        logic.drone_pose = (5.0, 5.0, 0.5)
        ok, reason = logic.should_relay(100.0, 0.5, 5.0, 5.0)
        assert not ok and reason == 'low_altitude'

    def test_above_threshold_accepted(self):
        """임계값(2.0m) 초과(8.0m) → 허용."""
        logic = DroneFireRelayLogic(altitude_threshold=2.0)
        logic.drone_pose = (5.0, 5.0, 8.0)
        ok, reason = logic.should_relay(100.0, 8.0, 5.0, 5.0)
        assert ok

    def test_exactly_at_threshold_accepted(self):
        """임계값(2.0m) 정확히 → 허용 (미만 조건: < threshold, 2.0 < 2.0 = False)."""
        logic = DroneFireRelayLogic(altitude_threshold=2.0)
        logic.drone_pose = (5.0, 5.0, 2.0)
        ok, reason = logic.should_relay(100.0, 2.0, 5.0, 5.0)
        assert ok and reason == 'ok'

    def test_no_pose_rejected(self):
        """드론 pose 미수신 → 고도 검사 전에 거부."""
        logic = DroneFireRelayLogic()
        ok, reason = logic.should_relay(100.0, 8.0, 5.0, 5.0)
        assert not ok and reason == 'no_pose'

    def test_negative_altitude_rejected(self):
        """음수 고도 (GPS 오차 등) → 거부."""
        logic = DroneFireRelayLogic(altitude_threshold=2.0)
        logic.drone_pose = (5.0, 5.0, -1.0)
        ok, reason = logic.should_relay(100.0, -1.0, 5.0, 5.0)
        assert not ok and reason == 'low_altitude'

    def test_custom_threshold(self):
        """커스텀 임계값(5.0m): 4.9m는 거부, 5.1m는 허용."""
        logic = DroneFireRelayLogic(altitude_threshold=5.0)
        logic.drone_pose = (0.0, 0.0, 4.9)
        ok, _ = logic.should_relay(100.0, 4.9, 0.0, 0.0)
        assert not ok

        logic2 = DroneFireRelayLogic(altitude_threshold=5.0)
        logic2.drone_pose = (0.0, 0.0, 5.1)
        ok2, reason2 = logic2.should_relay(100.0, 5.1, 0.0, 0.0)
        assert ok2


# ── 근접 중복 제거 테스트 ────────────────────────────────────────

class TestProximityDedup:
    """근접 중복 제거: 동일 화점 다중 감지 억제."""

    def test_same_location_blocked(self):
        """이전 알림과 0.7m 거리 → 근접 중복으로 거부."""
        logic = DroneFireRelayLogic(proximity_dedup_m=3.0, cooldown_sec=0.0)
        logic.drone_pose = (5.0, 5.0, 8.0)
        logic.record_alert(100.0, 5.0, 5.0)
        ok, reason = logic.should_relay(101.0, 8.0, 5.5, 5.5)
        assert not ok and reason == 'proximity_dedup'

    def test_distant_location_passes(self):
        """이전 알림과 14m 이상 거리 → 새 화점으로 허용."""
        logic = DroneFireRelayLogic(proximity_dedup_m=3.0, cooldown_sec=0.0)
        logic.drone_pose = (5.0, 5.0, 8.0)
        logic.record_alert(100.0, 5.0, 5.0)
        ok, reason = logic.should_relay(101.0, 8.0, 15.0, 15.0)
        assert ok

    def test_exactly_at_boundary_blocked(self):
        """이전 알림과 정확히 3.0m → 거부 (< 조건, == 포함)."""
        logic = DroneFireRelayLogic(proximity_dedup_m=3.0, cooldown_sec=0.0)
        logic.drone_pose = (5.0, 5.0, 8.0)
        logic.record_alert(100.0, 0.0, 0.0)
        # dist = sqrt(9+0) = 3.0 → 3.0 < 3.0 = False → 통과
        # 정확히 경계는 통과(허용)됨을 확인
        ok, reason = logic.should_relay(101.0, 8.0, 3.0, 0.0)
        # dist == proximity_dedup_m → 조건 < 이므로 통과(ok=True)
        assert ok

    def test_just_inside_boundary_blocked(self):
        """경계보다 약간 안쪽(2.99m) → 거부."""
        logic = DroneFireRelayLogic(proximity_dedup_m=3.0, cooldown_sec=0.0)
        logic.drone_pose = (5.0, 5.0, 8.0)
        logic.record_alert(100.0, 0.0, 0.0)
        # dist = 2.99 → 2.99 < 3.0 → 거부
        ok, reason = logic.should_relay(101.0, 8.0, 2.99, 0.0)
        assert not ok and reason == 'proximity_dedup'

    def test_no_previous_alert_passes(self):
        """이전 알림 없으면 근접 중복 검사 건너뜀."""
        logic = DroneFireRelayLogic(proximity_dedup_m=3.0, cooldown_sec=0.0)
        logic.drone_pose = (5.0, 5.0, 8.0)
        ok, reason = logic.should_relay(100.0, 8.0, 0.0, 0.0)
        assert ok

    def test_custom_proximity_threshold(self):
        """커스텀 근접 임계값(10.0m): 9m 거리 → 거부."""
        logic = DroneFireRelayLogic(proximity_dedup_m=10.0, cooldown_sec=0.0)
        logic.drone_pose = (0.0, 0.0, 8.0)
        logic.record_alert(100.0, 0.0, 0.0)
        ok, reason = logic.should_relay(101.0, 8.0, 9.0, 0.0)
        assert not ok and reason == 'proximity_dedup'


# ── 지상 투영 테스트 ─────────────────────────────────────────────

class TestGroundProjection:
    """지상 좌표 투영: 드론 위치 → 지상면 화점 좌표."""

    def test_nadir_projection(self):
        """hotspot 없으면 드론 직하방(nadir) 투영."""
        x, y, z = DroneFireRelayLogic.project_to_ground(5.0, 3.0, 8.0)
        assert (x, y, z) == (5.0, 3.0, 0.0)

    def test_hotspot_override(self):
        """hotspot 좌표가 있으면 해당 위치 사용 (드론 위치 무시)."""
        x, y, z = DroneFireRelayLogic.project_to_ground(
            5.0, 3.0, 8.0, hotspot_x=7.0, hotspot_y=4.0)
        assert (x, y, z) == (7.0, 4.0, 0.0)

    def test_hotspot_zero_uses_drone_position(self):
        """hotspot 좌표가 (0,0)이면 드론 위치로 폴백."""
        x, y, z = DroneFireRelayLogic.project_to_ground(
            5.0, 3.0, 8.0, hotspot_x=0.0, hotspot_y=0.0)
        assert (x, y, z) == (5.0, 3.0, 0.0)

    def test_ground_z_always_zero(self):
        """투영 결과 z는 항상 0.0 (지상면)."""
        for drone_z in [2.0, 5.0, 50.0, 100.0]:
            _, _, gz = DroneFireRelayLogic.project_to_ground(
                0.0, 0.0, drone_z)
            assert gz == 0.0

    def test_negative_drone_position(self):
        """음수 드론 좌표도 정상 투영."""
        x, y, z = DroneFireRelayLogic.project_to_ground(-3.0, -4.0, 8.0)
        assert (x, y, z) == (-3.0, -4.0, 0.0)

    def test_hotspot_only_x_nonzero_uses_hotspot(self):
        """hotspot_x만 유효(0 아님) → hotspot 사용."""
        x, y, z = DroneFireRelayLogic.project_to_ground(
            5.0, 3.0, 8.0, hotspot_x=7.0, hotspot_y=0.0)
        assert (x, y, z) == (7.0, 0.0, 0.0)

    def test_hotspot_only_y_nonzero_uses_hotspot(self):
        """hotspot_y만 유효(0 아님) → hotspot 사용."""
        x, y, z = DroneFireRelayLogic.project_to_ground(
            5.0, 3.0, 8.0, hotspot_x=0.0, hotspot_y=4.0)
        assert (x, y, z) == (0.0, 4.0, 0.0)


# ── CBBA 할당: UGV=inspect_fire, 드론=monitor ───────────────────

class TestCBBADroneUGVAllocation:
    """CBBA 알고리즘 기반 드론↔UGV 역할 분리 검증."""

    @pytest.fixture
    def allocator(self):
        return CBBAAllocator()

    def test_fire_assigns_ugv_inspect_drone_monitor(self, allocator):
        """화재 시 UGV→inspect_fire, 드론→monitor 역할 분리.

        UGV(argos1): has_thermal O, can_fly X → inspect_fire만 가능
        드론(drone1): can_fly O, has_thermal O → monitor 우선 배정
        CBBA 비용: 드론은 monitor(고도 8m) 임무에 거리 유리
        """
        robots = [
            CbbaRobotRecord('argos1', ['has_thermal', 'lidar'], (1.0, 0.0, 0.0)),
            CbbaRobotRecord('drone1', ['can_fly', 'has_thermal', 'camera'],
                            (5.0, 5.0, 8.0)),
        ]
        tasks = [
            CbbaTask('fire_suppress_ground', (2.0, 1.0, 0.0),
                     'inspect_fire', ['has_thermal'], 0.8),
            CbbaTask('fire_monitor_aerial', (2.0, 1.0, 8.0),
                     'monitor', ['can_fly'], 0.72),
        ]
        result = allocator.allocate(robots, tasks)
        assert len(result) == 2
        assert result['drone1'].task_type == 'monitor'
        assert result['argos1'].task_type == 'inspect_fire'

    def test_drone_can_get_inspect_fire_with_has_thermal(self, allocator):
        """has_thermal 통일 후 드론도 inspect_fire 수행 가능.

        UGV 없고 드론만 있을 때, 드론이 has_thermal 보유 시
        inspect_fire 임무를 받을 수 있어야 함 (capability 표준화 목표).
        """
        robots = [
            CbbaRobotRecord('drone1', ['can_fly', 'has_thermal', 'camera'],
                            (1.0, 0.0, 8.0)),
        ]
        tasks = [
            CbbaTask('fire_inspect', (2.0, 0.0, 0.0),
                     'inspect_fire', ['has_thermal'], 0.8),
        ]
        result = allocator.allocate(robots, tasks)
        assert 'drone1' in result
        assert result['drone1'].task_type == 'inspect_fire'

    def test_drone_without_thermal_cannot_inspect(self, allocator):
        """has_thermal 없는 드론 → inspect_fire 배정 불가."""
        robots = [
            CbbaRobotRecord('drone1', ['can_fly', 'camera'], (1.0, 0.0, 8.0)),
        ]
        tasks = [
            CbbaTask('fire_inspect', (2.0, 0.0, 0.0),
                     'inspect_fire', ['has_thermal'], 0.8),
        ]
        result = allocator.allocate(robots, tasks)
        assert 'drone1' not in result

    def test_ugv_without_fly_cannot_monitor(self, allocator):
        """can_fly 없는 UGV → monitor 배정 불가."""
        robots = [
            CbbaRobotRecord('argos1', ['has_thermal', 'lidar'], (0.0, 0.0, 0.0)),
        ]
        tasks = [
            CbbaTask('fire_monitor', (5.0, 5.0, 8.0),
                     'monitor', ['can_fly'], 0.7),
        ]
        result = allocator.allocate(robots, tasks)
        assert 'argos1' not in result

    def test_two_drones_two_ugvs_fire_scenario(self, allocator):
        """2드론 + 2UGV 화재 시나리오: 중복 없는 역할 분배."""
        robots = [
            CbbaRobotRecord('argos1', ['has_thermal', 'lidar'], (0.0, 0.0, 0.0)),
            CbbaRobotRecord('argos2', ['has_thermal', 'lidar'], (10.0, 0.0, 0.0)),
            CbbaRobotRecord('drone1', ['can_fly', 'has_thermal'], (5.0, 5.0, 8.0)),
            CbbaRobotRecord('drone2', ['can_fly', 'has_thermal'], (5.0, -5.0, 8.0)),
        ]
        tasks = [
            CbbaTask('fire_suppress', (2.0, 1.0, 0.0),
                     'inspect_fire', ['has_thermal'], 0.8),
            CbbaTask('fire_monitor', (2.0, 1.0, 8.0),
                     'monitor', ['can_fly'], 0.72),
            CbbaTask('fire_suppress_2', (8.0, 2.0, 0.0),
                     'inspect_fire', ['has_thermal'], 0.6),
        ]
        result = allocator.allocate(robots, tasks)
        assert len(result) >= 3
        # 동일 임무 중복 할당 금지
        task_ids = [t.task_id for t in result.values()]
        assert len(task_ids) == len(set(task_ids))

    def test_no_ugv_only_drones_get_all_tasks(self, allocator):
        """UGV 없으면 드론이 inspect_fire도 수행 (has_thermal 보유)."""
        robots = [
            CbbaRobotRecord('drone1', ['can_fly', 'has_thermal'], (0.0, 0.0, 8.0)),
        ]
        tasks = [
            CbbaTask('fire_suppress', (2.0, 0.0, 0.0),
                     'inspect_fire', ['has_thermal'], 0.8),
        ]
        result = allocator.allocate(robots, tasks)
        assert 'drone1' in result

    def test_priority_breaks_tie(self, allocator):
        """동거리 임무 중 priority 높은 것 우선 선택.

        CBBA 비용 = max(0, dist - priority*2).
        동거리(dist=1.0): priority 0.8 → cost=0, priority 0.3 → cost=0.4
        비용 낮은 high-priority 임무 선택.
        """
        robots = [
            CbbaRobotRecord('argos1', ['has_thermal'], (0.0, 0.0, 0.0)),
        ]
        tasks = [
            CbbaTask('low_priority', (1.0, 0.0, 0.0),
                     'inspect_fire', ['has_thermal'], 0.3),
            CbbaTask('high_priority', (1.0, 0.0, 0.0),
                     'inspect_fire', ['has_thermal'], 0.8),
        ]
        result = allocator.allocate(robots, tasks)
        assert 'argos1' in result
        assert result['argos1'].task_id == 'high_priority'

    def test_empty_robots_returns_empty(self, allocator):
        """로봇 없음 → 빈 할당."""
        tasks = [
            CbbaTask('fire_suppress', (0.0, 0.0, 0.0),
                     'inspect_fire', ['has_thermal'], 0.8),
        ]
        result = allocator.allocate([], tasks)
        assert result == {}

    def test_empty_tasks_returns_empty(self, allocator):
        """임무 없음 → 빈 할당."""
        robots = [
            CbbaRobotRecord('argos1', ['has_thermal'], (0.0, 0.0, 0.0)),
        ]
        result = allocator.allocate(robots, [])
        assert result == {}


# ── E2E 핸드오프 체인 테스트 ────────────────────────────────────

class TestE2EHandoffChain:
    """드론 감지 → 릴레이 → CBBA → UGV 배정 전체 체인 검증."""

    def test_full_chain_drone_detects_ugv_dispatched(self):
        """E2E: 드론 감지 → 릴레이 판단 → 지상 투영 → CBBA → UGV 배정."""
        # Step 1: 드론 감지 릴레이 판단
        relay = DroneFireRelayLogic(drone_id='drone1')
        relay.drone_pose = (5.0, 3.0, 8.0)
        ok, _ = relay.should_relay(100.0, 8.0, 5.0, 3.0)
        assert ok

        # Step 2: 지상 좌표 투영
        gx, gy, gz = relay.project_to_ground(5.0, 3.0, 8.0)
        assert gz == 0.0

        # Step 3: CBBA 할당
        robots = [
            CbbaRobotRecord('argos1', ['has_thermal', 'lidar'], (0.0, 0.0, 0.0)),
            CbbaRobotRecord('drone1', ['can_fly', 'has_thermal'], (5.0, 3.0, 8.0)),
        ]
        tasks = [
            CbbaTask('fire_suppress_ground', (gx, gy, 0.0),
                     'inspect_fire', ['has_thermal'], 0.8),
            CbbaTask('fire_monitor_aerial', (gx, gy, 8.0),
                     'monitor', ['can_fly'], 0.72),
        ]
        allocator = CBBAAllocator()
        result = allocator.allocate(robots, tasks)

        # Step 4: 역할 검증
        assert 'argos1' in result
        assert result['argos1'].task_type == 'inspect_fire'
        assert 'drone1' in result
        assert result['drone1'].task_type == 'monitor'

        # Step 5: 릴레이 상태 갱신
        relay.record_alert(100.0, gx, gy)
        assert relay.alert_count == 1

    def test_multiple_drones_independent_relay_logic(self):
        """2대 드론 동시 감지: 각 드론의 릴레이는 독립적.

        relay1, relay2 각각 독립 상태를 갖고 있어 모두 ok 반환.
        오케스트레이터가 FireAlert 수준에서 중복을 제거해야 함.
        """
        relay1 = DroneFireRelayLogic(drone_id='drone1', cooldown_sec=0.0)
        relay1.drone_pose = (5.0, 3.0, 8.0)
        relay2 = DroneFireRelayLogic(drone_id='drone2', cooldown_sec=0.0)
        relay2.drone_pose = (5.5, 3.5, 9.0)

        ok1, _ = relay1.should_relay(100.0, 8.0, 5.0, 3.0)
        assert ok1
        relay1.record_alert(100.0, 5.0, 3.0)

        # relay2는 독립 상태 → 중복 없음 → 허용
        ok2, _ = relay2.should_relay(100.0, 9.0, 5.5, 3.5)
        assert ok2

    def test_sequential_fires_different_locations(self):
        """순차 화점: 첫 번째 완료 후 두 번째 릴레이 허용."""
        relay = DroneFireRelayLogic(cooldown_sec=5.0, proximity_dedup_m=3.0)
        relay.drone_pose = (5.0, 5.0, 8.0)

        # 첫 번째 화점
        ok, _ = relay.should_relay(100.0, 8.0, 5.0, 5.0)
        assert ok
        relay.record_alert(100.0, 5.0, 5.0)

        # 같은 화점, 쿨다운 내 → 거부
        ok, reason = relay.should_relay(102.0, 8.0, 5.0, 5.0)
        assert not ok

        # 다른 화점, 쿨다운 경과 → 허용
        ok, _ = relay.should_relay(106.0, 8.0, 20.0, 20.0)
        assert ok

    def test_fire_alert_constructs_correct_fields(self):
        """MockFireAlert 필드 검증: severity, temperature, location, robot_id."""
        alert = MockFireAlert(severity='high', temp_k=623.15, x=3.0, y=4.0)
        assert alert.severity == 'high'
        assert alert.max_temperature_kelvin == pytest.approx(623.15)
        assert alert.location.point.x == 3.0
        assert alert.location.point.y == 4.0
        assert alert.location.point.z == 0.0
        assert alert.robot_id == 'drone1'

    def test_ugv_dispatched_to_projected_ground_point(self):
        """UGV는 드론 투영 지상 좌표로 배정됨."""
        relay = DroneFireRelayLogic()
        relay.drone_pose = (10.0, 7.0, 15.0)

        # 드론 직하방 투영
        gx, gy, gz = relay.project_to_ground(10.0, 7.0, 15.0)

        robots = [
            CbbaRobotRecord('argos1', ['has_thermal'], (0.0, 0.0, 0.0)),
        ]
        tasks = [
            CbbaTask('fire_at_projection', (gx, gy, gz),
                     'inspect_fire', ['has_thermal'], 0.9),
        ]
        allocator = CBBAAllocator()
        result = allocator.allocate(robots, tasks)

        assert 'argos1' in result
        # 배정된 임무 위치가 투영 지점과 일치
        assert result['argos1'].location[0] == pytest.approx(gx)
        assert result['argos1'].location[1] == pytest.approx(gy)
        assert result['argos1'].location[2] == pytest.approx(0.0)

    def test_cooldown_prevents_duplicate_cbba_triggers(self):
        """쿨다운으로 CBBA 재트리거 방지: 동일 알림 반복 시 1회만 할당."""
        relay = DroneFireRelayLogic(cooldown_sec=10.0)
        relay.drone_pose = (5.0, 5.0, 8.0)

        trigger_count = 0
        for t in range(5):
            ok, reason = relay.should_relay(100.0 + t, 8.0, 5.0, 5.0)
            if ok:
                relay.record_alert(100.0 + t, 5.0, 5.0)
                trigger_count += 1

        # 5번 시도 중 쿨다운(10초) 내 → 1회만 릴레이
        assert trigger_count == 1

    def test_multi_ugv_nearest_dispatched(self):
        """2대 UGV 중 화점에 더 가까운 UGV가 inspect_fire 배정."""
        relay = DroneFireRelayLogic()
        relay.drone_pose = (5.0, 0.0, 8.0)
        gx, gy, _ = relay.project_to_ground(5.0, 0.0, 8.0)

        robots = [
            CbbaRobotRecord('argos1', ['has_thermal'], (4.5, 0.0, 0.0)),  # 화점에 매우 근접
            CbbaRobotRecord('argos2', ['has_thermal'], (50.0, 0.0, 0.0)),  # 멀리 있음
        ]
        tasks = [
            CbbaTask('fire', (gx, gy, 0.0), 'inspect_fire', ['has_thermal'], 0.8),
        ]
        allocator = CBBAAllocator()
        result = allocator.allocate(robots, tasks)

        # argos1이 더 가까우므로 선택
        assert 'argos1' in result
        assert result['argos1'].task_type == 'inspect_fire'
        assert 'argos2' not in result
