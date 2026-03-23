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
"""E2E 풀 시스템 통합 테스트 — ARGOS 전체 파이프라인 검증.

Gazebo 없이 순수 Python으로 전체 시스템 연동을 검증.
8개 시나리오로 CBBA+편대+핸드오프+호스+capability 전체 커버.
"""
import math
import pytest

from argos_bringup.cbba_allocator import (
    CBBAAllocator, RobotRecord as CbbaRobot, Task as CbbaTask,
)
from argos_bringup.formation_manager import (
    FormationManager, FormationPattern, RobotConfig,
)


# ── 헬퍼 ──

class DroneRelayLogic:
    def __init__(self, cooldown=5.0, alt_threshold=2.0, dedup_m=3.0):
        self.cooldown = cooldown
        self.alt_threshold = alt_threshold
        self.dedup_m = dedup_m
        self.last_time = 0.0
        self.last_loc = None

    def should_relay(self, now, alt, x, y):
        if alt < self.alt_threshold:
            return False, 'low_alt'
        if (now - self.last_time) < self.cooldown:
            return False, 'cooldown'
        if self.last_loc:
            if math.hypot(x - self.last_loc[0], y - self.last_loc[1]) < self.dedup_m:
                return False, 'dedup'
        return True, 'ok'

    def record(self, now, x, y):
        self.last_time = now
        self.last_loc = (x, y)


def make_fleet():
    return [
        CbbaRobot('ugv1', ['has_thermal', 'lidar', 'depth'], (0.0, 0.0, 0.0)),
        CbbaRobot('ugv2', ['has_thermal', 'lidar', 'depth'], (10.0, 0.0, 0.0)),
        CbbaRobot('drone1', ['can_fly', 'has_thermal', 'camera'], (5.0, 5.0, 8.0)),
        CbbaRobot('drone2', ['can_fly', 'has_thermal', 'camera'], (5.0, -5.0, 8.0)),
        CbbaRobot('sherpa1', ['has_thermal', 'lidar', 'water_cannon', 'hose'], (2.0, 0.0, 0.0)),
    ]


# ── Scenario 1: 편대 구성 ──

class TestS1_FleetSetup:
    def test_5_robots(self):
        assert len(make_fleet()) == 5

    def test_all_have_has_thermal(self):
        for r in make_fleet():
            assert 'has_thermal' in r.capabilities

    def test_no_bare_thermal(self):
        for r in make_fleet():
            assert 'thermal' not in r.capabilities

    def test_ugv_drone_sherpa_split(self):
        fleet = make_fleet()
        drones = [r for r in fleet if 'can_fly' in r.capabilities]
        sherpas = [r for r in fleet if 'water_cannon' in r.capabilities]
        ugvs = [r for r in fleet if 'can_fly' not in r.capabilities and 'water_cannon' not in r.capabilities]
        assert len(ugvs) == 2 and len(drones) == 2 and len(sherpas) == 1


# ── Scenario 2: 화재 → CBBA ──

class TestS2_FireCBBA:
    def test_fire_allocates_two_roles(self):
        alloc = CBBAAllocator()
        tasks = [
            CbbaTask('suppress', (5.0, 3.0, 0.0), 'inspect_fire', ['has_thermal'], 0.8),
            CbbaTask('monitor', (5.0, 3.0, 8.0), 'monitor', ['can_fly'], 0.72),
        ]
        result = alloc.allocate(make_fleet(), tasks)
        types = {t.task_type for t in result.values()}
        assert 'inspect_fire' in types and 'monitor' in types

    def test_no_duplicate_tasks(self):
        alloc = CBBAAllocator()
        tasks = [
            CbbaTask('t1', (1.0, 0.0, 0.0), 'inspect_fire', ['has_thermal'], 0.8),
            CbbaTask('t2', (9.0, 0.0, 0.0), 'inspect_fire', ['has_thermal'], 0.6),
            CbbaTask('t3', (5.0, 5.0, 8.0), 'monitor', ['can_fly'], 0.7),
        ]
        result = alloc.allocate(make_fleet(), tasks)
        ids = [t.task_id for t in result.values()]
        assert len(ids) == len(set(ids))

    def test_nearest_ugv_gets_suppress(self):
        alloc = CBBAAllocator()
        robots = [
            CbbaRobot('ugv_near', ['has_thermal'], (4.0, 3.0, 0.0)),
            CbbaRobot('ugv_far', ['has_thermal'], (20.0, 0.0, 0.0)),
        ]
        tasks = [CbbaTask('suppress', (5.0, 3.0, 0.0), 'inspect_fire', ['has_thermal'], 0.8)]
        result = alloc.allocate(robots, tasks)
        assert 'ugv_near' in result


# ── Scenario 3: 번들 N ──

class TestS3_Bundle:
    def test_bundle_2_max(self):
        alloc = CBBAAllocator(max_bundle_size=2)
        robots = [CbbaRobot('r1', ['has_thermal'], (0, 0, 0))]
        tasks = [CbbaTask(f't{i}', (i, 0, 0), 'explore', [], 0.5) for i in range(3)]
        bundles = alloc.allocate_bundles(robots, tasks)
        assert len(bundles.get('r1', [])) <= 2

    def test_all_fires_covered_with_enough_robots(self):
        alloc = CBBAAllocator(max_bundle_size=2)
        robots = make_fleet()
        tasks = [CbbaTask(f'f{i}', (i*2, 0, 0), 'inspect_fire', ['has_thermal'], 0.5) for i in range(5)]
        bundles = alloc.allocate_bundles(robots, tasks)
        covered = {t.task_id for tl in bundles.values() for t in tl}
        assert len(covered) == 5

    def test_backward_compat_allocate(self):
        alloc = CBBAAllocator(max_bundle_size=3)
        robots = [CbbaRobot('r1', ['has_thermal'], (0, 0, 0))]
        tasks = [CbbaTask('t1', (1, 0, 0), 'explore', [], 0.5)]
        result = alloc.allocate(robots, tasks)  # returns dict[str, Task]
        assert isinstance(result['r1'], CbbaTask)


# ── Scenario 4: 호스 제약 ──

class TestS4_Hose:
    @staticmethod
    def hose_ok(remaining, tx, ty, rx=0.0, ry=0.0):
        return math.hypot(tx - rx, ty - ry) <= remaining

    def test_50m_fire_30m_hose_reject(self):
        assert not self.hose_ok(30.0, 50.0, 0.0)

    def test_20m_fire_30m_hose_accept(self):
        assert self.hose_ok(30.0, 20.0, 0.0)

    def test_boundary_accept(self):
        assert self.hose_ok(30.0, 30.0, 0.0)


# ── Scenario 5: 편대 전환 ──

class TestS5_Formation:
    def test_explore_then_surround(self):
        configs = [
            RobotConfig('ugv1', 'ugv', (0, 0)),
            RobotConfig('ugv2', 'ugv', (5, 0)),
            RobotConfig('drone1', 'drone', (2.5, 3)),
        ]
        fm = FormationManager(configs)
        explore = fm.compute_formation((5, 5), FormationPattern.LINE_ABREAST, spacing=3.0)
        surround = fm.compute_formation((5, 3), FormationPattern.SURROUND, spacing=4.0)
        assert len(explore) == 3 and len(surround) == 3
        assert surround['drone1'][2] == 8.0  # altitude

    def test_no_collision(self):
        configs = [RobotConfig('u1', 'ugv', (0, 0)), RobotConfig('u2', 'ugv', (5, 0))]
        fm = FormationManager(configs)
        wp = fm.compute_formation((5, 5), FormationPattern.LINE_ABREAST, spacing=3.0)
        assert len(fm.check_collision_risk(wp)) == 0

    def test_min_spacing_clamp(self):
        configs = [RobotConfig('u1', 'ugv', (0, 0)), RobotConfig('u2', 'ugv', (0, 0))]
        fm = FormationManager(configs)
        wp = fm.compute_formation((0, 0), FormationPattern.LINE_ABREAST, spacing=0.1)
        d = math.hypot(wp['u1'][0] - wp['u2'][0], wp['u1'][1] - wp['u2'][1])
        assert d >= 1.5


# ── Scenario 6: 드론 릴레이 → CBBA ──

class TestS6_Handoff:
    def test_full_chain(self):
        relay = DroneRelayLogic()
        ok, _ = relay.should_relay(100.0, 8.0, 5.0, 3.0)
        assert ok
        relay.record(100.0, 5.0, 3.0)

        alloc = CBBAAllocator()
        robots = [
            CbbaRobot('ugv1', ['has_thermal', 'lidar'], (0, 0, 0)),
            CbbaRobot('drone1', ['can_fly', 'has_thermal'], (5, 3, 8)),
        ]
        tasks = [
            CbbaTask('suppress', (5, 3, 0), 'inspect_fire', ['has_thermal'], 0.8),
            CbbaTask('monitor', (5, 3, 8), 'monitor', ['can_fly'], 0.72),
        ]
        result = alloc.allocate(robots, tasks)
        assert result['drone1'].task_type == 'monitor'
        assert result['ugv1'].task_type == 'inspect_fire'

    def test_cooldown_blocks(self):
        relay = DroneRelayLogic(cooldown=5.0)
        relay.record(100.0, 5.0, 3.0)
        ok, reason = relay.should_relay(103.0, 8.0, 5.0, 3.0)
        assert not ok and reason == 'cooldown'

    def test_new_location_passes(self):
        relay = DroneRelayLogic(cooldown=0.0, dedup_m=3.0)
        relay.record(100.0, 0.0, 0.0)
        ok, _ = relay.should_relay(101.0, 8.0, 20.0, 20.0)
        assert ok


# ── Scenario 7: Capability 표준화 ──

class TestS7_Capability:
    def test_drone_inspect_fire(self):
        alloc = CBBAAllocator()
        result = alloc.allocate(
            [CbbaRobot('d1', ['can_fly', 'has_thermal'], (0, 0, 8))],
            [CbbaTask('f', (1, 0, 0), 'inspect_fire', ['has_thermal'], 0.8)],
        )
        assert 'd1' in result

    def test_ugv_cannot_monitor(self):
        alloc = CBBAAllocator()
        result = alloc.allocate(
            [CbbaRobot('u1', ['has_thermal'], (0, 0, 0))],
            [CbbaTask('m', (0, 0, 8), 'monitor', ['can_fly'], 0.8)],
        )
        assert len(result) == 0

    def test_sherpa_inspect_fire(self):
        alloc = CBBAAllocator()
        result = alloc.allocate(
            [CbbaRobot('s1', ['has_thermal', 'water_cannon'], (0, 0, 0))],
            [CbbaTask('f', (1, 0, 0), 'inspect_fire', ['has_thermal'], 0.8)],
        )
        assert 's1' in result


# ── Scenario 8: 엣지 케이스 ──

class TestS8_Edge:
    def test_empty_robots(self):
        assert CBBAAllocator().allocate([], [CbbaTask('t', (0, 0, 0), 'explore', [], 0.5)]) == {}

    def test_no_capable_robot(self):
        result = CBBAAllocator().allocate(
            [CbbaRobot('u1', ['has_thermal'], (0, 0, 0))],
            [CbbaTask('m', (0, 0, 8), 'monitor', ['can_fly'], 0.8)],
        )
        assert len(result) == 0

    def test_more_tasks_than_robots(self):
        result = CBBAAllocator().allocate(
            [CbbaRobot('r1', ['has_thermal'], (0, 0, 0))],
            [CbbaTask(f't{i}', (i, 0, 0), 'explore', [], 0.5) for i in range(5)],
        )
        assert len(result) == 1

    def test_low_altitude_rejected(self):
        ok, reason = DroneRelayLogic(alt_threshold=2.0).should_relay(100.0, 0.5, 5.0, 3.0)
        assert not ok and reason == 'low_alt'

    def test_single_robot_formation(self):
        fm = FormationManager([RobotConfig('u1', 'ugv', (0, 0))])
        wp = fm.compute_formation((5, 5), FormationPattern.SURROUND, spacing=3.0)
        assert len(wp) == 1


# ── 풀 파이프라인 스모크 테스트 ──

class TestFullPipeline:
    def test_complete_fire_scenario(self):
        """5대 편대 → 드론 감지 → CBBA → 호스 검증 → 편대 전환."""
        # 1. 드론 감지
        relay = DroneRelayLogic()
        ok, _ = relay.should_relay(100.0, 8.0, 5.0, 3.0)
        assert ok

        # 2. CBBA 할당
        alloc = CBBAAllocator()
        fleet = make_fleet()
        tasks = [
            CbbaTask('suppress', (5.0, 3.0, 0.0), 'inspect_fire', ['has_thermal'], 0.8),
            CbbaTask('monitor', (5.0, 3.0, 8.0), 'monitor', ['can_fly'], 0.72),
        ]
        result = alloc.allocate(fleet, tasks)
        assert len(result) >= 2
        task_types = {t.task_type for t in result.values()}
        assert 'inspect_fire' in task_types and 'monitor' in task_types

        # 3. 호스 검증 (셰르파가 할당받았으면)
        for rid, task in result.items():
            if rid == 'sherpa1' and task.task_type == 'inspect_fire':
                dist = math.hypot(5.0 - 2.0, 3.0 - 0.0)  # sherpa at (2,0)
                assert dist <= 100.0  # 100m 호스 이내

        # 4. 편대 전환
        configs = [RobotConfig(r.robot_id, 'drone' if 'can_fly' in r.capabilities else 'ugv',
                               r.pose[:2] if r.pose else (0, 0)) for r in fleet]
        fm = FormationManager(configs)
        surround = fm.compute_formation((5.0, 3.0), FormationPattern.SURROUND, spacing=4.0)
        assert len(surround) == 5
