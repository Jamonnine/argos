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
"""CBBA ↔ 오케스트레이터 연동 통합 테스트.

테스트 범위:
  - 오케스트레이터 RobotRecord → CBBA RobotRecord 변환
  - 현재 상황 → CBBA Task 목록 생성
  - CBBA 할당 결과 → 오케스트레이터 명령 변환
  - 호스 제약 사후 검증
  - 재할당 변화 감지
  - 소방 시나리오 통합: 화재 감지 → CBBA 할당 → 진압 배정

rclpy 없이 순수 Python으로 실행 가능.

capability 명명 규칙 (cbba_allocator.py 기준):
  'has_thermal'  — 열화상 카메라 보유 (inspect_fire 임무 필수)
  'can_fly'      — 비행 가능 (monitor 임무 필수)
  'has_gripper'  — 파지 장치 보유 (rescue 임무 필수)
  'lidar'        — LiDAR 센서
  'radar'        — 레이더 센서
  'water_cannon' — 방수포
"""
import math
from collections import deque
from unittest.mock import MagicMock

import pytest

from argos_bringup.cbba_allocator import (
    CBBAAllocator,
    RobotRecord as CbbaRobotRecord,
    Task as CbbaTask,
)


# ── Mock 클래스 (rclpy 의존 제거) ──────────────────────────────

class MockPoint:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class MockPose:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.position = MockPoint(x, y, z)
        self.orientation = MockPoint(0, 0, 0)  # 단순화


class MockPoseStamped:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.pose = MockPose(x, y, z)
        self.header = MagicMock()


class MockFireAlert:
    def __init__(self, x, y, severity='medium', fire_type='general'):
        self.location = MagicMock()
        self.location.point = MockPoint(x, y, 0.0)
        self.severity = severity
        self.fire_type = fire_type
        self.max_temperature_kelvin = 573.15  # 300°C
        self.robot_id = 'argos1'
        self.header = MagicMock()


class MockVictim:
    def __init__(self, x, y):
        self.location = MagicMock()
        self.location.point = MockPoint(x, y, 0.0)


class OrchestratorRobotRecord:
    """오케스트레이터 RobotRecord 복제 (테스트용).

    orchestrator_node.py의 실제 RobotRecord와 동일한 필드 구조.
    """
    def __init__(self, robot_id, robot_type='ugv', capabilities=None,
                 pose=None, role='explore', comm_lost=False,
                 hose_remaining_m=-1.0, hose_kink_risk=0.0,
                 hose_charged=False, frontiers_remaining=0,
                 mission_lock=None, assigned_target=None):
        self.robot_id = robot_id
        self.robot_type = robot_type
        self.capabilities = capabilities or []
        self.pose = pose
        self.role = role
        self.comm_lost = comm_lost
        self.hose_remaining_m = hose_remaining_m
        self.hose_kink_risk = hose_kink_risk
        self.hose_charged = hose_charged
        self.hose_path = []
        self.frontiers_remaining = frontiers_remaining
        self.mission_lock = mission_lock
        self.assigned_target = assigned_target
        self.battery = 80.0
        self.state = 1  # EXPLORING
        self.last_seen = 100.0
        self.current_mission = 'explore'
        self.mission_progress = 0.5
        self.coverage = 30.0
        self.battery_warned = False
        self.battery_critical_acted = False
        self.last_seq = -1
        self.packet_loss_count = 0


# ── 헬퍼 함수 ───────────────────────────────────────────────────

# 로봇 타입별 기본 capabilities (cbba_allocator.py Task 정의 기준)
# inspect_fire → 'has_thermal' 필수
# monitor      → 'can_fly' 필수
# rescue       → 'has_gripper' 필수 (또는 [] 로 모든 로봇 허용)
_DEFAULT_CAPS = {
    'ugv':    ['has_thermal', 'lidar', 'depth', 'imu'],
    'drone':  ['can_fly', 'has_thermal', 'camera', 'imu'],
    'sherpa': ['has_thermal', 'lidar', 'depth', 'imu',
               'water_cannon', 'radar'],
}


def make_orch_robot(rid, rtype='ugv', caps=None, x=0.0, y=0.0,
                    role='explore', comm_lost=False,
                    hose_remaining=-1.0, frontiers=5):
    """오케스트레이터 RobotRecord 생성 헬퍼."""
    caps = caps or list(_DEFAULT_CAPS.get(rtype, []))
    z = 5.0 if rtype == 'drone' else 0.0
    pose = MockPoseStamped(x, y, z)
    return OrchestratorRobotRecord(
        robot_id=rid, robot_type=rtype, capabilities=caps,
        pose=pose, role=role, comm_lost=comm_lost,
        hose_remaining_m=hose_remaining, frontiers_remaining=frontiers,
    )


def convert_robots_to_cbba(robots_dict):
    """_convert_robots_to_cbba 로직 재현 (테스트용 독립 함수).

    orchestrator_node.py의 실제 변환 로직과 동일한 필터링 규칙:
      - comm_lost 로봇 제외
      - hose_supply 역할 로봇 제외
      - pose 없으면 None으로 전달
    """
    cbba_robots = []
    for rid, r in robots_dict.items():
        if r.comm_lost or r.role == 'hose_supply':
            continue
        pose_tuple = None
        if r.pose is not None:
            try:
                p = r.pose.pose.position
                pose_tuple = (p.x, p.y, p.z)
            except AttributeError:
                pose_tuple = None
        cbba_robots.append(CbbaRobotRecord(
            robot_id=rid,
            capabilities=list(r.capabilities),
            pose=pose_tuple,
        ))
    return cbba_robots


def build_task_list(fire_pt=None, severity='medium', fire_alerts=None,
                    robots_dict=None, victims=None):
    """_build_task_list 로직 재현 (테스트용 독립 함수).

    Task.required_capabilities는 cbba_allocator.py 가이드라인 준수:
      inspect_fire → ['has_thermal']
      monitor      → ['can_fly']
      explore      → []
      rescue       → []  (모든 로봇 구조 시도 가능)
    """
    tasks = []
    sev_map = {'low': 0.3, 'medium': 0.5, 'high': 0.8, 'critical': 1.0}

    if fire_pt is not None:
        fp = sev_map.get(severity, 0.5)
        # 지상 진압 임무: 열화상 카메라 보유 로봇 필수
        tasks.append(CbbaTask(
            task_id='fire_suppress_ground',
            location=(fire_pt.x, fire_pt.y, 0.0),
            task_type='inspect_fire',
            required_capabilities=['has_thermal'],
            priority=fp,
        ))
        # 항공 감시 임무: 드론(can_fly) 전용
        tasks.append(CbbaTask(
            task_id='fire_monitor_aerial',
            location=(fire_pt.x, fire_pt.y, 8.0),
            task_type='monitor',
            required_capabilities=['can_fly'],
            priority=fp * 0.9,
        ))

    if fire_alerts:
        processed = set()
        if fire_pt:
            processed.add((round(fire_pt.x, 1), round(fire_pt.y, 1)))
        for i, a in enumerate(fire_alerts[-5:]):
            loc = a.location.point
            key = (round(loc.x, 1), round(loc.y, 1))
            if key in processed:
                continue
            processed.add(key)
            ap = sev_map.get(getattr(a, 'severity', 'medium'), 0.5)
            tasks.append(CbbaTask(
                task_id=f'fire_alert_{i}',
                location=(loc.x, loc.y, 0.0),
                task_type='inspect_fire',
                required_capabilities=['has_thermal'],
                priority=ap * 0.8,
            ))

    if robots_dict and fire_pt is None:
        # 화재 없을 때만 탐색 임무 생성
        total = sum(r.frontiers_remaining for r in robots_dict.values())
        if total > 0:
            for rid, r in robots_dict.items():
                if r.comm_lost or r.frontiers_remaining == 0:
                    continue
                tasks.append(CbbaTask(
                    task_id=f'explore_{rid}',
                    location=(0.0, 0.0, 0.0),
                    task_type='explore',
                    required_capabilities=[],
                    priority=0.3,
                ))

    if victims:
        for i, v in enumerate(victims[-3:]):
            vp = getattr(v, 'location', None)
            if vp is None:
                continue
            tasks.append(CbbaTask(
                task_id=f'rescue_{i}',
                location=(vp.point.x, vp.point.y, 0.0),
                task_type='rescue',
                required_capabilities=[],
                priority=0.9,
            ))

    return tasks


# ── 변환 테스트 ──────────────────────────────────────────────────

class TestRobotConversion:
    """오케스트레이터 RobotRecord → CBBA RobotRecord 변환 검증."""

    def test_basic_ugv_conversion(self):
        """UGV 기본 변환: pose 추출, capabilities 보존."""
        robots = {'argos1': make_orch_robot('argos1', x=3.0, y=4.0)}
        result = convert_robots_to_cbba(robots)
        assert len(result) == 1
        assert result[0].robot_id == 'argos1'
        assert result[0].pose == (3.0, 4.0, 0.0)
        assert 'has_thermal' in result[0].capabilities

    def test_drone_conversion(self):
        """드론 변환: z=5.0 고도 포함 + can_fly capability."""
        robots = {'drone1': make_orch_robot('drone1', rtype='drone', x=1.0, y=2.0)}
        result = convert_robots_to_cbba(robots)
        assert len(result) == 1
        assert result[0].pose[2] == 5.0
        assert 'can_fly' in result[0].capabilities

    def test_comm_lost_excluded(self):
        """통신 두절 로봇 제외."""
        robots = {
            'argos1': make_orch_robot('argos1', comm_lost=True),
            'argos2': make_orch_robot('argos2', comm_lost=False),
        }
        result = convert_robots_to_cbba(robots)
        assert len(result) == 1
        assert result[0].robot_id == 'argos2'

    def test_hose_supply_excluded(self):
        """hose_supply 역할 로봇 제외."""
        robots = {
            'sherpa1': make_orch_robot('sherpa1', rtype='sherpa', role='hose_supply'),
            'sherpa2': make_orch_robot('sherpa2', rtype='sherpa', role='fire_attack'),
        }
        result = convert_robots_to_cbba(robots)
        assert len(result) == 1
        assert result[0].robot_id == 'sherpa2'

    def test_mixed_fleet_conversion(self):
        """이종 편대 전체 변환: 2UGV + 1드론 + 1셰르파."""
        robots = {
            'argos1': make_orch_robot('argos1', x=0.0, y=0.0),
            'argos2': make_orch_robot('argos2', x=5.0, y=0.0),
            'drone1': make_orch_robot('drone1', rtype='drone', x=2.0, y=3.0),
            'sherpa1': make_orch_robot('sherpa1', rtype='sherpa', x=1.0, y=1.0,
                                       hose_remaining=80.0),
        }
        result = convert_robots_to_cbba(robots)
        assert len(result) == 4
        ids = {r.robot_id for r in result}
        assert ids == {'argos1', 'argos2', 'drone1', 'sherpa1'}

    def test_no_pose_robot(self):
        """pose 없는 로봇: None으로 변환 (CBBA가 원점으로 처리)."""
        r = make_orch_robot('argos1')
        r.pose = None
        robots = {'argos1': r}
        result = convert_robots_to_cbba(robots)
        assert len(result) == 1
        assert result[0].pose is None

    def test_capabilities_preserved_exactly(self):
        """capabilities 목록이 정확히 복사되어야 함."""
        custom_caps = ['has_thermal', 'water_cannon', 'radar']
        robots = {'sherpa1': make_orch_robot('sherpa1', rtype='sherpa',
                                              caps=custom_caps)}
        result = convert_robots_to_cbba(robots)
        assert set(result[0].capabilities) == set(custom_caps)

    def test_both_comm_lost_and_hose_supply_excluded(self):
        """comm_lost + hose_supply 동시 → 두 조건 모두 적용."""
        robots = {
            'argos1': make_orch_robot('argos1', comm_lost=True),
            'sherpa1': make_orch_robot('sherpa1', rtype='sherpa', role='hose_supply'),
            'drone1': make_orch_robot('drone1', rtype='drone'),
        }
        result = convert_robots_to_cbba(robots)
        # drone1만 포함
        assert len(result) == 1
        assert result[0].robot_id == 'drone1'


# ── 태스크 생성 테스트 ───────────────────────────────────────────

class TestTaskGeneration:
    """현재 상황 → CBBA Task 목록 생성 검증."""

    def test_fire_creates_two_tasks(self):
        """화재 시 지상진압 + 항공감시 2개 임무 생성."""
        fire = MockPoint(5.0, 3.0)
        tasks = build_task_list(fire_pt=fire, severity='high')
        assert len(tasks) == 2
        types = {t.task_type for t in tasks}
        assert types == {'inspect_fire', 'monitor'}

    def test_fire_priority_mapping(self):
        """severity → priority 매핑 검증: low/medium/high/critical."""
        for sev, expected in [('low', 0.3), ('medium', 0.5),
                              ('high', 0.8), ('critical', 1.0)]:
            fire = MockPoint(0.0, 0.0)
            tasks = build_task_list(fire_pt=fire, severity=sev)
            ground = [t for t in tasks if t.task_id == 'fire_suppress_ground'][0]
            assert ground.priority == expected

    def test_monitor_lower_priority_than_suppress(self):
        """항공 감시는 지상 진압보다 약간 낮은 우선순위 (×0.9)."""
        fire = MockPoint(0.0, 0.0)
        tasks = build_task_list(fire_pt=fire, severity='critical')
        ground = [t for t in tasks if t.task_type == 'inspect_fire'][0]
        aerial = [t for t in tasks if t.task_type == 'monitor'][0]
        assert aerial.priority < ground.priority
        assert aerial.priority == pytest.approx(ground.priority * 0.9)

    def test_fire_task_capabilities_correct(self):
        """inspect_fire → has_thermal 필수, monitor → can_fly 필수."""
        fire = MockPoint(0.0, 0.0)
        tasks = build_task_list(fire_pt=fire)
        ground = [t for t in tasks if t.task_type == 'inspect_fire'][0]
        aerial = [t for t in tasks if t.task_type == 'monitor'][0]
        assert 'has_thermal' in ground.required_capabilities
        assert 'can_fly' in aerial.required_capabilities

    def test_explore_tasks_when_no_fire(self):
        """화재 없을 때: 프론티어가 있는 로봇에게만 탐색 임무 생성."""
        robots = {
            'argos1': make_orch_robot('argos1', frontiers=5),
            'argos2': make_orch_robot('argos2', frontiers=0),  # 프론티어 소진
        }
        tasks = build_task_list(robots_dict=robots)
        explore_tasks = [t for t in tasks if t.task_type == 'explore']
        assert len(explore_tasks) == 1  # frontiers>0 인 로봇만

    def test_no_explore_during_fire(self):
        """화재 중에는 탐색 임무 미생성."""
        fire = MockPoint(5.0, 5.0)
        robots = {'argos1': make_orch_robot('argos1', frontiers=10)}
        tasks = build_task_list(fire_pt=fire, robots_dict=robots)
        explore_tasks = [t for t in tasks if t.task_type == 'explore']
        assert len(explore_tasks) == 0

    def test_rescue_tasks_from_victims(self):
        """피해자 감지 시 구조 임무 생성, 우선순위 0.9."""
        victims = [MockVictim(3.0, 4.0), MockVictim(7.0, 8.0)]
        tasks = build_task_list(victims=victims)
        rescue_tasks = [t for t in tasks if t.task_type == 'rescue']
        assert len(rescue_tasks) == 2
        assert all(t.priority == 0.9 for t in rescue_tasks)

    def test_rescue_task_open_capabilities(self):
        """구조 임무 required_capabilities 빈 리스트 (모든 로봇 가능)."""
        victims = [MockVictim(1.0, 1.0)]
        tasks = build_task_list(victims=victims)
        rescue = [t for t in tasks if t.task_type == 'rescue'][0]
        assert rescue.required_capabilities == []

    def test_duplicate_fire_alerts_deduped(self):
        """동일 위치 화재 알림 중복 제거."""
        fire = MockPoint(5.0, 5.0)
        alerts = [
            MockFireAlert(5.0, 5.0, 'medium'),  # fire_pt(5,5)와 중복
            MockFireAlert(5.0, 5.0, 'high'),     # 동일 위치 중복
            MockFireAlert(10.0, 10.0, 'low'),    # 새 위치 → 추가됨
        ]
        tasks = build_task_list(fire_pt=fire, fire_alerts=alerts)
        fire_alert_tasks = [t for t in tasks if 'fire_alert' in t.task_id]
        # fire_pt (5,5)는 이미 처리됨 → 새 위치 (10,10)만 추가
        assert len(fire_alert_tasks) == 1

    def test_empty_situation_no_tasks(self):
        """화재·프론티어·피해자 없으면 임무 없음."""
        tasks = build_task_list()
        assert len(tasks) == 0

    def test_task_priority_valid_range(self):
        """생성된 모든 임무 priority가 0.0~1.0 범위 내."""
        fire = MockPoint(0.0, 0.0)
        victims = [MockVictim(1.0, 1.0)]
        alerts = [MockFireAlert(3.0, 3.0)]
        tasks = build_task_list(fire_pt=fire, severity='high',
                                fire_alerts=alerts, victims=victims)
        for t in tasks:
            assert 0.0 <= t.priority <= 1.0, \
                f'임무 {t.task_id} priority={t.priority} 범위 초과'


# ── E2E 할당 시나리오 ────────────────────────────────────────────

class TestE2EAllocation:
    """CBBA 할당기 E2E 시나리오 검증."""

    def test_fire_scenario_2ugv_1drone(self):
        """화재 시나리오: 2UGV + 1드론 → 지상진압(UGV) + 항공감시(드론).

        argos1(x=1)이 화점(x=2)에 더 가까우므로 inspect_fire 우선 배정.
        drone1은 can_fly 보유 → monitor 담당.
        """
        robots = {
            'argos1': make_orch_robot('argos1', x=1.0, y=0.0),
            'argos2': make_orch_robot('argos2', x=8.0, y=0.0),
            'drone1': make_orch_robot('drone1', rtype='drone', x=5.0, y=5.0),
        }
        fire = MockPoint(2.0, 1.0)
        cbba_robots = convert_robots_to_cbba(robots)
        tasks = build_task_list(fire_pt=fire, severity='high')

        allocator = CBBAAllocator()
        assignments = allocator.allocate(cbba_robots, tasks)

        # 최소 2대 할당 (지상 + 항공)
        assert len(assignments) >= 2
        # 드론은 can_fly → monitor 임무만 가능
        if 'drone1' in assignments:
            assert assignments['drone1'].task_type == 'monitor'
        # UGV는 can_fly 없음 → inspect_fire 또는 미할당
        for rid in ('argos1', 'argos2'):
            if rid in assignments:
                assert assignments[rid].task_type == 'inspect_fire'

    def test_nearest_robot_gets_fire_suppress(self):
        """화점에 가장 가까운 로봇이 지상 진압 임무 획득.

        argos1(x=0) vs argos2(x=20) — 화점(x=0.1), argos1이 명백히 유리.
        """
        robots = {
            'argos1': make_orch_robot('argos1', x=0.0, y=0.0),
            'argos2': make_orch_robot('argos2', x=20.0, y=0.0),
        }
        fire = MockPoint(0.1, 0.0)  # argos1 바로 옆
        cbba_robots = convert_robots_to_cbba(robots)
        tasks = build_task_list(fire_pt=fire, severity='critical')

        # inspect_fire만 필터 (monitor는 can_fly 없으면 배정 불가)
        inspect_tasks = [t for t in tasks if t.task_type == 'inspect_fire']
        allocator = CBBAAllocator()
        assignments = allocator.allocate(cbba_robots, inspect_tasks)

        assert 'argos1' in assignments
        assert assignments['argos1'].task_type == 'inspect_fire'

    def test_sherpa_with_hose_constraint(self):
        """셰르파(방수포+열화상) → 화재 진압 임무 할당 가능."""
        robots = {
            'sherpa1': make_orch_robot('sherpa1', rtype='sherpa', x=0.0, y=0.0,
                                       hose_remaining=50.0),
        }
        fire = MockPoint(30.0, 0.0)  # 30m 거리 (호스 50m 내)
        cbba_robots = convert_robots_to_cbba(robots)
        tasks = build_task_list(fire_pt=fire, severity='critical')

        # 셰르파는 has_thermal 보유 → inspect_fire 가능
        inspect_tasks = [t for t in tasks if t.task_type == 'inspect_fire']
        allocator = CBBAAllocator()
        assignments = allocator.allocate(cbba_robots, inspect_tasks)

        assert 'sherpa1' in assignments
        assert assignments['sherpa1'].task_type == 'inspect_fire'

    def test_hose_supply_robot_not_allocated(self):
        """hose_supply 전담 로봇은 CBBA 할당 대상 아님."""
        robots = {
            'sherpa1': make_orch_robot('sherpa1', rtype='sherpa',
                                       role='hose_supply', x=0.0),
            'argos1': make_orch_robot('argos1', x=5.0),
        }
        cbba_robots = convert_robots_to_cbba(robots)
        # sherpa1 제외 확인
        ids = {r.robot_id for r in cbba_robots}
        assert 'sherpa1' not in ids
        assert 'argos1' in ids

    def test_all_comm_lost_empty_allocation(self):
        """전원 통신 두절 → 빈 할당."""
        robots = {
            'argos1': make_orch_robot('argos1', comm_lost=True),
            'argos2': make_orch_robot('argos2', comm_lost=True),
        }
        fire = MockPoint(5.0, 5.0)
        cbba_robots = convert_robots_to_cbba(robots)
        tasks = build_task_list(fire_pt=fire, severity='high')

        allocator = CBBAAllocator()
        assignments = allocator.allocate(cbba_robots, tasks)
        assert len(assignments) == 0

    def test_reallocation_detects_change(self):
        """재할당 변화 감지 — task_id 변경 시 다름으로 판정."""
        old = {'argos1': CbbaTask('t1', (0, 0, 0), 'explore', [], 0.3)}
        new = {'argos1': CbbaTask('t2', (5, 5, 0), 'inspect_fire',
                                   ['has_thermal'], 0.8)}

        old_map = {k: v.task_id for k, v in old.items()}
        new_map = {k: v.task_id for k, v in new.items()}
        assert old_map != new_map

    def test_reallocation_no_change(self):
        """동일 task_id → 변화 없음으로 판정."""
        t1 = CbbaTask('t1', (0, 0, 0), 'explore', [], 0.3)
        t2 = CbbaTask('t1', (0, 0, 0), 'explore', [], 0.3)
        old = {'argos1': t1}
        new = {'argos1': t2}

        old_map = {k: v.task_id for k, v in old.items()}
        new_map = {k: v.task_id for k, v in new.items()}
        assert old_map == new_map

    def test_multi_fire_multi_robot_allocation(self):
        """다중 화점 + 다중 로봇 시나리오: 중복 없는 할당."""
        robots = {
            'argos1': make_orch_robot('argos1', x=0.0, y=0.0),
            'argos2': make_orch_robot('argos2', x=10.0, y=0.0),
            'drone1': make_orch_robot('drone1', rtype='drone', x=5.0, y=5.0),
            'drone2': make_orch_robot('drone2', rtype='drone', x=5.0, y=-5.0),
        }
        fire = MockPoint(2.0, 1.0)
        alerts = [
            MockFireAlert(8.0, 2.0, 'medium'),  # 추가 화점
        ]
        cbba_robots = convert_robots_to_cbba(robots)
        tasks = build_task_list(fire_pt=fire, severity='high', fire_alerts=alerts)

        allocator = CBBAAllocator()
        assignments = allocator.allocate(cbba_robots, tasks)

        # 할당된 임무 간 중복 없음
        task_ids = [t.task_id for t in assignments.values()]
        assert len(task_ids) == len(set(task_ids)), '동일 임무 중복 할당 금지'

    def test_explore_allocation_without_fire(self):
        """탐색 모드: 화재 없을 때 프론티어 있는 로봇 수만큼 explore 임무 생성.

        CBBA는 capability 제약 없는 explore 임무에 모든 로봇 입찰 허용.
        생성된 임무 수(2개)만큼 할당되며, 임무 수가 상한선임을 검증.
        """
        robots = {
            'argos1': make_orch_robot('argos1', x=0.0, frontiers=5),
            'argos2': make_orch_robot('argos2', x=10.0, frontiers=3),
            'drone1': make_orch_robot('drone1', rtype='drone', x=5.0, frontiers=0),
        }
        cbba_robots = convert_robots_to_cbba(robots)
        tasks = build_task_list(robots_dict=robots)

        # 임무 수 확인: frontiers>0 인 argos1, argos2만 임무 생성됨
        explore_tasks = [t for t in tasks if t.task_type == 'explore']
        assert len(explore_tasks) == 2  # argos1, argos2만 (drone1 frontiers=0)

        allocator = CBBAAllocator()
        assignments = allocator.allocate(cbba_robots, tasks)

        # 할당 수 ≤ 임무 수 (2개), 중복 없음
        explore_assigned = [
            rid for rid, t in assignments.items() if t.task_type == 'explore'
        ]
        assert len(explore_assigned) <= 2
        task_ids = [t.task_id for t in assignments.values()]
        assert len(task_ids) == len(set(task_ids)), '동일 임무 중복 할당 금지'

    def test_rescue_priority_over_explore(self):
        """구조 임무(priority=0.9)가 탐색(priority=0.3)보다 우선.

        CBBA 비용 = max(0, dist*1.0 - priority*2.0).
        rescue: 로봇 바로 옆(dist≈0) → cost = max(0, 0 - 0.9*2) = 0.
        explore: 먼 위치(dist=20) → cost = max(0, 20 - 0.3*2) = 19.4.
        rescue 비용(0) < explore 비용(19.4) → rescue 선택.
        """
        robots = {'argos1': make_orch_robot('argos1', x=0.0, frontiers=5)}
        victims = [MockVictim(0.0, 0.0)]  # 로봇 바로 위치 (dist=0)
        cbba_robots = convert_robots_to_cbba(robots)

        # explore 임무: 멀리 있어 높은 거리 비용
        explore_task = CbbaTask(
            task_id='explore_argos1',
            location=(20.0, 0.0, 0.0),  # 20m 거리 → cost=20-0.6=19.4
            task_type='explore',
            required_capabilities=[],
            priority=0.3,
        )
        rescue_task = CbbaTask(
            task_id='rescue_0',
            location=(0.0, 0.0, 0.0),   # 0m 거리 → cost=max(0,-1.8)=0
            task_type='rescue',
            required_capabilities=[],
            priority=0.9,
        )
        tasks = [explore_task, rescue_task]

        allocator = CBBAAllocator()
        assignments = allocator.allocate(cbba_robots, tasks)

        # 단일 로봇 → 비용 낮은 rescue 선택 (cost=0 < explore cost=19.4)
        assert 'argos1' in assignments
        assert assignments['argos1'].task_type == 'rescue', \
            '구조 임무(cost=0)가 탐색(cost=19.4)보다 우선해야 함'

    def test_drone_only_gets_monitor_not_inspect(self):
        """드론은 has_thermal 없으면 inspect_fire 배정 불가.

        드론 capabilities: ['can_fly', 'camera', 'imu'] — has_thermal 제거.
        """
        no_thermal_drone_caps = ['can_fly', 'camera', 'imu']
        robots = {
            'drone1': make_orch_robot('drone1', rtype='drone',
                                      caps=no_thermal_drone_caps)
        }
        fire = MockPoint(0.0, 0.0)
        cbba_robots = convert_robots_to_cbba(robots)
        tasks = build_task_list(fire_pt=fire)

        allocator = CBBAAllocator()
        assignments = allocator.allocate(cbba_robots, tasks)

        if 'drone1' in assignments:
            # has_thermal 없으므로 inspect_fire 불가
            assert assignments['drone1'].task_type != 'inspect_fire'
            assert assignments['drone1'].task_type == 'monitor'

    def test_no_capable_robot_leaves_task_unassigned(self):
        """필요 capability 보유 로봇 없는 임무 → 미할당 (오류 없이).

        UGV만 있는데 monitor(can_fly 필수) 임무 → 미할당.
        """
        robots = {'argos1': make_orch_robot('argos1')}  # UGV, can_fly 없음
        tasks = [
            CbbaTask('monitor_only', (5.0, 5.0, 8.0), 'monitor',
                     ['can_fly'], 0.8)
        ]
        cbba_robots = convert_robots_to_cbba(robots)

        allocator = CBBAAllocator()
        assignments = allocator.allocate(cbba_robots, tasks)

        # 임무 미할당 (argos1은 can_fly 없음)
        assert 'argos1' not in assignments

    def test_single_robot_single_task_allocation(self):
        """로봇 1대, 임무 1개 → 정상 할당."""
        robots = {'argos1': make_orch_robot('argos1', x=0.0)}
        tasks = [
            CbbaTask('t1', (3.0, 4.0, 0.0), 'inspect_fire',
                     ['has_thermal'], 0.8)
        ]
        cbba_robots = convert_robots_to_cbba(robots)

        allocator = CBBAAllocator()
        assignments = allocator.allocate(cbba_robots, tasks)

        assert 'argos1' in assignments
        assert assignments['argos1'].task_id == 't1'

    def test_more_robots_than_tasks(self):
        """로봇 수 > 임무 수 → 임무당 1로봇 할당, 초과 로봇은 미할당."""
        robots = {
            'argos1': make_orch_robot('argos1', x=0.0),
            'argos2': make_orch_robot('argos2', x=5.0),
            'argos3': make_orch_robot('argos3', x=10.0),
        }
        # 임무 1개만
        tasks = [
            CbbaTask('t1', (2.0, 0.0, 0.0), 'inspect_fire',
                     ['has_thermal'], 0.5)
        ]
        cbba_robots = convert_robots_to_cbba(robots)

        allocator = CBBAAllocator()
        assignments = allocator.allocate(cbba_robots, tasks)

        # 임무 1개 → 최대 1대 할당
        assert len(assignments) == 1
        # 할당된 로봇의 task_id 검증
        assert list(assignments.values())[0].task_id == 't1'
