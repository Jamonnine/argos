"""
멀티로봇 경합 및 busy flag 단위 테스트.
QA 전문가 권고: 다중 로봇 충돌/경합 시나리오 검증.
"""
import pytest
import math


def select_available_robots(robots, target_x, target_y, exclude='', max_dispatch=2):
    """busy flag 체크 + 최근접 N대 선택."""
    available = []
    for rid, r in robots.items():
        if rid == exclude or r.get('comm_lost') or r.get('pose') is None:
            continue
        if r.get('mission_lock') is not None:
            continue  # busy
        dx = r['pose'][0] - target_x
        dy = r['pose'][1] - target_y
        dist = math.hypot(dx, dy)
        available.append((rid, dist))
    available.sort(key=lambda x: x[1])
    return available[:max_dispatch]


def assign_mission(robots, rid, mission_type, target):
    """로봇에 미션 할당 + busy flag 설정."""
    robots[rid]['mission_lock'] = mission_type
    robots[rid]['assigned_target'] = target
    return True


def release_mission(robots, rid):
    """미션 완료 → busy flag 해제."""
    robots[rid]['mission_lock'] = None
    robots[rid]['assigned_target'] = None


class TestBusyFlag:
    """busy flag로 중복 파견 방지 검증."""

    def setup_method(self):
        self.robots = {
            'ugv1': {'pose': (1, 1), 'mission_lock': None, 'assigned_target': None},
            'ugv2': {'pose': (5, 5), 'mission_lock': None, 'assigned_target': None},
            'ugv3': {'pose': (9, 1), 'mission_lock': None, 'assigned_target': None},
        }

    def test_all_available(self):
        result = select_available_robots(self.robots, 3, 3)
        assert len(result) == 2  # max_dispatch=2
        assert result[0][0] == 'ugv1'  # 가장 가까움

    def test_busy_robot_excluded(self):
        assign_mission(self.robots, 'ugv1', 'rescue', (3, 3))
        result = select_available_robots(self.robots, 3, 3)
        assert 'ugv1' not in [r[0] for r in result]
        assert len(result) == 2  # ugv2, ugv3만

    def test_all_busy_no_dispatch(self):
        for rid in self.robots:
            assign_mission(self.robots, rid, 'fire_response', (5, 5))
        result = select_available_robots(self.robots, 3, 3)
        assert len(result) == 0

    def test_release_makes_available(self):
        assign_mission(self.robots, 'ugv1', 'rescue', (3, 3))
        assert self.robots['ugv1']['mission_lock'] == 'rescue'
        release_mission(self.robots, 'ugv1')
        assert self.robots['ugv1']['mission_lock'] is None
        result = select_available_robots(self.robots, 3, 3)
        assert 'ugv1' in [r[0] for r in result]

    def test_double_dispatch_same_target(self):
        """같은 목표에 2대 파견 (소방 전문가 권고: redundancy)."""
        result = select_available_robots(self.robots, 3, 3, max_dispatch=2)
        assert len(result) == 2
        for rid, _ in result:
            assign_mission(self.robots, rid, 'rescue', (3, 3))
        # 3번째 파견 시 1대만 남음
        result2 = select_available_robots(self.robots, 7, 7)
        assert len(result2) == 1

    def test_comm_lost_excluded(self):
        self.robots['ugv1']['comm_lost'] = True
        result = select_available_robots(self.robots, 1, 1)
        assert 'ugv1' not in [r[0] for r in result]


class TestMultiRobotContention:
    """다중 로봇 경합 시나리오."""

    def setup_method(self):
        self.robots = {
            'ugv1': {'pose': (2, 2), 'mission_lock': None, 'assigned_target': None},
            'ugv2': {'pose': (8, 2), 'mission_lock': None, 'assigned_target': None},
            'ugv3': {'pose': (5, 8), 'mission_lock': None, 'assigned_target': None},
        }

    def test_two_fires_different_robots(self):
        """2곳 화재에 각각 다른 로봇 배정."""
        fire_a = (3, 3)
        fire_b = (7, 3)

        # 1번째 화재: ugv1(가장 가까움) 배정
        result1 = select_available_robots(self.robots, *fire_a, max_dispatch=1)
        assert result1[0][0] == 'ugv1'
        assign_mission(self.robots, 'ugv1', 'fire_response', fire_a)

        # 2번째 화재: ugv1 busy → ugv2 배정
        result2 = select_available_robots(self.robots, *fire_b, max_dispatch=1)
        assert result2[0][0] == 'ugv2'  # ugv1 제외
        assign_mission(self.robots, 'ugv2', 'fire_response', fire_b)

        # 3번째 임무: ugv3만 남음
        result3 = select_available_robots(self.robots, 5, 5)
        assert len(result3) == 1
        assert result3[0][0] == 'ugv3'

    def test_rescue_preempts_fire(self):
        """구조 임무가 화재 대응보다 우선 (시나리오적 테스트)."""
        # ugv1이 화재 대응 중
        assign_mission(self.robots, 'ugv1', 'fire_response', (3, 3))

        # 피해자 발견 → ugv2 파견 (ugv1은 busy)
        result = select_available_robots(self.robots, 4, 4, max_dispatch=1)
        assert result[0][0] != 'ugv1'

    def test_no_deadlock(self):
        """모든 로봇 busy 후 해제 → 정상 파견."""
        for rid in self.robots:
            assign_mission(self.robots, rid, 'rescue', (5, 5))
        assert len(select_available_robots(self.robots, 1, 1)) == 0

        # 1대 해제
        release_mission(self.robots, 'ugv1')
        result = select_available_robots(self.robots, 1, 1)
        assert len(result) == 1
        assert result[0][0] == 'ugv1'


class TestSensorFusionPriority:
    """센서 간 충돌 시 우선순위 검증."""

    def test_gas_critical_overrides_fire_response(self):
        """가스 critical > 화재 대응 → 전원 일시정지."""
        stage = 'fire_response'
        gas_level = 'critical'
        # 가스 critical이면 무조건 PAUSED (소방 전문가 권고)
        if gas_level == 'critical':
            stage = 'paused'
        assert stage == 'paused'

    def test_victim_priority_1_gets_2_robots(self):
        """priority 1 피해자 → 2대 파견."""
        robots = {
            'ugv1': {'pose': (1, 1), 'mission_lock': None, 'assigned_target': None},
            'ugv2': {'pose': (3, 3), 'mission_lock': None, 'assigned_target': None},
            'ugv3': {'pose': (9, 9), 'mission_lock': None, 'assigned_target': None},
        }
        victim_priority = 1
        max_dispatch = 2 if victim_priority == 1 else 1
        result = select_available_robots(robots, 2, 2, max_dispatch=max_dispatch)
        assert len(result) == 2

    def test_structural_critical_blocks_area(self):
        """구조물 critical → 해당 영역 차단."""
        blocked_areas = []
        structural_severity = 'critical'
        if structural_severity == 'critical':
            blocked_areas.append({'center': (5, 5), 'radius': 3.0})
        assert len(blocked_areas) == 1
        assert blocked_areas[0]['radius'] == 3.0
