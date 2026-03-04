"""
Orchestrator 핵심 로직 단위 테스트.
rclpy 없이 순수 Python으로 오케스트레이터의 핵심 판단 로직을 검증.
"""

import pytest
import math


# ── 오케스트레이터 내부 상수 재현 ──

class MockRobotStatus:
    STATE_IDLE = 0
    STATE_EXPLORING = 1
    STATE_ON_MISSION = 2
    STATE_RETURNING = 3
    STATE_ERROR = 4
    STATE_COMM_LOST = 5


class MockMissionState:
    STAGE_INIT = 0
    STAGE_EXPLORING = 1
    STAGE_FIRE_RESPONSE = 2
    STAGE_RETURNING = 3
    STAGE_COMPLETE = 4
    STAGE_PAUSED = 5


class RobotRecord:
    """오케스트레이터 내부 RobotRecord의 테스트용 복제."""

    def __init__(self, robot_id, **kwargs):
        self.robot_id = robot_id
        self.robot_type = kwargs.get('robot_type', 'ugv')
        self.state = kwargs.get('state', MockRobotStatus.STATE_IDLE)
        self.last_seen = kwargs.get('last_seen', 0.0)
        self.battery = kwargs.get('battery', 100.0)
        self.current_mission = kwargs.get('current_mission', 'idle')
        self.mission_progress = 0.0
        self.frontiers_remaining = kwargs.get('frontiers_remaining', 0)
        self.coverage = kwargs.get('coverage', 0.0)
        self.capabilities = []
        self.pose = kwargs.get('pose', None)
        self.comm_lost = kwargs.get('comm_lost', False)
        self.battery_warned = False
        self.battery_critical_acted = False


# ── 스테이지 전환 로직 (orchestrator_node.py에서 추출) ──

def check_stage_init(robots):
    """INIT → EXPLORING 전환 조건: 모든 로봇이 1회 이상 상태 보고."""
    return all(r.last_seen > 0 for r in robots.values())


def check_stage_exploring(robots):
    """EXPLORING → RETURNING 전환 조건: 모든 활성 로봇이 탐색 완료."""
    active_robots = [
        r for r in robots.values()
        if not r.comm_lost and r.last_seen > 0
    ]
    return (
        len(active_robots) > 0
        and all(r.current_mission == 'complete' for r in active_robots)
    )


def check_battery(battery, critical=15.0, warning=30.0):
    """배터리 수준 판정."""
    if battery <= critical:
        return 'critical'
    elif battery <= warning:
        return 'warning'
    return 'ok'


def select_nearest_ugv(robots, fire_x, fire_y):
    """화점에 가장 가까운 UGV 선정 (orchestrator._execute_fire_response 로직)."""
    best_ugv = None
    best_dist = float('inf')
    for rid, r in robots.items():
        if r.robot_type != 'ugv' or r.comm_lost or r.pose is None:
            continue
        dx = r.pose[0] - fire_x
        dy = r.pose[1] - fire_y
        dist = math.hypot(dx, dy)
        if dist < best_dist:
            best_dist = dist
            best_ugv = rid
    return best_ugv, best_dist


def fire_severity_rank(severity):
    """심각도 랭킹 (에스컬레이션 판단용)."""
    rank = {'low': 0, 'medium': 1, 'high': 2, 'critical': 3}
    return rank.get(severity, -1)


# ══════════════════════════════════════════════════
# 테스트
# ══════════════════════════════════════════════════

class TestStageTransitions:

    def test_init_to_exploring_all_seen(self):
        """모든 로봇이 상태를 보고하면 EXPLORING으로 전환."""
        robots = {
            'argos1': RobotRecord('argos1', last_seen=1.0),
            'argos2': RobotRecord('argos2', last_seen=2.0),
        }
        assert check_stage_init(robots) is True

    def test_init_stays_if_not_all_seen(self):
        """일부 로봇만 보고하면 INIT 유지."""
        robots = {
            'argos1': RobotRecord('argos1', last_seen=1.0),
            'argos2': RobotRecord('argos2', last_seen=0.0),
        }
        assert check_stage_init(robots) is False

    def test_exploring_to_returning_all_complete(self):
        """모든 로봇 current_mission='complete'이면 RETURNING."""
        robots = {
            'argos1': RobotRecord('argos1', last_seen=1.0, current_mission='complete'),
            'argos2': RobotRecord('argos2', last_seen=2.0, current_mission='complete'),
        }
        assert check_stage_exploring(robots) is True

    def test_exploring_stays_if_some_exploring(self):
        """일부 로봇이 아직 탐색 중이면 EXPLORING 유지."""
        robots = {
            'argos1': RobotRecord('argos1', last_seen=1.0, current_mission='complete'),
            'argos2': RobotRecord('argos2', last_seen=2.0, current_mission='exploring'),
        }
        assert check_stage_exploring(robots) is False

    def test_exploring_no_early_return_for_idle(self):
        """STATE_IDLE인 로봇이 있어도 current_mission이 'complete'가 아니면 전환 안 함.
        이전 버그: STATE_IDLE만으로 '완료'로 오인하던 문제 (PR #1에서 수정)."""
        robots = {
            'argos1': RobotRecord('argos1', last_seen=1.0,
                                  state=MockRobotStatus.STATE_IDLE,
                                  current_mission='idle'),
            'argos2': RobotRecord('argos2', last_seen=2.0,
                                  state=MockRobotStatus.STATE_IDLE,
                                  current_mission='idle'),
        }
        assert check_stage_exploring(robots) is False

    def test_exploring_ignores_comm_lost(self):
        """통신 두절 로봇은 완료 판정에서 제외."""
        robots = {
            'argos1': RobotRecord('argos1', last_seen=1.0, current_mission='complete'),
            'argos2': RobotRecord('argos2', last_seen=2.0, comm_lost=True,
                                  current_mission='exploring'),
        }
        assert check_stage_exploring(robots) is True

    def test_exploring_empty_active_robots(self):
        """모든 로봇이 통신 두절이면 전환하지 않음."""
        robots = {
            'argos1': RobotRecord('argos1', last_seen=1.0, comm_lost=True),
        }
        assert check_stage_exploring(robots) is False


class TestBatteryCheck:

    def test_battery_ok(self):
        assert check_battery(80.0) == 'ok'

    def test_battery_warning(self):
        assert check_battery(25.0) == 'warning'

    def test_battery_critical(self):
        assert check_battery(10.0) == 'critical'

    def test_battery_exact_warning_threshold(self):
        assert check_battery(30.0) == 'warning'

    def test_battery_exact_critical_threshold(self):
        assert check_battery(15.0) == 'critical'

    def test_battery_zero(self):
        assert check_battery(0.0) == 'critical'


class TestFireResponse:

    def test_select_nearest_ugv(self):
        """화점에 가장 가까운 UGV 선택."""
        robots = {
            'argos1': RobotRecord('argos1', pose=(1.0, 1.0)),
            'argos2': RobotRecord('argos2', pose=(8.0, 8.0)),
        }
        best, dist = select_nearest_ugv(robots, fire_x=2.0, fire_y=1.0)
        assert best == 'argos1'
        assert dist == pytest.approx(1.0)

    def test_select_nearest_ugv_ignores_drones(self):
        """드론은 UGV 선정에서 제외."""
        robots = {
            'drone1': RobotRecord('drone1', robot_type='drone', pose=(2.0, 1.0)),
            'argos1': RobotRecord('argos1', robot_type='ugv', pose=(5.0, 5.0)),
        }
        best, _ = select_nearest_ugv(robots, fire_x=2.0, fire_y=1.0)
        assert best == 'argos1'

    def test_select_nearest_ugv_ignores_comm_lost(self):
        """통신 두절 로봇 제외."""
        robots = {
            'argos1': RobotRecord('argos1', pose=(1.0, 1.0), comm_lost=True),
            'argos2': RobotRecord('argos2', pose=(8.0, 8.0)),
        }
        best, _ = select_nearest_ugv(robots, fire_x=2.0, fire_y=1.0)
        assert best == 'argos2'

    def test_select_nearest_ugv_no_pose(self):
        """위치 미보고 로봇 제외."""
        robots = {
            'argos1': RobotRecord('argos1', pose=None),
        }
        best, _ = select_nearest_ugv(robots, fire_x=2.0, fire_y=1.0)
        assert best is None

    def test_fire_severity_ranking(self):
        assert fire_severity_rank('low') < fire_severity_rank('medium')
        assert fire_severity_rank('medium') < fire_severity_rank('high')
        assert fire_severity_rank('high') < fire_severity_rank('critical')

    def test_fire_escalation_higher_severity(self):
        """더 높은 심각도가 에스컬레이션 트리거."""
        current = fire_severity_rank('medium')
        new = fire_severity_rank('critical')
        assert new > current  # 에스컬레이션 발생

    def test_fire_escalation_same_severity(self):
        """같은 심각도는 에스컬레이션 없음."""
        current = fire_severity_rank('high')
        new = fire_severity_rank('high')
        assert not (new > current)
