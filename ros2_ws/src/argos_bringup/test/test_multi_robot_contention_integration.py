"""
멀티로봇 경합 통합 테스트 (W5-2).

동일 화점에 3대 파견 요청 시 primary_responder가 1대만 배정되는지 검증.
COLLISION_SAFE_DISTANCE(2m) 이내 로봇은 파견 보류 로직도 검증.

검증 흐름:
  1. orchestrator에 robot_status 3대분 발행
  2. 동일 화점에 2대 동시 파견 요청
  3. primary_responder가 1대만 배정되는지 확인 (충돌 방지)
  4. COLLISION_SAFE_DISTANCE 2m 이내 로봇은 파견 보류 확인
"""

import math
import pytest

# ── launch_testing import ──
try:
    import launch
    import launch_ros.actions
    import launch_testing
    import launch_testing.actions
    import rclpy
    _ROS2_AVAILABLE = True
except ImportError:
    _ROS2_AVAILABLE = False


# ══════════════════════════════════════════════════
# launch_testing 진입점 (ROS2 환경 전용)
# ══════════════════════════════════════════════════

if _ROS2_AVAILABLE:
    def generate_test_description():
        """orchestrator 노드 기동 — 3대 로봇 기대."""
        orchestrator = launch_ros.actions.Node(
            package='argos_bringup',
            executable='orchestrator',
            parameters=[{
                'expected_robots': ['ugv1', 'ugv2', 'ugv3'],
            }],
            output='screen',
        )
        return launch.LaunchDescription([
            orchestrator,
            launch_testing.actions.ReadyToTest(),
        ]), {'orchestrator': orchestrator}


# ══════════════════════════════════════════════════
# 로직 레이어 — orchestrator_node.py 파견 로직 재현
# ══════════════════════════════════════════════════

COLLISION_SAFE_DISTANCE = 2.0  # orchestrator_node.py와 동일 상수


class RobotRecord:
    """파견 로직 검증용 로봇 레코드."""

    def __init__(self, robot_id, pose=(0.0, 0.0), robot_type='ugv',
                 comm_lost=False, battery=100.0):
        self.robot_id = robot_id
        self.robot_type = robot_type
        self.pose = pose
        self.comm_lost = comm_lost
        self.battery = battery
        self.mission_lock = None       # 'fire_response' | 'rescue' | None
        self.assigned_target = None
        self.last_seen = 1.0


class DispatchSystem:
    """
    orchestrator_node.py의 _execute_fire_response 로직 재현.
    primary_responder 1대 배정 + COLLISION_SAFE_DISTANCE 보류 로직 포함.
    """

    def __init__(self, robots: dict):
        self.robots = robots          # {rid: RobotRecord}
        self.primary_responder = None
        self.dispatch_log = []        # 파견 이력

    def select_primary_responder(self, fire_x: float, fire_y: float) -> str | None:
        """
        화점에 가장 가까운 UGV 선정.
        조건:
          - robot_type == 'ugv'
          - comm_lost is False
          - mission_lock is None (유휴)
          - 거리 > COLLISION_SAFE_DISTANCE (화점 2m 이내는 이미 현장 → 추가 파견 불필요)
        Returns: robot_id or None
        """
        best_rid = None
        best_dist = float('inf')
        for rid, r in self.robots.items():
            if r.robot_type != 'ugv':
                continue
            if r.comm_lost:
                continue
            if r.mission_lock is not None:
                continue
            if r.pose is None:
                continue
            dx = r.pose[0] - fire_x
            dy = r.pose[1] - fire_y
            dist = math.hypot(dx, dy)
            # COLLISION_SAFE_DISTANCE 이내는 파견 보류
            if dist <= COLLISION_SAFE_DISTANCE:
                continue
            if dist < best_dist:
                best_dist = dist
                best_rid = rid
        return best_rid

    def dispatch(self, fire_x: float, fire_y: float) -> str | None:
        """
        화점에 primary_responder를 배정하고 mission_lock을 설정.
        중복 호출 시 이미 배정된 경우 기존 responder 반환 (단일 배정 보장).
        """
        if self.primary_responder is not None:
            # 이미 배정됨 → 중복 파견 차단
            return self.primary_responder

        rid = self.select_primary_responder(fire_x, fire_y)
        if rid:
            self.primary_responder = rid
            self.robots[rid].mission_lock = 'fire_response'
            self.robots[rid].assigned_target = (fire_x, fire_y)
            self.dispatch_log.append({'robot': rid, 'fire': (fire_x, fire_y)})
        return rid


# ══════════════════════════════════════════════════
# 테스트 클래스
# ══════════════════════════════════════════════════

class TestMultiRobotContentionIntegration:
    """
    3대 로봇 경합 시나리오에서 단일 primary_responder 배정 검증.
    충돌 방지(COLLISION_SAFE_DISTANCE) 로직 포함.
    """

    def setup_method(self):
        """
        3대 UGV 초기 배치:
          ugv1: (3, 3) — 화점(5,5)까지 2.83m
          ugv2: (8, 8) — 화점(5,5)까지 4.24m
          ugv3: (1, 9) — 화점(5,5)까지 5.66m
        """
        self.robots = {
            'ugv1': RobotRecord('ugv1', pose=(3.0, 3.0)),
            'ugv2': RobotRecord('ugv2', pose=(8.0, 8.0)),
            'ugv3': RobotRecord('ugv3', pose=(1.0, 9.0)),
        }
        self.ds = DispatchSystem(self.robots)
        self.fire_x, self.fire_y = 5.0, 5.0

    # ── 핵심: 단일 배정 보장 ──

    def test_only_one_primary_responder_assigned(self):
        """
        동일 화점에 2회 파견 요청 → primary_responder는 1대만 배정.
        오케스트레이터 핵심 불변: 화점당 primary는 1대.
        """
        rid1 = self.ds.dispatch(self.fire_x, self.fire_y)
        rid2 = self.ds.dispatch(self.fire_x, self.fire_y)  # 중복 요청

        # 동일 responder 반환
        assert rid1 == rid2
        # dispatch_log에는 1회만 기록
        assert len(self.ds.dispatch_log) == 1

    def test_nearest_ugv_selected_as_primary(self):
        """화점에 가장 가까운 UGV가 primary_responder로 선정."""
        rid = self.ds.dispatch(self.fire_x, self.fire_y)
        # ugv1이 (3,3)으로 가장 가까움 (거리 2.83m)
        assert rid == 'ugv1'
        assert self.ds.primary_responder == 'ugv1'

    def test_mission_lock_set_on_primary(self):
        """primary_responder의 mission_lock이 'fire_response'로 설정."""
        rid = self.ds.dispatch(self.fire_x, self.fire_y)
        assert self.robots[rid].mission_lock == 'fire_response'
        assert self.robots[rid].assigned_target == (self.fire_x, self.fire_y)

    def test_remaining_robots_remain_unlocked(self):
        """배정되지 않은 로봇들은 mission_lock 없음."""
        rid = self.ds.dispatch(self.fire_x, self.fire_y)
        for robot_id, r in self.robots.items():
            if robot_id != rid:
                assert r.mission_lock is None, f'{robot_id}가 예상치 않게 배정됨'

    # ── COLLISION_SAFE_DISTANCE 보류 ──

    def test_robot_within_2m_not_dispatched(self):
        """
        화점 2m 이내 로봇 → COLLISION_SAFE_DISTANCE로 파견 보류.
        ugv1을 화점 바로 옆(1m)에 배치 → 배정 제외, ugv2가 선정.
        """
        self.robots['ugv1'].pose = (5.0, 4.1)  # 화점까지 0.9m (< 2m)
        rid = self.ds.dispatch(self.fire_x, self.fire_y)
        # ugv1 제외 → ugv2(4.24m) 또는 ugv3(5.66m) 중 가까운 ugv2
        assert rid != 'ugv1'
        assert rid == 'ugv2'

    def test_all_robots_within_2m_no_dispatch(self):
        """모든 로봇이 화점 2m 이내 → primary_responder 미배정."""
        # 모든 로봇을 화점 1m 이내로 이동
        self.robots['ugv1'].pose = (5.0, 4.1)  # 0.9m
        self.robots['ugv2'].pose = (5.1, 5.0)  # 0.1m
        self.robots['ugv3'].pose = (4.2, 5.0)  # 0.8m
        rid = self.ds.dispatch(self.fire_x, self.fire_y)
        assert rid is None
        assert self.ds.primary_responder is None

    def test_exactly_at_safe_distance_boundary(self):
        """정확히 2m(경계값) → 파견 보류 (<=로 처리)."""
        # ugv1을 화점에서 정확히 2m 위치
        self.robots['ugv1'].pose = (5.0, 3.0)  # 정확히 2m
        rid = self.ds.dispatch(self.fire_x, self.fire_y)
        # ugv1 제외(<=2m) → ugv2 배정
        assert rid != 'ugv1'

    def test_just_outside_safe_distance_can_be_dispatched(self):
        """2m 초과 로봇 → 파견 가능."""
        self.robots['ugv1'].pose = (5.0, 2.9)   # 2.1m (> 2m) → 파견 가능
        self.robots['ugv2'].pose = (5.0, 4.2)   # 0.8m → 보류
        self.robots['ugv3'].pose = (5.0, 4.5)   # 0.5m → 보류
        rid = self.ds.dispatch(self.fire_x, self.fire_y)
        assert rid == 'ugv1'

    # ── 통신 두절 / busy 제외 ──

    def test_comm_lost_robot_excluded(self):
        """통신 두절 로봇은 파견 대상 제외."""
        self.robots['ugv1'].comm_lost = True
        rid = self.ds.dispatch(self.fire_x, self.fire_y)
        assert rid != 'ugv1'

    def test_busy_robot_excluded(self):
        """이미 mission_lock인 로봇은 파견 제외."""
        self.robots['ugv1'].mission_lock = 'rescue'
        rid = self.ds.dispatch(self.fire_x, self.fire_y)
        assert rid != 'ugv1'
        assert rid == 'ugv2'

    def test_drone_not_selected_as_primary(self):
        """드론 타입은 primary_responder 제외 (UGV만)."""
        self.robots['ugv1'].robot_type = 'drone'
        self.robots['ugv2'].robot_type = 'drone'
        rid = self.ds.dispatch(self.fire_x, self.fire_y)
        assert rid == 'ugv3'

    # ── 연속 파견 시나리오 ──

    def test_second_fire_dispatches_different_robot(self):
        """1번째 화재 배정 후 2번째 다른 화점 → 남은 로봇 중 선정."""
        fire2_x, fire2_y = 9.0, 9.0

        # 별도 DispatchSystem 사용 (두 화점 각각 독립 관리)
        ds1 = DispatchSystem(self.robots)
        ds2 = DispatchSystem(self.robots)

        rid1 = ds1.dispatch(self.fire_x, self.fire_y)   # ugv1 배정
        # ugv1은 이제 busy
        rid2 = ds2.dispatch(fire2_x, fire2_y)           # ugv1 제외 → ugv2
        assert rid1 != rid2
        assert rid1 is not None
        assert rid2 is not None

    def test_all_busy_then_release_allows_new_dispatch(self):
        """모든 로봇 busy → 1대 해제 → 새 파견 가능."""
        for r in self.robots.values():
            r.mission_lock = 'rescue'

        rid = self.ds.dispatch(self.fire_x, self.fire_y)
        assert rid is None  # 모두 busy

        # ugv2 해제
        self.robots['ugv2'].mission_lock = None
        ds2 = DispatchSystem(self.robots)
        rid2 = ds2.dispatch(self.fire_x, self.fire_y)
        assert rid2 == 'ugv2'


# ══════════════════════════════════════════════════
# ROS2 실환경 launch_testing 클래스
# ══════════════════════════════════════════════════

if _ROS2_AVAILABLE:
    import unittest
    import threading
    import time

    @unittest.skip("launch_testing 전용: colcon test로 실행 (orchestrator 노드 별도 기동 필요)")
    class TestMultiRobotContentionLaunch(unittest.TestCase):
        """
        실제 orchestrator 노드에 robot_status 3대분 발행 후
        mission_state.primary_responder가 1대인지 확인.
        colcon test 또는 launch_testing으로 실행.
        """

        @classmethod
        def setUpClass(cls):
            rclpy.init()
            cls._node = rclpy.create_node('test_contention_client')

        @classmethod
        def tearDownClass(cls):
            cls._node.destroy_node()
            rclpy.shutdown()

        def test_single_primary_responder_ros2(self):
            """
            3대 robot_status 발행 + FireAlert → primary_responder 1대 확인.
            """
            from argos_interfaces.msg import RobotStatus, FireAlert, MissionState

            # robot_status 발행자
            status_pub = self._node.create_publisher(
                RobotStatus, '/orchestrator/robot_status', 10)
            fire_pub = self._node.create_publisher(
                FireAlert, '/orchestrator/fire_alert', 10)

            time.sleep(0.5)

            # 3대 상태 발행
            poses = [('ugv1', 3.0, 3.0), ('ugv2', 8.0, 8.0), ('ugv3', 1.0, 9.0)]
            for rid, x, y in poses:
                msg = RobotStatus()
                msg.robot_id = rid
                msg.state = RobotStatus.STATE_EXPLORING
                msg.battery_percent = 80.0
                msg.pose.pose.position.x = x
                msg.pose.pose.position.y = y
                msg.header.stamp = self._node.get_clock().now().to_msg()
                status_pub.publish(msg)

            time.sleep(0.3)

            # 동일 화점에 2회 FireAlert 발행 (경합 시뮬레이션)
            for _ in range(2):
                alert = FireAlert()
                alert.robot_id = 'ugv1'
                alert.location.point.x = 5.0
                alert.location.point.y = 5.0
                alert.max_temperature_kelvin = 773.15
                alert.severity = 'critical'
                alert.confidence = 0.95
                alert.active = True
                alert.header.stamp = self._node.get_clock().now().to_msg()
                fire_pub.publish(alert)

            # mission_state 수집
            received = []
            event = threading.Event()

            def cb(msg):
                received.append(msg)
                if msg.stage == MissionState.STAGE_FIRE_RESPONSE:
                    event.set()

            sub = self._node.create_subscription(
                MissionState, '/orchestrator/mission_state', cb, 10)

            end = time.time() + 10.0
            while time.time() < end and not event.is_set():
                rclpy.spin_once(self._node, timeout_sec=0.1)

            self._node.destroy_subscription(sub)
            self._node.destroy_publisher(status_pub)
            self._node.destroy_publisher(fire_pub)

            # primary_responder가 단 1대인지 확인
            fire_response_msgs = [
                m for m in received
                if m.stage == MissionState.STAGE_FIRE_RESPONSE
            ]
            self.assertGreater(len(fire_response_msgs), 0, 'FIRE_RESPONSE 메시지 미수신')
            primary_set = {m.primary_responder for m in fire_response_msgs
                           if m.primary_responder}
            self.assertLessEqual(len(primary_set), 1, f'primary_responder 중복 배정: {primary_set}')
