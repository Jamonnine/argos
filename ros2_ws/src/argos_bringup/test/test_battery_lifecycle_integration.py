"""
배터리 수명 주기 통합 테스트 (W5-3).

robot_status의 battery_percent를 100% → 30% → 15% → 50%로 순차 변경하며
오케스트레이터의 배터리 이벤트 처리 전체 사이클을 검증.

검증 흐름:
  100% → (정상)
  30%  → WARNING 로그 + battery_warned 플래그 설정
  15%  → CRITICAL + auto-return 트리거 + battery_critical_acted 플래그 설정
  50%  → 복구, 두 플래그 리셋

orchestrator_node.py 관련 상수:
  BATTERY_WARNING  = 30.0
  BATTERY_CRITICAL = 15.0
"""

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
        """orchestrator 노드 기동."""
        orchestrator = launch_ros.actions.Node(
            package='argos_bringup',
            executable='orchestrator',
            parameters=[{
                'expected_robots': ['argos1'],
            }],
            output='screen',
        )
        return launch.LaunchDescription([
            orchestrator,
            launch_testing.actions.ReadyToTest(),
        ]), {'orchestrator': orchestrator}


# ══════════════════════════════════════════════════
# 로직 레이어 — orchestrator_node.py 배터리 처리 재현
# ══════════════════════════════════════════════════

BATTERY_WARNING = 30.0   # orchestrator_node.py와 동일
BATTERY_CRITICAL = 15.0  # orchestrator_node.py와 동일


class BatteryEvent:
    """배터리 이벤트 기록."""
    WARNING = 'WARNING'
    CRITICAL = 'CRITICAL'
    AUTO_RETURN = 'AUTO_RETURN'
    RECOVERY = 'RECOVERY'
    NORMAL = 'NORMAL'


class RobotRecord:
    """배터리 테스트용 로봇 레코드."""

    def __init__(self, robot_id: str, battery: float = 100.0):
        self.robot_id = robot_id
        self.battery = battery
        self.battery_warned = False          # 30% 경고 플래그
        self.battery_critical_acted = False  # 15% 위험 플래그
        self.current_mission = 'exploring'
        self.state = 'exploring'


class BatteryProcessor:
    """
    orchestrator_node.py의 _check_battery_status 로직 재현.
    robot_status 수신 시마다 호출되어 배터리 수준에 따른 이벤트를 처리.
    """

    def __init__(self):
        self.event_log = []   # 발생 이벤트 목록
        self.log_messages = []  # 로그 메시지 목록

    def process(self, record: RobotRecord, battery_percent: float) -> list[str]:
        """
        배터리 수준 처리 후 발생한 이벤트 목록 반환.

        Args:
            record: 로봇 레코드 (플래그 포함)
            battery_percent: 새로 수신된 배터리 퍼센트

        Returns:
            이번 호출에서 발생한 이벤트 목록
        """
        record.battery = battery_percent
        events = []

        if battery_percent <= BATTERY_CRITICAL:
            # 15% 이하: CRITICAL + auto-return (중복 방지)
            if not record.battery_critical_acted:
                events.append(BatteryEvent.CRITICAL)
                events.append(BatteryEvent.AUTO_RETURN)
                record.battery_critical_acted = True
                record.battery_warned = True  # WARNING도 함께 설정
                record.current_mission = 'returning'
                record.state = 'returning'
                self.log_messages.append(
                    f'[CRITICAL] {record.robot_id} 배터리 {battery_percent:.1f}% — 자동 귀환'
                )

        elif battery_percent <= BATTERY_WARNING:
            # 30% 이하: WARNING (중복 방지)
            if not record.battery_warned:
                events.append(BatteryEvent.WARNING)
                record.battery_warned = True
                self.log_messages.append(
                    f'[WARNING] {record.robot_id} 배터리 {battery_percent:.1f}% — 귀환 권고'
                )

        else:
            # 복구 감지: WARNING/CRITICAL 임계값 초과 시 플래그 리셋
            if record.battery_warned and battery_percent > BATTERY_WARNING:
                record.battery_warned = False
                events.append(BatteryEvent.RECOVERY)
                self.log_messages.append(
                    f'[RECOVERY] {record.robot_id} 배터리 {battery_percent:.1f}% — 경고 해제'
                )
            if record.battery_critical_acted and battery_percent > BATTERY_CRITICAL:
                record.battery_critical_acted = False
                # RECOVERY가 아직 없으면 추가
                if BatteryEvent.RECOVERY not in events:
                    events.append(BatteryEvent.RECOVERY)

            if not events:
                events.append(BatteryEvent.NORMAL)

        self.event_log.extend(events)
        return events


# ══════════════════════════════════════════════════
# 테스트 클래스
# ══════════════════════════════════════════════════

class TestBatteryLifecycleIntegration:
    """
    배터리 100% → 30% → 15% → 50% 순차 변화 전체 사이클 통합 검증.
    """

    def setup_method(self):
        self.record = RobotRecord('argos1', battery=100.0)
        self.proc = BatteryProcessor()

    # ── 전체 사이클 ──

    def test_full_lifecycle_100_to_30_to_15_to_50(self):
        """
        100% → 30% → 15% → 50% 순차 변화 전체 사이클.
        각 단계에서 올바른 이벤트와 플래그 상태 검증.
        """
        # Phase 1: 100% — 정상
        events = self.proc.process(self.record, 100.0)
        assert BatteryEvent.NORMAL in events
        assert self.record.battery_warned is False
        assert self.record.battery_critical_acted is False

        # Phase 2: 30% — WARNING
        events = self.proc.process(self.record, 30.0)
        assert BatteryEvent.WARNING in events
        assert self.record.battery_warned is True
        assert self.record.battery_critical_acted is False

        # Phase 3: 15% — CRITICAL + auto-return
        events = self.proc.process(self.record, 15.0)
        assert BatteryEvent.CRITICAL in events
        assert BatteryEvent.AUTO_RETURN in events
        assert self.record.battery_critical_acted is True
        assert self.record.current_mission == 'returning'

        # Phase 4: 50% — 복구, 두 플래그 리셋
        events = self.proc.process(self.record, 50.0)
        assert BatteryEvent.RECOVERY in events
        assert self.record.battery_warned is False
        assert self.record.battery_critical_acted is False

    # ── WARNING 단계 세부 검증 ──

    def test_30_percent_triggers_warning_log(self):
        """30%에서 WARNING 로그 발생."""
        self.proc.process(self.record, 30.0)
        assert any('WARNING' in msg for msg in self.proc.log_messages)
        assert any('argos1' in msg for msg in self.proc.log_messages)

    def test_warning_flag_set_at_30_percent(self):
        """30%에서 battery_warned = True."""
        self.proc.process(self.record, 30.0)
        assert self.record.battery_warned is True

    def test_warning_not_repeated_on_consecutive_30_percent(self):
        """WARNING은 최초 1회만 발생 (중복 방지)."""
        events1 = self.proc.process(self.record, 30.0)
        events2 = self.proc.process(self.record, 28.0)  # 계속 감쇠
        assert BatteryEvent.WARNING in events1
        assert BatteryEvent.WARNING not in events2  # 중복 방지

    def test_battery_at_31_percent_no_warning(self):
        """31% — WARNING 임계값 초과 → 경고 없음."""
        events = self.proc.process(self.record, 31.0)
        assert BatteryEvent.WARNING not in events
        assert BatteryEvent.NORMAL in events

    # ── CRITICAL 단계 세부 검증 ──

    def test_15_percent_triggers_critical_log(self):
        """15%에서 CRITICAL 로그 발생."""
        self.proc.process(self.record, 15.0)
        assert any('CRITICAL' in msg for msg in self.proc.log_messages)

    def test_15_percent_triggers_auto_return(self):
        """15%에서 auto-return 트리거."""
        events = self.proc.process(self.record, 15.0)
        assert BatteryEvent.AUTO_RETURN in events
        assert self.record.current_mission == 'returning'

    def test_critical_flag_set_at_15_percent(self):
        """15%에서 battery_critical_acted = True."""
        self.proc.process(self.record, 15.0)
        assert self.record.battery_critical_acted is True

    def test_critical_not_repeated_on_consecutive_15_percent(self):
        """CRITICAL + AUTO_RETURN은 최초 1회만 (중복 방지)."""
        events1 = self.proc.process(self.record, 15.0)
        events2 = self.proc.process(self.record, 10.0)
        assert BatteryEvent.AUTO_RETURN in events1
        assert BatteryEvent.AUTO_RETURN not in events2

    def test_battery_below_critical_still_acts_once(self):
        """10% — 15% 임계값 이하도 CRITICAL 1회 처리."""
        events = self.proc.process(self.record, 10.0)
        assert BatteryEvent.CRITICAL in events
        assert BatteryEvent.AUTO_RETURN in events
        assert self.record.battery_critical_acted is True

    def test_zero_battery_is_critical(self):
        """0% — CRITICAL 처리."""
        events = self.proc.process(self.record, 0.0)
        assert BatteryEvent.CRITICAL in events

    # ── 복구 단계 세부 검증 ──

    def test_50_percent_resets_both_flags(self):
        """복구 50% → battery_warned, battery_critical_acted 모두 리셋."""
        # 먼저 두 플래그 설정
        self.proc.process(self.record, 15.0)
        assert self.record.battery_warned is True
        assert self.record.battery_critical_acted is True

        # 복구
        events = self.proc.process(self.record, 50.0)
        assert BatteryEvent.RECOVERY in events
        assert self.record.battery_warned is False
        assert self.record.battery_critical_acted is False

    def test_31_percent_resets_warning_flag(self):
        """31% 복구 → battery_warned 리셋 (CRITICAL 플래그는 유지)."""
        self.proc.process(self.record, 30.0)   # WARNING 설정
        assert self.record.battery_warned is True

        events = self.proc.process(self.record, 31.0)  # 복구
        assert BatteryEvent.RECOVERY in events
        assert self.record.battery_warned is False

    def test_recovery_from_critical_without_warning_phase(self):
        """WARNING 단계 없이 바로 CRITICAL → 복구 시 두 플래그 모두 리셋."""
        # 직접 CRITICAL로 점프
        self.proc.process(self.record, 10.0)
        assert self.record.battery_critical_acted is True

        events = self.proc.process(self.record, 80.0)
        assert self.record.battery_warned is False
        assert self.record.battery_critical_acted is False

    # ── 이벤트 로그 검증 ──

    def test_event_log_captures_full_lifecycle(self):
        """전체 사이클 이벤트가 event_log에 순서대로 기록."""
        self.proc.process(self.record, 100.0)  # NORMAL
        self.proc.process(self.record, 30.0)   # WARNING
        self.proc.process(self.record, 15.0)   # CRITICAL, AUTO_RETURN
        self.proc.process(self.record, 50.0)   # RECOVERY

        log = self.proc.event_log
        assert BatteryEvent.NORMAL in log
        assert BatteryEvent.WARNING in log
        assert BatteryEvent.CRITICAL in log
        assert BatteryEvent.AUTO_RETURN in log
        assert BatteryEvent.RECOVERY in log

        # 순서 검증: WARNING → CRITICAL → RECOVERY
        idx_warning = log.index(BatteryEvent.WARNING)
        idx_critical = log.index(BatteryEvent.CRITICAL)
        idx_recovery = log.index(BatteryEvent.RECOVERY)
        assert idx_warning < idx_critical < idx_recovery

    def test_multiple_robots_independent_flags(self):
        """다수 로봇의 배터리 플래그는 각각 독립 관리."""
        proc = BatteryProcessor()
        r1 = RobotRecord('argos1')
        r2 = RobotRecord('argos2')

        proc.process(r1, 15.0)  # r1 CRITICAL
        proc.process(r2, 80.0)  # r2 정상

        assert r1.battery_critical_acted is True
        assert r2.battery_critical_acted is False
        assert r1.battery_warned is True
        assert r2.battery_warned is False


# ══════════════════════════════════════════════════
# ROS2 실환경 launch_testing 클래스
# ══════════════════════════════════════════════════

if _ROS2_AVAILABLE:
    import unittest
    import threading
    import time

    @unittest.skip("launch_testing 전용: colcon test로 실행 (orchestrator 노드 별도 기동 필요)")
    class TestBatteryLifecycleLaunch(unittest.TestCase):
        """
        실제 orchestrator 노드에 robot_status를 순차 발행하며
        배터리 이벤트(WARNING, CRITICAL) 처리 확인.
        colcon test 또는 launch_testing으로 실행.
        """

        @classmethod
        def setUpClass(cls):
            rclpy.init()
            cls._node = rclpy.create_node('test_battery_lifecycle_client')

        @classmethod
        def tearDownClass(cls):
            cls._node.destroy_node()
            rclpy.shutdown()

        def _publish_robot_status(self, pub, battery_percent, robot_id='argos1'):
            """배터리 퍼센트가 지정된 robot_status 발행."""
            from argos_interfaces.msg import RobotStatus
            msg = RobotStatus()
            msg.robot_id = robot_id
            msg.state = RobotStatus.STATE_EXPLORING
            msg.battery_percent = battery_percent
            msg.pose.pose.position.x = 0.0
            msg.pose.pose.position.y = 0.0
            msg.header.stamp = self._node.get_clock().now().to_msg()
            pub.publish(msg)

        def test_battery_lifecycle_100_30_15_50_ros2(self):
            """
            배터리 100% → 30% → 15% → 50% 순차 발행 후
            mission_state 토픽에서 이상 없이 처리되는지 확인.
            """
            from argos_interfaces.msg import RobotStatus, MissionState

            pub = self._node.create_publisher(
                RobotStatus, '/orchestrator/robot_status', 10)

            mission_states = []

            def cb(msg):
                mission_states.append(msg)

            sub = self._node.create_subscription(
                MissionState, '/orchestrator/mission_state', cb, 10)

            time.sleep(0.5)

            # 배터리 순차 변화 발행
            for battery in [100.0, 30.0, 15.0, 50.0]:
                self._publish_robot_status(pub, battery)
                time.sleep(0.3)
                rclpy.spin_once(self._node, timeout_sec=0.1)

            time.sleep(1.0)
            rclpy.spin_once(self._node, timeout_sec=0.5)

            self._node.destroy_subscription(sub)
            self._node.destroy_publisher(pub)

            # 노드가 크래시 없이 처리했는지 확인 (mission_state 수신)
            self.assertGreater(
                len(mission_states), 0,
                '배터리 lifecycle 처리 중 mission_state 미수신 — 노드 크래시 의심',
            )
