"""
ARGOS Orchestrator Node (MS-7)
===============================
이종 군집 소방 로봇 오케스트레이션의 중앙 지휘 노드.

감독 자율성(Supervised Autonomy) 아키텍처:
  - 오케스트레이터가 "무엇을"을 결정, 로봇이 "어떻게"를 자율 수행
  - DARPA SubT CERBERUS/CoSTAR 우승팀과 동일 패턴
  - 통신 두절 시 로봇은 마지막 임무를 독립 수행

소방 지휘체계 매핑:
  현장 지휘관(인간) → ARGOS 오케스트레이터 → 개별 로봇
    STAGE_INIT: 로봇 등록, 능력 확인
    STAGE_EXPLORING: 프론티어 탐색 진행
    STAGE_FIRE_RESPONSE: 화점 감지 → 재할당
    STAGE_RETURNING: 전원 귀환
    STAGE_COMPLETE: 임무 종료

토픽:
  구독: /orchestrator/robot_status (RobotStatus, per robot)
        /orchestrator/fire_alert (FireAlert)
  발행: /orchestrator/mission_state (MissionState)
  서비스: /orchestrator/emergency_stop (Trigger) — 전원 즉시 정지
          /orchestrator/set_stage (SetStage) — 임무 단계 전환
"""

import time
from collections import deque
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    LivelinessPolicy,
)
from rclpy.callback_groups import ReentrantCallbackGroup

from std_srvs.srv import Trigger
from geometry_msgs.msg import Point, Twist
from my_robot_interfaces.msg import RobotStatus, FireAlert, MissionState


class RobotRecord:
    """오케스트레이터가 관리하는 개별 로봇 레코드."""

    def __init__(self, robot_id: str):
        self.robot_id = robot_id
        self.robot_type = 'ugv'
        self.state = RobotStatus.STATE_IDLE
        self.last_seen = 0.0
        self.battery = 100.0
        self.current_mission = 'idle'
        self.mission_progress = 0.0
        self.frontiers_remaining = 0
        self.coverage = 0.0
        self.capabilities = []
        self.pose = None
        self.comm_lost = False


class OrchestratorNode(Node):

    # Heartbeat 타임아웃 (초)
    HEARTBEAT_TIMEOUT = 10.0
    # 상태 발행 주기 (초)
    PUBLISH_RATE = 2.0

    def __init__(self):
        super().__init__('orchestrator')

        # --- Parameters ---
        self.declare_parameter('expected_robots', ['argos1', 'argos2'])
        self.declare_parameter('use_sim_time', True)

        expected = self.get_parameter('expected_robots').value

        # --- State ---
        self.stage = MissionState.STAGE_INIT
        self.robots = {}  # {robot_id: RobotRecord}
        self.fire_alerts = deque(maxlen=100)  # 최신 100개만 유지
        self.start_time = self.get_clock().now()
        self.paused = False
        self.robot_stop_pubs = {}  # emergency_stop용 cmd_vel 발행자

        # 예상 로봇 사전 등록
        for rid in expected:
            self.robots[rid] = RobotRecord(rid)

        cb_group = ReentrantCallbackGroup()

        # --- QoS (임무 명령용: RELIABLE + TRANSIENT_LOCAL) ---
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10,
        )

        # --- Subscribers ---
        self.status_sub = self.create_subscription(
            RobotStatus, '/orchestrator/robot_status',
            self.robot_status_callback, 10)

        self.fire_sub = self.create_subscription(
            FireAlert, '/orchestrator/fire_alert',
            self.fire_alert_callback, reliable_qos)

        # --- Publishers ---
        self.mission_pub = self.create_publisher(
            MissionState, '/orchestrator/mission_state', reliable_qos)

        # --- Services ---
        self.stop_srv = self.create_service(
            Trigger, '/orchestrator/emergency_stop',
            self.emergency_stop_callback, callback_group=cb_group)

        self.resume_srv = self.create_service(
            Trigger, '/orchestrator/resume',
            self.resume_callback, callback_group=cb_group)

        # --- Timers ---
        self.state_timer = self.create_timer(
            self.PUBLISH_RATE, self.publish_mission_state)
        self.heartbeat_timer = self.create_timer(
            self.HEARTBEAT_TIMEOUT / 2, self.check_heartbeats)

        self.get_logger().info(
            f'Orchestrator initialized — expecting {len(self.robots)} robots: '
            f'{list(self.robots.keys())}')

    # ─────────────────── Callbacks ───────────────────

    def robot_status_callback(self, msg: RobotStatus):
        """로봇 상태 수신 → 레지스트리 갱신."""
        rid = msg.robot_id
        now = self.get_clock().now().nanoseconds / 1e9

        if rid not in self.robots:
            self.robots[rid] = RobotRecord(rid)
            self.get_logger().info(f'New robot registered: {rid}')

        r = self.robots[rid]
        r.robot_type = msg.robot_type
        r.state = msg.state
        r.last_seen = now
        r.battery = msg.battery_percent
        r.current_mission = msg.current_mission
        r.mission_progress = msg.mission_progress
        r.frontiers_remaining = msg.frontiers_remaining
        r.coverage = msg.coverage_percent
        r.capabilities = list(msg.capabilities)
        r.pose = msg.pose

        if r.comm_lost:
            r.comm_lost = False
            self.get_logger().info(f'Robot {rid} reconnected')

        # 자동 단계 전환
        self._auto_stage_transition()

    def fire_alert_callback(self, msg: FireAlert):
        """화점 감지 알림 처리."""
        self.fire_alerts.append(msg)
        temp_c = msg.max_temperature_kelvin - 273.15
        self.get_logger().warn(
            f'FIRE ALERT from {msg.robot_id}: {msg.severity} '
            f'({temp_c:.0f}C) at ({msg.location.point.x:.1f}, '
            f'{msg.location.point.y:.1f})')

        # 화점 감지 시 FIRE_RESPONSE 단계로 전환
        if self.stage == MissionState.STAGE_EXPLORING:
            self.stage = MissionState.STAGE_FIRE_RESPONSE
            self.get_logger().warn('Stage → FIRE_RESPONSE')

    def emergency_stop_callback(self, request, response):
        """긴급 정지 — 모든 로봇에 영속도 명령 + 귀환 전환."""
        self.stage = MissionState.STAGE_RETURNING
        self.paused = True
        # 모든 로봇에 정지 명령 (cmd_vel = 0)
        for rid in self.robots:
            if rid not in self.robot_stop_pubs:
                self.robot_stop_pubs[rid] = self.create_publisher(
                    Twist, f'/{rid}/cmd_vel', 10)
            self.robot_stop_pubs[rid].publish(Twist())
        self.get_logger().warn('EMERGENCY STOP — all robots stopped')
        response.success = True
        response.message = 'Emergency stop issued. All robots stopped.'
        return response

    def resume_callback(self, request, response):
        """긴급 정지 해제."""
        if self.paused:
            self.paused = False
            self.stage = MissionState.STAGE_EXPLORING
            self.get_logger().info('Resumed — back to EXPLORING')
            response.success = True
            response.message = 'Resumed'
        else:
            response.success = False
            response.message = 'Not paused'
        return response

    # ─────────────────── Core Logic ───────────────────

    def check_heartbeats(self):
        """주기적 heartbeat 점검: 타임아웃 시 통신 두절 판정."""
        now = self.get_clock().now().nanoseconds / 1e9
        lost_count = 0

        for rid, r in self.robots.items():
            if r.last_seen == 0.0:
                continue  # 아직 첫 상태 미수신

            elapsed = now - r.last_seen
            if elapsed > self.HEARTBEAT_TIMEOUT and not r.comm_lost:
                r.comm_lost = True
                r.state = RobotStatus.STATE_COMM_LOST
                self.get_logger().warn(
                    f'COMM LOST: {rid} (no status for {elapsed:.0f}s)')

            if r.comm_lost:
                lost_count += 1

        # 절반 이상 두절 시 경고
        active_count = sum(1 for r in self.robots.values() if r.last_seen > 0)
        if active_count > 0 and lost_count > active_count / 2:
            self.get_logger().error(
                f'CRITICAL: {lost_count}/{active_count} robots comm lost')

    def _auto_stage_transition(self):
        """로봇 상태 기반 자동 단계 전환."""
        if self.stage == MissionState.STAGE_INIT:
            # 모든 예상 로봇이 1회 이상 상태를 보고하면 탐색 시작
            all_seen = all(
                r.last_seen > 0 for r in self.robots.values())
            if all_seen:
                self.stage = MissionState.STAGE_EXPLORING
                self.get_logger().info(
                    f'All {len(self.robots)} robots online — '
                    f'Stage → EXPLORING')

        elif self.stage == MissionState.STAGE_EXPLORING:
            # 모든 로봇이 탐색 완료 시 → COMPLETE
            all_complete = all(
                r.current_mission == 'complete' or r.state == RobotStatus.STATE_IDLE
                for r in self.robots.values()
                if not r.comm_lost
            )
            if all_complete and any(r.last_seen > 0 for r in self.robots.values()):
                active = [r for r in self.robots.values() if not r.comm_lost]
                if active:
                    self.stage = MissionState.STAGE_COMPLETE
                    self.get_logger().info('All exploration complete — Stage → COMPLETE')

    def publish_mission_state(self):
        """전체 임무 상태 발행."""
        msg = MissionState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.stage = self.stage

        # 로봇 집계
        msg.total_robots = len(self.robots)
        msg.active_robots = sum(
            1 for r in self.robots.values()
            if not r.comm_lost and r.state != RobotStatus.STATE_IDLE
        )
        msg.comm_lost_robots = sum(
            1 for r in self.robots.values() if r.comm_lost)

        # 커버리지 평균
        active_coverages = [
            r.coverage for r in self.robots.values() if not r.comm_lost]
        if active_coverages:
            msg.overall_coverage_percent = sum(active_coverages) / len(active_coverages)

        # 화점
        active_fires = [f for f in self.fire_alerts if f.active]
        msg.fire_count = len(active_fires)
        msg.fire_locations = [f.location.point for f in active_fires]

        # 로봇별 요약
        msg.robot_ids = list(self.robots.keys())
        msg.robot_states = [r.state for r in self.robots.values()]
        msg.robot_missions = [r.current_mission for r in self.robots.values()]

        # 경과 시간
        elapsed = self.get_clock().now() - self.start_time
        msg.elapsed_sec = elapsed.nanoseconds / 1e9

        self.mission_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OrchestratorNode()

    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
