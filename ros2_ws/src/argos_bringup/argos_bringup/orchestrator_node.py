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
        /orchestrator/autonomy_mode (std_msgs/String) — "CENTRALIZED" | "LOCAL_AUTONOMY"
        /orchestrator/hose_conflict (std_msgs/String) — JSON: {"rid_a", "rid_b", "resolution"}
  서비스: /orchestrator/emergency_stop (Trigger) — 전원 즉시 정지
          /orchestrator/set_stage (SetStage) — 임무 단계 전환

LifecycleNode 상태:
  unconfigured → configure() → inactive
  inactive     → activate()  → active (구독/발행/타이머/서비스 활성)
  active       → deactivate() → inactive (통신 중단)
  inactive     → cleanup()   → unconfigured
"""

import math
import threading
from collections import deque
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, State
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
)
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup

from std_srvs.srv import Trigger
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Point, Twist, TwistStamped, PoseStamped
from argos_interfaces.msg import (
    RobotStatus, FireAlert, MissionState,
    GasReading, VictimDetection, StructuralAlert, AudioEvent,
)
from argos_bringup.validation_utils import (
    validate_robot_id, validate_severity, validate_danger_level,
    validate_timestamp, clamp_sensor,
)
from argos_bringup.cbba_allocator import (
    CBBAAllocator,
    RobotRecord as CbbaRobotRecord,
    Task as CbbaTask,
)
from argos_bringup.orchestrator_types import (
    RobotRecord,
    FIRE_TACTICS,
    CONSENSUS_RADIUS,
    COLLISION_SAFE_DISTANCE,
    StopRobotCommand,
    WaypointCommand,
    AutonomyModeCommand,
    HoseConflictEvent,
    StageTransition,
    DispatchRescueCommand,
    FireResponseRequest,
)
from argos_bringup.sensor_fusion import SensorFusion
from argos_bringup.robot_dispatcher import RobotDispatcher


# RobotRecord, FIRE_TACTICS, constants → orchestrator_types.py (G-1 SRP)


class OrchestratorNode(LifecycleNode):
    """오케스트레이터 노드 — LifecycleNode 패턴.

    G-1 SRP 리팩토링 Phase 1 완료:
      - RobotRecord, FIRE_TACTICS, 상수 → orchestrator_types.py 분리
      - 명령 객체 패턴 도입 (StopRobotCommand, WaypointCommand 등)
    Phase 2~3 예정:
      - SensorFusion: 8중 센싱 콜백, 위험도 집계 분리
      - RobotDispatcher: CBBA, 호스, heartbeat, 파견 분리
    """

    # Heartbeat 타임아웃 (초) — 근거: 로봇 발행 주기 2초 × 5회 미수신 = 10초
    # 소방 현장: 콘크리트 벽 전파 차단 2~3초 + DDS 재연결 2~3초 = 5~6초 가정
    HEARTBEAT_TIMEOUT = 10.0
    # 상태 발행 주기 (초) — 근거: MPPI 컨트롤러 20Hz의 1/10 (충분한 피드백)
    PUBLISH_RATE = 2.0
    # 배터리 임계값 (%) — 근거: 소방 현장 운용 표준
    # 30%: 잔여 ~36분 (120분 배터리 기준) → 기지 복귀 충분
    # 15%: 잔여 ~18분 → 즉시 귀환하지 않으면 현장 방치 위험
    BATTERY_WARNING = 30.0
    BATTERY_CRITICAL = 15.0

    def __init__(self):
        super().__init__('orchestrator')

        # ── 파라미터 선언만 (activate 전에는 토픽·타이머·서비스 생성 금지) ──
        self.declare_parameter('expected_robots', ['argos1', 'argos2'])
        self.declare_parameter('return_timeout_sec', 30.0)
        self.declare_parameter('fire_alert_expiry_sec', 300.0)  # 5분 후 자동 비활성

        # 런타임 상태 (on_configure에서 초기화)
        self.return_timeout = None
        self.fire_expiry = None
        self.stage = None
        self.robots = {}
        self._robots_lock = None
        self.start_time = None
        self.paused = False
        self.robot_stop_pubs = {}
        self.drone_wp_pubs = {}
        self.primary_responder = None
        self.fire_response_target = None
        self.return_start_time = None
        self.sensor_fusion = None  # G-1 SRP: on_configure에서 SensorFusion 생성
        self.dispatcher = None  # G-1 SRP Phase 3: on_configure에서 RobotDispatcher 생성

        # 구독/발행/서비스/타이머 핸들 (on_activate에서 생성, on_deactivate에서 해제)
        self.status_sub = None
        self.fire_sub = None
        self._sensor_subs = []  # 8중 센싱 구독 핸들 리스트
        self.mission_pub = None
        self.autonomy_mode_pub = None  # C-3: 로봇별 자율 모드 토픽
        self._hose_conflict_pub = None  # 호스 충돌 이벤트 발행자
        self.stop_srv = None
        self.resume_srv = None
        self._gas_watchdog_timer = None
        self.state_timer = None
        self.heartbeat_timer = None
        self._hose_check_timer = None  # 주기적 호스 충돌 검사 타이머

    # ─────────────────── Lifecycle Callbacks ───────────────────

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """파라미터 읽기 + 내부 상태 초기화.

        inactive 상태로 진입 전 호출됨. ROS 통신(토픽·서비스·타이머)은 만들지 않는다.
        """
        expected = self.get_parameter('expected_robots').value
        self.return_timeout = self.get_parameter('return_timeout_sec').value
        self.fire_expiry = self.get_parameter('fire_alert_expiry_sec').value

        # 상태 초기화
        self.stage = MissionState.STAGE_INIT
        self.robots = {}
        self._robots_lock = threading.Lock()
        self.start_time = self.get_clock().now()
        self.paused = False
        self.robot_stop_pubs = {}
        self.drone_wp_pubs = {}
        self.primary_responder = None
        self.fire_response_target = None
        self.return_start_time = None

        # 예상 로봇 사전 등록
        for rid in expected:
            self.robots[rid] = RobotRecord(rid)

        # G-1 SRP: SensorFusion 모듈 생성
        self.sensor_fusion = SensorFusion(
            logger=self.get_logger(),
            clock_fn=lambda: self.get_clock().now().nanoseconds / 1e9,
        )

        # G-1 SRP Phase 3: RobotDispatcher 모듈 생성
        self.dispatcher = RobotDispatcher(
            logger=self.get_logger(),
            clock_fn=lambda: self.get_clock().now().nanoseconds / 1e9,
            sensor_fusion=self.sensor_fusion,
        )

        self.get_logger().info(
            f'Orchestrator configured — expecting {len(self.robots)} robots: '
            f'{list(self.robots.keys())}')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """구독/발행/서비스/타이머 생성 → 임무 시작.

        active 상태로 진입 전 호출됨.
        """
        cb_group = ReentrantCallbackGroup()

        # QoS 프로파일 정의
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10,
        )
        # H9: TODO — robot_status 구독자가 TRANSIENT_LOCAL 사용 시 발행자도 동일 설정 필요
        #            (현재 VOLATILE 기본값 → late-joining subscriber 초기 상태 수신 못함)
        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10,
            deadline=Duration(seconds=5),
        )
        gas_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10,
            deadline=Duration(seconds=2),
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10,
            deadline=Duration(seconds=5),
        )
        from rclpy.qos import HistoryPolicy
        audio_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # --- Subscribers ---
        self.status_sub = self.create_subscription(
            RobotStatus, '/orchestrator/robot_status',
            self._on_robot_status, status_qos)

        self.fire_sub = self.create_subscription(
            FireAlert, '/orchestrator/fire_alert',
            self._on_fire_alert, reliable_qos)

        # --- 8중 센싱 구독 (QoS 분화, 통신 전문가 권고) ---
        self._sensor_subs = []
        expected = list(self.robots.keys())
        for rid in expected:
            self._sensor_subs.append(self.create_subscription(
                GasReading, f'/{rid}/gas/reading',
                self._on_gas, gas_qos))
            self._sensor_subs.append(self.create_subscription(
                VictimDetection, f'/{rid}/victim/detections',
                self._on_victim, sensor_qos))
            self._sensor_subs.append(self.create_subscription(
                StructuralAlert, f'/{rid}/structural/alerts',
                self._on_structural, sensor_qos))
            self._sensor_subs.append(self.create_subscription(
                AudioEvent, f'/{rid}/audio/events',
                self._on_audio, audio_qos))

        # --- 호스 상태 구독 (셰르파 로봇 전용) ---
        # sherpa_ 접두사를 가진 로봇에만 구독 생성
        # 호스 없는 로봇(UGV, 드론)은 hose_remaining_m = -1로 유지됨
        for rid in expected:
            if rid.startswith('sherpa'):
                self._sensor_subs.append(self.create_subscription(
                    Float32MultiArray,
                    f'/{rid}/hose/status',
                    lambda msg, r=rid: self._on_hose_status(r, msg),
                    sensor_qos,
                ))
                self.get_logger().info(f'호스 상태 구독 등록: /{rid}/hose/status')

        # --- Publishers ---
        self.mission_pub = self.create_publisher(
            MissionState, '/orchestrator/mission_state', reliable_qos)

        # C-3: 자율 모드 발행자 — TRANSIENT_LOCAL로 늦게 접속한 구독자도 최신 값 수신
        autonomy_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.autonomy_mode_pub = self.create_publisher(
            String, '/orchestrator/autonomy_mode', autonomy_qos)

        # 호스 충돌 이벤트 발행자 — 외부 모니터링 / 대시보드 연동용
        self._hose_conflict_pub = self.create_publisher(
            String, '/orchestrator/hose_conflict', 10)

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
            self.HEARTBEAT_TIMEOUT / 2, self._on_heartbeat)
        # H1: gas sensor watchdog — 5초 이상 수신 없으면 경고
        self._gas_watchdog_timer = self.create_timer(5.0, self._on_gas_watchdog)
        # 호스 충돌 주기 검사 — 2초마다 모든 셰르파 쌍 교차 검사
        self._hose_check_timer = self.create_timer(2.0, self._on_hose_check)

        self.get_logger().info(
            f'Orchestrator activated — {len(expected)} robots: {expected}')
        self.dispatcher.init_cbba()
        self._cbba_realloc_timer = self.create_timer(10.0, self._on_cbba_realloc)
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """모든 핸들 destroy → 임무 중단.

        inactive 상태로 진입 전 호출됨.
        """
        # 타이머 해제
        if self.state_timer is not None:
            self.state_timer.cancel()
            self.destroy_timer(self.state_timer)
            self.state_timer = None
        if self.heartbeat_timer is not None:
            self.heartbeat_timer.cancel()
            self.destroy_timer(self.heartbeat_timer)
            self.heartbeat_timer = None
        # H1: gas watchdog 타이머 해제
        if self._gas_watchdog_timer is not None:
            self._gas_watchdog_timer.cancel()
            self.destroy_timer(self._gas_watchdog_timer)
            self._gas_watchdog_timer = None
        # 호스 충돌 검사 타이머 해제
        if self._hose_check_timer is not None:
            self._hose_check_timer.cancel()
            self.destroy_timer(self._hose_check_timer)
            self._hose_check_timer = None
        # CBBA 주기 재할당 타이머 해제
        if hasattr(self, '_cbba_realloc_timer') and self._cbba_realloc_timer is not None:
            self._cbba_realloc_timer.cancel()
            self.destroy_timer(self._cbba_realloc_timer)
            self._cbba_realloc_timer = None

        # 서비스 해제
        if self.stop_srv is not None:
            self.destroy_service(self.stop_srv)
            self.stop_srv = None
        if self.resume_srv is not None:
            self.destroy_service(self.resume_srv)
            self.resume_srv = None

        # 발행자 해제
        if self.mission_pub is not None:
            self.destroy_publisher(self.mission_pub)
            self.mission_pub = None
        if self.autonomy_mode_pub is not None:
            self.destroy_publisher(self.autonomy_mode_pub)
            self.autonomy_mode_pub = None
        if self._hose_conflict_pub is not None:
            self.destroy_publisher(self._hose_conflict_pub)
            self._hose_conflict_pub = None

        # emergency_stop용 cmd_vel 발행자 해제
        for rid, pub in list(self.robot_stop_pubs.items()):
            self.destroy_publisher(pub)
        self.robot_stop_pubs = {}

        # 드론 웨이포인트 발행자 해제
        for rid, pub in list(self.drone_wp_pubs.items()):
            self.destroy_publisher(pub)
        self.drone_wp_pubs = {}

        # 8중 센싱 구독 해제
        for sub in self._sensor_subs:
            self.destroy_subscription(sub)
        self._sensor_subs = []

        # 기본 구독 해제
        if self.fire_sub is not None:
            self.destroy_subscription(self.fire_sub)
            self.fire_sub = None
        if self.status_sub is not None:
            self.destroy_subscription(self.status_sub)
            self.status_sub = None

        self.get_logger().info('Orchestrator deactivated')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """내부 상태 리셋 → unconfigured로 복귀."""
        self.stage = None
        self.robots = {}
        self._robots_lock = None
        self.paused = False
        self.primary_responder = None
        self.fire_response_target = None
        self.return_start_time = None
        self.sensor_fusion = None
        self.dispatcher = None
        self.get_logger().info('Orchestrator cleaned up')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """종료 시 로그만."""
        self.get_logger().info('Orchestrator shutting down')
        return TransitionCallbackReturn.SUCCESS

    # ─────────────────── Callbacks ───────────────────

    def _on_robot_status(self, msg: RobotStatus):
        """로봇 상태 수신 → dispatcher 위임 후 액션 실행 (G-1 SRP Phase 3)."""
        actions = self.dispatcher.robot_status_callback(
            msg, self.robots, self._robots_lock)
        self._execute_actions(actions)
        self._auto_stage_transition()

    def emergency_stop_callback(self, request, response):
        """긴급 정지 — 모든 로봇에 정지 명령 3회 연속 발행 (신뢰성 강화)."""
        self.stage = MissionState.STAGE_RETURNING
        self.return_start_time = self.get_clock().now()
        self.paused = True
        ESTOP_REPEAT = 3  # 네트워크 손실 대비 3회 연속 발행

        with self._robots_lock:
            robot_ids = list(self.robots.keys())

        success_count = 0
        for rid in robot_ids:
            if rid not in self.robot_stop_pubs:
                self.robot_stop_pubs[rid] = self.create_publisher(
                    TwistStamped, f'/{rid}/cmd_vel', 10)
            for _ in range(ESTOP_REPEAT):
                stop_cmd = TwistStamped()
                stop_cmd.header.stamp = self.get_clock().now().to_msg()
                self.robot_stop_pubs[rid].publish(stop_cmd)
            success_count += 1
            self.get_logger().error(f'E-STOP issued to {rid} (x{ESTOP_REPEAT})')

        self.get_logger().error(
            f'EMERGENCY STOP — {success_count}/{len(robot_ids)} robots stopped')
        response.success = success_count == len(robot_ids)
        response.message = f'E-Stop sent to {success_count}/{len(robot_ids)} robots'
        return response

    def resume_callback(self, request, response):
        """긴급 정지 해제 / 철수 승인.

        지휘관이 호출:
        - 일반 정지 해제 → EXPLORING 복귀
        - 철수 승인 (evacuation_recommended=True) → RETURNING 전환
        """
        if self.paused:
            self.paused = False
            if getattr(self.sensor_fusion, '_evacuation_recommended', False):
                # 지휘관이 철수를 승인함
                self.sensor_fusion._evacuation_recommended = False
                self.stage = MissionState.STAGE_RETURNING
                self.return_start_time = self.get_clock().now()
                self.get_logger().error(
                    'EVACUATION APPROVED by commander — RETURNING')
                response.message = 'Evacuation approved — all robots returning'
            else:
                self.stage = MissionState.STAGE_EXPLORING
                self.get_logger().info('Resumed — back to EXPLORING')
                response.message = 'Resumed'
            response.success = True
        else:
            response.success = False
            response.message = 'Not paused'
        return response

    # ─────────────────── 8중 센싱 Wrapper Callbacks (G-1 SRP) ───────────────────

    def _execute_actions(self, actions: list):
        """센서 퓨전/디스패처 모듈이 반환한 명령 객체를 실행."""
        from argos_interfaces.msg import MissionState
        for action in actions:
            if isinstance(action, StopRobotCommand):
                rid = action.robot_id
                if rid not in self.robot_stop_pubs:
                    self.robot_stop_pubs[rid] = self.create_publisher(
                        TwistStamped, f'/{rid}/cmd_vel', 10)
                stop_cmd = TwistStamped()
                stop_cmd.header.stamp = self.get_clock().now().to_msg()
                self.robot_stop_pubs[rid].publish(stop_cmd)
            elif isinstance(action, WaypointCommand):
                rid = action.robot_id
                topic = f'/{rid}/drone/waypoint'
                if rid not in self.drone_wp_pubs:
                    self.drone_wp_pubs[rid] = self.create_publisher(
                        PoseStamped, topic, 10)
                wp = PoseStamped()
                wp.header.stamp = self.get_clock().now().to_msg()
                wp.header.frame_id = action.frame_id
                wp.pose.position.x = action.x
                wp.pose.position.y = action.y
                wp.pose.position.z = action.z
                self.drone_wp_pubs[rid].publish(wp)
            elif isinstance(action, AutonomyModeCommand):
                self._publish_autonomy_mode(action.robot_id, action.mode)
            elif isinstance(action, HoseConflictEvent):
                if self._hose_conflict_pub is not None:
                    import json
                    event_msg = String()
                    event_msg.data = json.dumps({
                        'rid_a': action.rid_a,
                        'rid_b': action.rid_b,
                        'resolution': action.resolution,
                    }, ensure_ascii=False)
                    self._hose_conflict_pub.publish(event_msg)
            elif isinstance(action, DispatchRescueCommand):
                with self._robots_lock:
                    if action.robot_id in self.robots:
                        self.robots[action.robot_id].mission_lock = 'rescue'
                        from geometry_msgs.msg import Point
                        self.robots[action.robot_id].assigned_target = Point(
                            x=action.target_x, y=action.target_y, z=action.target_z)
                # primary_responder 갱신 (fire_suppress_ground 배정 시)
                self.primary_responder = action.robot_id
            elif isinstance(action, StageTransition):
                self.stage = action.new_stage
                if action.new_stage == MissionState.STAGE_FIRE_RESPONSE:
                    self.get_logger().warn('Stage → FIRE_RESPONSE')
                elif action.new_stage == MissionState.STAGE_RETURNING:
                    self.return_start_time = self.get_clock().now()
                elif action.new_stage == MissionState.STAGE_PAUSED:
                    self.paused = True
            elif isinstance(action, FireResponseRequest):
                from geometry_msgs.msg import Point
                fire_pt = Point(x=action.fire_x, y=action.fire_y, z=0.0)
                self.fire_response_target = fire_pt
                self._execute_fire_response_cbba(fire_pt, action.severity)
        # 공유 상태 동기화
        self.fire_response_target = self.sensor_fusion.fire_response_target
        self.primary_responder = self.sensor_fusion.primary_responder

    def _on_fire_alert(self, msg):
        """화재 알림 → SensorFusion 위임 후 액션 실행."""
        actions = self.sensor_fusion.fire_alert_callback(msg, self.stage)
        self._execute_actions(actions)
        self._auto_stage_transition()

    def _on_gas(self, msg):
        """가스 콜백 → SensorFusion 위임 후 액션 실행."""
        actions = self.sensor_fusion.gas_callback(msg, self.stage)
        self._execute_actions(actions)

    def _on_victim(self, msg):
        """피해자 감지 → SensorFusion 위임 후 액션 실행."""
        with self._robots_lock:
            snapshot = dict(self.robots)
        actions = self.sensor_fusion.victim_callback(msg, snapshot)
        self._execute_actions(actions)

    def _on_structural(self, msg):
        """구조물 알림 → SensorFusion 위임 후 액션 실행."""
        actions = self.sensor_fusion.structural_callback(msg)
        self._execute_actions(actions)

    def _on_audio(self, msg):
        """음향 이벤트 → SensorFusion 위임 후 액션 실행."""
        with self._robots_lock:
            snapshot = dict(self.robots)
        actions = self.sensor_fusion.audio_callback(msg, snapshot)
        self._execute_actions(actions)

    def _on_gas_watchdog(self):
        """가스 watchdog → SensorFusion 위임."""
        self.sensor_fusion.check_gas_watchdog()

    # ─────────────────── 호스 / Heartbeat / CBBA 래퍼 (G-1 SRP Phase 3) ───────────────────

    def _on_hose_status(self, robot_id: str, msg):
        """호스 상태 콜백 → dispatcher 위임 후 액션 실행."""
        actions = self.dispatcher.hose_status_callback(
            robot_id, msg, self.robots, self._robots_lock)
        self._execute_actions(actions)

    def _on_heartbeat(self):
        """heartbeat 타이머 → dispatcher 위임 후 액션 실행."""
        actions = self.dispatcher.check_heartbeats(
            self.robots, self._robots_lock, self.HEARTBEAT_TIMEOUT)
        self._execute_actions(actions)

    def _on_hose_check(self):
        """호스 충돌 주기 검사 → dispatcher 위임 후 액션 실행."""
        actions = self.dispatcher.periodic_hose_check(
            self.robots, self._robots_lock)
        self._execute_actions(actions)

    def _on_cbba_realloc(self):
        """CBBA 주기 재할당 → dispatcher 위임 후 액션 실행."""
        actions = self.dispatcher.cbba_periodic_realloc(
            self.robots, self._robots_lock, self.stage)
        self._execute_actions(actions)


    # ─────────────────── Core Logic ───────────────────

    def _publish_autonomy_mode(self, rid: str, mode: str):
        """C-3: 특정 로봇의 자율 모드 토픽 발행.

        Args:
            rid: 대상 로봇 ID
            mode: "CENTRALIZED" | "LOCAL_AUTONOMY"
        """
        if self.autonomy_mode_pub is None:
            return
        msg = String()
        msg.data = mode
        self.autonomy_mode_pub.publish(msg)

    def _execute_fire_response_cbba(self, fire_pt, severity: str):
        """CBBA 기반 화재 대응 래퍼 — dispatcher 위임 후 액션 실행."""
        actions = self.dispatcher.execute_fire_response_cbba(
            fire_pt, severity, self.robots, self._robots_lock, self.stage)
        self._execute_actions(actions)

    def _execute_fire_response(self, fire_pt, severity: str):
        """화재 대응 폴백 래퍼 — dispatcher 위임 후 액션 실행."""
        actions = self.dispatcher.execute_fire_response(
            fire_pt, severity, self.robots, self._robots_lock)
        self._execute_actions(actions)

    def _auto_stage_transition(self):
        """로봇 상태 기반 자동 단계 전환."""
        with self._robots_lock:
            snapshot = dict(self.robots)

        if self.stage == MissionState.STAGE_INIT:
            all_seen = all(
                r.last_seen > 0 for r in snapshot.values())
            if all_seen:
                self.stage = MissionState.STAGE_EXPLORING
                self.get_logger().info(
                    f'All {len(snapshot)} robots online — '
                    f'Stage → EXPLORING')

        elif self.stage == MissionState.STAGE_EXPLORING:
            active_robots = [
                r for r in snapshot.values()
                if not r.comm_lost and r.last_seen > 0
            ]
            all_complete = (
                len(active_robots) > 0
                and all(r.current_mission == 'complete' for r in active_robots)
            )
            if all_complete:
                active = [r for r in snapshot.values() if not r.comm_lost]
                if active:
                    self.stage = MissionState.STAGE_RETURNING
                    self.return_start_time = self.get_clock().now()
                    self.get_logger().info(
                        'All exploration complete — Stage → RETURNING')

        elif self.stage == MissionState.STAGE_RETURNING:
            # O2 fix: return_start_time이 None이면 지금 시각으로 초기화
            if self.return_start_time is None:
                self.return_start_time = self.get_clock().now()
            all_idle = all(
                r.state == RobotStatus.STATE_IDLE
                for r in snapshot.values()
                if not r.comm_lost
            )
            elapsed = (self.get_clock().now() - self.return_start_time).nanoseconds / 1e9
            if all_idle or elapsed > self.return_timeout:
                self.stage = MissionState.STAGE_COMPLETE
                self.get_logger().info('All robots returned — Stage → COMPLETE')

        elif self.stage == MissionState.STAGE_FIRE_RESPONSE:
            now_sec = self.get_clock().now().nanoseconds / 1e9
            active_fires = [
                f for f in self.sensor_fusion.fire_alerts
                if f.active and (now_sec - (f.header.stamp.sec + f.header.stamp.nanosec / 1e9)) < self.fire_expiry
            ]
            if not active_fires:
                self.stage = MissionState.STAGE_EXPLORING
                self.primary_responder = None
                self.fire_response_target = None
                self.sensor_fusion.fire_response_target = None
                self.sensor_fusion.primary_responder = None
                self.sensor_fusion._current_fire_severity = None
                self.get_logger().info(
                    'No active fires remaining — Stage → EXPLORING')

    def publish_mission_state(self):
        """전체 임무 상태 발행."""
        msg = MissionState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.stage = self.stage

        with self._robots_lock:
            snapshot = dict(self.robots)

        msg.total_robots = len(snapshot)
        msg.active_robots = sum(
            1 for r in snapshot.values()
            if not r.comm_lost and r.state != RobotStatus.STATE_IDLE
        )
        msg.comm_lost_robots = sum(
            1 for r in snapshot.values() if r.comm_lost)

        active_coverages = [
            r.coverage for r in snapshot.values() if not r.comm_lost]
        if active_coverages:
            msg.overall_coverage_percent = sum(active_coverages) / len(active_coverages)

        now_sec = self.get_clock().now().nanoseconds / 1e9
        active_fires = [
            f for f in self.sensor_fusion.fire_alerts
            if f.active and (now_sec - (f.header.stamp.sec + f.header.stamp.nanosec / 1e9)) < self.fire_expiry
        ]
        msg.fire_count = len(active_fires)
        msg.fire_locations = [f.location.point for f in active_fires]

        msg.robot_ids = list(snapshot.keys())
        msg.robot_states = [r.state for r in snapshot.values()]
        msg.robot_missions = [r.current_mission for r in snapshot.values()]

        msg.primary_responder = self.primary_responder or ''

        msg.gas_danger_level = self.sensor_fusion.gas_danger_level
        msg.victims_detected_count = len(self.sensor_fusion.victims_detected)
        msg.blocked_areas_count = len(self.sensor_fusion.blocked_areas)
        msg.audio_alerts_count = len(self.sensor_fusion.audio_alerts)

        elapsed = self.get_clock().now() - self.start_time
        msg.elapsed_sec = elapsed.nanoseconds / 1e9

        self.mission_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OrchestratorNode()
    node.trigger_configure()
    node.trigger_activate()

    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.trigger_deactivate()
        node.trigger_cleanup()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
