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
            self.robot_status_callback, status_qos)

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
                    lambda msg, r=rid: self._hose_status_callback(r, msg),
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
            self.HEARTBEAT_TIMEOUT / 2, self.check_heartbeats)
        # H1: gas sensor watchdog — 5초 이상 수신 없으면 경고
        self._gas_watchdog_timer = self.create_timer(5.0, self._on_gas_watchdog)
        # 호스 충돌 주기 검사 — 2초마다 모든 셰르파 쌍 교차 검사
        self._hose_check_timer = self.create_timer(2.0, self._periodic_hose_check)

        self.get_logger().info(
            f'Orchestrator activated — {len(expected)} robots: {expected}')
        self._init_cbba()
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
        self.get_logger().info('Orchestrator cleaned up')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """종료 시 로그만."""
        self.get_logger().info('Orchestrator shutting down')
        return TransitionCallbackReturn.SUCCESS

    # ─────────────────── Callbacks ───────────────────

    def robot_status_callback(self, msg: RobotStatus):
        """로봇 상태 수신 → 레지스트리 갱신 + 패킷 손실 감지 (작업 4)."""
        rid = msg.robot_id

        # 입력 검증: robot_id
        if not validate_robot_id(rid):
            self.get_logger().error(
                f'Invalid robot_id rejected: "{rid}"', throttle_duration_sec=5.0)
            return

        now = self.get_clock().now().nanoseconds / 1e9

        with self._robots_lock:
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

            # ── 패킷 손실 감지 (seq_number 기반) ──
            # RobotStatus 메시지에 seq_number 필드가 있는 경우 활용.
            # 필드 미존재 시 getattr로 안전 접근하여 하위 호환성 유지.
            msg_seq = getattr(msg, 'seq_number', -1)
            if msg_seq >= 0 and r.last_seq >= 0:
                expected_seq = r.last_seq + 1
                if msg_seq != expected_seq:
                    lost = abs(msg_seq - expected_seq)
                    r.packet_loss_count += lost
                    self.get_logger().warn(
                        f'PACKET LOSS [{rid}]: expected seq={expected_seq}, '
                        f'got={msg_seq}, lost={lost}, '
                        f'total_lost={r.packet_loss_count}')
            if msg_seq >= 0:
                r.last_seq = msg_seq

            if r.comm_lost:
                r.comm_lost = False
                # C-3: 재연결 시 CENTRALIZED 복귀 발행 (lock 밖에서 수행하기 위해 플래그로 전달)
                _reconnected = True
            else:
                _reconnected = False

        # C-3: 재연결 확인 후 lock 밖에서 발행 (lock 내 발행 시 데드락 위험)
        if _reconnected:
            self.get_logger().info(f'Robot {rid} reconnected — resuming CENTRALIZED')
            self._publish_autonomy_mode(rid, 'CENTRALIZED')

        # 배터리 경고/자동귀환 (lock 밖에서 — r은 참조로 안전)
        self._check_battery(rid, r)

        # 자동 단계 전환
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
        """센서 퓨전 모듈이 반환한 명령 객체를 실행."""
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
            elif isinstance(action, DispatchRescueCommand):
                with self._robots_lock:
                    if action.robot_id in self.robots:
                        self.robots[action.robot_id].mission_lock = 'rescue'
                        from geometry_msgs.msg import Point
                        self.robots[action.robot_id].assigned_target = Point(
                            x=action.target_x, y=action.target_y, z=action.target_z)
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

    # ─────────────────── 호스 제약 관리 ───────────────────

    def _hose_status_callback(self, robot_id: str, msg: Float32MultiArray):
        """셰르파 로봇의 호스 상태 수신 → RobotRecord 갱신.

        Float32MultiArray 레이아웃 (data 인덱스):
          [0] hose_remaining_m  — 남은 호스 길이 (m)
          [1] hose_kink_risk    — 꺾임 위험도 [0.0-1.0]
          [2] hose_charged      — 가압 여부 (0.0=false, 1.0=true)
          [3] path_x_0, [4] path_y_0, [5] path_x_1, [6] path_y_1, ...
               — 호스 경유점 목록 (짝수 인덱스=x, 홀수 인덱스=y)
        """
        if len(msg.data) < 3:
            self.get_logger().warn(
                f'[호스] {robot_id}: 데이터 길이 부족 ({len(msg.data)}개) — 무시')
            return

        with self._robots_lock:
            if robot_id not in self.robots:
                self.robots[robot_id] = RobotRecord(robot_id)
            r = self.robots[robot_id]
            r.hose_remaining_m = float(msg.data[0])
            r.hose_kink_risk = float(msg.data[1])
            r.hose_charged = bool(msg.data[2] > 0.5)

            # 경유점 파싱 (인덱스 3부터 2개씩 x, y 쌍)
            path_data = msg.data[3:]
            r.hose_path = [
                (float(path_data[i]), float(path_data[i + 1]))
                for i in range(0, len(path_data) - 1, 2)
            ]

        if r.hose_kink_risk > 0.7:
            self.get_logger().warn(
                f'[호스] {robot_id}: 꺾임 위험 높음 ({r.hose_kink_risk:.2f}) '
                f'— 경로 조정 권고')

    def _check_hose_depth(self, robot_id: str, target_x: float, target_y: float) -> bool:
        """진입 깊이와 남은 호스 길이를 비교해 진입 가능 여부 판단.

        호스가 없는 로봇(hose_remaining_m = -1)은 항상 True 반환.

        Args:
            robot_id: 대상 로봇 ID
            target_x, target_y: 목표 위치 (맵 좌표)

        Returns:
            bool: True = 진입 가능 / False = 호스 부족으로 진입 불가
        """
        with self._robots_lock:
            r = self.robots.get(robot_id)

        if r is None or r.hose_remaining_m < 0:
            # 호스 없는 로봇 — 제약 없음
            return True

        if r.pose is None:
            # 위치 미확인 시 보수적으로 허용
            return True

        rx = r.pose.pose.position.x
        ry = r.pose.pose.position.y
        required_depth = math.hypot(target_x - rx, target_y - ry)

        if required_depth > r.hose_remaining_m:
            self.get_logger().warn(
                f'[호스] {robot_id}: 진입 깊이 {required_depth:.1f}m > '
                f'남은 호스 {r.hose_remaining_m:.1f}m — 진입 불가')
            return False

        return True

    def _detect_hose_conflict(self, rid_a: str, rid_b: str) -> bool:
        """두 로봇의 호스 경로가 교차하는지 확인 (선분 교차 알고리즘).

        교차 감지 시 WARN 로그 + rid_b의 assigned_target 초기화(재할당 트리거).
        호스 경로가 없는 로봇은 교차 없음으로 처리.

        Returns:
            bool: True = 교차 충돌 감지됨
        """
        with self._robots_lock:
            ra = self.robots.get(rid_a)
            rb = self.robots.get(rid_b)

        if ra is None or rb is None:
            return False
        if len(ra.hose_path) < 2 or len(rb.hose_path) < 2:
            # 경유점이 2개 미만이면 선분 구성 불가
            return False

        def _cross(o, a, b):
            """외적으로 방향 판별."""
            return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

        def _on_segment(p, q, r_pt):
            """점 r_pt가 선분 pq 위에 있는지 확인."""
            return (min(p[0], q[0]) <= r_pt[0] <= max(p[0], q[0]) and
                    min(p[1], q[1]) <= r_pt[1] <= max(p[1], q[1]))

        def _segments_intersect(p1, p2, p3, p4):
            """두 선분 (p1-p2)와 (p3-p4)의 교차 여부."""
            d1 = _cross(p3, p4, p1)
            d2 = _cross(p3, p4, p2)
            d3 = _cross(p1, p2, p3)
            d4 = _cross(p1, p2, p4)

            if ((d1 > 0 < d2) or (d1 < 0 > d2)) and \
               ((d3 > 0 < d4) or (d3 < 0 > d4)):
                return True
            # 동일선상(collinear) 처리
            if d1 == 0 and _on_segment(p3, p4, p1):
                return True
            if d2 == 0 and _on_segment(p3, p4, p2):
                return True
            if d3 == 0 and _on_segment(p1, p2, p3):
                return True
            if d4 == 0 and _on_segment(p1, p2, p4):
                return True
            return False

        # 모든 선분 쌍 비교
        path_a = ra.hose_path
        path_b = rb.hose_path
        for i in range(len(path_a) - 1):
            for j in range(len(path_b) - 1):
                if _segments_intersect(path_a[i], path_a[i + 1],
                                       path_b[j], path_b[j + 1]):
                    self.get_logger().warn(
                        f'[호스] 경로 충돌: {rid_a} ↔ {rid_b} — '
                        f'{rid_b} 재할당 필요')
                    # 충돌한 로봇(rid_b)의 목표 초기화 → 재할당 트리거
                    with self._robots_lock:
                        if rid_b in self.robots:
                            self.robots[rid_b].assigned_target = None
                            self.robots[rid_b].mission_lock = None
                    return True

        return False

    def _periodic_hose_check(self):
        """모든 셰르파 쌍의 호스 경로 교차 검사 (2초 타이머).

        hose_remaining_m >= 0인 로봇(셰르파)끼리만 교차 검사.
        교차 감지 시 _resolve_hose_conflict로 역할 재분리.
        """
        with self._robots_lock:
            sherpa_ids = [
                rid for rid, rec in self.robots.items()
                if rec.hose_remaining_m >= 0
            ]

        for i, rid_a in enumerate(sherpa_ids):
            for rid_b in sherpa_ids[i + 1:]:
                if self._detect_hose_conflict(rid_a, rid_b):
                    self._resolve_hose_conflict(rid_a, rid_b)

    def _resolve_hose_conflict(self, rid_a: str, rid_b: str):
        """호스 교차 시 역할 재분리.

        전략: 호스가 더 적게 풀린(hose_remaining_m 값이 더 큰) 로봇을
        호스공급 전담으로 전환하고, 나머지를 진압 로봇으로 배정.

        hose_remaining_m이 클수록 아직 많이 풀리지 않은 상태 →
        덜 전진한 쪽이 뒤에서 릴을 관리하기에 적합.

        Args:
            rid_a: 첫 번째 셰르파 로봇 ID
            rid_b: 두 번째 셰르파 로봇 ID
        """
        with self._robots_lock:
            rec_a = self.robots.get(rid_a)
            rec_b = self.robots.get(rid_b)

        if rec_a is None or rec_b is None:
            return

        # 호스를 더 많이 남긴(덜 풀린) 로봇이 공급 전담
        if rec_a.hose_remaining_m > rec_b.hose_remaining_m:
            supply_robot, attack_robot = rid_a, rid_b
        else:
            supply_robot, attack_robot = rid_b, rid_a

        self.get_logger().warn(
            f'[호스충돌] {rid_a} <-> {rid_b} 교차 감지. '
            f'{supply_robot}을 hose_supply 전담으로 전환, '
            f'{attack_robot}을 fire_attack으로 유지')

        # 공급 로봇: 현재 위치 정지 + 역할 전환
        with self._robots_lock:
            if supply_robot in self.robots:
                self.robots[supply_robot].role = 'hose_supply'
        self._cancel_goal(supply_robot)

        # 진압 로봇: 역할 전환 + 경로 재계획 (교차 방향 회피)
        with self._robots_lock:
            if attack_robot in self.robots:
                self.robots[attack_robot].role = 'fire_attack'
        self._replan_avoiding_hose(attack_robot, supply_robot)

        # 호스 충돌 이벤트 발행 (대시보드 / 외부 모니터링 연동)
        if self._hose_conflict_pub is not None:
            import json
            event_msg = String()
            event_msg.data = json.dumps({
                'rid_a': rid_a,
                'rid_b': rid_b,
                'resolution': f'{supply_robot}→hose_supply, {attack_robot}→fire_attack',
            }, ensure_ascii=False)
            self._hose_conflict_pub.publish(event_msg)

    # ─────────────────── CBBA 통합 할당 ───────────────────

    def _init_cbba(self):
        """CBBA 할당기 초기화. on_activate에서 호출."""
        self.allocator = CBBAAllocator(self.get_logger())
        self._cbba_realloc_timer = self.create_timer(
            10.0,  # 10초 주기 재할당 (소방 현장 동적 변화 대응)
            self._cbba_periodic_realloc,
        )
        self._last_cbba_assignments = {}  # 마지막 할당 결과 캐시
        self.get_logger().info('CBBA allocator initialized (realloc every 10s)')

    def _convert_robots_to_cbba(self) -> list:
        """오케스트레이터 RobotRecord → CBBA RobotRecord 변환.

        comm_lost 로봇, hose_supply 전담 로봇은 제외.
        호스 제약 반영: hose_remaining_m이 낮은 로봇은 먼 임무 비선호 (capability에 반영하지 않고,
        오케스트레이터 측에서 할당 후 검증).
        """
        with self._robots_lock:
            snapshot = dict(self.robots)

        cbba_robots = []
        for rid, r in snapshot.items():
            if r.comm_lost:
                continue
            # hose_supply 전담은 CBBA 할당 대상에서 제외 (현 위치 고정)
            if r.role == 'hose_supply':
                continue

            # pose 변환: PoseStamped → (x, y, z) 튜플
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

    def _build_task_list(self, fire_pt=None, severity='medium') -> list:
        """현재 상황을 CBBA Task 목록으로 변환.

        소방 시나리오 임무 타입:
          - inspect_fire: 화점 접근 진압 (UGV, thermal 필요)
          - monitor: 화점 상공 감시 (드론, can_fly 필요)
          - explore: 잔여 프론티어 탐색 (모든 로봇)
          - rescue: 구조 대상 접근 (has_gripper 권장)
        """
        tasks = []
        severity_to_priority = {
            'low': 0.3, 'medium': 0.5, 'high': 0.8, 'critical': 1.0
        }

        # 1) 화재 대응 임무 (최우선)
        if fire_pt is not None:
            fire_priority = severity_to_priority.get(severity, 0.5)

            # 지상 진압 임무 (UGV)
            tasks.append(CbbaTask(
                task_id='fire_suppress_ground',
                location=(fire_pt.x, fire_pt.y, 0.0),
                task_type='inspect_fire',
                required_capabilities=['has_thermal'],
                priority=fire_priority,
            ))

            # 항공 감시 임무 (드론)
            tasks.append(CbbaTask(
                task_id='fire_monitor_aerial',
                location=(fire_pt.x, fire_pt.y, 8.0),
                task_type='monitor',
                required_capabilities=['can_fly'],
                priority=fire_priority * 0.9,  # 진압보다 약간 낮은 우선순위
            ))

        # 2) 추가 화점 (fire_alerts에서 미처리 화점 추출)
        if self.sensor_fusion.fire_alerts:
            processed_locations = set()
            if fire_pt:
                processed_locations.add((round(fire_pt.x, 1), round(fire_pt.y, 1)))

            for i, alert in enumerate(self.sensor_fusion.fire_alerts[-5:]):  # 최근 5건만
                loc = alert.location.point
                loc_key = (round(loc.x, 1), round(loc.y, 1))
                if loc_key in processed_locations:
                    continue
                processed_locations.add(loc_key)

                alert_priority = severity_to_priority.get(
                    getattr(alert, 'severity', 'medium'), 0.5)
                tasks.append(CbbaTask(
                    task_id=f'fire_alert_{i}',
                    location=(loc.x, loc.y, 0.0),
                    task_type='inspect_fire',
                    required_capabilities=['has_thermal'],
                    priority=alert_priority * 0.8,
                ))

        # 3) 탐색 임무 (잔여 프론티어가 있는 경우)
        with self._robots_lock:
            snapshot = dict(self.robots)

        total_frontiers = sum(r.frontiers_remaining for r in snapshot.values())
        if total_frontiers > 0 and fire_pt is None:
            for i, (rid, r) in enumerate(snapshot.items()):
                if r.comm_lost or r.frontiers_remaining == 0:
                    continue
                tasks.append(CbbaTask(
                    task_id=f'explore_{rid}',
                    location=(0.0, 0.0, 0.0),
                    task_type='explore',
                    required_capabilities=[],
                    priority=0.3,
                ))

        # 4) 구조 임무 (피해자 감지 시)
        if self.sensor_fusion.victims_detected:
            for i, victim in enumerate(self.sensor_fusion.victims_detected[-3:]):
                v_pt = getattr(victim, 'location', None)
                if v_pt is None:
                    continue
                tasks.append(CbbaTask(
                    task_id=f'rescue_{i}',
                    location=(v_pt.point.x, v_pt.point.y, 0.0),
                    task_type='rescue',
                    required_capabilities=[],
                    priority=0.9,
                ))

        return tasks

    def _apply_cbba_assignments(self, assignments: dict, fire_pt=None):
        """CBBA 할당 결과를 실행: goal 발행, mission_lock 갱신.

        Args:
            assignments: {robot_id: CbbaTask} — CBBA 결과
            fire_pt: 화재 위치 (Point) — primary_responder 결정용
        """
        with self._robots_lock:
            for rid, task in assignments.items():
                if rid not in self.robots:
                    continue
                r = self.robots[rid]

                # 호스 길이 제약 사후 검증 (CBBA는 호스를 모름)
                if r.hose_remaining_m >= 0:
                    target_x, target_y = task.location[0], task.location[1]
                    if not self._check_hose_depth(rid, target_x, target_y):
                        self.get_logger().warn(
                            f'CBBA 할당 거부: {rid} → {task.task_id} — 호스 부족')
                        continue

                # mission_lock 갱신
                if task.task_type in ('inspect_fire', 'rescue'):
                    r.mission_lock = 'fire_response' if 'fire' in task.task_type else 'rescue'
                    r.assigned_target = Point(
                        x=task.location[0], y=task.location[1], z=task.location[2])

                # primary_responder 결정 (지상 진압 임무 할당자)
                if task.task_id == 'fire_suppress_ground' and fire_pt is not None:
                    self.primary_responder = rid
                    self.get_logger().warn(
                        f'CBBA PRIMARY RESPONDER: {rid} → {task.task_id}')

        # 드론 웨이포인트 발행 (lock 밖에서)
        for rid, task in assignments.items():
            if task.task_type == 'monitor':
                topic = f'/{rid}/drone/waypoint'
                if rid not in self.drone_wp_pubs:
                    self.drone_wp_pubs[rid] = self.create_publisher(
                        PoseStamped, topic, 10)
                wp = PoseStamped()
                wp.header.stamp = self.get_clock().now().to_msg()
                wp.header.frame_id = 'map'
                wp.pose.position.x = task.location[0]
                wp.pose.position.y = task.location[1]
                wp.pose.position.z = task.location[2]
                self.drone_wp_pubs[rid].publish(wp)
                self.get_logger().warn(
                    f'CBBA DRONE {rid} → ({task.location[0]:.1f}, '
                    f'{task.location[1]:.1f}, {task.location[2]:.1f})')

        self._last_cbba_assignments = dict(assignments)
        self.get_logger().info(
            f'CBBA 할당 완료: {len(assignments)}대 배정 '
            f'({", ".join(f"{k}→{v.task_id}" for k, v in assignments.items())})')

    def _execute_fire_response_cbba(self, fire_pt, severity: str):
        """CBBA 기반 화재 대응 — _execute_fire_response의 CBBA 버전.

        기존 최근접-UGV 로직 대신 CBBA 경매로 최적 할당 결정.
        소방 전술(FIRE_TACTICS) 체크는 기존 로직 재사용.
        """
        self._current_fire_severity = severity

        # 소방 전술 체크 (기존 로직 유지)
        fire_type = getattr(
            self.sensor_fusion.fire_alerts[-1] if self.sensor_fusion.fire_alerts else None,
            'fire_type', 'general') or 'general'
        tactics = FIRE_TACTICS.get(fire_type, FIRE_TACTICS['general'])

        if not tactics['robot_capable']:
            self.get_logger().error(
                f'[TTS] 화재 유형 "{fire_type}" — 로봇 자율 대응 불가. '
                f'지휘관 즉시 현장 확인 요망.')
        if tactics['human_mandatory']:
            self.get_logger().error(
                f'[TTS] 사람만 수행 가능한 작업: {tactics["human_mandatory"]} — '
                f'지휘관에게 전달.')

        # CBBA 할당
        cbba_robots = self._convert_robots_to_cbba()
        tasks = self._build_task_list(fire_pt=fire_pt, severity=severity)

        if not cbba_robots or not tasks:
            self.get_logger().warn('CBBA: 가용 로봇 또는 임무 없음 — fallback to nearest')
            self._execute_fire_response(fire_pt, severity)
            return

        assignments = self.allocator.allocate(cbba_robots, tasks)
        self._apply_cbba_assignments(assignments, fire_pt=fire_pt)

        # 심각도별 대응 (기존 로직)
        if severity == 'critical':
            self.get_logger().error(
                '[TTS] 긴급: 화재 CRITICAL 판정 — 즉시 진압 필요.')

        # 화재 확산 추정
        spread_info = self.sensor_fusion.estimate_fire_spread()
        if spread_info['trend'] == 'growing' and spread_info['spread_rate'] > 10.0:
            self.get_logger().error(
                f'[TTS] 화재 급속 확산: {spread_info["spread_rate"]:.1f}K/s')

    def _cbba_periodic_realloc(self):
        """주기적 CBBA 재할당 (10초 주기).

        실행 조건:
          - FIRE_RESPONSE 단계이고 활성 화재가 있을 때
        목적: 로봇 탈락·추가 화점·호스 고갈 등 동적 변화에 대응.
        """
        if not hasattr(self, 'allocator'):
            return

        from argos_interfaces.msg import MissionState as MS
        if self.stage != MS.STAGE_FIRE_RESPONSE:
            return

        if self.fire_response_target is None:
            return

        # 재할당
        cbba_robots = self._convert_robots_to_cbba()
        tasks = self._build_task_list(
            fire_pt=self.fire_response_target,
            severity=getattr(self, '_current_fire_severity', 'medium'),
        )

        if not cbba_robots or not tasks:
            return

        new_assignments = self.allocator.allocate(cbba_robots, tasks)

        # 변화 감지: 할당이 바뀌었을 때만 적용
        if self._assignments_changed(new_assignments):
            self.get_logger().info('CBBA 재할당: 상황 변화 감지 → 임무 재분배')
            self._apply_cbba_assignments(new_assignments, fire_pt=self.fire_response_target)

    def _assignments_changed(self, new_assignments: dict) -> bool:
        """이전 할당과 비교하여 변화 여부 판단."""
        if not self._last_cbba_assignments:
            return bool(new_assignments)

        old_map = {k: v.task_id for k, v in self._last_cbba_assignments.items()}
        new_map = {k: v.task_id for k, v in new_assignments.items()}
        return old_map != new_map

    def _cancel_goal(self, robot_id: str):
        """지정 로봇에 정지 명령 발행 (목표 취소).

        robot_stop_pubs 발행자를 재사용하거나 신규 생성.
        """
        if robot_id not in self.robot_stop_pubs:
            self.robot_stop_pubs[robot_id] = self.create_publisher(
                TwistStamped, f'/{robot_id}/cmd_vel', 10)
        stop_cmd = TwistStamped()
        stop_cmd.header.stamp = self.get_clock().now().to_msg()
        self.robot_stop_pubs[robot_id].publish(stop_cmd)
        self.get_logger().info(f'[호스충돌] {robot_id} 정지 명령 발행 (목표 취소)')

    def _replan_avoiding_hose(self, robot_id: str, supply_robot_id: str):
        """호스 교차 회피 경로 재계획 (단순 구현).

        교차점 반대 방향으로 1m 이동 후 기존 임무 재탐색.
        supply_robot의 현재 위치를 기준으로 회피 방향 계산.

        Args:
            robot_id: 경로를 재계획할 진압 로봇 ID
            supply_robot_id: 공급 로봇 ID (회피 기준점)
        """
        with self._robots_lock:
            r_attack = self.robots.get(robot_id)
            r_supply = self.robots.get(supply_robot_id)

        if r_attack is None or r_attack.pose is None:
            self.get_logger().warn(
                f'[호스충돌] {robot_id} 위치 미확인 — 재계획 스킵')
            return

        ax = r_attack.pose.pose.position.x
        ay = r_attack.pose.pose.position.y

        # 공급 로봇 위치가 있으면 그 반대 방향으로 1m 이동
        if r_supply is not None and r_supply.pose is not None:
            sx = r_supply.pose.pose.position.x
            sy = r_supply.pose.pose.position.y
            dx = ax - sx
            dy = ay - sy
            dist = math.hypot(dx, dy)
            if dist > 1e-6:
                # 정규화 후 1m 회피 목표 생성
                nx = ax + (dx / dist) * 1.0
                ny = ay + (dy / dist) * 1.0
            else:
                # 두 로봇이 동일 위치 → x축 +1m 회피
                nx, ny = ax + 1.0, ay
        else:
            # 공급 로봇 위치 없음 → x축 +1m 회피
            nx, ny = ax + 1.0, ay

        # 임시 웨이포인트 발행 (드론 웨이포인트 채널 재사용)
        # TODO: UGV 전용 웨이포인트 토픽 분리 시 여기 교체
        topic = f'/{robot_id}/drone/waypoint'
        if robot_id not in self.drone_wp_pubs:
            self.drone_wp_pubs[robot_id] = self.create_publisher(
                PoseStamped, topic, 10)
        wp = PoseStamped()
        wp.header.stamp = self.get_clock().now().to_msg()
        wp.header.frame_id = 'map'
        wp.pose.position.x = nx
        wp.pose.position.y = ny
        wp.pose.position.z = 0.0
        self.drone_wp_pubs[robot_id].publish(wp)
        self.get_logger().warn(
            f'[호스충돌] {robot_id} 재경로: ({ax:.1f},{ay:.1f}) → '
            f'({nx:.1f},{ny:.1f}) (교차 방향 1m 회피)')

    # ─────────────────── Core Logic ───────────────────

    def _check_battery(self, rid: str, r: 'RobotRecord'):
        """배터리 수준 체크: 경고 → 자동 귀환, 회복 시 플래그 리셋."""
        if (r.battery <= self.BATTERY_CRITICAL
                and r.state != RobotStatus.STATE_RETURNING
                and not r.battery_critical_acted):
            r.battery_critical_acted = True
            self.get_logger().error(
                f'BATTERY CRITICAL: {rid} at {r.battery:.0f}% — auto-return')
            if rid not in self.robot_stop_pubs:
                self.robot_stop_pubs[rid] = self.create_publisher(
                    TwistStamped, f'/{rid}/cmd_vel', 10)
            stop_cmd = TwistStamped()
            stop_cmd.header.stamp = self.get_clock().now().to_msg()
            self.robot_stop_pubs[rid].publish(stop_cmd)
        elif r.battery <= self.BATTERY_WARNING and not r.battery_warned:
            r.battery_warned = True
            self.get_logger().warn(
                f'BATTERY LOW: {rid} at {r.battery:.0f}%')
        # O5: 배터리 회복 시 플래그 리셋 (배터리 교체/충전 시나리오)
        if r.battery > self.BATTERY_WARNING and r.battery_warned:
            r.battery_warned = False
        if r.battery > self.BATTERY_CRITICAL and r.battery_critical_acted:
            r.battery_critical_acted = False

    def check_heartbeats(self):
        """주기적 heartbeat 점검: 타임아웃 시 통신 두절 판정 + 자율 모드 전환 발행."""
        now = self.get_clock().now().nanoseconds / 1e9
        lost_count = 0

        with self._robots_lock:
            snapshot = dict(self.robots)

        for rid, r in snapshot.items():
            if r.last_seen == 0.0:
                continue

            elapsed = now - r.last_seen
            if elapsed > self.HEARTBEAT_TIMEOUT and not r.comm_lost:
                # C-3: 두절 감지 → LOCAL_AUTONOMY 전환 발행
                r.comm_lost = True
                r.state = RobotStatus.STATE_COMM_LOST
                self.get_logger().warn(
                    f'Robot {rid} comm lost — switching to LOCAL_AUTONOMY')
                self._publish_autonomy_mode(rid, 'LOCAL_AUTONOMY')

            if r.comm_lost:
                lost_count += 1

        active_count = sum(1 for r in snapshot.values() if r.last_seen > 0)
        if active_count > 0 and lost_count > active_count / 2:
            self.get_logger().error(
                f'CRITICAL: {lost_count}/{active_count} robots comm lost')

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

    def _execute_fire_response(self, fire_pt: Point, severity: str):
        """화재 대응 전술 실행: 최근접 UGV 배정 + 드론 화점 상공 배치.

        작업 3 추가:
        - 화재 유형별 robot_capable 체크 → human_mandatory 작업 지휘관 통보
        - critical 판단 시 TTS prefix "[TTS]" 추가 (대시보드 Web Speech API 연동)
        작업 2 추가:
        - 충돌 방지: 대상 위치 2m 이내 다른 로봇 있으면 파견 보류
        """
        self._current_fire_severity = severity

        # 화재 유형 추론 (간이 — 실제는 YOLO 분류 결과 사용)
        # 기본 'general'로 처리, FireAlert에 fire_type 필드가 있으면 활용
        fire_type = getattr(
            self.sensor_fusion.fire_alerts[-1] if self.sensor_fusion.fire_alerts else None,
            'fire_type', 'general') or 'general'
        tactics = FIRE_TACTICS.get(fire_type, FIRE_TACTICS['general'])

        # ── 로봇/인간 역할 구분 (작업 3) ──
        if not tactics['robot_capable']:
            # 로봇 대응 불가 화재 유형 → 지휘관에게 알림
            self.get_logger().error(
                f'[TTS] 화재 유형 "{fire_type}" — 로봇 자율 대응 불가. '
                f'지휘관 즉시 현장 확인 요망.')
        if tactics['human_mandatory']:
            self.get_logger().error(
                f'[TTS] 사람만 수행 가능한 작업: {tactics["human_mandatory"]} — '
                f'지휘관에게 전달.')

        # 1) 최근접 UGV 선정
        # 역할 기반 우선순위: fire_attack > explore > 기타 (hose_supply 제외)
        with self._robots_lock:
            snapshot = dict(self.robots)
        best_ugv = None
        best_dist = float('inf')
        for rid, r in snapshot.items():
            if r.robot_type != 'ugv' or r.comm_lost or r.pose is None:
                continue
            # hose_supply 역할 로봇은 진압 임무 수신 거부
            if r.role == 'hose_supply':
                continue
            dx = r.pose.pose.position.x - fire_pt.x
            dy = r.pose.pose.position.y - fire_pt.y
            dist = math.hypot(dx, dy)
            # fire_attack 역할은 거리에 패널티 없이 우선 선정
            if r.role == 'fire_attack':
                dist *= 0.5  # fire_attack 로봇 우선 선택 (가중치 보정)
            if dist < best_dist:
                best_dist = dist
                best_ugv = rid

        if best_ugv:
            # 충돌 방지 체크 (작업 2)
            with self._robots_lock:
                _snapshot_for_collision = dict(self.robots)
            if self.sensor_fusion.check_collision_risk(fire_pt, best_ugv, _snapshot_for_collision):
                self.get_logger().warn(
                    f'Dispatch to {best_ugv} deferred — collision risk at fire point')
            # 호스 길이 제약 체크 (셰르파 전용 — 호스 없는 로봇은 자동 통과)
            elif not self._check_hose_depth(best_ugv, fire_pt.x, fire_pt.y):
                self.get_logger().warn(
                    f'Dispatch to {best_ugv} deferred — hose length insufficient')
            else:
                self.primary_responder = best_ugv
                self.get_logger().warn(
                    f'PRIMARY RESPONDER: {best_ugv} (dist={best_dist:.1f}m)')
        else:
            self.get_logger().warn('No UGV available for fire response')

        # 2) 드론을 화점 상공으로 이동
        for rid, r in snapshot.items():
            if r.robot_type != 'drone' or r.comm_lost:
                continue
            topic = f'/{rid}/drone/waypoint'
            if rid not in self.drone_wp_pubs:
                self.drone_wp_pubs[rid] = self.create_publisher(
                    PoseStamped, topic, 10)
            wp = PoseStamped()
            wp.header.stamp = self.get_clock().now().to_msg()
            wp.header.frame_id = 'map'
            wp.pose.position.x = fire_pt.x
            wp.pose.position.y = fire_pt.y
            wp.pose.position.z = 8.0  # 정찰 고도
            self.drone_wp_pubs[rid].publish(wp)
            self.get_logger().warn(
                f'DRONE {rid} → fire location '
                f'({fire_pt.x:.1f}, {fire_pt.y:.1f}, 8.0m)')

        # 3) 심각도별 대응 + TTS (작업 3)
        if severity == 'critical':
            self.get_logger().error(
                '[TTS] 긴급: 화재 CRITICAL 판정 — 즉시 진압 필요. '
                '지휘관 현장 통제 요청.')
        else:
            self.get_logger().warn(
                f'Fire severity: {severity} — 감시 모드 유지')

        # 4) 화재 확산 추정 (작업 3)
        spread_info = self.sensor_fusion.estimate_fire_spread()
        if spread_info['trend'] == 'growing' and spread_info['spread_rate'] > 10.0:
            self.get_logger().error(
                f'[TTS] 화재 급속 확산 감지: {spread_info["spread_rate"]:.1f}K/s — '
                f'추정 반경 {spread_info["estimated_radius_m"]:.1f}m. '
                f'추가 진압 로봇 파견 검토.')

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
