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
from geometry_msgs.msg import Point, Twist, TwistStamped, PoseStamped
from argos_interfaces.msg import (
    RobotStatus, FireAlert, MissionState,
    GasReading, VictimDetection, StructuralAlert, AudioEvent,
)
from argos_bringup.validation_utils import (
    validate_robot_id, validate_severity, validate_danger_level,
    validate_timestamp, clamp_sensor,
)


# ── 소방 전술 모듈 (소방 전문가 권고) ──

# 화재 유형별 대응 전술
FIRE_TACTICS = {
    'electrical': {
        'suppression': 'water_fog',       # 전기화재: 무상수 (감전 방지)
        'robot_capable': True,            # 로봇 대응 가능
        'human_mandatory': ['전원차단'],    # 사람만 가능한 작업
        'approach_distance': 5.0,         # 최소 접근 거리 (m)
    },
    'oil': {
        'suppression': 'foam_spray',      # 유류화재: 포말 소화
        'robot_capable': True,
        'human_mandatory': ['포말탱크 연결'],
        'approach_distance': 3.0,
    },
    'gas': {
        'suppression': 'ventilation',     # 가스화재: 환기 + 가스 차단
        'robot_capable': False,           # 가스 밸브 차단은 사람만
        'human_mandatory': ['가스밸브 차단', '환기'],
        'approach_distance': 10.0,        # 폭발 위험으로 원거리
    },
    'general': {
        'suppression': 'water_fog',       # 일반화재
        'robot_capable': True,
        'human_mandatory': [],
        'approach_distance': 2.0,
    },
}

# 센서 합의 반경 (m) — 다른 로봇이 이 거리 이내에 있으면 동일 화점 확인으로 판정
CONSENSUS_RADIUS = 5.0
# 충돌 방지 안전 거리 (m) — 이 거리 이내에 다른 로봇이 있으면 파견 보류
COLLISION_SAFE_DISTANCE = 2.0


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
        self.battery_warned = False  # 배터리 경고 중복 방지
        self.battery_critical_acted = False  # 배터리 위험 중복 방지
        # 센서 퓨전 전문가 권고: busy flag (중복 파견 방지)
        self.mission_lock = None     # 'fire_response' | 'rescue' | None
        self.assigned_target = None  # 현재 할당된 목표 위치 (Point)
        # 통신 품질 추적 (패킷 손실 감지, 작업 4)
        self.last_seq = -1           # 마지막 수신 seq_number (-1=미수신)
        self.packet_loss_count = 0   # 누적 패킷 손실 카운트


class OrchestratorNode(LifecycleNode):
    """오케스트레이터 노드 — LifecycleNode 패턴.

    - __init__    : 파라미터 선언만 수행 (토픽·타이머·서비스 생성 금지)
    - on_configure: 파라미터 읽기 + 상태 변수 초기화
    - on_activate : 구독/발행/서비스/타이머 생성 → 임무 시작
    - on_deactivate: 모든 핸들 destroy → 임무 중단
    - on_cleanup  : 내부 상태 리셋
    - on_shutdown : 로그만

    M1: TODO — 이 클래스는 현재 1000줄 이상으로 SRP를 위반함.
    향후 리팩토링 시 아래 3개 노드로 분리 권장:
      - MissionStateMachine: stage 전환, 타이머, 상태 발행
      - SensorFusion: 8중 센싱 콜백, 위험도 집계
      - RobotDispatcher: 파견 로직, 로봇 레지스트리, heartbeat
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
        self.fire_alerts = None
        self.start_time = None
        self.paused = False
        self.robot_stop_pubs = {}
        self.drone_wp_pubs = {}
        self.primary_responder = None
        self.fire_response_target = None
        self.return_start_time = None
        self._thermal_history = None
        self.gas_danger_level = 'safe'
        self.victims_detected = None
        self.blocked_areas = None
        self.audio_alerts = None
        self._evacuation_recommended = False
        self._gas_critical_count = 0
        self._gas_critical_threshold = 2

        # H1: gas 센서 watchdog — 마지막 수신 시간 추적
        self._last_gas_time = None

        # 구독/발행/서비스/타이머 핸들 (on_activate에서 생성, on_deactivate에서 해제)
        self.status_sub = None
        self.fire_sub = None
        self._sensor_subs = []  # 8중 센싱 구독 핸들 리스트
        self.mission_pub = None
        self.stop_srv = None
        self.resume_srv = None
        self._gas_watchdog_timer = None
        self.state_timer = None
        self.heartbeat_timer = None

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
        self.fire_alerts = deque(maxlen=100)
        self.start_time = self.get_clock().now()
        self.paused = False
        self.robot_stop_pubs = {}
        self.drone_wp_pubs = {}
        self.primary_responder = None
        self.fire_response_target = None
        self.return_start_time = None

        # 화재 확산 추적용 열화상 시계열 (작업 3)
        self._thermal_history = deque(maxlen=50)

        # 예상 로봇 사전 등록
        for rid in expected:
            self.robots[rid] = RobotRecord(rid)

        # 센싱 상태 초기화
        self.gas_danger_level = 'safe'
        self.victims_detected = deque(maxlen=200)
        # H7: blocked_areas → deque로 교체 (thread-safety + 자동 용량 제한)
        self.blocked_areas = deque(maxlen=100)
        self.audio_alerts = deque(maxlen=50)
        self._evacuation_recommended = False
        self._gas_critical_count = 0
        self._gas_critical_threshold = 2

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
            self.fire_alert_callback, reliable_qos)

        # --- 8중 센싱 구독 (QoS 분화, 통신 전문가 권고) ---
        self._sensor_subs = []
        expected = list(self.robots.keys())
        for rid in expected:
            self._sensor_subs.append(self.create_subscription(
                GasReading, f'/{rid}/gas/reading',
                self.gas_callback, gas_qos))
            self._sensor_subs.append(self.create_subscription(
                VictimDetection, f'/{rid}/victim/detections',
                self.victim_callback, sensor_qos))
            self._sensor_subs.append(self.create_subscription(
                StructuralAlert, f'/{rid}/structural/alerts',
                self.structural_callback, sensor_qos))
            self._sensor_subs.append(self.create_subscription(
                AudioEvent, f'/{rid}/audio/events',
                self.audio_callback, audio_qos))

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
        # H1: gas sensor watchdog — 5초 이상 수신 없으면 경고
        self._last_gas_time = self.get_clock().now()
        self._gas_watchdog_timer = self.create_timer(5.0, self._check_gas_watchdog)

        self.get_logger().info(
            f'Orchestrator activated — {len(expected)} robots: {expected}')
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
        self.fire_alerts = None
        self.paused = False
        self.primary_responder = None
        self.fire_response_target = None
        self.return_start_time = None
        self._thermal_history = None
        self.gas_danger_level = 'safe'
        self.victims_detected = None
        self.blocked_areas = None
        self.audio_alerts = None
        self._evacuation_recommended = False
        self._gas_critical_count = 0
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
                self.get_logger().info(f'Robot {rid} reconnected')

        # 배터리 경고/자동귀환 (lock 밖에서 — r은 참조로 안전)
        self._check_battery(rid, r)

        # 자동 단계 전환
        self._auto_stage_transition()

    def fire_alert_callback(self, msg: FireAlert):
        """화점 감지 알림 → 센서 합의 검증 → 전술적 대응 실행 (작업 2)."""
        # 입력 검증
        if not validate_robot_id(msg.robot_id):
            self.get_logger().error(f'Invalid robot_id in FireAlert: "{msg.robot_id}"')
            return
        if not validate_severity(msg.severity):
            self.get_logger().error(f'Invalid severity in FireAlert: "{msg.severity}"')
            return
        # 타임스탬프 검증 (클록 스큐 감지)
        now_sec = self.get_clock().now().nanoseconds / 1e9
        msg_sec = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if not validate_timestamp(msg_sec, now_sec):
            self.get_logger().warn(
                f'Suspicious timestamp from {msg.robot_id} '
                f'(skew={abs(now_sec - msg_sec):.1f}s) — ignoring')
            return

        self.fire_alerts.append(msg)

        # 열화상 시계열에 온도 추가 (화재 확산 추정용, 작업 3)
        self._thermal_history.append((now_sec, msg.max_temperature_kelvin))

        temp_c = msg.max_temperature_kelvin - 273.15
        fire_pt = msg.location.point
        self.get_logger().warn(
            f'FIRE ALERT from {msg.robot_id}: {msg.severity} '
            f'({temp_c:.0f}C) at ({fire_pt.x:.1f}, {fire_pt.y:.1f})')

        # 화점 감지 시 FIRE_RESPONSE 단계로 전환 + 전술 실행
        if self.stage == MissionState.STAGE_EXPLORING:
            self.stage = MissionState.STAGE_FIRE_RESPONSE
            self.fire_response_target = fire_pt
            self.get_logger().warn('Stage → FIRE_RESPONSE')
            self._execute_fire_response(fire_pt, msg.severity)
        elif self.stage == MissionState.STAGE_FIRE_RESPONSE:
            # 추가 화점: 더 심각하면 대응 대상 갱신
            severity_rank = {'low': 0, 'medium': 1, 'high': 2, 'critical': 3}
            current_sev = severity_rank.get(
                getattr(self, '_current_fire_severity', 'low'), 0)
            new_sev = severity_rank.get(msg.severity, 0)
            if new_sev > current_sev:
                self.fire_response_target = fire_pt
                self._execute_fire_response(fire_pt, msg.severity)
                self.get_logger().warn(
                    f'Fire escalation: {msg.severity} — re-dispatching')

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
            if getattr(self, '_evacuation_recommended', False):
                # 지휘관이 철수를 승인함
                self._evacuation_recommended = False
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

    # ─────────────────── 8중 센싱 Callbacks ───────────────────

    def _check_gas_watchdog(self):
        """H1: gas 센서 watchdog — 5초 이상 수신 없으면 경고."""
        if self._last_gas_time is None:
            return
        elapsed = (self.get_clock().now() - self._last_gas_time).nanoseconds / 1e9
        if elapsed > 5.0:
            self.get_logger().warn(
                f'GAS SENSOR TIMEOUT: {elapsed:.1f}초 동안 수신 없음 — 센서 연결 확인 필요',
                throttle_duration_sec=10.0)

    def gas_callback(self, msg: GasReading):
        """가스 위험도 기반 임무 재할당 (입력 검증 포함)."""
        rid = msg.robot_id
        if not validate_robot_id(rid):
            return
        if not validate_danger_level(msg.danger_level):
            self.get_logger().warn(f'Invalid gas danger_level: {msg.danger_level}')
            return
        # H1: gas 수신 시각 갱신
        self._last_gas_time = self.get_clock().now()

        if msg.evacuate_recommended and msg.danger_level == 'critical':
            # Temporal smoothing: N회 연속 critical이어야 실제 판정 (오탐 방지)
            self._gas_critical_count += 1
            if self._gas_critical_count >= self._gas_critical_threshold:
                self.gas_danger_level = 'critical'
                self.get_logger().error(
                    f'GAS CRITICAL CONFIRMED ({self._gas_critical_count}회 연속) from {rid}: '
                    f'CO={msg.co_ppm:.0f}ppm O2={msg.o2_percent:.1f}% LEL={msg.lel_percent:.0f}%')
                # 소방 전문가 권고: 자동 철수 대신 PAUSED → 지휘관 승인 후 RETURNING
                if self.stage not in (MissionState.STAGE_RETURNING,
                                      MissionState.STAGE_COMPLETE,
                                      MissionState.STAGE_PAUSED):
                    self.stage = MissionState.STAGE_PAUSED
                    self.paused = True
                    self._evacuation_recommended = True
                    self.get_logger().error(
                        'EVACUATION RECOMMENDED — 지휘관 /orchestrator/resume 승인 시 철수 시작')
            else:
                self.get_logger().warn(
                    f'GAS CRITICAL from {rid} ({self._gas_critical_count}/{self._gas_critical_threshold}) '
                    f'— 확인 대기 중')
        elif msg.danger_level == 'danger':
            # H6: danger는 완전 리셋이 아닌 점진적 감소 — 짧은 복귀 후 재상승 방지
            self._gas_critical_count = max(0, self._gas_critical_count - 1)
            self.gas_danger_level = 'danger'
            self.get_logger().warn(
                f'GAS DANGER from {rid}: {msg.hazard_types} — 감시 강화')
        elif msg.danger_level in ('safe', 'caution'):
            self._gas_critical_count = 0  # critical 연속 카운트 리셋
            if self.gas_danger_level in ('danger', 'critical'):
                self.gas_danger_level = msg.danger_level

    def victim_callback(self, msg: VictimDetection):
        """피해자 감지 → 구조 우선순위 판정 + 최근접 로봇 파견."""
        rid = msg.robot_id
        if not validate_robot_id(rid):
            return
        # 타임스탬프 정렬 (센서 퓨전 전문가 권고)
        now_sec = self.get_clock().now().nanoseconds / 1e9
        msg_sec = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        msg_age = now_sec - msg_sec
        if msg_age > 5.0:
            self.get_logger().warn(
                f'Stale victim detection from {rid} ({msg_age:.1f}s old) — skipped',
                throttle_duration_sec=5.0)
            return

        loc = msg.location.point

        self.get_logger().error(
            f'VICTIM DETECTED by {rid}: '
            f'({loc.x:.1f}, {loc.y:.1f}) '
            f'conf={msg.confidence:.2f} '
            f'status={msg.estimated_status} '
            f'priority={msg.rescue_priority}')

        # 중복 방지 (5m 이내 기존 피해자)
        for v in self.victims_detected:
            dist = math.hypot(loc.x - v.x, loc.y - v.y)
            if dist < 5.0:
                return
        self.victims_detected.append(loc)

        # 우선순위 1(움직임 없음) → 최근접 UGV 파견
        if msg.rescue_priority == 1:
            self._dispatch_rescue(loc, rid)

    def structural_callback(self, msg: StructuralAlert):
        """구조물 손상 → 경로 회피 / 철수 판단."""
        rid = msg.robot_id
        loc = msg.location.point

        self.get_logger().warn(
            f'STRUCTURAL {msg.severity.upper()}: {msg.alert_type} '
            f'from {rid} at ({loc.x:.1f}, {loc.y:.1f}) '
            f'disp={msg.displacement_m:.2f}m')

        if msg.area_blocked:
            self.blocked_areas.append({
                'center': loc,
                'radius': msg.affected_radius_m,
                'type': msg.alert_type,
            })
            self.get_logger().error(
                f'AREA BLOCKED: radius={msg.affected_radius_m:.1f}m — '
                f'{msg.recommended_actions}')

        if msg.severity == 'critical':
            self.get_logger().error(
                f'CRITICAL STRUCTURAL from {rid} — 해당 로봇 즉시 철수 지휘')
            # H5: 구조물 critical 시 해당 로봇 즉시 정지 (log only → 실제 정지 명령)
            if rid not in self.robot_stop_pubs:
                self.robot_stop_pubs[rid] = self.create_publisher(
                    TwistStamped, f'/{rid}/cmd_vel', 10)
            stop_cmd = TwistStamped()
            stop_cmd.header.stamp = self.get_clock().now().to_msg()
            self.robot_stop_pubs[rid].publish(stop_cmd)
            self.get_logger().error(
                f'EVACUATION: {rid} stopped due to structural critical')

    def audio_callback(self, msg: AudioEvent):
        """음향 이벤트 → 피해자 탐색 / 폭발 경고."""
        rid = msg.robot_id
        self.audio_alerts.append(msg)

        if msg.event_type == 'cry_for_help' and msg.immediate_response_needed:
            loc = msg.estimated_location.point
            self.get_logger().error(
                f'CRY FOR HELP detected by {rid} at '
                f'({loc.x:.1f}, {loc.y:.1f}) '
                f'conf={msg.confidence:.2f} — 구조 임무 파견')
            self._dispatch_rescue(loc, rid)

        elif msg.event_type == 'explosion':
            self.get_logger().error(
                f'EXPLOSION detected by {rid} — '
                f'intensity={msg.intensity_db:.0f}dB — 전원 주의')

        elif msg.event_type == 'gas_leak':
            self.get_logger().warn(
                f'GAS LEAK sound detected by {rid} — '
                f'가스 센서 데이터 교차 확인 필요')

        elif msg.event_type == 'collapse':
            self.get_logger().error(
                f'COLLAPSE sound detected by {rid} — '
                f'구조물 모니터 데이터 교차 확인 필요')

    def _dispatch_rescue(self, target: Point, excluding_rid: str):
        """최근접 가용 로봇을 구조 대상 위치로 파견."""
        with self._robots_lock:
            snapshot = dict(self.robots)

        available = []
        for rid, r in snapshot.items():
            if rid == excluding_rid or r.comm_lost or r.pose is None:
                continue
            if r.mission_lock is not None:
                continue  # busy — 다른 임무 수행 중
            dx = r.pose.pose.position.x - target.x
            dy = r.pose.pose.position.y - target.y
            dist = math.hypot(dx, dy)
            available.append((rid, dist))

        available.sort(key=lambda x: x[1])

        dispatched = []
        for rid, dist in available[:2]:
            with self._robots_lock:
                if rid in self.robots:
                    self.robots[rid].mission_lock = 'rescue'
                    self.robots[rid].assigned_target = target
            dispatched.append((rid, dist))
            self.get_logger().warn(
                f'RESCUE DISPATCH: {rid} → '
                f'({target.x:.1f}, {target.y:.1f}) dist={dist:.1f}m')

        if not dispatched:
            self.get_logger().warn('No available robot for rescue dispatch')
        elif len(dispatched) == 1:
            self.get_logger().warn(
                f'Only 1 robot available for rescue (2 recommended)')

    # ─────────────────── 보안: 센서 합의 + 충돌 방지 (작업 2) ───────────────────

    def _verify_fire_with_consensus(self, fire_pt: Point,
                                    reporting_rid: str) -> str:
        """화점 감지 합의 메커니즘 — 다중 로봇 교차 검증.

        화점 주변 CONSENSUS_RADIUS(5m) 이내에 다른 로봇이 있고
        해당 로봇이 최근(5초 이내) 활성 상태이면 "confirmed" 반환.
        단일 로봇 감지만 있으면 "unconfirmed" 반환.

        Returns:
            str: "confirmed" | "unconfirmed"
        """
        now = self.get_clock().now().nanoseconds / 1e9
        confirming_robots = []

        with self._robots_lock:
            snapshot = dict(self.robots)

        for rid, r in snapshot.items():
            if rid == reporting_rid:
                continue
            if r.comm_lost or r.pose is None:
                continue
            # 최근 5초 이내 활성 상태인지 확인
            if now - r.last_seen > 5.0:
                continue

            # 해당 로봇이 화점 CONSENSUS_RADIUS 이내에 있는지 확인
            rx = r.pose.pose.position.x
            ry = r.pose.pose.position.y
            dist_to_fire = math.hypot(rx - fire_pt.x, ry - fire_pt.y)
            if dist_to_fire <= CONSENSUS_RADIUS:
                confirming_robots.append((rid, dist_to_fire))

        if len(confirming_robots) >= 1:
            # 2대 이상 확인 → confirmed
            self.get_logger().info(
                f'FIRE CONSENSUS confirmed by '
                f'{[r[0] for r in confirming_robots]} '
                f'(within {CONSENSUS_RADIUS}m of fire)')
            return 'confirmed'
        else:
            # 1대만 감지 → unconfirmed (단독 센서 오탐 가능성)
            self.get_logger().warn(
                f'FIRE UNCONFIRMED: only {reporting_rid} detected fire at '
                f'({fire_pt.x:.1f}, {fire_pt.y:.1f}) — awaiting 2nd confirmation')
            return 'unconfirmed'

    def _check_collision_risk(self, target: Point,
                               excluding_rid: str) -> bool:
        """충돌 방지 — 파견 대상 위치에 다른 로봇이 근접해 있는지 확인.

        COLLISION_SAFE_DISTANCE(2m) 이내에 다른 로봇이 있으면
        True(위험)를 반환하여 파견을 보류한다.

        Returns:
            bool: True = 충돌 위험 있음 (파견 보류 권고)
        """
        with self._robots_lock:
            snapshot = dict(self.robots)

        for rid, r in snapshot.items():
            if rid == excluding_rid:
                continue
            if r.comm_lost or r.pose is None:
                continue
            rx = r.pose.pose.position.x
            ry = r.pose.pose.position.y
            dist = math.hypot(rx - target.x, ry - target.y)
            if dist < COLLISION_SAFE_DISTANCE:
                self.get_logger().warn(
                    f'COLLISION RISK: {rid} is {dist:.1f}m from target '
                    f'({target.x:.1f}, {target.y:.1f}) — dispatch deferred')
                return True

        return False

    # ─────────────────── 소방: 화재 확산 예측 (작업 3) ───────────────────

    def _estimate_fire_spread(self) -> dict:
        """열화상 시계열 데이터로 화재 확산 속도 추정.

        Kalman 필터 개념을 단순화하여 적용:
          - 상태: [온도(K), 온도변화율(K/s)]
          - 관측: max_temperature_kelvin
          - 예측 출력: spread_rate(K/s), trend('growing'|'stable'|'declining')

        열화상 데이터가 3개 미만이면 추정 불가 반환.

        Returns:
            dict: {
                'spread_rate': float (K/s),
                'trend': 'growing'|'stable'|'declining',
                'estimated_radius_m': float,
                'data_points': int,
            }
        """
        if len(self._thermal_history) < 3:
            return {
                'spread_rate': 0.0,
                'trend': 'unknown',
                'estimated_radius_m': 0.0,
                'data_points': len(self._thermal_history),
            }

        # 시계열 → 온도 변화율 계산 (간이 Kalman: 최소제곱 기울기 추정)
        times = [t for t, _ in self._thermal_history]
        temps = [k for _, k in self._thermal_history]

        t0 = times[0]
        n = len(times)
        # 최소제곱 선형 회귀로 온도 변화율(기울기) 추정
        sum_x = sum(t - t0 for t in times)
        sum_y = sum(temps)
        sum_xy = sum((t - t0) * temp for t, temp in zip(times, temps))
        sum_x2 = sum((t - t0) ** 2 for t in times)
        denom = n * sum_x2 - sum_x ** 2
        if abs(denom) < 1e-9:
            spread_rate = 0.0
        else:
            spread_rate = (n * sum_xy - sum_x * sum_y) / denom  # K/s

        # 화재 반경 추정: 경험식 r = sqrt(T_excess / k), k=10 (간이 모델)
        latest_temp = temps[-1]
        ambient_temp_k = 300.0  # 상온 27°C = 300K
        temp_excess = max(0.0, latest_temp - ambient_temp_k)
        estimated_radius = math.sqrt(temp_excess / 10.0) if temp_excess > 0 else 0.0

        # 트렌드 판정
        if spread_rate > 5.0:
            trend = 'growing'
        elif spread_rate < -5.0:
            trend = 'declining'
        else:
            trend = 'stable'

        if trend == 'growing':
            self.get_logger().warn(
                f'FIRE SPREADING: rate={spread_rate:.1f}K/s, '
                f'est_radius={estimated_radius:.1f}m — 진압 로봇 추가 파견 검토')

        return {
            'spread_rate': spread_rate,
            'trend': trend,
            'estimated_radius_m': estimated_radius,
            'data_points': n,
        }

    # ─────────────────── Sensor Fusion ───────────────────

    def compute_situation_score(self) -> dict:
        """8중 센싱 데이터 가중 합산 — 종합 위험도 점수 산출."""
        gas_score = {'safe': 0.0, 'caution': 0.2, 'danger': 0.6, 'critical': 1.0
                     }.get(self.gas_danger_level, 0.0)

        fire_score = 0.0
        if self.fire_response_target is not None:
            sev = getattr(self, '_current_fire_severity', 'low')
            fire_score = {'low': 0.1, 'medium': 0.3, 'high': 0.6, 'critical': 1.0
                          }.get(sev, 0.0)

        structural_score = {'safe': 0.0, 'warning': 0.2, 'danger': 0.6, 'critical': 1.0
                            }.get(getattr(self, '_structural_level', 'safe'), 0.0)

        victim_score = min(1.0, len(self.victims_detected) * 0.3)

        audio_score = 0.0
        if self.audio_alerts:
            latest = self.audio_alerts[-1]
            if latest.immediate_response_needed:
                audio_score = 0.8
            elif latest.danger_level in ('danger', 'critical'):
                audio_score = 0.5

        weights = {
            'gas': 0.30,
            'fire': 0.25,
            'structural': 0.20,
            'victim': 0.15,
            'audio': 0.10,
        }

        total_score = (
            weights['gas'] * gas_score +
            weights['fire'] * fire_score +
            weights['structural'] * structural_score +
            weights['victim'] * victim_score +
            weights['audio'] * audio_score
        )

        if total_score >= 0.7:
            level, recommend = 'critical', 'evacuate'
        elif total_score >= 0.5:
            level, recommend = 'danger', 'pause'
        elif total_score >= 0.3:
            level, recommend = 'caution', 'monitor'
        else:
            level, recommend = 'safe', 'continue'

        return {
            'score': total_score,
            'level': level,
            'recommend': recommend,
            'factors': {
                'gas': gas_score, 'fire': fire_score,
                'structural': structural_score,
                'victim': victim_score, 'audio': audio_score,
            }
        }

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
        """주기적 heartbeat 점검: 타임아웃 시 통신 두절 판정."""
        now = self.get_clock().now().nanoseconds / 1e9
        lost_count = 0

        with self._robots_lock:
            snapshot = dict(self.robots)

        for rid, r in snapshot.items():
            if r.last_seen == 0.0:
                continue

            elapsed = now - r.last_seen
            if elapsed > self.HEARTBEAT_TIMEOUT and not r.comm_lost:
                r.comm_lost = True
                r.state = RobotStatus.STATE_COMM_LOST
                self.get_logger().warn(
                    f'COMM LOST: {rid} (no status for {elapsed:.0f}s)')

            if r.comm_lost:
                lost_count += 1

        active_count = sum(1 for r in snapshot.values() if r.last_seen > 0)
        if active_count > 0 and lost_count > active_count / 2:
            self.get_logger().error(
                f'CRITICAL: {lost_count}/{active_count} robots comm lost')

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
            self.fire_alerts[-1] if self.fire_alerts else None,
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
        with self._robots_lock:
            snapshot = dict(self.robots)
        best_ugv = None
        best_dist = float('inf')
        for rid, r in snapshot.items():
            if r.robot_type != 'ugv' or r.comm_lost or r.pose is None:
                continue
            dx = r.pose.pose.position.x - fire_pt.x
            dy = r.pose.pose.position.y - fire_pt.y
            dist = math.hypot(dx, dy)
            if dist < best_dist:
                best_dist = dist
                best_ugv = rid

        if best_ugv:
            # 충돌 방지 체크 (작업 2)
            if self._check_collision_risk(fire_pt, best_ugv):
                self.get_logger().warn(
                    f'Dispatch to {best_ugv} deferred — collision risk at fire point')
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
        spread_info = self._estimate_fire_spread()
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
                f for f in self.fire_alerts
                if f.active and (now_sec - (f.header.stamp.sec + f.header.stamp.nanosec / 1e9)) < self.fire_expiry
            ]
            if not active_fires:
                self.stage = MissionState.STAGE_EXPLORING
                self.primary_responder = None
                self.fire_response_target = None
                self._current_fire_severity = None
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
            f for f in self.fire_alerts
            if f.active and (now_sec - (f.header.stamp.sec + f.header.stamp.nanosec / 1e9)) < self.fire_expiry
        ]
        msg.fire_count = len(active_fires)
        msg.fire_locations = [f.location.point for f in active_fires]

        msg.robot_ids = list(snapshot.keys())
        msg.robot_states = [r.state for r in snapshot.values()]
        msg.robot_missions = [r.current_mission for r in snapshot.values()]

        msg.primary_responder = self.primary_responder or ''

        msg.gas_danger_level = self.gas_danger_level
        msg.victims_detected_count = len(self.victims_detected)
        msg.blocked_areas_count = len(self.blocked_areas)
        msg.audio_alerts_count = len(self.audio_alerts)

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
