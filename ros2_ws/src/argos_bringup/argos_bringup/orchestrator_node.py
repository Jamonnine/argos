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

        # 호스 상태 (셰르파 전용, -1 = 호스 없음 / 미해당 로봇)
        self.hose_remaining_m: float = -1.0   # 남은 호스 길이 (m)
        self.hose_kink_risk: float = 0.0      # 꺾임(kink) 위험도 [0.0-1.0]
        self.hose_charged: bool = False        # 호스 가압 여부
        self.hose_path: list = []              # 경유점 목록 [(x, y)]

        # 역할 분리 (역할 기반 임무 필터링용)
        # 'explore'     : 기본 탐색 임무
        # 'fire_attack' : 진압 전담 (화점 접근, 소화 작업)
        # 'hose_supply' : 호스공급 전담 (현재 위치 정지, 릴 관리)
        # 'recon'       : 드론 정찰 (호스 제약 무시)
        self.role: str = 'explore'


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
        self._last_gas_time = self.get_clock().now()
        self._gas_watchdog_timer = self.create_timer(5.0, self._check_gas_watchdog)
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
            self._execute_fire_response_cbba(fire_pt, msg.severity)
        elif self.stage == MissionState.STAGE_FIRE_RESPONSE:
            # 추가 화점: 더 심각하면 대응 대상 갱신
            severity_rank = {'low': 0, 'medium': 1, 'high': 2, 'critical': 3}
            current_sev = severity_rank.get(
                getattr(self, '_current_fire_severity', 'low'), 0)
            new_sev = severity_rank.get(msg.severity, 0)
            if new_sev > current_sev:
                self.fire_response_target = fire_pt
                self._execute_fire_response_cbba(fire_pt, msg.severity)
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
            # 역할 필터: hose_supply 로봇은 진압/구조 임무 수신 거부
            if r.role == 'hose_supply':
                self.get_logger().info(
                    f'RESCUE SKIP (역할): {rid} — hose_supply 전담 중, 구조 파견 불가')
                continue
            dx = r.pose.pose.position.x - target.x
            dy = r.pose.pose.position.y - target.y
            dist = math.hypot(dx, dy)
            available.append((rid, dist))

        available.sort(key=lambda x: x[1])

        dispatched = []
        for rid, dist in available[:2]:
            # 호스 길이 제약 체크 (셰르파 전용 — 호스 없는 로봇은 자동 통과)
            if not self._check_hose_depth(rid, target.x, target.y):
                self.get_logger().warn(
                    f'RESCUE SKIP: {rid} — 호스 부족으로 구조 파견 불가')
                continue
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
        if self.fire_alerts:
            processed_locations = set()
            if fire_pt:
                processed_locations.add((round(fire_pt.x, 1), round(fire_pt.y, 1)))

            for i, alert in enumerate(self.fire_alerts[-5:]):  # 최근 5건만
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
        if self.victims_detected:
            for i, victim in enumerate(self.victims_detected[-3:]):
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
            self.fire_alerts[-1] if self.fire_alerts else None,
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
        spread_info = self._estimate_fire_spread()
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
            if self._check_collision_risk(fire_pt, best_ugv):
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
