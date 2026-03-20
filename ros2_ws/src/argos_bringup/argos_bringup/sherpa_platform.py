#!/usr/bin/env python3
"""sherpa_platform.py — ARGOS HR-셰르파 UGV PlatformInterface 구현

HR-셰르파(Sherpa)는 소방 진압 보조를 위한 대형 UGV.
100m 소방 호스를 탑재하여 화재 현장 깊숙이 진입한다.
호스 제약(진입 깊이, 급커브, 충수 후진)을 Nav2 이동 전에 사전 검증한다.

아키텍처:
  Orchestrator
      ↓ PlatformInterface.move_to(x, y)  ← 호스 제약 사전 체크
  SherpaPlatform (이 파일)
      ↓ Nav2 NavigateToPose Action
  Nav2 (로컬 플래너 + 글로벌 플래너)
      ↓ cmd_vel (TwistStamped)
  diff_drive_controller → Gazebo Sherpa UGV

UGVPlatform과의 차이:
  - 호스 제약 사전 체크 (_check_hose_constraints)
  - /hose/status 토픽 구독 (Float32MultiArray)
  - return_home 시 호스 릴 회수 시퀀스 포함
  - 배터리 모델: 20kWh / 5시간 (5kph 정속 기준)
  - 최대 속도: 1.4 m/s (5kph)

호스 상태 토픽 (/hose/status — Float32MultiArray):
  data[0]: 호스 잔여 길이 (m, 0 ~ 100)
  data[1]: 꺾임 위험도 (0.0=안전, 1.0=위험)
  data[2]: 충수 여부 (0.0=건수, 1.0=충수)

ROS 파라미터:
  robot_id           (string) : 로봇 식별자 (기본: "sherpa1")
  home_x/home_y      (float)  : 귀환 좌표 (기본: 0.0, 0.0)
  battery_start      (float)  : 초기 배터리 % (기본: 100.0)
  nav_timeout        (float)  : Nav2 Goal 타임아웃 초 (기본: 120.0)
  map_frame          (string) : TF 맵 프레임 (기본: "map")
  base_frame         (string) : TF 베이스 프레임 (기본: "base_footprint")
  hose_total_length  (float)  : 호스 총 길이 m (기본: 100.0)
  hose_min_bend_radius (float): 호스 최소 굽힘 반경 m (기본: 0.5)
  hose_mode          (string) : "active"(토픽 사용) | "offline"(호스 없음 모드)
"""

import math
import threading
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Float32MultiArray
from tf2_ros import Buffer, TransformListener
import tf2_ros

from argos_bringup.platform_interface import PlatformInterface, RobotCapabilities

# ── 배터리 상수 (HR-셰르파 하드웨어 스펙) ────────────────────────────────────
# 20kWh 배터리, 5kph(1.4m/s) 정속 5시간 운용 기준
_SHERPA_BATTERY_KWH = 20.0          # kWh
_SHERPA_ENDURANCE_HOURS = 5.0       # 시간 (이동 시)
_SHERPA_DRAIN_RATE_PCT_PER_MIN = 100.0 / (_SHERPA_ENDURANCE_HOURS * 60.0)  # %/분 ≈ 0.333
_SHERPA_IDLE_FACTOR = 0.25          # 정지 시 소비율 (이동의 25%)

# 호스 스펙 상수
_HOSE_DIAMETER_MM = 65
_HOSE_MAX_FLOW_LPM = 2650
_HOSE_MAX_PRESSURE_BAR = 35
_WATER_CURTAIN_NOZZLES = 24


class SherpaPlatform(PlatformInterface):
    """HR-셰르파 UGV 플랫폼 — 호스 제약 포함 PlatformInterface 구현체.

    소방 호스를 100m 탑재하고 화재 건물 내부로 진입하는 대형 UGV.
    이동 전 호스 제약(진입 깊이·급커브·충수 후진)을 사전 검증하여
    현장 안전 사고를 예방한다.

    호스 없는 모드 (hose_mode=offline):
      /hose/status 토픽이 없어도 graceful 동작.
      모든 호스 제약 체크가 통과(True)되고, 호스 관련 경고만 출력.

    사용 예시:
      node = rclpy.create_node('sherpa_wrapper')
      sherpa = SherpaPlatform(node, robot_id='sherpa1')
      ok = sherpa.move_to(20.0, 15.0)   # 호스 제약 자동 체크 포함
    """

    def __init__(self, node: Node, robot_id: str = 'sherpa1'):
        """SherpaPlatform 초기화.

        Args:
            node: 이 플랫폼이 귀속될 ROS 2 노드.
            robot_id: 로봇 식별자. 네임스페이스 구성에 사용.
        """
        self._node = node
        self._robot_id = robot_id

        # ── 파라미터 (선언 + 읽기) ────────────────────────────────────────────
        node.declare_parameter(f'{robot_id}.home_x', 0.0)
        node.declare_parameter(f'{robot_id}.home_y', 0.0)
        node.declare_parameter(f'{robot_id}.battery_start', 100.0)
        node.declare_parameter(f'{robot_id}.nav_timeout', 120.0)
        node.declare_parameter(f'{robot_id}.map_frame', 'map')
        node.declare_parameter(f'{robot_id}.base_frame', 'base_footprint')
        node.declare_parameter(f'{robot_id}.hose_total_length', 100.0)
        node.declare_parameter(f'{robot_id}.hose_min_bend_radius', 0.5)
        node.declare_parameter(f'{robot_id}.hose_mode', 'active')

        self._home_x = node.get_parameter(f'{robot_id}.home_x').value
        self._home_y = node.get_parameter(f'{robot_id}.home_y').value
        self._battery = node.get_parameter(f'{robot_id}.battery_start').value
        self._nav_timeout = node.get_parameter(f'{robot_id}.nav_timeout').value
        self._map_frame = node.get_parameter(f'{robot_id}.map_frame').value
        self._base_frame = node.get_parameter(f'{robot_id}.base_frame').value
        self._hose_total_m = node.get_parameter(f'{robot_id}.hose_total_length').value
        self._hose_min_bend_radius = node.get_parameter(
            f'{robot_id}.hose_min_bend_radius').value
        self._hose_mode = node.get_parameter(f'{robot_id}.hose_mode').value

        # ── 내부 상태 ─────────────────────────────────────────────────────────
        self._lock = threading.Lock()
        self._battery_last_update = time.time()
        self._navigating = False
        self._goal_handle: Optional[object] = None

        # ── 호스 상태 (Float32MultiArray data[0..2]) ──────────────────────────
        # offline 모드: 기본값으로 초기화 (제약 없음)
        self.hose_remaining_m: float = self._hose_total_m   # data[0]
        self.hose_kink_risk: float = 0.0                    # data[1] 0=안전~1=위험
        self.hose_charged: bool = False                     # data[2] 1.0=충수

        # 토픽 수신 여부 추적 (offline 모드 감지)
        self._hose_topic_received = False

        # ── TF2 (맵 프레임에서 로봇 위치 조회) ──────────────────────────────
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node)

        # ── Nav2 액션 클라이언트 ──────────────────────────────────────────────
        nav_action = f'{robot_id}/navigate_to_pose'
        self._cb_group = ReentrantCallbackGroup()
        self._nav_client = ActionClient(
            node,
            NavigateToPose,
            nav_action,
            callback_group=self._cb_group,
        )

        # ── 긴급 정지용 cmd_vel 퍼블리셔 ────────────────────────────────────
        # Jazzy: diff_drive_controller는 TwistStamped만 수락 (ros2.md ★★★)
        self._cmd_pub = node.create_publisher(
            TwistStamped,
            f'{robot_id}/cmd_vel',
            10,
        )

        # ── 호스 상태 구독 (/hose/status) ───────────────────────────────────
        # hose_mode=offline이어도 구독은 시도 (나중에 연결될 수 있음)
        self._hose_sub = node.create_subscription(
            Float32MultiArray,
            '/hose/status',
            self._hose_status_callback,
            10,
        )

        # ── 배터리 소비 타이머 (10초 주기) ───────────────────────────────────
        node.create_timer(10.0, self._update_battery)

        node.get_logger().info(
            f'[SherpaPlatform:{robot_id}] 초기화 완료 '
            f'(home=({self._home_x:.1f},{self._home_y:.1f}), '
            f'battery={self._battery:.0f}%, '
            f'hose={self._hose_total_m:.0f}m, '
            f'hose_mode={self._hose_mode}, '
            f'nav_action={nav_action})'
        )

    # ──────────────────────────────────────────────────────────────────────────
    # PlatformInterface 구현
    # ──────────────────────────────────────────────────────────────────────────

    def move_to(self, x: float, y: float, z: float = 0.0) -> bool:
        """호스 제약 사전 체크 후 Nav2 NavigateToPose 액션으로 이동.

        셰르파 특화 이동 흐름:
          1) 배터리 임계값 체크 (10% 미만 거부)
          2) 호스 제약 체크 (진입 깊이 / 급커브 / 충수 후진)
          3) Nav2 서버 연결 확인
          4) Goal 전송

        UGV는 평면 이동이므로 z 파라미터는 무시된다.

        Args:
            x: 목표 x (m, map 프레임 ENU)
            y: 목표 y (m, map 프레임 ENU)
            z: 무시됨 (UGV 평면 이동)

        Returns:
            True: 액션 서버에 Goal 전송 성공
            False: 배터리 부족 / 호스 제약 위반 / 서버 미연결
        """
        # ① 배터리 임계값 (10% 이하 이동 금지 — 귀환 불가 방지)
        if self._battery < 10.0:
            self._node.get_logger().warn(
                f'[{self._robot_id}] 배터리 부족 ({self._battery:.1f}%) — 이동 거부'
            )
            return False

        # ② 호스 제약 사전 체크 (셰르파 핵심 차별점)
        ok, reason = self._check_hose_constraints(x, y)
        if not ok:
            self._node.get_logger().warn(
                f'[{self._robot_id}] 호스 제약 위반 → 이동 거부: {reason}'
            )
            return False

        # ③ Nav2 서버 연결 확인 (논블로킹)
        if not self._nav_client.server_is_ready():
            self._node.get_logger().warn(
                f'[{self._robot_id}] Nav2 서버({self._robot_id}/navigate_to_pose) 미연결'
            )
            return False

        # ④ Goal 메시지 구성
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self._map_frame
        goal.pose.header.stamp = self._node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.w = 1.0  # 도달 후 방향은 Nav2 결정

        self._node.get_logger().info(
            f'[{self._robot_id}] move_to({x:.2f}, {y:.2f}) 전송 '
            f'(호스잔여={self.hose_remaining_m:.1f}m, 충수={self.hose_charged})'
        )

        # 비동기 전송
        future = self._nav_client.send_goal_async(
            goal,
            feedback_callback=self._nav_feedback_callback,
        )
        future.add_done_callback(self._nav_goal_response_callback)

        with self._lock:
            self._navigating = True

        return True

    def return_home(self) -> bool:
        """호스 릴 회수 고려 귀환.

        충수 상태일 때는 호스 릴 회수 경고를 출력하고 이동을 진행한다.
        호스를 즉시 회수할 수 없으므로 경고 로그 후 이동 진행 (운용자 판단).

        Returns:
            True: 귀환 명령 전송 성공
            False: 서버 미연결
        """
        if self._battery < 5.0:
            self._node.get_logger().error(
                f'[{self._robot_id}] 배터리 위험 ({self._battery:.1f}%) — '
                '귀환 시도하지만 도달 불보장'
            )

        # 충수 상태 귀환: 운용자에게 호스 릴 회수 경고
        if self.hose_charged:
            self._node.get_logger().warn(
                f'[{self._robot_id}] 충수 상태 귀환 — '
                '호스 릴 수동 회수 확인 필요 (배관 잔압 주의)'
            )

        # 귀환 시 호스 잔여 길이 내 거리 체크는 생략
        # (귀환 경로는 이미 왔던 경로로 추정 — 제약 우회)
        if not self._nav_client.server_is_ready():
            self._node.get_logger().warn(
                f'[{self._robot_id}] Nav2 서버 미연결 — 귀환 불가'
            )
            return False

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self._map_frame
        goal.pose.header.stamp = self._node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(self._home_x)
        goal.pose.pose.position.y = float(self._home_y)
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.w = 1.0

        self._node.get_logger().info(
            f'[{self._robot_id}] 귀환 → home({self._home_x:.1f}, {self._home_y:.1f})'
        )

        future = self._nav_client.send_goal_async(
            goal,
            feedback_callback=self._nav_feedback_callback,
        )
        future.add_done_callback(self._nav_goal_response_callback)

        with self._lock:
            self._navigating = True

        return True

    def get_pose(self) -> tuple[float, float, float]:
        """TF2로 map → {robot_id}/base_footprint 변환 조회.

        Returns:
            (x, y, z) 튜플 (m, map 프레임 ENU)
            조회 실패 시 (0.0, 0.0, 0.0) 반환
        """
        base_frame = self._base_frame
        if self._robot_id and not base_frame.startswith(self._robot_id):
            base_frame = f'{self._robot_id}/{self._base_frame}'

        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame,
                base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
            t = tf.transform.translation
            return (t.x, t.y, t.z)
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self._node.get_logger().debug(
                f'[{self._robot_id}] TF 조회 실패 ({base_frame}): {e}',
                throttle_duration_sec=5.0,
            )
            return (0.0, 0.0, 0.0)

    def get_battery(self) -> float:
        """배터리 잔량 반환.

        셰르파 배터리 모델:
          20kWh / 5시간(5kph 정속) = 4kW 소비
          drain_rate ≈ 0.333 %/분 (이동 시)
          idle_factor = 0.25 (정지 시 대기 전력)

        Returns:
            배터리 잔량 (%, 0.0 ~ 100.0)
        """
        with self._lock:
            return max(0.0, round(self._battery, 1))

    def get_capabilities(self) -> RobotCapabilities:
        """HR-셰르파 능력 명세 반환.

        셰르파 특화 extra 필드:
          - hose_length_m: 호스 총 길이 (m)
          - hose_diameter_mm: 호스 구경 (mm)
          - max_flow_lpm: 최대 방수량 (L/min)
          - max_pressure_bar: 최대 압력 (bar)
          - water_curtain_nozzles: 수막 노즐 수
          - thermal_dual: SWIR + LWIR 이중 열화상
          - weight_kg: 차량 중량 (kg)
          - wheel_count: 바퀴 수 (6륜 구동)

        Returns:
            셰르파 능력 명세 (battery_capacity는 현재 잔량 반영)
        """
        return RobotCapabilities(
            can_drive=True,
            can_fly=False,
            has_thermal=True,
            has_lidar=True,
            max_speed=1.4,                       # 5kph = 1.389 m/s
            battery_capacity=self._battery,
            platform_type='sherpa',
            extra={
                'hose_length_m': self._hose_total_m,
                'hose_diameter_mm': _HOSE_DIAMETER_MM,
                'max_flow_lpm': _HOSE_MAX_FLOW_LPM,
                'max_pressure_bar': _HOSE_MAX_PRESSURE_BAR,
                'water_curtain_nozzles': _WATER_CURTAIN_NOZZLES,
                'thermal_dual': True,            # SWIR + LWIR
                'weight_kg': 2000,
                'wheel_count': 6,
                'hose_remaining_m': self.hose_remaining_m,
                'hose_charged': self.hose_charged,
            }
        )

    def emergency_stop(self) -> None:
        """긴급 정지 — Nav2 취소 + cmd_vel=0 즉시 발행.

        소방 현장 긴급 정지 시 우선순위:
          1) Nav2 Goal 취소 (경로 계획 중단)
          2) cmd_vel TwistStamped 제로 발행 (즉시 정지)

        충수 상태라도 호스 손상 위험보다 인명 보호 우선.
        예외 발생 시에도 계속 진행 (오케스트레이터 루프 보호).
        """
        self._node.get_logger().error(
            f'[{self._robot_id}] EMERGENCY STOP — 전체 이동 중단'
            + (' (충수 상태 — 호스 압력 주의)' if self.hose_charged else '')
        )

        # 채널 1: Nav2 현재 Goal 취소
        try:
            with self._lock:
                handle = self._goal_handle
            if handle is not None:
                handle.cancel_goal_async()
        except Exception as e:
            self._node.get_logger().error(
                f'[{self._robot_id}] Nav2 취소 실패: {e}'
            )

        # 채널 2: cmd_vel=0 직접 발행 (TwistStamped — Jazzy 표준)
        try:
            stop_msg = TwistStamped()
            stop_msg.header.stamp = self._node.get_clock().now().to_msg()
            stop_msg.header.frame_id = 'base_link'
            stop_msg.twist.linear.x = 0.0
            stop_msg.twist.linear.y = 0.0
            stop_msg.twist.linear.z = 0.0
            stop_msg.twist.angular.x = 0.0
            stop_msg.twist.angular.y = 0.0
            stop_msg.twist.angular.z = 0.0
            self._cmd_pub.publish(stop_msg)
        except Exception as e:
            self._node.get_logger().error(
                f'[{self._robot_id}] cmd_vel 제로 발행 실패: {e}'
            )

        with self._lock:
            self._navigating = False

    # ──────────────────────────────────────────────────────────────────────────
    # 호스 제약 체크 (셰르파 핵심 차별점)
    # ──────────────────────────────────────────────────────────────────────────

    def _check_hose_constraints(
        self, target_x: float, target_y: float
    ) -> tuple[bool, str]:
        """호스 제약 검증.

        offline 모드 또는 토픽 미수신 시: 모든 체크 통과 (graceful 동작).
        active 모드 + 토픽 수신 시: 3가지 제약 순서대로 검증.

        제약 1 — 진입 깊이: 목표까지 직선 거리가 호스 잔여 길이를 초과하는가?
          현재 위치에서 목표까지 유클리드 거리로 추정.
          실제 경로는 Nav2가 생성하지만 사전에 직선 거리로 보수적 추정.

        제약 2 — 급커브: 꺾임 위험도가 임계값(0.7) 이상인가?
          /hose/status data[1] 값 기반 판단.

        제약 3 — 충수 후진: 충수 상태에서 목표가 현재 위치보다 홈 방향인가?
          홈 쪽으로 이동 = 후진으로 간주 (호스 배관 손상 위험).

        Args:
            target_x: 목표 x 좌표 (m, map 프레임)
            target_y: 목표 y 좌표 (m, map 프레임)

        Returns:
            (통과 여부, 거부 사유 문자열)
            통과 시: (True, "")
            거부 시: (False, 사유)
        """
        # offline 모드 또는 토픽 미수신: 제약 없이 통과
        if self._hose_mode == 'offline' or not self._hose_topic_received:
            if self._hose_mode == 'active' and not self._hose_topic_received:
                self._node.get_logger().warn(
                    f'[{self._robot_id}] /hose/status 미수신 — 호스 제약 우회 (offline 간주)',
                    throttle_duration_sec=30.0,
                )
            return True, ''

        # 현재 위치 조회
        cur_x, cur_y, _ = self.get_pose()

        # ── 제약 1: 진입 깊이 (호스 잔여 길이 초과?) ─────────────────────────
        depth = self._calc_distance(cur_x, cur_y, target_x, target_y)
        if depth > self.hose_remaining_m:
            return (
                False,
                f'진입 깊이 {depth:.1f}m > 호스 잔여 {self.hose_remaining_m:.1f}m'
            )

        # ── 제약 2: 급커브 (꺾임 위험도 임계값 0.7 이상?) ────────────────────
        if self._would_cause_kink(target_x, target_y):
            return False, f'급커브 — 호스 꺾임 위험 (risk={self.hose_kink_risk:.2f})'

        # ── 제약 3: 충수 상태 후진 금지 ─────────────────────────────────────
        if self.hose_charged and self._requires_reverse(cur_x, cur_y, target_x, target_y):
            return False, '충수 상태 후진 금지 — 배관 손상 위험'

        return True, ''

    def _calc_distance(
        self, x1: float, y1: float, x2: float, y2: float
    ) -> float:
        """두 좌표 사이 유클리드 거리 계산 (m)."""
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def _calc_distance_to(self, target_x: float, target_y: float) -> float:
        """현재 위치에서 목표까지 거리 계산 (m)."""
        cur_x, cur_y, _ = self.get_pose()
        return self._calc_distance(cur_x, cur_y, target_x, target_y)

    def _would_cause_kink(self, target_x: float, target_y: float) -> bool:
        """목표 방향이 급커브(호스 꺾임)를 유발하는지 판단.

        현재 구현: /hose/status 토픽의 hose_kink_risk 임계값 기반 판단.
          - hose_kink_risk ≥ 0.7 → 급커브 위험

        향후 확장 가능:
          - Nav2 경로 피드백의 곡률 분석
          - 전방 LiDAR 기반 통로 폭 판단

        기하학적 U턴(홈→현재→목표 각도 계산)은 사용하지 않는다.
        이유: 직선 후퇴도 180도로 잘못 감지되어 오탐 발생.
        U턴 시 실제 호스 꺾임은 /hose/status 토픽이 감지한다.

        Returns:
            True: 급커브 위험 (토픽 기반)
            False: 안전
        """
        _KINK_RISK_THRESHOLD = 0.7
        return self.hose_kink_risk >= _KINK_RISK_THRESHOLD

    def _requires_reverse(
        self,
        cur_x: float, cur_y: float,
        target_x: float, target_y: float,
    ) -> bool:
        """목표 이동이 실질적으로 후진(홈 방향)인지 판단.

        판단 기준: 목표가 현재보다 홈에 더 가까워지면 후진으로 간주.
        충수 상태에서 후진은 배관 손상 위험이 있다.

        Returns:
            True: 후진 (홈 방향 이동)
            False: 전진 (화재 현장 방향 이동) 또는 횡방향
        """
        dist_cur_to_home = self._calc_distance(cur_x, cur_y, self._home_x, self._home_y)
        dist_target_to_home = self._calc_distance(
            target_x, target_y, self._home_x, self._home_y
        )
        # 목표가 홈에 더 가까워지면 후진
        return dist_target_to_home < dist_cur_to_home

    # ──────────────────────────────────────────────────────────────────────────
    # 호스 상태 토픽 콜백
    # ──────────────────────────────────────────────────────────────────────────

    def _hose_status_callback(self, msg: Float32MultiArray) -> None:
        """/hose/status Float32MultiArray 수신 처리.

        data[0]: 호스 잔여 길이 (m)
        data[1]: 꺾임 위험도 (0.0~1.0)
        data[2]: 충수 여부 (0.0=건수, 1.0=충수)

        데이터 부족 시 graceful 처리 (기존 값 유지).
        """
        data = msg.data
        if len(data) < 1:
            self._node.get_logger().warn(
                f'[{self._robot_id}] /hose/status 데이터 부족 (len={len(data)})'
            )
            return

        self._hose_topic_received = True

        # data[0]: 호스 잔여 길이
        self.hose_remaining_m = float(data[0])

        # data[1]: 꺾임 위험도 (선택 필드)
        if len(data) >= 2:
            self.hose_kink_risk = float(data[1])

        # data[2]: 충수 여부 (선택 필드)
        if len(data) >= 3:
            self.hose_charged = float(data[2]) >= 0.5

        self._node.get_logger().debug(
            f'[{self._robot_id}] 호스 상태 갱신: '
            f'잔여={self.hose_remaining_m:.1f}m, '
            f'꺾임위험={self.hose_kink_risk:.2f}, '
            f'충수={self.hose_charged}'
        )

        # 잔여 호스 20m 미만 경고
        if self.hose_remaining_m < 20.0:
            self._node.get_logger().warn(
                f'[{self._robot_id}] 호스 잔여량 부족: {self.hose_remaining_m:.1f}m',
                throttle_duration_sec=30.0,
            )

    # ──────────────────────────────────────────────────────────────────────────
    # Nav2 콜백
    # ──────────────────────────────────────────────────────────────────────────

    def _nav_goal_response_callback(self, future) -> None:
        """Nav2 Goal 수락/거절 응답 처리."""
        try:
            handle = future.result()
        except Exception as e:
            self._node.get_logger().error(
                f'[{self._robot_id}] Goal 응답 오류: {e}'
            )
            with self._lock:
                self._navigating = False
            return

        if not handle.accepted:
            self._node.get_logger().warn(
                f'[{self._robot_id}] Nav2 Goal 거절됨'
            )
            with self._lock:
                self._navigating = False
            return

        self._node.get_logger().info(f'[{self._robot_id}] Nav2 Goal 수락됨')
        with self._lock:
            self._goal_handle = handle

        result_future = handle.get_result_async()
        result_future.add_done_callback(self._nav_result_callback)

    def _nav_feedback_callback(self, feedback_msg) -> None:
        """Nav2 피드백 수신 — 현재 위치 + 호스 잔여 로그 (debug 레벨)."""
        fb = feedback_msg.feedback
        pose = fb.current_pose.pose.position
        self._node.get_logger().debug(
            f'[{self._robot_id}] Nav2 진행 중: '
            f'({pose.x:.2f}, {pose.y:.2f}), '
            f'호스잔여={self.hose_remaining_m:.1f}m',
            throttle_duration_sec=3.0,
        )

    def _nav_result_callback(self, future) -> None:
        """Nav2 결과 수신 — 이동 완료 또는 실패 처리."""
        try:
            result = future.result()
        except Exception as e:
            self._node.get_logger().error(
                f'[{self._robot_id}] Nav2 결과 수신 오류: {e}'
            )
            with self._lock:
                self._navigating = False
            return

        from action_msgs.msg import GoalStatus
        status = result.status
        with self._lock:
            self._navigating = False
            self._goal_handle = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            self._node.get_logger().info(f'[{self._robot_id}] 목표 도달 성공')
        else:
            self._node.get_logger().warn(
                f'[{self._robot_id}] Nav2 이동 실패 (status={status})'
            )

    # ──────────────────────────────────────────────────────────────────────────
    # 배터리 시뮬레이션
    # ──────────────────────────────────────────────────────────────────────────

    def _update_battery(self) -> None:
        """배터리 소비 시뮬레이션 (10초 주기 타이머 콜백).

        셰르파 모델: 20kWh / 5시간 = drain_rate ≈ 0.333 %/분
          이동 중: drain_rate × 1.0
          정지 중: drain_rate × 0.25 (대기 전력, 유압 시스템 포함)
        """
        now = time.time()
        elapsed_min = (now - self._battery_last_update) / 60.0

        with self._lock:
            drain_factor = 1.0 if self._navigating else _SHERPA_IDLE_FACTOR
            drain = _SHERPA_DRAIN_RATE_PCT_PER_MIN * elapsed_min * drain_factor
            self._battery = max(0.0, self._battery - drain)
            self._battery_last_update = now

        if self._battery < 20.0:
            self._node.get_logger().warn(
                f'[{self._robot_id}] 배터리 부족 경고: {self._battery:.1f}%',
                throttle_duration_sec=30.0,
            )


def main(args=None):
    """SherpaPlatform 단독 실행 테스트 노드."""
    rclpy.init(args=args)
    node = rclpy.create_node('sherpa_platform_test')
    sherpa = SherpaPlatform(node, robot_id='sherpa1')

    node.get_logger().info('SherpaPlatform 테스트 노드 시작')
    node.get_logger().info(f'capabilities: {sherpa.get_capabilities()}')
    node.get_logger().info(f'battery: {sherpa.get_battery()}%')
    node.get_logger().info(f'pose: {sherpa.get_pose()}')

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
