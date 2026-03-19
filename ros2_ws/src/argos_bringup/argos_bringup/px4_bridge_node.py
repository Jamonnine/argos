#!/usr/bin/env python3
"""px4_bridge_node.py — ARGOS ↔ PX4 Offboard 브릿지

드론 전문가 권고: 현재 Gazebo 네이티브 드론 컨트롤러를
PX4 offboard 모드로 전환하기 위한 브릿지 노드.

아키텍처:
  ARGOS cmd_vel (TwistStamped, Jazzy) → PX4 Bridge → TrajectorySetpoint (PX4)
  PX4 VehicleOdometry → PX4 Bridge → ARGOS odom (Odometry)

참조: px4-offboard (ETH Zurich), aerial-autonomy-stack
리서치: knowledge/projects/research/2026-03-15-px4-ros2-drone-integration-deep-dive.md

주요 변환:
  - ENU (ROS2) ↔ NED (PX4) 좌표계 변환
  - Twist → TrajectorySetpoint (위치/속도 명령)
  - OffboardControlMode 2Hz 이상 발행 필수 (arm 전 100회 선발행 필수)

현재 상태: px4_msgs 설치 시 완전 동작, 미설치 시 passthrough 모드
PX4 SITL 설치 후 활성화: launch 파라미터 use_px4:=true
멀티드론: px4_instance 파라미터로 /px4_N/ 네임스페이스 자동 적용

Offboard 모드 진입 시퀀스:
  1. OffboardControlMode + TrajectorySetpoint 100회 이상 선발행 (offboard_preflight_count)
  2. set_offboard_mode() 호출 → PX4가 offboard 전환 수락
  3. arm() 호출
  4. 웨이포인트 순회 시작

서비스:
  ~/arm          (std_srvs/srv/Trigger) — 드론 ARM
  ~/disarm       (std_srvs/srv/Trigger) — 드론 DISARM
  ~/takeoff      (std_srvs/srv/Trigger) — 이륙 (첫 웨이포인트 z로 상승)
  ~/land         (std_srvs/srv/Trigger) — 현재 위치에서 착륙
  ~/goto         (argos_interfaces/srv/Goto or SetBool fallback)
  ~/start_mission (std_srvs/srv/Trigger) — 웨이포인트 자동 순회 시작
  ~/stop_mission  (std_srvs/srv/Trigger) — 미션 정지 (호버링 유지)
"""
import math
from enum import IntEnum
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger

# px4_msgs 미설치 환경에서도 노드가 구동되도록 graceful fallback
try:
    from px4_msgs.msg import (
        OffboardControlMode,
        TrajectorySetpoint,
        VehicleCommand,
        VehicleLocalPosition,
        VehicleStatus,
    )
    PX4_MSGS_AVAILABLE = True
except ImportError:
    PX4_MSGS_AVAILABLE = False

# argos_interfaces Goto 서비스 — 미설치 시 Trigger로 대체
try:
    from argos_interfaces.srv import Goto as GotoSrv
    GOTO_SRV_AVAILABLE = True
except ImportError:
    GOTO_SRV_AVAILABLE = False


class FlightState(IntEnum):
    """드론 비행 상태 머신."""
    IDLE = 0           # 미션 없음 (지상 또는 호버링)
    PREFLIGHT = 1      # offboard 선발행 중 (arm 전 필수)
    ARMING = 2         # arm 명령 전송 후 확인 대기
    TAKEOFF = 3        # 이륙 중 (첫 웨이포인트 z 고도까지)
    WAYPOINT = 4       # 웨이포인트 순회 중
    HOVER = 5          # 미션 일시 정지 (호버링)
    LANDING = 6        # 착륙 중


class PX4BridgeNode(Node):
    """ARGOS cmd_vel ↔ PX4 offboard 브릿지.

    PX4 미설치 시 passthrough 모드 (변환 없이 토픽 릴레이).
    PX4 설치 시 offboard 모드 활성화 + 웨이포인트 자동 순회.

    멀티드론 사용 시 px4_instance 파라미터로 인스턴스 지정:
      px4_instance=0 → /fmu/in/... (기본 네임스페이스 없음)
      px4_instance=1 → /px4_1/fmu/in/...
      px4_instance=N → /px4_N/fmu/in/...
    """

    # offboard 모드 진입 전 최소 선발행 횟수 (PX4 권고: ≥100회)
    OFFBOARD_PREFLIGHT_COUNT = 110

    # 웨이포인트 도달 허용 오차 (미터)
    WAYPOINT_REACH_RADIUS = 0.5

    def __init__(self):
        super().__init__('px4_bridge')

        # ── 기본 파라미터 ────────────────────────────────────────────────────
        self.declare_parameter('use_px4', False)
        self.declare_parameter('robot_id', 'drone1')
        self.declare_parameter('offboard_rate', 10.0)    # Hz (최소 2Hz)
        self.declare_parameter('px4_instance', 0)

        # 웨이포인트 파라미터: ENU [x,y,z] 리스트. 기본값: 3m 사각형
        # ROS2 파라미터는 중첩 리스트를 지원하지 않으므로 평탄화 후 3개씩 분할
        self.declare_parameter(
            'waypoints',
            [2.0, 0.0, 3.0,
             2.0, 2.0, 3.0,
             0.0, 2.0, 3.0,
             0.0, 0.0, 3.0])  # ENU (x, y, z) × 4점

        # 지오펜스 파라미터 (ENU 좌표 기준, 미터)
        self.declare_parameter('geofence_x_min', -20.0)
        self.declare_parameter('geofence_x_max',  20.0)
        self.declare_parameter('geofence_y_min', -20.0)
        self.declare_parameter('geofence_y_max',  20.0)
        self.declare_parameter('geofence_z_max',  10.0)  # 최대 고도

        # ── 파라미터 읽기 ─────────────────────────────────────────────────────
        self.use_px4 = self.get_parameter('use_px4').value
        self.robot_id = self.get_parameter('robot_id').value
        self.px4_instance = self.get_parameter('px4_instance').value

        raw_wp = self.get_parameter('waypoints').value
        self._waypoints: List[Tuple[float, float, float]] = self._parse_waypoints(raw_wp)

        self._geofence = {
            'x_min': self.get_parameter('geofence_x_min').value,
            'x_max': self.get_parameter('geofence_x_max').value,
            'y_min': self.get_parameter('geofence_y_min').value,
            'y_max': self.get_parameter('geofence_y_max').value,
            'z_max': self.get_parameter('geofence_z_max').value,
        }

        # 멀티드론 네임스페이스: instance=0이면 접두사 없음, 1이상이면 /px4_N/
        self._fmu_ns = f'/px4_{self.px4_instance}' if self.px4_instance > 0 else ''

        # ── 상태 변수 ────────────────────────────────────────────────────────
        self._vehicle_armed = False
        self._nav_state = 0              # VehicleStatus.nav_state
        self._flight_state = FlightState.IDLE

        # 현재 ENU 위치 (VehicleLocalPosition → NED→ENU 변환)
        self._pos_enu: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._vel_enu: Tuple[float, float, float] = (0.0, 0.0, 0.0)

        # offboard 선발행 카운터 (arm 전 OFFBOARD_PREFLIGHT_COUNT 이상 필요)
        self._preflight_count = 0

        # 웨이포인트 인덱스
        self._wp_index = 0

        # 현재 목표 위치 (ENU). IDLE 진입 시 마지막 위치로 호버링
        self._target_enu: Optional[Tuple[float, float, float]] = None

        # ── 초기화 분기 ──────────────────────────────────────────────────────
        if self.use_px4:
            self._init_px4_mode()
        else:
            self._init_passthrough_mode()

        self.get_logger().info(
            f'PX4 Bridge initialized (use_px4={self.use_px4}, '
            f'robot={self.robot_id}, instance={self.px4_instance}, '
            f'px4_msgs={PX4_MSGS_AVAILABLE})')

        if self._waypoints:
            wp_str = ', '.join(f'({x:.1f},{y:.1f},{z:.1f})' for x, y, z in self._waypoints)
            self.get_logger().info(f'Waypoints (ENU): [{wp_str}]')

        gf = self._geofence
        self.get_logger().info(
            f'Geofence (ENU): x[{gf["x_min"]},{gf["x_max"]}] '
            f'y[{gf["y_min"]},{gf["y_max"]}] z_max={gf["z_max"]}')

    # ──────────────────────────────────────────────────────────────────────────
    # 초기화
    # ──────────────────────────────────────────────────────────────────────────

    def _init_passthrough_mode(self):
        """PX4 미사용 — cmd_vel 그대로 전달 (현재 Gazebo 네이티브)."""
        self.get_logger().info('Passthrough mode (no PX4)')

    def _init_px4_mode(self):
        """PX4 offboard 모드 초기화.

        필수 조건:
          1. PX4 SITL 기동 (gz sim + PX4)
          2. Micro XRCE-DDS Agent 실행: MicroXRCEAgent udp4 -p 8888
          3. /fmu/in/offboard_control_mode 2Hz 이상 발행
        """
        if not PX4_MSGS_AVAILABLE:
            self.get_logger().error(
                'px4_msgs not installed — PX4 offboard mode unavailable. '
                'Install: git clone https://github.com/PX4/px4_msgs.git '
                'into your workspace and rebuild.')
            return

        self.get_logger().info('PX4 offboard mode initializing...')

        ns = self._fmu_ns

        # ── Publishers: ROS2 → PX4 ──────────────────────────────────────────
        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            f'{ns}/fmu/in/offboard_control_mode',
            10)

        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            f'{ns}/fmu/in/trajectory_setpoint',
            10)

        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            f'{ns}/fmu/in/vehicle_command',
            10)

        # ── Subscribers: PX4 → ROS2 ─────────────────────────────────────────
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            f'{ns}/fmu/out/vehicle_status',
            self._vehicle_status_callback,
            10)

        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            f'{ns}/fmu/out/vehicle_local_position',
            self._local_position_callback,
            10)

        # ── ARGOS 입력 구독 ──────────────────────────────────────────────────
        # Jazzy: diff_drive_controller가 TwistStamped만 수락 (ros2.md ★★★)
        # cmd_vel은 미션 비활성 시(IDLE/HOVER) 속도 수동 제어에 사용
        self.cmd_sub = self.create_subscription(
            TwistStamped, 'cmd_vel', self._cmd_vel_to_px4, 10)

        # ── 서비스: 외부 제어 ────────────────────────────────────────────────
        self._create_services()

        # ── Offboard heartbeat 타이머 ─────────────────────────────────────────
        # PX4는 OffboardControlMode를 2Hz 이상 수신해야 offboard 유지
        rate = self.get_parameter('offboard_rate').value
        self.offboard_timer = self.create_timer(
            1.0 / rate, self._offboard_control_loop)

        self.get_logger().warn(
            f'PX4 offboard mode ready — topics: {ns}/fmu/in/*, {ns}/fmu/out/*. '
            'Requires PX4 SITL + MicroXRCEAgent udp4 -p 8888')

    def _create_services(self):
        """드론 제어 서비스 등록."""
        self.srv_arm = self.create_service(
            Trigger, '~/arm', self._svc_arm)
        self.srv_disarm = self.create_service(
            Trigger, '~/disarm', self._svc_disarm)
        self.srv_takeoff = self.create_service(
            Trigger, '~/takeoff', self._svc_takeoff)
        self.srv_land = self.create_service(
            Trigger, '~/land', self._svc_land)
        self.srv_start = self.create_service(
            Trigger, '~/start_mission', self._svc_start_mission)
        self.srv_stop = self.create_service(
            Trigger, '~/stop_mission', self._svc_stop_mission)

        if GOTO_SRV_AVAILABLE:
            self.srv_goto = self.create_service(
                GotoSrv, '~/goto', self._svc_goto)
            self.get_logger().info('~/goto service ready (argos_interfaces/Goto)')
        else:
            self.get_logger().warn(
                '~/goto service unavailable — argos_interfaces.srv.Goto not found. '
                'Use start_mission/stop_mission instead.')

    # ──────────────────────────────────────────────────────────────────────────
    # 유틸리티
    # ──────────────────────────────────────────────────────────────────────────

    @staticmethod
    def _parse_waypoints(flat: list) -> List[Tuple[float, float, float]]:
        """평탄화된 [x0,y0,z0, x1,y1,z1, ...] → [(x0,y0,z0), ...] 변환."""
        if len(flat) % 3 != 0:
            raise ValueError(
                f'waypoints 파라미터 길이({len(flat)})가 3의 배수가 아닙니다.')
        return [(float(flat[i]), float(flat[i + 1]), float(flat[i + 2]))
                for i in range(0, len(flat), 3)]

    @staticmethod
    def enu_to_ned(x_enu: float, y_enu: float, z_enu: float
                  ) -> Tuple[float, float, float]:
        """ENU (ROS2) → NED (PX4) 좌표 변환.

        ENU: x=East, y=North, z=Up
        NED: x=North, y=East, z=Down
        """
        return y_enu, x_enu, -z_enu   # (North, East, Down)

    @staticmethod
    def ned_to_enu(x_ned: float, y_ned: float, z_ned: float
                  ) -> Tuple[float, float, float]:
        """NED (PX4) → ENU (ROS2) 좌표 변환."""
        return y_ned, x_ned, -z_ned   # (East, North, Up)

    def _distance_to_target(self) -> float:
        """현재 ENU 위치에서 _target_enu까지의 3D 거리."""
        if self._target_enu is None:
            return float('inf')
        dx = self._pos_enu[0] - self._target_enu[0]
        dy = self._pos_enu[1] - self._target_enu[1]
        dz = self._pos_enu[2] - self._target_enu[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _check_geofence(self, x_enu: float, y_enu: float, z_enu: float) -> bool:
        """목표 좌표가 지오펜스 내부인지 검증. 위반 시 False."""
        gf = self._geofence
        if not (gf['x_min'] <= x_enu <= gf['x_max']):
            self.get_logger().warn(
                f'Geofence violation: x={x_enu:.2f} out of [{gf["x_min"]},{gf["x_max"]}]')
            return False
        if not (gf['y_min'] <= y_enu <= gf['y_max']):
            self.get_logger().warn(
                f'Geofence violation: y={y_enu:.2f} out of [{gf["y_min"]},{gf["y_max"]}]')
            return False
        if z_enu > gf['z_max']:
            self.get_logger().warn(
                f'Geofence violation: z={z_enu:.2f} > z_max={gf["z_max"]}')
            return False
        if z_enu < 0.0:
            self.get_logger().warn(f'Geofence violation: z={z_enu:.2f} < 0')
            return False
        return True

    def _now_us(self) -> int:
        """현재 ROS clock을 마이크로초(us)로 반환 — PX4 timestamp 필드용."""
        return self.get_clock().now().nanoseconds // 1000

    # ──────────────────────────────────────────────────────────────────────────
    # 구독 콜백
    # ──────────────────────────────────────────────────────────────────────────

    def _vehicle_status_callback(self, msg: 'VehicleStatus'):
        """PX4 VehicleStatus 수신 — arming 및 nav_state 모니터링."""
        prev_armed = self._vehicle_armed
        # arming_state: 1=STANDBY, 2=ARMED (px4_msgs 상수)
        self._vehicle_armed = (msg.arming_state == 2)
        self._nav_state = msg.nav_state

        if prev_armed != self._vehicle_armed:
            state_str = 'ARMED' if self._vehicle_armed else 'DISARMED'
            self.get_logger().info(
                f'[{self.robot_id}] Arming state → {state_str} '
                f'(nav_state={self._nav_state}, flight_state={self._flight_state.name})')

            # arm 확인: ARMING 상태에서 실제 armed되면 TAKEOFF로 전환
            if self._vehicle_armed and self._flight_state == FlightState.ARMING:
                self._start_takeoff()

    def _local_position_callback(self, msg: 'VehicleLocalPosition'):
        """PX4 VehicleLocalPosition 수신 — 현재 위치(NED→ENU 변환) 갱신."""
        # PX4 local position은 NED 좌표계
        self._pos_enu = self.ned_to_enu(msg.x, msg.y, msg.z)
        self._vel_enu = self.ned_to_enu(msg.vx, msg.vy, msg.vz)

    def _cmd_vel_to_px4(self, msg: TwistStamped):
        """ARGOS TwistStamped → PX4 TrajectorySetpoint 변환 (속도 제어 모드).

        Jazzy: cmd_vel 타입은 TwistStamped (ros2.md ★★★).
        필드 접근: msg.twist.linear.x (Twist와 다름).

        IDLE 또는 HOVER 상태에서만 수동 cmd_vel 적용.
        미션 중(WAYPOINT, TAKEOFF)에는 무시.
        """
        if not PX4_MSGS_AVAILABLE:
            return
        if self._flight_state not in (FlightState.IDLE, FlightState.HOVER):
            return

        # TwistStamped → twist 필드 접근 후 ENU → NED 변환
        vx_ned, vy_ned, vz_ned = self.enu_to_ned(
            msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z)
        yaw_rate = msg.twist.angular.z

        setpoint = TrajectorySetpoint()
        setpoint.position = [float('nan'), float('nan'), float('nan')]
        setpoint.velocity = [vx_ned, vy_ned, vz_ned]
        setpoint.yawspeed = -yaw_rate  # ENU yaw → NED yaw (부호 반전)
        setpoint.yaw = float('nan')
        setpoint.timestamp = self._now_us()
        self.trajectory_pub.publish(setpoint)

        self.get_logger().debug(
            f'PX4 cmd_vel: vx={vx_ned:.2f} vy={vy_ned:.2f} vz={vz_ned:.2f} '
            f'yaw_rate={-yaw_rate:.2f}',
            throttle_duration_sec=1.0)

    # ──────────────────────────────────────────────────────────────────────────
    # 메인 제어 루프 (offboard heartbeat + 상태 머신)
    # ──────────────────────────────────────────────────────────────────────────

    def _offboard_control_loop(self):
        """offboard heartbeat + 비행 상태 머신.

        이 타이머가 PX4의 OffboardControlMode keepalive를 담당.
        PX4는 2Hz 이상 수신 없으면 failsafe 전환.

        PREFLIGHT 단계에서 OFFBOARD_PREFLIGHT_COUNT 회 발행 후
        자동으로 offboard 모드 전환 + arm 명령 전송.
        """
        if not PX4_MSGS_AVAILABLE:
            return

        # 1) OffboardControlMode 항상 발행 (상태 무관)
        self._publish_offboard_control_mode()

        # 2) 상태별 setpoint 발행 + 전환 로직
        state = self._flight_state

        if state == FlightState.IDLE:
            # IDLE: 목표 없음 — 호버링 setpoint로 위치 유지
            if self._target_enu is not None:
                self._publish_position_setpoint(*self._target_enu)

        elif state == FlightState.PREFLIGHT:
            # PREFLIGHT: 현재 위치 0고도 setpoint 발행 (드론이 지상에 있으므로 z=0.5m)
            hold_x, hold_y, _ = self._pos_enu
            self._publish_position_setpoint(hold_x, hold_y, 0.5)

            self._preflight_count += 1
            if self._preflight_count >= self.OFFBOARD_PREFLIGHT_COUNT:
                self.get_logger().info(
                    f'[{self.robot_id}] Preflight complete ({self._preflight_count} msgs). '
                    'Switching to OFFBOARD mode + ARM...')
                self.set_offboard_mode()
                self._flight_state = FlightState.ARMING
                self.arm()

        elif state == FlightState.ARMING:
            # ARMING: arm 명령 전송 중 — 현재 위치 유지
            hold_x, hold_y, _ = self._pos_enu
            self._publish_position_setpoint(hold_x, hold_y, 0.5)
            # arm 확인은 _vehicle_status_callback에서 처리

        elif state == FlightState.TAKEOFF:
            if self._target_enu is not None:
                self._publish_position_setpoint(*self._target_enu)
                dist = self._distance_to_target()
                if dist < self.WAYPOINT_REACH_RADIUS:
                    self.get_logger().info(
                        f'[{self.robot_id}] Takeoff complete (dist={dist:.2f}m). '
                        'Starting waypoint mission...')
                    self._wp_index = 0
                    self._flight_state = FlightState.WAYPOINT
                    self._set_current_waypoint()

        elif state == FlightState.WAYPOINT:
            if self._target_enu is not None:
                self._publish_position_setpoint(*self._target_enu)
                dist = self._distance_to_target()
                if dist < self.WAYPOINT_REACH_RADIUS:
                    self._advance_waypoint()

        elif state == FlightState.HOVER:
            # HOVER: _target_enu 유지 (stop_mission 후 호버링)
            if self._target_enu is not None:
                self._publish_position_setpoint(*self._target_enu)

        elif state == FlightState.LANDING:
            # 착륙: 현재 x,y에서 z=0으로 하강
            if self._target_enu is not None:
                self._publish_position_setpoint(*self._target_enu)
                # 착륙 완료 판단: z < 0.2m 이면 disarm
                if self._pos_enu[2] < 0.2:
                    self.get_logger().info(
                        f'[{self.robot_id}] Landing complete. Disarming...')
                    self.disarm()
                    self._flight_state = FlightState.IDLE
                    self._target_enu = None

    def _publish_offboard_control_mode(self):
        """OffboardControlMode heartbeat 발행.

        position=True: 위치 제어 우선.
        속도 제어만 필요 시 velocity=True, position=False로 변경.
        """
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self._now_us()
        self.offboard_pub.publish(msg)

    def _publish_position_setpoint(self, x_enu: float, y_enu: float, z_enu: float,
                                   yaw_enu: float = 0.0):
        """ENU 위치 setpoint 발행 (NED 변환 포함)."""
        if not PX4_MSGS_AVAILABLE:
            return

        x_ned, y_ned, z_ned = self.enu_to_ned(x_enu, y_enu, z_enu)
        # ENU yaw (CCW from East) → NED yaw (CW from North): π/2 - yaw_enu
        yaw_ned = math.pi / 2.0 - yaw_enu

        setpoint = TrajectorySetpoint()
        setpoint.position = [x_ned, y_ned, z_ned]
        setpoint.velocity = [float('nan'), float('nan'), float('nan')]
        setpoint.yaw = yaw_ned
        setpoint.timestamp = self._now_us()
        self.trajectory_pub.publish(setpoint)

    # ──────────────────────────────────────────────────────────────────────────
    # 웨이포인트 관리
    # ──────────────────────────────────────────────────────────────────────────

    def _set_current_waypoint(self):
        """현재 웨이포인트(_wp_index)를 목표로 설정."""
        if not self._waypoints:
            self.get_logger().warn('웨이포인트 목록이 비어있습니다.')
            self._flight_state = FlightState.HOVER
            return

        wp = self._waypoints[self._wp_index]
        if not self._check_geofence(*wp):
            self.get_logger().error(
                f'Waypoint #{self._wp_index} {wp} 지오펜스 위반. '
                'HOVER 모드로 전환.')
            self._flight_state = FlightState.HOVER
            return

        self._target_enu = wp
        self.get_logger().info(
            f'[{self.robot_id}] Waypoint {self._wp_index + 1}/{len(self._waypoints)}: '
            f'ENU({wp[0]:.1f}, {wp[1]:.1f}, {wp[2]:.1f})')

    def _advance_waypoint(self):
        """다음 웨이포인트로 이동 (순환)."""
        wp = self._waypoints[self._wp_index]
        self.get_logger().info(
            f'[{self.robot_id}] Waypoint {self._wp_index + 1} 도달 '
            f'ENU({wp[0]:.1f},{wp[1]:.1f},{wp[2]:.1f})')

        self._wp_index = (self._wp_index + 1) % len(self._waypoints)

        if self._wp_index == 0:
            self.get_logger().info(
                f'[{self.robot_id}] 웨이포인트 1순환 완료. 반복 중...')

        self._set_current_waypoint()

    def _start_takeoff(self):
        """이륙 시작 — 첫 웨이포인트 고도로 상승."""
        if self._waypoints:
            takeoff_z = self._waypoints[0][2]
        else:
            takeoff_z = 3.0  # 기본 이륙 고도

        hold_x, hold_y, _ = self._pos_enu
        self._target_enu = (hold_x, hold_y, takeoff_z)
        self._flight_state = FlightState.TAKEOFF
        self.get_logger().info(
            f'[{self.robot_id}] TAKEOFF → z={takeoff_z:.1f}m (ENU)')

    # ──────────────────────────────────────────────────────────────────────────
    # PX4 커맨드 헬퍼
    # ──────────────────────────────────────────────────────────────────────────

    def publish_vehicle_command(self, command: int, param1: float = 0.0,
                                param2: float = 0.0):
        """VehicleCommand 발행 헬퍼.

        주요 커맨드:
          VEHICLE_CMD_COMPONENT_ARM_DISARM → arm(1.0) / disarm(0.0)
          VEHICLE_CMD_DO_SET_MODE          → offboard(param1=1.0, param2=6.0)
          VEHICLE_CMD_NAV_LAND             → 착륙

        멀티드론: target_system = px4_instance + 1 (0=broadcast)
        """
        if not PX4_MSGS_AVAILABLE:
            return

        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = self.px4_instance + 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self._now_us()
        self.vehicle_command_pub.publish(msg)

    def arm(self):
        """드론 ARM 명령."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info(f'[{self.robot_id}] ARM command sent')

    def disarm(self):
        """드론 DISARM 명령."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info(f'[{self.robot_id}] DISARM command sent')

    def set_offboard_mode(self):
        """PX4 OFFBOARD 모드 전환 명령.

        param1=1.0: MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        param2=6.0: PX4_CUSTOM_MAIN_MODE_OFFBOARD
        OFFBOARD_PREFLIGHT_COUNT 회 발행 후 이 명령을 보내야 전환됨.
        """
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0)
        self.get_logger().info(f'[{self.robot_id}] OFFBOARD mode command sent')

    # ──────────────────────────────────────────────────────────────────────────
    # 서비스 핸들러
    # ──────────────────────────────────────────────────────────────────────────

    def _svc_arm(self, req: Trigger.Request, res: Trigger.Response):
        """arm 서비스 — PREFLIGHT 시퀀스 시작."""
        if not PX4_MSGS_AVAILABLE:
            res.success = False
            res.message = 'px4_msgs 미설치'
            return res

        if self._flight_state != FlightState.IDLE:
            res.success = False
            res.message = f'현재 상태({self._flight_state.name})에서 arm 불가. IDLE 필요'
            return res

        self._preflight_count = 0
        self._flight_state = FlightState.PREFLIGHT
        res.success = True
        res.message = (f'PREFLIGHT 시작 — {self.OFFBOARD_PREFLIGHT_COUNT}회 '
                       'heartbeat 발행 후 자동 ARM')
        self.get_logger().info(f'[{self.robot_id}] {res.message}')
        return res

    def _svc_disarm(self, req: Trigger.Request, res: Trigger.Response):
        """disarm 서비스."""
        if not PX4_MSGS_AVAILABLE:
            res.success = False
            res.message = 'px4_msgs 미설치'
            return res

        self.disarm()
        self._flight_state = FlightState.IDLE
        self._target_enu = None
        res.success = True
        res.message = 'DISARM 명령 전송'
        return res

    def _svc_takeoff(self, req: Trigger.Request, res: Trigger.Response):
        """takeoff 서비스 — ARM 되어있어야 함."""
        if not PX4_MSGS_AVAILABLE:
            res.success = False
            res.message = 'px4_msgs 미설치'
            return res

        if not self._vehicle_armed:
            res.success = False
            res.message = '드론이 ARM 상태가 아닙니다. arm 서비스 먼저 호출'
            return res

        self._start_takeoff()
        res.success = True
        res.message = f'이륙 시작 → z={self._target_enu[2]:.1f}m'
        return res

    def _svc_land(self, req: Trigger.Request, res: Trigger.Response):
        """land 서비스 — 현재 x,y 유지하며 착륙."""
        if not PX4_MSGS_AVAILABLE:
            res.success = False
            res.message = 'px4_msgs 미설치'
            return res

        hold_x, hold_y, _ = self._pos_enu
        self._target_enu = (hold_x, hold_y, 0.0)
        self._flight_state = FlightState.LANDING
        res.success = True
        res.message = f'착륙 시작 — 현재 위치 x={hold_x:.1f} y={hold_y:.1f}'
        self.get_logger().info(f'[{self.robot_id}] {res.message}')
        return res

    def _svc_start_mission(self, req: Trigger.Request, res: Trigger.Response):
        """start_mission 서비스 — 웨이포인트 순회 시작."""
        if not PX4_MSGS_AVAILABLE:
            res.success = False
            res.message = 'px4_msgs 미설치'
            return res

        if not self._vehicle_armed:
            res.success = False
            res.message = '드론이 ARM 상태가 아닙니다.'
            return res

        if not self._waypoints:
            res.success = False
            res.message = '웨이포인트 목록이 비어있습니다.'
            return res

        self._wp_index = 0
        self._flight_state = FlightState.WAYPOINT
        self._set_current_waypoint()
        res.success = True
        res.message = f'웨이포인트 미션 시작 — {len(self._waypoints)}개 웨이포인트'
        self.get_logger().info(f'[{self.robot_id}] {res.message}')
        return res

    def _svc_stop_mission(self, req: Trigger.Request, res: Trigger.Response):
        """stop_mission 서비스 — 현재 위치에서 호버링."""
        self._target_enu = self._pos_enu  # 현재 위치 유지
        self._flight_state = FlightState.HOVER
        res.success = True
        pos = self._pos_enu
        res.message = (f'미션 정지 — HOVER @ ENU({pos[0]:.1f},{pos[1]:.1f},{pos[2]:.1f})')
        self.get_logger().info(f'[{self.robot_id}] {res.message}')
        return res

    def _svc_goto(self, req, res):
        """goto 서비스 — 지정 ENU 좌표로 이동 (argos_interfaces/Goto)."""
        if not PX4_MSGS_AVAILABLE:
            res.success = False
            res.message = 'px4_msgs 미설치'
            return res

        if not self._vehicle_armed:
            res.success = False
            res.message = '드론이 ARM 상태가 아닙니다.'
            return res

        x, y, z = float(req.x), float(req.y), float(req.z)
        if not self._check_geofence(x, y, z):
            res.success = False
            res.message = f'지오펜스 위반: ENU({x:.1f},{y:.1f},{z:.1f})'
            return res

        self._target_enu = (x, y, z)
        self._flight_state = FlightState.HOVER  # goto도 HOVER 상태에서 이동
        res.success = True
        res.message = f'GOTO → ENU({x:.1f},{y:.1f},{z:.1f})'
        self.get_logger().info(f'[{self.robot_id}] {res.message}')
        return res

    # ──────────────────────────────────────────────────────────────────────────
    # 공개 헬퍼 (오케스트레이터 직접 호출용)
    # ──────────────────────────────────────────────────────────────────────────

    def publish_trajectory_setpoint_position(self, x_enu: float, y_enu: float,
                                             z_enu: float, yaw_enu: float = 0.0):
        """ENU 좌표로 위치 setpoint 직접 발행 (오케스트레이터용).

        서비스 대신 오케스트레이터 노드가 직접 호출하는 경우 사용.
        """
        self._publish_position_setpoint(x_enu, y_enu, z_enu, yaw_enu)


def main(args=None):
    rclpy.init(args=args)
    node = PX4BridgeNode()
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
