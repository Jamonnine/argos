#!/usr/bin/env python3
"""px4_platform.py — ARGOS PX4 드론 PlatformInterface 구현

기존 px4_bridge_node.py 기능을 PlatformInterface로 래핑한다.
오케스트레이터는 이 클래스를 모르고 PlatformInterface만 안다.

아키텍처:
  Orchestrator
      ↓ PlatformInterface.move_to(x, y, z)
  PX4Platform (이 파일)
      ↓ TrajectorySetpoint (ENU→NED 변환)
  Micro XRCE-DDS Agent
      ↓ uXRCE-DDS
  PX4 SITL / 실기체

좌표계 변환:
  ROS 2 (ENU): x=East, y=North, z=Up
  PX4  (NED): x=North, y=East, z=Down
  변환: (x_ned, y_ned, z_ned) = (y_enu, x_enu, -z_enu)

px4_msgs 미설치 환경:
  PX4_MSGS_AVAILABLE = False → 모든 PX4 토픽 발행 스킵.
  emergency_stop은 cmd_vel=0만 발행 (passthrough 모드).

ROS 파라미터:
  robot_id         (string) : 로봇 식별자 (기본: "drone1")
  px4_instance     (int)    : PX4 인스턴스 번호 (0=단일, N→/px4_N/)
  home_x/home_y/home_z (float): 귀환 좌표 (기본: 0.0, 0.0, 3.0)
  cruise_altitude  (float)  : 기본 순항 고도 m (기본: 3.0)
  battery_start    (float)  : 초기 배터리 % (기본: 100.0)
  battery_drain_rate (float): % / 분 소비율 (기본: 2.0)
  geofence_*       (float)  : 지오펜스 ENU 좌표 범위
"""

import math
import threading
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

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

from argos_bringup.platform_interface import PlatformInterface, RobotCapabilities


class PX4Platform(PlatformInterface):
    """PX4 드론 플랫폼 — PlatformInterface 구현체.

    px4_bridge_node.py의 핵심 기능(좌표 변환, Offboard 제어,
    VehicleLocalPosition 수신)을 PlatformInterface 계약으로 노출한다.

    미션 흐름 (Offboard 모드 진입 전제):
      1. move_to(x, y, z) 호출
      2. TrajectorySetpoint(NED 변환) 발행
      3. get_pose()로 위치 확인 (VehicleLocalPosition NED→ENU)
      4. emergency_stop() → DISARM 명령

    px4_bridge_node.py와의 차이:
      - PREFLIGHT/ARMING 상태 머신 없음 (상위 노드 책임)
      - 단순 setpoint 발행 + 위치/배터리 조회에 집중
      - PlatformInterface 계약만 구현

    사용 예시:
      node = rclpy.create_node('drone_wrapper')
      drone = PX4Platform(node, robot_id='drone1')
      drone.move_to(5.0, 3.0, 4.0)   # ENU (East, North, 4m 고도)
    """

    def __init__(self, node: Node, robot_id: str = 'drone1'):
        """PX4Platform 초기화.

        Args:
            node: 이 플랫폼이 귀속될 ROS 2 노드.
            robot_id: 로봇 식별자 (파라미터 네임스페이스에 사용).
        """
        self._node = node
        self._robot_id = robot_id

        # ── 파라미터 (선언 + 읽기) ────────────────────────────────────────────
        node.declare_parameter(f'{robot_id}.px4_instance', 0)
        node.declare_parameter(f'{robot_id}.home_x', 0.0)
        node.declare_parameter(f'{robot_id}.home_y', 0.0)
        node.declare_parameter(f'{robot_id}.home_z', 3.0)
        node.declare_parameter(f'{robot_id}.cruise_altitude', 3.0)
        node.declare_parameter(f'{robot_id}.battery_start', 100.0)
        node.declare_parameter(f'{robot_id}.battery_drain_rate', 2.0)  # % / 분
        # 지오펜스 (ENU, 미터)
        node.declare_parameter(f'{robot_id}.geofence_x_min', -20.0)
        node.declare_parameter(f'{robot_id}.geofence_x_max',  20.0)
        node.declare_parameter(f'{robot_id}.geofence_y_min', -20.0)
        node.declare_parameter(f'{robot_id}.geofence_y_max',  20.0)
        node.declare_parameter(f'{robot_id}.geofence_z_max',  10.0)

        self._px4_instance = node.get_parameter(f'{robot_id}.px4_instance').value
        self._home = (
            node.get_parameter(f'{robot_id}.home_x').value,
            node.get_parameter(f'{robot_id}.home_y').value,
            node.get_parameter(f'{robot_id}.home_z').value,
        )
        self._cruise_alt = node.get_parameter(f'{robot_id}.cruise_altitude').value
        self._battery = node.get_parameter(f'{robot_id}.battery_start').value
        self._drain_rate = node.get_parameter(f'{robot_id}.battery_drain_rate').value
        self._geofence = {
            'x_min': node.get_parameter(f'{robot_id}.geofence_x_min').value,
            'x_max': node.get_parameter(f'{robot_id}.geofence_x_max').value,
            'y_min': node.get_parameter(f'{robot_id}.geofence_y_min').value,
            'y_max': node.get_parameter(f'{robot_id}.geofence_y_max').value,
            'z_max': node.get_parameter(f'{robot_id}.geofence_z_max').value,
        }

        # 멀티드론 FMU 네임스페이스: instance=0이면 접두사 없음, 1이상이면 /px4_N/
        self._fmu_ns = f'/px4_{self._px4_instance}' if self._px4_instance > 0 else ''

        # ── 내부 상태 ─────────────────────────────────────────────────────────
        self._lock = threading.Lock()
        # 현재 ENU 위치 (VehicleLocalPosition NED→ENU 변환 후 저장)
        self._pos_enu: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._armed = False
        self._nav_state = 0
        self._battery_last_update = time.time()
        self._moving = False

        # ── PX4 토픽 퍼블리셔 (px4_msgs 설치 시) ────────────────────────────
        if PX4_MSGS_AVAILABLE:
            ns = self._fmu_ns
            self._offboard_pub = node.create_publisher(
                OffboardControlMode,
                f'{ns}/fmu/in/offboard_control_mode',
                10,
            )
            self._trajectory_pub = node.create_publisher(
                TrajectorySetpoint,
                f'{ns}/fmu/in/trajectory_setpoint',
                10,
            )
            self._vehicle_cmd_pub = node.create_publisher(
                VehicleCommand,
                f'{ns}/fmu/in/vehicle_command',
                10,
            )
            # PX4 상태 구독
            self._status_sub = node.create_subscription(
                VehicleStatus,
                f'{ns}/fmu/out/vehicle_status',
                self._vehicle_status_callback,
                10,
            )
            self._local_pos_sub = node.create_subscription(
                VehicleLocalPosition,
                f'{ns}/fmu/out/vehicle_local_position',
                self._local_position_callback,
                10,
            )
        else:
            node.get_logger().warn(
                f'[PX4Platform:{robot_id}] px4_msgs 미설치 — '
                'PX4 토픽 발행 비활성. passthrough 모드.')

        # ── 긴급 정지용 cmd_vel 퍼블리셔 ────────────────────────────────────
        # emergency_stop 시 passthrough 모드에서도 cmd_vel=0 발행
        self._cmd_pub = node.create_publisher(
            TwistStamped,
            f'{robot_id}/cmd_vel',
            10,
        )

        # ── 배터리 소비 타이머 (10초 주기) ───────────────────────────────────
        node.create_timer(10.0, self._update_battery)

        node.get_logger().info(
            f'[PX4Platform:{robot_id}] 초기화 완료 '
            f'(fmu_ns="{self._fmu_ns}", '
            f'home={self._home}, '
            f'px4_msgs={PX4_MSGS_AVAILABLE})')

    # ──────────────────────────────────────────────────────────────────────────
    # PlatformInterface 구현
    # ──────────────────────────────────────────────────────────────────────────

    def move_to(self, x: float, y: float, z: float = 0.0) -> bool:
        """TrajectorySetpoint으로 ENU 좌표 이동 명령 발행.

        z=0.0이면 cruise_altitude 파라미터를 사용한다.
        지오펜스 위반 시 False 반환.

        좌표 변환:
          ENU (x=East, y=North, z=Up)
          → NED (x=North, y=East, z=Down)

        Args:
            x: 목표 x (m, ENU East)
            y: 목표 y (m, ENU North)
            z: 목표 z (m, ENU Up). 0.0이면 cruise_altitude 사용.

        Returns:
            True: setpoint 발행 성공
            False: 지오펜스 위반, px4_msgs 미설치, 배터리 부족
        """
        if not PX4_MSGS_AVAILABLE:
            self._node.get_logger().warn(
                f'[{self._robot_id}] px4_msgs 미설치 — move_to 스킵')
            return False

        if self._battery < 15.0:
            self._node.get_logger().warn(
                f'[{self._robot_id}] 배터리 부족 ({self._battery:.1f}%) — 이동 거부')
            return False

        # z=0.0이면 기본 순항 고도 사용
        target_z = z if z > 0.0 else self._cruise_alt

        # 지오펜스 검증
        if not self._check_geofence(x, y, target_z):
            return False

        # ENU → NED 변환 후 TrajectorySetpoint 발행
        x_ned, y_ned, z_ned = self.enu_to_ned(x, y, target_z)

        msg = TrajectorySetpoint()
        msg.position = [x_ned, y_ned, z_ned]
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        # ENU yaw=0 → NED yaw = π/2
        msg.yaw = math.pi / 2.0
        msg.timestamp = self._now_us()
        self._trajectory_pub.publish(msg)

        # OffboardControlMode heartbeat 함께 발행 (2Hz 이상 유지 필수)
        self._publish_offboard_mode()

        with self._lock:
            self._moving = True

        self._node.get_logger().info(
            f'[{self._robot_id}] move_to ENU({x:.2f},{y:.2f},{target_z:.2f}) → '
            f'NED({x_ned:.2f},{y_ned:.2f},{z_ned:.2f})')
        return True

    def return_home(self) -> bool:
        """홈 좌표로 귀환.

        파라미터 home_x, home_y, home_z로 정의된 귀환점으로 복귀.

        Returns:
            True: 귀환 setpoint 발행 성공
            False: px4_msgs 미설치 또는 지오펜스 위반
        """
        hx, hy, hz = self._home
        self._node.get_logger().info(
            f'[{self._robot_id}] 귀환 → home({hx:.1f},{hy:.1f},{hz:.1f})')
        return self.move_to(hx, hy, hz)

    def get_pose(self) -> tuple[float, float, float]:
        """현재 위치 반환 (ENU, VehicleLocalPosition NED→ENU 변환).

        px4_msgs 미설치 시 (0.0, 0.0, 0.0) 반환.
        _local_position_callback에서 최신 위치로 갱신됨.

        Returns:
            (x, y, z) 튜플 (m, ENU 좌표계)
        """
        with self._lock:
            return self._pos_enu

    def get_battery(self) -> float:
        """배터리 잔량 반환 (파라미터 기반 시뮬레이션).

        실 배터리 센서 없음 — 시간 경과 + 이동 여부로 추정.
        드론은 UGV보다 소비율이 높다 (기본 2.0% / 분).

        Returns:
            배터리 잔량 (%, 0.0 ~ 100.0)
        """
        with self._lock:
            return max(0.0, round(self._battery, 1))

    def get_capabilities(self) -> RobotCapabilities:
        """PX4 드론 능력 명세 반환.

        ARGOS 드론 하드웨어 구성 기반:
          - 비행 가능 (PX4 Offboard)
          - 지상 주행 불가
          - 열화상 탑재 (drone_controller_node에서 처리)
          - LiDAR 없음 (드론은 카메라 기반 탐지)

        Returns:
            드론 고정 능력 명세 (캐시됨)
        """
        return RobotCapabilities(
            can_fly=True,
            can_drive=False,
            has_thermal=True,
            has_lidar=False,
            max_speed=2.0,       # 드론 최대 수평 속도 (m/s)
            battery_capacity=self._battery,
            platform_type='drone',
        )

    def emergency_stop(self) -> None:
        """긴급 정지 — DISARM 명령 + cmd_vel=0 발행.

        PX4 Tier-3 SW E-STOP:
          1) VEHICLE_CMD_COMPONENT_ARM_DISARM(0) → PX4 DISARM
          2) cmd_vel=0 발행 (passthrough 모드 대비)

        DISARM은 착륙 완료 후 실행하는 것이 안전하지만
        긴급 상황에서는 강제 DISARM 허용 (안전 책임은 상위 시스템).

        예외 억제 — 오케스트레이터 루프 보호.
        """
        self._node.get_logger().error(
            f'[{self._robot_id}] EMERGENCY STOP — DISARM 명령 전송')

        # 채널 1: PX4 DISARM (ARM_DISARM param1=0.0)
        if PX4_MSGS_AVAILABLE:
            try:
                self._publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                    param1=0.0,
                )
            except Exception as e:
                self._node.get_logger().error(
                    f'[{self._robot_id}] DISARM 명령 실패: {e}')

        # 채널 2: cmd_vel=0 (passthrough 모드 + 추가 안전)
        try:
            stop_msg = TwistStamped()
            stop_msg.header.stamp = self._node.get_clock().now().to_msg()
            stop_msg.header.frame_id = 'base_link'
            # 모든 속도 필드 기본값 0.0
            self._cmd_pub.publish(stop_msg)
        except Exception as e:
            self._node.get_logger().error(
                f'[{self._robot_id}] cmd_vel 제로 발행 실패: {e}')

        with self._lock:
            self._moving = False

    # ──────────────────────────────────────────────────────────────────────────
    # PX4 유틸리티
    # ──────────────────────────────────────────────────────────────────────────

    @staticmethod
    def enu_to_ned(x_enu: float, y_enu: float, z_enu: float,
                   ) -> Tuple[float, float, float]:
        """ENU (ROS2) → NED (PX4) 좌표 변환.

        ENU: x=East,  y=North, z=Up
        NED: x=North, y=East,  z=Down
        """
        return y_enu, x_enu, -z_enu

    @staticmethod
    def ned_to_enu(x_ned: float, y_ned: float, z_ned: float,
                   ) -> Tuple[float, float, float]:
        """NED (PX4) → ENU (ROS2) 좌표 변환."""
        return y_ned, x_ned, -z_ned

    def _now_us(self) -> int:
        """현재 ROS clock을 마이크로초(us)로 반환 — PX4 timestamp 필드용."""
        return self._node.get_clock().now().nanoseconds // 1000

    def _check_geofence(self, x_enu: float, y_enu: float, z_enu: float) -> bool:
        """목표 좌표가 지오펜스 내부인지 검증. 위반 시 False."""
        gf = self._geofence
        if not (gf['x_min'] <= x_enu <= gf['x_max']):
            self._node.get_logger().warn(
                f'[{self._robot_id}] 지오펜스 위반: x={x_enu:.2f} '
                f'범위[{gf["x_min"]},{gf["x_max"]}]')
            return False
        if not (gf['y_min'] <= y_enu <= gf['y_max']):
            self._node.get_logger().warn(
                f'[{self._robot_id}] 지오펜스 위반: y={y_enu:.2f} '
                f'범위[{gf["y_min"]},{gf["y_max"]}]')
            return False
        if z_enu > gf['z_max']:
            self._node.get_logger().warn(
                f'[{self._robot_id}] 지오펜스 위반: z={z_enu:.2f} > '
                f'z_max={gf["z_max"]}')
            return False
        if z_enu < 0.0:
            self._node.get_logger().warn(
                f'[{self._robot_id}] 지오펜스 위반: z={z_enu:.2f} < 0')
            return False
        return True

    def _publish_offboard_mode(self):
        """OffboardControlMode heartbeat 발행 (위치 제어 모드).

        PX4는 이 메시지를 2Hz 이상 수신해야 offboard 모드를 유지한다.
        move_to 호출 시마다 함께 발행 (별도 타이머 불필요).
        """
        if not PX4_MSGS_AVAILABLE:
            return
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self._now_us()
        self._offboard_pub.publish(msg)

    def _publish_vehicle_command(self, command: int,
                                  param1: float = 0.0,
                                  param2: float = 0.0):
        """VehicleCommand 발행 헬퍼.

        멀티드론: target_system = px4_instance + 1 (0=broadcast).
        """
        if not PX4_MSGS_AVAILABLE:
            return
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = self._px4_instance + 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self._now_us()
        self._vehicle_cmd_pub.publish(msg)

    # ──────────────────────────────────────────────────────────────────────────
    # PX4 구독 콜백
    # ──────────────────────────────────────────────────────────────────────────

    def _vehicle_status_callback(self, msg: 'VehicleStatus'):
        """PX4 VehicleStatus 수신 — arming 상태 갱신."""
        with self._lock:
            # arming_state: 1=STANDBY, 2=ARMED
            self._armed = (msg.arming_state == 2)
            self._nav_state = msg.nav_state

    def _local_position_callback(self, msg: 'VehicleLocalPosition'):
        """PX4 VehicleLocalPosition 수신 — 현재 위치(NED→ENU) 갱신.

        PX4 local position은 NED 좌표계.
        get_pose() 호출 시 이 값을 반환한다.
        """
        enu = self.ned_to_enu(msg.x, msg.y, msg.z)
        with self._lock:
            self._pos_enu = enu

    # ──────────────────────────────────────────────────────────────────────────
    # 배터리 시뮬레이션
    # ──────────────────────────────────────────────────────────────────────────

    def _update_battery(self):
        """배터리 소비 시뮬레이션 (10초 주기 타이머 콜백).

        비행 중: drain_rate × 주기 분 소비
        호버링: drain_rate × 0.6 소비 (정지 비행도 전력 많이 사용)
        """
        now = time.time()
        elapsed_min = (now - self._battery_last_update) / 60.0

        with self._lock:
            drain_factor = 1.0 if self._moving else 0.6
            drain = self._drain_rate * elapsed_min * drain_factor
            self._battery = max(0.0, self._battery - drain)
            self._battery_last_update = now

        if self._battery < 25.0:
            self._node.get_logger().warn(
                f'[{self._robot_id}] 드론 배터리 경고: {self._battery:.1f}% '
                '(RTL 권장)',
                throttle_duration_sec=30.0,
            )


def main(args=None):
    """PX4Platform 단독 실행 테스트 노드."""
    rclpy.init(args=args)
    node = rclpy.create_node('px4_platform_test')
    drone = PX4Platform(node, robot_id='drone1')

    node.get_logger().info('PX4Platform 테스트 노드 시작')
    node.get_logger().info(f'capabilities: {drone.get_capabilities()}')
    node.get_logger().info(f'battery: {drone.get_battery()}%')
    node.get_logger().info(f'pose: {drone.get_pose()}')
    node.get_logger().info(f'px4_msgs 설치 여부: {PX4_MSGS_AVAILABLE}')

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
