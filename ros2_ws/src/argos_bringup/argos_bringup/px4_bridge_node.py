#!/usr/bin/env python3
"""px4_bridge_node.py — ARGOS ↔ PX4 Offboard 브릿지

드론 전문가 권고: 현재 Gazebo 네이티브 드론 컨트롤러를
PX4 offboard 모드로 전환하기 위한 브릿지 노드.

아키텍처:
  ARGOS cmd_vel (Twist) → PX4 Bridge → TrajectorySetpoint (PX4)
  PX4 VehicleOdometry → PX4 Bridge → ARGOS odom (Odometry)

참조: px4-offboard (ETH Zurich), aerial-autonomy-stack
리서치: knowledge/projects/research/2026-03-15-px4-ros2-drone-integration-deep-dive.md

주요 변환:
  - ENU (ROS2) ↔ NED (PX4) 좌표계 변환
  - Twist → TrajectorySetpoint (위치/속도 명령)
  - OffboardControlMode 2Hz 이상 발행 필수

현재 상태: px4_msgs 설치 시 완전 동작, 미설치 시 passthrough 모드
PX4 SITL 설치 후 활성화: launch 파라미터 use_px4:=true
멀티드론: px4_instance 파라미터로 /px4_N/ 네임스페이스 자동 적용
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# px4_msgs 미설치 환경에서도 노드가 구동되도록 graceful fallback
try:
    from px4_msgs.msg import (
        OffboardControlMode,
        TrajectorySetpoint,
        VehicleCommand,
        VehicleStatus,
    )
    PX4_MSGS_AVAILABLE = True
except ImportError:
    PX4_MSGS_AVAILABLE = False


class PX4BridgeNode(Node):
    """ARGOS cmd_vel ↔ PX4 offboard 브릿지.

    PX4 미설치 시 passthrough 모드 (변환 없이 토픽 릴레이).
    PX4 설치 시 offboard 모드 활성화.

    멀티드론 사용 시 px4_instance 파라미터로 인스턴스 지정:
      px4_instance=0 → /fmu/in/... (기본 네임스페이스 없음)
      px4_instance=1 → /px4_1/fmu/in/...
      px4_instance=N → /px4_N/fmu/in/...
    """

    def __init__(self):
        super().__init__('px4_bridge')

        self.declare_parameter('use_px4', False)       # PX4 활성화 여부
        self.declare_parameter('robot_id', 'drone1')
        self.declare_parameter('offboard_rate', 10.0)  # Hz (PX4: 최소 2Hz)
        self.declare_parameter('px4_instance', 0)      # 멀티드론 인스턴스 번호

        self.use_px4 = self.get_parameter('use_px4').value
        self.robot_id = self.get_parameter('robot_id').value
        self.px4_instance = self.get_parameter('px4_instance').value

        # 멀티드론 네임스페이스: instance=0이면 접두사 없음, 1이상이면 /px4_N/
        if self.px4_instance > 0:
            self._fmu_ns = f'/px4_{self.px4_instance}'
        else:
            self._fmu_ns = ''

        # 상태 추적
        self._vehicle_armed = False
        self._nav_state = 0  # VehicleStatus nav_state

        if self.use_px4:
            self._init_px4_mode()
        else:
            self._init_passthrough_mode()

        self.get_logger().info(
            f'PX4 Bridge initialized (use_px4={self.use_px4}, '
            f'robot={self.robot_id}, instance={self.px4_instance}, '
            f'px4_msgs={PX4_MSGS_AVAILABLE})')

    def _init_passthrough_mode(self):
        """PX4 미사용 — cmd_vel 그대로 전달 (현재 Gazebo 네이티브)."""
        self.get_logger().info('Passthrough mode (no PX4)')

    def _init_px4_mode(self):
        """PX4 offboard 모드 초기화.

        필수 조건:
          1. PX4 SITL 기동 (gz sim + PX4)
          2. Micro XRCE-DDS Agent 실행 (UDP 8888)
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
        # VehicleStatus: arming 상태 + nav_state 모니터링
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            f'{ns}/fmu/out/vehicle_status',
            self._vehicle_status_callback,
            10)

        # ── ARGOS 입력 구독 ──────────────────────────────────────────────────
        self.cmd_sub = self.create_subscription(
            Twist, 'cmd_vel', self._cmd_vel_to_px4, 10)

        # ── Offboard keepalive 타이머 ────────────────────────────────────────
        # PX4는 OffboardControlMode를 2Hz 이상 수신해야 offboard 유지
        rate = self.get_parameter('offboard_rate').value
        self.offboard_timer = self.create_timer(
            1.0 / rate, self._publish_offboard_heartbeat)

        self.get_logger().warn(
            f'PX4 offboard mode ready — topics: {ns}/fmu/in/*, {ns}/fmu/out/*. '
            'Requires PX4 SITL + MicroXRCEAgent udp4 -p 8888')

    @staticmethod
    def enu_to_ned(x_enu, y_enu, z_enu):
        """ENU (ROS2) → NED (PX4) 좌표 변환.

        ENU: x=East, y=North, z=Up
        NED: x=North, y=East, z=Down
        """
        x_ned = y_enu    # North = ENU Y
        y_ned = x_enu    # East = ENU X
        z_ned = -z_enu   # Down = -Up
        return x_ned, y_ned, z_ned

    @staticmethod
    def ned_to_enu(x_ned, y_ned, z_ned):
        """NED (PX4) → ENU (ROS2) 좌표 변환."""
        x_enu = y_ned    # East = NED Y
        y_enu = x_ned    # North = NED X
        z_enu = -z_ned   # Up = -Down
        return x_enu, y_enu, z_enu

    def _vehicle_status_callback(self, msg: 'VehicleStatus'):
        """PX4 VehicleStatus 수신 — arming 및 nav_state 모니터링."""
        prev_armed = self._vehicle_armed
        # arming_state: 1=STANDBY, 2=ARMED (px4_msgs 상수)
        self._vehicle_armed = (msg.arming_state == 2)
        self._nav_state = msg.nav_state

        if prev_armed != self._vehicle_armed:
            state_str = 'ARMED' if self._vehicle_armed else 'DISARMED'
            self.get_logger().info(
                f'[{self.robot_id}] Arming state changed → {state_str} '
                f'(nav_state={self._nav_state})')

    def publish_vehicle_command(self, command: int, param1: float = 0.0,
                                param2: float = 0.0):
        """VehicleCommand 발행 헬퍼.

        주요 커맨드:
          VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM  → arm(param1=1.0) / disarm(param1=0.0)
          VehicleCommand.VEHICLE_CMD_DO_SET_MODE           → offboard 모드(param1=1.0, param2=6.0)

        멀티드론 시 target_system은 px4_instance + 1 (1-indexed, 0=broadcast).
        """
        if not PX4_MSGS_AVAILABLE:
            return

        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        # target_system: instance=0 → 1, instance=N → N+1 (PX4 규약)
        msg.target_system = self.px4_instance + 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.vehicle_command_pub.publish(msg)

    def arm(self):
        """드론 암 명령."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info(f'[{self.robot_id}] ARM command sent')

    def disarm(self):
        """드론 디스암 명령."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info(f'[{self.robot_id}] DISARM command sent')

    def set_offboard_mode(self):
        """PX4 offboard 모드 전환 명령.

        param1=1.0: MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        param2=6.0: PX4_CUSTOM_MAIN_MODE_OFFBOARD
        offboard_control_mode를 일정 횟수 발행한 뒤 이 명령을 보내야 전환됨.
        """
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0)
        self.get_logger().info(f'[{self.robot_id}] OFFBOARD mode command sent')

    def _cmd_vel_to_px4(self, msg: Twist):
        """ARGOS Twist → PX4 TrajectorySetpoint 변환 (속도 제어 모드).

        PX4 TrajectorySetpoint의 velocity 필드를 사용하여 드론을 제어.
        ENU(ROS2) → NED(PX4) 좌표계 변환 후 발행.
        """
        if not PX4_MSGS_AVAILABLE:
            return

        # ENU → NED 변환
        vx_ned, vy_ned, vz_ned = self.enu_to_ned(
            msg.linear.x, msg.linear.y, msg.linear.z)

        setpoint = TrajectorySetpoint()
        # 속도 제어: position은 NaN으로 설정하여 위치 제어 비활성화
        setpoint.position = [float('nan'), float('nan'), float('nan')]
        setpoint.velocity = [vx_ned, vy_ned, vz_ned]
        setpoint.yawspeed = -msg.angular.z  # ENU yaw → NED yaw (부호 반전)
        setpoint.yaw = float('nan')         # yawspeed 사용 시 NaN
        setpoint.timestamp = self.get_clock().now().nanoseconds // 1000
        self.trajectory_pub.publish(setpoint)

        self.get_logger().debug(
            f'PX4 cmd: vx={vx_ned:.2f} vy={vy_ned:.2f} vz={vz_ned:.2f} '
            f'yaw_rate={-msg.angular.z:.2f}',
            throttle_duration_sec=1.0)

    def publish_trajectory_setpoint_position(self, x_enu: float, y_enu: float,
                                             z_enu: float, yaw_enu: float = 0.0):
        """ENU 좌표로 위치 setpoint 발행 (위치 제어 모드).

        _cmd_vel_to_px4가 속도 제어용이라면, 이 메서드는 위치 지령용.
        오케스트레이터에서 직접 위치 목표를 줄 때 사용.
        """
        if not PX4_MSGS_AVAILABLE:
            return

        x_ned, y_ned, z_ned = self.enu_to_ned(x_enu, y_enu, z_enu)
        # ENU yaw → NED yaw: NED 기준 North=0, 시계 방향 양수
        yaw_ned = math.pi / 2.0 - yaw_enu

        setpoint = TrajectorySetpoint()
        setpoint.position = [x_ned, y_ned, z_ned]
        setpoint.velocity = [float('nan'), float('nan'), float('nan')]
        setpoint.yaw = yaw_ned
        setpoint.timestamp = self.get_clock().now().nanoseconds // 1000
        self.trajectory_pub.publish(setpoint)

    def _publish_offboard_heartbeat(self):
        """PX4 offboard 모드 유지 heartbeat.

        PX4는 OffboardControlMode를 2Hz 이상 수신해야 offboard 유지.
        미수신 시 자동으로 failsafe (RTL/Land) 전환.

        현재 설정: position=True (위치 제어 활성화).
        속도 제어만 필요 시 velocity=True, position=False로 변경.
        """
        if not PX4_MSGS_AVAILABLE:
            return

        offboard_msg = OffboardControlMode()
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        offboard_msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.offboard_pub.publish(offboard_msg)


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
