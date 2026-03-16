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

현재 상태: 스켈레톤 (PX4 미설치 시 독립 실행 불가)
PX4 SITL 설치 후 활성화: launch 파라미터 use_px4:=true
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class PX4BridgeNode(Node):
    """ARGOS cmd_vel ↔ PX4 offboard 브릿지.

    PX4 미설치 시 passthrough 모드 (변환 없이 토픽 릴레이).
    PX4 설치 시 offboard 모드 활성화.
    """

    def __init__(self):
        super().__init__('px4_bridge')

        self.declare_parameter('use_px4', False)  # PX4 활성화 여부
        self.declare_parameter('robot_id', 'drone1')
        self.declare_parameter('offboard_rate', 10.0)  # Hz (PX4: 최소 2Hz)

        self.use_px4 = self.get_parameter('use_px4').value
        self.robot_id = self.get_parameter('robot_id').value

        if self.use_px4:
            self._init_px4_mode()
        else:
            self._init_passthrough_mode()

        self.get_logger().info(
            f'PX4 Bridge initialized (use_px4={self.use_px4}, robot={self.robot_id})')

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
        self.get_logger().info('PX4 offboard mode initializing...')

        # ARGOS → PX4 변환 구독
        self.cmd_sub = self.create_subscription(
            Twist, 'cmd_vel', self._cmd_vel_to_px4, 10)

        # PX4 → ARGOS 변환 (향후 구현)
        # self.px4_odom_sub = ...

        # Offboard keepalive 타이머
        rate = self.get_parameter('offboard_rate').value
        self.offboard_timer = self.create_timer(
            1.0 / rate, self._publish_offboard_heartbeat)

        self.get_logger().warn(
            'PX4 offboard mode — requires PX4 SITL + uXRCE-DDS Agent')

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

    def _cmd_vel_to_px4(self, msg: Twist):
        """ARGOS Twist → PX4 TrajectorySetpoint 변환.

        PX4는 TrajectorySetpoint (NED 좌표) 사용:
          /fmu/in/trajectory_setpoint
          /fmu/in/offboard_control_mode

        현재: 로그만 출력 (PX4 메시지 타입 미임포트 시 스켈레톤)
        """
        # ENU → NED 변환
        vx_ned, vy_ned, vz_ned = self.enu_to_ned(
            msg.linear.x, msg.linear.y, msg.linear.z)

        self.get_logger().debug(
            f'PX4 cmd: vx={vx_ned:.2f} vy={vy_ned:.2f} vz={vz_ned:.2f} '
            f'yaw_rate={msg.angular.z:.2f}',
            throttle_duration_sec=1.0)

        # TODO: PX4 메시지 발행
        # trajectory_setpoint = TrajectorySetpoint()
        # trajectory_setpoint.velocity = [vx_ned, vy_ned, vz_ned]
        # self.trajectory_pub.publish(trajectory_setpoint)

    def _publish_offboard_heartbeat(self):
        """PX4 offboard 모드 유지 heartbeat.

        PX4는 OffboardControlMode를 2Hz 이상 수신해야 offboard 유지.
        미수신 시 자동으로 failsafe (RTL/Land) 전환.
        """
        # TODO: OffboardControlMode 발행
        # offboard_msg = OffboardControlMode()
        # offboard_msg.velocity = True
        # self.offboard_pub.publish(offboard_msg)
        pass


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
