"""
ARGOS Drone Waypoint Controller
=================================
드론의 웨이포인트 비행을 관리하는 간단한 P 제어기.

PX4/ArduPilot 없이 Gazebo MulticopterVelocityControl 플러그인과 연동.
cmd_vel (Twist)로 직접 속도 명령을 전송하여 UGV와 동일한 인터페이스 유지.

기능:
  - 이착륙 (takeoff/land)
  - 웨이포인트 비행 (고도 유지 + 수평 이동)
  - 호버링 (정지 비행)
  - 오케스트레이터 명령 수신

토픽:
  구독: odom (nav_msgs/Odometry) — 위치 추정
        drone/waypoint (PoseStamped) — 웨이포인트 명령
  발행: cmd_vel (Twist) — 속도 명령
        drone/state (String) — 드론 상태
"""

import math
import threading
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger


class DroneController(Node):

    def __init__(self):
        super().__init__('drone_controller')

        # --- Parameters ---
        self.declare_parameter('cruise_altitude', 8.0)    # m
        self.declare_parameter('max_horizontal_speed', 2.0)  # m/s
        self.declare_parameter('max_vertical_speed', 1.0)    # m/s
        self.declare_parameter('position_tolerance', 0.5)    # m
        self.declare_parameter('altitude_tolerance', 0.3)    # m
        self.declare_parameter('control_rate', 20.0)         # Hz
        self.declare_parameter('kp_horizontal', 0.8)         # P gain
        self.declare_parameter('kp_vertical', 1.0)           # P gain
        self.declare_parameter('kp_yaw', 0.5)                # P gain
        self.declare_parameter('landing_timeout', 30.0)        # D2: 착륙 타임아웃 (초)

        self.cruise_alt = self.get_parameter('cruise_altitude').value
        self.max_h_speed = self.get_parameter('max_horizontal_speed').value
        self.max_v_speed = self.get_parameter('max_vertical_speed').value
        self.pos_tol = self.get_parameter('position_tolerance').value
        self.alt_tol = self.get_parameter('altitude_tolerance').value
        rate = self.get_parameter('control_rate').value
        self.kp_h = self.get_parameter('kp_horizontal').value
        self.kp_v = self.get_parameter('kp_vertical').value
        self.kp_yaw = self.get_parameter('kp_yaw').value
        self.landing_timeout = self.get_parameter('landing_timeout').value

        # --- State ---
        self.state = 'grounded'  # grounded, taking_off, hovering, flying, landing
        self.current_pose = None  # (x, y, z, yaw)
        self.target_waypoint = None  # (x, y, z)
        self.waypoint_queue = []
        self._wp_lock = threading.Lock()  # D1: waypoint_queue 스레드 안전
        self._landing_start_time = None   # D2: 착륙 시작 시각

        # --- Subscribers ---
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.waypoint_sub = self.create_subscription(
            PoseStamped, 'drone/waypoint', self.waypoint_callback, 10)

        # --- Publishers ---
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.state_pub = self.create_publisher(String, 'drone/state', 10)

        # --- Services ---
        self.takeoff_srv = self.create_service(
            Trigger, 'drone/takeoff', self.takeoff_callback)
        self.land_srv = self.create_service(
            Trigger, 'drone/land', self.land_callback)

        # --- Control Loop ---
        self.control_timer = self.create_timer(1.0 / rate, self.control_loop)

        self.get_logger().info(
            f'DroneController ready (cruise alt: {self.cruise_alt}m)')

    # ─────────────────── Callbacks ───────────────────

    def odom_callback(self, msg: Odometry):
        """오도메트리로 현재 위치 갱신."""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        # 쿼터니언 → yaw (간단 변환)
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.current_pose = (p.x, p.y, p.z, yaw)

    def waypoint_callback(self, msg: PoseStamped):
        """웨이포인트 수신 → 항상 큐에 추가, hovering이면 즉시 출발."""
        wp = (msg.pose.position.x, msg.pose.position.y,
              msg.pose.position.z if msg.pose.position.z > 0
              else self.cruise_alt)

        with self._wp_lock:
            self.waypoint_queue.append(wp)
            queue_len = len(self.waypoint_queue)
        self.get_logger().info(
            f'Waypoint queued ({wp[0]:.1f}, {wp[1]:.1f}, {wp[2]:.1f}) '
            f'— {queue_len} in queue')

        # hovering 상태이고 현재 목표 없으면 즉시 출발
        if self.state == 'hovering' and self.target_waypoint is None:
            with self._wp_lock:
                if self.waypoint_queue:
                    self.target_waypoint = self.waypoint_queue.pop(0)
            if self.target_waypoint:
                self.state = 'flying'
                self.get_logger().info(
                    f'Flying to ({self.target_waypoint[0]:.1f}, '
                    f'{self.target_waypoint[1]:.1f}, '
                    f'{self.target_waypoint[2]:.1f})')

    def takeoff_callback(self, request, response):
        """이륙 서비스."""
        if self.state == 'grounded' and self.current_pose:
            self.state = 'taking_off'
            x, y, _, _ = self.current_pose
            self.target_waypoint = (x, y, self.cruise_alt)
            self.get_logger().info(
                f'Taking off to {self.cruise_alt}m')
            response.success = True
            response.message = f'Taking off to {self.cruise_alt}m'
        else:
            response.success = False
            response.message = f'Cannot takeoff (state={self.state})'
        return response

    def land_callback(self, request, response):
        """착륙 서비스."""
        if self.state in ('hovering', 'flying') and self.current_pose:
            self.state = 'landing'
            self._landing_start_time = self.get_clock().now()
            x, y, _, _ = self.current_pose
            self.target_waypoint = (x, y, 0.0)
            self.get_logger().info('Landing...')
            response.success = True
            response.message = 'Landing'
        else:
            response.success = False
            response.message = f'Cannot land (state={self.state})'
        return response

    # ─────────────────── Control Loop ───────────────────

    def control_loop(self):
        """P 제어기 기반 속도 명령 생성."""
        if self.current_pose is None:
            return

        cmd = Twist()
        x, y, z, yaw = self.current_pose

        if self.state == 'grounded':
            self.cmd_pub.publish(cmd)  # 정지
            self._publish_state()
            return

        if self.target_waypoint is None:
            self.cmd_pub.publish(cmd)
            self._publish_state()
            return

        tx, ty, tz = self.target_waypoint

        # 수평 거리, 수직 거리
        dx = tx - x
        dy = ty - y
        dz = tz - z
        h_dist = math.hypot(dx, dy)

        # --- 수직 제어 (고도) ---
        vz = self._clamp(self.kp_v * dz, -self.max_v_speed, self.max_v_speed)
        cmd.linear.z = vz

        # --- 수평 제어 (바디 프레임 변환) ---
        # 월드→바디 회전
        cos_yaw = math.cos(-yaw)
        sin_yaw = math.sin(-yaw)
        body_dx = cos_yaw * dx - sin_yaw * dy
        body_dy = sin_yaw * dx + cos_yaw * dy

        vx = self._clamp(self.kp_h * body_dx, -self.max_h_speed, self.max_h_speed)
        vy = self._clamp(self.kp_h * body_dy, -self.max_h_speed, self.max_h_speed)
        cmd.linear.x = vx
        cmd.linear.y = vy

        # --- Yaw 제어 (이동 방향으로 기수 정렬) ---
        if h_dist > self.pos_tol:
            target_yaw = math.atan2(dy, dx)
            yaw_error = target_yaw - yaw
            # -pi ~ pi 정규화
            while yaw_error > math.pi:
                yaw_error -= 2.0 * math.pi
            while yaw_error < -math.pi:
                yaw_error += 2.0 * math.pi
            cmd.angular.z = self._clamp(
                self.kp_yaw * yaw_error, -1.0, 1.0)

        self.cmd_pub.publish(cmd)

        # --- 상태 전환 ---
        if self.state == 'taking_off':
            if abs(dz) < self.alt_tol:
                self.get_logger().info('Hovering at cruise altitude')
                with self._wp_lock:
                    if self.waypoint_queue:
                        self.target_waypoint = self.waypoint_queue.pop(0)
                        self.state = 'flying'
                    else:
                        self.state = 'hovering'
                        self.target_waypoint = None

        elif self.state == 'flying':
            if h_dist < self.pos_tol and abs(dz) < self.alt_tol:
                with self._wp_lock:
                    remaining = len(self.waypoint_queue)
                self.get_logger().info(
                    f'Reached waypoint ({tx:.1f}, {ty:.1f}) '
                    f'— {remaining} remaining')
                with self._wp_lock:
                    if self.waypoint_queue:
                        self.target_waypoint = self.waypoint_queue.pop(0)
                    else:
                        self.state = 'hovering'
                        self.target_waypoint = None
                        self.get_logger().info(
                            'All waypoints completed — hovering')

        elif self.state == 'landing':
            # D2: 착륙 시작 시각 기록
            if self._landing_start_time is None:
                self._landing_start_time = self.get_clock().now()
            # 착륙 조건: 고도 0.3m 이하 AND 수직 오차 < 0.4m
            landing_elapsed = (self.get_clock().now() - self._landing_start_time).nanoseconds / 1e9
            if (z < 0.3 and abs(dz) < 0.4) or landing_elapsed > self.landing_timeout:
                if landing_elapsed > self.landing_timeout:
                    self.get_logger().warn(
                        f'Landing timeout ({self.landing_timeout}s) — forced ground')
                self.state = 'grounded'
                self.target_waypoint = None
                self._landing_start_time = None
                self.get_logger().info('Landed')
                self.cmd_pub.publish(Twist())

        self._publish_state()

    # ─────────────────── Helpers ───────────────────

    @staticmethod
    def _clamp(val, min_val, max_val):
        return max(min_val, min(val, max_val))

    def _publish_state(self):
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 안전 셧다운: 속도 제로 발행 (호버링 모드)
        if node.state != 'grounded':
            node.get_logger().info('Shutdown: sending zero velocity')
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
