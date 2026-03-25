"""
ARGOS Drone Waypoint Controller
=================================
드론의 웨이포인트 비행을 관리하는 PID 제어기.

PX4/ArduPilot 없이 Gazebo MulticopterVelocityControl 플러그인과 연동.
cmd_vel (TwistStamped)로 직접 속도 명령을 전송하여 UGV와 동일한 인터페이스 유지.

기능:
  - 이착륙 (takeoff/land)
  - 웨이포인트 비행 (고도 유지 + 수평 이동)
  - 호버링 (정지 비행)
  - 오케스트레이터 명령 수신

토픽:
  구독: odom (nav_msgs/Odometry) — 위치 추정
        drone/waypoint (PoseStamped) — 웨이포인트 명령
  발행: cmd_vel (TwistStamped) — 속도 명령
        drone/state (String) — 드론 상태

LifecycleNode 상태:
  unconfigured → configure() → inactive
  inactive     → activate()  → active (구독/발행/서비스/타이머 활성)
  active       → deactivate() → inactive (통신 중단)
  inactive     → cleanup()   → unconfigured
"""

import math
import threading
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, State

from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger


class DroneController(LifecycleNode):

    def __init__(self):
        super().__init__('drone_controller')

        # ── 파라미터 선언만 (activate 전에는 토픽·타이머·서비스 생성 금지) ──
        self.declare_parameter('cruise_altitude', 8.0)    # m
        self.declare_parameter('max_horizontal_speed', 2.0)  # m/s
        self.declare_parameter('max_vertical_speed', 1.0)    # m/s
        self.declare_parameter('position_tolerance', 0.5)    # m
        self.declare_parameter('altitude_tolerance', 0.3)    # m
        self.declare_parameter('control_rate', 20.0)         # Hz
        self.declare_parameter('kp_horizontal', 0.8)         # P gain
        self.declare_parameter('ki_horizontal', 0.05)        # I gain (PID)
        self.declare_parameter('kd_horizontal', 0.15)        # D gain (PID)
        self.declare_parameter('kp_vertical', 1.0)           # P gain
        self.declare_parameter('ki_vertical', 0.02)          # I gain
        self.declare_parameter('kd_vertical', 0.1)           # D gain
        self.declare_parameter('kp_yaw', 0.5)                # P gain
        self.declare_parameter('landing_timeout', 30.0)      # 착륙 타임아웃 (초)
        # 배터리 모델 (드론 전문가 권고)
        self.declare_parameter('battery_capacity_mah', 2000.0)
        self.declare_parameter('battery_rtl_percent', 20.0)  # RTL 임계값 (%)
        self.declare_parameter('power_consumption_w', 80.0)  # 호버링 평균 소비 전력
        # 지오펜스 (드론 전문가 권고: 비행 범위 제한)
        self.declare_parameter('geofence_x_min', -20.0)
        self.declare_parameter('geofence_x_max', 20.0)
        self.declare_parameter('geofence_y_min', -20.0)
        self.declare_parameter('geofence_y_max', 20.0)
        self.declare_parameter('geofence_z_max', 15.0)  # 최대 고도

        # 런타임 파라미터 (on_configure에서 초기화)
        self.cruise_alt = None
        self.max_h_speed = None
        self.max_v_speed = None
        self.pos_tol = None
        self.alt_tol = None
        self.kp_h = None
        self.ki_h = None
        self.kd_h = None
        self.kp_v = None
        self.ki_v = None
        self.kd_v = None
        self.kp_yaw = None
        self.landing_timeout = None
        self._geofence = None

        # PID 상태 (on_configure에서 초기화)
        self._integral_h = None
        self._integral_v = None
        self._prev_error_h = None
        self._prev_error_v = None
        self._integral_max = None

        # 배터리 모델 (on_configure에서 초기화)
        self._battery_voltage = None
        self._battery_energy_wh = None
        self._battery_remaining_wh = None
        self._battery_rtl_percent = None
        self._power_consumption = None
        self._battery_rtl_triggered = None

        # 비행 상태 (on_configure에서 초기화)
        self.state = None
        self.current_pose = None
        self.target_waypoint = None
        self.waypoint_queue = None
        self._wp_lock = None
        self._landing_start_time = None

        # Failsafe 타이밍 (on_configure에서 초기화)
        self._odom_timeout_sec = None
        self._last_odom_time = None
        self._last_control_time = None

        # 구독/발행/서비스/타이머 핸들 (on_activate에서 생성, on_deactivate에서 해제)
        self.odom_sub = None
        self.waypoint_sub = None
        self.cmd_pub = None
        self.state_pub = None
        self.takeoff_srv = None
        self.land_srv = None
        self.control_timer = None

    # ─────────────────── Lifecycle Callbacks ───────────────────

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """파라미터 읽기 + 내부 상태 초기화.

        inactive 상태로 진입 전 호출됨. ROS 통신(토픽·서비스·타이머)은 만들지 않는다.
        """
        self.cruise_alt = self.get_parameter('cruise_altitude').value
        self.max_h_speed = self.get_parameter('max_horizontal_speed').value
        self.max_v_speed = self.get_parameter('max_vertical_speed').value
        self.pos_tol = self.get_parameter('position_tolerance').value
        self.alt_tol = self.get_parameter('altitude_tolerance').value
        self.kp_h = self.get_parameter('kp_horizontal').value
        self.ki_h = self.get_parameter('ki_horizontal').value
        self.kd_h = self.get_parameter('kd_horizontal').value
        self.kp_v = self.get_parameter('kp_vertical').value
        self.ki_v = self.get_parameter('ki_vertical').value
        self.kd_v = self.get_parameter('kd_vertical').value
        self.kp_yaw = self.get_parameter('kp_yaw').value
        self.landing_timeout = self.get_parameter('landing_timeout').value

        # 지오펜스 유효성 검증
        gx_min = self.get_parameter('geofence_x_min').value
        gx_max = self.get_parameter('geofence_x_max').value
        gy_min = self.get_parameter('geofence_y_min').value
        gy_max = self.get_parameter('geofence_y_max').value
        gz_max = self.get_parameter('geofence_z_max').value
        if gx_min >= gx_max or gy_min >= gy_max or gz_max <= 0:
            self.get_logger().error(
                f'Invalid geofence params: x=[{gx_min},{gx_max}] '
                f'y=[{gy_min},{gy_max}] z_max={gz_max}')
            return TransitionCallbackReturn.FAILURE
        self._geofence = {
            'x': (gx_min, gx_max),
            'y': (gy_min, gy_max),
            'z_max': gz_max,
        }

        # PID 상태 초기화
        self._integral_h = [0.0, 0.0]  # [x, y] 적분 오차
        self._integral_v = 0.0          # z 적분 오차
        self._prev_error_h = [0.0, 0.0] # [x, y] 이전 오차
        self._prev_error_v = 0.0        # z 이전 오차
        self._integral_max = 5.0        # 적분 포화 방지

        # 배터리 모델 초기화
        battery_cap = self.get_parameter('battery_capacity_mah').value
        self._battery_voltage = 14.8  # 4S LiPo 공칭 전압
        self._battery_energy_wh = battery_cap * self._battery_voltage / 1000.0
        self._battery_remaining_wh = self._battery_energy_wh
        self._battery_rtl_percent = self.get_parameter('battery_rtl_percent').value
        self._power_consumption = self.get_parameter('power_consumption_w').value
        self._battery_rtl_triggered = False

        # 비행 상태 초기화
        self.state = 'grounded'  # grounded, taking_off, hovering, flying, landing
        self.current_pose = None  # (x, y, z, yaw)
        self.target_waypoint = None  # (x, y, z)
        self.waypoint_queue = []
        self._wp_lock = threading.Lock()  # waypoint_queue 스레드 안전
        self._landing_start_time = None

        # Failsafe 타이밍 초기화 (오도메트리 손실 시 자동 착륙)
        self._odom_timeout_sec = 3.0  # N초 미수신 시 failsafe
        self._last_odom_time = None
        self._last_control_time = None  # PID dt 실측용

        self.get_logger().info(
            f'DroneController configured (cruise alt: {self.cruise_alt}m, '
            f'geofence: x={self._geofence["x"]} y={self._geofence["y"]} z_max={gz_max})')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """구독/발행/서비스/타이머 생성 → 임무 시작.

        active 상태로 진입 전 호출됨.
        """
        rate = self.get_parameter('control_rate').value

        # --- Subscribers ---
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.waypoint_sub = self.create_subscription(
            PoseStamped, 'drone/waypoint', self.waypoint_callback, 10)

        # --- Publishers ---
        self.cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.state_pub = self.create_publisher(String, 'drone/state', 10)

        # --- Services ---
        self.takeoff_srv = self.create_service(
            Trigger, 'drone/takeoff', self.takeoff_callback)
        self.land_srv = self.create_service(
            Trigger, 'drone/land', self.land_callback)

        # --- Control Loop ---
        self.control_timer = self.create_timer(1.0 / rate, self.control_loop)

        self.get_logger().info('DroneController activated')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """모든 핸들 destroy → 통신 중단.

        inactive 상태로 진입 전 호출됨.
        """
        # 타이머 해제
        if self.control_timer is not None:
            self.control_timer.cancel()
            self.destroy_timer(self.control_timer)
            self.control_timer = None

        # 서비스 해제
        if self.takeoff_srv is not None:
            self.destroy_service(self.takeoff_srv)
            self.takeoff_srv = None
        if self.land_srv is not None:
            self.destroy_service(self.land_srv)
            self.land_srv = None

        # 발행자 해제
        if self.cmd_pub is not None:
            self.destroy_publisher(self.cmd_pub)
            self.cmd_pub = None
        if self.state_pub is not None:
            self.destroy_publisher(self.state_pub)
            self.state_pub = None

        # 구독자 해제
        if self.waypoint_sub is not None:
            self.destroy_subscription(self.waypoint_sub)
            self.waypoint_sub = None
        if self.odom_sub is not None:
            self.destroy_subscription(self.odom_sub)
            self.odom_sub = None

        self.get_logger().info('DroneController deactivated')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """내부 상태 리셋 → unconfigured로 복귀."""
        self.state = None
        self.current_pose = None
        self.target_waypoint = None
        self.waypoint_queue = None
        self._wp_lock = None
        self._landing_start_time = None
        self._integral_h = None
        self._integral_v = None
        self._prev_error_h = None
        self._prev_error_v = None
        self._battery_rtl_triggered = None
        self._battery_remaining_wh = None
        self._last_odom_time = None
        self._last_control_time = None
        self._geofence = None
        self.get_logger().info('DroneController cleaned up')
        return TransitionCallbackReturn.SUCCESS

    # ─────────────────── Callbacks ───────────────────

    def odom_callback(self, msg: Odometry):
        """오도메트리로 현재 위치 갱신 + failsafe 타이머 리셋."""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.current_pose = (p.x, p.y, p.z, yaw)
        self._last_odom_time = self.get_clock().now()

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

    def _check_drone_collision(self, target_x, target_y, target_z,
                               safety_radius=3.0) -> bool:
        """멀티드론 충돌 위험 체크 (드론 전문가 권고).

        다른 드론의 위치와 대상 위치 간 거리가 safety_radius 이내면 True.
        현재: 오케스트레이터가 관리하므로 여기서는 로컬 체크만.
        """
        # 단일 드론 환경에서는 항상 False
        # 멀티드론 환경에서는 /exploration/targets 구독으로 확인
        if hasattr(self, '_other_drone_positions'):
            for ox, oy, oz in self._other_drone_positions:
                dist = math.sqrt(
                    (target_x - ox)**2 + (target_y - oy)**2 + (target_z - oz)**2)
                if dist < safety_radius:
                    self.get_logger().warn(
                        f'DRONE COLLISION RISK: target ({target_x:.1f},{target_y:.1f},{target_z:.1f}) '
                        f'too close to other drone ({ox:.1f},{oy:.1f},{oz:.1f}) dist={dist:.1f}m')
                    return True
        return False

    def takeoff_callback(self, request, response):
        """이륙 서비스."""
        if self.state != 'grounded':
            response.success = False
            response.message = f'Cannot takeoff (state={self.state})'
            return response

        if self.current_pose:
            x, y, _, _ = self.current_pose
        else:
            # odom 미수신 시 스폰 좌표(0,0)에서 이륙
            self.get_logger().warn('No odom yet, taking off from origin')
            x, y = 0.0, 0.0

        self.state = 'taking_off'
        self.target_waypoint = (x, y, self.cruise_alt)
        self.get_logger().info(f'Taking off to {self.cruise_alt}m')
        response.success = True
        response.message = f'Taking off to {self.cruise_alt}m'
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

    def _check_geofence(self, x, y, z) -> bool:
        """지오펜스 범위 체크. 범위 밖이면 True (위반)."""
        gf = self._geofence
        if x < gf['x'][0] or x > gf['x'][1]:
            return True
        if y < gf['y'][0] or y > gf['y'][1]:
            return True
        if z > gf['z_max']:
            return True
        return False

    def control_loop(self):
        """PID 제어기 기반 속도 명령 생성 (v2: P→PID 전환)."""
        if self.current_pose is None:
            return

        # Failsafe: 오도메트리 N초 미수신 시 자동 착륙 (드론 전문가 권고)
        if (self._last_odom_time is not None and
                self.state not in ('grounded', 'landing')):
            odom_age = (self.get_clock().now() - self._last_odom_time).nanoseconds / 1e9
            if odom_age > self._odom_timeout_sec:
                self.get_logger().error(
                    f'ODOM TIMEOUT ({odom_age:.1f}s) — FAILSAFE AUTO LAND')
                self.state = 'landing'
                self._landing_start_time = self.get_clock().now()
                x, y, _, _ = self.current_pose
                self.target_waypoint = (x, y, 0.0)

        cmd = self._make_cmd()
        x, y, z, yaw = self.current_pose

        # 지오펜스 체크 (비행 중에만)
        if self.state not in ('grounded',) and self._check_geofence(x, y, z):
            self.get_logger().error(
                f'GEOFENCE VIOLATION at ({x:.1f}, {y:.1f}, {z:.1f}) — AUTO LAND')
            self.state = 'landing'
            self._landing_start_time = self.get_clock().now()
            self.target_waypoint = (x, y, 0.0)

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

        # --- 배터리 시뮬레이션 (비행 중 소비) ---
        # 실측 dt 계산 (고정 dt 대신 실제 경과 시간 사용)
        now_time = self.get_clock().now()
        if self._last_control_time is not None:
            dt = (now_time - self._last_control_time).nanoseconds / 1e9
            dt = max(0.001, min(dt, 0.5))  # 안전 범위: 1ms ~ 500ms
        else:
            dt = 1.0 / self.get_parameter('control_rate').value
        self._last_control_time = now_time

        if self.state not in ('grounded',):
            self._battery_remaining_wh -= self._power_consumption * dt / 3600.0
            battery_percent = max(0.0, self._battery_remaining_wh / self._battery_energy_wh * 100.0)
            if battery_percent <= self._battery_rtl_percent and not self._battery_rtl_triggered:
                self._battery_rtl_triggered = True
                self.get_logger().error(
                    f'BATTERY LOW ({battery_percent:.0f}%) — AUTO RTL')
                self.state = 'landing'
                self._landing_start_time = self.get_clock().now()
                if self.current_pose:
                    self.target_waypoint = (x, y, 0.0)

        # --- 수직 PID 제어 (고도) ---
        self._integral_v = self._clamp(
            self._integral_v + dz, -self._integral_max, self._integral_max)
        d_error_v = dz - self._prev_error_v
        self._prev_error_v = dz
        vz = self._clamp(
            self.kp_v * dz + self.ki_v * self._integral_v + self.kd_v * d_error_v,
            -self.max_v_speed, self.max_v_speed)
        cmd.twist.linear.z = vz

        # --- 수평 PID 제어 (바디 프레임 변환) ---
        cos_yaw = math.cos(-yaw)
        sin_yaw = math.sin(-yaw)
        body_dx = cos_yaw * dx - sin_yaw * dy
        body_dy = sin_yaw * dx + cos_yaw * dy

        # PID: 적분 + 미분 항 추가
        self._integral_h[0] = self._clamp(
            self._integral_h[0] + body_dx, -self._integral_max, self._integral_max)
        self._integral_h[1] = self._clamp(
            self._integral_h[1] + body_dy, -self._integral_max, self._integral_max)
        d_error_hx = body_dx - self._prev_error_h[0]
        d_error_hy = body_dy - self._prev_error_h[1]
        self._prev_error_h = [body_dx, body_dy]

        vx = self._clamp(
            self.kp_h * body_dx + self.ki_h * self._integral_h[0] + self.kd_h * d_error_hx,
            -self.max_h_speed, self.max_h_speed)
        vy = self._clamp(
            self.kp_h * body_dy + self.ki_h * self._integral_h[1] + self.kd_h * d_error_hy,
            -self.max_h_speed, self.max_h_speed)
        cmd.twist.linear.x = vx
        cmd.twist.linear.y = vy

        # --- Yaw 제어 (이동 방향으로 기수 정렬) ---
        if h_dist > self.pos_tol:
            target_yaw = math.atan2(dy, dx)
            yaw_error = target_yaw - yaw
            # -pi ~ pi 정규화
            while yaw_error > math.pi:
                yaw_error -= 2.0 * math.pi
            while yaw_error < -math.pi:
                yaw_error += 2.0 * math.pi
            cmd.twist.angular.z = self._clamp(
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
            # 착륙 시작 시각 기록
            if self._landing_start_time is None:
                self._landing_start_time = self.get_clock().now()
            # 착륙 조건: 고도 0.3m 이하 AND 수직 오차 < 0.4m AND 수직 속도 충분히 작음
            landing_elapsed = (self.get_clock().now() - self._landing_start_time).nanoseconds / 1e9
            vz = abs(self.current_pose[2] - z) / max(dt, 0.001) if hasattr(self, '_prev_z') else 0.0
            self._prev_z = z
            if (z < 0.3 and abs(dz) < 0.4 and vz < 0.3) or landing_elapsed > self.landing_timeout:
                if landing_elapsed > self.landing_timeout:
                    self.get_logger().warn(
                        f'Landing timeout ({self.landing_timeout}s) — forced ground')
                self.state = 'grounded'
                self.target_waypoint = None
                self._landing_start_time = None
                self.get_logger().info('Landed')
                self._publish_stop()

        self._publish_state()

    # ─────────────────── Helpers ───────────────────

    def _make_cmd(self) -> TwistStamped:
        """TwistStamped 메시지 생성 (Jazzy 표준)."""
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        return cmd

    def _publish_stop(self):
        """정지 명령 발행."""
        cmd = self._make_cmd()
        self.cmd_pub.publish(cmd)

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
