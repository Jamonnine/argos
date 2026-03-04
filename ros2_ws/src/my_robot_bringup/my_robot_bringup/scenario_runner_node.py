"""
ARGOS 통합 시연 시나리오 러너
================================
소방 작전 시뮬레이션을 자동 실행하는 시나리오 스크립트.

흐름 (7단계):
  0. WAIT_READY   — 모든 로봇 오케스트레이터 등록 대기
  1. DRONE_TAKEOFF — 드론 이륙 명령 + 정찰 웨이포인트 전송
  2. EXPLORING     — UGV 프론티어 탐색 + 드론 정찰 모니터링
  3. FIRE_DETECTED — 화점 감지 대응 관찰 (또는 시뮬레이션)
  4. EMERGENCY     — 긴급 정지 시연
  5. RESUME        — 작전 재개 시연
  6. LANDING       — 드론 착륙 + 시나리오 완료

사용법:
  ros2 launch argos_description demo.launch.py
  ros2 launch argos_description demo.launch.py simulate_fire:=false
"""

import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from my_robot_interfaces.msg import MissionState, ThermalDetection


class ScenarioRunner(Node):

    # 시나리오 단계
    WAIT_READY = 0
    DRONE_TAKEOFF = 1
    EXPLORING = 2
    FIRE_DETECTED = 3
    EMERGENCY = 4
    RESUME = 5
    LANDING = 6
    COMPLETE = 7

    PHASE_NAMES = [
        'WAIT_READY', 'DRONE_TAKEOFF', 'EXPLORING', 'FIRE_DETECTED',
        'EMERGENCY', 'RESUME', 'LANDING', 'COMPLETE',
    ]

    def __init__(self):
        super().__init__('scenario_runner')

        # --- Parameters ---
        self.declare_parameter('simulate_fire', True)
        self.declare_parameter('fire_delay_sec', 30.0)
        self.declare_parameter('emergency_delay_sec', 45.0)
        self.declare_parameter('drone_name', 'drone1')
        self.declare_parameter('target_ugv', 'argos1')
        self.declare_parameter('use_sim_time', True)

        self.simulate_fire = self.get_parameter('simulate_fire').value
        self.fire_delay = self.get_parameter('fire_delay_sec').value
        self.emergency_delay = self.get_parameter('emergency_delay_sec').value
        self.drone_name = self.get_parameter('drone_name').value
        self.target_ugv = self.get_parameter('target_ugv').value

        # --- State ---
        self.phase = self.WAIT_READY
        self.start_time = None
        self.mission_stage = MissionState.STAGE_INIT

        # --- Subscribers ---
        self.mission_sub = self.create_subscription(
            MissionState, '/orchestrator/mission_state',
            self.mission_state_callback, 10)

        # --- Publishers ---
        self.drone_wp_pub = self.create_publisher(
            PoseStamped, f'/{self.drone_name}/drone/waypoint', 10)

        if self.simulate_fire:
            self.thermal_pub = self.create_publisher(
                ThermalDetection,
                f'/{self.target_ugv}/thermal/detections', 10)

        # --- Service Clients ---
        self.takeoff_client = self.create_client(
            Trigger, f'/{self.drone_name}/drone/takeoff')
        self.land_client = self.create_client(
            Trigger, f'/{self.drone_name}/drone/land')
        self.estop_client = self.create_client(
            Trigger, '/orchestrator/emergency_stop')
        self.resume_client = self.create_client(
            Trigger, '/orchestrator/resume')

        # --- Main Loop (1 Hz) ---
        self.timer = self.create_timer(1.0, self.scenario_tick)

        self.get_logger().info('=' * 55)
        self.get_logger().info('  ARGOS Fire Response Scenario Runner')
        self.get_logger().info(f'  simulate_fire={self.simulate_fire}  '
                               f'fire_delay={self.fire_delay}s')
        self.get_logger().info('=' * 55)
        self.get_logger().info('[Phase 0] Waiting for all robots to register...')

    # ─────────────── Callbacks ───────────────

    def mission_state_callback(self, msg: MissionState):
        self.mission_stage = msg.stage

    # ─────────────── Main Loop ───────────────

    def scenario_tick(self):
        now = self.get_clock().now()

        # --- Phase 0: 로봇 등록 대기 ---
        if self.phase == self.WAIT_READY:
            if self.mission_stage >= MissionState.STAGE_EXPLORING:
                self.start_time = now
                self._advance(self.DRONE_TAKEOFF)
                self.get_logger().info(
                    'All robots registered. Commanding drone takeoff...')
                self._call_service(self.takeoff_client, 'Drone takeoff')

        # --- Phase 1: 드론 이륙 + 웨이포인트 ---
        elif self.phase == self.DRONE_TAKEOFF:
            if self._elapsed(now) > 8.0:
                self._advance(self.EXPLORING)
                self.get_logger().info(
                    'Sending drone patrol waypoints...')
                self._send_drone_waypoints()

        # --- Phase 2: 탐색 모니터링 ---
        elif self.phase == self.EXPLORING:
            elapsed = self._elapsed(now)

            # 실제 화재 감지 (오케스트레이터가 전환)
            if self.mission_stage == MissionState.STAGE_FIRE_RESPONSE:
                self._advance(self.FIRE_DETECTED)
                self.get_logger().info(
                    'FIRE DETECTED by sensor pipeline!')

            # 시뮬레이션 화재 (타이머 기반)
            elif self.simulate_fire and elapsed > self.fire_delay:
                self.get_logger().info(
                    f'Simulating fire detection at {elapsed:.0f}s...')
                self._simulate_fire()
                self._advance(self.FIRE_DETECTED)

        # --- Phase 3: 화재 대응 관찰 ---
        elif self.phase == self.FIRE_DETECTED:
            if self._elapsed(now) > self.emergency_delay:
                self._advance(self.EMERGENCY)
                self.get_logger().info(
                    'EMERGENCY STOP: All units return!')
                self._call_service(self.estop_client, 'Emergency stop')

        # --- Phase 4: 긴급 정지 10초 유지 ---
        elif self.phase == self.EMERGENCY:
            if self._elapsed(now) > self.emergency_delay + 10.0:
                self._advance(self.RESUME)
                self.get_logger().info(
                    'Resuming operations...')
                self._call_service(self.resume_client, 'Resume')

        # --- Phase 5: 재개 후 10초 대기 ---
        elif self.phase == self.RESUME:
            if self._elapsed(now) > self.emergency_delay + 20.0:
                self._advance(self.LANDING)
                self.get_logger().info(
                    'Mission complete. Landing drone...')
                self._call_service(self.land_client, 'Drone land')

        # --- Phase 6: 착륙 대기 ---
        elif self.phase == self.LANDING:
            if self._elapsed(now) > self.emergency_delay + 30.0:
                self._advance(self.COMPLETE)
                self.get_logger().info('=' * 55)
                self.get_logger().info(
                    '  SCENARIO COMPLETE  '
                    f'(total {self._elapsed(now):.0f}s)')
                self.get_logger().info('=' * 55)
                self.timer.cancel()

    # ─────────────── Helpers ───────────────

    def _elapsed(self, now):
        if self.start_time is None:
            return 0.0
        return (now - self.start_time).nanoseconds / 1e9

    def _advance(self, new_phase):
        self.get_logger().info(
            f'[Phase {new_phase}] {self.PHASE_NAMES[new_phase]}')
        self.phase = new_phase

    def _call_service(self, client, name):
        if not client.service_is_ready():
            self.get_logger().warn(f'{name}: service not available')
            return
        req = Trigger.Request()
        future = client.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f'{name}: {f.result().message}'
                if f.result() else f'{name}: call failed'))

    def _send_drone_waypoints(self):
        """건물 상공 정찰 웨이포인트 (방 A → 방 B → 방 C → 중앙)."""
        waypoints = [
            (1.5, 6.5, 8.0),   # 방 A 상공 (화원 위치)
            (8.5, 6.5, 8.0),   # 방 B 상공
            (5.0, 2.5, 8.0),   # 방 C 상공
            (5.0, 5.0, 8.0),   # 복도 상공
        ]
        for i, (wx, wy, wz) in enumerate(waypoints):
            wp = PoseStamped()
            wp.header.stamp = self.get_clock().now().to_msg()
            wp.header.frame_id = 'map'
            wp.pose.position.x = wx
            wp.pose.position.y = wy
            wp.pose.position.z = wz
            self.drone_wp_pub.publish(wp)
            self.get_logger().info(
                f'  WP{i}: ({wx:.1f}, {wy:.1f}, {wz:.1f})')

    def _simulate_fire(self):
        """가짜 화재 감지 메시지를 UGV 열화상 토픽에 발행."""
        det = ThermalDetection()
        det.header.stamp = self.get_clock().now().to_msg()
        det.max_temperature_kelvin = 573.15   # 300°C → severity=high
        det.mean_temperature_kelvin = 473.15
        det.confidence = 0.95
        det.severity = 'high'
        det.area_ratio = 0.15
        self.thermal_pub.publish(det)
        self.get_logger().info(
            '  Published simulated thermal detection '
            '(300C, severity=high)')


def main(args=None):
    rclpy.init(args=args)
    node = ScenarioRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
