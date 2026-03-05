"""
ARGOS Robot Status Publisher
=============================
개별 로봇의 상태를 수집하여 오케스트레이터에 주기적으로 보고.

UGV와 드론 모두 지원:
- UGV: TF2(map→base_footprint) 위치, exploration/status 상태, map 커버리지
- 드론: odom 직접 구독 위치, drone/state 상태 (TF 미사용)

토픽:
  구독: exploration/status (String, from frontier_explorer — UGV)
        drone/state (String, from drone_controller — 드론)
        thermal/detections (ThermalDetection, from hotspot_detector)
        odom (Odometry — 드론 위치 폴백)
  발행: /orchestrator/robot_status (RobotStatus)
        /orchestrator/fire_alert (FireAlert, 화점 감지 시)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, LivelinessPolicy
from rclpy.duration import Duration

import numpy as np

from std_msgs.msg import String, UInt32
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from my_robot_interfaces.msg import RobotStatus, FireAlert, ThermalDetection

from tf2_ros import Buffer, TransformListener


class RobotStatusPublisher(Node):

    REPORT_RATE = 2.0  # Hz

    def __init__(self):
        super().__init__('robot_status_publisher')

        # --- Parameters ---
        self.declare_parameter('robot_id', '')
        self.declare_parameter('robot_type', 'ugv')
        self.declare_parameter('capabilities', ['thermal', 'lidar', 'depth', 'imu'])
        self.declare_parameter('battery_drain_rate', 0.02)  # %/초 (시뮬레이션 감쇠)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')

        self.robot_id = self.get_parameter('robot_id').value
        if not self.robot_id:
            self.get_logger().error(
                'robot_id 파라미터가 비어있음! launch 파일 확인 필요')
        self.robot_type = self.get_parameter('robot_type').value
        self.capabilities = self.get_parameter('capabilities').value
        self.drain_rate = self.get_parameter('battery_drain_rate').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # --- State ---
        self.exploration_status = 'idle'
        self.battery = 100.0
        self.battery_start_time = self.get_clock().now()
        self.coverage_percent = 0.0
        self.odom_pose = None  # 드론 odom 폴백용
        self.frontiers_remaining = 0
        self.nav_error_count = 0

        # --- TF2 (UGV 전용, 드론은 odom 직접 사용) ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- QoS ---
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10,
        )

        # --- Subscribers ---
        self.status_sub = self.create_subscription(
            String, 'exploration/status',
            self.exploration_status_callback, 10)

        self.thermal_sub = self.create_subscription(
            ThermalDetection, 'thermal/detections',
            self.thermal_callback, 10)

        # SLAM toolbox는 TRANSIENT_LOCAL로 map 발행 → 구독도 일치시켜야
        # 노드 늦게 시작해도 마지막 map을 수신할 수 있음
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, map_qos)

        # UGV: frontier_explorer에서 탐색 통계 수신
        self.frontier_count_sub = self.create_subscription(
            UInt32, 'exploration/frontier_count',
            self.frontier_count_callback, 10)
        self.nav_error_sub = self.create_subscription(
            UInt32, 'exploration/nav_error_count',
            self.nav_error_callback, 10)

        # 드론: drone/state 구독 + odom 직접 구독
        if self.robot_type == 'drone':
            self.drone_state_sub = self.create_subscription(
                String, 'drone/state',
                self.drone_state_callback, 10)
            self.odom_sub = self.create_subscription(
                Odometry, 'odom',
                self.odom_callback, 10)

        # --- Publishers (Deadline QoS: .msg 문서와 일치하는 5초 보장) ---
        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10,
            deadline=Duration(seconds=5),
        )
        self.status_pub = self.create_publisher(
            RobotStatus, '/orchestrator/robot_status', status_qos)

        self.fire_pub = self.create_publisher(
            FireAlert, '/orchestrator/fire_alert', reliable_qos)

        # --- Timer ---
        self.report_timer = self.create_timer(
            1.0 / self.REPORT_RATE, self.publish_status)

        self.get_logger().info(
            f'RobotStatus publisher ready: {self.robot_id} ({self.robot_type})')

    def exploration_status_callback(self, msg: String):
        self.exploration_status = msg.data

    def frontier_count_callback(self, msg: UInt32):
        """프론티어 탐색기에서 남은 프론티어 수 수신."""
        self.frontiers_remaining = msg.data

    def nav_error_callback(self, msg: UInt32):
        """Nav2 실패 횟수 수신."""
        self.nav_error_count = msg.data

    def drone_state_callback(self, msg: String):
        """드론 상태 수신 (grounded/taking_off/hovering/flying/landing)."""
        self.exploration_status = msg.data

    def odom_callback(self, msg: Odometry):
        """드론 odom → PoseStamped 변환 캐시."""
        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = 'odom'
        pose.pose = msg.pose.pose
        self.odom_pose = pose

    def map_callback(self, msg: OccupancyGrid):
        """맵 데이터로 탐색 커버리지 계산 (free / (free+unknown))."""
        data = np.array(msg.data, dtype=np.int8)
        free = int(np.sum(data == 0))
        unknown = int(np.sum(data == -1))
        total = free + unknown
        if total > 0:
            self.coverage_percent = (free / total) * 100.0

    def thermal_callback(self, msg: ThermalDetection):
        """high/critical 감지 시 FireAlert 발행."""
        if msg.severity in ('high', 'critical'):
            alert = FireAlert()
            alert.header.stamp = self.get_clock().now().to_msg()
            alert.robot_id = self.robot_id

            # 로봇 현재 위치를 화점 위치로 사용 (정밀 위치는 향후 개선)
            pose = self._get_robot_pose()
            if pose:
                alert.location.header = pose.header
                alert.location.point.x = pose.pose.position.x
                alert.location.point.y = pose.pose.position.y
                alert.location.point.z = 0.0

            alert.max_temperature_kelvin = msg.max_temperature_kelvin
            alert.severity = msg.severity
            alert.confidence = msg.confidence
            alert.active = True

            self.fire_pub.publish(alert)

    def publish_status(self):
        """주기적 상태 발행."""
        msg = RobotStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.robot_id = self.robot_id
        msg.robot_type = self.robot_type

        # 상태 매핑 (UGV + 드론 공통)
        status_map = {
            # UGV 상태
            'idle': RobotStatus.STATE_IDLE,
            'exploring': RobotStatus.STATE_EXPLORING,
            'navigating': RobotStatus.STATE_EXPLORING,
            'paused_thermal': RobotStatus.STATE_ON_MISSION,
            'complete': RobotStatus.STATE_IDLE,
            # 드론 상태
            'grounded': RobotStatus.STATE_IDLE,
            'taking_off': RobotStatus.STATE_ON_MISSION,
            'hovering': RobotStatus.STATE_EXPLORING,
            'flying': RobotStatus.STATE_EXPLORING,
            'landing': RobotStatus.STATE_ON_MISSION,
        }
        msg.state = status_map.get(
            self.exploration_status, RobotStatus.STATE_IDLE)

        # 위치
        pose = self._get_robot_pose()
        if pose:
            msg.pose = pose

        # 배터리: 시간 기반 선형 감쇠
        elapsed = (self.get_clock().now() - self.battery_start_time).nanoseconds / 1e9
        self.battery = max(0.0, 100.0 - self.drain_rate * elapsed)
        msg.battery_percent = self.battery
        msg.current_mission = self.exploration_status
        msg.coverage_percent = self.coverage_percent
        msg.frontiers_remaining = self.frontiers_remaining
        msg.mission_progress = min(self.coverage_percent / 100.0, 1.0)
        msg.nav_error_count = self.nav_error_count
        msg.capabilities = self.capabilities

        self.status_pub.publish(msg)

    def _get_robot_pose(self):
        """로봇 위치 조회. UGV: TF2(map→base_footprint), 드론: odom 직접 사용."""
        # 드론은 SLAM을 사용하지 않으므로 odom 직접 사용
        if self.robot_type == 'drone':
            return self.odom_pose

        # UGV: TF2 변환 (SLAM이 map 프레임 제공)
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
            pose = PoseStamped()
            pose.header = t.header
            pose.pose.position.x = t.transform.translation.x
            pose.pose.position.y = t.transform.translation.y
            pose.pose.position.z = t.transform.translation.z
            pose.pose.orientation = t.transform.rotation
            return pose
        except Exception:
            return None


def main(args=None):
    rclpy.init(args=args)
    node = RobotStatusPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
