"""
ARGOS Robot Status Publisher
=============================
개별 로봇의 상태를 수집하여 오케스트레이터에 주기적으로 보고.

이 노드는 frontier_explorer, hotspot_detector 등의 하위 노드에서
상태 정보를 수집하고, RobotStatus 메시지로 통합하여 발행한다.

토픽:
  구독: exploration/status (String, from frontier_explorer)
        thermal/detections (ThermalDetection, from hotspot_detector)
  발행: /orchestrator/robot_status (RobotStatus)
        /orchestrator/fire_alert (FireAlert, 화점 감지 시)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PointStamped
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
        self.declare_parameter('use_sim_time', True)

        self.robot_id = self.get_parameter('robot_id').value
        self.robot_type = self.get_parameter('robot_type').value
        self.capabilities = self.get_parameter('capabilities').value

        # --- State ---
        self.exploration_status = 'idle'
        self.battery = 100.0  # 시뮬레이션에서는 고정

        # --- TF2 ---
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

        # --- Publishers ---
        self.status_pub = self.create_publisher(
            RobotStatus, '/orchestrator/robot_status', 10)

        self.fire_pub = self.create_publisher(
            FireAlert, '/orchestrator/fire_alert', reliable_qos)

        # --- Timer ---
        self.report_timer = self.create_timer(
            1.0 / self.REPORT_RATE, self.publish_status)

        self.get_logger().info(
            f'RobotStatus publisher ready: {self.robot_id} ({self.robot_type})')

    def exploration_status_callback(self, msg: String):
        self.exploration_status = msg.data

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

        # 상태 매핑
        status_map = {
            'idle': RobotStatus.STATE_IDLE,
            'exploring': RobotStatus.STATE_EXPLORING,
            'navigating': RobotStatus.STATE_EXPLORING,
            'paused_thermal': RobotStatus.STATE_ON_MISSION,
            'complete': RobotStatus.STATE_IDLE,
        }
        msg.state = status_map.get(
            self.exploration_status, RobotStatus.STATE_EXPLORING)

        # 위치
        pose = self._get_robot_pose()
        if pose:
            msg.pose = pose

        msg.battery_percent = self.battery
        msg.current_mission = self.exploration_status
        msg.capabilities = self.capabilities

        self.status_pub.publish(msg)

    def _get_robot_pose(self):
        """TF2로 맵 내 로봇 위치 조회."""
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
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
