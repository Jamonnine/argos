#!/usr/bin/env python3
"""structural_monitor_node.py — ARGOS 구조물 안전성 모니터링 노드

LiDAR 포인트클라우드를 시간에 따라 비교하여
바닥 함몰, 천장 처짐, 벽체 변형 등 구조물 변화를 감지.

원리: 정적 환경의 포인트클라우드는 시간이 지나도 동일해야 함.
      변화가 감지되면 → 구조물 손상 가능성 경고.
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from argos_interfaces.msg import StructuralAlert
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class StructuralMonitorNode(Node):
    """LiDAR 기반 구조물 안전성 모니터링."""

    def __init__(self):
        super().__init__('structural_monitor')

        self.declare_parameter('robot_id', 'argos_1')
        self.declare_parameter('check_interval', 5.0)  # 초
        self.declare_parameter('displacement_threshold', 0.3)  # 미터
        self.declare_parameter('scan_buffer_size', 10)  # 비교용 스캔 버퍼
        self.declare_parameter('min_change_points', 5)  # 최소 변화 포인트

        self.robot_id = self.get_parameter('robot_id').value
        self.threshold = self.get_parameter('displacement_threshold').value
        self.buffer_size = self.get_parameter('scan_buffer_size').value
        self.min_points = self.get_parameter('min_change_points').value

        # 스캔 버퍼
        self.scan_buffer = []
        self.robot_position = None

        # 구독
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self._scan_cb, 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self._odom_cb, 10)

        # 발행
        self.alert_pub = self.create_publisher(
            StructuralAlert, 'structural/alerts', 10)

        # 주기적 점검
        interval = self.get_parameter('check_interval').value
        self.timer = self.create_timer(interval, self._check_structural)

        self.get_logger().info(
            f'[{self.robot_id}] Structural monitor initialized '
            f'(threshold={self.threshold}m, interval={interval}s)')

    def _odom_cb(self, msg):
        self.robot_position = msg.pose.pose.position

    def _scan_cb(self, msg):
        """LiDAR 스캔을 버퍼에 저장."""
        ranges = np.array(msg.ranges, dtype=np.float32)
        # inf/nan을 max_range로 대체
        ranges = np.where(np.isfinite(ranges), ranges, msg.range_max)
        self.scan_buffer.append(ranges)
        if len(self.scan_buffer) > self.buffer_size:
            self.scan_buffer.pop(0)

    def _check_structural(self):
        """버퍼의 첫 스캔과 마지막 스캔을 비교하여 변화 감지."""
        if len(self.scan_buffer) < 2 or self.robot_position is None:
            return

        baseline = self.scan_buffer[0]
        current = self.scan_buffer[-1]

        # 길이 맞추기
        min_len = min(len(baseline), len(current))
        baseline = baseline[:min_len]
        current = current[:min_len]

        # 노이즈 방지: 버퍼 충분할 때 중앙값 기반 비교 (v2 개선)
        if len(self.scan_buffer) >= 6:
            half = len(self.scan_buffer) // 2
            baseline = np.median(self.scan_buffer[:half], axis=0)[:min_len]
            current = np.median(self.scan_buffer[half:], axis=0)[:min_len]

        # 차이 계산
        diff = np.abs(current - baseline)

        # 유의미한 변화 포인트 수
        changed_mask = diff > self.threshold
        n_changed = int(np.sum(changed_mask))

        if n_changed >= self.min_points:
            # 최대 변위
            max_disp = float(np.max(diff[changed_mask]))

            # 심각도 판정
            if max_disp > 1.0:
                severity = 'critical'
                alert_type = 'floor_collapse'
            elif max_disp > 0.5:
                severity = 'danger'
                alert_type = 'ceiling_sag'
            else:
                severity = 'warning'
                alert_type = 'structural_shift'

            # 경고 발행
            alert = StructuralAlert()
            alert.header.stamp = self.get_clock().now().to_msg()
            alert.header.frame_id = 'map'
            alert.robot_id = self.robot_id
            alert.alert_type = alert_type
            alert.severity = severity
            alert.confidence = min(1.0, n_changed / max(1, min_len))

            alert.location = PointStamped()
            alert.location.header = alert.header
            alert.location.point = self.robot_position

            alert.affected_radius_m = 3.0
            alert.displacement_m = max_disp
            alert.change_rate_m_per_s = max_disp / max(1.0,
                self.get_parameter('check_interval').value)

            alert.area_blocked = severity in ('danger', 'critical')
            if severity == 'critical':
                alert.recommended_actions = ['evacuate', 'reroute']
            elif severity == 'danger':
                alert.recommended_actions = ['reroute', 'monitor']
            else:
                alert.recommended_actions = ['monitor']

            self.alert_pub.publish(alert)
            self.get_logger().warn(
                f'STRUCTURAL {severity.upper()}: {alert_type} '
                f'disp={max_disp:.2f}m points={n_changed} '
                f'blocked={alert.area_blocked}')

            # 버퍼 리셋 (새 기준선)
            self.scan_buffer = [current]


def main(args=None):
    rclpy.init(args=args)
    node = StructuralMonitorNode()
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
