#!/usr/bin/env python3
"""step_detector_node.py — ARGOS 계단/단차 감지 노드

Nav 전문가 권고: depth 카메라로 바닥 높이 급변을 감지하여
계단/단차/구덩이를 탐지. LiDAR(2D)로는 감지 불가능한 수직 위험.

원리:
  1. depth 이미지 구독 (RealSense D435 등)
  2. 하단 영역(바닥) 깊이값의 gradient 계산
  3. gradient가 임계값 초과 → 계단/단차 경고
  4. Nav2 costmap에 keepout 반영 (향후)
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


class StepDetectorNode(Node):
    """depth gradient 기반 계단/단차 감지."""

    def __init__(self):
        super().__init__('step_detector')

        self.declare_parameter('robot_id', 'argos_1')
        self.declare_parameter('gradient_threshold', 0.3)  # m — 30cm 이상 급변 시 경고
        self.declare_parameter('scan_row_ratio', 0.8)       # 이미지 하단 80% 위치 스캔
        self.declare_parameter('min_consecutive', 5)         # 최소 연속 픽셀

        self.threshold = self.get_parameter('gradient_threshold').value
        self.scan_ratio = self.get_parameter('scan_row_ratio').value
        self.min_consecutive = self.get_parameter('min_consecutive').value

        # 구독
        self.depth_sub = self.create_subscription(
            Image, 'depth/image_raw', self._depth_cb, 10)

        # 발행
        self.step_pub = self.create_publisher(
            Bool, 'step/detected', 10)

        self.get_logger().info(
            f'Step detector initialized (threshold={self.threshold}m)')

    def _depth_cb(self, msg):
        """depth 이미지에서 바닥 높이 급변 감지."""
        try:
            # depth 이미지 파싱
            if '16UC1' in msg.encoding or 'mono16' in msg.encoding:
                depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(
                    msg.height, msg.width).astype(np.float32) / 1000.0  # mm → m
            elif '32FC1' in msg.encoding:
                depth = np.frombuffer(msg.data, dtype=np.float32).reshape(
                    msg.height, msg.width)
            else:
                return

            # 바닥 영역 스캔 (이미지 하단 80% 행)
            scan_row = int(msg.height * self.scan_ratio)
            row = depth[scan_row, :]

            # NaN/Inf 제거
            valid = np.where(np.isfinite(row) & (row > 0.1) & (row < 10.0))[0]
            if len(valid) < self.min_consecutive:
                return

            # gradient 계산 (인접 픽셀 간 깊이 차이)
            valid_depths = row[valid]
            gradient = np.abs(np.diff(valid_depths))

            # 급변 감지
            step_detected = np.any(gradient > self.threshold)

            if step_detected:
                n_steps = int(np.sum(gradient > self.threshold))
                max_step = float(np.max(gradient))
                self.get_logger().warn(
                    f'STEP DETECTED: {n_steps} points, max={max_step:.2f}m',
                    throttle_duration_sec=2.0)

            msg_out = Bool()
            msg_out.data = bool(step_detected)
            self.step_pub.publish(msg_out)

        except Exception as e:
            self.get_logger().error(
                f'Step detection error: {e}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = StepDetectorNode()
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
