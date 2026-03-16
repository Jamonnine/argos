#!/usr/bin/env python3
"""smoke_effect_node.py — ARGOS 연기 시뮬레이션 (카메라 후처리)

시뮬레이션 전문가 권고: Gazebo 파티클 대신 카메라 이미지 후처리로 연기 효과.
화원 거리에 따라 가시거리가 줄어드는 효과를 시뮬레이션.

원리:
  1. RGB/열화상 이미지 구독
  2. 화원 거리 기반 연기 밀도 계산
  3. Gaussian blur + 회색 alpha blending 적용
  4. 연기 적용된 이미지 재발행

소방 현장 현실:
  - 연기 속 가시거리: 30cm ~ 3m
  - LiDAR: 연기 입자에 의한 산란 (false positive)
  - 카메라: 대비 저하 + 색상 왜곡
"""
import cv2
import numpy as np
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry


class SmokeEffectNode(Node):
    """카메라 이미지에 연기 효과를 적용하는 후처리 노드."""

    def __init__(self):
        super().__init__('smoke_effect')

        self.declare_parameter('robot_id', 'argos_1')
        self.declare_parameter('enabled', True)
        self.declare_parameter('max_density', 0.8)        # 최대 연기 밀도 (0~1)
        self.declare_parameter('smoke_color_gray', 180)   # 연기 색상 (0=검정, 255=흰색)
        self.declare_parameter('blur_kernel', 21)         # Gaussian blur 커널
        self.declare_parameter('fire_influence_range', 10.0)  # 화원으로부터 연기 영향 범위 (m)

        self.enabled = self.get_parameter('enabled').value
        self.max_density = self.get_parameter('max_density').value
        self.smoke_gray = self.get_parameter('smoke_color_gray').value
        self.blur_k = self.get_parameter('blur_kernel').value
        self.influence_range = self.get_parameter('fire_influence_range').value

        self.robot_position = None
        self.fire_positions = []  # (x, y) 화원 위치

        # 구독
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self._image_cb, 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self._odom_cb, 10)

        # 발행 (연기 적용된 이미지)
        self.smoke_pub = self.create_publisher(
            Image, 'camera/image_smoke', 10)

        # 시뮬용 화원 위치 파라미터
        self.declare_parameter('fire_positions', [])
        raw = self.get_parameter('fire_positions').value
        if raw:
            for i in range(0, len(raw) - 1, 2):
                try:
                    self.fire_positions.append((float(raw[i]), float(raw[i+1])))
                except (ValueError, IndexError):
                    break

        self.get_logger().info(
            f'Smoke effect initialized (enabled={self.enabled}, '
            f'max_density={self.max_density}, fires={len(self.fire_positions)})')

    def _odom_cb(self, msg):
        self.robot_position = msg.pose.pose.position

    def _compute_smoke_density(self) -> float:
        """로봇-화원 거리 기반 연기 밀도 계산.

        Returns:
            0.0 (맑음) ~ max_density (짙은 연기)
        """
        if self.robot_position is None or not self.fire_positions:
            return 0.0

        min_dist = float('inf')
        for fx, fy in self.fire_positions:
            dist = math.sqrt(
                (self.robot_position.x - fx)**2 +
                (self.robot_position.y - fy)**2)
            if dist < min_dist:
                min_dist = dist

        if min_dist >= self.influence_range:
            return 0.0

        # 가까울수록 짙은 연기 (지수 감쇠)
        density = self.max_density * math.exp(-0.3 * min_dist)
        return min(self.max_density, density)

    def _apply_smoke(self, frame: np.ndarray, density: float) -> np.ndarray:
        """프레임에 연기 효과 적용.

        1. 회색 연기 alpha blending
        2. Gaussian blur (가시거리 저하)
        """
        if density < 0.01:
            return frame

        # 연기 색상 레이어
        smoke_layer = np.full_like(frame, self.smoke_gray)

        # Alpha blending
        blended = cv2.addWeighted(
            frame, 1.0 - density,
            smoke_layer, density,
            0)

        # Gaussian blur (연기 밀도에 비례)
        k = max(3, int(self.blur_k * density))
        if k % 2 == 0:
            k += 1
        blurred = cv2.GaussianBlur(blended, (k, k), 0)

        return blurred

    def _image_cb(self, msg):
        if not self.enabled:
            return

        density = self._compute_smoke_density()
        if density < 0.01:
            return  # 연기 없으면 원본 유지 (재발행 불필요)

        try:
            # ROS Image → numpy (간단 변환, cv_bridge 미사용)
            if msg.encoding in ('bgr8', 'rgb8'):
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3)
            elif msg.encoding == 'mono8':
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width)
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            else:
                return

            # 연기 적용
            smoked = self._apply_smoke(frame, density)

            # numpy → ROS Image
            out_msg = Image()
            out_msg.header = msg.header
            out_msg.height = smoked.shape[0]
            out_msg.width = smoked.shape[1]
            out_msg.encoding = 'bgr8'
            out_msg.step = smoked.shape[1] * 3
            out_msg.data = smoked.tobytes()

            self.smoke_pub.publish(out_msg)

        except Exception as e:
            self.get_logger().error(
                f'Smoke effect error: {e}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = SmokeEffectNode()
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
