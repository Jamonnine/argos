#!/usr/bin/env python3
"""
Data Processor Node - 데이터 처리 및 통계 계산

이 노드는 온도 데이터를 구독하여 통계를 계산하고,
여러 토픽으로 발행합니다. (Fan-Out 패턴의 소비자이자 새로운 발행자)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math
from collections import deque


class DataProcessor(Node):
    """
    슬라이딩 윈도우 방식으로 온도 통계를 계산하는 노드

    아키텍처 패턴:
    - Subscriber: /temperature 구독
    - Publisher: 여러 통계 토픽 발행
    - Stateful: 최근 N개 샘플을 메모리에 유지
    """

    def __init__(self):
        super().__init__('data_processor')

        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # Parameters
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        self.declare_parameter('window_size', 10)
        self.window_size = self.get_parameter('window_size').value

        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # 슬라이딩 윈도우 (최근 N개 샘플만 유지)
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        self.temperature_window = deque(maxlen=self.window_size)

        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # Subscriber: 온도 데이터 수신
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        self.subscription = self.create_subscription(
            Float32,
            'temperature',
            self.temperature_callback,
            10
        )

        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # Publishers: 통계 데이터 발행 (각 통계마다 별도 토픽)
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        self.pub_mean = self.create_publisher(Float32, 'statistics/mean', 10)
        self.pub_std = self.create_publisher(Float32, 'statistics/std_dev', 10)
        self.pub_min = self.create_publisher(Float32, 'statistics/min', 10)
        self.pub_max = self.create_publisher(Float32, 'statistics/max', 10)

        self.get_logger().info('📊 Data Processor Started!')
        self.get_logger().info(f'   Window Size: {self.window_size} samples')

    def temperature_callback(self, msg):
        """온도 데이터 수신 및 통계 계산"""
        temp = msg.data

        # 윈도우에 추가 (자동으로 오래된 것 제거됨)
        self.temperature_window.append(temp)

        # 충분한 샘플이 모였을 때만 통계 계산
        if len(self.temperature_window) >= 2:
            # 통계 계산
            mean = sum(self.temperature_window) / len(self.temperature_window)
            variance = sum((x - mean) ** 2 for x in self.temperature_window) / len(self.temperature_window)
            std_dev = math.sqrt(variance)
            min_val = min(self.temperature_window)
            max_val = max(self.temperature_window)

            # 각 통계를 별도 토픽으로 발행
            self.pub_mean.publish(Float32(data=mean))
            self.pub_std.publish(Float32(data=std_dev))
            self.pub_min.publish(Float32(data=min_val))
            self.pub_max.publish(Float32(data=max_val))

            # 로그 출력
            self.get_logger().info(
                f'📈 Statistics (window={len(self.temperature_window)}): '
                f'Mean={mean:.2f}°C, StdDev={std_dev:.2f}, '
                f'Range=[{min_val:.2f}, {max_val:.2f}]'
            )


def main(args=None):
    rclpy.init(args=args)
    node = DataProcessor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Shutting down Data Processor...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
