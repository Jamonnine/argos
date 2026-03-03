#!/usr/bin/env python3
"""
Temperature Monitor Node (Subscriber)
/temperature 토픽을 구독하여 온도 데이터를 모니터링
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class TemperatureMonitor(Node):
    """온도 모니터링 노드 - Subscriber 패턴 구현"""

    def __init__(self):
        # 노드 이름: 'temperature_monitor'
        super().__init__('temperature_monitor')

        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # Parameters 선언 (기본값 제공)
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # declare_parameter(이름, 기본값)
        self.declare_parameter('threshold_high', 28.0)
        self.declare_parameter('threshold_low', 22.0)

        # Parameter 값 가져오기
        self.THRESHOLD_HIGH = self.get_parameter('threshold_high').value
        self.THRESHOLD_LOW = self.get_parameter('threshold_low').value

        # Subscriber 생성: Float32 타입, /temperature 토픽, QoS 10
        self.subscription = self.create_subscription(
            Float32,
            'temperature',
            self.temperature_callback,  # 메시지 수신 시 호출될 콜백
            10
        )

        # 통계 추적용 상태 변수
        self.min_temp = float('inf')   # 최소 온도 (무한대로 초기화)
        self.max_temp = float('-inf')  # 최대 온도 (-무한대로 초기화)
        self.temp_sum = 0.0            # 온도 합계 (평균 계산용)
        self.count = 0                 # 수신 횟수

        self.get_logger().info('🔍 Temperature Monitor Started!')
        self.get_logger().info(f'   High Threshold: {self.THRESHOLD_HIGH}°C (from parameter)')
        self.get_logger().info(f'   Low Threshold: {self.THRESHOLD_LOW}°C (from parameter)')

    def temperature_callback(self, msg):
        """
        메시지 수신 시 자동으로 호출되는 콜백 함수

        Args:
            msg (Float32): 수신한 온도 데이터
        """
        temp = msg.data
        self.count += 1

        # 통계 업데이트
        self.temp_sum += temp
        if temp < self.min_temp:
            self.min_temp = temp
        if temp > self.max_temp:
            self.max_temp = temp

        # 평균 온도 계산
        avg_temp = self.temp_sum / self.count

        # 기본 로그 출력
        self.get_logger().info(
            f'📥 Received: {temp:.2f}°C | '
            f'Avg: {avg_temp:.2f}°C | '
            f'Min: {self.min_temp:.2f}°C | '
            f'Max: {self.max_temp:.2f}°C | '
            f'(#{self.count})'
        )

        # 임계값 체크 (경고 시스템)
        if temp > self.THRESHOLD_HIGH:
            self.get_logger().warn(
                f'⚠️  HIGH TEMPERATURE WARNING: {temp:.2f}°C '
                f'(Threshold: {self.THRESHOLD_HIGH}°C)'
            )
        elif temp < self.THRESHOLD_LOW:
            self.get_logger().warn(
                f'⚠️  LOW TEMPERATURE WARNING: {temp:.2f}°C '
                f'(Threshold: {self.THRESHOLD_LOW}°C)'
            )


def main(args=None):
    """노드 실행 엔트리포인트"""
    rclpy.init(args=args)
    node = TemperatureMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Shutting down...')
    finally:
        # 최종 통계 출력
        if node.count > 0:
            final_avg = node.temp_sum / node.count
            node.get_logger().info('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
            node.get_logger().info('📊 Final Statistics:')
            node.get_logger().info(f'   Total Readings: {node.count}')
            node.get_logger().info(f'   Average: {final_avg:.2f}°C')
            node.get_logger().info(f'   Min: {node.min_temp:.2f}°C')
            node.get_logger().info(f'   Max: {node.max_temp:.2f}°C')
            node.get_logger().info('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
