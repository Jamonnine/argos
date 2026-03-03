#!/usr/bin/env python3
"""
카메라 Publisher 노드
테스트용 이미지를 주기적으로 발행 (실제 카메라 시뮬레이션)

설계 원칙:
1. 센서 추상화: 실제 카메라든 시뮬레이션이든 같은 인터페이스
2. 표준 토픽: /camera/image_raw (ROS REP 표준)
3. 30Hz: 일반적인 카메라 프레임 레이트
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        # Publisher 생성 (표준 토픽 이름)
        self.publisher_ = self.create_publisher(
            Image,
            '/camera/image_raw',
            10  # QoS: 최근 10개 메시지 버퍼
        )

        # cv_bridge 초기화 (OpenCV ↔ ROS 변환기)
        self.bridge = CvBridge()

        # 타이머: 30Hz로 이미지 발행 (1/30 = 0.0333초마다)
        # 실제 카메라 시뮬레이션
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

        self.get_logger().info('=' * 50)
        self.get_logger().info('카메라 Publisher 시작')
        self.get_logger().info('  - 프레임 레이트: 30Hz')
        self.get_logger().info('  - 토픽: /camera/image_raw')
        self.get_logger().info('  - 인코딩: bgr8')
        self.get_logger().info('=' * 50)

        # 프레임 카운터
        self.frame_count = 0

    def timer_callback(self):
        """30Hz로 호출되는 타이머 콜백"""
        # 테스트용 이미지 생성
        image = self.generate_test_image()

        # OpenCV Mat → ROS Image 메시지
        try:
            ros_image = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')

            # Timestamp 설정 (매우 중요!)
            # 센서 융합할 때 시간 동기화 필수
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = 'camera_frame'

            # 발행
            self.publisher_.publish(ros_image)

            self.frame_count += 1

            # 1초마다 로그 (30프레임마다)
            if self.frame_count % 30 == 0:
                self.get_logger().info(
                    f'프레임 {self.frame_count} 발행 '
                    f'({self.frame_count // 30}초 경과)'
                )

        except Exception as e:
            self.get_logger().error(f'이미지 변환 실패: {e}')

    def generate_test_image(self):
        """
        테스트용 이미지 생성
        - 움직이는 빨간 공 (좌우 진동)
        - 고정된 파란 사각형
        """
        # 640x480 검은 배경
        image = np.zeros((480, 640, 3), dtype=np.uint8)

        # 시간에 따라 움직이는 빨간 공
        # 사인파 운동: x = 320 + 200 * sin(t)
        t = self.frame_count / 30.0  # 초 단위 시간
        x = int(320 + 200 * np.sin(t * 2))  # 약 120 ~ 520 범위
        y = 240  # 중앙 고정

        # 빨간 원 그리기 (BGR 순서!)
        cv2.circle(image, (x, y), 50, (0, 0, 255), -1)

        # 파란 사각형 (고정, 왼쪽 하단)
        cv2.rectangle(image, (50, 400), (150, 470), (255, 0, 0), -1)

        # 프레임 번호 표시 (디버깅용)
        cv2.putText(
            image,
            f'Frame: {self.frame_count}',
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )

        # 타임스탬프 표시
        cv2.putText(
            image,
            f'Time: {t:.2f}s',
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )

        return image


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C 감지, 종료합니다...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
