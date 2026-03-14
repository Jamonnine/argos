#!/usr/bin/env python3
"""
물체 감지 노드
카메라 이미지를 받아서 빨간 물체를 감지하고 위치를 계산

설계 원칙:
1. 이벤트 기반: 이미지 도착 시 자동 콜백
2. 알고리즘 캡슐화: detect_red_objects()는 재사용 가능
3. 성능 고려: 대규모 처리 시 프레임 스킵 가능
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')

        # Subscriber 생성
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10  # QoS: 최근 10개 메시지 버퍼
        )

        # cv_bridge 초기화
        self.bridge = CvBridge()

        self.get_logger().info('=' * 50)
        self.get_logger().info('물체 감지 노드 시작')
        self.get_logger().info('  - 구독 토픽: /camera/image_raw')
        self.get_logger().info('  - 감지 대상: 빨간색 물체')
        self.get_logger().info('  - 알고리즘: HSV + 윤곽선')
        self.get_logger().info('=' * 50)

        # 처리한 프레임 수
        self.processed_frames = 0

        # 성능 측정
        self.last_log_time = self.get_clock().now()

    def image_callback(self, msg):
        """
        이미지 메시지를 받을 때마다 호출
        이벤트 기반 처리: Polling 없음!
        """
        try:
            # ROS Image → OpenCV Mat 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # 물체 감지 수행
            detections = self.detect_red_objects(cv_image)

            # 결과 처리
            self.process_detections(detections, msg.header.stamp)

            # 성능 로그 (1초마다)
            now = self.get_clock().now()
            elapsed = (now - self.last_log_time).nanoseconds / 1e9

            if elapsed >= 1.0:
                fps = self.processed_frames / elapsed
                self.get_logger().info(
                    f'처리 속도: {fps:.1f} FPS '
                    f'(총 {self.processed_frames}프레임)'
                )
                self.last_log_time = now
                self.processed_frames = 0

            self.processed_frames += 1

            # 첫 프레임 결과 이미지 저장 (디버깅용)
            if self.processed_frames == 1:
                result_image = self.draw_detections(cv_image, detections)
                cv2.imwrite('/tmp/ros_detection_result.png', result_image)
                self.get_logger().info(
                    '첫 프레임 결과 저장: /tmp/ros_detection_result.png'
                )

        except Exception as e:
            self.get_logger().error(f'이미지 처리 실패: {e}')

    def detect_red_objects(self, image):
        """
        빨간 물체 감지 (핵심 알고리즘)

        반환: [(cx, cy, area), ...] 리스트
        """
        # BGR → HSV 변환
        # HSV는 색상 기반 필터링에 훨씬 효과적
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # 빨간색 범위 (HSV)
        # Hue는 0-180, 빨간색은 0-10 또는 170-180
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        # 마스크 생성: 빨간색 픽셀만 True (흰색)
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 | mask2  # OR 연산

        # 윤곽선 찾기
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,  # 가장 바깥 윤곽선만
            cv2.CHAIN_APPROX_SIMPLE  # 윤곽선 압축
        )

        # 감지된 물체 리스트
        detections = []

        for contour in contours:
            # 작은 노이즈 필터링 (면적 < 500 픽셀)
            area = cv2.contourArea(contour)
            if area < 500:
                continue

            # 중심점 계산 (모멘트 사용)
            M = cv2.moments(contour)
            if M["m00"] != 0:  # 면적이 0이 아니면
                cx = int(M["m10"] / M["m00"])  # 무게중심 x
                cy = int(M["m01"] / M["m00"])  # 무게중심 y
                detections.append((cx, cy, area))

        return detections

    def process_detections(self, detections, timestamp):
        """
        감지 결과 처리
        나중에 MoveIt 2와 통합 시 이 부분 확장
        """
        if detections:
            for i, detection in enumerate(detections):
                cx, cy, area = detection
                self.get_logger().info(
                    f'[{timestamp.sec}.{timestamp.nanosec:09d}] '
                    f'물체 {i+1}: 중심 ({cx:3d}, {cy:3d}), '
                    f'면적 {area:7.1f}'
                )
        # else:
        #     self.get_logger().debug('감지된 물체 없음')

    def draw_detections(self, image, detections):
        """
        감지 결과를 이미지에 그리기 (시각화)
        """
        result = image.copy()

        for i, detection in enumerate(detections):
            cx, cy, area = detection

            # 중심점에 십자 표시
            cv2.drawMarker(
                result, (cx, cy), (0, 255, 255),  # 노란색
                markerType=cv2.MARKER_CROSS,
                markerSize=30, thickness=3
            )

            # 텍스트 표시
            text = f'Object {i+1}'
            cv2.putText(
                result, text, (cx - 40, cy - 70),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2
            )

            # 면적 표시
            cv2.putText(
                result, f'Area: {area:.0f}', (cx - 40, cy + 90),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2
            )

            # 경계 상자 (Bounding Box)
            x, y, w, h = cv2.boundingRect(
                np.array([[cx, cy]])  # 단순화, 실제로는 contour 사용
            )
            cv2.rectangle(result, (cx-60, cy-60), (cx+60, cy+60),
                         (0, 255, 0), 2)

        return result


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C 감지, 종료합니다...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
