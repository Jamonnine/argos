"""
ARGOS 열화상 화점 감지 노드 (Hotspot Detector)
================================================
열화상 카메라(L8 mono8)에서 화점을 감지하여 ThermalDetection 메시지로 퍼블리시.

L8 카메라 특성:
  - 8비트 그레이스케일 (0~255)
  - 뷰 내 최고온 → 255로 자동 정규화 (실제 FLIR AGC와 유사)
  - 절대 온도 ≈ 픽셀값 × resolution(3.0 K) + min_temp
  - 정규화 때문에 고정 임계값 대신 적응형(상위 N%) 방식 사용

토픽 (상대경로 — 네임스페이스 자동 적용):
  구독: thermal/image_raw (sensor_msgs/Image, mono8)
  발행: thermal/detections (ThermalDetection[])
        thermal/hotspot_viz (sensor_msgs/Image, 디버그 시각화)

LifecycleNode 상태:
  unconfigured → configure() → inactive
  inactive     → activate()  → active (구독/발행 활성, bridge 준비됨)
  active       → deactivate() → inactive (구독/발행 해제)
  inactive     → cleanup()   → unconfigured
"""

import numpy as np
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, State
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, RegionOfInterest
from geometry_msgs.msg import Point
from argos_interfaces.msg import ThermalDetection

# cv_bridge: ROS Image ↔ OpenCV numpy 변환
from cv_bridge import CvBridge

# OpenCV는 런타임에만 필요 (빌드 의존성 아님)
import cv2


class HotspotDetectorNode(LifecycleNode):
    """열화상 이미지에서 화점을 감지하는 노드.

    LifecycleNode 패턴:
    - __init__  : 파라미터 선언만 수행 (토픽·bridge 생성 금지)
    - on_configure  : 파라미터 읽기 + bridge 생성 + 상태 초기화
    - on_activate   : 구독/발행 생성 → 감지 시작
    - on_deactivate : 구독/발행 해제 → 감지 중단
    - on_cleanup    : 내부 상태 리셋
    """

    # L8 카메라 기본 resolution (K/pixel)
    L8_RESOLUTION = 3.0
    # L8 기본 최소 온도 (K)
    L8_MIN_TEMP = 253.15  # -20°C

    def __init__(self):
        super().__init__('hotspot_detector')

        # ── 파라미터 선언만 (activate 전에는 토픽·bridge 생성 금지) ──
        self.declare_parameter('top_percent', 0.05)     # 상위 5% 고온 픽셀
        self.declare_parameter('min_area', 20)           # 최소 화점 면적 (px)
        self.declare_parameter('l8_resolution', 3.0)     # K/pixel
        self.declare_parameter('l8_min_temp', 253.15)    # 최소 온도 (K)
        self.declare_parameter('severity_thresholds',    # 심각도 온도 기준 (K)
                               [323.15, 473.15, 673.15])  # 50°C, 200°C, 400°C
        self.declare_parameter('max_detections', 5)   # H4: 프레임당 최대 발행 화점 수 (센서퓨전 권고: 1→5)

        # 런타임 상태 (on_configure에서 초기화)
        self.top_percent = None
        self.min_area = None
        self.l8_resolution = None
        self.l8_min_temp = None
        self.severity_thresholds = None
        self.max_detections = None
        self.bridge = None  # on_configure에서 생성
        self._last_publish_time = None
        self._publish_cooldown = 1.0

        # 구독/발행 핸들 (on_activate에서 생성, on_deactivate에서 해제)
        self.sub_thermal = None
        self.pub_detection = None
        self.pub_viz = None

    # ─────────────────── Lifecycle Callbacks ───────────────────

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """파라미터 읽기 + bridge 생성 + 내부 상태 초기화.

        inactive 상태로 진입 전 호출됨. 여기서 ROS 토픽은 만들지 않는다.
        bridge 객체는 ROS 통신 없이 순수 변환 유틸리티이므로 on_configure에서 생성.
        """
        self.top_percent = self.get_parameter('top_percent').value
        self.min_area = self.get_parameter('min_area').value
        self.l8_resolution = self.get_parameter('l8_resolution').value
        self.l8_min_temp = self.get_parameter('l8_min_temp').value
        severity_list = self.get_parameter('severity_thresholds').value
        # H1: severity_thresholds 길이 검증 (low/medium/high/critical 경계 3개 필요)
        if len(severity_list) != 3:
            self.get_logger().error(
                f'severity_thresholds must have 3 values, got {len(severity_list)}. '
                f'Using defaults [323.15, 473.15, 673.15]')
            severity_list = [323.15, 473.15, 673.15]
        self.severity_thresholds = severity_list
        self.max_detections = self.get_parameter('max_detections').value

        # bridge 객체는 on_configure에서 생성 (ROS 통신 없는 순수 변환 유틸리티)
        self.bridge = CvBridge()
        self._last_publish_time = None
        self._publish_cooldown = 1.0

        self.get_logger().info(
            f'Hotspot detector configured (top {self.top_percent*100:.0f}%, '
            f'min_area={self.min_area}px)')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """구독/발행 생성 → 감지 시작.

        active 상태로 진입 전 호출됨.
        """
        # --- QoS: 센서 데이터 (Best Effort, 최신 1개만) ---
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # --- 구독/발행 ---
        self.sub_thermal = self.create_subscription(
            Image, 'thermal/image_raw', self.thermal_callback, sensor_qos)

        self.pub_detection = self.create_publisher(
            ThermalDetection, 'thermal/detections', 10)

        self.pub_viz = self.create_publisher(
            Image, 'thermal/hotspot_viz', sensor_qos)

        self.get_logger().info(
            f'Hotspot detector activated (top {self.top_percent*100:.0f}%, '
            f'min_area={self.min_area}px)')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """구독/발행 해제 → 감지 중단.

        inactive 상태로 진입 전 호출됨.
        """
        if self.sub_thermal is not None:
            self.destroy_subscription(self.sub_thermal)
            self.sub_thermal = None

        if self.pub_detection is not None:
            self.destroy_publisher(self.pub_detection)
            self.pub_detection = None

        if self.pub_viz is not None:
            self.destroy_publisher(self.pub_viz)
            self.pub_viz = None

        self.get_logger().info('Hotspot detector deactivated')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """내부 상태 리셋 → unconfigured로 복귀."""
        self.bridge = None
        self._last_publish_time = None
        self.get_logger().info('Hotspot detector cleaned up')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """종료 시 정리."""
        self.get_logger().info('Hotspot detector shutting down')
        return TransitionCallbackReturn.SUCCESS

    # ─────────────────── 핵심 로직 ───────────────────

    def pixel_to_kelvin(self, pixel_value: float) -> float:
        """L8 픽셀값 → 추정 온도(K) 변환."""
        # H2: 음수 켈빈 온도 방지 (물리적으로 불가능한 값)
        return max(0.0, self.l8_min_temp + pixel_value * self.l8_resolution)

    def classify_severity(self, temp_kelvin: float) -> str:
        """온도 기반 심각도 분류."""
        if temp_kelvin >= self.severity_thresholds[2]:
            return 'critical'   # 400°C+: 플래시오버 임박
        elif temp_kelvin >= self.severity_thresholds[1]:
            return 'high'       # 200°C+: 활성 화재
        elif temp_kelvin >= self.severity_thresholds[0]:
            return 'medium'     # 50°C+: 잠재 열원
        else:
            return 'low'        # 50°C 미만: 정상

    def thermal_callback(self, msg: Image):
        """열화상 이미지 수신 → 화점 감지 → 결과 발행."""
        # ROS Image → numpy 변환
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge 변환 실패: {e}')
            return

        h, w = cv_img.shape
        total_pixels = h * w

        # 적응형 임계값: 상위 N% 픽셀값
        flat = cv_img.flatten()
        threshold = np.percentile(flat, 100.0 * (1.0 - self.top_percent))

        # 임계값이 너무 낮으면 (전체가 균일 온도) 감지 건너뛰기
        if threshold < 10:
            return

        # 이진화 + 컨투어 검출
        _, binary = cv2.threshold(cv_img, int(threshold), 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(
            binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 시각화 이미지 준비 (컬러맵 적용)
        viz_img = cv2.applyColorMap(cv_img, cv2.COLORMAP_INFERNO)

        # H4: 프레임 내 상위 N개 화점 추적 (max_detections 파라미터)
        top_detections = []  # (temp, det) 리스트

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_area:
                continue

            # 바운딩 박스
            x, y, bw, bh = cv2.boundingRect(contour)

            # 화점 영역 온도 계산
            roi = cv_img[y:y+bh, x:x+bw]
            mask = np.zeros_like(cv_img[y:y+bh, x:x+bw])
            shifted_contour = contour - np.array([x, y])
            cv2.drawContours(mask, [shifted_contour], 0, 255, -1)

            masked_pixels = roi[mask > 0]
            if len(masked_pixels) == 0:
                continue

            max_pixel = float(np.max(masked_pixels))
            mean_pixel = float(np.mean(masked_pixels))

            max_temp = self.pixel_to_kelvin(max_pixel)
            mean_temp = self.pixel_to_kelvin(mean_pixel)

            # 중심점
            moments = cv2.moments(contour)
            if moments['m00'] > 0:
                cx = moments['m10'] / moments['m00']
                cy = moments['m01'] / moments['m00']
            else:
                cx = x + bw / 2.0
                cy = y + bh / 2.0

            # 신뢰도: 화점 영역의 평균 밝기 / 255
            confidence = mean_pixel / 255.0

            severity = self.classify_severity(max_temp)

            # ThermalDetection 메시지 생성
            det = ThermalDetection()
            det.header = msg.header
            det.max_temperature_kelvin = max_temp
            det.mean_temperature_kelvin = mean_temp
            det.bbox = RegionOfInterest(
                x_offset=int(x), y_offset=int(y),
                width=int(bw), height=int(bh),
                do_rectify=False)
            det.centroid_image = Point(x=cx, y=cy, z=0.0)
            det.confidence = min(confidence, 1.0)
            det.severity = severity
            det.area_ratio = float(area) / float(total_pixels)

            # 상위 N개 화점 추적 (온도 내림차순)
            top_detections.append((max_temp, det))
            top_detections.sort(key=lambda x: x[0], reverse=True)
            top_detections = top_detections[:self.max_detections]

            # 시각화에 표시 (모든 화점)
            color = {
                'low': (0, 255, 0),
                'medium': (0, 255, 255),
                'high': (0, 128, 255),
                'critical': (0, 0, 255),
            }.get(severity, (255, 255, 255))

            cv2.rectangle(viz_img, (x, y), (x+bw, y+bh), color, 2)
            label = f'{max_temp-273.15:.0f}C [{severity}]'
            cv2.putText(viz_img, label, (x, y-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

        # 상위 N개 화점 발행 (쿨다운 적용)
        if top_detections:
            now = self.get_clock().now()
            should_publish = (
                self._last_publish_time is None
                or (now - self._last_publish_time).nanoseconds / 1e9
                >= self._publish_cooldown
            )
            if should_publish:
                for _, det in top_detections:
                    self.pub_detection.publish(det)
                self._last_publish_time = now

        # 시각화 이미지 발행
        try:
            viz_msg = self.bridge.cv2_to_imgmsg(viz_img, encoding='bgr8')
            viz_msg.header = msg.header
            self.pub_viz.publish(viz_msg)
        except Exception as e:
            self.get_logger().error(f'시각화 이미지 변환 실패: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = HotspotDetectorNode()

    # LifecycleNode 단독 실행 시 수동으로 configure → activate 전환
    node.trigger_configure()
    node.trigger_activate()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.trigger_deactivate()
        node.trigger_cleanup()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
