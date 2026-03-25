#!/usr/bin/env python3
"""victim_detector_node.py — ARGOS 피해자 감지 노드

열화상 카메라에서 인체(생존자)를 감지하여 구조 우선순위를 판정.
열화상 시그니처(~36°C 체온)와 형태 분석으로 사람을 탐지.

시뮬레이션 모드: 가상 피해자 위치 기반 감지.
실제 모드: YOLOv8 열화상 인체 감지 모델 (FLIR ADAS 학습).

LifecycleNode 상태:
  unconfigured → configure() → inactive
  inactive     → activate()  → active (구독/발행/타이머 활성)
  active       → deactivate() → inactive (통신 중단)
"""
import math
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, State

from argos_interfaces.msg import VictimDetection, ThermalDetection
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image


class VictimDetectorNode(LifecycleNode):
    """열화상/RGB 기반 피해자 감지 + 구조 우선순위 판정."""

    def __init__(self):
        super().__init__('victim_detector')

        # ── 파라미터 선언만 (activate 전에는 토픽·타이머 생성 금지) ──
        self.declare_parameter('robot_id', 'argos_1')
        self.declare_parameter('simulation_mode', True)
        self.declare_parameter('detection_interval', 1.0)  # 초
        self.declare_parameter('body_temp_min_k', 305.0)  # ~32°C
        self.declare_parameter('body_temp_max_k', 313.0)  # ~40°C
        self.declare_parameter('model_path', '')  # YOLOv8 가중치 경로
        self.declare_parameter('victim_positions', [])  # [x1,y1, x2,y2, ...]

        # 런타임 상태 (on_configure에서 초기화)
        self.robot_id = None
        self.sim_mode = None
        self.model_path = None
        self.victim_positions = None
        self.detected_victims = None
        self.victim_counter = None
        self._detection_cooldown_sec = None
        self._cleanup_interval_sec = None
        self._stale_threshold_sec = None
        self.model = None
        self.robot_position = None

        # 구독/발행/타이머 핸들 (on_activate에서 생성, on_deactivate에서 해제)
        self.odom_sub = None
        self.thermal_sub = None
        self.victim_pub = None
        self.timer = None
        self.cleanup_timer = None

    # ─────────────────── Lifecycle Callbacks ───────────────────

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """파라미터 읽기 + 내부 상태 초기화.

        inactive 상태로 진입 전 호출됨. ROS 통신(토픽·타이머)은 만들지 않는다.
        """
        self.robot_id = self.get_parameter('robot_id').value
        self.sim_mode = self.get_parameter('simulation_mode').value
        self.model_path = self.get_parameter('model_path').value

        # 네임스페이스에서 robot_id 자동 추출
        ns = self.get_namespace().strip('/')
        if ns:
            self.robot_id = ns

        # 시뮬레이션: 가상 피해자 위치 파싱 (launch에서 파라미터로 설정)
        victim_flat = self.get_parameter('victim_positions').value
        self.victim_positions = []
        if victim_flat:
            for i in range(0, len(victim_flat) - 1, 2):
                self.victim_positions.append((victim_flat[i], victim_flat[i+1]))

        # 감지 이력 (재감지 방지)
        self.detected_victims = {}  # id -> last_seen_time
        self.victim_counter = 0
        self._detection_cooldown_sec = 10.0
        self._cleanup_interval_sec = 60.0
        self._stale_threshold_sec = 300.0  # 5분 미감지 시 이력 삭제

        # 로봇 위치 초기화
        self.robot_position = None

        # AI 모델 로딩 (실제 모드) — 실패 시 FAILURE 반환
        self.model = None
        if not self.sim_mode and self.model_path:
            try:
                from ultralytics import YOLO
                self.model = YOLO(self.model_path)
                self.get_logger().info(f'Victim detection model loaded: {self.model_path}')
            except Exception as e:
                self.get_logger().error(f'Model load failed: {e}')
                return TransitionCallbackReturn.FAILURE

        self.get_logger().info(
            f'[{self.robot_id}] Victim detector configured '
            f'(sim={self.sim_mode}, victims={len(self.victim_positions)})')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """구독/발행/타이머 생성 → 감지 시작.

        active 상태로 진입 전 호출됨.
        """
        # 구독
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self._odom_cb, 10)
        if not self.sim_mode:
            self.thermal_sub = self.create_subscription(
                Image, 'thermal/image_raw', self._thermal_cb, 10)

        # 발행
        self.victim_pub = self.create_publisher(
            VictimDetection, 'victim/detections', 10)

        # 타이머 (시뮬레이션 모드)
        if self.sim_mode:
            interval = self.get_parameter('detection_interval').value
            self.timer = self.create_timer(interval, self._simulate_detection)

        # 메모리 누수 방지: 오래된 감지 이력 정리
        self.cleanup_timer = self.create_timer(
            self._cleanup_interval_sec, self._cleanup_old_detections)

        self.get_logger().info(
            f'[{self.robot_id}] Victim detector activated '
            f'(sim={self.sim_mode}, victims={len(self.victim_positions)})')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """모든 핸들 destroy → 감지 중단.

        inactive 상태로 진입 전 호출됨.
        """
        # 타이머 해제
        if self.timer is not None:
            self.timer.cancel()
            self.destroy_timer(self.timer)
            self.timer = None
        if self.cleanup_timer is not None:
            self.cleanup_timer.cancel()
            self.destroy_timer(self.cleanup_timer)
            self.cleanup_timer = None

        # 발행자 해제
        if self.victim_pub is not None:
            self.destroy_publisher(self.victim_pub)
            self.victim_pub = None

        # 구독 해제
        if self.thermal_sub is not None:
            self.destroy_subscription(self.thermal_sub)
            self.thermal_sub = None
        if self.odom_sub is not None:
            self.destroy_subscription(self.odom_sub)
            self.odom_sub = None

        self.get_logger().info(f'[{self.robot_id}] Victim detector deactivated')
        return TransitionCallbackReturn.SUCCESS

    # ─────────────────── Callbacks ───────────────────

    def _odom_cb(self, msg):
        self.robot_position = msg.pose.pose.position

    def _thermal_cb(self, msg):
        """실제 열화상 이미지에서 인체 감지 (AI 모델 사용)."""
        if self.model is None:
            return
        try:
            from cv_bridge import CvBridge
            bridge = CvBridge()
            frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model(frame, verbose=False, conf=0.4)

            for box in results[0].boxes:
                cls_name = self.model.names[int(box.cls[0])]
                if cls_name.lower() in ('person', 'human', 'people'):
                    det = VictimDetection()
                    det.header.stamp = self.get_clock().now().to_msg()
                    det.header.frame_id = 'map'
                    det.robot_id = self.robot_id
                    det.confidence = float(box.conf[0])
                    det.detection_source = 'thermal'
                    det.estimated_status = 'unknown'
                    det.rescue_priority = 2
                    if self.robot_position:
                        det.location.header = det.header
                        det.location.point = self.robot_position
                    self.victim_pub.publish(det)
        except Exception as e:
            self.get_logger().error(f'Thermal detection error: {e}', throttle_duration_sec=5.0)

    def _simulate_detection(self):
        """시뮬레이션: 가상 피해자 위치 기반 감지."""
        if self.robot_position is None or not self.victim_positions:
            return

        detection_range = 8.0  # 감지 범위 (미터)

        for i, (vx, vy) in enumerate(self.victim_positions):
            dist = math.sqrt(
                (self.robot_position.x - vx)**2 +
                (self.robot_position.y - vy)**2)

            if dist <= detection_range:
                victim_id = f'victim_{i}'
                # M4: ROS 클록 사용 (시뮬레이션 시각 동기화, time.time() 제거)
                now = self.get_clock().now().nanoseconds / 1e9

                # 쿨다운 이내 재감지 방지
                if victim_id in self.detected_victims:
                    if now - self.detected_victims[victim_id] < self._detection_cooldown_sec:
                        continue

                self.detected_victims[victim_id] = now

                det = VictimDetection()
                det.header.stamp = self.get_clock().now().to_msg()
                det.header.frame_id = 'map'
                det.robot_id = self.robot_id

                det.location = PointStamped()
                det.location.header = det.header
                det.location.point.x = vx
                det.location.point.y = vy
                det.location.point.z = 0.0

                # 거리 기반 신뢰도 (가우시안 감쇠 — 선형보다 현실적)
                det.confidence = max(0.2, math.exp(-(dist / detection_range)**2))
                det.detection_source = 'thermal'
                det.body_temperature_kelvin = 310.0  # ~37°C
                det.estimated_status = 'stationary'
                det.time_stationary_sec = 60.0  # 시뮬: 움직이지 않는 것으로 가정
                det.rescue_priority = 1  # 최우선 (움직임 없음)

                self.victim_pub.publish(det)
                self.get_logger().warn(
                    f'VICTIM DETECTED at ({vx:.1f}, {vy:.1f}) '
                    f'dist={dist:.1f}m conf={det.confidence:.2f} '
                    f'priority={det.rescue_priority}')


    def _cleanup_old_detections(self):
        """메모리 누수 방지: 오래된 감지 이력 삭제."""
        # M4: ROS 클록 사용 (시뮬레이션 시각 동기화)
        now = self.get_clock().now().nanoseconds / 1e9
        expired = [vid for vid, t in self.detected_victims.items()
                   if now - t > self._stale_threshold_sec]
        for vid in expired:
            del self.detected_victims[vid]
        if expired:
            self.get_logger().debug(f'Cleaned up {len(expired)} stale victim records')


def main(args=None):
    rclpy.init(args=args)
    node = VictimDetectorNode()
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
