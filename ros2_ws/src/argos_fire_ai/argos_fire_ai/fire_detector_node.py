# Copyright (C) 2026 jamonnine
# SPDX-License-Identifier: AGPL-3.0
#
# 이 파일은 AGPL-3.0 라이선스로 배포됩니다.
# argos_bringup (Apache 2.0)과 라이선스가 다르므로 별도 패키지로 분리합니다.

"""YOLOv8 기반 화재·연기 감지 ROS2 노드.

AI-Hub 학습 모델 (mAP50=0.953, 15종 화재현장 객체)을 사용합니다.
ultralytics 라이브러리가 없는 환경에서도 graceful하게 초기화 실패를 처리합니다.
"""

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header

# cv_bridge: ROS Image ↔ OpenCV numpy 배열 변환
try:
    from cv_bridge import CvBridge
    CV_BRIDGE_AVAILABLE = True
except ImportError:
    CV_BRIDGE_AVAILABLE = False

# OpenCV: 바운딩박스 시각화
try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False

# ultralytics: YOLOv8 추론 엔진
try:
    from ultralytics import YOLO
    ULTRALYTICS_AVAILABLE = True
except ImportError:
    ULTRALYTICS_AVAILABLE = False

# argos_interfaces: ARGOS 공용 메시지 타입
from argos_interfaces.msg import DetectedObject, DetectionArray
from sensor_msgs.msg import RegionOfInterest
from geometry_msgs.msg import Pose


# AI-Hub 모델의 15종 화재현장 클래스 (AGPL 모델 소유자 표기 유지 필수)
FIRE_CLASS_NAMES = [
    'fire',          # 화염
    'smoke',         # 연기
    'person',        # 인명
    'extinguisher',  # 소화기
    'hydrant',       # 소화전
    'door',          # 문
    'window',        # 창문
    'stairway',      # 계단
    'vehicle',       # 차량
    'debris',        # 잔해
    'hazmat',        # 위험물
    'electrical',    # 전기시설
    'gas_cylinder',  # 가스통
    'ladder',        # 사다리
    'hose',          # 호스
]

# 시각화용 클래스별 색상 (BGR)
CLASS_COLORS = {
    'fire':        (0,   0,   255),  # 빨강
    'smoke':       (128, 128, 128),  # 회색
    'person':      (0,   255, 0  ),  # 초록
    'extinguisher':(255, 0,   0  ),  # 파랑
    'hydrant':     (255, 100, 0  ),  # 주황
}
DEFAULT_COLOR = (0, 255, 255)  # 노랑 (미분류)


class FireDetectorNode(Node):
    """YOLOv8을 사용하여 화재 현장 객체를 탐지하는 ROS2 노드.

    구독:
        image_raw (sensor_msgs/Image): 입력 RGB 이미지

    발행:
        /fire_detections (argos_interfaces/DetectionArray): 탐지 결과 배열
        /fire_detections_viz (sensor_msgs/Image): 시각화 이미지 (바운딩박스 오버레이)
    """

    def __init__(self):
        super().__init__('fire_detector')

        # ──────────────────────────────────────────
        # 파라미터 선언
        # ──────────────────────────────────────────
        self.declare_parameter(
            'model_path',
            'models/fire-smoke/pretrained/luminous0219_best.pt',
        )
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('max_fps', 5.0)
        self.declare_parameter('input_size', 320)
        # use_sim_time은 ROS2 Jazzy에서 자동 선언됨 — 중복 선언 금지

        model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.max_fps = self.get_parameter('max_fps').value
        self.input_size = self.get_parameter('input_size').value

        # ──────────────────────────────────────────
        # AGPL-3.0 라이선스 고지 (시작 시 1회)
        # ──────────────────────────────────────────
        self.get_logger().warn(
            '[AGPL-3.0] fire_detector_node는 AGPL-3.0 라이선스입니다. '
            'AI-Hub YOLOv8 모델(luminous0219_best.pt)을 포함합니다. '
            '소스코드 공개 의무가 있는 라이선스입니다.'
        )

        # ──────────────────────────────────────────
        # 의존성 점검
        # ──────────────────────────────────────────
        self._model = None
        self._bridge = None

        if not ULTRALYTICS_AVAILABLE:
            self.get_logger().error(
                'ultralytics 라이브러리를 찾을 수 없습니다. '
                '설치 명령: pip install ultralytics. '
                '탐지 기능이 비활성화됩니다.'
            )
        else:
            self._load_model(model_path)

        if not CV_BRIDGE_AVAILABLE:
            self.get_logger().error(
                'cv_bridge를 찾을 수 없습니다. '
                'ROS 이미지 ↔ OpenCV 변환이 불가합니다.'
            )
        else:
            self._bridge = CvBridge()

        if not CV2_AVAILABLE:
            self.get_logger().warn(
                'OpenCV(cv2)를 찾을 수 없습니다. '
                '시각화 기능이 비활성화됩니다.'
            )

        # ──────────────────────────────────────────
        # 속도 제한 (max_fps)
        # ──────────────────────────────────────────
        self._min_interval = 1.0 / max(self.max_fps, 0.1)  # 0으로 나누기 방지
        self._last_processed_time = 0.0

        # ──────────────────────────────────────────
        # 퍼블리셔
        # ──────────────────────────────────────────
        self._pub_detections = self.create_publisher(
            DetectionArray,
            '/fire_detections',
            qos_profile=10,
        )
        self._pub_viz = self.create_publisher(
            Image,
            '/fire_detections_viz',
            qos_profile=10,
        )

        # ──────────────────────────────────────────
        # 구독자 (상대 토픽 — 네임스페이스 상속)
        # ──────────────────────────────────────────
        self._sub_image = self.create_subscription(
            Image,
            'image_raw',
            self._image_callback,
            qos_profile=10,
        )

        self.get_logger().info(
            f'fire_detector 초기화 완료 | '
            f'model={model_path} | '
            f'conf={self.conf_threshold} | '
            f'max_fps={self.max_fps} | '
            f'input_size={self.input_size}'
        )

    # ──────────────────────────────────────────────────
    # 모델 로딩
    # ──────────────────────────────────────────────────

    def _load_model(self, model_path: str) -> None:
        """YOLOv8 모델을 로드합니다. 실패 시 노드는 계속 실행됩니다."""
        try:
            self._model = YOLO(model_path)
            self.get_logger().info(f'YOLOv8 모델 로드 완료: {model_path}')
        except FileNotFoundError:
            self.get_logger().error(
                f'모델 파일을 찾을 수 없습니다: {model_path}. '
                '탐지 기능이 비활성화됩니다.'
            )
        except Exception as e:
            self.get_logger().error(
                f'모델 로드 실패: {e}. '
                '탐지 기능이 비활성화됩니다.'
            )

    # ──────────────────────────────────────────────────
    # 이미지 콜백
    # ──────────────────────────────────────────────────

    def _image_callback(self, msg: Image) -> None:
        """image_raw 수신 시 호출됩니다. max_fps 속도 제한을 적용합니다."""
        # 의존성 미충족 시 조용히 무시
        if self._model is None or self._bridge is None:
            return

        # 속도 제한: 현재 시각 기준 (wall clock)
        now = time.monotonic()
        if now - self._last_processed_time < self._min_interval:
            return
        self._last_processed_time = now

        # ROS Image → OpenCV numpy 배열
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'이미지 변환 실패: {e}')
            return

        # YOLOv8 추론
        results = self._run_inference(cv_image)
        if results is None:
            return

        # 결과 파싱 → DetectionArray 발행
        detection_array = self._build_detection_array(msg.header, results)
        self._pub_detections.publish(detection_array)

        # 시각화 이미지 발행
        if CV2_AVAILABLE and self._pub_viz.get_subscription_count() > 0:
            viz_image = self._draw_detections(cv_image, detection_array)
            try:
                viz_msg = self._bridge.cv2_to_imgmsg(viz_image, encoding='bgr8')
                viz_msg.header = msg.header
                self._pub_viz.publish(viz_msg)
            except Exception as e:
                self.get_logger().warn(f'시각화 이미지 변환 실패: {e}')

    # ──────────────────────────────────────────────────
    # YOLOv8 추론
    # ──────────────────────────────────────────────────

    def _run_inference(self, cv_image):
        """YOLOv8 추론을 실행합니다. 실패 시 None 반환."""
        try:
            results = self._model.predict(
                source=cv_image,
                imgsz=self.input_size,
                conf=self.conf_threshold,
                verbose=False,  # 매 프레임 로그 억제
            )
            return results
        except Exception as e:
            self.get_logger().warn(f'YOLOv8 추론 실패: {e}')
            return None

    # ──────────────────────────────────────────────────
    # 탐지 결과 → DetectionArray 변환
    # ──────────────────────────────────────────────────

    def _build_detection_array(
        self, source_header: Header, results
    ) -> DetectionArray:
        """YOLOv8 결과를 argos_interfaces/DetectionArray로 변환합니다."""
        array_msg = DetectionArray()
        array_msg.header = source_header
        array_msg.detector_name = 'yolo_v8'
        array_msg.frame_id = 0  # 프레임 카운터 (추후 확장 가능)

        if not results or len(results) == 0:
            return array_msg

        # results는 리스트. 단일 이미지이므로 results[0] 사용
        result = results[0]
        boxes = result.boxes

        if boxes is None or len(boxes) == 0:
            return array_msg

        for box in boxes:
            obj = DetectedObject()
            obj.header = source_header

            # 클래스 이름
            cls_idx = int(box.cls[0].item())
            if cls_idx < len(FIRE_CLASS_NAMES):
                obj.class_name = FIRE_CLASS_NAMES[cls_idx]
            else:
                obj.class_name = f'class_{cls_idx}'

            # 확신도
            obj.confidence = float(box.conf[0].item())

            # 추적 ID (YOLO 추적 미사용 시 -1)
            obj.track_id = -1

            # 2D 바운딩박스 (xywh → RegionOfInterest)
            xywh = box.xywh[0]  # [x_center, y_center, w, h]
            x_center = float(xywh[0].item())
            y_center = float(xywh[1].item())
            w = float(xywh[2].item())
            h = float(xywh[3].item())

            roi = RegionOfInterest()
            roi.x_offset = max(0, int(x_center - w / 2))
            roi.y_offset = max(0, int(y_center - h / 2))
            roi.width = int(w)
            roi.height = int(h)
            roi.do_rectify = False
            obj.bbox_2d = roi

            # 3D 위치 (카메라 전용 노드이므로 기본값 유지)
            obj.pose_3d = Pose()
            obj.distance = -1.0  # 깊이 정보 없음

            array_msg.detections.append(obj)

        return array_msg

    # ──────────────────────────────────────────────────
    # 바운딩박스 시각화
    # ──────────────────────────────────────────────────

    def _draw_detections(self, cv_image, detection_array: DetectionArray):
        """탐지 결과를 이미지에 오버레이합니다."""
        viz = cv_image.copy()

        for obj in detection_array.detections:
            roi = obj.bbox_2d
            x1 = roi.x_offset
            y1 = roi.y_offset
            x2 = x1 + roi.width
            y2 = y1 + roi.height

            color = CLASS_COLORS.get(obj.class_name, DEFAULT_COLOR)

            # 바운딩박스
            cv2.rectangle(viz, (x1, y1), (x2, y2), color, thickness=2)

            # 레이블 (클래스명 + 확신도)
            label = f'{obj.class_name} {obj.confidence:.2f}'
            label_size, baseline = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1
            )
            # 레이블 배경
            cv2.rectangle(
                viz,
                (x1, y1 - label_size[1] - baseline),
                (x1 + label_size[0], y1),
                color,
                thickness=cv2.FILLED,
            )
            cv2.putText(
                viz, label,
                (x1, y1 - baseline),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (0, 0, 0),  # 검정 텍스트
                thickness=1,
            )

        return viz


# ──────────────────────────────────────────────────────
# 엔트리포인트
# ──────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = FireDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
