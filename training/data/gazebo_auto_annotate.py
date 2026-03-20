#!/usr/bin/env python3
"""
ARGOS Gazebo 자동 어노테이션 수집기
=====================================
Gazebo Harmonic 세그멘테이션 카메라로 촬영한 영상을
YOLO 학습용 레이블로 자동 변환하여 저장한다.

사용법:
  # Gazebo + navigation.launch.py 실행 후:
  python3 gazebo_auto_annotate.py --output-dir training/data/gazebo_synth --duration 60

출력:
  gazebo_synth/
  ├── images/000000.png  ... — RGB 원본
  └── labels/000000.txt  ... — YOLO 형식 레이블 (클래스 cx cy w h)

YOLO 클래스 정의:
  0 = fire   (fire_source_a, fire_source_b)
  1 = smoke  (smoke_a, smoke_b)
  2 = person (mannequin_*)

의존성:
  pip install opencv-python numpy
  ROS2 패키지: sensor_msgs, cv_bridge, rclpy

주의:
  - Gazebo 세그멘테이션 ID는 SDF 모델 등록 순서에 따라 부여됨
  - 시뮬레이션 실행 중 모델 추가/삭제 시 ID가 바뀔 수 있으므로
    월드 재시작 없이 스크립트 재실행 금지
  - cv_bridge는 ROS2 환경에서 NumPy 1.x 와 호환됨 (NumPy 2.x 주의)
"""

import argparse
import os
import sys
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

try:
    from cv_bridge import CvBridge, CvBridgeError
except ImportError:
    print("[오류] cv_bridge 없음. ROS2 환경에서 실행하세요.")
    sys.exit(1)


# ===========================================================================
#  클래스 매핑 테이블
#  fire_building.sdf 모델명 → YOLO 클래스 ID
#  Gazebo semantic segmentation은 모델명(label)을 픽셀 색상에 인코딩함.
#  SegmentationCameraPlugin이 RGB 채널에 (r, g, b) = (label_id) 형태로 기록.
#  아래 매핑은 모델명 문자열 패턴으로 클래스를 결정한다.
# ===========================================================================
CLASS_PATTERNS = {
    # 패턴 문자열 → YOLO 클래스 ID
    "fire_source": 0,   # fire_source_a, fire_source_b
    "flame":       0,   # flame_a, flame_b (시각적 화염 오브젝트)
    "smoke":       1,   # smoke_a, smoke_b
    "mannequin":   2,   # mannequin_* (요구조자)
}

# 클래스 이름 (YOLO yaml 작성용 참고)
CLASS_NAMES = {0: "fire", 1: "smoke", 2: "person"}

# 최소 바운딩 박스 픽셀 크기 (너무 작으면 노이즈로 제거)
MIN_BBOX_AREA_PX = 100  # 10×10 px 미만 무시

# connectedComponents 설정
CC_CONNECTIVITY = 8     # 8-연결 컴포넌트 (대각선 포함)


def model_name_to_class_id(model_name: str) -> Optional[int]:
    """
    모델명 → YOLO 클래스 ID 변환.
    매핑 없는 모델(벽, 바닥, 장애물 등)은 None 반환.
    """
    name_lower = model_name.lower()
    for pattern, class_id in CLASS_PATTERNS.items():
        if pattern in name_lower:
            return class_id
    return None


class SegmentationAnnotator(Node):
    """
    ROS2 노드: 세그멘테이션 + RGB 이미지를 동기화하여 YOLO 레이블 생성.

    내부적으로 세그멘테이션 이미지의 컬러 채널로부터 클래스 마스크를 추출하고,
    connectedComponents로 개별 객체를 분리한 뒤 bbox를 계산한다.
    """

    def __init__(self, output_dir: str, duration: float):
        super().__init__("gazebo_auto_annotate")

        self.output_dir = Path(output_dir)
        self.duration = duration
        self.bridge = CvBridge()

        # 출력 디렉토리 생성
        self.img_dir = self.output_dir / "images"
        self.lbl_dir = self.output_dir / "labels"
        self.img_dir.mkdir(parents=True, exist_ok=True)
        self.lbl_dir.mkdir(parents=True, exist_ok=True)

        # 최신 수신 이미지 캐시 (동기화용)
        self._seg_msg: Optional[Image] = None
        self._rgb_msg: Optional[Image] = None

        # 프레임 카운터 및 타이머
        self._frame_idx = 0
        self._start_time = time.time()

        # QoS: Sensor Data (신뢰성 BEST_EFFORT, 최신 1개만 유지)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # 구독 설정
        self._seg_sub = self.create_subscription(
            Image,
            "/segmentation/image",
            self._seg_callback,
            sensor_qos,
        )
        self._rgb_sub = self.create_subscription(
            Image,
            "/camera/image_raw",
            self._rgb_callback,
            sensor_qos,
        )

        # 처리 타이머: 5Hz (세그멘테이션 카메라 update_rate와 동일)
        self._timer = self.create_timer(0.2, self._process_frame)

        self.get_logger().info(
            f"어노테이션 수집 시작 — 출력: {self.output_dir}, "
            f"지속시간: {self.duration}초"
        )

    # -----------------------------------------------------------------------
    #  콜백: 최신 이미지 캐시
    # -----------------------------------------------------------------------

    def _seg_callback(self, msg: Image) -> None:
        self._seg_msg = msg

    def _rgb_callback(self, msg: Image) -> None:
        self._rgb_msg = msg

    # -----------------------------------------------------------------------
    #  핵심 처리: 세그멘테이션 → YOLO 레이블
    # -----------------------------------------------------------------------

    def _process_frame(self) -> None:
        """타이머 콜백: 세그멘테이션+RGB 쌍이 모두 준비되면 어노테이션 저장."""

        # 수집 시간 초과 시 종료
        elapsed = time.time() - self._start_time
        if elapsed > self.duration:
            self.get_logger().info(
                f"수집 완료 — {self._frame_idx}프레임 저장 ({self.output_dir})"
            )
            self.destroy_node()
            rclpy.shutdown()
            return

        # 두 이미지 모두 수신되지 않았으면 건너뜀
        if self._seg_msg is None or self._rgb_msg is None:
            return

        try:
            # ROS Image → NumPy 변환
            # 세그멘테이션: Gazebo가 RGB8로 인코딩된 레이블 맵을 출력
            seg_cv = self.bridge.imgmsg_to_cv2(self._seg_msg, desired_encoding="rgb8")
            rgb_cv = self.bridge.imgmsg_to_cv2(self._rgb_msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().warn(f"cv_bridge 변환 실패: {e}")
            return

        # YOLO 레이블 추출
        labels = self._extract_yolo_labels(seg_cv)

        # 레이블이 없으면 저장 생략 (배경만인 프레임 제외로 데이터 품질 향상)
        if not labels:
            return

        # 파일명 (6자리 제로패딩)
        stem = f"{self._frame_idx:06d}"

        # RGB 이미지 저장
        img_path = self.img_dir / f"{stem}.png"
        cv2.imwrite(str(img_path), rgb_cv)

        # YOLO 레이블 저장
        lbl_path = self.lbl_dir / f"{stem}.txt"
        with open(lbl_path, "w", encoding="utf-8") as f:
            for lbl in labels:
                f.write(lbl + "\n")

        self._frame_idx += 1

        if self._frame_idx % 10 == 0:
            self.get_logger().info(
                f"[{elapsed:.1f}s] {self._frame_idx}프레임 저장 완료"
            )

    def _extract_yolo_labels(self, seg_img: np.ndarray) -> list[str]:
        """
        세그멘테이션 이미지에서 YOLO 형식 레이블 추출.

        Gazebo Harmonic SegmentationCamera 동작 원리:
        - semantic 모드: 각 모델에 고유 RGB 색상 부여
        - 픽셀 색상 = 해당 모델의 레이블 색상
        - R 채널과 G 채널로 최대 65536개 모델 구분 가능

        처리 흐름:
        1. 고유 색상 목록 추출 → 각 색상이 어떤 모델인지 판별
        2. 타겟 클래스에 해당하는 색상의 마스크 생성
        3. connectedComponents로 개별 객체 분리
        4. 각 컴포넌트의 bbox 계산 → 정규화 → YOLO 형식 문자열 생성

        Returns:
            ["class_id cx cy w h", ...] 형식의 문자열 리스트
        """
        h, w = seg_img.shape[:2]
        labels = []

        # 이미지에 등장하는 고유 색상 추출
        # reshape: (H,W,3) → (H*W, 3) 후 unique
        flat = seg_img.reshape(-1, 3)
        unique_colors = np.unique(flat, axis=0)

        # 순수 배경(검정)은 제외
        unique_colors = unique_colors[~np.all(unique_colors == 0, axis=1)]

        for color in unique_colors:
            # 색상 → 클래스 ID 결정
            # Gazebo는 내부적으로 label_id를 RGB에 인코딩함:
            #   label_id = R + G * 256 (B는 미사용)
            # 그러나 모델명 기반 매핑을 위해 label_id를 얻은 뒤
            # Gazebo의 내부 레이블 테이블을 조회해야 함.
            #
            # 현재 구현: label_id로 직접 모델명을 얻는 ROS API가 없으므로
            # 색상 범위(heuristic)로 클래스를 분류.
            # → 더 정확한 방법: /segmentation/labels_map 토픽(Gazebo 확장)
            #   또는 시뮬레이션 시작 시 색상-모델명 테이블을 사전 수집.
            class_id = self._color_to_class_id(color)
            if class_id is None:
                continue

            # 해당 색상 픽셀만 마스크로 추출
            mask = np.all(seg_img == color, axis=2).astype(np.uint8) * 255

            # connectedComponents: 동일 클래스 내 개별 객체 분리
            num_labels, cc_map = cv2.connectedComponents(
                mask, connectivity=CC_CONNECTIVITY
            )

            # 레이블 0 = 배경, 1부터 실제 컴포넌트
            for comp_id in range(1, num_labels):
                comp_mask = (cc_map == comp_id).astype(np.uint8)

                # 픽셀 수 기반 최소 크기 필터
                if comp_mask.sum() < MIN_BBOX_AREA_PX:
                    continue

                # 바운딩 박스 계산 (x, y, w, h — OpenCV 관례)
                x, y, bw, bh = cv2.boundingRect(comp_mask)

                # YOLO 정규화: 중심 좌표 및 크기를 [0, 1] 범위로
                cx_norm = (x + bw / 2.0) / w
                cy_norm = (y + bh / 2.0) / h
                w_norm = bw / w
                h_norm = bh / h

                # 이미지 범위 클램핑 (부동소수점 오차 방지)
                cx_norm = max(0.0, min(1.0, cx_norm))
                cy_norm = max(0.0, min(1.0, cy_norm))
                w_norm = max(0.0, min(1.0, w_norm))
                h_norm = max(0.0, min(1.0, h_norm))

                labels.append(
                    f"{class_id} {cx_norm:.6f} {cy_norm:.6f} "
                    f"{w_norm:.6f} {h_norm:.6f}"
                )

        return labels

    def _color_to_class_id(self, color: np.ndarray) -> Optional[int]:
        """
        RGB 색상 → YOLO 클래스 ID.

        Gazebo Harmonic 세맨틱 세그멘테이션 색상 할당 규칙:
        - 모델 인덱스 순으로 고정 색상 팔레트를 순환 적용
        - 색상은 시뮬레이션 실행마다 동일 SDF라면 동일하게 유지

        fire_building.sdf 모델 등록 순서 기준 예상 색상 (실험으로 확인 필요):
          #1  ground_plane           → (0, 0, 128) 계열
          #2  wall_*                 → (128, *, *) 계열
          ...
          fire_source_a/b, smoke_a/b, mannequin_* → 고유 색상

        현재 구현: Heuristic 색상 범위 분류.
        정확도를 높이려면 Gazebo 실행 후 색상 테이블을 수동 캘리브레이션하여
        COLOR_TABLE 딕셔너리를 업데이트할 것.

        실용적 캘리브레이션 방법:
          1. Gazebo 실행 후 세그멘테이션 뷰 스크린샷
          2. 화재/연기/사람 위치 픽셀의 색상 픽업
          3. 아래 COLOR_TABLE에 (r, g, b): class_id 형태로 추가

        Returns:
            클래스 ID (0=fire, 1=smoke, 2=person) 또는 None (무시 대상)
        """
        r, g, b = int(color[0]), int(color[1]), int(color[2])

        # ── 색상 테이블 (Gazebo 실행 후 캘리브레이션 필요) ──────────────────
        # 아래는 Gazebo Harmonic의 기본 팔레트 패턴을 기반으로 한 추정값.
        # fire_building.sdf 모델 순서: ground(0) walls(1-9) fire_source(10-11)
        # smoke(12-13) flame(14-15) ...
        #
        # Gazebo 색상 생성 공식 (내부):
        #   label_id를 RGB로 인코딩: B=label_id%256, G=label_id//256%256, R=label_id//65536
        # → fire_source_a (label 10): B=10, G=0, R=0 → (0, 0, 10) → 거의 검정
        #
        # 실제로는 SegmentationCameraPlugin이 별도 색상 맵을 사용하므로
        # 아래 값은 캘리브레이션 후 갱신할 것.
        # ───────────────────────────────────────────────────────────────────

        # TODO: Gazebo 실행 후 실제 색상값으로 교체
        COLOR_TABLE: dict[tuple[int, int, int], int] = {
            # fire_source_a, fire_source_b — 주황/적색 계열
            (255, 128, 0):   0,   # fire
            (255, 64,  0):   0,
            (200, 50,  0):   0,
            # smoke_a, smoke_b — 회색 계열
            (128, 128, 128): 1,   # smoke
            (160, 160, 160): 1,
            (100, 100, 100): 1,
            # mannequin_* — 청록/청색 계열 (Gazebo 팔레트 기본값)
            (0,   200, 200): 2,   # person
            (0,   180, 180): 2,
        }

        key = (r, g, b)
        if key in COLOR_TABLE:
            return COLOR_TABLE[key]

        # 색상 범위 매칭 (COLOR_TABLE 미등록 시 fallback)
        # 화염: R 높음, G 중간, B 낮음
        if r > 180 and g < 150 and b < 60:
            return 0  # fire

        # 연기: R≈G≈B (회색), 중간 밝기
        if abs(r - g) < 20 and abs(g - b) < 20 and 60 < r < 200:
            return 1  # smoke

        # 사람(마네킹): Gazebo 팔레트에서 청록 계열 예상
        if b > 150 and r < 100 and g > 150:
            return 2  # person

        return None  # 무시 (벽, 바닥, 장애물 등)


# ===========================================================================
#  CLI 진입점
# ===========================================================================

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="ARGOS Gazebo 자동 어노테이션 수집기",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
예시:
  python3 gazebo_auto_annotate.py \\
      --output-dir training/data/gazebo_synth \\
      --duration 60

  python3 gazebo_auto_annotate.py \\
      --output-dir /tmp/synth_test \\
      --duration 30
        """,
    )
    parser.add_argument(
        "--output-dir",
        default="gazebo_synth",
        help="출력 디렉토리 (기본: ./gazebo_synth)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=60.0,
        help="수집 시간(초, 기본: 60)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    rclpy.init()

    try:
        node = SegmentationAnnotator(
            output_dir=args.output_dir,
            duration=args.duration,
        )
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[중단] 사용자 인터럽트 — 수집 종료")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
