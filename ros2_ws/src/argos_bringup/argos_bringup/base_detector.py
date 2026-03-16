"""base_detector.py — ARGOS AI 감지 모델 추상 인터페이스

AI/ML 전문가 권고: 모델 변경 시 코드 수정 최소화를 위한 추상화.
YOLOv8, YOLOv11, Detectron2 등 어떤 프레임워크든 이 인터페이스를 구현하면
ARGOS 노드에서 플러그인 방식으로 교체 가능.

사용 예:
    detector = YOLODetector('best.pt')
    results = detector.detect(frame, conf=0.5)
    for det in results:
        print(f'{det.class_name}: {det.confidence:.2f} at {det.bbox}')
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List, Optional
import numpy as np


@dataclass
class Detection:
    """단일 감지 결과."""
    class_name: str           # 클래스명 (e.g., 'fire', 'smoke', 'person')
    class_id: int             # 클래스 인덱스
    confidence: float         # 신뢰도 0.0~1.0
    bbox: tuple               # (x1, y1, x2, y2) 픽셀 좌표
    is_hazard: bool = False   # 위험물 여부 (전기화재 원인물 등)
    metadata: dict = field(default_factory=dict)  # 추가 정보


class BaseDetector(ABC):
    """AI 감지 모델 추상 인터페이스.

    모든 ARGOS AI 감지 노드는 이 인터페이스를 통해 모델에 접근.
    모델 교체 시 이 클래스의 서브클래스만 변경하면 됨.
    """

    @abstractmethod
    def load(self, model_path: str) -> None:
        """모델 가중치 로드.

        Args:
            model_path: .pt, .onnx, .engine 파일 경로
        """
        pass

    @abstractmethod
    def detect(self, frame: np.ndarray, conf: float = 0.5,
               imgsz: int = 640) -> List[Detection]:
        """프레임에서 객체 감지 실행.

        Args:
            frame: BGR 이미지 (numpy array, HxWx3)
            conf: 최소 신뢰도 임계값
            imgsz: 추론 이미지 크기

        Returns:
            Detection 리스트
        """
        pass

    @abstractmethod
    def get_classes(self) -> dict:
        """모델의 클래스 이름 딕셔너리 반환.

        Returns:
            {0: 'fire', 1: 'smoke', ...}
        """
        pass

    @property
    @abstractmethod
    def model_name(self) -> str:
        """모델 식별자 (로깅/디버깅용)."""
        pass


class YOLODetector(BaseDetector):
    """Ultralytics YOLO 기반 감지기 (YOLOv8/v10/v11 호환)."""

    def __init__(self, model_path: str = ''):
        self._model = None
        self._model_path = model_path
        self._classes = {}
        if model_path:
            self.load(model_path)

    def load(self, model_path: str) -> None:
        from ultralytics import YOLO
        self._model = YOLO(model_path)
        self._model_path = model_path
        self._classes = dict(self._model.names)

    def detect(self, frame: np.ndarray, conf: float = 0.5,
               imgsz: int = 640) -> List[Detection]:
        if self._model is None:
            return []
        results = self._model(frame, verbose=False, conf=conf, imgsz=imgsz)
        detections = []
        for box in results[0].boxes:
            cls_id = int(box.cls[0])
            det = Detection(
                class_name=self._classes.get(cls_id, f'class_{cls_id}'),
                class_id=cls_id,
                confidence=float(box.conf[0]),
                bbox=tuple(int(x) for x in box.xyxy[0].tolist()),
            )
            detections.append(det)
        return detections

    def get_classes(self) -> dict:
        return self._classes

    @property
    def model_name(self) -> str:
        return f'YOLO({self._model_path})'
