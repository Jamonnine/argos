#!/usr/bin/env python3
"""ARGOS 불꽃/연기 감지 모델 학습 스크립트

D-Fire 데이터셋(21,527장)으로 YOLOv8n 파인튜닝.
화재 현장에서 불꽃과 연기를 바운딩박스로 탐지.

사전 준비:
  1. D-Fire 데이터셋 다운로드 (아래 중 택 1):
     - Kaggle: https://www.kaggle.com/datasets/sayedgamal99/smoke-fire-detection-yolo
     - OneDrive: https://1drv.ms/u/c/c0bd25b6b048b01d/EbLgD7bES4FDvUN37Grxn8QBF5gIBBc7YV2qklF08GCiBw
  2. 압축 해제 후 datasets/d-fire/ 아래에 배치:
     datasets/d-fire/
       ├── images/
       │   ├── train/
       │   ├── val/
       │   └── test/
       └── labels/
           ├── train/
           ├── val/
           └── test/
  3. pip install ultralytics

실행:
  python train.py                    # 기본 (GPU 자동 감지)
  python train.py --device cpu       # CPU 강제
  python train.py --epochs 50        # 에폭 수 변경
  python train.py --imgsz 960        # 고해상도 학습

결과:
  runs/detect/fire-smoke/weights/best.pt  → ARGOS 화재 감지 노드에 사용
"""
import argparse
from ultralytics import YOLO


def main():
    parser = argparse.ArgumentParser(description='ARGOS Fire/Smoke Detection Training')
    parser.add_argument('--model', default='yolov8n.pt', help='Base model (default: yolov8n.pt)')
    parser.add_argument('--data', default='data.yaml', help='Dataset config')
    parser.add_argument('--epochs', type=int, default=30, help='Training epochs')
    parser.add_argument('--imgsz', type=int, default=640, help='Image size')
    parser.add_argument('--device', default='0', help='Device (0=GPU, cpu=CPU)')
    parser.add_argument('--batch', type=int, default=16, help='Batch size')
    parser.add_argument('--name', default='fire-smoke', help='Run name')
    args = parser.parse_args()

    model = YOLO(args.model)

    results = model.train(
        data=args.data,
        epochs=args.epochs,
        imgsz=args.imgsz,
        device=args.device,
        batch=args.batch,
        name=args.name,
        workers=4,
        patience=10,       # 10 에폭 개선 없으면 조기 종료
        save=True,
        save_period=10,     # 10 에폭마다 체크포인트
        pretrained=True,
        optimizer='auto',
        lr0=0.01,
        lrf=0.01,
        mosaic=1.0,         # 모자이크 증강
        flipud=0.5,         # 상하 반전 (화재 영상 특성)
        fliplr=0.5,         # 좌우 반전
    )

    print(f"\n학습 완료!")
    print(f"최적 가중치: runs/detect/{args.name}/weights/best.pt")
    print(f"→ ARGOS fire_detection_node.py에서 이 파일을 사용하세요.")


if __name__ == '__main__':
    main()
