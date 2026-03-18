#!/usr/bin/env python3
"""evaluate.py — ARGOS 화재 감지 모델 평가 스크립트

AI/ML 전문가 권고: 혼동행렬 + 오탐/미탐 분석.

실행:
  python evaluate.py --weights pretrained/sayedgamal99_yolo11_nano.pt --data data.yaml
  python evaluate.py --weights /path/to/best.pt --conf 0.3 --imgsz 640
"""
import argparse
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description='ARGOS Fire Model Evaluation')
    parser.add_argument('--weights', required=True, help='Model weights path')
    parser.add_argument('--data', default='data.yaml', help='Dataset config')
    parser.add_argument('--conf', type=float, default=0.25, help='Confidence threshold')
    parser.add_argument('--imgsz', type=int, default=640, help='Image size')
    parser.add_argument('--split', default='test', help='Dataset split (train/val/test)')
    args = parser.parse_args()

    from ultralytics import YOLO

    weights = Path(args.weights)
    if not weights.exists():
        print(f"가중치 파일 없음: {weights}")
        return

    model = YOLO(str(weights))
    print(f"Model: {weights.name} | Classes: {model.names}")

    # 평가 실행
    metrics = model.val(
        data=args.data,
        split=args.split,
        imgsz=args.imgsz,
        conf=args.conf,
        plots=True,  # 혼동행렬 + PR 곡선 자동 생성
        save_json=True,
    )

    # 결과 출력
    print(f"\n=== 평가 결과 ===")
    print(f"mAP@0.5:     {metrics.box.map50:.4f}")
    print(f"mAP@0.5:0.95: {metrics.box.map:.4f}")
    print(f"Precision:    {metrics.box.mp:.4f}")
    print(f"Recall:       {metrics.box.mr:.4f}")

    # 클래스별 성능
    print(f"\n=== 클래스별 성능 ===")
    for i, name in model.names.items():
        if i < len(metrics.box.maps):
            print(f"  {name}: mAP50={metrics.box.maps[i]:.4f}")

    # 오탐/미탐 분석
    print(f"\n=== 오탐/미탐 분석 ===")
    print(f"  Precision {metrics.box.mp:.3f} → 오탐율(FP) = {1-metrics.box.mp:.3f}")
    print(f"  Recall {metrics.box.mr:.3f} → 미탐율(FN) = {1-metrics.box.mr:.3f}")
    print(f"  소방 현장 기준: 미탐(FN)이 오탐(FP)보다 위험")
    print(f"  → conf 임계값 낮추면 미탐↓ 오탐↑ (트레이드오프)")

    print(f"\n혼동행렬: runs/detect/val/confusion_matrix.png")
    print(f"PR 곡선: runs/detect/val/PR_curve.png")


if __name__ == '__main__':
    main()
