#!/usr/bin/env python3
"""convert_tensorrt.py — YOLOv8 → ONNX → TensorRT 변환

AI 전문가 권고: 엣지 배포 시 TensorRT로 2~3배 추가 가속.

실행:
  # ONNX 변환
  python convert_tensorrt.py --weights pretrained/sayedgamal99_yolo11_nano.pt --format onnx

  # TensorRT 변환 (NVIDIA GPU 필요)
  python convert_tensorrt.py --weights pretrained/sayedgamal99_yolo11_nano.pt --format engine

  # 벤치마크 비교
  python convert_tensorrt.py --weights pretrained/sayedgamal99_yolo11_nano.pt --benchmark
"""
import argparse
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description='ARGOS Model Format Conversion')
    parser.add_argument('--weights', required=True, help='Input .pt weights')
    parser.add_argument('--format', default='onnx', choices=['onnx', 'engine', 'tflite'],
                        help='Output format')
    parser.add_argument('--imgsz', type=int, default=640, help='Image size')
    parser.add_argument('--half', action='store_true', help='FP16 half precision')
    parser.add_argument('--benchmark', action='store_true', help='Run speed benchmark')
    args = parser.parse_args()

    from ultralytics import YOLO
    import time

    weights = Path(args.weights)
    if not weights.exists():
        print(f"가중치 없음: {weights}")
        return

    model = YOLO(str(weights))
    print(f"Model: {weights.name} | Classes: {model.names}")

    if args.benchmark:
        import numpy as np
        dummy = np.random.randint(0, 255, (args.imgsz, args.imgsz, 3), dtype=np.uint8)

        # PT 벤치마크
        model(dummy, verbose=False, device=0)
        times_pt = []
        for _ in range(20):
            s = time.perf_counter()
            model(dummy, verbose=False, imgsz=args.imgsz, device=0)
            times_pt.append((time.perf_counter() - s) * 1000)
        print(f"\nPT:  {sum(times_pt)/len(times_pt):.1f}ms ({1000/(sum(times_pt)/len(times_pt)):.1f} FPS)")

        # ONNX 변환 + 벤치마크
        onnx_path = model.export(format='onnx', imgsz=args.imgsz, half=args.half)
        if onnx_path:
            onnx_model = YOLO(onnx_path)
            onnx_model(dummy, verbose=False)
            times_onnx = []
            for _ in range(20):
                s = time.perf_counter()
                onnx_model(dummy, verbose=False, imgsz=args.imgsz)
                times_onnx.append((time.perf_counter() - s) * 1000)
            print(f"ONNX: {sum(times_onnx)/len(times_onnx):.1f}ms ({1000/(sum(times_onnx)/len(times_onnx)):.1f} FPS)")

        print(f"\n변환 완료: {onnx_path}")
    else:
        # 단순 변환
        output_path = model.export(
            format=args.format,
            imgsz=args.imgsz,
            half=args.half,
        )
        print(f"\n변환 완료: {output_path}")
        print(f"크기: {Path(output_path).stat().st_size / 1024 / 1024:.1f} MB")


if __name__ == '__main__':
    main()
