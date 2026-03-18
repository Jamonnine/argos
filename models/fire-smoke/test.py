#!/usr/bin/env python3
"""ARGOS 불꽃/연기 감지 모델 테스트 및 벤치마크

학습 완료된 best.pt 가중치를 로드하여:
1. 모델 정보 출력
2. 샘플 이미지 추론 테스트
3. 추론 속도 벤치마크
4. ARGOS 통합 적합성 평가

실행:
  python test.py                                    # 학습 결과 best.pt 테스트
  python test.py --weights /path/to/best.pt         # 커스텀 가중치
  python test.py --image /path/to/fire_image.jpg    # 특정 이미지 테스트
"""
import argparse
import time
import numpy as np
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description='ARGOS Fire Model Test')
    parser.add_argument('--weights', default='runs/detect/fire-smoke/weights/best.pt')
    parser.add_argument('--image', default=None, help='Test image path')
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--conf', type=float, default=0.5)
    args = parser.parse_args()

    from ultralytics import YOLO

    weights = Path(args.weights)
    if not weights.exists():
        print(f"가중치 파일 없음: {weights}")
        print("먼저 train.py로 학습을 실행하세요.")
        return

    model = YOLO(str(weights))

    # 모델 정보
    print("=== 모델 정보 ===")
    print(f"가중치: {weights}")
    print(f"크기: {weights.stat().st_size / 1024 / 1024:.1f} MB")
    print(f"클래스: {model.names}")
    print(f"task: {model.task}")

    # 추론 테스트
    if args.image:
        print(f"\n=== 이미지 추론: {args.image} ===")
        results = model(args.image, conf=args.conf, imgsz=args.imgsz)
        for r in results:
            for box in r.boxes:
                cls = model.names[int(box.cls[0])]
                conf = float(box.conf[0])
                xyxy = box.xyxy[0].tolist()
                print(f"  {cls}: {conf:.2f} [{xyxy[0]:.0f},{xyxy[1]:.0f},{xyxy[2]:.0f},{xyxy[3]:.0f}]")
            if len(r.boxes) == 0:
                print("  감지된 객체 없음")
            # 결과 이미지 저장
            r.save(filename='test_result.jpg')
            print(f"  결과 저장: test_result.jpg")
    else:
        print("\n(--image 미지정, 더미 이미지로 벤치마크만 실행)")

    # 벤치마크
    print(f"\n=== 추론 벤치마크 ({args.imgsz}x{args.imgsz}) ===")
    dummy = np.random.randint(0, 255, (args.imgsz, args.imgsz, 3), dtype=np.uint8)
    model(dummy, verbose=False)  # warmup

    times = []
    for _ in range(10):
        s = time.perf_counter()
        model(dummy, verbose=False, imgsz=args.imgsz, conf=args.conf)
        times.append((time.perf_counter() - s) * 1000)

    avg = sum(times) / len(times)
    print(f"  평균: {avg:.1f}ms ({1000/avg:.1f} FPS)")
    print(f"  최소: {min(times):.1f}ms / 최대: {max(times):.1f}ms")

    # ARGOS 적합성
    import torch
    print(f"\n=== ARGOS 적합성 ===")
    print(f"  GPU: {'CUDA' if torch.cuda.is_available() else 'CPU only'}")
    print(f"  실시간 가능 (640x640): {'Yes' if avg < 100 else 'GPU 필요'}")
    print(f"  라이선스: 학술용 (별도 패키지 분리 권장)")


if __name__ == '__main__':
    main()
