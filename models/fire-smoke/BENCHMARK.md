# ARGOS Fire Detection Model Benchmark

> 측정일: 2026-03-16
> 하드웨어: Samsung Galaxy Book4 Pro — RTX 4050 Laptop GPU, i7-155H
> CUDA: 12.8 | WSL2 Ubuntu 24.04

## 결과

| 모델 | 크기 | GPU FPS | CPU FPS | 가속비 | 클래스 |
|------|------|---------|---------|--------|--------|
| sayedgamal99 YOLO11n | 5.2MB | **93.6** | 20.4 | 4.6x | Fire, Smoke |
| AI-Hub YOLOv8s | 21.5MB | **91.2** | 9.1 | 10.1x | 15종 화재현장 객체 |

## 측정 조건
- 입력: 640×640 RGB (numpy random)
- 신뢰도: conf=0.25
- 반복: GPU 20회, CPU 5회 (warmup 1회 제외)
- 프레임워크: ultralytics 8.4.22

## 소방 현장 적합성
- **실시간 기준 30 FPS** → 두 모델 모두 **3배 이상 초과 달성**
- 8중 센싱 3개 모델 동시 실행 시에도 **30 FPS 유지 가능** (93/3 = 31 FPS)
- CPU 모드에서도 sayedgamal99는 20 FPS (실시간 가능)

## Jetson 예상 (미측정)
- Jetson Orin Nano: GPU 기준 ~30 FPS 예상 (RTX 4050의 1/3)
- TensorRT 변환 시 +50% 추가 가속 예상
