#!/bin/bash
# ARGOS YOLOv8 → TensorRT 변환 스크립트
#
# 목적: 화점·연기 감지 YOLOv8 모델을 TensorRT FP16 엔진으로 변환
#       Jetson 또는 NVIDIA GPU 탑재 로봇에서 실시간 추론 성능 확보
#
# 전제:
#   pip install ultralytics tensorrt
#   NVIDIA GPU 및 CUDA 드라이버 설치 완료
#   (Jetson: JetPack SDK 포함)
#
# 사용법:
#   bash scripts/convert-tensorrt.sh [모델경로] [출력디렉토리]
#   예) bash scripts/convert-tensorrt.sh models/best.pt models/
#       bash scripts/convert-tensorrt.sh models/fire_yolov8n.pt models/
#
# 출력:
#   models/best.engine  (FP16 TensorRT 엔진)

set -e  # 오류 발생 시 즉시 종료

MODEL_PATH="${1:-models/best.pt}"
OUTPUT_DIR="${2:-models/}"

# 입력 파일 존재 확인
if [ ! -f "$MODEL_PATH" ]; then
    echo "[ERROR] 모델 파일을 찾을 수 없습니다: $MODEL_PATH"
    echo "  사용 가능한 모델:"
    find "${OUTPUT_DIR}" -name "*.pt" 2>/dev/null | sed 's/^/    /' || echo "    (없음)"
    exit 1
fi

# Python/ultralytics 설치 확인
if ! python3 -c "import ultralytics" 2>/dev/null; then
    echo "[ERROR] ultralytics 패키지가 설치되지 않았습니다."
    echo "  pip install ultralytics tensorrt"
    exit 1
fi

# GPU 확인
GPU_INFO=$(python3 -c "import torch; print(torch.cuda.get_device_name(0))" 2>/dev/null || echo "")
if [ -z "$GPU_INFO" ]; then
    echo "[WARN] CUDA GPU를 감지하지 못했습니다. CPU로 변환 시도합니다."
    DEVICE_FLAG="cpu"
else
    echo "[OK] GPU 감지: $GPU_INFO"
    DEVICE_FLAG="0"
fi

ENGINE_PATH="${MODEL_PATH%.pt}.engine"

echo "=== YOLOv8 → TensorRT 변환 ==="
echo "입력 모델: $MODEL_PATH"
echo "출력 엔진: $ENGINE_PATH"
echo "정밀도: FP16"
echo ""

# TensorRT 엔진 변환
echo "[1/2] TensorRT 변환 중 (수 분 소요)..."
python3 - <<PYEOF
from ultralytics import YOLO
import sys

model = YOLO('$MODEL_PATH')

# FP16 TensorRT 엔진 변환
# half=True: FP16 정밀도 (Jetson 추천, 속도 2배 향상)
# device=0:  첫 번째 GPU 사용
exported = model.export(
    format='engine',
    half=($( [ "$DEVICE_FLAG" = "0" ] && echo "True" || echo "False" )),
    device='$DEVICE_FLAG',
    verbose=False
)
print(f"[OK] 변환 완료: {exported}")
PYEOF

# 변환 결과 확인
if [ ! -f "$ENGINE_PATH" ]; then
    echo "[ERROR] 엔진 파일 생성 실패: $ENGINE_PATH"
    exit 1
fi

ENGINE_SIZE=$(du -h "$ENGINE_PATH" | cut -f1)
echo "  엔진 크기: $ENGINE_SIZE"
echo ""

# 추론 벤치마크
echo "[2/2] 추론 속도 벤치마크..."
python3 - <<PYEOF
from ultralytics import YOLO

model = YOLO('$ENGINE_PATH')

# 샘플 이미지로 워밍업 및 속도 측정
# 소방 현장 실용 기준: 30fps 이상 = 실시간 가능
results = model.predict('https://ultralytics.com/images/bus.jpg', verbose=False)
speed = results[0].speed

print(f"  전처리:  {speed['preprocess']:.1f} ms")
print(f"  추론:    {speed['inference']:.1f} ms")
print(f"  후처리:  {speed['postprocess']:.1f} ms")
total = speed['preprocess'] + speed['inference'] + speed['postprocess']
fps = 1000 / total if total > 0 else 0
print(f"  합계:    {total:.1f} ms ({fps:.1f} FPS)")

if fps >= 30:
    print(f"  [OK] 실시간 처리 가능 (30 FPS 기준 충족)")
else:
    print(f"  [WARN] 실시간 처리 미달 — 모델 경량화 또는 해상도 축소 검토")
PYEOF

echo ""
echo "=== 변환 완료 ==="
echo "엔진 파일: $ENGINE_PATH"
echo ""
echo "ROS2 노드에서 사용:"
echo "  model_path: '$ENGINE_PATH'  # hotspot_detector.launch.py 파라미터"
