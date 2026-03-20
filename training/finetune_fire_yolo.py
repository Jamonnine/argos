#!/usr/bin/env python3
"""
AI-Hub best.pt를 혼합 데이터셋으로 파인튜닝.

사용법:
  python finetune_fire_yolo.py --data fire_mixed/data.yaml --epochs 50 --batch 8
  # Kaggle P100 (16GB):    --batch 16 --workers 4
  # RTX 4050 (6GB):        --batch 4  --workers 2
  # CPU만 사용:            --device cpu

사전 준비:
  1. prepare_fire_dataset.py 실행 → fire_mixed/ 생성
  2. pip install ultralytics

결과:
  training/runs/fire_finetune/weights/best.pt
  → ARGOS fire_detection_node.py에서 이 파일을 사용하세요.

라이선스 주의:
  - ultralytics: AGPL-3.0. 상업 배포 시 Enterprise 라이선스 필요.
  - 학습된 가중치는 ARGOS Apache-2.0 메인 패키지와 분리 필수.
    별도 패키지: argos_fire_ai (라이선스 오염 차단)
"""

import argparse
import sys
from pathlib import Path


# pretrained best.pt 기본 경로 (AI-Hub luminous0219_best.pt)
DEFAULT_WEIGHTS = str(
    Path(__file__).parent.parent / "models" / "fire-smoke" / "pretrained" / "luminous0219_best.pt"
)

# 기본 data.yaml 경로
DEFAULT_DATA = str(Path(__file__).parent / "data" / "fire_mixed" / "data.yaml")


def check_ultralytics():
    """ultralytics 설치 여부 확인 (graceful 에러)."""
    try:
        import ultralytics  # noqa: F401
        return True
    except ImportError:
        print("[오류] ultralytics 패키지가 설치되지 않았습니다.")
        print()
        print("  설치 방법:")
        print("    pip install ultralytics")
        print()
        print("  GPU 가속 (RTX 4050 등):")
        print("    pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121")
        print("    pip install ultralytics")
        print()
        print("[라이선스] ultralytics는 AGPL-3.0 라이선스입니다.")
        print("  상업적 배포 시 Enterprise 라이선스가 필요합니다.")
        print("  ARGOS에서는 argos_fire_ai 패키지로 분리하여 사용하세요.")
        return False


def check_data_yaml(data_path: str) -> bool:
    """data.yaml 존재 및 유효성 확인."""
    p = Path(data_path)
    if not p.exists():
        print(f"[오류] data.yaml 없음: {p}")
        print()
        print("  데이터셋 준비 후 재시도하세요:")
        print("    python training/data/prepare_fire_dataset.py --output-dir training/data/fire_mixed")
        return False
    return True


def check_weights(weights_path: str) -> bool:
    """가중치 파일 존재 확인."""
    p = Path(weights_path)
    if not p.exists():
        print(f"[경고] 가중치 파일 없음: {p}")
        print()
        print("  AI-Hub luminous0219_best.pt 경로:")
        print(f"    {DEFAULT_WEIGHTS}")
        print()
        print("  대신 공개 YOLOv8 가중치로 파인튜닝 가능:")
        print("    --weights yolov8n.pt  (nano, 6.3MB)")
        print("    --weights yolov8s.pt  (small, 22MB)")
        return False
    return True


def get_device_info(device: str) -> str:
    """디바이스 정보 출력용 문자열 반환."""
    if device == "cpu":
        return "CPU (느림, 테스트용)"
    try:
        import torch
        if torch.cuda.is_available():
            name = torch.cuda.get_device_name(0)
            vram = torch.cuda.get_device_properties(0).total_memory / 1024**3
            return f"GPU: {name} ({vram:.1f}GB VRAM)"
        return "GPU 없음 → CPU로 대체"
    except ImportError:
        return f"device={device} (torch 미설치)"


def recommend_batch_size(device: str) -> int:
    """VRAM 기준 권장 배치 사이즈 반환."""
    if device == "cpu":
        return 4
    try:
        import torch
        if torch.cuda.is_available():
            vram_gb = torch.cuda.get_device_properties(0).total_memory / 1024**3
            if vram_gb >= 16:
                return 16   # P100/V100 (Kaggle)
            elif vram_gb >= 8:
                return 8    # RTX 3070/4060
            elif vram_gb >= 6:
                return 4    # RTX 4050 (민발님 PC)
            else:
                return 2    # VRAM 부족
    except ImportError:
        pass
    return 4


def main():
    parser = argparse.ArgumentParser(
        description="ARGOS YOLOv8 화재 탐지 파인튜닝",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--weights",
        default=DEFAULT_WEIGHTS,
        help=f"기반 가중치 파일 (기본: luminous0219_best.pt)",
    )
    parser.add_argument(
        "--data",
        default=DEFAULT_DATA,
        help="data.yaml 경로 (기본: training/data/fire_mixed/data.yaml)",
    )
    parser.add_argument(
        "--epochs",
        type=int,
        default=50,
        help="학습 에폭 수 (기본: 50)",
    )
    parser.add_argument(
        "--imgsz",
        type=int,
        default=320,
        help="입력 이미지 크기 (기본: 320, CPU에서 17.8 FPS 확보)",
    )
    parser.add_argument(
        "--batch",
        type=int,
        default=0,
        help="배치 사이즈 (기본: 0=자동, RTX4050→4, P100→16)",
    )
    parser.add_argument(
        "--workers",
        type=int,
        default=2,
        help="데이터 로더 워커 수 (기본: 2)",
    )
    parser.add_argument(
        "--device",
        default="0",
        help="학습 디바이스 (기본: 0=GPU, cpu=CPU)",
    )
    parser.add_argument(
        "--patience",
        type=int,
        default=10,
        help="조기 종료 patience (기본: 10 에폭)",
    )
    parser.add_argument(
        "--name",
        default="fire_finetune",
        help="실행 이름 (기본: fire_finetune)",
    )
    parser.add_argument(
        "--project",
        default=str(Path(__file__).parent / "runs"),
        help="결과 저장 디렉토리 (기본: training/runs)",
    )
    parser.add_argument(
        "--freeze",
        type=int,
        default=10,
        help="동결할 레이어 수 (기본: 10, backbone 동결 → 빠른 파인튜닝)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="실제 학습 없이 설정만 출력",
    )
    args = parser.parse_args()

    # ── 사전 검증 ──
    if not check_ultralytics():
        sys.exit(1)

    if not check_data_yaml(args.data):
        sys.exit(1)

    weights_ok = check_weights(args.weights)
    if not weights_ok:
        # 가중치 없으면 공개 YOLOv8s로 대체
        fallback = "yolov8s.pt"
        print(f"  → 공개 YOLOv8s ({fallback})로 파인튜닝을 시도합니다.")
        args.weights = fallback

    # ── 배치 사이즈 자동 결정 ──
    if args.batch == 0:
        args.batch = recommend_batch_size(args.device)
        print(f"[자동] 배치 사이즈: {args.batch} (VRAM 기반)")

    # ── 설정 출력 ──
    device_info = get_device_info(args.device)
    weights_path = Path(args.weights)
    data_path = Path(args.data)
    project_path = Path(args.project)

    print("=" * 60)
    print("ARGOS YOLOv8 화재 탐지 파인튜닝")
    print("=" * 60)
    print(f"  기반 가중치:  {weights_path.name}")
    print(f"  데이터셋:     {data_path}")
    print(f"  에폭:         {args.epochs}")
    print(f"  이미지 크기:  {args.imgsz}px")
    print(f"  배치:         {args.batch}")
    print(f"  워커:         {args.workers}")
    print(f"  디바이스:     {device_info}")
    print(f"  동결 레이어:  {args.freeze}개 (backbone 동결)")
    print(f"  조기 종료:    {args.patience} 에폭")
    print(f"  결과 경로:    {project_path / args.name}")
    print()

    if args.dry_run:
        print("[dry-run] 실제 학습 없이 종료합니다.")
        return

    # ── 학습 실행 ──
    from ultralytics import YOLO

    print("[로드] 가중치 파일 로딩...")
    model = YOLO(args.weights)

    print("[학습] 파인튜닝 시작...")
    print()

    results = model.train(
        data=str(data_path),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        workers=args.workers,
        device=args.device,
        patience=args.patience,
        save=True,
        save_period=10,         # 10 에폭마다 체크포인트 저장
        project=str(project_path),
        name=args.name,
        freeze=args.freeze,     # backbone 동결 → 전이학습 최적화
        # 증강 파라미터 (화재 도메인 최적화)
        mosaic=1.0,             # 4장 합성 증강
        flipud=0.3,             # 상하 반전 (드론 시점 고려)
        fliplr=0.5,             # 좌우 반전
        hsv_h=0.015,            # 색조 변환 (조명 다양화)
        hsv_s=0.5,              # 채도 (연기 농도 다양화)
        hsv_v=0.4,              # 명도 (야간/낮 환경)
        scale=0.5,              # 스케일 증강 (거리 다양화)
        # 학습률
        lr0=0.001,              # 파인튜닝용 낮은 초기 LR
        lrf=0.01,               # 최종 LR 비율
        warmup_epochs=3,        # 워밍업
        optimizer="AdamW",      # 파인튜닝에 적합
        # 기타
        val=True,               # 검증 활성화
        plots=True,             # 학습 곡선 저장
        verbose=True,
    )

    # ── 결과 출력 ──
    best_weights = project_path / args.name / "weights" / "best.pt"
    last_weights = project_path / args.name / "weights" / "last.pt"

    print()
    print("=" * 60)
    print("파인튜닝 완료")
    print("=" * 60)

    # mAP50 출력
    try:
        metrics = results.results_dict
        map50 = metrics.get("metrics/mAP50(B)", 0)
        map50_95 = metrics.get("metrics/mAP50-95(B)", 0)
        precision = metrics.get("metrics/precision(B)", 0)
        recall = metrics.get("metrics/recall(B)", 0)

        print(f"  mAP50:      {map50:.4f}  (기존 best.pt: 0.953)")
        print(f"  mAP50-95:   {map50_95:.4f}")
        print(f"  Precision:  {precision:.4f}")
        print(f"  Recall:     {recall:.4f}")

        # 성능 비교
        if map50 >= 0.953:
            print()
            print("  [성공] 기존 mAP50(0.953) 유지 또는 향상!")
        elif map50 >= 0.90:
            print()
            print(f"  [주의] mAP50 소폭 감소 ({map50:.4f} < 0.953)")
            print("  recall 향상 vs mAP 트레이드오프 확인 필요")
        else:
            print()
            print(f"  [경고] mAP50 대폭 감소 ({map50:.4f})")
            print("  하이퍼파라미터 재검토 또는 epoch 증가 필요")
    except Exception:
        pass

    print()
    print(f"  최적 가중치:  {best_weights}")
    print(f"  최종 가중치:  {last_weights}")
    print()
    print("다음 단계:")
    print(f"  1) ARGOS fire_detection_node.py에 best.pt 경로 지정")
    print(f"     MODEL_PATH = '{best_weights}'")
    print()
    print(f"  2) 엣지 케이스 평가:")
    print(f"     python models/fire-smoke/evaluate.py --weights {best_weights}")
    print()
    print("  3) argos_fire_ai 패키지로 분리 (AGPL 라이선스 오염 차단)")
    print()
    print("[라이선스 주의]")
    print("  ultralytics AGPL-3.0: 상업 배포 시 Enterprise 라이선스 필요")
    print("  학습된 가중치는 argos_fire_ai(분리 패키지)에만 포함")
    print("  ARGOS 메인 패키지(Apache-2.0)에 직접 포함 금지")


if __name__ == "__main__":
    main()
