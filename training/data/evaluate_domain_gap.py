# Copyright 2026 민발 (Minbal) — Daegu Gangbuk Fire Station
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""E-SYN-5: 도메인 갭 평가 — 합성 vs 실제 화재 탐지 정량 비교.

평가 항목:
  1. SYN-FIRE (합성) val set  → mAP50
  2. D-Fire  (실제) test set → mAP50
  3. FLAME   (합성 증강)     → mAP50
  4. 도메인 갭 = mAP50(합성) - mAP50(실제)
  5. OOD(Out-of-Distribution) 분석

사용법:
  python evaluate_domain_gap.py \\
    --weights training/runs/synfire_finetune/weights/best.pt \\
    --synfire-val training/data/fire_dataset/val \\
    --dfire-test ~/datasets/DFireDataset/test \\
    --flame-dir training/data/flame_output \\
    --output training/runs/domain_gap_report.json

결과 형식 (JSON):
  {
    "synfire_val":  {"mAP50": 0.715, "mAP50_95": 0.39, "precision": 0.75, "recall": 0.65},
    "dfire_test":   {"mAP50": ..., ...},
    "flame_aug":    {"mAP50": ..., ...},
    "domain_gap":   {"mAP50_drop": ..., "relative_drop_pct": ...},
    "recommendation": "..."
  }
"""
import argparse
import json
import sys
from pathlib import Path
from datetime import datetime


def evaluate_dataset(model, data_yaml_or_dir, dataset_name, imgsz=640):
    """단일 데이터셋에 대해 YOLOv8 평가 실행."""
    print(f"\n[평가] {dataset_name}...")

    try:
        results = model.val(
            data=str(data_yaml_or_dir),
            imgsz=imgsz,
            batch=4,
            verbose=False,
        )

        metrics = {
            "mAP50":     round(float(results.box.map50), 4),
            "mAP50_95":  round(float(results.box.map),   4),
            "precision": round(float(results.box.mp),    4),
            "recall":    round(float(results.box.mr),    4),
            "dataset":   dataset_name,
            # results.box.n 은 ultralytics 버전에 따라 없을 수 있음
            "images":    int(getattr(results.box, "n", 0)),
        }

        print(
            f"  mAP50={metrics['mAP50']:.3f}  "
            f"P={metrics['precision']:.3f}  R={metrics['recall']:.3f}  "
            f"이미지={metrics['images']}장"
        )
        return metrics

    except Exception as exc:  # noqa: BLE001
        print(f"  [오류] {dataset_name} 평가 실패: {exc}")
        return {
            "mAP50": 0.0, "mAP50_95": 0.0,
            "precision": 0.0, "recall": 0.0,
            "dataset": dataset_name,
            "error": str(exc),
        }


def compute_domain_gap(synfire_metrics, dfire_metrics):
    """도메인 갭 계산 및 심각도 분류.

    기준 (arXiv 2024 권고 기반):
      - relative_drop > 30% → HIGH  (실데이터 혼합 필수)
      - 15% < relative_drop <= 30% → MEDIUM (FLAME 증강 권장)
      - relative_drop <= 15% → LOW  (현재 합성 데이터 충분)
    """
    syn_map  = synfire_metrics.get("mAP50", 0.0)
    real_map = dfire_metrics.get("mAP50", 0.0)
    drop     = round(syn_map - real_map, 4)
    rel_pct  = round(drop / max(syn_map, 0.001) * 100, 1)

    if rel_pct > 30:
        severity       = "HIGH"
        recommendation = (
            "도메인 갭 심각. D-Fire 실데이터 혼합 필수 "
            "(실제:합성 = 8:2 이상)"
        )
    elif rel_pct > 15:
        severity       = "MEDIUM"
        recommendation = (
            "도메인 갭 보통. FLAME 증강 추가 또는 "
            "D-Fire 50% 혼합 권장"
        )
    else:
        severity       = "LOW"
        recommendation = "도메인 갭 낮음. 현재 합성 데이터로 충분"

    return {
        "mAP50_synthetic":   syn_map,
        "mAP50_real":        real_map,
        "mAP50_drop":        drop,
        "relative_drop_pct": rel_pct,
        "severity":          severity,
        "recommendation":    recommendation,
    }


def build_report(synfire, dfire, flame, gap, weights_path):
    """JSON 보고서 딕셔너리 생성."""
    map50_values = [
        v.get("mAP50", 0.0)
        for v in (synfire, dfire, flame)
        if v.get("mAP50", 0.0) > 0.0
    ]

    return {
        "evaluation_date":  datetime.now().isoformat(),
        "model":            str(weights_path),
        "synfire_val":      synfire,
        "dfire_test":       dfire,
        "flame_augmented":  flame,
        "domain_gap":       gap,
        "summary": {
            "best_mAP50":          max(map50_values) if map50_values else 0.0,
            "worst_mAP50":         min(map50_values) if map50_values else 0.0,
            "datasets_evaluated":  len(map50_values),
        },
    }


def write_report(report, output_path):
    """보고서를 JSON 파일로 저장하고 콘솔에 요약 출력."""
    out = Path(output_path)
    out.parent.mkdir(parents=True, exist_ok=True)

    with open(out, "w", encoding="utf-8") as fh:
        json.dump(report, fh, indent=2, ensure_ascii=False)

    gap = report["domain_gap"]
    sf  = report["synfire_val"]
    df  = report["dfire_test"]
    fl  = report["flame_augmented"]

    print("\n" + "=" * 55)
    print("E-SYN-5 도메인 갭 평가 결과")
    print("=" * 55)
    print(f"  합성(SYN-FIRE) mAP50 : {sf.get('mAP50', 'N/A')}")
    print(f"  실제(D-Fire)   mAP50 : {df.get('mAP50', 'N/A')}")
    print(f"  증강(FLAME)    mAP50 : {fl.get('mAP50', 'N/A')}")
    print(
        f"  도메인 갭             : {gap.get('mAP50_drop', 'N/A')}  "
        f"({gap.get('relative_drop_pct', 'N/A')}%)  [{gap.get('severity', '?')}]"
    )
    print(f"  권고              : {gap.get('recommendation', '')}")
    print(f"  보고서 저장       : {out}")
    print("=" * 55)


def main():
    parser = argparse.ArgumentParser(
        description="E-SYN-5: 합성/실제 화재 탐지 도메인 갭 평가",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--weights", required=True,
        help="YOLOv8 가중치 파일 경로 (best.pt)",
    )
    parser.add_argument(
        "--synfire-val", default=None,
        help="SYN-FIRE val data.yaml 또는 이미지 디렉토리",
    )
    parser.add_argument(
        "--dfire-test", default=None,
        help="D-Fire test data.yaml 또는 이미지 디렉토리",
    )
    parser.add_argument(
        "--flame-dir", default=None,
        help="FLAME 생성 이미지 디렉토리",
    )
    parser.add_argument(
        "--output",
        default="training/runs/domain_gap_report.json",
        help="JSON 보고서 저장 경로 (기본: training/runs/domain_gap_report.json)",
    )
    parser.add_argument(
        "--imgsz", type=int, default=640,
        help="추론 이미지 크기 (기본: 640)",
    )
    args = parser.parse_args()

    # ultralytics 설치 확인
    try:
        from ultralytics import YOLO  # noqa: PLC0415
    except ImportError:
        print("[오류] ultralytics 패키지가 필요합니다: pip install ultralytics")
        sys.exit(1)

    weights_path = Path(args.weights)
    if not weights_path.exists():
        print(f"[오류] 가중치 파일을 찾을 수 없습니다: {weights_path}")
        sys.exit(1)

    model = YOLO(str(weights_path))
    print(f"[모델 로드] {weights_path}")

    # 기본값: 미제공 표시
    synfire = {"mAP50": 0.0, "note": "미제공"}
    dfire   = {"mAP50": 0.0, "note": "미제공"}
    flame   = {"mAP50": 0.0, "note": "미제공"}

    if args.synfire_val:
        synfire = evaluate_dataset(model, args.synfire_val, "SYN-FIRE (val)",    args.imgsz)
    if args.dfire_test:
        dfire   = evaluate_dataset(model, args.dfire_test,  "D-Fire (test)",     args.imgsz)
    if args.flame_dir:
        flame   = evaluate_dataset(model, args.flame_dir,   "FLAME (augmented)", args.imgsz)

    # 도메인 갭 계산
    gap = compute_domain_gap(synfire, dfire)

    # 보고서 생성 및 저장
    report = build_report(synfire, dfire, flame, gap, weights_path)
    write_report(report, args.output)


if __name__ == "__main__":
    main()
