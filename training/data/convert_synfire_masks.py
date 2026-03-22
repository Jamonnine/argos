#!/usr/bin/env python3
"""SYN-FIRE 세그멘테이션 마스크 → YOLO 바운딩박스 변환.

SYN-FIRE 구조:
  imgs/0.png, 1.png, ... (1920×1080 RGB)
  masks/0.png, 1.png, ... (1920×1080 RGBA, 흰색=화재 검은색=배경)

출력 (YOLO 형식):
  images/0.png (원본 복사)
  labels/0.txt (class_id center_x center_y width height, normalized)

사용법:
  python convert_synfire_masks.py --input ~/datasets/synfire --output ~/datasets/synfire_yolo
"""
import argparse
import os
import shutil
from pathlib import Path

import cv2
import numpy as np


def mask_to_yolo_bboxes(mask_path: str, class_id: int = 0) -> list:
    """바이너리 마스크 → YOLO 바운딩박스 리스트.

    Args:
        mask_path: 마스크 이미지 경로 (RGBA, 흰색=대상)
        class_id: YOLO 클래스 ID (0=fire)

    Returns:
        [(class_id, cx, cy, w, h), ...] — 좌표는 [0,1] 정규화
    """
    mask = cv2.imread(mask_path, cv2.IMREAD_UNCHANGED)
    if mask is None:
        return []

    h, w = mask.shape[:2]

    # RGB 채널 합산 → 흰색 영역 추출
    if mask.ndim == 3:
        gray = cv2.cvtColor(mask[:, :, :3], cv2.COLOR_BGR2GRAY)
    else:
        gray = mask

    # 이진화 (흰색 = 화재)
    _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

    # 연결 영역 찾기
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    bboxes = []
    min_area = 100  # 최소 면적 (노이즈 필터)

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area:
            continue

        x, y, bw, bh = cv2.boundingRect(contour)

        # YOLO 형식: center_x, center_y, width, height (정규화)
        cx = (x + bw / 2) / w
        cy = (y + bh / 2) / h
        nw = bw / w
        nh = bh / h

        bboxes.append((class_id, cx, cy, nw, nh))

    return bboxes


def convert_dataset(input_dir: Path, output_dir: Path):
    """전체 데이터셋 변환."""
    imgs_dir = input_dir / "imgs"
    masks_dir = input_dir / "masks"

    if not imgs_dir.exists():
        raise FileNotFoundError(f"이미지 디렉토리 없음: {imgs_dir}")
    if not masks_dir.exists():
        raise FileNotFoundError(f"마스크 디렉토리 없음: {masks_dir}")

    out_images = output_dir / "images"
    out_labels = output_dir / "labels"
    out_images.mkdir(parents=True, exist_ok=True)
    out_labels.mkdir(parents=True, exist_ok=True)

    img_files = sorted(imgs_dir.glob("*.png"))
    print(f"[SYN-FIRE] {len(img_files)}장 변환 시작...")

    stats = {"total": 0, "with_fire": 0, "no_fire": 0, "total_bboxes": 0}

    for img_file in img_files:
        stem = img_file.stem
        mask_file = masks_dir / f"{stem}.png"

        # 이미지 복사
        shutil.copy2(img_file, out_images / img_file.name)

        # 마스크 → YOLO 변환
        if mask_file.exists():
            bboxes = mask_to_yolo_bboxes(str(mask_file), class_id=0)
        else:
            bboxes = []

        # 라벨 파일 작성
        label_path = out_labels / f"{stem}.txt"
        with open(label_path, "w") as f:
            for cls, cx, cy, w, h in bboxes:
                f.write(f"{cls} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}\n")

        stats["total"] += 1
        if bboxes:
            stats["with_fire"] += 1
            stats["total_bboxes"] += len(bboxes)
        else:
            stats["no_fire"] += 1

        if stats["total"] % 200 == 0:
            print(f"  {stats['total']}/{len(img_files)}...")

    print(f"\n[결과]")
    print(f"  총 이미지: {stats['total']}")
    print(f"  화재 포함: {stats['with_fire']} ({stats['with_fire']/max(stats['total'],1)*100:.1f}%)")
    print(f"  화재 없음: {stats['no_fire']}")
    print(f"  총 바운딩박스: {stats['total_bboxes']}")
    print(f"  평균 bbox/이미지: {stats['total_bboxes']/max(stats['with_fire'],1):.1f}")
    print(f"  출력: {output_dir}")

    return stats


def main():
    parser = argparse.ArgumentParser(description="SYN-FIRE 마스크→YOLO 변환")
    parser.add_argument("--input", required=True, help="SYN-FIRE 루트 (imgs/, masks/)")
    parser.add_argument("--output", required=True, help="YOLO 출력 디렉토리")
    args = parser.parse_args()

    stats = convert_dataset(Path(args.input), Path(args.output))
    print(f"\n완료: {stats['total']}장 → {stats['total_bboxes']} bboxes")


if __name__ == "__main__":
    main()
