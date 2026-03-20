#!/usr/bin/env python3
"""
합성 화재 데이터셋 준비 파이프라인.

1단계: 데이터셋 다운로드 (SYN-FIRE + D-Fire)
2단계: YOLO 형식 통일 (images/ + labels/)
3단계: 혼합 비율 조정 (실제:합성 = 8:2, arXiv 2024 권고)
4단계: train/val 분할 (85:15)
5단계: data.yaml 생성

사용법:
  python prepare_fire_dataset.py --output-dir ./fire_mixed
  python prepare_fire_dataset.py --syn-ratio 0.2 --val-ratio 0.15
  python prepare_fire_dataset.py --skip-download --output-dir ./fire_mixed

데이터셋 출처:
  - SYN-FIRE: NVIDIA Omniverse 합성 실내 화재 2,030장 (FigShare, CC BY 4.0)
    https://figshare.com/articles/dataset/SYN-FIRE/21812734
  - D-Fire: 드론 화재/연기 탐지 21,527장 (GitHub/Kaggle, CC0)
    https://github.com/gaiasd/DFireDataset

라이선스 주의:
  - SYN-FIRE: CC BY 4.0 (출처 표기 필수)
  - D-Fire: CC0 Public Domain
  - AI-Hub 데이터는 별도 계약 필요 (본 스크립트에서 직접 다운로드 불가)
  - ultralytics(YOLOv8): AGPL-3.0 → 상업 배포 시 별도 라이선스 필요
"""

import argparse
import json
import os
import random
import shutil
import sys
import zipfile
from pathlib import Path


# ──────────────────────────────────────────
# 상수 및 설정
# ──────────────────────────────────────────

# SYN-FIRE FigShare 다운로드 URL (Article ID: 21812734)
SYNFIRE_FIGSHARE_URL = "https://figshare.com/ndownloader/articles/21812734/versions/1"

# D-Fire GitHub 릴리즈 (OneDrive 공식 미러)
DFIRE_ONEDRIVE_URL = (
    "https://1drv.ms/u/c/c0bd25b6b048b01d/EbLgD7bES4FDvUN37Grxn8QBF5gIBBc7YV2qklF08GCiBw"
)

# D-Fire Kaggle 대체 다운로드 (kaggle CLI 필요)
DFIRE_KAGGLE_DATASET = "sayedgamal99/smoke-fire-detection-yolo"

# 클래스 정의 (2종으로 통합)
CLASS_NAMES = {0: "fire", 1: "smoke"}

# SYN-FIRE COCO 카테고리 → YOLO 클래스 매핑
# SYN-FIRE는 "fire"와 "smoke" 두 카테고리를 사용
SYNFIRE_CATEGORY_MAP = {
    "fire": 0,
    "smoke": 1,
    "flame": 0,   # 동의어 처리
    "blaze": 0,
}

# AI-Hub 15종 → 2종 매핑 (AI-Hub 데이터를 수동으로 배치한 경우)
AIHUB_CLASS_MAP = {
    # fire 계열 (class_id → 0)
    0: 0,   # 화재
    1: 0,   # 불꽃
    2: 0,   # 전기화재_스파크
    3: 0,   # 전기화재_아크
    4: 0,   # 전기화재_단락
    5: 0,   # 전기화재_과열
    6: 0,   # 전기화재_접촉불량
    # smoke 계열 (class_id → 1)
    7: 1,   # 연기
    8: 1,   # 흑연
    9: 1,   # 백연
    # 기타 화재현장 객체 → fire로 통합 (conservative mapping)
    10: 0,  # 화재현장_잔불
    11: 0,  # 화재현장_불씨
    12: 1,  # 화재현장_연기잔류
    13: 0,  # 폭발
    14: 0,  # 열화상_고온체
}

# 랜덤 시드 (재현성)
RANDOM_SEED = 42


# ──────────────────────────────────────────
# 유틸리티 함수
# ──────────────────────────────────────────

def check_dependencies():
    """필수 패키지 설치 여부 확인."""
    missing = []
    try:
        import requests  # noqa: F401
    except ImportError:
        missing.append("requests")
    try:
        from PIL import Image  # noqa: F401
    except ImportError:
        missing.append("Pillow")
    if missing:
        print(f"[오류] 필수 패키지 미설치: {', '.join(missing)}")
        print(f"       pip install {' '.join(missing)}")
        sys.exit(1)


def download_file(url: str, dest_path: Path, desc: str = "") -> bool:
    """파일 다운로드 (진행률 표시 포함)."""
    try:
        import requests
    except ImportError:
        print("[오류] requests 패키지가 필요합니다: pip install requests")
        return False

    print(f"[다운로드] {desc or url}")
    print(f"  → {dest_path}")

    try:
        resp = requests.get(url, stream=True, timeout=60)
        resp.raise_for_status()

        total = int(resp.headers.get("content-length", 0))
        downloaded = 0
        chunk_size = 8192

        with open(dest_path, "wb") as f:
            for chunk in resp.iter_content(chunk_size=chunk_size):
                if chunk:
                    f.write(chunk)
                    downloaded += len(chunk)
                    if total > 0:
                        pct = downloaded / total * 100
                        # 10% 단위로 진행률 출력
                        if int(pct) % 10 == 0 and int(pct) != int((downloaded - chunk_size) / total * 100):
                            print(f"  {pct:.0f}%...", end="\r")

        print(f"  완료 ({downloaded / 1024 / 1024:.1f} MB)")
        return True

    except requests.exceptions.RequestException as e:
        print(f"[오류] 다운로드 실패: {e}")
        return False


def extract_zip(zip_path: Path, extract_dir: Path):
    """ZIP 파일 압축 해제."""
    print(f"[압축해제] {zip_path.name} → {extract_dir}")
    with zipfile.ZipFile(zip_path, "r") as zf:
        zf.extractall(extract_dir)
    print(f"  완료")


# ──────────────────────────────────────────
# 다운로드 함수
# ──────────────────────────────────────────

def download_synfire(download_dir: Path) -> Path:
    """SYN-FIRE 데이터셋 다운로드 (FigShare).

    Returns:
        압축 해제된 SYN-FIRE 루트 디렉토리 경로
    """
    synfire_dir = download_dir / "synfire_raw"
    if synfire_dir.exists():
        print(f"[스킵] SYN-FIRE 이미 존재: {synfire_dir}")
        return synfire_dir

    zip_path = download_dir / "synfire.zip"

    if not zip_path.exists():
        print("\n[SYN-FIRE 다운로드]")
        print("  출처: FigShare (NVIDIA Omniverse 합성 화재, CC BY 4.0)")
        print("  크기: 약 700MB")
        print()

        success = download_file(
            SYNFIRE_FIGSHARE_URL,
            zip_path,
            "SYN-FIRE (FigShare Article 21812734)",
        )

        if not success:
            # FigShare 직접 다운로드 실패 시 수동 안내
            print("\n[안내] FigShare 자동 다운로드 실패.")
            print("  수동 다운로드: https://figshare.com/articles/dataset/SYN-FIRE/21812734")
            print(f"  다운로드 후 이 경로에 저장: {zip_path}")
            raise RuntimeError("SYN-FIRE 다운로드 실패. 수동 다운로드 후 재시도하세요.")

    extract_zip(zip_path, synfire_dir)
    return synfire_dir


def download_dfire(download_dir: Path) -> Path:
    """D-Fire 데이터셋 다운로드.

    Kaggle CLI 설치 시 Kaggle에서 다운로드.
    없으면 수동 다운로드 안내.

    Returns:
        D-Fire 루트 디렉토리 경로 (images/ + labels/ 구조)
    """
    dfire_dir = download_dir / "dfire_raw"
    if dfire_dir.exists():
        print(f"[스킵] D-Fire 이미 존재: {dfire_dir}")
        return dfire_dir

    print("\n[D-Fire 다운로드]")
    print("  출처: GitHub DFireDataset (CC0, 드론 화재/연기)")
    print("  크기: 약 2.5GB")
    print()

    # Kaggle CLI 시도
    kaggle_available = shutil.which("kaggle") is not None
    if kaggle_available:
        print("  Kaggle CLI 감지됨. Kaggle에서 다운로드 시도...")
        dfire_dir.mkdir(parents=True, exist_ok=True)
        ret = os.system(
            f'kaggle datasets download -d {DFIRE_KAGGLE_DATASET} -p "{dfire_dir}" --unzip'
        )
        if ret == 0:
            print("  Kaggle 다운로드 완료")
            return dfire_dir
        print("  Kaggle 다운로드 실패. 수동 안내로 전환...")

    # 수동 다운로드 안내
    print("\n[안내] D-Fire 수동 다운로드 필요:")
    print("  1) Kaggle: https://www.kaggle.com/datasets/sayedgamal99/smoke-fire-detection-yolo")
    print("  2) OneDrive (공식 미러):")
    print(f"     {DFIRE_ONEDRIVE_URL}")
    print(f"  3) 압축 해제 후 아래 경로에 배치:")
    print(f"     {dfire_dir}/")
    print("       ├── images/train/  val/  test/")
    print("       └── labels/train/  val/  test/")
    print()

    raise RuntimeError("D-Fire 수동 다운로드가 필요합니다. 위 안내를 따라 데이터를 배치한 후 --skip-download 옵션으로 재시도하세요.")


# ──────────────────────────────────────────
# COCO → YOLO 변환 (SYN-FIRE)
# ──────────────────────────────────────────

def coco_bbox_to_yolo(bbox, img_w: int, img_h: int):
    """COCO [x_min, y_min, width, height] → YOLO [cx, cy, w, h] 정규화."""
    x_min, y_min, w, h = bbox
    cx = (x_min + w / 2) / img_w
    cy = (y_min + h / 2) / img_h
    nw = w / img_w
    nh = h / img_h
    # 경계 클리핑 (0~1 범위 보장)
    cx = max(0.0, min(1.0, cx))
    cy = max(0.0, min(1.0, cy))
    nw = max(0.0, min(1.0, nw))
    nh = max(0.0, min(1.0, nh))
    return cx, cy, nw, nh


def convert_synfire_coco_to_yolo(synfire_dir: Path, output_dir: Path) -> list:
    """SYN-FIRE COCO 어노테이션을 YOLO 형식으로 변환.

    SYN-FIRE 구조 (예상):
      synfire_raw/
        images/  (또는 JPEGImages/)
        annotations.json  (또는 instances_*.json)

    Returns:
        변환된 샘플 목록 [{"image": Path, "label": Path}, ...]
    """
    from PIL import Image as PILImage

    print("\n[변환] SYN-FIRE COCO → YOLO")

    # 어노테이션 파일 탐색 (COCO 형식 JSON)
    json_files = list(synfire_dir.rglob("*.json"))
    if not json_files:
        raise FileNotFoundError(f"SYN-FIRE 어노테이션 JSON 없음: {synfire_dir}")

    # 가장 큰 JSON 파일 = 메인 어노테이션
    annotation_file = max(json_files, key=lambda p: p.stat().st_size)
    print(f"  어노테이션 파일: {annotation_file}")

    with open(annotation_file, "r", encoding="utf-8") as f:
        coco_data = json.load(f)

    # COCO 카테고리 → YOLO 클래스 매핑 구성
    cat_to_yolo = {}
    for cat in coco_data.get("categories", []):
        name = cat["name"].lower()
        if name in SYNFIRE_CATEGORY_MAP:
            cat_to_yolo[cat["id"]] = SYNFIRE_CATEGORY_MAP[name]
        else:
            # 알 수 없는 카테고리는 fire(0)으로 보수적 처리
            print(f"  [경고] 알 수 없는 카테고리 '{name}' → fire(0) 매핑")
            cat_to_yolo[cat["id"]] = 0

    # 이미지 ID → 파일명 매핑
    id_to_image = {img["id"]: img for img in coco_data.get("images", [])}

    # 이미지별 어노테이션 그룹화
    from collections import defaultdict
    img_annotations = defaultdict(list)
    for ann in coco_data.get("annotations", []):
        img_annotations[ann["image_id"]].append(ann)

    # 출력 디렉토리 생성
    out_img_dir = output_dir / "synfire" / "images"
    out_lbl_dir = output_dir / "synfire" / "labels"
    out_img_dir.mkdir(parents=True, exist_ok=True)
    out_lbl_dir.mkdir(parents=True, exist_ok=True)

    converted = []
    skipped = 0

    for img_id, img_info in id_to_image.items():
        # 이미지 파일 탐색
        file_name = img_info.get("file_name", "")
        img_path = None
        for candidate in synfire_dir.rglob(Path(file_name).name):
            img_path = candidate
            break

        if img_path is None or not img_path.exists():
            skipped += 1
            continue

        # 이미지 크기 확인 (COCO 메타데이터 우선, 없으면 PIL)
        img_w = img_info.get("width", 0)
        img_h = img_info.get("height", 0)
        if img_w == 0 or img_h == 0:
            try:
                with PILImage.open(img_path) as pil_img:
                    img_w, img_h = pil_img.size
            except Exception:
                skipped += 1
                continue

        # YOLO 라벨 생성
        annotations = img_annotations.get(img_id, [])
        yolo_lines = []
        for ann in annotations:
            cat_id = ann.get("category_id")
            yolo_class = cat_to_yolo.get(cat_id, 0)
            bbox = ann.get("bbox", [])
            if len(bbox) != 4:
                continue
            cx, cy, w, h = coco_bbox_to_yolo(bbox, img_w, img_h)
            # 너무 작은 박스 필터링 (노이즈 제거)
            if w < 0.001 or h < 0.001:
                continue
            yolo_lines.append(f"{yolo_class} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}")

        # 어노테이션 없는 이미지는 네거티브 샘플로 포함 (빈 라벨)
        # 합성 데이터는 모두 화재 이미지여야 하므로 어노테이션 없으면 스킵
        if not yolo_lines:
            skipped += 1
            continue

        # 파일 복사 및 라벨 저장
        dest_img = out_img_dir / img_path.name
        dest_lbl = out_lbl_dir / (img_path.stem + ".txt")

        shutil.copy2(img_path, dest_img)
        with open(dest_lbl, "w", encoding="utf-8") as f:
            f.write("\n".join(yolo_lines))

        converted.append({"image": dest_img, "label": dest_lbl, "source": "synfire"})

    print(f"  변환 완료: {len(converted)}장 (스킵: {skipped}장)")
    return converted


# ──────────────────────────────────────────
# D-Fire 처리 (이미 YOLO 형식)
# ──────────────────────────────────────────

def collect_dfire_samples(dfire_dir: Path, output_dir: Path) -> list:
    """D-Fire 데이터셋 샘플 수집 (이미 YOLO 형식).

    D-Fire 구조:
      dfire_raw/
        images/train/  val/  test/
        labels/train/  val/  test/

    Returns:
        샘플 목록 [{"image": Path, "label": Path, "source": "dfire"}, ...]
    """
    print("\n[수집] D-Fire 샘플")

    out_img_dir = output_dir / "dfire" / "images"
    out_lbl_dir = output_dir / "dfire" / "labels"
    out_img_dir.mkdir(parents=True, exist_ok=True)
    out_lbl_dir.mkdir(parents=True, exist_ok=True)

    samples = []
    image_exts = {".jpg", ".jpeg", ".png", ".bmp"}

    # train/val/test 모두 수집 (나중에 재분할)
    for split in ["train", "val", "test"]:
        img_split_dir = dfire_dir / "images" / split
        lbl_split_dir = dfire_dir / "labels" / split

        if not img_split_dir.exists():
            # 구조가 다른 경우 직접 탐색
            img_split_dir = dfire_dir / split / "images"
            lbl_split_dir = dfire_dir / split / "labels"

        if not img_split_dir.exists():
            continue

        for img_path in img_split_dir.iterdir():
            if img_path.suffix.lower() not in image_exts:
                continue

            lbl_path = lbl_split_dir / (img_path.stem + ".txt")

            # 라벨 없는 이미지는 네거티브 샘플 (빈 라벨)
            if not lbl_path.exists():
                # 빈 라벨 파일 생성 (YOLO는 라벨 없음=배경으로 처리)
                dest_img = out_img_dir / img_path.name
                dest_lbl = out_lbl_dir / (img_path.stem + ".txt")
                shutil.copy2(img_path, dest_img)
                dest_lbl.write_text("")
                samples.append({"image": dest_img, "label": dest_lbl, "source": "dfire"})
                continue

            dest_img = out_img_dir / img_path.name
            dest_lbl = out_lbl_dir / (img_path.stem + ".txt")

            shutil.copy2(img_path, dest_img)
            shutil.copy2(lbl_path, dest_lbl)
            samples.append({"image": dest_img, "label": dest_lbl, "source": "dfire"})

    print(f"  수집 완료: {len(samples)}장")
    return samples


def collect_aihub_samples(aihub_dir: Path, output_dir: Path) -> list:
    """AI-Hub 데이터 수집 (수동 배치 시).

    AI-Hub 데이터는 직접 다운로드가 불가능하여 수동 배치 후 처리.
    15종 클래스 → fire/smoke 2종으로 재매핑.

    AI-Hub 예상 구조:
      aihub_dir/
        images/  (JPG/PNG)
        labels/  (YOLO 형식 TXT, 클래스 0~14)

    Returns:
        샘플 목록
    """
    if not aihub_dir.exists():
        return []

    print(f"\n[수집] AI-Hub 데이터: {aihub_dir}")

    out_img_dir = output_dir / "aihub" / "images"
    out_lbl_dir = output_dir / "aihub" / "labels"
    out_img_dir.mkdir(parents=True, exist_ok=True)
    out_lbl_dir.mkdir(parents=True, exist_ok=True)

    samples = []
    image_exts = {".jpg", ".jpeg", ".png"}

    img_dir = aihub_dir / "images"
    lbl_dir = aihub_dir / "labels"

    if not img_dir.exists():
        print("  [경고] images/ 디렉토리 없음")
        return []

    for img_path in img_dir.rglob("*"):
        if img_path.suffix.lower() not in image_exts:
            continue

        lbl_path = lbl_dir / img_path.relative_to(img_dir).with_suffix(".txt")
        if not lbl_path.exists():
            continue

        # 클래스 재매핑 (15종 → 2종)
        new_lines = []
        with open(lbl_path, "r") as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) < 5:
                    continue
                old_class = int(parts[0])
                new_class = AIHUB_CLASS_MAP.get(old_class, 0)
                new_lines.append(f"{new_class} {' '.join(parts[1:])}")

        if not new_lines:
            continue

        dest_img = out_img_dir / img_path.name
        dest_lbl = out_lbl_dir / (img_path.stem + ".txt")

        shutil.copy2(img_path, dest_img)
        with open(dest_lbl, "w") as f:
            f.write("\n".join(new_lines))

        samples.append({"image": dest_img, "label": dest_lbl, "source": "aihub"})

    print(f"  수집 완료: {len(samples)}장")
    return samples


# ──────────────────────────────────────────
# 혼합 비율 조정 및 분할
# ──────────────────────────────────────────

def mix_and_split(
    real_samples: list,
    synthetic_samples: list,
    output_dir: Path,
    syn_ratio: float = 0.2,
    val_ratio: float = 0.15,
) -> dict:
    """실제 데이터와 합성 데이터를 목표 비율로 혼합하고 train/val 분할.

    Args:
        real_samples: 실제 데이터 샘플 목록 (D-Fire + AI-Hub)
        synthetic_samples: 합성 데이터 샘플 목록 (SYN-FIRE)
        output_dir: 출력 디렉토리
        syn_ratio: 합성 데이터 비율 (0.2 = 20%)
        val_ratio: 검증 세트 비율 (0.15 = 15%)

    Returns:
        분할 결과 통계 딕셔너리
    """
    print(f"\n[혼합] 실제:{1-syn_ratio:.0%} / 합성:{syn_ratio:.0%}")
    print(f"  실제 데이터: {len(real_samples)}장")
    print(f"  합성 데이터: {len(synthetic_samples)}장")

    random.seed(RANDOM_SEED)

    # 혼합 비율 계산
    # 목표: 전체 N장 중 syn_ratio 비율이 합성 데이터
    # 실제 데이터를 기준으로 전체 크기 결정
    n_real = len(real_samples)
    n_syn = len(synthetic_samples)

    if n_real == 0:
        raise ValueError("실제 데이터가 없습니다. D-Fire 또는 AI-Hub 데이터를 준비하세요.")

    # 합성 데이터가 목표 비율을 초과하면 다운샘플링
    target_syn = int(n_real * syn_ratio / (1 - syn_ratio))
    if n_syn > target_syn:
        print(f"  합성 데이터 다운샘플: {n_syn} → {target_syn}장")
        synthetic_samples = random.sample(synthetic_samples, target_syn)
    elif n_syn < target_syn and n_syn > 0:
        # 합성 데이터가 부족하면 실제 데이터를 다운샘플링하여 비율 맞춤
        target_real = int(n_syn * (1 - syn_ratio) / syn_ratio)
        if target_real < n_real:
            print(f"  실제 데이터 다운샘플: {n_real} → {target_real}장 (합성 부족)")
            real_samples = random.sample(real_samples, target_real)

    # 전체 혼합
    all_samples = real_samples + synthetic_samples
    random.shuffle(all_samples)

    total = len(all_samples)
    n_val = max(1, int(total * val_ratio))
    n_train = total - n_val

    train_samples = all_samples[:n_train]
    val_samples = all_samples[n_train:]

    print(f"  총 {total}장 → 학습 {n_train}장 / 검증 {n_val}장")

    # 출력 디렉토리 구성
    for split_name, split_samples in [("train", train_samples), ("val", val_samples)]:
        split_img_dir = output_dir / "images" / split_name
        split_lbl_dir = output_dir / "labels" / split_name
        split_img_dir.mkdir(parents=True, exist_ok=True)
        split_lbl_dir.mkdir(parents=True, exist_ok=True)

        for sample in split_samples:
            img_src = Path(sample["image"])
            lbl_src = Path(sample["label"])

            # 파일명 충돌 방지: source 접두사 추가
            source = sample.get("source", "unknown")
            dest_name = f"{source}_{img_src.name}"
            dest_img = split_img_dir / dest_name
            dest_lbl = split_lbl_dir / f"{source}_{img_src.stem}.txt"

            if img_src != dest_img:
                shutil.copy2(img_src, dest_img)
            if lbl_src != dest_lbl:
                shutil.copy2(lbl_src, dest_lbl)

    # 통계 계산
    n_syn_in_train = sum(1 for s in train_samples if s.get("source") == "synfire")
    n_syn_in_val = sum(1 for s in val_samples if s.get("source") == "synfire")
    actual_syn_ratio = len(synthetic_samples) / total if total > 0 else 0

    return {
        "total": total,
        "train": n_train,
        "val": n_val,
        "synthetic": len(synthetic_samples),
        "real": len(real_samples),
        "actual_syn_ratio": actual_syn_ratio,
        "syn_in_train": n_syn_in_train,
        "syn_in_val": n_syn_in_val,
    }


# ──────────────────────────────────────────
# 클래스 분포 통계
# ──────────────────────────────────────────

def compute_class_distribution(labels_dir: Path) -> dict:
    """라벨 디렉토리에서 클래스별 분포 계산."""
    counts = {0: 0, 1: 0}  # fire, smoke
    total_images = 0
    empty_images = 0

    for lbl_file in labels_dir.rglob("*.txt"):
        total_images += 1
        lines = lbl_file.read_text(encoding="utf-8").strip().splitlines()
        if not lines:
            empty_images += 1
            continue
        for line in lines:
            parts = line.strip().split()
            if parts:
                cls = int(parts[0])
                counts[cls] = counts.get(cls, 0) + 1

    return {
        "total_images": total_images,
        "empty_images": empty_images,
        "fire_annotations": counts.get(0, 0),
        "smoke_annotations": counts.get(1, 0),
    }


# ──────────────────────────────────────────
# data.yaml 생성
# ──────────────────────────────────────────

def generate_data_yaml(output_dir: Path, stats: dict):
    """Ultralytics 호환 data.yaml 생성."""
    yaml_content = f"""# ARGOS 혼합 화재 데이터셋 (실제 + 합성)
# 생성: prepare_fire_dataset.py
# 총 이미지: {stats['total']}장 (학습 {stats['train']} / 검증 {stats['val']})
# 합성 비율: {stats['actual_syn_ratio']:.1%} (SYN-FIRE {stats['synthetic']}장)
# 실제 데이터: {stats['real']}장 (D-Fire + AI-Hub)

path: {output_dir.resolve().as_posix()}
train: images/train
val: images/val

# 클래스 (2종, AI-Hub 15종 → 통합)
nc: 2
names:
  0: fire
  1: smoke

# 데이터 출처
# - SYN-FIRE: NVIDIA Omniverse 합성 (FigShare, CC BY 4.0)
# - D-Fire: 드론 화재/연기 (GitHub gaiasd, CC0)
# - AI-Hub: 화재 발생 예측 영상 (선택, 공공AI 라이선스)
#
# arXiv 2024 권고: 합성:실제 = 2:8 이상 비율 유지
# 랜덤 시드: {RANDOM_SEED} (재현성 보장)
"""
    yaml_path = output_dir / "data.yaml"
    yaml_path.write_text(yaml_content, encoding="utf-8")
    print(f"\n[생성] data.yaml → {yaml_path}")
    return yaml_path


# ──────────────────────────────────────────
# 메인 파이프라인
# ──────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="ARGOS 합성 화재 데이터셋 준비 파이프라인",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("./fire_mixed"),
        help="출력 디렉토리 (기본: ./fire_mixed)",
    )
    parser.add_argument(
        "--download-dir",
        type=Path,
        default=Path("./downloads"),
        help="다운로드 임시 디렉토리 (기본: ./downloads)",
    )
    parser.add_argument(
        "--syn-ratio",
        type=float,
        default=0.2,
        help="합성 데이터 비율 (기본: 0.2 = 20%%)",
    )
    parser.add_argument(
        "--val-ratio",
        type=float,
        default=0.15,
        help="검증 세트 비율 (기본: 0.15 = 15%%)",
    )
    parser.add_argument(
        "--skip-download",
        action="store_true",
        help="다운로드 스킵 (이미 다운로드된 경우)",
    )
    parser.add_argument(
        "--dfire-dir",
        type=Path,
        default=None,
        help="D-Fire 데이터 경로 (수동 지정, --skip-download와 함께 사용)",
    )
    parser.add_argument(
        "--synfire-dir",
        type=Path,
        default=None,
        help="SYN-FIRE 데이터 경로 (수동 지정, --skip-download와 함께 사용)",
    )
    parser.add_argument(
        "--aihub-dir",
        type=Path,
        default=None,
        help="AI-Hub 데이터 경로 (선택, 수동 배치 후 지정)",
    )
    args = parser.parse_args()

    # 입력 검증
    if not (0 < args.syn_ratio < 1):
        print("[오류] --syn-ratio는 0~1 사이 값이어야 합니다.")
        sys.exit(1)
    if not (0 < args.val_ratio < 0.5):
        print("[오류] --val-ratio는 0~0.5 사이 값이어야 합니다.")
        sys.exit(1)

    # 의존성 확인
    check_dependencies()

    print("=" * 60)
    print("ARGOS 합성 화재 데이터셋 준비 파이프라인")
    print("=" * 60)
    print(f"출력 디렉토리: {args.output_dir.resolve()}")
    print(f"합성 데이터 비율: {args.syn_ratio:.0%}")
    print(f"검증 세트 비율: {args.val_ratio:.0%}")
    print()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    args.download_dir.mkdir(parents=True, exist_ok=True)

    # ── 1단계: 데이터셋 다운로드 ──
    print("[1단계] 데이터셋 다운로드")

    synfire_raw_dir = args.synfire_dir
    dfire_raw_dir = args.dfire_dir

    if not args.skip_download:
        # SYN-FIRE 다운로드
        try:
            synfire_raw_dir = download_synfire(args.download_dir)
        except RuntimeError as e:
            print(f"\n[경고] {e}")
            print("SYN-FIRE 없이 D-Fire만으로 진행합니다.")
            synfire_raw_dir = None

        # D-Fire 다운로드
        try:
            dfire_raw_dir = download_dfire(args.download_dir)
        except RuntimeError as e:
            print(f"\n[오류] {e}")
            sys.exit(1)
    else:
        # --skip-download 시 기본 경로 설정
        if synfire_raw_dir is None:
            synfire_raw_dir = args.download_dir / "synfire_raw"
        if dfire_raw_dir is None:
            dfire_raw_dir = args.download_dir / "dfire_raw"

        if not dfire_raw_dir.exists():
            print(f"[오류] D-Fire 경로 없음: {dfire_raw_dir}")
            print("  --dfire-dir 옵션으로 경로를 지정하세요.")
            sys.exit(1)

    # ── 2단계: YOLO 형식 통일 ──
    print("\n[2단계] YOLO 형식 통일")

    intermediate_dir = args.output_dir / "_intermediate"

    synthetic_samples = []
    if synfire_raw_dir and synfire_raw_dir.exists():
        try:
            synthetic_samples = convert_synfire_coco_to_yolo(synfire_raw_dir, intermediate_dir)
        except Exception as e:
            print(f"[경고] SYN-FIRE 변환 실패: {e}")
            print("  SYN-FIRE 없이 계속합니다.")
            synthetic_samples = []
    else:
        print("[스킵] SYN-FIRE 디렉토리 없음")

    # D-Fire 수집
    real_samples = collect_dfire_samples(dfire_raw_dir, intermediate_dir)

    # AI-Hub 수집 (선택)
    if args.aihub_dir:
        aihub_samples = collect_aihub_samples(args.aihub_dir, intermediate_dir)
        real_samples.extend(aihub_samples)

    if not real_samples:
        print("[오류] 실제 데이터가 없습니다.")
        sys.exit(1)

    # ── 3~4단계: 혼합 비율 조정 + 분할 ──
    print("\n[3~4단계] 혼합 비율 조정 + train/val 분할")
    stats = mix_and_split(
        real_samples=real_samples,
        synthetic_samples=synthetic_samples,
        output_dir=args.output_dir,
        syn_ratio=args.syn_ratio,
        val_ratio=args.val_ratio,
    )

    # ── 5단계: data.yaml 생성 ──
    print("\n[5단계] data.yaml 생성")
    yaml_path = generate_data_yaml(args.output_dir, stats)

    # ── 통계 출력 ──
    print("\n" + "=" * 60)
    print("준비 완료 — 최종 통계")
    print("=" * 60)
    print(f"  총 이미지:      {stats['total']:,}장")
    print(f"  학습 세트:      {stats['train']:,}장")
    print(f"  검증 세트:      {stats['val']:,}장")
    print(f"  실제 데이터:    {stats['real']:,}장 (D-Fire 등)")
    print(f"  합성 데이터:    {stats['synthetic']:,}장 (SYN-FIRE)")
    print(f"  실제 비율:      {1 - stats['actual_syn_ratio']:.1%}")
    print(f"  합성 비율:      {stats['actual_syn_ratio']:.1%}")
    print()

    # 클래스 분포
    train_lbl_dir = args.output_dir / "labels" / "train"
    if train_lbl_dir.exists():
        dist = compute_class_distribution(args.output_dir / "labels")
        print("클래스 분포:")
        print(f"  fire  (0): {dist['fire_annotations']:,}개 어노테이션")
        print(f"  smoke (1): {dist['smoke_annotations']:,}개 어노테이션")
        total_ann = dist["fire_annotations"] + dist["smoke_annotations"]
        if total_ann > 0:
            print(f"  fire/smoke 비율: {dist['fire_annotations']/total_ann:.1%} / {dist['smoke_annotations']/total_ann:.1%}")

    print()
    print(f"data.yaml: {yaml_path}")
    print()
    print("다음 단계:")
    print(f"  python finetune_fire_yolo.py --data {yaml_path}")
    print()

    # arXiv 2024 비율 검증 경고
    if stats["actual_syn_ratio"] < 0.2 and stats["synthetic"] > 0:
        print(f"[주의] 합성 비율 {stats['actual_syn_ratio']:.1%} < 20% (arXiv 2024 권고 최소값)")
        print("  --syn-ratio 0.2 이상을 권장합니다.")

    print()
    print("[라이선스 주의]")
    print("  - SYN-FIRE: CC BY 4.0 (출처 표기 필수)")
    print("  - D-Fire: CC0 Public Domain")
    print("  - AI-Hub: 공공AI 라이선스 (사전 계약 필요)")
    print("  - ultralytics: AGPL-3.0 (상업 배포 시 별도 라이선스)")


if __name__ == "__main__":
    main()
