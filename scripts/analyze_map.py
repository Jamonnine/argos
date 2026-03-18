#!/usr/bin/env python3
"""
ARGOS OccupancyGrid 맵 분석 유틸리티
=====================================
저장된 PGM 맵 파일을 읽어 프론티어를 감지하고,
min_frontier_size 파라미터에 따른 유효 프론티어 수를 시뮬레이션한다.

사용법:
  # PGM 파일 분석
  python3 scripts/analyze_map.py /tmp/argos_map.pgm

  # YAML 메타 파일 있을 때 (nav2_map_server 형식)
  python3 scripts/analyze_map.py /tmp/argos_map.yaml

  # min_frontier_size 범위 변경 (기본 1~15)
  python3 scripts/analyze_map.py /tmp/argos_map.pgm --size-range 1 20

  # 프론티어 클러스터 시각화 저장 (PNG)
  python3 scripts/analyze_map.py /tmp/argos_map.pgm --viz /tmp/frontiers.png

PGM 관례 (nav2_map_server 기준):
  254 → free (탐색됨)
  0   → occupied (장애물)
  205 → unknown  (미탐색)

ROS OccupancyGrid save_map 관례:
  254 → free
  0   → occupied
  128 → unknown
"""

import argparse
import sys
from pathlib import Path

import numpy as np


# ── PGM 로더 ─────────────────────────────────────────────────────────────────

def load_pgm(path: str) -> tuple[np.ndarray, dict]:
    """PGM 파일 로드.

    Returns:
        img: uint8 numpy 배열 (H×W)
        meta: {'width', 'height', 'maxval'}
    """
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(f'파일 없음: {path}')

    with open(p, 'rb') as f:
        # 헤더 파싱 (P5 = 바이너리 그레이스케일)
        magic = f.readline().decode('ascii').strip()
        if magic not in ('P5', 'P2'):
            raise ValueError(f'PGM 아님 (magic={magic}). P5/P2만 지원.')

        # 주석 행 건너뜀
        while True:
            line = f.readline().decode('ascii').strip()
            if not line.startswith('#'):
                break
        w, h = map(int, line.split())
        maxval = int(f.readline().decode('ascii').strip())

        if magic == 'P5':
            raw = np.frombuffer(f.read(), dtype=np.uint8)
        else:  # P2 (텍스트 형식)
            raw = np.array(f.read().decode('ascii').split(), dtype=np.uint8)

    img = raw[:h * w].reshape(h, w)
    return img, {'width': w, 'height': h, 'maxval': maxval}


def load_yaml_map(yaml_path: str) -> tuple[np.ndarray, dict]:
    """nav2 map_server YAML 메타 파일에서 PGM 경로 추출 후 로드."""
    import re
    p = Path(yaml_path)
    content = p.read_text(encoding='utf-8')

    image_match = re.search(r'image\s*:\s*(.+)', content)
    if not image_match:
        raise ValueError('YAML에 image 필드 없음')

    image_file = image_match.group(1).strip()
    if not Path(image_file).is_absolute():
        image_file = str(p.parent / image_file)

    resolution = 0.05
    res_match = re.search(r'resolution\s*:\s*([\d.]+)', content)
    if res_match:
        resolution = float(res_match.group(1))

    img, meta = load_pgm(image_file)
    meta['resolution'] = resolution
    meta['yaml_path'] = yaml_path
    return img, meta


# ── OccupancyGrid 변환 ────────────────────────────────────────────────────────

def pgm_to_occupancy(img: np.ndarray) -> np.ndarray:
    """PGM uint8 → OccupancyGrid int8 (-1/0/100) 변환.

    nav2_map_server 기준:
      254/255 → free (0)
      0~5     → occupied (100)
      기타    → unknown (-1)
    """
    occ = np.full(img.shape, -1, dtype=np.int8)  # 기본: unknown
    occ[img >= 250] = 0    # free
    occ[img <= 5] = 100    # occupied
    return occ


# ── 프론티어 감지 ─────────────────────────────────────────────────────────────

def detect_frontiers(occ: np.ndarray) -> tuple[np.ndarray, int]:
    """OccupancyGrid에서 프론티어 클러스터 감지.

    frontier_explorer_node.py의 detect_frontiers()와 동일한 알고리즘.

    Returns:
        labeled: 클러스터 레이블 배열 (0=배경)
        n_labels: 클러스터 수
    """
    try:
        import cv2
    except ImportError:
        print('[ERROR] OpenCV(cv2)가 필요합니다: pip install opencv-python-headless')
        sys.exit(1)

    free = (occ == 0).astype(np.uint8)
    unknown = (occ == -1).astype(np.uint8)

    kernel = np.ones((3, 3), dtype=np.uint8)
    unknown_dilated = cv2.dilate(unknown, kernel, iterations=1)
    frontier_mask = (free & unknown_dilated.astype(bool)).astype(np.uint8) * 255

    n_labels, labeled = cv2.connectedComponents(frontier_mask)
    return labeled, max(0, n_labels - 1)


# ── 분석 ─────────────────────────────────────────────────────────────────────

def analyze_frontiers(labeled: np.ndarray, n_total: int, occ: np.ndarray,
                      resolution: float = 0.05) -> list[dict]:
    """각 프론티어 클러스터의 크기·위치·면적 계산.

    Returns:
        list of {id, size_cells, area_m2, centroid_px, centroid_m}
    """
    clusters = []
    for i in range(1, n_total + 1):
        cells = np.argwhere(labeled == i)
        if len(cells) == 0:
            continue
        cy, cx = cells.mean(axis=0)
        clusters.append({
            'id': i,
            'size_cells': len(cells),
            'area_m2': len(cells) * resolution ** 2,
            'centroid_px': (float(cx), float(cy)),
            'centroid_m': (float(cx * resolution), float(cy * resolution)),
        })

    # 크기 내림차순 정렬
    clusters.sort(key=lambda c: c['size_cells'], reverse=True)
    return clusters


def simulate_min_size_filter(clusters: list[dict],
                              size_range: tuple[int, int]) -> list[dict]:
    """min_frontier_size 1~N 범위에서 유효 프론티어 수 시뮬레이션.

    Returns:
        list of {min_size, valid_count, valid_pct}
    """
    results = []
    total = len(clusters)
    for s in range(size_range[0], size_range[1] + 1):
        valid = sum(1 for c in clusters if c['size_cells'] >= s)
        results.append({
            'min_size': s,
            'valid_count': valid,
            'valid_pct': valid / total * 100 if total > 0 else 0.0,
        })
    return results


# ── 시각화 ────────────────────────────────────────────────────────────────────

def save_viz(img: np.ndarray, labeled: np.ndarray, clusters: list[dict],
             out_path: str, min_size: int = 3):
    """프론티어 클러스터를 컬러로 오버레이한 PNG 저장."""
    try:
        from PIL import Image, ImageDraw, ImageFont
    except ImportError:
        print('[WARN] Pillow 없음. 시각화 저장 스킵. pip install Pillow')
        return

    # RGB 변환
    rgb = np.stack([img, img, img], axis=-1)

    # 유효 프론티어: 초록, 소형: 노랑
    for c in clusters:
        cells = np.argwhere(labeled == c['id'])
        color = (0, 255, 80) if c['size_cells'] >= min_size else (255, 220, 0)
        for row, col in cells:
            rgb[row, col] = color

    pil_img = Image.fromarray(rgb)
    draw = ImageDraw.Draw(pil_img)

    # centroid 번호 표시
    for c in clusters:
        if c['size_cells'] < min_size:
            continue
        cx, cy = c['centroid_px']
        draw.ellipse([(cx - 3, cy - 3), (cx + 3, cy + 3)], fill=(255, 0, 0))
        draw.text((cx + 4, cy - 6), str(c['id']), fill=(255, 0, 0))

    out = Path(out_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    pil_img.save(out)
    print(f'[VIZ SAVED] {out}')


# ── 출력 포맷터 ───────────────────────────────────────────────────────────────

def print_map_stats(occ: np.ndarray, meta: dict):
    h, w = occ.shape
    total = occ.size
    free = int(np.sum(occ == 0))
    occupied = int(np.sum(occ == 100))
    unknown = int(np.sum(occ == -1))
    explored = free + occupied
    res = meta.get('resolution', 0.05)

    print('=' * 60)
    print('  맵 통계')
    print('=' * 60)
    print(f'  크기:       {w} x {h} 픽셀  ({w*res:.1f} x {h*res:.1f} m)')
    print(f'  해상도:     {res} m/cell')
    print(f'  전체 셀:    {total:,}')
    print(f'  free:       {free:,}  ({free/total*100:.1f}%)')
    print(f'  occupied:   {occupied:,}  ({occupied/total*100:.1f}%)')
    print(f'  unknown:    {unknown:,}  ({unknown/total*100:.1f}%)')
    print(f'  탐색률:     {explored/total*100:.1f}%')
    print()


def print_frontier_list(clusters: list[dict], min_size: int, top_n: int = 20):
    print('=' * 60)
    print(f'  프론티어 클러스터 (min_size={min_size} 기준 유효: '
          f'{sum(1 for c in clusters if c["size_cells"] >= min_size)}개 / '
          f'전체 {len(clusters)}개)')
    print('=' * 60)
    print(f'  {"ID":>4}  {"크기(셀)":>8}  {"면적(m²)":>8}  '
          f'{"중심(px)":>14}  유효')
    print('-' * 60)
    for c in clusters[:top_n]:
        valid = 'O' if c['size_cells'] >= min_size else 'x'
        cx, cy = c['centroid_px']
        print(f'  {c["id"]:>4}  {c["size_cells"]:>8d}  '
              f'{c["area_m2"]:>8.3f}  '
              f'  ({cx:>5.1f},{cy:>5.1f})  {valid}')
    if len(clusters) > top_n:
        print(f'  ... ({len(clusters) - top_n}개 생략)')
    print()


def print_size_simulation(sim_results: list[dict]):
    print('=' * 60)
    print('  min_frontier_size 시뮬레이션')
    print('=' * 60)
    print(f'  {"min_size":>8}  {"유효수":>6}  {"유효%":>6}  권장')
    print('-' * 60)
    prev_valid = None
    for r in sim_results:
        s = r['min_size']
        v = r['valid_count']
        pct = r['valid_pct']
        # 권장 표시: 유효 수가 5~30개 사이인 구간
        recommend = '<-- 권장' if 5 <= v <= 30 else ''
        # 변화 지점 강조
        change = ' (↓변화)' if prev_valid is not None and v < prev_valid else ''
        print(f'  {s:>8d}  {v:>6d}  {pct:>5.1f}%  {recommend}{change}')
        prev_valid = v
    print()


# ── 메인 ─────────────────────────────────────────────────────────────────────

def parse_args():
    parser = argparse.ArgumentParser(
        description='ARGOS OccupancyGrid 맵 분석 유틸리티')
    parser.add_argument('map_path', type=str,
                        help='PGM 또는 YAML 맵 파일 경로')
    parser.add_argument(
        '--size-range', type=int, nargs=2, default=[1, 15],
        metavar=('MIN', 'MAX'),
        help='min_frontier_size 시뮬레이션 범위 (기본: 1 15)')
    parser.add_argument(
        '--min-size', type=int, default=3,
        help='프론티어 목록 출력 시 기준 크기 (기본: 3)')
    parser.add_argument(
        '--viz', type=str, default=None,
        help='프론티어 시각화 PNG 저장 경로')
    parser.add_argument(
        '--top-n', type=int, default=20,
        help='클러스터 목록 최대 출력 수 (기본: 20)')
    return parser.parse_args()


def main():
    args = parse_args()
    path = args.map_path

    # 파일 로드
    if path.endswith('.yaml') or path.endswith('.yml'):
        img, meta = load_yaml_map(path)
        print(f'[LOAD] YAML: {path}')
        print(f'  이미지: {meta.get("yaml_path", path)}')
    else:
        img, meta = load_pgm(path)
        meta.setdefault('resolution', 0.05)
        print(f'[LOAD] PGM: {path}')

    print(f'  크기: {meta["width"]} x {meta["height"]}  '
          f'maxval={meta["maxval"]}\n')

    # OccupancyGrid 변환
    occ = pgm_to_occupancy(img)

    # 맵 통계
    print_map_stats(occ, meta)

    # 프론티어 감지
    labeled, n_total = detect_frontiers(occ)
    print(f'감지된 프론티어 클러스터: {n_total}개')
    print()

    if n_total == 0:
        print('[INFO] 프론티어 없음 — 맵이 완전히 탐색됐거나 all-unknown 상태.')
        sys.exit(0)

    # 클러스터 분석
    clusters = analyze_frontiers(labeled, n_total, occ,
                                 resolution=meta.get('resolution', 0.05))

    # 프론티어 목록 출력
    print_frontier_list(clusters, min_size=args.min_size, top_n=args.top_n)

    # min_frontier_size 시뮬레이션
    size_range = (args.size_range[0], args.size_range[1])
    sim = simulate_min_size_filter(clusters, size_range)
    print_size_simulation(sim)

    # 시각화 저장
    if args.viz:
        save_viz(img, labeled, clusters, args.viz, min_size=args.min_size)

    # 요약 권고
    print('=' * 60)
    print('  권고')
    print('=' * 60)
    valid_at_3 = sum(1 for c in clusters if c['size_cells'] >= 3)
    valid_at_20 = sum(1 for c in clusters if c['size_cells'] >= 20)
    print(f'  min_size=3  → 유효 프론티어 {valid_at_3}개')
    print(f'  min_size=20 → 유효 프론티어 {valid_at_20}개')
    print()
    if valid_at_3 > 0 and valid_at_20 == 0:
        print('  → navigation.launch.py의 min_frontier_size: 3 유지 권장')
        print('    (frontier_explorer_node.py 기본값 20은 launch에서 3으로 오버라이드됨)')
    elif valid_at_20 > 5:
        print('  → min_frontier_size: 20도 충분한 프론티어 수 확보 가능')
    print()


if __name__ == '__main__':
    main()
