#!/usr/bin/env python3
"""
ARGOS 데모 콘셉트 애니메이션 GIF 생성기.

이종 군집(UGV 3대 + 드론 1대)이 건물을 탐색하고
화재를 감지하여 대응하는 시나리오를 시각화.
"""

import math
import numpy as np
from PIL import Image, ImageDraw, ImageFont

# --- 설정 ---
W, H = 800, 600
FRAMES = 120          # 총 프레임
FPS = 12              # 초당 프레임
BG = (15, 23, 42)     # 배경 (dark navy)
GRID_COLOR = (30, 41, 59)
WALL_COLOR = (71, 85, 105)
FREE_COLOR = (30, 58, 95)
UNKNOWN_COLOR = (15, 23, 42)
FRONTIER_COLOR = (56, 189, 248)  # cyan
FIRE_COLOR = (239, 68, 68)
TRAIL_ALPHA = 80

# 로봇 색상
UGV_COLORS = [
    (34, 197, 94),   # green (argos1)
    (59, 130, 246),  # blue (argos2)
    (168, 85, 247),  # purple (argos3)
]
DRONE_COLOR = (251, 191, 36)  # amber


def draw_building(draw, t_ratio):
    """실내 건물 평면도 (벽)."""
    walls = [
        # 외벽
        (100, 80, 700, 80), (100, 520, 700, 520),
        (100, 80, 100, 520), (700, 80, 700, 520),
        # 내벽 (방 분리)
        (300, 80, 300, 280), (300, 340, 300, 520),
        (500, 80, 500, 200), (500, 260, 500, 520),
        # 작은 내벽
        (100, 300, 220, 300),
        (580, 300, 700, 300),
    ]
    for x1, y1, x2, y2 in walls:
        draw.line([(x1, y1), (x2, y2)], fill=WALL_COLOR, width=3)


def get_robot_path(robot_idx, t_ratio):
    """각 로봇의 경로 생성 (사전 정의)."""
    # UGV1: 왼쪽 상단 방 탐색
    if robot_idx == 0:
        waypoints = [
            (130, 400), (180, 150), (250, 200), (200, 400),
            (150, 300), (250, 150), (200, 250),
        ]
    # UGV2: 중앙 방 탐색
    elif robot_idx == 1:
        waypoints = [
            (130, 450), (350, 150), (400, 300), (350, 450),
            (450, 200), (400, 400), (350, 300),
        ]
    # UGV3: 오른쪽 방 탐색
    elif robot_idx == 2:
        waypoints = [
            (130, 500), (550, 400), (650, 200), (600, 450),
            (550, 150), (650, 350), (600, 250),
        ]
    # Drone: 높은 고도 순찰 (넓은 패턴)
    else:
        waypoints = [
            (400, 300), (200, 150), (600, 150), (600, 450),
            (200, 450), (400, 300), (300, 200),
        ]

    # t_ratio → 현재 위치 보간
    total_segs = len(waypoints) - 1
    seg_idx = min(int(t_ratio * total_segs), total_segs - 1)
    seg_t = (t_ratio * total_segs) - seg_idx
    seg_t = min(1.0, max(0.0, seg_t))

    x1, y1 = waypoints[seg_idx]
    x2, y2 = waypoints[min(seg_idx + 1, len(waypoints) - 1)]
    x = x1 + (x2 - x1) * seg_t
    y = y1 + (y2 - y1) * seg_t
    return x, y


def draw_explored_area(draw, t_ratio):
    """탐색된 영역을 점진적으로 표시."""
    # 원형으로 탐색 영역 확대 (간단 근사)
    for robot_idx in range(3):
        for t_step in np.linspace(0, t_ratio, int(t_ratio * 40) + 1):
            rx, ry = get_robot_path(robot_idx, t_step)
            radius = 25
            draw.ellipse(
                [rx - radius, ry - radius, rx + radius, ry + radius],
                fill=FREE_COLOR)


def draw_frontiers(draw, t_ratio):
    """프론티어 (탐색 경계) 표시."""
    if t_ratio < 0.1:
        return
    # 프론티어를 몇 개 점으로 표시
    frontier_points = [
        (280, 200, 0.1, 0.6), (280, 400, 0.15, 0.5),
        (480, 180, 0.2, 0.7), (480, 400, 0.25, 0.65),
        (600, 300, 0.3, 0.8),
    ]
    for fx, fy, appear, disappear in frontier_points:
        if appear < t_ratio < disappear:
            alpha = int(128 * math.sin((t_ratio - appear) / (disappear - appear) * math.pi))
            for d in range(8, 0, -2):
                c = tuple(int(FRONTIER_COLOR[i] * (d / 8.0)) for i in range(3))
                draw.ellipse([fx - d, fy - d, fx + d, fy + d], fill=c)


def draw_fire(draw, t_ratio, frame_idx):
    """화재 발생 + 감지 시각화."""
    fire_start = 0.55  # 55%에서 화재 발생
    if t_ratio < fire_start:
        return None

    fire_x, fire_y = 580, 380
    # 불꽃 애니메이션
    pulse = math.sin(frame_idx * 0.5) * 5 + 15
    for r in range(int(pulse), 0, -3):
        alpha = int(200 * r / pulse)
        c = (min(255, 239 + (15 - r) * 3), max(0, 68 - r * 2), 0)
        draw.ellipse([fire_x - r, fire_y - r, fire_x + r, fire_y + r],
                     fill=c)

    # 감지 후 alert 텍스트
    if t_ratio > fire_start + 0.05:
        draw.text((fire_x + 20, fire_y - 20), "FIRE!", fill=FIRE_COLOR)

    return (fire_x, fire_y)


def draw_robot(draw, x, y, color, robot_type='ugv', size=8):
    """로봇 아이콘 그리기."""
    if robot_type == 'ugv':
        # 사각형 (UGV)
        draw.rectangle([x - size, y - size, x + size, y + size],
                       fill=color, outline=(255, 255, 255), width=1)
    else:
        # 다이아몬드 (드론)
        points = [(x, y - size), (x + size, y), (x, y + size), (x - size, y)]
        draw.polygon(points, fill=color, outline=(255, 255, 255))


def draw_trails(draw, robot_idx, t_ratio, color):
    """로봇 이동 궤적."""
    points = []
    for t in np.linspace(0, t_ratio, max(2, int(t_ratio * 60))):
        x, y = get_robot_path(robot_idx, t)
        points.append((x, y))

    if len(points) < 2:
        return

    trail_color = tuple(int(c * 0.4) for c in color)
    for i in range(1, len(points)):
        draw.line([points[i - 1], points[i]], fill=trail_color, width=1)


def draw_hud(draw, t_ratio, fire_pos):
    """HUD 오버레이 (상태 정보)."""
    # 타이틀
    draw.text((20, 10), "ARGOS - Multi-Robot Fire Response",
              fill=(226, 232, 240))

    # 미션 단계
    if t_ratio < 0.05:
        stage = "INIT"
        stage_color = (148, 163, 184)
    elif t_ratio < 0.55:
        stage = "EXPLORING"
        stage_color = (34, 197, 94)
    elif t_ratio < 0.75:
        stage = "FIRE RESPONSE"
        stage_color = FIRE_COLOR
    elif t_ratio < 0.90:
        stage = "RETURNING"
        stage_color = (59, 130, 246)
    else:
        stage = "COMPLETE"
        stage_color = (168, 85, 247)

    draw.text((20, 35), f"Stage: {stage}", fill=stage_color)

    # 로봇 상태
    y_off = 555
    labels = [
        ("argos1", UGV_COLORS[0]),
        ("argos2", UGV_COLORS[1]),
        ("argos3", UGV_COLORS[2]),
        ("scout_drone", DRONE_COLOR),
    ]
    for i, (name, color) in enumerate(labels):
        x_off = 100 + i * 170
        draw.rectangle([x_off, y_off, x_off + 10, y_off + 10], fill=color)
        draw.text((x_off + 15, y_off - 2), name, fill=(203, 213, 225))

    # 커버리지
    coverage = min(100, int(t_ratio * 120))
    draw.text((650, 35), f"Coverage: {coverage}%", fill=(148, 163, 184))

    # 화재 감지 시 빨간 테두리
    if fire_pos and t_ratio > 0.55:
        for offset in range(2):
            draw.rectangle(
                [offset, offset, W - 1 - offset, H - 1 - offset],
                outline=FIRE_COLOR, width=1)


def generate_frame(frame_idx):
    """단일 프레임 생성."""
    img = Image.new('RGB', (W, H), BG)
    draw = ImageDraw.Draw(img)

    t_ratio = frame_idx / FRAMES

    # 그리드
    for x in range(100, 701, 50):
        draw.line([(x, 80), (x, 520)], fill=GRID_COLOR, width=1)
    for y in range(80, 521, 50):
        draw.line([(100, y), (700, y)], fill=GRID_COLOR, width=1)

    # 탐색 영역
    draw_explored_area(draw, t_ratio)

    # 건물
    draw_building(draw, t_ratio)

    # 프론티어
    draw_frontiers(draw, t_ratio)

    # 화재
    fire_pos = draw_fire(draw, t_ratio, frame_idx)

    # 궤적 + 로봇
    for i in range(3):
        effective_t = t_ratio
        if t_ratio > 0.90:
            # 귀환: 시작점으로 이동
            return_progress = (t_ratio - 0.90) / 0.10
            rx, ry = get_robot_path(i, 0.90)
            start_x, start_y = get_robot_path(i, 0.0)
            rx = rx + (start_x - rx) * return_progress
            ry = ry + (start_y - ry) * return_progress
        else:
            rx, ry = get_robot_path(i, effective_t)
        draw_trails(draw, i, min(t_ratio, 0.90), UGV_COLORS[i])
        draw_robot(draw, rx, ry, UGV_COLORS[i], 'ugv')

    # 드론
    if t_ratio > 0.08:
        drone_t = min(1.0, (t_ratio - 0.08) / 0.85)
        if t_ratio > 0.90:
            dx, dy = get_robot_path(3, 1.0)
            return_progress = (t_ratio - 0.90) / 0.10
            start_x, start_y = get_robot_path(3, 0.0)
            dx = dx + (start_x - dx) * return_progress
            dy = dy + (start_y - dy) * return_progress
        else:
            dx, dy = get_robot_path(3, drone_t)
        draw_trails(draw, 3, min(drone_t, 0.90), DRONE_COLOR)
        draw_robot(draw, dx, dy, DRONE_COLOR, 'drone', size=10)

    # HUD
    draw_hud(draw, t_ratio, fire_pos)

    return img


def main():
    import os
    output_dir = os.path.dirname(os.path.abspath(__file__))
    output_path = os.path.join(
        output_dir, '..', 'docs', 'demo.gif')
    output_path = os.path.abspath(output_path)

    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    print(f"Generating {FRAMES} frames...")
    frames = []
    for i in range(FRAMES):
        frames.append(generate_frame(i))
        if (i + 1) % 20 == 0:
            print(f"  {i + 1}/{FRAMES}")

    print(f"Saving GIF to {output_path}...")
    frames[0].save(
        output_path,
        save_all=True,
        append_images=frames[1:],
        duration=int(1000 / FPS),
        loop=0,
        optimize=True,
    )

    # 파일 크기 확인
    size_kb = os.path.getsize(output_path) / 1024
    print(f"Done! {size_kb:.0f} KB")


if __name__ == '__main__':
    main()
