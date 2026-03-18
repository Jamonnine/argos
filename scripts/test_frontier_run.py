#!/usr/bin/env python3
"""
ARGOS Frontier 탐색 검증 스크립트
===================================
navigation.launch.py를 별도 터미널에서 먼저 실행한 뒤, 이 스크립트로
탐색 진행 상황을 모니터링한다.

사용법:
  # 기본 실행 (120초 타임아웃)
  python3 scripts/test_frontier_run.py

  # 타임아웃 변경
  python3 scripts/test_frontier_run.py --timeout 180

  # 완료 후 맵 PGM으로 저장
  python3 scripts/test_frontier_run.py --save-map /tmp/argos_final_map.pgm

  # 탐색 성공 기준 변경 (기본 70%)
  python3 scripts/test_frontier_run.py --target-explored 80

토픽 요구사항:
  /map                        (nav_msgs/OccupancyGrid, TRANSIENT_LOCAL)
  /frontier_explorer/exploration/frontier_count  (std_msgs/UInt32)
  /frontier_explorer/exploration/status          (std_msgs/String)
  /frontier_explorer/exploration/nav_error_count (std_msgs/UInt32)
"""

import argparse
import sys
import time
import threading
from pathlib import Path

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
)
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String, UInt32


# ── 탐색률 계산 ──────────────────────────────────────────────────────────────

def calc_explored_pct(data: np.ndarray) -> float:
    """OccupancyGrid 데이터에서 탐색률(%) 계산.

    탐색률 = (free + occupied) / total_cells * 100
    unknown = -1인 셀은 미탐색으로 처리.
    """
    total = data.size
    if total == 0:
        return 0.0
    unknown = np.sum(data == -1)
    explored = total - unknown
    return explored / total * 100.0


def count_frontiers_from_map(data: np.ndarray) -> int:
    """OccupancyGrid에서 프론티어 셀 수를 간이 추정.

    free-unknown 경계 셀의 수를 반환한다 (min_frontier_size 필터 미적용).
    정확한 클러스터 수는 exploration/frontier_count 토픽에서 수신한 값을 우선 사용.
    """
    try:
        import cv2
        h, w = data.shape
        free = (data == 0).astype(np.uint8)
        unknown = (data == -1).astype(np.uint8)
        kernel = np.ones((3, 3), dtype=np.uint8)
        unknown_dilated = cv2.dilate(unknown, kernel, iterations=1)
        frontier_mask = (free & unknown_dilated.astype(bool)).astype(np.uint8) * 255
        n_labels, _ = cv2.connectedComponents(frontier_mask)
        return max(0, n_labels - 1)
    except ImportError:
        # cv2 없을 경우 단순 경계 픽셀 수 반환
        free = (data == 0)
        unknown = (data == -1)
        frontier_count = 0
        for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            shifted = np.roll(unknown, (dy, dx), axis=(0, 1))
            frontier_count += np.sum(free & shifted)
        return int(frontier_count)


def save_map_pgm(map_msg: OccupancyGrid, path: str):
    """OccupancyGrid를 PGM 이미지로 저장.

    ROS 관례:
      free(0)    → 254 (밝음)
      occupied(>0) → 0  (어두움)
      unknown(-1) → 128 (회색)
    """
    w = map_msg.info.width
    h = map_msg.info.height
    data = np.array(map_msg.data, dtype=np.int8).reshape(h, w)

    img = np.full((h, w), 128, dtype=np.uint8)  # 기본: unknown(회색)
    img[data == 0] = 254                          # free: 밝음
    img[data > 0] = 0                             # occupied: 어두움

    # PGM 헤더 작성 (P5 = 바이너리 그레이스케일)
    pgm_path = Path(path)
    pgm_path.parent.mkdir(parents=True, exist_ok=True)
    with open(pgm_path, 'wb') as f:
        header = f'P5\n{w} {h}\n255\n'
        f.write(header.encode('ascii'))
        f.write(img.tobytes())

    print(f'[MAP SAVED] {pgm_path}  ({w}x{h}px)')


# ── ROS2 모니터 노드 ──────────────────────────────────────────────────────────

class FrontierMonitor(Node):
    """Frontier 탐색 진행 상황 모니터."""

    def __init__(self, target_explored_pct: float):
        super().__init__('frontier_monitor')

        self.target_explored_pct = target_explored_pct

        # 수신 데이터
        self.latest_map: OccupancyGrid | None = None
        self.frontier_count: int = 0        # exploration/frontier_count 토픽 값
        self.nav_goals_sent: int = 0        # status 변화 시 카운트
        self.nav_error_count: int = 0
        self.explore_status: str = 'unknown'

        # 통계
        self.explored_pct: float = 0.0
        self.success: bool = False
        self.start_time: float = time.monotonic()

        self._lock = threading.Lock()

        # QoS: /map은 TRANSIENT_LOCAL (slam_toolbox가 latch 발행)
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._map_cb, map_qos)

        # frontier_count, status, nav_error_count 구독
        # namespace: frontier_explorer 노드 이름에 따라 달라짐
        # navigation.launch.py 기준으로 namespace 없이 동작하므로 절대 경로 사용
        for topic, msg_type, cb in [
            ('/frontier_explorer/exploration/frontier_count', UInt32,
             self._frontier_count_cb),
            ('/frontier_explorer/exploration/status', String,
             self._status_cb),
            ('/frontier_explorer/exploration/nav_error_count', UInt32,
             self._nav_error_cb),
            # 네임스페이스 없이 단일 로봇 실행 시 fallback
            ('exploration/frontier_count', UInt32,
             self._frontier_count_cb),
            ('exploration/status', String,
             self._status_cb),
            ('exploration/nav_error_count', UInt32,
             self._nav_error_cb),
        ]:
            self.create_subscription(msg_type, topic, cb, 10)

    def _map_cb(self, msg: OccupancyGrid):
        with self._lock:
            self.latest_map = msg
            data = np.array(msg.data, dtype=np.int8)
            self.explored_pct = calc_explored_pct(data)
            if self.explored_pct >= self.target_explored_pct and not self.success:
                self.success = True

    def _frontier_count_cb(self, msg: UInt32):
        with self._lock:
            self.frontier_count = msg.data

    def _status_cb(self, msg: String):
        with self._lock:
            prev = self.explore_status
            self.explore_status = msg.data
            # 'navigating' 상태 전환 시 목표 전송 카운트 증가
            if msg.data == 'navigating' and prev != 'navigating':
                self.nav_goals_sent += 1

    def _nav_error_cb(self, msg: UInt32):
        with self._lock:
            self.nav_error_count = msg.data

    def snapshot(self) -> dict:
        with self._lock:
            return {
                'elapsed': time.monotonic() - self.start_time,
                'explored_pct': self.explored_pct,
                'frontier_count': self.frontier_count,
                'nav_goals_sent': self.nav_goals_sent,
                'nav_error_count': self.nav_error_count,
                'status': self.explore_status,
                'map_size': (
                    (self.latest_map.info.width, self.latest_map.info.height)
                    if self.latest_map else (0, 0)
                ),
                'success': self.success,
                'latest_map': self.latest_map,
            }


# ── 메인 ─────────────────────────────────────────────────────────────────────

def parse_args():
    parser = argparse.ArgumentParser(
        description='ARGOS Frontier 탐색 검증 스크립트')
    parser.add_argument(
        '--timeout', type=float, default=120.0,
        help='최대 대기 시간(초). 기본: 120')
    parser.add_argument(
        '--interval', type=float, default=10.0,
        help='출력 간격(초). 기본: 10')
    parser.add_argument(
        '--target-explored', type=float, default=70.0,
        dest='target_explored',
        help='탐색 성공 기준(%). 기본: 70')
    parser.add_argument(
        '--save-map', type=str, default=None,
        dest='save_map',
        help='완료 후 OccupancyGrid를 PGM으로 저장할 경로')
    return parser.parse_args()


def print_header():
    print('=' * 60)
    print('  ARGOS Frontier 탐색 검증 모니터')
    print('  navigation.launch.py를 먼저 실행해주세요.')
    print('=' * 60)
    print(f'{"시간(s)":>8}  {"탐색률%":>8}  {"프론티어":>8}  '
          f'{"목표전송":>8}  {"실패":>6}  상태')
    print('-' * 60)


def print_snapshot(snap: dict):
    elapsed = snap['elapsed']
    pct = snap['explored_pct']
    fc = snap['frontier_count']
    goals = snap['nav_goals_sent']
    errors = snap['nav_error_count']
    status = snap['status']
    w, h = snap['map_size']
    print(f'{elapsed:>8.1f}  {pct:>8.1f}  {fc:>8d}  '
          f'{goals:>8d}  {errors:>6d}  {status}  [맵 {w}x{h}]')


def analyze_final_map(node: FrontierMonitor):
    """타임아웃 시 최종 맵 상세 분석 출력."""
    snap = node.snapshot()
    map_msg = snap['latest_map']

    print()
    print('=' * 60)
    print('  [타임아웃] 최종 맵 분석')
    print('=' * 60)

    if map_msg is None:
        print('  /map 토픽 미수신. Nav2/SLAM이 실행 중인지 확인.')
        return

    w, h = map_msg.info.width, map_msg.info.height
    res = map_msg.info.resolution
    data = np.array(map_msg.data, dtype=np.int8)

    total = data.size
    free = int(np.sum(data == 0))
    occupied = int(np.sum(data > 0))
    unknown = int(np.sum(data == -1))
    explored = free + occupied

    print(f'  맵 크기:      {w} x {h}  ({w*res:.1f} x {h*res:.1f} m)')
    print(f'  해상도:       {res} m/cell')
    print(f'  전체 셀:      {total:,}')
    print(f'  free:         {free:,}  ({free/total*100:.1f}%)')
    print(f'  occupied:     {occupied:,}  ({occupied/total*100:.1f}%)')
    print(f'  unknown:      {unknown:,}  ({unknown/total*100:.1f}%)')
    print(f'  탐색률:       {explored/total*100:.1f}%')
    print()

    # 간이 프론티어 분석
    reshaped = data.reshape(h, w)
    fc = count_frontiers_from_map(reshaped)
    print(f'  감지된 프론티어 클러스터(간이): {fc}개')
    print()

    # 진단 힌트
    if snap['nav_goals_sent'] == 0:
        print('  진단: Nav2 목표가 하나도 전송되지 않았습니다.')
        print('    → frontier_explorer가 activate 상태인지 확인')
        print('    → ros2 lifecycle set /frontier_explorer activate')
    elif snap['nav_error_count'] > snap['nav_goals_sent'] * 0.5:
        print('  진단: Nav2 실패율 50% 초과.')
        print('    → collision_monitor PolygonStop 크기 확인')
        print('    → inflation_radius / costmap 해상도 조정 검토')
    elif fc > 5 and snap['explored_pct'] < 50.0:
        print('  진단: 프론티어가 많은데 탐색률이 낮습니다.')
        print('    → min_frontier_size를 낮춰보세요 (현재 기본 3)')
        print('    → exclusion_radius 과대 설정 여부 확인')


def main():
    args = parse_args()

    rclpy.init()
    node = FrontierMonitor(target_explored_pct=args.target_explored)

    # 스핀 스레드
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print_header()
    print(f'타임아웃: {args.timeout:.0f}초 | '
          f'출력 간격: {args.interval:.0f}초 | '
          f'성공 기준: {args.target_explored:.0f}%')
    print()

    deadline = time.monotonic() + args.timeout
    next_print = time.monotonic() + args.interval
    result = 'timeout'

    try:
        while time.monotonic() < deadline:
            time.sleep(0.5)

            snap = node.snapshot()

            # 탐색 성공
            if snap['success']:
                print_snapshot(snap)
                print()
                print(f'[SUCCESS] 탐색률 {snap["explored_pct"]:.1f}% 달성! '
                      f'(목표 {args.target_explored:.0f}%)')
                result = 'success'
                break

            # 탐색 완료 선언 (frontier 없음)
            if snap['status'] == 'complete':
                print_snapshot(snap)
                print()
                print(f'[COMPLETE] FrontierExplorer가 탐색 완료를 선언했습니다.')
                print(f'  최종 탐색률: {snap["explored_pct"]:.1f}%')
                result = 'complete'
                break

            # 주기 출력
            if time.monotonic() >= next_print:
                print_snapshot(snap)
                next_print = time.monotonic() + args.interval

    except KeyboardInterrupt:
        print('\n[INTERRUPTED] 사용자가 중단했습니다.')
        result = 'interrupted'

    # 최종 스냅샷 출력
    snap = node.snapshot()
    if result == 'timeout':
        print_snapshot(snap)
        analyze_final_map(node)

    # 맵 저장
    if args.save_map and snap['latest_map'] is not None:
        save_map_pgm(snap['latest_map'], args.save_map)
    elif args.save_map:
        print('[WARN] 저장할 맵 데이터가 없습니다.')

    print()
    print(f'결과: {result.upper()}')
    print(f'총 경과: {snap["elapsed"]:.1f}초 | '
          f'탐색률: {snap["explored_pct"]:.1f}% | '
          f'목표전송: {snap["nav_goals_sent"]}회 | '
          f'실패: {snap["nav_error_count"]}회')

    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()

    sys.exit(0 if result in ('success', 'complete') else 1)


if __name__ == '__main__':
    main()
