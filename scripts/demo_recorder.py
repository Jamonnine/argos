#!/usr/bin/env python3
"""
ARGOS 데모 녹화 보조 스크립트
================================
exploration.launch.py 또는 demo.launch.py 실행 중 자동 로깅.

사용법:
  python3 scripts/demo_recorder.py
  python3 scripts/demo_recorder.py --output /tmp/argos_demo.csv

CSV 컬럼:
  timestamp, robot_id, x, y, battery, state, coverage, stage

종료: Ctrl+C → 요약 통계 출력
"""

import argparse
import csv
import math
import os
import sys
import time
from collections import defaultdict
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)
from nav_msgs.msg import Odometry
from argos_interfaces.msg import RobotStatus, MissionState

# 기록 대상 로봇 ID
ROBOT_IDS = ('argos1', 'argos2', 'drone1')
# UGV만 odom 추적 (드론은 robot_status pose 사용)
UGV_IDS = ('argos1', 'argos2')

COLLISION_DISTANCE_M = 0.8

STATE_NAMES = {
    0: 'IDLE',
    1: 'EXPLORING',
    2: 'ON_MISSION',
    3: 'RETURNING',
    4: 'ERROR',
    5: 'COMM_LOST',
}

STAGE_NAMES = {
    0: 'INIT',
    1: 'EXPLORING',
    2: 'FIRE_RESPONSE',
    3: 'RETURNING',
    4: 'COMPLETE',
    5: 'PAUSED',
}

CSV_FIELDS = [
    'timestamp',
    'robot_id',
    'x',
    'y',
    'battery',
    'state',
    'coverage',
    'stage',
]


class DemoRecorder(Node):
    """데모 데이터 1초 간격 CSV 기록 노드."""

    def __init__(self, output_path: str):
        super().__init__('demo_recorder')

        self._output_path = output_path
        self._start_wall = time.monotonic()
        self._start_dt = datetime.now()

        # 최신 상태 버퍼
        self._robot_status: dict[str, RobotStatus] = {}
        self._odom: dict[str, tuple[float, float]] = {}  # robot_id → (x, y)
        self._mission_state: MissionState | None = None

        # 통계용 이동 거리 누적 (직전 위치 기준)
        self._prev_odom: dict[str, tuple[float, float]] = {}
        self._distance_sum: dict[str, float] = defaultdict(float)

        # 충돌 감지
        self._collision_count = 0

        # --- QoS ---
        be_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # --- 구독 ---
        for rid in ROBOT_IDS:
            self.create_subscription(
                RobotStatus,
                f'/{rid}/robot_status',
                lambda msg, r=rid: self._on_status(r, msg),
                be_qos,
            )

        for rid in UGV_IDS:
            self.create_subscription(
                Odometry,
                f'/{rid}/odom',
                lambda msg, r=rid: self._on_odom(r, msg),
                be_qos,
            )

        self.create_subscription(
            MissionState,
            '/mission_state',
            self._on_mission_state,
            be_qos,
        )

        # --- CSV 파일 초기화 ---
        self._csv_file = open(output_path, 'w', newline='', encoding='utf-8')
        self._csv_writer = csv.DictWriter(
            self._csv_file, fieldnames=CSV_FIELDS
        )
        self._csv_writer.writeheader()
        self._csv_file.flush()

        # --- 1초 기록 타이머 ---
        self.create_timer(1.0, self._record_tick)

        self.get_logger().info(f'[녹화] 시작 → {output_path}')
        self.get_logger().info(
            '  1초 간격 기록 | Ctrl+C로 종료 시 요약 출력'
        )

    # ------------------------------------------------------------------
    # 콜백
    # ------------------------------------------------------------------

    def _on_status(self, robot_id: str, msg: RobotStatus) -> None:
        self._robot_status[robot_id] = msg

    def _on_odom(self, robot_id: str, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        cur = (x, y)
        self._odom[robot_id] = cur

        # 이동 거리 누적
        if robot_id in self._prev_odom:
            prev = self._prev_odom[robot_id]
            self._distance_sum[robot_id] += math.sqrt(
                (cur[0] - prev[0]) ** 2 + (cur[1] - prev[1]) ** 2
            )
        self._prev_odom[robot_id] = cur

        # 충돌 감지
        self._check_collision(robot_id)

    def _on_mission_state(self, msg: MissionState) -> None:
        self._mission_state = msg

    # ------------------------------------------------------------------
    # 충돌 감지
    # ------------------------------------------------------------------

    def _check_collision(self, updated_id: str) -> None:
        """updated_id와 다른 로봇들 간 거리 확인."""
        if updated_id not in self._odom:
            return
        for other_id, other_pos in self._odom.items():
            if other_id == updated_id:
                continue
            dist = math.sqrt(
                (self._odom[updated_id][0] - other_pos[0]) ** 2
                + (self._odom[updated_id][1] - other_pos[1]) ** 2
            )
            if dist < COLLISION_DISTANCE_M:
                self._collision_count += 1
                elapsed = time.monotonic() - self._start_wall
                print(
                    f'[{elapsed:>6.1f}s] 충돌 경고: {updated_id} ↔ {other_id} '
                    f'거리 {dist:.2f}m | 누적 {self._collision_count}회'
                )

    # ------------------------------------------------------------------
    # 1초 기록
    # ------------------------------------------------------------------

    def _record_tick(self) -> None:
        """1초마다 모든 로봇 상태를 CSV에 기록."""
        now_ts = datetime.now().isoformat(timespec='milliseconds')
        stage = (
            STAGE_NAMES.get(self._mission_state.stage, str(self._mission_state.stage))
            if self._mission_state is not None
            else 'N/A'
        )

        for rid in ROBOT_IDS:
            st = self._robot_status.get(rid)
            odom_pos = self._odom.get(rid)

            # 위치: odom 우선, 없으면 robot_status pose
            if odom_pos is not None:
                x, y = odom_pos
            elif st is not None:
                x = st.pose.pose.position.x
                y = st.pose.pose.position.y
            else:
                x, y = float('nan'), float('nan')

            battery = st.battery_percent if st else float('nan')
            state_str = (
                STATE_NAMES.get(st.state, str(st.state)) if st else 'N/A'
            )
            coverage = st.coverage_percent if st else float('nan')

            row = {
                'timestamp': now_ts,
                'robot_id': rid,
                'x': f'{x:.3f}',
                'y': f'{y:.3f}',
                'battery': f'{battery:.1f}',
                'state': state_str,
                'coverage': f'{coverage:.2f}',
                'stage': stage,
            }
            self._csv_writer.writerow(row)

        self._csv_file.flush()

    # ------------------------------------------------------------------
    # 종료 + 요약
    # ------------------------------------------------------------------

    def print_summary(self) -> None:
        """Ctrl+C 시 요약 통계 출력."""
        elapsed = time.monotonic() - self._start_wall

        print(f'\n{"="*60}')
        print(f'[데모 녹화 요약]  경과: {elapsed:.1f}s')
        print(f'  CSV 저장 경로: {self._output_path}')
        print(f'{"="*60}')

        # 총 탐색 시간
        m, s = divmod(int(elapsed), 60)
        print(f'\n  총 탐색 시간: {m}분 {s}초')

        # 각 로봇 이동 거리
        print(f'\n  [로봇별 이동 거리]')
        for rid in ROBOT_IDS:
            dist = self._distance_sum.get(rid, 0.0)
            print(f'    {rid}: {dist:.2f}m')

        # 최종 coverage
        print(f'\n  [최종 coverage]')
        for rid in ROBOT_IDS:
            st = self._robot_status.get(rid)
            cov = st.coverage_percent if st else float('nan')
            print(f'    {rid}: {cov:.1f}%')

        # 오케스트레이터 최종 상태
        if self._mission_state is not None:
            ms = self._mission_state
            print(
                f'\n  오케스트레이터 총 coverage: {ms.overall_coverage_percent:.1f}%'
            )

        # 충돌 횟수
        print(f'\n  충돌 경고 횟수: {self._collision_count}회')
        print(f'{"="*60}\n')

    def shutdown(self) -> None:
        """파일 닫기."""
        if self._csv_file and not self._csv_file.closed:
            self._csv_file.close()


# ======================================================================
# 진입점
# ======================================================================

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='ARGOS 데모 녹화 스크립트'
    )
    default_path = os.path.join(
        '/tmp',
        f'argos_demo_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv',
    )
    parser.add_argument(
        '--output', '-o',
        default=default_path,
        help=f'CSV 저장 경로 (기본값: {default_path})',
    )
    # ros2 run 시 ROS 인자 무시
    return parser.parse_args(
        [a for a in sys.argv[1:] if not a.startswith('__')]
    )


def main() -> None:
    args = parse_args()
    rclpy.init()
    recorder = DemoRecorder(output_path=args.output)
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.print_summary()
        recorder.shutdown()
        recorder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
