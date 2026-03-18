#!/usr/bin/env python3
"""
ARGOS 데모 녹화 보조 스크립트
================================
exploration.launch.py 또는 demo.launch.py 실행 중 자동 로깅.

사용법:
  python3 scripts/demo_recorder.py
  python3 scripts/demo_recorder.py --output-dir /tmp/argos_logs
  python3 scripts/demo_recorder.py --duration 120

CSV 컬럼:
  timestamp, robot_id, x, y, z, stage, battery, coverage_pct

이벤트 CSV 컬럼 (화재 알림):
  timestamp, event_type, robot_id, fire_x, fire_y, fire_z,
  severity, confidence, temperature_k

종료: Ctrl+C 또는 --duration 만료 → 요약 통계 출력
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
from nav_msgs.msg import Odometry, OccupancyGrid
from argos_interfaces.msg import FireAlert, MissionState, RobotStatus

# 기록 대상 로봇 ID (odom 구독 모두 포함)
ROBOT_IDS = ('argos1', 'argos2', 'drone1')

# 근접 거리 임계값 (충돌 경고)
COLLISION_DISTANCE_M = 0.8

# 10초 간격 요약 출력
SUMMARY_INTERVAL_SEC = 10

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

# 상태 CSV 컬럼
CSV_FIELDS = [
    'timestamp',
    'robot_id',
    'x',
    'y',
    'z',
    'stage',
    'battery',
    'coverage_pct',
]

# 화재 이벤트 CSV 컬럼
FIRE_EVENT_FIELDS = [
    'timestamp',
    'event_type',
    'robot_id',
    'fire_x',
    'fire_y',
    'fire_z',
    'severity',
    'confidence',
    'temperature_k',
]


class DemoRecorder(Node):
    """데모 데이터 1초 간격 CSV 기록 + 화재 이벤트 로깅 노드."""

    def __init__(self, output_dir: str, duration_sec: float):
        super().__init__('demo_recorder')

        self._output_dir = output_dir
        self._duration_sec = duration_sec
        self._start_wall = time.monotonic()
        self._start_dt = datetime.now()

        # 최신 상태 버퍼
        self._robot_status: dict[str, RobotStatus] = {}
        self._odom: dict[str, tuple[float, float, float]] = {}  # robot_id → (x, y, z)
        self._mission_state: MissionState | None = None

        # 이동 거리 누적 (직전 위치 기준)
        self._prev_odom: dict[str, tuple[float, float, float]] = {}
        self._distance_sum: dict[str, float] = defaultdict(float)

        # 충돌 감지
        self._collision_count = 0
        # 충돌 중복 방지: 마지막 충돌 감지 시각 (pair → wall_time)
        self._last_collision_time: dict[frozenset, float] = {}
        self._collision_cooldown_sec = 3.0

        # 화재 감지 횟수
        self._fire_alert_count = 0

        # 10초 요약 타이머용 직전 출력 시각
        self._last_summary_wall = self._start_wall

        # 출력 파일 경로 생성
        ts_str = self._start_dt.strftime('%Y%m%d_%H%M%S')
        os.makedirs(output_dir, exist_ok=True)
        self._csv_path = os.path.join(output_dir, f'demo_{ts_str}.csv')
        self._fire_csv_path = os.path.join(output_dir, f'demo_{ts_str}_fire_events.csv')

        # --- QoS ---
        # 일반 센서 토픽: BEST_EFFORT
        be_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # /map: TRANSIENT_LOCAL (지연 구독자도 수신)
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # --- 로봇 상태 + odom 구독 ---
        for rid in ROBOT_IDS:
            self.create_subscription(
                RobotStatus,
                f'/{rid}/robot_status',
                lambda msg, r=rid: self._on_status(r, msg),
                be_qos,
            )
            self.create_subscription(
                Odometry,
                f'/{rid}/odom',
                lambda msg, r=rid: self._on_odom(r, msg),
                be_qos,
            )

        # --- 맵 구독 (커버리지 보조) ---
        self.create_subscription(
            OccupancyGrid,
            '/map',
            self._on_map,
            map_qos,
        )

        # --- 오케스트레이터 토픽 ---
        self.create_subscription(
            MissionState,
            '/orchestrator/mission_state',
            self._on_mission_state,
            be_qos,
        )
        self.create_subscription(
            FireAlert,
            '/orchestrator/fire_alert',
            self._on_fire_alert,
            be_qos,
        )

        # --- CSV 파일 초기화 (상태 로그) ---
        self._csv_file = open(self._csv_path, 'w', newline='', encoding='utf-8')
        self._csv_writer = csv.DictWriter(self._csv_file, fieldnames=CSV_FIELDS)
        self._csv_writer.writeheader()
        self._csv_file.flush()

        # --- CSV 파일 초기화 (화재 이벤트 로그) ---
        self._fire_csv_file = open(self._fire_csv_path, 'w', newline='', encoding='utf-8')
        self._fire_csv_writer = csv.DictWriter(
            self._fire_csv_file, fieldnames=FIRE_EVENT_FIELDS
        )
        self._fire_csv_writer.writeheader()
        self._fire_csv_file.flush()

        # --- 타이머 ---
        self.create_timer(1.0, self._record_tick)
        self.create_timer(SUMMARY_INTERVAL_SEC, self._periodic_summary)
        if duration_sec > 0:
            self.create_timer(duration_sec, self._on_duration_expired)

        self.get_logger().info(f'[녹화] 시작 → {self._csv_path}')
        self.get_logger().info(f'[이벤트] 화재 알림 로그 → {self._fire_csv_path}')
        if duration_sec > 0:
            self.get_logger().info(f'[녹화] 자동 종료 설정: {duration_sec:.0f}초 후')
        self.get_logger().info('  1초 간격 기록 | Ctrl+C 또는 --duration 만료 시 요약 출력')

    # ------------------------------------------------------------------
    # 콜백
    # ------------------------------------------------------------------

    def _on_status(self, robot_id: str, msg: RobotStatus) -> None:
        self._robot_status[robot_id] = msg

    def _on_odom(self, robot_id: str, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        cur = (x, y, z)
        self._odom[robot_id] = cur

        # 이동 거리 누적 (3D 유클리드)
        if robot_id in self._prev_odom:
            prev = self._prev_odom[robot_id]
            self._distance_sum[robot_id] += math.sqrt(
                (cur[0] - prev[0]) ** 2
                + (cur[1] - prev[1]) ** 2
                + (cur[2] - prev[2]) ** 2
            )
        self._prev_odom[robot_id] = cur

        # UGV 간 충돌 감지 (드론 제외: 고도차로 오탐 가능)
        if robot_id != 'drone1':
            self._check_collision(robot_id)

    def _on_map(self, msg: OccupancyGrid) -> None:
        # 현재는 수신 확인만 (커버리지는 RobotStatus 기반 사용)
        pass

    def _on_mission_state(self, msg: MissionState) -> None:
        self._mission_state = msg

    def _on_fire_alert(self, msg: FireAlert) -> None:
        """화재 알림 수신 → 이벤트 CSV 기록 + 카운터 증가."""
        self._fire_alert_count += 1
        now_ts = datetime.now().isoformat(timespec='milliseconds')
        elapsed = time.monotonic() - self._start_wall

        fx = msg.location.point.x
        fy = msg.location.point.y
        fz = msg.location.point.z

        row = {
            'timestamp': now_ts,
            'event_type': 'FIRE_ALERT',
            'robot_id': msg.robot_id,
            'fire_x': f'{fx:.3f}',
            'fire_y': f'{fy:.3f}',
            'fire_z': f'{fz:.3f}',
            'severity': msg.severity,
            'confidence': f'{msg.confidence:.3f}',
            'temperature_k': f'{msg.max_temperature_kelvin:.1f}',
        }
        self._fire_csv_writer.writerow(row)
        self._fire_csv_file.flush()

        print(
            f'[{elapsed:>6.1f}s] 화재 알림 #{self._fire_alert_count}: '
            f'{msg.robot_id} 감지 | 위치 ({fx:.1f}, {fy:.1f}) | '
            f'심각도={msg.severity} | 신뢰도={msg.confidence:.0%} | '
            f'온도={msg.max_temperature_kelvin:.0f}K'
        )

    def _on_duration_expired(self) -> None:
        """--duration 만료 시 호출."""
        elapsed = time.monotonic() - self._start_wall
        self.get_logger().info(f'[녹화] 지정 시간 {elapsed:.0f}초 완료, 종료 중...')
        raise KeyboardInterrupt

    # ------------------------------------------------------------------
    # 충돌 감지 (UGV 전용)
    # ------------------------------------------------------------------

    def _check_collision(self, updated_id: str) -> None:
        """updated_id와 다른 UGV 간 거리 확인 (쿨다운 포함)."""
        if updated_id not in self._odom:
            return
        now_wall = time.monotonic()
        pos_a = self._odom[updated_id]

        for other_id, other_pos in self._odom.items():
            if other_id == updated_id or other_id == 'drone1':
                continue
            dist = math.sqrt(
                (pos_a[0] - other_pos[0]) ** 2
                + (pos_a[1] - other_pos[1]) ** 2
            )
            if dist < COLLISION_DISTANCE_M:
                pair = frozenset([updated_id, other_id])
                last_t = self._last_collision_time.get(pair, 0.0)
                if now_wall - last_t >= self._collision_cooldown_sec:
                    self._collision_count += 1
                    self._last_collision_time[pair] = now_wall
                    elapsed = now_wall - self._start_wall
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
            STAGE_NAMES.get(
                self._mission_state.stage, str(self._mission_state.stage)
            )
            if self._mission_state is not None
            else 'N/A'
        )

        for rid in ROBOT_IDS:
            st = self._robot_status.get(rid)
            odom_pos = self._odom.get(rid)

            # 위치: odom 우선, 없으면 robot_status pose
            if odom_pos is not None:
                x, y, z = odom_pos
            elif st is not None:
                x = st.pose.pose.position.x
                y = st.pose.pose.position.y
                z = st.pose.pose.position.z
            else:
                x, y, z = float('nan'), float('nan'), float('nan')

            battery = st.battery_percent if st else float('nan')
            coverage = st.coverage_percent if st else float('nan')

            row = {
                'timestamp': now_ts,
                'robot_id': rid,
                'x': f'{x:.3f}',
                'y': f'{y:.3f}',
                'z': f'{z:.3f}',
                'stage': stage,
                'battery': f'{battery:.1f}',
                'coverage_pct': f'{coverage:.2f}',
            }
            self._csv_writer.writerow(row)

        self._csv_file.flush()

    # ------------------------------------------------------------------
    # 10초 간격 콘솔 요약
    # ------------------------------------------------------------------

    def _periodic_summary(self) -> None:
        """10초마다 현재 상태 요약을 콘솔에 출력."""
        elapsed = time.monotonic() - self._start_wall
        m, s = divmod(int(elapsed), 60)

        # 오케스트레이터 커버리지 (있으면 우선)
        if self._mission_state is not None:
            overall_cov = self._mission_state.overall_coverage_percent
            stage_str = STAGE_NAMES.get(
                self._mission_state.stage, str(self._mission_state.stage)
            )
            fire_count = self._mission_state.fire_count
        else:
            overall_cov = float('nan')
            stage_str = 'N/A'
            fire_count = self._fire_alert_count

        print(f'\n{"─"*56}')
        print(f'  [10초 요약] 경과 {m:02d}:{s:02d} | 단계: {stage_str}')
        print(f'  탐색 커버리지: {overall_cov:.1f}%  화재감지: {fire_count}건  충돌경고: {self._collision_count}회')
        print(f'  {'로봇':<10} {'위치 (x,y,z)':<30} {'이동거리':>8}  배터리')
        for rid in ROBOT_IDS:
            pos = self._odom.get(rid)
            dist = self._distance_sum.get(rid, 0.0)
            st = self._robot_status.get(rid)
            bat = f'{st.battery_percent:.0f}%' if st else 'N/A'
            if pos is not None:
                pos_str = f'({pos[0]:>6.1f},{pos[1]:>6.1f},{pos[2]:>5.1f})'
            else:
                pos_str = '(  N/A  )'
            print(f'  {rid:<10} {pos_str:<30} {dist:>7.1f}m  {bat}')
        print(f'{"─"*56}')

    # ------------------------------------------------------------------
    # 종료 + 요약
    # ------------------------------------------------------------------

    def print_summary(self) -> None:
        """종료 시 최종 요약 통계 출력."""
        elapsed = time.monotonic() - self._start_wall
        m, s = divmod(int(elapsed), 60)

        print(f'\n{"="*60}')
        print(f'[ARGOS 데모 녹화 최종 요약]')
        print(f'  시작: {self._start_dt.strftime("%Y-%m-%d %H:%M:%S")}')
        print(f'  종료: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}')
        print(f'  총 탐색 시간: {m}분 {s}초 ({elapsed:.1f}s)')
        print(f'{"="*60}')

        # 출력 파일 경로
        print(f'\n  [출력 파일]')
        print(f'    상태 로그   : {self._csv_path}')
        print(f'    화재 이벤트 : {self._fire_csv_path}')

        # 로봇별 이동 거리
        print(f'\n  [로봇별 이동 거리]')
        for rid in ROBOT_IDS:
            dist = self._distance_sum.get(rid, 0.0)
            print(f'    {rid:<10}: {dist:.2f}m')

        # 최종 커버리지
        print(f'\n  [최종 탐색 커버리지]')
        if self._mission_state is not None:
            print(
                f'    전체 (오케스트레이터): '
                f'{self._mission_state.overall_coverage_percent:.1f}%'
            )
        for rid in ROBOT_IDS:
            st = self._robot_status.get(rid)
            cov = st.coverage_percent if st else float('nan')
            print(f'    {rid:<10}: {cov:.1f}%')

        # 화재 감지
        print(f'\n  [화재 감지]')
        fa_count = self._fire_alert_count
        if self._mission_state is not None:
            fa_count = max(fa_count, self._mission_state.fire_count)
        print(f'    총 화재 알림 수신: {fa_count}건')

        # 충돌 횟수
        print(f'\n  [안전 지표]')
        print(f'    충돌 경고 횟수: {self._collision_count}회')

        print(f'{"="*60}\n')

    def shutdown(self) -> None:
        """CSV 파일 닫기."""
        for f in (self._csv_file, self._fire_csv_file):
            if f and not f.closed:
                f.close()


# ======================================================================
# 진입점
# ======================================================================

def _default_output_dir() -> str:
    """스크립트 위치 기준 projects/argos/data/ 반환."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(script_dir, '..', 'data')


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='ARGOS 데모 녹화 스크립트',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            '예시:\n'
            '  python3 scripts/demo_recorder.py\n'
            '  python3 scripts/demo_recorder.py --duration 120\n'
            '  python3 scripts/demo_recorder.py --output-dir /tmp/argos_logs\n'
        ),
    )
    parser.add_argument(
        '--output-dir', '-d',
        default=_default_output_dir(),
        metavar='DIR',
        help='CSV 저장 디렉토리 (기본값: projects/argos/data/)',
    )
    parser.add_argument(
        '--duration', '-t',
        type=float,
        default=300.0,
        metavar='SEC',
        help='녹화 시간(초), 0이면 무제한 (기본값: 300)',
    )
    # ros2 run 전달 인자 무시
    return parser.parse_args(
        [a for a in sys.argv[1:] if not a.startswith('__')]
    )


def main() -> None:
    args = parse_args()
    output_dir = os.path.realpath(args.output_dir)
    duration = max(0.0, args.duration)

    rclpy.init()
    recorder = DemoRecorder(output_dir=output_dir, duration_sec=duration)
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
