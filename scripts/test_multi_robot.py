#!/usr/bin/env python3
"""
ARGOS 멀티로봇 협업탐색 검증 스크립트
=======================================
exploration.launch.py 실행 후 별도 터미널에서 실행.

사용법:
  ros2 run argos_bringup ...  # 또는
  python3 scripts/test_multi_robot.py

성공 기준:
  - 3대 모두 STATE_EXPLORING 상태
  - 로봇 간 최소 거리 >= 0.8m (충돌 없음)
  - coverage_percent 증가 추세
  - 180초 이내 달성
"""

import math
import sys
import time

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

# 검증 파라미터
TIMEOUT_SEC = 180
REPORT_INTERVAL_SEC = 10
COLLISION_DISTANCE_M = 0.8  # 이 거리 미만이면 충돌 경고

# 상태 코드 → 문자열 매핑
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


class MultiRobotVerifier(Node):
    """멀티로봇 검증 노드."""

    def __init__(self):
        super().__init__('multi_robot_verifier')

        # --- QoS: RobotStatus는 Deadline QoS(5s) 발행자와 호환 ---
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # UGV 상태 추적
        self._ugv_status: dict[str, RobotStatus] = {}
        # 드론 포함 전체 상태
        self._all_status: dict[str, RobotStatus] = {}
        # odom 위치 (충돌 감지용)
        self._odom_positions: dict[str, tuple[float, float]] = {}
        # 미션 상태
        self._mission_state: MissionState | None = None
        # 충돌 경고 누적 횟수
        self._collision_count = 0
        # 첫 coverage 기록 (증가 추세 확인)
        self._first_coverage: dict[str, float] = {}
        # 상태 변경 타임스탬프
        self._exploring_start: dict[str, float] = {}

        start_time = self.get_clock().now()
        self._start_wall = time.monotonic()

        # --- 구독 ---
        for robot_id in ('argos1', 'argos2'):
            self.create_subscription(
                RobotStatus,
                f'/{robot_id}/robot_status',
                lambda msg, rid=robot_id: self._on_robot_status(rid, msg),
                best_effort_qos,
            )
            self.create_subscription(
                Odometry,
                f'/{robot_id}/odom',
                lambda msg, rid=robot_id: self._on_odom(rid, msg),
                best_effort_qos,
            )

        self.create_subscription(
            MissionState,
            '/mission_state',
            self._on_mission_state,
            best_effort_qos,
        )

        # 10초 주기 보고 타이머
        self._report_timer = self.create_timer(
            REPORT_INTERVAL_SEC, self._report
        )
        # 타임아웃 타이머
        self._timeout_timer = self.create_timer(
            TIMEOUT_SEC, self._on_timeout
        )

        self.get_logger().info(
            f'[검증] 시작. 타임아웃 {TIMEOUT_SEC}s, 보고 간격 {REPORT_INTERVAL_SEC}s'
        )
        self.get_logger().info(
            '  구독 토픽: /argos1/robot_status, /argos2/robot_status, /mission_state'
        )
        self.get_logger().info(
            '  성공 기준: 2 UGV EXPLORING + 충돌 없음 + coverage 증가'
        )

    # ------------------------------------------------------------------
    # 콜백
    # ------------------------------------------------------------------

    def _on_robot_status(self, robot_id: str, msg: RobotStatus) -> None:
        """RobotStatus 콜백."""
        prev = self._all_status.get(robot_id)
        self._all_status[robot_id] = msg

        if robot_id.startswith('argos'):
            self._ugv_status[robot_id] = msg

        # 첫 coverage 기록
        if robot_id not in self._first_coverage:
            self._first_coverage[robot_id] = msg.coverage_percent

        # EXPLORING 상태 진입 시간 기록
        if prev is None or prev.state != 1:
            if msg.state == 1:  # STATE_EXPLORING
                self._exploring_start[robot_id] = time.monotonic()

    def _on_odom(self, robot_id: str, msg: Odometry) -> None:
        """odom 콜백 — 충돌 감지용 위치 갱신."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self._odom_positions[robot_id] = (x, y)
        self._check_collision()

    def _on_mission_state(self, msg: MissionState) -> None:
        """MissionState 콜백."""
        self._mission_state = msg

    # ------------------------------------------------------------------
    # 충돌 감지
    # ------------------------------------------------------------------

    def _check_collision(self) -> None:
        """등록된 모든 로봇 쌍 간 거리 확인."""
        robot_ids = list(self._odom_positions.keys())
        for i in range(len(robot_ids)):
            for j in range(i + 1, len(robot_ids)):
                rid_a = robot_ids[i]
                rid_b = robot_ids[j]
                dist = self._distance(
                    self._odom_positions[rid_a],
                    self._odom_positions[rid_b],
                )
                if dist < COLLISION_DISTANCE_M:
                    self._collision_count += 1
                    self.get_logger().warn(
                        f'[충돌 경고] {rid_a} ↔ {rid_b} 거리 {dist:.2f}m '
                        f'(기준 {COLLISION_DISTANCE_M}m) | 누적 {self._collision_count}회'
                    )

    @staticmethod
    def _distance(a: tuple[float, float], b: tuple[float, float]) -> float:
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def _min_robot_distance(self) -> float:
        """현재 로봇 간 최소 거리 반환. 위치 없으면 inf."""
        robot_ids = list(self._odom_positions.keys())
        min_dist = float('inf')
        for i in range(len(robot_ids)):
            for j in range(i + 1, len(robot_ids)):
                d = self._distance(
                    self._odom_positions[robot_ids[i]],
                    self._odom_positions[robot_ids[j]],
                )
                min_dist = min(min_dist, d)
        return min_dist

    # ------------------------------------------------------------------
    # 상태 보고
    # ------------------------------------------------------------------

    def _report(self) -> None:
        """10초 주기 상태 보고."""
        elapsed = time.monotonic() - self._start_wall
        print(f'\n{"="*60}')
        print(f'[{elapsed:>6.1f}s] ARGOS 멀티로봇 상태 보고')
        print(f'{"="*60}')

        # 각 UGV 상태
        for rid in ('argos1', 'argos2'):
            if rid in self._ugv_status:
                st = self._ugv_status[rid]
                pos = self._odom_positions.get(rid, ('?', '?'))
                x_str = f'{pos[0]:.2f}' if isinstance(pos[0], float) else '?'
                y_str = f'{pos[1]:.2f}' if isinstance(pos[1], float) else '?'
                state_name = STATE_NAMES.get(st.state, f'?({st.state})')
                print(
                    f'  {rid:8s} | ({x_str:6s}, {y_str:6s}) | '
                    f'배터리 {st.battery_percent:5.1f}% | '
                    f'상태 {state_name:12s} | '
                    f'coverage {st.coverage_percent:5.1f}%'
                )
            else:
                print(f'  {rid:8s} | 토픽 수신 대기 중...')

        # 오케스트레이터 상태
        if self._mission_state is not None:
            ms = self._mission_state
            stage_name = STAGE_NAMES.get(ms.stage, f'?({ms.stage})')
            print(
                f'\n  오케스트레이터 | 스테이지 {stage_name:14s} | '
                f'총 coverage {ms.overall_coverage_percent:.1f}% | '
                f'활성 로봇 {ms.active_robots}/{ms.total_robots}'
            )
        else:
            print('\n  오케스트레이터 | /mission_state 수신 대기 중...')

        # 로봇 간 거리
        min_dist = self._min_robot_distance()
        dist_str = f'{min_dist:.2f}m' if min_dist != float('inf') else '측정 불가'
        collision_flag = ' [경고]' if min_dist < COLLISION_DISTANCE_M else ''
        print(f'\n  최소 로봇 간 거리: {dist_str}{collision_flag}')
        print(f'  충돌 경고 누적: {self._collision_count}회')

        # 성공 기준 점검
        self._check_success_criteria()
        print(f'{"="*60}')

    def _check_success_criteria(self) -> bool:
        """성공 기준 점검 — 모두 충족 시 True 반환."""
        ugv_ids = ('argos1', 'argos2')

        # 기준 1: 2 UGV 모두 EXPLORING
        all_exploring = all(
            self._ugv_status.get(rid, None) is not None
            and self._ugv_status[rid].state == 1  # STATE_EXPLORING
            for rid in ugv_ids
        )

        # 기준 2: 충돌 없음
        no_collision = self._collision_count == 0

        # 기준 3: coverage 증가 (첫 기록 대비)
        coverage_increasing = all(
            (
                rid in self._ugv_status
                and rid in self._first_coverage
                and self._ugv_status[rid].coverage_percent
                >= self._first_coverage[rid]
            )
            for rid in ugv_ids
        )

        print(f'\n  [성공 기준]')
        print(f'    2 UGV EXPLORING: {"✓" if all_exploring else "✗"}')
        print(f'    충돌 없음:        {"✓" if no_collision else "✗"}')
        print(f'    coverage 증가:   {"✓" if coverage_increasing else "✗"}')

        if all_exploring and no_collision and coverage_increasing:
            elapsed = time.monotonic() - self._start_wall
            print(f'\n  [성공] 모든 기준 충족! 경과 시간: {elapsed:.1f}s')
            return True

        return False

    # ------------------------------------------------------------------
    # 타임아웃 + 종합 보고
    # ------------------------------------------------------------------

    def _on_timeout(self) -> None:
        """180초 타임아웃 — 종합 보고 후 종료."""
        self._timeout_timer.cancel()
        self._report_timer.cancel()
        elapsed = time.monotonic() - self._start_wall

        print(f'\n{"#"*60}')
        print(f'[종합 보고] {elapsed:.1f}s 경과 후 검증 완료')
        print(f'{"#"*60}')

        success = self._check_success_criteria()

        print(f'\n  총 충돌 경고: {self._collision_count}회')

        # 각 로봇 coverage 변화
        print(f'\n  [coverage 변화]')
        for rid in ('argos1', 'argos2'):
            first = self._first_coverage.get(rid, None)
            final = (
                self._ugv_status[rid].coverage_percent
                if rid in self._ugv_status
                else None
            )
            if first is not None and final is not None:
                print(f'    {rid}: {first:.1f}% → {final:.1f}% '
                      f'(+{final - first:.1f}%)')
            else:
                print(f'    {rid}: 데이터 없음')

        result = '[PASS]' if success else '[FAIL]'
        print(f'\n  최종 결과: {result}')
        print(f'{"#"*60}\n')

        rclpy.shutdown()
        sys.exit(0 if success else 1)


# ======================================================================
# 진입점
# ======================================================================

def main() -> None:
    rclpy.init()
    node = MultiRobotVerifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n[검증] Ctrl+C 감지. 현재까지 결과:')
        node._report()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
