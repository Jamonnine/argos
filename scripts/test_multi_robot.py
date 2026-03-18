#!/usr/bin/env python3
"""
ARGOS 멀티로봇 협업탐색 검증 스크립트
=======================================
exploration.launch.py 실행 후 별도 터미널에서 실행.

사용법:
  python3 scripts/test_multi_robot.py

성공 기준 (3개 모두 충족 시 PASS):
  1. 3대 모두 이동 확인 (odom 누적 거리 > 0.5m)
  2. 충돌 0건 (로봇 간 최소 거리 >= 0.8m 유지)
  3. 300초 이내 완료

로봇 스폰 위치:
  - argos1 (UGV): (3.0, 2.5)
  - argos2 (UGV): (7.0, 2.5)
  - drone1 (Drone): (5.0, 4.0)
"""

import math
import sys
import time
from collections import defaultdict

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)
from nav_msgs.msg import Odometry, OccupancyGrid

# argos_interfaces 선택적 임포트 (없으면 생략)
try:
    from argos_interfaces.msg import MissionState
    _HAS_MISSION_STATE = True
except ImportError:
    _HAS_MISSION_STATE = False

# 검증 파라미터
TIMEOUT_SEC = 300            # RTF 0.28x 보정 — wall time 기준
REPORT_INTERVAL_SEC = 10     # 상태 보고 주기
COLLISION_DISTANCE_M = 0.8   # 이 거리 미만이면 충돌 경고
MOVE_THRESHOLD_M = 0.5       # 이 거리 이상 이동해야 "이동 확인"
COVERAGE_PASS_PCT = 25.0     # PASS 판정 coverage 기준 (%)
COLLISION_CHECK_INTERVAL = 1.0  # 충돌 감지 주기 (초)

# 오케스트레이터 스테이지 이름
STAGE_NAMES = {
    0: 'INIT',
    1: 'EXPLORING',
    2: 'FIRE_RESPONSE',
    3: 'RETURNING',
    4: 'COMPLETE',
    5: 'PAUSED',
}

# 모니터링 대상 로봇
ALL_ROBOT_IDS = ('argos1', 'argos2', 'drone1')


class MultiRobotVerifier(Node):
    """멀티로봇 검증 노드 — 3대(UGV×2 + Drone×1) 동시 모니터링."""

    def __init__(self):
        super().__init__('multi_robot_verifier')

        # --- QoS 프로파일 ---
        # UGV odom: DDC가 RELIABLE+TRANSIENT_LOCAL로 발행
        ugv_odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # Drone odom: VOLATILE 발행 (MulticopterVelocityControl)
        drone_odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # /map: TRANSIENT_LOCAL (늦게 구독해도 최신 맵 수신)
        transient_local_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # --- 내부 상태 ---
        # 현재 위치 {robot_id: (x, y, z)}  — drone1은 z축 유효
        self._positions: dict[str, tuple[float, float, float]] = {}
        # 직전 위치 (누적 거리 계산용)
        self._prev_positions: dict[str, tuple[float, float, float]] = {}
        # 누적 이동 거리 {robot_id: float}
        self._traveled: dict[str, float] = defaultdict(float)
        # 충돌 경고 누적 횟수
        self._collision_count = 0
        # 마지막 충돌 감지 시각 (1초 쿨다운)
        self._last_collision_check = 0.0
        # /map 기반 coverage
        self._coverage_pct: float = 0.0
        # 오케스트레이터 상태
        self._mission_state = None
        # 시작 시각
        self._start_wall = time.monotonic()

        # --- 구독 ---
        # UGV odom
        for robot_id in ('argos1', 'argos2'):
            self.create_subscription(
                Odometry,
                f'/{robot_id}/odom',
                lambda msg, rid=robot_id: self._on_odom(rid, msg),
                ugv_odom_qos,
            )

        # Drone odom — z축 이동도 유효 (VOLATILE QoS)
        self.create_subscription(
            Odometry,
            '/drone1/odom',
            lambda msg: self._on_odom('drone1', msg),
            drone_odom_qos,
        )

        # /map 구독 — TRANSIENT_LOCAL QoS 필수
        self.create_subscription(
            OccupancyGrid,
            '/map',
            self._on_map,
            transient_local_qos,
        )

        # 오케스트레이터 상태
        if _HAS_MISSION_STATE:
            self.create_subscription(
                MissionState,
                '/orchestrator/mission_state',
                self._on_mission_state,
                ugv_odom_qos,
            )

        # --- 타이머 ---
        # 10초 주기 보고
        self._report_timer = self.create_timer(REPORT_INTERVAL_SEC, self._report)
        # 1초 주기 충돌 감지
        self._collision_timer = self.create_timer(
            COLLISION_CHECK_INTERVAL, self._check_collision
        )
        # 타임아웃
        self._timeout_timer = self.create_timer(TIMEOUT_SEC, self._on_timeout)

        self.get_logger().info(
            f'[검증] 시작 | 타임아웃 {TIMEOUT_SEC}s | 보고 {REPORT_INTERVAL_SEC}s 주기'
        )
        self.get_logger().info(
            '  구독: /argos1/odom, /argos2/odom, /drone1/odom, /map'
            + (', /orchestrator/mission_state' if _HAS_MISSION_STATE else '')
        )
        self.get_logger().info(
            f'  PASS 기준: 3대 이동 >{MOVE_THRESHOLD_M}m + 충돌 0건 + {TIMEOUT_SEC}s 이내'
        )

    # ------------------------------------------------------------------
    # 콜백
    # ------------------------------------------------------------------

    def _on_odom(self, robot_id: str, msg: Odometry) -> None:
        """odom 콜백 — 위치 갱신 및 누적 거리 계산."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        cur = (x, y, z)

        # 누적 이동 거리 계산
        if robot_id in self._prev_positions:
            prev = self._prev_positions[robot_id]
            # drone1은 3D 거리, UGV는 2D 거리 (z 변화 무시)
            if robot_id == 'drone1':
                delta = math.sqrt(
                    (cur[0] - prev[0]) ** 2
                    + (cur[1] - prev[1]) ** 2
                    + (cur[2] - prev[2]) ** 2
                )
            else:
                delta = math.sqrt(
                    (cur[0] - prev[0]) ** 2
                    + (cur[1] - prev[1]) ** 2
                )
            # 노이즈 필터 — 1cm 미만 미세 진동 무시
            if delta > 0.01:
                self._traveled[robot_id] += delta

        self._prev_positions[robot_id] = cur
        self._positions[robot_id] = cur

    def _on_map(self, msg: OccupancyGrid) -> None:
        """OccupancyGrid 콜백 — 탐색률(explored %) 계산."""
        total = msg.info.width * msg.info.height
        if total == 0:
            return
        # 알려진 셀 (0=자유공간, 100=장애물) / -1은 unknown
        known = sum(1 for v in msg.data if v >= 0)
        self._coverage_pct = (known / total) * 100.0

    def _on_mission_state(self, msg) -> None:
        """오케스트레이터 MissionState 콜백."""
        self._mission_state = msg

    # ------------------------------------------------------------------
    # 충돌 감지 (1초 주기 타이머)
    # ------------------------------------------------------------------

    def _check_collision(self) -> None:
        """등록된 모든 로봇 쌍 간 2D 거리 확인 — 0.8m 미만이면 경고."""
        # 이동 전(원점) 로봇만 있으면 충돌 체크 스킵
        moved_robots = [rid for rid in ALL_ROBOT_IDS
                        if rid in self._positions and self._traveled.get(rid, 0) > 0.1]
        if len(moved_robots) < 2:
            return

        for i in range(len(moved_robots)):
            for j in range(i + 1, len(moved_robots)):
                rid_a = moved_robots[i]
                rid_b = moved_robots[j]
                dist = self._dist2d(
                    self._positions[rid_a],
                    self._positions[rid_b],
                )
                if dist < COLLISION_DISTANCE_M:
                    self._collision_count += 1
                    self.get_logger().warn(
                        f'[충돌 경고] {rid_a} <-> {rid_b} '
                        f'거리 {dist:.2f}m (기준 {COLLISION_DISTANCE_M}m) '
                        f'| 누적 {self._collision_count}회'
                    )

    @staticmethod
    def _dist2d(
        a: tuple[float, float, float],
        b: tuple[float, float, float],
    ) -> float:
        """XY 평면 2D 거리 (드론 고도 무시 — 지상 로봇과의 충돌은 XY 기준)."""
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def _min_robot_distance(self) -> float:
        """현재 로봇 간 최소 2D 거리. 위치 미수신이면 inf 반환."""
        robot_ids = [rid for rid in ALL_ROBOT_IDS if rid in self._positions]
        min_dist = float('inf')
        for i in range(len(robot_ids)):
            for j in range(i + 1, len(robot_ids)):
                d = self._dist2d(
                    self._positions[robot_ids[i]],
                    self._positions[robot_ids[j]],
                )
                min_dist = min(min_dist, d)
        return min_dist

    # ------------------------------------------------------------------
    # 상태 보고 (10초 주기)
    # ------------------------------------------------------------------

    def _report(self) -> None:
        """인라인 형식으로 상태 보고."""
        elapsed = time.monotonic() - self._start_wall
        elapsed_int = int(elapsed)

        # 위치 문자열
        pos_parts = []
        for rid in ALL_ROBOT_IDS:
            if rid in self._positions:
                x, y, z = self._positions[rid]
                if rid == 'drone1':
                    pos_parts.append(f'{rid}:({x:.1f},{y:.1f},z{z:.1f})')
                else:
                    pos_parts.append(f'{rid}:({x:.1f},{y:.1f})')
            else:
                pos_parts.append(f'{rid}:(?,?)')

        # 최소 거리
        min_dist = self._min_robot_distance()
        dist_str = f'{min_dist:.1f}m' if min_dist != float('inf') else '?m'

        # 출력 한 줄
        line = (
            f'[{elapsed_int}s] '
            + ' '.join(pos_parts)
            + f' | min_dist:{dist_str}'
            + f' | coverage:{self._coverage_pct:.1f}%'
        )
        print(line)

        # 이동 거리 상세 (오케스트레이터 포함)
        travel_parts = [
            f'{rid}:{self._traveled[rid]:.2f}m'
            for rid in ALL_ROBOT_IDS
        ]
        print(f'  traveled: {" ".join(travel_parts)} | collisions:{self._collision_count}')

        # 오케스트레이터 상태
        if self._mission_state is not None:
            ms = self._mission_state
            stage = STAGE_NAMES.get(ms.stage, f'?({ms.stage})')
            print(f'  orchestrator: stage={stage}')
        elif _HAS_MISSION_STATE:
            print('  orchestrator: /orchestrator/mission_state 대기 중...')

        # 성공 기준 실시간 점검
        self._check_pass_criteria(print_result=False)

    # ------------------------------------------------------------------
    # 성공 기준 점검
    # ------------------------------------------------------------------

    def _check_pass_criteria(self, print_result: bool = True) -> bool:
        """
        PASS 기준 3개 동시 충족 여부 반환.

        1. 3대 모두 이동 확인 (누적 거리 > MOVE_THRESHOLD_M)
        2. 충돌 0건
        3. 300초 이내 (타임아웃 전 호출 시 자동 충족)
        """
        elapsed = time.monotonic() - self._start_wall

        # 기준 1: 3대 이동 확인
        all_moved = all(
            self._traveled[rid] >= MOVE_THRESHOLD_M
            for rid in ALL_ROBOT_IDS
        )

        # 기준 2: 충돌 없음
        no_collision = self._collision_count == 0

        # 기준 3: 타임아웃 이내
        within_timeout = elapsed <= TIMEOUT_SEC

        passed = all_moved and no_collision and within_timeout

        if print_result:
            print()
            print(f'  [성공 기준 판정]')
            for rid in ALL_ROBOT_IDS:
                moved = self._traveled[rid] >= MOVE_THRESHOLD_M
                sym = 'O' if moved else 'X'
                print(
                    f'    {rid} 이동: [{sym}] '
                    f'{self._traveled[rid]:.2f}m / {MOVE_THRESHOLD_M}m'
                )
            print(f'    충돌 없음:  [{"O" if no_collision else "X"}] '
                  f'{self._collision_count}건')
            print(f'    시간 이내:  [{"O" if within_timeout else "X"}] '
                  f'{elapsed:.1f}s / {TIMEOUT_SEC}s')
            print(f'    coverage:  {self._coverage_pct:.1f}%')

        if passed:
            coverage_tag = f'coverage {self._coverage_pct:.1f}%+'
            print(
                f'[PASS] 3대 이동 확인 + 충돌 0건 + {coverage_tag}'
            )

        return passed

    # ------------------------------------------------------------------
    # 타임아웃
    # ------------------------------------------------------------------

    def _on_timeout(self) -> None:
        """300초 타임아웃 — 종합 보고 후 종료."""
        self._timeout_timer.cancel()
        self._report_timer.cancel()
        self._collision_timer.cancel()

        elapsed = time.monotonic() - self._start_wall
        print()
        print('#' * 60)
        print(f'[종합 보고] {elapsed:.1f}s 후 검증 완료')
        print('#' * 60)

        success = self._check_pass_criteria(print_result=True)

        # 이동 거리 요약
        print()
        print('  [이동 거리 요약]')
        for rid in ALL_ROBOT_IDS:
            moved = self._traveled[rid] >= MOVE_THRESHOLD_M
            sym = 'O' if moved else 'X'
            print(f'    {rid}: {self._traveled[rid]:.3f}m [{sym}]')

        result_tag = '[PASS]' if success else '[FAIL]'
        print()
        print(f'  최종 결과: {result_tag}')
        print('#' * 60)

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
        node._check_pass_criteria(print_result=True)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
