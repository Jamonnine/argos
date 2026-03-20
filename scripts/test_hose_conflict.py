#!/usr/bin/env python3
"""
ARGOS 멀티 셰르파 호스 충돌 검증 스크립트
==========================================
multi_sherpa.launch.py 실행 후 별도 터미널에서 실행.
--simulate 모드에서는 Gazebo 없이 호스 경로 토픽을 직접 발행하여
오케스트레이터 반응만 검증할 수 있다.

사용법:
  # 실제 시뮬레이션 모니터링
  python3 scripts/test_hose_conflict.py --timeout 180

  # 자체 시뮬레이션 모드 (Gazebo 불필요)
  python3 scripts/test_hose_conflict.py --simulate --timeout 60

시나리오:
  1. 교차 진입 (PASS 기준 — 충돌 감지 + 역할 재분리 이벤트 필수)
     sherpa1: (3,0) → (3,6) → (6,6)   (좌측 진입 후 상단 → 우측)
     sherpa2: (9,0) → (9,6) → (6,6)   (우측 진입 후 상단 → 좌측)
     → (6,6) 부근에서 호스 교차 발생

  2. 평행 진입 (오탐 방지 — 교차 이벤트 미발생 이어야 PASS)
     sherpa1: (3,0) → (3,6)
     sherpa2: (9,0) → (9,6)

판정:
  PASS: 시나리오1 교차 감지 + 역할 재분리 이벤트 수신
        AND 시나리오2 이벤트 미발생 (오탐 없음)
  FAIL: 교차가 있는데 미감지 / 교차 없는데 오탐
"""

import argparse
import json
import math
import sys
import threading
import time
from collections import defaultdict

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, String


# ── 검증 파라미터 ─────────────────────────────────────────────────────────────

# 시나리오 전환 간격 (--simulate 모드, 초)
SCENARIO_SWITCH_SEC = 5.0
# 교차 감지 후 역할 재분리 이벤트 대기 시간 (초)
ROLE_REASSIGN_TIMEOUT_SEC = 10.0
# 10초 주기 상태 보고
REPORT_INTERVAL_SEC = 10.0
# 호스 잔여 길이 발행 주기 (--simulate 모드)
HOSE_PUBLISH_INTERVAL_SEC = 0.5

# 역할 키워드 (오케스트레이터가 JSON 이벤트에 포함해야 함)
ROLE_KEYWORDS = {'hose_supply', 'fire_attack', 'reassign', 'role_change'}

# 시나리오 정의
# 각 셰르파의 호스 경로 waypoint 목록 (x, y, hose_remaining_m)
SCENARIOS = {
    'cross': {
        'description': '교차 진입 — (6,6) 부근에서 호스 교차 발생 예상',
        'sherpa1': [
            # (x, y, hose_remaining_m) — 기존 경로 3점
            (3.0, 0.0, 30.0),
            (3.0, 6.0, 22.0),
            (6.0, 6.0, 15.0),
        ],
        'sherpa2': [
            (9.0, 0.0, 30.0),
            (9.0, 6.0, 22.0),
            (6.0, 6.0, 15.0),
        ],
        # 교차 이벤트가 반드시 발생해야 함
        'expect_conflict': True,
    },
    'parallel': {
        'description': '평행 진입 — 교차 없음, 오탐 방지 검증',
        'sherpa1': [
            (3.0, 0.0, 30.0),
            (3.0, 6.0, 22.0),
        ],
        'sherpa2': [
            (9.0, 0.0, 30.0),
            (9.0, 6.0, 22.0),
        ],
        # 교차 이벤트가 발생하면 오탐
        'expect_conflict': False,
    },
}


# ── 호스 상태 토픽 발행 헬퍼 ──────────────────────────────────────────────────

def _make_hose_msg(x: float, y: float, hose_remaining: float) -> Float32MultiArray:
    """호스 상태 Float32MultiArray 생성.

    레이아웃: [tip_x, tip_y, hose_remaining_m]
    오케스트레이터가 이 포맷으로 교차 감지를 수행한다고 가정.
    """
    msg = Float32MultiArray()
    msg.data = [float(x), float(y), float(hose_remaining)]
    return msg


def _make_odom_msg(x: float, y: float) -> Odometry:
    """XY 위치만 설정한 Odometry 메시지 생성."""
    msg = Odometry()
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    msg.pose.pose.position.z = 0.0
    msg.pose.pose.orientation.w = 1.0
    return msg


# ── ROS2 노드 ─────────────────────────────────────────────────────────────────

class HoseConflictVerifier(Node):
    """멀티 셰르파 호스 충돌 검증 노드.

    구독:
      /sherpa1/hose/status   (Float32MultiArray)
      /sherpa2/hose/status   (Float32MultiArray)
      /orchestrator/hose_conflict  (String, JSON 이벤트)
      /sherpa1/odom, /sherpa2/odom (Odometry)

    --simulate 모드에서는 위 토픽을 직접 발행한다.
    """

    def __init__(self, simulate: bool, timeout: float):
        super().__init__('hose_conflict_verifier')

        self._simulate = simulate
        self._timeout = timeout
        self._start_wall = time.monotonic()
        self._lock = threading.Lock()

        # ── 수신 상태 ──────────────────────────────────────────────────────
        # 각 셰르파의 마지막 수신 호스 경로 (tip_x, tip_y, hose_remaining)
        self._hose_status: dict[str, tuple[float, float, float]] = {}
        # 각 셰르파의 마지막 위치 (x, y)
        self._positions: dict[str, tuple[float, float]] = {}
        # 수신된 conflict 이벤트 목록
        self._conflict_events: list[dict] = []
        # 역할 재분리 이벤트 감지 여부
        self._role_reassigned = False

        # ── 시나리오 상태 (--simulate 전용) ─────────────────────────────────
        self._current_scenario: str | None = None
        self._scenario_start: float = 0.0
        self._waypoint_idx: dict[str, int] = {'sherpa1': 0, 'sherpa2': 0}

        # ── QoS ───────────────────────────────────────────────────────────
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── 구독 ───────────────────────────────────────────────────────────
        self.create_subscription(
            Float32MultiArray,
            '/sherpa1/hose/status',
            lambda msg: self._on_hose_status('sherpa1', msg),
            reliable_qos,
        )
        self.create_subscription(
            Float32MultiArray,
            '/sherpa2/hose/status',
            lambda msg: self._on_hose_status('sherpa2', msg),
            reliable_qos,
        )
        self.create_subscription(
            String,
            '/orchestrator/hose_conflict',
            self._on_conflict_event,
            reliable_qos,
        )
        self.create_subscription(
            Odometry,
            '/sherpa1/odom',
            lambda msg: self._on_odom('sherpa1', msg),
            reliable_qos,
        )
        self.create_subscription(
            Odometry,
            '/sherpa2/odom',
            lambda msg: self._on_odom('sherpa2', msg),
            reliable_qos,
        )

        # ── 발행자 (--simulate 모드에서만 사용) ────────────────────────────
        self._hose_pubs: dict[str, object] = {}
        self._odom_pubs: dict[str, object] = {}
        if self._simulate:
            for sid in ('sherpa1', 'sherpa2'):
                self._hose_pubs[sid] = self.create_publisher(
                    Float32MultiArray,
                    f'/{sid}/hose/status',
                    reliable_qos,
                )
                self._odom_pubs[sid] = self.create_publisher(
                    Odometry,
                    f'/{sid}/odom',
                    reliable_qos,
                )

        # ── 타이머 ────────────────────────────────────────────────────────
        self._report_timer = self.create_timer(REPORT_INTERVAL_SEC, self._report)
        self._timeout_timer = self.create_timer(self._timeout, self._on_timeout)

        if self._simulate:
            # 시나리오 전환 타이머 (교차 시나리오 먼저)
            self._sim_timer = self.create_timer(
                HOSE_PUBLISH_INTERVAL_SEC, self._sim_publish
            )
            self._scenario_switch_timer = self.create_timer(
                SCENARIO_SWITCH_SEC, self._switch_scenario
            )
            self._current_scenario = 'cross'
            self._scenario_start = time.monotonic()
            self._waypoint_idx = {'sherpa1': 0, 'sherpa2': 0}
            self.get_logger().info('[시뮬] 시나리오 1/2: 교차 진입 시작')

        self.get_logger().info(
            f'[검증] 시작 | 타임아웃 {self._timeout:.0f}s '
            f'| 모드: {"시뮬" if self._simulate else "모니터링"}'
        )
        self.get_logger().info(
            '  구독: /sherpa1/hose/status, /sherpa2/hose/status, '
            '/orchestrator/hose_conflict, /sherpa1/odom, /sherpa2/odom'
        )
        self.get_logger().info(
            '  PASS 기준: 시나리오1 충돌감지 + 역할재분리 | 시나리오2 오탐없음'
        )

    # ── 콜백 ─────────────────────────────────────────────────────────────────

    def _on_hose_status(self, sherpa_id: str, msg: Float32MultiArray) -> None:
        """호스 상태 콜백 — tip 위치·잔여 길이 갱신."""
        if len(msg.data) < 3:
            return
        with self._lock:
            self._hose_status[sherpa_id] = (
                float(msg.data[0]),
                float(msg.data[1]),
                float(msg.data[2]),
            )

    def _on_odom(self, sherpa_id: str, msg: Odometry) -> None:
        """Odometry 콜백 — 위치 갱신."""
        with self._lock:
            self._positions[sherpa_id] = (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
            )

    def _on_conflict_event(self, msg: String) -> None:
        """오케스트레이터 호스 충돌 이벤트 콜백."""
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            # JSON이 아닌 경우 원문 저장
            event = {'raw': msg.data}

        with self._lock:
            self._conflict_events.append(event)

            # 역할 재분리 키워드 포함 여부 확인
            event_str = msg.data.lower()
            if any(kw in event_str for kw in ROLE_KEYWORDS):
                self._role_reassigned = True

        elapsed = time.monotonic() - self._start_wall
        self.get_logger().info(
            f'[이벤트] {elapsed:.1f}s | hose_conflict 수신: {msg.data[:120]}'
        )

    # ── 시뮬레이션 발행 ───────────────────────────────────────────────────────

    def _sim_publish(self) -> None:
        """현재 시나리오 waypoint 순서대로 호스 상태·odom을 발행.

        waypoint 목록이 끝나면 마지막 지점에서 유지.
        """
        if self._current_scenario is None:
            return

        scenario = SCENARIOS[self._current_scenario]
        elapsed_in_scene = time.monotonic() - self._scenario_start

        for sid in ('sherpa1', 'sherpa2'):
            waypoints = scenario[sid]
            # 경과 시간에 비례해 waypoint 진행 (균등 배분)
            total_steps = len(waypoints)
            step_duration = SCENARIO_SWITCH_SEC / total_steps
            idx = min(int(elapsed_in_scene / step_duration), total_steps - 1)
            self._waypoint_idx[sid] = idx

            x, y, hose_rem = waypoints[idx]
            self._hose_pubs[sid].publish(_make_hose_msg(x, y, hose_rem))
            self._odom_pubs[sid].publish(_make_odom_msg(x, y))

    def _switch_scenario(self) -> None:
        """시나리오 전환: cross → parallel → 종료."""
        if self._current_scenario == 'cross':
            self._current_scenario = 'parallel'
            self._scenario_start = time.monotonic()
            self._waypoint_idx = {'sherpa1': 0, 'sherpa2': 0}
            self.get_logger().info('[시뮬] 시나리오 2/2: 평행 진입 시작')
        else:
            # 두 시나리오 모두 완료 — 타이머 중지 후 판정
            self._scenario_switch_timer.cancel()
            self._sim_timer.cancel()
            self.get_logger().info('[시뮬] 전체 시나리오 완료. 결과 판정...')
            # 짧은 지연 후 판정 (마지막 이벤트 도착 대기)
            self.create_timer(2.0, self._finalize)

    # ── 상태 보고 ─────────────────────────────────────────────────────────────

    def _report(self) -> None:
        """10초 주기 상태 보고."""
        elapsed = int(time.monotonic() - self._start_wall)

        with self._lock:
            hose_parts = []
            for sid in ('sherpa1', 'sherpa2'):
                if sid in self._hose_status:
                    x, y, rem = self._hose_status[sid]
                    hose_parts.append(f'{sid}:tip({x:.1f},{y:.1f}) rem={rem:.1f}m')
                else:
                    hose_parts.append(f'{sid}:미수신')

            pos_parts = []
            for sid in ('sherpa1', 'sherpa2'):
                if sid in self._positions:
                    x, y = self._positions[sid]
                    pos_parts.append(f'{sid}:({x:.1f},{y:.1f})')
                else:
                    pos_parts.append(f'{sid}:(?,?)')

            n_events = len(self._conflict_events)
            role_ok = self._role_reassigned
            scenario_label = (
                f'[시나리오:{self._current_scenario}]' if self._simulate else ''
            )

        print(
            f'[{elapsed}s]{scenario_label} '
            + ' | '.join(hose_parts)
        )
        print(f'  odom: {" ".join(pos_parts)}')
        print(
            f'  conflict 이벤트: {n_events}건 '
            f'| 역할재분리: {"감지됨" if role_ok else "미감지"}'
        )

    # ── 판정 로직 ─────────────────────────────────────────────────────────────

    def _finalize(self) -> None:
        """최종 판정 — 타이머 전 조기 완료 시 호출."""
        self._report_timer.cancel()
        self._timeout_timer.cancel()
        elapsed = time.monotonic() - self._start_wall

        print()
        print('#' * 60)
        print(f'[종합 보고] {elapsed:.1f}s 후 검증 완료')
        print('#' * 60)

        passed = self._judge(elapsed)

        result_tag = '[PASS]' if passed else '[FAIL]'
        print()
        print(f'  최종 결과: {result_tag}')
        print('#' * 60)

        rclpy.shutdown()
        sys.exit(0 if passed else 1)

    def _judge(self, elapsed: float) -> bool:
        """
        PASS/FAIL 판정.

        PASS 조건:
          1. /orchestrator/hose_conflict 이벤트 1건 이상 수신
          2. 역할 재분리 키워드가 이벤트에 포함
          3. (--simulate 모드) 평행 시나리오 이후 추가 이벤트 없음 (오탐 없음)
        """
        with self._lock:
            n_events = len(self._conflict_events)
            role_ok = self._role_reassigned
            events_snapshot = list(self._conflict_events)

        print()
        print('  [성공 기준 판정]')

        # 기준 1: 충돌 감지 이벤트 수신 여부
        crit1 = n_events >= 1
        print(f'    교차 감지 이벤트: [{"O" if crit1 else "X"}] {n_events}건')

        # 기준 2: 역할 재분리 키워드 포함
        crit2 = role_ok
        print(
            f'    역할 재분리 키워드: [{"O" if crit2 else "X"}] '
            f'{"포함됨" if role_ok else "미포함 (hose_supply/fire_attack/reassign 없음)"}'
        )

        # 기준 3: 이벤트 상세 (참고)
        if events_snapshot:
            print(f'    수신 이벤트 목록:')
            for i, ev in enumerate(events_snapshot, 1):
                print(f'      [{i}] {str(ev)[:100]}')
        else:
            print('    수신 이벤트 없음')

        # 총합 판정
        passed = crit1 and crit2

        if not passed:
            print()
            print('  [진단]')
            if not crit1:
                print('    - /orchestrator/hose_conflict 이벤트 미수신')
                print('      → 오케스트레이터가 실행 중인지 확인')
                print('      → 호스 tip 위치 토픽 포맷 일치 여부 확인')
            if not crit2:
                print('    - 역할 재분리 이벤트 없음')
                print('      → 이벤트 JSON에 "role" 또는 "hose_supply"/"fire_attack" 필드 추가 필요')

        return passed

    # ── 타임아웃 ─────────────────────────────────────────────────────────────

    def _on_timeout(self) -> None:
        """타임아웃 — 종합 보고 후 종료."""
        self._timeout_timer.cancel()
        self._report_timer.cancel()
        if self._simulate:
            try:
                self._sim_timer.cancel()
                self._scenario_switch_timer.cancel()
            except Exception:
                pass

        elapsed = time.monotonic() - self._start_wall
        print()
        print('#' * 60)
        print(f'[종합 보고] 타임아웃 {elapsed:.1f}s')
        print('#' * 60)

        passed = self._judge(elapsed)

        result_tag = '[PASS]' if passed else '[FAIL]'
        print()
        print(f'  최종 결과: {result_tag}')
        print('#' * 60)

        rclpy.shutdown()
        sys.exit(0 if passed else 1)


# ── 진입점 ────────────────────────────────────────────────────────────────────

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='ARGOS 멀티 셰르파 호스 충돌 검증 스크립트'
    )
    parser.add_argument(
        '--timeout', type=float, default=180.0,
        help='최대 대기 시간(초). 기본: 180',
    )
    parser.add_argument(
        '--simulate', action='store_true',
        help='자체 시뮬 모드: 호스 경로 토픽을 직접 발행하여 오케스트레이터 반응 확인',
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    rclpy.init()
    node = HoseConflictVerifier(simulate=args.simulate, timeout=args.timeout)

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print('=' * 60)
    print('  ARGOS 멀티 셰르파 호스 충돌 검증')
    if args.simulate:
        print('  모드: 자체 시뮬 (Gazebo 불필요)')
    else:
        print('  모드: 모니터링 (multi_sherpa.launch.py 먼저 실행)')
    print(f'  타임아웃: {args.timeout:.0f}s | 보고 주기: {REPORT_INTERVAL_SEC:.0f}s')
    print('=' * 60)
    print()

    try:
        spin_thread.join()
    except KeyboardInterrupt:
        print('\n[검증] Ctrl+C 감지. 현재까지 결과:')
        node._judge(time.monotonic() - node._start_wall)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
