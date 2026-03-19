#!/usr/bin/env python3
"""
ARGOS PX4 Offboard 비행 검증 스크립트
=======================================
PX4 SITL Docker 환경에서 드론 offboard 비행 시나리오를 자동 검증한다.

사전 조건:
  1. PX4 SITL 기동 (gz sim + PX4)
  2. Micro XRCE-DDS Agent 실행: MicroXRCEAgent udp4 -p 8888
  3. px4_bridge_node 실행 (use_px4:=true)

사용법:
  # 기본 실행 (300초 타임아웃)
  python3 scripts/test_px4_offboard.py

  # 타임아웃 변경
  python3 scripts/test_px4_offboard.py --timeout 600

  # px4_bridge_node 네임스페이스 지정 (멀티드론)
  python3 scripts/test_px4_offboard.py --ns /drone1

  # 궤적 CSV 저장 경로 변경
  python3 scripts/test_px4_offboard.py --csv /tmp/flight_trace.csv

검증 시나리오:
  1. px4_bridge_node 존재 확인 (토픽 응답 여부)
  2. ARM → Offboard 모드 전환
  3. 이륙 (3m 고도)
  4. 웨이포인트 4점 순회:
       (2,0,3) → (2,2,3) → (0,2,3) → (0,0,3)
  5. 각 웨이포인트 1m 이내 도달 확인
  6. 착륙 후 z < 0.5m 확인
  7. 전체 궤적 CSV 저장 (timestamp, x, y, z)

판정 기준 (모두 충족 시 PASS):
  - 웨이포인트 4점 모두 1m 이내 도달
  - 착륙 후 z < 0.5m
  - 전체 소요 시간 300초 이내
"""

import argparse
import csv
import math
import sys
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# px4_msgs 선택적 임포트 (미설치 시 명확한 오류 메시지)
try:
    from px4_msgs.msg import (
        OffboardControlMode,
        TrajectorySetpoint,
        VehicleCommand,
        VehicleLocalPosition,
        VehicleStatus,
    )
    PX4_MSGS_AVAILABLE = True
except ImportError:
    PX4_MSGS_AVAILABLE = False

# nav_msgs Odometry: px4_bridge_node가 /px4/odom으로 발행하는 폴백 토픽
from nav_msgs.msg import Odometry


# ── 상수 ──────────────────────────────────────────────────────────────────────

# 웨이포인트 (ENU 좌표, 단위: m)
WAYPOINTS_ENU = [
    (2.0, 0.0, 3.0),
    (2.0, 2.0, 3.0),
    (0.0, 2.0, 3.0),
    (0.0, 0.0, 3.0),
]

TAKEOFF_ALT_M = 3.0         # 이륙 목표 고도 (m)
WAYPOINT_TOL_M = 1.0        # 웨이포인트 도달 허용 반경 (m)
LAND_Z_THRESHOLD_M = 0.5    # 착륙 완료 고도 기준 (m)
OFFBOARD_PRE_SEND_COUNT = 20  # offboard 전환 전 heartbeat 선발송 횟수 (PX4 요구사항)
OFFBOARD_RATE_HZ = 10.0     # OffboardControlMode 발행 주기

# 상태 이름 (출력용)
STATES = [
    'INIT',          # 0
    'ARM',           # 1
    'TAKEOFF',       # 2
    'WAYPOINT',      # 3
    'LAND',          # 4
    'DONE',          # 5
    'FAILED',        # 6
]


# ── ENU ↔ NED 변환 (px4_bridge_node.py와 동일) ──────────────────────────────

def enu_to_ned(x_enu: float, y_enu: float, z_enu: float):
    """ENU (ROS2) → NED (PX4) 좌표 변환."""
    return y_enu, x_enu, -z_enu  # N=ENU_Y, E=ENU_X, D=-ENU_Z


def ned_to_enu(x_ned: float, y_ned: float, z_ned: float):
    """NED (PX4) → ENU (ROS2) 좌표 변환."""
    return y_ned, x_ned, -z_ned  # E=NED_Y, N=NED_X, U=-NED_Z


# ── 검증 노드 ─────────────────────────────────────────────────────────────────

class PX4OffboardVerifier(Node):
    """PX4 Offboard 비행 검증 노드.

    ARM → Offboard 전환 → 이륙 → 웨이포인트 순회 → 착륙 순서로
    자동 진행하고, 각 단계별 판정 기준을 검증한다.

    px4_msgs 미설치 환경에서는 /px4/odom (Odometry) 토픽만 수신해
    위치 확인까지만 수행하고, 커맨드 발행은 불가 상태로 명확히 보고한다.
    """

    def __init__(self, fmu_ns: str, csv_path: str, timeout_sec: float):
        super().__init__('px4_offboard_verifier')

        self._fmu_ns = fmu_ns          # /fmu/in|out 접두사 (멀티드론: /px4_1 등)
        self._csv_path = csv_path
        self._timeout_sec = timeout_sec

        # ── 내부 상태 ──────────────────────────────────────────────
        self._state_idx = 0            # 현재 단계 인덱스 (STATES 리스트 기준)
        self._lock = threading.Lock()

        # 현재 위치 (ENU 좌표, VehicleLocalPosition 또는 Odometry 수신 후 갱신)
        self._pos_enu: tuple[float, float, float] | None = None
        self._armed = False            # arming 상태
        self._nav_state = 0            # PX4 nav_state

        # 웨이포인트 진행 추적
        self._wp_index = 0             # 현재 목표 웨이포인트 인덱스
        self._wp_reached = [False] * len(WAYPOINTS_ENU)

        # offboard heartbeat 카운터 (전환 전 선발송용)
        self._offboard_sent_count = 0

        # 궤적 기록 (timestamp, x, y, z)
        self._trajectory: list[tuple[float, float, float, float]] = []
        self._start_wall = time.monotonic()

        # 결과
        self._result: str | None = None  # 'PASS' | 'FAIL' | 'TIMEOUT'

        # ── QoS 프로파일 ──────────────────────────────────────────
        # PX4 SITL: BEST_EFFORT + VOLATILE (빠른 상태 갱신)
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── 구독 ──────────────────────────────────────────────────
        if PX4_MSGS_AVAILABLE:
            # PX4 VehicleLocalPosition: 위치 추정 (NED 좌표, ENU로 변환)
            self._local_pos_sub = self.create_subscription(
                VehicleLocalPosition,
                f'{self._fmu_ns}/fmu/out/vehicle_local_position',
                self._local_pos_cb,
                px4_qos,
            )
            # VehicleStatus: arming + nav_state 모니터링
            self._status_sub = self.create_subscription(
                VehicleStatus,
                f'{self._fmu_ns}/fmu/out/vehicle_status',
                self._vehicle_status_cb,
                px4_qos,
            )
        else:
            # px4_msgs 미설치: /px4/odom (Odometry) 폴백
            # px4_bridge_node의 passthrough 모드에서 발행됨
            self._odom_sub = self.create_subscription(
                Odometry,
                '/px4/odom',
                self._odom_cb,
                10,
            )
            self.get_logger().warn(
                'px4_msgs not installed — 위치 모니터링만 가능. '
                'ARM/Offboard 커맨드는 px4_msgs 설치 후 사용 가능.')

        # ── 발행자 (px4_msgs 필요) ────────────────────────────────
        if PX4_MSGS_AVAILABLE:
            self._offboard_pub = self.create_publisher(
                OffboardControlMode,
                f'{self._fmu_ns}/fmu/in/offboard_control_mode',
                10,
            )
            self._setpoint_pub = self.create_publisher(
                TrajectorySetpoint,
                f'{self._fmu_ns}/fmu/in/trajectory_setpoint',
                10,
            )
            self._cmd_pub = self.create_publisher(
                VehicleCommand,
                f'{self._fmu_ns}/fmu/in/vehicle_command',
                10,
            )

        # ── 타이머 ────────────────────────────────────────────────
        # offboard heartbeat + 상태 머신 (10Hz)
        self._ctrl_timer = self.create_timer(
            1.0 / OFFBOARD_RATE_HZ, self._control_loop)

        # 상태 보고 (5초 주기)
        self._report_timer = self.create_timer(5.0, self._report)

        # 타임아웃
        self._timeout_timer = self.create_timer(
            self._timeout_sec, self._on_timeout)

        self.get_logger().info(
            f'PX4 Offboard 검증 시작 | fmu_ns={self._fmu_ns!r} | '
            f'타임아웃={self._timeout_sec:.0f}s | csv={self._csv_path!r} | '
            f'px4_msgs={PX4_MSGS_AVAILABLE}')
        self.get_logger().info(
            f'웨이포인트 {len(WAYPOINTS_ENU)}점: {WAYPOINTS_ENU}')

    # ── 콜백 ──────────────────────────────────────────────────────────────────

    def _local_pos_cb(self, msg: 'VehicleLocalPosition') -> None:
        """VehicleLocalPosition 수신 — NED → ENU 변환 후 내부 위치 갱신."""
        # PX4 VehicleLocalPosition: x=North, y=East, z=Down (NED)
        x_enu, y_enu, z_enu = ned_to_enu(msg.x, msg.y, msg.z)
        with self._lock:
            self._pos_enu = (x_enu, y_enu, z_enu)
            self._record_trajectory(x_enu, y_enu, z_enu)

    def _odom_cb(self, msg: Odometry) -> None:
        """Odometry 폴백 수신 (px4_msgs 미설치 시). ENU 좌표 그대로 사용."""
        p = msg.pose.pose.position
        with self._lock:
            self._pos_enu = (p.x, p.y, p.z)
            self._record_trajectory(p.x, p.y, p.z)

    def _vehicle_status_cb(self, msg: 'VehicleStatus') -> None:
        """VehicleStatus 수신 — arming_state + nav_state 갱신."""
        with self._lock:
            prev_armed = self._armed
            # arming_state: 1=STANDBY, 2=ARMED
            self._armed = (msg.arming_state == 2)
            self._nav_state = msg.nav_state
            if prev_armed != self._armed:
                state = 'ARMED' if self._armed else 'DISARMED'
                self.get_logger().info(
                    f'[VehicleStatus] arming → {state} (nav_state={self._nav_state})')

    def _record_trajectory(self, x: float, y: float, z: float) -> None:
        """궤적 기록 (lock 내부에서 호출)."""
        elapsed = time.monotonic() - self._start_wall
        self._trajectory.append((elapsed, x, y, z))

    # ── 상태 머신 ──────────────────────────────────────────────────────────────

    def _control_loop(self) -> None:
        """10Hz 제어 루프 — offboard heartbeat 발행 + 상태 머신 실행."""
        if not PX4_MSGS_AVAILABLE:
            # px4_msgs 없음: 위치만 모니터링 (커맨드 불가)
            self._monitor_only()
            return

        # offboard heartbeat 항상 발행 (PX4 offboard 모드 유지 필수)
        self._publish_offboard_heartbeat()

        with self._lock:
            state = STATES[self._state_idx]
            pos = self._pos_enu
            armed = self._armed
            nav_state = self._nav_state

        if state == 'INIT':
            self._do_init(pos, armed)
        elif state == 'ARM':
            self._do_arm(armed, nav_state)
        elif state == 'TAKEOFF':
            self._do_takeoff(pos)
        elif state == 'WAYPOINT':
            self._do_waypoint(pos)
        elif state == 'LAND':
            self._do_land(pos)
        # DONE / FAILED: 루프는 계속 돌지만 액션 없음

    def _monitor_only(self) -> None:
        """px4_msgs 미설치 환경: 위치 수신 여부만 보고."""
        with self._lock:
            pos = self._pos_enu
        if pos is not None and self._state_idx == 0:
            self.get_logger().info(
                f'[위치 수신 확인] ENU=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})')
            with self._lock:
                self._state_idx = 6  # FAILED (커맨드 불가)
            self._fail('px4_msgs 미설치 — ARM/Offboard 불가. '
                       '위치 수신은 확인됨.')

    def _do_init(self, pos, armed) -> None:
        """INIT: px4_bridge_node 응답 확인 (위치 토픽 수신 여부)."""
        self._offboard_sent_count += 1

        if pos is None:
            # 아직 위치 미수신: 최초 5회는 대기 (노드 기동 딜레이 허용)
            if self._offboard_sent_count % 50 == 0:  # 5초마다 로그
                self.get_logger().warn(
                    f'[INIT] 위치 토픽 미수신 ({self._offboard_sent_count / OFFBOARD_RATE_HZ:.0f}s 경과). '
                    f'px4_bridge_node와 PX4 SITL 실행 중인지 확인.')
            return

        # 위치 수신 확인 → ARM 단계로 진행
        self.get_logger().info(
            f'[INIT] 위치 토픽 수신 확인. '
            f'ENU=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})')

        # offboard 전환 전 선발송 횟수 충족 여부 확인
        if self._offboard_sent_count < OFFBOARD_PRE_SEND_COUNT:
            return  # 아직 선발송 중

        self.get_logger().info(
            f'[INIT → ARM] offboard heartbeat {self._offboard_sent_count}회 선발송 완료.')
        with self._lock:
            self._state_idx = 1  # ARM

    def _do_arm(self, armed: bool, nav_state: int) -> None:
        """ARM: arm 명령 + offboard 모드 전환."""
        if not armed:
            # ARM 명령 발행
            self._publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            self.get_logger().info('[ARM] ARM 명령 발행...')
        else:
            # arm 확인 → offboard 모드 전환 명령
            # nav_state=14: OFFBOARD 모드 (px4_msgs 상수)
            if nav_state != 14:
                self._publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                    param1=1.0,   # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                    param2=6.0,   # PX4_CUSTOM_MAIN_MODE_OFFBOARD
                )
                self.get_logger().info(
                    f'[ARM] ARMED 확인 → Offboard 모드 전환 명령 (현재 nav_state={nav_state})')
            else:
                self.get_logger().info('[ARM → TAKEOFF] Offboard 모드 확인. 이륙 시작.')
                # 이륙 setpoint 먼저 발행 (현재 위치 기준)
                with self._lock:
                    pos = self._pos_enu
                if pos:
                    self._publish_position_setpoint(pos[0], pos[1], TAKEOFF_ALT_M)
                with self._lock:
                    self._state_idx = 2  # TAKEOFF

    def _do_takeoff(self, pos) -> None:
        """TAKEOFF: 3m 고도까지 상승 확인."""
        if pos is None:
            return

        # 이륙 목표 setpoint 지속 발행 (현재 XY 위치 유지, Z=3m)
        self._publish_position_setpoint(pos[0], pos[1], TAKEOFF_ALT_M)

        alt_err = abs(pos[2] - TAKEOFF_ALT_M)
        if alt_err < WAYPOINT_TOL_M:
            self.get_logger().info(
                f'[TAKEOFF] 이륙 완료! z={pos[2]:.2f}m (목표 {TAKEOFF_ALT_M}m, 오차 {alt_err:.2f}m)')
            with self._lock:
                self._state_idx = 3  # WAYPOINT
                self._wp_index = 0

    def _do_waypoint(self, pos) -> None:
        """WAYPOINT: 웨이포인트 4점 순회."""
        if pos is None:
            return

        with self._lock:
            wp_idx = self._wp_index

        if wp_idx >= len(WAYPOINTS_ENU):
            # 모든 웨이포인트 완료 → 착륙 단계
            self.get_logger().info('[WAYPOINT → LAND] 전체 웨이포인트 완료. 착륙 시작.')
            with self._lock:
                self._state_idx = 4  # LAND
            return

        tx, ty, tz = WAYPOINTS_ENU[wp_idx]

        # setpoint 발행
        self._publish_position_setpoint(tx, ty, tz)

        # 도달 확인
        dist = math.sqrt(
            (pos[0] - tx) ** 2
            + (pos[1] - ty) ** 2
            + (pos[2] - tz) ** 2
        )

        if dist < WAYPOINT_TOL_M:
            self.get_logger().info(
                f'[WAYPOINT {wp_idx + 1}/{len(WAYPOINTS_ENU)}] '
                f'도달! ({tx},{ty},{tz}) 오차={dist:.2f}m < {WAYPOINT_TOL_M}m')
            with self._lock:
                self._wp_reached[wp_idx] = True
                self._wp_index += 1
        else:
            # 5초마다 진행 상황 로그 (10Hz 루프에서 과도한 출력 방지)
            elapsed_count = int((time.monotonic() - self._start_wall) * OFFBOARD_RATE_HZ)
            if elapsed_count % 50 == 0:
                self.get_logger().info(
                    f'[WAYPOINT {wp_idx + 1}/{len(WAYPOINTS_ENU)}] '
                    f'목표=({tx},{ty},{tz}) 현재=({pos[0]:.1f},{pos[1]:.1f},{pos[2]:.1f}) '
                    f'거리={dist:.2f}m')

    def _do_land(self, pos) -> None:
        """LAND: 현재 XY 위치에서 착륙 (z=0)."""
        if pos is None:
            return

        # 착륙 setpoint: 현재 XY 유지, z=0
        self._publish_position_setpoint(pos[0], pos[1], 0.0)

        if pos[2] < LAND_Z_THRESHOLD_M:
            self.get_logger().info(
                f'[LAND] 착륙 완료! z={pos[2]:.2f}m (기준 <{LAND_Z_THRESHOLD_M}m)')
            with self._lock:
                self._state_idx = 5  # DONE
            self._finalize(success=True)

    # ── PX4 커맨드 헬퍼 ───────────────────────────────────────────────────────

    def _publish_offboard_heartbeat(self) -> None:
        """OffboardControlMode heartbeat 발행 (PX4 offboard 유지 필수)."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self._offboard_pub.publish(msg)

    def _publish_position_setpoint(self, x_enu: float, y_enu: float, z_enu: float,
                                   yaw_enu: float = 0.0) -> None:
        """ENU 위치 setpoint 발행 (NED 변환 후 PX4로 전달)."""
        x_ned, y_ned, z_ned = enu_to_ned(x_enu, y_enu, z_enu)
        # ENU yaw → NED yaw
        yaw_ned = math.pi / 2.0 - yaw_enu

        msg = TrajectorySetpoint()
        msg.position = [x_ned, y_ned, z_ned]
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        msg.yaw = yaw_ned
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self._setpoint_pub.publish(msg)

    def _publish_vehicle_command(self, command: int, param1: float = 0.0,
                                 param2: float = 0.0) -> None:
        """VehicleCommand 발행."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self._cmd_pub.publish(msg)

    # ── 결과 처리 ──────────────────────────────────────────────────────────────

    def _finalize(self, success: bool) -> None:
        """검증 완료 처리 — CSV 저장 + 최종 보고."""
        elapsed = time.monotonic() - self._start_wall

        with self._lock:
            wp_reached = list(self._wp_reached)
            pos = self._pos_enu
            trajectory = list(self._trajectory)

        all_wp_reached = all(wp_reached)
        landed_ok = pos is not None and pos[2] < LAND_Z_THRESHOLD_M
        within_timeout = elapsed <= self._timeout_sec

        passed = success and all_wp_reached and landed_ok and within_timeout
        self._result = 'PASS' if passed else 'FAIL'

        # CSV 저장
        self._save_csv(trajectory)

        # 최종 보고
        print()
        print('=' * 60)
        print(f'  [최종 결과] {self._result}  ({elapsed:.1f}s / {self._timeout_sec:.0f}s)')
        print('=' * 60)
        print(f'  웨이포인트 도달:')
        for i, (wp, reached) in enumerate(zip(WAYPOINTS_ENU, wp_reached)):
            sym = 'O' if reached else 'X'
            print(f'    [{sym}] WP{i + 1}: {wp}')
        landed_str = f'z={pos[2]:.2f}m' if pos else '위치 미수신'
        print(f'  착륙 확인:    [{"O" if landed_ok else "X"}] {landed_str} (기준 <{LAND_Z_THRESHOLD_M}m)')
        print(f'  시간 이내:    [{"O" if within_timeout else "X"}] {elapsed:.1f}s / {self._timeout_sec:.0f}s')
        print(f'  궤적 저장:    {self._csv_path} ({len(trajectory)}점)')
        print('=' * 60)

        # 셧다운
        rclpy.shutdown()

    def _fail(self, reason: str) -> None:
        """실패 처리."""
        self.get_logger().error(f'[FAIL] {reason}')
        self._result = 'FAIL'
        elapsed = time.monotonic() - self._start_wall

        with self._lock:
            trajectory = list(self._trajectory)
        self._save_csv(trajectory)

        print()
        print('=' * 60)
        print(f'  [최종 결과] FAIL  ({elapsed:.1f}s)')
        print(f'  사유: {reason}')
        print('=' * 60)

        rclpy.shutdown()

    def _on_timeout(self) -> None:
        """타임아웃 처리."""
        self._timeout_timer.cancel()
        with self._lock:
            state = STATES[self._state_idx]
            wp_reached = list(self._wp_reached)
            pos = self._pos_enu
        self.get_logger().error(
            f'[TIMEOUT] {self._timeout_sec:.0f}s 초과 — 현재 단계: {state}')
        self._result = 'TIMEOUT'
        elapsed = time.monotonic() - self._start_wall

        with self._lock:
            trajectory = list(self._trajectory)
        self._save_csv(trajectory)

        print()
        print('=' * 60)
        print(f'  [최종 결과] TIMEOUT  ({elapsed:.1f}s / {self._timeout_sec:.0f}s)')
        print(f'  마지막 단계: {state}')
        print(f'  웨이포인트 도달: {sum(wp_reached)}/{len(wp_reached)}점')
        if pos:
            print(f'  마지막 위치: ENU=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})')
        print(f'  궤적 저장: {self._csv_path} ({len(trajectory)}점)')
        print('=' * 60)

        rclpy.shutdown()

    # ── 상태 보고 (5초 주기) ──────────────────────────────────────────────────

    def _report(self) -> None:
        """5초 주기 진행 상황 출력."""
        elapsed = time.monotonic() - self._start_wall
        with self._lock:
            state = STATES[self._state_idx]
            pos = self._pos_enu
            armed = self._armed
            wp_idx = self._wp_index
            wp_reached = list(self._wp_reached)

        pos_str = (f'({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f})'
                   if pos else '미수신')
        print(f'[{elapsed:6.1f}s] 단계={state:<10} 위치={pos_str} '
              f'armed={armed} WP={wp_idx}/{len(WAYPOINTS_ENU)} '
              f'도달={sum(wp_reached)}점')

    # ── CSV 저장 ───────────────────────────────────────────────────────────────

    def _save_csv(self, trajectory: list) -> None:
        """궤적 데이터를 CSV로 저장."""
        try:
            csv_file = Path(self._csv_path)
            csv_file.parent.mkdir(parents=True, exist_ok=True)
            with open(csv_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp_sec', 'x_enu_m', 'y_enu_m', 'z_enu_m'])
                writer.writerows(trajectory)
            self.get_logger().info(f'궤적 CSV 저장 완료: {csv_file} ({len(trajectory)}행)')
        except Exception as e:
            self.get_logger().error(f'CSV 저장 실패: {e}')


# ── 진입점 ─────────────────────────────────────────────────────────────────────

def parse_args():
    parser = argparse.ArgumentParser(
        description='ARGOS PX4 Offboard 비행 검증 스크립트',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        '--timeout', type=float, default=300.0,
        help='최대 허용 시간(초). 기본: 300')
    parser.add_argument(
        '--ns', type=str, default='',
        help='PX4 FMU 네임스페이스 접두사 (예: /px4_1). 기본: 빈 문자열(단일 드론)')
    parser.add_argument(
        '--csv', type=str, default='/tmp/argos_px4_offboard_trace.csv',
        help='궤적 CSV 저장 경로. 기본: /tmp/argos_px4_offboard_trace.csv')
    return parser.parse_args()


def print_header(args):
    print('=' * 60)
    print('  ARGOS PX4 Offboard 비행 검증')
    print('=' * 60)
    print(f'  타임아웃: {args.timeout:.0f}s')
    print(f'  FMU NS:  {args.ns!r}')
    print(f'  CSV:     {args.csv}')
    print(f'  px4_msgs: {"설치됨" if PX4_MSGS_AVAILABLE else "미설치 (위치 모니터링만 가능)"}')
    print()
    print('  사전 조건:')
    print('    1. PX4 SITL 기동 (gz sim + PX4)')
    print('    2. MicroXRCEAgent udp4 -p 8888')
    print('    3. px4_bridge_node (use_px4:=true)')
    print()
    print(f'  웨이포인트 ({len(WAYPOINTS_ENU)}점, ENU 좌표):')
    for i, wp in enumerate(WAYPOINTS_ENU):
        print(f'    WP{i + 1}: {wp}')
    print()
    print('  판정 기준:')
    print(f'    - 웨이포인트 4점 모두 {WAYPOINT_TOL_M}m 이내 도달')
    print(f'    - 착륙 후 z < {LAND_Z_THRESHOLD_M}m')
    print(f'    - 전체 소요 시간 <= 타임아웃')
    print('=' * 60)
    print()


def main():
    args = parse_args()
    print_header(args)

    if not PX4_MSGS_AVAILABLE:
        print('[경고] px4_msgs 패키지가 설치되지 않았습니다.')
        print('  ARM/Offboard 커맨드 발행 불가 — 위치 모니터링만 수행합니다.')
        print('  설치 방법:')
        print('    cd ~/ros2_ws/src')
        print('    git clone https://github.com/PX4/px4_msgs.git')
        print('    cd ~/ros2_ws && colcon build --packages-select px4_msgs')
        print()

    rclpy.init()
    node = PX4OffboardVerifier(
        fmu_ns=args.ns,
        csv_path=args.csv,
        timeout_sec=args.timeout,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n[중단] Ctrl+C 감지.')
        with node._lock:
            trajectory = list(node._trajectory)
        node._save_csv(trajectory)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    result = node._result or 'INTERRUPTED'
    print(f'\n결과: {result}')
    sys.exit(0 if result == 'PASS' else 1)


if __name__ == '__main__':
    main()
