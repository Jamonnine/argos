# Copyright 2024 민발 (Minbal), 대구강북소방서
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""로봇 디스패처 모듈 — CBBA + 호스 + heartbeat + 파견.

G-1 SRP 리팩토링 Phase 3: orchestrator_node.py에서 분리.
ROS 미의존 순수 로직 — rclpy import 없음.
"""

import math

from argos_bringup.cbba_allocator import (
    CBBAAllocator,
    RobotRecord as CbbaRobotRecord,
    Task as CbbaTask,
)
from argos_bringup.orchestrator_types import (
    RobotRecord,
    FIRE_TACTICS,
    CONSENSUS_RADIUS,  # noqa: F401 — 외부 참조용 재노출
    COLLISION_SAFE_DISTANCE,  # noqa: F401
    StopRobotCommand,
    WaypointCommand,
    AutonomyModeCommand,
    HoseConflictEvent,
    DispatchRescueCommand,
    FireResponseRequest,  # noqa: F401
)
from argos_bringup.validation_utils import validate_robot_id, validate_timestamp


class RobotDispatcher:
    """CBBA 기반 로봇 파견 + 호스 제약 + heartbeat 관리.

    ROS 미의존 순수 로직 모듈.
    모든 ROS 사이드이펙트(발행·구독)는 명령 객체로 반환하여
    OrchestratorNode._execute_actions()에서 처리.
    """

    def __init__(self, logger, clock_fn, sensor_fusion):
        """초기화.

        Args:
            logger: .info()/.warn()/.error() 를 가진 로거 객체
            clock_fn: callable() -> float (현재 시각, 초 단위)
            sensor_fusion: SensorFusion 인스턴스 (fire_alerts, victims_detected 접근용)
        """
        self._log = logger
        self._clock_fn = clock_fn
        self._sensor_fusion = sensor_fusion
        self.allocator = None
        self._last_cbba_assignments = {}
        # CBBA 주기 재할당 시 severity 캐시
        self._current_fire_severity = 'medium'

    # ─────────────────── Group A: 로봇 관리 ───────────────────

    def robot_status_callback(self, msg, robots: dict, lock) -> list:
        """로봇 상태 수신 → 레지스트리 갱신 + 패킷 손실 감지.

        Args:
            msg: RobotStatus 메시지
            robots: 로봇 레코드 딕셔너리 {robot_id: RobotRecord}
            lock: threading.Lock 인스턴스

        Returns:
            list[command]: AutonomyModeCommand(재연결) + StopRobotCommand(배터리 위급)
        """
        from argos_interfaces.msg import RobotStatus  # 지연 임포트로 테스트 분리 유지
        rid = msg.robot_id

        # 입력 검증
        if not validate_robot_id(rid):
            self._log.error(
                f'Invalid robot_id rejected: "{rid}"')
            return []

        now = self._clock_fn()
        commands = []
        _reconnected = False

        with lock:
            if rid not in robots:
                robots[rid] = RobotRecord(rid)
                self._log.info(f'New robot registered: {rid}')

            r = robots[rid]
            r.robot_type = msg.robot_type
            r.state = msg.state
            r.last_seen = now
            r.battery = msg.battery_percent
            r.current_mission = msg.current_mission
            r.mission_progress = msg.mission_progress
            r.frontiers_remaining = msg.frontiers_remaining
            r.coverage = msg.coverage_percent
            r.capabilities = list(msg.capabilities)
            r.pose = msg.pose

            # 패킷 손실 감지 (seq_number 기반)
            msg_seq = getattr(msg, 'seq_number', -1)
            if msg_seq >= 0 and r.last_seq >= 0:
                expected_seq = r.last_seq + 1
                if msg_seq != expected_seq:
                    lost = abs(msg_seq - expected_seq)
                    r.packet_loss_count += lost
                    self._log.warn(
                        f'PACKET LOSS [{rid}]: expected seq={expected_seq}, '
                        f'got={msg_seq}, lost={lost}, '
                        f'total_lost={r.packet_loss_count}')
            if msg_seq >= 0:
                r.last_seq = msg_seq

            # 재연결 감지
            if r.comm_lost:
                r.comm_lost = False
                _reconnected = True

        # 재연결 시 CENTRALIZED 복귀 명령
        if _reconnected:
            self._log.info(f'Robot {rid} reconnected — resuming CENTRALIZED')
            commands.append(AutonomyModeCommand(robot_id=rid, mode='CENTRALIZED'))

        # 배터리 체크 (lock 밖에서 — r은 참조로 안전)
        with lock:
            r_ref = robots.get(rid)
        if r_ref is not None:
            battery_cmds = self.check_battery(
                rid, r_ref,
                battery_warning=30.0,
                battery_critical=15.0,
            )
            commands.extend(battery_cmds)

        return commands

    def check_battery(
        self,
        rid: str,
        robot_record: RobotRecord,
        battery_warning: float,
        battery_critical: float,
    ) -> list:
        """배터리 수준 체크.

        Args:
            rid: 로봇 ID
            robot_record: 해당 로봇의 RobotRecord
            battery_warning: 경고 임계값 (%)
            battery_critical: 위급 임계값 (%)

        Returns:
            list[StopRobotCommand]: 위급 수준이면 정지 명령 반환
        """
        from argos_interfaces.msg import RobotStatus
        r = robot_record
        commands = []

        if (r.battery <= battery_critical
                and r.state != RobotStatus.STATE_RETURNING
                and not r.battery_critical_acted):
            r.battery_critical_acted = True
            self._log.error(
                f'BATTERY CRITICAL: {rid} at {r.battery:.0f}% — auto-return')
            commands.append(StopRobotCommand(robot_id=rid))
        elif r.battery <= battery_warning and not r.battery_warned:
            r.battery_warned = True
            self._log.warn(f'BATTERY LOW: {rid} at {r.battery:.0f}%')

        # 배터리 회복 시 플래그 리셋
        if r.battery > battery_warning and r.battery_warned:
            r.battery_warned = False
        if r.battery > battery_critical and r.battery_critical_acted:
            r.battery_critical_acted = False

        return commands

    def check_heartbeats(self, robots: dict, lock, heartbeat_timeout: float) -> list:
        """주기적 heartbeat 점검: 타임아웃 시 통신 두절 판정 + 자율 모드 전환.

        Args:
            robots: 로봇 레코드 딕셔너리
            lock: threading.Lock
            heartbeat_timeout: 타임아웃 기준 (초)

        Returns:
            list[AutonomyModeCommand]: 두절 감지 시 LOCAL_AUTONOMY 명령
        """
        from argos_interfaces.msg import RobotStatus
        now = self._clock_fn()
        commands = []
        lost_count = 0

        with lock:
            snapshot = dict(robots)

        for rid, r in snapshot.items():
            if r.last_seen == 0.0:
                continue

            elapsed = now - r.last_seen
            if elapsed > heartbeat_timeout and not r.comm_lost:
                r.comm_lost = True
                r.state = RobotStatus.STATE_COMM_LOST
                self._log.warn(
                    f'Robot {rid} comm lost — switching to LOCAL_AUTONOMY')
                commands.append(AutonomyModeCommand(robot_id=rid, mode='LOCAL_AUTONOMY'))

            if r.comm_lost:
                lost_count += 1

        active_count = sum(1 for r in snapshot.values() if r.last_seen > 0)
        if active_count > 0 and lost_count > active_count / 2:
            self._log.error(
                f'CRITICAL: {lost_count}/{active_count} robots comm lost')

        return commands

    # ─────────────────── Group B: 호스 제약 ───────────────────

    def hose_status_callback(
        self, robot_id: str, msg, robots: dict, lock
    ) -> list:
        """셰르파 로봇의 호스 상태 수신 → RobotRecord 갱신.

        Float32MultiArray 레이아웃:
          [0] hose_remaining_m  [1] hose_kink_risk
          [2] hose_charged      [3..] 경유점 x,y 쌍

        Returns:
            list: 현재 구현에서는 빈 리스트 (경고 로그만 출력)
        """
        if len(msg.data) < 3:
            self._log.warn(
                f'[호스] {robot_id}: 데이터 길이 부족 ({len(msg.data)}개) — 무시')
            return []

        with lock:
            if robot_id not in robots:
                robots[robot_id] = RobotRecord(robot_id)
            r = robots[robot_id]
            r.hose_remaining_m = float(msg.data[0])
            r.hose_kink_risk = float(msg.data[1])
            r.hose_charged = bool(msg.data[2] > 0.5)

            path_data = msg.data[3:]
            r.hose_path = [
                (float(path_data[i]), float(path_data[i + 1]))
                for i in range(0, len(path_data) - 1, 2)
            ]

        if r.hose_kink_risk > 0.7:
            self._log.warn(
                f'[호스] {robot_id}: 꺾임 위험 높음 ({r.hose_kink_risk:.2f}) '
                f'— 경로 조정 권고')

        return []

    def check_hose_depth(
        self,
        robot_id: str,
        target_x: float,
        target_y: float,
        robots: dict,
        lock,
    ) -> bool:
        """진입 깊이와 남은 호스 길이를 비교해 진입 가능 여부 판단.

        Args:
            robot_id: 대상 로봇 ID
            target_x, target_y: 목표 위치 (맵 좌표)
            robots: 로봇 레코드 딕셔너리
            lock: threading.Lock

        Returns:
            bool: True = 진입 가능 / False = 호스 부족으로 진입 불가
        """
        with lock:
            r = robots.get(robot_id)

        if r is None or r.hose_remaining_m < 0:
            return True

        if r.pose is None:
            return True

        rx = r.pose.pose.position.x
        ry = r.pose.pose.position.y
        required_depth = math.hypot(target_x - rx, target_y - ry)

        if required_depth > r.hose_remaining_m:
            self._log.warn(
                f'[호스] {robot_id}: 진입 깊이 {required_depth:.1f}m > '
                f'남은 호스 {r.hose_remaining_m:.1f}m — 진입 불가')
            return False

        return True

    def detect_hose_conflict(self, rid_a: str, rid_b: str, robots: dict) -> bool:
        """두 로봇의 호스 경로가 교차하는지 확인 (선분 교차 알고리즘).

        교차 감지 시 rid_b의 assigned_target 초기화(재할당 트리거).

        Args:
            rid_a, rid_b: 비교할 두 로봇 ID
            robots: 로봇 레코드 딕셔너리 (직접 접근, lock 없음 — 호출자가 관리)

        Returns:
            bool: True = 교차 충돌 감지됨
        """
        ra = robots.get(rid_a)
        rb = robots.get(rid_b)

        if ra is None or rb is None:
            return False
        if len(ra.hose_path) < 2 or len(rb.hose_path) < 2:
            return False

        def _cross(o, a, b):
            """외적으로 방향 판별."""
            return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

        def _on_segment(p, q, r_pt):
            """점 r_pt가 선분 pq 위에 있는지 확인."""
            return (min(p[0], q[0]) <= r_pt[0] <= max(p[0], q[0]) and
                    min(p[1], q[1]) <= r_pt[1] <= max(p[1], q[1]))

        def _segments_intersect(p1, p2, p3, p4):
            """두 선분 (p1-p2)와 (p3-p4)의 교차 여부."""
            d1 = _cross(p3, p4, p1)
            d2 = _cross(p3, p4, p2)
            d3 = _cross(p1, p2, p3)
            d4 = _cross(p1, p2, p4)

            if ((d1 > 0 < d2) or (d1 < 0 > d2)) and \
               ((d3 > 0 < d4) or (d3 < 0 > d4)):
                return True
            if d1 == 0 and _on_segment(p3, p4, p1):
                return True
            if d2 == 0 and _on_segment(p3, p4, p2):
                return True
            if d3 == 0 and _on_segment(p1, p2, p3):
                return True
            if d4 == 0 and _on_segment(p1, p2, p4):
                return True
            return False

        path_a = ra.hose_path
        path_b = rb.hose_path
        for i in range(len(path_a) - 1):
            for j in range(len(path_b) - 1):
                if _segments_intersect(path_a[i], path_a[i + 1],
                                       path_b[j], path_b[j + 1]):
                    self._log.warn(
                        f'[호스] 경로 충돌: {rid_a} ↔ {rid_b} — {rid_b} 재할당 필요')
                    # 충돌한 로봇(rid_b)의 목표 초기화 → 재할당 트리거
                    if rid_b in robots:
                        robots[rid_b].assigned_target = None
                        robots[rid_b].mission_lock = None
                    return True

        return False

    def periodic_hose_check(self, robots: dict, lock) -> list:
        """모든 셰르파 쌍의 호스 경로 교차 검사.

        hose_remaining_m >= 0인 로봇(셰르파)끼리만 교차 검사.
        교차 감지 시 resolve_hose_conflict로 역할 재분리.

        Returns:
            list[command]: resolve_hose_conflict 결과 병합
        """
        with lock:
            sherpa_ids = [
                rid for rid, rec in robots.items()
                if rec.hose_remaining_m >= 0
            ]
            snapshot = dict(robots)

        commands = []
        for i, rid_a in enumerate(sherpa_ids):
            for rid_b in sherpa_ids[i + 1:]:
                if self.detect_hose_conflict(rid_a, rid_b, snapshot):
                    cmds = self.resolve_hose_conflict(rid_a, rid_b, robots, lock)
                    commands.extend(cmds)
        return commands

    def resolve_hose_conflict(
        self, rid_a: str, rid_b: str, robots: dict, lock
    ) -> list:
        """호스 교차 시 역할 재분리.

        호스를 더 많이 남긴(덜 풀린) 로봇을 공급 전담으로 전환.

        Returns:
            list[command]: HoseConflictEvent + StopRobotCommand + WaypointCommand
        """
        with lock:
            rec_a = robots.get(rid_a)
            rec_b = robots.get(rid_b)

        if rec_a is None or rec_b is None:
            return []

        if rec_a.hose_remaining_m > rec_b.hose_remaining_m:
            supply_robot, attack_robot = rid_a, rid_b
        else:
            supply_robot, attack_robot = rid_b, rid_a

        self._log.warn(
            f'[호스충돌] {rid_a} <-> {rid_b} 교차 감지. '
            f'{supply_robot}을 hose_supply 전담으로 전환, '
            f'{attack_robot}을 fire_attack으로 유지')

        # 역할 갱신
        with lock:
            if supply_robot in robots:
                robots[supply_robot].role = 'hose_supply'
            if attack_robot in robots:
                robots[attack_robot].role = 'fire_attack'

        commands = []

        # 호스 충돌 이벤트
        commands.append(HoseConflictEvent(
            rid_a=rid_a,
            rid_b=rid_b,
            resolution=f'{supply_robot}→hose_supply, {attack_robot}→fire_attack',
        ))

        # 공급 로봇 정지
        commands.extend(self.cancel_goal(supply_robot))

        # 진압 로봇 경로 재계획
        commands.extend(self.replan_avoiding_hose(attack_robot, supply_robot, robots))

        return commands

    def replan_avoiding_hose(
        self, robot_id: str, supply_robot_id: str, robots: dict
    ) -> list:
        """호스 교차 회피 경로 재계획.

        교차점 반대 방향으로 1m 이동 후 기존 임무 재탐색.

        Args:
            robot_id: 경로 재계획 대상 진압 로봇 ID
            supply_robot_id: 공급 로봇 ID (회피 기준점)
            robots: 로봇 레코드 딕셔너리

        Returns:
            list[WaypointCommand]
        """
        r_attack = robots.get(robot_id)
        r_supply = robots.get(supply_robot_id)

        if r_attack is None or r_attack.pose is None:
            self._log.warn(f'[호스충돌] {robot_id} 위치 미확인 — 재계획 스킵')
            return []

        ax = r_attack.pose.pose.position.x
        ay = r_attack.pose.pose.position.y

        if r_supply is not None and r_supply.pose is not None:
            sx = r_supply.pose.pose.position.x
            sy = r_supply.pose.pose.position.y
            dx = ax - sx
            dy = ay - sy
            dist = math.hypot(dx, dy)
            if dist > 1e-6:
                nx = ax + (dx / dist) * 1.0
                ny = ay + (dy / dist) * 1.0
            else:
                nx, ny = ax + 1.0, ay
        else:
            nx, ny = ax + 1.0, ay

        self._log.warn(
            f'[호스충돌] {robot_id} 재경로: ({ax:.1f},{ay:.1f}) → '
            f'({nx:.1f},{ny:.1f}) (교차 방향 1m 회피)')

        return [WaypointCommand(robot_id=robot_id, x=nx, y=ny, z=0.0, frame_id='map')]

    # ─────────────────── Group C: CBBA + 화재 대응 ───────────────────

    def init_cbba(self) -> None:
        """CBBA 할당기 초기화. OrchestratorNode.on_activate()에서 호출."""
        self.allocator = CBBAAllocator(self._log)
        self._last_cbba_assignments = {}
        self._log.info('CBBA allocator initialized (realloc every 10s)')

    def convert_robots_to_cbba(self, robots: dict, lock) -> list:
        """오케스트레이터 RobotRecord → CBBA RobotRecord 변환.

        comm_lost 로봇, hose_supply 전담 로봇은 제외.

        Returns:
            list[CbbaRobotRecord]
        """
        with lock:
            snapshot = dict(robots)

        cbba_robots = []
        for rid, r in snapshot.items():
            if r.comm_lost:
                continue
            if r.role == 'hose_supply':
                continue

            pose_tuple = None
            if r.pose is not None:
                try:
                    p = r.pose.pose.position
                    pose_tuple = (p.x, p.y, p.z)
                except AttributeError:
                    pose_tuple = None

            cbba_robots.append(CbbaRobotRecord(
                robot_id=rid,
                capabilities=list(r.capabilities),
                pose=pose_tuple,
            ))

        return cbba_robots

    def build_task_list(
        self, fire_pt, severity: str, robots: dict, lock
    ) -> list:
        """현재 상황을 CBBA Task 목록으로 변환.

        소방 시나리오 임무 타입:
          inspect_fire / monitor / explore / rescue

        Args:
            fire_pt: geometry_msgs.msg.Point 또는 None
            severity: 'low' | 'medium' | 'high' | 'critical'
            robots: 로봇 레코드 딕셔너리
            lock: threading.Lock

        Returns:
            list[CbbaTask]
        """
        tasks = []
        severity_to_priority = {
            'low': 0.3, 'medium': 0.5, 'high': 0.8, 'critical': 1.0
        }

        # 1) 화재 대응 임무 (최우선)
        if fire_pt is not None:
            fire_priority = severity_to_priority.get(severity, 0.5)

            tasks.append(CbbaTask(
                task_id='fire_suppress_ground',
                location=(fire_pt.x, fire_pt.y, 0.0),
                task_type='inspect_fire',
                required_capabilities=['has_thermal'],
                priority=fire_priority,
            ))
            tasks.append(CbbaTask(
                task_id='fire_monitor_aerial',
                location=(fire_pt.x, fire_pt.y, 8.0),
                task_type='monitor',
                required_capabilities=['can_fly'],
                priority=fire_priority * 0.9,
            ))

        # 2) 추가 화점 (fire_alerts에서 미처리 화점 추출)
        if self._sensor_fusion.fire_alerts:
            processed_locations = set()
            if fire_pt:
                processed_locations.add((round(fire_pt.x, 1), round(fire_pt.y, 1)))

            for i, alert in enumerate(self._sensor_fusion.fire_alerts[-5:]):
                loc = alert.location.point
                loc_key = (round(loc.x, 1), round(loc.y, 1))
                if loc_key in processed_locations:
                    continue
                processed_locations.add(loc_key)

                alert_priority = severity_to_priority.get(
                    getattr(alert, 'severity', 'medium'), 0.5)
                tasks.append(CbbaTask(
                    task_id=f'fire_alert_{i}',
                    location=(loc.x, loc.y, 0.0),
                    task_type='inspect_fire',
                    required_capabilities=['has_thermal'],
                    priority=alert_priority * 0.8,
                ))

        # 3) 탐색 임무 (잔여 프론티어가 있는 경우)
        with lock:
            snapshot = dict(robots)

        total_frontiers = sum(r.frontiers_remaining for r in snapshot.values())
        if total_frontiers > 0 and fire_pt is None:
            for i, (rid, r) in enumerate(snapshot.items()):
                if r.comm_lost or r.frontiers_remaining == 0:
                    continue
                tasks.append(CbbaTask(
                    task_id=f'explore_{rid}',
                    location=(0.0, 0.0, 0.0),
                    task_type='explore',
                    required_capabilities=[],
                    priority=0.3,
                ))

        # 4) 구조 임무 (피해자 감지 시)
        if self._sensor_fusion.victims_detected:
            for i, victim in enumerate(self._sensor_fusion.victims_detected[-3:]):
                v_pt = getattr(victim, 'location', None)
                if v_pt is None:
                    continue
                tasks.append(CbbaTask(
                    task_id=f'rescue_{i}',
                    location=(v_pt.point.x, v_pt.point.y, 0.0),
                    task_type='rescue',
                    required_capabilities=[],
                    priority=0.9,
                ))

        return tasks

    def apply_cbba_assignments(
        self, assignments: dict, fire_pt, robots: dict, lock
    ) -> list:
        """CBBA 할당 결과를 명령 객체로 변환.

        드론 웨이포인트 발행, mission_lock·assigned_target 갱신,
        primary_responder 결정을 명령 객체로 반환.

        Args:
            assignments: {robot_id: CbbaTask}
            fire_pt: geometry_msgs.msg.Point 또는 None
            robots: 로봇 레코드 딕셔너리
            lock: threading.Lock

        Returns:
            list[command]: WaypointCommand + DispatchRescueCommand
        """
        from geometry_msgs.msg import Point
        commands = []

        with lock:
            for rid, task in assignments.items():
                if rid not in robots:
                    continue
                r = robots[rid]

                # 호스 길이 제약 사후 검증
                if r.hose_remaining_m >= 0:
                    target_x, target_y = task.location[0], task.location[1]
                    if not self.check_hose_depth(rid, target_x, target_y, robots, lock):
                        self._log.warn(
                            f'CBBA 할당 거부: {rid} → {task.task_id} — 호스 부족')
                        continue

                # mission_lock 갱신
                if task.task_type in ('inspect_fire', 'rescue'):
                    r.mission_lock = (
                        'fire_response' if 'fire' in task.task_type else 'rescue')
                    r.assigned_target = Point(
                        x=task.location[0], y=task.location[1], z=task.location[2])

        # 드론 웨이포인트 명령 (lock 밖에서)
        for rid, task in assignments.items():
            if task.task_type == 'monitor':
                commands.append(WaypointCommand(
                    robot_id=rid,
                    x=task.location[0],
                    y=task.location[1],
                    z=task.location[2],
                    frame_id='map',
                ))
                self._log.warn(
                    f'CBBA DRONE {rid} → ({task.location[0]:.1f}, '
                    f'{task.location[1]:.1f}, {task.location[2]:.1f})')

            # 지상 진압 임무 → primary_responder 결정 (DispatchRescueCommand 재사용)
            if task.task_id == 'fire_suppress_ground' and fire_pt is not None:
                commands.append(DispatchRescueCommand(
                    robot_id=rid,
                    target_x=fire_pt.x,
                    target_y=fire_pt.y,
                    target_z=0.0,
                ))
                self._log.warn(f'CBBA PRIMARY RESPONDER: {rid} → {task.task_id}')

        self._last_cbba_assignments = dict(assignments)
        self._log.info(
            f'CBBA 할당 완료: {len(assignments)}대 배정 '
            f'({", ".join(f"{k}→{v.task_id}" for k, v in assignments.items())})')

        return commands

    def execute_fire_response_cbba(
        self, fire_pt, severity: str, robots: dict, lock, stage
    ) -> list:
        """CBBA 기반 화재 대응.

        소방 전술 체크 + CBBA 경매 할당 + 심각도 대응.
        CBBA 실패 시 execute_fire_response로 폴백.

        Args:
            fire_pt: geometry_msgs.msg.Point
            severity: 화재 심각도
            robots: 로봇 레코드 딕셔너리
            lock: threading.Lock
            stage: 현재 임무 단계 (로그용)

        Returns:
            list[command]
        """
        self._current_fire_severity = severity

        # 소방 전술 체크
        fire_type = getattr(
            self._sensor_fusion.fire_alerts[-1]
            if self._sensor_fusion.fire_alerts else None,
            'fire_type', 'general') or 'general'
        tactics = FIRE_TACTICS.get(fire_type, FIRE_TACTICS['general'])

        if not tactics['robot_capable']:
            self._log.error(
                f'[TTS] 화재 유형 "{fire_type}" — 로봇 자율 대응 불가. '
                f'지휘관 즉시 현장 확인 요망.')
        if tactics['human_mandatory']:
            self._log.error(
                f'[TTS] 사람만 수행 가능한 작업: {tactics["human_mandatory"]} — '
                f'지휘관에게 전달.')

        # CBBA 할당
        cbba_robots = self.convert_robots_to_cbba(robots, lock)
        tasks = self.build_task_list(fire_pt=fire_pt, severity=severity,
                                     robots=robots, lock=lock)

        if not cbba_robots or not tasks:
            self._log.warn('CBBA: 가용 로봇 또는 임무 없음 — fallback to nearest')
            return self.execute_fire_response(fire_pt, severity, robots, lock)

        assignments = self.allocator.allocate(cbba_robots, tasks)
        commands = self.apply_cbba_assignments(
            assignments, fire_pt=fire_pt, robots=robots, lock=lock)

        if severity == 'critical':
            self._log.error('[TTS] 긴급: 화재 CRITICAL 판정 — 즉시 진압 필요.')

        # 화재 확산 추정
        spread_info = self._sensor_fusion.estimate_fire_spread()
        if spread_info['trend'] == 'growing' and spread_info['spread_rate'] > 10.0:
            self._log.error(
                f'[TTS] 화재 급속 확산: {spread_info["spread_rate"]:.1f}K/s')

        return commands

    def execute_fire_response(
        self, fire_pt, severity: str, robots: dict, lock
    ) -> list:
        """화재 대응 전술 실행 (폴백): 최근접 UGV 배정 + 드론 화점 상공 배치.

        Args:
            fire_pt: geometry_msgs.msg.Point
            severity: 화재 심각도
            robots: 로봇 레코드 딕셔너리
            lock: threading.Lock

        Returns:
            list[command]: DispatchRescueCommand + WaypointCommand
        """
        self._current_fire_severity = severity

        fire_type = getattr(
            self._sensor_fusion.fire_alerts[-1]
            if self._sensor_fusion.fire_alerts else None,
            'fire_type', 'general') or 'general'
        tactics = FIRE_TACTICS.get(fire_type, FIRE_TACTICS['general'])

        if not tactics['robot_capable']:
            self._log.error(
                f'[TTS] 화재 유형 "{fire_type}" — 로봇 자율 대응 불가. '
                f'지휘관 즉시 현장 확인 요망.')
        if tactics['human_mandatory']:
            self._log.error(
                f'[TTS] 사람만 수행 가능한 작업: {tactics["human_mandatory"]} — '
                f'지휘관에게 전달.')

        with lock:
            snapshot = dict(robots)

        commands = []

        # 1) 최근접 UGV 선정
        best_ugv = None
        best_dist = float('inf')
        for rid, r in snapshot.items():
            if r.robot_type != 'ugv' or r.comm_lost or r.pose is None:
                continue
            if r.role == 'hose_supply':
                continue
            dx = r.pose.pose.position.x - fire_pt.x
            dy = r.pose.pose.position.y - fire_pt.y
            dist = math.hypot(dx, dy)
            if r.role == 'fire_attack':
                dist *= 0.5
            if dist < best_dist:
                best_dist = dist
                best_ugv = rid

        if best_ugv:
            # 충돌 방지 체크
            if self._sensor_fusion.check_collision_risk(
                    fire_pt, best_ugv, snapshot):
                self._log.warn(
                    f'Dispatch to {best_ugv} deferred — collision risk at fire point')
            elif not self.check_hose_depth(
                    best_ugv, fire_pt.x, fire_pt.y, robots, lock):
                self._log.warn(
                    f'Dispatch to {best_ugv} deferred — hose length insufficient')
            else:
                self._log.warn(
                    f'PRIMARY RESPONDER: {best_ugv} (dist={best_dist:.1f}m)')
                commands.append(DispatchRescueCommand(
                    robot_id=best_ugv,
                    target_x=fire_pt.x,
                    target_y=fire_pt.y,
                    target_z=0.0,
                ))
        else:
            self._log.warn('No UGV available for fire response')

        # 2) 드론을 화점 상공으로 이동
        for rid, r in snapshot.items():
            if r.robot_type != 'drone' or r.comm_lost:
                continue
            commands.append(WaypointCommand(
                robot_id=rid,
                x=fire_pt.x,
                y=fire_pt.y,
                z=8.0,
                frame_id='map',
            ))
            self._log.warn(
                f'DRONE {rid} → fire location '
                f'({fire_pt.x:.1f}, {fire_pt.y:.1f}, 8.0m)')

        # 3) 심각도별 대응 + TTS
        if severity == 'critical':
            self._log.error(
                '[TTS] 긴급: 화재 CRITICAL 판정 — 즉시 진압 필요. '
                '지휘관 현장 통제 요청.')
        else:
            self._log.warn(f'Fire severity: {severity} — 감시 모드 유지')

        # 4) 화재 확산 추정
        spread_info = self._sensor_fusion.estimate_fire_spread()
        if spread_info['trend'] == 'growing' and spread_info['spread_rate'] > 10.0:
            self._log.error(
                f'[TTS] 화재 급속 확산 감지: {spread_info["spread_rate"]:.1f}K/s — '
                f'추정 반경 {spread_info["estimated_radius_m"]:.1f}m. '
                f'추가 진압 로봇 파견 검토.')

        return commands

    def cbba_periodic_realloc(self, robots: dict, lock, stage) -> list:
        """주기적 CBBA 재할당 (10초 주기).

        실행 조건: FIRE_RESPONSE 단계이고 활성 화재가 있을 때.

        Returns:
            list[command]: 변화 감지 시 apply_cbba_assignments 결과
        """
        from argos_interfaces.msg import MissionState as MS
        if self.allocator is None:
            return []

        if stage != MS.STAGE_FIRE_RESPONSE:
            return []

        fire_target = self._sensor_fusion.fire_response_target
        if fire_target is None:
            return []

        cbba_robots = self.convert_robots_to_cbba(robots, lock)
        tasks = self.build_task_list(
            fire_pt=fire_target,
            severity=self._current_fire_severity,
            robots=robots,
            lock=lock,
        )

        if not cbba_robots or not tasks:
            return []

        new_assignments = self.allocator.allocate(cbba_robots, tasks)

        if self.assignments_changed(new_assignments):
            self._log.info('CBBA 재할당: 상황 변화 감지 → 임무 재분배')
            return self.apply_cbba_assignments(
                new_assignments, fire_pt=fire_target, robots=robots, lock=lock)

        return []

    def assignments_changed(self, new_assignments: dict) -> bool:
        """이전 할당과 비교하여 변화 여부 판단 (순수 로직).

        Args:
            new_assignments: {robot_id: CbbaTask} 새 할당 결과

        Returns:
            bool: 변화가 있으면 True
        """
        if not self._last_cbba_assignments:
            return bool(new_assignments)

        old_map = {k: v.task_id for k, v in self._last_cbba_assignments.items()}
        new_map = {k: v.task_id for k, v in new_assignments.items()}
        return old_map != new_map

    def cancel_goal(self, robot_id: str) -> list:
        """지정 로봇에 정지 명령 발행 (목표 취소).

        Args:
            robot_id: 정지할 로봇 ID

        Returns:
            list[StopRobotCommand]
        """
        self._log.info(f'[호스충돌] {robot_id} 정지 명령 (목표 취소)')
        return [StopRobotCommand(robot_id=robot_id)]
