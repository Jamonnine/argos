# Copyright 2026 Minbal (대구강북소방서)
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

"""센서 퓨전 모듈 — 8중 센싱 콜백 + 위험도 집계.

G-1 SRP 리팩토링 Phase 2: orchestrator_node.py에서 분리.
ROS 미의존 순수 로직 — rclpy import 없음.

사용:
  fusion = SensorFusion(logger, clock_fn)
  actions = fusion.gas_callback(msg, robot_snapshot)
  # actions: list of command objects (StopRobotCommand, etc.)
"""

import math
from collections import deque

from argos_bringup.orchestrator_types import (
    CONSENSUS_RADIUS,
    COLLISION_SAFE_DISTANCE,
    StopRobotCommand,
    DispatchRescueCommand,
    StageTransition,
    FireResponseRequest,
)
from argos_bringup.validation_utils import (
    validate_robot_id,
    validate_severity,
    validate_danger_level,
    validate_timestamp,
)


class SensorFusion:
    """8중 센싱 콜백 + 위험도 집계 — 순수 로직 클래스.

    ROS2 의존성 없음. 모든 외부 부작용은 반환된 명령 객체를 통해 수행.
    robots_snapshot은 메서드 인자로 전달 (저장하지 않음).
    """

    def __init__(self, logger, clock_fn):
        """생성자.

        Args:
            logger: .info() / .warn() / .error() 메서드를 가진 로거 객체
            clock_fn: 현재 시간(float, 초)을 반환하는 callable
        """
        self._log = logger
        self._clock_fn = clock_fn

        # ── 가스 상태 ──
        self.gas_danger_level: str = 'safe'
        self._gas_critical_count: int = 0
        self._gas_critical_threshold: int = 2
        self._last_gas_time: float = None

        # ── 8중 센싱 누적 버퍼 ──
        self.victims_detected: deque = deque(maxlen=20)
        self.blocked_areas: deque = deque(maxlen=10)
        self.audio_alerts: deque = deque(maxlen=10)
        self.fire_alerts: deque = deque(maxlen=50)

        # ── 화재 추적 ──
        self._thermal_history: deque = deque(maxlen=100)
        self._current_fire_severity: str = None
        self.fire_response_target = None  # Point 또는 None
        self.primary_responder: str = None

        # ── 상황 판단 ──
        self._evacuation_recommended: bool = False

    # ─────────────────── Watchdog ───────────────────

    def check_gas_watchdog(self) -> list:
        """가스 센서 watchdog — 5초 이상 수신 없으면 경고 로그.

        Returns:
            list: 항상 빈 리스트 (명령 없음, 로그만 발행)
        """
        if self._last_gas_time is None:
            return []

        elapsed = self._clock_fn() - self._last_gas_time
        if elapsed > 5.0:
            self._log.warn(
                f'GAS SENSOR TIMEOUT: {elapsed:.1f}초 동안 수신 없음 — 센서 연결 확인 필요'
            )
        return []

    # ─────────────────── 가스 센서 ───────────────────

    def gas_callback(self, msg, stage) -> list:
        """가스 위험도 기반 임무 재할당 판정.

        Args:
            msg: GasReading 메시지 (robot_id, danger_level, evacuate_recommended,
                 co_ppm, o2_percent, lel_percent, hazard_types 속성 필요)
            stage: 현재 임무 단계 (MissionState.STAGE_* 상수)

        Returns:
            list: StageTransition 명령 (임계치 초과 시) 또는 빈 리스트
        """
        rid = msg.robot_id
        if not validate_robot_id(rid):
            return []

        if not validate_danger_level(msg.danger_level):
            self._log.warn(f'Invalid gas danger_level: {msg.danger_level}')
            return []

        # 수신 시각 갱신
        self._last_gas_time = self._clock_fn()

        # MissionState 상수 — rclpy 없이 정수 비교를 위해 지역 참조
        # argos_interfaces.msg.MissionState에 정의된 STAGE_* 값과 일치
        # 순환 import 방지를 위해 런타임에 값 비교 (stage는 정수)
        STAGE_RETURNING = 3   # MissionState.STAGE_RETURNING
        STAGE_COMPLETE = 4    # MissionState.STAGE_COMPLETE
        STAGE_PAUSED = 5      # MissionState.STAGE_PAUSED

        if msg.evacuate_recommended and msg.danger_level == 'critical':
            # Temporal smoothing: N회 연속 critical이어야 실제 판정 (오탐 방지)
            self._gas_critical_count += 1
            if self._gas_critical_count >= self._gas_critical_threshold:
                self.gas_danger_level = 'critical'
                self._log.error(
                    f'GAS CRITICAL CONFIRMED ({self._gas_critical_count}회 연속) from {rid}: '
                    f'CO={msg.co_ppm:.0f}ppm O2={msg.o2_percent:.1f}% LEL={msg.lel_percent:.0f}%'
                )
                # 소방 전문가 권고: 자동 철수 대신 PAUSED → 지휘관 승인 후 RETURNING
                if stage not in (STAGE_RETURNING, STAGE_COMPLETE, STAGE_PAUSED):
                    self._evacuation_recommended = True
                    self._log.error(
                        'EVACUATION RECOMMENDED — 지휘관 /orchestrator/resume 승인 시 철수 시작'
                    )
                    return [StageTransition(
                        new_stage=STAGE_PAUSED,
                        reason='gas_critical_confirmed',
                    )]
            else:
                self._log.warn(
                    f'GAS CRITICAL from {rid} ({self._gas_critical_count}/{self._gas_critical_threshold}) '
                    f'— 확인 대기 중'
                )

        elif msg.danger_level == 'danger':
            # H6: danger는 완전 리셋이 아닌 점진적 감소 — 짧은 복귀 후 재상승 방지
            self._gas_critical_count = max(0, self._gas_critical_count - 1)
            self.gas_danger_level = 'danger'
            self._log.warn(
                f'GAS DANGER from {rid}: {msg.hazard_types} — 감시 강화'
            )

        elif msg.danger_level in ('safe', 'caution'):
            self._gas_critical_count = 0
            if self.gas_danger_level in ('danger', 'critical'):
                self.gas_danger_level = msg.danger_level

        return []

    # ─────────────────── 피해자 감지 ───────────────────

    def victim_callback(self, msg, robots_snapshot: dict) -> list:
        """피해자 감지 → 구조 우선순위 판정.

        Args:
            msg: VictimDetection 메시지 (robot_id, header, location, confidence,
                 estimated_status, rescue_priority 속성 필요)
            robots_snapshot: {robot_id: RobotRecord} 딕셔너리 복사본

        Returns:
            list: DispatchRescueCommand 목록 또는 빈 리스트
        """
        rid = msg.robot_id
        if not validate_robot_id(rid):
            return []

        # 타임스탬프 정렬 (센서 퓨전 전문가 권고)
        now_sec = self._clock_fn()
        msg_sec = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        msg_age = now_sec - msg_sec
        if msg_age > 5.0:
            self._log.warn(
                f'Stale victim detection from {rid} ({msg_age:.1f}s old) — skipped'
            )
            return []

        loc = msg.location.point

        self._log.error(
            f'VICTIM DETECTED by {rid}: '
            f'({loc.x:.1f}, {loc.y:.1f}) '
            f'conf={msg.confidence:.2f} '
            f'status={msg.estimated_status} '
            f'priority={msg.rescue_priority}'
        )

        # 중복 방지 (5m 이내 기존 피해자 위치)
        for v in self.victims_detected:
            dist = math.hypot(loc.x - v.x, loc.y - v.y)
            if dist < 5.0:
                return []

        self.victims_detected.append(loc)

        # 우선순위 1(움직임 없음) → 즉시 구조 파견
        if msg.rescue_priority == 1:
            return self.dispatch_rescue(
                target=loc,
                excluding_rid=rid,
                robots_snapshot=robots_snapshot,
                hose_check_fn=lambda r_id, x, y: True,  # 기본: 호스 제약 없음
            )

        return []

    # ─────────────────── 구조물 감지 ───────────────────

    def structural_callback(self, msg) -> list:
        """구조물 손상 → 경로 회피 / 로봇 정지 명령.

        Args:
            msg: StructuralAlert 메시지 (robot_id, severity, alert_type,
                 location, displacement_m, area_blocked, affected_radius_m,
                 recommended_actions 속성 필요)

        Returns:
            list: StopRobotCommand 목록 또는 빈 리스트
        """
        rid = msg.robot_id
        loc = msg.location.point

        self._log.warn(
            f'STRUCTURAL {msg.severity.upper()}: {msg.alert_type} '
            f'from {rid} at ({loc.x:.1f}, {loc.y:.1f}) '
            f'disp={msg.displacement_m:.2f}m'
        )

        if msg.area_blocked:
            self.blocked_areas.append({
                'center': loc,
                'radius': msg.affected_radius_m,
                'type': msg.alert_type,
            })
            self._log.error(
                f'AREA BLOCKED: radius={msg.affected_radius_m:.1f}m — '
                f'{msg.recommended_actions}'
            )

        if msg.severity == 'critical':
            self._log.error(
                f'CRITICAL STRUCTURAL from {rid} — 해당 로봇 즉시 철수 지휘'
            )
            return [StopRobotCommand(robot_id=rid)]

        return []

    # ─────────────────── 음향 이벤트 ───────────────────

    def audio_callback(self, msg, robots_snapshot: dict) -> list:
        """음향 이벤트 → 피해자 탐색 / 폭발 경고.

        Args:
            msg: AudioEvent 메시지 (robot_id, event_type, immediate_response_needed,
                 estimated_location, confidence, intensity_db, danger_level 속성 필요)
            robots_snapshot: {robot_id: RobotRecord} 딕셔너리 복사본

        Returns:
            list: DispatchRescueCommand 목록 또는 빈 리스트
        """
        rid = msg.robot_id
        self.audio_alerts.append(msg)

        if msg.event_type == 'cry_for_help' and msg.immediate_response_needed:
            loc = msg.estimated_location.point
            self._log.error(
                f'CRY FOR HELP detected by {rid} at '
                f'({loc.x:.1f}, {loc.y:.1f}) '
                f'conf={msg.confidence:.2f} — 구조 임무 파견'
            )
            return self.dispatch_rescue(
                target=loc,
                excluding_rid=rid,
                robots_snapshot=robots_snapshot,
                hose_check_fn=lambda r_id, x, y: True,
            )

        elif msg.event_type == 'explosion':
            self._log.error(
                f'EXPLOSION detected by {rid} — '
                f'intensity={msg.intensity_db:.0f}dB — 전원 주의'
            )

        elif msg.event_type == 'gas_leak':
            self._log.warn(
                f'GAS LEAK sound detected by {rid} — '
                f'가스 센서 데이터 교차 확인 필요'
            )

        elif msg.event_type == 'collapse':
            self._log.error(
                f'COLLAPSE sound detected by {rid} — '
                f'구조물 모니터 데이터 교차 확인 필요'
            )

        return []

    # ─────────────────── 구조 파견 ───────────────────

    def dispatch_rescue(self, target, excluding_rid: str,
                        robots_snapshot: dict, hose_check_fn) -> list:
        """최근접 가용 로봇을 구조 대상 위치로 파견.

        Args:
            target: 목표 위치 (x, y 속성을 가진 Point 객체)
            excluding_rid: 파견 제외 로봇 ID (감지 로봇 자신)
            robots_snapshot: {robot_id: RobotRecord} 딕셔너리 복사본
            hose_check_fn: (robot_id, x, y) -> bool — 호스 깊이 검증 함수

        Returns:
            list: DispatchRescueCommand 목록 (최대 2개)
        """
        available = []
        for rid, r in robots_snapshot.items():
            if rid == excluding_rid or r.comm_lost or r.pose is None:
                continue
            if r.mission_lock is not None:
                continue  # 다른 임무 수행 중
            # hose_supply 역할 로봇은 구조 파견 불가
            if getattr(r, 'role', 'explore') == 'hose_supply':
                self._log.info(
                    f'RESCUE SKIP (역할): {rid} — hose_supply 전담 중, 구조 파견 불가'
                )
                continue

            dx = r.pose.pose.position.x - target.x
            dy = r.pose.pose.position.y - target.y
            dist = math.hypot(dx, dy)
            available.append((rid, dist))

        available.sort(key=lambda x: x[1])

        commands = []
        for rid, dist in available[:2]:
            # 호스 길이 제약 체크
            if not hose_check_fn(rid, target.x, target.y):
                self._log.warn(
                    f'RESCUE SKIP: {rid} — 호스 부족으로 구조 파견 불가'
                )
                continue
            commands.append(DispatchRescueCommand(
                robot_id=rid,
                target_x=target.x,
                target_y=target.y,
                target_z=getattr(target, 'z', 0.0),
            ))
            self._log.warn(
                f'RESCUE DISPATCH: {rid} → '
                f'({target.x:.1f}, {target.y:.1f}) dist={dist:.1f}m'
            )

        if not commands:
            self._log.warn('No available robot for rescue dispatch')
        elif len(commands) == 1:
            self._log.warn('Only 1 robot available for rescue (2 recommended)')

        return commands

    # ─────────────────── 화재 경보 ───────────────────

    def fire_alert_callback(self, msg, stage) -> list:
        """화점 감지 알림 → 센서 합의 검증 → 전술적 대응 요청.

        Args:
            msg: FireAlert 메시지 (robot_id, severity, header, location,
                 max_temperature_kelvin 속성 필요)
            stage: 현재 임무 단계 (MissionState.STAGE_* 정수)

        Returns:
            list: StageTransition, FireResponseRequest 명령 목록 또는 빈 리스트
        """
        # 입력 검증
        if not validate_robot_id(msg.robot_id):
            self._log.error(f'Invalid robot_id in FireAlert: "{msg.robot_id}"')
            return []

        if not validate_severity(msg.severity):
            self._log.error(f'Invalid severity in FireAlert: "{msg.severity}"')
            return []

        # 타임스탬프 검증 (클록 스큐 감지)
        now_sec = self._clock_fn()
        msg_sec = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if not validate_timestamp(msg_sec, now_sec):
            self._log.warn(
                f'Suspicious timestamp from {msg.robot_id} '
                f'(skew={abs(now_sec - msg_sec):.1f}s) — ignoring'
            )
            return []

        self.fire_alerts.append(msg)
        self._thermal_history.append((now_sec, msg.max_temperature_kelvin))

        temp_c = msg.max_temperature_kelvin - 273.15
        fire_pt = msg.location.point
        self._log.warn(
            f'FIRE ALERT from {msg.robot_id}: {msg.severity} '
            f'({temp_c:.0f}C) at ({fire_pt.x:.1f}, {fire_pt.y:.1f})'
        )

        # MissionState 상수 (정수 비교)
        STAGE_EXPLORING = 1      # MissionState.STAGE_EXPLORING
        STAGE_FIRE_RESPONSE = 2  # MissionState.STAGE_FIRE_RESPONSE

        if stage == STAGE_EXPLORING:
            # 화점 감지 시 FIRE_RESPONSE 단계로 전환
            self.fire_response_target = fire_pt
            self._current_fire_severity = msg.severity
            self._log.warn('Stage → FIRE_RESPONSE')
            return [
                StageTransition(
                    new_stage=STAGE_FIRE_RESPONSE,
                    reason='fire_detected',
                ),
                FireResponseRequest(
                    fire_x=fire_pt.x,
                    fire_y=fire_pt.y,
                    severity=msg.severity,
                ),
            ]

        elif stage == STAGE_FIRE_RESPONSE:
            # 추가 화점: 더 심각하면 대응 대상 갱신
            severity_rank = {'low': 0, 'medium': 1, 'high': 2, 'critical': 3}
            current_sev = severity_rank.get(self._current_fire_severity or 'low', 0)
            new_sev = severity_rank.get(msg.severity, 0)
            if new_sev > current_sev:
                self.fire_response_target = fire_pt
                self._current_fire_severity = msg.severity
                self._log.warn(
                    f'Fire escalation: {msg.severity} — re-dispatching'
                )
                return [FireResponseRequest(
                    fire_x=fire_pt.x,
                    fire_y=fire_pt.y,
                    severity=msg.severity,
                )]

        return []

    # ─────────────────── 화재 합의 검증 ───────────────────

    def verify_fire_with_consensus(self, fire_pt, reporting_rid: str,
                                   robots_snapshot: dict) -> str:
        """화점 감지 합의 메커니즘 — 다중 로봇 교차 검증.

        화점 주변 CONSENSUS_RADIUS(5m) 이내에 다른 로봇이 있고
        해당 로봇이 최근(5초 이내) 활성 상태이면 "confirmed" 반환.

        Args:
            fire_pt: 화점 위치 (x, y 속성을 가진 Point 객체)
            reporting_rid: 최초 감지 로봇 ID
            robots_snapshot: {robot_id: RobotRecord} 딕셔너리 복사본

        Returns:
            str: "confirmed" | "unconfirmed" | "denied"
        """
        now = self._clock_fn()
        confirming_robots = []

        for rid, r in robots_snapshot.items():
            if rid == reporting_rid:
                continue
            if r.comm_lost or r.pose is None:
                continue
            # 최근 5초 이내 활성 상태 확인
            if now - r.last_seen > 5.0:
                continue

            rx = r.pose.pose.position.x
            ry = r.pose.pose.position.y
            dist_to_fire = math.hypot(rx - fire_pt.x, ry - fire_pt.y)
            if dist_to_fire <= CONSENSUS_RADIUS:
                confirming_robots.append((rid, dist_to_fire))

        if len(confirming_robots) >= 1:
            self._log.info(
                f'FIRE CONSENSUS confirmed by '
                f'{[r[0] for r in confirming_robots]} '
                f'(within {CONSENSUS_RADIUS}m of fire)'
            )
            return 'confirmed'
        else:
            self._log.warn(
                f'FIRE UNCONFIRMED: only {reporting_rid} detected fire at '
                f'({fire_pt.x:.1f}, {fire_pt.y:.1f}) — awaiting 2nd confirmation'
            )
            return 'unconfirmed'

    # ─────────────────── 충돌 방지 ───────────────────

    def check_collision_risk(self, target, excluding_rid: str,
                              robots_snapshot: dict) -> bool:
        """충돌 방지 — 파견 대상 위치에 다른 로봇이 근접해 있는지 확인.

        COLLISION_SAFE_DISTANCE(2m) 이내에 다른 로봇이 있으면
        True(위험)를 반환하여 파견을 보류한다.

        Args:
            target: 목표 위치 (x, y 속성을 가진 Point 객체)
            excluding_rid: 검사 제외 로봇 ID
            robots_snapshot: {robot_id: RobotRecord} 딕셔너리 복사본

        Returns:
            bool: True = 충돌 위험 있음 (파견 보류 권고)
        """
        for rid, r in robots_snapshot.items():
            if rid == excluding_rid:
                continue
            if r.comm_lost or r.pose is None:
                continue
            rx = r.pose.pose.position.x
            ry = r.pose.pose.position.y
            dist = math.hypot(rx - target.x, ry - target.y)
            if dist < COLLISION_SAFE_DISTANCE:
                self._log.warn(
                    f'COLLISION RISK: {rid} is {dist:.1f}m from target '
                    f'({target.x:.1f}, {target.y:.1f}) — dispatch deferred'
                )
                return True

        return False

    # ─────────────────── 화재 확산 추정 ───────────────────

    def estimate_fire_spread(self) -> dict:
        """열화상 시계열 데이터로 화재 확산 속도 추정.

        Kalman 필터 개념을 단순화하여 적용:
          - 상태: [온도(K), 온도변화율(K/s)]
          - 관측: max_temperature_kelvin
          - 예측 출력: spread_rate(K/s), trend('growing'|'stable'|'declining')

        열화상 데이터가 3개 미만이면 추정 불가 반환.

        Returns:
            dict: {
                'spread_rate': float (K/s),
                'trend': str ('growing'|'stable'|'declining'|'unknown'),
                'estimated_radius_m': float,
                'data_points': int,
            }
        """
        if len(self._thermal_history) < 3:
            return {
                'spread_rate': 0.0,
                'trend': 'unknown',
                'estimated_radius_m': 0.0,
                'data_points': len(self._thermal_history),
            }

        # 시계열 → 온도 변화율 계산 (최소제곱 선형 회귀)
        times = [t for t, _ in self._thermal_history]
        temps = [k for _, k in self._thermal_history]

        t0 = times[0]
        n = len(times)
        sum_x = sum(t - t0 for t in times)
        sum_y = sum(temps)
        sum_xy = sum((t - t0) * temp for t, temp in zip(times, temps))
        sum_x2 = sum((t - t0) ** 2 for t in times)
        denom = n * sum_x2 - sum_x ** 2
        if abs(denom) < 1e-9:
            spread_rate = 0.0
        else:
            spread_rate = (n * sum_xy - sum_x * sum_y) / denom  # K/s

        # 화재 반경 추정: 경험식 r = sqrt(T_excess / k), k=10 (간이 모델)
        latest_temp = temps[-1]
        ambient_temp_k = 300.0  # 상온 27°C = 300K
        temp_excess = max(0.0, latest_temp - ambient_temp_k)
        estimated_radius = math.sqrt(temp_excess / 10.0) if temp_excess > 0 else 0.0

        # 트렌드 판정
        if spread_rate > 5.0:
            trend = 'growing'
        elif spread_rate < -5.0:
            trend = 'declining'
        else:
            trend = 'stable'

        if trend == 'growing':
            self._log.warn(
                f'FIRE SPREADING: rate={spread_rate:.1f}K/s, '
                f'est_radius={estimated_radius:.1f}m — 진압 로봇 추가 파견 검토'
            )

        return {
            'spread_rate': spread_rate,
            'trend': trend,
            'estimated_radius_m': estimated_radius,
            'data_points': n,
        }

    # ─────────────────── 종합 위험도 점수 ───────────────────

    def compute_situation_score(self, smoke_density: float = 0.0) -> dict:
        """8중 센싱 데이터 가중 합산 — 종합 위험도 점수 산출.

        Args:
            smoke_density: 연기 농도 (0.0=맑음 ~ 1.0=100%/m 농연). # NFRI 2025 리빙랩

        Returns:
            dict: {
                'score': float (0.0~1.0),
                'level': str ('safe'|'caution'|'danger'|'critical'),
                'recommend': str ('continue'|'monitor'|'pause'|'evacuate'),
                'factors': dict (각 센서별 점수),
            }
        """
        # NFRI 실험 데이터 기반 센서 환경별 신뢰도 (2025 리빙랩 실화재 실험) # NFRI 2025 리빙랩
        # smoke_density: 0.0(맑음) ~ 1.0(100%/m 농연)
        # 튜플 순서: (clear, light_smoke, dense_smoke, water_spray)
        SENSOR_RELIABILITY_NFRI = {
            'gas':        (1.0, 0.95, 0.90, 0.85),  # 가스: 농연 영향 적음
            'fire':       (1.0, 0.85, 0.50, 0.30),  # 열화상: 농연 시 시인성 저하, 분무 시 노이즈
            'structural': (1.0, 0.70, 0.30, 0.20),  # LiDAR: 농연 시 성능 급락
            'victim':     (1.0, 0.80, 0.60, 0.40),  # IR: 농연 중간, 분무 시 저하
            'audio':      (1.0, 1.0,  0.95, 0.90),  # 음향: 농연 영향 없음, 분무 소음만
        }

        def _smoke_reliability(key: str) -> float:
            """smoke_density 값에 따라 신뢰도 테이블에서 선형 보간."""
            # NFRI 2025 리빙랩: 0.0→clear, 0~0.4→light, 0.4~0.8→dense, 0.8~1.0→water_spray 근사
            vals = SENSOR_RELIABILITY_NFRI[key]
            if smoke_density <= 0.0:
                return vals[0]
            elif smoke_density <= 0.4:
                t = smoke_density / 0.4
                return vals[0] + t * (vals[1] - vals[0])
            elif smoke_density <= 0.8:
                t = (smoke_density - 0.4) / 0.4
                return vals[1] + t * (vals[2] - vals[1])
            else:
                t = (smoke_density - 0.8) / 0.2
                return vals[2] + t * (vals[3] - vals[2])
        # 가스 점수
        gas_score = {
            'safe': 0.0, 'caution': 0.2, 'danger': 0.6, 'critical': 1.0
        }.get(self.gas_danger_level, 0.0)

        # 화재 점수
        fire_score = 0.0
        if self.fire_response_target is not None:
            sev = self._current_fire_severity or 'low'
            fire_score = {
                'low': 0.1, 'medium': 0.3, 'high': 0.6, 'critical': 1.0
            }.get(sev, 0.0)

        # 구조물 점수 (blocked_areas 기반 간이 산출)
        structural_score = 0.0
        if self.blocked_areas:
            # 막힌 구역이 있으면 최소 danger 수준 적용
            structural_score = 0.4
            # 최근 추가된 구역의 반경이 클수록 위험도 상승
            latest = self.blocked_areas[-1]
            radius = latest.get('radius', 0.0)
            if radius > 5.0:
                structural_score = 0.6

        # 피해자 점수
        victim_score = min(1.0, len(self.victims_detected) * 0.3)

        # 음향 점수
        audio_score = 0.0
        if self.audio_alerts:
            latest = self.audio_alerts[-1]
            if getattr(latest, 'immediate_response_needed', False):
                audio_score = 0.8
            elif getattr(latest, 'danger_level', 'safe') in ('danger', 'critical'):
                audio_score = 0.5

        # 가중 합산 (NFRI 신뢰도 보정 적용) # NFRI 2025 리빙랩
        weights = {
            'gas': 0.30,
            'fire': 0.25,
            'structural': 0.20,
            'victim': 0.15,
            'audio': 0.10,
        }

        # smoke_density 기반 신뢰도 보정: 가중치에 신뢰도 곱수 반영
        adjusted_weights = {k: w * _smoke_reliability(k) for k, w in weights.items()}

        total_score = (
            adjusted_weights['gas'] * gas_score +
            adjusted_weights['fire'] * fire_score +
            adjusted_weights['structural'] * structural_score +
            adjusted_weights['victim'] * victim_score +
            adjusted_weights['audio'] * audio_score
        )

        # 종합 판정
        if total_score >= 0.7:
            level, recommend = 'critical', 'evacuate'
        elif total_score >= 0.5:
            level, recommend = 'danger', 'pause'
        elif total_score >= 0.3:
            level, recommend = 'caution', 'monitor'
        else:
            level, recommend = 'safe', 'continue'

        return {
            'score': total_score,
            'level': level,
            'recommend': recommend,
            'factors': {
                'gas': gas_score,
                'fire': fire_score,
                'structural': structural_score,
                'victim': victim_score,
                'audio': audio_score,
            },
        }
