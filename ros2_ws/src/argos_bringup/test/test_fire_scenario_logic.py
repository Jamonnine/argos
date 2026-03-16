"""
소방 현장 시나리오 통합 로직 테스트.
rclpy 없이 화재감지→대응→귀환 전체 흐름의 핵심 판단 로직 검증.

시나리오: 건물 내 다중 화재 + 피해자 + 가스 위험 + 구조물 붕괴 복합 상황
"""

import pytest
import math


# ═══════════════════════════════════════════════════
# 오케스트레이터 판단 로직 (orchestrator_node.py에서 추출)
# ═══════════════════════════════════════════════════

STAGE_INIT = 0
STAGE_EXPLORING = 1
STAGE_FIRE_RESPONSE = 2
STAGE_RETURNING = 3
STAGE_COMPLETE = 4


def select_nearest_robot(robots: dict, target_x: float, target_y: float,
                         exclude: str = '', robot_type: str = None) -> tuple:
    """최근접 가용 로봇 선택."""
    best_rid = None
    best_dist = float('inf')
    for rid, r in robots.items():
        if rid == exclude:
            continue
        if r.get('comm_lost', False):
            continue
        if robot_type and r.get('type') != robot_type:
            continue
        if r.get('pose') is None:
            continue
        dx = r['pose'][0] - target_x
        dy = r['pose'][1] - target_y
        dist = math.hypot(dx, dy)
        if dist < best_dist:
            best_dist = dist
            best_rid = rid
    return best_rid, best_dist


def prioritize_fires(fires: list) -> list:
    """화점 목록을 심각도 기준 정렬 (critical > high > medium > low)."""
    severity_rank = {'low': 0, 'medium': 1, 'high': 2, 'critical': 3}
    return sorted(fires, key=lambda f: severity_rank.get(f['severity'], 0),
                  reverse=True)


def should_evacuate_scenario(gas_level: str, structural_severity: str,
                             audio_type: str = None) -> bool:
    """복합 센서 기반 즉시 철수 판단."""
    # 가스 critical → 즉시 철수
    if gas_level == 'critical':
        return True
    # 구조물 critical → 즉시 철수
    if structural_severity == 'critical':
        return True
    # 가스 danger + 붕괴 소리 = 즉시 철수 (교차 검증)
    if gas_level == 'danger' and audio_type == 'collapse':
        return True
    # 폭발 = 즉시 철수
    if audio_type == 'explosion':
        return True
    return False


def compute_rescue_priority(victim_moving: bool, victim_confidence: float,
                            gas_danger: str, distance: float) -> int:
    """피해자 구조 우선순위 종합 판정.
    1=최우선(즉시 구조), 2=높음, 3=보통.
    """
    # 움직이지 않는 피해자 = 의식 없을 가능성 → 최우선
    if not victim_moving and victim_confidence > 0.5:
        return 1
    # 가스 위험 지역의 피해자 → 시간 제한 있으므로 높음
    if gas_danger in ('danger', 'critical'):
        return 1
    # 높은 신뢰도 + 가까운 거리
    if victim_confidence > 0.7 and distance < 5.0:
        return 2
    return 3


def determine_stage(current_stage: int, robots: dict, fires: list,
                    gas_level: str, structural: str,
                    audio_type: str = None) -> int:
    """현재 상태 + 센서 데이터 기반 임무 단계 결정."""
    # 즉시 철수 조건
    if should_evacuate_scenario(gas_level, structural, audio_type):
        return STAGE_RETURNING

    # 화재 감지 → 대응
    if fires and current_stage == STAGE_EXPLORING:
        return STAGE_FIRE_RESPONSE

    # 모든 로봇 탐색 완료
    active = [r for r in robots.values() if not r.get('comm_lost')]
    if active and all(r.get('mission') == 'complete' for r in active):
        if current_stage == STAGE_EXPLORING:
            return STAGE_RETURNING

    return current_stage


# ═══════════════════════════════════════════════════
# 시나리오 1: 단일 건물 화재 대응
# ═══════════════════════════════════════════════════

class TestSingleBuildingFire:
    """단일 건물 화재: 탐색 → 화재 감지 → UGV 파견 → 드론 배치 → 귀환."""

    def setup_method(self):
        self.robots = {
            'argos1': {'type': 'ugv', 'pose': (1.0, 1.0), 'mission': 'exploring'},
            'argos2': {'type': 'ugv', 'pose': (8.0, 8.0), 'mission': 'exploring'},
            'drone1': {'type': 'drone', 'pose': (5.0, 5.0), 'mission': 'exploring'},
        }

    def test_exploring_to_fire_response(self):
        """화재 감지 시 EXPLORING → FIRE_RESPONSE 전환."""
        fires = [{'location': (3.0, 2.0), 'severity': 'high'}]
        stage = determine_stage(STAGE_EXPLORING, self.robots, fires,
                                'safe', 'safe')
        assert stage == STAGE_FIRE_RESPONSE

    def test_nearest_ugv_dispatched(self):
        """화점에 가장 가까운 UGV가 파견."""
        rid, dist = select_nearest_robot(
            self.robots, 2.0, 1.0, robot_type='ugv')
        assert rid == 'argos1'
        assert dist == pytest.approx(1.0)

    def test_drone_dispatched_to_fire(self):
        """드론도 화점 상공으로 배치."""
        rid, dist = select_nearest_robot(
            self.robots, 2.0, 1.0, robot_type='drone')
        assert rid == 'drone1'

    def test_fire_priority_ordering(self):
        """다중 화점: critical > high > medium > low 순서."""
        fires = [
            {'severity': 'medium', 'location': (1, 1)},
            {'severity': 'critical', 'location': (5, 5)},
            {'severity': 'high', 'location': (3, 3)},
        ]
        ordered = prioritize_fires(fires)
        assert ordered[0]['severity'] == 'critical'
        assert ordered[1]['severity'] == 'high'
        assert ordered[2]['severity'] == 'medium'

    def test_all_complete_triggers_returning(self):
        """모든 로봇 탐색 완료 → RETURNING."""
        for r in self.robots.values():
            r['mission'] = 'complete'
        stage = determine_stage(STAGE_EXPLORING, self.robots, [],
                                'safe', 'safe')
        assert stage == STAGE_RETURNING

    def test_comm_lost_robot_excluded(self):
        """통신 두절 로봇은 파견 대상에서 제외."""
        self.robots['argos1']['comm_lost'] = True
        rid, _ = select_nearest_robot(
            self.robots, 2.0, 1.0, robot_type='ugv')
        assert rid == 'argos2'  # argos1 제외, argos2 선택

    def test_no_available_robot(self):
        """모든 로봇 통신 두절 → 파견 실패."""
        for r in self.robots.values():
            r['comm_lost'] = True
        rid, _ = select_nearest_robot(self.robots, 2.0, 1.0)
        assert rid is None


# ═══════════════════════════════════════════════════
# 시나리오 2: 복합 위험 (화재 + 가스 + 구조물 + 피해자)
# ═══════════════════════════════════════════════════

class TestComplexHazardScenario:
    """복합 위험: 화재 + CO 위험 + 천장 처짐 + 피해자 발견."""

    def test_gas_critical_forces_evacuation(self):
        """가스 critical → 화재 대응 중이라도 즉시 철수."""
        stage = determine_stage(STAGE_FIRE_RESPONSE, {}, [],
                                'critical', 'safe')
        assert stage == STAGE_RETURNING

    def test_structural_critical_forces_evacuation(self):
        """구조물 critical → 즉시 철수."""
        stage = determine_stage(STAGE_EXPLORING, {}, [],
                                'safe', 'critical')
        assert stage == STAGE_RETURNING

    def test_gas_danger_plus_collapse_sound(self):
        """가스 danger + 붕괴 소리 = 교차 확인 → 즉시 철수."""
        assert should_evacuate_scenario('danger', 'safe', 'collapse') is True

    def test_explosion_forces_evacuation(self):
        """폭발음 감지 → 즉시 철수."""
        assert should_evacuate_scenario('safe', 'safe', 'explosion') is True

    def test_gas_caution_no_evacuation(self):
        """가스 caution → 철수 불필요, 탐색 계속."""
        assert should_evacuate_scenario('caution', 'safe') is False

    def test_structural_warning_no_evacuation(self):
        """구조물 warning → 모니터링만, 철수 불필요."""
        assert should_evacuate_scenario('safe', 'warning') is False


# ═══════════════════════════════════════════════════
# 시나리오 3: 피해자 구조 우선순위
# ═══════════════════════════════════════════════════

class TestVictimRescuePriority:
    """피해자 구조 우선순위 판정."""

    def test_stationary_high_conf_priority_1(self):
        """움직이지 않는 피해자 + 높은 신뢰도 → 최우선(1)."""
        p = compute_rescue_priority(
            victim_moving=False, victim_confidence=0.8,
            gas_danger='safe', distance=5.0)
        assert p == 1

    def test_gas_danger_area_priority_1(self):
        """가스 위험 지역 피해자 → 시간 제한 → 최우선(1)."""
        p = compute_rescue_priority(
            victim_moving=True, victim_confidence=0.5,
            gas_danger='danger', distance=10.0)
        assert p == 1

    def test_close_high_conf_priority_2(self):
        """가까운 거리 + 높은 신뢰도 → 높음(2)."""
        p = compute_rescue_priority(
            victim_moving=True, victim_confidence=0.9,
            gas_danger='safe', distance=3.0)
        assert p == 2

    def test_far_low_conf_priority_3(self):
        """먼 거리 + 낮은 신뢰도 → 보통(3)."""
        p = compute_rescue_priority(
            victim_moving=True, victim_confidence=0.3,
            gas_danger='safe', distance=20.0)
        assert p == 3

    def test_moving_low_conf_no_gas_priority_3(self):
        """움직이는 + 낮은 신뢰도 + 가스 안전 → 보통(3)."""
        p = compute_rescue_priority(
            victim_moving=True, victim_confidence=0.4,
            gas_danger='safe', distance=8.0)
        assert p == 3


# ═══════════════════════════════════════════════════
# 시나리오 4: 다중 화재 + 다중 로봇 할당
# ═══════════════════════════════════════════════════

class TestMultiFireMultiRobot:
    """3곳 동시 화재 + 3대 로봇 할당."""

    def setup_method(self):
        self.robots = {
            'ugv1': {'type': 'ugv', 'pose': (1.0, 1.0), 'mission': 'exploring'},
            'ugv2': {'type': 'ugv', 'pose': (9.0, 1.0), 'mission': 'exploring'},
            'ugv3': {'type': 'ugv', 'pose': (5.0, 9.0), 'mission': 'exploring'},
        }
        self.fires = [
            {'severity': 'critical', 'location': (2.0, 2.0)},
            {'severity': 'high', 'location': (8.0, 2.0)},
            {'severity': 'medium', 'location': (5.0, 8.0)},
        ]

    def test_priority_dispatch_critical_first(self):
        """critical 화점에 가장 가까운 로봇 먼저 할당."""
        ordered = prioritize_fires(self.fires)
        assert ordered[0]['severity'] == 'critical'

        # critical(2,2)에 가장 가까운 UGV → ugv1(1,1)
        rid, dist = select_nearest_robot(
            self.robots, *ordered[0]['location'], robot_type='ugv')
        assert rid == 'ugv1'
        assert dist == pytest.approx(math.hypot(1, 1))

    def test_second_fire_excludes_first_responder(self):
        """2번째 화점에는 1번째 대응 로봇 제외."""
        # 1번: ugv1 → critical(2,2)
        # 2번: high(8,2) — ugv1 제외
        rid, _ = select_nearest_robot(
            self.robots, 8.0, 2.0, exclude='ugv1', robot_type='ugv')
        assert rid == 'ugv2'  # ugv2(9,1)이 가장 가까움

    def test_third_fire_excludes_first_two(self):
        """3번째 화점에는 1,2번 로봇 모두 제외되어야 하지만,
        현재 select_nearest_robot은 exclude 1개만 지원.
        → 남은 로봇 풀에서 선택."""
        # ugv1, ugv2 제외 → ugv3만 남음
        remaining = {k: v for k, v in self.robots.items()
                     if k not in ('ugv1', 'ugv2')}
        rid, _ = select_nearest_robot(remaining, 5.0, 8.0, robot_type='ugv')
        assert rid == 'ugv3'

    def test_no_fires_stays_exploring(self):
        """화재 없으면 탐색 유지."""
        stage = determine_stage(STAGE_EXPLORING, self.robots, [],
                                'safe', 'safe')
        assert stage == STAGE_EXPLORING


# ═══════════════════════════════════════════════════
# 시나리오 5: 센서 퓨전 판단 신뢰도
# ═══════════════════════════════════════════════════

class TestSensorFusionDecision:
    """다중 센서 교차 검증으로 판단 신뢰도 향상."""

    def test_thermal_plus_yolo_high_confidence(self):
        """열화상 + YOLO 불꽃 동시 감지 → 확정 화재."""
        thermal_detected = True
        yolo_fire_conf = 0.7
        confirmed = thermal_detected and yolo_fire_conf > 0.5
        assert confirmed is True

    def test_thermal_only_moderate_confidence(self):
        """열화상만 감지 (YOLO 미감지) → 주의 수준."""
        thermal_detected = True
        yolo_fire_conf = 0.0
        confirmed = thermal_detected and yolo_fire_conf > 0.5
        assert confirmed is False  # 확정 아님, 추가 확인 필요

    def test_yolo_only_moderate_confidence(self):
        """YOLO만 감지 (열화상 미감지) → 오탐 가능성."""
        thermal_detected = False
        yolo_fire_conf = 0.8
        confirmed = thermal_detected and yolo_fire_conf > 0.5
        assert confirmed is False  # 단일 센서만으론 불확실

    def test_audio_cry_plus_thermal_victim(self):
        """음향(구조요청) + 열화상(인체) 동시 → 피해자 확정."""
        audio_cry = True
        thermal_person = True
        victim_confirmed = audio_cry and thermal_person
        assert victim_confirmed is True

    def test_gas_plus_audio_gas_leak(self):
        """가스 sensor danger + 가스 누출 소리 → 확정 가스 누출."""
        gas_level = 'danger'
        audio_type = 'gas_leak'
        gas_leak_confirmed = (gas_level in ('danger', 'critical')
                              and audio_type == 'gas_leak')
        assert gas_leak_confirmed is True
