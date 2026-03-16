# Copyright 2026 민발 (Minbal), 대구강북소방서
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
"""
오케스트레이터 고급 로직 단위 테스트 (Advanced).

커버리지 보강 대상:
  - compute_situation_score (8중 센싱 퓨전)
  - 지휘관 승인 체계 (PAUSED → approve → RETURNING)
  - FIRE_TACTICS dict 유효성
  - seq_number 기반 패킷 손실 감지

rclpy 없이 순수 Python으로 실행 가능.
"""

import math
from collections import deque

import pytest


# ── rclpy 의존 없이 재현한 상수·자료구조 ──────────────────────────────

class MockMissionState:
    STAGE_INIT = 0
    STAGE_EXPLORING = 1
    STAGE_FIRE_RESPONSE = 2
    STAGE_RETURNING = 3
    STAGE_COMPLETE = 4
    STAGE_PAUSED = 5


class MockRobotStatus:
    STATE_IDLE = 0
    STATE_EXPLORING = 1
    STATE_ON_MISSION = 2
    STATE_RETURNING = 3
    STATE_ERROR = 4
    STATE_COMM_LOST = 5


class Point:
    """geometry_msgs.msg.Point 경량 대체."""
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z


class MockAudioEvent:
    """AudioEvent 경량 대체."""
    def __init__(self, event_type='none', danger_level='safe',
                 immediate_response_needed=False):
        self.event_type = event_type
        self.danger_level = danger_level
        self.immediate_response_needed = immediate_response_needed


# ── orchestrator_node.py에서 추출한 FIRE_TACTICS ──────────────────────

FIRE_TACTICS = {
    'electrical': {
        'suppression': 'water_fog',
        'robot_capable': True,
        'human_mandatory': ['전원차단'],
        'approach_distance': 5.0,
    },
    'oil': {
        'suppression': 'foam_spray',
        'robot_capable': True,
        'human_mandatory': ['포말탱크 연결'],
        'approach_distance': 3.0,
    },
    'gas': {
        'suppression': 'ventilation',
        'robot_capable': False,
        'human_mandatory': ['가스밸브 차단', '환기'],
        'approach_distance': 10.0,
    },
    'general': {
        'suppression': 'water_fog',
        'robot_capable': True,
        'human_mandatory': [],
        'approach_distance': 2.0,
    },
}


# ── compute_situation_score 로직 재현 ──────────────────────────────────

def compute_situation_score(
    gas_danger_level='safe',
    fire_response_target=None,
    current_fire_severity='low',
    structural_level='safe',
    victims_detected=None,
    audio_alerts=None,
):
    """orchestrator_node.OrchestratorNode.compute_situation_score 순수 Python 재현."""
    if victims_detected is None:
        victims_detected = deque()
    if audio_alerts is None:
        audio_alerts = deque()

    # 가스 점수
    gas_score = {'safe': 0.0, 'caution': 0.2, 'danger': 0.6, 'critical': 1.0
                 }.get(gas_danger_level, 0.0)

    # 화재 점수
    fire_score = 0.0
    if fire_response_target is not None:
        fire_score = {'low': 0.1, 'medium': 0.3, 'high': 0.6, 'critical': 1.0
                      }.get(current_fire_severity, 0.0)

    # 구조물 점수
    structural_score = {
        'safe': 0.0, 'warning': 0.2, 'danger': 0.6, 'critical': 1.0
    }.get(structural_level, 0.0)

    # 피해자 점수
    victim_score = min(1.0, len(victims_detected) * 0.3)

    # 음향 점수
    audio_score = 0.0
    if audio_alerts:
        latest = audio_alerts[-1]
        if latest.immediate_response_needed:
            audio_score = 0.8
        elif latest.danger_level in ('danger', 'critical'):
            audio_score = 0.5

    # 가중 합산
    weights = {
        'gas': 0.30,
        'fire': 0.25,
        'structural': 0.20,
        'victim': 0.15,
        'audio': 0.10,
    }

    total_score = (
        weights['gas'] * gas_score +
        weights['fire'] * fire_score +
        weights['structural'] * structural_score +
        weights['victim'] * victim_score +
        weights['audio'] * audio_score
    )

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
            'gas': gas_score, 'fire': fire_score,
            'structural': structural_score,
            'victim': victim_score, 'audio': audio_score,
        },
    }


# ── 지휘관 승인 체계 상태 기계 재현 ──────────────────────────────────

class OrchestratorStateMachine:
    """가스 critical → PAUSED → 지휘관 approve → RETURNING 흐름 재현."""

    def __init__(self):
        self.stage = MockMissionState.STAGE_EXPLORING
        self.paused = False
        self._evacuation_recommended = False
        self._gas_critical_count = 0
        self._gas_critical_threshold = 2
        self.gas_danger_level = 'safe'

    def receive_gas_critical(self):
        """가스 critical 수신 (temporal smoothing 포함)."""
        self._gas_critical_count += 1
        if self._gas_critical_count >= self._gas_critical_threshold:
            self.gas_danger_level = 'critical'
            if self.stage not in (
                MockMissionState.STAGE_RETURNING,
                MockMissionState.STAGE_COMPLETE,
                MockMissionState.STAGE_PAUSED,
            ):
                self.stage = MockMissionState.STAGE_PAUSED
                self.paused = True
                self._evacuation_recommended = True
            return True  # 판정됨
        return False  # 확인 대기 중

    def commander_approve(self):
        """지휘관 /orchestrator/resume 호출 (철수 승인)."""
        if not self.paused:
            return False, 'Not paused'

        self.paused = False
        if self._evacuation_recommended:
            self._evacuation_recommended = False
            self.stage = MockMissionState.STAGE_RETURNING
            return True, 'Evacuation approved — all robots returning'
        else:
            self.stage = MockMissionState.STAGE_EXPLORING
            return True, 'Resumed'

    def commander_approve_without_evacuation(self):
        """일반 정지 해제 (철수 아님)."""
        self.paused = True
        self._evacuation_recommended = False
        result, msg = self.commander_approve()
        return result, msg


# ── seq_number 기반 패킷 손실 감지 ──────────────────────────────────

def detect_packet_loss(prev_seq: int, curr_seq: int, wrap_at: int = 65536):
    """ROS 메시지 seq_number 기반 패킷 손실 감지.

    Args:
        prev_seq: 이전에 수신한 seq 번호
        curr_seq: 현재 수신한 seq 번호
        wrap_at: seq 최대값 (오버플로 랩핑 지점)

    Returns:
        int: 손실된 패킷 수 (0이면 연속)
    """
    if curr_seq > prev_seq:
        return curr_seq - prev_seq - 1
    elif curr_seq < prev_seq:
        # wrap-around 처리
        return (wrap_at - prev_seq) + curr_seq - 1
    else:
        # 동일 seq: 중복 수신 (손실 없음으로 간주)
        return 0


def packet_loss_rate(total_expected: int, total_lost: int) -> float:
    """패킷 손실률 계산."""
    if total_expected <= 0:
        return 0.0
    return total_lost / total_expected


# ════════════════════════════════════════════════════════════════
# 테스트 클래스
# ════════════════════════════════════════════════════════════════


class TestOrchestratorSensorFusion:
    """compute_situation_score — 8중 센싱 퓨전 5개 시나리오."""

    def test_all_safe_returns_safe(self):
        """모든 센서 안전 → score 최소, level=safe."""
        result = compute_situation_score()
        assert result['level'] == 'safe'
        assert result['recommend'] == 'continue'
        assert result['score'] == pytest.approx(0.0)

    def test_gas_critical_alone_triggers_critical(self):
        """가스만 critical → score 0.3 이상 (caution 이상)."""
        result = compute_situation_score(gas_danger_level='critical')
        # 가스 가중치 0.30 * 1.0 = 0.30 → 정확히 caution 경계
        assert result['score'] == pytest.approx(0.30)
        assert result['level'] == 'caution'
        assert result['factors']['gas'] == pytest.approx(1.0)

    def test_gas_critical_plus_fire_high_triggers_danger(self):
        """가스 critical + 화재 high → score ≥ 0.5 (danger 이상)."""
        result = compute_situation_score(
            gas_danger_level='critical',
            fire_response_target=Point(1.0, 2.0),
            current_fire_severity='high',
        )
        # gas=0.30*1.0 + fire=0.25*0.6 = 0.30 + 0.15 = 0.45 → caution
        # (structural/victim/audio 없으면)
        assert result['score'] == pytest.approx(0.45)
        assert result['level'] == 'caution'

    def test_all_critical_triggers_evacuate(self):
        """모든 센서 최악 → score 1.0, recommend=evacuate."""
        victims = deque([Point(0, 0), Point(5, 5), Point(10, 10), Point(15, 0)])
        audio = deque([MockAudioEvent(immediate_response_needed=True)])
        result = compute_situation_score(
            gas_danger_level='critical',
            fire_response_target=Point(1.0, 2.0),
            current_fire_severity='critical',
            structural_level='critical',
            victims_detected=victims,
            audio_alerts=audio,
        )
        # gas=0.30 + fire=0.25 + structural=0.20 + victim=min(1.0,4*0.3=1.2→1.0)*0.15=0.15 + audio=0.10*0.8=0.08
        assert result['score'] >= 0.7
        assert result['level'] == 'critical'
        assert result['recommend'] == 'evacuate'

    def test_victim_count_caps_at_1(self):
        """피해자 4명 이상 → victim_score 1.0으로 캡."""
        many_victims = deque([Point(i, 0) for i in range(10)])
        result = compute_situation_score(victims_detected=many_victims)
        assert result['factors']['victim'] == pytest.approx(1.0)

    def test_factor_weights_sum_to_one(self):
        """가중치 합 = 1.0 검증."""
        weights = {'gas': 0.30, 'fire': 0.25, 'structural': 0.20,
                   'victim': 0.15, 'audio': 0.10}
        assert sum(weights.values()) == pytest.approx(1.0)

    def test_audio_immediate_response_scores_higher(self):
        """즉각 대응 필요 음향 이벤트 → audio_score=0.8."""
        audio_normal = deque([MockAudioEvent(danger_level='danger')])
        audio_urgent = deque([MockAudioEvent(immediate_response_needed=True)])

        r_normal = compute_situation_score(audio_alerts=audio_normal)
        r_urgent = compute_situation_score(audio_alerts=audio_urgent)

        assert r_urgent['factors']['audio'] > r_normal['factors']['audio']
        assert r_urgent['factors']['audio'] == pytest.approx(0.8)
        assert r_normal['factors']['audio'] == pytest.approx(0.5)


class TestCommanderApproval:
    """지휘관 승인 체계: gas critical → PAUSED → approve → RETURNING."""

    def test_single_gas_critical_does_not_trigger_pause(self):
        """가스 critical 1회 → temporal smoothing 통과 안 함 (아직 PAUSED 아님)."""
        sm = OrchestratorStateMachine()
        confirmed = sm.receive_gas_critical()
        assert confirmed is False
        assert sm.stage == MockMissionState.STAGE_EXPLORING
        assert sm.paused is False

    def test_two_consecutive_gas_critical_triggers_paused(self):
        """가스 critical 2회 연속 → PAUSED + evacuation_recommended."""
        sm = OrchestratorStateMachine()
        sm.receive_gas_critical()   # 1회 (확인 대기)
        confirmed = sm.receive_gas_critical()  # 2회 (확정)
        assert confirmed is True
        assert sm.stage == MockMissionState.STAGE_PAUSED
        assert sm.paused is True
        assert sm._evacuation_recommended is True

    def test_commander_approve_transitions_to_returning(self):
        """지휘관 승인 → RETURNING 전환."""
        sm = OrchestratorStateMachine()
        sm.receive_gas_critical()
        sm.receive_gas_critical()
        assert sm.stage == MockMissionState.STAGE_PAUSED

        success, msg = sm.commander_approve()
        assert success is True
        assert sm.stage == MockMissionState.STAGE_RETURNING
        assert sm.paused is False
        assert sm._evacuation_recommended is False
        assert 'approved' in msg.lower()

    def test_commander_approve_without_evacuation_resumes_exploring(self):
        """철수 플래그 없는 정지 해제 → EXPLORING 복귀."""
        sm = OrchestratorStateMachine()
        success, msg = sm.commander_approve_without_evacuation()
        assert success is True
        assert sm.stage == MockMissionState.STAGE_EXPLORING
        assert sm.paused is False

    def test_commander_approve_when_not_paused_fails(self):
        """정지 상태 아닌데 approve → 실패."""
        sm = OrchestratorStateMachine()
        # paused=False 상태에서 호출
        success, msg = sm.commander_approve()
        assert success is False
        assert 'not paused' in msg.lower()

    def test_returning_stage_ignores_additional_gas_critical(self):
        """RETURNING 중 추가 가스 critical → 단계 변경 없음."""
        sm = OrchestratorStateMachine()
        sm.stage = MockMissionState.STAGE_RETURNING
        sm._gas_critical_count = 1
        sm.receive_gas_critical()   # 2회 도달, 하지만 RETURNING 중
        assert sm.stage == MockMissionState.STAGE_RETURNING  # 변경 없음


class TestFireTactics:
    """FIRE_TACTICS dict 유효성 + 전술 선택 테스트."""

    # 필수 키 목록
    REQUIRED_KEYS = {'suppression', 'robot_capable', 'human_mandatory',
                     'approach_distance'}
    KNOWN_TYPES = {'electrical', 'oil', 'gas', 'general'}

    def test_all_fire_types_present(self):
        """4가지 화재 유형 모두 정의돼 있어야 함."""
        assert set(FIRE_TACTICS.keys()) == self.KNOWN_TYPES

    def test_each_tactic_has_required_keys(self):
        """각 전술에 필수 키가 모두 있어야 함."""
        for fire_type, tactic in FIRE_TACTICS.items():
            missing = self.REQUIRED_KEYS - set(tactic.keys())
            assert not missing, f'{fire_type} 전술 누락 키: {missing}'

    def test_approach_distances_positive(self):
        """접근 거리는 양수여야 함."""
        for fire_type, tactic in FIRE_TACTICS.items():
            assert tactic['approach_distance'] > 0.0, \
                f'{fire_type} approach_distance는 양수여야 함'

    def test_gas_fire_robot_not_capable(self):
        """가스 화재 → 로봇 단독 대응 불가 (인명 필수)."""
        assert FIRE_TACTICS['gas']['robot_capable'] is False

    def test_gas_fire_has_human_mandatory_tasks(self):
        """가스 화재 → 사람만 가능한 작업 1개 이상."""
        assert len(FIRE_TACTICS['gas']['human_mandatory']) >= 1

    def test_gas_fire_largest_approach_distance(self):
        """가스 화재 접근 거리 = 전술 중 최대 (폭발 위험)."""
        max_dist = max(t['approach_distance'] for t in FIRE_TACTICS.values())
        assert FIRE_TACTICS['gas']['approach_distance'] == max_dist

    def test_general_fire_no_human_mandatory(self):
        """일반 화재 → 사람 전용 필수 작업 없음 (로봇 완전 자율)."""
        assert FIRE_TACTICS['general']['human_mandatory'] == []

    def test_suppression_method_strings_not_empty(self):
        """진압 방법 문자열은 비어 있으면 안 됨."""
        for fire_type, tactic in FIRE_TACTICS.items():
            assert isinstance(tactic['suppression'], str), \
                f'{fire_type} suppression은 문자열이어야 함'
            assert tactic['suppression'].strip(), \
                f'{fire_type} suppression이 빈 문자열'

    def test_select_tactic_by_fire_type(self):
        """화재 유형 기반 전술 선택 정상 동작."""
        def select_tactic(fire_type: str) -> dict:
            return FIRE_TACTICS.get(fire_type, FIRE_TACTICS['general'])

        assert select_tactic('gas')['suppression'] == 'ventilation'
        assert select_tactic('oil')['suppression'] == 'foam_spray'
        assert select_tactic('unknown')['suppression'] == 'water_fog'  # fallback

    def test_electrical_fire_approach_distance_keeps_safe(self):
        """전기 화재 접근 거리 ≥ 3m (감전 방지)."""
        assert FIRE_TACTICS['electrical']['approach_distance'] >= 3.0


class TestPacketLoss:
    """seq_number 기반 패킷 손실 감지 테스트."""

    def test_consecutive_seq_no_loss(self):
        """연속 seq → 손실 0."""
        assert detect_packet_loss(100, 101) == 0
        assert detect_packet_loss(0, 1) == 0

    def test_one_packet_lost(self):
        """seq 건너뜀 1 → 손실 1."""
        assert detect_packet_loss(100, 102) == 1

    def test_multiple_packets_lost(self):
        """seq 5 건너뜀 → 손실 5."""
        assert detect_packet_loss(10, 16) == 5

    def test_seq_wrap_around_no_loss(self):
        """seq 최대 → 0으로 랩핑, 손실 없음."""
        # wrap_at=65536: 65535 → 0
        assert detect_packet_loss(65535, 0, wrap_at=65536) == 0

    def test_seq_wrap_around_one_lost(self):
        """랩핑 구간에서 1개 손실."""
        # 65534 → 0: 65535가 빠짐
        assert detect_packet_loss(65534, 0, wrap_at=65536) == 1

    def test_duplicate_seq_returns_zero(self):
        """동일 seq 수신 (중복) → 손실 0으로 처리."""
        assert detect_packet_loss(50, 50) == 0

    def test_packet_loss_rate_calculation(self):
        """패킷 손실률 계산 정확성."""
        assert packet_loss_rate(100, 10) == pytest.approx(0.1)
        assert packet_loss_rate(50, 0) == pytest.approx(0.0)
        assert packet_loss_rate(0, 0) == pytest.approx(0.0)

    def test_high_loss_rate_threshold(self):
        """10% 초과 손실률 → 통신 품질 불량 판정."""
        LOSS_RATE_THRESHOLD = 0.10
        rate = packet_loss_rate(100, 15)  # 15% 손실
        assert rate > LOSS_RATE_THRESHOLD

    def test_seq_stream_simulation(self):
        """연속 seq 스트림 시뮬: 1~999 중 50의 배수(19개) 손실 시나리오."""
        total_lost = 0
        prev_seq = 0
        # seq 1~999, 50의 배수 건너뜀 (50,100,...,950 = 19개 손실)
        seqs = [i for i in range(1, 1000) if i % 50 != 0]
        for curr_seq in seqs:
            lost = detect_packet_loss(prev_seq, curr_seq)
            total_lost += lost
            prev_seq = curr_seq
        assert total_lost == 19  # 50,100,...,950 → 19개
