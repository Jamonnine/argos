"""
센서 간 충돌 우선순위 행렬 테스트.
센서 퓨전 전문가 권고: 다중 센서 동시 충돌 시 해소 규칙.
"""
import pytest


# 우선순위 행렬 (센서 퓨전 전문가 권고)
# 높을수록 우선
PRIORITY = {
    'gas_critical': 100,      # 가스 critical → 최우선 (생명 위험)
    'structural_critical': 95, # 구조물 붕괴 → 즉시 철수
    'explosion_audio': 90,     # 폭발음 → 즉시 대피
    'victim_priority_1': 85,   # 움직이지 않는 피해자 → 긴급 구조
    'fire_critical': 80,       # 화재 critical → 진압 대응
    'gas_danger': 70,          # 가스 danger → 경고
    'victim_priority_2': 65,   # 일반 피해자
    'fire_high': 60,           # 화재 high
    'structural_danger': 55,   # 구조물 danger
    'fire_medium': 40,         # 화재 medium
    'gas_caution': 20,         # 가스 주의
    'audio_info': 10,          # 일반 음향
}


def resolve_priority(*events):
    """다중 이벤트 중 최우선 결정."""
    if not events:
        return None, 0
    best = max(events, key=lambda e: PRIORITY.get(e, 0))
    return best, PRIORITY.get(best, 0)


def determine_action(priority_event):
    """우선순위 이벤트에 따른 행동 결정."""
    if priority_event in ('gas_critical', 'structural_critical', 'explosion_audio'):
        return 'evacuate'
    elif priority_event in ('victim_priority_1',):
        return 'rescue_dispatch_2x'  # 2대 파견
    elif priority_event in ('fire_critical', 'fire_high'):
        return 'fire_response'
    elif priority_event in ('gas_danger', 'victim_priority_2'):
        return 'monitor_alert'
    else:
        return 'continue'


class TestPriorityMatrix:

    def test_gas_critical_highest(self):
        best, score = resolve_priority('gas_critical', 'fire_high')
        assert best == 'gas_critical'
        assert score == 100

    def test_structural_over_fire(self):
        best, _ = resolve_priority('structural_critical', 'fire_critical')
        assert best == 'structural_critical'

    def test_explosion_over_victim(self):
        best, _ = resolve_priority('explosion_audio', 'victim_priority_1')
        assert best == 'explosion_audio'

    def test_victim_over_fire(self):
        best, _ = resolve_priority('victim_priority_1', 'fire_critical')
        assert best == 'victim_priority_1'

    def test_triple_conflict(self):
        """가스+화재+피해자 3중 충돌 → 가스 최우선."""
        best, _ = resolve_priority(
            'gas_critical', 'fire_critical', 'victim_priority_1')
        assert best == 'gas_critical'

    def test_all_low_priority(self):
        best, _ = resolve_priority('gas_caution', 'audio_info')
        assert best == 'gas_caution'

    def test_no_events(self):
        best, score = resolve_priority()
        assert best is None
        assert score == 0


class TestActionDetermination:

    def test_evacuate_on_gas_critical(self):
        assert determine_action('gas_critical') == 'evacuate'

    def test_evacuate_on_structural(self):
        assert determine_action('structural_critical') == 'evacuate'

    def test_evacuate_on_explosion(self):
        assert determine_action('explosion_audio') == 'evacuate'

    def test_rescue_2x_on_victim_1(self):
        assert determine_action('victim_priority_1') == 'rescue_dispatch_2x'

    def test_fire_response_on_critical(self):
        assert determine_action('fire_critical') == 'fire_response'

    def test_monitor_on_gas_danger(self):
        assert determine_action('gas_danger') == 'monitor_alert'

    def test_continue_on_low(self):
        assert determine_action('audio_info') == 'continue'

    def test_full_scenario(self):
        """전체 시나리오: 동시 다중 이벤트 → 행동 결정."""
        events = ['fire_high', 'gas_danger', 'victim_priority_2']
        best, _ = resolve_priority(*events)
        action = determine_action(best)
        assert action == 'monitor_alert'  # gas_danger가 70으로 최고
