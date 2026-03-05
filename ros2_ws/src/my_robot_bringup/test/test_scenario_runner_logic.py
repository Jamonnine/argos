"""
Scenario Runner 핵심 로직 단위 테스트.
rclpy 없이 페이즈 진행, 경과시간, 화재 시뮬레이션 메시지 구조 검증.
"""

import pytest


# ── scenario_runner_node.py에서 추출한 상수/로직 ──

# 시나리오 단계 상수
WAIT_READY = 0
DRONE_TAKEOFF = 1
EXPLORING = 2
FIRE_DETECTED = 3
EMERGENCY = 4
RESUME = 5
LANDING = 6
COMPLETE = 7

PHASE_NAMES = [
    'WAIT_READY', 'DRONE_TAKEOFF', 'EXPLORING', 'FIRE_DETECTED',
    'EMERGENCY', 'RESUME', 'LANDING', 'COMPLETE',
]


class MockMissionState:
    STAGE_INIT = 0
    STAGE_EXPLORING = 1
    STAGE_FIRE_RESPONSE = 2
    STAGE_RETURNING = 3
    STAGE_COMPLETE = 4


def phase_elapsed(phase_start_time, current_time_ns):
    """현재 페이즈 시작으로부터의 경과 시간 (초)."""
    if phase_start_time is None:
        return 0.0
    return (current_time_ns - phase_start_time) / 1e9


def total_elapsed(start_time, current_time_ns):
    """시나리오 전체 경과 시간 (초)."""
    if start_time is None:
        return 0.0
    return (current_time_ns - start_time) / 1e9


def should_simulate_fire(simulate_fire, phase, phase_elapsed_sec, fire_delay):
    """시뮬레이션 화재 트리거 조건."""
    return (
        simulate_fire
        and phase == EXPLORING
        and phase_elapsed_sec > fire_delay
    )


def should_advance_fire_detected(phase, phase_elapsed_sec, timeout=15.0):
    """화재 감지 후 EMERGENCY 전환 조건."""
    return phase == FIRE_DETECTED and phase_elapsed_sec > timeout


def should_advance_emergency(phase, phase_elapsed_sec, timeout=10.0):
    """긴급 정지 후 RESUME 전환 조건."""
    return phase == EMERGENCY and phase_elapsed_sec > timeout


def should_advance_resume(phase, phase_elapsed_sec, timeout=10.0):
    """재개 후 LANDING 전환 조건."""
    return phase == RESUME and phase_elapsed_sec > timeout


def should_advance_landing(phase, phase_elapsed_sec, timeout=10.0):
    """착륙 대기 후 COMPLETE 전환 조건."""
    return phase == LANDING and phase_elapsed_sec > timeout


def check_wait_ready(mission_stage):
    """WAIT_READY → DRONE_TAKEOFF 전환 조건."""
    return mission_stage >= MockMissionState.STAGE_EXPLORING


def check_exploring_fire(mission_stage):
    """EXPLORING 중 실제 화재 감지 확인."""
    return mission_stage == MockMissionState.STAGE_FIRE_RESPONSE


def validate_thermal_message(max_temp_k, severity, confidence):
    """가짜 화재 메시지 유효성 검증."""
    return (
        max_temp_k > 273.15  # 0°C 이상
        and severity in ('low', 'medium', 'high', 'critical')
        and 0.0 <= confidence <= 1.0
    )


def track_service_failure(failures, success):
    """서비스 실패 추적 (S2)."""
    if not success:
        return failures + 1
    return failures


# ══════════════════════════════════════════════════
# 테스트
# ══════════════════════════════════════════════════

class TestPhaseProgression:
    """시나리오 페이즈 순서 검증."""

    def test_phase_constants_sequential(self):
        """페이즈 상수가 순차적."""
        phases = [WAIT_READY, DRONE_TAKEOFF, EXPLORING,
                  FIRE_DETECTED, EMERGENCY, RESUME, LANDING, COMPLETE]
        for i in range(len(phases) - 1):
            assert phases[i] < phases[i + 1]

    def test_phase_names_count(self):
        """PHASE_NAMES가 8단계 모두 포함."""
        assert len(PHASE_NAMES) == 8
        assert PHASE_NAMES[WAIT_READY] == 'WAIT_READY'
        assert PHASE_NAMES[COMPLETE] == 'COMPLETE'

    def test_full_scenario_sequence(self):
        """전체 시나리오 순서 시뮬레이션."""
        phase = WAIT_READY

        # Phase 0 → 1: 로봇 등록 완료
        assert check_wait_ready(MockMissionState.STAGE_EXPLORING)
        phase = DRONE_TAKEOFF

        # Phase 1 → 2: 이륙 완료 (8초 후)
        assert phase_elapsed(0, 9e9) > 8.0
        phase = EXPLORING

        # Phase 2 → 3: 화재 시뮬레이션 (30초 후)
        assert should_simulate_fire(True, EXPLORING, 31.0, 30.0)
        phase = FIRE_DETECTED

        # Phase 3 → 4: 15초 관찰 후 긴급정지
        assert should_advance_fire_detected(FIRE_DETECTED, 16.0)
        phase = EMERGENCY

        # Phase 4 → 5: 10초 유지 후 재개
        assert should_advance_emergency(EMERGENCY, 11.0)
        phase = RESUME

        # Phase 5 → 6: 10초 후 착륙
        assert should_advance_resume(RESUME, 11.0)
        phase = LANDING

        # Phase 6 → 7: 10초 후 완료
        assert should_advance_landing(LANDING, 11.0)
        phase = COMPLETE
        assert phase == COMPLETE


class TestPhaseElapsed:
    """경과 시간 계산 검증."""

    def test_phase_elapsed_none_start(self):
        """시작 시간 None → 0초."""
        assert phase_elapsed(None, 5e9) == 0.0

    def test_phase_elapsed_normal(self):
        """정상 경과 시간 계산."""
        # 1초 후
        result = phase_elapsed(0, 1e9)
        assert result == pytest.approx(1.0)

    def test_total_elapsed_none(self):
        """전체 시작 None → 0초."""
        assert total_elapsed(None, 10e9) == 0.0

    def test_total_elapsed_60s(self):
        """60초 경과."""
        start = 10e9  # 10초에 시작
        now = 70e9    # 70초
        assert total_elapsed(start, now) == pytest.approx(60.0)


class TestFireSimulation:
    """시뮬레이션 화재 트리거 + 메시지 구조."""

    def test_simulate_fire_triggers(self):
        """simulate_fire=True, EXPLORING, delay 초과 시 트리거."""
        assert should_simulate_fire(True, EXPLORING, 31.0, 30.0) is True

    def test_simulate_fire_too_early(self):
        """delay 미만이면 트리거 안 함."""
        assert should_simulate_fire(True, EXPLORING, 15.0, 30.0) is False

    def test_simulate_fire_disabled(self):
        """simulate_fire=False이면 트리거 안 함."""
        assert should_simulate_fire(False, EXPLORING, 100.0, 30.0) is False

    def test_simulate_fire_wrong_phase(self):
        """EXPLORING이 아닌 페이즈에서는 트리거 안 함."""
        assert should_simulate_fire(True, DRONE_TAKEOFF, 100.0, 30.0) is False

    def test_thermal_message_valid(self):
        """가짜 화재 메시지 유효성 검증 (300°C = 573.15K)."""
        assert validate_thermal_message(573.15, 'high', 0.95) is True

    def test_thermal_message_invalid_temp(self):
        """영하 온도는 유효하지 않음."""
        assert validate_thermal_message(200.0, 'low', 0.5) is False

    def test_thermal_message_invalid_severity(self):
        """유효하지 않은 심각도."""
        assert validate_thermal_message(500.0, 'unknown', 0.8) is False

    def test_thermal_message_invalid_confidence(self):
        """범위 초과 신뢰도."""
        assert validate_thermal_message(500.0, 'high', 1.5) is False


class TestWaitReady:
    """WAIT_READY 전환 조건."""

    def test_init_stage_stays(self):
        """INIT 단계에서는 대기."""
        assert check_wait_ready(MockMissionState.STAGE_INIT) is False

    def test_exploring_stage_advances(self):
        """EXPLORING 이상이면 진행."""
        assert check_wait_ready(MockMissionState.STAGE_EXPLORING) is True

    def test_fire_response_also_advances(self):
        """FIRE_RESPONSE도 EXPLORING보다 크므로 진행."""
        assert check_wait_ready(MockMissionState.STAGE_FIRE_RESPONSE) is True


class TestExploringFireDetection:
    """EXPLORING 중 실제 화재 감지."""

    def test_fire_detected(self):
        assert check_exploring_fire(MockMissionState.STAGE_FIRE_RESPONSE) is True

    def test_still_exploring(self):
        assert check_exploring_fire(MockMissionState.STAGE_EXPLORING) is False


class TestServiceFailureTracking:
    """S2: 서비스 실패 추적."""

    def test_failure_increments(self):
        assert track_service_failure(0, False) == 1
        assert track_service_failure(3, False) == 4

    def test_success_no_increment(self):
        assert track_service_failure(0, True) == 0
        assert track_service_failure(5, True) == 5

    def test_cumulative_failures(self):
        """연속 실패 누적."""
        failures = 0
        for _ in range(5):
            failures = track_service_failure(failures, False)
        assert failures == 5
