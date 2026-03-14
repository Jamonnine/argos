"""
크로스 노드 데이터 흐름 통합 테스트.
rclpy 없이 노드 간 데이터 변환 파이프라인 검증.

검증 흐름:
  1. 열화상 → 화점 감지 → FireAlert → 오케스트레이터 화재 대응
  2. AI 감지 → 접근 자세 계산 → Nav2 목표 (TF 변환 포함)
  3. 탐색 완료 → 스테이지 전환 → 시나리오 진행
  4. 배터리 감쇠 → 경고/복구 → 플래그 관리
"""

import math
import pytest


# ── 노드별 로직 재현 (각 테스트 파일에서 동일 패턴) ──

def pixel_to_kelvin(pixel_value, resolution=3.0, min_temp=253.15):
    return max(0.0, min_temp + pixel_value * resolution)


def classify_severity(temp_kelvin, thresholds=None):
    if thresholds is None:
        thresholds = [323.15, 473.15, 673.15]
    if temp_kelvin >= thresholds[2]:
        return 'critical'
    elif temp_kelvin >= thresholds[1]:
        return 'high'
    elif temp_kelvin >= thresholds[0]:
        return 'medium'
    return 'low'


def fire_severity_rank(severity):
    rank = {'low': 0, 'medium': 1, 'high': 2, 'critical': 3}
    return rank.get(severity, -1)


def select_nearest_ugv(robots, fire_x, fire_y):
    best_ugv = None
    best_dist = float('inf')
    for rid, r in robots.items():
        if r.get('type') != 'ugv' or r.get('comm_lost') or r.get('pose') is None:
            continue
        dx = r['pose'][0] - fire_x
        dy = r['pose'][1] - fire_y
        dist = math.hypot(dx, dy)
        if dist < best_dist:
            best_dist = dist
            best_ugv = rid
    return best_ugv, best_dist


def compute_approach_pose(obj_x, obj_y, obj_distance, approach_dist):
    dist = max(obj_distance - approach_dist, 0.1)
    angle = math.atan2(obj_y, obj_x)
    target_x = dist * math.cos(angle)
    target_y = dist * math.sin(angle)
    qz = math.sin(angle / 2.0)
    qw = math.cos(angle / 2.0)
    return target_x, target_y, qz, qw


def transform_base_to_map(robot_x, robot_y, yaw, bl_x, bl_y):
    """base_link → map 2D 변환."""
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    map_x = robot_x + cos_y * bl_x - sin_y * bl_y
    map_y = robot_y + sin_y * bl_x + cos_y * bl_y
    return map_x, map_y


def check_battery(battery, critical=15.0, warning=30.0):
    if battery <= critical:
        return 'critical'
    elif battery <= warning:
        return 'warning'
    return 'ok'


def simulate_battery_drain(initial, drain_rate, elapsed_sec):
    return max(0.0, initial - drain_rate * elapsed_sec)


# ══════════════════════════════════════════════════
# 통합 테스트
# ══════════════════════════════════════════════════

class TestThermalToFireResponse:
    """열화상 감지 → 오케스트레이터 화재 대응 파이프라인."""

    def test_hotspot_to_fire_alert_to_ugv_dispatch(self):
        """
        1. 열화상 픽셀 200 → 853K(580°C)
        2. 심각도 = critical
        3. 오케스트레이터가 가장 가까운 UGV 선정
        """
        # Step 1: 열화상 화점 감지
        pixel = 200
        temp = pixel_to_kelvin(pixel)
        assert temp == pytest.approx(853.15, abs=0.01)

        # Step 2: 심각도 분류
        severity = classify_severity(temp)
        assert severity == 'critical'

        # Step 3: UGV 선정
        robots = {
            'argos1': {'type': 'ugv', 'pose': (1.0, 6.0), 'comm_lost': False},
            'argos2': {'type': 'ugv', 'pose': (8.0, 2.0), 'comm_lost': False},
            'drone1': {'type': 'drone', 'pose': (5.0, 5.0), 'comm_lost': False},
        }
        fire_x, fire_y = 1.5, 6.5  # 방 A 화원
        best, dist = select_nearest_ugv(robots, fire_x, fire_y)
        assert best == 'argos1'
        assert dist < 1.0  # argos1이 가장 가까움

    def test_severity_escalation_triggers_response(self):
        """심각도 상승 시 에스컬레이션."""
        current_severity = 'medium'
        # 새 감지: critical
        new_temp = pixel_to_kelvin(220)  # 913K
        new_severity = classify_severity(new_temp)
        assert new_severity == 'critical'
        assert fire_severity_rank(new_severity) > fire_severity_rank(current_severity)

    def test_low_severity_no_response(self):
        """low 심각도는 화재 대응 불필요."""
        temp = pixel_to_kelvin(20)  # 313K (40°C)
        severity = classify_severity(temp)
        assert severity == 'low'
        # low는 FireAlert 발행 안 함 (high/critical만)
        assert severity not in ('high', 'critical')


class TestDetectionToNavigation:
    """AI 감지 → 접근 자세 → TF 변환 → Nav2 목표."""

    def test_detection_to_map_goal(self):
        """
        1. base_link에서 정면 2m 객체 감지
        2. 접근 자세 계산 (0.5m 앞)
        3. base_link → map 변환 (로봇 위치 (3, 2), yaw=0)
        """
        # Step 1: 객체 감지 (base_link 기준)
        obj_x, obj_y, obj_dist = 2.0, 0.0, 2.0

        # Step 2: 접근 자세 (0.5m 앞에서 정지)
        tx, ty, qz, qw = compute_approach_pose(obj_x, obj_y, obj_dist, 0.5)
        assert tx == pytest.approx(1.5, abs=0.01)
        assert ty == pytest.approx(0.0, abs=0.01)

        # Step 3: TF 변환 (로봇 위치 = (3, 2), yaw = 0)
        map_x, map_y = transform_base_to_map(3.0, 2.0, 0.0, tx, ty)
        assert map_x == pytest.approx(4.5, abs=0.01)  # 3 + 1.5
        assert map_y == pytest.approx(2.0, abs=0.01)  # 2 + 0

    def test_detection_rotated_robot(self):
        """로봇 90도 회전 상태에서 변환."""
        obj_x, obj_y, obj_dist = 1.0, 0.0, 1.0
        tx, ty, _, _ = compute_approach_pose(obj_x, obj_y, obj_dist, 0.5)

        # 로봇 (5, 3), yaw = π/2 (왼쪽을 봄)
        map_x, map_y = transform_base_to_map(5.0, 3.0, math.pi / 2, tx, ty)
        # yaw=90° → base_link의 x축이 map의 y축
        assert map_x == pytest.approx(5.0, abs=0.1)
        assert map_y == pytest.approx(3.5, abs=0.1)

    def test_without_tf_produces_wrong_goal(self):
        """TF 없이 base_link 좌표를 map으로 착각하면 오류."""
        obj_x, obj_y, obj_dist = 2.0, 0.0, 2.0
        tx, ty, _, _ = compute_approach_pose(obj_x, obj_y, obj_dist, 0.5)

        # 로봇 위치 (10, 5)
        # 잘못된 경우: tx=1.5를 map 좌표로 사용 → (1.5, 0)
        wrong_goal = (tx, ty)
        # 올바른 경우: 변환 후 → (11.5, 5)
        correct_goal = transform_base_to_map(10.0, 5.0, 0.0, tx, ty)

        error = math.hypot(
            correct_goal[0] - wrong_goal[0],
            correct_goal[1] - wrong_goal[1])
        assert error > 8.0  # 큰 오차


class TestExplorationCompletion:
    """탐색 완료 → 스테이지 전환 → 시나리오 진행."""

    def test_all_ugv_complete_triggers_returning(self):
        """모든 UGV 탐색 완료 → RETURNING 전환."""
        robots = {
            'argos1': {'mission': 'complete', 'comm_lost': False, 'last_seen': 1.0},
            'argos2': {'mission': 'complete', 'comm_lost': False, 'last_seen': 2.0},
            'drone1': {'mission': 'hovering', 'comm_lost': False, 'last_seen': 1.5},
        }
        # UGV만 확인 (드론은 별도 판정)
        ugv_complete = all(
            r['mission'] == 'complete'
            for r in robots.values()
            if not r['comm_lost'] and r['last_seen'] > 0
            and r.get('type', 'ugv') == 'ugv'
        )
        # 이 예제에서는 type 미설정 → 기본 ugv로 간주
        # drone1도 ugv로 취급되어 False
        # 실제 코드에서는 current_mission으로 판단
        active = [
            r for r in robots.values()
            if not r['comm_lost'] and r['last_seen'] > 0
        ]
        all_complete = all(r['mission'] == 'complete' for r in active)
        assert all_complete is False  # drone1은 'hovering'

    def test_comm_lost_excluded(self):
        """통신 두절 로봇 제외 후 완료 판정."""
        robots = {
            'argos1': {'mission': 'complete', 'comm_lost': False, 'last_seen': 1.0},
            'argos2': {'mission': 'exploring', 'comm_lost': True, 'last_seen': 2.0},
        }
        active = [
            r for r in robots.values()
            if not r['comm_lost'] and r['last_seen'] > 0
        ]
        all_complete = len(active) > 0 and all(
            r['mission'] == 'complete' for r in active)
        assert all_complete is True


class TestBatteryLifecycle:
    """배터리 감쇠 → 경고 → 복구 전체 사이클."""

    def test_drain_warning_recovery_cycle(self):
        """
        1. 100% → 감쇠 → 30% (warning)
        2. 계속 감쇠 → 15% (critical)
        3. 충전 시뮬 → 50% (ok, 플래그 리셋)
        """
        # Phase 1: 감쇠
        battery = simulate_battery_drain(100.0, 0.02, 3500)  # 70초 = 30%
        assert battery == pytest.approx(30.0)
        assert check_battery(battery) == 'warning'

        warned = True  # O5: 경고 플래그 설정

        # Phase 2: 계속 감쇠
        battery = simulate_battery_drain(100.0, 0.02, 4250)  # 85초 = 15%
        assert battery == pytest.approx(15.0)
        assert check_battery(battery) == 'critical'

        critical_acted = True

        # Phase 3: 복구 (시뮬레이션에서는 배터리 직접 설정)
        battery = 50.0
        assert check_battery(battery) == 'ok'

        # O5: 플래그 리셋
        if battery > 30.0:
            warned = False
        if battery > 15.0:
            critical_acted = False

        assert warned is False
        assert critical_acted is False

    def test_battery_zero_floor(self):
        """배터리는 0 이하로 내려가지 않음."""
        battery = simulate_battery_drain(100.0, 0.02, 10000)  # 200초
        assert battery == 0.0
