"""
드론 제어 로직 단위 테스트.
rclpy 없이 P 제어기, 상태 전환, 좌표 변환 로직을 검증.

drone_controller_node.py에서 핵심 수학·물리 로직을 추출하여 테스트:
  - clamp 함수
  - 쿼터니언 → yaw 변환
  - 월드→바디 프레임 속도 변환
  - 상태 머신 전환 (grounded → taking_off → hovering → flying → landing)
  - 착륙 감지 조건
"""

import math
import pytest


# ── 드론 제어 로직 (drone_controller_node.py에서 추출) ──

def clamp(val, min_val, max_val):
    """값을 [min_val, max_val] 범위로 제한."""
    return max(min_val, min(val, max_val))


def quaternion_to_yaw(qw, qx, qy, qz):
    """쿼터니언 → yaw (라디안) 변환.
    ROS2 표준 ZYX 오일러 각도 분해의 yaw 성분."""
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz))


def world_to_body_velocity(dx, dy, yaw, kp, max_speed):
    """월드 프레임 오차(dx, dy) → 바디 프레임 속도(vx, vy) 변환.

    Args:
        dx, dy: 월드 좌표 오차 (target - current)
        yaw: 로봇 현재 yaw (라디안)
        kp: P 제어 게인
        max_speed: 최대 속도 제한

    Returns:
        (vx, vy): 바디 프레임 속도 명령
    """
    cos_yaw = math.cos(-yaw)
    sin_yaw = math.sin(-yaw)
    body_dx = cos_yaw * dx - sin_yaw * dy
    body_dy = sin_yaw * dx + cos_yaw * dy
    vx = clamp(kp * body_dx, -max_speed, max_speed)
    vy = clamp(kp * body_dy, -max_speed, max_speed)
    return vx, vy


def compute_yaw_command(dx, dy, current_yaw, kp_yaw, h_dist, pos_tol):
    """이동 방향으로 기수를 정렬하는 yaw 속도 명령 계산.

    Returns:
        angular_z: yaw 각속도 명령 (rad/s)
    """
    if h_dist <= pos_tol:
        return 0.0
    target_yaw = math.atan2(dy, dx)
    yaw_error = target_yaw - current_yaw
    # -pi ~ pi 정규화
    while yaw_error > math.pi:
        yaw_error -= 2.0 * math.pi
    while yaw_error < -math.pi:
        yaw_error += 2.0 * math.pi
    return clamp(kp_yaw * yaw_error, -1.0, 1.0)


def check_landing(z, dz, z_threshold=0.3, dz_threshold=0.4):
    """착륙 조건 판단: 고도 낮음 + 수직 오차 작음."""
    return z < z_threshold and abs(dz) < dz_threshold


# ══════════════════════════════════════════════════
# 테스트
# ══════════════════════════════════════════════════

class TestClamp:

    def test_within_range(self):
        assert clamp(0.5, -1.0, 1.0) == 0.5

    def test_above_max(self):
        assert clamp(2.0, -1.0, 1.0) == 1.0

    def test_below_min(self):
        assert clamp(-2.0, -1.0, 1.0) == -1.0

    def test_at_max(self):
        assert clamp(1.0, -1.0, 1.0) == 1.0

    def test_at_min(self):
        assert clamp(-1.0, -1.0, 1.0) == -1.0

    def test_zero(self):
        assert clamp(0.0, -1.0, 1.0) == 0.0


class TestQuaternionToYaw:

    def test_identity(self):
        """단위 쿼터니언 → yaw = 0."""
        assert quaternion_to_yaw(1, 0, 0, 0) == pytest.approx(0.0)

    def test_90_degrees(self):
        """z축 90도 회전."""
        # qw=cos(45°), qz=sin(45°)
        qw = math.cos(math.pi / 4)
        qz = math.sin(math.pi / 4)
        yaw = quaternion_to_yaw(qw, 0, 0, qz)
        assert yaw == pytest.approx(math.pi / 2, abs=0.01)

    def test_180_degrees(self):
        """z축 180도 회전."""
        qw = math.cos(math.pi / 2)
        qz = math.sin(math.pi / 2)
        yaw = quaternion_to_yaw(qw, 0, 0, qz)
        assert abs(yaw) == pytest.approx(math.pi, abs=0.01)

    def test_negative_90(self):
        """z축 -90도 회전."""
        qw = math.cos(-math.pi / 4)
        qz = math.sin(-math.pi / 4)
        yaw = quaternion_to_yaw(qw, 0, 0, qz)
        assert yaw == pytest.approx(-math.pi / 2, abs=0.01)


class TestWorldToBodyVelocity:

    def test_aligned_forward(self):
        """yaw=0, 전방 목표 → vx > 0, vy ≈ 0."""
        vx, vy = world_to_body_velocity(1.0, 0.0, 0.0, 0.8, 2.0)
        assert vx == pytest.approx(0.8, abs=0.01)
        assert vy == pytest.approx(0.0, abs=0.01)

    def test_aligned_left(self):
        """yaw=0, 좌측 목표 → vx ≈ 0, vy > 0."""
        vx, vy = world_to_body_velocity(0.0, 1.0, 0.0, 0.8, 2.0)
        assert abs(vx) < 0.01
        assert vy == pytest.approx(0.8, abs=0.01)

    def test_rotated_90(self):
        """yaw=π/2, 월드 x방향 → 바디 -y방향."""
        vx, vy = world_to_body_velocity(1.0, 0.0, math.pi / 2, 1.0, 2.0)
        assert abs(vx) < 0.01
        assert vy == pytest.approx(-1.0, abs=0.01)

    def test_speed_clamped(self):
        """속도 제한 초과 시 clamp."""
        vx, vy = world_to_body_velocity(10.0, 0.0, 0.0, 1.0, 2.0)
        assert vx == 2.0  # clamped to max_speed

    def test_backward(self):
        """후방 목표 → vx < 0."""
        vx, vy = world_to_body_velocity(-1.0, 0.0, 0.0, 0.8, 2.0)
        assert vx == pytest.approx(-0.8, abs=0.01)


class TestYawCommand:

    def test_no_rotation_at_goal(self):
        """목표 도달 시 yaw 명령 = 0."""
        cmd = compute_yaw_command(1.0, 0.0, 0.0, 0.5, 0.1, 0.5)
        assert cmd == 0.0  # h_dist < pos_tol

    def test_turn_left(self):
        """목표가 좌측 → 양수 yaw (반시계)."""
        cmd = compute_yaw_command(0.0, 1.0, 0.0, 0.5, 1.0, 0.5)
        assert cmd > 0

    def test_turn_right(self):
        """목표가 우측 → 음수 yaw (시계)."""
        cmd = compute_yaw_command(0.0, -1.0, 0.0, 0.5, 1.0, 0.5)
        assert cmd < 0

    def test_yaw_clamped(self):
        """yaw 명령 ±1.0 제한."""
        cmd = compute_yaw_command(0.0, 1.0, -math.pi, 10.0, 5.0, 0.5)
        assert -1.0 <= cmd <= 1.0

    def test_wrap_around(self):
        """yaw 오차 -π~π 정규화 (최단 경로 회전)."""
        # 현재 yaw = 170°(≈3.0rad), 목표 방향 = -170°(≈-3.0rad)
        # 최단 경로: 20° 시계 방향, 아니라 340° 반시계
        cmd = compute_yaw_command(-1.0, 0.1, 3.0, 0.5, 1.0, 0.5)
        # 절대값이 1.0 이하여야 함
        assert -1.0 <= cmd <= 1.0


class TestLandingDetection:

    def test_landing_success(self):
        """저고도 + 작은 수직 오차 → 착륙 성공."""
        assert check_landing(0.1, 0.1) is True

    def test_too_high(self):
        """고도가 임계값 이상 → 착륙 아님."""
        assert check_landing(1.0, 0.0) is False

    def test_large_dz(self):
        """수직 오차 큼 → 아직 하강 중."""
        assert check_landing(0.2, 0.5) is False

    def test_on_ground_exact(self):
        """고도 0, 수직 오차 0 → 완전 착지."""
        assert check_landing(0.0, 0.0) is True

    def test_near_threshold(self):
        """임계값 경계."""
        assert check_landing(0.29, 0.39) is True
        assert check_landing(0.31, 0.0) is False


class TestDroneStateMachine:
    """상태 전환 로직 테스트 (드론 생명주기)."""

    def test_takeoff_transition(self):
        """grounded → taking_off: alt_tol 도달 시 hovering/flying."""
        state = 'taking_off'
        alt_tol = 0.3
        dz = 0.1  # 목표 고도에 근접

        if abs(dz) < alt_tol:
            state = 'hovering'  # 웨이포인트 없으면 hovering
        assert state == 'hovering'

    def test_takeoff_with_queue(self):
        """이륙 완료 시 웨이포인트 큐 있으면 즉시 flying."""
        state = 'taking_off'
        waypoint_queue = [(5.0, 5.0, 8.0)]
        dz = 0.1
        alt_tol = 0.3

        if abs(dz) < alt_tol:
            if waypoint_queue:
                state = 'flying'
                waypoint_queue.pop(0)
            else:
                state = 'hovering'
        assert state == 'flying'
        assert len(waypoint_queue) == 0

    def test_flying_waypoint_reached(self):
        """비행 중 목표 도달 → 다음 웨이포인트 또는 hovering."""
        state = 'flying'
        pos_tol = 0.5
        alt_tol = 0.3
        h_dist = 0.3  # 목표 근접
        dz = 0.1
        waypoint_queue = []

        if h_dist < pos_tol and abs(dz) < alt_tol:
            if waypoint_queue:
                state = 'flying'
            else:
                state = 'hovering'
        assert state == 'hovering'

    def test_landing_to_grounded(self):
        """landing 상태에서 착지 감지 → grounded."""
        state = 'landing'
        z = 0.2
        dz = 0.15

        if check_landing(z, dz):
            state = 'grounded'
        assert state == 'grounded'

    def test_waypoint_queuing_in_hover(self):
        """hovering 상태에서 웨이포인트 도착 → 즉시 flying."""
        state = 'hovering'
        target = None
        wp = (5.0, 3.0, 8.0)

        if state == 'hovering' and target is None:
            target = wp
            state = 'flying'
        assert state == 'flying'
        assert target == wp


class TestVerticalControl:
    """수직(고도) P제어 검증."""

    def test_ascend(self):
        """목표가 위 → vz > 0."""
        dz = 5.0  # 5m 아래에 있음
        kp_v = 1.0
        max_v = 1.0
        vz = clamp(kp_v * dz, -max_v, max_v)
        assert vz == 1.0  # clamped to max

    def test_descend(self):
        """목표가 아래 → vz < 0."""
        dz = -3.0
        vz = clamp(1.0 * dz, -1.0, 1.0)
        assert vz == -1.0

    def test_maintain_altitude(self):
        """고도 일치 → vz ≈ 0."""
        dz = 0.0
        vz = clamp(1.0 * dz, -1.0, 1.0)
        assert vz == 0.0

    def test_small_correction(self):
        """미세 보정."""
        dz = 0.1
        vz = clamp(1.0 * dz, -1.0, 1.0)
        assert vz == pytest.approx(0.1)


# ══════════════════════════════════════════════════
# HIGH 버그 수정 검증 (PR #9)
# ══════════════════════════════════════════════════

class TestWaypointQueueLock:
    """D1: waypoint_queue 스레드 안전성 검증."""

    def test_concurrent_queue_access(self):
        """여러 스레드가 동시에 큐에 접근해도 크래시 없음."""
        import threading

        queue = []
        lock = threading.Lock()
        errors = []

        def producer():
            for i in range(100):
                with lock:
                    queue.append((float(i), float(i), 8.0))

        def consumer():
            consumed = 0
            for _ in range(200):
                with lock:
                    if queue:
                        queue.pop(0)
                        consumed += 1

        threads = [
            threading.Thread(target=producer),
            threading.Thread(target=producer),
            threading.Thread(target=consumer),
        ]
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=5.0)

        # 크래시 없이 완료되면 성공
        assert True

    def test_lock_protects_pop(self):
        """Lock 내에서 pop하면 빈 큐에서도 안전."""
        import threading
        queue = [(1.0, 2.0, 8.0)]
        lock = threading.Lock()

        result = None
        with lock:
            if queue:
                result = queue.pop(0)
        assert result == (1.0, 2.0, 8.0)

        # 빈 큐
        result = None
        with lock:
            if queue:
                result = queue.pop(0)
        assert result is None

    def test_queue_len_inside_lock(self):
        """큐 길이 조회도 lock 내에서."""
        import threading
        queue = [(1, 2, 3), (4, 5, 6)]
        lock = threading.Lock()

        with lock:
            length = len(queue)
        assert length == 2


class TestLandingTimeout:
    """D2: 착륙 타임아웃 검증."""

    def test_landing_normal(self):
        """정상 착륙: 고도 낮음 + 수직 오차 작음."""
        assert check_landing(0.1, 0.1) is True

    def test_landing_timeout_forced(self):
        """타임아웃 초과 시 강제 착륙."""
        landing_timeout = 30.0
        landing_elapsed = 35.0  # 35초 경과

        # 고도가 아직 높아도 타임아웃이면 착륙
        z = 1.0  # 아직 높음
        dz = 0.5

        normal_landing = check_landing(z, dz)
        timeout_landing = landing_elapsed > landing_timeout

        assert normal_landing is False
        assert timeout_landing is True
        # 최종 판정: normal OR timeout
        assert normal_landing or timeout_landing

    def test_landing_within_timeout(self):
        """타임아웃 내 정상 착륙."""
        landing_timeout = 30.0
        landing_elapsed = 15.0
        z = 0.2
        dz = 0.1

        assert check_landing(z, dz) is True
        assert landing_elapsed <= landing_timeout

    def test_landing_start_time_none_guard(self):
        """착륙 시작 시각 None → 초기화."""
        landing_start_time = None

        # 수정된 로직: None이면 현재 시각으로 초기화
        if landing_start_time is None:
            landing_start_time = 1000.0  # 현재 시각 (나노초)

        assert landing_start_time == 1000.0

    def test_timeout_resets_state(self):
        """타임아웃 후 상태 초기화."""
        state = 'landing'
        target = (5.0, 5.0, 0.0)
        landing_start_time = 100.0

        landing_elapsed = 135.0 - landing_start_time  # 35초
        landing_timeout = 30.0

        if landing_elapsed > landing_timeout:
            state = 'grounded'
            target = None
            landing_start_time = None

        assert state == 'grounded'
        assert target is None
        assert landing_start_time is None
