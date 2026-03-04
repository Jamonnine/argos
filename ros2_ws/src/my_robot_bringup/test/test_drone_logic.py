"""
Drone Controller 핵심 로직 단위 테스트.
rclpy 없이 P제어, 좌표 변환, 상태 전환 로직 검증.
"""

import pytest
import math


# ── 드론 제어 로직 (drone_controller_node.py에서 추출) ──

def clamp(val, min_val, max_val):
    """값을 [min_val, max_val] 범위로 제한."""
    return max(min_val, min(val, max_val))


def compute_velocity_command(current_pose, target_waypoint,
                             kp_h=0.8, kp_v=1.0, kp_yaw=0.5,
                             max_h_speed=2.0, max_v_speed=1.0,
                             pos_tol=0.5):
    """P 제어기 기반 속도 명령 계산.
    current_pose: (x, y, z, yaw)
    target_waypoint: (tx, ty, tz)
    Returns: (vx, vy, vz, wz) — 바디 프레임 속도
    """
    x, y, z, yaw = current_pose
    tx, ty, tz = target_waypoint

    dx = tx - x
    dy = ty - y
    dz = tz - z
    h_dist = math.hypot(dx, dy)

    # 수직 제어
    vz = clamp(kp_v * dz, -max_v_speed, max_v_speed)

    # 수평 제어 (월드 → 바디 프레임)
    cos_yaw = math.cos(-yaw)
    sin_yaw = math.sin(-yaw)
    body_dx = cos_yaw * dx - sin_yaw * dy
    body_dy = sin_yaw * dx + cos_yaw * dy

    vx = clamp(kp_h * body_dx, -max_h_speed, max_h_speed)
    vy = clamp(kp_h * body_dy, -max_h_speed, max_h_speed)

    # Yaw 제어
    wz = 0.0
    if h_dist > pos_tol:
        target_yaw = math.atan2(dy, dx)
        yaw_error = target_yaw - yaw
        while yaw_error > math.pi:
            yaw_error -= 2.0 * math.pi
        while yaw_error < -math.pi:
            yaw_error += 2.0 * math.pi
        wz = clamp(kp_yaw * yaw_error, -1.0, 1.0)

    return (vx, vy, vz, wz)


def quaternion_to_yaw(qw, qx, qy, qz):
    """쿼터니언 → yaw 변환."""
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz))


def check_waypoint_reached(current_pose, target_waypoint,
                           pos_tol=0.5, alt_tol=0.3):
    """웨이포인트 도달 판정."""
    x, y, z, _ = current_pose
    tx, ty, tz = target_waypoint
    h_dist = math.hypot(tx - x, ty - y)
    v_dist = abs(tz - z)
    return h_dist < pos_tol and v_dist < alt_tol


def check_landing_complete(z, dz, ground_threshold=0.3, dz_threshold=0.4):
    """착륙 완료 판정."""
    return z < ground_threshold and abs(dz) < dz_threshold


class DroneStateMachine:
    """드론 상태 머신 (노드 의존성 없는 순수 로직)."""

    def __init__(self, cruise_alt=8.0, pos_tol=0.5, alt_tol=0.3):
        self.state = 'grounded'
        self.cruise_alt = cruise_alt
        self.pos_tol = pos_tol
        self.alt_tol = alt_tol
        self.target_waypoint = None
        self.waypoint_queue = []

    def takeoff(self, current_x, current_y):
        """이륙 요청."""
        if self.state != 'grounded':
            return False
        self.state = 'taking_off'
        self.target_waypoint = (current_x, current_y, self.cruise_alt)
        return True

    def land(self, current_x, current_y):
        """착륙 요청."""
        if self.state not in ('hovering', 'flying'):
            return False
        self.state = 'landing'
        self.target_waypoint = (current_x, current_y, 0.0)
        return True

    def add_waypoint(self, wp):
        """웨이포인트 추가 (hovering이면 즉시 출발)."""
        self.waypoint_queue.append(wp)
        if self.state == 'hovering' and self.target_waypoint is None:
            self.target_waypoint = self.waypoint_queue.pop(0)
            self.state = 'flying'

    def update(self, current_pose):
        """상태 전환 업데이트."""
        x, y, z, _ = current_pose
        if self.target_waypoint is None:
            return

        tx, ty, tz = self.target_waypoint
        h_dist = math.hypot(tx - x, ty - y)
        dz = tz - z

        if self.state == 'taking_off':
            if abs(dz) < self.alt_tol:
                if self.waypoint_queue:
                    self.target_waypoint = self.waypoint_queue.pop(0)
                    self.state = 'flying'
                else:
                    self.state = 'hovering'
                    self.target_waypoint = None

        elif self.state == 'flying':
            if h_dist < self.pos_tol and abs(dz) < self.alt_tol:
                if self.waypoint_queue:
                    self.target_waypoint = self.waypoint_queue.pop(0)
                else:
                    self.state = 'hovering'
                    self.target_waypoint = None

        elif self.state == 'landing':
            if z < 0.3 and abs(dz) < 0.4:
                self.state = 'grounded'
                self.target_waypoint = None


# ══════════════════════════════════════════════════
# 테스트
# ══════════════════════════════════════════════════

class TestClamp:

    def test_within_range(self):
        assert clamp(5.0, 0.0, 10.0) == 5.0

    def test_below_min(self):
        assert clamp(-5.0, 0.0, 10.0) == 0.0

    def test_above_max(self):
        assert clamp(15.0, 0.0, 10.0) == 10.0

    def test_at_boundary(self):
        assert clamp(0.0, 0.0, 10.0) == 0.0
        assert clamp(10.0, 0.0, 10.0) == 10.0


class TestVelocityCommand:

    def test_stationary_at_target(self):
        """목표 지점에 있으면 속도 ≈ 0."""
        pose = (5.0, 5.0, 8.0, 0.0)
        target = (5.0, 5.0, 8.0)
        vx, vy, vz, wz = compute_velocity_command(pose, target)
        assert abs(vx) < 0.01
        assert abs(vy) < 0.01
        assert abs(vz) < 0.01

    def test_vertical_ascend(self):
        """수직 상승 → vz > 0."""
        pose = (0.0, 0.0, 0.0, 0.0)
        target = (0.0, 0.0, 8.0)
        vx, vy, vz, wz = compute_velocity_command(pose, target)
        assert vz > 0
        assert vz <= 1.0  # max_v_speed

    def test_vertical_descend(self):
        """수직 하강 → vz < 0."""
        pose = (0.0, 0.0, 8.0, 0.0)
        target = (0.0, 0.0, 0.0)
        vx, vy, vz, wz = compute_velocity_command(pose, target)
        assert vz < 0

    def test_forward_movement_yaw_zero(self):
        """yaw=0, 전방 목표 → vx > 0."""
        pose = (0.0, 0.0, 8.0, 0.0)
        target = (10.0, 0.0, 8.0)
        vx, vy, vz, wz = compute_velocity_command(pose, target)
        assert vx > 0

    def test_body_frame_rotation(self):
        """yaw=π/2(왼쪽 90도) 회전 시 월드 x축 목표 → 바디 프레임 변환."""
        pose = (0.0, 0.0, 8.0, math.pi / 2)
        target = (10.0, 0.0, 8.0)
        vx, vy, vz, wz = compute_velocity_command(pose, target)
        # yaw=π/2이면 월드 x+ → 바디 y-
        assert vy < 0

    def test_speed_clamping(self):
        """매우 먼 목표 → 속도 max 제한."""
        pose = (0.0, 0.0, 0.0, 0.0)
        target = (1000.0, 0.0, 1000.0)
        vx, vy, vz, wz = compute_velocity_command(
            pose, target, max_h_speed=2.0, max_v_speed=1.0)
        assert abs(vx) <= 2.0
        assert abs(vy) <= 2.0
        assert abs(vz) <= 1.0


class TestQuaternionToYaw:

    def test_identity(self):
        """단위 쿼터니언 → yaw=0."""
        yaw = quaternion_to_yaw(1.0, 0.0, 0.0, 0.0)
        assert yaw == pytest.approx(0.0)

    def test_90_degrees(self):
        """z축 90도 회전."""
        # qw=cos(45°), qz=sin(45°) for 90° rotation around z
        qw = math.cos(math.pi / 4)
        qz = math.sin(math.pi / 4)
        yaw = quaternion_to_yaw(qw, 0.0, 0.0, qz)
        assert yaw == pytest.approx(math.pi / 2, abs=0.01)

    def test_180_degrees(self):
        """z축 180도 회전."""
        qw = math.cos(math.pi / 2)
        qz = math.sin(math.pi / 2)
        yaw = quaternion_to_yaw(qw, 0.0, 0.0, qz)
        assert abs(yaw) == pytest.approx(math.pi, abs=0.01)


class TestWaypointReached:

    def test_at_waypoint(self):
        pose = (5.0, 5.0, 8.0, 0.0)
        wp = (5.0, 5.0, 8.0)
        assert check_waypoint_reached(pose, wp) is True

    def test_close_enough(self):
        pose = (5.2, 5.3, 8.1, 0.0)
        wp = (5.0, 5.0, 8.0)
        assert check_waypoint_reached(pose, wp, pos_tol=0.5, alt_tol=0.3) is True

    def test_too_far_horizontal(self):
        pose = (0.0, 0.0, 8.0, 0.0)
        wp = (5.0, 5.0, 8.0)
        assert check_waypoint_reached(pose, wp) is False

    def test_too_far_vertical(self):
        pose = (5.0, 5.0, 5.0, 0.0)
        wp = (5.0, 5.0, 8.0)
        assert check_waypoint_reached(pose, wp) is False


class TestLandingComplete:

    def test_on_ground(self):
        assert check_landing_complete(0.1, 0.1) is True

    def test_still_descending(self):
        assert check_landing_complete(2.0, -2.0) is False

    def test_near_ground_high_dz(self):
        """고도 낮지만 수직 오차 큼 → 미완료."""
        assert check_landing_complete(0.2, 1.0) is False


class TestDroneStateMachine:

    def test_initial_state(self):
        sm = DroneStateMachine()
        assert sm.state == 'grounded'

    def test_takeoff_from_grounded(self):
        sm = DroneStateMachine(cruise_alt=8.0)
        assert sm.takeoff(0.0, 0.0) is True
        assert sm.state == 'taking_off'
        assert sm.target_waypoint == (0.0, 0.0, 8.0)

    def test_takeoff_fails_while_flying(self):
        sm = DroneStateMachine()
        sm.state = 'flying'
        assert sm.takeoff(0.0, 0.0) is False

    def test_taking_off_to_hovering(self):
        """고도 도달 시 hovering으로 전환."""
        sm = DroneStateMachine(cruise_alt=8.0, alt_tol=0.3)
        sm.takeoff(0.0, 0.0)
        sm.update((0.0, 0.0, 7.9, 0.0))  # 고도 7.9 → |dz| = 0.1 < 0.3
        assert sm.state == 'hovering'

    def test_takeoff_to_flying_with_queued_wp(self):
        """이륙 + 큐에 웨이포인트 → 도달 시 바로 flying."""
        sm = DroneStateMachine(cruise_alt=8.0, alt_tol=0.3)
        sm.takeoff(0.0, 0.0)
        sm.waypoint_queue.append((10.0, 10.0, 8.0))
        sm.update((0.0, 0.0, 8.0, 0.0))  # 고도 도달
        assert sm.state == 'flying'
        assert sm.target_waypoint == (10.0, 10.0, 8.0)

    def test_land_from_hovering(self):
        sm = DroneStateMachine()
        sm.state = 'hovering'
        assert sm.land(5.0, 5.0) is True
        assert sm.state == 'landing'
        assert sm.target_waypoint == (5.0, 5.0, 0.0)

    def test_land_fails_from_grounded(self):
        sm = DroneStateMachine()
        assert sm.land(0.0, 0.0) is False

    def test_landing_to_grounded(self):
        """착륙 판정."""
        sm = DroneStateMachine()
        sm.state = 'landing'
        sm.target_waypoint = (5.0, 5.0, 0.0)
        sm.update((5.0, 5.0, 0.1, 0.0))  # z=0.1 < 0.3
        assert sm.state == 'grounded'
        assert sm.target_waypoint is None

    def test_waypoint_queue_auto_fly(self):
        """hovering + 웨이포인트 추가 → 자동 flying."""
        sm = DroneStateMachine()
        sm.state = 'hovering'
        sm.target_waypoint = None
        sm.add_waypoint((10.0, 0.0, 8.0))
        assert sm.state == 'flying'
        assert sm.target_waypoint == (10.0, 0.0, 8.0)

    def test_waypoint_chain(self):
        """웨이포인트 연속 비행."""
        sm = DroneStateMachine(pos_tol=0.5, alt_tol=0.3)
        sm.state = 'flying'
        sm.target_waypoint = (5.0, 5.0, 8.0)
        sm.waypoint_queue = [(10.0, 10.0, 8.0)]

        # 첫 번째 웨이포인트 도달
        sm.update((5.0, 5.0, 8.0, 0.0))
        assert sm.state == 'flying'
        assert sm.target_waypoint == (10.0, 10.0, 8.0)

        # 두 번째 웨이포인트 도달 → hovering
        sm.update((10.0, 10.0, 8.0, 0.0))
        assert sm.state == 'hovering'

    def test_full_flight_cycle(self):
        """전체 비행 주기: grounded → takeoff → fly → hover → land → grounded."""
        sm = DroneStateMachine(cruise_alt=8.0, pos_tol=0.5, alt_tol=0.3)

        # 이륙
        sm.takeoff(0.0, 0.0)
        assert sm.state == 'taking_off'

        # 고도 도달
        sm.update((0.0, 0.0, 8.0, 0.0))
        assert sm.state == 'hovering'

        # 웨이포인트 비행
        sm.add_waypoint((10.0, 0.0, 8.0))
        assert sm.state == 'flying'

        # 도착
        sm.update((10.0, 0.0, 8.0, 0.0))
        assert sm.state == 'hovering'

        # 착륙
        sm.land(10.0, 0.0)
        assert sm.state == 'landing'

        # 지면 도달
        sm.update((10.0, 0.0, 0.1, 0.0))
        assert sm.state == 'grounded'
