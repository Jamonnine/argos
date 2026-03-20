"""
hose_aware_planner.py — Hose-Aware Path Planner (Nav2 경로 필터)
=================================================================
Nav2가 생성한 경로(/{robot_id}/plan)를 받아 소방호스 제약을 검증·수정한다.

소방호스 제약:
  1. 총 경로 길이 > 호스 잔여: 잔여 길이의 90%까지 트림
  2. 급커브 구간: 연속 3점 외접원 반경 < min_bend_radius → 경로 스무딩
  3. 충수 상태 후진 구간: 발견 즉시 거부 (수정 불가)

역할:
  - 경로 "필터링"만 수행. Nav2를 직접 제어하지 않음.
  - 오케스트레이터가 hose_filtered_plan을 참조하여 goal 승인/거부 결정.

토픽:
  구독: /{robot_id}/plan               (nav_msgs/Path) — Nav2 플래너 출력
        /{robot_id}/hose/status         (Float32MultiArray)
          data[0]: 잔여 길이 (m, 0~100)
          data[1]: 꺾임 위험도 (0.0~1.0)  ← 현재 미사용 (경로 기반 검출로 대체)
          data[2]: 충수 여부 (0.0=건수, 1.0=충수)
  발행: /{robot_id}/hose_filtered_plan  (nav_msgs/Path) — 수정된 경로
        /{robot_id}/hose_planner/status (String)        — 검증 결과 메시지

파라미터:
  robot_id                (str)   : 로봇 식별자 (기본: "sherpa1")
  min_bend_radius         (float) : 최소 굽힘 반경 m (기본: 0.5)
  path_trim_safety_factor (float) : 잔여 길이 대비 트림 비율 (기본: 0.9)
  sharp_turn_threshold_deg(float) : 급커브 판단 각도 (기본: 90.0도)
"""

import math

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, State
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray, String


class HoseAwarePlanner(LifecycleNode):
    """소방호스 제약 기반 Nav2 경로 필터 노드.

    Nav2 플래너가 출력한 경로(nav_msgs/Path)를 3가지 제약으로 검증하고,
    필요 시 수정된 경로를 hose_filtered_plan으로 발행한다.
    """

    def __init__(self):
        super().__init__('hose_aware_planner')

        # 파라미터 선언 (activate 전에만)
        self.declare_parameter('robot_id', 'sherpa1')
        self.declare_parameter('min_bend_radius', 0.5)
        self.declare_parameter('path_trim_safety_factor', 0.9)
        self.declare_parameter('sharp_turn_threshold_deg', 90.0)

        # 런타임 상태 (on_configure에서 초기화)
        self.robot_id: str = ''
        self.min_bend_radius: float = 0.5
        self.path_trim_safety_factor: float = 0.9
        self.sharp_turn_threshold_deg: float = 90.0

        # 호스 상태 (토픽 수신 시 갱신)
        self.hose_remaining: float = 100.0   # m
        self.hose_charged: bool = False      # 충수 여부
        self._hose_received: bool = False    # 토픽 수신 여부

        # 핸들 (on_activate에서 생성, on_deactivate에서 해제)
        self._plan_sub = None
        self._hose_sub = None
        self._filtered_pub = None
        self._status_pub = None
        self._cb_group = None

    # ─────────────────── Lifecycle ───────────────────

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """파라미터 읽기 + 내부 상태 초기화."""
        self.robot_id = self.get_parameter('robot_id').value
        self.min_bend_radius = self.get_parameter('min_bend_radius').value
        self.path_trim_safety_factor = self.get_parameter('path_trim_safety_factor').value
        self.sharp_turn_threshold_deg = self.get_parameter('sharp_turn_threshold_deg').value

        self.hose_remaining = 100.0
        self.hose_charged = False
        self._hose_received = False

        self.get_logger().info(
            f'HoseAwarePlanner configured '
            f'(robot={self.robot_id}, min_bend_radius={self.min_bend_radius}m, '
            f'sharp_turn_threshold={self.sharp_turn_threshold_deg}deg)'
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """구독/발행 생성."""
        self._cb_group = ReentrantCallbackGroup()

        # nav_msgs/Path는 TRANSIENT_LOCAL이 아닌 일반 QoS 사용 (플래너 출력 스트림)
        plan_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10,
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=1,
        )

        # 경로 구독
        self._plan_sub = self.create_subscription(
            Path,
            f'/{self.robot_id}/plan',
            self._plan_callback,
            plan_qos,
            callback_group=self._cb_group,
        )

        # 호스 상태 구독
        self._hose_sub = self.create_subscription(
            Float32MultiArray,
            f'/{self.robot_id}/hose/status',
            self._hose_callback,
            sensor_qos,
            callback_group=self._cb_group,
        )

        # 수정된 경로 발행
        self._filtered_pub = self.create_publisher(
            Path,
            f'/{self.robot_id}/hose_filtered_plan',
            10,
        )

        # 검증 결과 상태 발행
        self._status_pub = self.create_publisher(
            String,
            f'/{self.robot_id}/hose_planner/status',
            10,
        )

        self.get_logger().info(
            f'HoseAwarePlanner activated — 구독: /{self.robot_id}/plan'
        )
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """구독/발행 해제."""
        if self._plan_sub:
            self.destroy_subscription(self._plan_sub)
            self._plan_sub = None
        if self._hose_sub:
            self.destroy_subscription(self._hose_sub)
            self._hose_sub = None
        if self._filtered_pub:
            self.destroy_publisher(self._filtered_pub)
            self._filtered_pub = None
        if self._status_pub:
            self.destroy_publisher(self._status_pub)
            self._status_pub = None

        self.get_logger().info('HoseAwarePlanner deactivated')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """내부 상태 리셋."""
        self.hose_remaining = 100.0
        self.hose_charged = False
        self._hose_received = False
        self._cb_group = None
        self.get_logger().info('HoseAwarePlanner cleaned up')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('HoseAwarePlanner shutting down')
        return TransitionCallbackReturn.SUCCESS

    # ─────────────────── Callbacks ───────────────────

    def _hose_callback(self, msg: Float32MultiArray) -> None:
        """/hose/status 수신 처리.

        data[0]: 잔여 길이 (m)
        data[2]: 충수 여부 (0.0=건수, 1.0=충수)
        """
        data = msg.data
        if len(data) >= 1:
            self.hose_remaining = float(data[0])
        if len(data) >= 3:
            self.hose_charged = bool(data[2] >= 0.5)
        self._hose_received = True

    def _plan_callback(self, msg: Path) -> None:
        """Nav2 경로 수신 시 호스 제약 검증 + 필터링 경로 발행."""
        if not self._hose_received:
            # 호스 상태 미수신 → 경고 후 원본 그대로 통과 (graceful)
            self.get_logger().warn(
                f'[{self.robot_id}] /hose/status 미수신 — 경로 필터링 우회',
                throttle_duration_sec=30.0,
            )
            self._publish(msg, 'hose_status_unknown — pass_through')
            return

        passed, reason, output_path = self.validate_path(msg)

        if passed:
            self.get_logger().debug(f'[{self.robot_id}] 경로 검증 통과')
        else:
            self.get_logger().warn(f'[{self.robot_id}] 경로 수정: {reason}')

        self._publish(output_path, reason)

    def _publish(self, path: Path, reason: str) -> None:
        """수정된 경로와 상태 메시지를 발행."""
        self._filtered_pub.publish(path)
        status_msg = String()
        status_msg.data = reason
        self._status_pub.publish(status_msg)

    # ─────────────────── 경로 검증 (공개 API) ───────────────────

    def validate_path(self, path: Path) -> tuple:
        """경로 검증 + 수정.

        Args:
            path: Nav2 플래너 출력 경로 (nav_msgs/Path)

        Returns:
            (통과 여부, 사유 문자열, 수정된 경로)
            통과: (True, 'OK', 원본 경로)
            수정: (False, 사유, 수정된 경로)
            거부: (False, 사유, 빈/원본 경로 — 오케스트레이터가 거부 처리)
        """
        if len(path.poses) < 2:
            # 점이 1개 이하면 검증 불가 → 통과
            return True, 'OK', path

        # ── 제약 1: 총 경로 길이 vs 호스 잔여 ──────────────────────────────
        total_length = self._calc_path_length(path)
        if total_length > self.hose_remaining:
            safe_limit = self.hose_remaining * self.path_trim_safety_factor
            trimmed = self._trim_path_to_length(path, safe_limit)
            return (
                False,
                f'경로 {total_length:.1f}m > 호스 {self.hose_remaining:.1f}m '
                f'— {safe_limit:.1f}m로 트림',
                trimmed,
            )

        # ── 제약 2: 급커브 구간 검출 + 스무딩 ──────────────────────────────
        sharp_turns = self._find_sharp_turns(path, self.min_bend_radius)
        if sharp_turns:
            smoothed = self._smooth_sharp_turns(path, sharp_turns)
            return (
                False,
                f'급커브 {len(sharp_turns)}곳 완화',
                smoothed,
            )

        # ── 제약 3: 충수 상태 후진 구간 검출 ────────────────────────────────
        if self.hose_charged:
            reverse_segments = self._find_reverse_segments(path)
            if reverse_segments:
                return (
                    False,
                    f'충수 후진 구간 {len(reverse_segments)}곳 감지 — 경로 거부',
                    path,  # 수정 불가 — 원본 반환하되 False로 거부
                )

        return True, 'OK', path

    # ─────────────────── 경로 길이 계산 ───────────────────

    def _calc_path_length(self, path: Path) -> float:
        """경로 총 길이 계산 (m).

        연속 포즈 쌍의 유클리드 거리 합산.
        """
        total = 0.0
        poses = path.poses
        for i in range(len(poses) - 1):
            p0 = poses[i].pose.position
            p1 = poses[i + 1].pose.position
            total += math.hypot(p1.x - p0.x, p1.y - p0.y)
        return total

    # ─────────────────── 경로 트림 ───────────────────

    def _trim_path_to_length(self, path: Path, max_length: float) -> Path:
        """경로를 max_length(m) 이내로 잘라 반환.

        마지막 포즈가 max_length에 딱 맞지 않아도 가장 가까운 포즈까지만 포함.
        최소 1개 포즈는 항상 유지한다.
        """
        trimmed = Path()
        trimmed.header = path.header

        poses = path.poses
        if not poses:
            return trimmed

        trimmed.poses.append(poses[0])
        accumulated = 0.0

        for i in range(1, len(poses)):
            p0 = poses[i - 1].pose.position
            p1 = poses[i].pose.position
            seg = math.hypot(p1.x - p0.x, p1.y - p0.y)

            if accumulated + seg > max_length:
                # 이번 포즈를 추가하면 한도 초과 → 여기서 중단
                break

            accumulated += seg
            trimmed.poses.append(poses[i])

        return trimmed

    # ─────────────────── 급커브 감지 ───────────────────

    def _find_sharp_turns(self, path: Path, min_radius: float) -> list:
        """경로의 연속 3점으로 곡률 반경을 계산하여 급커브 인덱스 반환.

        외접원 반경 < min_radius 인 구간을 급커브로 판정.

        Returns:
            list of (index, radius): 급커브 구간 인덱스와 외접원 반경
        """
        sharp = []
        poses = path.poses

        for i in range(1, len(poses) - 1):
            p0 = poses[i - 1].pose.position
            p1 = poses[i].pose.position
            p2 = poses[i + 1].pose.position

            radius = self._circumradius(p0, p1, p2)

            # 세 점이 일직선이면 반경 무한대 → 급커브 아님
            if radius is not None and radius < min_radius:
                sharp.append((i, radius))

        return sharp

    def _circumradius(self, p0, p1, p2) -> float:
        """세 점의 외접원 반경 계산 (m).

        세 점이 일직선(넓이≈0)이면 None 반환.

        수식: R = (a * b * c) / (4 * Area)
          a, b, c: 삼각형 세 변의 길이
          Area: 삼각형 넓이 (shoelace formula)
        """
        ax, ay = p0.x, p0.y
        bx, by = p1.x, p1.y
        cx, cy = p2.x, p2.y

        # 세 변의 길이
        a = math.hypot(bx - cx, by - cy)
        b = math.hypot(ax - cx, ay - cy)
        c = math.hypot(ax - bx, ay - by)

        # 삼각형 넓이 (shoelace)
        area = abs((bx - ax) * (cy - ay) - (cx - ax) * (by - ay)) / 2.0

        if area < 1e-9:
            # 일직선 — 반경 무한대, 급커브 아님
            return None

        return (a * b * c) / (4.0 * area)

    # ─────────────────── 급커브 스무딩 ───────────────────

    def _smooth_sharp_turns(self, path: Path, sharp_turns: list) -> Path:
        """급커브 구간을 완화한 경로 반환.

        전략: 급커브 인덱스의 포즈를 전후 포즈의 중간점으로 교체.
        복잡한 스플라인 대신 단순 midpoint smoothing 적용.
        (실제 배포 환경에서는 NavFn/SPLINE 플래너로 교체 권장)

        Args:
            path: 원본 경로
            sharp_turns: _find_sharp_turns 결과 [(index, radius), ...]

        Returns:
            스무딩된 경로 (nav_msgs/Path)
        """
        import copy
        smoothed = Path()
        smoothed.header = path.header

        # poses 복사 (원본 불변)
        smoothed.poses = [copy.deepcopy(p) for p in path.poses]

        sharp_indices = {idx for idx, _ in sharp_turns}

        for idx in sharp_indices:
            poses = smoothed.poses
            if idx <= 0 or idx >= len(poses) - 1:
                continue

            prev_p = poses[idx - 1].pose.position
            next_p = poses[idx + 1].pose.position

            # 전후 포즈의 중간점으로 교체
            poses[idx].pose.position.x = (prev_p.x + next_p.x) / 2.0
            poses[idx].pose.position.y = (prev_p.y + next_p.y) / 2.0
            poses[idx].pose.position.z = (prev_p.z + next_p.z) / 2.0

        return smoothed

    # ─────────────────── 후진 구간 감지 ───────────────────

    def _find_reverse_segments(self, path: Path) -> list:
        """충수 상태에서 후진에 해당하는 구간 인덱스 반환.

        후진 판단 기준:
          - 경로 첫 포즈를 기준으로 "전체 이동 방향"을 결정
          - 각 구간 벡터가 전체 방향과 반대(내적 < 0)이면 후진으로 판정

        소방호스는 충수 시 후진 이동으로 인해 배관 꼬임 발생.

        Returns:
            list of int: 후진 구간 시작 인덱스 리스트
        """
        poses = path.poses
        if len(poses) < 2:
            return []

        # 전체 이동 벡터 (시작 → 끝)
        start = poses[0].pose.position
        end = poses[-1].pose.position
        overall_dx = end.x - start.x
        overall_dy = end.y - start.y
        overall_len = math.hypot(overall_dx, overall_dy)

        if overall_len < 1e-9:
            # 시작과 끝이 동일 — 후진 판단 불가
            return []

        # 정규화된 전체 방향 벡터
        dir_x = overall_dx / overall_len
        dir_y = overall_dy / overall_len

        reverse_indices = []
        for i in range(len(poses) - 1):
            p0 = poses[i].pose.position
            p1 = poses[i + 1].pose.position
            seg_dx = p1.x - p0.x
            seg_dy = p1.y - p0.y

            # 내적 < 0 → 전체 방향과 반대 = 후진
            dot = seg_dx * dir_x + seg_dy * dir_y
            if dot < 0.0:
                reverse_indices.append(i)

        return reverse_indices


# ─────────────────── Entry Point ───────────────────

def main(args=None):
    rclpy.init(args=args)
    node = HoseAwarePlanner()

    # LifecycleNode 단독 실행 시 수동 전환
    node.trigger_configure()
    node.trigger_activate()

    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.trigger_deactivate()
        node.trigger_cleanup()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
