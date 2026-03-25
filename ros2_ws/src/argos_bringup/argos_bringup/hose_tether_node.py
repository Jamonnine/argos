#!/usr/bin/env python3
"""hose_tether_node.py — ARGOS HR-셰르파 호스 릴 상태 추적 노드

odom을 구독하여 로봇 이동 경로를 누적하고,
배포된 호스 길이·꺾임 위험도·충수 여부를 10Hz로 발행한다.

발행 토픽:
  /{robot_id}/hose/status   (std_msgs/Float32MultiArray)
    data[0]: remaining_m   — 호스 잔여 길이 (m, 100 - deployed_length)
    data[1]: kink_risk      — 꺾임 위험도 (0.0=안전 ~ 1.0=위험)
    data[2]: charged        — 방수포 활성 여부 (0.0=건수 / 1.0=충수)

  /{robot_id}/hose/path_viz (nav_msgs/Path)
    호스 경로 시각화 (RViz용)

구독 토픽:
  /{robot_id}/odom          (nav_msgs/Odometry, RELIABLE+TRANSIENT_LOCAL)

서비스:
  /hose/retract             (std_srvs/Trigger) — 호스 배포 길이 초기화

파라미터:
  max_hose_length  (float): 호스 총 길이 m (기본 100.0)
  min_bend_radius  (float): 최소 꺾임 반경 m (기본 0.5)
  robot_id         (str)  : 로봇 ID — 빈 문자열이면 namespace 자동 추출
  hose_anchor_x    (float): 호스 고정점 x (진입구, 기본 0.0)
  hose_anchor_y    (float): 호스 고정점 y (진입구, 기본 0.0)

호스 고정점(anchor)은 소방 진입구 좌표. 로봇이 anchor에서 멀어질수록
호스가 풀려나가 deployed_length가 증가한다.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger


# 경로 점 최소 간격 (m) — 더 짧은 간격은 노이즈로 간주하여 무시
_MIN_POINT_SPACING_M = 0.5

# 꺾임 위험도 계산 기준 점 수
_KINK_WINDOW = 3


class HoseTetherNode(Node):
    """호스 릴 상태 추적 노드.

    odom 위치를 경로 리스트에 누적하여 배포 길이를 계산하고,
    최근 3점의 곡률로 꺾임 위험도를 산출한다.
    sherpa_platform.py의 _hose_status_callback과 동일한 포맷으로 발행한다.
    """

    def __init__(self):
        super().__init__('hose_tether_node')

        # ── 파라미터 선언 및 읽기 ────────────────────────────────────────────
        self.declare_parameter('max_hose_length', 100.0)
        self.declare_parameter('min_bend_radius', 0.5)
        self.declare_parameter('robot_id', '')
        self.declare_parameter('hose_anchor_x', 0.0)
        self.declare_parameter('hose_anchor_y', 0.0)
        # NFRI 고압 호스릴 스펙 (2025 개발품) # NFRI 2025 리빙랩
        self.declare_parameter('hose_inner_diameter_mm', 32.0)  # 내경 32mm (기존 65mm는 일반 소방호스)
        self.declare_parameter('hose_glow_material', True)       # 야광호스 (LED 30분→2~6시간 발광)

        self._max_hose_m = self.get_parameter('max_hose_length').value
        self._min_bend_radius = self.get_parameter('min_bend_radius').value
        self._hose_anchor_x = self.get_parameter('hose_anchor_x').value
        self._hose_anchor_y = self.get_parameter('hose_anchor_y').value

        # robot_id: 파라미터 → namespace 자동 추출 순서 (robot_status_node.py 패턴)
        robot_id = self.get_parameter('robot_id').value
        if not robot_id:
            ns = self.get_namespace().strip('/')
            if ns:
                robot_id = ns
                self.get_logger().warn(
                    f'robot_id 비어있음 → namespace에서 추출: {robot_id}'
                )
            else:
                robot_id = 'sherpa1'
                self.get_logger().warn(
                    f'robot_id·namespace 모두 비어있음 → 기본값 사용: {robot_id}'
                )
        self._robot_id = robot_id

        # ── 내부 상태 ────────────────────────────────────────────────────────
        # 호스 경로 점 리스트 [(x, y), ...]
        # 첫 점: 호스 고정점(anchor)에서 시작
        self._path_points: list[tuple[float, float]] = [
            (self._hose_anchor_x, self._hose_anchor_y)
        ]

        # 누적 배포 길이 (m)
        self._deployed_length: float = 0.0

        # 방수포 활성 여부 (외부 명령으로 변경 가능)
        self._charged: bool = False

        # ── odom QoS: RELIABLE + TRANSIENT_LOCAL (DDC 패턴, ros2.md ★★★) ───
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10,
        )

        # ── 구독자 ───────────────────────────────────────────────────────────
        odom_topic = f'/{robot_id}/odom'
        self._odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self._odom_callback,
            odom_qos,
        )

        # ── 발행자 ───────────────────────────────────────────────────────────
        status_topic = f'/{robot_id}/hose/status'
        self._status_pub = self.create_publisher(
            Float32MultiArray,
            status_topic,
            10,
        )

        path_viz_topic = f'/{robot_id}/hose/path_viz'
        self._path_viz_pub = self.create_publisher(
            Path,
            path_viz_topic,
            10,
        )

        # ── 서비스: 호스 회수 (배포 길이 초기화) ────────────────────────────
        self._retract_srv = self.create_service(
            Trigger,
            '/hose/retract',
            self._retract_callback,
        )

        # ── 발행 타이머 (10Hz) ───────────────────────────────────────────────
        self._pub_timer = self.create_timer(0.1, self._publish_status)

        self.get_logger().info(
            f'[HoseTether:{robot_id}] 초기화 완료 '
            f'(max_hose={self._max_hose_m:.0f}m, '
            f'min_bend_radius={self._min_bend_radius:.2f}m, '
            f'anchor=({self._hose_anchor_x:.1f},{self._hose_anchor_y:.1f}), '
            f'odom={odom_topic})'
        )

    # ──────────────────────────────────────────────────────────────────────────
    # odom 콜백 — 경로 점 누적 + 배포 길이 갱신
    # ──────────────────────────────────────────────────────────────────────────

    def _odom_callback(self, msg: Odometry) -> None:
        """odom 수신 시 경로 점 추가 및 배포 길이 누적.

        0.5m 미만 이동은 노이즈로 판단하여 경로 점을 추가하지 않는다.
        """
        pos = msg.pose.pose.position
        new_x, new_y = float(pos.x), float(pos.y)

        # 마지막 경로 점과의 거리 계산
        last_x, last_y = self._path_points[-1]
        dist = _euclidean(last_x, last_y, new_x, new_y)

        if dist < _MIN_POINT_SPACING_M:
            return  # 최소 간격 미달 — 노이즈 무시

        # 경로 점 추가 및 배포 길이 누적
        self._path_points.append((new_x, new_y))
        self._deployed_length += dist

        # 물리적 한계 초과 방지
        self._deployed_length = min(self._deployed_length, self._max_hose_m)

    # ──────────────────────────────────────────────────────────────────────────
    # 꺾임 위험도 계산
    # ──────────────────────────────────────────────────────────────────────────

    def _calc_kink_risk(self) -> float:
        """최근 3점으로 호스 꺾임 위험도 계산 (0.0 ~ 1.0).

        곡률 공식 (3점 P1, P2, P3):
          - chord 벡터 P1→P2, P2→P3의 내적으로 사이각 θ 산출
          - chord_length = |P1→P3| (두 끝점 거리)
          - 곡률 κ = 2·sin(θ) / chord_length
          - 굽힘 반경 r = 1/κ (κ=0이면 r=∞)
          - r < min_bend_radius → risk = 1.0
          - r ≥ min_bend_radius → risk = clamp(min_bend_radius / r, 0.0, 1.0)

        경로 점이 3개 미만이면 0.0 반환 (아직 꺾임 계산 불가).
        """
        if len(self._path_points) < _KINK_WINDOW:
            return 0.0

        # 최근 3점 추출
        p1 = self._path_points[-3]
        p2 = self._path_points[-2]
        p3 = self._path_points[-1]

        # P1→P2, P2→P3 벡터
        v1x = p2[0] - p1[0]
        v1y = p2[1] - p1[1]
        v2x = p3[0] - p2[0]
        v2y = p3[1] - p2[1]

        len_v1 = math.sqrt(v1x ** 2 + v1y ** 2)
        len_v2 = math.sqrt(v2x ** 2 + v2y ** 2)

        if len_v1 < 1e-6 or len_v2 < 1e-6:
            # 이동 거리가 거의 없음 — 꺾임 없음
            return 0.0

        # 사이각 θ (내적으로 cos θ 산출)
        cos_theta = (v1x * v2x + v1y * v2y) / (len_v1 * len_v2)
        cos_theta = max(-1.0, min(1.0, cos_theta))  # 수치 오차 클램프
        theta = math.acos(cos_theta)  # 0 ~ π

        # chord 길이: P1 → P3 직선 거리
        chord_length = _euclidean(p1[0], p1[1], p3[0], p3[1])

        if chord_length < 1e-6:
            # 제자리 회전(P1≈P3) — 최대 위험으로 판단
            return 1.0

        # 곡률 κ = 2·sin(θ) / chord_length
        curvature = 2.0 * math.sin(theta) / chord_length

        if curvature < 1e-6:
            # 직선 이동 — 꺾임 없음
            return 0.0

        # 굽힘 반경 r = 1/κ
        bend_radius = 1.0 / curvature

        if bend_radius < self._min_bend_radius:
            return 1.0

        # 정규화: 반경이 작을수록 위험도 상승
        risk = self._min_bend_radius / bend_radius
        return max(0.0, min(1.0, risk))

    # ──────────────────────────────────────────────────────────────────────────
    # 발행: 호스 상태 + 경로 시각화
    # ──────────────────────────────────────────────────────────────────────────

    def _publish_status(self) -> None:
        """10Hz 타이머 콜백 — 호스 상태와 경로 시각화 발행."""
        now = self.get_clock().now().to_msg()

        # ── Float32MultiArray: [remaining_m, kink_risk, charged] ─────────────
        remaining_m = max(0.0, self._max_hose_m - self._deployed_length)
        kink_risk = self._calc_kink_risk()
        charged = 1.0 if self._charged else 0.0

        status_msg = Float32MultiArray()
        status_msg.data = [remaining_m, kink_risk, charged]
        self._status_pub.publish(status_msg)

        # ── nav_msgs/Path: 호스 경로 시각화 ──────────────────────────────────
        path_msg = Path()
        path_msg.header.stamp = now
        path_msg.header.frame_id = 'map'

        for x, y in self._path_points:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = now
            pose_stamped.header.frame_id = 'map'
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.w = 1.0
            path_msg.poses.append(pose_stamped)

        self._path_viz_pub.publish(path_msg)

        # 잔여 호스 20m 미만 경고 (30초 쓰로틀)
        if remaining_m < 20.0:
            self.get_logger().warn(
                f'[{self._robot_id}] 호스 잔여량 부족: {remaining_m:.1f}m '
                f'(배포={self._deployed_length:.1f}m)',
                throttle_duration_sec=30.0,
            )

    # ──────────────────────────────────────────────────────────────────────────
    # 서비스: 호스 회수 (초기화)
    # ──────────────────────────────────────────────────────────────────────────

    def _retract_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """/hose/retract 서비스 — 호스 배포 길이 및 경로 초기화.

        로봇이 귀환하여 호스를 물리적으로 회수한 후 호출.
        경로 점은 anchor 위치로 초기화된다.
        """
        prev_deployed = self._deployed_length

        # 경로·배포 길이 초기화
        self._path_points = [(self._hose_anchor_x, self._hose_anchor_y)]
        self._deployed_length = 0.0

        self.get_logger().info(
            f'[{self._robot_id}] 호스 회수 완료 '
            f'(이전 배포={prev_deployed:.1f}m → 0.0m, '
            f'anchor=({self._hose_anchor_x:.1f},{self._hose_anchor_y:.1f}))'
        )

        response.success = True
        response.message = (
            f'호스 회수 완료: {prev_deployed:.1f}m → 0.0m'
        )
        return response


# ──────────────────────────────────────────────────────────────────────────────
# 유틸리티
# ──────────────────────────────────────────────────────────────────────────────

def _euclidean(x1: float, y1: float, x2: float, y2: float) -> float:
    """두 좌표 사이 유클리드 거리 (m)."""
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


# ──────────────────────────────────────────────────────────────────────────────
# 진입점
# ──────────────────────────────────────────────────────────────────────────────

def main(args=None):
    """노드 진입점."""
    rclpy.init(args=args)
    node = HoseTetherNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
