"""
qos_demo_node.py — Day 19: QoS 정책 실험

이 노드는 세 가지 QoS 패턴을 동시에 보여줍니다:

1. 센서 구독 (BEST_EFFORT): LiDAR - 최신성 우선
2. 상태 발행 (RELIABLE + TRANSIENT_LOCAL): 로봇 상태 - 늦게 연결해도 수신
3. QoS 불일치 감지: 잘못된 QoS 조합이 어떻게 통신을 막는지

실무 인사이트:
- Nav2의 /map은 TRANSIENT_LOCAL → 이 노드가 언제 시작해도 지도 수신
- LiDAR는 BEST_EFFORT → QoS 맞춰야 수신 가능
- QoS 불일치는 에러 없이 조용히 통신이 안 됨 (가장 위험한 버그)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    qos_profile_sensor_data,      # 미리 정의된 센서용 QoS
    qos_profile_system_default,
)
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String


class QoSDemoNode(Node):
    """QoS 정책의 실제 동작을 보여주는 데모 노드."""

    def __init__(self):
        super().__init__('qos_demo')

        # ── QoS 프로파일 정의 ──────────────────────────────────────

        # 패턴 1: 센서 데이터용 QoS
        # BEST_EFFORT: 손실 허용, 최신성 우선
        # VOLATILE: 과거 데이터 불필요
        # KEEP_LAST(1): 가장 최신 스캔만
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # 참고: 위와 동일 → qos_profile_sensor_data (미리 정의된 것)

        # 패턴 2: 중요 상태 데이터용 QoS
        # RELIABLE: 반드시 전달
        # TRANSIENT_LOCAL: Late Joiner도 즉시 수신
        # KEEP_LAST(1): 가장 최신 상태만 유지
        latching_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Subscriber 1: 센서 QoS로 LiDAR 구독 ──────────────────
        # 핵심: Gazebo 브릿지가 BEST_EFFORT로 발행하므로 맞춰야 수신됨
        self._scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self._on_scan,
            qos_profile=sensor_qos,   # BEST_EFFORT ↔ BEST_EFFORT: 정상
        )

        # ── Subscriber 2: TRANSIENT_LOCAL로 /map 구독 ─────────────
        # 핵심: map_server가 TRANSIENT_LOCAL로 발행
        # → 이 노드가 늦게 시작해도 현재 지도를 즉시 받음
        self._map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self._on_map,
            qos_profile=latching_qos,
        )

        # ── Publisher: 로봇 상태를 TRANSIENT_LOCAL로 발행 ─────────
        # 이유: 모니터링 노드가 언제 켜지든 현재 상태를 즉시 알 수 있어야 함
        self._status_pub = self.create_publisher(
            String,
            '/robot_qos_status',
            qos_profile=latching_qos,
        )

        # ── 통계 추적 ─────────────────────────────────────────────
        self._scan_count = 0
        self._map_received = False
        self._last_scan_range = 0.0

        # 5초마다 상태 발행 및 QoS 통계 출력
        self._timer = self.create_timer(5.0, self._report_status)

        self.get_logger().info(
            'QoSDemoNode 시작\n'
            '  /scan  구독: BEST_EFFORT (센서 QoS)\n'
            '  /map   구독: RELIABLE + TRANSIENT_LOCAL\n'
            '  /robot_qos_status 발행: RELIABLE + TRANSIENT_LOCAL'
        )

    def _on_scan(self, msg: LaserScan):
        """LiDAR 수신 카운터."""
        self._scan_count += 1
        # 유효한 거리 중 최솟값 (가장 가까운 장애물)
        valid = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if valid:
            self._last_scan_range = min(valid)

    def _on_map(self, msg: OccupancyGrid):
        """지도 수신 확인."""
        if not self._map_received:
            self._map_received = True
            w, h = msg.info.width, msg.info.height
            res = msg.info.resolution
            self.get_logger().info(
                f'지도 수신 성공! (TRANSIENT_LOCAL 덕분에 늦게 시작해도 수신)\n'
                f'  크기: {w}x{h} 픽셀, 해상도: {res}m/픽셀\n'
                f'  실제 크기: {w*res:.1f}m x {h*res:.1f}m'
            )

    def _report_status(self):
        """5초마다 QoS 통계 출력."""
        status_msg = String()
        status_msg.data = (
            f'scan_count={self._scan_count}, '
            f'map_received={self._map_received}, '
            f'nearest_obstacle={self._last_scan_range:.2f}m'
        )
        self._status_pub.publish(status_msg)

        self.get_logger().info(
            f'[QoS 통계] 5초간 /scan 수신: {self._scan_count}회 '
            f'| 지도: {"✅" if self._map_received else "❌"} '
            f'| 최근접 장애물: {self._last_scan_range:.2f}m'
        )
        self._scan_count = 0  # 5초 카운터 리셋


def main(args=None):
    rclpy.init(args=args)
    node = QoSDemoNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
