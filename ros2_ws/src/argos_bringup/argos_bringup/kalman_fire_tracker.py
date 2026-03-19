#!/usr/bin/env python3
"""kalman_fire_tracker.py — Kalman 기반 화점 추적 + 확산 예측 ROS2 노드

perception_bridge_node가 감지한 FireAlert를 수신하여
SimpleKalmanTracker로 추적하고, 5분 후 화점 위치를 예측·발행합니다.

발행:
  /fire_prediction   (PoseArray)   — 예측 화점 위치 배열 (5분 후)
  /evacuation_route  (PoseStamped) — 예측 화점 반대 방향 대피 목적지

구독:
  /orchestrator/fire_alert  (FireAlert)  — 화점 감지 결과
  odom                      (Odometry)   — 로봇 현재 위치

파라미터:
  prediction_horizon_sec  : 예측 지평선 (초, 기본 300 = 5분)
  safe_distance_m         : 대피 목적지 거리 (m, 기본 10)
  publish_rate_hz         : 예측 발행 주기 (Hz, 기본 0.2 = 5초마다)
"""

import time
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseArray, PoseStamped, Pose
from nav_msgs.msg import Odometry
from argos_interfaces.msg import FireAlert

from argos_bringup.kalman_tracker import SimpleKalmanTracker


class KalmanFireTrackerNode(Node):
    """화점 추적 + 확산 예측 노드.

    SimpleKalmanTracker 인스턴스를 소유하고,
    FireAlert 콜백마다 Kalman 업데이트를 수행합니다.
    주기 타이머에서 predict_fire_positions / get_evacuation_direction를
    호출해 결과를 발행합니다.
    """

    def __init__(self):
        super().__init__('kalman_fire_tracker')

        # ── 파라미터 선언 ──
        self.declare_parameter('prediction_horizon_sec', 300.0)
        self.declare_parameter('safe_distance_m', 10.0)
        self.declare_parameter('publish_rate_hz', 0.2)   # 기본 5초마다

        horizon = self.get_parameter('prediction_horizon_sec').value
        self.safe_dist = self.get_parameter('safe_distance_m').value
        pub_rate = self.get_parameter('publish_rate_hz').value

        # 발행 빈도 상한 (10Hz) — 불필요한 과부하 방지
        pub_rate = min(pub_rate, 10.0)

        # ── Kalman 추적기 ──
        self.tracker = SimpleKalmanTracker(
            max_distance=5.0,   # 화점은 천천히 이동 → 5m 범위 매칭
            max_missed=30,      # 약 150초 (5초 주기 기준) 미감지까지 유지
        )
        self.horizon_sec = horizon

        # ── 로봇 위치 (odom 콜백에서 갱신) ──
        self.robot_x = 0.0
        self.robot_y = 0.0

        # ── QoS 설정 ──
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=5,
        )

        # ── 구독 ──
        self.fire_sub = self.create_subscription(
            FireAlert,
            '/orchestrator/fire_alert',
            self._fire_alert_cb,
            reliable_qos,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self._odom_cb,
            10,
        )

        # ── 발행 ──
        self.prediction_pub = self.create_publisher(
            PoseArray,
            '/fire_prediction',
            reliable_qos,
        )
        self.evac_pub = self.create_publisher(
            PoseStamped,
            '/evacuation_route',
            reliable_qos,
        )

        # ── 주기 타이머 ──
        self.timer = self.create_timer(1.0 / pub_rate, self._publish_predictions)

        self.get_logger().info(
            f'KalmanFireTracker 초기화 완료 — '
            f'horizon={horizon}s, safe_dist={self.safe_dist}m, '
            f'pub_rate={pub_rate}Hz'
        )

    # ─────────────────── 콜백 ───────────────────

    def _odom_cb(self, msg: Odometry):
        """로봇 위치 갱신."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def _fire_alert_cb(self, msg: FireAlert):
        """FireAlert 수신 → Kalman 업데이트.

        FireAlert.active=True 인 경우만 추적에 반영.
        active=False(소화 완료)는 무시 (트랙은 missed_count 만료로 자동 제거).
        """
        if not msg.active:
            return

        fire_x = msg.location.point.x
        fire_y = msg.location.point.y
        confidence = float(msg.confidence) if hasattr(msg, 'confidence') else 0.8

        # Kalman 업데이트: [(x, y, class_name, confidence)]
        now = time.time()
        self.tracker.update(
            [(fire_x, fire_y, 'fire', confidence)],
            timestamp=now,
        )

        self.get_logger().debug(
            f'FireAlert 수신: ({fire_x:.2f}, {fire_y:.2f}) '
            f'활성 트랙={len(self.tracker.tracks)}개'
        )

    # ─────────────────── 예측 발행 ───────────────────

    def _publish_predictions(self):
        """예측 화점 위치 + 대피 경로 발행."""
        stamp = self.get_clock().now().to_msg()

        # 1. 화점 위치 예측
        predictions = self.tracker.predict_fire_positions(self.horizon_sec)
        self._publish_fire_prediction(stamp, predictions)

        # 2. 대피 경로 계산 + 발행
        evac = self.tracker.get_evacuation_direction(
            robot_x=self.robot_x,
            robot_y=self.robot_y,
            safe_distance=self.safe_dist,
            horizon_sec=self.horizon_sec,
        )
        if evac is not None:
            self._publish_evacuation_route(stamp, evac)

    def _publish_fire_prediction(self, stamp, predictions: list):
        """예측 화점 위치를 PoseArray로 발행.

        PoseArray: header + poses 배열.
        각 Pose는 예측 위치(x, y)를 담으며,
        orientation은 이동 방향(heading)을 쿼터니언으로 인코딩합니다.
        """
        msg = PoseArray()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'

        for pred in predictions:
            pose = Pose()
            pose.position.x = pred['predicted_x']
            pose.position.y = pred['predicted_y']
            pose.position.z = 0.0

            # heading → 쿼터니언 (yaw만 사용, roll/pitch=0)
            heading = pred['heading_rad']
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = math.sin(heading / 2.0)
            pose.orientation.w = math.cos(heading / 2.0)

            msg.poses.append(pose)

        self.prediction_pub.publish(msg)

        if predictions:
            self.get_logger().info(
                f'[화점예측] {len(predictions)}개 화점 → '
                f'{self.horizon_sec:.0f}s 후 위치 발행'
            )

    def _publish_evacuation_route(self, stamp, evac: dict):
        """대피 목적지를 PoseStamped로 발행.

        orientation은 대피 방향(away_heading_rad)을 쿼터니언으로 인코딩합니다.
        """
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'

        msg.pose.position.x = evac['safe_x']
        msg.pose.position.y = evac['safe_y']
        msg.pose.position.z = 0.0

        heading = evac['away_heading_rad']
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(heading / 2.0)
        msg.pose.orientation.w = math.cos(heading / 2.0)

        self.evac_pub.publish(msg)

        self.get_logger().info(
            f'[대피경로] 목적지=({evac["safe_x"]:.2f}, {evac["safe_y"]:.2f}) '
            f'화점중심=({evac["fire_centroid_x"]:.2f}, {evac["fire_centroid_y"]:.2f}) '
            f'화점수={evac["fire_count"]}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = KalmanFireTrackerNode()
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
