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
드론 열화상 → 오케스트레이터 FireAlert 릴레이 노드.

소방 현장 워크플로우:
  1. 드론이 열화상 카메라로 화점 감지 (hotspot_tracker 발행)
  2. 이 노드가 드론 위치 기반으로 지상 좌표 추정
  3. FireAlert 생성 → /orchestrator/fire_alert 발행
  4. 오케스트레이터가 CBBA로 UGV 진압 + 드론 감시 할당

핵심 설계:
  - 직하방 투영: 드론 고도에서 직하방 가정 (x,y는 드론과 동일, z=0)
  - 쿨다운: 동일 위치 반복 감지 시 중복 발행 방지 (5초)
  - 근접 중복 제거: 3m 이내 화점은 동일 화점으로 판정
  - 고도 검증: 지상(2m 미만)에서의 감지는 무시 (UGV 열화상과 구분)

토픽:
  구독: /{drone_id}/thermal_camera/hotspot (FireAlert — hotspot_tracker 발행)
        /{drone_id}/odom (Odometry)
  발행: /orchestrator/fire_alert (FireAlert)
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, PointStamped
from argos_interfaces.msg import FireAlert


class DroneFireRelayNode(Node):
    """드론 열화상 감지 → 오케스트레이터 FireAlert 릴레이."""

    def __init__(self):
        super().__init__('drone_fire_relay')

        # 파라미터
        self.declare_parameter('drone_id', 'drone1')
        self.declare_parameter('cooldown_sec', 5.0)
        self.declare_parameter('altitude_threshold', 2.0)
        self.declare_parameter('proximity_dedup_m', 3.0)

        self.drone_id = self.get_parameter('drone_id').value
        self.cooldown_sec = self.get_parameter('cooldown_sec').value
        self.altitude_threshold = self.get_parameter('altitude_threshold').value
        self.proximity_dedup_m = self.get_parameter('proximity_dedup_m').value

        # 상태
        self._drone_pose = None          # (x, y, z)
        self._last_alert_time = 0.0      # 마지막 발행 시각 (단조 시계 초)
        self._last_alert_location = None  # (x, y) — 근접 중복 판정용
        self._alert_count = 0            # 발행 누적 횟수 (로그용)

        # QoS — RELIABLE + TRANSIENT_LOCAL: 오케스트레이터 늦게 뜰 경우 대비
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10,
        )

        # 구독: 드론 odom (기본 QoS 사용 — sim/실물 모두 BEST_EFFORT 발행)
        self._odom_sub = self.create_subscription(
            Odometry,
            f'/{self.drone_id}/odom',
            self._odom_callback,
            10,
        )

        # 구독: 드론 열화상 화점 (hotspot_tracker가 RELIABLE로 발행)
        self._hotspot_sub = self.create_subscription(
            FireAlert,
            f'/{self.drone_id}/thermal_camera/hotspot',
            self._hotspot_callback,
            reliable_qos,
        )

        # 발행: 오케스트레이터 FireAlert
        self._alert_pub = self.create_publisher(
            FireAlert,
            '/orchestrator/fire_alert',
            reliable_qos,
        )

        self.get_logger().info(
            f'DroneFireRelay 초기화 완료: drone_id={self.drone_id} '
            f'cooldown={self.cooldown_sec}s '
            f'alt_threshold={self.altitude_threshold}m '
            f'dedup={self.proximity_dedup_m}m'
        )

    # ──────────────────────────────────────────────────
    # 콜백
    # ──────────────────────────────────────────────────

    def _odom_callback(self, msg: Odometry) -> None:
        """드론 위치 갱신."""
        p = msg.pose.pose.position
        self._drone_pose = (p.x, p.y, p.z)

    def _hotspot_callback(self, msg: FireAlert) -> None:
        """열화상 화점 수신 → 검증 → FireAlert 릴레이.

        처리 순서:
          1. 드론 위치 유효성 확인
          2. 고도 검증 (UGV 열화상과 구분)
          3. 쿨다운 확인
          4. 지상 좌표 추정 (직하방 투영)
          5. 근접 중복 제거
          6. FireAlert 발행
        """
        now = self.get_clock().now().nanoseconds / 1e9

        # 1. 드론 위치 유효성 검증
        if self._drone_pose is None:
            self.get_logger().warn(
                f'[{self.drone_id}] 화점 감지했으나 odom 미수신 — 무시')
            return

        # 2. 고도 검증: altitude_threshold 미만은 지상 로봇과 구분 불가로 릴레이 생략
        drone_alt = self._drone_pose[2]
        if drone_alt < self.altitude_threshold:
            self.get_logger().debug(
                f'[{self.drone_id}] 고도 {drone_alt:.1f}m < '
                f'{self.altitude_threshold}m — 지상 감지, 릴레이 생략'
            )
            return

        # 3. 쿨다운 검증
        elapsed = now - self._last_alert_time
        if elapsed < self.cooldown_sec:
            self.get_logger().debug(
                f'[{self.drone_id}] 쿨다운 중 (경과={elapsed:.1f}s / '
                f'필요={self.cooldown_sec}s)'
            )
            return

        # 4. 지상 좌표 추정 (직하방 투영)
        ground_pt = self._project_to_ground(msg)

        # 5. 근접 중복 제거
        if self._last_alert_location is not None:
            dist = math.hypot(
                ground_pt.x - self._last_alert_location[0],
                ground_pt.y - self._last_alert_location[1],
            )
            if dist < self.proximity_dedup_m:
                self.get_logger().debug(
                    f'[{self.drone_id}] 근접 중복 '
                    f'(dist={dist:.1f}m < {self.proximity_dedup_m}m)'
                )
                return

        # 6. FireAlert 생성 + 발행
        alert = FireAlert()
        stamp = self.get_clock().now().to_msg()

        alert.header.stamp = stamp
        alert.header.frame_id = 'map'
        alert.robot_id = self.drone_id
        alert.severity = msg.severity if msg.severity else 'medium'
        alert.max_temperature_kelvin = msg.max_temperature_kelvin
        alert.confidence = msg.confidence
        alert.active = True

        alert.location = PointStamped()
        alert.location.header.stamp = stamp
        alert.location.header.frame_id = 'map'
        alert.location.point = ground_pt

        self._alert_pub.publish(alert)

        # 상태 갱신
        self._last_alert_time = now
        self._last_alert_location = (ground_pt.x, ground_pt.y)
        self._alert_count += 1

        temp_c = msg.max_temperature_kelvin - 273.15
        self.get_logger().warn(
            f'[{self.drone_id}] FIRE RELAY #{self._alert_count}: '
            f'severity={msg.severity} temp={temp_c:.0f}°C '
            f'location=({ground_pt.x:.1f}, {ground_pt.y:.1f}) '
            f'drone_alt={drone_alt:.1f}m'
        )

    # ──────────────────────────────────────────────────
    # 헬퍼
    # ──────────────────────────────────────────────────

    def _project_to_ground(self, msg: FireAlert) -> Point:
        """드론 위치에서 화점 지상 좌표 추정.

        우선순위:
          1. hotspot_tracker가 이미 유효한 지상 좌표를 계산한 경우 → 그대로 사용
          2. 없으면 직하방 투영: 드론 (x, y) → 지상 z=0

        향후 확장:
          - 카메라 내외부 파라미터(intrinsics + extrinsics) 기반 정밀 투영
          - 고도·카메라 틸트각을 이용한 수평 오프셋 보정
        """
        # hotspot_tracker가 이미 지상 좌표를 계산한 경우 그대로 사용
        if (
            msg.location is not None
            and hasattr(msg.location, 'point')
            and (msg.location.point.x != 0.0 or msg.location.point.y != 0.0)
        ):
            return Point(
                x=msg.location.point.x,
                y=msg.location.point.y,
                z=0.0,
            )

        # 직하방 투영: 드론 x,y → 지상 z=0
        return Point(
            x=self._drone_pose[0],
            y=self._drone_pose[1],
            z=0.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = DroneFireRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
