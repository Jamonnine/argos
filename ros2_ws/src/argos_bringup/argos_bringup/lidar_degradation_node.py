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
"""lidar_degradation_node.py — 방수포 분사 시 LiDAR 신뢰도 저하 시뮬레이션

NFRI(국립소방연구원) 실험 데이터 기반:
  - 목재 화재 + 방수포 분사 → 물입자가 LiDAR 반사점으로 오인식
  - 방수포 분사 방향 ±30° 범위에서 탐지 오차 최대 76.7%
  - 폴리우레탄/스티로폼 연기는 레이더 정확도에 유의미한 영향 없음

노드 역할:
  - 원본 LaserScan + 방수포 상태를 구독
  - 방수포 활성 시 오탐(가짜 근거리 반사점) 주입 후 scan_degraded 발행
  - 현재 LiDAR 신뢰도 (0.0~1.0) 토픽 발행 → 오케스트레이터가 Nav2 가중치 조정에 활용

Nav2 연동 권고:
  - collision_monitor: scan_degraded 대신 scan_base(원본) 사용
  - 오케스트레이터: 방수 중 SLAM costmap 가중치 감소 (reliability < 0.5 임계)
"""

import copy
import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32


class LidarDegradationNode(Node):
    """방수포 분사 시 LiDAR 노이즈를 주입하는 시뮬레이션 노드.

    방수포가 비활성 상태일 때는 원본 스캔을 그대로 발행하며,
    활성 상태로 전환되면 NFRI 실험 파라미터(76.7% 오탐률, ±30°)를
    적용한 degraded 스캔을 발행한다.
    """

    def __init__(self):
        super().__init__('lidar_degradation')

        # ── 파라미터 선언 ───────────────────────────────────────────────
        self.declare_parameter('robot_id', 'argos_1')

        # NFRI 실험 기반 기본값
        self.declare_parameter('noise_probability', 0.767)       # 76.7% 오탐률
        self.declare_parameter('noise_cone_deg', 30.0)           # 방수포 방향 ±30°
        self.declare_parameter('noise_range_min', 0.5)           # 가짜 반사점 최소 거리 (m)
        self.declare_parameter('noise_range_max', 3.0)           # 가짜 반사점 최대 거리 (m)
        self.declare_parameter('reliability_when_spraying', 0.233)  # 1 - 0.767

        # 방수포 정면 방향 오프셋 (로봇 기준 라디안, 0.0 = 전방)
        self.declare_parameter('cannon_angle_rad', 0.0)

        # ── 파라미터 로드 ───────────────────────────────────────────────
        self.robot_id = self.get_parameter('robot_id').value
        self.noise_probability = self.get_parameter('noise_probability').value
        self.noise_cone_rad = math.radians(
            self.get_parameter('noise_cone_deg').value)
        self.noise_range_min = self.get_parameter('noise_range_min').value
        self.noise_range_max = self.get_parameter('noise_range_max').value
        self.reliability_spraying = self.get_parameter(
            'reliability_when_spraying').value
        self.cannon_angle = self.get_parameter('cannon_angle_rad').value

        # ── 내부 상태 ───────────────────────────────────────────────────
        self.cannon_active: bool = False

        # ── QoS 설정 (센서 데이터: Best Effort) ────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        # ── 구독 ────────────────────────────────────────────────────────
        self.scan_sub = self.create_subscription(
            LaserScan,
            f'/{self.robot_id}/scan',
            self._scan_cb,
            sensor_qos,
        )
        self.cannon_sub = self.create_subscription(
            Bool,
            f'/{self.robot_id}/water_cannon/active',
            self._cannon_cb,
            10,
        )

        # ── 발행 ────────────────────────────────────────────────────────
        self.degraded_pub = self.create_publisher(
            LaserScan,
            f'/{self.robot_id}/scan_degraded',
            sensor_qos,
        )
        self.reliability_pub = self.create_publisher(
            Float32,
            f'/{self.robot_id}/lidar/reliability',
            10,
        )

        self.get_logger().info(
            f'LidarDegradationNode 초기화 완료 | robot_id={self.robot_id} '
            f'noise_prob={self.noise_probability:.3f} '
            f'cone={math.degrees(self.noise_cone_rad):.1f}°'
        )

    # ── 콜백 ───────────────────────────────────────────────────────────

    def _cannon_cb(self, msg: Bool) -> None:
        """방수포 상태 토픽 수신.

        상태 변화 시 로그 출력으로 시뮬레이션 이벤트 추적.
        """
        prev = self.cannon_active
        self.cannon_active = msg.data
        if prev != self.cannon_active:
            state_str = '활성 (노이즈 주입 시작)' if self.cannon_active else '비활성 (원본 복원)'
            self.get_logger().info(f'방수포 상태 변경: {state_str}')

    def _scan_cb(self, scan: LaserScan) -> None:
        """원본 LaserScan 수신 → 오탐 주입 후 발행."""
        # degraded 스캔 생성 및 발행
        degraded = self.degrade_scan(scan)
        self.degraded_pub.publish(degraded)

        # 신뢰도 토픽 발행
        reliability_msg = Float32()
        reliability_msg.data = (
            self.reliability_spraying if self.cannon_active else 1.0
        )
        self.reliability_pub.publish(reliability_msg)

    # ── 핵심 오탐 모델 ────────────────────────────────────────────────

    def degrade_scan(self, scan: LaserScan) -> LaserScan:
        """방수포 상태에 따라 LaserScan에 오탐 노이즈 주입.

        방수포 비활성 → 원본 스캔 그대로 반환 (deepcopy 없이 참조 반환).
        방수포 활성   → 방수포 방향 ±noise_cone_rad 범위 내 빔에 대해
                        noise_probability 확률로 가짜 근거리 반사점 삽입.

        NFRI 실험 모델:
          - 물입자가 LiDAR 광선을 조기 반사 → 실제 장애물보다 가까운 거리값 반환
          - 가짜 반사 거리: [noise_range_min, noise_range_max] 균등 분포

        Args:
            scan: 원본 LaserScan 메시지

        Returns:
            노이즈가 주입된 LaserScan (원본 불변 보장)
        """
        if not self.cannon_active:
            # 방수포 비활성: 원본 그대로 반환
            return scan

        degraded = copy.deepcopy(scan)
        n_ranges = len(degraded.ranges)

        if n_ranges == 0:
            return degraded

        # 방수포 방향 ±cone 범위 빔에 노이즈 주입
        for i in range(n_ranges):
            angle = scan.angle_min + i * scan.angle_increment

            # 각도 차이를 [-π, π] 범위로 정규화
            angle_diff = angle - self.cannon_angle
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

            if abs(angle_diff) < self.noise_cone_rad:
                # 76.7% 확률로 가짜 근거리 반사점 생성
                if random.random() < self.noise_probability:
                    degraded.ranges[i] = random.uniform(
                        self.noise_range_min, self.noise_range_max
                    )

        return degraded


def main(args=None):
    rclpy.init(args=args)
    node = LidarDegradationNode()
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
