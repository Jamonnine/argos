"""scan_frame_relay.py — /scan의 frame_id를 base_footprint로 변환

slam_toolbox가 lidar_link TF를 찾지 못하는 WSL2/Nav2 통합 환경 우회 노드.

문제 원인:
  - slam_toolbox MessageFilter: /scan(frame_id=lidar_link) 수신 후
    TF 트리에서 map→odom→base_footprint→lidar_link 체인을 조회
  - Nav2 전체 스택 기동 시 DDC의 odom→base_footprint TF 발행 지연이
    MessageFilter 큐를 채워 "queue is full" 반복 발생
  - standalone Gazebo에서는 정상(tf2_echo 확인됨), Nav2와 함께면 실패

해결 전략:
  - /scan (frame_id: lidar_link) → /scan_base (frame_id: base_footprint) 릴레이
  - slam_toolbox는 base_footprint 프레임만 조회 → DDC가 직접 발행하므로 안정적
  - lidar_link→base_footprint 오프셋: z=0.295m(ARGOS URDF 기준), 2D SLAM에서는 무영향
  - 기존 URDF/launch 변경 없이 파라미터 레벨에서 우회

사용:
  navigation.launch.py에서 DDC 완료 후 이 노드를 먼저 기동.
  slam_toolbox의 scan_topic: scan_base로 변경 필요 (nav2_params.yaml).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan


class ScanFrameRelay(Node):
    """LaserScan frame_id를 lidar_link에서 base_footprint로 변환하는 릴레이 노드."""

    def __init__(self):
        super().__init__('scan_frame_relay')

        # 파라미터 선언: 소스/대상 토픽과 변환할 프레임 ID
        self.declare_parameter('input_topic', 'scan')
        self.declare_parameter('output_topic', 'scan_base')
        self.declare_parameter('target_frame', 'base_footprint')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self._target_frame = self.get_parameter('target_frame').get_parameter_value().string_value

        # namespace 프리픽스 지원: 멀티로봇 환경(argos1/, argos2/) 대응
        ns = self.get_namespace().lstrip('/')
        if ns:
            self._target_frame = f'{ns}/{self._target_frame}'

        # LaserScan QoS: sensor_data(BEST_EFFORT, KEEP_LAST 5)
        # slam_toolbox는 기본적으로 BEST_EFFORT로 scan을 구독
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self._pub = self.create_publisher(LaserScan, output_topic, qos)
        self._sub = self.create_subscription(
            LaserScan,
            input_topic,
            self._relay_callback,
            qos,
        )

        self.get_logger().info(
            f'ScanFrameRelay 시작: {input_topic} → {output_topic} '
            f'(frame_id → {self._target_frame})'
        )

    def _relay_callback(self, msg: LaserScan) -> None:
        """frame_id만 교체하여 재발행. 스캔 데이터 자체는 무변경."""
        # 얕은 복사로 헤더만 교체 (데이터 배열 재할당 없음 → 지연 최소화)
        msg.header.frame_id = self._target_frame
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFrameRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
