#!/usr/bin/env python3
"""audio_detector_node.py — ARGOS 음향 감지 노드

화재 현장의 음향 이벤트를 감지하여 위험과 생존자를 탐지.
시각 센서의 사각지대(벽 뒤, 연기 속, 어둠)를 보완.

감지 대상:
  - explosion: 폭발음 (저주파, 고강도)
  - gas_leak: 가스 누출 (고주파 쉿 소리)
  - cry_for_help: 구조 요청 (음성 대역 300-3400Hz)
  - collapse: 붕괴음 (중저주파, 연속)
  - glass_break: 유리 파손 (고주파 임펄스)
  - alarm: 화재 경보기 (주기적 고주파)
  - breathing: 호흡음 (근거리, 저강도)

시뮬레이션 모드: 이벤트 위치 기반 거리+방향 가상 감지.
실제 모드: 마이크 어레이 + 음향 분류 AI 모델.
"""
import math
import time as time_module
import rclpy
from rclpy.node import Node

from argos_interfaces.msg import AudioEvent
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry


# 음향 이벤트 특성 정의
AUDIO_PROFILES = {
    'explosion': {
        'freq_range': (20, 200), 'intensity_range': (100, 140),
        'duration_range': (0.1, 2.0), 'danger': 'critical',
        'detection_range': 50.0, 'immediate': True
    },
    'gas_leak': {
        'freq_range': (2000, 8000), 'intensity_range': (40, 70),
        'duration_range': (5.0, 999.0), 'danger': 'danger',
        'detection_range': 15.0, 'immediate': True
    },
    'cry_for_help': {
        'freq_range': (300, 3400), 'intensity_range': (60, 90),
        'duration_range': (0.5, 5.0), 'danger': 'critical',
        'detection_range': 30.0, 'immediate': True
    },
    'collapse': {
        'freq_range': (50, 500), 'intensity_range': (80, 120),
        'duration_range': (1.0, 10.0), 'danger': 'danger',
        'detection_range': 40.0, 'immediate': True
    },
    'glass_break': {
        'freq_range': (3000, 12000), 'intensity_range': (70, 100),
        'duration_range': (0.05, 0.5), 'danger': 'caution',
        'detection_range': 20.0, 'immediate': False
    },
    'alarm': {
        'freq_range': (2000, 4000), 'intensity_range': (80, 100),
        'duration_range': (0.5, 2.0), 'danger': 'info',
        'detection_range': 30.0, 'immediate': False
    },
    'breathing': {
        'freq_range': (100, 1000), 'intensity_range': (20, 40),
        'duration_range': (1.0, 4.0), 'danger': 'critical',
        'detection_range': 2.0, 'immediate': True  # 호흡음은 2m 이내에서만 감지 가능
    },
}

AUDIO_DETECTION_COOLDOWN_SEC = 5.0  # 동일 음원 재감지 쿨다운
AUDIO_STALE_THRESHOLD_SEC = 300.0   # 5분 이상 미감지 이력 삭제


class AudioDetectorNode(Node):
    """음향 이벤트 감지 + 방향 추정 + 위험도 판정."""

    def __init__(self):
        super().__init__('audio_detector')

        self.declare_parameter('robot_id', 'argos_1')
        self.declare_parameter('simulation_mode', True)
        self.declare_parameter('scan_interval', 1.0)
        # 시뮬: 음원 위치와 타입 [x1,y1,"type1", x2,y2,"type2", ...]
        self.declare_parameter('sound_sources', [])

        self.robot_id = self.get_parameter('robot_id').value
        self.sim_mode = self.get_parameter('simulation_mode').value

        # 네임스페이스에서 robot_id 자동 추출
        ns = self.get_namespace().strip('/')
        if ns:
            self.robot_id = ns

        self.robot_position = None
        self.robot_yaw = 0.0

        # 시뮬레이션 음원 파싱
        self.sound_sources = []
        raw = self.get_parameter('sound_sources').value
        if raw:
            i = 0
            while i < len(raw) - 2:
                try:
                    x, y = float(raw[i]), float(raw[i+1])
                    stype = str(raw[i+2])
                    self.sound_sources.append((x, y, stype))
                    i += 3
                except (ValueError, IndexError):
                    break

        # 감지 이력 (쿨다운)
        self.last_detection = {}  # source_id -> timestamp

        # 구독
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self._odom_cb, 10)

        # 발행
        self.audio_pub = self.create_publisher(
            AudioEvent, 'audio/events', 10)

        # 타이머
        interval = self.get_parameter('scan_interval').value
        self.timer = self.create_timer(interval, self._scan_audio)

        self.get_logger().info(
            f'[{self.robot_id}] Audio detector initialized '
            f'(sim={self.sim_mode}, sources={len(self.sound_sources)})')

    def _odom_cb(self, msg):
        self.robot_position = msg.pose.pose.position
        # 쿼터니언 → yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _scan_audio(self):
        """음향 환경 스캔 — 감지 범위 내 음원 탐지."""
        if self.robot_position is None:
            return

        if self.sim_mode:
            self._simulate_audio()

    def _simulate_audio(self):
        """시뮬레이션: 음원 위치 기반 가상 음향 감지."""
        now = time_module.time()

        for i, (sx, sy, stype) in enumerate(self.sound_sources):
            if stype not in AUDIO_PROFILES:
                continue

            profile = AUDIO_PROFILES[stype]
            dist = math.sqrt(
                (self.robot_position.x - sx)**2 +
                (self.robot_position.y - sy)**2)

            detection_range = profile['detection_range']
            if dist > detection_range:
                continue

            # 쿨다운
            source_id = f'src_{i}_{stype}'
            if source_id in self.last_detection:
                if now - self.last_detection[source_id] < AUDIO_DETECTION_COOLDOWN_SEC:
                    continue
            self.last_detection[source_id] = now

            # 방향 계산 (로봇 기준)
            angle_to_source = math.atan2(
                sy - self.robot_position.y,
                sx - self.robot_position.x)
            relative_angle = angle_to_source - self.robot_yaw

            # 거리 기반 강도 감쇠
            intensity_range = profile['intensity_range']
            base_intensity = (intensity_range[0] + intensity_range[1]) / 2
            # 역제곱법칙 적용
            perceived_intensity = base_intensity - 20 * math.log10(max(1.0, dist))

            # 거리 기반 신뢰도
            confidence = max(0.2, 1.0 - (dist / detection_range) ** 0.5)

            # 이벤트 발행
            event = AudioEvent()
            event.header.stamp = self.get_clock().now().to_msg()
            event.header.frame_id = 'map'
            event.robot_id = self.robot_id
            event.event_type = stype
            event.confidence = confidence
            event.intensity_db = perceived_intensity
            event.frequency_hz = float(sum(profile['freq_range']) / 2)
            event.duration_sec = float(sum(profile['duration_range']) / 2)
            event.direction_rad = relative_angle

            event.estimated_location = PointStamped()
            event.estimated_location.header = event.header
            event.estimated_location.point.x = sx
            event.estimated_location.point.y = sy

            event.danger_level = profile['danger']
            event.immediate_response_needed = profile['immediate']

            self.audio_pub.publish(event)

            log_fn = (self.get_logger().warn if profile['immediate']
                      else self.get_logger().info)
            log_fn(
                f'AUDIO [{stype}] dist={dist:.1f}m '
                f'dir={math.degrees(relative_angle):.0f}deg '
                f'intensity={perceived_intensity:.0f}dB '
                f'danger={profile["danger"]}')


def main(args=None):
    rclpy.init(args=args)
    node = AudioDetectorNode()
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
