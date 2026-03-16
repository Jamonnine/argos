#!/usr/bin/env python3
"""gas_sensor_node.py — ARGOS 가스 센서 노드

화재 현장의 유독가스(CO/O2/LEL/CO2/HCN)를 측정하여
오케스트레이터에 위험도를 보고하는 노드.

시뮬레이션 모드: 화재 위치 기반 가스 확산 모델로 가상 측정.
실제 모드: 하드웨어 가스 센서(MQ-7, MQ-135 등)에서 읽기.

소방 현장 가스 기준 (NIOSH/OSHA):
  CO:  50ppm=주의, 200ppm=위험, 800ppm=즉시철수, 1200ppm=치명
  O2:  19.5%=정상하한, 18%=질식위험, 16%=의식상실
  LEL: 10%=주의, 25%=즉시철수 (폭발하한의 %)
  HCN: 10ppm=주의, 50ppm=위험 (플라스틱 연소 시)

LifecycleNode 상태:
  unconfigured → configure() → inactive
  inactive     → activate()  → active (구독/발행/타이머 활성)
  active       → deactivate() → inactive (타이머·토픽 중단)
  inactive     → cleanup()   → unconfigured
"""
import math
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, State
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from argos_interfaces.msg import GasReading, FireAlert
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry


# 가스 위험 기준치 (소방 현장 표준, NIOSH/OSHA)
GAS_THRESHOLDS = {
    'co': {'safe': 25, 'caution': 50, 'danger': 200, 'critical': 800},
    'o2_low': {'safe': 19.5, 'caution': 18.0, 'danger': 16.0, 'critical': 14.0},
    'lel': {'safe': 5, 'caution': 10, 'danger': 25, 'critical': 50},
    'hcn': {'safe': 4.7, 'caution': 10, 'danger': 50, 'critical': 100},
}

# 가스 확산 최대 농도 (화원 근접 시)
GAS_MAX_CONCENTRATION = {
    'co': 1500.0,       # ppm — 화원 직근 CO
    'o2_depletion': 6.0, # % — 산소 소모량
    'lel': 30.0,         # % — 가연성 가스
    'co2': 10000.0,      # ppm — CO2 (일반 화재 1~3%)
    'hcn': 80.0,         # ppm — HCN (플라스틱 연소)
}

FIRE_LOCATION_DEDUP_DISTANCE = 5.0  # 미터, 동일 화점 판정 거리


class GasSensorNode(LifecycleNode):
    """가스 센서 노드 — CO/O2/LEL/CO2/HCN 측정 + 위험도 판정.

    LifecycleNode 패턴:
    - __init__  : 파라미터 선언만 수행 (토픽·타이머 생성 금지)
    - on_configure  : 파라미터 읽기 + 상태 초기화
    - on_activate   : 구독/발행/타이머 생성 → 측정 시작
    - on_deactivate : 타이머 취소, 구독/발행 해제 → 측정 중단
    - on_cleanup    : 내부 상태 리셋
    """

    def __init__(self):
        super().__init__('gas_sensor')

        # ── 파라미터 선언만 (activate 전에는 토픽·타이머 생성 금지) ──
        self.declare_parameter('robot_id', 'argos_1')
        self.declare_parameter('sensor_frame', 'gas_sensor_link')
        self.declare_parameter('publish_rate', 2.0)  # Hz
        self.declare_parameter('simulation_mode', True)
        # 가스 확산 파라미터 (시뮬레이션용)
        self.declare_parameter('gas_decay_rate', 0.3)  # 거리에 따른 감쇠율
        self.declare_parameter('ambient_co', 0.0)
        self.declare_parameter('ambient_o2', 20.9)

        # 런타임 상태 (on_configure에서 초기화)
        self.robot_id = None
        self.sensor_frame = None
        self.sim_mode = None
        self.decay_rate = None
        self.robot_position = None
        self.fire_locations = []

        # 구독/발행/타이머 핸들 (on_activate에서 생성, on_deactivate에서 해제)
        self.odom_sub = None
        self.fire_sub = None
        self.gas_pub = None
        self.timer = None
        self._exclusive_group = None

    # ─────────────────── Lifecycle Callbacks ───────────────────

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """파라미터 읽기 + 내부 상태 초기화.

        inactive 상태로 진입 전 호출됨. 여기서 ROS 통신은 만들지 않는다.
        """
        self.robot_id = self.get_parameter('robot_id').value
        self.sensor_frame = self.get_parameter('sensor_frame').value
        self.sim_mode = self.get_parameter('simulation_mode').value
        self.decay_rate = self.get_parameter('gas_decay_rate').value

        # 네임스페이스에서 robot_id 자동 추출 (멀티로봇 호환)
        ns = self.get_namespace().strip('/')
        if ns:
            self.robot_id = ns

        self.robot_position = None
        self.fire_locations = []

        # 파라미터 동적 갱신 콜백 등록
        self.add_on_set_parameters_callback(self._on_parameter_change)

        self.get_logger().info(
            f'[{self.robot_id}] Gas sensor configured '
            f'(sim={self.sim_mode})')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """구독/발행/타이머 생성 → 측정 시작.

        active 상태로 진입 전 호출됨.
        """
        # QoS — 가스 데이터는 critical이므로 RELIABLE 발행
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        # 콜백 그룹 (경쟁 조건 방지)
        self._exclusive_group = MutuallyExclusiveCallbackGroup()

        # 구독
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self._odom_cb, 10,
            callback_group=self._exclusive_group)
        self.fire_sub = self.create_subscription(
            FireAlert, '/orchestrator/fire_alert', self._fire_cb, reliable_qos)

        # 발행 — RELIABLE QoS (가스 데이터 손실 방지)
        self.gas_pub = self.create_publisher(
            GasReading, 'gas/reading', reliable_qos)

        # 타이머 (발행 빈도 상한 20Hz — DoS 방지)
        MAX_PUBLISH_RATE = 20.0
        rate = min(self.get_parameter('publish_rate').value, MAX_PUBLISH_RATE)
        self.timer = self.create_timer(
            1.0 / rate, self._measure_and_publish,
            callback_group=self._exclusive_group)

        self.get_logger().info(
            f'[{self.robot_id}] Gas sensor activated (rate={rate}Hz)')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """타이머 취소 + 구독/발행 해제 → 측정 중단.

        inactive 상태로 진입 전 호출됨.
        """
        # 타이머 취소
        if self.timer is not None:
            self.timer.cancel()
            self.destroy_timer(self.timer)
            self.timer = None

        # 구독 해제
        if self.odom_sub is not None:
            self.destroy_subscription(self.odom_sub)
            self.odom_sub = None
        if self.fire_sub is not None:
            self.destroy_subscription(self.fire_sub)
            self.fire_sub = None

        # 발행자 해제
        if self.gas_pub is not None:
            self.destroy_publisher(self.gas_pub)
            self.gas_pub = None

        self.get_logger().info(f'[{self.robot_id}] Gas sensor deactivated')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """내부 상태 리셋 → unconfigured로 복귀."""
        self.robot_position = None
        self.fire_locations = []
        self._exclusive_group = None
        self.get_logger().info(f'[{self.robot_id}] Gas sensor cleaned up')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """종료 시 정리."""
        self.get_logger().info(f'[{self.robot_id}] Gas sensor shutting down')
        return TransitionCallbackReturn.SUCCESS

    # ─────────────────── Parameter Callback ───────────────────

    def _on_parameter_change(self, params):
        """파라미터 런타임 변경 시 콜백."""
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'gas_decay_rate':
                self.decay_rate = p.value
                self.get_logger().info(f'gas_decay_rate updated: {p.value}')
            elif p.name == 'publish_rate':
                # 타이머 재생성은 복잡하므로 로그만
                self.get_logger().warn(
                    f'publish_rate changed to {p.value} — restart required')
        return SetParametersResult(successful=True)

    # ─────────────────── Topic Callbacks ───────────────────

    def _odom_cb(self, msg):
        self.robot_position = msg.pose.pose.position

    def _fire_cb(self, msg):
        if msg.active:
            loc = msg.location.point
            # 중복 방지 (5m 이내 기존 화점 있으면 무시)
            for existing in self.fire_locations:
                if existing is None:
                    continue
                dist = math.sqrt(
                    (loc.x - existing.x)**2 + (loc.y - existing.y)**2)
                if dist < FIRE_LOCATION_DEDUP_DISTANCE:
                    return
            self.fire_locations.append(loc)
            self.get_logger().info(
                f'Fire source registered at ({loc.x:.1f}, {loc.y:.1f})')

    def _measure_and_publish(self):
        if self.robot_position is None:
            self.get_logger().debug(
                'Robot position not yet available', throttle_duration_sec=5.0)
            return

        try:
            if self.sim_mode:
                reading = self._simulate_gas()
            else:
                reading = self._read_hardware()
        except Exception as e:
            self.get_logger().error(
                f'Gas measurement failed: {e}', throttle_duration_sec=5.0)
            return

        if reading is None:
            return

        self.gas_pub.publish(reading)

        # 위험 시 경고 로그
        if reading.danger_level in ('danger', 'critical'):
            self.get_logger().warn(
                f'GAS {reading.danger_level.upper()}: '
                f'CO={reading.co_ppm:.0f}ppm O2={reading.o2_percent:.1f}% '
                f'LEL={reading.lel_percent:.0f}% HCN={reading.hcn_ppm:.0f}ppm '
                f'evacuate={reading.evacuate_recommended}')

    def _simulate_gas(self) -> GasReading:
        """화재 위치 기반 가스 확산 시뮬레이션.

        가스 농도 = 기저값 + sum(각 화원에서의 기여)
        기여 = 최대농도 * exp(-decay * distance)
        """
        msg = GasReading()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.sensor_frame
        msg.robot_id = self.robot_id

        # 기저값 (정상 대기)
        co = self.get_parameter('ambient_co').value
        o2 = self.get_parameter('ambient_o2').value
        lel = 0.0
        co2 = 400.0  # 대기 중 CO2 ~400ppm
        hcn = 0.0

        # 각 화원에서의 가스 기여
        for fire_loc in self.fire_locations:
            if fire_loc is None:
                continue
            try:
                dist = math.sqrt(
                    (self.robot_position.x - fire_loc.x)**2 +
                    (self.robot_position.y - fire_loc.y)**2)
            except (AttributeError, TypeError):
                continue

            # 거리 기반 감쇠 (지수 함수)
            intensity = math.exp(-self.decay_rate * dist)

            co += GAS_MAX_CONCENTRATION['co'] * intensity
            o2 -= GAS_MAX_CONCENTRATION['o2_depletion'] * intensity
            lel += GAS_MAX_CONCENTRATION['lel'] * intensity
            co2 += GAS_MAX_CONCENTRATION['co2'] * intensity  # 10000ppm = 1%
            hcn += GAS_MAX_CONCENTRATION['hcn'] * intensity

        # 센서 노이즈 추가 (시뮬 전문가 권고: ±5% 상대 노이즈)
        import random
        noise_factor = 0.05  # 5% 상대 오차
        co += random.gauss(0, max(1.0, co * noise_factor))
        o2 += random.gauss(0, 0.3)  # O2: ±0.3% 절대 오차
        lel += random.gauss(0, max(0.5, lel * noise_factor))
        co2 += random.gauss(0, max(10.0, co2 * noise_factor))
        hcn += random.gauss(0, max(0.5, hcn * noise_factor))

        # 값 클램핑
        msg.co_ppm = max(0.0, co)
        msg.o2_percent = max(0.0, min(20.9, o2))
        msg.lel_percent = max(0.0, min(100.0, lel))
        msg.co2_ppm = max(0.0, co2)
        msg.hcn_ppm = max(0.0, hcn)

        # 위치
        msg.location = PointStamped()
        msg.location.header = msg.header
        msg.location.header.frame_id = 'map'
        msg.location.point = self.robot_position

        # 위험도 판정
        msg.danger_level, msg.hazard_types, msg.evacuate_recommended = \
            self._assess_danger(msg)

        return msg

    def _read_hardware(self) -> GasReading:
        """하드웨어 가스 센서 읽기 (미구현 — 센서 연결 시 구현)."""
        msg = GasReading()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.robot_id = self.robot_id
        msg.danger_level = 'safe'
        msg.evacuate_recommended = False
        self.get_logger().warn('Hardware gas sensor not implemented', once=True)
        return msg

    def _assess_danger(self, msg) -> tuple:
        """가스 농도 기반 위험도 종합 판정."""
        hazards = []
        max_level = 'safe'
        levels_order = ['safe', 'caution', 'danger', 'critical']

        # CO 판정
        for level in reversed(levels_order):
            if level == 'safe':
                continue
            if msg.co_ppm >= GAS_THRESHOLDS['co'][level]:
                hazards.append(f'co_{level}')
                if levels_order.index(level) > levels_order.index(max_level):
                    max_level = level
                break

        # O2 판정 (역방향 — 낮을수록 위험)
        for level in reversed(levels_order):
            if level == 'safe':
                continue
            if msg.o2_percent <= GAS_THRESHOLDS['o2_low'][level]:
                hazards.append(f'o2_{level}')
                if levels_order.index(level) > levels_order.index(max_level):
                    max_level = level
                break

        # LEL 판정
        for level in reversed(levels_order):
            if level == 'safe':
                continue
            if msg.lel_percent >= GAS_THRESHOLDS['lel'][level]:
                hazards.append(f'lel_{level}')
                if levels_order.index(level) > levels_order.index(max_level):
                    max_level = level
                break

        # HCN 판정
        for level in reversed(levels_order):
            if level == 'safe':
                continue
            if msg.hcn_ppm >= GAS_THRESHOLDS['hcn'][level]:
                hazards.append(f'hcn_{level}')
                if levels_order.index(level) > levels_order.index(max_level):
                    max_level = level
                break

        # 즉시 철수 조건
        evacuate = (
            max_level == 'critical' or
            msg.co_ppm >= 800 or
            msg.o2_percent <= 16.0 or
            msg.lel_percent >= 25.0
        )

        return max_level, hazards, evacuate


def main(args=None):
    rclpy.init(args=args)
    node = GasSensorNode()

    # LifecycleNode는 외부(lifecycle_manager 또는 ros2 lifecycle)에서
    # configure → activate 전환을 직접 호출해야 active 상태가 된다.
    # 단독 실행 시에는 수동으로 전환한다.
    node.trigger_configure()
    node.trigger_activate()

    try:
        rclpy.spin(node)
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
