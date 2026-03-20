#!/usr/bin/env python3
"""water_curtain_node.py — ARGOS HR-셰르파 자체분무(Water Curtain) + 방수포 시뮬레이션

NFRI HR-셰르파 제원 기반 물 소비·냉각 효과를 시뮬레이션한다.
실제 열화상 데이터 수정은 이 노드의 책임이 아니며,
cooling_factor만 발행하여 hotspot_detector 등 외부 노드가 적용하도록 한다.

자체분무 (Water Curtain):
  - 미분무 노즐 24EA (오리피스 1.8Φ)
  - 소모유량 36 LPM @10 bar
  - 차량 주변온도 50°C 이하 유지
  - cooling_factor = 0.7 (주변온도 30% 감소 효과)

방수포 (Water Cannon):
  - 최대 2,650 LPM @35 bar, 사거리 50m
  - cooling_factor = 0.9 (방수 방향 90% 냉각 효과)

물 공급 모델:
  - 호스 연결(기본): 외부 소화전 공급 → 탱크 무제한
  - 호스 미연결: 자체 수조 200L → 약 5.5분 (분무만) 또는 ~4.5초 (방수포만)

발행 토픽:
  /{robot_id}/water_curtain/active   (std_msgs/Bool)   — 자체분무 ON/OFF
  /{robot_id}/water_cannon/active    (std_msgs/Bool)   — 방수포 ON/OFF
  /{robot_id}/water/flow_rate        (std_msgs/Float32) — 현재 총 유량 (LPM)
  /{robot_id}/water/remaining_time   (std_msgs/Float32) — 잔여 방수 시간 (분, 호스 미연결 시)
  /{robot_id}/thermal_cooling_factor (std_msgs/Float32) — 냉각 계수 (0.0 ~ 1.0)

서비스:
  /{robot_id}/water_curtain/activate (std_srvs/SetBool) — 자체분무 ON/OFF
  /{robot_id}/water_cannon/activate  (std_srvs/SetBool) — 방수포 ON/OFF

파라미터:
  curtain_flow_lpm    (float): 자체분무 유량 (기본 36.0 LPM)
  cannon_flow_lpm     (float): 방수포 유량 (기본 2650.0 LPM)
  internal_tank_liters(float): 자체 수조 용량 (기본 200.0 L)
  hose_connected      (bool) : 외부 호스 연결 여부 (기본 True)
  robot_id            (str)  : 로봇 ID — 빈 문자열이면 namespace 자동 추출
  publish_rate_hz     (float): 상태 발행 주파수 (기본 10.0 Hz)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from std_srvs.srv import SetBool


# 자체분무 냉각 계수 — 미분무로 30% 온도 감소
_CURTAIN_COOLING_FACTOR = 0.7

# 방수포 냉각 계수 — 고압 방수로 90% 냉각
_CANNON_COOLING_FACTOR = 0.9

# 탱크 고갈 경고 임계값 (잔여 시간, 분)
_LOW_WATER_WARN_MIN = 1.0


class WaterCurtainNode(Node):
    """HR-셰르파 자체분무 및 방수포 시뮬레이션 노드.

    서비스로 ON/OFF 제어를 받고, 물 소비량·냉각 계수를 주기적으로 발행한다.
    thermal 데이터 직접 수정은 하지 않으며 cooling_factor만 제공한다.
    """

    def __init__(self):
        super().__init__('water_curtain_node')

        # ── 파라미터 선언 및 읽기 ────────────────────────────────────────────
        self.declare_parameter('robot_id', '')
        self.declare_parameter('curtain_flow_lpm', 36.0)
        self.declare_parameter('cannon_flow_lpm', 2650.0)
        self.declare_parameter('internal_tank_liters', 200.0)
        self.declare_parameter('hose_connected', True)
        self.declare_parameter('publish_rate_hz', 10.0)

        # robot_id: 파라미터 → namespace 자동 추출 순서
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

        self._curtain_flow_lpm: float = self.get_parameter('curtain_flow_lpm').value
        self._cannon_flow_lpm: float = self.get_parameter('cannon_flow_lpm').value
        self._internal_tank_liters: float = self.get_parameter('internal_tank_liters').value
        self._hose_connected: bool = self.get_parameter('hose_connected').value
        publish_rate_hz: float = self.get_parameter('publish_rate_hz').value

        # ── 내부 상태 ────────────────────────────────────────────────────────
        # 활성화 여부
        self._curtain_active: bool = False
        self._cannon_active: bool = False

        # 자체 수조 잔여량 (L) — 호스 미연결 시에만 소모
        self._tank_remaining_liters: float = self._internal_tank_liters

        # ── 서비스 ───────────────────────────────────────────────────────────
        self._curtain_srv = self.create_service(
            SetBool,
            f'/{robot_id}/water_curtain/activate',
            self._curtain_activate_callback,
        )
        self._cannon_srv = self.create_service(
            SetBool,
            f'/{robot_id}/water_cannon/activate',
            self._cannon_activate_callback,
        )

        # ── 발행자 ───────────────────────────────────────────────────────────
        self._curtain_state_pub = self.create_publisher(
            Bool,
            f'/{robot_id}/water_curtain/active',
            10,
        )
        self._cannon_state_pub = self.create_publisher(
            Bool,
            f'/{robot_id}/water_cannon/active',
            10,
        )
        self._flow_rate_pub = self.create_publisher(
            Float32,
            f'/{robot_id}/water/flow_rate',
            10,
        )
        self._remaining_time_pub = self.create_publisher(
            Float32,
            f'/{robot_id}/water/remaining_time',
            10,
        )
        self._cooling_factor_pub = self.create_publisher(
            Float32,
            f'/{robot_id}/thermal_cooling_factor',
            10,
        )

        # ── 발행 타이머 ──────────────────────────────────────────────────────
        self._pub_timer = self.create_timer(
            1.0 / publish_rate_hz, self._publish_status
        )

        self.get_logger().info(
            f'[WaterCurtain:{robot_id}] 초기화 완료 '
            f'(curtain={self._curtain_flow_lpm:.0f}LPM, '
            f'cannon={self._cannon_flow_lpm:.0f}LPM, '
            f'tank={self._internal_tank_liters:.0f}L, '
            f'hose_connected={self._hose_connected})'
        )

    # ──────────────────────────────────────────────────────────────────────────
    # 서비스 콜백
    # ──────────────────────────────────────────────────────────────────────────

    def _curtain_activate_callback(
        self,
        request: SetBool.Request,
        response: SetBool.Response,
    ) -> SetBool.Response:
        """/{robot_id}/water_curtain/activate 서비스 콜백.

        호스 미연결 + 탱크 고갈 시 활성화 거부.
        """
        if request.data and not self._hose_connected and self._tank_remaining_liters <= 0.0:
            response.success = False
            response.message = '자체 수조 고갈 — 자체분무 활성화 불가'
            self.get_logger().error(
                f'[{self._robot_id}] 자체분무 활성화 실패: 수조 고갈'
            )
            return response

        prev = self._curtain_active
        self._curtain_active = request.data
        action = 'ON' if request.data else 'OFF'
        response.success = True
        response.message = f'자체분무 {action}'

        if prev != self._curtain_active:
            self.get_logger().info(
                f'[{self._robot_id}] 자체분무 {action} '
                f'(유량={self._curtain_flow_lpm:.0f}LPM)'
            )
        return response

    def _cannon_activate_callback(
        self,
        request: SetBool.Request,
        response: SetBool.Response,
    ) -> SetBool.Response:
        """/{robot_id}/water_cannon/activate 서비스 콜백.

        호스 미연결 + 탱크 고갈 시 활성화 거부.
        방수포는 유량이 커서 호스 미연결 상태에서는 수초 만에 탱크 고갈 주의.
        """
        if request.data and not self._hose_connected and self._tank_remaining_liters <= 0.0:
            response.success = False
            response.message = '자체 수조 고갈 — 방수포 활성화 불가'
            self.get_logger().error(
                f'[{self._robot_id}] 방수포 활성화 실패: 수조 고갈'
            )
            return response

        prev = self._cannon_active
        self._cannon_active = request.data
        action = 'ON' if request.data else 'OFF'
        response.success = True
        response.message = f'방수포 {action}'

        if prev != self._cannon_active:
            self.get_logger().info(
                f'[{self._robot_id}] 방수포 {action} '
                f'(유량={self._cannon_flow_lpm:.0f}LPM, 사거리=50m)'
            )
        return response

    # ──────────────────────────────────────────────────────────────────────────
    # 물 소비 모델
    # ──────────────────────────────────────────────────────────────────────────

    def _compute_flow_rate(self) -> float:
        """현재 총 유량 (LPM) 계산.

        자체분무와 방수포가 동시에 활성화될 수 있다.
        """
        flow = 0.0
        if self._curtain_active:
            flow += self._curtain_flow_lpm
        if self._cannon_active:
            flow += self._cannon_flow_lpm
        return flow

    def _consume_tank(self, flow_lpm: float, dt_sec: float) -> None:
        """호스 미연결 시 자체 수조 소모.

        Args:
            flow_lpm: 현재 유량 (LPM)
            dt_sec:   경과 시간 (초)
        """
        if self._hose_connected or flow_lpm <= 0.0:
            return

        consumed_liters = flow_lpm * (dt_sec / 60.0)
        self._tank_remaining_liters = max(0.0, self._tank_remaining_liters - consumed_liters)

        # 탱크 고갈 시 자동 OFF
        if self._tank_remaining_liters <= 0.0:
            if self._curtain_active or self._cannon_active:
                self._curtain_active = False
                self._cannon_active = False
                self.get_logger().error(
                    f'[{self._robot_id}] 자체 수조 고갈 — 자체분무·방수포 강제 OFF'
                )

    def _compute_remaining_time_min(self, flow_lpm: float) -> float:
        """잔여 방수 시간 계산 (분).

        호스 연결 시: -1.0 반환 (무제한 의미)
        방수 비활성: -1.0 반환 (시간 개념 없음)
        """
        if self._hose_connected:
            return -1.0
        if flow_lpm <= 0.0:
            # 정지 상태에서 수조 잔여량 기준 예상 분무 가능 시간
            total_flow = self._curtain_flow_lpm + self._cannon_flow_lpm
            if total_flow <= 0.0:
                return -1.0
            return self._tank_remaining_liters / total_flow
        return self._tank_remaining_liters / flow_lpm

    # ──────────────────────────────────────────────────────────────────────────
    # 냉각 계수 계산
    # ──────────────────────────────────────────────────────────────────────────

    def _compute_cooling_factor(self) -> float:
        """현재 냉각 계수 계산 (0.0 ~ 1.0).

        방수포가 우선 적용 (더 높은 냉각 효과).
        두 시스템 동시 활성 시 방수포 계수 사용.

        Returns:
            0.0 — 냉각 없음
            0.7 — 자체분무만 활성 (주변 30% 냉각)
            0.9 — 방수포 활성 (방수 방향 90% 냉각)
        """
        if self._cannon_active:
            return _CANNON_COOLING_FACTOR
        if self._curtain_active:
            return _CURTAIN_COOLING_FACTOR
        return 0.0

    # ──────────────────────────────────────────────────────────────────────────
    # 발행
    # ──────────────────────────────────────────────────────────────────────────

    def _publish_status(self) -> None:
        """타이머 콜백 — 모든 상태 토픽 발행 + 수조 소모 처리."""
        # 타이머 주기를 dt로 사용
        dt_sec = self._pub_timer.timer_period_ns / 1e9

        # 유량 계산 및 수조 소모
        flow_lpm = self._compute_flow_rate()
        self._consume_tank(flow_lpm, dt_sec)

        # 잔여 시간 계산
        remaining_min = self._compute_remaining_time_min(flow_lpm)

        # 냉각 계수 계산
        cooling = self._compute_cooling_factor()

        # ── 자체분무 상태 발행 ────────────────────────────────────────────────
        curtain_msg = Bool()
        curtain_msg.data = self._curtain_active
        self._curtain_state_pub.publish(curtain_msg)

        # ── 방수포 상태 발행 ─────────────────────────────────────────────────
        cannon_msg = Bool()
        cannon_msg.data = self._cannon_active
        self._cannon_state_pub.publish(cannon_msg)

        # ── 총 유량 발행 ─────────────────────────────────────────────────────
        flow_msg = Float32()
        flow_msg.data = float(flow_lpm)
        self._flow_rate_pub.publish(flow_msg)

        # ── 잔여 방수 시간 발행 ───────────────────────────────────────────────
        remaining_msg = Float32()
        remaining_msg.data = float(remaining_min)
        self._remaining_time_pub.publish(remaining_msg)

        # ── 냉각 계수 발행 ───────────────────────────────────────────────────
        cooling_msg = Float32()
        cooling_msg.data = float(cooling)
        self._cooling_factor_pub.publish(cooling_msg)

        # ── 수조 부족 경고 (호스 미연결, 방수 중, 잔여 1분 미만) ────────────
        if (
            not self._hose_connected
            and flow_lpm > 0.0
            and 0.0 < remaining_min < _LOW_WATER_WARN_MIN
        ):
            self.get_logger().warn(
                f'[{self._robot_id}] 자체 수조 부족: '
                f'잔여={self._tank_remaining_liters:.1f}L '
                f'({remaining_min:.1f}분)',
                throttle_duration_sec=10.0,
            )


# ──────────────────────────────────────────────────────────────────────────────
# 진입점
# ──────────────────────────────────────────────────────────────────────────────

def main(args=None):
    """노드 진입점."""
    rclpy.init(args=args)
    node = WaterCurtainNode()
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
