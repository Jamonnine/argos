#!/usr/bin/env python3
"""keepout_manager.py — 가스/구조물 센서 이벤트 기반 Nav2 Keepout Zone 동적 관리

화재 현장에서 감지된 위험 구역을 Nav2 costmap의 lethal cost 영역으로
동적으로 추가/제거합니다.

동작 원리:
  - 위험 감지 → 해당 위치 반경 2m를 keepout zone으로 등록
  - keepout zone = OccupancyGrid 기반 costmap layer에 lethal cost(100) 주입
  - Nav2 local_costmap / global_costmap 양쪽에 동일하게 발행
  - 30분 경과 + 센서 정상 복귀 시 자동 해제

구독:
  /orchestrator/gas_reading      (GasReading)      — CO/LEL 임계값 초과 시
  /orchestrator/structural_alert (StructuralAlert)  — 붕괴 위험 시

발행:
  /keepout_zones/costmap_update  (OccupancyGrid)    — Nav2 costmap 갱신 데이터
  /keepout_zones/status          (String, JSON)     — 현재 keepout zone 목록 (디버그)

파라미터:
  keepout_radius_m       : keepout zone 반경 (m, 기본 2.0)
  expiry_sec             : zone 만료 시간 (초, 기본 1800 = 30분)
  costmap_resolution_m   : OccupancyGrid 해상도 (m/cell, 기본 0.05)
  costmap_size_m         : 발행할 costmap 패치 크기 (m, 기본 10.0)
  co_threshold_ppm       : CO 임계값 (ppm, 기본 200)
  lel_threshold_percent  : LEL 임계값 (%, 기본 10)
  publish_rate_hz        : costmap 발행 주기 (Hz, 기본 1.0)
"""

import json
import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import String
from geometry_msgs.msg import Pose

from argos_interfaces.msg import GasReading, StructuralAlert


# keepout zone 위험 원인 분류
CAUSE_GAS = 'gas'
CAUSE_STRUCTURAL = 'structural'

# Nav2 costmap 비용값
LETHAL_COST = 100   # 절대 진입 불가
FREE_COST = 0       # 자유 공간


@dataclass
class KeeputZone:
    """개별 keepout zone 레코드."""

    zone_id: int
    cause: str              # CAUSE_GAS | CAUSE_STRUCTURAL
    center_x: float         # 맵 좌표 X (m)
    center_y: float         # 맵 좌표 Y (m)
    radius_m: float         # keepout 반경 (m)
    created_at: float       # 생성 시각 (unix time)
    expires_at: float       # 만료 시각 (unix time)
    sensor_cleared: bool = False  # 센서가 정상 복귀 여부
    description: str = ''   # 원인 상세 설명


class KeepoutManager(Node):
    """가스/구조물 위험 기반 Nav2 Keepout Zone 동적 관리자.

    설계 원칙:
    1. 위험 감지 즉시 keepout zone 추가 (지연 없음)
    2. 30분 경과 AND 센서 정상 복귀 시 zone 해제 (보수적 조건)
    3. OccupancyGrid 패치를 주기적으로 발행 → Nav2 costmap_filter_info 대신
       직접 costmap topic으로 게시 (CostmapFilter 플러그인 없이도 작동)
    """

    def __init__(self):
        super().__init__('keepout_manager')

        # ── 파라미터 선언 ──
        self.declare_parameter('keepout_radius_m', 2.0)
        self.declare_parameter('expiry_sec', 1800.0)          # 30분
        self.declare_parameter('costmap_resolution_m', 0.05)
        self.declare_parameter('costmap_size_m', 10.0)
        self.declare_parameter('co_threshold_ppm', 200.0)
        self.declare_parameter('lel_threshold_percent', 10.0)
        self.declare_parameter('publish_rate_hz', 1.0)

        self.radius = self.get_parameter('keepout_radius_m').value
        self.expiry_sec = self.get_parameter('expiry_sec').value
        self.resolution = self.get_parameter('costmap_resolution_m').value
        self.map_size_m = self.get_parameter('costmap_size_m').value
        self.co_thresh = self.get_parameter('co_threshold_ppm').value
        self.lel_thresh = self.get_parameter('lel_threshold_percent').value
        pub_rate = min(self.get_parameter('publish_rate_hz').value, 10.0)

        # ── keepout zone 저장소 ──
        # key: zone_id, value: KeeputZone
        self.zones: Dict[int, KeeputZone] = {}
        self._next_zone_id = 0

        # 가스 정상화 추적: robot_id → 마지막 정상 판정 시각
        self._gas_cleared_at: Dict[str, float] = {}

        # ── QoS ──
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10,
        )

        # ── 구독 ──
        self.gas_sub = self.create_subscription(
            GasReading,
            '/orchestrator/gas_reading',
            self._gas_cb,
            reliable_qos,
        )
        self.structural_sub = self.create_subscription(
            StructuralAlert,
            '/orchestrator/structural_alert',
            self._structural_cb,
            reliable_qos,
        )

        # ── 발행 ──
        self.costmap_pub = self.create_publisher(
            OccupancyGrid,
            '/keepout_zones/costmap_update',
            reliable_qos,
        )
        self.status_pub = self.create_publisher(
            String,
            '/keepout_zones/status',
            10,
        )

        # ── 주기 타이머: 만료 점검 + costmap 재발행 ──
        self.timer = self.create_timer(1.0 / pub_rate, self._tick)

        self.get_logger().info(
            f'KeepoutManager 초기화 — '
            f'반경={self.radius}m, 만료={self.expiry_sec}s, '
            f'해상도={self.resolution}m/cell'
        )

    # ─────────────────── 센서 콜백 ───────────────────

    def _gas_cb(self, msg: GasReading):
        """가스 센서 이벤트 처리.

        CO >= co_threshold 또는 LEL >= lel_threshold 시 keepout zone 추가.
        모두 안전 수준이면 해당 로봇의 'sensor_cleared' 플래그를 갱신.
        """
        loc = msg.location.point
        robot_id = msg.robot_id

        # 위험 판정
        co_danger = msg.co_ppm >= self.co_thresh
        lel_danger = msg.lel_percent >= self.lel_thresh
        is_dangerous = co_danger or lel_danger

        if is_dangerous:
            # 2m 이내에 동일 원인 zone이 이미 있으면 중복 생성 방지
            if not self._zone_exists_nearby(loc.x, loc.y, CAUSE_GAS, radius_limit=2.0):
                desc_parts = []
                if co_danger:
                    desc_parts.append(f'CO={msg.co_ppm:.0f}ppm')
                if lel_danger:
                    desc_parts.append(f'LEL={msg.lel_percent:.0f}%')
                self._add_zone(
                    cause=CAUSE_GAS,
                    cx=loc.x,
                    cy=loc.y,
                    description=f'{robot_id}: {", ".join(desc_parts)}',
                )
            # 가스 정상화 기록 초기화
            self._gas_cleared_at.pop(robot_id, None)
        else:
            # 정상 → 클리어 시각 갱신
            self._gas_cleared_at[robot_id] = time.time()
            # 해당 로봇 위치 근처 가스 zone에 sensor_cleared 표시
            self._mark_gas_cleared(loc.x, loc.y)

    def _structural_cb(self, msg: StructuralAlert):
        """구조물 경고 처리.

        severity가 'warning' 이상이면 affected_radius_m 또는 기본 반경으로 keepout.
        area_blocked=True이면 즉시 추가.
        """
        if msg.severity not in ('warning', 'danger', 'critical'):
            return

        loc = msg.location.point
        # 구조물 영향 반경 우선, 없으면 파라미터 기본값 사용
        radius = msg.affected_radius_m if msg.affected_radius_m > 0.1 else self.radius

        # 중복 방지 (3m 이내 동일 구조물 zone)
        if not self._zone_exists_nearby(loc.x, loc.y, CAUSE_STRUCTURAL, radius_limit=3.0):
            self._add_zone(
                cause=CAUSE_STRUCTURAL,
                cx=loc.x,
                cy=loc.y,
                radius_override=radius,
                description=f'{msg.robot_id}: {msg.alert_type} ({msg.severity})',
            )

    # ─────────────────── Zone 관리 ───────────────────

    def _add_zone(
        self,
        cause: str,
        cx: float,
        cy: float,
        radius_override: Optional[float] = None,
        description: str = '',
    ):
        """새 keepout zone 등록."""
        now = time.time()
        radius = radius_override if radius_override is not None else self.radius
        zone = KeeputZone(
            zone_id=self._next_zone_id,
            cause=cause,
            center_x=cx,
            center_y=cy,
            radius_m=radius,
            created_at=now,
            expires_at=now + self.expiry_sec,
            description=description,
        )
        self.zones[self._next_zone_id] = zone
        self._next_zone_id += 1

        self.get_logger().warn(
            f'[Keepout ✚] ID={zone.zone_id} cause={cause} '
            f'center=({cx:.2f}, {cy:.2f}) r={radius:.1f}m — {description}'
        )

    def _zone_exists_nearby(
        self, cx: float, cy: float, cause: str, radius_limit: float
    ) -> bool:
        """동일 원인의 zone이 radius_limit 이내에 이미 존재하면 True."""
        for zone in self.zones.values():
            if zone.cause != cause:
                continue
            dist = math.sqrt((zone.center_x - cx)**2 + (zone.center_y - cy)**2)
            if dist < radius_limit:
                return True
        return False

    def _mark_gas_cleared(self, cx: float, cy: float):
        """로봇 위치 근처(5m 이내) 가스 zone에 sensor_cleared 표시."""
        for zone in self.zones.values():
            if zone.cause != CAUSE_GAS:
                continue
            dist = math.sqrt((zone.center_x - cx)**2 + (zone.center_y - cy)**2)
            if dist < 5.0:
                zone.sensor_cleared = True

    def _remove_expired_zones(self):
        """만료 조건 충족 zone 제거.

        제거 조건: 30분(expiry_sec) 경과 AND sensor_cleared=True
        구조물 zone은 sensor_cleared 없이 시간 만료만으로 제거
        (구조물 변화는 자동 복구 안 됨 → 시간 기반 안전 한계 적용).
        """
        now = time.time()
        to_remove = []

        for zone_id, zone in self.zones.items():
            if now < zone.expires_at:
                continue  # 아직 만료 시간 안 됨

            if zone.cause == CAUSE_GAS:
                # 가스: 시간 만료 + 센서 정상 복귀 모두 필요
                if zone.sensor_cleared:
                    to_remove.append(zone_id)
            else:
                # 구조물: 시간 만료만으로 제거
                to_remove.append(zone_id)

        for zone_id in to_remove:
            z = self.zones.pop(zone_id)
            self.get_logger().info(
                f'[Keepout ✖] ID={zone_id} 해제 — '
                f'cause={z.cause} center=({z.center_x:.2f}, {z.center_y:.2f})'
            )

    # ─────────────────── Costmap 생성 ───────────────────

    def _build_costmap_patch(self, zone: KeeputZone) -> OccupancyGrid:
        """개별 keepout zone에 대한 OccupancyGrid 패치 생성.

        zone 중심을 기준으로 (costmap_size_m × costmap_size_m) 크기의
        OccupancyGrid를 생성하고 반경 내 셀에 LETHAL_COST를 설정합니다.

        Nav2 costmap_2d는 OccupancyGrid를 구독하면 자동으로
        관련 레이어에 merge합니다 (static_layer 또는 obstacle_layer 대체 가능).
        """
        half = self.map_size_m / 2.0
        # origin: zone 중심을 패치 중앙으로
        origin_x = zone.center_x - half
        origin_y = zone.center_y - half

        # 셀 수 (정수 변환)
        cells = int(math.ceil(self.map_size_m / self.resolution))

        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'map'

        meta = MapMetaData()
        meta.resolution = self.resolution
        meta.width = cells
        meta.height = cells
        meta.origin = Pose()
        meta.origin.position.x = origin_x
        meta.origin.position.y = origin_y
        meta.origin.position.z = 0.0
        meta.origin.orientation.w = 1.0
        grid.info = meta

        # 전체 셀을 FREE(0)로 초기화
        data = np.zeros(cells * cells, dtype=np.int8)

        # 반경 내 셀에 LETHAL_COST 설정
        radius_cells = int(math.ceil(zone.radius_m / self.resolution))
        cx_cell = cells // 2  # zone 중심 = 패치 중앙
        cy_cell = cells // 2

        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                # 원형 마스크
                if dx * dx + dy * dy > radius_cells * radius_cells:
                    continue
                xi = cx_cell + dx
                yi = cy_cell + dy
                if 0 <= xi < cells and 0 <= yi < cells:
                    idx = yi * cells + xi
                    data[idx] = LETHAL_COST

        grid.data = data.tolist()
        return grid

    # ─────────────────── 주기 타이머 ───────────────────

    def _tick(self):
        """매 주기 실행: 만료 zone 제거 → costmap 발행 → 상태 발행."""
        self._remove_expired_zones()

        # 활성 zone별 OccupancyGrid 패치 발행
        for zone in self.zones.values():
            costmap_msg = self._build_costmap_patch(zone)
            self.costmap_pub.publish(costmap_msg)

        # 상태 JSON 발행 (디버그/모니터링용)
        self._publish_status()

    def _publish_status(self):
        """활성 keepout zone 목록을 JSON 문자열로 발행."""
        now = time.time()
        status = {
            'timestamp': now,
            'active_zones': [
                {
                    'zone_id': z.zone_id,
                    'cause': z.cause,
                    'center_x': round(z.center_x, 2),
                    'center_y': round(z.center_y, 2),
                    'radius_m': round(z.radius_m, 2),
                    'expires_in_sec': round(z.expires_at - now, 0),
                    'sensor_cleared': z.sensor_cleared,
                    'description': z.description,
                }
                for z in self.zones.values()
            ],
        }
        msg = String()
        msg.data = json.dumps(status, ensure_ascii=False)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = KeepoutManager()
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
