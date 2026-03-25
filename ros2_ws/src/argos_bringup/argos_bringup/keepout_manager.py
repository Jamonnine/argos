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

from std_msgs.msg import Float32MultiArray

from argos_interfaces.msg import GasReading, StructuralAlert


# keepout zone 위험 원인 분류
CAUSE_GAS = 'gas'
CAUSE_STRUCTURAL = 'structural'
CAUSE_HOSE = 'hose'  # 호스 경로 장애물

# Nav2 costmap 비용값
LETHAL_COST = 100   # 절대 진입 불가
FREE_COST = 0       # 자유 공간


@dataclass
class KeeputZone:
    """개별 keepout zone 레코드."""

    zone_id: int
    cause: str              # CAUSE_GAS | CAUSE_STRUCTURAL | CAUSE_HOSE
    center_x: float         # 맵 좌표 X (m)
    center_y: float         # 맵 좌표 Y (m)
    radius_m: float         # keepout 반경 (m)
    created_at: float       # 생성 시각 (unix time)
    expires_at: float       # 만료 시각 (unix time)
    sensor_cleared: bool = False  # 센서가 정상 복귀 여부
    description: str = ''   # 원인 상세 설명
    robot_id: str = ''      # 호스 zone의 경우 소유 로봇 ID


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
        # 호스 keepout 파라미터
        # NFRI 호스릴 스펙: 내경 32mm (외경 ~43mm) + 안전 여유 172mm = 215mm 반경 # NFRI 2025 리빙랩
        # 야광호스 적용 (일광/LED 30분 노출 → 2~6시간 발광, 지하 화재 진입 시 시인성 확보)
        self.declare_parameter('hose_radius_m', 0.215)
        self.declare_parameter('hose_expiry_sec', 3600.0)     # 1시간
        self.declare_parameter('sherpa_robots', ['sherpa1', 'sherpa2'])  # 셰르파 로봇 목록

        self.radius = self.get_parameter('keepout_radius_m').value
        self.expiry_sec = self.get_parameter('expiry_sec').value
        self.resolution = self.get_parameter('costmap_resolution_m').value
        self.map_size_m = self.get_parameter('costmap_size_m').value
        self.co_thresh = self.get_parameter('co_threshold_ppm').value
        self.lel_thresh = self.get_parameter('lel_threshold_percent').value
        pub_rate = min(self.get_parameter('publish_rate_hz').value, 10.0)
        self.hose_radius = self.get_parameter('hose_radius_m').value
        self.hose_expiry_sec = self.get_parameter('hose_expiry_sec').value
        self._sherpa_robots: List[str] = list(
            self.get_parameter('sherpa_robots').value)

        # ── keepout zone 저장소 ──
        # key: zone_id, value: KeeputZone
        self.zones: Dict[int, KeeputZone] = {}
        self._next_zone_id = 0

        # 가스 정상화 추적: robot_id → 마지막 정상 판정 시각
        self._gas_cleared_at: Dict[str, float] = {}

        # 호스 path_viz 구독 핸들 (deactivate 시 해제용)
        self._hose_path_subs: List = []

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

        # ── 호스 경로 시각화 구독 (셰르파 로봇 → 자동 keepout 갱신) ──
        for rid in self._sherpa_robots:
            sub = self.create_subscription(
                Float32MultiArray,
                f'/{rid}/hose/path_viz',
                lambda msg, r=rid: self._hose_path_viz_cb(r, msg),
                10,
            )
            self._hose_path_subs.append(sub)
            self.get_logger().info(f'호스 경로 구독 등록: /{rid}/hose/path_viz')

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

    # ─────────────────── 호스 경로 Keepout 관리 ───────────────────

    def mark_hose_path(self, robot_id: str, path_points: List[tuple]):
        """호스 경로를 costmap 선형 장애물로 등록.

        경로 선분마다 중간점을 keepout zone 중심으로 등록합니다.
        반경: hose_radius_m (0.215m = 호스 직경 65mm + 여유 150mm).
        만료: hose_expiry_sec (기본 3600초).

        Args:
            robot_id: 호스를 소유한 셰르파 로봇 ID
            path_points: [(x, y)] 형태의 경유점 목록 (최소 2개)
        """
        if len(path_points) < 2:
            return

        # 기존 호스 zone 먼저 삭제 (갱신 방식)
        self.clear_hose_path(robot_id)

        now = time.time()
        for i in range(len(path_points) - 1):
            x0, y0 = path_points[i]
            x1, y1 = path_points[i + 1]
            # 선분 중간점을 zone 중심으로 사용
            cx = (x0 + x1) / 2.0
            cy = (y0 + y1) / 2.0

            zone = KeeputZone(
                zone_id=self._next_zone_id,
                cause=CAUSE_HOSE,
                center_x=cx,
                center_y=cy,
                radius_m=self.hose_radius,
                created_at=now,
                expires_at=now + self.hose_expiry_sec,
                sensor_cleared=False,
                description=f'{robot_id} 호스 경로 seg{i}',
                robot_id=robot_id,
            )
            self.zones[self._next_zone_id] = zone
            self._next_zone_id += 1

        self.get_logger().info(
            f'[호스 Keepout] {robot_id}: {len(path_points) - 1}개 선분 등록 '
            f'(r={self.hose_radius}m, 만료={self.hose_expiry_sec}s)')

    def clear_hose_path(self, robot_id: str):
        """해당 로봇의 호스 keepout zone 전체 삭제.

        호스 회수 또는 경로 갱신 시 호출.

        Args:
            robot_id: 호스를 소유한 셰르파 로봇 ID
        """
        to_remove = [
            zid for zid, z in self.zones.items()
            if z.cause == CAUSE_HOSE and z.robot_id == robot_id
        ]
        for zid in to_remove:
            self.zones.pop(zid)

        if to_remove:
            self.get_logger().info(
                f'[호스 Keepout] {robot_id}: {len(to_remove)}개 zone 해제')

    def _hose_path_viz_cb(self, robot_id: str, msg: Float32MultiArray):
        """/{robot_id}/hose/path_viz 구독 콜백 → 자동 keepout 갱신.

        Float32MultiArray.data 레이아웃:
          짝수 인덱스=x, 홀수 인덱스=y (순서대로 경유점 쌍)
          예: [x0, y0, x1, y1, x2, y2, ...]

        데이터가 비어 있으면 호스 회수로 판단하고 keepout 삭제.
        """
        if len(msg.data) == 0:
            # 빈 데이터 = 호스 회수 신호
            self.clear_hose_path(robot_id)
            return

        if len(msg.data) % 2 != 0:
            self.get_logger().warn(
                f'[호스 Keepout] {robot_id}: path_viz 데이터 길이 홀수 '
                f'({len(msg.data)}) — 무시')
            return

        path = [
            (float(msg.data[i]), float(msg.data[i + 1]))
            for i in range(0, len(msg.data), 2)
        ]
        self.mark_hose_path(robot_id, path)

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
                # 구조물 / 호스: 시간 만료만으로 제거
                # 호스 zone은 clear_hose_path()로 명시 삭제도 가능
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
                    'robot_id': z.robot_id,  # 호스 zone 소유 로봇 (빈 문자열=비호스)
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
