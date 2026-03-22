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
"""편대 조율 매니저 (Formation Manager)
====================================
이종 군집(UGV+드론)의 편대 패턴 생성 + 충돌 방지.

소방 현장 편대 패턴:
  LINE_ABREAST: 횡대 — 넓은 영역 동시 탐색 (기본)
  COLUMN:       종대 — 좁은 통로 진입
  ECHELON:      제대 — 한쪽 방향 우선 탐색
  SURROUND:     포위 — 화점 포위 진압

사용:
  manager = FormationManager(robot_configs)
  waypoints = manager.compute_formation(
      center=(5.0, 3.0),
      pattern='LINE_ABREAST',
      spacing=2.0,
      heading=0.0,
  )
  # waypoints = {'argos1': (3.0, 3.0), 'argos2': (7.0, 3.0), 'drone1': (5.0, 3.0, 8.0), ...}

S-D5 연동:
  CBBAAllocator가 태스크를 할당한 뒤, FormationManager가 각 로봇의
  목표 좌표를 조율하여 충돌 없는 편대 웨이포인트를 제공한다.
  OrchestratorNode에서 두 객체를 순서대로 호출한다:
    assignments = allocator.allocate(robots, tasks)
    waypoints   = formation.compute_formation(center, pattern)
"""
import math
from dataclasses import dataclass
from typing import Optional


@dataclass
class RobotConfig:
    """편대 로봇 설정.

    robot_type:
        'ugv'   — 지상 로봇. 좌표 = (x, y). 고도 없음.
        'drone' — 비행 로봇. 좌표 = (x, y, z).
    current_pos:
        UGV 는 (x, y), 드론은 (x, y, z) 로 전달한다.
        z 값은 고도 참고용이며 편대 계산에서는 drone_altitude 로 덮어쓴다.
    """

    robot_id: str
    robot_type: str       # 'ugv' | 'drone'
    current_pos: tuple    # (x, y) 또는 (x, y, z)


class FormationPattern:
    """편대 패턴 상수."""

    LINE_ABREAST = 'LINE_ABREAST'   # 횡대 — 진행 방향에 수직 배열
    COLUMN = 'COLUMN'               # 종대 — 진행 방향으로 일렬
    ECHELON = 'ECHELON'             # 제대 — 45도 사선 배열
    SURROUND = 'SURROUND'           # 포위 — 화점 중심 원형 배열


class FormationManager:
    """이종 군집 편대 조율 매니저.

    드론은 UGV 위 고정 고도(기본 8 m)에서 정찰.
    UGV는 지상에서 spacing 간격으로 배치.

    설계 원칙:
      - UGV-드론 간 충돌은 고도 차이로 물리적 불가능 → 2D 충돌 검사는 동종(同種)끼리만 수행
      - spacing < MIN_SPACING 이면 MIN_SPACING 으로 강제 클램핑
      - 편대 중심(center)은 (x, y) ENU 좌표 (m)
    """

    DEFAULT_DRONE_ALTITUDE = 8.0   # 드론 기본 고도 (m)
    MIN_SPACING = 1.5              # 최소 간격 (m) — 충돌 방지 하드 리밋

    def __init__(self, robots: list):
        """편대 매니저 초기화.

        Args:
            robots: RobotConfig 객체 리스트. 빈 리스트도 허용.
        """
        self._robots = {r.robot_id: r for r in robots}
        # 등록 순서 보존 — 편대 인덱스 결정에 영향
        self._ugvs = [r for r in robots if r.robot_type == 'ugv']
        self._drones = [r for r in robots if r.robot_type == 'drone']

    # ── 공개 API ──────────────────────────────────────────────────────────────

    def compute_formation(
        self,
        center: tuple,
        pattern: str = FormationPattern.LINE_ABREAST,
        spacing: float = 3.0,
        heading: float = 0.0,
        drone_altitude: float = DEFAULT_DRONE_ALTITUDE,
    ) -> dict:
        """편대 패턴에 따른 각 로봇의 목표 좌표 계산.

        Args:
            center:         편대 중심점 (x, y) ENU 좌표 (m).
            pattern:        FormationPattern 상수 중 하나.
            spacing:        로봇 간 간격 (m). MIN_SPACING 미만이면 클램핑.
            heading:        편대 진행 방향 (rad, ENU 기준 반시계).
            drone_altitude: 드론 고도 (m). 기본 8 m.

        Returns:
            {robot_id: (x, y) for UGV, (x, y, z) for drone}

        Raises:
            ValueError: 알 수 없는 패턴 문자열일 때.
        """
        # 간격 하한 클램핑
        spacing = max(spacing, self.MIN_SPACING)

        if pattern == FormationPattern.LINE_ABREAST:
            return self._line_abreast(center, spacing, heading, drone_altitude)
        elif pattern == FormationPattern.COLUMN:
            return self._column(center, spacing, heading, drone_altitude)
        elif pattern == FormationPattern.ECHELON:
            return self._echelon(center, spacing, heading, drone_altitude)
        elif pattern == FormationPattern.SURROUND:
            return self._surround(center, spacing, drone_altitude)
        else:
            raise ValueError(f"알 수 없는 편대 패턴: {pattern!r}")

    def check_collision_risk(
        self,
        waypoints: dict,
        min_distance: float = 1.5,
    ) -> list:
        """편대 웨이포인트 간 충돌 위험 검사.

        UGV-드론 간 충돌은 고도 차이로 불가 → 동종 로봇끼리만 검사.
        고도는 무시하고 2D(x, y) 거리만 사용.

        Args:
            waypoints:    compute_formation() 결과 딕셔너리.
            min_distance: 이 거리 미만이면 위험으로 판정 (m).

        Returns:
            [(robot_a_id, robot_b_id, distance), ...] — 위험 쌍 목록.
            위험 없으면 빈 리스트.
        """
        risks = []
        ids = list(waypoints.keys())

        for i in range(len(ids)):
            for j in range(i + 1, len(ids)):
                id_a, id_b = ids[i], ids[j]

                # 동종 여부 확인 — 미등록 로봇은 'ugv' 로 간주
                type_a = self._robots[id_a].robot_type if id_a in self._robots else 'ugv'
                type_b = self._robots[id_b].robot_type if id_b in self._robots else 'ugv'
                if type_a != type_b:
                    continue

                pos_a = waypoints[id_a]
                pos_b = waypoints[id_b]
                dist = math.hypot(pos_a[0] - pos_b[0], pos_a[1] - pos_b[1])

                if dist < min_distance:
                    risks.append((id_a, id_b, dist))

        return risks

    # ── 프로퍼티 ──────────────────────────────────────────────────────────────

    @property
    def robot_count(self) -> int:
        """등록된 전체 로봇 수."""
        return len(self._robots)

    @property
    def ugv_count(self) -> int:
        """등록된 UGV 수."""
        return len(self._ugvs)

    @property
    def drone_count(self) -> int:
        """등록된 드론 수."""
        return len(self._drones)

    # ── 내부 패턴 계산 ────────────────────────────────────────────────────────

    def _line_abreast(self, center, spacing, heading, alt):
        """횡대: UGV들이 진행 방향에 수직으로 배치, 드론은 UGV와 페어링.

        UGV 배치: 중심에서 수직(perp) 방향으로 균등 분산.
        드론 배치: i번째 드론은 i번째 UGV 정위 상공. UGV보다 많으면 편대 중심 상공.
        """
        result = {}
        n_ugv = len(self._ugvs)
        # 진행 방향(heading)에 수직인 방향 — 횡대 전개 기준
        perp = heading + math.pi / 2

        for i, ugv in enumerate(self._ugvs):
            # 중심 기준으로 좌우 균등 분산 (홀수: 중앙 1개, 짝수: 중앙 없음)
            offset = (i - (n_ugv - 1) / 2) * spacing
            x = center[0] + offset * math.cos(perp)
            y = center[1] + offset * math.sin(perp)
            result[ugv.robot_id] = (x, y)

        for i, drone in enumerate(self._drones):
            if i < n_ugv:
                # UGV와 1:1 페어링 — 같은 X/Y, 지정 고도
                ugv_pos = result[self._ugvs[i].robot_id]
                result[drone.robot_id] = (ugv_pos[0], ugv_pos[1], alt)
            else:
                # 페어링할 UGV 없음 — 편대 중심 상공
                result[drone.robot_id] = (center[0], center[1], alt)

        return result

    def _column(self, center, spacing, heading, alt):
        """종대: 진행 방향(heading)으로 UGV/드론 각각 일렬 배치.

        UGV 종열과 드론 종열을 각각 중심 정렬한다.
        좁은 통로 진입, 계단 오르기 등 종심 이동에 적합.
        """
        result = {}
        n_ugv = len(self._ugvs)
        n_drone = len(self._drones)

        for i, ugv in enumerate(self._ugvs):
            offset = (i - (n_ugv - 1) / 2) * spacing
            x = center[0] + offset * math.cos(heading)
            y = center[1] + offset * math.sin(heading)
            result[ugv.robot_id] = (x, y)

        for i, drone in enumerate(self._drones):
            offset = (i - (n_drone - 1) / 2) * spacing
            x = center[0] + offset * math.cos(heading)
            y = center[1] + offset * math.sin(heading)
            result[drone.robot_id] = (x, y, alt)

        return result

    def _echelon(self, center, spacing, heading, alt):
        """제대: heading + 45도 방향으로 모든 로봇 사선 배치.

        UGV 먼저, 드론 뒤 순서로 사선 배열.
        한쪽 방향 탐색 집중 시 사용. 최선두 로봇이 선두 정찰.
        """
        result = {}
        # 사선 방향 = 진행 방향 + 45도
        echelon_angle = heading + math.pi / 4
        all_robots = self._ugvs + self._drones

        for i, robot in enumerate(all_robots):
            offset = i * spacing
            x = center[0] + offset * math.cos(echelon_angle)
            y = center[1] + offset * math.sin(echelon_angle)
            if robot.robot_type == 'drone':
                result[robot.robot_id] = (x, y, alt)
            else:
                result[robot.robot_id] = (x, y)

        return result

    def _surround(self, center, spacing, alt):
        """포위: 화점 중심으로 모든 로봇 원형(등각) 배치.

        spacing = 반지름(m). 로봇이 1개면 중심 정위.
        UGV 먼저, 드론 뒤 순서로 등각 배분.
        소방 현장 화점 봉쇄, 360도 관측에 최적.
        """
        result = {}
        radius = spacing
        all_robots = self._ugvs + self._drones
        n = len(all_robots)

        if n == 0:
            return result

        for i, robot in enumerate(all_robots):
            # 360도를 n등분하여 등각 배치
            angle = 2 * math.pi * i / n
            x = center[0] + radius * math.cos(angle)
            y = center[1] + radius * math.sin(angle)
            if robot.robot_type == 'drone':
                result[robot.robot_id] = (x, y, alt)
            else:
                result[robot.robot_id] = (x, y)

        return result
