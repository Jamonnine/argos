"""
CBBA 태스크 할당기 (Consensus-Based Bundle Algorithm)
=======================================================
참조 논문: Choi et al., "Consensus-Based Decentralized Auctions for Robust Task Allocation"
           IEEE Transactions on Robotics, 2009
           ACBBA 경매 할당 (Springer 2025) — argos-tutor.md 참조

소방 현장 적용:
  - 이종 로봇(UGV/드론)의 capabilities로 임무 필터링
  - 거리 기반 비용 + priority 가중치 조합
  - 분산 합의(Distributed Consensus)로 충돌 없는 최적 할당
  - 번들 크기 N: 로봇당 최대 N개 임무 할당 (Phase E, v4.0)

알고리즘 개요 (3단계):
  Phase 1 - 번들 빌드: 각 로봇이 자신의 capability 매칭 임무를 비용순 선택
  Phase 2 - 경매:     로봇별 입찰가(bid) 계산 + 최저 비용 임무 선점
  Phase 3 - 합의:     충돌(동일 임무 복수 입찰) 해소 → 최적 할당 확정

오케스트레이터 연동:
  OrchestratorNode.on_activate() 에서 self.allocator = CBBAAllocator(self.get_logger()) 생성
  화재 감지 시 (단일 할당): assignments = self.allocator.allocate(robot_list, task_list)
  번들 할당 시:            bundles = self.allocator.allocate_bundles(robot_list, task_list)

성능:
  로봇 N개, 임무 M개 기준: O(N × M × bundle_size) per iteration, 최대 N×bundle_size회 합의 순환
  소방 현장 규모(N=10, M=30)에서 실시간 동작 충분
"""

import math
import logging
from dataclasses import dataclass, field
from typing import Optional


# ── 임무(Task) 정의 ──────────────────────────────────────────────────────────

@dataclass
class Task:
    """소방 현장 임무 단위.

    task_type별 required_capabilities 가이드라인:
      'explore'      → [] (모든 로봇 가능)
      'inspect_fire' → ['has_thermal'] (열화상 카메라 필수)
      'rescue'       → ['has_gripper'] (파지 장치 필수)
      'monitor'      → ['can_fly'] (항공 관측 — 드론 전용)
    """
    task_id: str
    location: tuple[float, float, float]   # (x, y, z) — ENU 좌표 (m)
    task_type: str                          # 'explore' | 'inspect_fire' | 'rescue' | 'monitor'
    required_capabilities: list[str]        # 예: ['can_fly', 'has_thermal']
    priority: float = 0.5                  # 0.0(낮음) ~ 1.0(긴급)

    def __post_init__(self):
        """우선순위 범위 검증."""
        if not 0.0 <= self.priority <= 1.0:
            raise ValueError(
                f"Task '{self.task_id}': priority={self.priority} 는 0.0~1.0 범위여야 합니다.")


@dataclass
class RobotRecord:
    """CBBAAllocator가 참조하는 로봇 레코드.

    orchestrator_node.py의 RobotRecord와 별개로 정의 (순환 의존 방지).
    오케스트레이터에서 변환해서 전달:
      cbba_robots = [
          RobotRecord(r.robot_id, r.capabilities, r.pose)
          for r in self.robots.values() if not r.comm_lost
      ]
    """
    robot_id: str
    capabilities: list[str]               # 예: ['can_fly', 'has_thermal']
    pose: Optional[object] = None          # geometry_msgs.msg.PoseStamped 또는 None


# ── CBBA 핵심 알고리즘 ────────────────────────────────────────────────────────

class CBBAAllocator:
    """Consensus-Based Bundle Algorithm 태스크 할당기.

    Phase D (v3.5) 목표: 이종 군집 편대의 역할 분담 자동화.
    소방 지휘체계 매핑:
      현장 지휘관(인간) → OrchestratorNode → CBBAAllocator → 개별 로봇

    설계 원칙:
      1. "인터페이스에 프로그래밍" — 로봇 타입(UGV/드론) 무관, capabilities로만 판단
      2. 분산 가능 설계 — Phase 3 합의는 로봇 간 직접 통신으로 확장 가능
      3. 소방 우선순위 반영 — priority 1.0(긴급) 임무는 비용 계산에서 우대
    """

    # 비용 가중치 상수
    DISTANCE_WEIGHT = 1.0        # 거리 비용 가중치
    PRIORITY_WEIGHT = 2.0        # 우선순위 가중치 (priority 높을수록 비용 감소)
    CAPABILITY_MISMATCH_COST = 1e9  # capability 미충족 시 사실상 제외

    def __init__(self, logger=None, max_bundle_size: int = 1):
        """초기화.

        Args:
            logger: rclpy.node.Node.get_logger() 반환값 (ROS2 환경) 또는
                    logging.Logger (순수 Python 환경, 단위 테스트용)
            max_bundle_size: 로봇당 할당 가능한 최대 임무 수 (기본값=1, 하위 호환).
                             allocate_bundles() 호출 시 기본 번들 크기로 사용됨.
        """
        if logger is None:
            self._log = logging.getLogger('CBBAAllocator')
        else:
            self._log = logger

        if max_bundle_size < 1:
            raise ValueError(
                f'max_bundle_size={max_bundle_size}는 1 이상이어야 합니다.'
            )
        self.max_bundle_size = max_bundle_size

    # ── 공개 인터페이스 ──────────────────────────────────────────────────────

    def allocate_bundles(
        self,
        robots: list[RobotRecord],
        tasks: list[Task],
        max_bundle_size: int = 0,
    ) -> dict[str, list[Task]]:
        """CBBA 번들 할당 — 로봇당 최대 N개 임무 할당.

        Args:
            robots: 현재 운용 가능한 로봇 목록 (comm_lost 제외 권장)
            tasks:  할당 대상 임무 목록
            max_bundle_size: 로봇당 최대 임무 수.
                             0이면 self.max_bundle_size 사용 (기본값).

        Returns:
            dict[str, list[Task]]: 예) {'argos1': [Task1, Task2], 'drone1': [Task3]}
            임무를 할당받지 못한 로봇은 결과 dict에 포함되지 않음.

        주의:
            - tasks가 robots보다 적으면 일부 로봇은 미할당
            - 모든 capability 요건을 충족하는 로봇이 없는 임무는 미할당 (경고 로그)
            - max_bundle_size=1일 때 기존 allocate()와 동일한 할당 결과 보장
        """
        # 번들 크기 결정 (0이면 인스턴스 기본값 사용)
        bundle_n = max_bundle_size if max_bundle_size > 0 else self.max_bundle_size

        if not robots:
            self._log.warning('CBBA: 로봇 목록이 비어 있습니다. 할당 중단.')
            return {}

        if not tasks:
            self._log.info('CBBA: 임무 목록이 비어 있습니다.')
            return {}

        self._log.info(
            f'CBBA 번들 시작: 로봇 {len(robots)}대, 임무 {len(tasks)}건, '
            f'번들 크기 {bundle_n}'
        )

        # Phase 1: 비용 행렬 계산
        cost_matrix = self._build_cost_matrix(robots, tasks)

        # Phase 2: 경매 — 각 로봇이 최저 비용 임무 순서대로 입찰 목록 구성
        bids = self._run_auction(robots, tasks, cost_matrix)

        # Phase 3: 번들 합의 — 로봇당 최대 bundle_n개 임무 할당
        assignments = self._resolve_consensus_bundles(
            robots, tasks, bids, cost_matrix, bundle_n
        )

        # 결과 로그
        for robot_id, task_list in assignments.items():
            task_ids = [t.task_id for t in task_list]
            self._log.info(
                f'CBBA 번들 할당: {robot_id} → {task_ids}'
            )

        assigned_task_ids = {
            t.task_id
            for task_list in assignments.values()
            for t in task_list
        }
        unassigned = [
            t.task_id for t in tasks
            if t.task_id not in assigned_task_ids
        ]
        if unassigned:
            self._log.warning(f'CBBA: 미할당 임무 {len(unassigned)}건: {unassigned}')

        return assignments

    def allocate(
        self,
        robots: list[RobotRecord],
        tasks: list[Task],
    ) -> dict[str, Task]:
        """기존 호환 인터페이스: 로봇당 임무 1개만 반환.

        내부적으로 allocate_bundles(max_bundle_size=1)을 호출하고
        각 로봇의 첫 번째 임무만 반환합니다.

        Args:
            robots: 현재 운용 가능한 로봇 목록 (comm_lost 제외 권장)
            tasks:  할당 대상 임무 목록

        Returns:
            dict[str, Task]: 예) {'argos1': Task(...), 'drone1': Task(...)}
            임무를 할당받지 못한 로봇은 결과 dict에 포함되지 않음.

        주의:
            - tasks가 robots보다 적으면 일부 로봇은 미할당
            - 모든 capability 요건을 충족하는 로봇이 없는 임무는 미할당 (경고 로그)
        """
        bundles = self.allocate_bundles(robots, tasks, max_bundle_size=1)
        # 각 로봇에서 첫 번째 임무만 추출 (번들 크기=1이므로 항상 최대 1개)
        return {rid: task_list[0] for rid, task_list in bundles.items() if task_list}

    # ── Phase 1: 비용 행렬 ──────────────────────────────────────────────────

    def _build_cost_matrix(
        self,
        robots: list[RobotRecord],
        tasks: list[Task],
    ) -> list[list[float]]:
        """로봇-임무 비용 행렬 계산.

        비용 = 거리 비용 × DISTANCE_WEIGHT − 우선순위 기여 × PRIORITY_WEIGHT
               (낮을수록 선호)

        거리 비용: 로봇 현재 위치 → 임무 위치 유클리드 거리
        우선순위 기여: priority × PRIORITY_WEIGHT (우선순위 높으면 비용 감소)
        capability 미충족: CAPABILITY_MISMATCH_COST (사실상 제외)

        Returns:
            cost[i][j] = 로봇 i가 임무 j를 수행하는 비용 (float)
        """
        n_robots = len(robots)
        n_tasks = len(tasks)

        # 초기화
        cost = [[0.0] * n_tasks for _ in range(n_robots)]

        for i, robot in enumerate(robots):
            robot_pos = self._get_robot_position(robot)

            for j, task in enumerate(tasks):
                # capability 검증 (최우선)
                if not self._check_capabilities(robot, task):
                    cost[i][j] = self.CAPABILITY_MISMATCH_COST
                    continue

                # 거리 비용
                dist = self._distance(robot_pos, task.location)

                # 우선순위 기여: priority=1.0이면 PRIORITY_WEIGHT만큼 비용 감소
                priority_bonus = task.priority * self.PRIORITY_WEIGHT

                # 최종 비용 (음수 방지: max with 0)
                cost[i][j] = max(
                    0.0,
                    dist * self.DISTANCE_WEIGHT - priority_bonus
                )

        return cost

    # ── Phase 2: 경매 ────────────────────────────────────────────────────────

    def _run_auction(
        self,
        robots: list[RobotRecord],
        tasks: list[Task],
        cost_matrix: list[list[float]],
    ) -> dict[str, list[tuple[float, str]]]:
        """경매 단계: 각 로봇이 자신의 입찰 목록(비용 오름차순) 구성.

        capability를 충족하는 모든 임무를 비용 오름차순으로 정렬하여 반환.
        실제 번들 크기 제한(N개 선택)은 _resolve_consensus_bundles()에서 수행.
        allocate_bundles()와 allocate() 모두 이 메서드를 공유함.

        Returns:
            bids[robot_id] = [(cost, task_id), ...]  — 입찰 목록 (비용 오름차순)
        """
        bids: dict[str, list[tuple[float, str]]] = {}

        for i, robot in enumerate(robots):
            # capability 미충족 임무 제외
            valid_bids = [
                (cost_matrix[i][j], tasks[j].task_id)
                for j in range(len(tasks))
                if cost_matrix[i][j] < self.CAPABILITY_MISMATCH_COST
            ]

            if not valid_bids:
                self._log.warning(
                    f'CBBA: {robot.robot_id}는 수행 가능한 임무가 없습니다. '
                    f'보유 capabilities: {robot.capabilities}'
                )
                bids[robot.robot_id] = []
            else:
                # 비용 오름차순 정렬 (낮을수록 선호)
                valid_bids.sort(key=lambda x: x[0])
                bids[robot.robot_id] = valid_bids

        return bids

    # ── Phase 3: 합의 ────────────────────────────────────────────────────────

    def _resolve_consensus(
        self,
        robots: list[RobotRecord],
        tasks: list[Task],
        bids: dict[str, list[tuple[float, str]]],
        cost_matrix: list[list[float]],
    ) -> dict[str, Task]:
        """합의 단계: 충돌(동일 임무 복수 입찰) 해소 → 최적 할당 확정.

        알고리즘:
          1. 모든 로봇의 1순위 입찰 수집
          2. 임무별로 충돌 감지 (복수 입찰)
          3. 충돌 시: 최저 비용 로봇이 승리, 패자는 다음 입찰로 이동
          4. 미할당 임무 없어질 때까지 반복 (최대 N 순환)
        """
        # task_id → Task 역방향 매핑
        task_map = {t.task_id: t for t in tasks}

        # 로봇별 현재 입찰 포인터 (0 = 1순위 입찰)
        bid_pointer = {robot.robot_id: 0 for robot in robots}

        assignments: dict[str, str] = {}  # robot_id → task_id

        # 최대 반복: 로봇 수 × 임무 수 (이론적 상한)
        max_iter = len(robots) * max(len(tasks), 1)

        for iteration in range(max_iter):
            # 이번 라운드 각 로봇의 현재 입찰
            current_bids: dict[str, tuple[float, str]] = {}  # robot_id → (cost, task_id)

            for robot in robots:
                rid = robot.robot_id
                ptr = bid_pointer[rid]
                if ptr < len(bids[rid]) and rid not in assignments:
                    current_bids[rid] = bids[rid][ptr]

            if not current_bids:
                # 모든 로봇 할당 완료 또는 입찰 소진
                break

            # 임무별 입찰 집계 (충돌 감지)
            # task_id → [(cost, robot_id), ...]
            task_bids: dict[str, list[tuple[float, str]]] = {}
            for rid, (cost, tid) in current_bids.items():
                if tid not in task_bids:
                    task_bids[tid] = []
                task_bids[tid].append((cost, rid))

            # 충돌 해소: 임무당 최저 비용 로봇 선정
            resolved_this_round = False

            # 이미 할당된 임무 집합 (이번 라운드 충돌 체크용)
            already_assigned_tasks = set(assignments.values())

            for tid, competing_bids in task_bids.items():
                # 이미 다른 로봇에게 할당된 임무라면 모든 입찰자 다음 순위로 이동
                if tid in already_assigned_tasks:
                    for _, bidder_id in competing_bids:
                        if bidder_id not in assignments:
                            bid_pointer[bidder_id] += 1
                    continue

                # 최저 비용 로봇 = 경매 승자
                competing_bids.sort(key=lambda x: x[0])
                winner_cost, winner_id = competing_bids[0]

                # 승자 할당 (아직 미할당 로봇인 경우만)
                if winner_id not in assignments:
                    assignments[winner_id] = tid
                    already_assigned_tasks.add(tid)
                    resolved_this_round = True
                    self._log.debug(
                        f'CBBA iter={iteration}: {winner_id} → {tid} '
                        f'(cost={winner_cost:.2f})'
                    )

                # 패자: 다음 입찰로 이동
                for _, loser_id in competing_bids[1:]:
                    if loser_id not in assignments:
                        bid_pointer[loser_id] += 1
                        self._log.debug(
                            f'CBBA: {loser_id} 패배 → 다음 입찰로 (ptr={bid_pointer[loser_id]})'
                        )

            if not resolved_this_round:
                break

        # 최종: robot_id → Task 객체 매핑
        result: dict[str, Task] = {}
        for rid, tid in assignments.items():
            if tid in task_map:
                result[rid] = task_map[tid]

        return result

    def _resolve_consensus_bundles(
        self,
        robots: list[RobotRecord],
        tasks: list[Task],
        bids: dict[str, list[tuple[float, str]]],
        cost_matrix: list[list[float]],
        bundle_n: int,
    ) -> dict[str, list[Task]]:
        """번들 합의 단계: 로봇당 최대 bundle_n개 임무를 충돌 없이 할당.

        bundle_n=1일 때 _resolve_consensus()와 동일한 결과를 보장합니다.

        알고리즘:
          1. 아직 할당받지 못한 임무 집합(unassigned_tasks) 관리
          2. 각 라운드에서 로봇별로 입찰 목록 중 아직 미할당 임무를 순서대로 탐색
          3. 같은 임무에 복수 로봇이 입찰하면 비용 최저 로봇이 낙찰
          4. 로봇의 번들이 bundle_n에 도달하거나 더 이상 낙찰 가능 임무가 없을 때까지 반복

        Returns:
            dict[str, list[Task]]: 로봇 ID → 할당된 Task 리스트
        """
        # task_id → Task 역방향 매핑
        task_map = {t.task_id: t for t in tasks}

        # 로봇별 현재 입찰 포인터
        bid_pointer = {robot.robot_id: 0 for robot in robots}

        # 로봇별 할당된 task_id 리스트 (순서 보존)
        assignments: dict[str, list[str]] = {robot.robot_id: [] for robot in robots}

        # 전체 미할당 임무 집합 (중복 방지)
        unassigned_tasks: set[str] = {t.task_id for t in tasks}

        # 최대 반복: 로봇 수 × 임무 수 × 번들 크기 (이론적 상한)
        max_iter = len(robots) * max(len(tasks), 1) * bundle_n

        for iteration in range(max_iter):
            # 아직 번들이 덜 찬 로봇 + 입찰 가능한 임무가 남은 로봇 목록
            active_robots = [
                robot for robot in robots
                if len(assignments[robot.robot_id]) < bundle_n
            ]
            if not active_robots:
                break

            if not unassigned_tasks:
                break

            # 이번 라운드 각 활성 로봇의 현재 최우선 미할당 임무 입찰
            # robot_id → (cost, task_id) — 이미 할당된 임무는 건너뜀
            current_bids: dict[str, tuple[float, str]] = {}

            for robot in active_robots:
                rid = robot.robot_id
                # 입찰 목록에서 아직 할당 안 된 임무를 첫 번째로 찾음
                while bid_pointer[rid] < len(bids[rid]):
                    cost, tid = bids[rid][bid_pointer[rid]]
                    if tid in unassigned_tasks:
                        current_bids[rid] = (cost, tid)
                        break
                    bid_pointer[rid] += 1

            if not current_bids:
                # 모든 활성 로봇의 입찰 가능 임무 소진
                break

            # 임무별 입찰 집계 (충돌 감지)
            # task_id → [(cost, robot_id), ...]
            task_bids: dict[str, list[tuple[float, str]]] = {}
            for rid, (cost, tid) in current_bids.items():
                if tid not in task_bids:
                    task_bids[tid] = []
                task_bids[tid].append((cost, rid))

            # 충돌 해소: 임무당 최저 비용 로봇 낙찰
            resolved_this_round = False

            for tid, competing_bids in task_bids.items():
                if tid not in unassigned_tasks:
                    # 이미 이번 라운드 내에서 다른 경쟁에서 배정된 경우
                    for _, bidder_id in competing_bids:
                        bid_pointer[bidder_id] += 1
                    continue

                # 최저 비용 로봇 = 낙찰자
                competing_bids.sort(key=lambda x: x[0])
                winner_cost, winner_id = competing_bids[0]

                # 낙찰자 번들에 추가
                if len(assignments[winner_id]) < bundle_n:
                    assignments[winner_id].append(tid)
                    unassigned_tasks.discard(tid)
                    resolved_this_round = True
                    self._log.debug(
                        f'CBBA 번들 iter={iteration}: {winner_id} → {tid} '
                        f'(cost={winner_cost:.2f}, '
                        f'bundle={len(assignments[winner_id])}/{bundle_n})'
                    )
                    # 낙찰자 포인터 진행 (다음 라운드에서 다른 임무 탐색)
                    bid_pointer[winner_id] += 1

                # 패자: 포인터 진행 (다음 입찰 후보로)
                for _, loser_id in competing_bids[1:]:
                    bid_pointer[loser_id] += 1
                    self._log.debug(
                        f'CBBA 번들: {loser_id} 패배 → 다음 입찰로 '
                        f'(ptr={bid_pointer[loser_id]})'
                    )

            if not resolved_this_round:
                break

        # 최종: robot_id → list[Task] 매핑 (빈 번들 로봇 제외)
        result: dict[str, list[Task]] = {}
        for rid, task_ids in assignments.items():
            if task_ids:
                result[rid] = [task_map[tid] for tid in task_ids if tid in task_map]

        return result

    # ── 헬퍼 메서드 ─────────────────────────────────────────────────────────

    def _get_robot_position(self, robot: RobotRecord) -> tuple[float, float, float]:
        """로봇 현재 위치 추출. pose 없으면 원점(0,0,0) 반환.

        ROS2 PoseStamped 또는 None 모두 처리.
        """
        if robot.pose is None:
            return (0.0, 0.0, 0.0)

        try:
            # geometry_msgs.msg.PoseStamped 구조
            p = robot.pose.pose.position
            return (p.x, p.y, p.z)
        except AttributeError:
            # 단위 테스트: (x, y, z) 튜플로 전달된 경우
            if isinstance(robot.pose, (tuple, list)) and len(robot.pose) >= 3:
                return (float(robot.pose[0]), float(robot.pose[1]), float(robot.pose[2]))
            return (0.0, 0.0, 0.0)

    @staticmethod
    def _distance(
        pos_a: tuple[float, float, float],
        pos_b: tuple[float, float, float],
    ) -> float:
        """3D 유클리드 거리 (m)."""
        return math.sqrt(
            (pos_a[0] - pos_b[0]) ** 2
            + (pos_a[1] - pos_b[1]) ** 2
            + (pos_a[2] - pos_b[2]) ** 2
        )

    @staticmethod
    def _check_capabilities(robot: RobotRecord, task: Task) -> bool:
        """로봇이 임무의 required_capabilities를 모두 보유하는지 확인.

        required_capabilities가 빈 리스트면 모든 로봇 수행 가능.
        """
        if not task.required_capabilities:
            return True
        robot_caps = set(robot.capabilities)
        return all(cap in robot_caps for cap in task.required_capabilities)
