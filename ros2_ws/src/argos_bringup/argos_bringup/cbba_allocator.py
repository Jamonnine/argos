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

알고리즘 개요 (3단계):
  Phase 1 - 번들 빌드: 각 로봇이 자신의 capability 매칭 임무를 비용순 선택
  Phase 2 - 경매:     로봇별 입찰가(bid) 계산 + 최저 비용 임무 선점
  Phase 3 - 합의:     충돌(동일 임무 복수 입찰) 해소 → 최적 할당 확정

오케스트레이터 연동:
  OrchestratorNode.on_activate() 에서 self.allocator = CBBAAllocator(self.get_logger()) 생성
  화재 감지 시: assignments = self.allocator.allocate(robot_list, task_list)

성능:
  로봇 N개, 임무 M개 기준: O(N × M) per iteration, 최대 N회 합의 순환
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

    def __init__(self, logger=None):
        """초기화.

        logger: rclpy.node.Node.get_logger() 반환값 (ROS2 환경) 또는
                logging.Logger (순수 Python 환경, 단위 테스트용)
        """
        if logger is None:
            self._log = logging.getLogger('CBBAAllocator')
        else:
            self._log = logger

    # ── 공개 인터페이스 ──────────────────────────────────────────────────────

    def allocate(
        self,
        robots: list[RobotRecord],
        tasks: list[Task],
    ) -> dict[str, Task]:
        """CBBA 3단계 실행 → 로봇 ID → 할당된 Task 매핑 반환.

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
        if not robots:
            self._log.warning('CBBA: 로봇 목록이 비어 있습니다. 할당 중단.')
            return {}

        if not tasks:
            self._log.info('CBBA: 임무 목록이 비어 있습니다.')
            return {}

        self._log.info(
            f'CBBA 시작: 로봇 {len(robots)}대, 임무 {len(tasks)}건'
        )

        # Phase 1: 비용 행렬 계산
        cost_matrix = self._build_cost_matrix(robots, tasks)

        # Phase 2: 경매 — 각 로봇이 최저 비용 임무 입찰
        bids = self._run_auction(robots, tasks, cost_matrix)

        # Phase 3: 합의 — 충돌 해소 → 최적 할당
        assignments = self._resolve_consensus(robots, tasks, bids, cost_matrix)

        # 결과 로그
        for robot_id, task in assignments.items():
            self._log.info(
                f'CBBA 할당: {robot_id} → {task.task_id} '
                f'(type={task.task_type}, priority={task.priority:.2f})'
            )

        unassigned = [
            t.task_id for t in tasks
            if t.task_id not in {ta.task_id for ta in assignments.values()}
        ]
        if unassigned:
            self._log.warning(f'CBBA: 미할당 임무 {len(unassigned)}건: {unassigned}')

        return assignments

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
        """경매 단계: 각 로봇이 자신의 번들(최저 비용 임무 집합) 구성.

        각 로봇은 capability 충족 임무 중 비용이 가장 낮은 것을 1개 입찰.
        번들 크기를 1로 제한하는 이유:
          소방 현장에서는 한 로봇이 여러 임무를 동시 수행하기 어려움.
          Phase E(v4.0)에서 번들 크기 N으로 확장 예정 (CBBA 원본 알고리즘).

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
