"""
ARGOS MARL 학습 환경 — 멀티에이전트 소방 로봇 강화학습

참조 논문: Cyborg/Griffith (2026.02) — 3단계 커리큘럼, 소방 로봇 99.67% 자율 성공률
인터페이스: gymnasium.Env (PettingZoo 병렬 환경 추후 전환 고려)
ROS2 의존 없음 — 순수 Python + NumPy (학습 서버/로컬 모두 실행 가능)

커리큘럼:
    Level 1: 단일 로봇, 장애물 없음 → 탐색 행동 학습
    Level 2: 단일 로봇, 장애물 있음 → 충돌 회피 학습
    Level 3: 3로봇, 다중 화재 → 분산 협업 학습

설계 원칙:
    - 보상 설계: coverage_delta × 1.0 + fire_proximity × 2.0 - collision × 10.0
    - 맵: 20×20 격자 (실제 소방 현장 ~20m × 20m 대응)
    - 배터리 모델링 → Phase F 실세계 전환 시 현실적 제약 반영
"""

from __future__ import annotations

import copy
import random
from typing import Any

import gymnasium as gym
import numpy as np
from gymnasium import spaces


# ──────────────────────────────────────────
# 상수
# ──────────────────────────────────────────
MAP_SIZE = 20          # 격자 크기 (20×20)
CELL_SIZE = 1.0        # 셀 1개 = 1m
MAX_STEPS = 500        # 에피소드 최대 스텝

# 액션: 이산형 4가지
ACTION_FORWARD   = 0
ACTION_TURN_LEFT = 1
ACTION_TURN_RIGHT = 2
ACTION_STOP      = 3

# 헤딩: 0=북(+y), 1=동(+x), 2=남(-y), 3=서(-x)
HEADING_DX = [0, 1, 0, -1]   # 동서 이동량
HEADING_DY = [1, 0, -1, 0]   # 남북 이동량

# 보상 계수
REWARD_COVERAGE_COEFF   =  1.0
REWARD_FIRE_COEFF       =  2.0
REWARD_COLLISION_PENALTY = 10.0
REWARD_STEP_PENALTY      = -0.01   # 시간 지체 패널티

# 배터리
BATTERY_MAX      = 100.0
BATTERY_MOVE_COST = 1.0    # 이동 시 소모
BATTERY_STOP_COST = 0.1   # 정지 시도 소모 (유지 비용)

# 커리큘럼 레벨 설정
CURRICULUM = {
    1: {"n_robots": 1, "n_fires": 1, "n_obstacles": 0},
    2: {"n_robots": 1, "n_fires": 1, "n_obstacles": 8},
    3: {"n_robots": 3, "n_fires": 3, "n_obstacles": 12},
}


# ──────────────────────────────────────────
# 보조 자료구조
# ──────────────────────────────────────────
class RobotState:
    """단일 로봇의 상태 컨테이너."""

    __slots__ = ["x", "y", "heading", "battery", "active"]

    def __init__(self, x: int, y: int, heading: int = 0):
        self.x = x
        self.y = y
        self.heading = heading        # 0~3 (북/동/남/서)
        self.battery = BATTERY_MAX
        self.active = True            # 배터리 소진 시 False

    def as_array(self) -> np.ndarray:
        """Observation 벡터: [x, y, heading, battery] — 정규화."""
        return np.array(
            [
                self.x / MAP_SIZE,
                self.y / MAP_SIZE,
                self.heading / 3.0,
                self.battery / BATTERY_MAX,
            ],
            dtype=np.float32,
        )


# ──────────────────────────────────────────
# 메인 환경 클래스
# ──────────────────────────────────────────
class ArgosFireEnv(gym.Env):
    """
    ARGOS 멀티에이전트 소방 로봇 MARL 환경.

    Observation Space (per agent):
        [로봇 상태 4차원] × n_robots
        + [맵 커버리지 스칼라 1]
        + [화점 위치 (x, y) × n_fires]
        총: 4 × n_robots + 1 + 2 × n_fires

    Action Space (per agent):
        Discrete(4) — {전진, 좌회전, 우회전, 정지}

    Step Return:
        obs, reward, terminated, truncated, info
        - terminated: 모든 화점 발견 or 배터리 전원 소진
        - truncated: MAX_STEPS 초과
    """

    metadata = {"render_modes": ["human", "ansi"], "render_fps": 10}

    def __init__(
        self,
        curriculum_level: int = 1,
        render_mode: str | None = None,
        seed: int | None = None,
    ):
        super().__init__()
        assert curriculum_level in CURRICULUM, f"유효한 커리큘럼 레벨: {list(CURRICULUM)}"

        self.curriculum_level = curriculum_level
        self.render_mode = render_mode
        self._rng = np.random.default_rng(seed)

        cfg = CURRICULUM[curriculum_level]
        self.n_robots    = cfg["n_robots"]
        self.n_fires     = cfg["n_fires"]
        self.n_obstacles = cfg["n_obstacles"]

        # Observation: 각 로봇 4 + 커버리지 1 + 화점 xy × n_fires
        obs_dim = 4 * self.n_robots + 1 + 2 * self.n_fires

        # 멀티에이전트 → 에이전트별 observation을 하나의 공유 벡터로 제공
        # (중앙화 학습 / 분산 실행 CTDE 방식 지원)
        self.observation_space = spaces.Box(
            low=0.0, high=1.0, shape=(obs_dim,), dtype=np.float32
        )
        # 에이전트별 액션 (학습 시 vectorized)
        self.action_space = spaces.Discrete(4)

        # 내부 상태 (reset()에서 초기화)
        self.robots: list[RobotState] = []
        self.fire_positions: list[tuple[int, int]] = []
        self.obstacle_map: np.ndarray = np.zeros((MAP_SIZE, MAP_SIZE), dtype=bool)
        self.coverage_map: np.ndarray = np.zeros((MAP_SIZE, MAP_SIZE), dtype=bool)
        self.discovered_fires: set[tuple[int, int]] = set()
        self.step_count: int = 0

    # ──────────────────────────────────────────
    # gym 필수 메서드
    # ──────────────────────────────────────────

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict[str, Any] | None = None,
    ) -> tuple[np.ndarray, dict]:
        """에피소드 초기화. 맵·로봇·화점·장애물 무작위 배치."""
        super().reset(seed=seed)
        if seed is not None:
            self._rng = np.random.default_rng(seed)

        self.step_count = 0
        self.obstacle_map[:] = False
        self.coverage_map[:] = False
        self.discovered_fires.clear()

        # 배치 순서: 장애물 → 화점 → 로봇 (충돌 없는 위치 보장)
        occupied: set[tuple[int, int]] = set()

        self._place_obstacles(occupied)
        self.fire_positions = self._place_entities(self.n_fires, occupied)
        robot_starts = self._place_entities(self.n_robots, occupied)

        self.robots = [
            RobotState(x, y, heading=int(self._rng.integers(0, 4)))
            for x, y in robot_starts
        ]

        # 초기 위치 커버리지 마킹
        for r in self.robots:
            self.coverage_map[r.x, r.y] = True

        obs = self._get_obs()
        info = self._get_info()
        return obs, info

    def step(
        self, action: int | np.ndarray
    ) -> tuple[np.ndarray, float, bool, bool, dict]:
        """
        단일 스텝 실행.

        멀티에이전트 사용 패턴:
            # CTDE (중앙화 학습 / 분산 실행)
            actions = policy.predict(obs)  # shape: (n_robots,)
            obs, reward, terminated, truncated, info = env.step(actions)

        단일 에이전트 사용 패턴:
            obs, reward, terminated, truncated, info = env.step(action_int)
        """
        # 단일 int → 배열로 통일
        if isinstance(action, (int, np.integer)):
            actions = [int(action)] * self.n_robots
        else:
            actions = [int(a) for a in np.asarray(action).flatten()]
            if len(actions) < self.n_robots:
                # 액션 부족 시 STOP으로 패딩
                actions += [ACTION_STOP] * (self.n_robots - len(actions))

        prev_coverage = self.coverage_map.sum()
        total_reward = 0.0

        for i, (robot, act) in enumerate(zip(self.robots, actions)):
            if not robot.active:
                continue  # 배터리 소진 로봇은 스킵

            reward, collision = self._apply_action(robot, act)
            total_reward += reward

            # 화점 발견 보상 (최초 1회)
            for fx, fy in self.fire_positions:
                if (fx, fy) not in self.discovered_fires:
                    dist = abs(robot.x - fx) + abs(robot.y - fy)
                    if dist <= 2:  # 2셀 이내 감지 범위
                        self.discovered_fires.add((fx, fy))
                        total_reward += REWARD_FIRE_COEFF * 5.0  # 최초 발견 보너스

        # 커버리지 델타 보상 (팀 공동)
        new_coverage = self.coverage_map.sum()
        coverage_delta = (new_coverage - prev_coverage) / (MAP_SIZE * MAP_SIZE)
        total_reward += REWARD_COVERAGE_COEFF * coverage_delta

        # 스텝 패널티 (탐색 효율 유도)
        total_reward += REWARD_STEP_PENALTY

        self.step_count += 1

        # 종료 조건
        all_fires_found = len(self.discovered_fires) == self.n_fires
        all_batteries_dead = all(not r.active for r in self.robots)
        terminated = all_fires_found or all_batteries_dead
        truncated = self.step_count >= MAX_STEPS

        obs = self._get_obs()
        info = self._get_info()
        info["coverage_delta"] = float(coverage_delta)
        info["fires_found"] = len(self.discovered_fires)

        if self.render_mode == "human":
            self.render()

        return obs, float(total_reward), terminated, truncated, info

    def render(self) -> str | None:
        """ASCII 맵 렌더링."""
        grid = [["." for _ in range(MAP_SIZE)] for _ in range(MAP_SIZE)]

        # 장애물
        for x in range(MAP_SIZE):
            for y in range(MAP_SIZE):
                if self.obstacle_map[x, y]:
                    grid[y][x] = "#"

        # 커버리지
        for x in range(MAP_SIZE):
            for y in range(MAP_SIZE):
                if self.coverage_map[x, y] and grid[y][x] == ".":
                    grid[y][x] = "·"

        # 화점
        for fx, fy in self.fire_positions:
            marker = "F" if (fx, fy) in self.discovered_fires else "f"
            grid[fy][fx] = marker

        # 로봇
        heading_chars = ["^", ">", "v", "<"]
        for i, robot in enumerate(self.robots):
            if robot.active:
                grid[robot.y][robot.x] = heading_chars[robot.heading]
            else:
                grid[robot.y][robot.x] = "X"  # 배터리 소진

        # 출력
        lines = []
        lines.append(f"Step: {self.step_count}/{MAX_STEPS}  Level: {self.curriculum_level}")
        lines.append(f"Coverage: {self.coverage_map.mean()*100:.1f}%  Fires: {len(self.discovered_fires)}/{self.n_fires}")
        lines.append("+" + "-" * MAP_SIZE + "+")
        for row in reversed(grid):  # y축 반전 (위쪽=북)
            lines.append("|" + "".join(row) + "|")
        lines.append("+" + "-" * MAP_SIZE + "+")

        output = "\n".join(lines)
        if self.render_mode == "human":
            print(output)
        return output

    def close(self):
        pass

    # ──────────────────────────────────────────
    # 커리큘럼 제어
    # ──────────────────────────────────────────

    def set_curriculum_level(self, level: int) -> None:
        """외부에서 커리큘럼 레벨 변경 (auto-curriculum 지원)."""
        assert level in CURRICULUM, f"유효한 레벨: {list(CURRICULUM)}"
        self.curriculum_level = level
        cfg = CURRICULUM[level]
        self.n_robots    = cfg["n_robots"]
        self.n_fires     = cfg["n_fires"]
        self.n_obstacles = cfg["n_obstacles"]

        # observation_space 재계산
        obs_dim = 4 * self.n_robots + 1 + 2 * self.n_fires
        self.observation_space = spaces.Box(
            low=0.0, high=1.0, shape=(obs_dim,), dtype=np.float32
        )

    # ──────────────────────────────────────────
    # 내부 헬퍼
    # ──────────────────────────────────────────

    def _apply_action(
        self, robot: RobotState, action: int
    ) -> tuple[float, bool]:
        """
        액션 적용 → (reward, collision) 반환.
        배터리 소진 시 robot.active = False.
        """
        collision = False

        if action == ACTION_FORWARD:
            dx = HEADING_DX[robot.heading]
            dy = HEADING_DY[robot.heading]
            nx, ny = robot.x + dx, robot.y + dy

            if self._is_valid(nx, ny):
                robot.x, robot.y = nx, ny
                self.coverage_map[nx, ny] = True
                robot.battery -= BATTERY_MOVE_COST
            else:
                # 벽/장애물 충돌
                collision = True
                robot.battery -= BATTERY_MOVE_COST * 0.5  # 충돌 시 절반 소모

        elif action == ACTION_TURN_LEFT:
            robot.heading = (robot.heading - 1) % 4
            robot.battery -= BATTERY_MOVE_COST * 0.3

        elif action == ACTION_TURN_RIGHT:
            robot.heading = (robot.heading + 1) % 4
            robot.battery -= BATTERY_MOVE_COST * 0.3

        elif action == ACTION_STOP:
            robot.battery -= BATTERY_STOP_COST

        # 배터리 하한
        robot.battery = max(0.0, robot.battery)
        if robot.battery <= 0.0:
            robot.active = False

        # 충돌 패널티
        reward = -REWARD_COLLISION_PENALTY if collision else 0.0

        # 화점 근접 보상 (연속적, 매 스텝)
        for fx, fy in self.fire_positions:
            if (fx, fy) not in self.discovered_fires:
                max_dist = MAP_SIZE * 2
                dist = abs(robot.x - fx) + abs(robot.y - fy)
                proximity = 1.0 - dist / max_dist
                reward += REWARD_FIRE_COEFF * proximity * 0.1  # 약한 유인 보상

        return reward, collision

    def _is_valid(self, x: int, y: int) -> bool:
        """유효한 이동 위치 여부 (맵 경계 + 장애물 체크)."""
        if not (0 <= x < MAP_SIZE and 0 <= y < MAP_SIZE):
            return False
        return not self.obstacle_map[x, y]

    def _place_obstacles(self, occupied: set[tuple[int, int]]) -> None:
        """장애물 무작위 배치 (클러스터 패턴으로 실제 건물 형태 근사)."""
        placed = 0
        attempts = 0
        while placed < self.n_obstacles and attempts < 1000:
            attempts += 1
            # 클러스터 중심
            cx = int(self._rng.integers(2, MAP_SIZE - 2))
            cy = int(self._rng.integers(2, MAP_SIZE - 2))
            # 2×2 또는 1×1 블록
            size = int(self._rng.integers(1, 3))
            for dx in range(size):
                for dy in range(size):
                    pos = (cx + dx, cy + dy)
                    if (
                        0 <= pos[0] < MAP_SIZE
                        and 0 <= pos[1] < MAP_SIZE
                        and pos not in occupied
                    ):
                        self.obstacle_map[pos[0], pos[1]] = True
                        occupied.add(pos)
                        placed += 1

    def _place_entities(
        self, n: int, occupied: set[tuple[int, int]]
    ) -> list[tuple[int, int]]:
        """n개 엔티티를 occupied 외 위치에 무작위 배치."""
        positions = []
        attempts = 0
        while len(positions) < n and attempts < 10000:
            attempts += 1
            x = int(self._rng.integers(0, MAP_SIZE))
            y = int(self._rng.integers(0, MAP_SIZE))
            pos = (x, y)
            if pos not in occupied:
                positions.append(pos)
                occupied.add(pos)
        return positions

    def _get_obs(self) -> np.ndarray:
        """전체 observation 벡터 구성."""
        obs_parts = []

        # 각 로봇 상태 (비활성 로봇은 0 벡터)
        for robot in self.robots:
            if robot.active:
                obs_parts.append(robot.as_array())
            else:
                obs_parts.append(np.zeros(4, dtype=np.float32))

        # 팀 커버리지 (스칼라)
        coverage = float(self.coverage_map.mean())
        obs_parts.append(np.array([coverage], dtype=np.float32))

        # 화점 위치 (정규화)
        for fx, fy in self.fire_positions:
            obs_parts.append(
                np.array([fx / MAP_SIZE, fy / MAP_SIZE], dtype=np.float32)
            )

        return np.concatenate(obs_parts)

    def _get_info(self) -> dict:
        """디버그 정보 딕셔너리."""
        return {
            "step": self.step_count,
            "curriculum_level": self.curriculum_level,
            "coverage_pct": float(self.coverage_map.mean() * 100),
            "fires_found": len(self.discovered_fires),
            "n_fires": self.n_fires,
            "robot_batteries": [r.battery for r in self.robots],
            "active_robots": sum(r.active for r in self.robots),
        }


# ──────────────────────────────────────────
# 팩토리 헬퍼 (gym.make 호환)
# ──────────────────────────────────────────

def make_argos_env(
    curriculum_level: int = 1,
    render_mode: str | None = None,
    seed: int | None = None,
) -> ArgosFireEnv:
    """
    편의 팩토리.

    사용 예:
        env = make_argos_env(curriculum_level=3, seed=42)
        obs, info = env.reset()
        for _ in range(100):
            action = env.action_space.sample()
            obs, reward, terminated, truncated, info = env.step(action)
            if terminated or truncated:
                obs, info = env.reset()
    """
    env = ArgosFireEnv(
        curriculum_level=curriculum_level,
        render_mode=render_mode,
        seed=seed,
    )
    return env


# ──────────────────────────────────────────
# 빠른 동작 확인 (직접 실행 시)
# ──────────────────────────────────────────
if __name__ == "__main__":
    print("=== ARGOS MARL 환경 동작 확인 ===\n")

    for level in [1, 2, 3]:
        print(f"--- 커리큘럼 Level {level} ---")
        env = make_argos_env(curriculum_level=level, render_mode="ansi", seed=level)
        obs, info = env.reset()

        print(f"Obs shape: {obs.shape}")
        print(f"Action space: {env.action_space}")
        print(env.render())

        total_reward = 0.0
        for step in range(50):
            # Level 3: n_robots개 액션 배열
            if level == 3:
                actions = np.array([env.action_space.sample() for _ in range(env.n_robots)])
            else:
                actions = env.action_space.sample()

            obs, reward, terminated, truncated, info = env.step(actions)
            total_reward += reward

            if terminated or truncated:
                print(f"  종료: step={step+1}, reward_sum={total_reward:.2f}, info={info}")
                break
        else:
            print(f"  50스텝 완료: reward_sum={total_reward:.2f}, info={info}")

        env.close()
        print()
