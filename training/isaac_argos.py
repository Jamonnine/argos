"""
ARGOS Isaac Lab 환경 정의 — 스켈레톤

Isaac Lab GPU 병렬 환경으로 ARGOS UGV를 학습시키는 환경 클래스.
RTX 4050 (VRAM 6GB) 기준 64~128개 병렬 환경 목표.

의존성 (Isaac Lab 컨테이너 내부):
    isaaclab (IsaacLab 0.3+)
    torch (CUDA 지원 내장)
    gymnasium

실행 환경:
    docker compose exec isaac-lab python /workspace/training/isaac_argos.py

아키텍처 설계:
    ArgosUGVEnvCfg  — 환경 설정 (로봇 모델, 물리, 보상)
    ArgosUGVEnv     — 실제 환경 클래스 (IsaacLab DirectRLEnv 상속)
    main()          — 단독 실행 시 기본 학습 루프

참조:
    - Isaac Lab DirectRLEnv: https://isaac-sim.github.io/IsaacLab/source/tutorials/03_envs/create_direct_rl_env.html
    - ARGOS URDF: ros2_ws/src/argos_description/urdf/argos_ugv.xacro
    - MARL 환경: training/marl_env.py (ROS2 없는 순수 Python 버전)
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

import numpy as np

# ──────────────────────────────────────────
# Isaac Lab 조건부 임포트
# Isaac Lab 없는 환경에서도 스켈레톤 구조 확인 가능하도록 분기
# ──────────────────────────────────────────
try:
    import torch

    # Isaac Lab 핵심 모듈
    import isaaclab.sim as sim_utils
    from isaaclab.assets import ArticulationCfg, AssetBaseCfg
    from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
    from isaaclab.scene import InteractiveSceneCfg
    from isaaclab.sim import SimulationCfg
    from isaaclab.sim.converters import UrdfConverterCfg
    from isaaclab.terrains import TerrainImporterCfg
    from isaaclab.utils import configclass
    from isaaclab.utils.math import subtract_frame_transforms

    ISAAC_LAB_AVAILABLE = True
except ImportError:
    ISAAC_LAB_AVAILABLE = False
    print("[WARNING] Isaac Lab 미설치 환경. 스켈레톤 구조만 확인 가능.")
    print("          Isaac Lab 컨테이너 내에서 실행하세요.")
    print("          docker compose exec isaac-lab python training/isaac_argos.py")

# ──────────────────────────────────────────
# 경로 상수
# ──────────────────────────────────────────
# ARGOS UGV URDF 경로 (Isaac Lab 컨테이너 내 마운트 경로)
ARGOS_URDF_PATH = "/workspace/ros2_ws/src/argos_description/urdf/argos_ugv.xacro"

# USD 변환 캐시 경로 (최초 변환 후 재사용)
ARGOS_USD_PATH = "/workspace/checkpoints/argos_ugv.usd"

# 기본 Isaac Sim 에셋 경로
ISAACLAB_ASSETS_ROOT = os.environ.get(
    "ISAACLAB_ASSETS_ROOT", "/isaac-sim/IsaacLab/source/extensions/omni.isaac.lab_assets"
)


# ──────────────────────────────────────────
# 환경 설정 (Dataclass / @configclass)
# ──────────────────────────────────────────

@dataclass
class ArgosUGVEnvCfg:
    """
    ARGOS UGV Isaac Lab 환경 설정.

    주요 파라미터:
        num_envs: 병렬 환경 수 (RTX 4050 권장: 64)
        env_spacing: 환경 간격 (m) — 로봇이 서로 간섭하지 않도록
        episode_length_s: 에피소드 최대 시간 (초)
        map_size: 탐색 맵 크기 (m)
    """

    # 병렬 환경
    num_envs: int = int(os.environ.get("ARGOS_NUM_ENVS", 64))
    env_spacing: float = 25.0   # 환경 간격 (m) — 20m 맵 + 여유

    # 시뮬레이션 설정
    sim_dt: float = 0.02        # 50Hz 물리 스텝
    decimation: int = 4         # RL policy: 12.5Hz (50 / 4)
    episode_length_s: float = 60.0  # 에피소드 최대 60초

    # 맵 설정 (marl_env.py와 일치)
    map_size: float = 20.0       # 20m × 20m

    # 로봇 물리
    robot_mass_kg: float = 15.0
    max_wheel_velocity: float = 5.0  # rad/s

    # 보상 계수 (marl_env.py와 동일 설계)
    reward_coverage_coeff: float = 1.0
    reward_fire_coeff: float = 2.0
    reward_collision_penalty: float = 10.0

    # 커리큘럼
    curriculum_level: int = int(os.environ.get("ARGOS_CURRICULUM_LEVEL", 1))

    # 관측 차원 (marl_env.py와 동일 설계, 단일 로봇 기준)
    # [x, y, heading, battery] + [coverage] + [fire_x, fire_y]
    obs_dim: int = 4 + 1 + 2     # = 7


# ──────────────────────────────────────────
# Isaac Lab 환경 클래스 (스켈레톤)
# ──────────────────────────────────────────

if ISAAC_LAB_AVAILABLE:

    @configclass
    class ArgosSceneCfg(InteractiveSceneCfg):
        """
        ARGOS 시뮬레이션 씬 구성.

        구성 요소:
            - ground: 평지 지형 (Phase E 기본 → Phase F에서 불규칙 지형 교체)
            - robot: ARGOS UGV (URDF → USD 자동 변환)
            - fire_marker: 화점 시각화 (구 형태 파티클)
        """

        # 지형: 평지 (20×20m 소방 현장 시나리오)
        terrain = TerrainImporterCfg(
            prim_path="/World/ground",
            terrain_type="plane",
            collision_group=-1,
            physics_material=sim_utils.RigidBodyMaterialCfg(
                friction_combine_mode="multiply",
                restitution_combine_mode="multiply",
                static_friction=1.0,
                dynamic_friction=1.0,
            ),
            debug_vis=False,
        )

        # ARGOS UGV 로봇
        # URDF를 USD로 자동 변환 (ArgosUrdfConverter 사용)
        # 실제 xacro 파일: ros2_ws/src/argos_description/urdf/argos_ugv.xacro
        robot = ArticulationCfg(
            prim_path="{ENV_REGEX_NS}/Robot",
            spawn=sim_utils.UrdfFileCfg(
                asset_path=ARGOS_URDF_PATH,
                # xacro 처리: ros2_ws가 마운트되어 있어야 함
                # 미처리 시 argos_ugv.urdf로 직접 변환 대체 가능
                fix_base=False,
                make_instanceable=True,
            ),
            init_state=ArticulationCfg.InitialStateCfg(
                pos=(0.0, 0.0, 0.1),    # z=0.1m (지면에서 약간 위)
                joint_pos={".*": 0.0},
            ),
            actuators={
                # 4륜 차동구동 (ARGOS UGV 기본 구성)
                "wheel_drives": sim_utils.ImplicitActuatorCfg(
                    joint_names_expr=[".*_wheel_joint"],
                    velocity_limit=5.0,   # rad/s
                    effort_limit=20.0,    # Nm
                    stiffness=0.0,        # 속도 제어 → stiffness=0
                    damping=1000.0,
                ),
            },
        )

    class ArgosUGVEnv(DirectRLEnv):
        """
        ARGOS UGV Isaac Lab GPU 병렬 환경.

        상속: DirectRLEnv (IsaacLab 0.3+)
        인터페이스: gymnasium.Env 호환

        주요 메서드:
            _setup_scene()        — 씬 초기화 (로봇, 지형, 화점)
            _pre_physics_step()   — 액션 → 휠 속도 변환
            _get_observations()   — 관측 벡터 구성
            _get_rewards()        — 보상 계산
            _get_dones()          — 종료 조건 판단
            _reset_idx()          — 에피소드별 초기화
        """

        cfg: ArgosUGVEnvCfg
        scene_cfg: ArgosSceneCfg

        def __init__(self, cfg: ArgosUGVEnvCfg, render_mode: str | None = None):
            # Isaac Lab DirectRLEnvCfg 매핑
            isaac_cfg = DirectRLEnvCfg(
                num_envs=cfg.num_envs,
                env_spacing=cfg.env_spacing,
                sim=SimulationCfg(dt=cfg.sim_dt, render_interval=cfg.decimation),
                decimation=cfg.decimation,
                episode_length_s=cfg.episode_length_s,
                observation_space=cfg.obs_dim,
                action_space=4,  # Discrete(4): 전진/좌회전/우회전/정지
            )
            super().__init__(cfg=isaac_cfg, render_mode=render_mode)
            self._argos_cfg = cfg

            # 환경별 상태 버퍼 (GPU 텐서)
            self._coverage_map = torch.zeros(
                (cfg.num_envs, int(cfg.map_size), int(cfg.map_size)),
                device=self.device,
                dtype=torch.bool,
            )
            self._fire_positions = torch.zeros(
                (cfg.num_envs, 2), device=self.device, dtype=torch.float32
            )
            self._battery = torch.full(
                (cfg.num_envs,), 100.0, device=self.device, dtype=torch.float32
            )

        def _setup_scene(self) -> None:
            """씬 초기화: 로봇·지형·화점 스폰."""
            # 씬 구성 (ArgosSceneCfg 기반)
            self.scene_cfg = ArgosSceneCfg(
                num_envs=self._argos_cfg.num_envs,
                env_spacing=self._argos_cfg.env_spacing,
            )
            # 지형 및 로봇 스폰
            self._robot = self.scene_cfg.robot.spawn(self.scene)
            self.scene.terrain = self.scene_cfg.terrain.spawn(self.scene)

            # 화점 위치 무작위 초기화
            self._randomize_fire_positions()

            # 조명 (기본 하늘광)
            sim_utils.DomeLightCfg(
                intensity=3000.0, color=(0.75, 0.75, 0.75)
            ).spawn(prim_path="/World/skyLight")

        def _pre_physics_step(self, actions: torch.Tensor) -> None:
            """
            이산 액션 → 연속 휠 속도 변환.

            액션 매핑:
                0 (전진):    left=+v, right=+v
                1 (좌회전):  left=-w, right=+w
                2 (우회전):  left=+w, right=-w
                3 (정지):    left=0,  right=0
            """
            v_lin = 1.0    # 전진 선속도 (m/s)
            v_ang = 0.8    # 회전 각속도 보정

            wheel_vel = torch.zeros(
                (self.num_envs, 4), device=self.device, dtype=torch.float32
            )
            # 0: 전진
            fwd_mask = actions == 0
            wheel_vel[fwd_mask, :] = v_lin
            # 1: 좌회전
            left_mask = actions == 1
            wheel_vel[left_mask, 0] = -v_ang   # 왼쪽 휠
            wheel_vel[left_mask, 1] = -v_ang
            wheel_vel[left_mask, 2] = +v_ang   # 오른쪽 휠
            wheel_vel[left_mask, 3] = +v_ang
            # 2: 우회전
            right_mask = actions == 2
            wheel_vel[right_mask, 0] = +v_ang
            wheel_vel[right_mask, 1] = +v_ang
            wheel_vel[right_mask, 2] = -v_ang
            wheel_vel[right_mask, 3] = -v_ang
            # 3: 정지 → 이미 0

            # 로봇 관절에 목표 속도 설정
            self._robot.set_joint_velocity_target(wheel_vel)

        def _get_observations(self) -> dict:
            """
            관측 벡터 구성 (GPU 텐서).

            반환:
                {"policy": tensor(num_envs, obs_dim)}
                obs_dim = 7: [x, y, heading, battery, coverage, fire_x, fire_y]
            """
            # 로봇 위치
            robot_pos = self._robot.data.root_pos_w[:, :2]     # (N, 2): x, y
            # 로봇 헤딩 (yaw)
            robot_yaw = self._get_yaw()                          # (N, 1)
            # 배터리 (정규화)
            battery_norm = (self._battery / 100.0).unsqueeze(-1) # (N, 1)
            # 팀 커버리지 (스칼라)
            coverage = self._coverage_map.float().mean(dim=(1, 2)).unsqueeze(-1)  # (N, 1)
            # 화점 위치 (정규화)
            fire_norm = self._fire_positions / self._argos_cfg.map_size  # (N, 2)

            obs = torch.cat(
                [
                    robot_pos / self._argos_cfg.map_size,  # (N, 2) 정규화
                    robot_yaw / (2 * math.pi),             # (N, 1) 정규화
                    battery_norm,                          # (N, 1)
                    coverage,                              # (N, 1)
                    fire_norm,                             # (N, 2)
                ],
                dim=-1,
            )  # (N, 7)

            # 커버리지 맵 업데이트
            self._update_coverage(robot_pos)

            return {"policy": obs}

        def _get_rewards(self) -> torch.Tensor:
            """
            보상 계산 (marl_env.py와 동일 설계).

            coverage_delta × 1.0 + fire_proximity × 2.0 - collision × 10.0
            """
            cfg = self._argos_cfg

            # 커버리지 델타
            prev_coverage = getattr(self, "_prev_coverage", None)
            curr_coverage = self._coverage_map.float().mean(dim=(1, 2))
            if prev_coverage is None:
                coverage_reward = torch.zeros(self.num_envs, device=self.device)
            else:
                coverage_delta = curr_coverage - prev_coverage
                coverage_reward = cfg.reward_coverage_coeff * coverage_delta
            self._prev_coverage = curr_coverage.clone()

            # 화점 근접 보상
            robot_pos = self._robot.data.root_pos_w[:, :2]
            dist_to_fire = torch.norm(robot_pos - self._fire_positions, dim=-1)
            max_dist = cfg.map_size * math.sqrt(2)
            proximity = 1.0 - dist_to_fire / max_dist
            fire_reward = cfg.reward_fire_coeff * proximity * 0.1

            # 충돌 패널티 (접촉 감지)
            collision_mask = self._detect_collisions()
            collision_penalty = cfg.reward_collision_penalty * collision_mask.float()

            # 스텝 패널티
            step_penalty = torch.full(
                (self.num_envs,), -0.01, device=self.device
            )

            return coverage_reward + fire_reward - collision_penalty + step_penalty

        def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
            """
            종료 조건.

            Returns:
                (terminated, truncated)
                terminated: 화점 발견 or 배터리 소진
                truncated: 에피소드 최대 시간 초과
            """
            # 화점 발견 (2m 이내)
            robot_pos = self._robot.data.root_pos_w[:, :2]
            dist_to_fire = torch.norm(robot_pos - self._fire_positions, dim=-1)
            fire_found = dist_to_fire < 2.0

            # 배터리 소진
            battery_dead = self._battery <= 0.0

            terminated = fire_found | battery_dead
            truncated = self.episode_length_buf >= self.max_episode_length

            return terminated, truncated

        def _reset_idx(self, env_ids: torch.Tensor) -> None:
            """특정 환경 인덱스 초기화 (에피소드 종료 후 자동 호출)."""
            super()._reset_idx(env_ids)

            # 로봇 위치 초기화 (환경별 원점 근처 랜덤)
            n = len(env_ids)
            init_pos = torch.zeros((n, 3), device=self.device)
            init_pos[:, :2] = (
                torch.rand((n, 2), device=self.device) * 2.0 - 1.0
            )  # [-1, 1] m 범위
            init_pos[:, 2] = 0.1  # z = 0.1m
            self._robot.write_root_pose_to_sim(init_pos, env_ids=env_ids)

            # 배터리 초기화
            self._battery[env_ids] = 100.0

            # 커버리지 맵 초기화
            self._coverage_map[env_ids] = False

            # 화점 재배치
            self._randomize_fire_positions(env_ids)

        # ──────────────────────────────────────────
        # 내부 헬퍼
        # ──────────────────────────────────────────

        def _get_yaw(self) -> torch.Tensor:
            """쿼터니언 → yaw 각도 변환."""
            quat = self._robot.data.root_quat_w   # (N, 4): w, x, y, z
            w, x, y, z = quat[:, 0], quat[:, 1], quat[:, 2], quat[:, 3]
            yaw = torch.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
            return yaw.unsqueeze(-1)

        def _update_coverage(self, robot_pos: torch.Tensor) -> None:
            """로봇 현재 위치를 커버리지 맵에 마킹."""
            map_size = int(self._argos_cfg.map_size)
            # 위치 → 격자 인덱스
            grid_x = (robot_pos[:, 0]).long().clamp(0, map_size - 1)
            grid_y = (robot_pos[:, 1]).long().clamp(0, map_size - 1)
            env_ids = torch.arange(self.num_envs, device=self.device)
            self._coverage_map[env_ids, grid_x, grid_y] = True

        def _detect_collisions(self) -> torch.Tensor:
            """간단한 충돌 감지 (맵 경계 이탈 기준)."""
            robot_pos = self._robot.data.root_pos_w[:, :2]
            half = self._argos_cfg.map_size / 2
            out_of_bounds = (
                (robot_pos[:, 0].abs() > half) | (robot_pos[:, 1].abs() > half)
            )
            return out_of_bounds

        def _randomize_fire_positions(
            self, env_ids: torch.Tensor | None = None
        ) -> None:
            """화점 위치 무작위 초기화."""
            if env_ids is None:
                env_ids = torch.arange(self.num_envs, device=self.device)

            n = len(env_ids)
            half = self._argos_cfg.map_size / 2 - 2.0  # 경계에서 2m 안쪽
            fire_pos = (torch.rand((n, 2), device=self.device) * 2 - 1) * half
            self._fire_positions[env_ids] = fire_pos


# ──────────────────────────────────────────
# Isaac Lab 없는 환경 — 더미 클래스 (구조 확인용)
# ──────────────────────────────────────────
else:
    class ArgosUGVEnv:  # type: ignore[no-redef]
        """Isaac Lab 없는 환경에서 스켈레톤 구조 확인용 더미."""

        def __init__(self, cfg: ArgosUGVEnvCfg | None = None, **kwargs):
            print("[ArgosUGVEnv] Isaac Lab 없음 — 더미 클래스")
            print("  실제 실행: docker compose exec isaac-lab python training/isaac_argos.py")


# ──────────────────────────────────────────
# 메인 — 기본 학습 루프 (단독 실행 시)
# ──────────────────────────────────────────
def main():
    """
    기본 학습 루프.

    실제 학습 시에는 stable-baselines3 또는 rllib과 연동:
        from stable_baselines3 import PPO
        env = ArgosUGVEnv(ArgosUGVEnvCfg(num_envs=64))
        model = PPO("MlpPolicy", env, verbose=1)
        model.learn(total_timesteps=1_000_000)

    MARL (Level 3) 시에는 marl_env.py 직접 사용 권장:
        from training.marl_env import make_argos_env
        env = make_argos_env(curriculum_level=3)
    """
    print("=== ARGOS Isaac Lab 환경 초기화 ===")

    cfg = ArgosUGVEnvCfg()
    print(f"병렬 환경 수: {cfg.num_envs}")
    print(f"커리큘럼 레벨: {cfg.curriculum_level}")
    print(f"에피소드 최대 시간: {cfg.episode_length_s}초")
    print(f"GPU: {os.environ.get('NVIDIA_VISIBLE_DEVICES', '미설정')}")

    if not ISAAC_LAB_AVAILABLE:
        print("\n[INFO] Isaac Lab 없음 — marl_env.py로 대체 실행")
        print("       ROS2 독립 순수 Python 환경으로 학습 가능:")
        print("       python training/marl_env.py")
        return

    # Isaac Lab 환경 실행
    env = ArgosUGVEnv(cfg)
    obs, info = env.reset()
    print(f"\n환경 초기화 완료")
    print(f"Observation shape: {obs['policy'].shape}")

    # 100 스텝 샘플 실행
    print("\n100 스텝 샘플 실행 중...")
    for step in range(100):
        # 랜덤 액션 (실제 학습 시에는 policy.predict로 교체)
        actions = torch.randint(0, 4, (cfg.num_envs,))
        obs, rewards, terminated, truncated, info = env.step(actions)

        done_count = (terminated | truncated).sum().item()
        if done_count > 0:
            print(f"  스텝 {step+1}: {done_count}개 환경 종료 후 자동 리셋")

    print("\n샘플 실행 완료.")
    env.close()


if __name__ == "__main__":
    main()
