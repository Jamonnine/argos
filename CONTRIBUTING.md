# Contributing to ARGOS

Thank you for your interest in contributing to **ARGOS** — a heterogeneous firefighting robot orchestration platform built on ROS 2 Jazzy and Gazebo Harmonic.

ARGOS is licensed under the **Apache License 2.0**. Contributions are welcome from the robotics, fire safety, and autonomous systems communities.

---

## 1. Welcome

ARGOS (Autonomous Robot Ground-air Orchestration System) coordinates drones, UGVs, and legged robots as a unified team for fire scene reconnaissance and suppression support. The project is developed by a single firefighter-developer and is designed to be a practical, deployable system — not a demo.

- **Main repo**: https://github.com/Jamonnine/argos
- **License**: Apache 2.0 (main packages) | AGPL-3.0 (`argos_fire_ai` — see §7)
- **Language**: Python 3.12+ | ROS 2 Jazzy | Gazebo Harmonic
- **Tests**: 813 passing (as of v3.7)

---

## 2. Development Setup

### Prerequisites

- Ubuntu 24.04 (WSL2 recommended for Windows users; see `docs/wsl2-setup-guide.md`)
- ROS 2 Jazzy Jalisco

### Install ROS 2 Jazzy

```bash
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-gazebo-ros-pkgs
```

### Clone and Build

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/Jamonnine/argos.git
cd ..
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

> **Note (WSL2)**: Create a symlink `ln -s /mnt/c/path/to/argos/ros2_ws ~/ros2_ws` for convenience.

---

## 3. Running Tests

All tests live under `ros2_ws/src/argos_bringup/test/`.

```bash
cd ros2_ws/src/argos_bringup
python -m pytest test/ -q
```

Run with verbose output:

```bash
python -m pytest test/ -v --tb=short
```

Run a specific test file:

```bash
python -m pytest test/test_cbba_allocator.py -v
```

Integration tests using `launch_testing` (requires a sourced workspace):

```bash
python -m pytest test/test_fire_scenario_sil.py -v
```

All 813 tests must pass before a PR is merged. Do not submit a PR with failing tests.

---

## 4. Code Style

Please follow `docs/coding-style-guide.md`. Key rules:

- **Formatter**: Black (`line-length=100`)
- **Linter**: Flake8 (`max-line=100`, ignore `E501,W503`)
- **Type hints**: Encouraged for all public functions
- **Python version**: 3.12+
- **Docstrings**: Google style
- **Node class names**: `XxxNode` (PascalCase)
- **Topic names**: `sensor/reading` (lowercase with slashes)
- **Parameters**: `snake_case` via `declare_parameter()`
- **QoS**: Sensor topics → `BEST_EFFORT`; command topics → `RELIABLE + TRANSIENT_LOCAL`

Install pre-commit hooks:

```bash
pip install pre-commit
pre-commit install
```

Hooks run automatically on commit: `black`, `flake8`, `trailing-whitespace`.

---

## 5. PR Checklist

Before opening a pull request, confirm all of the following:

- [ ] `python -m pytest test/ -q` passes (all 813+ tests green)
- [ ] `python -m ast.parse <changed_files>` (or `python -c "import ast; ast.parse(open('file.py').read())"`) raises no errors
- [ ] Robot capability names use the canonical form: `has_thermal`, `can_fly`, `has_gas_sensor`, `has_hose`, `has_water_curtain`
- [ ] Logic modules (`sensor_fusion.py`, `robot_dispatcher.py`, `cbba_allocator.py`) contain **no `rclpy` imports**
- [ ] No new secrets, credentials, or PX4 SITL binaries committed
- [ ] PR title is 70 characters or fewer
- [ ] PR description includes a Summary (what changed and why) and a Test Plan (how you verified it)

---

## 6. Architecture

ARGOS uses a **4-layer architecture**. Understanding this before contributing is essential.

```
┌─────────────────────────────────────────┐
│  Layer 1: Orchestrator                  │  임무 할당, 전략 판단, CBBA 경매
│  (orchestrator_node.py)                 │  LifecycleNode, SROS2 지원
├─────────────────────────────────────────┤
│  Layer 2: Mission Modules               │  Frontier 탐색, 시나리오 실행
│  (frontier_explorer, scenario_runner)   │  교체 가능한 미션 단위
├─────────────────────────────────────────┤
│  Layer 3: Core Services                 │  센서 퓨전, 로봇 디스패치
│  (sensor_fusion.py, robot_dispatcher.py)│  rclpy 미의존 순수 로직
│  (kalman_fire_tracker, keepout_manager) │  단위 테스트 100% 가능
├─────────────────────────────────────────┤
│  Layer 4: Platform                      │  UGV / Drone / Sherpa 각 구현
│  (ugv_platform.py, px4_platform.py,     │  동일 PlatformInterface 상향 제공
│   sherpa_platform.py)                   │
└─────────────────────────────────────────┘
```

**Key design rule**: The Orchestrator does not know the robot type. It queries `capabilities` only (e.g., `has_thermal`, `can_fly`). Platform-specific code lives exclusively in Layer 4.

**Logic modules are ROS-free**: `sensor_fusion.py` and `robot_dispatcher.py` must not import `rclpy`. This enables fast, dependency-free unit testing.

---

## 7. License

| Package | License | Notes |
|---------|---------|-------|
| `argos_bringup` | Apache 2.0 | 모든 핵심 노드 포함 |
| `argos_interfaces` | Apache 2.0 | 커스텀 메시지/서비스/액션 |
| `argos_description` | Apache 2.0 | URDF/xacro 모델 |
| `argos_fire_ai` | AGPL-3.0 | YOLOv8 기반 화재 탐지 (AI-Hub 데이터) |

**Do not mix Apache and AGPL code.** `argos_fire_ai` is a separate ROS 2 package and must remain isolated. If your contribution touches fire detection inference, it belongs in `argos_fire_ai`, not `argos_bringup`.

When adding a third-party dependency, confirm its license is compatible with Apache 2.0 and note it in the `LICENSE` file.

---

## 8. Issue Templates

### Bug Report

```
**Environment**
- OS: Ubuntu 24.04 / WSL2
- ROS 2: Jazzy
- ARGOS version / commit SHA:

**Steps to reproduce**
1.
2.
3.

**Expected behavior**

**Actual behavior**

**Relevant logs**
(paste ros2 launch output or pytest output)
```

### Feature Request

```
**Problem to solve**
(What firefighting scenario or operational need does this address?)

**Proposed solution**
(Node / topic / parameter changes)

**Architecture layer affected**
[ ] Orchestrator  [ ] Mission  [ ] Core  [ ] Platform  [ ] Interfaces

**Alternatives considered**

**References**
(Papers, open-source projects, field reports)
```

---

Questions? Open a GitHub Discussion or refer to `docs/troubleshooting/` for common issues.
