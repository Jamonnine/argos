# ARGOS

> **A**utonomous **R**obot **G**roup **O**rchestration **S**ystem

이종 군집 소방 로봇 오케스트레이션 플랫폼.
드론, 차량형(UGV), 보행형 로봇이 한 팀으로 화재 현장을 탐색하고 정보를 공유하는 시스템.

## Status

| Milestone | Description | Status |
|-----------|-------------|--------|
| MS-1 | PatrolArea Action Server/Client | Done |
| MS-2 | ARGOS UGV URDF (xacro modular) | Done |
| MS-3 | Nav2 + SLAM integration | Done |
| MS-4 | Thermal camera simulation | Done |
| MS-5 | Multi-robot namespace separation | Done |
| MS-6 | Distributed exploration | Done |
| MS-7 | Orchestrator node | Done |
| MS-8 | Drone platform (heterogeneous) | Done |
| MS-9 | Integration demo scenario | Done |
| MS-10 | Web UI (rosbridge) | Done |

## Architecture

```
Orchestrator (Command & Control)
    |
Mission Modules (Recon / Suppression / Rescue / Hazmat)
    |
Core Services (Localization / Navigation / Swarm Comm / Health / Shared Awareness)
    |
Platform (Drone | UGV | Legged) — same interface upward
```

## Tech Stack

- **ROS 2 Jazzy** on Ubuntu 24.04 (WSL2)
- **Gazebo Harmonic** + ros_gz bridge
- **ros2_control** + diff_drive_controller (4WD skid-steer)
- **Nav2** + slam_toolbox
- **rosbridge_server** + roslibjs (web dashboard)
- **Zenoh** (planned, for degraded WiFi environments)

## Packages

| Package | Description |
|---------|-------------|
| `argos_description` | URDF/xacro robot model (3-layer modular: platform/sensors/control) |
| `my_robot_bringup` | Nodes, launch files, scripts |
| `my_robot_interfaces` | Custom .msg/.srv/.action definitions |

## Quick Start

```bash
# Build
cd ~/ros2_ws && colcon build

# Visualize robot model
ros2 launch argos_description display.launch.py

# Simulate in Gazebo
ros2 launch argos_description gazebo.launch.py

# Navigation + SLAM + Thermal detection
ros2 launch argos_description navigation.launch.py

# Multi-robot simulation (2 UGVs)
ros2 launch argos_description multi_robot.launch.py

# Autonomous frontier exploration (single robot)
ros2 launch argos_description navigation.launch.py explore:=true

# Heterogeneous autonomous exploration (2 UGVs + 1 Drone)
ros2 launch argos_description exploration.launch.py

# Full fire response demo (auto scenario)
ros2 launch argos_description demo.launch.py

# Web monitoring dashboard (requires: sudo apt install ros-jazzy-rosbridge-server)
ros2 launch argos_description monitor.launch.py
# Then: cd $(ros2 pkg prefix argos_description)/share/argos_description/web
#       python3 -m http.server 8080
#       Open http://localhost:8080

# Test patrol action
ros2 run my_robot_bringup patrol_server &
ros2 run my_robot_bringup patrol_client
```

## References

- DARPA SubT Challenge architectures (CERBERUS, CSIRO)
- Open-RMF (heterogeneous fleet management)
- Linorobot2, Husarion Panther (UGV URDF design)
- Hyundai HR-Sherpa (unmanned firefighting robot)

## License

Apache-2.0
