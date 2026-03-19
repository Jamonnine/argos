"""
ARGOS Navigation Launch — Zenoh RMW 래퍼 (Phase C-1).
======================================================
기존 navigation.launch.py를 Zenoh RMW 환경으로 실행하는 래퍼.

Zenoh 선택 이유:
  - 소방 현장 무선망(NAT, 멀티캐스트 불가) 투과성 우수
  - gossip 라우팅으로 노드 자동 검색 (DDS multicast 불필요)
  - 단일 TCP 세션으로 트래픽 집약 → 무선 대역폭 절약

전제 조건:
  sudo apt install ros-jazzy-rmw-zenoh-cpp
  # zenoh 라우터 실행 (선택): ros2 run rmw_zenoh_cpp rmw_zenohd

사용법:
  # 기본 (Zenoh peer 모드)
  ros2 launch argos_description navigation_zenoh.launch.py

  # headless SIL 테스트
  ros2 launch argos_description navigation_zenoh.launch.py headless:=true explore:=true

  # 사용자 지정 Zenoh 설정 파일
  ros2 launch argos_description navigation_zenoh.launch.py \\
      zenoh_config:=/path/to/zenoh_config.json5

  # 기존 navigation.launch.py와 동일한 인자 모두 전달 가능
  ros2 launch argos_description navigation_zenoh.launch.py \\
      world:=fire_building.sdf explore:=true headless:=true
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('argos_description')

    # 기본 Zenoh 설정 파일: argos_description/config/zenoh_config.json5
    default_zenoh_config = os.path.join(pkg_dir, 'config', 'zenoh_config.json5')

    # ── Launch 인자 ──

    # Zenoh 전용 인자
    zenoh_config_arg = DeclareLaunchArgument(
        'zenoh_config',
        default_value=default_zenoh_config,
        description='Zenoh 설정 파일 경로 (.json5). '
                    '기본값: argos_description/config/zenoh_config.json5'
    )

    # navigation.launch.py 통과 인자 (동일한 default 유지)
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_dir, 'worlds', 'indoor_test.sdf'),
        description='Gazebo world file (navigation.launch.py로 전달)'
    )

    explore_arg = DeclareLaunchArgument(
        'explore',
        default_value='false',
        description='Enable autonomous frontier exploration (navigation.launch.py로 전달)'
    )

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo headless (navigation.launch.py로 전달)'
    )

    # ── Zenoh RMW 환경 변수 설정 ──

    # RMW_IMPLEMENTATION: rmw_zenoh_cpp 강제 지정
    set_rmw = SetEnvironmentVariable(
        name='RMW_IMPLEMENTATION',
        value='rmw_zenoh_cpp',
    )

    # ZENOH_CONFIG_FILE: rmw_zenoh_cpp가 참조하는 설정 파일 경로
    set_zenoh_config = SetEnvironmentVariable(
        name='ZENOH_CONFIG_FILE',
        value=LaunchConfiguration('zenoh_config'),
    )

    # ── 기존 navigation.launch.py 포함 ──
    # 모든 launch_arguments를 그대로 전달하여 기능 동등성 보장.
    # RMW_IMPLEMENTATION 환경 변수는 SetEnvironmentVariable로 이미 적용됨.
    navigation_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'explore': LaunchConfiguration('explore'),
            'headless': LaunchConfiguration('headless'),
        }.items(),
    )

    # Zenoh 라우터 데몬 (피어간 통신에 필수)
    zenoh_router = ExecuteProcess(
        cmd=['ros2', 'run', 'rmw_zenoh_cpp', 'rmw_zenohd'],
        output='screen',
    )

    return LaunchDescription([
        # 인자 선언
        zenoh_config_arg,
        world_arg,
        explore_arg,
        headless_arg,
        # Zenoh 환경 설정
        set_rmw,
        set_zenoh_config,
        # Zenoh 라우터 먼저 시작
        zenoh_router,
        # 라우터 초기화 3초 대기 후 navigation 스택 기동
        TimerAction(period=3.0, actions=[navigation_include]),
    ])
