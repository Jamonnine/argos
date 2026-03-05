"""
ARGOS 통합 시연 Launch
=======================
UGV + 드론 이종 군집 탐색 + 소방 작전 시나리오 자동 실행.

exploration.launch.py의 전체 스택에 scenario_runner를 추가하여
화재 신고 → 드론 정찰 → UGV 탐색 → 화점 감지 → 긴급 정지 → 재개
→ 귀환의 완전 자동 시연을 수행.

사용법:
  ros2 launch argos_description demo.launch.py
  ros2 launch argos_description demo.launch.py simulate_fire:=false
  ros2 launch argos_description demo.launch.py fire_delay:=60.0
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
    AnyLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('argos_description')

    # --- Launch Arguments ---
    simulate_fire_arg = DeclareLaunchArgument(
        'simulate_fire',
        default_value='true',
        description='Simulate fire detection (bypass thermal pipeline)',
    )

    fire_delay_arg = DeclareLaunchArgument(
        'fire_delay',
        default_value='30.0',
        description='Seconds after exploration starts to simulate fire',
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_dir, 'worlds', 'indoor_test.sdf'),
        description='Gazebo world file',
    )

    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run Gazebo headless (no GUI, EGL rendering for sensors)',
    )

    # --- 전체 탐색 스택 (UGV + Drone + Orchestrator) ---
    exploration_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'exploration.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'headless': LaunchConfiguration('headless'),
        }.items(),
    )

    # --- 시나리오 러너 (25초 지연: 오케스트레이터 초기화 대기) ---
    scenario_runner = TimerAction(
        period=25.0,
        actions=[
            Node(
                package='my_robot_bringup',
                executable='scenario_runner',
                name='scenario_runner',
                parameters=[{
                    'use_sim_time': True,
                    'simulate_fire': LaunchConfiguration('simulate_fire'),
                    'fire_delay_sec': LaunchConfiguration('fire_delay'),
                    'drone_name': 'drone1',
                    'target_ugv': 'argos1',
                }],
                output='screen',
            ),
        ],
    )

    # --- rosbridge WebSocket (웹 대시보드 자동 연결) ---
    rosbridge_launch = PathJoinSubstitution([
        FindPackageShare('rosbridge_server'),
        'launch', 'rosbridge_websocket_launch.xml',
    ])

    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(rosbridge_launch),
        launch_arguments={'port': '9090'}.items(),
    )

    web_dir = PathJoinSubstitution([
        FindPackageShare('argos_description'), 'web',
    ])

    dashboard_info = LogInfo(msg=[
        '\n', '=' * 55, '\n',
        '  ARGOS Web Dashboard\n',
        '  Open: cd ', web_dir, ' && python3 -m http.server 8080\n',
        '  Then visit http://localhost:8080\n',
        '=' * 55,
    ])

    return LaunchDescription([
        simulate_fire_arg,
        fire_delay_arg,
        world_arg,
        headless_arg,
        exploration_stack,
        scenario_runner,
        rosbridge,
        dashboard_info,
    ])
