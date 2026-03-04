"""
ARGOS UGV Navigation Launch (Gazebo + Nav2 + SLAM + Thermal + Exploration)
==========================================================================
하나의 launch로 전체 자율 탐색 스택 실행:
  Gazebo(월드+로봇) → 컨트롤러 → SLAM → Nav2 → 화점 감지 → 프론티어 탐색

실행 순서 (이벤트 체이닝):
  1. Gazebo 시뮬레이션 + robot_state_publisher
  2. 로봇 스폰 (create)
  3. 스폰 완료 → joint_state_broadcaster 로드
  4. JSB 완료 → diff_drive_controller 로드
  5. 토픽 릴레이 (cmd_vel / odom 라우팅)
  6. Nav2 bringup (slam=True) — Gazebo /clock 이후 시작
  7. Hotspot detector — 열화상 화점 감지
  8. Frontier explorer — 자율 탐색 (explore:=true 시)

사용법:
  ros2 launch argos_description navigation.launch.py
  ros2 launch argos_description navigation.launch.py explore:=true
  ros2 launch argos_description navigation.launch.py world:=empty.sdf
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('argos_description')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    urdf_file = os.path.join(pkg_dir, 'urdf', 'argos_ugv.urdf.xacro')
    bridge_config = os.path.join(pkg_dir, 'config', 'gz_bridge.yaml')
    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    # --- Launch 인자 ---
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_dir, 'worlds', 'indoor_test.sdf'),
        description='Gazebo world file'
    )

    explore_arg = DeclareLaunchArgument(
        'explore', default_value='false',
        description='Enable autonomous frontier exploration'
    )

    # --- URDF ---
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file, ' use_sim:=true']),
        value_type=str
    )

    # --- 1. Robot State Publisher ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # --- 2. Gazebo Harmonic ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ]),
        launch_arguments={
            'gz_args': ['-r ', LaunchConfiguration('world')],
        }.items(),
    )

    # --- 3. 로봇 스폰 ---
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'argos_ugv',
            '-topic', '/robot_description',
            '-x', '5.0',    # 방 C 중앙
            '-y', '2.5',
            '-z', '0.3',
        ],
        output='screen',
    )

    # --- 4. Gazebo ↔ ROS 2 브리지 ---
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p',
                   f'config_file:={bridge_config}'],
        output='screen',
    )

    # --- 5. 컨트롤러 (이벤트 체이닝) ---
    load_jsb = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen',
    )

    load_ddc = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_drive_controller'],
        output='screen',
    )

    # 스폰 완료 → JSB 로드
    jsb_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[load_jsb],
        )
    )

    # JSB 완료 → DDC 로드
    ddc_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_jsb,
            on_exit=[load_ddc],
        )
    )

    # --- 6. 토픽 릴레이 (diff_drive_controller 네임스페이스 ↔ 표준 토픽) ---
    # diff_drive_controller는 /diff_drive_controller/cmd_vel을 구독하지만,
    # Nav2는 /cmd_vel로 퍼블리시 → 릴레이 필수.
    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        arguments=['/cmd_vel', '/diff_drive_controller/cmd_vel'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # diff_drive_controller의 /diff_drive_controller/odom → 표준 /odom
    odom_relay = Node(
        package='topic_tools',
        executable='relay',
        name='odom_relay',
        arguments=['/diff_drive_controller/odom', '/odom'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # --- 7. Nav2 + SLAM ---
    # Gazebo가 /clock 퍼블리시를 시작한 뒤에 Nav2를 기동해야 함.
    # TimerAction으로 5초 지연 (Gazebo 초기화 대기).
    nav2_bringup = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'True',
                    'slam': 'True',
                    'params_file': nav2_params,
                    'autostart': 'True',
                    'use_composition': 'False',
                }.items(),
            ),
        ],
    )

    # --- 8. 열화상 화점 감지 ---
    hotspot_detector = Node(
        package='my_robot_bringup',
        executable='hotspot_detector',
        name='hotspot_detector',
        parameters=[{
            'use_sim_time': True,
            'top_percent': 0.05,
            'min_area': 20,
            'l8_resolution': 3.0,
            'l8_min_temp': 253.15,
        }],
        output='screen',
    )

    # --- 9. 프론티어 자율 탐색 (explore:=true 시 활성화) ---
    frontier_explorer = Node(
        package='my_robot_bringup',
        executable='frontier_explorer',
        name='frontier_explorer',
        parameters=[{
            'use_sim_time': True,
            'min_frontier_size': 8,
            'exclusion_radius': 2.0,
            'blacklist_radius': 1.0,
            'exploration_rate': 1.0,
            'robot_name': '',
            'thermal_pause': True,
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('explore')),
    )

    return LaunchDescription([
        world_arg,
        explore_arg,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        gz_bridge,
        jsb_after_spawn,
        ddc_after_jsb,
        cmd_vel_relay,
        odom_relay,
        nav2_bringup,
        hotspot_detector,
        frontier_explorer,
    ])
