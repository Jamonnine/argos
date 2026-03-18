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
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('argos_description')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'argos_ugv.urdf.xacro')
    bridge_config = os.path.join(pkg_dir, 'config', 'gz_bridge.yaml')
    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    # FastDDS SHM 비활성화 파일: config/fastdds_no_shm.xml
    # WSL2에서 필요 시 Nav2 전용으로 적용 (전역 적용 시 Gazebo 내부 통신 차단)
    # 사용법: FASTRTPS_DEFAULT_PROFILES_FILE=... ros2 launch ...

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

    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run Gazebo headless (no GUI, EGL rendering for sensors)'
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
    gz_args = PythonExpression([
        "'-s --headless-rendering -r ' if '",
        LaunchConfiguration('headless'),
        "' == 'true' else '-r '",
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ]),
        launch_arguments={
            'gz_args': [gz_args, LaunchConfiguration('world')],
        }.items(),
    )

    # --- 3. 로봇 스폰 ---
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'argos_ugv',
            '-topic', '/robot_description',
            '-x', '2.0',    # 방 C 좌측 (table_c 회피: 5.0,2.5)
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

    # --- 5. 컨트롤러 (Node spawner — 멀티로봇 확장 대응) ---
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    ddc_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='screen',
    )

    # 스폰 완료 → JSB 로드
    jsb_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[jsb_spawner],
        )
    )

    # JSB 완료 → DDC 로드
    ddc_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[ddc_spawner],
        )
    )

    # cmd_vel/odom 릴레이 제거: ros2_control.urdf.xacro의 remapping으로 직접 연결

    # --- 6.5. scan_frame_relay — lidar_link TF 조회 실패 우회 ---
    # 문제: Nav2 전체 스택 기동 시 slam_toolbox MessageFilter가
    #   /scan(frame_id=lidar_link)의 TF 체인(odom→base_footprint→lidar_link)을
    #   조회할 때 DDC odom TF 발행 지연으로 "queue is full" 반복 발생.
    # 해결: /scan → /scan_base (frame_id=base_footprint)로 변환.
    #   slam_toolbox는 base_footprint만 조회 → DDC가 직접 발행하여 안정적.
    scan_relay = Node(
        package='argos_bringup',
        executable='scan_frame_relay',
        name='scan_frame_relay',
        parameters=[{
            'use_sim_time': True,
            'input_topic': 'scan',
            'output_topic': 'scan_base',
            'target_frame': 'base_footprint',
        }],
        output='screen',
    )

    # --- 7. Nav2 + SLAM ---
    # DDC 활성화 완료 후 Nav2 기동 (odom TF 발행이 시작된 뒤 시작해야 함).
    # 이전: TimerAction(5.0) — wall time 기반이라 RTF 0.28x에서 DDC보다 먼저 시작됨.
    # 수정: DDC 완료 이벤트에 체이닝 + 추가 5초 대기 (DDS 발견 시간 확보).
    nav2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'argos_nav2_bringup.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'slam': 'True',
            'params_file': nav2_params,
            'autostart': 'True',
        }.items(),
    )
    # DDC 완료 후 처리:
    #   1) scan_frame_relay 즉시 기동 (scan 릴레이를 Nav2 기동 전에 준비)
    #   2) 45초 후 Nav2 기동 (RTF 0.28x 기준 odom TF 축적 대기)
    #   3) 90초 후 frontier_explorer 기동 (Nav2 lifecycle 완전 활성화 대기)
    #
    # 타이밍 계산 (모두 DDC spawner 완료 시점 기준 wall time):
    #   DDC 완료:            T+0s   (launch 시작 후 ~20s)
    #   Nav2 기동:           T+45s  (odom TF 축적 대기)
    #   bt_navigator active: T+75s  (Nav2 lifecycle 활성화 ~30s 소요)
    #   frontier 시작:       T+90s  (bt_navigator active 후 15s 여유)
    #
    # 이전 버그: frontier_explorer를 LaunchDescription 최상위에 TimerAction(90s)으로
    #   등록 → launch 프로세스 시작 기준 90초 카운트 (DDC 완료 기준 아님).
    #   DDC가 ~20초에 완료되면 frontier는 DDC+70s에 시작 → bt_navigator(DDC+75s)보다
    #   5초 빨리 시작될 수 있음 → nav_server_ready=False 영구화.
    # 수정: frontier를 nav2_after_ddc 이벤트 체인 내부로 이동 (DDC 기준 90s 보장).
    frontier_node = Node(
        package='argos_bringup',
        executable='frontier_explorer',
        name='frontier_explorer',
        parameters=[{
            'use_sim_time': True,
            'min_frontier_size': 3,  # 초기 맵이 작아 8이면 문 감지 못함
            'exclusion_radius': 2.0,
            'blacklist_radius': 1.0,
            'exploration_rate': 0.5,
            'robot_name': '',
            'thermal_pause': True,
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('explore')),
    )

    nav2_after_ddc = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=ddc_spawner,
            on_exit=[
                scan_relay,
                TimerAction(period=45.0, actions=[nav2_include]),
                TimerAction(period=90.0, actions=[frontier_node]),
            ],
        )
    )

    # --- 8. 열화상 화점 감지 ---
    hotspot_detector = Node(
        package='argos_bringup',
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

    return LaunchDescription([
        world_arg,
        explore_arg,
        headless_arg,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        gz_bridge,
        jsb_after_spawn,
        ddc_after_jsb,
        nav2_after_ddc,
        hotspot_detector,
    ])
