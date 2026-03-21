"""
셰르파 단일 로봇 Gazebo 시뮬레이션 Launch
==========================================
셰르파 UGV 1대를 Gazebo에 스폰하고 컨트롤러까지 기동.
DDC spawner 완료 후 cmd_vel로 이동 가능한 상태가 된다.

실행 순서 (이벤트 체이닝):
  1. Gazebo 시뮬레이션 + robot_state_publisher
  2. 로봇 스폰 (create, z=1.0 — 낙하 안착)
  3. 스폰 완료 → joint_state_broadcaster 로드
  4. JSB 완료 → diff_drive_controller 로드
  5. DDC 완료 → cmd_vel 수신 가능 상태
  6. (선택) nav:=true 시 30초 후 Nav2 기동

사용법:
  ros2 launch argos_description sherpa_sim.launch.py
  ros2 launch argos_description sherpa_sim.launch.py headless:=true nav:=true
  ros2 launch argos_description sherpa_sim.launch.py world:=/path/to/sherpa_fire_test.sdf
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

    # 셰르파 전용 URDF / 컨트롤러 YAML
    urdf_file = os.path.join(pkg_dir, 'urdf', 'sherpa', 'sherpa.urdf.xacro')
    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    # --- Launch 인자 ---
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_dir, 'worlds', 'indoor_test.sdf'),
        description='Gazebo world file (기본: indoor_test.sdf)',
    )

    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run Gazebo headless (no GUI, EGL rendering for sensors)',
    )

    nav_arg = DeclareLaunchArgument(
        'nav', default_value='false',
        description='Nav2 활성화 여부 (기본: false)',
    )

    # --- URDF (namespace 파라미터 전달) ---
    # 단일 로봇이므로 namespace='' (글로벌 TF 사용, frame_prefix 없음)
    robot_description = ParameterValue(
        Command([
            'xacro ', urdf_file,
            ' use_sim:=true',
            ' namespace:=',  # 빈 문자열 → 글로벌 네임스페이스
        ]),
        value_type=str,
    )

    # --- 1. Robot State Publisher ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        output='screen',
    )

    # --- 2. Gazebo Harmonic ---
    # headless=true 시 -s --headless-rendering 추가 (GPU 없는 환경 대응)
    gz_args = PythonExpression([
        "'-s --headless-rendering -r ' if '",
        LaunchConfiguration('headless'),
        "' == 'true' else '-r '",
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py',
            )
        ]),
        launch_arguments={
            'gz_args': [gz_args, LaunchConfiguration('world')],
        }.items(),
    )

    # --- 3. 로봇 스폰 ---
    # z=1.0: 셰르파 높이 1.9m, 낙하 후 지면 안착
    # (multi_sherpa.launch.py의 z=0.45보다 높게 설정 — 단독 스폰 시 충돌 회피)
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'sherpa',
            '-topic', '/robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0',
        ],
        output='screen',
    )

    # --- 4. Gazebo ↔ ROS 2 브리지 ---
    # /clock 포함: 단일 로봇이므로 단일 브리지 노드에서 처리
    # scan, camera, thermal_swir, thermal_lwir, imu, /clock
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/thermal_swir/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/thermal_lwir/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        output='screen',
    )

    # --- 5. 컨트롤러 spawner (JSB → DDC 순차) ---
    # sherpa_controllers.yaml: 6WD diff_drive (front/mid/rear × left/right joint)
    # URDF의 ros2_control 플러그인이 sherpa_controllers.yaml을 로드함
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager-timeout', '30',
            '--switch-timeout', '20',
        ],
        output='screen',
    )

    ddc_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager-timeout', '30',
            '--switch-timeout', '20',
        ],
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
    # DDC 활성화 완료 시점 = cmd_vel 토픽 수신 가능 시점
    ddc_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[ddc_spawner],
        )
    )

    # --- 6. Nav2 (선택적, nav:=true 시) ---
    # DDC 완료 후 30초 대기 (odom TF 축적 + DDS 발견 시간 확보).
    # 단일 로봇 단순 테스트 환경이므로 navigation.launch.py(45s)보다 짧게 설정.
    # RTF가 낮은 환경에서 Nav2 기동이 늦으면 이 값을 45s 이상으로 늘릴 것.
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
        condition=IfCondition(LaunchConfiguration('nav')),
    )

    # DDC 완료 → 30초 대기 → Nav2 기동 (nav:=true 시에만)
    nav2_after_ddc = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=ddc_spawner,
            on_exit=[
                TimerAction(period=30.0, actions=[nav2_include]),
            ],
        )
    )

    return LaunchDescription([
        world_arg,
        headless_arg,
        nav_arg,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        gz_bridge,
        jsb_after_spawn,
        ddc_after_jsb,
        nav2_after_ddc,
    ])
