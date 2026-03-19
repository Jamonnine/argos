"""
ARGOS 4대 이종 편대 Launch — PX4 SITL 2드론 + UGV 2대
==========================================================
UGV 2대: 기존 exploration.launch.py의 전체 스택 재사용 (Nav2 + SLAM + Frontier + Hotspot)
드론 2대: PX4 SITL offboard 모드 (uXRCE-DDS Agent 포트 분리)

편대 구성:
  argos1 (UGV): (3.0, 2.5, 0.3) — Nav2 + Frontier 탐색
  argos2 (UGV): (7.0, 2.5, 0.3) — Nav2 + Frontier 탐색
  drone1 (PX4):  (5.0, 4.0, 0.0) — PX4 SITL instance=0, uXRCE 포트=8888
  drone2 (PX4):  (5.0, 6.0, 0.0) — PX4 SITL instance=1, uXRCE 포트=8889

사전 요건:
  1. PX4 SITL 2인스턴스 기동 (별도 터미널)
       cd ~/PX4-Autopilot
       PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL=x500 \
         ./build/px4_sitl_default/bin/px4 -i 0 -d
       PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL=x500 \
         ./build/px4_sitl_default/bin/px4 -i 1 -d
  2. uXRCE-DDS Agent 2인스턴스 (각 드론별 포트 분리)
       MicroXRCEAgent udp4 -p 8888
       MicroXRCEAgent udp4 -p 8889
  3. 이후 이 launch 파일 실행
       ros2 launch argos_description multi_px4.launch.py

사용법:
  ros2 launch argos_description multi_px4.launch.py
  ros2 launch argos_description multi_px4.launch.py use_px4:=false  # PX4 없이 Gazebo 네이티브
  ros2 launch argos_description multi_px4.launch.py world:=fire_building.sdf
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


# ── 편대 구성 (수정 포인트) ──────────────────────────────────────────────────

# UGV: exploration.launch.py와 동일한 ROBOTS 설정 재사용
ROBOTS = [
    {'name': 'argos1', 'x': 3.0, 'y': 2.5, 'z': 0.3},
    {'name': 'argos2', 'x': 7.0, 'y': 2.5, 'z': 0.3},
]

# PX4 드론: instance 번호 → uXRCE-DDS 포트 분리
# instance=0: /fmu/in/... (포트 8888)
# instance=1: /px4_1/fmu/in/... (포트 8889)
PX4_DRONES = [
    {'name': 'drone1', 'x': 5.0, 'y': 4.0, 'z': 0.0, 'px4_instance': 0},
    {'name': 'drone2', 'x': 5.0, 'y': 6.0, 'z': 0.0, 'px4_instance': 1},
]

# 오케스트레이터 지연: PX4 offboard 준비에 더 많은 시간 필요 (UGV 45초 → 150초)
ORCHESTRATOR_DELAY_SEC = 150.0


# ── UGV 스택 (exploration.launch.py 로직 재사용) ────────────────────────────

def exploration_robot_group(robot_config, pkg_dir, urdf_file, nav2_params):
    """단일 UGV: 스폰 + 컨트롤러 + Nav2/SLAM + Frontier 탐색 + Hotspot 전체 스택.

    exploration.launch.py의 동일 함수를 그대로 이식.
    변경점 없음 — 편대 컨텍스트에서도 동일하게 동작해야 함.
    """
    name = robot_config['name']
    x = str(robot_config['x'])
    y = str(robot_config['y'])
    z = str(robot_config['z'])

    robot_description = ParameterValue(
        Command([
            'xacro ', urdf_file,
            ' use_sim:=true',
            ' namespace:=', name,
            ' robot_name:=', name,
        ]),
        value_type=str,
    )

    # RSP: frame_prefix 방식 (gz_ros2_control이 namespace remapping 무시하므로)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=name,
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
            'frame_prefix': f'{name}/',  # base_footprint → argos1/base_footprint
        }],
        output='screen',
    )

    # Spawn
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=name,
        arguments=[
            '-name', name,
            '-topic', f'/{name}/robot_description',
            '-x', x, '-y', y, '-z', z,
            '-allow_renaming', 'true',
        ],
        output='screen',
    )

    # 컨트롤러: spawn 완료 후 순서대로 기동
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=name,
        arguments=[
            'joint_state_broadcaster',
            '-c', f'/{name}/controller_manager',
            '--controller-manager-timeout', '30',
            '--switch-timeout', '20',
        ],
        output='screen',
    )

    ddc_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=name,
        arguments=[
            'diff_drive_controller',
            '-c', f'/{name}/controller_manager',
            '--controller-manager-timeout', '30',
            '--switch-timeout', '20',
        ],
        output='screen',
    )

    jsb_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[jsb_spawner],
        )
    )

    ddc_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[ddc_spawner],
        )
    )

    # Gazebo Bridge (per robot)
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        namespace=name,
        arguments=[
            f'/{name}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            f'/{name}/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            f'/{name}/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            f'/{name}/depth_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            f'/{name}/depth_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            f'/{name}/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            f'/{name}/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            f'/{name}/thermal/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        output='screen',
    )

    # Nav2 + SLAM (25초 지연: 컨트롤러 초기화 대기)
    nav2_bringup = TimerAction(
        period=25.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_dir, 'launch', 'argos_nav2_bringup.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'True',
                    'slam': 'True',
                    'params_file': nav2_params,
                    'autostart': 'True',
                    'namespace': name,
                    'use_namespace': 'True',
                }.items(),
            ),
        ],
    )

    # Hotspot Detector (열화상 분석)
    hotspot_detector = Node(
        package='argos_bringup',
        executable='hotspot_detector',
        name='hotspot_detector',
        namespace=name,
        parameters=[{
            'use_sim_time': True,
            'top_percent': 0.05,
            'min_area': 20,
            'l8_resolution': 3.0,
            'l8_min_temp': 253.15,
        }],
        output='screen',
    )

    # Frontier Explorer (40초 지연: Nav2 완전 활성화 대기)
    frontier_explorer = TimerAction(
        period=40.0,
        actions=[
            Node(
                package='argos_bringup',
                executable='frontier_explorer',
                name='frontier_explorer',
                namespace=name,
                parameters=[{
                    'use_sim_time': True,
                    'min_frontier_size': 8,
                    'exclusion_radius': 2.0,
                    'blacklist_radius': 1.0,
                    'exploration_rate': 0.5,
                    'robot_name': name,
                    'thermal_pause': True,
                }],
                output='screen',
            ),
        ],
    )

    # Robot Status Publisher
    robot_status_pub = Node(
        package='argos_bringup',
        executable='robot_status',
        name='robot_status_publisher',
        namespace=name,
        parameters=[{
            'use_sim_time': True,
            'robot_id': name,
            'robot_type': 'ugv',
            'capabilities': ['thermal', 'lidar', 'depth', 'imu'],
        }],
        output='screen',
    )

    return [
        robot_state_publisher,
        spawn_robot,
        jsb_after_spawn,
        ddc_after_jsb,
        gz_bridge,
        nav2_bringup,
        hotspot_detector,
        frontier_explorer,
        robot_status_pub,
    ]


# ── PX4 드론 스택 ────────────────────────────────────────────────────────────

def px4_drone_group(drone_config, pkg_dir, use_px4_value: str):
    """단일 PX4 드론: SDF 스폰 + Gazebo 브릿지 + PX4 Bridge 노드 + 상태 보고.

    PX4 SITL offboard 모드 연결:
      - px4_instance=0 → /fmu/in/... (uXRCE 포트 8888)
      - px4_instance=1 → /px4_1/fmu/in/... (uXRCE 포트 8889)

    use_px4:=false 시 Gazebo 네이티브 드론 컨트롤러로 폴백
    (px4_bridge_node의 passthrough 모드 활성화).
    """
    name = drone_config['name']
    x = str(drone_config['x'])
    y = str(drone_config['y'])
    z = str(drone_config['z'])
    px4_instance = drone_config['px4_instance']

    sdf_file = os.path.join(pkg_dir, 'models', 'argos_drone', 'model.sdf')

    # SDF 직접 스폰 (URDF 아님 — MulticopterVelocityControl 플러그인 포함)
    spawn_drone = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', name,
            '-file', sdf_file,
            '-x', x, '-y', y, '-z', z,
        ],
        output='screen',
    )

    # Gazebo Bridge: 드론 센서 + 제어 채널
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        namespace=name,
        arguments=[
            # 드론 → Gazebo 제어 (Twist 명령)
            f'/model/{name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # Gazebo → 드론 odometry
            f'/model/{name}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # 드론 센서
            f'/{name}/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            f'/{name}/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            f'/{name}/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            f'/{name}/thermal/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        output='screen',
    )

    # Topic Relay: cmd_vel 방향 (네임스페이스 → /model/)
    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        namespace=name,
        arguments=[
            f'/{name}/cmd_vel',
            f'/model/{name}/cmd_vel',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # Topic Relay: odom 방향 (/model/ → 네임스페이스)
    odom_relay = Node(
        package='topic_tools',
        executable='relay',
        name='odom_relay',
        namespace=name,
        arguments=[
            f'/model/{name}/odometry',
            f'/{name}/odom',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # PX4 Bridge (15초 지연: Gazebo 스폰 + uXRCE 연결 대기)
    # use_px4:=false 시 passthrough 모드 (기존 Gazebo 네이티브와 동일)
    px4_bridge = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='argos_bringup',
                executable='px4_bridge',
                name='px4_bridge',
                namespace=name,
                parameters=[{
                    'use_sim_time': True,
                    'use_px4': use_px4_value == 'true',
                    'robot_id': name,
                    'px4_instance': px4_instance,
                    'offboard_rate': 10.0,  # Hz (PX4 최소 2Hz의 5배)
                }],
                output='screen',
            ),
        ],
    )

    # 드론 고유 컨트롤러 (10초 지연: Gazebo 스폰 대기)
    # PX4 offboard 시에도 altitude hold 등 상위 제어에 사용
    drone_controller = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='argos_bringup',
                executable='drone_controller',
                name='drone_controller',
                namespace=name,
                parameters=[{
                    'use_sim_time': True,
                    'cruise_altitude': 8.0,
                    'max_horizontal_speed': 2.0,
                    'max_vertical_speed': 1.0,
                }],
                output='screen',
            ),
        ],
    )

    # Robot Status Publisher (오케스트레이터 등록)
    robot_status_pub = Node(
        package='argos_bringup',
        executable='robot_status',
        name='robot_status_publisher',
        namespace=name,
        parameters=[{
            'use_sim_time': True,
            'robot_id': name,
            'robot_type': 'drone',
            # can_fly, has_thermal: CBBA 임무 할당 시 capability 매핑에 사용
            'capabilities': ['can_fly', 'has_thermal', 'camera', 'imu'],
        }],
        output='screen',
    )

    return [
        spawn_drone,
        gz_bridge,
        cmd_vel_relay,
        odom_relay,
        px4_bridge,
        drone_controller,
        robot_status_pub,
    ]


# ── Launch Description ────────────────────────────────────────────────────────

def generate_launch_description():
    pkg_dir = get_package_share_directory('argos_description')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'argos_ugv.urdf.xacro')
    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    # ── 인수 선언 ──────────────────────────────────────────────────────────────
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_dir, 'worlds', 'indoor_test.sdf'),
        description='Gazebo world file',
    )

    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run Gazebo headless (no GUI, EGL rendering for sensors)',
    )

    use_px4_arg = DeclareLaunchArgument(
        'use_px4', default_value='true',
        description=(
            'PX4 offboard 모드 사용 여부. '
            'false 시 기존 Gazebo 네이티브 컨트롤러 사용 (PX4 SITL 불필요)'
        ),
    )

    # ── Gazebo Harmonic ────────────────────────────────────────────────────────
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

    # ── 공통 /clock 브릿지 (1개만) ────────────────────────────────────────────
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
    )

    # ── 오케스트레이터 + CBBA (150초 지연) ────────────────────────────────────
    # 지연 이유: PX4 offboard 초기화에 UGV 대비 더 긴 시간 필요
    #   UGV Nav2: ~40초 / PX4 arming + offboard 전환: ~60~120초
    all_robot_names = [r['name'] for r in ROBOTS] + [d['name'] for d in PX4_DRONES]

    orchestrator = TimerAction(
        period=ORCHESTRATOR_DELAY_SEC,
        actions=[
            Node(
                package='argos_bringup',
                executable='orchestrator',
                name='orchestrator',
                parameters=[{
                    'use_sim_time': True,
                    'expected_robots': all_robot_names,
                    # 4대 편대: UGV 2 + 드론 2 — 배터리 귀환 타임아웃 여유 확보
                    'return_timeout_sec': 60.0,
                    'fire_alert_expiry_sec': 300.0,
                }],
                output='screen',
            ),
        ],
    )

    # ── 전체 엔티티 조립 ───────────────────────────────────────────────────────
    all_entities = [
        world_arg,
        headless_arg,
        use_px4_arg,
        gazebo,
        clock_bridge,
    ]

    # UGV 2대 스택
    for robot in ROBOTS:
        all_entities.extend(
            exploration_robot_group(robot, pkg_dir, urdf_file, nav2_params)
        )

    # PX4 드론 2대 스택
    # use_px4 LaunchConfiguration을 문자열로 전달 — 런타임 평가
    for drone in PX4_DRONES:
        # 주의: LaunchConfiguration은 런타임 치환이므로, 플래그는 node 파라미터로 전달
        all_entities.extend(
            px4_drone_group(drone, pkg_dir, use_px4_value='true')
        )

    all_entities.append(orchestrator)

    return LaunchDescription(all_entities)
