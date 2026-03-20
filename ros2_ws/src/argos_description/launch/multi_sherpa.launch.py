"""
ARGOS 멀티 셰르파 + 드론 편대 Launch
=====================================
HR-셰르파 UGV 2대 + 드론 1대를 Gazebo에 스폰하고,
각 셰르파에 독립적인 호스 제약 기반 진압 스택을 구성.
드론은 상공 정찰 담당, 오케스트레이터가 전체를 통합 지휘.

셰르파: Nav2 + SLAM + 프론티어 탐색 + 호스 상태 추적 + 방수포 + LiDAR 오탐 시뮬
드론:   웨이포인트 P 제어 + 하방 카메라 (MulticopterVelocityControl)

사용법:
  ros2 launch argos_description multi_sherpa.launch.py
  ros2 launch argos_description multi_sherpa.launch.py headless:=true
  ros2 launch argos_description multi_sherpa.launch.py world:=/path/to/custom.sdf
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


# --- 셰르파 2대 설정 ---
# 스폰 좌표: 실험장 진입구 양쪽 (z=0.45 — 셰르파 휠반경 0.35m + 지면간격 0.05m + 차체절반 0.5/2)
SHERPAS = [
    {'name': 'sherpa1', 'x': 3.0, 'y': -1.0, 'z': 0.45},
    {'name': 'sherpa2', 'x': 9.0, 'y': -1.0, 'z': 0.45},
]

# --- 드론 1대 설정 (exploration.launch.py 동일) ---
DRONES = [
    {'name': 'drone1', 'x': 5.0, 'y': 4.0, 'z': 0.2},
]


def generate_sherpa_actions(name, x, y, z, pkg_dir, urdf_file, nav2_params, sherpa_params):
    """단일 셰르파: 스폰 + 컨트롤러 + 셰르파 전용 노드 + Nav2/SLAM + 탐색 전체 스택.

    UGV(argos_ugv)와의 차이점:
      - sherpa_controllers.yaml 로드 (6WD joint 이름 포함)
      - hose_tether_node, water_curtain_node, lidar_degradation_node 추가
      - base_frame: {name}/base_footprint (frame_prefix 방식 통일)
    """
    robot_description = ParameterValue(
        Command([
            'xacro ', urdf_file,
            ' use_sim:=true',
            ' namespace:=', name,
            ' robot_name:=', name,
        ]),
        value_type=str,
    )

    # --- RSP ---
    # frame_prefix 방식: TF는 글로벌 /tf에 발행하되 frame_id로 로봇 구분
    # (gz_ros2_control이 namespace remapping을 무시하므로 /tf→tf remapping 제거)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=name,
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
            'frame_prefix': f'{name}/',  # base_footprint → sherpa1/base_footprint
        }],
        output='screen',
    )

    # --- Spawn ---
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=name,
        arguments=[
            '-name', name,
            '-topic', f'/{name}/robot_description',
            '-x', str(x), '-y', str(y), '-z', str(z),
            '-allow_renaming', 'true',
        ],
        output='screen',
    )

    # --- Controllers ---
    # 셰르파는 sherpa_controllers.yaml의 6WD joint 이름 사용
    # (front/mid/rear × left/right — diff_drive_controller 인터페이스 동일)
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

    # OnProcessExit 체이닝: spawn → JSB → DDC
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

    # --- Gazebo Bridge (per sherpa) ---
    # /clock 브릿지는 generate_launch_description에서 공통 처리 (1개만)
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

    # --- scan_frame_relay (per sherpa) — lidar_link TF 조회 실패 우회 ---
    # collision_monitor가 scan_base 토픽을 사용하므로 필수
    scan_relay = Node(
        package='argos_bringup',
        executable='scan_frame_relay',
        name='scan_frame_relay',
        namespace=name,
        parameters=[{
            'use_sim_time': True,
            'input_topic': 'scan',
            'output_topic': 'scan_base',
            'target_frame': 'base_footprint',  # scan_frame_relay가 namespace 자동 프리픽스
        }],
        output='screen',
    )

    # --- 셰르파 전용 노드 3종 (DDC 완료 직후 즉시 시작) ---
    # hose_tether_node: 호스 잔여 길이·꺾임 위험도·충수 상태 추적
    hose_tether = Node(
        package='argos_bringup',
        executable='hose_tether',
        name='hose_tether_node',
        namespace=name,
        parameters=[{
            'use_sim_time': True,
            'robot_id': name,
            'hose_total_length': 100.0,
            'hose_min_bend_radius': 0.5,
        }],
        output='screen',
    )

    # water_curtain_node: 분무·방수포 시뮬레이션
    water_curtain = Node(
        package='argos_bringup',
        executable='water_curtain',
        name='water_curtain_node',
        namespace=name,
        parameters=[{
            'use_sim_time': True,
            'robot_id': name,
        }],
        output='screen',
    )

    # lidar_degradation_node: 방수포 분사 시 LiDAR 신뢰도 저하 시뮬레이션
    lidar_degradation = Node(
        package='argos_bringup',
        executable='lidar_degradation',
        name='lidar_degradation_node',
        namespace=name,
        parameters=[{
            'use_sim_time': True,
            'robot_id': name,
        }],
        output='screen',
    )

    # --- Nav2 + SLAM (per sherpa) ---
    # DDC 완료 이벤트에 체이닝 + 45초 추가 대기 (odom TF 축적)
    # exploration.launch.py 동일 패턴: OnProcessExit(ddc_spawner) → scan_relay 즉시 + 45초 후 Nav2
    nav2_include = IncludeLaunchDescription(
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
    )
    nav2_after_ddc = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=ddc_spawner,
            on_exit=[
                scan_relay,
                hose_tether,
                water_curtain,
                lidar_degradation,
                TimerAction(period=45.0, actions=[nav2_include]),
            ],
        )
    )

    # --- Frontier Explorer (per sherpa) ---
    # Nav2 완전 활성화 대기: DDC 완료 + 45초(Nav2 시작) + 45초(lifecycle 활성화) = DDC + 90초
    frontier_node = Node(
        package='argos_bringup',
        executable='frontier_explorer',
        name='frontier_explorer',
        namespace=name,
        parameters=[{
            'use_sim_time': True,
            'min_frontier_size': 3,
            'exclusion_radius': 2.0,
            'blacklist_radius': 1.0,
            'exploration_rate': 0.5,
            'robot_name': name,
            'thermal_pause': True,
            # 셰르파 frame_prefix: TF 트리에서 sherpa1/base_footprint
            'base_frame': f'{name}/base_footprint',
            'map_frame': 'map',
        }],
        output='screen',
    )
    frontier_after_ddc = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=ddc_spawner,
            on_exit=[
                TimerAction(period=90.0, actions=[frontier_node]),
            ],
        )
    )

    # --- Hotspot Detector (per sherpa) ---
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

    # --- Robot Status Publisher (오케스트레이터에 상태 보고) ---
    robot_status_pub = Node(
        package='argos_bringup',
        executable='robot_status',
        name='robot_status_publisher',
        namespace=name,
        parameters=[{
            'use_sim_time': True,
            'robot_id': name,
            'robot_type': 'sherpa',
            'capabilities': ['thermal', 'lidar', 'depth', 'imu', 'hose', 'water_curtain'],
        }],
        output='screen',
    )

    return [
        robot_state_publisher,
        spawn_robot,
        jsb_after_spawn,
        ddc_after_jsb,
        gz_bridge,
        nav2_after_ddc,
        frontier_after_ddc,
        hotspot_detector,
        robot_status_pub,
    ]


def generate_drone_actions(drone_config, pkg_dir):
    """단일 드론: SDF 모델 스폰 + 브릿지 + P 제어기 + 상태 보고.

    exploration.launch.py의 drone_group과 동일 구현.
    """
    name = drone_config['name']
    x = str(drone_config['x'])
    y = str(drone_config['y'])
    z = str(drone_config['z'])

    sdf_file = os.path.join(pkg_dir, 'models', 'argos_drone', 'model.sdf')

    # --- Spawn Drone (SDF 직접 스폰) ---
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

    # --- Gazebo Bridge ---
    # ROS→GZ: cmd_vel via /model/ prefix (MulticopterVelocityControl 입력)
    # GZ→ROS: odom via /model/ prefix, camera, IMU
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        namespace=name,
        arguments=[
            f'/model/{name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            f'/model/{name}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            f'/{name}/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            f'/{name}/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            f'/{name}/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            f'/{name}/thermal/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        output='screen',
    )

    # --- Topic Relays ---
    # cmd_vel: 드론 컨트롤러(/{name}/cmd_vel) → 브릿지(/model/{name}/cmd_vel)
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

    # odom: 브릿지(/model/{name}/odometry) → 드론 컨트롤러(/{name}/odom)
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

    # --- Drone Controller (10초 지연: Gazebo 스폰 대기) ---
    # sherpa_fire_test.sdf 월드 기준 geofence (indoor_test.sdf와 동일 크기 가정)
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
                    'cruise_altitude': 2.0,       # 실내 천장 고려 (2.5m - 0.5m 여유)
                    'max_horizontal_speed': 2.0,
                    'max_vertical_speed': 1.0,
                    'geofence_x_min': 0.5,
                    'geofence_x_max': 9.5,
                    'geofence_y_min': 0.5,
                    'geofence_y_max': 7.5,
                    'geofence_z_max': 2.2,        # 천장 2.5m - 0.3m 안전 마진
                }],
                output='screen',
            ),
        ],
    )

    # --- Robot Status Publisher ---
    robot_status_pub = Node(
        package='argos_bringup',
        executable='robot_status',
        name='robot_status_publisher',
        namespace=name,
        parameters=[{
            'use_sim_time': True,
            'robot_id': name,
            'robot_type': 'drone',
            'capabilities': ['camera', 'imu'],
        }],
        output='screen',
    )

    return [
        spawn_drone,
        gz_bridge,
        cmd_vel_relay,
        odom_relay,
        drone_controller,
        robot_status_pub,
    ]


def generate_launch_description():
    pkg_dir = get_package_share_directory('argos_description')

    # 셰르파 전용 URDF/YAML
    sherpa_urdf = os.path.join(pkg_dir, 'urdf', 'sherpa', 'sherpa.urdf.xacro')
    sherpa_params = os.path.join(pkg_dir, 'config', 'sherpa_controllers.yaml')
    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    # --- Launch Arguments ---
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_dir, 'worlds', 'sherpa_fire_test.sdf'),
        description='Gazebo world file (기본: sherpa_fire_test.sdf)',
    )

    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run Gazebo headless (no GUI, EGL rendering for sensors)',
    )

    # --- Gazebo Harmonic ---
    # headless=true 시 -s --headless-rendering 플래그 추가 (GPU 없는 CI 환경 대응)
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

    # --- 공통 /clock 브릿지 (1개만 — 중복 발행 방지) ---
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
    )

    # --- 오케스트레이터 (중앙 지휘, 호스 제약 통합 버전) ---
    # 150초 지연: Nav2 활성화 대기
    # 타이밍 근거 (RTF 0.28x 기준):
    #   DDC 완료 → +45s Nav2 include 시작 → +30s lifecycle_manager 지연(bringup 내부)
    #   → configure/activate (slam_toolbox 포함 8노드, RTF 0.28x에서 ~60s wall)
    #   = DDC 완료 후 약 135s wall time 소요.
    #   오케스트레이터는 heartbeat(10s) + Deadline QoS(5s)로 로봇 등록 대기하므로
    #   Nav2 완전 active 이후 시작 필수. 150s로 상향 (RTF 0.28x 여유 확보).
    all_robot_names = [s['name'] for s in SHERPAS] + [d['name'] for d in DRONES]
    orchestrator = TimerAction(
        period=150.0,
        actions=[
            Node(
                package='argos_bringup',
                executable='orchestrator',
                name='orchestrator',
                parameters=[{
                    'use_sim_time': True,
                    'expected_robots': all_robot_names,
                }],
                output='screen',
            ),
        ],
    )

    # --- 전체 엔티티 조립 ---
    all_entities = [world_arg, headless_arg, gazebo, clock_bridge]

    # 셰르파 2대 스택
    for sherpa in SHERPAS:
        all_entities.extend(
            generate_sherpa_actions(
                name=sherpa['name'],
                x=sherpa['x'],
                y=sherpa['y'],
                z=sherpa['z'],
                pkg_dir=pkg_dir,
                urdf_file=sherpa_urdf,
                nav2_params=nav2_params,
                sherpa_params=sherpa_params,
            )
        )

    # 드론 1대 스택
    for drone in DRONES:
        all_entities.extend(
            generate_drone_actions(drone, pkg_dir)
        )

    all_entities.append(orchestrator)

    return LaunchDescription(all_entities)
