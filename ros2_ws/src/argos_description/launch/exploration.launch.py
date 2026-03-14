"""
ARGOS 이종 군집 자율 탐색 Launch
================================
UGV N대 + 드론 M대를 Gazebo에 스폰하고, 각 로봇에 독립적인
자율 탐색 스택을 구성. 오케스트레이터가 전체를 통합 지휘.

UGV: Nav2 + SLAM + 프론티어 탐색 + 열화상 감지
드론: 웨이포인트 P 제어 + 하방 카메라 (MulticopterVelocityControl)

사용법:
  ros2 launch argos_description exploration.launch.py
  ros2 launch argos_description exploration.launch.py world:=empty.sdf
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
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


# --- UGV 설정 (multi_robot.launch.py와 동일하게 유지할 것) ---
ROBOTS = [
    {'name': 'argos1', 'x': 3.0, 'y': 2.5, 'z': 0.3},
    {'name': 'argos2', 'x': 7.0, 'y': 2.5, 'z': 0.3},
]

# --- 드론 설정 ---
DRONES = [
    {'name': 'drone1', 'x': 5.0, 'y': 0.0, 'z': 0.2},
]


def exploration_robot_group(robot_config, pkg_dir, urdf_file, nav2_params):
    """단일 로봇: 스폰 + 컨트롤러 + Nav2/SLAM + 탐색 전체 스택."""
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

    # --- RSP ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=name,
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
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
            '-x', x, '-y', y, '-z', z,
            '-allow_renaming', 'true',
        ],
        output='screen',
    )

    # --- Controllers ---
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

    # --- Topic Relays ---
    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        namespace=name,
        arguments=[
            f'/{name}/cmd_vel',
            f'/{name}/diff_drive_controller/cmd_vel',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    odom_relay = Node(
        package='topic_tools',
        executable='relay',
        name='odom_relay',
        namespace=name,
        arguments=[
            f'/{name}/diff_drive_controller/odom',
            f'/{name}/odom',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # --- Gazebo Bridge (per robot, /clock은 generate_launch_description에서 공통 처리) ---
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

    # --- Nav2 + SLAM (per robot, 25초 지연: 멀티로봇 Gazebo + 컨트롤러 초기화 대기) ---
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

    # --- Hotspot Detector ---
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

    # --- Frontier Explorer (40초 지연: Nav2 완전 활성화 대기) ---
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

    # --- Robot Status Publisher (오케스트레이터에 상태 보고) ---
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
        cmd_vel_relay,
        odom_relay,
        gz_bridge,
        nav2_bringup,
        hotspot_detector,
        frontier_explorer,
        robot_status_pub,
    ]


def drone_group(drone_config, pkg_dir):
    """단일 드론: SDF 모델 스폰 + 브릿지 + P 제어기 + 상태 보고."""
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

    # --- Robot Status Publisher (오케스트레이터에 드론 상태 보고) ---
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
    urdf_file = os.path.join(pkg_dir, 'urdf', 'argos_ugv.urdf.xacro')
    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_dir, 'worlds', 'indoor_test.sdf'),
        description='Gazebo world file',
    )

    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run Gazebo headless (no GUI, EGL rendering for sensors)',
    )

    # --- Gazebo Harmonic ---
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

    # --- 오케스트레이터 (중앙 지휘 — 45초 지연: Nav2 + 탐색 초기화 대기) ---
    all_robot_names = [r['name'] for r in ROBOTS] + [d['name'] for d in DRONES]
    orchestrator = TimerAction(
        period=45.0,
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

    # --- 공통 /clock 브릿지 (1개만 필요) ---
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
    )

    # --- 전체 엔티티 조립 ---
    all_entities = [world_arg, gazebo, clock_bridge]

    # UGV 스택
    for robot in ROBOTS:
        all_entities.extend(
            exploration_robot_group(
                robot, pkg_dir, urdf_file, nav2_params)
        )

    # 드론 스택
    for drone in DRONES:
        all_entities.extend(
            drone_group(drone, pkg_dir))

    all_entities.append(orchestrator)

    return LaunchDescription(all_entities)
