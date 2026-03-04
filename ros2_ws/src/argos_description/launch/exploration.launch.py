"""
ARGOS 멀티로봇 자율 탐색 Launch
================================
N대 UGV를 Gazebo에 스폰하고, 각 로봇에 독립적인
Nav2 + SLAM + 프론티어 탐색 + 열화상 감지 스택을 구성.

각 로봇은 자체 네임스페이스에서 독립적으로 SLAM/Nav2를 수행하며,
/exploration/targets 토픽으로 탐색 대상을 공유하여 중복 탐색을 방지.

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
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


# --- 로봇 설정 (multi_robot.launch.py와 동일) ---
ROBOTS = [
    {'name': 'argos1', 'x': 3.0, 'y': 2.5, 'z': 0.3},
    {'name': 'argos2', 'x': 7.0, 'y': 2.5, 'z': 0.3},
]


def exploration_robot_group(robot_config, pkg_dir, urdf_file, nav2_bringup_dir, nav2_params):
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

    # --- Gazebo Bridge (per robot) ---
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        namespace=name,
        arguments=[
            f'/{name}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            f'/{name}/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            f'/{name}/depth_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            f'/{name}/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            f'/{name}/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            f'/{name}/thermal/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
    )

    # --- Nav2 + SLAM (per robot, 10초 지연: Gazebo 초기화 대기) ---
    nav2_bringup = TimerAction(
        period=10.0,
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
                    'namespace': name,
                    'use_namespace': 'True',
                }.items(),
            ),
        ],
    )

    # --- Hotspot Detector ---
    hotspot_detector = Node(
        package='my_robot_bringup',
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

    # --- Frontier Explorer (15초 지연: Nav2 초기화 대기) ---
    frontier_explorer = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='my_robot_bringup',
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
        package='my_robot_bringup',
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


def generate_launch_description():
    pkg_dir = get_package_share_directory('argos_description')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'argos_ugv.urdf.xacro')
    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_dir, 'worlds', 'indoor_test.sdf'),
        description='Gazebo world file',
    )

    # --- Gazebo Harmonic ---
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

    # --- 오케스트레이터 (중앙 지휘 — 20초 지연: 로봇 초기화 대기) ---
    orchestrator = TimerAction(
        period=20.0,
        actions=[
            Node(
                package='my_robot_bringup',
                executable='orchestrator',
                name='orchestrator',
                parameters=[{
                    'use_sim_time': True,
                    'expected_robots': [r['name'] for r in ROBOTS],
                }],
                output='screen',
            ),
        ],
    )

    # --- 각 로봇 전체 스택 ---
    all_entities = [world_arg, gazebo]
    for robot in ROBOTS:
        all_entities.extend(
            exploration_robot_group(
                robot, pkg_dir, urdf_file, nav2_bringup_dir, nav2_params)
        )
    all_entities.append(orchestrator)

    return LaunchDescription(all_entities)
