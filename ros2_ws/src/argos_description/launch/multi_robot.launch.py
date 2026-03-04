"""
ARGOS 멀티로봇 Launch (Gazebo + N대 UGV)
==========================================
같은 URDF로 여러 ARGOS UGV를 네임스페이스 분리하여 동시 스폰.
각 로봇은 독립적인 controller_manager, 센서 토픽, TF tree를 가짐.

실행 순서:
  1. Gazebo 시뮬레이션 (월드만, 로봇 없이)
  2. 로봇 N대 각각: RSP → 스폰 → JSB → DDC → 브리지 → 릴레이
  3. (옵션) Nav2 + SLAM

사용법:
  ros2 launch argos_description multi_robot.launch.py
  ros2 launch argos_description multi_robot.launch.py world:=empty.sdf
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


# --- 로봇 설정 (exploration.launch.py와 동일하게 유지할 것) ---
ROBOTS = [
    {'name': 'argos1', 'x': 3.0, 'y': 2.5, 'z': 0.3},
    {'name': 'argos2', 'x': 7.0, 'y': 2.5, 'z': 0.3},
]


def spawn_robot_group(robot_config, pkg_dir, urdf_file):
    """단일 로봇의 전체 스택 구성 (/clock은 generate_launch_description에서 공통 처리)."""
    name = robot_config['name']
    x = str(robot_config['x'])
    y = str(robot_config['y'])
    z = str(robot_config['z'])

    # URDF (네임스페이스 파라미터 전달)
    robot_description = ParameterValue(
        Command([
            'xacro ', urdf_file,
            ' use_sim:=true',
            ' namespace:=', name,
            ' robot_name:=', name,
        ]),
        value_type=str,
    )

    # 브리지 설정 파일
    bridge_config = os.path.join(pkg_dir, 'config', 'gz_bridge.yaml')

    # --- Robot State Publisher ---
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

    # --- Gazebo 스폰 ---
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

    # --- 컨트롤러 Spawner (ExecuteProcess → Node 방식) ---
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

    # 스폰 완료 → JSB → DDC 순서 보장
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

    # --- 토픽 릴레이 (cmd_vel / odom) ---
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

    # --- Gazebo ↔ ROS 2 브리지 (네임스페이스 적용) ---
    # 각 로봇의 센서 토픽을 개별 브리지
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        namespace=name,
        arguments=[
            # LiDAR
            f'/{name}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # RGB Camera
            f'/{name}/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            f'/{name}/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # Depth Camera
            f'/{name}/depth_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            f'/{name}/depth_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            f'/{name}/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            # IMU
            f'/{name}/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # Thermal Camera
            f'/{name}/thermal/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
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
    ]


def generate_launch_description():
    pkg_dir = get_package_share_directory('argos_description')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'argos_ugv.urdf.xacro')

    # --- Launch 인자 ---
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_dir, 'worlds', 'indoor_test.sdf'),
        description='Gazebo world file',
    )

    # --- Gazebo Harmonic (월드만) ---
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

    # --- 각 로봇 스택 구성 ---
    all_entities = [world_arg, gazebo, clock_bridge]
    for robot in ROBOTS:
        all_entities.extend(
            spawn_robot_group(robot, pkg_dir, urdf_file)
        )

    return LaunchDescription(all_entities)
