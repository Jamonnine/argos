"""
ARGOS UGV Gazebo Harmonic 시뮬레이션 Launch
=============================================
Gazebo에 로봇을 스폰하고, ros_gz_bridge로 토픽 브리지 연결.
diff_drive_controller로 /cmd_vel 수신 → 4WD 바퀴 회전.

사용법:
  ros2 launch argos_description gazebo.launch.py
  ros2 launch argos_description gazebo.launch.py world:=fire_building.sdf
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
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('argos_description')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'argos_ugv.urdf.xacro')
    bridge_config = os.path.join(pkg_dir, 'config', 'gz_bridge.yaml')

    # Launch 인자 — 기본 월드를 indoor_test.sdf로 (시연용)
    indoor_world = os.path.join(pkg_dir, 'worlds', 'indoor_test.sdf')
    world_arg = DeclareLaunchArgument(
        'world', default_value=indoor_world,
        description='Gazebo world file'
    )

    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run Gazebo headless (no GUI, EGL rendering for sensors)'
    )

    # URDF → robot_description 파라미터 (namespace 비어있음 = 단일 로봇)
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file, ' use_sim:=true namespace:= robot_name:=argos_ugv']),
        value_type=str
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # Gazebo Harmonic 실행
    # headless=true → -s --headless-rendering (서버 모드 + EGL 렌더링)
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

    # Gazebo에 로봇 스폰
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'argos_ugv',
            '-topic', '/robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.3',
        ],
        output='screen',
    )

    # Gazebo ↔ ROS 2 브리지
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p',
                   f'config_file:={bridge_config}'],
        output='screen',
    )

    # 컨트롤러 (Node spawner — 멀티로봇 확장 대응 패턴 통일)
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

    # 스폰 완료 후 JSB 로드 (gz_ros2_control이 controller_manager를 먼저 초기화해야 함)
    jsb_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[jsb_spawner],
        )
    )

    # JSB 로드 완료 후 DDC 로드 (순서 보장)
    ddc_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[ddc_spawner],
        )
    )

    # cmd_vel/odom 릴레이 제거: ros2_control.urdf.xacro의 remapping으로 직접 연결
    # diff_drive_controller/cmd_vel → cmd_vel, diff_drive_controller/odom → odom

    return LaunchDescription([
        world_arg,
        headless_arg,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        gz_bridge,
        jsb_after_spawn,
        ddc_after_jsb,
    ])
