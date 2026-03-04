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
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('argos_description')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'argos_ugv.urdf.xacro')
    bridge_config = os.path.join(pkg_dir, 'config', 'gz_bridge.yaml')

    # Launch 인자
    world_arg = DeclareLaunchArgument(
        'world', default_value='empty.sdf',
        description='Gazebo world file'
    )

    # URDF → robot_description 파라미터
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file, ' use_sim:=true']),
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

    # 컨트롤러: joint_state_broadcaster (먼저 로드)
    load_jsb = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen',
    )

    # 컨트롤러: diff_drive_controller (JSB 다음에 로드)
    load_ddc = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_drive_controller'],
        output='screen',
    )

    # 스폰 완료 후 JSB 로드 (gz_ros2_control이 controller_manager를 먼저 초기화해야 함)
    jsb_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[load_jsb],
        )
    )

    # JSB 로드 완료 후 DDC 로드 (순서 보장)
    ddc_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_jsb,
            on_exit=[load_ddc],
        )
    )

    # --- 토픽 릴레이 (diff_drive_controller 네임스페이스 ↔ 표준 토픽) ---
    # diff_drive_controller는 /diff_drive_controller/cmd_vel을 구독하지만,
    # 수동 조작이나 Nav2는 /cmd_vel로 퍼블리시하므로 릴레이 필요.
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

    return LaunchDescription([
        world_arg,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        gz_bridge,
        jsb_after_spawn,
        ddc_after_jsb,
        cmd_vel_relay,
        odom_relay,
    ])
