"""
ARGOS Nav2 Bringup (docking_server 제외)
========================================
nav2_bringup의 bringup_launch.py를 대체.
Nav2 Jazzy의 기본 bringup은 opennav_docking을 포함하며,
dock 플러그인 미설정 시 lifecycle_manager가 전체 중단(abort)됨.

이 파일은 동일한 Nav2 스택을 docking 없이 구성:
  SLAM(slam_toolbox) + Nav2(controller, planner, behavior, bt_nav, ...)

사용법 (직접 사용하지 않음 — navigation.launch.py/exploration.launch.py에서 include):
  launch_arguments: use_sim_time, slam, params_file, autostart, namespace, use_namespace
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LifecycleNode, Node, PushROSNamespace, SetParameter
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_dir = os.path.join(nav2_bringup_dir, 'launch')

    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    slam = LaunchConfiguration('slam')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')

    # --- docking_server 제외한 lifecycle 노드 목록 ---
    # slam_toolbox를 맨 앞에 배치: SLAM이 먼저 활성화되어야 map TF 발행 시작
    # smoother_server, route_server 제거: nav2_params.yaml에 설정 없음
    # slam_toolbox 맨 앞 필수: SLAM→map TF→planner global_costmap 의존 체인
    lifecycle_nodes = [
        'slam_toolbox',
        'controller_server',
        'planner_server',
        'behavior_server',
        'velocity_smoother',
        'collision_monitor',
        'bt_navigator',
        'waypoint_follower',
    ]

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # 네임스페이스 치환 (멀티로봇 대응)
    # <robot_namespace>/ → argos1/ (멀티로봇) 또는 빈 문자열 (단일로봇)
    # TF 프레임 이름이 네임스페이스 접두사를 갖기 때문에 params도 일치시킴
    frame_prefix = PythonExpression([
        "'", namespace, "/' if '",
        use_namespace, "'.lower() == 'true' else ''",
    ])
    params_file_replaced = ReplaceString(
        source_file=params_file,
        replacements={'<robot_namespace>/': frame_prefix},
    )

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file_replaced,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    # --- Launch Arguments ---
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='',
    )
    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace', default_value='false',
    )
    declare_slam_cmd = DeclareLaunchArgument(
        'slam', default_value='True',
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
    )
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml'),
    )
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
    )

    # --- SLAM + Nav2 (단일 GroupAction으로 네임스페이스 통합 적용) ---
    bringup_group = GroupAction(
        actions=[
            PushROSNamespace(
                condition=IfCondition(use_namespace),
                namespace=namespace,
            ),
            # SLAM — slam_launch.py 우회, 직접 slam_toolbox 생성
            # 이유: slam_launch.py의 HasNodeParams가 ReplaceString/RewrittenYaml
            #   결과물에서 slam_toolbox 섹션을 탐지 못함 → 기본 설정 폴백 →
            #   scan_topic이 /scan으로 되어 lidar_link 프레임 수신 → SLAM 실패.
            # 해결: configured_params(RewrittenYaml+ParameterFile)로 직접 전달.
            LifecycleNode(
                condition=IfCondition(slam),
                package='slam_toolbox',
                executable='sync_slam_toolbox_node',
                name='slam_toolbox',
                namespace='',
                output='screen',
                parameters=[configured_params,
                            {'use_sim_time': use_sim_time,
                             'use_lifecycle_manager': True}],
                remappings=[('/scan', 'scan'), ('/tf', 'tf'), ('/tf_static', 'tf_static'),
                            ('/map', 'map')],
            ),
            # Nav2 Navigation Nodes (docking 제외)
            SetParameter('use_sim_time', use_sim_time),
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                parameters=[configured_params],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package='nav2_route',
                executable='route_server',
                name='route_server',
                output='screen',
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[configured_params],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                parameters=[configured_params],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_collision_monitor',
                executable='collision_monitor',
                name='collision_monitor',
                output='screen',
                parameters=[configured_params],
                remappings=remappings,
            ),
            # lifecycle_manager는 GroupAction 밖에서 30초 지연 시작 (아래 참조)
        ],
    )

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(bringup_group)

    # lifecycle_manager를 30초 지연 시작 (2026-03-19)
    # 이유: slam_toolbox configure가 Gazebo clock 안정화 전에 실패하면
    #   lifecycle_manager가 전체 abort → 30초면 clock+TF 안정화 충분
    ld.add_action(TimerAction(
        period=30.0,
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes},
                    {'bond_timeout': 0.0},
                    {'attempt_respawn_reconnection': True},
                ],
            ),
        ],
    ))

    return ld
