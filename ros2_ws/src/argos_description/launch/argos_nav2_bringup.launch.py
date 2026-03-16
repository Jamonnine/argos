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
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushROSNamespace, SetParameter
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
    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'route_server',
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
            # SLAM (nav2_bringup의 slam_launch.py 재사용)
            # 핵심: params_file_replaced 사용 — <robot_namespace>/ 치환된 버전
            # 원본 params_file을 전달하면 slam_toolbox가 "<robot_namespace>/base_footprint"을
            # 문자 그대로 프레임 이름으로 사용하여 TF lookup 100% 실패함
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_launch_dir, 'slam_launch.py')
                ),
                condition=IfCondition(slam),
                launch_arguments={
                    'namespace': namespace,
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'use_respawn': 'False',
                    'params_file': params_file_replaced,
                }.items(),
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
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes},
                ],
            ),
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

    return ld
