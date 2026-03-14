"""
Nav2 시뮬레이션 Launch 파일 (Day 15)
use_composition:=False 로 각 노드를 독립 프로세스로 실행.
이렇게 해야 use_sim_time 파라미터가 모든 노드에 정확히 전달됨.

params_file로 nav2_params_sim.yaml 사용:
- burger.yaml 기반이지만 yaml_filename을 절대 경로로 수정
- 이유: burger.yaml의 map_server.ros__parameters.yaml_filename이 상대 경로 "map.yaml"로
  하드코딩되어 있어, launch 파일에서 절대 경로를 전달해도 노드 레벨 우선순위에 의해
  상대 경로가 적용됨 → 노드 자체 params에 절대 경로를 박아야 해결됨.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    tb3_nav2_dir = get_package_share_directory('turtlebot3_navigation2')
    bringup_dir = get_package_share_directory('argos_bringup')

    map_file = os.path.join(tb3_nav2_dir, 'map', 'map.yaml')
    # nav2_params_sim.yaml: burger.yaml 기반, yaml_filename 절대 경로로 수정
    params_file = os.path.join(bringup_dir, 'config', 'nav2_params_sim.yaml')
    rviz_config = os.path.join(tb3_nav2_dir, 'rviz', 'tb3_navigation2.rviz')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'use_composition': 'False',   # 핵심: 독립 프로세스로 실행
            'map': map_file,
            'params_file': params_file,
            'autostart': 'True',
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([nav2, rviz])
