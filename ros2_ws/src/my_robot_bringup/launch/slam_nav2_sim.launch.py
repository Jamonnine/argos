"""
SLAM + Nav2 통합 Launch 파일 (Day 17)

핵심 차이점 (vs nav2_sim.launch.py):
- slam=True: AMCL + map_server 대신 slam_toolbox 사용
- map 파일 불필요: slam_toolbox가 실시간으로 /map 제공
- slam_toolbox가 map→odom TF 제공 (AMCL 역할까지 수행)

이 파일 하나로:
  Gazebo 없이 SLAM + Nav2 + RViz2 실행
  (Gazebo는 별도 Task로 실행됨)
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
    bringup_dir = get_package_share_directory('my_robot_bringup')

    # nav2_params_sim.yaml 사용 (slam_toolbox 파라미터 없으면 기본값 사용됨)
    params_file = os.path.join(bringup_dir, 'config', 'nav2_params_sim.yaml')
    rviz_config = os.path.join(tb3_nav2_dir, 'rviz', 'tb3_navigation2.rviz')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'use_composition': 'False',
            'slam': 'True',          # 핵심: AMCL 대신 slam_toolbox 사용
            # 'map': 생략              # 지도 파일 불필요 - slam_toolbox가 실시간 제공
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
