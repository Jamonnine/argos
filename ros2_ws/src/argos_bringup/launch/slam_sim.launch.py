"""
SLAM 시뮬레이션 Launch 파일 (Day 16)

slam_toolbox online_async 모드로 실시간 지도 생성.
- /scan → slam_toolbox → /map (OccupancyGrid)
- slam_toolbox가 map→odom TF도 브로드캐스트
- use_sim_time:=True 필수 (Gazebo 시뮬레이션 시간 사용)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    slam_params_file = os.path.join(
        get_package_share_directory('argos_bringup'),
        'config',
        'slam_params.yaml'
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': True}],
    )

    rviz_config = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'),
        'rviz',
        'tb3_navigation2.rviz'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([slam_toolbox, rviz])
