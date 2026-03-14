#!/usr/bin/env python3
"""
Nav2 RViz2 Launch File
Navigation2를 위한 완벽한 RViz2 설정으로 실행
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 패키지 경로
    pkg_dir = get_package_share_directory('argos_bringup')

    # RViz 설정 파일 경로
    rviz_config_file = os.path.join(pkg_dir, 'config', 'nav2.rviz')

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': True}],
        ),
    ])
