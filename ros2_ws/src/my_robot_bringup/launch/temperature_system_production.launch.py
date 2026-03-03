#!/usr/bin/env python3
"""
Temperature System - Production Environment Launch

프로덕션 환경용 Launch File입니다.
config/temperature_monitor_production.yaml에서 설정을 읽어옵니다.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 패키지의 share 디렉토리 경로 얻기
    package_dir = get_package_share_directory('my_robot_bringup')

    # YAML 파일 경로 (프로덕션 환경용)
    config_file = os.path.join(
        package_dir,
        'config',
        'temperature_monitor_production.yaml'
    )

    temperature_sensor_node = Node(
        package='my_robot_bringup',
        executable='temperature_sensor',
        name='temperature_sensor',
        output='screen',
    )

    temperature_monitor_node = Node(
        package='my_robot_bringup',
        executable='temperature_monitor',
        name='temperature_monitor',
        output='screen',
        parameters=[config_file]  # YAML 파일에서 파라미터 로드
    )

    return LaunchDescription([
        temperature_sensor_node,
        temperature_monitor_node,
    ])
