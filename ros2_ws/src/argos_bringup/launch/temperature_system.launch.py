#!/usr/bin/env python3
"""
Temperature Monitoring System Launch File

이 Launch File은 온도 센서와 모니터를 함께 실행합니다.
실무에서 시스템 구성을 관리하는 방법을 보여줍니다.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch 시스템이 호출하는 함수
    여기서 실행할 노드들을 선언합니다.
    """

    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    # 노드 1: Temperature Sensor (Publisher)
    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    temperature_sensor_node = Node(
        package='argos_bringup',
        executable='temperature_sensor',
        name='temperature_sensor',  # 노드 이름 (ros2 node list에 표시됨)
        output='screen',  # 로그를 터미널에 출력
    )

    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    # 노드 2: Temperature Monitor (Subscriber)
    # Parameters를 여기서 설정할 수 있습니다
    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    temperature_monitor_node = Node(
        package='argos_bringup',
        executable='temperature_monitor',
        name='temperature_monitor',
        output='screen',
        parameters=[{
            'threshold_high': 27.0,  # Launch file에서 파라미터 설정
            'threshold_low': 23.0,
        }]
    )

    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    # LaunchDescription: 모든 노드를 모아서 반환
    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    return LaunchDescription([
        temperature_sensor_node,
        temperature_monitor_node,
    ])
