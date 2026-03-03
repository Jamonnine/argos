#!/usr/bin/env python3
"""
Multi-Node Temperature System Launch

이 Launch File은 다중 노드 시스템을 시연합니다:
- Temperature Sensor (데이터 생성)
- Temperature Monitor (경고 시스템)
- Data Processor (통계 계산)

아키텍처 패턴:
- Fan-Out: Sensor → 여러 Subscriber
- Pipeline: Sensor → Processor → (미래의 Visualizer)
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    세 개의 노드를 동시에 실행하여 데이터 파이프라인을 구성합니다.
    """

    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    # 노드 1: Temperature Sensor (데이터 소스)
    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    sensor_node = Node(
        package='my_robot_bringup',
        executable='temperature_sensor',
        name='temperature_sensor',
        output='screen',
    )

    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    # 노드 2: Temperature Monitor (경고 시스템)
    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    monitor_node = Node(
        package='my_robot_bringup',
        executable='temperature_monitor',
        name='temperature_monitor',
        output='screen',
        parameters=[{
            'threshold_high': 27.0,
            'threshold_low': 23.0,
        }]
    )

    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    # 노드 3: Data Processor (통계 계산)
    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    processor_node = Node(
        package='my_robot_bringup',
        executable='data_processor',
        name='data_processor',
        output='screen',
        parameters=[{
            'window_size': 10,  # 최근 10개 샘플로 통계 계산
        }]
    )

    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    # 모든 노드 반환
    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    return LaunchDescription([
        sensor_node,
        monitor_node,
        processor_node,
    ])
