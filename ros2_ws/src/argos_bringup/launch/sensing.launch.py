"""sensing.launch.py — ARGOS 8중 센싱 스택 launch

가스/피해자/구조물/음향 4개 센싱 노드를 구동.
기존 exploration.launch.py 또는 demo.launch.py와 함께 사용.

사용법:
  # 단독 구동 (시뮬레이션 모드)
  ros2 launch argos_bringup sensing.launch.py

  # 특정 로봇 네임스페이스
  ros2 launch argos_bringup sensing.launch.py robot_id:=argos1

  # 피해자 위치 시뮬레이션
  ros2 launch argos_bringup sensing.launch.py victim_x:=5.0 victim_y:=3.0
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Launch 인자
    robot_id_arg = DeclareLaunchArgument(
        'robot_id', default_value='argos1',
        description='Robot identifier for namespace')

    sim_mode_arg = DeclareLaunchArgument(
        'simulation_mode', default_value='true',
        description='Use simulated sensors (true) or hardware (false)')

    # Config 파일 경로
    config_path = os.path.join(
        get_package_share_directory('argos_bringup'),
        'config', 'sensors.yaml')

    robot_id = LaunchConfiguration('robot_id')
    sim_mode = LaunchConfiguration('simulation_mode')

    # === 가스 센서 노드 ===
    gas_sensor_node = Node(
        package='argos_bringup',
        executable='gas_sensor',
        name='gas_sensor',
        namespace=robot_id,
        parameters=[config_path, {
            'robot_id': robot_id,
            'simulation_mode': sim_mode,
        }],
        remappings=[
            ('odom', 'odom'),
        ],
        output='screen',
    )

    # === 피해자 감지 노드 ===
    victim_detector_node = Node(
        package='argos_bringup',
        executable='victim_detector',
        name='victim_detector',
        namespace=robot_id,
        parameters=[config_path, {
            'robot_id': robot_id,
            'simulation_mode': sim_mode,
        }],
        remappings=[
            ('odom', 'odom'),
            ('thermal/image_raw', 'thermal/image_raw'),
        ],
        output='screen',
    )

    # === 구조물 모니터링 노드 ===
    structural_monitor_node = Node(
        package='argos_bringup',
        executable='structural_monitor',
        name='structural_monitor',
        namespace=robot_id,
        parameters=[config_path, {
            'robot_id': robot_id,
        }],
        remappings=[
            ('scan', 'scan'),
            ('odom', 'odom'),
        ],
        output='screen',
    )

    # === 음향 감지 노드 ===
    audio_detector_node = Node(
        package='argos_bringup',
        executable='audio_detector',
        name='audio_detector',
        namespace=robot_id,
        parameters=[config_path, {
            'robot_id': robot_id,
            'simulation_mode': sim_mode,
        }],
        remappings=[
            ('odom', 'odom'),
        ],
        output='screen',
    )

    return LaunchDescription([
        robot_id_arg,
        sim_mode_arg,
        gas_sensor_node,
        victim_detector_node,
        structural_monitor_node,
        audio_detector_node,
    ])
