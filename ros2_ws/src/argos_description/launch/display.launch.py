"""
ARGOS UGV RViz 시각화 Launch
==============================
URDF 확인용. Gazebo 없이 RViz에서 로봇 모델만 띄움.
joint_state_publisher_gui로 바퀴 조인트 수동 조작 가능.

사용법:
  ros2 launch argos_description display.launch.py
"""

import os
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('argos_description')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'argos_ugv.urdf.xacro')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file, ' use_sim:=false']),
        value_type=str
    )

    return LaunchDescription([
        # Robot State Publisher: URDF → TF 변환
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),

        # Joint State Publisher GUI: 바퀴 조인트 수동 제어 슬라이더
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_dir, 'rviz', 'display.rviz')],
        ),
    ])
