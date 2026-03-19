"""
ARGOS 멀티로봇 분산 SLAM 맵 병합 Launch
=========================================
각 UGV가 slam_toolbox로 독립 생성한 로컬 맵을
multirobot_map_merge 패키지로 통합하여 하나의 글로벌 맵을 출력.

입력 토픽:
  /argos1/map  (nav_msgs/OccupancyGrid, TRANSIENT_LOCAL QoS)
  /argos2/map  (nav_msgs/OccupancyGrid, TRANSIENT_LOCAL QoS)

출력 토픽:
  /map_merged  (nav_msgs/OccupancyGrid)

패키지 설치 (ROS 2 Jazzy):
  sudo apt install ros-jazzy-multirobot-map-merge
  # 바이너리 미제공 시 소스 빌드:
  # git clone -b jazzy https://github.com/ifollow-robotics/multirobot_map_merge ros2_ws/src/

사용법 (단독):
  ros2 launch argos_description map_merge.launch.py

exploration.launch.py에서 include 시:
  from launch.launch_description_sources import PythonLaunchDescriptionSource
  IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(pkg_dir, 'launch', 'map_merge.launch.py')
      )
  )

주의:
  - known_init_poses: true → exploration.launch.py의 ROBOTS 스폰 좌표와 반드시 동기화
  - SLAM 노드들이 /argos*/map을 발행하기 시작한 뒤 실행할 것 (TimerAction 권장)
  - 멀티로봇 map_frame은 'map' (단일 글로벌 프레임)으로 통일 가정
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# exploration.launch.py ROBOTS 리스트와 반드시 동기화
# 형식: (로봇 이름, 초기 x, 초기 y, 초기 yaw_rad)
ROBOT_INIT_POSES = [
    ('argos1', 3.0, 2.5, 0.0),
    ('argos2', 7.0, 2.5, 0.0),
]


def generate_launch_description():
    pkg_dir = get_package_share_directory('argos_description')
    params_file = os.path.join(pkg_dir, 'config', 'map_merge_params.yaml')

    # ── Launch 인자 ────────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='시뮬레이션 클럭 사용 여부 (Gazebo 환경: true)',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── 초기 위치 파라미터 동적 생성 ──────────────────────────────────
    # YAML 파일 값과 이중화: launch 파라미터로도 명시하여 YAML 로드 실패 시 대비
    init_pose_params = {}
    for name, x, y, yaw in ROBOT_INIT_POSES:
        init_pose_params[f'init_pose_{name}'] = [float(x), float(y), float(yaw)]

    # ── multirobot_map_merge 노드 ──────────────────────────────────────
    # 입력: /<robot_name>/map (각 로봇 SLAM 맵 토픽, TRANSIENT_LOCAL QoS)
    # 출력: /map_merged
    #
    # 토픽 탐색 방식:
    #   known_init_poses: true → map_topics 파라미터로 명시적 지정
    #   known_init_poses: false → 동적 탐색 (초기 위치 미리 알 수 없을 때)
    #
    # map_topics: 각 로봇의 맵 토픽을 명시적으로 지정
    #   이유: discovery_rate로 자동 탐색 시 TRANSIENT_LOCAL QoS 맵을
    #         수신 타이밍에 따라 놓칠 수 있으므로 명시 지정이 안정적
    map_topics = ' '.join([f'/{name}/map' for name, *_ in ROBOT_INIT_POSES])

    map_merge_node = Node(
        package='multirobot_map_merge',
        executable='map_merge',
        name='map_merge',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                # 병합 주기 (1Hz)
                'merging_rate': 1.0,
                # 새 로봇 탐색 주기 (0.5Hz)
                'discovery_rate': 0.5,
                # 위치 추정 임계값
                'estimation_confidence': 0.5,
                # 초기 위치 알려짐: True → init_pose_<robot> 파라미터 사용
                'known_init_poses': True,
                # 출력 해상도 (slam_params.yaml과 통일)
                'resolution': 0.05,
                # 각 로봇 초기 위치 [x, y, yaw(rad)]
                **init_pose_params,
                # 명시적 맵 토픽 목록 (공백 구분 문자열)
                'map_topics': map_topics,
            },
        ],
        # /map_merged → /map_merged (글로벌 네임스페이스 유지)
        # 리매핑 없이 기본 출력 토픽 사용
        remappings=[],
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_merge_node,
    ])
