"""
ARGOS 웹 모니터링 Launch
========================
rosbridge_server를 시작하여 웹 대시보드에서 ROS 2 토픽을
실시간 모니터링할 수 있게 한다.

설치 (최초 1회):
  sudo apt install ros-jazzy-rosbridge-server

사용법:
  # 1) 탐색/시연 launch와 별도 터미널에서 실행
  ros2 launch argos_description monitor.launch.py

  # 2) 다른 터미널에서 웹 서버 시작
  cd $(ros2 pkg prefix argos_description)/share/argos_description/web
  python3 -m http.server 8080

  # 3) 브라우저에서 접속
  http://localhost:8080
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'port', default_value='9090',
        description='rosbridge WebSocket port',
    )

    # rosbridge_server launch (런타임에 경로 해석 — 미설치 시 launch 에러)
    rosbridge_launch = PathJoinSubstitution([
        FindPackageShare('rosbridge_server'),
        'launch', 'rosbridge_websocket_launch.xml',
    ])

    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(rosbridge_launch),
        launch_arguments={
            'port': LaunchConfiguration('port'),
        }.items(),
    )

    web_dir = PathJoinSubstitution([
        FindPackageShare('argos_description'), 'web',
    ])

    info_msg = LogInfo(msg=[
        '\n',
        '=' * 55, '\n',
        '  ARGOS Web Dashboard\n',
        '=' * 55, '\n',
        '  WebSocket: ws://localhost:', LaunchConfiguration('port'), '\n',
        '  Web files: ', web_dir, '\n',
        '\n',
        '  To open dashboard:\n',
        '    cd ', web_dir, '\n',
        '    python3 -m http.server 8080\n',
        '    Then open http://localhost:8080\n',
        '=' * 55,
    ])

    return LaunchDescription([
        port_arg,
        rosbridge,
        info_msg,
    ])
