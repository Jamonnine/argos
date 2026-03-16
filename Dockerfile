# ARGOS - 이종 군집 소방 로봇 오케스트레이션 플랫폼
# Docker 빌드: docker build -t argos:latest .
# Docker 실행: docker run -it argos:latest bash

FROM osrf/ros:jazzy-desktop

LABEL maintainer="jamonnine@github.com"
LABEL description="ARGOS: Autonomous Robot Group Orchestration System"
LABEL license="Apache-2.0"

# 시스템 의존성
RUN apt-get update -qq && apt-get install -y -qq \
    python3-pip \
    python3-pytest \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-ros-gz \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-controller-manager \
    ros-jazzy-diff-drive-controller \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-topic-tools \
    ros-jazzy-rosbridge-server \
    python3-opencv \
    python3-numpy \
    && rm -rf /var/lib/apt/lists/*

# 워크스페이스 설정
WORKDIR /ws
COPY ros2_ws/src /ws/src

# 빌드
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 엔트리포인트
COPY docker-entrypoint.sh /docker-entrypoint.sh
RUN chmod +x /docker-entrypoint.sh

ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["bash"]
