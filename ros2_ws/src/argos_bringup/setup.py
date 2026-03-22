from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'argos_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files 등록
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Config files 등록 (YAML + RViz)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        # Scripts 등록 (Python 실행 파일)
        (os.path.join('lib', package_name), glob('scripts/*.py')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='jamonnine',
    maintainer_email='jamonnine@github.com',
    description='ARGOS core nodes: orchestrator, 8-sensor suite, frontier explorer, drone controller',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
            'pytest-cov',
        ],
    },
    entry_points={
        'console_scripts': [
            'temperature_sensor = argos_bringup.temperature_sensor:main',
            'temperature_monitor = argos_bringup.temperature_monitor:main',
            'data_processor = argos_bringup.data_processor:main',
            'turtle_controller = argos_bringup.turtle_controller:main',
            'perception_bridge = argos_bringup.perception_bridge_node:main',
            'qos_demo = argos_bringup.qos_demo_node:main',
            'patrol_server = argos_bringup.patrol_action_server:main',
            'patrol_client = argos_bringup.patrol_action_client:main',
            'hotspot_detector = argos_bringup.hotspot_detector_node:main',
            'frontier_explorer = argos_bringup.frontier_explorer_node:main',
            'orchestrator = argos_bringup.orchestrator_node:main',
            'robot_status = argos_bringup.robot_status_node:main',
            'drone_controller = argos_bringup.drone_controller_node:main',
            'scenario_runner = argos_bringup.scenario_runner_node:main',
            'gas_sensor = argos_bringup.gas_sensor_node:main',
            'victim_detector = argos_bringup.victim_detector_node:main',
            'structural_monitor = argos_bringup.structural_monitor_node:main',
            'audio_detector = argos_bringup.audio_detector_node:main',
            'smoke_effect = argos_bringup.smoke_effect_node:main',
            'step_detector = argos_bringup.step_detector_node:main',
            'px4_bridge = argos_bringup.px4_bridge_node:main',
            'mcp_robot_server = argos_bringup.mcp_robot_server:main',
            # lidar_link TF 우회: /scan → /scan_base (frame_id=base_footprint) 릴레이
            'scan_frame_relay = argos_bringup.scan_frame_relay:main',
            # 위험 구역 keepout zone 동적 관리
            'keepout_manager = argos_bringup.keepout_manager:main',
            # Kalman 화점 추적 + 확산 예측
            'kalman_fire_tracker = argos_bringup.kalman_fire_tracker:main',
            # Platform 계층 — PlatformInterface 구현체 테스트 진입점
            'ugv_platform = argos_bringup.ugv_platform:main',
            'px4_platform = argos_bringup.px4_platform:main',
            # HR-셰르파 호스 릴 상태 추적
            'hose_tether = argos_bringup.hose_tether_node:main',
            # HR-셰르파 자체분무(Water Curtain) + 방수포 시뮬레이션
            'water_curtain = argos_bringup.water_curtain_node:main',
            # 방수포 분사 시 LiDAR 신뢰도 저하 시뮬레이션 (NFRI 실험 데이터 기반)
            'lidar_degradation = argos_bringup.lidar_degradation_node:main',
            # S-D4: 드론 열화상 → 오케스트레이터 FireAlert 릴레이 (드론→UGV 핸드오프)
            'drone_fire_relay = argos_bringup.drone_fire_relay_node:main',
            # S-D5: 이종 군집 편대 패턴 (횡대/종대/제대/포위) — FormationManager는 라이브러리, 노드 아님
        ],
    },
)
