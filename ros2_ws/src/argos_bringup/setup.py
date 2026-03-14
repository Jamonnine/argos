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
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jamonnine',
    maintainer_email='jamonnine@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
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
        ],
    },
)
