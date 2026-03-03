"""
RGB-D Point Cloud Launch 파일

이 Launch 파일은 RGB-D 카메라 시뮬레이션부터 Point Cloud 생성까지의
전체 파이프라인을 한 번에 시작합니다.

실행되는 노드들:
1. rgbd_camera_simulator: 가상의 RGB-D 카메라
2. depth_to_pointcloud: Depth → Point Cloud 변환

Launch 파일의 장점:
- 하나의 명령으로 여러 노드 실행
- 노드 간 의존성 관리 (순서 제어)
- 파라미터 중앙 관리
- 조건부 실행 (환경에 따라 다른 노드 실행)

용어 설명:
- Launch: "발사하다". 로켓을 쏘듯이 노드들을 한꺼번에 시작.
- LaunchDescription: Launch 파일의 "설명서". 어떤 노드를 어떻게 실행할지 정의.
- Node: 실행할 노드 하나를 표현하는 객체.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Launch Description 생성 함수

    ROS 2 Launch 시스템은 이 함수를 호출해서 무엇을 실행할지 알아냅니다.
    이 함수가 반환하는 LaunchDescription 객체에는 실행할 모든 노드/프로세스가
    담겨 있습니다.

    용어 설명:
    - generate_launch_description: ROS 2 Launch 시스템의 "진입점(Entry Point)".
      Launch 파일을 실행하면 이 함수가 자동으로 호출됨.
    - 함수 이름을 바꾸면 안 됨! ROS 2가 이 이름으로 찾기 때문.
    """

    # ===== 노드 1: RGB-D 카메라 시뮬레이터 =====
    # 용어 설명:
    # - Node: Launch에서 노드를 표현하는 클래스.
    # - package: 노드가 속한 패키지 이름.
    # - executable: 실행할 파일 이름. .py 확장자 포함.
    # - name: 노드의 이름. 이게 ros2 node list에 표시되는 이름.
    # - output: 노드의 출력을 어디로 보낼지.
    #   - 'screen': 터미널에 직접 출력 (로그를 즉시 볼 수 있음)
    #   - 'log': 파일에 저장 (~/.ros/log/)
    camera_node = Node(
        package='my_robot_bringup',
        executable='rgbd_camera_simulator.py',
        name='rgbd_camera_simulator',
        output='screen',
        # emulate_tty=True는 "터미널처럼 동작"하라는 뜻.
        # 이게 있으면 색상 코드(ANSI color)가 제대로 표시됨.
        # 없으면 로그가 흑백으로만 나옴.
        emulate_tty=True,
    )

    # ===== 노드 2: Depth to Point Cloud 변환기 =====
    pointcloud_node = Node(
        package='my_robot_bringup',
        executable='depth_to_pointcloud.py',
        name='depth_to_pointcloud',
        output='screen',
        emulate_tty=True,
    )

    # ===== (선택) RViz2 자동 실행 =====
    # RViz2를 Launch 파일에서 바로 실행할 수도 있습니다.
    # 하지만 여기서는 주석 처리해둡니다. 왜냐하면:
    # 1. RViz2는 GUI 프로그램이라 Launch에서 실행하면 종료 시 복잡함.
    # 2. RViz2 설정(.rviz 파일)을 먼저 저장해야 자동 로딩 가능.
    # 3. 학습 목적이므로 수동으로 실행하는 게 더 명확함.
    #
    # 용어 설명:
    # - ExecuteProcess: 일반 프로그램(ROS 노드가 아님)을 실행하는 액션.
    # - cmd: 실행할 명령어와 인자를 리스트로.
    # - output: 출력 방식.
    #
    # rviz_node = ExecuteProcess(
    #     cmd=['rviz2'],
    #     output='screen'
    # )

    # ===== 노드 3: Voxel Grid Downsampler (Day 11 추가) =====
    # parameters: 노드에 파라미터를 전달하는 방법.
    # 딕셔너리 리스트 형태로 전달함.
    # 여기서 voxel_size=0.01 은 1cm 복셀을 의미함.
    downsampler_node = Node(
        package='my_robot_bringup',
        executable='voxel_grid_downsampler.py',
        name='voxel_grid_downsampler',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'voxel_size': 0.05,        # 복셀 크기: 5cm
            'input_topic': '/camera/points',
            'output_topic': '/points_downsampled',
        }]
    )

    # ===== 노드 4: Statistical Outlier Removal (Day 11 추가) =====
    outlier_removal_node = Node(
        package='my_robot_bringup',
        executable='statistical_outlier_removal.py',
        name='statistical_outlier_removal',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'k_neighbors': 20,
            'std_multiplier': 2.0,
            'input_topic': '/points_downsampled',
            'output_topic': '/points_filtered',
        }]
    )

    # ===== 노드 5: RANSAC Plane Segmentation (Day 11 추가) =====
    plane_seg_node = Node(
        package='my_robot_bringup',
        executable='plane_segmentation.py',
        name='plane_segmentation',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'distance_threshold': 0.02,   # 2cm 이내 = 바닥 평면
            'ransac_iterations': 100,
            'input_topic': '/points_filtered',
            'output_obstacles_topic': '/points_obstacles',
            'output_plane_topic': '/points_plane',
        }]
    )

    # ===== 노드 6: Euclidean Clustering (Day 11 추가) =====
    clustering_node = Node(
        package='my_robot_bringup',
        executable='euclidean_clustering.py',
        name='euclidean_clustering',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'cluster_tolerance': 0.1,   # 10cm 이내 포인트는 같은 물체
            'min_cluster_size': 5,
            'max_cluster_size': 5000,
            'input_topic': '/points_obstacles',
        }]
    )

    # ===== RViz2 시각화 (Day 11 추가) =====
    # TimerAction: 다른 노드들이 먼저 시작된 후 3초 뒤에 RViz2를 띄움
    # 이유: RViz2가 너무 빨리 뜨면 토픽이 아직 없어서 연결이 안 됨
    pkg_share = get_package_share_directory('my_robot_bringup')
    rviz_config = os.path.join(pkg_share, 'config', 'pointcloud_pipeline.rviz')

    rviz_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                output='screen',
            )
        ]
    )

    # ===== 노드 7: Vision-to-MoveIt Bridge (Day 12 추가) =====
    bridge_node = Node(
        package='my_robot_bringup',
        executable='vision_moveit_bridge.py',
        name='vision_moveit_bridge',
        output='screen',
        emulate_tty=True,
    )

    # ===== 노드 8: Simulated Arm Controller (Day 12 추가) =====
    arm_sim_node = Node(
        package='my_robot_bringup',
        executable='arm_controller_sim.py',
        name='arm_controller_sim',
        output='screen',
        emulate_tty=True,
    )

    # ===== LaunchDescription 반환 =====
    return LaunchDescription([
        camera_node,
        pointcloud_node,
        downsampler_node,        # Day 11 추가
        outlier_removal_node,    # Day 11 추가
        plane_seg_node,          # Day 11 추가
        clustering_node,         # Day 11 추가
        bridge_node,             # Day 12 추가
        arm_sim_node,            # Day 12 추가
        rviz_node,               # Day 11 추가 - 3초 후 자동 시작
    ])
