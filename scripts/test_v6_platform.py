#!/usr/bin/env python3
"""V-6: UGVPlatform Gazebo 실증 스크립트.

단일 로봇(navigation.launch.py) 환경에서 UGVPlatform 검증.
robot_id='' → 네임스페이스 없는 토픽 사용 (/navigate_to_pose, /cmd_vel)

실행: navigation.launch.py 기동 후
  python3 scripts/test_v6_platform.py
"""
import sys
import time
import threading
sys.path.insert(0, '/home/jamonnine/ros2_ws/install/argos_bringup/lib/python3.12/dist-packages')

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from argos_bringup.ugv_platform import UGVPlatform


def spin_thread(executor):
    """별도 스레드에서 노드 spin (TF/odom 콜백 처리)."""
    executor.spin()


def main():
    rclpy.init()
    node = Node('v6_test', parameter_overrides=[
        rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True),
    ])

    # robot_id='' → 단일 로봇 (네임스페이스 없음)
    platform = UGVPlatform(node, robot_id='')

    # spin 스레드 시작 (TF, odom 콜백 처리용)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_t = threading.Thread(target=spin_thread, args=(executor,), daemon=True)
    spin_t.start()

    # 1. Capabilities
    caps = platform.get_capabilities()
    print(f'[V-6] Capabilities: can_fly={caps.can_fly}, has_thermal={caps.has_thermal}, can_drive={caps.can_drive}')

    # 2. 초기 pose (TF 안정화 대기) — get_pose() returns (x, y, z) tuple
    for i in range(10):
        time.sleep(1)
        pose = platform.get_pose()
        if pose and pose != (0.0, 0.0, 0.0):
            print(f'[V-6] Initial pose: x={pose[0]:.3f}, y={pose[1]:.3f}, z={pose[2]:.3f}')
            break
    else:
        print(f'[V-6] Initial pose after 10s: {pose}')

    # 3. Battery
    batt = platform.get_battery()
    print(f'[V-6] Battery: {batt:.1f}%')

    # 4. move_to
    print('[V-6] Sending move_to(1.0, 0.0, 0.0)...')
    try:
        result = platform.move_to(1.0, 0.0, 0.0)
        print(f'[V-6] move_to accepted: {result}')
    except Exception as e:
        print(f'[V-6] move_to error: {e}')

    # 5. 이동 대기 (40초)
    print('[V-6] Waiting 40s for navigation...')
    time.sleep(40)

    # 6. 최종 pose
    pose = platform.get_pose()
    if pose and pose != (0.0, 0.0, 0.0):
        print(f'[V-6] Final pose: x={pose[0]:.3f}, y={pose[1]:.3f}')
    else:
        print(f'[V-6] Final pose: {pose}')

    # 7. emergency_stop
    print('[V-6] Emergency stop...')
    platform.emergency_stop()
    print('[V-6] Emergency stop sent')

    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    print('[V-6] TEST COMPLETE')


if __name__ == '__main__':
    main()
