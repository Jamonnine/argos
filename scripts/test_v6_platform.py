#!/usr/bin/env python3
"""V-6: UGVPlatform Gazebo 실증 스크립트."""
import sys
import time
sys.path.insert(0, '/home/jamonnine/ros2_ws/install/argos_bringup/lib/python3.12/dist-packages')

import rclpy
from rclpy.node import Node

rclpy.init()
node = Node('v6_test', parameter_overrides=[
    rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True),
])

# UGVPlatform import
from argos_bringup.ugv_platform import UGVPlatform
platform = UGVPlatform(node)

# capabilities 확인
caps = platform.get_capabilities()
print(f'[V-6] Capabilities: can_fly={caps.can_fly}, has_thermal={caps.has_thermal}, can_drive={caps.can_drive}')

# 초기 pose
time.sleep(3)
pose = platform.get_pose()
if pose:
    print(f'[V-6] Initial pose: x={pose.pose.position.x:.3f}, y={pose.pose.position.y:.3f}')
else:
    print('[V-6] Initial pose: not available yet')

# move_to
print('[V-6] Sending move_to(1.0, 0.0, 0.0)...')
try:
    result = platform.move_to(1.0, 0.0, 0.0)
    print(f'[V-6] move_to result: {result}')
except Exception as e:
    print(f'[V-6] move_to error: {e}')

# 30초 대기 후 pose
time.sleep(30)
pose = platform.get_pose()
if pose:
    print(f'[V-6] Final pose: x={pose.pose.position.x:.3f}, y={pose.pose.position.y:.3f}')
else:
    print('[V-6] Final pose: not available')

# battery
batt = platform.get_battery()
print(f'[V-6] Battery: {batt:.1f}%')

node.destroy_node()
rclpy.shutdown()
print('[V-6] TEST COMPLETE')
