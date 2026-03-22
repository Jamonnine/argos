#!/usr/bin/env python3
"""S-A4 데모 녹화 스크립트.

Gazebo 카메라 토픽에서 이미지를 캡처하여 MP4 비디오로 저장.
Nav2 자율 이동 시나리오를 자동 실행하면서 동시 녹화.

사용법:
  1. navigation.launch.py 실행 (별도 터미널)
  2. python3 scripts/record_demo.py --duration 60 --output demo.mp4

또는 단독 실행 (launch 포함):
  python3 scripts/record_demo.py --with-launch --duration 90
"""
import argparse
import os
import sys
import time
import threading
import subprocess

sys.path.insert(0, '/home/jamonnine/ros2_ws/install/argos_bringup/lib/python3.12/dist-packages')

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class DemoRecorder(Node):
    """Gazebo 카메라 토픽 녹화 노드."""

    def __init__(self, output_path, fps=10.0, camera_topic='/camera/image_raw'):
        super().__init__('demo_recorder')
        self.declare_parameter('use_sim_time', True)

        self._bridge = CvBridge()
        self._output_path = output_path
        self._fps = fps
        self._writer = None
        self._frame_count = 0
        self._recording = True

        self._sub = self.create_subscription(
            Image, camera_topic, self._image_callback, 10)
        self.get_logger().info(f'Recording {camera_topic} → {output_path} @ {fps}fps')

    def _image_callback(self, msg):
        if not self._recording:
            return
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'Frame conversion error: {e}')
            return

        if self._writer is None:
            h, w = frame.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self._writer = cv2.VideoWriter(self._output_path, fourcc, self._fps, (w, h))
            self.get_logger().info(f'Video initialized: {w}x{h}')

        self._writer.write(frame)
        self._frame_count += 1
        if self._frame_count % 50 == 0:
            self.get_logger().info(f'Recorded {self._frame_count} frames')

    def stop(self):
        self._recording = False
        if self._writer:
            self._writer.release()
            self.get_logger().info(
                f'Recording complete: {self._frame_count} frames → {self._output_path}')


def run_nav2_demo(duration_sec):
    """Nav2 자율 이동 데모 시나리오."""
    time.sleep(5)  # 녹화 시작 대기

    env = os.environ.copy()
    bash = '/bin/bash'

    def ros2_cmd(cmd):
        subprocess.run(
            [bash, '-c', f'source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && {cmd}'],
            env=env, timeout=30, capture_output=True)

    # SLAM 프라이밍 (회전)
    print('[DEMO] SLAM priming (rotation)...')
    for _ in range(15):
        ros2_cmd('ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped '
                 '"{header: {frame_id: base_link}, twist: {linear: {x: 0.0}, angular: {z: 0.5}}}"')
        time.sleep(0.3)
    time.sleep(10)

    # Nav2 Goal 1
    print('[DEMO] Goal 1: (2.0, 0.0)...')
    ros2_cmd('timeout 45 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose '
             '"{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}}"')

    if duration_sec > 60:
        # Nav2 Goal 2
        print('[DEMO] Goal 2: (0.0, 2.0)...')
        ros2_cmd('timeout 45 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose '
                 '"{pose: {header: {frame_id: map}, pose: {position: {x: 0.0, y: 2.0}, orientation: {w: 1.0}}}}"')

    print('[DEMO] Scenario complete')


def main():
    parser = argparse.ArgumentParser(description='ARGOS Demo Recorder')
    parser.add_argument('--output', default='/tmp/argos_demo.mp4', help='Output video path')
    parser.add_argument('--duration', type=int, default=60, help='Recording duration (seconds)')
    parser.add_argument('--fps', type=float, default=10.0, help='Video FPS')
    parser.add_argument('--topic', default='/camera/image_raw', help='Camera topic')
    args = parser.parse_args()

    rclpy.init()
    recorder = DemoRecorder(args.output, args.fps, args.topic)

    executor = SingleThreadedExecutor()
    executor.add_node(recorder)
    spin_t = threading.Thread(target=executor.spin, daemon=True)
    spin_t.start()

    # 데모 시나리오 (별도 스레드)
    demo_t = threading.Thread(target=run_nav2_demo, args=(args.duration,), daemon=True)
    demo_t.start()

    # 녹화 시간
    print(f'[REC] Recording for {args.duration}s...')
    time.sleep(args.duration)

    # 종료
    recorder.stop()
    executor.shutdown()
    recorder.destroy_node()
    rclpy.shutdown()

    # 결과
    if os.path.exists(args.output):
        size_mb = os.path.getsize(args.output) / 1024 / 1024
        print(f'[REC] Saved: {args.output} ({size_mb:.1f} MB)')
    else:
        print('[REC] WARNING: Output file not created (no frames received)')


if __name__ == '__main__':
    main()
