#!/usr/bin/env python3
"""
Vision-to-MoveIt Bridge — Action Client (Day 13)

Day 12에서는 PoseStamped Topic을 발행했습니다.
Day 13에서는 Action Client로 변환하여 실제 MoveIt과 동일한 인터페이스를 사용합니다.

Action Client의 장점:
    1. 목표 전송 후 피드백(진행률) 실시간 수신 가능
    2. 팔이 목표에 도달했는지 Result로 확인 가능
    3. 필요 시 cancel_goal()로 중간 취소 가능
    4. 이것이 실제 MoveIt MoveGroupInterface의 동작 방식
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import tf2_ros
import tf2_geometry_msgs  # noqa: F401
import numpy as np
from argos_interfaces.action import MoveToTarget


class VisionMoveItBridge(Node):

    def __init__(self):
        super().__init__('vision_moveit_bridge')

        # TF2: 좌표계 변환 시스템
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Action Client: 'move_to_target' 액션 서버와 연결
        self._action_client = ActionClient(self, MoveToTarget, 'move_to_target')

        # Subscribe: 감지된 물체
        self.sub_clusters = self.create_subscription(
            MarkerArray, '/cluster_markers', self.on_cluster, 10)

        # Publish: 시각화 마커
        self.pub_grasp_marker = self.create_publisher(Marker, '/grasp_target_marker', 10)
        self.pub_status = self.create_publisher(String, '/bridge_status', 10)

        # 상태 관리
        self.is_executing = False   # 현재 goal 실행 중 여부
        self.frame_count = 0

        self.get_logger().info(
            'Vision-MoveIt Bridge (Action Client) initialized\n'
            '  Watching:  /cluster_markers\n'
            '  Connects:  move_to_target (Action Server)\n'
            '  Role: Perception → Action Client bridge'
        )

    def on_cluster(self, msg: MarkerArray):
        """
        클러스터 감지 시 Action Goal 전송.

        이미 팔이 이동 중이면(is_executing=True) 새 goal을 보내지 않습니다.
        실제 MoveIt에서도 이전 goal이 완료되기 전까지는 새 goal을 큐에 쌓거나 무시합니다.
        """
        if self.is_executing:
            return  # 이미 이동 중 — 새 goal 무시

        box_markers = [m for m in msg.markers if m.type == Marker.CUBE]
        if not box_markers:
            return

        # 가장 가까운 물체 선택
        closest = min(
            box_markers,
            key=lambda m: m.pose.position.x**2 + m.pose.position.y**2 + m.pose.position.z**2
        )

        # 파지 목표 계산 (물체 위 10cm)
        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = closest.header.frame_id
        target.pose.position.x = closest.pose.position.x
        target.pose.position.y = closest.pose.position.y
        target.pose.position.z = closest.pose.position.z + 0.10
        target.pose.orientation.y = 0.707
        target.pose.orientation.w = 0.707

        # TF2 변환: camera_frame → base_link
        try:
            target_in_base = self.tf_buffer.transform(target, 'base_link')
        except Exception as e:
            self.get_logger().warn(f'TF transform failed: {e}')
            return

        self.publish_grasp_marker(target_in_base)
        self.send_goal(target_in_base)

    def send_goal(self, target_pose: PoseStamped):
        """
        Action Server에 Goal을 비동기로 전송합니다.

        send_goal_async: 응답을 기다리지 않고 즉시 반환 (비동기).
        add_done_callback: 서버가 goal을 수락/거부하면 호출될 콜백 등록.
        """
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Action server not available')
            return

        goal_msg = MoveToTarget.Goal()
        goal_msg.target_pose = target_pose

        self.is_executing = True
        self.get_logger().info(
            f'Sending goal: ({target_pose.pose.position.x:.2f}, '
            f'{target_pose.pose.position.y:.2f}, '
            f'{target_pose.pose.position.z:.2f})'
        )

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.on_feedback  # 피드백 수신 콜백
        )
        send_goal_future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        """
        서버가 goal을 수락/거부했을 때 호출되는 콜백.
        Goal이 수락되면 result 콜백을 등록합니다.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by server')
            self.is_executing = False
            return

        self.get_logger().info('Goal accepted — waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_result)

    def on_feedback(self, feedback_msg):
        """
        서버가 보내는 피드백 수신 (0.1초마다).
        Topic 기반으로는 절대 받을 수 없었던 실시간 진행 상황입니다.
        """
        fb = feedback_msg.feedback
        progress_pct = int(fb.progress * 100)

        status = String()
        status.data = (
            f'[{fb.current_state}] {progress_pct}% | '
            f'({fb.current_pose.pose.position.x:.2f}, '
            f'{fb.current_pose.pose.position.y:.2f}, '
            f'{fb.current_pose.pose.position.z:.2f}) | '
            f'dist: {fb.distance_remaining:.2f}m'
        )
        self.pub_status.publish(status)

        self.frame_count += 1
        if self.frame_count % 10 == 0:
            self.get_logger().info(f'Feedback: {status.data}')

    def on_result(self, future):
        """
        Goal 실행 완료 후 최종 Result 수신.
        Topic 기반으로는 "성공했는지" 알 방법이 없었습니다.
        """
        result = future.result().result
        self.is_executing = False

        if result.success:
            self.get_logger().info(f'SUCCESS: {result.message}')
        else:
            self.get_logger().warn(f'FAILED: {result.message}')

        # 다음 Goal을 위해 초기화
        self.frame_count = 0

    def publish_grasp_marker(self, pose_stamped: PoseStamped):
        marker = Marker()
        marker.header = pose_stamped.header
        marker.ns = 'grasp_target'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose_stamped.pose
        marker.scale.x = marker.scale.y = marker.scale.z = 0.06
        marker.color.r = 1.0
        marker.color.g = 0.84
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime.sec = 2
        self.pub_grasp_marker.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = VisionMoveItBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
