#!/usr/bin/env python3
"""
Simulated Arm Controller - Action Server (Day 13)

Day 12에서는 Topic 기반으로 구현했습니다.
Day 13에서는 ROS 2 Action Server로 변환합니다.

실제 MoveIt의 move_group 노드가 이 구조로 동작합니다:
    - Goal 수신: 이동할 목표 자세
    - Feedback 발행: 진행률 + 현재 위치 (0.1초마다)
    - Result 반환: 성공/실패 + 최종 위치

Action Server vs Topic의 차이:
    Topic:  Fire-and-forget. 발행하면 끝, 결과 확인 불가.
    Action: 양방향. Goal → Feedback... → Result 전체 흐름 추적 가능.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Point
from my_robot_interfaces.action import MoveToTarget
import numpy as np
import time


class ArmControllerSim(Node):

    def __init__(self):
        super().__init__('arm_controller_sim')

        # ReentrantCallbackGroup: 여러 콜백이 동시에 실행 가능하게 함
        # Action Server에서 필수 — goal 처리 중에도 cancel 요청을 받아야 하므로
        callback_group = ReentrantCallbackGroup()

        # Action Server 생성
        # 이름 'move_to_target': 클라이언트는 이 이름으로 서버를 찾습니다
        self._action_server = ActionServer(
            self,
            MoveToTarget,                    # Action 타입
            'move_to_target',                # Action 이름
            execute_callback=self.execute_goal,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=callback_group,
        )

        # 팔의 현재 위치 (base_link 기준)
        self.current_pose = PoseStamped()
        self.current_pose.header.frame_id = 'base_link'
        self.current_pose.pose.position.x = 0.3
        self.current_pose.pose.position.y = 0.0
        self.current_pose.pose.position.z = 0.5
        self.current_pose.pose.orientation.w = 1.0

        # 현재 상태 표시용
        self.current_state = 'IDLE'

        # RViz 시각화용 발행자
        self.pub_viz = self.create_publisher(MarkerArray, '/arm_visualization', 10)
        self.pub_status = self.create_publisher(String, '/arm_status', 10)

        # 10Hz로 RViz 마커 갱신 (goal 없을 때도 팔 위치 표시)
        self.viz_timer = self.create_timer(0.1, self.publish_visualization)

        self.get_logger().info(
            'Arm Controller (Action Server) initialized\n'
            '  Action: /move_to_target\n'
            '  Type:   my_robot_interfaces/action/MoveToTarget\n'
            '  Initial pose: (0.3, 0.0, 0.5) in base_link'
        )

    def goal_callback(self, goal_request):
        """
        새 Goal 수신 시 수락 여부를 결정합니다.

        실제 MoveIt에서는 여기서 목표 자세가 팔의 도달 범위(workspace) 안인지,
        관절 한계(joint limits)를 위반하지 않는지 사전 검사합니다.
        """
        self.get_logger().info('Goal received — accepted')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """
        Cancel 요청 수신 시 처리합니다.
        Topic/Service로는 불가능했던 '중간에 취소' 기능입니다.
        """
        self.get_logger().info('Cancel requested — accepted')
        return CancelResponse.ACCEPT

    async def execute_goal(self, goal_handle):
        """
        Goal 실행 — 이 함수가 실제 팔 이동을 시뮬레이션합니다.
        async 함수: Python 비동기 함수. 중간에 다른 작업(cancel 확인 등)이
        끼어들 수 있도록 합니다.
        """
        target = goal_handle.request.target_pose
        self.get_logger().info(
            f'Executing goal: '
            f'({target.pose.position.x:.2f}, '
            f'{target.pose.position.y:.2f}, '
            f'{target.pose.position.z:.2f})'
        )

        feedback_msg = MoveToTarget.Feedback()

        # ── 1단계: PLANNING ──────────────────────────────
        self.current_state = 'PLANNING'
        PLAN_TIME = 0.8  # 계획에 걸리는 시간 (실제 OMPL은 수백ms)

        plan_start = time.time()
        while time.time() - plan_start < PLAN_TIME:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return MoveToTarget.Result(success=False, message='Canceled during planning')

            feedback_msg.progress = 0.0
            feedback_msg.current_state = 'PLANNING'
            feedback_msg.current_pose = self.current_pose
            feedback_msg.distance_remaining = self._calc_distance(
                self.current_pose, target)
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)

        # ── 2단계: MOVING ────────────────────────────────
        self.current_state = 'MOVING'
        MOVE_TIME = 3.0

        start_pos = np.array([
            self.current_pose.pose.position.x,
            self.current_pose.pose.position.y,
            self.current_pose.pose.position.z,
        ])
        target_pos = np.array([
            target.pose.position.x,
            target.pose.position.y,
            target.pose.position.z,
        ])

        move_start = time.time()
        while True:
            elapsed = time.time() - move_start
            t = min(elapsed / MOVE_TIME, 1.0)

            # 취소 확인 (이게 Action의 핵심 장점)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.current_state = 'IDLE'
                result = MoveToTarget.Result()
                result.success = False
                result.message = 'Canceled during movement'
                result.final_pose = self.current_pose
                return result

            # Smooth Step 보간
            t_s = t * t * (3.0 - 2.0 * t)
            current_pos = start_pos + t_s * (target_pos - start_pos)

            self.current_pose.header.stamp = self.get_clock().now().to_msg()
            self.current_pose.pose.position.x = float(current_pos[0])
            self.current_pose.pose.position.y = float(current_pos[1])
            self.current_pose.pose.position.z = float(current_pos[2])
            self.current_pose.pose.orientation = target.pose.orientation

            # Feedback 발행 (0.1초마다)
            feedback_msg.progress = t
            feedback_msg.current_state = 'MOVING'
            feedback_msg.current_pose = self.current_pose
            feedback_msg.distance_remaining = float(
                np.linalg.norm(target_pos - current_pos))
            goal_handle.publish_feedback(feedback_msg)

            if t >= 1.0:
                break
            time.sleep(0.1)

        # ── 3단계: REACHED ───────────────────────────────
        self.current_state = 'REACHED'
        self.get_logger().info('Target REACHED!')

        goal_handle.succeed()

        result = MoveToTarget.Result()
        result.success = True
        result.message = 'Successfully reached target pose'
        result.final_pose = self.current_pose
        return result

    def _calc_distance(self, pose_a: PoseStamped, pose_b: PoseStamped) -> float:
        dx = pose_a.pose.position.x - pose_b.pose.position.x
        dy = pose_a.pose.position.y - pose_b.pose.position.y
        dz = pose_a.pose.position.z - pose_b.pose.position.z
        return float(np.sqrt(dx**2 + dy**2 + dz**2))

    def publish_visualization(self):
        """RViz2 마커 갱신 (10Hz)"""
        STATE_COLORS = {
            'IDLE':     (0.5, 0.5, 0.5),
            'PLANNING': (1.0, 1.0, 0.0),
            'MOVING':   (0.0, 1.0, 0.0),
            'REACHED':  (0.3, 0.7, 1.0),
        }
        r, g, b = STATE_COLORS.get(self.current_state, (1.0, 1.0, 1.0))

        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        sphere = Marker()
        sphere.header.frame_id = 'base_link'
        sphere.header.stamp = stamp
        sphere.ns = 'arm_ee'
        sphere.id = 0
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose = self.current_pose.pose
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.08
        sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = r, g, b, 1.0

        text = Marker()
        text.header.frame_id = 'base_link'
        text.header.stamp = stamp
        text.ns = 'arm_status_text'
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose = self.current_pose.pose
        text.pose.position.z = self.current_pose.pose.position.z + 0.12
        text.scale.z = 0.07
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = f'ARM [{self.current_state}]'

        markers.markers.extend([sphere, text])
        self.pub_viz.publish(markers)

        status_msg = String()
        status_msg.data = self.current_state
        self.pub_status.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ArmControllerSim()
        # MultiThreadedExecutor: Action Server의 비동기 실행을 위해 필수
        # SingleThreadedExecutor로는 goal 실행 중 cancel을 처리 못함
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
