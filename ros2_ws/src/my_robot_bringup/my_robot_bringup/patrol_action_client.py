"""
ARGOS PatrolArea Action Client (오케스트레이터 시뮬레이터)
========================================================
지휘관 역할 — 로봇에 구역 수색 명령을 보내고,
진행 상황을 모니터링하며, 필요 시 철수 명령을 내립니다.

사용법:
  ros2 run my_robot_bringup patrol_client          # 일반 수색
  ros2 run my_robot_bringup patrol_client -- --cancel  # 3초 후 철수 명령
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from my_robot_interfaces.action import PatrolArea
from geometry_msgs.msg import PoseStamped

import sys
import time
import threading


class PatrolActionClient(Node):
    """
    ARGOS 오케스트레이터 시뮬레이터.

    소방 비유:
    - 이 노드 = 지휘관
    - send_goal = "3팀, 3층 수색 개시"
    - feedback_callback = 대원의 경과 보고 수신
    - cancel = "전원 철수!"
    """

    def __init__(self):
        super().__init__('patrol_action_client')

        self._action_client = ActionClient(
            self, PatrolArea, 'patrol_area'
        )
        self._goal_handle = None

    def send_patrol_goal(self, cancel_after_sec=0):
        """
        수색 명령 전송.
        cancel_after_sec > 0 이면, 해당 초 후 철수 명령.
        """
        self.get_logger().info('Action Server 연결 대기...')
        self._action_client.wait_for_server()

        # 시뮬레이션 경유지 생성 (5개 방)
        goal_msg = PatrolArea.Goal()
        goal_msg.timeout_sec = 60.0
        goal_msg.search_pattern = 'sequential'
        goal_msg.priority = 1  # 긴급 — 인명수색

        for i in range(5):
            wp = PoseStamped()
            wp.header.frame_id = 'map'
            wp.pose.position.x = float(i * 3)   # 3m 간격
            wp.pose.position.y = float(i % 2 * 2)  # 지그재그
            wp.pose.position.z = 0.0
            goal_msg.waypoints.append(wp)

        self.get_logger().info(
            f'수색 명령 전송: 경유지 {len(goal_msg.waypoints)}개, '
            f'우선순위 {goal_msg.priority} (긴급-인명수색)'
        )

        # Goal 전송 (비동기)
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

        # 철수 테스트 모드
        if cancel_after_sec > 0:
            threading.Timer(
                cancel_after_sec,
                self._send_cancel
            ).start()

    def goal_response_callback(self, future):
        """Goal 수락/거절 응답 처리."""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('수색 명령 거절됨!')
            return

        self.get_logger().info('수색 명령 수락됨 — 임무 진행 중')
        self._goal_handle = goal_handle

        # Result 대기
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """
        로봇의 경과 보고 수신.

        소방 비유: 무전기로 들어오는 대원의 보고.
        "현재 302호 진행 중, 진행률 40%, 배터리 80%"
        """
        fb = feedback_msg.feedback
        discoveries_str = ', '.join(fb.discoveries_so_far) if fb.discoveries_so_far else '없음'

        self.get_logger().info(
            f'[경과보고] 경유지 {fb.current_waypoint_index + 1} | '
            f'상태: {fb.status} | '
            f'진행률: {fb.progress:.0%} | '
            f'배터리: {fb.battery_percent:.0f}% | '
            f'발견: {discoveries_str}'
        )

    def result_callback(self, future):
        """
        최종 보고 수신.

        소방 비유: "수색 완료/철수 완료 보고"
        """
        result = future.result().result
        status_map = {
            'COMPLETED': '수색 완료',
            'CANCELLED': '철수 완료',
            'TIMEOUT': '시간 초과',
            'ABORTED': '임무 중단'
        }

        status_text = status_map.get(result.result_code, result.result_code)

        self.get_logger().info('=' * 50)
        self.get_logger().info(f'[최종보고] {status_text}')
        self.get_logger().info(f'  성공: {result.success}')
        self.get_logger().info(
            f'  경유지: {result.waypoints_visited}개 방문 '
            f'(커버리지 {result.coverage_percent:.0f}%)'
        )
        self.get_logger().info(f'  소요시간: {result.elapsed_sec:.1f}초')
        self.get_logger().info(f'  이동거리: {result.distance_traveled:.1f}m')

        if result.discoveries:
            self.get_logger().info(f'  발견사항 ({len(result.discoveries)}건):')
            for d in result.discoveries:
                self.get_logger().warn(f'    → {d}')
        else:
            self.get_logger().info('  발견사항: 없음')

        self.get_logger().info('=' * 50)

        # 완료 후 종료
        rclpy.shutdown()

    def _send_cancel(self):
        """철수 명령 전송."""
        if self._goal_handle:
            self.get_logger().warn('지휘관: 전원 철수!')
            self._goal_handle.cancel_goal_async()


def main(args=None):
    rclpy.init(args=args)
    node = PatrolActionClient()

    # --cancel 플래그 확인
    cancel_mode = '--cancel' in sys.argv
    if cancel_mode:
        node.get_logger().warn('철수 테스트 모드: 3초 후 철수 명령 발행')
        node.send_patrol_goal(cancel_after_sec=3)
    else:
        node.send_patrol_goal()

    rclpy.spin(node)


if __name__ == '__main__':
    main()
