"""
ARGOS PatrolArea Action Server
==============================
오케스트레이터의 구역 수색 명령을 실행하는 Action Server.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from my_robot_interfaces.action import PatrolArea
from geometry_msgs.msg import PoseStamped

import time
import random


class PatrolActionServer(Node):

    def __init__(self):
        super().__init__('patrol_action_server')
        self._action_server = ActionServer(
            self,
            PatrolArea,
            'patrol_area',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )
        self.get_logger().info('PatrolArea Action Server 시작 — 수색 명령 대기 중')

    def goal_callback(self, goal_request):
        waypoint_count = len(goal_request.waypoints)
        self.get_logger().info(
            f'수색 명령 수신: 경유지 {waypoint_count}개, '
            f'우선순위 {goal_request.priority}, 제한시간 {goal_request.timeout_sec:.0f}초'
        )
        if waypoint_count == 0:
            self.get_logger().warn('경유지 없음 — 명령 거절')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().warn('철수 명령 수신 — 수색 중단!')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """동기 execute — MultiThreadedExecutor에서 cancel은 별도 스레드 처리."""
        self.get_logger().info('수색 임무 개시!')

        waypoints = goal_handle.request.waypoints
        timeout = goal_handle.request.timeout_sec
        total = len(waypoints)

        discoveries = []
        distance = 0.0
        start_time = time.time()
        visited = 0
        feedback = PatrolArea.Feedback()

        for i, wp in enumerate(waypoints):
            # 취소 확인
            if goal_handle.is_cancel_requested:
                result = self._result(False, 'CANCELLED', visited, total, discoveries, distance, start_time)
                goal_handle.canceled(result)
                self.get_logger().warn(f'철수 완료 — {visited}/{total} 경유지 수색 후 중단')
                return result

            # 타임아웃 확인
            if timeout > 0 and (time.time() - start_time) > timeout:
                result = self._result(False, 'TIMEOUT', visited, total, discoveries, distance, start_time)
                goal_handle.abort(result)
                self.get_logger().warn(f'제한시간 {timeout:.0f}초 초과')
                return result

            # 이동
            self.get_logger().info(f'경유지 {i+1}/{total} 이동 중 ({wp.pose.position.x:.1f}, {wp.pose.position.y:.1f})')
            feedback.current_pose = wp
            feedback.current_waypoint_index = i
            feedback.progress = float(i) / total
            feedback.status = 'MOVING'
            feedback.battery_percent = max(10.0, 95.0 - (i * 5.0))
            feedback.discoveries_so_far = list(discoveries)
            goal_handle.publish_feedback(feedback)
            time.sleep(1.0)

            # 이동 중 취소 재확인
            if goal_handle.is_cancel_requested:
                result = self._result(False, 'CANCELLED', visited, total, discoveries, distance, start_time)
                goal_handle.canceled(result)
                return result

            # 스캔
            feedback.status = 'SCANNING'
            feedback.progress = (float(i) + 0.5) / total
            goal_handle.publish_feedback(feedback)
            time.sleep(1.0)

            # 발견 시뮬레이션 (30%)
            if random.random() < 0.3:
                dtype = random.choice(['THERMAL', 'VICTIM', 'HAZARD'])
                discovery = f'{dtype}:ROOM-{i+1:03d}'
                discoveries.append(discovery)
                self.get_logger().warn(f'발견! {discovery}')

            visited += 1
            distance += random.uniform(3.0, 8.0)

        # 완료
        result = self._result(True, 'COMPLETED', visited, total, discoveries, distance, start_time)
        goal_handle.succeed(result)
        self.get_logger().info(f'수색 완료! {visited}개 경유지, {len(discoveries)}건 발견')
        return result

    def _result(self, success, code, visited, total, discoveries, distance, start_time):
        r = PatrolArea.Result()
        r.success = success
        r.result_code = code
        r.waypoints_visited = visited
        r.coverage_percent = (visited / total * 100.0) if total > 0 else 0.0
        r.elapsed_sec = float(time.time() - start_time)
        r.distance_traveled = distance
        r.discoveries = list(discoveries)
        return r


def main(args=None):
    rclpy.init(args=args)
    node = PatrolActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
