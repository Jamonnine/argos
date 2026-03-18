"""
ARGOS Nav2 Goal 검증 스크립트
==============================
bt_navigator가 active 상태가 될 때까지 대기한 뒤
NavigateToPose goal을 발행하고 결과를 보고한다.

사용법:
  python test_nav2_goal.py
  python test_nav2_goal.py --x 5.0 --y 5.0
  python test_nav2_goal.py --x 2.0 --y 5.0 --timeout 180

indoor_test.sdf 맵 좌표 참조:
  방 A: (1.0~4.0, 0.5~4.5)  방 B: (1.0~4.0, 5.5~9.5)
  방 C: (7.0~10.0, 0.5~4.5) 복도: (5.0~6.0, 0.5~9.5)
  로봇 스폰: (2.0, 2.5)      기본 목표: (5.0, 5.0) — 방 B 입구

환경:
  ROS 2 Jazzy, Gazebo Harmonic, WSL2 Ubuntu 24.04
  RTF 0.28x 기준 wall time 타임아웃 적용
"""

import argparse
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry


class Nav2GoalTester(Node):
    def __init__(self, goal_x: float, goal_y: float, nav_timeout: float):
        super().__init__('nav2_goal_tester')

        self.goal_x = goal_x
        self.goal_y = goal_y
        self.nav_timeout = nav_timeout

        # bt_navigator lifecycle 상태 조회 클라이언트
        self._lifecycle_client = self.create_client(
            GetState,
            '/bt_navigator/get_state',
        )

        # NavigateToPose 액션 클라이언트
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
        )

        # 최종 odom 위치 저장용
        self._last_odom: Odometry | None = None
        self._odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self._odom_cb,
            10,
        )

    def _odom_cb(self, msg: Odometry) -> None:
        self._last_odom = msg

    # ------------------------------------------------------------------
    # 1. bt_navigator lifecycle 활성화 대기
    # ------------------------------------------------------------------
    def wait_for_bt_navigator(self, max_wait: float = 120.0) -> bool:
        """
        bt_navigator가 active(3) 상태가 될 때까지 폴링.
        max_wait 초 초과 시 False 반환.
        """
        self.get_logger().info(
            f'bt_navigator active 대기 중 (최대 {max_wait:.0f}초)...'
        )

        # lifecycle 서비스 연결 대기
        if not self._lifecycle_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                '/bt_navigator/get_state 서비스 미발견 — Nav2가 기동됐는지 확인하세요'
            )
            return False

        deadline = time.monotonic() + max_wait
        poll_interval = 3.0  # 초
        req = GetState.Request()

        while time.monotonic() < deadline:
            future = self._lifecycle_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

            if future.done() and future.result() is not None:
                state_id = future.result().current_state.id
                state_label = future.result().current_state.label
                self.get_logger().info(
                    f'  bt_navigator 상태: {state_label} (id={state_id})'
                )
                # active = 3 (lifecycle_msgs/State.PRIMARY_STATE_ACTIVE)
                if state_id == 3:
                    self.get_logger().info('bt_navigator active 확인.')
                    return True
            else:
                self.get_logger().warn('GetState 응답 없음 — 재시도')

            time.sleep(poll_interval)

        self.get_logger().error(
            f'bt_navigator가 {max_wait:.0f}초 내에 active 상태가 되지 않았습니다.'
        )
        return False

    # ------------------------------------------------------------------
    # 2. odom 현재 위치 출력
    # ------------------------------------------------------------------
    def _print_odom(self, label: str) -> None:
        # 최신 odom 수신 대기 (최대 3초)
        deadline = time.monotonic() + 3.0
        while self._last_odom is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self._last_odom is None:
            self.get_logger().warn(f'[{label}] odom 미수신')
            return

        pos = self._last_odom.pose.pose.position
        self.get_logger().info(
            f'[{label}] odom 위치 — x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f}'
        )

    # ------------------------------------------------------------------
    # 3. NavigateToPose goal 발행 및 결과 대기
    # ------------------------------------------------------------------
    def send_goal(self) -> bool:
        """
        goal 발행 후 결과 대기.
        성공 시 True, 실패/타임아웃 시 False 반환.
        """
        self.get_logger().info(
            f'navigate_to_pose 액션 서버 대기 중...'
        )
        if not self._action_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('navigate_to_pose 액션 서버 미발견')
            return False

        # goal 메시지 구성
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = self.goal_x
        goal_msg.pose.pose.position.y = self.goal_y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(
            f'Goal 전송: x={self.goal_x}, y={self.goal_y} | 타임아웃={self.nav_timeout:.0f}초'
        )

        # 전송 전 odom
        self._print_odom('전송 전')

        # goal 전송
        send_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_cb,
        )
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)

        if not send_future.done() or send_future.result() is None:
            self.get_logger().error('goal 전송 실패 (future 미완료)')
            return False

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('goal rejected — bt_navigator가 목표를 거절했습니다')
            return False

        self.get_logger().info('goal accepted. 결과 대기 중...')

        # 결과 대기
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=self.nav_timeout
        )

        if not result_future.done():
            self.get_logger().error(
                f'결과 타임아웃 ({self.nav_timeout:.0f}초) — RTF 0.28x 환경에서는 더 긴 타임아웃 필요'
            )
            return False

        status = result_future.result().status
        # action_msgs/GoalStatus: SUCCEEDED=4, ABORTED=6, CANCELED=5
        if status == 4:
            self.get_logger().info('Goal 도달 성공 (SUCCEEDED)')
            self._print_odom('도달 후')
            return True
        else:
            status_map = {5: 'CANCELED', 6: 'ABORTED'}
            label = status_map.get(status, f'UNKNOWN({status})')
            self.get_logger().error(f'Goal 실패: {label}')
            self._print_odom('실패 후')
            return False

    def _feedback_cb(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        dist = fb.distance_remaining
        self.get_logger().info(
            f'  [피드백] 남은 거리: {dist:.2f}m',
            throttle_duration_sec=5.0,
        )


# ----------------------------------------------------------------------
# 메인
# ----------------------------------------------------------------------
def main() -> None:
    parser = argparse.ArgumentParser(
        description='ARGOS Nav2 Goal 검증 스크립트'
    )
    parser.add_argument(
        '--x', type=float, default=5.0,
        help='목표 x 좌표 (기본값: 5.0 — 복도 중앙)'
    )
    parser.add_argument(
        '--y', type=float, default=5.0,
        help='목표 y 좌표 (기본값: 5.0 — 복도 중앙)'
    )
    parser.add_argument(
        '--wait-timeout', type=float, default=120.0,
        help='bt_navigator 활성화 대기 최대 시간(초) (기본값: 120)'
    )
    parser.add_argument(
        '--timeout', type=float, default=180.0,
        help='Goal 결과 대기 최대 시간(초) (기본값: 180 — RTF 0.28x 보정)'
    )
    # ROS 인자 분리 (ros2 run 경유 시 필요)
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = Nav2GoalTester(
        goal_x=args.x,
        goal_y=args.y,
        nav_timeout=args.timeout,
    )

    try:
        # 1단계: bt_navigator active 대기
        bt_active = node.wait_for_bt_navigator(max_wait=args.wait_timeout)
        if not bt_active:
            node.get_logger().error('bt_navigator 활성화 실패 — 종료')
            sys.exit(1)

        # 2단계: goal 발행 및 결과 확인
        success = node.send_goal()
        if success:
            node.get_logger().info('테스트 통과 — Nav2 Goal 이동 검증 완료')
            sys.exit(0)
        else:
            node.get_logger().error('테스트 실패 — Goal 미도달')
            sys.exit(2)

    except KeyboardInterrupt:
        node.get_logger().info('사용자 중단')
        sys.exit(130)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
