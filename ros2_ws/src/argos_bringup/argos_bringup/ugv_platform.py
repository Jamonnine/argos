#!/usr/bin/env python3
"""ugv_platform.py — ARGOS UGV PlatformInterface 구현

기존 Nav2 NavigateToPose 기반 UGV 제어를 PlatformInterface로 래핑한다.
오케스트레이터는 이 클래스를 모르고 PlatformInterface만 안다.

아키텍처:
  Orchestrator
      ↓ PlatformInterface.move_to(x, y)
  UGVPlatform (이 파일)
      ↓ Nav2 NavigateToPose Action
  Nav2 (로컬 플래너 + 글로벌 플래너)
      ↓ cmd_vel (TwistStamped)
  diff_drive_controller → Gazebo UGV

TF2 경로: map → odom → base_footprint
  get_pose()는 map→base_footprint 변환을 조회한다.

배터리 모델:
  실 배터리 센서 없음 — 파라미터 기반 시뮬레이션.
  이동 거리와 경과 시간을 기반으로 소비량 추정.

ROS 파라미터:
  robot_id         (string)  : 로봇 식별자 (기본: "ugv1")
  home_x/home_y    (float)   : 귀환 좌표 (기본: 0.0, 0.0)
  battery_start    (float)   : 초기 배터리 % (기본: 100.0)
  battery_drain_rate (float) : % / 분 소비율 (기본: 1.0)
  nav_timeout      (float)   : Nav2 Goal 타임아웃 초 (기본: 60.0)
  map_frame        (string)  : TF 맵 프레임 (기본: "map")
  base_frame       (string)  : TF 베이스 프레임 (기본: "base_footprint")
"""

import math
import threading
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener
import tf2_ros

from argos_bringup.platform_interface import PlatformInterface, RobotCapabilities


class UGVPlatform(PlatformInterface):
    """UGV 플랫폼 — Nav2 기반 PlatformInterface 구현체.

    멀티로봇 환경에서는 네임스페이스(ugv1, ugv2)를 통해 격리.
    Nav2 액션 서버도 해당 네임스페이스 아래에 위치해야 한다.
    예) ugv1/navigate_to_pose

    사용 예시:
      node = rclpy.create_node('ugv_wrapper')
      ugv = UGVPlatform(node, robot_id='ugv1')
      ugv.move_to(5.0, 3.0)
    """

    def __init__(self, node: Node, robot_id: str = 'ugv1'):
        """UGVPlatform 초기화.

        Args:
            node: 이 플랫폼이 귀속될 ROS 2 노드.
                  UGV 전용 노드이거나 오케스트레이터 노드 공유 가능.
            robot_id: 로봇 식별자. 네임스페이스 구성에 사용.
        """
        self._node = node
        self._robot_id = robot_id

        # ── 파라미터 (선언 + 읽기) ────────────────────────────────────────────
        node.declare_parameter(f'{robot_id}.home_x', 0.0)
        node.declare_parameter(f'{robot_id}.home_y', 0.0)
        node.declare_parameter(f'{robot_id}.battery_start', 100.0)
        node.declare_parameter(f'{robot_id}.battery_drain_rate', 1.0)  # % / 분
        node.declare_parameter(f'{robot_id}.nav_timeout', 60.0)
        node.declare_parameter(f'{robot_id}.map_frame', 'map')
        node.declare_parameter(f'{robot_id}.base_frame', 'base_footprint')

        self._home_x = node.get_parameter(f'{robot_id}.home_x').value
        self._home_y = node.get_parameter(f'{robot_id}.home_y').value
        self._battery = node.get_parameter(f'{robot_id}.battery_start').value
        self._drain_rate = node.get_parameter(f'{robot_id}.battery_drain_rate').value
        self._nav_timeout = node.get_parameter(f'{robot_id}.nav_timeout').value
        self._map_frame = node.get_parameter(f'{robot_id}.map_frame').value
        self._base_frame = node.get_parameter(f'{robot_id}.base_frame').value

        # ── 내부 상태 ─────────────────────────────────────────────────────────
        self._lock = threading.Lock()
        self._battery_last_update = time.time()
        self._navigating = False
        self._goal_handle: Optional[object] = None

        # ── TF2 (맵 프레임에서 로봇 위치 조회) ──────────────────────────────
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node)

        # ── Nav2 액션 클라이언트 ──────────────────────────────────────────────
        # 멀티로봇: 네임스페이스 접두사 적용 (ugv1/navigate_to_pose)
        nav_action = f'{robot_id}/navigate_to_pose'
        self._cb_group = ReentrantCallbackGroup()
        self._nav_client = ActionClient(
            node,
            NavigateToPose,
            nav_action,
            callback_group=self._cb_group,
        )

        # ── 긴급 정지용 cmd_vel 퍼블리셔 ────────────────────────────────────
        # emergency_stop 시 직접 cmd_vel=0 발행 (Nav2 취소와 병행)
        # Jazzy: diff_drive_controller는 TwistStamped만 수락 (ros2.md ★★★)
        self._cmd_pub = node.create_publisher(
            TwistStamped,
            f'{robot_id}/cmd_vel',
            10,
        )

        # ── 배터리 소비 타이머 (10초 주기) ───────────────────────────────────
        node.create_timer(10.0, self._update_battery)

        node.get_logger().info(
            f'[UGVPlatform:{robot_id}] 초기화 완료 '
            f'(home=({self._home_x:.1f},{self._home_y:.1f}), '
            f'battery={self._battery:.0f}%, '
            f'nav_action={nav_action})')

    # ──────────────────────────────────────────────────────────────────────────
    # PlatformInterface 구현
    # ──────────────────────────────────────────────────────────────────────────

    def move_to(self, x: float, y: float, z: float = 0.0) -> bool:
        """Nav2 NavigateToPose 액션으로 목표 좌표 이동.

        UGV는 평면 이동이므로 z 파라미터는 무시된다.
        기존 이동 중이면 취소 후 새 목표로 이동.

        Args:
            x: 목표 x (m, map 프레임 ENU)
            y: 목표 y (m, map 프레임 ENU)
            z: 무시됨 (UGV 평면 이동)

        Returns:
            True: 액션 서버에 Goal 전송 성공
            False: 서버 미연결 또는 배터리 부족
        """
        # 배터리 임계값 (10% 이하 이동 금지 — 귀환 불가 방지)
        if self._battery < 10.0:
            self._node.get_logger().warn(
                f'[{self._robot_id}] 배터리 부족 ({self._battery:.1f}%) — 이동 거부')
            return False

        # Nav2 서버 연결 확인 (논블로킹)
        if not self._nav_client.server_is_ready():
            self._node.get_logger().warn(
                f'[{self._robot_id}] Nav2 서버({self._robot_id}/navigate_to_pose) 미연결')
            return False

        # Goal 메시지 구성
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self._map_frame
        goal.pose.header.stamp = self._node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.position.z = 0.0
        # 기본 방향: 단위 쿼터니언 (w=1.0) — 도달 후 방향은 Nav2가 결정
        goal.pose.pose.orientation.w = 1.0

        self._node.get_logger().info(
            f'[{self._robot_id}] move_to({x:.2f}, {y:.2f}) 전송')

        # 비동기 전송 (콜백으로 결과 처리)
        future = self._nav_client.send_goal_async(
            goal,
            feedback_callback=self._nav_feedback_callback,
        )
        future.add_done_callback(self._nav_goal_response_callback)

        with self._lock:
            self._navigating = True

        return True

    def return_home(self) -> bool:
        """홈 좌표로 귀환.

        파라미터 home_x, home_y로 정의된 출발점으로 복귀.
        배터리 부족 경고 우선, 이동은 시도.

        Returns:
            True: 귀환 명령 전송 성공
            False: 서버 미연결
        """
        if self._battery < 5.0:
            self._node.get_logger().error(
                f'[{self._robot_id}] 배터리 위험 ({self._battery:.1f}%) — '
                '귀환 시도하지만 도달 불보장')

        self._node.get_logger().info(
            f'[{self._robot_id}] 귀환 → home({self._home_x:.1f}, {self._home_y:.1f})')
        return self.move_to(self._home_x, self._home_y, 0.0)

    def get_pose(self) -> tuple[float, float, float]:
        """TF2로 map → base_footprint 변환 조회.

        멀티로봇 환경: base_footprint는 네임스페이스 접두사 포함
        (예: ugv1/base_footprint). frame_prefix는 URDF에서 설정.

        Returns:
            (x, y, z) 튜플 (m, map 프레임 ENU)
            조회 실패 시 (0.0, 0.0, 0.0) 반환
        """
        # TF 프레임: 네임스페이스가 있으면 접두사 포함
        # argos_description xacro의 frame_prefix 파라미터와 일치해야 함
        base_frame = self._base_frame
        if self._robot_id and not base_frame.startswith(self._robot_id):
            base_frame = f'{self._robot_id}/{self._base_frame}'

        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame,
                base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
            t = tf.transform.translation
            return (t.x, t.y, t.z)
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self._node.get_logger().debug(
                f'[{self._robot_id}] TF 조회 실패 ({base_frame}): {e}',
                throttle_duration_sec=5.0,
            )
            return (0.0, 0.0, 0.0)

    def get_battery(self) -> float:
        """배터리 잔량 반환 (파라미터 기반 시뮬레이션).

        실 배터리 센서 없음 — 시간 경과에 따른 소비율로 추정.
        battery_drain_rate: % / 분 (기본 1.0%)

        Returns:
            배터리 잔량 (%, 0.0 ~ 100.0)
        """
        with self._lock:
            return max(0.0, round(self._battery, 1))

    def get_capabilities(self) -> RobotCapabilities:
        """UGV 능력 명세 반환.

        ARGOS UGV (argos_description) 하드웨어 구성 기반:
          - 지상 주행 가능 (diff drive)
          - LiDAR 탑재 (slam_toolbox 입력)
          - 열화상 탑재 (hotspot_detector_node 입력)
          - 비행 불가

        Returns:
            UGV 고정 능력 명세 (캐시됨)
        """
        return RobotCapabilities(
            can_fly=False,
            can_drive=True,
            has_thermal=True,
            has_lidar=True,
            max_speed=0.5,       # diff_drive_controller 최대 속도 (m/s)
            battery_capacity=self._battery,
            platform_type='ugv',
        )

    def emergency_stop(self) -> None:
        """긴급 정지 — Nav2 취소 + cmd_vel=0 즉시 발행.

        두 채널 동시 실행:
          1) Nav2 Goal 취소 (경로 계획 중단)
          2) cmd_vel TwistStamped 제로 발행 (컨트롤러 즉시 정지)

        예외 발생 시에도 계속 진행 (오케스트레이터 루프 보호).
        """
        self._node.get_logger().error(
            f'[{self._robot_id}] EMERGENCY STOP — 전체 이동 중단')

        # 채널 1: Nav2 현재 Goal 취소
        try:
            with self._lock:
                handle = self._goal_handle
            if handle is not None:
                handle.cancel_goal_async()
        except Exception as e:
            self._node.get_logger().error(
                f'[{self._robot_id}] Nav2 취소 실패: {e}')

        # 채널 2: cmd_vel=0 직접 발행 (TwistStamped — Jazzy 표준)
        try:
            stop_msg = TwistStamped()
            stop_msg.header.stamp = self._node.get_clock().now().to_msg()
            stop_msg.header.frame_id = 'base_link'
            # 모든 속도 필드 기본값 0.0 — 명시적으로 확인
            stop_msg.twist.linear.x = 0.0
            stop_msg.twist.linear.y = 0.0
            stop_msg.twist.linear.z = 0.0
            stop_msg.twist.angular.x = 0.0
            stop_msg.twist.angular.y = 0.0
            stop_msg.twist.angular.z = 0.0
            self._cmd_pub.publish(stop_msg)
        except Exception as e:
            self._node.get_logger().error(
                f'[{self._robot_id}] cmd_vel 제로 발행 실패: {e}')

        with self._lock:
            self._navigating = False

    # ──────────────────────────────────────────────────────────────────────────
    # 내부 메서드 (Nav2 콜백 + 배터리 시뮬레이션)
    # ──────────────────────────────────────────────────────────────────────────

    def _nav_goal_response_callback(self, future):
        """Nav2 Goal 수락/거절 응답 처리."""
        try:
            handle = future.result()
        except Exception as e:
            self._node.get_logger().error(
                f'[{self._robot_id}] Goal 응답 오류: {e}')
            with self._lock:
                self._navigating = False
            return

        if not handle.accepted:
            self._node.get_logger().warn(
                f'[{self._robot_id}] Nav2 Goal 거절됨')
            with self._lock:
                self._navigating = False
            return

        self._node.get_logger().info(
            f'[{self._robot_id}] Nav2 Goal 수락됨')
        with self._lock:
            self._goal_handle = handle

        # 결과 대기 콜백 등록
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._nav_result_callback)

    def _nav_feedback_callback(self, feedback_msg):
        """Nav2 피드백 수신 — 현재 위치 로그 (debug 레벨)."""
        fb = feedback_msg.feedback
        pose = fb.current_pose.pose.position
        self._node.get_logger().debug(
            f'[{self._robot_id}] Nav2 진행 중: '
            f'({pose.x:.2f}, {pose.y:.2f})',
            throttle_duration_sec=3.0,
        )

    def _nav_result_callback(self, future):
        """Nav2 결과 수신 — 이동 완료 또는 실패 처리."""
        try:
            result = future.result()
        except Exception as e:
            self._node.get_logger().error(
                f'[{self._robot_id}] Nav2 결과 수신 오류: {e}')
            with self._lock:
                self._navigating = False
            return

        from action_msgs.msg import GoalStatus
        status = result.status
        with self._lock:
            self._navigating = False
            self._goal_handle = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            self._node.get_logger().info(
                f'[{self._robot_id}] 목표 도달 성공')
        else:
            self._node.get_logger().warn(
                f'[{self._robot_id}] Nav2 이동 실패 (status={status})')

    def _update_battery(self):
        """배터리 소비 시뮬레이션 (10초 주기 타이머 콜백).

        이동 중: drain_rate × (주기 분) 소비
        정지 중: drain_rate × 0.3 소비 (대기 전력)
        """
        now = time.time()
        elapsed_min = (now - self._battery_last_update) / 60.0

        with self._lock:
            drain_factor = 1.0 if self._navigating else 0.3
            drain = self._drain_rate * elapsed_min * drain_factor
            self._battery = max(0.0, self._battery - drain)
            self._battery_last_update = now

        if self._battery < 20.0:
            self._node.get_logger().warn(
                f'[{self._robot_id}] 배터리 부족 경고: {self._battery:.1f}%',
                throttle_duration_sec=30.0,
            )


def main(args=None):
    """UGVPlatform 단독 실행 테스트 노드."""
    rclpy.init(args=args)
    node = rclpy.create_node('ugv_platform_test')
    ugv = UGVPlatform(node, robot_id='ugv1')

    node.get_logger().info('UGVPlatform 테스트 노드 시작')
    node.get_logger().info(f'capabilities: {ugv.get_capabilities()}')
    node.get_logger().info(f'battery: {ugv.get_battery()}%')
    node.get_logger().info(f'pose: {ugv.get_pose()}')

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
