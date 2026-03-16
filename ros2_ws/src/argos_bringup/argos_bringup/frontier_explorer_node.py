"""
ARGOS Frontier Explorer Node
=============================
OccupancyGrid에서 프론티어(미탐색 경계)를 감지하고,
Nav2 NavigateToPose 액션으로 자율 탐색을 수행한다.

프론티어(Frontier)란?
  알려진 자유 공간(free)과 미지 공간(unknown)의 경계.
  "아직 안 가본 곳의 입구" — 여기로 이동하면 새 영역이 스캔됨.

알고리즘:
  1. OccupancyGrid에서 free-unknown 경계 셀 추출
  2. Connected components로 프론티어 클러스터링
  3. 크기 필터링 → 유효 프론티어만 선별
  4. TF2로 로봇 현재 위치 조회
  5. 거리 기반 최적 프론티어 선택 (다른 로봇 대상 회피)
  6. NavigateToPose 액션으로 이동
  7. 도착/실패 → 블랙리스트 추가 → 다음 프론티어 탐색
  8. 프론티어 없음 → 탐색 완료 선언

멀티로봇 지원:
  /exploration/targets 토픽으로 각 로봇의 현재 대상을 공유.
  다른 로봇이 향하는 프론티어는 exclusion_radius로 회피.

열화상 연동:
  high/critical 감지 시 탐색 일시정지.
  exploration/resume 서비스로 재개.

토픽:
  구독: /map (nav_msgs/OccupancyGrid)
        /thermal/detections (ThermalDetection)
        /exploration/targets (PoseStamped, 다른 로봇 대상)
  발행: /exploration/targets (PoseStamped, 내 대상)
        exploration/status (String)
        exploration/frontiers_viz (MarkerArray, RViz 시각화)
  서비스: exploration/resume (Trigger)
  액션: navigate_to_pose (NavigateToPose, Nav2)

LifecycleNode 상태:
  unconfigured → configure() → inactive
  inactive     → activate()  → active (구독/발행/타이머/ActionClient/TF2 활성)
  active       → deactivate() → inactive (타이머·토픽·ActionClient 중단)
  inactive     → cleanup()   → unconfigured
"""

import math
import threading
from collections import deque
import numpy as np
import cv2

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, State
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, ColorRGBA, UInt32
from std_srvs.srv import Trigger
from nav2_msgs.action import NavigateToPose
from argos_interfaces.msg import ThermalDetection

from tf2_ros import Buffer, TransformListener


class FrontierExplorer(LifecycleNode):

    def __init__(self):
        super().__init__('frontier_explorer')

        # ── 파라미터 선언만 (activate 전에는 토픽·타이머·ActionClient·TF2 생성 금지) ──
        # Nav 전문가 권고: 해상도 5cm일 때 8셀=0.2m². 방화문(1.2m) 감지에 부족.
        # 20셀=0.5m²으로 상향. 탐색 초기에는 작은 프론티어도 허용(동적 조정 예정).
        self.declare_parameter('min_frontier_size', 20)
        self.declare_parameter('exclusion_radius', 2.0)
        self.declare_parameter('blacklist_radius', 1.0)
        self.declare_parameter('exploration_rate', 1.0)
        self.declare_parameter('robot_name', '')
        self.declare_parameter('thermal_pause', True)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')

        # 런타임 상태 (on_configure에서 초기화)
        self.min_frontier_size = None
        self.exclusion_radius = None
        self.blacklist_radius = None
        self.robot_name = None
        self.map_frame = None
        self.base_frame = None
        self.thermal_pause_enabled = None
        self.current_map = None
        self.blacklisted = None
        self.other_targets = {}
        self.current_goal = None
        self.navigating = False
        self.paused = False
        self.exploration_complete = False
        self.nav_goal_handle = None
        self.no_frontier_count = 0
        self.nav_server_ready = False
        self._nav_lock = None
        self._goal_cancelled = False
        self.nav_error_count = 0

        # 구독/발행/타이머/ActionClient/TF2 핸들
        # (on_activate에서 생성, on_deactivate에서 해제)
        self.map_sub = None
        self.thermal_sub = None
        self.peers_sub = None
        self.target_pub = None
        self.status_pub = None
        self.frontier_viz_pub = None
        self.frontier_count_pub = None
        self.nav_error_pub = None
        self.resume_srv = None
        self.nav_client = None
        self.server_check_timer = None
        self.explore_timer = None
        self.tf_buffer = None
        self.tf_listener = None
        self._cb_group = None

    # ─────────────────── Lifecycle Callbacks ───────────────────

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """파라미터 읽기 + 내부 상태 초기화.

        inactive 상태로 진입 전 호출됨. 여기서 ROS 통신은 만들지 않는다.
        """
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.exclusion_radius = self.get_parameter('exclusion_radius').value
        self.blacklist_radius = self.get_parameter('blacklist_radius').value
        self.robot_name = self.get_parameter('robot_name').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.thermal_pause_enabled = self.get_parameter('thermal_pause').value

        # 상태 초기화
        self.current_map = None
        self.blacklisted = deque(maxlen=200)
        self.other_targets = {}
        self.current_goal = None
        self.navigating = False
        self.paused = False
        self.exploration_complete = False
        self.nav_goal_handle = None
        self.no_frontier_count = 0
        self.nav_server_ready = False
        self._nav_lock = threading.Lock()
        self._goal_cancelled = False
        self.nav_error_count = 0

        self.get_logger().info(
            f'FrontierExplorer configured (robot: {self.robot_name or "single"})')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """구독/발행/타이머/ActionClient/TF2 생성 → 활성화.

        active 상태로 진입 전 호출됨.
        """
        exploration_rate = self.get_parameter('exploration_rate').value

        # --- Callback group ---
        self._cb_group = ReentrantCallbackGroup()

        # --- TF2 (로봇 위치 조회) — on_activate에서 생성 ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- QoS ---
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # --- Subscribers ---
        self.map_sub = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, map_qos)

        if self.thermal_pause_enabled:
            self.thermal_sub = self.create_subscription(
                ThermalDetection, 'thermal/detections',
                self.thermal_callback, sensor_qos)

        # 멀티로봇: 다른 로봇 탐색 대상 수신
        self.peers_sub = self.create_subscription(
            PoseStamped, '/exploration/targets',
            self.peer_target_callback, 10)

        # --- Publishers ---
        self.target_pub = self.create_publisher(
            PoseStamped, '/exploration/targets', 10)
        self.status_pub = self.create_publisher(
            String, 'exploration/status', 10)
        self.frontier_viz_pub = self.create_publisher(
            MarkerArray, 'exploration/frontiers_viz', 10)
        self.frontier_count_pub = self.create_publisher(
            UInt32, 'exploration/frontier_count', 10)
        self.nav_error_pub = self.create_publisher(
            UInt32, 'exploration/nav_error_count', 10)

        # --- Services ---
        self.resume_srv = self.create_service(
            Trigger, 'exploration/resume', self.resume_callback)

        # --- Nav2 Action Client — on_activate에서 생성 ---
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self._cb_group)

        # --- Nav2 서버 체크 타이머 (블로킹 방지) ---
        self.server_check_timer = self.create_timer(
            2.0, self._check_nav_server)

        # --- Timer ---
        self.explore_timer = self.create_timer(
            1.0 / exploration_rate, self.explore_cycle)

        self.get_logger().info(
            f'FrontierExplorer activated (robot: {self.robot_name or "single"})')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """타이머 취소 + 구독/발행/ActionClient/TF2 해제 → 중단.

        inactive 상태로 진입 전 호출됨.
        """
        # 진행 중인 Nav2 goal 취소 (graceful shutdown)
        with self._nav_lock:
            handle = self.nav_goal_handle
        if handle is not None:
            self.get_logger().info('Cancelling active Nav2 goal...')
            handle.cancel_goal_async()
            self.nav_goal_handle = None

        # 타이머 취소
        if self.server_check_timer is not None:
            self.server_check_timer.cancel()
            self.destroy_timer(self.server_check_timer)
            self.server_check_timer = None

        if self.explore_timer is not None:
            self.explore_timer.cancel()
            self.destroy_timer(self.explore_timer)
            self.explore_timer = None

        # 구독 해제
        if self.map_sub is not None:
            self.destroy_subscription(self.map_sub)
            self.map_sub = None
        if self.thermal_sub is not None:
            self.destroy_subscription(self.thermal_sub)
            self.thermal_sub = None
        if self.peers_sub is not None:
            self.destroy_subscription(self.peers_sub)
            self.peers_sub = None

        # 발행자 해제
        if self.target_pub is not None:
            self.destroy_publisher(self.target_pub)
            self.target_pub = None
        if self.status_pub is not None:
            self.destroy_publisher(self.status_pub)
            self.status_pub = None
        if self.frontier_viz_pub is not None:
            self.destroy_publisher(self.frontier_viz_pub)
            self.frontier_viz_pub = None
        if self.frontier_count_pub is not None:
            self.destroy_publisher(self.frontier_count_pub)
            self.frontier_count_pub = None
        if self.nav_error_pub is not None:
            self.destroy_publisher(self.nav_error_pub)
            self.nav_error_pub = None

        # 서비스 해제
        if self.resume_srv is not None:
            self.destroy_service(self.resume_srv)
            self.resume_srv = None

        # ActionClient 해제 — on_deactivate에서 정리
        if self.nav_client is not None:
            self.nav_client.destroy()
            self.nav_client = None

        # TF2 해제 — on_deactivate에서 정리
        if self.tf_listener is not None:
            self.tf_listener = None
        if self.tf_buffer is not None:
            self.tf_buffer = None

        self.nav_server_ready = False
        self.navigating = False

        self.get_logger().info('FrontierExplorer deactivated')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """내부 상태 리셋 → unconfigured로 복귀."""
        self.current_map = None
        self.blacklisted = None
        self.other_targets = {}
        self.current_goal = None
        self.navigating = False
        self.paused = False
        self.exploration_complete = False
        self.nav_goal_handle = None
        self.no_frontier_count = 0
        self.nav_server_ready = False
        self._nav_lock = None
        self._goal_cancelled = False
        self.nav_error_count = 0
        self._cb_group = None
        self.get_logger().info('FrontierExplorer cleaned up')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """종료 시 정리."""
        self.get_logger().info('FrontierExplorer shutting down')
        return TransitionCallbackReturn.SUCCESS

    # ─────────────────── Callbacks ───────────────────

    def map_callback(self, msg: OccupancyGrid):
        self.current_map = msg

    def thermal_callback(self, msg: ThermalDetection):
        """high/critical 열화상 감지 시 탐색 일시정지 + keepout zone.

        Nav 전문가 권고: 화점 좌표를 블랙리스트에 추가 → 재탐색 방지.
        """
        if msg.severity in ('high', 'critical') and not self.paused:
            self.paused = True
            temp_c = msg.max_temperature_kelvin - 273.15
            self.get_logger().warn(
                f'THERMAL {msg.severity.upper()}: {temp_c:.0f} C '
                f'— exploration PAUSED')
            # F2 fix: lock 내에서 handle 캡처 + flag 설정, lock 밖에서 cancel
            handle_to_cancel = None
            with self._nav_lock:
                if self.nav_goal_handle is not None:
                    self._goal_cancelled = True
                    handle_to_cancel = self.nav_goal_handle
            if handle_to_cancel is not None:
                handle_to_cancel.cancel_goal_async()
            self.publish_status('paused_thermal')

            # Keepout zone: 화점 좌표를 블랙리스트에 추가 (Nav 전문가 권고)
            if hasattr(msg, 'centroid_image') and self.current_goal is not None:
                # 현재 목표 근처 화점 → 해당 목표 영구 블랙리스트
                self.blacklisted.append(self.current_goal)
                self.get_logger().info(
                    f'Keepout: blacklisted frontier near hotspot')

    def peer_target_callback(self, msg: PoseStamped):
        """다른 로봇의 탐색 대상 수신 (자기 자신 필터링)."""
        peer = msg.header.frame_id
        my_name = self.robot_name or self.get_name()
        if not peer or peer == my_name:
            return  # 자기 자신이 발행한 메시지 무시
        self.other_targets[peer] = (
            msg.pose.position.x, msg.pose.position.y)

    def resume_callback(self, request, response):
        """탐색 재개 서비스."""
        if self.paused:
            self.paused = False
            self.get_logger().info('Exploration RESUMED (via service)')
            self.publish_status('exploring')
            response.success = True
            response.message = 'Exploration resumed'
        else:
            response.success = False
            response.message = 'Not paused'
        return response

    # ─────────────────── Nav2 Server Check ───────────────────

    def _check_nav_server(self):
        """Nav2 액션 서버 가용성을 비블로킹으로 체크."""
        if self.nav_client.server_is_ready():
            self.nav_server_ready = True
            self.server_check_timer.cancel()
            self.get_logger().info('Nav2 action server available')

    # ─────────────────── Core Loop ───────────────────

    def explore_cycle(self):
        """주기적 프론티어 탐색 사이클."""
        if self.current_map is None or self.paused or self.exploration_complete:
            return
        with self._nav_lock:
            if self.navigating:
                return

        frontiers = self.detect_frontiers()
        self.publish_frontier_markers(frontiers)
        self.frontier_count_pub.publish(UInt32(data=len(frontiers)))

        if not frontiers:
            self.no_frontier_count += 1
            if self.no_frontier_count >= 5:
                self.exploration_complete = True
                self.explore_timer.cancel()
                self.get_logger().info(
                    'No frontiers for 5 cycles — EXPLORATION COMPLETE')
                self.publish_status('complete')
            return

        self.no_frontier_count = 0
        target = self.select_best_frontier(frontiers)
        if target is None:
            return

        self.send_nav_goal(target)

    # ─────────────────── Frontier Detection ───────────────────

    def detect_frontiers(self):
        """OccupancyGrid에서 프론티어 클러스터 감지.

        Returns:
            list of (x, y, size): 월드 좌표 중심 + 셀 수
        """
        m = self.current_map
        w, h = m.info.width, m.info.height
        res = m.info.resolution
        # F3: 빈 맵 방어 (width=0, height=0, resolution=0)
        if w == 0 or h == 0 or res <= 0.0:
            return []
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y

        data = np.array(m.data, dtype=np.int8).reshape(h, w)

        free = (data == 0)
        unknown = (data == -1)

        # 프론티어 = unknown에 인접한 free 셀
        kernel = np.ones((3, 3), dtype=np.uint8)
        unknown_dilated = cv2.dilate(
            unknown.astype(np.uint8), kernel, iterations=1).astype(bool)
        frontier_mask = (free & unknown_dilated).astype(np.uint8) * 255

        # Connected components로 클러스터링
        n_labels, labeled = cv2.connectedComponents(frontier_mask)

        frontiers = []
        for i in range(1, n_labels):
            cells = np.argwhere(labeled == i)
            if len(cells) < self.min_frontier_size:
                continue
            cy, cx = cells.mean(axis=0)
            wx = cx * res + ox
            wy = cy * res + oy

            if self._is_blacklisted(wx, wy):
                continue

            frontiers.append((wx, wy, len(cells)))

        return frontiers

    # ─────────────────── Frontier Selection ───────────────────

    def select_best_frontier(self, frontiers):
        """거리 기반 최적 프론티어 선택 (다른 로봇 대상 회피)."""
        robot_pos = self._get_robot_position()
        if robot_pos is None:
            return None

        rx, ry = robot_pos
        candidates = []

        for fx, fy, size in frontiers:
            # 다른 로봇 대상과 너무 가까우면 스킵
            too_close = any(
                math.hypot(fx - px, fy - py) < self.exclusion_radius
                for px, py in self.other_targets.values()
            )
            if too_close:
                continue

            dist = math.hypot(fx - rx, fy - ry)
            candidates.append((dist, fx, fy, size))

        if not candidates:
            return None

        candidates.sort(key=lambda c: c[0])
        _, bx, by, _ = candidates[0]
        return (bx, by)

    # ─────────────────── Navigation ───────────────────

    def send_nav_goal(self, target):
        """Nav2 NavigateToPose 목표 전송 (비블로킹)."""
        if not self.nav_server_ready:
            self.get_logger().warn('Nav2 server not yet available')
            return

        tx, ty = target
        with self._nav_lock:
            self.current_goal = target
            self.navigating = True

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.map_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = tx
        goal_msg.pose.pose.position.y = ty
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'Frontier target: ({tx:.1f}, {ty:.1f})')
        self.publish_status('navigating')

        # 멀티로봇 대상 공유
        target_msg = PoseStamped()
        target_msg.header.frame_id = self.robot_name or self.get_name()
        target_msg.header.stamp = self.get_clock().now().to_msg()
        target_msg.pose.position.x = tx
        target_msg.pose.position.y = ty
        self.target_pub.publish(target_msg)

        future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self._nav_feedback)
        future.add_done_callback(self._nav_goal_response)

    def _nav_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().warn(f'Nav2 goal send failed: {e}')
            with self._nav_lock:
                self.navigating = False
            return

        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 goal rejected — blacklisting')
            with self._nav_lock:
                self.blacklisted.append(self.current_goal)
                self.navigating = False
            return

        self.nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result)

    def _nav_feedback(self, feedback_msg):
        pass

    def _nav_result(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().warn(f'Nav2 result retrieval failed: {e}')
            with self._nav_lock:
                self.navigating = False
                self.nav_goal_handle = None
            return

        with self._nav_lock:
            if self.current_goal is None:
                self.navigating = False
                self.nav_goal_handle = None
                return

            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(
                    f'Reached frontier ({self.current_goal[0]:.1f}, '
                    f'{self.current_goal[1]:.1f})')
                self.blacklisted.append(self.current_goal)
            elif self._goal_cancelled:
                # 열화상 등 의도적 취소 — 블랙리스트 안 함 (재개 시 재시도 가능)
                self.get_logger().info(
                    f'Goal cancelled (will retry on resume)')
                self._goal_cancelled = False
            else:
                self.nav_error_count += 1
                self.get_logger().warn(
                    f'Nav failed (status={result.status}) — blacklisting '
                    f'(errors: {self.nav_error_count})')
                self.blacklisted.append(self.current_goal)
                self.nav_error_pub.publish(
                    UInt32(data=self.nav_error_count))

            self.current_goal = None
            self.navigating = False
            self.nav_goal_handle = None

    # ─────────────────── Helpers ───────────────────

    def _get_robot_position(self):
        """TF2로 로봇의 map 프레임 내 위치 조회."""
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception:
            return None

    def _is_blacklisted(self, x, y):
        # F4: O(1) numpy 벡터화 (기존 O(n) 순회 대체)
        if not self.blacklisted:
            return False
        bl = np.array(self.blacklisted)
        dists = np.hypot(bl[:, 0] - x, bl[:, 1] - y)
        return bool(np.any(dists < self.blacklist_radius))

    def publish_status(self, status: str):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def publish_frontier_markers(self, frontiers):
        """프론티어를 RViz 마커로 시각화."""
        markers = MarkerArray()

        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)

        for i, (fx, fy, size) in enumerate(frontiers):
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'frontiers'
            m.id = i + 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = fx
            m.pose.position.y = fy
            m.pose.position.z = 0.3
            scale = min(0.15 + size * 0.005, 0.8)
            m.scale.x = scale
            m.scale.y = scale
            m.scale.z = scale
            m.color = ColorRGBA(r=0.0, g=1.0, b=0.5, a=0.8)
            m.lifetime.sec = 2
            markers.markers.append(m)

        if self.current_goal:
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'current_target'
            m.id = 0
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.pose.position.x = self.current_goal[0]
            m.pose.position.y = self.current_goal[1]
            m.pose.position.z = 0.5
            m.scale.x = 0.5
            m.scale.y = 0.1
            m.scale.z = 0.1
            m.color = ColorRGBA(r=1.0, g=0.3, b=0.0, a=1.0)
            markers.markers.append(m)

        self.frontier_viz_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()

    # LifecycleNode 단독 실행 시 수동으로 configure → activate 전환
    node.trigger_configure()
    node.trigger_activate()

    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.trigger_deactivate()
        node.trigger_cleanup()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
