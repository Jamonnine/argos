"""
perception_bridge_node.py — Day 18: AI-Robotics 통합 브릿지

아키텍처 역할:
  [AI 인식] → /detections (DetectionArray)
                       ↓
            [이 노드: Perception Bridge]
                       ↓
  서비스: /navigate_to_object (NavigateToObject.srv)
  액션 클라이언트: Nav2 NavigateToPose

실무 패턴:
- 서비스 서버: "AI가 감지한 X 앞으로 가줘" 요청 처리
- 최근 감지 결과 캐싱 (deque로 슬라이딩 윈도우)
- TF2로 camera_frame → map 좌표 변환 (실무 핵심!)
- Nav2 Action 클라이언트로 실제 이동 명령
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from collections import deque
import math

# 표준 인터페이스
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose_stamped

# 우리가 직접 만든 커스텀 인터페이스
from my_robot_interfaces.msg import DetectedObject, DetectionArray
from my_robot_interfaces.srv import NavigateToObject


class PerceptionBridgeNode(Node):
    """
    AI 인식 결과를 Nav2 이동 명령으로 변환하는 브릿지 노드.

    설계 원칙:
    1. 인터페이스 분리: AI팀은 /detections만 발행, 이 노드가 Nav2 연결 담당
    2. 캐싱: 가장 최근 N개 감지 결과를 메모리에 유지 → 서비스 요청 시 즉시 응답
    3. 신뢰도 필터링: min_confidence 파라미터로 품질 기준 제어
    """

    def __init__(self):
        super().__init__('perception_bridge')

        # ReentrantCallbackGroup: 서비스와 액션이 동시에 실행 가능하도록
        # (SingleThreadedExecutor 기본값으로는 서비스 중에 다른 콜백 블로킹됨)
        self._cb_group = ReentrantCallbackGroup()

        # ── 파라미터 ───────────────────────────────────────────────
        self.declare_parameter('detection_cache_size', 10)
        self.declare_parameter('default_approach_distance', 0.5)
        self.declare_parameter('simulate_detections', False)

        cache_size = self.get_parameter('detection_cache_size').value
        self._approach_dist = self.get_parameter('default_approach_distance').value
        self._simulate = self.get_parameter('simulate_detections').value

        # ── TF2 (base_link → map 변환) ─────────────────────────────
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ── 감지 결과 캐시 ─────────────────────────────────────────
        # deque: FIFO, maxlen 초과 시 오래된 항목 자동 제거
        self._detection_cache: deque = deque(maxlen=cache_size)

        # ── Subscriber: 실제 AI 감지 결과 구독 ────────────────────
        self._detection_sub = self.create_subscription(
            DetectionArray,
            '/detections',
            self._on_detections,
            10,
            callback_group=self._cb_group,
        )

        # ── Publisher: 감지 결과 발행 (시뮬레이션 모드) ────────────
        self._detection_pub = self.create_publisher(DetectionArray, '/detections', 10)

        # ── LiDAR 구독 (시뮬레이션: LiDAR → 가짜 감지 결과) ───────
        if self._simulate:
            self._scan_sub = self.create_subscription(
                LaserScan, '/scan', self._on_scan_simulate, 10,
                callback_group=self._cb_group,
            )
            self.get_logger().info('시뮬레이션 모드: LiDAR → 가짜 감지 결과 생성 중')

        # ── Service Server: NavigateToObject ───────────────────────
        self._nav_to_obj_srv = self.create_service(
            NavigateToObject,
            '/navigate_to_object',
            self._handle_navigate_to_object,
            callback_group=self._cb_group,
        )

        # ── Nav2 Action Client ─────────────────────────────────────
        self._nav2_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self._cb_group,
        )
        self._nav2_server_ready = False
        self._server_check_timer = self.create_timer(
            1.0, self._check_nav2_server)

        self.get_logger().info(
            'PerceptionBridgeNode 시작\n'
            '  서비스: /navigate_to_object\n'
            '  구독:   /detections\n'
            '  액션:   navigate_to_pose (Nav2)'
        )

    # ──────────────────────────────────────────────────────────────
    # Nav2 서버 비블로킹 체크
    # ──────────────────────────────────────────────────────────────

    def _check_nav2_server(self):
        """Nav2 서버 준비 상태를 비블로킹으로 확인."""
        if self._nav2_client.server_is_ready():
            self._nav2_server_ready = True
            self._server_check_timer.cancel()
            self.get_logger().info('Nav2 서버 연결 완료')

    # ──────────────────────────────────────────────────────────────
    # 감지 결과 캐싱
    # ──────────────────────────────────────────────────────────────

    def _on_detections(self, msg: DetectionArray):
        """실제 AI 감지 결과 수신 → 캐시에 저장."""
        # P3: 자기 발행 메시지 무시 (에코 루프 방지)
        if msg.detector_name == 'lidar_simulator' and self._simulate:
            return
        for obj in msg.detections:
            self._detection_cache.append(obj)
        self.get_logger().debug(
            f'{len(msg.detections)}개 감지 결과 캐시됨 '
            f'(현재 캐시: {len(self._detection_cache)}개)'
        )

    # ──────────────────────────────────────────────────────────────
    # 시뮬레이션: LiDAR 가장 가까운 점 → 가짜 "box" 객체
    # ──────────────────────────────────────────────────────────────

    def _on_scan_simulate(self, scan: LaserScan):
        """
        LiDAR에서 가장 가까운 점을 'box' 객체로 시뮬레이션.
        실무에서는 이 자리에 YOLO/Detectron2 출력이 들어옵니다.
        """
        # 유효한 거리 값만 (inf, nan 제외)
        valid = [
            (i, r) for i, r in enumerate(scan.ranges)
            if scan.range_min < r < scan.range_max
        ]
        if not valid:
            return

        # 가장 가까운 점
        closest_idx, closest_dist = min(valid, key=lambda x: x[1])
        angle = scan.angle_min + closest_idx * scan.angle_increment

        # 극좌표 → 직교좌표 (base_link 기준)
        x = closest_dist * math.cos(angle)
        y = closest_dist * math.sin(angle)

        # 0.3m 이상 거리의 객체만 (너무 가까우면 노이즈)
        if closest_dist < 0.3:
            return

        # DetectedObject 생성
        obj = DetectedObject()
        obj.header = Header()
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.header.frame_id = 'base_link'   # LiDAR가 base_link 기준
        obj.class_name = 'obstacle'
        obj.confidence = 0.85
        obj.track_id = closest_idx          # 인덱스를 임시 ID로 사용
        obj.pose_3d.position.x = x
        obj.pose_3d.position.y = y
        obj.pose_3d.position.z = 0.0
        obj.pose_3d.orientation.w = 1.0
        obj.distance = closest_dist

        # P3: 에코 루프 방지 — 자기 발행 → 자기 구독 경로 대신 직접 캐시 저장
        self._detection_cache.append(obj)

        # 외부 구독자용으로도 발행 (다른 노드에서 필요할 수 있음)
        arr = DetectionArray()
        arr.header = obj.header
        arr.detections = [obj]
        arr.detector_name = 'lidar_simulator'
        self._detection_pub.publish(arr)

    # ──────────────────────────────────────────────────────────────
    # Service Server: NavigateToObject 처리
    # ──────────────────────────────────────────────────────────────

    def _handle_navigate_to_object(
        self,
        request: NavigateToObject.Request,
        response: NavigateToObject.Response,
    ) -> NavigateToObject.Response:
        """
        핵심 서비스 핸들러: AI 감지 결과 → Nav2 목표 변환.

        설계 결정:
        - Service는 "수락 여부"만 반환 (즉시 응답)
        - 실제 이동은 Nav2 Action으로 비동기 처리
        - 이유: Service를 블로킹으로 쓰면 다른 요청이 대기해야 함
        """
        self.get_logger().info(
            f'NavigateToObject 요청: class={request.target_class}, '
            f'min_conf={request.min_confidence:.2f}'
        )

        # 1. 캐시에서 요청 클래스의 객체 찾기
        target = self._find_best_detection(
            request.target_class,
            request.min_confidence,
        )

        if target is None:
            response.accepted = False
            response.message = (
                f"'{request.target_class}' 클래스 객체를 찾지 못했습니다. "
                f"(캐시: {len(self._detection_cache)}개, "
                f"min_confidence: {request.min_confidence:.2f})"
            )
            self.get_logger().warn(response.message)
            return response

        # 2. 목표 위치 계산 (객체 앞 approach_distance만큼 떨어진 곳)
        approach = request.approach_distance
        if approach <= 0.0:
            approach = self._approach_dist

        target_pose = self._compute_approach_pose(target, approach)
        response.accepted = True
        response.message = (
            f"'{request.target_class}' 객체 발견 "
            f"(confidence={target.confidence:.2f}, "
            f"distance={target.distance:.2f}m). "
            f"Nav2 이동 명령 전송."
        )
        response.target_pose = target_pose
        response.object_id = str(target.track_id)
        response.estimated_distance = target.distance

        # 3. Nav2 Action으로 이동 명령 전송 (비동기)
        self._send_nav2_goal(target_pose)

        self.get_logger().info(response.message)
        return response

    def _find_best_detection(
        self, class_name: str, min_confidence: float
    ):
        """캐시에서 해당 클래스 중 가장 신뢰도 높은 객체 반환.

        P2: strict 매칭 — 요청 클래스와 정확히 일치하는 객체만 반환.
        """
        candidates = [
            obj for obj in self._detection_cache
            if obj.class_name == class_name
            and obj.confidence >= min_confidence
        ]
        if not candidates:
            return None
        # 신뢰도 기준 정렬, 가장 높은 것 반환
        return max(candidates, key=lambda o: o.confidence)

    def _compute_approach_pose(
        self, obj: DetectedObject, approach_dist: float
    ) -> Pose:
        """
        객체 위치에서 approach_dist만큼 앞에 있는 목표 자세 계산.

        실무 노트:
        obj.header.frame_id가 'base_link'이므로,
        실제 배포에서는 TF2로 map 좌표계로 변환해야 합니다.
        여기서는 시뮬레이션 단순화를 위해 직접 사용합니다.
        """
        obj_x = obj.pose_3d.position.x
        obj_y = obj.pose_3d.position.y
        dist = max(obj.distance - approach_dist, 0.1)

        # 객체 방향으로 approach_dist 앞
        angle = math.atan2(obj_y, obj_x)
        target_x = dist * math.cos(angle)
        target_y = dist * math.sin(angle)

        pose = Pose()
        pose.position.x = target_x
        pose.position.y = target_y
        pose.position.z = 0.0
        # 객체를 향하도록 yaw 설정
        pose.orientation.z = math.sin(angle / 2.0)
        pose.orientation.w = math.cos(angle / 2.0)
        return pose

    def _send_nav2_goal(self, pose: Pose):
        """Nav2 NavigateToPose Action에 목표 전송 (비블로킹).

        P1 fix: base_link 좌표를 TF2로 map 프레임으로 변환 후 전송.
        Nav2는 map 프레임 목표를 기대하므로 base_link 그대로 보내면 엉뚱한 위치로 이동.
        """
        if not self._nav2_server_ready:
            self.get_logger().warn('Nav2 서버 미준비 — 목표 드롭')
            return

        # base_link 기준 PoseStamped 생성
        pose_bl = PoseStamped()
        pose_bl.header.stamp = self.get_clock().now().to_msg()
        pose_bl.header.frame_id = 'base_link'
        pose_bl.pose = pose

        # TF2로 base_link → map 변환
        try:
            transform = self._tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            pose_map = do_transform_pose_stamped(pose_bl, transform)
        except Exception as e:
            self.get_logger().warn(f'TF2 변환 실패 (base_link→map): {e} — 목표 드롭')
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose_map

        self.get_logger().info(
            f'Nav2 목표 전송 (map): x={pose_map.pose.position.x:.2f}, '
            f'y={pose_map.pose.position.y:.2f}'
        )
        self._nav2_client.send_goal_async(goal)


def main(args=None):
    rclpy.init(args=args)
    from rclpy.executors import MultiThreadedExecutor

    node = PerceptionBridgeNode()
    # MultiThreadedExecutor: ReentrantCallbackGroup과 함께
    # 서비스 + 액션 + 구독이 동시에 처리되도록
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('PerceptionBridge shutting down')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
