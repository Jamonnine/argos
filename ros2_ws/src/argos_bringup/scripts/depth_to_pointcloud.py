#!/usr/bin/env python3
"""
Depth to Point Cloud 변환 노드

이 노드는 RGB-D 카메라의 Depth 이미지를 3D Point Cloud로 변환합니다.

주요 개념:
1. Camera Intrinsics: 카메라 내부 파라미터 (fx, fy, cx, cy)
2. Unprojection: 2D 픽셀 + Depth → 3D 좌표 변환
3. TF2: 좌표계 변환 (camera_frame → base_link)
4. PointCloud2: ROS 2 표준 Point Cloud 메시지

설계 원칙:
- 이벤트 기반: Depth 이미지가 도착하면 자동 처리
- 센서 추상화: camera_info로 Intrinsics를 동적으로 받음
- 성능 최적화: NumPy 벡터화 연산 사용
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
import struct

# TF2 라이브러리
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped


class DepthToPointCloud(Node):
    """
    Depth 이미지를 Point Cloud로 변환하는 노드

    Subscriber:
    - /camera/depth/image_raw (Depth 이미지)
    - /camera/image_raw (RGB 이미지, 색상 정보)
    - /camera/camera_info (Camera Intrinsics)

    Publisher:
    - /camera/points (PointCloud2)
    """

    def __init__(self):
        super().__init__('depth_to_pointcloud')

        # ===== 1. cv_bridge 초기화 =====
        # OpenCV ↔ ROS 이미지 변환기
        self.bridge = CvBridge()

        # ===== 2. TF2 Buffer & Listener =====
        # TF2 Buffer: 과거 10초간의 모든 Transform을 메모리에 저장
        self.tf_buffer = Buffer()
        # TF2 Listener: /tf, /tf_static 토픽을 구독해서 Buffer에 저장
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ===== 3. Camera Intrinsics (아직 모름, camera_info에서 받을 예정) =====
        self.camera_info = None
        self.fx = None  # Focal length X
        self.fy = None  # Focal length Y
        self.cx = None  # Principal point X (이미지 중심)
        self.cy = None  # Principal point Y

        # ===== 4. RGB 이미지 캐시 =====
        # Depth와 RGB는 별도 토픽으로 오므로, 타임스탬프를 맞춰야 함
        # 여기서는 단순화를 위해 최신 RGB를 사용
        self.latest_rgb = None

        # ===== 5. Subscribers =====
        # Camera Info: Intrinsics 정보
        self.info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )

        # RGB 이미지
        self.rgb_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.rgb_callback,
            10
        )

        # Depth 이미지 (메인 트리거)
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10
        )

        # ===== 6. Publisher =====
        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            '/camera/points',
            10
        )

        self.get_logger().info('=' * 60)
        self.get_logger().info('Depth to PointCloud 노드 시작')
        self.get_logger().info('  구독 토픽:')
        self.get_logger().info('    - /camera/depth/image_raw (Depth)')
        self.get_logger().info('    - /camera/image_raw (RGB)')
        self.get_logger().info('    - /camera/camera_info (Intrinsics)')
        self.get_logger().info('  발행 토픽:')
        self.get_logger().info('    - /camera/points (PointCloud2)')
        self.get_logger().info('=' * 60)

        # 처리한 프레임 수
        self.frame_count = 0

    def camera_info_callback(self, msg):
        """
        Camera Intrinsics를 받는 콜백

        CameraInfo 메시지 구조:
        - K[9]: Intrinsic matrix (3x3을 1D로 펼친 것)
          [fx,  0, cx,
            0, fy, cy,
            0,  0,  1]
        - D[5]: Distortion coefficients (렌즈 왜곡 계수)

        용어 설명:
        - Intrinsic Matrix: 카메라 내부 파라미터. 렌즈 특성.
        - fx, fy: Focal Length (초점 거리). 픽셀 단위로 표현.
          값이 클수록 "망원" (멀리 보임), 작을수록 "광각" (넓게 보임)
        - cx, cy: Principal Point (주점). 광축이 이미지 평면과 만나는 점.
          이상적으로는 이미지 정중앙이지만, 제조 오차로 약간 틀어질 수 있음.
        """
        if self.camera_info is None:
            self.camera_info = msg

            # K 행렬에서 파라미터 추출
            # K는 row-major order로 저장: [K0, K1, K2, K3, K4, K5, K6, K7, K8]
            self.fx = msg.k[0]  # K[0, 0]
            self.fy = msg.k[4]  # K[1, 1]
            self.cx = msg.k[2]  # K[0, 2]
            self.cy = msg.k[5]  # K[1, 2]

            self.get_logger().info('Camera Intrinsics 수신:')
            self.get_logger().info(f'  fx = {self.fx:.2f} (X축 초점 거리)')
            self.get_logger().info(f'  fy = {self.fy:.2f} (Y축 초점 거리)')
            self.get_logger().info(f'  cx = {self.cx:.2f} (이미지 중심 X)')
            self.get_logger().info(f'  cy = {self.cy:.2f} (이미지 중심 Y)')
            self.get_logger().info(f'  이미지 크기: {msg.width}x{msg.height}')

    def rgb_callback(self, msg):
        """
        RGB 이미지를 캐시에 저장

        동기화 문제:
        Depth와 RGB는 별도 토픽으로 전송되므로, 타임스탬프가 약간 다를 수 있음.
        실무에서는 message_filters 패키지의 TimeSynchronizer를 사용해서
        타임스탬프가 일치하는 메시지만 매칭.

        여기서는 단순화를 위해 최신 RGB를 사용.
        """
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'RGB 변환 실패: {e}')

    def depth_callback(self, msg):
        """
        Depth 이미지를 받아서 Point Cloud로 변환

        처리 파이프라인:
        1. Depth 이미지 → NumPy 배열
        2. Unprojection (2D + Depth → 3D)
        3. RGB 색상 추가
        4. PointCloud2 메시지 생성
        5. 발행
        """
        # ===== 전제 조건 확인 =====
        if self.camera_info is None:
            # Intrinsics를 아직 못 받았으면, 처리 불가능
            self.get_logger().warn('Camera Info 대기 중...', throttle_duration_sec=1.0)
            return

        try:
            # ===== 1. Depth 이미지 변환 =====
            # 'passthrough': 원본 데이터 타입 유지 (보통 16UC1 또는 32FC1)
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # Depth 이미지 타입 확인
            # 16UC1: 16-bit unsigned int (단위: mm)
            # 32FC1: 32-bit float (단위: m)
            if depth_image.dtype == np.uint16:
                # mm → m 변환
                depth_image = depth_image.astype(np.float32) / 1000.0

            height, width = depth_image.shape

            # ===== 2. Unprojection =====
            # 픽셀 그리드 생성: (u, v) 좌표
            # np.meshgrid: 2D 그리드를 한 번에 생성 (벡터화!)
            # indexing='xy': (u, v) 순서 (기본은 ij라서 (v, u)가 됨)
            u, v = np.meshgrid(
                np.arange(width),   # u: 0, 1, 2, ..., width-1
                np.arange(height),  # v: 0, 1, 2, ..., height-1
                indexing='xy'
            )

            # Unprojection 공식:
            # X = (u - cx) * depth / fx
            # Y = (v - cy) * depth / fy
            # Z = depth

            # 벡터화 연산 (픽셀 단위 for loop 없음!)
            x = (u - self.cx) * depth_image / self.fx
            y = (v - self.cy) * depth_image / self.fy
            z = depth_image

            # ===== 3. 유효한 점만 필터링 =====
            # 유효 조건:
            # - Depth > 0 (유효한 측정값)
            # - Depth < 10.0 (최대 범위, 센서마다 다름)
            # - np.isfinite() (NaN, Inf 제거)
            valid_mask = (z > 0.0) & (z < 10.0) & np.isfinite(z)

            # 마스크 적용 (1D 배열로 펼침)
            x = x[valid_mask]
            y = y[valid_mask]
            z = z[valid_mask]

            # ===== 4. RGB 색상 추가 =====
            if self.latest_rgb is not None:
                # RGB 이미지도 같은 마스크 적용
                # latest_rgb: (height, width, 3)
                # valid_mask: (height, width)
                rgb = self.latest_rgb[valid_mask]  # (N, 3)
            else:
                # RGB 없으면 흰색으로
                rgb = np.ones((len(x), 3), dtype=np.uint8) * 255

            # ===== 5. PointCloud2 메시지 생성 =====
            pointcloud_msg = self.create_pointcloud2(
                x, y, z, rgb,
                frame_id=msg.header.frame_id,  # 'camera_frame' (원본 좌표계)
                timestamp=msg.header.stamp
            )

            # ===== 6. 발행 =====
            self.pointcloud_pub.publish(pointcloud_msg)

            self.frame_count += 1
            if self.frame_count % 30 == 0:
                self.get_logger().info(
                    f'PointCloud 발행: {len(x):,}개 점, '
                    f'프레임 {self.frame_count}'
                )

        except Exception as e:
            self.get_logger().error(f'Depth 처리 실패: {e}')

    def create_pointcloud2(self, x, y, z, rgb, frame_id, timestamp):
        """
        NumPy 배열을 PointCloud2 메시지로 변환

        PointCloud2 구조:
        - Header: 타임스탬프, 좌표계 ID
        - fields: 각 점의 데이터 필드 정의
        - data: 실제 바이너리 데이터 (flat byte array)

        우리의 Point 구조:
        - x, y, z: float32 (4바이트씩)
        - rgb: uint32 (4바이트, R|G|B 패킹)

        총 Point 크기: 16바이트
        """
        # ===== 1. PointField 정의 =====
        # PointField: 각 필드의 이름, 오프셋, 타입 정의
        fields = [
            PointField(
                name='x',
                offset=0,
                datatype=PointField.FLOAT32,
                count=1
            ),
            PointField(
                name='y',
                offset=4,
                datatype=PointField.FLOAT32,
                count=1
            ),
            PointField(
                name='z',
                offset=8,
                datatype=PointField.FLOAT32,
                count=1
            ),
            PointField(
                name='rgb',
                offset=12,
                datatype=PointField.UINT32,
                count=1
            ),
        ]

        # ===== 2. RGB 패킹 =====
        # RGB (3바이트)를 UINT32 (4바이트)로 패킹
        # 형식: 0x00RRGGBB (big-endian)
        # 예: R=255, G=128, B=64 → 0x00FF8040
        r = rgb[:, 2].astype(np.uint32)  # OpenCV는 BGR 순서
        g = rgb[:, 1].astype(np.uint32)
        b = rgb[:, 0].astype(np.uint32)

        rgb_packed = (r << 16) | (g << 8) | b

        # ===== 3. 구조화 배열 생성 =====
        # NumPy structured array: 여러 타입의 필드를 하나의 배열로
        num_points = len(x)
        cloud_data = np.zeros(
            num_points,
            dtype=[
                ('x', np.float32),
                ('y', np.float32),
                ('z', np.float32),
                ('rgb', np.uint32),
            ]
        )

        cloud_data['x'] = x
        cloud_data['y'] = y
        cloud_data['z'] = z
        cloud_data['rgb'] = rgb_packed

        # ===== 4. PointCloud2 메시지 =====
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = timestamp
        msg.header.frame_id = frame_id

        msg.height = 1  # Unorganized (1D array)
        msg.width = num_points
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 16  # 한 점의 바이트 크기
        msg.row_step = msg.point_step * num_points
        msg.is_dense = True  # NaN/Inf 없음 (이미 필터링했으므로)
        msg.data = cloud_data.tobytes()  # NumPy → bytes

        return msg


def main(args=None):
    rclpy.init(args=args)
    node = DepthToPointCloud()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C 감지, 종료합니다...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
