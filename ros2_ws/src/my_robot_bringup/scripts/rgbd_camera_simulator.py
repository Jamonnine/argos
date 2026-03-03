#!/usr/bin/env python3
"""
RGB-D 카메라 시뮬레이터

이 노드는 실제 RGB-D 카메라(Intel RealSense, Kinect 등)를 시뮬레이션합니다.
가상의 3D 장면을 렌더링해서 RGB와 Depth 이미지를 생성합니다.

발행 토픽:
- /camera/image_raw: RGB 이미지
- /camera/depth/image_raw: Depth 이미지
- /camera/camera_info: 카메라 내부 파라미터

TF:
- base_link → camera_frame: 카메라가 로봇 몸통에서 어디 붙어있는지

설계 목표:
1. 실제 카메라와 동일한 인터페이스 제공 (센서 추상화)
2. 테스트 가능한 간단한 장면 (빨간 공, 파란 상자 등)
3. Depth 정확성 (Unprojection이 올바르게 작동하도록)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
import cv2

# TF2 브로드캐스터
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped


class RGBDCameraSimulator(Node):
    """
    가상의 RGB-D 카메라

    시뮬레이션 장면:
    - 빨간 공: 카메라 앞 1.0미터, 반지름 0.1미터
    - 파란 상자: 카메라 앞 2.0미터, 크기 0.3×0.3미터
    - 배경 벽: 5.0미터
    """

    def __init__(self):
        super().__init__('rgbd_camera_simulator')

        # ===== 카메라 파라미터 =====
        # 이미지 해상도
        # 용어 설명:
        # - Resolution(해상도): 이미지의 픽셀 개수. 높을수록 디테일하지만, 메모리와 계산량 증가.
        # - 640×480은 VGA 해상도. 옛날 웹캠 표준. 요즘은 HD(1920×1080) 많이 씀.
        self.width = 640
        self.height = 480

        # Focal Length (초점 거리, 픽셀 단위)
        # 용어 설명:
        # - Focal Length: 렌즈에서 이미지 센서까지의 거리를 픽셀로 환산한 값.
        # - 값이 클수록 "망원" (좁은 시야각), 작을수록 "광각" (넓은 시야각).
        # - 실제 카메라: 일반적으로 400~800 범위. 우리는 중간값 600 사용.
        self.fx = 600.0  # X축 초점 거리
        self.fy = 600.0  # Y축 초점 거리 (일반적으로 fx와 같음)

        # Principal Point (주점, 픽셀 단위)
        # 용어 설명:
        # - Principal Point: 광축(렌즈 중심에서 직각으로 내린 선)이 이미지 평면과 만나는 점.
        # - 이상적으로는 이미지 정중앙 (width/2, height/2).
        # - 실제 카메라는 렌즈와 센서 정렬 오차로 약간 틀어질 수 있음.
        self.cx = self.width / 2.0   # 320.0 (이미지 중심 X)
        self.cy = self.height / 2.0  # 240.0 (이미지 중심 Y)

        # ===== cv_bridge 초기화 =====
        self.bridge = CvBridge()

        # ===== TF2 Static Transform Broadcaster =====
        # 용어 설명:
        # - Static Transform: 절대 변하지 않는 Transform. 카메라가 로봇에 나사로 고정되어 있으면 Static.
        # - Broadcaster: 한 곳에서 여러 곳으로 정보를 전송하는 역할.
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        # 카메라 Transform 발행
        # "base_link에서 camera_frame으로 가는 방법"을 정의
        self.broadcast_static_tf()

        # ===== Publishers =====
        # RGB 이미지 Publisher
        # 용어 설명:
        # - Publisher: ROS 2에서 데이터를 "발행"하는 객체. 신문사가 신문을 발행하듯이.
        # - Topic: 데이터가 흐르는 "채널". 라디오 주파수처럼 특정 이름을 가짐.
        # - Queue Size (10): 메시지를 몇 개까지 메모리에 쌓아둘지. 받는 쪽이 느리면 오래된 메시지는 버려짐.
        self.rgb_pub = self.create_publisher(
            Image,
            '/camera/image_raw',
            10
        )

        # Depth 이미지 Publisher
        self.depth_pub = self.create_publisher(
            Image,
            '/camera/depth/image_raw',
            10
        )

        # Camera Info Publisher
        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            '/camera/camera_info',
            10
        )

        # ===== Timer: 30Hz로 이미지 생성 =====
        # 용어 설명:
        # - Timer: 일정 주기마다 함수를 자동 호출. 알람시계처럼.
        # - 30Hz = 초당 30번 = 0.0333초마다 1번 = 33.3ms마다 1번
        # - 실제 카메라는 보통 30Hz 또는 60Hz로 이미지를 전송함.
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

        # 프레임 카운터 (디버깅용)
        self.frame_count = 0

        self.get_logger().info('=' * 60)
        self.get_logger().info('RGB-D 카메라 시뮬레이터 시작')
        self.get_logger().info(f'  해상도: {self.width}×{self.height}')
        self.get_logger().info(f'  초점 거리: fx={self.fx:.1f}, fy={self.fy:.1f}')
        self.get_logger().info(f'  주점: cx={self.cx:.1f}, cy={self.cy:.1f}')
        self.get_logger().info(f'  프레임 레이트: 30 Hz')
        self.get_logger().info('  발행 토픽:')
        self.get_logger().info('    - /camera/image_raw (RGB)')
        self.get_logger().info('    - /camera/depth/image_raw (Depth)')
        self.get_logger().info('    - /camera/camera_info (Intrinsics)')
        self.get_logger().info('=' * 60)

    def broadcast_static_tf(self):
        """
        Static Transform 발행: base_link → camera_frame

        카메라가 로봇 몸통에서 어디 붙어있는지를 정의합니다.

        우리 설정:
        - 위치: 몸통 중심에서 앞으로 0.1m, 위로 0.3m
        - 회전: 30도 아래를 향함 (고개를 숙인 것처럼)

        용어 설명:
        - Translation: "위치 이동". 한 좌표계에서 다른 좌표계로 얼마나 떨어졌는가.
        - Rotation: "회전". 한 좌표계가 다른 좌표계에 비해 얼마나 틀어졌는가.
        - Quaternion: 회전을 표현하는 수학적 방법. 4개 숫자 (w, x, y, z)로 표현.
        """
        t = TransformStamped()

        # Header: 메타데이터
        # 용어 설명:
        # - Header: 메시지의 "머리말". 언제, 어떤 좌표계인지 등의 정보.
        # - stamp: "도장 찍다". 메시지가 언제 생성되었는지 시간 기록.
        # - frame_id: "좌표계 ID". 이 Transform의 부모(출발점).
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'  # 부모: 로봇 몸통
        t.child_frame_id = 'camera_frame'  # 자식: 카메라

        # Translation: 위치 (미터 단위)
        # base_link 원점에서 camera_frame 원점으로 가는 벡터
        t.transform.translation.x = 0.1  # 앞으로 10cm
        t.transform.translation.y = 0.0  # 좌우 중앙
        t.transform.translation.z = 0.3  # 위로 30cm

        # Rotation: 회전 (Quaternion으로 표현)
        # 30도 아래를 보도록 Pitch 회전
        # 용어 설명:
        # - Pitch: Y축 기준 회전. 고개를 끄덕이는 동작. "위아래"를 본다.
        # - Roll: X축 기준 회전. 고개를 갸웃하는 동작. "좌우로 기울인다".
        # - Yaw: Z축 기준 회전. 고개를 좌우로 돌리는 동작. "왼쪽/오른쪽"을 본다.
        #
        # Euler Angles → Quaternion 변환 (30도 Pitch)
        # 30도 = 0.5236 라디안
        # 용어 설명:
        # - Radian(라디안): 각도를 표현하는 다른 방법. 수학에서 표준.
        # - 180도 = π 라디안, 90도 = π/2 라디안
        # - 30도 = 30 × (π/180) = 0.5236 라디안
        pitch = -0.5236  # -30도 (음수 = 아래를 봄)

        # Quaternion 계산 (단순화: Pitch만 회전, Roll=0, Yaw=0)
        # 용어 설명:
        # - sin, cos: 삼각함수. 회전을 계산할 때 사용.
        # - Quaternion 공식 (Pitch만):
        #   w = cos(pitch/2)
        #   x = 0
        #   y = sin(pitch/2)
        #   z = 0
        import math
        t.transform.rotation.w = math.cos(pitch / 2.0)  # ≈ 0.9659
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = math.sin(pitch / 2.0)  # ≈ -0.2588
        t.transform.rotation.z = 0.0

        # Static Transform 발행
        # 용어 설명:
        # - sendTransform: "Transform을 보내다". TF 시스템에 등록.
        # - Static: "정적인". 한 번 보내면 계속 유효. 움직이지 않는 부품용.
        self.tf_static_broadcaster.sendTransform(t)

        self.get_logger().info('Static TF 발행 완료: base_link → camera_frame')

    def timer_callback(self):
        """
        30Hz 타이머 콜백: 매 프레임마다 이미지 생성 및 발행

        단계:
        1. RGB 이미지 생성 (가상 3D 장면 렌더링)
        2. Depth 이미지 생성 (각 픽셀의 거리 계산)
        3. Camera Info 생성
        4. ROS 2 메시지로 변환
        5. 발행
        """
        # 현재 시각 (모든 메시지가 동일한 타임스탬프를 가져야 동기화됨)
        now = self.get_clock().now().to_msg()

        # ===== 1. RGB 이미지 생성 =====
        rgb_image = self.generate_rgb_image()

        # ===== 2. Depth 이미지 생성 =====
        depth_image = self.generate_depth_image()

        # ===== 3. Camera Info 생성 =====
        camera_info = self.generate_camera_info(now)

        # ===== 4. ROS 2 메시지로 변환 =====
        try:
            # RGB: OpenCV Mat → ROS Image
            # 용어 설명:
            # - encoding: 픽셀 데이터를 어떻게 해석할지. 'bgr8' = Blue-Green-Red, 각 8비트.
            # - OpenCV는 BGR 순서를 사용 (RGB 아님!). 역사적 이유...
            rgb_msg = self.bridge.cv2_to_imgmsg(rgb_image, encoding='bgr8')
            rgb_msg.header.stamp = now
            rgb_msg.header.frame_id = 'camera_frame'

            # Depth: NumPy Array → ROS Image
            # 용어 설명:
            # - 32FC1: 32-bit Float, 1 Channel (채널 = 색상 수. Gray는 1, RGB는 3)
            # - Depth는 단일 값(거리)이므로 1채널.
            depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding='32FC1')
            depth_msg.header.stamp = now
            depth_msg.header.frame_id = 'camera_frame'

            # ===== 5. 발행 =====
            self.rgb_pub.publish(rgb_msg)
            self.depth_pub.publish(depth_msg)
            self.camera_info_pub.publish(camera_info)

            self.frame_count += 1

            # 1초마다 로그 (30프레임마다)
            if self.frame_count % 30 == 0:
                self.get_logger().info(
                    f'프레임 {self.frame_count} 발행 '
                    f'({self.frame_count // 30}초 경과)'
                )

        except Exception as e:
            self.get_logger().error(f'이미지 변환 실패: {e}')

    def generate_rgb_image(self):
        """
        가상 3D 장면의 RGB 이미지 생성

        장면 구성:
        - 빨간 공: 중앙, 거리 1.0m
        - 파란 상자: 오른쪽, 거리 2.0m
        - 회색 배경: 거리 5.0m

        반환: (height, width, 3) NumPy 배열, dtype=uint8, BGR 순서
        """
        # 검은 배경 생성
        # 용어 설명:
        # - np.zeros: 모든 값이 0인 배열 생성. 픽셀 값 0 = 검정색.
        # - dtype=uint8: 8비트 부호 없는 정수. 0~255 범위. 이미지 픽셀의 표준.
        # - shape=(height, width, 3): 세로 × 가로 × 채널(BGR).
        image = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        # 배경을 회색으로 칠하기
        # 용어 설명:
        # - [:, :] = value: NumPy 슬라이싱. 모든 행, 모든 열에 값 할당.
        # - (100, 100, 100): BGR 모두 100 = 어두운 회색.
        image[:, :] = (100, 100, 100)

        # ===== 빨간 공 그리기 =====
        # 3D 위치: 카메라 앞 1.0m, 중앙
        ball_center_3d = np.array([0.0, 0.0, 1.0])  # (X, Y, Z)
        ball_radius_3d = 0.1  # 반지름 0.1m = 10cm

        # 3D → 2D 투영 (Projection)
        # 용어 설명:
        # - Projection: 3D 공간의 점을 2D 이미지 평면으로 "투사". 카메라가 하는 일.
        # - 공식: u = fx * X/Z + cx, v = fy * Y/Z + cy
        ball_center_2d = self.project_point(ball_center_3d)

        # 반지름도 투영 (거리에 따라 작아짐)
        # 용어 설명:
        # - Perspective(원근법): 멀리 있는 물체가 작게 보이는 현상.
        # - 반지름 투영: radius_2d = fx * radius_3d / Z
        ball_radius_2d = int(self.fx * ball_radius_3d / ball_center_3d[2])

        # OpenCV로 원 그리기
        # 용어 설명:
        # - cv2.circle: OpenCV의 원 그리기 함수.
        # - center: 중심 좌표 (u, v), 정수 튜플.
        # - radius: 반지름, 정수.
        # - color: (B, G, R) 튜플. 빨간색 = (0, 0, 255).
        # - thickness=-1: 채워진 원. 양수면 테두리만.
        cv2.circle(
            image,
            center=(int(ball_center_2d[0]), int(ball_center_2d[1])),
            radius=ball_radius_2d,
            color=(0, 0, 255),  # 빨간색 (BGR 순서!)
            thickness=-1  # 채우기
        )

        # ===== 파란 상자 그리기 =====
        # 3D 위치: 카메라 앞 2.0m, 오른쪽 0.5m
        box_center_3d = np.array([0.5, 0.0, 2.0])  # (X, Y, Z)
        box_size_3d = 0.3  # 한 변 30cm

        # 상자의 네 모서리 계산 (XY 평면)
        # 용어 설명:
        # - 상자는 사각형. 중심에서 ±size/2 떨어진 4개 점.
        half_size = box_size_3d / 2.0
        corners_3d = [
            box_center_3d + np.array([-half_size, -half_size, 0]),  # 왼쪽 위
            box_center_3d + np.array([half_size, -half_size, 0]),   # 오른쪽 위
            box_center_3d + np.array([half_size, half_size, 0]),    # 오른쪽 아래
            box_center_3d + np.array([-half_size, half_size, 0]),   # 왼쪽 아래
        ]

        # 모서리들을 2D로 투영
        corners_2d = [self.project_point(corner) for corner in corners_3d]

        # 정수 좌표로 변환 (OpenCV는 정수 픽셀 좌표 필요)
        # 용어 설명:
        # - astype(int): NumPy 배열의 타입 변환. float → int.
        # - reshape(-1, 1, 2): OpenCV polylines 함수가 요구하는 형태.
        #   -1 = "자동 계산", 1 = "한 개 폴리곤", 2 = "(x, y) 쌍"
        pts = np.array(corners_2d, dtype=int).reshape((-1, 1, 2))

        # 다각형 그리기
        # 용어 설명:
        # - cv2.polylines: 여러 선분을 연결해서 다각형 그리기.
        # - isClosed=True: 마지막 점과 첫 점을 연결 (닫힌 도형).
        # - color=(255, 0, 0): 파란색 (BGR!).
        # - thickness=2: 선 두께 2픽셀.
        cv2.polylines(image, [pts], isClosed=True, color=(255, 0, 0), thickness=2)

        return image

    def generate_depth_image(self):
        """
        Depth 이미지 생성 (간단한 기하학적 모델)

        각 픽셀마다 "이 방향에 물체가 있다면 얼마나 떨어졌나?"를 계산합니다.

        우리의 간단한 장면:
        - 빨간 공: 거리 1.0m에 구체
        - 파란 상자: 거리 2.0m에 평면
        - 배경: 거리 5.0m

        실제 Depth 카메라는 적외선이나 Stereo로 측정하지만,
        우리는 기하학적으로 계산합니다.

        반환: (height, width) NumPy 배열, dtype=float32, 단위=미터
        """
        # 모든 픽셀을 배경 거리(5.0m)로 초기화
        # 용어 설명:
        # - np.full: 모든 값이 특정 값인 배열 생성.
        # - dtype=float32: 32비트 부동소수점. Depth는 소수점 필요 (예: 1.543m).
        depth = np.full((self.height, self.width), 5.0, dtype=np.float32)

        # ===== 빨간 공의 Depth 추가 =====
        # 3D 위치
        ball_center_3d = np.array([0.0, 0.0, 1.0])
        ball_radius_3d = 0.1

        # 2D 투영
        ball_center_2d = self.project_point(ball_center_3d)
        ball_radius_2d = int(self.fx * ball_radius_3d / ball_center_3d[2])

        # 공 영역에 Depth 값 쓰기
        # 용어 설명:
        # - 구체는 중심에서 거리에 따라 표면 깊이가 달라짐.
        # - 간단화: 원 영역을 균일하게 1.0m로 설정 (실제로는 곡면이지만).
        cv2.circle(
            depth,
            center=(int(ball_center_2d[0]), int(ball_center_2d[1])),
            radius=ball_radius_2d,
            color=1.0,  # 거리 1.0m (단일 값, Depth는 1채널)
            thickness=-1  # 채우기
        )

        # ===== 파란 상자의 Depth 추가 =====
        box_center_3d = np.array([0.5, 0.0, 2.0])
        box_size_3d = 0.3
        half_size = box_size_3d / 2.0

        # 상자 모서리 투영
        corners_3d = [
            box_center_3d + np.array([-half_size, -half_size, 0]),
            box_center_3d + np.array([half_size, -half_size, 0]),
            box_center_3d + np.array([half_size, half_size, 0]),
            box_center_3d + np.array([-half_size, half_size, 0]),
        ]
        corners_2d = [self.project_point(corner) for corner in corners_3d]
        pts = np.array(corners_2d, dtype=int).reshape((-1, 1, 2))

        # 다각형 내부를 2.0m로 채우기
        # 용어 설명:
        # - cv2.fillPoly: 다각형 내부를 특정 색상으로 채우기.
        # - 여기서는 "색상" 대신 "거리 값" 2.0을 씀.
        cv2.fillPoly(depth, [pts], color=2.0)

        return depth

    def project_point(self, point_3d):
        """
        3D 점을 2D 이미지 평면으로 투영 (Projection)

        입력: point_3d = [X, Y, Z] (카메라 좌표계, 미터)
        출력: point_2d = [u, v] (픽셀 좌표)

        Pinhole Camera Model (핀홀 카메라 모델):
        용어 설명:
        - Pinhole: "핀 구멍". 아주 작은 구멍 하나로만 빛이 들어오는 카메라.
        - 실제 카메라는 렌즈가 있지만, 수학적으로는 핀홀로 단순화 가능.

        투영 공식:
        u = fx * (X / Z) + cx
        v = fy * (Y / Z) + cy

        왜 Z로 나누나?
        - Z는 깊이 (카메라로부터 얼마나 멀리 있는가).
        - 멀리 있을수록 (Z 큰) 작게 보임 (u, v 작아짐).
        - 이게 바로 원근법 (Perspective).

        예시:
        - X=0, Y=0, Z=1.0 (정면 1m): u=cx, v=cy (이미지 중앙)
        - X=0.5, Y=0, Z=1.0 (오른쪽 50cm, 1m): u=cx+300 (오른쪽으로 치우침)
        - X=0.5, Y=0, Z=2.0 (오른쪽 50cm, 2m): u=cx+150 (덜 치우침, 멀어서)
        """
        X, Y, Z = point_3d

        # Z가 0이면 나눗셈 오류 (무한대). 방어 코드.
        # 용어 설명:
        # - Division by Zero: 0으로 나누기. 수학적으로 정의 안 됨. 컴퓨터는 에러 또는 Inf.
        if Z <= 0.0:
            Z = 0.0001  # 아주 작은 값으로 대체 (거의 카메라 렌즈 바로 앞)

        u = self.fx * (X / Z) + self.cx
        v = self.fy * (Y / Z) + self.cy

        return np.array([u, v])

    def generate_camera_info(self, stamp):
        """
        CameraInfo 메시지 생성

        이 메시지는 카메라의 "설명서"입니다.
        "나는 이런 특성의 카메라입니다" 라고 알려주는 거죠.

        포함 내용:
        - Intrinsic Matrix K: fx, fy, cx, cy
        - Distortion Coefficients D: k1, k2, p1, p2, k3 (렌즈 왜곡)
        - Image Size: width, height

        용어 설명:
        - Distortion(왜곡): 렌즈는 완벽하지 않아서 이미지가 휘거나 찌그러짐.
          - Radial Distortion(방사 왜곡): 중심에서 멀수록 휘어짐. 어안 렌즈.
          - Tangential Distortion(접선 왜곡): 렌즈와 센서가 평행하지 않을 때.
        - k1, k2, k3: Radial distortion 계수.
        - p1, p2: Tangential distortion 계수.

        우리 시뮬레이션은 "완벽한 카메라"이므로 왜곡 없음 (모두 0).
        """
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = 'camera_frame'

        # 이미지 크기
        msg.width = self.width
        msg.height = self.height

        # Intrinsic Matrix K (3x3을 1D 배열로)
        # 용어 설명:
        # - Row-major order: 행 우선 순서. 첫 행, 둘째 행, 셋째 행 순으로 나열.
        #   [[a, b, c],   →  [a, b, c, d, e, f, g, h, i]
        #    [d, e, f],
        #    [g, h, i]]
        msg.k = [
            self.fx, 0.0,     self.cx,  # 첫 행: fx, 0, cx
            0.0,     self.fy, self.cy,  # 둘째 행: 0, fy, cy
            0.0,     0.0,     1.0       # 셋째 행: 0, 0, 1
        ]

        # Distortion Coefficients D (왜곡 없음)
        # 용어 설명:
        # - [k1, k2, p1, p2, k3]: OpenCV 왜곡 모델 표준 순서.
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        # Distortion Model 이름
        # 용어 설명:
        # - 'plumb_bob': OpenCV 기본 왜곡 모델 이름. 실제 의미는 없고, 역사적 이름.
        # - 다른 모델: 'fisheye' (어안 렌즈), 'rational_polynomial' (고차 다항식) 등.
        msg.distortion_model = 'plumb_bob'

        # Rectification Matrix R (스테레오 카메라용, 단일 카메라는 항등 행렬)
        # 용어 설명:
        # - Rectification: "보정". 스테레오 카메라의 좌우 이미지를 평행하게 맞추는 과정.
        # - 단일 카메라는 필요 없으므로 항등 행렬 (아무것도 안 함).
        # - Identity Matrix(항등 행렬): 곱해도 변하지 않는 행렬. 곱셈의 1 역할.
        #   [[1, 0, 0],
        #    [0, 1, 0],
        #    [0, 0, 1]]
        msg.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]

        # Projection Matrix P (3x4, 스테레오용)
        # 용어 설명:
        # - Projection Matrix: Intrinsic + Extrinsic 합친 것.
        # - 스테레오 카메라는 Baseline(두 카메라 사이 거리) 정보 포함.
        # - 단일 카메라는 K와 동일, 마지막 열은 0.
        msg.p = [
            self.fx, 0.0,     self.cx, 0.0,  # K의 첫 행 + 0
            0.0,     self.fy, self.cy, 0.0,  # K의 둘째 행 + 0
            0.0,     0.0,     1.0,     0.0   # K의 셋째 행 + 0
        ]

        return msg


def main(args=None):
    rclpy.init(args=args)
    node = RGBDCameraSimulator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C 감지, 종료합니다...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
