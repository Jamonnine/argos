"""ARGOS Rerun 시각화 브릿지 — 3D 로봇 궤적/센서 실시간 시각화

DimOS 벤치마킹: Rerun 통합 패턴.
ARGOS: ROS2 토픽 구독 → Rerun 로깅.

rerun 미설치 시 no-op (stderr 경고만 출력, 노드 동작 유지).

Rerun이란:
    rerun.io — 로봇/AI 데이터 실시간 3D 시각화 도구.
    RViz2보다 설치 간단하고 웹 브라우저에서 확인 가능.
    pip install rerun-sdk 로 설치.

사용 예시:
    bridge = RerunBridge(app_name='argos_viz')

    # ROS2 콜백에서 호출
    def pose_callback(msg):
        bridge.log_robot_pose('argos1', msg)

    def scan_callback(msg):
        bridge.log_scan(msg)

    def thermal_callback(msg):
        bridge.log_thermal(msg)

    # 화재 경보
    bridge.log_fire_alert({'robot_id': 'argos1', 'x': 3.0, 'y': 4.0, 'temp': 120.0})
"""

from __future__ import annotations

import logging
from typing import Any, Optional

# Rerun 선택적 임포트 — 없으면 no-op
try:
    import rerun as rr
    import numpy as np
    RERUN_AVAILABLE = True
except ImportError:
    RERUN_AVAILABLE = False

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# RerunBridge — ROS2 메시지 → Rerun 로깅
# ---------------------------------------------------------------------------

class RerunBridge:
    """ROS2 토픽 데이터를 Rerun에 실시간으로 로깅한다.

    Rerun 미설치 시 모든 log_* 메서드는 no-op (경고 1회만 출력).
    노드 동작에는 전혀 영향 없음.

    엔티티 경로 설계 (Rerun 계층):
        robots/{robot_id}/pose     — 로봇 위치 (Transform3D)
        robots/{robot_id}/trail    — 이동 궤적 (LineStrips3D)
        sensors/lidar/scan         — LiDAR 포인트클라우드 (Points3D)
        sensors/thermal/image      — 열화상 이미지 (Image)
        events/fire_alerts         — 화재 경보 마커 (Boxes3D)
    """

    def __init__(self, app_name: str = 'argos_viz') -> None:
        self.app_name = app_name
        self._initialized = False
        self._warned_no_rerun = False

        # 궤적 버퍼: {robot_id: [(x, y, z), ...]}
        self._trails: dict[str, list] = {}

        if RERUN_AVAILABLE:
            self._init_rerun()
        else:
            logger.warning(
                'Rerun 미설치: pip install rerun-sdk. RerunBridge는 no-op 모드로 실행.'
            )

    def _init_rerun(self) -> None:
        """Rerun 뷰어 초기화."""
        try:
            rr.init(self.app_name, spawn=True)
            # 시간 타임라인 설정 (ROS2 시뮬레이션 시간 기준)
            rr.set_time_sequence('frame', 0)
            self._initialized = True
            logger.info('RerunBridge 초기화 완료: app=%s', self.app_name)
        except Exception as exc:
            logger.warning('Rerun 초기화 실패: %s', exc)
            self._initialized = False

    def _check_available(self) -> bool:
        """Rerun 사용 가능 여부 확인. 불가 시 경고 1회 출력."""
        if not RERUN_AVAILABLE or not self._initialized:
            if not self._warned_no_rerun:
                logger.warning(
                    'RerunBridge: Rerun 비활성화 상태 — 시각화 스킵'
                )
                self._warned_no_rerun = True
            return False
        return True

    # ------------------------------------------------------------------
    # 로봇 위치 로깅
    # ------------------------------------------------------------------

    def log_robot_pose(self, robot_id: str, pose: Any, frame_seq: Optional[int] = None) -> None:
        """로봇 위치를 Rerun에 로깅한다.

        인수:
            robot_id: 로봇 식별자 (e.g. 'argos1')
            pose: geometry_msgs/PoseStamped 또는 {'x': float, 'y': float, 'z': float,
                  'qx': float, 'qy': float, 'qz': float, 'qw': float} 딕셔너리
            frame_seq: 타임라인 시퀀스 번호 (None이면 자동 증가)
        """
        if not self._check_available():
            return

        try:
            # ROS2 메시지 또는 딕셔너리 모두 지원
            if hasattr(pose, 'pose'):
                # geometry_msgs/PoseStamped
                p = pose.pose.position
                o = pose.pose.orientation
                x, y, z = p.x, p.y, p.z
                qx, qy, qz, qw = o.x, o.y, o.z, o.w
            elif hasattr(pose, 'position'):
                # geometry_msgs/Pose
                p = pose.position
                o = pose.orientation
                x, y, z = p.x, p.y, p.z
                qx, qy, qz, qw = o.x, o.y, o.z, o.w
            else:
                # 딕셔너리 폴백
                x = float(pose.get('x', 0.0))
                y = float(pose.get('y', 0.0))
                z = float(pose.get('z', 0.0))
                qx = float(pose.get('qx', 0.0))
                qy = float(pose.get('qy', 0.0))
                qz = float(pose.get('qz', 0.0))
                qw = float(pose.get('qw', 1.0))

            if frame_seq is not None:
                rr.set_time_sequence('frame', frame_seq)

            # 로봇 Transform 로깅
            rr.log(
                f'robots/{robot_id}/pose',
                rr.Transform3D(
                    translation=[x, y, z],
                    rotation=rr.Quaternion(xyzw=[qx, qy, qz, qw]),
                ),
            )

            # 궤적 누적 로깅
            if robot_id not in self._trails:
                self._trails[robot_id] = []
            self._trails[robot_id].append([x, y, z])
            # 최근 200포인트만 유지
            if len(self._trails[robot_id]) > 200:
                self._trails[robot_id] = self._trails[robot_id][-200:]

            if len(self._trails[robot_id]) >= 2:
                rr.log(
                    f'robots/{robot_id}/trail',
                    rr.LineStrips3D([self._trails[robot_id]]),
                )

        except Exception as exc:
            logger.debug('log_robot_pose 오류 (robot_id=%s): %s', robot_id, exc)

    # ------------------------------------------------------------------
    # LiDAR 스캔 로깅
    # ------------------------------------------------------------------

    def log_scan(self, scan_msg: Any) -> None:
        """LiDAR 스캔 데이터를 Rerun 포인트클라우드로 로깅한다.

        인수:
            scan_msg: sensor_msgs/LaserScan 또는 sensor_msgs/PointCloud2
        """
        if not self._check_available():
            return

        try:
            if hasattr(scan_msg, 'ranges'):
                # LaserScan → 2D 포인트 변환
                import math
                points = []
                angle = scan_msg.angle_min
                for r in scan_msg.ranges:
                    if scan_msg.range_min <= r <= scan_msg.range_max:
                        points.append([
                            r * math.cos(angle),
                            r * math.sin(angle),
                            0.0,
                        ])
                    angle += scan_msg.angle_increment

                if points:
                    rr.log(
                        'sensors/lidar/scan',
                        rr.Points3D(points, colors=[0, 200, 255]),
                    )
            else:
                # PointCloud2: 원시 로깅 (상세 변환 생략)
                logger.debug('log_scan: PointCloud2 원시 데이터 (변환 미구현)')

        except Exception as exc:
            logger.debug('log_scan 오류: %s', exc)

    # ------------------------------------------------------------------
    # 열화상 이미지 로깅
    # ------------------------------------------------------------------

    def log_thermal(self, image_msg: Any) -> None:
        """열화상 이미지를 Rerun에 로깅한다.

        인수:
            image_msg: sensor_msgs/Image (encoding=mono8 또는 16UC1)
                       또는 {'data': bytes, 'width': int, 'height': int} 딕셔너리
        """
        if not self._check_available():
            return

        try:
            if hasattr(image_msg, 'data') and hasattr(image_msg, 'width'):
                width = image_msg.width
                height = image_msg.height
                # numpy 배열로 변환
                img_array = np.frombuffer(bytes(image_msg.data), dtype=np.uint8)
                if img_array.size == width * height:
                    img_array = img_array.reshape((height, width))
                    # 컬러맵 적용 (열화상: 낮은값=파랑, 높은값=빨강)
                    # 간단히 단채널 이미지로 로깅
                    rr.log('sensors/thermal/image', rr.Image(img_array))
                else:
                    logger.debug('log_thermal: 이미지 크기 불일치')
            else:
                logger.debug('log_thermal: 지원하지 않는 이미지 형식')

        except Exception as exc:
            logger.debug('log_thermal 오류: %s', exc)

    # ------------------------------------------------------------------
    # 화재 경보 로깅
    # ------------------------------------------------------------------

    def log_fire_alert(self, alert_msg: Any) -> None:
        """화재 경보를 Rerun 3D 마커로 로깅한다.

        인수:
            alert_msg: argos_interfaces/FireAlert 또는
                       {'robot_id': str, 'x': float, 'y': float, 'temp': float} 딕셔너리
        """
        if not self._check_available():
            return

        try:
            if hasattr(alert_msg, 'location'):
                # FireAlert msg
                x = alert_msg.location.x
                y = alert_msg.location.y
                z = getattr(alert_msg.location, 'z', 0.0)
                temp = getattr(alert_msg, 'temperature', 0.0)
                robot_id = getattr(alert_msg, 'robot_id', 'unknown')
            else:
                # 딕셔너리 폴백
                x = float(alert_msg.get('x', 0.0))
                y = float(alert_msg.get('y', 0.0))
                z = float(alert_msg.get('z', 0.0))
                temp = float(alert_msg.get('temp', 0.0))
                robot_id = str(alert_msg.get('robot_id', 'unknown'))

            # 화재 위치를 빨간 박스로 표시
            rr.log(
                f'events/fire_alerts/{robot_id}',
                rr.Boxes3D(
                    centers=[[x, y, z + 0.5]],
                    half_sizes=[[0.3, 0.3, 0.5]],
                    colors=[[255, 50, 0, 200]],
                    labels=[f'FIRE {temp:.0f}°C'],
                ),
            )
            logger.info(
                'RerunBridge: 화재 경보 로깅 (robot=%s, x=%.1f, y=%.1f, temp=%.1f°C)',
                robot_id, x, y, temp,
            )

        except Exception as exc:
            logger.debug('log_fire_alert 오류: %s', exc)

    def __repr__(self) -> str:
        return (
            f'RerunBridge(app={self.app_name!r}, '
            f'rerun={RERUN_AVAILABLE}, initialized={self._initialized})'
        )
