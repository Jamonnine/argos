"""kalman_tracker.py — 화점/피해자 위치 시간 추적 (Kalman 필터)

센서 퓨전 전문가 권고: 단일 프레임 감지가 아닌 시계열 추적.
화점 위치의 이동/확산 속도를 추정하여 오탐 필터링 + 확산 예측.

참조: Fire SLAM 논문 — YOLOv8n + Kalman 필터로 3D 화점 위치 실시간 추정.

강화 내역 (W8 권고사항):
- 시간 기반 Kalman 업데이트 (dt 적용)
- 화재 확산 반경 추정 (get_fire_radius)
- 화점-피해자 근접 경고 (check_victim_fire_proximity)
- 트랙 통계 (get_statistics)
- 확산 추세 판정 (get_fire_spread_trend)

확산 예측 기능 (신규):
- predict_fire_positions: 속도 벡터 기반 N분 후 화점 위치 예측
- get_evacuation_direction: 예측 화점 반대 방향 대피 경로 계산
"""

import time
import numpy as np
from dataclasses import dataclass, field
from typing import Optional, Dict, List


@dataclass
class TrackedObject:
    """추적 대상 (화점 또는 피해자)."""
    track_id: int
    class_name: str           # 'fire', 'victim', 'smoke'
    x: float                  # 현재 추정 위치 X
    y: float                  # 현재 추정 위치 Y
    vx: float = 0.0           # 추정 속도 X (m/s)
    vy: float = 0.0           # 추정 속도 Y (m/s)
    confidence: float = 0.5   # 추적 신뢰도
    age: int = 0              # 프레임 수 (오래될수록 높음)
    missed_count: int = 0     # 연속 미감지 횟수
    covariance: float = 1.0   # 위치 불확실성
    # 시간 기반 추적을 위한 추가 필드
    last_update_time: float = 0.0   # 마지막 업데이트 타임스탬프 (unix time)
    detection_area: float = 1.0     # 감지된 객체 면적 추정값 (m²)
    spread_history: List[float] = field(default_factory=list)  # 확산 속도 이력


class SimpleKalmanTracker:
    """단순 2D Kalman 필터 기반 다중 객체 추적기.

    사용:
        tracker = SimpleKalmanTracker()
        # 매 프레임:
        tracks = tracker.update(detections)
        # detections = [(x, y, class_name, confidence), ...]
    """

    def __init__(self, max_distance: float = 3.0, max_missed: int = 10,
                 process_noise: float = 0.1, measurement_noise: float = 0.5):
        """
        Args:
            max_distance: 감지-트랙 매칭 최대 거리 (m)
            max_missed: 연속 미감지 시 트랙 삭제 기준
            process_noise: 프로세스 노이즈 (이동 불확실성)
            measurement_noise: 측정 노이즈 (센서 오차)
        """
        self.tracks: Dict[int, TrackedObject] = {}
        self._next_id = 0
        self.max_distance = max_distance
        self.max_missed = max_missed
        self.Q = process_noise      # 프로세스 노이즈
        self.R = measurement_noise  # 측정 노이즈

    def update(self, detections: list, timestamp: Optional[float] = None) -> list:
        """새 감지 결과로 트랙 갱신.

        Args:
            detections: [(x, y, class_name, confidence), ...]
                        또는 [(x, y, class_name, confidence, area), ...]
                        area는 선택적 (감지 객체 면적 m², 없으면 기본값 1.0)
            timestamp: 현재 타임스탬프 (unix time, 초 단위).
                       None이면 프레임 단위 동작 (기존 방식 유지).

        Returns:
            활성 TrackedObject 리스트
        """
        # 타임스탬프 결정: None이면 현재 시각 사용 (프레임 단위 호환)
        now = timestamp if timestamp is not None else time.time()

        # 1. 예측 단계 (시간 기반 또는 프레임 기반)
        for track in self.tracks.values():
            if timestamp is not None and track.last_update_time > 0:
                # 시간 기반: dt만큼 위치 예측
                dt = now - track.last_update_time
                dt = max(0.0, min(dt, 5.0))  # dt 이상값 클램핑 (최대 5초)
                track.x += track.vx * dt
                track.y += track.vy * dt
                track.covariance += self.Q * dt
            else:
                # 프레임 기반: 기존 동작 유지 (하위 호환)
                track.x += track.vx
                track.y += track.vy
                track.covariance += self.Q
            track.missed_count += 1

        # 2. 매칭 (헝가리안 대신 단순 최근접)
        matched_tracks = set()
        matched_dets = set()

        for det_idx, det in enumerate(detections):
            # area 필드 선택적 파싱
            if len(det) >= 5:
                dx, dy, cls, conf, area = det[0], det[1], det[2], det[3], det[4]
            else:
                dx, dy, cls, conf = det[0], det[1], det[2], det[3]
                area = 1.0

            best_track_id = None
            best_dist = self.max_distance

            for tid, track in self.tracks.items():
                if tid in matched_tracks:
                    continue
                dist = np.sqrt((track.x - dx)**2 + (track.y - dy)**2)
                if dist < best_dist:
                    best_dist = dist
                    best_track_id = tid

            if best_track_id is not None:
                # 갱신 단계 (Kalman gain)
                track = self.tracks[best_track_id]
                K = track.covariance / (track.covariance + self.R)

                # 속도 추정 (이전 위치 → 현재 위치, dt 반영)
                if timestamp is not None and track.last_update_time > 0:
                    dt = max(1e-6, now - track.last_update_time)
                    raw_vx = (dx - track.x) / dt
                    raw_vy = (dy - track.y) / dt
                else:
                    raw_vx = dx - track.x
                    raw_vy = dy - track.y

                track.vx = 0.8 * track.vx + 0.2 * raw_vx
                track.vy = 0.8 * track.vy + 0.2 * raw_vy

                # 위치 갱신
                track.x += K * (dx - track.x)
                track.y += K * (dy - track.y)
                track.covariance *= (1 - K)
                track.confidence = 0.7 * track.confidence + 0.3 * conf
                track.age += 1
                track.missed_count = 0
                track.last_update_time = now
                track.detection_area = area

                # 화점 확산 속도 이력 기록 (최근 20개)
                if cls == 'fire':
                    speed = float(np.sqrt(track.vx**2 + track.vy**2))
                    track.spread_history.append(speed)
                    if len(track.spread_history) > 20:
                        track.spread_history.pop(0)

                matched_tracks.add(best_track_id)
                matched_dets.add(det_idx)

        # 3. 새 트랙 생성 (미매칭 감지)
        for det_idx, det in enumerate(detections):
            if det_idx not in matched_dets:
                if len(det) >= 5:
                    dx, dy, cls, conf, area = det[0], det[1], det[2], det[3], det[4]
                else:
                    dx, dy, cls, conf = det[0], det[1], det[2], det[3]
                    area = 1.0
                self.tracks[self._next_id] = TrackedObject(
                    track_id=self._next_id,
                    class_name=cls,
                    x=dx, y=dy,
                    confidence=conf,
                    last_update_time=now,
                    detection_area=area,
                )
                self._next_id += 1

        # 4. 오래된 트랙 삭제
        to_remove = [
            tid for tid, t in self.tracks.items()
            if t.missed_count > self.max_missed
        ]
        for tid in to_remove:
            del self.tracks[tid]

        return list(self.tracks.values())

    def get_fire_spread_rate(self) -> Optional[float]:
        """화점들의 평균 확산 속도 (m/s).

        Returns:
            확산 속도. 화점 트랙이 없으면 None.
        """
        fire_tracks = [
            t for t in self.tracks.values()
            if t.class_name == 'fire' and t.age > 3
        ]
        if not fire_tracks:
            return None
        speeds = [np.sqrt(t.vx**2 + t.vy**2) for t in fire_tracks]
        return float(np.mean(speeds))

    def get_fire_radius(self, track_id: int) -> Optional[float]:
        """화점 트랙의 추정 확산 반경 (m).

        covariance(위치 불확실성)와 detection_area(감지 면적)를 결합하여
        현재 화점이 차지하는 실효 반경을 추정합니다.

        Args:
            track_id: 조회할 트랙 ID

        Returns:
            추정 반경 (m). 해당 트랙이 없거나 화점이 아니면 None.
        """
        track = self.tracks.get(track_id)
        if track is None or track.class_name != 'fire':
            return None

        # 감지 면적 기반 기본 반경 (원형 근사)
        area_radius = float(np.sqrt(track.detection_area / np.pi))

        # covariance 기반 불확실성 반경 (1-sigma)
        uncertainty_radius = float(np.sqrt(track.covariance))

        # 두 반경의 RSS(Root Sum of Squares) 결합
        combined_radius = float(np.sqrt(area_radius**2 + uncertainty_radius**2))
        return combined_radius

    def check_victim_fire_proximity(
        self, radius_threshold: float = 3.0
    ) -> List[dict]:
        """화점과 피해자 간 근접 경고 목록 반환.

        화점 트랙과 피해자 트랙 간 거리를 계산하여
        radius_threshold 이내이면 경고를 생성합니다.

        Args:
            radius_threshold: 경고 발동 거리 기준 (m, 기본 3.0m)

        Returns:
            경고 딕셔너리 리스트. 각 항목:
            {
                'fire_track_id': int,
                'victim_track_id': int,
                'distance': float,     # 실제 거리 (m)
                'fire_radius': float,  # 화점 추정 반경 (m)
                'severity': str,       # 'CRITICAL' | 'WARNING'
            }
        """
        fire_tracks = [
            t for t in self.tracks.values() if t.class_name == 'fire'
        ]
        victim_tracks = [
            t for t in self.tracks.values() if t.class_name == 'victim'
        ]

        warnings = []
        for fire in fire_tracks:
            fire_radius = self.get_fire_radius(fire.track_id) or 1.0
            for victim in victim_tracks:
                dist = float(np.sqrt(
                    (fire.x - victim.x)**2 + (fire.y - victim.y)**2
                ))
                if dist <= radius_threshold:
                    # 화점 반경 내부면 CRITICAL, 임계 거리 내부면 WARNING
                    severity = 'CRITICAL' if dist <= fire_radius else 'WARNING'
                    warnings.append({
                        'fire_track_id': fire.track_id,
                        'victim_track_id': victim.track_id,
                        'distance': dist,
                        'fire_radius': fire_radius,
                        'severity': severity,
                    })

        # 거리 가까운 순 정렬
        warnings.sort(key=lambda w: w['distance'])
        return warnings

    def get_statistics(self) -> dict:
        """현재 트랙 전체에 대한 요약 통계.

        Returns:
            {
                'total': int,             # 전체 활성 트랙 수
                'fire_count': int,        # 화점 트랙 수
                'victim_count': int,      # 피해자 트랙 수
                'smoke_count': int,       # 연기 트랙 수
                'avg_confidence': float,  # 전체 평균 신뢰도
                'oldest_age': int,        # 가장 오래된 트랙 age (프레임)
                'oldest_track_id': Optional[int],  # 가장 오래된 트랙 ID
            }
        """
        tracks = list(self.tracks.values())
        if not tracks:
            return {
                'total': 0,
                'fire_count': 0,
                'victim_count': 0,
                'smoke_count': 0,
                'avg_confidence': 0.0,
                'oldest_age': 0,
                'oldest_track_id': None,
            }

        fire_tracks = [t for t in tracks if t.class_name == 'fire']
        victim_tracks = [t for t in tracks if t.class_name == 'victim']
        smoke_tracks = [t for t in tracks if t.class_name == 'smoke']

        avg_confidence = float(np.mean([t.confidence for t in tracks]))

        oldest = max(tracks, key=lambda t: t.age)

        return {
            'total': len(tracks),
            'fire_count': len(fire_tracks),
            'victim_count': len(victim_tracks),
            'smoke_count': len(smoke_tracks),
            'avg_confidence': avg_confidence,
            'oldest_age': oldest.age,
            'oldest_track_id': oldest.track_id,
        }

    def predict_fire_positions(
        self, horizon_sec: float = 300.0
    ) -> List[dict]:
        """속도 벡터 기반 화점 위치 예측.

        현재 추정된 속도(vx, vy)를 선형 외삽하여
        horizon_sec 초 후의 화점 위치를 계산합니다.

        소방 현장 기준: 목조 건물 화재 확산 속도 평균 0.3~0.8 m/min.
        예측 지평선 기본값 5분(300s) — 대피 판단 골든타임.

        Args:
            horizon_sec: 예측 지평선 (초, 기본 300s = 5분)

        Returns:
            예측 딕셔너리 리스트. 각 항목:
            {
                'track_id': int,
                'current_x': float,      # 현재 위치 X (m)
                'current_y': float,      # 현재 위치 Y (m)
                'predicted_x': float,    # 예측 위치 X (m)
                'predicted_y': float,    # 예측 위치 Y (m)
                'speed_mps': float,      # 현재 이동 속도 (m/s)
                'heading_rad': float,    # 이동 방향 (라디안, atan2)
                'confidence': float,     # 트랙 신뢰도
                'horizon_sec': float,    # 예측 지평선 (초)
            }
        """
        fire_tracks = [
            t for t in self.tracks.values()
            if t.class_name == 'fire' and t.age > 3
        ]

        predictions = []
        for track in fire_tracks:
            speed = float(np.sqrt(track.vx**2 + track.vy**2))
            heading = float(np.arctan2(track.vy, track.vx))

            # 선형 외삽: x(t) = x0 + vx * dt
            pred_x = track.x + track.vx * horizon_sec
            pred_y = track.y + track.vy * horizon_sec

            predictions.append({
                'track_id': track.track_id,
                'current_x': track.x,
                'current_y': track.y,
                'predicted_x': pred_x,
                'predicted_y': pred_y,
                'speed_mps': speed,
                'heading_rad': heading,
                'confidence': track.confidence,
                'horizon_sec': horizon_sec,
            })

        return predictions

    def get_evacuation_direction(
        self,
        robot_x: float,
        robot_y: float,
        safe_distance: float = 10.0,
        horizon_sec: float = 300.0,
    ) -> Optional[dict]:
        """예측 화점 반대 방향으로 안전 구역 계산.

        모든 예측 화점의 무게중심을 구하고,
        현재 로봇 위치에서 무게중심 반대 방향으로
        safe_distance 만큼 이동한 지점을 대피 목적지로 반환합니다.

        Args:
            robot_x: 로봇 현재 위치 X (m)
            robot_y: 로봇 현재 위치 Y (m)
            safe_distance: 대피 목적지까지의 거리 (m, 기본 10m)
            horizon_sec: 화점 예측 지평선 (초)

        Returns:
            {
                'safe_x': float,          # 대피 목적지 X (m)
                'safe_y': float,          # 대피 목적지 Y (m)
                'away_heading_rad': float, # 대피 방향 (라디안)
                'fire_centroid_x': float, # 화점 무게중심 X
                'fire_centroid_y': float, # 화점 무게중심 Y
                'fire_count': int,        # 예측에 사용된 화점 수
            }
            화점이 없으면 None.
        """
        predictions = self.predict_fire_positions(horizon_sec)
        if not predictions:
            return None

        # 예측 화점 무게중심 (신뢰도 가중 평균)
        total_weight = sum(p['confidence'] for p in predictions)
        if total_weight < 1e-6:
            total_weight = len(predictions)
            centroid_x = float(np.mean([p['predicted_x'] for p in predictions]))
            centroid_y = float(np.mean([p['predicted_y'] for p in predictions]))
        else:
            centroid_x = sum(
                p['predicted_x'] * p['confidence'] for p in predictions
            ) / total_weight
            centroid_y = sum(
                p['predicted_y'] * p['confidence'] for p in predictions
            ) / total_weight

        # 로봇 → 화점 무게중심 벡터
        dx = centroid_x - robot_x
        dy = centroid_y - robot_y
        dist_to_centroid = float(np.sqrt(dx**2 + dy**2))

        if dist_to_centroid < 1e-3:
            # 로봇이 화점 무게중심과 거의 동일 위치 → 임의 방향(북쪽)으로 대피
            away_heading = float(np.pi / 2)
        else:
            # 화점 방향의 역방향
            toward_heading = float(np.arctan2(dy, dx))
            away_heading = float(toward_heading + np.pi)

        # 대피 목적지 = 현재 위치 + 역방향으로 safe_distance
        safe_x = robot_x + safe_distance * float(np.cos(away_heading))
        safe_y = robot_y + safe_distance * float(np.sin(away_heading))

        return {
            'safe_x': safe_x,
            'safe_y': safe_y,
            'away_heading_rad': away_heading,
            'fire_centroid_x': centroid_x,
            'fire_centroid_y': centroid_y,
            'fire_count': len(predictions),
        }

    def get_fire_spread_trend(self, window: int = 5) -> dict:
        """화점 확산 추세 판정.

        각 화점 트랙의 spread_history(최근 확산 속도 이력)를 분석하여
        전체적인 확산 추세를 반환합니다.

        Args:
            window: 추세 비교에 사용할 최근 이력 윈도우 크기 (기본 5)

        Returns:
            {
                'trend': str,             # 'accelerating'|'steady'|'decelerating'|'unknown'
                'avg_recent_speed': float,  # 최근 평균 확산 속도 (m/s)
                'avg_early_speed': float,   # 초기 평균 확산 속도 (m/s)
                'fire_count': int,          # 분석에 사용된 화점 트랙 수
            }
        """
        # 이력이 충분한 화점 트랙만 선택 (window*2 이상)
        fire_tracks = [
            t for t in self.tracks.values()
            if t.class_name == 'fire' and len(t.spread_history) >= window * 2
        ]

        if not fire_tracks:
            return {
                'trend': 'unknown',
                'avg_recent_speed': 0.0,
                'avg_early_speed': 0.0,
                'fire_count': 0,
            }

        recent_speeds = []
        early_speeds = []
        for track in fire_tracks:
            history = track.spread_history
            recent_speeds.extend(history[-window:])
            early_speeds.extend(history[:window])

        avg_recent = float(np.mean(recent_speeds))
        avg_early = float(np.mean(early_speeds))

        # 변화율 10% 이상이면 추세 있다고 판단
        if avg_early < 1e-6:
            trend = 'unknown'
        else:
            ratio = (avg_recent - avg_early) / avg_early
            if ratio > 0.1:
                trend = 'accelerating'
            elif ratio < -0.1:
                trend = 'decelerating'
            else:
                trend = 'steady'

        return {
            'trend': trend,
            'avg_recent_speed': avg_recent,
            'avg_early_speed': avg_early,
            'fire_count': len(fire_tracks),
        }
