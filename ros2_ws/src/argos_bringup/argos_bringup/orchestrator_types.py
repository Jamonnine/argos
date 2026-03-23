"""오케스트레이터 공유 타입 + 상수.

G-1 SRP 리팩토링의 일환으로 orchestrator_node.py에서 추출.
sensor_fusion.py, robot_dispatcher.py, orchestrator_node.py가 공유.
"""
from dataclasses import dataclass, field
from typing import Optional

from argos_interfaces.msg import RobotStatus


# ── 소방 전술 모듈 (소방 전문가 권고) ──

FIRE_TACTICS = {
    'electrical': {
        'suppression': 'water_fog',
        'robot_capable': True,
        'human_mandatory': ['전원차단'],
        'approach_distance': 5.0,
    },
    'oil': {
        'suppression': 'foam_spray',
        'robot_capable': True,
        'human_mandatory': ['포말탱크 연결'],
        'approach_distance': 3.0,
    },
    'gas': {
        'suppression': 'ventilation',
        'robot_capable': False,
        'human_mandatory': ['가스밸브 차단', '환기'],
        'approach_distance': 10.0,
    },
    'general': {
        'suppression': 'water_fog',
        'robot_capable': True,
        'human_mandatory': [],
        'approach_distance': 2.0,
    },
}

# 센서 합의 반경 (m)
CONSENSUS_RADIUS = 5.0
# 충돌 방지 안전 거리 (m)
COLLISION_SAFE_DISTANCE = 2.0


# ── 로봇 레코드 ──

class RobotRecord:
    """오케스트레이터가 관리하는 개별 로봇 레코드."""

    def __init__(self, robot_id: str):
        self.robot_id = robot_id
        self.robot_type = 'ugv'
        self.state = RobotStatus.STATE_IDLE
        self.last_seen = 0.0
        self.battery = 100.0
        self.current_mission = 'idle'
        self.mission_progress = 0.0
        self.frontiers_remaining = 0
        self.coverage = 0.0
        self.capabilities = []
        self.pose = None
        self.comm_lost = False
        self.battery_warned = False
        self.battery_critical_acted = False
        self.mission_lock = None
        self.assigned_target = None
        self.last_seq = -1
        self.packet_loss_count = 0
        self.hose_remaining_m: float = -1.0
        self.hose_kink_risk: float = 0.0
        self.hose_charged: bool = False
        self.hose_path: list = []
        self.role: str = 'explore'


# ── 명령 객체 (Side-Effect Boundary) ──

@dataclass
class StopRobotCommand:
    """로봇 정지 명령."""
    robot_id: str


@dataclass
class WaypointCommand:
    """드론/UGV 목표 좌표."""
    robot_id: str
    x: float
    y: float
    z: float = 0.0
    frame_id: str = 'map'


@dataclass
class AutonomyModeCommand:
    """자율 모드 전환."""
    robot_id: str
    mode: str  # 'CENTRALIZED' | 'LOCAL_AUTONOMY'


@dataclass
class HoseConflictEvent:
    """호스 충돌 이벤트."""
    rid_a: str
    rid_b: str
    resolution: str


@dataclass
class StageTransition:
    """임무 단계 전환 요청."""
    new_stage: int
    reason: str
