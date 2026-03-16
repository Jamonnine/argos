"""ARGOS SkillLibrary — 로봇 능력 동적 등록/쿼리/조합

DimOS 벤치마킹: AbstractRobotSkill + SkillLibrary 패턴.
ARGOS 네이티브: ROS2 액션/서비스 기반, DimOS 직접 의존 없음.

사용 예시:
    library = SkillLibrary()
    library.register(PatrolSkill(robot_id='argos1'))
    library.register(DetectFireSkill(robot_id='argos1'))

    # 로봇별 능력 조회
    skills = library.get_robot_skills('argos1')

    # 타입별 능력 조회
    patrol_skills = library.get_skills_by_type('patrol')

    # 특정 하드웨어 능력이 필요한 스킬 수행 가능 로봇 조회
    robots = library.get_capable_robots(['thermal', 'lidar'])

    # MCP 도구 형식 변환 (Claude API 호환)
    tools = library.to_mcp_tools()
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any


# ---------------------------------------------------------------------------
# 스킬 디스크립터 — 스킬 메타데이터 구조체
# ---------------------------------------------------------------------------

@dataclass
class SkillDescriptor:
    """스킬 메타데이터. 등록·쿼리·MCP 변환 모두 이 구조체 기준."""
    name: str                          # 'patrol_area' — 고유 식별자
    skill_type: str                    # 'patrol' | 'detect_fire' | 'return_home' | 'rescue'
    robot_id: str                      # 'argos1' — 어느 로봇의 스킬인지
    robot_type: str                    # 'ugv' | 'drone'
    description: str                   # 자연어 설명 (Claude가 읽음)
    parameters: dict = field(default_factory=dict)  # {name: {type, default, description}}
    required_capabilities: list = field(default_factory=list)  # ['thermal', 'lidar']
    ros2_action: str = ''              # ROS2 액션 서버 이름 (e.g. 'navigate_to_pose')
    ros2_service: str = ''             # ROS2 서비스 이름 (ros2_action 없을 때 대안)


# ---------------------------------------------------------------------------
# 추상 기본 스킬
# ---------------------------------------------------------------------------

class AbstractRobotSkill(ABC):
    """모든 로봇 스킬의 기본 클래스.

    구체 스킬은 get_descriptor()와 validate_params()를 반드시 구현해야 한다.
    """

    def __init__(self, robot_id: str, robot_type: str = 'ugv') -> None:
        self.robot_id = robot_id
        self.robot_type = robot_type

    @abstractmethod
    def get_descriptor(self) -> SkillDescriptor:
        """스킬 메타데이터를 반환한다."""
        ...

    def validate_params(self, **kwargs: Any) -> bool:
        """파라미터 유효성 검사. 기본 구현은 항상 True.

        구체 스킬에서 오버라이드하여 타입·범위 검사 추가 가능.
        """
        return True

    def to_mcp_tool(self) -> dict:
        """MCP Tool 형식으로 변환 (Claude API 호환).

        반환 형식은 Anthropic MCP Tool Schema를 따른다.
        """
        desc = self.get_descriptor()
        # MCP inputSchema: JSON Schema 형식으로 파라미터 변환
        properties: dict = {}
        required_params: list = []
        for param_name, meta in desc.parameters.items():
            prop: dict = {'description': meta.get('description', '')}
            # 타입 매핑: Python 타입 힌트 → JSON Schema 타입
            type_map = {'float': 'number', 'int': 'integer', 'str': 'string', 'bool': 'boolean'}
            prop['type'] = type_map.get(meta.get('type', 'str'), 'string')
            if 'default' in meta:
                prop['default'] = meta['default']
            else:
                required_params.append(param_name)
            properties[param_name] = prop

        return {
            'name': desc.name,
            'description': desc.description,
            'inputSchema': {
                'type': 'object',
                'properties': properties,
                'required': required_params,
            },
        }


# ---------------------------------------------------------------------------
# 구체 스킬 구현 4종
# ---------------------------------------------------------------------------

class PatrolSkill(AbstractRobotSkill):
    """지정 웨이포인트 순찰 스킬.

    Nav2 navigate_to_pose 액션을 반복 호출하여 순찰 루트를 순회한다.
    """

    def get_descriptor(self) -> SkillDescriptor:
        return SkillDescriptor(
            name=f'patrol_{self.robot_id}',
            skill_type='patrol',
            robot_id=self.robot_id,
            robot_type=self.robot_type,
            description=(
                f'로봇 {self.robot_id}가 지정된 웨이포인트 목록을 순서대로 순찰한다. '
                '화재 현장 건물 내부 탐색에 사용. Nav2 액션 기반.'
            ),
            parameters={
                'waypoints': {
                    'type': 'str',
                    'description': '순찰 웨이포인트 JSON 배열 (x,y,yaw)',
                },
                'loop': {
                    'type': 'bool',
                    'default': False,
                    'description': 'True이면 웨이포인트 완주 후 반복',
                },
                'speed': {
                    'type': 'float',
                    'default': 0.5,
                    'description': '이동 속도 (m/s)',
                },
            },
            required_capabilities=['lidar', 'nav2'],
            ros2_action='navigate_to_pose',
        )

    def validate_params(self, **kwargs: Any) -> bool:
        """waypoints 파라미터 존재 여부 및 speed 범위 검사."""
        if 'waypoints' not in kwargs:
            return False
        speed = kwargs.get('speed', 0.5)
        return 0.0 < float(speed) <= 2.0


class DetectFireSkill(AbstractRobotSkill):
    """열화상 센서 기반 화점 탐지 스킬.

    perception_bridge_node의 thermal 파이프라인을 활성화하고
    화점 좌표를 /argos/fire_alert 토픽으로 발행한다.
    """

    def get_descriptor(self) -> SkillDescriptor:
        return SkillDescriptor(
            name=f'detect_fire_{self.robot_id}',
            skill_type='detect_fire',
            robot_id=self.robot_id,
            robot_type=self.robot_type,
            description=(
                f'로봇 {self.robot_id}의 열화상 카메라로 화점을 탐지한다. '
                '온도 임계값 초과 영역을 지도에 마킹. hotspot_detector_node 연동.'
            ),
            parameters={
                'temperature_threshold': {
                    'type': 'float',
                    'default': 60.0,
                    'description': '화점 판단 온도 임계값 (°C)',
                },
                'min_area': {
                    'type': 'int',
                    'default': 10,
                    'description': '최소 화점 픽셀 면적 (노이즈 필터)',
                },
            },
            required_capabilities=['thermal'],
            ros2_service='/hotspot_detector/detect',
        )

    def validate_params(self, **kwargs: Any) -> bool:
        """temperature_threshold가 현실적 범위(20~300°C)인지 검사."""
        threshold = kwargs.get('temperature_threshold', 60.0)
        return 20.0 <= float(threshold) <= 300.0


class ReturnHomeSkill(AbstractRobotSkill):
    """귀환 베이스 스테이션 복귀 스킬.

    배터리 부족 또는 임무 완료 시 지정된 홈 좌표로 자율 복귀.
    """

    def get_descriptor(self) -> SkillDescriptor:
        return SkillDescriptor(
            name=f'return_home_{self.robot_id}',
            skill_type='return_home',
            robot_id=self.robot_id,
            robot_type=self.robot_type,
            description=(
                f'로봇 {self.robot_id}가 베이스 스테이션으로 복귀한다. '
                '배터리 임계값 도달 또는 임무 완료 시 자동 트리거.'
            ),
            parameters={
                'home_x': {
                    'type': 'float',
                    'default': 0.0,
                    'description': '홈 X 좌표 (m)',
                },
                'home_y': {
                    'type': 'float',
                    'default': 0.0,
                    'description': '홈 Y 좌표 (m)',
                },
                'emergency': {
                    'type': 'bool',
                    'default': False,
                    'description': 'True이면 최단 경로로 즉시 복귀',
                },
            },
            required_capabilities=['nav2'],
            ros2_action='navigate_to_pose',
        )


class RescueSkill(AbstractRobotSkill):
    """요구조자 위치 마킹 및 구조팀 안내 스킬.

    victim_detector_node가 감지한 요구조자 좌표를
    오케스트레이터에게 보고하고 구조팀 진입 경로를 안내한다.
    """

    def get_descriptor(self) -> SkillDescriptor:
        return SkillDescriptor(
            name=f'rescue_{self.robot_id}',
            skill_type='rescue',
            robot_id=self.robot_id,
            robot_type=self.robot_type,
            description=(
                f'로봇 {self.robot_id}가 요구조자를 탐지하고 구조 지원을 수행한다. '
                '음성 탐지 + 열화상 조합으로 생존자 위치 확인. '
                'victim_detector_node + audio_detector_node 연동.'
            ),
            parameters={
                'search_radius': {
                    'type': 'float',
                    'default': 5.0,
                    'description': '탐색 반경 (m)',
                },
                'use_audio': {
                    'type': 'bool',
                    'default': True,
                    'description': '음성 탐지 센서 병행 사용 여부',
                },
            },
            required_capabilities=['thermal', 'camera'],
            ros2_service='/victim_detector/search',
        )

    def validate_params(self, **kwargs: Any) -> bool:
        """search_radius가 양수인지 검사."""
        radius = kwargs.get('search_radius', 5.0)
        return float(radius) > 0.0


# ---------------------------------------------------------------------------
# SkillLibrary — 중앙 스킬 레지스트리
# ---------------------------------------------------------------------------

class SkillLibrary:
    """로봇 스킬 중앙 레지스트리.

    오케스트레이터가 "어떤 로봇이 열화상 탐지를 할 수 있나?"처럼
    능력 기반으로 로봇을 선택할 수 있도록 지원한다.

    사용 예시:
        library = SkillLibrary()
        library.register(PatrolSkill('argos1'))
        library.register(DetectFireSkill('argos1', robot_type='ugv'))
        library.register(PatrolSkill('drone1', robot_type='drone'))

        # 전체 스킬 MCP 도구 목록
        tools = library.to_mcp_tools()

        # argos1이 할 수 있는 모든 스킬
        argos1_skills = library.get_robot_skills('argos1')

        # 열화상 능력 있는 로봇 목록
        thermal_robots = library.get_capable_robots(['thermal'])
    """

    def __init__(self) -> None:
        # {skill_name: AbstractRobotSkill} — 스킬 이름은 고유해야 함
        self._skills: dict[str, AbstractRobotSkill] = {}

    def register(self, skill: AbstractRobotSkill) -> None:
        """스킬을 등록한다.

        같은 이름의 스킬이 이미 있으면 덮어쓴다 (재등록 허용).
        """
        descriptor = skill.get_descriptor()
        self._skills[descriptor.name] = skill

    def unregister(self, skill_name: str) -> bool:
        """스킬 등록을 해제한다.

        반환값: 존재했던 스킬을 제거하면 True, 없었으면 False.
        """
        if skill_name in self._skills:
            del self._skills[skill_name]
            return True
        return False

    def get_robot_skills(self, robot_id: str) -> list[SkillDescriptor]:
        """특정 로봇이 보유한 모든 스킬 디스크립터 목록."""
        return [
            skill.get_descriptor()
            for skill in self._skills.values()
            if skill.robot_id == robot_id
        ]

    def get_skills_by_type(self, skill_type: str) -> list[SkillDescriptor]:
        """특정 타입(patrol/detect_fire/return_home/rescue)의 스킬 목록."""
        return [
            skill.get_descriptor()
            for skill in self._skills.values()
            if skill.get_descriptor().skill_type == skill_type
        ]

    def get_capable_robots(self, required_capabilities: list[str]) -> list[str]:
        """지정된 하드웨어 능력을 모두 보유한 로봇 ID 목록.

        예: get_capable_robots(['thermal', 'lidar']) →
            thermal과 lidar를 둘 다 가진 로봇만 반환.
        """
        capable: set[str] = set()
        required = set(required_capabilities)
        for skill in self._skills.values():
            desc = skill.get_descriptor()
            if required.issubset(set(desc.required_capabilities)):
                capable.add(desc.robot_id)
        return sorted(capable)

    def to_mcp_tools(self) -> list[dict]:
        """모든 스킬을 MCP Tool 형식 리스트로 변환.

        Claude API의 tools 파라미터에 직접 전달 가능한 형식.
        """
        return [skill.to_mcp_tool() for skill in self._skills.values()]

    def query(self, **filters: Any) -> list[SkillDescriptor]:
        """유연한 스킬 쿼리.

        지원 필터:
            robot_id='argos1'
            skill_type='patrol'
            robot_type='ugv'
            has_capability='thermal'  (단일 능력 보유 여부)

        예:
            library.query(robot_type='drone', skill_type='detect_fire')
        """
        results = []
        for skill in self._skills.values():
            desc = skill.get_descriptor()
            match = True

            if 'robot_id' in filters and desc.robot_id != filters['robot_id']:
                match = False
            if 'skill_type' in filters and desc.skill_type != filters['skill_type']:
                match = False
            if 'robot_type' in filters and desc.robot_type != filters['robot_type']:
                match = False
            if 'has_capability' in filters:
                cap = filters['has_capability']
                if cap not in desc.required_capabilities:
                    match = False

            if match:
                results.append(desc)

        return results

    def __len__(self) -> int:
        return len(self._skills)

    def __repr__(self) -> str:
        return f'SkillLibrary(skills={len(self._skills)})'
