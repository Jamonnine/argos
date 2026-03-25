"""ARGOS MCP 로봇 서버 — Claude/MCP 클라이언트에서 자연어로 로봇 제어

DimOS 벤치마킹: MCP 서버 패턴.
ARGOS: SkillLibrary의 스킬을 MCP Tool로 노출.

구조:
    Claude/MCP Client → MCP Protocol → ARGOS MCP Server → SkillLibrary → ROS2 Action/Service

MCP SDK가 없으면 JSON-RPC stdio 모드로 graceful fallback.
ROS2가 없어도 import 에러 없이 동작 (스켈레톤/테스트 목적).

사용 예시:
    # MCP stdio 서버로 실행
    python -m argos_bringup.mcp_robot_server

    # 코드에서 직접 사용
    library = SkillLibrary()
    library.register(PatrolSkill('argos1'))
    server = ArgosMCPServer(library)
    tools = server.list_tools()
    result = server.call_tool('patrol_argos1', {'waypoints': '[[0,0,0],[1,0,0]]'})
"""

from __future__ import annotations

import json
import sys
import logging
from typing import Any

from argos_bringup.skill_library import (
    SkillLibrary,
    PatrolSkill,
    DetectFireSkill,
    ReturnHomeSkill,
    RescueSkill,
)

# MCP SDK 선택적 임포트 — 없어도 graceful fallback
try:
    from mcp.server import Server as MCPServer
    from mcp.server.stdio import stdio_server
    MCP_AVAILABLE = True
except ImportError:
    MCP_AVAILABLE = False

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# MCP 응답 헬퍼 — SDK 없을 때 직접 JSON-RPC 응답 생성
# ---------------------------------------------------------------------------

def _make_json_rpc_response(request_id: Any, result: Any) -> dict:
    """JSON-RPC 2.0 성공 응답 딕셔너리 생성."""
    return {
        'jsonrpc': '2.0',
        'id': request_id,
        'result': result,
    }


def _make_json_rpc_error(request_id: Any, code: int, message: str) -> dict:
    """JSON-RPC 2.0 에러 응답 딕셔너리 생성."""
    return {
        'jsonrpc': '2.0',
        'id': request_id,
        'error': {'code': code, 'message': message},
    }


# ---------------------------------------------------------------------------
# ArgosMCPServer
# ---------------------------------------------------------------------------

class ArgosMCPServer:
    """ARGOS 로봇 제어 MCP 서버.

    SkillLibrary의 스킬을 MCP Tool로 노출하여
    Claude 또는 MCP 클라이언트가 자연어 + 도구 호출로
    로봇 스킬을 직접 발동할 수 있게 한다.

    실제 ROS2 액션/서비스 실행은 _dispatch_ros2() 에서 수행.
    현재는 스켈레톤 — 실제 rclpy 호출로 교체 필요.
    """

    def __init__(self, skill_library: SkillLibrary) -> None:
        self.library = skill_library
        # ROS2 노드 인스턴스 (실제 실행 시 외부에서 주입)
        self._ros2_node: Any = None
        logger.info(
            'ArgosMCPServer 초기화 완료 (MCP SDK=%s, 등록 스킬=%d개)',
            MCP_AVAILABLE,
            len(self.library),
        )

    # ------------------------------------------------------------------
    # MCP 핸들러
    # ------------------------------------------------------------------

    def list_tools(self) -> list[dict]:
        """MCP list_tools 핸들러 — 등록된 모든 스킬을 MCP Tool 형식으로 반환.

        반환 형식은 Anthropic MCP Tool Schema(name, description, inputSchema)를 따른다.
        """
        tools = self.library.to_mcp_tools()
        logger.debug('list_tools 요청: %d개 도구 반환', len(tools))
        return tools

    def call_tool(self, name: str, args: dict) -> dict:
        """MCP call_tool 핸들러 — 스킬 이름과 인수로 로봇 스킬을 발동.

        인수:
            name: 스킬 이름 (e.g. 'patrol_argos1')
            args: 스킬 파라미터 딕셔너리

        반환:
            {'content': [{'type': 'text', 'text': <결과 메시지>}]}
        """
        # 스킬 존재 여부 확인
        skill = self.library._skills.get(name)
        if skill is None:
            # 등록된 스킬 목록 힌트 제공
            available = list(self.library._skills.keys())
            msg = f"스킬 '{name}'을 찾을 수 없습니다. 등록된 스킬: {available}"
            logger.warning(msg)
            return {'content': [{'type': 'text', 'text': msg}], 'isError': True}

        # 파라미터 유효성 검사
        if not skill.validate_params(**args):
            msg = f"스킬 '{name}' 파라미터 유효성 검사 실패: {args}"
            logger.warning(msg)
            return {'content': [{'type': 'text', 'text': msg}], 'isError': True}

        # ROS2 디스패치 (실제 발동)
        try:
            result_msg = self._dispatch_ros2(skill, args)
        except Exception as exc:
            msg = f"스킬 '{name}' 실행 중 오류: {exc}"
            logger.exception(msg)
            return {'content': [{'type': 'text', 'text': msg}], 'isError': True}

        logger.info("스킬 실행 완료: name=%s args=%s", name, args)
        return {'content': [{'type': 'text', 'text': result_msg}]}

    # ------------------------------------------------------------------
    # ROS2 디스패치 — 실제 액션/서비스 호출 스텁
    # ------------------------------------------------------------------

    def _dispatch_ros2(self, skill: Any, args: dict) -> str:
        """ROS2 액션 또는 서비스를 호출한다.

        현재 구현: 스켈레톤 (로그만 출력).
        실제 전환 시: rclpy.action.ActionClient 또는 rclpy.Client 사용.

        전환 예시:
            if desc.ros2_action:
                client = ActionClient(self._ros2_node, NavigateToPose, desc.ros2_action)
                goal = NavigateToPose.Goal()
                future = client.send_goal_async(goal)
                rclpy.spin_until_future_complete(self._ros2_node, future)
        """
        desc = skill.get_descriptor()
        endpoint = desc.ros2_action or desc.ros2_service or '(미설정)'

        # G-5: ROS2 노드가 주입된 경우 실제 Action/Service 호출
        if self._ros2_node is not None:
            return self._dispatch_ros2_real(desc, args)

        # fallback: Mock 응답 (테스트/스켈레톤 모드)
        msg = (
            f"[MOCK] 스킬 실행: robot={desc.robot_id}, "
            f"skill={desc.name}, endpoint={endpoint}, args={args}"
        )
        logger.info(msg)
        return msg

    def _dispatch_ros2_real(self, desc, args: dict) -> str:
        """실제 ROS2 Action/Service 호출 (ros2_node 주입 시).

        G-5 TODO 해소: line 170 — Mock → 실 rclpy 호출 전환.
        """
        try:
            import rclpy
            from rclpy.action import ActionClient
        except ImportError:
            return f"[ERROR] rclpy 미설치 — Mock 모드로 대체"

        node = self._ros2_node

        if desc.ros2_action:
            # Action 기반 스킬 (patrol, navigate_to_pose 등)
            try:
                from nav2_msgs.action import NavigateToPose
                from argos_interfaces.action import PatrolArea
            except ImportError:
                return f"[ERROR] Action 메시지 import 실패"

            action_name = desc.ros2_action
            logger.info(f'[ROS2] Action 호출: {action_name}, args={args}')

            if 'patrol' in action_name.lower():
                client = ActionClient(node, PatrolArea, action_name)
            else:
                client = ActionClient(node, NavigateToPose, action_name)

            if not client.wait_for_server(timeout_sec=5.0):
                return f"[TIMEOUT] Action 서버 '{action_name}' 5초 내 미응답"

            if 'patrol' in action_name.lower():
                goal = PatrolArea.Goal()
            else:
                goal = NavigateToPose.Goal()
                goal.pose.header.frame_id = 'map'
                goal.pose.pose.position.x = float(args.get('x', 0.0))
                goal.pose.pose.position.y = float(args.get('y', 0.0))
                goal.pose.pose.orientation.w = 1.0

            future = client.send_goal_async(goal)
            rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

            if future.result() is not None:
                return f"[OK] Action '{action_name}' goal accepted"
            return f"[FAIL] Action '{action_name}' goal rejected"

        elif desc.ros2_service:
            # Service 기반 스킬 (detect_fire 등)
            from std_srvs.srv import Trigger
            service_name = desc.ros2_service
            logger.info(f'[ROS2] Service 호출: {service_name}')

            client = node.create_client(Trigger, service_name)
            if not client.wait_for_service(timeout_sec=5.0):
                return f"[TIMEOUT] Service '{service_name}' 5초 내 미응답"

            request = Trigger.Request()
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

            if future.result() is not None:
                resp = future.result()
                return f"[OK] Service '{service_name}': {resp.message}"
            return f"[FAIL] Service '{service_name}' 응답 없음"

        return f"[SKIP] 스킬 '{desc.name}': ros2_action/service 미설정"

    # ------------------------------------------------------------------
    # 실행 모드
    # ------------------------------------------------------------------

    def run_stdio(self) -> None:
        """stdio 모드 MCP 서버 실행.

        MCP SDK가 있으면 공식 SDK 사용,
        없으면 직접 JSON-RPC 2.0 stdio 루프로 폴백.
        """
        if MCP_AVAILABLE:
            self._run_with_sdk()
        else:
            logger.warning(
                'MCP SDK 없음 (pip install mcp). 직접 JSON-RPC stdio 폴백 모드 실행.'
            )
            self._run_stdio_fallback()

    def _run_with_sdk(self) -> None:
        """공식 MCP SDK를 사용한 서버 실행 (SDK 설치된 환경).

        G-5 TODO 해소: line 196 — MCP SDK 핸들러 등록 완성.
        """
        import asyncio

        server = MCPServer('argos-mcp-server')
        mcp_server = self  # closure용

        @server.list_tools()
        async def handle_list_tools():
            """등록된 모든 스킬을 MCP Tool 형식으로 반환."""
            return mcp_server.list_tools()

        @server.call_tool()
        async def handle_call_tool(name: str, arguments: dict):
            """스킬 이름과 인수로 로봇 스킬 발동."""
            return mcp_server.call_tool(name, arguments or {})

        logger.info('MCP SDK 서버 기동: %d개 스킬 등록', len(self.library))

        async def _serve():
            async with stdio_server() as (read, write):
                await server.run(read, write, server.create_initialization_options())

        asyncio.run(_serve())

    def _run_stdio_fallback(self) -> None:
        """MCP SDK 없이 JSON-RPC 2.0 stdio 루프."""
        logger.info('JSON-RPC stdio 폴백 서버 시작. stdin 대기 중...')
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            try:
                request = json.loads(line)
            except json.JSONDecodeError as exc:
                response = _make_json_rpc_error(None, -32700, f'JSON 파싱 오류: {exc}')
                print(json.dumps(response, ensure_ascii=False), flush=True)
                continue

            request_id = request.get('id')
            method = request.get('method', '')

            if method == 'tools/list':
                tools = self.list_tools()
                response = _make_json_rpc_response(request_id, {'tools': tools})

            elif method == 'tools/call':
                params = request.get('params', {})
                tool_name = params.get('name', '')
                tool_args = params.get('arguments', {})
                result = self.call_tool(tool_name, tool_args)
                response = _make_json_rpc_response(request_id, result)

            elif method == 'initialize':
                # MCP 핸드셰이크 응답
                response = _make_json_rpc_response(request_id, {
                    'protocolVersion': '2024-11-05',
                    'capabilities': {'tools': {}},
                    'serverInfo': {'name': 'argos-mcp-server', 'version': '0.1.0'},
                })

            else:
                response = _make_json_rpc_error(
                    request_id, -32601, f"알 수 없는 메서드: '{method}'"
                )

            print(json.dumps(response, ensure_ascii=False), flush=True)


# ---------------------------------------------------------------------------
# 기본 SkillLibrary 팩토리 — 표준 4개 로봇 스킬 등록
# ---------------------------------------------------------------------------

def create_default_library(robot_id: str = 'argos1', robot_type: str = 'ugv') -> SkillLibrary:
    """표준 ARGOS 로봇 스킬 4종을 등록한 SkillLibrary를 반환한다."""
    library = SkillLibrary()
    library.register(PatrolSkill(robot_id=robot_id, robot_type=robot_type))
    library.register(DetectFireSkill(robot_id=robot_id, robot_type=robot_type))
    library.register(ReturnHomeSkill(robot_id=robot_id, robot_type=robot_type))
    library.register(RescueSkill(robot_id=robot_id, robot_type=robot_type))
    return library


# ---------------------------------------------------------------------------
# 엔트리포인트
# ---------------------------------------------------------------------------

def main() -> None:
    """MCP stdio 서버 엔트리포인트 (setup.py console_scripts 등록용)."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
        stream=sys.stderr,  # MCP 서버는 stderr에 로그, stdout은 JSON-RPC 전용
    )
    library = create_default_library()
    server = ArgosMCPServer(library)
    server.run_stdio()


if __name__ == '__main__':
    main()
