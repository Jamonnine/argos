#!/usr/bin/env python3
"""platform_interface.py — ARGOS Platform 계층 추상 인터페이스

4계층 아키텍처 — Platform 계층 (최하단):
  Orchestrator → Mission Modules → Core Services → **Platform**

설계 원칙:
  "인터페이스에 프로그래밍" — 오케스트레이터는 로봇 타입을 모른다.
  UGV인지 드론인지 보행형인지 모르고, capabilities만 질의한다.
  DARPA SubT CERBERUS/CoSTAR와 동일 추상화 전략.

사용 방법:
  class MyUGV(PlatformInterface):
      def move_to(self, x, y, z=0.0) -> bool: ...
      # 나머지 5개 추상 메서드 구현

  robot: PlatformInterface = MyUGV()
  if robot.get_capabilities().can_drive:
      robot.move_to(5.0, 3.0)

포함 모듈:
  - RobotCapabilities: 로봇 능력 데이터클래스
  - PlatformInterface: 추상 기반 클래스 (ABC)
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class RobotCapabilities:
    """로봇 플랫폼의 하드웨어 능력 명세.

    오케스트레이터가 임무 할당 전 질의하는 능력 집합.
    새 능력 추가 시 이 클래스만 확장하면 기존 코드에 영향 없음.

    Attributes:
        can_fly: 비행 능력 여부 (드론, UAV)
        can_drive: 지상 주행 능력 여부 (UGV, 차량형)
        has_thermal: 열화상 카메라 탑재 여부
        has_lidar: LiDAR 탑재 여부
        max_speed: 최대 이동 속도 (m/s)
        battery_capacity: 배터리 용량 (%, 100.0 = 완충)
        platform_type: 플랫폼 식별자 ("ugv" | "drone" | "legged" | "unknown")
    """
    can_fly: bool = False
    can_drive: bool = False
    has_thermal: bool = False
    has_lidar: bool = False
    max_speed: float = 0.0
    battery_capacity: float = 100.0
    platform_type: str = "unknown"
    # 추후 확장용: has_manipulator, has_hose, payload_kg 등
    extra: dict = field(default_factory=dict)


class PlatformInterface(ABC):
    """ARGOS 플랫폼 통합 추상 인터페이스.

    모든 로봇 플랫폼(UGV, 드론, 보행형 등)이 구현해야 하는 계약.
    오케스트레이터는 이 인터페이스를 통해서만 로봇과 통신한다.

    구현 클래스 필수 사항:
      - 6개 추상 메서드 전부 구현
      - move_to 실패 시 False 반환 (예외 발생 금지 — 오케스트레이터 루프 보호)
      - emergency_stop은 반드시 동기 실행 (async 불허)

    스레드 안전성:
      emergency_stop()은 어느 스레드에서도 호출 가능해야 한다.
      ROS 2 콜백 스레드에서 호출될 수 있으므로 내부 lock 사용 권장.
    """

    # ──────────────────────────────────────────────────────────────────────────
    # 이동 제어
    # ──────────────────────────────────────────────────────────────────────────

    @abstractmethod
    def move_to(self, x: float, y: float, z: float = 0.0) -> bool:
        """지정 좌표로 이동 명령 전송 (비동기 — 즉시 반환).

        좌표계: 맵 프레임 ENU (East-North-Up).
          UGV: z 무시 (평면 이동)
          드론: z = 고도 (m, ENU)

        Args:
            x: 목표 x 좌표 (m, ENU East)
            y: 목표 y 좌표 (m, ENU North)
            z: 목표 z 좌표 (m, ENU Up). UGV는 무시.

        Returns:
            True: 이동 명령이 성공적으로 전송됨 (도달 보장 아님)
            False: 명령 전송 실패 (지오펜스 위반, 연결 오류 등)
        """
        ...

    @abstractmethod
    def return_home(self) -> bool:
        """귀환 명령 (홈 포지션 또는 출발점으로 복귀).

        배터리 부족 또는 임무 종료 시 오케스트레이터가 호출.
        홈 좌표는 구현 클래스 파라미터에서 정의.

        Returns:
            True: 귀환 명령 전송 성공
            False: 귀환 불가 (이미 착륙 중, 연결 오류 등)
        """
        ...

    # ──────────────────────────────────────────────────────────────────────────
    # 상태 조회
    # ──────────────────────────────────────────────────────────────────────────

    @abstractmethod
    def get_pose(self) -> tuple[float, float, float]:
        """현재 위치 조회 (맵 프레임 ENU).

        Returns:
            (x, y, z) 튜플 (m, ENU 좌표계)
            조회 실패 시 (0.0, 0.0, 0.0) 반환 (예외 발생 금지)
        """
        ...

    @abstractmethod
    def get_battery(self) -> float:
        """배터리 잔량 조회.

        Returns:
            배터리 잔량 (%, 0.0 ~ 100.0)
            센서 없는 경우 시뮬레이션 값 반환
        """
        ...

    @abstractmethod
    def get_capabilities(self) -> RobotCapabilities:
        """로봇 능력 명세 반환.

        오케스트레이터가 임무 할당 전 이 메서드를 호출하여
        로봇이 특정 임무를 수행할 수 있는지 판단한다.

        Returns:
            RobotCapabilities 인스턴스 (캐시 가능, 런타임 변경 없음)
        """
        ...

    # ──────────────────────────────────────────────────────────────────────────
    # 안전 제어
    # ──────────────────────────────────────────────────────────────────────────

    @abstractmethod
    def emergency_stop(self) -> None:
        """긴급 정지 — 모든 이동 즉시 중단.

        소방 현장에서 구조대원 접근, 구조물 붕괴 위험 등
        최우선 안전 명령. 반드시 동기 실행, 예외 억제.

        E-STOP 계층:
          Tier-1 (H/W watchdog) — 하드웨어 수준 (이 인터페이스 범위 외)
          Tier-2 (OS signal)     — 프로세스 수준 (이 인터페이스 범위 외)
          Tier-3 (SW E-STOP)     — 이 메서드가 담당

        구현 시 주의:
          - 예외 발생 금지 (오케스트레이터 루프 보호)
          - 가능하면 cmd_vel=0 + Nav2 취소 동시 실행
          - 드론: 호버링(비착륙) 또는 안전 착륙 중 선택
        """
        ...

    # ──────────────────────────────────────────────────────────────────────────
    # 편의 메서드 (기본 구현 제공 — 필요 시 오버라이드)
    # ──────────────────────────────────────────────────────────────────────────

    def is_alive(self) -> bool:
        """플랫폼 정상 동작 여부 확인 (헬스 체크).

        기본 구현: get_battery가 0 초과면 살아있다고 간주.
        구현 클래스에서 하트비트 타임아웃 등으로 오버라이드 가능.
        """
        try:
            return self.get_battery() > 0.0
        except Exception:
            return False

    def get_platform_type(self) -> str:
        """플랫폼 타입 문자열 반환 ("ugv" | "drone" | "legged" | "unknown").

        get_capabilities().platform_type의 단축 접근자.
        """
        return self.get_capabilities().platform_type
