"""severity_utils.py — ARGOS 심각도 판정 공통 모듈

가스/열화상/구조물/음향 등 다양한 센서의 심각도 판정을
중앙화하여 DRY 원칙을 준수하고 기준값 변경 시 1곳만 수정.

소방 현장 기준 (NIOSH/OSHA/ICS):
  safe     → 정상, 작업 계속
  caution  → 주의, 모니터링 강화
  danger   → 위험, 진입 제한 또는 감시 체계 전환
  critical → 치명, 즉시 철수 또는 긴급 대응
"""

from typing import Optional

# 심각도 등급 (순서 = 우선순위)
SEVERITY_LEVELS = ('safe', 'caution', 'danger', 'critical')


def classify_by_thresholds(
    value: float,
    thresholds: list,
    labels: Optional[tuple] = None,
    reverse: bool = False,
) -> str:
    """값을 임계값 목록과 비교하여 심각도 등급 반환.

    Args:
        value: 판정 대상 값
        thresholds: [caution_threshold, danger_threshold, critical_threshold]
        labels: 등급 이름 (기본: SEVERITY_LEVELS)
        reverse: True면 역방향 비교 (O2처럼 낮을수록 위험)

    Returns:
        심각도 등급 문자열
    """
    if labels is None:
        labels = SEVERITY_LEVELS

    if len(thresholds) != 3:
        return labels[0]  # safe

    if reverse:
        # O2: 낮을수록 위험 (19.5 > 18 > 16 > 14)
        if value <= thresholds[2]:
            return labels[3]  # critical
        elif value <= thresholds[1]:
            return labels[2]  # danger
        elif value <= thresholds[0]:
            return labels[1]  # caution
        else:
            return labels[0]  # safe
    else:
        # CO/LEL/HCN: 높을수록 위험 (50 < 200 < 800)
        if value >= thresholds[2]:
            return labels[3]  # critical
        elif value >= thresholds[1]:
            return labels[2]  # danger
        elif value >= thresholds[0]:
            return labels[1]  # caution
        else:
            return labels[0]  # safe


def worst_severity(*levels: str) -> str:
    """여러 심각도 등급 중 가장 심각한 것을 반환."""
    max_idx = 0
    for level in levels:
        try:
            idx = SEVERITY_LEVELS.index(level)
            if idx > max_idx:
                max_idx = idx
        except ValueError:
            continue
    return SEVERITY_LEVELS[max_idx]


def should_evacuate(
    co_ppm: float = 0.0,
    o2_percent: float = 20.9,
    lel_percent: float = 0.0,
    overall_level: str = 'safe',
) -> bool:
    """즉시 철수 여부 판정 (소방 현장 표준).

    철수 조건 (OR):
    - 전체 등급 critical
    - CO >= 800ppm (NIOSH IDLH)
    - O2 <= 16% (의식 상실 위험)
    - LEL >= 25% (폭발 위험)
    """
    return (
        overall_level == 'critical' or
        co_ppm >= 800.0 or
        o2_percent <= 16.0 or
        lel_percent >= 25.0
    )
