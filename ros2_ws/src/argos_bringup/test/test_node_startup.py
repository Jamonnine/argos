"""
노드 기동 통합 테스트 — ROS2 없이 임포트 + 초기화 검증.
QA 전문가 권고: 미테스트 노드 해소.

rclpy.init() 없이 모듈 임포트만 검증 (CI에서 ROS2 없이도 실행 가능).
"""
import pytest
import importlib


# 모든 ARGOS 노드 모듈 목록
ARGOS_MODULES = [
    'argos_bringup.orchestrator_node',
    'argos_bringup.frontier_explorer_node',
    'argos_bringup.hotspot_detector_node',
    'argos_bringup.robot_status_node',
    'argos_bringup.drone_controller_node',
    'argos_bringup.scenario_runner_node',
    'argos_bringup.perception_bridge_node',
    'argos_bringup.gas_sensor_node',
    'argos_bringup.victim_detector_node',
    'argos_bringup.structural_monitor_node',
    'argos_bringup.audio_detector_node',
    'argos_bringup.smoke_effect_node',
    'argos_bringup.severity_utils',
    'argos_bringup.validation_utils',
    'argos_bringup.base_detector',
    'argos_bringup.kalman_tracker',
]


class TestModuleImports:
    """모든 ARGOS 모듈이 임포트 가능한지 검증."""

    @pytest.mark.parametrize('module_name', ARGOS_MODULES)
    def test_module_imports(self, module_name):
        """각 모듈이 ImportError 없이 로드되는지."""
        try:
            mod = importlib.import_module(module_name)
            assert mod is not None
        except ImportError as e:
            # rclpy 미설치 환경에서는 rclpy 관련 임포트 실패 허용
            if 'rclpy' in str(e) or 'argos_interfaces' in str(e):
                pytest.skip(f'ROS2 dependency not available: {e}')
            else:
                raise


class TestUtilityModules:
    """유틸리티 모듈의 핵심 함수 존재 검증."""

    def test_severity_utils_functions(self):
        from argos_bringup.severity_utils import (
            classify_by_thresholds, worst_severity, should_evacuate,
            SEVERITY_LEVELS,
        )
        assert len(SEVERITY_LEVELS) == 4
        assert classify_by_thresholds(100, [50, 200, 800]) == 'caution'
        assert worst_severity('safe', 'danger') == 'danger'

    def test_validation_utils_functions(self):
        from argos_bringup.validation_utils import (
            validate_robot_id, validate_severity, clamp_sensor,
            validate_timestamp, is_finite, sanitize_float,
        )
        assert validate_robot_id('argos1') is True
        assert validate_robot_id('<script>') is False
        assert validate_severity('critical') is True
        assert clamp_sensor(99999, 'co_ppm') == 5000.0
        assert is_finite(float('nan')) is False

    def test_kalman_tracker_functions(self):
        from argos_bringup.kalman_tracker import SimpleKalmanTracker
        tracker = SimpleKalmanTracker()
        tracks = tracker.update([(1.0, 2.0, 'fire', 0.9)])
        assert len(tracks) == 1
        assert tracks[0].class_name == 'fire'

    def test_base_detector_interface(self):
        from argos_bringup.base_detector import BaseDetector, Detection
        # 추상 클래스 인스턴스화 불가 확인
        with pytest.raises(TypeError):
            BaseDetector()
        # Detection 데이터클래스
        det = Detection(class_name='fire', class_id=0, confidence=0.9, bbox=(0, 0, 100, 100))
        assert det.class_name == 'fire'
        assert det.is_hazard is False
