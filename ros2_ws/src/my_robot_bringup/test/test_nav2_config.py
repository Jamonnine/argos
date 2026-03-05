"""
Nav2 설정 파일(nav2_params.yaml) 구조 검증 테스트.
YAML 파싱, 필수 키 존재, 플러그인 이름, ARGOS 물리 파라미터 정합성 검증.
rclpy 없이 순수 Python 단위 테스트.
"""

import os
import pytest
import yaml


# nav2_params.yaml 경로 (프로젝트 상대 경로)
YAML_PATH = os.path.join(
    os.path.dirname(__file__), '..', '..', 'argos_description',
    'config', 'nav2_params.yaml')


@pytest.fixture(scope='module')
def nav2_config():
    """YAML 파일 로드 (모듈 레벨 캐싱)."""
    with open(YAML_PATH, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


# ════════════════════════════════════════
# YAML 구조 & 파싱
# ════════════════════════════════════════

class TestYamlStructure:

    def test_file_exists(self):
        """설정 파일 존재 확인."""
        assert os.path.isfile(YAML_PATH), f'{YAML_PATH} not found'

    def test_yaml_parseable(self, nav2_config):
        """YAML 파싱 성공."""
        assert isinstance(nav2_config, dict)

    def test_top_level_sections(self, nav2_config):
        """필수 최상위 섹션 존재."""
        required = [
            'bt_navigator', 'controller_server', 'planner_server',
            'local_costmap', 'global_costmap', 'behavior_server',
            'velocity_smoother', 'collision_monitor', 'slam_toolbox',
        ]
        for section in required:
            assert section in nav2_config, f'Missing section: {section}'


# ════════════════════════════════════════
# SmacPlanner2D 검증
# ════════════════════════════════════════

class TestSmacPlanner:

    def test_planner_plugin(self, nav2_config):
        """SmacPlanner2D 플러그인 설정 확인."""
        ps = nav2_config['planner_server']['ros__parameters']
        assert 'GridBased' in ps['planner_plugins']
        plugin = ps['GridBased']['plugin']
        assert plugin == 'nav2_smac_planner::SmacPlanner2D'

    def test_planner_tolerance(self, nav2_config):
        """경로 탐색 tolerance 합리적 범위."""
        ps = nav2_config['planner_server']['ros__parameters']
        tol = ps['GridBased']['tolerance']
        assert 0.1 <= tol <= 1.0, f'tolerance={tol} out of range'

    def test_max_planning_time(self, nav2_config):
        """최대 계획 시간 설정 존재."""
        ps = nav2_config['planner_server']['ros__parameters']
        t = ps['GridBased']['max_planning_time']
        assert 1.0 <= t <= 30.0

    def test_smoother_config(self, nav2_config):
        """경로 스무더 설정 존재."""
        ps = nav2_config['planner_server']['ros__parameters']
        smoother = ps['GridBased']['smoother']
        assert smoother['max_iterations'] > 0
        assert 0.0 < smoother['w_smooth'] < 1.0
        assert 0.0 < smoother['w_data'] < 1.0


# ════════════════════════════════════════
# MPPI Controller 검증
# ════════════════════════════════════════

class TestMPPIController:

    def test_controller_plugin(self, nav2_config):
        """MPPI Controller 플러그인 설정 확인."""
        cs = nav2_config['controller_server']['ros__parameters']
        plugin = cs['FollowPath']['plugin']
        assert plugin == 'nav2_mppi_controller::MPPIController'

    def test_motion_model(self, nav2_config):
        """ARGOS UGV 운동 모델 = DiffDrive."""
        cs = nav2_config['controller_server']['ros__parameters']
        assert cs['FollowPath']['motion_model'] == 'DiffDrive'

    def test_velocity_limits_match_argos(self, nav2_config):
        """MPPI 속도 제한이 ARGOS 사양과 일치."""
        fp = nav2_config['controller_server']['ros__parameters']['FollowPath']
        assert fp['vx_max'] == 0.5   # ARGOS max forward speed
        assert fp['vy_max'] == 0.0   # 차동구동: 횡방향 없음
        assert fp['wz_max'] == 1.5   # ARGOS max angular speed

    def test_batch_size_reasonable(self, nav2_config):
        """궤적 샘플 수 합리적 범위 (CPU 기반)."""
        fp = nav2_config['controller_server']['ros__parameters']['FollowPath']
        bs = fp['batch_size']
        assert 500 <= bs <= 10000, f'batch_size={bs}'

    def test_prediction_horizon(self, nav2_config):
        """예측 구간 = time_steps × model_dt (합리적 범위)."""
        fp = nav2_config['controller_server']['ros__parameters']['FollowPath']
        horizon = fp['time_steps'] * fp['model_dt']
        assert 1.0 <= horizon <= 5.0, f'horizon={horizon}s'

    def test_critics_exist(self, nav2_config):
        """필수 MPPI critics 설정 존재."""
        fp = nav2_config['controller_server']['ros__parameters']['FollowPath']
        critics = fp['critics']
        required_critics = ['ConstraintCritic', 'CostCritic', 'GoalCritic']
        for c in required_critics:
            assert c in critics, f'Missing critic: {c}'

    def test_controller_frequency(self, nav2_config):
        """MPPI는 고빈도 제어가 효과적 (≥15Hz)."""
        cs = nav2_config['controller_server']['ros__parameters']
        freq = cs['controller_frequency']
        assert freq >= 15.0, f'controller_frequency={freq} too low for MPPI'

    def test_diff_drive_no_lateral(self, nav2_config):
        """차동구동: 횡방향 속도 노이즈 = 0."""
        fp = nav2_config['controller_server']['ros__parameters']['FollowPath']
        assert fp['vy_std'] == 0.0


# ════════════════════════════════════════
# Costmap 검증
# ════════════════════════════════════════

class TestCostmap:

    def test_local_costmap_frame(self, nav2_config):
        """local costmap base frame = base_footprint."""
        lc = nav2_config['local_costmap']['local_costmap']['ros__parameters']
        assert lc['robot_base_frame'] == 'base_footprint'

    def test_global_costmap_frame(self, nav2_config):
        """global costmap base frame = base_footprint."""
        gc = nav2_config['global_costmap']['global_costmap']['ros__parameters']
        assert gc['robot_base_frame'] == 'base_footprint'

    def test_footprint_argos_size(self, nav2_config):
        """footprint 폴리곤이 ARGOS 크기(0.6×0.4m + 여유)와 일치."""
        lc = nav2_config['local_costmap']['local_costmap']['ros__parameters']
        fp_str = lc['footprint']
        assert '0.35' in fp_str  # ±0.35m = 0.7m > 0.6m ✓
        assert '0.25' in fp_str  # ±0.25m = 0.5m > 0.4m ✓


# ════════════════════════════════════════
# Velocity Smoother 정합성
# ════════════════════════════════════════

class TestVelocitySmoother:

    def test_max_velocity_matches_mppi(self, nav2_config):
        """velocity_smoother의 max_velocity가 MPPI 제한과 일치."""
        vs = nav2_config['velocity_smoother']['ros__parameters']
        fp = nav2_config['controller_server']['ros__parameters']['FollowPath']
        assert vs['max_velocity'][0] >= fp['vx_max']   # forward
        assert vs['max_velocity'][2] >= fp['wz_max']   # angular

    def test_smoothing_frequency_matches_controller(self, nav2_config):
        """smoothing_frequency ≥ controller_frequency."""
        vs = nav2_config['velocity_smoother']['ros__parameters']
        cs = nav2_config['controller_server']['ros__parameters']
        assert vs['smoothing_frequency'] >= cs['controller_frequency']


# ════════════════════════════════════════
# SLAM Toolbox 검증
# ════════════════════════════════════════

class TestSLAMToolbox:

    def test_base_frame(self, nav2_config):
        """SLAM base_frame = base_footprint."""
        st = nav2_config['slam_toolbox']['ros__parameters']
        assert st['base_frame'] == 'base_footprint'

    def test_laser_range(self, nav2_config):
        """LiDAR max range 설정."""
        st = nav2_config['slam_toolbox']['ros__parameters']
        assert 5.0 <= st['max_laser_range'] <= 30.0


# ════════════════════════════════════════
# Collision Monitor 검증
# ════════════════════════════════════════

class TestCollisionMonitor:

    def test_base_frame(self, nav2_config):
        """collision_monitor base_frame = base_footprint."""
        cm = nav2_config['collision_monitor']['ros__parameters']
        assert cm['base_frame_id'] == 'base_footprint'

    def test_polygon_stop_exists(self, nav2_config):
        """PolygonStop 긴급 정지 영역 설정."""
        cm = nav2_config['collision_monitor']['ros__parameters']
        assert 'PolygonStop' in cm['polygons']
        assert cm['PolygonStop']['action_type'] == 'stop'

    def test_polygon_slow_exists(self, nav2_config):
        """PolygonSlow 감속 영역 설정."""
        cm = nav2_config['collision_monitor']['ros__parameters']
        assert 'PolygonSlow' in cm['polygons']
        assert cm['PolygonSlow']['action_type'] == 'slowdown'
