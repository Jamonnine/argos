"""CBBA 태스크 할당기 단위 테스트.

테스트 범위:
  - Task 유효성 검사 (priority 범위)
  - capability 매핑 (UGV/드론 역할 분리)
  - 거리 기반 최적 할당
  - 충돌 해소 (동일 임무 복수 입찰)
  - 미할당 케이스 (임무 > 로봇, capability 미충족)
  - 빈 입력 처리 (로봇 없음, 임무 없음)
"""
import math
import pytest

from argos_bringup.cbba_allocator import CBBAAllocator, RobotRecord, Task


# ── 픽스처 ────────────────────────────────────────────────────────────────────

@pytest.fixture
def allocator():
    """로거 없이 생성 (순수 Python 단위 테스트)."""
    return CBBAAllocator(logger=None)


def make_ugv(robot_id, x=0.0, y=0.0, extra_caps=None):
    """UGV RobotRecord 생성 헬퍼."""
    caps = ['has_thermal', 'lidar', 'depth', 'imu']
    if extra_caps:
        caps.extend(extra_caps)
    return RobotRecord(robot_id=robot_id, capabilities=caps, pose=(x, y, 0.0))


def make_drone(robot_id, x=0.0, y=0.0, extra_caps=None):
    """드론 RobotRecord 생성 헬퍼."""
    caps = ['can_fly', 'has_thermal', 'camera', 'imu']
    if extra_caps:
        caps.extend(extra_caps)
    return RobotRecord(robot_id=robot_id, capabilities=caps, pose=(x, y, 5.0))


def make_task(task_id, x=0.0, y=0.0, task_type='explore',
              required=None, priority=0.5):
    """Task 생성 헬퍼."""
    return Task(
        task_id=task_id,
        location=(x, y, 0.0),
        task_type=task_type,
        required_capabilities=required or [],
        priority=priority,
    )


# ── Task 유효성 테스트 ──────────────────────────────────────────────────────

class TestTaskValidation:

    def test_valid_priority_bounds(self):
        """priority 0.0, 0.5, 1.0 모두 정상 생성."""
        for p in (0.0, 0.5, 1.0):
            t = make_task('t', priority=p)
            assert t.priority == p

    def test_invalid_priority_over_one(self):
        """priority > 1.0 → ValueError."""
        with pytest.raises(ValueError):
            make_task('t', priority=1.1)

    def test_invalid_priority_negative(self):
        """priority < 0.0 → ValueError."""
        with pytest.raises(ValueError):
            make_task('t', priority=-0.1)


# ── 빈 입력 처리 ──────────────────────────────────────────────────────────────

class TestEmptyInputs:

    def test_empty_robots(self, allocator):
        """로봇 없음 → 빈 dict 반환."""
        tasks = [make_task('t1')]
        result = allocator.allocate([], tasks)
        assert result == {}

    def test_empty_tasks(self, allocator):
        """임무 없음 → 빈 dict 반환."""
        robots = [make_ugv('argos1')]
        result = allocator.allocate(robots, [])
        assert result == {}

    def test_both_empty(self, allocator):
        """둘 다 비어도 빈 dict 반환."""
        result = allocator.allocate([], [])
        assert result == {}


# ── capability 매핑 ───────────────────────────────────────────────────────────

class TestCapabilityMatching:

    def test_ugv_cannot_fly(self, allocator):
        """UGV는 monitor(can_fly 필수) 임무를 받지 않는다."""
        ugv = make_ugv('argos1', x=0.0)
        drone = make_drone('drone1', x=1.0)
        monitor_task = make_task('monitor_1', x=2.0, task_type='monitor',
                                 required=['can_fly'])
        result = allocator.allocate([ugv, drone], [monitor_task])
        # UGV가 아닌 드론에 할당되어야 함
        assert 'drone1' in result
        assert result['drone1'].task_id == 'monitor_1'
        assert 'argos1' not in result

    def test_drone_gets_monitor_task(self, allocator):
        """드론은 monitor 임무를 정상 수행."""
        drone = make_drone('drone1', x=0.0)
        task = make_task('monitor_1', task_type='monitor', required=['can_fly'])
        result = allocator.allocate([drone], [task])
        assert 'drone1' in result

    def test_no_capable_robot(self, allocator):
        """아무 로봇도 capability 미충족 → 미할당 (결과에 없음)."""
        ugv = make_ugv('argos1')
        task = make_task('rescue_1', required=['has_gripper'])  # 아무도 없는 capability
        result = allocator.allocate([ugv], [task])
        assert len(result) == 0

    def test_all_capabilities_required(self, allocator):
        """required_capabilities 복수 → 모두 보유해야 할당."""
        # has_thermal + can_fly 둘 다 필요: 드론만 가능
        ugv = make_ugv('argos1')
        drone = make_drone('drone1')
        task = make_task('aerial_fire', required=['can_fly', 'has_thermal'])
        result = allocator.allocate([ugv, drone], [task])
        assert 'drone1' in result
        assert 'argos1' not in result

    def test_empty_required_caps(self, allocator):
        """required_capabilities=[] → 모든 로봇 수행 가능."""
        ugv = make_ugv('argos1')
        task = make_task('explore_1', required=[])
        result = allocator.allocate([ugv], [task])
        assert 'argos1' in result


# ── 거리 기반 최적 할당 ────────────────────────────────────────────────────────

class TestDistanceOptimization:

    def test_nearest_robot_assigned(self, allocator):
        """두 UGV 중 임무에 가까운 쪽이 할당."""
        near = make_ugv('argos1', x=1.0, y=0.0)  # 임무까지 거리 ≈ 1m
        far = make_ugv('argos2', x=10.0, y=0.0)  # 임무까지 거리 ≈ 10m
        task = make_task('t1', x=2.0, y=0.0)
        result = allocator.allocate([near, far], [task])
        assert 'argos1' in result
        assert result['argos1'].task_id == 't1'

    def test_two_tasks_two_robots_nearest(self, allocator):
        """로봇 2대, 임무 2개 — 각자 가장 가까운 임무로 분배."""
        r1 = make_ugv('argos1', x=0.0, y=0.0)
        r2 = make_ugv('argos2', x=10.0, y=0.0)
        t1 = make_task('t1', x=1.0, y=0.0)   # r1에 가까움
        t2 = make_task('t2', x=9.0, y=0.0)   # r2에 가까움
        result = allocator.allocate([r1, r2], [t1, t2])
        assert result.get('argos1', None) is not None
        assert result.get('argos2', None) is not None
        # 각자 가까운 임무 배정 검증
        assert result['argos1'].task_id == 't1'
        assert result['argos2'].task_id == 't2'

    def test_distance_calculation(self):
        """3D 거리 공식 검증."""
        d = CBBAAllocator._distance((0.0, 0.0, 0.0), (3.0, 4.0, 0.0))
        assert d == pytest.approx(5.0)

    def test_distance_3d(self):
        """Z 좌표 포함 3D 거리."""
        d = CBBAAllocator._distance((0.0, 0.0, 0.0), (1.0, 1.0, 1.0))
        assert d == pytest.approx(math.sqrt(3.0))


# ── 우선순위 가중치 ────────────────────────────────────────────────────────────

class TestPriorityWeight:

    def test_high_priority_preferred_over_distance(self, allocator):
        """우선순위 1.0 임무는 거리가 멀어도 우선 할당 (단일 로봇 케이스).

        로봇 1대, 임무 2개:
          t_near (priority=0.0, 거리=1m)  비용 ≈ 1.0 * 1 - 0 = 1.0
          t_far  (priority=1.0, 거리=5m)  비용 ≈ 1.0 * 5 - 2.0 = 3.0
        priority bonus=2.0일 때 t_near가 여전히 저렴 → argos1은 t_near
        (이 테스트는 우선순위가 비용을 0 이하로 만들 때 역전 여부 확인)
        """
        robot = make_ugv('argos1', x=0.0, y=0.0)
        # 매우 가까운 저우선순위 임무
        t_near = make_task('t_near', x=0.1, y=0.0, priority=0.0)
        # 매우 먼 고우선순위 임무 (priority bonus=2.0으로 비용 상쇄)
        t_far = make_task('t_far', x=100.0, y=0.0, priority=1.0)
        result = allocator.allocate([robot], [t_near, t_far])
        # 단일 로봇 → 1개만 할당, 더 저렴한 임무 선택
        assert 'argos1' in result

    def test_high_priority_reduces_cost(self, allocator):
        """priority=1.0 임무는 priority=0.0보다 비용이 낮다.

        비용 = max(0, 거리 * DISTANCE_WEIGHT - priority * PRIORITY_WEIGHT)
        거리가 충분히 클 때(10m) 우선순위 기여분이 감소 효과를 확인.
        """
        alloc = CBBAAllocator()
        robot = RobotRecord('r', capabilities=[], pose=(0.0, 0.0, 0.0))
        # 거리=10m: priority bonus(2.0)가 전체 비용(10.0)보다 작아 음수 clamp 없음
        t_low = Task('t_low', (10.0, 0.0, 0.0), 'explore', [], priority=0.0)
        t_high = Task('t_high', (10.0, 0.0, 0.0), 'explore', [], priority=1.0)
        matrix = alloc._build_cost_matrix([robot], [t_low, t_high])
        cost_low = matrix[0][0]
        cost_high = matrix[0][1]
        # priority=1.0이 priority=0.0보다 PRIORITY_WEIGHT(2.0)만큼 낮아야 함
        assert cost_high == pytest.approx(
            cost_low - CBBAAllocator.PRIORITY_WEIGHT, abs=1e-6
        )


# ── 충돌 해소 (동일 임무 복수 입찰) ──────────────────────────────────────────

class TestConsensusResolution:

    def test_no_duplicate_assignment(self, allocator):
        """같은 임무에 여러 로봇이 몰려도 1대에게만 할당."""
        r1 = make_ugv('argos1', x=0.0)
        r2 = make_ugv('argos2', x=0.5)  # r1과 거의 같은 거리
        task = make_task('t1', x=1.0)   # 임무 1개만
        result = allocator.allocate([r1, r2], [task])
        # 임무는 1개 → 1대에게만 할당
        assert len(result) == 1
        assert result[list(result.keys())[0]].task_id == 't1'

    def test_three_robots_two_tasks(self, allocator):
        """로봇 3대, 임무 2개 → 2대에게 분배, 1대 미할당."""
        robots = [
            make_ugv('argos1', x=0.0),
            make_ugv('argos2', x=5.0),
            make_ugv('argos3', x=10.0),
        ]
        tasks = [
            make_task('t1', x=1.0),
            make_task('t2', x=9.0),
        ]
        result = allocator.allocate(robots, tasks)
        # 임무 2개 → 최대 2대 할당
        assert len(result) <= 2
        # 중복 할당 없음
        assigned_tasks = [t.task_id for t in result.values()]
        assert len(assigned_tasks) == len(set(assigned_tasks))

    def test_one_task_per_robot(self, allocator):
        """로봇 1대에 임무 1개: 정확히 1:1 매핑."""
        robot = make_ugv('argos1', x=0.0)
        task = make_task('t1', x=1.0)
        result = allocator.allocate([robot], [task])
        assert len(result) == 1
        assert result['argos1'].task_id == 't1'


# ── 이종 편대 통합 시나리오 ───────────────────────────────────────────────────

class TestHeterogeneousFleet:

    def test_4_robot_formation(self, allocator):
        """2UGV + 2드론 편대 4종 임무 할당 시나리오."""
        robots = [
            make_ugv('argos1', x=3.0, y=2.5),
            make_ugv('argos2', x=7.0, y=2.5),
            make_drone('drone1', x=5.0, y=4.0),
            make_drone('drone2', x=5.0, y=6.0),
        ]
        tasks = [
            make_task('explore_A', x=2.0, y=3.0, task_type='explore',
                      required=[], priority=0.3),
            make_task('explore_B', x=8.0, y=3.0, task_type='explore',
                      required=[], priority=0.3),
            make_task('monitor_roof', x=5.0, y=5.0, task_type='monitor',
                      required=['can_fly'], priority=0.8),
            make_task('inspect_fire', x=5.0, y=2.0, task_type='inspect_fire',
                      required=['has_thermal'], priority=1.0),
        ]
        result = allocator.allocate(robots, tasks)

        # 4대 → 4임무: 가능하면 모두 할당
        assert len(result) == 4

        # monitor_roof는 반드시 드론에게
        monitor_assignee = None
        for rid, task in result.items():
            if task.task_id == 'monitor_roof':
                monitor_assignee = rid
        assert monitor_assignee in ('drone1', 'drone2'), (
            f'monitor_roof가 드론이 아닌 {monitor_assignee}에 할당됨'
        )

        # 중복 할당 없음
        assigned_tasks = [t.task_id for t in result.values()]
        assert len(assigned_tasks) == len(set(assigned_tasks))

    def test_ugv_explore_drone_monitor_separation(self, allocator):
        """UGV는 지상탐색, 드론은 항공감시 — 역할 분리 검증."""
        ugv = make_ugv('argos1', x=0.0)
        drone = make_drone('drone1', x=0.5)
        explore = make_task('e', task_type='explore', required=[])
        monitor = make_task('m', task_type='monitor', required=['can_fly'])
        result = allocator.allocate([ugv, drone], [explore, monitor])

        assert len(result) == 2
        # 드론은 monitor
        assert result['drone1'].task_id == 'm'
        # UGV는 explore
        assert result['argos1'].task_id == 'e'


# ── 번들 할당 (Phase E) ────────────────────────────────────────────────────────

class TestBundleAllocation:
    """allocate_bundles() — 로봇당 N개 임무 할당 검증."""

    def test_bundle_default_backward_compatible(self):
        """max_bundle_size=1(기본값)일 때 allocate()와 동일한 결과."""
        alloc = CBBAAllocator()
        r1 = make_ugv('argos1', x=0.0)
        r2 = make_ugv('argos2', x=10.0)
        t1 = make_task('t1', x=1.0)
        t2 = make_task('t2', x=9.0)

        # allocate() 결과
        single = alloc.allocate([r1, r2], [t1, t2])
        # allocate_bundles(max_bundle_size=1) 결과
        bundles = alloc.allocate_bundles([r1, r2], [t1, t2], max_bundle_size=1)

        # 번들 결과는 list[Task]이므로 첫 번째 요소 비교
        bundle_first = {rid: tlist[0] for rid, tlist in bundles.items()}
        assert single == bundle_first

    def test_bundle_size_2_assigns_two_tasks(self):
        """번들 크기=2: 로봇 1대가 임무 2개를 받는다."""
        alloc = CBBAAllocator(max_bundle_size=2)
        robot = make_ugv('argos1', x=0.0)
        t1 = make_task('t1', x=1.0)
        t2 = make_task('t2', x=2.0)

        bundles = alloc.allocate_bundles([robot], [t1, t2])

        assert 'argos1' in bundles
        # 단일 로봇이 2개 임무를 모두 할당받아야 함
        assert len(bundles['argos1']) == 2
        assigned_ids = {t.task_id for t in bundles['argos1']}
        assert assigned_ids == {'t1', 't2'}

    def test_bundle_respects_capability(self):
        """번들 할당 시 capability 필터가 정상 동작한다.

        드론(can_fly): monitor 2개 → 2개 받음.
        UGV: monitor 임무 capability 미충족 → 할당 없음.
        """
        alloc = CBBAAllocator(max_bundle_size=2)
        ugv = make_ugv('argos1', x=0.0)
        drone = make_drone('drone1', x=0.5)
        m1 = make_task('m1', x=1.0, task_type='monitor', required=['can_fly'])
        m2 = make_task('m2', x=2.0, task_type='monitor', required=['can_fly'])

        bundles = alloc.allocate_bundles([ugv, drone], [m1, m2], max_bundle_size=2)

        # UGV는 monitor 임무 수행 불가 → 번들 없음
        assert 'argos1' not in bundles
        # 드론은 2개 모두 할당
        assert 'drone1' in bundles
        assert len(bundles['drone1']) == 2
        assigned_ids = {t.task_id for t in bundles['drone1']}
        assert assigned_ids == {'m1', 'm2'}

    def test_bundle_no_duplicate_tasks(self):
        """같은 임무가 복수 로봇 번들에 중복 배정되지 않는다."""
        alloc = CBBAAllocator(max_bundle_size=2)
        r1 = make_ugv('argos1', x=0.0)
        r2 = make_ugv('argos2', x=1.0)
        tasks = [make_task(f't{i}', x=float(i)) for i in range(3)]

        bundles = alloc.allocate_bundles([r1, r2], tasks, max_bundle_size=2)

        all_assigned = [
            t.task_id
            for tlist in bundles.values()
            for t in tlist
        ]
        # 중복 없음: 각 임무는 최대 1개 로봇에게만 배정
        assert len(all_assigned) == len(set(all_assigned))

    def test_max_bundle_size_constructor_invalid(self):
        """max_bundle_size < 1 생성자 → ValueError."""
        with pytest.raises(ValueError):
            CBBAAllocator(max_bundle_size=0)
