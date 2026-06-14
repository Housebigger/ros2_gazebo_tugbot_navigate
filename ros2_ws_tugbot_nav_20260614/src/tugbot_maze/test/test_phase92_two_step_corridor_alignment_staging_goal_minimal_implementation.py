from __future__ import annotations

import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src' / 'tugbot_maze'))

from tugbot_maze.grid_utils import OccupancyGridInfo, OccupancyGridView
from tugbot_maze.maze_perception import (
    generate_second_step_forward_goal_after_staging,
    plan_two_step_corridor_alignment_staging_goal,
)

MAZE_EXPLORER = ROOT / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_explorer.py'
MAZE_PERCEPTION = ROOT / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_perception.py'
PHASE92_REPORT = ROOT / 'doc' / 'doc_report' / 'phase92_two_step_corridor_alignment_staging_goal_minimal_implementation_report.md'


def _read(path: Path) -> str:
    assert path.exists(), f'missing {path}'
    return path.read_text(encoding='utf-8')


def _corridor_grid(width_m=4.0, height_m=4.0, resolution=0.05):
    width = int(width_m / resolution)
    height = int(height_m / resolution)
    origin_x = -width_m / 2.0
    origin_y = -height_m / 2.0
    data = [0] * (width * height)
    info = OccupancyGridInfo(width=width, height=height, resolution=resolution, origin_x=origin_x, origin_y=origin_y)
    for y in range(height):
        for x in range(width):
            wx = origin_x + (x + 0.5) * resolution
            if abs(wx) >= 0.95:
                data[x + y * width] = 100
    return OccupancyGridView(info, data, occupied_threshold=65)


def _flat_local_cost_grid(width_m=4.0, height_m=4.0, resolution=0.05, default_cost=10):
    width = int(width_m / resolution)
    height = int(height_m / resolution)
    origin_x = -width_m / 2.0
    origin_y = -height_m / 2.0
    data = [int(default_cost)] * (width * height)
    return OccupancyGridView(
        OccupancyGridInfo(width=width, height=height, resolution=resolution, origin_x=origin_x, origin_y=origin_y),
        data,
        occupied_threshold=99,
    )


def _phase90_style_source_forward_refinement(*, hard_safe_count=0, safety_floor_ok=False, front_wedge_ok=False):
    rejected = []
    for index in range(6):
        rejected.append({
            'candidate_index': index,
            'target_xy': [0.0, 1.0 + 0.05 * index],
            'same_corridor': True,
            'two_side_wall_evidence': True,
            'occupancy_free': True,
            'target_has_clearance': True,
            'safety_floor_ok': safety_floor_ok,
            'footprint_lethal_not_increased': index % 2 == 0,
            'front_wedge_lethal_not_increased': front_wedge_ok,
            'forward_progress_ok': True,
            'candidate_reject_reasons': [] if safety_floor_ok else ['safety_floor_ok'],
        })
    return {
        'enabled': True,
        'candidate_count': len(rejected),
        'hard_safety_pass_candidate_count': hard_safe_count,
        'refinement_applied': False,
        'refinement_reject_reason': 'lethal_cost_regression',
        'original_target_preserved_on_reject': True,
        'rejected_candidate_summaries': rejected,
        'multi_candidate_forward_search': {
            'enabled': True,
            'candidate_count': len(rejected),
            'hard_safety_pass_candidate_count': hard_safe_count,
            'refinement_applied': False,
            'refinement_reject_reason': 'lethal_cost_regression',
            'original_target_preserved_on_reject': True,
            'rejected_candidate_summaries': rejected,
            'branch_scoring_changed': False,
            'fallback_terminal_acceptance_used': False,
        },
        'branch_scoring_changed': False,
        'fallback_terminal_acceptance_used': False,
    }


def test_phase92_trigger_bundle_positive_generates_short_hard_safe_staging_pose():
    direction = math.pi / 2.0
    dispatch_pose = (0.45, 1.0, direction)
    original_target = (0.0, 1.02)

    result = plan_two_step_corridor_alignment_staging_goal(
        map_grid=_corridor_grid(),
        local_cost_grid=_flat_local_cost_grid(),
        dispatch_pose=dispatch_pose,
        original_target=original_target,
        direction_rad=direction,
        clearance_radius_m=0.35,
        source_forward_refinement=_phase90_style_source_forward_refinement(),
        min_clearance_floor_m=0.35,
        staging_lateral_offsets_m=(0.15, 0.25, 0.35),
        staging_forward_offsets_m=(0.0, 0.05),
        max_staging_distance_m=0.35,
        local_cost_radius_m=0.04,
        front_wedge_radius_m=0.05,
    )

    assert result['two_step_staging_plan']['enabled'] is True
    assert all(result['two_step_staging_plan']['trigger_conditions'].values())
    assert result['staging_applied'] is True
    assert result['two_step_stage_dispatch_requested'] is True
    pose = result['staging_goal_pose']
    assert pose is not None
    assert math.hypot(pose['x'] - dispatch_pose[0], pose['y'] - dispatch_pose[1]) <= 0.35 + 1e-9
    assert abs(pose['yaw'] - direction) < 1e-9
    assert abs(pose['lateral_residual_after_m']) < abs(pose['lateral_residual_before_m'])
    check = result['staging_executability_check']
    assert check['hard_safety_pass'] is True
    assert check['same_corridor'] is True
    assert check['two_side_wall_evidence'] is True
    assert check['safety_floor_ok'] is True
    assert check['footprint_lethal_not_increased'] is True
    assert check['front_wedge_lethal_not_increased'] is True
    assert result['second_step_forward_goal'] is None
    assert result['branch_scoring_changed'] is False
    assert result['fallback_terminal_acceptance_used'] is False


def test_phase92_trigger_negative_preserves_original_target_when_single_step_has_hard_safe_candidate():
    direction = math.pi / 2.0
    result = plan_two_step_corridor_alignment_staging_goal(
        map_grid=_corridor_grid(),
        local_cost_grid=_flat_local_cost_grid(),
        dispatch_pose=(0.45, 1.0, direction),
        original_target=(0.0, 1.02),
        direction_rad=direction,
        clearance_radius_m=0.35,
        source_forward_refinement=_phase90_style_source_forward_refinement(hard_safe_count=1),
        min_clearance_floor_m=0.35,
    )

    assert result['staging_applied'] is False
    assert result['two_step_stage_dispatch_requested'] is False
    assert result['staging_goal_pose'] is None
    assert result['refined_target'] == result['original_target']
    assert result['original_target_preserved_on_reject'] is True
    assert result['staging_reject_reason'] == 'single_step_forward_search_had_hard_safe_candidate'
    assert result['two_step_staging_plan']['trigger_conditions']['single_step_forward_search_no_hard_safety_pass'] is False
    assert result['branch_scoring_changed'] is False
    assert result['fallback_terminal_acceptance_used'] is False


def test_phase92_staging_rejects_when_candidate_cannot_pass_safety_floor():
    direction = math.pi / 2.0
    result = plan_two_step_corridor_alignment_staging_goal(
        map_grid=_corridor_grid(),
        local_cost_grid=_flat_local_cost_grid(),
        dispatch_pose=(0.45, 1.0, direction),
        original_target=(0.0, 1.02),
        direction_rad=direction,
        clearance_radius_m=0.35,
        source_forward_refinement=_phase90_style_source_forward_refinement(),
        min_clearance_floor_m=0.90,
        staging_lateral_offsets_m=(0.15, 0.25, 0.35),
        staging_forward_offsets_m=(0.0,),
        max_staging_distance_m=0.35,
    )

    assert result['two_step_staging_plan']['enabled'] is True
    assert result['staging_applied'] is False
    assert result['staging_goal_pose'] is None
    assert result['refined_target'] == result['original_target']
    assert result['staging_reject_reason'] == 'staging_safety_floor_blocked'
    assert result['staging_executability_check']['hard_safety_pass'] is False
    assert result['staging_executability_check']['safety_floor_ok'] is False
    assert result['original_target_preserved_on_reject'] is True


def test_phase92_second_step_requires_fresh_scan_local_costmap_and_tf_before_phase88_refinement():
    direction = math.pi / 2.0
    stale = generate_second_step_forward_goal_after_staging(
        map_grid=_corridor_grid(),
        local_cost_grid=_flat_local_cost_grid(),
        staged_pose=(0.20, 1.0, direction),
        original_target=(0.0, 1.55),
        direction_rad=direction,
        clearance_radius_m=0.35,
        fresh_scan_received=True,
        fresh_local_costmap_received=False,
        fresh_tf_received=True,
        min_clearance_floor_m=0.35,
    )

    assert stale['generated_after_fresh_evidence'] is False
    assert stale['selected_candidate_target'] is None
    assert stale['reject_reason'] == 'second_step_fresh_evidence_missing'
    assert stale['fresh_scan_received'] is True
    assert stale['fresh_local_costmap_received'] is False
    assert stale['fresh_tf_received'] is True

    fresh = generate_second_step_forward_goal_after_staging(
        map_grid=_corridor_grid(),
        local_cost_grid=_flat_local_cost_grid(),
        staged_pose=(0.20, 1.0, direction),
        original_target=(0.0, 1.55),
        direction_rad=direction,
        clearance_radius_m=0.35,
        fresh_scan_received=True,
        fresh_local_costmap_received=True,
        fresh_tf_received=True,
        min_clearance_floor_m=0.35,
        forward_offsets_m=(0.0, 0.1),
        lateral_offsets_m=(0.0, 0.20),
        heading_offsets_rad=(0.0,),
        local_cost_radius_m=0.04,
        front_wedge_radius_m=0.05,
    )

    assert fresh['generated_after_fresh_evidence'] is True
    assert fresh['selected_candidate_target'] is not None
    assert fresh['selected_candidate_yaw'] is not None
    assert fresh['hard_safety_pass_candidate_count'] >= 1
    assert fresh['forward_refinement_result']['refinement_applied'] is True
    assert fresh['branch_scoring_changed'] is False
    assert fresh['fallback_terminal_acceptance_used'] is False


def test_phase92_goal_events_contract_and_runtime_guardrails_are_exposed_in_maze_explorer():
    source = _read(MAZE_EXPLORER)
    perception = _read(MAZE_PERCEPTION)
    send_goal = source[source.index('def _send_goal'):source.index('def _goal_feedback_callback')]
    success = source[source.index('def _handle_goal_success'):source.index('def _handle_goal_failure')]
    publish = source[source.index('def _publish_goal_event'):source.index('def _mark_exhausted')]

    for token in [
        'two_step_staging_plan',
        'staging_goal_pose',
        'staging_reason',
        'staging_executability_check',
        'second_step_forward_goal',
        'staging_applied',
        'staging_reject_reason',
        'branch_scoring_changed',
        'fallback_terminal_acceptance_used',
    ]:
        assert token in perception
        assert token in send_goal
        assert token in publish

    for token in [
        'corridor_alignment_staging',
        'two_step_stage_dispatch_requested',
        '_dispatch_second_step_after_corridor_alignment_staging',
        'fresh_scan_received',
        'fresh_local_costmap_received',
        'fresh_tf_received',
    ]:
        assert token in source
    assert "completed_goal_kind == 'corridor_alignment_staging'" in success
    assert 'self.topology.choose_next_branch(node.node_id, exit_xy=(self.exit_x, self.exit_y))' in source
    assert 'near_exit_terminal_acceptance' in source


def test_phase92_report_records_validation_guardrails_and_stop_condition():
    report = _read(PHASE92_REPORT)
    for token in [
        'Phase92',
        'Two-step corridor alignment staging goal minimal implementation',
        'Phase90-style trigger bundle',
        'near-goal lateral residual',
        'single-step forward search no hard-safety-pass',
        'safety_floor dominant blocker',
        'execution-time footprint/front-wedge risk',
        'staging is not fallback',
        'staging is not terminal acceptance',
        'timeout is not success',
        'fresh scan/local costmap/TF',
        'branch_scoring_changed=false',
        'fallback_terminal_acceptance_used=false',
        'No Nav2/MPPI/controller tuning',
        'No inflation/robot_radius/clearance_radius_m/map threshold tuning',
        'No autonomous exploration success claimed',
        'No exit success claimed',
        'Stop: do not enter Phase93',
    ]:
        assert token in report
