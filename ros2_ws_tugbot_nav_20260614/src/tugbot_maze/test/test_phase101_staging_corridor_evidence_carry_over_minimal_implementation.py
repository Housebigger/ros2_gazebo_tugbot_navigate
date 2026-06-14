from __future__ import annotations

import math
from pathlib import Path
import sys
from typing import Any

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src' / 'tugbot_maze'))

from tugbot_maze.grid_utils import OccupancyGridInfo, OccupancyGridView
import tugbot_maze.maze_perception as perception
from tugbot_maze.maze_perception import plan_two_step_corridor_alignment_staging_goal

MAZE_EXPLORER = ROOT / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_explorer.py'
MAZE_PERCEPTION = ROOT / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_perception.py'
PHASE101_REPORT = ROOT / 'doc' / 'doc_report' / 'phase101_staging_corridor_evidence_carry_over_minimal_implementation_report.md'


REJECT_REASONS = {
    'carry_over_source_stale',
    'frame_mismatch',
    'heading_mismatch',
    'forward_window_not_trustworthy',
    'staging_not_consistent_with_source_corridor',
    'source_missing_same_corridor_or_two_side_wall',
    'source_branch_or_goal_sequence_mismatch',
    'staging_safety_recompute_failed',
    'insufficient_carry_over_evidence',
}

ALLOWED_CARRY_OVER_FIELDS = {
    'same_corridor',
    'two_side_wall_evidence',
    'corridor_heading',
    'wall_clearance_context',
    'source_forward_window',
}

FORBIDDEN_CARRY_OVER_FIELDS = {
    'hard_safety_pass',
    'safety_floor_ok',
    'occupancy_free',
    'target_has_clearance',
    'footprint_lethal_not_increased',
    'front_wedge_lethal_not_increased',
    'fresh_scan_local_costmap_tf',
}


def _read(path: Path) -> str:
    assert path.exists(), f'missing {path}'
    return path.read_text(encoding='utf-8')


def _dummy_grid() -> OccupancyGridView:
    info = OccupancyGridInfo(width=20, height=20, resolution=0.05, origin_x=-0.5, origin_y=-0.5)
    return OccupancyGridView(info, [0] * (info.width * info.height), occupied_threshold=65)


def _phase101_source_forward_refinement(**overrides: Any) -> dict[str, Any]:
    rejected = []
    for index, y in enumerate([0.90, 1.00, 1.10, 1.20]):
        rejected.append({
            'candidate_index': index,
            'target_xy': [0.0, y],
            'target_yaw': math.pi / 2.0,
            'same_corridor': True,
            'two_side_wall_evidence': True,
            'occupancy_free': True,
            'target_has_clearance': True,
            'safety_floor_ok': False,
            'footprint_lethal_not_increased': index % 2 == 0,
            'front_wedge_lethal_not_increased': False,
            'forward_progress_ok': True,
            'min_clearance_m': 0.42,
            'left_wall_clearance_m': 0.60,
            'right_wall_clearance_m': 0.60,
            'left_wall_hit': True,
            'right_wall_hit': True,
            'candidate_reject_reasons': ['safety_floor_ok'],
        })
    source = {
        'enabled': True,
        'goal_sequence': 7,
        'branch_context_id': 'goal1-branch-a',
        'source_event_type': 'dispatch',
        'source_timestamp': 10.0,
        'frame_id': 'map',
        'corridor_heading_yaw': math.pi / 2.0,
        'candidate_count': len(rejected),
        'hard_safety_pass_candidate_count': 0,
        'refinement_applied': False,
        'refinement_reject_reason': 'lethal_cost_regression',
        'original_target_preserved_on_reject': True,
        'original_metrics': {
            'target': [0.0, 1.0],
            'same_corridor': True,
            'two_side_wall_evidence': True,
            'left_wall_clearance_m': 0.60,
            'right_wall_clearance_m': 0.60,
            'left_wall_hit': True,
            'right_wall_hit': True,
        },
        'rejected_candidate_summaries': rejected,
        'multi_candidate_forward_search': {
            'enabled': True,
            'goal_sequence': 7,
            'branch_context_id': 'goal1-branch-a',
            'source_event_type': 'dispatch',
            'source_timestamp': 10.0,
            'frame_id': 'map',
            'corridor_heading_yaw': math.pi / 2.0,
            'candidate_count': len(rejected),
            'hard_safety_pass_candidate_count': 0,
            'refinement_applied': False,
            'refinement_reject_reason': 'lethal_cost_regression',
            'original_target_preserved_on_reject': True,
            'original_metrics': {
                'target': [0.0, 1.0],
                'same_corridor': True,
                'two_side_wall_evidence': True,
                'left_wall_clearance_m': 0.60,
                'right_wall_clearance_m': 0.60,
                'left_wall_hit': True,
                'right_wall_hit': True,
            },
            'rejected_candidate_summaries': rejected,
            'branch_scoring_changed': False,
            'fallback_terminal_acceptance_used': False,
        },
        'execution_time_footprint_front_wedge_risk': True,
        'branch_scoring_changed': False,
        'fallback_terminal_acceptance_used': False,
    }
    for key, value in overrides.items():
        source[key] = value
        if isinstance(source.get('multi_candidate_forward_search'), dict):
            source['multi_candidate_forward_search'][key] = value
    return source


def _patch_staging_metrics(monkeypatch, *, local_two_side: bool = False, safety_ok: bool = True, target_clearance: bool = True):
    def fake_metrics(**kwargs):
        target = kwargs['target_xy']
        dispatch = kwargs['dispatch_xy']
        direction = kwargs['direction_rad']
        progress = ((float(target[0]) - float(dispatch[0])) * math.cos(direction)
                    + (float(target[1]) - float(dispatch[1])) * math.sin(direction))
        is_baseline = abs(float(target[0]) - float(dispatch[0])) < 1e-9 and abs(float(target[1]) - float(dispatch[1])) < 1e-9
        return {
            'target': [float(target[0]), float(target[1])],
            'same_corridor': True,
            'two_side_wall_evidence': True if is_baseline else local_two_side,
            'occupancy_cell_value': 0,
            'occupancy_free': True,
            'target_has_clearance': True if is_baseline else target_clearance,
            'left_wall_clearance_m': 0.60 if local_two_side else 1.50,
            'right_wall_clearance_m': 0.60 if local_two_side else 1.50,
            'left_wall_hit': bool(local_two_side),
            'right_wall_hit': bool(local_two_side),
            'balance_error_m': 0.0,
            'min_clearance_m': 0.60 if safety_ok else 0.10,
            'nearest_obstacle_distance_m': 0.60 if safety_ok else 0.10,
            'forward_progress_m': float(progress),
            'local_cost_sample_count': 8,
            'local_cost_max_radius': 10,
            'local_cost_mean_radius': 10.0,
            'footprint_high_cost_count': 0,
            'footprint_lethal_count': 0,
            'front_wedge_sample_count': 8,
            'front_wedge_cost_max': 10,
            'front_wedge_high_cost_count': 0,
            'front_wedge_lethal_count': 0,
        }
    monkeypatch.setattr(perception, '_centerline_candidate_metrics', fake_metrics)


def _plan_with_phase101_defaults(**kwargs: Any) -> dict[str, Any]:
    params = {
        'map_grid': _dummy_grid(),
        'local_cost_grid': _dummy_grid(),
        'dispatch_pose': (0.20, 0.0, math.pi / 2.0),
        'original_target': (0.0, 1.0),
        'direction_rad': math.pi / 2.0,
        'clearance_radius_m': 0.35,
        'source_forward_refinement': _phase101_source_forward_refinement(),
        'min_clearance_floor_m': 0.35,
        'staging_lateral_offsets_m': (0.10,),
        'staging_forward_offsets_m': (0.05,),
        'max_staging_distance_m': 0.35,
        'current_goal_sequence': 7,
        'current_branch_context_id': 'goal1-branch-a',
        'current_timestamp_sec': 12.0,
        'staging_frame_id': 'map',
    }
    params.update(kwargs)
    return plan_two_step_corridor_alignment_staging_goal(**params)


def test_phase101_carry_over_applies_only_corridor_evidence_when_staging_window_lacks_two_side_wall(monkeypatch):
    _patch_staging_metrics(monkeypatch, local_two_side=False, safety_ok=True, target_clearance=True)

    result = _plan_with_phase101_defaults()

    assert result['corridor_evidence_carry_over']['evaluated'] is True
    assert result['corridor_evidence_carry_over']['eligible'] is True
    assert set(result['corridor_evidence_carry_over']['allowed_fields']) == ALLOWED_CARRY_OVER_FIELDS
    assert set(result['corridor_evidence_carry_over']['forbidden_fields']) == FORBIDDEN_CARRY_OVER_FIELDS
    assert result['carry_over_applied'] is True
    assert result['carry_over_reject_reason'] is None
    assert result['safety_evidence_recomputed'] is True
    assert result['staging_applied'] is True
    assert result['staging_goal_pose'] is not None
    assert result['source_forward_window']['two_side_wall_count'] >= 1
    assert result['staging_window']['two_side_wall_count'] == 0

    check = result['staging_executability_check']
    assert check['hard_safety_pass'] is True
    assert check['same_corridor'] is True
    assert check['two_side_wall_evidence'] is True
    assert check['local_two_side_wall_evidence'] is False
    assert check['corridor_evidence_source'] == 'phase88_forward_window_carry_over'
    assert check['target_has_clearance'] is True
    assert check['occupancy_free'] is True
    assert check['safety_floor_ok'] is True
    assert check['footprint_lethal_not_increased'] is True
    assert check['front_wedge_lethal_not_increased'] is True
    assert result['branch_scoring_changed'] is False
    assert result['fallback_terminal_acceptance_used'] is False


def test_phase101_forbidden_safety_evidence_is_not_reused_and_staging_safety_recompute_can_reject(monkeypatch):
    _patch_staging_metrics(monkeypatch, local_two_side=False, safety_ok=False, target_clearance=False)
    source = _phase101_source_forward_refinement(
        hard_safety_pass=True,
        safety_floor_ok=True,
        occupancy_free=True,
        target_has_clearance=True,
        footprint_lethal_not_increased=True,
        front_wedge_lethal_not_increased=True,
    )

    result = _plan_with_phase101_defaults(source_forward_refinement=source)

    assert result['corridor_evidence_carry_over']['evaluated'] is True
    assert result['corridor_evidence_carry_over']['eligible'] is True
    assert result['carry_over_applied'] is True
    assert result['carry_over_reject_reason'] == 'staging_safety_recompute_failed'
    assert result['safety_evidence_recomputed'] is True
    assert result['staging_applied'] is False
    assert result['staging_goal_pose'] is None
    assert result['staging_reject_reason'] == 'staging_safety_recompute_failed'
    check = result['staging_executability_check']
    assert check['hard_safety_pass'] is False
    assert check['target_has_clearance'] is False
    assert check['safety_floor_ok'] is False
    assert check['local_two_side_wall_evidence'] is False


def test_phase101_carry_over_rejects_source_and_geometry_mismatches(monkeypatch):
    _patch_staging_metrics(monkeypatch, local_two_side=False, safety_ok=True, target_clearance=True)

    cases = [
        ({'current_timestamp_sec': 99.0}, 'carry_over_source_stale'),
        ({'source_forward_refinement': _phase101_source_forward_refinement(frame_id='odom')}, 'frame_mismatch'),
        ({'source_forward_refinement': _phase101_source_forward_refinement(corridor_heading_yaw=0.0)}, 'heading_mismatch'),
        ({'source_forward_refinement': _phase101_source_forward_refinement(goal_sequence=8)}, 'source_branch_or_goal_sequence_mismatch'),
        ({'source_forward_refinement': _phase101_source_forward_refinement(branch_context_id='other-branch')}, 'source_branch_or_goal_sequence_mismatch'),
    ]
    for overrides, reason in cases:
        result = _plan_with_phase101_defaults(**overrides)
        assert result['corridor_evidence_carry_over']['evaluated'] is True
        assert result['carry_over_applied'] is False
        assert result['carry_over_reject_reason'] == reason
        assert result['staging_applied'] is False

    missing_window = {
        'candidate_count': 1,
        'x_range': {'min': 0.0, 'max': 0.1},
        'y_range': {'min': 1.0, 'max': 1.1},
        'same_corridor_count': 1,
        'two_side_wall_count': 0,
        'corridor_heading': math.pi / 2.0,
        'wall_clearance_context': {'left_wall_hit': True, 'right_wall_hit': False},
    }
    result = perception._evaluate_corridor_evidence_carry_over(
        source={'goal_sequence': 7, 'branch_context_id': 'goal1-branch-a', 'frame_id': 'map', 'source_timestamp': 10.0},
        source_forward_window=missing_window,
        staging_window={'candidate_count': 1, 'two_side_wall_count': 0},
        candidates=[{'target_xy': [0.1, 0.1], 'same_corridor': True}],
        dispatch_pose=(0.2, 0.0, math.pi / 2.0),
        original_target=(0.0, 1.0),
        direction_rad=math.pi / 2.0,
        current_goal_sequence=7,
        current_branch_context_id='goal1-branch-a',
        current_timestamp_sec=12.0,
        staging_frame_id='map',
        source_max_age_sec=30.0,
        heading_tolerance_rad=math.radians(25.0),
    )
    assert result['carry_over_reject_reason'] == 'source_missing_same_corridor_or_two_side_wall'

    no_window = dict(missing_window)
    no_window['two_side_wall_count'] = 1
    no_window['x_range'] = {'min': None, 'max': None}
    result = perception._evaluate_corridor_evidence_carry_over(
        source={'goal_sequence': 7, 'branch_context_id': 'goal1-branch-a', 'frame_id': 'map', 'source_timestamp': 10.0},
        source_forward_window=no_window,
        staging_window={'candidate_count': 1, 'two_side_wall_count': 0},
        candidates=[{'target_xy': [0.1, 0.1], 'same_corridor': True}],
        dispatch_pose=(0.2, 0.0, math.pi / 2.0),
        original_target=(0.0, 1.0),
        direction_rad=math.pi / 2.0,
        current_goal_sequence=7,
        current_branch_context_id='goal1-branch-a',
        current_timestamp_sec=12.0,
        staging_frame_id='map',
        source_max_age_sec=30.0,
        heading_tolerance_rad=math.radians(25.0),
    )
    assert result['carry_over_reject_reason'] == 'forward_window_not_trustworthy'

    behind_target = _phase101_source_forward_refinement()
    result = _plan_with_phase101_defaults(original_target=(0.0, -1.0), source_forward_refinement=behind_target)
    assert result['carry_over_reject_reason'] == 'staging_not_consistent_with_source_corridor'


def test_phase101_reject_reason_inventory_and_goal_events_contract_are_exposed():
    source = _read(MAZE_EXPLORER)
    perception_source = _read(MAZE_PERCEPTION)
    send_goal = source[source.index('def _send_goal'):source.index('def _goal_feedback_callback')]

    for reason in REJECT_REASONS:
        assert reason in perception_source

    for token in [
        'corridor_evidence_carry_over',
        'carry_over_source',
        'carry_over_applied',
        'carry_over_reject_reason',
        'source_forward_window',
        'staging_window',
        'safety_evidence_recomputed',
        'branch_scoring_changed',
        'fallback_terminal_acceptance_used',
    ]:
        assert token in perception_source
        assert token in send_goal

    assert "'branch_scoring_changed': False" in perception_source or 'branch_scoring_changed=false' in perception_source
    assert "'fallback_terminal_acceptance_used': False" in perception_source or 'fallback_terminal_acceptance_used=false' in perception_source
    assert 'choose_next_branch' in source
    assert 'near_exit_terminal_acceptance' in source
    assert 'set_parameters' not in perception_source


def test_phase101_report_records_validation_guardrails_and_stop_condition():
    report = _read(PHASE101_REPORT)
    for token in [
        'Phase101',
        'Staging corridor evidence carry-over minimal implementation',
        'carry_over_applied=true does not mean staging_applied=true',
        'safety_evidence_recomputed=true',
        'hard_safety_pass',
        'occupancy_free',
        'target_has_clearance',
        'footprint_lethal_not_increased',
        'front_wedge_lethal_not_increased',
        'No branch scoring change',
        'No exploration order change',
        'No centerline gate change',
        'No directional readiness change',
        'No fallback/terminal acceptance change',
        'No Nav2/MPPI/controller tuning',
        'No inflation/robot_radius/clearance_radius_m/map threshold tuning',
        'No autonomous exploration success claimed',
        'No exit success claimed',
        'Stop: do not enter Phase102',
    ]:
        assert token in report
