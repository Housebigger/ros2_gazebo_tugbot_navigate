from __future__ import annotations

import json
import math
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src' / 'tugbot_maze'))

from tugbot_maze.grid_utils import OccupancyGridInfo, OccupancyGridView
from tugbot_maze.maze_perception import plan_two_step_corridor_alignment_staging_goal

MAZE_EXPLORER = ROOT / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_explorer.py'
MAZE_PERCEPTION = ROOT / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_perception.py'
ANALYZER = ROOT / 'tools' / 'analyze_phase143_staging_gate_artifact_completeness.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase143_staging_gate_artifact_completeness_minimal_implementation_report.md'

REQUIRED_TOP_LEVEL_FIELDS = [
    'lateral_residual_before_m',
    'lateral_residual_after_m',
    'target_local_cost',
    'target_local_cost_max_radius',
    'path_corridor_min_clearance_m',
    'target_clearance_m',
    'candidate_branch_count',
    'last_open_direction_count',
    'last_candidate_count',
    'branch_angle',
    'selected_branch_geometry',
    'source_single_step',
    'trigger_conditions',
    'staging_reason',
    'staging_reject_reason',
    'gate_classification_candidate',
    'gate_artifact_missing_fields',
    'gate_artifact_complete',
]

REQUIRED_SOURCE_SINGLE_STEP_FIELDS = [
    'candidate_count',
    'hard_safety_pass_candidate_count',
    'hard_safe_candidate_summaries',
    'refinement_applied',
    'refinement_reject_reason',
    'original_target_preserved_on_reject',
    'selected_candidate_index',
    'selected_candidate_target',
    'selected_candidate_yaw',
]

REQUIRED_TRIGGER_FIELDS = [
    'near_goal_lateral_residual',
    'single_step_forward_search_no_hard_safety_pass',
    'safety_floor_dominant_blocker',
    'execution_time_footprint_front_wedge_risk',
]

REQUIRED_CLASSIFICATIONS = [
    'staging_not_needed_direct_explore',
    'staging_expected_but_not_triggered',
    'staging_triggered_corridor_alignment',
    'staging_triggered_but_unsafe_or_unavailable',
    'insufficient_staging_gate_evidence',
]


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


def _candidate(index: int, *, hard_safe: bool, safety_floor_ok: bool = False, front_wedge_ok: bool = False):
    return {
        'candidate_index': index,
        'target_xy': [0.10 + index * 0.02, 1.10 + index * 0.03],
        'target': [0.10 + index * 0.02, 1.10 + index * 0.03],
        'target_yaw': math.pi / 2.0,
        'lateral_residual_before_m': 0.45,
        'lateral_residual_after_m': 0.12 + 0.01 * index,
        'target_local_cost': 10 + index,
        'target_local_cost_max_radius': 20 + index,
        'local_cost_max_radius': 20 + index,
        'path_corridor_min_clearance_m': 0.55,
        'target_clearance_m': 0.60,
        'min_clearance_m': 0.55,
        'front_wedge_cost_max': 40 + index,
        'front_wedge_high_cost_count': 2,
        'front_wedge_lethal_count': 0,
        'front_wedge_sample_count': 12,
        'footprint_lethal_count': 0,
        'same_corridor': True,
        'two_side_wall_evidence': True,
        'occupancy_free': True,
        'target_has_clearance': True,
        'safety_floor_ok': safety_floor_ok,
        'footprint_lethal_not_increased': True,
        'front_wedge_lethal_not_increased': front_wedge_ok,
        'forward_progress_ok': True,
        'hard_safety_pass': hard_safe,
        'candidate_reject_reasons': [] if hard_safe else ['safety_floor_ok'],
    }


def _source_forward_refinement(*, hard_safe_count: int):
    candidates = [_candidate(0, hard_safe=hard_safe_count > 0, safety_floor_ok=hard_safe_count > 0, front_wedge_ok=hard_safe_count > 0)]
    candidates.extend(_candidate(i, hard_safe=False, safety_floor_ok=False, front_wedge_ok=False) for i in range(1, 6))
    selected = candidates[0] if hard_safe_count > 0 else None
    rejected = [row for row in candidates if not row.get('hard_safety_pass')]
    nested = {
        'enabled': True,
        'candidate_count': len(candidates),
        'hard_safety_pass_candidate_count': hard_safe_count,
        'refinement_applied': False,
        'refinement_reject_reason': 'lethal_cost_regression',
        'original_target_preserved_on_reject': True,
        'selected_candidate_index': selected.get('candidate_index') if selected else None,
        'selected_candidate_target': selected.get('target_xy') if selected else None,
        'selected_candidate_yaw': selected.get('target_yaw') if selected else None,
        'selected_metrics': selected,
        'candidates': candidates,
        'rejected_candidate_summaries': rejected,
        'selection_priority_trace': ['phase143_test_source_only_no_branch_scoring_change'],
        'branch_scoring_changed': False,
        'fallback_terminal_acceptance_used': False,
    }
    return {
        'enabled': True,
        **nested,
        'multi_candidate_forward_search': nested,
        'branch_scoring_changed': False,
        'fallback_terminal_acceptance_used': False,
    }


def _plan(*, hard_safe_count: int):
    direction = math.pi / 2.0
    return plan_two_step_corridor_alignment_staging_goal(
        map_grid=_corridor_grid(),
        local_cost_grid=_flat_local_cost_grid(),
        dispatch_pose=(0.45, 1.0, direction),
        original_target=(0.0, 1.02),
        direction_rad=direction,
        clearance_radius_m=0.35,
        source_forward_refinement=_source_forward_refinement(hard_safe_count=hard_safe_count),
        min_clearance_floor_m=0.35,
        staging_lateral_offsets_m=(0.15, 0.25, 0.35),
        staging_forward_offsets_m=(0.0, 0.05),
        max_staging_distance_m=0.35,
        local_cost_radius_m=0.04,
        front_wedge_radius_m=0.05,
    )


def _artifact(result: dict) -> dict:
    artifact = result.get('staging_gate_artifact_completeness')
    assert isinstance(artifact, dict), result
    return artifact


def _assert_required_shape(artifact: dict) -> None:
    for field in REQUIRED_TOP_LEVEL_FIELDS:
        assert field in artifact, f'missing {field}'
    for field in REQUIRED_SOURCE_SINGLE_STEP_FIELDS:
        assert field in artifact['source_single_step'], f'missing source_single_step.{field}'
    for field in REQUIRED_TRIGGER_FIELDS:
        assert field in artifact['trigger_conditions'], f'missing trigger_conditions.{field}'
    assert isinstance(artifact['source_single_step']['hard_safe_candidate_summaries'], list)
    assert isinstance(artifact['gate_artifact_missing_fields'], list)


def test_phase143_direct_explore_reject_staging_serializes_complete_gate_artifact():
    result = _plan(hard_safe_count=1)
    artifact = _artifact(result)

    _assert_required_shape(artifact)
    assert result['staging_applied'] is False
    assert artifact['staging_reject_reason'] == 'single_step_forward_search_had_hard_safe_candidate'
    assert artifact['gate_classification_candidate'] == 'staging_not_needed_direct_explore'
    assert artifact['source_single_step']['candidate_count'] == 6
    assert artifact['source_single_step']['hard_safety_pass_candidate_count'] == 1
    assert artifact['source_single_step']['hard_safe_candidate_summaries'][0]['hard_safety_pass'] is True
    assert artifact['source_single_step']['hard_safe_candidate_summaries'][0]['target_local_cost'] is not None
    assert artifact['source_single_step']['hard_safe_candidate_summaries'][0]['path_corridor_min_clearance_m'] is not None
    assert artifact['source_single_step']['hard_safe_candidate_summaries'][0]['target_clearance_m'] is not None
    assert artifact['trigger_conditions']['single_step_forward_search_no_hard_safety_pass'] is False
    assert artifact['lateral_residual_before_m'] is not None
    assert artifact['lateral_residual_after_m'] is not None
    assert artifact['target_local_cost'] is not None
    assert artifact['target_local_cost_max_radius'] is not None
    assert artifact['path_corridor_min_clearance_m'] is not None
    assert artifact['target_clearance_m'] is not None
    assert artifact['gate_artifact_complete'] is True


def test_phase143_staging_triggered_path_serializes_symmetric_gate_artifact():
    result = _plan(hard_safe_count=0)
    artifact = _artifact(result)

    _assert_required_shape(artifact)
    assert result['staging_applied'] is True
    assert artifact['staging_reason'] == 'reduce_lateral_residual_and_front_wedge_risk_before_second_step_forward_goal'
    assert artifact['staging_reject_reason'] is None
    assert artifact['gate_classification_candidate'] == 'staging_triggered_corridor_alignment'
    assert artifact['source_single_step']['candidate_count'] == 6
    assert artifact['source_single_step']['hard_safety_pass_candidate_count'] == 0
    assert artifact['trigger_conditions']['single_step_forward_search_no_hard_safety_pass'] is True
    assert artifact['lateral_residual_before_m'] is not None
    assert artifact['lateral_residual_after_m'] is not None
    assert artifact['target_local_cost'] is not None
    assert artifact['target_local_cost_max_radius'] is not None
    assert artifact['path_corridor_min_clearance_m'] is not None
    assert artifact['target_clearance_m'] is not None
    assert artifact['gate_artifact_complete'] is True


def test_phase143_explorer_source_contains_event_context_serialization_without_gate_logic_change():
    explorer = MAZE_EXPLORER.read_text(encoding='utf-8')
    perception = MAZE_PERCEPTION.read_text(encoding='utf-8')
    combined = explorer + '\n' + perception

    for phrase in [
        'def _phase143_staging_gate_artifact_context',
        "'selected_branch_geometry'",
        "'candidate_branch_count'",
        "'last_open_direction_count'",
        "'last_candidate_count'",
        "'target_local_cost'",
        "'target_local_cost_max_radius'",
        "'path_corridor_min_clearance_m'",
        "'target_clearance_m'",
        "'source_single_step'",
        "'hard_safe_candidate_summaries'",
        "'staging_not_needed_direct_explore'",
        "'staging_expected_but_not_triggered'",
        "'staging_triggered_corridor_alignment'",
        "'staging_triggered_but_unsafe_or_unavailable'",
        "'insufficient_staging_gate_evidence'",
    ]:
        assert phrase in combined

    # Phase143 is serialization-only; the existing gate predicates and forbidden behavior flags remain visible.
    assert 'def _two_step_staging_trigger_conditions' in perception
    assert "return 'single_step_forward_search_had_hard_safe_candidate'" in perception
    assert "'branch_scoring_changed': False" in combined
    assert "'fallback_terminal_acceptance_used': False" in combined


def test_phase143_static_analyzer_classifies_both_paths_without_runtime():
    assert ANALYZER.exists(), f'missing static analyzer: {ANALYZER}'
    completed = subprocess.run(
        [sys.executable, str(ANALYZER), '--json-only'],
        cwd=ROOT,
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )
    payload = json.loads(completed.stdout)
    assert payload['phase'] == 'Phase143'
    assert payload['valid'] is True
    assert payload['runtime_executed'] is False
    assert payload['direct_explore_reject_staging']['classification'] == 'staging_not_needed_direct_explore'
    assert payload['staging_triggered']['classification'] == 'staging_triggered_corridor_alignment'
    assert payload['all_required_fields_present'] is True
    assert payload['gate_logic_changed'] is False


def test_phase143_report_records_minimal_implementation_and_stop_before_phase144():
    assert REPORT.exists(), f'missing Phase143 report: {REPORT}'
    text = REPORT.read_text(encoding='utf-8')
    required = [
        'PHASE143_STAGING_GATE_ARTIFACT_COMPLETENESS_MINIMAL_IMPLEMENTATION_COMPLETE_STOP_BEFORE_PHASE144',
        'artifact/diagnostic serialization only',
        'direct-explore reject staging path',
        'staging-triggered path',
        'staging_not_needed_direct_explore',
        'staging_expected_but_not_triggered',
        'staging_triggered_corridor_alignment',
        'staging_triggered_but_unsafe_or_unavailable',
        'insufficient_staging_gate_evidence',
        'No Gazebo/RViz/Nav2 runtime was launched',
        'No NavigateToPose goal was sent',
        'No maze_explorer runtime was started',
        'No staging/explore/third goal was sent',
        'No Nav2/MPPI/controller/goal checker/config tuning was performed',
        'No gate logic, threshold, exploration strategy, branch scoring, centerline, fallback, or terminal acceptance change was made',
        'No staging was disabled',
        'No autonomous exploration success or exit success is claimed',
        'Phase144 not entered',
    ]
    for phrase in required:
        assert phrase in text
    for forbidden in [
        'autonomous exploration success achieved',
        'exit success achieved',
        'Phase127 timeout fixed',
        'Phase141 runtime success proved',
        'tune MPPI',
        'disable staging',
        'run Phase144',
    ]:
        assert forbidden not in text


def test_phase143_source_lists_required_classification_tokens():
    source = MAZE_PERCEPTION.read_text(encoding='utf-8')
    for token in REQUIRED_CLASSIFICATIONS:
        assert token in source
