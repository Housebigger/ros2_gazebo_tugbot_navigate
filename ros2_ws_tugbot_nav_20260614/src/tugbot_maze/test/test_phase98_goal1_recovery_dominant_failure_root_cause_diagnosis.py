from __future__ import annotations

import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
ANALYZER = ROOT / 'tools' / 'analyze_phase98_goal1_recovery_dominant_failure_root_cause.py'


def load_analyzer():
    spec = importlib.util.spec_from_file_location('phase98_analyzer', ANALYZER)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_jsonl(path: Path, rows: list[dict]) -> None:
    path.write_text('\n'.join(json.dumps(row) for row in rows) + '\n', encoding='utf-8')


def make_artifacts(tmp_path: Path, *, goal1: dict | None = None, analysis_goal1: dict | None = None) -> Path:
    run_id = 'phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke'
    ad = tmp_path / run_id
    ad.mkdir()
    dispatch = {
        'event': 'dispatch',
        'goal_sequence': 1,
        'target': [2.017, 0.924],
        'selected_candidate_target': [2.017, 0.924],
        'selected_candidate_yaw': 1.5635,
        'centerline_refinement_reason': 'lethal_cost_regression',
        'centerline_refinement_applied': False,
        'staging_reject_reason': 'missing_two_side_wall_evidence',
        'staging_applied': False,
        'second_step_forward_goal': None,
        'two_step_staging_plan': {
            'enabled': True,
            'trigger_conditions': {
                'execution_time_footprint_front_wedge_risk': True,
                'near_goal_lateral_residual': True,
                'safety_floor_dominant_blocker': True,
                'single_step_forward_search_no_hard_safety_pass': True,
            },
            'source_single_step': {
                'candidate_count': 63,
                'hard_safety_pass_candidate_count': 0,
                'refinement_applied': False,
                'refinement_reject_reason': 'lethal_cost_regression',
                'original_target_preserved_on_reject': True,
            },
        },
        'staging_executability_check': {
            'checked': True,
            'reason': 'missing_two_side_wall_evidence',
            'hard_safety_pass': False,
            'two_side_wall_evidence': False,
            'same_corridor': False,
            'safety_floor_ok': False,
            'footprint_lethal_not_increased': False,
            'front_wedge_lethal_not_increased': False,
            'forward_progress_ok': False,
            'bounded_short_distance': False,
        },
        'centerline_target_refinement': {
            'candidate_count': 3,
            'hard_safety_pass_candidate_count': 0,
            'refinement_applied': False,
            'refinement_reject_reason': 'lethal_cost_regression',
            'original_target_preserved_on_reject': True,
            'branch_scoring_changed': False,
            'fallback_terminal_acceptance_used': False,
            'candidates': [
                {'candidate_index': 1, 'hard_safety_pass': False, 'safety_floor_ok': False, 'candidate_reject_reasons': ['safety_floor_ok'], 'min_clearance_m': 0.42, 'front_wedge_lethal_count': 0, 'footprint_lethal_count': 0, 'local_cost_max_radius': 46},
                {'candidate_index': 2, 'hard_safety_pass': False, 'safety_floor_ok': False, 'candidate_reject_reasons': ['front_wedge_lethal_not_increased', 'safety_floor_ok'], 'min_clearance_m': 0.30, 'front_wedge_lethal_count': 9, 'footprint_lethal_count': 0, 'local_cost_max_radius': 63},
                {'candidate_index': 3, 'hard_safety_pass': False, 'safety_floor_ok': False, 'candidate_reject_reasons': ['footprint_lethal_not_increased', 'front_wedge_lethal_not_increased', 'safety_floor_ok'], 'min_clearance_m': 0.15, 'front_wedge_lethal_count': 59, 'footprint_lethal_count': 37, 'local_cost_max_radius': 99},
            ],
        },
    }
    failure = dict(dispatch)
    failure.update({'event': 'failure', 'result_status': 'failure'})
    if goal1:
        dispatch.update(goal1)
        failure.update(goal1)
    write_jsonl(ad / f'{run_id}_goal_events.jsonl', [dispatch, failure])
    write_jsonl(ad / f'{run_id}_nav2_feedback.jsonl', [
        {'goal_sequence': 1, 'elapsed_sec': 10.0, 'distance_remaining': 0.93, 'number_of_recoveries': 0},
        {'goal_sequence': 1, 'elapsed_sec': 60.0, 'distance_remaining': 0.36, 'number_of_recoveries': 5},
        {'goal_sequence': 1, 'elapsed_sec': 170.0, 'distance_remaining': 0.347, 'number_of_recoveries': 15},
    ])
    write_jsonl(ad / f'{run_id}_local_costmap_samples.jsonl', [
        {'goal_sequence': 1, 'elapsed_sec': 160.0, 'front_wedge_lethal_count': 109, 'robot_footprint_lethal_count': 39, 'target_footprint_lethal_count': 0, 'front_wedge_clearance_min_m': 0.1, 'robot_in_local_costmap_bounds': True},
    ])
    (ad / f'{run_id}_raw_capture.json').write_text(json.dumps({'scan': {'available': True}, 'map': {'available': True}, 'local_costmap': {'available': True}, 'odom': {'available': True}, 'tf': {'available': True}}), encoding='utf-8')
    ag1 = {
        'terminal_outcome': 'failure',
        'recoveries': {'max': 15},
        'nav2_feedback': {'distance_remaining_last': 0.347, 'distance_remaining_first_nonzero': 0.933, 'distance_remaining_progress_delta': 0.586},
        'local_cost_risk': {'front_wedge_lethal_count_max': 109, 'robot_footprint_lethal_count_max': 39, 'target_footprint_lethal_count_max': 0, 'front_wedge_clearance_min_m': 0.1, 'terminal_local_cost_blocked': True},
        'refinement_applied': False,
        'hard_safety_pass_candidate_count': 0,
        'staging_applied': False,
    }
    if analysis_goal1:
        ag1.update(analysis_goal1)
    (ad / f'{run_id}_analysis.json').write_text(json.dumps({'classification': 'INGRESS_GUIDED_REFINEMENT_CHAIN_RECOVERY_DOMINANT', 'per_goal': {'1': ag1}, 'observed_goal_sequences': [1, 2], 'evidence_gaps': []}), encoding='utf-8')
    return ad


def test_phase98_classification_tokens_are_exact():
    module = load_analyzer()
    assert set(module.ALLOWED_CLASSIFICATIONS) == {
        'GOAL1_STAGING_REJECT_ROOT_CAUSE',
        'GOAL1_SINGLE_STEP_REFINEMENT_NO_HARD_SAFE_CANDIDATE',
        'GOAL1_NAV2_EXECUTION_RECOVERY_DOMINANT',
        'GOAL1_LOCAL_COST_FOOTPRINT_FRONT_WEDGE_BLOCKED',
        'GOAL1_STAGING_TRIGGER_OR_CHECK_AMBIGUOUS',
        'GOAL1_FAILURE_DIAGNOSIS_INSUFFICIENT_EVIDENCE',
    }


def test_goal1_staging_reject_root_cause_extracts_required_fields(tmp_path: Path):
    module = load_analyzer()
    ad = make_artifacts(tmp_path)
    result = module.analyze_phase98(ad)
    assert result['classification'] == 'GOAL1_STAGING_REJECT_ROOT_CAUSE'
    g1 = result['goal1']
    assert g1['refinement_reject_reason'] == 'lethal_cost_regression'
    assert g1['hard_safety_pass_candidate_count'] == 0
    assert g1['staging_reject_reason'] == 'missing_two_side_wall_evidence'
    assert g1['staging_trigger_bundle']['single_step_forward_search_no_hard_safety_pass'] is True
    assert g1['staging_executability_check']['two_side_wall_evidence'] is False
    assert g1['terminal_outcome'] == 'failure'
    assert g1['recovery_timeline']['max_recoveries'] == 15
    assert g1['local_cost_terminal_metrics']['terminal_local_cost_blocked'] is True
    assert result['answers']['why_phase88_refinement_not_applied']
    assert result['answers']['why_staging_enabled_but_not_applied']


def test_single_step_no_hard_safe_candidate_path_without_staging_reject(tmp_path: Path):
    module = load_analyzer()
    ad = make_artifacts(tmp_path, goal1={'staging_reject_reason': None, 'staging_executability_check': {'checked': False}})
    result = module.analyze_phase98(ad)
    assert result['classification'] == 'GOAL1_SINGLE_STEP_REFINEMENT_NO_HARD_SAFE_CANDIDATE'
    assert result['goal1']['single_step_candidate_failure_landscape']['reject_reason_counts']['safety_floor_ok'] == 3


def test_insufficient_evidence_when_goal1_artifacts_missing(tmp_path: Path):
    module = load_analyzer()
    ad = tmp_path / 'empty'
    ad.mkdir()
    result = module.analyze_phase98(ad)
    assert result['classification'] == 'GOAL1_FAILURE_DIAGNOSIS_INSUFFICIENT_EVIDENCE'
    assert result['evidence_gaps']


def test_guardrails_and_report_tokens_present(tmp_path: Path):
    module = load_analyzer()
    ad = make_artifacts(tmp_path)
    result = module.analyze_phase98(ad)
    assert result['guardrails'] == {
        'algorithm_changed': False,
        'phase88_92_logic_changed': False,
        'branch_scoring_or_order_changed': False,
        'nav2_config_changed': False,
        'autonomous_exploration_success_claimed': False,
        'exit_success_claimed': False,
        'phase99_entered': False,
    }
    summary = module.render_minimal_summary(result)
    assert 'GOAL1_STAGING_REJECT_ROOT_CAUSE' in summary
    assert 'missing_two_side_wall_evidence' in summary
