from __future__ import annotations

import importlib.util
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
ANALYZER = ROOT / 'tools' / 'analyze_phase86_lethal_cost_regression_reject_root_cause.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase86_lethal_cost_regression_reject_root_cause_report.md'
ARTIFACT_DIR = ROOT / 'log' / 'phase86_lethal_cost_regression_reject_root_cause'
OUTPUT_JSON = ARTIFACT_DIR / 'phase86_lethal_cost_regression_reject_root_cause.json'
MIN_SUMMARY = ARTIFACT_DIR / 'phase86_lethal_cost_regression_reject_root_cause_minimal_summary.md'

ALLOWED_CLASSIFICATIONS = {
    'REFINED_TARGET_IN_HIGH_COST_BAND',
    'CENTERLINE_PROJECTION_DISTANCE_OR_POSE_TOO_AGGRESSIVE',
    'FORWARD_EXECUTABILITY_CHECK_TOO_CONSERVATIVE',
    'BASELINE_ALREADY_RISKY_COMPARISON_AMBIGUOUS',
    'MULTI_CANDIDATE_FORWARD_SEARCH_NEEDED',
    'INSUFFICIENT_REJECT_DIAGNOSTIC_EVIDENCE',
}

REJECT_SUBITEMS = {
    'safety_floor_ok',
    'footprint_lethal_not_increased',
    'front_wedge_lethal_not_increased',
    'forward_progress_not_lowered',
    'target_has_clearance',
    'occupancy_free',
    'same_corridor',
    'two_side_wall_evidence',
}


def load_analyzer():
    assert ANALYZER.exists(), f'missing Phase86 analyzer: {ANALYZER}'
    spec = importlib.util.spec_from_file_location('phase86_analyzer', ANALYZER)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_jsonl(path: Path, payloads: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('\n'.join(json.dumps(p, sort_keys=True) for p in payloads) + '\n', encoding='utf-8')


def base_forward_check(**overrides):
    data = {
        'checked': True,
        'passed': False,
        'reason': 'lethal_cost_regression',
        'safety_floor_ok': False,
        'footprint_lethal_not_increased': False,
        'front_wedge_lethal_not_increased': False,
        'forward_progress_not_lowered': True,
        'target_has_clearance': True,
        'occupancy_free': True,
        'same_corridor': True,
        'two_side_wall_evidence': True,
        'local_cost_sample_count': 112,
        'front_wedge_sample_count': 184,
    }
    data.update(overrides)
    return data


def candidate(**overrides):
    data = {
        'target': [2.1, 1.0],
        'lateral_offset_m': 0.0,
        'forward_offset_m': 0.0,
        'balance_error_m': 0.05,
        'same_corridor': True,
        'two_side_wall_evidence': True,
        'occupancy_free': True,
        'target_has_clearance': True,
        'safety_floor_ok': True,
        'footprint_lethal_not_increased': True,
        'front_wedge_lethal_not_increased': True,
        'forward_progress_not_lowered': True,
        'forward_progress_not_obviously_lowered': True,
        'local_cost_max_radius': 54,
        'front_wedge_cost_max': 63,
        'footprint_lethal_count': 0,
        'front_wedge_lethal_count': 0,
        'min_clearance_m': 0.46,
        'eligible': False,
        'balance_first_eligible': False,
    }
    data.update(overrides)
    return data


def make_artifact_dir(
    tmp_path: Path,
    *,
    candidates: list[dict] | None = None,
    forward_check: dict | None = None,
    original_metrics: dict | None = None,
    include_analysis: bool = True,
    include_raw: bool = True,
    include_local_samples: bool = True,
) -> Path:
    artifact_dir = tmp_path / 'phase85_like_artifacts'
    artifact_dir.mkdir(parents=True, exist_ok=True)
    if include_analysis:
        original = original_metrics or {
            'target': [2.084, 1.023],
            'same_corridor': True,
            'two_side_wall_evidence': True,
            'occupancy_free': True,
            'target_has_clearance': True,
            'local_cost_max_radius': 54,
            'front_wedge_cost_max': 63,
            'footprint_lethal_count': 0,
            'front_wedge_lethal_count': 0,
            'min_clearance_m': 0.46,
            'balance_error_m': 0.05,
            'forward_progress_m': 1.0,
        }
        cands = candidates if candidates is not None else [candidate()]
        analysis = {
            'classification': 'REFINEMENT_REJECTED_OR_NOT_TRIGGERED',
            'validation_goal_sequence': 1,
            'used_target_match_fallback': True,
            'goal2_dispatch_context': {
                'original_target': original['target'],
                'centerline_projected_target': None,
                'target': original['target'],
                'corridor_heading_yaw': 1.56,
                'refinement_applied': False,
                'refinement_reject_reason': 'lethal_cost_regression',
                'forward_executability_check': forward_check or base_forward_check(),
                'branch_scoring_changed': False,
                'fallback_terminal_acceptance_used': False,
                'centerline_target_refinement': {
                    'refinement_applied': False,
                    'refinement_reject_reason': 'lethal_cost_regression',
                    'reason': 'lethal_cost_regression',
                    'candidate_count': len(cands),
                    'two_side_wall_candidate_count': sum(1 for c in cands if c.get('two_side_wall_evidence') is True),
                    'strict_eligible_candidate_count': sum(1 for c in cands if c.get('eligible') is True),
                    'eligible_candidate_count': sum(1 for c in cands if c.get('eligible') is True),
                    'balance_first_eligible_candidate_count': sum(1 for c in cands if c.get('balance_first_eligible') is True),
                    'original_target': original['target'],
                    'refined_target': original['target'],
                    'centerline_projected_target': None,
                    'corridor_heading_yaw': 1.56,
                    'original_metrics': original,
                    'forward_executability_check': forward_check or base_forward_check(),
                    'candidates': cands,
                    'branch_scoring_changed': False,
                    'fallback_terminal_acceptance_used': False,
                },
            },
            'goal2_refinement_summary': {
                'refinement_applied': False,
                'refinement_reject_reason': 'lethal_cost_regression',
            },
            'guardrails': {
                'no_strategy_or_config_tuning': True,
                'branch_scoring_changed_false': True,
                'fallback_terminal_acceptance_used_false': True,
                'no_nav2_mppi_controller_tuning': True,
                'no_inflation_robot_radius_clearance_map_threshold_tuning': True,
            },
            'complete_autonomous_success_claimed': False,
            'exit_success_claimed': False,
        }
        (artifact_dir / 'phase85_goal2_corridor_aligned_refinement_bounded_validation_analysis.json').write_text(
            json.dumps(analysis, indent=2, sort_keys=True), encoding='utf-8'
        )
    write_jsonl(
        artifact_dir / 'phase85_goal2_corridor_aligned_refinement_bounded_validation_goal_events.jsonl',
        [{'state': {'event': 'dispatch', 'goal_sequence': 1}}],
    )
    if include_raw:
        (artifact_dir / 'phase85_goal2_corridor_aligned_refinement_bounded_validation_raw_capture.json').write_text(
            json.dumps({'scan': {'ranges': [2.0]}, 'local_costmap': {'data': [0]}, 'footprint': {'points': []}, 'odom': {}, 'tf': {'map->base_link': {}}}),
            encoding='utf-8',
        )
    if include_local_samples:
        write_jsonl(
            artifact_dir / 'phase85_goal2_corridor_aligned_refinement_bounded_validation_local_costmap_samples.jsonl',
            [{
                'goal_sequence': 1,
                'front_wedge_cost': {'lethal_count': 0, 'max': 63},
                'robot_footprint_cost': {'lethal_count': 0, 'max': 46},
                'target_footprint_cost': {'lethal_count': 0, 'max': 54},
            }],
        )
    return artifact_dir


def test_phase86_classifies_centerline_projection_pose_too_aggressive_from_candidate_metrics(tmp_path):
    module = load_analyzer()
    cands = [
        candidate(balance_error_m=0.05, forward_offset_m=0.0, min_clearance_m=0.46, local_cost_max_radius=54),
        candidate(
            balance_error_m=0.0,
            forward_offset_m=0.1,
            target_has_clearance=False,
            safety_floor_ok=False,
            min_clearance_m=0.05,
            local_cost_max_radius=84,
            front_wedge_cost_max=84,
        ),
        candidate(
            balance_error_m=0.0,
            forward_offset_m=0.2,
            target_has_clearance=False,
            safety_floor_ok=False,
            min_clearance_m=0.05,
            local_cost_max_radius=84,
            front_wedge_cost_max=73,
        ),
    ]
    artifact_dir = make_artifact_dir(tmp_path, candidates=cands)

    result = module.analyze_phase85_artifacts(artifact_dir)

    assert result['classification'] == 'CENTERLINE_PROJECTION_DISTANCE_OR_POSE_TOO_AGGRESSIVE'
    assert result['lethal_reject_context']['refinement_reject_reason'] == 'lethal_cost_regression'
    assert result['candidate_aggregate']['candidate_count'] == 3
    assert result['candidate_aggregate']['best_balance_candidate_count'] == 2
    assert result['candidate_aggregate']['best_balance_candidates_all_clearance_failed'] is True
    assert result['complete_autonomous_success_claimed'] is False
    assert result['exit_success_claimed'] is False


def test_phase86_reports_requested_reject_subitems(tmp_path):
    module = load_analyzer()
    artifact_dir = make_artifact_dir(
        tmp_path,
        candidates=[
            candidate(safety_floor_ok=False, footprint_lethal_not_increased=False, front_wedge_lethal_not_increased=False),
            candidate(target_has_clearance=False, occupancy_free=True, same_corridor=True, two_side_wall_evidence=True),
        ],
        forward_check=base_forward_check(forward_progress_not_lowered=False),
    )

    result = module.analyze_phase85_artifacts(artifact_dir)

    assert set(result['reject_subitems']) == REJECT_SUBITEMS
    assert result['reject_subitems']['safety_floor_ok']['candidate_false_count'] == 1
    assert result['reject_subitems']['footprint_lethal_not_increased']['candidate_false_count'] == 1
    assert result['reject_subitems']['front_wedge_lethal_not_increased']['candidate_false_count'] == 1
    assert result['reject_subitems']['forward_progress_not_lowered']['forward_check_value'] is False
    assert result['reject_subitems']['target_has_clearance']['candidate_false_count'] == 1
    assert result['reject_subitems']['occupancy_free']['candidate_true_count'] == 2
    assert result['reject_subitems']['same_corridor']['candidate_true_count'] == 2
    assert result['reject_subitems']['two_side_wall_evidence']['candidate_true_count'] == 2


def test_phase86_classifies_forward_executability_check_too_conservative_when_hard_safe_candidate_exists(tmp_path):
    module = load_analyzer()
    artifact_dir = make_artifact_dir(
        tmp_path,
        candidates=[candidate(balance_error_m=0.0, eligible=False, balance_first_eligible=False)],
        forward_check=base_forward_check(
            safety_floor_ok=False,
            footprint_lethal_not_increased=False,
            front_wedge_lethal_not_increased=False,
            forward_progress_not_lowered=False,
        ),
    )

    result = module.analyze_phase85_artifacts(artifact_dir)

    assert result['classification'] == 'FORWARD_EXECUTABILITY_CHECK_TOO_CONSERVATIVE'
    assert result['candidate_aggregate']['hard_safety_pass_candidate_count'] == 1
    assert 'hard_safe_candidate_exists_but_reject_reason_lethal_cost_regression' in result['classification_reasons']


def test_phase86_missing_required_artifacts_returns_insufficient(tmp_path):
    module = load_analyzer()
    artifact_dir = make_artifact_dir(tmp_path, include_analysis=False, include_raw=False, include_local_samples=False)

    result = module.analyze_phase85_artifacts(artifact_dir)

    assert result['classification'] == 'INSUFFICIENT_REJECT_DIAGNOSTIC_EVIDENCE'
    assert 'missing_phase85_analysis_json' in result['evidence_gaps']
    assert 'missing_phase85_raw_capture_json' in result['evidence_gaps']
    assert 'missing_phase85_local_costmap_samples_jsonl' in result['evidence_gaps']


def test_phase86_static_outputs_and_guardrails_after_real_artifact_analysis():
    assert OUTPUT_JSON.exists(), 'Phase86 JSON artifact is missing'
    assert MIN_SUMMARY.exists(), 'Phase86 minimal summary is missing'
    assert REPORT.exists(), 'Phase86 report is missing'

    data = json.loads(OUTPUT_JSON.read_text(encoding='utf-8'))
    assert data['classification'] in ALLOWED_CLASSIFICATIONS
    assert set(data['allowed_classifications']) == ALLOWED_CLASSIFICATIONS
    assert set(data['reject_subitems']) == REJECT_SUBITEMS
    assert data['source_phase85_classification'] == 'REFINEMENT_REJECTED_OR_NOT_TRIGGERED'
    assert data['lethal_reject_context']['refinement_reject_reason'] == 'lethal_cost_regression'
    assert data['guardrails']['no_strategy_or_config_tuning'] is True
    assert data['guardrails']['no_nav2_mppi_controller_tuning'] is True
    assert data['guardrails']['no_inflation_robot_radius_clearance_map_threshold_tuning'] is True
    assert data['complete_autonomous_success_claimed'] is False
    assert data['exit_success_claimed'] is False
    assert data['phase87_entered'] is False

    report = REPORT.read_text(encoding='utf-8')
    for token in [
        'Phase86',
        'lethal_cost_regression',
        'REFINED_TARGET_IN_HIGH_COST_BAND',
        'CENTERLINE_PROJECTION_DISTANCE_OR_POSE_TOO_AGGRESSIVE',
        'FORWARD_EXECUTABILITY_CHECK_TOO_CONSERVATIVE',
        'BASELINE_ALREADY_RISKY_COMPARISON_AMBIGUOUS',
        'MULTI_CANDIDATE_FORWARD_SEARCH_NEEDED',
        'INSUFFICIENT_REJECT_DIAGNOSTIC_EVIDENCE',
        'No maze_explorer strategy changed',
        'No branch scoring changed',
        'No centerline gate changed',
        'No directional readiness override changed',
        'No fallback/terminal acceptance changed',
        'No Nav2/MPPI/controller tuning',
        'No inflation/robot_radius/clearance_radius_m/map threshold tuning',
        'No autonomous exploration success claimed',
        'No exit success claimed',
        'Phase87 not entered',
    ]:
        assert token in report
