from __future__ import annotations

import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
RUN_ID = 'phase94_applied_single_step_refinement_execution_outcome_diagnosis'
ANALYZER = ROOT / 'tools' / 'analyze_phase94_applied_single_step_refinement_execution_outcome.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase94_applied_single_step_refinement_execution_outcome_diagnosis_report.md'
PHASE93_DIR = ROOT / 'log' / 'phase93_two_step_staging_bounded_goal2_validation'
PHASE89_DIR = ROOT / 'log' / 'phase89_safety_first_refinement_bounded_goal2_validation'
PHASE85_DIR = ROOT / 'log' / 'phase85_goal2_corridor_aligned_refinement_bounded_validation'

ALLOWED_PHASE94_CLASSIFICATIONS = {
    'APPLIED_REFINEMENT_EXECUTION_IMPROVED',
    'APPLIED_REFINEMENT_STILL_RECOVERY_DOMINANT',
    'APPLIED_REFINEMENT_HELD_BEFORE_TERMINAL_OUTCOME',
    'APPLIED_REFINEMENT_LOCAL_COST_RISK_PERSISTS',
    'APPLIED_REFINEMENT_DIAGNOSIS_INSUFFICIENT_EVIDENCE',
}

REQUIRED_EXTRACTED_FIELDS = {
    'selected_refined_target',
    'selected_candidate_index',
    'selected_candidate_target',
    'selected_candidate_yaw',
    'selection_priority_trace',
    'hard_safety_pass_candidate_count',
    'rejected_candidate_summaries',
}


def _load_analyzer():
    spec = importlib.util.spec_from_file_location('phase94_analyzer', ANALYZER)
    assert spec and spec.loader, f'missing analyzer: {ANALYZER}'
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _write_jsonl(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('\n'.join(json.dumps({'state': row}, sort_keys=True) for row in rows) + '\n', encoding='utf-8')


def _dispatch_event(*, refined: bool = True) -> dict:
    centerline = {
        'enabled': True,
        'multi_candidate_forward_search': True,
        'candidate_count': 63,
        'hard_safety_pass_candidate_count': 6 if refined else 0,
        'selected_candidate_index': 10 if refined else None,
        'selected_candidate_target': [2.0854, 1.0229] if refined else None,
        'selected_candidate_yaw': 1.5653 if refined else None,
        'refined_target': [2.0854, 1.0229] if refined else [2.07, 1.02],
        'original_target': [2.07, 1.02],
        'centerline_projected_target': [2.0854, 1.0229] if refined else None,
        'refinement_applied': refined,
        'refinement_reject_reason': None if refined else 'lethal_cost_regression',
        'selection_priority_trace': [
            'hard safety pass',
            'no footprint/front-wedge lethal regression',
            'safety_floor_ok',
            'forward_progress_ok',
            'clearance better',
            'balance error smaller',
        ],
        'rejected_candidate_summaries': [
            {'candidate_index': 0, 'hard_safety_pass': False, 'candidate_reject_reasons': ['safety_floor_ok']},
            {'candidate_index': 11, 'hard_safety_pass': True, 'candidate_reject_reasons': []},
        ],
        'branch_scoring_changed': False,
        'fallback_terminal_acceptance_used': False,
    }
    return {
        'event': 'dispatch',
        'goal_sequence': 1,
        'goal_kind': 'explore',
        'original_target': [2.07, 1.02],
        'target': [2.0854, 1.0229] if refined else [2.07, 1.02],
        'refined_target': [2.0854, 1.0229] if refined else [2.07, 1.02],
        'centerline_target_refinement': centerline,
        'branch_scoring_changed': False,
        'fallback_terminal_acceptance_used': False,
    }


def _make_case(tmp_path: Path, case: str) -> Path:
    phase93 = tmp_path / case / 'phase93'
    phase89 = tmp_path / case / 'phase89'
    phase85 = tmp_path / case / 'phase85'
    phase93.mkdir(parents=True)
    phase89.mkdir(parents=True)
    phase85.mkdir(parents=True)

    if case == 'insufficient':
        _write_jsonl(phase93 / 'phase93_goal_events.jsonl', [{'event': 'dispatch', 'goal_sequence': 1}])
        _write_jsonl(phase93 / 'phase93_nav2_feedback.jsonl', [])
        _write_jsonl(phase93 / 'phase93_local_costmap_samples.jsonl', [])
        (phase93 / 'phase93_raw_capture.json').write_text('{}', encoding='utf-8')
        return phase93

    terminal_event = None
    if case == 'improved':
        terminal_event = {'event': 'success', 'goal_sequence': 1, 'result_reason': 'goal_reached'}
        nav_rows = [
            {'goal_sequence': 1, 'distance_remaining': 1.0, 'number_of_recoveries': 0, 'elapsed_sec': 1.0},
            {'goal_sequence': 1, 'distance_remaining': 0.03, 'number_of_recoveries': 0, 'elapsed_sec': 10.0},
        ]
        cost_rows = [
            {'goal_sequence': 1, 'front_wedge_cost': {'lethal_count': 0}, 'robot_footprint_cost': {'lethal_count': 0}},
        ]
    elif case == 'recovery_dominant':
        terminal_event = {'event': 'timeout', 'goal_sequence': 1, 'result_reason': 'goal_timeout'}
        nav_rows = [
            {'goal_sequence': 1, 'distance_remaining': 1.0, 'number_of_recoveries': 0, 'elapsed_sec': 1.0},
            {'goal_sequence': 1, 'distance_remaining': 0.33, 'number_of_recoveries': 13, 'elapsed_sec': 150.0},
        ]
        cost_rows = [
            {'goal_sequence': 1, 'front_wedge_cost': {'lethal_count': 1}, 'robot_footprint_cost': {'lethal_count': 0}},
        ]
    elif case == 'held':
        nav_rows = [
            {'goal_sequence': 1, 'distance_remaining': 1.0, 'number_of_recoveries': 0, 'elapsed_sec': 1.0},
            {'goal_sequence': 1, 'distance_remaining': 0.33, 'number_of_recoveries': 13, 'elapsed_sec': 150.0},
        ]
        cost_rows = [
            {'goal_sequence': 1, 'front_wedge_cost': {'lethal_count': 1}, 'robot_footprint_cost': {'lethal_count': 0}},
        ]
    elif case == 'local_cost':
        terminal_event = {'event': 'timeout', 'goal_sequence': 1, 'result_reason': 'goal_timeout'}
        nav_rows = [
            {'goal_sequence': 1, 'distance_remaining': 1.0, 'number_of_recoveries': 0, 'elapsed_sec': 1.0},
            {'goal_sequence': 1, 'distance_remaining': 0.34, 'number_of_recoveries': 2, 'elapsed_sec': 55.0},
        ]
        cost_rows = [
            {'goal_sequence': 1, 'front_wedge_cost': {'lethal_count': 129}, 'robot_footprint_cost': {'lethal_count': 52}},
        ]
    else:
        raise AssertionError(case)

    events = [_dispatch_event(refined=True)]
    if terminal_event:
        events.append(terminal_event)
    _write_jsonl(phase93 / 'phase93_goal_events.jsonl', events)
    _write_jsonl(phase93 / 'phase93_nav2_feedback.jsonl', nav_rows)
    _write_jsonl(phase93 / 'phase93_local_costmap_samples.jsonl', cost_rows)
    (phase93 / 'phase93_raw_capture.json').write_text(json.dumps({'scan': {}, 'local_costmap': {}, 'odom': {}, 'tf': {'map->base_link': {'available': True}}, 'footprint': {}}), encoding='utf-8')
    (phase93 / 'phase93_analysis.json').write_text(json.dumps({'classification': 'TWO_STEP_STAGING_NOT_TRIGGERED'}), encoding='utf-8')

    # Baseline comparison artifacts intentionally minimal: the analyzer must accept
    # prior phase analysis JSON summaries and compare candidate count/refinement,
    # recoveries, distance remaining, and local-cost maxima without needing a rerun.
    for phase_dir, classification, hard_safe, applied, dist_last, rec_max, front, footprint in [
        (phase89, 'PHASE88_REFINEMENT_STILL_REJECTED', 0, False, 0.3378, 4, 142, 52),
        (phase85, 'REFINEMENT_REJECTED_OR_NOT_TRIGGERED', 0, False, 0.3802, 4, 148, 54),
    ]:
        (phase_dir / 'analysis.json').write_text(json.dumps({
            'classification': classification,
            'goal2_refinement_summary': {
                'refinement_applied': applied,
                'hard_safety_pass_candidate_count': hard_safe,
                'selected_candidate_target': None,
                'nav2_target_shift_m': 0.0,
            },
            'nav2_feedback_summary': {
                'distance_remaining_last': dist_last,
                'recoveries_max': rec_max,
            },
            'local_costmap_summary': {
                'front_wedge_lethal_count_max': front,
                'robot_footprint_lethal_count_max': footprint,
            },
            'goal2_outcome': {'timed_out': True, 'succeeded': False, 'observed': True},
        }), encoding='utf-8')
    return phase93


def test_phase94_analyzer_contract_and_field_extraction(tmp_path):
    analyzer = _load_analyzer()
    assert set(analyzer.ALLOWED_CLASSIFICATIONS) == ALLOWED_PHASE94_CLASSIFICATIONS
    assert REQUIRED_EXTRACTED_FIELDS.issubset(set(analyzer.REQUIRED_EXTRACTED_REFINEMENT_FIELDS))
    result = analyzer.analyze_artifact_dir(
        _make_case(tmp_path, 'held'),
        phase89_dir=tmp_path / 'held' / 'phase89',
        phase85_dir=tmp_path / 'held' / 'phase85',
    )
    extracted = result['selected_refinement']
    assert extracted['refinement_applied'] is True
    assert extracted['selected_candidate_index'] == 10
    assert extracted['selected_candidate_target'] == [2.0854, 1.0229]
    assert extracted['selected_candidate_yaw'] == 1.5653
    assert extracted['hard_safety_pass_candidate_count'] == 6
    assert extracted['rejected_candidate_summary_count'] == 2
    assert result['guardrails']['no_branch_scoring_changed'] is True
    assert result['guardrails']['no_fallback_terminal_acceptance_changed'] is True


def test_phase94_classifies_terminal_success_with_low_recovery_as_improved(tmp_path):
    analyzer = _load_analyzer()
    result = analyzer.analyze_artifact_dir(
        _make_case(tmp_path, 'improved'),
        phase89_dir=tmp_path / 'improved' / 'phase89',
        phase85_dir=tmp_path / 'improved' / 'phase85',
    )
    assert result['classification'] == 'APPLIED_REFINEMENT_EXECUTION_IMPROVED'
    assert result['execution_outcome']['terminal_succeeded'] is True
    assert result['execution_outcome']['recoveries_max'] == 0
    assert result['success_claim_guardrails']['autonomous_exploration_success_claimed'] is False
    assert result['success_claim_guardrails']['exit_success_claimed'] is False


def test_phase94_classifies_high_recovery_without_terminal_as_held_before_terminal(tmp_path):
    analyzer = _load_analyzer()
    result = analyzer.analyze_artifact_dir(
        _make_case(tmp_path, 'held'),
        phase89_dir=tmp_path / 'held' / 'phase89',
        phase85_dir=tmp_path / 'held' / 'phase85',
    )
    assert result['classification'] == 'APPLIED_REFINEMENT_HELD_BEFORE_TERMINAL_OUTCOME'
    assert result['execution_outcome']['terminal_observed'] is False
    assert result['execution_outcome']['held_before_terminal_outcome'] is True
    assert result['execution_outcome']['recoveries_max'] == 13
    assert 'missing_terminal_outcome_for_applied_refinement' in result['evidence_gaps']


def test_phase94_classifies_terminal_timeout_with_recovery_dominant(tmp_path):
    analyzer = _load_analyzer()
    result = analyzer.analyze_artifact_dir(
        _make_case(tmp_path, 'recovery_dominant'),
        phase89_dir=tmp_path / 'recovery_dominant' / 'phase89',
        phase85_dir=tmp_path / 'recovery_dominant' / 'phase85',
    )
    assert result['classification'] == 'APPLIED_REFINEMENT_STILL_RECOVERY_DOMINANT'
    assert result['execution_outcome']['terminal_timed_out'] is True
    assert result['execution_outcome']['recoveries_max'] >= 5


def test_phase94_classifies_local_cost_risk_when_lethal_cost_persists(tmp_path):
    analyzer = _load_analyzer()
    result = analyzer.analyze_artifact_dir(
        _make_case(tmp_path, 'local_cost'),
        phase89_dir=tmp_path / 'local_cost' / 'phase89',
        phase85_dir=tmp_path / 'local_cost' / 'phase85',
    )
    assert result['classification'] == 'APPLIED_REFINEMENT_LOCAL_COST_RISK_PERSISTS'
    assert result['local_cost_risk']['local_cost_risk_present'] is True
    assert result['local_cost_risk']['front_wedge_lethal_count_max'] == 129
    assert result['local_cost_risk']['robot_footprint_lethal_count_max'] == 52


def test_phase94_insufficient_when_applied_refinement_fields_missing(tmp_path):
    analyzer = _load_analyzer()
    result = analyzer.analyze_artifact_dir(_make_case(tmp_path, 'insufficient'))
    assert result['classification'] == 'APPLIED_REFINEMENT_DIAGNOSIS_INSUFFICIENT_EVIDENCE'
    assert any(gap.startswith('missing_refinement_field:') for gap in result['evidence_gaps'])
    assert result['selected_refinement']['refinement_applied'] is not True


def test_phase94_real_artifacts_and_report_contract_after_completion():
    assert PHASE93_DIR.exists(), 'Phase93 artifacts must remain available for Phase94 diagnosis'
    for path in [
        PHASE93_DIR / 'phase93_two_step_staging_bounded_goal2_validation_goal_events.jsonl',
        PHASE93_DIR / 'phase93_two_step_staging_bounded_goal2_validation_nav2_feedback.jsonl',
        PHASE93_DIR / 'phase93_two_step_staging_bounded_goal2_validation_local_costmap_samples.jsonl',
        PHASE93_DIR / 'phase93_two_step_staging_bounded_goal2_validation_raw_capture.json',
        PHASE93_DIR / 'phase93_two_step_staging_bounded_goal2_validation_analysis.json',
    ]:
        assert path.exists(), f'missing required Phase93 input: {path}'
    assert ANALYZER.exists(), 'Phase94 analyzer must be added before final validation'
    assert REPORT.exists(), 'Phase94 report must be written before final validation'
    report = REPORT.read_text(encoding='utf-8')
    for token in [
        'Applied single-step refinement execution outcome diagnosis',
        'APPLIED_REFINEMENT_HELD_BEFORE_TERMINAL_OUTCOME',
        'hard_safety_pass_candidate_count=6',
        'recoveries_max=13',
        'No terminal outcome was captured',
        'No maze_explorer strategy changed',
        'No Nav2/MPPI/controller tuning',
        'No autonomous exploration success claimed',
        'No exit success claimed',
        'Phase95 not entered',
    ]:
        assert token in report
