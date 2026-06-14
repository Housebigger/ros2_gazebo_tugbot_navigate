from __future__ import annotations

import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
ANALYZER = ROOT / 'tools' / 'analyze_phase95_applied_refinement_terminal_outcome_bounded_validation.py'
RECORDER = ROOT / 'tools' / 'record_phase95_terminal_evidence.py'
RUNBOOK = ROOT / 'tools' / 'run_phase95_applied_refinement_terminal_outcome_bounded_validation.sh'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase95_applied_refinement_terminal_outcome_bounded_validation_report.md'
RUNBOOK_DOC = ROOT / 'doc' / 'doc_report' / 'phase95_applied_refinement_terminal_outcome_bounded_validation_runbook.md'

ALLOWED_PHASE95_CLASSIFICATIONS = {
    'APPLIED_REFINEMENT_TERMINAL_SUCCEEDED',
    'APPLIED_REFINEMENT_TERMINAL_TIMEOUT_RECOVERY_DOMINANT',
    'APPLIED_REFINEMENT_TERMINAL_LOCAL_COST_BLOCKED',
    'APPLIED_REFINEMENT_TERMINAL_CANCEL_OR_FAILURE',
    'APPLIED_REFINEMENT_TERMINAL_VALIDATION_INSUFFICIENT_EVIDENCE',
}

REQUIRED_EXTRACTED_FIELDS = {
    'selected_candidate_target',
    'selected_candidate_yaw',
    'hard_safety_pass_candidate_count',
    'selection_priority_trace',
    'terminal_event',
    'terminal_reason',
    'terminal_pose',
    'terminal_local_cost_metrics',
}


def _load_analyzer():
    spec = importlib.util.spec_from_file_location('phase95_analyzer', ANALYZER)
    assert spec and spec.loader, f'missing analyzer: {ANALYZER}'
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _write_jsonl(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('\n'.join(json.dumps({'state': row}, sort_keys=True) for row in rows) + '\n', encoding='utf-8')


def _dispatch_event(*, refined: bool = True, goal_sequence: int = 1) -> dict:
    refinement = {
        'enabled': True,
        'multi_candidate_forward_search': True,
        'candidate_count': 63,
        'hard_safety_pass_candidate_count': 6 if refined else 0,
        'selected_candidate_index': 10 if refined else None,
        'selected_candidate_target': [2.0854, 1.0229] if refined else None,
        'selected_candidate_yaw': 1.5653 if refined else None,
        'refined_target': [2.0854, 1.0229] if refined else [2.07, 1.02],
        'original_target': [2.0854, 1.0229],
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
            {'candidate_index': 10, 'hard_safety_pass': True, 'candidate_reject_reasons': []},
        ],
        'branch_scoring_changed': False,
        'fallback_terminal_acceptance_used': False,
    }
    return {
        'event': 'dispatch',
        'goal_sequence': goal_sequence,
        'goal_kind': 'explore',
        'target': [2.0854, 1.0229] if refined else [2.07, 1.02],
        'original_target': [2.0854, 1.0229],
        'refined_target': [2.0854, 1.0229] if refined else [2.07, 1.02],
        'centerline_target_refinement': refinement,
        'branch_scoring_changed': False,
        'fallback_terminal_acceptance_used': False,
    }


def _write_prior_analysis(base: Path) -> None:
    phase_values = {
        'phase85': {
            'classification': 'REFINEMENT_REJECTED_OR_NOT_TRIGGERED',
            'refinement_applied': False,
            'hard_safety_pass_candidate_count': 0,
            'recoveries_max': 4,
            'distance_remaining_last': 0.3802,
            'terminal_outcome': 'timeout',
            'front_wedge_lethal_count_max': 148,
            'robot_footprint_lethal_count_max': 54,
        },
        'phase89': {
            'classification': 'PHASE88_REFINEMENT_STILL_REJECTED',
            'refinement_applied': False,
            'hard_safety_pass_candidate_count': 0,
            'recoveries_max': 4,
            'distance_remaining_last': 0.3378,
            'terminal_outcome': 'timeout',
            'front_wedge_lethal_count_max': 142,
            'robot_footprint_lethal_count_max': 52,
        },
        'phase94': {
            'classification': 'APPLIED_REFINEMENT_HELD_BEFORE_TERMINAL_OUTCOME',
            'refinement_applied': True,
            'hard_safety_pass_candidate_count': 6,
            'recoveries_max': 13,
            'distance_remaining_last': 0.3291,
            'terminal_outcome': 'held_before_terminal',
            'front_wedge_lethal_count_max': 129,
            'robot_footprint_lethal_count_max': 52,
        },
    }
    for name, values in phase_values.items():
        phase_dir = base / name
        phase_dir.mkdir(parents=True, exist_ok=True)
        (phase_dir / 'analysis.json').write_text(json.dumps(values, sort_keys=True), encoding='utf-8')


def _make_case(tmp_path: Path, case: str) -> Path:
    artifact_dir = tmp_path / case / 'phase95'
    artifact_dir.mkdir(parents=True)
    _write_prior_analysis(tmp_path / case)

    refined = case != 'insufficient_not_applied'
    events = [_dispatch_event(refined=refined)]
    nav_rows = [
        {'goal_sequence': 1, 'distance_remaining': 1.05, 'number_of_recoveries': 0, 'elapsed_sec': 1.0},
        {'goal_sequence': 1, 'distance_remaining': 0.33, 'number_of_recoveries': 0, 'elapsed_sec': 10.0},
    ]
    local_rows = [
        {'goal_sequence': 1, 'elapsed_sec': 2.0, 'robot_pose': [2.0, 1.0, 1.5], 'front_wedge_cost': {'lethal_count': 0}, 'robot_footprint_cost': {'lethal_count': 0}, 'target_footprint_cost': {'lethal_count': 0}},
    ]

    if case == 'success':
        events.append({'event': 'success', 'goal_sequence': 1, 'result_reason': 'goal_reached', 'robot_pose': [2.08, 1.02, 1.56]})
        nav_rows[-1]['distance_remaining'] = 0.02
    elif case == 'timeout_recovery':
        events.append({'event': 'timeout', 'goal_sequence': 1, 'result_reason': 'goal_timeout', 'robot_pose': [2.2, 1.0, 1.6]})
        nav_rows[-1]['number_of_recoveries'] = 13
    elif case == 'local_cost_blocked':
        events.append({'event': 'timeout', 'goal_sequence': 1, 'result_reason': 'goal_timeout', 'robot_pose': [2.2, 1.0, 1.6]})
        nav_rows[-1]['number_of_recoveries'] = 2
        local_rows.append({'goal_sequence': 1, 'elapsed_sec': 55.0, 'robot_pose': [2.2, 1.0, 1.6], 'front_wedge_cost': {'lethal_count': 58, 'max': 99}, 'robot_footprint_cost': {'lethal_count': 52, 'max': 99}, 'target_footprint_cost': {'lethal_count': 0, 'max': 54}, 'front_wedge_clearance_m': 0.05})
    elif case == 'cancel_or_failure':
        events.append({'event': 'failure', 'goal_sequence': 1, 'result_reason': 'nav2_failed', 'robot_pose': [2.1, 1.0, 1.6]})
    elif case == 'insufficient_no_terminal':
        pass
    elif case == 'insufficient_not_applied':
        events.append({'event': 'timeout', 'goal_sequence': 1, 'result_reason': 'goal_timeout'})
    else:
        raise AssertionError(case)

    _write_jsonl(artifact_dir / 'phase95_goal_events.jsonl', events)
    _write_jsonl(artifact_dir / 'phase95_nav2_feedback.jsonl', nav_rows)
    _write_jsonl(artifact_dir / 'phase95_local_costmap_samples.jsonl', local_rows)
    (artifact_dir / 'phase95_raw_capture.json').write_text(json.dumps({
        'robot_pose_by_frame': {'map': [2.2, 1.0, 1.6]},
        'scan': {},
        'local_costmap': {},
        'odom': {'pose': {'x': 2.2, 'y': 1.0, 'yaw': 1.6}},
        'tf': {'map->base_link': {'available': True}},
        'footprint': {},
    }), encoding='utf-8')
    return artifact_dir


def test_phase95_analyzer_contract_and_success_classification(tmp_path):
    analyzer = _load_analyzer()
    assert set(analyzer.ALLOWED_CLASSIFICATIONS) == ALLOWED_PHASE95_CLASSIFICATIONS
    assert REQUIRED_EXTRACTED_FIELDS.issubset(set(analyzer.REQUIRED_EXTRACTED_FIELDS))
    result = analyzer.analyze_artifact_dir(
        _make_case(tmp_path, 'success'),
        phase85_dir=tmp_path / 'success' / 'phase85',
        phase89_dir=tmp_path / 'success' / 'phase89',
        phase94_dir=tmp_path / 'success' / 'phase94',
    )
    assert result['classification'] == 'APPLIED_REFINEMENT_TERMINAL_SUCCEEDED'
    assert result['terminal_outcome']['terminal_succeeded'] is True
    assert result['selected_refinement']['refinement_applied'] is True
    assert result['selected_refinement']['hard_safety_pass_candidate_count'] == 6
    assert result['terminal_outcome']['terminal_pose'] == [2.08, 1.02, 1.56]
    assert result['success_claim_guardrails']['autonomous_exploration_success_claimed'] is False
    assert result['success_claim_guardrails']['exit_success_claimed'] is False


def test_phase95_classifies_timeout_recovery_dominant_when_terminal_timeout_has_high_recoveries(tmp_path):
    analyzer = _load_analyzer()
    result = analyzer.analyze_artifact_dir(
        _make_case(tmp_path, 'timeout_recovery'),
        phase85_dir=tmp_path / 'timeout_recovery' / 'phase85',
        phase89_dir=tmp_path / 'timeout_recovery' / 'phase89',
        phase94_dir=tmp_path / 'timeout_recovery' / 'phase94',
    )
    assert result['classification'] == 'APPLIED_REFINEMENT_TERMINAL_TIMEOUT_RECOVERY_DOMINANT'
    assert result['terminal_outcome']['terminal_timed_out'] is True
    assert result['nav2_feedback_summary']['recoveries_max'] == 13


def test_phase95_classifies_timeout_local_cost_blocked_with_terminal_front_wedge_and_footprint_metrics(tmp_path):
    analyzer = _load_analyzer()
    result = analyzer.analyze_artifact_dir(
        _make_case(tmp_path, 'local_cost_blocked'),
        phase85_dir=tmp_path / 'local_cost_blocked' / 'phase85',
        phase89_dir=tmp_path / 'local_cost_blocked' / 'phase89',
        phase94_dir=tmp_path / 'local_cost_blocked' / 'phase94',
    )
    assert result['classification'] == 'APPLIED_REFINEMENT_TERMINAL_LOCAL_COST_BLOCKED'
    metrics = result['terminal_local_cost_metrics']
    assert metrics['front_wedge_lethal_count_last'] == 58
    assert metrics['robot_footprint_lethal_count_last'] == 52
    assert metrics['target_footprint_lethal_count_last'] == 0
    assert metrics['terminal_local_cost_blocked'] is True


def test_phase95_classifies_terminal_cancel_or_failure(tmp_path):
    analyzer = _load_analyzer()
    result = analyzer.analyze_artifact_dir(
        _make_case(tmp_path, 'cancel_or_failure'),
        phase85_dir=tmp_path / 'cancel_or_failure' / 'phase85',
        phase89_dir=tmp_path / 'cancel_or_failure' / 'phase89',
        phase94_dir=tmp_path / 'cancel_or_failure' / 'phase94',
    )
    assert result['classification'] == 'APPLIED_REFINEMENT_TERMINAL_CANCEL_OR_FAILURE'
    assert result['terminal_outcome']['terminal_failed'] is True


def test_phase95_insufficient_without_terminal_or_without_applied_refinement(tmp_path):
    analyzer = _load_analyzer()
    no_terminal = analyzer.analyze_artifact_dir(_make_case(tmp_path, 'insufficient_no_terminal'))
    not_applied = analyzer.analyze_artifact_dir(_make_case(tmp_path, 'insufficient_not_applied'))
    assert no_terminal['classification'] == 'APPLIED_REFINEMENT_TERMINAL_VALIDATION_INSUFFICIENT_EVIDENCE'
    assert 'missing_terminal_outcome_for_applied_refinement' in no_terminal['evidence_gaps']
    assert not_applied['classification'] == 'APPLIED_REFINEMENT_TERMINAL_VALIDATION_INSUFFICIENT_EVIDENCE'
    assert 'applied_refinement_not_observed' in not_applied['evidence_gaps']


def test_phase95_prefers_applied_dispatch_when_legacy_goal2_dispatch_precedes_it(tmp_path):
    analyzer = _load_analyzer()
    artifact_dir = tmp_path / 'legacy_then_applied' / 'phase95'
    artifact_dir.mkdir(parents=True)
    _write_jsonl(artifact_dir / 'phase95_goal_events.jsonl', [
        _dispatch_event(refined=False, goal_sequence=1),
        {'event': 'failure', 'goal_sequence': 1, 'result_reason': 'blocked_nav2', 'robot_pose': [2.34, 0.82, 1.59]},
        _dispatch_event(refined=True, goal_sequence=2),
        {'event': 'success', 'goal_sequence': 2, 'result_reason': 'succeeded', 'robot_pose': [1.96, 2.02, 1.56]},
    ])
    _write_jsonl(artifact_dir / 'phase95_nav2_feedback.jsonl', [
        {'goal_sequence': 1, 'distance_remaining': 0.33, 'number_of_recoveries': 15, 'elapsed_sec': 170.0},
        {'goal_sequence': 2, 'distance_remaining': 0.02, 'number_of_recoveries': 0, 'elapsed_sec': 182.0},
    ])
    _write_jsonl(artifact_dir / 'phase95_local_costmap_samples.jsonl', [
        {'goal_sequence': 2, 'elapsed_sec': 180.0, 'robot_pose': [1.96, 2.02, 1.56], 'front_wedge_cost': {'lethal_count': 0}, 'robot_footprint_cost': {'lethal_count': 0}, 'target_footprint_cost': {'lethal_count': 0}},
    ])
    (artifact_dir / 'phase95_raw_capture.json').write_text(json.dumps({
        'robot_pose_by_frame': {'map': [1.96, 2.02, 1.56]},
        'odom': {'pose': {'x': 1.96, 'y': 2.02, 'yaw': 1.56}},
    }), encoding='utf-8')
    result = analyzer.analyze_artifact_dir(artifact_dir)
    assert result['validation_goal_sequence'] == 2
    assert result['selected_refinement']['refinement_applied'] is True
    assert result['selected_refinement']['selected_candidate_target'] == [2.0854, 1.0229]
    assert result['classification'] == 'APPLIED_REFINEMENT_TERMINAL_SUCCEEDED'


def test_phase95_comparison_includes_phase85_phase89_phase94(tmp_path):
    analyzer = _load_analyzer()
    result = analyzer.analyze_artifact_dir(
        _make_case(tmp_path, 'local_cost_blocked'),
        phase85_dir=tmp_path / 'local_cost_blocked' / 'phase85',
        phase89_dir=tmp_path / 'local_cost_blocked' / 'phase89',
        phase94_dir=tmp_path / 'local_cost_blocked' / 'phase94',
    )
    comparison = result['comparison_to_phase85_phase89_phase94']
    assert comparison['phase85']['classification'] == 'REFINEMENT_REJECTED_OR_NOT_TRIGGERED'
    assert comparison['phase89']['classification'] == 'PHASE88_REFINEMENT_STILL_REJECTED'
    assert comparison['phase94']['classification'] == 'APPLIED_REFINEMENT_HELD_BEFORE_TERMINAL_OUTCOME'
    assert comparison['phase95_vs_phase94']['terminal_outcome_changed_from_held_to_terminal'] is True
    assert comparison['phase95_vs_phase89']['refinement_applied_changed_false_to_true'] is True


def test_phase95_runbook_and_recorder_are_bounded_read_only_and_guarded():
    runbook = RUNBOOK.read_text(encoding='utf-8')
    recorder = RECORDER.read_text(encoding='utf-8')
    assert 'PHASE95_TERMINAL_WINDOW_SEC' in runbook
    assert 'PHASE95_RUNTIME_RECORD_TIMEOUT_SEC' in runbook
    assert 'PHASE95_MAX_GOALS must be 2 or 3' in runbook
    assert 'while true' in runbook  # scene hold is explicit, after bounded terminal validation
    assert '--stop-on-terminal' in runbook
    assert 'ros2 param set' not in runbook
    assert 'nav2_slam_params.yaml' in runbook
    assert 'No maze_explorer strategy edits' in runbook
    assert 'Goal2-equivalent' in runbook
    assert 'create_client' not in recorder
    assert 'send_goal' not in recorder
    assert '/local_costmap/costmap' in recorder
    assert '/scan' in recorder and '/odom' in recorder


def test_phase95_report_and_runbook_document_required_outputs():
    report = REPORT.read_text(encoding='utf-8')
    runbook = RUNBOOK_DOC.read_text(encoding='utf-8')
    for token in [
        'Applied refinement terminal outcome bounded validation',
        'APPLIED_REFINEMENT_TERMINAL_',
        'selected_candidate_target',
        'selected_candidate_yaw',
        'hard_safety_pass_candidate_count',
        'distance_remaining',
        'recoveries',
        'terminal_event',
        'terminal_reason',
        'terminal_pose',
        'front_wedge',
        'footprint',
        'Phase85',
        'Phase89',
        'Phase94',
        'No autonomous exploration success claimed',
        'Phase96 not entered',
    ]:
        assert token in report
    assert 'PHASE95_TERMINAL_WINDOW_SEC' in runbook
    assert 'Goal2-equivalent terminal outcome' in runbook
