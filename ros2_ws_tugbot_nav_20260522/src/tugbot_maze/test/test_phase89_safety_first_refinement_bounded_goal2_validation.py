from __future__ import annotations

import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
RUN_ID = 'phase89_safety_first_refinement_bounded_goal2_validation'
ANALYZER = ROOT / 'tools' / 'analyze_phase89_safety_first_refinement_goal2_validation.py'
RECORDER = ROOT / 'tools' / 'record_phase89_safety_first_refinement_evidence.py'
RUNBOOK = ROOT / 'tools' / 'run_phase89_safety_first_refinement_bounded_goal2_validation.sh'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase89_safety_first_refinement_bounded_goal2_validation_report.md'
RUNBOOK_DOC = ROOT / 'doc' / 'doc_report' / 'phase89_safety_first_refinement_bounded_goal2_validation_runbook.md'

REQUIRED_PHASE88_FIELDS = {
    'multi_candidate_forward_search',
    'candidate_family',
    'candidate_count',
    'hard_safety_pass_candidate_count',
    'selected_candidate_index',
    'selected_candidate_target',
    'selected_candidate_yaw',
    'selection_priority_trace',
    'rejected_candidate_summaries',
    'refinement_applied',
    'refinement_reject_reason',
    'original_target_preserved_on_reject',
    'branch_scoring_changed',
    'fallback_terminal_acceptance_used',
}

ALLOWED_PHASE89_CLASSIFICATIONS = {
    'PHASE88_REFINEMENT_APPLIED_GOAL2_IMPROVED',
    'PHASE88_REFINEMENT_APPLIED_BUT_GOAL2_TIMEOUT',
    'PHASE88_REFINEMENT_STILL_REJECTED',
    'PHASE88_VALIDATION_INSUFFICIENT_EVIDENCE',
}


def _load_analyzer():
    spec = importlib.util.spec_from_file_location('phase89_analyzer', ANALYZER)
    assert spec and spec.loader, f'missing analyzer: {ANALYZER}'
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _write_jsonl(path: Path, rows: list[dict]) -> None:
    path.write_text('\n'.join(json.dumps({'state': row}, sort_keys=True) for row in rows) + '\n', encoding='utf-8')


def _make_artifact_dir(tmp_path: Path, *, applied: bool, timed_out: bool, selected_balance_error: float = 0.2, safer_than_best_centered: bool = True) -> Path:
    artifact_dir = tmp_path / RUN_ID
    artifact_dir.mkdir(parents=True)
    dispatch = {
        'event': 'dispatch',
        'goal_sequence': 1,
        'target': [2.09, 1.02] if applied else [2.084224301395533, 1.023027384249055],
        'original_target': [2.084224301395533, 1.023027384249055],
        'centerline_projected_target': [2.09, 1.02] if applied else None,
        'corridor_heading_yaw': 1.566,
        'refinement_applied': applied,
        'refinement_reject_reason': None if applied else 'lethal_cost_regression',
        'multi_candidate_forward_search': True,
        'candidate_family': ['centerline_projection', 'small_lateral_offsets', 'multiple_forward_offsets', 'corridor_heading_variants'],
        'candidate_count': 12,
        'hard_safety_pass_candidate_count': 2 if applied else 0,
        'selected_candidate_index': 7 if applied else None,
        'selected_candidate_target': [2.09, 1.02] if applied else None,
        'selected_candidate_yaw': 1.55 if applied else None,
        'selection_priority_trace': [
            'hard_safety_pass',
            'no_footprint_front_wedge_lethal_regression',
            'safety_floor_ok',
            'forward_progress_ok',
            'clearance_better',
            'balance_error_smaller',
        ],
        'selected_candidate_hard_safety_pass': True if applied else None,
        'selected_candidate_balance_error_m': selected_balance_error if applied else None,
        'best_balance_error_m': 0.0,
        'selected_candidate_not_best_centered_but_safer': safer_than_best_centered if applied else False,
        'rejected_candidate_summaries': [
            {'candidate_index': 0, 'family': 'centerline_projection', 'balance_error_m': 0.0, 'hard_safety_pass': False, 'reject_reasons': ['front_wedge_lethal_regression']},
        ],
        'original_target_preserved_on_reject': (not applied),
        'branch_scoring_changed': False,
        'fallback_terminal_acceptance_used': False,
        'forward_executability_check': {'passed': applied, 'reason': None if applied else 'lethal_cost_regression'},
    }
    outcome = {
        'event': 'timeout' if timed_out else 'success',
        'goal_sequence': 1,
        'target': dispatch['target'],
        'result_reason': 'goal_timeout' if timed_out else 'goal_reached',
    }
    _write_jsonl(artifact_dir / f'{RUN_ID}_goal_events.jsonl', [dispatch, outcome])
    _write_jsonl(artifact_dir / f'{RUN_ID}_nav2_feedback.jsonl', [{'goal_sequence': 1, 'distance_remaining': 0.33 if timed_out else 0.01, 'number_of_recoveries': 2 if timed_out else 0}])
    _write_jsonl(artifact_dir / f'{RUN_ID}_local_costmap_samples.jsonl', [{'goal_sequence': 1, 'front_wedge_lethal_count': 0 if applied else 70, 'robot_footprint_lethal_count': 0 if applied else 52}])
    raw = {'scan': {'topic': '/scan'}, 'local_costmap': {'topic': '/local_costmap/costmap'}, 'odom': {'topic': '/odom'}, 'tf': {'map->base_link': {'available': True}}, 'footprint': {'topic': '/local_costmap/published_footprint'}}
    (artifact_dir / f'{RUN_ID}_raw_capture.json').write_text(json.dumps(raw), encoding='utf-8')
    return artifact_dir


def test_phase89_tooling_files_and_runbook_guardrails_exist():
    for path in [ANALYZER, RECORDER, RUNBOOK, RUNBOOK_DOC]:
        assert path.exists(), f'missing Phase89 tooling/runbook file: {path}'
    runbook_text = RUNBOOK.read_text(encoding='utf-8')
    assert 'headless:=false' in runbook_text
    assert 'use_rviz:=true' in runbook_text
    assert 'PHASE89_MAX_GOALS' in runbook_text and 'MAX_GOALS' in runbook_text
    assert 'centerline_target_refinement_enabled:=true' in runbook_text
    assert 'directional_local_costmap_readiness_override_enabled:=true' in runbook_text
    assert 'near_exit_fallback_enabled:=false' in runbook_text
    assert 'No Nav2/MPPI/controller tuning' in runbook_text
    assert 'No autonomous exploration success claimed' in runbook_text
    assert 'SCENE_HELD_WAITING_FOR_USER_SCREENSHOT' in runbook_text
    recorder_text = RECORDER.read_text(encoding='utf-8')
    for topic in ['/maze/goal_events', '/local_costmap/costmap', '/scan', '/odom']:
        assert topic in recorder_text
    assert 'send no goals' in recorder_text or 'sends no goals' in recorder_text


def test_phase89_analyzer_contract_and_applied_timeout_classification(tmp_path):
    analyzer = _load_analyzer()
    assert set(analyzer.ALLOWED_CLASSIFICATIONS) == ALLOWED_PHASE89_CLASSIFICATIONS
    assert REQUIRED_PHASE88_FIELDS.issubset(set(analyzer.REQUIRED_PHASE88_DISPATCH_FIELDS))
    artifact_dir = _make_artifact_dir(tmp_path, applied=True, timed_out=True)
    result = analyzer.analyze_artifact_dir(artifact_dir)
    assert result['classification'] == 'PHASE88_REFINEMENT_APPLIED_BUT_GOAL2_TIMEOUT'
    ctx = result['goal2_dispatch_context']
    assert ctx['multi_candidate_forward_search'] is True
    assert ctx['selected_candidate_hard_safety_pass'] is True
    assert result['selected_candidate_assessment']['selected_candidate_hard_safety_pass'] is True
    assert result['selected_candidate_assessment']['selected_candidate_not_best_centered_but_safer'] is True
    assert result['goal2_dispatch_context']['branch_scoring_changed'] is False
    assert result['goal2_dispatch_context']['fallback_terminal_acceptance_used'] is False


def test_phase89_analyzer_applied_improved_and_rejected_classifications(tmp_path):
    analyzer = _load_analyzer()
    improved_dir = _make_artifact_dir(tmp_path / 'improved', applied=True, timed_out=False)
    improved = analyzer.analyze_artifact_dir(improved_dir)
    assert improved['classification'] == 'PHASE88_REFINEMENT_APPLIED_GOAL2_IMPROVED'
    rejected_dir = _make_artifact_dir(tmp_path / 'rejected', applied=False, timed_out=True)
    rejected = analyzer.analyze_artifact_dir(rejected_dir)
    assert rejected['classification'] == 'PHASE88_REFINEMENT_STILL_REJECTED'
    assert rejected['goal2_dispatch_context']['original_target_preserved_on_reject'] is True
    assert rejected['goal2_refinement_summary']['refinement_reject_reason'] == 'lethal_cost_regression'


def test_phase89_analyzer_insufficient_evidence_requires_phase88_fields(tmp_path):
    analyzer = _load_analyzer()
    artifact_dir = tmp_path / RUN_ID
    artifact_dir.mkdir()
    _write_jsonl(artifact_dir / f'{RUN_ID}_goal_events.jsonl', [{'event': 'dispatch', 'goal_sequence': 1, 'refinement_applied': True}, {'event': 'success', 'goal_sequence': 1}])
    result = analyzer.analyze_artifact_dir(artifact_dir)
    assert result['classification'] == 'PHASE88_VALIDATION_INSUFFICIENT_EVIDENCE'
    assert any(gap.startswith('missing_dispatch_field:multi_candidate_forward_search') for gap in result['evidence_gaps'])
    assert 'missing_nav2_feedback_artifact' in result['evidence_gaps']
    assert 'missing_raw_capture_artifact' in result['evidence_gaps']


def test_phase89_report_documents_validation_scope_and_no_success_claim():
    assert REPORT.exists(), f'missing Phase89 report: {REPORT}'
    text = REPORT.read_text(encoding='utf-8')
    for token in ALLOWED_PHASE89_CLASSIFICATIONS:
        assert token in text
    for phrase in [
        'only validates Phase88',
        'No maze_explorer strategy changed',
        'No branch scoring changed',
        'No centerline gate changed',
        'No directional readiness changed',
        'No fallback/terminal acceptance changed',
        'No Nav2/MPPI/controller tuning',
        'No autonomous exploration success claimed',
        'No exit success claimed',
        'Phase90 not entered',
    ]:
        assert phrase in text
