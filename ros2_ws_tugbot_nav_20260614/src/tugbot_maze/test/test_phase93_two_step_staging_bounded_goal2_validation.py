from __future__ import annotations

import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
RUN_ID = 'phase93_two_step_staging_bounded_goal2_validation'
ANALYZER = ROOT / 'tools' / 'analyze_phase93_two_step_staging_goal2_validation.py'
RECORDER = ROOT / 'tools' / 'record_phase93_two_step_staging_evidence.py'
RUNBOOK = ROOT / 'tools' / 'run_phase93_two_step_staging_bounded_goal2_validation.sh'
RUNBOOK_DOC = ROOT / 'doc' / 'doc_report' / 'phase93_two_step_staging_bounded_goal2_validation_runbook.md'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase93_two_step_staging_bounded_goal2_validation_report.md'

ALLOWED_PHASE93_CLASSIFICATIONS = {
    'TWO_STEP_STAGING_APPLIED_AND_SECOND_STEP_DISPATCHED',
    'TWO_STEP_STAGING_APPLIED_BUT_SECOND_STEP_FAILED',
    'TWO_STEP_STAGING_REJECTED',
    'TWO_STEP_STAGING_NOT_TRIGGERED',
    'TWO_STEP_STAGING_VALIDATION_INSUFFICIENT_EVIDENCE',
}

REQUIRED_PHASE92_FIELDS = {
    'two_step_staging_plan',
    'staging_goal_pose',
    'staging_reason',
    'staging_executability_check',
    'second_step_forward_goal',
    'staging_applied',
    'staging_reject_reason',
    'branch_scoring_changed',
    'fallback_terminal_acceptance_used',
}

TRIGGER_KEYS = {
    'near_goal_lateral_residual',
    'single_step_forward_search_no_hard_safety_pass',
    'safety_floor_dominant_blocker',
    'execution_time_footprint_front_wedge_risk',
}


def _load_analyzer():
    spec = importlib.util.spec_from_file_location('phase93_analyzer', ANALYZER)
    assert spec and spec.loader, f'missing analyzer: {ANALYZER}'
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _write_jsonl(path: Path, rows: list[dict]) -> None:
    path.write_text('\n'.join(json.dumps({'state': row}, sort_keys=True) for row in rows) + '\n', encoding='utf-8')


def _base_dispatch(*, staging_applied: bool, trigger_all: bool = True, hard_safe: bool | None = None) -> dict:
    trigger_conditions = {key: trigger_all for key in TRIGGER_KEYS}
    hard_safe = staging_applied if hard_safe is None else hard_safe
    return {
        'event': 'dispatch',
        'goal_kind': 'explore',
        'goal_sequence': 1,
        'target': [2.084224301395533, 1.023027384249055],
        'original_target': [2.084224301395533, 1.023027384249055],
        'refinement_applied': False,
        'refinement_reject_reason': 'lethal_cost_regression',
        'multi_candidate_forward_search': True,
        'candidate_count': 63,
        'hard_safety_pass_candidate_count': 0,
        'original_target_preserved_on_reject': not staging_applied,
        'two_step_staging_plan': {
            'enabled': bool(trigger_all),
            'trigger_conditions': trigger_conditions,
            'source_single_step': {
                'candidate_count': 63,
                'hard_safety_pass_candidate_count': 0,
                'refinement_applied': False,
                'refinement_reject_reason': 'lethal_cost_regression',
                'original_target_preserved_on_reject': True,
            },
        },
        'staging_goal_pose': {'x': 2.24, 'y': 1.02, 'yaw': 1.57} if staging_applied else None,
        'staging_reason': 'reduce_lateral_residual_and_front_wedge_risk_before_second_step_forward_goal' if staging_applied else None,
        'staging_executability_check': {
            'checked': bool(trigger_all),
            'hard_safety_pass': hard_safe,
            'same_corridor': True,
            'two_side_wall_evidence': True,
            'safety_floor_ok': hard_safe,
            'footprint_lethal_not_increased': hard_safe,
            'front_wedge_lethal_not_increased': hard_safe,
            'forward_progress_ok': True,
            'local_costmap_stamp_age_sec': 0.11,
            'tf_stamp_age_sec': 0.03,
        },
        'second_step_forward_goal': None,
        'staging_applied': staging_applied,
        'staging_reject_reason': None if staging_applied else ('all_staging_candidates_failed_hard_safety' if trigger_all else 'trigger_bundle_not_satisfied'),
        'branch_scoring_changed': False,
        'fallback_terminal_acceptance_used': False,
    }


def _make_artifact_dir(tmp_path: Path, *, case: str) -> Path:
    artifact_dir = tmp_path / case / RUN_ID
    artifact_dir.mkdir(parents=True)
    rows: list[dict]
    if case == 'applied_dispatched':
        dispatch = _base_dispatch(staging_applied=True)
        staging_success = {
            'event': 'success',
            'goal_kind': 'corridor_alignment_staging',
            'goal_sequence': 1,
            'result_reason': 'goal_reached',
            'target': [2.24, 1.02],
            'staging_applied': True,
            'second_step_forward_goal': {
                'generated_after_fresh_evidence': False,
                'fresh_scan_received': False,
                'fresh_local_costmap_received': False,
                'fresh_tf_received': False,
            },
        }
        second_dispatch = {
            'event': 'dispatch',
            'goal_kind': 'explore',
            'goal_sequence': 2,
            'target': [2.44, 1.02],
            'original_target': [2.084224301395533, 1.023027384249055],
            'second_step_forward_goal': {
                'generated_after_fresh_evidence': True,
                'fresh_scan_received': True,
                'fresh_local_costmap_received': True,
                'fresh_tf_received': True,
                'selected_candidate_target': [2.44, 1.02],
                'selected_candidate_yaw': 1.57,
                'hard_safety_pass_candidate_count': 1,
            },
            'branch_scoring_changed': False,
            'fallback_terminal_acceptance_used': False,
        }
        rows = [dispatch, staging_success, second_dispatch]
    elif case == 'applied_failed':
        dispatch = _base_dispatch(staging_applied=True)
        staging_success = {'event': 'success', 'goal_kind': 'corridor_alignment_staging', 'goal_sequence': 1, 'result_reason': 'goal_reached'}
        second_dispatch = {
            'event': 'dispatch',
            'goal_kind': 'explore',
            'goal_sequence': 2,
            'target': [2.44, 1.02],
            'second_step_forward_goal': {
                'generated_after_fresh_evidence': True,
                'fresh_scan_received': True,
                'fresh_local_costmap_received': True,
                'fresh_tf_received': True,
                'selected_candidate_target': [2.44, 1.02],
                'hard_safety_pass_candidate_count': 1,
            },
            'branch_scoring_changed': False,
            'fallback_terminal_acceptance_used': False,
        }
        timeout = {'event': 'timeout', 'goal_kind': 'explore', 'goal_sequence': 2, 'result_reason': 'goal_timeout'}
        rows = [dispatch, staging_success, second_dispatch, timeout]
    elif case == 'rejected':
        rows = [_base_dispatch(staging_applied=False, trigger_all=True, hard_safe=False), {'event': 'timeout', 'goal_sequence': 1, 'result_reason': 'goal_timeout'}]
    elif case == 'not_triggered':
        rows = [_base_dispatch(staging_applied=False, trigger_all=False, hard_safe=False), {'event': 'timeout', 'goal_sequence': 1, 'result_reason': 'goal_timeout'}]
    elif case == 'insufficient':
        rows = [{'event': 'dispatch', 'goal_sequence': 1, 'staging_applied': True}, {'event': 'success', 'goal_sequence': 1}]
    else:
        raise AssertionError(case)
    _write_jsonl(artifact_dir / f'{RUN_ID}_goal_events.jsonl', rows)
    _write_jsonl(artifact_dir / f'{RUN_ID}_nav2_feedback.jsonl', [
        {'goal_sequence': 1, 'distance_remaining': 0.02, 'number_of_recoveries': 0},
        {'goal_sequence': 2, 'distance_remaining': 0.30 if case == 'applied_failed' else 0.05, 'number_of_recoveries': 2 if case == 'applied_failed' else 0},
    ])
    _write_jsonl(artifact_dir / f'{RUN_ID}_local_costmap_samples.jsonl', [
        {'goal_sequence': 1, 'front_wedge_lethal_count': 66, 'robot_footprint_lethal_count': 52, 'sample_age_sec': 0.12},
        {'goal_sequence': 2, 'front_wedge_lethal_count': 5 if case != 'applied_failed' else 80, 'robot_footprint_lethal_count': 0 if case != 'applied_failed' else 25, 'sample_age_sec': 0.08},
    ])
    raw = {
        'scan': {'topic': '/scan', 'stamp_sec': 10.1},
        'local_costmap': {'topic': '/local_costmap/costmap', 'stamp_sec': 10.2},
        'odom': {'topic': '/odom'},
        'tf': {'map->base_link': {'available': True, 'stamp_sec': 10.3}},
        'footprint': {'topic': '/local_costmap/published_footprint'},
        'fresh_evidence_timestamps': {'after_staging_success_scan_sec': 10.1, 'after_staging_success_local_costmap_sec': 10.2, 'after_staging_success_tf_sec': 10.3},
    }
    (artifact_dir / f'{RUN_ID}_raw_capture.json').write_text(json.dumps(raw), encoding='utf-8')
    return artifact_dir


def test_phase93_tooling_files_and_runbook_guardrails_exist():
    for path in [ANALYZER, RECORDER, RUNBOOK, RUNBOOK_DOC]:
        assert path.exists(), f'missing Phase93 tooling/runbook file: {path}'
    runbook = RUNBOOK.read_text(encoding='utf-8')
    for token in [
        'headless:=false',
        'use_rviz:=true',
        'PHASE93_MAX_GOALS',
        'SCENE_HELD_WAITING_FOR_USER_SCREENSHOT',
        'corridor_alignment_staging',
        'No Nav2/MPPI/controller tuning',
        'No branch scoring changed',
        'No fallback/terminal acceptance changed',
        'No autonomous exploration success claimed',
        'No exit success claimed',
    ]:
        assert token in runbook
    recorder = RECORDER.read_text(encoding='utf-8')
    for topic in ['/maze/goal_events', '/local_costmap/costmap', '/scan', '/odom']:
        assert topic in recorder
    assert 'fresh evidence' in recorder
    assert 'sends no goals' in recorder or 'send no goals' in recorder


def test_phase93_analyzer_contract_and_applied_second_step_dispatched(tmp_path):
    analyzer = _load_analyzer()
    assert set(analyzer.ALLOWED_CLASSIFICATIONS) == ALLOWED_PHASE93_CLASSIFICATIONS
    assert REQUIRED_PHASE92_FIELDS.issubset(set(analyzer.REQUIRED_PHASE92_DISPATCH_FIELDS))
    result = analyzer.analyze_artifact_dir(_make_artifact_dir(tmp_path, case='applied_dispatched'))
    assert result['classification'] == 'TWO_STEP_STAGING_APPLIED_AND_SECOND_STEP_DISPATCHED'
    assert result['trigger_bundle']['all_trigger_conditions_true'] is True
    assert result['staging_summary']['staging_hard_safety_pass'] is True
    assert result['staging_summary']['staging_nav2_dispatched'] is True
    assert result['fresh_evidence_summary']['all_fresh_evidence_received'] is True
    assert result['second_step_summary']['second_step_forward_goal_generated'] is True
    assert result['second_step_summary']['second_step_nav2_dispatched'] is True
    assert result['guardrails']['no_branch_scoring_changed'] is True
    assert result['guardrails']['no_fallback_terminal_acceptance_changed'] is True


def test_phase93_analyzer_applied_but_second_step_failed(tmp_path):
    analyzer = _load_analyzer()
    result = analyzer.analyze_artifact_dir(_make_artifact_dir(tmp_path, case='applied_failed'))
    assert result['classification'] == 'TWO_STEP_STAGING_APPLIED_BUT_SECOND_STEP_FAILED'
    assert result['second_step_summary']['second_step_timed_out_or_failed'] is True
    assert result['goal2_timeout_recovery_local_cost_risk']['recoveries_max'] >= 1
    assert result['goal2_timeout_recovery_local_cost_risk']['local_cost_risk_present'] is True


def test_phase93_analyzer_rejected_and_not_triggered_classifications(tmp_path):
    analyzer = _load_analyzer()
    rejected = analyzer.analyze_artifact_dir(_make_artifact_dir(tmp_path, case='rejected'))
    assert rejected['classification'] == 'TWO_STEP_STAGING_REJECTED'
    assert rejected['staging_summary']['staging_applied'] is False
    assert rejected['staging_summary']['staging_reject_reason'] == 'all_staging_candidates_failed_hard_safety'
    not_triggered = analyzer.analyze_artifact_dir(_make_artifact_dir(tmp_path, case='not_triggered'))
    assert not_triggered['classification'] == 'TWO_STEP_STAGING_NOT_TRIGGERED'
    assert not_triggered['trigger_bundle']['all_trigger_conditions_true'] is False


def test_phase93_analyzer_insufficient_evidence_requires_phase92_fields(tmp_path):
    analyzer = _load_analyzer()
    result = analyzer.analyze_artifact_dir(_make_artifact_dir(tmp_path, case='insufficient'))
    assert result['classification'] == 'TWO_STEP_STAGING_VALIDATION_INSUFFICIENT_EVIDENCE'
    assert any(gap.startswith('missing_dispatch_field:two_step_staging_plan') for gap in result['evidence_gaps'])
    assert 'missing_or_invalid_trigger_bundle' in result['evidence_gaps']


def test_phase93_report_documents_validation_only_scope_after_completion():
    assert REPORT.exists(), f'missing Phase93 report: {REPORT}'
    text = REPORT.read_text(encoding='utf-8')
    for token in ALLOWED_PHASE93_CLASSIFICATIONS:
        assert token in text
    for phrase in [
        'only validates Phase92 two-step staging',
        'No maze_explorer strategy changed',
        'No Phase92 staging logic changed',
        'No branch scoring changed',
        'No exploration order changed',
        'No centerline gate changed',
        'No directional readiness changed',
        'No fallback/terminal acceptance changed',
        'No Nav2/MPPI/controller tuning',
        'No autonomous exploration success claimed',
        'No exit success claimed',
        'Phase94 not entered',
    ]:
        assert phrase in text
