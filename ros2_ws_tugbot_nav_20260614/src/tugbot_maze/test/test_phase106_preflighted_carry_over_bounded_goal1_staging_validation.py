from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[3]
ANALYZER = ROOT / 'tools' / 'analyze_phase106_preflighted_carry_over_bounded_goal1_staging_validation.py'
RECORDER = ROOT / 'tools' / 'record_phase106_preflighted_carry_over_validation_evidence.py'
WRAPPER = ROOT / 'tools' / 'run_phase106_preflighted_carry_over_bounded_goal1_staging_validation.sh'
RUNBOOK = ROOT / 'doc' / 'doc_report' / 'phase106_preflighted_carry_over_bounded_goal1_staging_validation_runbook.md'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase106_preflighted_carry_over_bounded_goal1_staging_validation_report.md'
RUN_ID = 'phase106_preflighted_carry_over_bounded_goal1_staging_validation'


def _load_analyzer():
    assert ANALYZER.exists(), f'missing {ANALYZER}'
    spec = importlib.util.spec_from_file_location('phase106_analyzer', ANALYZER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _write_jsonl(path: Path, rows: list[dict]) -> None:
    path.write_text('\n'.join(json.dumps(row, sort_keys=True) for row in rows) + '\n')


def _dispatch_row(**overrides):
    row = {
        'event': 'dispatch',
        'goal_sequence': 1,
        'target': [2.017031729360098, 0.9240155526940924],
        'selected_candidate_target': [2.017031729360098, 0.9240155526940924],
        'selected_candidate_yaw': 1.563578938982321,
        'hard_safety_pass_candidate_count': 0,
        'corridor_evidence_carry_over': {'evaluated': True, 'eligible': True},
        'carry_over_source': {
            'goal_sequence': 1,
            'frame_id': 'map',
            'same_corridor': True,
            'two_side_wall_evidence': True,
            'corridor_heading': 1.563578938982321,
            'hard_safety_pass': False,
            'safety_floor_ok': False,
        },
        'carry_over_applied': True,
        'carry_over_reject_reason': None,
        'source_forward_window': {
            'candidate_count': 63,
            'same_corridor_count': 63,
            'two_side_wall_count': 63,
            'y_range': {'min': 0.9207677563752615, 'max': 1.1272581399668515},
        },
        'staging_window': {
            'candidate_count': 9,
            'same_corridor_count': 9,
            'two_side_wall_count': 0,
            'y_range': {'min': 0.024038993401414976, 'max': 0.12439725515215896},
        },
        'safety_evidence_recomputed': True,
        'staging_executability_check': {
            'checked': True,
            'hard_safety_pass': False,
            'same_corridor': True,
            'two_side_wall_evidence': True,
            'local_two_side_wall_evidence': False,
            'safety_evidence_recomputed': True,
            'target_has_clearance': False,
            'safety_floor_ok': False,
            'footprint_lethal_not_increased': True,
            'front_wedge_lethal_not_increased': True,
            'reason': 'staging_safety_recompute_failed',
        },
        'two_step_staging_plan': {
            'enabled': True,
            'trigger_conditions': {'single_step_forward_search_no_hard_safety_pass': True},
        },
        'staging_applied': False,
        'staging_reject_reason': 'staging_safety_recompute_failed',
        'branch_scoring_changed': False,
        'fallback_terminal_acceptance_used': False,
    }
    row.update(overrides)
    return row


def _artifact_dir(tmp_path: Path, row: dict | None, *, preflight: dict | None = None, ingress: dict | None = None, trigger: dict | None = None) -> Path:
    artifact_dir = tmp_path / 'artifact'
    artifact_dir.mkdir(parents=True)
    if row is not None:
        _write_jsonl(artifact_dir / f'{RUN_ID}_goal_events.jsonl', [{'state': row}])
    _write_jsonl(artifact_dir / f'{RUN_ID}_nav2_feedback.jsonl', [
        {'goal_sequence': 1, 'distance_remaining': 0.9, 'number_of_recoveries': 0, 'elapsed_sec': 1.0},
    ])
    _write_jsonl(artifact_dir / f'{RUN_ID}_local_costmap_samples.jsonl', [
        {'goal_sequence': 1, 'front_wedge_cost': {'summary': {'lethal_count': 0}}, 'robot_footprint_cost': {'summary': {'lethal_count': 0}}},
    ])
    preflight = {'ingress_preflight': {'passed': True, 'ingress_goal_sent': True, 'maze_explorer_started': bool(row is not None)}} if preflight is None else preflight
    ingress = {'success': True, 'status': 'succeeded', 'ingress_goal_sent': True} if ingress is None else ingress
    (artifact_dir / f'{RUN_ID}_ingress_preflight.json').write_text(json.dumps(preflight, indent=2, sort_keys=True))
    (artifact_dir / f'{RUN_ID}_ingress_result.json').write_text(json.dumps(ingress, indent=2, sort_keys=True))
    if trigger is not None:
        (artifact_dir / f'{RUN_ID}_trigger_detected.json').write_text(json.dumps(trigger, indent=2, sort_keys=True))
    (artifact_dir / f'{RUN_ID}_raw_capture.json').write_text(json.dumps({
        'scan': {'available': True},
        'map': {'summary': {'known_count': 10}},
        'local_costmap': {'summary': {'known_count': 10}},
        'odom': {'available': True},
        'tf': {'map->base_link': {'available': True}},
    }))
    return artifact_dir


def test_phase106_analyzer_classification_set_and_safety_rejected(tmp_path):
    analyzer = _load_analyzer()
    assert analyzer.ALLOWED_CLASSIFICATIONS == {
        'PREFLIGHTED_CARRY_OVER_APPLIED_STAGING_APPLIED',
        'PREFLIGHTED_CARRY_OVER_APPLIED_STAGING_SAFETY_REJECTED',
        'PREFLIGHTED_CARRY_OVER_REJECTED',
        'PREFLIGHTED_CARRY_OVER_NOT_TRIGGERED',
        'PHASE105_PREFLIGHT_REJECTED_BEFORE_INGRESS',
        'PREFLIGHTED_INGRESS_FAILED_AFTER_PREFLIGHT_PASS',
        'PREFLIGHTED_GOAL1_CARRY_OVER_VALIDATION_INSUFFICIENT_EVIDENCE',
    }
    result = analyzer.analyze_artifact_dir(_artifact_dir(tmp_path, _dispatch_row()), run_id=RUN_ID)
    assert result['classification'] == 'PREFLIGHTED_CARRY_OVER_APPLIED_STAGING_SAFETY_REJECTED'
    assert result['direct_answers']['phase105_preflight_passed'] is True
    assert result['direct_answers']['ingress_goal_sent_only_after_preflight_pass'] is True
    assert result['direct_answers']['ingress_success'] is True
    assert result['direct_answers']['goal1_dispatched'] is True
    assert result['direct_answers']['missing_two_side_wall_fixed_by_carry_over'] is True
    assert result['direct_answers']['carry_over_is_corridor_level_only'] is True
    assert result['direct_answers']['safety_evidence_recomputed_is_true'] is True
    assert result['direct_answers']['staging_safety_recompute_failed'] is True
    assert result['goal1']['branch_scoring_changed'] is False
    assert result['goal1']['fallback_terminal_acceptance_used'] is False


def test_phase106_analyzer_preflight_and_ingress_failure_precedence(tmp_path):
    analyzer = _load_analyzer()
    reject_artifact = _artifact_dir(
        tmp_path / 'reject',
        None,
        preflight={'ingress_preflight': {'passed': False, 'ingress_goal_sent': False, 'maze_explorer_started': False, 'ingress_preflight_reject_reason': 'ingress_tf_unstable'}},
        ingress={'success': False, 'status': 'preflight_rejected', 'ingress_goal_sent': False, 'maze_explorer_started': False},
        trigger={'trigger': 'ingress_preflight_rejected_explorer_not_started', 'ingress_preflight_reject_reason': 'ingress_tf_unstable'},
    )
    reject = analyzer.analyze_artifact_dir(reject_artifact, run_id=RUN_ID)
    assert reject['classification'] == 'PHASE105_PREFLIGHT_REJECTED_BEFORE_INGRESS'
    assert reject['direct_answers']['phase105_preflight_passed'] is False
    assert reject['direct_answers']['ingress_goal_sent_only_after_preflight_pass'] is True
    assert reject['ingress_preflight_reject_reason'] == 'ingress_tf_unstable'

    ingress_failed_artifact = _artifact_dir(
        tmp_path / 'ingress_failed',
        None,
        preflight={'ingress_preflight': {'passed': True, 'ingress_goal_sent': True, 'maze_explorer_started': False}},
        ingress={'success': False, 'status': 6, 'error_code': 102, 'ingress_goal_sent': True},
        trigger={'trigger': 'ingress_failed_explorer_not_started'},
    )
    failed = analyzer.analyze_artifact_dir(ingress_failed_artifact, run_id=RUN_ID)
    assert failed['classification'] == 'PREFLIGHTED_INGRESS_FAILED_AFTER_PREFLIGHT_PASS'
    assert failed['direct_answers']['phase105_preflight_passed'] is True
    assert failed['direct_answers']['ingress_success'] is False
    assert 'ingress_failed_after_preflight_pass' in failed['evidence_gaps']


def test_phase106_analyzer_other_goal1_classifications(tmp_path):
    analyzer = _load_analyzer()
    applied = _dispatch_row(staging_applied=True, staging_reject_reason=None, staging_executability_check={'checked': True, 'hard_safety_pass': True, 'same_corridor': True, 'two_side_wall_evidence': True, 'safety_evidence_recomputed': True})
    assert analyzer.analyze_artifact_dir(_artifact_dir(tmp_path / 'applied', applied), run_id=RUN_ID)['classification'] == 'PREFLIGHTED_CARRY_OVER_APPLIED_STAGING_APPLIED'

    rejected = _dispatch_row(carry_over_applied=False, carry_over_reject_reason='heading_mismatch', staging_reject_reason='heading_mismatch')
    assert analyzer.analyze_artifact_dir(_artifact_dir(tmp_path / 'rejected', rejected), run_id=RUN_ID)['classification'] == 'PREFLIGHTED_CARRY_OVER_REJECTED'

    not_triggered = _dispatch_row(corridor_evidence_carry_over={'evaluated': False, 'eligible': False}, carry_over_applied=False, carry_over_reject_reason=None)
    assert analyzer.analyze_artifact_dir(_artifact_dir(tmp_path / 'not_triggered', not_triggered), run_id=RUN_ID)['classification'] == 'PREFLIGHTED_CARRY_OVER_NOT_TRIGGERED'

    empty = _artifact_dir(tmp_path / 'empty', None)
    assert analyzer.analyze_artifact_dir(empty, run_id=RUN_ID)['classification'] == 'PREFLIGHTED_GOAL1_CARRY_OVER_VALIDATION_INSUFFICIENT_EVIDENCE'


def test_phase106_recorder_is_read_only_and_captures_required_evidence():
    text = RECORDER.read_text()
    assert 'from rclpy.action import ActionClient' not in text
    assert '.send_goal' not in text
    assert 'set_parameters(' not in text
    for topic in ['/maze/goal_events', '/scan', '/map', '/local_costmap/costmap', '/local_costmap/published_footprint', '/odom']:
        assert topic in text
    assert 'TransformListener' in text
    assert 'phase106_preflighted_carry_over_bounded_goal1_staging_validation' in text


def test_phase106_wrapper_preflight_fail_closed_and_guarded():
    text = WRAPPER.read_text()
    assert 'initial_cleanup' in text
    assert 'phase105_inner_ingress_tf_controller_preflight.py' in text
    assert 'write_preflight_reject_ingress_result' in text
    assert 'send_ingress_goal' in text
    assert text.index('run_ingress_preflight') < text.index('send_ingress_goal')
    assert 'if run_ingress_preflight; then' in text
    assert 'mark_ingress_preflight_wrapper_state false false' in text
    assert 'PHASE106_MAX_GOALS' in text
    assert 'max_goals:="$MAX_GOALS"' in text
    assert 'headless:=false' in text and 'use_rviz:=true' in text
    assert 'near_exit_fallback_enabled:=false' in text
    assert 'git diff -- src/tugbot_navigation/config' in text
    forbidden_tuning = ['controller_server:', 'inflation_radius:', 'robot_radius:', 'clearance_radius_m:=']
    for token in forbidden_tuning:
        assert token not in text


def test_phase106_runbook_report_and_nav2_config_guard_present():
    for path in [RUNBOOK, REPORT]:
        assert path.exists(), f'missing {path}'
        text = path.read_text()
        for token in [
            'PREFLIGHTED_CARRY_OVER_APPLIED_STAGING_APPLIED',
            'PREFLIGHTED_CARRY_OVER_APPLIED_STAGING_SAFETY_REJECTED',
            'PREFLIGHTED_CARRY_OVER_REJECTED',
            'PREFLIGHTED_CARRY_OVER_NOT_TRIGGERED',
            'PHASE105_PREFLIGHT_REJECTED_BEFORE_INGRESS',
            'PREFLIGHTED_INGRESS_FAILED_AFTER_PREFLIGHT_PASS',
            'PREFLIGHTED_GOAL1_CARRY_OVER_VALIDATION_INSUFFICIENT_EVIDENCE',
            'missing_two_side_wall_evidence',
            'safety_evidence_recomputed',
            'No autonomous exploration success claimed',
            'No exit success claimed',
            'Phase107 not entered',
        ]:
            assert token in text

    result = subprocess.run(['git', 'diff', '--', 'src/tugbot_navigation/config'], cwd=ROOT, text=True, capture_output=True, check=False)
    assert result.stdout == ''
