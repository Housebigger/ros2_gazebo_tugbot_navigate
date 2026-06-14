from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[3]
ANALYZER = ROOT / 'tools' / 'analyze_phase102_carry_over_bounded_goal1_staging_validation.py'
RECORDER = ROOT / 'tools' / 'record_phase102_carry_over_validation_evidence.py'
WRAPPER = ROOT / 'tools' / 'run_phase102_carry_over_bounded_goal1_staging_validation.sh'
RUNBOOK = ROOT / 'doc' / 'doc_report' / 'phase102_carry_over_bounded_goal1_staging_validation_runbook.md'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase102_carry_over_bounded_goal1_staging_validation_report.md'


def _load_analyzer():
    assert ANALYZER.exists(), f'missing {ANALYZER}'
    spec = importlib.util.spec_from_file_location('phase102_analyzer', ANALYZER)
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
        'two_step_staging_plan': {'enabled': True, 'trigger_conditions': {'single_step_forward_search_no_hard_safety_pass': True}},
        'staging_applied': False,
        'staging_reject_reason': 'staging_safety_recompute_failed',
        'branch_scoring_changed': False,
        'fallback_terminal_acceptance_used': False,
    }
    row.update(overrides)
    return row


def _artifact_dir(tmp_path: Path, row: dict) -> Path:
    artifact_dir = tmp_path / 'artifact'
    artifact_dir.mkdir(parents=True)
    run_id = 'phase102_carry_over_bounded_goal1_staging_validation'
    _write_jsonl(artifact_dir / f'{run_id}_goal_events.jsonl', [{'state': row}])
    _write_jsonl(artifact_dir / f'{run_id}_nav2_feedback.jsonl', [
        {'goal_sequence': 1, 'distance_remaining': 0.9, 'number_of_recoveries': 0, 'elapsed_sec': 1.0},
    ])
    _write_jsonl(artifact_dir / f'{run_id}_local_costmap_samples.jsonl', [
        {'goal_sequence': 1, 'front_wedge_cost': {'summary': {'lethal_count': 0}}, 'robot_footprint_cost': {'summary': {'lethal_count': 0}}},
    ])
    (artifact_dir / f'{run_id}_ingress_result.json').write_text(json.dumps({'success': True, 'status': 'succeeded'}))
    (artifact_dir / f'{run_id}_raw_capture.json').write_text(json.dumps({
        'scan': {'available': True}, 'map': {'summary': {'known_count': 10}}, 'local_costmap': {'summary': {'known_count': 10}},
        'odom': {'available': True}, 'tf': {'map->base_link': {'available': True}},
    }))
    return artifact_dir


def test_phase102_analyzer_classification_set_and_applied_safety_rejected(tmp_path):
    analyzer = _load_analyzer()
    assert analyzer.ALLOWED_CLASSIFICATIONS == {
        'CARRY_OVER_APPLIED_STAGING_APPLIED',
        'CARRY_OVER_APPLIED_STAGING_SAFETY_REJECTED',
        'CARRY_OVER_REJECTED',
        'CARRY_OVER_NOT_TRIGGERED',
        'GOAL1_CARRY_OVER_VALIDATION_INSUFFICIENT_EVIDENCE',
    }
    artifact_dir = _artifact_dir(tmp_path, _dispatch_row())
    result = analyzer.analyze_artifact_dir(artifact_dir, run_id='phase102_carry_over_bounded_goal1_staging_validation')
    assert result['classification'] == 'CARRY_OVER_APPLIED_STAGING_SAFETY_REJECTED'
    goal1 = result['goal1']
    assert goal1['carry_over_applied'] is True
    assert goal1['staging_applied'] is False
    assert goal1['staging_reject_reason'] == 'staging_safety_recompute_failed'
    assert goal1['safety_evidence_recomputed'] is True
    assert result['direct_answers']['missing_two_side_wall_fixed_by_carry_over'] is True
    assert result['direct_answers']['carry_over_is_corridor_level_only'] is True
    assert result['guardrails']['no_success_claimed'] is True


def test_phase102_analyzer_other_classifications(tmp_path):
    analyzer = _load_analyzer()
    run_id = 'phase102_carry_over_bounded_goal1_staging_validation'

    applied = _dispatch_row(staging_applied=True, staging_reject_reason=None, staging_executability_check={'checked': True, 'hard_safety_pass': True, 'same_corridor': True, 'two_side_wall_evidence': True, 'safety_evidence_recomputed': True})
    assert analyzer.analyze_artifact_dir(_artifact_dir(tmp_path / 'applied', applied), run_id=run_id)['classification'] == 'CARRY_OVER_APPLIED_STAGING_APPLIED'

    rejected = _dispatch_row(carry_over_applied=False, carry_over_reject_reason='heading_mismatch', staging_reject_reason='heading_mismatch')
    assert analyzer.analyze_artifact_dir(_artifact_dir(tmp_path / 'rejected', rejected), run_id=run_id)['classification'] == 'CARRY_OVER_REJECTED'

    not_triggered = _dispatch_row(corridor_evidence_carry_over={'evaluated': False, 'eligible': False}, carry_over_applied=False, carry_over_reject_reason=None)
    assert analyzer.analyze_artifact_dir(_artifact_dir(tmp_path / 'not_triggered', not_triggered), run_id=run_id)['classification'] == 'CARRY_OVER_NOT_TRIGGERED'

    empty = tmp_path / 'empty'
    empty.mkdir()
    assert analyzer.analyze_artifact_dir(empty, run_id=run_id)['classification'] == 'GOAL1_CARRY_OVER_VALIDATION_INSUFFICIENT_EVIDENCE'


def test_phase102_recorder_is_read_only_and_captures_required_topics():
    text = RECORDER.read_text()
    assert 'from rclpy.action import ActionClient' not in text
    assert '.send_goal' not in text
    assert 'set_parameters(' not in text
    for topic in ['/maze/goal_events', '/scan', '/map', '/local_costmap/costmap', '/local_costmap/published_footprint', '/odom']:
        assert topic in text
    assert 'TransformListener' in text


def test_phase102_wrapper_ingress_guided_bounded_and_guarded():
    text = WRAPPER.read_text()
    assert 'explicit inner-ingress Nav2 goal before maze_explorer' in text
    assert 'analyze_phase74_directional_local_costmap_readiness_gate_validation.py --send-inner-ingress-goal' in text
    assert 'max_goals:="$MAX_GOALS"' in text
    assert 'PHASE102_MAX_GOALS' in text
    assert 'wait_for_goal1_carry_over_or_terminal' in text
    assert 'staging_or_carry_over_context_observed' in text
    assert 'headless:=false' in text and 'use_rviz:=true' in text
    assert 'near_exit_fallback_enabled:=false' in text
    assert 'centerline_target_refinement_enabled:=true' in text
    assert 'git diff -- src/tugbot_navigation/config' in text
    forbidden_tuning = ['controller_server:', 'inflation_radius:', 'robot_radius:', 'clearance_radius_m:=']
    for token in forbidden_tuning:
        assert token not in text


def test_phase102_runbook_report_and_static_guards_present():
    for path in [RUNBOOK, REPORT]:
        assert path.exists(), f'missing {path}'
        text = path.read_text()
        for token in [
            'CARRY_OVER_APPLIED_STAGING_APPLIED',
            'CARRY_OVER_APPLIED_STAGING_SAFETY_REJECTED',
            'CARRY_OVER_REJECTED',
            'CARRY_OVER_NOT_TRIGGERED',
            'GOAL1_CARRY_OVER_VALIDATION_INSUFFICIENT_EVIDENCE',
            'missing_two_side_wall_evidence',
            'safety_evidence_recomputed',
            'No autonomous exploration success claimed',
            'No exit success claimed',
            'Phase103 not entered',
        ]:
            assert token in text

    result = subprocess.run(['git', 'diff', '--', 'src/tugbot_navigation/config'], cwd=ROOT, text=True, capture_output=True, check=False)
    assert result.stdout == ''
