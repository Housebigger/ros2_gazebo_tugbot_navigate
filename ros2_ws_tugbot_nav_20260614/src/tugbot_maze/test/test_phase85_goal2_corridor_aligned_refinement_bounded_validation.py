from __future__ import annotations

import importlib.util
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
ANALYZER = ROOT / 'tools' / 'analyze_phase85_goal2_corridor_aligned_refinement_validation.py'
RUNBOOK = ROOT / 'tools' / 'run_phase85_goal2_corridor_aligned_refinement_bounded_validation.sh'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase85_goal2_corridor_aligned_refinement_bounded_validation_report.md'
ARTIFACT_DIR = ROOT / 'log' / 'phase85_goal2_corridor_aligned_refinement_bounded_validation'
OUTPUT_JSON = ARTIFACT_DIR / 'phase85_goal2_corridor_aligned_refinement_bounded_validation_analysis.json'
MIN_SUMMARY = ARTIFACT_DIR / 'phase85_goal2_corridor_aligned_refinement_bounded_validation_minimal_field_summary.md'
CLEANUP_SUMMARY = ARTIFACT_DIR / 'phase85_process_cleanup_summary.md'


ALLOWED_CLASSIFICATIONS = {
    'REFINEMENT_APPLIED_AND_GOAL2_IMPROVED',
    'REFINEMENT_APPLIED_BUT_GOAL2_STILL_TIMEOUT',
    'REFINEMENT_REJECTED_OR_NOT_TRIGGERED',
    'INSUFFICIENT_VALIDATION_EVIDENCE',
}


def load_analyzer():
    assert ANALYZER.exists(), f'missing Phase85 analyzer: {ANALYZER}'
    spec = importlib.util.spec_from_file_location('phase85_analyzer', ANALYZER)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_jsonl(path: Path, payloads: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('\n'.join(json.dumps(p, sort_keys=True) for p in payloads) + '\n', encoding='utf-8')


def make_dispatch(*, applied: bool = True) -> dict:
    return {
        'event': 'dispatch',
        'goal_sequence': 2,
        'target': [2.06, 1.30] if applied else [2.057855221699651, 1.0261005743935105],
        'original_target': [2.057855221699651, 1.0261005743935105],
        'centerline_projected_target': [2.06, 1.30] if applied else None,
        'corridor_heading_yaw': 1.5968650403250249 if applied else None,
        'refinement_applied': applied,
        'refinement_reject_reason': None if applied else 'missing_two_side_wall_evidence',
        'forward_executability_check': {
            'checked': True,
            'passed': applied,
            'reason': None if applied else 'missing_two_side_wall_evidence',
        },
        'branch_scoring_changed': False,
        'fallback_terminal_acceptance_used': False,
    }


def make_artifact_dir(tmp_path: Path, *, dispatch: dict | None, outcome: dict | None, feedback: list[dict] | None = None) -> Path:
    artifact_dir = tmp_path / 'phase85_artifact'
    artifact_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    if dispatch is not None:
        rows.append({'elapsed_sec': 1.0, 'state': dispatch})
    if outcome is not None:
        rows.append({'elapsed_sec': 20.0, 'state': outcome})
    write_jsonl(artifact_dir / 'phase85_goal2_corridor_aligned_refinement_bounded_validation_goal_events.jsonl', rows)
    write_jsonl(
        artifact_dir / 'phase85_goal2_corridor_aligned_refinement_bounded_validation_nav2_feedback.jsonl',
        feedback or [{'goal_sequence': 2, 'distance_remaining': 0.08, 'number_of_recoveries': 0}],
    )
    (artifact_dir / 'phase85_goal2_corridor_aligned_refinement_bounded_validation_raw_capture.json').write_text(json.dumps({
        'scan': {'ranges': [2.0], 'frame_id': 'scan_omni'},
        'local_costmap': {'data': [0], 'frame_id': 'odom'},
        'odom': {'pose': {'x': 2.0, 'y': 1.2, 'yaw': 1.59}},
        'tf': {'map->base_link': {'available': True}},
    }), encoding='utf-8')
    return artifact_dir


def test_phase85_analyzer_classifies_applied_and_improved_when_goal2_succeeds(tmp_path):
    module = load_analyzer()
    artifact_dir = make_artifact_dir(
        tmp_path,
        dispatch=make_dispatch(applied=True),
        outcome={'event': 'success', 'goal_sequence': 2, 'result_reason': 'goal_reached'},
        feedback=[{'goal_sequence': 2, 'distance_remaining': 0.05, 'number_of_recoveries': 0}],
    )

    result = module.analyze_artifact_dir(artifact_dir)

    assert result['classification'] == 'REFINEMENT_APPLIED_AND_GOAL2_IMPROVED'
    assert result['goal2_dispatch_context']['refinement_applied'] is True
    assert result['goal2_dispatch_context']['branch_scoring_changed'] is False
    assert result['goal2_dispatch_context']['fallback_terminal_acceptance_used'] is False
    assert result['goal2_refinement_summary']['target_shift_m'] > 0.05
    assert result['goal2_refinement_summary']['forward_executability_passed'] is True
    assert result['complete_autonomous_success_claimed'] is False
    assert result['exit_success_claimed'] is False


def test_phase85_analyzer_classifies_applied_but_timeout(tmp_path):
    module = load_analyzer()
    artifact_dir = make_artifact_dir(
        tmp_path,
        dispatch=make_dispatch(applied=True),
        outcome={'event': 'timeout', 'goal_sequence': 2, 'result_reason': 'goal_timeout'},
        feedback=[{'goal_sequence': 2, 'distance_remaining': 0.31, 'number_of_recoveries': 2}],
    )

    result = module.analyze_artifact_dir(artifact_dir)

    assert result['classification'] == 'REFINEMENT_APPLIED_BUT_GOAL2_STILL_TIMEOUT'
    assert result['goal2_outcome']['timed_out'] is True
    assert result['nav2_feedback_summary']['recoveries_max'] == 2


def test_phase85_analyzer_classifies_rejected_or_not_triggered(tmp_path):
    module = load_analyzer()
    artifact_dir = make_artifact_dir(
        tmp_path,
        dispatch=make_dispatch(applied=False),
        outcome={'event': 'timeout', 'goal_sequence': 2, 'result_reason': 'goal_timeout'},
    )

    result = module.analyze_artifact_dir(artifact_dir)

    assert result['classification'] == 'REFINEMENT_REJECTED_OR_NOT_TRIGGERED'
    assert result['goal2_dispatch_context']['refinement_reject_reason'] == 'missing_two_side_wall_evidence'
    assert result['goal2_refinement_summary']['target_shift_m'] == 0.0


def test_phase85_analyzer_reports_insufficient_evidence_when_goal2_dispatch_missing(tmp_path):
    module = load_analyzer()
    artifact_dir = make_artifact_dir(
        tmp_path,
        dispatch=None,
        outcome={'event': 'timeout', 'goal_sequence': 2, 'result_reason': 'goal_timeout'},
    )

    result = module.analyze_artifact_dir(artifact_dir)

    assert result['classification'] == 'INSUFFICIENT_VALIDATION_EVIDENCE'
    assert 'missing_goal2_dispatch_event' in result['evidence_gaps']


def test_phase85_static_outputs_and_report_contract():
    assert RUNBOOK.exists(), 'Phase85 bounded reproduction runbook is missing'
    assert OUTPUT_JSON.exists(), 'Phase85 analysis JSON artifact is missing'
    assert MIN_SUMMARY.exists(), 'Phase85 minimal field summary is missing'
    assert CLEANUP_SUMMARY.exists(), 'Phase85 cleanup summary is missing'
    assert REPORT.exists(), 'Phase85 report is missing'

    data = json.loads(OUTPUT_JSON.read_text(encoding='utf-8'))
    assert data['classification'] in ALLOWED_CLASSIFICATIONS
    assert data['phase86_entered'] is False
    assert data['complete_autonomous_success_claimed'] is False
    assert data['exit_success_claimed'] is False
    assert data['guardrails']['no_strategy_or_config_tuning'] is True
    assert 'goal2_dispatch_context' in data

    report = REPORT.read_text(encoding='utf-8')
    for token in [
        'Phase85',
        'REFINEMENT_APPLIED_AND_GOAL2_IMPROVED',
        'REFINEMENT_APPLIED_BUT_GOAL2_STILL_TIMEOUT',
        'REFINEMENT_REJECTED_OR_NOT_TRIGGERED',
        'INSUFFICIENT_VALIDATION_EVIDENCE',
        'No maze_explorer strategy changed',
        'No Nav2/MPPI/controller tuning',
        'No autonomous exploration success claimed',
        'No exit success claimed',
        'Phase86 not entered',
    ]:
        assert token in report
