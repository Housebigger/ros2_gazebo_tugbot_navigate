from __future__ import annotations

import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
ANALYZER = ROOT / 'tools' / 'analyze_phase96_refinement_chain_bounded_multi_goal_smoke.py'
WRAPPER = ROOT / 'tools' / 'run_phase96_refinement_chain_bounded_multi_goal_smoke.sh'
RECORDER = ROOT / 'tools' / 'record_phase96_smoke_evidence.py'
RUNBOOK = ROOT / 'doc' / 'doc_report' / 'phase96_refinement_chain_bounded_multi_goal_smoke_runbook.md'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase96_refinement_chain_bounded_multi_goal_smoke_report.md'
RUN_ID = 'phase96_refinement_chain_bounded_multi_goal_smoke'


def load_analyzer():
    assert ANALYZER.exists(), 'Phase96 analyzer must exist before tests can import it'
    spec = importlib.util.spec_from_file_location('phase96_analyzer', ANALYZER)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_jsonl(path: Path, rows: list[dict]) -> None:
    path.write_text('\n'.join(json.dumps(row, sort_keys=True) for row in rows) + '\n')


def event_row(seq: int, event: str, *, refinement: bool = False, hard: int = 0, staging: bool = False, reject: str = 'trigger_bundle_not_satisfied') -> dict:
    ref = {
        'refinement_applied': refinement,
        'multi_candidate_forward_search': {
            'candidate_family': {'centerline_projection': True, 'bounded_local_search': True},
            'candidate_count': 21,
            'selected_candidate_index': 4 if refinement else None,
            'selected_candidate_target': [1.9 + seq * 0.1, 2.0] if refinement else None,
            'selected_candidate_yaw': 1.57 if refinement else None,
            'selection_priority_trace': ['hard safety pass', 'no footprint/front-wedge lethal regression'],
        },
        'hard_safety_pass_candidate_count': hard,
        'selected_candidate_target': [1.9 + seq * 0.1, 2.0] if refinement else None,
        'selected_candidate_yaw': 1.57 if refinement else None,
    }
    return {
        'elapsed_sec': float(seq * 10),
        'state': {
            'event': event,
            'goal_sequence': seq,
            'goal_kind': 'explore',
            'target': [2.0 + seq, 1.0 + seq],
            'centerline_target_refinement': ref,
            'two_step_staging_plan': {
                'eligible': staging,
                'staging_applied': staging,
                'staging_goal_pose': [1.5, 1.5, 0.0] if staging else None,
                'staging_reject_reason': None if staging else reject,
            },
            'staging_applied': staging,
            'staging_reject_reason': None if staging else reject,
            'second_step_forward_goal': {'target': [2.5, 2.5], 'fresh_evidence': True} if staging else None,
            'branch_scoring_changed': False,
            'fallback_terminal_acceptance_used': False,
            'result_status': 4 if event == 'success' else (6 if event in {'failure', 'timeout'} else None),
        },
    }


def feedback_rows(seq: int, recoveries: int = 0) -> list[dict]:
    rows = []
    for idx, dist in enumerate([1.1, 0.8, 0.3, 0.05]):
        rows.append({
            'event': 'nav2_feedback',
            'goal_sequence': seq,
            'elapsed_sec': seq * 10 + idx,
            'distance_remaining': dist,
            'number_of_recoveries': recoveries if idx >= 2 else 0,
            'navigation_time_sec': idx + 0.1,
        })
    return rows


def local_rows(seq: int, lethal: int = 0) -> list[dict]:
    return [{
        'event': 'local_costmap_sample',
        'goal_sequence': seq,
        'elapsed_sec': seq * 10 + 0.5,
        'robot_pose': [1.0 + seq, 2.0, 0.0],
        'dispatch_target': [2.0 + seq, 3.0],
        'front_wedge_clearance_m': 0.5,
        'front_wedge_cost': {'lethal_count': lethal, 'high_cost_count': lethal, 'max': 99 if lethal else 46},
        'robot_footprint_cost': {'lethal_count': 0, 'high_cost_count': 0, 'max': 46},
        'target_footprint_cost': {'lethal_count': 0, 'high_cost_count': 0, 'max': 46},
        'local_costmap_target_evidence': {'available': True, 'in_bounds': True, 'value': 0},
    }]


def make_artifact_dir(tmp_path: Path, *, events: list[dict], recoveries: dict[int, int] | None = None, include_required_evidence: bool = True) -> Path:
    recoveries = recoveries or {}
    artifact = tmp_path / RUN_ID
    artifact.mkdir(parents=True)
    write_jsonl(artifact / f'{RUN_ID}_goal_events.jsonl', events)
    if include_required_evidence:
        feedback = []
        locals_ = []
        seqs = sorted({int(row['state']['goal_sequence']) for row in events})
        for seq in seqs:
            feedback.extend(feedback_rows(seq, recoveries=recoveries.get(seq, 0)))
            locals_.extend(local_rows(seq, lethal=seq - 1))
        write_jsonl(artifact / f'{RUN_ID}_nav2_feedback.jsonl', feedback)
        write_jsonl(artifact / f'{RUN_ID}_local_costmap_samples.jsonl', locals_)
        (artifact / f'{RUN_ID}_raw_capture.json').write_text(json.dumps({
            'scan': {'ranges_count': 720},
            'odom': {'pose': {'x': 1.0, 'y': 2.0, 'yaw': 0.0}},
            'tf': {'map->base_link': {'available': True}},
            'local_costmap': {'summary': {'lethal_count': 5}},
        }))
    return artifact


def test_phase96_analyzer_contract_exposes_required_classifications_and_fields():
    analyzer = load_analyzer()
    assert analyzer.ALLOWED_CLASSIFICATIONS == {
        'REFINEMENT_CHAIN_BOUNDED_SMOKE_PASS',
        'REFINEMENT_CHAIN_GOAL_TIMEOUT',
        'REFINEMENT_CHAIN_RECOVERY_DOMINANT',
        'REFINEMENT_CHAIN_STAGING_TRIGGERED_NEEDS_REVIEW',
        'REFINEMENT_CHAIN_INSUFFICIENT_EVIDENCE',
    }
    for field in [
        'refinement_applied',
        'multi_candidate_forward_search',
        'hard_safety_pass_candidate_count',
        'two_step_staging_plan',
        'staging_applied',
        'second_step_forward_goal',
        'terminal_outcome',
        'recoveries',
        'local_cost_risk',
    ]:
        assert field in analyzer.REQUIRED_GOAL_FIELDS


def test_phase96_analyzer_classifies_two_goal_successful_refinement_chain_smoke(tmp_path):
    analyzer = load_analyzer()
    artifact = make_artifact_dir(tmp_path, events=[
        event_row(1, 'dispatch', refinement=False, hard=0),
        event_row(1, 'success', refinement=False, hard=0),
        event_row(2, 'dispatch', refinement=True, hard=17, reject='single_step_forward_search_had_hard_safe_candidate'),
        event_row(2, 'success', refinement=True, hard=17, reject='single_step_forward_search_had_hard_safe_candidate'),
    ])

    result = analyzer.analyze_artifacts(artifact, run_id=RUN_ID, max_goals=3)

    assert result['classification'] == 'REFINEMENT_CHAIN_BOUNDED_SMOKE_PASS'
    assert result['observed_goal_count'] == 2
    assert result['evidence_gaps'] == []
    assert result['per_goal']['2']['refinement_applied'] is True
    assert result['per_goal']['2']['hard_safety_pass_candidate_count'] == 17
    assert result['per_goal']['2']['terminal_outcome'] == 'succeeded'
    assert result['per_goal']['2']['recoveries']['max'] == 0
    assert result['guardrails']['algorithm_changed'] is False


def test_phase96_analyzer_flags_staging_triggered_for_review(tmp_path):
    analyzer = load_analyzer()
    artifact = make_artifact_dir(tmp_path, events=[
        event_row(1, 'dispatch', refinement=False, hard=0),
        event_row(1, 'success', refinement=False, hard=0),
        event_row(2, 'dispatch', refinement=False, hard=0, staging=True),
        event_row(2, 'success', refinement=False, hard=0, staging=True),
    ])

    result = analyzer.analyze_artifacts(artifact, run_id=RUN_ID, max_goals=3)

    assert result['classification'] == 'REFINEMENT_CHAIN_STAGING_TRIGGERED_NEEDS_REVIEW'
    assert result['per_goal']['2']['staging_applied'] is True
    assert result['per_goal']['2']['second_step_forward_goal'] is not None


def test_phase96_analyzer_flags_timeout_and_recovery_dominant_separately(tmp_path):
    analyzer = load_analyzer()
    timeout_artifact = make_artifact_dir(tmp_path / 'timeout', events=[
        event_row(1, 'dispatch'),
        event_row(1, 'timeout'),
    ])
    recovery_artifact = make_artifact_dir(tmp_path / 'recovery', events=[
        event_row(1, 'dispatch'),
        event_row(1, 'success'),
        event_row(2, 'dispatch', refinement=True, hard=4),
        event_row(2, 'success', refinement=True, hard=4),
    ], recoveries={2: 5})

    timeout_result = analyzer.analyze_artifacts(timeout_artifact, run_id=RUN_ID, max_goals=3)
    recovery_result = analyzer.analyze_artifacts(recovery_artifact, run_id=RUN_ID, max_goals=3)

    assert timeout_result['classification'] == 'REFINEMENT_CHAIN_GOAL_TIMEOUT'
    assert timeout_result['per_goal']['1']['terminal_outcome'] == 'timeout'
    assert recovery_result['classification'] == 'REFINEMENT_CHAIN_RECOVERY_DOMINANT'
    assert recovery_result['per_goal']['2']['recoveries']['max'] == 5


def test_phase96_analyzer_reports_insufficient_evidence_when_required_streams_missing(tmp_path):
    analyzer = load_analyzer()
    artifact = make_artifact_dir(tmp_path, events=[
        event_row(1, 'dispatch'),
        event_row(1, 'success'),
    ], include_required_evidence=False)

    result = analyzer.analyze_artifacts(artifact, run_id=RUN_ID, max_goals=3)

    assert result['classification'] == 'REFINEMENT_CHAIN_INSUFFICIENT_EVIDENCE'
    assert 'nav2_feedback_missing' in result['evidence_gaps']
    assert 'local_costmap_samples_missing' in result['evidence_gaps']
    assert 'raw_scan_odom_tf_capture_missing' in result['evidence_gaps']


def test_phase96_wrapper_runbook_and_report_preserve_bounded_validation_guardrails():
    for path in [WRAPPER, RUNBOOK, REPORT]:
        assert path.exists(), f'{path} must exist'
    wrapper = WRAPPER.read_text()
    runbook = RUNBOOK.read_text()
    report = REPORT.read_text()

    assert 'PHASE96_MAX_GOALS must be 2 or 3' in wrapper
    assert 'max_goals:=2~3' in runbook
    assert 'No maze_explorer strategy changed' in report
    assert 'No Phase88/92 logic changed' in report
    assert 'No Nav2/MPPI/controller tuning' in report
    assert 'No inflation/robot_radius/clearance_radius_m/map threshold tuning' in report
    assert 'No autonomous exploration success claimed' in report
    assert 'No exit success claimed' in report
    assert 'Phase97 not entered' in report
    forbidden = ['nav2_slam_params.yaml <<', 'inflation_radius:', 'robot_radius:', 'clearance_radius_m:=']
    assert not any(token in wrapper for token in forbidden)


def test_phase96_recorder_is_read_only_and_captures_required_topics():
    assert RECORDER.exists(), 'Phase96 recorder must exist'
    text = RECORDER.read_text()
    for topic in ['/maze/goal_events', '/scan', '/local_costmap/costmap', '/local_costmap/published_footprint', '/odom']:
        assert topic in text
    for frame_pair in ['map', 'base_link', 'odom']:
        assert frame_pair in text
    assert 'ActionClient' not in text
    assert 'send_goal' not in text
    assert 'set_parameters' not in text
