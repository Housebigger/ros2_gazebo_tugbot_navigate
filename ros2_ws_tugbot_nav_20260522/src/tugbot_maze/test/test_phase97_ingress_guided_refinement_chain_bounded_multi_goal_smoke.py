from __future__ import annotations

import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
ANALYZER = ROOT / 'tools' / 'analyze_phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke.py'
WRAPPER = ROOT / 'tools' / 'run_phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke.sh'
RECORDER = ROOT / 'tools' / 'record_phase97_smoke_evidence.py'
RUNBOOK = ROOT / 'doc' / 'doc_report' / 'phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke_runbook.md'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke_report.md'
RUN_ID = 'phase97_ingress_guided_refinement_chain_bounded_multi_goal_smoke'


def load_analyzer():
    spec = importlib.util.spec_from_file_location('phase97_analyzer', ANALYZER)
    assert spec and spec.loader, 'Phase97 analyzer must be importable'
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_json(path: Path, data: dict) -> None:
    path.write_text(json.dumps(data) + '\n', encoding='utf-8')


def write_jsonl(path: Path, rows: list[dict]) -> None:
    path.write_text('\n'.join(json.dumps({'state': row}) for row in rows) + '\n', encoding='utf-8')


def make_artifact(tmp_path: Path, ingress_success: bool = True) -> Path:
    artifact = tmp_path / RUN_ID
    artifact.mkdir(parents=True)
    write_json(artifact / f'{RUN_ID}_ingress_result.json', {'success': ingress_success, 'status': 'succeeded' if ingress_success else 'failed'})
    write_json(artifact / f'{RUN_ID}_raw_capture.json', {'scan': {}, 'map': {}, 'odom': {}, 'tf': {}, 'local_costmap': {}})
    return artifact


def dispatch(seq: int, *, staging: bool = False) -> dict:
    return {
        'event': 'dispatch',
        'goal_sequence': seq,
        'target': [seq + 1.0, seq + 2.0],
        'centerline_refinement_applied': True,
        'centerline_target_refinement': {
            'applied': True,
            'hard_safety_pass_candidate_count': 6,
            'multi_candidate_forward_search': {
                'candidate_count': 9,
                'hard_safety_pass_candidate_count': 6,
                'selected_candidate_target': [seq + 1.0, seq + 2.0],
                'selected_candidate_yaw': 1.57,
            },
        },
        'two_step_staging_plan': {'present': True, 'enabled': staging, 'second_step_forward_goal': None},
        'staging_applied': staging,
        'second_step_forward_goal': None,
        'branch_scoring_changed': False,
        'fallback_terminal_acceptance_used': False,
    }


def success(seq: int) -> dict:
    return {'event': 'success', 'goal_sequence': seq, 'robot_pose': [seq, seq, 0.0]}


def feedback(seq: int, recoveries: int = 0) -> dict:
    return {'goal_sequence': seq, 'distance_remaining': 0.05, 'number_of_recoveries': recoveries}


def local(seq: int) -> dict:
    return {
        'goal_sequence': seq,
        'front_wedge_cost': {'lethal_count': 0},
        'robot_footprint_cost': {'lethal_count': 0},
        'target_footprint_cost': {'lethal_count': 0},
        'front_wedge_clearance_m': 0.4,
    }


def test_phase97_analyzer_exposes_exact_ingress_guided_classification_set():
    analyzer = load_analyzer()
    assert analyzer.ALLOWED_CLASSIFICATIONS == {
        'INGRESS_GUIDED_REFINEMENT_CHAIN_BOUNDED_PASS',
        'INGRESS_GUIDED_REFINEMENT_CHAIN_GOAL_TIMEOUT',
        'INGRESS_GUIDED_REFINEMENT_CHAIN_RECOVERY_DOMINANT',
        'INGRESS_GUIDED_REFINEMENT_CHAIN_STAGING_TRIGGERED_NEEDS_REVIEW',
        'INGRESS_GUIDED_REFINEMENT_CHAIN_INSUFFICIENT_EVIDENCE',
        'INGRESS_GUIDED_MULTI_GOAL_INGRESS_FAILED',
    }


def test_phase97_two_successful_terminal_goals_classify_bounded_pass(tmp_path: Path):
    analyzer = load_analyzer()
    artifact = make_artifact(tmp_path)
    write_jsonl(artifact / f'{RUN_ID}_goal_events.jsonl', [dispatch(1), success(1), dispatch(2), success(2)])
    write_jsonl(artifact / f'{RUN_ID}_nav2_feedback.jsonl', [feedback(1), feedback(2)])
    write_jsonl(artifact / f'{RUN_ID}_local_costmap_samples.jsonl', [local(1), local(2)])
    result = analyzer.analyze_artifacts(artifact, run_id=RUN_ID, max_goals=3)
    assert result['classification'] == 'INGRESS_GUIDED_REFINEMENT_CHAIN_BOUNDED_PASS'
    assert result['observed_goal_sequences'] == [1, 2]
    assert result['per_goal']['1']['refinement_applied'] is True
    assert result['per_goal']['1']['multi_candidate_forward_search']['present'] is True
    assert result['per_goal']['1']['hard_safety_pass_candidate_count'] == 6
    assert result['per_goal']['1']['terminal_outcome'] == 'succeeded'


def test_phase97_ingress_failure_short_circuits_multi_goal_analysis(tmp_path: Path):
    analyzer = load_analyzer()
    artifact = make_artifact(tmp_path, ingress_success=False)
    result = analyzer.analyze_artifacts(artifact, run_id=RUN_ID, max_goals=3)
    assert result['classification'] == 'INGRESS_GUIDED_MULTI_GOAL_INGRESS_FAILED'
    assert result['ingress']['success'] is False


def test_phase97_staging_goal_is_needs_review(tmp_path: Path):
    analyzer = load_analyzer()
    artifact = make_artifact(tmp_path)
    write_jsonl(artifact / f'{RUN_ID}_goal_events.jsonl', [dispatch(1, staging=True), success(1), dispatch(2), success(2)])
    write_jsonl(artifact / f'{RUN_ID}_nav2_feedback.jsonl', [feedback(1), feedback(2)])
    write_jsonl(artifact / f'{RUN_ID}_local_costmap_samples.jsonl', [local(1), local(2)])
    result = analyzer.analyze_artifacts(artifact, run_id=RUN_ID, max_goals=3)
    assert result['classification'] == 'INGRESS_GUIDED_REFINEMENT_CHAIN_STAGING_TRIGGERED_NEEDS_REVIEW'


def test_phase97_wrapper_runbook_guard_bounded_ingress_flow_and_terminal_wait():
    for path in [WRAPPER, RUNBOOK, REPORT]:
        assert path.exists(), f'{path} must exist'
    text = WRAPPER.read_text() + RUNBOOK.read_text() + REPORT.read_text()
    assert 'explicit inner-ingress Nav2 goal before maze_explorer' in text
    assert 'max_goals:=2~3' in text
    assert 'wait_for_terminal_set_or_failure' in text
    assert '/navigate_to_pose' in text and '/map' in text and '/scan' in text and '/local_costmap/costmap' in text
    assert 'TF ready' in text or 'tf ready' in text
    assert 'No Phase88/92 refinement logic changed' in text
    assert 'No branch scoring/exploration order/centerline gate/directional readiness/fallback/terminal acceptance changed' in text
    assert 'No Nav2/MPPI/controller/inflation/robot_radius/clearance_radius_m/map threshold tuning' in text
    assert 'No autonomous exploration success claimed' in text
    assert 'No exit success claimed' in text
    assert 'Phase98 not entered' in text


def test_phase97_recorder_is_read_only_and_captures_required_topics():
    assert RECORDER.exists(), 'Phase97 raw evidence recorder must exist'
    text = RECORDER.read_text()
    for topic in ['/maze/goal_events', '/scan', '/map', '/local_costmap/costmap', '/local_costmap/published_footprint', '/odom']:
        assert topic in text
    for frame_name in ['map', 'base_link', 'odom']:
        assert frame_name in text
    assert 'ActionClient' not in text
    assert 'send_goal' not in text
    assert 'set_parameters' not in text
