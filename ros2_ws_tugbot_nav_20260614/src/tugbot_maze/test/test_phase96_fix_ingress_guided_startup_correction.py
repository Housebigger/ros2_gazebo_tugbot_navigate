from __future__ import annotations

import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
ANALYZER = ROOT / 'tools' / 'analyze_phase96_fix_ingress_guided_startup_correction.py'
WRAPPER = ROOT / 'tools' / 'run_phase96_fix_ingress_guided_startup_correction.sh'
RUNBOOK = ROOT / 'doc' / 'doc_report' / 'phase96_fix_ingress_guided_startup_correction_runbook.md'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase96_fix_ingress_guided_startup_correction_report.md'
RECORDER = ROOT / 'tools' / 'record_phase96_fix_smoke_evidence.py'
RUN_ID = 'phase96_fix_ingress_guided_startup_correction'


def load_analyzer():
    assert ANALYZER.exists(), 'Phase96-fix analyzer must exist before tests can import it'
    spec = importlib.util.spec_from_file_location('phase96_fix_analyzer', ANALYZER)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_jsonl(path: Path, rows: list[dict]) -> None:
    path.write_text('\n'.join(json.dumps(row, sort_keys=True) for row in rows) + '\n')


def event(seq: int, name: str) -> dict:
    return {'state': {'event': name, 'goal_sequence': seq, 'goal_kind': 'explore'}}


def test_phase96_fix_analyzer_contract_classifies_dispatch_after_ingress(tmp_path):
    analyzer = load_analyzer()
    artifact = tmp_path / RUN_ID
    artifact.mkdir()
    (artifact / f'{RUN_ID}_ingress_result.json').write_text(json.dumps({
        'status': 'succeeded',
        'target': [2.0, 0.0, 0.0],
        'distance_to_goal_m': 0.08,
    }))
    write_jsonl(artifact / f'{RUN_ID}_goal_events.jsonl', [event(1, 'dispatch')])
    (artifact / f'{RUN_ID}_raw_capture.json').write_text(json.dumps({
        'scan': {'ranges_count': 900},
        'map': {'summary': {'known_count': 100}},
        'local_costmap': {'summary': {'lethal_count': 10}},
        'odom': {'pose': {'x': 1.95, 'y': 0.02, 'yaw': 0.0}},
        'tf': {'map->base_link': {'available': True}},
    }))

    result = analyzer.analyze_artifacts(artifact, run_id=RUN_ID, max_goals=3)

    assert result['classification'] == 'INGRESS_GUIDED_DISPATCH_OBSERVED'
    assert result['ingress']['status'] == 'succeeded'
    assert result['dispatch_observed'] is True
    assert result['observed_goal_sequences'] == [1]
    assert result['evidence_gaps'] == []
    assert result['guardrails']['algorithm_changed'] is False
    assert result['guardrails']['nav2_config_changed'] is False


def test_phase96_fix_analyzer_classifies_still_blocked_when_ingress_succeeds_but_no_dispatch(tmp_path):
    analyzer = load_analyzer()
    artifact = tmp_path / RUN_ID
    artifact.mkdir()
    (artifact / f'{RUN_ID}_ingress_result.json').write_text(json.dumps({'status': 'succeeded', 'target': [2.0, 0.0, 0.0]}))
    write_jsonl(artifact / f'{RUN_ID}_goal_events.jsonl', [])
    write_jsonl(artifact / f'{RUN_ID}_explorer_state.jsonl', [{
        'state': {
            'mode': 'WAIT_FOR_DISPATCH_ENTRY_READINESS',
            'dispatch_readiness_blocking_reasons': ['map_sufficient'],
            'dispatch_readiness_gate': {'checks': {'map_sufficient': False, 'scan_sufficient': True, 'tf_sufficient': True}},
        }
    }])
    (artifact / f'{RUN_ID}_raw_capture.json').write_text(json.dumps({
        'scan': {'ranges_count': 900},
        'map': {'summary': {'known_count': 5}},
        'local_costmap': {'summary': {'lethal_count': 10}},
        'odom': {'pose': {'x': 2.0, 'y': 0.0, 'yaw': 0.0}},
        'tf': {'map->base_link': {'available': True}},
    }))

    result = analyzer.analyze_artifacts(artifact, run_id=RUN_ID, max_goals=3)

    assert result['classification'] == 'INGRESS_GUIDED_DISPATCH_STILL_BLOCKED'
    assert result['dispatch_observed'] is False
    assert result['last_explorer_state']['mode'] == 'WAIT_FOR_DISPATCH_ENTRY_READINESS'
    assert result['last_explorer_state']['blocking_reasons'] == ['map_sufficient']


def test_phase96_fix_wrapper_sends_ingress_before_maze_explorer_and_preserves_guards():
    assert WRAPPER.exists(), 'Phase96-fix wrapper must exist'
    text = WRAPPER.read_text()
    assert 'PHASE96_FIX_INGRESS_X' in text
    assert 'send_ingress_goal' in text
    assert 'wait_for_ingress_terminal' in text
    assert 'start_explorer' in text
    assert text.index('wait_for_ingress_terminal') < text.index('start_explorer')
    assert 'ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose' in text
    assert 'PHASE96_FIX_MAX_GOALS must be 2 or 3' in text
    assert 'max_goals:="$MAX_GOALS"' in text
    for forbidden in ['inflation_radius:', 'robot_radius:', 'clearance_radius_m:=', 'nav2_slam_params.yaml <<']:
        assert forbidden not in text


def test_phase96_fix_recorder_is_read_only_and_captures_map_scan_costmap_odom_tf():
    assert RECORDER.exists(), 'Phase96-fix raw evidence recorder must exist'
    text = RECORDER.read_text()
    for topic in ['/maze/goal_events', '/scan', '/map', '/local_costmap/costmap', '/local_costmap/published_footprint', '/odom']:
        assert topic in text
    for frame_name in ['map', 'base_link', 'odom']:
        assert frame_name in text
    assert 'ActionClient' not in text
    assert 'send_goal' not in text
    assert 'set_parameters' not in text
