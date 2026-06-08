from pathlib import Path
import json
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[3]
WRAPPER = ROOT / 'tools' / 'run_phase52_startup_map_boundary_warmup.sh'
ANALYZER = ROOT / 'tools' / 'analyze_phase52_startup_map_boundary_warmup.py'
MAZE_EXPLORER = ROOT / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_explorer.py'
PHASE52_REPORT = ROOT / 'doc' / 'doc_report' / 'phase52_startup_map_boundary_warmup_report.md'


def _read(path: Path) -> str:
    assert path.exists(), f'missing {path}'
    return path.read_text(encoding='utf-8')


def test_phase52_maze_explorer_supports_no_dispatch_warmup_mode_before_goal_budget_exhaustion():
    source = _read(MAZE_EXPLORER)
    for token in [
        "declare_parameter('startup_warmup_no_dispatch'",
        'STARTUP_WARMUP_NO_DISPATCH',
        'startup_warmup_no_dispatch',
        "'startup_warmup_no_dispatch'",
    ]:
        assert token in source
    explore_body = source[source.index('def _explore_once'):source.index('def _dispatch_entry_readiness_gate')]
    assert 'startup_warmup_no_dispatch' in explore_body
    goal_budget_guard = "if self.goal_count >= self.max_goals and not self.startup_warmup_no_dispatch"
    assert goal_budget_guard in explore_body
    gate_guard_index = explore_body.index('if self.startup_warmup_no_dispatch:', explore_body.index("gate = self._dispatch_entry_readiness_gate(robot_pose)"))
    assert gate_guard_index > explore_body.index("gate = self._dispatch_entry_readiness_gate(robot_pose)")
    assert gate_guard_index < explore_body.index('self.mode = AT_NODE_ANALYZE')
    no_dispatch_block = explore_body[gate_guard_index:explore_body.index('self.mode = AT_NODE_ANALYZE')]
    assert '_analyze_and_dispatch' not in no_dispatch_block
    assert '_mark_exhausted' not in no_dispatch_block


def test_phase52_wrapper_is_bounded_uses_active_world_and_forbids_dispatch():
    text = _read(WRAPPER)
    for token in [
        'RUN_ID="phase52_startup_map_boundary_warmup"',
        'ARTIFACT_DIR="log/${RUN_ID}"',
        'tugbot_maze_world_20260528_clean_scaled2x.sdf',
        'maze_20260528_scaled_instance.yaml',
        'RUN_TIMEOUT_SEC="${PHASE52_RUN_TIMEOUT_SEC:-360}"',
        'RUNTIME_RECORDER_TIMEOUT_SEC="${PHASE52_RUNTIME_RECORDER_TIMEOUT_SEC:-330}"',
        'max_goals:="0"',
        'startup_warmup_no_dispatch:=true',
        'near_exit_fallback_enabled:=false',
        'dispatch_readiness_min_map_known_ratio:=0.70',
        'dispatch_readiness_min_map_free_ratio:=0.50',
        'clearance_radius_m:=0.38',
        'ros2 action info /navigate_to_pose',
        'ros2 topic info /goal_pose',
        'ros2 lifecycle get /controller_server',
        'tools/analyze_phase52_startup_map_boundary_warmup.py --record-runtime',
        'tools/analyze_phase52_startup_map_boundary_warmup.py --analyze',
        'cleanup_processes_after.txt',
    ]:
        assert token in text
    assert 'nav2_slam_phase' not in text
    assert 'candidate_costcritic' not in text


def test_phase52_analyzer_classifies_ready_without_dispatch(tmp_path):
    runtime = tmp_path / 'runtime.json'
    states = tmp_path / 'states.jsonl'
    events = tmp_path / 'events.jsonl'
    output = tmp_path / 'summary.json'
    action = tmp_path / 'action.txt'
    goal_pose = tmp_path / 'goal_pose.txt'
    lifecycle = tmp_path / 'lifecycle.txt'
    cleanup = tmp_path / 'cleanup.txt'

    action.write_text('/navigate_to_pose\nAction servers: 1\n', encoding='utf-8')
    goal_pose.write_text('Subscription count: 1\n', encoding='utf-8')
    lifecycle.write_text('/controller_server\nactive [3]\n/planner_server\nactive [3]\n/bt_navigator\nactive [3]\n', encoding='utf-8')
    cleanup.write_text('', encoding='utf-8')
    runtime.write_text(json.dumps({'snapshots': [
        {'elapsed_sec': 10.0, 'map': {'inclusive_near_robot': {'known_ratio': 0.44, 'free_ratio': 0.44, 'out_of_bounds_count': 481, 'in_bounds_count': 772}, 'width': 62, 'height': 243}, 'scan': {'finite_count': 322}, 'tf_lookups': {'map->base_link': {'available': True}}, 'local_costmap': {'inclusive_near_robot': {'known_ratio': 1.0, 'free_ratio': 0.95}}},
        {'elapsed_sec': 300.0, 'map': {'inclusive_near_robot': {'known_ratio': 0.75, 'free_ratio': 0.72, 'out_of_bounds_count': 0, 'in_bounds_count': 1253}, 'width': 82, 'height': 243}, 'scan': {'finite_count': 322}, 'tf_lookups': {'map->base_link': {'available': True}}, 'local_costmap': {'inclusive_near_robot': {'known_ratio': 1.0, 'free_ratio': 0.95}}},
    ]}), encoding='utf-8')
    states.write_text('\n'.join([
        json.dumps({'elapsed_sec': 20.0, 'state': {'mode': 'WAIT_FOR_DISPATCH_ENTRY_READINESS', 'goal_count': 0, 'dispatch_readiness_gate': {'passed': False, 'blocking_reasons': ['map_sufficient'], 'map': {'known_ratio': 0.44, 'free_ratio': 0.44, 'out_of_bounds_count': 481}}}}),
        json.dumps({'elapsed_sec': 301.0, 'state': {'mode': 'STARTUP_WARMUP_NO_DISPATCH', 'goal_count': 0, 'last_topology_sampling_diagnostics': {}, 'dispatch_readiness_gate': {'passed': True, 'blocking_reasons': [], 'map': {'known_ratio': 0.75, 'free_ratio': 0.72, 'out_of_bounds_count': 0}}}}),
    ]) + '\n', encoding='utf-8')
    events.write_text('', encoding='utf-8')

    proc = subprocess.run([
        sys.executable, str(ANALYZER), '--analyze',
        '--artifact-dir', str(tmp_path),
        '--runtime-evidence', str(runtime),
        '--explorer-state', str(states),
        '--goal-events', str(events),
        '--action-info', str(action),
        '--goal-pose-info', str(goal_pose),
        '--lifecycle-states', str(lifecycle),
        '--cleanup-processes-after', str(cleanup),
        '--output', str(output),
    ], text=True, capture_output=True, check=True)
    assert 'MAP_SUFFICIENCY_NATURALLY_READY_AFTER_WARMUP' in proc.stdout
    data = json.loads(output.read_text(encoding='utf-8'))
    assert data['classification'] == 'MAP_SUFFICIENCY_NATURALLY_READY_AFTER_WARMUP'
    assert data['dispatch_events'] == 0
    assert data['goal_events_samples'] == 0
    assert data['topology_sampling_occurred'] is False
    assert data['guardrails']['no_dispatch_goal'] is True


def test_phase52_analyzer_flags_dispatch_guardrail_violation(tmp_path):
    runtime = tmp_path / 'runtime.json'
    states = tmp_path / 'states.jsonl'
    events = tmp_path / 'events.jsonl'
    output = tmp_path / 'summary.json'
    empty = tmp_path / 'empty.txt'
    empty.write_text('', encoding='utf-8')
    runtime.write_text(json.dumps({'snapshots': []}), encoding='utf-8')
    states.write_text(json.dumps({'elapsed_sec': 1.0, 'state': {'mode': 'NAVIGATING', 'goal_count': 1}}) + '\n', encoding='utf-8')
    events.write_text(json.dumps({'event': 'dispatch'}) + '\n', encoding='utf-8')
    proc = subprocess.run([
        sys.executable, str(ANALYZER), '--analyze',
        '--artifact-dir', str(tmp_path),
        '--runtime-evidence', str(runtime),
        '--explorer-state', str(states),
        '--goal-events', str(events),
        '--action-info', str(empty),
        '--goal-pose-info', str(empty),
        '--lifecycle-states', str(empty),
        '--cleanup-processes-after', str(empty),
        '--output', str(output),
    ], text=True, capture_output=True, check=True)
    assert 'GUARDRAIL_VIOLATION_DISPATCH_OCCURRED' in proc.stdout


def test_phase52_report_exists_after_completion_with_required_classification_and_guardrails():
    report = _read(PHASE52_REPORT)
    for token in [
        'Phase52: Startup Map Boundary Warmup Bounded Runtime Validation',
        'MAP_SUFFICIENCY_NATURALLY_READY_AFTER_WARMUP',
        'MAP_BOUNDARY_STILL_BLOCKING_AFTER_WARMUP',
        'WARMUP_INCONCLUSIVE_RUNTIME_DATA_GAP',
        'GUARDRAIL_VIOLATION_DISPATCH_OCCURRED',
        'No autonomous exploration success claim',
        'No clearance strategy modification',
        'No Nav2 / MPPI / controller parameter tuning',
        'No map sufficiency threshold change',
    ]:
        assert token in report
