from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
WRAPPER = ROOT / 'tools' / 'run_phase50_dispatch_entry_readiness_gate_bounded_runtime.sh'
ANALYZER = ROOT / 'tools' / 'analyze_phase50_dispatch_entry_readiness_gate_runtime.py'
MAZE_EXPLORER = ROOT / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_explorer.py'
PHASE49_REPORT = ROOT / 'doc' / 'doc_report' / 'phase49_dispatch_entry_readiness_gate_implementation_report.md'
PHASE49_TEST = ROOT / 'src' / 'tugbot_maze' / 'test' / 'test_phase49_dispatch_entry_readiness_gate.py'
ACTIVE_WORLD = 'tugbot_maze_world_20260528_clean_scaled2x.sdf'
ACTIVE_METADATA = 'maze_20260528_scaled_instance.yaml'
RUN_ID = 'phase50_dispatch_entry_readiness_gate_bounded_runtime'


def _read(path: Path) -> str:
    assert path.exists(), f'missing {path}'
    return path.read_text(encoding='utf-8')


def test_phase50_wrapper_exists_and_uses_bounded_active_scaled2x_runtime():
    source = _read(WRAPPER)
    assert 'set -euo pipefail' in source
    assert f'RUN_ID="{RUN_ID}"' in source
    assert 'ARTIFACT_DIR="log/${RUN_ID}"' in source
    assert ACTIVE_WORLD in source
    assert ACTIVE_METADATA in source
    assert 'RUN_TIMEOUT_SEC="${PHASE50_RUN_TIMEOUT_SEC:-180}"' in source
    assert 'if (( RUN_TIMEOUT_SEC < 120 || RUN_TIMEOUT_SEC > 240 )); then' in source
    assert 'MAX_GOALS="${PHASE50_MAX_GOALS:-1}"' in source
    assert 'goal_timeout_sec:="$GOAL_TIMEOUT_SEC"' in source
    assert 'headless:=true' in source
    assert 'explorer_type:=maze_dfs' in source
    assert 'near_exit_fallback_enabled:=false' in source
    assert 'entrance_x:="0.0"' in source
    assert 'entrance_y:="0.0"' in source
    assert 'exit_x:="21.072562"' in source
    assert 'exit_y:="18.083566"' in source
    assert 'active_truth_frame=map' in source


def test_phase50_wrapper_records_gate_and_full_runtime_evidence_without_strategy_or_nav2_tuning():
    source = _read(WRAPPER)
    for artifact in [
        'explorer_state.jsonl',
        'goal_events.jsonl',
        'phase50_runtime_evidence.json',
        'phase50_dispatch_entry_readiness_gate_bounded_runtime.json',
        'navigate_to_pose_action_info.txt',
        'goal_pose_topic_info.txt',
        'lifecycle_states.txt',
        'nav2_config_diff.txt',
        'cleanup_processes_after.txt',
    ]:
        assert artifact in source
    for command in [
        'record_explorer_state_series.py --topic /maze/explorer_state',
        'record_explorer_state_series.py --topic /maze/goal_events',
        'analyze_phase50_dispatch_entry_readiness_gate_runtime.py --record-runtime',
        'ros2 action info /navigate_to_pose',
        'ros2 topic info /goal_pose',
        'ros2 lifecycle get /controller_server',
        'ros2 lifecycle get /planner_server',
        'ros2 lifecycle get /bt_navigator',
    ]:
        assert command in source
    assert 'git diff -- src/tugbot_navigation/config | tee "$NAV2_CONFIG_DIFF"' in source
    launch_block = source[source.index('timeout --preserve-status "$RUN_TIMEOUT_SEC"'):source.index('2>&1 | tee "$LAUNCH_LOG"')]
    assert 'clearance_radius_m:=' not in launch_block
    assert 'params_file:="$NAV2_PARAMS"' in launch_block
    assert 'nav2_slam_params.yaml' in source
    assert 'nav2_slam_candidate' not in source


def test_phase50_maze_explorer_declares_lifecycle_nodes_as_string_to_accept_launch_override():
    source = _read(MAZE_EXPLORER)
    assert "'dispatch_readiness_required_lifecycle_nodes'" in source
    assert "'/controller_server,/planner_server,/bt_navigator'" in source
    assert "['/controller_server', '/planner_server', '/bt_navigator']" not in source[source.index("'dispatch_readiness_required_lifecycle_nodes'"):source.index('self.dispatch_readiness_required_lifecycle_nodes')]
    assert '_string_list_parameter(required_lifecycle_nodes_value)' in source


def test_phase50_analyzer_classifies_gate_timing_and_clearance_next_layer():
    source = _read(ANALYZER)
    for token in [
        'PHASE50_ALLOWED_CLASSIFICATIONS',
        'GATE_VALIDATED_TOPOLOGY_SAMPLING_AFTER_READY',
        'GATE_VALIDATED_CLEARANCE_GEOMETRY_NEXT_LAYER',
        'GATE_WAITING_FOR_READINESS_DATA',
        'GATE_BYPASS_OR_EARLY_TOPOLOGY_BUG',
        'INSUFFICIENT_RUNTIME_EVIDENCE',
        'dispatch_readiness_gate',
        'WAIT_FOR_DISPATCH_ENTRY_READINESS',
        'dispatch_readiness_gate_passed',
        'dispatch_readiness_blocking_reasons',
        'last_topology_sampling_diagnostics',
        'clearance_radius_blocked',
        'first_topology_after_gate_ready',
        'waited_before_ready',
        'complete_autonomous_success_claimed',
    ]:
        assert token in source


def test_phase50_analyzer_offline_synthetic_validates_clearance_geometry_next_layer(tmp_path):
    explorer = tmp_path / 'explorer_state.jsonl'
    goal_events = tmp_path / 'goal_events.jsonl'
    runtime = tmp_path / 'runtime.json'
    action = tmp_path / 'action.txt'
    goal_pose = tmp_path / 'goal_pose.txt'
    cleanup = tmp_path / 'cleanup.txt'
    lifecycle = tmp_path / 'lifecycle.txt'
    output = tmp_path / 'summary.json'

    rows = [
        {
            'seq': 1,
            'elapsed_sec': 2.0,
            'state': {
                'mode': 'WAIT_FOR_DISPATCH_ENTRY_READINESS',
                'dispatch_readiness_gate_passed': False,
                'dispatch_readiness_blocking_reasons': ['map_sufficient', 'local_costmap_sufficient'],
                'dispatch_readiness_gate': {'passed': False},
            },
        },
        {
            'seq': 2,
            'elapsed_sec': 30.0,
            'state': {
                'mode': 'AT_NODE_ANALYZE',
                'dispatch_readiness_gate_passed': True,
                'dispatch_readiness_blocking_reasons': [],
                'dispatch_readiness_gate': {'passed': True},
                'last_topology_sampling_diagnostics': {
                    'sampled_direction_count': 4,
                    'reject_reason_counts': {'clearance_radius_blocked': 4},
                    'raw_open_direction_count': 0,
                    'filtered_open_direction_count': 0,
                    'candidate_count': 0,
                },
            },
        },
    ]
    explorer.write_text('\n'.join(__import__('json').dumps(row) for row in rows), encoding='utf-8')
    goal_events.write_text('', encoding='utf-8')
    runtime.write_text('{"snapshots": [{"elapsed_sec": 5, "scan": {"available": true, "finite_count": 300}}]}', encoding='utf-8')
    action.write_text('/navigate_to_pose\nAction servers: 1\n', encoding='utf-8')
    goal_pose.write_text('Subscription count: 1\n', encoding='utf-8')
    cleanup.write_text('', encoding='utf-8')
    lifecycle.write_text('/controller_server\nactive [3]\n/planner_server\nactive [3]\n/bt_navigator\nactive [3]\n', encoding='utf-8')

    import subprocess, sys
    result = subprocess.run([
        sys.executable, str(ANALYZER), '--analyze',
        '--artifact-dir', str(tmp_path),
        '--output', str(output),
        '--explorer-state', str(explorer),
        '--goal-events', str(goal_events),
        '--runtime-evidence', str(runtime),
        '--action-info', str(action),
        '--goal-pose-info', str(goal_pose),
        '--lifecycle-states', str(lifecycle),
        '--cleanup-processes-after', str(cleanup),
    ], text=True, capture_output=True, check=True)
    assert 'GATE_VALIDATED_CLEARANCE_GEOMETRY_NEXT_LAYER' in result.stdout
    data = __import__('json').loads(output.read_text(encoding='utf-8'))
    assert data['classification'] == 'GATE_VALIDATED_CLEARANCE_GEOMETRY_NEXT_LAYER'
    assert data['waited_before_ready'] is True
    assert data['first_topology_after_gate_ready'] is True
    assert data['complete_autonomous_success_claimed'] is False


def test_phase49_report_explicitly_points_to_phase50_runtime_validation():
    report = _read(PHASE49_REPORT)
    assert 'Next runtime phase' in report
    assert 'WAIT_FOR_DISPATCH_ENTRY_READINESS' in report
    assert 'first topology sampling' in report
    assert 'No Gazebo / SLAM / Nav2 / maze_explorer runtime launched' in report
    assert 'DISPATCH_ENTRY_READINESS_GATE_IMPLEMENTED' in report
    assert PHASE49_TEST.exists()
