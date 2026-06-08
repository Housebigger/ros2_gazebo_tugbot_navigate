from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
WRAPPER = ROOT / 'tools' / 'run_phase43_initial_topology_sampling_diagnostics.sh'
ANALYZER = ROOT / 'tools' / 'analyze_phase43_initial_topology_sampling_diagnostics.py'
MAZE_EXPLORER = ROOT / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_explorer.py'
PHASE42_WRAPPER = ROOT / 'tools' / 'run_phase42_map_frame_truth_bounded_smoke.sh'
ACTIVE_WORLD = 'tugbot_maze_world_20260528_clean_scaled2x.sdf'
ACTIVE_METADATA = 'maze_20260528_scaled_instance.yaml'
RUN_ID = 'phase43_initial_topology_sampling_diagnostics'


def _read(path: Path) -> str:
    assert path.exists(), f'missing {path}'
    return path.read_text(encoding='utf-8')


def test_phase43_wrapper_exists_and_uses_independent_run_id_and_artifact_dir():
    source = _read(WRAPPER)
    assert 'set -euo pipefail' in source
    assert f'RUN_ID="{RUN_ID}"' in source
    assert 'ARTIFACT_DIR="log/${RUN_ID}"' in source
    assert 'Phase43 Initial Topology Sampling Diagnostics' in source
    assert 'run_phase42_map_frame_truth_bounded_smoke.sh' not in source


def test_phase43_wrapper_reuses_phase42_truth_but_is_bounded_diagnostics_not_long_run():
    source = _read(WRAPPER)
    assert ACTIVE_WORLD in source
    assert ACTIVE_METADATA in source
    for token in [
        'entrance_x:="0.0"',
        'entrance_y:="0.0"',
        'entrance_yaw:="0.0"',
        'exit_x:="21.072562"',
        'exit_y:="18.083566"',
        'exit_radius:="1.2"',
        'active_truth_frame=map',
        'near_exit_fallback_enabled:=false',
        'max_goals:="$MAX_GOALS"',
        'MAX_GOALS="${PHASE43_MAX_GOALS:-4}"',
    ]:
        assert token in source
    assert 'RUN_TIMEOUT_SEC="${PHASE43_RUN_TIMEOUT_SEC:-240}"' in source
    assert 'if (( RUN_TIMEOUT_SEC < 120 || RUN_TIMEOUT_SEC > 300 )); then' in source
    launch_block = source[source.index('timeout --preserve-status "$RUN_TIMEOUT_SEC"'):source.index('2>&1 | tee "$LAUNCH_LOG"')]
    for legacy_token in ['entrance_x:="-4.0"', 'entrance_y:="-3.0"', 'exit_x:="4.0"', 'exit_y:="3.0"', 'exit_radius:="0.6"', 'tugbot_maze_world.sdf', 'maze_instance.yaml']:
        assert legacy_token not in launch_block


def test_phase43_wrapper_records_full_startup_topology_evidence_and_cleanup():
    source = _read(WRAPPER)
    for artifact in [
        'launch.log',
        'explorer_state.jsonl',
        'goal_events.jsonl',
        'phase43_initial_topology_full_data.json',
        'phase43_initial_topology_sampling_diagnostics.json',
        'map_scan_odom_tf_costmap_samples.json',
        'first_topology_sampling_snapshot.json',
        'navigate_to_pose_action_info.txt',
        'nav2_config_diff.txt',
        'cleanup_processes_after.txt',
    ]:
        assert artifact in source
    assert 'tools/analyze_phase43_initial_topology_sampling_diagnostics.py --record-runtime' in source
    assert 'record_explorer_state_series.py --topic /maze/explorer_state' in source
    assert 'record_explorer_state_series.py --topic /maze/goal_events' in source
    assert 'ros2 action info /navigate_to_pose' in source
    assert 'trap cleanup EXIT' in source
    for cleanup_token in ['maze_explorer', 'maze_goal_monitor', 'ros_gz_bridge', 'slam_toolbox', 'controller_server', 'planner_server', 'bt_navigator', 'gz sim']:
        assert cleanup_token in source


def test_phase43_analyzer_declares_required_classifications_and_acceptance_fields():
    source = _read(ANALYZER)
    for token in [
        'PHASE43_ALLOWED_CLASSIFICATIONS',
        'INSUFFICIENT_INITIAL_MAP_OR_SCAN',
        'FRAME_MISMATCH',
        'ROBOT_NOT_AT_ENTRANCE',
        'COSTMAP_BLOCKED_OR_UNKNOWN',
        'LASER_SCAN_EMPTY_OR_INVALID',
        'TOPOLOGY_REJECTION_CAUSE_IDENTIFIED',
        'INCONCLUSIVE_NEEDS_TARGETED_RUNTIME_CAPTURE',
        'active_map_frame_truth_used',
        'first_topology_sampling_evidence_captured',
        'rejection_reason_present_or_explained',
        'nav2_action_server_available',
        'cleanup_empty',
        'complete_autonomous_success_claimed',
    ]:
        assert token in source


def test_phase43_maze_explorer_only_adds_readonly_topology_sampling_diagnostics():
    source = _read(MAZE_EXPLORER)
    for token in [
        'last_topology_sampling_diagnostics',
        '_build_topology_sampling_diagnostics',
        'sampled_direction',
        'sampled_endpoint',
        'map_cell_state',
        'clearance_result',
        'costmap_lethal_or_unknown_result',
        'tf_lookup_result',
        'reject_reason',
    ]:
        assert token in source
    analyze_body = source[source.index('def _analyze_and_dispatch'):source.index('def _empty_branch_choice_diagnostics')]
    assert analyze_body.index('local = classify_local_topology') < analyze_body.index('self.last_topology_sampling_diagnostics = self._build_topology_sampling_diagnostics') < analyze_body.index('chosen = self.topology.choose_next_branch')
    assert 'self.last_topology_sampling_diagnostics' in source[source.index('def _state_payload'):]


def test_phase42_wrapper_is_not_overwritten_by_phase43_run_id():
    source = _read(PHASE42_WRAPPER)
    assert 'RUN_ID="phase42_map_frame_truth_bounded_smoke"' in source
    assert RUN_ID not in source
