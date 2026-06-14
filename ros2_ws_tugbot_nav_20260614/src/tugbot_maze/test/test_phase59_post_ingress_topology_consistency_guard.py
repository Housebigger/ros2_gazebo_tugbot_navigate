from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
MAZE_EXPLORER = ROOT / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_explorer.py'
WRAPPER = ROOT / 'tools' / 'run_phase59_post_ingress_topology_consistency_guard.sh'
ANALYZER = ROOT / 'tools' / 'analyze_phase59_post_ingress_topology_consistency_guard.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase59_post_ingress_topology_consistency_guard_report.md'
RUN_ID = 'phase59_post_ingress_topology_consistency_guard'


def _read(path: Path) -> str:
    assert path.exists(), f'missing {path}'
    return path.read_text(encoding='utf-8')


def test_phase59_maze_explorer_declares_conservative_guard_parameters_and_state():
    source = _read(MAZE_EXPLORER)
    assert "WAIT_FOR_TOPOLOGY_CONSISTENCY = 'WAIT_FOR_TOPOLOGY_CONSISTENCY'" in source
    assert "declare_parameter('topology_consistency_enabled', True)" in source
    assert "declare_parameter('topology_consistency_required_no_candidate_frames', 2)" in source
    assert "declare_parameter('topology_consistency_window_sec'" in source
    assert 'self.topology_consistency_required_no_candidate_frames' in source
    assert 'self.topology_consistency_window_sec' in source
    assert 'self.topology_consistency_frames' in source
    assert 'self.topology_consistency_candidate_recovered' in source


def test_phase59_guard_delays_only_dead_end_policy_no_candidate_terminalization():
    source = _read(MAZE_EXPLORER)
    assert '_maybe_delay_no_candidate_terminalization' in source
    assert '_record_topology_consistency_frame' in source
    assert '_reset_topology_consistency_guard' in source
    assert '_topology_consistency_no_candidate_confirmed' in source
    assert 'dead_end_policy_no_branch_options' in source
    assert 'node_policy_no_untried_branch' in source
    assert 'terminalization_delayed' in source
    assert 'dead_end_policy_pending' in source
    assert 'dead_end_policy_confirmed' in source
    # A valid candidate must still dispatch normally and reset the guard rather than entering the wait state.
    dispatch_block = source[source.index('if chosen is not None:'):source.index('backtrack_target = self.topology.next_backtrack_target', source.index('if chosen is not None:'))]
    assert '_reset_topology_consistency_guard' in dispatch_block
    assert '_send_goal(chosen.target_xy' in dispatch_block


def test_phase59_structured_diagnostics_schema_in_state_and_goal_events():
    source = _read(MAZE_EXPLORER)
    for field in [
        'topology_consistency_enabled',
        'topology_consistency_required_no_candidate_frames',
        'topology_consistency_window_sec',
        'topology_consistency_frame_index',
        'topology_consistency_frames',
        'topology_consistency_frame_count',
        'raw_open_direction_count',
        'filtered_open_direction_count',
        'candidate_before_filter_count',
        'candidate_after_filter_count',
        'dead_end_policy_state',
        'terminalization_delayed',
        'candidate_recovered_during_consistency_window',
        'topology_consistency_guard_status',
        'topology_consistency_terminalization_reason',
    ]:
        assert field in source


def test_phase59_wrapper_and_analyzer_contract():
    wrapper = _read(WRAPPER)
    analyzer = _read(ANALYZER)
    joined = wrapper + '\n' + analyzer
    assert f'RUN_ID="{RUN_ID}"' in wrapper
    assert f"RUN_ID = '{RUN_ID}'" in analyzer
    assert f'log/${{RUN_ID}}' in wrapper
    assert 'tugbot_maze_world_20260528_clean_scaled2x.sdf' in joined
    assert 'maze_20260528_scaled_instance.yaml' in joined
    assert 'ingress_waypoint_map=x=${INGRESS_X},y=${INGRESS_Y},yaw=${INGRESS_YAW}' in wrapper
    assert '-p max_goals:=1' in wrapper
    assert '-p near_exit_fallback_enabled:=false' in wrapper
    assert '-p startup_warmup_no_dispatch:=false' in wrapper
    assert '-p topology_consistency_enabled:=true' in wrapper
    assert '-p topology_consistency_required_no_candidate_frames:=' in wrapper
    assert '-p topology_consistency_window_sec:=' in wrapper
    assert 'PHASE59_REPLAY_COUNT' in wrapper
    assert 'REPLAY_COUNT' in wrapper
    assert 'no Nav2/MPPI/controller parameter edits' in joined
    assert 'no clearance_radius_m tuning' in joined
    assert 'no map sufficiency threshold tuning' in joined
    assert 'no branch selection strategy change' in joined
    assert 'no fallback/terminal acceptance' in joined
    assert 'no autonomous exploration success claim' in joined
    assert 'first dispatch is not exit success' in joined


def test_phase59_analyzer_classifications_and_metrics():
    analyzer = _read(ANALYZER)
    for classification in [
        'TOPOLOGY_CONSISTENCY_GUARD_IMPLEMENTED_STATIC_ONLY',
        'TOPOLOGY_CONSISTENCY_RECOVERED_CANDIDATE_AND_DISPATCH',
        'TOPOLOGY_CONSISTENCY_CONFIRMED_NO_CANDIDATE',
        'TOPOLOGY_CONSISTENCY_INCONCLUSIVE_RUNTIME_VARIANCE',
        'GUARDRAIL_VIOLATION_STRATEGY_CHANGED',
    ]:
        assert classification in analyzer
    for metric in [
        'consistency_frames',
        'consistency_frame_count',
        'topology_consistency_frame_index',
        'raw_open_direction_count',
        'filtered_open_direction_count',
        'candidate_before_filter_count',
        'candidate_after_filter_count',
        'dead_end_policy_state',
        'terminalization_delayed',
        'candidate_recovered_during_consistency_window',
        'confirmed_no_candidate_frames',
        'dispatch_observed',
        'first_dispatch_observed_not_exit_success',
        'complete_autonomous_success_claimed',
    ]:
        assert metric in analyzer


def test_phase59_report_contract():
    report = _read(REPORT)
    assert 'Phase59' in report
    assert 'Post-Ingress Multi-frame Topology Consistency Guard' in report
    assert RUN_ID in report
    assert 'CANDIDATE_FORMATION_UNSTABLE_DEAD_END_POLICY_SENSITIVE' in report
    assert 'does not claim autonomous exploration success' in report or '不得写成自主探索成功' in report
    assert 'first dispatch is not exit success' in report or '不得把 first dispatch 写成出口成功' in report
