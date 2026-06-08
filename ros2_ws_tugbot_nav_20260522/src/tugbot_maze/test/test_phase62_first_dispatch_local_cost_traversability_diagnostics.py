from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
MAZE_EXPLORER = ROOT / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_explorer.py'
WRAPPER = ROOT / 'tools' / 'run_phase62_first_dispatch_local_cost_traversability_diagnostics.sh'
ANALYZER = ROOT / 'tools' / 'analyze_phase62_first_dispatch_local_cost_traversability_diagnostics.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase62_first_dispatch_local_cost_traversability_diagnostics_report.md'
RUN_ID = 'phase62_first_dispatch_local_cost_traversability_diagnostics'


def _read(path: Path) -> str:
    assert path.exists(), f'missing {path}'
    return path.read_text(encoding='utf-8')


def test_phase62_maze_explorer_dispatch_diagnostics_schema_is_read_only():
    source = _read(MAZE_EXPLORER)
    for field in [
        'phase62_first_dispatch_traversability',
        'phase62_local_costmap_patch',
        'phase62_target_footprint_cost',
        'phase62_front_wedge_clearance_m',
        'phase62_robot_to_target_progress',
        'phase62_cmd_vel_summary',
        'phase62_nav2_feedback_summary',
        'phase62_nav2_result_summary',
        'phase62_timestamp_consistency',
    ]:
        assert field in source
    assert 'create_subscription(Twist' not in source, 'Phase62 should use external runtime recorder for cmd_vel, not change explorer control path'
    assert 'send_goal_async(goal_msg, feedback_callback=' in source, 'Nav2 feedback must be recorded read-only with the action request'


def test_phase62_dispatch_local_patch_and_footprint_helpers_are_present_without_clearance_tuning():
    source = _read(MAZE_EXPLORER)
    for helper in [
        '_compute_phase62_first_dispatch_traversability_diagnostics',
        '_local_costmap_patch_for_point',
        '_local_cost_target_footprint_evidence',
        '_phase62_timestamp_consistency',
    ]:
        assert helper in source
    block = source[
        source.index('def _compute_phase62_first_dispatch_traversability_diagnostics'):
        source.index('def _compute_local_cost_diagnostics')
    ]
    for required in [
        'phase62_local_costmap_patch',
        'phase62_target_footprint_cost',
        'phase62_front_wedge_clearance_m',
        'phase62_target_cell_state',
        'phase62_local_cost_thresholds',
        'dispatch_target_local_cost_max_radius',
    ]:
        assert required in block
    assert "declare_parameter('clearance_radius_m'" in source
    assert "declare_parameter('clearance_radius_m', 0.35)" in source, 'Phase62 must not tune clearance_radius_m default'
    assert "declare_parameter('dispatch_readiness_min_map_known_ratio', 0.70)" in source, 'Phase62 must not tune map sufficiency threshold'


def test_phase62_wrapper_contract_bounded_ingress_then_explorer_with_execution_recorder():
    wrapper = _read(WRAPPER)
    assert f'RUN_ID="{RUN_ID}"' in wrapper
    assert 'tugbot_maze_world_20260528_clean_scaled2x.sdf' in wrapper
    assert 'maze_20260528_scaled_instance.yaml' in wrapper
    assert 'PHASE62_REPLAY_COUNT' in wrapper
    assert 'REPLAY_COUNT < 1 || REPLAY_COUNT > 2' in wrapper
    assert 'MAX_GOALS="1"' in wrapper
    assert '-p max_goals:=1' in wrapper
    assert '-p near_exit_fallback_enabled:=false' in wrapper
    assert '-p post_ingress_single_open_exception_enabled:=true' in wrapper
    assert 'start_first_dispatch_execution_recorder' in wrapper
    for artifact in [
        'first_dispatch_execution.jsonl',
        'controller_dynamics.jsonl',
        'nav2_feedback.jsonl',
        'local_costmap_samples.jsonl',
        'global_plan_samples.jsonl',
        'collision_monitor_state.jsonl',
        'first_dispatch_execution_summary.json',
    ]:
        assert artifact in wrapper
    for guardrail in [
        'no Nav2/MPPI/controller parameter edits',
        'no clearance_radius_m tuning',
        'no map sufficiency threshold tuning',
        'no branch selection scoring change',
        'no fallback/terminal acceptance',
        'no autonomous exploration success claim',
        'first dispatch is not exit success',
    ]:
        assert guardrail in wrapper


def test_phase62_analyzer_classifications_and_runtime_evidence_contract():
    analyzer = _read(ANALYZER)
    assert f"RUN_ID = '{RUN_ID}'" in analyzer
    for classification in [
        'TARGET_TOO_CLOSE_TO_WALL',
        'LOCAL_COSTMAP_INFLATION_DOMINANT',
        'CORRIDOR_TOO_NARROW',
        'SENSOR_COSTMAP_MISALIGNMENT',
        'INSUFFICIENT_EVIDENCE',
        'GUARDRAIL_VIOLATION_STRATEGY_CHANGED',
    ]:
        assert classification in analyzer
    for metric in [
        'local_costmap_patch_summary',
        'target_footprint_cost_summary',
        'front_wedge_clearance_summary',
        'robot_to_target_progress_summary',
        'cmd_vel_summary',
        'nav2_feedback_summary',
        'nav2_result_summary',
        'planner_controller_log_correlation',
        'timestamp_consistency_summary',
        'runtime_evidence_required',
    ]:
        assert metric in analyzer
    assert 'classify_phase62' in analyzer
    assert 'INSUFFICIENT_EVIDENCE' in analyzer and 'not dispatch_observed' in analyzer


def test_phase62_report_contract_preserves_phase61_and_stop_condition():
    report = _read(REPORT)
    assert 'Phase62' in report
    assert 'First Dispatch Local Cost / Traversability Diagnostics' in report
    assert RUN_ID in report
    assert 'SINGLE_OPEN_EXCEPTION_APPLIED_AND_DISPATCHED' in report
    assert 'not autonomous exploration success' in report or '不是自主探索成功' in report
    assert 'not exit success' in report or '不是出口成功' in report
    assert 'bounded runtime diagnostics' in report
    assert '不进入 Phase63' in report or 'Do not enter Phase63' in report
