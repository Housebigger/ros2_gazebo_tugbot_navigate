from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
MAZE_CONFIG = ROOT / 'src' / 'tugbot_maze' / 'config' / 'maze_20260528_scaled_instance.yaml'
WRAPPER = ROOT / 'tools' / 'run_phase65_ingress_goal_depth_adjustment_inner_staging.sh'
ANALYZER = ROOT / 'tools' / 'analyze_phase65_ingress_goal_depth_adjustment_inner_staging.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase65_ingress_goal_depth_adjustment_inner_staging_report.md'
NAV2_CONFIG = ROOT / 'src' / 'tugbot_navigation' / 'config' / 'nav2_slam_params.yaml'
RUN_ID = 'phase65_ingress_goal_depth_adjustment_inner_staging'


def _read(path: Path) -> str:
    assert path.exists(), f'missing {path}'
    return path.read_text(encoding='utf-8')


def test_phase65_metadata_preserves_old_ingress_and_declares_inner_staging_candidate():
    config = _read(MAZE_CONFIG)
    assert 'ingress_waypoint_map:' in config
    assert 'x_m: 1.0' in config
    assert 'phase53_selected_original_ingress_waypoint_map' in config
    assert 'phase65_inner_ingress_waypoint_map' in config
    assert 'phase65_inner_ingress_depth_adjustment_m' in config
    assert 'entry_staging_hypothesis: ENTRY_STAGING_TOO_SHALLOW' in config
    assert 'x_m: 2.0' in config
    assert 'yaw_rad: 0.0' in config


def test_phase65_wrapper_contract_is_bounded_inner_ingress_then_explorer():
    wrapper = _read(WRAPPER)
    assert f'RUN_ID="{RUN_ID}"' in wrapper
    assert 'tugbot_maze_world_20260528_clean_scaled2x.sdf' in wrapper
    assert 'maze_20260528_scaled_instance.yaml' in wrapper
    assert 'ACTIVE_INGRESS_X="1.0"' in wrapper
    assert 'INNER_INGRESS_X="2.0"' in wrapper
    assert 'send_inner_ingress_goal' in wrapper
    assert 'ingress_waypoint_map=x=${INNER_INGRESS_X},y=${INNER_INGRESS_Y},yaw=${INNER_INGRESS_YAW}' in wrapper
    assert '-p max_goals:=1' in wrapper
    assert 'MAX_GOALS="1"' in wrapper
    assert '-p near_exit_fallback_enabled:=false' in wrapper
    assert '-p startup_warmup_no_dispatch:=false' in wrapper
    assert 'record_explorer_state_series.py --topic /maze/goal_events' in wrapper
    assert 'first_dispatch_execution_summary.json' in wrapper
    for guardrail in [
        'no Nav2/MPPI/controller parameter edits',
        'no clearance_radius_m tuning',
        'no map sufficiency threshold tuning',
        'no branch selection scoring change',
        'no target projection integration',
        'no fallback/terminal acceptance change',
        'no autonomous exploration success claim',
        'no exit success claim',
        'bounded runtime only',
        'max_goals=1',
    ]:
        assert guardrail in wrapper


def test_phase65_analyzer_schema_and_classifications():
    analyzer = _read(ANALYZER)
    assert f"RUN_ID = '{RUN_ID}'" in analyzer
    for classification in [
        'INNER_INGRESS_FIX_IMPROVES_FIRST_DISPATCH',
        'ENTRY_STAGING_TOO_SHALLOW_CONFIRMED',
        'NO_IMPROVEMENT_GEOMETRY_STILL_BLOCKED',
        'INSUFFICIENT_EVIDENCE',
        'GUARDRAIL_VIOLATION_STRATEGY_CHANGED',
    ]:
        assert classification in analyzer
    for field in [
        'old_ingress_waypoint_map',
        'inner_ingress_waypoint_map',
        'inner_ingress_depth_adjustment_m',
        'old_ingress_runtime_pose_map',
        'inner_ingress_runtime_pose_map',
        'robot_pose_inside_maze_after_inner_ingress',
        'first_topology_summary',
        'first_dispatch_target_map',
        'first_dispatch_target_shift_from_phase62_m',
        'target_local_cost_improved_vs_phase62',
        'target_footprint_lethal_count_improved_vs_phase62',
        'front_wedge_high_cost_count_improved_vs_phase62',
        'dispatch_observed_not_exit_success',
        'complete_autonomous_success_claimed',
    ]:
        assert field in analyzer
    assert 'classify_phase65' in analyzer


def test_phase65_report_contract_preserves_guardrails_and_prior_context():
    report = _read(REPORT)
    assert 'Phase65' in report
    assert 'Ingress Goal Depth Adjustment' in report
    assert RUN_ID in report
    assert 'ENTRY_STAGING_TOO_SHALLOW' in report
    assert 'GEOMETRY_FEASIBILITY_BLOCKED' in report
    assert 'CORRIDOR_TOO_NARROW' in report
    assert 'SINGLE_OPEN_EXCEPTION_APPLIED_AND_DISPATCHED' in report
    assert 'not autonomous exploration success' in report or '不声明 autonomous exploration success' in report or '不是自主探索成功' in report
    assert 'not exit success' in report or '不是出口成功' in report
    assert '不进入 Phase66' in report or 'Do not enter Phase66' in report


def test_phase65_guardrails_do_not_modify_nav2_config_or_branch_strategy():
    nav2 = _read(NAV2_CONFIG)
    assert 'robot_radius: 0.35' in nav2
    assert 'inflation_radius: 0.46' in nav2
    analyzer = _read(ANALYZER)
    wrapper = _read(WRAPPER)
    joined = analyzer + '\n' + wrapper
    assert 'target_projection' not in joined
    assert 'projection integration' in joined
    assert 'score_for_exit' not in joined
    assert 'near_exit_fallback_enabled:=false' in wrapper
