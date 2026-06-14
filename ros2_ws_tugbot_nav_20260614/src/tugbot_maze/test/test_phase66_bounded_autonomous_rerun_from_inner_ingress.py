from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
WRAPPER = ROOT / 'tools' / 'run_phase66_bounded_autonomous_rerun_from_inner_ingress.sh'
ANALYZER = ROOT / 'tools' / 'analyze_phase66_bounded_autonomous_rerun_from_inner_ingress.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase66_bounded_autonomous_rerun_from_inner_ingress_report.md'
MAZE_CONFIG = ROOT / 'src' / 'tugbot_maze' / 'config' / 'maze_20260528_scaled_instance.yaml'
NAV2_CONFIG = ROOT / 'src' / 'tugbot_navigation' / 'config' / 'nav2_slam_params.yaml'
RUN_ID = 'phase66_bounded_autonomous_rerun_from_inner_ingress'


def _read(path: Path) -> str:
    assert path.exists(), f'missing {path}'
    return path.read_text(encoding='utf-8')


def test_phase66_wrapper_contract_is_inner_ingress_bounded_autonomous_smoke():
    wrapper = _read(WRAPPER)
    assert f'RUN_ID="{RUN_ID}"' in wrapper
    assert 'tugbot_maze_world_20260528_clean_scaled2x.sdf' in wrapper
    assert 'maze_20260528_scaled_instance.yaml' in wrapper
    assert 'INNER_INGRESS_X="2.0"' in wrapper
    assert 'INNER_INGRESS_Y="0.0"' in wrapper
    assert 'send-inner-ingress-goal' in wrapper
    assert 'MAX_GOALS="${PHASE66_MAX_GOALS:-4}"' in wrapper
    assert 'PHASE66_MAX_GOALS must be 3..5' in wrapper
    assert '-p max_goals:="$MAX_GOALS"' in wrapper
    assert 'timeout --preserve-status "$RUN_TIMEOUT_SEC" ros2 launch' in wrapper
    assert 'record_explorer_state_series.py --topic /maze/goal_events' in wrapper
    assert 'record_explorer_state_series.py --topic /maze/explorer_state' in wrapper
    assert 'analyze_phase66_bounded_autonomous_rerun_from_inner_ingress.py --record-runtime' in wrapper
    for artifact in [
        'phase66_runtime_timeline.jsonl',
        'phase66_controller_dynamics.jsonl',
        'phase66_nav2_feedback.jsonl',
        'phase66_local_costmap_samples.jsonl',
    ]:
        assert artifact in wrapper
    for guardrail in [
        'no Nav2/MPPI/controller parameter edits',
        'no inflation/robot_radius/clearance_radius_m/map threshold tuning',
        'no branch scoring change',
        'no target projection integration',
        'no fallback/terminal acceptance change',
        'no autonomous exploration success claim',
        'bounded runtime only',
    ]:
        assert guardrail in wrapper


def test_phase66_analyzer_schema_classifications_and_diagnostics():
    analyzer = _read(ANALYZER)
    assert f"RUN_ID = '{RUN_ID}'" in analyzer
    for classification in [
        'INNER_INGRESS_BOUNDED_PROGRESS',
        'INNER_INGRESS_TIMEOUT_REMAINS',
        'NO_CANDIDATE_REMAINS',
        'LOCAL_COST_RISK_REMAINS',
        'INSUFFICIENT_EVIDENCE',
        'GUARDRAIL_VIOLATION_STRATEGY_CHANGED',
    ]:
        assert classification in analyzer
    for field in [
        'inner_ingress_goal_success',
        'dispatch_count',
        'outcome_count',
        'timeout_count',
        'no_candidate_observed',
        'local_cost_risk_observed',
        'per_goal_summaries',
        'cmd_vel_summary',
        'nav2_feedback_summary',
        'robot_progress_summary',
        'local_cost_footprint_front_wedge_summary',
        'complete_autonomous_success_claimed',
        'exit_success_claimed',
    ]:
        assert field in analyzer
    assert 'class Phase66RuntimeRecorder' in analyzer
    assert '--record-runtime' in analyzer


def test_phase66_metadata_preserves_phase65_inner_ingress_and_nav2_config_guardrail():
    config = _read(MAZE_CONFIG)
    assert 'phase65_inner_ingress_waypoint_map:' in config
    assert 'x_m: 2.0' in config
    assert 'phase65_inner_ingress_depth_adjustment_m: 1.0' in config
    nav2 = _read(NAV2_CONFIG)
    assert 'robot_radius: 0.35' in nav2
    assert 'inflation_radius: 0.46' in nav2
    joined = _read(WRAPPER) + '\n' + _read(ANALYZER)
    assert 'target_projection' not in joined
    assert 'score_for_exit' not in joined
    assert 'clearance_radius_m:=' not in joined
    assert 'inflation_radius:=' not in joined
    assert 'robot_radius:=' not in joined


def test_phase66_report_contract_stop_and_no_success_claim():
    report = _read(REPORT)
    assert 'Phase66' in report
    assert 'Bounded Autonomous Rerun From Inner Ingress' in report
    assert RUN_ID in report
    assert 'INNER_INGRESS_FIX_IMPROVES_FIRST_DISPATCH' in report
    assert 'active clean scaled2x world' in report
    assert 'max_goals=4' in report or 'max_goals 4' in report
    assert '不宣称 autonomous exploration success' in report or 'no autonomous exploration success claim' in report
    assert '不进入 Phase67' in report or 'Do not enter Phase67' in report
