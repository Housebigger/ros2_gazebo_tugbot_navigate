from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
WRAPPER = ROOT / 'tools' / 'run_phase57_first_dispatch_navigation_timeout_diagnostics.sh'
ANALYZER = ROOT / 'tools' / 'analyze_phase57_first_dispatch_navigation_timeout_diagnostics.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase57_first_dispatch_navigation_timeout_diagnostics_report.md'
RUN_ID = 'phase57_first_dispatch_navigation_timeout_diagnostics'

ALLOWED_CLASSIFICATIONS = {
    'TIMEOUT_TARGET_IN_HIGH_LOCAL_COST',
    'TIMEOUT_FRONT_WEDGE_BLOCKED',
    'TIMEOUT_CONTROLLER_NO_CMD_VEL',
    'TIMEOUT_ROBOT_STUCK_NEAR_WALL',
    'TIMEOUT_GOAL_TOO_CLOSE_TO_OBSTACLE',
    'TIMEOUT_NAV2_EXECUTION_CAUSE_IDENTIFIED',
    'TIMEOUT_INCONCLUSIVE_DATA_GAP',
    'GUARDRAIL_VIOLATION_STRATEGY_CHANGED',
    'FIRST_DISPATCH_NAVIGATION_SUCCEEDED_BOUNDED',
}


def test_phase57_wrapper_contract_and_guardrails():
    text = WRAPPER.read_text()
    assert f'RUN_ID="{RUN_ID}"' in text
    assert f'ARTIFACT_DIR="log/${{RUN_ID}}"' in text
    assert 'tugbot_maze_world_20260528_clean_scaled2x.sdf' in text
    assert 'maze_20260528_scaled_instance.yaml' in text
    assert 'tugbot_maze_slam_nav.launch.py' in text
    assert 'tools/analyze_phase57_first_dispatch_navigation_timeout_diagnostics.py --record-execution' in text
    assert 'tools/analyze_phase57_first_dispatch_navigation_timeout_diagnostics.py --send-ingress-goal' in text
    assert 'ros2 run tugbot_maze maze_explorer' in text
    assert 'maze_explorer --ros-args' in text
    assert '-p max_goals:=1' in text
    assert '-p near_exit_fallback_enabled:=false' in text
    assert '-p startup_warmup_no_dispatch:=false' in text
    assert '-p clearance_radius_m' not in text
    assert '-p dispatch_readiness_min_map_known_ratio' not in text
    assert '-p dispatch_readiness_min_map_free_ratio' not in text
    assert '-p goal_timeout_sec' not in text
    assert 'INGRESS_X="1.0"' in text
    assert 'INGRESS_Y="0.0"' in text
    assert 'FIRST_DISPATCH_EXECUTION_JSONL' in text
    assert 'CONTROLLER_DYNAMICS_JSONL' in text
    assert 'NAV2_FEEDBACK_JSONL' in text
    assert 'COLLISION_MONITOR_JSONL' in text
    assert 'FIRST_DISPATCH_EXECUTION_SUMMARY' in text
    assert 'RUN_TIMEOUT_SEC' in text and 'RUN_TIMEOUT_SEC > 360' in text
    assert 'max_goals=1' in text
    assert 'GUARDRAIL_VIOLATION_STRATEGY_CHANGED' in text
    assert 'autonomous exploration success' in text
    assert 'first dispatch is not exit success' in text
    assert 'git diff -- src/tugbot_navigation/config' in text
    assert 'cleanup()' in text


def test_phase57_analyzer_contract_and_classifications():
    text = ANALYZER.read_text()
    for classification in ALLOWED_CLASSIFICATIONS:
        assert classification in text
    assert 'PHASE = ' in text and 'Phase57 First Dispatch Navigation Timeout Diagnostics' in text
    assert 'RUN_ID = ' in text and RUN_ID in text
    assert 'INGRESS_WAYPOINT_MAP' in text and "'x_m': 1.0" in text
    assert 'ACCEPTANCE_RADIUS_M = 0.35' in text
    assert 'read_goal_events_jsonl' in text
    assert 'read_execution_jsonl' in text
    assert 'classify_first_dispatch_execution' in text
    required_fields = [
        'dispatch_target',
        'dispatch_target_local_cost',
        'dispatch_target_local_cost_max_radius',
        'global_plan_summary',
        'local_costmap_target_evidence',
        'local_costmap_path_evidence',
        'footprint_cost_timeline',
        'front_wedge_clearance_timeline',
        'cmd_vel_timeline_summary',
        'pose_progress_summary',
        'nav2_feedback_distance_remaining_summary',
        'collision_monitor_status_summary',
        'controller_server_log_summary',
        'bt_navigator_log_summary',
        'goal_outcome',
        'timeout_reason',
    ]
    for field in required_fields:
        assert field in text
    assert 'complete_autonomous_success_claimed' in text
    assert 'first dispatch is not exit success' in text
    assert 'no Nav2/MPPI/controller parameter edits' in text


def test_phase57_artifact_contract():
    text = ANALYZER.read_text()
    required_artifacts = [
        'first_dispatch_execution.jsonl',
        'controller_dynamics.jsonl',
        'nav2_feedback.jsonl',
        'local_costmap_samples.jsonl',
        'global_plan_samples.jsonl',
        'collision_monitor_state.jsonl',
        'first_dispatch_execution_summary.json',
        'explorer_state.jsonl',
        'goal_events.jsonl',
        'controller_server/bt_navigator logs',
        'summary/analysis JSON',
        'cleanup_processes_after.txt',
    ]
    for phrase in required_artifacts:
        assert phrase in text


def test_phase57_report_exists_and_preserves_scope():
    text = REPORT.read_text()
    assert '# Phase57: First Dispatch Navigation Timeout Diagnostics' in text
    assert any(classification in text for classification in ALLOWED_CLASSIFICATIONS)
    assert 'does not claim autonomous exploration success' in text
    assert 'first dispatch is not exit success' in text
    assert 'max_goals=1' in text
    assert 'near_exit_fallback_enabled=false' in text
    assert 'startup_warmup_no_dispatch=false' in text
    assert 'Nav2 config diff: empty' in text
