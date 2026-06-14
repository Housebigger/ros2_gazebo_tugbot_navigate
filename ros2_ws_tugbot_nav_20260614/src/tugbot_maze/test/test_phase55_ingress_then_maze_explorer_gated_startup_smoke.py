from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
WRAPPER = ROOT / 'tools' / 'run_phase55_ingress_then_maze_explorer_gated_startup_smoke.sh'
ANALYZER = ROOT / 'tools' / 'analyze_phase55_ingress_then_maze_explorer_gated_startup_smoke.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase55_ingress_then_maze_explorer_gated_startup_smoke_report.md'
RUN_ID = 'phase55_ingress_then_maze_explorer_gated_startup_smoke'

ALLOWED_CLASSIFICATIONS = {
    'INGRESS_THEN_GATE_READY_FIRST_DISPATCH',
    'INGRESS_THEN_GATE_READY_NO_DISPATCH_TOPOLOGY_REJECTED',
    'INGRESS_REACHED_GATE_STILL_NOT_READY',
    'INGRESS_GOAL_FAILED',
    'GUARDRAIL_VIOLATION_UNBOUNDED_OR_SUCCESS_CLAIM',
}


def test_phase55_wrapper_contract_and_guardrails():
    text = WRAPPER.read_text()
    assert f'RUN_ID="{RUN_ID}"' in text
    assert f'ARTIFACT_DIR="log/${{RUN_ID}}"' in text
    assert 'tugbot_maze_world_20260528_clean_scaled2x.sdf' in text
    assert 'maze_20260528_scaled_instance.yaml' in text
    assert 'tugbot_maze_slam_nav.launch.py' in text
    assert 'ros2 run tugbot_maze maze_explorer' in text
    assert 'maze_explorer --ros-args' in text
    assert "-p max_goals:=1" in text
    assert "-p near_exit_fallback_enabled:=false" in text
    assert "-p startup_warmup_no_dispatch:=false" in text
    assert "-p clearance_radius_m" not in text
    assert "-p dispatch_readiness_min_map_known_ratio" not in text
    assert "-p dispatch_readiness_min_map_free_ratio" not in text
    assert "INGRESS_X=\"1.0\"" in text
    assert "INGRESS_Y=\"0.0\"" in text
    assert 'ACCEPTANCE_RADIUS_M="${PHASE55_ACCEPTANCE_RADIUS_M:-0.35}"' in text
    assert 'RUN_TIMEOUT_SEC' in text and 'RUN_TIMEOUT_SEC > 360' in text
    assert 'launch_phase55' in text
    assert 'start_maze_explorer_after_ingress' in text
    assert 'cleanup()' in text
    assert 'git diff -- src/tugbot_navigation/config' in text
    assert 'phase55_ingress_then_maze_explorer_gated_startup_smoke_explorer_state.jsonl' in text
    assert 'phase55_ingress_then_maze_explorer_gated_startup_smoke_goal_events.jsonl' in text
    assert 'GUARDRAIL_VIOLATION_UNBOUNDED_OR_SUCCESS_CLAIM' in text
    assert 'autonomous exploration success' in text


def test_phase55_analyzer_contract_and_classifications():
    text = ANALYZER.read_text()
    for classification in ALLOWED_CLASSIFICATIONS:
        assert classification in text
    assert 'PHASE = ' in text and 'Phase55 Ingress-then-maze_explorer Gated Startup Smoke' in text
    assert 'RUN_ID = ' in text and RUN_ID in text
    assert 'INGRESS_WAYPOINT_MAP' in text and "'x_m': 1.0" in text
    assert 'ACCEPTANCE_RADIUS_M = 0.35' in text
    assert 'read_explorer_state_jsonl' in text
    assert 'read_goal_events_jsonl' in text
    assert 'first_gate_ready_state' in text
    assert 'first_topology_state' in text
    assert 'first_dispatch_event' in text
    assert 'gate_payload_before_ingress' in text
    assert 'gate_payload_after_ingress' in text
    assert 'topology_sampling_diagnostics' in text
    assert 'complete_autonomous_success_claimed' in text
    assert 'max_goals_limited_to_one' in text
    assert 'near_exit_fallback_disabled' in text


def test_phase55_artifact_and_report_contract():
    text = ANALYZER.read_text()
    required_artifacts = [
        'launch.log',
        'ingress action result',
        'lifecycle/action/topic readiness',
        'robot pose timeline',
        'explorer_state.jsonl',
        'goal_events.jsonl',
        'gate payload before/after ingress',
        'first topology sampling diagnostics',
        'map/scan/TF/local/global costmap evidence',
        'summary/analysis JSON',
        'cleanup_processes_after.txt',
    ]
    for phrase in required_artifacts:
        assert phrase in text


def test_phase55_report_exists_and_preserves_scope():
    text = REPORT.read_text()
    assert '# Phase55: Ingress-then-maze_explorer Gated Startup Smoke' in text
    assert any(classification in text for classification in ALLOWED_CLASSIFICATIONS)
    assert 'does not claim autonomous exploration success' in text
    assert 'does not require reaching the exit' in text
    assert 'max_goals=1' in text
    assert 'near_exit_fallback_enabled=false' in text
    assert 'startup_warmup_no_dispatch=false' in text
    assert 'Nav2 config diff: empty' in text
