from pathlib import Path
import json

ROOT = Path(__file__).resolve().parents[3]
ARTIFACT_DIR = ROOT / 'log' / 'phase77_goal2_visual_root_cause_replay'
EVENT_JSON = ARTIFACT_DIR / 'phase77_goal2_timeout_visual_root_cause_event.json'
PLAN_JSON = ARTIFACT_DIR / 'phase77_goal2_timeout_visual_root_cause_replay_visual_root_cause_plan.json'
MARKER_JSON = ARTIFACT_DIR / 'phase77_goal2_timeout_visual_root_cause_replay_marker_specs.json'
CHECKLIST_MD = ARTIFACT_DIR / 'phase77_goal2_timeout_visual_root_cause_replay_screenshot_checklist.md'
HUMAN_TEMPLATE_MD = ARTIFACT_DIR / 'phase77_goal2_timeout_visual_root_cause_replay_human_observation_report_template.md'
REPORT_MD = ROOT / 'doc' / 'doc_report' / 'phase77_goal2_timeout_visual_root_cause_replay_first_application_report.md'

PHASE75_SUMMARY = ROOT / 'log' / 'phase75_goal2_timeout_after_directional_redispatch_diagnosis' / 'phase75_goal2_timeout_after_directional_redispatch_diagnosis.json'
REPLAY_DIR = ROOT / 'log' / 'phase74_directional_local_costmap_readiness_gate_validation' / 'replay_02'

REQUIRED_SOURCE_FILES = [
    REPLAY_DIR / 'phase74_directional_local_costmap_readiness_gate_validation_replay_02_goal_events.jsonl',
    REPLAY_DIR / 'phase74_controller_dynamics.jsonl',
    REPLAY_DIR / 'phase74_local_costmap_samples.jsonl',
    REPLAY_DIR / 'phase74_nav2_feedback.jsonl',
    REPLAY_DIR / 'phase74_global_plan_samples.jsonl',
]

REQUIRED_MARKERS = {
    'dispatch_target_marker',
    'original_target_marker',
    'refined_target_marker',
    'robot_trajectory_marker',
    'terminal_pose_marker',
    'footprint_marker',
    'front_wedge_marker',
    'local_cost_high_lethal_marker',
    'nearest_wall_marker',
    'corridor_centerline_marker',
    'goal_tolerance_circle_marker',
}

REQUIRED_SCREENSHOTS = {
    'dispatch',
    'near_goal',
    'recovery',
    'timeout',
    'final_pose',
}

OBSERVATION_CLASSES = [
    'TARGET_POSE_GEOMETRY_BAD',
    'LOCAL_COSTMAP_EXECUTION_BLOCKED',
    'FOOTPRINT_FRONT_WEDGE_COLLISION_RISK',
    'CORRIDOR_CENTERLINE_ALIGNMENT_BAD',
    'NAV2_CONTROLLER_RECOVERY_LOOP_DOMINANT',
    'INSUFFICIENT_VISUAL_EVIDENCE',
]

FORBIDDEN_SUCCESS_CLAIMS = [
    'autonomous exploration success achieved',
    'exit success achieved',
    'Phase78 entered',
]


def _load_json(path: Path):
    return json.loads(path.read_text(encoding='utf-8'))


def test_phase77_uses_real_phase75_goal2_timeout_artifacts_without_fabricating_missing_inputs():
    assert PHASE75_SUMMARY.exists(), 'Phase75 summary JSON is the real Goal2 timeout input source'
    for path in REQUIRED_SOURCE_FILES:
        assert path.exists(), f'missing required replay source artifact: {path}'
    assert EVENT_JSON.exists(), 'Phase77 must save the exact event JSON passed into the Phase76 template'
    event = _load_json(EVENT_JSON)
    assert event['source_phase75_summary'] == 'log/phase75_goal2_timeout_after_directional_redispatch_diagnosis/phase75_goal2_timeout_after_directional_redispatch_diagnosis.json'
    assert event['source_replay_artifact_dir'] == 'log/phase74_directional_local_costmap_readiness_gate_validation/replay_02'
    assert event['classification'] == 'GOAL2_TIMEOUT_LOCAL_COST_RECOVERY_LOOP'
    assert event['result_reason'] == 'goal_timeout'
    assert event['goal_sequence'] == 2
    assert event['visual_input_completeness']['goal_events_jsonl'] is True
    assert event['visual_input_completeness']['controller_dynamics_jsonl'] is True
    assert event['visual_input_completeness']['local_costmap_samples_jsonl'] is True
    assert event['visual_input_completeness']['nav2_feedback_jsonl'] is True
    assert event['visual_input_completeness']['global_plan_samples_jsonl'] is True
    assert event['insufficient_visual_replay_input'] is False
    assert event['missing_required_artifacts'] == []
    assert 'direct_nearest_wall_point' in event['missing_optional_visual_fields']
    assert event['trajectory_point_count_exported'] > 2
    assert event['local_cost_sample_count_goal2'] > 0


def test_phase77_generated_visual_replay_package_matches_phase76_contract_and_goal2_evidence():
    for path in [PLAN_JSON, MARKER_JSON, CHECKLIST_MD, HUMAN_TEMPLATE_MD]:
        assert path.exists(), f'missing generated Phase77 replay artifact: {path}'
    plan = _load_json(PLAN_JSON)
    assert plan['run_id'] == 'phase77_goal2_timeout_visual_root_cause_replay'
    assert plan['trigger']['selected'] == 'goal_timeout'
    for token in ['goal_timeout', 'local_cost_risk', 'recovery_count>0', 'near-goal outside tolerance']:
        assert token in plan['trigger']['matched']
    source = plan['source_event']
    assert source['classification'] == 'GOAL2_TIMEOUT_LOCAL_COST_RECOVERY_LOOP'
    assert source['number_of_recoveries'] == 3
    assert source['near_goal_outside_tolerance'] is True
    assert source['timeout_front_wedge_cost_max'] == 99
    assert source['timeout_footprint_cost_max'] == 99
    assert source['guardrail_note'] == 'Phase77 replay artifact generation only; no strategy/config tuning.'
    assert plan['guardrails']['navigation_strategy_changed'] is False
    assert plan['guardrails']['nav2_mppi_controller_tuned'] is False
    assert plan['guardrails']['branch_scoring_changed'] is False
    assert plan['guardrails']['centerline_gate_changed'] is False
    assert plan['guardrails']['directional_readiness_override_changed'] is False
    assert plan['guardrails']['fallback_or_terminal_acceptance_changed'] is False
    assert plan['guardrails']['autonomous_success_claimed'] is False
    assert plan['guardrails']['exit_success_claimed'] is False
    markers = {marker['name'] for marker in _load_json(MARKER_JSON)}
    assert REQUIRED_MARKERS.issubset(markers)
    checklist = CHECKLIST_MD.read_text(encoding='utf-8')
    for moment in REQUIRED_SCREENSHOTS:
        assert moment in checklist
    human_template = HUMAN_TEMPLATE_MD.read_text(encoding='utf-8')
    assert 'VISUAL_EVIDENCE_PENDING_HUMAN_OBSERVATION' in human_template
    assert 'front wedge' in human_template
    assert 'nearest wall' in human_template


def test_phase77_report_records_observation_classes_checklist_guardrails_and_stop_condition():
    assert REPORT_MD.exists(), 'Phase77 final report must be written under doc/doc_report/'
    report = REPORT_MD.read_text(encoding='utf-8')
    assert 'Phase77: Goal2 timeout Phase76 visual root-cause replay first application' in report
    assert 'INSUFFICIENT_VISUAL_REPLAY_INPUT' in report
    assert 'not emitted' in report or 'not required' in report
    for label in OBSERVATION_CLASSES:
        assert label in report
    for moment in REQUIRED_SCREENSHOTS:
        assert moment in report
    assert 'direct_nearest_wall_point' in report
    assert 'preexisting RViz/Gazebo screenshots' in report
    assert 'No navigation strategy changed' in report
    assert 'No Nav2/MPPI/controller/inflation/robot_radius/clearance_radius_m/map threshold tuning' in report
    assert 'No autonomous exploration success claimed' in report
    assert 'No exit success claimed' in report
    assert 'Phase78 not entered' in report
    for forbidden in FORBIDDEN_SUCCESS_CLAIMS:
        assert forbidden not in report
