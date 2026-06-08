from pathlib import Path
import importlib.util
import json

ROOT = Path(__file__).resolve().parents[3]
WORKFLOW = ROOT / 'doc' / 'doc_proposal' / 'phase76_obstacle_triggered_visual_root_cause_workflow.md'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase76_obstacle_triggered_visual_root_cause_workflow_report.md'
TEMPLATE = ROOT / 'tools' / 'phase76_visual_root_cause_replay_template.py'
RUNBOOK = ROOT / 'tools' / 'run_phase76_visual_root_cause_replay_template.sh'

TRIGGER_TOKENS = [
    'goal_timeout',
    'FAILED_EXHAUSTED',
    'no_candidate',
    'local_cost_risk',
    'recovery_count>0',
    'near-goal outside tolerance',
    're-dispatch readiness blocked',
]

MARKER_TOKENS = [
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
]

SCREENSHOT_MOMENTS = [
    'dispatch',
    'near_goal',
    'recovery',
    'timeout',
    'final_pose',
]

FORBIDDEN_STRATEGY_TOKENS = [
    'CostCritic.cost_weight:',
    'inflation_radius:',
    'robot_radius:',
    'clearance_radius_m:=',
    'dispatch_readiness_min_local_costmap_free_ratio:=',
    'branch_score',
    'directional_local_costmap_readiness_override_enabled:=false',
    'near_exit_fallback_enabled:=true',
]


def _load_template():
    spec = importlib.util.spec_from_file_location('phase76_visual_root_cause_replay_template', TEMPLATE)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_phase76_workflow_document_exists_with_triggers_markers_and_guardrails():
    assert WORKFLOW.exists(), 'Phase76 workflow proposal document must exist'
    text = WORKFLOW.read_text(encoding='utf-8')
    assert '# Phase76: Obstacle-Triggered Visual Root-Cause Workflow' in text
    assert 'Phase75' in text and 'GOAL2_TIMEOUT_LOCAL_COST_RECOVERY_LOOP' in text
    assert '不调 Nav2/MPPI/controller' in text
    assert '不调 inflation/robot_radius/clearance_radius_m/map threshold' in text
    assert '不改 branch scoring' in text
    assert '不改 centerline gate' in text
    assert '不改 directional readiness override' in text
    assert '不改 fallback/terminal acceptance' in text
    assert '不声明 autonomous exploration success' in text
    assert '不声明 exit success' in text
    assert '不跑长时间实验' in text
    for token in TRIGGER_TOKENS + MARKER_TOKENS + SCREENSHOT_MOMENTS:
        assert token in text
    assert '人工观察报告模板' in text
    assert 'replay command' in text


def test_phase76_template_contract_lists_all_trigger_marker_and_screenshot_requirements():
    assert TEMPLATE.exists(), 'Phase76 reusable replay/marker/checklist template must exist'
    text = TEMPLATE.read_text(encoding='utf-8')
    assert 'Phase76 Obstacle-Triggered Visual Root-Cause Workflow' in text
    assert 'no Nav2/MPPI/controller tuning' in text
    assert 'no autonomous exploration success claim' in text
    assert 'no exit success claim' in text
    for token in TRIGGER_TOKENS + MARKER_TOKENS + SCREENSHOT_MOMENTS:
        assert token in text
    for forbidden in FORBIDDEN_STRATEGY_TOKENS:
        assert forbidden not in text


def test_phase76_template_builds_replay_plan_marker_specs_and_checklist(tmp_path):
    module = _load_template()
    event = {
        'event': 'timeout',
        'result_reason': 'goal_timeout',
        'goal_sequence': 2,
        'target': [2.05, 1.02],
        'original_target': [2.05, 1.02],
        'refined_target': [2.10, 1.03],
        'dispatch_pose': [2.30, 0.03, 0.02],
        'terminal_pose': [2.42, 1.03, 1.61],
        'final_pose': [2.42, 1.03, 1.61],
        'branch_angle': 1.57,
        'number_of_recoveries': 3,
        'local_cost_risk': True,
        'near_goal_outside_tolerance': True,
        'timeout_front_wedge_cost_max': 99,
        'timeout_footprint_lethal_cell_count': 48,
        'nearest_wall_point': [2.45, 1.08],
        'corridor_centerline_points': [[2.0, 0.0], [2.0, 2.0]],
    }
    plan = module.build_visual_root_cause_plan(event, artifact_dir=tmp_path, run_id='phase76_test')
    assert plan['run_id'] == 'phase76_test'
    assert plan['trigger']['selected'] == 'goal_timeout'
    assert 'goal_timeout' in plan['trigger']['matched']
    assert 'recovery_count>0' in plan['trigger']['matched']
    assert 'local_cost_risk' in plan['trigger']['matched']
    assert 'near-goal outside tolerance' in plan['trigger']['matched']
    assert plan['guardrails']['autonomous_success_claimed'] is False
    assert plan['guardrails']['exit_success_claimed'] is False
    assert 'ros2 launch tugbot_bringup phase76_visual_root_cause_overlay.launch.py' in plan['replay_command']
    names = {marker['name'] for marker in plan['marker_specs']}
    assert set(MARKER_TOKENS).issubset(names)
    checklist_ids = {item['id'] for item in plan['screenshot_checklist']}
    assert set(SCREENSHOT_MOMENTS).issubset(checklist_ids)
    report = module.render_human_observation_report_template(plan)
    assert '人工观察报告模板' in report
    assert 'dispatch target' in report
    assert 'front wedge' in report
    assert 'local cost high/lethal' in report
    assert 'nearest wall' in report
    assert 'corridor centerline' in report


def test_phase76_template_cli_writes_plan_checklist_and_report(tmp_path):
    module = _load_template()
    source = tmp_path / 'event.json'
    source.write_text(json.dumps({
        'event': 'FAILED_EXHAUSTED',
        'result_reason': 'FAILED_EXHAUSTED',
        'goal_sequence': 4,
        'target': [3.0, 2.0],
        'dispatch_pose': [2.0, 2.0, 0.0],
        'terminal_pose': [2.2, 2.1, 0.1],
        're_dispatch_readiness_blocked': True,
        'no_candidate': True,
    }), encoding='utf-8')
    output_dir = tmp_path / 'out'
    rc = module.main(['--event-json', str(source), '--artifact-dir', str(output_dir), '--run-id', 'phase76_cli'])
    assert rc == 0
    plan_path = output_dir / 'phase76_cli_visual_root_cause_plan.json'
    checklist_path = output_dir / 'phase76_cli_screenshot_checklist.md'
    report_path = output_dir / 'phase76_cli_human_observation_report_template.md'
    marker_path = output_dir / 'phase76_cli_marker_specs.json'
    for path in [plan_path, checklist_path, report_path, marker_path]:
        assert path.exists(), f'missing generated artifact {path}'
    plan = json.loads(plan_path.read_text(encoding='utf-8'))
    assert plan['trigger']['selected'] == 'FAILED_EXHAUSTED'
    assert 're-dispatch readiness blocked' in plan['trigger']['matched']
    assert 'no_candidate' in plan['trigger']['matched']
    assert 'final_pose' in checklist_path.read_text(encoding='utf-8')


def test_phase76_runbook_and_final_report_are_stop_scope_only():
    assert RUNBOOK.exists(), 'Phase76 replay runbook shell template must exist'
    script = RUNBOOK.read_text(encoding='utf-8')
    assert 'PHASE76_VISUAL_REPLAY_ONLY=1' in script
    assert 'PHASE76_MAX_OBSERVE_SEC="${PHASE76_MAX_OBSERVE_SEC:-90}"' in script
    assert 'phase76_visual_root_cause_replay_template.py' in script
    assert 'PHASE76_READY_FOR_HUMAN_OBSERVATION' in script
    assert 'RViz/Gazebo' in script
    for forbidden in FORBIDDEN_STRATEGY_TOKENS:
        assert forbidden not in script

    assert REPORT.exists(), 'Phase76 final report must exist'
    report = REPORT.read_text(encoding='utf-8')
    assert 'Phase76: Obstacle-Triggered Visual Root-Cause Workflow' in report
    assert 'workflow/template only' in report
    assert 'No navigation strategy changed' in report
    assert 'Phase77 not entered' in report
    assert '不声明 autonomous exploration success' in report
    assert '不声明 exit success' in report
