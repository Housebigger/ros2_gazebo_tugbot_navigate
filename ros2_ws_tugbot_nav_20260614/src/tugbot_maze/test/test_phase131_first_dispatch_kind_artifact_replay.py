import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
ANALYZER = ROOT / 'tools' / 'analyze_phase131_first_dispatch_kind_artifact_replay.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase131_first_dispatch_kind_artifact_replay_report.md'
LOG_DIR = ROOT / 'log' / 'phase131_first_dispatch_kind_artifact_replay'
OUTPUT_JSON = LOG_DIR / 'phase131_first_dispatch_kind_artifact_replay_analysis.json'
OUTPUT_MD = LOG_DIR / 'phase131_first_dispatch_kind_artifact_replay_summary.md'

PHASE124_ARTIFACT = ROOT / 'log' / 'phase124_first_exploration_goal_dispatch_smoke' / 'phase124_first_exploration_goal_dispatch_smoke_rerun_artifact.json'
PHASE125_ARTIFACT = ROOT / 'log' / 'phase125_first_exploration_goal_execution_result_smoke' / 'phase125_first_exploration_goal_execution_result_smoke_artifact.json'
PHASE129_ARTIFACT = ROOT / 'log' / 'phase129_instrumented_first_goal_timeout_diagnosis' / 'phase129_instrumented_first_goal_timeout_diagnosis_rerun.json'


def _load_analyzer():
    assert ANALYZER.exists(), f'missing Phase131 analyzer: {ANALYZER}'
    spec = importlib.util.spec_from_file_location('phase131_analyzer', ANALYZER)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _read_json(path: Path):
    assert path.exists(), f'missing expected artifact: {path}'
    return json.loads(path.read_text(encoding='utf-8'))


def test_phase131_analyzer_declares_artifact_replay_guardrails_and_classifications():
    module = _load_analyzer()
    assert module.MODE == 'first_dispatch_kind_artifact_replay'
    assert module.CLASSIFICATIONS == [
        'FIRST_DISPATCH_KIND_STABLE_EXPLORE',
        'FIRST_DISPATCH_KIND_CHANGED_TO_STAGING_POST_INGRESS',
        'FIRST_DISPATCH_KIND_CHANGED_BY_POSE_TOPOLOGY_DRIFT',
        'FIRST_DISPATCH_KIND_CONTRACT_AMBIGUOUS',
        'INSUFFICIENT_DISPATCH_KIND_EVIDENCE',
    ]
    assert module.FORBIDDEN_RUNTIME_ACTIONS == {
        'launch_gazebo_rviz_nav2': False,
        'send_navigate_to_pose_goal': False,
        'start_maze_explorer': False,
        'send_exploration_corridor_or_staging_goal': False,
        'tune_nav2_mppi_controller_goal_checker_config': False,
        'change_exploration_strategy_branch_scoring_centerline_fallback_terminal_acceptance': False,
        'claim_autonomous_or_exit_success': False,
    }


def test_phase131_normalizes_required_field_matrix_from_synthetic_dispatch_events():
    module = _load_analyzer()
    artifact = {
        'classification': 'SYNTHETIC',
        'dispatch_event_count': 1,
        'maze_explorer_max_goals': 1,
        'first_goal_result_artifact': {
            'goal_events': [{
                'event': 'dispatch',
                'goal_kind': 'corridor_alignment_staging',
                'goal_sequence': 1,
                'dispatch_pose': [1.85, 0.02, 0.0],
                'target': [1.70, 0.07],
                'original_target': [1.70, 1.02],
                'refined_target': [1.70, 0.07],
                'current_node_id': 2,
                'start_node_id': 2,
                'local_topology': 'junction',
                'candidate_family': {'centerline_projection': True},
                'candidate_count': 63,
                'candidate_branch_count': 3,
                'last_open_direction_count': 3,
                'last_candidate_count': 3,
                'near_exit': False,
                'selected_due_to_context': 'topology_exit_bias_score',
                'goal_count_before_dispatch': 0,
                'staging_applied': True,
                'two_step_stage_dispatch_requested': True,
                'staging_reason': 'reduce_lateral_residual_and_front_wedge_risk_before_second_step_forward_goal',
                'staging_goal_pose': {'x': 1.70, 'y': 0.07, 'yaw': 1.56, 'lateral_residual_before_m': 0.15},
                'post_ingress_context_active': False,
                'single_open_exception_applied': False,
                'topology_consistency_guard': {'topology_consistency_status': 'idle'},
            }]
        },
    }
    normalized = module.normalize_phase_artifact('Phase129', Path('synthetic.json'), artifact)
    required = normalized['required_fields']
    for field in [
        'robot_pose_after_ingress',
        'dispatch_pose',
        'goal_kind',
        'current_node_id',
        'start_node_id',
        'topology_state',
        'candidate_family',
        'candidate_rank',
        'candidate_count',
        'candidate_branch_count',
        'last_open_direction_count',
        'last_candidate_count',
        'near_exit',
        'post_ingress_flags',
        'active_edge_state_machine',
        'goal_count_max_goals',
        'raw_target',
        'refined_target',
        'original_target',
        'selection_reason',
    ]:
        assert field in required
    assert required['goal_kind']['value'] == 'corridor_alignment_staging'
    assert required['candidate_branch_count']['value'] == 3
    assert required['refined_target']['value'] == [1.70, 0.07]
    assert normalized['staging_evidence']['staging_applied'] is True
    assert normalized['staging_evidence']['staging_reason'].startswith('reduce_lateral_residual')


def test_phase131_synthetic_contract_classifications_are_conservative():
    module = _load_analyzer()
    explore_event = {
        'event': 'dispatch',
        'goal_kind': 'explore',
        'dispatch_pose': [1.85, 0.02, 0.0],
        'target': [2.08, 1.02],
        'original_target': [2.08, 1.02],
        'refined_target': [2.08, 1.02],
        'current_node_id': 2,
        'start_node_id': 2,
        'local_topology': 'junction',
        'candidate_branch_count': 4,
        'last_open_direction_count': 4,
        'last_candidate_count': 4,
        'near_exit': False,
        'staging_applied': False,
        'post_ingress_context_active': False,
    }
    p124 = module.normalize_phase_artifact('Phase124', Path('p124.json'), {'classification': 'OK', 'maze_explorer_max_goals': 1, 'first_goal_artifact': {'goal_events': [explore_event]}})
    p125 = module.normalize_phase_artifact('Phase125', Path('p125.json'), {'classification': 'OK', 'maze_explorer_max_goals': 1, 'first_goal_result_artifact': {'goal_events': [explore_event]}})
    p129_explore = module.normalize_phase_artifact('Phase129', Path('p129.json'), {'classification': 'OK', 'maze_explorer_max_goals': 1, 'first_goal_result_artifact': {'goal_events': [explore_event]}})
    stable = module.classify_dispatch_kind([p124, p125, p129_explore])
    assert stable['classification'] == 'FIRST_DISPATCH_KIND_STABLE_EXPLORE'

    p129_post_ingress_event = dict(explore_event)
    p129_post_ingress_event.update({
        'goal_kind': 'corridor_alignment_staging',
        'target': [1.70, 0.07],
        'refined_target': [1.70, 0.07],
        'staging_applied': True,
        'two_step_stage_dispatch_requested': True,
        'staging_reason': 'reduce_lateral_residual_and_front_wedge_risk_before_second_step_forward_goal',
        'post_ingress_context_active': True,
        'first_post_ingress_topology_node': True,
    })
    p129_post_ingress = module.normalize_phase_artifact('Phase129', Path('p129.json'), {'classification': 'OK', 'maze_explorer_max_goals': 1, 'first_goal_result_artifact': {'goal_events': [p129_post_ingress_event]}})
    post = module.classify_dispatch_kind([p124, p125, p129_post_ingress])
    assert post['classification'] == 'FIRST_DISPATCH_KIND_CHANGED_TO_STAGING_POST_INGRESS'

    p129_topology_event = dict(p129_post_ingress_event)
    p129_topology_event.update({
        'post_ingress_context_active': False,
        'first_post_ingress_topology_node': False,
        'candidate_branch_count': 3,
        'last_open_direction_count': 3,
        'last_candidate_count': 3,
    })
    p129_topology = module.normalize_phase_artifact('Phase129', Path('p129.json'), {'classification': 'OK', 'maze_explorer_max_goals': 1, 'first_goal_result_artifact': {'goal_events': [p129_topology_event]}})
    drift = module.classify_dispatch_kind([p124, p125, p129_topology])
    assert drift['classification'] == 'FIRST_DISPATCH_KIND_CHANGED_BY_POSE_TOPOLOGY_DRIFT'


def test_phase131_real_phase124_125_129_artifact_replay_classifies_actual_discrepancy_without_runtime():
    module = _load_analyzer()
    result = module.analyze_artifacts(
        phase124_path=PHASE124_ARTIFACT,
        phase125_path=PHASE125_ARTIFACT,
        phase129_path=PHASE129_ARTIFACT,
    )
    assert result['mode'] == 'first_dispatch_kind_artifact_replay'
    assert result['guardrails']['forbidden_runtime_actions'] == module.FORBIDDEN_RUNTIME_ACTIONS
    assert result['classification'] == 'FIRST_DISPATCH_KIND_CHANGED_BY_POSE_TOPOLOGY_DRIFT'
    assert result['dispatch_kind_sequence'] == ['explore', 'explore', 'corridor_alignment_staging']
    assert result['phase_comparison']['Phase124']['required_fields']['candidate_branch_count']['value'] == 4
    assert result['phase_comparison']['Phase125']['required_fields']['candidate_branch_count']['value'] == 4
    assert result['phase_comparison']['Phase129']['required_fields']['candidate_branch_count']['value'] == 3
    assert result['phase_comparison']['Phase129']['staging_evidence']['staging_applied'] is True
    assert result['trigger_evidence']['topology_or_candidate_drift']['candidate_branch_count_delta_from_phase125'] == -1
    assert result['trigger_evidence']['pose_yaw_drift']['dispatch_pose_xy_delta_from_phase125_m'] < 0.02
    assert result['trigger_evidence']['post_ingress_context']['post_ingress_context_active'] is False
    assert 'Phase129 staging transformed an explore original_target into a near-robot staging target' in result['diagnosis_summary']
    assert result['claims']['autonomous_exploration_success'] is False
    assert result['claims']['exit_success'] is False


def test_phase131_cli_outputs_json_and_markdown_summary_after_artifact_replay():
    module = _load_analyzer()
    # This test verifies contract shape only; it does not launch ROS/Gazebo/Nav2 or start maze_explorer.
    result = module.analyze_artifacts(
        phase124_path=PHASE124_ARTIFACT,
        phase125_path=PHASE125_ARTIFACT,
        phase129_path=PHASE129_ARTIFACT,
    )
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    module.write_outputs(result, OUTPUT_JSON, OUTPUT_MD)
    assert OUTPUT_JSON.exists()
    assert OUTPUT_MD.exists()
    saved = _read_json(OUTPUT_JSON)
    summary = OUTPUT_MD.read_text(encoding='utf-8')
    assert saved['classification'] == 'FIRST_DISPATCH_KIND_CHANGED_BY_POSE_TOPOLOGY_DRIFT'
    assert 'FIRST_DISPATCH_KIND_CHANGED_BY_POSE_TOPOLOGY_DRIFT' in summary
    assert 'No Gazebo/RViz/Nav2 runtime was launched' in summary
    assert 'No maze_explorer was started' in summary
    assert 'Phase132 not entered' in summary


def test_phase131_report_records_completion_and_verification_without_success_claims():
    text = REPORT.read_text(encoding='utf-8')
    required = [
        'PHASE131_FIRST_DISPATCH_KIND_ARTIFACT_REPLAY_COMPLETE_STOP_BEFORE_PHASE132',
        'FIRST_DISPATCH_KIND_CHANGED_BY_POSE_TOPOLOGY_DRIFT',
        'Phase124/125 first dispatch remained goal_kind=explore',
        'Phase129 first dispatch changed to goal_kind=corridor_alignment_staging',
        'candidate_branch_count changed from 4 to 3',
        'post_ingress_context_active=false',
        'staging_applied=true',
        'diagnostic-only',
        'No Gazebo/RViz/Nav2 runtime was launched',
        'No NavigateToPose goal was sent',
        'No maze_explorer was started',
        'No Nav2/MPPI/controller/goal checker/config tuning was performed',
        'No exploration strategy/branch scoring/centerline/fallback/terminal acceptance change was made',
        'No autonomous exploration success or exit success is claimed',
        'Phase132 not entered',
    ]
    for phrase in required:
        assert phrase in text


def test_phase131_does_not_add_runtime_runner_or_nav_config_changes():
    forbidden_runtime_runner = ROOT / 'tools' / 'run_phase131_first_dispatch_kind_artifact_replay.py'
    assert not forbidden_runtime_runner.exists()
    source = ANALYZER.read_text(encoding='utf-8')
    forbidden_tokens = [
        'ros2 launch',
        'NavigateToPose.Goal',
        'send_goal_async',
        'rclpy.init',
        'maze_explorer max_goals',
        'controller_server:',
        'FollowPath:',
    ]
    for token in forbidden_tokens:
        assert token not in source
