import importlib.util
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
RUNNER = ROOT / 'tools' / 'run_phase139_instrumented_second_step_contract_runtime_verification.py'
ANALYZER = ROOT / 'tools' / 'analyze_phase139_instrumented_second_step_contract_runtime_verification.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase139_instrumented_second_step_contract_runtime_verification_report.md'
LOG_DIR = ROOT / 'log' / 'phase139_instrumented_second_step_contract_runtime_verification'


def _load(path: Path, module_name: str):
    assert path.exists(), f'missing module: {path}'
    spec = importlib.util.spec_from_file_location(module_name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _pending(*, runtime_serialized=True, exists=True, generated=13.0):
    return {
        'exists': exists,
        'runtime_serialized': runtime_serialized,
        'original_goal_kind': 'explore',
        'original_target': {'x': 1.68, 'y': 0.97, 'frame_id': 'map'},
        'direction_rad': 1.57,
        'start_node_id': 7,
        'active_branch': {'angle_rad': 1.57, 'target_xy': [1.68, 0.97], 'state': 'open'},
        'staging_goal_pose': {'x': 1.68, 'y': 0.02, 'yaw': 0.0, 'frame_id': 'map'},
        'staging_plan': {'two_step_staging_plan': {'mode': 'corridor_alignment'}, 'staging_goal_pose': {'x': 1.68, 'y': 0.02, 'yaw': 0.0, 'frame_id': 'map'}},
        'staging_result_status_label': 'SUCCEEDED',
        'staging_succeeded_wall_time_sec': 12.0,
        'fresh_scan_received': True,
        'fresh_local_costmap_received': True,
        'fresh_tf_received': True,
        'fresh_scan_sample_time_sec': 12.2,
        'fresh_local_costmap_sample_time_sec': 12.3,
        'fresh_tf_sample_time_sec': 12.4,
    }


def _second_payload(*, generated=True, valid=True, generation_time=13.0):
    return {
        'valid': valid,
        'map_frame_id': 'map',
        'x': 1.68,
        'y': 0.97,
        'yaw': 1.57,
        'selected_candidate_target': [1.68, 0.97],
        'selected_candidate_yaw': 1.57,
        'generated_after_fresh_evidence': generated,
        'fresh_scan_received': True,
        'fresh_local_costmap_received': True,
        'fresh_tf_received': True,
        'fresh_scan_sample_time_sec': 12.2,
        'fresh_local_costmap_sample_time_sec': 12.3,
        'fresh_tf_sample_time_sec': 12.4,
        'generation_wall_time_sec': generation_time,
        'front_wedge_risk_after_staging': {'available': True, 'max': 70, 'high_cost_count': 2, 'lethal_count': 0, 'sample_count': 136},
        'lateral_residual_after': 0.01,
        'candidate_count': 5,
        'hard_safety_pass_candidate_count': 2,
        'selected_candidate_index': 1,
        'selection_priority_trace': [{'candidate_index': 1, 'reason': 'hard_safety_pass'}],
        'rejected_candidate_summaries': [{'candidate_index': 0, 'reason': 'front_wedge_cost'}],
    }


def _artifact(*, status_label='ACCEPTED', timeout=False, rejected=False, third=False, pending=None, second_payload=None):
    if pending is None:
        pending = _pending()
    if second_payload is None:
        second_payload = _second_payload()
    second = {
        'available': True,
        'goal_kind': 'explore',
        'goal_sequence': 2,
        'target': {'x': 1.68, 'y': 0.97, 'yaw': 1.57, 'frame_id': 'map'},
        'selected_due_to_context': 'phase92_corridor_alignment_staging_second_step',
        'second_step_forward_goal': second_payload,
        'freshness_after_staging': {
            'fresh_scan_received': True,
            'fresh_local_costmap_received': True,
            'fresh_tf_received': True,
            'fresh_scan_sample_time_sec': 12.2,
            'fresh_local_costmap_sample_time_sec': 12.3,
            'fresh_tf_sample_time_sec': 12.4,
            'generation_wall_time_sec': 13.0,
        },
        'front_wedge_risk_after_staging': second_payload.get('front_wedge_risk_after_staging'),
        'lateral_residual_after': second_payload.get('lateral_residual_after'),
        'skip_two_step_staging': True,
        'phase138_recursion_guard': True,
        'phase136_recursion_guard': True,
        'recursion_guard': True,
        'two_step_stage_dispatch_requested': False,
        'staging_applied': False,
        'prior_staging_applied': True,
        'accepted': not rejected,
        'rejected': rejected,
        'timeout': timeout,
        'result_status_label': status_label,
    }
    return {
        'phase': 'Phase139',
        'mode': 'instrumented_second_step_contract_runtime_verification',
        'classification': None,
        'handoff_allowed': True,
        'maze_explorer_started': True,
        'maze_explorer_max_goals': 2,
        'dispatch_event_count': 2 if not third else 3,
        'second_step_goal_count': 1,
        'second_step_attempted': True,
        'third_goal_dispatched': third,
        'first_literal_dispatch': {
            'available': True,
            'goal_kind': 'corridor_alignment_staging',
            'goal_sequence': 1,
            'accepted': True,
            'rejected': False,
            'timeout': False,
            'result_status_label': 'SUCCEEDED',
            'staging_applied': True,
            'two_step_stage_dispatch_requested': True,
        },
        'staging_result': {'accepted': True, 'rejected': False, 'timeout': False, 'result_status_label': 'SUCCEEDED'},
        'pending_corridor_alignment_second_step': pending,
        'freshness_after_staging': second['freshness_after_staging'],
        'front_wedge_risk_after_staging': second_payload.get('front_wedge_risk_after_staging'),
        'lateral_residual_after': second_payload.get('lateral_residual_after'),
        'second_step_dispatch': second,
        'goal_events': [],
        'explorer_states': [],
        'nav2_feedback': [],
        'action_status_samples': [],
        'cmd_vel_timeline': [],
        'odom_velocity_timeline': [],
        'observation_timed_out': timeout,
        'stop_reason': 'unit',
        'guardrails': {
            'only_explicit_inner_ingress_goal_sent': True,
            'max_goals_two': True,
            'staging_and_second_step_only': not third,
            'one_second_step_goal_only': True,
            'third_goal_dispatched_false': not third,
            'manual_goal1_forbidden': True,
            'no_goal1_carry_over_branch_centerline_fallback_terminal_exit_manual_goal': True,
            'branch_scoring_changed': False,
            'centerline_gate_changed': False,
            'fallback_changed': False,
            'terminal_acceptance_changed': False,
            'direct_staging_disablement': False,
            'nav2_config_changed': False,
            'autonomous_success_claimed': False,
            'exit_success_claimed': False,
            'phase127_timeout_fixed_claimed': False,
            'no_autonomous_success_claim': True,
            'no_exit_success_claim': True,
        },
        'claims': {
            'autonomous_exploration_success': False,
            'exit_success': False,
            'phase127_timeout_fixed': False,
            'second_step_is_autonomous_exploration_success': False,
            'second_step_is_exit_success': False,
        },
    }


def test_phase139_runner_and_analyzer_constants_preserve_bounded_visible_stack_contract():
    runner = _load(RUNNER, 'phase139_runner_constants')
    analyzer = _load(ANALYZER, 'phase139_analyzer_constants')
    assert runner.PHASE == 'Phase139'
    assert runner.MODE == 'instrumented_second_step_contract_runtime_verification'
    assert runner.LOG_DIR.name == 'phase139_instrumented_second_step_contract_runtime_verification'
    assert runner.LOCKED_GOAL == {'frame_id': 'map', 'x': 2.0, 'y': 0.0, 'yaw': 0.0}
    assert runner.CLASSIFICATIONS == [
        'SECOND_STEP_CONTRACT_VERIFIED_EXPLORE_ACCEPTED_STOP',
        'SECOND_STEP_CONTRACT_VERIFIED_EXPLORE_RESULT_SUCCEEDED_STOP',
        'SECOND_STEP_CONTRACT_VERIFIED_EXPLORE_TIMEOUT_DIAGNOSTIC_FAIL',
        'SECOND_STEP_CONTRACT_VERIFIED_EXPLORE_REJECTED_DIAGNOSTIC_FAIL',
        'SECOND_STEP_CONTRACT_STILL_AMBIGUOUS',
        'STAGING_SUCCEEDED_SECOND_STEP_NOT_READY_FAIL_CLOSED',
    ]
    assert analyzer.CLASSIFICATIONS == runner.CLASSIFICATIONS
    assert runner.FORBIDDEN_ACTIONS['third_goal'] is False
    assert runner.FORBIDDEN_ACTIONS['full_exploration'] is False
    assert runner.FORBIDDEN_ACTIONS['nav2_mppi_controller_goal_checker_config_tuning'] is False
    assert runner.FORBIDDEN_ACTIONS['autonomous_or_exit_success_claim'] is False
    assert runner.FORBIDDEN_ACTIONS['phase127_timeout_fixed_claim'] is False


def test_phase139_analyzer_verifies_complete_phase138_runtime_contract_for_accepted_stop():
    analyzer = _load(ANALYZER, 'phase139_analyzer_accepted')
    result = analyzer.analyze_artifact(_artifact(status_label='ACCEPTED'))
    assert result['valid'] is True
    assert result['classification'] == 'SECOND_STEP_CONTRACT_VERIFIED_EXPLORE_ACCEPTED_STOP'
    for name, ok in result['contract_checks'].items():
        assert ok is True, name
    assert result['runtime_contract']['pending_corridor_alignment_second_step']['runtime_serialized']['value'] is True
    assert result['runtime_contract']['second_step_forward_goal']['valid']['value'] is True
    assert result['runtime_contract']['timestamp_ordering']['generated_after_all_fresh_evidence']['value'] is True
    assert result['second_step_policy']['third_goal_dispatched'] is False
    assert result['claims']['autonomous_exploration_success'] is False
    assert result['claims']['exit_success'] is False
    assert result['claims']['phase127_timeout_fixed'] is False


def test_phase139_analyzer_maps_result_succeeded_timeout_and_rejected_to_diagnostic_classifications():
    analyzer = _load(ANALYZER, 'phase139_analyzer_outcomes')
    succeeded = analyzer.analyze_artifact(_artifact(status_label='SUCCEEDED'))
    assert succeeded['classification'] == 'SECOND_STEP_CONTRACT_VERIFIED_EXPLORE_RESULT_SUCCEEDED_STOP'
    timed_out = analyzer.analyze_artifact(_artifact(status_label='TIMEOUT', timeout=True))
    assert timed_out['classification'] == 'SECOND_STEP_CONTRACT_VERIFIED_EXPLORE_TIMEOUT_DIAGNOSTIC_FAIL'
    rejected = analyzer.analyze_artifact(_artifact(status_label='ABORTED', rejected=True))
    assert rejected['classification'] == 'SECOND_STEP_CONTRACT_VERIFIED_EXPLORE_REJECTED_DIAGNOSTIC_FAIL'
    assert timed_out['claims']['phase127_timeout_fixed'] is False


def test_phase139_analyzer_rejects_missing_runtime_serialization_stale_timestamps_and_third_goal():
    analyzer = _load(ANALYZER, 'phase139_analyzer_rejects_gaps')
    missing_pending = analyzer.analyze_artifact(_artifact(pending=_pending(runtime_serialized=False)))
    assert missing_pending['valid'] is False
    assert missing_pending['classification'] == 'SECOND_STEP_CONTRACT_STILL_AMBIGUOUS'
    assert missing_pending['contract_checks']['pending_runtime_serialized'] is False

    stale = analyzer.analyze_artifact(_artifact(second_payload=_second_payload(generation_time=12.0)))
    assert stale['valid'] is False
    assert stale['classification'] == 'SECOND_STEP_CONTRACT_STILL_AMBIGUOUS'
    assert stale['contract_checks']['timestamp_ordering_generated_after_fresh_evidence'] is False

    third = analyzer.analyze_artifact(_artifact(third=True))
    assert third['valid'] is False
    assert third['classification'] == 'SECOND_STEP_CONTRACT_STILL_AMBIGUOUS'
    assert third['contract_checks']['third_goal_dispatched_false'] is False


def test_phase139_runner_normalizes_phase136_runtime_artifact_to_phase139_contract_without_relabeling_success_claims():
    runner = _load(RUNNER, 'phase139_runner_normalize')
    artifact = runner.build_phase139_artifact(run_id='unit-phase139', source_artifact=_artifact(status_label='SUCCEEDED'))
    assert artifact['phase'] == 'Phase139'
    assert artifact['mode'] == 'instrumented_second_step_contract_runtime_verification'
    assert artifact['classification'] == 'SECOND_STEP_CONTRACT_VERIFIED_EXPLORE_RESULT_SUCCEEDED_STOP'
    assert artifact['analysis']['valid'] is True
    assert artifact['third_goal_dispatched'] is False
    assert artifact['claims']['autonomous_exploration_success'] is False
    assert artifact['claims']['exit_success'] is False
    assert artifact['claims']['phase127_timeout_fixed'] is False
    assert artifact['forbidden_actions']['third_goal'] is False


def test_phase139_report_records_runtime_scope_cleanup_and_stop_before_phase140():
    text = REPORT.read_text(encoding='utf-8')
    required = [
        'PHASE139_INSTRUMENTED_SECOND_STEP_CONTRACT_RUNTIME_VERIFICATION',
        'visible-stack',
        'explicit inner-ingress goal: map,x=2.0,y=0.0,yaw=0.0',
        'max_goals:=2',
        'No third goal',
        'No full exploration',
        'No Nav2/MPPI/controller/goal checker/config tuning',
        'No branch scoring/centerline/fallback/terminal acceptance change',
        'No autonomous exploration success or exit success is claimed',
        'Phase127 timeout is not claimed fixed',
        'cleanup',
        'Phase140 not entered',
    ]
    for phrase in required:
        assert phrase in text
