import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
RUNNER = ROOT / 'tools' / 'run_phase129_instrumented_first_goal_timeout_diagnosis.py'
ANALYZER = ROOT / 'tools' / 'analyze_phase129_instrumented_first_goal_timeout_diagnosis.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase129_instrumented_first_goal_timeout_diagnosis_report.md'


def _load(path: Path, module_name: str):
    assert path.exists(), f'missing module: {path}'
    spec = importlib.util.spec_from_file_location(module_name, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _base_runner_artifact(runner, *, status='TIMEOUT', timeout=True):
    record = runner.initial_instrumented_first_goal_record(max_goals=1)
    record.update({
        'maze_explorer_start_allowed': True,
        'maze_explorer_started': True,
        'exploration_goal_dispatched': True,
        'goal_event_count': 2,
        'dispatch_event_count': 1,
        'second_goal_dispatched': False,
        'stop_reason': 'first_explore_goal_terminal_stop',
    })
    record['first_goal'].update({
        'candidate': {'candidate_family': 'junction', 'candidate_rank': 1, 'candidate_source': 'maze_explorer_current_algorithm'},
        'pose': {'frame_id': 'map', 'x': 2.087, 'y': 1.023, 'yaw': 1.565},
        'frame_id': 'map',
        'goal_kind': 'explore',
        'accepted': True,
        'rejected': False,
        'timeout': timeout,
        'result_status_label': status,
        'abort_text': 'goal_timeout' if timeout else None,
        'nav2_feedback_timeline': [
            {'received_wall_time_sec': 10.0, 'distance_remaining': 0.38, 'number_of_recoveries': 3},
            {'received_wall_time_sec': 11.0, 'distance_remaining': 0.38, 'number_of_recoveries': 4},
        ],
        'distance_to_first_goal': {'available': True, 'meters': 0.356},
        'robot_pose_at_result': {'pose_available': True, 'x': 2.44, 'y': 1.02, 'yaw': 0.0},
        'frontier_evidence': {
            'candidate_branch_count': 2,
            'candidate_branches': [
                {'rank': 1, 'target_local_cost': 0, 'target_local_cost_max_radius': 54, 'path_corridor_min_clearance_m': 0.65, 'target_clearance_m': 0.65},
                {'rank': 2, 'target_local_cost': 90, 'target_local_cost_max_radius': 90, 'path_corridor_min_clearance_m': 0.25, 'target_clearance_m': 0.25},
            ],
            'chosen_branch_rank': 1,
        },
    })
    artifact = runner.build_phase129_artifact(
        run_id='unit',
        phase120_artifact={'classification': 'INGRESS_RESULT_SUCCEEDED_STOP_NO_MAZE_EXPLORER'},
        handoff_artifact={'preconditions': {'inner_ingress_result_succeeded': True}},
        first_goal_record=record,
        classification=runner.classify_phase129(record),
    )
    return artifact


def test_phase129_runner_and_analyzer_contract_constants():
    runner = _load(RUNNER, 'phase129_runner_contract')
    analyzer = _load(ANALYZER, 'phase129_analyzer_contract')
    assert runner.PHASE == 'Phase129'
    assert runner.MODE == 'instrumented_first_goal_timeout_diagnosis'
    assert set(runner.CLASSIFICATIONS) == {
        'FIRST_GOAL_RESULT_SUCCEEDED_WITH_INSTRUMENTATION_STOP',
        'FIRST_GOAL_TIMEOUT_LOCAL_COST_BLOCKED',
        'FIRST_GOAL_TIMEOUT_CONTROLLER_STALL',
        'FIRST_GOAL_TIMEOUT_GOAL_TOLERANCE_EDGE',
        'FIRST_GOAL_TIMEOUT_CANDIDATE_TOO_RISKY',
        'INSUFFICIENT_TIMEOUT_EVIDENCE',
    }
    assert analyzer.CLASSIFICATIONS == runner.CLASSIFICATIONS
    assert runner.FORBIDDEN_ACTIONS == {
        'second_exploration_goal': False,
        'manual_goal1_carry_over_staging_branch_centerline_fallback_terminal_exit_goal': False,
        'nav2_mppi_controller_goal_checker_config_tuning': False,
        'exploration_strategy_branch_scoring_centerline_fallback_terminal_acceptance_change': False,
        'autonomous_or_exit_success_claim': False,
    }


def test_phase129_initial_record_contains_required_instrumentation_fields():
    runner = _load(RUNNER, 'phase129_runner_schema')
    record = runner.initial_instrumented_first_goal_record(max_goals=1)
    for key in runner.REQUIRED_RESULT_FIELDS:
        assert key in record, key
    required = [
        'cmd_vel_timeline',
        'odom_velocity_timeline',
        'commanded_vs_measured_velocity',
        'robot_pose_trace',
        'distance_error_curve',
        'yaw_error_curve',
        'goal_checker_state',
        'local_costmap_windows',
        'global_costmap_windows',
        'footprint_front_path_cost_snapshots',
        'planned_path_snapshots',
        'bt_recovery_events',
        'controller_server_logs',
        'bt_navigator_logs',
        'first_goal_candidate_risk',
        'evidence_coverage',
        'evidence_gaps',
    ]
    for key in required:
        assert key in record['instrumentation'], key
    assert record['maze_explorer_max_goals'] == 1
    assert record['second_goal_dispatched'] is False


def test_phase129_classifies_success_as_bounded_first_goal_only():
    runner = _load(RUNNER, 'phase129_runner_success')
    artifact = _base_runner_artifact(runner, status='SUCCEEDED', timeout=False)
    assert artifact['classification'] == 'FIRST_GOAL_RESULT_SUCCEEDED_WITH_INSTRUMENTATION_STOP'
    assert artifact['guardrails']['max_goals_one'] is True
    assert artifact['guardrails']['second_goal_dispatched_false'] is True
    assert artifact['guardrails']['no_autonomous_success_claim'] is True
    assert artifact['guardrails']['no_exit_success_claim'] is True


def test_phase129_classifies_local_cost_blocked_with_time_aligned_high_costs():
    runner = _load(RUNNER, 'phase129_runner_local_cost')
    artifact = _base_runner_artifact(runner)
    instr = artifact['instrumentation']
    instr['footprint_front_path_cost_snapshots'] = [
        {'sample_wall_time_sec': 10.0, 'footprint_max': 99, 'front_max': 100, 'path_0_5m_max': 73, 'path_1_0m_max': 84, 'in_bounds': True},
        {'sample_wall_time_sec': 11.0, 'footprint_max': 99, 'front_max': 100, 'path_0_5m_max': 73, 'path_1_0m_max': 84, 'in_bounds': True},
    ]
    analysis = runner.analyze_instrumented_evidence(artifact)
    assert analysis['classification'] == 'FIRST_GOAL_TIMEOUT_LOCAL_COST_BLOCKED'
    assert analysis['evidence']['local_cost_blocked']['supports'] is True
    assert analysis['evidence']['controller_stall']['supports'] is False


def test_phase129_classifies_controller_stall_only_when_motion_evidence_dominates():
    runner = _load(RUNNER, 'phase129_runner_controller')
    artifact = _base_runner_artifact(runner)
    instr = artifact['instrumentation']
    instr['cmd_vel_timeline'] = [
        {'sample_wall_time_sec': 10.0, 'linear_x': 0.25, 'angular_z': 0.6},
        {'sample_wall_time_sec': 11.0, 'linear_x': -0.1, 'angular_z': -0.6},
    ]
    instr['odom_velocity_timeline'] = [
        {'sample_wall_time_sec': 10.0, 'linear_x': 0.0, 'angular_z': 0.0},
        {'sample_wall_time_sec': 11.0, 'linear_x': 0.0, 'angular_z': 0.0},
    ]
    instr['commanded_vs_measured_velocity'] = {
        'available': True,
        'cmd_nonzero_count': 2,
        'odom_near_zero_count': 2,
        'command_actual_mismatch_count': 2,
        'oscillation_count': 1,
    }
    analysis = runner.analyze_instrumented_evidence(artifact)
    assert analysis['classification'] == 'FIRST_GOAL_TIMEOUT_CONTROLLER_STALL'
    assert analysis['evidence']['controller_stall']['supports'] is True


def test_phase129_classifies_goal_tolerance_edge_with_pose_yaw_goal_checker_state():
    runner = _load(RUNNER, 'phase129_runner_tolerance')
    artifact = _base_runner_artifact(runner)
    instr = artifact['instrumentation']
    instr['robot_pose_trace'] = [{'x': 2.43, 'y': 1.02, 'yaw': 0.0}, {'x': 2.44, 'y': 1.02, 'yaw': 0.0}]
    instr['distance_error_curve'] = [{'distance_error_m': 0.356}, {'distance_error_m': 0.356}]
    instr['yaw_error_curve'] = [{'yaw_error_rad': 0.45}, {'yaw_error_rad': 0.44}]
    instr['goal_checker_state'] = {
        'available': True,
        'xy_tolerance_m': 0.35,
        'yaw_tolerance_rad': 0.25,
        'distance_error_m': 0.356,
        'yaw_error_rad': 0.44,
        'xy_within_tolerance': False,
        'yaw_within_tolerance': False,
    }
    analysis = runner.analyze_instrumented_evidence(artifact)
    assert analysis['classification'] == 'FIRST_GOAL_TIMEOUT_GOAL_TOLERANCE_EDGE'
    assert analysis['evidence']['goal_tolerance_edge']['supports'] is True


def test_phase129_classifies_candidate_too_risky_from_dispatch_risk():
    runner = _load(RUNNER, 'phase129_runner_candidate')
    artifact = _base_runner_artifact(runner)
    risk = artifact['instrumentation']['first_goal_candidate_risk']
    risk.update({
        'available': True,
        'target_clearance_m': 0.20,
        'target_local_cost_max_radius': 99,
        'path_corridor_min_clearance_m': 0.20,
        'selected_branch_riskier_than_alternatives': True,
        'near_wall': True,
        'near_high_cost_band': True,
    })
    analysis = runner.analyze_instrumented_evidence(artifact)
    assert analysis['classification'] == 'FIRST_GOAL_TIMEOUT_CANDIDATE_TOO_RISKY'
    assert analysis['evidence']['candidate_too_risky']['supports'] is True


def test_phase129_analyzer_validates_required_instrumentation_and_guardrails():
    runner = _load(RUNNER, 'phase129_runner_analyzer_sample')
    analyzer = _load(ANALYZER, 'phase129_analyzer_validation')
    artifact = _base_runner_artifact(runner)
    artifact['instrumentation']['footprint_front_path_cost_snapshots'] = [{'footprint_max': 99, 'front_max': 100, 'path_0_5m_max': 73, 'path_1_0m_max': 84, 'in_bounds': True}]
    analysis = analyzer.analyze_artifact(artifact)
    assert analysis['classification'] == 'FIRST_GOAL_TIMEOUT_LOCAL_COST_BLOCKED'
    assert analysis['valid'] is True
    assert analysis['guardrails']['second_goal_dispatched_false'] is True
    assert analysis['coverage']['cmd_vel_timeline']['present'] is False
    artifact['dispatch_event_count'] = 2
    artifact['second_goal_dispatched'] = True
    bad = analyzer.analyze_artifact(artifact)
    assert bad['valid'] is False
    assert bad['guardrails']['second_goal_dispatched_false'] is False


def test_phase129_report_records_runtime_diagnosis_boundaries_after_completion():
    if not REPORT.exists():
        return
    text = REPORT.read_text(encoding='utf-8')
    required = [
        'PHASE129_INSTRUMENTED_FIRST_GOAL_TIMEOUT_DIAGNOSIS_COMPLETE_STOP_BEFORE_PHASE130',
        'visible-stack',
        'max_goals=1',
        'cmd_vel timeline',
        'odom velocity timeline',
        'local/global costmap windows',
        'controller_server logs',
        'bt_navigator logs',
        'first goal candidate risk',
        'No second exploration goal',
        'No Nav2/MPPI/controller/goal checker/config tuning',
        'No autonomous exploration success or exit success is claimed',
        'Phase130 not entered',
    ]
    for phrase in required:
        assert phrase in text
