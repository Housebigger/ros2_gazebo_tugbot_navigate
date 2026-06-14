import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
ANALYZER = ROOT / 'tools' / 'analyze_phase127_first_goal_timeout_artifact_replay.py'
REPORT = ROOT / 'doc' / 'doc_report' / 'phase127_first_goal_timeout_artifact_replay_report.md'
PHASE125_LOG = ROOT / 'log' / 'phase125_first_exploration_goal_execution_result_smoke'
PHASE127_LOG = ROOT / 'log' / 'phase127_first_goal_timeout_artifact_replay'


def _load():
    assert ANALYZER.exists(), f'missing Phase127 analyzer: {ANALYZER}'
    spec = importlib.util.spec_from_file_location('phase127_analyzer', ANALYZER)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _base_artifact():
    return {
        'phase': 'Phase125',
        'classification': 'FIRST_EXPLORATION_GOAL_RESULT_TIMEOUT_DIAGNOSTIC_FAIL',
        'maze_explorer_max_goals': 1,
        'maze_explorer_started': True,
        'second_goal_dispatched': False,
        'dispatch_event_count': 1,
        'goal_event_count': 2,
        'guardrails': {
            'nav2_config_changed': False,
            'branch_scoring_changed': False,
            'centerline_gate_changed': False,
            'fallback_changed': False,
            'terminal_acceptance_changed': False,
        },
        'first_goal': {
            'goal_kind': 'explore',
            'accepted': True,
            'timeout': True,
            'result_status_label': 'TIMEOUT',
            'abort_text': 'goal_timeout',
            'pose': {'frame_id': 'map', 'x': 2.087, 'y': 1.023, 'yaw': 1.565},
            'robot_pose_at_result': {'available': True, 'x': 2.444, 'y': 1.015},
            'distance_to_first_goal': {'available': True, 'meters': 0.356},
            'nav2_feedback_timeline': [
                {'distance_remaining': 0.38, 'navigation_time_sec': 44.8, 'number_of_recoveries': 4},
                {'distance_remaining': 0.38, 'navigation_time_sec': 45.6, 'number_of_recoveries': 4},
            ],
            'candidate': {
                'candidate_family': 'junction',
                'candidate_rank': 1,
                'candidate_source': 'maze_explorer_current_algorithm',
            },
            'frontier_evidence': {
                'candidate_branch_count': 2,
                'candidate_branches': [
                    {'rank': 1, 'target_local_cost': 0, 'target_local_cost_max_radius': 54, 'path_corridor_min_clearance_m': 0.65, 'target_clearance_m': 0.65, 'dispatch_path_local_cost_max': 0},
                    {'rank': 2, 'target_local_cost': 99, 'target_local_cost_max_radius': 99, 'path_corridor_min_clearance_m': 0.35, 'target_clearance_m': 0.35, 'dispatch_path_local_cost_max': 99},
                ],
                'chosen_branch_rank': 1,
                'local_topology': 'junction',
            },
            'topology_evidence': {'local_topology': 'junction'},
        },
        'first_goal_result_artifact': {
            'scan_freshness': {'fresh': True},
            'tf_freshness': {'fresh': True},
            'costmap_freshness': {'fresh': True, 'local_costmap_available': True},
            'local_costmap_samples': [{'seen': True, 'frame_id': 'odom'}],
            'goal_events': [],
        },
    }


def test_phase127_contract_constants_and_phase126_classifications():
    analyzer = _load()
    assert analyzer.PHASE == 'Phase127'
    assert analyzer.MODE == 'first_goal_timeout_artifact_replay'
    assert set(analyzer.CLASSIFICATIONS) == {
        'FIRST_GOAL_TIMEOUT_LOCAL_COST_BLOCKED',
        'FIRST_GOAL_TIMEOUT_CONTROLLER_STALL',
        'FIRST_GOAL_TIMEOUT_GOAL_TOLERANCE_EDGE',
        'FIRST_GOAL_TIMEOUT_CANDIDATE_TOO_RISKY',
        'INSUFFICIENT_TIMEOUT_EVIDENCE',
    }
    assert analyzer.FORBIDDEN_RUNTIME_ACTIONS == {
        'launch_runtime': False,
        'send_navigate_to_pose': False,
        'start_maze_explorer': False,
        'send_second_exploration_goal': False,
        'tune_nav2_mppi_controller_goal_checker_config': False,
        'change_exploration_strategy': False,
        'claim_autonomous_or_exit_success': False,
    }


def test_phase127_parses_timeout_local_cost_line():
    analyzer = _load()
    parsed = analyzer.parse_timeout_local_cost_line('phase23 timeout local cost seq=1 footprint_max=99 front_max=100 path_0_5m_max=73 path_1_0m_max=84')
    assert parsed == {
        'seq': 1,
        'footprint_max': 99,
        'front_max': 100,
        'path_0_5m_max': 73,
        'path_1_0m_max': 84,
    }


def test_phase127_classifies_local_cost_blocked_when_phase125_timeout_costs_are_high():
    analyzer = _load()
    result = analyzer.analyze_phase125_artifact(_base_artifact(), stderr_text='phase23 timeout local cost seq=1 footprint_max=99 front_max=100 path_0_5m_max=73 path_1_0m_max=84')
    assert result['classification'] == 'FIRST_GOAL_TIMEOUT_LOCAL_COST_BLOCKED'
    assert result['evidence']['local_cost']['supports_local_cost_blocked'] is True
    assert result['evidence']['nav2_feedback']['max_recoveries'] == 4
    assert result['evidence']['candidate_risk']['supports_candidate_too_risky'] is False
    assert 'cmd_vel timeline missing' in result['evidence_gaps']
    assert result['guardrails']['no_runtime_replay'] is True
    assert result['guardrails']['no_success_claim'] is True


def test_phase127_classifies_candidate_too_risky_when_selected_candidate_cost_and_clearance_are_bad():
    analyzer = _load()
    artifact = _base_artifact()
    branches = artifact['first_goal']['frontier_evidence']['candidate_branches']
    branches[0].update({'target_local_cost': 99, 'target_local_cost_max_radius': 99, 'path_corridor_min_clearance_m': 0.20, 'target_clearance_m': 0.20, 'dispatch_path_local_cost_max': 99})
    branches[1].update({'target_local_cost': 0, 'target_local_cost_max_radius': 0, 'path_corridor_min_clearance_m': 0.75, 'target_clearance_m': 0.75, 'dispatch_path_local_cost_max': 0})
    result = analyzer.analyze_phase125_artifact(artifact, stderr_text='')
    assert result['classification'] == 'FIRST_GOAL_TIMEOUT_CANDIDATE_TOO_RISKY'
    assert result['evidence']['candidate_risk']['supports_candidate_too_risky'] is True


def test_phase127_can_classify_controller_stall_only_with_velocity_evidence():
    analyzer = _load()
    artifact = _base_artifact()
    artifact['diagnostic_replay_extensions'] = {
        'cmd_vel_timeline': [{'linear_x': 0.25, 'angular_z': 0.7}, {'linear_x': -0.1, 'angular_z': -0.7}],
        'odom_velocity_timeline': [{'linear_x': 0.0, 'angular_z': 0.0}, {'linear_x': 0.01, 'angular_z': 0.0}],
    }
    result = analyzer.analyze_phase125_artifact(artifact, stderr_text='')
    assert result['classification'] == 'FIRST_GOAL_TIMEOUT_CONTROLLER_STALL'
    assert result['evidence']['controller_stall']['supports_controller_stall'] is True


def test_phase127_can_classify_goal_tolerance_edge_only_with_goal_checker_evidence():
    analyzer = _load()
    artifact = _base_artifact()
    artifact['diagnostic_replay_extensions'] = {
        'robot_pose_trace': [{'x': 2.43, 'y': 1.02}, {'x': 2.44, 'y': 1.02}],
        'goal_checker_state': {'xy_tolerance_m': 0.35, 'yaw_tolerance_rad': 0.25, 'distance_error_m': 0.356, 'yaw_error_rad': 0.4},
    }
    result = analyzer.analyze_phase125_artifact(artifact, stderr_text='')
    assert result['classification'] == 'FIRST_GOAL_TIMEOUT_GOAL_TOLERANCE_EDGE'
    assert result['evidence']['goal_tolerance']['supports_goal_tolerance_edge'] is True


def test_phase127_returns_insufficient_when_artifact_lacks_required_discriminating_evidence():
    analyzer = _load()
    artifact = _base_artifact()
    artifact['first_goal']['nav2_feedback_timeline'] = []
    artifact['first_goal_result_artifact']['local_costmap_samples'] = []
    artifact['first_goal']['frontier_evidence']['candidate_branches'] = []
    result = analyzer.analyze_phase125_artifact(artifact, stderr_text='')
    assert result['classification'] == 'INSUFFICIENT_TIMEOUT_EVIDENCE'
    assert result['evidence']['local_cost']['supports_local_cost_blocked'] is False
    assert result['evidence']['controller_stall']['supports_controller_stall'] is False
    assert result['evidence']['candidate_risk']['supports_candidate_too_risky'] is False
    assert 'timeout local cost line missing' in result['evidence_gaps']


def test_phase127_real_phase125_inputs_exist_and_report_contract_after_run():
    assert (PHASE125_LOG / 'phase125_first_exploration_goal_execution_result_smoke_artifact.json').exists()
    assert (PHASE125_LOG / 'phase125_first_exploration_goal_execution_result_smoke_artifact_maze_explorer_stderr.log').exists()
    if REPORT.exists():
        text = REPORT.read_text(encoding='utf-8')
        required = [
            'PHASE127_FIRST_GOAL_TIMEOUT_ARTIFACT_REPLAY_COMPLETE_STOP_BEFORE_PHASE128',
            'diagnosis-only artifact replay',
            'FIRST_GOAL_TIMEOUT_LOCAL_COST_BLOCKED',
            'cmd_vel timeline missing',
            'odom velocity timeline missing',
            'goal checker state missing',
            'No Gazebo/RViz/Nav2 runtime was launched',
            'No NavigateToPose goal was sent',
            'No maze_explorer was started',
            'No Nav2/MPPI/controller/goal checker/config tuning was performed',
            'No autonomous exploration success or exit success is claimed',
            'Phase128 not entered',
        ]
        for phrase in required:
            assert phrase in text
