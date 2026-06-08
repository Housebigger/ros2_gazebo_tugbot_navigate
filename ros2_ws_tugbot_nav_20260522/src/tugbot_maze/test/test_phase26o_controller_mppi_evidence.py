import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
ANALYZER = ROOT / 'tools' / 'analyze_phase26o_controller_mppi_evidence.py'


def write_json(path: Path, payload: dict):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, sort_keys=True) + '\n', encoding='utf-8')


def test_phase26o_reports_controller_active_fresh_paths_and_mppi_critic_instrumentation_gap(tmp_path):
    log_dir = tmp_path / 'log'
    run_id = 'synthetic'
    write_json(log_dir / 'phase26n_goal_timeline.json', {
        'phase': '26N',
        'cases': [{
            'run_id': run_id,
            'goal_sequence': 2,
            'cmd_near_zero_relation': {
                'recovery_time': 100.0,
                'progress_failure_time': 99.9,
                'controller_abort_time': 99.91,
                'first_cmd_near_zero_time': 100.3,
                'first_cmd_near_zero_after_recovery_sec': 0.3,
            },
            'path_update_cadence': {'path_update_count_after_recovery': 3},
            'local_cost_windows': {
                'high_cost_window_count_after_recovery': 2,
                'first_high_cost_window_after_recovery_sec': 0.1,
            },
        }],
    })
    params_file = tmp_path / 'nav2_params.yaml'
    params_file.write_text('''
controller_server:
  ros__parameters:
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      visualize: true
      publish_critics_stats: false
      publish_optimal_trajectory: false
      TrajectoryVisualizer:
        trajectory_step: 5
        time_step: 3
''', encoding='utf-8')
    write_json(log_dir / f'{run_id}_params_fingerprint.json', {
        'run_id': run_id,
        'params_file': {'path': str(params_file)},
    })
    launch_log = log_dir / f'{run_id}_launch.log'
    launch_log.write_text('\n'.join([
        '[controller_server-7] [INFO] [90.000000000] [controller_server]: Created controller : FollowPath of type nav2_mppi_controller::MPPIController',
        '[controller_server-7] [INFO] [90.010000000] [controller_server]: Critic loaded : mppi::critics::CostCritic',
        '[controller_server-7] [INFO] [90.020000000] [controller_server]: Critic loaded : mppi::critics::GoalCritic',
        '[controller_server-7] [INFO] [90.030000000] [controller_server]: Critic loaded : mppi::critics::PathAlignCritic',
        '[controller_server-7] [INFO] [90.040000000] [controller_server]: Critic loaded : mppi::critics::PathFollowCritic',
        '[controller_server-7] [INFO] [90.050000000] [MPPIController]: Activated MPPI Controller: FollowPath',
        '[controller_server-7] [ERROR] [99.900000000] [controller_server]: Failed to make progress',
        '[controller_server-7] [WARN] [99.910000000] [controller_server]: [follow_path] [ActionServer] Aborting handle.',
        '[controller_server-7] [INFO] [100.000000000] [local_costmap.local_costmap]: Received request to clear entirely the local_costmap',
        '[controller_server-7] [INFO] [100.020000000] [controller_server]: Received a goal, begin computing control effort.',
        '[controller_server-7] [INFO] [100.120000000] [controller_server]: Passing new path to controller.',
        '[controller_server-7] [INFO] [101.120000000] [controller_server]: Passing new path to controller.',
        '[controller_server-7] [INFO] [102.120000000] [controller_server]: Passing new path to controller.',
    ]) + '\n', encoding='utf-8')
    output = tmp_path / 'phase26o.json'

    result = subprocess.run([
        sys.executable,
        str(ANALYZER),
        '--log-dir', str(log_dir),
        '--timeline-json', str(log_dir / 'phase26n_goal_timeline.json'),
        '--output-json', str(output),
    ], text=True, capture_output=True, check=False)

    assert result.returncode == 0, result.stderr
    data = json.loads(output.read_text(encoding='utf-8'))
    assert data['phase'] == '26O'
    assert data['analysis_only'] is True
    assert data['decision']['phase27_candidate_signal'] == 'not_supported'
    assert data['decision']['parameter_tuning_signal'] == 'not_supported'
    assert data['decision']['next_recommendation'] == 'add_or_enable_mppi_controller_debug_evidence_before_intervention'

    case = data['cases'][0]
    assert case['run_id'] == run_id
    assert case['goal_sequence'] == 2
    assert case['controller_state']['mppi_controller_configured'] is True
    assert case['controller_state']['mppi_controller_activated'] is True
    assert case['controller_state']['received_goal_after_recovery'] is True
    assert case['plan_freshness']['path_updates_after_recovery_count'] == 3
    assert case['plan_freshness']['first_path_update_after_recovery_sec'] == 0.12
    assert case['mppi_critic_evidence']['loaded_critics'] == [
        'CostCritic', 'GoalCritic', 'PathAlignCritic', 'PathFollowCritic'
    ]
    assert case['mppi_critic_evidence']['per_cycle_critic_scores_present'] is False
    assert case['mppi_critic_evidence']['trajectory_validity_logs_present'] is False
    assert case['mppi_critic_evidence']['zero_velocity_reason_logs_present'] is False
    assert case['mppi_critic_evidence']['params']['visualize'] is True
    assert case['mppi_critic_evidence']['params']['publish_critics_stats'] is False
    assert case['mppi_critic_evidence']['params']['publish_optimal_trajectory'] is False
    assert case['inference'] == 'controller_active_and_fresh_paths_but_no_per_cycle_mppi_critic_evidence'
    assert 'mppi_critic_scores_or_selected_trajectory_costs' in case['instrumentation_gaps']
    assert case['local_cost_vs_cmd_timing']['high_cost_before_first_cmd_near_zero'] is True
