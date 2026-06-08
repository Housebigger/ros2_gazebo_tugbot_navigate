import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
ANALYZER = ROOT / 'tools' / 'analyze_phase26n_goal_timeline.py'


def write_json(path: Path, payload: dict):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, sort_keys=True) + '\n', encoding='utf-8')


def write_jsonl(path: Path, rows: list[dict]):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('\n'.join(json.dumps(row, sort_keys=True) for row in rows) + '\n', encoding='utf-8')


def prepare_timeline_fixture(log_dir: Path):
    run_id = 'synthetic'
    seq = 2
    write_jsonl(log_dir / f'{run_id}_goal_events.jsonl', [
        {'event': 'dispatch', 'goal_sequence': seq, 'wall_time': 10.0},
        {'event': 'timeout', 'goal_sequence': seq, 'wall_time': 45.0, 'result_reason': 'goal_timeout'},
    ])
    write_json(log_dir / f'{run_id}_goal_nav2_analysis.json', {
        'goals': [{
            'goal_sequence': seq,
            'outcome': 'timeout',
            'clear_costmap_times': [30.0],
            'progress_failure_times': [37.0],
            'controller_abort_times': [38.0],
        }]
    })
    write_json(log_dir / f'{run_id}_timeout_subtypes.json', {
        'timeouts': [{
            'goal_sequence': seq,
            'combined_subtype': 'side_cost_or_timing_late_silent+cmd_silent_after_recovery_abort',
            'timing_subtype': 'cmd_silent_after_recovery_abort',
        }]
    })
    write_jsonl(log_dir / f'{run_id}_post_recovery_snapshots.jsonl', [
        {'event': 'path_update', 'goal_sequence': seq, 'wall_time': 31.0},
        {'event': 'path_update', 'goal_sequence': seq, 'wall_time': 32.0},
        {'event': 'snapshot', 'snapshot_type': 'periodic_active_goal', 'goal_sequence': seq, 'wall_time': 32.0,
         'path_ahead_1_0m_cost_max': 100, 'path_ahead_1_0m_high_cost_count': 5,
         'path_ahead_1_0m_first_high_cost_distance_m': 0.5, 'robot_to_path_distance_m': 0.04,
         'chosen_route_corridor_cost_max': 40, 'explored_candidate_corridor_cost_max': None},
        {'event': 'snapshot', 'snapshot_type': 'near_zero_onset', 'goal_sequence': seq, 'wall_time': 31.2,
         'path_ahead_1_0m_cost_max': 99, 'path_ahead_1_0m_high_cost_count': 4,
         'path_ahead_1_0m_first_high_cost_distance_m': 0.6, 'robot_to_path_distance_m': 0.08,
         'chosen_route_corridor_cost_max': 40, 'explored_candidate_corridor_cost_max': None},
    ])
    write_jsonl(log_dir / f'{run_id}_controller_dynamics.jsonl', [
        {'source': 'cmd_vel', 'goal_sequence': seq, 'wall_time': 30.5, 'linear_x': 0.0, 'angular_z': 0.0},
        {'source': 'cmd_vel', 'goal_sequence': seq, 'wall_time': 31.0, 'linear_x': 0.0, 'angular_z': 0.0},
        {'source': 'cmd_vel', 'goal_sequence': seq, 'wall_time': 35.0, 'linear_x': 0.0, 'angular_z': 0.0},
    ])
    return run_id, seq


def test_phase26n_timeline_orders_goal_events_and_classifies_cmd_near_zero_before_progress_and_abort(tmp_path):
    log_dir = tmp_path / 'log'
    run_id, seq = prepare_timeline_fixture(log_dir)
    output = tmp_path / 'timeline.json'
    result = subprocess.run([
        sys.executable,
        str(ANALYZER),
        '--log-dir', str(log_dir),
        '--case', f'{run_id}:{seq}',
        '--output-json', str(output),
    ], text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr
    data = json.loads(output.read_text(encoding='utf-8'))
    assert data['phase'] == '26N'
    assert data['analysis_only'] is True
    assert data['decision']['phase27_candidate_signal'] == 'not_supported'
    assert data['decision']['next_recommendation'] == 'inspect_controller_mppi_critic_logs_before_intervention'

    case = data['cases'][0]
    assert case['run_id'] == run_id
    assert case['goal_sequence'] == seq
    assert case['classification'] == 'cmd_near_zero_starts_before_progress_failure_and_abort_while_paths_continue'
    assert case['cmd_near_zero_relation']['first_cmd_near_zero_after_recovery_sec'] == 0.5
    assert case['cmd_near_zero_relation']['cmd_near_zero_before_progress_failure'] is True
    assert case['cmd_near_zero_relation']['cmd_near_zero_before_controller_abort'] is True
    assert case['path_update_cadence']['path_update_count_after_recovery'] == 2
    assert case['local_cost_windows']['high_cost_window_count_after_recovery'] == 2
    labels = [event['label'] for event in case['timeline']]
    assert labels == [
        'dispatch',
        'recovery_clear_costmap',
        'first_cmd_near_zero_after_recovery',
        'path_update',
        'near_zero_snapshot',
        'local_cost_high_window',
        'path_update',
        'local_cost_high_window',
        'progress_failure',
        'controller_abort',
        'timeout',
    ]
