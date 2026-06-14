import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
ANALYZER = ROOT / 'tools' / 'compare_phase26b_failed_vs_success_runs.py'


def write_json(path: Path, data: dict):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data) + '\n', encoding='utf-8')


def write_jsonl(path: Path, rows: list[dict]):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('\n'.join(json.dumps(row) for row in rows) + '\n', encoding='utf-8')


def write_run(log_dir: Path, run_id: str, mode: str, exit_distance: float, events: list[dict], subtypes: list[dict], recoveries: list[dict]):
    write_jsonl(log_dir / f'{run_id}_explorer_state.jsonl', [{
        'state': {
            'mode': mode,
            'goal_count': max(row['goal_sequence'] for row in events),
            'goal_success_count': sum(1 for row in events if row['event'] == 'success'),
            'goal_failure_count': sum(1 for row in events if row['event'] == 'timeout'),
            'timeout_cancel_count': sum(1 for row in events if row['event'] == 'timeout'),
            'blocked_branch_count': 0,
            'blacklisted_goal_count': 0,
            'exit_distance_m': exit_distance,
        }
    }])
    write_jsonl(log_dir / f'{run_id}_goal_events.jsonl', [{'state': row} for row in events])
    write_json(log_dir / f'{run_id}_timeout_subtypes.json', {
        'summary': {
            'timeout_count': len(subtypes),
            'controller_subtype_counts': {
                'footprint_path_blocked_late_silent': sum(1 for row in subtypes if row['controller_subtype'] == 'footprint_path_blocked_late_silent'),
            },
        },
        'timeout_subtypes': subtypes,
    })
    write_json(log_dir / f'{run_id}_failure_windows.json', {'summary': {}, 'failure_windows': [
        {
            'goal_sequence': row['goal_sequence'],
            'classification': row['classification'],
            'timeout_path_ahead_1_0m_cost_max': row['reasons']['path_ahead_1_0m_cost_max'],
            'timeout_footprint_cost_max': row['reasons']['footprint_cost_max'],
            'near_zero_cmd_duration_sec': row['near_zero_cmd_duration_sec'],
        }
        for row in subtypes
    ]})
    write_json(log_dir / f'{run_id}_post_recovery_enriched.json', {
        'summary': {
            'controller_received_path_but_cmd_near_zero_count': sum(1 for row in recoveries if row.get('controller_received_path_but_cmd_near_zero')),
            'recovery_count': len(recoveries),
        },
        'enriched_recovery_snapshots': recoveries,
    })


def test_phase26c_alignment_explains_high_exit_distance_as_route_divergence_not_missing_path_refresh(tmp_path):
    log_dir = tmp_path / 'log'
    success_events = [
        {'event': 'success', 'goal_sequence': 8, 'robot_exit_dist_at_dispatch': 1.1, 'target_exit_dist': 0.7, 'target': [3.5, 3.6], 'branch_angle': 0.1, 'near_exit': True},
        {'event': 'terminal_cancel', 'goal_sequence': 9, 'robot_exit_dist_at_dispatch': 0.7, 'target_exit_dist': 1.0, 'target': [3.0, 3.3], 'branch_angle': 1.7, 'near_exit': False, 'result_reason': 'exit_reached'},
    ]
    failed_events = [
        {'event': 'success', 'goal_sequence': 9, 'robot_exit_dist_at_dispatch': 1.71, 'target_exit_dist': 1.02, 'target': [3.2, 3.6], 'branch_angle': 0.26, 'near_exit': False},
        {'event': 'timeout', 'goal_sequence': 10, 'robot_exit_dist_at_dispatch': 1.09, 'target_exit_dist': 1.13, 'target': [2.9, 2.6], 'branch_angle': -1.39, 'near_exit': False},
        {'event': 'success', 'goal_sequence': 11, 'robot_exit_dist_at_dispatch': 0.76, 'target_exit_dist': 1.22, 'target': [2.9, 3.5], 'branch_angle': 1.75, 'near_exit': False},
        {'event': 'success', 'goal_sequence': 12, 'robot_exit_dist_at_dispatch': 1.29, 'target_exit_dist': 1.21, 'target': [2.9, 2.5], 'branch_angle': -1.18, 'near_exit': False},
    ]
    failed_subtypes = [{
        'goal_sequence': 10,
        'classification': 'healthy_motion_but_late_stall',
        'controller_subtype': 'footprint_path_blocked_late_silent',
        'timing_subtype': 'cmd_silent_after_recovery_abort',
        'near_zero_cmd_duration_sec': 8.4,
        'reasons': {
            'footprint_cost_max': 99,
            'path_ahead_1_0m_cost_max': 99,
            'front_wedge_cost_max': 99,
            'near_zero_cmd_to_first_progress_failure_sec': 10.5,
        },
    }]
    write_run(log_dir, 'phase26b_candidate_run1', 'EXIT_REACHED', 0.57, success_events, [], [])
    write_run(log_dir, 'phase26b_candidate_run2', 'FAILED_EXHAUSTED', 1.33, failed_events, failed_subtypes, [
        {
            'goal_sequence': 10,
            'path_update_count_after_recovery': 3,
            'near_zero_path_ahead_1_0m_cost_max': 99,
            'near_zero_robot_to_path_distance_m': 0.07,
            'controller_received_path_but_cmd_near_zero': False,
        }
    ])
    output = tmp_path / 'alignment.json'
    result = subprocess.run([
        sys.executable,
        str(ANALYZER),
        '--log-dir', str(log_dir),
        '--runs', 'phase26b_candidate_run1,phase26b_candidate_run2',
        '--focus-run', 'phase26b_candidate_run2',
        '--output-json', str(output),
    ], text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr
    data = json.loads(output.read_text(encoding='utf-8'))
    focus = data['focus_run_analysis']
    assert focus['run_id'] == 'phase26b_candidate_run2'
    assert focus['primary_explanation'] == 'route_divergence_after_near_exit_timeout_with_persistent_local_cost_pressure'
    assert focus['high_exit_distance_without_high_timeout_count'] is True
    assert focus['late_successes_moving_away_from_exit_count'] == 1
    assert focus['post_recovery_path_refresh_absent'] is False
    assert focus['severe_late_silent_sequences'] == [10]
    assert data['alignment_summary']['failed_run_count'] == 1
    assert data['alignment_summary']['exit_reached_run_count'] == 1
