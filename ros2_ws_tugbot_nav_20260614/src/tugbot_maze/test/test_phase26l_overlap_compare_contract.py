import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
ANALYZER = ROOT / 'tools' / 'analyze_phase26l_runtime_spatial_contract.py'


def write_jsonl(path: Path, rows: list[dict]):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('\n'.join(json.dumps({'state': row}) for row in rows) + '\n', encoding='utf-8')


def test_phase26l_analyzer_keeps_failed_only_view_when_compare_runs_overlap(tmp_path):
    log_dir = tmp_path / 'log'
    write_jsonl(log_dir / 'same_run_goal_events.jsonl', [
        {
            'event': 'dispatch',
            'goal_sequence': 1,
            'dispatch_pose': [0.0, 0.0, 0.0],
            'robot_exit_dist_at_dispatch': 1.0,
            'target_exit_dist': 1.4,
            'target': [0.0, -2.0],
            'candidate_branches': [{'rejection_reason': 'explored', 'target': [2.0, 0.0], 'target_exit_dist': 0.6}],
        },
        {'event': 'timeout', 'goal_sequence': 1, 'result_reason': 'goal_timeout'},
        {
            'event': 'dispatch',
            'goal_sequence': 2,
            'dispatch_pose': [0.0, 0.0, 0.0],
            'robot_exit_dist_at_dispatch': 1.0,
            'target_exit_dist': 0.5,
            'target': [1.0, 0.0],
            'candidate_branches': [],
        },
        {'event': 'success', 'goal_sequence': 2, 'result_reason': 'succeeded'},
    ])
    write_jsonl(log_dir / 'same_run_post_recovery_snapshots.jsonl', [
        {
            'event': 'snapshot',
            'snapshot_type': 'near_zero_onset',
            'goal_sequence': 1,
            'path_ahead_1_0m_cost_max': 100,
            'path_ahead_1_0m_high_cost_count': 3,
            'path_ahead_1_0m_nearest_high_cost_point': [0.0, -0.6],
            'path_ahead_1_0m_high_cost_centroid': [0.0, -0.8],
            'path_ahead_1_0m_first_high_cost_distance_m': 0.55,
            'chosen_route_corridor_cost_max': 100,
            'chosen_route_corridor_cost_mean': 82.5,
            'chosen_route_first_high_cost_distance_m': 0.45,
            'explored_candidate_corridor_cost_max': 34,
            'explored_candidate_corridor_cost_mean': 8.0,
            'explored_candidate_first_high_cost_distance_m': None,
        },
        {
            'event': 'snapshot',
            'snapshot_type': 'near_zero_onset',
            'goal_sequence': 2,
            'path_ahead_1_0m_cost_max': 100,
            'path_ahead_1_0m_high_cost_count': 2,
            'path_ahead_1_0m_nearest_high_cost_point': [0.3, 0.0],
            'path_ahead_1_0m_high_cost_centroid': [0.5, 0.0],
            'path_ahead_1_0m_first_high_cost_distance_m': 0.3,
            'chosen_route_corridor_cost_max': 100,
            'chosen_route_corridor_cost_mean': 75.0,
            'chosen_route_first_high_cost_distance_m': 0.3,
            'explored_candidate_corridor_cost_max': None,
            'explored_candidate_corridor_cost_mean': None,
            'explored_candidate_first_high_cost_distance_m': None,
        },
    ])
    output = tmp_path / 'overlap.json'
    result = subprocess.run([
        sys.executable,
        str(ANALYZER),
        '--log-dir', str(log_dir),
        '--failed-runs', 'same_run',
        '--compare-runs', 'same_run',
        '--output-json', str(output),
    ], text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr
    data = json.loads(output.read_text(encoding='utf-8'))
    assert data['failed_runs']['same_run']['failed_only'] is True
    assert data['failed_runs']['same_run']['case_count'] == 1
    assert data['compare_runs']['same_run']['failed_only'] is False
    assert data['compare_runs']['same_run']['case_count'] == 2
    assert data['comparison']['failed_runs']['chosen_high_explored_clean_contract_signal_count'] == 1
