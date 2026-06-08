import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
ANALYZER = ROOT / 'tools' / 'analyze_phase26d_branch_choice_snapshots.py'


def write_jsonl(path: Path, rows: list[dict]):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('\n'.join(json.dumps({'state': row}) for row in rows) + '\n', encoding='utf-8')


def test_phase26d_branch_choice_analyzer_marks_exit_as_success_and_exposes_missing_candidate_rank(tmp_path):
    log_dir = tmp_path / 'log'
    write_jsonl(log_dir / 'phase26b_candidate_run2_goal_events.jsonl', [
        {
            'event': 'success',
            'goal_sequence': 9,
            'current_node_id': 8,
            'local_topology': 'junction',
            'last_candidate_count': 4,
            'last_open_direction_count': 4,
            'robot_exit_dist_at_dispatch': 1.71,
            'target_exit_dist': 1.02,
            'target': [3.2, 3.6],
            'branch_angle': 0.26,
            'near_exit': False,
            'dispatch_path_local_cost_max': 0,
            'target_clearance_m': 0.7,
        },
        {
            'event': 'timeout',
            'goal_sequence': 10,
            'current_node_id': 8,
            'local_topology': 'junction',
            'last_candidate_count': 3,
            'last_open_direction_count': 3,
            'robot_exit_dist_at_dispatch': 1.09,
            'target_exit_dist': 1.13,
            'target': [2.9, 2.6],
            'branch_angle': -1.39,
            'near_exit': False,
            'dispatch_path_local_cost_max': 0,
            'target_clearance_m': 0.85,
            'timeout_path_ahead_1_0m_cost_max': 99,
        },
        {
            'event': 'success',
            'goal_sequence': 11,
            'current_node_id': 9,
            'local_topology': 'junction',
            'last_candidate_count': 3,
            'last_open_direction_count': 3,
            'robot_exit_dist_at_dispatch': 0.76,
            'target_exit_dist': 1.22,
            'target': [2.9, 3.5],
            'branch_angle': 1.75,
            'near_exit': False,
            'dispatch_path_local_cost_max': 48,
            'target_clearance_m': 1.0,
        },
    ])
    write_jsonl(log_dir / 'phase26b_candidate_run1_goal_events.jsonl', [
        {
            'event': 'timeout',
            'goal_sequence': 8,
            'current_node_id': 7,
            'local_topology': 'junction',
            'last_candidate_count': 3,
            'last_open_direction_count': 3,
            'robot_exit_dist_at_dispatch': 1.38,
            'target_exit_dist': 0.57,
            'target': [3.4, 3.0],
            'branch_angle': 0.33,
            'near_exit': True,
            'dispatch_path_local_cost_max': 73,
            'target_clearance_m': 0.5,
            'timeout_path_ahead_1_0m_cost_max': 100,
        },
        {
            'event': 'terminal_cancel',
            'goal_sequence': 9,
            'current_node_id': 8,
            'local_topology': 'corridor',
            'last_candidate_count': 2,
            'last_open_direction_count': 2,
            'robot_exit_dist_at_dispatch': 0.65,
            'target_exit_dist': 1.14,
            'target': [2.9, 3.4],
            'branch_angle': 1.89,
            'near_exit': False,
            'result_reason': 'exit_reached',
        },
    ])
    output = tmp_path / 'branch_alignment.json'
    result = subprocess.run([
        sys.executable,
        str(ANALYZER),
        '--log-dir', str(log_dir),
        '--runs', 'phase26b_candidate_run1,phase26b_candidate_run2',
        '--focus-run', 'phase26b_candidate_run2',
        '--focus-timeout-seq', '10',
        '--output-json', str(output),
    ], text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr
    data = json.loads(output.read_text(encoding='utf-8'))
    assert data['success_definition']['success_condition'] == 'robot_reaches_exit_coordinates'
    assert data['success_definition']['full_map_exploration_required'] is False
    focus = data['focus_run_analysis']
    assert focus['post_timeout_choice']['goal_sequence'] == 11
    assert focus['post_timeout_choice']['exit_progress_delta_m'] < 0
    assert focus['post_timeout_choice']['classification'] == 'successful_branch_moved_away_from_exit'
    assert focus['pre_timeout_choice']['classification'] == 'exit_progress_branch'
    assert focus['comparison_to_success_runs']['success_runs_reached_exit_after_away_branch_count'] == 1
    assert focus['artifact_gaps']['chosen_branch_rank'] is True
    assert focus['artifact_gaps']['rejected_branch_summary'] is True
    assert focus['diagnosis'] == 'post_timeout_branch_choice_moved_away_from_exit_but_candidate_ranking_missing'
