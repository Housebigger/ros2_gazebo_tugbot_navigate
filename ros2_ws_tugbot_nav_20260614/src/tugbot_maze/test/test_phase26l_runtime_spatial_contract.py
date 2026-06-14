import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
RECORDER = ROOT / 'tools' / 'record_post_recovery_snapshots.py'
ANALYZER = ROOT / 'tools' / 'analyze_phase26l_runtime_spatial_contract.py'


def write_jsonl(path: Path, rows: list[dict]):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('\n'.join(json.dumps({'state': row}) for row in rows) + '\n', encoding='utf-8')


def test_phase26l_recorder_source_contract_exposes_high_cost_cell_and_corridor_spatial_fields():
    source = RECORDER.read_text(encoding='utf-8')
    required_tokens = [
        'path_ahead_1_0m_high_cost_points',
        'path_ahead_1_0m_high_cost_count',
        'path_ahead_1_0m_high_cost_centroid',
        'path_ahead_1_0m_nearest_high_cost_point',
        'path_ahead_1_0m_first_high_cost_distance_m',
        'chosen_route_corridor_cost_max',
        'chosen_route_corridor_cost_mean',
        'chosen_route_first_high_cost_distance_m',
        'explored_candidate_corridor_cost_max',
        'explored_candidate_corridor_cost_mean',
        'explored_candidate_first_high_cost_distance_m',
        'selected_explored_candidate_target',
        'max-high-cost-points',
        '_line_samples',
        '_high_cost_summary',
        '_route_corridor_summary',
    ]
    for token in required_tokens:
        assert token in source, token


def test_phase26l_contract_analyzer_gates_phase27_only_when_chosen_route_high_and_explored_corridor_clean(tmp_path):
    log_dir = tmp_path / 'log'
    write_jsonl(log_dir / 'candidate_goal_events.jsonl', [
        {
            'event': 'dispatch',
            'goal_sequence': 8,
            'robot_exit_dist_at_dispatch': 1.0,
            'target_exit_dist': 1.4,
            'target': [0.0, -2.0],
            'candidate_branches': [
                {'rejection_reason': 'explored', 'target': [2.0, 0.0], 'target_exit_dist': 0.6},
                {'rejection_reason': None, 'rank': 1, 'target': [0.0, -2.0], 'target_exit_dist': 1.4},
            ],
        },
        {'event': 'timeout', 'goal_sequence': 8, 'result_reason': 'goal_timeout'},
    ])
    write_jsonl(log_dir / 'candidate_post_recovery_snapshots.jsonl', [
        {
            'event': 'snapshot',
            'snapshot_type': 'near_zero_onset',
            'goal_sequence': 8,
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
        }
    ])
    output = tmp_path / 'phase26l.json'
    result = subprocess.run([
        sys.executable,
        str(ANALYZER),
        '--log-dir', str(log_dir),
        '--failed-runs', 'candidate',
        '--output-json', str(output),
    ], text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr
    data = json.loads(output.read_text(encoding='utf-8'))
    assert data['phase'] == '26L'
    assert data['analysis_only'] is True
    assert 'do_not_enter_phase27_from_contract_only' in data['decision']['guardrails']

    case = data['runs']['candidate']['cases'][0]
    assert case['goal_sequence'] == 8
    assert case['route_context'] == 'route_divergence_chosen_moves_away_rejected_moves_toward_exit'
    assert case['spatial_contract_complete'] is True
    assert case['chosen_route_high_cost'] is True
    assert case['explored_candidate_corridor_clean'] is True
    assert case['path_ahead_high_cost_cells_present'] is True
    assert case['classification'] == 'chosen_high_explored_clean_contract_signal'

    assert data['comparison']['failed_runs']['chosen_high_explored_clean_contract_signal_count'] == 1
    assert data['decision']['phase27_candidate_signal'] == 'possible_only_after_real_matched_repeat_validation'
    assert data['decision']['next_recommendation'] == 'collect_real_phase26l_artifacts_before_any_phase27'


def test_phase26l_contract_analyzer_rejects_missing_high_cost_coordinates_and_dirty_explored_corridor(tmp_path):
    log_dir = tmp_path / 'log'
    write_jsonl(log_dir / 'dirty_goal_events.jsonl', [
        {
            'event': 'dispatch',
            'goal_sequence': 4,
            'robot_exit_dist_at_dispatch': 1.0,
            'target_exit_dist': 1.5,
            'target': [0.0, -1.0],
            'candidate_branches': [{'rejection_reason': 'explored', 'target': [1.0, 0.0], 'target_exit_dist': 0.7}],
        },
        {'event': 'timeout', 'goal_sequence': 4, 'result_reason': 'goal_timeout'},
    ])
    write_jsonl(log_dir / 'dirty_post_recovery_snapshots.jsonl', [
        {
            'event': 'snapshot',
            'snapshot_type': 'near_zero_onset',
            'goal_sequence': 4,
            'path_ahead_1_0m_cost_max': 100,
            'chosen_route_corridor_cost_max': 100,
            'explored_candidate_corridor_cost_max': 99,
        }
    ])
    output = tmp_path / 'dirty.json'
    result = subprocess.run([
        sys.executable,
        str(ANALYZER),
        '--log-dir', str(log_dir),
        '--failed-runs', 'dirty',
        '--output-json', str(output),
    ], text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr
    data = json.loads(output.read_text(encoding='utf-8'))
    case = data['runs']['dirty']['cases'][0]
    assert case['spatial_contract_complete'] is False
    assert case['path_ahead_high_cost_cells_present'] is False
    assert case['explored_candidate_corridor_clean'] is False
    assert case['classification'] == 'insufficient_or_dirty_explored_corridor'
    assert data['decision']['phase27_candidate_signal'] == 'not_supported'
