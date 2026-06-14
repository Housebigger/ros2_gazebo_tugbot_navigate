import json
import math
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
ANALYZER = ROOT / 'tools' / 'analyze_phase26k_spatial_local_cost_alignment.py'


def write_jsonl(path: Path, rows: list[dict]):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('\n'.join(json.dumps({'state': row}) for row in rows) + '\n', encoding='utf-8')


def candidate(angle: float, target: list[float], target_exit: float, rejection: str | None = 'explored', state='explored', **extra) -> dict:
    row = {
        'branch_angle': angle,
        'target': target,
        'target_exit_dist': target_exit,
        'dispatch_path_local_cost_max': 0,
        'target_local_cost': 0,
        'target_local_cost_max_radius': 0,
        'rejection_reason': rejection,
        'state': state,
        'rank': None,
    }
    row.update(extra)
    return row


def dispatch(seq: int, pose: list[float], chosen_target: list[float], explored_target: list[float], chosen_exit=1.4, explored_exit=0.7, **extra) -> dict:
    row = {
        'event': 'dispatch',
        'goal_sequence': seq,
        'goal_kind': 'explore',
        'start_node_id': 1,
        'current_node_id': 1,
        'dispatch_pose': pose,
        'branch_angle': -1.57,
        'target': chosen_target,
        'target_exit_dist': chosen_exit,
        'robot_exit_dist_at_dispatch': 1.0,
        'candidate_branch_count': 2,
        'candidate_branches': [
            candidate(0.0, explored_target, explored_exit, dispatch_path_local_cost_max=0, target_local_cost=0),
            candidate(-1.57, chosen_target, chosen_exit, rejection=None, state='untried', rank=1),
        ],
    }
    row.update(extra)
    return row


def snapshot(seq: int, pose: list[float], cost: int, snapshot_type='periodic_active_goal') -> dict:
    return {
        'event': 'snapshot',
        'snapshot_type': snapshot_type,
        'goal_sequence': seq,
        'robot_pose': pose,
        'path_ahead_0_5m_cost_max': cost,
        'path_ahead_1_0m_cost_max': cost,
        'path_ahead_1_0m_cost_mean': float(cost),
        'robot_to_path_distance_m': 0.02,
        'post_recovery_path_update_count': 3,
        'last_cmd': {'linear_x': 0.0, 'angular_z': 0.0},
    }


def test_phase26k_classifies_anchor_high_cost_as_chosen_route_and_detects_overlap_with_success_case(tmp_path):
    log_dir = tmp_path / 'log'
    write_jsonl(log_dir / 'anchor_goal_events.jsonl', [
        dispatch(8, [0.0, 0.0, 0.0], [0.0, -2.0], [2.0, 0.0]),
        {'event': 'timeout', 'goal_sequence': 8, 'result_reason': 'goal_timeout'},
    ])
    write_jsonl(log_dir / 'anchor_post_recovery_snapshots.jsonl', [
        snapshot(8, [0.0, -1.0, -math.pi / 2], 100, 'near_zero_onset'),
        snapshot(8, [0.0, -1.5, -math.pi / 2], 100),
    ])

    write_jsonl(log_dir / 'success_goal_events.jsonl', [
        dispatch(10, [0.0, 0.0, 0.0], [0.0, -2.0], [2.0, 0.0]),
        {'event': 'success', 'goal_sequence': 10, 'result_reason': 'succeeded'},
    ])
    write_jsonl(log_dir / 'success_post_recovery_snapshots.jsonl', [
        snapshot(10, [0.0, -1.1, -math.pi / 2], 100, 'near_zero_onset'),
    ])

    output = tmp_path / 'phase26k.json'
    result = subprocess.run([
        sys.executable,
        str(ANALYZER),
        '--log-dir', str(log_dir),
        '--anchor-run', 'anchor',
        '--anchor-seq', '8',
        '--compare-cases', 'success:10:success_same_choke',
        '--output-json', str(output),
    ], text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr
    data = json.loads(output.read_text(encoding='utf-8'))
    assert data['phase'] == '26K'
    assert data['analysis_only'] is True
    assert 'do_not_enter_phase27' in data['decision']['guardrails']

    anchor = data['anchor_case']
    assert anchor['near_zero_alignment']['classification'] == 'chosen_route'
    assert anchor['high_cost_distribution']['classification'] == 'chosen_route_dominant'
    assert anchor['explored_candidate_corridor_clean'] is True
    assert anchor['route_context'] == 'route_divergence_chosen_moves_away_rejected_moves_toward_exit'

    compare = data['comparison_cases'][0]
    assert compare['label'] == 'success_same_choke'
    assert compare['min_high_cost_overlap_with_anchor_m'] <= 0.2
    assert data['decision']['spatial_interpretation'] == 'chosen_route_high_cost_with_success_overlap_not_explored_corridor'
    assert data['decision']['phase27_candidate_signal'] == 'not_supported'
    assert data['decision']['next_recommendation'] == 'analysis_only_consider_runtime_spatial_diagnostics_not_branch_selection'


def test_phase26k_distinguishes_shared_dispatch_choke_from_chosen_route_only(tmp_path):
    log_dir = tmp_path / 'log'
    write_jsonl(log_dir / 'shared_goal_events.jsonl', [
        dispatch(3, [0.0, 0.0, 0.0], [1.0, 0.0], [0.0, 1.0], chosen_exit=1.1, explored_exit=0.8),
        {'event': 'timeout', 'goal_sequence': 3, 'result_reason': 'goal_timeout'},
    ])
    write_jsonl(log_dir / 'shared_post_recovery_snapshots.jsonl', [
        snapshot(3, [0.05, 0.05, math.pi / 4], 99, 'near_zero_onset'),
    ])
    output = tmp_path / 'shared.json'
    result = subprocess.run([
        sys.executable,
        str(ANALYZER),
        '--log-dir', str(log_dir),
        '--anchor-run', 'shared',
        '--anchor-seq', '3',
        '--output-json', str(output),
    ], text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr
    data = json.loads(output.read_text(encoding='utf-8'))
    anchor = data['anchor_case']
    assert anchor['near_zero_alignment']['classification'] == 'shared_dispatch_or_corridor_choke'
    assert anchor['explored_candidate_corridor_clean'] is False
    assert data['decision']['spatial_interpretation'] == 'shared_choke_or_ambiguous'
