import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
ANALYZER = ROOT / 'tools' / 'analyze_phase26i_forward_shift_stats.py'


def write_jsonl(path: Path, rows: list[dict]):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('\n'.join(json.dumps({'state': row}) for row in rows) + '\n', encoding='utf-8')


def candidate(angle: float, target: list[float], target_exit: float, rejection: str | None = 'explored', state='explored', **extra) -> dict:
    row = {
        'branch_angle': angle,
        'target': target,
        'target_exit_dist': target_exit,
        'target_clearance_m': 0.8,
        'path_corridor_min_clearance_m': 0.7,
        'dispatch_path_local_cost_max': 0,
        'dispatch_path_local_cost_mean': 0.0,
        'target_local_cost': 0,
        'target_local_cost_max_radius': 0,
        'is_reverse_candidate': False,
        'is_backtrack_context': False,
        'is_near_exit_candidate': target_exit <= 1.0,
        'rejection_reason': rejection,
        'rank': None,
        'state': state,
    }
    row.update(extra)
    return row


def dispatch(seq: int, start_node: int, angle: float, target: list[float], target_exit: float, candidates: list[dict] | None = None, **extra) -> dict:
    row = {
        'event': 'dispatch',
        'goal_sequence': seq,
        'goal_kind': 'explore',
        'start_node_id': start_node,
        'current_node_id': start_node,
        'branch_angle': angle,
        'target': target,
        'target_exit_dist': target_exit,
        'robot_exit_dist_at_dispatch': target_exit + 0.4,
        'candidate_branch_count': len(candidates or []),
        'candidate_branches': candidates or [],
        'dispatch_path_local_cost_max': 0,
        'dispatch_target_local_cost': 0,
        'dispatch_target_local_cost_max_radius': 0,
    }
    row.update(extra)
    return row


def explorer_final(path: Path, mode: str, exit_distance: float):
    write_jsonl(path, [
        {'mode': 'NAVIGATING', 'exit_distance_m': 2.0},
        {'mode': mode, 'exit_distance_m': exit_distance},
    ])


def test_phase26i_counts_same_start_explored_forward_shift_and_splits_cost_classes(tmp_path):
    log_dir = tmp_path / 'log'
    write_jsonl(log_dir / 'failed_clean_goal_events.jsonl', [
        dispatch(1, start_node=7, angle=0.10, target=[1.0, 0.0], target_exit=1.20),
        {'event': 'success', 'goal_sequence': 1, 'result_reason': 'succeeded'},
        dispatch(
            2,
            start_node=7,
            angle=-1.20,
            target=[0.4, -0.8],
            target_exit=1.80,
            candidates=[
                candidate(0.14, [1.5, 0.0], 0.80),
                candidate(-1.20, [0.4, -0.8], 1.80, rejection=None, state='untried', rank=1),
            ],
        ),
        {'event': 'timeout', 'goal_sequence': 2, 'result_reason': 'goal_timeout'},
    ])
    explorer_final(log_dir / 'failed_clean_explorer_state.jsonl', 'FAILED_EXHAUSTED', 1.7)

    write_jsonl(log_dir / 'exit_high_goal_events.jsonl', [
        dispatch(1, start_node=9, angle=0.20, target=[2.0, 0.0], target_exit=1.10),
        {'event': 'success', 'goal_sequence': 1, 'result_reason': 'succeeded'},
        dispatch(
            2,
            start_node=9,
            angle=1.20,
            target=[2.0, 0.8],
            target_exit=0.60,
            candidates=[
                candidate(0.25, [2.4, 0.0], 0.70, dispatch_path_local_cost_max=99, target_local_cost=99),
                candidate(1.20, [2.0, 0.8], 0.60, rejection=None, state='untried', rank=1),
            ],
        ),
        {'event': 'terminal_cancel', 'goal_sequence': 2, 'result_reason': 'exit_reached'},
    ])
    explorer_final(log_dir / 'exit_high_explorer_state.jsonl', 'EXIT_REACHED', 0.4)

    output = tmp_path / 'phase26i.json'
    result = subprocess.run([
        sys.executable,
        str(ANALYZER),
        '--log-dir', str(log_dir),
        '--failed-runs', 'failed_clean',
        '--success-runs', 'exit_high',
        '--output-json', str(output),
    ], text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr

    data = json.loads(output.read_text(encoding='utf-8'))
    assert data['phase'] == '26I'
    assert data['analysis_only'] is True
    assert 'do_not_change_branch_selection' in data['decision']['guardrails']

    failed_case = data['runs']['failed_clean']['forward_shift_cases'][0]
    assert failed_case['source_goal_sequence'] == 1
    assert failed_case['later_goal_sequence'] == 2
    assert failed_case['start_node_id'] == 7
    assert failed_case['source_target_exit_dist'] == 1.2
    assert failed_case['later_explored_candidate_target_exit_dist'] == 0.8
    assert failed_case['target_exit_dist_delta_m'] == -0.4
    assert failed_case['target_forward_shift_distance_m'] == 0.5
    assert failed_case['angle_delta_rad'] < 0.1
    assert failed_case['cost_classification'] == 'clean_low_cost'
    assert failed_case['closer_to_exit'] is True

    assert data['runs']['failed_clean']['summary']['same_start_explored_forward_shift_count'] == 1
    assert data['runs']['failed_clean']['summary']['clean_low_cost_explored_closer_count'] == 1
    assert data['runs']['exit_high']['summary']['high_local_cost_explored_closer_count'] == 1
    assert data['comparison']['FAILED_EXHAUSTED']['clean_low_cost_explored_closer_count'] == 1
    assert data['comparison']['EXIT_REACHED']['clean_low_cost_explored_closer_count'] == 0
    assert data['decision']['stable_clean_low_cost_failed_only_signal'] is True
    assert data['decision']['phase27_candidate_signal'] == 'possible_but_requires_real_run_repeats'


def test_phase26i_does_not_treat_different_start_node_or_not_closer_as_forward_shift_signal(tmp_path):
    log_dir = tmp_path / 'log'
    write_jsonl(log_dir / 'mixed_goal_events.jsonl', [
        dispatch(1, start_node=3, angle=0.0, target=[0.0, 0.0], target_exit=0.9),
        {'event': 'success', 'goal_sequence': 1, 'result_reason': 'succeeded'},
        dispatch(2, start_node=4, angle=1.0, target=[0.0, 1.0], target_exit=1.2, candidates=[
            candidate(0.02, [0.6, 0.0], 0.6),
        ]),
        dispatch(3, start_node=3, angle=1.0, target=[0.0, 2.0], target_exit=1.4, candidates=[
            candidate(0.03, [0.8, 0.0], 1.1),
        ]),
    ])
    explorer_final(log_dir / 'mixed_explorer_state.jsonl', 'FAILED_EXHAUSTED', 2.0)
    output = tmp_path / 'phase26i_mixed.json'
    result = subprocess.run([
        sys.executable,
        str(ANALYZER),
        '--log-dir', str(log_dir),
        '--failed-runs', 'mixed',
        '--output-json', str(output),
    ], text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr
    data = json.loads(output.read_text(encoding='utf-8'))
    assert data['runs']['mixed']['summary']['same_start_explored_forward_shift_count'] == 1
    assert data['runs']['mixed']['summary']['clean_low_cost_explored_closer_count'] == 0
    assert data['decision']['stable_clean_low_cost_failed_only_signal'] is False
    assert data['decision']['phase27_candidate_signal'] == 'not_supported'
