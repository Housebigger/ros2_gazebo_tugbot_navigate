import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
ANALYZER = ROOT / 'tools' / 'analyze_phase26j_forward_shift_artifact_join.py'


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


def dispatch(seq: int, start_node: int, angle: float, target: list[float], target_exit: float, robot_exit: float, candidates: list[dict] | None = None, **extra) -> dict:
    row = {
        'event': 'dispatch',
        'goal_sequence': seq,
        'goal_kind': 'explore',
        'start_node_id': start_node,
        'current_node_id': start_node,
        'branch_angle': angle,
        'target': target,
        'target_exit_dist': target_exit,
        'robot_exit_dist_at_dispatch': robot_exit,
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


def test_phase26j_joins_forward_shift_cases_with_outcome_timeout_and_post_recovery_artifacts(tmp_path):
    log_dir = tmp_path / 'log'
    write_jsonl(log_dir / 'failed_high_goal_events.jsonl', [
        dispatch(1, start_node=7, angle=0.10, target=[1.0, 0.0], target_exit=1.20, robot_exit=1.60),
        {'event': 'success', 'goal_sequence': 1, 'result_reason': 'succeeded'},
        dispatch(
            2,
            start_node=7,
            angle=-1.20,
            target=[0.4, -0.8],
            target_exit=1.80,
            robot_exit=1.20,
            candidates=[
                candidate(0.14, [1.5, 0.0], 0.80, dispatch_path_local_cost_max=99, target_local_cost=99),
                candidate(-1.20, [0.4, -0.8], 1.80, rejection=None, state='untried', rank=1),
            ],
        ),
        {'event': 'timeout', 'goal_sequence': 2, 'result_reason': 'goal_timeout'},
    ])
    explorer_final(log_dir / 'failed_high_explorer_state.jsonl', 'FAILED_EXHAUSTED', 1.7)
    (log_dir / 'failed_high_timeout_subtypes.json').write_text(json.dumps({
        'timeout_subtypes': [{
            'goal_sequence': 2,
            'classification': 'healthy_motion_but_late_stall',
            'controller_subtype': 'footprint_path_blocked_late_silent',
            'timing_subtype': 'cmd_silent_after_recovery_abort',
            'combined_subtype': 'footprint_path_blocked_late_silent+cmd_silent_after_recovery_abort',
            'reasons': {
                'footprint_path_blocked': True,
                'path_ahead_1_0m_cost_max': 100,
                'footprint_cost_max': 99,
                'front_wedge_cost_max': 100,
            },
        }]
    }), encoding='utf-8')
    (log_dir / 'failed_high_post_recovery_enriched.json').write_text(json.dumps({
        'enriched_recovery_snapshots': [{
            'goal_sequence': 2,
            'snapshot_density_sufficient': True,
            'pre_recovery_path_ahead_1_0m_cost_max': 0,
            'post_recovery_path_ahead_1_0m_cost_max': 0,
            'near_zero_path_ahead_1_0m_cost_max': 100,
            'near_zero_robot_to_path_distance_m': 0.03,
            'path_update_count_after_recovery': 3,
        }]
    }), encoding='utf-8')

    write_jsonl(log_dir / 'exit_high_goal_events.jsonl', [
        dispatch(1, start_node=9, angle=0.20, target=[2.0, 0.0], target_exit=1.10, robot_exit=1.50),
        {'event': 'success', 'goal_sequence': 1, 'result_reason': 'succeeded'},
        dispatch(
            2,
            start_node=9,
            angle=1.20,
            target=[2.0, 0.8],
            target_exit=0.60,
            robot_exit=1.20,
            candidates=[
                candidate(0.25, [2.4, 0.0], 0.70, dispatch_path_local_cost_max=99, target_local_cost=99),
                candidate(1.20, [2.0, 0.8], 0.60, rejection=None, state='untried', rank=1),
            ],
        ),
        {'event': 'terminal_cancel', 'goal_sequence': 2, 'result_reason': 'exit_reached'},
    ])
    explorer_final(log_dir / 'exit_high_explorer_state.jsonl', 'EXIT_REACHED', 0.4)

    output = tmp_path / 'phase26j.json'
    result = subprocess.run([
        sys.executable,
        str(ANALYZER),
        '--log-dir', str(log_dir),
        '--failed-runs', 'failed_high',
        '--success-runs', 'exit_high',
        '--output-json', str(output),
    ], text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr

    data = json.loads(output.read_text(encoding='utf-8'))
    assert data['phase'] == '26J'
    assert data['analysis_only'] is True
    assert 'do_not_enter_phase27' in data['decision']['guardrails']

    failed_case = data['runs']['failed_high']['joined_forward_shift_cases'][0]
    assert failed_case['later_goal_sequence'] == 2
    assert failed_case['later_outcome']['event'] == 'timeout'
    assert failed_case['timeout_subtype']['controller_subtype'] == 'footprint_path_blocked_late_silent'
    assert failed_case['post_recovery']['near_zero_path_ahead_1_0m_cost_max'] == 100
    assert failed_case['route_context'] == 'route_divergence_chosen_moves_away_rejected_moves_toward_exit'
    assert failed_case['evidence_class'] == 'route_divergence_high_cost_timeout_failure_window'
    assert failed_case['failure_window_flags']['footprint_path_blocked'] is True
    assert failed_case['failure_window_flags']['late_silent'] is True
    assert failed_case['failure_window_flags']['near_zero_path_ahead_high_cost'] is True

    exit_case = data['runs']['exit_high']['joined_forward_shift_cases'][0]
    assert exit_case['later_outcome']['event'] == 'terminal_cancel'
    assert exit_case['route_context'] == 'non_divergent_chosen_also_toward_exit'
    assert exit_case['evidence_class'] == 'harmless_or_non_divergent_high_cost'

    assert data['comparison']['FAILED_EXHAUSTED']['route_divergence_high_cost_timeout_failure_window_count'] == 1
    assert data['comparison']['FAILED_EXHAUSTED']['high_cost_closer_with_footprint_path_blocked_count'] == 1
    assert data['comparison']['FAILED_EXHAUSTED']['high_cost_closer_with_near_zero_path_high_cost_count'] == 1
    assert data['comparison']['EXIT_REACHED']['harmless_or_non_divergent_high_cost_count'] == 1
    assert data['decision']['phase27_candidate_signal'] == 'not_supported'
    assert data['decision']['high_cost_failure_window_overlap_in_failed_runs'] is True


def test_phase26j_keeps_clean_route_divergence_separate_from_high_cost_failure_window(tmp_path):
    log_dir = tmp_path / 'log'
    write_jsonl(log_dir / 'failed_clean_goal_events.jsonl', [
        dispatch(1, start_node=3, angle=0.0, target=[0.0, 0.0], target_exit=1.0, robot_exit=1.4),
        {'event': 'success', 'goal_sequence': 1, 'result_reason': 'succeeded'},
        dispatch(2, start_node=3, angle=-1.0, target=[0.0, -1.0], target_exit=1.5, robot_exit=1.1, candidates=[
            candidate(0.02, [0.6, 0.0], 0.7),
            candidate(-1.0, [0.0, -1.0], 1.5, rejection=None, state='untried', rank=1),
        ]),
        {'event': 'timeout', 'goal_sequence': 2, 'result_reason': 'goal_timeout'},
    ])
    explorer_final(log_dir / 'failed_clean_explorer_state.jsonl', 'FAILED_EXHAUSTED', 2.0)
    output = tmp_path / 'phase26j_clean.json'
    result = subprocess.run([
        sys.executable,
        str(ANALYZER),
        '--log-dir', str(log_dir),
        '--failed-runs', 'failed_clean',
        '--output-json', str(output),
    ], text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr
    data = json.loads(output.read_text(encoding='utf-8'))
    case = data['runs']['failed_clean']['joined_forward_shift_cases'][0]
    assert case['cost_classification'] == 'clean_low_cost'
    assert case['route_context'] == 'route_divergence_chosen_moves_away_rejected_moves_toward_exit'
    assert case['evidence_class'] == 'clean_route_divergence_candidate'
    assert data['comparison']['FAILED_EXHAUSTED']['clean_route_divergence_candidate_count'] == 1
    assert data['decision']['phase27_candidate_signal'] == 'possible_only_after_matched_repeat_validation'
