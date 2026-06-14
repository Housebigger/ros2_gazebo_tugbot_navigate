import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
ANALYZER = ROOT / 'tools' / 'analyze_phase26f_branch_choice_diagnostics.py'


def write_jsonl(path: Path, rows: list[dict]):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('\n'.join(json.dumps({'state': row}) for row in rows) + '\n', encoding='utf-8')


def dispatch(seq: int, robot_exit: float, target_exit: float, chosen_rank: int, candidates: list[dict], **extra) -> dict:
    row = {
        'event': 'dispatch',
        'goal_sequence': seq,
        'goal_kind': 'explore',
        'chosen_branch_rank': chosen_rank,
        'candidate_branch_count': len(candidates),
        'candidate_branches': candidates,
        'robot_exit_dist_at_dispatch': robot_exit,
        'target_exit_dist': target_exit,
        'target': [float(seq), 0.0],
        'branch_angle': 0.0,
        'near_exit': target_exit <= 1.0,
        'selected_due_to_context': 'topology_exit_bias_score',
        'dispatch_path_local_cost_max': 0,
        'dispatch_target_local_cost': 0,
        'target_clearance_m': 0.8,
    }
    row.update(extra)
    return row


def candidate(angle: float, target_exit: float, delta: float, rejection: str | None, **extra) -> dict:
    row = {
        'branch_angle': angle,
        'target': [angle, target_exit],
        'target_exit_dist': target_exit,
        'exit_progress_delta_m': delta,
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
    }
    row.update(extra)
    return row


def test_phase26f_analyzer_flags_route_divergence_when_chosen_moves_away_but_rejected_moves_toward_exit(tmp_path):
    log_dir = tmp_path / 'log'
    write_jsonl(log_dir / 'baseline_a_goal_events.jsonl', [
        dispatch(
            1,
            robot_exit=1.0,
            target_exit=1.4,
            chosen_rank=1,
            candidates=[
                candidate(0.0, 1.4, -0.4, None),
                candidate(1.57, 0.7, 0.3, 'lower_rank_not_selected'),
            ],
        ),
        {'event': 'success', 'goal_sequence': 1, 'result_reason': 'succeeded'},
        dispatch(
            2,
            robot_exit=1.4,
            target_exit=0.6,
            chosen_rank=1,
            candidates=[candidate(0.0, 0.6, 0.8, None)],
        ),
        {'event': 'terminal_cancel', 'goal_sequence': 2, 'result_reason': 'exit_reached'},
    ])
    write_jsonl(log_dir / 'candidate_a_goal_events.jsonl', [
        dispatch(
            1,
            robot_exit=1.2,
            target_exit=0.8,
            chosen_rank=1,
            candidates=[candidate(0.0, 0.8, 0.4, None), candidate(1.57, 1.5, -0.3, 'lower_rank_not_selected')],
            near_exit=True,
        ),
        {'event': 'success', 'goal_sequence': 1, 'result_reason': 'succeeded'},
    ])
    output = tmp_path / 'phase26f.json'
    result = subprocess.run([
        sys.executable,
        str(ANALYZER),
        '--log-dir', str(log_dir),
        '--baseline-runs', 'baseline_a',
        '--candidate-runs', 'candidate_a',
        '--output-json', str(output),
    ], text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr
    data = json.loads(output.read_text(encoding='utf-8'))
    assert data['phase'] == '26F'
    assert data['analysis_only'] is True
    assert data['matched_groups']['baseline']['run_ids'] == ['baseline_a']
    assert data['matched_groups']['candidate']['run_ids'] == ['candidate_a']

    baseline = data['runs']['baseline_a']
    assert baseline['summary']['dispatch_count'] == 2
    assert baseline['summary']['route_divergence_case_count'] == 1
    case = baseline['route_divergence_cases'][0]
    assert case['goal_sequence'] == 1
    assert case['chosen_exit_progress_delta_m'] < 0
    assert case['best_rejected_exit_progress_delta_m'] > 0
    assert case['classification'] == 'chosen_moves_away_rejected_moves_toward_exit'
    assert 'near_exit' in case['contexts']
    assert 'local_cost_clear' in case['contexts']

    assert data['matched_comparison']['baseline']['route_divergence_case_count'] == 1
    assert data['matched_comparison']['candidate']['route_divergence_case_count'] == 0
    assert data['decision']['recommendation'] == 'analysis_only_collect_more_or_repeat'
    assert 'do_not_enter_phase27' in data['decision']['guardrails']


def test_phase26f_analyzer_separates_reverse_backtrack_near_exit_cost_and_blacklist_contexts(tmp_path):
    log_dir = tmp_path / 'log'
    write_jsonl(log_dir / 'baseline_contexts_goal_events.jsonl', [
        dispatch(
            3,
            robot_exit=2.0,
            target_exit=2.3,
            chosen_rank=2,
            goal_kind='backtrack',
            candidates=[
                candidate(3.14, 2.3, -0.3, None, is_reverse_candidate=True, is_backtrack_context=True, dispatch_path_local_cost_max=95),
                candidate(0.0, 1.5, 0.5, 'blacklisted', is_near_exit_candidate=False),
            ],
            dispatch_path_local_cost_max=95,
            dispatch_target_local_cost=90,
            near_exit=False,
        ),
    ])
    output = tmp_path / 'phase26f_contexts.json'
    result = subprocess.run([
        sys.executable,
        str(ANALYZER),
        '--log-dir', str(log_dir),
        '--baseline-runs', 'baseline_contexts',
        '--output-json', str(output),
    ], text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr
    data = json.loads(output.read_text(encoding='utf-8'))
    row = data['runs']['baseline_contexts']['dispatch_summaries'][0]
    assert row['chosen_branch_rank'] == 2
    assert row['candidate_branch_count'] == 2
    assert row['context_flags']['reverse'] is True
    assert row['context_flags']['backtrack'] is True
    assert row['context_flags']['near_exit'] is False
    assert row['context_flags']['local_cost_constrained'] is True
    assert row['context_flags']['blacklisted_or_rejected'] is True
    assert row['best_rejected_exit_progress_delta_m'] == 0.5
    assert row['best_rejected_rejection_reason'] == 'blacklisted'
