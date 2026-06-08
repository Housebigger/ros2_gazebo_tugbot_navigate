#!/usr/bin/env python3
"""Phase26U controller/MPPI log signal parser.

Diagnostics-only. Joins Phase26R/S/T timelines and artifacts, then searches
controller launch logs around dispatch/recovery/first-cmd-near-zero/timeout
windows for controller/MPPI textual hints.
"""

from __future__ import annotations

import argparse
import json
import re
from collections import Counter
from pathlib import Path
from typing import Any

PATTERNS: dict[str, re.Pattern[str]] = {
    'no_valid_control': re.compile(r'no valid (control|trajectory|trajectories)|failed to find valid|invalid trajectory', re.I),
    'invalid_trajectory': re.compile(r'invalid trajectory|trajectory invalid|invalid control', re.I),
    'collision': re.compile(r'\bcollision\b|colliding|collision check', re.I),
    'near_collision': re.compile(r'near[- ]?collision|near collision', re.I),
    'constraint': re.compile(r'constraint', re.I),
    'progress_failure': re.compile(r'Failed to make progress', re.I),
    'controller_abort': re.compile(r'Aborting handle|controller abort|ActionServer\] Aborting', re.I),
    'zero_velocity': re.compile(r'zero velocity|cmd_vel.*zero|velocity command selected', re.I),
}
CRITICS = ['CostCritic', 'PathAlignCritic', 'PathFollowCritic', 'GoalCritic', 'GoalAngleCritic', 'PathAngleCritic', 'ConstraintCritic']


def load_json(path: Path) -> dict[str, Any]:
    if not path.exists() or path.stat().st_size == 0:
        return {}
    return json.loads(path.read_text(encoding='utf-8'))


def iter_jsonl(path: Path):
    if not path.exists():
        return
    with path.open(encoding='utf-8') as f:
        for line in f:
            if line.strip():
                try:
                    yield json.loads(line)
                except json.JSONDecodeError:
                    continue


def log_time(line: str) -> float | None:
    # ROS launch lines usually contain wall timestamps like [1779765833.739092794].
    # Unit tests may use shorter synthetic timestamps like [100.500000000].
    matches = re.findall(r'\[(\d+(?:\.\d+)?)\]', line)
    if matches:
        try:
            return float(matches[-1])
        except ValueError:
            return None
    return None


def load_log_lines(path: Path) -> list[dict[str, Any]]:
    rows = []
    if not path.exists():
        return rows
    with path.open(encoding='utf-8', errors='replace') as f:
        for idx, line in enumerate(f, start=1):
            t = log_time(line)
            rows.append({'line_no': idx, 'time': t, 'text': line.rstrip('\n')})
    return rows


def timeline_cases(log_dir: Path, run_id: str) -> list[dict[str, Any]]:
    data = load_json(log_dir / f'{run_id}_phase26p_single_goal_timeline.json')
    cases = data.get('cases') or []
    if cases:
        return cases
    return []


def goal_event_time(log_dir: Path, run_id: str, goal_sequence: int, event_names: set[str]) -> float | None:
    for row in iter_jsonl(log_dir / f'{run_id}_goal_events.jsonl') or []:
        if int(row.get('goal_sequence') or -1) == int(goal_sequence) and str(row.get('event')) in event_names:
            value = row.get('wall_time') or row.get('time')
            try:
                return float(value)
            except (TypeError, ValueError):
                pass
    return None


def count_signals(lines: list[dict[str, Any]], start: float, end: float) -> tuple[dict[str, int], dict[str, int], list[dict[str, Any]]]:
    counts = Counter()
    critic_counts = Counter()
    matches = []
    for row in lines:
        t = row.get('time')
        if t is None or t < start or t > end:
            continue
        text = row['text']
        matched_names = []
        for name, pattern in PATTERNS.items():
            if pattern.search(text):
                counts[name] += 1
                matched_names.append(name)
        for critic in CRITICS:
            if critic in text:
                critic_counts[critic] += 1
        if matched_names or any(critic in text for critic in CRITICS):
            matches.append({'time': t, 'line_no': row['line_no'], 'signals': matched_names, 'text': text[:500]})
    return dict(counts), dict(critic_counts), matches


def classify(signals: dict[str, int], critic_counts: dict[str, int], mppi_condition: str | None) -> str:
    if signals.get('no_valid_control', 0) or signals.get('invalid_trajectory', 0):
        return 'controller_no_valid_control_or_invalid_trajectory_log_signal'
    if signals.get('near_collision', 0) and (critic_counts.get('CostCritic', 0) or critic_counts.get('ConstraintCritic', 0)):
        return 'near_collision_or_constraint_log_signal'
    if signals.get('collision', 0) and critic_counts:
        return 'collision_or_critic_log_signal'
    if signals.get('progress_failure', 0) or signals.get('controller_abort', 0):
        return 'progress_failure_controller_abort_only_no_specific_mppi_reason'
    return mppi_condition or 'no_specific_controller_log_signal'


def analyze_run(log_dir: Path, run_id: str) -> list[dict[str, Any]]:
    lines = load_log_lines(log_dir / f'{run_id}_launch.log')
    mppi = load_json(log_dir / f'{run_id}_phase26p_mppi_evidence_analysis.json')
    mppi_by_seq = {int(c.get('goal_sequence')): c for c in mppi.get('cases', []) if c.get('goal_sequence') is not None}
    cases = []
    for case in timeline_cases(log_dir, run_id):
        seq = int(case.get('goal_sequence'))
        rel = case.get('cmd_near_zero_relation') or {}
        first_cmd = rel.get('first_cmd_near_zero_time')
        recovery = rel.get('recovery_time')
        progress = rel.get('progress_failure_time')
        abort = rel.get('controller_abort_time')
        dispatch = goal_event_time(log_dir, run_id, seq, {'dispatch'})
        timeout = goal_event_time(log_dir, run_id, seq, {'timeout', 'timeout_cancel_result', 'goal_timeout'})
        anchors = [v for v in [dispatch, recovery, first_cmd, timeout, progress, abort] if isinstance(v, (int, float))]
        if not anchors:
            continue
        start = min(anchors) - 1.0
        end = max(anchors) + 1.0
        signals, critic_counts, matches = count_signals(lines, start, end)
        mppi_case = mppi_by_seq.get(seq, {})
        mppi_condition = mppi_case.get('condition_hypothesis')
        all_signal_keys = set(PATTERNS)
        signal_out = {f'{name}_count': signals.get(name, 0) for name in sorted(all_signal_keys)}
        signal_out['critic_keyword_counts'] = {critic: critic_counts.get(critic, 0) for critic in CRITICS}
        cases.append({
            'run_id': run_id,
            'goal_sequence': seq,
            'window': {
                'start_time': start,
                'end_time': end,
                'dispatch_time': dispatch,
                'recovery_time': recovery,
                'first_cmd_near_zero_time': first_cmd,
                'timeout_time': timeout,
                'progress_failure_time': progress,
                'controller_abort_time': abort,
            },
            'signals': signal_out,
            'matched_log_lines': matches[:30],
            'mppi_condition_hypothesis': mppi_condition,
            'condition_hypothesis': classify(signals, critic_counts, mppi_condition),
        })
    return cases


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--log-dir', type=Path, default=Path('log'))
    p.add_argument('--run-ids', nargs='+', required=True)
    p.add_argument('--output-json', type=Path, required=True)
    return p.parse_args()


def main() -> int:
    args = parse_args()
    cases = [case for run_id in args.run_ids for case in analyze_run(args.log_dir, run_id)]
    condition_counts = Counter(c['condition_hypothesis'] for c in cases)
    specific = any(c in {'controller_no_valid_control_or_invalid_trajectory_log_signal', 'near_collision_or_constraint_log_signal', 'collision_or_critic_log_signal'} for c in condition_counts)
    report = {
        'phase': '26U',
        'analysis_only': True,
        'source_run_ids': args.run_ids,
        'cases': cases,
        'summary': {
            'case_count': len(cases),
            'condition_hypothesis_counts': dict(condition_counts),
            'specific_log_signal_case_count': sum(1 for c in cases if c['condition_hypothesis'] in {'controller_no_valid_control_or_invalid_trajectory_log_signal', 'near_collision_or_constraint_log_signal', 'collision_or_critic_log_signal'}),
        },
        'decision': {
            'phase27_candidate_signal': 'review_only' if specific else 'not_supported',
            'intervention_allowed': False,
            'guardrails': ['diagnostics_only', 'do_not_change_branch_selection', 'do_not_tune_nav2_controller_params', 'do_not_promote_or_reject_candidate'],
        },
    }
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(report, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    print(json.dumps({'summary': report['summary'], 'decision': report['decision']}, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
