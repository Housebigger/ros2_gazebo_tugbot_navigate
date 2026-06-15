#!/usr/bin/env python3
"""Phase26O controller_server / MPPI evidence analyzer.

Analysis-only: joins Phase26N timeout windows with existing launch logs and
post-recovery artifacts. It determines what can and cannot be concluded about
controller_server activity, fresh path delivery, MPPI setup, critic/debug-log
availability, and local-cost-vs-cmd timing. It does not run ROS/Gazebo/Nav2 and
does not change branch selection or Nav2/controller parameters.
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import Any

import yaml

LOG_TIME_RE = re.compile(r'\[(\d+(?:\.\d+)?)\]')
CRITIC_RE = re.compile(r'mppi::critics::([A-Za-z0-9_]+)')

PER_CYCLE_CRITIC_PATTERNS = (
    'critic score', 'critic scores', 'cost score', 'trajectory cost',
    'selected trajectory', 'best trajectory', 'critics data',
)
TRAJECTORY_VALIDITY_PATTERNS = (
    'trajectory valid', 'trajectory invalid', 'invalid trajectory',
    'valid trajectories', 'no valid trajectories', 'collision trajectory',
)
ZERO_VELOCITY_REASON_PATTERNS = (
    'zero velocity', 'cmd_vel zero', 'velocity command is zero',
    'near-zero', 'near zero', 'output zero', 'control effort zero',
)
EXCEPTION_PATTERNS = ('exception', 'Exception', 'Failed to compute control', 'No valid control')


def number(value: Any) -> float | None:
    if value is None or isinstance(value, bool):
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def load_json(path: Path) -> dict[str, Any]:
    if not path.exists() or path.stat().st_size == 0:
        return {}
    return json.loads(path.read_text(encoding='utf-8'))


def load_mppi_params(log_dir: Path, run_id: str) -> dict[str, Any]:
    fingerprint = load_json(log_dir / f'{run_id}_params_fingerprint.json')
    params_path_raw = (fingerprint.get('params_file') or {}).get('path')
    if not params_path_raw:
        return {}
    params_path = Path(params_path_raw)
    if not params_path.exists():
        return {'path': str(params_path), 'exists': False}
    data = yaml.safe_load(params_path.read_text(encoding='utf-8')) or {}
    follow_path = (((data.get('controller_server') or {}).get('ros__parameters') or {}).get('FollowPath') or {})
    return {
        'path': str(params_path),
        'exists': True,
        'visualize': follow_path.get('visualize'),
        'publish_critics_stats': follow_path.get('publish_critics_stats'),
        'publish_optimal_trajectory': follow_path.get('publish_optimal_trajectory'),
        'critic_index_to_visualize': follow_path.get('critic_index_to_visualize'),
        'trajectory_visualizer': follow_path.get('TrajectoryVisualizer'),
    }


def parse_log_time(line: str) -> float | None:
    matches = LOG_TIME_RE.findall(line)
    if not matches:
        return None
    return float(matches[0])


def load_log_events(path: Path) -> list[dict[str, Any]]:
    events = []
    if not path.exists():
        return events
    for lineno, line in enumerate(path.read_text(encoding='utf-8', errors='replace').splitlines(), start=1):
        timestamp = parse_log_time(line)
        if timestamp is None:
            continue
        events.append({'time': timestamp, 'line_number': lineno, 'line': line})
    return events


def events_between(events: list[dict[str, Any]], start: float | None, end: float | None) -> list[dict[str, Any]]:
    if start is None:
        return []
    upper = end if end is not None else start + 10.0
    return [event for event in events if start <= event['time'] <= upper]


def first_event_after(events: list[dict[str, Any]], start: float | None, pattern: str, end: float | None = None) -> dict[str, Any] | None:
    if start is None:
        return None
    upper = end if end is not None else float('inf')
    for event in events:
        if event['time'] >= start and event['time'] <= upper and pattern in event['line']:
            return event
    return None


def count_events_after(events: list[dict[str, Any]], start: float | None, pattern: str, end: float | None = None) -> tuple[int, list[float]]:
    if start is None:
        return 0, []
    upper = end if end is not None else float('inf')
    times = [event['time'] for event in events if start <= event['time'] <= upper and pattern in event['line']]
    return len(times), times


def loaded_critics(events: list[dict[str, Any]]) -> list[str]:
    critics = []
    for event in events:
        if 'Critic loaded' not in event['line']:
            continue
        match = CRITIC_RE.search(event['line'])
        if match:
            name = match.group(1)
            if name not in critics:
                critics.append(name)
    return critics


def has_any(events: list[dict[str, Any]], patterns: tuple[str, ...]) -> bool:
    lowered = [(event['line'] or '').lower() for event in events]
    return any(pattern.lower() in line for line in lowered for pattern in patterns)


def matching_lines(events: list[dict[str, Any]], patterns: tuple[str, ...], limit: int = 5) -> list[dict[str, Any]]:
    matches = []
    for event in events:
        line_lower = event['line'].lower()
        if any(pattern.lower() in line_lower for pattern in patterns):
            matches.append({'time': event['time'], 'line_number': event['line_number'], 'line': event['line']})
            if len(matches) >= limit:
                break
    return matches


def analyze_case(log_dir: Path, case: dict[str, Any]) -> dict[str, Any]:
    run_id = case['run_id']
    seq = int(case['goal_sequence'])
    launch_log = log_dir / f'{run_id}_launch.log'
    events = load_log_events(launch_log)

    relation = case.get('cmd_near_zero_relation', {})
    recovery = number(relation.get('recovery_time'))
    first_cmd = number(relation.get('first_cmd_near_zero_time'))
    window_end = first_cmd + 8.0 if first_cmd is not None else (recovery + 10.0 if recovery is not None else None)
    window_events = events_between(events, recovery, window_end)

    configured = any('Created controller : FollowPath of type nav2_mppi_controller::MPPIController' in e['line'] for e in events)
    activated = any('Activated MPPI Controller: FollowPath' in e['line'] for e in events)
    received_goal = first_event_after(events, recovery, 'Received a goal, begin computing control effort.', window_end)
    path_count, path_times = count_events_after(events, recovery, 'Passing new path to controller.', window_end)
    control_loop_miss_count, control_loop_miss_times = count_events_after(events, recovery, 'Control loop missed its desired rate', window_end)
    exception_lines = matching_lines(window_events, EXCEPTION_PATTERNS)
    critics = loaded_critics(events)
    mppi_params = load_mppi_params(log_dir, run_id)

    per_cycle_critic = has_any(window_events, PER_CYCLE_CRITIC_PATTERNS)
    traj_validity = has_any(window_events, TRAJECTORY_VALIDITY_PATTERNS)
    zero_reason = has_any(window_events, ZERO_VELOCITY_REASON_PATTERNS)

    local_cost = case.get('local_cost_windows', {})
    high_delta = number(local_cost.get('first_high_cost_window_after_recovery_sec'))
    cmd_delta = number(relation.get('first_cmd_near_zero_after_recovery_sec'))
    high_before_cmd = bool(high_delta is not None and cmd_delta is not None and high_delta <= cmd_delta)

    instrumentation_gaps = []
    if not per_cycle_critic:
        instrumentation_gaps.append('mppi_critic_scores_or_selected_trajectory_costs')
    if not traj_validity:
        instrumentation_gaps.append('trajectory_validity_or_rejection_reason')
    if not zero_reason:
        instrumentation_gaps.append('zero_velocity_reason_or_controller_cycle_outcome')
    if not exception_lines:
        instrumentation_gaps.append('controller_exception_or_invalid_control_logs_absent')

    if configured and activated and received_goal is not None and path_count > 0 and not per_cycle_critic and not traj_validity and not zero_reason:
        inference = 'controller_active_and_fresh_paths_but_no_per_cycle_mppi_critic_evidence'
    elif not configured or not activated:
        inference = 'insufficient_controller_activation_evidence'
    elif path_count == 0:
        inference = 'missing_fresh_path_updates_in_launch_log_window'
    else:
        inference = 'mixed_controller_mppi_evidence'

    return {
        'run_id': run_id,
        'goal_sequence': seq,
        'source_launch_log': str(launch_log),
        'window': {
            'recovery_time': recovery,
            'first_cmd_near_zero_time': first_cmd,
            'window_end_time': window_end,
            'window_event_count': len(window_events),
        },
        'controller_state': {
            'mppi_controller_configured': configured,
            'mppi_controller_activated': activated,
            'received_goal_after_recovery': received_goal is not None,
            'received_goal_after_recovery_sec': round(received_goal['time'] - recovery, 6) if received_goal is not None and recovery is not None else None,
            'control_loop_miss_count_after_recovery': control_loop_miss_count,
            'control_loop_miss_times_after_recovery_sec': [round(t - recovery, 6) for t in control_loop_miss_times] if recovery is not None else [],
            'exception_or_invalid_control_lines': exception_lines,
        },
        'plan_freshness': {
            'path_updates_after_recovery_count': path_count,
            'path_update_times_after_recovery_sec': [round(t - recovery, 6) for t in path_times] if recovery is not None else [],
            'first_path_update_after_recovery_sec': round(path_times[0] - recovery, 6) if path_times and recovery is not None else None,
            'phase26n_path_update_count': case.get('path_update_cadence', {}).get('path_update_count_after_recovery'),
        },
        'mppi_critic_evidence': {
            'loaded_critics': critics,
            'params': mppi_params,
            'per_cycle_critic_scores_present': per_cycle_critic,
            'trajectory_validity_logs_present': traj_validity,
            'zero_velocity_reason_logs_present': zero_reason,
            'per_cycle_critic_matching_lines': matching_lines(window_events, PER_CYCLE_CRITIC_PATTERNS),
            'trajectory_validity_matching_lines': matching_lines(window_events, TRAJECTORY_VALIDITY_PATTERNS),
            'zero_velocity_reason_matching_lines': matching_lines(window_events, ZERO_VELOCITY_REASON_PATTERNS),
        },
        'local_cost_vs_cmd_timing': {
            'high_cost_window_count_after_recovery': local_cost.get('high_cost_window_count_after_recovery'),
            'first_high_cost_window_after_recovery_sec': high_delta,
            'first_cmd_near_zero_after_recovery_sec': cmd_delta,
            'high_cost_before_first_cmd_near_zero': high_before_cmd,
        },
        'inference': inference,
        'instrumentation_gaps': instrumentation_gaps,
    }


def build_report(log_dir: Path, timeline_json: Path) -> dict[str, Any]:
    timeline = load_json(timeline_json)
    cases = [analyze_case(log_dir, case) for case in timeline.get('cases', [])]
    inference_counts: dict[str, int] = {}
    for case in cases:
        inference_counts[case['inference']] = inference_counts.get(case['inference'], 0) + 1
    high_before = sum(1 for case in cases if case['local_cost_vs_cmd_timing']['high_cost_before_first_cmd_near_zero'])
    return {
        'phase': '26O',
        'analysis_only': True,
        'source_timeline_json': str(timeline_json),
        'cases': cases,
        'summary': {
            'case_count': len(cases),
            'inference_counts': inference_counts,
            'controller_active_and_fresh_paths_count': sum(
                1 for case in cases
                if case['controller_state']['mppi_controller_configured']
                and case['controller_state']['mppi_controller_activated']
                and case['controller_state']['received_goal_after_recovery']
                and case['plan_freshness']['path_updates_after_recovery_count'] > 0
            ),
            'per_cycle_critic_scores_present_count': sum(1 for case in cases if case['mppi_critic_evidence']['per_cycle_critic_scores_present']),
            'trajectory_validity_logs_present_count': sum(1 for case in cases if case['mppi_critic_evidence']['trajectory_validity_logs_present']),
            'zero_velocity_reason_logs_present_count': sum(1 for case in cases if case['mppi_critic_evidence']['zero_velocity_reason_logs_present']),
            'high_cost_before_first_cmd_near_zero_count': high_before,
        },
        'decision': {
            'phase27_candidate_signal': 'not_supported',
            'parameter_tuning_signal': 'not_supported',
            'next_recommendation': 'add_or_enable_mppi_controller_debug_evidence_before_intervention',
            'guardrails': [
                'do_not_change_branch_selection',
                'do_not_tune_controller_from_phase26n_or_phase26o_log_availability_alone',
                'collect_critic_or_controller_cycle_reason_before_intervention',
            ],
        },
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log-dir', type=Path, required=True)
    parser.add_argument('--timeline-json', type=Path, required=True)
    parser.add_argument('--output-json', type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    report = build_report(args.log_dir, args.timeline_json)
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(report, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    print(json.dumps({'summary': report['summary'], 'decision': report['decision']}, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
