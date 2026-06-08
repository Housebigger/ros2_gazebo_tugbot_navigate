#!/usr/bin/env python3
"""Phase71 Post-Goal Success / Re-dispatch Readiness State Machine Diagnosis.

This diagnostics-only analyzer reuses Phase70 artifacts and reconstructs the
state-machine timeline after Goal1 reports success. Required evidence fields:
goal outcome, goal_count, mode transitions, entry readiness, candidate_count,
open_direction_count, local topology kind, map/scan/costmap/TF ages,
blacklist/visited/frontier/exhausted reasons, and whether the re-dispatch gate
was triggered.

Guardrails: no Nav2/MPPI/controller tuning, no inflation/robot_radius/
clearance_radius_m/map threshold tuning, no branch scoring change, no centerline
gate runtime behavior change, no fallback/terminal acceptance change, no
autonomous exploration success claim, no exit success claim, and do not enter
Phase72 from this analyzer.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

PHASE = 'Phase71 Post-Goal Success / Re-dispatch Readiness State Machine Diagnosis'
RUN_ID = 'phase71_post_goal_success_redispatch_readiness_state_machine_diagnosis'
SOURCE_PHASE70_RUN_ID = 'phase70_centerline_gate_relaxation_balance_first_runtime_validation'
PHASE70_CLASSIFICATION = 'BALANCE_FIRST_GATE_NO_APPLY'

ALLOWED_CLASSIFICATIONS = [
    'POST_SUCCESS_REDISTPATCH_BLOCKED_BY_READINESS',
    'POST_SUCCESS_NO_CANDIDATE',
    'POST_SUCCESS_EXHAUSTED_PREMATURELY',
    'SUCCESS_EVENT_SEMANTICS_MISMATCH',
    'INSUFFICIENT_EVIDENCE',
]

GUARDRAILS = [
    'diagnostics-only analyzer over Phase70 artifacts',
    'no Nav2/MPPI/controller tuning',
    'no inflation/robot_radius/clearance_radius_m/map threshold tuning',
    'no branch scoring change',
    'no centerline gate runtime behavior change',
    'no fallback/terminal acceptance change',
    'no autonomous exploration success claim',
    'no exit success claim',
    'do not enter Phase72',
]

OUTCOME_EVENTS = {'success', 'failure', 'timeout', 'timeout_cancel_result'}
TERMINAL_MODES = {'FAILED_EXHAUSTED', 'EXIT_REACHED'}


def _safe_json_load(path: Path | None, default: Any) -> Any:
    if not path or not path.exists() or not path.stat().st_size:
        return default
    try:
        return json.loads(path.read_text(encoding='utf-8', errors='replace'))
    except Exception:
        return default


def _safe_json_loads(text: str) -> Any | None:
    try:
        return json.loads(text)
    except Exception:
        return None


def _read_text(path: Path | None) -> str:
    if not path or not path.exists():
        return ''
    return path.read_text(encoding='utf-8', errors='replace')


def _read_jsonl(path: Path | None) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if not path or not path.exists() or not path.stat().st_size:
        return rows
    for line in path.read_text(encoding='utf-8', errors='replace').splitlines():
        if not line.strip():
            continue
        row = _safe_json_loads(line)
        if not isinstance(row, dict):
            continue
        payload = row.get('state') if isinstance(row.get('state'), dict) else row
        if not isinstance(payload, dict):
            continue
        item = dict(payload)
        item.setdefault('_recorder_elapsed_sec', row.get('elapsed_sec'))
        item.setdefault('_recorder_wall_time', row.get('wall_time'))
        rows.append(item)
    return rows


def _number(value: Any) -> float | None:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return out if math.isfinite(out) else None


def _int_or_none(value: Any) -> int | None:
    number = _number(value)
    if number is None:
        return None
    return int(number)


def _parse_max_goals(preflight_text: str) -> int | None:
    for line in preflight_text.splitlines():
        if line.startswith('MAX_GOALS='):
            raw = line.split('=', 1)[1].strip().strip('"')
            return _int_or_none(raw)
    return None


def _event_elapsed(event: dict[str, Any]) -> float | None:
    return _number(event.get('_recorder_elapsed_sec')) or _number(event.get('elapsed_sec'))


def _state_elapsed(state: dict[str, Any]) -> float | None:
    return _number(state.get('_recorder_elapsed_sec')) or _number(state.get('elapsed_sec'))


def _goal_events_by_sequence(events: list[dict[str, Any]]) -> dict[int, list[dict[str, Any]]]:
    grouped: dict[int, list[dict[str, Any]]] = {}
    for event in events:
        seq = event.get('goal_sequence')
        if seq is None:
            continue
        try:
            grouped.setdefault(int(seq), []).append(event)
        except (TypeError, ValueError):
            continue
    return grouped


def _first_goal_summary(events: list[dict[str, Any]], states: list[dict[str, Any]]) -> dict[str, Any]:
    grouped = _goal_events_by_sequence(events)
    seq = 1 if 1 in grouped else (min(grouped) if grouped else None)
    seq_events = grouped.get(seq, []) if seq is not None else []
    dispatch = next((e for e in seq_events if e.get('event') == 'dispatch'), None)
    outcome = next((e for e in seq_events if e.get('event') in OUTCOME_EVENTS), None)
    success_state_observed = bool(
        (outcome or {}).get('event') == 'success'
        and any(
            (int(s.get('goal_success_count') or 0) >= 1)
            or (s.get('last_completed_goal_sequence_id') == seq and seq is not None)
            for s in states
        )
    )
    return {
        'goal_sequence': seq,
        'dispatch_observed': dispatch is not None,
        'dispatch_elapsed_sec': _event_elapsed(dispatch or {}),
        'outcome_event': outcome.get('event') if outcome else None,
        'outcome_elapsed_sec': _event_elapsed(outcome or {}),
        'result_reason': (outcome or dispatch or {}).get('result_reason'),
        'result_status': (outcome or dispatch or {}).get('result_status'),
        'target': (dispatch or {}).get('target'),
        'success_state_observed': success_state_observed,
    }


def _mode_transitions(states: list[dict[str, Any]]) -> list[dict[str, Any]]:
    transitions: list[dict[str, Any]] = []
    previous_mode: Any = object()
    for index, state in enumerate(states):
        mode = state.get('mode')
        if mode != previous_mode:
            transitions.append({
                'state_index': index,
                'elapsed_sec': _state_elapsed(state),
                'mode': mode,
                'goal_count': state.get('goal_count'),
                'goal_success_count': state.get('goal_success_count'),
                'goal_failure_count': state.get('goal_failure_count'),
                'last_terminal_reason': state.get('last_terminal_reason'),
                'last_failure_reason': state.get('last_failure_reason'),
            })
            previous_mode = mode
    return transitions


def _post_success_states(states: list[dict[str, Any]], goal1: dict[str, Any]) -> list[dict[str, Any]]:
    outcome_elapsed = goal1.get('outcome_elapsed_sec')
    if outcome_elapsed is not None:
        filtered = [s for s in states if (_state_elapsed(s) is not None and _state_elapsed(s) >= outcome_elapsed)]
        if filtered:
            return filtered
    # State recorder cadence can miss the exact event timestamp. Fall back to counters.
    seq = goal1.get('goal_sequence')
    filtered = [
        s for s in states
        if int(s.get('goal_success_count') or 0) >= 1 or (seq is not None and s.get('last_completed_goal_sequence_id') == seq)
    ]
    return filtered


def _latest_post_success_state(states: list[dict[str, Any]], goal1: dict[str, Any]) -> dict[str, Any]:
    post_states = _post_success_states(states, goal1)
    return post_states[-1] if post_states else (states[-1] if states else {})


def _readiness_summary(state: dict[str, Any]) -> dict[str, Any]:
    gate = state.get('dispatch_readiness_gate') if isinstance(state.get('dispatch_readiness_gate'), dict) else {}
    checks = gate.get('checks') if isinstance(gate.get('checks'), dict) else {}
    blocking = state.get('dispatch_readiness_blocking_reasons') or gate.get('blocking_reasons') or []
    passed = state.get('dispatch_readiness_gate_passed')
    if passed is None:
        passed = gate.get('passed')
    if passed is None and checks:
        passed = all(bool(v) for v in checks.values()) and not bool(blocking)
    return {
        'passed': bool(passed) if passed is not None else None,
        'blocking_reasons': list(blocking) if isinstance(blocking, list) else [str(blocking)],
        'checks': checks,
        'first_pass_time_sec': state.get('dispatch_readiness_first_pass_time_sec'),
        'gate_present': bool(gate),
    }


def _resource_ages_summary(state: dict[str, Any]) -> dict[str, Any]:
    gate = state.get('dispatch_readiness_gate') if isinstance(state.get('dispatch_readiness_gate'), dict) else {}

    def section(name: str) -> dict[str, Any]:
        data = gate.get(name) if isinstance(gate.get(name), dict) else {}
        return {
            'sufficient': data.get('sufficient'),
            'sample_age_sec': data.get('sample_age_sec') or data.get('age_sec'),
            'stamp': data.get('map_stamp') or data.get('stamp') or data.get('latest_stamp'),
            'known_ratio': data.get('known_ratio'),
            'free_ratio': data.get('free_ratio'),
            'finite_count': data.get('finite_count'),
            'frame_id': data.get('grid_frame_id') or data.get('frame_id'),
        }

    return {
        'map': section('map'),
        'scan': section('scan'),
        'local_costmap': section('local_costmap'),
        'global_costmap': section('global_costmap'),
        'tf': section('tf'),
    }


def _candidate_summary(state: dict[str, Any]) -> dict[str, Any]:
    diag = state.get('last_topology_sampling_diagnostics') if isinstance(state.get('last_topology_sampling_diagnostics'), dict) else {}
    p56 = state.get('phase56_open_direction_to_candidate_diagnostics') if isinstance(state.get('phase56_open_direction_to_candidate_diagnostics'), dict) else {}
    local_kind = (
        state.get('last_local_topology_kind')
        or diag.get('local_topology_kind')
        or diag.get('kind')
    )
    raw_open = state.get('raw_open_direction_count')
    if raw_open is None:
        raw_open = diag.get('raw_open_direction_count', p56.get('raw_open_direction_count'))
    filtered_open = state.get('filtered_open_direction_count')
    if filtered_open is None:
        filtered_open = diag.get('filtered_open_direction_count', p56.get('filtered_open_direction_count'))
    before = state.get('candidate_before_filter_count')
    if before is None:
        before = diag.get('candidate_before_filter_count', p56.get('candidate_before_filter_count'))
    after = state.get('candidate_after_filter_count')
    if after is None:
        after = diag.get('candidate_after_filter_count', p56.get('candidate_after_filter_count'))
    if after is None:
        after = state.get('last_candidate_count')
    candidate_count = diag.get('candidate_branch_count')
    if candidate_count is None:
        candidate_count = state.get('last_candidate_count')
    return {
        'local_topology_kind': local_kind,
        'raw_open_direction_count': _int_or_none(raw_open),
        'filtered_open_direction_count': _int_or_none(filtered_open),
        'candidate_before_filter_count': _int_or_none(before),
        'candidate_after_filter_count': _int_or_none(after),
        'candidate_count': _int_or_none(candidate_count),
        'rejection_reason': p56.get('branch_candidate_rejection_reason') or diag.get('branch_candidate_rejection_reason'),
        'candidate_clearance_result': p56.get('candidate_clearance_result'),
        'candidate_map_cell_state': p56.get('candidate_map_cell_state'),
        'candidate_local_costmap_cell_state': p56.get('candidate_local_costmap_cell_state'),
        'diagnostics_available': bool(diag or p56),
    }


def _state_reasons_summary(state: dict[str, Any]) -> dict[str, Any]:
    topology_guard = state.get('topology_consistency_guard') if isinstance(state.get('topology_consistency_guard'), dict) else {}
    post_ingress_exception = state.get('post_ingress_single_open_exception') if isinstance(state.get('post_ingress_single_open_exception'), dict) else {}
    return {
        'blacklisted_goal_count': state.get('blacklisted_goal_count'),
        'blocked_branch_count': state.get('blocked_branch_count'),
        'last_terminal_reason': state.get('last_terminal_reason'),
        'last_failure_reason': state.get('last_failure_reason'),
        'startup_warmup_guard_reason': state.get('startup_warmup_guard_reason'),
        'topology_consistency_guard_status': state.get('topology_consistency_guard_status') or topology_guard.get('topology_consistency_guard_status'),
        'topology_consistency_terminalization_reason': state.get('topology_consistency_terminalization_reason') or topology_guard.get('topology_consistency_terminalization_reason'),
        'dead_end_policy_state': topology_guard.get('dead_end_policy_state'),
        'single_open_exception_reason': state.get('single_open_exception_reason') or post_ingress_exception.get('single_open_exception_reason'),
        'visited_or_frontier_evidence_available': any(k in state for k in ('visited_nodes', 'frontier_count', 'known_junctions', 'edges')),
    }


def _dispatch_after_success_count(events: list[dict[str, Any]], goal1: dict[str, Any]) -> int:
    outcome_elapsed = goal1.get('outcome_elapsed_sec')
    goal_seq = goal1.get('goal_sequence')
    count = 0
    for event in events:
        if event.get('event') != 'dispatch':
            continue
        seq = _int_or_none(event.get('goal_sequence'))
        if goal_seq is not None and seq is not None and seq > int(goal_seq):
            count += 1
            continue
        elapsed = _event_elapsed(event)
        if outcome_elapsed is not None and elapsed is not None and elapsed > float(outcome_elapsed):
            count += 1
    return count


def _post_success_blocking_stage(
    goal1: dict[str, Any],
    final_state: dict[str, Any],
    readiness: dict[str, Any],
    candidate: dict[str, Any],
    dispatch_after_success_count: int,
    max_goals: int | None,
) -> tuple[str, str]:
    if goal1.get('outcome_event') != 'success':
        return 'INSUFFICIENT_EVIDENCE', 'goal1_not_success'
    if not goal1.get('success_state_observed'):
        return 'SUCCESS_EVENT_SEMANTICS_MISMATCH', 'success_event_state_counter_mismatch'
    if dispatch_after_success_count > 0:
        return 'INSUFFICIENT_EVIDENCE', 'redispatch_observed_outside_phase71_blocker_scope'

    final_goal_count = _int_or_none(final_state.get('goal_count'))
    terminal_reason = str(final_state.get('last_terminal_reason') or '').lower()
    if (
        final_state.get('mode') == 'FAILED_EXHAUSTED'
        and max_goals is not None
        and final_goal_count is not None
        and final_goal_count >= max_goals
    ) or 'goal budget reached' in terminal_reason:
        return 'POST_SUCCESS_EXHAUSTED_PREMATURELY', 'goal_budget_exhausted_before_readiness_or_topology_resample'

    if readiness.get('passed') is False or readiness.get('blocking_reasons'):
        return 'POST_SUCCESS_REDISTPATCH_BLOCKED_BY_READINESS', 'dispatch_entry_readiness'

    candidate_after = candidate.get('candidate_after_filter_count')
    open_count = candidate.get('raw_open_direction_count') or candidate.get('filtered_open_direction_count')
    if candidate_after == 0 or (candidate_after is None and open_count == 0):
        return 'POST_SUCCESS_NO_CANDIDATE', 'post_success_candidate_formation'

    if final_state.get('mode') == 'FAILED_EXHAUSTED':
        return 'POST_SUCCESS_EXHAUSTED_PREMATURELY', 'exhausted_without_dispatch_after_success'

    return 'INSUFFICIENT_EVIDENCE', 'post_success_blocker_not_identified'


def summarize_replay(replay_dir: Path, max_goals: int | None, source_run_id: str = SOURCE_PHASE70_RUN_ID) -> dict[str, Any]:
    replay_id = replay_dir.name
    events = _read_jsonl(replay_dir / f'{source_run_id}_{replay_id}_goal_events.jsonl')
    states = _read_jsonl(replay_dir / f'{source_run_id}_{replay_id}_explorer_state.jsonl')
    ingress_action = _safe_json_load(replay_dir / f'{source_run_id}_{replay_id}_inner_ingress_navigate_to_pose_action_result.json', {})
    goal1 = _first_goal_summary(events, states)
    post_success_states = _post_success_states(states, goal1)
    post_state = _latest_post_success_state(states, goal1)
    final_state = states[-1] if states else {}
    readiness = _readiness_summary(post_state)
    resource_ages = _resource_ages_summary(post_state)
    candidate = _candidate_summary(post_state)
    dispatch_after_success_count = _dispatch_after_success_count(events, goal1)
    classification, blocking_stage = _post_success_blocking_stage(
        goal1, final_state, readiness, candidate, dispatch_after_success_count, max_goals
    )
    final_goal_count = _int_or_none(final_state.get('goal_count'))
    redispatch_gate_triggered = bool(
        goal1.get('outcome_event') == 'success'
        and goal1.get('success_state_observed')
        and not (max_goals is not None and final_goal_count is not None and final_goal_count >= max_goals)
        and final_state.get('last_terminal_reason') != 'goal budget reached'
    )
    return {
        'replay_id': replay_id,
        'artifact_dir': str(replay_dir),
        'source_phase70_run_id': source_run_id,
        'inner_ingress_goal_success': bool(ingress_action.get('success')),
        'state_sample_count': len(states),
        'goal_event_count': len(events),
        'goal1': goal1,
        'mode_transitions': _mode_transitions(states),
        'readiness_after_success': readiness,
        'resource_ages_after_success': resource_ages,
        'candidate_after_success': candidate,
        'blacklist_visited_frontier_exhausted_reasons': _state_reasons_summary(post_state),
        'post_success': {
            'post_success_state_sample_count': len(post_success_states),
            'final_mode': final_state.get('mode'),
            'final_goal_count': final_state.get('goal_count'),
            'max_goals': max_goals,
            'last_terminal_reason': final_state.get('last_terminal_reason'),
            'last_failure_reason': final_state.get('last_failure_reason'),
            'dispatch_after_success_count': dispatch_after_success_count,
            'redispatch_gate_triggered': redispatch_gate_triggered,
            'blocking_stage': blocking_stage,
            'classification': classification,
        },
        'classification': classification,
        'complete_autonomous_success_claimed': False,
        'exit_success_claimed': False,
    }


def classify_phase71(replays: list[dict[str, Any]]) -> str:
    if not replays:
        return 'INSUFFICIENT_EVIDENCE'
    priority = [
        'SUCCESS_EVENT_SEMANTICS_MISMATCH',
        'POST_SUCCESS_EXHAUSTED_PREMATURELY',
        'POST_SUCCESS_REDISTPATCH_BLOCKED_BY_READINESS',
        'POST_SUCCESS_NO_CANDIDATE',
    ]
    observed = {r.get('classification') for r in replays}
    for item in priority:
        if item in observed:
            return item
    return 'INSUFFICIENT_EVIDENCE'


def analyze_phase71(artifact_dir: Path | str, output: Path | str | None = None) -> dict[str, Any]:
    artifact_path = Path(artifact_dir)
    preflight_text = _read_text(artifact_path / f'{SOURCE_PHASE70_RUN_ID}_preflight.txt')
    max_goals = _parse_max_goals(preflight_text)
    replay_dirs = sorted(p for p in artifact_path.glob('replay_*') if p.is_dir())
    replays = [summarize_replay(path, max_goals=max_goals) for path in replay_dirs]
    classification = classify_phase71(replays)
    nav2_config_diff_empty = not _read_text(artifact_path / f'{SOURCE_PHASE70_RUN_ID}_nav2_config_diff.txt').strip()
    cleanup_empty = not _read_text(artifact_path / f'{SOURCE_PHASE70_RUN_ID}_cleanup_processes_after.txt').strip()
    summary = {
        'phase': PHASE,
        'run_id': RUN_ID,
        'source_phase70_run_id': SOURCE_PHASE70_RUN_ID,
        'source_phase70_artifact_dir': str(artifact_path),
        'source_phase70_classification_preserved': PHASE70_CLASSIFICATION,
        'classification': classification,
        'allowed_classifications': ALLOWED_CLASSIFICATIONS,
        'max_goals': max_goals,
        'replay_count': len(replays),
        'replays': replays,
        'metrics': {
            'goal1_success_replay_count': sum(1 for r in replays if r.get('goal1', {}).get('outcome_event') == 'success'),
            'dispatch_after_success_total': sum(int(r.get('post_success', {}).get('dispatch_after_success_count') or 0) for r in replays),
            'post_success_goal_budget_exhausted_count': sum(
                1 for r in replays
                if r.get('post_success', {}).get('blocking_stage') == 'goal_budget_exhausted_before_readiness_or_topology_resample'
            ),
        },
        'nav2_config_diff_empty': nav2_config_diff_empty,
        'cleanup_empty': cleanup_empty,
        'guardrails': GUARDRAILS,
        'complete_autonomous_success_claimed': False,
        'exit_success_claimed': False,
        'recommendations': [
            'Phase71 is diagnostics-only and stops here; do not enter Phase72 without human acceptance.',
            'For Phase70 replay_01, Goal1 success is real in goal_events and state counters, but max_goals=1 causes goal budget exhaustion before any post-success readiness/topology re-dispatch loop can run.',
        ],
    }
    if output is not None:
        out_path = Path(output)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(summary, indent=2, sort_keys=True), encoding='utf-8')
    return summary


def analyze(args: argparse.Namespace) -> int:
    summary = analyze_phase71(Path(args.artifact_dir), output=Path(args.output))
    print(json.dumps({'output': str(args.output), 'classification': summary['classification'], 'replay_count': summary['replay_count']}, sort_keys=True))
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=PHASE)
    parser.add_argument('--artifact-dir', default=str(Path('log') / SOURCE_PHASE70_RUN_ID))
    parser.add_argument('--output', required=True)
    args = parser.parse_args(argv)
    return analyze(args)


if __name__ == '__main__':
    raise SystemExit(main())
