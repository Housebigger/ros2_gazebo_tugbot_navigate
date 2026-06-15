#!/usr/bin/env python3
"""Phase74 Directional Local Costmap Readiness Gate / Bounded Runtime Validation.

Goal: validate the minimal direction-aware readiness override introduced after
Phase73 found the full-window local costmap gate too strict for post-success
redispatch in a narrow corridor.  The analyzer preserves both records:

- local_costmap_full_window: the original full-window local costmap gate verdict.
- local_costmap_directional_override: the optional Phase74 directional override.

Guardrails: bounded runtime only; no Nav2/MPPI/controller tuning; no
inflation/robot_radius/clearance_radius_m/map threshold tuning; no branch scoring
change; no centerline gate runtime behavior change; no fallback/terminal
acceptance change; no autonomous exploration success claim; no exit success
claim.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
from typing import Any

PHASE = 'Phase74 Directional Local Costmap Readiness Gate / Bounded Runtime Validation'
RUN_ID = 'phase74_directional_local_costmap_readiness_gate_validation'
PHASE73_CLASSIFICATION = 'LOCAL_DIRECTION_TRAVERSABLE_GATE_SHOULD_REPLACE_GLOBAL_RATIO'
PHASE72_CLASSIFICATION = 'MULTIGOAL_NO_REDISPATCH_AFTER_SUCCESS'
INNER_INGRESS_WAYPOINT_MAP = {'x_m': 2.0, 'y_m': 0.0, 'yaw_rad': 0.0}
ALLOWED_CLASSIFICATIONS = [
    'DIRECTIONAL_GATE_ENABLES_GOAL2_DISPATCH',
    'DIRECTIONAL_GATE_APPLIED_TIMEOUT_REMAINS',
    'DIRECTIONAL_GATE_NO_APPLY',
    'DIRECTIONAL_GATE_REGRESSION',
    'INSUFFICIENT_EVIDENCE',
]
GUARDRAILS = [
    'bounded runtime only',
    'replay=2',
    'max_goals=3',
    'no Nav2/MPPI/controller tuning',
    'no inflation/robot_radius/clearance_radius_m/map threshold tuning',
    'no branch scoring change',
    'no centerline gate runtime behavior change',
    'no fallback/terminal acceptance change',
    'no autonomous exploration success claim',
    'no exit success claim',
]


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
        if isinstance(row, dict):
            payload = row.get('state') if isinstance(row.get('state'), dict) else row
            if isinstance(payload, dict):
                payload = dict(payload)
                payload.setdefault('_recorder_elapsed_sec', row.get('elapsed_sec'))
                payload.setdefault('_recorder_wall_time', row.get('wall_time'))
                rows.append(payload)
    return rows


def _number(value: Any) -> float | None:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _phase72_module():  # pragma: no cover - ROS graph helpers delegated at runtime.
    path = Path(__file__).resolve().parent / 'analyze_phase72_multigoal_bounded_rerun_from_inner_ingress.py'
    spec = importlib.util.spec_from_file_location('phase72_multigoal_runtime_helper_for_phase74', path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'cannot load Phase72 helper module: {path}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    # The delegated send/record helpers write whatever explicit output paths the
    # Phase74 wrapper passes.  Override phase identity for JSON metadata.
    setattr(module, 'RUN_ID', RUN_ID)
    setattr(module, 'PHASE', PHASE)
    return module


def send_inner_ingress_goal(args: argparse.Namespace) -> int:  # pragma: no cover
    return int(_phase72_module().send_inner_ingress_goal(args))


def record_runtime(args: argparse.Namespace) -> int:  # pragma: no cover
    return int(_phase72_module().record_runtime(args))


def _goal_events_by_sequence(events: list[dict[str, Any]]) -> dict[int, list[dict[str, Any]]]:
    grouped: dict[int, list[dict[str, Any]]] = {}
    for event in events:
        seq = event.get('goal_sequence')
        if seq is None:
            continue
        try:
            key = int(seq)
        except (TypeError, ValueError):
            continue
        grouped.setdefault(key, []).append(event)
    return grouped


def _mode_transitions(states: list[dict[str, Any]]) -> list[dict[str, Any]]:
    transitions: list[dict[str, Any]] = []
    previous = object()
    for state in states:
        mode = state.get('mode')
        if mode != previous:
            transitions.append({
                'elapsed_sec': state.get('_recorder_elapsed_sec'),
                'mode': mode,
                'goal_count': state.get('goal_count'),
                'active_goal_sequence_id': state.get('active_goal_sequence_id'),
            })
            previous = mode
    return transitions


def _gate_from_row(row: dict[str, Any]) -> dict[str, Any] | None:
    gate = row.get('dispatch_readiness_gate')
    return gate if isinstance(gate, dict) else None


def _full_window_from_gate(gate: dict[str, Any] | None) -> dict[str, Any] | None:
    if not isinstance(gate, dict):
        return None
    full = gate.get('local_costmap_full_window')
    if isinstance(full, dict):
        return full
    local_costmap = gate.get('local_costmap')
    if isinstance(local_costmap, dict):
        nested = local_costmap.get('local_costmap_full_window')
        if isinstance(nested, dict):
            return nested
        return local_costmap
    return None


def _override_from_row(row: dict[str, Any]) -> dict[str, Any] | None:
    direct = row.get('local_costmap_directional_override')
    if isinstance(direct, dict):
        return direct
    gate = _gate_from_row(row)
    if isinstance(gate, dict):
        gate_direct = gate.get('local_costmap_directional_override')
        if isinstance(gate_direct, dict):
            return gate_direct
        local_costmap = gate.get('local_costmap')
        if isinstance(local_costmap, dict):
            nested = local_costmap.get('directional_override')
            if isinstance(nested, dict):
                return nested
    return None


def _directional_override_summary(rows: list[dict[str, Any]]) -> dict[str, Any]:
    override_rows = [payload for payload in (_override_from_row(row) for row in rows) if isinstance(payload, dict)]
    full_windows = [payload for payload in (_full_window_from_gate(_gate_from_row(row)) for row in rows) if isinstance(payload, dict)]
    applied = [payload for payload in override_rows if bool(payload.get('override_applied'))]
    considered = [payload for payload in override_rows if bool(payload.get('override_considered'))]
    selected = next((payload.get('selected_candidate_direction') for payload in applied if isinstance(payload.get('selected_candidate_direction'), dict)), None)
    candidate_directions: list[dict[str, Any]] = []
    for payload in override_rows:
        raw = payload.get('candidate_directions')
        if isinstance(raw, list):
            candidate_directions.extend([row for row in raw if isinstance(row, dict)])
    return {
        'local_costmap_full_window': full_windows[-1] if full_windows else None,
        'local_costmap_directional_override': applied[-1] if applied else (override_rows[-1] if override_rows else None),
        'override_observation_count': len(override_rows),
        'override_considered_count': len(considered),
        'override_applied_count': len(applied),
        'override_applied_observed': bool(applied),
        'full_window_gate_failed_observed': any(bool(row.get('full_window_gate_failed')) for row in override_rows),
        'selected_candidate_direction': selected,
        'candidate_direction_count': len(candidate_directions),
        'direction_traversable_count_max': max([int(row.get('direction_traversable_count') or 0) for row in override_rows] or [0]),
        'non_reverse_direction_traversable_count_max': max([int(row.get('non_reverse_direction_traversable_count') or 0) for row in override_rows] or [0]),
        'path_cost': selected.get('path_cost') if isinstance(selected, dict) else None,
        'min_clearance_m': selected.get('min_clearance_m') if isinstance(selected, dict) else None,
        'target_risk': selected.get('target_risk') if isinstance(selected, dict) else None,
        'footprint_risk': selected.get('footprint_risk') if isinstance(selected, dict) else None,
        'front_wedge_risk': selected.get('front_wedge_risk') if isinstance(selected, dict) else None,
    }


def _readiness_summary(rows: list[dict[str, Any]]) -> dict[str, Any]:
    gates = [gate for gate in (_gate_from_row(row) for row in rows) if isinstance(gate, dict)]
    blocking: dict[str, int] = {}
    for gate in gates:
        for reason in gate.get('blocking_reasons') or []:
            blocking[str(reason)] = blocking.get(str(reason), 0) + 1
    return {
        'gate_sample_count': len(gates),
        'passed_count': sum(1 for gate in gates if bool(gate.get('passed'))),
        'blocking_reason_counts': blocking,
        'latest_gate': gates[-1] if gates else None,
    }


def _per_goal_summaries(events: list[dict[str, Any]], states: list[dict[str, Any]]) -> list[dict[str, Any]]:
    grouped = _goal_events_by_sequence(events)
    per_goal: list[dict[str, Any]] = []
    for seq in sorted(grouped):
        seq_events = grouped[seq]
        dispatch = next((e for e in seq_events if e.get('event') == 'dispatch'), None)
        outcome = next((e for e in seq_events if e.get('event') in {'success', 'failure', 'timeout'}), None)
        cancel = next((e for e in seq_events if e.get('event') == 'timeout_cancel_result'), None)
        seq_states = [s for s in states if s.get('active_goal_sequence_id') == seq or s.get('goal_sequence_id') == seq or s.get('last_completed_goal_sequence_id') == seq]
        combined = seq_states + seq_events
        per_goal.append({
            'goal_sequence': seq,
            'dispatch_observed': dispatch is not None,
            'outcome_event': outcome.get('event') if outcome else None,
            'timeout_cancel_result_observed': cancel is not None,
            'timeout': bool((outcome or {}).get('event') == 'timeout' or cancel is not None),
            'result_status': (outcome or cancel or dispatch or {}).get('result_status'),
            'result_reason': (outcome or cancel or dispatch or {}).get('result_reason'),
            'target': (dispatch or {}).get('target'),
            'dispatch_pose': (dispatch or {}).get('dispatch_pose'),
            'candidate_open_direction_summary': {
                'local_topology_kind': (dispatch or {}).get('last_local_topology_kind'),
                'raw_open_direction_count': (dispatch or {}).get('raw_open_direction_count'),
                'filtered_open_direction_count': (dispatch or {}).get('filtered_open_direction_count'),
                'candidate_after_filter_count': (dispatch or {}).get('candidate_after_filter_count'),
                'candidate_branch_count': (dispatch or {}).get('candidate_branch_count'),
            },
            'readiness_summary': _readiness_summary(combined),
            'directional_override_summary': _directional_override_summary(combined),
            'centerline_refinement_applied': bool((dispatch or {}).get('centerline_refinement_applied')),
            'centerline_refinement_reason': (dispatch or {}).get('centerline_refinement_reason'),
            'branch_scoring_changed': bool((dispatch or {}).get('branch_scoring_changed', False)),
        })
    return per_goal


def summarize_replay(replay_dir: Path, run_id: str = RUN_ID) -> dict[str, Any]:
    replay_id = replay_dir.name
    ingress_action = _safe_json_load(replay_dir / f'{run_id}_{replay_id}_inner_ingress_navigate_to_pose_action_result.json', {})
    events = _read_jsonl(replay_dir / f'{run_id}_{replay_id}_goal_events.jsonl')
    states = _read_jsonl(replay_dir / f'{run_id}_{replay_id}_explorer_state.jsonl')
    # Runtime rows are retained for coverage counts; decisive Phase74 state is in explorer state/goal events.
    timeline = _read_jsonl(replay_dir / 'phase74_runtime_timeline.jsonl')
    local_rows = _read_jsonl(replay_dir / 'phase74_local_costmap_samples.jsonl')
    per_goal = _per_goal_summaries(events, states)
    goal1_success_time = next((e.get('_recorder_elapsed_sec') for e in events if int(e.get('goal_sequence') or -1) == 1 and e.get('event') == 'success'), None)
    dispatch_count = sum(1 for e in events if e.get('event') == 'dispatch')
    outcome_count = sum(1 for e in events if e.get('event') in {'success', 'failure', 'timeout', 'timeout_cancel_result'})
    timeout_count = sum(1 for e in events if e.get('event') in {'timeout', 'timeout_cancel_result'})
    goal1_success_observed = any(g.get('goal_sequence') == 1 and g.get('outcome_event') == 'success' for g in per_goal)
    dispatch_after_goal1_success_count = 0
    if goal1_success_observed:
        dispatch_after_goal1_success_count = sum(1 for e in events if e.get('event') == 'dispatch' and int(e.get('goal_sequence') or 0) > 1)
    final_state = states[-1] if states else {}
    combined_rows = states + events
    directional_summary = _directional_override_summary(combined_rows)
    goal2_dispatch_observed = any(g.get('goal_sequence') == 2 and g.get('dispatch_observed') for g in per_goal)
    exit_reached = any(e.get('mode') == 'EXIT_REACHED' for e in states + events)
    return {
        'replay_id': replay_id,
        'artifact_dir': str(replay_dir),
        'inner_ingress_goal_success': bool(ingress_action.get('success')),
        'inner_ingress_action_result': ingress_action,
        'state_sample_count': len(states),
        'goal_event_count': len(events),
        'runtime_timeline_sample_count': len(timeline),
        'local_costmap_sample_count': len(local_rows),
        'mode_transitions': _mode_transitions(states),
        'dispatch_count': dispatch_count,
        'outcome_count': outcome_count,
        'timeout_count': timeout_count,
        'goal1_success_observed': goal1_success_observed,
        'goal1_success_elapsed_sec': goal1_success_time,
        'dispatch_after_goal1_success_count': dispatch_after_goal1_success_count,
        'goal2_dispatch_observed': goal2_dispatch_observed,
        'readiness_summary': _readiness_summary(combined_rows),
        'directional_override_summary': directional_summary,
        'local_costmap_full_window': directional_summary.get('local_costmap_full_window'),
        'local_costmap_directional_override': directional_summary.get('local_costmap_directional_override'),
        'per_goal_summaries': per_goal,
        'final_mode': final_state.get('mode'),
        'final_goal_count': final_state.get('goal_count'),
        'last_failure_reason': final_state.get('last_failure_reason'),
        'last_terminal_reason': final_state.get('last_terminal_reason'),
        'blocked_branch_count': final_state.get('blocked_branch_count'),
        'blacklisted_goal_count': final_state.get('blacklisted_goal_count'),
        'exit_reached_by_existing_state': exit_reached,
        'complete_autonomous_success_claimed': False,
        'exit_success_claimed': False,
    }


def classify_phase74(replays: list[dict[str, Any]], guardrail_violation: bool = False) -> str:
    if guardrail_violation:
        return 'DIRECTIONAL_GATE_REGRESSION'
    if not replays or not any(r.get('inner_ingress_goal_success') for r in replays):
        return 'INSUFFICIENT_EVIDENCE'
    successful_goal1 = [r for r in replays if r.get('inner_ingress_goal_success') and r.get('goal1_success_observed')]
    if not successful_goal1:
        return 'INSUFFICIENT_EVIDENCE'
    override_applied = any((r.get('directional_override_summary') or {}).get('override_applied_observed') for r in successful_goal1)
    goal2_dispatch = any(r.get('goal2_dispatch_observed') for r in successful_goal1)
    timeout_after_override = any(override_applied and (r.get('timeout_count') or 0) > 0 for r in successful_goal1)
    if override_applied and goal2_dispatch:
        return 'DIRECTIONAL_GATE_ENABLES_GOAL2_DISPATCH'
    if timeout_after_override:
        return 'DIRECTIONAL_GATE_APPLIED_TIMEOUT_REMAINS'
    if not override_applied and not goal2_dispatch:
        return 'DIRECTIONAL_GATE_NO_APPLY'
    if any((r.get('blocked_branch_count') or 0) > 0 or (r.get('blacklisted_goal_count') or 0) > 0 for r in successful_goal1):
        return 'DIRECTIONAL_GATE_REGRESSION'
    return 'INSUFFICIENT_EVIDENCE'


def analyze_phase74(artifact_dir: Path | str, output: Path | str | None = None) -> dict[str, Any]:
    artifact_path = Path(artifact_dir)
    replay_dirs = sorted(p for p in artifact_path.glob('replay_*') if p.is_dir())
    replays = [summarize_replay(path, RUN_ID) for path in replay_dirs]
    preflight_text = _read_text(artifact_path / f'{RUN_ID}_preflight.txt')
    nav2_config_diff_empty = not _read_text(artifact_path / f'{RUN_ID}_nav2_config_diff.txt').strip()
    cleanup_empty = not _read_text(artifact_path / f'{RUN_ID}_cleanup_processes_after.txt').strip()
    max_goals = _number(next((line.split('=', 1)[1] for line in preflight_text.splitlines() if line.startswith('MAX_GOALS=')), None))
    replay_count_declared = _number(next((line.split('=', 1)[1] for line in preflight_text.splitlines() if line.startswith('REPLAY_COUNT=')), None))
    max_goals_ok = max_goals is not None and int(max_goals) == 3
    replay_count_ok = replay_count_declared is not None and int(replay_count_declared) == 2
    override_enabled = 'directional_local_costmap_readiness_override_enabled=true' in preflight_text
    forbidden_strategy_change = any(token in preflight_text for token in [
        'near_exit_fallback_enabled=true',
        'branch_scoring_changed=true',
        'centerline gate runtime behavior change=true',
    ])
    branch_scoring_changed = any(any(g.get('branch_scoring_changed') for g in r.get('per_goal_summaries', [])) for r in replays)
    centerline_runtime_changed = 'no centerline gate runtime behavior change' not in preflight_text
    guardrail_violation = bool(
        not nav2_config_diff_empty
        or forbidden_strategy_change
        or branch_scoring_changed
        or centerline_runtime_changed
        or not max_goals_ok
        or not replay_count_ok
        or not override_enabled
    )
    classification = classify_phase74(replays, guardrail_violation=guardrail_violation)
    summary = {
        'phase': PHASE,
        'run_id': RUN_ID,
        'classification': classification,
        'allowed_classifications': ALLOWED_CLASSIFICATIONS,
        'phase73_classification_preserved': PHASE73_CLASSIFICATION,
        'phase72_classification_preserved': PHASE72_CLASSIFICATION,
        'inner_ingress_waypoint_map': INNER_INGRESS_WAYPOINT_MAP,
        'artifact_dir': str(artifact_path),
        'guardrails': GUARDRAILS,
        'guardrail_checks': {
            'nav2_config_diff_empty': nav2_config_diff_empty,
            'cleanup_processes_after_empty': cleanup_empty,
            'max_goals_exactly_3': max_goals_ok,
            'replay_count_exactly_2': replay_count_ok,
            'directional_local_costmap_readiness_override_enabled': override_enabled,
            'branch_scoring_changed': branch_scoring_changed,
            'centerline_gate_runtime_behavior_changed': centerline_runtime_changed,
            'guardrail_violation': guardrail_violation,
        },
        'replay_count': len(replays),
        'metrics': {
            'goal1_success_replay_count': sum(1 for r in replays if r.get('goal1_success_observed')),
            'directional_override_applied_replay_count': sum(1 for r in replays if (r.get('directional_override_summary') or {}).get('override_applied_observed')),
            'redispatch_after_goal1_success_observed': any((r.get('dispatch_after_goal1_success_count') or 0) > 0 for r in replays),
            'goal2_dispatch_replay_count': sum(1 for r in replays if r.get('goal2_dispatch_observed')),
            'dispatch_count_total': sum(int(r.get('dispatch_count') or 0) for r in replays),
            'outcome_count_total': sum(int(r.get('outcome_count') or 0) for r in replays),
            'timeout_count_total': sum(int(r.get('timeout_count') or 0) for r in replays),
            'blocked_branch_count_total': sum(int(r.get('blocked_branch_count') or 0) for r in replays),
            'blacklisted_goal_count_total': sum(int(r.get('blacklisted_goal_count') or 0) for r in replays),
        },
        'replays': replays,
        'complete_autonomous_success_claimed': False,
        'exit_success_claimed': False,
    }
    if output is not None:
        out = Path(output)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(summary, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    return summary


def analyze(args: argparse.Namespace) -> int:
    summary = analyze_phase74(Path(args.artifact_dir), output=Path(args.output))
    print(json.dumps({'output': str(args.output), 'classification': summary['classification'], 'replay_count': summary['replay_count']}, sort_keys=True))
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=PHASE)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--send-inner-ingress-goal', action='store_true')
    mode.add_argument('--record-runtime', action='store_true')
    mode.add_argument('--analyze', action='store_true')
    parser.add_argument('--artifact-dir', default=str(Path('log') / RUN_ID))
    parser.add_argument('--output', required=True)
    parser.add_argument('--goal-timeout-sec', type=float, default=100.0)
    parser.add_argument('--timeout-sec', type=float, default=240.0)
    parser.add_argument('--periodic-snapshot-sec', type=float, default=1.0)
    parser.add_argument('--timeline-output', default=str(Path('log') / RUN_ID / 'phase74_runtime_timeline.jsonl'))
    parser.add_argument('--controller-dynamics-output', default=str(Path('log') / RUN_ID / 'phase74_controller_dynamics.jsonl'))
    parser.add_argument('--nav2-feedback-output', default=str(Path('log') / RUN_ID / 'phase74_nav2_feedback.jsonl'))
    parser.add_argument('--local-costmap-samples-output', default=str(Path('log') / RUN_ID / 'phase74_local_costmap_samples.jsonl'))
    parser.add_argument('--global-plan-samples-output', default=str(Path('log') / RUN_ID / 'phase74_global_plan_samples.jsonl'))
    parser.add_argument('--collision-monitor-output', default=str(Path('log') / RUN_ID / 'phase74_collision_monitor_state.jsonl'))
    parser.add_argument('--local-costmap-topic', default='/local_costmap/costmap')
    parser.add_argument('--path-topic', default='/plan')
    parser.add_argument('--odom-topic', default='/odom')
    parser.add_argument('--cmd-topic', default='/cmd_vel')
    parser.add_argument('--cmd-smoothed-topic', default='/cmd_vel_smoothed')
    parser.add_argument('--goal-events-topic', default='/maze/goal_events')
    parser.add_argument('--collision-monitor-topic', default='/collision_monitor_state')
    parser.add_argument('--nav2-feedback-topic', default='/navigate_to_pose/_action/feedback')
    args = parser.parse_args(argv)
    if args.send_inner_ingress_goal:
        return send_inner_ingress_goal(args)
    if args.record_runtime:
        return record_runtime(args)
    return analyze(args)


if __name__ == '__main__':
    raise SystemExit(main())
