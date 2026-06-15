#!/usr/bin/env python3
"""Phase59 Post-Ingress Multi-frame Topology Consistency Guard analyzer.

Phase59 verifies that maze_explorer delays only no-candidate/dead-end
terminalization long enough to re-sample topology for a small bounded window.
Guardrails: active scaled2x world only, bounded replay only, max_goals=1,
no Nav2/MPPI/controller parameter edits, no clearance_radius_m tuning,
no map sufficiency threshold tuning, no branch selection strategy change,
no fallback/terminal acceptance, no autonomous exploration success claim,
first dispatch is not exit success, and Phase57 first-dispatch timeout remains
TIMEOUT_INCONCLUSIVE_DATA_GAP.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
from typing import Any

PHASE = 'Phase59 Post-Ingress Multi-frame Topology Consistency Guard'
RUN_ID = 'phase59_post_ingress_topology_consistency_guard'
ACTIVE_WORLD = 'tugbot_maze_world_20260528_clean_scaled2x.sdf'
ACTIVE_METADATA = 'maze_20260528_scaled_instance.yaml'
PHASE58_BASELINE = 'CANDIDATE_FORMATION_UNSTABLE_DEAD_END_POLICY_SENSITIVE'
PHASE57_CONCLUSION = 'TIMEOUT_INCONCLUSIVE_DATA_GAP'
ALLOWED_CLASSIFICATIONS = {
    'TOPOLOGY_CONSISTENCY_GUARD_IMPLEMENTED_STATIC_ONLY',
    'TOPOLOGY_CONSISTENCY_RECOVERED_CANDIDATE_AND_DISPATCH',
    'TOPOLOGY_CONSISTENCY_CONFIRMED_NO_CANDIDATE',
    'TOPOLOGY_CONSISTENCY_INCONCLUSIVE_RUNTIME_VARIANCE',
    'GUARDRAIL_VIOLATION_STRATEGY_CHANGED',
}
GUARDRAILS = [
    'active scaled2x world only',
    'bounded replay only',
    'max_goals=1',
    'no Nav2/MPPI/controller parameter edits',
    'no clearance_radius_m tuning',
    'no map sufficiency threshold tuning',
    'no branch selection strategy change',
    'no fallback/terminal acceptance',
    'no old scaffold world/map',
    'no autonomous exploration success claim',
    'first dispatch is not exit success',
    'does not attribute first-dispatch timeout',
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


def _phase58_module() -> Any:  # pragma: no cover - exercised only in ROS runtime wrapper.
    path = Path(__file__).resolve().parent / 'analyze_phase58_post_ingress_candidate_formation_stability_replay.py'
    spec = importlib.util.spec_from_file_location('phase58_runtime_helpers', path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'cannot load Phase58 runtime helper module: {path}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    setattr(module, 'RUN_ID', RUN_ID)
    setattr(module, 'PHASE', PHASE)
    return module


def send_ingress_goal(args: argparse.Namespace) -> int:  # pragma: no cover - requires ROS graph.
    return int(_phase58_module().send_ingress_goal(args))


def record_stability(args: argparse.Namespace) -> int:  # pragma: no cover - requires ROS graph.
    return int(_phase58_module().record_stability(args))


def _extract_guard(payload: dict[str, Any]) -> dict[str, Any]:
    guard = payload.get('topology_consistency_guard')
    if isinstance(guard, dict):
        return guard
    sampling = payload.get('last_topology_sampling_diagnostics')
    if isinstance(sampling, dict) and isinstance(sampling.get('topology_consistency_guard'), dict):
        return dict(sampling['topology_consistency_guard'])
    phase56 = payload.get('phase56_open_direction_to_candidate_diagnostics')
    if isinstance(phase56, dict) and isinstance(phase56.get('topology_consistency_guard'), dict):
        return dict(phase56['topology_consistency_guard'])
    return {key: payload[key] for key in (
        'topology_consistency_enabled',
        'topology_consistency_required_no_candidate_frames',
        'topology_consistency_window_sec',
        'topology_consistency_frame_index',
        'topology_consistency_frames',
        'topology_consistency_frame_count',
        'raw_open_direction_count',
        'filtered_open_direction_count',
        'candidate_before_filter_count',
        'candidate_after_filter_count',
        'dead_end_policy_state',
        'terminalization_delayed',
        'candidate_recovered_during_consistency_window',
        'topology_consistency_guard_status',
        'topology_consistency_terminalization_reason',
        'confirmed_no_candidate_frames',
    ) if key in payload}


def _collect_guards(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    guards: list[dict[str, Any]] = []
    for row in rows:
        guard = _extract_guard(row)
        if guard:
            guards.append(guard)
    return guards


def _max_int(values: list[Any]) -> int:
    out = 0
    for value in values:
        try:
            if value is not None:
                out = max(out, int(value))
        except (TypeError, ValueError):
            pass
    return out


def summarize_replay(replay_dir: Path, aggregate_dir: Path) -> dict[str, Any]:
    replay_id = replay_dir.name
    action = _safe_json_load(replay_dir / f'{RUN_ID}_{replay_id}_ingress_navigate_to_pose_action_result.json', {})
    states = _read_jsonl(replay_dir / f'{RUN_ID}_{replay_id}_explorer_state.jsonl')
    events = _read_jsonl(replay_dir / f'{RUN_ID}_{replay_id}_goal_events.jsonl')
    all_payloads = states + events
    guards = _collect_guards(all_payloads)
    dispatch_events = [event for event in events if event.get('event') == 'dispatch']
    guard_events = [event for event in events if event.get('event') == 'topology_consistency_guard']
    consistency_frames_by_key: dict[tuple[int, str, int, int, int, int], dict[str, Any]] = {}
    for guard in guards:
        frames = guard.get('topology_consistency_frames')
        if isinstance(frames, list):
            for frame in frames:
                if isinstance(frame, dict):
                    key = (
                        int(frame.get('topology_consistency_frame_index') or 0),
                        str(frame.get('dead_end_policy_state') or ''),
                        int(frame.get('raw_open_direction_count') or 0),
                        int(frame.get('filtered_open_direction_count') or 0),
                        int(frame.get('candidate_before_filter_count') or 0),
                        int(frame.get('candidate_after_filter_count') or 0),
                    )
                    consistency_frames_by_key[key] = frame
    consistency_frames = list(consistency_frames_by_key.values())
    recovered = any(bool(g.get('candidate_recovered_during_consistency_window')) or g.get('topology_consistency_guard_status') == 'candidate_recovered' for g in guards)
    delayed = any(bool(g.get('terminalization_delayed')) for g in guards)
    confirmed_frames = _max_int([g.get('confirmed_no_candidate_frames') for g in guards] + [f.get('topology_consistency_frame_index') for f in consistency_frames if f.get('dead_end_policy_state') == 'confirmed'])
    confirmed = any(g.get('topology_consistency_guard_status') == 'confirmed_no_candidate' for g in guards) or confirmed_frames > 0
    failed_exhausted = any(s.get('mode') == 'FAILED_EXHAUSTED' for s in states)
    return {
        'replay_id': replay_id,
        'artifact_dir': str(replay_dir),
        'ingress_goal_success': bool(action.get('success')),
        'ingress_action': action,
        'state_sample_count': len(states),
        'goal_event_count': len(events),
        'guard_event_count': len(guard_events),
        'consistency_frames': consistency_frames,
        'consistency_frame_count': len(consistency_frames),
        'topology_consistency_frame_index': _max_int([frame.get('topology_consistency_frame_index') for frame in consistency_frames]),
        'raw_open_direction_count': [frame.get('raw_open_direction_count') for frame in consistency_frames],
        'filtered_open_direction_count': [frame.get('filtered_open_direction_count') for frame in consistency_frames],
        'candidate_before_filter_count': [frame.get('candidate_before_filter_count') for frame in consistency_frames],
        'candidate_after_filter_count': [frame.get('candidate_after_filter_count') for frame in consistency_frames],
        'dead_end_policy_state': [frame.get('dead_end_policy_state') for frame in consistency_frames],
        'terminalization_delayed': delayed,
        'candidate_recovered_during_consistency_window': recovered,
        'confirmed_no_candidate_frames': confirmed_frames,
        'confirmed_no_candidate': confirmed,
        'dispatch_observed': bool(dispatch_events),
        'first_dispatch_observed_not_exit_success': bool(dispatch_events),
        'first_dispatch_event': dispatch_events[0] if dispatch_events else None,
        'failed_exhausted_observed': failed_exhausted,
        'final_mode': states[-1].get('mode') if states else None,
    }


def _cleanup_empty(path: Path) -> bool:
    text = _read_text(path)
    lines = [line.strip() for line in text.splitlines() if line.strip()]
    allowed = ('daemon',)
    return all(any(token in line for token in allowed) for line in lines)


def analyze(args: argparse.Namespace) -> int:
    artifact_dir = Path(args.artifact_dir)
    replay_dirs = sorted(p for p in artifact_dir.glob('replay_*') if p.is_dir())
    replays = [summarize_replay(path, artifact_dir) for path in replay_dirs]
    nav2_config_diff = artifact_dir / f'{RUN_ID}_nav2_config_diff.txt'
    cleanup_after = artifact_dir / f'{RUN_ID}_cleanup_processes_after.txt'
    nav2_config_diff_empty = not _read_text(nav2_config_diff).strip()
    cleanup_empty = _cleanup_empty(cleanup_after)
    runtime_samples = sum(int(r['state_sample_count']) + int(r['goal_event_count']) for r in replays)
    dispatch_observed = any(r['dispatch_observed'] for r in replays)
    recovered_and_dispatched = any(r['candidate_recovered_during_consistency_window'] and r['dispatch_observed'] for r in replays)
    confirmed_no_candidate = any(r['confirmed_no_candidate'] for r in replays)
    guardrail_violation = not nav2_config_diff_empty
    if guardrail_violation:
        classification = 'GUARDRAIL_VIOLATION_STRATEGY_CHANGED'
    elif recovered_and_dispatched:
        classification = 'TOPOLOGY_CONSISTENCY_RECOVERED_CANDIDATE_AND_DISPATCH'
    elif confirmed_no_candidate:
        classification = 'TOPOLOGY_CONSISTENCY_CONFIRMED_NO_CANDIDATE'
    elif runtime_samples > 0:
        classification = 'TOPOLOGY_CONSISTENCY_INCONCLUSIVE_RUNTIME_VARIANCE'
    else:
        classification = 'TOPOLOGY_CONSISTENCY_GUARD_IMPLEMENTED_STATIC_ONLY'
    summary = {
        'phase': PHASE,
        'run_id': RUN_ID,
        'classification': classification,
        'allowed_classifications': sorted(ALLOWED_CLASSIFICATIONS),
        'artifact_dir': str(artifact_dir),
        'active_world': ACTIVE_WORLD,
        'active_metadata': ACTIVE_METADATA,
        'phase58_baseline_preserved': PHASE58_BASELINE,
        'phase57_conclusion_preserved': PHASE57_CONCLUSION,
        'replay_count': len(replays),
        'replays': replays,
        'metrics': {
            'consistency_frames': sum(len(r['consistency_frames']) for r in replays),
            'consistency_frame_count': sum(int(r['consistency_frame_count']) for r in replays),
            'confirmed_no_candidate_frames': max([int(r['confirmed_no_candidate_frames']) for r in replays], default=0),
            'dispatch_observed': dispatch_observed,
            'first_dispatch_observed_not_exit_success': dispatch_observed,
            'complete_autonomous_success_claimed': False,
            'candidate_recovered_during_consistency_window': any(r['candidate_recovered_during_consistency_window'] for r in replays),
            'terminalization_delayed': any(r['terminalization_delayed'] for r in replays),
        },
        'guardrail_violation': guardrail_violation,
        'nav2_config_diff_empty': nav2_config_diff_empty,
        'cleanup_empty': cleanup_empty,
        'complete_autonomous_success_claimed': False,
        'first_dispatch_timeout_attributed': False,
        'first_dispatch_is_not_exit_success': True,
        'recommendations': [
            'If classification is recovered-and-dispatch, treat it only as candidate recovery / first dispatch observed, not exit success.',
            'If classification is confirmed-no-candidate, inspect the per-frame raw/filtered/candidate counts before any later policy discussion.',
            'Do not tune Nav2/MPPI/controller, clearance_radius_m, map sufficiency, or branch selection in Phase59.',
        ],
        'guardrails': GUARDRAILS,
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(summary, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=PHASE)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--send-ingress-goal', action='store_true')
    mode.add_argument('--record-stability', action='store_true')
    mode.add_argument('--analyze', action='store_true')
    parser.add_argument('--output', required=True)
    parser.add_argument('--artifact-dir', default=f'log/{RUN_ID}')
    parser.add_argument('--goal-timeout-sec', type=float, default=100.0)
    parser.add_argument('--timeout-sec', type=float, default=120.0)
    parser.add_argument('--periodic-snapshot-sec', type=float, default=1.0)
    args = parser.parse_args(argv)
    if args.send_ingress_goal:
        return send_ingress_goal(args)
    if args.record_stability:
        return record_stability(args)
    return analyze(args)


if __name__ == '__main__':
    raise SystemExit(main())
