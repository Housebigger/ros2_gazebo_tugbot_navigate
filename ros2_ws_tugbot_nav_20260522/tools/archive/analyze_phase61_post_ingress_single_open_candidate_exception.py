#!/usr/bin/env python3
"""Phase61 Post-Ingress Single-Open-Direction Candidate Exception analyzer.

Phase61 verifies a narrowly scoped exception in maze_explorer: after ingress and
only at the first post-ingress topology node, a stable single open direction may
be converted into one ordinary BranchOption before dead-end terminalization.
Guardrails: active scaled2x world only, bounded replay only, max_goals=1,
no Nav2/MPPI/controller parameter edits, no clearance_radius_m tuning,
no map sufficiency threshold tuning, no ordinary branch selection scoring change,
no fallback/terminal acceptance, no autonomous exploration success claim, and
first dispatch is not exit success.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
from typing import Any

PHASE = 'Phase61 Post-Ingress Single-Open-Direction Candidate Exception'
RUN_ID = 'phase61_post_ingress_single_open_candidate_exception'
PHASE59_RUN_ID = 'phase59_post_ingress_topology_consistency_guard'
PHASE60_RUN_ID = 'phase60_single_open_direction_dead_end_policy_review'
ACTIVE_WORLD = 'tugbot_maze_world_20260528_clean_scaled2x.sdf'
ACTIVE_METADATA = 'maze_20260528_scaled_instance.yaml'
PHASE58_BASELINE = 'CANDIDATE_FORMATION_UNSTABLE_DEAD_END_POLICY_SENSITIVE'
PHASE59_CLASSIFICATION = 'TOPOLOGY_CONSISTENCY_CONFIRMED_NO_CANDIDATE'
PHASE60_CLASSIFICATION = 'SINGLE_OPEN_DIRECTION_MISCLASSIFIED_AS_DEAD_END'
PHASE57_CONCLUSION = 'TIMEOUT_INCONCLUSIVE_DATA_GAP'
ALLOWED_CLASSIFICATIONS = {
    'SINGLE_OPEN_EXCEPTION_IMPLEMENTED_STATIC_ONLY',
    'SINGLE_OPEN_EXCEPTION_APPLIED_AND_DISPATCHED',
    'SINGLE_OPEN_EXCEPTION_NOT_ELIGIBLE_CONFIRMED_DEAD_END',
    'SINGLE_OPEN_EXCEPTION_INCONCLUSIVE_RUNTIME_VARIANCE',
    'GUARDRAIL_VIOLATION_STRATEGY_CHANGED',
}
GUARDRAILS = [
    'active scaled2x world only',
    'bounded replay only',
    'max_goals=1',
    'no Nav2/MPPI/controller parameter edits',
    'no clearance_radius_m tuning',
    'no map sufficiency threshold tuning',
    'no ordinary branch selection scoring change',
    'no fallback/terminal acceptance',
    'no old scaffold world/map',
    'no autonomous exploration success claim',
    'first dispatch is not exit success',
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


def _phase59_module() -> Any:  # pragma: no cover - requires ROS runtime.
    path = Path(__file__).resolve().parent / 'analyze_phase59_post_ingress_topology_consistency_guard.py'
    spec = importlib.util.spec_from_file_location('phase59_runtime_helpers', path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'cannot load Phase59 runtime helper module: {path}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    setattr(module, 'RUN_ID', RUN_ID)
    setattr(module, 'PHASE', PHASE)
    return module


def send_ingress_goal(args: argparse.Namespace) -> int:  # pragma: no cover - requires ROS graph.
    return int(_phase59_module().send_ingress_goal(args))


def record_stability(args: argparse.Namespace) -> int:  # pragma: no cover - requires ROS graph.
    return int(_phase59_module().record_stability(args))


def _extract_exception(payload: dict[str, Any]) -> dict[str, Any]:
    direct = payload.get('post_ingress_single_open_exception')
    if isinstance(direct, dict):
        return dict(direct)
    sampling = payload.get('last_topology_sampling_diagnostics')
    if isinstance(sampling, dict) and isinstance(sampling.get('post_ingress_single_open_exception'), dict):
        return dict(sampling['post_ingress_single_open_exception'])
    p56 = payload.get('phase56_open_direction_to_candidate_diagnostics')
    if isinstance(p56, dict) and isinstance(p56.get('post_ingress_single_open_exception'), dict):
        return dict(p56['post_ingress_single_open_exception'])
    keys = (
        'post_ingress_context_active',
        'first_post_ingress_topology_node',
        'single_open_exception_eligible',
        'single_open_exception_applied',
        'single_open_exception_reason',
        'angle_to_return_entrance',
        'multi_frame_single_open_confirmed',
        'single_open_exception_candidate_recovered_during_consistency_window',
        'candidate_map_cell_state',
        'candidate_local_costmap_cell_state',
        'single_open_exception_dispatch_produced',
    )
    return {key: payload[key] for key in keys if key in payload}


def _collect_exceptions(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    exceptions = []
    for row in rows:
        item = _extract_exception(row)
        if item:
            exceptions.append(item)
    return exceptions


def summarize_replay(replay_dir: Path, aggregate_dir: Path) -> dict[str, Any]:
    replay_id = replay_dir.name
    action = _safe_json_load(replay_dir / f'{RUN_ID}_{replay_id}_ingress_navigate_to_pose_action_result.json', {})
    states = _read_jsonl(replay_dir / f'{RUN_ID}_{replay_id}_explorer_state.jsonl')
    events = _read_jsonl(replay_dir / f'{RUN_ID}_{replay_id}_goal_events.jsonl')
    all_payloads = states + events
    exceptions = _collect_exceptions(all_payloads)
    dispatch_events = [event for event in events if event.get('event') == 'dispatch']
    applied = any(bool(e.get('single_open_exception_applied')) for e in exceptions)
    eligible = any(bool(e.get('single_open_exception_eligible')) for e in exceptions)
    dispatch_produced = any(bool(e.get('single_open_exception_dispatch_produced')) for e in exceptions) or (applied and bool(dispatch_events))
    confirmed_dead_end = any(
        e.get('single_open_exception_reason') in {
            'not_first_post_ingress_topology_node',
            'not_multi_frame_confirmed',
            'aligned_with_return_to_entrance',
            'candidate_map_not_clear',
            'candidate_unavailable',
        }
        for e in exceptions
    ) and any(s.get('mode') == 'FAILED_EXHAUSTED' for s in states)
    first_exception = exceptions[0] if exceptions else {}
    last_exception = exceptions[-1] if exceptions else {}
    return {
        'replay_id': replay_id,
        'artifact_dir': str(replay_dir),
        'ingress_goal_success': bool(action.get('success')),
        'state_sample_count': len(states),
        'goal_event_count': len(events),
        'exception_samples': exceptions,
        'exception_sample_count': len(exceptions),
        'post_ingress_context_active': any(bool(e.get('post_ingress_context_active')) for e in exceptions),
        'first_post_ingress_topology_node': any(bool(e.get('first_post_ingress_topology_node')) for e in exceptions),
        'single_open_exception_eligible': eligible,
        'single_open_exception_applied': applied,
        'single_open_exception_reason': last_exception.get('single_open_exception_reason') or first_exception.get('single_open_exception_reason'),
        'angle_to_return_entrance': last_exception.get('angle_to_return_entrance') or first_exception.get('angle_to_return_entrance'),
        'multi_frame_single_open_confirmed': any(bool(e.get('multi_frame_single_open_confirmed')) for e in exceptions),
        'candidate_map_cell_state': last_exception.get('candidate_map_cell_state') or first_exception.get('candidate_map_cell_state'),
        'candidate_local_costmap_cell_state': last_exception.get('candidate_local_costmap_cell_state') or first_exception.get('candidate_local_costmap_cell_state'),
        'single_open_exception_dispatch_produced': dispatch_produced,
        'dispatch_observed': bool(dispatch_events),
        'first_dispatch_observed_not_exit_success': bool(dispatch_events),
        'first_dispatch_event': dispatch_events[0] if dispatch_events else None,
        'confirmed_dead_end_without_dispatch': confirmed_dead_end,
        'final_mode': states[-1].get('mode') if states else None,
    }


def _cleanup_empty(path: Path) -> bool:
    text = _read_text(path)
    lines = [line.strip() for line in text.splitlines() if line.strip()]
    allowed = ('daemon',)
    return all(any(token in line for token in allowed) for line in lines)


def _static_implementation_review(root: Path) -> dict[str, Any]:
    explorer_path = root / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_explorer.py'
    topology_path = root / 'src' / 'tugbot_maze' / 'tugbot_maze' / 'maze_topology.py'
    explorer = _read_text(explorer_path)
    topology = _read_text(topology_path)
    return {
        'parameters_present': "post_ingress_single_open_exception_enabled" in explorer,
        'exception_helper_present': '_maybe_apply_post_ingress_single_open_exception' in explorer,
        'context_scope_present': '_post_ingress_single_open_context_active' in explorer and 'goal_count == 0' in explorer,
        'multi_frame_single_open_confirmed_present': '_single_open_exception_multi_frame_confirmed' in explorer,
        'ordinary_branch_scoring_preserved': 'return max(candidates, key=lambda branch: branch.score_for_exit(node.xy, exit_xy, self.exit_bias_weight))' in topology,
        'diagnostics_present': all(key in explorer for key in [
            'post_ingress_context_active',
            'first_post_ingress_topology_node',
            'single_open_exception_eligible',
            'single_open_exception_applied',
            'angle_to_return_entrance',
            'candidate_map_cell_state',
            'candidate_local_costmap_cell_state',
        ]),
        'evidence_files': [str(explorer_path), str(topology_path)],
    }


def analyze(args: argparse.Namespace) -> int:
    artifact_dir = Path(args.artifact_dir)
    root = Path.cwd()
    replay_dirs = sorted(p for p in artifact_dir.glob('replay_*') if p.is_dir())
    replays = [summarize_replay(path, artifact_dir) for path in replay_dirs]
    nav2_config_diff = artifact_dir / f'{RUN_ID}_nav2_config_diff.txt'
    cleanup_after = artifact_dir / f'{RUN_ID}_cleanup_processes_after.txt'
    nav2_config_diff_empty = not _read_text(nav2_config_diff).strip()
    cleanup_empty = _cleanup_empty(cleanup_after)
    runtime_samples = sum(int(r['state_sample_count']) + int(r['goal_event_count']) for r in replays)
    dispatch_observed = any(r['dispatch_observed'] for r in replays)
    applied_and_dispatched = any(r['single_open_exception_applied'] and r['dispatch_observed'] for r in replays)
    not_eligible_dead_end = any(r['confirmed_dead_end_without_dispatch'] for r in replays)
    guardrail_violation = not nav2_config_diff_empty
    if guardrail_violation:
        classification = 'GUARDRAIL_VIOLATION_STRATEGY_CHANGED'
    elif applied_and_dispatched:
        classification = 'SINGLE_OPEN_EXCEPTION_APPLIED_AND_DISPATCHED'
    elif not_eligible_dead_end:
        classification = 'SINGLE_OPEN_EXCEPTION_NOT_ELIGIBLE_CONFIRMED_DEAD_END'
    elif runtime_samples > 0:
        classification = 'SINGLE_OPEN_EXCEPTION_INCONCLUSIVE_RUNTIME_VARIANCE'
    else:
        classification = 'SINGLE_OPEN_EXCEPTION_IMPLEMENTED_STATIC_ONLY'
    summary = {
        'phase': PHASE,
        'run_id': RUN_ID,
        'classification': classification,
        'allowed_classifications': sorted(ALLOWED_CLASSIFICATIONS),
        'artifact_dir': str(artifact_dir),
        'active_world': ACTIVE_WORLD,
        'active_metadata': ACTIVE_METADATA,
        'phase58_baseline_preserved': PHASE58_BASELINE,
        'phase59_classification_preserved': PHASE59_CLASSIFICATION,
        'phase60_classification_preserved': PHASE60_CLASSIFICATION,
        'phase57_conclusion_preserved': PHASE57_CONCLUSION,
        'static_implementation_review': _static_implementation_review(root),
        'replay_count': len(replays),
        'replays': replays,
        'metrics': {
            'post_ingress_context_active': any(r['post_ingress_context_active'] for r in replays),
            'first_post_ingress_topology_node': any(r['first_post_ingress_topology_node'] for r in replays),
            'single_open_exception_eligible': any(r['single_open_exception_eligible'] for r in replays),
            'single_open_exception_applied': any(r['single_open_exception_applied'] for r in replays),
            'single_open_exception_reason': [r['single_open_exception_reason'] for r in replays],
            'angle_to_return_entrance': [r['angle_to_return_entrance'] for r in replays],
            'multi_frame_single_open_confirmed': any(r['multi_frame_single_open_confirmed'] for r in replays),
            'candidate_map_cell_state': [r['candidate_map_cell_state'] for r in replays],
            'candidate_local_costmap_cell_state': [r['candidate_local_costmap_cell_state'] for r in replays],
            'single_open_exception_dispatch_produced': any(r['single_open_exception_dispatch_produced'] for r in replays),
            'dispatch_observed': dispatch_observed,
            'first_dispatch_observed_not_exit_success': dispatch_observed,
            'complete_autonomous_success_claimed': False,
        },
        'guardrails': GUARDRAILS,
        'guardrail_violation': guardrail_violation,
        'nav2_config_diff_empty': nav2_config_diff_empty,
        'cleanup_empty': cleanup_empty,
        'complete_autonomous_success_claimed': False,
        'first_dispatch_is_not_exit_success': True,
        'recommendations': [
            'If dispatch is observed, report only that the post-ingress single-open exception restored first dispatch; do not claim exit success.',
            'If no dispatch is observed, distinguish confirmed not-eligible dead-end evidence from runtime data insufficiency.',
            'Do not tune Nav2/MPPI/controller, clearance_radius_m, map sufficiency, or ordinary branch selection scoring in Phase61.',
        ],
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(summary, indent=2, sort_keys=True), encoding='utf-8')
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=PHASE)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--send-ingress-goal', action='store_true')
    mode.add_argument('--record-stability', action='store_true')
    mode.add_argument('--analyze', action='store_true')
    parser.add_argument('--artifact-dir', default=str(Path('log') / RUN_ID))
    parser.add_argument('--output', default=str(Path('log') / RUN_ID / f'{RUN_ID}.json'))
    parser.add_argument('--goal-timeout-sec', type=float, default=100.0)
    parser.add_argument('--timeout-sec', type=float, default=120.0)
    parser.add_argument('--periodic-snapshot-sec', type=float, default=1.0)
    args = parser.parse_args()
    if args.send_ingress_goal:
        return send_ingress_goal(args)
    if args.record_stability:
        return record_stability(args)
    return analyze(args)


if __name__ == '__main__':
    raise SystemExit(main())
