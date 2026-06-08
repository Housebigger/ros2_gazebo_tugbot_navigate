#!/usr/bin/env python3
"""Analyze Phase42 map-frame truth bounded autonomous smoke artifacts.

This analyzer is intentionally post-run/read-only. It does not tune Nav2, modify
maze_explorer strategy, continue fallback/terminal acceptance, or claim complete
autonomous exploration success unless an exit-reached event is actually present.
"""
from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import Any

PHASE42_ALLOWED_CLASSIFICATIONS = [
    'BOUNDED_SMOKE_DISPATCHED_GOAL',
    'BOUNDED_SMOKE_REACHED_EXIT_WITH_EVIDENCE',
    'BOUNDED_SMOKE_NO_DISPATCH_MAP_INSUFFICIENCY',
    'BOUNDED_SMOKE_NO_DISPATCH_TOPOLOGY_SAMPLING',
    'BOUNDED_SMOKE_NO_DISPATCH_COSTMAP_BLOCKED',
    'BOUNDED_SMOKE_NO_DISPATCH_TF_ALIGNMENT',
    'BOUNDED_SMOKE_NO_DISPATCH_OTHER',
    'BOUNDED_SMOKE_INSUFFICIENT_EVIDENCE',
]

LEGACY_TRUTH_TOKENS = [
    'maze_instance.yaml',
    'tugbot_maze_world.sdf',
    'entrance_x:=-4.0',
    'entrance_y:=-3.0',
    'exit_x:=4.0',
    'exit_y:=3.0',
    'exit_radius:=0.6',
]


def read_text(path: Path) -> str:
    try:
        return path.read_text(encoding='utf-8', errors='replace')
    except FileNotFoundError:
        return ''


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if not path.exists() or path.stat().st_size == 0:
        return rows
    for line in path.read_text(encoding='utf-8', errors='replace').splitlines():
        if not line.strip():
            continue
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError as exc:
            rows.append({'_parse_error': str(exc), 'raw': line})
    return rows


def payload(row: dict[str, Any]) -> dict[str, Any]:
    state = row.get('state', row)
    return state if isinstance(state, dict) else {'raw_state': state}


def file_nonempty(path: Path) -> bool:
    return path.exists() and path.stat().st_size > 0


def classify_no_dispatch(final_state: dict[str, Any], launch_log: str, artifact_dir: Path, evidence: dict[str, Any]) -> tuple[str, list[str]]:
    reasons: list[str] = []
    mode = str(final_state.get('mode', ''))
    terminal_reason = str(final_state.get('last_terminal_reason', '') or '')
    topo_kind = str(final_state.get('last_local_topology_kind', '') or '')
    open_count = final_state.get('last_open_direction_count')
    candidate_count = final_state.get('last_candidate_count')
    blocked_counts = final_state.get('last_blocked_reason_counts') or final_state.get('blocked_reason_counts') or {}

    map_text = read_text(artifact_dir / f"{evidence['run_id']}_map_once.txt")
    scan_text = read_text(artifact_dir / f"{evidence['run_id']}_scan_once.txt")
    local_costmap_text = read_text(artifact_dir / f"{evidence['run_id']}_local_costmap_once.txt")
    tf_text = read_text(artifact_dir / f"{evidence['run_id']}_tf_once.txt")

    if not map_text.strip() or 'data:' not in map_text or 'width: 0' in map_text:
        reasons.append('map sample missing/empty/truncated before dispatch')
        return 'BOUNDED_SMOKE_NO_DISPATCH_MAP_INSUFFICIENCY', reasons
    if not scan_text.strip() or 'ranges:' not in scan_text:
        reasons.append('scan sample missing before dispatch')
        return 'BOUNDED_SMOKE_NO_DISPATCH_MAP_INSUFFICIENCY', reasons
    if not tf_text.strip() or 'base_link' not in tf_text:
        reasons.append('TF sample missing base_link evidence')
        return 'BOUNDED_SMOKE_NO_DISPATCH_TF_ALIGNMENT', reasons
    if blocked_counts and any(key in str(blocked_counts) for key in ['cost', 'occupied', 'collision', 'clearance_radius_blocked']):
        reasons.append(f'topology blocked reason counts suggest costmap/clearance blocking: {blocked_counts}')
        return 'BOUNDED_SMOKE_NO_DISPATCH_COSTMAP_BLOCKED', reasons
    if local_costmap_text and any(token in local_costmap_text for token in ['data:', 'metadata:']):
        if mode == 'FAILED_EXHAUSTED' or 'no untried branches' in terminal_reason or topo_kind == 'unknown' or open_count == 0 or candidate_count == 0:
            reasons.append(f'topology failed before dispatch: mode={mode}, topology={topo_kind}, open={open_count}, candidates={candidate_count}, terminal={terminal_reason}')
            return 'BOUNDED_SMOKE_NO_DISPATCH_TOPOLOGY_SAMPLING', reasons
    if re.search(r'transform|extrapolat|lookup would require|TF', launch_log, flags=re.I):
        reasons.append('launch log contains TF/transform warning/error terms')
        return 'BOUNDED_SMOKE_NO_DISPATCH_TF_ALIGNMENT', reasons
    reasons.append('no dispatch with insufficient subtype evidence')
    return 'BOUNDED_SMOKE_NO_DISPATCH_OTHER', reasons


def analyze(artifact_dir: Path, run_id: str) -> dict[str, Any]:
    goal_events_path = artifact_dir / f'{run_id}_goal_events.jsonl'
    explorer_state_path = artifact_dir / f'{run_id}_explorer_state.jsonl'
    launch_log_path = artifact_dir / f'{run_id}_launch.log'
    action_info_path = artifact_dir / f'{run_id}_navigate_to_pose_action_info.txt'
    cleanup_path = artifact_dir / f'{run_id}_cleanup_processes_after.txt'

    goal_rows = read_jsonl(goal_events_path)
    state_rows = read_jsonl(explorer_state_path)
    goal_payloads = [payload(row) for row in goal_rows]
    state_payloads = [payload(row) for row in state_rows]
    final_state = state_payloads[-1] if state_payloads else {}
    launch_log = read_text(launch_log_path)
    action_info = read_text(action_info_path)
    cleanup_text = read_text(cleanup_path).strip()

    dispatch_events = [p for p in goal_payloads if p.get('event') in {'dispatch', 'goal_dispatch', 'navigate_to_pose_dispatch'}]
    outcome_events = [p for p in goal_payloads if p.get('event') in {'outcome', 'goal_result', 'nav2_result'}]
    exit_reached_events = [p for p in goal_payloads if p.get('event') == 'exit_reached' or p.get('mode') == 'EXIT_REACHED']
    if final_state.get('mode') == 'EXIT_REACHED':
        exit_reached_events.append(final_state)

    active_map_frame_truth_used = all(token in launch_log or token in read_text(artifact_dir / f'{run_id}_preflight.txt') for token in [
        'entrance_x=0.0', 'entrance_y=0.0', 'exit_x=21.072562', 'exit_y=18.083566', 'exit_radius=1.2'
    ])
    legacy_truth_seen = any(token in launch_log for token in LEGACY_TRUTH_TOKENS)
    nav2_action_server_available = 'Action server: /navigate_to_pose' in action_info or '/bt_navigator' in action_info
    maze_explorer_started = 'maze_explorer' in launch_log or bool(state_rows)
    explorer_state_has_data = bool(state_rows)
    goal_events_has_data = bool(goal_rows)
    cleanup_empty = cleanup_text == ''
    complete_autonomous_success_claimed = bool(exit_reached_events)

    evidence: dict[str, Any] = {
        'run_id': run_id,
        'artifact_dir': str(artifact_dir),
        'active_map_frame_truth_used': active_map_frame_truth_used,
        'legacy_truth_seen': legacy_truth_seen,
        'maze_explorer_started': maze_explorer_started,
        'explorer_state_samples': len(state_rows),
        'goal_events_samples': len(goal_rows),
        'goal_events_has_data': goal_events_has_data,
        'dispatch_events': len(dispatch_events),
        'outcome_events': len(outcome_events),
        'exit_reached_events': len(exit_reached_events),
        'dispatch_expected': True,
        'at_least_one_dispatch': bool(dispatch_events),
        'nav2_action_server_available': nav2_action_server_available,
        'cleanup_empty': cleanup_empty,
        'final_state': final_state,
        'last_goal_event': goal_payloads[-1] if goal_payloads else {},
        'complete_autonomous_success_claimed': complete_autonomous_success_claimed,
        'artifacts': {
            'launch_log': str(launch_log_path),
            'explorer_state_jsonl': str(explorer_state_path),
            'goal_events_jsonl': str(goal_events_path),
            'acceptance_analysis_json': str(artifact_dir / f'{run_id}_acceptance_analysis.json'),
            'summary_json': str(artifact_dir / f'{run_id}_summary.json'),
        },
        'required_sample_files': {
            name: file_nonempty(artifact_dir / f'{run_id}_{name}')
            for name in [
                'map_once.txt', 'scan_once.txt', 'odom_once.txt', 'tf_once.txt',
                'local_costmap_once.txt', 'global_costmap_once.txt', 'navigate_to_pose_action_info.txt'
            ]
        },
        'allowed_classifications': PHASE42_ALLOWED_CLASSIFICATIONS,
        'classification_reasons': [],
    }

    reasons = evidence['classification_reasons']
    if not active_map_frame_truth_used or legacy_truth_seen:
        reasons.append('active map-frame truth was not proven or legacy truth appeared')
        classification = 'BOUNDED_SMOKE_INSUFFICIENT_EVIDENCE'
    elif not maze_explorer_started or not explorer_state_has_data:
        reasons.append('maze_explorer/explorer_state evidence missing')
        classification = 'BOUNDED_SMOKE_INSUFFICIENT_EVIDENCE'
    elif not nav2_action_server_available:
        reasons.append('NavigateToPose action server not available')
        classification = 'BOUNDED_SMOKE_INSUFFICIENT_EVIDENCE'
    elif dispatch_events:
        reasons.append(f'dispatch_events={len(dispatch_events)}')
        if complete_autonomous_success_claimed:
            reasons.append('exit reached evidence present in bounded run')
            classification = 'BOUNDED_SMOKE_REACHED_EXIT_WITH_EVIDENCE'
        else:
            reasons.append('bounded run dispatched at least one autonomous goal; not complete exit success')
            classification = 'BOUNDED_SMOKE_DISPATCHED_GOAL'
    elif not cleanup_empty:
        reasons.append('cleanup artifact is not empty')
        classification = 'BOUNDED_SMOKE_INSUFFICIENT_EVIDENCE'
    else:
        classification, subtype_reasons = classify_no_dispatch(final_state, launch_log, artifact_dir, evidence)
        reasons.extend(subtype_reasons)

    evidence['classification'] = classification
    evidence['acceptance'] = {
        'active_map_frame_truth_used': active_map_frame_truth_used,
        'no_old_truth': not legacy_truth_seen,
        'maze_explorer_started': maze_explorer_started,
        'explorer_state_has_data': explorer_state_has_data,
        'goal_events_has_data_or_explained_no_dispatch': goal_events_has_data or classification.startswith('BOUNDED_SMOKE_NO_DISPATCH'),
        'at_least_one_dispatch_expected': True,
        'at_least_one_dispatch_observed': bool(dispatch_events),
        'nav2_action_server_available': nav2_action_server_available,
        'cleanup_empty': cleanup_empty,
    }
    evidence['bounded_smoke_partial_pass'] = classification in {'BOUNDED_SMOKE_DISPATCHED_GOAL', 'BOUNDED_SMOKE_REACHED_EXIT_WITH_EVIDENCE'}
    return evidence


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--artifact-dir', type=Path, default=Path('log/phase42_map_frame_truth_bounded_smoke'))
    parser.add_argument('--run-id', default='phase42_map_frame_truth_bounded_smoke')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    artifact_dir = args.artifact_dir.expanduser().resolve()
    result = analyze(artifact_dir, args.run_id)
    output = args.output or artifact_dir / f'{args.run_id}_acceptance_analysis.json'
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
