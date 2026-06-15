#!/usr/bin/env python3
"""Phase26K spatial/local-cost alignment for high-cost forward-shift cases.

Analysis-only: consumes existing goal_events and post_recovery_snapshots artifacts.
It does not start ROS, Gazebo, Nav2, recorders, or change branch-selection or
controller parameters.
"""

from __future__ import annotations

import argparse
from collections import Counter
import json
import math
from pathlib import Path
from typing import Any

HIGH_COST_THRESHOLD = 70.0
ALIGN_RADIUS_M = 0.20
ROUTE_MARGIN_M = 0.08
DISPATCH_SHARED_RADIUS_M = 0.18


def load_jsonl_payloads(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if not path.exists() or path.stat().st_size == 0:
        return rows
    for line in path.read_text(encoding='utf-8').splitlines():
        if not line.strip():
            continue
        row = json.loads(line)
        payload = row.get('state', row)
        if isinstance(payload, dict):
            rows.append(payload)
    return rows


def number(value: Any) -> float | None:
    if value is None or isinstance(value, bool):
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def target_payload(value: Any) -> list[float] | None:
    if isinstance(value, (list, tuple)) and len(value) >= 2:
        x = number(value[0])
        y = number(value[1])
        if x is not None and y is not None:
            return [round(x, 6), round(y, 6)]
    return None


def xy(value: Any) -> list[float] | None:
    payload = target_payload(value)
    return payload[:2] if payload is not None else None


def dist(a: list[float] | None, b: list[float] | None) -> float | None:
    if a is None or b is None:
        return None
    return math.hypot(a[0] - b[0], a[1] - b[1])


def point_segment_distance(p: list[float], a: list[float], b: list[float]) -> float:
    ax, ay = a
    bx, by = b
    px, py = p
    dx = bx - ax
    dy = by - ay
    denom = dx * dx + dy * dy
    if denom == 0.0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / denom))
    proj = [ax + t * dx, ay + t * dy]
    return math.hypot(px - proj[0], py - proj[1])


def segment_segment_distance(a1: list[float], a2: list[float], b1: list[float], b2: list[float]) -> float:
    return min(
        point_segment_distance(a1, b1, b2),
        point_segment_distance(a2, b1, b2),
        point_segment_distance(b1, a1, a2),
        point_segment_distance(b2, a1, a2),
    )


def ray_end_from_pose(pose: list[float], distance_m: float = 1.0) -> list[float] | None:
    if len(pose) < 3:
        return None
    yaw = number(pose[2])
    if yaw is None:
        return None
    return [pose[0] + math.cos(yaw) * distance_m, pose[1] + math.sin(yaw) * distance_m]


def classify_distances(chosen_dist: float | None, explored_dist: float | None, dispatch_dist: float | None = None) -> str:
    if chosen_dist is None or explored_dist is None:
        return 'unknown'
    if dispatch_dist is not None and dispatch_dist <= DISPATCH_SHARED_RADIUS_M:
        if chosen_dist <= ALIGN_RADIUS_M and explored_dist <= ALIGN_RADIUS_M:
            return 'shared_dispatch_or_corridor_choke'
    if chosen_dist <= ALIGN_RADIUS_M and explored_dist <= ALIGN_RADIUS_M and abs(chosen_dist - explored_dist) <= ROUTE_MARGIN_M:
        return 'shared_dispatch_or_corridor_choke'
    if chosen_dist + ROUTE_MARGIN_M < explored_dist:
        return 'chosen_route'
    if explored_dist + ROUTE_MARGIN_M < chosen_dist:
        return 'explored_candidate_corridor'
    return 'ambiguous_between_chosen_and_explored'


def load_dispatch(log_dir: Path, run_id: str, seq: int) -> dict[str, Any] | None:
    for row in load_jsonl_payloads(log_dir / f'{run_id}_goal_events.jsonl'):
        if row.get('event') == 'dispatch' and int(number(row.get('goal_sequence')) or -1) == seq:
            return row
    return None


def load_outcome(log_dir: Path, run_id: str, seq: int) -> dict[str, Any]:
    outcome = {'event': None, 'result_reason': None}
    for row in load_jsonl_payloads(log_dir / f'{run_id}_goal_events.jsonl'):
        if row.get('event') in ('success', 'timeout', 'failure', 'terminal_cancel', 'terminal_cancel_result') and int(number(row.get('goal_sequence')) or -1) == seq:
            outcome = {'event': row.get('event'), 'result_reason': row.get('result_reason')}
    return outcome


def selected_explored_candidate(dispatch: dict[str, Any]) -> dict[str, Any] | None:
    candidates = [c for c in dispatch.get('candidate_branches') or [] if isinstance(c, dict) and c.get('rejection_reason') == 'explored']
    if not candidates:
        return None
    def key(candidate: dict[str, Any]) -> float:
        value = number(candidate.get('target_exit_dist'))
        return value if value is not None else float('inf')
    return min(candidates, key=key)


def snapshot_rows(log_dir: Path, run_id: str, seq: int) -> list[dict[str, Any]]:
    rows = []
    for row in load_jsonl_payloads(log_dir / f'{run_id}_post_recovery_snapshots.jsonl'):
        if int(number(row.get('goal_sequence')) or -1) == seq and row.get('robot_pose'):
            rows.append(row)
    return rows


def compact_snapshot(row: dict[str, Any]) -> dict[str, Any]:
    pose = row.get('robot_pose')
    compact_pose = target_payload(pose)
    if compact_pose is not None and isinstance(pose, list) and len(pose) >= 3:
        compact_pose = [*compact_pose, round(float(pose[2]), 6)]
    return {
        'snapshot_type': row.get('snapshot_type'),
        'robot_pose': compact_pose,
        'path_ahead_0_5m_cost_max': number(row.get('path_ahead_0_5m_cost_max')),
        'path_ahead_1_0m_cost_max': number(row.get('path_ahead_1_0m_cost_max')),
        'path_ahead_1_0m_cost_mean': number(row.get('path_ahead_1_0m_cost_mean')),
        'robot_to_path_distance_m': number(row.get('robot_to_path_distance_m')),
        'post_recovery_path_update_count': int(number(row.get('post_recovery_path_update_count')) or 0),
    }


def route_geometry(dispatch: dict[str, Any], explored: dict[str, Any]) -> dict[str, Any]:
    dispatch_xy = xy(dispatch.get('dispatch_pose'))
    chosen_xy = xy(dispatch.get('target'))
    explored_xy = xy(explored.get('target'))
    return {
        'dispatch_xy': dispatch_xy,
        'chosen_target': chosen_xy,
        'explored_candidate_target': explored_xy,
        'chosen_target_exit_dist': number(dispatch.get('target_exit_dist')),
        'explored_candidate_target_exit_dist': number(explored.get('target_exit_dist')),
        'chosen_target_local_cost': number(dispatch.get('dispatch_target_local_cost')),
        'explored_candidate_target_local_cost': number(explored.get('target_local_cost')),
        'explored_candidate_target_local_cost_max_radius': number(explored.get('target_local_cost_max_radius')),
    }


def point_alignment(row: dict[str, Any], geom: dict[str, Any]) -> dict[str, Any]:
    pose = row.get('robot_pose')
    if not isinstance(pose, list) or len(pose) < 2:
        return {'classification': 'unknown'}
    p = [float(pose[0]), float(pose[1])]
    d = geom.get('dispatch_xy')
    chosen = geom.get('chosen_target')
    explored = geom.get('explored_candidate_target')
    if d is None or chosen is None or explored is None:
        return {'classification': 'unknown'}
    chosen_dist = point_segment_distance(p, d, chosen)
    explored_dist = point_segment_distance(p, d, explored)
    dispatch_dist = dist(p, d)
    return {
        'classification': classify_distances(chosen_dist, explored_dist, dispatch_dist),
        'distance_to_chosen_route_m': round(chosen_dist, 6),
        'distance_to_explored_candidate_route_m': round(explored_dist, 6),
        'distance_to_dispatch_m': round(dispatch_dist, 6) if dispatch_dist is not None else None,
    }


def ray_alignment(row: dict[str, Any], geom: dict[str, Any]) -> dict[str, Any]:
    pose = row.get('robot_pose')
    if not isinstance(pose, list) or len(pose) < 3:
        return {'classification': 'unknown'}
    p = [float(pose[0]), float(pose[1])]
    ray_end = ray_end_from_pose([float(pose[0]), float(pose[1]), float(pose[2])])
    d = geom.get('dispatch_xy')
    chosen = geom.get('chosen_target')
    explored = geom.get('explored_candidate_target')
    if d is None or chosen is None or explored is None or ray_end is None:
        return {'classification': 'unknown'}
    chosen_dist = segment_segment_distance(p, ray_end, d, chosen)
    explored_dist = segment_segment_distance(p, ray_end, d, explored)
    dispatch_dist = dist(p, d)
    endpoint_to_chosen_target = dist(ray_end, chosen)
    endpoint_to_explored_target = dist(ray_end, explored)
    classification = classify_distances(chosen_dist, explored_dist, dispatch_dist)
    if classification == 'ambiguous_between_chosen_and_explored' and endpoint_to_chosen_target is not None and endpoint_to_explored_target is not None:
        if endpoint_to_chosen_target + ROUTE_MARGIN_M < endpoint_to_explored_target:
            classification = 'chosen_route'
        elif endpoint_to_explored_target + ROUTE_MARGIN_M < endpoint_to_chosen_target:
            classification = 'explored_candidate_corridor'
    return {
        'classification': classification,
        'ray_end_xy': [round(ray_end[0], 6), round(ray_end[1], 6)],
        'ray_distance_to_chosen_route_m': round(chosen_dist, 6),
        'ray_distance_to_explored_candidate_route_m': round(explored_dist, 6),
        'ray_endpoint_to_chosen_target_m': round(endpoint_to_chosen_target, 6) if endpoint_to_chosen_target is not None else None,
        'ray_endpoint_to_explored_candidate_target_m': round(endpoint_to_explored_target, 6) if endpoint_to_explored_target is not None else None,
        'distance_to_dispatch_m': round(dispatch_dist, 6) if dispatch_dist is not None else None,
    }


def route_context(dispatch: dict[str, Any], explored: dict[str, Any]) -> str:
    robot_exit = number(dispatch.get('robot_exit_dist_at_dispatch'))
    chosen_exit = number(dispatch.get('target_exit_dist'))
    explored_exit = number(explored.get('target_exit_dist'))
    if robot_exit is None or chosen_exit is None or explored_exit is None:
        return 'unknown'
    chosen_delta = robot_exit - chosen_exit
    explored_delta = robot_exit - explored_exit
    if chosen_delta < -0.05 and explored_delta > 0.05:
        return 'route_divergence_chosen_moves_away_rejected_moves_toward_exit'
    if chosen_delta > 0.05:
        return 'non_divergent_chosen_also_toward_exit'
    return 'non_divergent_or_unknown_chosen_progress'


def analyze_case(log_dir: Path, run_id: str, seq: int, label: str | None = None) -> dict[str, Any]:
    dispatch = load_dispatch(log_dir, run_id, seq)
    if dispatch is None:
        return {'run_id': run_id, 'goal_sequence': seq, 'label': label, 'error': 'dispatch_not_found'}
    explored = selected_explored_candidate(dispatch)
    if explored is None:
        return {'run_id': run_id, 'goal_sequence': seq, 'label': label, 'error': 'explored_candidate_not_found'}
    geom = route_geometry(dispatch, explored)
    rows = snapshot_rows(log_dir, run_id, seq)
    high_rows = [row for row in rows if (number(row.get('path_ahead_1_0m_cost_max')) or 0.0) >= HIGH_COST_THRESHOLD]
    near_zero = next((row for row in rows if row.get('snapshot_type') == 'near_zero_onset'), None)
    high_alignments = []
    for row in high_rows:
        high_alignments.append({
            **compact_snapshot(row),
            'point_alignment': point_alignment(row, geom),
            'ray_alignment': ray_alignment(row, geom),
        })
    class_counts = Counter(item['point_alignment'].get('classification', 'unknown') for item in high_alignments)
    if not high_alignments:
        distribution = 'no_high_cost_path_ahead_samples'
    elif class_counts.get('chosen_route', 0) > max(class_counts.get('explored_candidate_corridor', 0), class_counts.get('shared_dispatch_or_corridor_choke', 0), class_counts.get('ambiguous_between_chosen_and_explored', 0)):
        distribution = 'chosen_route_dominant'
    elif class_counts.get('explored_candidate_corridor', 0) > max(class_counts.get('chosen_route', 0), class_counts.get('shared_dispatch_or_corridor_choke', 0), class_counts.get('ambiguous_between_chosen_and_explored', 0)):
        distribution = 'explored_candidate_corridor_dominant'
    else:
        distribution = 'shared_or_ambiguous'
    near_zero_alignment: dict[str, Any] = ray_alignment(near_zero, geom) if near_zero is not None else {'classification': 'missing_near_zero_onset'}
    if near_zero is not None:
        near_zero_alignment['snapshot'] = compact_snapshot(near_zero)
        near_zero_alignment['point_alignment'] = point_alignment(near_zero, geom)
    explored_corridor_clean = class_counts.get('explored_candidate_corridor', 0) == 0 and class_counts.get('shared_dispatch_or_corridor_choke', 0) == 0
    return {
        'run_id': run_id,
        'goal_sequence': seq,
        'label': label,
        'outcome': load_outcome(log_dir, run_id, seq),
        'route_context': route_context(dispatch, explored),
        'geometry': geom,
        'high_cost_distribution': {
            'classification': distribution,
            'high_cost_sample_count': len(high_alignments),
            'point_alignment_counts': dict(sorted(class_counts.items())),
        },
        'near_zero_alignment': near_zero_alignment,
        'explored_candidate_corridor_clean': bool(explored_corridor_clean),
        'high_cost_samples': high_alignments,
    }


def high_cost_points(case: dict[str, Any]) -> list[list[float]]:
    points = []
    for row in case.get('high_cost_samples', []):
        pose = row.get('robot_pose')
        if isinstance(pose, list) and len(pose) >= 2:
            points.append([float(pose[0]), float(pose[1])])
    return points


def min_overlap(anchor: dict[str, Any], other: dict[str, Any]) -> float | None:
    a_points = high_cost_points(anchor)
    b_points = high_cost_points(other)
    if not a_points or not b_points:
        return None
    best = min(math.hypot(a[0] - b[0], a[1] - b[1]) for a in a_points for b in b_points)
    return round(best, 6)


def parse_compare_cases(value: str | None) -> list[tuple[str, int, str]]:
    if not value:
        return []
    result = []
    for item in value.split(','):
        if not item.strip():
            continue
        parts = item.split(':')
        if len(parts) < 2:
            raise ValueError(f'Invalid compare case: {item}')
        run_id = parts[0]
        seq = int(parts[1])
        label = parts[2] if len(parts) >= 3 else f'{run_id}_{seq}'
        result.append((run_id, seq, label))
    return result


def decision(anchor: dict[str, Any], comparisons: list[dict[str, Any]]) -> dict[str, Any]:
    near_class = anchor.get('near_zero_alignment', {}).get('classification')
    high_class = anchor.get('high_cost_distribution', {}).get('classification')
    explored_clean = bool(anchor.get('explored_candidate_corridor_clean'))
    overlaps_success = any(
        (
            (overlap := number(case.get('min_high_cost_overlap_with_anchor_m'))) is not None
            and overlap <= 0.35
            and case.get('outcome', {}).get('event') == 'success'
        )
        for case in comparisons
    )
    if near_class == 'shared_dispatch_or_corridor_choke' or high_class == 'shared_or_ambiguous':
        interpretation = 'shared_choke_or_ambiguous'
        recommendation = 'analysis_only_continue_spatial_local_cost_diagnostics'
    elif high_class == 'chosen_route_dominant' and explored_clean and overlaps_success:
        interpretation = 'chosen_route_high_cost_with_success_overlap_not_explored_corridor'
        recommendation = 'analysis_only_consider_runtime_spatial_diagnostics_not_branch_selection'
    elif high_class == 'chosen_route_dominant' and explored_clean:
        interpretation = 'chosen_route_high_cost_explored_corridor_clean'
        recommendation = 'analysis_only_collect_runtime_spatial_diagnostics_before_any_intervention'
    elif high_class == 'explored_candidate_corridor_dominant':
        interpretation = 'explored_candidate_corridor_high_cost'
        recommendation = 'analysis_only_do_not_branch_select_into_high_cost_corridor'
    else:
        interpretation = 'mixed_or_insufficient_spatial_evidence'
        recommendation = 'analysis_only_continue_diagnostics'
    return {
        'spatial_interpretation': interpretation,
        'phase27_candidate_signal': 'not_supported',
        'next_recommendation': recommendation,
        'guardrails': [
            'do_not_enter_phase27',
            'do_not_change_branch_selection',
            'do_not_tune_nav2_or_controller_params_from_phase26k',
            'do_not_infer_clean_explored_corridor_without_runtime_cost_cells',
        ],
    }


def analyze(log_dir: Path, anchor_run: str, anchor_seq: int, compare_specs: list[tuple[str, int, str]]) -> dict[str, Any]:
    anchor = analyze_case(log_dir, anchor_run, anchor_seq, 'anchor')
    comparisons = []
    for run_id, seq, label in compare_specs:
        case = analyze_case(log_dir, run_id, seq, label)
        case['min_high_cost_overlap_with_anchor_m'] = min_overlap(anchor, case)
        comparisons.append(case)
    return {
        'phase': '26K',
        'purpose': 'spatial/local-cost alignment of high-cost path-ahead samples against chosen route and explored candidate corridor',
        'analysis_only': True,
        'anchor_case': anchor,
        'comparison_cases': comparisons,
        'decision': decision(anchor, comparisons),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log-dir', type=Path, default=Path('log'))
    parser.add_argument('--anchor-run', required=True)
    parser.add_argument('--anchor-seq', type=int, required=True)
    parser.add_argument('--compare-cases', default='', help='Comma-separated run:seq:label items')
    parser.add_argument('--output-json', type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    payload = analyze(args.log_dir, args.anchor_run, args.anchor_seq, parse_compare_cases(args.compare_cases))
    text = json.dumps(payload, indent=2, sort_keys=True) + '\n'
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(text, encoding='utf-8')
    print(text)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
