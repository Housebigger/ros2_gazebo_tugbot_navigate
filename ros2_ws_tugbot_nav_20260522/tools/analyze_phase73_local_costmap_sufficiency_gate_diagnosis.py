#!/usr/bin/env python3
"""Phase73 Local Costmap Sufficiency Gate Diagnosis / Threshold Semantics Review.

This is a read-only offline analyzer for Phase72 artifacts. It diagnoses the
post-Goal1 local_costmap_sufficient readiness gate semantics without changing
Nav2/MPPI/controller parameters, inflation/robot radius/clearance/map thresholds,
branch scoring, centerline runtime behavior, fallback/terminal acceptance, or
claiming autonomous/exit success.

The key question is whether a low full-window free_ratio means the robot is truly
blocked, or whether a narrow maze corridor naturally makes a 1 m circular window
contain too much wall/inflation for a global free-ratio gate even though the robot
nearby footprint and candidate-direction corridor slice may still be traversable.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

PHASE = 'Phase73 Local Costmap Sufficiency Gate Diagnosis / Threshold Semantics Review'
RUN_ID = 'phase73_local_costmap_sufficiency_gate_diagnosis'
PHASE72_RUN_ID = 'phase72_multigoal_bounded_rerun_from_inner_ingress'
PHASE72_CLASSIFICATION = 'MULTIGOAL_NO_REDISPATCH_AFTER_SUCCESS'
PHASE72_SUBCLASSIFICATION = 'POST_SUCCESS_REDISPATCH_BLOCKED_BY_LOCAL_COSTMAP_SUFFICIENCY'

HIGH_COST_THRESHOLD = 70
LETHAL_COST_THRESHOLD = 99
MIN_DIRECTION_CLEARANCE_M = 0.35

ALLOWED_CLASSIFICATIONS = [
    'FULL_WINDOW_FREE_RATIO_TOO_STRICT',
    'LOCAL_COSTMAP_TRUE_BLOCKED',
    'LOCAL_DIRECTION_TRAVERSABLE_GATE_SHOULD_REPLACE_GLOBAL_RATIO',
    'INSUFFICIENT_EVIDENCE',
]

GUARDRAILS = [
    'read-only Phase72 artifact analysis',
    'no Nav2/MPPI/controller tuning',
    'no inflation/robot_radius/clearance_radius_m/map threshold tuning',
    'no branch scoring change',
    'no centerline gate change',
    'no fallback/terminal acceptance change',
    'no autonomous exploration success claim',
    'no exit success claim',
]


def _safe_loads(text: str) -> Any | None:
    try:
        return json.loads(text)
    except Exception:
        return None


def _read_json(path: Path | None, default: Any) -> Any:
    if path is None or not path.exists() or not path.stat().st_size:
        return default
    try:
        return json.loads(path.read_text(encoding='utf-8', errors='replace'))
    except Exception:
        return default


def _read_jsonl(path: Path | None) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if path is None or not path.exists() or not path.stat().st_size:
        return rows
    for line in path.read_text(encoding='utf-8', errors='replace').splitlines():
        if not line.strip():
            continue
        raw = _safe_loads(line)
        if not isinstance(raw, dict):
            continue
        payload = raw.get('state') if isinstance(raw.get('state'), dict) else raw
        if not isinstance(payload, dict):
            continue
        row = dict(payload)
        row.setdefault('_recorder_elapsed_sec', raw.get('elapsed_sec'))
        row.setdefault('_recorder_wall_time', raw.get('wall_time'))
        rows.append(row)
    return rows


def _number(value: Any) -> float | None:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return out if math.isfinite(out) else None


def _int_or_none(value: Any) -> int | None:
    try:
        if value is None:
            return None
        return int(value)
    except (TypeError, ValueError):
        return None


def _first_non_none(*values: Any) -> Any:
    for value in values:
        if value is not None:
            return value
    return None


def _latest(rows: list[dict[str, Any]], predicate) -> dict[str, Any] | None:
    for row in reversed(rows):
        try:
            if predicate(row):
                return row
        except Exception:
            continue
    return None


def _ratio(numer: int | None, denom: int | None) -> float | None:
    if numer is None or denom is None or denom <= 0:
        return None
    return float(numer / denom)


def _summary_from_cost_dict(raw: dict[str, Any] | None) -> dict[str, Any]:
    raw = raw or {}
    sample_count = _int_or_none(raw.get('sample_count'))
    in_bounds = _int_or_none(raw.get('in_bounds_sample_count'))
    high = _int_or_none(raw.get('high_cost_count')) or 0
    lethal = _int_or_none(raw.get('lethal_count')) or 0
    max_value = _number(raw.get('max'))
    mean_value = _number(raw.get('mean'))
    return {
        'sample_count': sample_count,
        'in_bounds_sample_count': in_bounds,
        'max': max_value,
        'mean': mean_value,
        'high_cost_count': high,
        'lethal_count': lethal,
        'high_cost_ratio': _ratio(high, in_bounds or sample_count),
        'lethal_ratio': _ratio(lethal, in_bounds or sample_count),
        'inflated_count_estimate': max(0, high - lethal),
        'inflated_ratio_estimate': _ratio(max(0, high - lethal), in_bounds or sample_count),
        'source_semantics': 'Phase72 local-cost summary: high_cost_count>=70, lethal_count>=99; inflated_count_estimate=high_cost_count-lethal_count.',
    }


def _full_window_distribution(local: dict[str, Any] | None) -> dict[str, Any]:
    local = local or {}
    sample_count = _int_or_none(local.get('sample_count'))
    free = _int_or_none(local.get('free_count'))
    occupied = _int_or_none(local.get('occupied_count'))
    unknown = _int_or_none(local.get('unknown_count'))
    known = _int_or_none(local.get('known_count'))
    min_free_ratio = _number(local.get('min_free_ratio'))
    free_ratio = _number(local.get('free_ratio'))
    occupied_ratio = _number(local.get('occupied_ratio'))
    unknown_ratio = _number(local.get('unknown_ratio'))
    if free_ratio is None:
        free_ratio = _ratio(free, sample_count)
    if occupied_ratio is None:
        occupied_ratio = _ratio(occupied, sample_count)
    if unknown_ratio is None:
        unknown_ratio = _ratio(unknown, sample_count)
    # The readiness gate payload gives occupied_count but not raw cell values, so
    # exact wall-vs-inflation-vs-lethal decomposition is unavailable offline.
    # Keep a conservative estimate rather than inventing raw costmap bins.
    inflated_count_estimate = occupied
    return {
        'sample_count': sample_count,
        'in_bounds_count': _int_or_none(local.get('in_bounds_count')),
        'out_of_bounds_count': _int_or_none(local.get('out_of_bounds_count')),
        'known_count': known,
        'free_count': free,
        'occupied_count': occupied,
        'unknown_count': unknown,
        'free_ratio': free_ratio,
        'occupied_ratio': occupied_ratio,
        'unknown_ratio': unknown_ratio,
        'known_ratio': _number(local.get('known_ratio')),
        'min_free_ratio': min_free_ratio,
        'min_known_ratio': _number(local.get('min_known_ratio')),
        'free_ratio_margin_to_threshold': (free_ratio - min_free_ratio) if free_ratio is not None and min_free_ratio is not None else None,
        'radius_m': _number(local.get('radius_m')),
        'sample_age_sec': _number(local.get('sample_age_sec')),
        'grid_resolution': _number(local.get('grid_resolution')),
        'grid_width': _int_or_none(local.get('grid_width')),
        'grid_height': _int_or_none(local.get('grid_height')),
        'robot_in_bounds': local.get('robot_in_bounds'),
        'sample_window': local.get('sample_window'),
        'bbox_world': local.get('bbox_world'),
        'reason': local.get('reason'),
        'sufficient': local.get('sufficient'),
        'inflated_count_estimate': inflated_count_estimate,
        'inflated_ratio_estimate': _ratio(inflated_count_estimate, sample_count),
        'decomposition_limitation': 'Phase72 gate records free/occupied/unknown counts only; occupied combines lethal walls/obstacles and inflation-layer costs.',
    }


def _is_goal1_success(events: list[dict[str, Any]]) -> bool:
    return any(row.get('event') in {'success', 'goal_success', 'outcome'} and int(row.get('goal_sequence') or -1) == 1 and str(row.get('result_reason') or row.get('outcome_event') or 'success').lower() in {'success', 'succeeded', 'goal_succeeded'} for row in events)


def _post_success_blocked_state(states: list[dict[str, Any]]) -> dict[str, Any] | None:
    def is_blocked(row: dict[str, Any]) -> bool:
        gate = row.get('dispatch_readiness_gate') if isinstance(row.get('dispatch_readiness_gate'), dict) else {}
        blocking = row.get('dispatch_readiness_blocking_reasons') or gate.get('blocking_reasons') or []
        checks = gate.get('checks') if isinstance(gate.get('checks'), dict) else {}
        return (
            int(row.get('goal_count') or 0) >= 1
            and not bool(row.get('goal_active'))
            and (
                int(row.get('goal_success_count') or 0) >= 1
                or row.get('mode') == 'WAIT_FOR_DISPATCH_ENTRY_READINESS'
            )
            and (
                'local_costmap_sufficient' in blocking
                or checks.get('local_costmap_sufficient') is False
            )
        )
    return _latest(states, is_blocked)


def _latest_local_sample(local_rows: list[dict[str, Any]], blocked_state: dict[str, Any] | None = None) -> dict[str, Any] | None:
    if not local_rows:
        return None
    # If both sources have elapsed_sec, choose the nearest local sample at or before
    # the blocked state. Otherwise the final sample is the best Phase72 proxy for
    # post-success readiness hold.
    blocked_elapsed = _number((blocked_state or {}).get('_recorder_elapsed_sec'))
    if blocked_elapsed is not None:
        candidates = [row for row in local_rows if (_number(row.get('elapsed_sec')) is not None and _number(row.get('elapsed_sec')) <= blocked_elapsed + 2.0)]
        if candidates:
            return candidates[-1]
    return local_rows[-1]


def _candidate_branches(events: list[dict[str, Any]], states: list[dict[str, Any]]) -> list[dict[str, Any]]:
    dispatch = _latest(events, lambda row: row.get('event') == 'dispatch' and isinstance(row.get('candidate_branches'), list))
    branches = dispatch.get('candidate_branches') if dispatch else None
    if isinstance(branches, list) and branches:
        return [b for b in branches if isinstance(b, dict)]
    # Fallback to any candidate diagnostics embedded in the latest state.
    state = _latest(states, lambda row: isinstance(row.get('phase56_open_direction_to_candidate_diagnostics'), dict))
    diag = state.get('phase56_open_direction_to_candidate_diagnostics') if state else None
    if isinstance(diag, dict) and isinstance(diag.get('candidate_branches'), list):
        return [b for b in diag['candidate_branches'] if isinstance(b, dict)]
    return []


def _topology_local_cost_counts(blocked_state: dict[str, Any] | None) -> dict[str, Any]:
    topo = (blocked_state or {}).get('last_topology_sampling_diagnostics')
    if not isinstance(topo, dict):
        topo = {}
    samples = topo.get('samples') if isinstance(topo.get('samples'), list) else []
    counts: dict[str, int] = {}
    for sample in samples:
        if not isinstance(sample, dict):
            continue
        local = sample.get('local_costmap') if isinstance(sample.get('local_costmap'), dict) else {}
        result = sample.get('costmap_lethal_or_unknown_result') or local.get('costmap_lethal_or_unknown_result') or 'missing'
        counts[str(result)] = counts.get(str(result), 0) + 1
    return {
        'sample_count': len(samples),
        'result_counts': counts,
        'clear_or_inflated_count': int(counts.get('clear', 0) + counts.get('inflated', 0)),
        'blocked_like_count': int(counts.get('lethal_or_obstacle', 0) + counts.get('unknown', 0) + counts.get('out_of_bounds', 0)),
        'raw_open_direction_count': _first_non_none(topo.get('raw_open_direction_count'), (blocked_state or {}).get('raw_open_direction_count'), (blocked_state or {}).get('last_open_direction_count')),
        'filtered_open_direction_count': _first_non_none(topo.get('filtered_open_direction_count'), (blocked_state or {}).get('filtered_open_direction_count'), (blocked_state or {}).get('last_open_direction_count')),
        'candidate_after_filter_count': _first_non_none(topo.get('candidate_after_filter_count'), (blocked_state or {}).get('candidate_after_filter_count'), (blocked_state or {}).get('last_candidate_count')),
        'candidate_branch_count': _first_non_none(topo.get('candidate_branch_count'), (blocked_state or {}).get('candidate_branch_count'), (blocked_state or {}).get('last_candidate_count')),
    }


def _branch_number(branch: dict[str, Any], key: str) -> float | None:
    return _number(branch.get(key))


def _candidate_direction_corridor_slices(branches: list[dict[str, Any]], blocked_state: dict[str, Any] | None) -> dict[str, Any]:
    slices: list[dict[str, Any]] = []
    traversable_count = 0
    non_reverse_traversable_count = 0
    for branch in branches:
        target_cost = _branch_number(branch, 'target_local_cost')
        target_radius = _branch_number(branch, 'target_local_cost_max_radius')
        path_max = _branch_number(branch, 'dispatch_path_local_cost_max')
        path_mean = _branch_number(branch, 'dispatch_path_local_cost_mean')
        clearance = _branch_number(branch, 'path_corridor_min_clearance_m') or _branch_number(branch, 'target_clearance_m')
        reverse = bool(branch.get('is_reverse_candidate'))
        # A direction corridor slice is deliberately narrower than target-region
        # safety: it asks whether the route direction/path corridor contains a
        # locally passable slice. A candidate target can still have high radius or
        # footprint cost in a narrow corridor, so target-region risk is reported
        # separately and not used to block this semantic review.
        corridor_ok = (
            (path_max is None or path_max < HIGH_COST_THRESHOLD)
            and (clearance is None or clearance >= MIN_DIRECTION_CLEARANCE_M)
        )
        target_region_ok = (
            (target_cost is None or target_cost < HIGH_COST_THRESHOLD)
            and (target_radius is None or target_radius < LETHAL_COST_THRESHOLD)
        )
        if corridor_ok:
            traversable_count += 1
            if not reverse:
                non_reverse_traversable_count += 1
        slices.append({
            'rank': branch.get('rank'),
            'target': branch.get('target'),
            'branch_angle': branch.get('branch_angle'),
            'is_reverse_candidate': reverse,
            'rejection_reason': branch.get('rejection_reason'),
            'target_local_cost': target_cost,
            'target_local_cost_max_radius': target_radius,
            'dispatch_path_local_cost_max': path_max,
            'dispatch_path_local_cost_mean': path_mean,
            'path_corridor_min_clearance_m': clearance,
            'direction_corridor_traversable': bool(corridor_ok),
            'target_region_traversable': bool(target_region_ok),
            # Backward-compatible test/report alias.
            'direction_traversable': bool(corridor_ok),
            'semantic_note': 'diagnostic corridor slice from Phase72 candidate branch evidence; no branch scoring change made',
        })
    topo_counts = _topology_local_cost_counts(blocked_state)
    return {
        'candidate_count': len(branches),
        'slices': slices,
        'direction_traversable_count': traversable_count,
        'non_reverse_direction_traversable_count': non_reverse_traversable_count,
        'topology_local_cost_result_counts': topo_counts,
        'open_direction_count': _first_non_none(topo_counts.get('filtered_open_direction_count'), topo_counts.get('raw_open_direction_count')),
    }


def _target_footprint_front_wedge_local_cost(local_sample: dict[str, Any] | None) -> dict[str, Any]:
    local_sample = local_sample or {}
    target_ev = local_sample.get('local_costmap_target_evidence') if isinstance(local_sample.get('local_costmap_target_evidence'), dict) else {}
    radius_summary = target_ev.get('radius_cost_summary') if isinstance(target_ev.get('radius_cost_summary'), dict) else {}
    return {
        'sample_elapsed_sec': local_sample.get('elapsed_sec'),
        'goal_sequence': local_sample.get('goal_sequence'),
        'robot_pose': local_sample.get('robot_pose'),
        'dispatch_target': local_sample.get('dispatch_target'),
        'target_value': target_ev.get('value'),
        'target_in_bounds': target_ev.get('in_bounds'),
        'target_radius_cost_summary': _summary_from_cost_dict(radius_summary),
        'robot_footprint_cost': _summary_from_cost_dict(local_sample.get('robot_footprint_cost') if isinstance(local_sample.get('robot_footprint_cost'), dict) else {}),
        'target_footprint_cost': _summary_from_cost_dict(local_sample.get('target_footprint_cost') if isinstance(local_sample.get('target_footprint_cost'), dict) else {}),
        'front_wedge_cost': _summary_from_cost_dict(local_sample.get('front_wedge_cost') if isinstance(local_sample.get('front_wedge_cost'), dict) else {}),
    }


def _readiness_gate_summary(blocked_state: dict[str, Any] | None) -> dict[str, Any]:
    gate = (blocked_state or {}).get('dispatch_readiness_gate')
    if not isinstance(gate, dict):
        gate = {}
    checks = gate.get('checks') if isinstance(gate.get('checks'), dict) else {}
    blocking = (blocked_state or {}).get('dispatch_readiness_blocking_reasons') or gate.get('blocking_reasons') or []
    local = gate.get('local_costmap') if isinstance(gate.get('local_costmap'), dict) else {}
    other_checks_passed = all(
        checks.get(key) is True
        for key in ['map_sufficient', 'scan_sufficient', 'tf_sufficient', 'nav2_lifecycle_active', 'navigate_to_pose_action_ready', 'goal_pose_subscriber_ready']
        if key in checks
    )
    return {
        'blocked_by_local_costmap_sufficient': 'local_costmap_sufficient' in blocking or checks.get('local_costmap_sufficient') is False,
        'blocking_reasons': blocking,
        'checks': checks,
        'other_readiness_checks_passed': other_checks_passed,
        'local_costmap_reason': local.get('reason'),
        'local_costmap_sample_age_sec': local.get('sample_age_sec'),
    }


def _classify_replay(full_window: dict[str, Any], footprint: dict[str, Any], wedge: dict[str, Any], candidate_slices: dict[str, Any], gate_summary: dict[str, Any]) -> str:
    if not gate_summary.get('blocked_by_local_costmap_sufficient'):
        return 'INSUFFICIENT_EVIDENCE'
    free_ratio = _number(full_window.get('free_ratio'))
    min_free = _number(full_window.get('min_free_ratio'))
    full_ratio_failed = free_ratio is not None and min_free is not None and free_ratio < min_free
    direction_count = int(candidate_slices.get('direction_traversable_count') or 0)
    non_reverse_direction_count = int(candidate_slices.get('non_reverse_direction_traversable_count') or 0)
    topo = candidate_slices.get('topology_local_cost_result_counts') if isinstance(candidate_slices.get('topology_local_cost_result_counts'), dict) else {}
    topo_clear_or_inflated = int(topo.get('clear_or_inflated_count') or 0)
    footprint_lethal = int(footprint.get('lethal_count') or 0)
    wedge_lethal = int(wedge.get('lethal_count') or 0)
    if full_ratio_failed and (direction_count > 0 or topo_clear_or_inflated > 0):
        if non_reverse_direction_count > 0:
            return 'LOCAL_DIRECTION_TRAVERSABLE_GATE_SHOULD_REPLACE_GLOBAL_RATIO'
        return 'FULL_WINDOW_FREE_RATIO_TOO_STRICT'
    if full_ratio_failed and direction_count == 0 and footprint_lethal > 0 and wedge_lethal > 0:
        return 'LOCAL_COSTMAP_TRUE_BLOCKED'
    if full_ratio_failed:
        return 'FULL_WINDOW_FREE_RATIO_TOO_STRICT'
    return 'INSUFFICIENT_EVIDENCE'


def _analyze_replay(replay_dir: Path) -> dict[str, Any]:
    replay_id = replay_dir.name
    events = _read_jsonl(replay_dir / f'{PHASE72_RUN_ID}_{replay_id}_goal_events.jsonl')
    states = _read_jsonl(replay_dir / f'{PHASE72_RUN_ID}_{replay_id}_explorer_state.jsonl')
    local_rows = _read_jsonl(replay_dir / 'phase72_local_costmap_samples.jsonl')
    blocked_state = _post_success_blocked_state(states)
    gate = (blocked_state or {}).get('dispatch_readiness_gate') if isinstance((blocked_state or {}).get('dispatch_readiness_gate'), dict) else {}
    local_gate = gate.get('local_costmap') if isinstance(gate.get('local_costmap'), dict) else {}
    local_sample = _latest_local_sample(local_rows, blocked_state)
    target_cost = _target_footprint_front_wedge_local_cost(local_sample)
    candidate_slices = _candidate_direction_corridor_slices(_candidate_branches(events, states), blocked_state)
    full_window = _full_window_distribution(local_gate)
    gate_summary = _readiness_gate_summary(blocked_state)
    footprint = target_cost['robot_footprint_cost']
    wedge = target_cost['front_wedge_cost']
    classification = _classify_replay(full_window, footprint, wedge, candidate_slices, gate_summary)
    return {
        'replay_id': replay_id,
        'artifact_dir': str(replay_dir),
        'classification': classification,
        'goal1_success_observed': _is_goal1_success(events),
        'post_success_local_costmap_sufficiency_gate': gate_summary,
        'full_window_distribution': full_window,
        'robot_footprint_nearby_distribution': footprint,
        'front_wedge_distribution': wedge,
        'target_footprint_distribution': target_cost['target_footprint_cost'],
        'target_footprint_front_wedge_local_cost': target_cost,
        'candidate_direction_corridor_slices': candidate_slices,
        'candidate_count': candidate_slices.get('candidate_count'),
        'open_direction_count': candidate_slices.get('open_direction_count'),
        'topology_candidate_summary': candidate_slices.get('topology_local_cost_result_counts'),
        'interpretation': _interpret_replay(classification, full_window, footprint, wedge, candidate_slices),
    }


def _interpret_replay(classification: str, full_window: dict[str, Any], footprint: dict[str, Any], wedge: dict[str, Any], candidate_slices: dict[str, Any]) -> str:
    free_ratio = full_window.get('free_ratio')
    min_free = full_window.get('min_free_ratio')
    direction_count = candidate_slices.get('direction_traversable_count')
    if classification == 'LOCAL_DIRECTION_TRAVERSABLE_GATE_SHOULD_REPLACE_GLOBAL_RATIO':
        return f'Full 1m local-costmap window free_ratio={free_ratio} is below threshold={min_free}, but at least {direction_count} candidate-direction corridor slice(s) remain locally traversable; this supports replacing the global ratio gate with robot-nearby + candidate-direction semantics in a later phase.'
    if classification == 'FULL_WINDOW_FREE_RATIO_TOO_STRICT':
        return f'Full 1m local-costmap window free_ratio={free_ratio} is below threshold={min_free}; available topology/candidate evidence suggests the denominator may be too broad for narrow corridors, but directional evidence is weaker or reverse-only.'
    if classification == 'LOCAL_COSTMAP_TRUE_BLOCKED':
        return f'Full-window failure is accompanied by robot footprint lethal_count={footprint.get("lethal_count")} and front wedge lethal_count={wedge.get("lethal_count")} with no traversable candidate slice, supporting true local blockage.'
    return 'Phase72 artifacts do not provide enough post-success local-costmap evidence to decide gate semantics.'


def _overall_classification(replays: list[dict[str, Any]]) -> str:
    labels = [r.get('classification') for r in replays]
    if 'LOCAL_DIRECTION_TRAVERSABLE_GATE_SHOULD_REPLACE_GLOBAL_RATIO' in labels:
        return 'LOCAL_DIRECTION_TRAVERSABLE_GATE_SHOULD_REPLACE_GLOBAL_RATIO'
    if 'FULL_WINDOW_FREE_RATIO_TOO_STRICT' in labels:
        return 'FULL_WINDOW_FREE_RATIO_TOO_STRICT'
    if labels and all(label == 'LOCAL_COSTMAP_TRUE_BLOCKED' for label in labels):
        return 'LOCAL_COSTMAP_TRUE_BLOCKED'
    if 'LOCAL_COSTMAP_TRUE_BLOCKED' in labels:
        return 'LOCAL_COSTMAP_TRUE_BLOCKED'
    return 'INSUFFICIENT_EVIDENCE'


def _metrics(replays: list[dict[str, Any]]) -> dict[str, Any]:
    blocked = [r for r in replays if r.get('post_success_local_costmap_sufficiency_gate', {}).get('blocked_by_local_costmap_sufficient')]
    direction_counts = [int(r.get('candidate_direction_corridor_slices', {}).get('direction_traversable_count') or 0) for r in replays]
    free_ratios = [_number(r.get('full_window_distribution', {}).get('free_ratio')) for r in replays]
    margins = [_number(r.get('full_window_distribution', {}).get('free_ratio_margin_to_threshold')) for r in replays]
    return {
        'replay_count': len(replays),
        'post_success_local_costmap_blocked_replay_count': len(blocked),
        'goal1_success_replay_count': sum(1 for r in replays if r.get('goal1_success_observed')),
        'direction_traversable_replay_count': sum(1 for count in direction_counts if count > 0),
        'max_direction_traversable_count': max(direction_counts) if direction_counts else 0,
        'full_window_free_ratio_values': free_ratios,
        'full_window_free_ratio_margin_values': margins,
        'min_full_window_free_ratio': min([v for v in free_ratios if v is not None], default=None),
        'max_full_window_free_ratio': max([v for v in free_ratios if v is not None], default=None),
    }


def analyze_phase73(artifact_dir: str | Path, output: str | Path | None = None) -> dict[str, Any]:
    artifact_dir = Path(artifact_dir)
    replay_dirs = sorted([p for p in artifact_dir.iterdir() if p.is_dir() and p.name.startswith('replay_')]) if artifact_dir.exists() else []
    replays = [_analyze_replay(replay_dir) for replay_dir in replay_dirs]
    classification = _overall_classification(replays)
    metrics = _metrics(replays)
    result = {
        'phase': PHASE,
        'run_id': RUN_ID,
        'source_phase72_run_id': PHASE72_RUN_ID,
        'artifact_dir': str(artifact_dir),
        'source_phase72_classification': PHASE72_CLASSIFICATION,
        'source_phase72_subclassification': PHASE72_SUBCLASSIFICATION,
        'classification': classification,
        'allowed_classifications': ALLOWED_CLASSIFICATIONS,
        'guardrails': GUARDRAILS,
        'guardrail_violation': False,
        'complete_autonomous_success_claimed': False,
        'exit_success_claimed': False,
        'metrics': metrics,
        'threshold_semantics': {
            'current_gate_semantics': 'local_costmap_sufficient requires full circular near-robot window known_ratio>=0.95 and free_ratio>=0.50 before candidate formation/dispatch',
            'review_question': 'Should the gate use robot-nearby footprint plus candidate-direction corridor slice traversability instead of a global full-window free_ratio?',
            'replace_global_ratio_with_directional_gate_candidate': classification == 'LOCAL_DIRECTION_TRAVERSABLE_GATE_SHOULD_REPLACE_GLOBAL_RATIO',
            'do_not_change_threshold_in_phase73': True,
        },
        'replays': replays,
        'recommendations': _recommendations(classification),
    }
    if output is not None:
        out = Path(output)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    return result


def _recommendations(classification: str) -> list[str]:
    if classification == 'LOCAL_DIRECTION_TRAVERSABLE_GATE_SHOULD_REPLACE_GLOBAL_RATIO':
        return [
            'Do not tune the 0.50 free_ratio threshold in Phase73.',
            'For a later accepted phase, consider a diagnostics-first candidate gate that keeps freshness/knownness checks but replaces global full-window free_ratio with robot-footprint-nearby and selected/eligible candidate-direction corridor-slice traversability.',
            'Keep branch scoring and centerline runtime behavior unchanged unless a later phase explicitly authorizes an intervention.',
        ]
    if classification == 'FULL_WINDOW_FREE_RATIO_TOO_STRICT':
        return [
            'Do not tune thresholds yet; collect or reconstruct raw local costmap bins if exact inflation/wall decomposition is needed.',
            'Prefer evaluating local directional traversability before changing global readiness thresholds.',
        ]
    if classification == 'LOCAL_COSTMAP_TRUE_BLOCKED':
        return [
            'Treat the readiness hold as plausible true local blockage from available artifacts.',
            'Do not tune Nav2/local inflation in Phase73; if accepted, gather visual/raw-costmap evidence before any intervention.',
        ]
    return ['Evidence is insufficient; preserve Phase72 conclusion and collect richer raw local-costmap samples in a later phase if needed.']


def main() -> int:
    parser = argparse.ArgumentParser(description=PHASE)
    parser.add_argument('--artifact-dir', type=Path, default=Path('log') / PHASE72_RUN_ID)
    parser.add_argument('--output', type=Path, default=None)
    args = parser.parse_args()
    result = analyze_phase73(args.artifact_dir, args.output)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':  # pragma: no cover
    raise SystemExit(main())
