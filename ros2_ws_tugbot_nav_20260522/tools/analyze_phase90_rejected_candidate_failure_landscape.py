#!/usr/bin/env python3
"""Phase90 rejected-candidate failure landscape diagnosis.

Read-only analyzer for Phase89 artifacts. It diagnoses why the Phase88
multi-candidate forward-search family produced rejected candidates for the
Goal2-equivalent segment. It does not change Phase88 refinement,
maze_explorer strategy, branch scoring, exploration order, centerline gate,
directional readiness, fallback/terminal acceptance, Nav2, MPPI, controller,
inflation, robot radius, clearance radius, or map thresholds.
"""
from __future__ import annotations

import argparse
import json
import math
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any

RUN_ID = 'phase90_rejected_candidate_failure_landscape_diagnosis'
PHASE89_RUN_ID = 'phase89_safety_first_refinement_bounded_goal2_validation'

ALLOWED_CLASSIFICATIONS = {
    'STAGING_ALIGNMENT_GOAL_NEEDED',
    'CANDIDATE_FAMILY_TOO_LOCAL',
    'FRONT_WEDGE_DOMINANT_BLOCKER',
    'FOOTPRINT_DOMINANT_BLOCKER',
    'SAFETY_FLOOR_DOMINANT_BLOCKER',
    'OFFSET_GRID_INSUFFICIENT',
    'FAILURE_LANDSCAPE_INSUFFICIENT_EVIDENCE',
}

FAILURE_FIELDS = {
    'safety_floor': 'safety_floor_ok',
    'footprint_lethal_regression': 'footprint_lethal_not_increased',
    'front_wedge_lethal_regression': 'front_wedge_lethal_not_increased',
    'clearance_insufficient': 'target_has_clearance',
    'occupancy': 'occupancy_free',
    'same_corridor': 'same_corridor',
    'two_side_wall': 'two_side_wall_evidence',
}

OFFSET_KEYS = ['lateral_offset_m', 'forward_offset_m', 'heading_offset_rad']

GUARDRAILS = {
    'diagnosis_only': True,
    'no_phase88_refinement_changed': True,
    'no_maze_explorer_strategy_changed': True,
    'no_branch_scoring_changed': True,
    'no_exploration_order_changed': True,
    'no_centerline_gate_changed': True,
    'no_directional_readiness_changed': True,
    'no_fallback_terminal_acceptance_changed': True,
    'no_nav2_mppi_controller_tuning': True,
    'no_inflation_robot_radius_clearance_radius_map_threshold_tuning': True,
}


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


def _read_jsonl(path: Path | None) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if not path or not path.exists() or not path.stat().st_size:
        return rows
    for line in path.read_text(encoding='utf-8', errors='replace').splitlines():
        if not line.strip():
            continue
        loaded = _safe_json_loads(line)
        if not isinstance(loaded, dict):
            continue
        payload = loaded.get('state') if isinstance(loaded.get('state'), dict) else loaded
        if isinstance(payload, dict):
            row = dict(payload)
            row.setdefault('_recorder_elapsed_sec', loaded.get('elapsed_sec'))
            row.setdefault('_recorder_wall_time', loaded.get('wall_time'))
            rows.append(row)
    return rows


def _glob_first(artifact_dir: Path, suffix: str) -> Path | None:
    direct = artifact_dir / f'{PHASE89_RUN_ID}_{suffix}'
    if direct.exists():
        return direct
    matches = sorted(artifact_dir.glob(f'*{suffix}'))
    return matches[0] if matches else None


def _as_float(value: Any) -> float | None:
    try:
        if value is None:
            return None
        f = float(value)
        if math.isnan(f):
            return None
        return f
    except (TypeError, ValueError):
        return None


def _fmt_value(value: Any) -> str:
    f = _as_float(value)
    if f is None:
        return 'missing'
    return f'{f:.12g}'


def _metric_range(rows: list[dict[str, Any]], key: str) -> dict[str, float | None]:
    vals = [_as_float(row.get(key)) for row in rows]
    vals = [v for v in vals if v is not None]
    return {'min': min(vals) if vals else None, 'max': max(vals) if vals else None}


def _bool_failure_count(candidates: list[dict[str, Any]], key: str) -> dict[str, int]:
    true_count = sum(1 for c in candidates if c.get(key) is True)
    false_count = sum(1 for c in candidates if c.get(key) is False)
    missing_count = sum(1 for c in candidates if key not in c or c.get(key) is None)
    return {
        'true_count': true_count,
        'false_count': false_count,
        'missing_count': missing_count,
    }


def _forward_progress_counts(candidates: list[dict[str, Any]]) -> dict[str, int]:
    ok = 0
    failed = 0
    missing = 0
    for c in candidates:
        values = [c.get('forward_progress_ok'), c.get('forward_progress_not_lowered')]
        present = [v for v in values if v is not None]
        if not present:
            missing += 1
        elif all(v is True for v in present):
            ok += 1
        elif any(v is False for v in present):
            failed += 1
        else:
            missing += 1
    return {'true_count': ok, 'false_count': failed, 'missing_count': missing}


def _failure_reason_counts(candidates: list[dict[str, Any]]) -> dict[str, dict[str, int]]:
    out = {name: _bool_failure_count(candidates, field) for name, field in FAILURE_FIELDS.items()}
    out['forward_progress'] = _forward_progress_counts(candidates)
    return out


def _candidate_failed(candidate: dict[str, Any], reason: str) -> bool:
    if reason == 'forward_progress':
        values = [candidate.get('forward_progress_ok'), candidate.get('forward_progress_not_lowered')]
        return any(v is False for v in values if v is not None)
    field = FAILURE_FIELDS[reason]
    return candidate.get(field) is False


def _offset_distributions(candidates: list[dict[str, Any]]) -> dict[str, dict[str, Any]]:
    out: dict[str, dict[str, Any]] = {}
    for key in OFFSET_KEYS:
        buckets: dict[str, Any] = {}
        grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
        for c in candidates:
            grouped[_fmt_value(c.get(key))].append(c)
        for value, rows in sorted(grouped.items(), key=lambda item: item[0]):
            buckets[value] = {
                'candidate_count': len(rows),
                'hard_safety_pass_count': sum(1 for r in rows if r.get('hard_safety_pass') is True),
                'safety_floor_failed_count': sum(1 for r in rows if _candidate_failed(r, 'safety_floor')),
                'footprint_lethal_regression_count': sum(1 for r in rows if _candidate_failed(r, 'footprint_lethal_regression')),
                'front_wedge_lethal_regression_count': sum(1 for r in rows if _candidate_failed(r, 'front_wedge_lethal_regression')),
                'clearance_insufficient_count': sum(1 for r in rows if _candidate_failed(r, 'clearance_insufficient')),
                'forward_progress_failed_count': sum(1 for r in rows if _candidate_failed(r, 'forward_progress')),
                'occupancy_failed_count': sum(1 for r in rows if _candidate_failed(r, 'occupancy')),
                'same_corridor_failed_count': sum(1 for r in rows if _candidate_failed(r, 'same_corridor')),
                'two_side_wall_failed_count': sum(1 for r in rows if _candidate_failed(r, 'two_side_wall')),
                'min_clearance_m': _metric_range(rows, 'min_clearance_m'),
                'front_wedge_lethal_count': _metric_range(rows, 'front_wedge_lethal_count'),
                'footprint_lethal_count': _metric_range(rows, 'footprint_lethal_count'),
                'local_cost_max_radius': _metric_range(rows, 'local_cost_max_radius'),
            }
        out[key] = buckets
    return out


def _wall_side_concentration(candidates: list[dict[str, Any]]) -> dict[str, Any]:
    left_nearer = 0
    right_nearer = 0
    equal = 0
    missing = 0
    left_fail_reasons: Counter[str] = Counter()
    right_fail_reasons: Counter[str] = Counter()
    for c in candidates:
        left = _as_float(c.get('left_wall_clearance_m'))
        right = _as_float(c.get('right_wall_clearance_m'))
        if left is None or right is None:
            missing += 1
            continue
        target_counter = None
        if left < right:
            left_nearer += 1
            target_counter = left_fail_reasons
        elif right < left:
            right_nearer += 1
            target_counter = right_fail_reasons
        else:
            equal += 1
        if target_counter is not None:
            for reason in FAILURE_FIELDS:
                if _candidate_failed(c, reason):
                    target_counter[reason] += 1
            if _candidate_failed(c, 'forward_progress'):
                target_counter['forward_progress'] += 1
    total_known = left_nearer + right_nearer + equal
    dominant_side = None
    if total_known:
        if left_nearer / total_known >= 0.6:
            dominant_side = 'left_wall_nearer'
        elif right_nearer / total_known >= 0.6:
            dominant_side = 'right_wall_nearer'
        elif equal / total_known >= 0.6:
            dominant_side = 'balanced_or_centered'
    return {
        'left_wall_nearer_count': left_nearer,
        'right_wall_nearer_count': right_nearer,
        'equal_wall_clearance_count': equal,
        'missing_wall_clearance_count': missing,
        'dominant_side': dominant_side,
        'left_wall_nearer_failure_reasons': dict(left_fail_reasons),
        'right_wall_nearer_failure_reasons': dict(right_fail_reasons),
    }


def _offset_failure_concentration(candidates: list[dict[str, Any]]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for key in OFFSET_KEYS:
        grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
        for c in candidates:
            grouped[_fmt_value(c.get(key))].append(c)
        bucket_failure_counts = {
            value: sum(1 for row in rows if row.get('hard_safety_pass') is not True)
            for value, rows in grouped.items()
        }
        total_failed = sum(bucket_failure_counts.values())
        dominant_value = None
        dominant_fraction = None
        if total_failed:
            dominant_value, dominant_count = max(bucket_failure_counts.items(), key=lambda item: item[1])
            dominant_fraction = dominant_count / total_failed
            if dominant_fraction < 0.5:
                dominant_value = None
        out[key] = {
            'candidate_bucket_count': len(grouped),
            'failed_candidate_count_by_value': bucket_failure_counts,
            'dominant_failed_value': dominant_value,
            'dominant_failed_fraction': dominant_fraction,
        }
    return out


def _local_sample_summary(rows: list[dict[str, Any]], goal_sequence: int | None) -> dict[str, Any]:
    if goal_sequence is not None:
        rows = [r for r in rows if r.get('goal_sequence') == goal_sequence]

    def values(path: list[str]) -> list[float]:
        out: list[float] = []
        for row in rows:
            cur: Any = row
            for part in path:
                cur = cur.get(part) if isinstance(cur, dict) else None
            val = _as_float(cur)
            if val is not None:
                out.append(val)
        return out

    def summarize(path: list[str]) -> dict[str, float | None]:
        vals = values(path)
        return {'min': min(vals) if vals else None, 'max': max(vals) if vals else None, 'last': vals[-1] if vals else None}

    poses = [row.get('robot_pose') for row in rows if isinstance(row.get('robot_pose'), list)]
    return {
        'sample_count': len(rows),
        'front_wedge_lethal_count': summarize(['front_wedge_cost', 'lethal_count']),
        'robot_footprint_lethal_count': summarize(['robot_footprint_cost', 'lethal_count']),
        'target_footprint_lethal_count': summarize(['target_footprint_cost', 'lethal_count']),
        'front_wedge_cost_max': summarize(['front_wedge_cost', 'max']),
        'target_radius_cost_max': summarize(['local_costmap_target_evidence', 'radius_cost_summary', 'max']),
        'last_robot_pose': poses[-1] if poses else None,
    }


def _raw_capture_summary(raw: dict[str, Any]) -> dict[str, bool]:
    raw = raw if isinstance(raw, dict) else {}
    return {
        'scan_available': isinstance(raw.get('scan'), dict),
        'local_costmap_available': isinstance(raw.get('local_costmap'), dict),
        'footprint_available': isinstance(raw.get('footprint'), dict),
        'odom_available': isinstance(raw.get('odom'), dict),
        'tf_available': isinstance(raw.get('tf'), dict) and bool(raw.get('tf')),
    }


def _multi_candidate_present(value: Any) -> bool:
    if value is True:
        return True
    if isinstance(value, dict):
        if value.get('enabled') is True:
            return True
        if value.get('candidate_count') or value.get('candidates'):
            return True
        if isinstance(value.get('candidate_family'), dict):
            return True
    return False


def _least_risky_candidates(candidates: list[dict[str, Any]], limit: int = 10) -> list[dict[str, Any]]:
    def risk(c: dict[str, Any]) -> tuple[float, float, float, float, int]:
        front = _as_float(c.get('front_wedge_lethal_count')) or 0.0
        footprint = _as_float(c.get('footprint_lethal_count')) or 0.0
        local = _as_float(c.get('local_cost_max_radius')) or 999.0
        clearance = _as_float(c.get('min_clearance_m')) or 0.0
        index = int(c.get('candidate_index') or 0)
        return (front + footprint, local, -clearance, abs(_as_float(c.get('heading_offset_rad')) or 0.0), index)
    rows = []
    for c in sorted(candidates, key=risk)[:limit]:
        rows.append({
            'candidate_index': c.get('candidate_index'),
            'lateral_offset_m': c.get('lateral_offset_m'),
            'forward_offset_m': c.get('forward_offset_m'),
            'heading_offset_rad': c.get('heading_offset_rad'),
            'target': c.get('target') or c.get('target_xy'),
            'safety_floor_ok': c.get('safety_floor_ok'),
            'footprint_lethal_not_increased': c.get('footprint_lethal_not_increased'),
            'front_wedge_lethal_not_increased': c.get('front_wedge_lethal_not_increased'),
            'target_has_clearance': c.get('target_has_clearance'),
            'min_clearance_m': c.get('min_clearance_m'),
            'front_wedge_lethal_count': c.get('front_wedge_lethal_count'),
            'footprint_lethal_count': c.get('footprint_lethal_count'),
            'left_wall_clearance_m': c.get('left_wall_clearance_m'),
            'right_wall_clearance_m': c.get('right_wall_clearance_m'),
        })
    return rows


def _best_numeric(candidates: list[dict[str, Any]], key: str, fn) -> float | None:
    values = [_as_float(c.get(key)) for c in candidates]
    values = [v for v in values if v is not None]
    return fn(values) if values else None


def _classification(
    *,
    evidence_gaps: list[str],
    candidate_count: int,
    hard_safety_pass_count: int,
    failure_counts: dict[str, dict[str, int]],
    candidates: list[dict[str, Any]],
    local_summary: dict[str, Any],
) -> tuple[str, list[str], list[str]]:
    if evidence_gaps:
        return 'FAILURE_LANDSCAPE_INSUFFICIENT_EVIDENCE', ['missing_or_incomplete_phase89_candidate_landscape'], []
    if candidate_count <= 0:
        return 'FAILURE_LANDSCAPE_INSUFFICIENT_EVIDENCE', ['no_candidate_diagnostics_present'], []

    secondary: list[str] = []
    all_unexec = hard_safety_pass_count == 0
    safety_failed = failure_counts['safety_floor']['false_count']
    front_failed = failure_counts['front_wedge_lethal_regression']['false_count']
    footprint_failed = failure_counts['footprint_lethal_regression']['false_count']
    clearance_failed = failure_counts['clearance_insufficient']['false_count']
    min_clearance_max = _best_numeric(candidates, 'min_clearance_m', max)
    floor_values = [_as_float(c.get('safety_min_clearance_floor_m')) for c in candidates]
    floor_values = [v for v in floor_values if v is not None]
    safety_floor = max(floor_values) if floor_values else None
    zero_lethal_but_safety_failed = [
        c for c in candidates
        if c.get('safety_floor_ok') is False
        and c.get('footprint_lethal_not_increased') is True
        and c.get('front_wedge_lethal_not_increased') is True
    ]
    runtime_front_lethal_max = ((local_summary.get('front_wedge_lethal_count') or {}).get('max'))
    runtime_footprint_lethal_max = ((local_summary.get('robot_footprint_lethal_count') or {}).get('max'))
    runtime_execution_lethal = (_as_float(runtime_front_lethal_max) or 0.0) > 0.0 or (_as_float(runtime_footprint_lethal_max) or 0.0) > 0.0

    if all_unexec:
        if candidate_count >= 20 and safety_failed == candidate_count and zero_lethal_but_safety_failed and runtime_execution_lethal:
            secondary.extend(['SAFETY_FLOOR_DOMINANT_BLOCKER', 'CANDIDATE_FAMILY_TOO_LOCAL'])
            return 'STAGING_ALIGNMENT_GOAL_NEEDED', [
                'all_candidates_failed_hard_safety',
                'all_candidates_failed_safety_floor',
                'some_zero_lethal_candidates_still_failed_clearance_floor',
                'execution_time_front_wedge_or_footprint_lethal_present',
            ], secondary
        if safety_failed >= math.ceil(candidate_count * 0.8):
            reasons = ['safety_floor_failed_for_most_or_all_candidates']
            if min_clearance_max is not None and safety_floor is not None and min_clearance_max < safety_floor:
                reasons.append('candidate_family_max_clearance_below_safety_floor')
            return 'SAFETY_FLOOR_DOMINANT_BLOCKER', reasons, secondary
        if front_failed >= math.ceil(candidate_count * 0.6) and front_failed >= footprint_failed:
            return 'FRONT_WEDGE_DOMINANT_BLOCKER', ['front_wedge_lethal_regression_dominates_rejected_candidates'], secondary
        if footprint_failed >= math.ceil(candidate_count * 0.6) and footprint_failed > front_failed:
            return 'FOOTPRINT_DOMINANT_BLOCKER', ['footprint_lethal_regression_dominates_rejected_candidates'], secondary
        if clearance_failed >= math.ceil(candidate_count * 0.5):
            return 'CANDIDATE_FAMILY_TOO_LOCAL', ['candidate_family_clearance_failures_dominate_local_grid'], secondary
        return 'CANDIDATE_FAMILY_TOO_LOCAL', ['all_local_candidate_family_members_rejected_without_single_dominant_subreason'], secondary

    if front_failed >= math.ceil(candidate_count * 0.7) and front_failed >= footprint_failed:
        return 'FRONT_WEDGE_DOMINANT_BLOCKER', ['front_wedge_lethal_regression_dominates_rejected_candidates'], secondary
    if footprint_failed >= math.ceil(candidate_count * 0.7) and footprint_failed > front_failed:
        return 'FOOTPRINT_DOMINANT_BLOCKER', ['footprint_lethal_regression_dominates_rejected_candidates'], secondary

    # When failures are concentrated at outermost offsets and at least one non-edge candidate
    # survives hard safety, the existing grid is likely too sparse/narrow for diagnosis.
    offset_edge_signals = []
    for key in OFFSET_KEYS:
        vals = sorted(v for v in {_as_float(c.get(key)) for c in candidates} if v is not None)
        if len(vals) >= 3:
            edge_vals = {vals[0], vals[-1]}
            failed_at_edge = sum(1 for c in candidates if (c.get('hard_safety_pass') is not True and _as_float(c.get(key)) in edge_vals))
            failed_total = sum(1 for c in candidates if c.get('hard_safety_pass') is not True)
            if failed_total and failed_at_edge / failed_total >= 0.75:
                offset_edge_signals.append(key)
    if offset_edge_signals:
        return 'OFFSET_GRID_INSUFFICIENT', [f'failures_concentrated_at_grid_edges:{",".join(offset_edge_signals)}'], secondary

    return 'FAILURE_LANDSCAPE_INSUFFICIENT_EVIDENCE', ['candidate_landscape_present_but_no_conservative_dominant_classification'], secondary


def analyze_phase89_artifacts(artifact_dir: Path | str) -> dict[str, Any]:
    artifact_dir = Path(artifact_dir)
    analysis_path = _glob_first(artifact_dir, 'analysis.json')
    goal_events_path = _glob_first(artifact_dir, 'goal_events.jsonl')
    local_samples_path = _glob_first(artifact_dir, 'local_costmap_samples.jsonl')
    raw_capture_path = _glob_first(artifact_dir, 'raw_capture.json')

    analysis = _safe_json_load(analysis_path, {})
    goal_events = _read_jsonl(goal_events_path)
    local_samples = _read_jsonl(local_samples_path)
    raw_capture = _safe_json_load(raw_capture_path, {})

    evidence_gaps: list[str] = []
    if not analysis_path or not analysis:
        evidence_gaps.append('missing_phase89_analysis_json')
    if not goal_events_path or not goal_events:
        evidence_gaps.append('missing_phase89_goal_events_jsonl')
    if not local_samples_path or not local_samples:
        evidence_gaps.append('missing_phase89_local_costmap_samples_jsonl')
    if not raw_capture_path or not raw_capture:
        evidence_gaps.append('missing_phase89_raw_capture_json')

    context = analysis.get('goal2_dispatch_context') if isinstance(analysis, dict) else {}
    context = context if isinstance(context, dict) else {}
    nested = context.get('centerline_target_refinement')
    nested = nested if isinstance(nested, dict) else {}
    raw_candidates = nested.get('candidates')
    candidates = [c for c in raw_candidates if isinstance(c, dict)] if isinstance(raw_candidates, list) else []
    validation_goal_sequence = analysis.get('validation_goal_sequence') if isinstance(analysis, dict) else None

    if analysis and not context:
        evidence_gaps.append('missing_goal2_dispatch_context')
    if context and not nested:
        evidence_gaps.append('missing_centerline_target_refinement_payload')
    if context and not _multi_candidate_present(context.get('multi_candidate_forward_search')) and not _multi_candidate_present(nested.get('multi_candidate_forward_search')):
        evidence_gaps.append('multi_candidate_forward_search_not_present')
    if context and context.get('refinement_applied') is not False and nested.get('refinement_applied') is not False:
        evidence_gaps.append('source_refinement_was_not_rejected')
    if context and (context.get('refinement_reject_reason') or nested.get('refinement_reject_reason')) != 'lethal_cost_regression':
        evidence_gaps.append('source_reject_reason_not_lethal_cost_regression')
    if nested and not candidates:
        evidence_gaps.append('missing_rejected_candidate_diagnostics')

    candidate_count = len(candidates)
    hard_safety_pass_count = sum(1 for c in candidates if c.get('hard_safety_pass') is True)
    failure_counts = _failure_reason_counts(candidates)
    local_summary = _local_sample_summary(local_samples, validation_goal_sequence if isinstance(validation_goal_sequence, int) else None)
    raw_summary = _raw_capture_summary(raw_capture if isinstance(raw_capture, dict) else {})

    classification, classification_reasons, secondary = _classification(
        evidence_gaps=evidence_gaps,
        candidate_count=candidate_count,
        hard_safety_pass_count=hard_safety_pass_count,
        failure_counts=failure_counts,
        candidates=candidates,
        local_summary=local_summary,
    )

    min_clearance_max = _best_numeric(candidates, 'min_clearance_m', max)
    min_clearance_min = _best_numeric(candidates, 'min_clearance_m', min)
    safety_floor_failed = failure_counts['safety_floor']['false_count']
    zero_lethal_candidates = [
        c for c in candidates
        if c.get('footprint_lethal_not_increased') is True and c.get('front_wedge_lethal_not_increased') is True
    ]
    all_local_unexec = candidate_count > 0 and hard_safety_pass_count == 0
    staging_supported = (
        all_local_unexec
        and safety_floor_failed == candidate_count
        and bool(zero_lethal_candidates)
        and (((local_summary.get('front_wedge_lethal_count') or {}).get('max') or 0) > 0
             or ((local_summary.get('robot_footprint_lethal_count') or {}).get('max') or 0) > 0)
    )

    result = {
        'run_id': RUN_ID,
        'source_phase89_artifact_dir': str(artifact_dir),
        'classification': classification,
        'allowed_classifications': sorted(ALLOWED_CLASSIFICATIONS),
        'classification_reasons': classification_reasons,
        'secondary_supported_classifications': secondary,
        'evidence_gaps': evidence_gaps,
        'source_phase89_context': {
            'classification': analysis.get('classification') if isinstance(analysis, dict) else None,
            'validation_goal_sequence': validation_goal_sequence,
            'used_target_match_fallback': analysis.get('used_target_match_fallback') if isinstance(analysis, dict) else None,
            'candidate_count': context.get('candidate_count') or nested.get('candidate_count') or candidate_count,
            'hard_safety_pass_candidate_count': context.get('hard_safety_pass_candidate_count') if context.get('hard_safety_pass_candidate_count') is not None else nested.get('hard_safety_pass_candidate_count', hard_safety_pass_count),
            'multi_candidate_forward_search': context.get('multi_candidate_forward_search') if context.get('multi_candidate_forward_search') is not None else nested.get('multi_candidate_forward_search'),
            'candidate_family': context.get('candidate_family') or nested.get('candidate_family'),
            'refinement_applied': context.get('refinement_applied') if context.get('refinement_applied') is not None else nested.get('refinement_applied'),
            'refinement_reject_reason': context.get('refinement_reject_reason') or nested.get('refinement_reject_reason'),
            'original_target_preserved_on_reject': context.get('original_target_preserved_on_reject') if context.get('original_target_preserved_on_reject') is not None else nested.get('original_target_preserved_on_reject'),
            'branch_scoring_changed': context.get('branch_scoring_changed') if context.get('branch_scoring_changed') is not None else nested.get('branch_scoring_changed'),
            'fallback_terminal_acceptance_used': context.get('fallback_terminal_acceptance_used') if context.get('fallback_terminal_acceptance_used') is not None else nested.get('fallback_terminal_acceptance_used'),
            'original_target': context.get('original_target') or nested.get('original_target'),
            'nav2_goal_target': context.get('target'),
            'selection_priority_trace': context.get('selection_priority_trace') or nested.get('selection_priority_trace'),
            'rejected_candidate_summary_count': len(context.get('rejected_candidate_summaries') or nested.get('rejected_candidate_summaries') or []),
        },
        'candidate_failure_landscape': {
            'candidate_count': candidate_count,
            'hard_safety_pass_candidate_count': hard_safety_pass_count,
            'all_local_candidate_families_unexecutable': all_local_unexec,
            'failure_reason_counts': failure_counts,
            'offset_distributions': _offset_distributions(candidates),
            'offset_failure_concentration': _offset_failure_concentration(candidates),
            'wall_side_concentration': _wall_side_concentration(candidates),
            'metric_ranges': {
                'lateral_offset_m': _metric_range(candidates, 'lateral_offset_m'),
                'forward_offset_m': _metric_range(candidates, 'forward_offset_m'),
                'heading_offset_rad': _metric_range(candidates, 'heading_offset_rad'),
                'min_clearance_m': {'min': min_clearance_min, 'max': min_clearance_max},
                'local_cost_max_radius': _metric_range(candidates, 'local_cost_max_radius'),
                'front_wedge_lethal_count': _metric_range(candidates, 'front_wedge_lethal_count'),
                'footprint_lethal_count': _metric_range(candidates, 'footprint_lethal_count'),
                'front_wedge_high_cost_count': _metric_range(candidates, 'front_wedge_high_cost_count'),
                'footprint_high_cost_count': _metric_range(candidates, 'footprint_high_cost_count'),
            },
            'least_risky_rejected_candidates': _least_risky_candidates(candidates),
        },
        'two_step_intermediate_goal_assessment': {
            'staging_alignment_goal_supported': staging_supported,
            'assessment': 'two_step_corridor_alignment_staging_supported_before_forward_exploration_goal' if staging_supported else 'not_proven_from_phase89_candidate_landscape_alone',
            'supporting_reasons': [
                'no_hard_safety_pass_candidate_in_local_forward_search' if all_local_unexec else None,
                'zero_lethal_candidate_subset_still_fails_safety_floor' if zero_lethal_candidates and safety_floor_failed else None,
                'execution_time_front_wedge_or_footprint_lethal_present' if (((local_summary.get('front_wedge_lethal_count') or {}).get('max') or 0) > 0 or ((local_summary.get('robot_footprint_lethal_count') or {}).get('max') or 0) > 0) else None,
            ],
            'guardrail': 'assessment_only_no_strategy_change',
        },
        'local_costmap_sample_summary': local_summary,
        'raw_capture_summary': raw_summary,
        'source_files': {
            'phase89_analysis': str(analysis_path) if analysis_path else None,
            'goal_events': str(goal_events_path) if goal_events_path else None,
            'local_costmap_samples': str(local_samples_path) if local_samples_path else None,
            'raw_capture': str(raw_capture_path) if raw_capture_path else None,
        },
        'guardrails': dict(GUARDRAILS),
        'autonomous_exploration_success_claimed': False,
        'exit_success_claimed': False,
        'phase91_entered': False,
    }
    result['two_step_intermediate_goal_assessment']['supporting_reasons'] = [
        r for r in result['two_step_intermediate_goal_assessment']['supporting_reasons'] if r
    ]
    return result


def write_minimal_summary(result: dict[str, Any], path: Path) -> None:
    ctx = result.get('source_phase89_context') or {}
    land = result.get('candidate_failure_landscape') or {}
    counts = land.get('failure_reason_counts') or {}
    two_step = result.get('two_step_intermediate_goal_assessment') or {}
    lines = [
        '# Phase90 minimal summary',
        '',
        f"Classification: {result.get('classification')}",
        f"Evidence gaps: {result.get('evidence_gaps')}",
        f"Source Phase89 classification: {ctx.get('classification')}",
        f"Candidate count: {ctx.get('candidate_count')}",
        f"Hard safety pass candidate count: {ctx.get('hard_safety_pass_candidate_count')}",
        f"Refinement applied: {ctx.get('refinement_applied')}",
        f"Reject reason: {ctx.get('refinement_reject_reason')}",
        '',
        'Failure reason false counts:',
    ]
    for key in ['safety_floor', 'footprint_lethal_regression', 'front_wedge_lethal_regression', 'clearance_insufficient', 'forward_progress', 'occupancy', 'same_corridor', 'two_side_wall']:
        item = counts.get(key) or {}
        lines.append(f"- {key}: false={item.get('false_count')} true={item.get('true_count')} missing={item.get('missing_count')}")
    lines.extend([
        '',
        f"All local candidate families unexecutable: {land.get('all_local_candidate_families_unexecutable')}",
        f"Two-step staging supported: {two_step.get('staging_alignment_goal_supported')}",
        f"Two-step assessment: {two_step.get('assessment')}",
        f"Classification reasons: {result.get('classification_reasons')}",
        f"Secondary supported classifications: {result.get('secondary_supported_classifications')}",
        '',
        'Guardrails: No Phase88 refinement changed; No maze_explorer strategy changed; No branch scoring changed; No exploration order changed; No centerline gate changed; No directional readiness changed; No fallback/terminal acceptance changed; No Nav2/MPPI/controller tuning; No inflation/robot_radius/clearance_radius_m/map threshold tuning; No autonomous exploration success claimed; No exit success claimed; Phase91 not entered.',
    ])
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('\n'.join(lines) + '\n', encoding='utf-8')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--phase89-artifact-dir', type=Path, default=Path('log') / PHASE89_RUN_ID)
    parser.add_argument('--output-json', type=Path, default=Path('log') / RUN_ID / f'{RUN_ID}.json')
    parser.add_argument('--minimal-summary', type=Path, default=Path('log') / RUN_ID / f'{RUN_ID}_minimal_summary.md')
    args = parser.parse_args()

    result = analyze_phase89_artifacts(args.phase89_artifact_dir)
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    write_minimal_summary(result, args.minimal_summary)
    print(json.dumps({
        'classification': result['classification'],
        'evidence_gaps': result['evidence_gaps'],
        'candidate_count': result['source_phase89_context']['candidate_count'],
        'hard_safety_pass_candidate_count': result['source_phase89_context']['hard_safety_pass_candidate_count'],
        'output_json': str(args.output_json),
        'minimal_summary': str(args.minimal_summary),
    }, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
